"""B 档 seam 3 四条判据的离线单测（ticket 11）—— 不启 SITL、不启夹具、不读包。

**为什么必须单开一个文件**：一场 B 档要 3–5 分钟 SITL，而这一档是 **Phase 2 最高的闸**
（票面：上面没有退路）。判据层里一个笔误的代价不是"多跑一次"，而是**把闸的结论写反** ——
A 档已经栽过两次同类：判据⑤ 在一场爬升里照样过、判据⑧ 把"没饱和"判成 ❌
（见 `test_tier_a_judges.py` 的 docstring）。这个文件把人造 obs 喂进判据层，
让"该炸的时候炸、该过的时候过"变成一毫秒的事。

★ 本文件盯住的是 B 档**特有**的几件事（通用的那套在 `test_verify_judges.py` 与
  `test_tier_a_judges.py` 里，不重复）：

  1. **NED/ENU 的取反**（三处：起飞方向、真值高度、离地段最低点）—— 静默取反的症状是
     报告指向控制器，而真身是判据自己；
  2. **稳定窗口止于最后一次正常帧** —— 否则判据④ 那场（注入后全程兜底）会把
     "兜底保不住高度"记成"判据① 悬停精度不行"，两件事混成一条 ❌；
  3. **判据② 是两条轨迹、一场只判其中一条** —— 另一条当场记「本场不做」（`ok=None`、
     **不计数**）；而跟踪误差**只留档、不设界**（spec 只说"不撞地、不失控"，
     操作者 2026-09-30 明令**不编阈值**）⟹ 有一条**反向**断言钉着"误差再大也不改判"；
  4. **"没做" / "判不了" / "不通过" 的三分法** —— 前两者是 `ok=None` + `counts=False`，
     只有第三者才计数；而"仪器没覆盖到窗口"另有一条**计数**的前置行兜底；
  5. **判据④ 的 A/B/C 三层分开** —— 阶梯走没走（结构证据：悬停帧的推力恒等于
     −hover_thrust）、高度保没保住、这一场该记在谁头上，是**三个**读数；
     合成到一个 ❌ 里就会写出"阶梯坏了"这种归因错误的结论；
  6. **两根钟不许混用**（ticket 10 缺陷 A/B）—— `obs["t"]["offboard_enter_t"]` /
     `reference_timeline` 的秒数是**记录器钟**，`frames[].wall_ns` / 真值第 7 列是
     **绝对墙钟**；混用的症状是"窗口交集恒空"⇒ 一句"仪器没覆盖到"把判层的错记到飞机头上，
     而且**每一场读数都一模一样**。判"参考在动"同理：按**速度**判，不按位置。

（要 `rclpy` / `px4_msgs` / `nmpc`：被测模块导入期就 import 它们。没有环境直接 skip。）
"""

from __future__ import annotations

import importlib.util
import json
import math
import re
import sys
from pathlib import Path

import pytest

pytest.importorskip("rclpy")
pytest.importorskip("px4_msgs")
pytest.importorskip("nmpc")

HERE = Path(__file__).resolve().parent

#: 墙钟原点。取一个**真实量级**的 epoch（1.7e9 s）：判据层按绝对墙钟配对，
#: 这里若图省事从 0 开头，两列时基混了也照样"能跑"，测出来的就不是真东西。
T_ABS0 = 1_700_000_000.0
FRAME_HZ = 100.0        #: DUT 逐帧日志
TRUTH_HZ = 250.0        #: 真值流（SITL 全速）
DUR_S = 20.0


def _load():
    """按路径导入被测脚本（`tools/` 不是一个包）。

    ★ `sys.path` 要补 `HERE`：被测脚本里有一句 `import verify_offboard_smoke as v`
      与 `import verify_tier_a_hover as ta`（同目录复用世界/记录器/判据尺），
      而 pytest 的 rootdir 不在这一层。
    ★ 执行失败要把半成品从 `sys.modules` 里摘掉（否则下一次拿到的是空模块，
      报出来的是 `has no attribute …` 这种把人往错方向带的假象 —— 见
      `test_mocap_model.py::_load` 的 ★）。
    """
    if str(HERE) not in sys.path:
        sys.path.insert(0, str(HERE))
    name = "verify_tier_b_seam3"
    if name in sys.modules:
        return sys.modules[name]
    spec = importlib.util.spec_from_file_location(name, HERE / f"{name}.py")
    mod = importlib.util.module_from_spec(spec)
    sys.modules[name] = mod
    try:
        spec.loader.exec_module(mod)
    except BaseException:
        sys.modules.pop(name, None)
        raise
    return mod


b = _load()
ta = b.ta


# ────────────────────────────────────────────── 造观测


def wall(i, hz=FRAME_HZ):
    """第 `i` 帧的绝对墙钟 [s]（整数纳秒 → 秒，避免浮点漂移）。"""
    return (int(T_ABS0 * 1e9) + int(round(i * 1e9 / hz))) / 1e9


def frame(i, *, fallback="normal", ref=(0.0, 0.0, 1.0), thrust=None, note="",
          nav_state=None):
    """一"帧"DUT 逐帧日志（只造判据层真正读的那几列）。

    ★ `nav_state` 默认取 `OFFBOARD`（14）而不是 `None`：判据② 的"不失控"就判在它上面，
      默认留空的话**每一场人造场次都会被判成失控** —— 那会把"夹具少造了一列"伪装成
      "飞机失控"，正是本文件开头说的那类假象。真实帧里这一列恒在（`FRAME_KEYS`）。
    """
    return {"wall_ns": int(T_ABS0 * 1e9) + int(round(i * 1e9 / FRAME_HZ)),
            "tick": i, "fallback": fallback, "fallback_note": note,
            "x_ref0": list(ref), "thrust_body_z": thrust, "sat": [],
            "ref_anchor": "anchor" if i == 0 else "", "age_state_s": 0.01,
            "state_source": "mocap",
            "nav_state": b.v.NAV_STATE_OFFBOARD if nav_state is None else nav_state}


def rise_traj(t):
    """默认真值轨迹：1 s 内从地面升到 1 m，然后悬停。"""
    return (0.0, 0.0, min(1.0, t))


def truth_(dur_s=DUR_S, fn=None, hz=TRUTH_HZ, t_abs0=T_ABS0, n=None):
    """真值行 `[t_rel, x, y, z, vx, vy, vz, wall_s]`（**ENU**），与 `TruthTap.enu_rows` 同形。

    ★ 第 0 列是流自己的相对时基、第 7 列是绝对墙钟 —— **两列都要对**，
      因为它们各自被不同的判据读（分段用第 0 列、配对与窗口用第 7 列）。
    """
    n = int(dur_s * hz) if n is None else n
    rows = []
    for i in range(n):
        t_rel = i / hz
        x, y, z = rise_traj(t_rel) if fn is None else fn(t_rel)
        rows.append([t_rel, x, y, z, 0.0, 0.0, 0.0, t_abs0 + t_rel])
    return rows


def make_obs(*, frames=None, truth=None, extra_delay_ms=0.0, inject=0,
             mismatch="none", model_ht=0.1589, plant_param="z0.param",
             ref_span_z=0.0):
    """一份最小但**判据层跑得动**的 obs（形状与 `run_flight_b` 的输出一致）。"""
    frames = [frame(i) for i in range(int(DUR_S * FRAME_HZ))] if frames is None else frames
    truth = truth_() if truth is None else truth
    return {
        "name": "synthetic", "mismatch": mismatch, "plant_param": plant_param,
        "model_hover_thrust": model_ht,
        "plant_hover": b.plant_hover_thrust(plant_param),
        "extra_delay_ms": extra_delay_ms,
        "inject_solve_fail_after_frames": inject,
        "replay_up": True, "truth_n": len(truth), "truth_hz": TRUTH_HZ,
        "truth_att_n": 200, "truth_pos_n": len(truth),
        "tilt_deg_max": 1.5, "tilt_deg_max_steady": 0.5,
        "frames": frames, "truth": truth,
        "dut": {"frames": len(frames), "actual_hz": FRAME_HZ, "state_source": "mocap",
                "ages_ok": True, "sat_logged": True,
                "ref_events": ["anchor"], "track": {"ref_span_z": ref_span_z,
                                                    "ref_span_xy": 0.0}},
        "solve": {"n": 100, "p50": 3.0, "p99": 5.0, "max": 11.3, "warmup_n": 5,
                  "steady_n": 95, "steady_p50": 3.0, "steady_p99": 5.0,
                  "steady_max": 6.0, "cold_ms": 11.3, "cold_in_warmup": True,
                  "tick_p50_ms": 10.1, "tick_p99_ms": 11.0, "tick_max_ms": 12.0},
        "armed_ever": True, "rate_hz": FRAME_HZ,
        "t": {"offboard_enter": 3.0, "replay_frames_at_dut_start": 100},
        "replay": {"frames": int(DUR_S * FRAME_HZ), "base_delay_ms": 12.0,
                   "delay_ms": 12.0 + extra_delay_ms},
        "agent": "解锁命令 1 条、模式命令 1 条",
    }


def picked(checks, *prefixes):
    """按**前缀**挑判据（判据名很长，全等断言会把每条改名都变成一次测试失败）。"""
    return [c for c in checks if any(c["name"].startswith(p) for p in prefixes)]


def one(checks, prefix):
    got = picked(checks, prefix)
    assert len(got) == 1, f"{prefix!r} 该命中 1 条，实得 {[c['name'] for c in got]}"
    return got[0]


def counted(checks):
    return [c for c in checks if c.get("counts", True)]


def failed(checks):
    return [c["name"] for c in counted(checks) if c["ok"] is False]


# ────────────────────────────────────────────── NED / ENU 的取反（三处）


def test_起飞晚于五秒也认得出来_ENU要传rising():
    """★★ 三处取反里最隐蔽的一处：`ta.segments` 的起飞判法是给 **A 档 NED** 写的
    （第一帧 `z < liftoff_cross`）。B 档真值是 **ENU**，直接传 `liftoff_cross=0.15`
    取到的是**第一帧地面**（z=0 < 0.15）⟹ `liftoff_t` 记成 0、`climb` 恒等于"开场头 5 s"。

    而 B 档真值**从回放启动就开始录**、起飞在 OFFBOARD 之后好几秒（夹具要等帧、
    等 DUT、等解锁）⟹ 起飞晚于 5 s 时，"起飞段"里全是地面数据，判据② 会报
    **「真值高度没升上去」** —— 一个把仪器毛病写进控制器账上的假 ❌。
    """
    late = truth_(fn=lambda t: (0.0, 0.0, min(1.0, max(0.0, t - 8.0))))
    right = ta.segments(late, liftoff_cross=0.15, hover_s=8.0, rising=True)
    wrong = ta.segments(late, liftoff_cross=0.15, hover_s=8.0)          # 不传 rising
    assert right["liftoff_t"] == pytest.approx(8.15, abs=0.05)         # 8.0 起飞 + 爬到 0.15 m
    assert any(r[3] > 0.4 for r in right["climb"])                     # 认出来了
    assert wrong["liftoff_t"] == pytest.approx(0.0, abs=0.02)          # 记成了地面
    assert not any(r[3] > 0.4 for r in wrong["climb"])                 # ⟹ 假 ❌


def test_A档的判法一字未改_默认仍是NED():
    """`rising` 默认 `False` ⟹ A 档（NED，z 向下为正）的行为必须**逐字不变**。"""
    ned = [[i / 100.0, 0.0, 0.0, -min(1.0, i / 100.0), 0.0, 0.0, 0.0, T_ABS0 + i / 100.0]
           for i in range(500)]
    seg = ta.segments(ned, liftoff_cross=-0.15, hover_s=3.0)
    assert seg["liftoff_t"] == pytest.approx(0.15, abs=0.02)
    assert any(r[3] < -0.4 for r in seg["climb"])


def test_判据2的起飞段只认_升上去():
    assert one(b.evaluate_b(make_obs()), "② 行为")["ok"] is True


def test_判据3的好飞机不该被地面段判成触地():
    """★★ 第二处取反：`divergence` 的"没触地"若拿**整条真值**的最低点判
    （`z_min_all`），那它必然 ≈ 0 —— 真值是从回放启动就开录的，
    飞机那会儿正停在地面上。于是**每一场都 ❌**，报告会写成"触地了"。

    判在**离地段**（`z_min_air`）才对：这一场全程没下降，判据③ 该过。
    """
    obs = make_obs(extra_delay_ms=50.0)
    checks = b.evaluate_b(obs)
    di = obs["diverge"]
    assert di["z_min_all"] < 0.05                     # 整条真值含地面 ⟹ 恒触地
    # 离地段从**离地那一刻**起算 ⟹ 最低点就是那个 0.15 m 的门槛值（不是稳态的 1 m）：
    # 这一条判的是"有没有再掉回地面"，不是"悬停高度准不准"（那是判据① 的事）。
    assert di["z_min_air"] == pytest.approx(0.15, abs=0.01)
    assert di["z_min_air"] > b.GROUND_M
    assert one(checks, "★判据③")["ok"] is True


# ────────────────────────────────────────────── 稳定窗口


def test_稳定窗口止于最后一次正常帧():
    """★ 判据④ 那场里注入之后**全程**都是兜底帧。窗口若取到整场末尾，
    "兜底保不住高度"就会被记成"判据① 悬停精度不行" —— 一条 ❌ 混两件事。"""
    fs = [frame(i) for i in range(100)] + [frame(i, fallback="hover", thrust=-0.1589)
                                          for i in range(100, 300)]
    lo, hi = b.steady_bounds(fs, steady_s=1.0)
    assert hi == pytest.approx(fs[99]["wall_ns"] / 1e9)
    assert hi - lo == pytest.approx(1.0)


def test_稳定窗口在没有正常帧时退回末帧():
    """全是兜底帧（起手就没进来）⟹ 仍要给一个窗口，不能返回 None 把判据整条吞掉。"""
    fs = [frame(i, fallback="hover") for i in range(50)]
    lo, hi = b.steady_bounds(fs, steady_s=0.5)
    assert hi == pytest.approx(fs[-1]["wall_ns"] / 1e9)


def test_稳定窗口_空帧表返回None():
    assert b.steady_bounds([], steady_s=1.0) is None


def test_目标取中位数_不受锚点重钉那一拍的影响():
    """`x_ref0` 在锚点重钉那一拍照例会跳一下；均值会把它摊进目标里，中位数不会。"""
    fs = [frame(i, ref=(0.0, 0.0, 1.0)) for i in range(99)]
    fs.append(frame(99, ref=(5.0, 0.0, 1.0)))            # 一个离群点
    t = b.ref_target(fs, 0.0, fs[-1]["wall_ns"] / 1e9)
    assert t == [0.0, 0.0, 1.0]


# ────────────────────────────────────────────── 判据①


def test_判据1_悬停在目标上就通过():
    ck = one(b.evaluate_b(make_obs()), "★判据① 稳态位置误差")
    assert ck["ok"] is True, ck["detail"]


def test_判据1_偏了0_2米就不通过():
    """正好卡在票面的 0.15 m 外面 —— 这条是"判据真的会炸"的证据。"""
    obs = make_obs(truth=truth_(fn=lambda t: (0.2, 0.0, min(1.0, t))))
    ck = one(b.evaluate_b(obs), "★判据① 稳态位置误差")
    assert ck["ok"] is False
    assert "0.2" in ck["detail"]


def test_判据1_高度差也算在内():
    """水平 0 但高度差 0.2 m ⟹ 仍不通过（票面说的是"位置误差"，不是"水平误差"）。"""
    obs = make_obs(truth=truth_(fn=lambda t: (0.0, 0.0, 0.8)))
    assert one(b.evaluate_b(obs), "★判据① 稳态位置误差")["ok"] is False


def test_判据1_真值点太少记没做_并另有计入的前置行兜住():
    """★ 三分法：**数据不够**（`ok=None`，不计数）与**不通过**（`False`，计数）不是一回事。

    真值流没覆盖到窗口时，报"判据① 不通过"会把仪器毛病写成控制器结论 —— 而这一条
    恰恰是本档最容易被引用的那行。所以判据① 记「没做」，另由一条**计入**的
    前置行（仪器自检）保证"本场不能算通过"这件事仍然说得出。
    """
    obs = make_obs(truth=[r for r in truth_() if r[7] <= T_ABS0 + 2.0])
    checks = b.evaluate_b(obs)
    crit = one(checks, "★判据① 稳态位置误差")
    pre = one(checks, "★判据① 前置")
    assert crit["ok"] is None and crit["counts"] is False
    assert pre["ok"] is False and pre["counts"] is True
    assert "★判据① 稳态位置误差" not in failed(checks)      # 不在「不通过」名单里


def test_判据1振荡_高度在抖就不过():
    """±0.4 m 的方波 ⟹ 峰峰值 0.8 m > 0.35 m，穿均值频率也应该很高。"""
    obs = make_obs(truth=truth_(fn=lambda t: (0.0, 0.0, 1.0 + 0.4 * (int(t * TRUTH_HZ) % 2))))
    assert one(b.evaluate_b(obs), "★判据①·振荡")["ok"] is False


def test_判据1振荡_平的就过():
    assert one(b.evaluate_b(make_obs()), "★判据①·振荡")["ok"] is True


# ────────────────────────────────────────────── 判据②（两条轨迹，**定性判**）


def lemniscate(t):
    """造一条"会动的"参考（**r = 1.0 m** 的八字量级）：先爬升到 1 m，再画。

    ★ 判据②·轨迹2 只要求"参考动过、且整段落在 OFFBOARD 窗口里"，不要求它真是八字
      —— 形状那件事由 `bag/5_traj3_lemniscate`（2026-10-08 录成的 1.0 m 那份，票 05 起
      `T0` 的输入）与 `mocap_model` 那边的读数保证，
      这里造的是**判据层的输入**，不是参考本身。
    """
    return (1.0 * math.sin(t), 0.5 * math.sin(2 * t), min(1.0, t))


def traj_obs(*, ref_fn=lemniscate, truth_fn=lemniscate, move=None, n_moving=None,
             nav_state=None):
    """`--reference-source traj` 的一场（参考会动、时间线落在 OFFBOARD 窗口里）。

    `x_ref0` 与真值走**同一条**路径 ⟹ 跟踪误差 ≈ 0；要造误差就传另一个 `truth_fn`。

    ★★ 时基照 obs 的**真身**造：`offboard_enter_t` / `flight_end_t` / `move_*_s` 都是
      **记录器钟**（`rec.t0` 起算），而 `frames[].wall_ns` / 真值第 7 列是**绝对**墙钟。
      这里把 `T_ABS0` 当记录器钟的零点，并由 `t["first_frame"]=0.0` 告诉判层
      （判层靠 `rec_clock_zero()` 换算 —— ticket 10 缺陷 A）。
      ⚠️ 别图省事把两根钟**混着造**：那样测的是"两根钟恰好同源"这个假前提，
        真机上 (`rec.t0 ≈ 1.79e9`) 一跑就现原形。
    """
    ov, end = 2.0, 18.0                                    # ← 记录器钟的秒数
    frames = [frame(i, ref=ref_fn(wall(i) - T_ABS0), nav_state=nav_state)
              for i in range(int(DUR_S * FRAME_HZ))]
    obs = make_obs(frames=frames, truth=truth_(fn=truth_fn))
    obs["reference_source"] = "traj"
    obs["t"] = {"offboard_enter": 2.0, "first_frame": 0.0,
                "offboard_enter_t": ov, "flight_end_t": end}
    m_lo, m_hi = move or (3.0, 14.0)                       # ← 也是记录器钟
    obs["reference_timeline"] = {
        "n": 1300, "n_moving": 900 if n_moving is None else n_moving,
        "by_phase": {"pre": 200, "bag": 900, "hold": 200},
        "first_s": 2.5, "last_s": 17.5,
        "move_first_s": m_lo, "move_last_s": m_hi,
        "move_span_s": round(m_hi - m_lo, 3)}
    return obs


def test_判据2_悬停场判轨迹1_八字那条记本场不做():
    """`hold` 档判**悬停**那条；八字那条当场记「本场不做」（`ok=None`、**不计数**）。

    ★ 为什么另一条不能也计数：一场跑不出两条轨迹。两条都 `counts=True` 的话，
      每一场都会凭空多一条"不通过"，而那是"这一场没跑它"、不是"它不通过"。
    """
    checks = b.evaluate_b(make_obs())
    hover = one(checks, "★判据②·轨迹1")
    lem = one(checks, "★判据②·轨迹2")
    assert hover["ok"] is True and hover["counts"] is True
    assert lem["ok"] is None and lem["counts"] is False
    assert "reference-source traj" in lem["detail"]


def test_判据2_八字场判轨迹2_悬停那条记本场不做():
    obs = traj_obs()
    checks = b.evaluate_b(obs)
    hover = one(checks, "★判据②·轨迹1")
    lem = one(checks, "★判据②·轨迹2")
    assert lem["ok"] is True and lem["counts"] is True
    assert hover["ok"] is None and hover["counts"] is False
    # 误差读数必须**留档**（操作者要求：不设界，但要有数）
    # ★ `pair_frac` 是这一条的关键：两条流要真的配起来（`PAIR_TOL_S` 那条容差
    #   在这上面才是承重的）。只断言"配上了几个点"的话，容差写坏成 0（只有恰好同刻的
    #   才配上）照样能过 —— 变异测试实测：`n_paired > 100` 在容差归零时**仍绿**。
    err = obs["traj"]["lemniscate"]["err"]
    assert err["n_paired"] > 100 and err["pair_frac"] > 0.95 and err["h_mean"] < 0.01


def test_判据2_跟踪误差只留档_再大也不改判():
    """★★ 操作者 2026-09-30 裁定：spec 原文对这一条只有「不撞地、不失控」，**没有数值界**
    ⟹ **不编阈值**。把真值整体平移 3 m（误差远大于判据① 的 0.15 m 界），
    判据② 的 `ok` 仍必须是 `True`、误差照样进 `obs["traj"]["lemniscate"]["err"]`。

    ★ 这是一条**反向**断言（"偏这么多该不过"的直觉在这里是错的）。没有它，下一个人
      "顺手"给误差补个阈值就能全绿通过 —— 而那正是票面明令禁止的编阈值。
    """
    obs = traj_obs(truth_fn=lambda t: (lemniscate(t)[0] + 3.0, lemniscate(t)[1]
                                       + 3.0, lemniscate(t)[2]))
    lem = one(b.evaluate_b(obs), "★判据②·轨迹2")
    assert lem["ok"] is True and lem["counts"] is True
    assert obs["traj"]["lemniscate"]["err"]["h_mean"] > 3.0
    assert "只留档" in lem["detail"]


def test_判据2_八字没在OFFBOARD窗口里跑完就不过():
    """八字**跑出**了 OFFBOARD 窗口 ⟹ 那一段不受控 —— 判"没跑完"，且**计数**。"""
    obs = traj_obs(move=(1.0, 19.0))                       # 起点早于 OFFBOARD、终点晚于收手
    lem = one(b.evaluate_b(obs), "★判据②·轨迹2")
    assert lem["ok"] is False and lem["counts"] is True
    assert "整段落在窗口内=False" in lem["detail"]


def test_判据2_参考一帧没动记没做_不计数():
    """参考全程不动是**夹具的征兆**，不是"飞机飞完了八字" ⟹ 记「没做」，不计数。"""
    obs = traj_obs(n_moving=0)
    lem = one(b.evaluate_b(obs), "★判据②·轨迹2")
    assert lem["ok"] is None and lem["counts"] is False
    assert "没有一帧在动" in lem["detail"]


def test_判据2_参考时间线读不到记判不了_不计数():
    obs = traj_obs()
    obs["reference_timeline"] = {"n": 0, "why": "没有日志"}
    lem = one(b.evaluate_b(obs), "★判据②·轨迹2")
    assert lem["ok"] is None and lem["counts"] is False
    assert "判不了" in lem["detail"]


def test_判据2_悬停时离了OFFBOARD就算失控_不过():
    """★ "不失控"判在**逐帧 nav_state** 上：一帧掉出 OFFBOARD 就说明那一段不受控。

    人造场记默认 OFFBOARD，所以这里必须**主动弄坏**才能确认这一条有鉴别力
    （变异测试：断言红不了就是没鉴别力）。
    """
    frames = [frame(i) for i in range(int(DUR_S * FRAME_HZ))]
    frames[1000]["nav_state"] = 2                          # 掉出 OFFBOARD 一帧
    hover = one(b.evaluate_b(make_obs(frames=frames)), "★判据②·轨迹1")
    assert hover["ok"] is False and hover["counts"] is True


def test_判据2_悬停时掉到地面就算撞地_不过():
    truth = truth_(fn=lambda t: (0.0, 0.0, min(1.0, t) if t < 15 else 0.02))
    hover = one(b.evaluate_b(make_obs(truth=truth)), "★判据②·轨迹1")
    assert hover["ok"] is False and hover["counts"] is True


# ────────────────────────────────────────────── traj 档的模型自检（起世界之前拦住）


def test_模型里没有参考就拦住(tmp_path):
    """拿**包 3** 那份模型跑 traj 档 ⟹ 发出去的参考恒为 0、八字根本不会跑。

    ★ 这条错法**完全静默**（夹具不报错、飞机照飞），只在报告里留一句"参考一帧没动" ——
      所以必须在起世界之前拦住。它判在**纯函数**上 ⟹ 不用烧窗口就能变异测试。
    """
    p = tmp_path / "m3.json"
    p.write_text(json.dumps({"reference": {"n": 0, "why": "包里没有这个话题"}}),
                 encoding="utf-8")
    problem = b.traj_model_problem(p)
    assert problem is not None and "恒为 0" in problem and "mocap-model-5.json" in problem


def test_模型读不出来也拦住(tmp_path):
    p = tmp_path / "坏.json"
    p.write_text("{不是 json", encoding="utf-8")
    assert b.traj_model_problem(p) is not None
    assert b.traj_model_problem(tmp_path / "没有这个文件.json") is not None


def test_入库的三份模型各判各的():
    """★ 直接判**归档里那三份真模型**：包 3 那份该被拦住、包 4 / 包 5 该放行。

    用真文件而不是人造 dict，是因为这几份是 `run_matrix.sh` 真正会传的路径 ——
    人造 dict 测不出"包 5 的模型里其实没写进 `reference` 块"那类入库事故。

    ★ 包 4 那份**留着不删**：它证明 `traj_model_problem()` 只认"参考非不非空"、
      不认"是不是当前规格那条" —— 判据② 该跑哪条由**接线**（票 05）钉，不由这里钉。
      ⚠️ 后果要记住：**拿 0.5 m 的包 4 冒充 1.0 m 的规格，这个纯函数拦不住**
      （它参考非空、`span_s` 只是短一半）⟹ 挡它的是下面的 `test_T0接的是1_0m那份八字`。
    """
    ev = Path(b.__file__).resolve().parent / "evidence" / "tier-b"
    m3 = ev / "mocap-model-3.json"
    m4 = ev / "mocap-model-4.json"
    m5 = ev / "mocap-model-5.json"
    for m in (m3, m4, m5):
        assert m.exists(), f"归档模型缺了：{m}"
    assert b.traj_model_problem(m3) is not None, "包 3 的参考恒为 0，该被拦住"
    assert b.traj_model_problem(m4) is None, "包 4 的模型该带上它那条 0.5 m 八字参考"
    assert b.traj_model_problem(m5) is None, "包 5 的模型该带上那条 1.0 m 八字参考"


#: 仓库根 —— `run_matrix.sh` 里的 `--model` 是**相对它**写的（脚本先 `cd` 到这里）。
REPO_ROOT = HERE.parents[1]


def _t0_wiring():
    """从 `run_matrix.sh` 的 `T0)` 那一场**现取接线**：`(模型路径, --hold-s)`。

    ★★ 为什么不在这里逐字抄那两个数：抄了就跟脚本漂开 —— 它们**已经漂过一次**（票 05
      落地前：脚本指着包 4 那份 0.5 m 的旧规格，而判据② 早该跑 1.0 m 那条）。
      现取 ⟹ 改脚本（换成更长的参考、或把 `--hold-s` 改短）时，
      下面的断言**先红**，而不是等一场 SITL 白烧掉才发现。
    """
    sh = HERE / "evidence" / "tier-b" / "run_matrix.sh"
    text = sh.read_text(encoding="utf-8")
    m = re.search(r"^\s*T0\)(.*?);;", text, re.S | re.M)          # 带续行，接成一条再解析
    assert m, f"`run_matrix.sh` 里找不到 `T0)` 那一场：{sh}"
    line = m.group(1).replace("\\\n", " ")
    model = re.search(r"--model\s+(\S+)", line)
    hold = re.search(r"--hold-s\s+([0-9.]+)", line)
    assert model and hold, f"`T0)` 那一场里没同时给出 `--model` 与 `--hold-s`：{line!r}"
    path = Path(model.group(1))
    return (path if path.is_absolute() else REPO_ROOT / path), float(hold.group(1))


def _model_reference(model_path):
    """取出归档模型里那条参考。★ 缺 `reference` 块就**当场炸**（不是悄悄返回 `{}`）。"""
    ref = json.loads(model_path.read_text(encoding="utf-8")).get("reference") or {}
    assert ref.get("n"), f"{model_path} 里没有 `/traj/reference`（n={ref.get('n')}）"
    return ref


def test_T0接的是1_0m那份八字_不是0_5m那份():
    """★★ `T0` 那一场判的是**当前规格那条轨迹** —— 1.0 m 八字。

    ★ 为什么要有这一条：`traj_model_problem()` 只拦"参考恒为 0"，**拦不住**
      "拿 0.5 m 的旧包冒充 1.0 m 的规格"（旧包参考非空、只是短一半）。而规格一旦漂回
      旧包，报告读起来一切正常 —— 正是本仓库最怕的那类**静默失败**。
      ⟹ 这里直接量**接线指向的那份模型**的参考包络：包 5 = 2.000 × 1.000 m（r = 1.0），
      包 4 = 1.000 × 0.500 m（r = 0.5）—— 两者差整整一倍，**只比 x 半幅也分得开**。
    """
    model, _hold_s = _t0_wiring()
    ref = _model_reference(model)
    xs = [p[0] for p in ref["p"]]
    ys = [p[1] for p in ref["p"]]
    span_x, span_y = max(xs) - min(xs), max(ys) - min(ys)
    assert span_x == pytest.approx(2.0, abs=1e-3), (
        f"`T0` 接的参考包络 x = {span_x:.3f} m ⟹ 不是 1.0 m 那条八字（0.5 m 那份是 1.000 m）"
        f"；接线来自 {model}")
    assert span_y == pytest.approx(1.0, abs=1e-3), f"`T0` 接的参考包络 y = {span_y:.3f} m"


def test_hold_s太短就拦住_下界从模型现算():
    """★ `--hold-s` 的下界**从模型里量出来的参考全长现算**，不是拍脑袋。

    规则：`hold_s ≥ ref_hold_s + 参考全长 + 判据① 的稳态窗口`。三段是**串起来的**
    （`pre` 顶住首点 → 参考整条 → 留出稳态窗口），少一截的症状是判据① 的稳态窗口里
    混进八字或爬升段 —— 读出来像"悬停精度很差"，而控制器没问题。

    ★★ 这里的数与接线**都不写死**：`--model` / `--hold-s` 从 `run_matrix.sh` 的 `T0)`
      那一场现取，参考全长从**那个模型**里现量。改短脚本那一行 ⟹ 这条先红。
    """
    model, hold_s = _t0_wiring()
    span = _model_reference(model)["span_s"]
    need = 30.0 + span + b.STEADY_S                # 30 = `--ref-hold-s` 的默认（脚本没传）
    # ★ 脚本现在给的那个数**必须过**（这是"逐字过一遍"那条断言，值改成现取的了）
    assert b.traj_model_problem(model, ref_hold_s=30.0, hold_s=hold_s) is None, (
        f"`run_matrix.sh` 的 `T0)` 给的 `--hold-s {hold_s:g}` 撑不住 {model.name}"
        f"（下界 {need:.2f} s：30 + {span:.2f} + {b.STEADY_S:g}）")
    # 下界本身也要过得去（不是"刚好差一点也没事"）
    assert b.traj_model_problem(model, ref_hold_s=30.0, hold_s=math.ceil(need)) is None
    short = b.traj_model_problem(model, ref_hold_s=30.0, hold_s=need - 1.0)
    assert short is not None and "太短" in short and f"{need:.2f}" in short
    # ☠️ 那一句拍脑袋的旧下界（`ref_hold_s + 10`）**必须被这条判死**：
    #    它算不出"参考全长"这一截 —— 1.0 m 那份参考有 26.22 s，差着两条八字。
    assert b.traj_model_problem(model, ref_hold_s=30.0, hold_s=41.0) is not None


# ────────────────────────────────────────────── 末段推力那一列：折算常量从哪来


def _motors_text(mean=0.5):
    """造一份 `actuator_outputs_sim` 的控制台文本（四路同值 ⟹ 均值 = `mean`）。

    四路同值是有意的：差异那部分是姿态环在分配力矩，"总推力"只看均值（`motor_mean`）。
    """
    return "actuator_outputs_sim\n    output: [" + ", ".join(f"{mean:.4f}" for _ in range(16)) + "]"


def test_末段推力列按本场参数文件折算_不按入库机型文件():
    """★★ `thrust_n_end` = 电机均值 × 4 × `SIH_T_MAX`，而 `SIH_T_MAX` **必须取本场的**。

    ★ 为什么非要有这一列：`airframe_param()` 读的是**入库的静态机型文件**，看不见
      `--mismatch` 那份 overlay（`z0-plant/params/*.param`）—— 21 清单第 266 条拿 13 场
      逐场算过：`thrust±30` 那两档整整差 **0.769 / 1.429** 倍，而报告读起来一切正常
      （这正是本仓库最怕的那类静默失败）。phase3 的 `04` 又把入库机型文件从
      `11.8 N` 换成了 `46.3027 N`，所以"拿哪个来源"这件事**今天必须写死在代码里**。

    ★ 这一条**挑得开对错**：`thrust+30` 那份 overlay 是 `15.0484`、入库机型文件是
      `11.5757` —— 拿错来源差 0.769 倍，不是"差一点点"。
    """
    txt = _motors_text(0.5)
    nominal = b.plant_t_max_motor("z0.param")
    thrust30 = b.plant_t_max_motor("z0-thrust+30.param")
    airframe = b.v.airframe_param("SIH_T_MAX")
    assert nominal == pytest.approx(11.5757, abs=1e-4), f"标称 overlay 的 SIH_T_MAX = {nominal}"
    assert thrust30 == pytest.approx(15.0484, abs=1e-4), f"thrust+30 overlay 的 SIH_T_MAX = {thrust30}"
    # ☠️ 两个来源在**本场这一档**上必须真的不同，否则这条测试没有鉴别力
    assert abs(thrust30 - airframe) > 1.0, (
        f"thrust+30 overlay({thrust30}) 与入库机型文件({airframe}) 太近 ⟹ 这条断言挑不开对错")

    col = b.thrust_end_column(txt, "z0-thrust+30.param")
    assert col["thrust_t_max_n"] == pytest.approx(thrust30, abs=1e-4), (
        "折算常量没跟着本场参数文件走 ⟹ 回落到入库机型文件了")
    assert col["thrust_n_end"] == pytest.approx(0.5 * 4.0 * thrust30, abs=1e-4)
    # 拿入库机型文件那个常量会算出另一个数（**分开**断言，才证明差的不是浮点尾数）
    assert col["thrust_n_end"] != pytest.approx(0.5 * 4.0 * airframe, abs=1e-2)


def test_标称overlay与入库机型文件同源():
    """同一个常量在盘上有**两处**写法：机型文件的 `param set-default` 与标称 overlay
    `z0.param` 的 `param set`。两处不一致 ⟹ 一场"标称"飞行其实飞的是另一架飞机。

    ★ 这条是给"下次改机型只改了一处"准备的：`04` 重搭时 `--emit` 全量重生成的是
      overlay 那一批，机型文件由 `install_z0_airframe.sh` 装 —— 两条路，各写各的。
    """
    airframe = b.v.airframe_param("SIH_T_MAX")
    assert b.plant_t_max_motor("z0.param") == pytest.approx(airframe, abs=1e-4), (
        f"标称 overlay 的 SIH_T_MAX 与入库机型文件对不上："
        f"{b.plant_t_max_motor('z0.param')} vs {airframe} —— 标称那一场飞的不是机型文件那架飞机")


def test_末段推力列读不出参数文件时给出None_不抛():
    """读不出来要**返回 None**（判据层按"没读到"处理），不许抛、也不许悄悄用别处的常数。"""
    col = b.thrust_end_column(_motors_text(0.5), "没有这个文件.param")
    assert col["thrust_t_max_n"] is None and col["thrust_n_end"] is None
    assert col["motors_mean_end"] == pytest.approx(0.5, abs=1e-9)


# ────────────────────────────────────────────── 判据③（延迟）


def test_判据3_没加延迟就是没做_不计数():
    """★ 三分法：「没做」与「没通过」不是一回事。写反的症状是：
    一场**没做延迟实验**的飞行被判成"判据③ 不通过"。"""
    ck = one(b.evaluate_b(make_obs(extra_delay_ms=0.0)), "★判据③")
    assert ck["ok"] is None and ck["counts"] is False


def test_判据3_加了延迟且稳稳的_就通过():
    ck = one(b.evaluate_b(make_obs(extra_delay_ms=50.0)), "★判据③")
    assert ck["ok"] is True, ck["detail"]


def test_判据3_发散就不通过():
    """高度一路往下掉 ⟹ 末段垂直速度很大、离地段最低点触地 ⟹ 不过。"""
    obs = make_obs(extra_delay_ms=50.0,
                   truth=truth_(fn=lambda t: (0.0, 0.0, 1.0 - 0.5 * t)))
    assert one(b.evaluate_b(obs), "★判据③")["ok"] is False


def test_判据3_慢慢漂走也算发散():
    """高度还在 1 m 附近、但水平一路漂 —— "不能发散"包含这一路。"""
    obs = make_obs(extra_delay_ms=50.0,
                   truth=truth_(fn=lambda t: (0.3 * t, 0.0, 1.0)))
    assert one(b.evaluate_b(obs), "★判据③")["ok"] is False


# ────────────────────────────────────────────── 判据④（M5 兜底）


def _m5_obs(*, descent_pred=0.0, plant_param="z0-mass+30.param", mismatch="mass+30",
            hold_n=5, hover_n=500, model_ht=0.1589):
    """造一场"注入求解失败"的观测：前段正常，中段 `hold`，后段 `hover`；高度按
    `descent_pred` 匀加速往下掉（0 = 兜底把高度保持住了）。"""
    n_head, n = 20, 20 + hold_n + hover_n
    fs = ([frame(i) for i in range(n_head)]
          + [frame(n_head + i, fallback="hold", thrust=-0.60, note="注入的坏解")
             for i in range(hold_n)]
          + [frame(n_head + hold_n + i, fallback="hover", thrust=-model_ht,
                   note="注入的坏解：`-p inject_solve_fail_after_frames:=…` 在生效")
             for i in range(hover_n)])
    t_hover = wall(n_head + hold_n)
    rows = []
    for i in range(int((n / FRAME_HZ + 1.0) * TRUTH_HZ)):
        t_abs = T_ABS0 + i / TRUTH_HZ
        z = 1.0 if t_abs < t_hover else 1.0 - 0.5 * descent_pred * (t_abs - t_hover) ** 2
        rows.append([i / TRUTH_HZ, 0.0, 0.0, z, 0.0, 0.0, 0.0, t_abs])
    return make_obs(frames=fs, truth=rows, inject=800, model_ht=model_ht,
                    plant_param=plant_param, mismatch=mismatch)


def test_判据4_没注入就是没做_不计数():
    ck = one(b.evaluate_b(make_obs(inject=0)), "★判据④")
    assert ck["ok"] is None and ck["counts"] is False


def test_判据4A_阶梯按_hold_到_hover_走了_就算阶梯通过():
    """★ 结构证据：悬停帧的 `thrust_body_z` **恒等于 −hover_thrust**
    （`mpc_node.py:405`）⟹ 日志里出现这个常数就只能来自兜底。"""
    obs = _m5_obs()
    a = one(b.evaluate_b(obs), "★判据④-A")
    assert a["ok"] is True, a["detail"]
    assert obs["fallback_report"]["order"] == ["normal", "hold", "hover"]
    assert obs["fallback_report"]["hover_thrust_body_z_seen"] == [-0.1589]


def test_判据4A_只进hover没经过hold_不算走完阶梯():
    """`hold` 那一档是"内容原样、时间戳换新"——**跳过它**说明阶梯没有按设计走。"""
    fs = [frame(i) for i in range(20)] + [frame(20 + i, fallback="hover") for i in range(50)]
    obs = make_obs(frames=fs, inject=800)
    assert one(b.evaluate_b(obs), "★判据④-A")["ok"] is False


def test_判据4B_对得上的被控对象_兜底保住了高度就通过():
    """对照场（`--mismatch none`）：模型与被控对象同口径 ⟹ 兜底该把高度保持住。"""
    ck = one(b.evaluate_b(_m5_obs(plant_param="z0.param", mismatch="none")), "★判据④-B")
    assert ck["ok"] is True, ck["detail"]


def test_判据4B_失配被控对象_兜底保不住高度就不通过():
    """★ 这一条是**预期会 ❌** 的，而且是本票要的那条结论：
    兜底发的固定悬停推力是**模型参数**，失配机型需要 1.30 倍 ⟹ 必然下坠。
    它必须记成 ❌ + 实际数值，**不能**因为"知道原因"就放过。"""
    ck = one(b.evaluate_b(_m5_obs(descent_pred=2.264)), "★判据④-B")
    assert ck["ok"] is False
    assert "hover_thrust" in ck["detail"]


def test_判据4B_预测下降率与机型文件同源_且符号对():
    """`(1 − 模型/被控对象)·g`：**正 = 推力不足 = 下坠**。写反的话，
    "失配机型下坠"会被报成"上飘"，而两种情形的处置完全不同。"""
    obs = _m5_obs(descent_pred=2.264)
    b.evaluate_b(obs)
    pred = obs["m5"]["predicted_descent_ms2"]
    want = b.plant_hover_thrust("z0-mass+30.param")["hover_thrust"]
    assert pred == pytest.approx((1.0 - 0.1589 / want) * 9.81)
    assert pred > 0                                    # 模型推力不足 ⟹ 下坠
    assert pred == pytest.approx(2.264, abs=0.01)


def test_判据4B_对得上的被控对象_预测值应该是零():
    obs = _m5_obs(plant_param="z0.param", mismatch="none", model_ht=0.1589)
    b.evaluate_b(obs)
    assert abs(obs["m5"]["predicted_descent_ms2"]) < 0.01


def test_判据4C_归属行不计数():
    """归属是**读数**不是判决 —— 计进去会让"报告不通过"变成算术问题。"""
    ck = one(b.evaluate_b(_m5_obs()), "★判据④-C")
    assert ck["ok"] is None and ck["counts"] is False


# ────────────────────────────────────────────── 整体


def test_悬停场_计入的判据全过():
    """一场**正常悬停**的账面：计入的判据**一条都不该红**
    —— 这条给"判据层是不是整体判反了"一个一毫秒的答案。

    ★ 它原来断言「只有判据② 红」（那时判据② 是一行硬编码 `False`）。现在判据②
      是真判，一场正常悬停该判 ✅ ⟹ 断言反过来：`failed()` 必须**空**。
      ⚠️ 空名单 ≠ 全绿：判据② 的八字那条在这一场是 `counts=False`（本场不做），
      它**不在** `failed()` 里也不在"通过"里 —— 这正是它想要的（矩阵汇总另算）。
    """
    checks = b.evaluate_b(make_obs())
    assert failed(checks) == [], failed(checks)
    assert one(checks, "★判据②·轨迹2")["counts"] is False


def test_DUT一帧都没发时提前收手_且判据层不崩():
    obs = make_obs()
    obs["dut"]["frames"] = 0
    checks = b.evaluate_b(obs)
    assert any("DUT 一帧都没发" in c["name"] for c in checks)
    assert any(c["ok"] is False for c in checks)


def test_判据层是obs的纯函数_同一份obs两次结果一致():
    """`--replay` 的前提：判据**只读 obs**。任何"跑的时候顺手改了什么"都会破坏它。"""
    obs = make_obs()
    a = b.evaluate_b(obs)
    c = b.evaluate_b(obs)
    assert [(x["name"], x["ok"], x["counts"]) for x in a] == \
           [(x["name"], x["ok"], x["counts"]) for x in c]


# ────────────────────────────────────── ⓪b 仪器自检：DUT 报出的姿态 vs 真值
#
# ☠☠ 2026-09-29 B 档首飞就是这么废掉的：夹具把四元数分量序写反（`px4_pose_to_enu`
#     给的是 wxyz，而 `PoseStamped.orientation` 的字段是 x,y,z,w），水平机在节点眼里
#     变成"绕 x 转 90°"。**位置判据、饱和日志、闸门日志全都正常**，只有把
#     "DUT 报出来的姿态"与"同一时刻的真值"对撞才看得见 —— 所以这一条是**仪器自检**，
#     不是判据：它判红时，整场的位置读数都不作数。


def _level_att_tap(n=300, hz=TRUTH_HZ, t_abs0=T_ABS0, q_wxyz=(1.0, 0.0, 0.0, 0.0)):
    """只有 `att` 的假 tap（形状同 `TruthTap.att`：`(墙钟, w, x, y, z)`，**NED/FRD 原生**）。"""
    import types
    return types.SimpleNamespace(
        att=[(t_abs0 + i / hz,) + tuple(q_wxyz) for i in range(n)])


def _frames_with_state_q(q_flu_wxyz, n=200, delay_s=0.0):
    """造逐帧日志，`x0[6:10]` 放**节点报出来的姿态**（ENU/FLU，wxyz）。"""
    out = []
    for i in range(n):
        f = frame(i)
        f["x0"] = [0.0, 0.0, 1.0, 0.0, 0.0, 0.0] + list(q_flu_wxyz)
        out.append(f)
    return out


def test_仪器自检_分量序正确时判绿():
    """正对照：DUT 报的就是"真值过一遍那唯一一处换算"⟹ 夹角≈0、判据✅。

    ★ 没有这一条，"判红"那条说明不了任何事 —— 它可能只是把两个永远对不上的量
      放在一起比（那样它**恒红**，看起来还很像"很严"）。
    """
    q_ned = (1.0, 0.0, 0.0, 0.0)
    q_flu = b.px4_quat_to_enu(q_ned)
    obs = make_obs(frames=_frames_with_state_q(q_flu))
    obs["_tap"] = _level_att_tap()
    c = b.evaluate_b(obs)
    row = one(c, "⓪b")
    assert row["ok"] is True, row["detail"]
    assert row["counts"] is True
    assert obs["att_err_deg"]["median"] == pytest.approx(0.0, abs=1e-6), obs["att_err_deg"]


def _old_buggy_readback(q_msg_wxyz):
    """复刻**修之前**那条路，返回节点最终读到的 ENU/FLU 姿态（wxyz）。

    旧三行：`_on_att` 把 PX4 的 wxyz 存成 xyzw → 直接喂进 `px4_quat_to_enu`
    （它按 wxyz 解释）→ 又把返回的 wxyz 按 xyzw 写进消息字段。
    这里照抄这条路径，而不是手敲一个字面量：字面量只能钉住"我当时算出来的那个数"，
    抄路径才能钉住"错在哪一步"。
    """
    x, y, z, w = q_msg_wxyz                       # 旧的 `_on_att`：存成 xyzw
    arr = b.px4_quat_to_enu((x, y, z, w))         # 旧的转换调用（函数按 wxyz 解释）
    ox, oy, oz, ow = arr[0], arr[1], arr[2], arr[3]   # 旧的发布：当成 xyzw 写字段
    return (ow, ox, oy, oz)                       # 收端按 xyzw 读回来 ⟹ wxyz


def test_仪器自检_夹具分量序写反时判红_且读数是120度():
    """☠☠ 首飞那场的复现。

    错出来的姿态是「绕 x 转 90°」，对的应该是「绕 z 转 90°」；两个转轴**正交**，
    所以相对角是 **120°**（`2·acos(|⟨q1,q2⟩|)`）而不是 90° —— 这个数字本身就是
    "错的是分量序、不是换算公式"的指纹（公式错会给另一个角）。
    """
    q_ned = (1.0, 0.0, 0.0, 0.0)                  # 真值：水平（NED/FRD 下单位四元数）
    q_buggy = _old_buggy_readback(q_ned)
    # 与首飞逐帧日志里的 `x0[6:10]` 同形（差一个整体符号：`q` 与 `−q` 是同一姿态）
    for got, want in zip([abs(c) for c in q_buggy], [2 ** -0.5, 2 ** -0.5, 0.0, 0.0]):
        assert got == pytest.approx(want, abs=1e-12)

    obs = make_obs(frames=_frames_with_state_q(q_buggy))
    obs["_tap"] = _level_att_tap()
    c = b.evaluate_b(obs)
    row = one(c, "⓪b")
    assert row["ok"] is False, row["detail"]
    assert obs["att_err_deg"]["median"] == pytest.approx(120.0, abs=1e-6), obs["att_err_deg"]


def test_仪器自检_真值按已知延迟回退_所以判据3那场也说得通():
    """判据③ 那场动捕**故意**迟 50 ms ⟹ 配对时真值也要回退 50 ms。

    不回退的话，"延迟"本身会被记成"仪器姿态对不上"，判据③ 那场会**先被自检拦下**，
    而它拦下的理由还是错的（延迟是这一场要测的自变量，不是故障）。
    """
    q_ned = (1.0, 0.0, 0.0, 0.0)
    q_flu = b.px4_quat_to_enu(q_ned)
    tap = _level_att_tap()
    # 真值姿态在这一段里匀速转动 100°/s；DUT 看到的**本该**是 50 ms 之前那一帧
    import math as _m
    # 转速取 200°/s：50 ms 的延迟 ⟹ 10° 的夹角，而真值是 250 Hz（配对误差 ≤2 ms ⟹ ≤0.4°）
    # ⟹ 两个断言之间有三个数量级的余量，不靠"刚好越过阈值"。
    rate = _m.radians(200.0)
    tap.att = [(T_ABS0 + i / TRUTH_HZ, _m.cos(rate * (i / TRUTH_HZ) / 2), 0.0, 0.0,
                _m.sin(rate * (i / TRUTH_HZ) / 2)) for i in range(300)]
    n = 200
    frames = []
    for i in range(n):
        t = T_ABS0 + i / FRAME_HZ - 0.05                 # 状态是 50 ms 之前的
        k = min(299, max(0, int(round((t - T_ABS0) * TRUTH_HZ))))
        f = frame(i)
        f["x0"] = [0.0, 0.0, 1.0, 0.0, 0.0, 0.0] + list(b.px4_quat_to_enu(tap.att[k][1:]))
        frames.append(f)
    assert b.att_err_deg(tap, frames, delay_s=0.0)["median"] > 5.0, "不回退就该对不上"
    assert b.att_err_deg(tap, frames, delay_s=0.05)["median"] < 1.5, "回退 50 ms 后该对上"
    assert q_flu is not None


# ────────────────────────────────────── 判层的契约：它会**写回** obs（有意，不是纯函数）

def test_判层把派生块写回obs_这是刻意的契约不是副作用():
    """`evaluate_b` 名字像纯函数，实际会把派生块写回 `obs` —— `obs.json` 要留档的正是它们。

    ☠️ 钉住它是因为**这不是实现细节**：谁要是把"重判一次"写成 `evaluate_b(存下来的 obs)`，
    第二次读到的就是**第一次写下的块**（`att_err_deg` 直接复用、不重算）⟹ 判层改了却看不出
    差别、还以为"重判过了"。要干净重判得先删掉这些键，或者整场重飞。见判层 docstring 的 ⚠️。
    """
    obs = make_obs()
    # 这四个**无条件**写回（`m5` 只在注入求解失败那场才有 —— 见下面第二个断言）
    derived = ("fallback_report", "steady", "osc", "diverge")
    assert [k for k in derived if k in obs] == [], "前提：这些块本来不该在 obs 里"
    assert "m5" not in obs, "前提：没注入就没有 m5"
    b.evaluate_b(obs)
    missing = [k for k in derived if k not in obs]
    assert not missing, f"判完该写回这些派生块，却缺 {missing}"
    # 写回的是**真的派生读数**，不是占位：稳态块该带着判据① 用的那几个均值
    assert {"z_mean", "h_mean"} <= set(obs["steady"]), obs["steady"]
    # `m5` 是**条件**写回的：没注入的那场判不了兜底，自然也没有这个块
    m5_obs = _m5_obs(descent_pred=0.0)
    assert "m5" not in m5_obs, "前提：`_m5_obs` 造的是观测，判之前不该有 m5"
    b.evaluate_b(m5_obs)
    assert {"z_start", "z_end", "vz_tail"} <= set(m5_obs["m5"]), m5_obs["m5"]


def test_重判一份存下来的obs_会复用已存的姿态误差读数_不重算():
    """`_tap` 不在（JSON 落盘时会摘掉）但有 `att_err_deg` ⟹ **复用**，判层不再碰真值流。

    这是"`--replay` 复判"那条路的正常行为（读数随 JSON 落盘就是为了让 JSON 自给自足），
    但它同时意味着：**判层改了口径、拿旧 JSON 重判，⓪b 那一行不会跟着变**。
    """
    obs = make_obs()
    obs["att_err_deg"] = {"n": 42, "median": 7.5, "p95": 9.0, "max": 11.0}   # 哨兵：别处算不出来
    assert "_tap" not in obs
    detail = one(b.evaluate_b(obs), "⓪b ")["detail"]
    assert "取自本场落盘的读数" in detail, f"该走复用分支，实得：{detail[:80]}"
    assert "42 帧" in detail and "7.50" in detail, f"该报哨兵的数，实得：{detail[:80]}"


# ────────────────────────────────────────────── traj 档的接线（判据② 的八字那条）


def _cfg(**kw):
    """`--reference-source traj` 那一档的 cfg（只有 `dut_params_b`/`ref_z_offset_m` 用到的字段）。"""
    import types
    base = dict(log_file="/tmp/x.jsonl", hover_thrust=0.1589, rate_hz=100.0,
                hold_x=0.0, hold_y=0.0, hold_z=1.0,
                reference_source="hold", ref_hold_s=30.0, ref_z_offset_m=None,
                inject_solve_fail_after_frames=0)
    base.update(kw)
    return types.SimpleNamespace(**base)


def test_traj档的DUT参数_hold_offset全零_参考源写traj():
    """★★ "高度由谁叠"是这一档最容易接错的一根线，且**两条路写反都不会报错**：

    · DUT 拿到非零 `hold_offset` ⟹ `mpc_node._validate` 拒跑（吵，但至少停）；
    · **两边都不出高度** ⟹ 锚点在地面 ⟹ 飞机压根不打算离地，而判层只看到"它没动"。
    所以这里把"traj 档下 DUT 收到的 hold_offset 必须**逐字**是 0.0"钉死。
    """
    p = b.dut_params_b(_cfg(reference_source="traj", hold_z=1.0))
    assert "hold_offset_x:=0.0" in p and "hold_offset_y:=0.0" in p and "hold_offset_z:=0.0" in p
    assert "reference_source:=traj" in p
    assert "state_source:=mocap" in p, "B 档的身份不许被这条改动碰掉"


def test_hold档的DUT参数_逐字不变():
    """回归：加 `reference_source` 不许改掉老那一档的任何一条 —— 它上面挂着 F0–F4 全部基线。"""
    p = b.dut_params_b(_cfg(hold_x=0.1, hold_y=-0.2, hold_z=1.5))
    for want in ("hold_offset_x:=0.1", "hold_offset_y:=-0.2", "hold_offset_z:=1.5",
                 "reference_source:=hold", "state_source:=mocap",
                 "hover_thrust:=0.1589", "rate_hz:=100.0"):
        assert want in p, f"{want} 丢了：{p}"


def test_高度只由夹具叠_默认取hold_z():
    """一个旋钮（`--hold-z`）在两种档下各自落地：hold 档进 DUT，traj 档进夹具。

    ★ `--ref-z-offset-m` 显式给时以它为准（允许"起飞高度"与"轨迹高度"分开调）；
      不给就跟随 `--hold-z` —— 这条"跟随"正是 `ref_z_offset_m()` 单独成函数的原因：
      命令行与 obs 留档各写一遍，就会出现"命令行叠了 1 m、留档写 0"。
    """
    assert b.ref_z_offset_m(_cfg(hold_z=1.25)) == 1.25
    assert b.ref_z_offset_m(_cfg(hold_z=1.25, ref_z_offset_m=0.5)) == 0.5
    assert b.ref_z_offset_m(_cfg(hold_z=0.0)) == 0.0


def _ref_rows(t0_abs, phases):
    """造一份夹具的参考逐帧日志行（`wall_ns` 是**绝对**墙钟，`ref_timeline` 负责换算）。

    ★ 一条一行 `(phase, x, y, z, vx, vy)` —— **速度是必填的**：`ref_timeline` 判"在动"
      只看它（ticket 10 缺陷 B），所以造行的人必须像 `mocap_replay.py` 那样把速度写对。
    """
    out = []
    for i, (phase, x, y, z, vx, vy) in enumerate(phases):
        out.append({"wall_ns": int((t0_abs + i * 0.02) * 1e9), "phase": phase, "i": i,
                    "position": [x, y, z], "velocity": [vx, vy, 0.0], "yaw": 0.0})
    return "\n".join(json.dumps(r) for r in out) + "\n"


def test_参考时间线_换算到记录器墙钟_并认得出移动段(tmp_path):
    """★★ 这是 traj 档**唯一**能回答"八字到底在 OFFBOARD 窗口里跑没跑"的读数。

    为什么不能只看"夹具在发帧"：参考流**照着时间表**发，与 DUT 起没起来、进没进 OFFBOARD
    **完全无关**。DUT 起得慢 ⟹ 整条八字在**地面上**放完，而夹具汇总里帧数/节奏/掩码全对。

    ★ `pre` 那三帧**故意停在离原点 4 mm 的地方**（真身：`(0.004136, 0.004135)`）——
      按位置判的话它们全都会被算成"在动"（ticket 10 缺陷 B），按速度判才是 0。
    """
    t0 = T_ABS0
    phases = ([("pre", 0.004136, 0.004135, 1.0, 0.0, 0.0)] * 3
              + [("bag", 0.3, 0.0, 1.0, 0.5, 0.0), ("bag", 0.5, 0.1, 1.0, 0.5, 0.5),
                 ("bag", 0.0, 0.0, 1.0, 0.5, 0.5)]
              + [("hold", 0.0, 0.0, 1.0, 0.0, 0.0)] * 2)
    f = tmp_path / "ref.jsonl"
    f.write_text(_ref_rows(t0, phases), encoding="utf-8")
    proc = b.ReplayProc(_cfg(reference_source="traj"), tmp_path / "frames.jsonl", tmp_path / "s.json")
    proc.ref_out = f
    tl = proc.ref_timeline(t0)
    assert tl["n"] == 8 and tl["by_phase"] == {"pre": 3, "bag": 3, "hold": 2}
    # 移动段 = `bag` 那三帧；`pre` 虽然离原点 4 mm、`hold` 虽然 z=1 m，都不算"在动"
    assert tl["n_moving"] == 3, tl
    # 首帧在 0.0 s、移动段从第 4 帧起（3 帧 pre × 20 ms）
    assert tl["first_s"] == pytest.approx(0.0) and tl["move_first_s"] == pytest.approx(0.06)
    assert tl["move_span_s"] == pytest.approx(0.04)


def test_参考时间线_没有日志时说没做_不抛(tmp_path):
    """「没做」与「没通过」不是一回事（同判据③④ 的三分法）——
    没有夹具日志时这里要能判"判不了"，而不是让整层崩掉。"""
    proc = b.ReplayProc(_cfg(reference_source="traj"), tmp_path / "f.jsonl", tmp_path / "s.json")
    assert proc.ref_timeline(T_ABS0)["n"] == 0            # 没设 ref_out
    proc.ref_out = tmp_path / "nope.jsonl"                # 设了但文件不存在
    assert proc.ref_timeline(T_ABS0)["n"] == 0


# ────────────── 判层两处「写错了完全静默」的缺陷（ticket 10）
#
# ★★ 两处都在**判层**，都能只靠**离线重判归档 obs** 复现 —— 归档是 06 号票那批六场
#   （`evidence/tier-b/20261009-rebuild/`），拿它们重判能证明"修前红、修后绿"。
#   修前实测：六场**全部** `ok=None / counts=False`，逐场读数一模一样地
#   `DUT 帧 0 帧、真值 0 点` —— 六场读数全同，本身就是"不是飞机的事"的指纹。

ARCHIVE = HERE / "evidence" / "tier-b" / "20261009-rebuild"
#: 重判前**必须删掉**的块 —— `evaluate_b` 会把它们写回 `obs`，留着就等于"读到上一次判留下的"。
#: ⚠️ 它**不等于** `evaluate_b` docstring 里那张"写回的派生块"清单：那张表里还有
#:   `att_err_deg`，而它是**本场落盘的结果**、不是从 `frames` 现算的派生块 ⟹
#:   **不能删**（删了会退化成"判不了"、凭空少一条计入，实测 11 → 10；
#:   见 `docs/control/tier-b-sitl-seam3.md` §6.3）。
_DROP_BEFORE_REJUDGE = ("checks", "steady", "osc", "diverge", "m5", "fallback_report", "traj")


def archive_obs(tag):
    """读一份归档 obs，并把**参考时间线**按现行规则从落盘的原始逐帧重算。

    ★ 为什么要重算：落盘的那条 `reference_timeline` 是**缺陷 B 的错值**
      （`move_first_s = 6.380` —— 停在起点的 `pre` 段被算成"在动"）。
      归档**不就地改**（改它等于涂改实验记录），所以原始逐帧单独入了一份库；
      这里用的正是判层自己那支 `ref_timeline_from_rows`，**不是**另写一份规则。
    ★ `t0` 用**判层自己那支** `rec_clock_zero(obs["frames"], obs["t"])` —— 与判窗口时
      走的是**同一个**锚点，所以这份 obs 的状态就是判层真实会看到的状态。
      （另有一条**精确但循环**的取法：从落盘时间线的 `first_s` 反解，`first_s = wall − t0`。
      它能还原"采集当时那支代码本来会写下什么"，但它把待检验的值又读回去当输入了，
      **自证不了** `rec_clock_zero` 准不准 ⟹ 这里不用它，改用下面那条独立测试交叉核对。
      两者差 **3 ms**，来自 `first_frame` 在采集侧落盘到 0.01 s —— 见
      `test_零点锚点的精度_拿T0的参考日志交叉核对`。）
    """
    obs = json.loads((ARCHIVE / f"{tag}-summary.json").read_text(encoding="utf-8"))
    for k in _DROP_BEFORE_REJUDGE:
        obs.pop(k, None)
    ref = ARCHIVE / f"{tag}-replay-frames-reference.jsonl"
    if ref.exists():
        rows = [json.loads(l) for l in ref.read_text(encoding="utf-8").splitlines() if l.strip()]
        t0 = b.rec_clock_zero(obs.get("frames"), obs.get("t"))
        if t0 is not None:
            obs["reference_timeline"] = {**b.ref_timeline_from_rows(rows, t0), "log": str(ref)}
    return obs


# ── 缺陷 A：窗口的零点错配（记录器钟 ↔ 绝对墙钟）

def test_零点由obs里同刻的两根钟推出来():
    """`frames[0].wall_ns`（绝对）与 `t["first_frame"]`（记录器钟）是**同一刻**的两次读数。"""
    assert b.rec_clock_zero([frame(0)], {"first_frame": 4.0}) == pytest.approx(T_ABS0 - 4.0)


def test_取不到零点返回None_不瞎猜也不当同一根钟():
    """没有逐帧日志、或没有 `first_frame` ⟹ `None`（调用方一律记「判不了」）。

    ★ 退回"当作同一根钟"等于把本票要修的缺陷装回去；瞎猜一个零点更糟。
    """
    assert b.rec_clock_zero([], {"first_frame": 1.0}) is None
    assert b.rec_clock_zero([frame(0)], {}) is None


def test_判据2_记录器钟的窗口不换算就恒空():
    """★★ 缺陷 A 的靶心：`offboard_enter_t` / `flight_end_t` 是**记录器钟**的秒数，
    判层却拿它们当绝对墙钟使 ⟹ 窗口变成 `[1.79e9, 18.0]`（**一头 epoch、一头相对秒**），
    与 `frames[].wall_ns` 的交集**恒空** —— 读出来的正是那句
    "仪器没覆盖到窗口，不是飞机飞得不好"（把判层的错记到飞机头上）。

    这里造一场**本该判得出**的悬停场（记录器钟零点 = `T_ABS0`）：修前 → 0 帧 ⟹ 判不了。
    """
    obs = make_obs()
    obs["t"] = {"offboard_enter": 2.0, "first_frame": 0.0,
                "offboard_enter_t": 2.0, "flight_end_t": 18.0}
    hover = one(b.evaluate_b(obs), "★判据②·轨迹1")
    assert hover["ok"] is True and hover["counts"] is True
    h = obs["traj"]["hover"]
    # ★ 这两条就是变异测试的抓手：修前 `n_frames == 0`、`window == [T_ABS0-…, 18.0]`
    assert h["n_frames"] > 1000 and h["n_truth"] > 1000
    assert h["window"][0] > T_ABS0 and h["window"][1] > T_ABS0


def test_判据2_有窗口但取不到零点_记判不了不计数():
    """取不到零点 ⟹ 「判不了」（`ok=None` + `counts=False`），**不是**不通过。

    ★ 这条钉住"不许把判不了写成不通过"，也钉住"不许退回当作同一根钟"。
    """
    obs = make_obs()
    obs["t"] = {"offboard_enter": 2.0, "offboard_enter_t": 2.0, "flight_end_t": 18.0}
    hover = one(b.evaluate_b(obs), "★判据②·轨迹1")
    assert hover["ok"] is None and hover["counts"] is False
    assert "零点" in hover["detail"]


def test_判据2_traj档取不到零点也记判不了():
    obs = traj_obs()
    del obs["t"]["first_frame"]
    lem = one(b.evaluate_b(obs), "★判据②·轨迹2")
    assert lem["ok"] is None and lem["counts"] is False
    assert "零点" in lem["detail"]


# ── 缺陷 B：「在动」的门槛（位置 → 速度）

def test_参考在动按速度判_停在起点四毫米的不算():
    """★★ 缺陷 B 的靶心：包 5 的 `pre` 段 1500 帧停在 `(0.004136, 0.004135)` —— 离原点
    **4 mm**，比"位置非零"的旧门槛（`1e-6`）大三个量级 ⟹ 那些**静止帧**被算成"在动"，
    `move_first_s` 从 **36.388 s** 掉到 **6.380 s**、`n_moving` 从 610 涨到 2110。
    """
    rows = [{"wall_ns": int((T_ABS0 + 0.00) * 1e9), "phase": "pre", "i": 0,
             "position": [0.004136, 0.004135, 1.0], "velocity": [0.0, 0.0, 0.0], "yaw": 0.0},
            {"wall_ns": int((T_ABS0 + 0.02) * 1e9), "phase": "pre", "i": 1,
             "position": [0.004136, 0.004135, 1.0], "velocity": [0.0, 0.0, 0.0], "yaw": 0.0},
            {"wall_ns": int((T_ABS0 + 0.04) * 1e9), "phase": "bag", "i": 2,
             "position": [0.10, 0.00, 1.0], "velocity": [0.5, 0.0, 0.0], "yaw": 0.0},
            {"wall_ns": int((T_ABS0 + 0.06) * 1e9), "phase": "hold", "i": 3,
             "position": [0.10, 0.00, 1.0], "velocity": [0.0, 0.0, 0.0], "yaw": 0.0}]
    tl = b.ref_timeline_from_rows(rows, T_ABS0)
    assert tl["n"] == 4 and tl["n_moving"] == 1          # ← 按位置判是 3
    assert tl["move_first_s"] == pytest.approx(0.04)
    assert tl["move_span_s"] == pytest.approx(0.0)


def test_参考逐帧缺velocity列当场炸_不退回按位置判():
    """缺列就**炸**：退回位置那一路会把 `pre` 段算成"在动"，是同一个缺陷换张皮。"""
    with pytest.raises(KeyError):
        b.ref_timeline_from_rows(
            [{"wall_ns": int(T_ABS0 * 1e9), "phase": "pre", "i": 0,
              "position": [0.1, 0.0, 1.0]}], T_ABS0)


def test_零点锚点的精度_拿T0的参考日志交叉核对():
    """★ 判层自己推的零点 vs 真值：T0 的参考日志里**藏着**同一根钟的零点
    （`first_s = 参考首帧的绝对墙钟 − t0`）⟹ 正好拿来检验 `rec_clock_zero()` 准不准。

    实测偏差 **4 ms**（= 轮询延迟 + `first_frame` 落盘到 0.01 s）。它界定的是"收手"这种
    **松**界限与参考移动段的边界，4 ms 不改变任何一条读数 —— 但**必须**是可核的，
    否则"零点能从 obs 自己推出来"这句话就没有证据。
    """
    obs = json.loads((ARCHIVE / "T0-summary.json").read_text(encoding="utf-8"))
    rows = [json.loads(l) for l in (ARCHIVE / "T0-replay-frames-reference.jsonl")
            .read_text(encoding="utf-8").splitlines() if l.strip()]
    truth_zero = rows[0]["wall_ns"] * 1e-9 - obs["reference_timeline"]["first_s"]
    assert b.rec_clock_zero(obs["frames"], obs["t"]) == pytest.approx(truth_zero, abs=0.02)


def test_归档T0的参考时间线重算_移动段是36_388到48_565():
    """拿**入库的原始逐帧**重算 —— 与 06 号票离线预演那列"真值"同量级、差 3 ms（取法差）。

    容差取 5 ms：`t0` 是从 `first_frame`（落盘到 0.01 s）推的，再往下传一个数量级就失真了。
    ★ 别把这三行当成"抄预演"—— 它们是**重算出来**的，预演那列（36.385 / 48.562 / 2747 点）
      用的是另一种 `t0` 取法，已在 06 号票 Comments #3 点名作废。
    """
    tl = archive_obs("T0")["reference_timeline"]
    assert tl["n"] == 3770 and tl["by_phase"] == {"pre": 1500, "bag": 1312, "hold": 958}
    assert tl["n_moving"] == 610                          # ← 修前（按位置判）是 2110
    assert tl["move_first_s"] == pytest.approx(36.388, abs=5e-3)
    assert tl["move_last_s"] == pytest.approx(48.565, abs=5e-3)
    assert tl["move_span_s"] == pytest.approx(12.177, abs=5e-3)


# ── 归档离线重判（验收里的变异测试：同一份归档，修前红、修后绿）

def test_归档T0重判_判据2轨迹2从判不了变成通过():
    """★★ 同一份归档 obs（`T0-summary.json` + 落盘原始逐帧）：

    | 场 | 判定行 | 修前 | 修后 |
    |---|---|---|---|
    | `T0` | 判据②·轨迹2 | ⏭️ 不计数（窗口 `[1791527428.111, 48.562]`，0 帧 0 点）| ✅ 计数 |

    ★ 帧数/点数（1218 / 2746）**只有两个缺陷都修掉**才读得出来（窗口换算 + 按速度判"在动"），
      所以把它们钉死在这里：任一处回退，这几个断言会一起红。
      ⚠️ 点数与 06 号票预演那份（2747）差 1，纯粹是 `t0` 取法差 —— 见 `archive_obs` 的 docstring。
    """
    obs = archive_obs("T0")
    checks = b.evaluate_b(obs)
    lem = one(checks, "★判据②·轨迹2")
    assert lem["ok"] is True and lem["counts"] is True
    h = obs["traj"]["lemniscate"]
    assert (h["n_frames"], h["n_truth"]) == (1218, 2746)
    assert h["window"][0] > 1e9 and h["window"][1] > h["window"][0]
    assert "整段落在窗口内=True" in lem["detail"]
    # ★ 别把"修 bug"顺手做成"多计一条"：traj 档里悬停那条仍记「本场不做」
    hover = one(checks, "★判据②·轨迹1")
    assert hover["ok"] is None and hover["counts"] is False


#: 归档五场重判后，判据②·轨迹1 的**实测**判定（修前**五场全是** `⏭️ 不计数`）。
#: ★ `F3` 是**真的不通过**，不是判层的问题：它是唯一一场 **`mass+30` 且注入求解失败**
#:   的（`inject_solve_fail_after_frames=700`）—— 兜底发的悬停推力是**模型**的
#:   （`mpc_node.py:405`），按 +30% 的被控对象不够 ⟹ 飞机在窗口内从 +0.45 m 落回地面
#:   （真值高度区间 `[-0.003, +1.061] m`）⟹ "不撞地=False"。
#:   对照：`F4` 同样注入、但被控对象是**标称**（`mismatch=none`）⟹ ✅ —— 同一处境只
#:   换被控对象，把"兜底机制坏了"与"兜底的推力口径在被控对象上是错的"分开。
#: ⚠️ 这一行 ❌ **修前根本读不出来**（窗口恒空）⟹ 它是本票**新读出来的**一条，
#:   要登记回 06 号票。它不是"修 bug 顺手多计一条"（那条禁令针对的是把"判不了"当"通过"；
#:   这里相反：把**没读过**的真读数读出来）。
_RUN1_EXPECT = {"F0": True, "F1": True, "F2": True, "F3": False, "F4": True}


@pytest.mark.parametrize("tag", sorted(_RUN1_EXPECT))
def test_归档五场重判_悬停那条判得出_轨迹2仍记本场不做(tag):
    """`hold` 档那五场：判据②·轨迹1 从 `⏭️ 不计数` 变成**真判**（逐场见 `_RUN1_EXPECT`），
    而轨迹2 **仍**记「本场不做」（`ok=None` + `counts=False`）—— 别把修 bug 作成多计一条。

    ⚠️ ticket 10 的验收原文写「`F0`–`F4` 本来走**退路**分支、不受影响」—— **不成立**：
      那五份 obs **都带** `offboard_enter_t`（`grep -rl '"offboard_enter_t"' evidence/`
      命中六份、全在本批），走的是**主**分支、与 `T0` 一样判不了。所以它们**受影响**。
    """
    obs = archive_obs(tag)
    checks = b.evaluate_b(obs)
    hover = one(checks, "★判据②·轨迹1")
    assert hover["ok"] is _RUN1_EXPECT[tag] and hover["counts"] is True
    h = obs["traj"]["hover"]
    assert h["n_frames"] > 1000 and h["n_truth"] > 1000
    assert h["window"][0] > 1e9                       # ← 修前窗口是 [1.79e9, 30.3]，交集恒空
    lem = one(checks, "★判据②·轨迹2")
    assert lem["ok"] is None and lem["counts"] is False
    assert "没做" in lem["detail"]


def test_归档F0重判的窗口读数_与登记表逐位对上():
    """`F0` 是 06 号票 Comments #3 那张登记表里的第一场（DUT 1493 帧 / 真值 3442 点）——
    钉死这两个数，表一改这里就红。"""
    obs = archive_obs("F0")
    b.evaluate_b(obs)
    h = obs["traj"]["hover"]
    assert (h["n_frames"], h["n_truth"]) == (1493, 3442)
