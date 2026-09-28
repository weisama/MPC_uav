"""A 档闭环判据的离线单测（ticket 09）—— 不启 SITL、不启 agent。

**为什么必须单开一个文件**：`verify_tier_a_hover.py` 的每一条判据在真跑里要 ~80 s，
而这一轮**恰恰栽在判据本身上**——两处判据写出过"假绿"：

  · ⑤ 的电机推力判据容差 0.08，第一场飞机正以 0.9 m/s 爬升时它**照样过**
    （0.6978 vs 0.6233 = 差 0.0745，卡在容差里），而且没有"是否处于配平"这个前提；
  · ⑧ 的饱和判据写成 `bool(sat_any)` ⟹ 一场**没有饱和**的飞行被判 ❌
    —— 而"没饱和"正是"没顶约束"的证据，判反了。

这两条都是"看起来一片 ✅/❌ 就没人去试"的典型。这个文件把人造数喂进判据层，
让"该炸的时候炸、该过的时候过"变成一毫秒的事。

★ 与 `test_verify_judges.py` 的分工：那个文件测的是 ticket 05 的
  `verify_offboard_smoke.py`（跨档 + 高度）；这里测 A 档闭环**特有的**三件事：
  D6 的冷启动/稳态分账、归因表的**参考取法**分支及其顺序、锚点判据。

（要 `rclpy` / `px4_msgs` / `nmpc`：被测模块导入期就 import 它们。没有环境直接 skip。）
"""

from __future__ import annotations

import importlib.util
import math
import sys
from pathlib import Path

import pytest

pytest.importorskip("rclpy")
pytest.importorskip("px4_msgs")
pytest.importorskip("nmpc")

HERE = Path(__file__).resolve().parent


def _load():
    """按**路径**导入被测脚本（`tools/` 不是一个包）。

    ⚠️ 与 `test_verify_judges._load` 同一条理由：先塞 `sys.modules` 再 `exec_module`，
    否则 `@dataclass` 回表找不到模块对象。

    ★ `sys.path` 要补 `HERE`：被测脚本里有一句 `import verify_offboard_smoke as v`
    （同目录复用世界/记录器），而 pytest 的 rootdir 不在这一层。
    """
    if str(HERE) not in sys.path:
        sys.path.insert(0, str(HERE))
    name = "verify_tier_a_hover"
    if name in sys.modules:
        return sys.modules[name]
    spec = importlib.util.spec_from_file_location(name, HERE / f"{name}.py")
    mod = importlib.util.module_from_spec(spec)
    sys.modules[name] = mod
    spec.loader.exec_module(mod)
    return mod


ta = _load()


# ---------------------------------------------------------------- D6：冷启动/稳态分账


def _frame(i, solve_ms, fallback="normal", sat=None, **kw):
    """一帧够用的逐帧日志（只填 `solve_stats` / `summarise_frames` 真会读的字段）。"""
    f = {"wall_ns": 1_000_000_000 + i * 10_000_000, "solve_ms": solve_ms,
         "fallback": fallback, "sat": [] if sat is None else sat}
    f.update(kw)
    return f


def test_solve_stats_splits_cold_start_from_steady():
    """★★ 本票的第一手：全集最坏 16.123 ms 就是**第 0 拍**（`warmup`），稳态最坏 3.554 ms。

    两个档位**都要**给出来，且 `ladder_steady` 才是 D6 定档的依据（口径见 `solve_stats`
    的 ★★ 与 `docs/control/tier-a-sitl-hover.md`）。
    """
    frames = [_frame(0, 16.123, fallback="warmup")] + \
             [_frame(i, 3.554 if i == 40 else 2.5 + 0.001 * i) for i in range(1, 201)]
    s = ta.solve_stats(frames)
    assert s["n"] == 201
    assert s["max"] == 16.123                      # 全集最坏 = 第 0 拍
    assert s["ladder"] == ta.D6_LADDER[1][1]       # 按票面字面 ⟹ 16.123 > 10 ⟹ 降 50 Hz
    assert s["warmup_n"] == 1
    assert s["cold_ms"] == 16.123 and s["cold_in_warmup"] is True
    assert s["cold_fallback"] == "warmup"
    assert s["steady_n"] == 200 and s["steady_max"] == 3.554
    assert s["ladder_steady"] == ta.D6_LADDER[0][1]   # ★ 稳态 ⟹ 100 Hz（D6 定在它上面）


def test_solve_stats_flags_cold_start_that_is_not_behind_a_warmup_gate():
    """⚠️ **必须报得出来**的那种场面：没有任何 `warmup` 闸门，冷启动落在第 0 拍上。

    这时"稳态"集合里**照样含**那一拍 ⟹ 两个档位一致（16.123 > 10 ⟹ 50 Hz），
    而 `cold_in_warmup=False` 是在说"这 16 ms 是**边发指令边付的**"—— 判据⑦ 拿它把
    "稳态很快"这个结论拦住（否则会把一次真滞后的飞行读成 100 Hz 余量够）。
    """
    frames = [_frame(0, 16.123, fallback="normal")] + \
             [_frame(i, 2.5) for i in range(1, 101)]
    s = ta.solve_stats(frames)
    assert s["cold_in_warmup"] is False
    assert s["cold_ms"] == 16.123 and s["warmup_n"] == 0
    assert s["steady_max"] == 16.123                  # ★ 冷启动没被排掉：稳态里仍然看得见它
    assert s["ladder_steady"] == s["ladder"] == ta.D6_LADDER[1][1]   # 两条都是 50 Hz


def test_solve_stats_is_empty_not_crashing_without_samples():
    assert ta.solve_stats([]) == {"n": 0}
    assert ta.solve_stats([{"wall_ns": 1, "fallback": "warmup"}])["n"] == 0


# ---------------------------------------------------------------- ⑧ 饱和：判的是"记了没有"


def test_sat_logged_means_the_capability_is_in_the_log_not_that_it_fired():
    """☠️ 判据⑧ 曾写成 `bool(sat_any)` ⟹ 一场**没有饱和**的飞行被判 ❌（判反了）。

    票面要的是"日志**记录**约束饱和标志，供飞行后复盘"⟹ 判 `sat` 这一列在不在，
    饱和的**次数**照旧报（`sat_count`）。
    """
    no_sat = [dict(_frame(i, 2.5), sat=[]) for i in range(5)]
    s = ta.summarise_frames(no_sat)
    assert s["sat_any"] == [] and s["sat_logged"] is True
    assert s["sat_count"] == {}

    fired = [dict(_frame(i, 2.5), sat=["roll"]) for i in range(3)] + [dict(_frame(9, 2.5), sat=[])]
    s = ta.summarise_frames(fired)
    assert s["sat_any"] == ["roll"] and s["sat_count"] == {"roll": 3} and s["sat_logged"] is True

    never_logged = [{k: val for k, val in _frame(i, 2.5).items() if k != "sat"} for i in range(5)]
    assert ta.summarise_frames(never_logged)["sat_logged"] is False


# ---------------------------------------------------------------- 归因表：参考取法


def _obs(frames=200, offboard=3.0, track=None):
    return {"name": "人造", "source": "px4",
            "t": {"offboard_enter": offboard},
            "dut": {"frames": frames, "track": track if track is not None else {"ref_n": frames}},
            "truth": [], "hover_s": 3.0}


def _ck(name, ok):
    return {"name": name, "ok": ok, "detail": "", "counts": True}


def test_attribute_blames_the_reference_convention_before_sign_mapping():
    """★★ 本轮实测：参考跟着测量走时，**②（符号映射）也会跟着红**（飞机一直在爬）。

    归因若按 ②→⑥→③ 的顺序走，会一路读到"模型"。所以"参考取法"要排在②之前，
    并且**说的是票面三条之外的那一条**（票面那三条盖不住它）。
    """
    checks = [_ck(ta.CK_ANCHOR + "：目标全程没跟着测量走", False),
              _ck("② 行为：真值高度真的升上去了", False),
              _ck("③ 悬停高度 1 m", False)]
    text = ta.attribute(_obs(), checks)
    assert text.startswith("**参考取法"), text
    assert "0.9 m/s" in text and "hold_reference" in text
    # ★ 不许把票面那三条的名字当结论说出来 —— 那正是这一轮被误导的地方
    assert "**模型**" not in text and "**符号映射" not in text


def test_attribute_keeps_middleware_above_the_reference_branch():
    """帧数不够时 `ref_span` 也是 None ⟹ 锚定那条判据**同样会红**（"没测到"≠"没跟着走"）。

    那种场面要先说中间件；`attribute` 里那句 `ref_n >= 10` 的门槛就是为它设的。
    """
    checks = [_ck(ta.CK_ANCHOR + "：目标全程没跟着测量走", False)]
    assert ta.attribute(_obs(frames=0, offboard=None), checks).startswith("**中间件")
    # 帧太少（ref_n=3）⟹ 那条判据红的是"没测到"，不许读成"参考没锚住"
    assert not ta.attribute(_obs(track={"ref_n": 3}), checks).startswith("**参考取法")
    # ref_n 够大时，同一条判据才轮到说"参考取法"
    assert ta.attribute(_obs(), checks).startswith("**参考取法")


def test_attribute_still_says_模型_when_only_the_controller_is_left():
    """反例（防"新分支把所有红都吃掉"）：锚定过了、方向也对、就是对不上 ⟹ 还是"模型"。"""
    checks = [_ck(ta.CK_ANCHOR + "：目标全程没跟着测量走", True),
              _ck("② 行为：真值高度真的升上去了", True),
              _ck("③ 悬停高度 1 m", False)]
    assert ta.attribute(_obs(), checks).startswith("**模型")


def test_attribute_calls_a_clean_run_clean():
    checks = [_ck(ta.CK_ANCHOR + "：目标全程没跟着测量走", True)]
    assert "归因表不适用" in ta.attribute(_obs(), checks)


# ---------------------------------------------------------------- ⑨ 偏航（真值锚点）


def _quat_from_yaw_enu(psi):
    return [math.cos(psi / 2.0), 0.0, 0.0, math.sin(psi / 2.0)]


def _quat_from_yaw_ned(psi):
    """NED/FRD 下"只绕 z 朝下转 ψ"的四元数（分量形式与 ENU 那条一样，**含义不同**）。"""
    return [math.cos(psi / 2.0), 0.0, 0.0, math.sin(psi / 2.0)]


def _yaw_case(psi_seq, cmd_scale):
    """造一组配对样本：真值机头按 `psi_seq`（NED）转，节点侧按**教科书关系**换算。

    `cmd_scale = −1` ⟹ 指令方向正确（`ψ̇ = −ω_z`）；`+1` ⟹ 指令反了（判据⑨ 该红）。
    """
    pairs = []
    for k, psi in enumerate(psi_seq):
        pairs.append({"t": float(k), "psi_ned": psi,
                      "yaw_node": ta.wrap_pi(math.pi / 2.0 - psi),
                      "yaw_cmd": cmd_scale * (psi - psi_seq[0])})
    return pairs


def test_yaw_map_judge_accepts_a_consistent_sign_and_rejects_an_inverted_one():
    """★★ 判据⑨a：真值锚点 `dψ_ned/dt == −ω_z_FLU`（`sign-map.md` §5.2）。

    这条**不经过我们的换算**（ψ 从真值四元数按 PX4 自己的定义取），所以它能单独判
    "指令方向"。造 +8 rad 的累计转角：`cmd_scale = −1` 该过、`+1` 该炸。
    """
    psi = [0.1 * k for k in range(80)]                    # 累计 +7.9 rad
    ok = ta.yaw_verdict(_yaw_case(psi, -1.0))
    assert abs(ok["psi_span"] - 7.9) < 1e-6 and abs(ok["resid"]) < 1e-6   # 残差 ≈ 0
    bad = ta.yaw_verdict(_yaw_case(psi, +1.0))
    assert abs(bad["resid"]) > 10.0                       # 同号 ⟹ 残差 ≈ 2×7.9
    assert abs(bad["resid"]) > 0.25 * abs(bad["cmd_span"]) + 0.35         # 判据会红


def test_yaw_conversion_judge_catches_a_flipped_yaw():
    """判据⑨b：`π/2 − ψ_ned` == 节点换算出来的 ENU 机头朝向。

    换算 yaw 反了 ⟹ 误差 ≈ 2ψ（随航向线性变、跨度大），而**位置环照样可能正常** ——
    这正是它在 2026-09-28 那场里没被任何判据抓到、只能靠真值姿态抓的原因。
    """
    psi = [0.1 * k for k in range(80)]
    good = ta.yaw_verdict(_yaw_case(psi, -1.0))
    assert abs(good["conv_err_med"]) < 1e-6 and good["conv_err_span"] < 1e-6
    flipped = _yaw_case(psi, -1.0)
    for p in flipped:                                     # 把换算那半边取反
        p["yaw_node"] = ta.wrap_pi(-p["yaw_node"])
    bad = ta.yaw_verdict(flipped)
    assert bad["conv_err_span"] > 1.0                     # 跨度随航向线性涨 ⟹ 判据会红
    assert abs(bad["conv_err_med"]) > 0.15


def test_yaw_verdict_says_nothing_when_it_has_no_samples():
    assert ta.yaw_verdict([]) == {"n": 0}
    assert "conv_err_med" not in ta.yaw_verdict([{"t": 0, "psi_ned": 0, "yaw_node": 0,
                                                  "yaw_cmd": 0}])


def test_yaw_pairs_pairs_truth_to_the_nearest_frame_by_wall_clock():
    """配对走**墙钟**，不走"两边的时间戳"。

    ☠️ 第一版按时间戳配 ⟹ 采到 25 点、配出 **0** 对（判据⑨ 变成"没做"）。原因是一手事实：
      控制台 `listener` 打的是 PX4 原始 `hrt`（开机起算，实测 18.7e6 µs），而帧日志的 `ts`
      是 DDS 桥出来的**Unix 纪元** µs（实测 1.79e15）—— 都是 µs，零点差 56 年。
    """
    frames = [{"ts": 1_790_612_483_272_734 + i * 10_000, "wall_ns": int((100.0 + i * 0.01) * 1e9),
               "u": [7.0, 0.0, 0.0, 0.5],
               "x0": [0, 0, 1, 0, 0, 0, *_quat_from_yaw_enu(0.0)]} for i in range(101)]
    gt = [{"veh_us": 18_684_000 + i * 100_000, "wall_epoch": 100.0 + i * 0.1, "t": float(i),
           "q": _quat_from_yaw_ned(0.0)} for i in range(11)]
    pairs = ta.yaw_pairs(frames, gt, tol_s=0.25)
    assert len(pairs) == 11
    # 帧 u[3] = 0.5 rad/s、每对间隔 0.1 s ⟹ 累计 0.5 × 0.1 × k
    assert abs(pairs[10]["yaw_cmd"] - 0.5) < 1e-6
    # 真值 ψ_ned = 0 ⟹ 节点侧该看到 π/2；这里的 `x0` 是"只绕 z 转 0" ⟹ `yaw_node` = 0
    assert abs(pairs[0]["yaw_node"]) < 1e-9


def test_yaw_pairs_refuses_to_guess_when_the_evidence_is_incomplete():
    """缺 `u` / 缺 `x0` / 配不上时间 ⟹ **返回空**，而不是拿半截数据凑一行出来。

    ★ 这条纪律的理由与 `track()` 丢掉残缺样本一样：判据⑨ 的结论是"符号映射对不对"，
      拿半截数据算出来的 `∫u[3]dt` 会是一个**看起来很像真的**错数。
    """
    gt = [{"veh_us": 5_000_000, "wall_epoch": 42.0, "t": 0.0, "q": [1, 0, 0, 0]}]
    frames = [{"ts": 5_000_000, "wall_ns": int(42.0 * 1e9), "u": [7.0, 0, 0, 0.0],
               "x0": [0, 0, 1, 0, 0, 0, 1, 0, 0, 0]}]
    assert len(ta.yaw_pairs(frames, gt)) == 1
    del frames[0]["u"]
    assert ta.yaw_pairs(frames, gt) == []                  # 没有指令 ⟹ 无从判方向
    frames[0]["u"] = [7.0, 0, 0, 0.0]
    del frames[0]["x0"]
    assert ta.yaw_pairs(frames, gt) == []                  # 没有 `x0` ⟹ 无从判换算
    frames[0]["x0"] = [0, 0, 1, 0, 0, 0, 1, 0, 0, 0]
    # 时间差超出容差 ⟹ 不配对（`tol_s` 是硬门槛，不是"就近取一个"）
    assert ta.yaw_pairs(frames, [{"veh_us": 9_000_000, "wall_epoch": 99.0, "t": 0.0,
                                  "q": [1, 0, 0, 0]}]) == []


def test_payload_rows_carries_the_pairing_key_so_judge_9_can_actually_pair():
    """★ 回归（这条是**真出过**的）：`payload_rows` 必须把 `wall_epoch` 带出来。

    2026-09-28：它当时只输出 `{t, veh_us, <keys>}`，而 `pair_by_time` 要的是 `wall_epoch`
    （两个出口的时间基不同，见那边的 ⚠️）⟹ 采到的 33 个姿态真值样本**一个不剩**地被跳过，
    判据⑨ 报成"本场**没采到**姿态真值"。**采到了却配不上**与**没采到**在报告里长得一样，
    所以这条测试**从真正的生产者出发** —— 手搭样本字典的测试永远碰不到这个 bug
    （旧的 8 条判据⑨ 用例全是手搭的，就是这么漏过去的）。

    ★ 顺带钉住第二件事：没有配对键的行算**残缺样本**（`partial`），不当完整记录发出去。
    """
    s = ta.TruthSampler(None, "t", None, fields=(), arrays=("q",))
    s.samples = [{"veh_us": 5_000_000, "wall": 1.0, "wall_epoch": 42.0, "q": [1, 0, 0, 0]},
                 {"veh_us": 5_100_000, "wall": 1.1, "wall_epoch": 42.1, "q": [1, 0, 0, 0]},
                 {"veh_us": 5_200_000, "wall": 1.2, "q": [1, 0, 0, 0]}]      # ← 缺配对键
    rows, meta = s.payload_rows(("q",))
    assert len(rows) == 2 and meta["partial"] == 1
    assert all(isinstance(r["wall_epoch"], float) for r in rows)
    # 端到端：生产者吐出来的行，真的要配得上帧（这一条才是"判据⑨ 的原料接上了"）
    frames = [{"ts": 5_000_000, "wall_ns": int(42.0 * 1e9), "u": [7, 0, 0, 0.0],
               "x0": [0, 0, 1, 0, 0, 0, 1, 0, 0, 0]},
              {"ts": 5_100_000, "wall_ns": int(42.1 * 1e9), "u": [7, 0, 0, 0.0],
               "x0": [0, 0, 1, 0, 0, 0, 1, 0, 0, 0]}]
    assert len(ta.yaw_pairs(frames, rows)) == 2


def test_pairing_failures_are_counted_not_silently_swallowed():
    """配对失败要**记账**：判据必须能分开「没有数据」与「仪器坏了」。

    ★ 四个计数器对应四条真会发生的故障，修法各不相同（看 DUT 帧日志 / 看采样器 /
      看 `u` 列 / 看 `x0` 列）—— 合并成一个"配不上"就又把它们糊回一起了。
    """
    fr = [{"ts": 5_000_000, "wall_ns": int(42.0 * 1e9), "u": [7, 0, 0, 0.0],
           "x0": [0, 0, 1, 0, 0, 0, 1, 0, 0, 0]}]
    d = {}
    assert ta.pair_by_time([{"t": 0.0, "q": [1, 0, 0, 0]}], ("q",), fr, diag=d) == []
    assert (d["no_wall_epoch"], d["n_out"]) == (1, 0)          # 样本那侧没有配对键
    d = {}
    assert ta.pair_by_time([{"t": 0.0, "wall_epoch": 99.0, "q": [1, 0, 0, 0]}], ("q",),
                           fr, diag=d) == []
    assert d["too_far"] == 1                                   # 时间差超容差
    d = {}
    assert ta.pair_by_time([{"t": 0.0, "wall_epoch": 42.0, "q": [1, 0, 0, 0]}], ("q",),
                           [{"ts": 1}], diag=d) == []
    assert d["frames_no_stamp"] == 1                           # 帧那侧没有绝对墙钟


def test_yaw_pairs_records_which_ingredient_was_missing():
    """`yaw_pairs` 的四条"返回空"的路要各自留痕（`u_missing` / `x0_missing`）。"""
    gt = [{"veh_us": 5_000_000, "wall_epoch": 42.0, "t": 0.0, "q": [1, 0, 0, 0]}]
    fr = [{"ts": 5_000_000, "wall_ns": int(42.0 * 1e9), "x0": [0, 0, 1, 0, 0, 0, 1, 0, 0, 0]}]
    d = {}
    assert ta.yaw_pairs(fr, gt, diag=d) == []
    assert (d["n_gt"], d["u_missing"]) == (1, 1)               # 帧里没有 `u` ⟹ 无从判方向
    fr[0]["u"] = [7.0, 0, 0, 0.0]
    del fr[0]["x0"]
    d = {}
    assert ta.yaw_pairs(fr, gt, diag=d) == []
    assert d["x0_missing"] == 1                                # 帧里没有 `x0` ⟹ 无从判换算


def test_ned_yaw_of_a_pure_yaw_quaternion_is_the_angle_itself():
    """`yaw_ned_from_quat` 是判据⑨ 的**独立参照**，它自己先得对。

    ★ 用物理锚点对账：ψ = +90° 的 NED/FRD 四元数把机体 x 轴（前）转到 NED 的 +y（东）
    —— 这是 PX4 自己的定义（ψ 绕 z_down、北→东为正）。代数形式与 `node_core.yaw_from_quat`
    相同，但**输入是不同的一路**（那边吃的是换算过的 ENU/FLU），所以能当独立参照。
    """
    for psi in (0.0, 0.5, math.pi / 2.0, -1.2, 3.0):
        assert abs(ta.yaw_ned_from_quat(_quat_from_yaw_ned(psi)) - psi) < 1e-12


def test_unwrap_and_wrap_pi_are_inverse_enough_for_a_five_turn_spin():
    """实测那场是 **5 圈**（31 rad）：`unwrap` 必须在超过 ±π 累加时不丢圈数。"""
    seq = [ta.wrap_pi(0.1 * k) for k in range(320)]
    u = ta.unwrap(seq)
    assert abs(u[-1] - 31.9) < 1e-9
    assert abs(ta.wrap_pi(6 * math.pi + 0.3) - 0.3) < 1e-9


# ---------------------------------------------------------------- ⑨ 别名闸门（假 ❌ 的教训）


def test_yaw_verdict_refuses_to_judge_when_the_truth_samples_alias():
    """★★ 别名闸门：2026-09-28 那场转 **4.9 rad/s**，而真值只有 1 Hz ⟹ 两个样本之间转过
    4.9 rad **> π**，`unwrap` 走最短路径把它记成 −1.38 ⟹ 累计转角被记少三倍半**且方向相反**，
    判据⑨a 于是报"指令让飞机往反方向转"，把归因写成**符号映射** —— 而符号映射是对的
    （9b 的换算误差中位数只有 −0.031 rad）。这一条就是那个假 ❌ 的回归。

    闸门的原料是**稠密帧**测出来的转速上界（`yaw_rate_bound`）；这里直接喂进去。
    """
    rate = 4.9
    psi = [ta.wrap_pi(rate * k) for k in range(12)]           # 1 Hz 采样，每步 4.9 rad
    pairs = [{"t": float(k), "psi_ned": p,
              "yaw_node": ta.wrap_pi(math.pi / 2.0 - p),
              "yaw_cmd": -rate * k} for k, p in enumerate(psi)]  # 指令是对的（ψ̇ = −ω_z）
    naive = ta.yaw_verdict(pairs)                             # 没有转速参考 ⟹ 闸门不开
    assert abs(naive["resid"]) > 0.25 * abs(naive["cmd_span"]) + 0.35   # 那条假 ❌ 就是它
    assert abs(naive["psi_span"]) < 0.4 * rate * (len(psi) - 1)          # 转角被记少
    gated = ta.yaw_verdict(pairs, rate_hint=rate)
    assert gated["alias"] is True
    assert gated["dt_sample"] == 1.0
    assert gated["alias_turn"] > 4.0                          # 4.9 rad ≥ 0.9π ⟹ 解不出来
    # ★ ⑨b（换算）**不闸**：它是逐样本比出来的，不做真值序列的展开 ⟹ 转得快时照样可判
    #   （实测：那场 4.9 rad/s 的 ⑨b 读数中位数 −0.064 rad、跨度 0.138 rad —— 干净的）
    assert isinstance(gated["conv_err_med"], float)
    # ★ 转速参考缺失时闸门**不开**（判不了 ≠ 不通过）：一次仪器缺失不该变成一张空白报告
    assert naive["alias"] is None and ta.yaw_verdict(pairs, rate_hint=None)["alias"] is None


def test_yaw_rate_bound_takes_the_worst_of_both_dense_signals():
    """闸门原料：100 Hz 帧的 `x0` 机头序列与 `u[3]` 峰值，取**较大者**（保守）。

    ★ 只用**转角大小** ⟹ 换算方向反了不影响它；`u[3]` 那一路连符号映射整个反了都还是对的
      （幅值不变）⟹ 两路都不与判据⑨b 循环论证。
    ★ 帧少到算不出转速时返回 `None`（"闸门不开"），**不返回 0**（那会被读成"没转"）。
    """
    def frames(dt_s, rate, u3=0.0, n=40):
        out = []
        for k in range(n):
            q = _quat_from_yaw_enu(rate * k * dt_s)
            out.append({"wall_ns": int(round(k * dt_s * 1e9)),
                        "x0": [0.0] * 6 + q, "u": [7.0, 0.0, 0.0, u3]})
        return out

    assert ta.yaw_rate_bound(frames(0.01, 4.9)) == pytest.approx(4.9, abs=0.02)
    assert ta.yaw_rate_bound(frames(0.01, -0.8)) == pytest.approx(0.8, abs=0.02)
    # 帧里"没转"但指令在转 ⟹ 上界取指令那一路（反过来也一样）
    assert ta.yaw_rate_bound(frames(0.01, 0.0, u3=-4.696)) == pytest.approx(4.696)
    assert ta.yaw_rate_bound(frames(0.01, 2.0, u3=0.3)) == pytest.approx(2.0, abs=0.02)
    assert ta.yaw_rate_bound([]) is None
    assert ta.yaw_rate_bound([{"wall_ns": 1, "x0": [0.0] * 10}]) is None


def test_alias_gate_turns_a_false_sign_map_verdict_into_not_done():
    """闸门起作用后，**归因**不许再提"符号映射"（那是 2026-09-28 那条错结论）。

    这条走判据表的真实入口：⑨a 被降级成 `ok=None`（不计数）⟹ `attribute` 里
    `failed(CK_YAW_MAP)` 收不到它，于是归因落到 ⑥b（代价/前馈）那一条 —— 那才是真凶。
    ★ ⑨b 仍然**计数**（它是逐样本比出来的，不展开真值序列）⟹ 这里给它 ✅，
      它不该、也不会把归因抢走。
    """
    checks = [_ck(ta.CK_ANCHOR + "：目标全程没跟着测量走", True),
              _ck("② 行为：真值高度真的升上去了", True),
              _ck(ta.CK_YAW_MAP + "：指令方向与被控对象真的转的方向一致", None),
              _ck(ta.CK_YAW_CONV + "：我们换算出来的机头朝向 == 教科书那一个", True),
              _ck("⑥b 悬停段机头保持（EKF 帧）：末段机头转速 ≈ 0 且全程没转圈", False)]
    text = ta.attribute(_obs(), checks)
    assert text.startswith("**代价 / 前馈"), text
    assert "上一帧解" in text and "qw = 0" in text
    # ★★ 2026-09-28 复核抓出来的：⑨a 是 `ok=None`（**没做**）时，
    #   归因**不许**说"⑨ 已经证明两个方向都没反" —— 那一场方向根本没被量到。
    assert "没有被排除" in text, text
    assert "已经证明" not in text and "真的判到了方向" not in text
    # ★ 归因落在 ⑥b（代价/前馈）上 ⟹ 它**不该**自称"符号映射"（那是被推翻的那条结论）
    assert "**符号映射" not in text


# ------------------------------------------------- ⑨a 的判别力（"✅"≠"证实了方向"）


def test_yaw_verdict_reports_whether_the_cumulative_angle_can_judge_direction():
    """★★ `✅` 有两种：**证实了方向** 与 **没矛盾**。判据本身分不出来，得另出一位。

    判据是 `|resid| < 0.25·|cmd_span| + 0.35`；"指令反了"那个世界给的是
    `|psi_span − cmd_span| ≈ 2·|cmd_span|`（健康时 `|psi_span| ≈ |cmd_span|`）
    ⟹ 只有 `|cmd_span| > 0.2` 时那条判据**才可能**变红（推导见 `YAW_POWER_RAD`）。

    ★ 两个数都是**第一手读数**：第 8 场悬停得好、`∫u[3]dt = −0.0022` ⟹ 就算符号整个反了
      也只有 0.086，照样 ✅；第 5 场飞机在 4.9 rad/s 上转、`∫u[3]dt = +7.94` ⟹ 反了会给出 −15.7。
    """
    # 第 8 场那一头的量级：悬停得好，指令累计转角接近 0
    hover = [{"t": float(k), "psi_ned": 0.01 * k,
              "yaw_node": math.pi / 2.0 - 0.01 * k, "yaw_cmd": -0.0001 * k}
             for k in range(21)]
    v = ta.yaw_verdict(hover)
    assert ta.yaw_had_power(v) is False
    assert abs(v["cmd_span"]) < ta.YAW_POWER_RAD
    # ★ 判别力不足**不改变通过与否**：这条读数照样是过的那一半
    assert abs(v["resid"]) < 0.25 * abs(v["cmd_span"]) + 0.35
    # 把 yaw 的符号整个反掉（判据该红的那一半）—— 在这里它**照样绿**，这正是"没劲"的含义
    flipped = [dict(p, yaw_cmd=-p["yaw_cmd"]) for p in hover]
    assert abs(ta.yaw_verdict(flipped)["resid"]) < 0.25 * abs(v["cmd_span"]) + 0.35

    # 第 5 场那一头的量级：真在转 ⟹ 有劲
    spin = [{"t": float(k), "psi_ned": -0.33 * k,
             "yaw_node": math.pi / 2.0 + 0.33 * k, "yaw_cmd": 0.34 * k}
            for k in range(24)]
    s = ta.yaw_verdict(spin)
    assert ta.yaw_had_power(s) is True
    assert abs(s["cmd_span"]) > ta.YAW_POWER_RAD
    assert abs(s["resid"]) < 0.25 * abs(s["cmd_span"]) + 0.35
    # 反了的世界：残差 ≈ 2|cmd_span| ⟹ 判据会红（这就是"有劲"）
    inv = [dict(p, yaw_cmd=-p["yaw_cmd"]) for p in spin]
    iv = ta.yaw_verdict(inv)
    assert abs(iv["resid"]) > 0.25 * abs(iv["cmd_span"]) + 0.35


def _yaw_obs(cmd_span, alias=None):
    """造一份能喂给 `attribute` 的 obs（只带 ⑥b 分支要读的那几个键）。

    ★ 刻意**不塞 `discriminating` 位** —— 判别力是从 `cmd_span` 现算的
      （`yaw_had_power`），旧观测（本票之前记下来的那些）里没有那一位，
      而 `--replay` 要能重放它们。
    """
    o = _obs()
    o["yaw"] = {"n": 20, "cmd_span": cmd_span, "alias": alias}
    return o


def _six_b_checks():
    return [_ck(ta.CK_ANCHOR + "：目标全程没跟着测量走", True),
            _ck("② 行为：真值高度真的升上去了", True),
            _ck(ta.CK_YAW_MAP + "：指令方向与被控对象真的转的方向一致", True),
            _ck(ta.CK_YAW_CONV + "：我们换算出来的机头朝向 == 教科书那一个", True),
            _ck("⑥b 悬停段机头保持（EKF 帧）：末段机头转速 ≈ 0 且全程没转圈", False)]


def test_attribute_only_claims_the_direction_was_proved_when_judge_9_had_power():
    """★★ 同一条 ⑥b 分支，三种口径**必须说出不一样的话**（复核抓出来的第三种）。

    · 判别力够（第 5 场那种）⟹ 可以说"真的判到了方向"；
    · 判别力不足（第 8 场那种：悬停得好、`∫u[3]dt ≈ 0`）⟹ 只能说"没矛盾"，方向**另证**；
    · ⑨a 被别名闸掉 ⟹ **没做**，方向**根本没被排除**（原版在这里照样说"已证明"，说满了）。
    """
    strong = ta.attribute(_yaw_obs(7.94), _six_b_checks())
    assert "真的判到了方向" in strong and "不是符号映射" in strong
    weak = ta.attribute(_yaw_obs(-0.0022), _six_b_checks())
    assert "判别力不足" in weak and "另证" in weak
    assert "真的判到了方向" not in weak
    gated = ta.attribute(_yaw_obs(53.95, alias=True), _six_b_checks())
    assert "没有被排除" in gated
    assert "真的判到了方向" not in gated and "已经证明" not in gated
    # ★ 三条都必须仍然指向真凶（别为了说清口径把归因本身弄丢）
    for t in (strong, weak, gated):
        assert t.startswith("**代价 / 前馈"), t
        assert "上一帧解" in t and "qw = 0" in t
        assert "**符号映射" not in t
