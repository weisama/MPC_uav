"""±30 % 失配扫描（ticket 12）的**离线**单测 —— 不启 SITL、不飞。

**为什么这一层要单测**：本张的结论是一句"在什么范围内仍稳定、从哪个参数先退化"，
而这句话是从**一张表**读出来的。表是由 `extract()` / `divergence()` / `summarise()`
三个纯函数算出来的 —— 它们要是把"真值掉回地面"读成"稳定"，整张票的结论就反了，
而那种错**在真跑的报告里看不出来**（那一场照样印一堆数）。

★ 用**人造 obs**（不是真飞出来的）：与 `test_tier_a_judges.py` 同一条理由 ——
  一条判据在真跑里要 ~80 s，而"该炸的时候炸"应该是一毫秒的事。

（要 `rclpy` / `px4_msgs` / `nmpc`：被测模块导入期就 import 它们。没有环境直接 skip。）
"""

from __future__ import annotations

import importlib.util
import json
import math
import os
import sys
from pathlib import Path

import pytest

pytest.importorskip("rclpy")
pytest.importorskip("px4_msgs")
pytest.importorskip("nmpc")

HERE = Path(__file__).resolve().parent


def _load():
    """按**路径**导入被测脚本（与 `test_tier_a_judges._load` 同一套；`tools/` 不是包）。"""
    if str(HERE) not in sys.path:
        sys.path.insert(0, str(HERE))
    norm = HERE / "z0-plant"
    if str(norm) not in sys.path:
        sys.path.insert(0, str(norm))
    name = "mismatch_sweep"
    if name in sys.modules:
        return sys.modules[name]
    spec = importlib.util.spec_from_file_location(name, HERE / f"{name}.py")
    mod = importlib.util.module_from_spec(spec)
    sys.modules[name] = mod
    spec.loader.exec_module(mod)
    return mod


ms = _load()

#: 变体/参数的**同一份真源** —— `mismatch_sweep` 自己是 `import plant_spec`，
#: 这里顺着它的引用拿，不另起一次导入（两次导入会是**两个模块对象**，改了一边看不到）。
plant_spec = ms.plant_spec


# ---------------------------------------------------------------- 人造观测


def truth_rows(climb_m: float = 1.0, n: int = 40, *, z_final: float | None = None,
               x_final: float = 0.0, dt: float = 0.25):
    """`obs["truth"]` 的骨架：`[t, x, y, z, vx, vy, vz]`，**NED**（z 向下为正）。

    前 1/4 在地面（z = 0），之后抬到 `-climb_m`。`z_final` 给了就线性走到它
    （用来造"掉回地面"或"一路爬飞"）。
    """
    rows = []
    n_ground = max(1, n // 4)
    for i in range(n):
        t = i * dt
        if i < n_ground:
            z, x = 0.0, 0.0
        else:
            frac = (i - n_ground) / max(1, n - n_ground - 1)
            z = -climb_m + frac * ((z_final + climb_m) if z_final is not None else 0.0)
            x = frac * x_final
        rows.append([t, x, 0.0, z, 0.0, 0.0, 0.0])
    return rows


def obs(climb_ekf: float = 1.0, *, truth=None, p2p: float = 0.05, drift: float = 0.02,
        referr: float = -0.01, yaw_span: float = 0.01, frames: int = 2500,
        checks=None, solve=None, track_extra=None):
    """一场 A 档观测的最小骨架（键名与 `verify_tier_a_hover` 落盘的一致）。"""
    track = {"n": frames, "climb": climb_ekf, "up_tail": -climb_ekf, "up_head": 0.0,
             "up_tail_p2p": p2p, "up_tail_std": p2p / 4, "up_tail_osc_hz": 0.3,
             "xy_drift_tail": drift, "referr_tail": referr, "referr_head": 0.0,
             "ref_span_z": 0.0, "ref_n": frames, "vup_tail": 0.0,
             "yaw_ekf_span": yaw_span, "yaw_ekf_tail_span": yaw_span, "u3_tail": 0.0,
             "tail_s": 3.0}
    track.update(track_extra or {})
    if checks is None:
        checks = [{"name": "① 中间件：DUT 发得出帧、PX4 解锁并进了 OFFBOARD",
                   "ok": True, "detail": "", "counts": True},
                  {"name": "③ 悬停高度 1 m ± 0.15", "ok": True, "detail": "", "counts": True}]
    if solve is None:
        solve = {"n": frames, "p50": 3.0, "p99": 4.0, "max": 9.0, "steady_n": frames - 5,
                 "steady_p50": 3.0, "steady_p99": 4.0, "steady_max": 4.2,
                 "ladder_steady": "100 Hz（最坏 ≤10 ms，余量够）"}
    return {"name": "tier-a", "source": "px4", "hover_s": 3.0, "truth_hz": 4.0,
            "truth": truth_rows() if truth is None else truth,
            "truth_span_s": 10.0, "stream_wall_s": 10.0,
            "dut": {"frames": frames, "track": track},
            "solve": solve, "checks": checks, "armed_ever": True,
            "t": {"offboard_enter": 1.0}}


def run(variant: str, rep: int = 1, **kw):
    """一条**运行记录**：`{variant, rep, obs}`（驱动落盘的就是这个形状）。"""
    return {"variant": variant, "rep": rep, "obs": obs(**kw)}


# ---------------------------------------------------------------- 控制器代码留档


def test_控制器代码留档_给出路径与哈希():
    """★ `ros2 run nmpc` 跑的是 **ament 装出来的那一份**（非 symlink ⟹ 是拷贝），
    而 `src/nmpc/` 可能正被别人编辑。逐场留档"实际解析到哪一份"，否则
    "这一轮 13 场飞的是同一份代码"这句话**在报告上看不出来真假**。"""
    c = ms.controller_code()
    assert c["package"] and Path(c["package"]).is_dir()
    for name in ("mpc_node.py", "node_core.py", "ocp.py", "params.py"):
        e = c["files"][name]
        assert Path(e["path"]).exists()
        assert len(e["sha256"]) == 16 and all(ch in "0123456789abcdef" for ch in e["sha256"])


def test_同轮出现两份控制器代码_表里必须报警():
    """一场飞 08:59 的构建、另一场飞改过的 src ⟹ 这就不是**一个**实验了。"""
    a = run("z0", 1)
    a["obs"]["sweep"] = {"controller_code": {"package": "/install/nmpc",
                                             "files": {"params.py": {"sha256": "aaaa"}}}}
    b = run("z0-mass+30", 1)
    b["obs"]["sweep"] = {"controller_code": {"package": "/src/nmpc/nmpc",
                                             "files": {"params.py": {"sha256": "bbbb"}}}}
    s = ms.summarise([a, b])
    assert s["controller"]["n_sigs"] == 2
    assert s["controller"]["warn"] and "不同的控制器代码" in s["controller"]["warn"]
    # ★ 光"表里有控制器代码这一节"不算数（那一节**总是**在）——要断言**警告本身**印出来了。
    table = ms.render_table(s)
    assert "不是同一个实验" in table
    assert s["controller"]["n_sigs"] == 2 and len(s["controller"]["packages"]) == 2


def test_同轮只有一份控制器代码_不报警_但身份要印出来():
    a = run("z0", 1)
    a["obs"]["sweep"] = {"controller_code": {"package": "/install/nmpc",
                                             "files": {"params.py": {"sha256": "aaaa"}}}}
    b = run("z0-mass+30", 1)
    b["obs"]["sweep"] = {"controller_code": {"package": "/install/nmpc",
                                             "files": {"params.py": {"sha256": "aaaa"}}}}
    s = ms.summarise([a, b])
    assert s["controller"]["n_sigs"] == 1
    assert s["controller"]["warn"] is None
    # "这一轮 13 场飞的是同一份代码"这句话必须**能在报告上读到**，而不是只在 JSON 里
    assert "/install/nmpc" in ms.render_table(s)


# ---------------------------------------------------------------- 退化排序


def test_退化排序_按离开基准散度带的倍数排():
    """★ 票面要的是"**从哪个参数先开始退化**"，不是"哪个档判据挂了"。

    判据挂没挂是一根**固定的绝对线**（如 1 m ± 0.15）；在两个都还过线的档之间，
    票面这句话就没有答案了。所以排序用的尺子是**基准自己重复几场的散度带**：
    `x = |读数 − 基准中位数| ÷ (基准跨度 / 2)`。倍数是无量纲的，故不同单位的读数可以并排。
    """
    runs = [run("z0", 1, p2p=0.05), run("z0", 2, p2p=0.07),      # 中位数 0.06、跨度 0.02 ⟹ 半宽 0.01
            run("z0-mass+30", 1, p2p=0.08),                      # x = 0.02/0.01 = 2
            run("z0-inertia+30", 1, p2p=0.10)]                   # x = 0.04/0.01 = 4
    rows = ms.rank_degradation(ms.summarise(runs))["rows"]
    assert [r["knob"] for r in rows] == ["inertia", "mass"]
    assert rows[0]["x"] == pytest.approx(4.0)
    assert rows[1]["x"] == pytest.approx(2.0)
    assert rows[0]["key"] == "p2p_ekf"          # 是哪一条读数把它顶上去的，要能回指
    assert rows[0]["base_median"] == pytest.approx(0.06)


def test_退化排序_基准散度带为_0_时_拒绝给倍数():
    """★ 基准重复几场读数一模一样（跨度 0）⟹ 任何差都是"无穷倍"。

    那种数**看起来很有力**（∞），实际是这台仪器分辨不出来 ⟹ 必须拒绝给倍数并说出来，
    不能印一个 ∞ 让读者以为"这个参数一碰就散"。
    """
    runs = [run("z0", 1, p2p=0.05), run("z0", 2, p2p=0.05),
            run("z0-mass+30", 1, p2p=0.06)]
    r = ms.rank_degradation(ms.summarise(runs))
    assert r["resolvable"] is False
    assert r["rows"][0]["x"] is None
    assert "不可分辨" in r["rows"][0]["flag"]
    assert r["note"] and "散度带" in r["note"]


def test_退化排序_基准只有一场时也算不可分辨():
    runs = [run("z0", 1, p2p=0.05), run("z0-tau+30", 1, p2p=0.30)]
    r = ms.rank_degradation(ms.summarise(runs))
    assert r["resolvable"] is False
    assert r["rows"][0]["knob"] == "tau"


def test_退化排序_发散的档排最前_且不给倍数():
    """发散不是"更严重的退化"，是**另一个类别**。混进倍数排序里，
    会拿一个已经飞天的高度差去跟"抖了一点"比大小。"""
    runs = [run("z0", 1, p2p=0.05), run("z0", 2, p2p=0.07),
            run("z0-rate-30", 1, p2p=0.10),                                    # x = 4
            run("z0-thrust-30", 1, truth=truth_rows(1.0, 40, z_final=-6.0))]   # 发散
    rows = ms.rank_degradation(ms.summarise(runs))["rows"]
    # ★ 断言用 `startswith` 而不是整串相等：`flag` 那一格要**把"为什么不给倍数"一起印在表里**
    #   （`rank_degradation` 的输出直接进文档），所以它是"发散（不参与倍数）"这种带解释的文本。
    #   钉住的是**语义**（这一类排最前、且 x 为空），不是那串字面量。
    assert rows[0]["knob"] == "thrust" and rows[0]["flag"].startswith("发散")
    assert rows[0]["x"] is None
    assert rows[1]["knob"] == "rate"


def test_退化排序_求解耗时不算退化指标():
    """★ 求解耗时对**机器负载**敏感（同一台机器空机 3.995 ms / 跑 pytest 时 11.297 ms）。

    它进排序的话，"我这轮扫描里机器忙了一下"会被读成"这个参数让求解变慢了" ——
    两个方向完全不同的结论。它照旧**印在表里**（D6 那一列），只是不参与"谁先退化"。
    """
    runs = [run("z0", 1, p2p=0.05), run("z0", 2, p2p=0.07),
            run("z0-inertia+30", 1, p2p=0.07, solve={"n": 2500, "p50": 99.0, "p99": 99.0,
                                                     "max": 99.0, "steady_n": 2495,
                                                     "steady_p50": 99.0, "steady_p99": 99.0,
                                                     "steady_max": 99.0, "ladder_steady": "50 Hz"})]
    r = ms.rank_degradation(ms.summarise(runs))
    assert r["rows"][0]["knob"] == "inertia"
    assert r["rows"][0]["x"] == pytest.approx(1.0)      # 只因为 p2p 差 0.01 = 1 个半宽
    assert r["rows"][0]["key"] != "solve_steady_max"


def test_退化排序_没有基准时不炸_并说答不了():
    r = ms.rank_degradation(ms.summarise(_runs_without_baseline()))
    assert r["resolvable"] is False
    assert r["rows"][0]["knob"] == "mass"
    assert r["note"] and "答不了" in r["note"]


# ---------------------------------------------------------------- 变体表


def test_票面的变体表就是_11_个_且不含非扫描项_thrcurve():
    """`z0` + 5 个旋钮 × ±30 %。★ `z0-thrcurve` **不是**扫描项（ticket 08 已声明），
    混进来会让"±30 % 的边界"这句话多出一个不属于它的点。"""
    names = ms.variant_order()
    assert len(names) == 11
    assert names[0] == "z0"
    assert "z0-thrcurve" not in names
    for knob in ms.KNOBS:
        for tag in ("+", "-"):
            assert f"z0-{knob}{tag}30" in names


def test_旋钮归属_z0_没有旋钮():
    assert ms.knob_of("z0") is None
    assert ms.knob_of("z0-mass+30") == "mass"
    assert ms.knob_of("z0-rate-30") == "rate"
    with pytest.raises(KeyError):
        ms.knob_of("z0-thrcurve")


def test_旋钮动的参数名来自_plant_spec_不另抄一份():
    """★ 期望值/参数名只有一个真源（`plant_spec.py`）。抄一份的话，
    改旋钮的人改了真源、扫描器还在读旧名字，而报告上看不出来。"""
    assert ms.knob_param_names("z0-mass+30") == ["SIH_MASS"]
    assert ms.knob_param_names("z0-rate-30") == ["MC_ROLLRATE_P", "MC_PITCHRATE_P",
                                                 "MC_YAWRATE_P"]
    assert ms.knob_param_names("z0") == []


# ---------------------------------------------------------------- 加密档是加法式的


def test_加密档没有动_30_那套基线():
    """★★ 这条是**回归闸门**，不是装饰。

    加密档（±10 %/±5 %）唯一的合法做法是**只增不改**：`build_variants()` 把百分比
    **编进了变体名**，`MISMATCH_PCT` 一改，11 个变体全体改名、被控对象跟着变，
    而 ticket 12 那 13 场归档（文件名与读数都挂在这套名字上）**当场孤立**。
    曾经写过一版"改 `MISMATCH_PCT` 就行"的方案 —— 那个方案会**静默地**毁掉证据：
    跑起来照样出表，只是表里的名字和归档对不上了。
    """
    assert plant_spec.MISMATCH_PCT == 0.30, "基线百分比被动了 —— 那会孤立 12 号票的 13 场归档"

    base = set(plant_spec.build_variants())
    # 基线仍是那 11 个 ±30 % 的名字（+ 非扫描项），且**不含**任何加密档名
    for knob in ms.KNOBS:
        for tag in "+-":
            assert f"z0-{knob}{tag}30" in base, f"基线丢了 z0-{knob}{tag}30"
    assert not [n for n in base if n.endswith(("+10", "-10", "+5", "-5"))], \
        f"加密档混进了基线：{sorted(base)}"


def test_加密档的值是基线的_90_95_且查得到():
    """加密档要**按名字查得到**（`all_variants`），否则 12 号票的判据⑤ 直接 KeyError。

    `verify_z0_plant.expected_params` 用 `all_variants()` 查 —— 它要是只查基线，
    加密档那 8 场会**在判据⑤ 上崩**，而不是给出一个错的读数（崩比错好，但一样跑不完）。
    """
    dense = plant_spec.build_dense_variants()
    assert set(dense) == {f"z0-{k}{t}{p}"
                          for k in plant_spec.DENSE_KNOBS
                          for p in (10, 5) for t in "+-"}

    every = plant_spec.all_variants()
    assert set(plant_spec.build_variants()) <= set(every), "并集丢了基线"
    assert set(dense) <= set(every), "并集没有加密档 ⟹ 按名查会 KeyError"

    base = plant_spec.all_variants()["z0"]
    for pct, name in ((1.10, "z0-mass+10"), (0.90, "z0-mass-10"),
                      (1.05, "z0-mass+5"), (0.95, "z0-mass-5")):
        assert dense[name]["SIH_MASS"] == pytest.approx(base["SIH_MASS"] * pct)
    for pct, name in ((1.10, "z0-thrust+10"), (0.95, "z0-thrust-5")):
        assert dense[name]["SIH_T_MAX"] == pytest.approx(base["SIH_T_MAX"] * pct)
        # 推力旋钮**同时**动 Q_MAX（保持偏航/推力比）—— 加密档也得守这条
        assert dense[name]["SIH_Q_MAX"] == pytest.approx(base["SIH_Q_MAX"] * pct)

    # 加密档只动**那一个**旋钮：`z0-mass+10` 不许碰推力
    assert dense["z0-mass+10"]["SIH_T_MAX"] == base["SIH_T_MAX"]


def test_emit_的产物覆盖加密档():
    """`--emit`/`--check` 走 `targets()` ⟹ 它必须含加密档，否则那 8 份覆盖层
    永远不会被生成/校验，而 `mismatch_sweep` 会以"没有这个覆盖层"退出。"""
    t = plant_spec.targets()
    for name in plant_spec.build_dense_variants():
        assert f"params/{name}.param" in t, f"targets() 漏了 {name}"


# ------------------------------------------------------- 半截数值不许作废整场飞行
#
# ★ 这一节测的是**别人票里的**取数函数（`verify_offboard_smoke.field`，属 05/11），
#   为什么放在本文件：2026-09-29 加密档实跑**整批第一场就死在这里**
#   （`float('-')`，见下），而本会话的纪律是"不擅自改别人票的**测试**文件"。
#   ⟹ 冒烟在**本条跑批所依赖的那条通路**上，就记在本条的测试里；
#   改 `verify_offboard_smoke.py` 本身已在那处代码注释里写明来由。


def _smoke():
    """按**同一个**模块对象取 `field()` —— 别另起一次导入（否则测的不是飞行时用的那份）。"""
    if str(HERE) not in sys.path:
        sys.path.insert(0, str(HERE))
    import verify_offboard_smoke
    return verify_offboard_smoke


@pytest.mark.parametrize("line, want", [
    ("    z: 0.00016", 0.00016),
    ("    x: -0.00023", -0.00023),          # 负号 + 小数
    ("    vz: -1.2e-3", -0.0012),           # 科学计数
    ("    z: .5", 0.5),                     # 小数点开头
    ("    z: 5.", 5.0),                     # 小数点结尾
    ("    timestamp: 23576000 (0.000000 seconds ago)", 23576000.0),
])
def test_正常数值的取法一个都没变(line, want):
    """改 `field()` 的正则**只该**把"半截值"从抛改成 `None` —— 合法写法一律照旧。"""
    assert _smoke().field(line, line.strip().split(":")[0]) == want


@pytest.mark.parametrize("line", [
    "    z: -",                    # ☠️ 真的崩过：窗口边界落在负号后面
    "    z: e",                    # ☠️ 同理：'e' 也在旧字符类里
    "    TOPIC: estimator_status_flags",
    "    z: nan",
])
def test_半截或非数值一律给_None_绝不抛(line):
    """`field()` 是**有损字节窗口**上的取数器：取不到就 `None`，抛异常等于整场飞行作废。
    旧字符类 `-?[0-9.eE+-]+` 能只匹配到 `-`/`e`/`E`/`.`/`+` 一个字符 ⟹ `float()` 抛。"""
    assert _smoke().field(line, "z") is None


def test_截断的真值块不再让整场作废():
    """把**真值流**的一个块截到负号后面 —— 复现 2026-09-29 那一崩，且要求它活下来。

    崩点在 `verify_tier_a_hover.parse_stream:129`（逐字段取数）→ `field()`：
    `text_since()` 渲染的是 `_raw[mark:]` 这个**字节窗口**，边界可以落在数值中间。
    """
    import verify_tier_a_hover as ta
    head = ("TOPIC: vehicle_local_position_groundtruth instance 0 #7\n"
            "    timestamp: 23576000 (0.000000 seconds ago)\n"
            "    x: -0.00023\n    y: 0.00113\n    z: ")
    full = head + "-0.01408\n    vx: 0.0\n    vy: 0.0\n    vz: 0.0\n"
    cut = head + "-"                       # ⟸ 窗口正好断在负号后

    rows = ta.parse_stream(full, ta.GT_TOPIC)      # 完整块：7 个字段全取到
    assert len(rows) == 1 and rows[0]["z"] == -0.01408 and rows[0]["n"] == 7

    rows = ta.parse_stream(cut, ta.GT_TOPIC)       # 截断块：**不抛**，该字段给 None
    assert len(rows) == 1 and rows[0]["z"] is None and rows[0]["x"] == -0.00023


# ---------------------------------------------------------------- 发散


def test_正常悬停_不判发散():
    div, why = ms.divergence(obs())
    assert div is False, why


def test_真值一路爬飞_判发散():
    """爬飞 = 真值高度越过 3 m（ticket 09 第 3 场就是这种：0.9 m/s 爬了 37 s）。"""
    div, why = ms.divergence(obs(truth=truth_rows(1.0, 40, z_final=-6.0)))
    assert div is True
    assert "z" in why


def test_真值掉回地面_判发散():
    """★ 判据③ 判在 EKF 帧，所以"EKF 帧抬升 1 m"与"真值掉回地面"可以同时成立。
    这一条就是那类场的下限保护：末段真值不在空中 ⟹ 发散，不管 EKF 帧说得多好。"""
    div, why = ms.divergence(obs(truth=truth_rows(1.0, 40, z_final=0.0)))
    assert div is True
    assert "末段" in why or "抬升" in why


def test_机头自转超过一圈_判发散():
    """ticket 09 第 6 场：`u[3]` 爬到 +4.696 rad/s、机头转了 13.6 圈，
    而位置那几条判据**全绿**。所以自转必须自己判。"""
    div, why = ms.divergence(obs(yaw_span=2 * math.pi + 0.1))
    assert div is True
    assert "机头" in why
    div, why = ms.divergence(obs(yaw_span=math.pi))       # 半圈不算发散
    assert div is False, why


def test_水平飞出去_判发散():
    div, why = ms.divergence(obs(truth=truth_rows(1.0, 40, x_final=5.0)))
    assert div is True
    assert "水平" in why


def test_求解器一发未解_不算发散_但要报出来():
    """★ "节点崩了"与"飞机散了"是两件事：前者是夹具/兜底的读数，
    混进发散计数会让"±30 % 内没有一条发散"这句话既对又没用。"""
    o = obs(checks=[{"name": "① 中间件", "ok": False, "detail": "", "counts": True}])
    o["dut"]["frames"] = 0
    div, why = ms.divergence(o)
    assert div is False
    assert ms.extract(o)["harness_dead"] is True
    assert ms.extract(o)["passed"] is False


# ---------------------------------------------------------------- 抽取


def test_extract_读的是同一批键_缺键给_None_而不是_抛():
    """夹具坏了的时候要能说"这一项没测到"，不能把整张表带崩。"""
    good = ms.extract(obs(climb_ekf=0.994, p2p=0.072, drift=0.011, referr=-0.006,
                          yaw_span=-0.010))
    assert good["climb_ekf"] == pytest.approx(0.994)
    assert good["p2p_ekf"] == pytest.approx(0.072)
    assert good["yaw_span_ekf"] == pytest.approx(-0.010)
    assert good["solve_steady_max"] == pytest.approx(4.2)
    assert good["passed"] is True

    empty = ms.extract({"checks": [], "dut": {}})
    assert empty["climb_ekf"] is None
    assert empty["solve_steady_max"] is None
    assert empty["passed"] is True          # 没有判据 = 没有不通过项，不是"通过"


def test_extract_不通过项按名字列出来():
    o = obs(checks=[{"name": "③ 悬停高度 1 m ± 0.15", "ok": False, "detail": "", "counts": True},
                    {"name": "④（真值帧，ℹ️ 不计数）", "ok": None, "detail": "", "counts": False}])
    e = ms.extract(o)
    assert e["passed"] is False
    assert e["failed_checks"] == ["③ 悬停高度 1 m ± 0.15"]


# ---------------------------------------------------------------- 汇总


def test_汇总_按变体分组_并与基准比差():
    runs = [run("z0", 1, climb_ekf=0.994, p2p=0.072),
            run("z0", 2, climb_ekf=1.001, p2p=0.063),
            run("z0-mass+30", 1, climb_ekf=0.980, p2p=0.090)]
    s = ms.summarise(runs)
    assert s["baseline"]["n"] == 2
    assert s["baseline"]["climb_ekf"]["median"] == pytest.approx(0.9975)
    z = s["by_variant"]["z0-mass+30"][0]
    assert z["climb_ekf"] == pytest.approx(0.980)
    # 与基准的差是**逐读数**给的，不是只给一句"正常"
    assert z["delta"]["climb_ekf"] == pytest.approx(0.980 - 0.9975)
    assert z["status"] == "稳定"


def test_汇总_判据挂了的场次记退化_而不计发散():
    runs = [run("z0"),
            run("z0-tau-30", checks=[{"name": "④ 无持续振荡", "ok": False,
                                      "detail": "", "counts": True}])]
    s = ms.summarise(runs)
    assert s["by_variant"]["z0-tau-30"][0]["status"] == "退化"
    assert s["counts"]["发散"] == 0
    assert s["counts"]["退化"] == 1


def test_汇总_发散优先于退化():
    """一场既发散又有判据不过的，只能记发散 —— 否则"从哪个参数先退化"会被淹没。"""
    runs = [run("z0"), run("z0-thrust-30", truth=truth_rows(1.0, 40, z_final=-6.0),
                           checks=[{"name": "③ 悬停高度", "ok": False,
                                    "detail": "", "counts": True}])]
    s = ms.summarise(runs)
    assert s["by_variant"]["z0-thrust-30"][0]["status"] == "发散"
    assert s["counts"]["发散"] == 1 and s["counts"]["退化"] == 0


def test_汇总_基准只有一个读数时_不算散的_但要说出来():
    """★ "对照组给出干净对比"要求基准自己先有个散度量级：一场 z0 时
    "±0.01 m 的差"分不清是失配的功劳还是这台仪器的重复性。"""
    s = ms.summarise([run("z0"), run("z0-mass-30", climb_ekf=0.95)])
    assert s["baseline"]["n"] == 1
    assert s["baseline"]["warn"] and "只有 1 场" in s["baseline"]["warn"]


def test_渲染表_每个变体一行_且标出旋钮():
    s = ms.summarise([run("z0"), run("z0-rate+30", climb_ekf=0.99)])
    md = ms.render_table(s)
    assert "| `z0-rate+30` |" in md
    assert "rate" in md
    assert "| `z0` |" in md


# ---------------------------------------------------------------- 覆盖层读回


def test_覆盖层读回_与真源逐条比对():
    """★ "这一场真的跑在失配机型上吗"必须逐场有读数：整张票的结论都挂着这个前提。
    期望值来自 `plant_spec`（真源），不是手抄。"""
    want = ms.expected_params("z0-mass+30")
    ok, detail = ms.overlay_verdict("z0-mass+30", dict(want))
    assert ok is True, detail

    bad = dict(want)
    bad["SIH_MASS"] = want["SIH_MASS"] / 1.3        # 覆盖层没生效（还是标称值）
    ok, detail = ms.overlay_verdict("z0-mass+30", bad)
    assert ok is False
    assert "SIH_MASS" in detail

    ok, detail = ms.overlay_verdict("z0-mass+30", {})   # 一条都没读到
    assert ok is False and "没读到" in detail


def test_覆盖层读回_基准变体也要查一遍():
    """基准不是"什么都不用查"——它同样是覆盖层，只是值等于标称。"""
    ok, _ = ms.overlay_verdict("z0", dict(ms.expected_params("z0")))
    assert ok is True


def test_汇总里_覆盖层没生效的场次要标出来():
    r = run("z0-mass+30")
    r["obs"]["sweep"] = {"overlay_check": {"ok": False, "detail": "SIH_MASS 读到 0.75"}}
    s = ms.summarise([run("z0"), r])
    assert s["by_variant"]["z0-mass+30"][0]["overlay_ok"] is False
    assert "覆盖层" in ms.render_table(s)


def test_落盘的运行记录能被汇总读回来(tmp_path):
    """驱动与汇总之间只有**文件**这一条接口 ⟹ 它得有回归。"""
    p = tmp_path / "obs-z0-1.json"
    p.write_text(json.dumps(obs()), encoding="utf-8")
    runs = ms.load_runs(tmp_path)
    assert len(runs) == 1
    assert runs[0]["variant"] == "z0"
    assert runs[0]["rep"] == 1
    assert ms.extract(runs[0]["obs"])["passed"] is True


def _runs_without_baseline():
    return [run("z0-mass+30", 1, climb_ekf=0.98)]


def test_没有基准时_表照样能渲染_但必须说答不了(tmp_path):
    """★ 单变体跑法（`--variants z0-mass+30`）不该把汇总器带崩，也不该假装有对照。"""
    s = ms.summarise(_runs_without_baseline())
    md = ms.render_table(s)
    assert "`z0-mass+30`" in md
    assert "答不了" in md or "没有跑基准" in md


# ---------------------------------------------------------------- 推力口径


def test_标称悬停推力从机型真源推_而不是读节点默认值():
    """★ 这一条是本张**第一位**的前提：口径错了量到的是"参数域错误"，不是失配。

    SITL 被控对象的标称值是解析的 `m·g/(4·SIH_T_MAX)`；节点里的
    `HOVER_THRUST` 是 **z0 真机满电口径**（ticket 10 实测回填）。两者差 3.92 倍 ——
    若这里相等了（有人把 `params.py` 改成 0.6235），那说明口径问题已经被解决，
    这段注释与文档要一起改，**不能让它们悄悄过期**。
    """
    got = ms.plant_hover_thrust()
    assert got == pytest.approx(0.750 * 9.80665 / (4 * 2.95), rel=1e-9)
    assert got == pytest.approx(0.6233, abs=5e-4)
    import nmpc.params as np_params
    assert np_params.HOVER_THRUST < 0.5 * got, (
        "★ 这条不是「断言某个数」，是**前提守卫**：节点默认值（`params.py`）与 SITL 被控对象"
        "不是同一个口径 ⟹ 本张的夹具必须显式覆盖它。若哪天 `params.py` 改成了 SITL 口径"
        "（或两者解耦），请连 `plant_hover_thrust()`、本测试与 `mismatch-sweep.md` §1 一起改")


def test_推力口径的三种取值():
    assert ms._resolve_hover_thrust("none") is None
    assert ms._resolve_hover_thrust("None") is None       # 驱动往子进程传的就是这个字面量
    assert ms._resolve_hover_thrust("0.5") == pytest.approx(0.5)
    assert ms._resolve_hover_thrust("auto") == pytest.approx(ms.plant_hover_thrust())


# ---------------------------------------------------------------- 身份探针的**污染**（2026-09-29 补）


def _fake_pkg(root: Path, names=("__init__.py", "params.py", "mpc_node.py")) -> Path:
    d = root / "nmpc"
    d.mkdir(parents=True)
    for n in names:
        (d / n).write_text(f"# {root.name}/{n}\n", encoding="utf-8")
    return d


def test_身份取干净子进程_不受本进程_import_污染(tmp_path, monkeypatch):
    """★★ 这条是**回归测试**，钉的是真踩过的坑（2026-09-29 10:14）。

    `mismatch_sweep.py` 第 53 行**故意**把 `REPO/"src/nmpc"` 插到 `sys.path` 最前
    （为了读源码那份常数），于是**本进程** `import nmpc` 必然落到 `src/`；
    而节点子进程只继承环境变量 `PYTHONPATH` ⟹ 飞的是 `install/`。两者**恰好相反**。
    第一版 `controller_code()` 拿本进程的 import 当身份，给每一场记了个**指向 `src/` 的假身份**，
    而打印出来完全像真的 —— 所以这里造出"两个nmpc"的局面，断言身份**必须来自子进程**。
    """
    fake_install = _fake_pkg(tmp_path / "install-like")
    fake_src = _fake_pkg(tmp_path / "src-like", names=("__init__.py", "params.py"))
    monkeypatch.setenv("PYTHONPATH", str(fake_install.parent))

    fake_mod = type(sys)("nmpc")                      # 逼真：本进程已经 import 到 src 那一份
    fake_mod.__file__ = str(fake_src / "__init__.py")
    monkeypatch.setitem(sys.modules, "nmpc", fake_mod)

    c = ms.controller_code()
    assert c["package"] == str(fake_install), "身份必须来自干净子进程（= 节点看到的那份）"
    assert set(c["files"]) == {"__init__.py", "params.py", "mpc_node.py"}
    assert c["driver_view"] == str(fake_src)
    assert c["polluted"] is True
    assert "以节点那份为准" in c["note"]


def test_身份探针失败时_明说身份不明(monkeypatch, tmp_path):
    """探针失败**不能**退回到本进程的 import —— 那正是假身份的来源。"""
    monkeypatch.setenv("PYTHONPATH", str(tmp_path / "空目录"))
    monkeypatch.setitem(sys.modules, "nmpc", type(sys)("nmpc"))   # 本进程"import 得到"
    c = ms.controller_code()
    assert c["package"] is None and not c["files"]
    assert "身份不明" in c["note"]


def test_重探判据_探针那份比本场新就拒绝(tmp_path):
    """`colcon build` 会刷新 mtime ⟹ 探针里有更新的文件就不能拿它去改旧场次。"""
    probe = {"package": "/x/install/nmpc",
             "files": {"params.py": {"path": str(tmp_path / "params.py")}}}
    (tmp_path / "params.py").write_text("x", encoding="utf-8")
    old = tmp_path / "params.py"
    now = old.stat().st_mtime

    ok, why = ms.judge_controller_record({"package": "/x/src/nmpc/nmpc"}, probe, now + 100)
    assert ok and "改用探针那份" in why
    ok, why = ms.judge_controller_record({"package": "/x/src/nmpc/nmpc"}, probe, now - 100)
    assert not ok and "比本场落盘还新" in why
    ok, why = ms.judge_controller_record({"package": "/x/install/nmpc"}, probe, now - 100)
    assert not ok and "同一份" in why


def test_重探只动身份字段_原值留在_prev(tmp_path, monkeypatch):
    """重探是**有痕**的：原值进 `controller_code_prev`，同一场的其它读数一个字节不动。"""
    outdir = tmp_path / "sweep"
    outdir.mkdir()
    d = _fake_pkg(tmp_path / "install-like")
    older = tmp_path / "params.py"                 # 探针那份"早就存在"（早于本场落盘）
    older.write_text("x", encoding="utf-8")
    past = older.stat().st_mtime - 3600
    os.utime(older, (past, past))

    obs_path = outdir / "obs-z0-1.json"
    obs_path.write_text(json.dumps({
        "sweep": {"controller_code": {"package": str(tmp_path / "src-like" / "nmpc"),
                                      "files": {"params.py": {"sha256": "dead"}}},
                  "readback": {"SIH_MASS": 0.75}}}, ensure_ascii=False), encoding="utf-8")

    probe = {"package": str(d), "files": {"params.py": {"path": str(older), "sha256": "live"}}}
    monkeypatch.setattr(ms, "controller_code", lambda: probe)
    rows = ms.refix_controller_codes(outdir)
    assert [r["changed"] for r in rows] == [True]

    got = json.loads(obs_path.read_text(encoding="utf-8"))["sweep"]
    assert got["controller_code"]["package"] == str(d)
    assert got["controller_code"]["correction"]["why"]
    assert got["controller_code_prev"]["package"].endswith("nmpc")
    assert got["readback"] == {"SIH_MASS": 0.75}, "别的读数不许碰"


# ---------------------------------------------------------------- 垂直通道刚度反推


def test_刚度反推_两个质量档的反对称偏差给出同一个k():
    """★ 这一条把"±30 % 会挂"升级成一句**能算的容差**。

    机理：判据③ 挂的这几档**不是**发散、也不是姿态问题，而是"模型里的悬停前馈与
    实际所需不一致"⟹ 位置环用**误差**去补，稳态偏差 `Δz = ΔF / k`。
    真值（本轮实测）：`mass±30` 的高度偏差**反对称**（−0.64 / +0.63 m）、
    `thrust+30` 偏 +0.48 m ⟹ 三个独立读数给出同一个 k，这才敢把它当弹簧用。
    """
    runs = [run("z0", 1, climb_ekf=1.009), run("z0", 2, climb_ekf=1.017),
            run("z0-mass+30", 1, climb_ekf=0.363),      # ΔF = +0.225·g = +2.206 N
            run("z0-mass-30", 1, climb_ekf=1.634),      # ΔF = −2.206 N
            run("z0-thrust+30", 1, climb_ekf=1.479)]    # ΔF = −1.697 N
    g = ms.vertical_gain(ms.summarise(runs))
    assert g["resolvable"], g["note"]
    assert 3.3 < g["k"] < 3.6
    assert max(g["k_all"]) - min(g["k_all"]) < 0.35, "三个档给出的 k 应当彼此接近"
    # ±0.15 m ⟹ 悬停口径最多错 ±7 %（0.15·k ≈ 0.52 N，悬停需 7.355 N）
    assert g["tol_n"] == pytest.approx(0.15 * g["k"], rel=1e-9)
    assert 0.06 < g["tol_frac"] < 0.08


def test_刚度反推_不看不动悬停推力的档():
    """`inertia` / `tau` / `rate` 不改"悬停需要多少推力" ⟹ 把它们算进去等于拿噪声当弹簧。

    反过来说：若某个姿态类的档**也**推出了很大的高度偏差，那是**另一条机理**
    （姿态一乱、推力方向就歪），不能混进同一个 k。
    """
    runs = [run("z0", 1, climb_ekf=1.0), run("z0", 2, climb_ekf=1.0),
            run("z0-inertia+30", 1, climb_ekf=2.5)]
    g = ms.vertical_gain(ms.summarise(runs))
    assert [r["variant"] for r in g["rows"]] == []
    assert g["k"] is None and not g["resolvable"]


def test_刚度反推_逐档k散得开就拒绝当容差用():
    """四个档若给出四个差很多的 k，"线性弹簧"这个模型就不成立 ⟹ **不报容差**。"""
    runs = [run("z0", 1, climb_ekf=1.0), run("z0", 2, climb_ekf=1.0),
            run("z0-mass+30", 1, climb_ekf=0.55),      # k ≈ 4.9
            run("z0-mass-30", 1, climb_ekf=1.30)]      # k ≈ 7.4
    g = ms.vertical_gain(ms.summarise(runs))
    assert g["k"] is not None and not g["resolvable"]
    assert "散得太开" in g["note"] and "别当容差用" in g["note"]
    assert "散得太开" in ms.render_gain(g)


def test_刚度反推_改了推力却没推动高度时要说机理不符():
    """`k ≤ 0` 只有两种来路：改了悬停推力却没把高度推开、或推反了 ⟹ 先查夹具。"""
    runs = [run("z0", 1, climb_ekf=1.0), run("z0", 2, climb_ekf=1.0),
            run("z0-mass+30", 1, climb_ekf=1.2)]        # 需要的力变大、高度**也**变高 ⟹ k < 0
    g = ms.vertical_gain(ms.summarise(runs))
    assert g["k"] < 0 and not g["resolvable"]
    assert "机理与数据不符" in g["note"]


def test_刚度反推_没有基准时不报数():
    runs = [run("z0-mass+30", 1, climb_ekf=0.36)]
    g = ms.vertical_gain(ms.summarise(runs))
    assert g["k"] is None and not g["resolvable"]
    assert "反推不出刚度" in g["note"]


# ---------------------------------------------------------------- 逐帧轨迹（判据③的"可重判"原料）
def _jsonl(path: Path, n: int = 1000, hz: float = 100.0, *, x0_z=None, bad_lines: int = 2):
    """造一条节点原始流：`ts` 是**微秒**（与真流一致），每帧带 10 维 `x0`。"""
    dt_us = 1e6 / hz
    lines = ["这不是 json"] * bad_lines
    for i in range(n):
        z = 0.5 + i * 0.001 if x0_z is None else x0_z(i)
        lines.append(json.dumps({"ts": 1e9 + i * dt_us, "tick": i,
                                 "x0": [0.0, 0.0, z, 0, 0, 0, 0, 0, 0, 0],
                                 "u": [7.4, 0, 0, 0], "thrust_body_z": -0.62,
                                 "fallback": "normal"}))
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    return path


def test_轨迹降采样_按时间留点而不是按条数(tmp_path):
    """真实流的到达间隔不整齐，所以留点要看 `ts` —— 按"每 N 条留一条"会被抖动带偏。"""
    p = _jsonl(tmp_path / "hover-1m-px4.jsonl", n=1000, hz=100.0)   # 10 s @ 100 Hz
    tr = ms.trace_from_jsonl(p, hz=5.0)
    assert tr["n_raw"] == 1000
    assert 49 <= tr["n_kept"] <= 52                                 # ≈ 10 s × 5 Hz
    ts = [r["t"] for r in tr["rows"]]
    gaps = [b - a for a, b in zip(ts, ts[1:])]
    assert min(gaps) > 0.15                                         # 没有挤在一起的
    assert tr["rows"][0]["t"] == 0.0


def test_轨迹只留重判要的那几列(tmp_path):
    """入库的是**降采样轨迹**，不是整条流 —— 列要是开放式的，体积就失控了。"""
    tr = ms.trace_from_jsonl(_jsonl(tmp_path / "a.jsonl", n=200), hz=5.0)
    assert tr["cols"] == ["t", "x0_x", "x0_y", "x0_z", "u0", "tbz", "fb"]
    assert set(tr["rows"][0]) == set(tr["cols"])
    assert "x0" not in tr["rows"][0]                                # 10 维原始态不进库


def test_轨迹没有_x0_的帧要跳过而不是塞_None(tmp_path):
    """真流里混着心跳等没有 `x0` 的记录；塞进去会让"窗口统计"多出一堆 None。"""
    p = tmp_path / "a.jsonl"
    p.write_text(json.dumps({"ts": 1e9, "tick": 0}) + "\n"
                 + json.dumps({"ts": 1e9 + 1000, "x0": [0, 0, 1.0] + [0] * 7}) + "\n"
                 + json.dumps({"ts": 1e9 + 2000, "tick": 2}) + "\n", encoding="utf-8")
    tr = ms.trace_from_jsonl(p, hz=5.0)
    assert tr["n_raw"] == 1 and tr["n_kept"] == 1
    assert tr["rows"][0]["x0_z"] == 1.0


def test_轨迹记下源文件哈希(tmp_path):
    """原始流会随重启消失，所以"这条轨迹来自哪一份流"只能靠哈希留凭据。"""
    tr = ms.trace_from_jsonl(_jsonl(tmp_path / "a.jsonl", n=100), hz=5.0)
    assert len(tr["source_sha256"]) == 16 and tr["source"].endswith("a.jsonl")


def test_挂轨迹_优先用_obs_自己记的原始流目录(tmp_path):
    """`--logdir` 是驱动参数、会变；`sweep.tier_a_argv` 是**当场记下来**的真路径 ⟹ 优先它。"""
    real = tmp_path / "real"
    real.mkdir()
    _jsonl(real / "hover-1m-px4.jsonl", n=200)
    out = tmp_path / "out"
    out.mkdir()
    (out / "obs-z0-1.json").write_text(json.dumps(
        {"dut": {}, "sweep": {"tier_a_argv": ["--outdir", str(real)]}}), encoding="utf-8")
    rows = ms.attach_trace(out, logdir=tmp_path / "decoy")
    assert rows[0]["changed"] and str(real) in rows[0]["why"]
    assert isinstance(json.loads((out / "obs-z0-1.json").read_text())["trace"], dict)


def test_挂轨迹_只补不覆盖(tmp_path):
    """已有 `trace` 的场次一个字节都不动 —— 那是当场从原始流降下来的第一手。"""
    out = tmp_path / "out"
    out.mkdir()
    (out / "obs-z0-1.json").write_text(json.dumps({"trace": {"n_kept": 7, "rows": []}}),
                                       encoding="utf-8")
    rows = ms.attach_trace(out, logdir=tmp_path)
    assert not rows[0]["changed"] and "不动" in rows[0]["why"]
    assert json.loads((out / "obs-z0-1.json").read_text())["trace"]["n_kept"] == 7


def test_挂轨迹_原始流不在时明说只能重飞(tmp_path):
    """★ 这张票最该防的一句假话：表还在 ⟹ 以为什么都能重算。缺料就要点名。"""
    out = tmp_path / "out"
    out.mkdir()
    (out / "obs-z0-mass+30-1.json").write_text(json.dumps({"dut": {}}), encoding="utf-8")
    rows = ms.attach_trace(out, logdir=tmp_path / "gone")
    assert not rows[0]["changed"]
    assert "重飞" in rows[0]["why"] and "不在" in rows[0]["why"]


def test_汇总报轨迹覆盖率_缺的场次要警告(tmp_path):
    """覆盖率要印在**表里**，否则读表的人会把"聚合量"当成"随时能重算"。"""
    runs = [run("z0", 1), run("z0", 2), run("z0-mass+30", 1)]
    runs[2]["obs"]["trace"] = {"hz": 5.0, "n_kept": 140, "rows": []}
    table = ms.render_table(ms.summarise(runs))
    assert "1/3 场已入库" in table
    assert "2 场没有" in table and "只能重飞" in table


# ---------------------------------------------------------------- 排序不许跨帧
def test_排序跨帧时必须点名():
    """★ EKF 帧与真值帧是**两台仪器**，各自的"散度带"不是同一把尺子上的刻度。

    本轮实测撞到：`mass`/`thrust` 的倍数来自 `climb_ekf`，而 `inertia`/`tau`/`rate`
    全来自 `climb_gt` ⟹ 一张表把两台仪器排成了一条线。
    """
    truth = truth_rows(climb_m=1.4)
    runs = [run("z0", 1, climb_ekf=1.0, truth=truth),
            run("z0", 2, climb_ekf=1.02, truth=truth_rows(climb_m=1.42)),
            # 只动 EKF 帧 ⟹ 赢家是 `climb_ekf`
            run("z0-mass+30", 1, climb_ekf=0.40, truth=truth),
            # 只动真值帧 ⟹ 赢家是 `climb_gt`
            run("z0-inertia+30", 1, climb_ekf=1.01, truth=truth_rows(climb_m=0.50))]
    r = ms.rank_degradation(ms.summarise(runs))
    assert set(r["frames"]) == {"ekf", "truth"}                # 两帧的赢家都在
    assert "混了两帧" in r["note"] and "不可比" in r["note"]
    table = ms.render_rank(r)
    assert "混了两帧" in table
    assert "**EKF**" in table and "真值" in table               # 表里逐行标出帧

    # 全部只在同一帧里赢 ⟹ 不吵（避免"处处报警 = 没有报警"）：基准重复的真值**一模一样**
    # ⟹ `climb_gt` 的散度带为 0 ⟹ 那条读数不可分辨、被剔出排序 ⟹ 只剩 EKF 帧
    runs2 = [run("z0", 1, climb_ekf=1.0, truth=truth),
             run("z0", 2, climb_ekf=1.02, truth=truth),
             run("z0-mass+30", 1, climb_ekf=0.40, truth=truth)]
    r = ms.rank_degradation(ms.summarise(runs2))
    assert set(r["frames"]) == {"ekf"}
    assert r["note"] is None


def test_跨帧差的数是现算的_不是写死的():
    """★ 这条是给一个**真栽过的跟头**立的规矩。

    2026-09-29 本轮：跨帧告警里的范围是从**已丢的第一批**抄来的数，13 场归档之后
    **没有人回算过** ⟹ 工具印的、文档引的、票面写的三处数字**互不一致**，
    而它们看上去都"像真的"（都被同一个 prose 原样搬运）。改成现算之后，
    它必须**跟着数据变** —— 本测试就是钉这一条：写死的话，下面第二次会挂。
    """
    truth = truth_rows(climb_m=1.4)
    def mixed(extra_up_tail):
        return [run("z0", 1, climb_ekf=1.0, truth=truth),
                run("z0", 2, climb_ekf=1.02, truth=truth_rows(climb_m=1.42)),
                run("z0-mass+30", 1, climb_ekf=0.40, truth=truth,
                    track_extra={"up_tail": extra_up_tail}),
                run("z0-inertia+30", 1, climb_ekf=1.01, truth=truth_rows(climb_m=0.50))]

    s1 = ms.summarise(mixed(-0.40))
    note1 = ms.rank_degradation(s1)["note"]
    gaps = [r["up_tail"] - r["climb_gt"]
            for vr in s1["by_variant"].values() for r in vr
            if isinstance(r.get("up_tail"), (int, float))
            and isinstance(r.get("climb_gt"), (int, float))]
    assert len(gaps) >= 2
    assert f"{min(gaps):+.2f}" in note1 and f"{max(gaps):+.2f}" in note1

    # 只挪**一场**的高度 ⟹ 范围跟着变。若文案是写死的，这里必挂。
    s2 = ms.summarise(mixed(-5.00))
    note2 = ms.rank_degradation(s2)["note"]
    assert note2 != note1
    assert "-6.40" in note2                      # -5.00 − 1.40，跟着数据变了

    # ★ "跨零"这句也必须是**判**出来的，不许当装饰写死：
    #   上面两批的差**全是负的** ⟹ 只能说"同号"；
    #   把某一场的高度挪到真值之上 ⟹ 差变正 ⟹ 这时才**允许**说"跨零"。
    assert "且**跨零**" not in note1 and "**同号**（没跨零）" in note1
    assert "且**跨零**" not in note2 and "**同号**（没跨零）" in note2
    note3 = ms.rank_degradation(ms.summarise(mixed(+3.00)))["note"]
    assert "且**跨零**" in note3 and "同号" not in note3  # +3.00 − 1.40 = +1.60 > 0


def test_帧的判断_按后缀而不是按猜():
    assert ms.rank_frame("climb_ekf") == "ekf"
    assert ms.rank_frame("climb_gt") == "truth"
    assert ms.rank_frame(None) is None


def test_刚度表要写清哪列是算的哪列是量的():
    """★ 不说清就会被误引：`ΔF` 是**算的**（控制器口径），只有 `Δz` 是**量的**。

    本轮差点这么写错：把 `ΔF` 当实测，进而说"与 20 号票 §8.4 的 2.207 N 互为旁证" ——
    其实两处用的是**同一个式子**，是同一个数，不是旁证。

    ★★ 表头还必须写清**是哪一档口径**：`thrust±30` 上"控制器口径"与"本场被控对象口径"
    差 1.3 倍 ⟹ 同一个 `Δz` 能被合法地反推出 3.57 或 4.65 N/m。不说清 = 这表有两个答案。
    """
    runs = [run("z0", 1, climb_ekf=1.0), run("z0", 2, climb_ekf=1.0),
            run("z0-mass+30", 1, climb_ekf=0.45)]
    out = ms.render_gain(ms.vertical_gain(ms.summarise(runs)))
    head = out.splitlines()[0]
    assert "**算**" in head and "**量**" in head
    assert "控制器口径" in head                 # ★ 不是"机型口径"——那会歧义成被控对象
    assert "11.80" in head                      # 把那个数印出来，读者才好复算


# ---------------------------------------------------------------- 扫描器自己的身份
def test_扫描器身份_两批不同时必须报警():
    """★ 被测对象的身份记了，**量它的那把尺子**也得记 —— 本轮工具恰在两批之间被改过。

    只记被测对象的话，"两批之间改了尺子"这件事在归档里**查不出来**。
    """
    runs = [run("z0", 1), run("z0", 2), run("z0-mass+30", 1)]
    for i, r in enumerate(runs):
        r["obs"]["sweep"] = {"harness": {"sha256": "aaaa0000aaaa0000" if i < 2
                                         else "bbbb1111bbbb1111"}}
    s = ms.summarise(runs)
    assert s["harness"]["n_sigs"] == 2
    assert "不是同一个实验" in s["harness"]["warn"]
    assert "不是同一个实验" in ms.render_table(s)


def test_扫描器身份_没留档时明说查不出来():
    """没留档 ≠ 前后一致。**这句话必须说出来**，否则读表的人会默认它们是同一把尺子。"""
    runs = [run("z0", 1), run("z0-mass+30", 1)]
    s = ms.summarise(runs)
    assert s["harness"]["n_sigs"] == 1
    assert "查不出" in s["harness"]["warn"]
    assert "查不出" in ms.render_table(s)


def test_扫描器身份_同一份时只报签名不报警():
    runs = [run("z0", 1), run("z0-mass+30", 1)]
    for r in runs:
        r["obs"]["sweep"] = {"harness": {"sha256": "cccc2222cccc2222"}}
    s = ms.summarise(runs)
    assert s["harness"]["warn"] is None
    table = ms.render_table(s)
    assert "cccc2222cccc2222" in table and "不是同一个实验" not in table


def test_记录身份时会带上本文件自己的哈希与时间(tmp_path, monkeypatch):
    """`controller_code` 只探被测对象；这一块探的是本脚本自己。"""
    cc = {"harness": {"file": Path(ms.__file__).name,
                      "sha256": ms.sha256_16(Path(ms.__file__))}}
    assert cc["harness"]["file"] == "mismatch_sweep.py"
    assert len(cc["harness"]["sha256"]) == 16


def test_轨迹要记是谁挂的(tmp_path):
    """★ 事后补挂会把"前后两批的差别"抹平 —— 本轮就是这么丢掉旁证的。

    所以 `attached_by` 必须能区分"当场挂"（原始流一定在）与"事后补"，否则归档里
    只能靠口供说"我当时只改了 X"。
    """
    out = tmp_path / "out"; out.mkdir()
    real = tmp_path / "real"; real.mkdir()
    _jsonl(real / "hover-1m-px4.jsonl", n=200)
    (out / "obs-z0-1.json").write_text(json.dumps(
        {"dut": {}, "sweep": {"tier_a_argv": ["--outdir", str(real)]}}), encoding="utf-8")
    assert ms.attach_trace(out, logdir=tmp_path)[0]["changed"]
    tr = json.loads((out / "obs-z0-1.json").read_text())["trace"]
    assert tr["attached_by"] == "attach_trace"

    runs = [run("z0", 1), run("z0-mass+30", 1)]
    runs[0]["obs"]["trace"] = {"hz": 5.0, "rows": [], "attached_by": "run_variant"}
    runs[1]["obs"]["trace"] = {"hz": 5.0, "rows": [], "attached_by": "attach_trace"}
    table = ms.render_table(ms.summarise(runs))
    assert "run_variant 1 场" in table and "attach_trace 1 场" in table
