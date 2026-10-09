#!/usr/bin/env python3
"""`battery_from_ulog` 的自检 —— **不需要 pyulog、不需要真日志、不需要飞控通电**。

这里用一份**假 ULog**（只实现 `get_dataset()` / `data_list` / `initial_parameters`）
喂 `analyse_ulog()`。为什么值得分开一层：这些路径原本只在**归档日志**上暴露，
而"这份日志没飞行段"与"这段代码崩了"在批跑里长得一模一样
（都被 `except` 吞掉、都显示成"没数据"）。合成用例把它们钉住，且不依赖任何本地日志目录。

★ 合成的场景**故意**做成"地面上静止 60 s，飞行段里只有 10 s 真悬停"——
第一版 `hover_anchor` 没有"先夹到在飞段"这一步，最长稳态段被地面段整段赢走，
归档里八份日志**一份锚点都没取到**。这条就钉在这里。

跑法::

    python3 tools/battery-thrust/test_battery_from_ulog.py --self-test
    pytest tools/battery-thrust/test_battery_from_ulog.py
"""

from __future__ import annotations

import contextlib
import io
import pathlib
import sys
import types

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import battery_from_ulog as B  # noqa: E402

#: 合成场景的已知真值（解析算得出来，见 `_synth`）
T_HOVER = (70.0, 80.0)      # 真悬停段
I_HOVER = 3.6               # 悬停电流
I_MOVE = 6.0                # 机动电流
R_INT = 0.09                # 合成内阻
V_OC = 22.0                 # 飞行时开路电压
SPIKE_A = 12.0              # 一个孤点（用来区分"滑窗中位"与"单点最大"）


class _DS:
    """假 dataset：只要能 `ds.data[字段]` 就够了。"""

    def __init__(self, **cols):
        self.data = {k: np.asarray(v, dtype=float) for k, v in cols.items()}


class _FakeULog:
    def __init__(self, topics, params=None):
        self._t = dict(topics)
        self.data_list = [types.SimpleNamespace(name=n) for n in self._t]
        self.initial_parameters = dict(params or {})

    def get_dataset(self, name):
        if name in self._t:
            return self._t[name]
        raise KeyError(f"没有这个 topic: {name}")


def _us(t_sec):
    """秒 → 微秒（ulog 原样是微秒，这里按真实格式造）。"""
    return np.asarray(t_sec, dtype=float) * 1e6


#: 默认内嵌参数：与归档日志实测一致（`BAT1_CAPACITY = -1`、`BAT_V_OFFS_CURR = 0` 也照抄）
PARAMS = {
    "BAT1_A_PER_V": 17.0, "BAT1_V_DIV": 10.02, "BAT1_N_CELLS": 6.0,
    "BAT1_CAPACITY": -1.0, "BAT1_R_INTERNAL": 0.005, "BAT_AVRG_CURRENT": 15.0,
    "BAT_V_OFFS_CURR": 0.0, "BAT1_V_CHARGED": 4.2, "BAT1_V_EMPTY": 3.2,
}


def _synth(*, armed_from=60.0, flight=(62.0, 90.0), with_lpos=True, with_att=True,
           with_cm=True, cells_zero=True, params=None, i_spike=True):
    """一份**合成但结构完整**的日志。返回 `_FakeULog`。

    时间轴：0–100 s；`0–60 s` 在地面**静止**（这是个陷阱：比真悬停还"稳态"）；
    `62–90 s` 在飞；飞行段里只有 `70–80 s` 是真悬停，其余在动（|v| = 0.6）。
    """
    t_bat = np.arange(0.0, 100.0 + 1e-9, 0.2)   # 5 Hz
    t_cm = np.arange(0.0, 100.0 + 1e-9, 2.0)    # 0.5 Hz
    t_land = np.arange(0.0, 100.0 + 1e-9, 0.5)  # 2 Hz
    t_att = np.arange(0.0, 100.0 + 1e-9, 0.05)  # 20 Hz
    lo, hi = flight

    flying = (t_bat >= lo) & (t_bat < hi)
    hover = (t_bat >= T_HOVER[0]) & (t_bat < T_HOVER[1])
    current = np.where(hover, I_HOVER, np.where(flying, I_MOVE, 0.4))
    if i_spike:
        k = int(np.argmin(np.abs(t_bat - 66.4)))  # 机动段里的一个孤点
        current = current.copy()
        current[k] = SPIKE_A
    voltage = np.where(flying, V_OC - R_INT * current,
                       np.where(t_bat >= 95.0, 24.0, 25.2))

    topics = {
        "battery_status": _DS(
            timestamp=_us(t_bat), voltage_v=voltage,
            voltage_filtered_v=voltage, current_a=current,
            current_filtered_a=current,
            current_average_a=np.full(t_bat.size, 15.0),   # 初值 = BAT_AVRG_CURRENT
            discharged_mah=np.cumsum(current) * 0.2 / 3.6,  # 只是个单调增的占位
            remaining=np.clip(0.9 - 0.0002 * t_bat, 0.0, 1.0),
            scale=np.full(t_bat.size, 1.15), cell_count=np.full(t_bat.size, 6.0),
            **{f"voltage_cell_v[{i}]": (np.zeros(t_bat.size) if cells_zero
                                        else np.full(t_bat.size, 3.85))
               for i in range(6)},
        ),
        "vehicle_land_detected": _DS(
            timestamp=_us(t_land), landed=((t_land < lo) | (t_land >= hi)).astype(float)),
    }
    if with_cm:
        topics["vehicle_control_mode"] = _DS(
            timestamp=_us(t_cm), flag_armed=(t_cm >= armed_from).astype(float))
    if with_lpos:
        speed = np.where((t_bat >= lo) & (t_bat < hi), 0.6, 0.0)
        speed = np.where(hover, 0.0, speed)
        topics["vehicle_local_position"] = _DS(
            timestamp=_us(t_bat), vx=speed, vy=np.zeros(t_bat.size), vz=np.zeros(t_bat.size))
    if with_att:
        q0 = np.ones(t_att.size)
        topics["vehicle_attitude"] = _DS(
            timestamp=_us(t_att), **{f"q[{i}]": (q0 if i == 0 else np.zeros(t_att.size))
                                     for i in range(4)})
    return _FakeULog(topics, params if params is not None else PARAMS)


def _run(u, **kw):
    """`analyse_ulog` 需要 path 只是为了算 sha256/大小 —— 拿本文件顶一下。"""
    return B.analyse_ulog(u, __file__, **kw)


def _capture(fn, *a, **kw):
    """把 `fn` 打到 stdout 的东西收回来（报告/摘要都是直接 `print` 的）。"""
    buf = io.StringIO()
    with contextlib.redirect_stdout(buf):
        fn(*a, **kw)
    return buf.getvalue()


# ------------------------------------------------------------------ 主链路


def test_合成日志端到端():
    rep, code = _run(_synth())
    assert "error" not in rep, rep.get("error")
    assert code == B.EXIT_OK
    fw = rep["flight_window"]
    assert abs(fw["t0"] - 62.0) < 0.21 and abs(fw["t1"] - 89.8) < 0.21
    a = rep["hover_anchor"]
    assert abs(a["window_s"][0] - 70.0) < 0.21 and abs(a["window_s"][1] - 79.8) < 0.21
    assert abs(a["current_a"] - I_HOVER) < 1e-9
    assert abs(a["voltage_v"] - (V_OC - R_INT * I_HOVER)) < 1e-9
    assert abs(a["thrust_n"] - 0.75 * 9.80665) < 1e-9


def test_锚点被夹在在飞窗口内_地面段赢不走():
    """★★ 回归：地面静止段（60 s）比真悬停（10 s）长得多，绝不能让它赢。

    不夹的话锚点会落在 0–60 s 上：电压是**静置**电压、电流是**静态**电流，
    两个都不是"电池在悬停时要供多少"。合成场景就是为这条造的。
    """
    rep, _ = _run(_synth())
    a = rep["hover_anchor"]
    fw = rep["flight_window"]
    assert a["window_s"][0] >= fw["t0"] - 1e-9, "锚点跑到在飞窗口之外了"
    assert a["window_s"][1] <= fw["t1"] + 1e-9
    assert a["voltage_v"] < 24.0, "取到了地面段的静置电压（25.2 V 那一档）"


def test_可持续电流是滑窗中位_孤点只进raw_max():
    """★ 合成日志里有一个 12 A 的孤点，而真值是 6.0 A —— 两者必须分得开。"""
    rep, _ = _run(_synth())
    s = rep["current"]["sustained"]
    assert abs(s["value"] - I_MOVE) < 1e-9, f"孤点污染了判据：{s['value']}"
    assert abs(s["raw_max"] - SPIKE_A) < 1e-9
    assert s["raw_max"] > s["value"]


def test_全段最大落在起飞那一瞬要在摘要里标出来():
    """★ 票面问 ① 时**点名排除**了「起飞那一瞬的最大电流」，而"全段最大"天生往负载最高的
    那一瞬跑。合成日志默认就把最大值放在在飞窗口起点（62 s 起就在动 = 6.0 A）；
    把悬停挪到窗口开头（`flight=(70, 90)`）后，最大值落到 80 s ⟹ 两种情形必须分得开。

    变异：把 `at_flight_start` 恒置 True/False、或把 `★起` 标记从摘要行里去掉，本用例立刻红。
    """
    rep_start, _ = _run(_synth())
    rep_later, _ = _run(_synth(flight=(70.0, 90.0)))
    assert rep_start["current"]["sustained"]["at_flight_start"] is True
    assert rep_later["current"]["sustained"]["at_flight_start"] is False
    # ★ 摘要行也要跟着标 —— 批量扫（`--quiet`）时读者只看这一行
    marked = _capture(B._summary_line, rep_start)
    plain = _capture(B._summary_line, rep_later)
    assert "★起" in marked and "★起" not in plain


def test_前后半段的读数分别给出():
    """票面要求写清"用的是哪段窗口" —— 所以两半段各报一次，且末端电压更低。"""
    rep, _ = _run(_synth())
    f = rep["current"]["sustained_first_half"]
    l = rep["current"]["sustained_last_half"]
    assert f["sustained_a"]["value"] is not None and l["sustained_a"]["value"] is not None
    assert f["voltage_v_median"] is not None and l["voltage_v_median"] is not None


# ------------------------------------------------------------------ 两个常数不许混用


def test_外推目标就是f_max_不是t_max():
    """★★ 票面要求说清 `f_max`（OCP 的上限，按电池）与 `t_max`（归一化，按电机）是两个量。

    这条判据的可证伪形态：**换 `f_max`，外推结果必须跟着换**。
    若谁把 `t_max = 46.3027` 混进换算里（历史 bug：F 上限被从 11.8 抬到 46.3），
    外推目标就不会随 `f_max` 动 —— 那这条会红。
    """
    a, _ = _run(_synth())
    b, _ = _run(_synth(), f_max=2 * B.F_MAX_DEFAULT)
    e_a, e_b = a["extrapolation"], b["extrapolation"]
    assert abs(e_a["t_target_n"] - B.F_MAX_DEFAULT) < 1e-9
    assert abs(e_b["t_target_n"] - 2 * B.F_MAX_DEFAULT) < 1e-9
    assert e_b["current_a"] > e_a["current_a"], "换了 f_max 而外推不动 ⟹ 用的是别的常数"
    # 推力比是 f_max / 悬停推力：1.604 倍（这是 `f_max` 那一侧的数）
    assert abs(e_a["thrust_ratio"] - B.F_MAX_DEFAULT / (0.75 * 9.80665)) < 1e-6
    # t_max 只作为**提醒**出现，且必须写明它没参与换算
    assert abs(a["verdict"]["t_max_n"] - B.T_MAX_DEFAULT) < 1e-9
    assert abs(B.T_MAX_DEFAULT / B.F_MAX_DEFAULT - 3.924) < 1e-3
    assert "不是一个量" in a["verdict"]["t_max_note"]


def test_判据带前提字段():
    """判据只在"`BAT1_A_PER_V` 标度大致正确"下成立 —— 这条前提必须随读数一起给出。"""
    rep, _ = _run(_synth())
    v = rep["verdict"]
    assert "BAT1_A_PER_V" in v["premise"]
    assert 0 < v["fraction_of_rating"] < 1


# ------------------------------------------------------------------ 坏日志的路径


def test_没有在飞段时给干净结论而不是崩():
    """在飞窗口为空是**正常结论**（这份日志没飞行段），不是崩溃、也不该是"没数据"。"""
    rep, code = _run(_synth(armed_from=1e9))
    assert code == B.EXIT_NO_DATA
    assert "在飞窗口为空" in rep["error"]
    assert "没有飞行段" in rep["error"]


def test_缺battery_status就给EXIT_NO_DATA():
    u = _synth()
    del u._t["battery_status"]
    u.data_list = [types.SimpleNamespace(name=n) for n in u._t]
    rep, code = _run(u)
    assert code == B.EXIT_NO_DATA and "battery_status" in rep["error"]


def test_缺control_mode时明确拒绝():
    """分不出「在地面」与「在飞」时**不能默认全要** —— 地面上静止也满足一切稳态判据。"""
    rep, code = _run(_synth(with_cm=False))
    assert code == B.EXIT_NO_DATA and "vehicle_control_mode" in rep["error"]
    assert "地面上没有负载" in rep["error"]


def test_缺姿态时有在飞段但取不到锚点():
    """有在飞段、却挑不出稳态悬停 ⟹ 退出码 2（**不是** 0）—— "能读电流"与"能换算"是两件事。"""
    rep, code = _run(_synth(with_att=False))
    assert code == B.EXIT_UNAVAILABLE
    assert rep["current"]["sustained"]["value"] is not None, "电流那一列还是应该读出来的"
    assert "vehicle_attitude" in rep["anchor_error"]
    assert "verdict" not in rep


def test_报错路径自己不能炸_缺字段时报的是缺哪个():
    """★ `_load` 的提示里**不能**出现 `ulog.file_name` —— pyulog 的 `ULog` 压根不存文件名，
    那么写会在最需要它的那条路径上抛 `AttributeError`，把真因盖掉（同族工具踩过）。"""
    u = _synth()
    u._t["battery_status"] = _DS(timestamp=_us(np.arange(0.0, 10.0, 0.2)),
                                 voltage_v=np.zeros(50))  # 少了 current_a
    try:
        _run(u)
    except SystemExit as exc:
        msg = str(exc)
    else:
        raise AssertionError("字段缺了应当干净退出，而不是继续跑")
    assert "current_a" in msg and "battery_status" in msg
    assert "file_name" not in msg.split("python3")[0]


# ------------------------------------------------------------------ 取不到的量（票面问题 ③）


def test_单节电压全零要点名():
    """z0 是**模拟**电源模块：`voltage_cell_v[*]` 字段在、**整份全 0** ⟹ 单节电压取不到。"""
    rep, _ = _run(_synth(cells_zero=True))
    v = rep["channel_verdicts"]["voltage_cell_v"]
    assert v["verdict"] == "absent"
    assert rep["voltage"]["cell_voltage_all_zero"] is True
    assert any("voltage_cell_v" in m for m in rep["missing"])


def test_单节电压有非零读数时不说缺席():
    """反方向也要钉：非零时不能也说"缺席"（否则判据写成永真也能过）。"""
    rep, _ = _run(_synth(cells_zero=False))
    assert rep["channel_verdicts"]["voltage_cell_v"]["verdict"] == "usable"
    assert not any("voltage_cell_v" in m for m in rep["missing"])


def test_容量未知与标度未验证都要点名():
    """`BAT1_CAPACITY = -1` ⟹ `remaining` 是电压法；`BAT1_A_PER_V` 是**没人验证过**的标度。

    两条都必须出现在 `missing` 里 —— 票面问题 ③ 要的就是这份正面清单。
    """
    rep, _ = _run(_synth())
    blob = " ".join(rep["missing"])
    assert "BAT1_CAPACITY" in blob and "电压法" in blob
    assert "BAT1_A_PER_V" in blob and "无法独立验证" in blob
    assert "BAT_V_OFFS_CURR" in blob
    assert "15 A" in blob, "额定值本身的性质（热/寿命）也要说清"


def test_参数缺键给None不给0():
    """★ `None` = "这份日志里没有这个参数"，与"这个参数是 0"是两回事。

    `BAT1_A_PER_V` 若被当成 0，后面所有换算都会静默地变成 0 A / 0 W。
    """
    u = _synth(params={"BAT1_A_PER_V": 17.0})
    p = B._params(u)
    assert p["BAT1_A_PER_V"] == 17.0
    assert p["BAT1_N_CELLS"] is None and p["BAT1_CAPACITY"] is None


def test_电流平均列与scale都判成不可用():
    rep, _ = _run(_synth())
    cv = rep["channel_verdicts"]
    assert cv["current_average_a"]["verdict"] == "not-a-measurement"
    assert cv["scale"]["verdict"] == "not-a-measurement"


def test_判据自检这一节一定在报告里():
    """票面要求"判据要从**能区分对错**的那一列挑" —— 那一节的结论必须随读数一起给出。"""
    rep, _ = _run(_synth())
    sig = rep["signature"]
    assert sig["verdict"] in {"visible-in-voltage", "not-visible-in-voltage", "no-data"}
    assert sig["noise_current_1sigma_a"] is not None
    assert "fit_is_identifying" in sig, "拟合是否可辨识必须显式给出，否则 R 会被当测量值引用"
    assert "policy" in sig and "单点永不作为判据" in sig["policy"]


# ------------------------------------------------------------------ 目录模式 / 摘要


def test_目录模式递归找ulg(tmp_path=None):
    import tempfile
    import os
    with tempfile.TemporaryDirectory() as d:
        os.makedirs(os.path.join(d, "a", "fcu"))
        open(os.path.join(d, "a", "fcu", "x.ulg"), "w").close()
        os.makedirs(os.path.join(d, "old", "b", "fcu"))
        open(os.path.join(d, "old", "b", "fcu", "y.ulg"), "w").close()
        got = B._iter_targets(d)
        assert len(got) == 2, "目录模式必须**递归**找 —— 归档日志就埋在 <场景>/fcu/ 下面"
        assert B._iter_targets(os.path.join(d, "a", "fcu", "x.ulg")) == \
            [os.path.join(d, "a", "fcu", "x.ulg")]


def test_摘要里的场景名保留old前缀():
    """`3_traj1_hover` 在 `bag/` 与 `bag/old/` 下**同名但是两趟**（电量差 2 V 以上）。

    丢掉 `old/` 前缀后摘要里会出现两行一模一样的名字 —— 而这两行恰恰是本票
    结论要区分的那两档（满电 vs 低电量）。
    """
    assert B._scene_name("/x/bag/2_manual_takeoff/fcu/a.ulg") == "2_manual_takeoff"
    assert B._scene_name("/x/bag/old/3_traj1_hover/fcu/a.ulg") == "old/3_traj1_hover"
    assert B._scene_name("a.ulg") == "a.ulg"


def test_sha256对得上():
    here = pathlib.Path(__file__).resolve()
    import hashlib
    assert B._sha256(here) == hashlib.sha256(here.read_bytes()).hexdigest()


CASES = [v for k, v in sorted(globals().items()) if k.startswith("test_")]


def main() -> int:
    if "--self-test" not in sys.argv:
        print(__doc__)
        return 0
    passed, failed = 0, 0
    for fn in CASES:
        try:
            fn()
        except Exception as exc:  # noqa: BLE001
            failed += 1
            print(f"❌ {fn.__name__}\n     {type(exc).__name__}: {exc}")
        else:
            passed += 1
            print(f"✅ {fn.__name__}")
    print(f"\n{passed} passed, {failed} failed")
    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(main())
