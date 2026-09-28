#!/usr/bin/env python3
"""`hover_thrust_from_ulog` 的自检 —— **不需要 pyulog 日志文件、不需要飞控通电**。

这里用一份**假 ULog**（只实现 `get_dataset()` 与 `data_list`）喂 `analyse_ulog()`。
为什么值得单独有这么一层：这些判据原本只在 175 份 SITL 日志上跑过一遍才暴露，
其中两条是**崩溃**而不是干净结论 —— 而"崩溃"在批跑脚本里会被 `except` 吞掉，
看起来和"这份日志没数据"一模一样。合成用例把它们钉住，且不依赖任何本地日志目录。

跑法：

    python3 tools/hover-thrust/test_hover_thrust_from_ulog.py --self-test
    # 或（仓库惯例）
    pytest tools/hover-thrust/test_hover_thrust_from_ulog.py
"""

from __future__ import annotations

import pathlib
import sys
import types

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import hover_thrust_from_ulog as H  # noqa: E402

#: SITL z0 同构机的已知真值（同 `test_hover_thrust_core.py`，出处见 `evidence/README.md`）
TRUTH = 0.62330


class _DS:
    """假 dataset：只要能 `ds.data[字段]` 就够了。"""

    def __init__(self, **cols):
        self.data = {k: np.asarray(v, dtype=float) for k, v in cols.items()}


class _FakeULog:
    def __init__(self, topics):
        self._t = dict(topics)
        self.data_list = [types.SimpleNamespace(name=n) for n in self._t]

    def get_dataset(self, name):
        if name in self._t:
            return self._t[name]
        raise KeyError(f"没有这个 topic: {name}")


def _us(t_sec):
    """秒 → 微秒（ulog 原样是微秒，这里按真实格式造）。"""
    return np.asarray(t_sec, dtype=float) * 1e6


def _idle_quat(n, t_sec):
    return dict(
        timestamp=_us(t_sec),
        **{f"q[{i}]": np.full(n, 1.0 if i == 0 else 0.0) for i in range(4)},
    )


def _thrust_topic(t_sec, thrust_z):
    n = np.asarray(t_sec).size
    return _DS(timestamp=_us(t_sec), **{"xyz[2]": np.full(n, float(thrust_z))})


def _lpos_topic(t_sec):
    n = np.asarray(t_sec).size
    return _DS(timestamp=_us(t_sec), vx=np.zeros(n), vy=np.zeros(n), vz=np.zeros(n))


#: 一份**合成但结构完整**的日志：解锁与离地在两条不同的时间基上，
#: 悬停推力已知（= TRUTH），在飞窗口 = [12, 30) s。所有用例共用它。
def _synth(*, thrust_z=-TRUTH, armed_from=10.0, landed_off=(12.0, 30.0),
           with_hte=False, with_att=True, with_cm=True):
    t_cm = np.arange(0.0, 40.0 + 1e-9, 2.0)          # 21 个样本
    t_land = np.arange(0.0, 60.0 + 1e-9, 0.5)        # 121 个样本
    t_thr = np.arange(0.0, 50.0 + 1e-9, 0.05)        # 1001 个样本

    lo, hi = landed_off
    topics = {}

    if with_cm:
        topics["vehicle_control_mode"] = _DS(
            timestamp=_us(t_cm), flag_armed=(t_cm >= armed_from).astype(float)
        )
    topics["vehicle_land_detected"] = _DS(
        timestamp=_us(t_land),
        landed=((t_land < lo) | (t_land >= hi)).astype(float),
    )

    # 悬停段里推的是 TRUTH，其余（地面怠速/爬升）给一个明显不同的值
    in_win = (t_thr >= lo) & (t_thr < hi)
    topics["vehicle_thrust_setpoint"] = _DS(
        timestamp=_us(t_thr),
        **{"xyz[2]": np.where(in_win, float(thrust_z), -0.02)},
    )
    topics["vehicle_local_position"] = _lpos_topic(t_thr)
    if with_att:
        topics["vehicle_attitude"] = _DS(**_idle_quat(t_thr.size, t_thr))
    if with_hte:
        # 飞控自己的估计：故意给一个**错的**值 + valid=0（SITL 实测就是这样）
        topics["hover_thrust_estimate"] = _DS(
            timestamp=_us(t_thr), hover_thrust=np.full(t_thr.size, 0.5),
            valid=np.zeros(t_thr.size),
        )
    return _FakeULog(topics)


def _run(u, **kw):
    """`analyse_ulog` 需要 path 只是为了算 sha256/大小 —— 拿本文件顶一下。"""
    return H.analyse_ulog(u, __file__, **kw)


# ------------------------------------------------------------------ 主链路


def test_合成日志能收回已知真值():
    """端到端：解锁/离地/姿态/推力 → 读数。窗口与真值都是解析算得出来的。"""
    rep, code = _run(_synth())
    assert "error" not in rep, rep.get("error")
    assert rep["flight_window"]["t0"] == 12.0
    assert abs(rep["flight_window"]["t1"] - 29.95) < 1e-6
    assert rep["usable"] and code == H.EXIT_OK
    assert abs(rep["hover_thrust"] - TRUTH) < 1e-9
    assert rep["sign"] == "negative"


def test_飞控自己的估计不参与主口径():
    """★ HTE 开着但没收敛（valid=0）时，主口径必须**完全不受它影响**。

    SITL 实测就是 0.5 且 valid=0；拿它当读数会得到 0.5，而真值是 0.6233。
    """
    without = _run(_synth(with_hte=False))[0]
    with_hte = _run(_synth(with_hte=True))[0]
    assert with_hte["hover_thrust"] == without["hover_thrust"]
    assert with_hte["hte"]["valid_fraction"] == 0.0
    assert with_hte["hte"]["median_all"] == 0.5  # 它确实在日志里，只是不该被用


def test_没有HTE这个topic也能跑():
    rep, code = _run(_synth(with_hte=False))
    assert "hte" not in rep and code == H.EXIT_OK


# ------------------------------------------------------------------ 坏日志的路径


def test_在飞窗口为空时给干净结论而不是崩():
    """★ 175 份 SITL 日志里这类占大头（没起飞的）—— 曾经在这里抛
    `zero-size array to reduction operation minimum`，也就是**正常结论变成了崩溃**。
    """
    rep, code = _run(_synth(armed_from=1e9))  # 从未解锁
    assert code == H.EXIT_NO_DATA
    assert "在飞窗口为空" in rep["error"]
    assert "没有飞行段" in rep["error"]


def test_缺topic时报的是缺哪个_而不是AttributeError():
    """★ 报错路径自己不能炸：pyulog 的 `ULog` 不存文件名，`ulog.file_name` 会抛
    AttributeError，把"缺 vehicle_attitude"这个真因盖掉。"""
    try:
        _run(_synth(with_att=False))
    except SystemExit as exc:
        msg = str(exc)
    else:
        raise AssertionError("缺 topic 应当抛 SystemExit（干净地退出），而不是继续跑")
    assert "姿态" in msg and "vehicle_attitude" in msg
    assert "SDLOG_PROFILE" in msg
    assert "file_name" not in msg.split("python3")[0]  # 提示里不该出现那个不存在的属性


def test_缺control_mode时明确拒绝():
    """分不出「在地面」与「在飞」时**不能默认全要** —— 地面上推着飞机走也满足速度/倾角判据。"""
    rep, code = _run(_synth(with_cm=False))
    assert code == H.EXIT_NO_DATA
    assert "vehicle_control_mode" in rep["error"]


def test_解锁状态缺字段时报的是缺哪个字段_而不是KeyError():
    """★ 又一条"报错路径自己不能炸"，形状同那条 AttributeError。

    `vehicle_control_mode` **在**、但这份固件没记 `flag_armed`（或 `landed`）时，
    以前是直接 `cm.data["flag_armed"]` ⟹ `KeyError`。两类结局要分清：
    topic 不在 = 「这份日志没数据」（干净结论）；topic 在而字段不在 = 固件/机型不对（要报错）。
    """
    t_cm = np.arange(0.0, 40.0 + 1e-9, 2.0)
    u = _synth()
    u._t["vehicle_control_mode"] = _DS(timestamp=_us(t_cm), armed=np.zeros(t_cm.size))
    try:
        _run(u)
    except SystemExit as exc:
        msg = str(exc)
    else:
        raise AssertionError("topic 在、字段不在 ⟹ 该报错，而不是当成「这份日志没数据」继续跑")
    assert "flag_armed" in msg and "vehicle_control_mode" in msg
    assert "armed" in msg, "报错要把**实际**有哪些字段列出来，否则还是猜"


def test_只有竖直速度也算在动_掩码用的是三维速度模():
    """★ 速度判据读的是**三维**速度模 `sqrt(vx² + vy² + vz²)`，**不是**水平速度。

    这个区别有实际后果：垂直爬升/下降时 `vx = vy = 0`，只取水平分量就会把这一段当成"悬停" ——
    而那一刻推力拿的不只是 `m·g`（还得提供竖直加速度），读数就偏了。所以只给 `vz` 也必须判掉。
    （文档与 `--v-eps` 的 help 里早先写作"水平速度"，2026-09-28 复核订正为三维速度模。）
    """
    t = np.arange(0.0, 50.0 + 1e-9, 0.05)
    u = _synth()
    u._t["vehicle_local_position"] = _DS(
        timestamp=_us(t), vx=np.zeros(t.size), vy=np.zeros(t.size), vz=np.full(t.size, 0.5)
    )
    rep, code = _run(u)
    assert code == H.EXIT_UNUSABLE and rep["hover_thrust"] is None, \
        "只有竖直速度被当成悬停 ⟹ 判据退化成水平速度了"
    assert "没有一段稳态悬停够 2 s" in rep["notes"][0]


def test_三轴加速度无关_只看速度():
    """`v_eps` 判据用的是速度；本地位置里那三个加速度字段本工具**不读**。

    钉住这点是为了别在后人手里悄悄变成"用加速度修正推力"——
    那要另开一张票，且会改变读数口径（见 `evidence/README.md` 的"不能证明什么"）。
    """
    u = _synth()
    u._t["vehicle_local_position"] = _DS(
        timestamp=_us(np.arange(0.0, 50.0 + 1e-9, 0.05)),
        vx=np.zeros(1001), vy=np.zeros(1001), vz=np.zeros(1001),
        ax=np.full(1001, 9.0), ay=np.full(1001, 9.0), az=np.full(1001, 9.0),
    )
    rep, code = _run(u)
    assert code == H.EXIT_OK and abs(rep["hover_thrust"] - TRUTH) < 1e-9


# ------------------------------------------------------------------ 阶梯取值


def test_阶梯取值取此前最近的一个():
    """布尔量必须按阶梯取，**不能线性插值** —— 插出来的 0.5 既不是"已落地"也不是"未落地"。"""
    t_src = np.array([0.0, 10.0, 20.0])
    x = np.array([0.0, 1.0, 0.0])
    out = H._step_lookup(np.array([-1.0, 0.0, 9.99, 10.0, 20.0, 99.0]), t_src, x)
    assert list(out) == [0.0, 0.0, 0.0, 1.0, 0.0, 0.0]


def test_两条时间基合并后不广播炸也不串时刻():
    """★ 解锁在 `vehicle_control_mode` 的时间基上、离地在 `vehicle_land_detected` 的，
    两者样本数不同（21 vs 121）。各自落到查询点再加 ⟹ numpy 抛广播错；
    样本数**碰巧相同**时就变成静默地把两个不同时刻的状态配到一起 —— 这里两条都挡。
    """
    rep, _ = _run(_synth())
    # 逐点核对窗口边界：窗口必须是 [12, 30) 与推力时间基对齐后的那一段
    assert rep["flight_window"]["t0"] == 12.0
    assert 29.9 < rep["flight_window"]["t1"] <= 30.0
    # 窗口内的推力样本数 = 12→29.95 @ 20 Hz
    assert rep["flight_window"]["n_thrust_samples"] == 360


def test_推力时间基之外的样本不参与统计():
    """离地在 30 s 合上，之后那些样本必须被判在飞窗口外（不算进读数）。"""
    rep, _ = _run(_synth(thrust_z=-0.5, landed_off=(12.0, 20.0)))
    assert rep["usable"]
    assert abs(rep["hover_thrust"] - 0.5) < 1e-9
    assert rep["flight_window"]["t1"] <= 20.0


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
