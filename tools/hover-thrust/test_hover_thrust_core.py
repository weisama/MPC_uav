#!/usr/bin/env python3
"""`hover_thrust_core` 的自检 —— **不需要飞机、不需要日志、不需要飞控通电**。

判据全部是**合成信号**上的真值对：给一段已知的"稳态悬停"，看能不能原样收回来。
另有若干条**钉住被否掉的方案**的用例（均值、夹逼插值、静默取绝对值、抖动当机动）——
改动它们会立刻红。

跑法：

    python3 tools/hover-thrust/test_hover_thrust_core.py --self-test
    # 或（仓库惯例）
    pytest tools/hover-thrust/test_hover_thrust_core.py
"""

from __future__ import annotations

import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

from hover_thrust_core import (  # noqa: E402
    estimate_hover_thrust,
    interp_to,
    relative_gap,
    runs_of,
    steady_hover_mask,
    sustained_error_bound,
    t_max_for,
    tilt_rad_from_quat,
)

#: SITL 的 z0 同构机里那条**独立推导出来**的真值：`0.75 × 9.80665 / 2.95 / 4`
#: （出处 `tools/px4-sitl/z0-plant/plant_spec.py` 与 `sitl-z0-plant.md` §5.3）。
#: 合成用例拿它当"已知的悬停推力"，与真机日志无关。
TRUTH = 0.62330


def _hover_series(thrust_z=-TRUTH, n=400, dt=0.05, speed=0.0, tilt_deg=0.0, jitter=0.005, seed=1):
    """造一条**稳态悬停**：速度 0、倾角 0、`thrust_body[2]` 在真值附近抖。

    `jitter` 是**必须**有的：真机上 20 Hz 记录的控制量没有一拍不动的时候
    （实测 SITL 稳态段 std ≈ 0.02）。造一条完全平的线去测"抖动会不会误判"，
    等于没测。
    """
    rng = np.random.default_rng(seed)
    t = np.arange(n) * dt
    return (
        t,
        np.full(n, float(speed)),
        np.full(n, math.radians(tilt_deg)),
        np.full(n, float(thrust_z)) + jitter * rng.standard_normal(n),
    )


# ------------------------------------------------------------------ 姿态


def test_水平姿态的倾角是零():
    assert abs(float(tilt_rad_from_quat([1.0, 0.0, 0.0, 0.0]))) < 1e-12


def test_绕x轴滚转30度就是30度倾角():
    a = math.radians(30.0)
    q = [math.cos(a / 2), math.sin(a / 2), 0.0, 0.0]
    assert abs(math.degrees(float(tilt_rad_from_quat(q))) - 30.0) < 1e-9


def test_偏航_不影响倾角():
    """★ 纯偏航不是机动。绕 z 转 90° 的倾角必须仍是 0 —— 否则悬停中一偏航就被判掉。"""
    a = math.radians(90.0)
    q = [math.cos(a / 2), 0.0, 0.0, math.sin(a / 2)]
    assert abs(float(tilt_rad_from_quat(q))) < 1e-9


# ------------------------------------------------------------------ 插值


def test_插值落到源区间之外给NaN_不夹逼():
    """★ 钉住夹逼：夹逼会**造出**一段完美的悬停数据（速度 0、倾角 0），图上看不出来。"""
    out = interp_to([0.0, 1.0, 2.0], [0.0, 1.0, 2.0], [-0.5, 0.5, 2.5])
    assert np.isnan(out[0]) and np.isnan(out[2])
    assert abs(out[1] - 0.5) < 1e-12


def test_NaN_样本不进稳态掩码():
    t, speed, tilt, thrust = _hover_series(n=100)
    speed = speed.copy()
    speed[10:20] = np.nan
    mask = steady_hover_mask(t, speed, tilt, thrust)
    assert not mask[10:20].any()
    assert mask.sum() == 90


# ------------------------------------------------------------------ 稳态判据


def test_速度大的段被排除():
    t, speed, tilt, thrust = _hover_series(n=400)
    speed = speed.copy()
    speed[200:] = 1.0  # 后半段在飞
    est = estimate_hover_thrust(t, speed, tilt, thrust)
    assert est.usable
    assert est.segment.n == 200
    assert abs(est.hover_thrust - TRUTH) < 0.01


def test_慢速漂移仍算悬停():
    """★ 钉住"稳态"≠"速度严格为零"。

    真机动捕在环的位置环本来就允许厘米/秒级漂移；SITL 的"悬停"更是一条慢漂轨迹
    （实测该日志在飞窗口中位速度 0.17 m/s）。把 `v_eps` 卡在 0 附近 ⟹ 判出"日志里没有悬停"，
    而数据其实好得很。速度只是粗筛，精度由倾角那一项负责（见下一条）。
    """
    t, speed, tilt, thrust = _hover_series(n=400, speed=0.25)
    est = estimate_hover_thrust(t, speed, tilt, thrust)
    assert est.usable
    assert est.segment.n == 400
    assert relative_gap(est.hover_thrust, TRUTH) < 0.001


def test_默认倾角上限对应的推力误差在2个百分点内():
    """★ `tilt_eps` 不是"宽容度"，它是**读数的误差上界**：侧倾 θ 时竖直分量按 cos θ 拿。

    所以这个默认值只能往小改、不能随手往大改 —— 放到 30° 就是 13.4% 的推力增益误差，
    正是本张票要防的那种"悄悄错掉的增益"。改这里必须同时改文档里的误差界。
    """
    import inspect

    tilt_eps = inspect.signature(steady_hover_mask).parameters["tilt_eps"].default
    bound = 1.0 - math.cos(tilt_eps)
    assert bound < 0.02, f"默认倾角上限 {math.degrees(tilt_eps):g}° 的误差上界 {bound:.1%} 超了 2%"
    assert abs(bound - 0.0152) < 5e-4  # 10° ⟹ 1.52%，与文档里写的一致


def test_速度门与段长绑在一条误差界上():
    """★ `v_eps` / `min_duration` **不能各改各的** —— 它们由 `δ ≤ 2·v_eps/(g·T)` 绑着。

    这条界说的是"整段持续偏掉"能有多大：段里每个样本都要满足 `|v| ≤ v_eps`，
    所以速度变化 ≤ `2·v_eps`，而持续偏 `δ` 需要恒定 `a_z = δ·g`。
    默认值给 3.1%；谁把 `v_eps` 放宽到 0.6 而不同时加长段长，就会红在这条上。
    """
    import inspect

    sig = inspect.signature(steady_hover_mask).parameters
    v_eps = sig["v_eps"].default
    min_duration = inspect.signature(estimate_hover_thrust).parameters["min_duration"].default

    bound = sustained_error_bound(v_eps, min_duration)
    assert abs(bound - sustained_error_bound()) < 1e-12  # 与默认值那条一致
    assert 0.02 < bound < 0.05, f"默认误差界 {bound:.1%} 不在 2–5% 这个量级上"
    # 用它作"载荷余量"的判据：界若涨到 5% 以上，这一票的真实参数就不该采用
    assert sustained_error_bound(0.60, 2.0) > 0.05, "放宽 v_eps 必须能把界顶出去，用例才有意义"


def test_倾角大的段被排除():
    t, speed, tilt, thrust = _hover_series(n=400)
    tilt = tilt.copy()
    tilt[200:] = math.radians(25.0)
    est = estimate_hover_thrust(t, speed, tilt, thrust)
    assert est.usable
    assert est.segment.n == 200


def test_打满推力的段不算悬停():
    t, speed, tilt, thrust = _hover_series(n=400)
    thrust = thrust.copy()
    thrust[200:] = -0.999
    est = estimate_hover_thrust(t, speed, tilt, thrust)
    assert est.segment.n == 200


def test_地面怠速段不算悬停():
    """`thrust_body` 贴近 0 的那一段是停桨/怠速，不是悬停。"""
    t, speed, tilt, thrust = _hover_series(n=400)
    thrust = thrust.copy()
    thrust[:200] = -0.002
    est = estimate_hover_thrust(t, speed, tilt, thrust)
    assert est.segment.n == 200


def test_段太短不算():
    """1 s 的"悬停"不够 —— 位置环的积分还没把差补上。"""
    t, speed, tilt, thrust = _hover_series(n=100, dt=0.05)  # 共 5 s
    thrust = thrust.copy()
    thrust[:] = -0.02  # 全段都不是悬停
    thrust[40:60] = -TRUTH  # 中间 1 s 像悬停
    est = estimate_hover_thrust(t, speed, tilt, thrust, min_duration=2.0)
    assert est.hover_thrust is None
    assert not est.usable


# ------------------------------------------------------------------ 读数


def test_合成悬停段能原样收回():
    t, speed, tilt, thrust = _hover_series()
    est = estimate_hover_thrust(t, speed, tilt, thrust)
    assert est.usable
    assert relative_gap(est.hover_thrust, TRUTH) < 0.001


def test_取中位数_不被尖峰拉偏():
    """★ 钉住"用中位数而不是均值"。

    10 % 的拍打到 0.95（仍在上限内、不会被掩码丢掉），均值会被抬到 **0.653**，
    中位数仍是 0.6233。判据卡在 0.005 上 ⟹ 换成均值立刻红。
    """
    t, speed, tilt, thrust = _hover_series(n=400, jitter=0.0)
    thrust = thrust.copy()
    thrust[::10] = -0.95
    est = estimate_hover_thrust(t, speed, tilt, thrust)
    assert abs(est.hover_thrust - TRUTH) < 0.005
    assert abs(float(np.mean(-thrust)) - TRUTH) > 0.02  # 均值确实偏了，用例才有意义


def test_符号为正时不静默取绝对值():
    """★ 悬停时 `thrust_body[2]` 必须为负；为正说明约定错了，**不能**当 |x| 用。"""
    t, speed, tilt, thrust = _hover_series(thrust_z=+TRUTH)
    est = estimate_hover_thrust(t, speed, tilt, thrust)
    assert est.sign == "positive"
    assert est.hover_thrust is None
    assert not est.usable


def test_段内不一致超限判不可用():
    """电池掉压 / 积分还在爬 ⟹ 这一段里"悬停推力"不是一个数。"""
    t, speed, tilt, thrust = _hover_series(n=400, jitter=0.0)
    thrust = thrust.copy()
    thrust[200:] = -0.72
    est = estimate_hover_thrust(t, speed, tilt, thrust)
    assert est.hover_thrust is not None
    assert not est.usable
    assert any("段内不一致" in s for s in est.notes)


def test_段首台阶不会被漏掉():
    """★ 钉住"切成多份取极差"，而不是"前后半段之差"。

    台阶落在段首 1/6 处（0.6233 → 0.7480）时，**前半段的中位数已经被台阶后面主导**，
    前后之差恰好归零；切成 5 份取极差则抓得到。这条用例就是那个反例。
    """
    t, speed, tilt, thrust = _hover_series(n=800, jitter=0.0)
    thrust = thrust.copy()
    thrust[130:] = -0.7480  # 下界 1/6 处起跳
    front = -float(np.median(thrust[:400]))
    back = -float(np.median(thrust[400:]))
    assert abs(front - back) < 1e-9  # 前后半段之差确实归零 —— 用例才有意义
    est = estimate_hover_thrust(t, speed, tilt, thrust)
    assert not est.usable
    assert any("段内不一致" in s for s in est.notes)


def test_两段一致时段间不报警():
    t, speed, tilt, thrust = _hover_series(n=800, jitter=0.001)
    thrust = thrust.copy()
    thrust[300:400] = -0.02  # 中间插一段"不悬停"，切成两段
    est = estimate_hover_thrust(t, speed, tilt, thrust)
    assert est.usable
    assert len(est.segments) == 2
    assert est.segment_spread < 0.01


def test_两段不一致时段间报警():
    """★ 换个段就换个答案 = 它在日志里本来就不是一个数 —— 必须报出来，不许挑一个交差。

    ⚠️ 这段数据里**段是不等长的**（300 vs 400）：它同时钉住"不许用合并中位数当交叉口径"
    —— 合并中位数会等于占多数的那段，于是两边永远"一致"，报警永远不会响。
    """
    t, speed, tilt, thrust = _hover_series(n=800, jitter=0.0)
    thrust = thrust.copy()
    thrust[300:400] = -0.02  # 5 s 的"不悬停"把日志切成两段
    thrust[400:] = -0.70
    est = estimate_hover_thrust(t, speed, tilt, thrust)
    assert len(est.segments) == 2
    assert est.hover_thrust == max(s.thrust for s in est.segments)  # 主口径 = 最长那段
    pooled = -float(np.median(np.concatenate([thrust[:300], thrust[400:]])))
    assert relative_gap(pooled, est.hover_thrust) < 1e-9  # 合并中位数骗人：它=主口径
    assert not est.usable
    assert any("段间不一致" in s for s in est.notes)


def test_空输入不抛():
    empty = np.zeros(0)
    est = estimate_hover_thrust(empty, empty, empty, empty)
    assert est.hover_thrust is None
    assert not est.usable
    assert runs_of(np.zeros(0, dtype=bool), empty) == []


def test_长度对不上要报错():
    t, speed, tilt, thrust = _hover_series(n=100)
    try:
        steady_hover_mask(t, speed[:-1], tilt, thrust)
    except ValueError:
        return
    raise AssertionError("长度对不上应当抛 ValueError，而不是静默广播")


# ------------------------------------------------------------------ 导出关系


def test_t_max_导出关系与params一致():
    """`T_max = m·g / hover_thrust` —— 与 `nmpc/params.py` 的 `Z0Params.t_max` 同一条。

    占位值那套：`0.75 × 9.80665 / 0.6235` = 11.7964 N（`params.py` 用 g=9.81 得 11.80）。
    """
    assert abs(t_max_for(0.6235, 0.75) - 11.7964) < 5e-4
    assert relative_gap(t_max_for(0.6235, 0.75), 0.75 * 9.81 / 0.6235) < 1e-3


def test_hover_thrust_越界要报错():
    for bad in (0.0, 1.0, -0.2, 1.5):
        try:
            t_max_for(bad, 0.75)
        except ValueError:
            continue
        raise AssertionError(f"hover_thrust={bad} 应当抛 ValueError")


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
