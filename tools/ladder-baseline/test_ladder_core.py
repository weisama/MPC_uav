#!/usr/bin/env python3
"""`ladder_core` 的自检 —— **不需要包、不需要飞机、不需要开 ROS**。

判据全部是**合成信号**上的真值对：给一个已知的滞后，看能不能原样收回来；
给一个已知的常量偏移，看指标是不是它。跑法：

    python3 tools/ladder-baseline/test_ladder_core.py --self-test
    # 或（仓库惯例）
    pytest tools/ladder-baseline/test_ladder_core.py
"""

from __future__ import annotations

import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

from ladder_core import (  # noqa: E402
    anchor_index,
    best_shift,
    interp_columns,
    make_grid,
    path_length,
    summarize,
    track_window,
    tracking_metrics,
    trim_window,
)

# 合成信号：半径 1 m、0.1 Hz 的圆 ⟹ 线速度 2π·0.1·1 = 0.628 m/s，全程动、没有静止段
_R, _F = 1.0, 0.1


def _circle(t):
    return np.stack([_R * np.cos(2 * np.pi * _F * t), _R * np.sin(2 * np.pi * _F * t),
                     np.full_like(t, 1.0)], axis=1)


def _lagged(t, dt):
    """把圆**滞后** `dt` 秒（`dt > 0`）：实际(t) = 指令(t − dt)。

    ★ 上层的 `t` 刻意比窗口长，好让对齐搜索的 `dt` 范围内**不触发端点夹取** ——
    否则测的是"夹取能不能凑出低误差"，不是"滞后能不能被收回"。
    这里**不做 clip**：夹取是 `interp_columns` 端点行为，让它在被测的那一层发生。
    """
    return _circle(t - dt)


# ------------------------------------------------------------------ 窗口


def test窗口取最长连续段():
    """两段 `AUTO/TRACK`，要取**长的那段**；首末取差会把中间的非 TRACK 段吞进来。"""
    states = [(0.0, "AUTO/WAIT_START"), (1.0, "AUTO/TRACK")]
    states += [(1.0 + 0.1 * k, "AUTO/TRACK") for k in range(1, 21)]        # 1.0 → 3.0
    states += [(3.1 + 0.1 * k, "AUTO/HOLD") for k in range(0, 30)]         # 3.1 → 6.0
    states += [(6.1 + 0.1 * k, "AUTO/TRACK") for k in range(0, 51)]        # 6.1 → 11.1
    win = track_window(states)
    assert win is not None
    assert abs(win[0] - 6.1) < 1e-9, win
    assert abs(win[1] - 11.1) < 1e-9, win
    # 反例：首末取差会给出 (1.0, 11.1)，把 6.1 之前的 HOLD 段一起算进去
    assert win[1] - win[0] < 11.1 - 1.0 - 2.9


def test没有TRACK段要返回None():
    assert track_window([(0.0, "AUTO/WAIT_START"), (1.0, "MANUAL/ARMED")]) is None


def test窗口中间的断档要切开():
    """`AUTO/TRACK` 断档超过 `_GAP_S` 就是两段，不能被连成一段。

    ★★ 采样间隔按**真实的 1 Hz** 造（`/control/state` 实测中位 1000 ms）——
    造 0.1 s 间隔的话，`_GAP_S` 只要小于 0.1 s 就会把每拍切成一段，
    这条用例会以"窗口塌成一点"的形式失败，而失败的**原因看着像别的地方**。
    """
    states = [(float(k), "AUTO/TRACK") for k in range(0, 11)]          # 0 → 10
    states += [(15.0 + k, "AUTO/TRACK") for k in range(0, 16)]         # 15 → 30
    win = track_window(states)
    assert win is not None and abs(win[0] - 15.0) < 1e-9, win
    assert abs(win[1] - 30.0) < 1e-9, win


def test容忍连丢两拍不算断档():
    """1 Hz 的话题偶尔漏一拍（间隔 ~2 s）**不算**断开 —— 否则每条轨迹都被切成碎片。

    ★ 这条是 `_GAP_S` 的**下界**，上一条是它的上界；两条一起才钉住这个常数。
    """
    states = [(float(k), "AUTO/TRACK") for k in range(0, 6)]           # 0 → 5
    states += [(7.0 + k, "AUTO/TRACK") for k in range(0, 11)]          # 7 → 17（漏了一拍）
    win = track_window(states)
    assert win is not None and abs(win[0] - 0.0) < 1e-9, win
    assert abs(win[1] - 17.0) < 1e-9, win


# ------------------------------------------------------------------ 锚点与窗口裁剪


def test锚点取闸门打开前的最新一帧():
    """闸门 10.0 打开时，节点手里只有 9.9 那一帧；10.1 是**未来**，不能用。

    ★ 反例是"取最接近 `t_gate` 的那一帧"（会拿到 10.1）与"取第一帧"（会拿到 0.1）。
    两种错都会让参考整体偏一个真实位移，且在别的读数上完全看不出来。
    """
    t = np.array([0.1, 5.0, 9.9, 10.1, 20.0])
    assert anchor_index(10.0, t) == 2
    assert anchor_index(9.9, t) == 2          # 正好命中也要算"已收到"
    assert anchor_index(0.05, t) == 0          # 闸门比第一帧还早 ⟹ 用第一帧
    assert anchor_index(100.0, t) == 4


def test空测量取锚点要报错():
    try:
        anchor_index(1.0, np.array([]))
    except ValueError:
        return
    raise AssertionError("空序列应当报错，不能静默返回 0")


def test窗口裁剪只动左端():
    assert trim_window((10.0, 50.0), skip_s=5.0) == (15.0, 50.0)
    assert trim_window((10.0, 50.0), last_s=30.0) == (20.0, 50.0)
    # 两个都给：先切头，再留尾；取更靠右的那个起点
    assert trim_window((10.0, 50.0), skip_s=25.0, last_s=10.0) == (40.0, 50.0)
    # 切完为空必须报错 —— 静默的零长窗口会在 make_grid 里以别的原因炸
    try:
        trim_window((10.0, 50.0), skip_s=40.0)
    except ValueError:
        pass
    else:
        raise AssertionError("切完为空应当报错")


# ------------------------------------------------------------------ 网格与插值


def test网格含右端点且步长正确():
    g = make_grid(1.0, 2.0, 10.0)
    assert g[0] == 1.0
    assert abs(g[-1] - 2.0) < 1e-9, f"右端点被丢了：{g[-1]}"
    assert len(g) == 11


def test插值能原样还原线性函数():
    t = np.arange(0.0, 5.0, 0.01)
    x = np.stack([3.0 * t + 1.0, -2.0 * t, 0.5 * t - 7.0], axis=1)
    g = np.arange(0.5, 4.5, 0.037)
    got = interp_columns(t, x, g)
    want = np.stack([3.0 * g + 1.0, -2.0 * g, 0.5 * g - 7.0], axis=1)
    assert np.allclose(got, want, atol=1e-10), np.abs(got - want).max()


def test时间不单调要拒绝():
    t = np.array([0.0, 0.1, 0.1, 0.2])
    x = np.zeros((4, 3))
    try:
        interp_columns(t, x, np.array([0.0, 0.15]))
    except ValueError as exc:
        assert "单调" in str(exc), exc
    else:
        raise AssertionError("应当拒绝")


# ------------------------------------------------------------------ 指标


def test指标认得常量偏移():
    """★ 这条钉住**符号**：垂直误差是**带符号**的。

    搞反（`cmd − actual`）时 `z_mean` 会从 −0.02 变成 +0.02；
    取绝对值再平均时 `z_mean` 会变成 +0.02 —— 两种都立刻红。
    """
    t = np.arange(0.0, 4.0, 0.02)
    cmd = _circle(t)
    off = np.array([0.03, 0.04, -0.02])
    m = tracking_metrics(cmd + off, cmd)
    assert abs(m["xy_mean"] - 0.05) < 1e-12, m["xy_mean"]
    assert abs(m["xy_max"] - 0.05) < 1e-12, m["xy_max"]
    assert abs(m["z_mean"] + 0.02) < 1e-12, m["z_mean"]
    assert abs(m["z_absmean"] - 0.02) < 1e-12, m["z_absmean"]
    assert m["n"] == t.size


def test指标对上下对称的垂直误差给零均值():
    """上下各 3 cm 的振荡与总低 3 cm 的偏置，`z_mean` 必须分得开。"""
    t = np.arange(0.0, 4.0, 0.02)
    cmd = _circle(t)
    wob = cmd.copy()
    wob[:, 2] += 0.03 * np.sin(2 * np.pi * 1.0 * t)
    m = tracking_metrics(wob, cmd)
    assert abs(m["z_mean"]) < 1e-9, m["z_mean"]
    assert abs(m["z_absmean"] - 0.02) < 5e-3, m["z_absmean"]


# ------------------------------------------------------------------ 时间对齐


def test纯滞后能被原样收回():
    """核心判据：给 0.40 s 的滞后，`dt*` 必须收回到一个步长（0.02 s）内。"""
    t = np.arange(-5.0, 25.0, 0.005)
    g = make_grid(0.0, 12.0, 50.0)
    r = best_shift(t, _lagged(t, 0.40), t, _circle(t), g)
    assert abs(r["dt"] + 0.40) <= 0.02, r["dt"]
    # ★ 符号：滞后 ⟹ dt < 0（要往**回**取指令才对得上）。搞反会给出 +0.40。
    assert r["dt"] < 0, r["dt"]


def test对齐后残差远小于未对齐():
    t = np.arange(-5.0, 25.0, 0.005)
    g = make_grid(0.0, 12.0, 50.0)
    act = _lagged(t, 0.40)
    cmd = _circle(t)
    raw = tracking_metrics(interp_columns(t, act, g), interp_columns(t, cmd, g))
    r = best_shift(t, act, t, cmd, g)
    assert r["metrics"]["xy_mean"] < raw["xy_mean"] / 5.0, (r["metrics"]["xy_mean"], raw["xy_mean"])


def test零误差时最优偏移落在零附近():
    """实际 == 指令 ⟹ `dt*` 没有可分辨的峰，但也不该跑到搜索窗边上。"""
    t = np.arange(-5.0, 25.0, 0.005)
    g = make_grid(0.0, 12.0, 50.0)
    r = best_shift(t, _circle(t), t, _circle(t), g)
    assert abs(r["dt"]) <= 0.02, r["dt"]
    assert r["metrics"]["xy_mean"] < 1e-6, r["metrics"]["xy_mean"]


def test对齐不缩短网格():
    """★ 对齐只挪指令的时间轴，**不动**评估网格 —— 否则 (A)/(C) 两侧内缩量不同就不可比。

    ★★ 这条**必须用"指令只覆盖窗口"的那一档**才咬得住：指令两侧留得越宽，
    "把网格裁到共同区间"的实现裁掉的就越少，`n` 反而一样 —— 测试就变成摆设。
    实测过：只用宽覆盖那一档时，把 `best_shift` 改成逐档裁剪的变异体
    **14 项全过**（缺陷完全静默）。下面两档是修好之后补的。
    """
    n_expect = len(make_grid(0.0, 12.0, 50.0))
    t_wide = np.arange(-5.0, 25.0, 0.005)
    r_wide = summarize(t_wide, _lagged(t_wide, 0.40), t_wide, _circle(t_wide), (0.0, 12.0), hz=50.0)
    assert r_wide["unaligned"]["n"] == r_wide["aligned"]["metrics"]["n"] == n_expect
    # 窄覆盖：裁剪式实现会在这里缩掉一大截
    t_narrow = np.arange(0.0, 12.0, 0.005)
    r_narrow = summarize(t_narrow, _lagged(t_narrow, 0.40), t_narrow, _circle(t_narrow),
                         (0.0, 12.0), hz=50.0)
    assert r_narrow["aligned"]["metrics"]["n"] == n_expect, r_narrow["aligned"]["metrics"]["n"]


def test对齐曲线每一档都要给数():
    """偏移扫描**不许跳档** —— 跳掉的档位上如果恰好是极小值，最优点就被藏掉了。"""
    t = np.arange(-5.0, 25.0, 0.005)
    r = best_shift(t, _lagged(t, 0.40), t, _circle(t), make_grid(0.0, 12.0, 50.0),
                   lo_s=-2.5, hi_s=0.5, step_s=0.02)
    assert len(r["curve"]) == int(round((0.5 - (-2.5)) / 0.02)) + 1, len(r["curve"])


def test搜索范围超出指令覆盖要报出来():
    """指令序列只覆盖窗口本身时，负向偏移会**静默夹端点** —— 必须由旗子报出来。"""
    t = np.arange(0.0, 12.0, 0.005)          # 指令恰好只覆盖窗口
    r = summarize(t, _lagged(t, 0.40), t, _circle(t), (0.0, 12.0), hz=50.0)
    assert r["aligned"]["coverage_ok"] is False
    t2 = np.arange(-5.0, 25.0, 0.005)        # 两侧都留足
    r2 = summarize(t2, _lagged(t2, 0.40), t2, _circle(t2), (0.0, 12.0), hz=50.0)
    assert r2["aligned"]["coverage_ok"] is True


# ------------------------------------------------------------------ 路径长度


def test路径长度分得开慢与滞后():
    """同样 12 s：纯滞后 ⟹ 路径长度接近；走得慢 ⟹ 明显更短。

    这条是判据表里"0.57 m 是跟不上还是只是慢起机"的依据，必须真的能分开。
    """
    t = np.arange(-5.0, 25.0, 0.005)
    g = make_grid(0.0, 12.0, 50.0)
    cmd = interp_columns(t, _circle(t), g)
    lag = interp_columns(t, _lagged(t, 0.40), g)
    slow = interp_columns(t, _circle(0.7 * t), g)          # 全部减速到 70%
    ratio_lag = path_length(lag) / path_length(cmd)
    ratio_slow = path_length(slow) / path_length(cmd)
    assert 0.95 < ratio_lag < 1.05, ratio_lag
    assert ratio_slow < 0.75, ratio_slow


CASES = [v for k, v in sorted(globals().items()) if k.startswith("test")]


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
