#!/usr/bin/env python3
"""`tau_core` 的自检 —— **不需要飞机、不需要日志、不需要飞控通电**。

判据全部是**合成信号**上的真值对：给一个已知的 τ，看能不能原样收回来。
跑法：

    python3 tools/tau-analysis/test_tau_core.py --self-test
    # 或（仓库惯例）
    pytest tools/tau-analysis/test_tau_core.py
"""

from __future__ import annotations

import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

from tau_core import (  # noqa: E402
    estimate_lag,
    estimate_lag_segmented,
    lowpass_zero_phase,
    peak_lag,
    quat_body_down,
    quat_mul,
    quat_to_rate,
    resample_uniform,
    xcorr_series,
)


def _rich_signal(t, n_axes=3, f_max=3.0, seed=0):
    """一条"像飞行"的多通道信号：若干正弦叠加 + 少量噪声。"""
    rng = np.random.default_rng(seed)
    out = np.zeros((t.size, n_axes))
    for ax in range(n_axes):
        for _ in range(4):
            f = rng.uniform(0.2, f_max)
            ph = rng.uniform(0, 2 * np.pi)
            amp = rng.uniform(0.3, 1.0)
            out[:, ax] += amp * np.sin(2 * np.pi * f * t + ph)
    out += 0.01 * rng.standard_normal(out.shape)
    return out


def _delay_signal(t_src, x_src, t_dst, tau):
    """把 `x_src(t)` 平移 `tau` 后重采样到 `t_dst`（模拟"动捕比 IMU 晚 τ"）。"""
    out = np.empty((t_dst.size, x_src.shape[1]))
    for c in range(x_src.shape[1]):
        out[:, c] = np.interp(t_dst - tau, t_src, x_src[:, c], left=0.0, right=0.0)
    return out


# ------------------------------------------------------------------ 用例


def test_纯滞后能被原样收回():
    """核心判据：给 40 ms 的滞后，跨采样率也必须收回到 5 ms 内。"""
    t_a = np.arange(0.0, 20.0, 1 / 250)          # 飞控 IMU 侧 ~250 Hz
    a = _rich_signal(t_a, seed=1)
    tau_true = 0.040
    t_b = np.arange(0.0, 20.0, 1 / 30)           # 动捕侧 ~30 Hz
    b = _delay_signal(t_a, a, t_b, tau_true)
    est = estimate_lag(t_a, a, t_b, b, dt=0.002, max_lag_s=0.15, fc=5.0)
    assert abs(est.lag_s - tau_true) < 0.005, est
    assert est.r_peak > 0.8, est
    # ★ 换个搜索窗再看一遍 —— τ 与窗宽无关（这条是本模块唯一的"锐度"判据）。
    est_wide = estimate_lag(t_a, a, t_b, b, dt=0.002, max_lag_s=0.3, fc=5.0)
    assert abs(est_wide.lag_s - tau_true) < 0.005, est_wide
    assert abs(est.lag_s - est_wide.lag_s) < 0.003, (est, est_wide)


def test_常量序列拒绝给数而不是报窗边缘():
    """★ 飞机没动时，`argmax(|r|)` 只能挑噪声，τ 会稳定贴住 ±max_lag（窗边缘），
    看着像个正经数字。实测在"飞机没动"的 SITL 日志上正是这么表现的 ⟹ 零方差先拒。
    """
    t = np.arange(0.0, 20.0, 1 / 250)
    a = _rich_signal(t, seed=10)
    b = np.zeros((np.arange(0.0, 20.0, 1 / 30).size, 3))  # 动捕侧一个字节都没动
    try:
        estimate_lag(t, a, np.arange(0.0, 20.0, 1 / 30), b, dt=0.002, max_lag_s=0.3, fc=5.0)
    except ValueError as exc:
        assert "常量" in str(exc)
    else:
        raise AssertionError("应当拒绝")


def test_锐度不是可信度_它只是在量搜索窗():
    """★ 钉住一个**被否掉的口径**，免得日后有人觉得"加个锐度更专业"又加回来。

    `|r_peak| / std(r)` 看上去像"峰有多尖"，实际是"窗开多宽"：
    同一份数据、同一个 τ，窗一放宽它就往下掉。所以模块里没有它。
    """
    t_a = np.arange(0.0, 20.0, 1 / 250)
    a = _rich_signal(t_a, seed=9)
    t_b = np.arange(0.0, 20.0, 1 / 30)
    b = _delay_signal(t_a, a, t_b, 0.040)
    dt = 0.002
    _, A = resample_uniform(t_a, a, dt)
    _, B = resample_uniform(t_b, b, dt)
    A = lowpass_zero_phase(A, dt, 5.0)
    B = lowpass_zero_phase(B, dt, 5.0)
    n = min(A.shape[0], B.shape[0])
    A, B = A[:n], B[:n]  # 两侧网格长度差 1~2 个点，对齐（`estimate_lag` 里也是这么做的）

    lags_n, r_n = xcorr_series(A, B, dt, 0.15)
    lags_w, r_w = xcorr_series(A, B, dt, 0.30)
    sharp_n = abs(r_n).max() / r_n.std()
    sharp_w = abs(r_w).max() / r_w.std()
    # 窗一放宽它就掉。**掉多少依信号而定**（这条用例上 ~0.26，换个信号见过 7.7→2.3），
    # 所以这里只能钉方向、不能钉幅度 —— 而"幅度依信号而定"本身就是它不能当判据的理由。
    assert sharp_n > sharp_w + 0.05, (sharp_n, sharp_w)
    # 而 τ 两窗一致：
    assert abs(peak_lag(lags_n, r_n)[0] - 0.040) < 0.005
    assert abs(peak_lag(lags_w, r_w)[0] - 0.040) < 0.005


def test_两条序列起点不同不能混进τ():
    """★ 钉住一个**真的发生过**的 bug：`resample_uniform` 曾经各按**自己的** `t[0]`
    建网格，两条序列被错开 `t_b[0] − t_a[0]`，这个错位**一分不少地加进 τ**。

    构造要点：让**粗采样那一侧（动捕，30 Hz）先开始**，裁剪到共同区间后，
    细采样侧（IMU，164 Hz）的首样本会落在它之后将近**一整个 30 Hz 周期（≈33 ms）**。
    真 τ 是 0 —— 旧实现会收回 ~33 ms，且 r 照样接近 1（整个峰被平移了，看不出破绽）。
    """
    dt_a, dt_b = 1 / 164, 1 / 30
    t_a = np.arange(0.0001, 20.0, dt_a)   # IMU 侧晚 0.1 ms 起采
    a = _rich_signal(t_a, seed=12)
    t_b = np.arange(0.0, 20.0, dt_b)      # 动捕侧从 0 起采（更粗、更早）
    b = _delay_signal(t_a, a, t_b, 0.0)   # 真滞后 = 0
    est = estimate_lag(t_a, a, t_b, b, dt=0.002, max_lag_s=0.15, fc=5.0)
    assert abs(est.lag_s) < 0.005, est

    # 正控：真给 40 ms，仍要原样收回（说明上面不是因为"什么都不报"才过）
    b2 = _delay_signal(t_a, a, t_b, 0.040)
    est2 = estimate_lag(t_a, a, t_b, b2, dt=0.002, max_lag_s=0.15, fc=5.0)
    assert abs(est2.lag_s - 0.040) < 0.005, est2


def test_分段估计彼此对得上():
    """分段 τ 的**离散度**才是"这个数稳不稳"的证据。"""
    t_a = np.arange(0.0, 30.0, 1 / 250)
    a = _rich_signal(t_a, seed=7)
    t_b = np.arange(0.0, 30.0, 1 / 30)
    b = _delay_signal(t_a, a, t_b, 0.045)
    ests, med, span = estimate_lag_segmented(
        t_a, a, t_b, b, n_segments=6, dt=0.002, max_lag_s=0.2, fc=5.0
    )
    assert len(ests) >= 4, ests
    assert abs(med - 0.045) < 0.006, (med, ests)
    assert span < 0.02, (span, ests)


def test_分段不会把预热扣到每一段头上():
    """★ 钉住一个**静默**的坑：把 `warmup_s` 透传给每一段，
    21.9 s 窗口切 6 段、每段 3.65 s 再各扣 3 s ⟹ 全被"重叠太短"拒掉，
    分段返回空，看着像"这段数据不支持分段"，其实是这里扣重了。起飞瞬态只发生一次。
    """
    t_a = np.arange(0.0, 22.0, 1 / 250)
    a = _rich_signal(t_a, seed=11)
    t_b = np.arange(0.0, 22.0, 1 / 30)
    b = _delay_signal(t_a, a, t_b, 0.045)
    ests, med, span = estimate_lag_segmented(
        t_a, a, t_b, b, n_segments=6, dt=0.002, max_lag_s=0.2, fc=5.0, warmup_s=3.0
    )
    assert len(ests) >= 4, f"分段被扣成空的了：{ests}"
    assert abs(med - 0.045) < 0.008, (med, ests)


def test_激励量必须沿时间去均值():
    """★ 钉住**两个**都实测验过是错的激励口径：

    · 整个数组的 **RMS**：`quat_body_down` 按构造是单位向量 ⟹ 恒等于 1，
      飞机停着还是翻跟头都是 1（实测 B 三段全是 1.00）。
    · 整个数组的 **std**：量的是**通道之间**的差异，不是**随时间**的起伏 ——
      `body_down` 的 z 恒为 1、x/y 恒为 0 ⟹ 恒 ≈ 0.47，静止与翻滚分不开
      （实测 B 三段全是 0.468/0.474/0.468）。

    只有"逐通道沿时间去均值"才是在量"动没动"。
    """
    from tau_core import _excitation

    t = np.arange(0.0, 20.0, 0.002)
    still = np.tile(np.array([0.0, 0.0, 1.0]), (t.size, 1))  # 机体一动不动
    ang = 0.5 * np.sin(2 * np.pi * 0.6 * t)
    moving = quat_body_down(
        np.stack([np.cos(ang / 2), np.sin(ang / 2), np.zeros_like(t), np.zeros_like(t)], axis=-1)
    )

    # 两个错口径都分不开：
    assert abs(np.sqrt((still**2).sum(1).mean()) - np.sqrt((moving**2).sum(1).mean())) < 0.05
    assert abs(still.std() - moving.std()) < 0.05, (still.std(), moving.std())
    # 正确口径分得开：
    assert _excitation(still) < 1e-9 < _excitation(moving), (_excitation(still), _excitation(moving))

    # 速率型信号上它随幅度线性 —— 钉的是"它是个幅度量"，不钉任何通道合并的约定
    rng = np.random.default_rng(13)
    gyro = rng.standard_normal((t.size, 3)) * 0.3
    assert abs(_excitation(gyro * 2.0) - 2.0 * _excitation(gyro)) < 1e-9
    assert _excitation(gyro * 0.01) < 0.1 * _excitation(gyro) < _excitation(gyro)


def test_零相位滤波不会自己搬动峰():
    """滤波器若带群延迟，τ 会被它改掉 —— 这里钉住「滤波前后结论一致」。"""
    t_a = np.arange(0.0, 20.0, 1 / 250)
    a = _rich_signal(t_a, seed=2)
    t_b = np.arange(0.0, 20.0, 1 / 30)
    b = _delay_signal(t_a, a, t_b, 0.040)
    with_fc = estimate_lag(t_a, a, t_b, b, dt=0.002, max_lag_s=0.15, fc=5.0)
    no_fc = estimate_lag(t_a, a, t_b, b, dt=0.002, max_lag_s=0.15, fc=None)
    assert abs(with_fc.lag_s - no_fc.lag_s) < 0.003, (with_fc, no_fc)
    # 正控：真的低通确实压掉了高频（说明滤波动过手，不是空操作）
    dt = 0.002
    _, A = resample_uniform(t_a, a, dt)
    assert np.std(lowpass_zero_phase(A, dt, 0.5)) < np.std(lowpass_zero_phase(A, dt, 5.0))


def test_没有延迟时报零():
    t_a = np.arange(0.0, 15.0, 1 / 250)
    a = _rich_signal(t_a, seed=3)
    b = _delay_signal(t_a, a, t_a.copy(), 0.0)
    est = estimate_lag(t_a, a, t_a, b, dt=0.002, max_lag_s=0.1, fc=5.0)
    assert abs(est.lag_s) < 0.003, est


def test_符号反了要报负峰而不是找不到():
    """负峰是**要报告的事实**（坐标系约定相反），不能被 abs() 抹掉。"""
    t_a = np.arange(0.0, 15.0, 1 / 250)
    a = _rich_signal(t_a, seed=4)
    t_b = np.arange(0.0, 15.0, 1 / 30)
    b = -_delay_signal(t_a, a, t_b, 0.030)
    est = estimate_lag(t_a, a, t_b, b, dt=0.002, max_lag_s=0.15, fc=5.0)
    assert est.r_peak < -0.8, est
    assert abs(abs(est.lag_s) - 0.030) < 0.005, est


def test_抛物线插值给出亚采样峰():
    lags = np.arange(-5, 6, dtype=float) * 0.002
    r = np.exp(-(((lags - 0.001) / 0.006) ** 2))  # 真峰在 +1 ms，网格是 2 ms
    lag, rp = peak_lag(lags, r)
    assert abs(lag - 0.001) < 0.0005, lag
    assert rp > 0.97, rp  # 网格上最接近真峰的样本值（0.9726），不是 1.0


def test_同一条序列自相关恒为一():
    t = np.arange(0.0, 10.0, 0.002)
    lags, r = xcorr_series(t[:, None], t[:, None], 0.002, 0.01)
    assert r.size == 11
    assert np.allclose(np.abs(r), 1.0)


def test_四元数微分能还原已知角速度():
    """绕体轴 x 以 ω=0.8 rad/s 匀速转，`quat_to_rate` 必须原样还回来。"""
    t = np.arange(0.0, 2.0, 1 / 200)
    w = np.array([0.8, 0.0, 0.0])
    ang = np.linalg.norm(w) * t
    q = np.stack(
        [np.cos(ang / 2), np.sin(ang / 2) * 1.0, np.zeros_like(t), np.zeros_like(t)], axis=-1
    )
    t_out, omega = quat_to_rate(t, q)
    assert np.allclose(omega, w, atol=1e-6), omega[:, 0].mean()


def test_四元数微分对双覆盖免疫():
    """q 与 -q 是同一姿态；差分不能因此冒出一个巨大的假角速度。"""
    t = np.arange(0.0, 2.0, 1 / 200)
    ang = 0.5 * t
    q = np.stack([np.cos(ang / 2), np.sin(ang / 2), np.zeros_like(t), np.zeros_like(t)], axis=-1)
    q_neg = -q
    _, w1 = quat_to_rate(t, q)
    _, w2 = quat_to_rate(t, q_neg)
    assert np.allclose(w1, w2, atol=1e-9)


def test_记在左端会凭空多出半个采样周期():
    """★ 这是**刻意钉住的坑**：把 30 Hz 的半个周期（16.7 ms）挪进时间标记，τ 就跟着挪 16.7 ms。

    ⚠️ 本用例走的是**后果**（把 `t_b` 手动平移半个周期，看 τ 跟着变），
    它证明"半个周期确实会一分不少地加进 τ"。
    `attrib="left"` **那个分支本身**由下一条 `test_left只是个时间标记` 直测 ——
    两条互补，别把本条当成在测 `attrib`。
    """
    period = 1 / 30
    t_a = np.arange(0.0, 20.0, 1 / 250)
    a = _rich_signal(t_a, seed=5)
    t_b = np.arange(0.0, 20.0, period)
    b = _delay_signal(t_a, a, t_b, 0.040)
    mid = estimate_lag(t_a, a, t_b, b, dt=0.002, max_lag_s=0.2, fc=5.0)
    left = estimate_lag(t_a, a, t_b + period / 2, b, dt=0.002, max_lag_s=0.2, fc=5.0)
    assert abs((left.lag_s - mid.lag_s) - period / 2) < 0.006, (mid, left)


def test_left只是个时间标记():
    """`attrib` **只改时间标记，不改角速度本身** —— 直测 `quat_to_rate` 的这个分支。

    它有实体代码（`t_out = t[:-1] if attrib == "left" else ...`），
    所以必须有一条用例真的走到它；否则"别用 left"这句话没有任何东西钉着。
    """
    t = np.arange(0.0, 2.0, 1 / 30)
    ang = 0.4 * t
    q = np.stack([np.cos(ang / 2), np.sin(ang / 2), np.zeros_like(t), np.zeros_like(t)], axis=-1)

    t_mid, w_mid = quat_to_rate(t, q, attrib="midpoint")
    t_left, w_left = quat_to_rate(t, q, attrib="left")

    assert np.allclose(w_left, w_mid), "attrib 不该动 ω"
    assert np.allclose(t_left, t[:-1]), "left 应记在区间左端 t_i"
    assert np.allclose(t_mid, 0.5 * (t[:-1] + t[1:])), "midpoint 应记在区间中点"
    # 左端比中点早半个采样周期 ⟹ 相对另一条序列，动捕被**凭空推迟**了半个周期
    assert np.allclose(t_mid - t_left, 0.5 / 30)

    try:
        quat_to_rate(t, q, attrib="右边")
    except ValueError:
        pass
    else:  # pragma: no cover
        raise AssertionError("非法 attrib 应当直接报错，而不是被当成 midpoint 默默接受")


def test_体轴下向量对偏航不敏感():
    """绕世界 z 转任意角度，`quat_body_down` 必须一个字都不变。

    这条是"动捕参考系的偏航是任意的、但倾斜读数仍然可比"的依据。
    """
    q0 = np.array([np.cos(0.4), np.sin(0.4), 0.0, 0.0])  # 绕 x 倾 0.8 rad
    d0 = quat_body_down(q0)
    for psi in (0.3, 1.7, -2.5):
        qyaw = np.array([np.cos(psi / 2), 0.0, 0.0, np.sin(psi / 2)])
        d1 = quat_body_down(quat_mul(qyaw, q0))
        assert np.allclose(d0, d1, atol=1e-12), (psi, d0, d1)


def test_重采样保形状与端点():
    t = np.array([0.0, 0.3, 0.7, 1.0])
    x = np.array([[0.0], [3.0], [7.0], [10.0]])
    grid, out = resample_uniform(t, x, 0.1)
    assert out.shape == (11, 1)
    assert out[0, 0] == 0.0 and abs(out[-1, 0] - 10.0) < 1e-9
    assert abs(out[3, 0] - 3.0) < 1e-9


def test_形状对不上要报错():
    try:
        xcorr_series(np.zeros((10, 3)), np.zeros((10, 2)), 0.002, 0.01)
    except ValueError as exc:
        assert "形状" in str(exc)
    else:
        raise AssertionError("应当报错")


def test_重叠太短要拒绝给数():
    """重叠区不够长时"峰"是拼出来的 —— 宁可拒绝，也不要给一个像样的假数。"""
    t = np.arange(0.0, 1.0, 0.002)  # 只有 1 s
    a = _rich_signal(t, seed=8)
    try:
        estimate_lag(t, a, t, a, dt=0.002, max_lag_s=0.3, fc=5.0)
    except ValueError as exc:
        assert "重叠" in str(exc)
    else:
        raise AssertionError("应当拒绝")


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
