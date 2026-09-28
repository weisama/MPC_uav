"""τ（端到端延迟）估计的**纯逻辑** —— 重采样、零相位滤波、归一化互相关、找峰。

本模块**不 import** `pyulog` / `pymavlink` / `rclpy`，也不读文件：
它只吃两条 `(t, x)` 序列，吐一个滞后估计。读取 ulog 的适配层在 `tau_from_ulog.py`。
这样"相关峰怎么找的"可以拿**合成信号**自检（`--self-test`），不需要飞机、不需要日志。

术语：τ 恒指**第二条（动捕）相对第一条（飞控 IMU）的滞后**，单位秒。
`xcorr_series` 返回的 `r(k) > 0` 表示此刻的两条序列正相关。
"""

from __future__ import annotations

import numpy as np

__all__ = [
    "resample_uniform",
    "lowpass_zero_phase",
    "quat_mul",
    "quat_inv",
    "quat_to_rate",
    "quat_body_down",
    "xcorr_series",
    "peak_lag",
    "estimate_lag",
    "estimate_lag_segmented",
    "find_active_span",
    "LagEstimate",
]


# ---------------------------------------------------------------- 时间基


def resample_uniform(t, x, dt, t_start=None):
    """把 `(t, x)` 线性插值到步长 `dt` 的**统一网格**上。

    `t` 单位秒、必须单调递增；`x` 形状 `(N, C)`（也容忍 `(C, N)`）。
    返回 `(grid, out)`，`out` 形状 `(M, C)`。

    ★★ `t_start` 是**两条序列必须共享的网格原点**，默认取 `t[0]` 只是单序列时的方便值。

    为什么非共享不可：曾经两次调用都不传 `t_start`，于是各按**自己的** `t[0]` 建网格 ——
    两条序列就此被错开 `t_b[0] − t_a[0]`，而这个错位**一分不少地加进了 τ**。
    两个采样率不同（本机 IMU 164 Hz、动捕 30 Hz）时，裁剪到共同区间后首样本
    天然差最多半个采样周期（30 Hz 下 **16.7 ms**），于是同一个窗口换个 `--segments`
    就能让 τ 从 +53 ms 跳到 +20 ms，而 r 几乎不变 —— 因为"峰"整个被平移了。
    调用方（`estimate_lag`）**必须**把同一个 `t_start` 传给两条序列。
    """
    t = np.asarray(t, dtype=float)
    x = np.asarray(x, dtype=float)
    if x.ndim == 1:
        x = x[:, None]
    if x.shape[0] != t.size:
        x = x.T
    if x.shape[0] != t.size:
        raise ValueError(f"t 与 x 长度对不上：{t.size} vs {x.shape}")

    t0 = t[0] if t_start is None else float(t_start)
    grid = np.arange(t0, t[-1] + dt * 0.5, dt)
    out = np.empty((grid.size, x.shape[1]), dtype=float)
    for c in range(x.shape[1]):
        # 网格两端超出 `t` 覆盖范围的点，用端点值兜住（不外推成 0，
        # 那会在序列两端造出假的台阶，进而在相关里造出假的峰）
        out[:, c] = np.interp(grid, t, x[:, c])
    return grid, out


def lowpass_zero_phase(x, dt, fc, order=4):
    """Butterworth **零相位**低通（`filtfilt`）。

    ★ 必须是零相位。普通 `lfilter` 会带上一个与阶数/截止频率成正比的**群延迟**，
    那个延迟会**直接加进 τ 里** —— 也就是说滤波器自己会把结论改掉。
    `filtfilt` 前后各滤一遍、相位抵消，代价是**幅度响应被平方**（这里无所谓）。

    `fc <= 0` 或样本太少时原样返回。
    """
    if fc is None or fc <= 0:
        return x
    from scipy.signal import butter, filtfilt

    nyq = 0.5 / dt
    wn = fc / nyq
    if wn >= 1.0:
        return x
    # filtfilt 的边界填充需要长度 > 3 * (order+1)
    if x.shape[0] <= 3 * (order + 1):
        return x
    b, a = butter(order, wn, btype="low")
    return filtfilt(b, a, x, axis=0)


# ------------------------------------------------------------ 四元数工具
#
# 约定：q = [w, x, y, z]（Hamilton，与 PX4 `VehicleAttitude.q` / `VehicleOdometry.q` 的
# 日志字段顺序一致，`q[0]` 是实部），含义是 **body → world** 的旋转
# （`VehicleAttitude.msg`: "Quaternion rotation from the FRD body frame to the NED earth frame"）。


def quat_mul(q1, q2):
    """Hamilton 乘积 `q1 ⊗ q2`，输入形状 `(..., 4)`。"""
    q1 = np.asarray(q1, dtype=float)
    q2 = np.asarray(q2, dtype=float)
    w1, x1, y1, z1 = np.moveaxis(q1, -1, 0)
    w2, x2, y2, z2 = np.moveaxis(q2, -1, 0)
    return np.stack(
        [
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        ],
        axis=-1,
    )


def quat_inv(q):
    """单位四元数的逆 = 共轭。"""
    q = np.asarray(q, dtype=float)
    out = q.copy()
    out[..., 1:] *= -1.0
    return out


def quat_to_rate(t, q, attrib="midpoint"):
    """由 **body → world** 四元数序列求**体轴角速度**（rad/s）。

    用相邻样本的**体轴增量**：`dq = q(t_i)^-1 ⊗ q(t_{i+1})`，`ω ≈ 2·vec(dq)/dt`。

    ★★ `attrib` 决定这个增量记在**哪一刻**，而它**直接改变 τ 的结论**：
    增量 Δq 覆盖的是区间 `[t_i, t_{i+1}]`，
      · `"midpoint"`（默认）记在中点 —— 二阶精度、**无系统性滞后**；
      · `"left"` 记在 `t_i` —— 给动捕序列**凭空加上半个采样周期**的延迟
        （30 Hz 下 = **+16.7 ms**），足以把 50 ms 说成 67 ms。
    ⟹ 除非你在刻意复现某个旧口径，**不要用 `"left"`**。

    返回 `(t_out, omega)`，形状 `(N-1, 3)`。
    """
    t = np.asarray(t, dtype=float)
    q = np.asarray(q, dtype=float)
    if attrib not in ("midpoint", "left"):
        raise ValueError(f"attrib 只能是 midpoint/left，收到 {attrib!r}")

    dq = quat_mul(quat_inv(q[:-1]), q[1:])  # 体轴增量：q_i^-1 ⊗ q_{i+1}
    # 取短弧：四元数双覆盖（q 与 -q 同一姿态），w<0 时翻转符号，免得差分出现假的大跳
    dq = np.where(dq[:, :1] < 0.0, -dq, dq)
    dt = (t[1:] - t[:-1])[:, None]
    omega = 2.0 * dq[:, 1:] / dt
    t_out = t[:-1] if attrib == "left" else 0.5 * (t[:-1] + t[1:])
    return t_out, omega


def quat_body_down(q):
    """把**世界系的"下"**（NED 的 `ẑ`）表到体轴 —— 即 `R(q)ᵀ·ẑ`。

    这是"机体倾斜了多少、往哪边倾"的三维读数。它**不需要求导**，
    所以不受 `quat_to_rate` 那半个采样周期的影响；
    而且它对**偏航参考系任意**这件事免疫（绕世界 z 的旋转不改变 `ẑ`）。

    返回形状 `(..., 3)`，单位向量。
    """
    q = np.asarray(q, dtype=float)
    w, x, y, z = np.moveaxis(q, -1, 0)
    # R(q) 的第三**行** = R(q)ᵀ·ẑ
    return np.stack(
        [
            2.0 * (x * z - w * y),
            2.0 * (y * z + w * x),
            1.0 - 2.0 * (x * x + y * y),
        ],
        axis=-1,
    )


# ---------------------------------------------------------------- 互相关


def _excitation(x):
    """"这段信号动了多少" —— **逐通道沿时间求 std，再跨通道合并**（RMS 意义上的平均）。

    ★★ 两个很容易踩的口径，都实测验过是错的：
      · **整个数组的 RMS**：`quat_body_down` 按构造是**单位向量** ⟹ RMS **恒等于 1**，
        飞机停着还是翻跟头都是 1（实测 B 口径三段全是 1.00），毫无分辨力。
      · **整个数组的 std**：度量的是**通道之间**的差异而不是**随时间**的起伏 ——
        `body_down` 的 z 分量恒为 1、x/y 恒为 0 ⟹ std 恒 ≈ 0.47，
        静止与翻滚**分不开**（实测 B 口径三段全是 0.468/0.474/0.468）。
    只有"**沿时间**去均值"才是在量"动没动"：静止时每通道都是常量 ⟹ 0。
    """
    x = np.asarray(x, dtype=float)
    if x.ndim == 1:
        x = x[:, None]
    return float(np.sqrt((x.std(axis=0) ** 2).mean()))


def _ncc(a, b):
    """两段等长多通道序列的**归一化互相关**（各自去均值后）。

    分母用 Frobenius 范数 ⟹ 结果 ∈ [-1, 1]，与两条序列各自的**幅度/单位无关**
    （IMU 是 rad/s、动捕差分出来也是 rad/s，但即便单位不同这里也照样可比）。
    """
    a = a - a.mean(axis=0, keepdims=True)
    b = b - b.mean(axis=0, keepdims=True)
    num = float((a * b).sum())
    den = float(np.sqrt((a * a).sum() * (b * b).sum()))
    return num / den if den > 0.0 else 0.0


def xcorr_series(a, b, dt, max_lag_s):
    """`b` 相对 `a` 的滑动归一化互相关。

    返回 `(lags, r)`：`lags` 单位秒，`r[i]` 是 `b` 滞后 `lags[i]` 时的相关值。
    **`r` 有符号** —— 负峰意味着两条序列的**符号约定相反**（例如某一侧坐标系是左手系），
    那是要报告的事实，不是噪声。
    """
    a = np.asarray(a, dtype=float)
    b = np.asarray(b, dtype=float)
    if a.shape != b.shape:
        raise ValueError(f"两条序列形状必须相同：{a.shape} vs {b.shape}")
    n = a.shape[0]
    kmax = int(round(max_lag_s / dt))
    lags = np.arange(-kmax, kmax + 1)
    r = np.empty(lags.size, dtype=float)
    for i, k in enumerate(lags):
        if k >= 0:
            aa, bb = a[: n - k], b[k:]
        else:
            aa, bb = a[-k:], b[: n + k]
        r[i] = _ncc(aa, bb)
    return lags * dt, r


def peak_lag(lags, r):
    """在 `r` 上找**绝对值最大**的峰，用抛物线插值取亚采样位置。

    返回 `(lag_s, r_peak)`：
      · `lag_s` 带亚采样修正（相邻三点拟合抛物线顶点）；
      · `r_peak` 是**有符号**的峰高度 —— 负峰意味着两条序列符号约定相反。

    ★★ 这里**只**给"峰在哪"和"峰多高"，**不给峰的宽度**。
    两个被否掉的口径，别再加回来：
      · `|r_peak| / std(r)`（"锐度"）—— 依赖搜索窗。同一份数据、同一条曲线，
        窗从 ±50 ms 放宽到 ±300 ms 会让它从 7.7 掉到 2.3。拿它判可信度 = 判窗宽。
      · 半高全宽 —— 看着与窗无关，实测也**仍然**与窗有关
        （同一份数据 194 ms @ ±150 ms 窗 vs 226 ms @ ±300 ms 窗）。
        原因是这类信号（多正弦叠加、5 Hz 带限）的相关峰**本身就是平顶的**，
        "半高落在哪一点"没有定义，调阈值只是把一个不可复现的数调到看着能复现。
    ⟹ "这个数可不可信"要看 `r_peak`（有没有咬住）+ `estimate_lag_segmented`
      的**分段离散度**（换个时间窗还对不对得上）。这两个才是可复现的量。
    """
    r = np.asarray(r, dtype=float)
    lags = np.asarray(lags, dtype=float)
    if r.size == 0:
        return float("nan"), 0.0
    i = int(np.argmax(np.abs(r)))
    rp = float(r[i])
    if 0 < i < r.size - 1:
        y0, y1, y2 = float(r[i - 1]), rp, float(r[i + 1])
        denom = y0 - 2.0 * y1 + y2
        delta = 0.5 * (y0 - y2) / denom if denom != 0.0 else 0.0
        delta = float(np.clip(delta, -1.0, 1.0))
    else:
        delta = 0.0
    step = float(np.median(np.diff(lags))) if lags.size > 1 else 1.0
    return float(lags[i]) + delta * step, rp


def find_active_span(t, pos, move_eps=0.05, look_s=2.0, gap_s=3.0, min_span_s=5.0):
    """在 `(t, pos)` 里找出**真正在动**的那一段，返回 `(t_start, t_end)`。

    ★ 这一步不是可选项。一次飞行日志里飞机常常有**几分钟是停在地面上**的
    （待命、落地后没断电）。把静止段一起拿去做互相关，等于用一堆零去"稀释"信号：
    实测 z0 的 log 210 前 18 MB 里 366 s 中有 300 s 静止，
    不加这一步时三个相关全部掉到 |r| ≤ 0.03 —— 连"同一台 IMU 对自己"都相关不上。

    判据：`|pos[i] − pos[i 往前 look_s 秒]| > move_eps` 记为"在动"，
    再把间隔 ≤ `gap_s` 的活动段并起来，取**最长**的一段。
    都没动则 `ValueError`（不能给 τ）。

    返回的首尾各**不**留余量 —— 交给 `estimate_lag(warmup_s=...)` 去丢瞬态。
    """
    t = np.asarray(t, dtype=float)
    pos = np.asarray(pos, dtype=float)
    if pos.ndim == 1:
        pos = pos[:, None]
    if t.size < 3:
        raise ValueError(f"只有 {t.size} 个点，圈不出活动段")

    # 每个点与"往前 look_s 秒"那个点之间的位移
    j = np.searchsorted(t, t - look_s, side="left")
    j = np.clip(j, 0, t.size - 1)
    moving = np.linalg.norm(pos - pos[j], axis=1) > move_eps

    if not moving.any():
        raise ValueError(
            f"整段里没有任何时刻的位移超过 {move_eps} m —— 飞机没动过。"
            "这不是 τ=0，是**量不出来**，别给数。"
        )

    # 合并间隔 ≤ gap_s 的活动点，取最长的一段
    idx = np.flatnonzero(moving)
    splits = np.flatnonzero(np.diff(t[idx]) > gap_s)
    runs = np.split(idx, splits + 1)
    best = max(runs, key=lambda r: t[r[-1]] - t[r[0]])
    t0, t1 = float(t[best[0]]), float(t[best[-1]])
    if t1 - t0 < min_span_s:
        raise ValueError(
            f"最长的活动段只有 {t1 - t0:.1f} s（< {min_span_s} s）—— 太短，估不出 τ"
        )
    return t0, t1


class LagEstimate:
    """一次 τ 估计的全部可追溯信息。

    ★ **不含**任何"峰宽"字段，理由见 `peak_lag`（那个量不可复现）。
    τ 的可信度靠 `r_peak` + 分段离散度，不靠宽度。
    """

    def __init__(self, lag_s, r_peak, n_samples, dt, warmup_s, exc_a, exc_b):
        self.lag_s = lag_s
        self.r_peak = r_peak
        self.n_samples = n_samples
        self.dt = dt
        self.warmup_s = warmup_s
        # 激励量：重采样+滤波后，各通道**去均值**的起伏（std），不是 RMS。
        self.exc_a = exc_a
        self.exc_b = exc_b

    def __repr__(self):
        return (
            f"LagEstimate(τ={self.lag_s * 1e3:+.1f} ms, r={self.r_peak:+.3f}, "
            f"N={self.n_samples}, dt={self.dt * 1e3:.2f} ms, "
            f"激励σ={self.exc_a:.4g}/{self.exc_b:.4g})"
        )


def estimate_lag(t_a, a, t_b, b, dt=0.002, max_lag_s=0.3, fc=5.0, warmup_s=0.0):
    """端到端：重采样 → 零相位低通 → 归一化互相关 → 找峰。

    `a` = 参考（飞控 IMU），`b` = 待测滞后的那条（动捕）。
    `max_lag_s` 是搜索半径（**只搜这一段**，超出范围不给答案，免得把"别处的巧合"当峰）。
    """
    a = np.asarray(a, dtype=float)
    b = np.asarray(b, dtype=float)
    t_a = np.asarray(t_a, dtype=float)
    t_b = np.asarray(t_b, dtype=float)

    if warmup_s > 0.0:
        t0 = max(t_a[0], t_b[0]) + warmup_s
        t_a, a = t_a[t_a >= t0], a[t_a >= t0]
        t_b, b = t_b[t_b >= t0], b[t_b >= t0]

    t0 = max(t_a[0], t_b[0])
    t1 = min(t_a[-1], t_b[-1])
    t_a, a = t_a[(t_a >= t0) & (t_a <= t1)], a[(t_a >= t0) & (t_a <= t1)]
    t_b, b = t_b[(t_b >= t0) & (t_b <= t1)], b[(t_b >= t0) & (t_b <= t1)]

    span = t1 - t0
    # 太短就别给数：互相关的分辨率本来就有限，重叠区不够长时"峰"是拼出来的
    if span < max(2.0, 5.0 * max_lag_s):
        raise ValueError(
            f"两条序列的重叠只有 {span:.2f} s，不足以在 ±{max_lag_s*1e3:.0f} ms 内找峰"
            f"（至少要 {max(2.0, 5.0*max_lag_s):.2f} s）"
        )

    # ★ 两条序列用**同一个 `t_start=t0`** 建网格 —— 见 `resample_uniform` 的说明。
    # 各用各的 `t[0]` 会把"两条序列起点差多少"混进 τ 里。
    grid, A = resample_uniform(t_a, a, dt, t_start=t0)
    _, B = resample_uniform(t_b, b, dt, t_start=t0)
    n = min(A.shape[0], B.shape[0])
    A, B = A[:n], B[:n]
    grid = grid[:n]

    A = lowpass_zero_phase(A, dt, fc)
    B = lowpass_zero_phase(B, dt, fc)

    # ★ 零方差要**在搜峰之前**就拒掉。否则 `argmax(|r|)` 只能挑噪声，
    # 报出来的 τ 会稳定地贴在下标 0 或最后一格（= ±max_lag，窗边缘），
    # 看着像个正经数字 —— 实测在"飞机没动"的 SITL 日志上正是这么表现。
    if float(A.std()) == 0.0 or float(B.std()) == 0.0:
        raise ValueError(
            "重叠区内有一条序列是常量（std=0）—— 没有可相关的东西。"
            "多半是这段飞行没有机动激励（或该 topic 全零），不是 τ=0。"
        )

    lags, r = xcorr_series(A, B, dt, max_lag_s)
    lag, rp = peak_lag(lags, r)
    return LagEstimate(
        lag_s=lag,
        r_peak=rp,
        n_samples=n,
        dt=dt,
        warmup_s=warmup_s,
        exc_a=_excitation(A),
        exc_b=_excitation(B),
    )


def estimate_lag_segmented(t_a, a, t_b, b, n_segments=6, **kwargs):
    """把重叠区切成 `n_segments` 段，各估一次 τ。

    ★ 这才是 τ 的**可复现性**证据：`estimate_lag` 给出的是整段的一个数，
    而"这个数稳不稳"只能靠**分段之间彼此对不对得上**来看。
    返回 `(est_list, median_s, span_s)`，`span_s = max - min`。

    ★★ 但`span_s` 会**被没有激励的段带偏** —— 一段里飞机没转，相关就没有峰，
    τ 会贴到搜索窗边缘（±`max_lag_s`），于是极差动辄几百 ms，看着像"方法不稳"。
    实测 z0 的 log 210 正是这样：3 段里第 1 段（起飞前那 6 s）陀螺 RMS 只有 0.03 rad/s，
    τ 落在 −300 ms，而含机动的两段给 +57 / +49 ms。
    ⟹ **读 `span_s` 时必须同时看每段的 `exc_b`（激励σ）**（`LagEstimate` 里带），
    或者只看激励够大的那几段。`span_s` 大**不等于**方法不稳。

    段太短（< 2 s）或估不出来的段会被跳过；一段都没成则 `est_list` 为空。
    """
    t_a = np.asarray(t_a, dtype=float)
    t_b = np.asarray(t_b, dtype=float)
    t0 = max(t_a[0], t_b[0])
    t1 = min(t_a[-1], t_b[-1])

    # ★ `warmup_s` 在**整段**上扣一次，不能传给每一段各自扣。
    # 曾经把它透传下去：21.9 s 的窗口切 6 段、每段 3.65 s，每段再扣 3 s 就只剩 0.65 s，
    # `estimate_lag` 一律以"重叠太短"拒掉 ⟹ 分段全是空的，看着像"数据不支持分段"，
    # 其实是这里扣重了。起飞瞬态只发生一次。
    kwargs = dict(kwargs)
    warmup_s = float(kwargs.pop("warmup_s", 0.0) or 0.0)
    if warmup_s > 0.0:
        t0 += warmup_s
    if t1 - t0 < 2.0:
        return [], float("nan"), float("nan")

    edges = np.linspace(t0, t1, n_segments + 1)

    ests = []
    for i in range(n_segments):
        lo, hi = edges[i], edges[i + 1]
        if hi - lo < 2.0:
            continue
        sa = (t_a >= lo) & (t_a < hi)
        sb = (t_b >= lo) & (t_b < hi)
        if sa.sum() < 8 or sb.sum() < 8:
            continue
        try:
            ests.append(estimate_lag(t_a[sa], a[sa], t_b[sb], b[sb], **kwargs))
        except ValueError:
            continue

    if not ests:
        return [], float("nan"), float("nan")
    lags = np.array([e.lag_s for e in ests])
    return ests, float(np.median(lags)), float(lags.max() - lags.min())
