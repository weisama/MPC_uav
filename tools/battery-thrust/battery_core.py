#!/usr/bin/env python3
"""电池侧读数核心 —— **零依赖**（没有 pyulog、没有 ROS、不碰文件）。

服务 [phase3/08](../../.scratch/phase3/issues/08-battery-sustainable-thrust-from-archived-ulogs.md)
的三个问题里的前两个：

| 票面问题 | 这里的落点 |
|---|---|
| ① 可持续电流上限 | `sustained_max()` —— **1 s 窗中位**，不是单点尖峰 |
| ② 换算成推力/功率 | `hover_anchor()` + `extrapolate_power()`（`P ∝ T^1.5`） |
| ③ 取不到的量 | `channel_verdicts()` —— 逐字段判"这个量这次能不能当判据" |

★ **本模块里唯一有物理重量的一句话**：判据要选**能区分对错的那一列**。
`current_a` 的单点在样本级与 `voltage_v` 对不上（见 `esr_and_signature()` 的做法与返回值），
所以"可持续电流"只能落在**窗口均值**上 —— 单点读数会被抖动用掉。
这一条是把 `hover_thrust_core` 里"读数不随判据移动"那条纪律换个方向用：
那边防的是**判据挑数据**，这边防的是**噪声冒充读数**。

★ 另一个贯穿全模块的约定：**`f_max` 与 `t_max` 是两个量，不许混**（2026-09-29 解耦）——
`f_max` = OCP 的 `F` 上限（`ocp.py` 的 `ubu[0]`，按**电池**），
`t_max` = 归一化常数（按**电机/桨**，实测满量程）。
本模块**只**用 `f_max` 做"电池顶不顶得住"的换算；`t_max` 只被打印出来提醒别认错。
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

# ---------------------------------------------------------------- 常数

G_DEFAULT = 9.80665
#: 起飞质量。与 `tools/hover-thrust` / `src/nmpc` 的默认值同源（`docs/reference/hardware.md` §9）。
MASS_DEFAULT = 0.75

#: OCP 的 `F` 上限（`src/nmpc/nmpc/ocp.py` 的 `ubu[0]`）—— 按**电池**的那个数。
#: 真源是 `src/nmpc/nmpc/params.py` 的 `F_MAX_BATTERY`；此处只是默认值，可用 `--f-max` 覆盖。
F_MAX_DEFAULT = 11.8

#: 归一化常数（按**电机/桨**）—— 与 `f_max` **不是一个量**，本模块从不拿它做换算。
#: 真源是 `params.py` 的 `t_max` property（`= m·g / hover_thrust`）。
#: ⚠️ `hover_thrust` 变了它就跟着变（0.1589 ⟹ 46.3027 N）；这里只为打印提醒。
T_MAX_DEFAULT = 46.3027

#: `P ∝ T^n` 的指数（螺旋桨经验关系，与 `docs/reference/hardware.md` §6.2 同一个）。
#: ⚠️ 它是**经验近似**：小推力区与接近饱和区都会偏 —— 见该节 §6.4 的精度限制。
PROP_EXPONENT_DEFAULT = 1.5

#: PX4 `BAT_AVRG_CURRENT` 的默认值 —— 用来识别 `current_average_a` **是滤波器初值**而不是读数。
BAT_AVRG_CURRENT_DEFAULT = 15.0

#: 电芯的**持续放电额定** [A] —— VTC6 的 Mooch 实测口径（`docs/reference/hardware.md` §6.1）。
#: ⚠️ 它**不是** `BAT_AVRG_CURRENT_DEFAULT`：前者是**电池规格**，后者是 PX4 一个**滤波器初值**。
#: 两者都是 15.0 纯属巧合同值 —— 共用一个常数的话，PX4 哪天改了那个默认值，
#: 本工具拿来当"额定"的数会**跟着一起悄悄变**。
CELL_RATING_A_DEFAULT = 15.0

#: 整包内阻的**物理下界** [Ω] —— VTC6 单节 ≈13 mΩ × 6S1P（`hardware.md` §6.1）。
#: ★ 它是**外部输入**，正是这一点让 `esr_and_signature` 的自洽性检验免于自我循环：
#: 老化与线阻只会让真实内阻**更大**，所以 `noise_I · R_min` 是电压印记的**下界**。
R_MIN_PHYSICAL_OHM = 0.078

EXIT_OK = 0
EXIT_UNAVAILABLE = 2
EXIT_NO_DATA = 4


# ---------------------------------------------------------------- 取值


def step_lookup(t_query, t_src, x_src):
    """按**阶梯**语义取值：每个查询点取"此前最近的一个源样本"。

    ⚠️ `landed` / `flag_armed` 是布尔量，**不能线性插值**：插出来的 0.5 既不是"已落地"
    也不是"未落地"。这条与 `tools/hover-thrust` 的 `_step_lookup` 是同一个教训
    （那边是因为"两条时间基样本数碰巧相同 ⟹ 静默配对"才写成这样的）。
    查询点早于第一个源样本时返回 `x_src[0]`。
    """
    t_src = np.asarray(t_src, dtype=float)
    idx = np.searchsorted(t_src, np.asarray(t_query, dtype=float), side="right") - 1
    np.clip(idx, 0, t_src.size - 1, out=idx)
    return np.asarray(x_src, dtype=float)[idx]


def in_flight_mask(t_query, t_cm, armed, t_land, landed):
    """`flag_armed ∧ ¬landed` —— **读日志里的第一手状态，不猜**。

    `t_cm`/`armed` 在 `vehicle_control_mode` 的时间基上，`t_land`/`landed` 在
    `vehicle_land_detected` 的。★ **先在 `control_mode` 那条时间基上把两个状态合起来**，
    最后才落到查询点：两条的样本数不同（实测 710 vs 363），各自落到查询点再相与会抛广播错；
    样本数**碰巧相同**时它就会静默地把两个不同时刻的状态配到一起 —— 两种都要挡。
    """
    t_cm = np.asarray(t_cm, dtype=float)
    landed_at_cm = step_lookup(t_cm, t_land, landed)
    combined = (np.asarray(armed, dtype=float) > 0.5) & (landed_at_cm < 0.5)
    return step_lookup(t_query, t_cm, combined.astype(float)) > 0.5


def interp_to(t_src, x_src, t_query):
    """线性插值到 `t_query`；**落在源区间之外给 NaN**（不外推）。

    外推出来的数看着像读数，其实没有任何证据支撑。NaN 会在下游被掩码自然排除。
    """
    t_src = np.asarray(t_src, dtype=float)
    x_src = np.asarray(x_src, dtype=float)
    t_query = np.asarray(t_query, dtype=float)
    out = np.interp(t_query, t_src, x_src, left=np.nan, right=np.nan)
    # `np.interp` 的 left/right 用 NaN 时**插值结果仍会被端值替代**（旧行为）⟹ 手工再挡一次
    out = np.where((t_query < t_src[0]) | (t_query > t_src[-1]), np.nan, out)
    return out


# ---------------------------------------------------------------- 统计


def rolling_median(t, x, dt=1.0, min_n=3):
    """**按时间**取滑窗中位（不是按样本数）—— 采样率不齐时按样本数会偏心。

    返回 `(t_centers, medians, counts)`；窗内样本数 < `min_n` 的点**不进结果**。
    """
    t = np.asarray(t, dtype=float)
    x = np.asarray(x, dtype=float)
    half = 0.5 * float(dt)
    tc, med, n = [], [], []
    for i in range(t.size):
        m = np.abs(t - t[i]) <= half
        k = int(m.sum())
        if k < min_n:
            continue
        tc.append(t[i])
        med.append(float(np.median(x[m])))
        n.append(k)
    return np.asarray(tc), np.asarray(med), np.asarray(n, dtype=int)


def sustained_max(t, x, dt=1.0, min_n=3):
    """**可持续**的那一列：滑窗中位的最大值，以及它出现在哪。

    ★ 票面点名要的是"可持续"，不是 `x.max()`：单点尖峰（实测见过 11.38 A 的孤点）
    既可能是 ADC 毛刺、也可能是真的瞬态，两者都**不代表电池"可持续"给得出**。

    ⚠️ 但这仍旧是**观测到的需求**，不是电池的**能力** —— 而且"全段最大"天生往负载
    最高的那一瞬跑，归档里有几份就正好落在解锁瞬间，即票面点名排除的那一列。
    所以要配 `sustained_lands_at_flight_start` 一起读。

    返回 `{"value", "at_s", "raw_max", "raw_max_at_s", "n_windows"}`。
    没有足够长的窗口时 `value = None`（**不是** 0 —— "没量到"与"量到 0"不是一回事）。
    """
    tc, med, _ = rolling_median(t, x, dt=dt, min_n=min_n)
    x = np.asarray(x, dtype=float)
    out = {
        "value": None,
        "at_s": None,
        "raw_max": float(np.nanmax(x)) if x.size else None,
        "raw_max_at_s": float(np.asarray(t, dtype=float)[int(np.nanargmax(x))]) if x.size else None,
        "n_windows": int(tc.size),
        "window_s": float(dt),
        "min_n": int(min_n),
    }
    if tc.size:
        k = int(np.argmax(med))
        out["value"] = float(med[k])
        out["at_s"] = float(tc[k])
    return out


def sustained_lands_at_flight_start(sustained, t0, window_s):
    """`sustained` 取到最大值的那一段，是不是恰好落在在飞窗口的**头一个窗口**里。

    ★ 为什么值得专门判一下：票面问 ① 时**点名排除**了「起飞那一瞬的最大电流」。
    而"全段最大"这个统计量天生往负载最高的那一瞬跑 —— 归档里 8 份有 4 份正好落在
    解锁瞬间。不点名的话，读者会把那一行直接当成问 ① 的答案（一次复核就是这么读的）。

    返回 `True` / `False`；没有在飞窗口或算不出 `sustained` 时返回 `None`（**不是** `False`
    —— "没量到"与"量到不在起点"不是一回事）。
    """
    if sustained is None or t0 is None:
        return None
    at = sustained.get("at_s") if isinstance(sustained, dict) else None
    if at is None:
        return None
    return bool(float(at) - float(t0) <= float(window_s) + 1e-9)


def robust_noise(x):
    """样本级抖动的稳健估计：`1.4826 · MAD(一阶差分) / √2`。

    用差分是为了扣掉缓慢的趋势（电池掉压是个单调漂移，不是噪声）；
    除以 √2 是把"差分的方差 = 两倍原方差"折回去。
    用 MAD 而不是 std：这里**就是要它不为毛刺所动**（毛刺本身是另一条判据的事）。
    """
    x = np.asarray(x, dtype=float)
    if x.size < 3:
        return float("nan")
    d = np.diff(x)
    mad = float(np.median(np.abs(d - np.median(d))))
    return 1.4826 * mad / np.sqrt(2.0)


def longest_run(mask):
    """最长的一段连续 True，返回 `(i0, i1)`（左闭右开）；全 False 时 `(None, None)`。

    ⚠️ 返回 `(None, None)` 而不是 `(0, 0)`：后者会让下游拿着一个空切片当读数用。
    """
    mask = np.asarray(mask, dtype=bool)
    best = (None, None)
    best_len = 0
    i = 0
    n = mask.size
    while i < n:
        if mask[i]:
            j = i
            while j < n and mask[j]:
                j += 1
            if j - i > best_len:
                best_len = j - i
                best = (i, j)
            i = j
        else:
            i += 1
    return best


def steady_mask(speed, tilt_rad, v_eps=0.30, tilt_eps_rad=np.radians(10.0)):
    """逐样本判"这一段像不像稳态悬停"：`|v| ≤ v_eps ∧ 倾角 ≤ tilt_eps`。

    ⚠️ 这两个数**不是随手定的**，是从 `tools/hover-thrust` 借来的同一对（那边有量纲依据）：
    倾角给 `1 − cos 10° = 1.5%`、速度门给"整段持续偏掉的上界" `2·v_eps/(g·T)`。
    本模块只用它来**挑锚点窗口**（悬停时推力 ≡ `m·g`，这是锚点的物理依据），
    不用它来改任何推力读数。
    """
    speed = np.asarray(speed, dtype=float)
    tilt_rad = np.asarray(tilt_rad, dtype=float)
    return (np.abs(speed) <= v_eps) & (np.abs(tilt_rad) <= tilt_eps_rad)


def tilt_rad_from_quat(q):
    """姿态四元数 → 机体 z 轴相对竖直的夹角（rad）。与 `hover_thrust_core` 同式。"""
    q = np.asarray(q, dtype=float)
    # 机体系 z 轴在世界上投影的 -z 分量 = 机体 z 与竖直的夹角余弦
    w, x, y, z = q[..., 0], q[..., 1], q[..., 2], q[..., 3]
    cos_tilt = 1.0 - 2.0 * (x * x + y * y)
    return np.arccos(np.clip(cos_tilt, -1.0, 1.0))


# ---------------------------------------------------------------- 判据：这一列能不能用


def esr_and_signature(t, current, voltage, dt=1.0, min_n=3, r_min_ohm=R_MIN_PHYSICAL_OHM):
    """**"电流这一列到底能不能当判据"的自检** —— 靠电压给它做背书。

    做法：把两个量都做 1 s 中位分箱（压掉单点毛刺），再回归

        V = a + b·I + c·t

    带 `c·t` 那一项是必须的：电池在放电，电压本来就在往下漂，不扣掉它就会把
    "掉电"记到电流头上（`b` 被放大）。`R = −b`（记为 `r_int_ohm`）。

    ★ 自洽性检验：如果电流的**样本级抖动**是真的，它必须在电压上留下印记。但要算那个印记，
    **不能用拟合出来的 `R`** —— 那正是要检验的东西，会自我循环：`I` 与 `V` 完全无关时，
    带时间项的回归仍会给出一个非零的 `b`，于是"预期印记"非零、"耦合成立"，
    而报告看上去一切正常（**第三类静默失败**）。
    改用**外部**物理下界 `r_min_ohm`（`hardware.md` §6.1 的整包 ≈78 mΩ；
    老化与线阻只会让它更大，不会更小）⟹ 预期印记 = `noise_I · r_min_ohm` **是下界**。

    判据：真实电流的印记**至少**该是电压自身抖动的 3 倍。电压比物理允许的还安静
    ⟹ 那些单点抖**不是**在测电池电流。返回 `verdict`：

    | 值 | 含义 |
    |---|---|
    | `"not-visible-in-voltage"` | 电压比物理下界还安静 ⟹ 那些抖**不是**电池电流 |
    | `"visible-in-voltage"` | 电压吵到与"抖动是真的"相容 —— ⚠️ **这一档不是"单点可用"的同义词** |
    | `"no-data"` | 分箱太少，检验做不了 |

    ★★ **别让这条判据背它背不动的重量。** 实测五份日志里这个比值落在
    **2.6–3.4 倍**、正好骑在 3 倍阈值上 ⟹ 两种解释都活着：

    (a) 那些单点抖是采样噪声（不是电池电流）；或
    (b) 抖是真的，而**电压通道被硬件 RC 滤波**（电源模块的分压线上加电容很常见），
        把压降抹平了。

    日志里**分不开 (a) 与 (b)**。所以本工具**不用这条判据做决定**，而是写死一条
    `policy`：**一律用窗口均值，单点永不作为判据**。理由与这条判据无关 ——
    在两个世界里单点都是**错的统计量**：(a) 下它是纯噪声；(b) 下它是个真实的**瞬时**值，
    而我们要的是"持续"多少。均值在两个世界里都无偏。

    另见 `fit_is_identifying`：分箱相关性不显著时，拟合出的 `R` 只是噪声的产物，
    `r_int_ohm` 一栏**不该被当测量值引用**。
    """
    t = np.asarray(t, dtype=float)
    current = np.asarray(current, dtype=float)
    voltage = np.asarray(voltage, dtype=float)

    ni = robust_noise(current)
    nv = robust_noise(voltage)

    out = {
        "n_samples": int(t.size),
        "noise_current_1sigma_a": None if not np.isfinite(ni) else float(ni),
        "noise_voltage_1sigma_v": None if not np.isfinite(nv) else float(nv),
        "r_int_ohm": None,
        "r_min_ohm_ref": float(r_min_ohm),
        "residual_sd_v": None,
        "corr_binned": None,
        "corr_threshold_3sigma": None,
        "fit_is_identifying": None,
        "n_bins": 0,
        "expected_signature_v": None,
        "signature_ratio": None,
        "verdict": "no-data",
        #: ★ 这条**不受 `verdict` 影响**（见上）：即使判据说"电压里看得见抖"，单点也不当判据
        "policy": "一律用窗口均值；单点永不作为判据（两种解释下单点都是错的统计量）",
    }
    if t.size < 3:
        return out

    tc, im, _ = rolling_median(t, current, dt=dt, min_n=min_n)
    tv, vm, _ = rolling_median(t, voltage, dt=dt, min_n=min_n)
    # 两条的时间基是同一条（都在 battery_status 上），但仍按查询点对齐，免得将来自作主张
    if tc.size >= 8:
        vm_on_i = interp_to(tv, vm, tc)
        ok = np.isfinite(vm_on_i)
        if ok.sum() >= 8:
            im_o, vm_o, tt = im[ok], vm_on_i[ok], tc[ok]
            a = np.column_stack([np.ones_like(im_o), im_o, tt - tt[0]])
            coef, *_ = np.linalg.lstsq(a, vm_o, rcond=None)
            res = vm_o - a @ coef
            out["r_int_ohm"] = float(-coef[1])
            out["residual_sd_v"] = float(res.std())
            corr = float(np.corrcoef(im_o, vm_o)[0, 1])
            out["corr_binned"] = corr
            out["n_bins"] = int(im_o.size)
            # 零相关的 3σ 门槛 ≈ 3/√n。过不了它就说明 `b` 是噪声，不是内阻。
            thr = 3.0 / np.sqrt(im_o.size)
            out["corr_threshold_3sigma"] = float(thr)
            out["fit_is_identifying"] = bool(np.isfinite(corr) and abs(corr) > thr)

    # ★ 检验**没跑成**时必须说 `no-data`，不能落到默认分支去。
    # 这里踩过一次：没有分箱（窗口太短）时 `r_int_ohm` 是 `None`，但两边的噪声都是
    # 有限数（常数列的 MAD 是 0，0 是有限数！）⟹ 旧写法会给出 `"coupled"`，
    # 也就是**一次都没检验却宣称"电流这列可用"**。这与"`None` = 没量到"是同一条纪律。
    if out["n_bins"] >= 8 and np.isfinite(ni) and np.isfinite(nv):
        expect = ni * abs(float(r_min_ohm))
        out["expected_signature_v"] = float(expect)
        # ★ 判据：电流抖动若为真，其电压印记**至少**是 `noise_I · R_min`
        # （用下界，所以这条对"抖是真的"最宽容）。电压比它还安静 ⟹ 那些抖不是电池电流。
        ratio = expect / max(nv, 1e-9)
        out["signature_ratio"] = float(ratio)
        out["verdict"] = "not-visible-in-voltage" if ratio > 3.0 else "visible-in-voltage"
    return out


def channel_verdicts(fields):
    """逐字段判"这个量这次能不能当判据"，**把取不到的如实点名**（票面问题 ③）。

    输入是一个 dict：每个字段名 → `{"present": bool, "values": np.ndarray|None, ...}`。
    返回 `{字段名: {"verdict", "why"}}`，`verdict ∈ {"usable", "window-mean-only", "absent", "not-a-measurement"}`。

    ★ 三条判据都是**结构性**的，不是"看着不对"：

    | 字段 | 判据 | 后果 |
    |---|---|---|
    | `voltage_cell_v[*]` | 模拟电源模块**从不填**这些字段（PX4 只有 smart battery 驱动会填）| 单节电压**取不到**；`cell_count = 6` 是**参数值**不是读数 |
    | `current_average_a` | 初值 = `BAT_AVRG_CURRENT` 的慢滤波，实测整段压在 5–14 A | **不是测量值**，不能当"电池在供 15 A"的证据 |
    | `battery_status.scale` | 它是 `v_charged / v(volt-based SOC)`，**推力补偿系数**，不是电流标定 | 拿它当"电流有没有标定"会判错 —— 电流的标定是 `BAT1_A_PER_V` |
    """
    out = {}
    cells = fields.get("voltage_cell_v")
    if cells is not None:
        present = bool(cells.get("present"))
        nz = float(np.nanmax(np.abs(np.asarray(cells.get("values"), dtype=float)))) if present else 0.0
        if not present:
            out["voltage_cell_v"] = {
                "verdict": "absent",
                "why": "日志里没有这些字段",
            }
        elif nz == 0.0:
            out["voltage_cell_v"] = {
                "verdict": "absent",
                "why": "字段在，但**整份日志全为 0** —— 模拟电源模块不填单节电压"
                       "（PX4 只有 batt_smbus / smart_battery / tattu_can / cyphal 这些会填）。"
                       "`cell_count = BAT1_N_CELLS` 是**参数值**，别读成「这块电池真有 6 节在读」",
            }
        else:
            out["voltage_cell_v"] = {"verdict": "usable", "why": "有非零读数"}

    avg = fields.get("current_average_a")
    if avg is not None and avg.get("present"):
        a = np.asarray(avg.get("values"), dtype=float)
        ref = float(avg.get("bat_avrg_current", BAT_AVRG_CURRENT_DEFAULT))
        lo, hi = float(np.nanmin(a)), float(np.nanmax(a))
        # 判据：最小值贴近 BAT_AVRG_CURRENT ⟹ 它从头到尾都没离开初值
        if np.isfinite(lo) and lo >= 0.5 * ref:
            out["current_average_a"] = {
                "verdict": "not-a-measurement",
                "why": f"整份日志落在 [{lo:.2f}, {hi:.2f}] A，而 `BAT_AVRG_CURRENT = {ref:.1f}` "
                       f"是它的滤波器**初值**（`Battery::computeRemainingTime` 里 reset 成它）⟹ "
                       f"这是「没收敛的滤波器」，不是电池读数",
            }
        else:
            out["current_average_a"] = {
                "verdict": "not-a-measurement",
                "why": "它是电流的慢滤波（初值 `BAT_AVRG_CURRENT`），滞后与初值都不受控 ⟹ "
                       "任何一行都用它当「电池在供多少」都是把滤波器当仪器",
            }

    sc = fields.get("scale")
    if sc is not None and sc.get("present"):
        s = np.asarray(sc.get("values"), dtype=float)
        out["scale"] = {
            "verdict": "not-a-measurement",
            "why": f"`scale = v_charged / bat_v`（`Battery::computeScale`），是**推力补偿系数**，"
                   f"不是电流标定（实测 {np.nanmin(s):.3f}–{np.nanmax(s):.3f}，被夹在 [1.0, 1.3]）。"
                   f"电流的标定在 `BAT1_A_PER_V`；`scale` 为 0 是**不可能**的，"
                   f"拿它判「电流有没有标定」会判错",
        }
    return out


# ---------------------------------------------------------------- 换算


def extrapolate_power(p_ref, t_ref, t_query, n=PROP_EXPONENT_DEFAULT):
    """`P = P_ref · (T / T_ref)^n` —— 从已知工况点外推到目标推力。

    ⚠️ 这是**经验关系的外推**，不是测量：`n = 1.5` 在小推力区与接近饱和区都会偏
    （`docs/reference/hardware.md` §6.4）。本模块把它标成 `estimated`，从不当读数报。
    """
    if p_ref is None or t_ref in (None, 0) or t_query is None:
        return None
    return float(p_ref) * (float(t_query) / float(t_ref)) ** float(n)


def extrapolate_current(p_ref, t_ref, t_query, voltage_v, n=PROP_EXPONENT_DEFAULT):
    """先外推功率、再按**工作电压**折成电流：`I = P / V`。

    `voltage_v` 用**目标工况下的工作电压**（可由 `voltage_after_sag()` 迭代得到）——
    用锚点电压会低估电流（重载时电压更低、同样的功率要多抽电流）。
    """
    p = extrapolate_power(p_ref, t_ref, t_query, n=n)
    if p is None or voltage_v in (None, 0):
        return None
    return float(p) / float(voltage_v)


def voltage_after_sag(v_ref, i_ref, i_new, r_int):
    """`V_new = V_ref − R·(I_new − I_ref)` —— 只做一次线性外推，够用且不假装更准。

    `r_int` 为 None 时返回 `v_ref`（即不修正），并把这件事留给调用方在报告里说明。
    """
    if r_int is None or i_ref is None or i_new is None:
        return float(v_ref)
    return float(v_ref) - float(r_int) * (float(i_new) - float(i_ref))


@dataclass
class Anchor:
    """一个"推力已知"的工作点。稳态悬停时 `thrust_n ≡ m·g`（受力平衡）—— 这是锚点的全部依据。"""

    t0: float
    t1: float
    n: int
    thrust_n: float
    voltage_v: float
    current_a: float
    power_w: float
    notes: list = None

    def as_dict(self):
        return {
            "window_s": [self.t0, self.t1],
            "n": self.n,
            "thrust_n": self.thrust_n,
            "voltage_v": self.voltage_v,
            "current_a": self.current_a,
            "power_w": self.power_w,
            "notes": list(self.notes or []),
        }


def hover_anchor(t, voltage, current, speed, tilt_rad, mass=MASS_DEFAULT,
                 v_eps=0.30, tilt_eps_rad=np.radians(10.0), min_duration=2.0,
                 eligible=None):
    """从"稳态悬停"的**最长一段连续窗口**里取电池工作点。

    ★ 为什么悬停能当锚点：稳态悬停时**推力必须等于重力**（受力平衡），
    所以 `T = m·g` 是**物理**给的，不是从模型里抄的 —— 与控制器的 `hover_thrust` 无关。
    这与 `tools/hover-thrust` 的论证是同一条，只是用法相反：那边拿它反推归一化，
    这边拿它当"电流 ↔ 推力"的刻度。

    窗口外的样本一律不参与（`NaN` 掩码），**不外推**。

    `eligible`：可选的**前置掩码**（比如"在飞段"）。
    ★ 这个参数不是可有可无的装饰：地面上飞机静止不动、倾角为零，**它比真正的悬停更"稳态"**，
    于是"最长稳态段"会被地面段整段赢走。第一版没这个参数，结果五份日志里
    **一份锚点都没取到** —— 不是没有悬停，是搜索范围错了。
    """
    t = np.asarray(t, dtype=float)
    speed = np.asarray(speed, dtype=float)
    tilt_rad = np.asarray(tilt_rad, dtype=float)
    voltage = np.asarray(voltage, dtype=float)
    current = np.asarray(current, dtype=float)

    ok = np.isfinite(speed) & np.isfinite(tilt_rad) & np.isfinite(voltage) & np.isfinite(current)
    if eligible is not None:
        ok &= np.asarray(eligible, dtype=bool)
    m = steady_mask(np.where(ok, speed, np.inf), np.where(ok, tilt_rad, np.inf),
                    v_eps=v_eps, tilt_eps_rad=tilt_eps_rad)
    i0, i1 = longest_run(m)
    if i0 is None:
        return None, {
            "reason": "没有一段稳态悬停（判据：|v| ≤ %.2f m/s ∧ 倾角 ≤ %.1f° ∧ 连续 ≥ %.1f s）"
                      % (v_eps, np.degrees(tilt_eps_rad), min_duration)
        }
    # 段长不够也不要 —— 太短的话位置环的积分还没把差补上（同 `hover-thrust` 的 min_duration）
    span = t[i1 - 1] - t[i0]
    if span < min_duration:
        return None, {"reason": f"最长的稳态段只有 {span:.2f} s（< {min_duration:g} s）"}

    v = float(np.median(voltage[i0:i1]))
    i = float(np.mean(current[i0:i1]))  # ★ 窗口**均值**，不是中位：噪声零均值时均值才是无偏的
    thrust = float(mass) * G_DEFAULT
    return Anchor(
        t0=float(t[i0]), t1=float(t[i1 - 1]), n=int(i1 - i0),
        thrust_n=thrust, voltage_v=v, current_a=i, power_w=v * i,
        notes=[
            "电流取的是窗口**均值**（单点不可用，见 esr_and_signature 的 verdict）",
            f"该窗口内电流的样本级 1σ 抖动 = {robust_noise(current[i0:i1]):.3f} A",
        ],
    ), {}


def throughput_verdict(f_max, i_at_f_max=None, bat_current_rating=None):
    """把"电池顶不顶得住 `f_max`"写成**一条机器判据**，而不是一句印象。

    ★ 判据只看**两个**输入：外推出来的 `i_at_f_max` 与电芯的持续放电额定。
    `f_max` 只在输出里**回显**（便于报告自解释），不参与判断 —— 判断用的是比例。
    ⚠️ 这里曾经收过 `sustained_a / mass / r_int / v_ref / i_ref / n` 六个参数而**一个都没读**：
    `sustained_a` 尤其误导（调用方明明白白把"可持续电流"传进来，看着像判据的主输入）。
    实测不需要就删掉 —— 挂着不用的参数比缺参数更坏。

    ★ 判据用**比例**（占额定电流的多少），不用绝对安培 —— 因为电流有一条未标定的标度
    （`BAT1_A_PER_V`），比例在标度错了 k 倍的**同时**也一样错 k 倍；所以这条判据只在
    "`BAT1_A_PER_V` 大致正确"的前提下成立，报告里必须把这条前提写出来。
    """
    out = {
        "f_max_n": None if f_max is None else float(f_max),
        "current_at_f_max_a": None if i_at_f_max is None else float(i_at_f_max),
        "rating_a": None if bat_current_rating is None else float(bat_current_rating),
        "fraction_of_rating": None,
        "verdict": "unknown",
        "premise": "`BAT1_A_PER_V` 的标度大致正确（日志内无法独立验证 —— 见报告 missing 一节）",
    }
    if i_at_f_max is None or not bat_current_rating:
        return out
    frac = float(i_at_f_max) / float(bat_current_rating)
    out["fraction_of_rating"] = frac
    out["verdict"] = "conservative" if frac < 0.75 else ("marginal" if frac <= 1.0 else "infeasible")
    return out
