"""从飞控 ulog 反推模型里的 `hover_thrust` —— **纯逻辑**。

本模块**不 import** `pyulog` / `pymavlink` / `rclpy`，也不读文件：它只吃几条
已经对齐到**同一时间基**的数组，吐一个估计。读 ulog 的适配层在 `hover_thrust_from_ulog.py`。
这样"哪一段才算悬停""取中位还是均值"可以拿**合成信号**自检（`--self-test`），
不需要飞机、不需要日志、不需要飞控通电。

## 量的是什么

`hover_thrust` = **`T_hover / T_max`** —— 归一化推力坐标里悬停所占的比值。
第一手依据（`v1.14.3`，tag `1dacb4c`，逐跳读出来的）：

1. `mc_rate_control` 的 offboard 分支把 `thrust_body` **原样**放进 `vehicle_thrust_setpoint`
   （`src/modules/mc_rate_control/MulticopterRateControl.cpp:182`）；
2. `control_allocator` 把它当期望力旋量收下（`ControlAllocator.cpp:378`、`:398-400`）；
3. 伪逆分配矩阵的**推力列按各执行器 `ct` 的均值归一化**，于是"推力 = 1.0"正好等于
   "所有电机打满"（`ControlAllocationPseudoInverse.cpp:118-139` + `normalizeControlAllocationMatrix()`）；
4. 输出级的推力曲线 `THR_MDL_FAC` 只做 `rel_thrust ↔ 电机信号` 的换算，
   **输入就是归一化推力、且不涉及任何悬停油门**
   （`src/lib/mixer_module/functions/FunctionMotors.hpp:77-101`）。

⟹ 电机打满时飞机能出多少推力，`T_max` 就是多少；而**打满时给不出电机的 40 N，
只给得出电池限制的 ≈11.8 N**（[`phase2-implementation.md`](../../docs/control/phase2-implementation.md) §3.3）
⟹ 悬停时 `|thrust_body[2]|` 就是 `T_hover / T_max`，与模型里的 `hover_thrust` **同一个量**。
这也正是"两者必须自洽"那条判据的落点：PX4 中间**没有任何一处**做悬停油门重标定。

## 为什么这**不是**在复述一个参数

有人会问：悬停时位置控制器输出的推力难道不就是它自己的 `MPC_THR_HOVER` 吗？
**不是** —— 实测反证：SITL 的 z0 同构机 `MPC_THR_HOVER` 取的是 PX4 默认 **0.5**，
而被控对象真实的 `T_hover/T_max` = `0.75×9.80665/2.95/4` = **0.6233**；
日志里量出来的正是 **0.6233**，不是 0.5。原因是**位置环的积分**：
悬停要保持高度，实际推力**必须**等于 `m·g`，信念错了由积分补上。
⟹ 只要那一段**真的是稳态悬停**（这是本模块唯一要判的事），读数收敛到真值。

## 符号约定

PX4 是 **NED/FRD**：悬停时 `thrust_body[2]` **为负**（推力沿机体 −z）。
本模块因此取 `|median|`，但**不静默取绝对值** —— 若整段的 `thrust_body[2]` 是正的，
那是"悬停着的飞机在往下推"，物理上不可能，说明段选错了或约定变了，
一律置 `sign != "negative"`、`usable = False` 报出来。
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

__all__ = [
    "G_DEFAULT",
    "HoverSegment",
    "HoverThrustEstimate",
    "tilt_rad_from_quat",
    "interp_to",
    "steady_hover_mask",
    "sustained_error_bound",
    "runs_of",
    "segment_at",
    "estimate_hover_thrust",
    "t_max_for",
    "relative_gap",
]

#: 重力加速度 [m/s²] —— 与 `nmpc/params.py` 的 `G` 同一个值。
#:
#: ⚠️ 这里写的是 9.80665（= PX4 的 `CONSTANTS_ONE_G`），**而 `nmpc/params.py` 写的是 9.81**
#: —— 那是照抄参考实现 `multirotor_rate_model.py` 的 `g`，刻意不改（ticket 10 只动 `hover_thrust`）。
#: 两者差 0.03 %，落在本工具的不确定度里；**但它是两个文件里各写一份的同一个物理量**，
#: 所以本模块**不** import 包里的 `G`：`tools/` 下的分析脚本与 `src/` 下的飞行代码
#: 各自可跑，不该为了一个常数把分析工具绑到 ROS 包上。比大小的地方一律用 `relative_gap()`。
G_DEFAULT = 9.80665


# ------------------------------------------------------------------ 姿态


def tilt_rad_from_quat(q_wxyz):
    """由 `vehicle_attitude.q`（**w, x, y, z**，NED/FRD）算**倾角** [rad]。

    倾角 = 机体 z 轴与世界"下"（NED 的 +z）之间的夹角 = `arccos(R[2][2])`。

    而 `R[2][2] = 1 − 2(qx² + qy²)` —— 所以**根本不用建旋转矩阵**，
    也用不到 `qw`/`qz`：绕 z 的偏航**按构造对倾角免疫**（这正是我们要的：
    悬停时转多少偏航都不该被判成"在机动"）。
    """
    q = np.asarray(q_wxyz, dtype=float)
    if q.shape[-1] != 4:
        raise ValueError(f"四元数要 4 个分量 (w,x,y,z)，拿到 {q.shape}")
    qx, qy = q[..., 1], q[..., 2]
    r22 = 1.0 - 2.0 * (qx * qx + qy * qy)
    return np.arccos(np.clip(r22, -1.0, 1.0))


# ------------------------------------------------------------------ 时间基


def interp_to(t_src, x_src, t_dst):
    """把 `(t_src, x_src)` 线性插值到 `t_dst` 上。

    ★ **落到源区间之外的点返回 `NaN`，不夹逼（clamp）**。
    夹逼会**凭空造出**一段"看起来完全合理的悬停数据"：把飞行窗口外的最近值
    复制到窗口里，速度是 0、倾角是 0 —— 恰好满足稳态悬停的每一条判据。
    这类假数据在图上没有任何异常，只会让 `hover_thrust` 悄悄偏掉。
    让它是 `NaN`，`steady_hover_mask` 直接判 False。
    """
    t_src = np.asarray(t_src, dtype=float)
    x_src = np.asarray(x_src, dtype=float)
    t_dst = np.asarray(t_dst, dtype=float)
    if t_src.size < 2:
        return np.full(t_dst.shape, np.nan)
    return np.interp(t_dst, t_src, x_src, left=np.nan, right=np.nan)


# ------------------------------------------------------------------ 稳态判据


def steady_hover_mask(
    t,
    speed,
    tilt_rad,
    thrust_z,
    *,
    v_eps=0.30,
    tilt_eps=math.radians(10.0),
    thrust_abs_min=0.15,
    thrust_abs_max=0.98,
):
    """哪些样本**算稳态悬停** —— 唯一由本模块判的事。

    | 判据 | 默认 | 挡掉什么 |
    |---|---|---|
    | **三维**速度模 `|v| ≤ v_eps` | 0.30 m/s | 在飞的机动/巡航段（悬停时速度停在零附近） |
    | 倾角 `≤ tilt_eps` | 10° | 悬停但侧倾着顶风/顶位置误差 —— 那时推力被拿去抵消水平分量，不再等于 `m·g` |
    | `thrust_abs_min ≤ |thrust_z|` | 0.15 | 地面怠速/停桨段（`thrust_body` 贴近 0） |
    | `|thrust_z| ≤ thrust_abs_max` | 0.98 | 已经打满的段（不是悬停） |
    | 四项全**有限** | — | `NaN` —— 包括 `interp_to` 落在源区间外造出来的那些 |

    ★ **定量的那个判据是倾角，不是速度** —— 别把 `v_eps` 当精度旋钮。
    推力只在"竖直、且没有竖直加速度"时才等于 `m·g`：
    侧倾 `θ` 时竖直分量是按 `cos θ` 拿的，所以 `tilt_eps = 10°` 直接给出**读数误差上界
    `1 − cos 10° = 1.5%`**。速度只是个粗筛（"它没有在飞"），它自己不给出任何误差界；
    真机上悬停也不是速度严格为零 —— 动捕在环的位置环本来就允许厘米级漂移。
    所以 `v_eps = 0.30`：把"慢漂"留下、把"机动"挡掉，而精度交给倾角那一项去保证。

    ★ 速度门与"段内一致性"是**互补**的，两者一起才给出读数的误差界：
    "推力不等于 `m·g`"必然意味着**存在加速度**（受力平衡），竖直加速度 `a_z`
    直接给出该样本的推力相对误差 `|a_z|/g`。掩码是**逐样本**判的，
    所以一段**连通**的段里每个样本都满足 `|v| ≤ v_eps`，
    段的**速度变化**因此被卡在 `2·v_eps` 以内 ⟹ 想整段持续偏 `δ`（需要 `a_z = δ·g`）
    就必须 `δ·g·T ≤ 2·v_eps`，即
    **`δ ≤ 2·v_eps / (g·T)`**（默认 0.30 / 9.80665 / 2 s ⟹ **3.1%**）。
    实测那些 0.46 左右的坏段就是另一条路：`a_z ≈ −0.26 g`，
    只要 0.1 s 速度就出界，于是它**只能**表现为段内不一致（30–41%），被 `block_spread` 挡掉。

    ⚠️ 所以 `v_eps` 不是"宽容度"、`min_duration` 也不是"多凑点样本"：
    这三个数由上面那条界**绑在一起**，改一个就要重算另外两个。
    `sustained_error_bound()` 就是这条界，自检里有用例钉着它。

    ⚠️ **不要**在这里加"thrust 变化率小"之类的判据：`thrust_body` 是 20 Hz 记录的控制量，
    它天然在抖（实测 SITL 稳态段 std ≈ 0.02）；把抖动当机动会把**所有**段都判掉。
    段内的漂移另由 `segment_at()` 的 `drift` 报出来，不在这里一刀切。

    ⚠️ **刻意不判"符号必须是负"**：那会让"符号错了"表现为**空掩码**（看起来像"日志里没悬停"），
    而真因是约定变了。符号在 `estimate_hover_thrust` 里单独报。
    """
    t = np.asarray(t, dtype=float)
    speed = np.asarray(speed, dtype=float)
    tilt_rad = np.asarray(tilt_rad, dtype=float)
    thrust_z = np.asarray(thrust_z, dtype=float)
    for name, arr in (("speed", speed), ("tilt_rad", tilt_rad), ("thrust_z", thrust_z)):
        if arr.shape != t.shape:
            raise ValueError(f"{name} 与 t 长度对不上：{arr.shape} vs {t.shape}")

    finite = np.isfinite(speed) & np.isfinite(tilt_rad) & np.isfinite(thrust_z)
    return (
        finite
        & (np.abs(speed) <= v_eps)
        & (np.abs(tilt_rad) <= tilt_eps)
        & (np.abs(thrust_z) >= thrust_abs_min)
        & (np.abs(thrust_z) <= thrust_abs_max)
    )


def runs_of(mask, t, *, min_duration=2.0):
    """把布尔掩码切成 `[(i0, i1), ...]`（**半开区间**），只留时长 ≥ `min_duration` 的段。

    ⚠️ `min_duration` 不是可有可无的过滤器：悬停段越短，"实际推力 = m·g"这个前提
    越可能还没成立（位置环的积分还在补差、或者飞机正在穿越悬停点）。
    实测 SITL 的每份日志里最长稳态段有 200–400 个样本（10–20 s），取 2 s 是**宽松**的。
    """
    mask = np.asarray(mask, dtype=bool)
    t = np.asarray(t, dtype=float)
    if mask.size != t.size:
        raise ValueError(f"mask 与 t 长度对不上：{mask.size} vs {t.size}")
    if mask.size == 0 or not mask.any():
        return []

    edges = np.flatnonzero(np.diff(np.concatenate(([False], mask, [False])).astype(np.int8)))
    out = []
    for i0, i1 in zip(edges[0::2], edges[1::2]):
        if t[i1 - 1] - t[i0] >= min_duration:
            out.append((int(i0), int(i1)))
    return out


# ------------------------------------------------------------------ 段的读数


@dataclass(frozen=True)
class HoverSegment:
    """一段候选悬停段的读数。区间是半开的 `[i0, i1)`。"""

    i0: int
    i1: int
    t0: float
    t1: float
    n: int
    thrust: float
    """该段 `|thrust_body[2]|` 的**中位数** —— 主读数。

    ★ 为什么是中位数不是均值：控制量带尖峰（位置环在纠瞬间误差），均值被尖峰拉偏；
    中位数对"少数拍打得特别狠"免疫。有自检用例钉着（第 11 条）。
    """

    thrust_z_median: float
    """该段 `thrust_body[2]` **带符号**的中位数。悬停时必须是**负**的（NED/FRD）。"""
    iqr: float
    """四分位距 —— 段内离散度。**不是**标准差：抖动不是高斯，std 会被尾部放大。"""

    block_spread: float
    """**段内**一致性：把段切成若干等份，各份中位数之间的极差，再除以整段中位数。

    为什么一定要报它：悬停段里电池在掉压（或位置环积分还在爬），
    于是"悬停推力"在这段里**不是一个数**。实测 SITL 稳态段 < 1 %；超过 10 % 判不可用。

    ★ **不用"前后半段中位数之差"**：那条判据漏得很难看 —— 台阶落在靠前的位置时，
    前半段的中位数已经被台阶后面那部分主导，前后之差**归零**。
    实测过一个反例：段首 1/6 处从 0.6233 跳到 0.7480，前后半段之差是 **0.0**。
    切成多份取极差则没有这个盲区，且对噪声不敏感（块中位数的标准误远小于原始抖动）。
    """


def segment_at(t, thrust_z, i0, i1, *, n_blocks=5):
    """算一段区间的读数。`thrust_z` 是原始 `thrust_body[2]`（**带符号**，悬停时为负）。

    `HoverSegment.thrust` 一律取 `|median|`（正数，方便与 `hover_thrust` 直接比）；
    **符号本身**由 `thrust_z_median` 单独带出去，绝不在这里丢掉 —— 见模块 docstring 的符号约定。
    """
    t = np.asarray(t, dtype=float)
    thrust_z = np.asarray(thrust_z, dtype=float)
    seg = thrust_z[i0:i1]
    z_median = float(np.median(seg))
    mid = abs(z_median)

    block_spread = 0.0
    if mid > 0.0 and seg.size >= 2 * n_blocks:
        meds = [-float(np.median(b)) for b in np.array_split(seg, n_blocks)]
        block_spread = (max(meds) - min(meds)) / mid

    return HoverSegment(
        i0=int(i0),
        i1=int(i1),
        t0=float(t[i0]),
        t1=float(t[i1 - 1]),
        n=int(seg.size),
        thrust=mid,
        thrust_z_median=z_median,
        iqr=float(np.percentile(-seg, 75) - np.percentile(-seg, 25)),
        block_spread=float(block_spread),
    )


# ------------------------------------------------------------------ 估计


@dataclass(frozen=True)
class HoverThrustEstimate:
    """`hover_thrust` 的估计 + **它能不能用**，以及为什么。"""

    hover_thrust: float | None
    """主口径：**最长**那段稳态悬停的中位数 `|thrust_body[2]|`。`usable=False` 时为 None。"""

    segment: HoverSegment | None
    segments: tuple = field(default_factory=tuple)
    """全部候选段，按样本数降序。第一个就是主口径用的那段。"""

    segment_spread: float = 0.0
    """**段间**一致性：各候选段的中位数之间的极差，再除以主口径。

    为什么不用"全部段合并后取中位数"当交叉口径：**段不等长时它会骗人** ——
    只要最长的那段占了过半样本，合并中位数就**等于**主口径，两边永远"一致"。
    实测过一个反例：两段 0.6233 与 0.7480，长的占 60 %，合并中位数仍然是 0.7480。

    段间极差直接回答"**换个段会不会换个答案**"。超过 `spread_tol` 就置 `usable=False`：
    那说明这个量在当前日志里本来就不是一个数，不该挑一个交差。
    """

    sign: str = "unknown"
    """`"negative"`（正常）/ `"positive"`（悬停时推力朝下 —— 不可能）/ `"unknown"`。"""

    usable: bool = False
    notes: tuple = field(default_factory=tuple)
    """不可用 / 需要留意的地方，**逐条**列，不含糊过去。"""


def estimate_hover_thrust(
    t,
    speed,
    tilt_rad,
    thrust_z,
    *,
    min_duration=2.0,
    spread_tol=0.05,
    drift_tol=0.10,
    iqr_tol=0.05,
    **mask_kwargs,
):
    """从对齐好的四条序列估 `hover_thrust`。**永不抛**（空输入也是返回 `usable=False`）。"""
    mask = steady_hover_mask(t, speed, tilt_rad, thrust_z, **mask_kwargs)
    spans = runs_of(mask, t, min_duration=min_duration)
    if not spans:
        return HoverThrustEstimate(
            hover_thrust=None,
            segment=None,
            notes=(
                f"没有一段稳态悬停够 {min_duration:g} s"
                "（判据：|v| ≤ v_eps、倾角 ≤ tilt_eps、|thrust_z| 在 [min,max] 内；全段皆需有限）",
            ),
        )

    segments = tuple(
        sorted((segment_at(t, thrust_z, i0, i1) for i0, i1 in spans), key=lambda s: -s.n)
    )
    best = segments[0]

    # 符号：整段带符号的中位数。悬停时 thrust_body[2] 必须为负（NED/FRD）。
    if best.thrust_z_median < 0.0:
        sign = "negative"
    elif best.thrust_z_median > 0.0:
        sign = "positive"
    else:
        sign = "unknown"
    if sign != "negative":
        return HoverThrustEstimate(
            hover_thrust=None,
            segment=best,
            segments=segments,
            sign=sign,
            notes=(
                f"整段 thrust_body[2] 的中位数是 {best.thrust_z_median:+.4f}，"
                "悬停时它应当是**负**的（PX4 为 NED/FRD，推力沿机体 −z）⟹ "
                "要么这段不是悬停，要么符号约定变了 —— 两种都不能拿去回填",
            ),
        )

    # 段间一致性：各段中位数的极差（只有一段时为 0）。
    segment_spread = 0.0
    if len(segments) > 1:
        seg_thrusts = [s.thrust for s in segments]
        segment_spread = (max(seg_thrusts) - min(seg_thrusts)) / best.thrust

    notes = []
    if best.block_spread > drift_tol:
        notes.append(
            f"段内不一致 {best.block_spread:.1%}（切成 5 份后各份中位数的极差）"
            f"超过 {drift_tol:.0%} —— 这一段里悬停工作点在动（电池掉压 / 积分还在爬），"
            "读数不是一个数"
        )
    if segment_spread > spread_tol:
        notes.append(
            f"段间不一致 {segment_spread:.1%}（各候选段 {[round(s.thrust, 4) for s in segments]} "
            f"的极差）超过 {spread_tol:.0%} —— 换个段就换个答案，"
            "说明它在日志里本来就不是一个数，不该挑一个交差"
        )
    if best.iqr > iqr_tol:
        notes.append(
            f"段内四分位距 {best.iqr:.4f} 超过 {iqr_tol:.4f} —— 推力在明显抖，不是稳的悬停"
        )

    return HoverThrustEstimate(
        hover_thrust=best.thrust,
        segment=best,
        segments=segments,
        segment_spread=segment_spread,
        sign=sign,
        usable=not notes,
        notes=tuple(notes),
    )


# ------------------------------------------------------------------ 导出与比对


def t_max_for(hover_thrust, mass, g=G_DEFAULT):
    """`T_max = m·g / hover_thrust` [N] —— 与 `nmpc/params.py` 的 `Z0Params.t_max` **同一条**导出关系。

    两边各写一份是**刻意**的（见 `G_DEFAULT` 的说明）：本函数是**复核用**的，
    它存在的意义是"回填 `hover_thrust` 之后，`t_max` 会变成多少"能当场算出来，
    而不是要替代 `params.py` 里那条 property。
    """
    if not (0.0 < hover_thrust < 1.0):
        raise ValueError(f"hover_thrust 必须落在 (0,1)，拿到 {hover_thrust}")
    if mass <= 0.0:
        raise ValueError(f"质量必须为正，拿到 {mass}")
    return mass * g / hover_thrust


def sustained_error_bound(v_eps=0.30, min_duration=2.0, g=G_DEFAULT):
    """稳态段**持续**偏掉时，读数的相对误差上界 `δ ≤ 2·v_eps / (g·T)`。

    推导（也是为什么 `v_eps` / `min_duration` 不能各改各的）：掩码逐样本判 `|v| ≤ v_eps`，
    所以一段连通段里速度变化不超过 `2·v_eps`；而读数整段持续偏 `δ` 需要恒定竖直加速度
    `a_z = δ·g`（受力平衡），于是 `δ·g·T ≤ 2·v_eps`。

    默认值 ⟹ **3.1%**。真瞬态（速度还没长起来的那种）不走这条路，
    由段内一致性 `block_spread` 拦 —— 两条合起来才是完整的误差界。
    """
    if v_eps <= 0.0 or min_duration <= 0.0:
        raise ValueError(f"v_eps 与 min_duration 都必须为正：{v_eps}, {min_duration}")
    return 2.0 * v_eps / (g * min_duration)


def relative_gap(a, b):
    """两个数的**相对**差距 `|a−b| / max(|a|,|b|)`（两个都为 0 时返回 0）。

    比大小一律用它，不比绝对差：`hover_thrust` 是 0.6 量级、`t_max` 是 11 量级，
    同一个"差多少算对得上"在两种量纲上不是同一个绝对数。
    """
    a, b = float(a), float(b)
    scale = max(abs(a), abs(b))
    return 0.0 if scale == 0.0 else abs(a - b) / scale
