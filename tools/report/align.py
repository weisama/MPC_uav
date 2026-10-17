#!/usr/bin/env python3
"""对齐口径的纯函数集合：不读包、不碰文件，只做坐标与时间轴变换。

`build.py` 把各源的原始序列读成 `(t, x)`，然后统一从这里取口径。放在一处的好处是
**口径可测**：票 01 的合成用例与变异自检都直接打这些函数，不必起 ROS。

三个坐标约定（与 `docs/report/data/README.md` 必须逐字一致）：

1. 时间：秒，零点 `t = 0` 是 `/traj/start` 的**包记录时间戳**；
2. 位置：米，原点由 `AUTO/WAIT_START` 期间的**指令值**锁存（⛔ 不是起飞位置）；
3. 姿态：四元数一律 `[w, x, y, z]`（与 PX4 / `x0[6:10]` 一致），欧拉角是 ZYX 内旋。
"""

from __future__ import annotations

import numpy as np

#: 公共网格频率（Hz）。全仓只在这里定义一次，`build.py` 从这里取。
HZ = 100.0

# `AUTO/WAIT_START` 期间指令的允许散布（米）。六份包的实际散布由
# `BagExtract.origin_spread_m` 记录，黄金用例断言它 < 1e-6；这里留同一容差，
# 只是为了不让浮点尾数把好数据判死。
_ORIGIN_TOL_M = 1e-6


def find_state_window(t: np.ndarray, states: np.ndarray, state: str) -> tuple[float, float]:
    """找 `state` 的**首个**连续区间，返回 `[start, end)`（秒）。

    只取首个区间：原点锁存发生在第一次进入 `AUTO/WAIT_START`，之后（若重试）的
    指令值可能已经带上偏移。找不到直接抛错 —— 静默退化会让整条管线换口径。
    """
    t = np.asarray(t, dtype=float)
    mask = np.asarray(states) == state
    idx = np.flatnonzero(mask)
    if idx.size == 0:
        raise ValueError(f"找不到状态 {state}")
    start_i = idx[0]
    start = float(t[start_i])
    # 从 start_i 往后找第一处离开该状态的时刻。
    for j in range(start_i + 1, len(t)):
        if not mask[j]:
            return start, float(t[j])
    return start, float(t[-1])


def origin_from_command(cmd_t: np.ndarray, cmd_p: np.ndarray,
                        window: tuple[float, float],
                        tol: float = _ORIGIN_TOL_M) -> tuple[np.ndarray, float]:
    """取 `window` 内指令位置的恒定值作为轨迹原点。

    FSM 在 `AUTO/WAIT_START` 就锁存原点，比 `/traj/start` 早；拿后者时刻的飞机位置
    当原点会引入 6.5–10.5 cm 的系统偏差（票 01 口径 2）。

    返回 `(origin, spread)`；`spread` 是各样本到均值的最大距离。指令不恒定就抛
    `ValueError` —— 宁可整条管线失败，也不要拿均值糊过去。
    """
    cmd_t = np.asarray(cmd_t, dtype=float)
    cmd_p = np.asarray(cmd_p, dtype=float)
    lo, hi = window
    sel = (cmd_t >= lo) & (cmd_t < hi)
    if not np.any(sel):
        raise ValueError(f"窗口 [{lo:.6f}, {hi:.6f}) 内没有指令样本")
    pts = cmd_p[sel]
    origin = pts.mean(axis=0)
    spread = float(np.linalg.norm(pts - origin, axis=1).max())
    if spread > tol:
        raise ValueError(
            f"WAIT_START 期间指令不恒定：最大散布 {spread:.3e} m > {tol:.0e} m"
            f"（{int(sel.sum())} 个样本）")
    return origin, spread


def absolute_target(origin: np.ndarray, ref_p: np.ndarray) -> np.ndarray:
    """`/traj/reference.position` 是**相对**量：绝对目标 = 原点 + 参考。

    只在 TRACK 窗内成立（WAIT_START 期间参考流还没开始）。形状错误直接抛错，
    不做广播兜底 —— `(n, 3)` 与 `(3,)` 混用会静默算出错维度。
    """
    origin = np.asarray(origin, dtype=float)
    ref_p = np.asarray(ref_p, dtype=float)
    if origin.shape != (3,):
        raise ValueError(f"原点应为 (3,)，实际 {origin.shape}")
    if ref_p.ndim != 2 or ref_p.shape[1] != 3:
        raise ValueError(f"参考位置应为 (n, 3)，实际 {ref_p.shape}")
    return origin[None, :] + ref_p


def position_at(t: np.ndarray, p: np.ndarray, tq: float) -> np.ndarray:
    """在 `tq` 处线性取位置；越界顶住端点（不外推）。"""
    t = np.asarray(t, dtype=float)
    p = np.asarray(p, dtype=float)
    if t.size < 2:
        raise ValueError("位置序列至少需要 2 个样本")
    return np.array([float(np.interp(tq, t, p[:, i])) for i in range(p.shape[1])])


def completion_from_progress(t: np.ndarray, progress: np.ndarray,
                             t0: float) -> tuple[float, str]:
    """完成时刻 = **首个** `progress >= 1.0`。

    之后的参考会继续发（常常是零），拿末帧判完成会把窗口拉长到降落段。
    """
    t = np.asarray(t, dtype=float)
    progress = np.asarray(progress, dtype=float)
    hit = np.flatnonzero(progress >= 1.0)
    if hit.size == 0:
        raise ValueError("progress 从未达到 1.0，请改走参考流末尾")
    return float(t[hit[0]] - t0), "progress"


def completion_from_reference(t: np.ndarray, t0: float) -> tuple[float, str]:
    """没有 `progress=1.0` 时（悬停包被截断）截止到参考流末尾。"""
    t = np.asarray(t, dtype=float)
    if t.size == 0:
        raise ValueError("参考流为空")
    return float(t[-1] - t0), "ref_end"


def common_end(ends: dict[str, float]) -> float:
    """公共窗右端 = 各源完成时刻的**最小值**（短的那条说了算）。"""
    if not ends:
        raise ValueError("没有源就没有公共窗")
    return float(min(ends.values()))


def quat_to_euler(q: np.ndarray) -> np.ndarray:
    """四元数 `[w, x, y, z]` → 欧拉角 `[roll, pitch, yaw]`（ZYX 内旋，弧度）。"""
    q = np.asarray(q, dtype=float)
    w, x, y, z = q[:, 0], q[:, 1], q[:, 2], q[:, 3]
    roll = np.arctan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    pitch = np.arcsin(np.clip(2.0 * (w * y - z * x), -1.0, 1.0))
    yaw = np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return np.column_stack([roll, pitch, yaw])


def euler_to_quat(e: np.ndarray) -> np.ndarray:
    """欧拉角 `[roll, pitch, yaw]` → 四元数 `[w, x, y, z]`（`quat_to_euler` 的逆）。"""
    e = np.asarray(e, dtype=float)
    r, p, y = e[:, 0] / 2.0, e[:, 1] / 2.0, e[:, 2] / 2.0
    cr, sr = np.cos(r), np.sin(r)
    cp, sp = np.cos(p), np.sin(p)
    cy, sy = np.cos(y), np.sin(y)
    return np.column_stack([
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
    ])


def interp_columns(t: np.ndarray, x: np.ndarray, tq: np.ndarray) -> np.ndarray:
    """在多个查询时刻 `tq` 上逐列线性取值；越界顶住端点（与 `position_at` 同口径）。

    返回 `(len(tq), x.shape[1])`。给 `_target_identity` 这类"逐样本对表"用，
    避免每个调用点各写一遍 `np.interp` 列循环。
    """
    t = np.asarray(t, dtype=float)
    x = np.asarray(x, dtype=float)
    tq = np.asarray(tq, dtype=float)
    if t.size < 2 or x.ndim != 2 or x.shape[0] != t.size:
        raise ValueError(f"插值输入形状不一致：t {t.shape} / x {x.shape}")
    return np.column_stack([np.interp(tq, t, x[:, i]) for i in range(x.shape[1])])


def build_grid(end_s: float, hz: float = HZ) -> np.ndarray:
    """公共网格 `[0, end_s]`，步长 `1/hz`；末端点贴上边界，避免累积浮点误差漏掉。"""
    if end_s <= 0.0:
        raise ValueError(f"窗口长度必须为正：{end_s}")
    n = int(np.floor(end_s * hz + 1e-9))
    grid = np.arange(n + 1, dtype=float) / hz
    if grid[-1] < end_s - 1e-12:
        grid = np.append(grid, end_s)
    return grid


def resample_linear(t: np.ndarray, x: np.ndarray, grid: np.ndarray) -> np.ndarray:
    """逐列线性插值；`grid` 越出 `[t[0], t[-1]]` 的部分顶住端点。

    顶住而不是外推：`/traj/reference` 常比 `/traj/start` 晚 10–20 ms，
    外推会在 t=0 造出一个从未发过的值。
    """
    t = np.asarray(t, dtype=float)
    x = np.asarray(x, dtype=float)
    if t.size < 2:
        raise ValueError("插值序列至少需要 2 个样本")
    if x.ndim != 2 or x.shape[0] != t.size:
        raise ValueError(f"序列形状不一致：t {t.shape} / x {x.shape}")
    grid = np.asarray(grid, dtype=float)
    return np.column_stack([np.interp(grid, t, x[:, i]) for i in range(x.shape[1])])


def resample_zoh(t: np.ndarray, x: np.ndarray, grid: np.ndarray) -> np.ndarray:
    """零阶保持：`grid` 处取它之前最近的一次指令。控制量不该被线性插值。"""
    t = np.asarray(t, dtype=float)
    x = np.asarray(x, dtype=float)
    if t.size < 1:
        raise ValueError("保持序列至少需要 1 个样本")
    if x.ndim != 2 or x.shape[0] != t.size:
        raise ValueError(f"序列形状不一致：t {t.shape} / x {x.shape}")
    grid = np.asarray(grid, dtype=float)
    idx = np.searchsorted(t, grid, side="right") - 1
    idx = np.clip(idx, 0, t.size - 1)
    return x[idx]


def resample_quat(t: np.ndarray, q: np.ndarray, grid: np.ndarray) -> np.ndarray:
    """四元数（`[w, x, y, z]`）沿最短弧 slerp；越界顶住端点。

    同一个姿态有 `q` 与 `-q` 两种写法。逐点插值前先用点积判向，否则会在符号
    翻转处插出一条 180° 的假旋转。
    """
    t = np.asarray(t, dtype=float)
    q = np.asarray(q, dtype=float)
    if t.size < 2:
        raise ValueError("插值序列至少需要 2 个样本")
    if q.ndim != 2 or q.shape[1] != 4 or q.shape[0] != t.size:
        raise ValueError(f"四元数形状应为 (n, 4)，实际 {q.shape}")
    grid = np.asarray(grid, dtype=float)

    # 先在原始序列上统一符号，再做逐点插值。
    q = q.copy()
    for i in range(1, len(q)):
        if np.dot(q[i - 1], q[i]) < 0.0:
            q[i] = -q[i]

    hi = np.searchsorted(t, grid, side="right")
    hi = np.clip(hi, 1, t.size - 1)
    lo = hi - 1
    span = t[hi] - t[lo]
    frac = np.clip((grid - t[lo]) / np.where(span > 0, span, 1.0), 0.0, 1.0)

    q0, q1 = q[lo], q[hi]
    dot = np.sum(q0 * q1, axis=1)
    # 原始序列已统一符号；这里再兜一次，防御调用方直接喂未排序/未统一的序列。
    flip = dot < 0.0
    dot = np.where(flip, -dot, dot)
    q1 = np.where(flip[:, None], -q1, q1)

    out = np.empty_like(q0)
    lin = dot > 0.9995          # 夹角极小：线性插值 + 归一化更稳
    out[lin] = q0[lin] + frac[lin, None] * (q1[lin] - q0[lin])
    out[lin] /= np.linalg.norm(out[lin], axis=1, keepdims=True)

    rest = ~lin
    if np.any(rest):
        theta = np.arccos(np.clip(dot[rest], -1.0, 1.0))
        sin_theta = np.sin(theta)
        w0 = np.sin((1.0 - frac[rest]) * theta) / sin_theta
        w1 = np.sin(frac[rest] * theta) / sin_theta
        out[rest] = w0[:, None] * q0[rest] + w1[:, None] * q1[rest]
    return out
