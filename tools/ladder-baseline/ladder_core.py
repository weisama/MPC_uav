"""阶梯判据表的**纯逻辑** —— TRACK 窗口提取、同口径跟踪误差、时间对齐。

本模块**不 import** `rclpy` / `rosbag2_py` / `numpy` 之外的任何东西，也**不读文件**：
它只吃「时刻 + 位置」两条序列，吐一组指标。读 bag 的适配层在 `ladder_from_bag.py`。
这样「误差和口径是怎么算的」可以拿**合成信号**自检（`--self-test`），不需要包、不需要飞机。

## 三个必须分清的词

- **指令**（command）：控制器**当下要求**飞机在哪。这是"误差"的零点。
- **实际**（actual）：动捕量到的飞机在哪。恒取 `/mocap/pose_enu` —— 两侧同源，
  否则比的是两台仪器的差、不是两个控制器的差。
- **口径**（caliber）：指令那一侧**取自哪个话题**。
  (A) 级取 `/mavros/setpoint_raw/local`，(C) 级取 `/mpc/reference`。
  ★★ **口径不一致的两个数不能相减** —— 那是最容易被当成"MPC（模型预测控制）更好"的东西。

## 误差与时间对齐

未对齐误差 `e(t) = p_实际(t) − p_指令(t)` 是**唯一**有物理意义的定义（它就是"跟踪误差"）。
但 (A) 级的 e(t) 里混着两样东西：**真正的形状跟踪能力** 和 **一段起机爬升滞后**。
`best_shift` 用一个纯时移去吸收后者，好让前者露出来：

    dt* = argmin_dt  mean_t | p_实际(t) − p_指令(t + dt) |

`dt* < 0` 表示**实际滞后于指令**（要往回取指令才对得上）。
★★ 它是**诊断量，不是验收量**：验收线只认未对齐的那个 `e(t)`。
对齐后的数只是用来回答"这 0.57 m 到底是跟不上还是只是慢起机"。

术语：`dt` 恒指**对齐时作用在指令时间轴上的偏移**，单位秒，负 = 实际滞后。
"""

from __future__ import annotations

from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

__all__ = [
    "TRACK_TOKEN",
    "track_window",
    "anchor_index",
    "trim_window",
    "make_grid",
    "interp_columns",
    "tracking_metrics",
    "path_length",
    "best_shift",
    "segment_metrics",
    "summarize",
]

#: 控制状态串里代表"正在跟轨迹"的那个记号（`/control/state` 是 `std_msgs/String`）
TRACK_TOKEN = "AUTO/TRACK"


# ------------------------------------------------------------------ 窗口


def track_window(states: Sequence[Tuple[float, str]]) -> Optional[Tuple[float, float]]:
    """从 `[(时刻秒, 状态串), ...]` 里取**含 `AUTO/TRACK` 的最长连续段**。

    返回 `(t0, t1)`；一段都没有就返回 `None`。

    ★ 判据不能拿"动捕动了没有"去猜 —— 那是启发式，`eps` 一改窗口就变
    （`tools/tau-analysis/README.md`「读分段」里记着这条踩过的坑）。
    `/control/state` 里**写着**状态机在哪一档，为什么不读它。

    ★★ **必须取最长连续段，不能在整段首末取首尾**：一次飞行里
    `AUTO/TRACK` 可能出现不止一次（例如先试了一次又退回来），
    首尾取差会把中间的悬停/失控段**整个吞进窗口**、拉低误差。
    """
    best: Optional[Tuple[float, float]] = None
    lo: Optional[float] = None
    prev: Optional[float] = None
    for t, txt in states:
        inside = TRACK_TOKEN in txt
        if inside:
            if lo is None or (prev is not None and t - prev > _GAP_S):
                if lo is not None and prev is not None:
                    best = _longer(best, (lo, prev))
                lo = t
            prev = t
        else:
            if lo is not None and prev is not None:
                best = _longer(best, (lo, prev))
                lo = None
    if lo is not None and prev is not None:
        best = _longer(best, (lo, prev))
    return best


#: 状态串两次报 `AUTO/TRACK` 之间的最长间隔（秒）——超过就当窗口断了。
#: ★★ `/control/state` **只有 1 Hz**（z0 上实测：中位 1000 ms、最大 1038 ms，
#: 见 `evidence/`），所以这个阈值必须**明显大于 1 s**。
#: 设 0.5 s 的话每个采样点都成了独立的一小段（长度 0）⟹ 窗口塌成一个点，
#: `make_grid` 会以"窗口非法"报错 —— 这正是第一版踩的坑。
#: 取 2.5 s = 容忍连丢两拍。
_GAP_S = 2.5


def _longer(a: Optional[Tuple[float, float]], b: Tuple[float, float]) -> Tuple[float, float]:
    if a is None:
        return b
    return b if (b[1] - b[0]) > (a[1] - a[0]) else a


def anchor_index(t_gate: float, t_samples) -> int:
    """模型预测控制锚点规则：闸门**第一次打开**那一拍用的测量，是当时**已收到的最新一帧**。

    返回 `t_samples` 里满足 `t <= t_gate` 的最后一帧下标；一帧都没有（闸门比第一帧
    测量还早）就用第 0 帧 —— 与节点的行为一致：它不会有"未来的测量"。
    空序列直接报错（没有测量就谈不上锚点）。

    ★★ 这是"最近一帧"，**不是"最接近 `t_gate` 的那一帧"**：闸门打开后到达的样本
    在那一拍还不可见，拿它当锚点会把一个**未来位移**算进参考里。
    """
    t = np.asarray(t_samples, dtype=float)
    if t.size == 0:
        raise ValueError("没有测量帧，无法取锚点")
    if np.any(np.diff(t) < 0):
        raise ValueError("t_samples 必须单调不减")
    i = int(np.searchsorted(t, float(t_gate), side="right")) - 1
    return max(i, 0)


def trim_window(
    window: Tuple[float, float],
    skip_s: float = 0.0,
    last_s: Optional[float] = None,
) -> Tuple[float, float]:
    """把窗口**前** `skip_s` 秒切掉，再只留末尾 `last_s` 秒（`None` = 不限）。

    用于阶梯 3 的「系绳悬停 **30 s**」：整段 TRACK 里既有爬升瞬态又有稳态，
    判据只对稳态那 30 s 说话，切法必须写成参数、进 JSON，不能靠人眼挑。
    切完为空就报错 —— 静默给一个零长窗口会让 `make_grid` 在别处炸，看着像网格的问题。
    """
    lo, hi = float(window[0]), float(window[1])
    if skip_s < 0:
        raise ValueError(f"skip_s 不能为负：{skip_s}")
    lo = lo + skip_s
    if last_s is not None:
        if last_s <= 0:
            raise ValueError(f"last_s 必须为正：{last_s}")
        lo = max(lo, hi - float(last_s))
    if lo >= hi:
        raise ValueError(f"切完窗口为空：[{lo}, {hi}]（原窗口 {window}）")
    return (lo, hi)


def make_grid(t0: float, t1: float, hz: float) -> np.ndarray:
    """在 `[t0, t1]` 上建**均匀网格**。

    ★ 两侧必须共享同一个网格 —— 各按自己的 `t[0]` 建网格会把一条序列整体平移，
    那个平移会**原样加进误差**，而且看着完全正常（`tau-analysis` 的坑 1 就是它）。
    ★ `t1` 处的点**包含在内**（`endpoint=True`），否则每段轨迹都会少掉最后一格。
    """
    if t1 <= t0:
        raise ValueError(f"窗口非法：t0={t0} >= t1={t1}")
    n = int(np.floor((t1 - t0) * hz)) + 1
    return t0 + np.arange(n) / hz


def interp_columns(t, x, grid: np.ndarray) -> np.ndarray:
    """把 `(t, x)`（`x` 形状 `(N, C)`）线性插值到 `grid` 上，返回 `(M, C)`。

    `grid` **窗外的点会被夹住**（`np.interp` 的端点行为）——
    所以调用方必须自己保证 `grid` 落在两条序列共同覆盖的时段里，
    对齐用的偏移也一样（见 `best_shift` 的 `dt` 搜索范围）。
    """
    t = np.asarray(t, dtype=float)
    x = np.atleast_2d(np.asarray(x, dtype=float))
    if x.shape[0] != t.size:
        raise ValueError(f"t 有 {t.size} 个点，x 有 {x.shape[0]} 行")
    if t.size < 2:
        raise ValueError("少于 2 个点无法插值")
    if np.any(np.diff(t) <= 0):
        raise ValueError("t 必须严格单调递增")
    return np.stack([np.interp(grid, t, x[:, i]) for i in range(x.shape[1])], axis=1)


# ------------------------------------------------------------------ 指标


def tracking_metrics(actual: np.ndarray, cmd: np.ndarray) -> Dict[str, float]:
    """`actual − cmd` 的一组指标。`*_xy` 是**水平 2 范数**，`*_z` 是**带符号**的垂直误差。

    ★ 垂直误差**不取绝对值再平均**：`mean(|ez|)` 和 `|mean(ez)|` 差一个量级，
    而"总在低 3 cm"和"上下各 3 cm"是完全不同的物理（前者是悬停推力标定，
    后者是高度环振荡）—— 它们的归因不同（见判据表的归因树）。所以两个都报。

    ★ 所有量都是 `float`，`np.float64` 也要 `float()` 出去 —— 否则 JSON 序列化会漏。
    """
    d = np.asarray(actual, dtype=float) - np.asarray(cmd, dtype=float)
    if d.ndim != 2 or d.shape[1] < 3:
        raise ValueError(f"要 (N, >=3) 的位置序列，拿到 {d.shape}")
    exy = np.hypot(d[:, 0], d[:, 1])
    ez = d[:, 2]
    return {
        "n": int(d.shape[0]),
        "xy_mean": float(np.mean(exy)),
        "xy_max": float(np.max(exy)),
        "xy_p95": float(np.percentile(exy, 95)),
        "xy_rms": float(np.sqrt(np.mean(exy**2))),
        "z_mean": float(np.mean(ez)),
        "z_absmean": float(np.mean(np.abs(ez))),
        "z_max": float(np.max(np.abs(ez))),
    }


def path_length(pos: np.ndarray) -> float:
    """折线长度（m）。用来分辨"滞后"和"走得慢"：

    纯滞后 ⟹ 路径长度与指令的**接近相等**（只是晚到）；
    走得慢 ⟹ 同样时长里实际路径**明显更短**。
    """
    p = np.asarray(pos, dtype=float)
    if p.ndim != 2 or p.shape[0] < 2:
        return 0.0
    return float(np.sum(np.linalg.norm(np.diff(p, axis=0), axis=1)))


def best_shift(
    t_act,
    p_act,
    t_cmd,
    p_cmd,
    grid: np.ndarray,
    lo_s: float = -2.5,
    hi_s: float = 0.5,
    step_s: float = 0.02,
) -> Dict[str, object]:
    """在 `[lo_s, hi_s]` 上扫指令时间偏移，返回使**平均水平误差**最小的那一档。

    返回 `{"dt", "metrics", "curve"}`：`curve` 是 `[(dt, xy_mean), ...]`，
    ★ 一起报出来是刻意的 —— 单看最优值分不清"有一个尖峰"还是"整条曲线都是平的"。
    曲线平 ⟹ 这个窗口里根本没有可对齐的激励，`dt*` 无意义。

    ★★ 搜索范围**不覆盖整个窗口**：`dt` 一超出共同覆盖时段，`interp_columns`
    就会把端点**夹住**，于是"对齐"变成"拿静止点去凑"，误差假性下降。
    调用方要保证 `grid + lo_s` 与 `grid + hi_s` 都落在两侧序列里。
    """
    a = interp_columns(t_act, p_act, grid)
    best: Optional[Tuple[float, Dict[str, float]]] = None
    curve: List[Tuple[float, float]] = []
    for dt in np.arange(lo_s, hi_s + step_s / 2, step_s):
        c = interp_columns(t_cmd, p_cmd, grid + dt)
        m = tracking_metrics(a, c)
        curve.append((float(dt), m["xy_mean"]))
        if best is None or m["xy_mean"] < best[1]["xy_mean"]:
            best = (float(dt), m)
    assert best is not None
    return {"dt": best[0], "metrics": best[1], "curve": curve}


def segment_metrics(
    t_act,
    p_act,
    t_cmd,
    p_cmd,
    window: Tuple[float, float],
    n_seg: int = 4,
    hz: float = 50.0,
) -> List[Dict[str, float]]:
    """把窗口等分成 `n_seg` 段，各出一组指标 —— 回答"这是一个瞬态还是一个常驻偏差"。

    ★ 一段全窗的均值会**把起机瞬态和稳态混在一起**：同一架机同一条轨迹，
    只为窗口起点差 1 s，悬停那趟的读数就能差一倍（z0 实测，见 `evidence/`）。
    分段是唯一能把这两样分开看的读数，所以它对**每一段**都要有足够的窗口长度；
    段太短时（< 2 s）这里不拒绝、照给，但调用方应当心。
    """
    if n_seg < 1:
        raise ValueError("n_seg 至少为 1")
    lo, hi = window
    edges = np.linspace(lo, hi, n_seg + 1)
    out: List[Dict[str, float]] = []
    for k in range(n_seg):
        g = make_grid(float(edges[k]), float(edges[k + 1]), hz)
        a = interp_columns(t_act, p_act, g)
        c = interp_columns(t_cmd, p_cmd, g)
        m = tracking_metrics(a, c)
        out.append({"t0": float(edges[k] - lo), "t1": float(edges[k + 1] - lo), **m})
    return out


def summarize(
    t_act,
    p_act,
    t_cmd,
    p_cmd,
    window: Tuple[float, float],
    hz: float = 50.0,
    align: bool = True,
    align_lo_s: float = -2.5,
    align_hi_s: float = 0.5,
    align_step_s: float = 0.02,
) -> Dict[str, object]:
    """一个窗口的完整读数：未对齐指标 + （可选）时间对齐后的指标。

    `window` 由 `track_window` 给出。返回的字典**可以直接进 JSON**
    （`best_shift` 里那条 `curve` 会被压成 min/max，免得 JSON 里塞 150 个数）。

    ★ 窗口会**两侧各内缩 `align_hi_s .. -align_lo_s`** 吗？不会 —— 不内缩。
    未对齐那组读数必须覆盖**整个**窗口，否则 (A)/(C) 两侧内缩量不同就不可比了。
    对齐那组因为要 `grid + dt`，实际有效区间天然短一点，这个差别如实报在
    `aligned.n` 上。
    """
    grid = make_grid(window[0], window[1], hz)
    a = interp_columns(t_act, p_act, grid)
    c = interp_columns(t_cmd, p_cmd, grid)
    out: Dict[str, object] = {
        "window_s": [float(window[0]), float(window[1])],
        "window_len_s": float(window[1] - window[0]),
        "hz": float(hz),
        "unaligned": tracking_metrics(a, c),
        "path_len_actual": path_length(a),
        "path_len_cmd": path_length(c),
        "envelope_xy_actual": [float(np.ptp(a[:, 0])), float(np.ptp(a[:, 1]))],
        "envelope_xy_cmd": [float(np.ptp(c[:, 0])), float(np.ptp(c[:, 1]))],
    }
    if align:
        bs = best_shift(t_act, p_act, t_cmd, p_cmd, grid, align_lo_s, align_hi_s, align_step_s)
        curve = bs["curve"]
        out["aligned"] = {
            "dt_s": bs["dt"],
            "metrics": bs["metrics"],
            "curve_xy_mean_min": float(min(v for _, v in curve)),
            "curve_xy_mean_max": float(max(v for _, v in curve)),
            # ★★ 偏移扫到指令序列覆盖范围之外时，`interp_columns` 会**静默夹端点**，
            # 于是"对齐"退化成"拿一个静止点去凑"，误差假性下降。这个旗子就是那条闸门。
            # 它只说明 dt 搜索的**可达范围**被夹了，不说明 `dt_s` 这一档不可信 ——
            # 逐档判的话要去读 `curve`（`curve_xy_mean_min` 贴着 0 而 dt 又贴边，就是它）。
            "coverage_ok": bool(
                grid[0] + align_lo_s >= t_cmd[0] and grid[-1] + align_hi_s <= t_cmd[-1]
            ),
            # ★ 曲线平 ⟹ 这个窗口里没有"动"可对齐（例如悬停：指令是常量），
            # `dt_s` 只是噪声挑出来的一格，**不是滞后**。必须和数一起报。
            "flat": bool(
                max(v for _, v in curve) - min(v for _, v in curve) < _FLAT_M
            ),
        }
    return out


#: 对齐扫描曲线"平不平"的判据 [m] —— 起伏小于它 ⟹ 窗口里没有可对齐的激励
_FLAT_M = 0.01
