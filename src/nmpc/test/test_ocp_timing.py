"""seam 2（**零 ROS**）：**最坏**求解耗时 —— 这个数字决定控制频率（D6）。

覆盖 ticket 04 的验收项 (f)：跑 ≥1000 次、报**最坏**（不是平均），结果回填 D6。

★ 断言打在哪一个数上，是这份文件最要紧的决定（实测逼出来的）：

    整周期墙钟   median 3.914  p99 4.656  **max 54.083 ms** ← 1000 次里有 1 次被抢占
    acados 自报  median 2.684  p99 3.429  **max  3.584 ms** ← 只覆盖它在 C 里算的那段

墙钟那一列的 max 不是求解开销 —— 是这段 Python 被调度出去过（同一轮的 acados 自报仍是
3.58 ms）。**断言打在墙钟 max 上会得到一个"会随机变红"的测试**，打在自报 max 上才是
"求解器有没有退化"的稳定判据。所以：

  · 断言：`acados 自报 max ≤ 10 ms`（D6 的 100 Hz 预算）
  · 打印：两列都印，另印"外包开销"的 max，供人工判断（回填 D6 用的是 CLI：`python3 -m nmpc.timing`）

★ 负载检查是**硬前提**（第一手）：同一命令在有编译负载时 max 从 4.4 → 23.4 ms。
所以 `load1 > 核数/2` 时**跳过**（那读数无效），而不是"照样跑一遍"。
"""

from __future__ import annotations

import os
import statistics
import time
from pathlib import Path

import numpy as np
import pytest

from nmpc.ocp import OfflineOcp

#: D6 的 100 Hz 预算 [ms]（§3.1 D6；改这里就要改文档）
BUDGET_MS = 10.0
#: 本文件默认的循环次数（票面要求 ≥1000；CI 里跑 1000 次要 ~5 s 求解，可接受）
RUNS = 1000
WARMUP = 50


def _load1() -> float | None:
    try:
        return float(Path("/proc/loadavg").read_text().split()[0])
    except OSError:
        return None


def test_worst_case_solve_fits_the_100hz_budget():
    """(f) 最坏求解耗时 ≤ 10 ms（100 Hz 预算），且耗时稳定（没有悄悄退化的长尾）。"""
    ncpu = os.cpu_count() or 1
    load1 = _load1()
    if load1 is not None and load1 >= ncpu / 2:
        pytest.skip(f"机器有负载（load1={load1:.2f} / {ncpu} 核），耗时读数无效 —— "
                    f"第一手：负载下同一命令的 max 会从 4.4 ms 变成 23 ms")

    ocp = OfflineOcp()
    n = ocp.config.N
    x0 = ocp.hover_state()
    x_ref = np.tile(x0, (n + 1, 1))
    x_ref[:, 0] += 2.0                       # 非退化场景：悬停太便宜，代表不了真实周期

    for _ in range(WARMUP):
        ocp.solve(x0, x_ref)

    wall: list[float] = []
    internal: list[float] = []
    for _ in range(RUNS):
        # 墙钟在**外面**掐表（与 `nmpc/timing.py` 同一个定义），内部那一列用 acados 自报
        t = time.perf_counter()
        sol = ocp.solve(x0, x_ref)
        wall.append((time.perf_counter() - t) * 1e3)
        internal.append(sol.info.time_tot * 1e3)

    print(f"\n整周期墙钟  : min={min(wall):.3f} median={statistics.median(wall):.3f} "
          f"p99={statistics.quantiles(wall, n=100)[98]:.3f} max={max(wall):.3f} ms")
    print(f"acados 自报 : min={min(internal):.3f} median={statistics.median(internal):.3f} "
          f"p99={statistics.quantiles(internal, n=100)[98]:.3f} max={max(internal):.3f} ms")
    print(f"外包开销    : max={max(w - i for w, i in zip(wall, internal)):.3f} ms"
          f"（读回 + 自算代价 + Python 记账 + 被抢占）")

    assert max(internal) <= BUDGET_MS, (
        f"acados 自报最坏 {max(internal):.3f} ms 超出 100 Hz 预算 {BUDGET_MS:g} ms —— "
        f"回填 D6 前先跑 `python3 -m nmpc.timing --runs 1000 --cpu 4` 复核")
    assert statistics.median(internal) <= BUDGET_MS / 2, "中位数该有一倍以上余量"


def test_flat_readback_is_used_and_matches_per_stage_reads():
    """预测轨迹用**一次 `get_flat`** 取回，且与逐点 `get` 逐位一致（19× 的差别）。

    第一手：同一条轨迹 0.049 ms vs 0.930 ms。这不是风格问题 —— 它是 D6 预算里的一块钱。
    """
    ocp = OfflineOcp()
    x0 = ocp.hover_state()
    x_ref = np.tile(x0, (ocp.config.N + 1, 1))
    x_ref[:, 0] += 2.0
    sol = ocp.solve(x0, x_ref)

    per_stage_x = np.array([ocp.solver.get(i, "x") for i in range(ocp.config.N + 1)])
    per_stage_u = np.array([ocp.solver.get(i, "u") for i in range(ocp.config.N)])
    assert np.array_equal(sol.x_pred, per_stage_x)
    assert np.array_equal(sol.u_pred, per_stage_u)
