#!/usr/bin/env python3
"""量 z0 的 OCP **最坏**周期耗时 —— 这个数字决定 MPC 频率（D6）。

    export ACADOS_SOURCE_DIR=$HOME/acados
    export LD_LIBRARY_PATH=$ACADOS_SOURCE_DIR/lib:$LD_LIBRARY_PATH
    python3 -m nmpc.timing --runs 1000 --cpu 4

它回答的是 ticket 04 的验收项 (f) 与 `phase2-implementation.md` §5 的 **P3**
（"最坏求解耗时 → 回填 D6"）。判定线照抄 §3.1 D6 那一行：**最坏 ≤ 10 ms 保 100 Hz；
10–20 ms 降 50 Hz；> 20 ms 停下优化**。

⚠️ **两条前提，缺一个读数就是假的**（都是第一手，见 `docs/control/acados-orin-build.md` §6）：

1. **绑核**（`--cpu N`）：实测中位数 −29 %、最大值 −33 %。
2. **机器空载**：这台机器有并行会话，别人一编译（PX4 SITL 之类）读数就废 ——
   实测同一命令 max 从 4.4 ms 涨到 23 ms，判定行会翻成"100 Hz 不够"的**假**结论。
   本脚本自己报 `load1`，有负载时给判定行打失真标记（但不改数）。

★ 与 `tools/acados-orin/check_x500_ocp.py`（ticket 02）的分工：那个量的是**上游参考实现的
x500 占位配置**，只用来确认"acados 在本机解得动"；**本脚本量的是 z0 自己的 OCP**，
它的数是**能回填 P3** 的那一个。两者不可混用。
"""

from __future__ import annotations

import argparse
import os
import statistics
import sys
import time
from pathlib import Path

#: 判定线（照抄 §3.1 D6，改之前先去改文档）
HZ100_BUDGET_MS = 10.0
HZ50_BUDGET_MS = 20.0


def _machine_quiet() -> tuple[bool, str]:
    """1 分钟 load average 是否低于核数的一半。

    存在的理由（第一手）：2026-09-28 有一次复测撞上另一个会话在编译 PX4 SITL，
    同一命令的 max 从 4.39 ms 变成 23.4 ms、判定行直接翻成"不够"。
    """
    ncpu = os.cpu_count() or 1
    try:
        load1 = float(Path("/proc/loadavg").read_text().split()[0])
    except OSError:
        return True, "读不到 /proc/loadavg"
    return load1 < ncpu / 2, f"load1={load1:.2f} / {ncpu} 核"


def _cpu_line() -> str:
    freqs = []
    for cpu in range(os.cpu_count() or 1):
        p = Path(f"/sys/devices/system/cpu/cpu{cpu}/cpufreq/scaling_cur_freq")
        try:
            freqs.append(int(p.read_text().strip()) // 1000)
        except OSError:
            pass
    gov_p = Path("/sys/devices/system/cpu/cpu0/cpufreq/scaling_governor")
    gov = gov_p.read_text().strip() if gov_p.is_file() else "?"
    span = f"{min(freqs)}–{max(freqs)} MHz" if freqs else "读不到频率"
    return f"{os.cpu_count()} 核 @ {span}, governor={gov}"


def stat_line(name: str, samples_ms: list[float]) -> str:
    return (f"{name:<26} n={len(samples_ms):<5} min={min(samples_ms):7.3f}  "
            f"median={statistics.median(samples_ms):7.3f}  "
            f"p99={statistics.quantiles(samples_ms, n=100)[98]:7.3f}  "
            f"**max={max(samples_ms):7.3f} ms**")


def main() -> int:
    ap = argparse.ArgumentParser(description="量 z0 OCP 的最坏周期耗时（回填 P3 / D6）")
    ap.add_argument("--runs", type=int, default=1000, help="计时循环次数（报最坏值，不是平均）")
    ap.add_argument("--cpu", type=int, default=None, help="把本进程绑到一个核（降抖动，强烈建议）")
    ap.add_argument("--warmup", type=int, default=50, help="预热次数（首次调用含内存分配）")
    ap.add_argument("--offset", type=float, default=2.0,
                    help="场景：目标相对当前位置偏 +x 多少米（0 = 悬停，是退化的简单情形）")
    args = ap.parse_args()

    if not os.environ.get("ACADOS_SOURCE_DIR"):
        print("请先 export ACADOS_SOURCE_DIR=$HOME/acados", file=sys.stderr)
        return 2

    if args.cpu is not None:
        os.sched_setaffinity(0, {args.cpu})

    # ── 0. 环境自述（"这份读数是哪来的"的上下文，缺了没法复核） ──
    print("=" * 78)
    print(f"ACADOS_SOURCE_DIR : {os.environ['ACADOS_SOURCE_DIR']}")
    print(f"CPU               : {_cpu_line()}  亲和={sorted(os.sched_getaffinity(0))}")
    quiet, load_line = _machine_quiet()
    print(f"机器负载          : {load_line}{'' if quiet else '  ⚠️ 有负载，下面的耗时不可信！'}")
    print("=" * 78)

    # import 放这儿：报错信息能被上面的自述框住（缺环境变量时更易读）
    import numpy as np

    from .ocp import OfflineOcp

    t0 = time.perf_counter()
    ocp = OfflineOcp()
    print(f"OfflineOcp 创建（含代码生成 + C 编译）: {time.perf_counter() - t0:.1f} s")

    N = ocp.config.N
    x0 = ocp.hover_state()
    x_ref = np.tile(x0, (N + 1, 1))
    x_ref[:, 0] += args.offset          # 目标整体平移：非退化场景，比悬停更能代表真实周期
    u_zero = np.zeros((N, 4))
    print(f"场景: x_ref 相对 x0 偏 +x {args.offset:g} m"
          f"；整周期走缺省 u_ref（上一帧解平移），最小周期显式给 u_ref=0")
    print(f"配置: N={N} Tf={ocp.config.tf:g} s（内部步长 {ocp.dt * 1e3:g} ms）、"
          f"求解器={ocp.acados_ocp.solver_options.nlp_solver_type}+{ocp.acados_ocp.solver_options.qp_solver}")

    # ── 1. 整周期：**节点会走的路径**（prediction=True + 缺省 u_ref） ──
    for _ in range(args.warmup):
        ocp.solve(x0, x_ref)
    cycle_full: list[float] = []
    internal: list[float] = []
    for _ in range(args.runs):
        t = time.perf_counter()
        sol = ocp.solve(x0, x_ref)
        cycle_full.append((time.perf_counter() - t) * 1e3)
        internal.append(sol.info.time_tot * 1e3)
    if sol.info.degraded:
        print(f"⚠️ 本次循环里出现了退化的解：{sol.info.line()}", file=sys.stderr)

    # ── 2. 最小周期：**只要第一拍 u**（预测轨迹是给 RViz 的，可以降频取） ──
    for _ in range(args.warmup):
        ocp.solve(x0, x_ref, u_zero, prediction=False)
    cycle_min: list[float] = []
    for _ in range(args.runs):
        t = time.perf_counter()
        ocp.solve(x0, x_ref, u_zero, prediction=False)
        cycle_min.append((time.perf_counter() - t) * 1e3)

    print("-" * 78)
    print(stat_line("整周期（取预测轨迹）", cycle_full))
    print(stat_line("  ↑ acados 自报 time_tot", internal))
    print(stat_line("最小周期（只要 u[0]）", cycle_min))
    print(f"{'取轨迹 + 自算代价的增量':<26} median="
          f"{statistics.median(cycle_full) - statistics.median(cycle_min):7.3f} ms（按中位数之差）")

    # ★ 墙钟 − acados 自报 = **非求解开销**（逐点 get/读回、Python 记账、被抢占）。
    #   分开报的理由：acados 的 time_tot 只覆盖它在 C 里算的那段，**不含**被调度出去的时间。
    #   D6 问的是"能不能闭环"，所以判定看墙钟；但把两者印在一起，才能分辨
    #   "算法太慢"（内部也涨）和"被别的进程挤了"（只有墙钟涨）。
    jit = [w - i for w, i in zip(cycle_full, internal)]
    print(f"{'外层抖动（墙钟 − acados）':<26} median={statistics.median(jit):7.3f}  "
          f"**max={max(jit):7.3f} ms**  >5 ms 的次数={sum(1 for j in jit if j > 5.0)}/{len(jit)}")
    print("-" * 78)

    # ── 3. 判定行：D6。★ 必须把"这次读数可不可信"和结论印在一起，不能只印结论 ──
    worst_wall, worst_solver = max(cycle_full), max(internal)
    n_outlier = sum(1 for j in jit if j > 5.0)
    tail = "" if quiet else "（⚠️ 本次有负载，读数可能失真 —— 换空载机器重测）"
    if n_outlier:
        # 有抢占时**两个都印**：D6 的判定线（≤10 ms）说的是"能不能闭环"，那看墙钟；
        # 但"要不要优化求解器"看的是 acados 自报。混成一个数就会得出错误结论。
        print(f"最坏整周期 = {worst_wall:.3f} ms ⟹ {_band(worst_wall)}"
              f"（其中 {n_outlier}/{len(jit)} 次含非求解开销的抢占，单次最大 {max(jit):.3f} ms）")
        print(f"纯求解开销 max = {worst_solver:.3f} ms ⟹ {_band(worst_solver)}{tail}")
    else:
        print(f"最坏整周期 = {worst_wall:.3f} ms（acados 自报 max={worst_solver:.3f} ms）"
              f" ⟹ {_band(worst_wall)}{tail}")
    return 0


def _band(ms: float) -> str:
    """D6 的三档判定（线取自 §3.1 D6，改这里就要改文档）。"""
    if ms <= HZ100_BUDGET_MS:
        return f"保 100 Hz（预算 {HZ100_BUDGET_MS:g} ms）"
    if ms <= HZ50_BUDGET_MS:
        return f"降 50 Hz（{HZ100_BUDGET_MS:g}–{HZ50_BUDGET_MS:g} ms 区间）"
    return f"停下优化（> {HZ50_BUDGET_MS:g} ms）"


if __name__ == "__main__":
    raise SystemExit(main())
