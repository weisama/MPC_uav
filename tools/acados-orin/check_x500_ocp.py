#!/usr/bin/env python3
"""ticket 02 的验收脚本：acados 在本机**真解得动**（不只是编译过了）。

用的是**参考实现自带的例子**（`DISCOWER/px4-mpc` 的 `MultirotorRateModel()` —— x500 配置：
`mass = 1 kg`、`hover_thrust = 0.73`），且**不是抄一份过来跑**：脚本从仓库外的只读副本
`~/refs/px4-mpc` 按钉住的 commit `git archive` 出一份干净源码到 scratch 目录再 import，
这样「跑的是不是那个 commit 的代码」是可验证的，也不会往只读副本里写生成代码。

用法：
    export ACADOS_SOURCE_DIR=$HOME/acados
    export LD_LIBRARY_PATH=$ACADOS_SOURCE_DIR/lib:$LD_LIBRARY_PATH
    python3 tools/acados-orin/check_x500_ocp.py --runs 1000

量出来的数回填 `docs/control/phase2-implementation.md` §5 的 P3（最坏求解耗时）。
"""

from __future__ import annotations

import argparse
import os
import statistics
import subprocess
import sys
import time
from pathlib import Path

# ★ 钉住的上游 commit —— 与 docs/reference/refs-manifest.md §1 记的同一行
PX4_MPC_COMMIT = "3e541afb05350e30a0867420a5df40dbdfa92c13"
PX4_MPC_REF = Path.home() / "refs" / "px4-mpc"


def sh(cmd: list[str]) -> str:
    return subprocess.run(cmd, capture_output=True, text=True, check=True).stdout.strip()


def export_reference(scratch: Path) -> Path:
    """把钉住 commit 的参考实现导出一份到 scratch（幂等：同一 commit 不重复解包）。"""
    head = sh(["git", "-C", str(PX4_MPC_REF), "rev-parse", "HEAD"])
    if head != PX4_MPC_COMMIT:
        print(f"⚠️  {PX4_MPC_REF} 现在在 {head}，不是钉住的 {PX4_MPC_COMMIT}", file=sys.stderr)
    repo = scratch / "repo"
    marker = repo / ".exported_commit"
    if marker.is_file() and marker.read_text().strip() == PX4_MPC_COMMIT:
        return repo
    print(f"导出 {PX4_MPC_REF} @ {PX4_MPC_COMMIT} → {repo}")
    repo.mkdir(parents=True, exist_ok=True)
    tar = subprocess.run(
        ["git", "-C", str(PX4_MPC_REF), "archive", "--format=tar", PX4_MPC_COMMIT],
        capture_output=True, check=True,
    ).stdout
    subprocess.run(["tar", "-x", "-C", str(repo)], input=tar, check=True)
    marker.write_text(PX4_MPC_COMMIT + "\n")
    return repo


def stat_line(name: str, samples_ms: list[float]) -> str:
    worst = max(samples_ms)
    return (
        f"{name:<22} n={len(samples_ms):<5} "
        f"min={min(samples_ms):7.3f}  median={statistics.median(samples_ms):7.3f}  "
        f"p99={statistics.quantiles(samples_ms, n=100)[98]:7.3f}  **max={worst:7.3f} ms**"
    )


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--runs", type=int, default=1000, help="纯求解循环次数（报最坏耗时）")
    ap.add_argument("--node-runs", type=int, default=30,
                    help="整条 controller.solve() 的次数 —— 参考实现每周期都在重建 CasADi 模型")
    ap.add_argument("--scratch", type=Path, default=Path.home() / "acados-ocp-check")
    ap.add_argument("--cpu", type=int, default=None, help="把本进程绑到某个核（降抖动）")
    args = ap.parse_args()

    acados_dir = os.environ.get("ACADOS_SOURCE_DIR")
    if not acados_dir:
        print("请先 export ACADOS_SOURCE_DIR=$HOME/acados", file=sys.stderr)
        return 2
    acados_dir = Path(acados_dir)

    if args.cpu is not None:
        os.sched_setaffinity(0, {args.cpu})

    # ── 0. 环境自述（这些是「这份读数是哪来的」的上下文，缺了没法复核）
    print("=" * 78)
    print(f"ACADOS_SOURCE_DIR     : {acados_dir}")
    print(f"LD_LIBRARY_PATH[0:40] : {os.environ.get('LD_LIBRARY_PATH', '')[:40]}...")
    print(f"BLASFEO_TARGET (构建时): {_blasfeo_target(acados_dir)}")
    print(f"OpenMP 已编入 libacados: {_has_openmp(acados_dir)}")
    tera = acados_dir / "bin" / "t_renderer"
    print(f"t_renderer            : {sh([str(tera), '--version']) if tera.is_file() else '缺失'}")
    print(f"CPU                    : {_cpu_line()}  亲和={sorted(os.sched_getaffinity(0))}")
    print("=" * 78)

    # ── 1. 参考实现（钉住 commit，导出到 scratch，不动只读副本）
    repo = export_reference(args.scratch)
    sys.path.insert(0, str(repo / "px4_mpc"))
    from px4_mpc.models.multirotor_rate_model import MultirotorRateModel          # noqa: E402
    from px4_mpc.controllers.multirotor_rate_mpc import MultirotorRateMPC        # noqa: E402

    # ── 2. 确认这确实是「1 kg / 悬停油门 0.73 的 x500 配置」
    model = MultirotorRateModel()
    expected_max_thrust = 1.0 * 9.81 / 0.73
    assert model.mass == 1.0, f"mass = {model.mass}，不是 x500 的 1 kg"
    assert abs(model.max_thrust - expected_max_thrust) < 1e-9, "max_thrust 与 0.73 不自洽"
    print(f"模型: mass={model.mass} kg, max_thrust={model.max_thrust:.4f} N ⟸ hover_thrust=0.73")

    # ── 3. 建 AcadosOcpSolver（触发代码生成 + 生成代码的 C 编译）
    t0 = time.perf_counter()
    controller = MultirotorRateMPC(model)
    build_s = time.perf_counter() - t0
    print(f"AcadosOcpSolver 创建成功（含代码生成与编译）: {build_s:.1f} s")

    solver = controller.ocp_solver
    x0 = controller.x0

    # ── 4. solve() 必须返回 0（参考实现的 solve() 在非 0 时 raise，这里直接读状态码）
    solver.set(0, "lbx", x0)
    solver.set(0, "ubx", x0)
    status = solver.solve()
    print(f"首次 solve() → status = {status}   time_tot = {solver.get_stats('time_tot') * 1e3:.3f} ms")
    if status != 0:
        print(f"❌ 状态码非 0：{status}", file=sys.stderr)
        return 1
    simU, simX = controller.solve(x0)
    print(f"controller.solve() 正常返回：simU{simU.shape} simX{simX.shape}，"
          f"首帧 u=[F={simU[0,0]:.3f} N, ω={simU[0,1:4]}]")

    # ── 5. 纯求解耗时：报**最坏**，不是平均
    for _ in range(50):  # 预热（第一次调用含内存分配）
        solver.solve()
    samples: list[float] = []
    internal: list[float] = []
    for _ in range(args.runs):
        t = time.perf_counter()
        solver.solve()
        samples.append((time.perf_counter() - t) * 1e3)
        internal.append(solver.get_stats("time_tot") * 1e3)

    # ── 6. 整条 controller.solve() 的耗时（参考实现每周期重建 CasADi 模型）
    node_samples: list[float] = []
    for _ in range(args.node_runs):
        t = time.perf_counter()
        controller.solve(x0)
        node_samples.append((time.perf_counter() - t) * 1e3)

    print("-" * 78)
    print(stat_line("纯 ocp_solver.solve()", samples))
    print(stat_line("  ↑ acados 自报 time_tot", internal))
    print(stat_line("controller.solve(x0)", node_samples))
    print("-" * 78)
    print(f"最坏纯求解耗时 = {max(samples):.3f} ms ⟹ "
          f"100 Hz 预算 10 ms {'够' if max(samples) < 10 else '不够'}")
    return 0


def _has_openmp(acados_dir: Path) -> bool:
    out = subprocess.run(["nm", "-D", str(acados_dir / "lib" / "libacados.so")],
                         capture_output=True, text=True).stdout
    return "omp_get_thread_num" in out


def _blasfeo_target(acados_dir: Path) -> str:
    """构建时的 BLASFEO 目标只在 cmake cache 里留痕（install 目录不带）。

    约定：构建目录 = <安装目录>-build（tools/acados-orin/build_acados.sh 的默认值）。
    """
    cache = Path(f"{acados_dir}-build") / "CMakeCache.txt"
    try:
        for line in cache.read_text().splitlines():
            if line.startswith("BLASFEO_TARGET:"):
                return f"{line.split('=', 1)[1]}  (来自 {cache})"
    except OSError:
        pass
    return f"未知（构建目录 {cache} 不在）"


def _cpu_line() -> str:
    freqs = []
    for cpu in range(os.cpu_count() or 1):
        p = Path(f"/sys/devices/system/cpu/cpu{cpu}/cpufreq/scaling_cur_freq")
        try:
            freqs.append(int(p.read_text().strip()) // 1000)
        except OSError:
            pass
    gov = Path("/sys/devices/system/cpu/cpu0/cpufreq/scaling_governor")
    gov_s = gov.read_text().strip() if gov.is_file() else "?"
    return f"{os.cpu_count()} 核 @ {freqs} MHz, governor={gov_s}"


if __name__ == "__main__":
    sys.exit(main())
