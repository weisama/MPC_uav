"""冻结状态下的 `SQP_RTI` 迭代收敛扫描 —— ticket 16 裁定 `R_yaw` 的第一手证据。

**这是什么**：把节点的**每拍行为**单独拿出来、把测量与参考**钉死**，于是"上一帧解当 warm start"
这件事就退化成一个**不动点迭代** `u_{k+1} = RTI(x0, x_ref, u_k)`。这正是台架 A 档黑盒
（`test_mpc_node.py` 的假 PX4 状态静止）跑的那个东西，离线重放已与 DUT 自己 JSONL **逐拍对上**。

**为什么要有它**：ticket 16 一度把 `R_yaw` 从 `5e2` 降到 `50`（"方案 E"），收回的理由是
"10 m 不可行步进下指令逐拍乱跳"。判"不收敛"有个陷阱 —— **探针截断太早会把慢收敛读成极限环**
（本工具初版就吃了这一发：60 拍看到 `|Δu| = 0.888`，写下"没有不动点"；跑到 400 拍是 `2.6e-4`）。
所以本工具一律**按拍记录 `|Δu|` 的整条衰减曲线**，并据此判 `极限环 / 慢收敛 / 已收敛`。

**两张表**（都由 `main()` 打印，都可 `--out` / `--sweep-out` 落 JSON）：

1. **偏移 × `R_yaw` 那张**（`scan`）—— 钉"哪个权重在哪个工况有极限环"；
2. **`R_yaw` 单轴那张**（`sweep_r_yaw`）—— 钉 10 m 不可行步进下**收敛判据落在哪两档之间**
   （`ocp.py` 里"判据在 50 与 55 之间"那句话的出处）。★ 加第二张表是因为第一张只有
   `{50, 500}` 两个值，而结论引用了中间那几档 —— **引用了就得落地**（/tmp 过不了重启）。

跑法：

    source install/setup.bash
    export ACADOS_SOURCE_DIR=$HOME/acados LD_LIBRARY_PATH=$ACADOS_SOURCE_DIR/lib:$LD_LIBRARY_PATH
    python3 tools/rti-convergence/rti_convergence.py \
        --out       tools/rti-convergence/evidence/scan.json \
        --sweep-out tools/rti-convergence/evidence/r_yaw_sweep.json

⚠️ 本工具**只描述**冻结状态下的迭代，不证明实机行为 —— 真机上 `x0` 每拍都在变。
"""

from __future__ import annotations

import argparse
import json
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Sequence, Tuple

import numpy as np

REPO = Path(__file__).resolve().parents[2]
if str(REPO / "src" / "nmpc") not in sys.path:
    sys.path.insert(0, str(REPO / "src" / "nmpc"))

from nmpc import Z0Params  # noqa: E402
from nmpc.node_core import (  # noqa: E402
    ControlConfig,
    hold_feedforward,
    hold_reference,
    pose_to_state,
    px4_quat_to_enu,
)

#: 默认拍数。★ **别调小**：判"收敛"只在跑够之后才成立（见模块 docstring 的那个坑）。
DEFAULT_TICKS = 400

#: `[F, ωx, ωy, ωz]` 的分量名 —— 读数与断言都按名字走，避免"看错列"。
U_NAMES = ("F", "w_x", "w_y", "w_z")

#: `R_yaw` 单轴扫描的档位。★ 密在 50…100 那一段：判据的边界在 `50` 与 `55` 之间，
#: 而 `500` 是落地值、`50` 是已撤回的"方案 E"，两端都要留在表里当锚点。
R_YAW_SWEEP = (50.0, 55.0, 60.0, 65.0, 70.0, 80.0, 90.0, 100.0, 150.0, 200.0, 300.0, 500.0)


@dataclass(frozen=True)
class Weights:
    """代价权重（只列本工具扫的那几项；其余照 `OcpConfig` 的默认）。"""

    qw: float = 0.1
    qatt: float = 2.0
    r_xy: float = 5e2
    r_yaw: float = 5e2

    def q_diag(self) -> Tuple[float, ...]:
        return (1e1,) * 6 + (self.qw, self.qatt, self.qatt, self.qatt)

    def r_diag(self) -> Tuple[float, ...]:
        return (1e1, self.r_xy, self.r_xy, self.r_yaw)


def iterate(offset: Sequence[float] = (10.0, 0.0, 0.0), *,
            weights: Weights = Weights(), ticks: int = DEFAULT_TICKS,
            codegen_dir: str | None = None) -> np.ndarray:
    """钉死状态与参考，连解 `ticks` 次，返回 `(ticks, 4)` 的 `u` 序列。

    ★ **每次调用都新建 `OfflineOcp`**：共用一个实例会让上一条场景的迭代点污染下一条
    （本工具第一版就吃过这个 —— 换 `hold_offset` 方向时读出了假的 `ωx = +2.03`）。
    """
    from nmpc.ocp import OcpConfig, OfflineOcp

    cfg = OcpConfig(q_diag=weights.q_diag(), r_diag=weights.r_diag())
    ocp = OfflineOcp(config=cfg, codegen_dir=codegen_dir)

    x0 = pose_to_state((0.0, 0.0, 0.0), px4_quat_to_enu((1.0, 0.0, 0.0, 0.0)))
    x_ref = hold_reference(x0, ControlConfig(hold_offset=tuple(offset)), cfg.N)
    u_ref = hold_feedforward(Z0Params(), cfg.N)

    return np.array([
        ocp.solve(x0, x_ref, u_ref=u_ref).u.copy() for _ in range(ticks)
    ])


def step_sizes(us: np.ndarray) -> np.ndarray:
    """`d[k] = |u_{k+1} − u_k|_∞` —— 相邻两拍指令的最大分量变化，长度 `ticks − 1`。"""
    return np.abs(np.diff(us, axis=0)).max(axis=1)


def classify(us: np.ndarray, *, settled_tol: float = 1e-6,
             window: int = 10) -> Dict[str, object]:
    """按 `|Δu|` 的**整条曲线**判：极限环 / 慢收敛 / 已收敛。

    判据刻意保持粗糙 —— 它要能分辨这三类，不是精确标定：
    - **已收敛**：末拍 `|Δu| < settled_tol`（RTI 到不动点后逐拍变化恰为 0）；
    - **极限环**：末段（后 `window` 拍）与中段（`window` 拍之前那一段）**都不衰减**；
    - **慢收敛**：末拍还在动，但比中段小了一个量级以上。
    """
    d = step_sizes(us)
    mid = float(d[d.size // 2 - window // 2: d.size // 2 + window // 2].max())
    end = float(d[-window:].max())
    if end < settled_tol:
        verdict = "已收敛"
    elif end > 0.9 * mid:
        verdict = "极限环"
    elif end < 0.2 * mid:
        verdict = "慢收敛"
    else:
        verdict = "未定"
    return {
        "mid": mid,
        "end": end,
        "u_end": {n: float(us[-1, i]) for i, n in enumerate(U_NAMES)},
        "verdict": verdict,
    }


def _row(off_m: float, r_yaw: float, us: np.ndarray, ticks: int) -> dict:
    """一行读数（两张表共用同一套列）。"""
    return {
        "offset_m": off_m,
        "r_yaw": r_yaw,
        "ticks": ticks,
        **classify(us),
        "tail": [float(x) for x in step_sizes(us)[-5:]],
    }


def scan(offsets: Sequence[float], r_yaws: Sequence[float], *,
         ticks: int = DEFAULT_TICKS) -> List[dict]:
    rows: List[dict] = []
    for off_m in offsets:
        for r_yaw in r_yaws:
            us = iterate((off_m, 0.0, 0.0), weights=Weights(r_yaw=r_yaw), ticks=ticks)
            rows.append(_row(off_m, r_yaw, us, ticks))
    return rows


def sweep_r_yaw(r_yaws: Sequence[float] = R_YAW_SWEEP, *,
                offset: Sequence[float] = (10.0, 0.0, 0.0),
                ticks: int = DEFAULT_TICKS) -> List[dict]:
    """固定一条不可行步进（缺省 10 m），只扫 `R_yaw` —— 收敛判据的**边界**就在这条上。"""
    return [
        _row(offset[0], r_yaw, iterate(offset, weights=Weights(r_yaw=r_yaw), ticks=ticks), ticks)
        for r_yaw in r_yaws
    ]


def _print(rows: Sequence[dict]) -> None:
    print(f"{'偏移':>7} {'R_yaw':>7} {'|Δu|(中段)':>13} {'|Δu|(末段)':>13}  "
          f"{'末拍 w_y':>10} {'末拍 w_z':>10}  判定")
    for r in rows:
        print(f"{r['offset_m']:6.1f}m {r['r_yaw']:7.0f} {r['mid']:13.5f} {r['end']:13.5f}  "
              f"{r['u_end']['w_y']:+10.5f} {r['u_end']['w_z']:+10.5f}  {r['verdict']}")


def main(argv: Sequence[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--ticks", type=int, default=DEFAULT_TICKS)
    ap.add_argument("--out", type=Path, default=None, help="把「偏移 × R_yaw」那张表写成 JSON")
    ap.add_argument("--sweep-out", type=Path, default=None,
                    help="把「R_yaw 单轴」那张表写成 JSON")
    args = ap.parse_args(argv)

    offsets = (2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 10.0, 12.0)
    r_yaws = (5e2, 50.0)          # 落地值 vs 已撤回的"方案 E"

    print(f"每档 {args.ticks} 拍；|Δu| = 相邻两拍指令的最大分量变化")

    # ★ 每张表**只算一次**（一次 `scan` 要真解 9×2×400 次 OCP），打印与落盘共用同一份结果。
    print("\n【表 1】偏移 × R_yaw")
    table = scan(offsets, r_yaws, ticks=args.ticks)
    _print(table)

    print("\n【表 2】R_yaw 单轴 @ 10 m 不可行步进")
    sweep = sweep_r_yaw(ticks=args.ticks)
    _print(sweep)

    for path, rows in ((args.out, table), (args.sweep_out, sweep)):
        if path:
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(json.dumps(rows, indent=2, ensure_ascii=False) + "\n",
                            encoding="utf-8")
            print(f"\n写好了：{path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
