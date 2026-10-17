#!/usr/bin/env python3
"""从**原始 DUT JSONL** 抽取 `solver-summary.json`（票 02 第 3 步）。

**为什么单独抽**：Tier B 的 `summary.json` 只投影 `FRAME_KEYS` 那几列
（`tools/px4-sitl/verify_tier_b_seam3.py:161-164`），`solve_ms` / `solve_ok` /
`degraded` / `sat` **都不在里面** —— 而那几列只在 DUT 自己那份逐帧 JSONL 里
（原始 JSONL **不入库**，8 MB 级）。所以每场跑完、删原始日志之前，先把它们压成一张小表。

口径（与 `verify_tier_a_hover.solve_stats` 对齐，另加票面要的 mean/p95）：

- `solve_ms` 只统计**真的解过**的拍（键存在且是数）；`prestream` 档不解 ⟹ 不在内。
- `mean` / `p50` / `p95` / `max` 都对排序后的数组取；`p95` 用**最近秩**
  （`arr[ceil(0.95·n)−1]`），与 `p99` 在 `solve_stats` 里的取法同族。
- `degraded` 是真值计数（`SolveInfo.degraded`），`solve_ok` 为假 = 求解失败。
- 饱和：`sat` 是**名字列表**（`node_core.saturated_bounds`：`F_min`/`F_max`/`wx`/`wy`/`wz`），
  按名字计**拍数**；`frames_with_sat` 是"至少顶一条"的拍数。
- `warmup` 档单列一套读数：D6 定在**稳态**（去掉 acados 冷启动）上；
  `warmup.first_solved_ms` 取**第一个真的解过**的拍（首帧是 `prestream`，不解）。
- 坏行（JSON 解析失败）计入 `bad_lines` 并在 stdout 报出；一个可解析帧都没有则直接报错
  —— 静默跳过会让 `n_frames` 悄悄偏小。

用法：

    python3 tools/report/evidence/sitl/extract_solver_summary.py \
        --jsonl /tmp/phase5-sitl/R-square/R-square-dut.jsonl \
        --tag R-square \
        --out tools/report/evidence/sitl/R-square/solver-summary.json
"""

from __future__ import annotations

import argparse
import collections
import json
import math
from pathlib import Path


def _pct(arr: list[float], q: float) -> float:
    """最近秩分位数（`q` 取 0–1）；空数组由调用方先挡掉。"""
    if not arr:
        return float("nan")
    k = min(len(arr) - 1, max(0, math.ceil(q * len(arr)) - 1))
    return arr[k]


def _block(vals: list[float]) -> dict:
    if not vals:
        return {"n": 0}
    arr = sorted(vals)
    return {"n": len(arr),
            "mean": round(sum(arr) / len(arr), 3),
            "p50": round(_pct(arr, 0.50), 3),
            "p95": round(_pct(arr, 0.95), 3),
            "max": round(arr[-1], 3),
            "min": round(arr[0], 3)}


def summarise(rows: list[dict], tag: str, bad_lines: int = 0) -> dict:
    solved = [r for r in rows if isinstance(r.get("solve_ms"), (int, float))]
    normal = [r for r in solved if r.get("fallback") != "warmup"]
    warm = [r for r in rows if r.get("fallback") == "warmup"]
    sat_counts = collections.Counter()
    frames_with_sat = 0
    for r in rows:
        sat = r.get("sat") or []
        if sat:
            frames_with_sat += 1
            for name in sat:
                sat_counts[str(name)] += 1
    return {
        "tag": tag,
        "source": "原始 DUT JSONL（未入库；脚本见同目录 extract_solver_summary.py）",
        "n_frames": len(rows),
        "bad_lines": bad_lines,
        "n_solved": len(solved),
        "fallback_dist": dict(collections.Counter(str(r.get("fallback")) for r in rows)),
        "solve_ms_all_solved": _block([float(r["solve_ms"]) for r in solved]),
        "solve_ms_steady": _block([float(r["solve_ms"]) for r in normal]),
        "warmup": {"n": len(warm),
                   # ★ 别取 `rows[0]`：首帧属于 `prestream`，**根本不解**（`solve_ms` 为空）。
                   #   这一列要的是"第一次真的解"的耗时，即第一个带 `solve_ms` 的拍。
                   "first_solved_ms": (round(float(solved[0]["solve_ms"]), 3)
                                       if solved else None)},
        "degraded_frames": sum(1 for r in rows if r.get("degraded") is True),
        "solve_failed_frames": sum(1 for r in rows if r.get("solve_ok") is False),
        "solve_ok_missing_frames": sum(1 for r in rows if r.get("solve_ok") is None),
        "saturation": {"frames_with_sat": frames_with_sat,
                       "by_name": dict(sorted(sat_counts.items()))},
        "notes": [
            "solve_ms 只统计真的解过的拍（prestream 档不解、不在内）",
            "p95 用最近秩 arr[ceil(0.95*n)-1]",
            "solve_ms_steady = 去掉 fallback=='warmup' 的拍（D6 的口径）",
            "sat 为空列表 = 一拍都没顶到约束（node_core.saturated_bounds 的生产阈值）",
        ],
    }


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--jsonl", required=True)
    ap.add_argument("--tag", required=True)
    ap.add_argument("--out", required=True)
    args = ap.parse_args()
    rows, bad = [], 0
    for line in Path(args.jsonl).read_text(encoding="utf-8").splitlines():
        if not line.strip():
            continue
        try:
            rows.append(json.loads(line))
        except json.JSONDecodeError:
            bad += 1                      # ★ 坏行要计数：静默跳过会让 n_frames 悄悄偏小
    if not rows:
        raise SystemExit(f"{args.jsonl} 里没有可解析的帧（坏行 {bad}）")
    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(summarise(rows, args.tag, bad), indent=1, ensure_ascii=False),
                   encoding="utf-8")
    print(f"{args.tag}: {out}（{out.stat().st_size} B，{len(rows)} 帧原始，坏行 {bad}）")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
