#!/usr/bin/env python3
"""把 `hover_thrust_from_ulog` 在一**批**日志上跑一遍 —— 跨日志复现性与判据敏感性。

单份日志给出一个数，说明不了什么（那一份可能恰好干净）。这个脚本回答两个问题：

1. **跨日志复现**：N 份日志各自独立地给出同一个数吗？差多少？
2. **判据敏感性**（反"卡在分界上挑数据"）：把 `--v-eps` 在几档之间扫一遍，
   读数会不会跟着变？**会变就说明那个数是被判据挑出来的，不是数据里的。**

用法::

    python3 tools/hover-thrust/corpus_check.py <日志目录> [<日志目录> ...]
    python3 tools/hover-thrust/corpus_check.py <目录> --expect 0.62330 --v-eps 0.30 0.40 0.60

只读本地文件。**不碰飞控、不发任何 MAVLink。**
"""

from __future__ import annotations

import argparse
import collections
import glob
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import hover_thrust_from_ulog as H  # noqa: E402

STATUS = {
    H.EXIT_OK: "✅ 可用",
    H.EXIT_UNUSABLE: "⚠️ 有飞行、但无稳态段/不一致",
    H.EXIT_NO_DATA: "· 没有飞行段",
}


def collect(paths, v_eps, min_duration=2.0):
    """对每份日志跑一次，返回 `{文件名: (hover, code, notes)}` 与失败明细。"""
    out, missing, crashed = {}, [], []
    for p in paths:
        try:
            rep, code = H.analyse(
                p, min_duration=min_duration, mask_kwargs={"v_eps": v_eps}
            )
        except SystemExit as exc:  # 缺必需 topic —— 工具**有意**干净退出，批跑不该当崩溃
            missing.append((os.path.basename(p), str(exc).splitlines()[0]))
            continue
        except Exception as exc:  # noqa: BLE001
            crashed.append((os.path.basename(p), f"{type(exc).__name__}: {exc}"))
            continue
        out[os.path.basename(p)] = (rep.get("hover_thrust"), code, rep.get("notes", []))
    return out, missing, crashed


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="在一批 ulog 上跑 hover_thrust 反推（只读）")
    ap.add_argument("dirs", nargs="+", help="含 *.ulg 的目录（递归）")
    ap.add_argument("--expect", type=float, default=None, help="已知真值（SITL 同构机用）")
    ap.add_argument("--expect-tol", type=float, default=0.02)
    ap.add_argument("--v-eps", type=float, nargs="+", default=[0.30], help="要扫的速度门档位")
    ap.add_argument("--min-duration", type=float, default=2.0)
    args = ap.parse_args(argv)

    paths = sorted(p for d in args.dirs for p in glob.glob(os.path.join(d, "**", "*.ulg"), recursive=True))
    if not paths:
        raise SystemExit(f"这些目录里没有 *.ulg：{args.dirs}")
    print(f"日志目录 : {args.dirs}")
    print(f"日志份数 : {len(paths)}")
    print(f"判据     : min_duration={args.min_duration:g} s，速度门扫 {args.v_eps}")

    runs = {}
    for veps in args.v_eps:
        res, missing, crashed = collect(paths, veps, args.min_duration)
        runs[veps] = res
        cnt = collections.Counter(STATUS.get(c, f"? code {c}") for _, c, _ in res.values())
        cnt["缺必需 topic（干净退出）"] = len(missing)
        cnt["崩溃"] = len(crashed)
        vals = np.array([h for h, c, _ in res.values() if c == H.EXIT_OK], dtype=float)
        tentative = np.array([h for h, c, _ in res.values() if h is not None], dtype=float)

        print(f"\n=== v_eps = {veps:g} m/s")
        for k, v in cnt.most_common():
            print(f"  {k:28s} {v}")
        if crash := crashed:
            print("  崩溃明细：")
            for n, e in crash[:10]:
                print(f"    {n}: {e}")
        if vals.size == 0:
            print("  没有可用读数")
            continue

        print(
            f"  可用读数 {vals.size} 份（另有 {tentative.size - vals.size} 份有试读值但被判不可用）\n"
            f"    范围 {vals.min():.5f} – {vals.max():.5f}   "
            f"极差 {(vals.max() - vals.min()) / np.median(vals):.4%}   "
            f"中位 {np.median(vals):.5f}"
        )
        for s in sorted({n for _, _, ns in runs[veps].values() for n in ns})[:3]:
            print(f"    ⚠️ {s[:100]}")
        if args.expect is not None:
            worst = float(np.max(np.abs(vals - args.expect)) / args.expect)
            ok = worst <= args.expect_tol
            print(
                f"  {'✅' if ok else '❌'} 与已知值 {args.expect:.5f} 的最大偏差 {worst:.4%}"
                f"（判据 ≤ {args.expect_tol:.2%}）"
            )

    # 敏感性：读数会不会跟着判据走
    if len(args.v_eps) > 1:
        print("\n=== 判据敏感性（同一个数会不会跟着判据变）")
        seen = collections.Counter(
            round(h, 6)
            for res in runs.values()
            for h, _c, _n in res.values()
            if h is not None
        )
        print("  所有档位、所有日志出现过的读数（去重）：")
        for val, n in seen.most_common():
            print(f"    {val:.5f}   {n} 次")
        if len(seen) == 1:
            print("  ⟹ 只有一个数：读数**不**随判据移动，它不是被判据挑出来的")
        else:
            print("  ⚠️ 出现了不止一个数 —— 读数依赖判据 ⟹ 上表里那个「主口径」站不住，要查")
        print("  逐对比较（两份都给出试读值的日志）：")
        for i, va in enumerate(args.v_eps):
            for vb in args.v_eps[i + 1 :]:
                both = [n for n in runs[va] if runs[va][n][0] is not None and runs[vb][n][0] is not None]
                diff = [n for n in both if abs(runs[va][n][0] - runs[vb][n][0]) > 1e-9]
                print(f"    v_eps {va:g} vs {vb:g}：共同 {len(both):3d} 份，读数不同 {len(diff)} 份")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
