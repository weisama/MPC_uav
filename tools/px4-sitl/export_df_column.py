#!/usr/bin/env python3
"""把 DUT 逐帧日志里的垂直偏置列（`df`）抽成归档用的 JSON（ticket 20 §13.4 的底料）。

## 为什么需要它

ticket 20 的 §13.4 要论证"偏置**落在哪**"（`u_ref` 旧落点 vs 发出去的指令 = 候选 A），
唯一的判据是**发布出去的力与 `u[0] + ΔF` 对得上**。而：

- `df` / `u_ref0` 两列**只存在于 DUT 自己的 `-dut.jsonl`**；
- 逐帧 jsonl（48 MB）沿 `F0–F4` 的惯例**未入库**，且 `/tmp` 过不了重启 ⟹ **会永久丢**；
- 归档的 `<tag>-summary.json` 里**没有这两列** ⟹ 光看归档，§13.4 的每一句都**复核不了**。

⟹ 本脚本把需要的四列抽出来落进 `evidence/tier-b/df-column-<日期>.json`，让那张表可复核。

## 发布点的闭合式（本脚本同时算残差，用来核这件事）

    -thrust_body_z * f_max  ==  clip(u[0]_k + df_{k-1}, 0, f_max) * f_max / t_max

两个因子都不是凑的，各自有出处：

1. **`df` 差一拍** —— 日志里 `thrust_body_z` 带的是**上一帧**的 `ΔF`。
   （用同帧的 `df_k` 去算，残差立刻从 1e-7 涨到 7e-4 N 量级。）
2. **`f_max` vs `t_max` 的归一化差** —— 发布那一路按 `t_max = m·g/hover_thrust` 归一化
   （SITL 口径下 = 11.8003208 N），而 `u[0]` 是 `f_max = 11.8 N` 口径
   ⟹ 恒定的相对因子 `f_max/t_max - 1 = -2.7183e-5`（实测两场一致到 4 位有效数字）。

## 用法

    python3 tools/px4-sitl/export_df_column.py --tags F0c F1c --out <路径>

`--tags` 里每一场都要有 `/tmp/tier-b/<tag>-dut.jsonl`（跑完 `run_matrix.sh` 当场抽；
**过了重启就没了**，所以这一步必须与飞行同一次开机内做完）。

★ 这一条是本仓库的一条老账：见 `.scratch/phase2/issues/20-...md` 的 Comments
（"要能复核的数字，当场就落进仓库"）。
"""

from __future__ import annotations

import argparse
import json
import pathlib
import statistics as st

F_MAX = 11.8  # 节点发出去那一路的**上限**口径（N，按电池的 `f_max`；与重搭无关）
# ⚠️ **重搭前**的 B 档口径（旧被控对象 `T_MAX_TOTAL = 11.8 N` ⟹ 解析悬停 0.6235）。
#    被控对象已于 2026-10-09 按真机实测重搭（落地记录见 ticket 21 / phase3 的 `04`）⟹ 本常数
#    仅为**回放旧归档**而保留；跑新基线前必须同步改（或直接改用节点默认值）。
HOVER_THRUST = 0.6235
MASS = 0.75  # SIH 标称质量（kg）


def t_max() -> float:
    """发布那一路的归一化分母（N）。与 `f_max` 相差 2.7e-5，是那条恒定残差的出处。"""
    return t_hover() / HOVER_THRUST


def t_hover() -> float:
    """悬停所需总推力 `m·g`（N）= `ΔF` 抗饱和下界的出处（`[-t_hover, f_max - t_hover]`）。"""
    return MASS * 9.81


def load_normal(path: pathlib.Path) -> list[dict]:
    """读 `-dut.jsonl`，只留 `fallback == "normal"` 的帧（`warmup` 几帧不进统计）。"""
    rows = [json.loads(line) for line in path.read_text().splitlines() if line.strip()]
    return rows, [r for r in rows if r.get("fallback") == "normal"]


def export_one(tag: str, dut: pathlib.Path) -> dict:
    rows, normal = load_normal(dut)
    if not normal:
        raise SystemExit(f"{tag}: {dut} 里没有 fallback == normal 的帧 —— 路径/口径对不上？")
    d = [r["df"] for r in normal]
    u0 = [r["u"][0] for r in normal]
    tz = [r["thrust_body_z"] for r in normal]
    cmd = [-x * F_MAX for x in tz]  # 换回 N
    ur0 = [r["u_ref0"][0] for r in normal]
    # 闭合式残差（差一拍 + f_max/t_max 归一化差）；对照：用同帧 df 的那一版
    res = [abs(cmd[k] - (u0[k] + d[k - 1]) * (F_MAX / t_max())) for k in range(1, len(normal))]
    res_same = [abs(cmd[k] - (u0[k] + d[k]) * (F_MAX / t_max())) for k in range(1, len(normal))]
    eps = [(cmd[k] - (u0[k] + d[k - 1])) / (u0[k] + d[k - 1]) for k in range(1, len(normal))]
    return {
        "fallback_dist": {k: sum(1 for r in rows if r.get("fallback") == k) for k in sorted({str(r.get("fallback")) for r in rows})},
        "n_normal": len(normal),
        "frames": [[r["tick"], r["df"], r["u"][0], r["thrust_body_z"]] for r in normal],
        "df_final": d[-1],
        "df_mean_last800": st.fmean(d[-800:]),
        "df_minmax": [min(d), max(d)],
        "df_clamp": [-t_hover(), F_MAX - t_hover()],
        "u_ref0_0": {"distinct": len(set(ur0)), "min": min(ur0), "max": max(ur0),
                     "first": ur0[0], "last": ur0[-1]},
        "identity_residual_closed_form": {"median": st.median(res), "max": max(res)},
        "identity_residual_df_same_frame": {"median": st.median(res_same)},
        "identity_epsilon_measured": st.median(eps),
    }


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--tags", nargs="+", default=["F0b", "F1b", "F0c", "F1c"])
    ap.add_argument("--dut-dir", default="/tmp/tier-b", help="逐帧日志所在目录（跑完 run_matrix.sh 当场抽）")
    ap.add_argument("--out", required=True)
    args = ap.parse_args()

    out: dict = {
        "_这是": "ticket 20 §13.4 的逐帧底料 —— 由 tools/px4-sitl/export_df_column.py 生成，别手改",
        "_为什么要有它": "`df`/`u_ref0` 两列只在 DUT 的 `-dut.jsonl` 里，而归档的 `-summary.json` 不含它们 "
                     "⟹ 没有这份，§13.4 的每一句都复核不了（逐帧 jsonl 未入库且 /tmp 过不了重启）。",
        "_逐帧数组": "[tick, df, u[0], thrust_body_z]，只含 fallback=normal 的帧",
        "_发布点闭合式": "-thrust_body_z*f_max == clip(u[0]_k + df_{k-1}, 0, f_max) * f_max/t_max；"
                    "差一拍见 mpc_node.py 的日志路径，f_max/t_max 见 t_max()",
        "t_max_N": t_max(),
        "f_max_N": F_MAX,
        "epsilon_predicted": F_MAX / t_max() - 1,
    }
    for tag in args.tags:
        out[tag] = export_one(tag, pathlib.Path(args.dut_dir) / f"{tag}-dut.jsonl")

    p = pathlib.Path(args.out)
    p.write_text(json.dumps(out, ensure_ascii=False, indent=1) + "\n")
    for tag in args.tags:
        o = out[tag]
        print(f"{tag}: normal {o['n_normal']} 帧 | ΔF 末 {o['df_final']:+.4f} | "
              f"u_ref0[0] 去重 {o['u_ref0_0']['distinct']} | "
              f"闭合式残差 中位 {o['identity_residual_closed_form']['median']:.2e} "
              f"（同帧 df 版 {o['identity_residual_df_same_frame']['median']:.2e}）")
    print(f"⟹ {p}")


if __name__ == "__main__":
    main()
