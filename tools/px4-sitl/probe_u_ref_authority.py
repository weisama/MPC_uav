#!/usr/bin/env python3
"""`u_ref` 对"发出去的推力"到底有多大权威？（ticket 20 的诊断工具，2026-09-29）

**问题从哪来**：ticket 20 裁定"垂直通道的积分作用做成**外层前馈偏置**"，落点选的是
`u_ref`（`u_ref[:,0] = t_hover + ΔF`，OCP 的**代价参考**）。B 档 SITL 复跑的结果是
**判据① 仍不达标、而且比加偏置前更差**（同机型 `mass+30`：−0.6406 m → −0.7200 m），
`ΔF` 全程顶在夹取上限 4.4425 N。本工具回答的是"为什么"里最要紧的那一半：

    把 ΔF 加到 `u_ref` 上，**求解器解出来的 `u[0]` 到底往哪边走、走多少**？

**做法**：拿一场飞行的**逐帧日志**里某一拍的 `x0`（10 维全状态）与那一场的参考
（`x_ref0` / `x_ref0_q`），造若干个**全新**的 `OfflineOcp`（每个各有一份自己的热启动历史），
每个跑 `--settle` 拍 `solve(..., prediction=False)`。热启动会自己收敛到一个不动点，
比较各档的不动点即得权威。

⚠️ **本工具钉的是那个"已被证伪的落点"（ΔF ⟹ `u_ref`），这是故意的** —— 它存在的全部理由
  就是给 F1b 那场负结果找机理。**节点现在的 `_tick` 不是这个顺序**（2026-09-29 裁定 4 之后：
  `u_ref` 是常量，偏置落在 `_normal_frame` 里 `u_cmd[0] = clip(u[0] + ΔF, 0, f_max)`，
  中间那步是 `_clear_bias_if_disabled()`）。所以本工具**不要**当成"节点在做什么"的文档读，
  它是"那个坏落点做了什么"的记录；换落点之后的离线闭环复算在
  `tools/px4-sitl/sim_vertical_bias_loop.py`。

★ **为什么"钉住 x0 反复解"能代表飞行**：节点的求解器是 `SQP_RTI`（**每拍只迭代一次**）
  且**不清内部迭代量** ⟹ 每拍是从上一拍的解热启动的一次线性化-QP。稳态下动捕量到的 `x0`
  近似不变，所以飞行里那条迭代几乎就是"同一个 x0 反复解"。**本工具能自证同源**：
  取 F1b 末拍的状态 + 那一场实际的 `u_ref[0]=11.8`，解出的 `u[0]` 与日志里那一场的
  `u[0]` **逐位相同**（9.56038，`ω` 两路也到小数点后 5 位）。

❌ **它不是什么**：不是闭环仿真 —— 被控对象没接进来，`x0` 是钉死的。它量的是
  "偏置对**求解器解出的 u**的权威"，**不是**"偏置对**高度**的权威"。两者的桥是环路里
  另外那条通路（位置误差 ⟹ 推力），本工具把它一起量出来（见输出里的 `d u / d e_z`）。

用法（**先 source 工作空间**，与其它 tools 一致）：

    source /opt/ros/humble/setup.bash && source install/setup.bash
    python3 tools/px4-sitl/probe_u_ref_authority.py --frames /tmp/tier-b/F1b-dut.jsonl
    python3 tools/px4-sitl/probe_u_ref_authority.py --frames F1b-dut.jsonl --last-frame 0 \
        --u-ref 5,7.3575,9,9.5648,10.5,11.8

⚠️ 本工具**只读**日志、只算不飞（几秒），但会**代码生成 + 编译 C**到 `--codegen-dir`
  （默认 `/tmp/probe-u-ref-codegen`，与节点用的那份**分开**，不污染别人的 cache）。
"""

import argparse
import json
import sys
from pathlib import Path

import numpy as np

#: B 档 SITL 的模型口径（**重搭前**）：见 `evidence/tier-b/F1b-summary.json` 的 `model_hover_thrust`。
#: ⚠️ 被控对象已于 2026-10-09 按真机实测重搭（落地记录见 ticket 21 / phase3 的 `04`）⟹ 本常数
#:    **不再是现行口径**，仅为回放那批旧归档而保留；现行 B 档用节点默认值（`0.1589`）。
HOVER_THRUST_SITL = 0.6235


def read_frame(path: Path, back: int = 1) -> dict:
    """从逐帧 jsonl 里取倒数第 `back` 行（`back=1` ⟹ 末行）。只保留 10 维状态与参考。"""
    lines = [ln for ln in path.read_text().splitlines() if ln.strip()]
    if not lines:
        raise SystemExit(f"{path} 是空的")
    d = json.loads(lines[-back])
    need = ("x0", "x_ref0", "x_ref0_q", "u", "u_ref0", "df")
    miss = [k for k in need if k not in d]
    if miss:
        raise SystemExit(f"{path} 倒数第 {back} 行缺字段 {miss}（这份日志是 ticket 20 之后的节点写的吗？）")
    return d


def fixed_point(ocp, x0, x_ref, u_ref, settle: int):
    """钉住 `x0` 反复解，返回收敛后的解。热启动在 `ocp` 内部，调用方每档造新实例。"""
    for _ in range(settle):
        sol = ocp.solve(x0, x_ref, u_ref=u_ref, prediction=False)
    return sol


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0],
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--frames", required=True, help="逐帧日志（`*-dut.jsonl`），从这里取状态与参考")
    ap.add_argument("--last-frame", type=int, default=1,
                    help="取倒数第几行（1 = 末行；>1 用来避开末端暂态）")
    ap.add_argument("--hover-thrust", type=float, default=HOVER_THRUST_SITL)
    ap.add_argument("--u-ref", default=None,
                    help="要扫的 u_ref[0] 列表（逗号分隔；默认 = t_hover 加一串口径差）")
    ap.add_argument("--settle", type=int, default=300, help="每档解几拍（达到不动点用）")
    ap.add_argument("--codegen-dir", default="/tmp/probe-u-ref-codegen")
    ap.add_argument("--no-position-error", action="store_true",
                    help="对照：把参考 z 换成当前 z（消掉位置误差那条通路）")
    ap.add_argument("--json", default=None, help="结果写到哪（不写就只打印）")
    args = ap.parse_args(argv)

    from nmpc.params import Z0Params          # 延迟 import：让 `--help` 不依赖 acados
    from nmpc.ocp import OfflineOcp

    fr = read_frame(Path(args.frames), args.last_frame)
    x0 = np.asarray(fr["x0"], float)
    z_ref = float(x0[2]) if args.no_position_error else float(fr["x_ref0"][2])
    x_ref0 = np.asarray(fr["x_ref0"], float).copy()
    x_ref0[2] = z_ref
    params = Z0Params(hover_thrust=args.hover_thrust)

    if args.u_ref:
        u_list = [float(v) for v in args.u_ref.split(",")]
    else:
        u_list = [5.0, params.t_hover, 9.0, 0.975 * 9.81, 10.5, params.f_max]

    print(f"# 状态来自 {args.frames} 倒数第 {args.last_frame} 行"
          f"（tick={fr.get('tick')} df={fr.get('df')} u_ref0[0]={fr['u_ref0'][0]}）")
    print(f"# 模型口径 mass={params.mass} hover_thrust={params.hover_thrust} "
          f"t_hover={params.t_hover:.4f} N t_max={params.t_max:.4f} N f_max={params.f_max} N")
    print(f"# x0 = {np.round(x0, 6).tolist()}")
    print(f"# 参考 z = {z_ref:.6f} m ⟹ 位置误差 e_z = {z_ref - x0[2]:+.4f} m"
          f"{'（对照：无位置误差）' if args.no_position_error else ''}")
    print()

    rows = []
    for U in u_list:
        ocp = OfflineOcp(params=params, codegen_dir=f"{args.codegen_dir}/{U:.4f}-{z_ref:.4f}")
        N = ocp.config.N
        x_ref = np.tile(np.concatenate([x_ref0, np.zeros(3), np.asarray(fr["x_ref0_q"], float)]),
                        (N + 1, 1))
        u_ref = np.tile(np.array([U, 0.0, 0.0, 0.0]), (N, 1))
        sol = fixed_point(ocp, x0, x_ref, u_ref, args.settle)
        rows.append({"u_ref0": U, "u0": float(sol.u[0]), "u": [float(v) for v in sol.u]})
        print(f"  u_ref[0] = {U:8.4f}  ⟹  u[0] = {sol.u[0]:.5f} N"
              f"   (Δ vs t_hover = {sol.u[0] - params.t_hover:+.5f})")

    us = np.array([r["u_ref0"] for r in rows])
    ys = np.array([r["u0"] for r in rows])
    slope, icept = np.polyfit(us, ys, 1)
    resid = float(np.abs(np.polyval([slope, icept], us) - ys).max())
    print()
    print(f"⟹ 斜率 d u[0] / d u_ref[0] = {slope:+.5f} N/N（线性残差 max {resid:.2e} N）")
    if slope < 0:
        print("   ⚠️ **符号是反的**：抬高 `u_ref` 会让发出去的推力**变小** ⟹ 把位置误差"
              "积分进 `u_ref` 是一条**正反馈**（越沉 ⟹ ΔF 越大 ⟹ 推力越小 ⟹ 更沉）。")
    else:
        print("   符号是正的 ⟹ 积分进 `u_ref` 是同号通路，但增益只有 "
              f"{slope:.4f}（真正管事的是位置误差那条通路）。")
    print(f"# 同一拍上位置误差通路的增益（差商用）：见下方 json 的 `du_d_ez`")
    du_dez = (ys[us == params.t_hover][0] - params.t_hover) / (z_ref - x0[2]) \
        if (z_ref - x0[2]) != 0 and np.any(us == params.t_hover) else None
    if du_dez is not None:
        print(f"⟹ d u[0] / d e_z = {du_dez:+.5f} N/m")

    if args.json:
        Path(args.json).write_text(json.dumps({
            "frames": args.frames, "last_frame": args.last_frame,
            "model": {"mass": params.mass, "hover_thrust": params.hover_thrust,
                      "t_hover": params.t_hover, "t_max": params.t_max, "f_max": params.f_max},
            "x0": x0.tolist(), "z_ref": z_ref, "e_z": z_ref - x0[2],
            "rows": rows, "du_d_u_ref": float(slope), "fit_resid_max": resid,
            "du_d_ez": (float(du_dez) if du_dez is not None else None),
            "settle": args.settle,
        }, ensure_ascii=False, indent=1), encoding="utf-8")
        print(f"# 写到 {args.json}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
