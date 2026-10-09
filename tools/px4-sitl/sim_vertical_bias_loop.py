#!/usr/bin/env python3
"""垂直通道的**离线闭环**复算 —— 换落点之前先把"能不能过"算出来（ticket 20，2026-09-29）

**为什么要有它**：ticket 20 的裁定 1 把垂直积分作用的落点选在 `u_ref`，飞出来是**负结果**
（判据① 对 `mass+30` 从 −0.6406 恶化到 −0.7200 m），根因已由
[`probe_u_ref_authority.py`](probe_u_ref_authority.py) 量到：那条通路增益是**反号**的
（`d u[0]/d u_ref[0] = −0.0616 N/N` ⟹ 正反馈）。换落点是新裁定，而**机器窗口稀缺** ——
本工具把那件事先在离线算完：把被控对象接进环路，逐拍跑"节点每拍做的事 + 简单的对象动力学"。

**三种落点并排**（这也是本工具的自检结构）：

| `--layout` | 积分量加到哪 | 对应 |
|---|---|---|
| `none` | 不加（`ΔF ≡ 0`） | 11 号票的 F1（加积分器之前） |
| `u_ref` | `u_ref[:,0] = t_hover + ΔF` | **已飞过的 F1b**（裁定 1 的落点） |
| `command` | 解完之后 `u[0] += ΔF`，再夹 `[0, f_max]` | **待裁定的候选 A** |

★ **它的可信度靠"复现两场已飞的读数"来自证**：`none` 该落在 −0.64 m 附近、`u_ref` 该落在
  −0.72 m 附近（F1 / F1b 的真值帧判读）。**复现不上就不许拿它预测第三种**。
  ⚠️ 判据① 是拿**真值帧**判的（含动捕噪声/时延、起降暂态、姿态耦合），本工具这些**都没有** ——
  所以自证只要求"落点量级与相对恶化对上"，不要求小数点后两位（差多少要如实写出来）。

**本工具的口径（缺口都列在这，别外推）**：
- 只算**垂直一维**：`z'' = (F − m_p·g)/m_p`。姿态恒为水平、无横滚/俯仰耦合、无电机/电池动态
  （SIH 那套 `THR_MDL_FAC = 0` 的线性推力曲线也没进来）、无风。
- **测量 = 真值**：不加动捕噪声、不加时延、不走来帧调度。⟹ 它**不能**回答"判据①·振荡"那条，
  也**不能**替代 B 档 SITL（那才是判据① 的验收台）。
- 参考取**恒定 1.0 m**（与那一场的 `x_ref0` 同量级），初始 `z = 1.0`、`v = 0`。
- `x0` 的姿态四元数用 **F1b 那一场实测的那个**（动捕落在 `−q` 分支），其余通道为 0。

用法（**先 source 工作空间**）：

    source /opt/ros/humble/setup.bash && source install/setup.bash
    python3 tools/px4-sitl/sim_vertical_bias_loop.py                      # 三种落点并排
    python3 tools/px4-sitl/sim_vertical_bias_loop.py --layout command --ki 2.0 --seconds 40
"""

import argparse
import sys
import numpy as np

#: 模型口径（B 档 SITL，**重搭前**）：与 `evidence/tier-b/F*-summary.json` 的 `model_hover_thrust` 同源。
#: ⚠️ 被控对象已于 2026-10-09 按真机实测重搭（落地记录见 ticket 21 / phase3 的 `04`）⟹ 本常数
#:    **不再是现行口径**，仅为回放那批旧归档而保留；现行 B 档用节点默认值（`0.1589`）。
HOVER_THRUST_SITL = 0.6235
#: 被控对象：`mass+30` 的 SIH 机型（`z0-mass+30.param` ⟹ 见 `F1b-summary.json` 的 `plant_hover`）
PLANT_MASS = 0.975
G = 9.81
#: F1b 那一场逐帧日志末拍的四元数（取它只为"与飞的那一场同形"，它不是本工具的自由参数）
FLIGHT_QUAT = np.array([-0.716799, 0.000272, -0.000772, -0.697279])
#: 参考姿态：与 x0 同一个姿态，只是不取负分支（`hold_reference` 用的是这个）
REF_QUAT = np.array([0.707107, 0.0, 0.0, 0.707107])


def run(layout: str, ki: float, seconds: float, z_ref: float, rate_hz: float,
        codegen_dir: str, verbose: bool):
    from nmpc.params import Z0Params
    from nmpc.ocp import OfflineOcp

    params = Z0Params(hover_thrust=HOVER_THRUST_SITL)
    # ★ codegen 目录**只按 layout 分**，不按 ki 分：模型/参数与 ki 无关 ⟹ 一个 layout 只编一次
    ocp = OfflineOcp(params=params, codegen_dir=f"{codegen_dir}/{layout}")
    N = ocp.config.N
    dt = 1.0 / rate_hz
    n = int(seconds * rate_hz)

    x_ref = np.tile(np.concatenate([[0.0, 0.0, z_ref], np.zeros(3), REF_QUAT]), (N + 1, 1))
    # —— 被控对象（垂直一维）——
    z, vz = z_ref, 0.0
    # —— 积分器状态与夹取（与 `mpc_node._integrate_bias_z` 同界）——
    lo, hi = -params.t_hover, params.f_max - params.t_hover
    df = 0.0
    hist = []
    for k in range(n):
        # ☠️ `vz` **必须**喂实测值：OCP 的速度项就是环路里唯一的阻尼（实测 `D = 2.66 N/(m/s)`）。
        #   2026-09-29 本工具第一版这里写成 `np.zeros(3)` ⟹ 仿真里环路**无阻尼**，
        #   于是 `m ë + G e = δ` 成了一个周期 3.3 s 的无阻尼振子，把"振荡/发散"全造出来了。
        x0 = np.concatenate([[0.0, 0.0, z], [0.0, 0.0, vz], FLIGHT_QUAT])
        # ① 这一拍喂给求解器的 u_ref（`layout=command` 时**不**含 ΔF —— 那正是新落点的定义）
        u_cmd_ref = params.t_hover + (df if layout == "u_ref" else 0.0)
        u_ref = np.tile(np.array([u_cmd_ref, 0.0, 0.0, 0.0]), (N, 1))
        sol = ocp.solve(x0, x_ref, u_ref=u_ref, prediction=False)
        # ② 发出去的推力：`command` 落点在这里加 ΔF，然后夹（与 OCP 的 ubu 同界）
        f_out = float(sol.u[0]) + (df if layout == "command" else 0.0)
        f_out = float(np.clip(f_out, 0.0, params.f_max))
        # ③ 判完再积分（节点就是这个顺序：只有"正常帧"才积）。本工具没有兜底帧 ⟹ 每拍都积。
        # ⚠️ `layout=none` = **不加积分作用**（ΔF 恒 0，不是"积了但不落点"）—— 那是 11 号票的 F1。
        if ki > 0 and layout != "none":
            df = float(np.clip(df + ki * (z_ref - z) * dt, lo, hi))
        # ④ 对象：垂直一维
        acc = (f_out - PLANT_MASS * G) / PLANT_MASS
        vz += acc * dt
        z += vz * dt
        hist.append((k * dt, z, vz, f_out, df))
    h = np.array(hist)
    tail = h[h[:, 0] >= seconds - 8.0]        # 与判据① 同口径的"末 8 s"
    err = tail[:, 1] - z_ref
    return {
        "layout": layout, "ki": ki, "dt": dt, "n": n,
        "z_ref": z_ref,
        "mean_err_last8": float(np.mean(err)),
        "max_abs_err_last8": float(np.max(np.abs(err))),
        #: ★ **峰峰值** —— 自检必须比这一列：只比均值会放过"仿真在振、而飞行没振"这种错
        #:   （2026-09-29 第一版就是这么放过去的：均值对上了、峰峰值差了 27 倍）。
        "ptp_last8": float(np.max(err) - np.min(err)),
        "df_final": float(h[-1, 4]), "df_min": float(h[:, 4].min()), "df_max": float(h[:, 4].max()),
        "f_final": float(h[-1, 3]),
        "plant_hover_force": PLANT_MASS * G,
        "df_clamp": [lo, hi],
        "hist": h[::max(1, n // 40)].tolist(),
    }


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0],
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--layout", default="all", choices=("all", "none", "u_ref", "command"))
    ap.add_argument("--ki", default="1.0",
                    help="积分增益 [N/(m·s)]（与 `ki_z` 同义）；**逗号分隔可扫多个** ⟹ 量可行域")
    ap.add_argument("--seconds", type=float, default=25.0)
    ap.add_argument("--z-ref", type=float, default=1.0)
    ap.add_argument("--rate-hz", type=float, default=100.0)
    ap.add_argument("--codegen-dir", default="/tmp/sim-vertical-bias")
    ap.add_argument("--verbose", action="store_true")
    args = ap.parse_args(argv)

    layouts = ("none", "u_ref", "command") if args.layout == "all" else (args.layout,)
    kis = [float(v) for v in str(args.ki).split(",") if v.strip()]
    print(f"# 对象 mass={PLANT_MASS} kg ⟹ 悬停推力 {PLANT_MASS * G:.5f} N；"
          f"模型 t_hover = {0.75 * G:.5f} N ⟹ 口径差 {PLANT_MASS * G - 0.75 * G:+.5f} N")
    print(f"# 参考 z = {args.z_ref} m，起步 z = {args.z_ref} m、v = 0；"
          f"速率 {args.rate_hz} Hz、{args.seconds} s；ki = {kis}")
    print(f"# ⚠️ 无动捕噪声/时延、无姿态耦合、对象只有垂直一维 ⟹ 不能替代 B 档 SITL\n")
    rows = []
    for lay in layouts:
        for ki in kis:
            r = run(lay, ki, args.seconds, args.z_ref, args.rate_hz, args.codegen_dir, args.verbose)
            rows.append(r)
            print(f"[{lay:8s} ki={ki:<6g}] 末 8 s 垂直误差 均值 {r['mean_err_last8']:+.4f} m"
                  f"（绝对值最大 {r['max_abs_err_last8']:.4f}、**峰峰值 {r['ptp_last8']:.4f}**）"
                  f"  ΔF 末值 {r['df_final']:+.4f} N"
                  f"（区间 [{r['df_min']:+.3f}, {r['df_max']:+.3f}]，夹取界 {r['df_clamp'][0]:+.4f}…{r['df_clamp'][1]:+.4f}）"
                  f"  F 末值 {r['f_final']:.5f} N")
    print()
    got = {(r["layout"], r["ki"]): r for r in rows}
    #: 已飞读数（真值帧判读）：垂直均值 / 末段高度峰峰值。**两列都要比** ——
    #: 只比均值会放过"仿真在振、飞行没振"（2026-09-29 栽过：均值像、峰峰值差 27 倍）。
    FLOWN = {"none": (-0.6406, 0.0483), "u_ref": (-0.7200, 0.0543)}
    for lay in ("none", "u_ref"):
        r = got.get((lay, kis[0]))
        if r is not None:
            m_ref, p_ref = FLOWN[lay]
            print(f"⟹ 自检 [{lay}]：均值 {r['mean_err_last8']:+.4f} m 对已飞 {m_ref:+.4f}（差 "
                  f"{r['mean_err_last8'] - m_ref:+.4f}）；**峰峰值 {r['ptp_last8']:.4f} m 对已飞 {p_ref:.4f}**"
                  f"（差 {r['ptp_last8'] - p_ref:+.4f}）"
                  f"{'  ⚠️ 峰峰值差一个量级 ⟹ 仿真与飞行不同形，别拿它预测' if r['ptp_last8'] > 5 * p_ref else ''}")
    if "command" in layouts:
        print("⟹ 候选 A（`command` 落点）的可行域（两条都要过：均值 < 0.15 m **且** 峰峰值 < 0.35 m）：")
        for r in rows:
            if r["layout"] == "command":
                m, ptp = r["mean_err_last8"], r["ptp_last8"]
                ok = abs(m) < 0.15 and ptp < 0.35
                print(f"     ki = {r['ki']:<6g} ⟹ 均值 {m:+.4f} m、峰峰值 {ptp:.4f} m  "
                      f"{'✅ 过线' if ok else '❌ 不过线'}"
                      f"  (ΔF 末值 {r['df_final']:+.4f} N、峰值 {r['df_max']:+.3f} N"
                      f"{'、**顶夹**' if abs(r['df_max'] - r['df_clamp'][1]) < 1e-6 else ''})")
    return 0


if __name__ == "__main__":
    sys.exit(main())
