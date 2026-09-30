#!/usr/bin/env python3
"""`yaw_zeroed` 错配的**离线闭环**量级 —— 给 ticket 24 的裁定供数（2026-09-30）

**要量的是什么**：`mocap` 节点在**阶段 B（解锁之后）**只把**姿态**左乘了 `Rz(−yaw0)`
（[`mocap_rel_pose.hpp:134`](../../src/mocap/include/mocap/mocap_rel_pose.hpp) 的 `yaw_zeroed`），
而**位置与速度保持世界系不动**（`relative_position` 只做平移）。进 `nmpc` 的 10 状态
`x = [p, v, q]` 里于是 `p`/`v` 在世界系、`q` 在世界系转过 `−yaw0` 的那个系。而模型
（[`model.py:180-182`](../../src/nmpc/nmpc/model.py)：`v̇ = R(q)·[0,0,F]/m + g`）假定 `q` 与
`p`/`v` 同系。票面把这件事叫「两路进 MPC（模型预测控制）的量不在同一个旋转系」，**未在实飞里量过**
（ticket 24 §3 自己标了"推导"），本工具就是路线 4「先量量级」那一步。

**机制（一句话）**：机体系两边是同一个 ⟹ 控制器解出来的**机体角速率指令是对的向量**，
错的是它把「转多少机体」映射成「世界系里朝哪使劲」这一步 —— 实际的水平推力方向比它以为的
**差了 `yaw0`**（竖直分量不受影响，`Rz·ẑ = ẑ`）。带回环之后：

- **位置环吸收得了它**：不动点在"水平 + 推力沿 z"那一支，`Rz` 不动竖直 ⟹ 稳态**位置**误差不被这一条改变；
- **机头吸收不了**：控制器把**它的估计** `q' = Rz(−yaw0)q` 驱到参考，于是**真机机头**
  停在「参考机头 + `yaw0`」上 —— 这是**稳态**偏差，不是暂态；
- **阻尼按 `cos(yaw0)` 缩水**：反馈里插了一个 `Rz(yaw0)` ⟹ 误差动力学是
  `ë = Rz(yaw0)·(−K_p e − K_d ė)`，`K_p ≈ kI` 时等效阻尼 ∝ `cos(yaw0)`，
  **90° 附近退化成纯振子**。

**这个模型可信到哪一步（缺口都列在这，别外推）**：

- 被控对象用的是**控制器自己的那个模型**（`Z0MultirotorRateModel.step_rk4`）。
  ⚠️ 本仓库有一处**否决**过"与控制器同构的轻量被控模型"（[11 号票](evidence/tier-b/README.md)
  "一个被考虑并否决的方案"），理由是那样**抓不到"模型与 PX4 不一致"**。本题**不吃那一刀**：
  被研究的错配**不在动力学里**，而在「量测的旋转系」这一层 —— 本工具把**真姿态**留在真世界系、
  只把**喂给控制器的姿态**转掉，错配是**显式注入的参数**，不是模型自带的。
  ⚠️ **但它仍然证不了"真机上一定这么动"** —— 真机还有电机/电池动态、`THR_MDL_FAC` 曲线、
  动捕噪声与 50 ms 延迟、姿态环带宽。**它给的是量级与符号**，不是验收读数。
- 姿态环**不假**：角速率指令被当成瞬时兑现（`q̇ = ½·skew(ω)⊗q`），没有 PX4 那个内环的带宽与时延。
  ⟹ 本工具的"振荡"是**外环**的振荡，真机上会与内环耦合。
- 只跑**位置保持**一个工况（静止锚点参考 + 悬停前馈），不跑轨迹。
- 无噪声、无时延、无风、无求解失败兜底。

**用法**（**先 source 工作空间**）：

    source /opt/ros/humble/setup.bash && source install/setup.bash
    export ACADOS_SOURCE_DIR=$HOME/acados LD_LIBRARY_PATH=$ACADOS_SOURCE_DIR/lib:$LD_LIBRARY_PATH
    python3 tools/px4-sitl/sim_yaw_zero_loop.py                    # 扫 0/5/15/30/60/90°
    python3 tools/px4-sitl/sim_yaw_zero_loop.py --yaw0 0 90 --seconds 20
"""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

REPO = Path(__file__).resolve().parents[2]
if str(REPO / "src" / "nmpc") not in sys.path:
    sys.path.insert(0, str(REPO / "src" / "nmpc"))

from nmpc.model import normalize_quat, quat_mul  # noqa: E402
from nmpc.node_core import (  # noqa: E402
    TAU_DEFAULT_S,
    ControlConfig,
    extrapolate_state,
    hold_feedforward,
    hold_reference,
    pose_to_state,
)

#: 默认扫的档位（0 是自检档：这一档必须与「没错配」逐位一致）
YAW0_DEGS: Tuple[float, ...] = (0.0, 5.0, 15.0, 30.0, 60.0, 90.0)

#: 注入的权威出处 —— 改这里的话先去核那份源码
YAW_ZEROED_SOURCE = "src/mocap/include/mocap/mocap_rel_pose.hpp:134 `yaw_zeroed`"


# ───────────────────────── 四元数小工具（wxyz） ─────────────────────────
#
# ★ 乘/归一化/`yaw_quat` **一律从 `nmpc.model` 取**，不在本文件重写 —— 这个仓库里
#   "同一套约定抄两份"已经被点过名（ticket 21 的 `plant_hover_thrust` 同名不同物）。
#   本文件只补 numpy 侧那三个 model 没有的：旋转矩阵、欧拉角、角度归一。


def quat_from_yaw(yaw: float) -> np.ndarray:
    """绕世界 z 的纯 yaw 四元数（ENU 从 +x 逆时针）。"""
    return np.array([math.cos(yaw / 2.0), 0.0, 0.0, math.sin(yaw / 2.0)], dtype=float)


def quat_from_euler(rpy: Sequence[float]) -> np.ndarray:
    """ZYX 分解 `R = Rz(yaw)·Ry(pitch)·Rx(roll)` —— 与 `yaw_zeroed` 的注释同一套约定。"""
    roll, pitch, yaw = (float(v) for v in rpy)
    return normalize_quat(quat_mul(quat_mul(quat_from_yaw(yaw), _quat_from_pitch(pitch)),
                                   _quat_from_roll(roll)))


def _quat_from_roll(a: float) -> np.ndarray:
    return np.array([math.cos(a / 2.0), math.sin(a / 2.0), 0.0, 0.0], dtype=float)


def _quat_from_pitch(a: float) -> np.ndarray:
    return np.array([math.cos(a / 2.0), 0.0, math.sin(a / 2.0), 0.0], dtype=float)


def quat_to_rot_mat(q: Sequence[float]) -> np.ndarray:
    """`[w,x,y,z]` → 3×3 旋转矩阵（把机体系矢量转到世界系）。"""
    w, x, y, z = (float(v) for v in normalize_quat(np.asarray(q, dtype=float)))
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])


def rot_mat_to_quat(m: np.ndarray) -> np.ndarray:
    """3×3 旋转矩阵 → `[w,x,y,z]`（取 `w ≥ 0` 的那一支）。"""
    m = np.asarray(m, dtype=float)
    tr = float(np.trace(m))
    if tr > 0.0:
        s = math.sqrt(tr + 1.0) * 2.0
        q = np.array([0.25 * s, (m[2, 1] - m[1, 2]) / s, (m[0, 2] - m[2, 0]) / s,
                      (m[1, 0] - m[0, 1]) / s])
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = math.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2.0
        q = np.array([(m[2, 1] - m[1, 2]) / s, 0.25 * s, (m[0, 1] + m[1, 0]) / s,
                      (m[0, 2] + m[2, 0]) / s])
    elif m[1, 1] > m[2, 2]:
        s = math.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2.0
        q = np.array([(m[0, 2] - m[2, 0]) / s, (m[0, 1] + m[1, 0]) / s, 0.25 * s,
                      (m[1, 2] + m[2, 1]) / s])
    else:
        s = math.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2.0
        q = np.array([(m[1, 0] - m[0, 1]) / s, (m[0, 2] + m[2, 0]) / s,
                      (m[1, 2] + m[2, 1]) / s, 0.25 * s])
    return normalize_quat(q)


def euler_from_quat(q: Sequence[float]) -> Tuple[float, float, float]:
    """`[w,x,y,z]` → `(roll, pitch, yaw)`（ZYX）。"""
    w, x, y, z = (float(v) for v in normalize_quat(np.asarray(q, dtype=float)))
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2.0 * (w * y - z * x))))
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return roll, pitch, yaw


def wrap_deg(a: float) -> float:
    """把角度归到 `[−180, 180)` —— 注意是**左闭右开**：`180` 会落到 `−180`。"""
    return (a + 180.0) % 360.0 - 180.0


def yaw_zero(q_true: Sequence[float], yaw0_rad: float) -> np.ndarray:
    """★ **注入的那一步** —— `q' = Rz(−yaw0) ⊗ q`，与 C++ 的 `yaw_zeroed` 逐字同构。

    出处：`src/mocap/include/mocap/mocap_rel_pose.hpp:134`。
    位置与速度**不过**这一步（`relative_position` 只做平移）—— 正是错配的来源，别在这里"顺手也修"。
    """
    return normalize_quat(quat_mul(quat_from_yaw(-float(yaw0_rad)),
                                   normalize_quat(np.asarray(q_true, dtype=float))))


def _rz(angle: float) -> np.ndarray:
    """绕世界 z 的 3×3 旋转矩阵（`angle` 为**左乘**要用的那个角）。"""
    c, s = math.cos(angle), math.sin(angle)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


#: 两条**对照臂** —— 只有 `mismatch` 是现实，`consistent` 是路线 2 那个"位置也一起转"的修法。
#: ★★ 它存在的唯一理由是**证伪**：若 `consistent` 下退化**照样发生**，那说明我这条环路自身有问题，
#:    与那个 `Rz` 无关 —— 没有这一臂，"45° 发散"就可以是我把环路写错的结果。
MODES = ("mismatch", "consistent")


# ───────────────────────── 装置 ─────────────────────────


class Rig:
    """一个 `OfflineOcp` + 它的模型（**同一个实例** —— 别建两份，codegen 会做两遍）。"""

    def __init__(self, hover_thrust: Optional[float] = None,
                 codegen_dir: Optional[str] = None, verbose: bool = False):
        from nmpc.ocp import OfflineOcp
        from nmpc.params import Z0Params

        self.params = Z0Params() if hover_thrust is None else Z0Params(hover_thrust=hover_thrust)
        self.ocp = OfflineOcp(params=self.params, codegen_dir=codegen_dir, verbose=verbose)
        self.model = self.ocp.model
        self.N = self.ocp.config.N


# ───────────────────────── 闭环 ─────────────────────────


def run(yaw0_deg: float, *, rig: Rig, mode: str = "mismatch", seconds: float = 10.0,
        rate_hz: float = 50.0,
        p_init: Sequence[float] = (0.4, 0.0, 1.0), p_ref: Sequence[float] = (0.0, 0.0, 1.0),
        yaw_init_deg: float = 0.0, yaw_ref_deg: float = 0.0,
        tau_s: float = TAU_DEFAULT_S, judge_window_s: float = 3.0,
        settle_window_s: float = 1.0) -> Dict[str, object]:
    """跑一场**位置保持**闭环，`yaw0` 是注入进 `mocap` 那一侧的归零角。

    ★ 复刻的是**节点的每拍行为**，不是另写一套控制律：
      `x_meas → extrapolate_state(τ) → ocp.solve(x0, x_ref, u_ref=hold_feedforward) → u`
      每一步都用 `node_core` 里那个函数本身；被控对象只把 `u` 积分成**真世界系**的状态。

    **两条臂**（`mode`）：

    | mode | 喂给控制器的量 | 对应 |
    |---|---|---|
    | `mismatch`（现实） | **只有姿态**左乘 `Rz(−yaw0)`，位置/速度原样 | `mocap_rel_pose.hpp:283` 那一行 |
    | `consistent`（对照） | 位置、速度、姿态**一起**做 `Rz(−yaw0)`，参考锚点也跟着转 | 路线 2「把位置也一起转」 |

    ★★ `consistent` 那一臂是**为了证伪**：它把整个世界转过同一个角，数学上与 `yaw0 = 0` 等价
      ⟹ 它**必须**与无错配跑得一样好。若它也退化，那"错配导致退化"这个归因就不成立，
      该怀疑的是本文件的环路而不是那个 `Rz`。

    度量**统一对着物理参考点**（两条臂的目标是同一个物理点）⟹ 两臂可以直接比。
    """
    if mode not in MODES:
        raise ValueError(f"mode 只能是 {MODES}，收到 {mode!r}")
    # ★★ 每场**先复位求解器** —— 否则上一场（尤其是发散那几档）的迭代点会当热启动喂进本场。
    #    本会话实测过这个污染：`yaw0 = 90°` 跑飞之后紧跟的 `yaw0 = 0` 那一场
    #    机头稳到了 **−24.70°**（同一场单独跑是 **0.00°**）—— 读数全废，而且**不报错**。
    #    ⚠️ 场**内**的热启动要保留（节点就是那样跑的），复位只发生在本函数入口。
    rig.ocp.solver.reset()
    N = rig.N
    cfg = ControlConfig(tau_s=tau_s)                       # `hold_yaw=None` ⟹ 取锚点机头
    yaw0 = math.radians(float(yaw0_deg))
    yaw_ref = math.radians(yaw_ref_deg)
    rot = _rz(-yaw0)                                       # 左乘用的那个 Rz(−yaw0)
    consistent = (mode == "consistent")

    p_ref_v = np.asarray(p_ref, dtype=float)
    anchor_p = (rot @ p_ref_v) if consistent else p_ref_v
    anchor = np.concatenate([anchor_p, np.zeros(3), quat_from_yaw(yaw_ref - yaw0 if consistent
                                                                 else yaw_ref)])
    x_ref = hold_reference(anchor, cfg, N)
    u_ref = hold_feedforward(rig.params, N)

    x_true = pose_to_state(p_init, quat_from_yaw(math.radians(yaw_init_deg)))
    dt = 1.0 / float(rate_hz)
    n = int(round(seconds * rate_hz))

    t: List[float] = []
    p: List[np.ndarray] = []
    v: List[np.ndarray] = []
    q: List[np.ndarray] = []
    uu: List[np.ndarray] = []
    fail_reason = ""

    for k in range(n):
        # ① 这一拍控制器**看到**的量。（`consistent` 时位置/速度也一起转 —— 那是路线 2 的世界）
        pv = (rot @ x_true[:3], rot @ x_true[3:6]) if consistent else (x_true[:3], x_true[3:6])
        x_meas = np.concatenate([pv[0], pv[1], yaw_zero(x_true[6:10], yaw0)])
        x0 = extrapolate_state(x_meas, cfg.tau_s)
        sol = rig.ocp.solve(x0, x_ref, u_ref=u_ref, prediction=False)
        if not sol.info.ok:
            fail_reason = f"第 {k} 拍求解不可用：{sol.info.line()}"
            break
        u = np.asarray(sol.u, dtype=float).reshape(4)
        # ② 被控对象：**真姿态**积分，指令按机体轴兑现（姿态环不假，见文件头）
        x_true = np.asarray(rig.model.step_rk4(x_true, u, dt), dtype=float)
        x_true[6:10] = normalize_quat(x_true[6:10])
        t.append(k * dt)
        p.append(x_true[:3].copy())
        v.append(x_true[3:6].copy())
        q.append(x_true[6:10].copy())
        uu.append(u.copy())

    # ★ 「跑不满」也是一种未达 —— 必须带得出原因，别让 `ok=False` 配一条空串
    if not fail_reason and len(t) < 2:
        fail_reason = f"只跑到 {len(t)} 拍，不足判定窗口（seconds={seconds:g} @ {rate_hz:g} Hz）"

    out: Dict[str, object] = {
        "yaw0_deg": float(yaw0_deg), "mode": mode,
        "ok": (not fail_reason) and len(t) >= 2,
        "completed": not fail_reason,
        "fail_reason": fail_reason, "n_frames": len(t),
        "seconds": seconds, "rate_hz": rate_hz, "t": np.asarray(t),
        "p": np.asarray(p).reshape(-1, 3) if p else np.zeros((0, 3)),
        "v": np.asarray(v).reshape(-1, 3) if v else np.zeros((0, 3)),
        "q": np.asarray(q).reshape(-1, 4) if q else np.zeros((0, 4)),
        "u": np.asarray(uu).reshape(-1, 4) if uu else np.zeros((0, 4)),
    }
    if len(t) >= 2:
        out.update(_metrics(out, p_ref=p_ref_v, yaw_ref_deg=yaw_ref_deg,
                            judge_window_s=judge_window_s, settle_window_s=settle_window_s))
    else:
        out.update({k: float("nan") for k in
                    ("final_horiz_err_m", "heading_err_deg", "err_pp_m", "err_max_m",
                     "move_dir_deg", "final_yaw_deg")})
    return out


def _metrics(out: Dict[str, object], *, p_ref: np.ndarray, yaw_ref_deg: float,
             judge_window_s: float, settle_window_s: float) -> Dict[str, object]:
    t = np.asarray(out["t"], dtype=float)
    p = np.asarray(out["p"], dtype=float)
    v = np.asarray(out["v"], dtype=float)
    yaws = np.array([math.degrees(euler_from_quat(q)[2])
                     for q in np.asarray(out["q"], dtype=float)])

    err = np.linalg.norm(p[:, :2] - p_ref[:2], axis=1)
    tail = t >= (t[-1] - judge_window_s)
    settle = t >= (t[-1] - settle_window_s)

    # 初段"往哪走"：第一帧速度超过 0.05 m/s 的方向。位置误差在 +x ⟹ 没错配时应指向 −x（180°）
    speed = np.linalg.norm(v[:, :2], axis=1)
    k0 = int(np.argmax(speed > 0.05)) if bool(np.any(speed > 0.05)) else 0
    move_dir = math.degrees(math.atan2(v[k0, 1], v[k0, 0])) if speed[k0] > 0 else float("nan")

    return {
        "final_horiz_err_m": float(np.mean(err[settle])),
        "final_yaw_deg": float(yaws[-1]),
        "heading_err_deg": float(wrap_deg(yaws[-1] - yaw_ref_deg)),
        "err_pp_m": float(np.max(err[tail]) - np.min(err[tail])),
        "err_max_m": float(np.max(err)),
        "move_dir_deg": float(move_dir),
        "yaw_rate_dps": float((yaws[-1] - yaws[0]) / max(1e-9, t[-1] - t[0])),
        "yaw_series_deg": yaws,
    }


# ───────────────────────── 打印 ─────────────────────────


HEAD = ("yaw0°", "臂", "末段水平误差m", "机头°/机头速率°/s", "末段误差峰峰m", "最大误差m",
        "初段去向°", "跑满")


def _row(r: Dict[str, object]) -> str:
    arm = "错配" if r["mode"] == "mismatch" else "一致(对照)"
    if r["n_frames"] < 2:
        return f"{r['yaw0_deg']:>6.0f}  {arm:<11}  ☠️ 一拍都没跑成：{r['fail_reason']}"
    return (f"{r['yaw0_deg']:>6.0f}  {arm:<11}  {r['final_horiz_err_m']:>12.4f}  "
            f"{r['heading_err_deg']:>8.2f}/{r['yaw_rate_dps']:>6.2f}  "
            f"{r['err_pp_m']:>13.4f}  {r['err_max_m']:>8.4f}  {r['move_dir_deg']:>8.1f}  "
            f"{'✅' if r['completed'] else '❌'}")


def main(argv: Optional[Sequence[str]] = None) -> int:
    ap = argparse.ArgumentParser(description="yaw_zeroed 错配的离线闭环量级（ticket 24 路线 4）")
    ap.add_argument("--yaw0", type=float, nargs="*", default=list(YAW0_DEGS),
                    help=f"要扫的归零角（度），默认 {' '.join(str(v) for v in YAW0_DEGS)}")
    ap.add_argument("--seconds", type=float, default=30.0)
    ap.add_argument("--rate-hz", type=float, default=50.0)
    ap.add_argument("--p-init", type=float, nargs=3, default=(0.4, 0.0, 1.0))
    ap.add_argument("--p-ref", type=float, nargs=3, default=(0.0, 0.0, 1.0))
    ap.add_argument("--modes", nargs="*", default=list(MODES), choices=list(MODES))
    ap.add_argument("--codegen-dir", default=None)
    a = ap.parse_args(argv)

    print(f"注入口径：{YAW_ZEROED_SOURCE}")
    print(f"初始位置 {tuple(a.p_init)} → 参考 {tuple(a.p_ref)}；{a.seconds:g} s @ {a.rate_hz:g} Hz")
    rig = Rig(codegen_dir=a.codegen_dir)
    print(f"OCP: N={rig.N}，m={rig.params.mass} kg，t_hover={rig.params.t_hover:.4f} N，"
          f"f_max={rig.params.f_max} N，t_max={rig.params.t_max:.4f} N\n")
    print("  ".join(f"{h:>11}" for h in HEAD))
    for mode in a.modes:
        for y in a.yaw0:
            r = run(y, rig=rig, mode=mode, seconds=a.seconds, rate_hz=a.rate_hz,
                    p_init=a.p_init, p_ref=a.p_ref)
            print(_row(r))
    print("\n读法：①「末段水平误差」两臂都在 0 附近 ⟹ 位置环**吸收得了**这个错配，"
          "指纹不在这里；"
          "\n      ②「机头°/速率」是**稳态**列 —— 错配臂会朝「参考机头 + yaw0」缓慢爬，"
          "但爬得**很慢**；"
          "\n      ③「末段误差峰峰 / 最大误差」是**稳定性**列 —— yaw0 大到某个值开始发散，"
          "那一档就是临界；"
          "\n      ④ ★ **一致(对照)臂必须与 yaw0 = 0 一样好** —— 它要是也退化，"
          "那本工具的归因就不成立，先查环路别信结论。")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
