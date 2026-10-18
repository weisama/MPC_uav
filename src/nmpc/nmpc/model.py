"""10 状态四旋翼模型（`ω` **不作状态**）—— ENU 世界系 / FLU 机体系。

############################################################################
#
#   Copyright (C) 2022 PX4 Development Team. All rights reserved.
#
#   本文件的动力学与四元数工具**取自参考实现 `DISCOWER/px4-mpc` @ 3e541af**
#   （`px4_mpc/models/multirotor_rate_model.py`，BSD 3-Clause，版权与许可条款照原样保留）。
#   相对它的改动：参数走 `Z0Params`（M1/M3/M4）、`get_acados_model()` 的结果**在构造时缓存**
#   （参考实现每个控制周期重建 2 次 CasADi 表达式，见下）。
#
############################################################################

★ 坐标系约定（**先说，因为后面每个式子都依赖它**）

    世界系 **ENU**（x 东、y 北、z 天）—— `g = [0, 0, −9.81]`
    机体系 **FLU**（x 前、y 左、z 上 = 推力方向）—— `force = [0, 0, F]`，`F ≥ 0`

**不是 PX4 的 NED/FRD。** 选 ENU/FLU 的推导、理由与代价、以及"MPC 内部 FLU →
PX4 FRD"的那张映射表，**唯一权威在 [`docs/control/sign-map.md`](../../../docs/control/sign-map.md)** ——
本文件不复制那张表的任何数值（[该文件 §8](../../../docs/control/sign-map.md) 的防漂移契约）。

状态与控制量（`nx = 10`、`nu = 4`）：

    x = [p(3), v(3), q(4)]        p/v 在世界系 [m]/[m/s]，q = 机体 ← 世界 的单位四元数 [w,x,y,z]
    u = [F, ωx, ωy, ωz]           F [N]，ω 在**机体系** [rad/s]

动力学（`ω` **不作状态**，姿态运动学直接用角速率输入 —— 即"内环完美跟踪"假设）：

    ṗ = v
    v̇ = R(q)·[0,0,F]/m + g
    q̇ = ½·skew(ω)⊗q          ← 用 ω_des 代替真实 ω

★ **这是刻意的工程化简，不是论文的标准做法**：项目依据的两篇论文（`Sun2022`、`Foehn2022`）都把 `ω`
作为状态。选它的理由是参考实现已被实飞验证、且正好是路线图规划的"步骤 1"。升级到 13 状态
（`ω̇ = k_ω(ω_des − ω)`，需辨识 `k_ω`）的方向**与论文一致**，不是自创。
见 [`phase2-implementation.md`](../../../docs/control/phase2-implementation.md) §2.1。
"""

from __future__ import annotations

import casadi as cs
import numpy as np
from acados_template import AcadosModel

from .params import Z0Params

#: 状态维度 / 控制维度 / 四元数起始下标（`nmpc.ocp` 与测试都用这几个常量，不写字面量）
NX = 10
NU = 4
Q_SLICE = slice(6, 10)


def skew_symmetric(v) -> cs.MX:
    """四元数乘法的反对称矩阵（`q̇ = ½·skew(ω)⊗q` 里的那个）。

    ⚠️ 它**不是**"任意向量的叉乘矩阵"：`cs.vertcat(v, ...)` 的排布是为**四元数右乘**写的
    （见下式与 `q̇` 那一行）。照抄参考实现，**不要**按教科书叉乘矩阵去"改正"它。
    """
    return cs.vertcat(
        cs.horzcat(0, -v[0], -v[1], -v[2]),
        cs.horzcat(v[0], 0, v[2], -v[1]),
        cs.horzcat(v[1], -v[2], 0, v[0]),
        cs.horzcat(v[2], v[1], -v[0], 0),
    )


def q_to_rot_mat(q) -> cs.MX:
    """单位四元数 `[w,x,y,z]` → 旋转矩阵 `R(q)`（把**机体系**矢量转到**世界系**）。"""
    qw, qx, qy, qz = q[0], q[1], q[2], q[3]
    return cs.vertcat(
        cs.horzcat(1 - 2 * (qy**2 + qz**2), 2 * (qx * qy - qw * qz), 2 * (qx * qz + qw * qy)),
        cs.horzcat(2 * (qx * qy + qw * qz), 1 - 2 * (qx**2 + qz**2), 2 * (qy * qz - qw * qx)),
        cs.horzcat(2 * (qx * qz - qw * qy), 2 * (qy * qz + qw * qx), 1 - 2 * (qx**2 + qy**2)),
    )


def v_dot_q(v, q) -> cs.MX:
    """把机体系矢量 `v` 转到世界系。"""
    return cs.mtimes(q_to_rot_mat(q), v)


def quat_from_rotvec(rotvec: np.ndarray) -> np.ndarray:
    """旋转矢量（轴 × 角度，机体系/世界系同式）→ 四元数 `[w,x,y,z]`。测试与参考轨迹要用。"""
    rotvec = np.asarray(rotvec, dtype=float)
    theta = float(np.linalg.norm(rotvec))
    if theta < 1e-12:
        return np.array([1.0, 0.0, 0.0, 0.0])
    axis = rotvec / theta
    return np.concatenate([[np.cos(theta / 2)], axis * np.sin(theta / 2)])


def yaw_quat(yaw: float) -> np.ndarray:
    """纯偏航角的四元数 —— 绕世界 z（ENU 的「天」）转。"""
    return np.array([np.cos(yaw / 2), 0.0, 0.0, np.sin(yaw / 2)])


def quat_mul(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """四元数乘法 `a ⊗ b`（先转 `b` 再转 `a`）。"""
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return np.array([
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    ])


def normalize_quat(q: np.ndarray) -> np.ndarray:
    """把四元数归一化 —— 求解器不保证 `|q| = 1`（模型里没有这条约束），闭环仿真用它。"""
    q = np.asarray(q, dtype=float)
    return q / np.linalg.norm(q)


def quat_angle(a: np.ndarray, b: np.ndarray) -> float:
    """两个姿态之间的**测地角** [rad]（0…π）—— "这两个姿态差多少"，与朝向参数化无关。

    ★ `|a·b|`：四元数 `q` 与 `−q` 是**同一个姿态**，不取绝对值的话"姿态没变"会被算成 2π。
    ★ 用 `acos` 而不是逐分量比差：分量差在 `q ≈ −q` 附近会给出 2 的量级（"翻转了"），
      而实际姿态一模一样。
    用在**位姿合理性判据**里（`node_core.NodeCore._pose_jump`）：动捕刚体认错/遮挡恢复时
    会给出一个"瞬移"的姿态，判据要能把它与真实机动分开。
    """
    a = normalize_quat(np.asarray(a, dtype=float).reshape(4))
    b = normalize_quat(np.asarray(b, dtype=float).reshape(4))
    return float(2.0 * np.arccos(np.clip(abs(float(np.dot(a, b))), 0.0, 1.0)))


class Z0MultirotorRateModel:
    """z0 的 10 状态模型 —— 负责给出 `AcadosModel` 与一个可调用的动力学函数。

    ★ **`AcadosModel` 只在构造时建一次**（`self.acados_model`）。参考实现是在每个控制周期里
    调 2 次 `get_acados_model()`（只为取 `nx`/`nu` 两个整数），单次 **3.10 ms / max 8.39 ms**，
    而真正解一次 QP 只要 2.81 ms —— 照抄它，100 Hz 的预算在"还没开始算"时就花光了
    （[`acados-orin-build.md`](../../../docs/control/acados-orin-build.md) §6.2）。
    这里把它挪到构造期，并**同时缓存** `nx`/`nu`。
    """

    def __init__(self, params: Z0Params | None = None):
        self.params = params or Z0Params()
        self.acados_model = self._build_acados_model()
        self.nx = NX
        self.nu = NU
        # 供测试与闭环仿真用的一份纯数值动力学（CasADi Function，不参与 OCP）
        self._f_expl = cs.Function("z0_f_expl", [self.acados_model.x, self.acados_model.u],
                                   [self.acados_model.f_expl_expr])

    # ── 参数转发（参考实现里是 `model.mass` / `model.max_thrust` / `model.max_rate`） ──
    @property
    def mass(self) -> float:
        return self.params.mass

    @property
    def max_thrust(self) -> float:
        return self.params.t_max

    @property
    def max_rate(self) -> tuple[float, float, float]:
        return self.params.rate_max

    def _build_acados_model(self) -> AcadosModel:
        p = cs.MX.sym("p", 3)
        v = cs.MX.sym("v", 3)
        q = cs.MX.sym("q", 4)
        x = cs.vertcat(p, v, q)

        F = cs.MX.sym("F")
        w = cs.MX.sym("w", 3)
        u = cs.vertcat(F, w)

        p_dot = cs.MX.sym("p_dot", 3)
        v_dot = cs.MX.sym("v_dot", 3)
        q_dot = cs.MX.sym("q_dot", 4)
        xdot = cs.vertcat(p_dot, v_dot, q_dot)

        g = cs.vertcat(0.0, 0.0, -9.81)  # ENU：重力朝 −z
        force = cs.vertcat(0.0, 0.0, F)  # FLU：推力沿机体 +z

        a_thrust = v_dot_q(force, q) / self.mass

        f_expl = cs.vertcat(v, a_thrust + g, 1 / 2 * cs.mtimes(skew_symmetric(w), q))

        model = AcadosModel()
        model.f_expl_expr = f_expl
        model.f_impl_expr = xdot - f_expl
        model.x = x
        model.xdot = xdot
        model.u = u
        model.name = "z0_multirotor_rate_model"
        return model

    # ── 给测试与闭环仿真用的数值接口 ──
    def dynamics(self, x: np.ndarray, u: np.ndarray) -> np.ndarray:
        """一步显式动力学 `ẋ = f(x, u)`（纯数值，不碰求解器）。"""
        return np.array(self._f_expl(x, u)).reshape(NX)

    def step_rk4(self, x: np.ndarray, u: np.ndarray, dt: float) -> np.ndarray:
        """四阶 Runge–Kutta 推进一步 —— 闭环仿真用，**不是** OCP 里的积分器。

        ⚠️ OCP 内部用的是 acados 的 ERK（4 级 3 步，见 `ocp.py`），这里只是给测试提供一个
        "自洽地往前走一步"的手段。二者的差别不影响本阶段要问的问题（代价是否罚相对参考的偏差）。
        """
        k1 = self.dynamics(x, u)
        k2 = self.dynamics(x + 0.5 * dt * k1, u)
        k3 = self.dynamics(x + 0.5 * dt * k2, u)
        k4 = self.dynamics(x + dt * k3, u)
        return x + dt / 6.0 * (k1 + 2 * k2 + 2 * k3 + k4)
