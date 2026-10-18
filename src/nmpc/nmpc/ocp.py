"""离线 OCP —— `(状态, 参考) → u = [F, ωx, ωy, ωz]`，**零 ROS**。

这是 [ticket 04](../../../.scratch/phase2/issues/04-offline-ocp.md) 的交付物，也是
[`spec.md`](../../../.scratch/phase2/spec.md) 的 **seam 2**：把 OCP 的求解函数当作被测单元，
**不接 ROS、不接通信、不起节点**。理由（任一条都成立，见 spec 的 Testing Decisions）：

1. 它是**唯一**能测出最坏求解耗时的地方，而这个数字直接决定 MPC 频率（→ D6）；
2. 数值求解器的失败**在话题边界上没有任何诊断分辨率** —— "输出不对"无法区分是模型错、代价错、
   约束错还是没收敛；
3. 它本来就是实施清单里的步骤 S3，不是为本张票新造的开销。

⚠️ **本模块证明不了"符号约定与真 PX4 一致"**：它只能验证"求解器自洽"。
`(状态, 参考) → u` 里的 `u` 是 **MPC 内部的 FLU 量**，
映射到 PX4 的 `VehicleRatesSetpoint` 是**另一件事**（权威表见
[`docs/control/sign-map.md`](../../../docs/control/sign-map.md)，实现归 ticket 05 / 06）。
所以本模块里**没有一行**涉及 `roll` / `pitch` / `yaw` / `thrust_body`。

求解器配置（照抄参考实现，`phase2-implementation.md` §2.6「可直接抄的部分」）：
`SQP_RTI` + `PARTIAL_CONDENSING_HPIPM` + `GAUSS_NEWTON` + `ERK`（4 级 3 步），**每控制周期一次迭代**。
"""

from __future__ import annotations

import os
import time
from dataclasses import dataclass, field
from pathlib import Path

import casadi as cs
import numpy as np
import scipy.linalg
from acados_template import AcadosOcp, AcadosOcpSolver

from .model import NU, NX, Q_SLICE, Z0MultirotorRateModel
from .params import Z0Params

#: 代码生成目录的默认落点：`$MPC_CODEGEN_DIR` → `$XDG_CACHE_HOME/mpc-ocp-codegen` → `~/.cache/...`
#: ★ **刻意不放进源码树**（参考实现的路子是 `px4_mpc/mpc_codegen/`）：生成物 + 编译出的 `.so`
#: 不该跟着 git 走，也不该在 colcon 构建时往 `src/` 里写东西。
DEFAULT_CODEGEN_DIR = Path(
    os.environ.get("MPC_CODEGEN_DIR")
    or Path(os.environ.get("XDG_CACHE_HOME", Path.home() / ".cache")) / "mpc-ocp-codegen"
)


@dataclass(frozen=True)
class OcpConfig:
    """求解器与代价的配置。

    求解器那一套（`SQP_RTI`/`HPIPM`/`GAUSS_NEWTON`/`ERK`、`N`/`tf`）照抄参考实现 `multirotor_rate_mpc.py`；
    **代价权重是 z0 的（ticket 16，2026-09-29 操作者裁定）**，与参考实现不同的只有 `q_diag` 的四元数段：
    `(0.0, 0.1, 0.1, 0.1)` → `(0.1, 2.0, 2.0, 2.0)`。偏航角速率权重 `R_yaw` **与参考实现同为 500**
    （☠️ 曾经跟着一起降到 50，见 `r_diag` 的 ☠️）。
    """

    #: 预测步数 / 时域 [s] —— 参考实现是 `N=50, Tf=1.0` ⟹ 内部步长 20 ms
    N: int = 50
    tf: float = 1.0

    #: 状态代价（对角线），顺序 = `[p(3), v(3), q(4)]`
    #: ★ 四元数段第 1 项 `qw = 0.1`（**不是参考实现的 0**）：`qw = 0` 使代价**分不出**参考机头 `ψ`
    #:   与它的镜像 `2π − ψ`（两者只差 `w`）—— 偏航成了一条**死通道**（ticket 09 闭环实测）。
    #:   `qatt = 2.0`（参考实现 0.1）见 `r_diag` 与本类 docstring。
    q_diag: tuple[float, ...] = (1e1, 1e1, 1e1, 1e1, 1e1, 1e1, 0.1, 2.0, 2.0, 2.0)
    #: 终端状态代价（对角线），顺序同上
    #: ⚠️ 终端四元数段仍是 `0`（照抄参考实现）—— 镜像井的**阶段项**由 `q_diag` 的 `qw` 消掉即可。
    q_e_diag: tuple[float, ...] = (3e2, 3e2, 3e2, 1e2, 1e2, 1e2, 0.0, 0.0, 0.0, 0.0)
    #: 控制代价（对角线），顺序 = `[F, ωx, ωy, ωz]`。三轴同为 5e2、`F` 1e1（照抄参考实现）。
    #: ☠️ 2026-09-29 一度把 `ωz` 降到 **50**（"方案 E"），**同日撤回**：`R_yaw` 一低，
    #:   **10 m 不可行步进下 RTI 迭代收敛得极慢** —— 逐拍变化 `|Δu|` 按几何衰减，
    #:   在 **60 拍**的探针里看着像"逐拍乱跳"（`ωz` 在 ±0.85…1.19 rad/s 之间翻、`ωy` ±0.15），
    #:   实际上跑到 400 拍就衰减到 `2.6e-4`（⟹ 不是极限环，是慢收敛）。
    #:   发出指令的那段窗口（节点每拍一次迭代）就在这段慢收敛里 ⟹ 指令帧帧大幅摆动。
    #:   同一场景 `R_yaw = 500` 第 50 拍就已经精确收敛（`|Δu| = 0.00000`、交叉轴 ≤ 1.2e-3）。
    #:   判据在 50 与 55 之间（`R_yaw ∈ 55/60/65/70/80/90/100/150/200/300/500` 都收敛，
    #:   单轴读数 = `tools/rti-convergence/evidence/r_yaw_sweep.json`）。
    #:   ⚠️ 但**这不是"`R_yaw=500` 就稳"**：同一张扫描里 `R_yaw=5e2` 在 **6 m**（`ωx` 极限环
    #:   ±0.595）与 **7 m**（`F` 极限环 ±1.62）上也各有**真极限环**（150→399 拍不衰减）；
    #:   `R_yaw=50` 的真极限环在 3/4/5/6 m（12 m 那条不衰减也不收敛，判据记为"未定"）。
    #:   ⟹ 大不可行步进这一族的稳定区**逐工况**变，两个权重各有各的坏区，别把这条读成"越大越稳"。
    #:   读数与判据：`tools/rti-convergence/`（偏移 × `R_yaw` 那张 = `evidence/scan.json`，
    #:   `R_yaw` 单轴那张 = `evidence/r_yaw_sweep.json`）。
    #: ★ 它换来的"回正权威"比预想小得多，所以不值得买：**纯 0.4 rad 机头误差**的稳态 `ωz`
    #:   在 `qw=0.1, qatt=2.0` 下是 `R_yaw=500 ⟹ 3.9e-4`、`R_yaw=50 ⟹ 3.9e-3`（旧 A 组 2e-5）
    #:   —— 三个数**都在 100…1000 s 量级** ⟹"偏航是死通道"这条**没有**被这一项解决（见 ticket 16）。
    #:   ⚠️ 别把这条读成"`R_yaw` 越小越好"：它是**代价**，不是增益旋钮。
    r_diag: tuple[float, ...] = (1e1, 5e2, 5e2, 5e2)

    def __post_init__(self) -> None:
        if len(self.q_diag) != NX or len(self.q_e_diag) != NX or len(self.r_diag) != NU:
            raise ValueError("代价对角线长度必须是 nx=10 / nu=4")

    def cost(self, x_pred: np.ndarray, u_pred: np.ndarray, x_ref: np.ndarray,
             u_ref: np.ndarray) -> float:
        """按 acados `NONLINEAR_LS` 的定义**自己重算**一条轨迹的代价。

        `J = Σ_{i<N} ½‖y_i − y_ref,i‖²_W + ½‖x_N − x_ref,N‖²_{W_e}`，其中 `y = [x; u]`。
        ★ 装配时取的是 `W = 2·diag(权重)`（见 `_build_solver`），那个 `2` 与 acados 的 `½`
        正好抵消 ⟹ **这里的值就是 `Σ (权重 ⊙ 偏差²)`**，和本类那三条对角线同一个量纲，
        不需要再乘任何系数。

        ⚠️ **为什么不直接用 `solver.get_cost()`**：见 `SolveInfo.cost` 的说明 ——
        `SQP_RTI` 下它不反映返回轨迹的代价，实测差两个数量级。
        """
        ex = np.asarray(x_pred, float).reshape(self.N + 1, NX) - np.asarray(x_ref, float).reshape(self.N + 1, NX)
        eu = np.asarray(u_pred, float).reshape(self.N, NU) - np.asarray(u_ref, float).reshape(self.N, NU)
        j = float(np.sum(ex[:self.N] ** 2 @ np.asarray(self.q_diag))
                  + np.sum(eu ** 2 @ np.asarray(self.r_diag))
                  + np.sum(ex[self.N] ** 2 @ np.asarray(self.q_e_diag)))
        return j


@dataclass
class SolveInfo:
    """一次求解的**数值健康度** —— 使"求解器在悄悄退化"不会一直不可见（票面验收项）。

    ⚠️ **单看 `status == 0` 不足以说明健康**：SQP_RTI 每周期只做一次迭代，它可以在解还很差时
    照样返回 0。三样东西一起看：**状态码**（`status` / `qp_stat`）、**迭代数**（`sqp_iter` / `qp_iter`）、
    **残差类量**（见下）。

    ★ **第一手（2026-09-28，本机实测）：`SQP_RTI` 下 `get_stats("residuals")` 恒为全 0**，
    `get_stats("alpha")` 直接抛 `ValueError: alpha values are not available for SQP_RTI`。
    所以"残差"这一项**不能**照抄普通 SQP 的读法，本模块改为给出两个**自算**的替代量
    （不依赖 acados 内部记账，任何求解器类型下都有意义）：

    - `u_bound_violation`：返回的 `u` 相对 `[lbu, ubu]` 的**越界量**。RTI 只在**线性化后**的
      QP 上满足约束，越界（哪怕很小）是"模型/权重把解推到约束外面"的第一个可观测信号。
    - `quat_norm_drift`：预测轨迹上 `|q|² − 1` 的最大偏离。这个模型**没有 `|q| = 1` 的约束**，
      漂了之后 `R(q)` 不再正交，而**没有任何东西会报错** —— 所以留一个看得见的读数。
      ★ 实测（2026-09-28 本机，1 s 时域＝50 步，`ω` 打满 3.84 rad/s）：漂移 **1.0e-08**；
      悬停 1.8e-15、+x 2 m 2.4e-11、匀速偏航 6.0 rad/s 1.3e-10 —— 即**正常时它本来就该接近 0**，
      涨起来才有信息量（轨迹发散会把它顶上去）。用 `step_rk4` 以 20 ms 步长连续滚 30 s 也只到 6.7e-08。
      ⚠️ **它不进 `degraded`**：它是**模型精度**的指示灯（正常量级 1e-8），不是"求解器退化"的判据。

    ★ **第一手（2026-09-28，本机实测）：`solver.get_cost()` 在 `SQP_RTI` 下不可用**。
    同一次求解、同一个返回轨迹：`get_cost() = 516.07`，而按代价定义重算是 **49501.7**（差 96 倍）；
    单看第一步就已是 1220.25，所以它**也不是**"只算了某一段"。源码侧的旁证：
    `ocp_nlp_get_cost_value_from_submodules`（`ocp_nlp_common.c:3908`）在 `ocp_nlp_sqp_rti.c` 里
    没有调用点 —— RTI 不跑 `eval_cost`，那个字段留着上一轮/初始化的残留值。
    ⟹ 本模块的 `cost` **自己算**（`OcpConfig.cost`），`cost_acados` 只把原始读数留证。
    """

    status: int
    sqp_iter: int
    qp_iter: np.ndarray          # (2,) 两个 QP 阶段的迭代数
    qp_stat: np.ndarray          # (2,) QP 状态码（0 = 正常；1 = 迭代上限；3 = NaN；4 = inf）
    residuals: np.ndarray        # ⚠️ RTI 下恒为 0，保留只为"看得到它确实是 0"
    u_bound_violation: float     # 自算：u 相对 [lbu, ubu] 的越界量
    quat_norm_drift: float       # 自算：max | |q|² − 1 |
    cost: float                  # ★ 自算（`OcpConfig.cost`）；`prediction=False` 时是 nan
    cost_acados: float           # ⚠️ `solver.get_cost()` 的原样读数 —— 留证，见下
    time_tot: float              # acados 自报的求解耗时 [s]
    time_cycle: float            # 整个控制周期（设参考 + 设初值 + 求解 + 取解）[s]

    #: 判"越界"的阈值：`u` 越过约束超过它就置 `degraded`
    BOUND_TOL = 1e-6

    @property
    def residual_max(self) -> float:
        """acados 自报的残差上界 —— ⚠️ RTI 下**恒为 0**，不要拿它当健康依据。"""
        return float(np.max(np.abs(self.residuals))) if self.residuals.size else float("nan")

    @property
    def cost_available(self) -> bool:
        """本次求解有没有算出代价（`prediction=False` 时没读轨迹，算不了，见 `solve()`）。"""
        return bool(np.isfinite(self.cost))

    @property
    def ok(self) -> bool:
        """状态码层面正常（不含"数值悄悄退化"的判断，那是 `degraded`）。"""
        return self.status == 0 and bool(np.all(self.qp_stat == 0))

    @property
    def degraded(self) -> bool:
        """★ **"求解器在悄悄退化"的总开关**：状态码异常、`u` 越界、代价/耗时非有限 —— 任一命中。

        ⚠️ **刻意不含 `quat_norm_drift`**（理由见上）；**代价**只在真算过的时候才参与判断
        （`cost_available`），否则 `prediction=False` 会把 nan 误报成退化。
        """
        return (not self.ok
                or self.u_bound_violation > self.BOUND_TOL
                or (self.cost_available and not np.isfinite(self.cost))
                or not np.isfinite(self.time_tot))

    def line(self) -> str:
        cost = f"{self.cost:.6g}" if self.cost_available else "—(未取轨迹)"
        return (f"status={self.status} sqp_iter={self.sqp_iter} qp_iter={self.qp_iter.tolist()} "
                f"qp_stat={self.qp_stat.tolist()} "
                f"|Δq|={self.quat_norm_drift:.2e} u_viol={self.u_bound_violation:.2e} "
                f"cost={cost} time_tot={self.time_tot * 1e3:.3f} ms "
                f"cycle={self.time_cycle * 1e3:.3f} ms" + ("  ⚠️ DEGRADED" if self.degraded else ""))


@dataclass
class OcpSolution:
    """一次求解的完整结果。`u` 是**只取第一拍**的控制量 —— 那才是要下发的。"""

    u: np.ndarray                     # (nu,) 第一拍
    u_pred: np.ndarray                # (N, nu) 整条预测控制序列
    x_pred: np.ndarray                # (N+1, nx) 预测状态轨迹
    info: SolveInfo
    ref: np.ndarray = field(default=None, repr=False)  # 本次用的参考（便于复盘）


class OfflineOcp:
    """10 状态 OCP 的离线求解封装。

    用法（零 ROS）：

        ocp = OfflineOcp()
        sol = ocp.solve(x0, x_ref, u_ref)       # → sol.u = [F, ωx, ωy, ωz]
        print(sol.info.line())                  # 数值健康度

    **`u_ref` 的取法（M6，票面要求"写定，不留实现时随便定"）**：

    | 调用方式 | `u_ref` 取什么 | 用在哪 |
    |---|---|---|
    | `solve(x0, x_ref)` | **上一帧解平移一拍**（末尾重复）；本实例的第一次调用用悬停前馈 `[T_hover,0,0,0]` | 在线闭环：没有外部前馈时的缺省 |
    | `solve(x0, x_ref, u_ref)` | **调用方给的前馈**（如轨迹的解析前馈、或上一帧解） | 节点有参考轨迹时走这条 |

    ★ 为什么不能像参考实现那样把 `yref` 的控制部分**全填 0**：那等于**罚角速率的绝对值**，
    MPC 会把所有时刻的 ω 往零拉 —— 需要**持续角速率**的场景（绕圈、定点自旋）
    会出现系统性稳态偏差，且机动性被无声压制（坑 4 / M6）。
    """

    def __init__(self, params: Z0Params | None = None, config: OcpConfig | None = None,
                 codegen_dir: Path | str | None = None, verbose: bool = False):
        self.params = params or Z0Params()
        self.config = config or OcpConfig()
        self.model = Z0MultirotorRateModel(self.params)
        self.codegen_dir = Path(codegen_dir or DEFAULT_CODEGEN_DIR)
        self.codegen_dir.mkdir(parents=True, exist_ok=True)

        self._build_solver(verbose=verbose)
        # 缺省 `u_ref` 的第一帧：悬停前馈（M6 的参考量，见类 docstring 的表）
        self._u_ref_last = np.tile(np.array([self.params.t_hover, 0.0, 0.0, 0.0]), (self.config.N, 1))
        #: 解过几次。`last_prediction()` 用它回答"求解器里到底有没有一条轨迹"——
        #: 没解过的时候 `get_flat` 读到的是**初始化时写进去的那条**（近似水平悬停的猜测），
        #: 把它当预测轨迹画出来就是拿假数据糊弄人。
        self._solves = 0

    # ── 装配 ────────────────────────────────────────────────────────────
    def _build_solver(self, verbose: bool) -> None:
        cfg, model = self.config, self.model
        p = self.params
        acados_model = model.acados_model

        ocp = AcadosOcp()
        ocp.code_export_directory = str(self.codegen_dir)
        ocp.model = acados_model

        N = cfg.N

        # ── 代价：NONLINEAR_LS，y = [x; u]，罚 ‖y − y_ref‖²_Q,R（照抄骨架） ──
        Q = 2 * np.diag(cfg.q_diag)
        Q_e = 2 * np.diag(cfg.q_e_diag)
        R = 2 * np.diag(cfg.r_diag)
        ocp.cost.cost_type = "NONLINEAR_LS"
        ocp.cost.cost_type_e = "NONLINEAR_LS"
        ocp.cost.W = scipy.linalg.block_diag(Q, R)
        ocp.cost.W_e = scipy.linalg.block_diag(Q_e)
        ocp.model.cost_y_expr = cs.vertcat(acados_model.x, acados_model.u)
        ocp.model.cost_y_expr_e = acados_model.x
        # 初值（每次 solve 都会被覆盖，见 `_set_reference`）；这里给一组无歧义的默认
        x_default = np.array([0, 0, 0, 0, 0, 0, 1.0, 0, 0, 0])
        u_default = np.zeros(NU)
        # 注意：`ocp.cost.yref` 是**一维**的（长度 ny），逐阶段的参考由 `solver.set(i, "yref", …)` 下发
        ocp.cost.yref = self._pack_ref(x_default, u_default)
        ocp.cost.yref_e = x_default.copy()

        # ── 约束：★ 换成 z0 的实测参数，且**偏航上限单独给**（M3） ──
        w_max = np.array(p.rate_max)
        ocp.constraints.lbu = np.concatenate([[0.0], -w_max])          # F ≥ 0（FLU 下推力非负）
        # ★ `F` 上限用 **`f_max`（按电池，M4/D5）**，不是 `t_max`：
        #   `t_max` 是**归一化**常数（= 实测满量程 46.3 N），拿它当上限等于按电机（46.3 vs 11.8 差 3.9 倍），
        #   而且**不报错** —— 求解器会规划出电池兑现不了的推力。两者的解耦见 `params.py` 的 `F_MAX_BATTERY`。
        ocp.constraints.ubu = np.concatenate([[p.f_max], w_max])       # F ≤ f_max（★ 按电池，M4）
        ocp.constraints.idxbu = np.arange(NU)
        # ⚠️ 参考实现这里是 `[0, -w, -w, -0.5*w]` / `[Fmax, w, w, +0.5*w]`：
        #    ① `w_max = 0.5` 是占位符；② 偏航被写成横滚的**一半**，而 z0 的偏航上限比横滚**更大**（6.28 > 3.84）。

        ocp.constraints.x0 = np.zeros(NX)   # 每次都覆盖，见 `solve()`

        # ── 求解器：SQP_RTI（每周期一次迭代）+ HPIPM（照抄参考实现） ──
        so = ocp.solver_options
        so.N_horizon = N            # ⚠️ 不要写 `ocp.dims.N`（0.6.0 起已迁移，会打弃用横幅）
        so.qp_solver = "PARTIAL_CONDENSING_HPIPM"
        so.hessian_approx = "GAUSS_NEWTON"
        so.integrator_type = "ERK"
        so.nlp_solver_type = "SQP_RTI"
        so.sim_method_num_stages = 4
        so.sim_method_num_steps = 3
        so.qp_solver_cond_N = N
        so.tf = cfg.tf
        if verbose:
            so.print_level = 1

        json_path = self.codegen_dir / "acados_ocp.json"
        self.acados_ocp = ocp
        self.solver = AcadosOcpSolver(ocp, json_file=str(json_path), verbose=verbose)
        # ★ `nx`/`nu` 从模型直接取，**不在求解循环里重建 CasADi 表达式**（见 model.py 的说明）
        self.nx, self.nu = self.model.nx, self.model.nu

    # ── 参考的打包 / 下发 ───────────────────────────────────────────────
    @staticmethod
    def _pack_ref(x_row: np.ndarray, u_row: np.ndarray) -> np.ndarray:
        """`[x; u]` —— 与 `cost_y_expr` 的排布一致（NONLINEAR_LS 的 y）。"""
        return np.concatenate([x_row, u_row])

    def _set_reference(self, x_ref: np.ndarray, u_ref: np.ndarray) -> None:
        x_ref = np.asarray(x_ref, dtype=float).reshape(self.config.N + 1, NX)
        u_ref = np.asarray(u_ref, dtype=float).reshape(self.config.N, NU)
        for i in range(self.config.N):
            self.solver.set(i, "yref", self._pack_ref(x_ref[i], u_ref[i]))
        self.solver.set(self.config.N, "yref", x_ref[self.config.N])

    def solve(self, x0: np.ndarray, x_ref: np.ndarray, u_ref: np.ndarray | None = None,
              prediction: bool = True) -> OcpSolution:
        """解一次 OCP。**求解失败不 raise** —— 由状态码如实返回（兜底是节点的事，见 M5 / ticket 07）。
        `ValueError` 只用于**调用方用错了 API**（见下），不用于求解器本身的问题。

        `prediction=False` 时**只取第一拍 `u`**，不把整条预测轨迹读出来 —— 省 ~1.1 ms/周期
        （实测，见本包 README 的耗时分解）。节点每周期只需要 `u`；预测轨迹是给 RViz 的，
        可以降频取。

        ⚠️ **`prediction=False` 时必须显式传 `u_ref`**：缺省的"上一帧解平移一拍"就是**从预测轨迹
        里来的**，不读轨迹就没有它。**不能**悄悄退回悬停前馈 —— 那会让 M6 的参考量永远停在
        `[T_hover,0,0,0]`，持续角速率场景照样被压（这正是 M6 要修的东西，只是换成了静默失败）。
        """
        t0 = time.perf_counter()
        x0 = np.asarray(x0, dtype=float).reshape(NX)
        if u_ref is None:
            if not prediction:
                raise ValueError(
                    "prediction=False 不读预测轨迹 ⟹ 无法维护缺省的『上一帧解平移一拍』u_ref。"
                    "请显式传入 u_ref（有参考轨迹时用它的解析前馈），或将 prediction 置 True。")
            u_ref = self._shifted_previous()
        self._set_reference(x_ref, u_ref)
        self.solver.set(0, "lbx", x0)
        self.solver.set(0, "ubx", x0)

        status = int(self.solver.solve())
        self._solves += 1
        time_tot = float(self.solver.get_stats("time_tot"))
        u_first = np.array(self.solver.get(0, "u"))
        cost_acados = float(self.solver.get_cost())          # ⚠️ RTI 下不可信，只留证（见 SolveInfo）

        if prediction:
            # ★ 一次 `get_flat` 取全轨迹，**不逐点 get**：同样一条轨迹实测 0.049 ms vs 0.930 ms
            #   （19×，2026-09-28 本机、绑核）。在这个预算里这不是风格问题 —— 见本包 README。
            u_pred = np.array(self.solver.get_flat("u")).reshape(self.config.N, NU)
            x_pred = np.array(self.solver.get_flat("x")).reshape(self.config.N + 1, NX)
            u_ref_arr = np.asarray(u_ref, dtype=float).reshape(self.config.N, NU)
            x_ref_arr = np.asarray(x_ref, dtype=float).reshape(self.config.N + 1, NX)
            cost = self.config.cost(x_pred, u_pred, x_ref_arr, u_ref_arr)
        else:
            u_pred = None
            x_pred = None
            cost = float("nan")                              # 没读轨迹就算不了，别拿假数糊弄（见 cost_available）

        info = SolveInfo(
            status=status,
            sqp_iter=int(self.solver.get_stats("sqp_iter")),
            qp_iter=np.array(self.solver.get_stats("qp_iter")),
            qp_stat=np.array(self.solver.get_stats("qp_stat")),
            residuals=np.array(self.solver.get_stats("residuals")),
            u_bound_violation=self._bound_violation(u_first),
            quat_norm_drift=self._quat_drift(x_pred),
            cost=cost,
            cost_acados=cost_acados,
            time_tot=time_tot,
            time_cycle=time.perf_counter() - t0,
        )
        if status == 0 and prediction:
            self._u_ref_last = u_pred
        ref = np.asarray(x_ref, dtype=float).reshape(self.config.N + 1, NX)
        return OcpSolution(u=u_first, u_pred=u_pred, x_pred=x_pred, info=info, ref=ref)

    def last_prediction(self) -> tuple[np.ndarray, np.ndarray] | None:
        """把**求解器里那份当前迭代点**的预测轨迹读出来（`None` = 还没解过）。

        ★ 它**不额外解一次**：`get_flat` 实测 0.049 ms，而一次 `solve()` 是 3.5 ms 的量级
          （本包 README 的耗时分解）。节点的 RViz 那条预测路径走这里 ⟹ 可视化不占控制预算，
          而且画出来的就是"**刚发出去的那个 `u` 所属的那条轨迹**"（同一次迭代的结果），
          所以"节点此刻认为会发生什么"与"节点刚才发了什么"永远自洽。
        ⚠️ 它读的是**求解器内部状态**，不是 `solve()` 的返回值：`prediction=False` 时
          `OcpSolution.x_pred` 是 `None`，但求解器里**仍然**有一条轨迹（RTI 每周期都会算）。
          两条路读到的可能是**不同次**迭代的结果（若中间又解过一次）——节点里是同一个定时器
          回调，不存在这个问题；跨线程调用则不然（本包目前没有第二个线程）。
        """
        if self._solves == 0:
            return None
        u = np.array(self.solver.get_flat("u")).reshape(self.config.N, NU)
        x = np.array(self.solver.get_flat("x")).reshape(self.config.N + 1, NX)
        return x, u

    # ── 自算的健康度指标（不依赖 acados 的残差记账，见 SolveInfo 的说明） ──
    def _bound_violation(self, u: np.ndarray) -> float:
        """`u` 越过 `[lbu, ubu]` 的量（0 = 没越界）。

        ★ 上下界**从求解器实际接线的那份读**（`acados_ocp.constraints`），不在这里按参数重算一遍：
        重算就是同一组边界有第二个副本，哪天 M3/M4 只改了接线处，这个健康度指标会**悄无声息地
        拿旧边界去量新解** —— 而它正是用来发现"悄悄退化"的那个读数。
        """
        lb = np.asarray(self.acados_ocp.constraints.lbu)
        ub = np.asarray(self.acados_ocp.constraints.ubu)
        return float(max(np.max(np.maximum(lb - u, 0.0)), np.max(np.maximum(u - ub, 0.0))))

    def _quat_drift(self, x_pred: np.ndarray | None) -> float:
        """预测轨迹上 `|q|² − 1` 的最大绝对偏离（`prediction=False` 时**不取轨迹**，返回 0）。"""
        if x_pred is None:
            return 0.0
        q = x_pred[:, Q_SLICE]
        return float(np.max(np.abs(np.sum(q * q, axis=1) - 1.0)))

    def _shifted_previous(self) -> np.ndarray:
        """**上一帧解平移一拍**（末尾重复）—— M6 的缺省参考量，理由见类 docstring。"""
        u = np.empty_like(self._u_ref_last)
        u[:-1] = self._u_ref_last[1:]
        u[-1] = self._u_ref_last[-1]
        return u

    # ── 方便测试与复用的小工具 ──────────────────────────────────────────
    @property
    def dt(self) -> float:
        """OCP 内部步长 [s]。"""
        return self.config.tf / self.config.N

    def hover_state(self, p: tuple[float, float, float] = (0.0, 0.0, 0.0)) -> np.ndarray:
        """悬停状态：位置 `p`、速度 0、姿态单位四元数。"""
        return np.array([*p, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0])

    def copy_with(self, **kwargs) -> "OfflineOcp":
        """换参数/配置造一个新实例（[ticket 12](../../../.scratch/phase2/issues/12-mismatch-sweep.md) 的失配扫描用）。"""
        params = kwargs.pop("params", self.params)
        config = kwargs.pop("config", self.config)
        return OfflineOcp(params=params, config=config, codegen_dir=kwargs.pop("codegen_dir", self.codegen_dir))
