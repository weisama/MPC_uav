"""节点侧的**纯逻辑**（零 ROS）：外推、闸门、停机条件、参考构造。

★ 为什么要把这些从 `mpc_node.py` 里抠出来：它们全是"给定一串数，判断该不该发、发什么"，
  **一条话题、一个定时器都不需要**，所以能在一毫秒内被单测钉死（`test/test_node_core.py`）。
  留在节点里就会变成"只能靠起 DUT 子进程 + 假 PX4 才能测"——那些测试慢、且失败时
  分不清是逻辑错了还是 DDS 没发现。

★ 本模块与 `ocp.py` 的分界：这里**不碰求解器**（不 import acados），只负责
  "喂给求解器的初值与参考"，以及"这一拍到底该不该发"。求解与符号映射分别是
  `ocp.py` 与 `sign_map.py` 的事。

★★ **两档状态源**（ticket 09 的 A 档）：`state_source` 选**测量从哪里来**，其余逻辑一模一样。

| 档 | 测量 | 用在 |
|---|---|---|
| `mocap`（默认） | `/mocap/pose_enu`（**原生 ENU/FLU**） | Phase 2 的 B 档（动捕闭环） |
| `px4` | `/fmu/out/vehicle_local_position` + `vehicle_attitude`（**NED/FRD**） | A 档（MPC（模型预测控制）读 PX4 自己的状态） |

A 档的价值**不在"多一条输入"**：它是"**我们的约定与真 PX4 一致**"这个问题第一次被真正回答
（票面原文）—— 04 只验自洽、06 只验"映射表被实现对了"，只有状态**从 PX4 那头进来**，
整条链（NED/FRD → ENU/FLU → 模型 → 符号映射 → 话题）才第一次闭合。
⚠️ 所以这两档**不许共用一条消息类型**：动捕那条是 `PoseStamped`（本来就是 ENU/FLU），
  PX4 那两条是 NED/FRD 的裸字段 —— 转换**只写一处**（`px4_pose_to_enu`，见那里的 ★★）。

★ 停机条件**没有任何通道号**（票面验收项「软急停接口参数化」）。理由是第一手的：
  [`.scratch/phase1/issues/02`](../../../.scratch/phase1/issues/02-rc-channel-decision.md)
  已 resolved —— **急停 = ch8 高值 = PX4 硬件杀车，程序不读急停通道**，也就是**不存在**
  "软急停通道"。程序侧能观察到的只有两条既有路径（`phase2-implementation.md` §4 S8 那张表）：
  **打杆接管**（RC 一动，PX4 自己离开 OFFBOARD）与 **ch6 模式开关**（切模式）。
  两者在节点这一侧**长得一样** —— 都表现为 `vehicle_status.nav_state` 不再是 OFFBOARD
  ⟹ 本模块只认状态，不认通道。

★ 本模块还负责**失败路径**（ticket 07 / M5）的三块纯逻辑，各有第一手理由：
  1. **输入合理性**（`stamp_regress` / `pose_jump` / `attitude_jump`）：动捕会给出**不可信但看着正常**
     的一帧（重放、刚体认错、遮挡恢复）⟹ 这一拍**不发**比发一条错的强（`_hold()` 那侧的闸门）。
  2. **求解失败的兜底阶梯**（`FallbackLadder`）：一拍的解不可用时，先**保持上一帧可行指令**，
     连续失败到阈值再切**安全悬停**（悬停推力 + 零角速率）。★ 之所以不能一直"保持"：
     PX4 会**无限期锁存**最后收到的 setpoint（`MulticopterRateControl.cpp:177-184`，见
     `docs/control/mpc-node-fallback.md` 的第一手引用）——"不发"与"一直沿用最后一帧"在飞控侧
     是同一件事，所以陈旧命令必须由**我们**主动换掉。
  3. **发布前的整帧有限性校验**（`frame_is_finite`）：PX4 对三个角速率分量有 NaN 回退
     （退回实测角速率），**推力没有** —— 一个 NaN 推力会直达混控器。
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Optional, Tuple

import numpy as np

from .model import NX, NU, normalize_quat, quat_angle, quat_mul, yaw_quat
from .params import Z0Params

__all__ = [
    "TAU_DEFAULT_S", "WARMUP_CYCLES", "FALLBACK_HOLD_FRAMES", "JUMP_DT_FLOOR_S",
    "ARMING_STATE_ARMED", "NAV_STATE_OFFBOARD",
    "STATE_SOURCE_MOCAP", "STATE_SOURCE_PX4", "STATE_SOURCES", "SAT_TOL_REL",
    "StopConfig", "ControlConfig", "Cycle", "NodeCore", "FallbackLadder",
    "extrapolate_state", "hold_reference", "yaw_from_quat", "pose_to_state",
    "mocap_stamp_s", "px4_stamp_s", "px4_vector_to_enu", "px4_quat_to_enu",
    "px4_pose_to_enu", "saturated_bounds", "frame_is_finite", "solution_usable",
]

#: 状态外推的时间常数 τ [s] —— 票面 [06](../../../.scratch/phase2/issues/06-mpc-node-fake-px4-fixture.md)
#: 写死 **50 ms**（= 一个 20 Hz 的闭环延迟量级，也正好是 OCP 内部步长 `tf/N = 1.0/50`）。
#: ★ 它的含义是"这一拍算出来的 `u` 真正作用到机体上时，状态已经往前走了 τ"，
#: 所以喂给求解器的初值是 `x + v·τ`。**只外推位置**（速度/姿态的导数没有可靠的测量来源）。
#: ⚠️ **上飞机之前要确认这一档**：[`phase2-implementation.md`](../../../docs/control/phase2-implementation.md)
#: §4 的 **D9** 写的是"**实飞起始值仍取 `0`**（过补偿比不补偿更危险）"，τ ≈ 50 ms 只是量级
#: 与起点参照（ticket 03 实测）—— 那条状态是 **🔶 仍开放**、裁定人是操作者。
#: `-p tau_s:=0` 就是 D9 那一档，改一个参数即可，**代码不用动**。
TAU_DEFAULT_S = 0.05

#: 上电后先解多少次再发第一帧。★ 这不是保守，是**第一手结论**：
#: `SQP_RTI` 每周期只做**一次**迭代 ⟹ 全新实例的第 0 次 `solve()` 给出的**不是**最优解，
#: 而是"从出厂初始迭代点走一步"的结果。实测（`test/conftest.py` 的模块 docstring）：
#: 目标 +x 10 m 时第 0 次 `ω_y = 0.0000、F = 7.8111`（与悬停一模一样），第 1 次才 `ω_y` 打满。
#: 节点若把第 0 拍发出去，等于**第一帧朝悬停姿态走**（在最需要它动的时刻）。
WARMUP_CYCLES = 5

#: 求解失败之后先"保持上一帧可行指令"几拍，再切安全悬停（ticket 07 / M5）。
#: ★ 这个数的量级不是随手取的：一次求解的预算是 3.5 ms（ticket 04 实测），100 Hz 下一拍 10 ms
#:   ⟹ **10 拍 = 100 ms ≈ 28 个求解预算**。比它短，"偶发一次抖动就换姿态"太敏感；
#:   比它长，等于任由一条**陈旧命令**（可能是打满的角速率）继续作用在飞机上 ——
#:   而这条命令在飞控侧是**不会被超时清掉**的（见模块 docstring 第 2 条）。
#: ⚠️ 参数化（`ControlConfig.fallback_hold_frames`）：0 = 不保持、第一次失败就直接切安全悬停。
FALLBACK_HOLD_FRAMES = 10

#: 位姿合理性判据里 `dt` 的**下限** [s]。★ 它不是"保险余量"，是把判据变成**频率无关**的那一步：
#:   动捕 200 Hz 时两帧只隔 5 ms，而 DDS 投递抖动会让**到达时刻**差出好几毫秒 ⟹ 直接用实测
#:   `dt` 会让"允许位移"在抖动的那一帧缩到 1/5，把 12 m/s 的真实机动误判成跳变（一个**会让
#:   飞机无故停发**的假阳性）。取一个 100 Hz 控制周期做下限，等价于"至少按 10 ms 的机动量算"。
JUMP_DT_FLOOR_S = 0.01

#: PX4 的枚举（`px4_msgs/msg/VehicleStatus.msg` 逐字一致）。
#: ⚠️ 这里**故意只抄两个**（节点只用这两个）：把整张表抄进来就会有两份枚举，
#: 而"两份枚举"正是这一类 bug 的温床。
NAV_STATE_OFFBOARD = 14
ARMING_STATE_ARMED = 2

#: 状态源的名字（`-p state_source:=…`，见模块 docstring 的两档表）。
#: ⚠️ 写成一个**封闭集合**：参数层面写错一个字母（`px4_lp` / `PX4`）必须**启动即退**，
#: 否则它会被当成"等于 mocap"而静静地跑另一条路 —— 那种错在话题上看起来一模一样。
STATE_SOURCE_MOCAP = "mocap"
STATE_SOURCE_PX4 = "px4"
STATE_SOURCES = (STATE_SOURCE_MOCAP, STATE_SOURCE_PX4)

#: 判"顶在约束上"（饱和标志）的**相对**容差。★ 它只是**日志标志**的阈值，不是正确性判据
#: （正确性那条是 `SolveInfo.u_bound_violation`，用生产阈值判越界）。
#: 取 1‰ 的理由是第一手的：QP 给出的"打满"是 `3.8399998` 而不是 `3.84`（实测，见
#: `test_ocp_constraints.py` 的容差纪律）⟹ 用等号判饱和会**漏报**，而漏报的标志比没有标志更坏
#: （复盘时会以为"没打满"，于是去找别的根因）。
SAT_TOL_REL = 1e-3


# ───────────────────────────── 停机条件 ─────────────────────────────

@dataclass(frozen=True)
class StopConfig:
    """什么时候**停止发布**。全部是参数，**一个通道号都没有**（见模块 docstring）。

    ★ 每个 `timeout` 的量级**必须与上游频率匹配**（照抄 Phase 1 的教训）：
      · `/mocap/pose_enu` 实测 ~196 Hz ⟹ 0.5 s 是"明显漏了"的量级
      · `/fmu/out/vehicle_status` 是 **2 Hz 定频**（`Commander.cpp:1790`）⟹ 0.5 s
        只有**一个**周期，任意一次抖动都会误判。真 PX4 在这条流上的心跳是 1 s
        （`COM_OF_LOSS_T`，它自己也会因此退出 OFFBOARD）⟹ 本节点取 **0.6 s** 起，
        默认 `0.75`：小于 PX4 自己的容忍度，才不会"PX4 觉得还新鲜、我们先停了"。
      · `vehicle_local_position` / `vehicle_attitude` 实测 **32.5 / 31.6 Hz**（2026-09-28，
        z0 SITL，A 档探针）⟹ 一拍 31 ms，0.5 s = **16 拍**：与动捕那条同一个量级，
        也远小于 PX4 自己的失效判据。
    """

    mocap_timeout_s: float = 0.5
    #: PX4 那条状态源的龄上限 [s]（位置与姿态**各自**判 —— 它们是两条消息）
    px4_timeout_s: float = 0.5
    status_timeout_s: float = 0.75
    #: 离开 OFFBOARD 就停（打杆接管 / ch6 切模式在节点这一侧的**唯一**表现，见模块 docstring）
    stop_on_nav_state_exit: bool = True
    #: 掉出 armed 就停（含"还没解锁"——那种情况下本来也不该发）
    stop_on_armed_drop: bool = True
    #: 收到 armed + OFFBOARD 之后先解这么多次，**不发**（见 `WARMUP_CYCLES`）
    warmup_cycles: int = WARMUP_CYCLES

    # ── 输入合理性（ticket 07 / M5）：这三条拦的是"**看着正常**的一帧" ——
    #    超时那几条（上面两个）拦的是"没有数据"，拦不住"数据是错的"。
    #    ★ ticket 09：这三条**与状态源无关**（A 档下同样要拦 EKF 复位/认错给出的瞬移），
    #      只是名字里带着 `mocap`—— 它们是 ticket 07 收口时的命名，改名会牵动 `preflight_check`
    #      的两条串位断言、节点参数表与三份文档；**留着名字、在这里说清语义**比一次改名划算。
    #      每档专属的那两条超时（`mocap_timeout_s` / `px4_timeout_s`）倒是各叫各的。
    #: 允许的动捕 stamp **倒退**量 [s]（超过它 ⟹ `stamp_regress`）。
    #: ★ 不为 0 是因为"相等/抖动"必须放行：节点 100 Hz 取一拍而动捕 200 Hz，
    #:   同一个 stamp 会被读到两次；DDS 也真的会给出非单调的到达顺序。
    mocap_stamp_regress_s: float = 0.01
    #: 位姿跳变判据：两个**相邻接受帧**之间允许的位移速度 / 角速度上限。
    #: ★ 默认值的依据：`MPC_XY_VEL_MAX = 12 m/s`（[`params-analysis.md`](../../../docs/reference/params-analysis.md)，
    #:   z0 飞控现状）⟹ 位移上限取 **20 m/s**（1.7×，给真实机动留出余量 —— 判据宁可漏判，
    #:   也不能把一次正常机动当成跳变而**无故停发**）；角速度上限取 **2× `YAW_RATE_MAX`**
    #:   （6.28 ⟹ 12.6 rad/s），同理。
    #: ⚠️ 想临时关掉这两条判据（SITL 排查用）：给一个极大的值即可，不必加开关。
    mocap_jump_speed_max: float = 20.0
    mocap_jump_rate_max: float = 12.6


@dataclass(frozen=True)
class ControlConfig:
    """节点控制律侧的配置（唯一一份 —— CLI 参数、`Z0Params`、`OcpConfig` 都从这里接出去）。"""

    tau_s: float = TAU_DEFAULT_S
    #: 这一趟的**测量从哪里来**（两档见模块 docstring）。取值必须是 `STATE_SOURCES` 里的一个，
    #: 校验在节点侧（`mpc_node._validate`）—— 本模块只按它分支。
    state_source: str = STATE_SOURCE_MOCAP
    #: 参考点相对**位置保持锚点**的偏移 [m]（ENU）。默认 (0,0,0) = 原地位置保持。
    #: ★★ 锚点是**闸门第一次打开那一拍**的测量，之后不随测量移动（`NodeCore._update_anchor`）
    #:   ⟹ 有了它，`hold_offset` 才是一个**绝对**目标（"从交班点起往上 1 m"），位置误差才关得掉。
    #: ⚠️ 它曾经是"相对**当拍测量**的偏移"，那时这一项恒等于误差、闭环没有不动点可言 ——
    #:   第一手证据（0.9 m/s 稳定爬升 37 s）见 `hold_reference` 的 ★★。
    #: ★ 真正的轨迹接口（`/traj/reference` 订阅）仍是**未做项**（票 06 的账本判给了 ticket 09，
    #:   而 ticket 09 选的是"锚点"这条最小路径 —— 归因与取舍记在
    #:   `docs/control/tier-a-sitl-hover.md`）。
    hold_offset: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    #: 参考的机头（rad，ENU 从 +x 逆时针）。`None` = 保持**锚点那一拍**的机头。
    #: ★ 与位置同理：机头也是"交班时朝哪就保持朝哪"，不是一个每拍更新的量。
    hold_yaw: Optional[float] = None
    #: 求解不可用之后**先保持几拍**上一帧可行指令，再切安全悬停（`FallbackLadder`，见常量）。
    fallback_hold_frames: int = FALLBACK_HOLD_FRAMES
    stop: StopConfig = field(default_factory=StopConfig)


# ───────────────────────────── 纯函数 ─────────────────────────────

def yaw_from_quat(q) -> float:
    """从 `[w, x, y, z]`（ENU/FLU）取出偏航角 [rad]，从 +x 逆时针为正。

    与 `model.yaw_quat` 互为逆运算（`test_node_core.py` 里用往返对账盯着这一点）。
    ⚠️ 用 `atan2` 而不是 `asin(2(wy − zx))`：后者在 ±90° 附近退化，而我们要的是
    "机头朝哪"这一个连续量。
    """
    w, x, y, z = (float(v) for v in q)
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def pose_to_state(position, orientation, velocity=(0.0, 0.0, 0.0)) -> np.ndarray:
    """`(位置, 姿态四元数, 速度)`（**ENU/FLU**）→ 10 维状态 `[p(3), v(3), q(4)]`。

    ★ 这里**没有坐标系转换**，这正是 M7 要的结果：动捕原生就是 ENU/FLU、模型内部也是
      ENU/FLU ⟹ 参考实现在这一侧写的那段手写 NED→ENU 四元数变换（整个文件里最容易写错的一行）
      在 Phase 2 里**不存在**。
    ★ A 档（ticket 09）要接的 PX4 话题**是 NED/FRD** ⟹ 那段转换现在有了，就在上面那三个
      `px4_*` 函数里，**只有那一处**（它们把 NED/FRD 先变成 ENU/FLU，再进本函数）——
      否则"符号错了"与"模型解错了"再也分不开。
    """
    q = normalize_quat(np.asarray(orientation, dtype=float).reshape(4))
    return np.concatenate([np.asarray(position, dtype=float).reshape(3),
                           np.asarray(velocity, dtype=float).reshape(3), q])


# ─────────────────── PX4 的 NED/FRD → 我们的 ENU/FLU（**唯一一处**）───────────────────
#
# ★★ 为什么集中在这里、且只做**分量级**的换标签：参考实现在这一侧写的是手推的四元数分量
#    变换（`px4_mpc` 里那段整个文件最容易写错的地方）。我们的做法是把"两个坐标系互换"
#    写成**两个 180° 旋转**的复合 —— 每个都能用一句话说清、且能用旋转矩阵恒等式对账
#    （`test_node_core.py::test_px4_姿态转换满足旋转矩阵恒等式`），不必手推分量。
#
# 数值权威仍是 `docs/control/sign-map.md`（§2 的表 + §8 的代码契约）—— 本文件不复述它的数值。

#: NED→ENU 那个换标签：绕 `(1,1,0)/√2` 转 180°，即 `(x, y, z) → (y, x, −z)`（w-first 四元数）。
#: 实部为 0：180° 旋转总是这样。
Q_NED_TO_ENU = np.array([0.0, math.sqrt(0.5), math.sqrt(0.5), 0.0])
#: FRD→FLU 那个换标签：绕机体 x 转 180°，即 `(x, y, z) → (x, −y, −z)`。
#: ★ 这正是 `sign_map` 对**角速率**做的同一件事（`roll=ωx、pitch=−ωy、yaw=−ωz`）——
#:   同一个换标签出现在两处**不是重复**：那边是"把我们的 FLU 角速率写进 PX4 的 FRD 字段"，
#:   这里是"把 PX4 的 FRD 姿态读成我们的 FLU 姿态"。两边的 `M` 必须一致，
#:   所以 `test_node_core.py` 里有一条把两者对起来的断言。
Q_FRD_TO_FLU = np.array([0.0, 1.0, 0.0, 0.0])


def px4_vector_to_enu(v_ned) -> Tuple[float, float, float]:
    """NED 向量（位置/速度同用）→ ENU 分量：`(x, y, z) → (y, x, −z)`。

    物理锚点（调符号时先问这两个）：NED 的 `(1,0,0)`（北）是 ENU 的 `(0,1,0)`（+y）；
    NED 的 `(0,0,1)`（下）是 ENU 的 `(0,0,−1)`（−z）。
    """
    x, y, z = (float(c) for c in v_ned)
    return (y, x, -z)


def px4_quat_to_enu(q_ned_frd) -> np.ndarray:
    """「FRD 机体 → NED 世界」的四元数 → 「FLU 机体 → ENU 世界」的四元数。

    `q_flu = q_L ⊗ q_frd ⊗ q_M`，其中 `q_L` = `Q_NED_TO_ENU`、`q_M` = `Q_FRD_TO_FLU` ⟹
    旋转矩阵上就是 `R_enu = L · R_ned · M`（世界系换过标签、机体系也换过标签）。

    ⚠️ **它不是"把 pitch/yaw 取反"**：那是 `sign_map` 对**角速率向量**做的事（同一个 `M`，
      但只换了机体系那一半）。照抄那个写法会得到"姿态整体偏了 90°"——症状是
      "飞机一进 OFFBOARD 就朝一边歪"，而符号表看着完全正确。两个错**都要用矩阵恒等式抓**。
    """
    q = np.asarray(q_ned_frd, dtype=float).reshape(4)
    return quat_mul(quat_mul(Q_NED_TO_ENU, q), Q_FRD_TO_FLU)


def px4_pose_to_enu(position_ned, q_ned_frd, velocity_ned=(0.0, 0.0, 0.0)) -> tuple:
    """`(NED 位置, FRD→NED 四元数, NED 速度)` → `(ENU 位置, FLU 四元数, ENU 速度)`。

    输入的名字与语义**照抄 msg**（`VehicleLocalPosition.msg`：`# Position in local NED frame`；
    `VehicleAttitude.msg`：`# Quaternion rotation from the FRD body frame to the NED earth frame`）
    —— 不 import `px4_msgs`（本模块的契约），所以这里靠调用方按字段名传进来。
    返回的三样正好是 `pose_to_state` 的入参，中间不落地任何变量。
    """
    return (px4_vector_to_enu(position_ned),
            px4_quat_to_enu(q_ned_frd),
            px4_vector_to_enu(velocity_ned))


def px4_stamp_s(msg) -> Optional[float]:
    """PX4 那条状态消息的 `timestamp`（**微秒**）→ 秒；读不出来/为 0 ⟹ `None`。

    ★ 它**不是墙钟**（从飞控上电起算，DDS 边界上还被加过 `time_offset`）——但这里只做
      **同一路源的前后两帧自比**（判时间戳倒退、算跳变判据的 `dt`）⟹ 纪元无关紧要。
      这与 `mocap_stamp_s` 是同一条纪律，"新鲜度不用消息里的 stamp"那条不受影响。
    ⚠️ `timestamp == 0` 也返回 `None`：PX4 的反序列化器把 0 改写成 `hrt`，所以在话题面上
      "0"只可能是损坏的一帧（与 `mocap_stamp_s` 对 0 的处理同一取向：宁可判"没有时间戳"，
      也不判"1970 年"）。
    """
    ts = getattr(msg, "timestamp", None)
    if ts is None:
        return None
    s = float(ts) * 1e-6
    return s if s > 0.0 else None


def saturated_bounds(u, params: Z0Params) -> Tuple[str, ...]:
    """`u = [F, ωx, ωy, ωz]` **顶在哪几条约束上** → 名字元组（空元组 = 一条都没顶）。

    名字与 OCP 的 `lbu/ubu` 一一对应（`ocp.py`：`lbu = [0, −rate_max]`、`ubu = [f_max, rate_max]`）
    ⟹ **判据用的是生产阈值**（`Z0Params`），不是另抄一份数。
    ⚠️ `F_max` 比的是 **`f_max`（按电池）**，不是 `t_max`：`t_max` 是归一化常数（46.3 N），
    拿它比会让"顶到电池极限"这一档**永远不报**（假绿）—— 2026-09-29 之前就是这样。

    ★ 它是**日志面**：票面要"约束饱和标志"，而"它在顶约束"与"它在正常工作"必须分得开 ——
      节点不做任何裁剪（见 `mpc_node` 的「饱和表现为打满」），所以这里看到的就是求解器真的
      把 `u` 推到了边界上。反过来，`u` 只是在边界附近（差 > 1‰）**不算**饱和。
    """
    f, wx, wy, wz = (float(v) for v in u)
    out = []
    # 下界就是 0 ⟹ 相对容差在这里退化成绝对容差（写成 `max(1.0, abs(0.0))` 只是个
    # 照抄上面那条的模板，读的人会以为下界可配 —— 它不可配，`lbu[0]` 写死是 0）
    if abs(f) <= SAT_TOL_REL:
        out.append("F_min")
    elif abs(f - params.f_max) <= SAT_TOL_REL * max(1.0, abs(params.f_max)):
        out.append("F_max")
    for name, w, w_max in (("wx", wx, params.roll_rate_max),
                           ("wy", wy, params.pitch_rate_max),
                           ("wz", wz, params.yaw_rate_max)):
        if min(abs(w - w_max), abs(w + w_max)) <= SAT_TOL_REL * max(1.0, abs(w_max)):
            out.append(name)
    return tuple(out)


def mocap_stamp_s(msg) -> Optional[float]:
    """从 `/mocap/pose_enu` 的 `header.stamp` 取**秒**（float）；读不出来 ⟹ `None`。

    ★ 三种"读不出来"都返回 `None`，**都不报错**（与 `twist` 那条同一条纪律：缺字段不许崩）：
      · 没有 `header`（本仓的纯逻辑测试替身就是这种形状）；· `stamp` 的 `sec/nanosec` 不齐；
      · `stamp` 是 0（**源没填**，不是"1970 年"）—— `0 ≤ 基线` 会被读成"倒退"，那是误判。
    ⚠️ 与 `mpc_node._now_s()` 那条纪律**不冲突**：新鲜度仍然用**到达时刻**（PX4 的
      `hrt_absolute_time()` 从飞控上电起算，跨纪元相减无意义）；这里用 stamp 只做一件事 ——
      **同一路源的前后两帧自比**，比的是"这一路源自己说的话是不是在往回走"。
    """
    header = getattr(msg, "header", None)
    stamp = getattr(header, "stamp", None)
    sec = getattr(stamp, "sec", None)
    nanosec = getattr(stamp, "nanosec", None)
    if sec is None or nanosec is None:
        return None
    s = float(sec) + float(nanosec) * 1e-9
    return s if s > 0.0 else None


def frame_is_finite(sp) -> bool:
    """**整帧**有限性校验（票面验收项，第一手理由见模块 docstring 第 3 条）。

    ★ 校验的是会变的那**四个**数：`roll / pitch / yaw / thrust_body_z`。
      消息里另外两个分量（`thrust_body[0..2]` 的横向两个）是 `mpc_node._rates_msg` 里
      **写死 0.0 的字面量** —— "全量"这两个字容易被读成"包括那两个 0"，故在此写明。
    ⚠️ 它与 `sign_map.px4_rates_setpoint` 的入参检查**故意重复**：`sign_map` 守的是
      "**映射这条路**上的量"（`ω` / `F` / `T_max`），这里守的是"**凡是要写进那一帧的数**"。
      判据 2 的字面要求是**发布前**过一遍，而发布点只有一个 —— 这一道跟着那个点走，
      将来谁在帧里加了字段（比如换个话题），它仍然挡在前面。
    """
    try:
        values = [sp.roll, sp.pitch, sp.yaw, sp.thrust_body_z]
    except AttributeError:  # pragma: no cover - 形状不对时**拒发**，不猜
        return False
    return all(math.isfinite(float(v)) for v in values)


def solution_usable(sol) -> Optional[str]:
    """一次求解是否**可用**：`None` = 可用；否则返回一句人话原因（进日志）。

    ★ "可用"与"退化"（`SolveInfo.degraded`）**不是一件事**，这一条是 ticket 06 交接时
      点名的陷阱，别合并：
      · **退化**（状态码非零 / `u` 越出约束 / 代价或耗时非有限）⟹ **解仍然有限**，
        06 的裁定是**照样发** + `warn` + 计数；
      · **不可用**（下面两条）⟹ 解不能进话题，走 `FallbackLadder` 的兜底。
    ⚠️ 形状那一条不是洁癖：`u` 的排布是 `[F, ωx, ωy, ωz]`，若哪天它变成 3 维，
      `u[1:]`/`u[0]` 仍然"能跑"而**发出去的东西是错的**（`sign_map` 那边也拦不住）。
    """
    info = getattr(sol, "info", None)
    if info is None or not getattr(info, "ok", False):
        line = info.line() if hasattr(info, "line") else str(info)
        return f"求解器状态码异常（{line}）"
    u = np.asarray(sol.u, dtype=float)
    if u.shape != (NU,):
        return f"解的形状不是 ({NU},)：{u.shape}"
    if not np.isfinite(u).all():
        return "解里含非有限值（NaN/Inf）"
    return None


def extrapolate_state(x: np.ndarray, tau_s: float = TAU_DEFAULT_S) -> np.ndarray:
    """**状态外推 `x + v·τ`** —— 票面要求的"一行实现"，不做完整延迟补偿。

    只动位置：`p ← p + v·τ`。速度与姿态原样带走。
    理由：位置与速度都是量测（速度的来源见模块里 `velocity` 那几行的说明），而加速度要再差分一次，
    噪声会把外推变成一个抖动放大器；τ 只有 50 ms，省下的高阶项远小于噪声。
    ⚠️ 外推的是**初值**，参考点**不外推**（参考是"要到哪里"，不是"现在在哪"）——
      两者混在一起，外推就变成"把目标也搬走"，修正量会恰好抵消掉。
    """
    x = np.asarray(x, dtype=float).reshape(NX)
    out = x.copy()
    out[:3] = x[:3] + x[3:6] * float(tau_s)
    return out


def hold_reference(x_ref_pose: np.ndarray, config: ControlConfig, N: int) -> np.ndarray:
    """构造 `(N+1, NX)` 的位置保持参考：位置 = **锚点位置** + `hold_offset`，速度 0，姿态水平。

    ★ 姿态取**水平**（roll = pitch = 0）而非照抄测量姿态：悬停参考就该是水平的，
      而 OCP 的代价对四元数只罚**倾斜**（`qw` 权重是 0，见 `OcpConfig.q_diag`）⟹
      水平 + 当前机头才是"不倾斜、朝向不变"的那个目标。

    ★★ `x_ref_pose` 是**锚点**（`NodeCore` 在闸门第一次打开那一拍钉住的那一帧测量，
      见 `NodeCore._update_anchor`），**不是当拍的测量** —— 这是本函数最容易用错的地方，
      而且用错了**不报错**，只会让飞机一直追一个永远追不上的点。

      第一手（2026-09-28，z0 同构 SITL 首飞，`docs/control/tier-a-sitl-hover.md`）：
      当初这一行写的是 `hold_reference(x_meas, …)`（每拍用当拍测量重建参考），于是
      `x_ref = x_meas + hold_offset` 恒成立 ⟹ 位置误差**恒等于** `hold_offset`、
      一拍都不收敛。实测 `hold_offset_z = 1 m` 的那一场：飞机以 **0.9 m/s 稳定爬升 37 s**
      （EKF 帧抬升 30.9 m），日志里 `x0[2] − x_ref0[2]` 从第 0 帧到第 3740 帧**一次没关小**，
      停在 −0.955 m = `v·τ − 1` —— 位置项要往上、速度项要往下，稳态就落在那条匀速直线上。
      锚点定住之后这个差值才会随"飞机走到哪"而关小，位置环才真的有工作可做。
    """
    x = np.asarray(x_ref_pose, dtype=float).reshape(NX)
    yaw = config.hold_yaw if config.hold_yaw is not None else yaw_from_quat(x[6:10])
    target = np.concatenate([
        x[:3] + np.asarray(config.hold_offset, dtype=float),
        np.zeros(3),
        yaw_quat(yaw),
    ])
    return np.tile(target, (N + 1, 1))


def hold_feedforward(params: Z0Params, N: int) -> np.ndarray:
    """静止保持参考的**解析前馈** `(N, NU)` —— 使 `f(x_ref, u_ref) = 0`（"参考自己就是模型的一条解"）的那个 `u`。

    | 通道 | 取值 | 依据 |
    |---|---|---|
    | 推力 | `params.t_hover`（`m·g / T_max`） | 悬停配平：升力 = 重力，归一化后就是悬停油门 |
    | 三个角速率 | 0 | 参考是**静止**的（位置/速度/姿态的导数都为 0）⟹ 角速率必须为 0 |

    ★ 它取代了 `mpc_node` 里那句"上一帧解铺满时域"。为什么必须换：代价项是 `R·(u − u_ref)`，
      把**上一帧解**当参考量 ⟹ 解出一点偏、下一帧的参考量就跟着偏，是一条**正反馈**。
      `offline-ocp.md` §4.4 离线就记过"单调上爬、不收敛"（静态参考 + 缺省 `u_ref`：`ω_y`
      连解 12 次从 0.3225 爬到 1.3487 rad/s）；ticket 09 在**真 PX4 闭环**里复现并放大成
      runaway —— 那一场 `u[3]` 从 0 单调爬到 **+4.696 rad/s**、EKF 帧累计转了 **13.6 圈**
      （第一手：`docs/control/tier-a-sitl-hover.md`）。
      **最快的读数在台架上**（`test_mpc_node.py::test_前馈取参考的解析前馈_不跟着解爬`，
      2026-09-28 实测）：给一条 `hold_offset_x = 10 m` 的任务，只到**第 9 拍**控制参考就爬到
      `[11.80 N, 3.84, 0.0005, −0.0006]` —— 推力那一路先撞**上限**（这次实测时上限恰好是
      `T_max`，2026-09-29 解耦后那个角色归 `f_max`，两者都是 11.8 N ⟹ 读数不受影响）、横滚那一路
      撞上 3.84 rad/s，而**偏航那一路也开始漏**（任务是纯 +x 的，它本不该有偏航指令）。

    ★ 为什么这样**不**违反 `ocp.py` 里那句"不能悄悄退回悬停前馈"：那句禁的是 `solve()` 的
      **缺省**路径静默退化（调用方忘了传 `u_ref` 就偷偷换成悬停配平）。这里走的是 M6 §3.2
      的**第二行**——"调用方给的前馈（轨迹的解析前馈）"：我们的参考是静止锚点，
      它的解析前馈**就是** `[T_hover, 0, 0, 0]`，一行不差，而且是**显式**给的。

    ⚠️ 将来接**运动**参考（Phase 3 的轨迹）时，这个函数要按那条参考**自己的导数**算
      （加速度/角速率前馈），不能继续返回悬停配平 —— 否则 M6 §3.3 的实测（持续角速率
      被压成 0）会原样复发。那时候 `hold_reference` 也要一起换。
    """
    return np.tile(np.array([params.t_hover, 0.0, 0.0, 0.0], dtype=float), (N, 1))


# ───────────────────────────── 闸门 ─────────────────────────────

def _num_json(v) -> Optional[float]:
    """数 → JSON 可表示的值（非有限 ⟹ `None`）。

    ★ 那一条**龄**那几列会真用上（`age` / `age_state_s` …）：`age()` 在"一次都没收到过"
      时返回 `inf`（见 `NodeCore.age`），而 `json.dumps` 会把它写成 **`Infinity`** ——
      不是合法 JSON（`json.loads` 能读、别的解析器不能），而这些行是给**飞行后复盘**用的。
      写成 `null`，语义也正好是"没有"。
    """
    f = float(v)
    return round(f, 4) if math.isfinite(f) else None

@dataclass(frozen=True)
class Cycle:
    """一拍决策的**完整结果** —— 节点拿到它之后只剩"求解 + 映射 + 发布"三件机械的事。

    `publish=False` 时 `reason` 说清是哪一条闸门拦下的（要打在日志/证据里：
    失败时"为什么没发"是第一个要回答的问题）；`x0` / `x_ref` 仍会给出，
    因为**即使不发也要继续求解**（不然闸门一开，第一拍又是"出厂迭代点"，见 `WARMUP_CYCLES`）。
    """

    publish: bool
    reason: str
    #: 外推后的初值 / 参考 —— 闸门关着时**全是 None**（没有可信状态就不喂求解器）
    #: ⚠️ 这里**不放**测量位姿：它与 `x0[:3]` 只差一个 `v·τ`，而全仓没有任何读者
    #:   （ticket 06 的复核抓到过一次——M9 那件事的同型：定义了却没用的字段）。
    x0: Optional[np.ndarray]
    x_ref: Optional[np.ndarray]
    age_status_s: float
    #: **这一路状态源**的龄 [s]（`state_source` 决定是哪一路）。★ 名字是 `age_state_s` 而不是
    #: `age_mocap_s`（ticket 09）：A 档下这里装的是 PX4 两条消息里**较旧的那条的**龄，
    #: 两条各自的龄在 `NodeCore.describe_state()` 里分开取证（那里才是复盘时要看的）。
    age_state_s: float
    #: 闸门打开之后已经解了几拍（`warmup_cycles` 那一段的进度，给日志用）
    cycles_since_gate: int
    #: 拦下这一拍的那条判据的**读数**（如"位移 100.0 m > 允许 0.20 m"），只进日志。
    #: ★ 单有 `reason` 时，"pose_jump" 这四个字没法回答"差多少才算跳变"——而调这一档
    #:   阈值的人第一个要问的就是这个数。
    detail: str = ""
    #: 这一拍对**位置保持锚点**做了什么（`""` / `"anchor"` / `"reanchor_xy"` / …）。
    #: ★ 只在发生的那一拍非空 ⟹ 逐帧日志里一眼能看出"目标什么时候被搬过"（ticket 09）。
    #:   留默认值是为了不破坏既有构造点：绝大多数拍什么都不做。
    anchor: str = ""


class NodeCore:
    """把"话题上收到的东西"翻译成"这一拍该不该发、发什么"。

    ★ 纯逻辑、无 ROS、**无求解器**：`cycle()` 只回答该不该发 + 给出初值/参考；
      真正的 `solve()` 在节点里（它要 acados）。这样"闸门逻辑"能在一毫秒内被单测钉死。
    ★ 它**不做任何时间戳算术**：新鲜度用的是**到达时刻**（`Scenario`/节点的单调时钟），
      不是消息里的 `stamp`。理由是第一手的：PX4 的 `hrt_absolute_time()` 从**飞控上电**
      起算，与我们的墙钟 µs 差一个纪元 ⟹ 拿两个纪元做减法是错的（假 PX4 夹具
      `fake_px4.py` 就是把这条性质照抄过来的）。
    """

    def __init__(self, params: Z0Params, config: ControlConfig, horizon: int):
        self.params = params
        self.config = config
        self.horizon = int(horizon)

        self.status = None              # 最近一帧 vehicle_status（鸭子类型，不 import px4_msgs）
        self.mocap = None               # 最近一帧 /mocap/pose_enu
        self.status_at_s: Optional[float] = None
        self.mocap_at_s: Optional[float] = None
        self.status_frames = 0
        self.mocap_frames = 0
        # PX4 那一档的两个输入（**两条消息**，各自计时、各自计数 —— 见 `_measure_px4`）。
        # ⚠️ 两档的槽位同时存在、互不干扰：`state_source` 决定 `cycle()` 读哪一对。
        #    不共用一个 `state` 槽位是刻意的 —— 共用会让"这一拍用的是谁"变成隐式的，
        #    而两档的**语义完全不同**（一个原生 ENU/FLU，一个要先换标签）。
        self.local_position = None      # 最近一帧 /fmu/out/vehicle_local_position（NED）
        self.attitude = None            # 最近一帧 /fmu/out/vehicle_attitude（FRD→NED）
        self.local_position_at_s: Optional[float] = None
        self.attitude_at_s: Optional[float] = None
        self.local_position_frames = 0
        self.attitude_frames = 0
        self._gate_open = False         # 上一拍闸门是不是开着的（用来数 warmup）
        self._cycles_since_gate = 0     # 闸门打开后已经解了几拍

        # —— 输入合理性判据的基准（ticket 07 / M5）——
        # ★ 只有**被接受**的帧才更新它们：被拒的那一帧若把基准搬走，"跳变来的那一帧"立刻
        #   变成新基准、闸门下一拍就开了 ⟹ 判据等于不存在。
        # ★ 反过来，它们**不随闸门关闭清零**：一次持续的重放/认错要拒绝到底，直到源自己
        #   追平（实测：`VrpnSource.rewind` 的倒退窗口结束后 stamp 一次性向前跳回来 ⟹ 放行）。
        self._stamp_hi_s: Optional[float] = None
        self._last_pose: Optional[Tuple[np.ndarray, np.ndarray]] = None
        self._last_pose_at_s: Optional[float] = None
        self._last_pose_stamp_s: Optional[float] = None

        # —— 位置保持的**锚点**（ticket 09）——
        # ★ 有了它，"参考"才是一个**绝对**点：闸门第一次打开那一拍的测量（+ `hold_offset`），
        #   之后**不随测量移动**。没有它的时候参考是"测量 + 常量偏移"，位置误差永远关不掉
        #   —— 第一手证据与那个 0.9 m/s 的爬升见 `hold_reference` 的 ★★。
        # ★ 位置与机头**分开记**（两个 reset 计数器各自对应一件事）：一次纯粹的偏航复位
        #   不该把位置目标也搬走，反之亦然。见 `_update_anchor`。
        self._anchor_pos: Optional[np.ndarray] = None
        self._anchor_yaw: Optional[float] = None
        self._anchor_at_s: Optional[float] = None
        self._anchor_xy_reset: Optional[int] = None
        self._anchor_quat_reset: Optional[int] = None

    # ── 输入 ──
    def on_status(self, msg, at_s: float) -> None:
        self.status = msg
        self.status_at_s = float(at_s)
        self.status_frames += 1

    def on_mocap(self, msg, at_s: float) -> None:
        self.mocap = msg
        self.mocap_at_s = float(at_s)
        self.mocap_frames += 1

    def on_local_position(self, msg, at_s: float) -> None:
        self.local_position = msg
        self.local_position_at_s = float(at_s)
        self.local_position_frames += 1

    def on_attitude(self, msg, at_s: float) -> None:
        self.attitude = msg
        self.attitude_at_s = float(at_s)
        self.attitude_frames += 1

    def age(self, at_s: Optional[float], now_s: float) -> float:
        """多久没收到了 [s]（一次都没收到 = inf —— 不是 0，也不是"很大但有限"）。"""
        return float("inf") if at_s is None else float(now_s - at_s)

    def state_age(self, now_s: float) -> float:
        """**这一路状态源**的龄 [s]。

        ★ PX4 那一档取两条消息里**较旧的那条**（`max`）：位置新、姿态旧不算新鲜 ——
          两样都是喂求解器的必需输入，"至少有一条是新的"没有任何意义。
        """
        if self.config.state_source == STATE_SOURCE_PX4:
            return max(self.age(self.local_position_at_s, now_s),
                       self.age(self.attitude_at_s, now_s))
        return self.age(self.mocap_at_s, now_s)

    def describe_state(self, now_s: float) -> dict:
        """这一拍**状态源的取证面** —— 扁平标量、JSON 友好，直接进 JSONL。

        ★ 与 `Cycle` 分开：`Cycle` 是"决策"，这里是"决策的依据"。复盘时第一个问题是
          "这个数是**谁**给的、**多旧**"，而这两件事在话题记录里都看不出来。

        ★ 为什么连 `xy_valid` / `dead_reckoning` / `eph` 这些 PX4 自己的标志也留下来：
          第一手（2026-09-28，z0 SITL 地面静息、真值恒为 0）EKF 报 x≈−0.45 m、
          z 在 −0.18…−0.64 m 游走、速度到 0.58 m/s，而**四个有效性标志全是 1、
          `dead_reckoning=0`、`eph/epv≈0.33 m`** ⟹ 它们**不是**可用性判据
          （拿它当闸门 = 一个恒假的保证）。但它们是复盘时"这个偏差是谁给的"那条线索。
          **记录，但不闸**（闸门只有"龄"那一条）。
        """
        d = {"state_source": self.config.state_source}
        if self.config.state_source == STATE_SOURCE_PX4:
            d["age_pos_s"] = _num_json(self.age(self.local_position_at_s, now_s))
            d["age_att_s"] = _num_json(self.age(self.attitude_at_s, now_s))
            d["pos_cb"] = self.local_position_frames
            d["att_cb"] = self.attitude_frames
            lp = self.local_position
            for key, attr in (("ekf_xy_valid", "xy_valid"), ("ekf_z_valid", "z_valid"),
                              ("ekf_v_xy_valid", "v_xy_valid"), ("ekf_v_z_valid", "v_z_valid"),
                              ("ekf_dead_reckoning", "dead_reckoning")):
                # ⚠️ 用 `getattr(..., None)`：本模块不 import px4_msgs，字段缺了就当"没有"，
                #    不许崩 —— 日志面永远不该是"让节点挂掉"的那根稻草。
                v = None if lp is None else getattr(lp, attr, None)
                d[key] = None if v is None else int(bool(v))
            for key, attr in (("ekf_eph", "eph"), ("ekf_epv", "epv")):
                v = None if lp is None else getattr(lp, attr, None)
                d[key] = None if v is None else _num_json(v)
            if self.attitude is not None:
                d["quat_reset_counter"] = int(getattr(self.attitude, "quat_reset_counter", 0) or 0)
            if lp is not None:
                d["xy_reset_counter"] = int(getattr(lp, "xy_reset_counter", 0) or 0)
        else:
            d["age_state_s"] = _num_json(self.age(self.mocap_at_s, now_s))
            d["mocap_cb"] = self.mocap_frames
        return d

    # ── 锚点（位置保持的绝对点）──

    def _state_reset_counters(self) -> Tuple[Optional[int], Optional[int]]:
        """**状态帧**的复位计数 `(xy_reset_counter, quat_reset_counter)`；拿不到就是 `None`。

        ★ 这两个计数器的用处只有一个：它们**变了**就说明"帧换了"。EKF 复位之后
          `vehicle_local_position` 的 x/y（乃至 yaw）是在**另一个原点**上量的 ⟹ 锚点里存的那个
          绝对点在新的帧里已经不是同一个地方了 —— 继续追它等于朝一个不存在的点飞。
        ★ PX4 那一档两条消息各带一个：位置帧看 `xy_reset_counter`（`local_position`），
          姿态帧看 `quat_reset_counter`（`attitude`）。动捕那一档没有这种计数器（它的帧由
          动捕系统自己定），返回 `(None, None)` = "没有帧事件可跟"。
        ⚠️ `getattr(..., None)`：本模块不 import px4_msgs，字段缺了就当"没有"（同 `describe_state`）。
        """
        if self.config.state_source != STATE_SOURCE_PX4:
            return None, None
        xy = None if self.local_position is None else getattr(self.local_position, "xy_reset_counter", None)
        quat = None if self.attitude is None else getattr(self.attitude, "quat_reset_counter", None)
        return (None if xy is None else int(xy)), (None if quat is None else int(quat))

    def _update_anchor(self, x_meas: np.ndarray, now_s: float) -> str:
        """维护位置保持的锚点。返回这一拍**做了什么**（`""` / `"anchor"` / `"reanchor_xy"` /
        `"reanchor_quat"` / `"reanchor_xy+quat"`）—— 它进逐帧日志，因为"目标什么时候被搬过"
          是飞行后复盘要问的第一个问题（`Cycle.anchor`）。

        ★ 钉住的时刻是**闸门第一次打开的那一拍**（`cycle()` 只在测量通过全部判据之后调它）：
          闸门关着的时候没有可信状态，拿它当锚点等于把目标钉在一帧被拒的数据上。
        ★ **闸门再次打开时不重钉**（一次跳变/重放恢复之后，飞机该回到原来那个点，而不是
          "就近认一个新目标"）—— 重钉的口子只留给"帧真的换了"这一件事。
        """
        xy_reset, quat_reset = self._state_reset_counters()
        first = self._anchor_pos is None
        re_xy = (not first) and xy_reset is not None and xy_reset != self._anchor_xy_reset
        re_quat = (self._anchor_yaw is not None and quat_reset is not None
                   and quat_reset != self._anchor_quat_reset)

        if first or re_xy:
            self._anchor_pos = np.array(x_meas[:3], dtype=float)
            self._anchor_at_s = float(now_s)
            self._anchor_xy_reset = xy_reset
        if first or re_quat:
            self._anchor_yaw = yaw_from_quat(x_meas[6:10])
            self._anchor_quat_reset = quat_reset

        if first:
            return "anchor"
        return "+".join(n for n, hit in (("reanchor_xy", re_xy), ("reanchor_quat", re_quat)) if hit)

    def anchor_state(self) -> Optional[np.ndarray]:
        """锚点做成一帧状态（`[p, 0, yaw_quat]`），喂 `hold_reference`；没钉过就是 `None`。

        ★ 速度给 0：位置保持的目标就是"停在那儿"（`hold_reference` 里那条也是 0）。
        """
        if self._anchor_pos is None or self._anchor_yaw is None:
            return None
        return np.concatenate([self._anchor_pos, np.zeros(3), yaw_quat(self._anchor_yaw)])

    def anchor_yaw(self) -> Optional[float]:
        """锚点机头 [rad]（没钉过 = `None`）—— 只给**逐帧日志**用。

        ★ 与 `anchor_state()[6:10]` 是同一个量（那个是它的四元数形式）：这一路单独开口是为了
          日志里能直接读到一个**常量**，而不必每帧去解四元数。复盘"控制器在追哪个朝向"时，
          它与当拍 `x0[6:10]` 的**差**就是偏航误差（2026-09-28 的偏航排查缺的正是这一格）。
        """
        return self._anchor_yaw

    # ── 一拍 ──
    def cycle(self, now_s: float) -> Cycle:
        """算出这一拍的外推初值与参考，并判断该不该发。"""
        age_status = self.age(self.status_at_s, now_s)
        age_state = self.state_age(now_s)
        stop = self.config.stop

        if self.status is None:
            return self._hold("no_status", now_s, age_status, age_state)
        armed = int(self.status.arming_state) == ARMING_STATE_ARMED
        nav_state = int(self.status.nav_state)
        if stop.stop_on_armed_drop and not armed:
            return self._hold("not_armed", now_s, age_status, age_state)
        if stop.stop_on_nav_state_exit and nav_state != NAV_STATE_OFFBOARD:
            return self._hold("not_offboard", now_s, age_status, age_state)
        if age_status > stop.status_timeout_s:
            return self._hold("status_stale", now_s, age_status, age_state)

        # ── 测量：两档的分岔**只在这里**（上面的停机条件与下面的输入合理性对两档完全一样）──
        if self.config.state_source == STATE_SOURCE_PX4:
            x_meas, stamp_s, reason, detail = self._measure_px4(now_s)
        else:
            x_meas, stamp_s, reason, detail = self._measure_mocap(now_s)
        if reason is not None:
            return self._hold(reason, now_s, age_status, age_state, detail=detail)

        # ★ 输入合理性（ticket 07 / M5）：这一帧**数值上都正常**，但可能"不是这一秒的"。
        #   两条判据各有第一手的场景：重放/时间戳回退（`stamp_regress`）、刚体认错或遮挡恢复
        #   造成的瞬移（`pose_jump` / `attitude_jump`）。两者的取向都是**宁可漏判，不可误判** ——
        #   误判的代价是**无故停发**（阈值默认值的取法见 `StopConfig.mocap_jump_*`）。
        #   ★ ticket 09：判据**与源无关**（A 档下 EKF 复位/认错给出的瞬移同样只关闸门）。
        if stamp_s is not None and self._stamp_hi_s is not None:
            back = self._stamp_hi_s - stamp_s
            if back > stop.mocap_stamp_regress_s:
                return self._hold(
                    "stamp_regress", now_s, age_status, age_state,
                    detail=f"stamp 倒退 {back:.4f} s > 容差 {stop.mocap_stamp_regress_s:g} s"
                           f"（此前最高 {self._stamp_hi_s:.3f} s，本帧 {stamp_s:.3f} s）")
        jump = self._pose_jump(x_meas, now_s, stamp_s)
        if jump is not None:
            return self._hold(jump[0], now_s, age_status, age_state, detail=jump[1])

        x0 = extrapolate_state(x_meas, self.config.tau_s)
        # ★ 参考从**锚点**起，不从这一拍的测量起（ticket 09；理由与那场 0.9 m/s 的爬升见
        #   `hold_reference` 的 ★★）。锚点在这一拍之前钉/重钉（见 `_update_anchor`）。
        anchor_event = self._update_anchor(x_meas, now_s)
        x_ref = hold_reference(self.anchor_state(), self.config, self.horizon)

        # 这一帧被接受了 ⟹ 它成为下一拍的基准（位置/姿态/时间戳一起搬）
        self._last_pose = (x_meas[:3].copy(), x_meas[6:10].copy())
        self._last_pose_at_s = float(now_s)
        self._last_pose_stamp_s = stamp_s
        if stamp_s is not None:
            self._stamp_hi_s = stamp_s if self._stamp_hi_s is None else max(self._stamp_hi_s, stamp_s)

        cycles = (self._cycles_since_gate + 1) if self._gate_open else 1
        self._gate_open = True
        self._cycles_since_gate = cycles
        publish = cycles > stop.warmup_cycles
        return Cycle(publish=publish, reason="ok" if publish else "warmup",
                     x0=x0, x_ref=x_ref,
                     age_status_s=age_status, age_state_s=age_state,
                     cycles_since_gate=cycles, anchor=anchor_event)

    # ── 测量：两条路各自把自己的话题/坐标系收拾干净，交出来的都是同一件事 ──
    #    约定：返回 `(x_meas, stamp_s, reason, detail)`；`reason` 非 None ⟹ 当拍不发，
    #    `x_meas` 无意义。**两档返回的东西必须是同一个语义**（10 维、ENU/FLU、m 与 m/s）
    #    —— 这是"后面那半段可以共用"的唯一前提。
    def _measure_mocap(self, now_s: float):
        """动捕那一档：`/mocap/pose_enu`（`PoseStamped`，**原生 ENU/FLU**）→ `x_meas`。"""
        stop = self.config.stop
        if self.mocap is None:
            return None, None, "no_mocap", "还没收到 /mocap/pose_enu"
        age = self.age(self.mocap_at_s, now_s)
        if age > stop.mocap_timeout_s:
            return None, None, "mocap_stale", \
                f"动捕龄 {age:.2f} s > 上限 {stop.mocap_timeout_s:g} s"

        position = self.mocap.pose.position
        orientation = self.mocap.pose.orientation
        quat = np.array([orientation.w, orientation.x, orientation.y, orientation.z])
        if not np.isfinite(quat).all() or float(np.linalg.norm(quat)) < 0.5:
            # ⚠️ 动捕偶尔会吐全 0 的四元数（刚体丢帧后的默认帧）。归一化它只会得到 NaN，
            #    而 NaN 进了求解器就是"整条预测轨迹烂掉"—— 宁可这一拍不发。
            return None, None, "bad_mocap_quat", f"|q| = {float(np.linalg.norm(quat)):.3f}"

        # 速度：**约定**是从 `/mocap/pose_enu` 的 `twist` 字段取（ENU，m/s）。
        # ⚠️ 但 z0 现在这条话题是 `geometry_msgs/PoseStamped`（`mocap_rel_pose_node.cpp:93`）
        #    —— **根本没有 twist 字段** ⟹ 缺字段这条路**每拍都在走**，外推此刻等于恒等变换。
        #    真身里速度是有的（`/vrpn/<刚体>/twist`，`vrpn_listener.cpp:46-65`，同 frame），
        #    只是没人把它 republish 到 Phase 2 这一侧。**取法未定，见
        #    [`.scratch/phase2/issues/15`](../../../.scratch/phase2/issues/15-mocap-velocity-source.md)**。
        #    这里的退化行为（v ≡ 0）是刻意的：缺字段不许崩、也不许瞎猜一个速度出来。
        twist = getattr(self.mocap, "twist", None)
        linear = getattr(twist, "linear", None)
        velocity = (linear.x, linear.y, linear.z) if linear is not None else (0.0, 0.0, 0.0)

        x_meas = pose_to_state((position.x, position.y, position.z), quat, velocity)
        if not np.isfinite(x_meas).all():
            return None, None, "non_finite_mocap", "动捕帧里有非有限值（位置或速度）"
        return x_meas, mocap_stamp_s(self.mocap), None, ""

    def _measure_px4(self, now_s: float):
        """PX4 那一档：`vehicle_local_position` + `vehicle_attitude`（**NED/FRD**）→ `x_meas`。

        ★ 两条消息**各自**判新鲜度（位置实测 32.5 Hz、姿态 31.6 Hz，2026-09-28，z0 SITL）：
          它们是两条独立的流。只判位置那一条的后果是"姿态停在几秒前，节点却以为在闭环"，
          而这种错在话题上几乎看不出来（姿态变化慢，两个选择只差一个小倾角）。

        ★ 有效性标志（`xy_valid` / `dead_reckoning` / `eph`…）**不参与判据**，只进取证 ——
          第一手理由见 `describe_state()`。
        """
        stop = self.config.stop
        if self.local_position is None or self.attitude is None:
            missing = " / ".join(t for t, m in (("vehicle_local_position", self.local_position),
                                                ("vehicle_attitude", self.attitude)) if m is None)
            return None, None, "no_px4_state", f"还没收到 {missing}"
        age_pos = self.age(self.local_position_at_s, now_s)
        age_att = self.age(self.attitude_at_s, now_s)
        if age_pos > stop.px4_timeout_s:
            return None, None, "px4_stale", \
                f"位置龄 {age_pos:.2f} s > 上限 {stop.px4_timeout_s:g} s"
        if age_att > stop.px4_timeout_s:
            return None, None, "px4_attitude_stale", \
                f"姿态龄 {age_att:.2f} s > 上限 {stop.px4_timeout_s:g} s"

        att, lp = self.attitude, self.local_position
        quat = np.array([float(c) for c in att.q], dtype=float)
        if not np.isfinite(quat).all() or float(np.linalg.norm(quat)) < 0.5:
            return None, None, "bad_px4_quat", f"|q| = {float(np.linalg.norm(quat)):.3f}"

        # ★★ NED/FRD → ENU/FLU 的**唯一一处**在 `px4_pose_to_enu`（见那里的 ★★ 与
        #    `docs/control/sign-map.md`）。这一行是本票的"约定一致性"断言在代码里的落点：
        #    读错一个号，症状是"一进 OFFBOARD 就朝一边歪"，而符号表看着完全正确。
        x_meas = pose_to_state(*px4_pose_to_enu((lp.x, lp.y, lp.z), quat,
                                                (lp.vx, lp.vy, lp.vz)))
        if not np.isfinite(x_meas).all():
            return None, None, "non_finite_px4", "PX4 状态帧里有非有限值（位置或速度）"

        # 跳变判据的 `dt` 用**较早**的那个 stamp：两条流同频（各 ~32 Hz），差最多一拍，
        # 而判据的取向是"宁可漏判不可误判"（`dt` 取大的一侧 ⟹ 允许的位移/转角更大）。
        stamps = [s for s in (px4_stamp_s(lp), px4_stamp_s(att)) if s is not None]
        return x_meas, (min(stamps) if stamps else None), None, ""

    def _pose_jump(self, x_meas: np.ndarray, now_s: float,
                   stamp_s: Optional[float]) -> Optional[Tuple[str, str]]:
        """位姿跳变判据 → `(原因, 读数)`，或 `None`（这一帧看着可能）。

        ★ 判据是"**这两帧之间能不能走完这段位移/转过这个角**"：`上限 = 速率 × dt`，
          速率默认取 1.7× 真实机动上限（见 `StopConfig`），`dt` 有 `JUMP_DT_FLOOR_S` 的下限。
        ★ 没有基准（刚起来、或源刚被接受过第一帧）时**判不了**，返回 `None` —— 判据不猜。
        ★ 它**不是**闭锁：源只要回到基准附近（或持续足够久让 `dt` 把上限撑开），下一帧就被接受。
        """
        if self._last_pose is None or self._last_pose_at_s is None:
            return None
        last_p, last_q = self._last_pose
        stop = self.config.stop

        # `dt` 优先用**这一路源自己的** stamp 差（它才是"这两帧之间过了多久"）——
        # 读不出来时才退回到达时刻差（`mocap_stamp_s` 返回 None 的三种情形见那里）。
        dt = None
        if stamp_s is not None and self._last_pose_stamp_s is not None:
            dt = stamp_s - self._last_pose_stamp_s
        if dt is None or dt < 0.0:
            dt = now_s - self._last_pose_at_s
        dt = max(float(dt), JUMP_DT_FLOOR_S)

        moved = float(np.linalg.norm(np.asarray(x_meas[:3], dtype=float) - last_p))
        allowed_m = stop.mocap_jump_speed_max * dt
        if moved > allowed_m:
            return ("pose_jump",
                    f"位移 {moved:.3f} m > 允许 {allowed_m:.3f} m（dt={dt * 1e3:.1f} ms、"
                    f"上限 {stop.mocap_jump_speed_max:g} m/s）")
        turned = quat_angle(x_meas[6:10], last_q)
        allowed_rad = stop.mocap_jump_rate_max * dt
        if turned > allowed_rad:
            return ("attitude_jump",
                    f"转角 {math.degrees(turned):.1f}° > 允许 {math.degrees(allowed_rad):.1f}°"
                    f"（dt={dt * 1e3:.1f} ms、上限 {stop.mocap_jump_rate_max:g} rad/s）")
        return None

    def _hold(self, reason: str, now_s: float, age_status: float, age_state: float,
              detail: str = "") -> Cycle:
        """闸门关着的一拍：不发布，也**不给初值/参考**（没有可信的状态就不该喂求解器）。

        ★ 关闸门时把 warmup 计数清零：闸门再开时又要重新热（`WARMUP_CYCLES` 说的是
          "从可信状态出发的连续迭代"，断开之后上一帧的解已经不是这条轨迹的起点了）。
        ⚠️ 但**输入合理性的基准不在这里清零**（见 `__init__`）：那是"源自己说过什么"的
          高水位，与"我们发了没有"无关；清零会让一次持续的重放在下一拍就被当成正常帧。
        """
        self._gate_open = False
        self._cycles_since_gate = 0
        return Cycle(publish=False, reason=reason, x0=None, x_ref=None,
                     age_status_s=age_status, age_state_s=age_state,
                     cycles_since_gate=0, detail=detail)


# ───────────────────────────── 兜底阶梯 ─────────────────────────────

class FallbackLadder:
    """求解**不可用**时的兜底阶梯（ticket 07 / M5）：保持上一帧 → 安全悬停 → 解除。

    | 档位 | 什么时候 | 发什么 |
    |---|---|---|
    | `normal` | 求解可用 | 它的解 |
    | `hold` | 连续不可用 ≤ `hold_frames` | **上一帧可行指令**（原样，只换时间戳） |
    | `hover` | 连续不可用 > `hold_frames` | **安全悬停**（悬停推力 + 零角速率），并**置标志** |

    ★ 为什么不是"一路保持到底"：PX4 会**无限期锁存**最后收到的 `vehicle_rates_setpoint`
      （`MulticopterRateControl.cpp:177-184`，第一手引用见 `docs/control/mpc-node-fallback.md`）
      ⟹ "我们不发"与"飞控一直沿用最后一帧"在飞控侧**是同一件事**。一拍的解刚不可用时，
      上一条命令还是**可行**的（几毫秒前算出来的），所以先保持；但一直不可用的话，被沿用的
      就成了一条**陈旧**命令（可能正打满角速率）—— 必须由我们主动换成安全悬停。
    ★ 为什么不是"一失败就悬停"：一次抖动就换姿态，比保持一条 10 ms 前的可行命令更糟。
      两条合起来给出 `hold_frames` 的量级（见 `FALLBACK_HOLD_FRAMES`）。
    ★ 安全悬停**不依赖任何状态**（推力沿机体系 z、角速率置零）—— 这正是它在"状态不可信"
      之外仍然可用的原因：它不需要知道飞机现在是什么姿态。

    ★ 纯逻辑、零 ROS、**不认识消息类型**：调用方按 `phase` 自己取帧（提交给它的那些帧长什么样
      归 `sign_map`）。于是"第几拍换档、置标志、解除、二次触发"能在一毫秒内被单测钉死。
    """

    NORMAL = "normal"
    HOLD = "hold"
    HOVER = "hover"

    def __init__(self, hold_frames: int = FALLBACK_HOLD_FRAMES):
        self.hold_frames = int(hold_frames)
        self.phase = self.NORMAL
        self.streak = 0         # **连续**不可用的拍数（可用时清零 —— 判据是"连续"不是"累计"）
        self.reason = ""        # 当前这段连续的不可用是为什么（进日志/解除报告）
        self.holds = 0          # 累计"保持"拍数
        self.hovers = 0         # 累计"安全悬停"拍数
        self.events = 0         # 进入安全悬停的**次数**（= 置标志的次数，每次都要能上报）

    def on_ok(self) -> Optional[str]:
        """求解可用的一拍 → 解除报告（此前不是 `normal` 时给出，否则 `None`）。"""
        if self.phase == self.NORMAL:
            self.streak = 0
            self.reason = ""
            return None
        # ⚠️ 先取再清：报告里要写"此前是哪一档、持续了多少拍、为什么"
        report = (f"兜底解除：此前{'保持' if self.phase == self.HOLD else '安全悬停'}"
                  f"连续 {self.streak} 拍（原因：{self.reason}）")
        self.phase = self.NORMAL
        self.streak = 0
        self.reason = ""
        return report

    def on_fail(self, reason: str, can_hold: bool = True) -> Tuple[str, Optional[str]]:
        """求解不可用的一拍 → `(档位, 上报语)`；`上报语` 非空时调用方**必须**把它打出去。

        ★ `can_hold=False` = **没有可保持的上一帧**（第一拍就不可用 —— 此前一直在热身或闸门关着）
          ⟹ **跳过 `hold` 档直接进安全悬停**，并且**照样算一次"进入安全悬停"事件**。
          `hold` 的前提是"上一条命令还是可行的"，而那一拍它根本不存在（PX4 侧的初值是零推力）。
          ⚠️ 这一档必须由**调用方**告诉阶梯：阶梯不认识帧，只有节点知道 `_last_sp` 在不在。
        ★ "置标志并上报"落在**进入安全悬停那一次**（不是每一拍）：票面要的是"这个事件在日志里
          可检索"，每拍重复反而把日志淹掉；持续多少拍由累计量（`hovers`）回答。
        """
        self.streak += 1
        self.reason = str(reason)
        if can_hold and self.streak <= self.hold_frames:
            self.phase = self.HOLD
            self.holds += 1
            return self.HOLD, None
        new_event = self.phase != self.HOVER
        self.phase = self.HOVER
        self.hovers += 1
        if not new_event:
            return self.HOVER, None
        self.events += 1
        return self.HOVER, (f"兜底：连续 {self.streak} 拍没有可用解 ⟹ 切安全悬停"
                            f"（第 {self.events} 次），原因：{self.reason}"
                            + ("" if can_hold else "；★ 此前没有可保持的上一帧（第一拍就不可用）"))

    def reset(self) -> None:
        """闸门关闭时调用：档位与连续计数清零。

        ⚠️ 与 `on_ok` 的区别：`on_ok` 是"求解恢复可用"（要打解除报告），`reset` 是"我们这一拍
          根本不发"（闸门关了，档位毫无意义）。累计量（`holds`/`hovers`/`events`）**都不清零**。
        """
        self.phase = self.NORMAL
        self.streak = 0
        self.reason = ""

    def describe(self) -> str:
        return (f"兜底：保持 {self.holds} 拍 / 安全悬停 {self.hovers} 拍 / 触发 {self.events} 次"
                + (f"（此刻仍在 {self.phase}）" if self.phase != self.NORMAL else ""))
