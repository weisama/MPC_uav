"""节点侧的**纯逻辑**（零 ROS）：外推、闸门、停机条件、参考构造。

★ 为什么要把这些从 `mpc_node.py` 里抠出来：它们全是"给定一串数，判断该不该发、发什么"，
  **一条话题、一个定时器都不需要**，所以能在一毫秒内被单测钉死（`test/test_node_core.py`）。
  留在节点里就会变成"只能靠起 DUT 子进程 + 假 PX4 才能测"——那些测试慢、且失败时
  分不清是逻辑错了还是 DDS 没发现。

★ 本模块与 `ocp.py` 的分界：这里**不碰求解器**（不 import acados），只负责
  "喂给求解器的初值与参考"，以及"这一拍到底该不该发"。求解与符号映射分别是
  `ocp.py` 与 `sign_map.py` 的事。

★ 停机条件**没有任何通道号**（票面验收项「软急停接口参数化」）。理由是第一手的：
  [`.scratch/phase1/issues/02`](../../../.scratch/phase1/issues/02-rc-channel-decision.md)
  已 resolved —— **急停 = ch8 高值 = PX4 硬件杀车，程序不读急停通道**，也就是**不存在**
  "软急停通道"。程序侧能观察到的只有两条既有路径（`phase2-implementation.md` §4 S8 那张表）：
  **打杆接管**（RC 一动，PX4 自己离开 OFFBOARD）与 **ch6 模式开关**（切模式）。
  两者在节点这一侧**长得一样** —— 都表现为 `vehicle_status.nav_state` 不再是 OFFBOARD
  ⟹ 本模块只认状态，不认通道。
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Optional, Tuple

import numpy as np

from .model import NX, normalize_quat, yaw_quat
from .params import Z0Params

__all__ = [
    "TAU_DEFAULT_S", "WARMUP_CYCLES", "ARMING_STATE_ARMED", "NAV_STATE_OFFBOARD",
    "StopConfig", "ControlConfig", "Cycle", "NodeCore",
    "extrapolate_state", "hold_reference", "yaw_from_quat", "pose_to_state",
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

#: PX4 的枚举（`px4_msgs/msg/VehicleStatus.msg` 逐字一致）。
#: ⚠️ 这里**故意只抄两个**（节点只用这两个）：把整张表抄进来就会有两份枚举，
#: 而"两份枚举"正是这一类 bug 的温床。
NAV_STATE_OFFBOARD = 14
ARMING_STATE_ARMED = 2


# ───────────────────────────── 停机条件 ─────────────────────────────

@dataclass(frozen=True)
class StopConfig:
    """什么时候**停止发布**。全部是参数，**一个通道号都没有**（见模块 docstring）。

    ★ 两个 `timeout` 的量级**必须与上游频率匹配**（照抄 Phase 1 的教训）：
      · `/mocap/pose_enu` 实测 ~196 Hz ⟹ 0.5 s 是"明显漏了"的量级
      · `/fmu/out/vehicle_status` 是 **2 Hz 定频**（`Commander.cpp:1790`）⟹ 0.5 s
        只有**一个**周期，任意一次抖动都会误判。真 PX4 在这条流上的心跳是 1 s
        （`COM_OF_LOSS_T`，它自己也会因此退出 OFFBOARD）⟹ 本节点取 **0.6 s** 起，
        默认 `0.75`：小于 PX4 自己的容忍度，才不会"PX4 觉得还新鲜、我们先停了"。
    """

    mocap_timeout_s: float = 0.5
    status_timeout_s: float = 0.75
    #: 离开 OFFBOARD 就停（打杆接管 / ch6 切模式在节点这一侧的**唯一**表现，见模块 docstring）
    stop_on_nav_state_exit: bool = True
    #: 掉出 armed 就停（含"还没解锁"——那种情况下本来也不该发）
    stop_on_armed_drop: bool = True
    #: 收到 armed + OFFBOARD 之后先解这么多次，**不发**（见 `WARMUP_CYCLES`）
    warmup_cycles: int = WARMUP_CYCLES


@dataclass(frozen=True)
class ControlConfig:
    """节点控制律侧的配置（唯一一份 —— CLI 参数、`Z0Params`、`OcpConfig` 都从这里接出去）。"""

    tau_s: float = TAU_DEFAULT_S
    #: 参考点相对**测量位姿**的偏移 [m]（ENU）。默认 (0,0,0) = 原地位置保持。
    #: ★ 它是本票的"参数化参考"：真正的轨迹接口（`/traj/reference` 订阅）归 ticket 09，
    #: 这里只需要一个"能让 MPC（模型预测控制）真的动起来"的目标，好让符号/归一化/饱和
    #: 这三条验收项在**闭环**里被看见（一个恒定参考点也让闭环有一个明确的稳态）。
    hold_offset: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    #: 参考的机头（rad，ENU 从 +x 逆时针）。`None` = 保持当前机头（位置保持的正确默认）。
    hold_yaw: Optional[float] = None
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
      在 Phase 2 里**不存在**。若哪天要接一条 NED 的话题，转换必须写在这里、
      并且**只能有一处**（否则"符号错了"与"模型解错了"再也分不开）。
    """
    q = normalize_quat(np.asarray(orientation, dtype=float).reshape(4))
    return np.concatenate([np.asarray(position, dtype=float).reshape(3),
                           np.asarray(velocity, dtype=float).reshape(3), q])


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


def hold_reference(x_now: np.ndarray, config: ControlConfig, N: int) -> np.ndarray:
    """构造 `(N+1, NX)` 的位置保持参考：位置 = 测量位置 + `hold_offset`，速度 0，姿态水平。

    ★ 姿态取**水平**（roll = pitch = 0）而非照抄测量姿态：悬停参考就该是水平的，
      而 OCP 的代价对四元数只罚**倾斜**（`qw` 权重是 0，见 `OcpConfig.q_diag`）⟹
      水平 + 当前机头才是"不倾斜、朝向不变"的那个目标。
    """
    x = np.asarray(x_now, dtype=float).reshape(NX)
    yaw = config.hold_yaw if config.hold_yaw is not None else yaw_from_quat(x[6:10])
    target = np.concatenate([
        x[:3] + np.asarray(config.hold_offset, dtype=float),
        np.zeros(3),
        yaw_quat(yaw),
    ])
    return np.tile(target, (N + 1, 1))


# ───────────────────────────── 闸门 ─────────────────────────────

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
    age_mocap_s: float
    #: 闸门打开之后已经解了几拍（`warmup_cycles` 那一段的进度，给日志用）
    cycles_since_gate: int


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
        self._gate_open = False         # 上一拍闸门是不是开着的（用来数 warmup）
        self._cycles_since_gate = 0     # 闸门打开后已经解了几拍

    # ── 输入 ──
    def on_status(self, msg, at_s: float) -> None:
        self.status = msg
        self.status_at_s = float(at_s)
        self.status_frames += 1

    def on_mocap(self, msg, at_s: float) -> None:
        self.mocap = msg
        self.mocap_at_s = float(at_s)
        self.mocap_frames += 1

    def age(self, at_s: Optional[float], now_s: float) -> float:
        """多久没收到了 [s]（一次都没收到 = inf —— 不是 0，也不是"很大但有限"）。"""
        return float("inf") if at_s is None else float(now_s - at_s)

    # ── 一拍 ──
    def cycle(self, now_s: float) -> Cycle:
        """算出这一拍的外推初值与参考，并判断该不该发。"""
        age_status = self.age(self.status_at_s, now_s)
        age_mocap = self.age(self.mocap_at_s, now_s)
        stop = self.config.stop

        if self.status is None:
            return self._hold("no_status", now_s, age_status, age_mocap)
        armed = int(self.status.arming_state) == ARMING_STATE_ARMED
        nav_state = int(self.status.nav_state)
        if stop.stop_on_armed_drop and not armed:
            return self._hold("not_armed", now_s, age_status, age_mocap)
        if stop.stop_on_nav_state_exit and nav_state != NAV_STATE_OFFBOARD:
            return self._hold("not_offboard", now_s, age_status, age_mocap)
        if age_status > stop.status_timeout_s:
            return self._hold("status_stale", now_s, age_status, age_mocap)
        if self.mocap is None:
            return self._hold("no_mocap", now_s, age_status, age_mocap)
        if age_mocap > stop.mocap_timeout_s:
            return self._hold("mocap_stale", now_s, age_status, age_mocap)

        position = self.mocap.pose.position
        orientation = self.mocap.pose.orientation
        quat = np.array([orientation.w, orientation.x, orientation.y, orientation.z])
        if not np.isfinite(quat).all() or float(np.linalg.norm(quat)) < 0.5:
            # ⚠️ 动捕偶尔会吐全 0 的四元数（刚体丢帧后的默认帧）。归一化它只会得到 NaN，
            #    而 NaN 进了求解器就是"整条预测轨迹烂掉"—— 宁可这一拍不发。
            return self._hold("bad_mocap_quat", now_s, age_status, age_mocap)

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

        x_meas = pose_to_state((position.x, position.y, position.z),
                               quat, velocity)
        if not np.isfinite(x_meas).all():
            return self._hold("non_finite_mocap", now_s, age_status, age_mocap)

        x0 = extrapolate_state(x_meas, self.config.tau_s)
        x_ref = hold_reference(x_meas, self.config, self.horizon)

        cycles = (self._cycles_since_gate + 1) if self._gate_open else 1
        self._gate_open = True
        self._cycles_since_gate = cycles
        publish = cycles > stop.warmup_cycles
        return Cycle(publish=publish, reason="ok" if publish else "warmup",
                     x0=x0, x_ref=x_ref,
                     age_status_s=age_status, age_mocap_s=age_mocap,
                     cycles_since_gate=cycles)

    def _hold(self, reason: str, now_s: float, age_status: float, age_mocap: float) -> Cycle:
        """闸门关着的一拍：不发布，也**不给初值/参考**（没有可信的状态就不该喂求解器）。

        ★ 关闸门时把 warmup 计数清零：闸门再开时又要重新热（`WARMUP_CYCLES` 说的是
          "从可信状态出发的连续迭代"，断开之后上一帧的解已经不是这条轨迹的起点了）。
        """
        self._gate_open = False
        self._cycles_since_gate = 0
        return Cycle(publish=False, reason=reason, x0=None, x_ref=None,
                     age_status_s=age_status, age_mocap_s=age_mocap,
                     cycles_since_gate=0)
