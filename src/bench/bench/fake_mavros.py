"""假 mavros —— seam 1 的另一半。

★ 它假的是 **mavros 的行为**，不是 PX4 的行为。这条界线很重要：
   · mavros 的行为（话题名、频率、**QoS**、`/mavros/state` 1 Hz、`mode_sent` 的语义）在这里复刻
   · PX4 的行为（OFFBOARD 超时退出、EKF2 融合、模式需求检查）**不**在这里 ——
     那是 SITL / 真机的职责（见 .scratch/phase1/spec.md「SITL 降级为上机前手动跑一次的集成关卡」）

★★ QoS 是本文件最容易被低估的部分，每一条都对着真 mavros 2.15.1 的源码抄：

   | 话题 | 真 mavros 的发布/订阅 | 依据 |
   |---|---|---|
   | `/mavros/state` | **发布** `QoS(10).transient_local()`（可靠） | `sys_status.cpp` `mavros::StateQoS()` |
   | `/mavros/local_position/*` | **发布** `SensorDataQoS()`（**best_effort**） | `local_position.cpp` |
   | `/mavros/battery` | **发布** `SensorDataQoS()`（**best_effort**） | `sys_status.cpp` |
   | `/mavros/rc/in` | **发布** 默认 `10`（可靠） | `rc_io.cpp` |
   | `/mavros/estimator_status` | **发布** `QoS(10).transient_local()`（可靠 + 瞬态本地） | `sys_status.cpp` 用 `mavros::StateQoS()` |
   | `/mavros/setpoint_raw/local` | **订阅** `SensorDataQoS()`（best_effort） | `setpoint_raw.cpp:62` |
   | `/mavros/setpoint_raw/attitude` | **订阅** `SensorDataQoS()`（best_effort） | `setpoint_raw.cpp:62`（★ ticket 13 §四① 新增的那条流） |
   ★ `setpoint_raw/attitude` 这一行是 ticket 13 加的：(C) 级 `mpc_node` 的下行从
     `/fmu/in/vehicle_rates_setpoint` 换成了它。**用真的那个 QoS 是刻意的** ——
     夹具若图省事用默认（reliable），"DUT 发端 QoS 写错 ⟹ 永不匹配"这一类静默失效
     在 seam 1 上就**永远不会出现**（DUT 若写 best_effort、收端写 reliable，DDS 不兼容、
     不报错、只是收不到），而那正是本 seam 要挡的东西。
   ★ `estimator_status` 这一行是 ticket 16 加的：DUT 侧**故意用易失订阅**（见 `px4_interface.cpp`
     里那段 ★★）—— 易失订阅与瞬态本地发布方**兼容**（durability 只要求 offered ≥ requested），
     所以夹具照抄真 mavros 的发布方 QoS 才是对的：要让"DUT 会不会被历史回放喂饱计数"
     这件事在 seam 1 上**真的能被复现**，夹具就不能替它把 QoS 降级。

   为什么必须一致：QoS 不兼容的两个端点**不报错**，只是永远不匹配 ⟹ 表现为"节点没数据"。
   本文件用 `bench/__init__.py` 里那两个工厂函数把这件事写成代码（并被 `test_bench_selftest.py` 盯着）。

   ☠️ **上面"日志里看不出原因"这半句 2026-10-13 被第一手推翻了一半**（ticket 13 §四现场，
     本机 Humble + rmw_fastrtps，四格矩阵见 `attitude_publishers` 的 ⚠️⚠️）：
     **不兼容的那一格，两端各自的 rmw 都会打一条 WARN**，字符串长这样 ——
       `New subscription discovered on topic <T>, requesting incompatible QoS.
        No messages will be sent to it. Last incompatible policy: RELIABILITY`
     （订阅侧那条是 `New publisher discovered … offering incompatible QoS.
      No messages will be received from it.`）
     ⟹ **失配不是完全静默的**：它落在**进程自己的日志**里（于是也会落到 `dut_log()` 里）。
     ★ 但"有条 WARN 就够了吗"的答案是**不够**：① 它只在**发现**那一刻打一次，
       查历史日志时容易漏；② `ros2 topic list` / `hz` 确实照样看不出；
       ③ 本仓的 DUT 日志是**断言用的证据**，一条 WARN 不是判据。
     ⟹ 结论不变（夹具照抄真 QoS、用例直接读 profile），但**别再把"日志完全看不出原因"
       当成理由** —— 那句话会让人以为"没有 WARN 就说明 QoS 是对的"。

★ 运行方式：只被 `Scenario` 创建，不要单独跑（它没有 main）。
"""

from __future__ import annotations

import math
import threading
import time
from dataclasses import dataclass
from typing import Dict, List, Optional, Sequence

from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import AttitudeTarget, EstimatorStatus, PositionTarget, RCIn, State
from mavros_msgs.srv import CommandBool, SetMode
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
    qos_profile_services_default,
)
from sensor_msgs.msg import BatteryState


# ───────────────────────────── QoS：与真 mavros 逐字对齐 ─────────────────────────────

def mavros_state_qos() -> QoSProfile:
    """`/mavros/state`：**可靠 + 瞬态本地**。瞬态本地 ⟹ 晚订阅者立刻拿到最后一帧，
    这也是为什么"重放一个已经发过的 state"不需要额外机制。"""
    return QoSProfile(
        depth=10,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )


def mavros_sensor_qos() -> QoSProfile:
    """`/mavros/local_position/*` 与 `/mavros/battery`：`rclcpp::SensorDataQoS()` 的等价物
    （keep_last(5) / best_effort / volatile）。"""
    return qos_profile_sensor_data


def mavros_default_qos() -> QoSProfile:
    """`/mavros/rc/in`：默认的 `10`（可靠 / volatile）。"""
    return QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)


def _qos_accepts(offered: QoSProfile, requested: QoSProfile) -> bool:
    """**发布方提供的** QoS 能不能满足**订阅方要求的** QoS（DDS 的 RxO 规则）。

    ★ 为什么夹具要自带一份这个判据：ROS 2 的"两个端点接上了没有"**没有 API 能直接读**
      （rclpy 的 `Subscription` 没有 `get_publisher_count`；见 `attitude_publishers` 的 ☠️），
      而 `Node.get_publishers_info_by_topic` 读的是**发现库**、**不做 QoS 过滤**
      （第一手反例写在那条 docstring 的 ⚠️⚠️ 里）。⟹ 要判"接得上"只能自己按 RxO 算。

    ★ 规则只有**两条**方向性约束（`offered ≥ requested`）：
      · reliability：订阅要 `RELIABLE` ⟹ 发布方也必须是 `RELIABLE`
        （发布方 `RELIABLE`、订阅方 `BEST_EFFORT` **是兼容的** —— 反直觉，但这正是
         "夹具收端是 best_effort ⟹ DUT 发 reliable 照样接上"的由来）；
      · durability：订阅要 `TRANSIENT_LOCAL` ⟹ 发布方也必须是 `TRANSIENT_LOCAL`
        （反过来：瞬态本地的发布方 + volatile 的订阅方**兼容**，见模块 docstring 那段 ★）。
    ⚠️ `history` / `depth` **不参与**兼容性判定 —— 而且在本套栈的发现信息里
      `history` 回来是 `UNKNOWN`、`depth` 恒为 0，想判也判不了。
    """
    if (requested.reliability == ReliabilityPolicy.RELIABLE
            and offered.reliability != ReliabilityPolicy.RELIABLE):
        return False
    if (requested.durability == DurabilityPolicy.TRANSIENT_LOCAL
            and offered.durability != DurabilityPolicy.TRANSIENT_LOCAL):
        return False
    return True


# ───────────────────────────── 记录下来的东西 ─────────────────────────────

@dataclass
class Setpoint:
    """DUT 发来的一帧 setpoint。`at_s` 是**到达时刻**（相对 Scenario 的 t0），不是 `header.stamp`。

    ★ 两个时间要分开看：断言"频率"用 `at_s`（DUT 是否真的在流），
      断言"时间戳是否为微秒/倒退"用 `msg.header.stamp`（DUT 填了什么）。
    """
    at_s: float
    msg: PositionTarget


@dataclass
class ModeRequest:
    """一次 `/mavros/set_mode` 调用。`sent` 对应响应的 `mode_sent` 字段
    （⚠️ 只表示 SET_MODE 报文发出去了，不代表 PX4 接受 —— 见 px4_interface.hpp）。"""
    at_s: float
    custom_mode: str
    sent: bool


@dataclass
class ArmingRequest:
    at_s: float
    value: bool


@dataclass
class AttitudeSetpoint:
    """DUT 发到 `/mavros/setpoint_raw/attitude` 的一帧（ticket 13 §四① 新增的那条流）。

    ★★ **本票的关键设计**：(C) 级 CTBR 的下行换成 `mavros_msgs/AttitudeTarget` 之后，
      "符号/归一化"这一类错误（N1 推力反号、N2 多转一次 FLU→FRD）**只有在断言落在
      "我们发出的那个字段值"上时才抓得到**。若断言落在"飞控最终收到的物理方向"上，
      两次取负看起来和正确的一模一样（见 [判据全绿也可能是机理反了] 那一族）。
      ⟹ 这条记录必须把 DUT 填进去的**原值**逐条留下来，`type_mask` / `body_rate` 三分量 /
      `thrust` / `header.stamp` 一个都不能漏 —— 判据就落在它们上面。

    ⚠️ `msg` 是 rclpy 在回调里**反序列化出来的新对象**（每次回调一个）⟹ 留着它不会随
      后续帧变；但**不要**把它当"可变缓冲"就地改（那会污染这条记录）。
    """
    at_s: float
    msg: AttitudeTarget

    # ── 直接读"发出去的那几个字段"的便捷属性（判据全落在这一层）──
    @property
    def type_mask(self) -> int:
        #: ★ 必须是 `128`（`IGNORE_ATTITUDE`）。⛔ `IGNORE_THRUST = 64` 不许被置上：
        #:   置了就是"我们发的 thrust 被忽略" ⟹ 掉高度（而话题面上每一帧都正常）。
        return int(self.msg.type_mask)

    @property
    def body_rate(self) -> tuple:
        """`(x, y, z)` —— **FLU 原样**（我们**不**在这里取负；取负由 mavros 做，见 N2）。"""
        return (float(self.msg.body_rate.x), float(self.msg.body_rate.y),
                float(self.msg.body_rate.z))

    @property
    def thrust(self) -> float:
        """归一化推力，**正号**（`F / t_max`；取负由飞控的 `fill_thrust` 做，见 N1）。"""
        return float(self.msg.thrust)

    @property
    def stamp_ns(self):
        """`header.stamp` → 纳秒（`None` 表示没填）。★ 新载体上飞控**不读**它，见 §13。"""
        ts = self.msg.header.stamp
        return int(ts.sec) * 1_000_000_000 + int(ts.nanosec)


@dataclass
class StateFrame:
    """夹具**自己发出去**的一帧 `/mavros/state`（ticket 13 §四③ 的"起机屏障"要用）。

    ★ 为什么需要记它：心跳那条流删掉之后（§二④），DUT **在解锁 + OFFBOARD 之前什么都不发**
      ⟹ 旧版那种"收到第一帧心跳 ⟹ 节点起来了"的屏障没有了。而"夹具把 armed/OFFBOARD 注入
      下去了没有"必须有个可等的读数（否则只能 `sleep` 猜，那是抖动的来源）。
      ⚠️ 它记的是**夹具发出去的**，不是 DUT 收到的 —— 与旧版 `FakePx4.status_frames` 同一语义。
    """
    at_s: float
    msg: State


# ───────────────────────────── 主类 ─────────────────────────────

class FakeMavros:
    """假的 mavros。**所有 rclpy 调用都发生在 executor 线程里**（定时器/订阅/服务回调），
    主线程只改普通的 Python 属性 + 读记录（`list.append` 在 GIL 下是原子的）⟹
    不需要在测试里到处加锁，也不会出现"两个线程同时碰 rclpy"。

    要注入的故障全部是"改一个属性"：静默（`mute`）、改状态（`set_state`）、
    改位置（`set_position`）、改速度（`set_velocity`）、改电压（`set_voltage`）、
    改遥控（`set_rc`）、服务应答策略（`set_mode_policy` / `set_arming_policy`）、
    以及 mavros **自己会发的那帧合成断连状态**（`synth_disconnect`，ticket 18）。
    """

    SILENCEABLE = ("state", "local_position", "local_velocity", "rc", "battery",
                   "estimator_status")

    def __init__(
        self,
        node: Node,
        clock,
        rates_hz: Optional[Dict[str, float]] = None,
        topics: Optional[Dict[str, str]] = None,
        connected: bool = True,
        armed: bool = False,
        mode: str = "POSCTL",
        position: Sequence[float] = (0.0, 0.0, 0.0),
        # ★ 与 `bench/config.py` 的 DEFAULTS["initial"]["voltage"] 保持一致（6S 满电附近）：
        #   必须明显高于 DUT 的 `low_voltage_v`，否则"没打算注入低压"的场景会立刻降级。
        voltage: float = 25.0,
        rc_channels: Optional[Sequence[int]] = None,
        create_set_mode: bool = True,
        create_arming: bool = True,
        follow_set_mode: bool = True,
        follow_arming: bool = True,
        mode_transition_delay_s: float = 0.05,
    ) -> None:
        self.node = node
        # 时刻的尺子由 Scenario 提供（`clock.t()`），这样"记录里的一秒"与"时序里的一秒"是同一秒。
        self.clock = clock

        # ★ `estimator_status` 的 1.0 Hz 是**照抄实测**（2026-09-22 z0：1.000 Hz）——
        #   它是 MAVLink 的 `ESTIMATOR_STATUS` 流，PX4 侧默认就在发，mavros 收到即转发。
        #   ⚠️ 这个数决定了 DUT 侧 `ekf2_pos_lost_samples: 2` 折合多少秒（2 s）：改它就要重估那个参数。
        rates = {"state": 1.0, "local_position": 30.0, "local_velocity": 30.0,
                 "rc": 10.0, "battery": 1.0, "estimator_status": 1.0}
        rates.update(rates_hz or {})
        t = {
            "state": "/mavros/state",
            "local_position": "/mavros/local_position/pose",
            "local_velocity": "/mavros/local_position/velocity_local",
            "rc": "/mavros/rc/in",
            "battery": "/mavros/battery",
            "estimator_status": "/mavros/estimator_status",
            "setpoint": "/mavros/setpoint_raw/local",
            # ★ ticket 13 §四①：**(C) 级 CTBR 的新下行**（`mavros_msgs/AttitudeTarget`）。
            #   与上面那条 `setpoint_raw/local`（Phase 1 的 `control` 用的）**并存** ——
            #   两条话题是两个不同的消费者，夹具两条都要收。
            "attitude": "/mavros/setpoint_raw/attitude",
            "set_mode": "/mavros/set_mode",
            "arming": "/mavros/cmd/arming",
        }
        t.update(topics or {})
        self.topics = t

        # —— 可注入的状态（主线程改，executor 线程读）——
        # ⚠️ 全是"读一次就够"的标量/元组替换，不做就地修改 ⟹ 不用锁。
        self._muted: set = set()
        # ★ 合成断连帧的待发标记（ticket 18）：主线程置位，**定时器线程**消费（见 `_tick`）。
        self._pending_disconnect_frame = False
        self.connected = connected
        self.armed = armed
        self.mode = mode
        self.guided = False
        self.manual_input = False
        self.position = tuple(float(v) for v in position)
        # 速度（ROS/ENU，m/s）。默认 **零** = "静止" —— 悬停判稳的场景不需要动它；
        # 需要造"还在动"时用 `set_velocity`（判据的另一半要靠它才能被证伪）。
        self.velocity = (0.0, 0.0, 0.0)
        # 机头（ROS/ENU，绕 +z 的 yaw，逆时针为正）。默认 0 ⟹ 单位四元数。
        self.yaw = 0.0
        self.voltage = float(voltage)
        # ★★ ticket 16：EKF2 自报的"水平位置有绝对参考"（`estimator_status.pos_horiz_abs`）。
        #   默认 True —— 与"正常飞行"一致；`set_pos_horiz_abs(False)` 才造得出
        #   「帧还在到、但 EKF2 说位置没有绝对参考」这一类失效（动捕那两条判据看不见它）。
        self.pos_horiz_abs = True
        # ★ 已发出的 `estimator_status` 帧数（ticket 16）。存在的唯一理由是让
        #   "**连续 N 帧**"这类消抖判据在 seam 1 上能被**精确**构造与断言：
        #   靠 `sleep` 猜"现在发到第几帧了"在 1 Hz 上游上必然又慢又飘。
        self.estimator_status_frames = 0
        # 通道 1 起算（长度 16）。默认中位 1500，ch8 = 硬件杀车、ch9 = 开始（见 ticket 02 的分配）。
        self.rc_channels = list(rc_channels) if rc_channels is not None else (
            [1500] * 16)
        if rc_channels is None:
            self.rc_channels[7] = 1000   # ch8：硬件杀车（低 = 未触发）
            self.rc_channels[8] = 1000   # ch9：程序开始（低 = 未触发）
        # 服务应答策略：'ok' = mode_sent True；'fail' = mode_sent False；
        #                 'absent' = 服务不存在（DUT 走 service_is_ready() == False 那条分支）
        # ⚠️ 用下划线前缀：`set_mode_policy` 是**方法名**，属性与它同名会把它遮住。
        self._set_mode_policy = "ok" if create_set_mode else "absent"
        self._arming_policy = "ok" if create_arming else "absent"
        self.follow_set_mode = follow_set_mode
        # ★ 与 `follow_set_mode` 分开的一个开关：造"**飞控答应了但没真的解锁**"这种局面
        #   （真机上对应 prearm 检查在 ack 之后才失败 / 被 COM_ARM_CHK 拦下）。
        #   它和 `set_arming_policy('fail')` 是**两种不同的失效**：
        #     · `'fail'`          ⟹ 服务应答就是否定的（DUT 立刻知道被拒）
        #     · `follow_arming=False` ⟹ 应答是肯定的，但 armed 永不上升沿（DUT 只能等到超时）
        self.follow_arming = follow_arming
        self.mode_transition_delay_s = mode_transition_delay_s

        # —— 记录 ——
        self.setpoints: List[Setpoint] = []
        #: ★ ticket 13 §四①：`/mavros/setpoint_raw/attitude` 上收到的帧（(C) 级的新下行）。
        #: ⚠️ 与 `setpoints`（`/mavros/setpoint_raw/local`，Phase 1 的 `control` 用的）
        #: **分开两条列表** —— 两条话题的发布者可能同时在场（两条链并存），混在一起就分不清
        #: 哪一帧是谁发的了。
        self.attitude_setpoints: List[AttitudeSetpoint] = []
        #: 夹具发出去的 `/mavros/state` 帧（"armed/OFFBOARD 注入下去了没有"的屏障，见 `StateFrame`）。
        self.state_frames: List[StateFrame] = []
        self.mode_requests: List[ModeRequest] = []
        self.arming_requests: List[ArmingRequest] = []
        self._lock = threading.Lock()

        # —— 上游（与真 mavros 同名同 QoS）——
        self._state_pub = node.create_publisher(State, t["state"], mavros_state_qos())
        self._pose_pub = node.create_publisher(
            PoseStamped, t["local_position"], mavros_sensor_qos())
        self._vel_pub = node.create_publisher(
            TwistStamped, t["local_velocity"], mavros_sensor_qos())
        self._rc_pub = node.create_publisher(RCIn, t["rc"], mavros_default_qos())
        self._battery_pub = node.create_publisher(
            BatteryState, t["battery"], mavros_sensor_qos())
        # ★ ticket 16：真 mavros 用 `mavros::StateQoS()` 发它（`sys_status.cpp:579-584` 那一段里
        #   与 `/mavros/state` 同一个 QoS 对象），所以这里也用 `mavros_state_qos()`。
        #   ⚠️ DUT 侧是**易失**订阅 ⟹ 两者兼容但**不回放历史**（见 `px4_interface.cpp` 那段 ★★）。
        self._est_pub = node.create_publisher(
            EstimatorStatus, t["estimator_status"], mavros_state_qos())
        # 供"发现屏障"按 key 取用（见 matched_subscribers）
        self._pubs = {
            "state": self._state_pub,
            "local_position": self._pose_pub,
            "local_velocity": self._vel_pub,
            "rc": self._rc_pub,
            "battery": self._battery_pub,
            "estimator_status": self._est_pub,
        }

        # —— 下游（mavros 是订阅方，QoS 用 SensorDataQoS ⟹ 发布可靠一方也能匹配）——
        self._setpoint_sub = node.create_subscription(
            PositionTarget, t["setpoint"], self._on_setpoint, mavros_sensor_qos())
        # ★ ticket 13 §四①：新下行（`AttitudeTarget`）。QoS **必须**用真的那个
        #   `SensorDataQoS()`（best_effort / keep_last 5 / volatile）—— 见模块 docstring。
        self._attitude_sub = node.create_subscription(
            AttitudeTarget, t["attitude"], self._on_attitude_setpoint, mavros_sensor_qos())

        # —— 服务（真 mavros 用 `rclcpp::ServicesQoS()`：可靠 / volatile / depth 10）——
        self._set_mode_srv = None
        self._arming_srv = None
        if create_set_mode:
            self._set_mode_srv = node.create_service(
                SetMode, t["set_mode"], self._on_set_mode,
                qos_profile=qos_profile_services_default)
        if create_arming:
            self._arming_srv = node.create_service(
                CommandBool, t["arming"], self._on_arming,
                qos_profile=qos_profile_services_default)

        # —— 定时器（所有发布都在 executor 线程里发生）——
        self._timers = []
        for key, pub_cb in (("state", self._pub_state),
                            ("local_position", self._pub_pose),
                            ("local_velocity", self._pub_velocity),
                            ("rc", self._pub_rc),
                            ("battery", self._pub_battery),
                            ("estimator_status", self._pub_estimator_status)):
            period = 1.0 / max(1e-3, rates[key])
            self._timers.append(
                node.create_timer(period, lambda cb=pub_cb, k=key: self._tick(k, cb)))

        # 待生效的模式（模拟 PX4 接受 SET_MODE 之后把 mode 改掉）
        self._pending_mode: Optional[str] = None
        self._pending_mode_at_s: float = 0.0

    # ─────────────────── 时间 ───────────────────

    def t(self) -> float:
        """相对 Scenario 的 t0 的秒数 —— 与 Timeline 的事件时刻、记录的 `at_s` 同一把尺子。"""
        return self.clock.t()

    # ─────────────────── 发布 ───────────────────

    def _tick(self, key: str, cb) -> None:
        # ★★ 合成断连帧（ticket 18）优先于静默：真 mavros 是从 `conn_timeout` 那个 **wall timer**
        #   里发它的，与"收到 HEARTBEAT 才发 state"**不是同一条路径** ⟹ 即使 state 已被静默，
        #   这一帧也必须发出去（而且只发一次 —— 发完 flag 就清了）。
        if key == "state" and self._pending_disconnect_frame:
            self._pending_disconnect_frame = False
            self._publish_disconnect()
            return
        if key in self._muted:
            return
        cb()

    def _publish_disconnect(self) -> None:
        """`publish_disconnection()` 的逐字复刻（`sys_status.cpp:903-911`）。

        ★★ 六个字段**全是常量**（`connected`/`armed`/`guided`/`manual_input` 全 `False`、
           `mode=""`、`system_status=UNINIT`），与夹具自己的属性无关 ——
           真 mavros 也不看它们（它连飞控都联系不上）。这就是"那帧的 `armed=false` 不是读数"
           在夹具里的落点；控制节点把它当读数的后果见 ticket 18。
        """
        msg = State()
        msg.header.stamp = self._stamp()
        msg.connected = False
        msg.armed = False
        msg.guided = False
        msg.manual_input = False
        msg.mode = ""
        msg.system_status = 0   # MAV_STATE::UNINIT（`enum_value(MAV_STATE::UNINIT)`）
        self._state_pub.publish(msg)
        with self._lock:
            self.state_frames.append(StateFrame(at_s=self.t(), msg=msg))

    def _stamp(self):
        return self.node.get_clock().now().to_msg()

    def _pub_state(self) -> None:
        # 模拟 PX4 真的接受了 SET_MODE：延迟一小会儿后把 mode 改掉。
        if self._pending_mode is not None and self.t() >= self._pending_mode_at_s:
            self.mode = self._pending_mode
            self._pending_mode = None

        msg = State()
        msg.header.stamp = self._stamp()
        msg.connected = self.connected
        msg.armed = self.armed
        msg.guided = self.guided
        msg.manual_input = self.manual_input
        msg.mode = self.mode
        msg.system_status = 3  # MAV_STATE_STANDBY
        self._state_pub.publish(msg)
        with self._lock:
            self.state_frames.append(StateFrame(at_s=self.t(), msg=msg))

    def _pub_pose(self) -> None:
        x, y, z = self.position
        msg = PoseStamped()
        msg.header.stamp = self._stamp()
        msg.header.frame_id = "map"   # ← 真 mavros 的 local_position 用 "map"（不是 "world"）
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.position.z = float(z)
        # 机头由 `set_yaw` 决定（默认 0 ⟹ 单位四元数，与 ticket 05 那会儿一样）。
        msg.pose.orientation.w = math.cos(self.yaw / 2.0)
        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = math.sin(self.yaw / 2.0)
        self._pose_pub.publish(msg)

    def _pub_velocity(self) -> None:
        # ★ 与 `/mavros/local_position/pose` **同一条 MAVLink 消息**（真 mavros 在
        #   `local_position.cpp` 的同一个回调里发这两个话题）⟹ 两者的频率、时间戳、口径一致。
        #   速度同样是 **ROS/ENU**（z 向上为正），见 `set_velocity`。
        msg = TwistStamped()
        msg.header.stamp = self._stamp()
        msg.header.frame_id = "map"
        vx, vy, vz = self.velocity
        msg.twist.linear.x = float(vx)
        msg.twist.linear.y = float(vy)
        msg.twist.linear.z = float(vz)
        self._vel_pub.publish(msg)

    def _pub_rc(self) -> None:
        msg = RCIn()
        msg.header.stamp = self._stamp()
        msg.rssi = 100
        msg.channels = [int(c) for c in self.rc_channels]
        self._rc_pub.publish(msg)

    def _pub_battery(self) -> None:
        msg = BatteryState()
        msg.header.stamp = self._stamp()
        msg.voltage = float(self.voltage)
        msg.temperature = float("nan")
        msg.current = float("nan")
        msg.percentage = float("nan")
        msg.present = True
        msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
        msg.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_GOOD
        msg.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LIPO
        msg.cell_voltage = []
        msg.cell_temperature = []
        self._battery_pub.publish(msg)

    def _pub_estimator_status(self) -> None:
        # ★ 帧计数（ticket 16）：DUT 的判据是"**连续**几帧为假"，而"连续"这件事在测试里
        #   只能靠"我知道已经发了几帧"来精确构造 —— 用 sleep 猜帧数会既慢又飘。
        self.estimator_status_frames += 1
        # ★ ticket 16：**只填 DUT 真正读的那一位**，别的位保持 False。
        #   为什么值得刻意留白：真 mavros 的这一帧里有十来位（速度、垂直位置、GPS glitch…），
        #   而 DUT 的判据只认 `pos_horiz_abs_status_flag` —— 夹具把别的位全填 True 会让
        #   "以后有人顺手改成读另一位"这种改动在台架上**照样绿**（那位永远是 True）。
        #   留白 ⟹ 任何"读错了位"的改动都会立刻表现为"判据恒假"。
        msg = EstimatorStatus()
        msg.header.stamp = self._stamp()
        msg.pos_horiz_abs_status_flag = bool(self.pos_horiz_abs)
        self._est_pub.publish(msg)

    # ─────────────────── 发现屏障（★ 任何"没收到"的断言之前先用它）───────────────────

    def matched_subscribers(self, topic_key: str) -> int:
        """DUT 侧在这个话题上有几个订阅者**已经匹配上**。

        ★ 为什么这是夹具里最该先用的一个断言原语：**QoS 不兼容时这里恒为 0**。
          于是"没收到数据"能被一句话分成两种截然不同的原因 ——
            · `matched_subscribers == 0` ⟹ 根本没接上（QoS 不兼容 / 话题名写错）
            · `matched_subscribers > 0` 却收不到 ⟹ 才是 DUT 的逻辑/判据问题
          没有这个数，两种情况在测试日志里长得一模一样。
        """
        return self._pubs[topic_key].get_subscription_count()

    # ☠️ 更正（2026-09-22）：这里本来有一个 `set_mode_clients()`，实现是
    #     `self._set_mode_srv.get_num_clients()` —— **rclpy 的 Service 没有这个方法**，
    #     而它只被 `evidence()` 调用 ⟹ 后果是：**任何一次 bench 超时/失败都会变成
    #     AttributeError，而不是打出证据**（bug 藏在失败路径上，所以一直是绿的）。
    #   ★ 而且这个数在 rclpy Humble 上**根本拿不到**：Node 只暴露 `count_publishers` /
    #     `count_subscribers`，没有任何服务客户端计数的 API（第一手：rclpy/node.py 里
    #     grep `def get_.*_by_service` / `count_clients` 均无命中）。
    #   ⟹ 判断"DUT 的 /mavros/set_mode 客户端接上没有"只能看 `mode_requests` 有没有到过
    #     （已经打在 evidence 里）。

    def wait_for_subscribers(
        self, topic_key: str, count: int = 1, timeout_s: float = 10.0,
    ) -> bool:
        """等 DUT 在某个话题上接上（判据见 `matched_subscribers`）。"""
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            if self.matched_subscribers(topic_key) >= count:
                return True
            time.sleep(0.02)
        return False

    # ─────────────────── 收 DUT 的指令 ───────────────────

    def _on_setpoint(self, msg: PositionTarget) -> None:
        with self._lock:
            self.setpoints.append(Setpoint(at_s=self.t(), msg=msg))

    def _on_attitude_setpoint(self, msg: AttitudeTarget) -> None:
        """`/mavros/setpoint_raw/attitude` 的回调 —— **原样记下我们发出去的每个字段**。

        ★ 这里**不做任何解释/换算**（不取负、不归一化、不判对错）：记录层的职责是"抄下来"，
          判据层（`test_mpc_node.py`）才拿它去比。把解释揉进来会让"记录"与"判据"共用同一个
          可能出错的假设 —— 那正是 N1/N2 这类符号错误能静默通过的原因。
        """
        with self._lock:
            self.attitude_setpoints.append(AttitudeSetpoint(at_s=self.t(), msg=msg))

    def _on_set_mode(self, request, response):
        policy = self._set_mode_policy
        response.mode_sent = (policy == "ok")
        with self._lock:
            self.mode_requests.append(
                ModeRequest(at_s=self.t(), custom_mode=request.custom_mode,
                            sent=response.mode_sent))
        if response.mode_sent and self.follow_set_mode:
            self._pending_mode = request.custom_mode
            self._pending_mode_at_s = self.t() + self.mode_transition_delay_s
        return response

    def _on_arming(self, request, response):
        policy = self._arming_policy
        response.success = (policy == "ok")
        response.result = 0 if response.success else 1
        with self._lock:
            self.arming_requests.append(
                ArmingRequest(at_s=self.t(), value=bool(request.value)))
        if response.success and self.follow_arming:
            # 模拟 PX4 真的解锁了：下一次 `/mavros/state`（1 Hz）发布时 armed 变真
            # ⟹ DUT 看到的上升沿**必然滞后，最多 1 s**（这是真的，不是夹具的毛病）。
            self.armed = bool(request.value)
        return response

    # ─────────────────── 故障注入（主线程调用）───────────────────

    def mute(self, *keys: str) -> None:
        """让这些话题**停止发布**（不是"发空值"——真 mavros 静默时确实是一条都不发）。

        ★ 为什么这比"发空消息"更重要：控制节点的判据是"多久没收到一帧"，
          发空值等于"收到了"（判据恒真）。夹具必须能造出**真的没有**。
        """
        for k in keys:
            if k not in self.SILENCEABLE:
                raise ValueError(f"不可静默的话题：{k}（可选 {self.SILENCEABLE}）")
            self._muted.add(k)

    def unmute(self, *keys: str) -> None:
        for k in keys:
            self._muted.discard(k)

    def set_state(self, **fields) -> None:
        """改 `/mavros/state` 的字段（`connected` / `armed` / `mode` / `guided` / `manual_input`）。"""
        for key, value in fields.items():
            if not hasattr(self, key):
                raise ValueError(f"/mavros/state 没有字段 {key}")
            setattr(self, key, value)

    def synth_disconnect(self) -> None:
        """造 mavros **自己那帧合成断连状态**（ticket 18）。

        真 mavros 在链路超时后（参数 `conn_timeout`，默认 **10 s**）会**自己往
        `/mavros/state` 上发一帧** `connected=false, armed=false, guided=false, manual_input=false, mode=""`、
        `system_status=UNINIT` —— `sys_status.cpp:903-911` 的 `publish_disconnection()`，
        由 `:1411` 的 `connection_cb(false)` 触发（mavros 2.15.1 @ `22ae5b7`，第一手核实）。

        ★ 两个要点必须复刻对，否则测的就不是真机那个形状：

          1. **只发一帧，此后静默** —— state 只在 `handle_heartbeat` 里发 ⟹ 链路断着就没有下一帧。
             ⟹ 本方法发完那一帧就把 `state` 静默掉（`mute` 的语义：真的一条都不发）。
          2. **那帧里的 `armed=false` 是常量占位值，不是读数** —— 真 mavros 直接填 `false`，
             与它对飞控的了解无关。控制节点若把它当成"落地上锁"，就会把闩锁**在空中**清掉，
             这正是 ticket 18 的病灶。⟹ 本方法**不改**夹具自己的 `connected`/`armed`/`mode`
             属性（那些代表"下一帧真状态会说什么"），只发那一个常量帧。

        ⟹ 恢复用 `reconnect_state(**fields)`（= 真 mavros 收到下一帧 HEARTBEAT）。
        ⚠️ 帧由**定时器线程**发（真 mavros 也是从 wall timer 里发的）：主线程只置两个属性，
           下一拍（≤1 s）才发出去 —— 本文件的线程纪律见类 docstring。
        """
        self._pending_disconnect_frame = True
        self.mute("state")

    def reconnect_state(self, **fields) -> None:
        """链路恢复（= 真 mavros 收到下一帧 HEARTBEAT）：改回字段并恢复 1 Hz 发布。

        ★ 与 `synth_disconnect()` 是一对：那帧合成状态之后 `state` 是**静默**的，
          必须显式恢复，否则后面的用例等的是一个永远不来的话题。
        """
        self.set_state(**fields)
        self.unmute("state")

    def set_position(self, x: float, y: float, z: float) -> None:
        """搬动假 EKF 的位置（**ROS/ENU 口径，z 向上为正**）。用它可以造"飞机被移动"的场景。

        ⚠️ 是 ENU 不是 NED：它进的是 `/mavros/local_position/pose`，而 mavros 出向已做过
        NED→ENU（`local_position.cpp:157`）⟹ 这里的 z 就是"离地多高"。
        真 PX4 内部的 local NED 是 z 向下为正，**在 ROS 这一侧看不到那个口径**
        （DUT 把这个位置原样转给 setpoint 时，mavros 入向会再转回 NED，两边恰好抵消）。
        """
        self.position = (float(x), float(y), float(z))

    def set_voltage(self, voltage: float) -> None:
        self.voltage = float(voltage)

    def set_pos_horiz_abs(self, ok: bool) -> None:
        """改假 EKF2 自报的"水平位置有绝对参考"（`/mavros/estimator_status.pos_horiz_abs`）。

        ★ ticket 16 的**故障注入原语**：`False` 造的是"动捕帧还在正常到、`local_position`
        也还在发，但 EKF2 说它的水平位置没有绝对参考" —— 这正是 `mocap_timeout` /
        `mocap_stamp_stalled` 那两条判据**看不见**的一类失效，也是本判据存在的理由。

        ⚠️ 它**不改变** `local_position` 的任何读数（真 PX4 也是这样：失效信息只走状态位，
        不合成 NaN）⟹ 夹具若在这里顺手把位置改成 NaN，测的就不是同一条链了。
        """
        self.pos_horiz_abs = bool(ok)

    def set_yaw(self, yaw: float) -> None:
        """改假 EKF 报的机头朝向（**ROS/ENU：绕 +z 的 yaw，逆时针为正，弧度**）。

        ★ 为什么 ticket 08 才需要它：ticket 05 里 setpoint 的 yaw 是个固定参数（0），
          夹具从来不必让"飞机会转向"。ticket 08 起 yaw = **起飞机头**（起飞前跟当前机头，
          进入 TAKEOFF 那一刻锁一次、之后不变）⟹ 不把机头转到非零角度，
          "锁定"与"跟随"在台架上**无法区分**（两者都恒等于 0，测试会在两个都成立的情况下变绿）。

        ⚠️ 这是 ENU 口径：它进的是 `/mavros/local_position/pose` 的四元数，而 mavros 出向
        已做过 NED→ENU（`local_position.cpp:157`）。真 PX4 内部那个 NED yaw 在 ROS 这一侧看不到。
        """
        self.yaw = float(yaw)

    def set_velocity(self, vx: float, vy: float, vz: float) -> None:
        """改假 EKF 报的速度（**ROS/ENU 口径，m/s，z 向上为正**）。

        ⚠️ 与 `set_position` 是**两条独立的话题**（真 mavros 里位置与速度是同一个回调发的，
        但 DUT 订阅的是两条话题）⟹ 夹具可以造出"位置没变但速度很大"这种**内部不自洽**的场面。
        这正是要的：悬停稳定判据是 `位置误差 ∧ 速率` 两条**同时**成立，
        只动位置是证伪不了速率那一半的。
        """
        self.velocity = (float(vx), float(vy), float(vz))

    def set_rc(self, channels: Sequence[int]) -> None:
        self.rc_channels = [int(c) for c in channels]

    def set_rc_channel(self, channel: int, value: int) -> None:
        """改一路遥控通道（**通道号从 1 起算**，与遥控器上的标号一致）。"""
        ch = list(self.rc_channels)
        while len(ch) < channel:
            ch.append(1500)
        ch[channel - 1] = int(value)
        self.rc_channels = ch

    def set_mode_policy(self, policy: str) -> None:
        """`'ok'` / `'fail'` / `'absent'`。见 `__init__` 的说明。"""
        if policy not in ("ok", "fail", "absent"):
            raise ValueError("policy 只能是 ok / fail / absent")
        self.set_mode_service_available(policy != "absent")
        self._set_mode_policy = policy

    def set_arming_policy(self, policy: str) -> None:
        """`'ok'` / `'fail'` / `'absent'` —— 与 `set_mode_policy` 同一套语义，但**结果字段不同**：
        `CommandBool` 的应答是 `success` + `result`（MAV_RESULT 码），不是 `mode_sent`。
        ★ `'fail'` 造的是"飞控**明确拒绝**了这个请求"（`result = 1` = TEMPORARILY_REJECTED），
        这是 DUT 的"报原因 + 退回 STANDBY、不自动重试"那条路径唯一的注入面。
        """
        if policy not in ("ok", "fail", "absent"):
            raise ValueError("policy 只能是 ok / fail / absent")
        self.set_arming_service_available(policy != "absent")
        self._arming_policy = policy

    def set_arming_service_available(self, available: bool) -> None:
        """生成/销毁 `/mavros/cmd/arming` 服务（模拟"mavros 还没起来"）。
        ⚠️ 同 `set_mode_service_available`：运行时增删服务要等 DDS 发现传播。"""
        if available and self._arming_srv is None:
            self._arming_srv = self.node.create_service(
                CommandBool, self.topics["arming"], self._on_arming)
        elif not available and self._arming_srv is not None:
            self.node.destroy_service(self._arming_srv)
            self._arming_srv = None

    def set_mode_service_available(self, available: bool) -> None:
        """生成/销毁 `/mavros/set_mode` 服务 —— 模拟"mavros 还没起来"。

        ⚠️ 运行时增删服务要等 DDS 发现传播（本机通常 <0.5 s），测试里请用
        `wait_for(... service_is_ready ...)` 或留足时间再断言，不要立刻断言。
        """
        if available and self._set_mode_srv is None:
            self._set_mode_srv = self.node.create_service(
                SetMode, self.topics["set_mode"], self._on_set_mode)
        elif not available and self._set_mode_srv is not None:
            self.node.destroy_service(self._set_mode_srv)
            self._set_mode_srv = None

    # ─────────────────── 取记录 ───────────────────

    def setpoints_between(self, from_s: float, to_s: float) -> List[Setpoint]:
        with self._lock:
            return [s for s in self.setpoints if from_s <= s.at_s <= to_s]

    def attitude_setpoints_between(self, from_s: float, to_s: float) -> List[AttitudeSetpoint]:
        """窗口内的 `/mavros/setpoint_raw/attitude` 帧（判"闸门关着时零帧"要用它）。"""
        with self._lock:
            return [s for s in self.attitude_setpoints if from_s <= s.at_s <= to_s]

    def attitude_rate_hz(self, from_s: Optional[float] = None,
                         to_s: Optional[float] = None) -> float:
        """某段窗口内实测的**下行 setpoint 频率**。

        ★ 与 `Scenario.setpoint_rate_hz` 同一条纪律（**只在窗口上算平均，别断言相邻两帧的间隔**：
          rcl 的定时器是从回调开始时刻重新计时的，回调偶尔超时就会立刻补跑）。这里自带一份
          是因为新下行走的是**另一条话题**，而 `Scenario.setpoint_rate_hz` 读的是
          `/mavros/setpoint_raw/local` 那条（Phase 1 的 `control` 的）—— 两者不是一个流。
        """
        with self._lock:
            sps = list(self.attitude_setpoints)
        if not sps:
            return 0.0
        t_from = sps[0].at_s if from_s is None else from_s
        t_to = sps[-1].at_s if to_s is None else to_s
        n = sum(1 for s in sps if t_from <= s.at_s <= t_to)
        span = t_to - t_from
        return (n / span) if span > 0 else 0.0

    def attitude_publishers(self) -> int:
        """`/mavros/setpoint_raw/attitude` 上**与本夹具的收端能接上的** DUT 发布者有几个。

        ★ 这是**发布方向**上的那面镜子（`matched_subscribers` 是订阅方向的）。

        ☠️☠️ **2026-10-13（ticket 13 §四 现场）这里原来写的是 `self._attitude_sub
        .get_publisher_count()`** —— **rclpy 的 `Subscription` 根本没有这个方法**
        （`AttributeError`，第一手；`get_subscription_count()` 只长在 `Publisher` 上）。
        已按"读发现信息 + 自己判兼容"重写。★ 但重写的同时**必须**把下面这段话说对，
        因为旧注释里的因果是**反的**（见下一条 ⚠️）。

        ⚠️ **它测不出 DUT 把 QoS 写错**，这是 DDS 的 RxO 规则决定的：本夹具的收端是
          `SensorDataQoS()` = best_effort + volatile，而兼容性只要求 **offered ≥ requested**
          ⟹ DUT 发 reliable（ROS 2 的默认！）或 transient_local **照样兼容、照样接上**，
          这个数依然是 1。判"DUT 写没写对"只能靠 `attitude_publisher_qos()` **直接读**
          （那一对分工写在它的 docstring 里）。
          ⟹ 这个数的实际用途是**负面的那半**：为 0 就说明"这条话题上没有能接上的发布者" ——
            话题名写错 / DUT 没起 / DUT 因参数被拒**当场退出** 全都长这样，而 `ros2 topic list`
            看不出区别。

        ⚠️⚠️ **`Node.get_publishers_info_by_topic` 读的是发现库，它不做 QoS 过滤**
          （第一手，2026-10-13，本机 Humble + rmw_fastrtps，四格全跑）：

          | 发布方提供 | 订阅方要求 | 实收帧数 | `get_publishers_info_by_topic` |
          |---|---|---|---|
          | RELIABLE | RELIABLE | 100 | 1 |
          | RELIABLE | BEST_EFFORT | **100** | 1 |
          | BEST_EFFORT | RELIABLE | **0** | **1** |
          | BEST_EFFORT | BEST_EFFORT | 100 | 1 |

          ⟹ 两件事一起钉死了：① RxO 就是 `offered ≥ requested`（第 2 行 100 帧，
          即"best_effort 收端接得住 reliable 发端"）；② **不兼容的那一格（第 3 行）
          发现库里照样有 1 条** ⟹ "有几个发布者"与"有几个**接得上**"是两个数，
          后者必须自己算（`_qos_accepts`）。
          ☠️ **第一版探针把这两条都测错了**：它没调 `rclpy.spin_once`，回调从不执行
          ⟹ **四格全是 0 帧**，于是"RELIABLE 发 → BEST_EFFORT 收 = 0 帧"被误读成
          "这两个不兼容"。⚠️ 教训：**判"收没收到"之前先确认回调真的会跑** ——
          一个恒为 0 的计数器看起来既像"不兼容"也像"没 spin"。
          ⚠️ 顺带同一手实测把它 README 里那两条钉死了：发现信息里 `history` 回来是
            `UNKNOWN(3)`、`depth` **恒为 0**（与发布方实际写的值无关）⟹ 兼容判定里
            只能用 `reliability` / `durability` 两项。
        """
        mine = self._attitude_sub.qos_profile
        return sum(1 for info in self.node.get_publishers_info_by_topic(self.topics["attitude"])
                   if _qos_accepts(offered=info.qos_profile, requested=mine))

    def attitude_sub_qos(self) -> QoSProfile:
        """夹具**自己**在 `/mavros/setpoint_raw/attitude` 上的订阅 QoS（给"照抄真 mavros"那条断言）。

        ★ 它读的是**夹具的** profile（不是 DUT 的）⟹ 属于 seam 1 允许的"读夹具"范围。
          ⛔ 谁把它改成默认 QoS，"DUT 发端 QoS 写错 ⟹ 永不匹配"这一类静默失效在 seam 1 上
          就再也不会出现（见模块 docstring 的那段 ★）。
        """
        return self._attitude_sub.qos_profile

    def attitude_publisher_qos(self) -> Optional[QoSProfile]:
        """DUT 在 `/mavros/setpoint_raw/attitude` 上**发布方实际提供的** QoS（读不到返回 None）。

        ★ 与 `attitude_publishers()` 分工不同：那个数"接上了几个"，这个读"接上来的那一方报的是什么"。
          两者**都要**：`== 1` 只说明"兼容"，而 best_effort 与 reliable 对 `SensorDataQoS()`
          的收端**都兼容**（offered ≥ requested）⟹ 单看 `== 1` 分不出 DUT 写没写对。
        ⚠️ 只能对 `reliability` / `durability` 下断言：本套栈（Humble + rmw_fastrtps）的发现信息里
          `history` 回来是 `UNKNOWN`、`depth` **恒为 0**，与发布者实际写的值无关
          （第一手与反例见 `fake_px4.publisher_qos` 的 ⚠️⚠️ 与 `test_fake_px4_selftest` 那条）。
        ⚠️ 依赖 DDS 发现：DUT 还没发布过时返回 None，断言前先等 `attitude_publishers() >= 1`。
        """
        infos = self.node.get_publishers_info_by_topic(self.topics["attitude"])
        return infos[0].qos_profile if infos else None

    # ─────────────────── 人看的证据 ───────────────────

    def evidence(self) -> str:
        """出事时把人话证据打出来 —— 比"断言失败"多给的是**记录本身**。
        沿用 tools/prereq-evidence/ 的惯例：输出自描述的 markdown，不自动写文档。"""
        lines = [
            "### 假 mavros 记录",
            "",
            f"- setpoint：{len(self.setpoints)} 帧"
            + (f"，首帧 {self.setpoints[0].at_s:.2f} s，末帧 {self.setpoints[-1].at_s:.2f} s"
               if self.setpoints else "（**一帧都没有**）"),
            f"- `/mavros/set_mode` 调用：{len(self.mode_requests)} 次"
            + (f"（{', '.join(f'{r.custom_mode}@{r.at_s:.2f}s' for r in self.mode_requests)}）"
               if self.mode_requests else ""),
            # ★ ticket 08 起 `/mavros/cmd/arming` 也成了被测行为之一 ⟹ 记录要打在证据里。
            #   没有这一行时，"DUT 到底发没发解锁请求"只能靠读 DUT 日志猜。
            f"- `/mavros/cmd/arming` 调用：{len(self.arming_requests)} 次"
            + (f"（{', '.join(f'{r.value}@{r.at_s:.2f}s' for r in self.arming_requests)}）"
               if self.arming_requests else ""),
            # ★ ticket 13 §四① 起：Phase 4 的下行改走 `/mavros/setpoint_raw/attitude` ⟹ 记录要打进证据。
            #   没有这几行时，"我们到底发了什么 body_rate/thrust"只能靠读 DUT 日志猜 —— 而
            #   **N2（body_rate 方向被 mavros 再翻一次）**只有在看着**我们发出去的原值**时才抓得到。
            f"- `/mavros/setpoint_raw/attitude` 帧：{len(self.attitude_setpoints)} 帧"
            + (f"，首帧 {self.attitude_setpoints[0].at_s:.2f} s"
               f"，末帧 {self.attitude_setpoints[-1].at_s:.2f} s"
               if self.attitude_setpoints else "（**一帧都没有**）"),
            *(
                [
                    f"  - 末帧原值：type_mask={self.attitude_setpoints[-1].type_mask}"
                    f" body_rate(x,y,z)={tuple(round(v, 6) for v in self.attitude_setpoints[-1].body_rate)}"
                    f" thrust={self.attitude_setpoints[-1].thrust:.6f}"
                    f" stamp_ns={self.attitude_setpoints[-1].stamp_ns}",
                ]
                if self.attitude_setpoints else []
            ),
            f"- `/mavros/setpoint_raw/attitude` 上**接得上**的 DUT 发布者："
            f"{self.attitude_publishers()}"
            f"（★ 为 0 而 DUT 日志说'在发' ⟹ 要么话题名不对、要么它根本没建发布者；"
            f"⛔ 别读成'QoS 不兼容' —— 本夹具的收端是最弱的一档，见 `attitude_publishers` 的 ⚠️）",
            f"- 匹配到的 DUT 订阅者："
            + ", ".join(f"{k}={self.matched_subscribers(k)}" for k in self._pubs),
            f"- 当前状态：connected={self.connected} armed={self.armed} mode={self.mode}"
            f"；静默中={sorted(self._muted) or '无'}"
            f"；position={self.position} velocity={self.velocity}",
        ]
        return "\n".join(lines)
