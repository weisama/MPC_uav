"""假 PX4（uXRCE-DDS 那一半）—— seam 1 里 Phase 2 的"外部世界"。

★ 与 `fake_mavros.py` 的分工：那个假的是 **mavros**（`/mavros/*`），这个假的是 **PX4 自己**
  暴露给 uXRCE-DDS 的 `/fmu/*` 接口（Phase 2 走的是这条路，不经 mavros）。
  两者**可以同时存在**（Phase 1 的 `(A)` 链与 Phase 2 的 `(C)` 链并行，见
  [`.scratch/phase2/issues/06`](../../../.scratch/phase2/issues/06-mpc-node-fake-px4-fixture.md)
  的「回退」验收项），但一个场景里通常只起其中一个。

★★ **它假什么、不假什么**（本文件存在的理由是"真 PX4 没法被要求，假件可以"）：

   | 假 | 不假 |
   |---|---|
   | `/fmu/out/vehicle_status` 的**话题名、QoS、字段语义、2 Hz 定频 + 变化即刻发** | `COM_OF_LOSS_T` 超时退出 OFFBOARD（**真 PX4 / SITL 的行为**，已在 ticket 05 的 SITL 验证里实测过） |
   | `/fmu/out/vehicle_local_position` + `vehicle_attitude` 的**话题名、QoS、定频、字段单位与坐标系（NED/FRD）** | **EKF2 怎么估出那些数**（A 档的夹具给的是**我们指定的**状态：位置/速度/四元数都是注入的，不是估的） |
   | `/fmu/in/*` 的**订阅侧 QoS**（决定 DUT 的发布方能不能匹配上） | EKF2 怎么融合动捕、模式需求检查、prearm 的**具体**检查项 |
   | PX4 反序列化器对 `timestamp` 的读法（`recorded_age_s`，**只报告、不据此行动**） | 时钟同步的真实滞后 Δ（见 `recorded_age_s` 的说明） |
   | armed / nav_state / pre_flight_checks_pass 的**注入** | `vehicle_command` 的处理（MPC（模型预测控制）节点是纯控制器，不发命令 ⟹ 夹具也不处理）|

★ **为什么状态话题要"注入"而不是"仿真"**（ticket 09）：A 档那一侧要回答的是
  "**我们的约定与真 PX4 一致**"——即"给定一个**已知**的 NED/FRD 状态，节点发出的指令方向对不对"。
  仿真一个会自己动的被控对象，会让"指令错了"与"飞机真的朝那边动了"搅在一起
  （闭环里两者互相掩盖）；注入一个已知状态 ⟹ 预期指令**可以手算出来**。
  真闭环（"稳定悬停在 1 m"）是 SITL 的活，不在这一层。

  ⟹ "杀节点 → PX4 自行退出 OFFBOARD"这类**依赖 PX4 侧行为**的验收项不在这里做（SITL 的活）。

★ 运行方式：只被 `Scenario` 创建（`Scenario(px4=True)`），不要单独跑（它没有 main）。
"""

from __future__ import annotations

import math
import statistics
import time
from dataclasses import dataclass
from typing import Dict, List, Optional, Sequence, Tuple

from px4_msgs.msg import (OffboardControlMode, VehicleAttitude,  # noqa: F401
                          VehicleLocalPosition, VehicleRatesSetpoint, VehicleStatus)
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)

# ───────────────────────────── PX4 的枚举（px4_msgs 逐字一致）─────────────────────────────
#
# ⚠️ 这些**不是**随便挑的数：`VehicleStatus.msg` 里逐行写着同样的值，夹具抄的是它们。
#    写错一个数的后果是"夹具说它在 OFFBOARD，DUT 看到的却是别的模式"，而两边都不报错。
NAV_STATE_MANUAL = 0
NAV_STATE_POSCTL = 2
NAV_STATE_OFFBOARD = 14
ARMING_STATE_STANDBY = 1
ARMING_STATE_ARMED = 2

#: `Commander.hpp:198` `COMMANDER_MONITORING_INTERVAL{10_ms}` —— commander 主循环的周期
COMMANDER_LOOP_HZ = 100.0

#: `Commander.cpp:1790` 的注释逐字："publish states (armed, control_mode, vehicle_status,
#: failure_detector_status) at **2 Hz or immediately when changed**"，判据是
#: `now >= _vehicle_status.timestamp + 500_ms || _status_changed || nav_state_or_failsafe_changed
#: || !(_actuator_armed == actuator_armed_prev)`。
#: ★ 两条都要照抄：只抄 2 Hz 会让"注入 arm ⟹ 最多等 500 ms"变成夹具的假象（真机是 ≤10 ms），
#:   而 DUT 里任何"等状态变化"的判据都会被那个假象拖慢。
STATUS_PERIOD_S = 0.5

#: A 档那两条状态话题的发布频率 [Hz]。★ 第一手（2026-09-28，z0 SITL：
#: `vehicle_local_position` **32.5 Hz**、`vehicle_attitude` **31.6 Hz**）——
#: 两条是**各自独立**的流（EKF2 各按自己的更新节奏发），但频率只差 3 %，
#: 夹具用**同一个**定时器发两条并不掩盖任何被测行为：
#: DUT 必须**各自**判新鲜度这件事，由"单独静默其中一条"来考（`mute("attitude")`），
#: 而不是靠"两条频率不一样"。
#: ⚠️ 别把它设成 ~2 Hz（那会撞上 `status` 的节奏）：真 PX4 上这两条是**几十 Hz**，
#:   而 `px4_timeout_s = 0.5` 这个默认值（16 拍）正是按它取的。
STATE_HZ = 32.0


# ───────────────────────────── QoS：与 PX4 的 uXRCE-DDS 端点逐字对齐 ─────────────────────────────

def px4_out_qos() -> QoSProfile:
    """PX4 **发布** `/fmu/out/*` 时用的 QoS —— 夹具照抄它。

    第一手（z0 固件 `4a0e65f2` 的 v1.14.3 源码，`src/modules/uxrce_dds_client/utilities.hpp:59-63`
    的 `create_data_writer`）：

        uxrQoS_t qos = { .durability = UXR_DURABILITY_TRANSIENT_LOCAL,
                         .reliability = UXR_RELIABILITY_BEST_EFFORT,
                         .history = UXR_HISTORY_KEEP_LAST, .depth = 0 };

    ★ **BEST_EFFORT 这一条是订阅侧最容易踩的坑**：DUT 写 `RELIABLE`（ROS 2 的默认值）
      就**一个回调都不进**，而 `ros2 topic list` / `ros2 topic hz` 照样列得出来。
    ★ TRANSIENT_LOCAL 在这一侧是**宽松**的（DUT 订阅 VOLATILE 也兼容）——
      但这个字段决定了"晚起的订阅者会不会拿到重放"，所以照抄真值，不替它降级。
    ★ `depth = 0`：XRCE 侧 0 表示"不额外排队"。ROS 侧写 1 —— 2 Hz 的状态话题上，
      排队深度不是被测行为，而写 1 能让"夹具自己积压"不可能发生。
    """
    return QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                      reliability=ReliabilityPolicy.BEST_EFFORT,
                      durability=DurabilityPolicy.TRANSIENT_LOCAL)


def px4_in_qos(depth: int = 1) -> QoSProfile:
    """PX4 **订阅** `/fmu/in/*` 时用的 QoS（`utilities.hpp:114-119` 的 `create_data_reader`）：

        uxrQoS_t qos = { .durability = UXR_DURABILITY_VOLATILE,
                         .reliability = UXR_RELIABILITY_BEST_EFFORT,
                         .history = UXR_HISTORY_KEEP_LAST, .depth = queue_depth };

    其中 `queue_depth = uORB::DefaultQueueSize<...>::value * 2`（生成代码里逐话题写死）。

    ★ 夹具**必须**照抄 `VOLATILE` 而不是"顺手也写 TRANSIENT_LOCAL"：DUT 若只发 VOLATILE，
      在夹具里照样能匹配、在真机上也照样能匹配；夹具若要求 TRANSIENT_LOCAL，就会把
      "DUT 少写了一个字段"变成**夹具独有的**失败 —— 那是夹具在说谎。
      "DUT 有没有按票面要求发 TRANSIENT_LOCAL"是另一件事，用 `publisher_qos()` 单独量。
    """
    return QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=depth,
                      reliability=ReliabilityPolicy.BEST_EFFORT,
                      durability=DurabilityPolicy.VOLATILE)


# ───────────────────────────── 记录下来的东西 ─────────────────────────────

@dataclass
class StatusFrame:
    """夹具**发出**的一帧 `/fmu/out/vehicle_status`。"""
    at_s: float          # 发出时刻（相对 Scenario 的 t0）
    timestamp_us: int    # 帧上的 `timestamp`（= 夹具的 PX4 时钟，见 `hrt_us`）
    msg: VehicleStatus


@dataclass
class StateFrame:
    """夹具**发出**的一帧状态（`vehicle_local_position` / `vehicle_attitude`）。

    `topic_key` 是 `"local_position"` / `"attitude"`（与 `OUT_TOPICS` 同一套键，
    和 `mute()` / `matched_subscribers()` 用法一致）。
    """
    at_s: float
    timestamp_us: int
    topic_key: str
    msg: object


@dataclass
class DutFrame:
    """DUT 发来的一帧 `/fmu/in/*`。`at_s` 是**到达时刻**，不是 `msg.timestamp`。

    ★ 两个时间要分开看（与 `FakeMavros.Setpoint` 同一条纪律）：
      断言"频率/流断没断"用 `at_s`（DUT 到底发了没有）；
      断言"时间戳单位对不对"用 `msg.timestamp`。
    """
    at_s: float
    msg: object
    topic_key: str

    @property
    def timestamp_us(self) -> int:
        return int(self.msg.timestamp)


# ───────────────────────────── 主类 ─────────────────────────────

class FakePx4:
    """假的 PX4（uXRCE-DDS 侧）。**所有 rclpy 调用都发生在 executor 线程里**
    （定时器/订阅回调），主线程只改普通 Python 属性 + 读记录 ⟹ 与 `FakeMavros` 同一套线程纪律。

    要注入的故障全部是"改一个属性"或"掐一条话题"：`set_armed` / `set_nav_state` /
    `set_pre_flight_checks_pass` / `mute` / `unmute`。
    """

    SILENCEABLE = ("status", "local_position", "attitude")

    #: 夹具**发**的话题（DUT 订阅它们）。★ A 档那两条（`state_source=px4`）**默认就在发**：
    #: 夹具不是被测对象，"多一条没人订的话题"在 DDS 上什么也不影响（订阅方为 0 而已），
    #: 而"按档决定夹具发不发"会让**场景与档位耦合** —— 那正是这类夹具最容易长歪的地方。
    OUT_TOPICS = {
        "status": "/fmu/out/vehicle_status",
        "local_position": "/fmu/out/vehicle_local_position",
        "attitude": "/fmu/out/vehicle_attitude",
    }
    #: 夹具**收**的话题（DUT 发布它们）
    IN_TOPICS = {
        "rates": "/fmu/in/vehicle_rates_setpoint",
        "mode": "/fmu/in/offboard_control_mode",
    }

    def __init__(
        self,
        node: Node,
        clock,
        topics: Optional[Dict[str, str]] = None,
        armed: bool = False,
        nav_state: int = NAV_STATE_POSCTL,
        pre_flight_checks_pass: bool = True,
        vehicle_type: int = VehicleStatus.VEHICLE_TYPE_ROTARY_WING,
    ) -> None:
        self.node = node
        # 时刻的尺子由 Scenario 提供（`clock.t()`），这样"记录里的一秒"与"时序里的一秒"是同一秒。
        self.clock = clock

        t = dict(self.OUT_TOPICS)
        t.update(self.IN_TOPICS)
        t.update(topics or {})
        self.topics = t

        # —— 可注入的状态（主线程改，executor 线程读）——
        # ⚠️ 全是"读一次就够"的标量替换，不做就地修改 ⟹ 不用锁（同 `FakeMavros`）。
        self._muted: set = set()
        self._status_dirty = False
        self.armed = bool(armed)
        self.nav_state = int(nav_state)
        self.pre_flight_checks_pass = bool(pre_flight_checks_pass)
        self.vehicle_type = int(vehicle_type)

        # —— A 档的状态（NED/FRD，**注入**不是仿真，见文件头 ★）——
        # ★ 初始值 = "地面静息、机头朝北、水平"：NED 原点、零速度、单位四元数。
        #   它**不代表真值**（真机上这是 EKF 的输出），但它是最容易手算指令方向的起点。
        self.position_ned: Tuple[float, float, float] = (0.0, 0.0, 0.0)
        self.velocity_ned: Tuple[float, float, float] = (0.0, 0.0, 0.0)
        self.quat_ned_frd: Tuple[float, float, float, float] = (1.0, 0.0, 0.0, 0.0)
        #: `xy_valid` / `z_valid` / `v_xy_valid` / `v_z_valid`（每帧照发；见 `describe_state`）
        self.state_valid: bool = True
        self.dead_reckoning: bool = False
        self.eph: float = 0.0
        self.epv: float = 0.0

        # —— 记录 ——
        self.status_frames: List[StatusFrame] = []
        self.state_frames: List[StateFrame] = []
        self.frames: List[DutFrame] = []
        self._last_status_at_s: Optional[float] = None
        self._last_state_at_s: Optional[float] = None

        # —— 上游（与真 PX4 同名同 QoS）——
        self._status_pub = node.create_publisher(
            VehicleStatus, t["status"], px4_out_qos())
        self._pos_pub = node.create_publisher(
            VehicleLocalPosition, t["local_position"], px4_out_qos())
        self._att_pub = node.create_publisher(
            VehicleAttitude, t["attitude"], px4_out_qos())
        self._pubs = {"status": self._status_pub,
                      "local_position": self._pos_pub,
                      "attitude": self._att_pub}

        # —— 下游（PX4 是订阅方，QoS 照抄 `utilities.hpp:114-119`）——
        self._rates_frames: List[DutFrame] = []
        self._mode_frames: List[DutFrame] = []
        self._rates_sub = node.create_subscription(
            VehicleRatesSetpoint, t["rates"],
            lambda m: self._on_frame("rates", m), px4_in_qos())
        self._mode_sub = node.create_subscription(
            OffboardControlMode, t["mode"],
            lambda m: self._on_frame("mode", m), px4_in_qos())

        # —— 定时器 ——
        # ★ 按 **commander 主循环的周期**（10 ms）跑，而不是按 500 ms：见 `STATUS_PERIOD_S`
        #   的说明 —— "变化即刻发"这条只有在上限足够高时才复刻得出来。
        self._timer = node.create_timer(
            1.0 / COMMANDER_LOOP_HZ, self._tick_status)
        # ★ 状态那两条**定频照发**（不搞"变了才发"）：真 PX4 上它们是 EKF2 的更新节奏，
        #   与"值变没变"无关（悬停时数值几乎不变，但流一刻也不停）。
        self._state_timer = node.create_timer(1.0 / STATE_HZ, self._tick_state)

    # ─────────────────── 时间 ───────────────────

    def t(self) -> float:
        """相对 Scenario 的 t0 的秒数 —— 与 Timeline 的事件时刻、记录的 `at_s` 同一把尺子。"""
        return self.clock.t()

    def hrt_us(self) -> int:
        """夹具这一侧的 "PX4 时钟"（µs since boot）—— 填 `VehicleStatus.timestamp`。

        ★ 它是**从夹具就位起算**的，与 DUT 填的**墙钟 µs** 是**两个不同的纪元**。
          这不是夹具偷懒，是真机上的事实：PX4 的 `hrt_absolute_time()` 从飞控上电起算，
          而 uORB 消息上的 `timestamp` 全是这个纪元 ⟹ **把 DUT 自己的墙钟时间戳与状态帧的
          `timestamp` 相减毫无意义**（差的是"飞控开了多久"，不是数据新旧）。
          夹具照抄这个性质，正是为了让"拿两个纪元做差"的那类写法在这里就该露馅。
        """
        return int(self.t() * 1e6)

    # ─────────────────── 发布 ───────────────────

    def _tick_status(self) -> None:
        """`Commander.cpp:1786-1806` 的出版节奏（2 Hz 定频 + 变化即刻发）。"""
        now_s = self.t()
        due = (self._last_status_at_s is None
               or now_s >= self._last_status_at_s + STATUS_PERIOD_S)
        if not due and not self._status_dirty:
            return
        self._status_dirty = False
        if "status" in self._muted:
            return
        self._publish_status(now_s)

    def _publish_status(self, now_s: float) -> None:
        msg = VehicleStatus()
        msg.timestamp = self.hrt_us()
        # ★ **只填 DUT 真读的那几位**，其余保持零值（与 `FakeMavros._pub_estimator_status`
        #   同一条纪律）：真 `VehicleStatus` 有四十来个字段，夹具把它们全填成"正常值"会让
        #   "以后有人改成读另一位"这种改动在台架上**照样绿**。留白 ⟹ 读错位立刻表现为"判据恒假"。
        msg.arming_state = ARMING_STATE_ARMED if self.armed else ARMING_STATE_STANDBY
        msg.nav_state = int(self.nav_state)
        msg.nav_state_user_intention = int(self.nav_state)
        msg.nav_state_timestamp = self.hrt_us()
        msg.pre_flight_checks_pass = bool(self.pre_flight_checks_pass)
        msg.vehicle_type = int(self.vehicle_type)
        self._status_pub.publish(msg)

        self._last_status_at_s = now_s
        self.status_frames.append(StatusFrame(
            at_s=now_s, timestamp_us=int(msg.timestamp), msg=msg))

    def _tick_state(self) -> None:
        """A 档那两条状态话题的出版节奏：**定频**（`STATE_HZ`），静默按话题单独生效。"""
        now_s = self.t()
        emitted = False
        for key, build in (("local_position", self._build_local_position),
                           ("attitude", self._build_attitude)):
            if key in self._muted:
                continue
            msg = build()
            self._pubs[key].publish(msg)
            self.state_frames.append(StateFrame(
                at_s=now_s, timestamp_us=int(msg.timestamp), topic_key=key, msg=msg))
            emitted = True
        if emitted:
            self._last_state_at_s = now_s

    def _build_local_position(self) -> VehicleLocalPosition:
        """`VehicleLocalPosition.msg` 里**只有 DUT 真读的那些字段**（与 `_publish_status` 同纪律）。

        DUT 读的（`node_core._measure_px4` + `describe_state`）：`x/y/z`（**NED**，m）、
        `vx/vy/vz`（**NED**，m/s）、四个有效性标志、`dead_reckoning`、`eph` / `epv`、
        `xy_reset_counter`、`timestamp`。
        ⚠️ 其余四十来个字段**留零**（`dist_bottom` / `heading` / `ref_alt`…）：它们是"以后有人
        改成读另一列"时该在这层露馅的地方 —— 填满"正常值"会让那种改动照样绿。
        """
        m = VehicleLocalPosition()
        m.timestamp = self.hrt_us()
        m.x, m.y, m.z = (float(v) for v in self.position_ned)
        m.vx, m.vy, m.vz = (float(v) for v in self.velocity_ned)
        m.xy_valid = m.z_valid = m.v_xy_valid = m.v_z_valid = bool(self.state_valid)
        m.dead_reckoning = bool(self.dead_reckoning)
        m.eph = float(self.eph)
        m.epv = float(self.epv)
        m.xy_reset_counter = 0
        return m

    def _build_attitude(self) -> VehicleAttitude:
        """`q` 是「**FRD 机体 → NED 世界**」（`VehicleAttitude.msg` 里逐字写着），w-first。"""
        m = VehicleAttitude()
        m.timestamp = self.hrt_us()
        m.q = [float(v) for v in self.quat_ned_frd]
        m.quat_reset_counter = 0
        return m

    # ─────────────────── 收 DUT 的帧 ───────────────────

    def _on_frame(self, key: str, msg) -> None:
        frame = DutFrame(at_s=self.t(), msg=msg, topic_key=key)
        if key == "rates":
            self._rates_frames.append(frame)
        else:
            self._mode_frames.append(frame)

    # ─────────────────── 发现屏障（★ 任何"没收到"的断言之前先用它）───────────────────

    def matched_subscribers(self, topic_key: str) -> int:
        """夹具这条话题上有几个订阅者**已经匹配上**（只有 `status` 是夹具发布的话题）。

        ★ 为什么这是夹具里最该先用的一个断言原语：**QoS 不兼容时这里恒为 0**。
          于是"没收到状态"能被一句话分成两种截然不同的原因 ——
            · `== 0` ⟹ 根本没接上（QoS 不兼容 / 话题名写错 / 忘了 `shm` 前缀之类）
            · `> 0` 却收不到 ⟹ 才是 DUT 的逻辑问题
        """
        return self._pubs[topic_key].get_subscription_count()

    def published_frames(self, topic_key: str) -> List[DutFrame]:
        """DUT 发到某条 `/fmu/in/*` 上的全部帧（**只读快照**）。"""
        return list(self._rates_frames if topic_key == "rates" else self._mode_frames)

    def matched_publishers(self, topic_key: str) -> int:
        """DUT 在某条 `/fmu/in/*` 上有几个发布者**已经匹配上**（夹具是订阅方）。

        ★ 它量的是**另一半**：`matched_subscribers` 问"夹具发的 DUT 收得到吗"，
          这个问"DUT 发的夹具收得到吗"。两者都是 0 时才说明 DDS 根本没发现。
        ⚠️ 真正的 QoS 不兼容会表现为 `matched_publishers == 0` **且** 一帧都收不到；
          但"帧收不到"也可能是 DUT 根本没发（未解锁 / 未 offboard 的**正确**行为）
          ⟹ 判据要配正控（先让它该发，再看这里 > 0）。
        """
        return self.node.count_publishers(self.topics[topic_key])

    def wait_for_publishers(self, topic_key: str, count: int = 1,
                            timeout_s: float = 10.0) -> bool:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            if self.matched_publishers(topic_key) >= count:
                return True
            time.sleep(0.02)
        return False

    def wait_for_subscribers(self, topic_key: str, count: int = 1,
                             timeout_s: float = 10.0) -> bool:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            if self.matched_subscribers(topic_key) >= count:
                return True
            time.sleep(0.02)
        return False

    def publisher_qos(self, topic_key: str) -> Optional[QoSProfile]:
        """DUT 在 `/fmu/in/<topic_key>` 上**发布方实际提供的** QoS（读不到返回 None）。

        ★ 为什么单独要一个它：夹具自己的订阅是 `VOLATILE`（照抄 PX4），而票面要求 DUT
          **发** `TRANSIENT_LOCAL` —— 两者**兼容**，所以在"收不收得到"上永远看不出来。
          这一项是唯一能把"票面要求"与"能跑"分开的量。

        ⚠️⚠️ **只能对 `reliability` 与 `durability` 两项下断言**（本机实测，2026-09-28，
          Humble + rmw_fastrtps）：发现信息里的 `history` 回来是 `UNKNOWN(3)`、
          `depth` **恒为 0**，与发布者实际写的值**无关** —— `depth=1` 与 `depth=10`
          读回来一模一样。所以 `depth` 读成 0 **不是**"DUT 没写 depth"的证据，
          谁拿它下断言谁就会得到一个恒红的测试。
          ⟹ 票面「`depth 1`」这一项在这套栈上**黑盒不可观测**（详见
            `test/test_fake_px4_selftest.py` 里那条专门钉住这件事的用例）。
        ⚠️ 它依赖 DDS 发现：DUT 还没起来时返回 None，断言前请先过 `wait_for_publishers`。
        """
        infos = self.node.get_publishers_info_by_topic(self.topics[topic_key])
        return infos[0].qos_profile if infos else None

    # ─────────────────── 故障注入（主线程调用）───────────────────

    def mute(self, *keys: str) -> None:
        """让这些话题**停止发布**（不是"发空值"）。

        ★ 静默在真机上的对应事件是 **uXRCE-DDS 链路/客户端断了**：`/fmu/out/*` 就这样
          一条都不再来，而"发一帧空值"是完全不同的信号（DUT 会当成"收到了"）。
          所以夹具必须能造出**真的没有**。
        """
        for k in keys:
            if k not in self.SILENCEABLE:
                raise ValueError(f"不可静默的话题：{k}（可选 {self.SILENCEABLE}）")
            self._muted.add(k)

    def unmute(self, *keys: str) -> None:
        for k in keys:
            self._muted.discard(k)

    def set_armed(self, armed: bool) -> None:
        """改假 PX4 的解锁状态。★ **下一拍（≤10 ms）就发出去** —— 见 `STATUS_PERIOD_S`。"""
        self.armed = bool(armed)
        self._status_dirty = True

    def set_nav_state(self, nav_state: int) -> None:
        """改假 PX4 的当前模式（`NAV_STATE_OFFBOARD` 等）。变化即刻发。"""
        self.nav_state = int(nav_state)
        self._status_dirty = True

    def set_pre_flight_checks_pass(self, ok: bool) -> None:
        """改 `pre_flight_checks_pass`（DUT 解锁前的门槛之一）。变化即刻发。"""
        self.pre_flight_checks_pass = bool(ok)
        self._status_dirty = True

    # ─────────────────── A 档状态的注入（主线程调用）───────────────────
    #
    # ★ 参数一律是 **NED/FRD**（与真 PX4 的消息同坐标系）：夹具**不提供** ENU 侧的口子。
    #   理由与 `node_core.px4_pose_to_enu` 那条"转换只许有一处"是同一个：
    #   夹具里再写一遍 NED↔ENU，就等于把"到底谁换错了"变成不可回答的问题。
    #   ⟹ 写场景的人要**在 NED 里想**（"机头朝东"= 绕 NED 的 z 转 +90°），
    #     而这正是 A 档要考的那件事。

    def set_position_ned(self, x: float, y: float, z: float) -> None:
        """位置（**NED**，m）：`x` 北、`y` 东、`z` 下（高度 = `−z`）。"""
        self.position_ned = (float(x), float(y), float(z))

    def set_velocity_ned(self, vx: float, vy: float, vz: float) -> None:
        """速度（**NED**，m/s）。★ 这是**动捕那条源没有**的量（issue 15）—— A 档的外推真的在动。"""
        self.velocity_ned = (float(vx), float(vy), float(vz))

    def set_attitude_quat(self, q) -> None:
        """姿态四元数（**w-first**，「FRD 机体 → NED 世界」）。"""
        q = tuple(float(v) for v in q)
        if len(q) != 4:
            raise ValueError(f"四元数要四个分量（w, x, y, z），收到 {q!r}")
        self.quat_ned_frd = q

    def set_yaw_ned(self, yaw_rad: float) -> None:
        """只改**偏航**（绕 NED 的 z 轴，正 = 机头从北转向东），保持水平。

        ★ 它是"机头朝东/朝南"这类场景的**唯一**入口：手写 `(cos, 0, 0, sin)` 很容易写成
          绕错轴，而"姿态错了"在闭环里表现为"飞机朝一边歪"，看着像符号映射错了。
        """
        half = 0.5 * float(yaw_rad)
        self.quat_ned_frd = (math.cos(half), 0.0, 0.0, math.sin(half))

    def set_state_valid(self, ok: bool = True, dead_reckoning: bool = False,
                        eph: float = 0.0, epv: float = 0.0) -> None:
        """改那几个**有效性标志**（`xy_valid` / `z_valid` / `v_*_valid` 一起动）。

        ★ 它存在的理由不是"DUT 会据此停止发布"（**不会**：A 档只判"龄"，第一手证据见
          `node_core.describe_state`）——而是**取证**：这些位要原样进 JSONL，
          好让复盘时能把"EKF 自认为如何"与"它实际差多少"摆在一起看。
        """
        self.state_valid = bool(ok)
        self.dead_reckoning = bool(dead_reckoning)
        self.eph = float(eph)
        self.epv = float(epv)

    # ─────────────────── 取记录 ───────────────────

    def frames_between(self, topic_key: str, from_s: float, to_s: float) -> List[DutFrame]:
        return [f for f in self.published_frames(topic_key)
                if from_s <= f.at_s <= to_s]

    def rate_hz(self, topic_key: str, from_s: Optional[float] = None,
                to_s: Optional[float] = None) -> float:
        """某段窗口内 DUT 在一条 `/fmu/in/*` 上实测的频率。

        ★ 只在**窗口**上算，不要断言相邻两帧的间隔：rcl 的定时器从**回调开始时刻**重新计时，
          回调偶尔超时就会立刻补跑 ⟹ 50 Hz 的流里出现 0 ms 的间隔与 40 ms 的空洞都正常，
          而平均频率仍然对（这条纪律与 `Scenario.setpoint_rate_hz` 同源）。
        """
        frames = self.published_frames(topic_key)
        if not frames:
            return 0.0
        t_from = frames[0].at_s if from_s is None else from_s
        t_to = frames[-1].at_s if to_s is None else to_s
        n = sum(1 for f in frames if t_from <= f.at_s <= t_to)
        span = t_to - t_from
        return (n / span) if span > 0 else 0.0

    def status_rate_hz(self, from_s: Optional[float] = None,
                       to_s: Optional[float] = None) -> float:
        """夹具**自己**发状态的频率（默认应为 ~2 Hz —— 那是真 PX4 的定频）。"""
        frames = self.status_frames
        if len(frames) < 2:
            return 0.0
        a = frames[0].at_s if from_s is None else from_s
        b = frames[-1].at_s if to_s is None else to_s
        n = sum(1 for f in frames if a <= f.at_s <= b)
        return (n / (b - a)) if b > a else 0.0

    def state_rate_hz(self, topic_key: str, from_s: Optional[float] = None,
                      to_s: Optional[float] = None) -> float:
        """夹具自己发某条状态话题的频率（默认应为 ~`STATE_HZ` —— 真 PX4 是 32.5 / 31.6 Hz）。"""
        frames = [f for f in self.state_frames if f.topic_key == topic_key]
        if len(frames) < 2:
            return 0.0
        a = frames[0].at_s if from_s is None else from_s
        b = frames[-1].at_s if to_s is None else to_s
        n = sum(1 for f in frames if a <= f.at_s <= b)
        return (n / (b - a)) if b > a else 0.0

    # ─────────────────── 时间戳取证（★ 单位错是**静默**的）───────────────────

    def stamp_age_s(self, frame: DutFrame, now_s: Optional[float] = None) -> float:
        """DUT 那一帧的时间戳相对**墙钟**的"年龄" [s]（正 = 过去，负 = 未来）。

        ★ 它是 **ticket 05 那套取证开关的落脚点**：夹具不能替 DUT 发时间戳，但可以把
          "DUT 到底发了什么"翻译成一个有物理含义的数。真 PX4 记下的值是
          （生成的反序列化器 `uORB/ucdr/<话题>.h`）：

              if (topic.timestamp == 0) topic.timestamp = hrt_absolute_time();
              else topic.timestamp = math::min(topic.timestamp - time_offset, hrt_absolute_time());

        ⚠️ **本方法刻意不模拟 `time_offset` 的那段秒级滞后 Δ ∈ [0, 1) s**（05 实测 ≈0.18 s）：
          模拟它反而会把"挪 1 s 贴边"这个**边界**抹平（05 的实测结论：挪 1 s 仍进得去、
          挪 2 s 才被判不新鲜）。夹具只回答"这个大数是哪个纪元/单位"，
          "PX4 会不会因此判超时"是 SITL / 真机的活（见文件头的「不假」表）。

        ⚠️ `now_s` 是**墙钟秒**（`time.time()` 那一把尺子），**不是** `Scenario.t()`
          —— 后者是"相对夹具就位的秒数"。本方法第一版默认取了 `self.t()`，于是**正确的 µs
          会被判成 `ns`**：`1.79e15 µs` 减掉一个 ~5 s 的参照，年龄 ≈ +1.79e9 s，正落在
          "ms" 那一档。凡是要与 `msg.timestamp`（DUT 的墙钟）相减的量，参照只能是墙钟。
        """
        now = time.time() if now_s is None else now_s
        return (now * 1e6 - frame.timestamp_us) / 1e6

    def stamp_unit_guess(self, topic_key: str = "rates") -> Tuple[str, float]:
        """猜 DUT 用的是哪个**单位**，返回 `(单位, 中位年龄秒)`。

        判据只有一条：`msg.timestamp` 相对墙钟的比值（真实墙钟 µs ≈ 1.7e15 / 本机运行时长）。
        四种单位差三个数量级，猜错的余地很小：

          · `"us"`（正确）⟹ 年龄 ≈ 0（网络与调度的抖动，毫秒级）
          · `"ms"` ⟹ 年龄 ≈ 1.7e9 s（时间戳只有 µs 的千分之一）
          · `"ns"` ⟹ 年龄 ≈ −1.7e9 s（巨大正数，只有在 `min()` 之后才被钳回"新鲜"）
          · `"zero"` 或"没填" ⟹ 0，走反序列化里 `if (timestamp == 0)` 那一支
          · `"stale"` ⟹ 是有意义的 µs，但**往过去挪了**（年龄 > 0.5 s）—— 05 验证过：
            挪 1 s 贴边、挪 2 s 进不了 OFFBOARD

        ★ 返回的是一个**猜测**，不是断言：它存在的意义是"失败时一眼看出是哪一类"，
          断言请用 `stamp_age_s()`（一个连续的、可设阈值的量）。
        """
        frames = self.published_frames(topic_key)
        if not frames:
            return ("none", float("nan"))
        ages = [self.stamp_age_s(f) for f in frames]
        med = statistics.median(ages)
        if all(f.timestamp_us == 0 for f in frames):
            return ("zero", med)
        if abs(med) < 0.5:
            return ("us", med)
        if med > 1e6:
            return ("ms", med)
        if med < -1e6:
            return ("ns", med)
        return ("stale", med)

    # ─────────────────── 人看的证据 ───────────────────

    def evidence(self) -> str:
        """出事时把人话证据打出来 —— 比"断言失败"多给的是**记录本身**。"""
        def _describe(key: str) -> str:
            frames = self.published_frames(key)
            if not frames:
                return f"- `{self.topics[key]}`：**一帧都没有**（匹配到的 DUT 发布者 " \
                       f"{self.matched_publishers(key)} 个）"
            unit, age = self.stamp_unit_guess(key)
            return (f"- `{self.topics[key]}`：{len(frames)} 帧，"
                    f"首帧 {frames[0].at_s:.2f} s、末帧 {frames[-1].at_s:.2f} s ⟹ "
                    f"{self.rate_hz(key):.1f} Hz；时间戳单位猜测={unit}（中位年龄 {age:.3g} s）")

        return "\n".join([
            "### 假 PX4 记录",
            "",
            f"- 状态：armed={self.armed} nav_state={self.nav_state} "
            f"pre_flight_checks_pass={self.pre_flight_checks_pass}"
            f"；静默中={sorted(self._muted) or '无'}",
            f"- `{self.topics['status']}`（夹具发）：{len(self.status_frames)} 帧，"
            f"{self.status_rate_hz():.2f} Hz；匹配到的 DUT 订阅者 "
            f"{self.matched_subscribers('status')} 个",
            f"- A 档状态（夹具发）：" + "；".join(
                f"`{self.topics[k]}` {self.state_rate_hz(k):.1f} Hz"
                f"（订阅者 {self.matched_subscribers(k)} 个）"
                for k in ("local_position", "attitude"))
            + f"；注入的位姿 NED={tuple(round(v, 3) for v in self.position_ned)}、"
              f"v_NED={tuple(round(v, 3) for v in self.velocity_ned)}",
            _describe("rates"),
            _describe("mode"),
        ])
