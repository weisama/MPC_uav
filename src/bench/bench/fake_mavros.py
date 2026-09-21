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
   | `/mavros/setpoint_raw/local` | **订阅** `SensorDataQoS()`（best_effort） | `setpoint_raw.cpp` |

   为什么必须一致：QoS 不兼容的两个端点**不报错**，只是永远不匹配 ⟹ 表现为"节点没数据"，
   而日志、`ros2 topic list`、`ros2 topic hz` 全都看不出原因。
   本文件用 `bench/__init__.py` 里那两个工厂函数把这件事写成代码（并被 `test_bench_selftest.py` 盯着）。

★ 运行方式：只被 `Scenario` 创建，不要单独跑（它没有 main）。
"""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass
from typing import Dict, List, Optional, Sequence

from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import PositionTarget, RCIn, State
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


# ───────────────────────────── 主类 ─────────────────────────────

class FakeMavros:
    """假的 mavros。**所有 rclpy 调用都发生在 executor 线程里**（定时器/订阅/服务回调），
    主线程只改普通的 Python 属性 + 读记录（`list.append` 在 GIL 下是原子的）⟹
    不需要在测试里到处加锁，也不会出现"两个线程同时碰 rclpy"。

    要注入的故障全部是"改一个属性"：静默（`mute`）、改状态（`set_state`）、
    改电压（`set_voltage`）、改遥控（`set_rc`）、服务应答策略（`set_mode_policy`）。
    """

    SILENCEABLE = ("state", "local_position", "local_velocity", "rc", "battery")

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
        voltage: float = 16.0,
        rc_channels: Optional[Sequence[int]] = None,
        create_set_mode: bool = True,
        create_arming: bool = True,
        follow_set_mode: bool = True,
        mode_transition_delay_s: float = 0.05,
    ) -> None:
        self.node = node
        # 时刻的尺子由 Scenario 提供（`clock.t()`），这样"记录里的一秒"与"时序里的一秒"是同一秒。
        self.clock = clock

        rates = {"state": 1.0, "local_position": 30.0, "local_velocity": 30.0,
                 "rc": 10.0, "battery": 1.0}
        rates.update(rates_hz or {})
        t = {
            "state": "/mavros/state",
            "local_position": "/mavros/local_position/pose",
            "local_velocity": "/mavros/local_position/velocity_local",
            "rc": "/mavros/rc/in",
            "battery": "/mavros/battery",
            "setpoint": "/mavros/setpoint_raw/local",
            "set_mode": "/mavros/set_mode",
            "arming": "/mavros/cmd/arming",
        }
        t.update(topics or {})
        self.topics = t

        # —— 可注入的状态（主线程改，executor 线程读）——
        # ⚠️ 全是"读一次就够"的标量/元组替换，不做就地修改 ⟹ 不用锁。
        self._muted: set = set()
        self.connected = connected
        self.armed = armed
        self.mode = mode
        self.guided = False
        self.manual_input = False
        self.position = tuple(float(v) for v in position)
        self.voltage = float(voltage)
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
        self.mode_transition_delay_s = mode_transition_delay_s

        # —— 记录 ——
        self.setpoints: List[Setpoint] = []
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
        # 供"发现屏障"按 key 取用（见 matched_subscribers）
        self._pubs = {
            "state": self._state_pub,
            "local_position": self._pose_pub,
            "local_velocity": self._vel_pub,
            "rc": self._rc_pub,
            "battery": self._battery_pub,
        }

        # —— 下游（mavros 是订阅方，QoS 用 SensorDataQoS ⟹ 发布可靠一方也能匹配）——
        self._setpoint_sub = node.create_subscription(
            PositionTarget, t["setpoint"], self._on_setpoint, mavros_sensor_qos())

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
                            ("battery", self._pub_battery)):
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
        if key in self._muted:
            return
        cb()

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

    def _pub_pose(self) -> None:
        x, y, z = self.position
        msg = PoseStamped()
        msg.header.stamp = self._stamp()
        msg.header.frame_id = "map"   # ← 真 mavros 的 local_position 用 "map"（不是 "world"）
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.position.z = float(z)
        msg.pose.orientation.w = 1.0
        self._pose_pub.publish(msg)

    def _pub_velocity(self) -> None:
        msg = TwistStamped()
        msg.header.stamp = self._stamp()
        msg.header.frame_id = "map"
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

    def set_mode_clients(self) -> int:
        """DUT 侧有几个 `/mavros/set_mode` 客户端匹配上了（服务没建起来时返回 0）。"""
        return self._set_mode_srv.get_num_clients() if self._set_mode_srv else 0

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
        if response.success and self.follow_set_mode:
            # 模拟 PX4 真的解锁了：下一次 state 发布时 armed 变真
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
            f"- 匹配到的 DUT 订阅者："
            + ", ".join(f"{k}={self.matched_subscribers(k)}" for k in self._pubs)
            + f", set_mode 客户端={self.set_mode_clients()}",
            f"- 当前状态：connected={self.connected} armed={self.armed} mode={self.mode}"
            f"；静默中={sorted(self._muted) or '无'}；position={self.position}",
        ]
        return "\n".join(lines)
