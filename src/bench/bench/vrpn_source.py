"""脚本化的动捕输入 —— 替代 `vrpn_listener`（或者更下游的 `mocap` 节点）。

它的存在理由：动捕主机、VRPN、路由器**都不该出现在节点级测试里**。测试要的是
"在指定的秒数上，动捕说飞机在这里、并且说了 N 次"，而不是一台真的 Nokov。

★ 两种喂法（由 `target` 选）：
   · `target='vrpn'`  → 发 `/vrpn/<刚体>/pose`：**上游**，`mocap` 节点在回路里。
                        测 `mocap` 节点、或测"动捕→mocap→control"整链时用。
   · `target='mocap'` → 直接发 `/mocap/pose_enu`：**下游**，`mocap` 节点被旁路。
                        测 `control` 节点时用 —— 它的守卫只关心"动捕有没有数据"，
                        没必要为了造一帧数据把两个节点都拖进来。

★★ QoS：两条话题都必须是**可靠**（`rclcpp::QoS(10)`）。依据：
   · `vrpn_listener.cpp:142` `create_publisher<PoseStamped>(pose_topic_name, 10)`
   · `mocap_rel_pose_node.cpp:110` `create_publisher<PoseStamped>(pose_enu_topic_, QoS(10))`
   而下游的订阅方（`mocap` / `control`）也是 `QoS(10)` ⟹ 用 `SensorDataQoS()`（best_effort）
   发会让它们**永远收不到**，且没有任何报错。

★ 坐标系：本类发的是**动捕原生 ENU / 机体 FLU**，不做任何转换（与 `mocap` 节点的约定一致）。

★★ **位姿与速度是两条话题、两种消息类型**（ticket 15）：真身里 `vrpn_listener` 对每个 sender
   同时发 `/pose`（`PoseStamped`）与 `/twist`（`TwistStamped`），`mocap` 节点再把 twist
   **原样** republish 成 `/mocap/velocity_enu`。所以本夹具也照发两条 —— 与 `target` 一致：
   · `target='vrpn'`  → `/vrpn/<刚体>/pose` + `/vrpn/<刚体>/twist`
   · `target='mocap'` → `/mocap/pose_enu` + `/mocap/velocity_enu`
   ⚠️ **绝不能**把速度塞进 `PoseStamped` 里造出来（`PoseStamped` 根本没有 `twist` 字段）——
      ROS 2 里消息类型不匹配就是**不连接**，那会让夹具变成在说谎（ticket 15 的裁定）。
"""

from __future__ import annotations

import time
from typing import Callable, Optional, Sequence, Tuple

from geometry_msgs.msg import PoseStamped, TwistStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

Pose = Tuple[Sequence[float], Sequence[float]]


def mocap_qos() -> QoSProfile:
    """上游 `/vrpn/*` 与下游 `/mocap/*` 都是可靠 / volatile / depth 10。"""
    return QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)


class VrpnSource:
    """按 `rate_hz` 发布位姿；位姿可以固定、可以按函数变化、可以被冻住/倒退/暂停。"""

    def __init__(
        self,
        node: Node,
        target: str = "vrpn",
        body_name: str = "z0",
        topic: Optional[str] = None,
        rate_hz: float = 200.0,
        frame_id: Optional[str] = None,
        position: Sequence[float] = (0.0, 0.0, 0.0),
        orientation: Sequence[float] = (0.0, 0.0, 0.0, 1.0),
        velocity: Sequence[float] = (0.0, 0.0, 0.0),
    ) -> None:
        if target not in ("vrpn", "mocap"):
            raise ValueError("target 只能是 'vrpn'（发 /vrpn/<刚体>/pose）或 'mocap'（发 /mocap/pose_enu）")

        self.node = node
        self.target = target
        self.body_name = body_name
        if topic is None:
            topic = f"/vrpn/{body_name}/pose" if target == "vrpn" else "/mocap/pose_enu"
        self.topic = topic
        # 速度那条话题（ticket 15）：与 pose 同级、同一个上游。话题名同样照真身，
        # 不为了"测试方便"改名（真身是 `/vrpn/<刚体>/twist` 与 `/mocap/velocity_enu`）。
        self.velocity_topic = (
            f"/vrpn/{body_name}/twist" if target == "vrpn" else "/mocap/velocity_enu")
        if frame_id is None:
            # vrpn_listener 的默认 frame 是 "world"；mocap 节点原样转发它拿到的 frame_id。
            frame_id = "world"
        self.frame_id = frame_id

        self.position = tuple(float(v) for v in position)
        self.orientation = tuple(float(v) for v in orientation)
        self.velocity = tuple(float(v) for v in velocity)
        self.trajectory: Optional[Callable[[float], Pose]] = None

        # —— 可注入的输入故障（主线程改，executor 线程读）——
        self.paused = False          # 完全不发（"话题停止"）
        self.held_stamp = False      # 照发但时间戳不前进（"话题冻住"）
        self._held_stamp_value = None
        self._rewind_s = 0.0         # 时间戳相对当前时刻倒退多少
        self._rewind_until = 0.0

        self._pub = node.create_publisher(PoseStamped, self.topic, mocap_qos())
        self._vel_pub = node.create_publisher(TwistStamped, self.velocity_topic, mocap_qos())
        self._t0 = None
        self._timer = node.create_timer(1.0 / max(1e-3, rate_hz), self._publish)

    # ─────────────────── 发布 ───────────────────

    def _publish(self) -> None:
        if self.paused:
            return
        if self._t0 is None:
            self._t0 = time.monotonic()

        pos, quat = self.position, self.orientation
        if self.trajectory is not None:
            pos, quat = self.trajectory(time.monotonic() - self._t0)

        stamp = self._stamp()
        msg = PoseStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id
        msg.pose.position.x = float(pos[0])
        msg.pose.position.y = float(pos[1])
        msg.pose.position.z = float(pos[2])
        msg.pose.orientation.x = float(quat[0])
        msg.pose.orientation.y = float(quat[1])
        msg.pose.orientation.z = float(quat[2])
        msg.pose.orientation.w = float(quat[3])
        self._pub.publish(msg)

        # 速度是**独立的一条话题、另一种消息类型**（ticket 15）：真身 `vrpn_listener` 每拍
        # 也同时发 twist，`mocap` 节点原样转发。angular 留零 —— 照抄 `handleTwist` 的刻意行为。
        vel = TwistStamped()
        vel.header.stamp = stamp
        vel.header.frame_id = self.frame_id
        vel.twist.linear.x = float(self.velocity[0])
        vel.twist.linear.y = float(self.velocity[1])
        vel.twist.linear.z = float(self.velocity[2])
        self._vel_pub.publish(vel)

    def _stamp(self):
        now = self.node.get_clock().now()
        if self.held_stamp:
            if self._held_stamp_value is None:
                self._held_stamp_value = now.to_msg()
            return self._held_stamp_value
        self._held_stamp_value = None
        if time.monotonic() < self._rewind_until:
            return (now - Duration(seconds=self._rewind_s)).to_msg()
        return now.to_msg()

    # ─────────────────── 故障注入 ───────────────────

    def pause(self) -> None:
        """**停止发布**（"话题没有数据"）。★ 与 `hold_stamp` 是两种不同的失效：
        停止 = 一条都没有；冻住 = 有消息但时间戳不动。控制节点的判据只看"到没到"，
        而 mavros 的 vision_pose 会**丢弃时间戳不前进的消息** ⟹ 两者在 EKF2 眼里效果一样，
        在话题层面完全不一样，所以夹具要能分开造。"""
        self.paused = True

    def resume(self) -> None:
        self.paused = False

    def hold_stamp(self, held: bool = True) -> None:
        """时间戳不再前进（第一帧的值被一直重发）。"""
        self.held_stamp = held
        self._held_stamp_value = None

    def rewind(self, seconds: float, hold_s: float = 0.5) -> None:
        """让时间戳**倒退** `seconds` 秒，持续 `hold_s` 之后恢复（于是会看到一次
        向后的跳变、再一次向前的跳变）。"""
        self._rewind_s = float(seconds)
        self._rewind_until = time.monotonic() + hold_s

    def set_velocity(self, vx=None, vy=None, vz=None) -> None:
        """注入速度（ENU，m/s）—— 只改被点名的那几路（`None` = 不动）。

        ★ 与 `set_pose` 同一形状。它是 τ 那条路在 **B 档（动捕闭环）** 上唯一的黑盒注入面
          （ticket 15 之前根本不存在 —— `PoseStamped` 没有 `twist`）。
        """
        if vx is not None:
            self.velocity = (float(vx), self.velocity[1], self.velocity[2])
        if vy is not None:
            self.velocity = (self.velocity[0], float(vy), self.velocity[2])
        if vz is not None:
            self.velocity = (self.velocity[0], self.velocity[1], float(vz))

    def set_pose(self, position=None, orientation=None) -> None:
        if position is not None:
            self.position = tuple(float(v) for v in position)
        if orientation is not None:
            self.orientation = tuple(float(v) for v in orientation)
        self.trajectory = None

    def set_trajectory(self, fn: Optional[Callable[[float], Pose]]) -> None:
        """`fn(t_s) -> (position, orientation)`，`t_s` 从第一帧发布起算。"""
        self.trajectory = fn

    # ─────────────────── 发现屏障 ───────────────────

    def matched_subscribers(self) -> int:
        """下游有几个订阅者匹配上了（QoS 不兼容时恒为 0，见 `FakeMavros` 的同名方法）。"""
        return self._pub.get_subscription_count()

    def matched_velocity_subscribers(self) -> int:
        """**速度那条话题**上匹配到的订阅者数（ticket 15）—— 与位姿那条**分开**数。

        ★ 分开的理由是"QoS 不兼容就不报错、只是永不匹配"这条老坑：位姿订上了不等于速度也
          订上了，所以速度这条接缝要有自己的判据（`test_mpc_node` 的接口用例正是这么断的）。
        """
        return self._vel_pub.get_subscription_count()
