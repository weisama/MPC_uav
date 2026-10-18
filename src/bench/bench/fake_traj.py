"""假 `traj` 节点 —— 只假它的**对外行为**（三条话题），不假轨迹数学。

★★ 为什么需要它：`control` 的 `TRACK` 全靠 `/traj/*` 三条话题（ticket 07 定稿的接口），
  而 `traj` 是**另一个包**。真起 `traj` 会把两件事混在一起测：轨迹算得对不对（traj 的事）
  与"参考来了之后状态机怎么走"（control 的事）。这里只负责后者。
  ⚠️ 与 `FakeMavros` 同一条纪律：**假的是哪个节点**要说清楚 —— 假的是 `traj`，
  不是 `traj::trajectories` 里的那几条曲线（那是纯函数，由 `test_traj.cpp` 自己测）。

★★ 它的行为**逐条对着 `src/traj/src/traj_node.cpp` 抄**（第一手，2026-09-23 核对）。
  这四条不是细节，它们**各挡着一个具体的错误实现**：

  1. `tick()` 开头 `if (!running_) return;` ⟹ **`/traj/reference` 与 `/traj/progress`
     只在收到 `/traj/start` 之后才有帧**。少了这一条，"进 TRACK 第一帧就有参考"会让
     DUT 里"参考还没来"那条分支永远走不到（而它正是 `traj_timeout` 的入口）。
  2. `on_start` 在**没有选中轨迹**时只打一条 WARN 就忽略 ⟹ 加了 `selected=False` 这个开关。
     它对应现场最像"按键没反应"的一种场面：`/traj/start` 发了、DUT 那边一切正常、
     而参考一帧都不来（DUT 的 `diagnose()` 专门为这一条写了提示）。
  3. 运行中 `select()` 会 `running_ = false; elapsed_ = 0` 且**不发任何进度**（只有 `clear()`
     发 `progress 0.0`）⟹ **"运行中重选轨迹"在话题层只能表现为静默**。
     DUT 的重发 `/traj/start` 的闸门（`traj_stream_gap_s`）就是为它写的。
  4. 跑完之后 `running_` 仍为真、参考停在终点、**完成度保持 1.0** ⟹ 下一次任务在
     **同一个节点进程**里进 TRACK 时，第一帧就会看到 `progress == 1.0`
     （DUT 靠"必须先见过 < 1.0"的闩锁挡这件事，见 `FakeTraj.replay_done` /
     `fsm.hpp` 的 `track_started_`）。

  QoS 也是抄的：两个发布者都是 `rclcpp::QoS(10)`（可靠 + 易失），`/traj/start` 的订阅者
  同样是 `QoS(10)`。这一层错了不会报错，只会"永远收不到"（同 `FakeMavros` 的教训）。

  ★ **填了哪几个字段也是抄的**（2026-09-30 补，ticket 17）：真身 `publish_reference`
  （`traj_node.cpp:311-329`）填的是 `position` + `velocity` + `yaw`，`acceleration_or_force`
  与 `yaw_rate` 留 0（那几位在 `kTypeMask` 里被忽略）。本夹具原先把 `velocity` **漏了** ——
  对只看位置的消费者不可见，而 Phase 2 的 MPC 要拿它做时域外推 ⟹ 漏掉它 = 每条轨迹都被
  当成"停在那儿"，且不报错。`velocity_fn` 就是补的这三格。

★★ 它**不做二次成型**：`reference_fn` 说什么就发什么（没有限速、没有平滑）。
  真机上的限速在 `traj` 里（`max_speed_ms`，`traj_node.cpp:93/128`）—— 那是 `traj` 自己的
  正确性，由 `test_traj.cpp` 测；这里若也插一道限速，只会让"目标是不是等于原点 + 参考"
  这种**接口**判据变得难读。需要"参考是平滑的"那种场面时，自己给一个渐变函数。

★ 用法（它自己不会动，得有发布者发 `/traj/start` 来推它）：

    traj = FakeTraj(sc.node, sc, duration_s=2.0)
    traj.select()                 # 相当于操作员在 traj 那边选了一条轨迹
    ... DUT 进 TRACK 之后会发 /traj/start ⟹ 这里开始发参考
    traj.select()                 # 运行中重选：流断掉、时钟归零、什么都不发
    traj.silence()                # 流没了（模拟 traj 进程退出）
    traj.replay_done()            # 造"上一轮已经跑完、完成度停在 1.0"的场面
"""

import struct
from typing import Callable, List, Optional, Tuple

from mavros_msgs.msg import PositionTarget
from std_msgs.msg import Empty, Float32

__all__ = ["FakeTraj"]

#: 参考的 frame 标签（`traj_node.cpp` 的 `reference_frame_id` 默认值）。
#: ★ 纯标注：mavros 不读 `frame_id`（`setpoint_raw.cpp:209-249`），但**照抄**能让
#:   "DUT 有没有偷偷依赖这个标签"这件事在台架上显形。
REFERENCE_FRAME_ID = "traj_origin"

#: `traj_node.cpp` 的 `kTypeMask`：位置 + 速度 + yaw 有效（只忽略加速度与 yaw_rate）。
TYPE_MASK = (
    PositionTarget.IGNORE_AFX | PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ
    | PositionTarget.IGNORE_YAW_RATE
)


class FakeTraj:
    """假 `traj`：`/traj/reference` + `/traj/progress` + `/traj/start`。

    ⚠️ 只起**一个**实例（同一个域里两份夹具会让 `/traj/*` 上出现两股帧，同 bench README 的事故）。
    """

    def __init__(
        self,
        node,
        clock,
        reference_topic: str = "/traj/reference",
        progress_topic: str = "/traj/progress",
        start_topic: str = "/traj/start",
        rate_hz: float = 50.0,
        duration_s: float = 60.0,
        selected: bool = True,
        reference_fn: Optional[Callable[[float], Tuple[float, float, float]]] = None,
        velocity_fn: Optional[Callable[[float], Tuple[float, float, float]]] = None,
        publish_progress: bool = True,
    ) -> None:
        #: 场景时钟（`Scenario`）—— 频率类的东西一律用它，不用墙上时钟。
        self.clock = clock
        self._node = node
        #: 有没有选中轨迹（`False` ⟹ 收到 `/traj/start` 也**不发**参考，只记一笔）。
        self.selected = selected
        #: 这条轨迹要跑多久（s）。真实值：悬停 = `hover_duration_s`（60 s）、
        #: 方形/8 字 = 周长/速度 ⟹ **没有哪条轨迹的第一帧完成度是 1.0**。
        self.duration_s = duration_s
        #: 参考（**相对轨迹原点**的 ENU 位置）随已跑时间怎么变。
        #: 默认恒为 `(0, 0, 0)` = 原地悬停在轨迹原点上 —— 那正是 z0 的轨迹 1
        #: （`traj::Hover` 整条都发相对原点的零偏移），也是"目标 = 原点 + 参考"最裸的场面。
        self.reference_fn = reference_fn or (lambda _t: (0.0, 0.0, 0.0))
        #: ★ **参考自己的速度**（相对轨迹原点，ENU，m/s）—— 真身 `traj_node.cpp:321-324` 是
        #: 逐轴照抄 `ref.v` 的（`msg.velocity.x/y/z = ref.v.x/y/z`），而 `kTypeMask` **不忽略**
        #: 速度那三位 ⟹ 这三格是**有效字段**，不是"填了也没人看"。
        #: ⚠️ **2026-09-30 补（ticket 17）**：本夹具原先把这三格**漏掉了**（永远留 0），
        #:   而"0"与"没给"在**参考**这一侧是两个不同的意思（前者说"它就停在这儿"）。
        #:   对一个只看位置的消费者（Phase 1 的 `control`）漏掉它是不可见的；而 Phase 2 的
        #:   MPC 要把速度拿去做时域外推 ⟹ 漏掉它 = 每一条轨迹都被当成"停在那儿"，
        #:   且**完全不报错**。默认 `None` ⟹ 三个轴都发 0（= 补这一格之前的行为，旧用例不受影响）。
        self.velocity_fn = velocity_fn or (lambda _t: (0.0, 0.0, 0.0))
        self.publish_progress = publish_progress

        #: `running_`（`traj_node.cpp`）：只有它为真才发参考与进度。
        self.running = False
        #: 已跑时间（`elapsed_`）；`/traj/start` 与每次 select 都把它归零。
        self.elapsed_s = 0.0

        # —— 记录（断言用）——
        #: 收到的 `/traj/start` 的时刻（场景时钟）。★ 重发也在这里 —— 它是
        #: "DUT 有没有按闸门重发"唯一的可观测证据。
        self.starts: List[float] = []
        #: 参考的发布帧数（`running_` 为假时一帧都不发）。
        self.reference_frames = 0
        #: 发出去的完成度序列（最后一个是"当前完成度"）。
        self.progress_values: List[float] = []

        self._ref_pub = node.create_publisher(PositionTarget, reference_topic, 10)
        self._prog_pub = node.create_publisher(Float32, progress_topic, 10)
        self._start_sub = node.create_subscription(Empty, start_topic, self._on_start, 10)
        self._timer = node.create_timer(1.0 / rate_hz, self._tick)
        self._last_t = clock.t()

    # ─────────────────────────── 观测（断言用） ───────────────────────────

    def progress(self) -> float:
        """当前完成度（0–1）。★ 跑完之后**保持 1.0**（真实实现也一样，不会自己归零）。"""
        if self.duration_s <= 0.0:
            return 1.0
        return min(1.0, self.elapsed_s / self.duration_s)

    def matched_subscribers(self) -> int:
        """下游有几个订阅者匹配上了（QoS 不兼容时恒为 0，见 `FakeMavros` 的同名方法）。

        ★ 用例**必须**先等它 ≥ 1 再让 DUT 进 TRACK：`/traj/start` 是"我们发一次、`traj` 收"
          的方向，DDS 发现是异步的 —— 夹具若在 DUT 已经发过 start 之后才被发现，那一帧就丢了，
          而症状是"按键没反应"（现场最难查的一类），不是"夹具少接了一条话题"。
        """
        return self._ref_pub.get_subscription_count()

    # ─────────────────────────── 操作员的动作 ───────────────────────────

    def select(self, duration_s: Optional[float] = None, reference_fn=None) -> None:
        """选中轨迹（相当于 `traj` 的 `/traj/select`）。

        ★ 与**运行中**重选是同一个函数（真实实现里也是同一个 `select_kind()`）：
          `running_ = false`、`elapsed_ = 0`，而且**一帧都不发**。
          ⚠️ 所以"运行中重选"在话题层**只能表现为静默** —— 没有"我重选了"这条消息。
        """
        self.selected = True
        if duration_s is not None:
            self.duration_s = duration_s
        if reference_fn is not None:
            self.reference_fn = reference_fn
        self.running = False
        self.elapsed_s = 0.0

    def clear(self) -> None:
        """取消选中（`traj` 的 clear）：`running_ = false` + 发一帧 `progress 0.0`。
        ★ 这是**唯一**会在没跑的时候发进度的那条路径（`traj_node.cpp` 里只有 `clear()` 这么干）。"""
        self.selected = False
        self.running = False
        self.elapsed_s = 0.0
        self._publish_progress(0.0)

    def silence(self) -> None:
        """把流停掉（模拟 `traj` 进程没了 / 话题被掐）：**不归零、不发任何东西**。
        ★ 与 `Scenario` 的 `mute()` 是两种造法：`mute` 让话题上"有发布者、没有帧"，
          这个让 `traj` 自己不发 —— 对 DUT 而言两者同形（它只看帧到没到）。"""
        self.running = False

    def replay_done(self, duration_s: Optional[float] = None) -> None:
        """造"上一轮已经跑完"的场面：完成度**保持 1.0**（`traj` 跑完之后就是这样）。

        ★★ 它对应的真实缺陷：同一个 `control` 进程里飞第二个任务时，进 TRACK 的第一帧
          就会看到一个 `progress == 1.0` —— 没有闩锁的话当场判"任务飞完"并请求降落。
        """
        if duration_s is not None:
            self.duration_s = duration_s
        self.selected = True
        self.elapsed_s = self.duration_s
        self.running = True
        self._publish_reference()
        self._publish_progress(1.0)

    # ─────────────────────────── 内部 ───────────────────────────

    def _on_start(self, _msg) -> None:
        self.starts.append(self.clock.t())
        if not self.selected:
            # ★ 真实实现（`traj_node.cpp::on_start`）：没有选中轨迹时只打一条 WARN 就**忽略**。
            #   于是 DUT 那边"一切正常、一帧参考都没有" ⟹ 必须在台架上能造出来。
            return
        self.running = True
        self.elapsed_s = 0.0

    def _tick(self) -> None:
        now = self.clock.t()
        dt = now - self._last_t
        self._last_t = now
        # ★★ 这一行就是 `/traj/reference` 的全部开关（`traj_node.cpp::tick()` 的第一行）。
        if not self.running:
            return
        self.elapsed_s += dt
        self._publish_reference()
        self._publish_progress(self.progress())

    def _publish_reference(self) -> None:
        x, y, z = self.reference_fn(self.elapsed_s)
        vx, vy, vz = self.velocity_fn(self.elapsed_s)
        msg = PositionTarget()
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.header.frame_id = REFERENCE_FRAME_ID
        msg.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        msg.type_mask = TYPE_MASK
        msg.position.x = float(x)
        msg.position.y = float(y)
        msg.position.z = float(z)
        # ★ 速度照抄真身（`traj_node.cpp:321-324`）—— 掩码里速度那三位是**有效**的。
        msg.velocity.x = float(vx)
        msg.velocity.y = float(vy)
        msg.velocity.z = float(vz)
        self._ref_pub.publish(msg)
        self.reference_frames += 1

    def _publish_progress(self, value: float) -> None:
        if not self.publish_progress:
            return
        msg = Float32()
        msg.data = float(value)
        self._prog_pub.publish(msg)
        # ★★ 记的是**真的上了线**的那个数（`Float32` 量化之后的 double），不是 `value` 本身：
        #   `/traj/progress` 与 `/control/traj_progress` 都是 `Float32` ⟹ 话题本身就会把
        #   我们这边算出来的 double 抹成 float32。记原值的话，"转发出来的是否逐字相同"
        #   这条断言会永远不成立 —— 而那是**话题精度**的差，不是转发出了错（`test_control_track.py`
        #   的「转发完成度」那条第一次跑就红在这里）。
        quantized = struct.unpack("<f", struct.pack("<f", float(value)))[0]
        self.progress_values.append(quantized)
