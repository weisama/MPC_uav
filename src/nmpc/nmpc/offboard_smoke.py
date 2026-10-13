#!/usr/bin/env python3
"""ticket 05 的**最小 offboard 冒烟节点** —— 不含任何模型 / 代价 / 约束。

它只做三件事：① 按 50 Hz 发角速率/推力 setpoint；② 把 PX4 拉进 OFFBOARD（先发够 `--pre-stream`，再解锁、再切模式）；
③ 发的指令来自一条**参数化时间轴**（[`commands.py`](commands.py)），不来自求解器。

**为什么要有这个节点**：`[05]` 要排除的几个静默失败（消息类型不匹配 / QoS 不兼容 /
推力符号与归一化错）**都不报错、不崩溃**，只表现为"没反应"或"飞机朝错的方向加速"。
若把真 MPC（模型预测控制）和它们一起第一次接上真 PX4，出问题时分不清是哪一层。
所以本节点刻意**没有模型** —— 它错了，那就只可能是中间件或符号错了。

★★ **2026-10-12 起本节点走 mavros**（`/mavros/setpoint_raw/attitude` + `/mavros/cmd/arming` +
`/mavros/set_mode`），不再走 px4_msgs/uXRCE-DDS。起因、等价性与三处**不可等价**（N1 标量 `thrust` /
N2 mavros 替我们换 FLU→FRD / N3 闸门源只有 1 Hz）见
[`phase4/spec.md`](../../../.scratch/phase4/spec.md) §★★ 与
[`13`](../../../.scratch/phase4/issues/13-mavros-downlink-switch.md) §三。

**符号映射仍不在这里实现**，在 [`sign_map.py`](sign_map.py)（[`docs/control/sign-map.md`](../../../docs/control/sign-map.md)
§8 要求 05 的 stub 与 06 的节点共用同一份）—— 但**换了入口**：`sign_map.mavros_attitude_target()`。
★ 因为这条路交出去的角速率是 **FLU 原样**、推力是**正号**（取负由 mavros 与飞控做，
该文件 **§2.2 / §2.3** 那张"谁取负"的表）。

**与改通路之前（px4_msgs 版）的区别**（[`13`](../../../.scratch/phase4/issues/13-mavros-downlink-switch.md) §三）：

1. 心跳 `OffboardControlMode` 那条发布**删掉** —— 飞控从收到的 `SET_ATTITUDE_TARGET` 自己派生
   （`mavlink_receiver.cpp:1531-1535`），我们多发一条反而是**两条通路各说一次**。
2. `--timestamp-unit` / `--timestamp-shift-us` **删掉** —— 飞控那一段 handler **一次都没读**
   `time_boot_ms`（理由与证据见 [`sign-map.md`](../../../docs/control/sign-map.md) §2.3.1）。
   ⛔ 留着它会让操作者以为"时间戳这一档也测过了"，而这一场里那个开关**再也取不到第二个值**。
3. `--t-max` 变成**必须显式给**（2026-10-12 操作者裁定）—— 旧默认 `11.8` 是 `f_max`（按电池的
   `F` 上限），拿它当**归一化常数**会让悬停那一档多发 ≈3.92 倍，见 [`params.py`](params.py)
   的 `check_hover_consistency()`。启动时那条自检就是为了让这个错值**当场变红**。
4. `--break pitch-sign` / `yaw-sign` **改名**为 `--break pitch-preconvert` / `yaw-preconvert`：
   同一个开关在新载体上验的是 **N2**（我们多做了一次预转换 ⟹ 两次取负 = 不取负），**不是**"符号写反"。

用法（args 见 `--help`）：

    ros2 run nmpc offboard_smoke.py --mass 0.75 --t-max 46.3027 \
        --phases lift:1.2:1.2:0,0,0 --phases hover:8:1.0:0,0,0
    python3 src/nmpc/nmpc/offboard_smoke.py --help      # 不经 colcon 也能跑（自己补 sys.path）

⚠️ [`docs/control/sitl-offboard-smoke.md`](../../../docs/control/sitl-offboard-smoke.md) 描述的是
**换通路之前**的这个工具（SITL + uXRCE-DDS 的六项判据与 12 场表）⟹ 那里的跑法与那两组时间戳场景
**在新载体上没有对应物**，别照抄。
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from dataclasses import dataclass, replace
from pathlib import Path

if __package__ in (None, ""):  # 直接 `python3 …/offboard_smoke.py` 时把包目录挂上
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import rclpy  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,  # noqa: E402
                       ReliabilityPolicy)

from mavros_msgs.msg import AttitudeTarget, State  # noqa: E402
from mavros_msgs.srv import CommandBool, SetMode  # noqa: E402

from nmpc import commands, params, sign_map  # noqa: E402

#: `AttitudeTarget.type_mask` 里我们置的那一位 —— **用消息自己定义的常量**（`= 128`），不写魔数：
#: `mavros_msgs/msg/AttitudeTarget.msg` 里那五个位是 `IGNORE_ROLL_RATE=1` / `IGNORE_PITCH_RATE=2` /
#: `IGNORE_YAW_RATE=4` / `IGNORE_THRUST=64` / `IGNORE_ATTITUDE=128`（第一手：`/opt/ros/humble/share/
#: mavros_msgs/msg/AttitudeTarget.msg`）。
#: ⟹ `orientation` 被忽略、`body_rate` 三轴都被用、**`thrust` 不被忽略**（mavros 原样透传这个字段）。
#: 飞控侧读的正是同样这三位（`mavlink_receiver.cpp:1522-1526`，推力那一段还要求
#: `THROTTLE_IGNORE` 未置位，见 `:1588`）。
TYPE_MASK_IGNORE_ATTITUDE = AttitudeTarget.IGNORE_ATTITUDE

#: `/mavros/state.mode` 在 OFFBOARD 时的取值 —— ★ 它是**模式名字符串**，不是数字。
#: 取 `mavros_msgs/msg/State.msg` 里的常量 `MODE_PX4_OFFBOARD = OFFBOARD`（第一手：该文件），
#: 它同时也是 mavros 填这个字段的那条路径 `sys_status.cpp:926`（`uas->str_mode_v10(...)`）的取值，
#: 并且与本仓库**已经在跑**的 Phase 1 链是同一个串
#: （`src/control/include/control/px4_interface.hpp:117-119` 的 `ModeNames::offboard`）。
OFFBOARD_MODE = State.MODE_PX4_OFFBOARD

#: `/mavros/state` 的**上报周期**：mavros 每收到一条 HEARTBEAT 才发一帧
#: （`sys_status.cpp:966-972`）⟹ 频率 = 心跳频率（本机实测 **1.000 Hz**）。
#: ★ 它不进任何判据，只用来提醒一件事：**"等闸门打开"要按秒算**。
#:   轮询得比它快不会让确认更早发生，只会多打日志 —— 这里最容易出的错是
#:   "断言闸门已开"的那个轮询**短于**这个周期，从而在飞控还没上报之前就判绿（或恒红）。
STATE_PERIOD_S = 1.0

#: 发布侧：mavros 在 `setpoint_raw` 上订阅用的是 `SensorDataQoS()`
#: （best_effort / keep_last 5 / volatile，mavros `src/plugins/setpoint_raw.cpp:62`）
#: ⟹ 我们对齐到**同一档**。★ RELIABLE 的 writer 与 BEST_EFFORT 的 reader 在 DDS 里
#: 是**兼容**的，"能通"证明不了"对"—— 所以这里显式选、不留"能通就不管"。
QOS_PUB = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=5,
                     reliability=ReliabilityPolicy.BEST_EFFORT,
                     durability=DurabilityPolicy.VOLATILE)

#: 订阅侧：`/mavros/state` 的发布者是 `StateQoS()` = reliable + **transient_local**
#: （mavros `sys_status.cpp:579-583`）。我们**不**请求 `transient_local`：
#: 闸门要的是"现在是什么状态"，不是"最后一帧曾经是什么"（代价是最多晚 1 s，见 `STATE_PERIOD_S`）。
#: ⚠️ 两个取值**都收得到**（发布者是 reliable ⟹ 我们请求 best_effort 也兼容）⟹
#: 这个开关在 mavros 路径上**不再**像 px4_msgs 那条流一样能造出"零回调"。
QOS_STATE_RELIABLE = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                                reliability=ReliabilityPolicy.RELIABLE,
                                durability=DurabilityPolicy.VOLATILE)
QOS_STATE_BEST_EFFORT = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                                   reliability=ReliabilityPolicy.BEST_EFFORT,
                                   durability=DurabilityPolicy.VOLATILE)

BREAKS = ("none", "thrust-sign", "thrust-scale", "roll-sign",
          "pitch-preconvert", "yaw-preconvert")


@dataclass
class Config:
    #: ★ **没有默认值**（2026-10-12 操作者裁定）：它是**归一化常数** `T_max`，必须显式给。
    #: 旧默认 `11.8` 是 `f_max`（按电池的 `F` 上限）—— 另一个物理量，混用是**静默**的
    #: （悬停那一档多发 ≈3.92 倍）。见 `params.check_hover_consistency()`。
    t_max_n: float
    rate_hz: float = 50.0
    mass_kg: float = 0.75          # SITL z0 同构机的 SIH_MASS；实机是 750 g
    phases: tuple = ("lift:1.2:1.2:0,0,0", "hover:8:1.0:0,0,0")
    pre_stream_s: float = 1.5      # 切 OFFBOARD 前先发多久 setpoint
    arm: bool = True
    mode_switch: bool = True
    qos: str = "reliable"
    break_: str = "none"
    break_scale: float = 2.0
    auto_exit_after_s: float = 0.0
    log_file: str = ""
    log_every: int = 50
    #: 解锁 / 切模式命令的最小间隔。判据是"发到状态确认为止"，但重发快于闸门源的上报周期
    #: **什么也换不来**：`/mavros/state` 是 1 Hz ⟹ 取 1.1 s 让每两次尝试之间至少有一帧新状态
    #: （旧值 0.5 s 是给 px4_msgs 那条 2 Hz 以上的流配的）。
    #: ★ 默认值**从上一条导出**、不写死：请求间隔慢于上报周期一个余量，闸门那一次读才可能读到新值。
    #: （复核发现：写死 `1.1` 时 `STATE_PERIOD_S` 只被散文引用 —— 一个不参与计算的常量
    #: 迟早会被改成一个不成立的值也没人发现。）
    cmd_interval_s: float = STATE_PERIOD_S + 0.1


class OffboardSmoke(Node):
    """把 PX4 拉进 OFFBOARD 并照抄一条时间轴发指令的最小节点（mavros 载体）。"""

    def __init__(self, cfg: Config):
        super().__init__("offboard_smoke")
        self.cfg = cfg
        self.program = commands.parse_program(cfg.phases)
        self.hover_n = commands.hover_force_n(cfg.mass_kg)
        #: 这一对 `(m, T_max)` 隐含的悬停油门 —— 启动时打出来，让"归一化常数错了"在屏幕上可见。
        #: ★ 硬判据在 `validate_args()`（**建节点之前**就拒），这里只是把它算出来给人看。
        self.implied_hover = params.hover_thrust_implied(cfg.mass_kg, cfg.t_max_n)

        self.pub_att = self.create_publisher(
            AttitudeTarget, "/mavros/setpoint_raw/attitude", QOS_PUB)
        # ★ 解锁 / 切模式在这个载体上是**服务**（mavros `command.cpp:111-113` / `sys_status.cpp:617-621`），
        #   不是像 px4_msgs 那条路一样往 `/fmu/in/vehicle_command` 上发消息。
        self.cli_arming = self.create_client(CommandBool, "/mavros/cmd/arming")
        self.cli_mode = self.create_client(SetMode, "/mavros/set_mode")

        sub_qos = QOS_STATE_RELIABLE if cfg.qos == "reliable" else QOS_STATE_BEST_EFFORT
        self.sub_state = self.create_subscription(
            State, "/mavros/state", self._on_state, sub_qos)

        self.state = None
        self.state_count = 0
        self.arm_calls = 0
        self.arm_refused = 0
        self.mode_calls = 0
        self.mode_refused = 0
        self.arm_pending = False
        self.mode_pending = False
        self._svc_warned = set()
        self.published = 0
        self.first_publish_ns = None
        self.last_publish_ns = None
        self.offboard_since_ns = None
        self.last_cmd_ns = 0
        self.ticks = 0
        self._log = open(cfg.log_file, "w", encoding="utf-8") if cfg.log_file else None

        self.get_logger().info(
            f"指令时间轴（全重 {cfg.mass_kg:g} kg ⟹ 悬停力 m·g = {self.hover_n:.4f} N；"
            f"归一化常数 T_max = {cfg.t_max_n:.4f} N ⟹ 悬停归一化推力 {self.implied_hover:.4f}"
            f"，实测 hover_thrust = {params.HOVER_THRUST:g}）:\n"
            + commands.describe(self.program, self.hover_n)
            + "\n归一化后**发出去**的那个数（`AttitudeTarget.thrust`，正号；与上一行的『×悬停』是两回事）:\n"
            + self._describe_emitted())
        self.get_logger().info(
            f"取证开关：订阅 QoS={cfg.qos} 故障注入={cfg.break_}"
            + (f"（×{cfg.break_scale}）" if cfg.break_ == "thrust-scale" else ""))

        self.create_timer(1.0 / cfg.rate_hz, self._tick)

    # ---------------------------------------------------------------- 输入侧

    def _on_state(self, msg: State) -> None:
        self.state = msg
        self.state_count += 1

    # ---------------------------------------------------------------- 输出侧

    def _describe_emitted(self) -> str:
        """每段"归一化之后真正发出去多少" —— 走**真入口**算，不另写一遍公式。

        ★ 为什么这一列值得单独打（[`13`](../../../.scratch/phase4/issues/13-mavros-downlink-switch.md) §三② 第 3 条）：
        `commands.describe()` 打的是 `F = …N (…×悬停)`，那是**乘的基准**；
        归一化常数 `T_max` 错了，上一行看不出任何异常（它只印 `F`），而**这一行会**。
        改通路之前这里打的是 `thrust_body_z`（**负**号），现在是 `thrust`（**正**号）——
        ⛔ 两边的数**不许并列比较**（同一个物理量、两条路上反号的两个字段，见 §三④）。
        """
        out = []
        for phase in self.program:
            f_n = phase.thrust_scale * self.hover_n
            sp = sign_map.mavros_attitude_target(phase.omega_flu, f_n, self.cfg.t_max_n)
            out.append(f"    {phase.name:<10} F={f_n:6.3f}N ⟹ thrust=+{sp.thrust:.4f}")
        return "\n".join(out)

    def _map(self, omega, thrust_n):
        """内部量 → mavros 的四个字段。**唯一实现**在 `sign_map`，这里只做取证用的刻意破坏。"""
        t_max = self.cfg.t_max_n
        if self.cfg.break_ == "thrust-scale":
            t_max = self.cfg.t_max_n * self.cfg.break_scale
        sp = sign_map.mavros_attitude_target(omega, thrust_n, t_max)
        if self.cfg.break_ == "thrust-sign":
            # 把**我们交出去的正号**写反 ⟹ 飞控再加一个负号 ⟹ 推力朝下（"全速砸地"那一档）。
            sp = replace(sp, thrust=-sp.thrust)
        elif self.cfg.break_ == "roll-sign":
            # roll 在 mavros 那一步**不被取负**（它不换 x 分量）⟹ 这一档仍然是"符号写反"。
            sp = replace(sp, body_rate_x=-sp.body_rate_x)
        elif self.cfg.break_ == "pitch-preconvert":
            # ★ 这一档模拟的**不是**"符号写反"，是 **N2**：我们自己先转了一次 FLU→FRD，
            #   而 mavros 还会再转一次 ⟹ 两次取负 = 不取负（roll 不受影响、pitch/yaw **静默反向**）。
            sp = replace(sp, body_rate_y=-sp.body_rate_y)
        elif self.cfg.break_ == "yaw-preconvert":
            sp = replace(sp, body_rate_z=-sp.body_rate_z)
        return sp

    # ---------------------------------------------------- 解锁 / 切模式（服务侧）

    def _warn_service_once(self, name: str, what: str) -> None:
        """服务不可用只喊一次 —— 50 Hz 地喊会把"它压根没起来"这件事淹掉。"""
        if name not in self._svc_warned:
            self._svc_warned.add(name)
            self.get_logger().warn(f"{what} 服务不可用（{name}）—— 没有 mavros 或它还没起完？")

    def _request_arm(self) -> None:
        """`/mavros/cmd/arming`（`mavros_msgs/srv/CommandBool`）—— ★ **服务**，不是话题。

        第一手：mavros `src/plugins/command.cpp:111-113` 把它建在 `~/arming` 上；
        本仓库**已经在跑**的同一形态见 `src/control/src/px4_interface.cpp:177`。
        """
        if not self.cli_arming.service_is_ready():
            self._warn_service_once("/mavros/cmd/arming", "mavros 的 cmd/arming")
            return
        req = CommandBool.Request()
        req.value = True
        self.arm_pending = True
        self.arm_calls += 1
        self.cli_arming.call_async(req).add_done_callback(self._on_arm_response)

    def _on_arm_response(self, future) -> None:
        self.arm_pending = False
        try:
            res = future.result()
        except Exception as exc:  # 调用本身失败（服务掉了 / 超时）
            self.get_logger().warn(f"arming 服务调用失败：{exc!r}")
            return
        if not res.success:
            # ★ **被拒不是"没发生"**：应答里有 result 码，落在日志里才有复盘价值
            #   （旧载体往 `/fmu/in/vehicle_command` 上发消息，**根本没有应答可读**）。
            self.arm_refused += 1
            self.get_logger().warn(
                f"飞控**拒绝**解锁（第 {self.arm_refused} 次）：success=False result={int(res.result)}"
                f" —— 预检没过？还是 μORB 里有别的来源也在解锁？")

    def _request_mode(self, custom_mode: str) -> None:
        """`/mavros/set_mode`（`mavros_msgs/srv/SetMode`）。

        ★ `base_mode` 留 `0`、只给 `custom_mode` **字符串**：mavros 自己会补
        `CUSTOM_MODE_ENABLED` —— 这正是本仓库 Phase 1 链的用法
        （`src/control/src/px4_interface.cpp:546-547`，那里的注释逐字写着这一条）；
        消息自己的注释也是这么规定的（`mavros_msgs/srv/SetMode.srv`：`base_mode`
        "filled by MAV_MODE enum value or **0 if custom_mode != ''**"）。
        ⚠️ 应答 `mode_sent=True` **只表示 SET_MODE 报文发出去了** —— 真正的判据是
        稍后 `/mavros/state.mode` 变成期望值（最多晚一个 `STATE_PERIOD_S`）。
        """
        if not self.cli_mode.service_is_ready():
            self._warn_service_once("/mavros/set_mode", "mavros 的 set_mode")
            return
        req = SetMode.Request()
        req.base_mode = 0
        req.custom_mode = custom_mode
        self.mode_pending = True
        self.mode_calls += 1
        self.cli_mode.call_async(req).add_done_callback(self._on_mode_response)

    def _on_mode_response(self, future) -> None:
        self.mode_pending = False
        try:
            res = future.result()
        except Exception as exc:
            self.get_logger().warn(f"set_mode 服务调用失败：{exc!r}")
            return
        if not res.mode_sent:
            self.mode_refused += 1
            self.get_logger().warn(
                f"mavros **没把这个模式发出去**（第 {self.mode_refused} 次）："
                f"custom_mode={OFFBOARD_MODE!r} 解析不了？")

    # ---------------------------------------------------------------- 主循环

    def _tick(self) -> None:
        self.ticks += 1
        now_ns = time.time_ns()

        # ★ 闸门源是 `/mavros/state`（1 Hz）：`armed` 与 `mode` 都比 px4_msgs 那条流**晚**，
        #   最坏晚一个上报周期（`STATE_PERIOD_S`）。这里只用"最近一帧"，不猜、不外推。
        st = self.state
        armed = bool(st.armed) if st is not None else False
        mode = st.mode if st is not None else ""
        in_offboard = mode == OFFBOARD_MODE
        sys_status = int(st.system_status) if st is not None else -1

        # 程序时钟：**切进 OFFBOARD 的那一刻归零** —— 否则模式切换花掉的秒数会吃掉第一段
        # （★ 归零的那一帧可能比飞控真正进 OFFBOARD 晚最多一个 `STATE_PERIOD_S` ⟹ 时间轴偏短，
        #  不会偏长；要精确对齐得读飞控自己的日志，那不是本工具的事）。
        if self.offboard_since_ns is None and in_offboard:
            self.offboard_since_ns = now_ns
        t_program = 0.0 if self.offboard_since_ns is None else (now_ns - self.offboard_since_ns) / 1e9

        thrust_n, omega = commands.command_at(self.program, t_program, self.hover_n)
        sp = self._map(omega, thrust_n)

        att = AttitudeTarget()
        # `header.stamp`：mavros 会把它换算成 MAVLink 的 `time_boot_ms`（毫秒）——
        # ★ 而飞控**不读**那一位（`sign-map.md` §2.3.1）⟹ 这里填 `now()` 只为让消息自洽，
        #   **它不参与任何判据**（旧版那组"发错时间戳单位"的取证开关因此在 mavros 路径上没有对应物）。
        att.header.stamp = self.get_clock().now().to_msg()
        att.type_mask = TYPE_MASK_IGNORE_ATTITUDE
        # `orientation` 保持默认（全零）：`type_mask` 里 `IGNORE_ATTITUDE` 置位 ⟹
        # 飞控那一侧 `attitude = false`（`mavlink_receiver.cpp:1522`），这个字段既不填也不发。
        att.body_rate.x = float(sp.body_rate_x)
        att.body_rate.y = float(sp.body_rate_y)
        att.body_rate.z = float(sp.body_rate_z)
        att.thrust = float(sp.thrust)
        self.pub_att.publish(att)

        self.published += 1
        self.first_publish_ns = self.first_publish_ns or now_ns
        self.last_publish_ns = now_ns

        if self._log is not None:
            # ⚠️ 每个字段都要 `float()`：`att.body_rate.x` / `att.thrust` 是 ROS 的 **float32**，
            #    `json.dumps` 不认（`TypeError`），而定时器回调里抛异常 = 整个 spin 挂掉。
            self._log.write(json.dumps({
                "wall_ns": now_ns, "t_program_s": round(float(t_program), 4),
                # ★ `type_mask` 也记下来：它是"我们以为自己在说什么"的那一半 ——
                #   128 之外的任何值都会让飞控**换一种理解**（比如误置 THROTTLE_IGNORE ⟹ 推力被丢）。
                "type_mask": int(TYPE_MASK_IGNORE_ATTITUDE),
                "body_rate_x": float(att.body_rate.x), "body_rate_y": float(att.body_rate.y),
                "body_rate_z": float(att.body_rate.z),
                "thrust": float(att.thrust), "thrust_n": float(thrust_n),
                "hover_multiple": round(float(thrust_n / self.hover_n), 4),
                "mode": mode, "armed": int(armed), "system_status": sys_status,
                "state_cb": self.state_count,
            }) + "\n")
            self._log.flush()

        # ---- 拉进 OFFBOARD：setpoint 先发够 `pre_stream_s` ⟹ 解锁 ⟹ 切模式（每一步都等状态确认，不靠 sleep）
        # ★ 闸门开在**已发布时长**上，不是"起来之后等一会儿"：判据要的是"切 OFFBOARD 之前
        #   setpoint 已以 ≥2 Hz 发了 ≥1 s"，量的是**发出去的东西**，不是进程年龄。
        #   ⚠️ 这个 `pre_stream_s` 曾经只是个没接线的旋钮（`Config` 里有、`--pre-stream` 能改、
        #   但没人读）—— 于是实测 baseline 在首帧后 0.78 s 就进了 OFFBOARD，判据其实没被满足。
        streamed_s = (now_ns - self.first_publish_ns) / 1e9 if self.first_publish_ns else 0.0
        cmd_allowed = (now_ns - self.last_cmd_ns) / 1e9 >= self.cfg.cmd_interval_s
        armed_enough = streamed_s >= self.cfg.pre_stream_s
        # ⚠️ 旧版这里还有一道 `status.pre_flight_checks_pass` 守卫 —— **它在新载体上没有对应物**：
        #   `/mavros/state` 那个消息里没有这个字段（只有 connected / armed / mode / system_status）。
        #   ⟹ 预检没过时我们会**反复请求**解锁，而飞控**拒绝**（应答里的 result 码落在日志里）。
        #   ⛔ 别把"没有守卫"读成"解锁一定会成功"：`--no-arm` 仍然是"让别人来解锁"的那条路。
        if self.cfg.arm and armed_enough and not armed and not self.arm_pending and cmd_allowed:
            self._request_arm()
            self.last_cmd_ns = now_ns
        elif self.cfg.mode_switch and armed_enough and armed and not in_offboard \
                and not self.mode_pending and cmd_allowed:
            self._request_mode(OFFBOARD_MODE)
            self.last_cmd_ns = now_ns

        if self.ticks % self.cfg.log_every == 0:
            self.get_logger().info(
                f"t={t_program:5.1f}s 已发 {self.published:5d} 帧 | mode={mode!r} armed={int(armed)} "
                f"| F={thrust_n:6.3f}N ({thrust_n / self.hover_n:.2f}×悬停) "
                f"⟹ body_rate=({att.body_rate.x:+.3f},{att.body_rate.y:+.3f},{att.body_rate.z:+.3f}) "
                f"thrust=+{att.thrust:.4f} | state回调={self.state_count}")

        program_end = sum(p.duration_s for p in self.program)
        if self.cfg.auto_exit_after_s > 0 and t_program > program_end + self.cfg.auto_exit_after_s:
            self.get_logger().info(f"程序走完 + {self.cfg.auto_exit_after_s}s ⟹ 自行退出")
            raise SystemExit(0)

    # ---------------------------------------------------------------- 收尾

    def summary(self) -> str:
        span = (self.last_publish_ns - self.first_publish_ns) / 1e9 if self.first_publish_ns else 0.0
        rate = (self.published - 1) / span if span > 0 else float("nan")
        return (f"共发 {self.published} 帧，跨 {span:.2f}s ⟹ 实际 {rate:.1f} Hz；"
                f"解锁服务调用 {self.arm_calls} 次（被拒 {self.arm_refused}）、"
                f"切模式调用 {self.mode_calls} 次（未发出 {self.mode_refused}）；"
                f"进 OFFBOARD {'是' if self.offboard_since_ns else '**否**'}；"
                f"收到 /mavros/state {self.state_count} 条（订阅 QoS={self.cfg.qos}）")

    def close(self) -> None:
        if self._log is not None:
            self._log.close()
            self._log = None


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description="ticket 05 的最小 offboard 冒烟节点（无模型；2026-10-12 起走 mavros）",
                               formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--phases", action="append", metavar="名字:时长:悬停倍数:ωx,ωy,ωz",
                   help="指令时间轴，可重复；最后一段一直保持（默认 lift 1.2s×1.2 ⟹ hover 8s）")
    p.add_argument("--rate-hz", type=float, default=Config.rate_hz)
    p.add_argument("--mass", type=float, default=Config.mass_kg, help="kg；悬停推力 = m·g")
    p.add_argument("--t-max", type=float, required=True,
                   help="★ **必填**：**归一化常数** T_max [N]（不是 f_max！）。"
                        "今天的正确值是 46.3027 N = m·g / hover_thrust（m = 0.75 kg、"
                        "hover_thrust = 0.1589）。⛔ 11.8 N 是 `f_max`（按电池的 F 上限，"
                        "另一个物理量），拿它当归一化常数会让悬停那一档多发 ≈3.92 倍**且不报错**。"
                        "★ 它随 hover_thrust 变，而 hover_thrust 正是未结票 02 要回填的量 ⟹ "
                        "这里**不留默认值**，缺了就拒绝启动（操作者裁定，2026-10-12）")
    p.add_argument("--pre-stream", type=float, default=Config.pre_stream_s,
                   help="切 OFFBOARD 前先发这么久 setpoint（秒）")
    p.add_argument("--no-arm", action="store_true", help="不请求解锁（只发 setpoint）")
    p.add_argument("--no-mode-switch", action="store_true", help="不请求切模式")
    p.add_argument("--auto-exit-after", type=float, default=0.0,
                   help="程序走完后再过这么多秒自行退出（0 = 一直发到被杀）")
    p.add_argument("--log-file", default="", help="把每一帧 setpoint 写成 JSONL（取证用）")
    p.add_argument("--log-every", type=int, default=Config.log_every, help="每 N 帧打一行日志")
    p.add_argument("--cmd-interval", type=float, default=Config.cmd_interval_s,
                   help="解锁/切模式请求的最小间隔（秒）；≥ /mavros/state 的上报周期才有意义")
    # ---- 以下是**刻意的故障注入**，只为取证；06 的夹具做好后应挪进夹具侧
    p.add_argument("--qos", choices=("reliable", "best_effort"), default=Config.qos,
                   help="订阅 /mavros/state 的可靠性。★ 发布者是 `StateQoS`（reliable）⟹ "
                        "两个取值**都收得到**，这一档在 mavros 路径上不再能造出『零回调』")
    p.add_argument("--break", dest="break_", choices=BREAKS, default=Config.break_,
                   help="★ 刻意写错某一项（取证用，不是实现的一部分）。"
                        "`pitch-preconvert` / `yaw-preconvert` 模拟的是 **N2**：我们自己又转了一次 "
                        "FLU→FRD（转两次 = 不转，飞控最终收到的分毫不差），**不是**『符号写反』")
    p.add_argument("--break-scale", type=float, default=Config.break_scale,
                   help="--break thrust-scale 时把 T_max 乘上这个数（改错归一化常数）")
    return p


#: 必须在**建节点之前**挡住的两类参数：让节点算不出下一步的（0 / 非有限），
#: 以及写完第一帧日志之后才炸的（`--log-every 0`）。
_CHECKED_POSITIVE = ("--rate-hz", "--cmd-interval", "--pre-stream", "--mass", "--t-max")


def validate_args(args) -> None:
    """把"启动参数会炸"与"启动参数只是没意义"分开 —— 前者**现在**就拒，带一句话原因。

    ★ 为什么不能等它自己炸：`--rate-hz 0` 在 `1.0 / 0` 那行抛、`--log-every 0` 在
    `ticks % 0` 那行抛，而**第一帧日志已经落盘**了 ⟹ 验收脚本的"一帧都没发"体检
    （`dut_dead`，只在 `frames` 为空时触发）判不出来，后面每条判据的失败都会被算到
    被控对象头上 —— 正是本张要避免的归因污染。异常进的是定时器回调，节点带 traceback 退出。

    ★★ 2026-10-12 新增第三类：**归一化自洽**（`--mass` 与 `--t-max` 这一对）。
    它同样必须在建节点之前拒 —— 它是**静默**的（判据全绿、飞机上飘），
    而"跑完一整轮才发现悬停推力是 3.92 倍"正是这一条要避免的。
    """
    for name in _CHECKED_POSITIVE:
        val = getattr(args, name[2:].replace("-", "_"))
        if not math.isfinite(val) or val <= 0:
            raise SystemExit(f"{name} 要是正的有限数，收到 {val!r}")
    if int(args.log_every) < 1:
        raise SystemExit(f"--log-every 要是 ≥ 1 的整数，收到 {args.log_every!r}")
    try:
        params.check_hover_consistency(args.mass, args.t_max)
    except ValueError as exc:
        raise SystemExit(str(exc)) from exc


def main(argv=None) -> int:
    args = build_parser().parse_args(argv)
    validate_args(args)
    cfg = Config(
        t_max_n=args.t_max,
        rate_hz=args.rate_hz, mass_kg=args.mass,
        phases=tuple(args.phases) if args.phases else Config.phases,
        pre_stream_s=args.pre_stream,
        arm=not args.no_arm, mode_switch=not args.no_mode_switch,
        qos=args.qos,
        break_=args.break_, break_scale=args.break_scale,
        auto_exit_after_s=args.auto_exit_after, log_file=args.log_file,
        log_every=args.log_every, cmd_interval_s=args.cmd_interval,
    )

    rclpy.init(args=[a for a in sys.argv[:1]])
    node = OffboardSmoke(cfg)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.get_logger().info(node.summary())
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
