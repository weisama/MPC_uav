#!/usr/bin/env python3
"""ticket 05 的**最小 offboard 冒烟节点** —— 不含任何模型 / 代价 / 约束。

它只做三件事：① 按 50 Hz 发 offboard 心跳 + 角速率/推力 setpoint；
② 把 PX4 拉进 OFFBOARD（心跳先持续 ≥1 s，再解锁、再切模式）；
③ 发的指令来自一条**参数化时间轴**（[`commands.py`](commands.py)），不来自求解器。

**为什么要有这个节点**：`[05]` 要排除的四个静默失败（`px4_msgs` 版本不匹配 / QoS 不兼容 /
时间戳单位错 / 推力符号与归一化错）**都不报错、不崩溃**，只表现为"没反应"或"飞机朝错的方向加速"。
若把真 MPC（模型预测控制）和它们一起第一次接上真 PX4，出问题时分不清是哪一层。
所以本节点刻意**没有模型** —— 它错了，那就只可能是中间件或符号错了。

**与后续节点的关系**：符号映射**不在这里实现**，在
[`sign_map.py`](sign_map.py)（[`docs/control/sign-map.md`](../../../docs/control/sign-map.md) §8
要求 05 的 stub 与 06 的节点共用同一份）。等 [ticket 06](../../../.scratch/phase2/issues/06-mpc-node-fake-px4-fixture.md)
做出假 PX4 夹具后，本文件里那些 `--break-*` / `--timestamp-unit` / `--qos` 故障注入开关
**应该收进夹具侧**（夹具才是"外部世界"），见 06 的 Comments。

用法（args 与场景见 `docs/control/sitl-offboard-smoke.md`）：

    ros2 run nmpc offboard_smoke.py --phases lift:1.2:1.2:0,0,0 --phases hover:8:1.0:0,0,0
    python3 src/nmpc/nmpc/offboard_smoke.py --help      # 不经 colcon 也能跑（自己补 sys.path）
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

from px4_msgs.msg import (OffboardControlMode, VehicleCommand,  # noqa: E402
                          VehicleRatesSetpoint, VehicleStatus)

from nmpc import commands, sign_map  # noqa: E402

# PX4 的模式编号（`vehicle_status_s` / `vehicle_command_s` 的枚举，第一手见 px4_msgs）
NAV_STATE_OFFBOARD = 14
ARMING_STATE_ARMED = 2
# `VEHICLE_CMD_DO_SET_MODE` 的 param1/param2：启用自定义主模式，主模式 = OFFBOARD
MAV_MODE_FLAG_CUSTOM_MODE_ENABLED = 1.0
PX4_CUSTOM_MAIN_MODE_OFFBOARD = 6.0

#: PX4 的 `/fmu/out/*` 是 **BEST_EFFORT**（`dds_topics.h.em:62` 的 `best_effort_stream_id`），
#: 订阅侧写 `RELIABLE`（ROS 2 的默认值）就**一个回调都不进**，而话题照样列得出来。
QOS_BEST_EFFORT = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                             reliability=ReliabilityPolicy.BEST_EFFORT)
QOS_RELIABLE = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                          reliability=ReliabilityPolicy.RELIABLE)
#: 发布侧：`dds_topics.h.em:131` 给 PX4 的 reader 建的是 best_effort 流 ⟹ 我们也发 best_effort
#: （+ TRANSIENT_LOCAL 让后起的订阅者也能拿到最后一帧）。RELIABLE 的 writer 与 BEST_EFFORT
#: 的 reader 在 DDS 里是**兼容**的，所以这一侧错单位不会失联 —— 别把两件事混为一谈。
QOS_PUB = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                     reliability=ReliabilityPolicy.BEST_EFFORT,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)

BREAKS = ("none", "thrust-sign", "thrust-scale", "roll-sign", "pitch-sign", "yaw-sign")


@dataclass
class Config:
    rate_hz: float = 50.0
    mass_kg: float = 0.75          # SITL z0 同构机的 SIH_MASS；实机是 750 g
    t_max_n: float = 11.8          # 按电池的可用总推力（z0）
    phases: tuple = ("lift:1.2:1.2:0,0,0", "hover:8:1.0:0,0,0")
    pre_stream_s: float = 1.5      # 切 OFFBOARD 前先发多久心跳（判据要求 ≥1 s）
    arm: bool = True
    mode_switch: bool = True
    timestamp_unit: str = "us"
    #: 取证用：把时间戳整体往过去（负）/ 未来（正）挪这么多微秒。
    #: ★ 这才是**可达的**静默失败：PX4 记下的值 = `min(我们发的 − time_offset, hrt)`，
    #: 正确的 epoch µs 算出来是 `hrt + Δ`（Δ = 同步滞后，实测 0.08 ~ 0.24 s）⟹ 记下的是 `hrt`；
    #: 往过去挪 lag 秒、且 **lag > Δ** 才会换支，age = lag − Δ。余量是 `COM_OF_LOSS_T`（1 s）
    #: 减掉 Δ ⟹ **挪 1 s 还进得去（贴边）**，挪 2 s 才被判"不新鲜"，**进不了 OFFBOARD 也不报错**。
    timestamp_shift_us: int = 0
    qos: str = "best_effort"
    break_: str = "none"
    break_scale: float = 2.0
    auto_exit_after_s: float = 0.0
    log_file: str = ""
    log_every: int = 50
    #: 解锁 / 切模式命令的最小间隔。判据是"发到成功为止"，但 50 Hz 地重发只会把
    #: commander 淹掉（一秒钟 50 条 `DO_SET_MODE` 没有任何好处，日志也没法看）。
    cmd_interval_s: float = 0.5


class OffboardSmoke(Node):
    """把 PX4 拉进 OFFBOARD 并照抄一条时间轴发指令的最小节点。"""

    def __init__(self, cfg: Config):
        super().__init__("offboard_smoke")
        self.cfg = cfg
        self.program = commands.parse_program(cfg.phases)
        self.hover_n = commands.hover_force_n(cfg.mass_kg)

        self.pub_mode = self.create_publisher(
            OffboardControlMode, "/fmu/in/offboard_control_mode", QOS_PUB)
        self.pub_rates = self.create_publisher(
            VehicleRatesSetpoint, "/fmu/in/vehicle_rates_setpoint", QOS_PUB)
        self.pub_cmd = self.create_publisher(
            VehicleCommand, "/fmu/in/vehicle_command", QOS_PUB)

        sub_qos = QOS_BEST_EFFORT if cfg.qos == "best_effort" else QOS_RELIABLE
        self.sub_status = self.create_subscription(
            VehicleStatus, "/fmu/out/vehicle_status", self._on_status, sub_qos)

        self.status = None
        self.status_count = 0
        self.armed_cmd_sent = 0
        self.mode_cmd_sent = 0
        self.published = 0
        self.first_publish_ns = None
        self.last_publish_ns = None
        self.offboard_since_ns = None
        self.last_cmd_ns = 0
        self.ticks = 0
        self._log = open(cfg.log_file, "w", encoding="utf-8") if cfg.log_file else None

        self.get_logger().info(
            f"指令时间轴（悬停推力 {self.hover_n:.4f} N，T_max {cfg.t_max_n} N ⇒ "
            f"悬停归一化 {sign_map.px4_rates_setpoint((0, 0, 0), self.hover_n, cfg.t_max_n).thrust_body_z:.4f}）:\n"
            + commands.describe(self.program, self.hover_n))
        self.get_logger().info(
            f"取证开关：时间戳单位={cfg.timestamp_unit}"
            + (f"（挪 {cfg.timestamp_shift_us:+d} µs）" if cfg.timestamp_shift_us else "")
            + f" 订阅 QoS={cfg.qos} 故障注入={cfg.break_}"
            + (f"（×{cfg.break_scale}）" if cfg.break_ == "thrust-scale" else ""))

        self.create_timer(1.0 / cfg.rate_hz, self._tick)

    # ---------------------------------------------------------------- 输入侧

    def _on_status(self, msg: VehicleStatus) -> None:
        self.status = msg
        self.status_count += 1

    def _timestamp(self) -> int:
        """PX4 要**微秒**（`msg/*.msg` 逐字写 `(microseconds)`）；这里刻意能发错单位（取证用）。

        ★ PX4 收到之后**不会原样存下我们的时间戳**。生成的反序列化器（`uORB/ucdr/<话题>.h`）是：

            if (topic.timestamp == 0) topic.timestamp = hrt_absolute_time();
            else topic.timestamp = math::min(topic.timestamp - time_offset, hrt_absolute_time());

        其中 `time_offset = session->time_offset / 1000`（`dds_topics.h.em:103`，µs），
        而 `session->time_offset` 由 uXRCE 的时钟同步每秒刷新
        （`uxrce_dds_client.cpp:78`）。`timestamp` 是 **uint64**、`time_offset` 是 int64 ⇒
        `topic.timestamp - time_offset` 走**无符号**运算 —— 往过去算就是**下溢**。

        把记下时间戳记作 `age = hrt − 记下的值`。核心是那个**秒级滞后 Δ**：
        `time_offset` 每秒才刷新一次，所以 `now_us − time_offset = hrt + Δ`，
        Δ ∈ [0, 1) s（本次实测 **≈0.18 s**，与 `/fmu/out/*` 上量到的 −0.19 s 对得上）。
        受控实测（z0 SITL，场景见 `tools/px4-sitl/verify_offboard_smoke.py`）：

        | 我们发 | `ts − time_offset` | `min(·, hrt)` 记下 | age 实测 | 后果 |
        |---|---|---|---|---|
        | `now_us` | `hrt + Δ`（≥ hrt） | `hrt` | 0.000 s | ✓ 正常 |
        | `now_us − 1 s` | `hrt − (1 − Δ)` | 这一支 | **0.83 ~ 0.92 s**（随 Δ 漂） | ★ **仍 < `COM_OF_LOSS_T`(1 s) ⟹ 照样进 OFFBOARD**，余量只剩 Δ |
        | `now_us − 2 s` | `hrt − (2 − Δ)` | 这一支 | ≈ 1.8 s | ⟹ 超时 ⟹ **进不了 OFFBOARD**（且不报错）|
        | `now_ms` | 下溢成 1.8e19 | `hrt` | 0.004 s | 静默无害 |
        | `now_ns` | 1.8e18（比 hrt 大 12 个数量级） | `hrt` | 0.008 s | 静默无害 |
        | `0` | 不过 `min()`，走 `if (ts == 0) ts = hrt` | `hrt` | 0.004 s | 静默无害 |

        ⚠️ 第一版交付文档曾把"挪 1 s"写成一定进不去（探针里读到的 1.6 s 把发布窗口也算进去了）；
        实测边界是 **1 s − Δ**，会随 Δ 漂 ⟹ 真正稳的判据是"挪 ≥ 2 s"。
        所以：**发错单位是静默的**（下溢/巨大正数都被 `min()` 钳到 hrt，比正确的 µs 还"新鲜"），
        而"往过去挪够 2 s"才是真会被判不新鲜的那一侧。
        """
        ns = time.time_ns()
        ts = {"us": ns // 1_000, "ms": ns // 1_000_000, "ns": ns,
              # 0 不取"某个单位"，它走的是反序列化里**显式**的那一支：
              # `if (timestamp == 0) timestamp = hrt` —— 与"下溢后被 `min()` 钳住"是两条不同的代码路径。
              "zero": 0}[self.cfg.timestamp_unit]
        return ts + self.cfg.timestamp_shift_us

    # ---------------------------------------------------------------- 输出侧

    def _map(self, omega, thrust_n):
        """内部量 → PX4 四个字段。**唯一实现**在 `sign_map`，这里只做取证用的刻意破坏。"""
        t_max = self.cfg.t_max_n
        if self.cfg.break_ == "thrust-scale":
            t_max = self.cfg.t_max_n * self.cfg.break_scale
        sp = sign_map.px4_rates_setpoint(omega, thrust_n, t_max)
        if self.cfg.break_ == "thrust-sign":
            sp = replace(sp, thrust_body_z=-sp.thrust_body_z)
        elif self.cfg.break_ == "roll-sign":
            sp = replace(sp, roll=-sp.roll)
        elif self.cfg.break_ == "pitch-sign":
            sp = replace(sp, pitch=-sp.pitch)
        elif self.cfg.break_ == "yaw-sign":
            sp = replace(sp, yaw=-sp.yaw)
        return sp

    def _send_command(self, command: int, param1: float, param2: float = 0.0) -> None:
        msg = VehicleCommand()
        msg.timestamp = self._timestamp()
        msg.command = command
        msg.param1 = float(param1)
        msg.param2 = float(param2)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self.pub_cmd.publish(msg)

    def _tick(self) -> None:
        self.ticks += 1
        now_ns = time.time_ns()
        ts = self._timestamp()
        # ★ 一律 `int()`：`px4_msgs` 的枚举字段是 uint8，直接塞进 `json.dumps`
        #   会在**第一帧**就抛 `TypeError`，而定时器回调里抛异常 = 整个 spin 挂掉
        #   （症状是"节点起来一秒就没了、日志文件是空的"）。
        armed = self.status is not None and int(self.status.arming_state) == ARMING_STATE_ARMED
        nav = int(self.status.nav_state) if self.status is not None else -1

        # 程序时钟：**切进 OFFBOARD 的那一刻归零** —— 否则模式切换花掉的秒数会吃掉第一段
        if self.offboard_since_ns is None and nav == NAV_STATE_OFFBOARD:
            self.offboard_since_ns = now_ns
        t_program = 0.0 if self.offboard_since_ns is None else (now_ns - self.offboard_since_ns) / 1e9

        thrust_n, omega = commands.command_at(self.program, t_program, self.hover_n)
        sp = self._map(omega, thrust_n)

        mode = OffboardControlMode()
        mode.timestamp = ts
        mode.position = False
        mode.velocity = False
        mode.acceleration = False
        mode.attitude = False
        mode.body_rate = True
        mode.actuator = False
        self.pub_mode.publish(mode)

        rates = VehicleRatesSetpoint()
        rates.timestamp = ts
        rates.roll = float(sp.roll)
        rates.pitch = float(sp.pitch)
        rates.yaw = float(sp.yaw)
        rates.thrust_body[0] = 0.0
        rates.thrust_body[1] = 0.0
        rates.thrust_body[2] = float(sp.thrust_body_z)
        self.pub_rates.publish(rates)

        self.published += 1
        self.first_publish_ns = self.first_publish_ns or now_ns
        self.last_publish_ns = now_ns

        if self._log is not None:
            # ⚠️ 每个字段都要 `float()`：`rates.thrust_body[2]` 这类是 ROS 的 **float32**，
            #    `json.dumps` 不认（`TypeError`），而定时器回调里抛异常 = 整个 spin 挂掉。
            self._log.write(json.dumps({
                "wall_ns": now_ns, "t_program_s": round(float(t_program), 4),
                # ★ `ts` 是**发出去的**时间戳字段原值 —— 取证时要拿它跟 PX4 侧
                #   `listener` 读到的值对账（单位错的那两轮全靠这一对比）。
                "ts": ts, "ts_unit": self.cfg.timestamp_unit,
                "ts_shift_us": self.cfg.timestamp_shift_us,
                "roll": float(rates.roll), "pitch": float(rates.pitch),
                "yaw": float(rates.yaw),
                "thrust_body_z": float(rates.thrust_body[2]), "thrust_n": float(thrust_n),
                "nav_state": nav,
                "arming_state": -1 if self.status is None else int(self.status.arming_state),
                "status_cb": self.status_count,
            }) + "\n")
            self._log.flush()

        # ---- 拉进 OFFBOARD：心跳先发够 `pre_stream_s` ⟹ 解锁 ⟹ 切模式（每一步都等状态确认，不靠 sleep）
        # ★ 闸门开在**已发布时长**上，不是"起来之后等一会儿"：判据要的是"切 OFFBOARD 之前
        #   setpoint 已以 ≥2 Hz 发了 ≥1 s"，量的是**发出去的东西**，不是进程年龄。
        #   ⚠️ 这个 `pre_stream_s` 曾经只是个没接线的旋钮（`Config` 里有、`--pre-stream` 能改、
        #   但没人读）—— 于是实测 baseline 在首帧后 0.78 s 就进了 OFFBOARD，判据其实没被满足。
        streamed_s = (now_ns - self.first_publish_ns) / 1e9 if self.first_publish_ns else 0.0
        cmd_allowed = (now_ns - self.last_cmd_ns) / 1e9 >= self.cfg.cmd_interval_s
        armed_enough = streamed_s >= self.cfg.pre_stream_s
        if self.cfg.arm and armed_enough and not armed and self.status is not None \
                and self.status.pre_flight_checks_pass:
            if cmd_allowed:
                self._send_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
                self.armed_cmd_sent += 1
                self.last_cmd_ns = now_ns
        elif self.cfg.mode_switch and armed_enough and armed and nav != NAV_STATE_OFFBOARD \
                and cmd_allowed:
            self._send_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
                               MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, PX4_CUSTOM_MAIN_MODE_OFFBOARD)
            self.mode_cmd_sent += 1
            self.last_cmd_ns = now_ns

        if self.ticks % self.cfg.log_every == 0:
            self.get_logger().info(
                f"t={t_program:5.1f}s 已发 {self.published:5d} 帧 | nav={nav} armed={int(armed)} "
                f"| F={thrust_n:6.3f}N ω={tuple(round(w, 3) for w in omega)} "
                f"⟹ roll={rates.roll:+.3f} pitch={rates.pitch:+.3f} yaw={rates.yaw:+.3f} "
                f"thrust_body[2]={rates.thrust_body[2]:+.4f} | status回调={self.status_count}")

        program_end = sum(p.duration_s for p in self.program)
        if self.cfg.auto_exit_after_s > 0 and t_program > program_end + self.cfg.auto_exit_after_s:
            self.get_logger().info(f"程序走完 + {self.cfg.auto_exit_after_s}s ⟹ 自行退出")
            raise SystemExit(0)

    # ---------------------------------------------------------------- 收尾

    def summary(self) -> str:
        span = (self.last_publish_ns - self.first_publish_ns) / 1e9 if self.first_publish_ns else 0.0
        rate = (self.published - 1) / span if span > 0 else float("nan")
        return (f"共发 {self.published} 帧，跨 {span:.2f}s ⟹ 实际 {rate:.1f} Hz；"
                f"解锁命令 {self.armed_cmd_sent} 条、模式命令 {self.mode_cmd_sent} 条；"
                f"收到 vehicle_status {self.status_count} 条（订阅 QoS={self.cfg.qos}）")

    def close(self) -> None:
        if self._log is not None:
            self._log.close()
            self._log = None


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description="ticket 05 的最小 offboard 冒烟节点（无模型）",
                               formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--phases", action="append", metavar="名字:时长:悬停倍数:ωx,ωy,ωz",
                   help="指令时间轴，可重复；最后一段一直保持（默认 lift 1.2s×1.2 ⟹ hover 8s）")
    p.add_argument("--rate-hz", type=float, default=Config.rate_hz)
    p.add_argument("--mass", type=float, default=Config.mass_kg, help="kg；悬停推力 = m·g")
    p.add_argument("--t-max", type=float, default=Config.t_max_n, help="N；按电池的可用总推力")
    p.add_argument("--pre-stream", type=float, default=Config.pre_stream_s,
                   help="切 OFFBOARD 前先发这么久心跳（秒）")
    p.add_argument("--no-arm", action="store_true", help="不发解锁命令（只发 setpoint）")
    p.add_argument("--no-mode-switch", action="store_true", help="不发切模式命令")
    p.add_argument("--auto-exit-after", type=float, default=0.0,
                   help="程序走完后再过这么多秒自行退出（0 = 一直发到被杀）")
    p.add_argument("--log-file", default="", help="把每一帧 setpoint 写成 JSONL（取证用）")
    p.add_argument("--log-every", type=int, default=Config.log_every, help="每 N 帧打一行日志")
    p.add_argument("--cmd-interval", type=float, default=Config.cmd_interval_s,
                   help="解锁/切模式命令的最小重发间隔（秒）")
    # ---- 以下四个是**刻意的故障注入**，只为取证；06 的夹具做好后应挪进夹具侧
    p.add_argument("--timestamp-unit", choices=("us", "ms", "ns", "zero"),
                   default=Config.timestamp_unit,
                   help="★ 默认 us。发错单位的后果见交付文档：不是「被丢弃」，是「心跳永远新鲜」")
    p.add_argument("--timestamp-shift-us", type=int, default=Config.timestamp_shift_us,
                   help="★ 把时间戳往过去（负）/ 未来（正）挪 µs；挪 1 s 只到 age≈0.79 s"
                        "（贴边进得去），要挪 **2 s**（-2e6）才越过 COM_OF_LOSS_T")
    p.add_argument("--qos", choices=("best_effort", "reliable"), default=Config.qos,
                   help="★ 订阅侧 QoS；reliable 与 PX4 的 best_effort 发布者不兼容 ⟹ 收不到状态")
    p.add_argument("--break", dest="break_", choices=BREAKS, default=Config.break_,
                   help="★ 刻意写反某一项（取证用，不是实现的一部分）")
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
    """
    for name in _CHECKED_POSITIVE:
        val = getattr(args, name[2:].replace("-", "_"))
        if not math.isfinite(val) or val <= 0:
            raise SystemExit(f"{name} 要是正的有限数，收到 {val!r}")
    if int(args.log_every) < 1:
        raise SystemExit(f"--log-every 要是 ≥ 1 的整数，收到 {args.log_every!r}")


def main(argv=None) -> int:
    args = build_parser().parse_args(argv)
    validate_args(args)
    cfg = Config(
        rate_hz=args.rate_hz, mass_kg=args.mass, t_max_n=args.t_max,
        phases=tuple(args.phases) if args.phases else Config.phases,
        pre_stream_s=args.pre_stream,
        arm=not args.no_arm, mode_switch=not args.no_mode_switch,
        timestamp_unit=args.timestamp_unit, timestamp_shift_us=args.timestamp_shift_us,
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
