#!/usr/bin/env python3
"""Phase 2 的 **MPC（模型预测控制）节点** —— 把 `ocp` + `sign_map` 接到 PX4 的 `/fmu/*` 上。

层级是 **(C) 级 CTBR**：本节点只发"**总推力 + 机体角速率**"，**姿态环与混控仍留在飞控里**
（理由与备选见 [`phase2-implementation.md`](../../../docs/control/phase2-implementation.md) §2.1）。

★ 本文件是**唯一**一处把"求解结果"变成"话题上的一帧"的地方。所以这里只有机械的三件事：
  取一拍的决策（`node_core.NodeCore`）→ 解一次 OCP（`ocp.OfflineOcp`）→ 映射 + 发布
  （`sign_map.px4_rates_setpoint`）。**判断逻辑不在这个文件里** —— 闸门、外推、参考构造
  都在 `node_core.py`（零 ROS、可单测），符号与归一化在 `sign_map.py`（唯一一份）。
  这样"节点没发"与"节点发错了"才分得开。

接口（票面验收项逐条，都有第一手依据）：

| 方向 | 话题 | QoS | 依据 |
|---|---|---|---|
| 订阅 | `/mocap/pose_enu`（**ENU/FLU**，按 M7 状态源改到动捕） | 可靠 / depth 10 | `mocap_rel_pose_node.cpp:93` `QoS(10)` |
| 订阅 | `/fmu/out/vehicle_status` | **BEST_EFFORT** / depth 1 | PX4 的 writer 是 best_effort（`utilities.hpp:59-63`）⟹ 订阅侧写 RELIABLE 就**一个回调都不进** |
| 发布 | `/fmu/in/vehicle_rates_setpoint` | BEST_EFFORT + TRANSIENT_LOCAL / depth 1 | PX4 的 reader 是 best_effort（`utilities.hpp:114-119`）；TRANSIENT_LOCAL 让晚起的 reader 也能拿到最后一帧 |
| 发布 | `/fmu/in/offboard_control_mode`（`body_rate = True`，其余 False） | 同上 | 同上 |
| 发布 | `/mpc/predicted_path`、`/mpc/reference`（RViz 用，`nav_msgs/Path` / `Marker`） | 默认（可靠） | 本仓自己的观测面，不接飞控 |

**发不发**（两张表合成一条规则）：`offboard_control_mode` = **节点活着就发**；
`vehicle_rates_setpoint` = **只有** armed + OFFBOARD + 动捕新鲜（闸门在 `node_core.cycle()`）。
两者的完整理由见下面那段 ★★。

★ **时间戳一律微秒**（`msg/*.msg` 逐字写 `(microseconds)`；PX4 侧还会做
`min(ts − time_offset, hrt)`，见 `offboard_smoke._timestamp` 那份受控实测表）。
本节点的 `_timestamp()` 与 05 的 stub **同一份实现口径**：`time.time_ns() // 1000`。

★ **100 Hz 求解 / 50 Hz 心跳**：D6（控制频率）由 ticket 04 的实测给出 —— 求解器自报
`time_tot` 最大 3.45–3.58 ms ⟹ 100 Hz 有余量；而**整周期墙钟**曾出现过 54–56 ms 的抢占
（同一拍自报只有 3.4 ms）⟹ 节点侧必须绑核（见 `--ros-args -p cpu:=…` 与 `timing.py`）。
心跳按 `rate_hz / heartbeat_hz` 抽帧发（默认 2 抽 1 = 50 Hz）——它只需要 ≥2 Hz
（`COM_OF_LOSS_T` = 1 s 是 PX4 的判据），发得快是为了让"流断了"这件事在**秒级**就能看出来。

★★ **心跳无条件发，`vehicle_rates_setpoint` 才过闸门** —— 这不是风格，是**进不进得去**
OFFBOARD 的必要条件（票面验收项「未解锁 / 未在 offboard 时**不发布**」按"不发**指令**"读，
理由如下，**第一手**）：

```cpp
// src/modules/commander/HealthAndArmingChecks/checks/offboardCheck.cpp:40-65（z0 真身 4a0e65f2）
reporter.failsafeFlags().offboard_control_signal_lost = true;      // ← 每轮先置"丢"
if (_offboard_control_mode_sub.copy(&offboard_control_mode)) {
    bool data_is_recent = hrt_absolute_time() < offboard_control_mode.timestamp
                          + (hrt_abstime)(_param_com_of_loss_t.get() * 1_s);
    bool offboard_available = (…|| offboard_control_mode.body_rate || …) && data_is_recent;
    reporter.failsafeFlags().offboard_control_signal_lost = !offboard_available;
}
// checks/modeCheck.cpp:126-134
if (reporter.failsafeFlags().offboard_control_signal_lost && mode_req_offboard_signal != 0) {
    reporter.armingCheckFailure(…, "No offboard signal");
    reporter.clearCanRunBits(mode_req_offboard_signal);            // ← OFFBOARD 直接不可运行
}
```

⟹ **OFFBOARD 只有在"心跳已经以 ≥2 Hz 发过、且时间戳落在 `COM_OF_LOSS_T` 之内"时才能被切进去**。
若节点把心跳也闸在 `nav_state == OFFBOARD` 上，就成了**死锁**：不发心跳 ⟹ PX4 不放行
OFFBOARD ⟹ 闸门永远不开 ⟹ 永远不发。**真机与 SITL 上都会表现为"这飞机根本进不了 offboard"**。
（05 的 stub 早就是这么做的：它无条件发两个话题，并显式"心跳先发够 `pre_stream_s` ⟹ 解锁 ⟹
切模式"——见 `offboard_smoke._tick`。那份经验在本票里被**写成了依据**，不再是习惯。）

反向的两条也很重要，同样第一手：

- `offboard_control_mode` 的六个布尔位**只在 OFFBOARD 里**才影响 `vehicle_control_mode`
  （`ModeUtil/control_mode.cpp:115` 起，整个分支挂在 `case NAVIGATION_STATE_OFFBOARD:` 下面）
  ⟹ 在 POSCTL/MANUAL 里发这个心跳**不会**改变任何一个控制回路的选择，只有一个副作用：
  把 `offboard_control_signal_lost` 清掉（这正是我们要的）。
- ⚠️ **"不发 setpoint" ≠ "飞机松手"**：`COM_OF_LOSS_T` 那条判据看的是**心跳**，不是 setpoint。
  所以闸门关掉 `vehicle_rates_setpoint` 之后，只要心跳还在，PX4 会**继续沿用最后一帧命令**
  （`mc_rate_control` 没有新消息时不会清零 —— 见 ticket 07 的失效路径分析）。
  这正是"兜底该怎么做"必须由 [ticket 07](../../../.scratch/phase2/issues/07-solver-failure-fallback.md)
  给出真结论的原因；本票只把**接缝**留清楚：`-p heartbeat_always:=false` 会把心跳一起闸掉
  （= 交给 PX4 自己的失效保护，它在 `COM_OF_LOSS_T` 之后自己退出 OFFBOARD），
  **默认 True**，因为"进不去 OFFBOARD"是当场就飞不了的错，而"该松手时没松手"是 07 的活。

★ **状态外推** `x + v·τ`（τ = 50 ms）与**首拍不发**（`WARMUP_CYCLES`）都在 `node_core` 里，
理由见那边的 docstring（后者是第一手的：`SQP_RTI` 的第 0 次 `solve()` 只是"从出厂迭代点走一步"）。

★ **饱和表现为"打满"**：本节点**不做任何裁剪** —— 唯一的限幅点是 OCP 自己的
`lbu/ubu`（`F ∈ [0, T_max]`、`ω ∈ ±rate_max`）。所以"它在顶约束"与"它在正常工作"在话题上分得开：顶约束时 `ω` 恰好停在 `rate_max` 上，而不是被谁悄悄按比例缩回去。
⚠️ 反过来，若哪天有人在这里加一句 `clip()`，"在顶约束"就再也看不见了。

★ **回退**：Phase 2 是**加一条支路，不是替换** —— 本节点只发 `/fmu/in/*`，**不碰 mavros**，
  Phase 1 的 `(A)` 级链（`control_node` + mavros）原样可用。切回 = 不启本节点、
  由 `control_node` 接管（未解锁 / 未 OFFBOARD 时本节点一帧都不发，理由见 `node_core` 的闸门）。
"""

from __future__ import annotations

import json
import math
import os
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

if __package__ in (None, ""):  # 直接 `python3 …/mpc_node.py` 时把包目录挂上
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import numpy as np  # noqa: E402
import rclpy  # noqa: E402
from geometry_msgs.msg import PoseStamped  # noqa: E402
# ⚠️ **必须改名**：`nav_msgs/Path` 与 `pathlib.Path` 同名，直接 import 会把上面那个
#    悄悄换掉 —— 症状是"读 `/proc/self/status` 时 `Path.__init__() takes 1 positional argument`"，
#    即一个与可视化毫无关系的报错（本文件第一次跑就是这么炸的）。
from nav_msgs.msg import Path as NavPath  # noqa: E402
from rclpy.executors import ExternalShutdownException  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,  # noqa: E402
                       ReliabilityPolicy)
from visualization_msgs.msg import Marker  # noqa: E402

from px4_msgs.msg import (OffboardControlMode, VehicleRatesSetpoint,  # noqa: E402
                          VehicleStatus)

from nmpc import sign_map  # noqa: E402
from nmpc.node_core import ControlConfig, NodeCore, StopConfig  # noqa: E402
from nmpc.ocp import OfflineOcp  # noqa: E402
from nmpc.params import Z0Params  # noqa: E402

#: PX4 的 `/fmu/out/*` 是 **BEST_EFFORT**（`utilities.hpp:59-63`）⟹ 订阅侧写 RELIABLE
#: （ROS 2 的默认值）就**一个回调都不进**，而话题照样列得出来。
QOS_SUB_STATUS = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                            reliability=ReliabilityPolicy.BEST_EFFORT)
#: 发布侧：PX4 的 reader 是 best_effort（`utilities.hpp:114-119`）+ TRANSIENT_LOCAL
#: 让晚起的 reader 也能拿到最后一帧。这与 05 的 stub 是同一份口径（那边在 SITL 上验过）。
QOS_PUB_FMU = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                         reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
#: `/mocap/pose_enu` 是 `mocap` 节点用 `QoS(10)`（**可靠**）发的
#: （`mocap_rel_pose_node.cpp:93`，与 `bench/vrpn_source.py` 的 `mocap_qos()` 一致）。
QOS_MOCAP = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=10,
                       reliability=ReliabilityPolicy.RELIABLE)

TOPIC_MOCAP = "/mocap/pose_enu"
TOPIC_STATUS = "/fmu/out/vehicle_status"
TOPIC_RATES = "/fmu/in/vehicle_rates_setpoint"
TOPIC_MODE = "/fmu/in/offboard_control_mode"
TOPIC_PATH = "/mpc/predicted_path"
TOPIC_REF = "/mpc/reference"

#: 节点名 = 可执行文件名（`ros2 node list` 里一眼认出来）
NODE_NAME = "mpc_node"


@dataclass
class NodeParams:
    """把所有 `--ros-args -p` 收敛成一份**结构化的**配置 —— 启动时原样打进日志。

    ★ 为什么要原样打一遍：rclpy 与 rclcpp 不一样，**多写一个不存在的参数不会报错**
      （它会静静地待在那里，谁也没读）。所以正确性靠两件事守：① `_reject_unknown_params()`
      把拼错的参数名变成**启动即退**（与 rclcpp 的行为对齐）；② 这一份日志，
      让"我改了参数但没生效"在证据里一眼看出来。
    """

    rate_hz: float = 100.0
    heartbeat_hz: float = 50.0
    viz_hz: float = 10.0
    #: 心跳**无条件发**（默认）—— ★★ 理由见模块 docstring：心跳是 PX4 放行 OFFBOARD 的前提
    #: （`offboardCheck.cpp:40-65` + `modeCheck.cpp:126`），把它闸掉就是**进不去 OFFBOARD**。
    #: 置 False 时心跳与 setpoint 同生死（= 交给 PX4 的 `COM_OF_LOSS_T` 失效保护）。归 ticket 07。
    heartbeat_always: bool = True
    #: 求解用的模型参数（`t_max` 由 `hover_thrust` 导出，**不能**单独给 —— 见 `params.py`）
    mass: float = 0.75
    hover_thrust: float = 0.6235
    #: 状态外推 τ [s]（票面 50 ms）
    tau_s: float = 0.05
    #: 参考点相对测量位姿的偏移 [m]（ENU）
    hold_offset: tuple = (0.0, 0.0, 0.0)
    keep_yaw: bool = True
    hold_yaw_deg: float = 0.0
    #: 停机条件（**没有通道号**，理由见 `node_core`）
    mocap_timeout_s: float = 0.5
    status_timeout_s: float = 0.75
    stop_on_nav_state_exit: bool = True
    stop_on_armed_drop: bool = True
    warmup_cycles: int = 5
    #: 绑核（-1 = 不绑）。★ D6 的实测结论：绑核把最坏周期从 5.9 ms 压到 4.0 ms
    #: （`docs/control/offline-ocp.md` §5），而**整周期墙钟**的 54 ms 那一拍是被抢占 ⟹
    #: 节点侧必须有绑核这一档，否则 100 Hz 在有人编译的机器上是假的。
    cpu: int = -1
    #: >0 = 申请 SCHED_FIFO（要 CAP_SYS_NICE，失败只告警不退出）。默认 0 = 不动调度策略。
    rt_priority: int = 0
    #: RViz 的固定坐标系（与动捕上游一致，默认 "world" —— `vrpn_listener` 的默认 frame）
    rviz_frame: str = "world"
    log_file: str = ""
    log_every: int = 100

    def control_config(self) -> ControlConfig:
        yaw = None if self.keep_yaw else math.radians(self.hold_yaw_deg)
        return ControlConfig(
            tau_s=self.tau_s,
            hold_offset=tuple(float(v) for v in self.hold_offset),
            hold_yaw=yaw,
            stop=StopConfig(
                mocap_timeout_s=self.mocap_timeout_s,
                status_timeout_s=self.status_timeout_s,
                stop_on_nav_state_exit=self.stop_on_nav_state_exit,
                stop_on_armed_drop=self.stop_on_armed_drop,
                warmup_cycles=self.warmup_cycles,
            ),
        )

    def z0_params(self) -> Z0Params:
        return Z0Params(mass=self.mass, hover_thrust=self.hover_thrust)

    def describe(self) -> str:
        return ("rate=%g Hz 心跳=%g Hz viz=%g Hz | τ=%g s 参考偏移=%s yaw=%s"
                " | 模型 mass=%g kg hover_thrust=%g ⇒ T_max=%.4f N"
                " | 停机(mocap>%g s / status>%g s / 离开 offboard=%s / 掉 armed=%s)"
                " | 心跳%s"
                % (self.rate_hz, self.heartbeat_hz, self.viz_hz, self.tau_s,
                   tuple(round(float(v), 3) for v in self.hold_offset),
                   "保持当前" if self.keep_yaw else f"{self.hold_yaw_deg:g}°",
                   self.mass, self.hover_thrust, self.z0_params().t_max,
                   self.mocap_timeout_s, self.status_timeout_s,
                   self.stop_on_nav_state_exit, self.stop_on_armed_drop,
                   "无条件发（闸门只管 setpoint）" if self.heartbeat_always
                   else "与 setpoint 同生死（交给 PX4 的 COM_OF_LOSS_T）"))


class MpcNode(Node):
    """100 Hz 求解 + 50 Hz 心跳 + 10 Hz RViz 的 (C) 级 CTBR（总推力 + 机体角速率）节点。"""

    def __init__(self, params: NodeParams, ocp: Optional[OfflineOcp] = None):
        super().__init__(NODE_NAME)
        self.params = params
        self.z0 = params.z0_params()

        self._apply_cpu_affinity()

        # ★ 求解器在这一行**代码生成 + 编译 C**（首次几十秒，之后复用 `$MPC_CODEGEN_DIR`）
        #   ⟹ 绑核必须在它**之前**（编译出来的 `.so` 由这个进程执行，绑核改的是进程亲和性，
        #   但"先绑再建"能让首次编译也落在同一个核上，读数才有可比性）。
        self.ocp = ocp if ocp is not None else OfflineOcp(params=self.z0)
        self.config = params.control_config()
        self.core = NodeCore(self.z0, self.config, self.ocp.config.N)

        # —— 上游 ——
        self.sub_mocap = self.create_subscription(
            PoseStamped, TOPIC_MOCAP, self._on_mocap, QOS_MOCAP)
        self.sub_status = self.create_subscription(
            VehicleStatus, TOPIC_STATUS, self._on_status, QOS_SUB_STATUS)

        # —— 下游（飞控）——
        self.pub_rates = self.create_publisher(VehicleRatesSetpoint, TOPIC_RATES, QOS_PUB_FMU)
        self.pub_mode = self.create_publisher(OffboardControlMode, TOPIC_MODE, QOS_PUB_FMU)

        # —— 观测面（RViz）——
        self.pub_path = self.create_publisher(NavPath, TOPIC_PATH, 10)
        self.pub_ref = self.create_publisher(Marker, TOPIC_REF, 10)

        # —— 簿记 ——
        self.ticks = 0
        self.solves = 0
        self.published = 0
        self.heartbeats = 0
        self.gated = {}                  # reason → 次数（停机原因统计，进证据）
        self.solve_ms_max = 0.0
        self.degraded_count = 0
        self.rejected = 0
        self.first_publish_ns: Optional[int] = None
        self.last_publish_ns: Optional[int] = None
        #: 喂给下一次 `solve(u_ref=…)` 的前馈。⚠️ 必须**一开始就有值**：`prediction=False`
        #: 时 `u_ref` 不能省（`OfflineOcp.solve` 会 raise，见那边的 docstring），而热身拍
        #: （`publish=False`）**也要解**。首帧用悬停前馈 —— 与 `OfflineOcp` 自己的首帧缺省同源。
        self._u_ref: np.ndarray = np.tile(
            np.array([self.z0.t_hover, 0.0, 0.0, 0.0]), (self.ocp.config.N, 1))
        self._last_cycle = None
        self._log = open(params.log_file, "w", encoding="utf-8") if params.log_file else None

        # 心跳抽帧：`rate_hz / heartbeat_hz` 取整（默认 100/50 = 2）
        self._heartbeat_every = max(1, int(round(params.rate_hz / max(1e-3, params.heartbeat_hz))))

        self.get_logger().info(
            f"MPC（模型预测控制）节点启动：{params.describe()}\n"
            f"  OCP：N={self.ocp.config.N} tf={self.ocp.config.tf} s（内部步长 {self.ocp.dt * 1e3:.1f} ms）"
            f"；T_max={self.z0.t_max:.4f} N ⟹ 悬停归一化推力 "
            f"{sign_map.px4_rates_setpoint((0, 0, 0), self.z0.t_hover, self.z0.t_max).thrust_body_z:+.4f}"
            f"；CPU 亲和性={self._cpus_allowed()}")
        self.get_logger().info(
            "接口：订阅 %s（可靠/depth 10）+ %s（**BEST_EFFORT**/depth 1）；"
            "发布 %s 与 %s（BEST_EFFORT + TRANSIENT_LOCAL/depth 1）"
            % (TOPIC_MOCAP, TOPIC_STATUS, TOPIC_RATES, TOPIC_MODE))

        self.create_timer(1.0 / params.rate_hz, self._tick)
        self.create_timer(1.0 / max(1e-3, params.viz_hz), self._viz_tick)

    # ─────────────────────────── 进程侧设置 ───────────────────────────

    def _apply_cpu_affinity(self) -> None:
        """把本进程钉在一个核上（`-p cpu:=4`）。★ 这是 D6 的**前提之一**，不是优化。

        第一手（`offline-ocp.md` §5，同一份代码、三轮取最坏）：绑核后中位数 −29 %、
        最大值 −33 %；而**整周期墙钟**的 max 曾出现 54–56 ms —— 同一拍求解器自报只有 3.4 ms
        ⟹ 那 50 ms 全是被抢占。绑核不解决抢占（同核上还有别的活），但它把"被调度到别的核"
        这一项消掉了，是节点侧唯一能自己做的那一半。
        """
        cpu = int(self.params.cpu)
        if cpu < 0:
            return
        try:
            os.sched_setaffinity(0, {cpu})
        except OSError as exc:      # pragma: no cover - 容器/权限受限时才会走到
            self.get_logger().warn(f"绑核到 cpu{cpu} 失败：{exc}（继续跑，但最坏耗时没有保障）")
            return
        if self.params.rt_priority > 0:
            try:
                os.sched_setscheduler(0, os.SCHED_FIFO,
                                      os.sched_param(int(self.params.rt_priority)))
                self.get_logger().info(f"SCHED_FIFO 优先级 {self.params.rt_priority} 已生效")
            except OSError as exc:
                # ⚠️ 失败只告警：要 CAP_SYS_NICE（或 `chrt`/systemd 的授权），
                #    而"没权限"不该让节点起不来 —— 它只影响最坏耗时，不影响正确性。
                self.get_logger().warn(f"SCHED_FIFO 申请失败（不影响正确性）：{exc}")

    @staticmethod
    def _cpus_allowed() -> str:
        """`/proc/self/status` 里的 `Cpus_allowed_list`（绑核到底生效没有，读它最直接）。"""
        try:
            for line in Path("/proc/self/status").read_text().splitlines():
                if line.startswith("Cpus_allowed_list:"):
                    return line.split(":", 1)[1].strip()
        except OSError:  # pragma: no cover
            pass
        return "?"

    # ─────────────────────────── 输入 ───────────────────────────

    def _on_mocap(self, msg: PoseStamped) -> None:
        self.core.on_mocap(msg, self._now_s())

    def _on_status(self, msg: VehicleStatus) -> None:
        self.core.on_status(msg, self._now_s())

    def _now_s(self) -> float:
        """节点自己的秒表（`monotonic`）—— ★ 新鲜度用它，**不用消息里的 stamp**。

        理由：PX4 的 `hrt_absolute_time()` 从**飞控上电**起算，与我们的墙钟是两个纪元
        ⟹ 拿两个纪元做减法毫无意义（假 PX4 夹具 `bench/fake_px4.py` 把这条性质照抄了过来，
        所以这条错误在台架上就会露馅）。
        """
        return time.monotonic()

    def _timestamp(self) -> int:
        """PX4 要**微秒**。与 05 的 stub 同一份口径（`time.time_ns() // 1000`）。

        ★ 为什么必须用墙钟而不是 `monotonic`：uXRCE-DDS 的 `time_offset` 是相对**墙钟**同步的，
          PX4 收到后算 `timestamp − time_offset`。发单调时钟的值会得到一个**巨大的偏移**
          （本质上与"发错单位"同类），而它的表现是**静默的**（`min()` 把它钳回 `hrt`）。
          受控实测表见 `offboard_smoke._timestamp` 的 docstring。
        """
        return time.time_ns() // 1_000

    # ─────────────────────────── 一拍 ───────────────────────────

    def _tick(self) -> None:
        self.ticks += 1
        now_s = self._now_s()
        # 一拍一个时间戳，两个话题共用（同一拍的两条消息盖同一个时刻 —— 它们描述同一瞬间的意图）
        ts = self._timestamp()
        heartbeat_due = self.ticks % self._heartbeat_every == 0

        cycle = self.core.cycle(now_s)
        self._last_cycle = cycle

        # ① 心跳：**无条件**（默认）。★★ 理由见模块 docstring —— PX4 只在收到它之后才放行
        #    OFFBOARD（`offboardCheck.cpp` + `modeCheck.cpp:126`），闸掉它就永远进不去。
        if self.params.heartbeat_always and heartbeat_due:
            self._publish_mode(ts)

        if cycle.x0 is None:
            # 闸门关着：**不求解**（没有可信状态就没有意义的解），记住原因，日志限频
            self.gated[cycle.reason] = self.gated.get(cycle.reason, 0) + 1
            if self.ticks % max(1, self.params.log_every) == 0:
                self.get_logger().info(
                    f"t={now_s:.2f}s 闸门关闭（{cycle.reason}）：status 龄 {cycle.age_status_s:.2f}s / "
                    f"mocap 龄 {cycle.age_mocap_s:.2f}s ⟹ 不发 setpoint"
                    + ("（心跳照发）" if self.params.heartbeat_always else ""))
            return

        # ★ `prediction=False`：每周期只取第一拍 `u`（省 ~1.1 ms/周期，实测，见本包 README）。
        #   它要求**显式**传 `u_ref`（缺省那条"上一帧解平移一拍"是从预测轨迹里来的）
        #   ⟹ 这里用"上一拍的 `u` 铺满时域"（零阶保持）。比 `shifted_previous` 弱一点
        #   （看不到后 N−1 拍），但它不依赖读轨迹；RViz 那条预测路径是另外取出来的（见 `_viz_tick`）。
        sol = self.ocp.solve(cycle.x0, cycle.x_ref, u_ref=self._u_ref, prediction=False)
        self.solves += 1
        self.solve_ms_max = max(self.solve_ms_max, 1e3 * sol.info.time_tot)
        self._u_ref = np.tile(np.asarray(sol.u, dtype=float), (self.ocp.config.N, 1))
        if sol.info.degraded:
            self.degraded_count += 1
            if self.degraded_count in (1, 10) or self.degraded_count % 100 == 0:
                self.get_logger().warn(
                    f"求解退化（第 {self.degraded_count} 次）：{sol.info.line()}")

        if not cycle.publish:
            # 热身拍：**求解但绝不发**（理由见 `WARMUP_CYCLES`：第 0 拍的解是出厂迭代点）
            self.gated["warmup"] = self.gated.get("warmup", 0) + 1
            return

        try:
            sp = sign_map.px4_rates_setpoint(sol.u[1:], sol.u[0], self.z0.t_max)
        except ValueError as exc:
            # 非有限值 / 负推力：**这一拍不发**（上一拍的值留在 PX4 侧，由它自己的
            # `COM_OF_LOSS_T` 兜底）。兜底策略的完整设计归 ticket 07。
            self.rejected += 1
            self.get_logger().warn(f"这一拍拒绝下发：{exc}")
            return

        if not self.params.heartbeat_always and heartbeat_due:
            # 另一档：心跳与 setpoint 同生死（`heartbeat_always=False`）
            self._publish_mode(ts)
        self._publish_rates(ts, sp)

    def _publish_mode(self, ts: int) -> None:
        """offboard 心跳（`body_rate = True`，其余 False）。**抽帧发** ⟹ 默认 50 Hz。"""
        msg = OffboardControlMode()
        msg.timestamp = ts
        msg.position = False
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = True
        msg.actuator = False
        self.pub_mode.publish(msg)
        self.heartbeats += 1

    def _publish_rates(self, ts: int, sp) -> None:
        msg = VehicleRatesSetpoint()
        msg.timestamp = ts
        msg.roll = float(sp.roll)
        msg.pitch = float(sp.pitch)
        msg.yaw = float(sp.yaw)
        msg.thrust_body[0] = 0.0
        msg.thrust_body[1] = 0.0
        msg.thrust_body[2] = float(sp.thrust_body_z)
        self.pub_rates.publish(msg)

        now_ns = time.time_ns()
        self.published += 1
        self.first_publish_ns = self.first_publish_ns or now_ns
        self.last_publish_ns = now_ns
        self._log_frame(now_ns, ts, sp)

        if self.ticks % max(1, self.params.log_every) == 0:
            self.get_logger().info(
                "t=%.2fs 第 %d 帧 | roll=%+.3f pitch=%+.3f yaw=%+.3f thrust_body[2]=%+.4f"
                " | 解 %d 次、闸门 %s、退化 %d"
                % (self._now_s(), self.published, msg.roll, msg.pitch, msg.yaw,
                   msg.thrust_body[2], self.ticks,
                   ",".join(f"{k}×{v}" for k, v in sorted(self.gated.items())) or "无",
                   self.degraded_count))

    def _log_frame(self, now_ns: int, ts: int, sp) -> None:
        """逐帧 JSONL（取证用）。⚠️ 每个字段都要显式 `float()` / `int()`：

        `thrust_body[2]` 是 ROS 的 **float32**、`nav_state` 是 **uint8**，`json.dumps` 两个都不认
        —— 而定时器回调里抛异常 = **整个 spin 挂掉**（症状是"节点起来一秒就没了、日志文件是空的"）。
        这条是 05 用一次真事故换来的（`offboard_smoke._tick` 里的同一段）。
        """
        if self._log is None:
            return
        cycle = self._last_cycle
        self._log.write(json.dumps({
            "wall_ns": now_ns, "ts": int(ts), "tick": int(self.ticks),
            "roll": float(sp.roll), "pitch": float(sp.pitch), "yaw": float(sp.yaw),
            "thrust_body_z": float(sp.thrust_body_z),
            "age_status_s": round(float(cycle.age_status_s), 4),
            "age_mocap_s": round(float(cycle.age_mocap_s), 4),
            "cycles_since_gate": int(cycle.cycles_since_gate),
            "nav_state": int(self.core.status.nav_state),
            "arming_state": int(self.core.status.arming_state),
            "status_cb": int(self.core.status_frames),
            "mocap_cb": int(self.core.mocap_frames),
        }) + "\n")
        self._log.flush()

    # ─────────────────────────── RViz ───────────────────────────

    def _viz_tick(self) -> None:
        """把**求解器里那份当前迭代点**的预测轨迹画出来（100 ms 一张）+ 参考点。

        ★ 这里**不额外解一次**：`get_flat` 实测 0.049 ms，而一次 `solve()` 是 3.5 ms 的量级
          （见本包 README 的耗时分解）。读到的轨迹与刚发出去的 `u` **同源**（同一个迭代点），
          所以 RViz 里看到的就是"节点此刻认为会发生什么"——它是否在发疯，一眼能看出来。
        """
        pred = self.ocp.last_prediction()
        if pred is None:
            return
        x_pred, _ = pred
        if x_pred.shape[0] != self.ocp.config.N + 1:
            return
        stamp = self.get_clock().now().to_msg()

        path = NavPath()
        path.header.stamp = stamp
        path.header.frame_id = self.params.rviz_frame
        for row in x_pred:
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = float(row[0])
            pose.pose.position.y = float(row[1])
            pose.pose.position.z = float(row[2])
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)
        self.pub_path.publish(path)

        ref = self._last_ref
        if ref is not None:
            marker = Marker()
            marker.header.stamp = stamp
            marker.header.frame_id = self.params.rviz_frame
            marker.ns = "mpc_reference"
            marker.id = 0
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = float(ref[0])
            marker.pose.position.y = float(ref[1])
            marker.pose.position.z = float(ref[2])
            marker.pose.orientation.w = 1.0
            marker.scale.x = marker.scale.y = marker.scale.z = 0.15
            marker.color.a = 1.0
            marker.color.r = 1.0
            marker.color.g = 0.2
            marker.color.b = 0.2
            self.pub_ref.publish(marker)

    # ─────────────────────────── 收尾 ───────────────────────────

    def summary(self) -> str:
        span = ((self.last_publish_ns - self.first_publish_ns) / 1e9
                if self.first_publish_ns else 0.0)
        rate = (self.published - 1) / span if span > 0 else float("nan")
        return ("共发 %d 帧 setpoint（心跳 %d 个，%.0f Hz），跨 %.2fs ⟹ 实际 %.1f Hz；"
                "定时器 %d 拍、求解 %d 次、最坏 time_tot %.3f ms、退化 %d 拍、拒绝 %d 拍；"
                "闸门：%s"
                % (self.published, self.heartbeats, self.heartbeats / span if span > 0 else float("nan"),
                   span, rate, self.ticks, self.solves,
                   self.solve_ms_max, self.degraded_count, self.rejected,
                   ", ".join(f"{k}×{v}" for k, v in sorted(self.gated.items())) or "无"))

    def close(self) -> None:
        if self._log is not None:
            self._log.close()
            self._log = None

    @property
    def _last_ref(self):
        cycle = self._last_cycle
        return None if cycle is None or cycle.x_ref is None else cycle.x_ref[0][:3]


# ─────────────────────────── 参数 ───────────────────────────

#: `(参数名, 默认值, 类型)`。★ 默认值全部与 `NodeParams` 的字段一致 —— 两处一起改。
_PARAMS = (
    ("rate_hz", 100.0, float),
    ("heartbeat_hz", 50.0, float),
    ("viz_hz", 10.0, float),
    ("heartbeat_always", True, bool),
    ("mass", 0.75, float),
    ("hover_thrust", 0.6235, float),
    ("tau_s", 0.05, float),
    ("hold_offset_x", 0.0, float),
    ("hold_offset_y", 0.0, float),
    ("hold_offset_z", 0.0, float),
    ("keep_yaw", True, bool),
    ("hold_yaw_deg", 0.0, float),
    ("mocap_timeout_s", 0.5, float),
    ("status_timeout_s", 0.75, float),
    ("stop_on_nav_state_exit", True, bool),
    ("stop_on_armed_drop", True, bool),
    ("warmup_cycles", 5, int),
    ("cpu", -1, int),
    ("rt_priority", 0, int),
    ("rviz_frame", "world", str),
    ("log_file", "", str),
    ("log_every", 100, int),
)


def declare_params(node: Node) -> NodeParams:
    """声明并读取全部参数；**拼错名字当场退出**（rclpy 自己不会报，见 `NodeParams`）。"""
    _reject_unknown_params(node)
    for name, default, _type in _PARAMS:
        node.declare_parameter(name, default)
    get = node.get_parameter
    p = NodeParams(
        rate_hz=get("rate_hz").value,
        heartbeat_hz=get("heartbeat_hz").value,
        viz_hz=get("viz_hz").value,
        heartbeat_always=get("heartbeat_always").value,
        mass=get("mass").value,
        hover_thrust=get("hover_thrust").value,
        tau_s=get("tau_s").value,
        hold_offset=(get("hold_offset_x").value, get("hold_offset_y").value,
                     get("hold_offset_z").value),
        keep_yaw=get("keep_yaw").value,
        hold_yaw_deg=get("hold_yaw_deg").value,
        mocap_timeout_s=get("mocap_timeout_s").value,
        status_timeout_s=get("status_timeout_s").value,
        stop_on_nav_state_exit=get("stop_on_nav_state_exit").value,
        stop_on_armed_drop=get("stop_on_armed_drop").value,
        warmup_cycles=int(get("warmup_cycles").value),
        cpu=int(get("cpu").value),
        rt_priority=int(get("rt_priority").value),
        rviz_frame=get("rviz_frame").value,
        log_file=get("log_file").value,
        log_every=int(get("log_every").value),
    )
    _validate(p)
    return p


#: ROS 自带、**由 rclpy 声明**的参数：它们出现在 `-p` 里是常态，不是拼错的我们的参数。
#: `use_sim_time` 是 ROS 2 的标准参数（rclpy 的 `Node` 自己声明并读它）；
#: `start_type_description_topic` 是 Iron 之后 rclpy 自动声明的同一个性质的东西（这里先放行，
#: 因为"多放行一个 ROS 自带的名字"的代价是零，而"误拒一个"的代价是节点起不来）。
#: 另外 `qos_overrides.*` 前缀的覆盖在 `_reject_unknown_params` 里单独按前缀放行 ——
#: 它们来自 `ros2 param dump` 之类生成的 params 文件，名字里带话题路径，没法穷举。
STD_ROS_PARAMS = frozenset({"use_sim_time", "start_type_description_topic"})


def _reject_unknown_params(node: Node) -> None:
    """`--ros-args -p 名字:=值` 里的名字**既不在 `_PARAMS`、也不是 ROS 自带的** ⟹ **启动即退**。

    ★ rclpy 对"多写一个参数"是完全沉默的（它静静地待着，谁也没读）⟹ 台架上表现为
      "我把参数改了，但行为没变"，而所有的断言会指向被测逻辑。rclcpp 那边是另一种表现
      （参数名错会直接退进程），**两条路都要能用同一句话读懂**，所以这里主动补齐。

    ⚠️ **只拒"我们自己的参数名拼错了"，不拒 ROS 自带的参数** —— 这两件事必须分开：
      `use_sim_time` 由 rclpy 自己声明并读取（不是我们的），而 launch / params 文件里带上它
      是**常态**。第一手（2026-09-28，本机）：`ros2 run nmpc mpc_node.py --ros-args -p
      use_sim_time:=true` 曾经**启动即退、exit 2**（`未知参数：['use_sim_time']`）——
      一个"多写一个参数不会报错"的补丁，把标准参数变成了"节点起不来"。ticket 09/11 要用
      launch 起这个节点，这条不修就是**起不来**，而且报错说的是"未知参数"，指向的是参数名。
    """
    known = {name for name, _default, _type in _PARAMS} | STD_ROS_PARAMS
    # ⚠️ rclpy **没有公开的**"列出所有 `-p` 名字"的 API：`Node` 只在构造时把命令行给的
    #    覆盖存进 `_parameter_overrides`（`rclpy/node.py:203`）。所以这里读那个内部属性，
    #    并且**读不到就放行**（自己 `warn` 一句）—— 版本变化时降级成"不检查"，
    #    绝不让一句参数检查把节点拦在启动门外（那会把"没参数检查"升级成"节点起不来"）。
    overrides = getattr(node, "_parameter_overrides", None)
    if overrides is None:  # pragma: no cover - 只有 rclpy 换了实现才会走到
        node.get_logger().warn("拿不到参数覆盖表 ⟹ 跳过「未知参数」检查（见 mpc_node._reject_unknown_params）")
        return
    unknown = sorted(n for n in set(overrides) - known
                     if not n.startswith("qos_overrides"))
    if unknown:
        raise SystemExit(
            f"未知参数：{unknown}。已知参数：{sorted(known - STD_ROS_PARAMS)}"
            "（rclpy 不会因为多写一个参数而报错，所以这里主动拒 —— 否则它会静默地不生效）")


def _validate(p: NodeParams) -> None:
    """把"启动参数会炸"与"启动参数没意义"分开 —— 前者**现在**就拒，带一句话原因。

    与 `offboard_smoke.validate_args` 同一条纪律：`rate_hz=0` 会在 `1.0 / 0` 那行抛，
    而那时**节点已经起来了**（DDS 里也已经有话题了）⟹ 台架会把它读成"节点没发数据"。
    """
    for name in ("rate_hz", "heartbeat_hz", "viz_hz", "mass", "hover_thrust", "tau_s",
                 "mocap_timeout_s", "status_timeout_s"):
        val = getattr(p, name)
        if not math.isfinite(val) or val <= 0:
            raise SystemExit(f"参数 {name} 要是正的有限数，收到 {val!r}")
    if p.heartbeat_hz > p.rate_hz:
        raise SystemExit(
            f"heartbeat_hz({p.heartbeat_hz}) 不能大于 rate_hz({p.rate_hz})："
            "心跳是从求解节拍上抽帧发的（见 `_heartbeat_every`），比它快没有意义。")
    if p.tau_s > 0.2:
        raise SystemExit(f"tau_s={p.tau_s} 太大（这是执行延迟的量级，默认 0.05 s）")
    for name in ("warmup_cycles", "log_every"):
        if int(getattr(p, name)) < 0:
            raise SystemExit(f"参数 {name} 不能为负，收到 {getattr(p, name)!r}")
    if not all(math.isfinite(v) for v in p.hold_offset):
        raise SystemExit(f"参考偏移必须是有限数，收到 {p.hold_offset!r}")


def main(argv=None) -> int:
    rclpy.init(args=list(sys.argv[1:] if argv is None else argv))
    node = None
    try:
        # 参数先在**临时节点**上读：`MpcNode.__init__` 一上来就要用它们（绑核、求解器）。
        # ⚠️ 读完就毁掉它 —— 否则 `ros2 node list` 里会多一个谁也不是的节点，
        #    而台架上"多出来的那个节点"会被读成"有人起重复了"。
        probe = rclpy.create_node(f"{NODE_NAME}_params")
        try:
            params = declare_params(probe)
        finally:
            probe.destroy_node()
        node = MpcNode(params)
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        # ★ rclpy 自己的 SIGINT 处理器走的是这条路（不是 `KeyboardInterrupt`）：
        #   它先把 context 关掉，再让 `spin()` 抛这个异常。没接住就会在 Ctrl-C 时
        #   打一整页 traceback —— 而"收尾摘要"恰恰是 Ctrl-C 之后最想看到的那一行。
        pass
    except SystemExit as exc:
        # `_validate` / `_reject_unknown_params` 用 `SystemExit(消息)` 拒参数 ⟹
        # `exc.code` 是**字符串**（不是 0，也不是数字）—— 别去 `int()` 它。
        if exc.code not in (0, None):
            print(f"[mpc_node] 启动失败：{exc.code}", file=sys.stderr, flush=True)
            return 2
    finally:
        if node is not None:
            # ★ 收尾摘要走 `print` 而不是 `get_logger()`：SIGINT 那条路上 rclpy 已经把
            #   context 关掉了，此时发日志会打出 "Failed to publish log message to rosout"
            #   并且**摘要本身看不到** —— 而摘要恰恰是 Ctrl-C 之后最想读的一行。
            #   （台架也直接读 stdout 这一行，不必去剥 ROS 的日志前缀。）
            print(f"[mpc_node] 收尾：{node.summary()}", flush=True)
            node.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
