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
| 订阅 | `/mocap/pose_enu`（**ENU/FLU**，B 档；`state_source=mocap` 时才有这一条） | 可靠 / depth 10 | `mocap_rel_pose_node.cpp:110` `QoS(10)` |
| 订阅 | `/mocap/velocity_enu`（`geometry_msgs/TwistStamped`，**ENU**，B 档；`state_source=mocap` 时才有） | 可靠 / depth 10 | `mocap_rel_pose_node.cpp` 的 `QoS(10)`；★ 速度来源（ticket 15）—— `PoseStamped` 没有 `twist` 字段，`v` 必须走这条独立话题才能进 `x + v·τ` |
| 订阅 | `/fmu/out/vehicle_local_position` + `/fmu/out/vehicle_attitude`（**NED/FRD**，A 档；`state_source=px4` 时才有这两条） | **BEST_EFFORT** / depth 1 | 发布方实测 best_effort + TRANSIENT_LOCAL（2026-09-28，z0 SITL，A 档探针）⟹ 订阅侧写 RELIABLE 就**一个回调都不进** |
| 订阅 | `/fmu/out/vehicle_status` | **BEST_EFFORT** / depth 1 | PX4 的 writer 是 best_effort（`utilities.hpp:59-63`）⟹ 订阅侧写 RELIABLE 就**一个回调都不进** |
| 订阅 | `/traj/reference`（`mavros_msgs/PositionTarget`；`reference_source=traj` 时才有这一条） | 可靠 / depth 10 | `traj_node.cpp:108-109` 的 `rclcpp::QoS(10)`（默认 reliable + volatile） |
| 发布 | `/fmu/in/vehicle_rates_setpoint` | BEST_EFFORT + TRANSIENT_LOCAL / depth 1 | PX4 的 reader 是 best_effort（`utilities.hpp:114-119`）；TRANSIENT_LOCAL 让晚起的 reader 也能拿到最后一帧 |
| 发布 | `/fmu/in/offboard_control_mode`（`body_rate = True`，其余 False） | 同上 | 同上 |
| 发布 | `/mpc/predicted_path`、`/mpc/reference`（RViz 用，`nav_msgs/Path` / `Marker`） | 默认（可靠） | 本仓自己的观测面，不接飞控 |

★★ **两档状态源**（`-p state_source:=mocap|px4`，ticket 09）：两档**只差"测量从哪里来"**，
**控制器那一条路**（闸门、外推、参考、求解、映射、发布）完全共用。
⚠️ 说"分岔**只有一处**"是把话说满了（2026-09-28 复核：`state_source` 的条件分支全仓实有
9 处）—— 分岔的**内容**只有一处，但"这一档要订阅什么、要记什么"必然跟着分：
① **测量本身**（`_measure_mocap` / `_measure_px4`，见 `node_core` 的 ★★ 表）；
② **订阅**（`mocap` 建 `/mocap/pose_enu` + `/mocap/velocity_enu`，`px4` 建那两条 —— 且 QoS 不同）；
③ **日志的 `state_source` / `age_*` 键**（谁给的、多旧，这是 A 档复盘的入口）；
④ **参数校验**（`state_source` 不认识就启动即退；`px4` 档必须给话题名）。两档如下：

| 档 | 测量 | 话题（**参数可换**） | 用在 |
|---|---|---|---|
| `mocap`（默认） | 动捕，**原生 ENU/FLU** | `/mocap/pose_enu`（上游定死，`mocap_rel_pose_node.cpp:110`） | B 档（动捕闭环）—— 默认档，不给参数就**不改变任何现有行为** |
| `px4` | 飞控自己的估计，**NED/FRD** | `-p px4_position_topic` / `px4_attitude_topic` | A 档（ticket 09）—— 闭环第一次**只用了飞控自己给的东西** |

★ 为什么 A 档**不订阅**动捕（而不是"订阅了但不看"）：票面验收项写着"本档**不使用动捕输入**"，
  而"订阅了但不看"在话题边界上**不可证伪**（DDS 里那条订阅明明白白地在那儿）。
  条件订阅让这条验收项变成一个**黑盒可断言的事实**：
  `state_source=px4` 时 `/mocap/pose_enu` 的 `matched_subscribers() == 0`。

★★ **两档参考源**（`-p reference_source:=hold|traj`，ticket 17）：**与 `state_source` 正交**
（两个 `if`，不是一个 `else`），两档**只差"目标点从哪里来"**：

| 档 | 目标点 | 话题 | 用在 |
|---|---|---|---|
| `hold`（默认） | 锚点 + 参数化 `hold_offset`，速度 0、水平姿态 | 无（参数） | 悬停（ticket 09/20），**默认档 = 不改变任何现有行为** |
| `traj` | **锚点 + `/traj/reference` 给的相对量**，速度由参考自己给 | `/traj/reference`（`mavros_msgs/PositionTarget`，与 Phase 1 同话题同类型 ⟹ `traj` 节点零改动） | 飞轨迹（ticket 11 的判据②） |

★ 为什么复用 Phase 1 那个类型、那个话题名（ticket 17 的裁定，第一手）：
  ① 仓里的回放包（bags 3/4）**已经**把 `/traj/reference` 记成 `mavros_msgs/msg/PositionTarget`
     ⟹ 换类型就要为回放写一层转换，而那层转换是**没有真值可对**的那种代码；
  ② `mavros_msgs` 是**消息定义包**（`CMakeLists.txt` 只调 `rosidl_generate_interfaces`，
     全文件没有 `find_package(mavros)` / `libmavros` / 插件）⟹
     "把 mavros 拉进 Phase 2 的环里"这个担心不成立。
     ⚠️ **一处要说准**（2026-09-30 第一手）：它的 `package.xml:34` 有一条
     `<build_export_depend>libmavconn</build_export_depend>`，而它自己的构建**从不使用**
     （全仓没有 `find_package(libmavconn)`；本机 Humble 里 `/opt/ros/humble/share/libmavconn`
     根本不存在）⟹ 那是一条**悬空声明**，作用只到"下游在**构建依赖图**里排在它后面"，
     且 `libmavconn` 与 `mavros` 节点不是同一个包；
  ③ `traj` 节点一个字都不用改 ⟹ 轨迹的形状（三条几何、`traj_origin` 语义）与 Phase 1 完全同一份。

★ **轨迹原点取在锚点上**（= `reference_abs = 锚点 + p_rel`）：这与 Phase 1 的裁定同构 ——
  Phase 1 的 `traj_origin` 就是 `control` 在 HOVER 判稳那一刻记的**自己当前位置**
  （`phase1-implementation.md` §3.3 第一手表），**不是**动捕原点；理由是"偏移被自动吸收：
  在同一个坐标系里以当前点为 0 发指令"。Phase 2 的锚点（闸门第一次打开那一拍的测量）
  是同一件事 —— **加原点这件事发生在下游这一侧**，`traj` 只知道相对量。
  ⚠️ 由此带来的**已知限制**：`p_rel` 给的是形状，绝对落点由"闸门打开时飞机在哪"决定
  ⟹ 与"按动捕原点摆放的绝对轨迹"差一个常值平移。起飞点摆得离轨迹起点越近，这个平移越小。
  **这一点留给 ticket 11 的实飞去确认或推翻**，本票只把口径写死。

★ 为什么 `traj` 档**拒绝**非 0 的 `hold_offset`（`_validate`）：那时目标点已经是"锚点 + 相对量"，
  再叠一个平移既不是轨迹的一部分、也不是锚点的一部分，而症状是"整条轨迹偏了一截"
  —— 看着像轨迹算错了。切档时忘掉 `hold_offset_z` 是最自然的操作失误，所以让它**启动即退**。

⚠️ 话题名参数化（而不是写死）的理由 —— A 档真正要回答的是"**我们的约定与真 PX4 一致**"，
  而这个问题的证据链只有一条：拿一条**已知真值**的 PX4 状态流喂进去，看指令方向对不对。
  `vehicle_local_position_groundtruth` 是**同一个消息类型**（`VehicleLocalPosition`，见 msg 尾部的
  `TOPICS` 行），所以"换成真值那条流"是**改一个参数**、不是改代码。
  ⚠️ 但它默认**不在 DDS 上**：`dds_topics.yaml` → `dds_topics.h` 是**编译期**生成的
  （ticket 08 第一手），要暴露它得重编一份 —— 所以这条参数化的另一个用处是：
  那个决定可以**推迟到拿到读数之后**再做，而不必先把代码写死。

**发不发**（两张表合成一条规则）：`offboard_control_mode` = **节点活着就发**；
`vehicle_rates_setpoint` = **只有** armed + OFFBOARD + 状态新鲜（闸门在 `node_core.cycle()`）。
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
  —— 那个结构体订阅不到新消息时**原样留着**上一帧的值（`MulticopterRateControl.cpp:177-184`：
  三个角速率分量还有 `PX4_ISFINITE` 回退，**推力没有**）。
  ★ 这条性质就是下面 ★★★ 那段「兜底阶梯」的**全部理由**。
  `-p heartbeat_always:=false` 是**另一档**（把心跳一起闸掉 = 交给 PX4 自己的失效保护，
  它在 `COM_OF_LOSS_T` 之后自己退出 OFFBOARD）；**默认 True**，因为"进不去 OFFBOARD"
  是当场就飞不了的错。

★★★ **兜底阶梯（ticket 07 / M5）**：`求解不可用 → 保持上一帧可行指令 → 安全悬停`。
为什么是这样一个阶梯，而不是"一路保持"或"一失败就悬停"（完整推导见
[`docs/control/mpc-node-fallback.md`](../../../docs/control/mpc-node-fallback.md)）：

- **不发 ≠ 松手**（上一条）：PX4 会把最后一帧**无限期锁存**下去 ⟹ 一直不发的净效果是
  **让飞机继续执行一条陈旧的命令**（可能正打着满角速率）。所以"不可用"必须由我们
  **主动换成**一条安全的帧，不能靠沉默。
- 一拍的解刚不可用时，上一条命令还是**可行**的（几毫秒前算出来的）⟹ 先 `hold`
  （**内容原样、时间戳换新**，于是"在保持"这件事在话题上可观测、可断言）；
  连续失败超过 `fallback_hold_frames`（默认 10 拍 = 100 ms ≈ 28 个求解预算）才切 `hover`
  —— **悬停推力 + 零角速率**，★ 它**不依赖任何状态**，所以在"状态不可信"时仍然可用。
- 进入 `hover` 那一次**置标志并上报**（`get_logger().error` + JSONL 的 `fallback` 字段 +
  收尾摘要）⟹ 这个事件在飞行日志里**可检索**；"持续了多少拍"由累计量回答。

★ **故障注入**（`-p inject_solve_fail_after_frames:=N`，默认 0 = 关）：已发布 ≥ N 帧之后，
  每一拍的求解结果都被换成"状态码非零 + `u` 全 NaN + 耗时 NaN"的坏解。它存在的**唯一**理由
  是**夹具造不出来**：话题面上只看得到帧，而"求解失败"发生在**这个进程内部**；
  而 SITL 上"注入故障之后飞机不撒手"那条判据（ticket 11）只能靠它。
  ⟹ 启动时显著告警，且 `preflight_check` 把它列为**装桨前必须关**的项。

★ **状态外推** `x + v·τ`（τ = 50 ms）与**首拍不发**（`WARMUP_CYCLES`）都在 `node_core` 里，
理由见那边的 docstring（后者是第一手的：`SQP_RTI` 的第 0 次 `solve()` 只是"从出厂迭代点走一步"）。

★ **饱和表现为"打满"**：本节点**不做任何裁剪** —— 唯一的限幅点是 OCP 自己的
`lbu/ubu`（`F ∈ [0, f_max]`、`ω ∈ ±rate_max`）。所以"它在顶约束"与"它在正常工作"在话题上分得开：顶约束时 `ω` 恰好停在 `rate_max` 上，而不是被谁悄悄按比例缩回去。
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
from dataclasses import dataclass, replace
from pathlib import Path
from typing import Optional, Tuple

if __package__ in (None, ""):  # 直接 `python3 …/mpc_node.py` 时把包目录挂上
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import numpy as np  # noqa: E402
import rclpy  # noqa: E402
from geometry_msgs.msg import PoseStamped, TwistStamped  # noqa: E402
# ⚠️ **必须改名**：`nav_msgs/Path` 与 `pathlib.Path` 同名，直接 import 会把上面那个
#    悄悄换掉 —— 症状是"读 `/proc/self/status` 时 `Path.__init__() takes 1 positional argument`"，
#    即一个与可视化毫无关系的报错（本文件第一次跑就是这么炸的）。
# ★ `/traj/reference` 的类型**复用 Phase 1 的**（ticket 17 的裁定）：`traj` 节点一个字都不用改。
#   ⚠️ `mavros_msgs` 是**消息定义包**（`CMakeLists.txt` 只调 `rosidl_generate_interfaces`），
#   不把 mavros 那半边（mavros 节点、插件、`libmavros`）拉进 Phase 2 的环里 ——
#   Phase 2 走 uXRCE-DDS，与 mavros 是两条并行的下游，互不依赖。
#   （它 `package.xml` 里那条 `<build_export_depend>libmavconn</build_export_depend>` 是**悬空**的，
#   详见模块 docstring 的 ★★ 第 ② 条。）
from mavros_msgs.msg import PositionTarget  # noqa: E402
from nav_msgs.msg import Path as NavPath  # noqa: E402
from rclpy.executors import ExternalShutdownException  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,  # noqa: E402
                       ReliabilityPolicy)
from visualization_msgs.msg import Marker  # noqa: E402

from px4_msgs.msg import (OffboardControlMode, VehicleAttitude,  # noqa: E402
                          VehicleLocalPosition, VehicleRatesSetpoint,
                          VehicleStatus)

from nmpc import NU, sign_map  # noqa: E402
from nmpc.node_core import (FALLBACK_HOLD_FRAMES, REFERENCE_SOURCE_TRAJ,  # noqa: E402
                            REFERENCE_SOURCES, STATE_SOURCE_PX4, STATE_SOURCES,
                            TAU_MAX_S, TAU_MIN_S,
                            ControlConfig, FallbackLadder, NodeCore, StopConfig,
                            frame_is_finite, hold_feedforward, saturated_bounds,
                            solution_usable)
from nmpc.ocp import OfflineOcp  # noqa: E402
from nmpc.params import Z0Params  # noqa: E402

#: PX4 的 `/fmu/out/*` 是 **BEST_EFFORT**（`utilities.hpp:59-63`）⟹ 订阅侧写 RELIABLE
#: （ROS 2 的默认值）就**一个回调都不进**，而话题照样列得出来。
#: ★ A 档那两条状态话题（ticket 09）实测也是 best_effort + **TRANSIENT_LOCAL**（发布方
#: reliability=2 / durability=1，2026-09-28 z0 SITL 探针）：durability 只要**订阅方不强于发布方**
#: 就兼容，所以本 profile（VOLATILE）直接可用 —— 这一条是**量出来的**，不是推的。
QOS_SUB_STATUS = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                            reliability=ReliabilityPolicy.BEST_EFFORT)
#: 发布侧：PX4 的 reader 是 best_effort（`utilities.hpp:114-119`）+ TRANSIENT_LOCAL
#: 让晚起的 reader 也能拿到最后一帧。这与 05 的 stub 是同一份口径（那边在 SITL 上验过）。
QOS_PUB_FMU = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                         reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
#: `/mocap/pose_enu` 是 `mocap` 节点用 `QoS(10)`（**可靠**）发的
#: （`mocap_rel_pose_node.cpp:110`，与 `bench/vrpn_source.py` 的 `mocap_qos()` 一致）。
QOS_MOCAP = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=10,
                       reliability=ReliabilityPolicy.RELIABLE)
#: `/traj/reference` 的发布方是 `traj` 节点，用 `rclcpp::QoS(10)`
#: （`traj_node.cpp:108-109`：默认 reliable + volatile、depth 10）⟹ 与 `QOS_MOCAP` **同形**。
#: ⚠️ 仍然**单独一个名字**：两条话题的发布方毫无关系，共用一个名字意味着哪天
#: `mocap` 那条的 QoS 一改会顺带改到轨迹这条上，而这件事在话题面上看不出来。
QOS_TRAJ_REF = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=10,
                          reliability=ReliabilityPolicy.RELIABLE)

TOPIC_MOCAP = "/mocap/pose_enu"
#: 动捕**速度**（`geometry_msgs/TwistStamped`，ENU，与 `/mocap/pose_enu` 同一个 frame）——
#: `mocap` 节点把 `/vrpn/<刚体>/twist` 原样 republish 出来（ticket 15）。与 `TOPIC_MOCAP` 一样
#: **写死**（不参数化）：它是 `mocap` 档**定死**的上游，而 A 档那两条才是"可以换一条流"的参数。
TOPIC_MOCAP_VEL = "/mocap/velocity_enu"
#: **轨迹参考**（ticket 17）：`mavros_msgs/PositionTarget`，与 Phase 1 的 `traj` 节点同类型同话题名
#: ⟹ `traj` 侧零改动。与 `TOPIC_MOCAP` 一样**写死**（不参数化）：它是"轨迹"这件事的上游，
#: 而话题名换了就等于换了一套几何 —— 那种改动该改代码，不该改启动参数（同 `TOPIC_MOCAP` 的理）。
#: 只有 `reference_source=traj` 时才订阅（同 `state_source` 的条件订阅纪律）。
TOPIC_TRAJ_REF = "/traj/reference"
TOPIC_STATUS = "/fmu/out/vehicle_status"
#: A 档的状态话题（`state_source=px4`）。★ 默认值是**飞控自己的估计**（票面原文点名要的这两条），
#: 而它们是可以换的 —— 换话题（真值 / 记录回放）是参数的事，见模块 docstring 的表。
TOPIC_PX4_POS = "/fmu/out/vehicle_local_position"
TOPIC_PX4_ATT = "/fmu/out/vehicle_attitude"
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
    #: ★ 实测值（满电口径，2026-09-29 ticket 10 回填）—— 与 `params.HOVER_THRUST` **必须同值**，
    #: 漂了不报错：节点按下面 `_PARAMS` 表飞、测试按这里断言 ⟹ 测试全绿而飞机是错的。
    hover_thrust: float = 0.1589
    #: 状态外推 τ [s]。**默认 0.05**（= z0 上实测的 ≈50 ms，`mocap-setup.md` §6.3），
    #: 而**首飞按冻结值取 `0`**（= 不补偿）—— 加档的时机归 ticket `phase4/10`。
    #: ★ 合法区间是 `[TAU_MIN_S, TAU_MAX_S)` = `[0, 0.2)`（**0 合法**，见 `_validate` 里那段 ★★；
    #: 负 τ 仍然拒）。这两个端点是 `node_core` 里的常量，与 `preflight_check` 同一份。
    tau_s: float = 0.05
    #: ★★ **垂直通道外层前馈偏置的积分增益** [N/(m·s)]（ticket 20）：`ΔF ← ΔF + ki_z·e_z·Δt`，
    #: 最终加在**发出去的那条指令**上（`u_cmd[0] = clip(u[0] + ΔF, 0, f_max)`，见 `_normal_frame`）。
    #: `≤0` = **硬关掉**（退回"纯前馈 + OCP 反馈"的旧行为，
    #: 且把已经攒下的 `ΔF` 一起清零 ⟹ A/B 里"关掉"那一组不会带着前一组的学习量起飞）。
    #: ☠️ **落点原先选在 `u_ref`，已被实测证伪**（2026-09-29，ticket 20 §10）：`u_ref` 是**代价参考**
    #: 而不是指令，实测 `d u[0]/d u_ref[0] = −0.0616 N/N` —— **符号是反的** ⟹ 把位置误差积进 `u_ref`
    #: 是一条正反馈（越沉 ⟹ ΔF 越大 ⟹ 推力越小）。那条路的可行域是**空集**，调增益救不回来：
    #: 飞出来 `mass+30` 从 −0.6406 m **恶化到** −0.7200 m，且全程只有一个夹取上限在拦着它。
    #: 量法见 `tools/px4-sitl/probe_u_ref_authority.py`（钉住一拍的 `x0`、只动那一条通路）。
    #: ★ 换成"加在**指令**上"之后环路才闭合，而这个落点**有**可信的解析式：
    #: 被控对象 `m_p ë + D ė + G e = δ − ΔF`（`D`/`G` 是 OCP 环路自己的阻尼与刚度，离线实测
    #: `G = 3.45029 N/m`、`D = 2.65955 N/(m·s)`）+ `dΔF/dt = ki_z·e`
    #: ⟹ `m_p s³ + D s² + G s + ki_z = 0` ⟹ Routh 给出 **`ki_z < D·G/m_p`**。
    #: `mass+30`（`m_p = 0.975 kg`）⟹ 上界 **9.41**；离线闭环复算量到的起振点 `ki ≈ 9`（对上）。
    #: 下界由"判据① 的 8 s 窗口内积够 2.207 N"给出（≈0.9）。
    #: **1.0 落在 [0.9, 9.41] 正中间偏下**，离线闭环复算（`tools/px4-sitl/sim_vertical_bias_loop.py`）
    #: 读数：均值 −0.0000 m、峰峰值 0.0000 m、`ΔF` 稳态 **+2.2073 N**（对上 §8.4 预测的 +2.207）。
    #: ⚠️ 上界同时**随被控对象质量变**（`m_p` 在分母）⟹ 更重的机型上界更低，改机型要重算。
    #: 依据与推导（含被证伪的那一版）见 ticket 20 §8.3 / §10.7。
    ki_z: float = 1.0
    #: ★ **测量从哪里来**（ticket 09）：`mocap`（默认，B 档）或 `px4`（A 档）。
    #: 两档的差别**只在**"测量"这一段，见模块 docstring 的表；校验在 `_validate`。
    state_source: str = "mocap"
    #: A 档：PX4 状态话题（默认是飞控自己的估计；换真值/回放只是改这两个参数）
    px4_position_topic: str = TOPIC_PX4_POS
    px4_attitude_topic: str = TOPIC_PX4_ATT
    #: ★★ **参考从哪里来**（ticket 17）：`hold`（默认）= 参数化的悬停锚点 + `hold_offset`；
    #: `traj` = `/traj/reference`（`mavros_msgs/PositionTarget`，与 Phase 1 同一个话题）。
    #: ☠️ **与 `state_source` 正交**：两档各自独立选（A 档也能飞轨迹、动捕档也能只悬停），
    #:   所以这里**不合并**成一个"档位"参数 —— 那会造出两个只在某几种组合下成立的档名。
    #: ⚠️ `traj` 档下 `hold_offset` 必须是 0：那时的目标点是"锚点 + 轨迹给的相对量"，
    #:   再叠一个偏移就是一个**没人能说清它意味着什么**的平移（校验见 `_validate`）。
    reference_source: str = "hold"
    #: `traj` 档：参考**断流**判据 [s] ⟹ **冻结**（位置保持、速度置 0），**不是**停发。
    #: ★ 依据是 Phase 1 的裁定"缺一帧参考就保持上一个目标，不回退到原点"
    #:   （`phase1-implementation.md` §3.3）；停发的后果比丢帧激烈得多（见 `node_core` 的闸门）。
    #: ★★ **名字抄 Phase 1 的**（`src/control/README.md:119`）：它那一级就叫 `traj_stream_gap_s`
    #:   （0.5 s ⟹ 保持上一个目标），而 `traj_timeout_s` 在那边是**另一级**（3.0 s ⟹ 放弃这一趟）。
    #:   ☠️ 本参数**一度叫 `traj_timeout_s`**，那是同一个名字指两个阈值、两种后果 ——
    #:   现场照文档读数会读错一档，所以改掉。**本节点没有那第二级**（见 `node_core` 的说明）。
    #: 取 0.5 s 与 `mocap_timeout_s` / `px4_timeout_s` **同量级**：上游 `traj` 是 50 Hz 发的
    #:   ⟹ 25 拍才判超时，远大于抖动；而它在"上游真挂了"时半秒内就冻住。
    traj_stream_gap_s: float = 0.5
    #: 参考点相对**位置保持锚点**的偏移 [m]（ENU）。
    #: ★ 锚点 = 闸门第一次打开那一拍的测量，之后不再移动（ticket 09）⟹ 这是**绝对**目标；
    #:   早期写成"相对当拍测量"的版本会让误差永远关不掉，见 `node_core.hold_reference` 的 ★★。
    #: ⚠️ **只在 `reference_source=hold` 时被读**；`traj` 档下必须全 0（`_validate` 会拒非 0）。
    hold_offset: tuple = (0.0, 0.0, 0.0)
    keep_yaw: bool = True
    hold_yaw_deg: float = 0.0
    #: 停机条件（**没有通道号**，理由见 `node_core`）
    mocap_timeout_s: float = 0.5
    #: A 档：PX4 两条状态消息的龄上限 [s]（各判各的；实测 32.5 / 31.6 Hz ⟹ 0.5 s = 16 拍）
    px4_timeout_s: float = 0.5
    status_timeout_s: float = 0.75
    stop_on_nav_state_exit: bool = True
    stop_on_armed_drop: bool = True
    warmup_cycles: int = 5
    #: 动捕输入合理性（ticket 07 / M5）—— 三条判据的取法在 `node_core.StopConfig` 里
    mocap_stamp_regress_s: float = 0.01
    mocap_jump_speed_max: float = 20.0
    mocap_jump_rate_max: float = 12.6
    #: 求解不可用之后**先保持几拍**上一帧可行指令，再切安全悬停（`FallbackLadder`）
    fallback_hold_frames: int = FALLBACK_HOLD_FRAMES
    #: ★ **故障注入**（默认 0 = 关）：已发布 ≥ N 帧之后，每一拍的求解结果都被换成一个坏解。
    #: 为什么必须在 DUT 里开这个口子（而不是靠夹具）：见模块 docstring 的 ★ 那段。
    inject_solve_fail_after_frames: int = 0
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
            state_source=self.state_source,
            reference_source=self.reference_source,
            hold_offset=tuple(float(v) for v in self.hold_offset),
            hold_yaw=yaw,
            stop=StopConfig(
                mocap_timeout_s=self.mocap_timeout_s,
                px4_timeout_s=self.px4_timeout_s,
                traj_stream_gap_s=self.traj_stream_gap_s,
                status_timeout_s=self.status_timeout_s,
                stop_on_nav_state_exit=self.stop_on_nav_state_exit,
                stop_on_armed_drop=self.stop_on_armed_drop,
                warmup_cycles=self.warmup_cycles,
                mocap_stamp_regress_s=self.mocap_stamp_regress_s,
                mocap_jump_speed_max=self.mocap_jump_speed_max,
                mocap_jump_rate_max=self.mocap_jump_rate_max,
            ),
            fallback_hold_frames=int(self.fallback_hold_frames),
        )

    def z0_params(self) -> Z0Params:
        return Z0Params(mass=self.mass, hover_thrust=self.hover_thrust)

    def describe(self) -> str:
        return ("rate=%g Hz 心跳=%g Hz viz=%g Hz | ★状态源=%s（%s）"
                " | ★参考=%s"
                " | τ=%g s 参考偏移(仅hold档)=%s yaw=%s"
                " | 模型 mass=%g kg hover_thrust=%g ⇒ T_max=%.4f N"
                " | 垂直偏置 ki_z=%g N/(m·s)%s"
                " | 停机(mocap>%g s / px4>%g s / status>%g s / 离开 offboard=%s / 掉 armed=%s)"
                " | 心跳%s"
                " | 兜底：保持 %d 拍 → 安全悬停；位姿合理性上限 %g m/s / %g rad/s"
                % (self.rate_hz, self.heartbeat_hz, self.viz_hz,
                   self.state_source, self.state_source_desc(),
                   self.reference_source_desc(),
                   self.tau_s,
                   tuple(round(float(v), 3) for v in self.hold_offset),
                   "保持当前" if self.keep_yaw else f"{self.hold_yaw_deg:g}°",
                   self.mass, self.hover_thrust, self.z0_params().t_max,
                   self.ki_z, "（关）" if self.ki_z <= 0.0 else "",
                   self.mocap_timeout_s, self.px4_timeout_s, self.status_timeout_s,
                   self.stop_on_nav_state_exit, self.stop_on_armed_drop,
                   "无条件发（闸门只管 setpoint）" if self.heartbeat_always
                   else "与 setpoint 同生死（交给 PX4 的 COM_OF_LOSS_T）",
                   self.fallback_hold_frames, self.mocap_jump_speed_max,
                   self.mocap_jump_rate_max)
                + ("　⚠️ **故障注入已开**（已发 %d 帧之后每拍都换成坏解）—— 装桨前必须关"
                   % self.inject_solve_fail_after_frames
                   if int(self.inject_solve_fail_after_frames) > 0 else ""))

    def state_source_desc(self) -> str:
        """这一档**实际会订阅什么**（`ros2 node info` 与启动日志里的那一行）。

        ★ 它是"本档不使用动捕输入"这条验收项在**日志里**的落点：A 档的启动日志里
          不该出现 `/mocap/pose_enu` 这个词，而台架正是这么断言的。
        """
        if self.state_source == STATE_SOURCE_PX4:
            return f"PX4 {self.px4_position_topic} + {self.px4_attitude_topic}（NED/FRD）"
        return f"{TOPIC_MOCAP} + {TOPIC_MOCAP_VEL}（ENU/FLU）"

    def reference_source_desc(self) -> str:
        """参考那一档**实际会订阅什么**（与 `state_source_desc` 同一用途：日志 + 话题边界断言）。

        ★ 独立一行（不并进 `state_source_desc`）：两个来源正交，合成一句之后
          "`hold` 档有没有订阅 `/traj/reference`"在日志里就查不出来了。
        """
        if self.reference_source == REFERENCE_SOURCE_TRAJ:
            return f"{TOPIC_TRAJ_REF}（相对锚点，断流 {self.traj_stream_gap_s:g} s 冻结）"
        return f"参数化锚点 + 偏移 {tuple(round(float(v), 3) for v in self.hold_offset)}"


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
        # ★ 第 4 个参数是**求解器的**离散步长（`tf/N`），只有 `traj` 档的参考外推用它；
        #   从 `self.ocp` 现取（不写死、也不从 `tau_s` 推）—— 见 `NodeCore.__init__` 的 ★。
        self.core = NodeCore(self.z0, self.config, self.ocp.config.N, self.ocp.dt)

        # —— 上游 ——
        # ★★ 按档**条件订阅**（ticket 09）：A 档不订阅动捕，于是"本档不使用动捕输入"这条
        #    验收项在话题边界上**可断言**（`matched_subscribers() == 0`），而不是靠读代码相信。
        #    两档各自的槽位都在（`node_core` 里），但只有这一档的那条会被填。
        self.sub_mocap = None
        self.sub_mocap_vel = None
        self.sub_pos = None
        self.sub_att = None
        if params.state_source == STATE_SOURCE_PX4:
            self.sub_pos = self.create_subscription(
                VehicleLocalPosition, params.px4_position_topic, self._on_local_position,
                QOS_SUB_STATUS)
            self.sub_att = self.create_subscription(
                VehicleAttitude, params.px4_attitude_topic, self._on_attitude, QOS_SUB_STATUS)
        else:
            self.sub_mocap = self.create_subscription(
                PoseStamped, TOPIC_MOCAP, self._on_mocap, QOS_MOCAP)
            # ★ 速度是**独立的一条话题**（ticket 15）：`PoseStamped` 里没有 `twist` 字段，
            #   不能再从位姿那一条取。`mocap` 节点把 `/vrpn/<刚体>/twist` 原样 republish 成
            #   `/mocap/velocity_enu`（`TwistStamped`，同 frame）⟹ 与 `PoseStamped` 那条
            #   同样的 QoS（`mocap` 节点用 `QoS(10)` 发，见 QOS_MOCAP 的注释）。
            #   ⚠️ 两档的分岔仍只有这一处：A 档的速度在 `vehicle_local_position` 里，不订这条。
            self.sub_mocap_vel = self.create_subscription(
                TwistStamped, TOPIC_MOCAP_VEL, self._on_mocap_velocity, QOS_MOCAP)
        # ★ 参考那条（ticket 17）**与状态源正交**，所以是**另一个** `if`，不是上面那个的 `else`
        #   ⟹ "A 档不订阅动捕"那条断言（`matched_subscribers() == 0`）不受影响。
        self.sub_traj_ref = None
        if params.reference_source == REFERENCE_SOURCE_TRAJ:
            self.sub_traj_ref = self.create_subscription(
                PositionTarget, TOPIC_TRAJ_REF, self._on_traj_reference, QOS_TRAJ_REF)
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
        #: 所有**有限**的求解耗时 [ms]（p50 / p99 用）。⚠️ 不设上限地留 —— 这是 SITL/台架上
        #: 跑几十秒的短趟；一次 30 min 的实飞会攒到 1e5 个数（几十万个 float，无所谓）。
        self.solve_ms_all: list = []
        #: 约束饱和计数：`{"F_max": n, ...}` —— 与逐帧的 `sat` 字段互为汇总/明细
        self.sat_count: dict = {}
        self._last_sol = None
        self._last_sat: tuple = ()
        self.degraded_count = 0
        self.rejected = 0
        self.first_publish_ns: Optional[int] = None
        self.last_publish_ns: Optional[int] = None
        #: 喂给 `solve(u_ref=…)` 的前馈 = **参考自己的解析前馈**（静止锚点 ⟹ `[T_hover,0,0,0]`）。
        #: ⚠️ 必须**一开始就有值**：`prediction=False` 时 `u_ref` 不能省（`OfflineOcp.solve`
        #: 会 raise，见那边的 docstring），而热身拍（`publish=False`）**也要解**。
        #: ★★ **四路都是常量**（`t_hover` + 三个 0），`__init__` 建一次就不再变。
        #:   ☠️ 它**不含**垂直偏置 `ΔF`（ticket 20 的落点裁定，2026-09-29）：`ΔF` 加在**发出去的
        #:   指令**上（见 `_normal_frame`）。曾有一版让每拍 `hold_feedforward(…, ΔF)` 重算、把
        #:   `ΔF` 写进这里 —— 那条路已被实测证伪（`d u[0]/d u_ref[0] = −0.0616 N/N`，反号，
        #:   见 `NodeParams.ki_z` 的 ☠️）。谁要把 `ΔF` 加回这一份，先去看那张表的量法。
        #: ★ 常量 ⟹ 每拍省掉一次 `N×4` 的分配（`N = 50`）。
        #: ⚠️ 但它**不是** 2026-09-28 那一场 runaway 的回退：那次喂的是**上一帧解**，是一条正反馈
        #:   （解偏 ⟹ 下一帧的参考量跟着偏），真 PX4 闭环里把 `u[3]` 从 0 爬到 +4.696 rad/s
        #:   / EKF 帧转了 13.6 圈。依据与第一手读数见 `node_core.hold_feedforward` 的 ★ 两段。
        #:   ☠️ **`ΔF` 的来源是"实测位置误差的积分"，不是 `sol.u`** —— 谁要把 `ΔF` 改成从解里读，
        #:   就是把那条正反馈放回来。判据见 `test_mpc_node.py::test_垂直偏置不跟着解爬`。
        self._u_ref: np.ndarray = hold_feedforward(self.z0, self.ocp.config.N)

        #: 垂直通道的**外层前馈偏置**累积量 [N]（ticket 20）。
        #: ★ 为什么需要它：模型与被控对象之间的**恒定**口径差（失配机型、电池口径）在纯比例
        #:   闭环下只能靠**稳态位置误差**去平衡（OCP 的推力前馈是固定的 `t_hover`）⟹
        #:   稳态偏差正比于口径差，`mass+30` 实测 **−0.6406 m**（判据① 限 0.15 m）。
        #:   积分作用把"要靠位置误差才能撑住的推力"变成"记下来的推力"⟹ 稳态误差归零。
        #: ★ 它的落点是**发出去的指令**（`u_cmd[0] = clip(u[0] + ΔF, 0, f_max)`）：它不动 `f(x,u)`、
        #:   不动 codegen、不动 `params.py`，也**不动代价参考** `u_ref`（那正是被证伪的那条路）。
        self._df: float = 0.0
        #: 上一次积分/冻结的时刻 [s]（`None` = 还没积过）。★ 冻结期间也**推**它，否则恢复后
        #: `Δt` 会是"整段冻结时长"⟹ 一拍积进去一大坨，那正是判据① 窗口里最不该有的暂态。
        self._bias_at_s: Optional[float] = None

        # —— 兜底（ticket 07 / M5）——
        # ★ 悬停帧在这里**建一次**：它是常量（悬停推力 + 零角速率），而且**不依赖任何状态**
        #   —— 这正是"状态不可信时它仍然可用"的原因，也是它必须在 `__init__` 就备好的原因。
        self._hover_sp = sign_map.px4_rates_setpoint((0.0, 0.0, 0.0), self.z0.t_hover, self.z0.t_max)
        self.ladder = FallbackLadder(self.config.fallback_hold_frames)
        #: 「上一帧**可行**指令」（`hold` 档发的就是它，只由正常帧更新）
        self._last_sp = None
        #: 上一拍的闸门原因（用来判断"闸门刚关上"，那一拍要立刻打日志）
        self._last_gate_reason = ""
        self._last_cycle = None
        self._log = open(params.log_file, "w", encoding="utf-8") if params.log_file else None

        # 心跳抽帧：`rate_hz / heartbeat_hz` 取整（默认 100/50 = 2）
        self._heartbeat_every = max(1, int(round(params.rate_hz / max(1e-3, params.heartbeat_hz))))

        self.get_logger().info(
            f"MPC（模型预测控制）节点启动：{params.describe()}\n"
            f"  OCP：N={self.ocp.config.N} tf={self.ocp.config.tf} s（内部步长 {self.ocp.dt * 1e3:.1f} ms）"
            f"；T_max={self.z0.t_max:.4f} N ⟹ 悬停归一化推力 "
            f"{self._hover_sp.thrust_body_z:+.4f}"
            f"；CPU 亲和性={self._cpus_allowed()}")
        # ★ **这一行要列全所有的订阅**：它是"本档到底订了什么"在日志里的**唯一**落点，
        #   而 A 档那条验收项（"本档不使用动捕输入"）就是拿它在启动日志里做断言的。
        #   ⚠️ ticket 17 补了 `/traj/reference` 那一路 —— 原先漏了它，于是轨迹档的启动日志
        #   与悬停档**一模一样**，而"参考那条订上了没有"正是轨迹档第一个要回答的问题。
        subs = [f"{params.state_source_desc()}"
                f"（{'BEST_EFFORT/depth 1' if params.state_source == STATE_SOURCE_PX4 else '可靠/depth 10'}）",
                f"{TOPIC_STATUS}（**BEST_EFFORT**/depth 1）"]
        if params.reference_source == REFERENCE_SOURCE_TRAJ:
            subs.append(f"{TOPIC_TRAJ_REF}（可靠/depth 10）")
        self.get_logger().info(
            "接口：订阅 %s；发布 %s 与 %s（BEST_EFFORT + TRANSIENT_LOCAL/depth 1）"
            % (" + ".join(subs), TOPIC_RATES, TOPIC_MODE))
        if int(params.inject_solve_fail_after_frames) > 0:
            # ★ 用 **error** 级别、并且把"装桨前必须关"写在同一行：这一档一开，
            #   飞机收到的每一帧都是坏解兜底出来的 —— 它只该在台架/SITL 上出现。
            self.get_logger().error(
                "⚠️ **故障注入已开**：`inject_solve_fail_after_frames="
                f"{params.inject_solve_fail_after_frames}` ⟹ 已发布 ≥ {params.inject_solve_fail_after_frames} "
                "帧之后，每一拍的求解结果都会被换成坏解（走兜底阶梯）。"
                "**装桨前必须设回 0**（`preflight_check` 会拦这一条）。")

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

    def _on_mocap_velocity(self, msg: TwistStamped) -> None:
        self.core.on_mocap_velocity(msg, self._now_s())

    def _on_local_position(self, msg: VehicleLocalPosition) -> None:
        self.core.on_local_position(msg, self._now_s())

    def _on_attitude(self, msg: VehicleAttitude) -> None:
        self.core.on_attitude(msg, self._now_s())

    def _on_traj_reference(self, msg: PositionTarget) -> None:
        """`/traj/reference`（ticket 17）—— 掩码/非有限的解析在 `core.on_traj_reference` 里。"""
        self.core.on_traj_reference(msg, self._now_s())

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
            self._gate_closed(now_s, cycle)
            return
        self._last_gate_reason = ""

        # ★ `prediction=False`：每周期只取第一拍 `u`（省 ~1.1 ms/周期，实测，见本包 README）。
        #   它要求**显式**传 `u_ref`（缺省那条"上一帧解平移一拍"是从预测轨迹里来的）
        #   ⟹ 这里喂 `self._u_ref`（**参考自己的解析前馈**，`__init__` 建一次，不是上一帧解）。
        #   ⚠️ 这条注释的前身写的是"上一拍的 `u` 铺满时域（零阶保持）"—— 那是 ticket 06 的旧设计，
        #   2026-09-28 的 runaway 之后就不这么喂了（见 `__init__` 的 ★★）。本次只是把话说对，
        #   代码没变（ticket 20 §9.1）。RViz 那条预测路径是另外取出来的（见 `_viz_tick`）。
        # ★★ 一拍里的**顺序**（ticket 20）：`_clear_bias_if_disabled()`（只在 `ki_z ≤ 0` 时动手）
        #   ⟹ `solve` ⟹ 判这一帧能不能用、并**把 ΔF 加到发出去的那条指令上**（`_normal_frame`）
        #   ⟹ **最后**才 `_integrate_bias_z()`。为什么积分排在判之后：只有判完才知道这一帧是
        #   "正常帧"还是兜底帧，而兜底帧的位置误差**不可归因** ⟹ 不许积进去（见 `_gate_closed`）。
        #   ⚠️ 代价是**一拍的滞后**：这一拍 `solve` 与这一帧发出去的 ΔF 都只积到上一拍为止
        #   （100 Hz 下 10 ms）。对 Ki ≈ 1 的慢积分器可以忽略；换成"先积后解"就得先知道这一帧
        #   会不会兜底 ⟹ 做不到。
        self._clear_bias_if_disabled()
        sol = self.ocp.solve(cycle.x0, cycle.x_ref, u_ref=self._u_ref, prediction=False)
        self.solves += 1
        if self._inject_active():
            sol = _poisoned(sol)        # 故障注入（默认关）：见模块 docstring 的 ★ 那段
        self._last_sol = sol            # ★ 日志面要读它（票面：求解耗时 / 内部状态 / 饱和标志）
        #: ★ **约束饱和标志**（票面要的日志项）：这一拍 `u` 顶在哪几条 `lbu/ubu` 上。
        #: 判据用**生产阈值**（`Z0Params` ⟹ 与 OCP 的 `lbu/ubu` 同源），带容差 —— 见
        #: `node_core.saturated_bounds`。这里只算一次，日志与收尾摘要共用。
        self._last_sat = saturated_bounds(sol.u, self.z0)
        for name in self._last_sat:
            self.sat_count[name] = self.sat_count.get(name, 0) + 1
        t_tot = float(sol.info.time_tot)
        if math.isfinite(t_tot):
            # ★ 只在有限时才更新最坏耗时：`max(nan, x)` 恒为 nan ⟹ 一次非有限的耗时会让
            #   收尾摘要里那一行**从那一刻起永久失去意义**（而它正是判"100 Hz 还够不够"的那个数）。
            self.solve_ms_max = max(self.solve_ms_max, 1e3 * t_tot)
            #: ★ 全部耗时（`sliding window` 之外的那两个分位数要用）：**只有有限的才进来**，
            #: 于是"注入的坏解"（耗时 NaN）不会把 p50/p99 变成 NaN。
            self.solve_ms_all.append(1e3 * t_tot)
        if sol.info.degraded:
            self.degraded_count += 1
            if self.degraded_count in (1, 10) or self.degraded_count % 100 == 0:
                self.get_logger().warn(
                    f"求解退化（第 {self.degraded_count} 次）：{sol.info.line()}")

        # 这一拍的"正常帧"能不能用（三道判据 + 加 ΔF 都收口在 `_normal_frame` 里）
        sp, reason = self._normal_frame(sol)
        # ★★ 下面**只**更新那个垂直偏置，且它的来源是**实测位置误差**（`cycle`），不是 `sol`：
        #   `self._u_ref` 四路都是常量（理由见 `__init__` 的 ★★ 与 `node_core.hold_feedforward`）。
        #   曾经这一行是 `self._u_ref = tile(sol.u)`（"上一帧解铺满时域"）⟹ 正反馈、单调上爬、
        #   runaway。现在"这一拍 `sol` 有多离谱"只走 `_normal_frame` 的判据，
        #   **不许**回流成下一拍的前馈，也**不许**回流成下一拍的 `ΔF`。
        if cycle.anchor:
            # 锚点被钉/重钉的那一拍 ⟹ **复位**偏置（否则上一场的偏置会带进下一场；
            # 订锚点这件事只有 `NodeCore` 知道，所以顺着它的 `anchor` 事件接，不自己另判一次）
            self._df = 0.0
        if reason is None:
            self._integrate_bias_z(cycle, now_s)
        else:
            # 兜底帧：**冻结但不清零** —— 它的输出走 `self._hover_sp`（固定 `t_hover`，不读偏置），
            # 所以此刻偏置不影响输出，只影响"恢复之后从哪儿起步"。清零等于把已经学到的口径差
            # 扔掉，恢复后要重新收敛几秒 —— 而判据① 的窗口正是"最后一次正常帧往前 8 s"。
            self._bias_at_s = now_s

        if not cycle.publish:
            # 热身拍：**求解但绝不发**（理由见 `WARMUP_CYCLES`：第 0 拍的解是出厂迭代点）。
            # ★ 档位跟着复位（这一拍根本不发，档位毫无意义）；累计量留着 —— 它们是病历。
            self.ladder.reset()
            self.gated["warmup"] = self.gated.get("warmup", 0) + 1
            # ★ 热身拍**不发但照记**：它不进话题，却是"最坏求解耗时"里最容易漏掉的那几拍
            #   （RTI 冷启动、代码页刚进 cache）。票面要的是**实测最坏**，不是稳态中位数。
            self._log_frame(time.time_ns(), ts, sp, "warmup", reason or "")
            return

        frame, phase, note = sp, FallbackLadder.NORMAL, ""
        if reason is None:
            report = self.ladder.on_ok()
            if report:
                self.get_logger().warn(report)
        else:
            # ★★ 解不可用 ⟹ 走兜底阶梯（判据 1/2）。`report` 就是票面说的
            #    「置标志并上报」，只在**进入安全悬停那一次**非空（每拍重复会把日志淹掉）。
            phase, report = self.ladder.on_fail(reason, can_hold=self._last_sp is not None)
            frame, phase = self._fallback_frame(phase)
            note = reason
            self.rejected += 1
            if report:
                self.get_logger().error(report)

        if not self.params.heartbeat_always and heartbeat_due:
            # 另一档：心跳与 setpoint 同生死（`heartbeat_always=False`）
            self._publish_mode(ts)
        self._publish_rates(ts, frame, phase, note)

    # ─────────────────────────── 这一拍发什么 ───────────────────────────

    def _clear_bias_if_disabled(self) -> None:
        """**`ki_z ≤ 0` = 硬关掉，连状态一起清零**（不是"停止累积、留着旧值"）。

        `NodeParams.ki_z` 的语义是"积分作用开多大"，`0` 就该等于**没有积分作用**
        —— 留着上一段攒的偏置，A/B 对照里"关掉"那一组会带着前一组的学习量起飞。

        ★ **每拍无条件查一次**（在 `solve` 之前），而不是塞进 `_integrate_bias_z` 的入口：
          那个函数在兜底帧/闸门关着时**根本不会被调用**，而"关掉"这件事必须**立刻**生效
          —— 否则 `ki_z` 中途被置 0 之后，头一帧正常帧还会带着旧偏置发出去。
        ★ 它曾经叫 `_apply_bias_to_u_ref`、还负责每拍用 `ΔF` 重算 `u_ref`。那个落点已被实测
          证伪（`NodeParams.ki_z` 的 ☠️）⟹ 整个函数缩成这一件事，`self._u_ref` 也变成常量
          （`__init__` 建一次）。**名字改了就要跟着改**：改回"往 `u_ref` 里塞东西"之前先读那张表。
        """
        if self.params.ki_z <= 0.0:
            self._df = 0.0

    def _integrate_bias_z(self, cycle, now_s: float) -> None:
        """**纯积分**：把这一拍的垂直位置误差积进 `ΔF`，并**夹在界内**（ticket 20 裁定 1/2）。

        `e_z = z_ref − z_meas`（ENU、z 向上 ⟹ 偏低时为正 ⟹ 加推力）。
        ⚠️ 误差取的是**参考减测量**（`cycle.x_ref[0][2] − cycle.x0[2]`）：`x_ref` 从锚点起
          （不是从当拍测量起，见 `node_core.hold_reference` 的 ★★）⟹ 这个误差**关得掉**；
        用当拍测量当参考的话误差永远为 0、积分器就成了摆设。

        ★ **抗积分饱和：夹的是状态，不是输出**。只夹输出（`u[0] + ΔF` 那个和）的话，状态照样
          一直积上去 —— 顶在界上的那几秒全被记进状态里，误差反向之后要花同样长的时间才退回来。
          ⚠️ 夹在**这里**（状态写完就夹）而不是发布点：那样状态只在"用之前"被夹住，日志里
          （以及两拍之间）它仍是出界的；夹在这里 ⟹ **状态恒在界内**，"顶住时不再累积"才是
          **可直接观测**的（饱和段 `ΔF` 逐帧恒等，见日志的 `df` 列）。
        ★ 界取 `[−t_hover, f_max − t_hover]` 的口径 = **"偏置恰好能覆盖满量程推力"**：
          `−t_hover` 能把悬停推力整个抵消掉、`f_max − t_hover` 能把悬停推力顶到满量程。
          ⚠️ 它**不再**保证"和恒在 `[0, f_max]` 内"（那是旧落点 `u_ref[0] = t_hover + ΔF` 的说法：
          那个和本身就等于发出去的推力）。现在发出的是 `u[0] + ΔF`，`u[0]` 自己就能跑到满量程
          ⟹ **执行器界由发布点那次 `clip(u[0] + ΔF, 0, f_max)` 保证**（见 `_normal_frame`），
          这里的界只管抗饱和。两者**不能合并**：夹状态是为了"退得回来"，夹输出是为了"不越界"。
        """
        dt = 0.0 if self._bias_at_s is None else (now_s - self._bias_at_s)
        self._bias_at_s = now_s
        if self.params.ki_z <= 0.0 or dt <= 0.0:
            return
        e_z = float(cycle.x_ref[0][2] - cycle.x0[2])
        self._df = float(np.clip(self._df + self.params.ki_z * e_z * dt,
                                 -self.z0.t_hover, self.z0.f_max - self.z0.t_hover))

    def _gate_closed(self, now_s: float, cycle) -> None:
        """闸门关着的一拍：**不求解也不发 setpoint**（心跳照发，见模块 docstring）。

        ★ 闸门**刚**关上的那一拍立刻打一行日志（不必等 `log_every` 的抽帧）：输入合理性
          那两条判据给出的读数（"位移 100.000 m > 允许 0.200 m"）是调阈值的人第一个要问的数，
          而它可能只出现一两拍 —— 被抽帧抽掉就等于没有。
        ★ 这一拍**不积分**（位置误差不可归因），但把时钟推过去（见 `_bias_at_s` 的 ★）。
        """
        self._bias_at_s = now_s
        self.ladder.reset()
        self.gated[cycle.reason] = self.gated.get(cycle.reason, 0) + 1
        fresh = cycle.reason != self._last_gate_reason
        self._last_gate_reason = cycle.reason
        if fresh or self.ticks % max(1, self.params.log_every) == 0:
            self.get_logger().info(
                f"t={now_s:.2f}s 闸门关闭（{cycle.reason}）：status 龄 {cycle.age_status_s:.2f}s / "
                f"状态龄 {cycle.age_state_s:.2f}s ⟹ 不发 setpoint"
                + (f"；{cycle.detail}" if cycle.detail else "")
                + ("（心跳照发）" if self.params.heartbeat_always else ""))

    def _normal_frame(self, sol) -> Tuple[Optional[object], Optional[str]]:
        """把这一拍的解变成一个**可发布的帧** → `(帧, None)`，或 `(None, 原因)`。

        三道判据 + 一处**加偏置**都在这个**唯一**发布点收口（顺序别打乱）：

        ① `solution_usable`：求解器的状态码 / `u` 的形状与有限性（判据 1、判据 2 的解这一半）；
        ② **把垂直外层前馈偏置 `ΔF` 加到发出去的推力上**（ticket 20 的落点，见下）；
        ③ `sign_map.px4_rates_setpoint`：映射这条路自己的入参检查（非有限 / 负推力）；
        ④ `frame_is_finite`：**发布前整帧校验**（判据 2）—— ①②③ 各自只看一部分
           （① 看 `u`、③ 看 `ω`/`F`/`T_max`），而"凡是要写进那一帧的数"只有在这里才说得清。

        ★★ **② 为什么在这儿（ticket 20 裁定，2026-09-29）**：积分作用的落点是**发出去的指令**。
          ☠️ 不是 `u_ref` —— 那条路实测**反号**（`d u[0]/d u_ref[0] = −0.0616 N/N`），
          积上去是正反馈、可行域是空集（量法见 `tools/px4-sitl/probe_u_ref_authority.py`）。
          ★ 加在这个函数里而不是 `_publish_rates` 那边：ΔF 是"要写进那一帧的数"之一 ⟹ 与那三道
          判据同一个收口处；在别处再算一遍就等于有了**第二个产地**（本函数的存在理由就是消灭它）。
          ★ 夹 `[0, f_max]`（与 OCP 的 `lbu[0]/ubu[0]` 同界）⟹ 发出去的推力**恒在执行器界内**，
          这就是"夹状态（`ΔF` 的界只管抗饱和）+ 夹输出（这里管不越界）"里后半句的落点。
          ⚠️ 加完之后这一帧**可能**顶界，而日志的 `sat` 列算的是 `sol.u`（求解器给的、未加偏置）
          ⟹ "**发出去的那条**指令顶没顶界"要读 `−thrust_body_z × t_max`（PX4 机体 z 朝下 ⟹
          那一位是负的，见 `sign_map.px4_rates_setpoint:73`），别读 `sat`。
        """
        reason = solution_usable(sol)
        if reason is not None:
            return None, reason
        # ★★ ② 垂直外层前馈偏置的落点：加在**发出去的指令**上，不在 `u_ref` 上（见 docstring）。
        #   算在 `sol.u` 的副本上 ⟹ `self._last_sol` 与日志的 `u` 列仍是**求解器原样输出**，
        #   "解出来多少"与"发出去多少"在日志里是可分的两列（`u` vs `thrust_body_z`）。
        u_cmd = np.array(sol.u, dtype=float, copy=True)
        u_cmd[0] = float(np.clip(u_cmd[0] + self._df, 0.0, self.z0.f_max))
        try:
            sp = sign_map.px4_rates_setpoint(u_cmd[1:], u_cmd[0], self.z0.t_max)
        except ValueError as exc:
            return None, f"映射拒绝：{exc}"
        if not frame_is_finite(sp):
            return None, ("整帧校验不通过：roll/pitch/yaw/thrust_body_z 里有非有限值"
                          f"（{sp.roll!r} / {sp.pitch!r} / {sp.yaw!r} / {sp.thrust_body_z!r}）")
        return sp, None

    def _fallback_frame(self, phase: str):
        """按档位取这一拍要发的帧 → `(帧, 实际档位)`。

        ★ `hold` 需要"上一帧**可行**指令"，而它可能**不存在**（第一拍就不可用 —— 此前一直在
          热身或闸门关着）。那时退回到安全悬停：悬停帧**不依赖任何状态**（推力沿机体系 z +
          零角速率），而"不发"会把飞机留在 PX4 **锁存**的那一帧上（上电初值就是零推力）。
          ⚠️ 这个判断**归阶梯**（`on_fail(…, can_hold=)`，见 `test_兜底阶梯_没有可保持的帧时
          直接进悬停_且算一次事件`）：档位与实际发出的帧必须**同源**，否则 JSONL 打 `hover`、
          收尾摘要却说"此刻仍在 hold"，两边互相打脸；`holds` 也会把悬停帧记成保持帧。
        ★ `_last_sp` **只由正常帧更新** —— 否则兜底自身发的帧会把"上一帧可行指令"越推越旧，
          `hold` 就名存实亡（而它正是"别一抖动就换姿态"那一档）。
        """
        if phase == FallbackLadder.HOLD and self._last_sp is not None:
            return self._last_sp, FallbackLadder.HOLD
        # ⚠️ 走到这里有两种情形，**都必须回悬停**：① 档位本来就是 `hover`；② 档位说 `hold`
        #   却没有可保持的帧 —— 那只能是调用方漏传 `can_hold`（`on_fail` 的契约）。
        #   回落成安全悬停而不是发 `None`：帧的内容**永远**是安全的，不会因为簿记打错而改变。
        return self._hover_sp, FallbackLadder.HOVER

    def _inject_active(self) -> bool:
        """故障注入是否生效（默认关；语义见 `NodeParams.inject_solve_fail_after_frames`）。"""
        n = int(self.params.inject_solve_fail_after_frames)
        return n > 0 and self.published >= n

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

    def _publish_rates(self, ts: int, sp, phase: str = FallbackLadder.NORMAL,
                       note: str = "") -> None:
        """发布一帧 setpoint。★ **兜底帧与正常帧走的是同一条路**（同一个话题、同样的字段）。

        这**是刻意的**：`hold` 的语义是"内容原样、**时间戳换新**"——于是"它在保持"这件事
        在话题上可观测（同一个 `roll/pitch/yaw/thrust_body[2]` 连着出现 N 次，而 `timestamp`
        每次都在前进）。若兜底另走一条"不发"的路，PX4 那边照样在飞，而**我们这边什么都看不见**。
        """
        msg = VehicleRatesSetpoint()
        msg.timestamp = ts
        msg.roll = float(sp.roll)
        msg.pitch = float(sp.pitch)
        msg.yaw = float(sp.yaw)
        msg.thrust_body[0] = 0.0
        msg.thrust_body[1] = 0.0
        msg.thrust_body[2] = float(sp.thrust_body_z)
        if phase == FallbackLadder.NORMAL:
            self._last_sp = sp        # 「上一帧可行指令」只由正常帧更新（见 `_fallback_frame`）
        self.pub_rates.publish(msg)

        now_ns = time.time_ns()
        self.published += 1
        self.first_publish_ns = self.first_publish_ns or now_ns
        self.last_publish_ns = now_ns
        self._log_frame(now_ns, ts, sp, phase, note)

        if self.ticks % max(1, self.params.log_every) == 0:
            self.get_logger().info(
                "t=%.2fs 第 %d 帧 | roll=%+.3f pitch=%+.3f yaw=%+.3f thrust_body[2]=%+.4f"
                " | 解 %d 次、闸门 %s、退化 %d、%s"
                % (self._now_s(), self.published, msg.roll, msg.pitch, msg.yaw,
                   msg.thrust_body[2], self.ticks,
                   ",".join(f"{k}×{v}" for k, v in sorted(self.gated.items())) or "无",
                   self.degraded_count, self.ladder.describe()))

    def _log_frame(self, now_ns: int, ts: int, sp, phase: str = FallbackLadder.NORMAL,
                   note: str = "") -> None:
        """逐帧 JSONL（取证用）。⚠️ 每个字段都要显式 `float()` / `int()`：

        `thrust_body[2]` 是 ROS 的 **float32**、`nav_state` 是 **uint8**，`json.dumps` 两个都不认
        —— 而定时器回调里抛异常 = **整个 spin 挂掉**（症状是"节点起来一秒就没了、日志文件是空的"）。
        这条是 05 用一次真事故换来的（`offboard_smoke._tick` 里的同一段）。

        ★★ **票面验收项「日志记录 MPC（模型预测控制）内部状态、求解耗时、约束饱和标志，
        供飞行后复盘而不只依赖记忆」就落在这一行**。三样各自的读者：

        | 字段 | 复盘时回答的问题 |
        |---|---|
        | `solve_ms` / `solve_ok` / `status` / `sqp_iter` / `qp_stat` / `degraded` | "这一拍的解是怎么来的、求解器当时什么状态"（**D6 那个数字**就从 `solve_ms` 上取） |
        | `sat`（+ 收尾摘要的 `sat_count`） | "它有没有顶在约束上" —— 顶约束**不是错**，但漂移与顶约束同时出现就是归因线索 |
        | `u` / `x0` / `x_ref0` / `x_ref0_v` | 把一帧从话题上**重算回去**：`u` 是求解器给的、`x0` 是喂进去的、`x_ref0` + `x_ref0_v` 是要它去哪、走多快 |
        | `u_ref0` / `x_ref0_q` | 代价里的**控制参考**与**姿态参考**（`x_ref0` 只有位置分量）：判"棘轮"要看 `u` 与 `u_ref0` 的差，判"机头在追哪"要看 `x0[6:10]` 与 `x_ref0_q` 的差 |
        | `ref_anchor` | "这个目标是哪一刻钉的、后来有没有被搬走"（`""` = 常态；见 `node_core._update_anchor`） |
        | `state_source` / `age_*` / `ekf_*` | "这个数是**谁**给的、多旧"（A 档尤其重要，见 `node_core.describe_state`） |
        | `reference_source` / `ref_cb` / `ref_bad` / `ref_age_s` / `ref_frozen` | "**参考**是谁给的、多旧、有没有被冻住"（ticket 17）；`ref_frozen` 为真时 `x_ref0_v` 应当是 0 |
        | `df` | "那个垂直积分偏置积到了多少"（ticket 20）—— 与**上一拍**的 `u[0]` 一起 `−thrust_body_z·t_max`，就是"它到底有没有加到发出去的指令上"（☠️ **不是同一拍**：见下面 `df` 那一列的 ☠️） |

        ⚠️ 一帧里**同时**有 `u`（FLU/ENU 的内部量）与 `roll/pitch/yaw/thrust_body_z`（PX4 的
        FRD/NED 量）：这不是重复，而是**符号映射的现场** —— 复盘时两边一对照，
        "映射写错了"与"模型解错了"就能分开（这正是 `sign-map.md` 要的东西）。
        """
        if self._log is None:
            return
        cycle = self._last_cycle
        sol = self._last_sol
        info = None if sol is None else sol.info
        rec = {
            "wall_ns": now_ns, "ts": int(ts), "tick": int(self.ticks),
            "roll": _jnum(None if sp is None else sp.roll),
            "pitch": _jnum(None if sp is None else sp.pitch),
            "yaw": _jnum(None if sp is None else sp.yaw),
            "thrust_body_z": _jnum(None if sp is None else sp.thrust_body_z),
            #: ★ 兜底档位（"normal" / "hold" / "hover" / "warmup"）—— 票面要的"这个事件在飞行
            #: 日志里**可检索**"就是这一列：`grep '"fallback": "hover"'` 能一次捞出全段。
            #: `fallback_note` 是**为什么**（求解器那一行读数 / 映射拒绝的原因）。
            "fallback": str(phase),
            "fallback_note": str(note),
            # —— 求解耗时与求解器内部状态 ——
            "solve_ms": None if info is None else _jnum(
                1e3 * float(info.time_tot) if math.isfinite(float(info.time_tot)) else float("nan")),
            "solve_cycle_ms": None if info is None else _jnum(
                1e3 * float(getattr(info, "time_cycle", float("nan")))),
            "solve_ok": None if info is None else bool(info.ok),
            "degraded": None if info is None else bool(info.degraded),
            "status": None if info is None else _jint(getattr(info, "status", None)),
            "sqp_iter": None if info is None else _jint(getattr(info, "sqp_iter", None)),
            "qp_stat": None if info is None else _jlist(getattr(info, "qp_stat", None)),
            "u_viol": None if info is None else _jnum(getattr(info, "u_bound_violation", None)),
            "quat_drift": None if info is None else _jnum(getattr(info, "quat_norm_drift", None)),
            # —— 控的现场：给的是什么、要它去哪、解出来什么 ——
            "u": None if sol is None else _jlist(sol.u),
            "sat": list(self._last_sat),
            "x0": None if cycle is None or cycle.x0 is None else _jlist(cycle.x0),
            "x_ref0": None if cycle is None or cycle.x_ref is None else _jlist(cycle.x_ref[0][:3]),
            #: ★★ 参考的**速度**（`x_ref` 的第 3–6 个分量；`x_ref0` 只记了位置那三个）。
            #:   为什么要单独一列（ticket 17）：它是"参考在不在动、朝哪动、多快"的**唯一**
            #:   直接读数 —— 而它在 `hold` 档**恒为 0**（`hold_reference` 里速度那一项就是 0）
            #:   ⟹ 轨迹档一接上，这一列就是一次**跳变**，"参考接到没有"不必再从别处推断。
            #:   ⚠️ 它**不是**测量速度（那是 `x0[3:6]`，来自 PX4/动捕）：参考速度恒 0 而测量
            #:   速度非 0 = 飞机在往目标点归位或**在漂**；两者分不开时看 `ref_frozen`。
            "x_ref0_v": (None if cycle is None or cycle.x_ref is None
                         else _jlist(cycle.x_ref[0][3:6])),
            #: ★★ 参考的**姿态**单独一列（`x_ref0` 只记了前 3 个位置分量）。
            #:   为什么要它：判"控制器在追哪去"要的是 `x_ref0_q` 与 `x0[6:10]` 的**差**，
            #:   而只有前者没进日志时，飞行后从 `x0` 那一列**推不出**参考机头
            #:   （`x_ref0` 是锚点 + 偏移，锚点机头只出现在 `ref_anchor` 那一拍的
            #:   `x0` 里 —— 2026-09-28 的偏航复盘就卡在这一格，只能再飞一场）。
            "x_ref0_q": (None if cycle is None or cycle.x_ref is None
                         else _jlist(cycle.x_ref[0][6:10])),
            #: ★★ 喂给求解器的**前馈** `u_ref`（第一拍）。为什么要逐帧记：代价项 `R·(u − u_ref)`
            #:   让 `u_ref` 也成了"控制器在追什么"的一部分，而它**只在日志里看得见** ——
            #:   2026-09-28 的 runaway 复盘时手上只有 `u`（解出来的）与 `x_ref0`（状态参考），
            #:   偏偏缺"控制参考"，于是"棘轮"这条结论只能靠离线复算才站得住。
            #:   ★ **四路都恒等，逐帧常量**（`[t_hover, 0, 0, 0]`）。⚠️ 2026-09-29 起是**真的常量**：
            #:   曾经有一版让推力那一路随 `ΔF = t_hover + ΔF` 时变，落点搬到指令上之后又变回常量
            #:   ⟹ 拿这一列验证"参考漂没漂"的老断言重新成立（`test_垂直偏置不跟着解爬`）。
            #:   ☠️ 所以**别**在这一列里找 ΔF —— `ΔF` 有自己的一列（见下），而它发出去的样子
            #:   在 `−thrust_body_z × t_max`（那一列才是"发出去多少"；符号见 `sign_map:73`）。
            "u_ref0": None if self._u_ref is None else _jlist(self._u_ref[0]),
            #: ★ 垂直通道的**前馈偏置累积量** `ΔF` [N]（ticket 20）。
            #:   为什么要单独一列、而不是让人拿"发出去的推力 − `t_hover`"去减：
            #:   **那两个数的口径都能变**（`t_hover = mass·g`，而 `mass` 是节点参数、SITL 与实机不同；
            #:   更要紧的是本项目的 `hover_thrust` 口径本身就有 3.92 × 的分歧），而且发出去的推力
            #:   里还有求解器那份 `u[0]`（它每拍都不同）⟹ 那种减法**量不出积分器的状态**。
            #:   直接记状态 ⟹ 复盘时不需要知道任何常数就能读出"积分器积到了多少"。
            #: ☠️ **这一列与"这一帧发出去时用的那个 ΔF"差一拍** —— 别拿同一拍的
            #:   `−thrust_body_z·t_max − u[0]` 去减它：`_normal_frame` 用的是**上一拍积完**的值，
            #:   而 `_integrate_bias_z` 排在 `_normal_frame` **之后**（顺序见 `_tick` 的 ★★）。
            #:   实测（2026-09-29 台架，`ki_z = 20 N/(m·s)`、`e_z = 0.15 m`、100 Hz）：
            #:   同一拍相减差 **0.0300 N**，正好 `= ki_z·e_z·Δt` —— 而拿**上一拍**的 `df` 去减，
            #:   残差落到 `1e-4` 以内（那正是 `test_垂直偏置加在发出去的指令上_不在代价参考上`
            #:   的口径）。差一拍这件事是设计（`_integrate_bias_z` 的 docstring 写了为什么排在后面），
            #:   但**从日志上读的人不看到这一行就会量错**。
            #:   ★ 它还是**抗积分饱和**唯一的观测面：饱和段里 `ΔF` 应当**逐帧恒等**
            #:   （夹的是状态）；若换成"只夹输出"，这一列会继续单调长。
            "df": round(float(self._df), 6),
            #: 锚点机头 [rad]（没钉过 = `None`）。逐帧恒等 ⟹ 压缩后零成本，而它是"目标朝向"
            #: 这个常量的**直接读数**：`x_ref0_q` 能从它和 `hold_yaw` 反推，反之不能。
            "anchor_yaw": None if self.core.anchor_yaw() is None
            else _jnum(self.core.anchor_yaw()),
            #: ★ 位置保持**锚点**被钉/重钉的那一拍（`""` = 常态）。`x_ref0` 只有值、没有来历：
            #: "这个目标是哪一刻定下来的、后来有没有被搬走"从数值上看不出来 —— 而它是复盘
            #: 第一个要问的问题（ticket 09 首飞的爬升就是栽在"目标每拍都在跟着测量走"上）。
            "ref_anchor": "" if cycle is None else str(cycle.anchor),
            "age_status_s": round(float(cycle.age_status_s), 4),
            "age_state_s": _jnum(cycle.age_state_s),
            "cycles_since_gate": int(cycle.cycles_since_gate),
            "nav_state": int(self.core.status.nav_state),
            "arming_state": int(self.core.status.arming_state),
            "status_cb": int(self.core.status_frames),
        }
        # ★ 状态源的取证面（谁给的、多旧、PX4 自己怎么说）—— 扁平键，直接并进来。
        #   键名与上面不冲突（`describe_state` 给的是 `state_source` / `age_pos_s` / `ekf_*`）。
        rec.update(self.core.describe_state(self._now_s()))
        self._log.write(json.dumps(rec) + "\n")
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
                "定时器 %d 拍、求解 %d 次、%s、退化 %d 拍、拒绝 %d 拍；"
                "约束饱和：%s；闸门：%s；%s"
                % (self.published, self.heartbeats, self.heartbeats / span if span > 0 else float("nan"),
                   span, rate, self.ticks, self.solves, self.solve_ms_desc(),
                   self.degraded_count, self.rejected,
                   ", ".join(f"{k}×{v}" for k, v in sorted(self.sat_count.items())) or "无",
                   ", ".join(f"{k}×{v}" for k, v in sorted(self.gated.items())) or "无",
                   self.ladder.describe()))

    def solve_ms_desc(self) -> str:
        """求解耗时的**分布**（p50 / p99 / 最坏）—— ★ **D6 那个数字**就从这里读。

        ★ 为什么必须给分位数而不是只给最坏：最坏那一个是**你没法重复的那个数**（被抢占、
          首次编译、别的进程刚好在写盘），而 p99 才是"这台机器在这个负载下能不能跑 100 Hz"的量。
          ⚠️ 但也**不能只看 p50**（3.5 ms 的中位数会让 30 ms 的尾巴完全消失）——
          票面的判据写的是"**实测最坏**求解耗时"，所以两个都打出来，读的人才知道差距有多大。
        ★ 比例尺：≤10 ms ⟹ 100 Hz 有余；10–20 ms ⟹ 降 50 Hz；>20 ms ⟹ **停下优化，不硬上**。
        """
        n = len(self.solve_ms_all)
        if n == 0:
            return "求解耗时：无（一次都没解）"
        arr = np.sort(np.asarray(self.solve_ms_all, dtype=float))
        return ("求解耗时 p50 %.3f ms / p99 %.3f ms / 最坏 %.3f ms（%d 次）"
                % (float(np.percentile(arr, 50)), float(np.percentile(arr, 99)),
                   self.solve_ms_max, n))

    def close(self) -> None:
        if self._log is not None:
            self._log.close()
            self._log = None

    @property
    def _last_ref(self):
        cycle = self._last_cycle
        return None if cycle is None or cycle.x_ref is None else cycle.x_ref[0][:3]


# ─────────────────────────── 故障注入（ticket 07 / M5）───────────────────────────

class _InjectedInfo:
    """注入用的假 `SolveInfo`（只带调用方**真的读到**的那几样：`ok` / `degraded` / `line()` / `time_tot`）。

    ★ 刻意**不做** `SolveInfo` 的完整替身（没有 `status` / `sqp_iter` / `qp_stat`）：
      "状态码非零"这件事在本节点里的**唯一**表现就是 `ok=False`（`solution_usable` 只看它，
      不读 `status`），多摆几个没人读的属性只会让"到底哪条判据在起作用"变模糊。
    """

    time_tot = float("nan")

    @property
    def ok(self) -> bool:
        return False

    @property
    def degraded(self) -> bool:
        return True

    def line(self) -> str:
        return ("注入的坏解：`-p inject_solve_fail_after_frames:=…` 在生效"
                "（台架/SITL 专用，不是求解器真的失败了）")


def _poisoned(sol):
    """把一个**好解**换成一个"什么都不告诉我们"的坏解（故障注入用）。

    ★ 三样一起换，**不是**为了保险：它们各自考的是不同的路径 ——
      `u` 全 NaN 走的是判据 2（非有限 ⟹ 整帧丢弃），`ok=False`（= 状态码非零）走的是判据 1 的
      "求解失败"那一支，耗时 NaN 走的是簿记里的污染防护（`solve_ms_max`）。
      只换一样，另两条路径就永远没被夹具覆盖过。
    """
    return replace(sol, u=np.full(NU, float("nan")), info=_InjectedInfo())


# ─────────────────────────── JSON 安全（日志面）───────────────────────────
#
# ★ 为什么值得三个小函数：JSONL 是**飞行后复盘**的唯一逐帧记录，而它的两个杀手都不是逻辑错
#   —— `json.dumps` 对 numpy 标量 / `float32` / `uint8` 直接抛（一次就够让定时器回调挂掉），
#   而 `NaN` / `Infinity` 会被写成**非法 JSON**（`json.loads` 能读、别的解析器不能）。
#   所以边界上只有一句话：**进 JSON 的每个数都要过这里**。
#   ⚠️ 同一个取向在小写入端也有一份（`node_core._num_json`，管它自己的取证字段）；
#      两处都只有三行、且各自守住自己那一侧的边界 —— 合并成一处反而会让"哪一侧漏了"看不出来。

def _jnum(v, nd: Optional[int] = None) -> Optional[float]:
    """数 → JSON 安全（非有限 / `None` ⟹ `None`；`nd` 给了就四舍五入到那么多位）。"""
    if v is None:
        return None
    f = float(v)
    if not math.isfinite(f):
        return None
    return f if nd is None else round(f, nd)


def _jint(v) -> Optional[int]:
    """整数 → JSON 安全（`None` ⟹ `None`）。"""
    return None if v is None else int(v)


def _jlist(arr, nd: int = 6) -> Optional[list]:
    """数组 → JSON 安全（numpy 也认；非有限的元素写成 `None`）。"""
    if arr is None:
        return None
    return [_jnum(v, nd) for v in np.asarray(arr, dtype=float).ravel()]


# ─────────────────────────── 参数 ───────────────────────────

#: `(参数名, 默认值, 类型)`。★ 默认值全部与 `NodeParams` 的字段一致 —— 两处一起改。
_PARAMS = (
    ("rate_hz", 100.0, float),
    ("heartbeat_hz", 50.0, float),
    ("viz_hz", 10.0, float),
    ("heartbeat_always", True, bool),
    ("mass", 0.75, float),
    ("hover_thrust", 0.1589, float),
    ("tau_s", 0.05, float),
    ("ki_z", 1.0, float),
    ("state_source", "mocap", str),
    ("reference_source", "hold", str),
    ("traj_stream_gap_s", 0.5, float),
    ("px4_position_topic", TOPIC_PX4_POS, str),
    ("px4_attitude_topic", TOPIC_PX4_ATT, str),
    ("hold_offset_x", 0.0, float),
    ("hold_offset_y", 0.0, float),
    ("hold_offset_z", 0.0, float),
    ("keep_yaw", True, bool),
    ("hold_yaw_deg", 0.0, float),
    ("mocap_timeout_s", 0.5, float),
    ("px4_timeout_s", 0.5, float),
    ("status_timeout_s", 0.75, float),
    ("stop_on_nav_state_exit", True, bool),
    ("stop_on_armed_drop", True, bool),
    ("warmup_cycles", 5, int),
    ("mocap_stamp_regress_s", 0.01, float),
    ("mocap_jump_speed_max", 20.0, float),
    ("mocap_jump_rate_max", 12.6, float),
    ("fallback_hold_frames", FALLBACK_HOLD_FRAMES, int),
    ("inject_solve_fail_after_frames", 0, int),
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
        ki_z=get("ki_z").value,
        state_source=get("state_source").value,
        reference_source=get("reference_source").value,
        traj_stream_gap_s=get("traj_stream_gap_s").value,
        px4_position_topic=get("px4_position_topic").value,
        px4_attitude_topic=get("px4_attitude_topic").value,
        hold_offset=(get("hold_offset_x").value, get("hold_offset_y").value,
                     get("hold_offset_z").value),
        keep_yaw=get("keep_yaw").value,
        hold_yaw_deg=get("hold_yaw_deg").value,
        mocap_timeout_s=get("mocap_timeout_s").value,
        px4_timeout_s=get("px4_timeout_s").value,
        status_timeout_s=get("status_timeout_s").value,
        stop_on_nav_state_exit=get("stop_on_nav_state_exit").value,
        stop_on_armed_drop=get("stop_on_armed_drop").value,
        warmup_cycles=int(get("warmup_cycles").value),
        mocap_stamp_regress_s=get("mocap_stamp_regress_s").value,
        mocap_jump_speed_max=get("mocap_jump_speed_max").value,
        mocap_jump_rate_max=get("mocap_jump_rate_max").value,
        fallback_hold_frames=int(get("fallback_hold_frames").value),
        inject_solve_fail_after_frames=int(get("inject_solve_fail_after_frames").value),
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
    for name in ("rate_hz", "heartbeat_hz", "viz_hz", "mass", "hover_thrust",
                 "mocap_timeout_s", "px4_timeout_s", "traj_stream_gap_s", "status_timeout_s",
                 # 输入合理性那三条（ticket 07）：非有限会让判据**静默失效**
                 # （`nan > x` 恒假 ⟹ 跳变判据永远不触发，而"永远不触发"看起来像"没有跳变"）。
                 "mocap_stamp_regress_s", "mocap_jump_speed_max", "mocap_jump_rate_max"):
        val = getattr(p, name)
        if not math.isfinite(val) or val <= 0:
            raise SystemExit(f"参数 {name} 要是正的有限数，收到 {val!r}")
    if p.heartbeat_hz > p.rate_hz:
        raise SystemExit(
            f"heartbeat_hz({p.heartbeat_hz}) 不能大于 rate_hz({p.rate_hz})："
            "心跳是从求解节拍上抽帧发的（见 `_heartbeat_every`），比它快没有意义。")
    # ★★ `tau_s` **单独判**：它的下界是 **0**，不是"正的"（ticket `phase4/05`，2026-10-09）。
    #   τ = 0 = **不补偿**，是**首飞冻结值**（ticket `phase4/03`：过补偿比不补偿更危险
    #   ⟹ 实飞起始值取 0），所以它**不是**"无意义的参数"。
    #   ☠️ 2026-10-09 之前它被上面那个"正的有限数"的循环一起拦掉 ⟹ **冻结值在飞机上起不来**
    #   （第一手症状：`启动失败：参数 tau_s 要是正的有限数，收到 0.0`、`returncode=2`、
    #   一帧都不发 —— 与"参数名写错/话题没通"在现场分不开）。
    #   区间用的是 `node_core` 里那一对常量 `[TAU_MIN_S, TAU_MAX_S)`，与 `preflight_check`
    #   的「外推 τ 在合理区间」**同源**（不再是两处各写一遍的字面量）。
    #   ⚠️ 这两处**必须同口径** —— 本票第一版这里写的是 `<= 0.2`，与自检差了一个端点，
    #     于是 `τ = 0.2` 会"自检红、节点绿"，而在台架上这两者的区别只是**先看到哪一个**。
    #     ★ 顺带记一笔：改之前这两处**本来**就在这一端点上不一致（旧代码是 `> 0.2` 才拒、
    #     自检是 `< 0.2`）⟹ `τ = 0.2` 当时就已经"自检红、节点绿"，只是没人拿这个值得罪过。
    #   负 τ 仍然拒 —— 那是**反向外推**（拿"还没走到"的状态喂控制器），比不补偿更危险。
    if not (math.isfinite(p.tau_s) and TAU_MIN_S <= p.tau_s < TAU_MAX_S):
        raise SystemExit(
            f"参数 tau_s 要在 [{TAU_MIN_S}, {TAU_MAX_S}) s（0 = 不补偿，是首飞的冻结值；"
            f"负数是反向外推，≥ {TAU_MAX_S} 不是执行延迟的量级），收到 {p.tau_s!r}")
    for name in ("warmup_cycles", "log_every", "fallback_hold_frames",
                 "inject_solve_fail_after_frames"):
        if int(getattr(p, name)) < 0:
            raise SystemExit(f"参数 {name} 不能为负，收到 {getattr(p, name)!r}")
    if not all(math.isfinite(v) for v in p.hold_offset):
        raise SystemExit(f"参考偏移必须是有限数，收到 {p.hold_offset!r}")
    # ★ 状态源（ticket 09）：**现在**就拒，而不是等到第一拍才发现"这个档谁也不认识"。
    #   默认值是 `mocap` ⟹ 不写这个参数的现有用法一个字都不变（默认档必须是老行为）。
    if p.state_source not in STATE_SOURCES:
        raise SystemExit(f"state_source={p.state_source!r} 不认识，只能是 {list(STATE_SOURCES)}"
                         "（mocap = 动捕 ENU/FLU；px4 = 飞控自己的 NED/FRD，见模块 docstring）")
    # 空话题名会让 `create_subscription` 抛在 **`MpcNode.__init__` 里**（那时 DDS 已经起了一半）
    if p.state_source == STATE_SOURCE_PX4 and not (p.px4_position_topic and p.px4_attitude_topic):
        raise SystemExit(f"A 档的两条状态话题都不能为空，收到 {p.px4_position_topic!r} / "
                         f"{p.px4_attitude_topic!r}")
    # ★ 参考源（ticket 17）—— 同样**现在**就拒，不等第一拍。
    if p.reference_source not in REFERENCE_SOURCES:
        raise SystemExit(f"reference_source={p.reference_source!r} 不认识，只能是 "
                         f"{list(REFERENCE_SOURCES)}（hold = 参数化悬停锚点；"
                         "traj = /traj/reference，见模块 docstring）")
    # ☠️ `traj` 档下 `hold_offset` **必须**为 0：那时目标点是"锚点 + 轨迹给的相对量"，
    #    再叠一个偏移就是一个**没人能说清它意味着什么**的平移 —— 它既不是轨迹的一部分，
    #    也不是锚点的一部分，而在话题面上它表现为"飞机整体偏了一截"，看着像轨迹算错了。
    #    ⚠️ 这条**不是**形式主义：`hold_offset_z` 在 A 档悬停那几场里一直是个非 0 的值，
    #      切档时忘掉它是最自然的操作失误，而后果是**静默的常值位置偏差**。
    if p.reference_source == REFERENCE_SOURCE_TRAJ and any(float(v) != 0.0 for v in p.hold_offset):
        raise SystemExit(
            f"reference_source=traj 时 hold_offset 必须全 0，收到 {p.hold_offset!r}"
            "（traj 档的目标点 = 锚点 + /traj/reference 给的相对量，见 ticket 17）")


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
