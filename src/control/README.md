# control —— 自主飞行控制节点

把 PX4 拉进 OFFBOARD，并以 50 Hz 持续流「位置 + yaw」setpoint。

> ⚠️ **本包目前只实现到 `STANDBY`**（ticket 05）：已在流 setpoint、已切 OFFBOARD，
> 但**飞机未解锁**、setpoint 内容是"位置 = 当前位姿"（原地保持）⟹ **它不会让飞机动**。
> 起飞/悬停/降落归 ticket 08，轨迹归 ticket 07，失效路径归 ticket 06。

## ★ 运行纪律（比代码重要）

1. **启动顺序**：`mavros` → `vrpn_listener` → `mocap` → 摆好飞机、静止 → 本节点。
   顺序错了**不会报错**，只会停在 `INIT`，并从日志里逐条告诉你缺什么
   （话题、静默多久、话题图上有几个发布者、QoS 是什么）。
2. **`/mavros/state` 是 1 Hz，不是"节点卡了"**。实测 1.000 Hz（min 0.998 / max 1.001），
   根源是 mavros 只在收到 MAVLink HEARTBEAT 时才发这个话题，而 HEARTBEAT 是 PX4 的 1 Hz 定频流。
   ⟹ 本节点对它的超时判据是 **2.5 s**（不是 `px4ctrl` 的 0.5 s，那样会每秒都在报"飞控失联"）；
   代价是**解锁上升沿最多晚 1 s 被检出**（ticket 08 起飞前要把这条算进去）。
3. **退出 OFFBOARD 有两种方式**：停本节点（断流超过 `COM_OF_LOSS_T` 后 PX4 自行退出），
   或遥控器接管（ticket 02 的通道分配）。**不要用 QGC 抢模式** —— 那会让"谁在控制"变得说不清。
4. **`armed` 上升沿 = 异常**（本张没有解锁路径）。看到 ERROR 日志就去查是谁解的锁，
   不要"重试一下"。

## 两个正交维度（不是扁平枚举）

| 维度 | 取值 | 含义 |
|---|---|---|
| **控制权** `Authority` | `MANUAL` / `AUTO` | 谁在控制飞机 |
| **任务态** `Phase` | `INIT` / `STANDBY` / `TAKEOFF` / `HOVER` / `TRACK` / `LAND` | 程序在干什么 |

★ **`AUTO` 的定义是"本项目正在发会驱动飞机的指令（已解锁的 offboard）"**。
所以 `STANDBY` 虽然已经切进 OFFBOARD 模式，但飞机未解锁 ⟹ `/control/state` 仍是 `MANUAL/STANDBY`。
这条定义让"切模式"与"交出控制权"两件事不被混为一谈。

`TAKEOFF/HOVER/TRACK/LAND` 本张只在枚举里占位，**不在转移表里** —— 本张任何输入组合都到不了它们。

`/control/state` 发 `"<控制权>/<任务态>"`（如 `MANUAL/STANDBY`），**transient_local**
⟹ 晚订阅者立刻拿到当前值；另外每 `diag_period_s` 心跳重发一次。
观察：`ros2 topic echo /control/state`

## 数据流

```
/mavros/state ─────────────┐
/mavros/local_position/pose ┤
/mocap/pose_enu ────────────┼─▶ 守卫 ─▶ 状态机 ─▶ /mavros/setpoint_raw/local（PositionTarget）
/mavros/rc/in、/mavros/battery ┘        └─▶ /control/state（"<控制权>/<任务态>"）
```

| 话题 | 方向 | 类型 | QoS | 备注 |
|---|---|---|---|---|
| `/mavros/state` | 订阅 | `mavros_msgs/State` | 可靠 + **transient_local** | 上游 1 Hz |
| `/mavros/local_position/pose` | 订阅 | `geometry_msgs/PoseStamped` | **best_effort** | 上游 ~30 Hz |
| `/mavros/rc/in` | 订阅 | `mavros_msgs/RCIn` | 可靠 | 本张只订阅、**尚不参与判据**（超时判据属 ticket 06） |
| `/mavros/battery` | 订阅 | `sensor_msgs/BatteryState` | **best_effort** | 本张只订阅、**尚不参与判据**（超时判据属 ticket 09） |
| `/mocap/pose_enu` | 订阅 | `geometry_msgs/PoseStamped` | 可靠 | 来自 `mocap` 节点；**是守卫的输入**（新鲜度决定发不发 setpoint） |
| `/mavros/setpoint_raw/local` | **发布** | `mavros_msgs/PositionTarget` | 可靠 | 唯一的控制输出 |
| `/control/state` | **发布** | `std_msgs/String` | 可靠 + transient_local | `"MANUAL/STANDBY"` |

★★ **QoS 必须与真 mavros 逐条对齐，而且错了不会报错、只会"永远收不到"**：
`local_position` / `battery` 在真 mavros 上是 `SensorDataQoS()`（best_effort），
按默认（可靠）订阅会**静默不匹配**。依据与实测见 `include/control/px4_interface.hpp`
与 `src/bench/README.md`；`bench` 包有一条专门的测试盯着这件事。

## setpoint 的三个硬指标

| 字段 | 值 | 依据 |
|---|---|---|
| `type_mask` | **2552** | IGNORE_VX/VY/VZ + IGNORE_AFX/AFY/AFZ + IGNORE_YAW_RATE ⟹ 忽略速度/加速度/yaw_rate，**保留位置 + yaw**。不含 FORCE(512) 与 IGNORE_YAW(1024) |
| `coordinate_frame` | **1**（`FRAME_LOCAL_NED`） | ⚠️ **枚举名有误导性** —— 见下 |
| `header.stamp` | `now()` | ⚠️ 见下 |

⚠️⚠️ **`FRAME_LOCAL_NED` 不代表"你填的值是 NED"。** mavros 对入向的 `FRAME_LOCAL_NED` setpoint 做的是
**ENU→NED 转换**（`setpoint_raw.cpp:231` `transform_frame_enu_ned(position)`）⟹
**我们填进 `PositionTarget` 的是 ROS/ENU 口径**：

- **`position`：x 东、y 北、`z` 向上为正**（不是"向下为正"！照 NED 去翻 z 的符号会得到一架往地里钻的飞机）
- **`yaw`：从 ENU 的 +x 轴逆时针**，实测换算 **`yaw_ned = π/2 − yaw_enu`**
  （`setpoint_raw.cpp:235`，按 `frame_tf.hpp:180` + `ftf_frame_conversions.cpp:33/40` 逐字复算）
  ⟹ **本项目填的 `yaw = 0`，PX4 收到的是 +90°，即"机头朝东"**（不是"保持当前机头"）

★ 也正因如此，把 `/mavros/local_position/pose` 原样转给 setpoint 是**正确的**：
mavros 出向已做 NED→ENU（`local_position.cpp:157`），进向再做 ENU→NED ⟹ 一进一出恰好抵消，
PX4 收到的是它自己报的那个位置。**这不是巧合，是"两边都在 ROS 口径里"的必然结果。**

⚠️ 本张飞机未解锁 ⟹ 上面这些**不会让飞机真动**；但 ticket 08 定"起飞朝向"之前必须处理 `yaw`——
**不要留着默认 0 当"保持当前机头"用**。

⚠️ **spec 里"发往 PX4 的消息用微秒时间戳"那条不适用于本路径**（那是 Phase 2 走 uXRCE-DDS 时的约定）：
mavros 把 `header.stamp` 换算成 MAVLink 的 `time_boot_ms`（**毫秒**，且会截断成 32 位，
`mavros/include/mavros/plugin.hpp:264-267`），而 **PX4 根本不读这个字段**
（`mavlink_receiver.cpp::handle_message_set_position_target_local_ned` 一律用 `hrt_absolute_time()`
自己打戳，v1.14.3）。所以这里的口径与 mavros 自带的 setpoint 插件一致：`header.stamp = now()`。

## 参数（`config/control.yaml`）

| 参数 | 默认 | 说明 |
|---|---|---|
| `setpoint_rate_hz` | `50.0` | setpoint 频率。PX4 要求进入 OFFBOARD 期间 ≥2 Hz |
| `offboard_prestream_s` | `1.0` | ★ 切 OFFBOARD 前 setpoint 必须先连续流多久。**断流 PX4 会自动退出 OFFBOARD**（`COM_OF_LOSS_T`，z0 现值 1.0 s）⟹ 这是前提不是优化 |
| `offboard_retry_s` | `1.0` | 切模式失败后的重试间隔（真正的成功判据是 `/mavros/state.mode` 变成 `OFFBOARD`） |
| `enter_offboard` | `true` | 自动切 OFFBOARD（参数可关）。⚠️ 关掉它真 PX4 就永远读不到这些 setpoint |
| `yaw` | `0.0` | setpoint 的 yaw（弧度，**ENU 口径**）。⚠️ 填 0 ⟹ PX4 收到 +90°（机头朝东），见上文 |
| `diag_period_s` | `1.0` | 诊断日志周期；也是 `/control/state` 的心跳周期 |
| `state_timeout_s` | **`2.5`** | ★ 不是 0.5 s —— 该话题实测 1 Hz |
| `local_pos_timeout_s` | `0.5` | 上游 ~30 Hz |
| `mocap_timeout_s` | `0.5` | 上游 ~196 Hz |
| `rc_timeout_s` | `0.5` | 上游 10 Hz 级。⚠️ **本张只记账、不做判据**（"超时 → 降级"属 ticket 06） |
| `battery_timeout_s` | `2.0` | 上游 1 Hz 级。⚠️ 同上，判据属 ticket 09 |
| `*_topic` / `*_service` | mavros 2.15.1 默认名 | 多机/台架复用不改代码 |

## 与其它 ticket 的边界

| 本张**做** | 本张**不做**（归谁） |
|---|---|
| 守卫判定 → `INIT`/`STANDBY` | `TAKEOFF`/`HOVER`/`LAND` 的判据（ticket 08）、轨迹（07）、失效降级（06） |
| 50 Hz 流 setpoint、切 OFFBOARD | **解锁**（ticket 08）、任何位移 |
| 输入静默 → 停发 setpoint | `LAND` / `RTL` 等降级动作（ticket 06） |
| `/control/state` | `/control/traj_progress`（ticket 07） |

## 构建与测试

```bash
colcon build --packages-up-to control     # ★ 不要用 --packages-select：control test_depend 了 bench
colcon test  --packages-up-to control
```

`control` 的测试分两层：

| 文件 | 接缝 | 断言面 | 回答的问题 |
|---|---|---|---|
| `test/test_control_standby.py` | **seam 1**（话题边界） | 只断言外部可观测的话题 | 飞机**发了什么** |
| `test/test_fsm.cpp` | **seam 2**（纯函数，零 ROS） | 零依赖，直接调 `fsm.hpp` | **是哪条守卫**放行的 |

seam 1 的 DUT 是**子进程**（`control_node` 可执行文件），输入来自**假 mavros + 脚本化动捕**；
夹具在 `src/bench/`，用法见 `src/bench/README.md`。
**为什么两层都要**：四条守卫全失效在话题上的症状**完全一样**（没有 setpoint），
seam 1 到那里就没有诊断分辨率了；而"缺项清单是哪几条、顺序如何"是纯逻辑，在 seam 2 测，
失败信息直接指着那条守卫。

本包刻意**不**加 `ament_lint_auto`（沿用 `mocap` 的先例：全仓库既有 172 条 lint 失败全来自
vendored 的 `vrpn_listener`，新包再加 lint 只是增加噪声）。

覆盖的判据（seam 1，`test/test_control_standby.py`）：四条守卫**逐条**缺失时不发 setpoint（每条都配了正控）、
外部解锁时停发、运行中 `/mavros/state` 静默后流在 `state_timeout_s` 内停止、前置流时长（两侧都断言）、
`enter_offboard = false` 只流不切、杀进程后流停止、以及冒烟用例对 setpoint 逐字段与频率的断言。

seam 2（`test/test_fsm.cpp`，12 例）：四条守卫**逐条**掀掉时缺项清单恰好是那一条（不是只断言"不放行"——
那样"某条守卫恒假"也会通过）、缺项顺序固定、全零 `Guards` 里"未解锁"成立（默认值的纪律）、
`STANDBY` 下控制权仍是 `MANUAL`、前置流时长两侧、已在 OFFBOARD 不再请求、**断流后前置流重新计时**。

⚠️ **没被自动化覆盖的一处**：`request_offboard()` 的重试节流（重复请求抑制、悬空请求丢弃、
重试间隔）。它是**输出侧的发送节奏**而非输入判据，只有"切模式恰好失败/悬空"时才走到 ——
真台上的表现是"日志说在重试，实际没重试"，改动这段请连带跑一次台架。

## ⏳ 还没做的验收项（需要有飞控空闲的窗口）

| 项 | 为什么还没做 |
|---|---|
| 真 PX4 被拉进 OFFBOARD | `COM_OF_LOSS_T` / `COM_OBL_RC_ACT` / 模式需求检查都是 **PX4 侧**行为，夹具不模拟。需要在**拆桨**台架上跑一次 |
| 杀节点 → PX4 在 `COM_OF_LOSS_T`（1.0 s）后自行退出 OFFBOARD | 同上；夹具这一半已由 `test_杀掉_DUT_之后_setpoint_流停止` 覆盖 |

做这两项时 **必须在台架上拆桨**，并按 `docs/control/phase1-implementation.md` §7 的顺序来。

## 目录

```
include/control/fsm.hpp           ★ 纯逻辑、零 ROS：两个正交维度 + 守卫清单 + 转移表（判据都在这里）
include/control/px4_interface.hpp mavros 边界：话题/服务名、超时口径、QoS 依据
src/px4_interface.cpp             订阅/发布/服务 + 每条输入的新鲜度
src/control_node.cpp              只做接线：参数、定时器、日志（★ 不要在这里加判断）
config/control.yaml               参数（含每一条的"为什么是这个值"）
launch/control.launch.py          一键启动（只起本节点）
test/test_fsm.cpp                 ★ seam 2：纯函数单测（零 ROS 依赖）
test/test_control_standby.py      ★ seam 1：子进程 DUT + 假 mavros，只断言话题
```
