# 动捕 / PX4 / Offboard 接口事实

> 本项目所有"外部系统与 PX4 怎么说话"的事实汇总。
> 每条都标注了**来源**与**置信度**。相关：[`../PROJECT.md`](../PROJECT.md) 第 3 节
> ⚠️ 本文件包含**已纠正的错误认知**（第 7 节），请务必读——那些是真实踩过的坑。

---

## 1. 三层链路总览

```
动捕(Nokov)  ──VRPN/UDP:3883──▶  机载电脑  ──▶  PX4 飞控  ──DShot──▶  电机
                                  │
                       ┌──────────┴──────────┐
                       │                     │
                  安全网链路              控制链路
                  mavros → EKF2        MPC → 角速率环
```

**关键**：本项目把这两条链路**故意分开**。理由见 [`../PROJECT.md`](../PROJECT.md) 第 3.2 节——`(C)` 方案下 MPC 链路**不需要 PX4 有位置估计**，因此"动捕→EKF2"从"必需"降级为"安全网"。

---

## 2. 动捕 → PX4

> 📖 **本节只讲"是什么"（接口事实）。完整的"怎么搭"（依赖安装、坐标系标定、刚体创建、mavros 启动、`vrpn_relative_pose` 源码、开机自启、z0 适配清单）见 [`mocap-setup.md`](mocap-setup.md)。**

### 2.1 VRPN 输入

| 项 | 值 | 来源 |
|---|---|---|
| 协议 | **VRPN** | 用户笔记 |
| 端口 | **3883** | 标准 |
| 服务器 | Nokov 软件所在机器 `10.1.1.198` | 用户笔记 |
| 输出 | 刚体（rigid body）位姿 | — |

### 2.2 ★ 只有这两种消息能被 EKF2 使用

| 消息 | EKF2 能用吗 | 说明 |
|---|---|---|
| **`VISION_POSITION_ESTIMATE`** | ✅ **能** | mavros 的 `/mavros/vision_pose/pose` 就走这条 |
| `ODOMETRY`（且 `frame_id` 必须是 `MAV_FRAME_LOCAL_FRD`） | ✅ 能 | 高频版本 |
| **`ATT_POS_MOCAP`** | ❌ **不能** | ⚠️ **这是 LPE 时代的消息，EKF2 完全不用** |

> ⚠️ **`ATT_POS_MOCAP` 是一个广泛传播的坑。** 网上大量"PX4 接动捕"教程用这条消息，在 EKF2 时代（PX4 1.12+ 默认 `SYS_MC_EST_GROUP=2`）**发了等于没发**，飞控不会有任何反应也不报错。本项目 `SYS_MC_EST_GROUP = 2`（EKF2），因此**必须走 `VISION_POSITION_ESTIMATE`**。

### 2.3 用户已有的动捕经验（可直接复用）

来自用户笔记 `~/weisama-log/25.06.09_UAV_mocap/`（**本地仓库，不在本仓库内**，故不作链接）：

| 问题 | 现象 | 解决 |
|---|---|---|
| **延迟未知** | — | 从 PX4 日志里对比动捕四元数与 IMU 四元数，**实测出 50 ms**（`25.06.28_PX4_log.md`） |
| **飞行 10–60 s 后漂移** | QGC 里 `LOCAL_POSITION_NED` 明显失真，飞控不再采用动捕坐标 | **增大** `EKF2_EVP_NOISE` |
| **定点来回摆动** | 未触发上一条 | **减小** `EKF2_EVP_NOISE`，让飞控更信任动捕 |
| **动捕上电返回 (0,0,0)** | 刚体未识别时 | 自写 `vrpn_relative_pose` 节点，**记录初始位姿做相对转发** |
| 延迟补偿 | — | 正确的 `EKF2_EV_DELAY` 能**降低坐标噪声方差**，从而允许更小的 `EKF2_EVP_NOISE` |

> 📌 **第 4 条的成本最低、收益最高**：把延迟标对了，噪声参数就能往下压，控制精度直接受益。**建议 z0 上先做这件事。**

### 2.4 中间件选择：mavros

**决策**：动捕 → PX4 走 **mavros**（`/mavros/vision_pose/pose`），不走 uXRCE-DDS。

**理由**：用户已在旧机上跑通，且**所有调参经验（50 ms 延迟、EVP_NOISE 调法）都是在这条链路上积累的**。换成 uXRCE-DDS 等于把已验证的部分重新冒险。

**代价**：一个系统里同时跑 mavros 和 uXRCE-DDS Agent。**两者不冲突**——各连 PX4 的不同 UART 口（`MAV_0`/TELEM1 与 `MAV_1`/TELEM2）。

---

## 3. PX4 → 电机：Offboard 的五个层级

这是本项目架构选择的核心依据。

### 3.1 ★ 各级所需的"状态估计"（决定架构解耦能力）

PX4 官方 Offboard 文档明确列出每种 setpoint 需要什么估计：

| 控制量 | 所需状态估计 | 对应消息 | 本项目 |
|---|---|---|---|
| position / velocity / acceleration | **需要** position & velocity | `TrajectorySetpoint` | ❌ (A) 方案，不用 |
| attitude | **none** | `VehicleAttitudeSetpoint` | ⚠️ (B) 方案备选 |
| **body_rate** | **none** | **`VehicleRatesSetpoint`** | ✅ **(C) 方案** |
| thrust_and_torque | **none** | `VehicleThrustSetpoint` + `VehicleTorqueSetpoint` | — |
| direct_actuator | **none** | `ActuatorMotors` / `ActuatorServos` | ⚠️ (D) 方案，第二阶段 |

> ★★ **"none" 这一列是本项目最重要的架构洞察。**
> 选 `(C)` 后，**MPC 这条链完全不依赖 PX4 的位置估计**——动捕位姿直给 MPC，MPC 出 ω+T 给 PX4，PX4 有没有定位都不影响 MPC 工作。
> → 两条链路可以**独立调试、独立失败**。

### 3.2 全部相关消息

| 消息 | 关键字段 | 备注 |
|---|---|---|
| `OffboardControlMode` | 5 个 bool：`position` / `velocity` / `acceleration` / `attitude` / `body_rate` / `thrust_and_torque` / `direct_actuator` | **心跳**，见 3.3 |
| `TrajectorySetpoint` | `position[3]`, `velocity[3]`, `acceleration[3]`, `jerk[3]`, `yaw`, `yawspeed` | ⚠️ **没有 `thrust` 字段** |
| `VehicleAttitudeSetpoint` | `q_d[4]`（四元数）, **`thrust_body[3]`** | (B) 方案 |
| `VehicleRatesSetpoint` | **`roll`/`pitch`/`yaw` (rad/s)**, **`thrust`** | ✅ (C) 方案 |
| `VehicleThrustSetpoint` | `xyz[3]` | |
| `VehicleTorqueSetpoint` | `xyz[3]` | |
| `ActuatorMotors` | `control[12]`（归一化） | (D) 方案 |

### 3.3 ⚠️ 两个必须记住的接口约定

1. **心跳必须持续。**
   `OffboardControlMode` 必须以 **≥ 2 Hz** 持续发布，**且在切换进 Offboard 模式前已持续 ≥ 1 s**。断掉就会退出 Offboard。

2. **★ 推力是 `[-1, 1]` 归一化，不是 `0–1`。**
   这一点极其容易搞错：
   - 范围是 **`[-1, 1]`**（负数 = 反向推力）
   - **不是** `0–1`
   - **不是**"相对悬停归一化"
   - **不是**牛顿

   > 数值约定错了，MPC 算得再对，飞机也不会飞。**这是实测前必须写在纸上的一条。**

### 3.4 ⚠️ `(D)` 方案的安全代价

`ActuatorMotors` 会**绕过 PX4 的 `control_allocator`**。后果：

- PX4 的**动力饱和保护、失控保护、混控校验全部失效**
- 安全网只剩 `COM_RC_OVERRIDE` 打杆接管（而它本身依赖遥控器链路是通的——见 [`params-analysis.md`](params-analysis.md) 3.9）

→ **这解释了为什么 `(C)` 是主线而不是 `(D)`。** `(D)` 应该只在 `(C) + INDI` 稳定之后，作为终点而非起点。

---

## 4. MPC → PX4：中间件选择

### 4.1 三条路线的现状

| 方案 | 状态 | 评价 |
|---|---|---|
| **uXRCE-DDS** | ✅ **PX4 v1.14+ 的默认 ROS2 中间件** | ✅ 推荐 |
| **mavros** | ✅ 长期存在，ROS1/ROS2 都有 | ⚠️ 备选（见 4.3） |
| **`px4_ros_com`** | ❌ **v1.14 起已废弃**，仓库里只剩示例 | ❌ **不要用** |

> ⚠️ **已纠正的错误认知**：`px4_ros_com` 在 v1.14 之前是标准的 ROS2 桥（走 FastRTPS）。**从 v1.14 起被 uXRCE-DDS 取代**，现在仓库里保留的部分只是示例代码。任何 2023 年之前的教程都会过时地推荐它。

### 4.2 uXRCE-DDS 话题命名

| 前缀 | 方向 |
|---|---|
| `/fmu/in/*` | ROS2 → PX4（**发指令**） |
| `/fmu/out/*` | PX4 → ROS2（**读状态**） |

`(C)` 方案要用到的：

```
/fmu/in/offboard_control_mode     ← 心跳（≥2 Hz）
/fmu/in/vehicle_rates_setpoint    ← ★ MPC 的输出（ω_des + T_des）
/fmu/out/vehicle_odometry         ← 状态（可选，MPC 也可直接用动捕）
/fmu/out/vehicle_status           ← 判断是否进入 Offboard
/fmu/out/vehicle_attitude         ← 姿态
/fmu/out/vehicle_local_position   ← 位置/速度
```

### 4.3 ⚠️ 备选：全走 mavros

有一条**完全可行但代码更啰嗦**的路线：用 mavros 的 `setpoint_raw/attitude` 消息，其中也带 `body_rate` 字段。

| | 混合（mavros + uXRCE-DDS） | 全 mavros |
|---|---|---|
| 中间件数量 | 2 个 | **1 个** |
| 代码量 | **少**（有现成蓝本 `DISCOWER/px4-mpc`） | 多（要自己设计） |
| 已验证程度 | uXRCE-DDS 部分**未验证** | 动捕部分已验证，指令部分未验证 |
| 关键依据 | `px4-mpc` 用的是 `/fmu/in/vehicle_rates_setpoint`，**可直接对照** | |

**当前倾向：混合。** 理由：`px4-mpc` 是一个运行中的蓝本，"少写代码"在这个阶段比"单一中间件"更值钱。**但这仍是开放决策**（见 [`../PROJECT.md`](../PROJECT.md) O6）。

### 4.4 ★ `px4_msgs` 版本必须严格匹配

**⚠️ 在 PX4 v1.15 及以前，`px4_msgs` 与固件版本不匹配会导致静默失败**——话题名或字段错位**不会报错**，只是消息发出去没人收 / 收到的是垃圾。

**核对方法**（本项目脚本 `tools/px4-sitl/check_msg_compat.py`，跑法见 [`../control/px4-sitl-baseline.md`](../control/px4-sitl-baseline.md) §4）：

```bash
python3 tools/px4-sitl/check_msg_compat.py \
    --px4-tree <PX4-Autopilot 目录> --px4-msgs <你的 px4_msgs 目录> \
    --interface-lib <px4-ros2-interface-lib 目录>
```

☠️ **原写「（PX4 官方脚本）`./scripts/check-message-compatibility.py -v <你的 px4_msgs 目录> <PX4-Autopilot 目录>`」—— 该写法已被推翻**
—— ★ **那个路径是错的，而且照着改也还是会错**：该脚本**不在 PX4-Autopilot 里**（`Tools/` 下只有
`generate_msg_docs.py` / `px_generate_uorb_topic_files.py` / `px_generate_uorb_topic_helper.py`，仓库根**没有 `scripts/` 目录**；
v1.14.3 / v1.15.0 / v1.16.0 / fork `5ba2e91f` / `px4_msgs` 逐棵树查过，**全无**）。它的真身在
**`auterion/px4-ros2-interface-lib`** 的 `scripts/` 下 —— 但**直接跑它对本项目会报假 FAILED**：
它的待查类型清单取自**它自己那个仓库**（`message_compatibility_check.hpp`），而该仓**没有 `release/1.14` 分支**，
拿它查 v1.14 这一对会报 **20 个 v1.14 里根本不存在的类型**为「缺失」，报错原文里**没有** `Some files differ` 那一段
（即：**零个字段不符**）。上面那条 `check_msg_compat.py` **复用官方那份 `message_hash` 算法**，但把类型换成两侧
`msg/` 的**并集（184 个全覆盖）**，**严格强于官方脚本的子集检查**。权威出处见
[`../control/px4-sitl-baseline.md`](../control/px4-sitl-baseline.md) §2.3。

**必须选的版本**：**`release/1.14` @ `ffb6e80e`**（★ 2026-09-21 用户签字）。
它对应的是 **z0 固件真身** = `Minderring/PX4-Autopilot` @ `4a0e65f2`（**v1.14.3 系**的 MicoAir **厂商构建**，
不是上游 v1.14.3 原版；两者 `msg/` 已实测逐字节相同）。**不要用 main 分支**，
也**不要从任何 fork 生成** —— 见 [`refs-manifest.md`](refs-manifest.md) §2.5(d) / §2.6 执行版。

---

## 5. 坐标系（★ 符号错误的头号来源）

| 坐标系 | 约定 | 用在哪 |
|---|---|---|
| **PX4 机体系** | **FRD**：x 前 / y 右 / **z 下** | 所有 PX4 参数、`thrust_body` |
| **PX4 世界系** | **NED**：北-东-**地** | `vehicle_odometry` 等 |
| **ROS/动捕常用** | **ENU**：东-北-**天** | `PoseStamped`、mavros 的 vision_pose |
| **动捕软件自身** | 各自定义，需现场标定 | Nokov |

**关键变换**：

- ENU ↔ NED：**交换 x/y 并翻转 z**（不是简单取反）
- FRD ↔ FLU：**翻转 y 和 z**

> ⚠️ **本项目最容易出错的地方**：`VehicleRatesSetpoint` 的 `roll`/`pitch`/`yaw` 是 **FRD 机体系角速率**。如果 MPC 内部用的是 FLU 或者从 ENU 动捕数据推姿态，**横滚和偏航的符号会错**。
>
> **建议**：在文档里**用文字写死符号约定**（不只用公式），并在实飞前用一个"打正杆看飞机往哪边滚"的测试确认。这是用户已有经验里的老坑。

### 5.1 ★★ `mavros` 在哪些话题上帮你转、往哪个方向转（已读源码）

**`mavros` 的坐标转换只发生在它的输入/输出边界上，且两个方向都有**——**容易记反**：

| 话题 | 方向 | mavros 做什么 | 源码依据 |
|---|---|---|---|
| `/mavros/vision_pose/pose`（**我们 → PX4**） | 入 | **ENU→NED**（位置）、**ENU→NED + FLU→FRD**（姿态） | `mavros_extras/src/plugins/vision_pose_estimate.cpp` `send_vision_estimate()`：`ftf::transform_frame_enu_ned(tr.translation())` + `ftf::transform_orientation_enu_ned(ftf::transform_orientation_baselink_aircraft(q))` |
| `/mavros/odometry/out`（**PX4 → 我们**） | 出 | **NED→ENU** | `mavros/src/plugins/odometry.cpp` |

> ⚠️ **已纠正的错误认知（来自项目自身的旧笔记）**：早先笔记写的是"mavros 会把 `/mavros/odometry/out` 的 **ENU 转成 NED** 给飞控"。**方向写反了**——那是 **PX4→ROS 的出方向（NED→ENU）**，不是入方向。
>
> **为什么这个方向很重要**：因为 `/mavros/vision_pose/pose` 走的是**入方向（ENU→NED）**，所以**我们的动捕节点应该发原生 ENU/FLU 值，不要自己先转一遍**。自己转了 mavros 会再转一次 → **转两次**，症状是"能飞但方向不对/持续飘"。

**★ `send_vision_estimate()` 还有一个隐藏行为**：

```cpp
if (last_transform_stamp == stamp) { /* dropped */ return; }
```

**时间戳不前进的消息会被静默丢弃。** → 这是"**不要给动捕数据一个固定时间戳**"的第二个理由（第一个是时钟域问题，见 [`mocap-setup.md`](mocap-setup.md) 6.2）。

---

## 6. 时延预算

| 环节 | 量级 | 来源 |
|---|---|---|
| 动捕采集 + 解算 | 未知（Nokov 内部） | — |
| VRPN 网络传输 | ~ms | — |
| **动捕→IMU 相对延迟** | **实测 50 ms** | 用户 `25.06.28_PX4_log.md` |
| `EKF2_EV_DELAY` 设置 | **50.0 ms** | `z0.params` ✅ 与实测吻合 |
| uXRCE-DDS 传输 | ~1 ms | — |
| PX4 角速率环 | 1 kHz → **1 ms** | — |
| **MPC 求解** | **≤ 10 ms 预算**（100 Hz）；★ **实测纯求解 max `3.58 ms`** | `phase2-implementation.md` §5 的 P3 行 / D6（2026-09-28） |
| **动捕时间戳抖动** | ✅ **~2–5 ms** | ★ 已更正，见下 |

> ⚠️ **已更正的错误认知**：本文档早先版本把动捕时间戳抖动记为 **±10–100 ms**，并据此推断"真实最坏延迟可能是 150 ms"。**该数字是错的。**
>
> 第一手证据（`vrpn_listener.cpp` 第 37–40 行的源码注释）：抖动实际只有 **~2–5 ms**，因为 `stamp = now()` 用的是 **ROS 接收时刻**、与 IMU/mavros **同一时钟域**。**用 `msg_time` 才会出大问题**——动捕主机与 Jetson 时钟差 **~4 秒**。
>
> → **正确的最坏端到端延迟 ≈ 50 + 5 ≈ 55 ms**，不是 150 ms。
> → 详细论证与源码原文见 [`mocap-setup.md`](mocap-setup.md) **6.2**。

**含义**：

- 动捕链路 50 ms 的延迟**是 MPC 外部状态输入的固有延迟**。MPC 若不做延迟补偿，这 50 ms 会直接变成跟踪误差。
- **MPC 的预测时域天然能吃下这个延迟**——把 50 ms 的延迟显式建模成"状态滞后 N 步"，是 NMPC 相对 PID 的一个真实优势。
- → 📌 **值得做的一个实验**：在 MPC 里显式补偿 50 ms 延迟，对比不补偿的效果。这是一个有论文价值的对比。
- ✅ **时延预算是干净的**：唯一的显著延迟就是那 50 ms 本身，时间戳抖动可以忽略。**不需要为抖动另做任何事。**

---

## 7. ⚠️ 已纠正的错误认知（踩过的坑）

调查过程中产生过几个错误结论，**记录下来避免重复踩**：

| # | 错误认知 | 纠正 |
|---|---|---|
| 1 | `noetic` 容器是 Gazebo 仿真环境 | ❌ **错**。仿真环境是 `osrf/ros:humble-desktop-full`（容器 `ubuntu22`→`px4`）。`noetic` 容器是**为必须跑 ROS1 的开源包**准备的（Swarm-LIO2 / MID360）。 |
| 2 | (B) 方案用 `TrajectorySetpoint` + `type_mask` 推姿态和推力 | ❌ **错**。`TrajectorySetpoint` **根本没有 `thrust` 字段**。(B) 的正确消息是 `VehicleAttitudeSetpoint`（四元数 + `thrust_body`）。 |
| 3 | `px4_ros_com` 是本项目可用的中间件 | ❌ **错**。v1.14+ 已废弃，见 4.1。 |
| 4 | `CA_ROTORx_KM` 的正负号 = 螺旋桨物理转向 | ❌ **错**。那是 PX4 模板默认值（`CT=6.5`、`KM=±0.05` 对全部 12 个槽位一视同仁）。见 [`params-analysis.md`](params-analysis.md) 3.3。 |
| 5 | 用 HTTPS / GitHub API 探测 private 仓库，返回 404 就认为仓库不存在 | ❌ **错**。GitHub 对未授权 private 仓库**故意返回 404**。有 SSH key 时应用 `git ls-remote`。详见 [`../LOCAL-NOTES.md`](../LOCAL-NOTES.md) 第 3 节。 |
| 6 | "mavros 会把 `/mavros/odometry/out` 的 **ENU 转成 NED** 给飞控" | ❌ **方向写反了**。那是 **PX4→ROS 的出方向（NED→ENU）**。`/mavros/vision_pose/pose` 走的是**入方向（ENU→NED）**。★ **搞反的后果**：动捕节点自己先转一遍，mavros 再转一遍 → **转两次**，症状是"能飞但方向不对/持续飘"。源码依据见 5.1 |
| 7 | "给动捕数据一个固定/恒定时间戳" | ❌ **错**。`VisionPoseEstimatePlugin::send_vision_estimate()` 里 `if (last_transform_stamp == stamp) return;` —— **时间戳不前进的消息会被静默丢弃**。这是"不要动 `stamp`"的第二个理由 |

---

## 8. 实飞前检查清单

按 `(C)` 方案，上电到 Offboard 的完整链路自检：

- [ ] 动捕软件里能稳定识别刚体，坐标在 QGC 里能看到且不跳
- [ ] `listener vehicle_visual_odometry` 有数据，`EKF2_EV_CTRL = 11`
- [ ] QGC 里能切 **Position 模式**并稳定悬停（**这一步必须先过**——[_PROJECT.md_](../PROJECT.md) 里程碑 1）
- [ ] `COM_RC_OVERRIDE = 3` 生效：打杆能踢出 Offboard（**实飞前用低油门验证一次**）
- [ ] 遥控器链路确认（`COM_RC_IN_MODE` 含义已核实，见 params-analysis 3.9）
- [ ] `UXRCE_DDS_CFG` 已打开，`ros2 topic list | grep /fmu` 能看到话题
- [ ] `px4_msgs` 钉在 **`release/1.14` @ `ffb6e80e`**（与真身 `4a0e65f2` 的 `msg/` blob 全同；跑过 `tools/px4-sitl/check_msg_compat.py`，见 [`../control/px4-sitl-baseline.md`](../control/px4-sitl-baseline.md) §4）。☠️ 原记「跑过官方 `check-message-compatibility.py`」**作废**：那份**不在 PX4-Autopilot 里**，真身在 `auterion/px4-ros2-interface-lib`，**直接跑会给假 FAILED**，见 [`../control/px4-sitl-baseline.md`](../control/px4-sitl-baseline.md) §2.3
- [ ] `/fmu/in/offboard_control_mode` 心跳 ≥ 2 Hz 且持续 ≥ 1 s
- [ ] **推力数值范围写死为 `[-1, 1]`**，在代码里有断言
- [ ] 角速率符号（FRD）经实机验证
- [ ] 角速率不超过 `MC_ROLLRATE_MAX = 220 °/s`
- [ ] 电池监测链路确认（`listener battery_status`，见 params-analysis 3.5）
