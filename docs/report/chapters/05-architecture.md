# 第 5 章 系统架构与实验平台

本章回答三个问题：z0 的自主飞行栈分成哪几层，层与层之间用什么接口连接；同一条动捕数据为什么走两条路径，各自服务谁；哪些话题、坐标系与安全边界构成了两条控制链——offboard 位置控制与模型预测控制（MPC）。第 3 章讲零件与物理参数，第 6 章讲代码结构，第 7 章讲起链、遥控与状态机；本章只讲它们之间的连接关系与数据流，不重复各章的细节。

按操作者裁定，本章不画架构图；结构用文字与表描述，话题表是本章的主干。

---

## 5.1 符号与缩写

表 5-1 列出本章反复出现的缩写与符号。物理量的完整符号表在第 3 章；本章只补充与接口、频率、坐标系有关的几个。

**表 5-1　本章缩写与符号**

| 缩写/符号 | 含义 |
|---|---|
| ENU / NED | East-North-Up / North-East-Down，分别为 ROS 侧与 PX4 侧的世界系 |
| FLU / FRD | Forward-Left-Up / Forward-Right-Down，分别为 ROS 侧与 PX4 侧的机体系 |
| VRPN | Virtual Reality Peripheral Network，动捕数据的传输协议 |
| EKF2 | PX4 的扩展卡尔曼滤波器，融合 IMU、气压计与外部视觉 |
| CTBR | collective thrust + body rate，总推力 + 机体三轴角速率 |
| OCP | optimal control problem，最优控制问题 |
| SQP_RTI | sequential quadratic programming，real-time iteration |
| HPIPM | acados 使用的高性能内点法 QP 求解器 |
| DShot | 数字电调协议 |
| CRSF | Crossfire，遥控接收机与飞控之间的串行协议 |
| QGC | QGroundControl，地面站 |
| VP / LP | 本章对 `/mavros/vision_pose/pose` 与 `/mavros/local_position/pose` 的简称 |
| $f$ | 话题频率，单位 Hz；表中所有频率都带测量时点 |
| $t_0$ | 时间零点，取 `/traj/start` 的包记录时间戳 |
| $\tau$ | 端到端延迟，单位 s；本报告只在第 3、9 章给数值 |
| $T$ | 总推力，单位 N |
| $\boldsymbol{\omega}=(\omega_x,\omega_y,\omega_z)$ | 机体三轴角速率，单位 rad/s |

本章统一用 ENU/FLU 指 ROS 侧，用 NED/FRD 指 PX4 侧；出现“机体角速率”时，默认是 FLU 口径，除非明说 PX4 侧。

<big>**一句话总结：本章统一用 ENU/FLU 指 ROS 侧、NED/FRD 指 PX4 侧，VP 与 LP 是两条位姿话题的简称。**</big>

---

## 5.2 三层架构

z0 的飞行栈按**失效域**分成三层：定位层在地面，机载层与飞控层都在飞机上。表 5-2 给出每层的组成、输入输出，以及这一层失效时最先失去什么。

**表 5-2　三层架构**

| 层 | 主要组成 | 输入 | 输出 | 失效时先失去什么 |
|---|---|---|---|---|
| **定位层** | Nokov 光学动捕相机、Nokov 软件、VRPN 服务（`10.1.1.198:3883`） | 机体上 5 颗反光球的图像 | 刚体位姿与速度（VRPN） | 两条动捕路径同时失去状态；飞控失去视觉位置；只能遥控或降落 |
| **机载层** | Jetson Orin NX 8GB；ROS 2 Humble；`vrpn_listener`、`mocap`、`traj`、`control`、`nmpc`、`mavros` | VRPN/UDP；`/mavros/*` 的飞控状态与遥控通道 | setpoint（位置，或推力 + 角速率）、`/control/*`、录制包 | 控制链停止发帧；PX4 在 1 s 后退出 Offboard；遥控仍然可用 |
| **飞控层** | NxtPX4v2；PX4 1.14.3 厂商构建；EKF2、位置/姿态/角速率环、控制分配、失效保护、DShot | MAVLink setpoint、遥控、IMU 与气压计 | 四路电机指令 | 飞机失去全部控制；最后手段是 ch8 杀车或 PX4 自动降落 |

三层是按失效后果切的，不是按机箱切的。飞控与 Jetson 都装在飞机上，动捕主机在地面；把它们并列成三层，是因为它们的失效后果不同：定位层失效，飞机失去外部位置；机载层失效，飞机失去任务指令；飞控层失效，飞机失去全部控制。

层间只有两条物理连接：动捕主机到 Jetson 的 WiFi，Jetson 到飞控的串口。机载层没有任何一条线能直接驱动电调；飞控层也不运行我们的控制律。这个边界是后面所有安全论证的前提。

机载层内部，`control` 是任务总管，`traj` 只发参考，`mpc_node` 只解优化问题，`mavros` 是唯一与飞控说话的进程。第 6 章按包展开这些职责，本章只按角色使用这些名字。

<big>**一句话总结：定位层给位姿，机载层做任务与计算，飞控层保留角速率环、混控与失效保护。**</big>

---

## 5.3 两条动捕路径

同一条动捕数据在 `mocap` 节点内部就分成了两路。表 5-3 给出两路的链路、频率与消费者。

**表 5-3　两条动捕路径**

| 路径 | 链路 | 频率（2026-10-17 实测） | 消费者 | 作用 |
|---|---|---|---|---|
| **控制路径** | `/vrpn/z0/pose`、`/vrpn/z0/twist` → `mocap` → `/mocap/pose_enu`、`/mocap/velocity_enu` | 181–193 Hz（n/span） | `mpc_node` | 模型预测控制的状态源；不经 EKF2 |
| **安全网路径** | 同一条 `mocap` 输出 → `/mavros/vision_pose/pose` → mavros → `VISION_POSITION_ESTIMATE` → EKF2 → `/mavros/local_position/pose` | 输入 181–193 Hz；EKF2 输出 29.9–30.1 Hz | `control`（offboard 档）、PX4 失效保护 | 给 PX4 一个位置估计；支撑遥控接管、Offboard 丢失保护与自动降落 |

`mocap` 节点对两路位姿做同样的预处理：刚体原点平移到 IMU 中心、减原点、解锁后只归零 yaw。两路的差别不在动捕数据本身，而在传输与估计环节：控制路径直发相对位姿，安全网路径多经过一次 mavros 转换和 EKF2 融合。

**为什么两条都要。** 控制路径的存在理由来自 (C) 级的定义：模型预测控制输出总推力与机体角速率，PX4 的 body_rate setpoint 不要求位置估计（[PROJECT.md §3.2](../../PROJECT.md)）。让控制器直连动捕，模型就不依赖 PX4 的位置估计，也不会被 EKF2 的 30 Hz 输出与融合延迟限制。

安全网路径不能省，因为 PX4 的失效保护、遥控接管、Offboard 丢失后的自动降落，都需要一个位置估计；offboard 档的位置控制更是直接跑在 EKF2 的局部系里。没有安全网，控制路径一断，PX4 连“往哪落”都不知道。

但这不是传感器冗余。Nokov 掉线、`mocap` 节点崩掉，两条路径一起失效；冗余的是传输与估计环节，不是动捕本身。

安全网也不是模型预测控制的备用状态源。控制路径断了，模型预测控制节点停发；PX4 按自己的失效保护回 Position，而不是让控制器改读 EKF2。这一点把两条链的失败域分开了：控制链可以独立调试、独立失效，安全网链只负责让飞控还能安全收场。

两条路径的原点由不同环节定义：`/mavros/local_position/pose` 是 EKF2 的局部 ENU，`/mocap/pose_enu` 是 `mocap` 节点的相对 ENU。报告不假设它们相同；第 9 章对齐数据时每个源各减自己的原点。

<big>**一句话总结：同一条动捕数据分两路：控制路径直连模型预测控制，安全网路径经 EKF2 支撑飞控失效保护。**</big>

---

## 5.4 硬件连接与软件栈

### 5.4.1 硬件连接

表 5-4 给出本章架构涉及的物理连接。零件型号与物理规格见第 3 章，这里只列“谁连谁、走什么协议、哪个参数锁住它”。

**表 5-4　硬件连接**

| 连接 | 近端 | 远端 | 物理/协议 | 关键参数 | 证据 |
|---|---|---|---|---|---|
| 动捕 ↔ Jetson | Nokov 主机 `10.1.1.198` | Jetson WiFi `10.1.1.114/24`（2026-09-21 读数） | VRPN over UDP，端口 3883 | 同一网段；不需要额外网卡 | [mocap-setup.md §9](../../reference/mocap-setup.md) |
| Jetson ↔ 飞控 | `/dev/ttyTHS1` @ 921600 | 飞控 TELEM2 / Onboard | MAVLink 串口 | `MAV_1_CONFIG=102`、`SER_TEL2_BAUD=921600`；mavros 独占 | [mocap-setup.md §5](../../reference/mocap-setup.md)、[params-changes.md §E1](../../reference/params-changes.md) |
| 飞控 ↔ 电调 | 4 路 DShot | 33A 四合一电调 | DShot | `DSHOT_MIN=0.055`、`DSHOT_3D_ENABLE=0`、`DSHOT_TEL_CFG=0`（遥测关） | [`docs/assets/z0.params`](../../assets/z0.params) |
| 遥控 ↔ 飞控 | RadioMaster Pocket（ELRS 2.4 GHz、EdgeTX） | 飞控遥控输入 | CRSF | `RC_INPUT_PROTO=6`；ch5 解锁、ch6 模式、ch8 杀车、ch9 开始 | [params-changes.md §H](../../reference/params-changes.md) |
| Jetson ↔ QGC（可选） | mavros `gcs_url` | 地面站 | UDP 转发 MAVLink | 只在需要 QGC 时启用；不参与控制 | [mocap-setup.md §8](../../reference/mocap-setup.md) |

动捕这条链路是 WiFi，不是专用以太网。Jetson 必须和动捕主机在同一网段；走校园网网关时 `ping 10.1.1.198` 会 100% 丢包（2026-09-21 实测）。现场换网后要重新确认本机地址与路由。

Jetson 到飞控只有一条串口。mavros 打开 `/dev/ttyTHS1` 后，第二个进程再打开会失败；这也是 uXRCE-DDS 没有位置可放的直接原因之一。

遥控链路不经过 Jetson：接收机直接接飞控。Jetson 死机时，ch6 切模式、ch8 杀车仍然有效。

电调遥测是关的（`DSHOT_TEL_CFG=0`），所以飞控拿不到电调的转速与电流回传。第 3 章的推力标定用的是整机测量与飞控日志，没有电调回传可用。

<big>**一句话总结：动捕经 WiFi 进 Jetson，Jetson 经串口连飞控，遥控走 CRSF，飞控再以 DShot 驱动电调。**</big>

### 5.4.2 软件栈

表 5-5 列出两条链上实际运行的软件与版本。版本以本报告证据产出时的实际值为准；没有一手记录的项标“未记录”，不按默认值补。

**表 5-5　软件栈**

| 组件 | 版本/标识 | 跑在哪 | 角色 |
|---|---|---|---|
| Ubuntu / L4T | 22.04.5 LTS / R36.5.0 / 内核 5.15.185-tegra | Jetson | 宿主系统 |
| ROS 2 | Humble | Jetson | 节点间通信 |
| mavros | 2.15.1 @ `22ae5b7`，源码构建在 `~/mavros_ws` | Jetson | 唯一的 MAVLink 桥 |
| acados | `v0.6.0-41-g00947ddc3`，SQP_RTI + HPIPM | Jetson，aarch64 原生 | 优化问题求解 |
| PX4 | 1.14.3 系厂商构建 @ `4a0e65f2` | 飞控 | EKF2、控制环、混控、失效保护 |
| VRPN 客户端 | `ros-humble-vrpn` 7.35.0 | Jetson | 接收动捕 |
| Nokov 软件 | 版本未记录；VRPN banner 自报 7.34 | 动捕主机 | VRPN 服务端 |
| Python / rclpy | 3.10 | Jetson | `mpc_node.py` 等节点 |

飞控固件不重刷（[第 3 章](03-hardware.md)、[phase4 票 04](../../../.scratch/phase4/issues/04-firmware-reflash.md)）。mavros 是源码构建，不在本仓库；acados 在 Jetson 上原生编译，运行时需要 `ACADOS_SOURCE_DIR` 与 `LD_LIBRARY_PATH`。这两条环境变量缺一个，`mpc_node` 在建求解器时就会因为找不到 `libacados.so` / `libhpipm.so` 而起不来。

Nokov 软件的版本没有一手记录，只有 VRPN banner 自报的 `7.34`；这是未测项，不写成已知。

`px4_msgs` 在仓库里出现在 SITL 与台架路径上，但真机链路不使用它；下一节说明原因。

<big>**一句话总结：机载侧是 ROS 2 Humble 上的 mavros 与 acados，飞控侧是 PX4 1.14.3 厂商构建。**</big>

### 5.4.3 为什么没有 uXRCE-DDS

真机不用 uXRCE-DDS。2026-10-12 的定案（[phase4 票 13](../../../.scratch/phase4/issues/13-mavros-downlink-switch.md)）有三条第一手事实：`UXRCE_DDS_CFG=0`；`MAV_0_CONFIG=101`（TELEM1）、`MAV_1_CONFIG=102`（TELEM2 / Onboard，mavros 占着）、`MAV_2_CONFIG=0`；没有任何空闲串口分给 XRCE。飞控有 7 路 UART，但当前没有一路既已引出、又能从 MAVLink 手里让出来给 XRCE；再开一路是加硬件，不是改参数。

因此 (C) 级下行只有 mavros 一条：`/mavros/setpoint_raw/attitude`（`body_rate` + 标量 `thrust`）。没有独立的 `OffboardControlMode` 心跳流——PX4 收到 `SET_ATTITUDE_TARGET` 后会自己派生 Offboard 心跳，setpoint 流本身就是心跳；停流超过 `COM_OF_LOSS_T=1.0 s`，PX4 自行退出 Offboard。

代价是下行要经过 mavros 的消息转换与 QoS；收益是真机只有一条需要维护、需要排查的链路。Phase 2 的 SITL 证据曾经走 `px4_msgs` / uXRCE-DDS，那是另一个载体；第 9、10 章引用那批证据时要把这一点写明，不能把它当成真机链路的读数。

还有一个硬前置：`MAV_FWDEXTSP=1`。这个参数是 0 时，飞控的 `SET_ATTITUDE_TARGET` 处理器会把我们发的每一条 setpoint 静默丢掉，飞控侧没有日志；`nmpc/preflight_check.py` 已把它列为自检项。

<big>**一句话总结：uXRCE-DDS 在 z0 上没有可用串口，真机下行只有 mavros 一条链路。**</big>

---

## 5.5 话题表

表 5-6 是本章的主干：两条控制链经过的话题、类型、生产者与消费者、频率、坐标系，以及它何时存在。

**表 5-6　两条控制链的主干话题**

| 话题 | 类型 | 生产者 → 消费者 | 频率（2026-10-17 六份包实测） | 坐标系 / 语义 | 何时存在 |
|---|---|---|---|---|---|
| `/vrpn/z0/pose` | `geometry_msgs/PoseStamped` | `vrpn_listener` → `mocap` | 181–193 Hz（n/span） | 动捕原生 ENU/FLU；`frame_id="world"` | VRPN 数据到达后 |
| `/traj/select` | `std_msgs/UInt8` | `fly.py` / 手动 → `traj` | 事件（每次 1 帧） | 无量纲，1/2/3/4；0 = 清空 | 选轨迹时 |
| `/traj/start` | `std_msgs/Empty` | `control` → `traj` | 事件（每趟 1 帧） | 无载荷；包记录时间戳定义 $t_0$ | 进 TRACK 时 |
| `/traj/reference` | `mavros_msgs/PositionTarget` | `traj` → `control` | 50 Hz（中位 49.9–50.1；最大缺口 44–154 ms） | ROS ENU，相对轨迹原点；`frame_id="traj_origin"` 仅标注；`type_mask=2496` | select 且 start 后 |
| `/traj/progress` | `std_msgs/Float32` | `traj` → `control` | 50 Hz（与参考同帧） | 无量纲 0–1 | 同上 |
| `/control/state` | `std_msgs/String` | `control` → `fly.py` / `z0.py` / 录制 | 1 Hz 心跳；整包均值 1.05–1.40 Hz | `"<控制权>/<任务态>"`，如 `AUTO/TRACK` | 节点启动后 |
| `/control/target` | `mavros_msgs/PositionTarget` | `control` → `mpc_node` | 活跃期 50 Hz；整包 39.0–45.9 Hz（LAND 停发 7.2–7.9 s） | ROS ENU 绝对量；`type_mask=2496` | 仅 `output_mode=external` |
| `/mavros/setpoint_raw/local` | `mavros_msgs/PositionTarget` | `control` → `mavros` | 活跃期 50 Hz；整包 40.9–50.0 Hz（LAND 停发 6.6–7.1 s） | ROS ENU；`type_mask=2552`（位置 + yaw）；mavros 转 NED | 仅 offboard 档 |
| `/mavros/setpoint_raw/attitude` | `mavros_msgs/AttitudeTarget` | `mpc_node` → `mavros` | 98.8–99.8 Hz（n/span） | `body_rate` 为 FLU；`thrust` 为正号 0..1；`type_mask=128` | 仅模型预测控制档 |
| `/mavros/vision_pose/pose` | `geometry_msgs/PoseStamped` | `mocap` → `mavros` | 181–193 Hz（n/span） | ROS ENU/FLU；mavros 转 NED/FRD | `mocap` 启动后 |
| `/mocap/pose_enu` | `geometry_msgs/PoseStamped` | `mocap` → `mpc_node` / 录制 | 181–193 Hz（n/span） | 动捕原生 ENU/FLU，经外参、原点与 yaw 处理 | 同上 |
| `/mocap/velocity_enu` | `geometry_msgs/TwistStamped` | `mocap` → `mpc_node` / 录制 | 181–193 Hz（n/span） | ENU；角速度分量实测恒 0 | 同上 |
| `/mavros/local_position/pose` | `geometry_msgs/PoseStamped` | `mavros` → `control` | 29.9–30.1 Hz | ROS ENU（EKF2 输出） | mavros 连上后 |
| `/mavros/local_position/velocity_local` | `geometry_msgs/TwistStamped` | `mavros` → `control` | 29.9–30.0 Hz | ROS ENU | 同上 |
| `/mavros/state` | `mavros_msgs/State` | `mavros` → `control` / `mocap` / `mpc_node` / `z0.py` | 1.00 Hz | `connected` / `armed` / `mode` | 同上 |
| `/mavros/rc/in` | `mavros_msgs/RCIn` | `mavros` → `control` | 19.95–20.04 Hz | 原始 PWM 读数（µs）；归一化在 `control` 侧（`low_us=1000` / `high_us=2000`）；ch5 / ch6 / ch8 / ch9 | 同上 |

**频率是怎么量的。** 表里的数都是 2026-10-17 用 [`tools/report/topic_frequency.py`](../../../tools/report/topic_frequency.py) 对六份包重算的，读数存在 [`topic-frequency-20261017.json`](../../../tools/report/evidence/topic-frequency-20261017.json)：三份 offboard 包（`bag/3_traj1_hover`、`bag/4_traj2_square`、`bag/5_traj3_lemniscate`）与三份模型预测控制包（`070103`、`070446`、`070935`）。`bag/6_free__20261017-020757` 是轨迹 4 的起飞段，不在本章的三趟对比里。

估计口径分两种。动捕与视觉位姿用 n/span（帧数 ÷ 记录跨度）：这条流是成簇到达的，相邻帧中位间隔会把频率高估到 600–1200 Hz，n/span 才与 Nokov 配置说明的 200 Hz 出包率相符（该配置值本身未测）。50 Hz 的参考与 setpoint 用活跃期中位间隔：整包平均会被 MANUAL/LAND 的停发段拉低，不能代表飞行段的频率。

这些是快照读数，不是规格。频率随 CPU 负载、发布逻辑与录制起止变化；`/traj/reference` 在 20 ms 附近抖动是常态，缺口内消费者保持上一帧。六份包的最大缺口为 44–154 ms，读法见 [`docs/report/data/README.md`](../data/README.md) 的已知坑 1。

`/control/state` 是 1 Hz 心跳，状态变化时会立即补发一帧，所以整包均值略高于 1 Hz；`/mavros/state` 随 mavros 收到的系统状态心跳更新，模式与 armed 的变化最多滞后一个心跳周期（实测中位 1.00 Hz）。

`type_mask` 的语义是“置 1 = 忽略该字段”。读反了的症状是位置或 yaw 不生效，而话题上仍有帧；表里给出的掩码值就是代码里的断言值。

<big>**一句话总结：表中话题构成两条控制链；频率是六份包重算的实测值，不是规格。**</big>

---

## 5.6 两条数据链

两条链共用同一个任务总管和同一条轨迹源，差别只在“谁最后给 PX4 发指令、发到哪一层”。下面按操作顺序各走一遍。

### 5.6.1 offboard 档：位置 + yaw

1. 起飞前，操作台 `fly.py` 发 `/traj/select` 选轨迹；`traj` 只加载形状，不发参考，还要等 `/traj/start`。
2. 操作员拨 ch6 到 Offboard，按 ch9 第 1 次上升沿。`control` 先以 50 Hz 预流 setpoint；预流满 1 s 后请求 `/mavros/set_mode` 切 OFFBOARD；确认已进入 OFFBOARD、遥控新鲜且位置守卫成立后，才发 `/mavros/cmd/arming` 解锁——PX4 的解锁前置就是“已在 OFFBOARD 且 setpoint 在流”。解锁后起飞到 1 m 并悬停。
3. 从 STANDBY 起，`control` 就在发 `/mavros/setpoint_raw/local`，50 Hz，位置 + yaw（`type_mask=2552`，速度被忽略）。这段预流满足 PX4 的 Offboard 前置：切模式前 setpoint 已以 ≥2 Hz 持续 ≥1 s。
4. HOVER 判稳那一刻，`control` 记录 EKF 报的位置，作为轨迹原点；WAIT_START 期间它以这个值持续发流。数据管线从这个恒定指令值重建原点，而不是取 `/traj/start` 时刻的飞机位置（两者实测差 6.5–10.5 cm）。
5. 操作员第 2 次按 ch9：上升沿只记下“欠一次开始”，**松手**时 `control` 才进 TRACK 并自己发 `/traj/start`（按住 ≥1 s 则走长按降落，见 5.8）；`traj` 的时间轴从 0 开始，发布相对轨迹原点的 `/traj/reference`（50 Hz）。
6. `control` 把轨迹原点加到参考上，发绝对 `/mavros/setpoint_raw/local`；mavros 做 ENU→NED；PX4 的位置环 → 姿态环 → 角速率环 → 控制分配 → DShot。
7. 反馈来自 `/mavros/local_position/pose` 与 `/mavros/local_position/velocity_local`（EKF2，2026-10-17 实测 29.9–30.1 Hz）。这条链的控制器是 PX4 内置 PID：我们只给位置与 yaw，推力与角速率都由 PX4 自己算。
8. `/traj/progress` 首次 ≥1.0 时，`control` 进 LAND：停发 setpoint，并请求 `AUTO.LAND`；上锁由 PX4 的 `COM_DISARM_LAND=2.0` 完成，`control` 不发上锁。实测正方形与 8 字在 MANUAL/LAND 阶段有 6.6–7.1 s 的 setpoint 缺口。
9. 输入类失效（动捕超时、EKF 位置超时、mavros 状态超时等）下，`control` 先停发 setpoint，已解锁时再请求 `AUTO.LAND`，把飞机交给 PX4 的降落与上锁流程。若是打杆接管或模式被夺，`control` 只停发、不请求任何模式，把控制权交还飞行员。我们不发“让飞机继续飞”的兜底指令。

**控制量。** 位置 $(x,y,z)$ + yaw。速度虽然在 `/traj/reference` 里，但位置档的 `type_mask=2552` 把它忽略了；我们没有下发推力或机体角速率。

<big>**一句话总结：offboard 档只把位置与 yaw 交给 PX4，推力与角速率仍由飞控内置 PID 产生。**</big>

### 5.6.2 模型预测控制档：总推力 + 机体三轴角速率

1. 任务流程与 offboard 档逐字相同：选轨迹、ch6、第 1 次按 ch9 解锁起飞、判稳、第 2 次按 ch9 后松手进 TRACK、飞完自动降落。`control` 仍是任务总管。
2. 区别在输出：`control` 以 `output_mode=external` 起，不创建 `/mavros/setpoint_raw/local` 的发布者，只发 `/control/target`——绝对 ENU 位置、速度与 yaw（`type_mask=2496`），50 Hz。
3. `mpc_node` 订阅 `/control/target` 当参考，订阅 `/mocap/pose_enu` 与 `/mocap/velocity_enu` 当状态，按 100 Hz 解一个 10 状态优化问题。状态是位置、速度、四元数；控制量是总推力 $T$ 与机体三轴角速率 $\boldsymbol{\omega}$；模型、代价与约束见第 8 章。
4. 解出来的指令发到 `/mavros/setpoint_raw/attitude`：`body_rate` 是 FLU 机体系角速率，`thrust` 是正号归一化总推力（0..1），`type_mask=128` 表示忽略姿态四元数。mavros 把角速率 FLU→FRD，`thrust` 不转换；PX4 的 `fill_thrust` 再加负号落到 `thrust_body_z`。
5. PX4 只保留角速率环（1 kHz）与控制分配；位置环与姿态环由模型预测控制承担。这是 (C) 级的定义：我们下到角速率，不下到电机。
6. 反馈是原始动捕（181–193 Hz），不是 EKF2 的 30 Hz。`/control/target` 已经是绝对量，`mpc_node` 不再加锚点；这与旧 `traj` 参考源不同，后者要自己加锚点。
7. 在 `control` 请求 OFFBOARD 之前的那段预流里，`mpc_node` 以 `offboard_prestream=true` 发纯悬停帧，让 PX4 看见姿态流；它不 arm、不 set_mode，这两件事仍在 `control`。
8. 停机：`/control/target` 断流 0.5 s，`mpc_node` 以 `no_control_target` 关闸并**停流**——不发布，既不冻结在最后一个目标，也不发零帧；随后 PX4 在 `COM_OF_LOSS_T=1.0 s` 后退出 Offboard，按 `COM_OBL_RC_ACT=0` 回 Position。求解器不可用时还有 hold→hover 的兜底阶梯（[`mpc-node-fallback.md`](../../control/mpc-node-fallback.md)），但最后一道始终是 PX4 自己的超时。
9. 两条链不会同时写同一个话题：external 档下 `control` 一帧都不写 `/mavros/setpoint_raw/local`；offboard 档下不启动 `mpc_node`。这是结构上的互斥，不是现场纪律。

**控制量。** 总推力 $T$ + 机体三轴角速率 $\boldsymbol{\omega}$，即 CTBR。没有位置、速度或姿态角 setpoint 发给 PX4；模型预测控制接管位置环与姿态环，PX4 只保留角速率环、混控与安全机制。

<big>**一句话总结：模型预测控制档把总推力与机体三轴角速率交给 PX4，位置环与姿态环由控制器承担。**</big>

---

## 5.7 坐标系与转换发生在哪一侧

表 5-7 只回答“转换由谁做、往哪个方向做”。ENU↔NED 与 FLU↔FRD 的公式、四元数写法以及模型内部的推导，见第 8 章。

**表 5-7　坐标系与转换边界**

| 量 | ROS 侧（本仓节点之间） | mavros 边界 | PX4 侧 | 谁做转换 |
|---|---|---|---|---|
| 位置、速度 | ENU | `vision_pose` 入：ENU→NED；`local_position` 出：NED→ENU；`setpoint_raw/local` 入：ENU→NED | NED | mavros |
| 姿态四元数 | ENU/FLU | `vision_pose` 入：ENU→NED + FLU→FRD；`local_position` 出：NED→ENU + FRD→FLU | NED/FRD | mavros |
| 机体角速率 | FLU | `setpoint_raw/attitude` 入：FLU→FRD | FRD | mavros |
| 总推力 | 标量，正号 0..1 | 不转换 | `thrust_body_z` 为负 | PX4 `fill_thrust` |
| 模型内部状态 | ENU/FLU | — | — | 无（`pose_to_state` 不做转换） |

ROS 侧统一 ENU/FLU，PX4 侧统一 NED/FRD，转换只发生在 mavros 的输入/输出边界上，两个方向都有。模型预测控制在内部也用 ENU/FLU，所以状态输入这一侧没有坐标转换；这是刻意的，少一个转换点，就少一类符号错误。

**不要转两次。** 我们把动捕原生 ENU/FLU 交给 mavros，mavros 自己转；如果先在节点里转一遍，mavros 再转一遍，两次转换会互相抵消。角速率这条的指纹很隐蔽：roll 不受影响，pitch/yaw 静默反向，而飞控最终收到的数值与写对时完全一样——所以断言不能落在“PX4 收到了什么”上，只能落在我们发出的字段上（[`sign_map.py`](../../../src/nmpc/nmpc/sign_map.py) 的 `mavros_attitude_target`）。

`PositionTarget` 类 setpoint 的 `frame_id` 只是标注，mavros 读的是 `coordinate_frame` 与 `type_mask`；`AttitudeTarget` 没有 `coordinate_frame`，角速率按机体系处理。`/traj/reference` 的 `frame_id="traj_origin"` 也不是 TF 坐标系，它只说明“这个位置是相对轨迹原点的”。

两条路径的原点：`/mavros/local_position/pose` 是 EKF2 的局部 ENU，`/mocap/pose_enu` 是 `mocap` 节点的相对 ENU。它们由不同环节定义，不能默认相同；第 9 章对每个源各减自己的原点。报告里的“实际轨迹”统一用 `/mavros/vision_pose/pose`，但 offboard 与模型预测控制两组的控制器反馈并不是同一条话题——前者是 EKF2 的 30 Hz 输出，后者是原始动捕。

yaw 也由 mavros 转换；在 ROS 侧 `yaw=0` 指向 ENU 的 +x，不是“保持当前机头”。起飞朝向与轨迹 yaw 的处理见第 7、9 章。

<big>**一句话总结：ROS 侧统一 ENU/FLU，ENU↔NED 与 FLU↔FRD 只在 mavros 边界转换一次。**</big>

---

## 5.8 安全边界：谁能让飞机动

表 5-8 按“谁能直接让飞机动”列出动作、执行者与接口。状态机的完整转移与失效路径见第 7 章；这里只划责任边界。

**表 5-8　安全边界**

| 动作 | 谁执行 | 接口 / 触发 | 证据 |
|---|---|---|---|
| 解锁（程序） | `control` | `/mavros/cmd/arming`；ch9 第 1 次上升沿，或台架 `/control/takeoff` | [phase1-implementation.md §4](../../control/phase1-implementation.md) |
| 解锁（硬件开关） | 遥控器 → PX4 | ch5；`RC_MAP_ARM_SW=5` | [`docs/assets/z0.params`](../../assets/z0.params) |
| 起飞 | `control` | 状态机 `AUTO/TAKEOFF`；到 1 m 悬停 | 同上 |
| 切 OFFBOARD | `control` | `/mavros/set_mode`；切之前 setpoint 已预流 ≥1 s | 同上 |
| 切模式（硬件开关） | 遥控器 → PX4 | ch6；`RC_MAP_FLTMODE=6` | [`docs/assets/z0.params`](../../assets/z0.params) |
| 切 AUTO.LAND | `control` | `/mavros/set_mode`；任务完成、超时、失效，或在飞时长按 ch9 ≥1 s | [`fsm.hpp`](../../../src/control/include/control/fsm.hpp) |
| 发 setpoint（offboard） | `control` | `/mavros/setpoint_raw/local` | [`run/README.md`](../../../run/README.md) |
| 发 setpoint（模型预测控制） | `mpc_node` | `/mavros/setpoint_raw/attitude` | 同上 |
| 停发（offboard） | `control` | 打杆接管、模式被夺、动捕/状态超时、进 LAND | [phase1-implementation.md §4](../../control/phase1-implementation.md) |
| 停发（模型预测控制） | `mpc_node` | `/control/target` 断流 0.5 s、状态源超时、离开 OFFBOARD、掉 armed | [`node_core.py`](../../../src/nmpc/nmpc/node_core.py) |
| OFFBOARD 丢失 | PX4 | `COM_OF_LOSS_T=1.0 s`；`COM_OBL_RC_ACT=0` → Position | [`docs/assets/z0.params`](../../assets/z0.params) |
| 打杆接管 | PX4 | `COM_RC_OVERRIDE=3` | 同上 |
| 杀车 | PX4 | ch8；`RC_MAP_KILL_SW=8` | 同上 |
| 遥控失联 | PX4 | `NAV_RCL_ACT=1` → Hold（2026-09-28 实测） | [params-changes.md](../../reference/params-changes.md) |
| 上锁 | PX4 | `COM_DISARM_LAND=2.0`；`control` 不发上锁 | [`fsm.hpp`](../../../src/control/include/control/fsm.hpp) |
| 起链 / 录制 | `run/z0.py` | 只起停进程与录制；没有 arm / mode / setpoint 入口 | [`run/README.md`](../../../run/README.md) |

这张表的读法是“谁能直接让飞机动”。**程序侧**只有 `control` 能请求解锁、切模式与起飞；遥控器上另有 ch5 解锁开关与 ch6 模式开关，这两条硬件路径直达 PX4，不经过 Jetson。若解锁不是 `control` 发起的，状态机的 `kArmingByUs` 守卫不成立，`control` 会停发并告警，把飞机留在飞行员手里。`mpc_node` 只写执行器 setpoint；`traj` 只发参考；`run/z0.py` 只起进程与录制。`run/z0.py` 没有解锁、起飞、切模式、发帧的入口，这不是遗漏，而是把“开始键之前不发控制指令”这条防线留在飞控与 `control` 之间。

`control` 与 `mpc_node` 都只停发，不抢回 OFFBOARD；打杆接管后不自动切回 Offboard。最终接管权在 PX4 与遥控器。

`MAV_FWDEXTSP=1` 是整条 (C) 级下行的硬前置；它是 0 时 setpoint 被静默丢弃，飞控没有日志。`nmpc/preflight_check.py` 已把这条列入自检。

setpoint 自己就是 Offboard 心跳：没有单独的 `OffboardControlMode` 流。停流超过 1 s，PX4 自行退出 Offboard；这是第二道保护，不依赖我们的节点还活着。

遥控链路独立于 Jetson：Jetson 死了，ch6 与 ch8 仍然直达飞控。这是这套架构里唯一不经过机载计算机的人工接管路径。

安全网不是控制器的备用状态源。EKF2 那条路只保证 PX4 还能估计位置、还能执行失效保护；模型预测控制没有“改读 EKF2”这一档。

<big>**一句话总结：解锁、切模式与停发由 control、遥控器或 PX4 保护承担；模型预测控制节点只发指令。**</big>

---

## 5.9 本章引用清单

表 5-9 把本章的接口、参数与判定结论按主题归到仓库内的文件。正文里已经随行给过链接，这里只是便于回查。

**表 5-9　本章引用清单**

| 主题 | 文件 |
|---|---|
| 三层架构、两条链路的解耦理由 | [`docs/PROJECT.md` §3](../../PROJECT.md) |
| 话题与服务、坐标与原点 | [`docs/control/phase1-implementation.md` §2–3](../../control/phase1-implementation.md) |
| mavros 转换、下行定案、`MAV_FWDEXTSP` | [`docs/reference/px4-interface.md` §4.3、§5.1](../../reference/px4-interface.md) |
| 动捕链路、VRPN、串口与网段 | [`docs/reference/mocap-setup.md`](../../reference/mocap-setup.md) |
| 起链、两档、`run/z0.py` 的安全边界 | [`run/README.md`](../../../run/README.md) |
| 模型预测控制下行与闸门 | [`src/nmpc/nmpc/mpc_node.py`](../../../src/nmpc/nmpc/mpc_node.py)、[`node_core.py`](../../../src/nmpc/nmpc/node_core.py) |
| 符号映射与推力正负号 | [`src/nmpc/nmpc/sign_map.py`](../../../src/nmpc/nmpc/sign_map.py)、[`docs/control/sign-map.md`](../../control/sign-map.md) |
| `control` 的发布接口 | [`src/control/include/control/px4_interface.hpp`](../../../src/control/include/control/px4_interface.hpp)、[`px4_interface.cpp`](../../../src/control/src/px4_interface.cpp) |
| 动捕节点的预处理 | [`src/mocap/README.md`](../../../src/mocap/README.md) |
| 轨迹参考的频率与掩码 | [`src/traj/src/traj_node.cpp`](../../../src/traj/src/traj_node.cpp)、[`src/traj/config/traj.yaml`](../../../src/traj/config/traj.yaml) |
| 飞控参数现值 | [`docs/assets/z0.params`](../../assets/z0.params) |
| 数据口径与已知坑 | [`docs/report/data/README.md`](../data/README.md) |
| 表 5-6 的频率读法与复算脚本 | [`tools/report/topic_frequency.py`](../../../tools/report/topic_frequency.py)、[`topic-frequency-20261017.json`](../../../tools/report/evidence/topic-frequency-20261017.json) |
| 下行切换票 | [`.scratch/phase4/issues/13-mavros-downlink-switch.md`](../../../.scratch/phase4/issues/13-mavros-downlink-switch.md) |
| 任务总管与 `/control/target` | [`.scratch/phase4/issues/15-trajectory-flight-roles.md`](../../../.scratch/phase4/issues/15-trajectory-flight-roles.md) |
| 术语 | [`CONTEXT.md`](../../../CONTEXT.md) |

<big>**一句话总结：本章每个接口结论都能回到仓库内的文档、源码或参数文件。**</big>
