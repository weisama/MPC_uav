# 第 5 章 系统架构与实验平台

本章回答两个问题：z0 的自主飞行栈分成哪几层，层与层之间用什么接口连接；哪些话题、坐标系与安全边界构成了两条控制链——offboard 位置控制与 MPC。第 3 章讲零件与物理参数，第 6 章讲代码结构，第 7 章讲起链、遥控与状态机；本章只讲它们之间的连接关系与数据流，不重复各章的细节。

按操作者裁定，本章不画架构图；结构用文字与表描述，话题表是本章的主干。

---

## 5.1 符号与缩写

表 5-1 列出本章反复出现的缩写与符号。物理量的完整符号表在第 3 章；本章只补充与接口、频率、坐标系有关的几个。

**表 5-1　本章缩写与符号。**

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

**表 5-2　三层架构。**

| 层 | 主要组成 | 输入 | 输出 | 失效时先失去什么 |
|---|---|---|---|---|
| **定位层** | Nokov 光学动捕相机、Nokov 软件、VRPN 服务（`10.1.1.198:3883`） | 机体上 5 颗反光球的图像 | 刚体位姿与速度（VRPN） | 两条动捕路径同时失去状态；飞控失去视觉位置；只能遥控或降落 |
| **机载层** | Jetson Orin NX 8GB；ROS 2 Humble；`vrpn_listener`、`mocap`、`traj`、`control`、`nmpc`、`mavros` | VRPN/UDP；`/mavros/*` 的飞控状态与遥控通道 | setpoint（位置，或推力 + 角速率）、`/control/*`、录制包 | 控制链停止发帧；PX4 在 1 s 后退出 Offboard；遥控仍然可用 |
| **飞控层** | NxtPX4v2；PX4 1.14.3 厂商构建；EKF2、位置/姿态/角速率环、控制分配、失效保护、DShot | MAVLink setpoint、遥控、IMU 与气压计 | 四路电机指令 | 飞机失去全部控制；最后手段是 ch8 杀车或 PX4 自动降落 |

三层是按失效后果切的，不是按机箱切的。飞控与机载电脑都装在飞机上，动捕主机在地面；把它们并列成三层，是因为它们的失效后果不同：定位层失效，飞机失去外部位置；机载层失效，飞机失去任务指令；飞控层失效，飞机失去全部控制。

层间只有两条物理连接：动捕主机到机载电脑的 WiFi，机载电脑到飞控的串口。机载层没有任何一条线能直接驱动电调；飞控层也不运行我们的控制律。这个边界是后面所有安全论证的前提。

机载层内部，`control` 是任务总管，`traj` 只发参考，`mpc_node` 只解优化问题，`mavros` 是唯一与飞控说话的进程。第 6 章按包展开这些职责，本章只按角色使用这些名字。

<big>**一句话总结：定位层给位姿，机载层做任务与计算，飞控层保留角速率环、混控与失效保护。层级之间的输入输出标准参考成熟开源项目。**</big>

---

## 5.3 硬件连接与软件栈

### 5.3.1 硬件连接

表 5-3 给出本章架构涉及的物理连接。零件型号与物理规格见第 3 章，这里只列“谁连谁、走什么协议、哪个参数锁住它”。

**表 5-3　硬件连接。**

| 连接 | 近端 | 远端 | 物理/协议 | 关键参数 | 证据 |
|-------|---|---|---|---|---|
| 动捕 ↔ 机载电脑 | Nokov 主机 `10.1.1.198` | 机载电脑 WiFi `10.1.1.114/24`（2026-09-21 读数） | VRPN over UDP，端口 3883 | 同一网段；不需要额外网卡 | [mocap-setup.md §9](../../reference/mocap-setup.md) |
| 机载电脑 ↔ 飞控 | `/dev/ttyTHS1` @ 921600 | 飞控 TELEM2 / Onboard | MAVLink 串口 | `MAV_1_CONFIG=102`、`SER_TEL2_BAUD=921600`；mavros 独占 | [mocap-setup.md §5](../../reference/mocap-setup.md)、[params-changes.md §E1](../../reference/params-changes.md) |
| 飞控 ↔ 电调 | 4 路 DShot | 33A 四合一电调 | DShot | `DSHOT_MIN=0.055`、`DSHOT_3D_ENABLE=0`、`DSHOT_TEL_CFG=0`（遥测关） | [`docs/assets/z0.params`](../../assets/z0.params) |
| 遥控 ↔ 飞控 | RadioMaster Pocket（ELRS 2.4 GHz、EdgeTX） | 飞控遥控输入 | CRSF | `RC_INPUT_PROTO=6`；ch5 解锁、ch6 模式、ch8 杀车、ch9 开始 | [params-changes.md §H](../../reference/params-changes.md) |
| 机载电脑 ↔ [QGC](https://qgroundcontrol.com/)（可选） | mavros `gcs_url` | 地面站 | UDP 转发 MAVLink | 只在需要 QGC 时启用；不参与控制 | [mocap-setup.md §8](../../reference/mocap-setup.md) |

### 5.3.2 软件栈

表 5-4 列出两条链上实际运行的软件与版本。版本以本报告证据产出时的实际值为准；没有一手记录的项标“未记录”，不按默认值补。

**表 5-4　软件栈。**

| 组件 | 版本/标识 | 跑在哪 | 角色 |
|---|---|---|---|
| [Ubuntu](https://releases.ubuntu.com/22.04/) / L4T | 22.04.5 LTS / R36.5.0 / 内核 5.15.185-tegra | 机载电脑 | 宿主系统 |
| [ROS 2](https://docs.ros.org/en/humble/) | Humble | 机载电脑 | 节点间通信 |
| [mavros](https://github.com/mavlink/mavros) | 2.15.1 @ `22ae5b7`，源码构建在 `~/mavros_ws` | 机载电脑 | 唯一的 MAVLink 桥 |
| [acados](https://github.com/acados/acados) | `v0.6.0-41-g00947ddc3`，SQP_RTI + HPIPM | 机载电脑，aarch64 原生 | 优化问题求解 |
| [PX4](https://github.com/PX4/PX4-Autopilot) | 1.14.3 系厂商构建 @ `4a0e65f2` | 飞控 | EKF2、控制环、混控、失效保护 |
| [VRPN](https://github.com/vrpn/vrpn) 客户端 | `ros-humble-vrpn` 7.35.0 | 机载电脑 | 接收动捕 |
| [Nokov](https://www.nokov.com/) 软件 | 版本未记录；VRPN banner 自报 7.34 | 动捕主机 | VRPN 服务端 |
| Python / rclpy | 3.10 | 机载电脑 | `mpc_node.py` 等节点 |

---

## 5.4 话题表

表 5-5 是本章的主干：两条控制链经过的话题、类型、生产者与消费者、频率、坐标系，以及它何时存在。

**表 5-5　两条控制链的主干话题。**

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
| `/mavros/setpoint_raw/attitude` | `mavros_msgs/AttitudeTarget` | `mpc_node` → `mavros` | 98.8–99.8 Hz（n/span） | `body_rate` 为 FLU；`thrust` 为正号 0..1；`type_mask=128` | 仅 MPC 档 |
| `/mavros/vision_pose/pose` | `geometry_msgs/PoseStamped` | `mocap` → `mavros` | 181–193 Hz（n/span） | ROS ENU/FLU；mavros 转 NED/FRD | `mocap` 启动后 |
| `/mocap/pose_enu` | `geometry_msgs/PoseStamped` | `mocap` → `mpc_node` / 录制 | 181–193 Hz（n/span） | 动捕原生 ENU/FLU，经外参、原点与 yaw 处理 | 同上 |
| `/mocap/velocity_enu` | `geometry_msgs/TwistStamped` | `mocap` → `mpc_node` / 录制 | 181–193 Hz（n/span） | ENU；角速度分量实测恒 0 | 同上 |
| `/mavros/local_position/pose` | `geometry_msgs/PoseStamped` | `mavros` → `control` | 29.9–30.1 Hz | ROS ENU（EKF2 输出） | mavros 连上后 |
| `/mavros/local_position/velocity_local` | `geometry_msgs/TwistStamped` | `mavros` → `control` | 29.9–30.0 Hz | ROS ENU | 同上 |
| `/mavros/state` | `mavros_msgs/State` | `mavros` → `control` / `mocap` / `mpc_node` / `z0.py` | 1.00 Hz | `connected` / `armed` / `mode` | 同上 |
| `/mavros/rc/in` | `mavros_msgs/RCIn` | `mavros` → `control` | 19.95–20.04 Hz | 原始 PWM 读数（µs）；归一化在 `control` 侧（`low_us=1000` / `high_us=2000`）；ch5 / ch6 / ch8 / ch9 | 同上 |

---

## 5.5 两条数据链

两条链共用同一个任务总管、同一条轨迹源和同一套起飞/降落流程，差别只在“最后谁给 PX4 发指令、发到哪一层”。表 5-6 先列共同点，表 5-7 与表 5-8 按同一组项目分别展开两档。

### 5.5.1 共同点

**表 5-6　两条链的共同点。**

| 项目 | 两档共同的做法 |
|---|---|
| 任务总管 | `control` 负责选轨迹、解锁、切模式、状态机、轨迹原点与降落；`mpc_node` 不 arm、不 set_mode。 |
| 轨迹源 | `traj` 发布 `/traj/reference`（50 Hz，相对轨迹原点）与 `/traj/progress`；`control` 在进入 TRACK 时发 `/traj/start`。 |
| 操作流程 | ch6 选模式；第 1 次按 ch9 解锁起飞；判稳后第 2 次按 ch9，**松手**才进 TRACK；飞完自动降落。 |
| OFFBOARD 前置 | 切模式前 setpoint 已持续预流 ≥1 s；预流频率 ≥2 Hz（offboard 50 Hz，MPC 100 Hz）。 |
| 轨迹原点 | 在 `AUTO/WAIT_START` 判稳时锁存，取当时指令的恒定值；不取 `/traj/start` 时刻的飞机位置。 |
| 下行链路 | 只有 mavros 一条；不使用 uXRCE-DDS；`MAV_FWDEXTSP=1` 是硬前置。 |
| 结束与失效 | 任务完成时 `control` 停发并请求 `AUTO.LAND`；输入类失效（动捕、EKF 位置、mavros 状态超时等）同样停发，已解锁时请求 `AUTO.LAND`；打杆接管或模式被夺则只停发、交还控制权。 |
| 互斥 | 同一时刻只有一条链写 setpoint：external 档下 `control` 不写 `/mavros/setpoint_raw/local`；offboard 档下不启动 `mpc_node`。 |

<big>**一句话总结：两档共用任务总管、轨迹源、起飞降落流程与安全边界，只有最后一级的执行者与接口不同。**</big>

### 5.5.2 区别：两档各一张表

**表 5-7　offboard 档（位置 + yaw）。**

| 项目 | offboard 档 |
|---|---|
| 谁发指令 | `control` |
| 下行话题 | `/mavros/setpoint_raw/local`（`mavros_msgs/PositionTarget`） |
| 控制量 | 位置 $(x,y,z)$ + yaw；速度字段被忽略 |
| `type_mask` | 2552（位置 + yaw） |
| 指令频率 | 活跃期 50 Hz（2026-10-17 实测）；LAND 停发 6.6–7.1 s 拉低整包均值 |
| 参考 | `/traj/reference`（相对轨迹原点）+ 轨迹原点，绝对 ENU |
| 状态反馈 | `/mavros/local_position/pose`、`/mavros/local_position/velocity_local`（EKF2，29.9–30.1 Hz） |
| PX4 承担 | 位置环 → 姿态环 → 角速率环 → 控制分配 → DShot |
| 预流 | `control` 从 STANDBY 起以 50 Hz 发当前位置与当前机头 |
| 停止条件 | `/traj/progress` 首次 ≥1.0；输入类失效（已解锁时请求 `AUTO.LAND`）；打杆接管或模式被夺（只停发） |

**表 5-8　MPC 档（总推力 + 机体三轴角速率）。**

| 项目 | MPC 档 |
|---|---|
| 谁发指令 | `mpc_node` |
| 下行话题 | `/mavros/setpoint_raw/attitude`（`mavros_msgs/AttitudeTarget`） |
| 控制量 | 总推力 $T$ + 机体三轴角速率 $\boldsymbol{\omega}$（CTBR） |
| `type_mask` | 128（忽略姿态四元数） |
| 指令频率 | 求解 100 Hz；2026-10-17 实测 98.8–99.8 Hz |
| 参考 | `/control/target`（绝对 ENU 位置、速度与 yaw，`type_mask=2496`，50 Hz） |
| 状态反馈 | `/mocap/pose_enu`、`/mocap/velocity_enu`（原始动捕，181–193 Hz） |
| PX4 承担 | 角速率环（1 kHz）→ 控制分配 → DShot；位置环与姿态环由控制器承担 |
| 预流 | `mpc_node` 以 `offboard_prestream=true` 发纯悬停帧；`control` 仍负责 arm 与 set_mode |
| 停止条件 | `/control/target` 断流 0.5 s → 停流；PX4 在 `COM_OF_LOSS_T=1.0 s` 后退出 Offboard；求解器不可用时走 hold→hover 阶梯 |

<big>**一句话总结：offboard 档交位置与 yaw，MPC 档交总推力与机体三轴角速率，PX4 保留的环路因此不同。**</big>

---

## 5.6 坐标系与转换发生在哪一侧

表 5-9 只回答“转换由谁做、往哪个方向做”。ENU↔NED 与 FLU↔FRD 的公式、四元数写法以及模型内部的推导，见第 8 章。

**表 5-9　坐标系与转换边界。**

| 量 | ROS 侧（本仓节点之间） | mavros 边界 | PX4 侧 | 谁做转换 |
|---|---|---|---|---|
| 位置、速度 | ENU | `vision_pose` 入：ENU→NED；`local_position` 出：NED→ENU；`setpoint_raw/local` 入：ENU→NED | NED | mavros |
| 姿态四元数 | ENU/FLU | `vision_pose` 入：ENU→NED + FLU→FRD；`local_position` 出：NED→ENU + FRD→FLU | NED/FRD | mavros |
| 机体角速率 | FLU | `setpoint_raw/attitude` 入：FLU→FRD | FRD | mavros |
| 总推力 | 标量，正号 0..1 | 不转换 | `thrust_body_z` 为负 | PX4 `fill_thrust` |
| 模型内部状态 | ENU/FLU | — | — | 无（`pose_to_state` 不做转换） |


<big>**一句话总结：ROS 侧统一 ENU/FLU，ENU↔NED 与 FLU↔FRD 只在 mavros 边界转换一次。**</big>

---

## 5.7 安全边界：谁能让飞机动

表 5-10 按“谁能直接让飞机动”列出动作、执行者与接口。状态机的完整转移与失效路径见第 7 章；这里只划责任边界。

**表 5-10　安全边界。**

| 动作 | 谁执行 | 接口 / 触发 | 证据 |
|---|---|---|---|
| 解锁（程序） | `control` | `/mavros/cmd/arming`；ch9 第 1 次上升沿，或台架 `/control/takeoff` | [phase1-implementation.md §4](../../control/phase1-implementation.md) |
| 解锁（硬件开关） | 遥控器 → PX4 | ch5；`RC_MAP_ARM_SW=5` | [`docs/assets/z0.params`](../../assets/z0.params) |
| 起飞 | `control` | 状态机 `AUTO/TAKEOFF`；到 1 m 悬停 | 同上 |
| 切 OFFBOARD | `control` | `/mavros/set_mode`；切之前 setpoint 已预流 ≥1 s | 同上 |
| 切模式（硬件开关） | 遥控器 → PX4 | ch6；`RC_MAP_FLTMODE=6` | [`docs/assets/z0.params`](../../assets/z0.params) |
| 切 AUTO.LAND | `control` | `/mavros/set_mode`；任务完成、超时、失效，或在飞时长按 ch9 ≥1 s | [`fsm.hpp`](../../../src/control/include/control/fsm.hpp) |
| 发 setpoint（offboard） | `control` | `/mavros/setpoint_raw/local` | [`run/README.md`](../../../run/README.md) |
| 发 setpoint（MPC） | `mpc_node` | `/mavros/setpoint_raw/attitude` | 同上 |
| 停发（offboard） | `control` | 打杆接管、模式被夺、动捕/状态超时、进 LAND | [phase1-implementation.md §4](../../control/phase1-implementation.md) |
| 停发（MPC） | `mpc_node` | `/control/target` 断流 0.5 s、状态源超时、离开 OFFBOARD、掉 armed | [`node_core.py`](../../../src/nmpc/nmpc/node_core.py) |
| OFFBOARD 丢失 | PX4 | `COM_OF_LOSS_T=1.0 s`；`COM_OBL_RC_ACT=0` → Position | [`docs/assets/z0.params`](../../assets/z0.params) |
| 打杆接管 | PX4 | `COM_RC_OVERRIDE=3` | 同上 |
| 杀车 | PX4 | ch8；`RC_MAP_KILL_SW=8` | 同上 |
| 遥控失联 | PX4 | `NAV_RCL_ACT=1` → Hold（2026-09-28 实测） | [params-changes.md](../../reference/params-changes.md) |
| 上锁 | PX4 | `COM_DISARM_LAND=2.0`；`control` 不发上锁 | [`fsm.hpp`](../../../src/control/include/control/fsm.hpp) |
| 起链 / 录制 | [`run/z0.py`](../../../run/z0.py) | 只起停进程与录制；没有 arm / mode / setpoint 入口 | [`run/README.md`](../../../run/README.md) |


<big>**一句话总结：解锁、切模式与停发由 control、遥控器或 PX4 保护承担；MPC 节点只发指令。**</big>

---

## 5.8 本章引用清单

表 5-11 把本章的接口、参数与判定结论按主题归到仓库内的文件。正文里已经随行给过链接，这里只是便于回查。

**表 5-11　本章引用清单。**

| 主题 | 文件 |
|---|---|
| 三层架构、两条链路的解耦理由 | [`docs/PROJECT.md` §3](../../PROJECT.md) |
| 话题与服务、坐标与原点 | [`docs/control/phase1-implementation.md` §2–3](../../control/phase1-implementation.md) |
| mavros 转换、下行定案、`MAV_FWDEXTSP` | [`docs/reference/px4-interface.md` §4.3、§5.1](../../reference/px4-interface.md) |
| 动捕链路、VRPN、串口与网段 | [`docs/reference/mocap-setup.md`](../../reference/mocap-setup.md) |
| 起链、两档、[`run/z0.py`](../../../run/z0.py) 的安全边界 | [`run/README.md`](../../../run/README.md) |
| MPC 下行与闸门 | [`src/nmpc/nmpc/mpc_node.py`](../../../src/nmpc/nmpc/mpc_node.py)、[`node_core.py`](../../../src/nmpc/nmpc/node_core.py) |
| 符号映射与推力正负号 | [`src/nmpc/nmpc/sign_map.py`](../../../src/nmpc/nmpc/sign_map.py)、[`docs/control/sign-map.md`](../../control/sign-map.md) |
| `control` 的发布接口 | [`src/control/include/control/px4_interface.hpp`](../../../src/control/include/control/px4_interface.hpp)、[`px4_interface.cpp`](../../../src/control/src/px4_interface.cpp) |
| 动捕节点的预处理 | [`src/mocap/README.md`](../../../src/mocap/README.md) |
| 轨迹参考的频率与掩码 | [`src/traj/src/traj_node.cpp`](../../../src/traj/src/traj_node.cpp)、[`src/traj/config/traj.yaml`](../../../src/traj/config/traj.yaml) |
| 飞控参数现值 | [`docs/assets/z0.params`](../../assets/z0.params) |
| 数据口径与已知坑 | [`docs/report/data/README.md`](../data/README.md) |
| 表 5-5 的频率读法与复算脚本 | [`tools/report/topic_frequency.py`](../../../tools/report/topic_frequency.py)、[`topic-frequency-20261017.json`](../../../tools/report/evidence/topic-frequency-20261017.json) |
| 下行切换票 | [`.scratch/phase4/issues/13-mavros-downlink-switch.md`](../../../.scratch/phase4/issues/13-mavros-downlink-switch.md) |
| 任务总管与 `/control/target` | [`.scratch/phase4/issues/15-trajectory-flight-roles.md`](../../../.scratch/phase4/issues/15-trajectory-flight-roles.md) |
| 术语 | [`CONTEXT.md`](../../../CONTEXT.md) |
