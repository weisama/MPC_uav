# 第 6 章 项目结构与代码结构

第 5 章讲层与接口，本章讲这些接口落在哪些代码里。全章按「仓库目录—七个包—两条数据流—模型预测控制（MPC）的层分界—测试布局—证据目录」展开。目录树与计数都在 2026-10-17 16:18 UTC 从当前工作树现生成；它们描述的是这一刻的仓库，不是长期有效的清单。

---

## 6.1 符号、口径与快照

表 6-1 先固定本章反复出现的几个词。数学符号在本章几乎不出现；物理量符号表在第 3 章，架构缩写表在第 5 章。

**表 6-1　本章缩写与符号**

| 符号/缩写 | 含义 |
|---|---|
| colcon 根 | 运行 `colcon build` 的目录；本仓就是仓库根 |
| 包 | `src/` 下带 `package.xml` 的目录 |
| 入口脚本 | 由 `install(PROGRAMS)` 装到 `install/<包>/lib/<包>/` 的可执行文件 |
| 零 ROS 核心层 | 不 import `rclpy` 或任何具体消息包的纯 Python 模块 |
| seam 1 / seam 2 | 节点级话题边界 / 纯函数边界；定义见[第 1 阶段 spec](../../../.scratch/phase1/spec.md) |
| vendored | 原样纳入的上游代码，不改 |
| CTBR | collective thrust + body rate，总推力 + 机体角速率 |
| SITL | software-in-the-loop，软件在环 |
| OCP | optimal control problem，最优控制问题 |
| EKF2 | PX4 的扩展卡尔曼滤波器 |

本节还固定三件事：什么算「包」、什么算「飞行代码」、哪些数字是现算的。**包**按上表取 `package.xml` 的定义。**飞行代码**指会进入飞行链、能影响飞机动作的节点与模块；测试夹具、启动脚本与离线工具不算。这条区分在 6.3 节会反复用到，尤其是 `bench`。

本章引用的目录树和计数都按写稿时的工作树现算，未从 `README.md` 或旧票里抄。表 6-2 给出快照时点与复算命令；`build/`、`install/`、`log/` 是 colcon 的生成物，不在快照内。

**表 6-2　写稿快照与复算命令**

| 项 | 值或命令 |
|---|---|
| 快照时点 | 2026-10-17 16:18 UTC |
| ROS 2 包数 | `find src -maxdepth 2 -name package.xml \| wc -l` → 7 |
| 目录树 | `find . -maxdepth 2 …`（排除 `.git`、`build`、`install`、`log` 与缓存目录） |
| 包内测试文件 | `find src -path '*/test/*' -type f \( -name 'test_*.py' -o -name 'test_*.cpp' \) \| wc -l` → 26 |
| 仓库级测试文件 | `find run/tests -maxdepth 1 -name 'test_*.py' \| wc -l` → 5 |
| 工具测试文件 | `find tools -name 'test_*.py' \| wc -l` → 23 |

这些数字只用于说明量级与测试分布；改动仓库后必须按同一命令重算，不能把表里的值当成长期事实。

<big>**一句话总结：本章的目录树与计数都取自 2026-10-17 16:18 UTC 的工作树，复算命令随表给出。**</big>

---

## 6.2 仓库结构

仓库根就是 colcon 根：没有单独的 `src` 工作空间，`colcon build` 直接在仓库根执行，包在 `src/` 下。表 6-3 给出顶层目录的职责。

**表 6-3　顶层目录的职责**

| 目录 | 内容 | 是否进入飞行链 |
|---|---|---|
| `src/` | 7 个 ROS 2 包的源码、测试、launch 与配置 | 其中 5 个包会进入飞行链；`bringup` 负责起链；`bench` 只在测试里用 |
| `bag/` | 已录制的飞行数据：场景目录含 rosbag 与 `session.json`，飞控 `.ulg` 归档在 `fcu/`（有则存）；根目录有自动生成的总表 `MANIFEST.md`，`old/` 存归档 | 否，是证据 |
| `docs/` | 项目总纲、控制与接口文档、参考资料、报告章节与插图 | 否，是文档与证据 |
| `run/` | 人工起链与录制层：`z0.py`、`rec.py`、`up.sh`、`down.sh`、`tests/`、`logs/` | 间接进入：它只起停进程，不发控制指令 |
| `tools/` | 离线测量、参数辨识、SITL 扫描与报告数据管线 | 否，是离线工具 |
| `.scratch/` | 仓库内的 issue tracker：每阶段的 spec 与票 | 否，是过程记录 |

`README.md` 的目录段落写「6 个包」，但同一棵树下实际列了 7 个目录；本节按 `src/*/package.xml` 现算的 7 个写。旧目录树只作对照，不直接复制。

下面的树只展开到能看清职责的层级，不逐文件列全；`src/` 内部的文件在 6.3 节按包展开。

```text
ros_ws/                              ← colcon 根 = 仓库根
├── src/                             ← 7 个 ROS 2 包
│   ├── bench/                       测试夹具（不是飞行代码）
│   ├── bringup/                     启动层与操作台
│   ├── control/                     任务总管与控制节点
│   ├── mocap/                       动捕相对位姿节点
│   ├── nmpc/                        模型预测控制（核心层 + 节点层）
│   ├── traj/                        参考轨迹发生器
│   └── vrpn_listener/               VRPN → ROS 2 桥（vendored）
├── bag/                             录制数据与归档
├── docs/
│   ├── PROJECT.md                   项目总纲
│   ├── agents/                      协作代理的 issue tracker / triage 规则
│   ├── assets/                      参数快照、论文、演练记录
│   ├── control/                     控制实现、台架、安全路径文档
│   ├── reference/                   飞控、动捕、接口参考
│   └── report/                      第 5 阶段报告：chapters / data / figures / generated
├── run/                             人工起链与录制层
├── tools/                           离线测量与数据处理
├── .scratch/                        仓库内 issue tracker
├── README.md                        项目入口
├── CONTEXT.md                       术语表与书写约定
├── operation_manual.md              现场一页纸
└── CLAUDE.md                        给协作代理的项目约定
```

`bag/`、`docs/`、`run/`、`tools/` 与 `src/` 的分离不是按文件类型，而是按**变更节奏与失效后果**：`src/` 改一次要重新构建、可能要重新试飞；`bag/` 与 `docs/assets/` 基本只增不改；`run/` 与 `tools/` 可以在不碰飞行代码的情况下迭代。把这三类放在不同顶层目录，是为了让「这次改动要不要上飞机」一眼可判。

<big>**一句话总结：飞行代码集中在 src/ 的五个飞行包里，bringup 起链，bench 是测试夹具；数据、文档与工具分居其余目录。**</big>

---

## 6.3 七个 ROS 2 包

表 6-4 先给全景。职责一列只写这个包**唯一负责**的事；入口/运行方式一列写 `ros2 run`、`ros2 launch` 实际解析到的名字，或模块运行方式。

**表 6-4　七个包的职责、入口与主要依赖**

| 包 | 形态 | 职责 | 入口/运行方式 | 主要依赖 |
|---|---|---|---|---|
| `bench` | Python（`ament_cmake` + `ament_cmake_python`） | 台架夹具：假 mavros、假 PX4（现无 DUT）、脚本化动捕输入、被控对象模型、故障时序 | 没有 `ros2 run` 入口；由测试进程 import | `rclpy`、`geometry_msgs`、`sensor_msgs`、`std_msgs`、`mavros_msgs`、`px4_msgs` |
| `bringup` | Python launch + 脚本 | 启动层：按顺序拉起节点，提供操作台；不含飞行逻辑 | `ros2 launch bringup z0.launch.py`、`mavros.launch.py`；`ros2 run bringup fly.py`、`mavros_set_thrust_scaling.py` | `launch`、`launch_ros`、`rclpy`、`std_msgs`、`mavros_msgs`；exec 依赖 `vrpn_listener`、`mocap`、`traj`、`control` |
| `control` | C++ | 任务总管：权限与任务两维状态机、Offboard 位置控制、解锁/起飞/跟踪/降落 | `ros2 run control control_node`；launch `control.launch.py` | `rclcpp`、`mavros_msgs`、`geometry_msgs`、`sensor_msgs`、`std_msgs`、`std_srvs` |
| `mocap` | C++ | VRPN 刚体位姿 → 相对位姿（ENU/FLU），输出 vision pose 与两条控制用话题 | `ros2 run mocap mocap_rel_pose_node`；launch `mocap.launch.py` | `rclcpp`、`geometry_msgs`、`std_msgs`、`mavros_msgs` |
| `nmpc` | Python（调用 acados） | 模型预测控制节点 + 零 ROS 核心层 | `ros2 run nmpc mpc_node.py`、`offboard_smoke.py`、`preflight_check.py`；`python3 -m nmpc.timing`（模块运行） | `rclpy`、`geometry_msgs`、`std_msgs`、`nav_msgs`、`visualization_msgs`、`mavros_msgs`、acados 运行时 |
| `traj` | C++ | 四条解析参考轨迹：悬停、1 m 正方形、8 字、起飞；只发参考 | `ros2 run traj traj_node`；launch `traj.launch.py` | `rclcpp`、`geometry_msgs`、`std_msgs`、`mavros_msgs`、`visualization_msgs` |
| `vrpn_listener` | C++（vendored） | VRPN → ROS 2 桥：自动发现刚体并发布位姿、速度、加速度 | `ros2 run vrpn_listener vrpn_listener`；launch `sync_entity_state.launch` | `rclcpp`、`geometry_msgs`、`tf2`、系统 VRPN 库 |

「谁调用谁」不能只看依赖清单，还要看进程是谁起的：

- `bringup` 的 `z0.launch.py` 按需 include `mavros`，再依次 include `vrpn_listener`（可选）、`mocap`、`traj`、`control`；它不启动 `nmpc`。
- `run/z0.py` 是人工层：起链时按 `UP_CHAIN` 逐条起 `mavros`、`vrpn_listener`、`mocap`、`traj`、`control`；菜单里另有一项可以直接 `ros2 launch bringup z0.launch.py`。模型预测控制档再另起 `mpc_node.py`，并把 `control` 的 launch 参数设为 `output_mode:=external`。两档的差异在第 7 章展开。
- `bench` 不被任何飞行包依赖。它只被测试 import，并由测试用它起的假 mavros 与假动捕充当「外部世界」。
- 运行期的调用关系是话题，不是函数调用：`vrpn_listener → mocap → control`，`traj → control`，`control → mavros` 或 `control → nmpc → mavros`。

### 6.3.1 输入与定位：`vrpn_listener` 与 `mocap`

`vrpn_listener` 是 vendored 包：来源、commit 与 zip 校验和钉在 [`VENDORED.md`](../../../src/vrpn_listener/VENDORED.md)，逐文件 sha256 与上游一致。它自动发现 VRPN 刚体，为每个刚体发布 `/vrpn/<刚体>/pose`、`/twist`、`/accel` 三条话题。三处看似「可以顺手清理」的行为——时间戳用接收时刻 `now()`、角速度刻意留零、`frame_id` 走参数——都是上游原有设计，不能改；尤其 `now()` 是为了避开动捕主机与 Jetson 约 4 s 的时钟差，改回 `msg_time` 会毁掉 EKF2 融合。

`mocap` 是本地重写的节点。它订阅其中两条话题：位姿先平移到飞控 IMU 所在点，再减去两阶段原点（启动时锁一次、解锁时只归零 yaw）；速度则原样转发。然后分三路输出：`/mavros/vision_pose/pose` 喂 EKF2，`/mocap/pose_enu` 与 `/mocap/velocity_enu` 给控制器与记录。它**不做任何坐标转换**：ENU→NED 与 FLU→FRD 全部留给 mavros 在自己的边界上完成。这条分工让 `mocap` 可以只用纯函数测试，也避免同一套转换在两处各写一遍。

<big>**一句话总结：vrpn_listener 只搬运原始 VRPN，mocap 只做相对化与原点扣除，坐标转换全部留给 mavros。**</big>

### 6.3.2 任务与参考：`traj` 与 `control`

`traj` 只回答「第 t 秒该在哪、以多快的速度经过」。它提供四条解析轨迹：原地悬停、边长 1 m 的正方形、Gerono 8 字、以及从原点匀速直上 0.2 m 的起飞轨迹。`/traj/select` 选轨迹，`/traj/start` 定参考时钟零点；输出是 50 Hz 的 `/traj/reference`，另有 `/traj/progress` 与 RViz 用的 `/traj/marker`。参考**相对轨迹原点**，`traj` 不知道飞机在哪，也不碰 mavros。因此它可以在地面、没有飞控、没有动捕的条件下，把几何与限速跑一遍核对。

`control` 是任务总管。它把 `/traj/reference` 与轨迹原点相加，得到绝对目标，再按 `output_mode` 决定从哪里出去：`position` 档直接写 `/mavros/setpoint_raw/local`（位置 + yaw，`type_mask=2552`）；`external` 档只写 `/control/target`（绝对位置 + 速度 + yaw，`type_mask=2496`），由外部控制器接手。两种后端在构造时就**结构互斥**：`external` 档下根本不会创建 `/mavros/setpoint_raw/local` 的发布者，不是「创建了但发零」。`control` 还负责解锁、切 Offboard、起飞、跟踪、降落与失效停发；这些判据在零 ROS 的 `fsm.hpp`、`rc_stick.hpp`、`rc_switch.hpp` 里，节点层只做接线。

<big>**一句话总结：traj 只发相对参考，control 加回原点，并按 output_mode 决定自己发位置还是交给外部控制器。**</big>

### 6.3.3 控制器：`nmpc`

`nmpc` 是两层结构：`sign_map`、`commands`、`params`、`model`、`ocp`、`timing`、`node_core` 是零 ROS 核心层；`mpc_node.py`、`offboard_smoke.py`、`preflight_check.py` 是节点层入口脚本。核心层里，`ocp` 只输出内部 FLU 量 `u = [F, ωx, ωy, ωz]`，`sign_map` 负责把它变成下行字段；两者分开，是为了让「符号错了」与「模型解错了」可分别定位。

节点层按参数条件订阅：状态源固定为 `/mocap/pose_enu` 与 `/mocap/velocity_enu`，闸门源是 `/mavros/state`，参考源三选一（`hold`、`traj`、`control`），对应 `/traj/reference` 或 `/control/target`。输出只有一条执行器话题 `/mavros/setpoint_raw/attitude`：`body_rate` 为 FLU 原样、`thrust` 归一化到 0–1，`type_mask=128`（忽略姿态四元数）。它不 arm、不切模式、不读遥控器；这些动作留在 `control` 与 PX4。RViz 用的 `/mpc/predicted_path` 与 `/mpc/reference` 属于观测面，不进入飞行链。

<big>**一句话总结：nmpc 在核心层算目标与约束，节点层只把结果变成一帧总推力加机体角速率，不碰解锁与模式。**</big>

### 6.3.4 编排与测试：`bringup` 与 `bench`

`bringup` 只回答「按什么顺序、用什么参数把已有进程摆到台上」。`z0.launch.py` 默认 `use_mavros:=false`、`use_vrpn:=true`，依次拉起 `mocap`、`traj`、`control`（以及可选的 `vrpn_listener`），可选的 `mavros` 走本仓的薄封装 `mavros.launch.py`。`fly.py` 是操作台：只发 `/traj/select` 并显示 `/control/state` 与完成度；`/traj/start` 由 `control` 在进入跟踪时自己发。它没有解锁、切模式或直接发 setpoint 的入口。`mavros.launch.py` 额外挂 `mavros_set_thrust_scaling.py`，补上上游 `px4.launch` 因插件不读参数文件而漏掉的 `thrust_scaling`；这是起链补丁，不是飞行逻辑。

`bench` 是测试夹具，**不是飞行代码**。它提供假 mavros、假 PX4、脚本化动捕源 `VrpnSource`、假轨迹源 `FakeTraj`、被控对象 `Plant`、时间线 `Timeline` 与场景 `Scenario`，让节点级测试能在没有飞机、没有飞控、没有动捕棚的条件下跑。假 PX4 自 2026-10-13 起在本仓库已没有 DUT，只剩 `bench` 自测与 `Scenario(px4=True)` 的回退项会起它；节点级测试走假 mavros 的对应 setpoint 话题。它的安全边界是默认 `ROS_DOMAIN_ID=73` 且硬拒域 0，避免测试话题落到现场域。`bench` 没有可执行入口，也不被任何飞行包依赖；它只在 pytest 进程里被 import。

<big>**一句话总结：bringup 只把已有节点按顺序摆上台，bench 只搭测试用的假外部世界，两者都不产生飞行指令。**</big>

---

## 6.4 两条数据流：从 `/traj/reference` 到两个 setpoint 话题

两条控制路径共享前半段，在 `control` 的输出后端分叉。表 6-5 按「参考—任务层—控制器—载体—飞控」五段对照。

**表 6-5　两条数据流**

| 段 | offboard 位置档 | 外部控制器档（模型预测控制） |
|---|---|---|
| 1. 参考 | `traj` 发 `/traj/reference`（相对轨迹原点，50 Hz） | 同左 |
| 2. 任务层 | `control` 加轨迹原点，写 `/mavros/setpoint_raw/local`（`type_mask=2552`） | `control` 加轨迹原点，写 `/control/target`（绝对量，`type_mask=2496`） |
| 3. 控制器 | PX4 位置环直接消费位置 + yaw | `mpc_node` 的 `node_core` 做闸门与外推，`ocp` 求解，`sign_map` 映射为总推力 + 机体角速率 |
| 4. 载体 | mavros 做 ENU→NED 转换后转发位置 setpoint | `/mavros/setpoint_raw/attitude`（`type_mask=128`）→ mavros 做 FLU→FRD |
| 5. 飞控 | PX4 位置/速度环 → 姿态环 → 角速率环 → 混控 | PX4 角速率环 → 混控 |

两条路径的反馈也不同：位置档读 `/mavros/local_position/pose`（EKF2 输出约 30 Hz），外部控制器档读 `/mocap/pose_enu`（动捕原生约 190 Hz），不经过 EKF2；两个频率的测量口径在第 5 章。安全网（`/mavros/vision_pose/pose` → EKF2）仍然存在，但它只保证 PX4 还能估计位置、还能执行失效保护，不是控制器的备用状态源。

互斥不仅在话题上，也在进程上：`control` 的 `position` 档不会启动 `mpc_node`；`external` 档不创建位置 setpoint 发布者。外部控制器断流 0.5 s 时 `node_core` 自己关闸，PX4 的 Offboard 丢失保护（`COM_OF_LOSS_T=1.0 s`）是最后一道。两条路径都不依赖对方还活着。

<big>**一句话总结：两条路径共享 traj 与 control，分叉点在后端：自己发位置，或交给 nmpc 发推力与角速率。**</big>

---

## 6.5 模型预测控制的层分界：零 ROS 核心层与节点层

### 6.5.1 哪些模块属于核心层

`nmpc` 的分层按「有没有 ROS」切，不按文件类型切。核心层包括 `sign_map`、`commands`、`params`、`model`、`ocp`、`timing`、`node_core`；它们不许 import `rclpy`，也不许 import 任何具体消息包（`px4_msgs`、`mavros_msgs` 等）。这条约束的实际收益是：离线最优控制问题、台架夹具与纯函数测试都能直接 import 这些模块，不需要起 ROS 图。

`node_core.py` 是分界上的关键文件。它负责「这一拍发不发、发什么」：状态新鲜度闸门、状态外推、参考构造、断流与失效时的降级、有限值检查。它不 import acados，也不 import ROS，因此可以用普通 pytest 直接构造输入、断言输出。`ocp.py` 只负责求解并返回内部量；`sign_map.py` 只负责把内部量映射成下行字段。两件事分开后，符号错误与求解错误不会混在同一个失败里。

节点层是 `mpc_node.py`。它是唯一一处把求解结果变成话题上一帧的地方：声明参数、按档条件订阅、把消息搬进核心层、把核心层的输出填进 `AttitudeTarget`、打日志。`offboard_smoke.py` 与 `preflight_check.py` 也是入口脚本：前者把核心层接到话题上做最小冒烟，后者在装桨前做静态自检；它们不属于核心层。

「零 ROS」目前由测试里的 import 断言守着，但断言只点名 `rclpy` 与 `px4_msgs` 两个名字，还没有一条盯 `mavros_msgs`；这条缺口记在 [`票 13`](../../../.scratch/phase4/issues/13-mavros-downlink-switch.md) 的 Comments 里。分层的**设计**是完整的，**守卫**还有一处没补齐，读代码时不能把「现有断言全绿」当成「边界已经完全守住」。

<big>**一句话总结：核心层用纯函数决定发不发、发什么，节点层只做接线；零 ROS 边界尚缺 mavros_msgs 断言。**</big>

### 6.5.2 改哪一层要不要 build

判断依据不是「改的是哪个文件」，而是「这个进程 import 到的是哪一份」：

- **`ros2 run` 的入口脚本来自 `install/`。** `ros2 run nmpc mpc_node.py` 解析到 `install/nmpc/lib/nmpc/mpc_node.py`。常规安装是拷贝式，源码改动不会自动出现在安装目录；因此改 `mpc_node.py`、`offboard_smoke.py`、`preflight_check.py` **必须重新 `colcon build`**，否则跑的还是旧脚本。
- **包模块看 `PYTHONPATH`。** `import nmpc.node_core` 可能落到 `src/nmpc/nmpc/node_core.py`，也可能落到 `install/nmpc/local/lib/python3.10/dist-packages/nmpc/node_core.py`。跑场脚本（如 SITL 的 `run_matrix.sh`）会把 `src/nmpc` 前置到 `PYTHONPATH`，子进程继承这条路径；在这种上下文里改 `node_core.py`、`params.py`、`ocp.py` **不用 build** 就生效。反过来，只 source `install/` 再 `ros2 run`，读到的就是安装副本，仍然要 build。

安装方式会改变第一步的表现：仓库记录的常规安装是拷贝式；若用了 `--symlink-install`，`install/` 里是指向源码的软链，入口脚本的改动也会即时生效。判断仍以进程实际解析到的文件为准，不要依赖某一台机器的安装方式。

仓库里「`mpc_node.py` 要 build、`node_core.py` 不要」这条经验，成立的前提正是跑场脚本前置了源码路径。它不是一个无条件的全局规则；判断时先看这个进程的 `PYTHONPATH`，再决定要不要 build。判断错了的症状很隐蔽：旧的入口脚本配新的核心层，日志里会少掉新增的列或字段，读数却看起来「和以前一样」，容易被当成模型没变化。遇到这种情况，先核对进程实际 import 的文件，再谈控制效果。

<big>**一句话总结：入口脚本来自 install/，包模块看 PYTHONPATH 指向源码还是安装副本，所以 build 规则不同。**</big>

---

## 6.6 测试布局

### 6.6.1 包内测试：seam 1 与 seam 2

包内测试主体分成两个缝，另有两类辅助测试（见下）。**seam 2** 是纯逻辑：不起 ROS、不起节点，直接调用头文件或 Python 模块里的函数。`control` 的 `test_fsm.cpp`、`test_rc_stick.cpp`、`test_rc_switch.cpp`，`mocap` 的 `test_mocap_rel_pose.cpp`，`traj` 的 `test_trajectories.cpp`，以及 `nmpc` 的 `test_sign_map.py`、`test_commands.py`、`test_node_core.py`、`test_ocp_*.py`、`test_params.py` 都属于这一类。它们回答的是「给定输入，逻辑与数值对不对」。

**seam 1** 是节点级：把被测节点作为子进程起起来，用 `bench` 提供的假 mavros、假动捕与假轨迹喂话题，以话题断言为主，另看进程存活、退出码与日志。`control` 的 `test_control_standby.py`、`test_control_failsafe.py`、`test_control_takeoff.py`、`test_control_track.py`、`test_control_external_output.py`，`traj` 的 `test_traj_node.py`，`nmpc` 的 `test_mpc_node.py` 都属于这一类。它们回答的是「接线、QoS、频率、时序在真实消息面上对不对」。

`nmpc` 还另有 `test_mavros_interface.py`：它不需要 init ROS，也不起 DUT，断言字段名、`type_mask` 常量与启动参数，属于消息形状与接口契约测试。`test_preflight_check.py` 走另一条：被测的是纯逻辑、参数表与 `ast` 扫描，同样不起 DUT。

`bench` 自己也有自测：假 PX4 的行为、时间线、夹具初始化。它用 pytest 夹具把域号钉在 73，并硬拒域 0；同机跑多个夹具时要用 `colcon test --executor sequential`，否则会互相抢话题。`vrpn_listener` 是 vendored 包，没有测试，只保留 lint；`bringup` 只有 launch 与脚本，也没有测试——它的 CMake 注释明确说，再加一条「launch 能不能展开」的测试只会重复 `ros2 launch … --show-args`。

两个缝都不能替代实飞。seam 2 证明逻辑，seam 1 证明接线；真飞机的气动、振动、遥控链路与动捕遮挡不在任何夹具里。

<big>**一句话总结：包内 seam 2 直接调纯函数，seam 1 用 bench 夹具起节点子进程看话题，两者都不替代实飞。**</big>

### 6.6.2 仓库级测试：`run/tests/` 与 `tools/*/test_*.py`

`run/tests/` 守的是**操作层**，不是控制律。`test_operation_manual.py` 把现场一页纸里的菜单号、命令与话题名钉到 `z0.ACTIONS`、`z0.ACTION_BY_KEY`、`z0.PROBES`；`test_safety_paths_drill.py` 把安全路径演练文档钉到 `docs/assets/z0.params` 与真实入口点；`test_z0_menu.py` 测 `z0.py` 的纯函数；`test_rec.py` 测录制层的解析、拼装与安全边界；`test_z0_check_live.py` 则对着 `bench` 假 mavros 做正控，确认自检真的读到帧，而不是「满屏 ✓ 但零帧」。这一组测试的共同点是：能离线验的判据不留到现场。

`tools/*/test_*.py` 守的是**离线测量与数据处理**：电池推力、悬停推力、飞控参数、梯子基线、SITL 判据、符号映射、时间常数、RTI 收敛与报告数据管线。它们大多不需要飞机，有些需要已录的 bag 或 `.ulg`；它们回答的是「读数算得对不对、报告数据能不能复现」，不回答「控制律稳不稳」。

跑这些测试要先 source `/opt/ros/humble`、`~/px4_msgs_ws`、`~/mavros_ws` 与本仓的 `install/`；`bench` 在模块级 import `px4_msgs`；少了那个工作区，包内 seam 1 会在 collection 阶段报 `ModuleNotFoundError`，`run/tests` 与 `tools/*` 里依赖它的用例则会被 `importorskip` 跳过。全量测试还要设 `ACADOS_SOURCE_DIR`，否则 acados 会被猜成源码目录，几十个最优控制问题用例一起变红。这是环境问题，不是代码坏了。

<big>**一句话总结：run/tests 守操作文档与菜单脚本，tools 的测试守离线测量与报告数据，它们都不测控制律。**</big>

---

## 6.7 证据目录的角色

仓库里叫「证据」的东西分散在四个位置，职责不同，不能混放。

**表 6-6　证据目录**

| 目录 | 存什么 | 谁写 | 怎么用 |
|---|---|---|---|
| `bag/` | 场景目录的 rosbag 与 `session.json`；飞控 `.ulg` 若已归档则放在 `fcu/`；根目录有自动生成的总表 `MANIFEST.md`，`old/` 存归档；另有导出过 `.params` | 录制层 `run/rec.py` 与飞控 | 报告里的事故复现、频率与轨迹读数；不手改 |
| `tools/*/evidence/` | 各离线工具的输出：JSON、文本、原始扫描结果 | 对应工具 | 支撑参数辨识、SITL 判据、时间常数与报告数据；重跑工具生成，不手填 |
| `docs/assets/` | `z0.params` 参数快照、论文 PDF、演练记录、上游项目说明 | 人工整理 | 引用飞控参数与外部资料的**固定快照**；改动要留版本 |
| `docs/report/` | 报告章节、`data/`、`figures/`、`generated/`（图题等生成物） | 报告写作与数据管线 | 第 5 阶段的最终产物；结果数据与图由数据管线从 `data/` 生成 |

四类目录的共同纪律是：**原始记录只增不改，派生读数由工具重算**。`bag/` 里的一场飞行不能为了「让图好看」去改；`tools/*/evidence/` 里的数字不能手填，必须能由工具重跑；`docs/assets/z0.params` 是某个时点的参数快照，不是活飞控的实时值；`docs/report/generated/` 里的内容如果与 `data/` 对不上，以数据管线为准。

更细的逐场索引、文件清单与读数口径属于第 4 章、第 10 章与资料索引，本章只固定它们的角色与边界。

<big>**一句话总结：bag 存飞行记录，evidence 存读数，docs/assets 存参数与文献，docs/report 存报告产物。**</big>

---

## 6.8 本章引用清单

**表 6-7　本章引用清单**

| 主题 | 文件 |
|---|---|
| 包清单、依赖、入口 | [`src/*/package.xml`](../../../src/)、各包 `CMakeLists.txt` 与 `README.md` |
| 上游项目取舍与包来源 | [`docs/reference/code-inventory.md`](../../reference/code-inventory.md) |
| 模型预测控制的实现分层 | [`docs/control/phase2-implementation.md`](../../control/phase2-implementation.md)、[`src/nmpc/nmpc/__init__.py`](../../../src/nmpc/nmpc/__init__.py) |
| 节点入口与条件订阅 | [`src/nmpc/nmpc/mpc_node.py`](../../../src/nmpc/nmpc/mpc_node.py) |
| 零 ROS 决策逻辑 | [`src/nmpc/nmpc/node_core.py`](../../../src/nmpc/nmpc/node_core.py) |
| 下行映射 | [`src/nmpc/nmpc/sign_map.py`](../../../src/nmpc/nmpc/sign_map.py)、[`docs/control/sign-map.md`](../../control/sign-map.md) |
| 任务总管与两条输出后端 | [`src/control/README.md`](../../../src/control/README.md)、[`src/control/include/control/fsm.hpp`](../../../src/control/include/control/fsm.hpp) |
| 启动层与操作台 | [`src/bringup/README.md`](../../../src/bringup/README.md)、[`src/bringup/launch/z0.launch.py`](../../../src/bringup/launch/z0.launch.py) |
| 台架夹具与安全域 | [`src/bench/README.md`](../../../src/bench/README.md)、[`src/bench/bench/pytest_support.py`](../../../src/bench/bench/pytest_support.py) |
| 动捕相对位姿 | [`src/mocap/README.md`](../../../src/mocap/README.md) |
| 参考轨迹 | [`src/traj/README.md`](../../../src/traj/README.md) |
| VRPN 桥的来源与不可改点 | [`src/vrpn_listener/VENDORED.md`](../../../src/vrpn_listener/VENDORED.md) |
| 人工起链、录制与仓库级测试 | [`run/README.md`](../../../run/README.md)、[`run/tests/`](../../../run/tests/) |
| 录制数据的组织 | [`bag/README.md`](../../../bag/README.md) |
| 报告数据口径与复现 | [`docs/report/data/README.md`](../data/README.md) |
| 入口脚本与包模块的 build 规则 | [`.scratch/phase2/issues/20-vertical-channel-mismatch-offset.md`](../../../.scratch/phase2/issues/20-vertical-channel-mismatch-offset.md)、[`.scratch/phase4/issues/13-mavros-downlink-switch.md`](../../../.scratch/phase4/issues/13-mavros-downlink-switch.md) |
| 术语与书写约定 | [`CONTEXT.md`](../../../CONTEXT.md) |

<big>**一句话总结：本章每个结构结论都能回到仓库内的源码、包清单或第一手记录。**</big>
