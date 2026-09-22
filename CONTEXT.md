# CONTEXT —— 术语表

本文件是本项目的**唯一术语权威**。它只收录"词是什么意思"，不收录"怎么做"。
遇到歧义时以本文件为准；本文件与代码/文档冲突时，先改本文件再改代码。

---

## ⚠️ 最危险的一个歧义：`MPC`

**本项目里 `MPC` 一词指的是两个完全不同的东西，且 PX4 的命名与之撞车。**

| 写法 | 含义 | 出现在哪 |
|---|---|---|
| **MPC**（本项目语境） | **Model Predictive Control，模型预测控制** —— 用动力学模型在线求解有限时域最优控制问题 | 本文档、`MPC_uav` 仓库、acados、agilicious |
| **`MPC_*`**（PX4 参数前缀） | PX4 的 **Multicopter Position Controller**，即**级联 PID + 限幅**，**与模型预测控制毫无关系** | 飞控参数文件、QGC 参数页 |

**实例**：`MPC_THR_HOVER`、`MPC_XY_VEL_MAX`、`MPC_TILTMAX_AIR` 全都是 PX4 的 PID 控制器参数，**不是**模型预测控制的参数。`MPC_THR_HOVER` 读作"多旋翼位置控制器的悬停油门"，不是"MPC 求解器的悬停油门"。

**书写约定**：本项目中提到模型预测控制时，一律写全称或加限定："MPC（模型预测控制）"；引用 PX4 参数时一律写完整参数名并加反引号。**禁止**单独使用裸词 `MPC` 指代二者之一。

---

## 一、构型与机体几何

| 术语 | 定义 |
|---|---|
| **构型 (configuration / airframe)** | 旋翼数量、推力方向、机体几何的统称。本项目构型为**固定推力方向的四旋翼 X 型（Quad X）**。 |
| **X 型 / Quad X** | 四旋翼布局，机头方向落在两个电机之间，无正前方电机。PX4 机型编号 `4001`。 |
| **轴距 (wheelbase)** | **对角两个电机中心之间的距离**。本项目为 **152 mm**。注意：不叫"轮距"，也不是相邻电机间距。 |
| **臂长 (arm length)** | 机体中心到单个电机中心的距离 = 轴距 / 2。本项目为 **76 mm**。PX4 里体现为 `CA_ROTORx_PX/PY` 的组合模长。 |
| **机体系 (body frame)** | PX4 约定：**x 轴向前、y 轴向右、z 轴向下**（FRD, Forward-Right-Down）。 |
| **世界系 (world / NED)** | PX4 约定：**北-东-地**（NED）。动捕与 ROS 常用 **ENU**（东-北-天），两者相差一次坐标变换。 |
| **props-in / props-out** | 螺旋桨旋转方向布置的两种约定。**决定 MPC 模型里力矩 τ 的符号**，必须在文档里写死。 |
| **电机序 (motor order)** | 物理输出口编号 ↔ PX4 逻辑电机编号的映射。本项目飞控为 **`3-2-1-4`**（见 `docs/reference/params-analysis.md`）。 |
| **控制分配 (control allocation)** | 把期望的总推力与三轴力矩分配到各电机的过程。PX4 中由 `control_allocator` 模块与 `CA_*` 参数定义。 |
| **涵道机 (ducted / cinewhoop)** | 桨叶被环形涵道包围的构型。本项目属此类。**建模上按普通四轴处理**，涵道影响吸收进辨识出的推力/阻力系数。 |

## 二、动力与建模参数

| 术语 | 定义 |
|---|---|
| **推力系数 kt / KF** | 电机转速平方到推力的比例系数（N/(rad/s)² 或等价形式）。**本项目需系统辨识**。 |
| **扭矩系数 kq / KM** | 电机转速平方到反扭矩的比例系数。**本项目需系统辨识**。 |
| **悬停油门 (hover throttle)** | 悬停所需的归一化油门指令。**MPC 的工作点**，直接决定线性化点与推力可行域。 |
| **推重比 (T/W)** | 最大总推力 / 整机重量。**必须区分"电机能力上限的 T/W"与"电池能供出的 T/W"** —— 本项目两者差距巨大（5.44 vs 1.60），见 `docs/PROJECT.md` 第 2.4 节。 |
| **推力曲线 (thrust curve)** | 油门指令到实际推力的非线性映射。PX4 用 `THR_MDL_FAC` 参数化：`throttle = (1-c)·u + c·u²`。 |
| **转动惯量 (inertia, Ixx/Iyy/Izz)** | 机体三轴转动惯量。**本项目需系统辨识或 CAD 估算**。 |
| **电压跌落 (voltage sag)** | 大电流下电池内阻导致的端电压下降。**本项目不可忽略**（见 `docs/PROJECT.md` 第 2.4 节）。 |
| **系统辨识 (system identification, sysid)** | 从实验数据反推模型参数的过程。用户已有**电机阶跃响应两点法**的经验。 |
| **sim-to-real** | 把仿真里调好的策略迁移到真机。本项目的主要 gap 来源：推力曲线、电池跌落、动捕时延。 |

## 三、控制架构

| 术语 | 定义 |
|---|---|
| **Offboard 模式** | PX4 的一种飞行模式，控制指令由机载电脑（而非遥控器）给出。 |
| **外环 / 内环** | 外环处理位置与姿态，带宽低；内环处理角速率，带宽高。本项目 **MPC 在外环（100 Hz），PX4 角速率环在内环（1 kHz）**。 |
| **CTBR** | **Collective Thrust + Body Rate**。控制器输出"总推力 + 机体三轴角速率期望值"的形式。**是 `agilicious` 与 `px4-mpc` 共同采用的下发口径**，也是本项目 `(C)` 方案的形式。 |
| **`(C)` 方案** | 本项目对"MPC 输出形态"的编号，指 **MPC → `VehicleRatesSetpoint`（ω + 推力）→ PX4 角速率环**。编号 `(A)(B)(C)(D)` 的定义见 `docs/control/mpc-roadmap.md`。 |
| **INDI** | **Incremental Nonlinear Dynamic Inversion**，增量式非线性动态逆。**不依赖精确模型**的内环控制方法（用当前角加速度与操纵效能矩阵做增量更新），可替代 PID 做角速率跟踪。 |
| **几何控制 (geometric control)** | 在 SO(3) 流形上直接设计姿态控制的非线性方法。是 `agilicious` 的控制器之一。 |
| **NMPC** | 非线性模型预测控制（区别于线性 MPC）。 |
| **MPCC** | **Model Predictive Contouring Control**，模型预测轮廓控制 —— 优化"沿参考路径的进度"而非"点到点误差"，是竞速飞行的主流方法。 |
| **微分平坦 (differential flatness)** | 四旋翼的一类性质：全部状态与输入可由四个平坦输出（位置 xyz + 偏航）及其导数代数地表示。常用于生成前馈轨迹。 |
| **DFBC** | Differential-Flatness-Based Control，基于微分平坦的控制。`Sun2022_TRO_NMPC_vs_DFBC` 论文中作为 NMPC 的对照组。 |
| ★ **控制权 (authority)** | 自主飞控状态机的**第一个维度**：**谁在控制飞机**。`MANUAL` / `AUTO` 两值。**定义**：`AUTO` = 本项目正在发**会驱动飞机**的指令（即已解锁的 offboard）。⟹ "已切进 OFFBOARD 模式但飞机未解锁"**仍是 `MANUAL`** —— 切模式 ≠ 交出控制权。 |
| ★ **任务态 (phase)** | 状态机的**第二个维度**：**程序在干什么**。取值 `INIT` / `STANDBY` / `TAKEOFF` / `HOVER` / `TRACK` / `LAND` / `RC_OVERRIDE`。★ 与「控制权」**正交、不压成扁平枚举**（两个维度各自独立变化）。`/control/state` 发 `"<控制权>/<任务态>"`，如 `MANUAL/STANDBY`。早期 ticket 里画图时用的 `IDLE` 与代码里的 `INIT` 同义；`RC_OVERRIDE` 是 ticket 06（2026-09-22）新增的：**控制权被外部夺走**（遥控器打杆，或模式被切出 OFFBOARD）⟹ 本节点已停止发指令且**不会主动抢回**。 |
| ★ **失效降级 (degrade)** | 任一**失效判据**成立（输入超时 / 打杆接管 / 模式被夺走 / 低压 / 位置读数出现 NaN/Inf / 动捕时间戳不前进）⟹ 本节点**立即停止发 setpoint**；飞机**已解锁**时并主动请求 `AUTO.LAND`（**夺权类**除外 —— 打杆接管 `rc_override` 与我们持有的模式被外部切走 `mode_lost` 都算：那时控制权已交还飞行员，**不请求任何模式**，连 `AUTO.LAND` 也不请求）。⚠️ **`mode_lost` 的定义必须收窄到"我们正在驱动的那个模式被切走"**（`OFFBOARD` **且**我们在流 setpoint；或我们自己请求的 `AUTO.LAND`）：因为我们一停发，PX4 会在 `COM_OF_LOSS_T`（1.0 s）后**自己**退出 `OFFBOARD`，那个形状与"人被夺权"一模一样，而我们的判据（0.5 s）比它先到 ⟹ 真机上必然先看见它。把这种"自己退出"误判成夺权的代价：日志指向错的排查方向，且夺权会**升级**已闩锁的输入失效 ⟹ 连 `AUTO.LAND` 都不再重发。日志里以 `degrade=<原因>` 标记（如 `degrade=battery_low`），可 grep。★ **不是单向 latch**：复位条件见 `docs/control/phase1-implementation.md` §4.4（落地/上锁后自动复位；未解锁时的降级在判据恢复后自动复位）。⚠️ 上面那条"位置读数"**不等于**「EKF2 位置失效」：本链条上 EKF2 的位置失效**不会**以 NaN 出现（第一手核实，见 §4.4），真兜底是**动捕超时**。 |
| **OFFBOARD 前置流 (pre-stream)** | **切进 OFFBOARD 之前**必须先把 setpoint 连续流够一段时间（本项目 50 Hz 流 ≥1 s）—— 因为 **PX4 在 OFFBOARD 模式下一旦断流超 `COM_OF_LOSS_T` 就自行退出该模式**，所以"先流后切"是前提而不是优化。 |
| ★ **轨迹原点系 (trajectory origin frame)** | 以 **HOVER 判稳那一刻 EKF2 报的位置为原点**、朝向与 EKF2 局部系一致（ROS 侧 ENU，z 向上）的坐标系。**`traj` 发的参考全部在这个系里**——所以轨迹 1（悬停）的目标是 `(0,0,0)`：**1 m 高度属于原点、不属于轨迹**，在轨迹里再写 1.0 会变成 2 m。把相对参考加回绝对 setpoint 是 `control` 的**唯一职责**（ticket 09）。⚠️ 与**动捕原点**不是一回事，Phase 1 完全不依赖后者（`docs/control/phase1-implementation.md` §3.3）。 |
| ★ **参考完成度 (progress)** | `traj` 报的 0–1 标量（`/traj/progress`），驱动 §4.2 的 `TRACK → LAND`（`≥ 1.0`）。**悬停没有终点** ⟹ 用 `hover_duration_s` 的时长兜（默认 60 s = §5.3 的验收时长）。 |

## 四、求解器与工具链

| 术语 | 定义 |
|---|---|
| **acados** | 基于 CasADi 的嵌入式最优控制求解器，**C 实现**，自动生成求解器代码。本项目 MPC 的求解器由它生成。 |
| **CasADi** | 符号建模与自动微分框架（Python/C++）。用于定义 OCP。 |
| **HPIPM** | acados 默认的 QP 求解器（内点法），利用 OCP 的带状结构。 |
| **BLASFEO** | acados 使用的高性能线性代数库，**按 CPU 微架构编译**（Jetson 上需指定 ARM 目标）。 |
| **SQP_RTI** | **Real-Time Iteration** —— 每周期只做一次 SQP 迭代（分 preparation / feedback 两相），延迟极低，适合硬实时 MPC。本项目的选择。 |
| **OCP** | Optimal Control Problem，最优控制问题。 |
| **Tera / `t_renderer`** | acados 代码生成用的模板引擎二进制，**安装缺失时报 `EOFError`**，是 acados 最常见的坑。 |

## 五、状态估计与接入

| 术语 | 定义 |
|---|---|
| **动捕 (motion capture, mocap)** | 外部光学定位系统。本项目用 **Nokov**，经 VRPN 协议输出刚体位姿。 |
| **刚体 (rigid body)** | 动捕软件里标定的、由若干反光球构成的可跟踪目标。每个刚体有一个原点（质心位置），该原点与飞控 IMU 的偏移即**外参**。 |
| **外参 (extrinsics)** | 两个坐标系之间的固定变换。这里特指**动捕刚体原点 → 飞控 IMU** 的平移偏移，对应 PX4 参数 `EKF2_EV_POS_X/Y/Z`。 |
| **两阶段原点** | `mocap` 节点的原点策略：**节点启动**锁一次（保证解锁前 EKF2 就有视觉数据、已收敛），**解锁上升沿**再锁一次（"起飞前静止帧"）。两阶段的**切换量 = 这段时间里飞机被移动的量** ⟹ 运行纪律是「摆好后到解锁前别动它」。切换后**永不重锁**（飞行中重锁 = 给 EKF2 一记位置跳变）。 |
| **启动原点 (boot origin)** | 两阶段里的**阶段 A** 原点 = 节点启动后前 `boot_samples` 帧的**位置均值**。此阶段姿态**不**归零 yaw（保持绝对）。 |
| **起飞前静止帧原点 (arm origin)** | 两阶段里的**阶段 B** 原点 = **解锁上升沿后** `arm_samples` 帧（≈0.3 s）的位置均值，以及同一段姿态均值里的 **yaw**。此阶段位置减该原点，姿态**只归零 yaw**（roll/pitch 保持绝对）。 |
| **VRPN** | Virtual Reality Peripheral Network，动捕数据传输协议。默认端口 3883。 |
| **EKF2** | PX4 的扩展卡尔曼滤波器，**当前唯一在用的估计器**（LPE 已废弃）。 |
| **LPE** | Local Position Estimator，PX4 的旧估计器，**已不再使用**。 |
| **外部位姿 (external vision, EV)** | 由机外系统（动捕/VIO）提供的位姿，经 MAVLink 送入飞控。 |
| **`VISION_POSITION_ESTIMATE`** | 唯一能被 **EKF2** 使用的动捕位姿消息之一（另一个是 `MAV_FRAME_LOCAL_FRD` 的 `ODOMETRY`）。 |
| **`ATT_POS_MOCAP`** | 另一条动捕消息，**只对 LPE 有效，EKF2 不使用**。网上大量教程误用它。 |
| **uXRCE-DDS** | PX4 v1.14+ 的默认 ROS2 中间件。话题前缀 `/fmu/in/...` 与 `/fmu/out/...`。 |
| **mavros** | MAVLink 的 ROS 封装。ROS1/ROS2 均有。本项目动捕链路使用它。 |
| **`px4_ros_com`** | PX4 旧版 ROS2 桥（FastRTPS），**v1.14 起已废弃**，现仅作为示例保留。 |

## 六、平台与硬件专有名词

| 术语 | 定义 |
|---|---|
| ★ **z0** | **本项目这台无人机的正式名称。** 构型 = OddityRC XI35 Pro 机架（152 mm 轴距）+ 微空 NxtPX4v2 飞控 + Jetson Orin NX 8GB。**全部文档中一律写 `z0`**，不再用"新机/本机/这台机"等代称。当前状态：只遥控器飞过，动捕链路与 MPC 均未上机。 |
| **旧机** | 超维空间 S2-F290（PX4 **1.13.3** + 树莓派 4B）。**动捕定点飞行是在它上面验证通过的**——z0 上的一切动捕经验都源自它，需要重做一遍。见 `docs/PROJECT.md` 1.2。 |
| **NxtPX4v2 / Nxt-FC** | 微空科技（MicoAir）量产的 PX4 飞控，也被 HKUST 的 OmniNxt 采用。PX4 目标板名 `hkust_nxt-dual`。**z0 用的是它。** |
| **XI35 Pro** | 怪象（OddityRC）的 3.5 英寸涵道机架，轴距 152 mm。**z0 的机架。** |
| **`z0.params`** | **z0 的飞控参数文件**（`assets/z0.params`，928 行，其中 914 条参数）。内容逐字来自上游 `Huan-qun/Sim2real-UAV-Params-` 的 `S2R/assets/param/uav1.params`（那台机器叫 "Vehicle 1"），**尚未在 z0 的飞控上核实**——所以文档里凡说"`z0.params` 里 X 是默认值/可疑"，意思都是"**这是上游值，别当成 z0 的实测值**"。见 `docs/reference/params-analysis.md`。 |
| **动捕定点飞行** | 用动捕位姿闭环、让飞机自主保持位置悬停的飞行状态。**旧机上已实现，z0 上尚未。** |

## 七、测试与台架

| 术语 | 定义 |
|---|---|
| **接缝 (seam)** | 测试切入的边界。**接缝 1** = **ROS 话题/服务边界**（喂入向话题、断言出向话题），节点级测试走这条；**接缝 2** = **纯函数**（不碰 ROS，如坐标变换、时序逻辑）。★ 规矩：**能在接缝 2 测的就别拉到接缝 1**，能上接缝 1 的就别去动内部类。 |
| **台架夹具 (bench fixture)** | `src/bench/` 提供的测试基础设施（**不是飞行代码**）：一次"被控对象在环"的场景 —— 假 mavros + 脚本化动捕 + 故障注入时序 + **DUT 真子进程**。被测节点当**进程**启动，所以"进程被杀 ⟹ 流停止"这类判据才可测。 |
| ★ **假 mavros (fake mavros)** | 夹具里假扮 **mavros 的行为**（话题名、频率、**QoS**、`/mavros/state` 1 Hz、`mode_sent` 语义），**不假扮 PX4 的行为**（`COM_OF_LOSS_T` 退出、EKF2 融合、模式需求检查）。⚠️ QoS 必须逐条对齐真 mavros，**不兼容时不报错、只是永远收不到**。 |
| **脚本化动捕 (scripted mocap)** | 夹具里按秒表喂位姿的发布者，两种接法：喂 `/vrpn/<刚体>/pose`（上游，`mocap` 节点在回路里）或直接喂 `/mocap/pose_enu`（下游，旁路 `mocap`）。**动捕主机、VRPN、路由器都不该出现在节点级测试里。** |
| **DUT** | Device Under Test，被测节点。台架里它以**子进程**运行，绝不是 import 它的类。 |
