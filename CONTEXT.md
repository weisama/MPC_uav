# CONTEXT —— 术语表

本文件是本项目的**唯一术语权威**。它只收录"词是什么意思"，不收录"怎么做"。
遇到歧义时以本文件为准；本文件与代码/文档冲突时，先改本文件再改代码。

---

## ⚠️ 最危险的一个歧义：`MPC`

**本项目里 `MPC` 一词指的是两个完全不同的东西，且 PX4 的命名与之撞车。**

| 写法 | 含义 | 出现在哪 |
|---|---|---|
| **MPC**（本项目语境） | **Model Predictive Control（MPC）** —— 用动力学模型在线求解有限时域最优控制问题 | 本文档、`MPC_uav` 仓库、acados、agilicious |
| **`MPC_*`**（PX4 参数前缀） | PX4 的 **Multicopter Position Controller**，即**级联 PID + 限幅**，**与 MPC 控制器毫无关系** | 飞控参数文件、QGC 参数页 |

**实例**：`MPC_THR_HOVER`、`MPC_XY_VEL_MAX`、`MPC_TILTMAX_AIR` 全都是 PX4 的 PID 控制器参数，**不是** MPC 的参数。`MPC_THR_HOVER` 读作"多旋翼位置控制器的悬停油门"，不是"MPC 求解器的悬停油门"。

**书写约定（2026-10-18 操作者裁定）**：本项目中提到 MPC 时一律写 `MPC`，首次出现可写 `Model Predictive Control（MPC）`。引用 PX4 参数时一律写完整参数名并加反引号，并说明它是 Multicopter Position Controller，与 MPC 控制器无关。旧约定「禁止裸词 `MPC`」即日起作废。

**★ 2026-10-10 的历史清扫记录（已被 2026-10-18 裁定取代，仅存档）**：当时把所有**指代本项目控制器**的裸用 `MPC` 改成 `MPC（模型预测控制）`（每份文件首处）或全称 `模型预测控制`（其余）。**刻意没动**四类 —— ① 讲这个词本身的**指称**（本节、`docs/control/mismatch-sweep.md`、各 spec 的"禁止裸词"条款）；② **论文标题**与归档文件 [`docs/assets/MPC_uav-README.md`](docs/assets/MPC_uav-README.md)；③ **上游引文**（如 `open-source-survey.md` 引 `px4-mpc` 的原话）；④ 标识符 `MPC_uav` / `MPC_CODEGEN_DIR` 与 PX4 的 `MPC_*` 参数。
复扫命令：`grep -rnP '(?<![A-Za-z_/\-])MPC(?![A-Za-z_（\-\d])' --include='*.md' --include='*.yaml' --include='*.cpp' .` —— 剩余命中**全部属于上面四类**（逐条理由与例外清单见清扫的那次提交说明）。★ **受影响的旧票已就地加注**（`phase3/02`、`phase3/04`、`phase2/03`、`phase2/15`、`phase2/19`、`phase3/03`）：它们引的是**清扫前的原文**。

---

## 一、构型与机体几何

| 术语 | 定义 |
|---|---|
| **构型 (configuration / airframe)** | 旋翼数量、推力方向、机体几何的统称。本项目构型为**固定推力方向的四旋翼 X 型（Quad X）**。 |
| **X 型 / Quad X** | 四旋翼布局，机头方向落在两个电机之间，无正前方电机。PX4 机型编号 `4001`。 |
| **轴距 (wheelbase)** | **对角两个电机中心之间的距离**。本项目为 **152 mm**。注意：不叫"轮距"，也不是相邻电机间距。 |
| **臂长 (arm length)** | 机体中心到单个电机中心的距离 = 轴距 / 2。本项目为 **76 mm**。PX4 里体现为 `CA_ROTORx_PX/PY` 的组合模长。 |
| **机体系 (body frame)** | PX4 约定：**x 轴向前、y 轴向右、z 轴向下**（FRD, Forward-Right-Down）。 |
| **机体系（MPC 内部）** | **FLU**：x 朝前、**y 朝左**、**z 朝上（沿推力方向）** —— 与项目依据的两篇论文一致。★ 它与上面那个 PX4 的 FRD 是**同一台飞机的两套标注**（x 相同，y 与 z 取反），两者之间的映射表**全项目只有一处权威**：[`docs/control/sign-map.md`](docs/control/sign-map.md)。 |
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
| **悬停油门 (hover throttle) / `hover_thrust`** | 悬停所需的**归一化推力**：`hover_thrust ≡ T_hover/T_max = m·g/T_max`，即满推力占的比例。★ 在 `thrust_body` 通路上它是**推力**、不是油门 —— 从归一化推力再往下到电机信号，才是「推力曲线」（`THR_MDL_FAC`）那一层；两者混为一谈就是**静默的增益错误**。**MPC 的工作点**，直接决定线性化点与推力可行域。 |
| **推重比 (T/W)** | 最大总推力 / 整机重量。**必须区分"电机能力上限的 T/W"与"电池能供出的 T/W"** —— 本项目两者差距巨大：电机/桨 **6.29**（2026-09-29 悬停日志实测，满量程 46.30 N；旧外推 5.44）vs 电池 **1.60 ~ 2.54**（15 A / 30 A 外推）。★★ **这两个口径对应两个不同的代码常数，别合并**：归一化常数 `t_max` 按**电机**（= 实测满量程 46.30 N）、`F` 上限 `f_max` 按**电池**（11.8 N）。⚠️ 2026-09-29 之前两者**是同一个** `T_max`，回填实测 `hover_thrust` 时顺手把 `F` 上限抬了 3.9 倍而测试全绿（[`phase2-implementation.md`](docs/control/phase2-implementation.md) §3.3.3）。见 `docs/PROJECT.md` 第 2.4 节、[`hardware.md`](docs/reference/hardware.md) §6.3 ③。 |
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
| **`(C)` 方案** | 本项目对"MPC 输出形态"的编号：**MPC 输出「角速率 + 推力」，PX4 只跑角速率环 + 混控**。编号 `(A)(B)(C)(D)` 的定义见 [`docs/control/mpc-roadmap.md`](docs/control/mpc-roadmap.md)。<br>☠️ **2026-10-13（票 13 收口轮）**：原写"→ `VehicleRatesSetpoint`（ω + 推力）→" —— 那是**消息类型**，不是方案的定义，而且**已不是本项目的载体**（实机走 **mavros 的 `/mavros/setpoint_raw/attitude`**：`body_rate.x/y/z` + 标量 `thrust`；`VehicleRatesSetpoint` / uXRCE-DDS 那条路自 2026-10-13 起废）。⟹ **方案没变，载体变了**；引用本词条时"`(C)`"= **输出形态**，⛔ 别当成某条具体话题。 |
| **INDI** | **Incremental Nonlinear Dynamic Inversion**，增量式非线性动态逆。**不依赖精确模型**的内环控制方法（用当前角加速度与操纵效能矩阵做增量更新），可替代 PID 做角速率跟踪。 |
| **几何控制 (geometric control)** | 在 SO(3) 流形上直接设计姿态控制的非线性方法。是 `agilicious` 的控制器之一。 |
| **NMPC** | 非线性 MPC（区别于线性 MPC）。 |
| **MPCC** | **Model Predictive Contouring Control**，模型预测轮廓控制 —— 优化"沿参考路径的进度"而非"点到点误差"，是竞速飞行的主流方法。 |
| **微分平坦 (differential flatness)** | 四旋翼的一类性质：全部状态与输入可由四个平坦输出（位置 xyz + 偏航）及其导数代数地表示。常用于生成前馈轨迹。 |
| **DFBC** | Differential-Flatness-Based Control，基于微分平坦的控制。`Sun2022_TRO_NMPC_vs_DFBC` 论文中作为 NMPC 的对照组。 |
| ★ **控制权 (authority)** | 自主飞控状态机的**第一个维度**：**谁在控制飞机**。`MANUAL` / `AUTO` 两值。**定义**：`AUTO` = 本项目正在发**会驱动飞机**的指令（即已解锁的 offboard）。⟹ "已切进 OFFBOARD 模式但飞机未解锁"**仍是 `MANUAL`** —— 切模式 ≠ 交出控制权。 |
| ★ **任务态 (phase)** | 状态机的**第二个维度**：**程序在干什么**。取值 `INIT` / `STANDBY` / `TAKEOFF` / `HOVER` / `WAIT_START` / `TRACK` / `LAND` / `RC_OVERRIDE`。★ `WAIT_START` 是 ticket 08（2026-09-22）新增的：**已判稳、轨迹原点已记录**，目标 = 原点本身（停在原地）。★★ **ticket 09（2026-09-23）改了它的语义**：不再"保持 `hover_hold_s` 后自动降落"（那条参数**已不存在**），而是**等开始键的第 2 次上升沿**进 `TRACK`；兜底超时是 `wait_start_timeout_s`（默认 300 s）。`TRACK` 跑完（或超时）才走 `LAND`。它的存在让"任务正常走完"与"失效降级"两条 `LAND` 在任务态上就分开，而不是到日志里才分。★ 与「控制权」**正交、不压成扁平枚举**（两个维度各自独立变化）。`/control/state` 发 `"<控制权>/<任务态>"`，如 `MANUAL/STANDBY`。早期 ticket 里画图时用的 `IDLE` 与代码里的 `INIT` 同义；`RC_OVERRIDE` 是 ticket 06（2026-09-22）新增的：**控制权被外部夺走**（遥控器打杆，或模式被切出 OFFBOARD）⟹ 本节点已停止发指令且**不会主动抢回**。 |
| ★ **失效降级 (degrade)** | 任一**失效判据**成立（输入超时 / 打杆接管 / 模式被夺走 / 低压 / 位置读数出现 NaN/Inf / 动捕时间戳不前进 / **EKF2 自报水平位置没有绝对参考** `ekf2_pos_lost`）⟹ 本节点**立即停止发 setpoint**；飞机**已解锁**时并主动请求 `AUTO.LAND`（**夺权类**除外 —— 打杆接管 `rc_override` 与我们持有的模式被外部切走 `mode_lost` 都算：那时控制权已交还飞行员，**不请求任何模式**，连 `AUTO.LAND` 也不请求）。⚠️ **`mode_lost` 的定义必须收窄到"我们正在驱动的那个模式被切走"**（`OFFBOARD` **且**我们在流 setpoint；或我们自己请求的 `AUTO.LAND`）：因为我们一停发，PX4 会在 `COM_OF_LOSS_T`（1.0 s）后**自己**退出 `OFFBOARD`，那个形状与"人被夺权"一模一样，而我们的判据（0.5 s）比它先到 ⟹ 真机上必然先看见它。把这种"自己退出"误判成夺权的代价：日志指向错的排查方向，且夺权会**升级**已闩锁的输入失效 ⟹ 连 `AUTO.LAND` 都不再重发。日志里以 `degrade=<原因>` 标记（如 `degrade=battery_low`），可 grep。★ **不是单向 latch**：复位条件见 `docs/control/phase1-implementation.md` §4.4（**可信的落地上锁**后自动复位；未解锁时的降级在判据恢复后自动复位）。★★「**可信的落地上锁**」= `connected=true ∧ armed=false`（ticket 18，2026-09-22）：`armed` 的降沿**只从可信帧上认** —— mavros 在链路静默满 `conn_timeout`（参数，默认 10 s）后会**自己发一帧** `connected=false, armed=false, guided=false, manual_input=false, mode=""`、`system_status=UNINIT`（`sys_status.cpp:903-911` 的 `publish_disconnection()`，`:1411` 触发；mavros 2.15.1 @ `22ae5b7`），那帧里的 `armed=false` 是**占位值**而不是读数（它只说"现在不知道飞控怎么样"）。凭它复位 = 把闩锁**在空中**清掉，而链路恢复时若 PX4 仍 armed（我们停发后它多半已自行退出 OFFBOARD），节点就会走上"重新准备接管"那条路（★ 当前 `kArmingByUs` **恰好也**挡着它 —— 那帧的 `armed=false` 顺手清了归属标记 —— 但那是**偶然**的安全网，判据必须自己挡住；见 §4.4）。⚠️ 代价（刻意接受）：真出现"链路断了之后飞机在地上被人工上锁"时我们也**看不到**那次上锁，闩锁要等链路恢复、重新看到 `connected=true ∧ armed=false` 才复位（实测就是几秒）。⚠️ 上面那条 NaN 判据**不等于**「EKF2 位置失效」：本链条上 EKF2 的位置失效**不会**以 NaN 出现（第一手核实，见 §4.4）⟹ 它只是**防御性**的、真机永不触发。★★ 「EKF2 位置失效」的**可用载体是 `ekf2_pos_lost`**（`/mavros/estimator_status` 报 `pos_horiz_abs` 连续为假，**只在已解锁时算失效**，ticket 16 落实，2026-09-22）；但它**仍然不是兜底** —— 上游 1 Hz，加上 EKF2 自己清位要 0.4~1 s ⟹ 最坏 ~2.5–3 s，而**真兜底是动捕那两条**（`mocap_timeout` / `mocap_stamp_stalled`，0.5 s，动捕是唯一位置源）。`ekf2_pos_lost` 补的是"动捕帧正常到、`local_position` 照发、EKF2 却拒绝用"这一类场面。 |
| ★ **程序解锁 (arming by software)** | 由机载电脑调 `/mavros/cmd/arming` 解锁，**不靠遥控器拨杆**（`phase1-implementation.md` §H.6 的 **D3**）。★ 前置 = **已在 OFFBOARD** + setpoint 已在流（H.6 原文两条）+ **遥控在线**（§H.7.1，机器判据）。⚠️ **被拒或无应答 ⟹ 放弃本次起飞、退回 `STANDBY`，不自动重试**：静默重试会把"参数没配对"这类确定性拒绝变成一串没人看的日志。 |
| ★ **起飞命令 (`/control/takeoff`)** | `std_srvs/Trigger` 服务（ticket 08），★★ **ticket 09 起不再是唯一入口**：现场与实飞走的是**遥控器开始键（CH9）的第 1 次上升沿**，服务那条是调试与**拆桨台架**用的（台上没有遥控接收机）。两条走**同一个受理路径** `Fsm::request_takeoff`，判据逐条相同。用服务而不是话题：命令**要有应答** —— `success` 是布尔，`message` 以固定词表 `accepted` / `already_flying`（`to_string(TakeoffReply)`）**开头**、后接"缺哪一条前置"的人话（现场脚本可 grep 前缀），因为操作员按了没反应时第一个问题永远是"什么缺了"。★ **受理 ≠ 起飞**：受理之后前置不齐就一帧都不发解锁请求，由 `takeoff_timeout_s` 兜底作废。本节点**不读遥控器**（ticket 09 会把遥控器 ch9 的上升沿接到这里）。 |
| **OFFBOARD 前置流 (pre-stream)** | **切进 OFFBOARD 之前**必须先把 setpoint 连续流够一段时间（本项目 50 Hz 流 ≥1 s）—— 因为 **PX4 在 OFFBOARD 模式下一旦断流超 `COM_OF_LOSS_T` 就自行退出该模式**，所以"先流后切"是前提而不是优化。 |
| ★ **闸门 (gate)** | Phase 2 节点里"**这一拍到底发不发**"的判据层。它**在求解之前**：闸门关着 ⟹ 连状态外推都不做、**一帧 setpoint 都不发**（心跳照发）。关闸门的判据分两类：**超时**（动捕 / 飞控状态多久没到）与**输入合理性**（`stamp_regress` / `pose_jump` / `attitude_jump`，见 [`docs/control/mpc-node-fallback.md`](docs/control/mpc-node-fallback.md) §4）。⚠️ **闸门关 ≠ 飞机松手**：PX4 会把最后一帧 setpoint **无限期锁存**下去（第一手：`MulticopterRateControl.cpp:177-182` 把值存在成员变量里），所以"不发"在飞控侧等于"继续执行一条陈旧命令"。★★ 这与**兜底阶梯**是两件事：阶梯换的是**帧的内容**（照发），闸门管的是**发不发**。 |
| ★ **兜底阶梯 (fallback ladder)** | 求解不可用时**主动换帧**的三档：`normal`（正常解）→ `hold`（**保持上一帧可行指令**，内容原样、时间戳换新）→ `hover`（安全悬停）。判据 = **连续**失败帧数 > `fallback_hold_frames`（默认 **10 拍 = 100 ms**）。★ 为什么不能"不发"、为什么不能"一失败就悬停"：见 [`docs/control/mpc-node-fallback.md`](docs/control/mpc-node-fallback.md) §2。⚠️ 分界线是 `SolveInfo.ok`（状态码非零才进阶梯）—— **`degraded` 但 `ok` 的退化解照样发**（ticket 06 的现行行为）。 |
| ★ **输出后端 (output backend)** | `control` 的**输出通路选择**（`output_mode` 参数，票 15）：任务流程（开始键 / 解锁 / 切 OFFBOARD / 起飞 / 判稳 / 轨迹原点 / `/traj/start` / 降落）全部不变，只换"谁最后给 PX4 发指令"。`position` = `control` 自己写 `/mavros/setpoint_raw/local`；`external` = `control` 写内部绝对目标 `/control/target`，由另一个控制节点（当前是 `mpc_node`，`reference_source=control`）写执行器话题。★ 结构互斥：`control` 在构造时**只建一条**后端的发布者，不存在两路同时出帧。 |
| ★ **安全悬停 (safe hover)** | 兜底阶梯的最后一档：**悬停推力 + 零角速率**（`t_hover` / `t_max`，归一化推力分量 −0.1589；2026-09-29 前的占位值是 −0.6235）。★ 它**不依赖任何状态**（推力沿机体系 z、角速率置零）⟹ 即使"状态不可信"也成立，这是它区别于 `hold` 的性质。★ `hold` 退化为 `hover` 的边界：**第一拍就不可用**（那时没有"上一帧可行指令"）—— 若沉默，飞控的 `_thrust_setpoint{}` 是**零初始化**的 ⟹ 零推力掉下来（`MulticopterRateControl.hpp:124,127`）。 |
| ★ **故障注入 (fault injection)** | 人为把故障塞进被测系统，用来验证"出故障时它怎么做"。本项目**两条路**：**夹具侧**（`bench/timeline.py` 的配方：静默 / 改状态 / 动捕跳变…—— 故障来自"外部世界"，这是默认做法）与 **DUT 侧参数**（`-p inject_solve_fail_after_frames:=N`，默认 0 = 关）。★★ 为什么只在极少数情况开 DUT 侧口子：**"求解失败"发生在进程内部，夹具站在话题边界上造不出来**（见 `docs/control/mpc-node-fallback.md` §5，那里同时说明它为什么不违反 seam 1 定的「DUT 不该自带注入器」那条纪律）。它的护栏 = 默认关 + 启动 `error` 告警 + `preflight_check` 拦一道 ⟹ 注入开着**过不了装桨前自检**。 |
| ★ **轨迹原点 (trajectory origin)** | **稳定判据成立那一刻 EKF2 报的位置**（`/mavros/local_position/pose`，ROS 侧 ENU）。★ 它是**记下来的当前点**，不是悬停点、更不是动捕原点 —— 所以 EKF2 局部原点与动捕原点之间的那个**常值偏移被自动吸收**（我们是"以当前点为 0"发指令）。它与悬停点的差不超过 `hover_pos_tol_m`（判据保证）。日志里以 `traj_origin=(x, y, z)` 打出，可 grep、可与动捕对账。⚠️ 别与**起飞原点**混：起飞原点是**进入 `TAKEOFF` 那一刻**的位置，爬升与悬停点都从它推出来；两者差一个 `takeoff_altitude_m`。 |
| ★ **起飞原点 (takeoff origin)** | **进入 `TAKEOFF` 那一刻**（= 看到 `armed` 上升沿那一帧，**不是**发解锁请求那一帧）EKF2 报的位置。★ **锁一次**，之后每帧不跟着当前位姿跑 —— 否则"目标"会跟着飞机漂，"爬到 1 m"这句话就不成立。爬升目标 = 原点 + 斜坡，悬停点 = 原点正上方 `takeoff_altitude_m`。日志里以 `takeoff_origin=(x, y, z)` 打出。 |
| ★ **稳定判据 (stability criterion)** | 工单原文的量化条件：**位置误差 < `hover_pos_tol_m`(0.10 m) ∧ \|v\| < `hover_vel_tol_mps`(0.10 m/s)，持续 `hover_settle_s`(2 s)**。★ 位置误差相对**悬停点**（不是轨迹原点 —— 那是判稳**那一刻**才记的，拿它当基准是循环定义）。⚠️ **"持续"是连续的、不是累计的**：一帧不合格就重新计时。⚠️ 速度用**三维**模长，且要求 `velocity_valid`（速度话题一帧都没到过时 `v` 恰好是 0，"看起来静止"）。 |
| ★ **轨迹原点系 (trajectory origin frame)** | 以**轨迹原点**（见上）为原点、朝向与 EKF2 局部系一致（ROS 侧 ENU，z 向上）的坐标系。**`traj` 发的参考全部在这个系里**——所以轨迹 1（悬停）的目标是 `(0,0,0)`：**1 m 高度属于原点、不属于轨迹**，在轨迹里再写 1.0 会变成 2 m。把相对参考加回绝对 setpoint 是 `control` 的**唯一职责**（ticket 09）。⚠️ 与**动捕原点**不是一回事，Phase 1 完全不依赖后者（`docs/control/phase1-implementation.md` §3.3）。 |
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
| **Tera / `t_renderer`** | acados 代码生成用的模板引擎二进制。☠️ 旧话「安装缺失时报 `EOFError`」**在 v0.6.0 上不成立**（那是 v0.5.x 及以前交互式 `input()` 提问、非交互读到 EOF 的内建异常）。★ **但「必须装」这件事不变**：**真身是三种落点，其中「在、但渲染不出东西」完全静默** —— 静默两层后才炸成 `.so: cannot open shared object file`。详见 [`docs/control/acados-orin-build.md`](docs/control/acados-orin-build.md) §4 |

## 五、状态估计与接入

| 术语 | 定义 |
|---|---|
| **动捕 (motion capture, mocap)** | 外部光学定位系统。本项目用 **Nokov**，经 VRPN 协议输出刚体位姿。 |
| **刚体 (rigid body)** | 动捕软件里标定的、由若干反光球构成的可跟踪目标。每个刚体有一个原点（质心位置），该原点与飞控 IMU 的偏移即**外参**。 |
| **外参 (extrinsics)** | 两个坐标系之间的固定变换。这里特指**动捕刚体原点 → 飞控 IMU** 的平移偏移。★★ 落点在 **`mocap` 节点**（参数 `body_to_imu_x/y/z`，机体系 **FLU**、按该帧姿态旋转；z0 = `0/0/−0.080` m，2026-10-09 操作者裁定）—— **不是** PX4 参数 `EKF2_EV_POS_X/Y/Z`（那个**保持 `0/0/0`**，两边都做 = 偏移双计）。 |
| **两阶段原点** | `mocap` 节点的原点策略：**节点启动**锁一次（保证解锁前 EKF2 就有视觉数据、已收敛），**解锁上升沿**再锁一次（"起飞前静止帧"）。两阶段的**切换量 = 这段时间里飞机被移动的量** ⟹ 运行纪律是「摆好后到解锁前别动它」。切换后**永不重锁**（飞行中重锁 = 给 EKF2 一记位置跳变）。 |
| **启动原点 (boot origin)** | 两阶段里的**阶段 A** 原点 = 节点启动后前 `boot_samples` 帧的**位置均值**。此阶段姿态**不**归零 yaw（保持绝对）。 |
| **起飞前静止帧原点 (arm origin)** | 两阶段里的**阶段 B** 原点 = **解锁上升沿后** `arm_samples` 帧（≈0.3 s）的位置均值，以及同一段姿态均值里的 **yaw**。此阶段位置减该原点，姿态**只归零 yaw**（roll/pitch 保持绝对）。 |
| **VRPN** | Virtual Reality Peripheral Network，动捕数据传输协议。默认端口 3883。 |
| **EKF2** | PX4 的扩展卡尔曼滤波器，**当前唯一在用的估计器**（LPE 已废弃）。 |
| **LPE** | Local Position Estimator，PX4 的旧估计器，**已不再使用**。 |
| **外部位姿 (external vision, EV)** | 由机外系统（动捕/VIO）提供的位姿，经 MAVLink 送入飞控。 |
| **`VISION_POSITION_ESTIMATE`** | 唯一能被 **EKF2** 使用的动捕位姿消息之一（另一个是 `MAV_FRAME_LOCAL_FRD` 的 `ODOMETRY`）。 |
| **`ATT_POS_MOCAP`** | 另一条动捕消息，**只对 LPE 有效，EKF2 不使用**。网上大量教程误用它。 |
| **uXRCE-DDS** | PX4 v1.14+ 的默认 ROS2 中间件。话题前缀 `/fmu/in/...` 与 `/fmu/out/...`。☠️ **2026-10-13（票 13 收口轮）：本项目已不用它** —— z0 上这条链不通（`UXRCE_DDS_CFG = 0` + 无空闲串口可分），(C) 级下行 2026-10-12 定成走 **mavros**。本词条此后只用于读**别人**的文档 / 代码。见 [`px4-interface.md`](docs/reference/px4-interface.md) §4.3。 |
| **mavros** | MAVLink 的 ROS 封装。ROS1/ROS2 均有。本项目动捕链路使用它。 |
| **`px4_ros_com`** | PX4 旧版 ROS2 桥（FastRTPS），**v1.14 起已废弃**，现仅作为示例保留。 |

## 六、平台与硬件专有名词

| 术语 | 定义 |
|---|---|
| ★ **z0** | **本项目这台无人机的正式名称。** 构型 = OddityRC XI35 Pro 机架（152 mm 轴距）+ 微空 NxtPX4v2 飞控 + Jetson Orin NX 8GB。**全部文档中一律写 `z0`**，不再用"新机/本机/这台机"等代称。★ **状态不写死在本行**（写死的会过期）—— 认 [`docs/PROJECT.md`](docs/PROJECT.md) §1.2 的资产表（逐行带证据时点）。已知的两条：**动捕链路 2026-09-28 已在 z0 上飞成**（[`phase1/issues/09`](.scratch/phase1/issues/09-track-mission-script.md)）；**MPC 2026-10-17 已上 z0 实机**（Phase 4 `spec` 已 `done`；三趟自由飞 = 悬停 / 正方形 / 八字，见 [`phase4/issues/10`](.scratch/phase4/issues/10-ladder-closed-loop-and-free-flight.md) 与 `bag/6_free__20261017-*`）。 |
| **旧机** | 超维空间 S2-F290（PX4 **1.13.3** + 树莓派 4B）。**动捕定点飞行最早是在它上面验证通过的** —— z0 上的一切动捕经验都源自它；★ **z0 上 2026-09-28 已重做一遍**（见上面 `z0` 行）。见 `docs/PROJECT.md` 1.2。 |
| **NxtPX4v2 / Nxt-FC** | 微空科技（MicoAir）量产的 PX4 飞控，也被 HKUST 的 OmniNxt 采用。PX4 目标板名 `hkust_nxt-dual`。**z0 用的是它。** |
| **XI35 Pro** | 怪象（OddityRC）的 3.5 英寸涵道机架，轴距 152 mm。**z0 的机架。** |
| **`z0.params`** | **z0 的飞控参数文件**（`assets/z0.params`；★ 行数/条数随现场重导出变化，**要数就现算**：`wc -l < docs/assets/z0.params` 与 `grep -cE '^[A-Z0-9_]+[[:space:]]' docs/assets/z0.params`）。内容来自上游 `Huan-qun/Sim2real-UAV-Params-` 的 `S2R/assets/param/uav1.params`（那台机器叫 "Vehicle 1"），**2026-10-10 又用 QGC 从 z0 飞控重导出**了一次（见文件头）。★ **2026-09-21 那一版**已与 z0 飞控真身逐条对账（914/915 条名字全对上，见 [`.scratch/phase1/issues/15`](.scratch/phase1/issues/15-fcu-param-reconciliation.md)）⟹ **它就是飞机上跑的那份配置**；2026-10-10 的重导出**没有重做**这项逐条对账。⚠️ 但"对账一致"恰恰意味着文档里那些"X 是默认值/可疑"**真的在飞机上** —— **不等于它们是实测值**。见 `docs/reference/params-analysis.md`。 |
| **动捕定点飞行** | 用动捕位姿闭环、让飞机自主保持位置悬停的飞行状态。**旧机（S2-F290）上最早实现；z0 上 2026-09-28 已飞成**（[`.scratch/phase1/issues/09`](.scratch/phase1/issues/09-track-mission-script.md)）。 |

## 七、测试与台架

| 术语 | 定义 |
|---|---|
| **接缝 (seam)** | 测试切入的边界。**接缝 1** = **ROS 话题/服务边界**（喂入向话题、断言出向话题），节点级测试走这条；**接缝 2** = **纯函数**（不碰 ROS，如坐标变换、时序逻辑）。★ 规矩：**能在接缝 2 测的就别拉到接缝 1**，能上接缝 1 的就别去动内部类。 |
| **台架夹具 (bench fixture)** | `src/bench/` 提供的测试基础设施（**不是飞行代码**）：一次"被控对象在环"的场景 —— 假 mavros + 脚本化动捕 + **假 traj** + 故障注入时序 + **DUT 真子进程**（另有一件"假飞机"，见下两条）。被测节点当**进程**启动，所以"进程被杀 ⟹ 流停止"这类判据才可测。 |
| ★ **假 mavros (fake mavros)** | 夹具里假扮 **mavros 的行为**（话题名、频率、**QoS**、`/mavros/state` 1 Hz、`mode_sent` 语义、**`/mavros/cmd/arming` 的应答与"应答肯定但 `armed` 不上升沿"**、**`/mavros/estimator_status` 1 Hz 与自报的 `pos_horiz_abs`**），**不假扮 PX4 的行为**（`COM_OF_LOSS_T` 退出、EKF2 融合、模式需求检查）。⚠️ QoS 必须逐条对齐真 mavros，**不兼容时不报错、只是永远收不到**。 |
| ★ **假 PX4 (fake PX4)** | 夹具里假扮 **飞控的 `/fmu/*` 话题面**（`bench/fake_px4.py` 的 `FakePx4`，ticket 06）：`/fmu/out/vehicle_status` **2 Hz 定频 + 变化即刻发**、`arming_state`/`nav_state`/`pre_flight_checks_pass` 的取值、与真 writer/reader **逐字一致**的 QoS，注入面是 `mute` / `set_armed` / `set_nav_state` / `set_pre_flight_checks_pass`（都能排进 `Timeline` 按指定时刻触发）。**只假话题面、不假飞控的行为** —— `COM_OF_LOSS_T` 退出、EKF2 融合、模式需求检查、混控都不在里面，那几条要看只能上 SITL / 真机（与上面"假 mavros"的分工见 `src/bench/README.md` 的 ★ 调和那条）。 |
| **脚本化动捕 (scripted mocap)** | 夹具里按秒表喂位姿的发布者，两种接法：喂 `/vrpn/<刚体>/pose`（上游，`mocap` 节点在回路里）或直接喂 `/mocap/pose_enu`（下游，旁路 `mocap`）。**动捕主机、VRPN、路由器都不该出现在节点级测试里。** |
| ★ **假 traj (fake traj)** | 夹具里假扮 **`traj` 节点的接口与时钟**（`FakeTraj`，ticket 09）：收 `/traj/select` + `/traj/start`，发 50 Hz 的 `/traj/reference` 与 `/traj/progress`。★ 它复刻 `traj_node.cpp` 的两个**真行为**（没 `select`+`start` 时一帧都不发；跑完之后照旧 50 Hz 发、完成度**停在 1.0**），因为那两条正是 `control` 侧判据的分界线。**不假**轨迹的几何与运动学（那归 `traj` 自己的测试）。 |
| ★ **假飞机 (plant)** | ★ **不是夹具、是场景逻辑**（`bench/plant.py` 的 `Plant`，ticket 09 从 `control` 的测试里提升进来）：一架"速率受限地逼近 setpoint"的假飞机，速度 = 实际位移 / dt。同一个 DUT 在不同场景里配的是**不同的飞机** ⟹ 它与 `Timeline` 同类，而不是"外部世界"的一部分。 |
| **DUT** | Device Under Test，被测节点。台架里它以**子进程**运行，绝不是 import 它的类。 |
