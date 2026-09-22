# Phase 1 实施清单 —— offboard 位置定点（`(A)` 级）

> **本文件是 Phase 1 的施工图**，与 [`phase2-implementation.md`](phase2-implementation.md) 平行。
> **Phase 1 = offboard (A) 级位置控制，全 mavros2，PX4 跑全部内环 PID。**
>
> 上级文件：[`../PROJECT.md`](../PROJECT.md) §4 与里程碑表。
> 事实来源：[`../reference/px4-interface.md`](../reference/px4-interface.md)、[`../reference/mocap-setup.md`](../reference/mocap-setup.md)、[`../reference/params-analysis.md`](../reference/params-analysis.md)。
> 取舍理由（"为什么用这个包"）：[`../reference/code-inventory.md`](../reference/code-inventory.md)。
>
> ⚠️ **本文件中的结论，凡标注「第一手」的，都是从源码/仓库直接读出来的**，不是转述。相关第一手证据总表见 [`../reference/open-source-survey.md`](../reference/open-source-survey.md)。

---

## 0. 已定决策

| # | 决策 | 结论 | 依据 |
|---|---|---|---|
| H1 | 控制层级 | **`(A)` 位置 + yaw** | 用户"先 offboard 控制飞控内置 PID 定点飞行" |
| H2 | 中间件 | **全 mavros2，不碰 uXRCE-DDS** | 用户 Q7；uXRCE-DDS 推迟到 Phase 2 |
| H3 | 动捕数据源 | **`vrpn_listener.zip`（`Huan-qun/Sim2real-UAV-Params-`）** | 用户 Q9：「这是经过实机验证的」，**vendored 原样复用** |
| H4 | 动捕原点 | **未起飞时静止帧为原点**；姿态**仅归零 yaw** | 用户 Q16 / Q21(b) |
| H5 | 参考轨迹 | **脚本启动后输入数字选择**：①起飞 1 m 悬停 ②边长 1 m 正方形 ③半径 0.5 m 八字 | 用户 Q15 |
| H6 | SITL | **跳过**（Phase 1 无 MPC，实机试错代价可接受） | 用户 Q8(b)；Phase 2 **不跳** |
| H7 | 固件 | **留在 PX4 1.14.x，不刷固件** | 用户 Q12 |

**★ 指导思想：Phase 1 只引入一个新变量（动捕 + offboard 链路），把 MPC / acados / uXRCE-DDS 全部推迟。**
Phase 2 的所有新东西**追加**在这套骨架上，**不推翻**它 —— 见 [`phase2-implementation.md`](phase2-implementation.md) §3.4。

---

## 1. 包与文件

```
src/
├── vrpn_listener/                     ✅ vendored（原样）
│   ├── src/vrpn_listener.cpp
│   ├── include/vrpn_listener/vrpn_listener.hpp
│   ├── cmake/{FindVRPN,Findquatlib}.cmake
│   ├── config/params.yaml             server=10.1.1.198, port=3883
│   ├── launch/sync_entity_state.launch
│   └── VENDORED.md          ← ★ 新增：写明来源 zip、"勿替换"、stamp 不可改
│
├── mocap/                             ✅ 已实现（2026-09-21，ticket 04）
│   ├── src/mocap_rel_pose_node.cpp    ← 相对位姿节点（ROS 边界：话题进出）
│   ├── include/mocap/mocap_rel_pose.hpp  ← ★ 三个纯函数，**零 ROS 依赖**（seam 2）
│   ├── test/test_mocap_rel_pose.cpp   ← ★ 纯函数单测（seam 2，20 例）
│   ├── config/mocap.yaml              ← 刚体名 + 话题名都在这（多机改参数复用）
│   ├── launch/mocap.launch.py
│   └── README.md                      ← 参数表 + 运行纪律 + 两阶段原点的理由
│
├── traj/                              ✅ 已实现（2026-09-22，ticket 07）
│   ├── src/traj_node.cpp              ← 只做接线：参数、话题、定时器、RViz 折线
│   ├── include/traj/trajectories.hpp  ← ★ 三条轨迹的**纯函数**、**零 ROS 依赖**（seam 2；含弧长参数化）
│   ├── test/test_trajectories.cpp     ← ★ 纯函数单测（seam 2，20 例，独立 Simpson 法交叉核对周长）
│   ├── test/test_traj_node.py         ← ★ 节点级（seam 1，11 例）：只断言话题边界
│   ├── config/traj.yaml               ← 三条轨迹的参数 + 速率 + RViz
│   ├── launch/traj.launch.py
│   └── README.md                      ← 参数表 + 怎么用 1/2/3 选轨迹 + 它**不发** setpoint
│
├── control/                           ✅ 已实现（2026-09-21，ticket 05）—— **只到 STANDBY**
│                                      （Phase 2 的 MPC（模型预测控制）节点也放这里）
│   ├── src/control_node.cpp           ← 只做接线：参数、定时器、日志
│   ├── src/px4_interface.cpp          ← mavros 服务/话题封装 + 每条输入的新鲜度
│   ├── include/control/fsm.hpp        ← ★ 纯逻辑、零 ROS（seam 2）：两个正交维度 + 转移表
│   ├── include/control/px4_interface.hpp
│   ├── config/control.yaml
│   ├── launch/control.launch.py
│   ├── test/test_fsm.cpp              ← ★ 纯函数单测（seam 2，零 ROS）：守卫清单 + 前置流计时
│   ├── test/test_control_standby.py   ← ★ 节点级（seam 1）：DUT 是**子进程**，只断言话题边界
│   └── README.md                      ← ★ 先读它：参数表 + 两个维度的语义 + 与 06/08 的边界
│
├── bench/                             ✅ 已实现（2026-09-21，ticket 05）—— **测试基础设施，不是飞行代码**
│   ├── bench/fake_mavros.py           ← ★ 假 mavros（QoS 逐条对着真 mavros 源码抄）
│   ├── bench/vrpn_source.py           ← 脚本化动捕（喂 /vrpn/* 或直接喂 /mocap/pose_enu）
│   ├── bench/timeline.py              ← ★ 纯逻辑、零 ROS：故障注入时序 + 常用配方
│   ├── bench/scenario.py              ← Scenario：拼装 + DUT 子进程 + 秒表 + 证据输出
│   ├── bench/pytest_support.py        ← ★ ROS 域隔离（**拒绝 domain 0**）
│   ├── config/bench.yaml
│   ├── test/                          ← ★ 夹具自检（验它"假得像不像"）+ 时序纯逻辑单测
│   └── README.md                      ← ★ 夹具说明书：它假的是什么、能注入什么、怎么写场景
│
├── bringup/                           🆕 新写
│   ├── launch/z0.launch.py            ← 一键起 mocap+traj+control
│   ├── launch/mavros.launch.py        ← 或直接用 mavros 自带
│   ├── scripts/fly.py                 ← ★ 交互式脚本：输入 1/2/3 选轨迹
│   └── config/mavros_z0.yaml          ← mavros 的 fcu_url 等
│
└── （可选）tools/
    └── scripts/thrust_calibrate.py    🔶 从 XI35 重写（ROS2 + (A) 级话题）
```

**不需要自定义 msg 包** —— 见 §2，用 `std_msgs` 就够。**这是有意的简化**：Phase 1 不值得为消息定义增加构建复杂度。

## 2. 话题 / 服务接口

### 输入（`control` 订阅）

| 话题 | 类型 | 用途 | 备注 |
|---|---|---|---|
| `/mavros/state` | `mavros_msgs/State` | `connected` / `armed` / `mode` | ★ FSM 的主驱动 |
| `/mavros/local_position/pose` | `geometry_msgs/PoseStamped` | EKF2 融合位置 | ★ **轨迹原点与稳定判据都用它**，见 §3.3 |
| `/mavros/local_position/velocity_local` | `geometry_msgs/TwistStamped` | EKF2 融合速度 | ★ 稳定判据的 `|v|` 来源 |
| `/mavros/rc/in` | `mavros_msgs/RCIn` | **开始按键** | ★ 通道号是参数，实飞时确认。⚠️ ticket 05 只订阅、不做判据 |
| `/mavros/battery` | `sensor_msgs/BatteryState` | 低电压保护 | ⚠️ 见 §8.2。⚠️ ticket 05 只订阅、不做判据 |
| `/mocap/pose_enu` | `geometry_msgs/PoseStamped` | 相对位姿（**ENU，原生**） | Phase 1 用于记录（Q23）**以及 ticket 05 的守卫**（它的新鲜度决定发不发 setpoint）；**Phase 2 的 MPC 拿它当状态源** |
| `/traj/reference` | `mavros_msgs/PositionTarget` | ★ 参考轨迹（**相对轨迹原点**，不是绝对位置） | ticket 09 在 TRACK 里**加上轨迹原点**再发给 PX4。⚠️ **绝不能把这条直接 remap 到 `/mavros/setpoint_raw/local`** ：轨迹原点是 1 m 高度上的悬停点，直接转发 = 先掉到地面附近。原点的记录见 §3.3 |
| `/traj/progress` | `std_msgs/Float32` | 轨迹完成度 0–1 | ★ 驱动 §4.2 的 `TRACK → LAND`（`≥ 1.0`）。🔶 **ticket 09 未实现**：按设计 `/control/traj_progress` 就是它的转发（这样 `fly.py` 那侧不用改） |

### 输出（`control` 发布 / 调用）

| 话题 / 服务 | 类型 | 用途 |
|---|---|---|
| `/mavros/setpoint_raw/local` | `mavros_msgs/PositionTarget` | ★ **唯一的控制输出** |
| `/mavros/set_mode` | `mavros_msgs/SetMode` | 切 `OFFBOARD`（服务） |
| `/mavros/cmd/arming` | `mavros_msgs/CommandBool` | 解锁/上锁（服务） |
| `/control/state` | `std_msgs/String` | 当前状态名，给 `fly.py` 显示 |
| `/control/traj_progress` | `std_msgs/Float32` | 轨迹完成度 0–1 |

> ★ **两个维度显式分开（ticket 05 已实现）**：`/control/state` 发的是
> **`"<控制权>/<任务态>"`**（如 `MANUAL/STANDBY`），**不是**一个扁平状态名。
> 控制权 `MANUAL`/`AUTO` 的定义、任务态 `INIT`/`STANDBY`/`TAKEOFF`/`HOVER`/`TRACK`/`LAND`
> 见 [`CONTEXT.md`](../../CONTEXT.md) §三。它用 **`transient_local`** 发布
> （晚订阅者立刻拿到当前值，`fly.py`、测试、`ros2 topic echo` 都受益），且每 `diag_period_s` 心跳重发。
> 实现与边界见 [`src/control/README.md`](../../src/control/README.md)。

> ★★ **QoS：入向话题必须逐条对齐真 mavros，错了不报错、只是永远收不到**（ticket 05 实测 + 源码核实）：
> `/mavros/local_position/*` 与 `/mavros/battery` 是 **BEST_EFFORT**（`SensorDataQoS()`），
> `/mavros/state` 是 **可靠 + `transient_local`**（`mavros::StateQoS()`），`/mavros/rc/in` 是可靠。
> 用默认（可靠）订阅 `local_position` 会**一帧都收不到**，日志里只有一句
> `offering incompatible QoS`，节点不报错也不退出。
> 出向的 `/mavros/setpoint_raw/local` 反过来：mavros 用 `SensorDataQoS()` **订阅**它，
> 所以发布方用可靠（默认）**能**匹配，改成 best_effort 也没问题 —— 但**别改**，照抄现状最省事。
> 依据与实测见 [`src/bench/README.md`](../../src/bench/README.md)（夹具逐条复刻了这些 QoS，
> 并有专门的回归测试盯着"不可靠订阅者收不到 best_effort 数据"这件事）。

### `traj` 节点（轨迹发生器）

✅ **已实现（2026-09-22，[ticket 07](../../.scratch/phase1/issues/07-traj-reference-trajectories.md)）= [`src/traj/`](../../src/traj/)。**

| 方向 | 话题 | 类型 | 说明 |
|---|---|---|---|
| 订阅 | `/traj/select` | `std_msgs/UInt8` | **选轨迹：1/2/3**（`0` = 清空）。由 `fly.py` 或手打 `ros2 topic pub` 发 |
| 订阅 | `/traj/start` | `std_msgs/Empty` | ★ **开始跑**。与 `/traj/select` **分开**（见下） |
| 发布 | `/traj/reference` | `mavros_msgs/PositionTarget` | ★ 参考轨迹，**50 Hz**，**相对轨迹原点**；`type_mask = 2496`（位置+速度+yaw）、`frame_id = "traj_origin"` |
| 发布 | `/traj/progress` | `std_msgs/Float32` | 完成度 0–1 |
| 发布 | `/traj/marker` | `visualization_msgs/Marker` | RViz 折线（形状与尺寸的验收手段），无控制作用 |

> ★ **选与开始为什么是两条话题**：`select` 是"我打算飞哪条"（离线核对形状用，`/traj/marker` 一选就出，
> 不必等起飞）；`start` 是"现在开始计时"。运行中重选 ⟹ **参考立刻停流，且必须再发一次 `start`**
> （新的时间轴从 0 起）——否则 `control` 还在 TRACK 时会按**旧轨迹的惯性**把飞机拽到新轨迹上。
> 先例：Fast-Drone-XI35 的 `px4ctrl` 在 AUTO_HOVER 里发 `/traj_start_trigger`（`PX4CtrlFSM.cpp:202`、`:598-603`）。
>
> 🔶 **`/traj/start` 由谁在实飞中发，是 ticket 09 要定的事**（`control` 在进 `TRACK` 那一刻发是最自然的接线，
> 也是 XI35 的做法）；现在只有 `fly.py` / 手打能用。
>
> ⚠️ **`traj` 不发 setpoint、不碰 mavros、不判稳。** 它只回答"第 t 秒该在哪、以多快的速度经过"。

### `mocap` 节点

| 方向 | 话题 | 类型 | 说明 |
|---|---|---|---|
| 订阅 | `/vrpn/<刚体>/pose` | `geometry_msgs/PoseStamped` | 来自 `vrpn_listener` |
| 订阅 | `/mavros/state` | `mavros_msgs/State` | ★ 检测**解锁上升沿** |
| 发布 | `/mavros/vision_pose/pose` | `geometry_msgs/PoseStamped` | → PX4 EKF2（**ENU/FLU，mavros 负责转换** —— 已读 mavros 源码确认，见 §3.5） |
| 发布 | `/mocap/pose_enu` | `geometry_msgs/PoseStamped` | → 仅**记录**（Q23）+ Phase 2 MPC 状态源。**动捕原生 ENU，不做任何转换** |

> ★ **`mocap` 节点不做任何坐标系转换（Q14=a）。** 它只做「减原点 + yaw 归零」，
> 然后把**动捕原生 ENU/FLU** 同时喂给 mavros 的 `vision_pose`。
>
> ⚠️ **本文旧版曾让本节点额外发布一条 `/mocap/pose_ned`（NED/FRD）给"本项目控制器"用，
> 并警告"两路约定不一样、搞反了会飘"。该话题已按 Q14=a 删除** —— 理由：
> ① Phase 1 的控制器工作在 **EKF2 的局部系**里（§3.3；★ 该系在 ROS 侧**就是 ENU**，见 §3.1 的更正），
> 位置直接读 `/mavros/local_position/*`，**不需要** mocap 再发一份；② Phase 2 的 MPC（模型预测控制）用 ENU（§3.2 / G3）。
> 即**没有任何消费者**。删掉它同时消灭了"转两次 / 一次都没转"这一整类 bug。

> ✅ **已实现（2026-09-21，[ticket 04](../../.scratch/phase1/issues/04-mocap-relative-pose.md)）= [`src/mocap/`](../../src/mocap/)。**
> 实现细节（参数表、两阶段原点的理由、运行纪律）见 [`src/mocap/README.md`](../../src/mocap/README.md)。三条与本表有关的补充：
> - **刚体名走参数文件**（`config/mocap.yaml` 的 `body_name`）——它是**变量**不是常量（一天内实测到 `Z0`/`z0` 两个值），
>   且**多台无人机换一份 yaml 即可复用**，代码不动。抄错名字时节点会在 5 s 后把 graph 里**有发布者**的
>   `/vrpn/*/pose` 连"只差大小写"一起列出来（只订阅、无人发布的不算候选 —— 否则会出现"名字没错"的假象）。
> - **动捕数据看门狗**（spec 用户故事 10）：数据中断超 `no_data_warn_s`（默认 5 s）就 ERROR 一句
>   "EKF2 的视觉输入已失效，不要再起飞"，恢复时 WARN 一句；每次中断只报一次，不刷屏。
>   ⚠️ 这是"数据到没到"，与 §4.3-3 的 `msg_timeout`（"**消费**是否超时"，归 `control`）不是同一个东西。
> - **`<刚体>_Marker1..5` 这些 per-marker 话题不屏蔽**（本节点按名字精确订阅，多余话题无订阅开销；屏蔽要改 vendored 的
>   `vrpn_listener`，那是 ticket 03 明令不许的）。真嫌带宽就在 Nokov 侧关掉 marker 输出。

## 3. 坐标系与符号（★ 写死，不靠记忆）

### 3.1 三套坐标系

| 坐标系 | 约定 | 出现在哪 |
|---|---|---|
| **动捕 map（ENU）** | x=假定东 / y=北 / z=**上** | `/vrpn/<刚体>/pose` |
| **PX4 本地（NED）** | x=北 / y=东 / z=**下** | MAVLink 线上的 NED。⚠️ **ROS 侧的话题不是这个口径** —— 见下方更正 |
| **PX4 机体（FRD）** | x=前 / y=右 / z=下 | 姿态、角速率（Phase 2 才用） |

> ☠️☠️ **更正（2026-09-21，ticket 05 第一手核实）：mavros 的 ROS 侧**入向/出向**都是 ENU，
> 「`/mavros/local_position/*` 与 `/mavros/setpoint_raw/local` 是 NED」这句是错的。**
> 上表第 2 行必须读成"**MAVLink 线上的** NED"，而不是"ROS 话题的**。依据（都读过源码）：
>
> | 话题 | mavros 干什么 | 源码 |
> |---|---|---|
> | `/mavros/local_position/pose`（出向） | PX4 的 NED → **ENU** 后发布 | `local_position.cpp:157` `transform_frame_ned_enu(...)`（速度在 `:162`，同） |
> | `/mavros/setpoint_raw/local`（入向） | 收到的 **ENU** → NED 再发给 PX4 | `setpoint_raw.cpp:231` `transform_frame_enu_ned(position)` |
>
> **两条直接后果，都会咬人：**
> 1. **`position.z` 向上为正**（ENU）。★ 若照"NED"去写，起飞目标会写成 `z = -1.0` ⟹
>    **飞机往地里钻**。§5.2 的"高度 1.0 m"要的就是 `z = +1.0`。
> 2. **`yaw` 也是 ENU 口径**，实测换算 **`yaw_ned = π/2 − yaw_enu`**
>    （`setpoint_raw.cpp:235`；按 `frame_tf.hpp:180` + `ftf_frame_conversions.cpp:33/40` 复算）
>    ⟹ **`yaw = 0` 的意思是"机头朝 ENU 的 +x（假定东）"，不是"保持当前机头"**。
>    ticket 08 定起飞朝向时必须显式处理，**别把默认 0 当"不改变"用**。
> 3. ★ **好消息**：正因为两边都是 ENU，「把 `/mavros/local_position/pose` 原样转给 setpoint」
>    **恰好是正确的**（一进一出抵消，PX4 收到的是它自己报的值）——
>    ticket 05 的实现就是这么做且已验证。**不要**因为上表的措辞去加一次 ENU→NED 转换，
>    那会把飞机送错位置。
>
> 2026-09-21 已落：`src/control/README.md`、`include/control/px4_interface.hpp`、`config/control.yaml` 均已按本条改正。

### 3.2 ★ 位置转换（ENU → NED）

```
x_ned =  y_enu
y_ned =  x_enu
z_ned = -z_enu
```

⚠️ **这是交换 x/y 并翻转 z，不是简单取反。** 只写 `-` 是最常见的错误。

**姿态**：ENU→NED 是绕 z 轴 −90° 再镜像，**四元数不能照抄上面的公式**。Phase 1 的做法见 §3.4。

### 3.3 ★★ 两个"原点"是不同的，这一点必须分清

| 原点 | 在哪定义 | Phase 1 是否依赖 |
|---|---|---|
| **动捕原点** | `mocap` 节点在**解锁上升沿**锁定 | ❌ **Phase 1 不依赖** |
| **轨迹原点** | `control` 在 **HOVER 判定稳定那一刻**记录的 `/mavros/local_position/pose` | ✅ **Phase 1 完全依赖** |

**为什么这样设计**：`/mavros/setpoint_raw/local` 的坐标是 **EKF2 的局部系**（ROS 侧为 ENU，见 §3.1 的更正），而 EKF2 的局部原点在**它自己初始化时**确定，未必等于我们锁定的动捕原点。两者之间可能有常值偏移。

**→ 所以轨迹原点取"HOVER 稳定时 EKF2 报的位置"**，这样：
- **偏移被自动吸收**——我们是在同一个坐标系里"以当前点为 0"发指令
- **mocap 节点的原点怎么定都不影响 Phase 1**（它只影响喂给 EKF2 的 vision_pose）
- Phase 2 的 MPC 改用 `/mocap/pose_enu`，那时才需要动捕原点

> 📌 **这个设计让 Phase 1 与 Phase 2 的原点问题彻底解耦，是降低 Phase 1 风险的关键一步。**

> ★ **谁负责加这个原点：`control`，不是 `traj`（ticket 07 实测口径）。**
> `traj` 节点只发**相对轨迹原点**的参考 —— 它根本不知道飞机在哪，也就没有"轨迹原点"这个概念。
> 好处是 `traj` 可以在**没有飞控、没有动捕、没起飞**的情况下把三条几何跑一遍并核对
> （`traj` 的 seam 1 测试就是这么做的）；而"轨迹原点 + 相对参考 = 绝对 setpoint"这件事
> 只有 `control` 这一处发生 ⟹ **"重复加原点/一次都没加"这一整类 bug 只有一个可能出错的地方**。
> ⟹ ticket 09 接线时：`reference_abs = traj_origin + traj_relative`（高度也一并来自原点，见 §5.2 的澄清）。

### 3.4 ★ 姿态：仅归零 yaw

**本项目已定：位置做平移，姿态只归零 yaw，roll/pitch 保持绝对。**

```
解锁上升沿采集 ~0.3 s（约 30–120 帧）平均值：
    p0  = 位置均值
    ψ0  = yaw(q0)                                  ← 只取 yaw
    q_off = quat_from_euler(0, 0, -ψ0)             ← 纯 yaw 旋转

之后每帧：
    p_rel = p - p0                                 （★ 到此为止，**不做**坐标转换，见 §3.5）
    q_rel = q_off ⊗ q                              ← ★ 左乘，只绕世界 z
```

**为什么不能整个姿态做相对**：解锁时飞机停在地面，roll/pitch **通常不是精确的 0**（地面不平、起落架软）。整个姿态做相对之后，"水平"的定义变成了"解锁时那个姿态"——若解锁时有 5° 横滚，PX4 认为的竖直方向就偏 5°，**飞机持续往一个方向飘，而 EKF2 认为误差是零，控制器看不出来**。

#### ⚠️ 一个必须解决的前提：解锁前不能"没有 vision_pose"

**"原点在解锁上升沿锁定"这个决策，如果实现成"解锁前什么都不发"，会踩一个坑：**

EKF2 在解锁前就**没有任何视觉数据**——而 PX4 解锁与 OFFBOARD 进入都要求**已经收敛的位置/姿态估计**。等到解锁那一刻才第一次喂 vision_pose，EKF2 能不能干净地初始化并收敛，**是一个未验证的问题**（旧的已验证链路是**从节点启动就一直发**的）。

**★ 解法（既保留"解锁边沿锁定"的语义，又不让 EKF2 断粮）**：

```
阶段 A（节点启动 → 解锁前）：
    锁定"启动原点" p_boot  ← 启动后静止若干帧取均值
    持续发布 vision_pose（位置 = p − p_boot，姿态 = 绝对 q，不做 yaw 归零）
    → EKF2 在解锁前就已收敛，与旧链路一致

阶段 B（解锁上升沿）：
    把原点切换到 p_arm / ψ0（按上面的公式）
    → 得到一个"原点切换"，大小 = 飞机从启动到解锁期间被移动/旋转的量
```

**→ 所以纪律是：从节点启动到解锁之间，不要把飞机搬来搬去、不要转它。** 只要这一点做到，阶段 A→B 的切换量 ≈ 0，**没有不连续，也没有 EKF2 断粮**。

> ⚠️ **这比老的"机头朝假定东"仪式更宽松，但不是"什么纪律都不需要"。** 老仪式是"必须朝特定方向摆放"；新纪律是"**摆好后到解锁前别动它**"。
>
> 📌 **这也修正了本文档早先一处过于乐观的说法**：我曾写"选仅归零 yaw 之后，机头朝向的仪式就不再必要"。**准确的说法是**——**朝向不再重要，但"启动到解锁之间保持静止"仍然重要**。（朝向为何变得不重要：EKF2 融合视觉 yaw 后会把世界系对齐到动捕坐标系，所以 PX4 的"北"就是动捕的 +x；机头朝哪都行。）

> ✅ **这个方案可以直接在台架上验证**（§7 第 3 步）：不解锁，先看 QGC 的 `LOCAL_POSITION_NED` 是否**在解锁前就已经稳定**。若稳定 → EKF2 吃到了 vision_pose，阶段 A 生效。

### 3.5 ✅ `/mavros/vision_pose/pose` 的坐标系：**mavros 会转，已读源码确认**

**`mavros` 发布 `/mavros/vision_pose/pose` 的 `VisionPoseEstimatePlugin::send_vision_estimate()` 原文**（`mavros_extras/src/plugins/vision_pose_estimate.cpp`）：

```cpp
auto position = ftf::transform_frame_enu_ned(Eigen::Vector3d(tr.translation()));
auto rpy = ftf::quaternion_to_rpy(
  ftf::transform_orientation_enu_ned(
    ftf::transform_orientation_baselink_aircraft(Eigen::Quaterniond(tr.rotation()))));
```

**→ mavros 在边界上做了三次转换**：位置 ENU→NED、姿态 **FLU→FRD**、姿态 ENU→NED。

**所以 `mocap` 节点应该发布"动捕原生值"**（mocap 是 ENU，刚体是"x=机头 / y=左 / z=上"= **FLU**，与 `mavros` 的期望正好对上）：

```
发布到 /mavros/vision_pose/pose ：
    position    = p_rel        （动捕 ENU 值 − p0，不做 ENU→NED）
    orientation = q_rel        （绝对 q 左乘纯 yaw 归零，不做 ENU→NED、不做 FLU→FRD）
```

> ✅ **这证实了 [`mocap-setup.md`](../reference/mocap-setup.md) 7.3 的旧节点"直接透传"的做法是对的**——它当年能跑通不是巧合。
> 📌 **旧版这里还有一条 `/mocap/pose_ned`（"给本项目控制器用，我们自己做 ENU→NED"）—— 已按 Q14=a 删除。**
> 因为两条链**都用不上它**：喂 EKF2 的这条（本节）由 mavros 负责转换；
> Phase 2 的 MPC 直接用 ENU（§3.2 / G3）。**→ `mocap` 节点是全栈唯一一个不做坐标转换、只做
> 「减原点 + yaw 归零」的节点，这是本项目故意保持的简洁性。**

**⚠️ 一个 `mavros` 的隐藏行为**：`send_vision_estimate()` 开头有

```cpp
if (last_transform_stamp == stamp) { /* dropped */ return; }
```

**时间戳不前进的消息会被静默丢弃。** `vrpn_listener` 的 stamp 每帧都变，所以没问题——**但如果将来为了"修时间戳"而给它一个固定值，vision_pose 会整条失效**。这是"不要动 `stamp`"的第二个理由。

### 3.6 ⚠️ 推力数值范围（Phase 2 用，现在先写下来）

`VehicleRatesSetpoint.thrust` 与 `AttitudeTarget.thrust` 都是 **`[-1, 1]` 归一化**，**不是 `0–1`**、**不是牛顿**、**不是"相对悬停"**。

**数值约定错了，MPC 算得再对，飞机也不会飞。** 见 [`px4-interface.md`](../reference/px4-interface.md) 3.3。

### 3.7 ☠️ 更正：Phase 1 的 setpoint「时间戳单位」**不是微秒**（ticket 05 第一手核实）

**旧说法**（`.scratch/phase1/spec.md` 的「消息时间戳」段）："所有发往 PX4 的消息使用**微秒**时间戳"。

**这条只对 Phase 2 的 uXRCE-DDS 路径成立**（`px4_msgs` 的 `*Setpoint.timestamp` 确实是微秒 `uint64`）。
Phase 1 走的是 mavros，**不存在"微秒"这一层**：

1. **mavros 出向**：`PositionTarget.header.stamp`（ROS2 `builtin_interfaces/Time`）被换算成 MAVLink 的
   `time_boot_ms`，即 **毫秒**、且会**截断成 32 位** —— 见 `mavros/include/mavros/plugin.hpp:264-267`
   与 `setpoint_raw.cpp` 的 `local_cb`。
2. **PX4 侧根本不读这个字段**（★ 2026-09-21 在 `~/refs/px4-v1.15.0` 上逐行读过，与 z0 固件同代）：
   `src/modules/mavlink/mavlink_receiver.cpp:960`
   `MavlinkReceiver::handle_message_set_position_target_local_ned()` 只读 `type_mask` /
   `coordinate_frame` / 各分量，**`target_local_ned.time_boot_ms` 一次都没出现**；
   `trajectory_setpoint.timestamp` 是在发布处用 **`hrt_absolute_time()`** 自己打的
   （同上文件 **1067 行**：`setpoint.timestamp = hrt_absolute_time();`）。

**⟹ Phase 1 的实现口径**（与 mavros 自带 setpoint 插件一致）：`header.stamp = now()`，**不要**去凑"微秒"。

> ★ 同一次阅读还确认了两件与本节点行为直接相关的事：
> - 该 handler **只在 `nav_state == NAVIGATION_STATE_OFFBOARD` 时才把 setpoint 发进 ORB**
>   （`mavlink_receiver.cpp:1066` 的注释原文 "only publish setpoint once in OFFBOARD"）
>   ⟹ **切模式之前流的那些 setpoint 是被丢弃的**，
>   起作用的不是它们本身，而是它们每条都触发的 **`offboard_control_mode` 发布** ——
>   那才是"offboard 信号存在"的判据（`Commander::offboardControlCheck()`）。
>   这就是 §4.3-1「前置流」能生效的机制。
> - `type_mask = 2552` 不会撞上 `POSITION_TARGET_TYPEMASK_FORCE_SET` 那条错误分支
>   （该分支的条件是 `ocm.acceleration && (type_mask & FORCE_SET)`，而 2552 让加速度全为 NaN
>   ⟹ `ocm.acceleration == false`）。见 §8.6。

## 4. FSM 状态与转移

### 4.1 状态设计（把"谁在控制"与"在干什么"显式分开）

| 状态 | 含义 | 输出 |
|---|---|---|
| `INIT` | 上电：等 mavros 连接、动捕有数据、EKF2 收敛 | **不发 setpoint** |
| `STANDBY` | 未解锁；**已开始以 50 Hz 持续发布 setpoint**（位置=当前，yaw=0） | 流 setpoint，不解锁 |
| `TAKEOFF` | 已解锁且在 OFFBOARD，爬升到 1.0 m | 目标 z 线性上升，≤0.5 m/s |
| `HOVER` | 定点保持；同时**判定是否稳定** | 目标 = 轨迹原点 |
| `WAIT_START` | 已稳定，**等遥控器按键** | 目标 = 轨迹原点 |
| `TRACK` | 执行选定轨迹 | 目标 = 轨迹发生器输出 |
| `LAND` | 自动降落 | 交给 PX4 的 `AUTO.LAND` |
| `RC_OVERRIDE` | 遥控器打杆接管 | **停止发 setpoint**，退出 OFFBOARD |

> ★ **与已实现代码的对应关系（2026-09-22 更新，含 ticket 06）** —— 上表是**目标形态**，代码是它的子集：
> - **已实现**：`INIT`、`STANDBY`（ticket 05）、`LAND`、`RC_OVERRIDE`（ticket 06）。
>   见 [`src/control/include/control/fsm.hpp`](../../src/control/include/control/fsm.hpp) 的 `Phase`。
> - **`TAKEOFF`/`HOVER`/`TRACK` 仍只在枚举里占位**：任何输入组合都到不了，归 ticket 08/07。
> - **`RC_OVERRIDE` 与「控制权」是**两个维度**，但夺权时**同时**置位：代码里"谁在控制"是独立的
>   `Authority{MANUAL, AUTO}`，打杆/模式被切走 ⟹ `Authority = MANUAL` **且** `Phase = RC_OVERRIDE`
>   （ticket 06 的用户决策 ④）。所以 `/control/state` 在夺权时是 **`MANUAL/RC_OVERRIDE`**。
> - **`WAIT_START` 代码里还没有**：它是"已稳定、等按键"的中间态，归 ticket 08 引入
>   （`HOVER` 之后的判据也要那批工作一并定）。**不要**在 ticket 08 之前往枚举里加。
> - ★ 早期图纸里出现的 `IDLE`（ticket 05 的状态机示意图）**与本文的 `INIT`、代码的 `kInit` 同义** ——
>   同一件事的三个写法，见 [`CONTEXT.md`](../../CONTEXT.md) §三的「任务态」条。

### 4.2 转移表

**★ 已实现的部分（ticket 05+06）与本文原表有两处**结构性**差异，先说清楚，再看原表：**

1. **每一条失效边都要按「是否已解锁」分岔**（ticket 06 的用户决策 ①）：
   未解锁时**没有**可中止的任务，请求 `AUTO.LAND` 不但无意义，还会因为
   `AUTO.LAND` 在 `mode_req_prevent_arming` 名单里（`mode_requirements.cpp:142`，同处
   `:141` 还要求 `mode_req_local_position`）而与解锁路径打架
   —— 反过来 OFFBOARD 的要求里**没有** `prevent_arming`（`:121-124`），所以在 OFFBOARD 里解锁是合法的
   ⟹ **未解锁的失效 = 停发 + 回 `INIT`，不请求任何模式**；已解锁的失效才走 `LAND`。
2. **`LAND` 与 `RC_OVERRIDE` 都带"不抢回"语义**：夺权类失效**连 `AUTO.LAND` 也不请求**
   （那是在跟正在接管飞机的人抢），输入类失效才请求 `AUTO.LAND`。详见新建的 §4.4。

| 从 | 到 | 条件 |
|---|---|---|
| `INIT` | `STANDBY` | `state.connected` ∧ 动捕有数据 ∧ `local_position` 有效 |
| `STANDBY` | `TAKEOFF` | 收到解锁命令 ∧ `state.armed` ∧ `state.mode == "OFFBOARD"` |
| `TAKEOFF` | `HOVER` | `|z − 1.0| < 0.10 m` 且 `|v_z| < 0.1 m/s` 持续 1 s |
| `HOVER` | `WAIT_START` | **稳定判据**成立（见 §5.1）；★ **此刻记录轨迹原点** |
| `WAIT_START` | `TRACK` | **遥控器按键上升沿**（去抖 200 ms） |
| `TRACK` | `LAND` | 轨迹执行完毕（`traj_progress ≥ 1.0`） |
| 任意 | `RC_OVERRIDE` | `COM_RC_OVERRIDE` 触发 / 检测到 `state.mode != "OFFBOARD"` |
| 任意 | `LAND` | 电池低于阈值 / 输入超时 / `EKF2` 位置失效 |
| `RC_OVERRIDE` | `STANDBY` | 遥控器交还控制且飞机已稳定 |

> ★ **已实现版本的精确条件**（以 `fsm.hpp::first_degrade()` 与 `mode_is_ours()` 为准，不是上面这张目标表）：
> - ⟹ `RC_OVERRIDE`：**已解锁且**摇杆变化率超 `rc_stick_ov_us_per_s`（`rc_override`）；
>   或我们持有的模式（OFFBOARD / 我们请求的 `AUTO.LAND`）被外部切走（`mode_lost`）。
>   ⚠️ 原表写的"检测到 `state.mode != "OFFBOARD"`"太粗 —— 我们自己请求降落造成的
>   OFFBOARD→`AUTO.LAND` **不算**夺权（`mode_is_ours`）。
> - ⟹ `LAND`：已解锁 + 下列任一 —— `state_timeout` / `mocap_timeout` / `local_pos_timeout` /
>   `rc_timeout` / `battery_low` / `position_invalid` / `mocap_stamp_stalled`。逐个 token 见 §4.4。
> - 原表的"setpoint 未能持续发布"**不作为一条独立判据**：断流的**原因**都在上面那几条里，
>   而"我们自己停发"是结果不是原因（拿结果当判据会自我循环）。
> - ⟹ `MANUAL/INIT`（未解锁时的失效）：判据恢复即**自动**回 `STANDBY`（不闩锁）。

### 4.3 ★ 三个必须实现的细节（"实飞才会想到"的判据，来自 `px4ctrl`）

1. **OFFBOARD 前置流**：切进 OFFBOARD **之前**，setpoint 必须已经以 ≥2 Hz 持续发布 **≥1 s**。本项目用 50 Hz，`STANDBY` 阶段就在发了。**断流 PX4 会自动退出 OFFBOARD**（`COM_OF_LOSS_T = 1.0 s`，见 `z0.params`）。
   ★ **机制已第一手确认**（ticket 05）：起作用的是每条 setpoint 触发的 `offboard_control_mode` 发布，
   而不是 setpoint 本身（切模式前它们被 PX4 丢弃）—— 见 §3.7 的注。
   ★ **参数落点**：`offboard_prestream_s`（默认 1.0）＝"流够多久才允许切"，`setpoint_rate_hz`（默认 50）。
   台架测试断言的是：`前置流设 3.0 s` ⟹ 切模式时刻晚于首条 setpoint **≥ 2.95 s**（**硬约束**，
   差 0.05 s 是计时抖动），且不超过 **4.5 s**（上界是为了抓"根本没切"与"切得太离谱"，
   不是在卡 DUT 多快启动 —— 见 `src/bench/README.md` 的三条纪律）。
2. **`RC_OVERRIDE` 必须真的停止发 setpoint**：只是"不更新目标"是不够的——那等于让飞机悬停在原目标，操作员打杆也拉不回来。**要停发**，让 PX4 按 `COM_OBL_RC_ACT` 接管（`z0.params` 里 `COM_OBL_RC_ACT = 0` = 回 Position 模式）。
3. **`msg_timeout`**：`px4ctrl` 的 `ctrl_param_fpv.yaml` 里每个输入都有 0.5 s 超时（`odom` / `rc` / `cmd` / `imu` / `bat`）。**照抄这个思路**：任何关键输入超时 → 进 `LAND`。

   ★★ **但 0.5 s 这个数字不能照抄到 `/mavros/state` 上**（ticket 05 实测）：
   该话题实测 **1.000 Hz**（min 0.998 / max 1.001）—— mavros **只在收到 MAVLink HEARTBEAT 时才发布它**
   （`mavros/src/plugins/sys_status.cpp` 的 `handle_heartbeat` → `state_pub->publish`），
   而 HEARTBEAT 在 PX4 里是**定频流**（`mavlink/streams/HEARTBEAT.hpp` 的 `const_rate() == true`）
   ⟹ **这不是 mavros 的定时器，改不了 mavros 那一侧**。用 0.5 s 判据的后果是**每秒都在报"飞控失联"**。

   **本项目的超时口径**（ticket 05 实现，见 `src/control/config/control.yaml`）：

   | 输入 | 上游频率 | 超时 | 理由 | 判据（ticket 05 主体入；✅ = 已接上，见 §4.4） |
   |---|---|---|---|---|
   | `/mavros/state` | **1 Hz** | **2.5 s** | 2.5 个周期；容一次丢帧 | ✅ 超时 ⟹ 停发（已解锁时 `LAND`） |
   | `/mavros/local_position/pose` | ~30 Hz | 0.5 s | | ✅ 同上 |
   | `/mocap/pose_enu` | ~196 Hz | 0.5 s | | ✅ 同上 |
   | `/mavros/rc/in` | 10 Hz 级 | 0.5 s | | ✅ **只在已解锁时**超时 ⟹ `LAND`（未解锁时"遥控没开"不影响安全） |
   | `/mavros/battery` | 1 Hz 级 | 2.0 s | | ❌ **`battery_timeout_s` 刻意不做判据**；做的是**低电压**判据（值判断，与超时无关） |

   ★ 后两行的 `*_timeout_s` 参数与订阅**在 ticket 05 就位**（值与理由都不会再变），
   判据在 ticket 06 接上 —— 所以不需要再动话题层。⚠️ **低压保护归 ticket 06，不是 ticket 09**
   （早期几处文档把它记成 ticket 09，已就地更正；09 是"轨迹任务脚本"）。
   ⚠️ `battery_timeout_s` 是**唯一一条刻意不做判据的超时**：低压是**值**判断 ——
   飞控静默之后最后一帧读数若低，它**仍然低**，"超时就不判"是错的方向（详见 §4.4）。

   ⚠️ **代价：解锁上升沿最多晚 1 s 被检出**（1 Hz 话题的固有限制）——
   ticket 08 的起飞逻辑必须把这一秒算进去。本张只做"检出外部解锁 ⟹ 停发 setpoint + ERROR"，
   真正的起飞前置判据归 ticket 08。

### 4.4 ★★ 失效降级（ticket 06 实现记录，2026-09-22）

**总原则（`px4ctrl` 的原话是"任何关键输入超时 → 进 LAND"）在本项目被拆成两步：**

1. **先停发 setpoint**（不是"不更新目标" —— 那等于让飞机悬在原目标上，操作员打杆也拉不回来），
2. **再按「是否已解锁」决定要不要主动中止任务**（用户 2026-09-22 的决策 ①）。

理由是第一手的：`AUTO.LAND` 在 `mode_req_prevent_arming` 名单里（`mode_requirements.cpp:142`）
⟹ 未解锁时请求它既无意义（没有任务可中止）又**会挡住解锁**。
所以**未解锁的失效 ⟹ 停发 + 回 `INIT`，判据恢复就自动回 `STANDBY`**；
**已解锁的失效 ⟹ 停发 + 请求 `AUTO.LAND`（`Phase = LAND`）**。

**两类降级的处置不同，刻意不合并**（`fsm.hpp::is_takeover()`）：

| 类 | 原因 token | 处置 |
|---|---|---|
| **控制权被夺走** | `rc_override`、`mode_lost` | 停发 + 交还控制权（`MANUAL/RC_OVERRIDE`），**不请求任何模式** —— 既不抢回 OFFBOARD，**也不请求 `AUTO.LAND`**（那是在跟正在接管飞机的人抢） |
| **输入坏了** | `state_timeout`、`mocap_timeout`、`local_pos_timeout`、`rc_timeout`、`battery_low`、`position_invalid`、`mocap_stamp_stalled` | 停发；已解锁时请求 `AUTO.LAND` |

**闩锁与复位**（"可判定、可复位"，与 XI35 的 `kf_fusion_fail` 单向 latch 相对）：

| 闩锁种类 | 谁闩 | 复位条件 |
|---|---|---|
| 不闩锁 | 未解锁时的任何失效 | 判据恢复 ⟹ 立即回 `STANDBY`（**不需要任何动作**） |
| `kTakeover` | 夺权（任何解锁状态） | ① 落地上锁（`armed` true→false）；② 模式被**外部**交回 OFFBOARD（升沿 —— 闩锁期间我们不请求 OFFBOARD，所以只可能来自人）。
★ 判据本身**不含**"摇杆要先停下"：那是**自然结果** —— 飞行员若仍在打杆，复位后第一帧 `rc_override` 就重新成立 ⟹ 看上去像"没复位"。行为等价，但别把它当成一条额外条件 |
| `kArmedDegrade` | 已解锁时的输入失效 | **只有**落地上锁（电池不会因为有人切了模式就变满） |

★ 闩锁记的是**首因**（最接近根因），后续判据不改写它；唯一的例外是
**夺权可以升级已闩锁的输入失效** —— 飞行员永远优先（否则会出现"人在接管、我们还在请求 `AUTO.LAND`"的拉锯）。

★ **首因只在同一帧内按优先级取胜**（`first_degrade()` 的顺序：飞控失联 > 动捕 > 位置 > 遥控 > 低压 …），
**不同帧之间比的是各自的超时长短** —— 实测：五条输入同时静默时先报 `local_pos_timeout`（0.5 s），
2.5 s 之后才轮到 `state_timeout`。这不是 bug，日志里两条都会出现。

★ **日志可检索**：每帧降级写 `degrade=<token>`（token 由 `fsm.hpp::to_string(Degrade)` 给，有 `-Wswitch` 兜底），
复位那帧写 `degrade=none：降级已复位（此前 degrade=<原因>，持续 X s）`。
`grep -oE "degrade=[a-z_]+" 日志 | grep -v none` 捞全部降级事件。

#### ★★ 两条容易做错的判据细节（2026-09-22 复核补，都有第一手依据）

**(1) `mode_lost` 要看**丢的是哪个模式**。** 判据是"**上一帧**模式是我们的 **且上一帧我们在流
setpoint（或已经在**我们请求的** `AUTO.LAND` 里）**，这一帧模式不是我们的"。
只比"模式从 `OFFBOARD` 掉出去"是不够的：我们一停发，PX4 在 `COM_OF_LOSS_T`（1.0 s）后
**自己**退出 `OFFBOARD` —— 而我们的判据是 0.5 s，（同一个输入坏掉时）**真机上必然先看见"自己退出"那一帧**，
是确定性发生而不是偶发。误判的代价有两层：日志把排查方向指错；更实际的是
`is_takeover()` 会把已闩锁的输入失效**升级**成夺权 ⟹ 从此**不再重发 `AUTO.LAND`**
（闩锁把"降落失败就重试"的能力一起掐掉）。代价是**刻意接受**的：人手动把模式切走、而飞机当时
并未被我们驱动 ⟹ 交给**打杆**那条判据去抓（这也正是 PX4 `COM_RC_OVERRIDE` 的口径：人一动杆就是接管）。
★ 这一条的证据**只可能在 seam 2**（`test_fsm.cpp` 的三条用例）：夹具的假 mavros **不假扮 PX4 的
`COM_OF_LOSS_T` 退出**（`CONTEXT.md`），台架上永远造不出"我们自己停下来、PX4 跟着退出"这个形状。

**(2) 动捕戳的基准是**历史最大戳**，不是"上一帧的戳"。** 依据是 EKF2 自己的尺子：它收外部视觉样本
的条件是"比**已缓冲的最新样本**还新"（`EKF/estimator_interface.cpp:393`），不满足就只打一条
`ECL_WARN("EV data too fast")` 把这一帧丢掉（`:401-403`）。拿"上一帧"当基准的话，
**倒退到旧纪元之后继续正常前进**（夹具 `VrpnSource::rewind` 的形状）每一帧都比上一帧新
⟹ 我们的判据永远说"健康"，而 PX4 那边每一帧都在丢 —— 那才是真盲区（`position_invalid` 看不见它，
`mocap_timeout` 也看不见它，帧确实在到）。
★ 判别力是**实测**的，不是推演（2026-09-22）：探针量到倒退窗口里戳的真实走向 —— 一跳 **−4.9955 s**，
之后**每帧前进 +5 ms**；把 `on_mocap` 临时改回"基准 = 上一帧的戳、每帧无条件更新"，
`test_动捕时间戳倒退到旧纪元_持续超过判据即降级` 立刻变红（DUT 全程停在 `AUTO/STANDBY`，
一条降级都没有），而"戳冻住"那条仍然通过（戳相等 ⟹ 两种基准都成立）。

#### ⚠️ 本张**不能**兑现的三件事（诚实记录，别当成已完成）

1. **「EKF2 位置失效 ⟹ LAND」在这条链上抓不到**（`position_invalid` 判据是**防御性**的）。
   第一手核实（z0 固件 `4a0e65f2`，见 ticket 06 的 Comments）：
   `EKF2.cpp::PublishLocalPosition` 发的 `lpos.x/y/z` 没有 NaN 合成；
   `LOCAL_POSITION_NED.hpp` 在 MAVLink 层就丢掉了 `xy_valid`；mavros 2.15.1 原样转 ENU。
   ⟹ **真正的兜底是动捕超时**（动捕是本项目唯一的位置源）；`estimator_status` 那条候选载体开在 ticket 16。
2. **打杆接管的最终执行是 PX4 的事**（`COM_OBL_RC_ACT`，z0 现值 `0` = 回 Position 模式），
   夹具不模拟 PX4 ⟹ 台架只能验到"我们停发了、退到 `RC_OVERRIDE`、不抢回"。
   连带 ⚠️：**`NAV_RCL_ACT = 2`（Return）是 A1 阻塞项** —— A1 不落实，"RC 丢失 ⟹ 我们请求 LAND"
   之后 PX4 自己的 RC 失效保护仍会试图**返航**（室内无 GPS）。
3. ⚠️ **`/mavros/state` 超时这条在真机上不是"请求 `AUTO.LAND` 直到结束"**（2026-09-22 第二轮复核补，
   第一手：mavros 2.15.1 `sys_status.cpp`）。真丢链路时 mavros 会在 `conn_timeout`（默认 **10 s**，`:512`
   的 wall timer → `timeout_cb()` `:1311` → `connection_cb(connected=false)`）之后**自己发一帧合成状态**
   `connected=false, armed=false, guided=false, mode=""`（`publish_disconnection()` `:903-911`，`:1411` 处调用）。
   时序：HEARTBEAT 停 ⟹ 本节点 2.5 s 的判据先成立（此时 `armed` 仍是 true）⟹ 停发 + 请求 `AUTO.LAND`；
   再过 ~7.5 s 那帧合成状态到达 ⟹ `armed` 降沿**把闩锁复位**（`fsm.hpp:396` 只判 `armed`，不判 `connected`）
   ⟹ 节点回 `INIT`、不再请求（`guards` 里 `connected=false` ⟹ 仍报 `state_timeout`，日志不停）。
   **为什么这样不算错**：链路已断，`set_mode` 请求根本到不了 PX4，继续请求只是刷日志；
   那种场面的兜底是 PX4 自己的失效保护（`COM_OBL_RC_ACT` / `NAV_RCL_ACT`）。
   ⚠️ **真正的风险在"链路回来"那一刻**：那帧合成的 `armed=false` 把"落地上锁才复位"的闩锁
   **在空中**清掉了；若链路恢复时 PX4 仍 armed（我们停发后它多半已自行退出 OFFBOARD），
   节点会回到 `AUTO/STANDBY` 并**重新请求 OFFBOARD** —— 正是闩锁本该拦住的事。见 **ticket 18**（等决策，不擅自改安全语义）。
   ★ 夹具造不出这个形状：`FakeMavros.mute("state")` 只造**静默**（假 mavros 不假扮 mavros 自己的
   `conn_timeout` 逻辑）⟹ `test_control_failsafe.py` 的 state 分支证明的是"静默"那一半。

#### ★ ticket 06 与 08 之间的临时缝：`own_arming`

"已解锁时的失效 ⟹ 请求 `AUTO.LAND`"这条在台架上**没法验**（本张没有解锁路径）。
于是加了参数 `own_arming`（默认 `false`）：置 `true` = 把任何 `armed` 上升沿当作**本节点发起的解锁**
（⟹ 走 `LAND` 分支、不再报"外部解锁"）。**只在台架注入时用，实飞前保持 `false`，
ticket 08 落地时删掉它**。

#### ⚠️ `battery_timeout_s` 为什么不用

它的值（2.0 s）没错，错的是"拿它当判据"这个动作：低电压是**值**判断。
飞控静默之后最后一帧电压若低，它**仍然是低的** —— 此时"超时 ⟹ 不判低压"方向反了。
而且真出现"电池话题静默"时，`/mavros/state` 那条判据几乎必然一起成立（同一个 mavros）。
⟹ 本张**不加**"电池话题静默 ⟹ 降级"，保留 `battery_timeout_s` 参数与订阅只为记账/诊断。

## 5. 判据与轨迹

### 5.1 "稳定"判据（已定）

```
位置误差 < 0.10 m  ∧  |v| < 0.10 m/s      持续 2 s
```

- **位置误差**相对**轨迹原点**（§3.3），来源 `/mavros/local_position/pose`
- **速度**来源 `/mavros/local_position/velocity_local`（EKF2 融合，比动捕差分干净）
- **两者都在 EKF2 的同一个局部系里**（ROS 侧是 ENU，见 §3.1 的更正），与 setpoint 同口径，**不需要额外转换**

### 5.2 三条轨迹（已定，都在 §3.3 的轨迹原点、同一个**局部系**下定义）

> ⚠️ 该局部系在 **ROS 侧是 ENU**（`x` 东 / `y` 北 / **`z` 向上为正**）—— 见 §3.1 的更正。
> 下表"高度 1.0 m"对应的就是 **`z = +1.0`**。
>
> ✅ **2026-09-22 ticket 07 落地时的口径澄清（`traj` 节点里的"高度"是 0，不是 1.0）**：
> **1.0 m 这个高度属于"轨迹原点"，不属于轨迹本身。** 轨迹原点由 `control` 在 HOVER 判稳那一刻
> 从 `/mavros/local_position/pose` 记下来（§3.3），那时飞机**已经在 1 m 上**——
> 所以轨迹 1 在轨迹原点系里的目标就是 **`(0,0,0)`**，正方形的四个角是 `z = 0`。
> 把 1.0 再写进轨迹里 ⟹ 参考高度变成 **2 m**（叠加而非替代）。
> 实现上对应的参数是 `height_offset_m`（[`src/traj/config/traj.yaml`](../../src/traj/config/traj.yaml)），**默认 0.0 且唯一用途是台架上给视频做区分度**。

| 编号 | 轨迹 | 定义 | 速度 |
|---|---|---|---|
| **1** | 原地悬停 | 目标恒为原点，高度 1.0 m | — |
| **2** | 正方形 | 边长 **1.0 m**：`(0,0) → (1,0) → (1,1) → (0,1) → (0,0)` unit: m | ≤0.5 m/s |
| **3** | 八字 | Gerono 双纽线，a = **0.5 m**：<br>`x(t) = a·sin(t)`，`y(t) = a·sin(t)·cos(t)`，`t: 0→2π` | ≤0.5 m/s |

**★ 关于八字的实际尺寸**（数值积分算出来的，不是估的）：

- `x ∈ [−0.5, +0.5]`（宽 **1.0 m**），`y ∈ [−0.25, +0.25]`（高 **0.5 m**）
- 周长 **3.05 m** → 0.5 m/s 下一圈 **≈6.1 s**
- 正方形：周长 4 m → **8.0 s**

> 📌 **注意**：Gerono 双纽线的 y 幅度**只有 x 的一半**（这就是"a = 0.5 m"下 8 字偏扁的原因）。**若想要两个方向都是 ±0.5 m 的对称 8 字**，改用 Bernoulli 双纽线 `x = a·cos t/(1+sin²t)`，`y = a·sin t·cos t/(1+sin²t)`。**这是个可以随时改的显示偏好，不影响控制逻辑**——所以先按 Gerono 做，飞完不满意再换。

**实现方式（两条都行，推荐 (b)）**：

- **(a) 航点式**：逐点飞到，**每点用 §5.1 的判据确认停稳**再走下一个。简单、稳、但慢。
- **(b) 连续参考式**：**按时间采样**出解析参考（50 Hz 与 setpoint 同频），直接流给 PX4。轨迹在动捕原点坐标系里是"mocap x 为前"的直观方向，画出来干净。

> **★ (A) 级方案的最大好处在这里体现**：我们**只发位置**，动力学全部由 PX4 的 `MPC_*` PID 处理。**轨迹发生器不需要算任何加速度或姿态**——这就是为什么 Phase 1 能很快做出来。

**速度上限的实现**：**按弧长推进，不是按参数 `t` 匀速**。
正方形把弧长 `s = clamp(t)·v` 后按 `s / 边长` 取边；八字先建一张 `θ → 弧长` 的数值表再反查
（`x = a·sinθ` 的速度因子是 `√(cos²θ + cos²2θ)`，数值上 ∈ `[0.661, 1.414]`、均值 `0.970` ⟹
**按 θ 匀速时过中心那一点是平均速度的 1.46 倍**（最慢处只有 0.68 倍）；
把"平均"压到 0.5 m/s 则峰值到 **0.73 m/s**，直接违反上限）。
⇒ 时长：正方形 4 m → **8.0 s**；八字周长 **≈3.05 m → ≈6.1 s**。
（📌 旧版这里写的"八字周长 ≈ 4.2 m → 8.4 s"是**估的**，2026-09-22 按数值积分更正；
`src/traj/test/test_trajectories.cpp` 里有一条独立用 Simpson 法算周长的交叉核对盯着它。）

### 5.3 验收标准（已定）

- [ ] 定点**水平误差 ≤ 15 cm**
- [ ] **持续 60 s**
- [ ] **全程不碰摇杆**
- [ ] 能触发自动降落

## 6. 参数（`z0.params` 中需要核对或修改的）

✅ **2026-09-21 已与飞控真身逐条对账**：`z0.params` 的 914 条参数与飞控上读到的 **914/915 条名字全对上**
（唯一多出的是飞控内部的 `_HASH_CHECK`），**下表每一条的值都在飞控真身上读到了同一个数**
（21 条值差异全是设备标定类 / 飞行计数器类，与配置无关）。
⟹ **本表描述的就是飞控现状**（不再是"上游参考值"）；下面判 ⚠️ 的几条是**飞机上真实存在的待改项**。
对账全文与待办见 [`.scratch/phase1/issues/15-fcu-param-reconciliation.md`](../../.scratch/phase1/issues/15-fcu-param-reconciliation.md)。
☠️ **不要整份烧 `z0.params`**（会覆盖 z0 自己的加速度计与遥控器标定）—— 要改只挑配置类参数写。

★ 下表保留 Phase 1 前逐条确认的用途（**确认动作已做，结论见上**）：

| 参数 | 现值 | 判断 | 说明 |
|---|---|---|---|
| `EKF2_EV_CTRL` | `11` | ✅ 对 | 用外部视觉的位置 + yaw |
| `EKF2_EV_DELAY` | `50.0` | ✅ 对 | 与实测 50 ms 吻合 |
| `EKF2_MAG_TYPE` | `5` | ✅ 对 | = none，无磁力计（这正是"机头朝哪，哪就是北"能成立的前提） |
| `EKF2_HGT_REF` | `3` | ✅ 对 | = Vision，高度用动捕 |
| `COM_RC_OVERRIDE` | `3` | ✅ 对 | ★ 打杆能踢出 Offboard，**实飞前必须验证一次** |
| `COM_OBL_RC_ACT` | `0` | ✅ 对 | Offboard 被 RC 接管后回 Position 模式 |
| `COM_OF_LOSS_T` | `1.0` | ✅ 对 | Offboard 掉线 1 s 后触发 failsafe |
| `MAV_1_CONFIG` / `MAV_1_MODE` | `102` / `2` | ✅ 对 | TELEM2 = Onboard，**mavros 走这个口** |
| ⚠️ **`EKF2_EVP_NOISE`** | **`0.061`** | ⚠️ **偏小，需重调** | 用户实测笔记：**1.0 可用，0.1 过小**（过小 → `LOCAL_POSITION_NED` 发散）。**0.061 比 0.1 还小。** Phase 1 前按笔记的建议从 **1.0** 起步往下调 |
| ⚠️ **`MPC_XY_VEL_MAX`** | **`12.0`** | ⚠️ **过高，建议降** | 12 m/s 对 152 mm 涵道机是失控速度。Phase 1 建议降到 **1.0–2.0**，与 ≤0.5 m/s 的轨迹匹配 |
| ⚠️ **`BAT1_N_CELLS`** | `6` | ✅ 对（6S） | 但**同平台的板级默认是 4**，说明这份参数已按 z0 改过 |
| ⚠️ **`MPC_THR_HOVER`** | `0.50` | ⚠️ **需标定** | 悬停推力 50%。XI35 的实测值是 0.45。**这直接决定高度控制的好坏**，用 `thrust_calibrate` 流程标定 |
| `THR_MDL_FAC` | `0.45` | ✅ 对 | 与 XI35 的 `K3: 0.6786` **不同**，★ 两个值至少有一个是错的，**标定时一并确认** |
| `MC_ROLLRATE_MAX` / `MC_PITCHRATE_MAX` | `220` | 🔶 记录 | Phase 2 的 MPC 角速率上限不得越过它 |
| `COM_RC_IN_MODE` | `3` | 🔶 **待核实** | 含义与 z0 的遥控器接法要对上（见 [`params-analysis.md`](../reference/params-analysis.md) 3.9） |
| `RC_MAP_FLTMODE` | `6` | 🔶 **待核实** | ★ **开始按键所选的通道不能和它冲突**，见 §8.4 |

## 7. 验证顺序（已定，**跳过 Phase 1 的 SITL**）

| 步 | 做什么 | 通过判据 | 出问题看哪 |
|---|---|---|---|
| **0** | `vrpn_listener` 起来，`ros2 topic hz /vrpn/<刚体>/pose` | 稳定 ~100–400 Hz | 动捕 WiFi / `params.yaml` 的 server |
| **1** | `mocap` 节点起来，QGC 看 `VISION_POSITION_ESTIMATE` | 有数且不跳 | `mavros` 的 `vision_pose` 插件 |
| **1b** | ★ **坐标系确认实验**：把飞机**向动捕的 +x 方向（东）平移 0.5 m**，观察 QGC 的 `LOCAL_POSITION_NED` | **`y` 变化 0.5**（东）→ 与 §3.5 的源码结论一致 ✅ | 若 `x` 变化 → 说明有地方重复转换了，查 `mocap` 节点是否多转了一次 |
| **1c** | 把飞机**升高 0.3 m**，看 QGC 高度 | **向上为正**（`z_ned` 是负的，但 QGC 显示海拔向上） | 同上 |
| **2** | **遥控器 Position 模式**悬停 | 稳、不漂 | `EKF2_EVP_NOISE` / `MPC_THR_HOVER` |
| **3** | **无桨台架**跑完整 offboard 流程 | FSM 走完 `INIT→…→WAIT_START`，电机按预期转 | 别装桨！ |
| **3b** | 台架上**按遥控器按键** | 进 `TRACK` 且 `/control/state` 正确 | `start_channel` 参数 / §8.4 |
| **3c** | 台架上**打杆** | 立刻进 `RC_OVERRIDE` 并停发 setpoint | `COM_RC_OVERRIDE` |
| **4** | **带桨 offboard 定点**：轨迹 1 | §5.3 的验收标准 | — |
| **5** | 轨迹 2（正方形） | 画得出来、不超调 | `MPC_XY_VEL_MAX` |
| **6** | 轨迹 3（八字） | 同上 | — |

> **★ 第 3 步是"无桨台架"，不是 SITL。** 理由：SITL 验证不了**动捕链路**、**mavros 的真实串口**、**遥控器按键**这三件 Phase 1 真正的风险点。无桨台架能验证全部三件，且不炸机。

### 7.1 ★ 哪些已经进了自动化，哪些还必须上真台架（ticket 06 更新）

**已经自动化**（`colcon test --packages-up-to control`，共 96 例，不需飞控、不需动捕、可离线跑）：

- **seam 1**（`test/test_control_standby.py` 9 例 + `test/test_control_failsafe.py` 13 例，`src/bench/` 夹具）：
  守卫逐条生效 ⟹ 不发 setpoint；50 Hz 流的字段与频率；前置流时长；`/mavros/state` 静默后流在
  `state_timeout_s` 内停止；外部解锁 ⟹ 停发；`enter_offboard=false` 只流不切；杀进程后流停止。
  **ticket 06 的那一半**：打杆 ⟹ **立刻**停发 + 退 `RC_OVERRIDE` + 不抢回（配正控：此前确实请求过 OFFBOARD）；
  外部切走模式同前；三条输入超时**各一条判据**、任一条 ⟹ `LAND`（参数化三例）；
  低压 ⟹ 主动中止任务、落地上锁后复位；NaN 位置 ⟹ 停发 + `LAND`；未解锁时的降级只停发、判据恢复即自动复位；
  动捕戳**冻住** ⟹ 降级而**短促倒退不降级**；戳**倒退到旧纪元并持续** ⟹ 降级；
  五条输入同时静默 + `set_mode` 服务撤走 ⟹ 不崩；遥控帧通道数不足 ⟹ 不崩。
- **seam 2**（`test/test_fsm.cpp` 42 例 + `test/test_rc_stick.cpp` 13 例，零 ROS）：缺项清单是哪几条、
  什么顺序；断流后前置流重新计时；`STANDBY` 下控制权仍是 `MANUAL`。
  **ticket 06 的那一半**：两类降级的处置差异、闩锁种类与复位矩阵（含"夺权可升级已闩锁的输入失效"）、
  首因闩锁、`mode_lost` 按**丢的是哪个模式**区分（含"我们自己停发后 PX4 跟着退出不算夺权"的解锁/未解锁两条 ——
  见 §4.4(1)，这一条**只能在 seam 2 验**）、摇杆变化率检测器（阈值换算、断链后不复用旧读数、通道不足）。

★ 上面说的 96 例 = `control` 自己的 77（42 gtest + 13 gtest + 9 + 13）+ `bench` 夹具自检 19
（`test_bench_selftest.py` 13 + `test_timeline.py` 6，**它自己的测试**，不是 `control` 的验收项）。

**`traj`（ticket 07）同样是两层**，`colcon test --packages-up-to traj` = **31 例**全绿
（seam 2 的 20 + seam 1 的 11；★ 它的 seam 1 也用 `bench` 夹具，所以同样要 `--packages-up-to`）：

- **seam 2**（`traj/test/test_trajectories.cpp`，零 ROS，20 例）：三条几何的形状/尺寸/闭合、
  弧长均匀性（有限差分量速度波纹）、**限速扫过三条轨迹**、周长（含一条独立的 Simpson 法交叉核对）。
- **seam 1**（`traj/test/test_traj_node.py`，11 例）：`select` 与 `start` 的语义（**选了不 start ⟹ 一帧都不发**，
  且配了正控）、消息口径（`type_mask = 2496`、`FRAME_LOCAL_NED`、`frame_id`、恒定 yaw）、
  50 Hz **窗口**频率、正方形四条边且跑完停回原点、8 字的幅度与"偏扁"不变式（`max(x)/max(y) ≈ 2`）、
  **RViz 折线的形状/尺寸/周期重发** —— 题目里那条"人眼看形状"的验收项由此变成机器可查的。

> ★ `traj` 上实打实踩到的一条测试纪律：**差分速度必须用 `header.stamp`，不能用收到消息的时刻。**
> rcl 的定时器从**回调开始**重新计时，回调一超时就立刻补跑 ⟹ 接收时刻上会出现相邻仅 0.1 ms 的两帧，
> 而它们之间隔着 0.1 s 的运动 —— 用接收时刻算出来正方形"飞到 **4.5 m/s**"（限速的 9 倍），全是假的。
> 频率断言同理：**只在窗口上算**（`bench/README.md` 的纪律 2）。

**这些都不再需要人守着看。**

**⏳ 必须上真台架**（夹具**不模拟 PX4**，下面几条的判据全在 PX4 侧）：

| 项 | 为什么夹具做不了 |
|---|---|
| 真 PX4 被**拉进 OFFBOARD** | 模式需求检查（解锁前/后的差异）、`offboard_control_mode` 的存活判据都在 PX4 里 |
| 杀节点 / **我们自己停发** ⟹ PX4 在 **`COM_OF_LOSS_T`（1.0 s）后自行退出 OFFBOARD** | 同上。夹具已能证明"节点停发了"，**证明不了**"PX4 因此退出了"。★ 这一条还是 `mode_lost` 判据的**真机现场**（§4.4(1)）：我们的判据是 0.5 s，比 PX4 的 1.0 s 先到 ⟹ 真机上只要输入坏掉就**必然**先看见"PX4 自己退出"这个形状，而夹具永远造不出它 |
| **打杆 ⟹ PX4 按 `COM_OBL_RC_ACT` 接管**（z0 现值 `0` = 回 Position 模式） | 同上：执行在 PX4。本节点这一半（立刻停发、退到 `RC_OVERRIDE`、**不抢回 OFFBOARD**）已由 `test_control_failsafe.py` 覆盖 |

做这几项时 **必须拆桨**，并走本节的第 3 步序列。⚠️ 当前 `/dev/ttyTHS1` 被另一会话的 mavros 占着
（见 [ticket 14](../../.scratch/phase1/issues/14-mavros-from-source.md)），**不在 ticket 05 里抢**。

## 8. ★ 避坑清单（本轮调查挖出来的，逐条都有第一手证据）

### 8.1 ⚠️ 不要把"公开版 vrpn_listener"替换进来

`vrpn_listener.zip` 里的 `README.md` **还在教你 clone 公开的 `efc-robot/vrpn_client_ros2`**——**这是过时的**。源码已被定制（`stamp = now()` 替换了 `msg_time`，见 A.2）。
→ **vendored 后写一个 `VENDORED.md`**，写明来源、定制点、"勿替换"。

### 8.2 ⚠️ 电池电流采样**大概率**读的是错误通道

上游修复 commit **`129c0f188a`（2024-11-19，"hkust_nxt-dual: fix ADC_BATTERY_CURRENT channel mismatch"）**把 `ADC_BATTERY_CURRENT_CHANNEL` 从 `ADC12_CH(5)` 改为 `ADC12_CH(8)`。

| PX4 版本 | `nxt-dual` | `nxt-v1` |
|---|---|---|
| v1.15.0（2024-08-23） | `ADC12_CH(5)` ❌ | `ADC12_CH(5)` |
| v1.16.0 / main | **`ADC12_CH(8)`** ✅ | `ADC12_CH(5)` |

**★ 为什么说"大概率"而不是"万一"**：**z0 是 1.14.x，连 `hkust/nxt-dual` 这个板级目标都还不存在**（v1.15.0 才引入）→ **1.14 的构建几乎必然带 `CH(5)`**。

⚠️ **修复只针对 `nxt-dual`。`nxt-v1` 在 main 上仍是 `CH(5)`（对它是对的）——不要照抄。**

→ **若中招，电流/电量保护全是空转**：你会在电量充足时被强制降落，或更糟，**在过流时毫无保护**。
→ **排查**：上电后在 QGC 看电流读数——未解锁应 ≈0，上电后应合理。**恒为 0 或恒为怪值 = 中招。**

### 8.3 ⚠️ 不要刷 Nxt-FC 仓库里的预编译固件

`firmware/` 里的 `.elf` 是 **`hkust_nxt-v1`（单 BMI088 + BMP388）**，z0 是 **NxtPX4v2（双 BMI088 + SPL06）**。刷上去**传感器直接不工作**。
→ **用飞控出厂自带的 PX4 1.14，不自己刷。**

### 8.4 ⚠️ 开始按键的通道不能和 PX4 的模式开关冲突

`z0.params` 里 `RC_MAP_FLTMODE = 6`（第 6 通道是模式开关）。
→ **开始按键必须选一个未映射的通道**（或用未被 `RC_MAP_*` 占用的开关位）。否则按下去会**同时切换 PX4 飞行模式**，把我们踢出 OFFBOARD。
→ 参数化实现：`start_channel`（`rc/in` 的 `channels[]` 索引）+ `start_threshold` + 200 ms 去抖。

> ⚠️ **本节原先只给约束、没给通道号。** 完整占用表（含 **ch5 解锁**、**ch8 杀车** —— 本节原先漏了这两个）
> 见 [`../reference/params-analysis.md`](../reference/params-analysis.md) §3.15；
> ~~实际只剩 ch7 空闲~~ ☠️ **该句已作废（2026-09-21）**：未映射的通道是 **ch7 与 ch9…ch16**（共 9 个）。
>
> ★ **2026-09-21 已定案**（ticket [02](../../.scratch/phase1/issues/02-rc-channel-decision.md)）：
> **开始 = ch9 上升沿**（`channels[8]`）、**急停 = ch8 高值（沿用现有硬件杀车）**、**解锁 = 程序解锁**。
> 完整结论/依据/连带要求见 [`../reference/params-changes.md`](../reference/params-changes.md) **§H**
> （含一条要紧的连带结论：**首飞前遥控必须开机**，否则急停与打杆接管从解锁那刻起就是空的 —— §H.7.1）。
>
> **实现上仍然参数化**（`start_channel` / `start_threshold` / `release_threshold` / 200 ms 去抖，
> 值取 §H.1），且**边沿语义、上电即在高位不算边沿**这两条要照 §H.5 实现。

### 8.5 ⚠️ `TrajectorySetpoint` 没有 `thrust` 字段

网上大量教程说用 `TrajectorySetpoint` + `type_mask` 推姿态和推力。**错的**——它**根本没有 `thrust` 字段**。要发姿态得用 `VehicleAttitudeSetpoint`（四元数 + `thrust_body`）。
→ Phase 1 我们用 mavros 的 `setpoint_raw/local`，**不涉及这个问题**；Phase 2 要注意。

### 8.6 `mavros_msgs/PositionTarget` 的 `type_mask`

Phase 1 = **纯位置 + yaw**，对应的位掩码（置 1 = 忽略）：

```
IGNORE_VX|IGNORE_VY|IGNORE_VZ|IGNORE_AFX|IGNORE_AFY|IGNORE_AFZ|IGNORE_YAW_RATE
= 8|16|32|64|128|256|2048 = 2552
```

同时：`coordinate_frame = FRAME_LOCAL_NED (1)`。

☠️ **更正（2026-09-21）**：本节旧版写「`position.z` **向下为正**，`yaw` 用弧度（推荐 `yaw = 0` = 解锁时的机头方向）」——
**两句都错**（详见 §3.1 的更正）：mavros 对这个帧做 ENU→NED（`setpoint_raw.cpp:231/235`）⟹
**`position.z` 向上为正**、**`yaw = 0` 表示"机头朝 ENU 的 +x"而不是"解锁时的机头方向"**。
`yaw = 0` 按"保持当前机头"用，会在进入 OFFBOARD 的瞬间把机头拧到 90°（NED）。

> 📌 **若定点出现低频抖动**，标准下一步是**加上速度前馈**：轨迹发生器同时给出参考速度，把掩码改成 `2496`（只忽略加速度与 yaw_rate），填入 `velocity`。**这是 (A) 级方案的标准调优路径**，先不做。

> ⚠️ **别把"参考的掩码"和"下发给 PX4 的掩码"混成一个数** —— ticket 07 之后仓库里这两个数**故意不一样**：
>
> | 消息 | 掩码 | 为什么 |
> |---|---|---|
> | `traj` → `/traj/reference` | **2496**（位置+速度+yaw） | 它是**参考值**，由 `control` 消费。速度是免费的（轨迹本来就是解析式，顺带把导数给了），留着它 `control` 才有得选 |
> | `control` → `/mavros/setpoint_raw/local` | **2552**（位置+yaw） | Phase 1 的既定口径（本节上面那个表达式）。**速度前馈是上面 📌 里"先不做"的调优**，收不收 `traj` 的速度是 `control` 单方面的决定，与参考里带不带速度无关 |
>
> 两块掩码**唯一的相同点是坐标系**（`FRAME_LOCAL_NED`）。★ 另一条纪律：`/traj/reference`
> **永远不许 remap 到 `/mavros/setpoint_raw/local`** —— 它相对**轨迹原点**（= 1 m 高度上的悬停点），
> 直接转发 = 先往地面附近扎，见 §2 与 §3.3。

---

---

## 9. 需要**你**物理操作的 4 件事

| # | 做什么 | 用来确认 |
|---|---|---|
| **1** | **把 Jetson 连到动捕所在的那个 WiFi**（不是 `SUSTech-802.1x`），拿到 `10.1.1.x` 后验证 `ping 10.1.1.198` | 当前实测：路由走校园网关 `10.31.255.254`，**100% 丢包** |
| **2** | **在 QGC 看固件版本**，并**看电池电流读数**（未解锁应 ≈0） | 固件来源 + §8.2 的 ADC bug |
| **3** | **确认飞控接的是 `/dev/ttyTHS1` 还是 `/dev/ttyTHS2`** | 实测 z0 只有这两个串口（`THS0`/`THS3` 不存在，也没有 `/dev/ttyACM*`、`/dev/ttyUSB*`） |
| **4** | **实飞时确认遥控器那个按键对应哪个通道** | §8.4 的 `start_channel` |
---

---

## 附：Phase 2 会追加什么（**不推翻本文件**）

| 新增 | 内容 |
|---|---|
| 中间件 | 全 mavros2 → **mavros2 + uXRCE-DDS**（各连不同串口，不冲突） |
| 控制输出 | `/mavros/setpoint_raw/local` → **`/fmu/in/vehicle_rates_setpoint`**（`(C)` 级） |
| 状态输入 | `/mavros/local_position/*` → **`/mocap/pose_enu`**（★ 此时才依赖动捕原点） |
| 求解器 | 无 → **acados**（`SQP_RTI` + HPIPM） |
| 代码骨架 | 无 → 🔶 **`px4-mpc`** |
| 理论 | 无 → 6 篇论文 |
| 内环 | PX4 的 `MPC_*` PID → 同一套 → 之后考虑 **INDI** |

⚠️ **Phase 2 上线前必须先做 SITL 验证**（Phase 1 跳过了 SITL，Phase 2 不跳）—— 因为 MPC 的代价函数、约束、符号约定在实机上试错代价太高。

**详见 [`phase2-implementation.md`](phase2-implementation.md)。**
