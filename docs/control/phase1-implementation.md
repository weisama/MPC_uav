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
├── mocap/                             🔄 重写
│   ├── src/mocap_rel_pose_node.cpp    ← 相对位姿（核心）
│   ├── include/mocap/mocap_rel_pose.hpp
│   ├── config/mocap.yaml
│   └── launch/mocap.launch.py
│
├── traj/                              🆕 新写
│   ├── src/traj_node.cpp
│   ├── include/traj/{hover,square,lemniscate}.hpp
│   ├── config/traj.yaml
│   └── launch/traj.launch.py
│
├── control/                           🆕 新写（Phase 2 的 MPC 也放这里）
│   ├── src/control_node.cpp           ← FSM 主体
│   ├── src/px4_interface.cpp          ← mavros 服务/话题封装
│   ├── include/control/fsm.hpp
│   ├── include/control/px4_interface.hpp
│   ├── config/control.yaml
│   └── launch/control.launch.py
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
| `/mavros/rc/in` | `mavros_msgs/RCIn` | **开始按键** | ★ 通道号是参数，实飞时确认 |
| `/mavros/battery` | `sensor_msgs/BatteryState` | 低电压保护 | ⚠️ 见 §8.2 |
| `/mocap/pose_enu` | `geometry_msgs/PoseStamped` | 相对位姿（**ENU，原生**） | Phase 1 只用于记录（Q23）；**Phase 2 的 MPC 拿它当状态源** |
| `/traj/select` | `std_msgs/UInt8` | 选轨迹：1/2/3 | 由 `fly.py` 发布 |

### 输出（`control` 发布 / 调用）

| 话题 / 服务 | 类型 | 用途 |
|---|---|---|
| `/mavros/setpoint_raw/local` | `mavros_msgs/PositionTarget` | ★ **唯一的控制输出** |
| `/mavros/set_mode` | `mavros_msgs/SetMode` | 切 `OFFBOARD`（服务） |
| `/mavros/cmd/arming` | `mavros_msgs/CommandBool` | 解锁/上锁（服务） |
| `/control/state` | `std_msgs/String` | 当前状态名，给 `fly.py` 显示 |
| `/control/traj_progress` | `std_msgs/Float32` | 轨迹完成度 0–1 |

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
> ① Phase 1 的控制器工作在 **EKF2 的 local NED** 里（§3.3），位置直接读 `/mavros/local_position/*`，
> **根本不需要** mocap 的 NED 值；② Phase 2 的 MPC 用 ENU（§3.2 / G3）。
> 即**没有任何消费者**。删掉它同时消灭了"转两次 / 一次都没转"这一整类 bug。

## 3. 坐标系与符号（★ 写死，不靠记忆）

### 3.1 三套坐标系

| 坐标系 | 约定 | 出现在哪 |
|---|---|---|
| **动捕 map（ENU）** | x=假定东 / y=北 / z=**上** | `/vrpn/<刚体>/pose` |
| **PX4 本地（NED）** | x=北 / y=东 / z=**下** | `/mavros/local_position/*`、`/mavros/setpoint_raw/local` |
| **PX4 机体（FRD）** | x=前 / y=右 / z=下 | 姿态、角速率（Phase 2 才用） |

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

**为什么这样设计**：`/mavros/setpoint_raw/local` 的坐标是 **EKF2 的 local NED**，而 EKF2 的局部原点在**它自己初始化时**确定，未必等于我们锁定的动捕原点。两者之间可能有常值偏移。

**→ 所以轨迹原点取"HOVER 稳定时 EKF2 报的位置"**，这样：
- **偏移被自动吸收**——我们是在同一个坐标系里"以当前点为 0"发指令
- **mocap 节点的原点怎么定都不影响 Phase 1**（它只影响喂给 EKF2 的 vision_pose）
- Phase 2 的 MPC 改用 `/mocap/pose_enu`，那时才需要动捕原点

> 📌 **这个设计让 Phase 1 与 Phase 2 的原点问题彻底解耦，是降低 Phase 1 风险的关键一步。**

### 3.4 ★ 姿态：仅归零 yaw

**本项目已定：位置做平移，姿态只归零 yaw，roll/pitch 保持绝对。**

```
解锁上升沿采集 ~0.3 s（约 30–120 帧）平均值：
    p0  = 位置均值
    ψ0  = yaw(q0)                                  ← 只取 yaw
    q_off = quat_from_euler(0, 0, -ψ0)             ← 纯 yaw 旋转

之后每帧：
    p_rel = p - p0                                 （再做 §3.2 的 ENU→NED）
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

### 4.2 转移表

| 从 | 到 | 条件 |
|---|---|---|
| `INIT` | `STANDBY` | `state.connected` ∧ 动捕有数据 ∧ `local_position` 有效 |
| `STANDBY` | `TAKEOFF` | 收到解锁命令 ∧ `state.armed` ∧ `state.mode == "OFFBOARD"` |
| `TAKEOFF` | `HOVER` | `|z − 1.0| < 0.10 m` 且 `|v_z| < 0.1 m/s` 持续 1 s |
| `HOVER` | `WAIT_START` | **稳定判据**成立（见 §5.1）；★ **此刻记录轨迹原点** |
| `WAIT_START` | `TRACK` | **遥控器按键上升沿**（去抖 200 ms） |
| `TRACK` | `LAND` | 轨迹执行完毕（`traj_progress ≥ 1.0`） |
| 任意 | `RC_OVERRIDE` | `COM_RC_OVERRIDE` 触发 / 检测到 `state.mode != "OFFBOARD"` |
| 任意 | `LAND` | 电池低于阈值 / setpoint 未能持续发布 / `EKF2` 位置失效 |
| `RC_OVERRIDE` | `STANDBY` | 遥控器交还控制且飞机已稳定 |

### 4.3 ★ 三个必须实现的细节（"实飞才会想到"的判据，来自 `px4ctrl`）

1. **OFFBOARD 前置流**：切进 OFFBOARD **之前**，setpoint 必须已经以 ≥2 Hz 持续发布 **≥1 s**。本项目用 50 Hz，`STANDBY` 阶段就在发了。**断流 PX4 会自动退出 OFFBOARD**（`COM_OF_LOSS_T = 1.0 s`，见 `z0.params`）。
2. **`RC_OVERRIDE` 必须真的停止发 setpoint**：只是"不更新目标"是不够的——那等于让飞机悬停在原目标，操作员打杆也拉不回来。**要停发**，让 PX4 按 `COM_OBL_RC_ACT` 接管（`z0.params` 里 `COM_OBL_RC_ACT = 0` = 回 Position 模式）。
3. **`msg_timeout`**：`px4ctrl` 的 `ctrl_param_fpv.yaml` 里每个输入都有 0.5 s 超时（`odom` / `rc` / `cmd` / `imu` / `bat`）。**照抄这个思路**：任何关键输入超时 → 进 `LAND`。

## 5. 判据与轨迹

### 5.1 "稳定"判据（已定）

```
位置误差 < 0.10 m  ∧  |v| < 0.10 m/s      持续 2 s
```

- **位置误差**相对**轨迹原点**（§3.3），来源 `/mavros/local_position/pose`
- **速度**来源 `/mavros/local_position/velocity_local`（EKF2 融合，比动捕差分干净）
- **两者都在 EKF2 的 local NED 里**，与 setpoint 同坐标系，**不需要额外转换**

### 5.2 三条轨迹（已定，都在 §3.3 的轨迹原点、NED 下定义）

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

**速度上限的实现**：(b) 方式下把参数 `t` 的推进速率按 `max_speed` 缩放即可；正方形总长 4 m → 8 s；八字周长 ≈ 4.2 m → 8.4 s。

### 5.3 验收标准（已定）

- [ ] 定点**水平误差 ≤ 15 cm**
- [ ] **持续 60 s**
- [ ] **全程不碰摇杆**
- [ ] 能触发自动降落

## 6. 参数（`z0.params` 中需要核对或修改的）

★ **以下值来自现有 `z0.params`（928 行，尚未在 z0 飞控上核实）**，Phase 1 前必须逐条确认：

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

同时：`coordinate_frame = FRAME_LOCAL_NED (1)`，`position.z` **向下为正**，`yaw` 用弧度（推荐 `yaw = 0` = 解锁时的机头方向）。

> 📌 **若定点出现低频抖动**，标准下一步是**加上速度前馈**：轨迹发生器同时给出参考速度，把掩码改成 `2496`（只忽略加速度与 yaw_rate），填入 `velocity`。**这是 (A) 级方案的标准调优路径**，先不做。

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
