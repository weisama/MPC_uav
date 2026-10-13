# XI35 `PX4CtrlFSM` 源码分析（含转换表）

> **本文件是什么**：对 `Fast-Drone-XI35` 飞行状态机的**第一手源码分析**，供 z0 的 FSM 设计引用。
>
> **本文件不是什么**：**不是** Q8 决策否掉的「逐行移植映射表」。本项目已定：**代码溯源只写在代码注释里**
> （`// ported from XI35 PX4CtrlFSM.cpp:183`），不建独立映射文档。这里做的是**上游设计解读**，
> 与 [`code-inventory.md`](code-inventory.md) 同类。
>
> **取证方式**：本文所有结论均来自下表的仓库与 commit，**逐条读过源码**，不依赖记忆或二手总结。

## 0. 出处与取证

| 项 | 值 |
|---|---|
| 仓库 | `Longer95479/Fast-Drone-XI35` |
| 分支 / commit | `board` / **`8c80ddb511c814e42576d0b8f3532e9b776e1224`**（2026-02-02） |
| 本地路径 | `~/refs/Fast-Drone-XI35`（稀疏克隆副本见 `~/refs/xi35-sparse`） |
| 主文件 | `src/realflight_modules/px4ctrl/src/PX4CtrlFSM.{h,cpp}` |
| 接口级别 | **(B) 级姿态 + `use_bodyrate_ctrl` 可选转 (C) 级角速率** |

> ⚠️ **commit 已核对**：本文件的 commit 与 [`code-inventory.md`](code-inventory.md) §A.1 记录的
> `8c80ddb` 一致，且 2026-09-20 通过 `git ls-remote` 复核该仓库**未发生漂移**。

---

## 1. 五个状态

`PX4CtrlFSM.h:63-70`（逐字）：

```cpp
enum State_t {
    MANUAL_CTRL = 1,  // px4ctrl is deactived. FCU is controled by the remote controller only
    AUTO_HOVER,  // px4ctrl is actived, it will keep the drone hover from odom measurments while
                 // waiting for commands from PositionCommand topic.
    CMD_CTRL,    // px4ctrl is actived, and controling the drone.
    AUTO_TAKEOFF,
    AUTO_LAND
};
```

`PX4CtrlFSM.cpp:17-40` 的 ASCII 状态图（逐字）：

```
          system start
                |
                v
    ----- > MANUAL_CTRL <-----------------
    |         ^   |    \                 |
    |         |   |     \                |
    |         |   |      > AUTO_TAKEOFF  |
    |         |   |        /             |
    |         |   |       /              |
    |         |   v     /                |
    |       AUTO_HOVER <                 |
    |         ^   |  \  \                |
    |         |   |   \  \               |
    |         |	  |    > AUTO_LAND -------
    |         |   |
    |         |   v
    -------- CMD_CTRL
```

**设计要点**：`MANUAL_CTRL` 是**唯一的入口和唯一的出口**。任何异常都收敛回它，且回去时
一律 `toggle_offboard_mode(false)`。`CMD_CTRL` 不能直接着陆（见 §3 第 10 行）。

---

## 2. 时间常量

`PX4CtrlFSM.h:22-27`（逐字）：

```cpp
static constexpr double MOTORS_SPEEDUP_TIME   = 3.0;  // motors idle running for 3 seconds before takeoff
static constexpr double TAKEOFF_SPEEDUP_TIME  = 0.1;  // Time to speed up during takeoff font stage
static constexpr double DELAY_TRIGGER_TIME    = 2.0;  // Time to be delayed when reach at target height
```

`takeoff_land.landed` 初值 **`true`**（`h:17`）。这是「着陆检测器」的初始信任状态，配合
§3 第 11 行的 `last_state` 边沿逻辑使用。

**着陆检测判据**（`PX4CtrlFSM.cpp:389-392`，逐字）：

```cpp
constexpr double POSITION_DEVIATION_C = -0.5;  // 目标位置低于真实位置 0.5 m
constexpr double VELOCITY_THR_C       = 0.1;   // 速度低于 0.1 m/s
constexpr double TIME_KEEP_C          = 3.0;   // 上述两条须保持 3 s
```

---

## 3. ★ 转换表（本文核心）

**全部转换、守卫与副作用，逐条从 `PX4CtrlFSM::process()`（`cpp:42-366`）读出。**

| # | 转换 | 守卫（全部须真） | 副作用 | 拒绝/失败模式 |
|---|---|---|---|---|
| 1 | `MANUAL_CTRL → AUTO_HOVER` | `rc_data.enter_hover_mode` ∧ odom 新鲜 ∧ **未在发 cmd** ∧ `‖v‖ ≤ 3.0` | `resetThrustMapping(bat)`；`set_hov_with_odom()`；`toggle_offboard_mode(true)` | 3 种，各有独立 ERROR：无 odom / 已在发指令（"which is not allowed"）/ `‖v‖>3`（判定位失效） |
| 2 | `MANUAL_CTRL → AUTO_TAKEOFF` | `takeoff_land.enable` ∧ `triggered` ∧ `cmd==TAKEOFF` ∧ odom 新鲜 ∧ 未在发 cmd ∧ `‖v‖ ≤ 0.1` ∧ `landed` ∧（若收到 RC：`is_hover_mode` ∧ `is_command_mode` ∧ `check_centered()`） | 阻塞等 bat；`resetThrustMapping`；`set_start_pose_for_takeoff_land()`；`toggle_offboard_mode(true)`；**空转 0.1 s 等 FMU 切模式**；`enable_auto_arm` 则 `toggle_arm_disarm(true)`；记 `toggle_takeoff_land_time` | 5 种拒绝；其中 RC 未居中时**进入阻塞 `while` 循环**，直到飞行员把开关/摇杆摆好（`cpp:116-124`） |
| 3 | `MANUAL_CTRL → MANUAL_CTRL`（reboot） | `rc_data.toggle_reboot` ∧ **未解锁** | `reboot_FCU()`（MAV_CMD 246, param1=1） | 已解锁则 ERROR 拒绝："Disarm the drone first!" |
| 4 | `AUTO_HOVER → MANUAL_CTRL` | ¬`is_hover_mode` ∨ odom 超时 ∨ `kf_fusion_fail` | **`toggle_offboard_mode(false)`** | — |
| 5 | `AUTO_HOVER → CMD_CTRL` | `is_command_mode` ∧ cmd 新鲜 ∧ ¬`emergency_hover` ∧ ¬`search_hover` ∧ **`current_state.mode == "OFFBOARD"`** | — | 若 PX4 不在 OFFBOARD，**静默不跳转**（无日志） |
| 6 | `AUTO_HOVER → AUTO_LAND` | (`triggered` ∧ `cmd==LAND`) ∨ `emergency_hover` | `set_start_pose_for_takeoff_land()` | 见 §4：`emergency_hover` **是死代码**，实际只能由 LAND 指令触发 |
| 7 | `AUTO_HOVER → AUTO_HOVER` | 兜底分支 | `set_hov_with_rc()`（**摇杆直接搬动悬停点**）；`get_hover_des()`；若 `enter_command_mode` 或 `delay_trigger` 到期 → `publish_trigger()` | — |
| 8 | `CMD_CTRL → MANUAL_CTRL` | ¬`is_hover_mode` ∨ odom 超时 | `toggle_offboard_mode(false)` | — |
| 9 | `CMD_CTRL → AUTO_HOVER` | ¬`is_command_mode` ∨ cmd 超时 ∨ `emergency_hover` ∨ `search_hover` | `set_hov_with_odom()` | — |
| 10 | `CMD_CTRL → CMD_CTRL`（**拒绝着陆**） | `triggered` ∧ `cmd==LAND` | — | **ERROR**：要求先停发指令 > `msg_timeout.cmd` 秒，退回 `AUTO_HOVER` 再着陆 |
| 11 | `AUTO_TAKEOFF → AUTO_HOVER` | `t ≥ MOTORS_SPEEDUP_TIME` ∧ `odom.p(2) ≥ start_pose(2) + height` | `set_hov_with_odom()`；`delay_trigger = now + DELAY_TRIGGER_TIME`（2 s 后才放行指令） | — |
| 12 | `AUTO_TAKEOFF → AUTO_TAKEOFF`（前 3 s） | `t < MOTORS_SPEEDUP_TIME` | `get_rotor_speed_up_des()`：`des_a_z = exp((Δt−3)·6)·7 − 7`，**若 > 0.1 则清零并 ERROR** | — |
| 13 | `AUTO_TAKEOFF → AUTO_TAKEOFF`（之后） | 未达目标高度 | `get_takeoff_land_des(+speed)`：位置斜坡 + 前 0.1 s 正弦加速度 | — |
| 14 | `AUTO_LAND → MANUAL_CTRL` | ¬`is_hover_mode` ∨ odom 超时 | `toggle_offboard_mode(false)` | — |
| 15 | `AUTO_LAND → AUTO_HOVER` | ¬`is_command_mode` | `set_hov_with_odom()` | — |
| 16 | `AUTO_LAND → MANUAL_CTRL`（正常收尾） | `landed` ∧ `extended_state == LANDED_STATE_ON_GROUND` ∧ `toggle_arm_disarm(false)` **成功** | `toggle_offboard_mode(false)` | **限频 1 Hz**（`last_trial_time`），避免频繁调用 |
| 17 | `AUTO_LAND → AUTO_LAND` | ¬`landed` → `get_takeoff_land_des(−speed)`；否则 `motors_idling`（`thrust = 0.04`） | — | — |

**计数说明**：状态**对**转换共 **11 条**（去重后的不同状态对为 **8 对**；`AUTO_LAND→MANUAL_CTRL`
有两条守卫不同的路径，见第 14、16 行）。另有 6 条自环行为（第 3、7、10、12、13、17 行）。
⚠️ 早期文档曾称「14 条转换」，**该数字与本轮逐条枚举对不上**；以本表为准。

### 3.1 每条转换之后统一执行的四步（`cpp:318-365`）

1. **IMU 加速度低通**：仅在 `AUTO_TAKEOFF / AUTO_HOVER / CMD_CTRL` 三态下做（`LPF_imu_a`）
2. **推力模型在线估计**：`AUTO_TAKEOFF` 下 `Δt > 0.2` 时估计；`AUTO_HOVER / CMD_CTRL` 下持续估计
3. **解算控制量**：正常走 `controller.calculateControl(...)`；着陆起始阶段改走 `motors_idling`
4. **下发给 mavros**：`use_bodyrate_ctrl ? publish_bodyrate_ctrl : publish_attitude_ctrl`

**第 4 步是 z0 的关键分野**：

| 分支 | mavros 消息 | `type_mask` | 对应控制级别 |
|---|---|---|---|
| `publish_attitude_ctrl` | `AttitudeTarget` | `IGNORE_ROLL_RATE \| IGNORE_PITCH_RATE \| IGNORE_YAW_RATE`（给四元数） | **(B)** |
| `publish_bodyrate_ctrl` | `AttitudeTarget` | `IGNORE_ATTITUDE`（给 `body_rate`） | **(C)** |

**两者都用同一个 `AttitudeTarget` 消息，只靠 `type_mask` 区分** —— 这是 mavros 侧的 (B)/(C) 切换方式。
z0 的 Phase 2/3 走 uXRCE-DDS（`VehicleRatesSetpoint`），**不复用这条路径**，但**`type_mask` 这个
「同一消息、掩码切换语义」的设计值得借鉴**。

> ☠️ **2026-10-13（票 13 收口轮）：上面这两句今天整个反过来。** z0 的 Phase 2 **不再走 uXRCE-DDS**，
> 它的 (C) 级下行**就是走表里这一条**：`AttitudeTarget` + `type_mask = 128`（`IGNORE_ATTITUDE`）= 只给 `body_rate`，
> 也就是 `publish_bodyrate_ctrl` 那一行（[`13`](../../.scratch/phase4/issues/13-mavros-downlink-switch.md)、
> [`px4-interface.md`](px4-interface.md) §4.3）。⟹ 不是"借鉴这个设计"，而是**就是它**。
> ⚠️ 本页 §3 那张表本身（消息、掩码、哪一级）**逐项仍然正确**，作废的只是"z0 不复用它"这句。

---

## 4. ★ 三处死代码 / 陷阱（第一手确认）

| 位置 | 事实 | 后果 |
|---|---|---|
| `h:87-91` | `emergency_callback` 里 **赋值语句被注释掉**：`// if(msg->data == true) // emergency_hover = true;` | `emergency_hover` **永远为 `false`**。§3 第 5、6、9 行中所有 `emergency_hover` 条件**恒不成立** → 「紧急悬停」功能**完全不存在**。第 6 行的 `AUTO_HOVER→AUTO_LAND` 实际**只能由 LAND 指令触发** |
| `h:96-99` | `kf_fail_callback` 只在 `true` 时置位，**从不复位** | `kf_fusion_fail` 是**单向 latch**。一旦置位，`MANUAL_CTRL` 分支直接 `break`（`cpp:51-54`）且 `AUTO_HOVER→MANUAL_CTRL` 恒成立 → **只能手动着陆**（源码原文："please take land manually!"） |
| `h:93-94` | `search_hover` 可被外部置位，**但没有任何地方清它** | 一旦置位，`CMD_CTRL` 永远进不去（第 5 行守卫含 ¬`search_hover`），且会**把已在 `CMD_CTRL` 的状态踢回 `AUTO_HOVER`**（第 9 行）。是个只能重启才能退出的状态 |

**z0 的设计含义**：这三处**都不应照搬**。z0 需要一个**可判定、可复位**的失效降级路径，
而不是一个单向 latch。

---

## 5. ★ 与 Q5 决策的对照（打杆接管）

z0 已定 **Q5=(a)**：RC 打杆 = **立即停止发布 setpoint**，让 PX4 按 `COM_OBL_RC_ACT` 自行接管。

**XI35 的做法与这不同**：它的「接管」不是停发，而是**把摇杆位移积分进悬停点**（第 7 行，
`set_hov_with_rc()`，`cpp:496-523`）：

```cpp
hover_pose(0) += rc_data.ch[1] * param.max_manual_vel * delta_t * (param.rc_reverse.pitch ? -1 : 1);
hover_pose(1) += rc_data.ch[0] * param.max_manual_vel * delta_t * (param.rc_reverse.roll  ? -1 : 1);
hover_pose(2) += rc_data.ch[2] * param.max_manual_vel * delta_t * (param.rc_reverse.throttle ? -1 : 1);
hover_pose(3) += rc_data.ch[3] * param.max_manual_vel * delta_t * (param.rc_reverse.yaw  ? -1 : 1);
if (hover_pose(2) < -0.3) hover_pose(2) = -0.3;   // ★ 高度下限硬编码
```

**三个可直接借鉴的点**：

1. **`ch[]` 到轴的映射**：`ch[1]→x`、`ch[0]→y`、`ch[2]→z`、`ch[3]→yaw`。即 **`ch[0]` 是横滚（映射到 y）**，
   `ch[1]` 是俯仰（映射到 x）。照抄前**必须核对 `RC_MAP_ROLL/PITCH`**——z0 的参数是
   `RC_MAP_ROLL=1, RC_MAP_PITCH=2, RC_MAP_YAW=4, RC_MAP_THROTTLE=3`，通道序与之不同。
2. **`rc_reverse` 逐轴可配**，且**用在乘子位置而非分支位置**（`* (cond ? -1 : 1)`），比 `if` 分支紧凑。
3. **高度下限 `-0.3` 硬编码**，是防止摇杆把悬停点压到地下的护栏。**z0 若沿用此设计，这个值要重定**
   （152 mm 机架，0.3 m 的余量语义与 XI35 不同）。

> ⚠️ **注意 `ctrl_param_fpv.yaml` 不可复制**（见 [`code-inventory.md`](code-inventory.md)）：
> 该文件 `rc_reverse` 键**重复出现**，是 XI35 引入的缺陷（Fast-Drone-250 只有一个）；
> 且其质量参数是 645 g 机，与 z0 的 ~750 g 不符。

---

## 6. 参考实现里**没有**的东西（即 z0 必须自己设计的部分）

| 缺失 | 说明 |
|---|---|
| **任何位置/速度接口** | 全部走 (B)/(C) 级。**(A) 级 offboard（Phase 1）在此无参考价值** |
| **坐标变换** | 它假设 `odom` 已经是它要的约定，全程无 ENU/NED 转换代码。z0 的 ENU↔NED 是**全新工作** |
| **失效降级** | §4 的三处死代码说明其降级路径**未完成**。z0 需重新设计 |
| **延迟补偿** | 无。XI35 的 `kf_fusion` ESKF 带延迟补偿，但那是**定位侧**的，不是**控制指令侧**的。z0 的 Phase 2/3 需要后者（Q6=(a) 的状态外推） |
| **硬实时保证** | ROS 1 单线程 + `ros::Duration::sleep`，**无实时性保证**。z0 需评估 |

---

## 7. 相关文档

- [`code-inventory.md`](code-inventory.md) §A.1 —— XI35 逐文件取舍判定
- [`../control/phase1-implementation.md`](../control/phase1-implementation.md) §4 —— z0 自己的 FSM 草案
- [`params-analysis.md`](params-analysis.md) —— `z0.params` 逐项分析
- [`px4-interface.md`](px4-interface.md) —— mavros / uXRCE-DDS 接口事实
