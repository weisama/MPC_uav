# 15: 让菜单 9 的模型预测控制也能用开始键飞轨迹 1/2/3/4 —— 两种控制平替、共用 `control`/`traj`/开始键

Status: ready-for-agent
Type: task

## What to build（2026-10-17 操作者原话）

> `本票任务：让z0.py选9的mpc控制也可以用轨迹1234飞行，并且也是通过开始按键解锁起飞，开始按键开始轨迹。统一俩种控制的流程，让直接发目标点和mpc控制可以相互替代，其他control、traj、开始按键等都是俩种控制共用的`

> `并且后续还可能会有更多控制方式的加入，不同的控制方式相互平替，共用control、traj等共有流程`

**做法：把 `control` 变成唯一的任务总管，两种控制只是它的「输出后端」。**

```
                  ┌─ 后端 1（默认，position）：control 自己发 /mavros/setpoint_raw/local
traj ─参考─▶ control（FSM + 开始键 + 解锁 + 切模式 + 起飞 + 轨迹原点 + /traj/start + 降落）
                  └─ 后端 2（external）：control 发 /control/target（绝对目标）
                                         └─▶ mpc_node 求解并发 /mavros/setpoint_raw/attitude
```

- 菜单 9 第 1 档：`traj → control`（`output_mode=position`，默认，行为不变）。
- 菜单 9 第 2 档：`traj → mpc → control`（`control output_mode=external`；
  `mpc` 用 `reference_source=control` + `offboard_prestream=true`，**不再** `start_traj_reference`）。
- **开始键、解锁、切 OFFBOARD、起飞斜坡、判稳、轨迹原点、`/traj/start`、完成后 `AUTO.LAND`
  全部留在 `control`**，两档逐字相同；轨迹 1/2/3/4 两档都能飞。
- **同一时刻只有一条 PX4 setpoint 话题在被写**：`position` 档 `control` 写 local；
  `external` 档 `control` **不建 local 的发布者**，只写 `/control/target`，`mpc` 写 attitude。
- **失效**：`/control/target` 断流 0.5 s ⟹ `mpc` 以 `no_control_target` 关闸（**零帧**，
  不是冻结、也不是 `hold → hover` 阶梯接管 —— 那条阶梯只管求解不可用）；
  PX4 的 `COM_OF_LOSS_T = 1 s` 仍是最后一道。
- **可扩展**：将来的新控制方式只需消费 `/control/target`、自己写执行器话题；
  `control` / `traj` / 开始键 / 解锁 / 降落**都不用动**，`output_mode` 只加一个枚举值。

**为什么单开 15 号票**：票 [14](14-chain-controller-choice.md) 的 Comments 里记着一条更早的裁定
「将对话工作并入票 14 —— 不新开 15 号票」。那是**上一轮那件事**（菜单 8/9 与两条新参数）的落票方式；
本轮操作者**点名 15 号票**、问的是另一件事（两条控制平替）。两者不是同一条命令，
所以本票单开，**不改票 14 的任何一行**（票 14 的历史验收记录原样保留）。

**Blocked by:** 无。

## 范围 —— 动了哪几处

| 文件 | 改了什么 |
|---|---|
| `src/nmpc/nmpc/node_core.py` | 新增参考源 `control`：订阅 `/control/target` 的绝对目标、**不加锚点**、按目标速度外推；断流 0.5 s ⟹ `no_control_target` 关闸（**不是冻结**） |
| `src/nmpc/nmpc/mpc_node.py` | `reference_source=control`、`control_target_timeout_s`、条件订阅；`_validate` 要求 `hold_offset=0`、拒绝 `start_traj_reference` |
| `src/control/include/control/fsm.hpp` | `TrajReference` / `Decision` 增加速度三元组；`TRACK` 里原样转发参考速度，断流时归零 |
| `src/control/include/control/px4_interface.hpp` / `src/px4_interface.cpp` | `OutputMode{position, external}`、`/control/target` 发布者、掩码 2496、不认识的档启动即拒（退出码 2）；`publish_target()` 把"按后端分派"收在接口里（构造时只建一条发布者，调用方不判断后端） |
| `src/control/src/control_node.cpp` | 读 `output_mode`；每拍只调 `px4_->publish_target(...)`，不再自己 switch |
| `src/control/config/control.yaml` / `launch/control.launch.py` | `output_mode`（默认 `position`）+ `control_target_topic`；launch 参数可覆盖 |
| `run/z0.py` | 第 2 档尾巴改成 `traj → mpc → control`，带三条 optional；收尾话改写；模块 docstring 同步 |
| 测试 | `src/control/test/test_fsm.cpp`（速度转发/断流归零）、`src/control/test/test_control_external_output.py`（external 黑盒 seam 1）、`src/nmpc/test/test_mpc_node.py`（control 参考源三条）、`run/tests/test_z0_menu.py`（两档尾巴/参数/收尾） |

**仍然不新增任何会动飞机的入口**：`z0.py` 里照旧不许出现 `ros2 service call` / `topic pub`；
`mpc_node` 仍然不 arm、不 set_mode；解锁与切模式只由 `control` 的 FSM 发起。

---

## 调研（本票的前半程，2026-10-17）—— 下面每条结论都带 `文件:行号`

---

> ☠️☠️ **§0–§8 是「动手之前」的快照**（2026-10-17 调研轮）。其中凡说「模型预测控制链上没有
> `control`」「开始键在那条链上没有读者」「`mpc_node` 自己发 `/traj/start`」「解锁/切模式要手打」
> 的句子，**都是改造前的状态**，已被本轮施工取代 —— **现行事实看下面「实现后的现行事实」一节**。
> 保留原文是因为它解释了**为什么**要这样改（每条结论都有 `文件:行号`）。
> ⚠️ §1.4 / §8 里「飞行中重选轨迹会冻住」那条**未在本轮修**，仍然成立（见「本轮没做」）。

## 一句话回答（改造前）

- **`traj`** 只产生**相对轨迹原点的参考**。它不发 setpoint、不解锁、不切模式、不进 OFFBOARD。
  「选轨迹」只是准备并画折线；**`/traj/start` 才是参考时钟的零点**（`src/traj/src/traj_node.cpp:242-299`、`:305-340`）。
- **offboard 控制（`control`，(A) 级）** 是 Phase 1 那条链的**唯一驾驶员**：它读参考、**加上轨迹原点**、
  发 `/mavros/setpoint_raw/local`（位置 + yaw），并**自己**切 OFFBOARD、**自己**解锁、
  **自己**发 `/traj/start`、任务完成后请求 `AUTO.LAND`。
- **模型预测控制（`nmpc/mpc_node.py`，(C) 级）** 是另一条链的**唯一驾驶员**：它读同一条
  `/traj/reference`、**加上自己的锚点**、发 `/mavros/setpoint_raw/attitude`（角速率 + 归一化推力）。
  它**不解锁、不切模式、不读遥控器**——这三件事在这一档全归操作者。
- **开始按键（遥控器 ch9）只属于 `control`**。它在 (A) 级链里负责「第一次上升沿 = 解锁起飞、
  第二次松手 = 进 TRACK、在飞时长按 ≥1 s = 降落」；**模型预测控制那条链上没有 `control`，
  所以开始按键在那条链上完全不起作用**（`src/control/include/control/fsm.hpp:1036-1078`；
  `operation_manual.md:293-300`）。
- **两条链的「轨迹原点」不是同一个东西**：(A) 级是 `control` 在**悬停判稳那一刻**记的 EKF 位置；
  模型预测控制是 `mpc_node` 在**闸门第一次打开那一拍**钉的测量位置。两者语义都是「以此点为 0」，
  但**记录时机、归属、失效后是否重钉**都不同（§5）。

---

## 0. 角色分工（先看这张表）

| 组件 | 它是什么 | 它**发**什么 | 它**读**什么 | 它**不**做什么 |
|---|---|---|---|---|
| `traj`（轨迹功能包） | 参考发生器 | `/traj/reference`（`PositionTarget`，50 Hz，**相对轨迹原点**）、`/traj/progress`、`/traj/marker` | `/traj/select`、`/traj/start` | 不发 setpoint、不解锁、不切模式、不进 OFFBOARD；**跑完不降落** |
| `control`（(A) 级 offboard 控制） | 位置控制器 + 任务状态机 | `/mavros/setpoint_raw/local`（位置 + yaw）、`/traj/start`、`/mavros/set_mode`、`/mavros/cmd/arming`、`/control/state`、`/control/traj_progress` | `/traj/reference`、`/traj/progress`、`/mavros/state`、位姿/速度、`/mocap/pose_enu`、`/mavros/rc/in`、电池、EKF 状态 | 不产生轨迹几何；**不转发参考 yaw** |
| `mpc_node`（模型预测控制） | 非线性模型预测控制器 | `/mavros/setpoint_raw/attitude`（`body_rate` + `thrust`）、`/traj/start`（仅 `start_traj_reference=true` 时）、`/mpc/predicted_path`、`/mpc/reference` | `/traj/reference`、`/mavros/state`、`/mocap/pose_enu`（实机默认 `state_source=mocap`）、`/mocap/velocity_enu` | **不 arm、不 set_mode、不读 `/mavros/rc/in`**；不解锁、不切模式 |
| 开始按键（ch9） | `control` 的一个输入（不是 PX4 的通道） | —— | `/mavros/rc/in` 的 `channels[8]` | 在模型预测控制链上**没有读者**，因此不起作用 |
| 操作者 | 决定「哪条链、哪条轨迹、什么时候开始」 | 菜单 9 的控制器选择、`fly.py` 的 `/traj/select`、ch6/ch9、（模型预测控制档）手打解锁 + OFFBOARD | —— | —— |

★ 菜单 **9** 的两档**都必起 `traj`**：`offboard` 档的尾巴是 `("traj", "control")`，
模型预测控制档的尾巴是 `("traj", "mpc")`（`run/z0.py:1028-1049`）。「轨迹必须起没得选」是结构事实，
不是提示语。

---

## 1. `traj` —— 只产生「相对参考」

### 1.1「选」与「开始」是两条消息，别合成一条

| 消息 | 语义 | 证据 |
|---|---|---|
| `/traj/select`（`UInt8`：1 悬停 / 2 正方形 / 3 八字 / 4 起飞 / 0 清空） | **只准备轨迹**：构造对象、发折线、把 `running_` 置回 `false` | `traj_node.cpp:242-267`（`:251-252` 清运行态） |
| `/traj/start`（`Empty`） | **参考时钟零点**：`running_ = true; start_time_ = now()` | `traj_node.cpp:284-299`（`:292-293`） |

`tick()` 里有两道早退，顺序是刻意的：

```cpp
if (!trajectory_) return;          // 没选 ⟹ 一帧都不发
maybe_republish_path(stamp);       // 折线重发只跟"选没选"有关
if (!running_) return;             // 选了但没 /traj/start ⟹ 参考/完成度都不发
```

（`src/traj/src/traj_node.cpp:305-322`）

⟹ **选中轨迹不会让飞机动**；参考要等 `/traj/start`。这也解释了为什么「选轨迹」可以离线做
（只看 RViz 形状，不需要动捕/飞控/起飞，`traj_node.cpp:312-318`）。

### 1.2 参考是**相对轨迹原点**的

`publish_reference()` 发 `FRAME_LOCAL_NED` + `type_mask = 2496`（位置、速度、yaw 有效；
忽略加速度与 yaw_rate，`traj_node.cpp:342-348`、`:53`）。

**它不知道、也不加轨迹原点在哪**。加原点是控制侧的事：

- (A) 级：`d.target_x = traj_x_ + traj.x`（`fsm.hpp:1475-1477`）
- 模型预测控制：`锚点 + /traj/reference 的相对位置`（`src/nmpc/nmpc/node_core.py:631-668`）

### 1.3 跑完之后

`progress ≥ 1.0` 之后，参考**停在终点、完成度保持 1.0**，`traj` **不主动降落**
（`traj_node.cpp:333-339`）。收尾是控制侧看 `progress ≥ 1.0` 做的：
(A) 级请求 `AUTO.LAND`，模型预测控制只把参考冻在终点（见 §4）。

### 1.4 运行中重选

`select_kind()` 会把 `running_` 清掉、时钟归零（`traj_node.cpp:249-253`）⟹ 参考流**断掉**。
之后谁来「重新开始」两条链**不一样**：

- (A) 级：`control` 发现参考断流，在 `traj_stream_gap_s = 0.5 s` 后**重发** `/traj/start`
  （`fsm.hpp:1492-1502`）。
- 模型预测控制：**没有重发机制**。`traj_pos` 一旦收到过就再也不会变回 `None`
  （`node_core.py:897-912`、`:1134`），所以闸门不会退回 `no_traj_reference`，
  `_maybe_start_traj()` 也就不会被触发；参考超时只会走 `traj_frozen()` **就地冻结**
  （`node_core.py:1001-1010`、`:1136-1140`）。⟹ 在模型预测控制链上**飞行中重选轨迹，
  飞机会停在最后一个参考上**，直到重新起链或有人在别处发一帧 `/traj/start`。
  ⚠️ 这一条是**读代码得出的**，本次没有实飞复现（记在 §8）。

---

## 2. 开始按键（ch9）—— 只属于 `control`

### 2.1 它是什么

- 参数：`start_channel = 9`，读 `/mavros/rc/in` 的 `channels[8]`（1-based 通道号）；
  归一化阈值 `start_threshold = 0.75` / `release_threshold = 0.25`；去抖 `debounce_ms = 200`
  （`src/control/config/control.yaml:98-104`）。
- 它是**我们自己的通道**，不映射 `RC_MAP_*` ⟹ 拨它不会切模式、不会解锁、不会杀车
  （`operation_manual.md:62-73`）。
- 只有 `/mavros/rc/in` 的读者是 `control`：`Px4Interface::on_rc()` 同时喂四个摇杆变化率检测器
  和 `StartSwitchDetector`（`src/control/src/px4_interface.cpp` 的 `on_rc`）。
  **`mpc_node` 没有任何 RC 订阅**（`mpc_node.py` 的 `_PARAMS` / 订阅表里没有 `/mavros/rc/in`）。

### 2.2 三次动作的精确语义（按**任务态**判，不按「第几次」数）

`Fsm::note_start_edge()`（`fsm.hpp:1036-1078`）：

| 当拍任务态 | 按下（上升沿） | 松手 | 长按 ≥ `start_land_hold_s = 1.0 s` |
|---|---|---|---|
| `kIdle` | **受理起飞**（`request_takeoff()`） | 无 | 无（那一刻还没有降落资格） |
| `kWaitStart` | 只记 `track_on_release_ = true`，**不进 TRACK** | **进 TRACK**（`start_tracking()`） | 长按接管 ⟹ 降落 |
| `kClimb` / `kHover` / `kTrack` | 只给「长按降落资格」`land_hold_armed_ = true` | 无 | **请求降落**（`AUTO.LAND`） |
| `kRequested` / `kDone` | **丢掉**，连降落资格都不给 | 无 | 无 |

★ 第 2 次按下的关键在于**松手**才进 TRACK（`fsm.hpp:1051-1059`、`:1080-1093`）：
按下只是「记下」，手不松就不开始飞。

### 2.3 为什么模型预测控制链上它不起作用

开始键的实现**整段都在 `control` 里**：`control_node.cpp:380-392` 取边沿并投给 `Fsm`。
模型预测控制档的链是 `mavros → vrpn → mocap → traj → mpc`（`run/z0.py:1040-1049`），
**没有 `control`** ⟹ **没有人读 ch9**。操作手册把这一点写在明面上
（`operation_manual.md:293-300`：「遥控器开始键在这条链上不起作用」）。

⟹ 在那条链上，「什么时候开始飞轨迹」由 **`/traj/start`** 决定，而 `/traj/start` 的发布者是：

1. `control`（(A) 级，进 TRACK 那一刻）—— 这条链没有；
2. `mpc_node`（仅当 `start_traj_reference = true`，`z0.py` 的模型预测控制档会打开它）；
3. 人（手打 `ros2 topic pub`）—— 手册明确不建议，因为会把正在跑的参考时钟归零。

---

## 3. (A) 级 offboard 控制链：一条轨迹从选中到落地

**链**：`mavros → vrpn → mocap → traj → control`（`run/z0.py:1028-1039`）。

`control_node.cpp` 的 `tick()` 每 20 ms（`:12` `setpoint_rate_hz: 50`）做这一串
（`control_node.cpp:370-426`）：

1. 取走开始键边沿 → `fsm_->note_start_edge()`（`:380-383`）；
2. 取长按 / 松手事件（`:387-392`）；
3. 取守卫、健康、里程计、轨迹参考 → `fsm_->update()`（`:394-398`）；
4. 若 `d.stream_setpoint`：发 `/mavros/setpoint_raw/local`（`:403-407`）；
5. 若 `d.request_traj_start`：发 `/traj/start`（`:415-419`）；
6. 若 `d.request_arming`：程序解锁一次（`:421-423`）；
7. 若 `d.request_offboard`：请求 `OFFBOARD`（`:424-426`）；
8. 若 `d.request_land`：请求 `AUTO.LAND`（`:427+`）。

### 3.1 时序（每一步的触发者与判据）

| # | 谁触发 | 发生什么 | 判据 / 证据 |
|---|---|---|---|
| 1 | 人（`fly.py` 敲 1/2/3/4） | `/traj/select` → `traj` 记下轨迹、画折线 | `fly.py:107-122`；`traj_node.cpp:242-267` |
| 2 | 链起来后 | `control` 在 `STANDBY` **持续发 setpoint**（位置 = 当前位姿），流够 `offboard_prestream_s = 1.0 s` 后**请求 OFFBOARD** | `fsm.hpp:1323-1324`；`control.yaml:17` |
| 3 | 人（ch6 拨到第 6 档） | PX4 进入 `OFFBOARD`；`control` 看到 `in_offboard` 才允许解锁 | `control.yaml` / `operation_manual.md:71`；`fsm.hpp:1354-1366` |
| 4 | 人（ch9 第 1 次上升沿） | `kIdle → kRequested`，**受理起飞** | `fsm.hpp:1040-1049`、`:1017-1034` |
| 5 | 前置齐（已在 OFFBOARD + setpoint 在流 + RC 新鲜） | `control` 发 `/mavros/cmd/arming`，**只发一次** | `fsm.hpp:1354-1366` |
| 6 | `armed` 上升沿 | 锁 `origin_*` 与 `takeoff_yaw_`，进 `kClimb`：水平钉在原点，z 按 `climb_speed_mps = 0.5` 斜坡升到 `takeoff_altitude_m = 1.0` | `fsm.hpp:1342-1352`、`:1374-1405` |
| 7 | 到高并判稳 1 s | 进 `kHover`；悬停点 = 原点正上方 1.0 m | `fsm.hpp:1392-1405`、`:1408-1414` |
| 8 | 位置误差 < 0.10 m ∧ 速度 < 0.10 m/s 持续 2 s | **记下轨迹原点 = 当前 EKF 位置**，进 `kWaitStart` | `fsm.hpp:1416-1437` |
| 9 | 人（ch9 第 2 次，**松手**） | `start_tracking()`：进 `kTrack`，且 `traj_start_sent_ = false` ⟹ 下一帧必发 `/traj/start` | `fsm.hpp:1080-1093`、`:866-870` |
| 10 | `control` 自己 | 发 `/traj/start`（进 TRACK 的第一帧） | `fsm.hpp:1492-1502`；`control_node.cpp:415-419` |
| 11 | 每帧 | 目标 = **轨迹原点 + `/traj/reference`**；参参考 yaw **不转发**，机头保持起飞机头 | `fsm.hpp:1470-1490`、`:1464-1468` |
| 12 | `progress ≥ 1.0`（且必须先见过 < 1.0） | `end_reason = traj_done` → 请求 `AUTO.LAND` | `fsm.hpp:1504-1514` |
| 13 | 参考断流 > `traj_timeout_s = 3.0 s` | `degrade = traj_timeout` → 停发 + 请求 `AUTO.LAND` | `control.yaml:230`；`fsm.hpp` 的 health 判据 |

### 3.2 它发出去的帧

`/mavros/setpoint_raw/local`（`mavros_msgs/PositionTarget`），位置 + yaw，
`type_mask = 2552`（忽略速度、加速度、yaw_rate）。坐标是 ENU，mavros 负责 ENU→NED
（`src/control/src/px4_interface.cpp:522`、`publish_setpoint`）。

★ **yaw 不转发**：`traj` 的 `yaw_rad` 恒为 0，在 ENU 下 0 = 机头朝东；若照抄，飞机在空中会转 90°。
所以 `d.target_yaw = takeoff_yaw_`（`fsm.hpp:1466-1468`；`traj.yaml:96-100`）。

### 3.3 失效时

任何一条关键输入坏掉 ⟹ **立即停发 setpoint**；已解锁则请求 `AUTO.LAND`。
一停发，PX4 在 `COM_OF_LOSS_T = 1.0 s` 后自己退出 OFFBOARD
（`operational_manual.md:196-202`；`safety-paths-drill.md:126-131`）。
这就是「`traj` 只发参考、真正的驾驶员是 `control`」的反面证明：没有 `control` 在发帧，
参考再正常也飞不起来。

---

## 4. (C) 级模型预测控制链：同一条轨迹怎么飞

**链**：`mavros → vrpn → mocap → traj → mpc`（`run/z0.py:1040-1049`）。

`mpc_node` 以 `rate_hz = 100` 跑 `_tick()`（`mpc_node.py:1619-1620`、`:924-1036`）：

```
cycle = core.cycle(now_s)                 # 闸门 + 测量 + 参考 + 锚点
if start_traj_reference: _maybe_start_traj(cycle)   # 闸门判断之前
if cycle.x0 is None: _gate_closed(...); return      # 不求解、不（正常）发
solve OCP → _normal_frame(sol) → 兜底阶梯 → _publish_rates()
```

### 4.1 闸门（先决条件，顺序不能乱）

`NodeCore.cycle()`（`node_core.py:1078-1102`）按这个顺序关闸，任一不成立 ⟹ 当拍 **不求解、不发**：

1. `no_status`：还没收到 `/mavros/state`；
2. `not_armed`：`armed == false`（`stop_on_armed_drop = true`）；
3. `not_offboard`：`mode != "OFFBOARD"`（`stop_on_nav_state_exit = true`）；
4. `status_stale`：`/mavros/state` 龄 > `status_timeout_s = 2.5 s`（1 Hz 话题的余量）；
5. 测量坏：`no_mocap` / `mocap_stale` / `bad_mocap_quat` / `non_finite_mocap` …；
6. 输入合理性：`stamp_regress` / `pose_jump` / `attitude_jump`；
7. （仅轨迹档）`no_traj_reference`：**从来没收到过** `/traj/reference`。

★ 顺序本身有语义：`no_traj_reference` 排在第 1–6 之后 ⟹ 它成立**蕴含**「已解锁 + 在 OFFBOARD
+ 状态新鲜 + 测量可信」。`start_traj_reference` 正是靠这一点把 `/traj/start` 的发帧窗口
收窄到「操作者已把飞机带进 OFFBOARD、正等参考」那一拍（`mpc_node.py:1092-1098`）。

### 4.2 两个新参数（票 14 加的，默认都关）

| 参数 | 打开后 | 默认 | 谁打开 |
|---|---|---|---|
| `offboard_prestream` | 闸门因 **`not_armed` / `not_offboard`** 关着时，发**一帧纯悬停**（零角速率 + 悬停推力）；其余关闸原因照旧零帧 | `false`（`mpc_node.py:1633`） | `z0.py` 的模型预测控制档 |
| `start_traj_reference` | 闸门因 **`no_traj_reference`** 关着那一拍，发**一次** `/traj/start`；离开 armed/OFFBOARD 才重新武装 | `false`（`mpc_node.py:1634`） | 同上 |

- `offboard_prestream` 的权限边界（第一手，PX4 1.14.3 `mavlink_receiver.cpp:1530-1536` /
  `:1562` / `:1592`）：不在 OFFBOARD 时，飞控**只**消费 `offboard_control_mode`，
  姿态/角速率帧根本不到控制器 ⟹ 预流**动不了飞机**，唯一作用是让飞控肯收我们进 OFFBOARD
  （`src/nmpc/README.md:94-104`）。
- `start_traj_reference` 的触发条件只有 `no_traj_reference` 一个，而且用闩锁住「只发一次」
  —— 这个原因会连续几百拍为真，每拍都发 = 每 10 ms 把参考时钟归零一次，飞机永远不动
  （`mpc_node.py:1084-1118`）。

### 4.3 目标怎么算：锚点 + 相对参考

`cycle()` 在测量与输入合理性都通过之后才 `_update_anchor(x_meas, now_s)`（`node_core.py:1145`），
然后：

```python
x_ref = traj_reference(anchor_state(), ..., self.traj_pos, self.traj_vel, self.traj_yaw,
                       freeze=ref_frozen)
```

- **锚点**：闸门**第一次打开的那一拍**的测量位置 + 当时的 yaw（`_update_anchor`，
  `node_core.py:1031-1057`）。闸门再次打开**不重钉**；只有「帧真的换了」
  （PX4 的 `xy_reset_counter` / `quat_reset_counter` 变化）才重钉。
- **相对参考**：`目标 = 锚点 + /traj/reference 的位置`；速度按参考自己的速度线性外推
  （`p_ref(t_k) = 锚点 + p_rel + v·(k·dt)`，`node_core.py:631-668`）。
- **参考超时**：`traj_stream_gap_s = 0.5 s` 只**冻结**（速度置 0、位置不外推），
  **不关闸门**（`node_core.py:1001-1010`、`:1136-1140`）。停发会让兜底阶梯接管，
  比「就地停住」激烈得多，而参考丢帧本身可恢复。

### 4.4 解锁与切 OFFBOARD —— 这一档的两步**必须人做**

`mpc_node` 的闸门要求 `armed + mode == "OFFBOARD"`（`node_core.py:1095-1100`），
但它**自己不 arm、不 set_mode**：

- `_PARAMS` 里没有任何 arming / set_mode 参数（`mpc_node.py:1619-1661`）；
- 订阅表里没有 `/mavros/rc/in`（`mpc_node.py:338-394`）；
- 手册与 README 都把「解锁 + 切 OFFBOARD 由操作者手打」写在明面上
  （`operation_manual.md:293-300`；`src/nmpc/README.md:110-111`）。

⟹ 模型预测控制档的完整现场顺序是：

1. 起链（菜单 9 第 2 档）→ `mpc` 在闸门关着时按 `offboard_prestream` 发悬停预流帧；
2. 操作者**解锁**（这一档没有 `control`，所以不能靠 ch9 的「第一次上升沿」；
   走 QGC / ch5 / 其他手动解锁路径）；
3. 操作者把模式切到 `OFFBOARD`（ch6 第 6 档或 QGC）；
4. 闸门打开；若还没有参考，`start_traj_reference` 发一次 `/traj/start`；
5. 下一拍参考到达，`traj` 从 t = 0 开始发；**锚点就是此刻的位置** ⟹ 飞机应当
   **停在地面上再解锁/切模式**（`operation_manual.md:308-311`），否则「轨迹原点」会
   钉在半空中的某一点。
6. 之后没有自动降落：`progress ≥ 1.0` 只让参考停在终点，`mpc_node` **不请求 `AUTO.LAND`**。
   收尾要人做（打杆 / 拨 ch6 / 长按 ch9 —— 但 ch9 在这条链上没有读者，
   所以实际是打杆或拨 ch6 交还控制权；`operation_manual.md:104-118`）。

### 4.5 它发出去的帧

`/mavros/setpoint_raw/attitude`（`mavros_msgs/AttitudeTarget`）：

- `type_mask = 128`（`IGNORE_ATTITUDE`）⟹ 用 `body_rate`、忽略 `orientation`、`thrust` **不被忽略**；
- `body_rate` 三个分量填 **FLU 原样**（FLU→FRD 由 mavros 做）；
- `thrust` 填**正号**归一化 `F / T_max`；
- 每帧都走 `_normal_frame()` 的三道判据：`solution_usable` → 加垂直偏置 `ΔF` 并
  `clip(u[0]+ΔF, 0, f_max)` → `sign_map.mavros_attitude_target` → `frame_is_finite`
  （`mpc_node.py:1167-1211`）；
- 解不可用时走兜底阶梯 `hold → hover`，兜底帧与正常帧走**同一条话题、同样的字段**，
  只是内容不同（`mpc_node.py:1213-1230`、`:1242-1267`）。

---

## 5. 两条链的「轨迹原点」不是同一个东西

| | (A) 级 offboard 控制 | 模型预测控制 |
|---|---|---|
| 谁记 | `control` 的 `Fsm` | `mpc_node` 的 `NodeCore` |
| 什么时候记 | **悬停判稳之后**（位置误差 < 0.10 m ∧ 速度 < 0.10 m/s 持续 2 s） | **闸门第一次打开那一拍** |
| 记的是什么 | 那一刻的 EKF 位置（`odom.x/y/z`） | 那一刻的测量位置（动捕或 PX4，取决于 `state_source`） |
| 代码 | `fsm.hpp:1426-1434` | `node_core.py:1031-1057`、`:1145` |
| 闸门再次打开时 | 不重钉（`traj_origin` 在 `request_takeoff` 时清掉，本轮任务内固定） | **不重钉**；只有 PX4 帧复位计数变化才重钉 |
| 和悬停点的关系 | 判据保证两者相差 ≤ 0.10 m | 没有单独的悬停点：锚点本身就是目标原点 |

⟹ 同一个物理动作（「以当前位置为 0」），在两条链上发生在**任务的不同阶段**：
(A) 级发生在「起飞到 1 m 并稳住」之后；模型预测控制发生在「刚进 OFFBOARD」。
★ 这也是为什么模型预测控制档的**起飞要靠 `traj` 的第 4 条轨迹**（从锚点往上爬 0.2 m），
而不是 `control` 的爬升斜坡：这条链没有 `control`，没有 `takeoff_altitude_m` 那一套
（`traj.yaml:60-70`；`src/traj/src/traj_node.cpp:470-475`）。

---

## 6. 容易混的八件事（逐条带证据）

1. **「选轨迹」≠「开始飞」**。`/traj/select` 不产生参考；`/traj/start` 才是时钟零点
   （`traj_node.cpp:251-252`、`:292-293`、`:320-322`）。
2. **`traj` 不碰飞控**。不发 setpoint、不解锁、不进 OFFBOARD（`src/traj/config/traj.yaml:3`）。
3. **(A) 级链里解锁与切模式是 `control` 做的**，开始键只是触发器；模型预测控制链里这两件事
   **没有任何程序入口**（`fsm.hpp:1354-1366`、`:1323-1324`；`mpc_node.py:1619-1661`）。
4. **开始键在模型预测控制链上不存在**：不是「被忽略」，是**没有读者**（`control_node.cpp:380-392`
   是唯一读它的地方；`mpc_node.py` 不订阅 `/mavros/rc/in`）。
5. **`/traj/start` 的发布者随链而变**：(A) 级是 `control` 进 TRACK 时；模型预测控制是
   `mpc_node` 在 `no_traj_reference` 那一拍（且仅在参数打开时）。重复发会把参考时钟归零，
   现场表现为「飞机当场掉头飞回轨迹起点」（`fly.py:13-17`；`mpc_node.py:1096-1097`）。
6. **两档都起 `traj`**，没有「只跑 `hold`、不起 `traj`」的档（`run/z0.py:1028-1049`）。
7. **`fly.py` 在模型预测控制档的状态栏是空的，不是坏了**：它读 `/control/state` 与
   `/control/traj_progress`，而这两个话题只有 `control` 在发（`fly.py:50-57`；
   `operation_manual.md:314-317`）。
8. **`offboard_prestream` 只解决「飞控肯不肯收」，不解决「谁解锁」**；它在那两个原因下发的是
   纯悬停帧，而且不在 OFFBOARD 时飞控根本不消费姿态/角速率字段
   （`mpc_node.py:1133-1145`；`src/nmpc/README.md:94-104`）。

---

## 7. 证据索引（file:line）

| 结论 | 一手出处 |
|---|---|
| `traj` 的选/开始两段语义 | `src/traj/src/traj_node.cpp:242-299`、`:305-340` |
| 参考的坐标系与掩码 | `src/traj/src/traj_node.cpp:53`、`:342-348` |
| 第 4 条轨迹（起飞 0.2 m / 0.2 m/s） | `src/traj/config/traj.yaml:60-70`；`src/traj/src/traj_node.cpp:470-475` |
| 两档链的尾巴 | `run/z0.py:1028-1049`、`:1088-1090` |
| `control` 的每拍顺序 | `src/control/src/control_node.cpp:370-426` |
| 开始键三次动作的语义 | `src/control/include/control/fsm.hpp:1036-1108` |
| (A) 级起飞/悬停/轨迹原点/TRACK | `src/control/include/control/fsm.hpp:1337-1515` |
| (A) 级的 OFFBOARD 预流与解锁前置 | `src/control/include/control/fsm.hpp:1323-1324`、`:1354-1366` |
| 开始键参数 | `src/control/config/control.yaml:98-117` |
| 任务阈值与超时 | `src/control/config/control.yaml:193-230` |
| 模型预测控制闸门顺序 | `src/nmpc/nmpc/node_core.py:1078-1102` |
| 锚点钉住的时机与重钉规则 | `src/nmpc/nmpc/node_core.py:1031-1057` |
| 相对参考 + 锚点 | `src/nmpc/nmpc/node_core.py:631-668` |
| 参考超时只冻结 | `src/nmpc/nmpc/node_core.py:1001-1010`、`:1136-1140` |
| `mpc_node` 每拍顺序 | `src/nmpc/nmpc/mpc_node.py:924-1036` |
| `start_traj_reference` | `src/nmpc/nmpc/mpc_node.py:1084-1118`、`:1634` |
| `offboard_prestream` 与预流权限边界 | `src/nmpc/nmpc/mpc_node.py:1120-1165`、`:1633`；`src/nmpc/README.md:94-104` |
| 模型预测控制的下行帧与判据 | `src/nmpc/nmpc/mpc_node.py:1167-1211`、`:1242-1267` |
| 参数默认值 | `src/nmpc/nmpc/mpc_node.py:1619-1661` |
| 现场操作顺序与「起链 ≠ 能飞」 | `operation_manual.md:266-317` |
| 模型预测控制链上开始键不起作用 | `operation_manual.md:293-300` |

---

## 8. 本次**没有**核实的部分（不写成结论）

1. **模型预测控制链飞行中重选轨迹**（§1.4 后半）：结论是读代码推出的
   （`traj_pos` 收取后不回 `None` + `_maybe_start_traj` 只在 `no_traj_reference` 触发），
   本次没有实飞、也没有在 SITL 里复现。若后续要依赖「重选能接上」，先补一次实跑或
   SITL 用例。
2. **模型预测控制档的实际解锁路径**：手册说「由操作者手打」，但没有成文的 mavros 命令
   （`/mavros/cmd/arming` + `/mavros/set_mode`，见 `sign-map.md:111`、`:126`）。
   本次只核到「`mpc_node` 自己不 arm、不 set_mode」这条否定事实；
   **具体在 QGC 里点哪个按钮、ch5 会不会被判成外部解锁**，没有一手演练记录。
   ⚠️ 注意：(A) 级链的 `control` 会把「不是自己发起的解锁」判成外部解锁并停发
   （`bench-runbook.md:99-114`）；模型预测控制链没有这个判据，但也没有任何归属机制。
3. **`offboard_prestream` 的实机效果**：权限边界有 PX4 源码依据（不在 OFFBOARD 时不消费姿态帧），
   但「PX4 是否因此肯在未解锁状态下接受 OFFBOARD 请求」在真机上的读数本次没有。
   台架相关步骤见 `bench-runbook.md` T3。
4. **本文所有 `file:line` 都是 2026-10-17 的工作树快照**：工作树里还有别的会话的在制品
   （`git status` 有一批已修改文件），行号可能随并行改动漂移；引用前先按代码内容复核。

---

## 实现后的现行事实（2026-10-17 施工结果）

| 问题 | 改造前（§0–§8） | 现在 |
|---|---|---|
| 菜单 9 第 2 档起什么 | `traj → mpc` | **`traj → mpc → control`**（`control output_mode=external`） |
| 谁读开始键 ch9 | 只有 `control`；第 2 档没有 `control` ⟹ 不起作用 | **仍是 `control`**；两档都有它 ⟹ **第 2 档开始键照常工作** |
| 谁解锁 / 切 OFFBOARD / 起飞 / 判稳 / 记轨迹原点 / 发 `/traj/start` / 降落 | 第 2 档全靠操作者手打（且不会自动降落） | **全部由 `control` 做**，两档逐字相同 |
| `mpc_node` 的参考从哪来 | `/traj/reference`（相对量）+ 自己的锚点 | **`/control/target`（绝对量）**，**不加锚点** |
| 谁写 `/mavros/setpoint_raw/local` | `control`（第 1 档） | **只有 `position` 档的 `control`**；`external` 档下这个发布者**不建** |
| 谁写 `/mavros/setpoint_raw/attitude` | `mpc_node`（第 2 档） | `mpc_node`（第 2 档）—— 唯一变化是目标来源 |
| `/traj/start` 谁发 | 第 1 档 `control`；第 2 档 `mpc_node`（`start_traj_reference`） | **两档都由 `control`**；`mpc_node` 在 `control` 参考源下**拒绝**该参数 |
| 参考断流 | `mpc_node` 冻结在最后一个参考 | **`control` 目标断流 0.5 s ⟹ `no_control_target` 关闸**（**零帧**，不是冻结；也不是 `hold → hover` 阶梯接管） |
| 参考速度 | `mpc_node` 从 `/traj/reference` 自己读 | `control` 原样转发到 `/control/target`；断流时**归零**（不许拿旧速度外推） |

★ **坐标口径**：`/control/target` 的字段值是 **ENU / FLU**（与仓库里 `/traj/reference` 的既有做法一致）；
`coordinate_frame` 填 `FRAME_LOCAL_NED` 只是占位，**不做 NED 转换** —— mavros 在两条下行话题上
分别负责 ENU→NED / FLU→FRD。

★ **可扩展性**：新控制方式 = 一个消费 `/control/target`、自己写执行器话题的节点。
`control` 的 FSM、`traj`、开始键、解锁、降落**一行都不用改**；`output_mode` 加一个枚举值即可。

## 本轮**没做**（不静默）

1. **飞行中重选轨迹**（§1.4 / §8①）：`mpc_node` 的 `traj_pos` 收到后不回 `None` 的行为未改；
   第 2 档飞行中重选轨迹仍会停在最后一个参考上。要接上得改 `traj` 的重选语义或让 `control`
   在参考断流时重发 `/traj/start` —— **本轮没动**，因为两档共用 `control` 之后，
   重选时 `control` 的既有 `traj_stream_gap_s` 重发逻辑是否够用**没有实飞证据**。
2. **实飞验收**：本票所有结论来自用例与离线核读；两档「现场动作逐字相同」**没有实飞读数**。
3. **新的控制后端**：没有任何第三种控制方式的代码；本轮只把接口形状留好。
4. **`output_mode` 的 launch/CLI 之外的配置面**：只有 yaml 与 launch 参数，没有动态切换。
5. **`run/rec.py` 的预期话题没跟着分档**（归票 19，本轮**不改别人的在制品**）：
   场景 3/4/5 的 `expect` 写死 `/mavros/setpoint_raw/local`；第 2 档 `control` **不写**这条，
   由 `mpc` 写 `/mavros/setpoint_raw/attitude` ⟹ 用第 2 档录 3/4/5 会在收口摘要里
   误报"缺 `/mavros/setpoint_raw/local`"（包其实录到了，只是预期写错）。
   建议改为 `^/mavros/setpoint_raw/(local|attitude)$` 并跑 `run/tests/test_rec.py`。
6. **起链判据不到端到端**：`z0.py` 对 `mpc` 只等它自己的启动日志、对 `control` 只等
   `/control/state` 有帧；**没有**"`/control/target` 已在流"的判据。链起全了但没接通的
   情形，目前只能现场看 mpc 日志或 `ros2 topic hz /control/target`。

## 验收

★ 每一行**落地时**才打勾，证据写在方括号里。**没做就留空。**

- [x] 菜单 9 第 2 档的尾巴是 `traj → mpc → control`，两档都含 `traj` 与 `control`；
      第 2 档带 `reference_source:=control` / `offboard_prestream:=true` / `output_mode:=external`，
      **不带** `start_traj_reference`
      〔`run/tests/test_z0_menu.py` = **90 passed**：`test_两档的尾巴都起_traj_…`、
      `test_模型预测控制档没有_hold_那一条`、dry-run 用例〕
- [x] `control output_mode=external` 时**只发** `/control/target`、`/mavros/setpoint_raw/local` **零帧**；
      掩码 2496；STANDBY 目标是当前位置；TRACK 目标是「轨迹原点 + 参考」、速度原样转发
      〔`src/control/test/test_control_external_output.py` = **2 passed**：
      `test_external档_只发绝对目标_开始键与任务流程照旧`〕
- [x] 开始键第 1 次上升沿仍解锁 + 切 OFFBOARD + 起飞到 `WAIT_START`（只解锁一次）；
      第 2 次上升沿**松手**仍进 `TRACK` 并发 `/traj/start`
      〔同上用例；`fly_until` 分别等到 `AUTO/WAIT_START` 与 `AUTO/TRACK`〕
- [x] 不认识的 `output_mode` **启动即拒**（退出码非 0；日志含参数名与两个合法值；零 setpoint、零目标发布者）
      〔同上：`test_不认识的_output_mode_入口当场拒`〕
- [x] `TRACK` 里参考速度被转发到 `Decision`，参考断流时三元组归零
      〔`./build/control/test_fsm` 的 `Track.*` = **7 passed**，含
      `Track.参考速度原样转发_断流时归零`〕
- [x] `mpc_node reference_source=control`：没有目标 ⟹ 闸门不开且理由是 `no_control_target`；
      目标**直接用绝对值、不加锚点**、速度进日志；目标断流后**关闸**（零帧）、恢复后重新开闸
      〔`src/nmpc/test/test_mpc_node.py` 三条 control 档用例 **3 passed**〕
- [x] 文档同步：`run/README.md`、`operation_manual.md` §3.1″、`src/control/README.md`、
      `src/nmpc/README.md`、`docs/control/safety-paths-drill.md` §3.2、
      `docs/control/mpc-node-seam1.md` §12.3②
      〔本轮逐文件改写；文档守卫用例见全量测试〕
- [x] **全量测试**（含 `run/tests` 与各包 `colcon test`），`ACADOS_SOURCE_DIR` / `LD_LIBRARY_PATH` 都设
      〔`colcon test --executor sequential` + `BENCH_ROS_DOMAIN_ID=81`：
      **723 tests / 0 errors / 176 failures / 2 skipped**；分项：`control` **168 passed**、
      `nmpc` **255 passed**、`run/tests` **185 passed + 1 skipped**（`test_z0_check_live`
      要实机）。176 条失败**全部**是 `vrpn_listener` 的 copyright/cpplint/lint_cmake/uncrustify
      —— 本轮未改动的包，既有环境问题。⚠️ 并行（默认域 73）会让 nmpc/control 的 seam 1
      夹具互撞（“注入的 armed/OFFBOARD 没落到 /mavros/state 上”），必须串行 + 显式隔离域；
      一次并发跑出的 6 条 nmpc 红在隔离后整包复跑为 0 失败（含 SIGINT 收尾那条）。〕
- [x] **变异测试**：本轮每条新断言都主动弄坏一次确认会红
      〔9 条，均“手动破坏 → 对应断言转红 → 恢复后转绿”：
      ① external 档仍建 local 发布者 ⟹ `count_publishers(...) == 0` 红
      （最初只断言“零帧”时**没红**，补了发布者计数才红）；
      ② control 参考仍加锚点 ⟹ 锚点断言红；
      ③ 断流改成冻结 ⟹ 关闸断言红；
      ④ z0 第 2 档漏 `output_mode:=external` ⟹ chain 用例红；
      ⑤ 参考速度不转发 ⟹ `Track.参考速度原样转发_断流时归零` 红；
      ⑥ 掩码 2496 改成 2552 ⟹ external 用例红；
      ⑦ 未知 `output_mode` 不拒 ⟹ 退出码/日志断言红；
      ⑧ `declare_params()` 不读 `control_target_timeout_s` ⟹ 断流 0.45–0.70 s 断言红
      （实测“还在发 6 帧”，证明默认 0.5 s 的旧断言没有鉴别力）；
      ⑨ 删掉 `optionals_suffix` 的重复参数守卫 ⟹ `test_互斥的参考源不能同时打开` 红〕
- [x] `/code-review` 走一遍
      〔Standards / Spec 两轴各一个子代理。核实后修掉：`px4_interface.hpp` 的枚举注释
      抢了 `Px4Config` 的说明；`run/README.md` 引用已改名的 `CHAIN_CHOICES`、可选参数
      “三条”实为四条；两个参考源可同时打开（加 `ValueError` 守卫 + 单测）；`mpc-node-seam1.md`
      §12.3 索引仍写“② 仍未决”与下文矛盾；`control.yaml` 的 `output_mode` 在 launch 下被覆盖
      （就地标注）；`nmpc/README.md` “不再选 traj”说得太满；根 `README.md` 仍写四档。
      驳回并记录理由：`absolute_reference`/`traj_reference` 合并与测试夹具复制属判断类气味，
      本轮不动；`publish_setpoint` 的空发布者守卫已有注释说明，不是静默失败〕
- [ ] **实飞验收**（归操作者）：两档分别按开始键飞轨迹 1/2/3/4；本票不代做

## Comments

### 2026-10-17 —— 操作者要求（本票现行版）

> `本票任务：让z0.py选9的mpc控制也可以用轨迹1234飞行，并且也是通过开始按键解锁起飞，开始按键开始轨迹。统一俩种控制的流程，让直接发目标点和mpc控制可以相互替代，其他control、traj、开始按键等都是俩种控制共用的`

> `并且后续还可能会有更多控制方式的加入，不同的控制方式相互平替，共用control、traj等共有流程`

**设计裁定（由上一句直接推出，施工按此落地）：**

1. `control` 是**唯一任务总管**；「直接发目标点」与「模型预测控制」只是它的两种**输出后端**。
2. 两种后端**结构互斥**：`position` 档建 `/mavros/setpoint_raw/local` 发布者，
   `external` 档建 `/control/target` 发布者，**不并存**。
3. `/control/target` 发**绝对目标**；外部控制器**不许再加锚点**（否则轨迹原点会被加两次）。
4. `/traj/start` 归 `control`；`mpc_node` 的 `start_traj_reference` 在 `control` 参考源下**拒收**。
5. 目标断流按**关闸**处理，不按冻结处理（冻结会拿旧目标继续飞）。
6. 将来的控制方式沿用同一形状：消费 `/control/target`、自己写执行器话题；
   **不改** `control` / `traj` / 开始键 / 解锁 / 降落。

★ **Status 不动为 `ready-for-agent`**：归谁收口是操作者的裁定（本仓纪律）。

### 2026-10-17 —— 施工记录（简）

- **测试**：`test_z0_menu.py` 89 passed；`test_control_external_output.py` 2 passed；
  `test_fsm` 的 `Track.*` 7 passed；`test_mpc_node.py` 的 control 档 3 passed。
- **构建**：`control` 包 colcon build 成功（`test_fsm` 编过并跑过）。
- **未提交** ☠️（已被下面 05:5x 那条取代）：当时还没跑全量与复核；最终提交范围见下。
- **并行会话**：本工作树里有别的会话的在制品，提交前逐文件按特征串核对自己的改动仍在。

### 2026-10-17 —— 收口证据（全量 / 变异 / 复核）

- **全量**：串行 + 隔离域下 723 tests / 0 errors；`control` 168、`nmpc` 255、
  `run/tests` 185 passed + 1 skipped。唯一的 176 条红是未改动包 `vrpn_listener` 的 lint。
- **一次并发污染（记录在案）**：默认域并行跑时，nmpc 与 control 的 seam 1 夹具互相看见对方的
  假 `mavros`（报“注入的 armed/OFFBOARD 没落到 /mavros/state 上”“夹具报告的已经是解锁状态”），
  同域还有复核子代理起的 external 用例。隔离域 + 串行后这些红全部消失 ⟹ **不是代码缺陷**。
- **变异**：9 条新断言逐条弄坏均转红（清单见验收）。
- **复核**：两轴（Standards / Spec）各一个子代理；修掉 7 处，驳回 2 处判断类气味（理由见验收）。
- **提交**：只 add 本票路径。⚠️ `run/z0.py`、`run/tests/test_z0_menu.py` 同一文件里带着
  **先行未提交**的票 14 二定与 Ctrl-C 处置改动（本会话未改那些行）；整文件纳入并在提交信息里
  逐项写明。其余在制品（`src/traj/`、`run/rec.py`、`bag/`、`.scratch/09`、`.scratch/14` 等）不提交。
