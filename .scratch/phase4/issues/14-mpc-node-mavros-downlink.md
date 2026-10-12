# 14: `mpc_node` 下行换 mavros —— 话题 / 消息 / 闸门源 / 超时 / 依赖

Status: ready-for-agent
Type: task

**What to build:** 让模型预测控制节点把它的 (C) 级指令发到 **`/mavros/setpoint_raw/attitude`**
（`mavros_msgs/AttitudeTarget`），闸门改读 **`/mavros/state`**，并让整条链在没有 uXRCE-DDS 的 z0 上能跑起来。

**Blocked by:** [13](13-docs-mavros-downlink-canon.md)（口径还没冻之前动手，等于按个人理解写 N1/N2/N3）。
**口径不在本票决定** —— 三处所有权（N1/N2/N3）与"哪条路由谁取负"写在
[`spec.md`](../spec.md) §★★ 与 `sign-map.md` 里，**本票照抄**。

**它挡的是什么**：**被验对象换了一个，而三处差异全部以静默形式出错**。
话题名/QoS 写错 ⟹ 消息根本没人收（这个至少会静默不发，容易看出来）；
符号与闸门那三处**收得到、也不报错、飞机朝错的方向走**。

---

## 逐项落点

### ① `src/nmpc/nmpc/mpc_node.py` —— `TOPIC_*` 与 import

| 现值 | 换成 | 说明 |
|---|---|---|
| `TOPIC_RATES = "/fmu/in/vehicle_rates_setpoint"`（`VehicleRatesSetpoint`） | **`/mavros/setpoint_raw/attitude`**（`mavros_msgs/AttitudeTarget`） | 本票的主改点 |
| `TOPIC_STATUS = "/fmu/out/vehicle_status"`（`VehicleStatus`） | **`/mavros/state`**（`mavros_msgs/State`） | 闸门源，见 ③ |
| `TOPIC_MODE = "/fmu/in/offboard_control_mode"` | **删掉** | 心跳不用我们发：飞控从收到的 `SET_ATTITUDE_TARGET` 自己派生 `OffboardControlMode`（`mavlink_receiver.cpp:1531-1535`） |
| `TOPIC_PX4_POS` / `TOPIC_PX4_ATT` | **见 ⑤（A 档处置）** | `/fmu/out/vehicle_local_position` / `vehicle_attitude` 同样没了 |
| `TOPIC_MOCAP` / `TOPIC_MOCAP_VEL` / `TOPIC_TRAJ_REF` / `TOPIC_PATH` / `TOPIC_REF` | **不动** | 与下行无关 |

- `from px4_msgs.msg import (...)`（`mpc_node.py:208`）→ `from mavros_msgs.msg import AttitudeTarget, State`。
  ⛔ **别留一个没用到的 px4_msgs import** —— 那会让"已经不用 px4_msgs 了"这件事在依赖清单外看不出来。
- **QoS**：mavros 在 `setpoint_raw` 的订阅上用的是 **`SensorDataQoS()`**（best_effort / keep_last 5 / volatile，
  `setpoint_raw.cpp:62`），而现在的 `QOS_PUB_FMU` 是 best_effort + **TRANSIENT_LOCAL** depth 1。
  **要显式选**：保留 best_effort（兼容），把 durability/depth 对齐到收端的口径。
  ⛔ **不许"能通就不管"** —— 写死一个显式值并在 Comments 里写为什么。

### ② 消息怎么填（★ 符号这一步是本票最容易静默错的地方）

- `type_mask = 128`（`IGNORE_ATTITUDE`）⟹ `body_rate` 被用、`orientation` 被忽略、**`thrust` 不被忽略**。
  这与现在 `OffboardControlMode` 里报的 `body_rate = true` 一致；`type_mask` mavros **原样透传**。
- **`body_rate` 填 `sign_map` 的 ω 输出（FLU）原样**，⛔ **不许自己先转 FLU→FRD**：
  mavros 在 `attitude_cb` 里会做（`setpoint_raw.cpp:321-331`，常量 `ftf_frame_conversions.cpp:40`），
  而它做的**恰好就是** `sign_map` 对 pitch/yaw 的取负 ⟹ 自己再转一次 = **两次取负 = 不取负**，
  roll 不受影响、pitch/yaw **静默反向**。
- **`thrust` 填 `+F/T_max`**（正号）：飞控那行 `thrust_body_array[2] = -thrust`
  （`mavlink_receiver.cpp:1475`）会加负号。★ 这**正好与现有 `sign_map.thrust_body_z = -F/T_max` 反号**
  ⟹ 票面要求**在 `sign_map.py` 里加一个具名的第二入口**（仍是一个纯模块、一个无 ROS 的实现），
  例如 `mavros_attitude_target(omega_flu, thrust_n, t_max_n) -> MavrosAttitudeTarget`，
  字段名叫 `thrust` 而**不叫** `thrust_body_z`（字段名本身就是自证的）。
  ⛔ **不要**用 `path="mavros"` 这种布尔/字符串开关把两条路塞进同一个返回类型 ——
  那样字段名会开始撒谎，而**这正是 N1 要挡的静默**。
- `sign-map.md` §8 的"一个纯实现"契约**不变**；两个具名入口的 docstring 各自指回文档里"谁取负"那张表，
  且**不许复述数字**。

### ③ 闸门源 → `/mavros/state`

- 判据换成 **`armed == True`** 且 **`mode == "OFFBOARD"`**（对应 `NAVIGATION_STATE_OFFBOARD = 14`）。
  ★★ **开工第一件事：核 `mode` 那个字符串到底是什么** —— 本票开票时**没有**第一手核过
  `/mavros/state.mode` 在 OFFBOARD 时的取值（是 `"OFFBOARD"` 还是别的拼法）。
  **核法**：读 mavros 填这个字段的地方（`sys_status.cpp` 里 `/mavros/state` 的构造），
  或台架上 `ros2 topic echo /mavros/state` 对着遥控切 OFFBOARD 看。
  ⛔ **不许猜**；把核到的字符串与核法写进 Comments。
- **超时**：`status_timeout_s: float = 0.75`（`mpc_node.py` 起参数 / `node_core.py:208`）**必须改大**。
  `/mavros/state` 只在收到 MAVLink HEARTBEAT 时发布，**1 Hz 定频**（`sys_status.cpp:962-985`），
  而 `0.75 < 1.0` ⟹ `status_stale` 会**恒真**、每秒报"飞控失联"并停发 setpoint。
  **推荐 2.5 s**，与 Phase 1 同值同理由（`src/control/config/control.yaml:120-125` 那段注释写死了"不要照抄 0.5 s"）。
  ★ 本项**必须有一条断言护住**（`test_mpc_node.py` 里那条"闸门源超时"用例），
  并且要**变异测试**过：把参数改回 `0.75` ⟹ **必须变红**。⛔ 红不了就说明这条断言没鉴别力。
- `node_core.py` 的四种 hold（`no_status` / `not_armed` / `not_offboard` / `status_stale`）**语义不变**，
  变的只是数据来源；`stop_on_nav_state_exit=True` 的语义要重新对一遍（退出 OFFBOARD 时还能不能判出来）。

### ④ `heartbeat_hz` / `heartbeat_always` —— **语义变了，要显式选一个**

飞控**只有一条流**了（`SET_ATTITUDE_TARGET` 本身就是心跳），
⟹ 现在 `rate_hz`(100) 与 `heartbeat_hz`(50) 的**双频设计失去了它的物理对应物**。
本票必须**显式选**并写进 Comments（不许留成隐含行为）：

- **(a) 收成单频**：删掉心跳那条，`rate_hz` 一条流到底；或
- **(b) 重解释 `heartbeat_always`** 为"闸门关上也照发，但发一条**零推力 + 零角速率**的 setpoint"。

★ **两条路的差别在安全上是有意义的**：闸门关上（未解锁 / 停在非 OFFBOARD）时**发不发**，
决定飞机那一刻收到的是什么。飞控那侧 `:1593` 的发布**被 OFFBOARD 判据守着**，
所以非 OFFBOARD 时发什么都进不了控制环 —— **别把这个"当前无害"当成不写的理由**：
这是**别人的一行代码**在替我们兜底，而它不在本仓库里。

### ⑤ A 档（`state_source="px4"`）的处置 —— **二选一，写明理由**

它读 `/fmu/out/vehicle_local_position` + `/fmu/out/vehicle_attitude`，**换通路后同样死了**。

- **(a) 重指**到 `/mavros/local_position/*` + IMU 姿态 —— ⚠️ mavros 会**帮你转**（ENU→NED、FLU→FRD），
  "转两次"那个坑要重新过一遍（`px4-interface.md` §5.1）。
- **(b) 判为 uXRCE-DDS 专用、本阶段不可用**，并在 A 档入口加一条**显式拒绝**（fail loud，不是静默无数据）。

**本票默认取 (b)**，理由：本阶段"A 级回退"的载体是 **Phase 1 的 `control` 包整条链**，
不是节点内部的 px4 状态源；把一条死路留着不如让它当场喊出来。
⚠️ **若选 (a) 必须由操作者裁定** —— 它改的是回退路径，而回退路径是安全相关件。

### ⑤bis ★★ 一条**硬前置**：`MAV_FWDEXTSP = 1`

整个 `SET_ATTITUDE_TARGET` handler 被 **`_mavlink->get_forward_externalsp()`** 守着
（`mavlink_receiver.cpp:1508-1515`），也就是飞控参数 **`MAV_FWDEXTSP`**
（`.../src/modules/mavlink/mavlink_params.c:112-121`，"Forward external setpoint messages"）。
**z0 现值 = `1`**（[`z0.params:542`](../../../docs/assets/z0.params)）✓。
⚠️ **若它是 `0`，我们发的每一条 setpoint 都会被静默丢掉**（飞控侧**没有任何日志**，
表现和"话题名写错"几乎一样）。本票要：

- 把它**写进 `preflight_check.py` 的检查项**（与 ⑥ 的核查一起做）；
- 在票面 Comments 里**记下它今天已经是 `1`**（这样将来若变成 `0`，有据可查是"被改过"）。

### ⑥ 依赖与构建

- ★ **开工时先看清单：`mavros_msgs` 已经在里面了**（`src/nmpc/package.xml:38`）——
  所以本项的**动作是"撤掉 px4_msgs"**（`:22`，及 `:19` 那条指着 `~/px4_msgs_ws` 的注释），
  ⛔ **不是"加一条 mavros_msgs"**。
  `src/bench/package.xml` 那两份（`:23` 的 `mavros_msgs`、`:27` 的 `px4_msgs`，后者注释明确写着
  "是 `fake_px4.py` 用到的"）归 [16](16-fixtures-and-seam1-mavros.md)，
  本票**只改 nmpc 那一份**，⛔ 别顺手改 bench 的 —— 见 [别顺手改别的票]。
- `README.md` / `src/nmpc/README.md` 的构建说明里"source `~/px4_msgs_ws`"→ `~/mavros_ws`
  （后者本仓库 `control` / `bringup` **已经要用**，不是新依赖）。
- `src/nmpc/nmpc/__init__.py:18` 与 `src/nmpc/README.md:22` 那句
  "**除 `offboard_smoke` 外都不许 import `rclpy` / `px4_msgs`**" —— ★ 这句话**表述本身已经不准**
  （`mpc_node.py` 是入口脚本、本来就在那份约束之外）。**改成与新事实一致的表述**，⛔ **不许默默删掉**
  （它的**原意**——"核心层不依赖具体消息包"——仍然是对的且值得留）。
- ★ **开工时先核 `preflight_check.py`**（本票开票时**未核**）：里面若存在 `/fmu/*` 存在性检查
  或 `UXRCE_DDS_CFG` 检查，一并改；该换的换、该 ☠️ 的 ☠️。归属写进 Comments。

### ⑦ 顺带（**不是本票的主体，但别漏**）

- `docs/control/mpc-node-seam1.md` 的 §11.1 冻结档表与 §11.2 那条"可整段复制的命令"里，
  **有本票改掉的旋钮**（起参数 / 依赖 source 行）⟹ 逐处更新，别留读起来像还能照抄的命令。
  （seam 1 用例本身的改写归 [16](16-fixtures-and-seam1-mavros.md)。）

---

## 验收

- [ ] `mpc_node.py` 里**没有任何** `/fmu/*` 话题常量与 `px4_msgs` import（用 grep 现算，不靠肉眼）
- [ ] 下行发的是 `/mavros/setpoint_raw/attitude`，`type_mask = 128`，`body_rate` 是 **FLU 原样**（未预转换）
- [ ] `thrust` 发的是 **`+F/T_max`**，且这一处**在代码里能读出"飞控会加负号"**（注释带锚点）
- [ ] `sign_map.py` 有两个**具名**入口（px4_msgs 那条与被 mavros 用的那条），字段名各自自证，**没有**布尔开关
- [ ] 闸门读 `/mavros/state`，判据是 `armed` 且 `mode == OFFBOARD`；`mode` 的**字符串有第一手出处**（写进 Comments）
- [ ] `status_timeout_s` 已按 1 Hz 发布周期改大，且**有一条断言**护住；**变异测试**把值改回去 **必须变红**
- [ ] `heartbeat_hz` / `heartbeat_always` 的去留**显式选了 (a) 或 (b)** 并写进 Comments，没留隐含行为
- [ ] A 档：**显式选了 (a) 或 (b)**；(b) 的话入口会**当场拒绝**而不是静默无数据；
      选 (a) 的话在 Comments 里记了"已请操作者裁定"
- [ ] `src/nmpc/package.xml` 里的 `px4_msgs` 已撤（`mavros_msgs` **本来就有**，不是新加的）；README 的 source 行同步；**bench 的没动**
- [ ] `__init__.py` / `nmpc/README.md` 那句"不许 import"改成了与新事实一致的表述（**不是删除**）
- [ ] `preflight_check.py` 已核并处置（或明确记为"本票范围外 + 归哪张票"），
      且**含 `MAV_FWDEXTSP = 1` 这一条**（为 `0` 时整条下行静默死掉）
- [ ] Comments 里记了"开票时 `MAV_FWDEXTSP = 1`"（将来若变 `0`，有据可查是**被改过**）
- [ ] `mpc-node-seam1.md` §11.1 / §11.2 里被本票改掉的旋钮已同步（⛔ 不留读起来像能照抄的命令）

---

**为什么是 `ready-for-agent`**：纯软件改动，台架上就能验（配合 [16](16-fixtures-and-seam1-mavros.md) 的夹具），
**不需要飞机通电、不需要起链**。★ 唯一的例外见 ⑤：A 档若选 (a) 要操作者裁定 —— 票面已把它设成**非默认**。

---

## Comments

### 2026-10-12 —— 开票时已核过 / **未核**的，分开放

**已核（读源码得来，不是实测）**：

| 事实 | 锚点 |
|---|---|
| `fill_thrust` 加负号 | `mavlink_receiver.cpp:1456`（签名）、`:1475`（`thrust_body_array[2] = -thrust;`） |
| OCM 由 `SET_ATTITUDE_TARGET` 派生 | 同文件 `:1531-1535` |
| 只在 OFFBOARD 里发布 rates setpoint | 同文件 `:1593` |
| mavros 订阅用 `SensorDataQoS` | `setpoint_raw.cpp:62` |
| mavros 替我们转 `body_rate` | `setpoint_raw.cpp:321-331`；`ftf_frame_conversions.cpp:40` |
| `AttitudeTarget` 字段（无 `thrust_body`） | `/opt/ros/humble/share/mavros_msgs/msg/AttitudeTarget.msg` |
| `/mavros/state` 1 Hz + `StateQoS` | `sys_status.cpp:962-985`、`:583` |
| 同款超时坑的先例 | `src/control/config/control.yaml:120-125`（`state_timeout_s: 2.5`） |
| **硬前置** `MAV_FWDEXTSP = 1` | 消费端 `mavlink_receiver.cpp:1508-1515`；定义 `.../mavlink_params.c:112-121`；z0 现值 `z0.params:542` |

**未核（本票开工时要补，⛔ 不许当成已知）**：

1. **`/mavros/state.mode` 在 OFFBOARD 时的字符串**（见 ③）—— 只核到"飞控侧的判据是
   `NAVIGATION_STATE_OFFBOARD = 14`"，**mavros 报出来的名字没核**。
2. **`preflight_check.py` 里有没有 `/fmu/*` / `UXRCE_DDS_CFG` 的检查**（见 ⑥）。
3. **mavros 收端 QoS 与我们现在这个 publisher 的兼容性**：按 DDS 规则
   writer `best_effort` + `transient_local` 配 reader `best_effort` + `volatile` **应当可通**，
   但**没有实测过**（[16](16-fixtures-and-seam1-mavros.md) 的夹具要用**真的** `SensorDataQoS` 才算验过）。
4. **真机上有没有跑过一次**：**没有**。本票全部依据是读源码。
