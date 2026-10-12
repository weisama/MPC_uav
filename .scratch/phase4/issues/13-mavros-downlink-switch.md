# 13: (C) 级下行从 px4_msgs/uXRCE-DDS 换成 mavros

Status: ready-for-human
Type: task

**What to build:** 把 Phase 4 的 **(C) 级下行**从 px4_msgs/uXRCE-DDS 换成
**mavros 的 `/mavros/setpoint_raw/attitude`**，让四级阶梯在**不新增硬件、不抢串口**的前提下继续往下走 ——
含口径冻结（docs 先行）、节点、阶梯驱动工具、夹具与 seam 1、判据留痕，以及一条**与通路无关**的独立项。

**Blocked by:** 无。★ 但**本票内部有次序**（原先六张票的阻塞边，现变成同一张票里的先后）：
**§一 必须在 §二 / §三 之前**（口径没冻就动手 = 按个人理解写 N1/N2/N3）；
**§四 在 §二 之后**（DUT 得先会说 mavros 的话）；
**§五 在 §三 之后**（留痕要写"换成了什么"）；
**§六 与任何人无关、随时可做**。

**★ 本票为什么是 `ready-for-human`（而内部有一大半是 AFK 可推进的）**：
`ready-for-human` 只要**有一处**必须人来做就成立 —— 本票有两处：**§五**（§7.2 的"谁定的"列要操作者签字，
且其中一行有安全后果）与**§六**（要动飞机：通电、点电机 —— 车侧动作由操作者亲手做，
且"哪个口在转"只有眼睛看得到）。
**§一 / §二 / §三 / §四 全部是 AFK 可推进的**，验收清单里已按节标出归属，别把它们一起拖到人在场再做。

★★ **2026-10-12：四处"本来要人定"的口径已问过操作者并裁定**（四题全取推荐项 ——
§7.2 拆四行 / `--t-max` 改成必须显式给 / §六 排在阶梯 1 之前单独做一次 / A 档取 (b)）。
裁定**已回写**到对应小节与验收框，集中副本见 **Comments 末节**。
⚠️ **仍未定的只剩一处**：§7.2 那张表**"谁定的"列的签字** —— 那是操作者自己的名字，只有人能签。

**它挡的是什么**：一次**被验对象换掉了的**通路切换。三处差异（`spec.md` §★★ 的 **N1/N2/N3**）
**全部以静默形式出错** —— 判据全绿、消息也收得到，而飞机朝错的方向走。
所以本票的主线不是"把话题名改掉"，而是**把三处所有权钉死，再用能证伪它们的方式验一遍**。

---

## 0. 本票是怎么来的

**由 2026-10-12 同一会话里原来的六张票（13–18，已在 `70fac4b` 提交里）合并而来，
合并后沿用 `13` 这个号。**

⚠️ **号沿用 ≠ 内容沿用**：原来的 `13` 是"口径冻结"那一张，它现在是本票的 **§一**。
拿 `70fac4b` 那个提交里的 `13-docs-mavros-downlink-canon.md` 与本票对号的人会读岔 ——
那六份已 `git rm`，它们的**正文原样**在本票的 §一…§六 里，**没有第二处副本**。
（沿用 13 而不是新起一个号，是为了让号段接在 `12` 后面不留空；`13–18` 这一段现在只有 13 活着。）

合并的**只有容器，没有内容**：六节的原文逐字保留（含各自的锚点表与"未核"清单），
跨节引用改成了 `§一`…`§六` 的节内引用。
规范来源：[`spec.md`](../spec.md) → Implementation Decisions → **§★★ 2026-10-12**
（起因 / 等价性 / 硬前置 `MAV_FWDEXTSP` / 三处不可等价 N1–N3 / 范围）。

---

## 一、口径冻结（docs 先行，纯文档）

**它挡的是什么**：换载体本身不难，**难的是"换了之后哪一步归谁"没有一个共同口径**。
没有这份口径，票 §二/§三/§四 会各自按自己的理解写，而三处差异（N1/N2/N3）
**全都以"静默"的形式出错** —— 判据全绿、飞机朝错的方向走。本票就是先把这三条钉在纸上。
**谁做这一节**：纯文档、纯核对，锚点已在 [`spec.md`](../spec.md) §★★ 的表里给全，不需要台架、不需要飞机、**不需要操作者裁定**（"归谁"那类裁定已被这一节**排除在范围外**，它明确留给 §二）。**★ 这一节完全 AFK 可推进**（§二 / §三 / §四 都排在它后面）。

### ① `docs/control/sign-map.md` —— 加一列「这一步在 mavros 路径上由谁执行」

- **§2 权威表**：现有四行（roll `+1` / pitch `−1` / yaw `−1` / `thrust_body_2 = −F/T_max`）。
  加一列，逐行写清在 **mavros 路径**上这一步是"我们做"还是"别人做"：
  - roll：我们做（mavros 的 `AIRCRAFT_BASELINK_Q = R_x(π)` 不动 x 分量）；
  - pitch / yaw：**别人做** —— mavros 在 `attitude_cb` 里替我们做了（= 我们的取负）；
  - `thrust_body_2`：**别人做** —— 飞控的 `fill_thrust` 写 `thrust_body_array[2] = -thrust`。
- **新增一节**（建议 §2.1「两条下行路径的所有权对照」）：一张两列的表，列 px4_msgs 路径 vs mavros 路径，
  逐行写"负号/轴向由谁产生"，锚点写全：
  - `~/refs/px4-v1.14.3-build/src/modules/mavlink/mavlink_receiver.cpp:1456`（`fill_thrust` 签名）、
    `:1475`（`thrust_body_array[2] = -thrust;`）；
  - `~/mavros_ws/src/mavros/mavros/src/plugins/setpoint_raw.cpp:321-331`
    （`body_rate = ftf::transform_frame_baselink_aircraft(...)`）；
  - `.../mavros/src/lib/ftf_frame_conversions.cpp:40`（`AIRCRAFT_BASELINK_Q`）、
    `.../mavros/include/mavros/frame_tf.hpp:271`。
- ★★ **要顺手把 §2 拆成两半**：**"物理约定"（哪条轴、哪个方向、哪两个电机该加速）与"落到线上那几个字段"**。
  前者**一个字不变**（它是判据，且与走哪条通路无关）；
  后者按通路分叉 —— 现在 §2 末列写的 `thrust_body_2 = −F/T_max` 是**px4_msgs 路径上我们的输出**，
  在 mavros 路径上同一个量变成了**飞控内部的变量**（由 `fill_thrust` 产生）。
  ⛔ 不是删掉那一列，是**标清它属于哪条路**。
- ★★ **§9 那四件自检（`tools/sign-map/sign_map_core.py` + `test_sign_map_core.py`）语义要一并说清**：
  它是**独立推导与交叉核对**（模块自己的 docstring 写着"**本模块不是权威**"），
  四件事是：① 独立推导（只用基的定义推出 `[+1,−1,−1]`，再与文档的 `sign-map:canonical` 块对账）；
  ② **物理闭环**（同一条指令在 ENU/FLU 与 NED/FRD 两套约定下必须给同一个世界系加速度）；
  ③ **反例钉死**（`test_counterexample_pitch_not_flipped_flies_backwards` /
  `test_counterexample_thrust_sign_flipped_crashes` / `test_counterexample_yaw_not_flipped_turns_the_wrong_way`）；
  ④ 三方对账（外部仓库不在时**明确报 SKIP**）。
  ★★ **第 ③ 件在 mavros 路径上仍然成立，但它的"前提"换了主人**：
  它证的是"**这一步不许没人做**"（pitch/yaw 不取负 ⟹ 反向飞；推力不取负 ⟹ 砸地），
  而不是"**必须由我们做**"。走 mavros 之后**取负挪到了别人的代码里**，
  ⟹ 这三条反例从"我们的实现的错法"升级成"**所有权转移的安全性证明**"：
  **正因为它们钉死了"漏掉就会摔机"，才需要一条新断言去证明"在 mavros 路径上它恰好被做了、且只做了一次"**
  （那条断言归 §四，**本票只负责在文档里把这个论证写出来**）。
  ⛔ **不许默默删掉这三条**。
- ★ **§8 第 4·5 条契约在 mavros 路径上要重新表述**：第 4 条（docstring 回链、不复写数值）**不变**；
  第 5 条（"测试断言的是本文件那张表，不是实现里复述的口径"）**对象变了** ——
  我们的实现不再直接产出 `thrust_body[2]` ⟹ 断言的落点要重新写清（落到我们**发出**的那个字段上）。
  **取舍与措辞要在票面 Comments 里记一笔。**
- **§8 代码契约**里"docstring 不许复述数字"那条**不变**；但"一个纯实现"这句话要补上**它的输出投喂给谁**。

### ② `docs/control/mpc-node-seam1.md` §12.3 —— 三条未决**逐条改写**（☠️ 留痕，不许删原文）

现状（2026-10-10 写的，前提是 uXRCE-DDS）：
① 链路口在真机上不存在；② 谁解锁 / 谁切模式无主；③ 两条链互斥。

改法（**逐条**，体例与该页既有的 ☠️ 标注一致）：

| 条 | 新状态 |
|---|---|
| ① **链路口** | **☠️ 消解**：不走 uXRCE-DDS ⟹ 不占串口、不加硬件。原那三条 `MAV_*_CONFIG` 读数**保留**（它们是本决定的**起因证据**），只是结论从"归属待定"变成"此路不通，改走 mavros"。 |
| ② **谁解锁 / 谁切模式** | **仍然无主**，但换成 mavros 形态：接口从 `/fmu/in/vehicle_command` 变成 `/mavros/cmd/arming` + `/mavros/set_mode`；候选归属（QGC 手动 / 把 `ArmAgent` 抬出来 / `offboard_smoke.py` 自己干）**一个没少**。★ 归属由 §二 裁，本票只把"仍是未决"写清楚。 |
| ③ **两条链互斥** | **仍然成立，且更难办**：(A) 级与 (C) 级**现在都走 mavros** ⟹ 不只是抢 OFFBOARD，还**共用同一组话题**（`/mavros/set_mode`、`/mavros/state`）。原文照 ☠️ 处理，另起一段写新形态。 |

### ③ `docs/reference/px4-interface.md` —— 从"开放决策"改成"定案"

- **§4.3「⚠️ 备选：全走 mavros」**：现在写的是"**当前倾向：混合。**…**但这仍是开放决策**（见 PROJECT.md O6）"。
  改成 **定案 = 全走 mavros**，写明日期（2026-10-12）与起因（`UXRCE_DDS_CFG = 0` + 串口被占 + 分配器静默不启动）。
  ☠️ **原句就地标掉、另起一行写新结论**，别把"开放决策"那行直接改没。
- **§5.1 的表**（"mavros 在哪些话题上帮你转、往哪个方向转"）：**加一行**
  `/mavros/setpoint_raw/attitude` 的 `body_rate` —— 同样是 FLU→FRD。这一行是 N2 的存档处。
- **§8「实飞前检查清单」**：`UXRCE_DDS_CFG` 已打开 / `ros2 topic list | grep /fmu` 能看到话题
  这两条**就地标 ☠️**（写清"本条作废，因为不再走 `/fmu/*`"），不要原地删。
  ★★ **并且新增一条**：**`MAV_FWDEXTSP = 1`** —— 整个 `SET_ATTITUDE_TARGET` handler 被它守着
  （`mavlink_receiver.cpp:1508-1515` 的 `if (_mavlink->get_forward_externalsp() && …)`），
  参数定义与说明见 `.../src/modules/mavlink/mavlink_params.c:112-121`；
  **z0 现值 = `1`**（`docs/assets/z0.params:542`）。
  ⚠️ **它是 `0` 时整条下行静默死掉**（消息被丢、飞控无日志）⟹ **必须进清单**。
  ★ 这条**不是"作废项"而是"新增项"** —— 与上面两条的处置方向相反，别一起处理。
  ★★ **同时要看清这一点**：☠️ 掉的是**这两条**，
  清单里其它条（推力范围 `[-1,1]` 之类）**不受影响** —— 别顺手把整节的效力也降级。

### ④ 顺手核一遍"还有哪些活文档把 (C) 级下行写死成 px4_msgs / `/fmu/*`"

至少过一遍（用宽搜法，别只搜一个词）：`docs/`、`src/nmpc/README.md`、`README.md`、`CONTEXT.md`、
`src/nmpc/nmpc/__init__.py`（`:18` 那句"除 `offboard_smoke` 外都不许 import `rclpy` / `px4_msgs`"）。
逐处**留痕或转票**，⛔ 不留悬空引用。**本票不改这些**（它们是 §二 / §三 / §四 的落点），只**登记成一张清单**
放进 Comments —— 清单要带路径 + 行号 + 归哪一节。

---

## 二、节点换下行 —— 话题 / 消息 / 闸门源 / 超时 / 依赖

**它挡的是什么**：**被验对象换了一个，而三处差异全部以静默形式出错**。
话题名/QoS 写错 ⟹ 消息根本没人收（这个至少会静默不发，容易看出来）；
符号与闸门那三处**收得到、也不报错、飞机朝错的方向走**。
**谁做这一节**：纯软件改动，台架上就能验（配合 §四 的夹具），**不需要飞机通电、不需要起链**。★ 唯一的例外见 ⑤：A 档若选 (a) 要操作者裁定 —— 票面已把它设成**非默认**。

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

### ⑤ A 档（`state_source="px4"`）的处置 —— ✅ **2026-10-12 已裁定：取 (b)**

它读 `/fmu/out/vehicle_local_position` + `/fmu/out/vehicle_attitude`，**换通路后同样死了**。

- **(a) 重指**到 `/mavros/local_position/*` + IMU 姿态 —— ⚠️ mavros 会**帮你转**（ENU→NED、FLU→FRD），
  "转两次"那个坑要重新过一遍（`px4-interface.md` §5.1）。**未被采纳**（理由见下）。
- **(b) 判为 uXRCE-DDS 专用、本阶段不可用**，并在 A 档入口加一条**显式拒绝**（fail loud，不是静默无数据）。
  ★★ **操作者裁定（2026-10-12）：取 (b)**，理由：本阶段"A 级回退"的载体是 **Phase 1 的 `control` 包整条链**，
  不是节点内部的 px4 状态源；把一条死路留着不如让它当场喊出来。

★ (a) 是**回退路径**、属安全相关件 ⟹ 这一项**已问过操作者**（不是 agent 自选），裁定原文见 Comments 末节。
⛔ 落地时**不许**把 (a) 顺手做进来 —— 若将来要做，另开票。

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
  "是 `fake_px4.py` 用到的"）**归 §四**（它要先判 `fake_px4.py` 的去留），
  本节**只改 nmpc 那一份**，⛔ 别顺手改 bench 的 —— 一节的改动别漫出这一节。
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
  （seam 1 用例本身的改写归 §四。）

---

## 三、阶梯驱动工具 `offboard_smoke.py` + `--t-max` 那笔账

**它挡的是什么**：这个工具是**阶梯 1/2 唯一的执行者**。
它错了，阶梯 1/2 的结论就是错的 —— 而 ② 那笔账**今天就错着**（与换不换通路无关）。
**谁做这一节**：纯软件 + 台架上就能跑（`--break` 那几档本来就是干这个用的），**不需要飞机通电**。★ 真机复核（拆桨目视那一步）属于 [09](09-ladder-ground-and-tether20.md)，**这一节不碰飞机**。

### ① 接口换掉（与 §二 同一套口径）

| 现在 | 换成 |
|---|---|
| pub `/fmu/in/offboard_control_mode` | **删掉** —— 心跳由飞控从收到的 `SET_ATTITUDE_TARGET` 派生（`mavlink_receiver.cpp:1531-1535`） |
| pub `/fmu/in/vehicle_rates_setpoint`（`VehicleRatesSetpoint`） | pub **`/mavros/setpoint_raw/attitude`**（`mavros_msgs/AttitudeTarget`），`type_mask = 128` |
| pub `/fmu/in/vehicle_command`（`:289` 解锁 / `:294` `DO_SET_MODE`） | **`/mavros/cmd/arming`** + **`/mavros/set_mode`** |
| sub `/fmu/out/vehicle_status` | sub **`/mavros/state`**（`armed` 且 `mode == "OFFBOARD"`） |
| `from px4_msgs.msg import …` | `from mavros_msgs.msg import AttitudeTarget, State` |

- **`_map()`**（现在调 `sign_map.px4_rates_setpoint()` 再写 `rates.thrust_body[2]`）改成调
  §二 里那个**具名的 mavros 入口**：`body_rate` 填 FLU 原样
  （⛔ 不许自己预转换 —— mavros 会转，见 `spec.md` §★★ N2），`thrust` 填 **`+F/T_max`**（飞控加负号）。
- **心跳时序**：现在的 `--pre-stream`（默认 1.5 s）是"先进 OFFBOARD 再解锁"那套流程里的等待。
  飞控侧仍要求 OFFBOARD 前先收到流，所以这个**语义大致保留**，但要按 mavros 的实际时序重核一遍
  （`/mavros/state` 是 **1 Hz** 上报 ⟹ **等它变成 OFFBOARD 要按秒算，不能按 100 ms 算**）。
  ★ 这是最容易"看起来能跑但判据恒真"的地方：断言"闸门已开"的那个轮询必须**长于 1 s 的周期**，
  否则它会**在飞控还没上报之前就判绿**（或恒红）。
- `_CHECKED_POSITIVE`（`--rate-hz` / `--cmd-interval` / `--pre-stream` / `--mass` / `--t-max`）**保留**。

### ② ★★ `--t-max` 那笔账：**默认值是错的**（与换通路无关，今天就错着）

**现状**：`Config.t_max_n = 11.8`。而 `sign_map` 与 `first-flight-config.md` **N4** 说得很清楚：
- **`t_max = 46.3027 N`** = 实测满量程 = `m·g / hover_thrust`（**归一化的基准**）；
- **`f_max = 11.8 N`** = 电池受限的 **F 上界**（**另一个角色**）。

**后果（算术，不是实测）**：`--mass 0.75` ⟹ 悬停力 `m·g = 7.3575 N`；
`7.3575 / 11.8 = 0.6235`，而正确值 `7.3575 / 46.3027 = 0.1589`（**= `hover_thrust`**）
⟹ 悬停那一档发的数是 **≈3.92 倍**（`0.6235 / 0.1589`）。
★★ **这一条直接砸在阶梯 2 的安全边界上**（系绳 + 离地 20 cm 只给悬停推力）：
`κ` 被顶到 ≈3.92 ⟹ 上飘加速度 ≈ **2.92g** ⟹ 系绳那 20 cm 余量**约 0.12 s 就打满**
（`0.20 = ½·2.92g·t²`，**这是算术，不是实测**）。
同时 [`ladder-acceptance-criteria.md`](../../../docs/control/ladder-acceptance-criteria.md)
§1.2 的放行带 `κ ∈ [1.0002, 1.043]` **必然落在带外** ——
判据 ② **会红，而且红得没有诊断价值**（它是在用 3.9 倍的推力去测那道夹逼）。
§1.2 那张 ★★ 表里已经列了"归一的基准 `--t-max`（默认 11.8 N）"是 κ 的三处误差源之一 ——
**本票就是去还这一笔**。

**怎么做**（三条都要；第 1 条的选法**已裁定**）：
1. ✅ **2026-10-12 操作者裁定：`--t-max` 改成"必须显式给"** ——
   `Config.t_max_n = 11.8` 这个默认值**删掉**，不给就**拒绝跑**（fail loud，⛔ 不是回落到某个数），
   帮助文本里写明**今天的正确值 46.3027 N 与它的来历**（= `m·g / hover_thrust`，`m = 0.75 kg`、
   `hover_thrust = 0.1589`），以及**它为什么会变**（它依赖悬停油门，而悬停油门正是
   未结票 [`02`](02-hover-thrust-backfill.md)（悬停油门回填）要改的量 —— 冻结一个现值当默认值，
   P2 一落地它就变成下一个静默错值）。
   ⛔ **不许**留一个会被静默用错的默认值：这条是本次裁定的**全部意义**。
   ★ 连带：阶梯 2 的**跑法块**（`ladder-acceptance-criteria.md` §1.2 `:126-129`）要跟着加上 `--t-max`
   —— 那一处**并入 §五 C 行**（同一份证据），不单独成行。
2. ★★ **加一条自检**：悬停那一档（`悬停倍数 = 1.0`）归一化之后，**必须等于悬停油门**
   （`m·g / t_max` 对 `hover_thrust` 的容差断言，容差用生产阈值口径，**不要写 `== 0`**）。
   **这条自检正是能把 11.8 这个错值当场变红的东西** —— 换通路之前也该有。
   **变异测试**：把 `--t-max` 改回 `11.8` ⟹ 必须变红。红不了就是没鉴别力。
3. **把"乘的基准"与"归一化常数"在输出里分开打印**（现在 `describe()` 打的是
   `F=…N (…×悬停)`）⟹ 再加一列"归一化后发出的是多少"，**让 ② 这类错误在屏幕上可见**。

### ③ 故障注入开关：**逐个**处置（⛔ 不许整段留下不动）

现在有几档 `--break` 与 `--timestamp-*` / `--qos`。换载体后**有的还有意义、有的没有对应物了**：

| 开关 | 新载体上还有意义吗 | 处置 |
|---|---|---|
| `--break thrust-sign` | **有**（仍会造出"全速砸地"那一档） | 保留 |
| `--break thrust-scale` | **有** | 保留 |
| `--break roll-sign` | **有** —— roll 不被 mavros 取负 | 保留 |
| `--break pitch-sign` / `--break yaw-sign` | **有，但含义变了**：这两档在新载体上**恰好等于"我们多做了一次预转换"**（= `spec.md` §★★ 的 **N2** 那个错） | 保留，但**改名或在帮助文本里写明它现在验的是 N2**（"两条通路各转一次"），**不是**"符号写反"。★ 同一个名字指另一件事，是操作者会误判的地方 |
| `--timestamp-unit`（us/ms/ns/zero）、`--timestamp-shift-us` | ⛔ **没有** | **删掉或显式标注为 dead** —— 理由见下 |
| `--qos` | **有**（收端 `SensorDataQoS` 是真的） | 保留，但**取值集合要按新载体重核**（现在那组是按 px4_msgs 那条流设计的） |

★★ **为什么时间戳那几档必须处置掉**（这一条是本票最容易被忽略的地方）：
`SET_ATTITUDE_TARGET` 的 `time_boot_ms` 在飞控那段 handler
（`mavlink_receiver.cpp:1508-1598`，**整个函数**）里**一次都没被读** ——
飞控自己用 `hrt_absolute_time()` 打时间戳（`:1593` 附近）。
⟹ 我们把 `header.stamp` 写成**零、毫秒、纳秒**，**注入效果逐位相同**。
**留着它的害处大于删掉它**：它会让操作者以为"时间戳这一档也测过了"，
而这一场里那个开关**再也取不到第二个值**。
★ 这属于"**仪器根本没给那个量变过**"那一族静默，不是"这档过了"。

**处置要留痕**：删了什么、为什么删、以及"这一档在新载体上没有对应物"这句话，写进 Comments 与
（若该工具在活文档里被描述过）相应文档。

### ④ 「悬停倍数」这一列的语义

- **表头与基准不变**：基准恒为 `m·g`（`commands.hover_force_n()`），
  `--mass` 仍是"全重 kg"。
- **但这一列下面那个数换了身份**：以前它最终变成一个 `thrust_body_z`（我们自己取负），
  现在它变成 `AttitudeTarget.thrust`（**正号**，负号由飞控加）。
  ⟹ ⛔ **换载体前后的"悬停倍数"读数不许并列比较**（见 `spec.md` §★★ N1）。
- 见 ② 第 3 条：把"归一化后发出多少"也打出来，这一列的语义才在屏幕上自洽。

---

## 四、夹具与 seam 1 —— 用 mavros 形状重跑

**它挡的是什么**：换载体之后的"**搬到真机后话题/符号/超时改错了**"这一类错误。
★ 本票是全套里**唯一**能在台架上把 N1/N2/N3 三处**逐条证伪**的地方 ——
真机阶梯只能证明"没摔"，证明不了"我们发的那一条消息里 `body_rate` 的值是对的"。
**谁做这一节**：纯台架，不需要飞机通电、不需要起链。★ 验收最硬的一节。

### ① 夹具：`src/bench/bench/fake_mavros.py`（Phase 1 已有，**加一条流**）

它已经是"假飞控 + mavros 形状"（`/mavros/state`、`local_position`、`setpoint`、`set_mode`、`arming`…，
`:644` 行，模块 docstring 里已经逐话题写清了 mavros 的真实 QoS）。本票加的是：

- **新订阅 `/mavros/setpoint_raw/attitude`（`mavros_msgs/AttitudeTarget`）**，
  QoS 用**真的那个** —— 即 `SensorDataQoS()` 那一档（best_effort / keep_last 5 / volatile，
  出处 `~/mavros_ws/src/mavros/mavros/src/plugins/setpoint_raw.cpp:62`），
  ⛔ **不要**图省事用默认 QoS：那会把"QoS 不匹配"这一类错**静默放过**，
  而它正是本 seam 要挡的（`mpc-node-seam1.md` §… 的 QoS 用例）。
- **★ 逐条记录我们发出去的字段原值**（`type_mask` / `body_rate.x/y/z` / `thrust` / `header.stamp`），
  和现在记 `Setpoint(at_s, msg)` 一样的做法，并进 `evidence()`。
  ★★ **这是本票的关键设计**：**N2 只有当断言落在"我们发出的那个 `body_rate` 值"上时才抓得到**。
  若断言落在"最终飞控收到的物理方向"上，**两次取负看起来和正确的一模一样** ——
  ✋ 见 [判据全绿也可能是机理反了] 那一族。
- `set_mode` / `arming` 两条既有流：确认它们能被 `/mavros/cmd/arming` + `/mavros/set_mode` 触发
  （Phase 1 的 `follow_arming` / `follow_set_mode` 应当已经够用，**先核再改**）。
- `/mavros/state` 的发布：**用 `StateQoS()`**（reliable + **transient_local**，`sys_status.cpp:583`），
  按 **1 Hz** 走（`sys_status.cpp:962-985`）。
  ★ transient_local 的副作用要写进测试注释：**后加入的订阅者会立刻拿到最后一条 state**
  ⟹ 起机那一刻不会因为"等 1 s"而空转；但**稳态**仍受 1 Hz 约束
  （这正是 §二 ③ 那个超时坑的由来）。
  ⛔ 若夹具图快把它做成 10 Hz，那个坑在台架上就**永远不会出现**。

### ② `src/bench/bench/config.py`

- `DEFAULTS["topics"]` 加/改：`/mavros/setpoint_raw/attitude`；
- `DEFAULTS["rates_hz"]` 里 `state` 已是 `1.0`（**对**，别改快）；
- `DEFAULTS["px4"]["topics"]` 那组（`status` / `rates` / `mode`）：**本票要判它的去留**（见 ④）。

### ③ `src/nmpc/test/test_mpc_node.py`（2125 行 / 41 条；**约 25 条**依赖 `/fmu/*`）

**做法**：不是"删掉 25 条"，而是**逐条映射**（映射表本身就是交付物，写进
[`mpc-node-seam1.md`](../../../docs/control/mpc-node-seam1.md)，体例同它的 §11.3）：

| 类别 | 处置 |
|---|---|
| **话题名 / QoS**（接口那一节） | **逐条换**成新话题与新 QoS（含"QoS 不匹配就收不到"的负控） |
| **符号 + 推力归一化**（本节） | ★★ **重写断言对象**：断言落在**我们发出的那条消息**的 `body_rate` 与 `thrust` 上（见 ① 最后一条）。**必须有一条专测 N2**：断言 `body_rate` 是 **FLU 原样**（`pitch/yaw` **没有**被我们预先取负） |
| **闸门**（未解锁 / 未在 OFFBOARD / status 过期 ⟹ 不发布） | 数据源换成 `/mavros/state`；★ **超时那条必须变异测试**过（改回 `0.75` ⟹ 变红） |
| **时间戳**（那一节） | ⛔ **本类在新载体上没有对应物**：飞控不读 `time_boot_ms`（`mavlink_receiver.cpp:1508-1598` 整个 handler 里没读）⟹ **删除或标记为"不再可测"**，并在映射表里写明**为什么**。**不许留成"绿着但不测任何东西"** |
| **频率与 RViz**（那一节） | 按 §二 ④ 选定的心跳方案改（收成单频 / 双频重解释） |
| **A 档**（`state_source="px4"`） | 按 §二 ⑤ 的裁定改：选 (b) 的话断言"入口当场拒绝" |
| **求解失败兜底 / NaN 整帧丢弃 / 进程收尾** | **语义不变**，只换消息类型 |
| **轨迹参考 / 输入合理性** | **不动**（与下行无关） |

★ `FROZEN_PARAMS`（`tau_s=0.0`）等冻结档**保留**；起参数里与下行相关的（若有）同步换。
★ `DUT = ("nmpc", "mpc_node.py")` 与 `Scenario(dut=…, px4=…)` 的**开关语义**要改：
现在 `px4=True` 起的是 `fake_px4`。**新增一个 mavros 档位**（而不是把 `px4=True` 偷偷解释成 mavros）——
⛔ 名字撒谎比多一个参数贵。

### ④ `src/bench/bench/fake_px4.py` 的去留 —— **显式判，不许留着不管**

★★ **已有的一手痕迹**：`src/bench/package.xml:27` 的 `<depend>px4_msgs</depend>` 旁边就写着
"★ 假 PX4（`bench/fake_px4.py`）用到的 `/fmu/*` 消息类型"（`:24`）
⟹ **那条依赖的存在理由就是 `fake_px4.py`**。
若 `fake_px4.py` 被判"没有 DUT 了"，**这条依赖与那条注释要一起处置**（⛔ 别只留一个空依赖）。
（同一份 `package.xml` 的 `:23` 已经有 `mavros_msgs` —— **不用新加**。）

判据：**还有谁的 DUT 说 px4_msgs**？
- 若 `tools/px4-sitl/`（tier A/B 判据）+ 上游链路**还在用**它 ⟹ **保留**，并在它自己的文档/注释里
  写明"**只服务那条链**，不再服务模型预测控制节点"。
- 若**没有任何 DUT 用它** ⟹ 在 Comments 里写明"**它现在没有 DUT**"并归入待裁，
  ⛔ **不要**因为"反正没人看"就默默留着 —— 一个没有 DUT 的夹具会让后来者以为
  "px4_msgs 那条下行还是活的"。

### ⑤ 文档：`docs/control/mpc-node-seam1.md` 追加一节

按它的既有体例（§11 的形态）追加 **§12.4 或 §13**：这次跑的是什么、映射表、读数、
**本节没证明什么**（照 §12.4 的口径写）。
★ §11.1 冻结档表与 §11.2"可整段复制的命令"若被 §二 或本节改动
（**现在这两节在同一张票里，但落地的先后仍是 §二 → §四**），
**两处都不许留悬空** —— 后落地的那一节先把这两处核一遍（**先核再改，别各改一半**）。

★ **"本节没证明什么"里要新增一条**：**`MAV_FWDEXTSP`（硬前置，=1 才收 setpoint）夹具模拟不了**
—— 它是**真飞控的参数**，台架上没有它的对应物 ⟹ 这一条**只能在实飞前检查清单里兜**
（归 §一 ③ / §二 ⑤bis）。
⛔ 别让夹具"顺手"把它模拟掉 —— 那会把一条**真实存在的静默失效点**变成一个绿的用例。

---

## 五、阶梯判据页 §7.1 留痕

**谁做这一节**：**★ 这一节不需要飞机通电** —— 它是**纸面**工作，桌上就能做。（★ 但**不要**在真机阶梯 1 之后才做：留痕的意义正是在"下一次照着敲"之前把它做掉。）

### 为什么是 `ready-for-human`

三条，缺一条都不够：

1. **§7.2 的"谁定的"列要操作者签字** —— §7.0 明写阈值是"**操作者的取舍（多保守算够保守）**"，
   而已定稿的页面的改动**只能由定稿人认**。
2. ★★ **其中一行有安全后果**：`--t-max` 默认值那笔账（见下 **C 行**）算出来是 **≈3.92 倍悬停推力**，
   而它砸在**阶梯 2（系绳 + 离地 20 cm 只给悬停推力）**上 —— 这是全班里唯一会**让飞机以数倍悬停推力往上挣**的改动。
3. **"一行只能由一个证据促成"**（§7.0 条 3）：到底拆成几行、每行拿哪个数据当锚，
   是**判据体系内部**的取舍，不是执行细节。

★ 正文的**留痕文本**（☠️ 标哪里、§7.2 各行怎么写）**agent 可以先起草**；
⛔ 但**本票的 `Status` 只能由操作者切**。

★★ **2026-10-12 操作者已裁定**：**拆成四行 —— A / B / C / E**；
**D 行不单独成行，并入 C 行**（阶梯 2 的跑法块与 `--t-max` 那笔账是**同一份证据**促成，
§7.0 条 3 的"一行只能由一个证据促成"读作**一个证据一行、那一行里可以点到多处位置**）。
下面的 D 行小节保留原文（它是这个裁定的来由），但**不再产生独立的一行**。

---

### 要留痕的地点（逐处点名 —— 排序就是可见性）

#### A 行：阶梯 1 的**驱动工具**（信号载体）

| 位置 | 现在写的是什么 |
|---|---|
| **§1.1 程序段**（`ladder-acceptance-criteria.md:57-59`） | "用 `offboard_smoke.py`…**它错了就只可能是中间件、符号或归一化错**" |
| **§1.1 命令块**（`:61-67`） | `ros2 run nmpc offboard_smoke.py --phases roll+:3:1.0:+1.0,0,0 …` |

**促成的数据**：真机参数读数 —— `UXRCE_DDS_CFG = 0`（[`z0.params:949`](../../../docs/assets/z0.params)）、
`MAV_1_CONFIG = 102`（[`:534`](../../../docs/assets/z0.params)，那条串口被 mavros 占）。
★ **如实写：这是参数读数，不是飞行读数** —— ⛔ 不许把它写成"某次飞行的实测"。

**处置**：命令块换成新接口；`:57` 那句"只可能是中间件/符号/归一化错"的**论断要重核一遍**：
走 mavros 之后，多了一个**别人的中间件**（mavros 自己）与**飞控的一行**（`fill_thrust`）——
归因环从三条变成五条了吗？★ 这正是 §1.1 那一句要诚实回答的地方，**别只改命令块**。

#### B 行：阶梯 1 那张表的**中间一列**整列作废（**第三列不动**）

| 位置 | 现状 |
|---|---|
| **§1.1 表头**（`:77`） | "smoke 节点实际发出的 **PX4 字段**（sign-map.md §2）" |
| **§1.1 表体**（`:79-81`） | `roll_rate = +` / `pitch_rate = −` / `yaw_rate = −` |

**促成的数据**：同 A 行（换载体）—— 但要**分开成一行**，因为它废的东西不同：
A 行废的是"跑法"，B 行废的是"**我们往线上放的那个数**"。

**处置**：中间一列 ☠️，换成新载体上实际发出去的字段
（`body_rate.x/y/z` 与标量 `thrust`；**pitch/yaw 不再是负号**，负号由 mavros 那侧做；
`thrust` 是**正号**，负号由飞控 `fill_thrust` 做 —— 见 `spec.md` §★★ 的 N1/N2）。
★★ **第三列（物理预期：哪两个加速、哪两个减速）一个字不改** ——
**它才是判据**（操作者看的正是电机转速差方向），而且它是**纯物理**、与走哪条通路无关。
⛔ 别把整张表连根换掉：那样会把"当初为什么是这几行"一起擦掉。

#### C 行：★★ `--t-max` 那笔账 —— **本票里唯一有安全后果的一行**

| 位置 | 现状 |
|---|---|
| **§1.2 κ 的三处误差源之一**（`ladder-acceptance-criteria.md:183`） | "**归一的基准** `--t-max`（**默认 11.8 N**，config 页 **N4** 的 `f_max`）" |

**现状的事实**：`11.8 N` 是 `f_max`（电池受限的 **F 上界**），
而**归一化的基准**应是 `t_max = 46.3027 N`（= `m·g / hover_thrust`，config 页 **N4**）。
⟹ 默认值算出来的悬停档是 **`7.3575/11.8 = 0.6235`**，正确值是 **`7.3575/46.3027 = 0.1589`**（= `hover_thrust`）
—— **≈3.92 倍**。

**促成的数据**：★ **算术 + 两个已实测的常数**（`t_max = 46.3027 N`、`hover_thrust = 0.1589`）
—— **没有飞行数据作锚**，这一点**必须在 §7.2 的"促成的数据"列如实写出来**。
（对比：§7.2 现在那两行的锚分别是五份包的现算读数与"0.90 s 触地"的算式。）

**处置**：`11.8 N` 那处 ☠️，并把 §三 ② 定下来的新口径写进去。
★ **从严**：虽然这一处看起来是"工具的一个默认值"，但它**改变飞机实际收到的推力**，
而 §1.2 的四条判据**全部以"我们发的推力"为基准** ⟹ **按 §7.1 走，不按"顺手修 bug"走**。

★★ **节首要挂作废横幅**：§1.2 的那张 ★★ 误差源清单**整段**受这一行影响
（它列的第 2 条换了值），横幅按 §7.0 条 2 写在**节首**，并**逐项点名**。

★★ **本行里要一并点到第二处位置**（2026-10-12 裁定：D 行并入本行，不单独成行）：

| 位置 | 现状 |
|---|---|
| **§1.2 跑法块**（`:126-129`） | `ros2 run nmpc offboard_smoke.py --mass <全重kg> --phases lift:3:1.0:0,0,0 …` |

它受同一笔账影响：§三 ② 已裁定 **`--t-max` 改成"必须显式给"** ⟹ 这个跑法块
**不给 `--t-max` 就跑不起来**，必须补上（写今天的正确值 46.3027 N 与它的来历）。
⛔ **不是再加一行** —— 一个证据一行，那一行里可以点到多处位置。

#### D 行：阶梯 2 的**命令块** —— ✅ **2026-10-12 裁定：并入 C 行**

（本节保留原文，作为那个裁定的来由；**不再产生独立的一行**，
具体那处位置见上面 C 行末尾的第二张表。）

★ 原来的条件句是"若 §三 选了'`--t-max` 必须显式给'才独立成行" ——
**裁定取"必须显式给"，但拆行取四行**：证据是同一份（`--t-max` 那笔账），
所以并进 C 行；§7.0 条 3 的"一行只能由一个证据促成"读作"**一个证据一行、一行可点多处**"。

#### E 行：阶梯 3 判据 ⑤ 的**读数来源**

| 位置 | 现状 |
|---|---|
| **§1.3 判据 ⑤**（`:209`） | "全程 `nav_state = OFFBOARD(14)` 且 `arming_state = ARMED(2)`，**一次都不掉出**"（括注指向 `heartbeat_always`） |

**促成的数据**：同 A 行（换载体）。
**处置**：读的**量**没变（"全程在 OFFBOARD 且已解锁"），**读的地方**变了
（`/mavros/state` 的 `armed` 与 `mode`）⟹ 判据 ☠️ 后就地换源。
★ 括注里那个 `heartbeat_always` 的语义在 §二 ④ 里被重新定义了
⟹ **同步**（⛔ 别留一个指向已改名的旋钮的括注）。

#### F 行：**不动的**（明写出来，免得被"顺手一起改"）

- §1.1 的**物理预期列**与"这三行是怎么来的"（`ω = r × F`、`KM` 那一串推导）—— **纯物理，不动**；
- §1.1 的 ⚠️ "物理口 → 机体角要现场落实"（`:103-107`）与 §7.2 底部那一行（`:536`）—— **它归 §六**，
  **本节 ⛔ 一行都不碰**（★ 两节现在同票，但"谁改哪一行"照样分开：本节只动"驱动方式 / 字段列 / 读数来源"，
  那两处等 §六 的实测结果回填）；
- §1.1 `MC_*RATE_MAX` 那些读数、§1.2 ③ 的对照量级、§1.2 ① 的分诊表、
  §1.3 的 0.15 m / 0.10 m / 10 ms 各线、§1.4 全部、§2 的 (A) 级基线表、§3 五条归因环、§5 停飞回退 ——
  **一个数都不改**（本票只动"驱动方式 / 字段列 / 读数来源"这三类）。

---

## 六、独立项：用 QGC 执行器测试落实「物理口 ↔ 机体角」

**谁做这一节**：1. **它要动飞机**（通电、点电机）—— 而 **车侧/飞机侧动作由操作者亲手做**，agent 只报时机与读数，⛔ 不代跑；2. 判据里唯一需要"**眼睛**"的部分（**哪个口在转**）**没有自动化替代物** —— 飞控能把 `function` 值报出来（`listener`），但"哪个插头在转"只有人看得到。★ 这一节的**推导部分**（上面那条链）已经做完且是第一手的；剩下的是**上机验一眼**。**别把它和 [09](09-ladder-ground-and-tether20.md) 的阶梯 1 混成一件事** —— 阶梯 1 要用这张表，所以这张表**必须是阶梯 1 之前**的第一步。

### 为什么这张表现在还是空的

阶梯 1 的判据是"**目视电机转速差方向**"，而那条判据的前提是
"**我知道哪个物理口是哪个角**"。现在这条前提只有一半：

- **角点**是有的（[`ladder-acceptance-criteria.md`](../../../docs/control/ladder-acceptance-criteria.md) §1.1 `:83-100`，从
  `CA_ROTOR0..3_PX/PY` 与 `AZ = −1` 推出：**0 = 前右、1 = 后左、2 = 前左、3 = 后右**）；
- **物理口**是空的 —— §1.1 `:103-107` 明写"**上表是按角点写的，现场要确认的正是哪个角是哪个物理口**"，
  且这一行已经被登记进 §7.2 底部那张"**已登记、未处置的相邻事项**"表（`:536`）。
- ⚠️ **而且 `PWM_MAIN_FUNC1..4 = 103, 102, 101, 104` 是刻意改过的非默认电机序**
  （[`params-changes.md`](../../../docs/reference/params-changes.md) **D7**），
  ⟹ **"输出口 3" 驱动的才是 Motor1**，绝不能按"Motor n → 输出 n"想当然。

**不做这一步，阶梯 1 的六段就是在一张错位的表上判方向。**

★★ **2026-10-12 操作者裁定：排在阶梯 1 之前、单独做一次场次**（不与阶梯 1 合并到同一次通电里）。
理由：它是阶梯 1 那张表的前提；单独一次 = 拆桨 / 未解锁 / 逐个点动 / 目视哪个口在转，
**观察量干净**，也不必在阶梯 1 的场次里赶（一旦四行里出现不一致，当场就得停下查归因，
而那种"停下"夹在阶梯 1 的流程里最难处理）。

---

### 已经推出来的部分（**第一手，链条完整**）

| 机体角 | 转子序号 | 电机序号 | 物理输出口 |
|---|---|---|---|
| **前右** | `CA_ROTOR0` | **Motor1** | **MAIN3** |
| **后左** | `CA_ROTOR1` | **Motor2** | **MAIN2** |
| **前左** | `CA_ROTOR2` | **Motor3** | **MAIN1** |
| **后右** | `CA_ROTOR3` | **Motor4** | **MAIN4** |

**每一列怎么来的**：

1. **`PWM_MAIN_FUNC1..4 = 103, 102, 101, 104`**（`docs/assets/z0.params:699-702`；
   同值见 `bag/z0-params-2026-10-10.params:663-666`）。★ 这四个数是**输出功能 id**，
   而 `Motor` 这一族 id 从 **101** 起、共 12 个（`~/refs/px4-v1.14.3-build/src/lib/mixer_module/output_functions.yaml`）
   ⟹ `101 = Motor1`、`102 = Motor2`、`103 = Motor3`、`104 = Motor4`
   ⟹ **输出口 1 挂 Motor3、口 2 挂 Motor2、口 3 挂 Motor1、口 4 挂 Motor4**。
2. **转子序号 ↔ 电机序号**：`CA_ROTORn`（n 从 0 起）与 **Motor(n+1)** 直接对应 ——
   分配器按**下标顺序**读 `CA_ROTOR%u_PX/PY/…/KM` 并依次登记执行器
   （`.../src/modules/control_allocator/ActuatorEffectiveness/ActuatorEffectivenessRotors.cpp:54-77`；
   且 **v1.14.3 的 `src/` 里查不到 `MOT_ORDERING`**，`grep -rn 'MOT_ORDERING' src/` 零命中）。
3. **角点**：§1.1 `:85-88`（`CA_ROTOR0_PX/PY = +0.15/+0.15`、`1 = −/−`、`2 = +/−`、`3 = −/+`）。

★★ **但上表的第 3 列与第 4 列是"读参数读出来的"** ——
**本票的全部意义就是把它换成"看着它转出来的"**。⛔ 上表**不是**结论，是**待验的假设**。

---

### 怎么做（`MAV_CMD_ACTUATOR_TEST`，QGC 执行器测试页）

**前置**（全在 `docs/assets/z0.params` 现读）：

| 条件 | z0 现值 | 出处 |
|---|---|---|
| **未解锁**（飞控自己拒） | —— | `Commander.cpp:1433`（`_arm_state_machine.isArmed()` ⟹ `DENIED`） |
| `COM_MOT_TEST_EN = 1` | **1** ✓ | `z0.params:286`；飞控侧判据 `Commander.cpp:1437` |
| `CA_AIRFRAME = 0`（多旋翼） | **0** ✓ | `z0.params:115` |
| `CA_ROTOR_COUNT = 4` | **4** ✓ | `z0.params:226` |
| 安全开关（若有）必须已解除 | —— | `Commander.cpp:1433`（`_safety.isButtonAvailable() && !isSafetyOff()` ⟹ `DENIED`） |

**实测方式**：QGC → 执行器测试，逐个点动，**目视哪个物理插头在转**。
命令落到飞控的哪个字段（**第一手**，`Commander.cpp:1431-1469`）：

- `param1 = value`（`actuator_test.value = cmd.param1`，`:1463`）；
- `param2 = timeout`（**秒**，`:1465` 乘 1000；`≤ 0` ⟹ `ACTION_RELEASE_CONTROL`，
  且**上限被强制到 3000 ms**，`:1476-1478`）；
- `param5` = **`actuator_test.function`**（`:1442`）—— 若 `< 1000`，
  按 **MAVLink 的 `ACTUATOR_OUTPUT_FUNCTION`** 编号解释：**1..32 = 电机**
  （映射成 PX4 的 `FUNCTION_MOTOR1 + n − 1`，即 **1→Motor1=101**）、**33..64 = 舵机**；
  若 `≥ 1000` 则 `−= 1000`（直接用 PX4 的内部 id）。

★★ **一处本票开票时没核、必须现场钉住的**：**QGC 那个页面实际送的 `param5` 是哪个编号**
（电机序号 1..4？输出序号 1..4？还是 PX4 的 101..104？）—— **QGC 不在本仓库里，也没有读过它的源码**。
**不许猜。** 用飞控控制台看真值：

```
listener actuator_test      # 每次点动时看 function 落到哪个数
```

期望看到 **101 / 102 / 103 / 104**（= Motor1..4）。**若看到的是别的数，先把它记下来再判断** ——
本票的结论依赖"哪个电机真的被点着了"，不依赖"QGC 上那个标签叫什么"。

---

### 要填的表（本票的主交付）

> ★ **默认拆桨**（和阶梯 1 同一纪律）。只量电机转向，**不装桨**。

| QGC 点动项 | `listener` 看到的 `function` | 哪个物理口在转（MAIN1..4） | 上位机上的位置（前左/前右/后左/后右） | 转向（CW/CCW） |
|---|---|---|---|---|
| 1 | | | | |
| 2 | | | | |
| 3 | | | | |
| 4 | | | | |

**然后反核上表**：四行都要与"已推得的链条"一致；
**任一不一致 ⟹ 停**（⛔ 不许靠调参数去把它凑一致 —— 不一致这件事本身就是结论，
它可能是接线、参数、或我这条推导链里某一环错了，三种归因完全不同）。

**顺手一并记（很便宜，但要说清它不是什么）**：
- **转向**：`CA_ROTOR0/1_KM = +0.05`、`CA_ROTOR2/3_KM = −0.05`
  （`z0.params:134 / :158 / :166 / :174`），而 PX4 自己的文档写着
  "**Use a positive value for a rotor with CCW rotation. Use a negative value for a rotor with CW rotation.**"
  （`.../control_allocator/module.yaml:211-214`，第一手）
  ⟹ **期望：Motor1 / Motor2 一个转向、Motor3 / Motor4 另一个转向**。
  ★ **但这不是 KM 的判据**：KM 的判据是**阶梯 1 的 yaw 那一行**（推力起来之后才有效）。
  这里只是**提前看一眼**，不符时**先记录**，⛔ **别当场改 `CA_ROTORn_KM`**。
  （PX4 那两句没写视角 —— 实操按"从机身上方俯视"理解；现场判不出就**原文照抄**，别自己补一个视角。）
- **桨叶正反**（不转，纯目视）：正桨/反桨装错**在拆桨状态下看不出来**，
  顺手核对每个电机上桨的标记与它的**转向**是否配对。★ 这一条同样**只记录**。

---

### 写回哪里（三处，**都不在本节的行文范围里 ⟹ 本节只出"填什么"，落位各有其主**）

| 位置 | 改什么 | 归属 |
|---|---|---|
| `ladder-acceptance-criteria.md` §1.1 的 ⚠️（`:103-107`） | 把"现场要确认"换成**填好的表**（☠️ 原 ⚠️ 就地标掉，按 §7.1 在 §7.2 增一行） | ★ 见下 |
| 同页 §7.2 底部"已登记、未处置的相邻事项"表（`:536`） | 把那一行标为**已处置** | ★ 见下 |
| [`params-changes.md`](../../../docs/reference/params-changes.md) **D7** | 它自己写着"仍需现场确认'物理口→机体角'的完整映射" ⟹ 补上实测结果 | D7 的归属票（**不是本票**） |

★★ **与 §五 的分工要写死，别两节同时改 §7.2 的同一行**：
- **§五 ⛔ 不碰** §1.1 的 ⚠️ 与 §7.2 底部那一行（它在 F 行里已明写）；
- **本节**负责把实测结果落进这三处 —— ★ 落的仍然是**纸面**：`ladder-acceptance-criteria.md` 与
  `params-changes.md` 的改动由操作者（或带 §7.1 留痕的收口步骤）落笔，
  ⛔ **不是"点完电机就自动改好了"**。
- 与 §五 那几行**并存是对的**（一行一个证据）：§五 那一行记的是"驱动方式/字段列/读数来源换了"，
  本节这一行记的是"**哪个口是哪个角，看着它转出来的**" —— 两件事，两个锚。

---
## 验收

### 一、口径冻结（docs 先行，纯文档）

- [ ] `sign-map.md` §2 的表有"谁执行"一列，且 mavros 那一路的两处**都指向别人的代码**（带文件:行号）
- [ ] `sign-map.md` 新增的所有权对照节，px4_msgs 与 mavros 两条路**逐行对照**，锚点齐全
- [ ] `sign-map.md` §2 已分成"物理约定（不变）"与"落到线上那几个字段（按通路分叉）"两半
- [ ] §9 那四件自检的**适用范围**被写清；三条反例**没有被删**，而是被重新论证成"所有权转移的安全性证明"
- [ ] §8 第 5 条契约的落点已按新载体重新表述
- [ ] `mpc-node-seam1.md` §12.3 ① ② ③ **逐条**有 ☠️ 或改写，**原文可读**（能看出改前是什么）
- [ ] `px4-interface.md` §4.3 从"开放决策"变"定案"，带日期与起因；§5.1 多一行 `setpoint_raw/attitude`
- [ ] `px4-interface.md` §8 的两条检查项**就地标 ☠️**（不是删除），且该节其余条**未被顺手降级**
- [ ] §8 **新增了 `MAV_FWDEXTSP = 1` 这一条**（它是硬前置：为 `0` 时整条下行静默死掉）
- [ ] 活文档里"下行 = px4_msgs / `/fmu/*`"的位置**有一张登记清单**（路径 + 行号 + 归哪一节），**无悬空引用**
- [ ] 全程没有改任何 `.py` / `.xml` / 参数文件（`git status` 里只有 `docs/**` 与 `.scratch/**`）

### 二、节点换下行 —— 话题 / 消息 / 闸门源 / 超时 / 依赖

- [ ] `mpc_node.py` 里**没有任何** `/fmu/*` 话题常量与 `px4_msgs` import（用 grep 现算，不靠肉眼）
- [ ] 下行发的是 `/mavros/setpoint_raw/attitude`，`type_mask = 128`，`body_rate` 是 **FLU 原样**（未预转换）
- [ ] `thrust` 发的是 **`+F/T_max`**，且这一处**在代码里能读出"飞控会加负号"**（注释带锚点）
- [ ] `sign_map.py` 有两个**具名**入口（px4_msgs 那条与被 mavros 用的那条），字段名各自自证，**没有**布尔开关
- [ ] 闸门读 `/mavros/state`，判据是 `armed` 且 `mode == OFFBOARD`；`mode` 的**字符串有第一手出处**（写进 Comments）
- [ ] `status_timeout_s` 已按 1 Hz 发布周期改大，且**有一条断言**护住；**变异测试**把值改回去 **必须变红**
- [ ] `heartbeat_hz` / `heartbeat_always` 的去留**显式选了 (a) 或 (b)** 并写进 Comments，没留隐含行为
- [ ] A 档：按 2026-10-12 裁定取 **(b)** —— 入口**当场拒绝**（fail loud，不是静默无数据），
      ⛔ 没有顺手把 (a) 做进来
- [ ] `src/nmpc/package.xml` 里的 `px4_msgs` 已撤（`mavros_msgs` **本来就有**，不是新加的）；README 的 source 行同步；**bench 的没动**
- [ ] `__init__.py` / `nmpc/README.md` 那句"不许 import"改成了与新事实一致的表述（**不是删除**）
- [ ] `preflight_check.py` 已核并处置（或明确记为"本票范围外 + 归哪一节"），
      且**含 `MAV_FWDEXTSP = 1` 这一条**（为 `0` 时整条下行静默死掉）
- [ ] Comments 里记了"开票时 `MAV_FWDEXTSP = 1`"（将来若变 `0`，有据可查是**被改过**）
- [ ] `mpc-node-seam1.md` §11.1 / §11.2 里被本票改掉的旋钮已同步（⛔ 不留读起来像能照抄的命令）

### 三、阶梯驱动工具 `offboard_smoke.py` + `--t-max` 那笔账

- [ ] 工具发的是 `/mavros/setpoint_raw/attitude`，**不再**发任何 `/fmu/*` 话题（grep 现算）
- [ ] 解锁走 `/mavros/cmd/arming`、切模式走 `/mavros/set_mode`，闸门读 `/mavros/state`（`armed` + `mode`）
- [ ] `body_rate` 是 **FLU 原样**（⛔ 无预转换），`thrust` 是 **`+F/T_max`**
- [ ] **等闸门的那段轮询周期 ≥ `/mavros/state` 的发布周期**（1 Hz），⛔ 不是 100 ms 级的
- [ ] `--t-max` **已无默认值**（不给就拒绝跑，⛔ 不是回落到某个数），帮助文本写着今天的正确值
      `46.3027 N` 与它的来历（`m·g / hover_thrust`）以及"它为什么会变"
- [ ] 有"悬停档归一化后 == 悬停油门"的**自检**；**变异测试**把 `--t-max` 改回 11.8 **必须变红**
- [ ] 输出里"乘的基准"与"归一化常数"**分开可见**（② 那类错误在屏幕上看得见）
- [ ] `--timestamp-*` 那几档**已处置**（删或标 dead），且 Comments 里写明理由（飞控不读 `time_boot_ms`）
- [ ] `--break pitch-sign` / `yaw-sign` 的含义变化**在帮助文本里写清**（= N2，不是"符号写反"）
- [ ] `--qos` 的取值集合已按新载体重核
- [ ] 需要 §7.1 留痕的地点已整理成一份清单（哪一级 / 哪一行 / 为什么 / 换成什么），交给 §五
- [ ] **没有**改 §二 的落点（`mpc_node.py` / `node_core.py` 一行不碰）

### 四、夹具与 seam 1 —— 用 mavros 形状重跑

- [ ] `fake_mavros.py` 能收 `/mavros/setpoint_raw/attitude`，**QoS 与 mavros 真实取值一致**（不是默认档）
- [ ] 夹具**逐条记录**我们发出的字段原值，并能在 `evidence()` 里读到
- [ ] `/mavros/state` 走 `StateQoS()` + **1 Hz**；transient_local 的副作用写在注释里
- [ ] seam 1 全套跑过，话题名/QoS/符号/归一化/闸门**逐条**有对应用例，映射表进文档
- [ ] ★ 有**专测 N2** 的用例：断言 `body_rate` 是 FLU 原样（pitch/yaw **未**被我们预先取负），
      且**变异测试**过（故意在 DUT 侧预转换一次 ⟹ **必须变红**）
- [ ] 闸门超时那条的**变异测试**过（参数改回 `0.75` ⟹ 变红）
- [ ] 时间戳那一类用例**已删除或标为不可测**，并在映射表里写明理由（⛔ 无"绿着但不测东西"的残留）
- [ ] `config.py` 的话题/频率同步；`px4=True` 的语义**没有**被偷换
- [ ] `fake_px4.py` 的**去留有明确结论**（带判据），不是"留着不管"；
      若判为没有 DUT，`src/bench/package.xml:27` 的 `px4_msgs` 依赖与 `:24` 那条注释**一并处置**
- [ ] `mpc-node-seam1.md` 追加了新节，含"本节没证明什么"、
      **且其中明写 `MAV_FWDEXTSP` 这条硬前置夹具覆盖不到**
- [ ] §11.2 那条"可整段复制的命令"**与现状一致**（⛔ 不留读起来像能照抄的命令）

### 五、阶梯判据页 §7.1 留痕

- [ ] §7.2 **新增行**：A / B / C(+D) / E 逐条有行，每行五列齐全（日期 / 定稿？ / 改了什么 / 促成的数据 / 谁定的）
- [ ] **"促成的数据"列如实**：A·B·E 写"真机参数读数（`UXRCE_DDS_CFG = 0` / `MAV_1_CONFIG = 102`）"，
      **C 行明写"算术 + 两个已实测常数，无飞行数据作锚"**
- [ ] A 行：§1.1 的程序段与命令块已 ☠️ + 换新；★ 那句"只可能是中间件/符号/归一化错"**重核过**
- [ ] B 行：中间一列 ☠️ + 换新（`body_rate` + `thrust`，符号按 N1/N2）；**第三列一个字没改**
- [ ] C 行：`11.8 N` 那处 ☠️，新口径写入；**§1.2 误差源清单的节首挂了作废横幅并逐项点名**
- [ ] D 行：按 2026-10-12 裁定**并入 C 行**（不单独成行），且 C 行里**点到了 §1.2 跑法块那一处**
      （跑法块补上 `--t-max`，否则跑不起来）
- [ ] E 行：§1.3 ⑤ 换源；`heartbeat_always` 的括注已与 §二 ④ 同步
- [ ] F 行那批**逐条核过没被动**（用 grep 现算，不靠肉眼）
- [ ] §7.2 底部"已登记、未处置的相邻事项"表里 §1.1"物理口→机体角"那一行**没有被本票处置**（归 §六）
- [ ] **没有**改任何阈值、任何 `.py`、任何参数文件
- [ ] `谁定的` 列有操作者签字

### 六、独立项：用 QGC 执行器测试落实「物理口 ↔ 机体角」

- [ ] 四行表**填满**，每行都有"`listener actuator_test` 看到的 `function` 值"（⛔ 不是"QGC 上写的是几"）
- [ ] `param5` 的编号空间**有第一手结论**（101..104？还是 1..4？写清核法）
- [ ] 四行与"已推得的链条"**逐行核对**：一致 / 不一致（不一致 ⟹ 停，并写归因）
- [ ] 转向逐条记录（**并写明它不构成 KM 的判据**）
- [ ] 桨叶正反目视记录（不转状态下）
- [ ] 全程**拆桨**、**未解锁**（★ 车侧动作由操作者亲手做）
- [ ] 按 2026-10-12 裁定**单独做一次**、早于阶梯 1 的场次（⛔ 没有把它并进阶梯 1 那次通电里）
- [ ] 结果写进 §1.1 的 ⚠️（☠️ + §7.2 增行）、§7.2 底部登记表那一行标为已处置、`params-changes.md` D7
- [ ] **没有**改任何飞控参数（`CA_ROTORn_KM` / `PWM_MAIN_FUNC*` 都**只读不写**）
- [ ] **没有**碰 §五 的落点（下行载体那几处）

---

## Comments

### 2026-10-12 —— 开票时已核过 / 未核（原六票合并）

#### 一、口径冻结（docs 先行，纯文档）

**2026-10-12 —— 开票时已核过的第一手（省得开工时重搜）**

| 事实 | 锚点 |
|---|---|
| 真机 `UXRCE_DDS_CFG = 0` | `docs/assets/z0.params:949`；`bag/z0-params-2026-10-10.params:913` |
| `MAV_0_CONFIG = 101` / `MAV_1_CONFIG = 102` / `MAV_2_CONFIG = 0` | `docs/assets/z0.params:528 / :534 / :540`；bag 版 `:492 / :498 / :504` |
| 串口冲突时**静默不启动** | `~/refs/px4-v1.14.3-build/Tools/serial/rc.serial_port.jinja:3`（`set SERIAL_DEV none`）与 `:14`（`Conflicting config`） |
| 两条路汇合到同一条 uORB | `.../mavlink_receiver.cpp:1580-1596`（`:1593` 的发布**被 OFFBOARD 守着**） |
| 心跳由飞控派生 | 同文件 `:1531-1535` |
| `thrust_body[2] = −thrust` | 同文件 `:1456`（签名）与 `:1475` |
| `AttitudeTarget` 无 `thrust_body` | `/opt/ros/humble/share/mavros_msgs/msg/AttitudeTarget.msg`（字段：`type_mask` / `orientation` / `body_rate` / `thrust`） |
| mavros 自己转 `body_rate` | `~/mavros_ws/src/mavros/mavros/src/plugins/setpoint_raw.cpp:321-331`；常量在 `.../src/lib/ftf_frame_conversions.cpp:40` |
| `/mavros/state` 1 Hz、`StateQoS` | `.../src/plugins/sys_status.cpp:962-985`（每条 HEARTBEAT 发一次）与 `:583`（`mavros::StateQoS()`） |
| Phase 1 的同款坑 | `src/control/config/control.yaml:120-125`（`state_timeout_s: 2.5`，注释明写"不要照抄 0.5 s"） |
★ 这些**全部是读源码得来的**，**没有一条是实测**。别在文档里把它们写成"实测过"。

---

#### 二、节点换下行 —— 话题 / 消息 / 闸门源 / 超时 / 依赖

**2026-10-12 —— 开票时已核过 / **未核**的，分开放**

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
   但**没有实测过**（§四 的夹具要用**真的** `SensorDataQoS` 才算验过）。
4. **真机上有没有跑过一次**：**没有**。本票全部依据是读源码。

---

#### 三、阶梯驱动工具 `offboard_smoke.py` + `--t-max` 那笔账

**2026-10-12 —— 开票时已核过 / **未核**的**

**已核（读源码得来）**：
| 事实 | 锚点 |
|---|---|
| 飞控**不读** `time_boot_ms`，自己打时间戳 | `mavlink_receiver.cpp:1508-1598`（整个 handler，逐行看过） |
| 心跳由收到的消息派生 | 同文件 `:1531-1535`（且**不受 OFFBOARD 限制地**发） |
| rates setpoint 的发布**被 OFFBOARD 守着** | 同文件 `:1593` |
| `type_mask = 128` ⟹ `body_rates = true`、`attitude = false`、`fill_thrust` 被调用 | 同文件 `:1522-1524` 与 `:1589` |
| 悬停/上界的两个常数不是一个东西 | `docs/control/first-flight-config.md` **N4**（`t_max = 46.3027` vs `f_max = 11.8`） |
| κ 的误差源里已列了 `--t-max` 默认 11.8 | `docs/control/ladder-acceptance-criteria.md` §1.2 的 ★★ 段 |
**未核（开工时要补）**：
1. `/mavros/state.mode` 在 OFFBOARD 时的**字符串**（与 §二 ③ 同一处，两节共用一个答案）。
2. `--qos` 现在的取值集合与 mavros 收端 `SensorDataQoS` 的**实测**兼容性（与 §二 未核第 3 条同源）。
3. 真机上**没有**跑过一次。

---

#### 四、夹具与 seam 1 —— 用 mavros 形状重跑

**2026-10-12 —— 开票时已核过 / **未核**的**

**已核**：
- 收端 QoS = `SensorDataQoS()`：`~/mavros_ws/src/mavros/mavros/src/plugins/setpoint_raw.cpp:62`。
- `/mavros/state` 由 `StateQoS()`（reliable + transient_local）发布、每条 HEARTBEAT 一次 = 1 Hz：
  `.../src/plugins/sys_status.cpp:583` 与 `:962-985`。
- 飞控**不读** `time_boot_ms`，自己打 `hrt_absolute_time()`：
  `mavlink_receiver.cpp:1508-1598`（整个 handler 逐行看过）。
- `AttitudeTarget` 的字段与 `type_mask` 位常量（`IGNORE_ROLL_RATE=1/…/IGNORE_ATTITUDE=128`）：
  `/opt/ros/humble/share/mavros_msgs/msg/AttitudeTarget.msg`（**有** `std_msgs/Header`）。
**未核（开工时先核）**：
1. `fake_mavros.py` 的 `follow_arming` / `follow_set_mode` 是否**已经**支持
   `/mavros/cmd/arming` + `/mavros/set_mode` 的完整往返（**先读再改**）。
2. `tools/px4-sitl/` 那条链**现在**还用不用 `fake_px4.py`（④ 的判据靠它）。
3. 那 ~25 条依赖 `/fmu/*` 的用例**逐条清单**（本票只记了"约 25 条"，**没逐条编号**）——
   开工第一件事是把这份清单拉出来贴上，⛔ 别凭印象删。

---

#### 五、阶梯判据页 §7.1 留痕

**2026-10-12 —— 开票时已核过的**

| 事实 | 锚点 |
|---|---|
| 本页已定稿 v1、改动走 §7.1（☠️ + §7.2 增行 + **不许原地改数**） | `ladder-acceptance-criteria.md` §7.0（`:500-516`）与 §7.1（`:518-521`） |
| §7.2 现有两行、五列表头 | 同页 `:523-528` |
| §1.2 已把 `--t-max` 默认 11.8 N 列为 κ 的误差源之一 | 同页 `:176-189`（第 2 条在 `:183`） |
| §1.1 的表头写着"实际发出的 **PX4 字段**" | 同页 `:77-81` |
| §1.3 ⑤ 用的是 `nav_state`/`arming_state` 这套读数 | 同页 `:209` |
| "物理口 → 机体角"已被登记为**未处置相邻事项** | 同页 `:536`（§7.2 底部那张表） |
★ **本票的留痕文本全部是纸面推导**，**没有任何实测**：
A/B/E 行的锚是**参数读数**，C 行的锚是**算术**。⛔ 三处都别写成"实测/验证过"。

---

#### 六、独立项：用 QGC 执行器测试落实「物理口 ↔ 机体角」

**2026-10-12 —— 开票时已核过的**

| 事实 | 锚点 |
|---|---|
| `PWM_MAIN_FUNC1..4 = 103, 102, 101, 104` | `docs/assets/z0.params:699-702`；bag 版 `:663-666` |
| `Motor` 族 id 从 **101** 起、共 12 个 | `~/refs/px4-v1.14.3-build/src/lib/mixer_module/output_functions.yaml` |
| `CA_ROTORn` 按**下标顺序**读、转子 n ↔ Motor(n+1) | `.../control_allocator/ActuatorEffectiveness/ActuatorEffectivenessRotors.cpp:54-77` |
| v1.14.3 的 `src/` 里**没有** `MOT_ORDERING` | `grep -rn 'MOT_ORDERING' src/` **零命中** |
| 角点：0 前右 / 1 后左 / 2 前左 / 3 后右 | `docs/assets/z0.params:130-177`（`PX/PY`）+ `AZ = −1`；推导在判据页 §1.1 `:85-88` |
| `KM`：0/1 = +0.05、2/3 = −0.05 | `z0.params:134 / :158 / :166 / :174` |
| KM 正 ⟹ CCW、负 ⟹ CW（PX4 自己的话） | `.../control_allocator/module.yaml:211-214` |
| 执行器测试的拒绝条件与参数空间 | `.../commander/Commander.cpp:1431-1469`（`:1442` 取 `param5`、`:1463` `param1`、`:1465` `param2`、`:1476-1478` 上限 3000 ms） |
| z0 的四个前置条件现值 | `z0.params:286`（`COM_MOT_TEST_EN = 1`）、`:115`（`CA_AIRFRAME = 0`）、`:226`（`CA_ROTOR_COUNT = 4`） |
**未核（现场第一件事）**：
1. ★★ **QGC 执行器测试页实际送的 `param5` 编号空间**（QGC 不在本仓库里，源码没读过）。
   核法：`listener actuator_test` 看 `function` 的真值。
2. **`MAV_CMD_ACTUATOR_TEST` 的完整参数定义**（本票只读了飞控侧的**消费端**：
   `param1/param2/param5` 的用法是**从消费端反推**的，没读 MAVLink 消息定义那一侧的文档）。
   现场若发现某个参数对不上（例如超时行为与预期不同），**以飞控侧那段代码为准**，并把它记下来。
3. **真机上没做过**：四行表是空的，⛔ 别把上面"已推得的链条"当成实测结果引用。

---

### 2026-10-12 —— 操作者裁定（四题；票面已按裁定改写，⛔ 未开工）

开票后把这四处"本来要人定"的地方摆成选项问了一遍，**四处都取了推荐项**。
裁定**已回写**到上面对应的小节与验收框里；下面留一份集中副本，省得逐节翻。

| # | 问题 | 裁定 | 落点 |
|---|---|---|---|
| 1 | §7.2 增几行、怎么拆 | **四行：A / B / C / E**；**D 行并入 C 行**（同一份证据，§7.0 条 3 读作"一个证据一行、一行可点多处"） | §五 节首、C 行末尾第二张表、D 行小节、验收 §五 |
| 2 | `--t-max` 那笔账怎么落 | **改成"必须显式给"**：删掉默认值，不给就拒绝跑；帮助文本写明今天的正确值 `46.3027 N`（= `m·g / hover_thrust`）与它随悬停油门变；连带阶梯 2 跑法块补 `--t-max` | §三 ② 第 1 条、§五 C 行、验收 §三 |
| 3 | §六 执行器测试怎么排 | **排在阶梯 1 之前、单独做一次场次**（⛔ 不并进阶梯 1 那次通电） | §六 节首区、验收 §六 |
| 4 | §二 ⑤ A 档（`state_source="px4"`）怎么处置 | **(b)**：判为 uXRCE-DDS 专用、入口**显式拒绝**（fail loud）；(a) 未被采纳，将来要做**另开票** | §二 ⑤、验收 §二 |

★ 这四处的**共同点**：都不是"怎么实现"的问题，而是"**选哪条口径**"——所以按 `ready-for-human`
摆给操作者，而不是 agent 自己挑一个写进票面。

⚠️ **仍未定的只剩一处，且不能做成选项**：§7.2 那张表**"谁定的"列的签字**。
那是操作者自己的名字/裁定，只有人能给 —— 上面四题落定之后，那一列按裁定内容写好，由操作者签。

★ 这次裁定**没有动任何代码**：`.py` / `.xml` / 参数文件一个没碰，`Status` 仍是 `ready-for-human`。
