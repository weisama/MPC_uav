# 13: (C) 级下行从 px4_msgs/uXRCE-DDS 换成 mavros

Status: ready-for-human
Type: task

**★★ 2026-10-12 操作者裁定：重开本票（`Status: done` 退回），本轮只落 `§三` + `§一①` 的最小片段。**
退回的**不是**上面那条收口裁定的结论，而是"本票到此为止"这件事：§一–§五 的施工**仍未全部开工**，
而 `§三`（阶梯 1/2 唯一的执行者 `offboard_smoke.py`）+ `§一①`（`sign-map.md` 加"谁执行"一列）
被判定为**不必等人、也不该再等**的两节。**本轮范围与逐项落地情况见 Comments 的「2026-10-12 第二轮」一节**
（那里同时登记了本轮**新造出来的**悬空引用 —— 属于 §二 / §四 / §五 的落点，按 §一④ 的口径只登记、不改）。
★ `§五` / `§六` 仍需人（签字、动飞机）⟹ 状态回到 `ready-for-human`，而不是 `ready-for-agent`。

**☠️ 2026-10-12 操作者裁定收口 → `done`（本段随重开作废，原文留档）**：本票的交付 = **票面本身**，加上 **§六 的实测结果已入册**。
**§一–§五 描述的施工未开工** —— 现场证据：`src/nmpc/nmpc/mpc_node.py:255-261` 仍发 `/fmu/in/*`、
`src/nmpc/nmpc/offboard_smoke.py:108-112` 同、阶梯页 §7.2 里**没有** A/B/C/E 那四行。施工不在本票范围内。
★ 因此本仓库 `done` 的成文定义（`docs/agents/triage-labels.md:12`：「验收项**全部勾上**且结论已落进文档」）
在本票上**只满足后半句** —— 验收清单已于同日按操作者要求整节删除（`336b58c`）。
★ 本次收口**没有动任何代码、参数或别的文档**：`.py` / `.yaml` / `.params` / 阶梯页一个没碰。

**What to build:** 把 Phase 4 的 **(C) 级下行**从 px4_msgs/uXRCE-DDS 换成
**mavros 的 `/mavros/setpoint_raw/attitude`**，让四级阶梯在**不新增硬件、不抢串口**的前提下继续往下走 ——
含口径冻结（docs 先行）、节点、阶梯驱动工具、夹具与 seam 1、判据留痕，以及一条**与通路无关**的独立项。

**Blocked by:** 无。★ 但**本票内部有次序**（原先六张票的阻塞边，现变成同一张票里的先后）：
**§一 必须在 §二 / §三 之前**（口径没冻就动手 = 按个人理解写 N1/N2/N3）；
**§四 在 §二 之后**（DUT 得先会说 mavros 的话）；
**§五 在 §三 之后**（留痕要写"换成了什么"）；
**§六 与任何人无关、随时可做**。

**☠️ 2026-10-12 本段随收口作废（原文留档）**：下面这段是当初定 `Status: ready-for-human` 的理由；
票已于同日收口为 `done`（见上面那条），**`ready-for-human` 的判据不再适用**。
★★ 2026-10-12（同日第二轮）：**上面这条作废标记随重开一并撤回** —— `Status` 已退回
`ready-for-human`，本段的判据**重新适用**（本段原文一字未改：留档的是"它曾经被作废过"这件事）。

**★ 本票为什么是 `ready-for-human`（而内部有一大半是 AFK 可推进的）**：
`ready-for-human` 只要**有一处**必须人来做就成立 —— 本票有两处：**§五**（§7.2 的"谁定的"列要操作者签字，
且其中一行有安全后果）与**§六**（要动飞机：通电、点电机 —— 车侧动作由操作者亲手做，
且"哪个口在转"只有眼睛看得到）。
**§一 / §二 / §三 / §四 全部是 AFK 可推进的**，别把它们一起拖到人在场再做。

★★ **2026-10-12：四处"本来要人定"的口径已问过操作者并裁定**（四题全取推荐项 ——
§7.2 拆四行 / `--t-max` 改成必须显式给 / §六 排在阶梯 1 之前单独做一次 / A 档取 (b)）。
裁定**已回写**到对应小节，集中副本见 **Comments 末节**。
✅ **2026-10-12 操作者「签」**：§7.2 那四行的 **`谁定的` 列已签**，
逐字草稿见 §五 末尾的「✅ 2026-10-12：四行已签」——
★ 签的是**行**（行数、锚、措辞），**不是"已落地"**：那四行今天还不能写进阶梯页，理由写在那节里。

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
  （`mavlink_receiver.cpp:1516-1518` 的 `if (_mavlink->get_forward_externalsp() && …)`），
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
（`mavlink_receiver.cpp:1516-1518`），也就是飞控参数 **`MAV_FWDEXTSP`**
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
（`mavlink_receiver.cpp:1509-1599`，**整个函数**）里**一次都没被读** ——
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
**谁做这一节**：纯台架，不需要飞机通电、不需要起链。★ 证伪最硬的一节。

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
| **时间戳**（那一节） | ⛔ **本类在新载体上没有对应物**：飞控不读 `time_boot_ms`（`mavlink_receiver.cpp:1509-1599` 整个 handler 里没读）⟹ **删除或标记为"不再可测"**，并在映射表里写明**为什么**。**不许留成"绿着但不测任何东西"** |
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

### ✅ 2026-10-12：四行**已签**（`谁定的` 列）—— 草稿在这里，落地等改动本身

**操作者 2026-10-12「签」**。体例照 [`ladder-acceptance-criteria.md`](../../../docs/control/ladder-acceptance-criteria.md)
§7.2 现有两行（`| 日期 | 定稿？ | 改了什么 | 促成的数据（哪一级 / 哪个数） | 谁定的 |`），
`谁定的` 那一列照该页既有的写法（对照 `:528` 的 `操作者 2026-10-10「都按建议执行」`）。

| 日期 | 定稿？ | 改了什么 | 促成的数据（哪一级 / 哪个数） | 谁定的 |
|---|---|---|---|---|
| 2026-10-12 | ★（v1 之后） | **A 行**：§1.1 的**程序段与命令块** —— 驱动工具由 px4_msgs/uXRCE-DDS 形态换成 mavros 形态（下发 → `/mavros/setpoint_raw/attitude`；解锁/切模式 → `/mavros/cmd/arming` + `/mavros/set_mode`），原命令块就地 ☠️。★ 连带：`:57` 那句"它错了就只可能是中间件、符号或归一化错"**已重核**（走 mavros 后多了**别人的中间件**与**飞控那一行** `fill_thrust`） | 真机**参数**读数：`UXRCE_DDS_CFG = 0`（`z0.params:949`）、`MAV_1_CONFIG = 102`（`:534`，那条串口被 mavros 占）。★ 如实写：**这是参数读数，不是飞行读数** | 操作者 2026-10-12「签」 |
| 2026-10-12 | ★（v1 之后） | **B 行**：§1.1 那张表的**中间一列**（表头"smoke 节点实际发出的 **PX4 字段**"与表体 `roll_rate = +` / `pitch_rate = −` / `yaw_rate = −`）就地 ☠️，换成新载体上实际发出的字段：`body_rate.x/y/z` + 标量 `thrust` —— **pitch/yaw 不再是负号**（负号由 mavros 做）、**`thrust` 是正号**（负号由飞控 `fill_thrust` 做）。★★ **第三列（物理预期：哪两个加速、哪两个减速）一个字不改** | 同 A 行（换载体）—— **单独成行**，因为废的东西不同：A 废"跑法"，B 废"**我们往线上放的那个数**" | 操作者 2026-10-12「签」 |
| 2026-10-12 | ★（v1 之后） | **C 行（已含 D 行）**：§1.2 那处"**归一的基准** `--t-max`（默认 11.8 N，config 页 N4 的 `f_max`）"就地 ☠️ —— `11.8 N` 是**电池受限的 F 上界**，不是归一化基准；新口径按 §三 ② 的裁定写入（**`--t-max` 改成必须显式给**）。★ 连带同一行的第二处位置：§1.2 的**跑法块**补上 `--t-max`（不给就跑不起来）。★★ §1.2 的 ★★ 误差源清单**节首挂作废横幅并逐项点名** | ★ **算术 + 两个已实测常数**（`t_max = 46.3027 N`、`hover_thrust = 0.1589`）：`7.3575/11.8 = 0.6235` 对 `7.3575/46.3027 = 0.1589` ⟹ **≈3.92 倍**。★★ **没有飞行数据作锚，这一点如实写进"促成的数据"列** | 操作者 2026-10-12「签」 |
| 2026-10-12 | ★（v1 之后） | **E 行**：§1.3 判据 ⑤ 的**读数来源** —— 读的**量**没变（"全程在 OFFBOARD 且已解锁"），**读的地方**换成 `/mavros/state` 的 `armed` 与 `mode`；原判据就地 ☠️ 后换源。★ 连带：括注里 `heartbeat_always` 的语义与 §二 ④ 同步（⛔ 别留指向已改名旋钮的括注） | 同 A 行（换载体） | 操作者 2026-10-12「签」 |

★★ **签的是行，不是"已经落地"** —— 这四行的 `改了什么` 描述的是**将要发生**的改动，
而今天工具仍发 `/fmu/*`。所以**这四行此刻不写进 `ladder-acceptance-criteria.md`**：

- §7.1 的第 ③ 步要"把被废的那一条**就地标 ☠️**"，而那条（旧命令块 / 中间一列 / `11.8 N` / 判据 ⑤）
  **今天还是活的、照抄能跑** ⟹ 提前 ☠️ 会造出一个**当时为假的状态**（[散文里写死的数会过期] 那一族，
  状态词比数更毒）；
- 正因如此，本票的**票内次序**把 §五 排在 §三 之后：留痕要写"**换成了什么**"。

**落地时机**：A / B / E 三行随 §二 / §三 的改动落地；C 行随 §三 ② 的改动落地。
**落地的动作**：把这四行**逐字**插进 §7.2 表尾（一行一次，别合并），并按 §7.1 第 ③ 步把对应旧处 ☠️。
★ 每行的"促成的数据"列已按上面那张表写好 —— 落地时**照抄**，⛔ 别即兴改写。
⛔ 落地时**不许**顺手改 §7.2 底部那张"已登记、未处置的相邻事项"表（那一行归 §六）。

---

## 六、独立项：电机编号与转向 —— ✅ **2026-10-12 操作者已实测**

**结果**（操作者 2026-10-12 现场实测；本节**只记录**，⛔ 不改任何参数）：

| 机体角 | 转子序号 | 电机序号 | 物理输出口 |
|---|---|---|---|
| **前右** | `CA_ROTOR0` | **Motor1** | **MAIN3** |
| **后左** | `CA_ROTOR1` | **Motor2** | **MAIN2** |
| **前左** | `CA_ROTOR2` | **Motor3** | **MAIN1** |
| **后右** | `CA_ROTOR3` | **Motor4** | **MAIN4** |

- **与开票时推得的一致**：这张表与用 `PWM_MAIN_FUNC1..4 = 103, 102, 101, 104`（`z0.params:699-702`）
  ＋ `CA_ROTORn` 下标序（`ActuatorEffectivenessRotors.cpp:54-77`）推出来的**逐行相同** ⟹ **无意外**。
  ★ 而这正是它值得实测一眼的地方：`PWM_MAIN_FUNC` 是**刻意改过的非默认序**（`params-changes.md` D7）
  ⟹「Motor n → 输出 n」不成立，读错一行飞控**不报错**。
- **转向**：实测**无异常**。操作者原话「**电机 13 逆时针，24 顺时针**」—— 按**绕机架一圈**的编号读，
  即 PX4 序号下的「前右+后左 一组 / 前左+后右 另一组」，与 `CA_ROTOR0/1_KM = +0.05`、
  `CA_ROTOR2/3_KM = −0.05`（`z0.params:134/158/166/174`）的期望同义。
  ★ 原话照抄在上；括号里那种读法是**我的**，不是操作者说的。
- ⛔ **只读不写**：`CA_ROTORn_KM` / `PWM_MAIN_FUNC*` 一个没改。

**还没落位的三处**（本节只出"填什么"，落位各有其主）：`ladder-acceptance-criteria.md` §1.1 的 ⚠️（`:103-107`）、
同页 §7.2 底部"已登记、未处置的相邻事项"表那一行（`:536`）、`params-changes.md` **D7**（它自己的归属票，不是本票）。
★ 与 §五 的分工不变：**§五 ⛔ 不碰**这三处（它在 F 行里已明写）。

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
| **硬前置** `MAV_FWDEXTSP = 1` | 消费端 `mavlink_receiver.cpp:1516-1518`；定义 `.../mavlink_params.c:112-121`；z0 现值 `z0.params:542` |
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
| 飞控**不读** `time_boot_ms`，自己打时间戳 | `mavlink_receiver.cpp:1509-1599`（整个 handler，逐行看过） |
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
  `mavlink_receiver.cpp:1509-1599`（整个 handler 逐行看过）。
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
裁定**已回写**到上面对应的小节里；下面留一份集中副本，省得逐节翻。

| # | 问题 | 裁定 | 落点 |
|---|---|---|---|
| 1 | §7.2 增几行、怎么拆 | **四行：A / B / C / E**；**D 行并入 C 行**（同一份证据，§7.0 条 3 读作"一个证据一行、一行可点多处"） | §五 节首、C 行末尾第二张表、D 行小节 |
| 2 | `--t-max` 那笔账怎么落 | **改成"必须显式给"**：删掉默认值，不给就拒绝跑；帮助文本写明今天的正确值 `46.3027 N`（= `m·g / hover_thrust`）与它随悬停油门变；连带阶梯 2 跑法块补 `--t-max` | §三 ② 第 1 条、§五 C 行 |
| 3 | §六 执行器测试怎么排 | **排在阶梯 1 之前、单独做一次场次**（⛔ 不并进阶梯 1 那次通电） | §六 节首区 |
| 4 | §二 ⑤ A 档（`state_source="px4"`）怎么处置 | **(b)**：判为 uXRCE-DDS 专用、入口**显式拒绝**（fail loud）；(a) 未被采纳，将来要做**另开票** | §二 ⑤ |

★ 这四处的**共同点**：都不是"怎么实现"的问题，而是"**选哪条口径**"——所以按 `ready-for-human`
摆给操作者，而不是 agent 自己挑一个写进票面。

★ **第五处（不能做成选项的那一处）：§7.2「谁定的」列的签字 —— 操作者 2026-10-12「签」**。
四行（A / B / C 含 D / E）的**逐字草稿**见 §五 末尾那张表，体例照该页 §7.2 现有两行。
⚠️ **签的是行，不是"已落地"**：那四行描述的是**将要发生**的改动，而今天工具仍发 `/fmu/*`
—— 提前写进阶梯页会造出当时为假的状态（§7.1 第 ③ 步要 ☠️ 的那几条**今天还是活的**）。
所以它们随 §二 / §三 的改动落地，落地时照抄草稿、逐行插入。

★ 这次裁定**没有动任何代码**：`.py` / `.xml` / 参数文件一个没碰，`Status` 仍是 `ready-for-human`。

---

### 2026-10-12 —— 第二轮：`§一①` + `§三` 落地（本轮实测）

范围见票面节首那条重开块：**只落 `§三` + `§一①` 的最小片段**，其余不动。
本节登记 ①落了什么 ②本轮**新造出来的**悬空引用（按 `§一④` 的口径只登记、不改）
③没做的 ④施工中踩到并修掉的一处自伤（留作边界证据）⑤测试与套件状态。

#### 一、落了什么（逐项对票面）

**`§三①` 接口换掉** —— [`src/nmpc/nmpc/offboard_smoke.py`](../../../src/nmpc/nmpc/offboard_smoke.py) 按新载体改写：

| 票面要求 | 落地 | 位置 |
|---|---|---|
| pub `/fmu/in/offboard_control_mode` **删掉** | 删了（旧话题名在代码里**不再出现**，由 AST 断言守着） | —— |
| pub `/mavros/setpoint_raw/attitude`，`type_mask = 128` | `TYPE_MASK_IGNORE_ATTITUDE`（取自消息自己的常量），只置这一位 | `:77`、`:339` |
| `/mavros/cmd/arming` + `/mavros/set_mode` | 两个服务客户端 + 应答回调（被拒时把 `result` / `mode_sent` 打进日志） | `:246`、`:276` |
| sub `/mavros/state`（`armed` 且 `mode == "OFFBOARD"`） | ✓（模式名取 `State.MODE_PX4_OFFBOARD`，不手写字符串） | `:84`、`:318-322` |
| `_map()` 调 **具名 mavros 入口**、⛔ 不许预转换 | 调 `sign_map.mavros_attitude_target()`；预转换那一档仍可注入 | `:218-236` |
| `--pre-stream` 语义 "大致保留，按 mavros 时序重核" | 保留；★ 重核的结论见下 | `:457`、`:479` |
| `_CHECKED_POSITIVE` **保留** | 保留，并加一条归一化自洽 | `:457`、`:479` |

★ `§三①` 那条"**心跳时序要重核**"的第一手结论：闸门源 `/mavros/state` 是 **1 Hz**
（`mavros sys_status.cpp:966-972`，每收到一条 HEARTBEAT 才发一帧），
而旧载体上 `--cmd-interval` 的默认 **0.5 s** 是按"2 Hz 以上的流"配的 ⟹
**解锁/切模式的最小间隔必须 > 1 s**，默认提到 **1.1 s**（`Config.cmd_interval_s`）；
进 OFFBOARD 的那一刻把程序时钟归零（`offboard_since_ns`），
⚠️ 所以"切进 OFFBOARD"最多比飞控真正进入晚一个上报周期 ⟹ 时间轴偏**短**、不会偏长。

**`§三②` `--t-max` 那笔账**（三条全落）：
1. `Config.t_max_n` **没有默认值**（`Config()` ⟹ `TypeError`），argparse `required=True`，
   帮助文本写明今天的正确值 46.3027 N 的来历（`m·g / hover_thrust`）与**它为什么会变**；
2. 自检落在 [`params.check_hover_consistency()`](../../../src/nmpc/nmpc/params.py)：
   容差 = `NORM_TOL_REL = 1e-3`（**与 `node_core.SAT_TOL_REL` 同值同口径**，有漂移守卫钉着），
   `validate_args()` 在**建节点之前**调它，不一致 ⟹ `SystemExit`（fail loud，不给回落值）；
3. `_describe_emitted()` 每段打一行 `F=…N ⟹ thrust=+…`（**归一化之后真正发出去的那个数**）——
   ② 这类错误在屏幕上可见（`commands.describe()` 只印 `F`，看不出归一化常数错了）。

**变异测试（4 次，逐条都红了）** —— 按票面要求，新断言都要先弄坏一次：

| 变异 | 期望 | 结果 |
|---|---|---|
| `sign_map.mavros_attitude_target()` 里给 `body_rate_y` 加负号 | N2 断言红 | ✅ 2 条红（`test_mavros_entry_emits_the_flu_values_verbatim`、`test_the_two_entries_differ_only_in_who_applies_the_sign`） |
| `TYPE_MASK_IGNORE_ATTITUDE` 多置 `IGNORE_THRUST`（64） | 形状断言红 | ✅ `test_our_type_mask_ignores_only_the_orientation` |
| 节点里加一个 `/fmu/in/vehicle_rates_setpoint` 字符串字面量 | AST 断言红 | ✅ `test_the_px4_msgs_downlink_is_gone_from_the_code` |
| `NORM_TOL_REL` 改松到 `10.0` | 自检失去鉴别力 ⟹ 红 | ✅ 3 条红（含票面点名要的那一档） |

★ 票面 `§三②` 点名的那次变异（"**把 `--t-max` 改回 11.8 ⟹ 必须变红**"）现在是**常驻断言**：
`test_mavros_interface.py::test_the_old_default_is_rejected_before_the_node_is_built`
（`validate_args` 必须 `SystemExit`，且话术里要能**照抄出**正确值 46.3027）。

**`§三③` 故障注入逐条处置**：

| 开关 | 处置 | 依据 |
|---|---|---|
| `--break thrust-sign` / `thrust-scale` / `roll-sign` | **保留** | 三档在新载体上含义没变（roll 不被 mavros 取负 ⟹ 仍是"符号写反"） |
| `--break pitch-sign` / `yaw-sign` | **改名** `pitch-preconvert` / `yaw-preconvert` | 这两档现在模拟的是 **N2**（我们多做一次预转换），**不是**"符号写反"；⛔ 旧名字不再被接受（同一个名字指另一件事） |
| `--timestamp-unit` / `--timestamp-shift-us` | **删掉** | 飞控那段 handler 一次没读 `time_boot_ms`（`mavlink_receiver.cpp:1509-1599`）⟹ 注入效果逐位相同；留着会让操作者以为"这一档也测过了" |
| `--qos` | **保留**，取值集合不变，但帮助文本改写 | 发布者是 reliable ⟹ 两个取值**都收得到** ⟹ 在新载体上**不再能造出"零回调"** |

**`§一①`（[`docs/control/sign-map.md`](../../../docs/control/sign-map.md)）**：§2 拆成
**§2.1 端到端映射（判据，与通路无关）** / **§2.2 我们实际发出的字段（按通路分叉）** /
**§2.3 所有权对照 + 锚点表** / **§2.3.1 时间戳那一档为什么没有对应物**；
§8 契约 1 / 3 / 5 与 §9 重新表述。
★ `sign-map:canonical` 那个块**一个字节没动**（`test_sign_map.py` 解析它，`tools/sign-map/test_sign_map_core.py:230` 还断言它的键集）。

★ 票面 `§一①` 要"在 Comments 里记一笔"的那条（**§8 第 4·5 条契约的取舍与措辞**）：

> **§8.5 的落点换了对象**：从"飞控收到的那个字段"改成"**我们发出的那个字段**"。
> 理由是第一手核对过的：mavros 在 `attitude_cb` 里**自己**再转一次 FLU→FRD
> （`setpoint_raw.cpp:323-324`）⟹ 我们若预转换一次，**飞控最终收到的分毫不变**（N2）⟹
> 任何落在"飞控收到了什么"上的断言，在"我们写对了"和"我们多转了一次"两种情况下**都会绿**。
> 所以 mavros 路径的断言必须落在 `sign_map.mavros_attitude_target()` 的**返回值**上
> （`body_rate_* == ω 原样`、`thrust == +F/T_max`）。**§8.4**（docstring 不复述数值）**不变**。

★ **顺手复核过的锚点行号**（`§一①` 的活文档义务，2026-10-12 在 `~/refs/px4-v1.14.3-build` 下逐行看过）：

- `mavlink_receiver.cpp:1456`（`fill_thrust` 签名）、`:1475`（`thrust_body_array[2] = -thrust;`）、
  `:1522`（`const bool attitude = !(type_mask & …ATTITUDE_IGNORE);`）、`:1530-1535`（OCM 派生并发布）、
  `:1588-1589`（`THROTTLE_IGNORE` 未置位才 `fill_thrust`）、`:1593-1594`（只在 OFFBOARD 里发布 +
  飞控自己打时间戳）—— **逐条对上**。
- ⚠️ **`MAV_FWDEXTSP` 那个 `if` 在 `:1516`，不在 `:1515`**（`:1515` 是它上面那行注释
  `/* Only accept messages which are intended for this system */`）⟹ `sign-map.md` §2.3 已改成 `:1516`。
- ⚠️ 整个 handler 实际是 **`:1509-1599`**（`void` 在 `:1509`、函数名 `:1510`、闭括号 `:1599`）；
  票面 `§三③` 与 `spec.md` 原先那种 `1508-1598` 的**区间**写法整体偏一行
  （`:1508` 是空行、`:1598` 是函数体里那个 `}`，闭括号在旁边一行）——
  ✅ **2026-10-12 操作者裁定：原地改正**，两文件共 7 处 `1508-1598` 已改成 `1509-1599`
  （结论不变：那一整段里确实**没读** `time_boot_ms`）。
- ⚠️ 同源的第三处：`MAV_FWDEXTSP` 那个 `if` 的引用原先写 `:1508-1515`，**`if` 其实在 `:1516-1518`**
  （`:1515` 是它上面那行注释）—— 一并按同一裁定改为 `:1516-1518`（票面 3 处 + `spec.md` 1 处）。
  ★ 顺带核过、**没动**的：`:1530-1535` 与 `:1531-1535` 两种写法**都指到那段 OCM 派生代码**
  （`:1530` 是条件、`:1531-1535` 是块体），不是错，故保留原样。

#### 二、本轮**新造出来的**悬空引用（按 `§一④` 的口径：只登记、不改）

| 引用处 | 今天的状态 | 归谁 |
|---|---|---|
| [`verify_offboard_smoke.py`](../../../tools/px4-sitl/verify_offboard_smoke.py) `:544-564`（场景表） | 仍传**已删**的 `--timestamp-unit` / `--timestamp-shift-us` 与**旧名** `--break pitch-sign` ⟹ argparse 直接拒 | `§四`（或另开票） |
| [`test_verify_judges.py`](../../../tools/px4-sitl/test_verify_judges.py) `:123` | 同上（一条用例的 argv） | 同上 |
| `verify_offboard_smoke.py` 全文（话题 / 闸门源 / 判据 ⑤ 的"总推力"） | 钉在 `/fmu/*` + `px4_msgs` 上 | `§四` |
| [`src/bench/bench/fake_px4.py`](../../../src/bench/bench/fake_px4.py) | 旧载体夹具（假 PX4 的 uORB 侧） | `§四④`（要显式判去留） |
| [`src/bench/bench/fake_mavros.py`](../../../src/bench/bench/fake_mavros.py) | **已存在**（Phase 1 的），但**没有** `/mavros/setpoint_raw/attitude` 那条流 | `§四①` |
| [`ladder-acceptance-criteria.md`](../../../docs/control/ladder-acceptance-criteria.md) `:64`、`:127`（两处跑法块） | 缺 `--t-max` ⟹ 照抄会被拒 | `§五` C 行 |
| `ladder-acceptance-criteria.md:183` | 「归一的基准 `--t-max`（**默认 11.8 N**）」—— 这句话**今天为假** | `§五` C 行 |
| [`src/nmpc/README.md`](../../../src/nmpc/README.md) `:52`（跑法块） | 缺 `--t-max` | `§一④` 登记（本轮**未改**） |
| `mpc_node.py:255-261` 与 `package.xml:22` | 节点仍走 `/fmu/*`、仍依赖 `px4_msgs`（本票 `§二` 的活） | `§二` |
| [`mpc-node-seam1.md`](../../../docs/control/mpc-node-seam1.md) `:307` | 写着"删那个取证开关是 05 的事"—— 现在**已删**（在 `offboard_smoke.py` 里），该句口径要改 | `§四⑤` |
| [`sitl-offboard-smoke.md`](../../../docs/control/sitl-offboard-smoke.md) | **本轮已加 ☠️ 载体作废块**（页首 + §1 / §3.2 / §3.3 / §4 四处**节首**逐项点名） | 本票本轮 |

★ 两处**查过、结论是"不用改"**的（登记备查，免得下一个人重查）：
`src/nmpc/nmpc/__init__.py:16,18` 与 `src/nmpc/README.md:20,22` ——
它们写的是"`offboard_smoke` 是唯一 import `rclpy` / 中间件的那个"，换载体后**仍然成立**
（现在它是唯一 import `mavros_msgs` 的那个）。

#### 三、本轮**没做**的（票面里明确留着）

`§二`（`mpc_node.py` 换下行）、`§四`（夹具 / seam 1 / 上表那三份驱动）、
`§五`（阶梯判据页四行落地）、`§六`（结果放置），以及**任何一次上飞机的跑场**。
`Status` 由此停在 `ready-for-human` 而不是 `ready-for-agent`：`§五`（要签字 / 改写判据页）
与 `§六`（要动飞机）仍在。

#### 四、施工中踩到并修掉的一处**自伤**（留作边界证据）

把 `Z0Params.t_max` 改成调 `params.t_max_from_hover()`（"同一个公式只该有一处实现"看着更干净）⟹
`NodeParams(mass=0.0)` / `NodeParams(hover_thrust=1.2)` 在读属性时**直接抛**
⟹ [`preflight_check`](../../../src/nmpc/nmpc/preflight_check.py) 那两条**按名字点名**
（「质量为正」`preflight_check.py:101`、「悬停有上余量」`:104`）从**报错**退化成**崩掉**
（`test/test_preflight_check.py::test_坏参数会被点名[...]` 当场红，2 例）。

**根因**：属性是**读**、函数是**守门口** —— 混用之后"**拿着坏参数去问哪里不对**"这件事做不了，
而自检存在的意义正是把坏参数**说出来**。
已回退成 `return self.t_hover / self.hover_thrust`（与改动前**逐字相同**），
并把这条分工写成常驻断言（两个方向都钉）：
`test_params.py::test_the_property_reads_while_this_door_guards`。

★ 记下来是因为它**不在原计划里** —— 是**别的票写下的既有断言**替我们挡下来的（这正是它们的价值）。

#### 五、测试与套件状态（本轮实测）

**新增 / 受影响的测试**：
`src/nmpc/test/test_params.py`（12 例，**新增**，零 ROS）、
`src/nmpc/test/test_mavros_interface.py`（17 例，**新增**，要 `mavros_msgs` 但不起 mavros）、
`src/nmpc/test/test_sign_map.py`（+6 例，针对 mavros 入口与 N2）。

**全量套件（`src/nmpc/test/`，逐条实测，不是推算）**：**266 passed / 0 failed**，
= `--ignore=test/test_mpc_node.py` 那一组 **221 passed（63.9 s）**
+ `test/test_mpc_node.py`（seam 1，45 例）**45 passed（455.4 s）**。
⚠️ 分两段跑**不是**因为结果不同：本机 8 GB、seam 1 一例起一个 DUT 子进程，
两段合起来跑会超过单次命令的时限；而且**两段都跑 OCP 时会在共享的代码生成缓存
（`~/.cache/mpc-ocp-codegen`）上互踩** —— 本轮踩到过一次：另一个 pytest 进程正在重建
`libacados_ocp_solver_…so` 时，本进程的 OCP 用例会报
`OSError: …cannot open shared object file`（**看着像代码坏了，其实是并发**）。
如果要并行跑，用 `MPC_CODEGEN_DIR` 把两个进程的缓存隔开。

**跑之前要备的环境**（缺一条就会看到"几十个红"，那些**不是**代码问题）：

```bash
source /opt/ros/humble/setup.bash
source ~/px4_msgs_ws/install/setup.bash        # px4_msgs 在这个独立工作空间里
source install/setup.bash                      # 缺它 ⟹ seam 1 全红：`包 nmpc 不在环境里（prefix 查不到）`
export ACADOS_SOURCE_DIR=$HOME/acados          # 缺它 ⟹ 几十个 OCP 红
export LD_LIBRARY_PATH=$HOME/acados/lib:$LD_LIBRARY_PATH   # 缺它 ⟹ libhpipm.so 找不到
export PYTHONPATH=$PWD/../bench:$PYTHONPATH    # 缺它 ⟹ conftest 里的夹具 import 不到
cd src/nmpc && python3 -m pytest test/ -q
```

★ 本轮之前那张"47 failed / 21 errors"的读数**全部是环境**（缺 `px4_msgs` 工作空间、
缺 `LD_LIBRARY_PATH`、缺 `install/setup.bash`）+ 上面那 2 例自伤；备齐环境后是 0 failed。

**CLI 冒烟（不起 mavros、不起 SITL，只喂参数）**：
`--t-max 11.8` ⟹ 启动即被拒、话术里带正确值 46.3027 与 3.92 倍；
正确值 ⟹ 启动横幅把两行都打出来 ——
`hover F=7.355N (1.00×悬停) ⟹ thrust=+0.1588`（对得上实测 `hover_thrust = 0.1589`）
与 `lift F=8.826N (1.20×悬停) ⟹ thrust=+0.1906`，
JSONL 里 `type_mask=128`、`body_rate_x=+0.1`（**FLU 原样**）、`thrust=0.1906`、`hover_multiple=1.2`。
⚠️ 顺带看到一处**既有**噪音（不是本轮引入、也不是本轮改的）：`timeout`/`SIGTERM` 结束进程时
`rclpy.executors.ExternalShutdownException` 会带 traceback 退出（`SIGINT`/Ctrl-C 那条路是干净的）。
要不要收它，属于"停机路径"那张票，本票不碰。

### 2026-10-12 —— 第三轮：`/code-review` 两轴复核（同一批文件；未新增功能）

**做法**：`/code-review`，两个并行子代理（Standards / Spec），固定点 = `HEAD` = `95b1d57`。
★ 每条意见都**先回代码 / 一手源核实**再落地（本仓库的规矩）；核不实的写在这里。

**Standards 轴（4 条，核后 2 条为真）**

1. ✅ **真、已改** —— `sign-map.md` §8 契约 1 那句「本仓库里**唯一**的调用点是
   `offboard_smoke.py`（`px4_rates_setpoint`）与 `mpc_node.py`（…）」**同一 diff 里已为假、且自相矛盾**：
   `grep -n px4_rates_setpoint src/nmpc/nmpc/offboard_smoke.py` **零命中**（它今天调
   `mavros_attitude_target`，`:214` / `:223`），而"唯一"却列了两项。非测试调用点实为
   `mpc_node.py:557`/`:885` 与 `preflight_check.py:43`/`:172`/`:203`/`:380`。该句已重写。
2. ✅ **真、已改** —— 新写的两处「整个 handler」区间 `:1508-1598` 偏一行。
   第一手：`awk 'NR>=1505 && NR<=1520'` 那份 `mavlink_receiver.cpp` ——
   **`void` 在 `:1509`、函数名在 `:1510`、闭括号在 `:1599`** ⟹ 文档那两处已改为 `:1509-1599`
   （`sign-map.md` §2.3.1 与 `sitl-offboard-smoke.md` 顶块）。
   ✅ **操作者随后裁定"原地改正"** ⟹ 票面 `§三③` 与 `spec.md` 里那 6 处也一并改正，
   同源的 `MAV_FWDEXTSP` 引用 `1508-1515` → `:1516-1518`（票面 3 处 + `spec.md` 1 处）见上面那段锚点记录。
   ★ **已无一处偏一行**：把这两段"记录旧值"的提及本身排除后，全仓 `1508-15` 零命中
   （`grep -rn "1508-15" --include=*.md .` 应只剩本页这两段与 `sign-map.md` §2.3.1 那句）。
3. ❌ **驳** —— `_request_arm` / `_request_mode` 同形骨架（Duplicated Code）。
   两个服务的请求与应答字段名、语义都不同（`value`/`success` vs `custom_mode`/`mode_sent`），
   抽一个泛型服务包装会让"这个服务到底在等什么"变得不可读 ⟹ **保留**。
4. ✅ **真（小）、已改** —— `STATE_PERIOD_S = 1.0` 只被散文引用；`params` 两个函数各带一个
   **没有任何调用方**的 `g=` 形参（`grep` 全仓无一处传值）。两处都按本仓库自己那条
   "**仪器没给那个量变过**"的教训处置：
   - `Config.cmd_interval_s` 改为 `STATE_PERIOD_S + 0.1`（**数值不变 = 1.1**），让那个常量参与计算；
   - `t_max_from_hover` / `hover_thrust_implied` 的 `g=` 形参**删掉** —— 留一个没人传的旋钮，
     下一个人会以为"有人会改它"。

**Spec 轴（4 条，核后 2 条为真）**

1. ✅ **真、已补** —— **`§一①` 的 §9 那一条本轮没落**：票面 §一① 明写「§9 那四件自检…**语义要一并说清**」
   「**本票只负责在文档里把这个论证写出来**」，而本轮 diff 的最后一个 hunk 是
   `@@ -298,14 +373,31 @@`（止于新 `:404`），§9 在其后 ⟹ **一字未动**。
   ⛔ 而我上一轮那句 Comments「§8 契约 1 / 3 / 5 **与 §9** 重新表述」里的 **§9 是假的**（§8 那三处为真）。
   ⟹ 已补写 `sign-map.md` **§9.1**（第 ③ 件换了主人：三条反例一条没删，证的是"这一步不许没人做"，
   从"我们实现的错法"升级成"**所有权转移的安全性证明**"；那条新断言归本票 §四，本节只写论证）。
   三个反例测试名逐字核过存在（`tools/sign-map/test_sign_map_core.py:180`/`:193`/`:202`）。
2. ❌ **驳** —— `§2.1` 没按字面加"谁执行"列。该信息落在 **§2.3 所有权对照表**（6 行、两条通路并列，
   信息量比一列大），§2.1 里另有一句 ⭐ 明说「§2.1 那四行没说'由谁做' … 逐行见 §2.3」⟹ **保留**。
   硬往判据表里塞那一列，会把"与通路无关的物理约定"与"按通路分叉的所有权"混进同一张表。
3. ⚠️ **留给操作者裁定** —— `sitl-offboard-smoke.md` 那 44 行 ☠️ 留痕，Spec 轴判「越了 §一④ 的'只登记不改'」。
   我的依据是 **§三③ 的授权句**：「（**若该工具在活文档里被描述过**）相应文档」，
   而这一页正是描述该工具的那一页；该段也只**标作废**（明写"本页不重跑、也不改驱动"）。
   两处都引得出票面原文 ⟹ 不自行改回，**等裁定**。
4. ★ **附带发现（本轮只登记、不改）** —— `tools/px4-sitl/verify_offboard_smoke.py:592` 给每一场传
   `--cmd-interval 0.5`，**短于** `/mavros/state` 的 1 Hz 上报周期，正是 §三① 点名的那个
   "闸门还没上报就判绿（或恒红）"的风险。该脚本的归属是 **§四**（它今天也跑不通：场景表还传
   `--timestamp-unit` / `--break pitch-sign`）。

**复核后的验证**（复核改动是新的一批编辑，故重跑）：
`test_params` + `test_mavros_interface` + `test_sign_map` + `test_preflight_check` = **97 passed**；
`tools/sign-map/test_sign_map_core.py` = **18 passed**；
**全量 `test/` = 266 passed / 0 failed**（除 `test_mpc_node.py` 的 221 条 62.98 s + `test_mpc_node.py` 45 条 455.09 s，
两块分开跑，避开单次 600 s 上限）。
⚠️ 跑之前机器重启过一次（`uptime` 28 min）—— 没有别的会话在用 acados 代码生成缓存，故未隔离 `MPC_CODEGEN_DIR`。

### 2026-10-12 —— 第四轮：`§二` 整节落地（节点换下行 / 依赖与构建 / seam 1 页那两处）

范围：`§二` ①–⑦（含 ⑤bis 那条硬前置与 ⑥ 点名的 `preflight_check` 复核）。
★ 本节每条都**回 `git diff` 的 hunk 核过**再写（不是照计划抄）—— 计划的措辞与落地的措辞会分叉。
⛔ **`§四` / `§五` / `§六` 仍未做**，`Status` 因此仍停在 `ready-for-human`。

#### 一、落了什么（逐项对票面）

**`§二①` 话题 / import / QoS** —— [`src/nmpc/nmpc/mpc_node.py`](../../../src/nmpc/nmpc/mpc_node.py)：

| 票面要求 | 落地 | 位置 |
|---|---|---|
| `TOPIC_RATES` → `/mavros/setpoint_raw/attitude` | ✓（旧值留在 ☠️ 块里） | `:348-351` |
| `TOPIC_STATUS` → `/mavros/state` | ✓ | `:340` |
| `TOPIC_MODE`（心跳）**删掉** | ✓ 常量删除，☠️ 写明"别把话题名留着不用" | `:352-357` |
| `TOPIC_PX4_POS` / `ATT` 见 ⑤ | 常量**保留**（☠️ 块说明：唯一用处是拒绝话术与参数面），⛔ **不再被任何 `create_subscription` 用** | `:341-346`、`:618-623` |
| `from px4_msgs.msg import …` → `mavros_msgs` | ✓ 整块替换，☠️ 写明"别把没用的 import 留着以防万一" | `:267-273`、`:259` |
| QoS **显式选**、对齐收端 | ✓ `QOS_PUB_ATT` = best_effort / depth **5** / volatile（逐项对齐 `SensorDataQoS()`，`setpoint_raw.cpp:62` 用在 `:76-79`）；`QOS_SUB_MAVROS_STATE` = reliable / depth 10 / **transient_local**（对齐 `mavros::StateQoS()`，`qos.hpp:60-63`）。两条**都不是** ROS 默认值，各自在注释里写了为什么 | `:286-315` |

★ **两个 QoS 档位不是同一个**（这是最容易"能通就不管"的地方）：发布那条要贴 mavros 的**收端**，
订阅那条要贴 mavros 的**发端**（StateQoS 带 `transient_local` ⟹ 节点晚起也能立刻拿到最后一帧 state）。

**`§二②` 消息怎么填**：

| 票面要求 | 落地 |
|---|---|
| `type_mask = 128` | `TYPE_MASK_IGNORE_ATTITUDE = 128`，取自消息自己的常量（`AttitudeTarget.IGNORE_ATTITUDE`），`_validate`/测试两处交叉核对 |
| `body_rate` 填 **FLU 原样**，⛔ 不预转换 | ✓ 调 `sign_map.mavros_attitude_target()`；该函数"什么都不做"就是它的正确实现（docstring 里写明，且 N2 那一档故障注入仍可复现） |
| `thrust` 填 **`+F/T_max`** | ✓ 同上一处；并在 `_PARAMS` 的 `t_max` 旁留了"归一化常数填错由启动自检拦"的指针 |
| 具名第二入口（**不是** `path=` 开关） | ✓ `sign_map.mavros_attitude_target()` → `MavrosAttitudeTarget(body_rate_x/y/z, thrust)`，字段名逐字同消息；`Px4RatesSetpoint` 原样保留 |

**`§二③` 闸门源 → `/mavros/state`**：`node_core` 的四种 hold 语义不变，`armed` 由整数枚举改成
`State.armed`（bool）、`nav_state` 改成 `State.mode`（**字符串**）；`stop_on_nav_state_exit` 重对过
（`mode != "OFFBOARD"` ⟹ 退）。**超时**：`0.75` → **`2.5`**，且周期不再手抄 ——
唯一一份写在 `node_core.STATUS_SOURCE_PERIOD_S = 1.0`，节点默认值、自检、单测三处都引它。

**`§二④` 取 (a) 收成单频**：心跳流、`heartbeat_hz` / `heartbeat_always` 两个参数、
`_PARAMS` 条目、`declare_params` 的读取、抽帧逻辑、`summary()` 的心跳计数、**自检的那条心跳检查**
一并删除；每处都留 ☠️ 说明"旧值是什么、为什么没了"，⛔ 没有一处是静默删。

**`§二⑤` 取 (b)**：`state_source=px4` **入口即拒**（`REFUSE_PX4_STATE_SOURCE`，话术里点名
两条 `/fmu/out/*` 是 uXRCE-DDS 专用、并指回 A 级回退是 Phase 1 的 `control` 链）。
两处拒绝：`_validate`（`:1655`）与 `MpcNode.__init__`（`:618-623`）——
后者是给"直接构造节点"那条路兜的（夹具 / REPL），缺了它会**静静地按 mocap 档跑起来**。
⛔ 完全没有实现 (a)（重指 mavros local position）。

**`§二⑤bis` `MAV_FWDEXTSP`**：见下 §四（**今天已留档 = 1**，并进了自检）。

**`§二⑥` 依赖与构建**：

| 票面要求 | 落地 |
|---|---|
| 撤掉 `nmpc` 的 `<depend>px4_msgs</depend>`（`:22`）与其 `:19` 注释 | ✓ 两行换成一条 ☠️ 留痕（写明原注释"对 `bench` 仍成立、对本包不再成立"）。`bench` 那一份**一字未动**（归 §四） |
| `README.md` / `src/nmpc/README.md` 的 source 行 | ✓ 见下面那条 ⚠️ 偏离 |
| `__init__.py:18` / `README.md:22` 那句"除 `offboard_smoke` 外都不许 import `rclpy`/`px4_msgs`" | ✓ 两处都改成"**上表标「无」的那几个模块**不许 import ROS / 具体消息包"，并把原句用 ☠️ **原样留住**（票面明说⛔ 不许默默删） |
| 先核 `preflight_check.py` 有没有 `/fmu/*` / `UXRCE_DDS_CFG` 检查 | ✓ 核了，**票面那句猜测是空的**；但核出一条真的 —— 见下 §五 |

⚠️ **一处偏离（有理由，请核）**：票面写的是"source `~/px4_msgs_ws`"→"`~/mavros_ws`"，**我没做成替换**。
因为今天 `test/test_mpc_node.py` 仍 `from px4_msgs.msg import VehicleStatus`（`§四` 才换夹具）
⟹ 直接换掉会让**照抄文档的人跑不出 §四 之前的红**。所以两处都是**加一行 `~/mavros_ws`、
把 `~/px4_msgs_ws` 留成一行并标 ⚠️"暂时还不能删"**，且都写了"§四 落地时回来核这两处"
（票面 `§四⑤` 点名这两处不许留悬空）。仓库根的完整构造行现在是**三条 source**。

**`§二⑦` seam 1 页那两处**：`docs/control/mpc-node-seam1.md` §11.1 冻结档表的频率行
（`heartbeat_hz 50` 标 ☠️ 删除）、§11.2 那条命令（加 `~/mavros_ws` 并说明 `~/px4_msgs_ws` 为何还在）。
★ **我另外多改了两处**（票面没点名，理由与位置如下 —— 若判越权请当场驳）：
(1) **§2 整节加作废块**：§2.1–§2.4 整节是旧载体（话题表 / 心跳 / 时间戳全在里面），
不改会让同一页"§2 说 `/fmu/*`、§11 说已换载体"自相矛盾。**只作废、不代写新表**
（新表的权威在代码，映射表是 §四 的交付物）。
(2) **§11.3 表里三行**（心跳频率 / 时间戳单位 / 闸门）就地标 ☠️ + 说明该行属于 2026-10-10 的读数，
并在 §11.3 的 `45 passed` 那句下加一条"别把这个数当现状"的注。**用例名一个字没改**（改写归 §四）。

#### 二、三处静默各落在哪条断言上（本票的核心，逐条对）

| 静默 | 机理 | 今天的断言落在哪 |
|---|---|---|
| **N1** 推力反号 | 飞控 `fill_thrust` 会 `thrust_body_array[2] = -thrust`（`mavlink_receiver.cpp:1475`） | `sign_map.mavros_attitude_target()` 的**返回值**：`thrust == +F/T_max`（`test_sign_map.py`，上一轮已落） |
| **N2** 预转换 | mavros 在 `attitude_cb` 里**自己**做 FLU→FRD ⟹ 我们再做一次 = 不转，roll 不受影响、pitch/yaw **静默反向**，而**飞控最终收到的分毫不变** | 只能落在"**我们发出的那个字段**"上：`body_rate_* == ω 原样`。★ 这条与 `§一①` 那句"§8.5 的落点换了对象"是同一件事（上一轮 Comments 已记） |
| **N3** 1 Hz 的闸门源 | `/mavros/state` 跟着 MAVLink HEARTBEAT 走 1 Hz，而旧默认 `0.75 < 1.0` ⟹"状态超时"**恒真**、闸门一直关、一帧都不发 | 常量级：`test_node_core.py::test_状态源一_hz_阈值必须给够余量`（本票新增）；自检级：`check_params` 那条"状态时限 ≥ 闸门源 2 个周期"。**§四 的 seam 1 那条用例（起参数改回 0.75 ⟹ 红）仍在 §四** |

#### 三、`"OFFBOARD"` 这个字符串是怎么核的（票面 `§二③` 点名要写进 Comments）

**核法：三份一手源，互相独立**（都在 `~/mavros_ws` 里，2026-10-12 看的）：

1. `mavros/src/lib/uas_stringify.cpp:138` —— PX4 custom mode → 字符串那张表的建表行：
   `{px4::define_mode(px4::custom_mode::MAIN_MODE_OFFBOARD), "OFFBOARD"}`；
2. `mavros/src/plugins/sys_status.cpp:926` + `:968` —— `/mavros/state` 的 `mode` 字段就是
   `uas->str_mode_v10(hb.base_mode, hb.custom_mode)` 的返回值 ⟹ 上表那一行就是**真机上会看到的串**；
3. `mavros_msgs/msg/State.msg:73` —— `string MODE_PX4_OFFBOARD = OFFBOARD`（消息自带的常量）。
   ★ 代码里**不手写字符串**，用 `State.MODE_PX4_OFFBOARD`；有一条测试把它与 `node_core.MODE_OFFBOARD`
   和那条字面量**三边对齐**（`test_mpc_node.py::test_下行那条路的两个常量也对着真消息类`）。

⛔ 票面那条"不许猜"已满足；**仍未做**的是台上 `ros2 topic echo /mavros/state` 的实机复核（要飞控通电）。

#### 四、⑤bis 留档 + 本轮**新造出来**的一个发现：`thrust_scaling`

**① `MAV_FWDEXTSP` 今天 = `1`（留档）**：`docs/assets/z0.params`，**按名字查**（那份文件自己的表头写着
"行号每次重导出都会变"）⟹ `MAV_FWDEXTSP = 1`。★ 已进自检（`check_downlink_prereqs` 第一条），
且快照缺失时**算失败而不是跳过**（有负控用例）。⚠️ 快照是导出时刻的值 —— 自检的 detail 里写着
"实机起飞前请再读一次**现值**"，并给了 live 读法（先 `ros2 param list | grep` 找它挂在哪个节点，
再 `ros2 param get` —— ⛔ 不硬写节点名）。

**② 新发现：mavros 侧那个 `thrust_scaling` 参数，缺失 / NaN / 0 是三种不同的静默**
（一手：`mavros/src/plugins/setpoint_raw.cpp`）：

| 情形 | 代码 | 后果 |
|---|---|---|
| 参数**缺失**或值为 `NAN`（**声明默认值就是 `NAN`**，`:58-60`） | `auto ignore_thrust = req->thrust != 0.0 && std::isnan(thrust_scaling);`（`:294`） | 只要 `thrust != 0` ⟹ **整帧丢弃**（连 `body_rate` 一起），只打一条 `ERROR_THROTTLE(…, 5000, …)`（`:296-303`） |
| 值为 **`0.0`** | `thrust = std::min(1.0, std::max(0.0, req->thrust * thrust_scaling))`（`:307-313`） | 推力被**强制归零** ⟹ 掉高度（一条 `WARN_THROTTLE`） |
| 值为 **`1.0`** | 同上（乘 1） | 我们的语义（"`thrust` 就是 `F/T_max`"） |

★ **本机现值核过**：`~/mavros_ws/install/mavros/share/mavros/launch/px4_config.yaml:104`
（`/**/setpoint_raw:` → `thrust_scaling: 1.0`），且那份 yaml **确实被加载**
（`px4.launch:13` 的 `config_yaml` → `node.launch:24` 的 `<param from=… />`）。
⟹ 进了自检两条（"`thrust_scaling = 1.0`" + "那份 px4_config.yaml 确实被加载"），
并在 `mpc_node.py` 的模块 docstring 里写成常驻警告（★ 因为它**不是**我们能设的：
它活在 mavros 那半边）。

#### 五、`§二⑥` 点名的 `preflight_check.py` 复核 —— **结果写在这里（票面要求）**

**票面那句猜测是空的，但核出了一条真的**（两条都用同一搜法，免得"没有"取决于搜法）：

| 票面猜的 / 实有的 | 核实 | 处置 |
|---|---|---|
| `/fmu/*` 存在性检查 | **没有**。`grep -n "fmu\|UXRCE\|uxrce" src/nmpc/nmpc/preflight_check.py` ⟹ 零命中；`git show HEAD:` 那一版**也是零命中**（⟹ 不是本轮删掉的，是从没有过） | 无需改 |
| `UXRCE_DDS_CFG` 检查 | 同上，零命中 | 无需改 |
| ★ **实有的（票面没猜到）**：`heartbeat_hz ≥ 2 Hz` 检查（HEAD `:129-136`） | 心跳流已删（`§二④`）⟹ 这条检查失去对象 | **删除**，原地留 ☠️ |
| ★ **实有的（票面没猜到）**：`ok = p.status_timeout_s >= 0.5`，名"状态时限 ≥ 上游一个周期"，理由写在 detail 里（"假 PX4 与真 commander 都是 **2 Hz**"；HEAD `:158-161`） | 换载体后源是 **1 Hz** ⟹ **旧判据在真机上恒绿而恒错**：`0.75`（旧默认）满足 `>= 0.5`，而 `0.75 < 1.0` 让超时判据恒真 | **判据整个换掉**：`>= 2 × STATUS_SOURCE_PERIOD_S`，名字改成"状态时限 ≥ 闸门源 2 个周期" |

★ 换判据时顺手把一处**冗余条件**收掉了：原来是
`p.status_timeout_s > period and p.status_timeout_s >= 2 * period`，而 `period > 0` 时后半个
蕴含前半个（前半是死条件）⟹ 现在只留 `>= 2 * period`，**并把两条失败理由分开写**
（`<= period` ⟹ 恒真；`< 2*period` ⟹ 丢一拍就误判）。这条收紧由**新增的 `1.5` 那一档**钉住：
它是这条用例里**唯一**能区分 `> period` 与 `>= 2*period` 的取值（`0.75` / `1.0` 在两个判据下都红）。

#### 六、本轮**新造 / 仍然留着**的悬空引用（只登记，除注明外不改）

| 引用处 | 今天的状态 | 归谁 |
|---|---|---|
| `src/nmpc/test/test_mpc_node.py`（全文件） | 仍 import `px4_msgs`、仍钉 `/fmu/*`（约 25 条）⟹ **本票此刻是红的，且我**没有**跑它** | `§四③` |
| `src/bench/bench/fake_mavros.py` | 没有 `/mavros/setpoint_raw/attitude` 那条流 | `§四①` |
| `src/bench/bench/config.py`、`fake_px4.py`、`bench/package.xml` 的 `px4_msgs` | 未动 | `§四②④` |
| `tools/px4-sitl/verify_offboard_smoke.py` 全文 + `test_verify_judges.py:123` | 仍按 `/fmu/*` 起 DUT，且场景表传**已删**的 `--timestamp-unit` / 旧名 `--break pitch-sign` ⟹ 今天跑不通 | `§四`（上一轮已登记，状态未变） |
| `docs/control/ladder-acceptance-criteria.md:64/:127/:183` | 缺 `--t-max` / 写着"默认 11.8 N"（今天为假） | `§五` |
| **本轮新发现**：`preflight_check.check_signs` / `check_normalization` 的 `impl=` 仍**只**对 `px4_rates_setpoint` 对账 | mavros 那条入口**没有**同等的符号/归一化三方对账 ⟹ 换载体后"对账覆盖了哪条路"这件事**没有断言** | 本票**没改**（它要动 `tools/sign-map/sign_map_core.py`，属另一节的 scope）—— **登记待裁** |
| **本轮新发现**：核心层"零消息包"那条契约**只盯两个名字** | `test_sign_map.py` / `test_ocp_health.py` / `test_node_core.py` 里那三条断言查的是 `rclpy` / **`px4_msgs`**；**没有一条**查 `mavros_msgs` ⟹ "不许 import 具体消息包"目前是**写在 README 里的口径**，不是被断言守住的事实（已如实写进 `__init__.py` / `README.md` 的对应句） | 本票**没加**（加一条"没鉴别力"的断言不如不加）—— **登记待裁** |

#### 七、测试与套件状态（本轮实测，不是推算）

```bash
source /opt/ros/humble/setup.bash && source ~/mavros_ws/install/setup.bash \
  && source ~/px4_msgs_ws/install/setup.bash && source ~/ros_ws/install/setup.bash
export ACADOS_SOURCE_DIR=$HOME/acados && export LD_LIBRARY_PATH=$ACADOS_SOURCE_DIR/lib:$LD_LIBRARY_PATH
```

- `colcon build --packages-up-to nmpc` ⟹ **`bench` + `nmpc` 2 包 finished**（
  ⚠️ `mpc_node.py` 是入口脚本 ⟹ 它改了**必须** build，`node_core.py` 改了不用）；
- `test_preflight_check` + `test_node_core` + `test_mavros_interface` + `test_params`
  + `test_sign_map` + `test_commands` ⟹ **199 passed, 0 failed（4.81 s）**；
- `python3 nmpc/preflight_check.py` ⟹ **`✓ 26 项全部通过`**；
  ★ **反向也验了**：**不** source `~/mavros_ws` 时是 **`✗ 25 项里 1 项不过`**
  （`thrust_scaling` 那条**报"验不了"**，不是跳过也不是绿）—— 这是这条检查的**设计行为**，不是缺陷；
- **变异测试 2 次（都为确认新断言有鉴别力）**：

  | 变异 | 期望 | 结果 |
  |---|---|---|
  | `node_core.StopConfig.status_timeout_s` `2.5` → `0.75`（=`§二③` 的那个旧值） | 常量守卫红 | ✅ `1 failed, 67 passed`：`assert 0.75 > 1.0` |
  | 自检门槛 `>= 2 * period` → `> period` | 新增的 `1.5` 那一档红 | ✅ `1 failed`：`status_timeout_s = 1.5 … ok=True` 而用例要 `False` |

  （都在 `/tmp` 的副本上做，用 `PYTHONPATH` 指向副本 ⟹ **没碰共享工作树**；副本已删。）

⛔ **我没有跑 `test_mpc_node.py`**：它今天必然红（钉在 `/fmu/*` 上），跑了只会产出一个
"红"的读数而说不出新东西；它的改写属 `§四③`，改完再跑。

#### 八、本轮**没做**的（与票面对齐）

`§四`（夹具 + seam 1 + 那三份驱动）、`§五`（阶梯判据页四行落地）、`§六`（结果放置），
以及**任何一次上飞机的跑场**。**操作者侧仍待办**（要飞控通电 / 起 mavros，⛔ 车侧动作我不代跑）：
① `MAV_FWDEXTSP` 的**现值**；② `ros2 topic hz /mavros/state` 实测频率（本文的 1 Hz 是**源码**结论，
不是本机读数）；③ `ros2 param get` 那条 `thrust_scaling`（launch / 命令行可以覆盖那份 yaml）。
