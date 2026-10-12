# 13: 口径冻结 —— mavros 下行的三处所有权 + `mpc-node-seam1.md` §12.3 三条未决

Status: ready-for-agent
Type: docs

**What to build:** **(C) 级下行改走 mavros 之后，把"哪一步由谁负责"这件事在文档里写死。**
本票**只动文档，一行 Python 都不改**（代码在 [14](14-mpc-node-mavros-downlink.md) /
[15](15-offboard-smoke-mavros.md) / [16](16-fixtures-and-seam1-mavros.md)）。
规范来源：[`spec.md`](../spec.md) → Implementation Decisions → **§★★ 2026-10-12**（起因 / 等价性 /
三处不可等价 N1–N3）。

**Blocked by:** 无（纯文档，且起因那批读数是第一手的）。

**它挡的是什么**：换载体本身不难，**难的是"换了之后哪一步归谁"没有一个共同口径**。
没有这份口径，票 14/15/16 会各自按自己的理解写，而三处差异（N1/N2/N3）
**全都以"静默"的形式出错** —— 判据全绿、飞机朝错的方向走。本票就是先把这三条钉在纸上。

---

## 逐项落点

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
  （那条断言归 [16](16-fixtures-and-seam1-mavros.md)，**本票只负责在文档里把这个论证写出来**）。
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
| ② **谁解锁 / 谁切模式** | **仍然无主**，但换成 mavros 形态：接口从 `/fmu/in/vehicle_command` 变成 `/mavros/cmd/arming` + `/mavros/set_mode`；候选归属（QGC 手动 / 把 `ArmAgent` 抬出来 / `offboard_smoke.py` 自己干）**一个没少**。★ 归属由 [14](14-mpc-node-mavros-downlink.md) 裁，本票只把"仍是未决"写清楚。 |
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
逐处**留痕或转票**，⛔ 不留悬空引用。**本票不改这些**（它们是 14/15/16 的落点），只**登记成一张清单**
放进 Comments —— 清单要带路径 + 行号 + 归哪张票。

---

## 验收

- [ ] `sign-map.md` §2 的表有"谁执行"一列，且 mavros 那一路的两处**都指向别人的代码**（带文件:行号）
- [ ] `sign-map.md` 新增的所有权对照节，px4_msgs 与 mavros 两条路**逐行对照**，锚点齐全
- [ ] `sign-map.md` §2 已分成"物理约定（不变）"与"落到线上那几个字段（按通路分叉）"两半
- [ ] §9 那四件自检的**适用范围**被写清；三条反例**没有被删**，而是被重新论证成"所有权转移的安全性证明"
- [ ] §8 第 5 条契约的落点已按新载体重新表述
- [ ] `mpc-node-seam1.md` §12.3 ① ② ③ **逐条**有 ☠️ 或改写，**原文可读**（能看出改前是什么）
- [ ] `px4-interface.md` §4.3 从"开放决策"变"定案"，带日期与起因；§5.1 多一行 `setpoint_raw/attitude`
- [ ] `px4-interface.md` §8 的两条检查项**就地标 ☠️**（不是删除），且该节其余条**未被顺手降级**
- [ ] §8 **新增了 `MAV_FWDEXTSP = 1` 这一条**（它是硬前置：为 `0` 时整条下行静默死掉）
- [ ] 活文档里"下行 = px4_msgs / `/fmu/*`"的位置**有一张登记清单**（路径 + 行号 + 归哪张票），**无悬空引用**
- [ ] 全程没有改任何 `.py` / `.xml` / 参数文件（`git status` 里只有 `docs/**` 与 `.scratch/**`）

---

**为什么是 `ready-for-agent`**：纯文档、纯核对，锚点已在 [`spec.md`](../spec.md) §★★ 的表里给全，
不需要台架、不需要飞机、**不需要操作者裁定**（"归谁"那类裁定已被本票**排除在范围外**，
它明确留给 [14](14-mpc-node-mavros-downlink.md)）。

**★ 本票是纯 AFK 票**（[14](14-mpc-node-mavros-downlink.md) / [15](15-offboard-smoke-mavros.md) /
[16](16-fixtures-and-seam1-mavros.md) 都排在它后面）。

---

## Comments

### 2026-10-12 —— 开票时已核过的第一手（省得开工时重搜）

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
