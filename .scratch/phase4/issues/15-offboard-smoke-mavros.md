# 15: `offboard_smoke.py` 换成 mavros 下行 + `--t-max` 那笔账

Status: ready-for-agent
Type: task

**What to build:** 把阶梯 1/2 的**驱动工具** `src/nmpc/nmpc/offboard_smoke.py` 从 px4_msgs 那条路
搬到 **`/mavros/setpoint_raw/attitude`**（解锁 / 切模式 / 闸门一并换），
并**结清一笔已经存在的归一化错账**（`--t-max` 默认 11.8 N —— 见 ②）。

**Blocked by:** [13](13-docs-mavros-downlink-canon.md)（口径）。
口径不在本票决定；`sign-map.md` 里"谁取负"那张表怎么说，这里就怎么填。

**它挡的是什么**：这个工具是**阶梯 1/2 唯一的执行者**。
它错了，阶梯 1/2 的结论就是错的 —— 而 ② 那笔账**今天就错着**（与换不换通路无关）。

★ **本票碰的是判据页的驱动工具** ⟹ 落地后**必须**按
[`ladder-acceptance-criteria.md`](../../../docs/control/ladder-acceptance-criteria.md) **§7.1 留痕**：
点出哪一级的哪个数 → 在 §7.2 增一行 → 被废的那条**就地标 ☠️**。
**留痕本身是票 [17](17-ladder-criteria-trace.md) 的交付**；本票只负责**把需要留痕的事实写成清单**
（哪一级哪一行、为什么、换成什么），交给 17。

---

## ① 接口换掉（与 [14](14-mpc-node-mavros-downlink.md) 同一套口径）

| 现在 | 换成 |
|---|---|
| pub `/fmu/in/offboard_control_mode` | **删掉** —— 心跳由飞控从收到的 `SET_ATTITUDE_TARGET` 派生（`mavlink_receiver.cpp:1531-1535`） |
| pub `/fmu/in/vehicle_rates_setpoint`（`VehicleRatesSetpoint`） | pub **`/mavros/setpoint_raw/attitude`**（`mavros_msgs/AttitudeTarget`），`type_mask = 128` |
| pub `/fmu/in/vehicle_command`（`:289` 解锁 / `:294` `DO_SET_MODE`） | **`/mavros/cmd/arming`** + **`/mavros/set_mode`** |
| sub `/fmu/out/vehicle_status` | sub **`/mavros/state`**（`armed` 且 `mode == "OFFBOARD"`） |
| `from px4_msgs.msg import …` | `from mavros_msgs.msg import AttitudeTarget, State` |

- **`_map()`**（现在调 `sign_map.px4_rates_setpoint()` 再写 `rates.thrust_body[2]`）改成调
  [14](14-mpc-node-mavros-downlink.md) 里那个**具名的 mavros 入口**：`body_rate` 填 FLU 原样
  （⛔ 不许自己预转换 —— mavros 会转，见 `spec.md` §★★ N2），`thrust` 填 **`+F/T_max`**（飞控加负号）。
- **心跳时序**：现在的 `--pre-stream`（默认 1.5 s）是"先进 OFFBOARD 再解锁"那套流程里的等待。
  飞控侧仍要求 OFFBOARD 前先收到流，所以这个**语义大致保留**，但要按 mavros 的实际时序重核一遍
  （`/mavros/state` 是 **1 Hz** 上报 ⟹ **等它变成 OFFBOARD 要按秒算，不能按 100 ms 算**）。
  ★ 这是最容易"看起来能跑但判据恒真"的地方：断言"闸门已开"的那个轮询必须**长于 1 s 的周期**，
  否则它会**在飞控还没上报之前就判绿**（或恒红）。
- `_CHECKED_POSITIVE`（`--rate-hz` / `--cmd-interval` / `--pre-stream` / `--mass` / `--t-max`）**保留**。

## ② ★★ `--t-max` 那笔账：**默认值是错的**（与换通路无关，今天就错着）

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

**怎么做**（三条都要）：
1. **默认值改成正确的那个常数**（46.3027 N），或在帮助文本里写明它**必须**由操作者给 ——
   ⛔ **不许**留一个会被静默用错的默认值。**选哪种要写进 Comments。**
2. ★★ **加一条自检**：悬停那一档（`悬停倍数 = 1.0`）归一化之后，**必须等于悬停油门**
   （`m·g / t_max` 对 `hover_thrust` 的容差断言，容差用生产阈值口径，**不要写 `== 0`**）。
   **这条自检正是能把 11.8 这个错值当场变红的东西** —— 换通路之前也该有。
   **变异测试**：把 `--t-max` 改回 `11.8` ⟹ 必须变红。红不了就是没鉴别力。
3. **把"乘的基准"与"归一化常数"在输出里分开打印**（现在 `describe()` 打的是
   `F=…N (…×悬停)`）⟹ 再加一列"归一化后发出的是多少"，**让 ② 这类错误在屏幕上可见**。

## ③ 故障注入开关：**逐个**处置（⛔ 不许整段留下不动）

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

## ④ 「悬停倍数」这一列的语义

- **表头与基准不变**：基准恒为 `m·g`（`commands.hover_force_n()`），
  `--mass` 仍是"全重 kg"。
- **但这一列下面那个数换了身份**：以前它最终变成一个 `thrust_body_z`（我们自己取负），
  现在它变成 `AttitudeTarget.thrust`（**正号**，负号由飞控加）。
  ⟹ ⛔ **换载体前后的"悬停倍数"读数不许并列比较**（见 `spec.md` §★★ N1）。
- 见 ② 第 3 条：把"归一化后发出多少"也打出来，这一列的语义才在屏幕上自洽。

---

## 验收

- [ ] 工具发的是 `/mavros/setpoint_raw/attitude`，**不再**发任何 `/fmu/*` 话题（grep 现算）
- [ ] 解锁走 `/mavros/cmd/arming`、切模式走 `/mavros/set_mode`，闸门读 `/mavros/state`（`armed` + `mode`）
- [ ] `body_rate` 是 **FLU 原样**（⛔ 无预转换），`thrust` 是 **`+F/T_max`**
- [ ] **等闸门的那段轮询周期 ≥ `/mavros/state` 的发布周期**（1 Hz），⛔ 不是 100 ms 级的
- [ ] `--t-max` 的默认值**不再**是 11.8 N（或改成必须显式给），且**选法写进 Comments**
- [ ] 有"悬停档归一化后 == 悬停油门"的**自检**；**变异测试**把 `--t-max` 改回 11.8 **必须变红**
- [ ] 输出里"乘的基准"与"归一化常数"**分开可见**（② 那类错误在屏幕上看得见）
- [ ] `--timestamp-*` 那几档**已处置**（删或标 dead），且 Comments 里写明理由（飞控不读 `time_boot_ms`）
- [ ] `--break pitch-sign` / `yaw-sign` 的含义变化**在帮助文本里写清**（= N2，不是"符号写反"）
- [ ] `--qos` 的取值集合已按新载体重核
- [ ] 需要 §7.1 留痕的地点已整理成一份清单（哪一级 / 哪一行 / 为什么 / 换成什么），交给 [17](17-ladder-criteria-trace.md)
- [ ] **没有**改 [14](14-mpc-node-mavros-downlink.md) 的落点（`mpc_node.py` / `node_core.py` 一行不碰）

---

**为什么是 `ready-for-agent`**：纯软件 + 台架上就能跑（`--break` 那几档本来就是干这个用的），
**不需要飞机通电**。★ 真机复核（拆桨目视那一步）属于 [09](09-ladder-ground-and-tether20.md)，
**本票不碰飞机**。

---

## Comments

### 2026-10-12 —— 开票时已核过 / **未核**的

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
1. `/mavros/state.mode` 在 OFFBOARD 时的**字符串**（与 [14](14-mpc-node-mavros-downlink.md) ③ 同一处，两票共用一个答案）。
2. `--qos` 现在的取值集合与 mavros 收端 `SensorDataQoS` 的**实测**兼容性（与 [14](14-mpc-node-mavros-downlink.md) 未核第 3 条同源）。
3. 真机上**没有**跑过一次。
