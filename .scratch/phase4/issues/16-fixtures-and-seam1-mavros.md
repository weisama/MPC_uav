# 16: 夹具 + seam 1 —— 用 mavros 形状重跑话题边界测试

Status: ready-for-agent
Type: task

**What to build:** 把 seam 1（喂脚本输入 + 断言发出话题 + 假 peer）**换成 mavros 形状再跑一次**，
并补齐它需要的夹具。

**Blocked by:** [14](14-mpc-node-mavros-downlink.md)（DUT 得先会说 mavros 的话）。

★ **不新增 seam 形态** —— 这是**同一个形态的第 4 次应用**（Phase 1 → Phase 2 → Phase 4 首次 → 本次），
见 [`spec.md`](../spec.md) → Testing Decisions → §★★ 2026-10-12。

**它挡的是什么**：换载体之后的"**搬到真机后话题/符号/超时改错了**"这一类错误。
★ 本票是全套里**唯一**能在台架上把 N1/N2/N3 三处**逐条证伪**的地方 ——
真机阶梯只能证明"没摔"，证明不了"我们发的那一条消息里 `body_rate` 的值是对的"。

---

## ① 夹具：`src/bench/bench/fake_mavros.py`（Phase 1 已有，**加一条流**）

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
  （这正是 [14](14-mpc-node-mavros-downlink.md) ③ 那个超时坑的由来）。
  ⛔ 若夹具图快把它做成 10 Hz，那个坑在台架上就**永远不会出现**。

## ② `src/bench/bench/config.py`

- `DEFAULTS["topics"]` 加/改：`/mavros/setpoint_raw/attitude`；
- `DEFAULTS["rates_hz"]` 里 `state` 已是 `1.0`（**对**，别改快）；
- `DEFAULTS["px4"]["topics"]` 那组（`status` / `rates` / `mode`）：**本票要判它的去留**（见 ④）。

## ③ `src/nmpc/test/test_mpc_node.py`（2125 行 / 41 条；**约 25 条**依赖 `/fmu/*`）

**做法**：不是"删掉 25 条"，而是**逐条映射**（映射表本身就是交付物，写进
[`mpc-node-seam1.md`](../../../docs/control/mpc-node-seam1.md)，体例同它的 §11.3）：

| 类别 | 处置 |
|---|---|
| **话题名 / QoS**（接口那一节） | **逐条换**成新话题与新 QoS（含"QoS 不匹配就收不到"的负控） |
| **符号 + 推力归一化**（本节） | ★★ **重写断言对象**：断言落在**我们发出的那条消息**的 `body_rate` 与 `thrust` 上（见 ① 最后一条）。**必须有一条专测 N2**：断言 `body_rate` 是 **FLU 原样**（`pitch/yaw` **没有**被我们预先取负） |
| **闸门**（未解锁 / 未在 OFFBOARD / status 过期 ⟹ 不发布） | 数据源换成 `/mavros/state`；★ **超时那条必须变异测试**过（改回 `0.75` ⟹ 变红） |
| **时间戳**（那一节） | ⛔ **本类在新载体上没有对应物**：飞控不读 `time_boot_ms`（`mavlink_receiver.cpp:1508-1598` 整个 handler 里没读）⟹ **删除或标记为"不再可测"**，并在映射表里写明**为什么**。**不许留成"绿着但不测任何东西"** |
| **频率与 RViz**（那一节） | 按 [14](14-mpc-node-mavros-downlink.md) ④ 选定的心跳方案改（收成单频 / 双频重解释） |
| **A 档**（`state_source="px4"`） | 按 [14](14-mpc-node-mavros-downlink.md) ⑤ 的裁定改：选 (b) 的话断言"入口当场拒绝" |
| **求解失败兜底 / NaN 整帧丢弃 / 进程收尾** | **语义不变**，只换消息类型 |
| **轨迹参考 / 输入合理性** | **不动**（与下行无关） |

★ `FROZEN_PARAMS`（`tau_s=0.0`）等冻结档**保留**；起参数里与下行相关的（若有）同步换。
★ `DUT = ("nmpc", "mpc_node.py")` 与 `Scenario(dut=…, px4=…)` 的**开关语义**要改：
现在 `px4=True` 起的是 `fake_px4`。**新增一个 mavros 档位**（而不是把 `px4=True` 偷偷解释成 mavros）——
⛔ 名字撒谎比多一个参数贵。

## ④ `src/bench/bench/fake_px4.py` 的去留 —— **显式判，不许留着不管**

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

## ⑤ 文档：`docs/control/mpc-node-seam1.md` 追加一节

按它的既有体例（§11 的形态）追加 **§12.4 或 §13**：这次跑的是什么、映射表、读数、
**本节没证明什么**（照 §12.4 的口径写）。
★ §11.1 冻结档表与 §11.2"可整段复制的命令"若被本票或 [14](14-mpc-node-mavros-downlink.md) 改动，
**两票都不许留悬空** —— 谁先落地谁核一遍这两处（**先核再改，别各改一半**）。

★ **"本节没证明什么"里要新增一条**：**`MAV_FWDEXTSP`（硬前置，=1 才收 setpoint）夹具模拟不了**
—— 它是**真飞控的参数**，台架上没有它的对应物 ⟹ 这一条**只能在实飞前检查清单里兜**
（归 [13](13-docs-mavros-downlink-canon.md) ③ / [14](14-mpc-node-mavros-downlink.md) ⑤bis）。
⛔ 别让夹具"顺手"把它模拟掉 —— 那会把一条**真实存在的静默失效点**变成一个绿的用例。

---

## 验收

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

---

**为什么是 `ready-for-agent`**：纯台架。★ 这是本批票里**验收最硬**的一张 ——
N1/N2/N3 三处里，只有这里能**逐条**证伪。

---

## Comments

### 2026-10-12 —— 开票时已核过 / **未核**的

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
