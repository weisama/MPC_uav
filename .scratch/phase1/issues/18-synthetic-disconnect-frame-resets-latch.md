# 18: mavros 的**合成断连帧**会把 `armed` 降沿伪造出来 —— 已踢锁的降级会在空中被复位

Status: needs-triage
Type: task

**为什么单开一张：** ticket [06](06-control-failsafe-paths.md) 的第二轮复核（2026-09-22）顺着
"`/mavros/state` 超时这条判据的证据文案对不对"往下查，发现真机（不是台架）上会**多出一帧我们没预料的
`/mavros/state`**，它会把"落地上锁才复位"的闩锁**在空中**清掉。这要动的是**安全语义**（什么才算
"可信的落地上锁"），不是一句注释能改的 —— 按仓库纪律单独立票、等决策。

**第一手证据（mavros 2.15.1，commit `22ae5b7cc7cdb4cb9c2070a8213c72dae445a23e`，
`/home/chen/mavros_ws/src/mavros`）：**

1. `src/plugins/sys_status.cpp:903-911` `publish_disconnection()` 发的状态是
   `connected=false, armed=false, guided=false, mode=""`，`system_status=UNINIT`。
2. 它**不是**只在启动时发（`:647`）：`:1411` 在 `connection_cb(connected=false)` 里也会发。
3. 触发路径：`:512` 声明参数 `conn_timeout`（默认 **10.0 s**）→ 建一个该周期的 wall timer
   （`:516-518`）→ `timeout_cb()`（`:1311`）→ `uas->update_connection_status(false)` → `connection_cb`。
   ⟹ **HEARTBEAT 停掉 10 s 后，mavros 自己往 `/mavros/state` 上发一帧"断开"。**

**本节点会怎么走（第一手读过自己的代码）：**

| 时刻 | 发生了什么 |
|---|---|
| HEARTBEAT 停 | `/mavros/state` 不再有新帧（mavros 只在收到 HEARTBEAT 时发这个话题，实测 1 Hz） |
| +2.5 s | 本节点的 `state_timeout` 成立（`armed` 仍是**最后一帧的真值** = true）⟹ 停发 setpoint + 请求 `AUTO.LAND`，闩锁 `kArmedDegrade` |
| +10 s | mavros 的合成帧到达：`connected=false, armed=false` ⟹ `fresh(state)` 为真而 `connected=false`（判据**仍成立**，日志继续报 `state_timeout`），但 `armed` 出现降沿 ⟹ `fsm.hpp:396` 的复位分支命中（**它只判 `armed`，不判 `connected`**）⟹ 闩锁清空、节点回 `INIT`、**不再请求任何模式** |

**为什么这本身不算错：** 链路已经断了，`set_mode` 请求根本到不了 PX4，继续请求只是刷日志；
这种场面的兜底是 PX4 自己的失效保护（`COM_OBL_RC_ACT` / `NAV_RCL_ACT`）。

**⚠️ 真正的风险 —— "链路回来"那一刻：**

那帧合成的 `armed=false` 已经把闩锁清掉了。若链路恢复（mavros 重连、串口回来）时 PX4 **仍然 armed**
（很可能：我们停发之后它自己退出了 OFFBOARD，但没落地），那么：

1. `/mavros/state` 恢复 ⟹ `guards` 重新成立（`armed=true`、`connected=true`）；
2. 节点从 `INIT` 直接回到 `STANDBY` 并**开始流 setpoint**，前置流满足后**重新请求 `OFFBOARD`**
   —— 即**自主重新接管**，而"已解锁的输入失效要落地上锁才复位"这条闩锁本来就是要拦住它。

**要决策的问题（这是本票的核心，别直接改代码）：**

| 方案 | 含义 | 代价 |
|---|---|---|
| (a) 把 `connected` 纳入"落地上锁"的判据：只有 `connected=true && armed=false` 才认作落地上锁 | 合成的断连帧不再复位闩锁 | 真"链路断了之后飞机被人工落地上锁"时我们也**看不到**那次上锁，闩锁要等链路恢复后才可能复位 |
| (b) 单独记"见过断连帧"标志，遇到它就把降级升级成"需要人确认"（例如要求模式被外部交回 OFFBOARD 才复位） | 最保守 | 引入一个新的闩锁种类与对外语义（要同时改 `CONTEXT.md` / README / §4.4） |
| (c) 维持现状，只把这条**记账+告警**（链路恢复时若 armed 且闩锁曾被合成帧复位 ⟹ 打一条显眼的日志，并要求人不许在那种场面靠近飞机） | 改动最小 | 靠人守规矩 |

- [ ] 选一个方案（需要用户/设计决策），写进 `fsm.hpp` 的守卫或复位判据 + `CONTEXT.md` 的「失效降级」条
- [ ] 补 seam 2 单测：合成断连帧**不该**复位闩锁（或按所选方案断言新语义）；再补"真落地上锁仍然复位"
- [ ] **夹具能力**：让假 mavros 能发这一帧（`FakeMavros` 目前只有 `mute("state")` 造静默，
      不假扮 mavros 自己的 `conn_timeout` 逻辑）⟹ 之后 seam 1 也能验这条
- [ ] 复核 `test_control_failsafe.py` 的 state 分支说明：它现在证明的是"静默"那一半（已在 §4.4(3) 注明）

**Blocked by:** 无（ticket 06 已落地；本票改判据与文档）
**与 ticket 08 的关系：** 影响的是"已解锁时降级怎么复位"，起飞路径（08）落地前应有答案。
