# 18: mavros 的**合成断连帧**会把 `armed` 降沿伪造出来 —— 已踢锁的降级会在空中被复位

Status: done
Type: task

> ✅ **2026-09-22 收口（终态）**：用户裁定选 **(a)**「`connected` 纳入"落地上锁"的判据」+ 把「可信」同时
> 落在 **`was_armed_` 的冻结**上；本票当场实现并收口为 **`done`** —— 判据 + 夹具能力 + seam 2 三条 +
> seam 1 两条，全绿（`control` 112 例 + `bench` 19 例自检，0 失败；lint 只剩既有 172 条 vendored）。
> 判别力矩阵、第一手更正与刻意接受的代价见文末「实现记录」。

> ↻ **下面是本票收口**之前**的状态（2026-09-22 早些时候的裁定，保留作历史）**：
> ✅ **保留本票**（同期被删的是 17/19/20 三张"主线之外、前提已失效或未复现"的票）。
> 理由：这是全仓唯一一张记录「节点可能自主重新接管」的票，且它约束的正是 ticket 08 的起飞路径
> —— 删掉它，这个风险就从 tracker 里消失了，没有任何地方再提。
> **当时状态从 `needs-triage` 改为 `ready-for-human`**（卡在**人的决策**上，不是在等更多信息 ——
> 下面「要决策的问题」那张表的 (a)/(b)/(c) 需要人来选，选完才轮到改判据）；**该决策已于同日做出，见上**。

**为什么单开一张：** ticket [06](06-control-failsafe-paths.md) 的第二轮复核（2026-09-22）顺着
"`/mavros/state` 超时这条判据的证据文案对不对"往下查，发现真机（不是台架）上会**多出一帧我们没预料的
`/mavros/state`**，它会把"落地上锁才复位"的闩锁**在空中**清掉。这要动的是**安全语义**（什么才算
"可信的落地上锁"），不是一句注释能改的 —— 按仓库纪律单独立票、等决策。

**第一手证据（mavros 2.15.1，commit `22ae5b7cc7cdb4cb9c2070a8213c72dae445a23e`，
`/home/chen/mavros_ws/src/mavros`）：**

1. `src/plugins/sys_status.cpp:903-911` `publish_disconnection()` 发的状态是
   `connected=false, armed=false, guided=false, manual_input=false, mode=""`，`system_status=UNINIT`。
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

- [x] 选一个方案（需要用户/设计决策），写进 `fsm.hpp` 的守卫或复位判据 + `CONTEXT.md` 的「失效降级」条
- [x] 补 seam 2 单测：合成断连帧**不该**复位闩锁（或按所选方案断言新语义）；再补"真落地上锁仍然复位"
- [x] **夹具能力**：让假 mavros 能发这一帧（`FakeMavros` 目前只有 `mute("state")` 造静默，
      不假扮 mavros 自己的 `conn_timeout` 逻辑）⟹ 之后 seam 1 也能验这条
- [x] 复核 `test_control_failsafe.py` 的 state 分支说明：它现在证明的是"静默"那一半（已在 §4.4(3) 注明）

**Blocked by:** 无（ticket 06 已落地；本票改判据与文档）
**与 ticket 08 的关系：** 影响的是"已解锁时降级怎么复位"，起飞路径（08）落地前应有答案。

---

## 实现记录（2026-09-22）

**方案：用户裁定选 (a)**（`connected` 纳入"落地上锁"的判据），并明确"把「可信」落在**复位判据 +
冻结 `was_armed_`** 两层"。落点：`fsm.hpp::update()` 帧首的 `was_armed_` 刷新（`:761`）与 ① 的复位判据（`:787`）。

★★ **票面没写、读代码才发现的一件事：只做 (a) 会把"可复位"变成"单向 latch"。**
复位判据是 `armed` 的**降沿**，而旧的 `was_armed_ = guards.armed` 是**无条件**刷新的 ⟹ 那帧合成状态的
`armed=false` 会把降沿**吃掉**，链路恢复时"真的落地上锁"（`connected=true ∧ armed=false`）再也对不上边
⟹ 闩锁永不复位、节点永远停在 `MANUAL/INIT`（只能重启）。这正是 `fsm.hpp` 里新注释说的
"两处是同一条语义的两半，缺一不可"。**判别力是实测的**（seam 1 两条用例 × 三种实现）：

| `fsm.hpp` 的状态 | `…仍走_LAND_不重新接管` | `…已落地上锁_仍然复位` |
|---|---|---|
| 完整修复 | ✅ 过 | ✅ 过 |
| 半边（只加 `connected`，不冻结） | ✅ 过 | ❌ 红（停在 `MANUAL/INIT`） |
| 全撤（原状） | ❌ 红（回 `MANUAL/INIT`，闩锁被清） | ✅ 过（那帧伪造的降沿"代劳"了复位） |

★ **另一件票面判断需要更正的事**：票面 ⚠️ 段的"链路恢复 ⟹ 节点回 `AUTO/STANDBY` 并**重新请求
OFFBOARD**"在当前代码上**不会**发生 —— 第一手追踪：那帧伪造的降沿会顺手把 `arming_by_us_` 清掉
（`px4_interface.cpp::on_state`，`armed==false` 就清），于是恢复时的上升沿得到 `own_arming=false`
⟹ `kArmingByUs` 判据不成立 ⟹ 节点停在 `MANUAL/INIT`。**但这条安全网是偶然的**（它依赖
"合成帧的 `armed=false` 恰好也清了归属标记"），只有**陈旧 `arm_request_pending_`** 这个窄口能透过去
⟹ 本票的判据收口仍然是必要的，只是**风险面比票面写的小**。实测红/绿见上表。

★ **刻意接受的残留在本次实现里对外的样子**（已知、未改，超出裁定范围）：
合成帧之后的失联期间，`guards.armed=false` ⟹ ⑤ 不再请求 `AUTO.LAND`、`/control/state` 掉到
`MANUAL/INIT`（**闩锁其实还在**），而 `control_node.cpp` 里那条"失效已闩锁"的**限流告警**带
`last_phase_ != kInit` 的门 ⟹ 这段时间不打它（`degrade=state_timeout` 的 ERROR 每 10 s 仍在打，
文案里就写着"**已闩锁**：不会自己恢复"）。备选的一行改法（⑤ 里把已闩锁的 `kArmedDegrade` 当"已知已解锁"）
**没做**。已写进 §4.4 的 ★★「可信的落地上锁」与 `test_fsm.cpp` 的用例注释。

**改动清单：**

| 文件 | 改了什么 |
|---|---|
| `src/control/include/control/fsm.hpp` | 帧首 `was_armed_` 改为**只在可信帧上刷新**；① 的复位判据加 `guards.connected`（两条各有一段 ★★ 注释写着为什么缺一不可） |
| `src/control/test/test_fsm.cpp` | +3 条（62 例）：合成帧不清闩锁、合成帧之后仍 armed ⟹ 仍走 `LAND` 且不重请求 OFFBOARD、合成帧之后已上锁 ⟹ 仍然复位 |
| `src/bench/bench/fake_mavros.py` | +`synth_disconnect()` / `reconnect_state()` / `_publish_disconnect()`：复刻 `publish_disconnection()` 那帧（**常量字段**、**只发一帧**），不假扮 `conn_timeout` 计时逻辑 |
| `src/bench/test/test_bench_selftest.py` | +1 条自检（19 例）：那帧只发一帧、六个字段是常量、且**没改夹具自己的属性** —— 带正控 |
| `src/control/test/test_control_failsafe.py` | +2 条（19 例）：共用骨架 `_latch_then_synth_disconnect()`（已解锁 ⟹ 静默 ⟹ 闩锁 ⟹ 那一帧），分别验"恢复后仍 armed"与"恢复后已上锁" |
| `CONTEXT.md` / `docs/control/phase1-implementation.md` §4.4 / `src/control/README.md` / `src/bench/README.md` / `docs/control/bench-runbook.md` | 「可信的落地上锁」的定义与代价、判别力矩阵、夹具能力与自检清单、真机清单里"上锁那一下链路必须是活的" |

★ 顺带：**本票里同一个坑踩了两轮**（本仓有记录、仍然踩了）：`assert sc.wait_for(...)` —— `wait_for`
成功时返回 `None` ⟹ 断言恒假。先是 bench 自检里写了两处（当场改成 `sc.poll()`），随后 seam 1 的两条
新用例里又写了两处（跑出来是红的，第一眼差点当成"实现错了"—— 实际是**测试自己的假红**）。
`_assert_requests_recorded` 的 docstring 里就写着这条 ⟹ 说明"知道纪律"不等于"手感在"；
本票的改法是：`wait_for` 只当**语句**用，要断言一律 `sc.poll()`（并在 seam 1 那两处就地写了注释）。

---

## 复核（2026-09-22，`/code-review` 的 Standards + Spec 两轴）

两轴各跑一个子代理（同一份 diff，固定点 `3f80913`）。**每条都当假设回代码核实过**再决定改不改 ——
下面「采纳」的都是先在代码/文档里看到了第一手依据的；「驳」的写明为什么驳。

### 采纳并已改（Standards 轴）

| # | 复核意见 | 核实结果 |
|---|---|---|
| 1 | `src/control/README.md` 里描述被本票推翻却**没改**的那一段（仍写着 `fsm.hpp:396` 只判 `armed`、闩锁被清、节点会自主重请求 `OFFBOARD`，还挂着"要决策，别擅自改"） | 属实：那段与同文件已改的"可信的落地上锁"隔 30 行互相矛盾，且 `:396` 那个行号早已失效（真判据在 `:787`）⟹ 整段重写为收口后的语义。**同句在 `06` 票 `:125` 也活着**，但那一处是**引用的当时结论**、紧跟的 `:132` 就是就地更正 ⟹ 保留原样（引文不该被改写） |
| 2 | `src/bench/README.md:312` 的夹具自检条数没跟着 +1 | 属实，18 → **19**（数过 `def test_`） |
| 3 | 本票头部两个状态块互相矛盾（`Status: done` vs "卡在人的决策上"） | 属实：把先前的块标注为「收口**之前**的状态，保留作历史」 |
| 4 | `test_fsm.cpp` 三条新用例各抄一遍同样的 setup（Duplicated Code） | 属实：把 ticket 08 那段 `armed_offboard()` **上移**到公共助手区，并新增 `disconnect_frame()` 表达"合成帧的逐字形状" ⟹ 三条用例改为调用它 |
| 5 | `_latch_then_synth_disconnect()` 返回裸下标、名字说的却是"闩锁 + 那一帧" | 属实：改名 `n_requests_at_latch` 并在 docstring 里写明"返回的是**下标**，给用例切 `mode_requests[...:]` 用" |
| 6 | 文档精度：那帧的字段清单三处不一（`CONTEXT.md` 漏 `manual_input` 与 `system_status=UNINIT`） | 属实：全部对齐成**六个字段**，并把 `mavros 2.15.1 @ 22ae5b7` 补进 `CONTEXT.md` 与 §4.4（另两处本来就有） |

### 采纳并已改（Spec 轴）

| # | 复核意见 | 核实结果 |
|---|---|---|
| 7 | seam 1 第 1 条用例的名字/注释把"不许自主重新接管"当成本票的判别力所在，但那条断言在**三种实现下都过**（全撤里节点同样请求不了 OFFBOARD） | 属实 —— 判别力在 `MANUAL/LAND` 那句（全撤 ⟹ `INIT`）。用例名与注释改成"钉住一条**性质**"，并明写"这条不是本票的判别力所在" |
| 8 | "链路恢复 ⟹ 节点就会重新准备接管"这句**已被本实现记录判为不会发生**，却仍留在 `CONTEXT.md` / §4.4 / `README.md` / `fsm.hpp` 注释四处 | 属实（第一手：`px4_interface.cpp:160-164` + `guard_holds(kArmingByUs)`）⟹ 四处都改成"风险面比票面小；当前 `kArmingByUs` **恰好也**挡着它，但那是**偶然**的安全网，只有陈旧 `arm_request_pending_` 能透过去" |
| 9 | §4.4 说 state 分支"两半都证"，但 mavros 自己 `conn_timeout` 发帧那一半**没有测试**证 | 认了：稿面保留但标明"这一半只有源码第一手，夹具只造那一帧" |

### 驳（写明理由）

- **`fake_mavros.py::_tick` 里那条 `key == "state" and self._pending_disconnect_frame` 专用分支
  算 "Divergent Change"（判断题）** —— 驳。它**必须**在那个位置：合成帧走的是 mavros 的
  `conn_timeout` wall timer，与"收到 HEARTBEAT 才发 state"**不是同一条路径** ⟹ 只有**先于静默检查**
  才发得出去（真机上链路断着，这一帧照样会来）。分支上方那段 ★★ 注释写的就是这条；抽到别处反而
  会把这个"必须抢先"的顺序藏起来。
- **`synth_disconnect()` 顺手 `mute("state")` 算"两个副作用绑成对"** —— 驳（保留）。"发一帧"与
  "此后静默"在真机上是**同一条因果链**（那帧就是链路超时的产物），拆开反而要求每个调用方自己记得
  再静默一次 ⟹ 拆开更危险。`reconnect_state()` 是同名的配对出口，调用方漏配会在 `poll` 超时上暴露。

★ 复核里**没有**发现"实现与裁定不符"或 scope creep：`git diff` 只有 `fsm.hpp` 两个 hunk，
`kTakeover` 路径与 ⑤/⑥ 顺序都没动。唯一副作用（① 的复位分支是 `kTakeover`/`kArmedDegrade`
**共用**的，夺权闩锁的落地上锁复位也一并收紧）已在 §4.4 复位表注明。

### ⚠️ 收口时的三轮整仓跑：两轮各挂 1 条**既有偶发**（不是本票的），第三轮全绿

`BENCH_ROS_DOMAIN_ID=76 colcon test --packages-up-to control --executor sequential` 跑了三轮：
**第一、二轮**各只挂了
[`test_动捕时间戳冻住会降级_而短促倒退不会`](06-control-failsafe-paths.md)（ticket 06 的用例，
症状：DUT 停在 `AUTO/TAKEOFF`、`mocap_stamp_stalled` 没触发），而**本票的所有用例都过**。
核实过程与结论（第一手，全部记在 ticket 06 的「第四次、五次同族偶发」里）：

- 同一份文件**单独跑两轮 19/19 绿**（171 s ×2）、该单例单独跑 **5 次全过**；
- `build/control/Testing/Temporary/LastTestsFailed_*.log` 里，这张用例今天在
  **03:38 / 03:55 / 03:57 / 04:00 / 06:54 / 07:34** 就挂过（两种症状都有），**早于** ticket 16/18
  对该文件的任何改动（当时 13 例）⟹ 既有偶发，与本次改动无关；
- 本票改动**不碰** `first_degrade()`，且失败形状是"根本没降级"（不是"降级被错误复位"）⟹ 形状上也对不上。

★ **第三轮全绿**（`bench` 19+6、`control` 19+10+8+62+13 = 112，**0 失败**；lint 失败只剩 vendored
`vrpn_listener` 的既有基线）⟹ 本票的收口数字以这一轮为准。

★ 连带的构建事故也记在 ticket 06：**被中断的 `colcon build` 把 `build/bench` 留成半转换状态**
（该是符号链接的地方成了真目录、`install/bench/lib` 整个没了）⟹ build 一直失败在
`ament_cmake_python_symlink_bench`；`rm -rf build/bench install/bench` 重建即恢复（两者都在 `.gitignore` 里）。
