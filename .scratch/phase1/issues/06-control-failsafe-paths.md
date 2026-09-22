# 06: `control` 失效路径 —— RC 打杆接管 / 消息超时 / 低电压 / EKF2 失效

Status: done
Type: task

> ✅ **2026-09-22 用户裁定：收 `done`**（原为 `ready-for-human`）。代码与台架夹具已完成并全绿
> （96 例，见 Comments 的「复核时的数字」）。
> ★ **两条没勾的验收项仍然在，并且各自有归属**（`done` 不等于"没人管了"）：
> ① 的后半句「实测确认 PX4 按 `COM_OBL_RC_ACT` 接管」—— 需**人 + 空闲飞控 + 拆桨台架**，
> 步骤与判别法在 [`docs/control/bench-runbook.md`](../../../docs/control/bench-runbook.md) 的 **T6**；
> ④ 整条「EKF2 位置失效 ⟹ `LAND`」—— 原判据（NaN）**本链条上恒假**（第一手：EKF2 不发 NaN、
> MAVLink 丢 `xy_valid`、mavros 原样转）⟹ 当初**移交 [ticket 16](16-ekf2-position-failure-carrier.md) 跟踪**；
> ★★ **ticket 16 已于 2026-09-22 结清**：载体换成 `/mavros/estimator_status` 的 `pos_horiz_abs`
> （`degrade=ekf2_pos_lost`，**只在已解锁时算失效**、连续 2 帧），台架**可注入可测** ⟹ ④ 现在有兑现了。

**What to build:** 让**每一条单链路静默失效**都有确定的反应，而不是继续发指令。这一张是安全机制的主体 —— 那些看起来多余的判据，**每一条都是用事故换来的**。

**三件必须做对的事：**

1. **RC 打杆必须真的「停发 setpoint」** —— 只是"不更新目标"**不够**：那等于让飞机悬停在原目标，操作员打杆也拉不回来。停发后由 PX4 按 `COM_OBL_RC_ACT` 接管（z0 现值 `0` = 回 Position 模式）。z0 已**刻意配强**过 `COM_RC_OVERRIDE = 3` 与 `COM_RC_STICK_OV = 20`（打杆 20% 即接管，比默认灵敏），这是该方案的真实参数基础。
   ☠️ **更正（2026-09-22）**：`COM_RC_STICK_OV` 的语义是**归一化摇杆的每秒变化率**（`0.2/s`），
   **不是"打杆 20%"** —— 第一手链条与我们的等价实现见 `params-changes.md` **§H.11** 与 Comments。
2. **每条关键输入都有自己的超时判据** —— 动捕 / 飞控状态 / 遥控器，仿 `px4ctrl` 的 0.5 s 口径。任何一条超时 → 进 `LAND`。
   ★ **实现时按"是否已解锁"分岔**（用户 2026-09-22 决策 ①）：未解锁 ⟹ 停发 + 回 `INIT`（不请求 `LAND`），
   已解锁 ⟹ 停发 + 请求 `AUTO.LAND`。理由：`AUTO.LAND` 在 `mode_req_prevent_arming` 名单里 ⟹ 未解锁时请求它会挡住解锁。
3. **不照搬 XI35 的三处死代码。** 第一手确认：它的「紧急悬停」赋值语句被注释掉了（功能**完全不存在**）；`kf_fusion_fail` 是**单向 latch**（一旦置位只能手动着陆）；`search_hover` **没有任何地方清它**（只能重启退出）。z0 要的是**可判定、可复位**的失效降级路径，不是一个单向 latch。

★ **2026-09-21 补充（来自 [ticket 02](02-rc-channel-decision.md) 的定案，第一手核实，会改变本张的边界）：**

1. **急停不再是一条通道** —— 定案是 **ch8 高值 = PX4 硬件杀车**（停桨），程序**不读**急停通道。
   ⟹ 本张的"停发 setpoint"路径**只有打杆接管与模式开关两条**（`COM_RC_OVERRIDE=3` / ch6），
   别去实现一个"读急停通道"的分支（详情 §H.7）。
2. ⚠️ **遥控链路丢失时，PX4 可能什么都不做** —— `failsafe.cpp:340-357`：**如果解锁那一刻 RC 就已经是丢的**，
   固件把 manual control 当"可选"，**RC 丢失保护被忽略**，直到 RC 重新出现。
   ⟹ 在那种状态下，**本节点的"遥控器输入超时"判据是唯一防线**（不能指望 PX4 兜）。
   连带：**首飞前遥控必须开机且链路已建立**（§H.7.1）。
3. ⚠️ **RC 丢失保护的动作参数是 `NAV_RCL_ACT = 2`（Return）** —— 室内无 GPS 会去**返航**。
   这是 `params-changes.md` **A1** 的阻塞项（建议改 `1 = Hold`），**A1 不落实，本张的 RC 超时路径
   后果就不是"就地悬停"而是"试图返航"**。本张不改参数，但要在验收时点名这条依赖。
4. 「打杆接管 → PX4 按 `COM_OBL_RC_ACT` 接管」这条**仍然成立**，其触发点是
   **offboard 信号丢失**（`failsafe/failsafe.cpp:483-487`），不是 RC 丢失 —— 两者是**两条不同的**保护。

**Blocked by:** 05

- [x] 打杆 → **立刻停发 setpoint**（不是不更新目标），退到 `RC_OVERRIDE`
      —— 台架已验：末条 setpoint 在打杆后 **≤0.3 s**，此后**不再请求任何模式**（`RC_OVERRIDE` 是 ticket 06 新增的任务态）
- [ ] 实测确认 PX4 按 `COM_OBL_RC_ACT` 接管 —— **需拆桨台架**（夹具不模拟 PX4；本节点这一半已覆盖）
- [x] 动捕 / `/mavros/state` / 遥控器三条输入**各有一条超时判据**，任一条超时 → `LAND`
      —— 各有一条独立判据已验；★ **动作按"是否已解锁"分岔**（未解锁 ⟹ 停发 + 回 `INIT`，不请求 `LAND`），
      见 Comments 的「与原文不同的四处」。遥控器那条**只在已解锁时算失效**
- [x] 电池电压低于阈值 → 主动中止任务 → `LAND`（⚠️ 电流读数可能不可信，见 [`01`](01-hardware-prereqs.md) 的 ADC 判定；低电压保护这条兜底不依赖电流）
      —— 判据只看电压已验；**真实读数可不可信**仍归 ticket 01。★ 这条归**本张**（早期文档误记为 ticket 09，已就地更正）
- [x] EKF2 位置失效 → `LAND` —— ★★ **由 [ticket 16](16-ekf2-position-failure-carrier.md) 于 2026-09-22 兑现**：
      判据改成 `/mavros/estimator_status.pos_horiz_abs` 连续 2 帧为假（`degrade=ekf2_pos_lost`，仅已解锁时算），
      台架可注入（`set_pos_horiz_abs`）、seam 1/2 各有单测。**原先的 NaN 判据**（`position_invalid`）
      在本链条上恒假（第一手：EKF2 不发 NaN、MAVLink 丢 `xy_valid`、mavros 原样转）⟹ 降级为**防御性**保留；
      **真兜底仍是动捕那两条**（`mocap_timeout` / `mocap_stamp_stalled`）—— 见 §4.4 与 ticket 16 的 Comments
- [x] 失效降级路径**可复位**（不是单向 latch），且降级原因在日志里**可检索**
      —— `degrade=<token>` 可 grep（token 有 `-Wswitch` 兜底）；复位 = 未解锁失效**自动**回 `STANDBY`、已解锁失效**落地上锁**才复位
- [x] 故障注入全部在夹具里验证过：指定时刻让 `/mavros/state` 静默、注入时间戳倒退、注入电压跌落 —— **不留给天上第一次发生**
      ★ 三处按 ticket 原文验过，**并多验一条**：**单次**时间戳倒退**不**降级（196 Hz 里丢一帧是常态），
      "戳不前进"才降级 —— 两条用同一个用例的正控钉住
- [x] 节点在任何失效下**不崩**（崩溃会停发心跳，进而触发 PX4 失效保护）
      —— 五条输入同时静默 + 撤走 `set_mode` 服务（≈mavros 重启）时仍活着并按判据降级；通道数不足的遥控帧不崩

---

## Comments（2026-09-22，ticket 06 实现与复核）

### 与原文不同的四处 —— 都是刻意的，理由都是第一手的

1. **每条失效边按「是否已解锁」分岔**（用户决策 ①）。
   **为什么**：`AUTO.LAND` 在 `mode_req_prevent_arming` 名单里（`mode_requirements.cpp:142`，同处 `:141`
   还要求 `mode_req_local_position`）⟹ 未解锁时请求它既无意义（没有任务可中止）**又会挡住解锁**。
   ⟹ 未解锁的失效 = **停发 + 回 `INIT`**（判据恢复就自动回 `STANDBY`）；已解锁的失效 = 停发 + 请求 `AUTO.LAND`。
   反过来 OFFBOARD 的要求里**没有** `prevent_arming`（`:121-124`），所以在 OFFBOARD 里解锁是合法的（H.6）。
2. **`RC_OVERRIDE` 成为对外可见的任务态**（用户决策 ④）：夺权时 `/control/state` = `MANUAL/RC_OVERRIDE`。
   ⟹ 见 `CONTEXT.md` 的「任务态」条与 `docs/control/phase1-implementation.md` §4.1/§4.4。
3. **夺权事实自己判**（用户决策 ③）：打杆看 `/mavros/rc/in` 的**变化率**（照抄 PX4 算法），
   模式被切走看**"我们持有的模式"的下降沿**。**不依赖任何"PX4 是否已接管"的回报** ——
   那是 PX4 内部状态，MVLink 上没有一条可读的对应物。
4. **本张没有解锁路径**（解锁 = ticket 08），于是"已解锁时的失效请求 `AUTO.LAND`"在台架上没法验
   ⟹ 加临时参数 **`own_arming`**（默认 `false`，把任何 `armed` 上升沿当作本节点发起的解锁）。
   ⚠️ **只用于台架注入；实飞前保持 `false`，ticket 08 落地时删掉它**（用户决策 ②）。

### ★★ 第二轮复核（同日，`/code-review` 的 Standards + Spec 两轴）改掉的四处

1. **`mode_lost` 收窄成"丢的是哪个模式"**。原判据是"模式从 OFFBOARD 掉出去（下降沿）"，但那把
   **我们自己停发之后 PX4 的反应**也算了进去：本节点的判据是 0.5 s，PX4 的 `COM_OF_LOSS_T` 是 1.0 s
   ⟹ **真机上只要输入坏掉，必然先看见"PX4 自己退出 OFFBOARD"这一帧**（确定性，不是偶发）。
   误判代价：日志把排查方向指错；更实际的是 `is_takeover()` 会把**已闩锁的输入失效升级成夺权**
   ⟹ 从此**不再重发 `AUTO.LAND`**（把"降落失败就重试"的能力一起掐掉）。
   现判据 = "上一帧模式是我们的**且上一帧我们在流 setpoint**（或已在我们请求的 `AUTO.LAND` 里），
   这一帧不是"。刻意接受的代价：有人手动切走模式而当时飞机并未被我们驱动 ⟹ 交给**打杆**那条判据兜
   （`COM_RC_OVERRIDE` 的口径也一样：人一动杆就是接管）。
   ★ 证据在 `test_fsm.cpp` 的三条新用例里（`OFFBOARD的下降沿只在我们在流时才算夺权`、
   `停发之后PX4自己退出OFFBOARD不算夺权_继续请求降落`、`未解锁时…判据恢复即自动复位`）——
   **这条只能在 seam 2 验**：假 mavros 不假扮 PX4 的 `COM_OF_LOSS_T` 退出，台架上造不出这个形状（见下）。
   设计写在 `fsm.hpp` 的 `PrevFrame` 与 `first_degrade()` 上方，以及
   `docs/control/phase1-implementation.md` §4.4(1)。
2. **动捕戳的基准从"上一帧的戳"改成"历史最大戳"**。第一手依据：EKF2 收外部视觉样本的判据就是
   "必须比**已缓冲的最新样本**还新"（`EKF/estimator_interface.cpp:393`；不满足时 `:401-403` 只打一条
   `ECL_WARN("EV data too fast")` 并**丢弃该帧**）。"上一帧"基准对**倒退到旧纪元之后继续正常前进**
   （夹具 `VrpnSource.rewind` 的形状）**完全盲**：每一帧都比上一帧新 ⟹ 我们说健康，PX4 那边每帧都在丢。
   ★ 判别力是**实测**的，不是推演：探针量到倒退窗口里的真实戳（一跳 **−4.9955 s**，之后**每帧 +5 ms**），
   再把 `on_mocap` 临时改回"基准 = 上一帧、每帧无条件更新"（**必须有独立成员每帧无条件更新** ——
   第一次尝试只改了基准的来源、没改更新策略，`mocap_stamp_max_` 只在前进时更新 ⟹ 那份"旧判据"其实
   与新判据等价，实验无效），结果：`test_动捕时间戳倒退到旧纪元_持续超过判据即降级` **变红**
   （DUT 全程停在 `AUTO/STANDBY`，一条降级都没有），而"戳冻住"那条仍绿（相等 ⟹ 两种基准都成立）。
3. **一处代码引用更正**：`Commander.cpp` 里 `:2740` 是 `if (_arm_state_machine.isArmed())` **门**，
   `:2743-2745` 才是打杆条件（`sticks_moving && !flag_control_manual_enabled && ROTARY_WING`）。
   早先引 `:2729-2731` 是错的，`fsm.hpp` / `rc_stick.hpp` 的注释已就地更正（固件 `4a0e65f2`）。
4. **补齐正控**：所有"此后不再请求任何模式"的断言，现在都先证明"此前**确实**请求过 OFFBOARD"
   （`_assert_requests_recorded`）—— 否则 DUT 启动就崩会让负向断言**假绿**。
   ☠️ **这个正控的初版自己就是个 bug**：写成了 `assert sc.wait_for(...)`，而 **`Scenario.wait_for()`
   成功时返回 `None`、超时抛 `BenchTimeout`** ⟹ `assert None` **必然失败**，7 条用例一跑就红。
   改用布尔版 `sc.poll()` 后全绿。教训记在助手 docstring 里：夹具的等待 API 有"抛异常版"与"布尔版"两种，别混用。

### ★ 第三轮（同日的第二次两轴复核，针对**提交后的成品**）：一条实质发现 + 五处对账

**实质发现：`/mavros/state` 超时这条在真机上的下场与台架不同 ⟹ 已开
[ticket 18](18-synthetic-disconnect-frame-resets-latch.md)。**
mavros 真丢链路后会在 `conn_timeout`（默认 10 s）时**自己发一帧** `connected=false, armed=false, mode=""`
的合成状态（`sys_status.cpp:903-911`，`:1411` 触发；第一手）。于是：2.5 s 我们报超时并请求 `AUTO.LAND`，
~7.5 s 后那帧的 `armed` 降沿**把闩锁在空中复位**（`fsm.hpp:396` 只判 `armed`），节点回 `INIT`。
本节点这边不算错（链路断了，请求也到不了 PX4），但**链路恢复时闩锁已丢** —— PX4 若仍 armed，
我们会自主重新请求 `OFFBOARD`，正是闩锁要拦的事。**要决策**，已在 §4.4(3) 记账。
★ 夹具造不出这个形状（`mute("state")` 只造静默）⟹ 本文件 state 分支证明的是"静默"那一半。

> ✅ **2026-09-22 已由 [ticket 18](18-synthetic-disconnect-frame-resets-latch.md) 收口**：用户裁定选 (a)
> ——「`connected` 纳入"落地上锁"的判据」+「`armed` 的降沿只在可信帧上认」。上面这段里有两句**当时**的
> 说法要按第一手实测更正：①「那帧的 `armed` 降沿把闩锁复位（`fsm.hpp:396` 只判 `armed`）」是**当时的
> 代码**，现在判据在 `fsm.hpp` 的 `update()` ①（判据 `guards.connected && prev_armed && !guards.armed`），
> 合成帧不再复位闩锁；②「链路恢复时我们会自主重新请求 `OFFBOARD`」**当时也不会发生** —— 那帧的
> `armed=false` 顺手清了归属标记（`px4_interface.cpp:160-164`）⟹ `kArmingByUs` 挡着它。那是**偶然的
> 安全网**，判据不能靠它。夹具现在能造这个形状了（`FakeMavros.synth_disconnect()`）⟹ 本文件 state 分支
> **两半都证**（新增的两条用例）。

**另外五处（都属于"文档/证据与代码不符"，已就地改）：**

1. **`kStateTimeout` 的证据文案与判据不符**：判据是"新鲜 **且** `connected`"（`guards()`），
   而告警只写"静默 X s" —— 真遇到"话题新鲜但 `connected=false`"（正是上面那帧）会报"静默 0.0 s"，
   把排查方向指错（与"`mode_lost` 要避免的错误"同一类）。现改为两条都写
   （`connected=true/false`），并为此加了 `Px4Interface::state_connected()` 只读访问器。
2. **复位表里多写了一条不存在的判据**：§4.4 原写"打杆场景下还要求摇杆先停止动作"，
   而 `fsm.hpp` 只判模式升沿 —— "要等松杆"是**自然结果**（摇杆还在动 ⟹ 复位后第一帧 `rc_override`
   又重新成立），不是判据的一部分。已改写口径。
3. **"短促倒退不降级"这条负向断言缺正控**：注入静默失效时它会假绿。现加一个旁路订阅，
   先断言话题上**真的出现了向后跳变**（戳差 < −1 s），再断言不降级。
4. **一处编出来的出处**：`control_node.cpp` 里写"`land_mode` 可配是 ticket 06 验收项点名的"——
   ticket 全文没有这句话（它只点名"请求 `AUTO.LAND`"这个动作）。已改成"本张自己的选择"。
5. **两处计数过期**：`src/control/README.md` 的覆盖清单写着 12 例 / 39 例（实际 13 / 42），已更正。

**本轮的 ★★ 事故（正控自己假红，两处坑）：** 新加的"短促倒退不降级"正控第一版**失败**，
报"戳差最小 0.002 s"（一帧倒退都没看见）。**写临时探针实测**后真相是：注入其实一直是好的，
是**正控的观测方式**错了 ——（1）旁路订阅刚建好时**还没匹配**（几十~上百 ms），而窗口只有 0.2 s；
（2）更隐蔽：即使匹配上了，**在收到任何一帧之前**就注入 ⟹ 收到的**第一帧就是倒退后的那一帧**
⟹ 没有"前一帧"可比，探针实测窗口里只剩 +5 ms 的小步进，只有窗口结束时的**前跳 +5.0 s** 可见。
修法：先等匹配数 +1，**再等基线帧到齐**（`_watch_mocap_stamps`），两处都写进注释。
★ 这条纪律已上升为 `bench/README.md` 的"写测试的三条纪律"第 3 条的 ★★ 补充（夹具层的通用坑，
不只本张）。探针文件已删，无残留。

**驳掉的一条复核意见（第一手核实后不改语义，只把注释写准）：** 复核说"`fsm.hpp` 的 `record`
归因不准，真正必须帧末取的是 `guards` 派生的两个字段，`streaming_` 帧首帧末同值"。
逐行读下来是**反的**：`guards` 是 `const Guards &` 入参、`update()` 全程只读 ⟹ 两个字段帧首帧末必然同值；
而 `streaming_` 恰恰**在本帧被改**（`halt()`：328、⑥：480，都在 `record` 之前）⟹ 只有它必须帧末取。
原注释的语义本来就对，只是没说清"另外两个为什么不需要帧末取"。现按此改写，并在本处记一笔
（复核意见也要当假设验，不能照抄）。

**顺带记账（复核指出的两处"做了但没记"）：**

- **夹具默认电压 16.0 → 25.0 V**（`bench/config.py`、`bench.yaml`）：16.0 V **低于** DUT 的
  `low_voltage_v`（20.0）⟹ 任何 armed 场景一上来就撞低压降级。这是**跨包共用**的默认值，
  改它的理由写在 `config.py` 的注释里，本处补记一笔（ticket 05 的 standby 用例也吃这个值）。
- **`local_pos_timeout` 是第四条超时判据**（原文 ② 只点名动捕 / state / 遥控器）：它管的是
  EKF2 位置这条输入，README 与 §4.4 都记着，但原文没点名 ⟹ 在此明说一句"这是本张**多**加的一条"。

### 第一手核实（推翻或补齐了 ticket 假设的几处）

- ☠️ **`COM_RC_STICK_OV = 20` 是「归一化摇杆的每秒变化率」`0.2/s`，不是"打杆 20%"** ——
  早期文档（本仓库 `params-changes.md` §D2、`params-analysis.md` §3.13）那句读法是错的。
  第一手链条（固件 `4a0e65f2`）：`ManualControl.cpp:163-168` → `MovingDiff.hpp:40-58` → `AlphaFilter.hpp`。
  已按 z0 实测 `RC1_MIN/MAX` 折成 `rc_stick_ov_us_per_s = 100`（µs/s），**节点与固件同源**；
  就地更正见 `params-changes.md` **§H.11**。行为差别是真的：**有意的打杆都会触发**，
  **推到头保持不动不触发**（变化率为 0）。
- ☠️ **④「EKF2 位置失效 → LAND」在本链条上抓不到**（`position_invalid` 判据是**防御性**的）。
  第一手：`EKF2.cpp::PublishLocalPosition` 直接发 `_ekf.getPosition()`，`lpos.x/y/z` **没有 NaN 合成**；
  `streams/LOCAL_POSITION_NED.hpp` 在 MAVLink 层就丢掉了 `xy_valid`；mavros 2.15.1
  `local_position.cpp::handle_local_position_ned` 原样 NED→ENU。
  ⟹ 判据留着不亏（夹具能注入 NaN、节点行为可测，`test_bench_selftest.py` 有一条自检证明 NaN 真能到话题上），
  但**真正的兜底是动捕超时**（动捕是唯一位置源）。当时把候选载体 `/mavros/estimator_status`
  （`sys_status.cpp:1296` 的 `pos_horiz_abs_status_flag`；⚠️ 当时担心的"粘滞性"是**当时唯一没核实的一环** ——
  原因见票）开成 **ticket 16**。
  ★★ **ticket 16 结清（2026-09-22）**：`pos_horiz_abs` **每次发布现算**（`ekf_helper.cpp:761`）⟹ **会自清**，
  顾虑不成立；它已成为 `Degrade::kEkf2PosLost`（`degrade=ekf2_pos_lost`，仅已解锁时算失效、连续 2 帧）。
- ★ **低压保护归本张，不是 ticket 09** —— 本张原文验收项 ③ 就是它。几处文档（`bench/README.md`、
  `control/README.md`、`phase1-implementation.md`）曾把它记成 ticket 09（那是"轨迹任务脚本"），已就地更正。
- ★ **`battery_timeout_s` 刻意不做判据**：低压是**值**判断 —— 飞控静默之后最后一帧读数若低，它**仍然低**，
  "超时就不判"方向反了。真出现"电池话题静默"时，`/mavros/state` 那条判据几乎必然一起成立（同一个 mavros）。
  参数与订阅保留记账用。**没有**加"电池话题静默 ⟹ 降级"（没人论证过它的必要性）。
- ★ **`AUTO.LAND` 是尽力而为**：它要求 EKF2 的 local position（`mode_requirements.cpp:141`），
  位置真失效时 PX4 会**拒绝** ⟹ 那时唯一兜底是 PX4 自己的失效保护。
  连带：**`NAV_RCL_ACT = 2`（Return）是 A1 阻塞项** —— A1 不落实，"RC 超时 ⟹ 请求 LAND"之后
  PX4 自己的 RC 失效保护仍会试图**返航**（室内无 GPS）。本张不改参数。

### 关键的一条"不做"：不抢回

**夺权类降级（打杆 / 模式被切走）连 `AUTO.LAND` 也不请求**。理由具体：闩锁在 LAND 上的飞机，
飞行员打了杆想接管，而我们还在每 `offboard_retry_s` 请求一次 `AUTO.LAND` —— 那是**跟正在接管飞机的人抢**，
与"抢回 OFFBOARD"同一类拉锯。所以：**夺权 ⟹ 只停发 + 交还**；输入坏了 ⟹ 才请求 `AUTO.LAND`。
唯一的方向性例外：**夺权可以升级**一个已闩锁的输入失效（飞行员永远优先）。

### 新引入的注入面（每个都按"第一次用某能力先写自检"的纪律补了夹具自检）

`set_voltage` / `set_position(nan,…)` / `set_rc_channel` + "静置时通道恒定" / `VrpnSource.hold_stamp` / `rewind`
⟹ `src/bench/test/test_bench_selftest.py` 从 9 条涨到 **13 条**（`test_state_是_1Hz` 那条"3.4 s 内 2–3 帧"
也在其中 —— 它是后面那次"假红"事故的第一现场）。

### ⚠️ 顺带发现的两个夹具/基础设施问题（都不是本张的代码问题，但会咬下一个人）

1. **同一个 ROS 域里不能有两份夹具**（曾开过 ticket 17；2026-09-22 按"不影响飞行"删票归档，
   见 [`DELETED.md`](../DELETED.md)）。
   第一手证据：整仓 `colcon test` **默认并行跑包**，`bench` 的夹具（假 mavros，`armed=False`/`POSCTL`）
   与 `control` 的 DUT 同时在域 73 ⟹ **`test_control_standby` 9 条挂了 5 条**（清一色 `MANUAL/STANDBY`、
   "没在发 setpoint"）；同一份测试单独跑域 75 ⟹ **9 条全绿**。
   症状极具误导性：DUT 自己的日志完全正常，罪魁在测试进程里。
   临时做法：手动跑用 `BENCH_ROS_DOMAIN_ID=<非 0/73>`；整仓跑加 `--executor sequential`。
   ⚠️ **`BENCH_ROS_DOMAIN_ID` 对整仓跑没用**（它对所有包生效 ⟹ 两个包拿到同一个域，撞法不变）。
2. **改 `bench/*.py` 之后必须 `colcon build`**：`install/` 里是**真拷贝**（不是软链），
   而 `source install/setup.bash` 把它放进 `PYTHONPATH` ⟹ pytest 导入的是安装副本。
   这次咬到的是：`src/` 里已修掉的 `FakeMavros.evidence()` 崩溃（`Service` 没有 `get_num_clients`）
   在安装副本里还活着 ⟹ **任何一次夹具超时都变成 `AttributeError` 而不是打印证据**，
   "查不到证据"被误当成"夹具没记"。
   ★★ **中断过的 `colcon build` 会把这套 copies 链留成半截**（2026-09-22 ticket 18 收口时踩到）：
   `--symlink-install` 下 `build/bench/ament_cmake_python/bench/bench` 本该是**符号链接**，
   被杀掉的那次构建把它留成了**真目录**（文件是 11:00 的旧拷贝）⟹ 之后每次 build 都失败在
   `failed to create symbolic link ... because existing path cannot be removed: Is a directory`，
   而 `install/bench/lib/` 整个**不存在**（`import bench` 一度只能靠 pytest 的 `rootdir` 插入侥幸成功）。
   解法就是删掉这两个**纯构建产物**（都在 `.gitignore` 里）：`rm -rf build/bench install/bench` 再 build。
   ⚠️ 症状与"夹具代码不对"毫无关系，别去查代码。
   ★ 同一类症状还有一次**未复现的偶发**（同日 03:37 的整包跑）：`test_control_standby` 的 pytest
   进程 0.74 s 就死掉，留下的是 `ModuleNotFoundError: No module named 'bench'`（`bench` 不在它的
   `PYTHONPATH` 里），ctest 因此报 `missing_result`。随后单独跑（9 例绿）与**再次整包跑**
   （96 例 0 失败）都正常。**记账在这里、不再追**：真要复现先怀疑 `install/` 的 setup 链与并发，
   而不是这条用例本身。
   ★★ **第三次同族偶发**（2026-09-22，ticket 08 收尾的整仓跑）：`test_control_failsafe.py` 13 例中
   **挂 1 例** —— `test_动捕时间戳冻住会降级_而短促倒退不会` 的第 ② 段
   （`sc.wait_for_control_state("MANUAL/LAND", 6.0)` 超时，DUT 停在 `AUTO/TAKEOFF`；
   第 ① 段"短促倒退不降级"是**通过**的 ⟹ 注入链路本身通）。**未复现，机制未查明**：
   随后单独跑该例 **8.4 s 通过**，再整包跑 `control` **13/13 绿**。
   已经排掉的两个猜想（省的重复劳动）：**(a)** 不是"事件排在过去被丢掉"——
   `Timeline.due()` 取的是 `at_s <= t_s` 的全部，晚排也是下一拍就触发；
   **(b)** 不是"等待期间不触发时序事件"—— `Scenario.wait_for` 的 docstring 明写等判据时照常 `_fire_due()`。
   ⚠️ 与上面两次**同族但不是同一个**：那两次死在**进程启动**（import/`missing_result`），
   这次死在**等一个 6 s 的判据**⟹ 嫌疑指向"机器负载把某条真实时限吃掉了"，而不是夹具基础设施。
   **记账在这里、不再追**（要追先看 `wait_for` 的 `tick_s=0.02` 与实际发帧节拍在负载下的关系）。
   ★★ **第四、五次同族偶发**（2026-09-22，[ticket 18](18-synthetic-disconnect-frame-resets-latch.md) 收口时的两轮整仓跑）：
   同一张用例、同一段（② 冻住不降级，DUT 停在 `AUTO/TAKEOFF`）**两轮各挂 1 条**；而同一份文件
   **单独跑两轮 19/19 绿**（171 s ×2）、该单例单独跑 **5 次全过**。⟹ 与 ticket 18 的改动无关
   （它只动闩锁的**复位判据**，不碰 `first_degrade()`；且失败形状是"根本没降级"）。
   ★ 这一轮**多拿到一条时间线证据**：`build/control/Testing/Temporary/` 下的 `LastTest_*.log` /
   `LastTestsFailed_*.log` 记着今天**每一次** `colcon test` 的结果 ⟹ 把这张用例的既有失败排出来：

   | 时间 | 症状 |
   |---|---|
   | 03:38 | 正控：**假 mavros 一条 `OFFBOARD` 请求都没记到**（`_reach_auto` 的辅助正控，`test_control_failsafe.py:120`） |
   | 03:55 / 03:57 / 04:00 | **① 段**正控：注入的"向后跳变"没出现在话题上（戳差最小 ≈ +0.003 s） |
   | 06:54 / 07:34 | **② 段**：冻住没降级，DUT 停在 `AUTO/TAKEOFF` |
   | 11:06 / 11:15 | **② 段**：同上（ticket 18 收口的两轮） |

   ⟹ ① ② 两种症状**都是既有偶发**，且 03:38–04:00 那三条**早于 ticket 16/18 对这张文件的任何改动**
   （当时是 13 例）⟹ 这条用例的偶发**不是**本票之后才有的。
   ⚠️ **新线索**：它**只在 `colcon test`（ctest，`cwd=build/control`）语境下复现**，
   手跑 pytest（`cwd=仓库根`）从未复现。机制仍未查明；**已排掉的第二个猜想**：不是"同域两份夹具"
   —— 这台机器上 ctest 是**串行**跑的（`LastTest_20260922-1115.log` 里 `1/5 → 5/5` 顺序执行）。
   ★ 若将来要追：先量"ctest 包装层（`run_test.py` + 输出重定向 + `cwd`）对夹具定时/发帧节拍的影响"。
3. **夹具造不出"PX4 自己退出 OFFBOARD"这个形状**（`COM_OF_LOSS_T` 是 PX4 对我们停发的**反应**，
   而假 mavros 不假扮 PX4 —— `CONTEXT.md` 的「假 mavros」条）⟹ `mode_lost` 那条收窄判据的证据
   **只可能在 seam 2**。候选后续：给假 mavros 加一个"我们停发 N 秒后自己掉出 OFFBOARD"的开关
   （与"发布者节点名"同类的夹具能力 —— 后者已在 2026-09-22 做掉，见
   `test/test_control_standby.py::test_降级证据里带发布者的节点名`）。

### 复核时的数字

`colcon build --packages-up-to control` + `BENCH_ROS_DOMAIN_ID=76 colcon test --packages-up-to control
--executor sequential`：**96 例 0 失败**（第二轮复核后的数字；第一轮是 92 例 —— 那 4 例的差额是
`test_fsm.cpp` 与 `test_control_failsafe.py` 复核时补的用例，其中 3 条钉 `mode_lost` 的收窄判据）——

| 包 | 文件 | 例数 |
|---|---|---|
| `bench` | `test_bench_selftest.py`（夹具自检，ticket 06 新注入面 4 条在内） | 13 |
| `bench` | `test_timeline.py`（纯逻辑） | 6 |
| `control` | `test_control_failsafe.py`（★ 本张主验收面；13 = 11 个函数 + 参数化 3 例） | 13 |
| `control` | `test_control_standby.py` | 9 |
| `control` | `test_fsm.cpp`（seam 2：守卫 + 降级/闩锁/复位全覆盖） | 42 |
| `control` | `test_rc_stick.cpp`（seam 2：变化率判据） | 13 |

lint 失败数**不增**（仍全是 vendored `vrpn_listener` 的既有 172 条）。

### 遗留给人（拆桨台架）

★ **步骤、判据、要记录的读数、前置参数全部整理成清单了：[`docs/control/bench-runbook.md`](../../../docs/control/bench-runbook.md)**
（做台架**照它走**，不要照 `phase1-implementation.md` §7 的表走 —— 那张是"整个 Phase 1 的顺序"，
含尚未落地的 ticket 08/09，第 3b 步今天做不了）：

- **打杆 ⟹ PX4 按 `COM_OBL_RC_ACT` 接管**（z0 现值 `0` = 回 Position 模式）——本张验收项 ① 的后半句
  ⟹ runbook 的 **T6**（★ 内含"两次跑分开两条机制"的判别法：`COM_RC_OVERRIDE = 3` 与
  `COM_OF_LOSS_T`→`COM_OBL_RC_ACT` 会指向同一结果）；
- ★ **`mode_lost` 收窄的真机现场**（§4.4(1) 预测：输入坏掉后**不应**出现 `degrade=mode_lost`）⟹ runbook 的 **T7**；
- 真 PX4 被拉进 OFFBOARD / 杀节点后自行退出 ⟹ runbook 的 **T3 / T5**（那是 ticket 05 的两条）；
- 真电池读数的可信度（ticket 01 的 ADC 判定）——★ 其实 [A4 已实测判为**未中招**](../../../docs/reference/params-changes.md)
  （读数 36…38 有变化）；要更硬就是"换一次上电再采一组"，不必单列一步；
- ⚠️ **首飞前确认 RC 在场**（H.7.1：解锁那一刻 RC 就是丢的 ⟹ PX4 的 RC 丢失保护被忽略）⟹ runbook 的 **P0**。

⚠️ **④ 不在台架清单里**：原先判据恒假（EKF2 不发 NaN），**上飞机也验不出**；ticket 16 找到载体后
**改由自动化覆盖**（假 mavros 直接注入那一位）⟹ 仍然不在台架清单里（理由换了，见 `bench-runbook.md`）。
