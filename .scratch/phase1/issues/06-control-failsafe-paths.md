# 06: `control` 失效路径 —— RC 打杆接管 / 消息超时 / 低电压 / EKF2 失效

Status: ready-for-human
Type: task

> ⚠️ **为什么是 `ready-for-human` 而不是 `done`**：代码与台架夹具**已完成并全绿**（96 例，见 Comments 的
> 「复核时的数字」），但**两条验收项判的是 PX4 侧行为、夹具不模拟 PX4**：
> ① 的后半句"实测确认 PX4 按 `COM_OBL_RC_ACT` 接管"、④ 整条（见下）。这些需要**人 + 空闲飞控 + 拆桨台架**。
> 另外 ④ 还引出一个**第一手发现**：本链条上"EKF2 位置失效"**根本不会**以 NaN 出现 ⟹ 已开 ticket 16。

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
- [ ] EKF2 位置失效 → `LAND` —— **本链条上抓不到**（第一手核实：EKF2 不发 NaN、MAVLink 丢 `xy_valid`、mavros 原样转），
      见 Comments 与 §4.4；真正的兜底是**动捕超时**，候选载体已开 **ticket 16**
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
  但**真正的兜底是动捕超时**（动捕是唯一位置源）。候选载体 `/mavros/estimator_status`
  （`sys_status.cpp:1296` 的 `pos_horiz_abs_status_flag`；⚠️ PX4 的 ESTIMATOR_STATUS flag 可能有粘滞性，
  要用它得先确认"失效解除后会不会自动清")⟹ 开 **ticket 16**。
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

1. **同一个 ROS 域里不能有两份夹具**（已开 **ticket 17**）。
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
   ★ 同一类症状还有一次**未复现的偶发**（同日 03:37 的整包跑）：`test_control_standby` 的 pytest
   进程 0.74 s 就死掉，留下的是 `ModuleNotFoundError: No module named 'bench'`（`bench` 不在它的
   `PYTHONPATH` 里），ctest 因此报 `missing_result`。随后单独跑（9 例绿）与**再次整包跑**
   （96 例 0 失败）都正常。**记账在这里、不再追**：真要复现先怀疑 `install/` 的 setup 链与并发，
   而不是这条用例本身。
3. **夹具造不出"PX4 自己退出 OFFBOARD"这个形状**（`COM_OF_LOSS_T` 是 PX4 对我们停发的**反应**，
   而假 mavros 不假扮 PX4 —— `CONTEXT.md` 的「假 mavros」条）⟹ `mode_lost` 那条收窄判据的证据
   **只可能在 seam 2**。候选后续：给假 mavros 加一个"我们停发 N 秒后自己掉出 OFFBOARD"的开关
   （与 ticket 17 同类的夹具能力）。

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

### 遗留给人（拆桨台架，按 `docs/control/phase1-implementation.md` §7 的顺序）

- **打杆 ⟹ PX4 按 `COM_OBL_RC_ACT` 接管**（z0 现值 `0` = 回 Position 模式）——本张验收项 ① 的后半句；
- 真 PX4 被拉进 OFFBOARD / 杀节点后自行退出（ticket 05 的两条）；
- 真电池读数的可信度（ticket 01 的 ADC 判定）；
- ⚠️ **首飞前确认 RC 在场**（H.7.1：解锁那一刻 RC 就是丢的 ⟹ PX4 的 RC 丢失保护被忽略）。
