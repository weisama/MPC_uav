# 16: 「EKF2 位置失效」的**可用载体** —— ticket 06 ④ 在本链条上抓不到

Status: done
Type: task

**为什么单开一张：** ticket [06](06-control-failsafe-paths.md) 的验收项 ④ 是
「EKF2 位置失效 → `LAND`」。实现时按字面接了判据（`position_invalid` = `local_position` 出现 NaN/Inf），
但**第一手核实发现这条判据在本项目的链路上永远为假**（详见下），所以 ④ 实际上**没有兜住任何东西**。
这不能靠"判据存在"糊过去，也不能在本张里悄悄换设计 —— 换设计要动守卫清单与降级表，**必须单独立票**。

**第一手证据（2026-09-22，z0 固件 `Minderring/PX4-Autopilot` @ `4a0e65f2331b23b9c8352656e9ccb6edeaea5d6c`）：**

1. `src/modules/ekf2/EKF2.cpp::PublishLocalPosition` 直接发 `_ekf.getPosition()` ——
   `lpos.x/y/z` **没有任何 NaN 合成**（只有全局原点无效时 `ref_lat/lon/alt` 是 NaN，与局部位置无关）。
2. `src/modules/mavlink/streams/LOCAL_POSITION_NED.hpp` 把 x/y/z 原样发出，
   **不带 `xy_valid`**（这个 flag 在 MAVLink 这一层就丢了）。
3. mavros 2.15.1（commit `22ae5b7cc7cdb4cb9c2070a8213c72dae445a23e`，
   `/home/chen/mavros_ws/src/mavros`）`src/plugins/local_position.cpp::handle_local_position_ned`
   原样 NED→ENU 拷过去，不校验也不合成。

⟹ 结论：**"EKF2 位置失效"不会以 NaN 的形式出现在 `/mavros/local_position/pose` 上。**

**现状（不是零）：** 动捕是本项目**唯一**的位置源，所以 EKF2 位置失效**必然**先表现为
**动捕输入失效**（超时 / 时间戳不前进）—— 那两条判据（ticket 06 已实现）是**当下的真实兜底**。
本票要处理的是"有没有更早、更直接的载体"。

**候选载体（都未核实，别当结论用）：**

| 候选 | 出处 | 要核实什么 |
|---|---|---|
| `/mavros/estimator_status` 的 `pos_horiz_abs_status_flag` | `mavros/src/plugins/sys_status.cpp:1296` 一带 | ① 该 flag 在 PX4 侧由谁置/清（`estimator_status` 的 flag 语义在 1.14 里改过）；② **是否有粘滞性** —— 若失效解除后 flag 不会自己清，直接拿它做判据会变成一个单向 latch（正是 ticket 06 明确要避免的） |
| `EKF2` 的 `control_status_changes` / `cs_*` 位 | `EKF2.cpp` 发布的 `estimator_status` | 同上；另确认 z0 的 `EKF2_*` 参数下这些位会不会常置 |
| `/mavros/local_position/pose` 的 `header.frame_id` 或协方差 | mavros `local_position.cpp` | 协方差在 `LOCAL_POSITION_NED` 路径上是否被填充（很可能恒 0 ⟹ 不可用） |

- [x] 核实上表中**至少一条**载体在 z0 的固件 + mavros 版本上**真的可读、且能自清**（见下 §Comments）
- [x] 若找到可用载体 ⟹ 决定它属于哪一类降级（输入类 ⟹ 已解锁时 `LAND`）、进 `fsm.hpp` 的守卫/降级表并补 seam 2 单测
      ⟹ **载体找到了**：`/mavros/estimator_status.pos_horiz_abs` ⟹ `Degrade::kEkf2PosLost`（输入类），
      seam 2 两条 + seam 1 四条 + bench 自检两条
- [ ] ~~若**找不到**可用载体 ⟹ …~~ —— **不适用**（找到了）；但那条要"删掉或降级 `position_invalid` 的对外说法"
      的**用意**照做了，见下面「文档同步」
- [x] 无论走哪条，`src/control/README.md` 与 `CONTEXT.md` 的「失效降级」条要同步（另加 `docs/control/phase1-implementation.md` §4.4）

**Blocked by:** 无（ticket 06 已落地，本票只动判据与文档）
**与 ticket 08 的关系：** 起飞之后才有"位置失效要中止任务"的后果；本票可以独立做，但**结论要在 08 之前落地**
（否则首飞时对"位置失效怎么办"的答案会是一句没兜住的判据）。
（☆ 实际落地晚于 08 —— 08 已于 2026-09-22 完成；结论仍赶在首飞之前。）

## Comments（2026-09-22，实现与现场复核）

### 三个候选的核实结果：一个可用，两个被排除（都第一手）

| 候选 | 结论 | 依据 |
|---|---|---|
| `estimator_status.pos_horiz_abs` | ✅ **可用、且能自清** | ① **能自清**：PX4 侧那位是**每次发布现算**的（`EKF2.cpp:1644` → `get_ekf_soln_status`），
定义为 `(cs.gps \|\| cs.ev_pos) && _fault_status.value == 0`（`EKF/ekf_helper.cpp:761`）—— **不是粘滞计数器**，这正是本票最担心的那条；② **可读**：mavros 把它映射成 `pos_horiz_abs_status_flag`（`sys_status.cpp:1290-1296`，`flags & ESF::POS_HORIZ_ABS`），且该字段确实出现在 `mavros_msgs/EstimatorStatus.msg` |
| `control_status_changes` / `cs_*` 位 | ❌ **两边都读不到** | ① mavros 的 `EstimatorStatus.msg` **没有**这两个字段（只有 12 个 `*_status_flag` 布尔位）；② PX4 侧 `ESTIMATOR_STATUS.hpp` 里 `mavlink_estimator_status_t est_msg{}` **零初始化**且只逐个赋了 `time_usec`/`*_ratio`/`*_accuracy`/`flags` ⟹ 该字段**恒 0**。★ 另外它的名字本身就说明它是"**变化**掩码"而不是"状态"，拿它当状态判据从语义上就不对（这条是核实过程中才意识到的） |
| `local_position` 的协方差 / `frame_id` | ❌ **结构上不可能** | `/mavros/local_position/pose` 的类型是 `geometry_msgs/PoseStamped` —— **消息里没有协方差字段**（协方差在 `Odometry` / `PoseWithCovarianceStamped` 上）；mavros 发它时只赋 `header` 与 `pose.pose`（`local_position.cpp:196-199`）。`frame_id` 是参数化常量字符串，不随估计器健康变化 |

★ **补充一条本票没列、但同样读不到的**：MAVLink `ESTIMATOR_STATUS` 里的 `pos_horiz_ratio`（`est.pos_test_ratio`，
PX4 侧**确实填了**）**没有**被 mavros 转出来（`.msg` 里没有 ratio 字段）⟹ "用新息比做阈值"这条路要求换一条
读取路径（自写 mavros 插件 / uXRCE），**不在本票**（本票的判据是**布尔位**，阈值由 PX4 内的新息检验给）。

### 实测（z0 现场，2026-09-22）

- **频率 1.000 Hz**（`/mavros/estimator_status`）—— 与源码一致：z0 链路跑 `MAVLINK_MODE_ONBOARD`，
  该模式把 `ESTIMATOR_STATUS` 配成 **1.0 Hz**（`mavlink_main.cpp:1467`；`CONFIG`/USB 是 5.0、`NORMAL` 是 0.5）
  ⟹ **这是配置不是上限**，真要更快可以提流率，但 `estimator_timeout_s` 与 `ekf2_pos_lost_samples` 都得跟着重估。
- **反转复核（用户要求，2026-09-22 09:25~09:26 本机重做，归档如下）**：停掉动捕节点 ⟹ 该位
  **跟着翻假、恢复时自清**。探针只订阅（`/mavros/estimator_status` + `/mocap/pose_enu`，**不发任何东西**），
  原始日志落在 `/tmp/z0_reversal.log`（**`/tmp` 会丢**，所以把读数抄进本票）：

  | 时刻 t | 现场 | `pos_horiz_abs` | 同帧其它位 |
  |---|---|---|---|
  | 2.16 → 12.70 | 动捕在跑（≈200 Hz） | **True**（逐帧，含 `pos_horiz_rel`/`velocity_horiz` 皆 True） | `const_pos=False` |
  | 13.17 | 动捕**已停**（`mocap_rel_pose` 退出；帧数冻在 2414，`age_of_last=0.36`） | 仍是 True（上一帧读数） | —— |
  | 13.70 | 停机中 | **False** | `pos_horiz_rel=False`、`velocity_horiz=False`、`const_pos=True` |
  | 14.70 → 23.70 | 停机中（`age_of_last` 涨到 10.36 s） | 一直 **False** | 同上 |
  | 24.17 | 动捕**重启**（帧数 2467，`age_of_last=0.02`） | 仍 False（还没发布新帧） | —— |
  | 24.70 | 已恢复 | **True** | 全部同前 |

  ⟹ "可读 + 可自清"两条**都是实测过的**，不是只读源码推的（同类观测在 ticket 13 期间已做过一次，
  读数在 `docs/reference/mocap-setup.md` §不依赖 QGC 的硬校验；本次是**重做**，不是引用那一次）。
  ★ **顺手得到一条源码之外的收获**：动捕这一停，清位发生在**最后一帧之后 0.36~0.89 s 的窗口内**
  （13.17 尚为 True、13.70 已 False；最后一帧 ≈12.81）⟹ 与 `ev_control.cpp` 那条 **400 ms** 路径
  （`isTimedOut(_ev_sample_prev.time_us, 2 * EV_MAX_INTERVAL)`）吻合，且**排除**了 1 s 那条
  （1 s 落在 13.81 之后，而那时早已翻假）。即：**"动捕直接死掉"走的是快路径**，
  1 s 那条对应的是"帧还在到、但 EKF2 不融合"的另一种场面（见下）。
- **断开到翻假的机制延迟（第一手，用来论证"它不是兜底"）**：EV 位置停止被融合 ⟹
  `isTimedOut(aid_src.time_last_fuse, _params.no_aid_timeout_max)`（`EKF/ev_pos_control.cpp:256`），
  而 `no_aid_timeout_max` 在固件里**硬编码 1 s**（`EKF/EKF/common.h:449`，不是参数）⟹ `stopEvPosFusion()`
  把 `cs.ev_pos` 置假（`:297-303`）⟹ `pos_horiz_abs` 跟着假。
  **总延迟 ≈ 1 s（EKF2 自己）+ ≤1 帧（1 Hz 发布）+ 2 帧消抖 ≈ 3 s 量级** ⟹ 必然慢于动捕那两条（0.5 s）。

### 两个决策（用户 2026-09-22 裁定）

1. **处置 = 降级·仅已解锁算**（同 `rc_timeout` 的解锁门）。代价**接受**：飞机在空中才失效、且当时未解锁时，
   这条判据是隐形的。反向代价更贵：冷启动时"EKF2 还没锚到动捕上"是**常态**，不加门会把每次冷启动
   变成"节点不发东西了"。
2. **确认样本 = 连续 2 帧**（`ekf2_pos_lost_samples = 2`）。任何一帧为真立刻归零（**对称自清**）。
   代价不对称：早报的代价是"抖一下就停发 + 请求 `AUTO.LAND`"，晚报只是慢 ~1 s，而真兜底是动捕那两条（0.5 s）。

### 落地的东西

- `fsm.hpp`：`Health::ekf2_pos_lost`、`Degrade::kEkf2PosLost`（`to_string` → `ekf2_pos_lost`）、
  `first_degrade()` ⑥（**排在 `mocap_stamp_stalled` 之后** —— 戳冻住是**原因**、EKF2 拒用是**结果**）。
- `px4_interface.{hpp,cpp}`：新话题 + `estimator_timeout_s`（**新鲜度门，不是独立判据**）+ `ekf2_pos_lost_samples`；
  ★★ 订阅**故意用易失 `QoS(10)`**，不跟真 mavros 的 `transient_local`：那会让晚订阅者一次性回放约 10 条
  历史帧（现场实测）⟹ 陈年 `false` 会被当成"现在坏了"，而这条判据的后果是停发 + 请求 `AUTO.LAND`。
- 夹具 `set_pos_horiz_abs(ok)`：**只动那一位**（DUT 只读那一位）；`estimator_status_frames` 帧计数器让
  "连续 2 帧"在测试里是**确定性**的（对齐相位，不靠 sleep）。
- **grep 配方必须改**：`degrade=[a-z_]+` 会把 `ekf2_pos_lost` 截成 `ekf` ⟹ 三处文档一起改成
  `[a-z0-9_]+`，并在 `test_fsm.cpp::DegradeNames` 加一条**逐字符守卫**（任何 token 跑出 `[a-z0-9_]` 即变红）
  ⟹ 配方与词表不会再各自漂移。★ 这是我**自己踩出来的**：那条 seam 1 用例一开始报"日志里没有
  `degrade=ekf2_pos_lost`"，而日志里**有**。

### 判据的判别力是**实测**的（变异检查）

- **新鲜度门**：把 `health()` 里的 `fresh(topics_.estimator_status, now_s) &&` 临时删掉 ⟹
  `test_estimator_status静默时_陈旧的坏读数不作数` 立刻变红（陈旧的 `false` 被当成现状）。
- **"连续"不是"累计"**：**只有 seam 1 一条**（假-真交替三轮都不降级），且"单帧不降级"用帧计数器
  做了**确定性**断言。⚠️ 这条**只能在 seam 1 验**：计数的状态在 `px4_interface.cpp::on_estimator_status()`
  里（`pos_lost_streak_`），`fsm.hpp` 只读一个算好的 `Health::ekf2_pos_lost` ⟹ seam 2 的纯函数链上
  **结构上**测不到"累计 vs 连续"（seam 2 那两条钉的是**解锁门**与**优先级**）。
  （§4.4 起初把它写成"seam 2 与 seam 1 各有一条"，于 2026-09-22 复核时改正。）

### 文档同步

`src/control/README.md`（失效降级表 + 话题表 + 参数表 + 数据流 + seam 计数）、
`CONTEXT.md`（「失效降级」条 + 「假 mavros」条）、`docs/control/phase1-implementation.md` §4.3 超时表 /
**§4.4 新增一小节**（三个候选的表 + 两道门 + 频率是配置 / 不是上限）/ §4.2 / §7.1（例数按本次实测重数：
**131** = `control` 107 + `bench` 24；其中本票贡献 `+8`：failsafe `+4`、`test_fsm.cpp` `+2`（`git show HEAD:…` 是 57）、
bench 自检 `+2`。⚠️ 归零时**只能按已提交的那棵树数** —— 当时工作区里还压着 ticket 18 的 3 条测试，
按工作区数出来的 134 会把**别人的票**算到本票头上（已被复核逮到））、
`docs/control/bench-runbook.md`（"不在本清单里"那段改为"已由自动化覆盖，但真台架上看不到它**单独**触发"）、
`src/bench/README.md`（频率纪律 + 夹具能力清单 + 自检条数）。

★ 关于 `position_invalid` 的「对外说法」：**没删**，但三处措辞都改成"真机永不触发、只是防御性"，
且都点名**真兜底是动捕那两条**。它是夹具里唯一能注入的"位置类"失效（`set_position(nan)`），
节点行为可测，留着不亏。

### 遗留（不在本票）

- `first_degrade()` ⑥ 的**位置**（排在戳冻住之后）是本票定的**语义**选择，不是实测结论；若要改，
  seam 2 那条优先级用例会挡住。
- 「提流率换延迟」是**运维选项**（改 `mavlink` 模式 / 参数），本票**不改** —— 改了要重估上面两个数。

### 复核（2026-09-22，`/code-review` 双轴）落地的修正

复核意见**逐条回代码核实过**（不是照单全收），其中**一处是真 bug、一处是我自己的数错了**：

| 复核说 | 核实结果 | 动作 |
|---|---|---|
| `control_node.cpp` 同一枚举的两处复位文案互相矛盾（`describe_degrade` 写"判据恢复即复位"，`describe_reset` 写"落地上锁"） | **真 bug**：`kEkf2PosLost` 带解锁门 ⟹ 一旦成立必是 `Latch::kArmedDegrade` ⟹ 只有落地上锁能解（`fsm.hpp:776-782`） | 改成"复位：**落地上锁**（状态位恢复**不会**在空中把它解掉）"，并在注释里写清**为什么不能**写"判据恢复"（空中解闩锁 = 未经确认就恢复发 setpoint） |
| 延迟数字三处不一致（`CONTEXT.md` / §4.4 仍写"最坏 ~2 s"） | **真的漏改** | 两处改成 `1 Hz + 清位（400 ms / 1 s）+ 2 帧 ⟹ ~2.5–3 s` |
| `src/control/README.md` 的候选编号引错（写成"② 协方差、③ `ekf_status_report`"） | **真的引错**：票里 ② 是 `cs_*` 状态位、③ 才是协方差；`ekf_status_report` 全仓只此一处 | 改为 `②` 的 `cs_*` 状态位、`③` 的 `local_position` 协方差 |
| "连续不是累计"写了"seam 1 与 seam 2 **各一条**" | **over-claim**：计数的状态在 `px4_interface.cpp::on_estimator_status()`（边界层）⟹ seam 2 **结构上**测不到 | 两处改成**只有 seam 1 一条**，并把理由写进 §4.4（seam 2 那两条钉的是解锁门与优先级） |
| 决策 3「现场反转复核」没留下本次证据 | **确实没归档**：探针日志在 `/tmp`（易失） | 本次测量的**逐帧时间线**抄进本票 Comments（见上表），并把"400 ms 快路径"这条新证据同步进 §4.4 |
| `ekf2_pos_lost_samples` 填 0/负数的行为没写 | **确实没写**（`streak >= 0` 恒成立 ⟹ 一解锁就降落） | `px4_interface.hpp` + `control.yaml` 各加一条"必须 ≥ 1"；**不做钳位**（本仓无运行时参数校验，为单独一个参数加钳位既不一致、又会让参数写错这件事在日志里静默消失） |
| seam 1 新用例里 `hold(0.35)` 与"按帧数不按 sleep"的说法自相矛盾 | **不矛盾，但没说清**：`0.35 s` 是**保证每轮恰好一帧**（< 上游 1 s），不是在等判据 | 加注释写明这个数**不能往 1 s 以上调**（否则这轮变"连续 2 帧"，用例假红） |
| 六处近义重复的延迟解释 | **刻意保留**：这六处是**六个读者面**（操作手册 / 论证 / 头文件 / 参数文件 / 日志文案），本仓的规矩是每一处都要能**独立站住**（见 `control.yaml` 的写法） | 不改；在此记一笔，免得下一位复核再提一次 |

★ 复核还顺带逮到**我自己的数错了**：§7.1 把它写成 `122 → 134`，而 134 是**按工作区**（含 ticket 18 的
3 条测试）数的 ⟹ 与本票的提交不是同一棵树。已改为实测的 **131**（`control` 107 + `bench` 24），
并写明"数例数只能按要提交的那棵树数"。
