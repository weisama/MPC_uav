# 09: `WAIT_START → TRACK` —— 三条轨迹实飞 + 交互脚本 + bringup

Status: ready-for-human
Type: task

> ⏳ **2026-09-23：代码与两层测试已完成并全绿 ⟹ 交回人手里**（`Status: ready-for-human`）。
> ★ **9 个验收项里勾了 6 个**（1–5、8），**剩下的 3 个一条都没消失**，仍是 `[ ]` 并在
> 勾选清单里逐条写明"卡在哪"——它们全是**人的物理步骤**：
> ① 验收项 6 卡在**两个前置不在位**（EdgeTX 的 CH9 分配 §H.4 是人工动作；本机遥控链路当前是断的
>   §H.9）⟹ 台架上没得核，只能标"待核"，**不能**当"核过了"；
> ② 验收项 7（三条轨迹自由飞）与 ③ 验收项 9（§5.3 的 60 s 定点）都要**装桨、真空域**。
> ★ 本张的 `ready-for-human` 判的是**代码与测试**，不表示这些物理关卡已经过了；
> 按 `triage-labels.md`，收 `done` 是**操作者的裁定**（同 05/08 的做法）。
> ⚠️ **一处要操作者拍板的读法分歧**写在下面 Comments 的最后一条（验收项 2 的字面读法
> 与"OFFBOARD 前置流"的设计**直接冲突**）—— 在它被拍板之前，验收项 2 的勾记录的是
> **本张实测到的那三条**，不是"一个字都没发过"。

**What to build:** **完整的自主任务闭环。**

悬停稳定后程序停在 `WAIT_START` 等操作者按遥控器开始键（**200 ms 去抖**），按下 → `TRACK` 执行所选轨迹；轨迹执行完毕 → `LAND`。启动脚本让操作者**输入数字 1/2/3 选轨迹**，不需要改代码或重新 launch；飞行中能看到当前状态与轨迹完成度，所以判断飞机行为时知道它"应该"在做什么。

同时做**一键 bringup**：一条命令起 mocap + traj + control（+ mavros）。

**三道防线在本张汇合：**
- 开始键按下之前，程序**不发送任何控制指令** —— 飞机在我不期望的时候不会进 offboard
- 任意打杆立刻交还遥控器（[`06`](06-control-failsafe-paths.md) 已实现，本张验的是它在**真实飞行**里的行为）
- offboard 心跳丢失由 PX4 失效保护接管，程序侧不试图自己兜

★ **开始信号的具体通道来自 [`02`](02-rc-channel-decision.md) 的结论**，不是本张自己拍板。

✅ **2026-09-21 已 resolved**：**ch9 的上升沿**（`start_channel = 9` ⟹ `/mavros/rc/in` 的 `channels[8]`，
0-based），判据 = 先 `< 0.25` 再 `> 0.75`（归一化，与 PX4 的 `*_TH` 同一把尺）、**去抖 200 ms**。
结论与第一手依据见 [`params-changes.md`](../../../docs/reference/params-changes.md) **§H**。

实现上**照旧参数化**（值从配置读，别把 9 散落在代码里），但要按 §H.5 实现两条边沿语义：
① **第 1 次上升沿 = 解锁 + 起飞**（D3 = 程序解锁），**第 2 次 = 开始轨迹**；
② **上电即在高位不算边沿**（必须先见低位）。**电平本身不停任务** —— 停止靠打杆接管或模式开关（§H.7）。

⚠️ **两条前置还没满足，本张的台架勾选框现在做不了**：
① 要在 EdgeTX 里**先把 CH9 分配到一个开关**（§H.4，人工）；
② 本机遥控链路当前**是断的**（`RC_RECEIVER` health=0，实测见 §H.9）。
⟹ 第 6 个勾选框留空，等 ①② 完成再回填。

**Blocked by:** 02, 07, 08

- [x] `WAIT_START→TRACK` 由遥控器开始键**上升沿**触发，200 ms 去抖
      —— 判据在 `rc_switch.hpp::StartSwitchDetector`（零 ROS 依赖，12 条 `test_rc_switch.cpp` 钉住
      三条坑：上电即高位、去抖、一次边沿被消费两次），投进状态机的是
      `control_node.cpp` 每帧的 `Fsm::note_start_edge()`。★ 边沿的**归属由任务态决定**：
      `kIdle` → 起飞、`kWaitStart` → `task_=kTrack`、**其余一律丢掉并记一条日志**
      （否则悬停中误碰一下就把任务改道了）。⟹ 150 ms 的脉冲不算（`短于去抖窗的高位脉冲不算`）、
      上电即高位不算（`上电即在高位不算边沿`），两条都有 seam 1 的邻居（`test_control_track.py` T1）。
- [x] 开始键按下之前**不发送任何控制指令**
      —— ⚠️ **按下面 Comments 最后一条的读法勾的**（字面读法与"OFFBOARD 前置流"冲突，待操作者拍板）。
      实测到的三条：① 一帧 `/mavros/cmd/arming` 都没有；② 一条 `/traj/start` 都没有；
      ③ setpoint 的**目标 == 当前位置**（没有会驱动飞机的指令）。三条都在
      `test_control_track.py::_reach_wait_start()` 里作为**按键前的断言**（`PRE` 段），
      不是"事后看一眼日志"。
- [x] 启动脚本输入 1/2/3 切换轨迹，无需改代码或重新 launch
      —— `src/bringup/scripts/fly.py`（`ros2 run bringup fly.py`）。★ 它**只发 `/traj/select`**，
      不发 `/traj/start`、不调起飞：见那个文件的头两段（"谁在驱动飞机"不能是一笔糊涂账）。
      运行中重选也被支持（状态机按 `traj_stream_gap_s` 的闸门重发 `start`），脚本会多打一句提示。
- [x] 飞行中当前状态与轨迹完成度**可实时看到**
      —— `fly.py` 显示 `/control/state`（`transient_local`，所以订阅端也用 durable）+ 完成度进度条；
      显示粒度按 **5% 的完成度**走而不是按时间（悬停 60 s 不会刷 60 行），回车可立刻看一次。
      完成度是 `/control/traj_progress` —— 对 `/traj/progress` 的**逐字转发**（T7 逐值核对）。
- [x] 一键 bringup：一条命令起 mocap + traj + control（+ mavros）
      —— `ros2 launch bringup z0.launch.py`（新包 `src/bringup/`）。mavros **默认不起**
      （三个理由在 `z0.launch.py` 头部）；⚠️ 这条命令**不保证启动顺序**，而顺序的保证在**设计**里
      （control 停在 `MANUAL/INIT` 并逐条列出缺什么；traj 没 select+start 就不发参考）——
      唯一对顺序敏感的是"**起 mocap 之前飞机必须已摆好并静止**"，它在 launch 之外，写进了 README。
      台架冒烟：`MANUAL/INIT` + `/control/traj_progress` 等话题齐、`fly.py` 能选轨迹并退出（见 Comments）。
- [ ] 开始键通道在台架上用 `/mavros/rc/in` 核对过，确认**不触发 PX4 模式切换**（若与 [`02`](02-rc-channel-decision.md) 的决定不符，回 02 改决定，不要就地改代码）
      —— ⏳ **仍未做，但阻塞只剩一条**（2026-09-23 复核）：原写"两个前置不在位"，
      **其中一条已经不存在** —— 遥控链路**是通的**（2026-09-23 实测 `/mavros/rc/in` ≈8–10 Hz；
      §H.9 那句"链路是断的"只描述 **09-21 那一刻**）。**剩下的唯一阻塞是 EdgeTX 里给 CH9 分配开关**
      （§H.4，纯人工动作）。代码侧已按裁定落地并参数化
      （`start_channel: 9` / `start_threshold: 0.75` / `release_threshold: 0.25` / `debounce_ms: 200`）。
      ★ 上电时**先做这一条再做验收项 7**：通道错了的话，第 1 次上升沿会直接去解锁起飞。
      ★★ **但做之前先读 `params-changes.md` §I.3** —— `COM_RC_IN_MODE` 曾是 `3`，
      会让**整个遥控开关层静默失效**；在那个状态下做本项，得到的会是"拨了没反应"的**假结论**。
      ★ **判据落到读数上**（`rc_switch.hpp::folded`：`(us − start_low_us) / (high − low)`，**不钳位**；
      端点 `1000/2000`）：**低位必须 < 1250 µs、高位必须 > 1750 µs**；
      这个"µs 尺子"本身**还没被验过**（§H.3「单位未定案」）⟹ **若两端读数落不到这把尺子上，
      要改的是 `start_low_us` / `start_high_us` 两个端点，不是阈值**（§H.5 第 3 条）。
      ★ **做时 `control` 节点不要起**（现在是"没起"的状态，正好）—— 否则一条误触的上升沿
      会被 `kIdle` 受理成"解锁 + 起飞"；**拆桨**。
- [ ] 三条轨迹在自由飞里都跑完：1 m 悬停 / 1 m 正方形 / 0.5 m 八字
      —— ⏳ **人的物理步骤**（装桨 + 空域）。代码路径已就绪：`fly.py` 选 1/2/3、
      开始键第 2 次上升沿进 `TRACK`、`/traj/start` 由 `control` 自己发。
- [x] `TRACK` 完成后自动进 `LAND`，任务结束上锁
      —— 完成度 `progress ≥ 1.0` ⟹ `end_reason=traj_done` ⟹ `LAND`（请求 `AUTO.LAND`）⟹ 上锁
      ⟹ `MANUAL/STANDBY`（`test_control_track.py` T5 全链）。★ 有一条**必须**有的护栏：
      参考流在 `TRACK` 开头就带 1.0 时不许当场收口（T6，`track_started_` 那一位），
      否则"上一轮留下的完成度 1.0"会让第二个任务一进 TRACK 就降落。
- [ ] **§5.3 验收全部达标**：定点水平误差 **≤15 cm**、**持续 60 s**、**全程不碰摇杆**、能触发自动降落
      —— ⏳ **人的物理步骤**（同验收项 7；`fly.py` 的轨迹 1 时长就是 60 s，对齐这一条）。
      ⚠️ 起飞前确认 `wait_start_timeout_s`（默认 300 s）**大于** 60 s（节点启动时为此发一条 WARN）。
      ★★ **但这两个 60 s 是两件事，别混**：§5.3 的 60 s 悬停归**轨迹 1 自己的时长**
      （`traj` 的 `hover_duration_s: 60.0`，在 `TRACK` 里跑）；`wait_start_timeout_s` 是
      `WAIT_START` 的兜底（它是"操作员一直不按键"时才起作用的那个数）。配小了**不会**让验收飞不够，
      只会让"操作员还没准备好、飞机就自己落了" —— 阈值是**人的准备时间**的量级。
      （ticket 08 时代这两件事是同一个数 `hover_hold_s`，ticket 09 把它们分开了。）

## Comments

### 2026-09-22 —— [`07`](07-traj-reference-trajectories.md) 落地 ⟹ 本张的三条接线要求

1. **加轨迹原点**：`setpoint = traj_origin + /traj/reference`。`traj` 发的是**相对轨迹原点**的参考
   （悬停 = `(0,0,0)`，**1 m 高度在原点里，不在轨迹里**）。⚠️ 绝不能把 `/traj/reference` 直接 remap 到
   `/mavros/setpoint_raw/local` —— 那等于把原点丢掉，飞机会先往地面附近扎。
2. **进 `TRACK` 那一刻发一次 `/traj/start`**（`std_msgs/Empty`）。参考时钟的零点必须与"开始飞"同一时刻，
   否则参考已经在跑而飞机还停在起点。**运行中重选轨迹（`/traj/select`）之后要再发一次** ——
   重选会立刻停流并按新轨迹归零。
3. **`/control/traj_progress` 直接转发 `/traj/progress`**（`Float32`）；`TRACK → LAND` 用 `progress ≥ 1.0`（§4.2）。
   自己重算弧长等于把同一个数算两遍。**`/traj/marker` 只是可视化**，本张不必订阅。

★ 本张**不需要**动 `traj` 的任何代码或参数就能满足上面三条（都在 control 侧）—— 这正是"轨迹原点只在 control 加一次"的设计目的。

### 2026-09-23 —— 落地记录（代码 + 两层测试 + bringup）

**证据**：`test_control_track.py` **7 例全过**（`7 passed in 100.30s`）。
★ 复核改动落地后又整包跑了一遍：`ctest --test-dir build/control` **7/7 全过**
（`test_fsm`/`rc_stick`/`rc_switch` 三条 gtest 各 0.15 s；四条 pytest 共 404.89 s，
其中 `test_control_track` 97.87 s）；`bench` 夹具自检 **32 例全过**（`32 passed in 48.70s`）。
⚠️ 逐文件计数：control ≈ **139**（fsm 72 + rc_stick 13 + rc_switch 12 + standby 10 + failsafe 17
+ takeoff 8 + track 7）+ bench 32 = **171**，与 `phase1-implementation.md` §7.1 一致。
`bringup` 冒烟：`colcon build --packages-select bringup` 通过、`ros2 launch bringup z0.launch.py --show-args`
参数齐、真起一次（domain 73、`use_vrpn:=false`）⟹ `control` 停在 `MANUAL/INIT` 并逐条列出缺什么、
`/control/traj_progress` 等话题齐、`fly.py` 能选轨迹并按 `q` 退出。

#### 1. 边沿的归属**按任务态判，不数次数**（`fsm.hpp::note_start_edge`）

§H.5 说的"第 1 次 / 第 2 次上升沿"是**同一把开关在同一个任务里的两次动作**，
它们之间的区别**不在次数上、在"当时飞机在干什么"上**：`kIdle` ⟹ 起飞、`kWaitStart` ⟹ 进 `TRACK`、
其余 ⟹ 丢掉（`StartEdgeReply::kIgnored`，边界层记一条日志）。

★ 用一个计数器数"第几次按键"是**错**的，而且错得很具体：任务落地后计数器不会自己归零，
于是**第二个任务的第一次按键会被当成"第 2 次"**，飞机在操作员只想起飞的时候直接开跑轨迹。
用任务态判没有这个要同步的状态 —— `kIdle` 就是 `kIdle`，不管之前飞过几轮。

#### 2. `/traj/start` 的**重发**用"闸门"而不是"每次断流就发"

进 `TRACK` 的第一帧必发一次（`traj_start_sent_ = false` 保证），
之后**只在参考流断了 `traj_stream_gap_s`（0.5 s = 连丢 25 帧）**时重发。
⚠️ 这个闸门**不许调小**：重发会让 `traj` 把参考时钟归零，而它若其实还在跑，飞机当场掉头飞回起点 ——
0.5 s 是"确实断了"，不是"好像慢了"。判"真的没救"是另一条更长的 `traj_timeout_s`（3.0 s）。

#### 3. 两条**只有台架能抓到**的日志缺陷（都在 `control_node.cpp`，已修）

① **每次任务正常收尾都报一次"参考断了"**（T5 抓到，打印的 `age` 是 0.016 s）。
   根因：`alive = in_track && …` 在**离开 `TRACK` 的那一帧**翻假 ⟹ 把"任务正常结束"说成"参考断了"。
   ★ 它的代价不是难看：验收项 7/9 的自由飞**正是靠 grep 这些 token 判读现场**的，
   一条每次成功都出现的假告警会让真告警失去意义。改法：不在 `TRACK` 时**提前返回**
   （离开 TRACK 不是"断了"，是"不需要了"），并把 `traj_stream_seen_` 一并清掉。
   负控也钉住了：T3（运行中重选）**必须**看到那句"参考断了"。

② **日志的 token 与散文共用同一个字符串空间**。同一个短语（`轨迹参考已经来了`）在
   "→ AUTO/TRACK" 的散文句里也被逐字念了一遍 ⟹ 按出现次数断言（T6 期望 2 次、实际 4 次）。
   改法：测试里只认**加粗的那一份**（`**轨迹参考已经来了**`）。★ 同理
   `degrade=traj_timeout` 在散文句里出现过 ⟹ 断言要用公告的形状 `degrade=traj_timeout（持续`。
   ★ 推论（可复用）：**按 token 断言时，"token 会不会出现在别的句子里"必须先查一遍**，
   否则测试断言的是"这句话说过几次"，不是"这件事发生过没有"。

#### 4. 现场动作要点：开始键**每次压住 ≥ 0.3 s**（台架实测，写进了 `bringup/README.md`）

200 ms 去抖 × 10 Hz 采样 = **恰好 3 帧** ⟹ 需要相邻帧间隔 ≥ 0.100 s 才成立。
T1/T6 的第二次按键曾经**什么都没发生**（连"被丢掉"都没有 ⟹ 不是被忽略，是根本没产生边沿）。
测试里把 `START_HOLD_S` 取 0.5（4–5 帧）并把这一条写进现场说明。
★ 这是**判据与采样率的量化效应**，不是夹具的 bug：夹具发 rc 用的是真 10 Hz rclpy 定时器。

#### 5. `rc_switch.hpp`（新文件）在写测试时抓到的两个自己写的 bug

① `folded()` 写成 `if (!(span > 0.0)) return 0.0;` —— 把"反着配"（`low_us > high_us`，
   一份**合法配置**）与"配成相等"当成同一回事 ⟹ 方向相反的通道永远按不动。
   改 `if (span == 0.0)`，用例 `折算方向由参数决定`。
② 去抖边界：0.19999999999999996 那一类舍入 ⟹ 同一组输入时灵时不灵。
   加 `kDebounceEpsilonS = 1e-9`（只吸收舍入，比帧间隔小 8 个数量级）。

★ 另加 `StartSwitchDetector::reset()`（链路断过之后必须**重新看见低位**才承认下一次高位是边沿）：
断链期间开关动没动过我们不知道，而"已确认低位"那一位在那之前是真的 ⟹
"链路断了、开关恰好停在上位"会在恢复的第一帧凭空产生一条边沿 —— 那正是 §H.5.2 要挡的形状，
只是触发它的是断链而不是上电。代价（刻意接受）：抖动时操作员要放下再拨上去一次。

#### 6. `WAIT_START` 的出口换了，**`hover_hold_s` 这条参数已不存在**

ticket 08 的 `hover_hold_s`（"在轨迹原点上停多久后自动降落"）**被本张取代**：
降落改由**开始键**（进 `TRACK`）或**轨迹跑完**触发。
但"操作员一直不按键"这个**自己走不出去的态**必须有个兜底 ⟹ 那个数现在叫
`wait_start_timeout_s`（默认 **300 s**，`<= 0` = 永不超时）。
★ 取 300 s 的理由它是"兜底"不是"功能"：要长到够操作员处理完现场状况，短到万一他走了飞机不会一直悬着。
⚠️ 它与验收项 9 的 60 s 有**单向约束**（取 < 60 s 会在节点启动时发一条 WARN）—— 但**理由不是**
"飞机会在验收走完之前下来"：§5.3 的 60 s 归轨迹 1 的 `hover_duration_s`（在 `TRACK` 里跑），
`WAIT_START` 停多久与它无关。真正的代价是"操作员还没准备好、飞机就自己落了"。
另新增 `end_reason`（`traj_done` / `wait_start_timeout`）进日志，取代原来那个笼统的"任务走完"。

#### 7. 夹具侧的三处变化（`bench`）

- `FakeTraj`（新）：发 `/traj/reference` + `/traj/progress`，**复刻 `traj_node.cpp` 的两个坑** ——
  没 select+start 就一帧不发；完成度到 1.0 之后**照旧 50 Hz 发**、`progress` 停在 1.0。
- `_Plant` 从 `test_control_takeoff.py` **提升**成 `bench.plant.Plant`（+ `fly` / `fly_until`）：
  现在有两个 seam 1 用例要用"假飞机"，再留在单个测试文件里就得复制一份。
- `FakeTraj` 记录完成度时**记的是 Float32 量化之后的 double**：线上是 float32、测试里是 double，
  不量化就会拿一个线上从未出现过的数去比对（T7 抓到）。
- `Scenario.wait_for_subscribers()` 现在在 **DUT 已经死了**时直接抛断言并附带 DUT 日志尾部：
  参数名/类型写错会让 `rclcpp` 在构造参数时**直接退进程**，而原来的报错长得像"话题名不对"。

#### 8. `bringup` 包的两处**推翻文档**的取舍

- `z0.launch.py` 的 `use_mavros` **默认 false**（三条理由在文件头，都不是偷懒）。
- `docs/control/phase1-implementation.md` 目录图里的 `config/mavros_z0.yaml` **没有写**：
  上游 `px4.launch` 把 `pluginlists_yaml` / `config_yaml` **写死**了、没暴露成 arg ⟹
  **注入不了**我们自己的参数文件；z0 真要改的两样（串口、GCS 桥）本来就是 launch 参数。
  ★ 一个"没人读的配置文件"比"没有它"更难查 —— 现场会以为改了它就生效。
- `fly.py` **不发** `/traj/start`、**不调**起飞服务：参考时钟的零点必须与"开始飞"同一时刻，
  而"开始键之前不发控制指令"这条防线给程序入口就等于开后门。
- 冒烟时抓到 `fly.py` 自己的一个缺陷：打印去重的**比较键里带了时间戳** ⟹ 每 0.1 s 都在"变化"，
  去重形同虚设（悬停 60 s 会刷 600 行）。已修，并把"时间戳必须排除在比较键之外"写进注释。

#### 9. ⚠️ **要操作者拍板：验收项 2 的字面读法与设计冲突**

验收项 2 逐字是"**开始键按下之前不发送任何控制指令**"。但本项目的设计里，
`control` 从 `STANDBY` 起就**必须**连续流 setpoint（50 Hz ≥ 1 s 才允许切 `OFFBOARD`，
`phase1-implementation.md` §4.3-1：断流 PX4 会自己退出 OFFBOARD）——
**"一帧都不发"与"进 OFFBOARD"在 PX4 上不能同时成立。**
本张按下面这个读法落地并勾了这一格（都是 seam 1 里**按键之前**的断言）：

1. **一帧 `/mavros/cmd/arming` 都没有**（不解锁）；
2. **一条 `/traj/start` 都没有**（参考时钟不启动）；
3. setpoint 的**目标 == 当前位置**（没有会驱动飞机离开当前点的指令）。

★ 也就是说：它读作"**没有会驱动飞机的指令**"，而不是"话题上没有任何一帧"。
✗ 若裁定为**字面读法**，那它就不是本张能改的接线问题，而是**上游设计要动**
（"OFFBOARD 前置流"与本条二选一），并且会让 `MANUAL/STANDBY` 这条链失去意义 ⟹
请在下面回一句，我把它改成对应结论并回改文档。⚠️ 另需说明：**降级路径**（低压 / 打杆 / 模式被夺走）
在 `WAIT_START` 里**会**发 `AUTO.LAND`（在"已解锁"时）—— 那是失效保护，不是任务指令，
本条不覆盖它。

#### 10. 复核（`/code-review` 两轴）—— 改了什么、驳了什么、留了什么

用 `mattpocock-skills:code-review` 跑了两轴（standards / spec，各一个子代理，固定点 = `51f743d`，
改动当时**全部未提交**）。**每条都回代码核过**再动（复核意见当假设验），结论如下。

**已改（属实）**

| # | 哪条 | 实情 | 改法 |
|---|---|---|---|
| 1 | `bringup/package.xml` 缺 `mavros_msgs` | 属实：`fly.py` 顶层 `from mavros_msgs.msg import State`，而清单只有 `rclpy`/`std_msgs` ⟹ 不 source 别的空间时脚本直接 ImportError | 补 `<exec_depend>mavros_msgs</exec_depend>`，并写明它与"**节点** mavros 在 ~/mavros_ws"**不是一回事**（消息包在 `/opt/ros/humble`，`control`/`traj` 早已 `<depend>` 它） |
| 2 | `wait_start_timeout_s` 的告警阈值与理由 | **两处都错**：代码在 `< 30 s` 才告警，而 README/本票写"必须 > 60 s…会发一条 WARN"；更糟的是**理由本身失效了** —— ticket 08 时代那个 60 s 就是 §5.3 的验收时长（`hover_hold_s`），ticket 09 起 §5.3 的 60 s 归**轨迹 1 的 `hover_duration_s`**（在 `TRACK` 里跑），与 `WAIT_START` 停多久无关 | 阈值改 `60.0`；把三段话（`control_node.cpp` 注释 + WARN 文案、`control/README.md` 参数行、本票两处）的理由改成**人的准备时间**，并明写"**不是** §5.3 的时长约束" |
| 3 | `params-changes.md` §H.5.3 写 `to_normalized()` | 属实：函数叫 `folded()` | 改名 + 补"折算后**不钳位**（越界有信息量）" |
| 4 | §H.12 写 `test_rc_switch.cpp`「13 例」 | 属实：实测 **12** | 改 12 |
| 5 | `start_low_us/high_us` = 1000/2000 µs 是**未被验过的尺子** | 属实：§H.1 明说阈值只用归一化表述、**不写死 µs**，§H.3 记着"单位未定案"（ch13/14 = 880 低于 CRSF 下限 988）。实现**必须**落到一把具体尺子上才能把 0.75 折成读数 | 在 §H.5.3 补明：这是 PX4 `RC*_MIN/MAX` 默认值当代用值，**它的证伪/确认就是验收项 6（至今留空）**；上电先看 `channels[8]` 的低/高位读数，若不在 1000–2000 附近，**改的是这两个数、不是阈值** |
| 6 | `bench/fake_traj.py` 节标题错位 | 属实：`操作员的动作` 是**空节**，紧随的 `观测（断言用）` 下面挂的却是 `select`/`clear`/`silence`/`replay_done` 四个动作 | 标题与内容对齐；顺手把 `progress()` 归进 `观测`（它本来就是给断言用的读数） |
| 7 | `fly.py` 头注释"给一个程序入口等于在墙上开一扇后门"与 `/control/takeoff` 并存 | 属实（口径糊）：那句话说的是"**本脚本**不给程序入口"，但读者可以合理地问"服务不就是那个入口吗" | 改成明写：服务**是**存在的，它是**拆桨台架**用的（台上没有遥控接收机）；现场走开始键；本脚本刻意不调它，免得现场多一个"是谁让它起飞的"排查面 |
| 8 | `Px4Config::traj_progress` 名不副实 | 属实：同一个标识符在 `Px4Topics` 里是**话题名**、在 `Px4Config` 里是**回调** | 回调改名 `on_traj_progress`（与 `Px4Interface::on_traj_progress` 同名），并把这个理由写进注释 |
| 9 | `fly.py` 只有 `/mavros/state` 是硬编码字符串 | 属实（文件自己的约定是三条话题都提成常量） | 提成 `MAVROS_STATE_TOPIC` 并注明它**不是我们的话题**（归 mavros） |

**驳掉 / 留作后续（不改，理由在此）**

1. **`StartSwitchDetector` 的 5 个位置参数**（复核判为 Data Clumps）：确实和 `RcStart` 的 5 个字段一一对应，
   而且全是 `double`（`(0.75, 0.25, 0.2, 1000, 2000)` 传错顺序编译期抓不到）。
   **不改**：这是 seam 2 的头文件、构造点**只有一处**（`px4_interface.cpp` 的成员初始化表），
   而改签名要动 12 条 `test_rc_switch.cpp`。**留作后续**：真正的修法是让构造函数收一个 `RcStart`
   （或一个 `Thresholds` 小类型），那属于"ticket 09 之后的清理"，不在本票验收项里。
2. **`traj.alive(traj_stream_gap_s)` 在四处各自重算**（`fsm.hpp` 两处 + `control_node.cpp` 两处）：
   **不改**（本票内）。风险是真的（"流还活着"没有单一出处，改一处忘一处不会红），
   但它跨状态机与边界层，动它要重新设计"谁拥有这条判据"，属于结构改动。
   ★ 值得单独开一张票。
3. **`FCU_URL = "/dev/ttyTHS1:921600"` 在 `z0.launch.py` 与 `mavros.launch.py` 各存一份**：
   **不改**。两份都是**默认值**，`z0.launch.py` 通过参数传下去 ⟹ 实际以 `z0.launch.py` 为准；
   而 `mavros.launch.py` 被单独调用时也必须有个合理的默认（它是上游 `px4.launch` 的薄封装，
   保持可独立使用是有意的）。
4. **`Health::traj_stale` 由 `Fsm::update()` 填**、**`wait_start_timeout_s <= 0` 兼作启停开关**、
   **`bringup/mavros.launch.py` 单独一份 launch / `use_vrpn` 起 vrpn_listener / `fly.py` 的 0/h/q/回车**
   （后三条被 spec 轴判为"范围蔓延"）：**都不改**。第一条作者已在代码里注明是结构体的唯一例外；
   第二条是本仓既有的参数约定（`<= 0` = 永不，同 `COM_*_T` 的写法）；
   后三条各自有注释自辩，且都是"让现场少一步手动操作"的直接收益。
5. **`/control/takeoff` 服务是否该删**（spec 轴指出 `fly.py` 的话与它的存在相左）：
   **不删** —— 拆桨台架上没有遥控接收机，"RC 在线"这个起飞前置本身就过不了，
   没有服务则台架**无法完成起飞链路**（`bench-runbook.md` T4 就靠它）。改的是话（见上表第 7 条），
   不是功能。

#### 11. ⚠️ 整仓 `colcon test` 捞到一条**测试抖动**（未改，证据在此）

`colcon test --executor sequential`（2026-09-23 03:46–03:54）的结果**不是全绿**：

| 包 | 结果 |
|---|---|
| `bench` / `mocap` / `traj` / `bringup` | ✅ |
| `vrpn_listener` | ⚠️ 4 条失败 = **已知的 172 条 vendored lint**（`copyright`/`cpplint`/`lint_cmake`/`uncrustify`），与本票无关 |
| `control` | ❌ `test_control_failsafe` 失败 **1 例** |

失败的是 `test_动捕时间戳冻住会降级_而短促倒退不会` 的**第二半**：
`ASSERT wait_for_control_state("MANUAL/LAND", 6.0)` 超时，实际停在 `'AUTO/TAKEOFF'`
（`build/control/Testing/Temporary/LastTest_20260923-0346.log:409` 起）。

**复核结论：与本票的改动无关，是整仓并发下的抖动。** 三条证据：

1. **单独跑必过**：同一个文件 `ROS_DOMAIN_ID=73 pytest src/control/test/test_control_failsafe.py`
   ⟹ `19 passed in 170.84s`；只跑那一条 ⟹ `1 passed in 8.42s`。
2. **整包 ctest 也过**：`ctest --test-dir build/control`（复核改动落地后）⟹ `7/7 全过`，
   含 `test_control_failsafe` 的 172.61 s。⟹ 差异不在 ctest 路径，而在"整仓并发"。
3. **它在这张之前就在抖**：`build/control/Testing/Temporary/LastTestsFailed_*.log` 里
   `test_control_failsafe` 在 **2026-09-22 出现过 8 次**（0338 / 0355 / 0357 / 0400 / 0654 /
   0734 / 1106 / 1115），那时 ticket 09 的代码一行都还没写。
   （同一天还有 `test_control_standby` 4 次、`test_fsm` 8 次，也是整仓跑时的抖动。）

**症状的读法**（供后续定位）：DUT 在 6 s 内**既没降级也没超时**（`AUTO/TAKEOFF` 一动不动）
⟹ 它看见的动捕戳**一直在前进** ⟹ 要么冻住那次注入没生效（timeline 事件没被触发/被推后），
要么 `mocap_stamp_max_`（历史最大戳）与冻结值的关系在负载下走了另一条分支。
★ **本票不改它**：它不属于 ticket 09 的验收项，且当前无法在无负载下复现。
✅ **复核改动落地后的那一次整仓跑（04:10–04:19）没有复现** ⟹ `control` 这次是干净的，
整仓只剩 `vrpn_listener` 的 4 条 vendored lint（`copyright`/`cpplint`/`lint_cmake`/`uncrustify`，
`ctest --rerun-failed -N` 逐条对过）⟹ 它是**间歇性**的，不是某个确定输入下的失败。
按仓库惯例（缺陷入 tracker），**建议单独开一张票**并附上上面三条证据 —— 等操作者确认后我再建，
不擅自占号（17/19/20 已删，新建要同时改 `spec.md` 的 ticket 表）。

### 2026-09-23 —— 验收项 6：阻塞**只剩一条** + 台架仪器就绪（agent）

**① 阻塞变化**：原文写"两个前置不在位"，现在**只剩一条**：

| 前置 | 09-21 状态 | 09-23 复核 |
|---|---|---|
| 遥控链路 | 断（`RC_RECEIVER` health=0，§H.9） | ✅ **通** —— 实测 `/mavros/rc/in` ≈**8–10 Hz**（探针 521 s 累计 4139 帧）；之后那一段"又断了"是**操作者自己关的遥控**（已当面确认），**不是链路缺陷** |
| EdgeTX 的 CH9 分配（§H.4） | 未做 | ⏳ **仍未做**（纯人工动作）⟹ **本格的唯一阻塞** |

**② 顺带查出一条会让本项得出假结论的硬伤**：`COM_RC_IN_MODE` 曾是 `3` ——
它会让**整个遥控开关层静默失效**（ch6 模式开关 / ch5 解锁开关 / ☠️ **ch8 杀车** / 内八手势**全部失效**），
第一手链条与复现证据见 `params-changes.md` **§I.3**。
**⟹ 本项必须在 `COM_RC_IN_MODE = 0` 下做**（现役值已是 `0`，2026-09-23 用户拍板改的）。

**③ `RC_MAP_*` 全量复核 —— 本项"不触发 PX4 模式切换"那半边，参数侧已成立**：
30 条 `RC_MAP_*` 逐条读过，**非 0 的只有 `ROLL=1 / PITCH=2 / THROTTLE=3 / YAW=4 / ARM_SW=5 / FLTMODE=6 / KILL_SW=8`**
—— **没有任何一条指向 9** ⟹ 拨 ch9 不会切模式、不会解锁、不会杀车。
**剩下要核的只是读数刻度**（见验收项 6 的判据行：低位 `<1250 µs`、高位 `>1750 µs`）。

**④ 台架仪器就绪**：`/tmp/ch9_probe.py`（在 `/tmp`，**不入库**）—— 只订阅 `/mavros/rc/in` 与
`/mavros/state`，打印第 9 路的**原始读数 → 折算值 → 判成什么电平**、**16 路整帧**
（"拨 ch9 时别的通道不该动"）、以及模式变化。**它不发任何指令**。
用法 `python3 /tmp/ch9_probe.py <秒>`；操作者把 ch9 **下/上/下各停 ≥1 s 拨三个来回**。

★ **做本项时不要起 `control`**：现在它是"没起"的状态，正好 —— 起了之后一条误触的边沿
会被 `kIdle` 受理成"解锁 + 起飞"（且**拆桨**）。
