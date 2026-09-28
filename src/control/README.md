# control —— 自主飞行控制节点

把 PX4 拉进 OFFBOARD，并以 50 Hz 持续流「位置 + yaw」setpoint；
起飞、悬停判据、轨迹跟踪、自动降落（ticket 08 + 09）已经接上，**它现在会让飞机真的离地**。

> ★ **本包覆盖到 `STANDBY → TAKEOFF → HOVER → WAIT_START → TRACK → LAND`**（ticket 05 + 06 + 08 + 09）。
> 正常路径：`STANDBY` 里等到守卫齐 ⟹ 切 OFFBOARD ⟹ **遥控器开始键的第 1 次上升沿**（§H.5；服务
> `/control/takeoff` 是同一个受理路径的另一个入口）才**程序解锁**（H.6 的 D3）⟹
> 垂直慢爬到 1 m ⟹ 判稳、记轨迹原点 ⟹ 停在 `WAIT_START` 等**第 2 次上升沿** ⟹
> `TRACK` 飞所选轨迹 ⟹ 轨迹跑完（或等超时）⟹ 自动 `AUTO.LAND`。
> 失效路径：打杆夺权、输入超时、低压、位置异常、**参考流断了** ⟹ 停发/`LAND`。
>
> ⚠️ **起飞这条路径的判据只在台架/仿真上验过**：验收阶梯的 1–4 级（无桨台架 / 系绳离地 20 cm /
> 系绳闭环悬停 30 s / 自由飞）都**需要人 + 拆桨台架 + 飞控空闲窗口**，见本文末的「⏳ 还没做的验收项」。

## ★ 运行纪律（比代码重要）

1. **启动顺序**：`mavros` → `vrpn_listener` → `mocap` → 摆好飞机、静止 → 本节点。
   顺序错了**不会报错**，只会停在 `INIT`，并从日志里逐条告诉你缺什么
   （话题、静默多久、话题图上有几个发布者、QoS 是什么）。
2. **`/mavros/state` 是 1 Hz，不是"节点卡了"**。实测 1.000 Hz（min 0.998 / max 1.001），
   根源是 mavros 只在收到 MAVLink HEARTBEAT 时才发这个话题，而 HEARTBEAT 是 PX4 的 1 Hz 定频流。
   ⟹ 本节点对它的超时判据是 **2.5 s**（不是 `px4ctrl` 的 0.5 s，那样会每秒都在报"飞控失联"）；
   代价是**解锁上升沿最多晚 1 s 被检出**（ticket 08 起飞前要把这条算进去）。
3. **退出 OFFBOARD 有两种方式**：停本节点（断流超过 `COM_OF_LOSS_T` 后 PX4 自行退出），
   或遥控器接管（ticket 02 的通道分配）。**不要用 QGC 抢模式** —— 那会让"谁在控制"变得说不清。
4. **起飞的入口有两个，走同一个受理路径**（`Fsm::request_takeoff`）：主用的是**遥控器开始键
   （ch9）的第 1 次上升沿**（§H.5，ticket 09），调试用的是服务 `/control/takeoff`（ticket 08）。
   除这两个之外**没有别的触发方式**，也没有"自动起飞" ⟹ 看到飞机解锁而两条都没发生过，
   那就是**别人解的锁**（§H.6 的告警），去查是谁，不要"重试一下"。
   ⚠️ **起飞的前置里有一条"遥控在线"**（§H.7.1），所以**台上没有遥控接收机时永远解不了锁**
   —— 这是刻意的（起飞键就在遥控器上），不是 bug。
   ⚠️ 开始键是**一档两位置开关**：上电时它若已停在**高位**，那**不算边沿**（必须先见低位，H.5.2）。
   现场动作要点（**每次压住 ≥0.3 s** —— 200 ms 去抖对上 10 Hz 采样 = 恰好 3 帧，压得太准会有一半
   概率按不动）见 `params-changes.md` §H.12。
   ★★ 同一把开关还有**第 3 种用法**（ticket 07 增补，§H.5 第 4 条）：**在飞时长按 ≥1.2 s**
   （`start_land_hold_s = 1.0` + 去抖）⟹ **立即自动降落**（`end_reason=land_pressed`）——
   这是唯一一条"不用交还控制权"的停法。★ 资格看**按下那一刻**："按完起飞还按着不放"
   **不会**让飞机刚离地就降落（判据不能写成"到点那一刻是否在飞"，那会当场掉飞机）。
   ★ 也因此，`WAIT_START` 里"第 2 次按键 = 开始轨迹"的动作落到了**松手**上。
5. **降级之后想复位，只有两条路**（都不要去"帮它一把"）：
   - **落地上锁**（`connected=true ∧ armed=false` —— **可信的**那次降沿，见下面第 2 条）—— 主路径，
     对四类原因都有效；
   - **夺权场景**：让**摇杆停止动作**，再把模式**离开、然后交回** OFFBOARD（判的是模式升沿，
     所以我们得先看到它离开；摇杆还在动的话一复位就会立刻重新夺权）。
   ⚠️ 闩锁期间节点**从不自己请求 OFFBOARD** ⟹ 模式回到我们手里的那条升沿**只可能来自人**。

## 两个正交维度（不是扁平枚举）

| 维度 | 取值 | 含义 |
|---|---|---|
| **控制权** `Authority` | `MANUAL` / `AUTO` | 谁在控制飞机 |
| **任务态** `Phase` | `INIT` / `STANDBY` / `TAKEOFF` / `HOVER` / `WAIT_START` / `TRACK` / `LAND` / `RC_OVERRIDE` | 程序在干什么 |

★ **`AUTO` 的定义是"本项目正在发会驱动飞机的指令（已解锁的 offboard）"**。
所以 `STANDBY` 虽然已经切进 OFFBOARD 模式，但飞机未解锁 ⟹ `/control/state` 仍是 `MANUAL/STANDBY`。
这条定义让"切模式"与"交出控制权"两件事不被混为一谈。

`LAND` 有**四个**来源，动作逐字相同、只有原因不同：ticket 06 的失效降级（日志 `degrade=<原因>`）、
ticket 08 的 `WAIT_START` 兜底超时（`end_reason=wait_start_timeout`）、
ticket 09 的轨迹跑完（`end_reason=traj_done`）、
ticket 07 增补的**长按开始键**（`end_reason=land_pressed` —— ★ 这一条**不是**降级：操作员要求的收口）。

## 起飞 / 悬停 / 降落（ticket 08）

```
MANUAL/STANDBY ──开始键第 1 次上升沿（或 /control/takeoff）──▶ (仍在 STANDBY：解锁请求发出去，等 armed)
      │ armed 上升沿（锁原点 + 锁机头）
      ▼
AUTO/TAKEOFF ──爬到 origin+1 m，≤0.5 m/s──▶ AUTO/HOVER ──判稳 2 s──▶ AUTO/WAIT_START
                                                                          │ 开始键第 2 次上升沿 + 松手
                                                                          ▼
                                                                     AUTO/TRACK
                                                     （setpoint = 轨迹原点 + /traj/reference）
                                      运行中 /traj/select 重选 ⟹ 保持上一个目标 + 按闸门重发 /traj/start
         ★ 在飞时长按开始键 ≥1 s（ticket 07 增补）───────────────────────┐  │ 完成度 ≥ 1.0
                                                                          ▼  ▼
                                                              MANUAL/LAND（请求 AUTO.LAND）
                                                     ★ 四条来源动作逐字相同，只有 end_reason= 不同
                                                                            │ PX4 的 COM_DISARM_LAND=2.0 自己上锁
                                                                            ▼
                                                              MANUAL/STANDBY（armed 降沿 ⟹ 全部复位）
```

### 触发与前置

**两个入口，一个受理路径**：① **遥控器开始键的第 1 次上升沿**（现场用；§H.5）；
② `ros2 service call /control/takeoff std_srvs/srv/Trigger {}`（调试用；ticket 08）。
★ 服务那条的应答是 `success`（布尔）
+ `message`，后者**以** `accepted` / `already_flying` 开头（词表在 `fsm.hpp::to_string(TakeoffReply)`，
`control_node.cpp::on_takeoff_request` 把它接在 `message` 开头）—— 用服务而不是话题，
是因为**命令要有应答**：操作员按了没反应时，第一个问题永远是"什么缺了"，
而 `message` 后半段就是那个答案（缺哪条前置、当前任务态）。

★ **受理 ≠ 起飞。** 受理之后前置不齐就**一帧都不发解锁请求**，由 `takeoff_timeout_s` 兜底作废
（**不自动重试**：静默重试会把"参数没配对"这种确定性拒绝变成一串没人看的日志）。解锁的全部前置：

| 前置 | 出处 |
|---|---|
| 四条守卫全成立（已连接 / 动捕新鲜 / `local_position` 新鲜 / 解锁是我们发起的） | ticket 05 |
| **已在 OFFBOARD** + setpoint 已在连续流 | §H.6 原文两条；断流 PX4 会自动退出 OFFBOARD（`COM_OF_LOSS_T`） |
| **遥控在线**（`/mavros/rc/in` 新鲜） | §H.7.1，见下 ★ |

★★ **"遥控在线"这条是机器判据，不是人肉清单**（`fsm.hpp::update()` ⑦）。理由具体：遥控静默时解锁，
紧接着 `rc_timeout` 降级（**只在已解锁时算失效**）就会判失效并请求降落 ⟹ 飞机**起飞后立刻降落**，
比一开始就不解锁难查得多。代价是台上没接遥控接收机时永远解不了锁 —— 刻意如此。

### 判据

| 判据 | 值 | 基准 |
|---|---|---|
| 爬升 | 目标 z 从**进入 `TAKEOFF` 那一刻的 EKF 位置**线性上升到 +`takeoff_altitude_m`，速率 ≤`climb_speed_mps` | 起飞原点（锁一次，之后每帧不跟着当前位姿跑） |
| `TAKEOFF → HOVER` | `\|z − (原点+高度)\| < takeoff_alt_tol_m` ∧ `\|v_z\| < takeoff_vel_tol_mps` **持续** `takeoff_settle_s` | ⚠️ **原点+高度**，不是当帧的 `target_z` —— 写成 `target_z` 的话判据在斜坡**发出去的那一瞬间**就满足，斜坡形同虚设 |
| 稳定 | `位置误差 < hover_pos_tol_m` ∧ `\|v\| < hover_vel_tol_mps` **持续** `hover_settle_s` | **悬停点** = 起飞原点正上方 `takeoff_altitude_m`（不是轨迹原点 —— 它此刻还不存在，拿它当基准是循环定义） |
| `WAIT_START` 的两个出口 | ① **开始键第 2 次上升沿 ⟹ 松手** ⟹ 进 `TRACK`（⚠️ 不是降落）；② 等满 `wait_start_timeout_s`（默认 300 s）⟹ 走 `LAND`（`end_reason=wait_start_timeout`） | ② 是"操作员一直不按键"这条**自己走不出去的态**的兜底。★ §5.3 的验收要求它 **> 60 s**。★ ① 的动作在**松手**上（ticket 07 增补裁定 3）|
| `TRACK` / 在飞的态 收口 | ★ **长按开始键 ≥ `start_land_hold_s`（1.0 s）** ⟹ 立即 `LAND`（`end_reason=land_pressed`）。★ 资格看**按下那一刻**：只有在飞的态（`kClimb`/`kHover`/`kWaitStart`/`kTrack`）上产生的**边沿**才算数；未解锁（`kIdle`）与已受理未解锁（`kRequested`）⟹ 忽略并记 WARN | 不按"到点那一刻的当前态"判 —— 那样"按完起飞还按着不放"会在 1 s 后把正在爬升的飞机判去降落（第一手链条见 ticket 07 增补的 Comments）|
| `TRACK` 完成 | 参考完成度 `progress ≥ 1.0` ⟹ `LAND`（`end_reason=traj_done`） |
| `TRACK` 完成 | 参考完成度 `progress ≥ 1.0` ⟹ `LAND`（`end_reason=traj_done`） | ★ **必须先见过一次"完成度 < 1.0"**（`track_started_` 那一位）—— 否则上一轮留下的 1.0 会让第二个任务一进 `TRACK` 就降落 |
| `TRACK` 的参考流 | 断流 > `traj_stream_gap_s`（0.5 s）⟹ **保持上一个目标** + 重发一次 `/traj/start`；总断流 > `traj_timeout_s`（3.0 s）⟹ `degrade=traj_timeout` | ⚠️ 重发会让 `traj` 的参考时钟归零 ⟹ 那个闸门**不许调小**（判据见 `TaskLimits` 上那两段说明） |

★★ **轨迹原点 = 判稳那一刻 EKF 报的位置**（工单的 ★ 核心设计）。此后 `WAIT_START` 与
`TRACK` 的一切都以它为 0（`TRACK` 的目标就是它加参考，**不重记**：一个任务只有一个原点）。这条让 **EKF2 的局部原点与动捕原点之间的常值偏移被自动吸收** ——
我们是"以当前点为 0"发指令，于是 Phase 1 与 Phase 2 的原点问题彻底解耦。
⚠️ 它与悬停点的差不超过 `hover_pos_tol_m`（判据保证），所以记下来的那一刻目标跳变 ≤ 10 cm。

★ **两种位姿量都要求"量本身有效"**：`position_valid`（`PoseStamped` 有帧且非 NaN）与
`velocity_valid`（`velocity_local` 有帧）。少了后者，**速度话题一帧都没到过时 `vz` 恰好是 0**
（"看起来静止"）⟹ 判稳立刻通过。这是"没数据当成好消息"的老毛病，`fsm.hpp::Odometry` 把两者**分列**
（它们来自两条独立订阅，启动时可能只到了一条）。

★ **"持续"是连续的，不是累计的**：一帧不合格就重新计时（`settle_started_at_s_` 归 -1）。
到高与判稳两级**共用**这一个时刻变量（它们互斥：同一帧只可能在一个任务态里），每次进新的一级都重置。

## 轨迹跟踪（ticket 09）

```
WAIT_START ──开始键第 2 次上升沿──▶ TRACK ──progress ≥ 1.0──▶ LAND
```

**目标 = 轨迹原点 + `/traj/reference`**（★ 加原点这一步只在 control 做一次）。

⚠️⚠️ **绝不能把 `/traj/reference` 直接 remap 到 `/mavros/setpoint_raw/local`**：`traj` 发的是
**相对轨迹原点**的参考（轨迹 1 悬停 = `(0,0,0)`，**1 m 高度在原点里、不在轨迹里**）⟹
丢掉原点 = 让飞机先往地面附近扎。这条是 ticket 07 的 Comments ①，也是 `traj/README.md` 的纪律 1。

| 事 | 谁做 | 细节 |
|---|---|---|
| 发 `/traj/start` | **本节点**，在进 `TRACK` 的第一帧 | 参考时钟的零点必须与"开始飞"同一时刻。★ 操作台（`bringup/fly.py`）**故意不发**它 —— 谁驱动飞机只能有一个答案 |
| 运行中重选后**重发** `/traj/start` | 本节点，按闸门 | 参考流断了 `traj_stream_gap_s`（0.5 s）才发一次。⚠️ 闸门不许调小：重发会让 `traj` 归零时钟，它若其实还在跑，飞机当场掉头飞回起点 |
| 断流时保持**上一个目标** | 本节点 | 不是保持"当前位姿"、也不是回轨迹原点 —— 参考一回来就接着走，飞机不抽一下 |
| 转发完成度 | 本节点 → `/control/traj_progress` | **逐字转发** `/traj/progress`（`Float32`）。自己重算弧长等于把同一个数算两遍，还会与 `traj` 的定义漂移 |
| `progress ≥ 1.0` ⟹ 收口 | 本节点 | 走上面「三个来源」里那个正常出口 |
| `/traj/marker` | **不订阅** | 那是 RViz 的可视化，与判据无关 |

★ **`traj` 的两个行为决定了上面几条**（第一手，`traj_node.cpp`）：`select_kind()` 是**静默**的
（立刻停流、时钟归零，**不发任何提示**）；而跑完之后它**照旧 50 Hz 发**、完成度**停在 1.0** ⟹
"完成度 1.0"这件事在任务之间**不自动消失**，所以收口判据必须自己带一位 `track_started_`
（先见过 < 1.0 才认 ≥ 1.0），否则第二个任务一进 `TRACK` 就降落。

★ **边沿的归属按任务态判、不数次数**（`note_start_edge`）：`kIdle` ⟹ 起飞、`kWaitStart` ⟹ **松手**进 `TRACK`、
在飞的态（`kClimb`/`kHover`/`kWaitStart`/`kTrack`）⟹ **锁存降落资格**（长按 ≥1 s 即降落）、
`kRequested`/`kDone` ⟹ 丢掉并记日志。用计数器数"第几次按键"是错的 —— 落地后计数器不会自己归零，
**第二个任务的第一次按键会被当成"第 2 次"**，飞机在操作员只想起飞的时候直接开跑轨迹。

### 降落与上锁：我们只做一半

- **请求降落是我们的**：`d.request_land` ⟹ `/mavros/set_mode` 请求 `AUTO.LAND`（与 ticket 06 的降级
  走**同一个出口**、逐字相同的动作，靠 `Decision::end_reason` 区分措辞 —— 词表就三个值
  `traj_done` / `wait_start_timeout` / `land_pressed`；那个布尔量装不下多条正常出口，ticket 09 换掉了它）。
  已在我们请求的
  `AUTO.LAND` 里就**不重发**。
- **上锁不是我们的**：PX4 的 `COM_DISARM_LAND = 2.0`（z0 实测值）在它自己的着陆检测判定落地 2 s 后上锁
  ⟹ 我们**只等 `armed` 的降沿**，那一帧 `reset_mission()` 把原点/轨迹原点/任务态全部归零。
  ⚠️ 不清的话下一轮会在**上一轮的原点**上悬停 —— 而那是飞机上一次起飞的地方。
- 请求降落之后**停发 setpoint**，控制权回到 `MANUAL`（`AUTO` 的定义是"我们正在发会驱动飞机的指令"，
  而此刻是 PX4 在自己落）⟹ `/control/state` 打出 `MANUAL/LAND`。

### setpoint 的 yaw：起飞前跟随、起飞后锁定

| 阶段 | `target_yaw` | 为什么 |
|---|---|---|
| `INIT` / `STANDBY`（含已受理未解锁） | **当前机头**（`odom.yaw`） | 停机坪上不转向 |
| `TAKEOFF` / `HOVER` / `WAIT_START` / `TRACK` / `LAND` | **起飞机头**（进入 `TAKEOFF` 那一刻锁一次） | §3.4：起飞不转向。"飞机原地不动"只在 yaw 也被固定时才成立。★ `TRACK` 也在这一行：轨迹的 yaw 前馈"先不做"（§8.6），整段锁在起飞机头 |

☠️ **这条推翻了 ticket 05 的 `yaw` 参数（已删）**：那个参数默认 0，而 0 在 ROS/ENU 口径下经 mavros
换算后是 PX4 的 **+90°（机头朝东）** ⟹ 一旦在 OFFBOARD 里解锁，PX4 会**真的去转机头**。

### 日志里怎么复核（都是固定形状，可 grep）

| token | 含义 |
|---|---|
| `takeoff_origin=(x, y, z)` | 起飞原点（进入 `TAKEOFF` 那一刻的 EKF 位置） |
| `traj_origin=(x, y, z)` | **轨迹原点** —— 稳定判据成立那一刻的位置；与动捕原点对账就看它 |
| `MAV_RESULT=<n>` | `/mavros/cmd/arming` 的应答码（`0=ACCEPTED`，见 `px4_interface.cpp`；`0` 之外一律走"被拒"路径） |
| `任务正常走完（end_reason=<词表>）` | ★ 这次 `LAND` 是**正常收口**、不是降级。词表三个：`traj_done`（轨迹跑完）/ `wait_start_timeout`（等开始键超时）/ `land_pressed`（**操作员长按开始键**，ticket 07 增补）。⚠️ 来源与降级**动作逐字相同**，只有原因不同 ⟹ 判"是不是降级"要看有没有 `degrade=` |
| `degrade=<token>` | 降级原因（ticket 06 + 09 的 `traj_timeout`） |
| `遥控器开始键：**第 1 次上升沿**` / `**第 2 次上升沿**` | 确认过的边沿及其去向（起飞 / 记下"松手进 `TRACK`"） |
| `遥控器开始键：**松手** ⟹ 进入 TRACK` | 裁定 3 落地的那一帧：**松手**才真的开跑轨迹 |
| `遥控器开始键：上升沿**被丢掉**了` / `遥控器开始键：已**锁存降落资格**` | ★ **每一次边沿都要有日志**（含当前任务态）：现场最难查的一类问题是"按了没反应"，而它只有两种可能 —— 边沿没产生（去抖不够，见 §H.12）或产生了但当时没资格。★ 飞起来之后按下**不再叫"丢掉"**：它"锁存降落资格"（按住 ≥1 s 即降落）|
| `遥控器开始键：**长按 1.0 s** ⟹ 立即自动降落` / `遥控器开始键：长按**被忽略**了` | 长按**生效**（`end_reason=land_pressed`）/ **没生效**（"这一次按下发生在未解锁的时候"—— 资格看**按下那一刻**，不看按满的那一刻）|
| `TRACK：**轨迹参考已经来了**` / `**轨迹参考断了**` | 参考流的第一帧到达 / 断了（后者只在**真的断了**时打；离开 `TRACK` **不打** —— 那是"不需要了"，不是"断了"，ticket 09 的台架实测抓到的假告警） |
| `TRACK 进行中：` | 节流心跳：参考年龄 + 转发出去的完成度 |
| `traj_origin=(x, y, z)` | 已在上面列过；它同时是 `WAIT_START` / `TRACK` 的**唯一目标基准** |

★ 服务应答也有一套固定词表（`accepted` / `already_flying`，见 `to_string(TakeoffReply)`）：它出现在
**`/control/takeoff` 应答 `message` 的开头**，不在日志里 —— 用服务而不是话题，图的就是"命令有应答"。

## 失效降级（ticket 06）

**总原则：任何一条关键输入坏掉 ⟹ 停止发 setpoint**（不是"不更新目标" —— 那等于让飞机悬在原目标上，
操作员打杆也拉不回来），然后按**是否已解锁**分岔处置。

| `degrade=` token | 判据 | 未解锁 ⟹ | 已解锁 ⟹ |
|---|---|---|---|
| `rc_override` | 摇杆变化率超 `rc_stick_ov_us_per_s`（飞行员在动手） | 停发 + 交还控制权 | 同左，**且不请求任何模式** |
| `mode_lost` | **我们正在驱动**的那个模式被外部切走（上一帧在流 setpoint 的 `OFFBOARD` / 我们请求的 `AUTO.LAND`） | 停发 + 交还控制权 | 同左 |
| `state_timeout` | `/mavros/state` 静默 > `state_timeout_s`（2.5 s）⟹ 飞控失联 | 停发 + 回 `INIT` | 停发 + `LAND`（请求 `AUTO.LAND`） |
| `mocap_timeout` | `/mocap/pose_enu` 静默 > `mocap_timeout_s` | 同上 | 同上 |
| `local_pos_timeout` | `/mavros/local_position/pose` 静默 > `local_pos_timeout_s` | 同上 | 同上 |
| `rc_timeout` | `/mavros/rc/in` 静默 > `rc_timeout_s`（**只在已解锁时算失效**） | —— | 停发 + `LAND` |
| `battery_low` | `/mavros/battery` 电压 < `low_voltage_v` | 同上 | 同上 |
| `position_invalid` | `local_position` 出现 NaN/Inf（**防御性**，见下） | 同上 | 同上 |
| `mocap_stamp_stalled` | 动捕帧照发但时间戳**没超过历史最大戳** > `mocap_stamp_stall_s`（含冻住与倒退到旧纪元两种形状） | 同上 | 同上 |
| `ekf2_pos_lost` | `/mavros/estimator_status` 报 `pos_horiz_abs` **连续** `ekf2_pos_lost_samples`（2）帧为假（**只在已解锁时算失效**） | —— | 停发 + `LAND`（请求 `AUTO.LAND`） |
| `traj_timeout` | `TRACK` 里参考流断了 > `traj_timeout_s`（3.0 s，**重发 `/traj/start` 也没救回来**） | ——（只在 `TRACK` 里可能成立） | 停发 + `LAND` |
| `none` | 没有降级 | —— | —— |

★ **`ekf2_pos_lost` 是「EKF2 位置失效」在本链条上唯一真能读到的载体**（ticket 16，2026-09-22）：
判据就是 EKF2 自报的那一位（PX4 侧 `(cs.gps || cs.ev_pos) && _fault_status.value == 0`，
`EKF/ekf_helper.cpp:761`，**每次发布现算** ⟹ 会自清，不是锁存量）。它覆盖的是**另一类**失效 ——
"动捕帧正常到、`local_position` 照发，EKF2 却拒绝用"（融合失败 / `_fault_status` 非零），
那种场面 `mocap_timeout` / `mocap_stamp_stalled` 都看不见。**但它不是兜底**：上游 1 Hz + 两道
（EKF2 自己清 `ev_pos` 的 400 ms / 1 s，再加发布相位与我们这 2 帧消抖）⟹ **~2.5–3 s 才成立**，
比动捕那两条（0.5 s）慢一个量级。现场实测（z0，1.000 Hz）与完整反转复现见
`docs/control/phase1-implementation.md` §4.4。
⚠️ **只在已解锁时算失效**（与 `rc_timeout` 同款的解锁门）：这条判据的代价是**停发**，
而冷启动时"EKF2 还没把位置锚到动捕上"是常态 ⟹ 不加门会把每次冷启动的正常窗口变成"节点不发东西了"。

★ **两类降级的处置完全不同，不要合并**：`rc_override` / `mode_lost` 是**控制权被夺走**
⟹ 停发、交还，并且**不请求任何模式**（既不抢回 OFFBOARD，**也不请求 `AUTO.LAND`** ——
那是在跟正在接管飞机的人抢）；其余是**输入坏了** ⟹ 已解锁时主动中止任务、请求 `AUTO.LAND`。

★★ **`mode_lost` 的判据要看"丢的是哪个模式"**（2026-09-22 复核补）：只有**我们在流 setpoint 的
`OFFBOARD`**、或**我们自己请求的 `AUTO.LAND`**，被切走才算夺权。理由是我们自己一停发，PX4 就会在
`COM_OF_LOSS_T`（1.0 s）后**自行**退出 `OFFBOARD` —— 而本节点的判据是 0.5 s，
**真机上必然先看到"自己退出"这个形状**，它与"人被夺权"逐帧无法区分。误判的代价有两层：
日志把排查方向指错；更实际的是夺权会**升级**已闩锁的输入失效 ⟹ 从此连 `AUTO.LAND` 都不再重发。
代价是**刻意接受**的：有人手动切走模式、而当时飞机并未被我们驱动 ⟹ 由**打杆**那条判据兜
（PX4 `COM_RC_OVERRIDE` 的口径也一样：人一动杆就是接管）。
★ 这一条**只能在 seam 2 验**：假 mavros 不假扮 PX4 的 `COM_OF_LOSS_T` 退出（`CONTEXT.md`），
台架上造不出这个形状。详见 `docs/control/phase1-implementation.md` §4.4(1)。

★★ **动捕戳的基准是历史最大戳，不是"上一帧的戳"**（同上复核）：EKF2 收外部视觉样本的尺子就是
"比**已缓冲的最新样本**还新"（`EKF/estimator_interface.cpp:393`；不满足时只打一条
`ECL_WARN("EV data too fast")` 并把这一帧丢掉，`:401-403`）。"上一帧"基准对**倒退到旧纪元之后
继续正常前进**完全盲 —— 每一帧都比上一帧新 ⟹ 判据说健康，而 PX4 那边每一帧都在丢。
判别力是**实测**的（把基准临时改回"上一帧"⟹ 那条用例立刻变红），见 §4.4(2)。

★ **可复位（不是单向 latch）**，两条路径：

1. **未解锁的失效不闩锁** ⟹ 判据一恢复就自动回 `STANDBY`（台架上的证据就在这条）。
2. **已解锁的失效闩锁到"落地上锁"** —— `armed` 的 true→false 沿，**且那次降沿必须是可信的**
   （`connected=true`）。**不需要重启节点、也不需要任何"清故障"命令**：操作员落地上锁即复位。
   这是与 XI35 的 `kf_fusion_fail`（一旦置位只能手动着陆）刻意不同的地方。
   ★★ **「可信」这一半是 ticket 18 加的**（2026-09-22）：链路静默满 `conn_timeout`（mavros 参数，
   默认 10 s）后，**mavros 自己**会往 `/mavros/state` 上发一帧 `connected=false, armed=false, …`
   （`sys_status.cpp:903-911` 的 `publish_disconnection()`）—— 那帧里的 `armed=false` 是**占位值**、
   不是读数，凭它复位 = 把闩锁**在空中**清掉（链路恢复时若 PX4 仍 armed，节点就会走上"重新准备接管"
   那条路 —— ★ 当前 `kArmingByUs` **恰好也**挡着它，但那是**偶然的安全网**，判据必须自己挡住）。
   ⟹ 判据是"`connected` **且** `armed` true→false"，而 `armed` 的降沿**只在可信帧上认**
   （两处一对，缺一不可 —— 细节与刻意接受的代价见 `docs/control/phase1-implementation.md` §4.4）。
3. 夺权闩锁额外可由"外部把模式交回 OFFBOARD"复位（闩锁期间我们从不自己请求 OFFBOARD
   ⟹ 那条升沿只可能来自人）。**已解锁的输入失效不因切模式复位**（电池不会因为有人切了模式就变满）。
4. 闩锁记的是**首因**（最接近根因），不被后续判据改写；但**夺权可以升级**一个已闩锁的降级
   —— 飞行员永远优先（否则会出现"飞行员在接管、我们还在请求 AUTO.LAND"的拉锯）。

★ **日志可检索**：每个降级写 `degrade=<token>`（token 就是上表第一列，来自 `fsm.hpp::to_string(Degrade)`），
复位的那一帧写 `degrade=none：降级已复位（此前 degrade=<原因>）` ⟹
`grep -oE "degrade=[a-z0-9_]+" 日志 | grep -v none` 能捞出全部降级事件、
`grep "已复位" 日志` 能捞出全部复位时刻。
★ 字符类是 **`[a-z0-9_]`**、不是 `[a-z_]`（2026-09-22 改）：`ekf2_pos_lost` 里带数字，
旧配方会把它截成 `ekf` ⟹ 按旧配方捞出来的 token 不是真实原因，还会把"有没有这条降级"判错。
`test_fsm.cpp::DegradeNames` 有一条逐字符守卫，**任何 token 跑出这个字符集都会让 seam 2 变红** ⟹
文档里的配方和代码里的词表不会各自漂移。

⚠️ **拿不到 `AUTO.LAND` 时怎么办（诚实版）**：`AUTO.LAND` 需要 EKF2 的 local position
（`mode_requirements.cpp:141`），位置真失效时 PX4 会**拒绝**它 —— 那时唯一兜底是 PX4 自己的失效保护
（`NAV_RCL_ACT` / `COM_OBL_RC_ACT`）。★ **z0 现值 `NAV_RCL_ACT = 1`（Hold）** ——
**2026-09-28 第一手实测**（`ros2 param get /mavros/param NAV_RCL_ACT` ⟹ `1`）
⟹ 本节点的"RC 超时"路径后果是**就地保持**。☠️ 本文此前写"现值 `2`（Return）… A1 不落实"，
**那句是错的**（同一时期 ticket 13/15 都记着"已生效"）。

⚠️ **★ 2026-09-28：还有两处"现值 2 / A1 阻塞项"的旧话在源码里没改**（文档侧已全部清扫）：

| 位置 | 是什么 | 为什么要紧 |
|---|---|---|
| [`control_node.cpp:1093`](src/control_node.cpp#L1093) | `Degrade::kRcTimeout` 的**人话说明串** | ☠️ **运行时打给现场看的**，且正出现在"遥控静默且已解锁"这个场景 —— 说的正好是反的 |
| [`fsm.hpp:1285`](include/control/fsm.hpp#L1285) | 降落分支的注释 | 只是注释，但同属"引用了参数现值"这一类会腐坏的字面 |

★ **2026-09-28 操作者裁定：留到下次动 `control` 时一并改**（改源码**必须重建才生效**：
`colcon build --packages-select control` + 重跑 `colcon test`，否则改了个寂寞）。
⟹ **在那之前，这两处仍会输出错话** —— 别当成"已修完"。

⚠️ **`state_timeout` 这条在真机上不是"请求 `AUTO.LAND` 直到结束"**（2026-09-22 复核补，第一手：
mavros 2.15.1 @ `22ae5b7`）：真丢链路时 mavros 会在 `conn_timeout`（默认 10 s）之后**自己发一帧**
`connected=false, armed=false, guided=false, manual_input=false, mode=""` 的合成状态（`sys_status.cpp:903-911`，由 `:1411`
的 `connection_cb(false)` 触发）⟹ 我们在 2.5 s 报超时并请求 `AUTO.LAND`，~7.5 s 后那一帧到达。
链路已断时继续请求本来也到不了 PX4，所以那几个请求不算错。
★ **2026-09-22（ticket 18）已把这一帧的复位语义收口**：它**不算落地上锁** ⟹ 闩锁不被它清掉，
链路恢复时若 PX4 仍 armed 就**仍然走 `LAND`**（判据 = `connected=true ∧ armed=false`，且 `armed` 的降沿
只在**可信帧**上认；详见上面第 2 条与 `docs/control/phase1-implementation.md` §4.4 的「可信的落地上锁」）。
⚠️ 那一帧之后失联期间，对外状态会掉到 `MANUAL/INIT`（`armed=false` ⟹ 不再请求 `AUTO.LAND`，
**而闩锁其实还在**）—— 这是刻意接受的代价，别把它当成"复位了"。

⚠️ **`position_invalid`（EKF2 位置失效的 NaN 判据）是防御性的，真机这条链上抓不到东西**。
第一手核实（z0 固件 `4a0e65f2`）：`EKF2.cpp::PublishLocalPosition` 发的 `lpos.x/y/z` 不含 NaN 合成；
`LOCAL_POSITION_NED.hpp` 在 MAVLink 层就丢掉了 `xy_valid`；mavros 2.15.1 原样转 ENU。
⟹ 这条判据留着不亏（夹具能注入 NaN，节点行为可测），但它在真机上**永远不会触发**
（ticket 16 已定案，2026-09-22）——「EKF2 位置失效」在本链条上的可用载体是
**上表的 `ekf2_pos_lost`**（`/mavros/estimator_status`），不是这条 NaN 判据。
★ **真正的兜底仍然是动捕那两条**（`mocap_timeout` / `mocap_stamp_stalled`，0.5 s 量级）：
`ekf2_pos_lost` 只能算**补充**（~2.5–3 s 才成立），且它只在已解锁时算失效。
选型与被排除的候选（② 的 `cs_*` 状态位、③ 的 `local_position` 协方差）见
`docs/control/phase1-implementation.md` §4.4。

⚠️ **`battery_timeout_s` 刻意不做判据**：低压是**值**判断，不是**新鲜度**判断 ——
最后一帧读数若是低的，飞控静默之后它**仍然是低的**，此时"超时就不判"是错的方向。
（真要加的话是"电池话题静默 ⟹ 降级"，那是另一个判据，没人论证过，见 ticket 06 的 Comments。）

`/control/state` 发 `"<控制权>/<任务态>"`（如 `MANUAL/STANDBY`），**transient_local**
⟹ 晚订阅者立刻拿到当前值；另外每 `diag_period_s` 心跳重发一次。
观察：`ros2 topic echo /control/state`

## 数据流

```
/mavros/state ─────────────────┐
/mavros/local_position/pose ───┤
/mavros/local_position/velocity_local ┤
/mocap/pose_enu ───────────────┤
/mavros/estimator_status ──────┼─▶ 守卫/读数 ─▶ 状态机 ─▶ /mavros/setpoint_raw/local（PositionTarget）
/mavros/rc/in、/mavros/battery ┘            ├─▶ /control/state（"<控制权>/<任务态>"）
/traj/reference、/traj/progress ───────┘            └─▶ /control/traj_progress（转发 /traj/progress）

开始键上升沿（rc_switch）┐
/control/takeoff（服务） ┴──▶ 状态机 ──▶ /mavros/cmd/arming（服务）、/mavros/set_mode（服务）
                              └──▶ /traj/start（std_msgs/Empty，进 TRACK 时 + 断流时按闸门重发）
```

| 话题 / 服务 | 方向 | 类型 | QoS | 备注 |
|---|---|---|---|---|
| `/mavros/state` | 订阅 | `mavros_msgs/State` | 可靠 + **transient_local** | 上游 1 Hz |
| `/mavros/local_position/pose` | 订阅 | `geometry_msgs/PoseStamped` | **best_effort** | 上游 ~30 Hz；位置的唯一来源 |
| `/mavros/local_position/velocity_local` | 订阅 | `geometry_msgs/TwistStamped` | **best_effort** | ★ ticket 08 新增。**判稳要它**：EKF2 融合过的速度，比我们自己差动捕干净 |
| `/mavros/rc/in` | 订阅 | `mavros_msgs/RCIn` | 可靠 | **三个**用途：**打杆夺权**（变化率）+ **超时**（仅已解锁时算失效）+ ★ **开始键**（`start_channel` 的上升沿，ticket 09）；另有一条**起飞前置**（§H.7.1） |
| `/traj/reference` | 订阅 | `mavros_msgs/PositionTarget` | 可靠 | ★ ticket 09。上游 50 Hz。⚠️ 是**相对轨迹原点**的位置 ⟹ 目标 = 轨迹原点 + 它 |
| `/traj/progress` | 订阅 | `std_msgs/Float32` | 可靠 | ★ ticket 09。上游 50 Hz、0–1。`TRACK` 的收口判据就是它 |
| `/traj/start` | **发布** | `std_msgs/Empty` | 可靠 | ★ ticket 09。进 `TRACK` 的第一帧必发一次；之后只在**参考断流**时按 `traj_stream_gap_s` 重发 |
| `/control/traj_progress` | **发布** | `std_msgs/Float32` | 可靠 | ★ ticket 09。对 `/traj/progress` 的**逐字转发**。⚠️ 与它同名不同域：那是 `traj` 的、这条是我们的 —— 别名会让"订阅谁"变成猜谜 |
| `/mavros/battery` | 订阅 | `sensor_msgs/BatteryState` | **best_effort** | 低电压判据（**只看值**，不看新鲜度） |
| `/mavros/estimator_status` | 订阅 | `mavros_msgs/EstimatorStatus` | ★★ **故意用易失 `QoS(10)`** | 上游 1 Hz。`ekf2_pos_lost` 判据的载体（ticket 16）。⚠️ 真 mavros 这条是 **transient_local** 发的，我们**刻意不跟**：那会让晚订阅者一次性回放约 10 条历史消息（现场实测）⟹ 陈年 `false` 帧被当成"现在坏了"。由 `estimator_timeout_s` 这条新鲜度门兜 |
| `/mocap/pose_enu` | 订阅 | `geometry_msgs/PoseStamped` | 可靠 | 来自 `mocap` 节点；**是守卫的输入**（新鲜度决定发不发 setpoint） |
| `/mavros/setpoint_raw/local` | **发布** | `mavros_msgs/PositionTarget` | 可靠 | 唯一的控制输出 |
| `/control/state` | **发布** | `std_msgs/String` | 可靠 + transient_local | `"MANUAL/STANDBY"` |
| `/control/takeoff` | **服务** | `std_srvs/Trigger` | —— | ★ ticket 08：起飞的**调试入口**（ticket 09 起，主用的是遥控器 ch9 的第 1 次上升沿；两者走同一个 `request_takeoff`） |
| `/mavros/cmd/arming` | **服务（客户端）** | `mavros_msgs/CommandBool` | —— | ★ ticket 08：程序解锁（H.6 的 D3）。**被拒 ⟹ 放弃本次起飞**（不重试） |
| `/mavros/set_mode` | **服务（客户端）** | `mavros_msgs/SetMode` | —— | 切 `OFFBOARD`（ticket 05）与请求 `AUTO.LAND`（ticket 06/08） |

★ **速度话题与位置话题是同一条 MAVLink 消息**：mavros 在 `local_position.cpp` 的**同一个回调**里发这两条
话题，速度经 `transform_frame_ned_enu` ⟹ 频率、时间戳、口径（都是 ROS/ENU，z 向上为正）一致。
所以两者**没有"要不要互相等"的问题**，但**有效性各算各的**（两条独立订阅，启动时可能只到了一条）——
见上文「判据」一节的 `velocity_valid`。

★★ **QoS 必须与真 mavros 逐条对齐，而且错了不会报错、只会"永远收不到"**：
`local_position` / `battery` 在真 mavros 上是 `SensorDataQoS()`（best_effort），
按默认（可靠）订阅会**静默不匹配**。依据与实测见 `include/control/px4_interface.hpp`
与 `src/bench/README.md`；`bench` 包有一条专门的测试盯着这件事。

## setpoint 的三个硬指标

| 字段 | 值 | 依据 |
|---|---|---|
| `type_mask` | **2552** | IGNORE_VX/VY/VZ + IGNORE_AFX/AFY/AFZ + IGNORE_YAW_RATE ⟹ 忽略速度/加速度/yaw_rate，**保留位置 + yaw**。不含 FORCE(512) 与 IGNORE_YAW(1024) |
| `coordinate_frame` | **1**（`FRAME_LOCAL_NED`） | ⚠️ **枚举名有误导性** —— 见下 |
| `header.stamp` | `now()` | ⚠️ 见下 |

⚠️⚠️ **`FRAME_LOCAL_NED` 不代表"你填的值是 NED"。** mavros 对入向的 `FRAME_LOCAL_NED` setpoint 做的是
**ENU→NED 转换**（`setpoint_raw.cpp:231` `transform_frame_enu_ned(position)`）⟹
**我们填进 `PositionTarget` 的是 ROS/ENU 口径**：

- **`position`：x 东、y 北、`z` 向上为正**（不是"向下为正"！照 NED 去翻 z 的符号会得到一架往地里钻的飞机）
- **`yaw`：从 ENU 的 +x 轴逆时针**，实测换算 **`yaw_ned = π/2 − yaw_enu`**
  （`setpoint_raw.cpp:235`，按 `frame_tf.hpp:180` + `ftf_frame_conversions.cpp:33/40` 逐字复算）
  ⟹ **本项目填的 `yaw = 0`，PX4 收到的是 +90°，即"机头朝东"**（不是"保持当前机头"）

★ 也正因如此，读数与 setpoint **同口径**：`local_position` 出向已做 NED→ENU
（`local_position.cpp:157`），进向再做 ENU→NED ⟹ 一进一出恰好抵消，PX4 收到的是它自己报的那个位置。
**这不是巧合，是"两边都在 ROS 口径里"的必然结果** —— 判据与目标生成因此都不需要任何坐标转换。

★ **`target_x/y/z/yaw` 由状态机算**（ticket 08），边界层不再拿"当前位姿"顶替：
`STANDBY` = 当前位置（原地保持）、`TAKEOFF` = 原点 + 斜坡、`HOVER` = 悬停点、`WAIT_START` = 轨迹原点、
`TRACK` = **轨迹原点 + `/traj/reference`**（缺一帧参考就保持上一个目标，不回退到原点）。
"目标"这个概念在 ticket 08 才第一次与"当前"分家。
⚠️ 起飞朝向的处理见上文「setpoint 的 yaw」（ticket 05 那个默认 0 的 `yaw` 参数已删）。

⚠️ **spec 里"发往 PX4 的消息用微秒时间戳"那条不适用于本路径**（那是 Phase 2 走 uXRCE-DDS 时的约定）：
mavros 把 `header.stamp` 换算成 MAVLink 的 `time_boot_ms`（**毫秒**，且会截断成 32 位，
`mavros/include/mavros/plugin.hpp:264-267`），而 **PX4 根本不读这个字段**
（`mavlink_receiver.cpp::handle_message_set_position_target_local_ned` 一律用 `hrt_absolute_time()`
自己打戳，v1.14.3）。所以这里的口径与 mavros 自带的 setpoint 插件一致：`header.stamp = now()`。

## 参数（`config/control.yaml`）

| 参数 | 默认 | 说明 |
|---|---|---|
| `setpoint_rate_hz` | `50.0` | setpoint 频率。PX4 要求进入 OFFBOARD 期间 ≥2 Hz |
| `offboard_prestream_s` | `1.0` | ★ 切 OFFBOARD 前 setpoint 必须先连续流多久。**断流 PX4 会自动退出 OFFBOARD**（`COM_OF_LOSS_T`，z0 现值 1.0 s）⟹ 这是前提不是优化 |
| `offboard_retry_s` | `1.0` | 切模式失败后的重试间隔（真正的成功判据是 `/mavros/state.mode` 变成 `OFFBOARD`） |
| `enter_offboard` | `true` | 自动切 OFFBOARD（参数可关）。⚠️ 关掉它真 PX4 就永远读不到这些 setpoint |
| ~~`yaw`~~ | —— | ☠️ **ticket 08 删掉了**（填 0 ⟹ PX4 收到 +90°，机头朝东）。现在 setpoint 的 yaw 由状态机给：`STANDBY` 跟随当前机头、起飞后锁在起飞机头 |
| ~~`own_arming`~~ | —— | ☠️ **ticket 08 删掉了**（ticket 06 的台架临时缝）。归属改由 `Px4Interface::request_arming()` 真实追踪：只有拿到肯定应答之后观察到的第一个 `armed` 上升沿才算我们的 |
| `diag_period_s` | `1.0` | 诊断日志周期；也是 `/control/state` 的心跳周期 |
| `state_timeout_s` | **`2.5`** | ★ 不是 0.5 s —— 该话题实测 1 Hz |
| `local_pos_timeout_s` | `0.5` | 上游 ~30 Hz |
| `mocap_timeout_s` | `0.5` | 上游 ~196 Hz |
| `rc_timeout_s` | `0.5` | 上游 10 Hz 级。★ 判据**只在已解锁时成立** —— 未解锁时遥控链路有没有都不影响安全，而首飞前遥控又是必须开机的 |
| `battery_timeout_s` | `2.0` | 上游 1 Hz 级。⚠️ **刻意不做判据**（只看值、不看新鲜度），理由见下文「失效降级」的末段 |
| `low_voltage_v` | `20.0` | 低电压阈值（**只看电压，不看电流**）。≈ 6S 的 3.33 V/cell，对齐 PX4 自己的 `BAT_LOW_THR`。⚠️ **换电池必须改**（阈值是按 6S 定的；4S/5S 都不合用），且 z0 的 `COM_LOW_BAT_ACT = 0` ⟹ PX4 只警告、得我们来做 |
| `rc_stick_ov_us_per_s` | `100.0` | 摇杆"在动"的判据 = **变化率**（µs/s），不是偏移量。对齐 PX4 的 `COM_RC_STICK_OV = 20`（0.2/s），按 z0 实测 `RC1_MIN/MAX` 换算 ≈102 µs/s。⚠️ 改了 PX4 那个参数必须同步改这里 |
| `mocap_stamp_stall_s` | `0.5` | 动捕**时间戳**连续不前进多久算"冻住"（话题层看不见的失效） |
| `estimator_timeout_s` | **`2.5`** | ★ 不是 0.5 s —— `/mavros/estimator_status` 实测 1 Hz（z0 现场 1.000 Hz，ticket 16）。⚠️ 它是**新鲜度门**（陈旧的坏读数不作数），不是判据本身 |
| `ekf2_pos_lost_samples` | `2` | `ekf2_pos_lost` 要**连续**几帧报 `pos_horiz_abs=false` 才成立（= 连续帧数，不是累计帧数）。1 Hz ⟹ 这两帧本身占 ~1–2 s，再加上游那两道（400 ms / 1 s，见下）≈ 2.5–3 s 才落地。★ 取 2 的理由是代价不对称：早报的代价是"抖动一下就停发 + 请求 `AUTO.LAND`"，晚报的代价只是慢 1 s（真兜底是动捕那两条 0.5 s 的） |
| `rc_map_roll` … `rc_map_yaw` | `1/2/3/4` | 摇杆通道号（**1-based**）。⚠️ 必须与飞控上的 `RC_MAP_*` 一致 |
| `land_mode` | `AUTO.LAND` | 主动中止任务时请求的模式 |
| `start_channel` | `9` | ★ **1-based**（与 PX4 的 `RC_MAP_*` 同一约定）⟹ ch9 = `/mavros/rc/in` 的 `channels[8]`。⚠️ 值来自 §H.1 的裁定，**别就地改**；⚠️ 台架核对（验收项 6）**尚未做**（EdgeTX 的 CH9 分配 + 遥控链路都不在位） |
| `start_threshold` / `release_threshold` | `0.75` / `0.25` | 归一化阈值（**不是 µs**，与 PX4 的 `*_TH` 同一把尺）。两者之间是**死区**（回差）：不作判断，所以"开关停在阈值附近微抖"不会产生一串边沿 |
| `debounce_ms` | `200.0` | 电平要**连续**保持多久才算确认。★ 参数是毫秒、代码里是秒（§H.1 的裁定就是按毫秒说的）。⚠️ 对上 10 Hz 的帧间隔 ⟹ 要 3 帧；**现场每次压住 ≥0.3 s**，理由见 §H.12 |
| `start_land_hold_s` | `1.0` | ★ ticket 07 增补：**在飞时长按开始键满这么久 ⟹ 立即自动降落**（`end_reason=land_pressed`）。⚠️ 判据从**电平被确认**起算（`debounce_ms` 之后）⟹ **物理按住 ≈1.2 s**；`<= 0` 关闭该功能。★ 只有"在飞态产生边沿"的那一次按下才起算（资格见 `note_start_edge`）|
| `start_low_us` / `start_high_us` | `1000.0` / `2000.0` | 折算的两个端点。★ **方向由这两个数定义**（`low_us` ↔ 0、`high_us` ↔ 1），不由名字说了算 —— 换一个方向相反的通道，**把两个数对调**即可（不改代码）。⚠️ 若哪天把 ch9 也映射进 PX4 用，这两个数必须与那边的 `RC9_MIN/MAX/REV` 对齐（§H.1：`RC9_REV = 1`） |
| `*_topic` / `*_service` | mavros 2.15.1 默认名 | 多机/台架复用不改代码（`traj_*` / `control_traj_progress_topic` 同） |

任务判据的阈值（ticket 08）**单独一组**，与上面那组失效判据刻意分开（那一组管"什么时候停"，
这一组管"飞到什么算到"——合在一起之后调悬停精度会顺手改掉低压阈值）：

| 参数 | 默认 | 说明 |
|---|---|---|
| `takeoff_altitude_m` | `1.0` | 起飞目标高度（相对**起飞原点**） |
| `climb_speed_mps` | `0.5` | 爬升速率上限。★ 工单硬要求 **≤0.5 m/s**，改大直接违反验收项 |
| `takeoff_alt_tol_m` / `takeoff_vel_tol_mps` | `0.10` / `0.10` | 到高判据（高度容差 / 垂速容差） |
| `takeoff_settle_s` | `1.0` | 到高判据需持续多久 |
| `hover_pos_tol_m` / `hover_vel_tol_mps` | `0.10` / `0.10` | ★ 稳定判据（工单原文 `<0.10 m ∧ <0.10 m/s`）。速率是**三维**的 |
| `hover_settle_s` | `2.0` | 稳定判据需持续多久 |
| `wait_start_timeout_s` | `300.0` | ★ ticket 09 **取代**了 ticket 08 的 `hover_hold_s`（那条参数**已不存在**）：降落不再由"停够了"触发，而是由**开始键**或**轨迹跑完**触发；这个数是"操作员一直不按键"这条自己走不出去的态的兜底。`<= 0` = 永不超时。⚠️ **取 < 60 s 节点启动时会发一条 WARN**。★★ **但那不是 §5.3 的时长约束** —— §5.3 的 60 s 悬停归**轨迹 1 自己的时长**（`traj` 的 `hover_duration_s`，在 `TRACK` 里跑），与 `WAIT_START` 停多久**无关**；这个数配短了**不会**让验收飞不够，只会让"操作员还没准备好、飞机就自己落了"⟹ 阈值是**人的准备时间**的量级 |
| `traj_stream_gap_s` | `0.5` | 参考流断了这么久才**重发一次** `/traj/start`（= 连丢 25 帧 @50 Hz）。⚠️ **不许调小** —— 重发会把 `traj` 的参考时钟归零 |
| `traj_timeout_s` | `3.0` | 断了这么久判 `degrade=traj_timeout`。必须**大于**上面那个闸门（够重发一次并等它回来） |
| `takeoff_timeout_s` | `5.0` | 起飞命令的有效期：受理之后这么久还没解锁成功就作废、退回 `STANDBY`（**不自动重试**）。取 5.0 的理由：`/mavros/state` 只有 1 Hz（解锁成功最多晚 1 s 才知道）+ 服务往返 + 余量。⚠️ 它也**是**"前置不满足"的兜底时限 |

★ **还有一条不在参数表里的时限**（`control_node.cpp:request_arming_once` 的局部常量
`stale_after_s = 2.0`）：解锁**请求发出去之后**如果没有应答，2 s 就丢弃它并当作失败
（`note_arming_failed()` ⟹ 当场退回 `STANDBY`）。rclcpp **不会**超时一个没有回应的请求
（`rclcpp/client.hpp`），少了这条就会永久卡在"等 `/mavros/state.armed` 确认"，而日志还在说在等。
它比 `takeoff_timeout_s` 短是**刻意**的：悬空请求是"mavros 挂了/重启了"这个**确定性**故障，
早报早退比等满 5 s 强。⚠️ 它只是**局部常量不是参数** —— 想改得动代码。
（★ 复核指出这条在文档里没记，此处补上。）

## 与其它 ticket 的边界

| 本包**做** | 本包**不做**（归谁） |
|---|---|
| 守卫判定 → `INIT`/`STANDBY` | 轨迹的**几何与时钟**（归 `traj`：选哪条、第 t 秒在哪、完成度怎么算） |
| 50 Hz 流 setpoint、切 OFFBOARD | **上锁**（`COM_DISARM_LAND` 是 PX4 的事，我们只等 `armed` 降沿） |
| **程序解锁**（`/mavros/cmd/arming`）+ 起飞/悬停/判稳/自动降落（ticket 08） | 相对位姿（归 `mocap`）、动捕原始数据（归 `vrpn_listener`） |
| ★ 起飞的两个触发（开始键第 1 次上升沿 / `/control/takeoff`）—— **本节点读** `/mavros/rc/in`（开始键 + 打杆 + 超时） | 操作台的显示与选轨迹（归 `bringup/fly.py`） |
| ★ `TRACK` 的判据：目标 = 轨迹原点 + 参考、`/traj/start` 的时机与重发闸门、完成度收口（ticket 09） | **`traj` 的代码与参数** —— 本张一行都没动（ticket 07 的 Comments ③：三条接线要求全在 control 侧） |
| 输入静默 / 低压 / 位置异常 ⟹ 停发 + `LAND` | `RTL` 等其它降级动作、以及"位置失效 ⟹ 停桨"（不归本包，见 ticket 06 的 Comments） |
| 打杆夺权 ⟹ 停发 + 交还（`RC_OVERRIDE`） | 打杆接管的**执行**在 PX4（`COM_OBL_RC_ACT`），本包只是不再抢 |
| 可复位闩锁 + `degrade=` / `traj_origin=` 可检索日志 | 任何"清故障"命令 / 远程复位接口 |
| `/control/state`、`/control/traj_progress` | `/traj/*` 的任何参数与轨迹几何 |

## 构建与测试

```bash
colcon build --packages-up-to control     # ★ 不要用 --packages-select：control test_depend 了 bench
colcon test  --packages-up-to control --executor sequential   # ★ 串行：并行跑包会在同一个 ROS 域里撞夹具
```

★ **跑测试要独占一个 ROS 域**（`src/bench/README.md`「安全边界」有完整事故记录）：
整仓 `colcon test` **默认并行跑包**，两个包同时用域 73 ⟹ 两边的假 mavros 互相往话题里灌帧 ⟹
本包的 DUT 看到 `armed` 抖动 ⟹ **假红**（实测：`test_control_standby` 9 条挂 5 条，单跑 9 条全绿）。
手动跑单文件时用 `BENCH_ROS_DOMAIN_ID=<非 0/73 的域号>`。

`control` 的测试分两层、七个文件：

| 文件 | 接缝 | 断言面 | 回答的问题 |
|---|---|---|---|
| `test/test_control_standby.py` | **seam 1**（话题边界） | 只断言外部可观测的话题 | 正常路径下飞机**发了什么** |
| `test/test_control_failsafe.py` | **seam 1**（话题边界） | 同上 | 失效时**停止**了什么、请求了什么模式 |
| `test/test_control_takeoff.py` | **seam 1**（话题边界） | 同上（+ 服务应答） | 起飞这一整条链：**解锁、爬升、判稳、记轨迹原点、自动降落、复位** |
| `test/test_control_track.py` | **seam 1**（话题边界） | 同上 | ★ ticket 09：开始键两次上升沿（第 2 次**松手**才进 `TRACK`）、`/traj/start` 的时机与重发、目标 = 轨迹原点 + 参考、参考不来时的原地保持与超时、完成度收口、转发逐值核对；★ ticket 07 增补：`TRACK`/`WAIT_START` 里长按 ⟹ `end_reason=land_pressed` + 停发 + `AUTO.LAND`，起飞途中一直按着 ⟹ 长按被忽略 |
| `test/test_fsm.cpp` | **seam 2**（纯函数，零 ROS） | 零依赖，直接调 `fsm.hpp` | **是哪条守卫**放行的、降级怎么闩锁与复位、判据的**基准**是什么 |
| `test/test_rc_stick.cpp` | **seam 2**（纯函数，零 ROS） | 零依赖，直接调 `rc_stick.hpp` | 摇杆**在不在动**（变化率判据）|
| `test/test_rc_switch.cpp` | **seam 2**（纯函数，零 ROS） | 零依赖，直接调 `rc_switch.hpp` | ★ ticket 09：开始键的**边沿**（12 例）。三条坑在话题层面完全同形，所以在这里各钉一条。★ ticket 07 增补：**长按时长**与**松手事件**（7 例）—— 到点只报一次、松手重新计时、`hold_s <= 0` 关掉、上电即高位不产生、死区不打断计时 |

seam 1 的 DUT 是**子进程**（`control_node` 可执行文件），输入来自**假 mavros + 脚本化动捕**；
夹具在 `src/bench/`，用法见 `src/bench/README.md`。
**为什么两层都要**：四条守卫全失效在话题上的症状**完全一样**（没有 setpoint），
seam 1 到那里就没有诊断分辨率了；而"缺项清单是哪几条、顺序如何"是纯逻辑，在 seam 2 测，
失败信息直接指着那条守卫。

本包刻意**不**加 `ament_lint_auto`（沿用 `mocap` 的先例：全仓库既有 172 条 lint 失败全来自
vendored 的 `vrpn_listener`，新包再加 lint 只是增加噪声）。

覆盖的判据（seam 1）：四条守卫**逐条**缺失时不发 setpoint（每条都配了正控）、
外部解锁时停发、运行中 `/mavros/state` 静默后流在 `state_timeout_s` 内停止、前置流时长（两侧都断言）、
`enter_offboard = false` 只流不切、杀进程后流停止、以及冒烟用例对 setpoint 逐字段与频率的断言
（以上 `test/test_control_standby.py`）。

失效路径（seam 1，`test/test_control_failsafe.py`，19 例）：打杆 ⟹ setpoint 在 0.3 s 内停且
`RC_OVERRIDE` 且**不再请求任何模式**、外部切模式 ⟹ 同样不抢回、**交出即复位**、
三条输入超时各一条判据（`mocap`/`state`/`rc`，每条都断言 `degrade=` token 与 `AUTO.LAND` 请求）、
低压 ⟹ `LAND` 且**可信的落地上锁才复位**（复位前一直请求、复位后 `degrade=none`）、NaN 位置 ⟹ 停发 + `LAND`、
`ekf2_pos_lost` 四条（ticket 16）：报无绝对位置 ⟹ 停发 + `LAND` 且**可信的落地上锁才复位**、
**未解锁时不判**（只停发之外什么都不做）、"连续 2 帧"是**连续不是累计**（假-真交替 3 轮都不降级）、
`estimator_status` 静默 ⟹ **陈旧的坏读数不作数**（这条同时是新鲜度门的变异检查），
`mavros` 的**合成断连帧**两条（ticket 18，夹具 `synth_disconnect()` 补的能力）：
那一帧**不算落地上锁** ⟹ 链路恢复时 PX4 仍 armed 则**仍走 `LAND` 且不重新请求 OFFBOARD**；
而恢复时**真的已上锁**则**仍然复位**（闩锁不许变成单向的；这条钉的是"别把修复只做一半"——
判别力矩阵见 §4.4）、
未解锁失效 ⟹ 只停发且判据恢复**自动复位**、动捕戳**冻住** ⟹ 降级而**短促倒退不降级**、
戳**倒退到旧纪元并持续** > `mocap_stamp_stall_s` ⟹ 降级、
所有输入同时静默 ⟹ 节点不退出且报出最根本那条、通道数不足的遥控帧不崩、
外部解锁 ⟹ 只停发不请求 `LAND`（ticket 06 的 `own_arming` 缝已删 ⟹ 这一条的注入面从"翻一个参数"
换成了"不调 `/control/takeoff`"，基线也随之从 `AUTO/STANDBY` 变成 `AUTO/TAKEOFF`）。

起飞链（seam 1，`test/test_control_takeoff.py`，8 例）：全链冒烟（受理 ≠ 起飞 ⟹ 解锁 ⟹ 爬升到 1 m
且速率 ≤0.55 m/s、水平两轴钉在原点 ⟹ 判稳 ⟹ `traj_origin=` 与判稳那一刻的位置一致 ⟹ `WAIT_START`
的目标恰是轨迹原点 ⟹ `MANUAL/LAND` + 请求 `AUTO.LAND` + setpoint 停发 ⟹ 外部上锁 ⟹ 自动复位到
`MANUAL/STANDBY` ⟹ 第二次起飞被受理）、遥控不在线时**受理了但永不解锁**、解锁被拒 ⟹ 报 `MAV_RESULT`
且**不自动重试**、任务在跑时第二次起飞被拒（这条同时是"命令超时作废"的外部观测面）、
爬升速率两侧（`≤0.55` **且** `≥0.40` m/s —— 只断言上界的话"根本没爬"也会通过）、
判稳的两半各一条反例（速率不成立 / 位置不成立 ⟹ 不记轨迹原点）、
进入 `TAKEOFF` 后机头锁定（夹具注入非零机头）。

★轨迹链（seam 1，`test/test_control_track.py`，7 例）：两次上升沿的完整任务（第 1 次解锁起飞 ⟹
`WAIT_START` ⟹ 第 2 次进 `TRACK`）、**目标 = 轨迹原点 + 参考**（不是参考本身）、运行中重选保持上一个
目标并按闸门重发 `/traj/start`、参考一直不来则原地保持且 3 s 后 `degrade=traj_timeout`、
完成度到 1.0 ⟹ `end_reason=traj_done` + `LAND` + 上锁、**第二个任务不被上一轮留下的完成度 1.0 当场收口**、
`/control/traj_progress` 逐值转发。★ 前两条与最后一条的**反例**都在同一批里：T3 是"参考断了"那句
日志的**正控**（T5 断言正常收尾**不许**出现它）。

★★ **"假飞机"现在住在 `bench` 里**（`bench/plant.py` 的 `Plant` + `fly()` / `fly_until()`）：
速率受限地逼近 setpoint，速度 = 实际位移 / dt。它刻意**只在收到新 setpoint 时**才走一步 ——
否则探针的轮询比 DUT 的 50 Hz 快时会观测到 0 与 2× 交替的假速度抖动。它**不是**夹具的一部分
（那是**场景**逻辑："这一轮飞机该怎么飞"，与 `Timeline` 同类）。★ 它原本是
`test_control_takeoff.py` 文件末尾的私有 `_Plant`，**ticket 09 落地时**按那份文档写下的纪律
**提升进了 `bench` 并补上自检**（`bench` 自检里有它"真的会动、速度是真的"两条）；`control`
的两个 seam 1 文件现在共用它。

★★ **"假 traj"是 ticket 09 新加的**（`bench/fake_traj.py` 的 `FakeTraj`）：它刻意复刻
`traj_node.cpp` 的**两个坑**，因为那两条正是 `control` 侧判据的分界线 ——
① 没 `select` + `start` 就**一帧参考都不发**；② 跑完之后**照旧 50 Hz 发**、完成度**停在 1.0**。

seam 2（`test/test_fsm.cpp`，72 例）：四条守卫**逐条**掀掉时缺项清单恰好是那一条（不是只断言"不放行"——
那样"某条守卫恒假"也会通过）、缺项顺序固定、全零 `Guards` 里"未解锁"成立（默认值的纪律）、
`STANDBY` 下控制权仍是 `MANUAL`、前置流时长两侧、已在 OFFBOARD 不再请求、**断流后前置流重新计时**；
降级路径的**每条分支**：原因优先级（同一帧里多个判据成立时谁说话）、未解锁失效不闩锁 ⟹ 自动复位、
已解锁失效闩锁 ⟹ **不因输入恢复而空中自动接管**、可信的落地上锁才复位、
**合成断连帧不算落地上锁**（三条，ticket 18：那一帧之后闩锁仍在、链路恢复且 PX4 仍 armed ⟹ 仍走
`LAND` 而不重新请求 OFFBOARD、而真的已上锁 ⟹ 仍然复位 —— 后者拦的是"只改一半"：判据加了
`connected` 却仍无条件刷新 `was_armed_` 的话，那帧会把 `armed` 的降沿吃掉 ⟹ 单向 latch）、
**低压闩锁被夺权升级**、夺权闩锁不被输入超时改写、自己请求的 `AUTO.LAND` 生效**不算夺权**、降级期间不抢回 OFFBOARD、
外部交回即复位、`rc_override` 与 `mode_lost` 的先后（打杆后模式才掉出来仍报打杆）、
`ekf2_pos_lost` 的解锁门（未解锁不算失效）与**优先级**（同一帧里 `mocap_stamp_stalled` 说话，
理由：戳冻住是**原因**、EKF2 拒用是**结果**）、以及 `DegradeNames` 的**逐字符守卫**
（任何 `degrade=` token 跑出文档写的 `[a-z0-9_]` 都会变红 ⟹ 日志配方与代码词表不会各自漂移）；
起飞/悬停的 15 例（ticket 08）：受理后仍停在 `STANDBY`、解锁请求**一次性**、前置不齐不发请求、
遥控不在线不发请求且超时作废、被拒当场退回且**一帧都不补发**、解锁那一帧锁原点与机头、
爬升斜坡**逐帧**核对并封顶、**到高判据看原点+高度而不是当帧 `target_z`**、
**"持续"是连续不是累计**（一帧不合格就重新计时）、`velocity_valid` 必须为真、
判稳两条判据各一条反例、**轨迹原点记的是判稳那一刻的位置而不是悬停点**、
`TRACK` 完成度 ≥1.0（或 `WAIT_START` 超时）走降落出口、落地上锁后原点/轨迹原点全清
（且能重新起飞）、起飞后机头锁定而 `STANDBY` 跟随当前机头。

轨迹链（seam 2，同一文件，ticket 09）：**边沿按任务态归属**（`kIdle`⟹起飞、`kWaitStart`⟹**松手**进 `TRACK`、
在飞的态⟹**锁存降落资格**、`kRequested`/`kDone`⟹丢掉 —— 含"第二个任务的第一次按键**不会**被当成第 2 次"这条）、
**参考在 `TRACK` 开头就带 1.0 时不收口**（`track_started_`）、断流时**保持上一个目标**且
闸门到点才重发 `/traj/start`、`traj_stale` 的计时起点是**最后一帧参考**（不是进 TRACK 那一刻）、
`WAIT_START` 的超时出口（`end_reason=wait_start_timeout`，含"配成 `<= 0` 就不超时"那条边界）；
★ ticket 07 增补的 7 例：`TRACK` 里长按 ⟹ `Phase::kLand` + `request_land` + `end_reason=kLandPressed` + 停发、
**未解锁时按下的长按作废（即使到点时已解锁）** —— 就是"按完起飞一直按着⟹刚离地就降落"那个反例、
`WAIT_START` 里长按 ⟹ 直接降落且一帧 `TRACK` 都没进过、`WAIT_START` 里短按**松手** ⟹ 进 `TRACK`、
夺权闩锁里长按 ⟹ 什么都不请求、已在降级收口中 ⟹ 不报 `land_pressed`。
⚠️ **`end_reason` 的词表没有逐字符守卫**（`degrade=` 那一套有 `DegradeNames`，`end_reason` 没有对应的
一条 —— `to_string(EndReason)` 目前只被 `EXPECT_EQ` 按**值**断言过）。三个 token（`traj_done` /
`wait_start_timeout` / `land_pressed`）眼下都合法，所以不是缺陷，只是**两套日志词表的守卫不对称**；文档若哪天为
`end_reason=` 也发一条 grep 配方，就该把这条补上。

seam 2（`test/test_rc_stick.cpp`，13 例）：变化率判据的边界 —— 第一帧不作判断、快速推杆/反向推杆算在动、
**慢速大行程漂移不算在动**、推到头保持不动会衰减到阈值以下、非正周期与 NaN 不作判断（且不污染状态）、
单帧越过阈值不算、默认阈值与固件 `COM_RC_STICK_OV` 同源。

seam 2（`test/test_rc_switch.cpp`，19 例，ticket 09 + 07 增补）：开始键**边沿**判据的三个坑各钉一条 ——
① **上电即在高位不算边沿**（必须先确认过低位）；② **短于去抖窗的高位脉冲不算**、
死区读数**不打断**去抖计时、一直停在阈值带内不产生边沿；③ **一次边沿只能被取走一次**
（"取走"是消费语义 —— 可读的边沿会在下一帧被再读一次 ⟹ 一次按键被当成两次：第 1 次解锁、
第 2 次开轨迹）。另有三条边界：`reset()` 之后**必须重新看见低位**才承认下一次高位是边沿
（断链期间开关动没动过我们不知道）、**折算方向由参数决定**（两个端点**允许反着配**）、
端点配成相等时永远不产生边沿、越界读数不钳位（越界有信息量）。
★ 增补的 7 例（ticket 07）：**长按事件在恰好按满 `hold_s` 的那一帧产生且只产生一次**、
按不到 `hold_s` 就松手不产生、松手后再按**重新计时**、`hold_s <= 0` 整个关掉、
**上电即在高位不产生长按**（不是"操作员按下了"）、死区读数**不打断**长按计时、`take_release()` 是事件。

⚠️ **没被自动化覆盖的一处**：`request_mode()` 的重试节流（重复请求抑制、悬空请求丢弃、
重试间隔）。它是**输出侧的发送节奏**而非输入判据，只有"切模式恰好失败/悬空"时才走到 ——
真台上的表现是"日志说在重试，实际没重试"，改动这段请连带跑一次台架。

## ⏳ 还没做的验收项（需要有飞控空闲的窗口）

| 项 | 为什么还没做 |
|---|---|
| ★ **验收阶梯 1–4 级**（无桨台架 / 系绳离地 20 cm / 系绳闭环悬停 30 s / 自由飞 60 s） | **每一步都是人的物理动作**（拆桨、系绳、扶飞机、按 `/control/takeoff`），夹具替代不了。见下 |
| 真 PX4 被拉进 OFFBOARD | `COM_OF_LOSS_T` / `COM_OBL_RC_ACT` / 模式需求检查都是 **PX4 侧**行为，夹具不模拟。需要在**拆桨**台架上跑一次 |
| **解锁被飞控真的拒绝一次**（看 `MAV_RESULT=` 码） | 夹具能造"被拒"（`set_arming_policy('fail')`），但**真 PX4 的 prearm 检查**不模拟 ⟹ "哪种拒绝对应哪个码"要上真飞控 |
| **上锁确实由 `COM_DISARM_LAND = 2.0` 完成** | 夹具的 `armed` 是我们自己置的；真机上这条依赖 PX4 的着陆检测（落地判定 + 2 s） |
| ⚠️ **起飞时 `/mavros/state` 的 1 Hz 延迟** | 解锁成功最多晚 1 s 才知道 ⟹ 原点锁的是"**看到** armed 那一帧"的位置。台架上这一秒内的位移可以忽略，**真机上要确认飞机在被锁原点之前没有漂出容差** |
| 杀节点 → PX4 在 `COM_OF_LOSS_T`（1.0 s）后自行退出 OFFBOARD | 同上；夹具这一半已由 `test_杀掉_DUT_之后_setpoint_流停止` 覆盖 |
| **打杆 → PX4 按 `COM_OBL_RC_ACT` 接管**（z0 现值 `0` = 回 Position 模式） | 同上：执行在 PX4。本节点这一半（立刻停发、退到 `RC_OVERRIDE`、**不抢回 OFFBOARD**）已由 `test_control_failsafe.py` 覆盖 |
| 低压判据面对的是**真电池**读数 | 台架注入的是假 mavros 的电压，判据逻辑已覆盖；**真实读数可不可信**是 ticket 01 的 ADC 判定（⚠️ 判据本身只看电压、不依赖电流） |
| ★ **ticket 09 的开始键台架核对**（`/mavros/rc/in` 的 `channels[8]`，验收项 6） | **两个前置不在位**：EdgeTX 的 CH9 分配是人工动作（§H.4）、本机遥控链路当前是断的（§H.9）⟹ 没得核。★ 上电时**先做这一条再做自由飞** —— 通道错了的话第 1 次上升沿会直接去解锁起飞 |
| ★ **三条轨迹的自由飞**（悬停 / 正方形 / 八字）+ §5.3 的 60 s 定点 | **人的物理步骤**（装桨 + 空域）。代码侧已就绪；判读靠 `end_reason=` / `遥控器开始键：` / `TRACK 进行中：` 这几条固定形状的日志 |
| ~~**`NAV_RCL_ACT = 2`（Return）这条依赖**~~ | ✅ **A1 已落地**（2026-09-28 实测 `NAV_RCL_ACT = 1` Hold）⟹ "RC 丢失超时 ⟹ LAND"之后 PX4 自己那条 RC 失效保护是**就地保持**，不会返航 |
| ~~**"EKF2 位置失效"的真载体**~~ | ☠️ **ticket 16 已定案（2026-09-22）**：载体找到了，就是 `ekf2_pos_lost`（`/mavros/estimator_status.pos_horiz_abs`），**台架上可注入可测**（`test_control_failsafe.py` 四条 + bench 自检两条）⟹ 这一行不再挂"待办" |

做上面几项时 **必须在台架上拆桨**，并按 `docs/control/phase1-implementation.md` §7 的顺序来 ——
那一节与 `.scratch/phase1/issues/08-takeoff-hover-land.md` 的验收阶梯是同一条路线。

★★ **阶梯第 1 级（无桨台架）能看和不能看的东西**：拆了桨就没有推力，所以
`|z − 1.0| < 0.10 m` **按字面永远不可能成立** ⟹ 第 1 级的判据**只到 `TAKEOFF` 为止**
（电机按预期转、`TAKEOFF` 的 setpoint 斜坡正确），HOVER 及之后的判据在第 2/3/4 级。
这条是对 `phase1-implementation.md` §7 第 3 步的**修正**（本张落实），别照旧文档去台架上等 HOVER。

## 目录

```
include/control/fsm.hpp           ★ 纯逻辑、零 ROS：两个正交维度 + 守卫清单 + 降级判据/闩锁（判据都在这里）
include/control/rc_stick.hpp      ★ 纯逻辑、零 ROS：摇杆"在动"的变化率判据（对齐 COM_RC_STICK_OV）
include/control/rc_switch.hpp     ★ 纯逻辑、零 ROS：开始键的边沿判据（归一化阈值 + 死区 + 去抖 + 消费语义）
include/control/px4_interface.hpp mavros 边界：话题/服务名、超时口径、阈值依据、QoS 依据
src/px4_interface.cpp             订阅/发布/服务 + 每条输入的新鲜度与健康量
src/control_node.cpp              只做接线：参数、定时器、日志（★ 不要在这里加判断）
config/control.yaml               参数（含每一条的"为什么是这个值"）
launch/control.launch.py          一键启动（只起本节点）
test/test_fsm.cpp                 ★ seam 2：纯函数单测（零 ROS 依赖）
test/test_rc_stick.cpp            ★ seam 2：变化率判据单测（零 ROS 依赖）
test/test_rc_switch.cpp           ★ seam 2：开始键边沿判据单测（零 ROS 依赖）
test/test_control_standby.py      ★ seam 1：子进程 DUT + 假 mavros，只断言话题（正常路径）
test/test_control_failsafe.py     ★ seam 1：同上（失效路径）
test/test_control_takeoff.py      ★ seam 1：同上（起飞 → 悬停 → 自动降落）
test/test_control_track.py        ★ seam 1：同上（开始键 → TRACK → 任务收口）
```

★ 用例里用的**假飞机**（`Plant`）与**假 traj**（`FakeTraj`）住在 `src/bench/`，
不在本包的测试文件里 —— 夹具与场景逻辑归 `bench`，见 `src/bench/README.md`。
