# 轨迹飞行一页通 —— 从起链到收口（**人工操作**）

> **本文件是什么**：把"飞一次轨迹"现场要按的顺序串成一条线 —— 遥控器上每个通道干什么、
> 状态机怎么走、起链发哪几条命令、每一步怎么判读、出事了先看哪。
> **它是一张导航图**：每一段都指回权威出处，**不新造结论**。
>
> **它不是什么**：不是判据与记录的权威、不是命令的出处、也不给任何"让飞机动"的入口。
>
> | 想知道 | 权威在哪 |
> |---|---|
> | 起链的命令 / 每步核对 / 现场排障 | [`run/README.md`](../../run/README.md)（★ **命令的唯一出处**是 `run/z0.py` 的 `ACTIONS`） |
> | 遥控通道的裁定与依据（ch5/6/8/9、阈值、现场动作要点） | [`../reference/params-changes.md`](../reference/params-changes.md) **§H** |
> | 拆桨台架那 8 项（判据、读数、勾哪张票） | [`bench-runbook.md`](bench-runbook.md) |
> | 状态机 / 判据 / 降级 / 日志 token | [`../../src/control/README.md`](../../src/control/README.md) |
> | 参数（起飞高度、判稳阈值、超时） | [`../../src/control/config/control.yaml`](../../src/control/config/control.yaml) |
> | 术语（控制权、任务态、轨迹原点…） | [`CONTEXT.md`](../../CONTEXT.md) |
>
> ⚠️ **诚实交代（截至 2026-09-23）**：验收阶梯的第 **1–4** 级（无桨台架 / 系绳离地 20 cm /
> 系绳悬停 30 s / 自由飞）**一次都没做过**；**EdgeTX 侧给 ch9 分配开关（§H.4）还没做**、
> ch9 的台架核对（§H.8，验收项 6）**还没做** ⟹ **第一次上电先做 §5.2 那条核对**：
> 通道错了的话，**第 1 次上升沿会直接去解锁起飞**。
> ⚠️ **首飞阻塞项 A1**（`NAV_RCL_ACT` `2` → `1`）未落实：`2` = 返航，室内无 GPS
> ⟹ 遥控一丢 PX4 会**试图返航**（[`params-changes.md`](../reference/params-changes.md) §A1 / §H.7.1）。

---

## 0. 一页流程（时间轴）

```
① 摆好飞机（静止、固定）           ← 人；必须在起 mocap 之前
② 一键起链   run/up.sh             ← 人敲一条；mavros → vrpn → mocap → traj + control
③ 起链核对   run/z0.py check       ← 人看一眼那张表（§5.1）
④ 选轨迹     fly.py 敲 1/2/3       ← 人；**起飞前选好**
⑤ ch6 拨到第 6 档（Offboard）      ← 人；★ 必须在按开始键之前
⑥ 开始键第 1 次上升沿（压住 ≥0.3 s）← 人；= 程序解锁 + 起飞
   ── 自动 ──▶ 爬到 1 m（≤0.5 m/s）→ 判稳 2 s → 停在 WAIT_START（≤300 s 兜底）
⑦ 开始键第 2 次上升沿              ← 人；= 进 TRACK，飞所选轨迹
   ── 自动 ──▶ 完成度 ≥ 1.0 → 请求 AUTO.LAND → PX4 落地 2 s 后自己上锁 → 状态全复位
⑧ Ctrl-C 收链（在 `run/up.sh` 那个终端里）
```

★ **"飞"这件事上人一共只做 4 个动作**：**摆飞机、选轨迹、拨 ch6、按两次开始键**
（起链 ② / 核对 ③ / 收链 ⑧ 是另外三条命令，"飞"本身全是自动的）。
★ **两条不能反的顺序**：**摆飞机在起 `mocap` 之前**（阶段 A 原点在节点启动那一刻锁定）；
**ch6 在按开始键之前**（解锁前置要求"已在 OFFBOARD"，否则 5 s 后命令作废，见 §2.3）。
★ 想再来一次：**不需要重启任何节点** —— 重选轨迹、把 ch6 保持在 Offboard、再按开始键 ①、
② 即可（边沿的归属按**任务态**判、不数次数，见 §2.2 的 ★）。

---

## 1. 遥控器：四条摇杆 + 四个开关

`RC_MAP_*` 的占用是逐条第一手核过的（[`params-changes.md`](../reference/params-changes.md) §H.2）：
**非 0 的只有 7 条** —— `RC_MAP_ROLL/PITCH/THROTTLE/YAW` + `ARM_SW`/`FLTMODE`/`KILL_SW`
（= ch1–4、ch5、ch6、ch8；与 [`z0.params`](../assets/z0.params) 逐行核对一致）
⟹ 其余通道（含 **ch9**）**拨了不会切模式、不会解锁、不会杀车**。

| 通道 | 是什么 | 谁读它 | 现场怎么用它 |
|---|---|---|---|
| **ch1–ch4** | 摇杆：横滚 / 俯仰 / 油门 / 偏航（`RC_MAP_ROLL/PITCH/THROTTLE/YAW = 1/2/3/4`） | PX4（手动模式）+ **我们**（打杆夺权判据） | 飞行中**别碰杆**：判据是**变化率**，任何一次有意的打杆都会把我们踢出 AUTO（§2.4） |
| **ch5** | 解锁开关（`RC_MAP_ARM_SW = 5`，**保留的冗余硬件解锁路径**） | PX4 | ⚠️ **现场不要用它**：拨它会让 `armed` 变化**不是我们发起的** ⟹ 我们停发并退回 `MANUAL/INIT`（§2.3 的外部解锁语义） |
| **ch6** | 模式开关（`RC_MAP_FLTMODE = 6`） | PX4 | ★ **第 6 档 = Offboard**（`COM_FLTMODE6 = 7`，`7 = Offboard`）。★ 它**只有 3 个档在位**：`1 = Position`、`4 = Altitude`、`6 = Offboard`，`2/3/5` 未分配（**拨过去没有任何反应**）。**整段任务期间停在它上面**；拨离 = PX4 离开 OFFBOARD，我们判 `mode_lost` ⟹ 停发 + 交还，连 `AUTO.LAND` 都不请求 |
| **ch8** | 杀车开关（`RC_MAP_KILL_SW = 8`，`RC_KILLSWITCH_TH = 0.75`） | PX4 | 高值 = **立即停桨**（`ACTION_KILL → manual_lockdown`）⟹ 在空中拨它 = **飞机失去动力掉下来**。★ 唯一的人手急停，**不是"取消任务"**（§H.7） |
| **ch9** | **开始键**（`start_channel = 9`，我们读 `/mavros/rc/in` 的 `channels[8]`） | **我们** | 一档两位置开关，**两次上升沿**（§H.5）：第 1 次 = 解锁 + 起飞、第 2 次 = 进 `TRACK`。⚠️ **不与其他通道复用** ⟹ 拨它不会动 PX4 |

★ ch6 那两行的依据（逐字，不是印象）：`z0.params` 的 `COM_FLTMODE1/4/6 = 2/1/7` + 固件
`src/modules/commander/commander_params.c:361-374` 的 `@value` 清单（枚举是 **0 起**：
`2 = Position`、`1 = Altitude`、`7 = Offboard`）—— 见 [`params-analysis.md`](../reference/params-analysis.md) §(b)。

### 1.1 ★ 开始键的现场动作要点（两次上升沿）

- **上电即在高位不算边沿** —— 必须先见到低位（`< 0.25`）才承认下一次上升沿；上电时若它已停在
  高位，我们**不会自动解锁**（§H.5 第 2 条）。所以：**起飞前先把开关拨到低位**。
- **每次压住 ≥ 0.3 s 再松**（§H.12，台架实测）：去抖 200 ms 对上 `/mavros/rc/in` 的 10 Hz
  ⟹ 要**连续 3 帧**才算确认，压得太贴边会有一半概率按不动（且**连"被丢掉"的日志都没有**）。
- **不在能受理它的任务态里按 = 丢掉**，且日志会写"上升沿**被丢掉**了"（含当前任务态）——
  现场"按了没反应"**有三种**，先看日志里有没有那两条之一，再往下分：
  ① **边沿没产生**（压太短 / 没先回低位）⟹ 连日志都没有；
  ② **产生了但被丢掉**（任务态不对）⟹ 有"被丢掉"那条；
  ③ ★ **受理了但没起飞** ⟹ 有"第 1 次上升沿"那条，却没过 `armed` 那一步 ——
  这时**不是按键的问题**，去看 §2.3 那三条前置（最常见：ch6 不在 Offboard 档、遥控不在线），
  `takeoff_timeout_s = 5 s` 一到命令就作废，**要再按一次**。
- **阈值是归一化的，不是 µs**（`0.75` / `0.25`，死区在两者之间；折算成 µs 约 `1750` / `1250`）。
  ⚠️ 折算用的那两个端点（`start_low_us` / `start_high_us` = `1000 / 2000`）这把尺子**还没被验过**
  —— 本轮实测的 `chan*_raw` **落不到一条能自洽的 µs 刻度上**（§H.3 的"单位未定案"）⟹
  §5.2 的核对要连它一起看：实在不行改的是那两个端点，**不是阈值**。

### 1.2 想停任务，只有两条软路径（都要遥控器在场）

| 路径 | 动作 | 结果 |
|---|---|---|
| **(a) 打杆** | 快速推杆（变化率 > `100 µs/s`） | 我们 0.3 s 内**停发**、退到 `MANUAL/RC_OVERRIDE`、**不请求任何模式**；PX4 按 `COM_OBL_RC_ACT = 0` 落回 **Position** |
| **(b) 拨 ch6** | 拨离第 6 档 | PX4 离开 OFFBOARD、用它自己的控制器（§H.7 表） |
| ☠️ **(c) 拨 ch8** | —— | **停桨坠落**，不是停任务 |

⚠️ **遥控链路本身断掉时，(a)(b) 都不存在**（它们都要遥控器）。那时只剩 **offboard 断流保护**：
我们一停发，PX4 在 `COM_OF_LOSS_T = 1.0 s` 后自己退出 OFFBOARD、按 `COM_OBL_RC_ACT` 处置。
⚠️ 所以**解锁前遥控必须开机、链路已建立** —— 这不只是纪律：`failsafe.cpp` 里若解锁那一刻 RC
就是丢的，PX4 把遥控当"可选"、**RC 丢失保护被忽略**，而 `NAV_RCL_ACT = 2`（未改之前）会让
"飞行途中丢遥控"变成**试图返航**（§H.7.1，A1 阻塞项）。

---

## 2. 状态机（FSM）：两个维度 + 一条正常路径 + 一条失效路径

### 2.1 两个正交维度（别读成一个扁平枚举）

| 维度 | 取值 | 含义 |
|---|---|---|
| **控制权** | `MANUAL` / `AUTO` | 谁在控制。★ `AUTO` 的定义是"**我们正在发会驱动飞机的指令**（已解锁的 offboard）" ⟹ **已切进 OFFBOARD 但未解锁仍是 `MANUAL`** |
| **任务态** | `INIT` / `STANDBY` / `TAKEOFF` / `HOVER` / `WAIT_START` / `TRACK` / `LAND` / `RC_OVERRIDE` | 程序在干什么 |

`/control/state` 发的就是 `"<控制权>/<任务态>"`（如 `MANUAL/STANDBY`），**`transient_local`**
⟹ 晚订阅的终端也立刻拿得到当前值（`ros2 topic echo /control/state`）。
★ **`MANUAL/INIT` 与 `MANUAL/STANDBY` 都是正常起始态，不是故障**：起早了就停在 `INIT`，
日志会**逐条列出缺什么**，补齐后自己往下走。

### 2.2 正常路径（每一步谁触发、判据是什么、日志怎么认）

```
MANUAL/STANDBY ──开始键①（或台架 /control/takeoff）──▶ 仍在 STANDBY：发解锁请求，等 armed
      │ armed 上升沿 ⟹ 锁起飞原点 + 锁机头
      ▼
AUTO/TAKEOFF ──爬到 原点+1.0 m，≤0.5 m/s──▶ AUTO/HOVER ──判稳 2 s──▶ AUTO/WAIT_START
                                                                        │ 开始键②
                                                                        ▼
                                                                   AUTO/TRACK
                                                        （setpoint = 轨迹原点 + /traj/reference）
                                                                        │ 完成度 ≥ 1.0
                                                                        ▼
                                                          MANUAL/LAND（请求 AUTO.LAND，停发 setpoint）
                                                                        │ PX4 的 COM_DISARM_LAND=2.0
                                                                        ▼
                                                          MANUAL/STANDBY（armed 降沿 ⟹ 全部复位）
```

| 事 | 判据（都是自动的） | 日志 token |
|---|---|---|
| 放行（进 `STANDBY` 流 setpoint） | 四条守卫齐：**已连接 / 动捕新鲜 / `local_position` 新鲜 / 解锁是我们发起的** | 缺项清单在诊断日志里逐条列出 |
| 切 OFFBOARD | 先流够 `offboard_prestream_s = 1.0 s` 再请求（**断流 PX4 会自动退出 OFFBOARD**） | `mode` 变 `OFFBOARD` |
| 解锁 | 程序解锁（`/mavros/cmd/arming`），**三条前置见 §2.3** | `MAV_RESULT=0`（`0` 之外一律走"被拒"路径，**不自动重试**） |
| 起飞原点 | **看到 `armed` 那一帧**的 EKF 位置（不是发请求那一帧） | `takeoff_origin=(x, y, z)` |
| 到高 | `\|z − (原点+1.0)\| < 0.10 m ∧ \|v_z\| < 0.10 m/s` **持续** `takeoff_settle_s = 1.0 s` | —— |
| 判稳 | 位置误差 < 0.10 m ∧ **三维**速率 < 0.10 m/s **持续** `hover_settle_s = 2.0 s` | —— |
| **轨迹原点** | **判稳那一刻** EKF 报的位置（★ 此后 `WAIT_START` / `TRACK` 的一切都以它为 0） | `traj_origin=(x, y, z)` |
| `WAIT_START` 两个出口 | ① 开始键② ⟹ `TRACK`；② 等满 `wait_start_timeout_s = 300 s` ⟹ 自动降落 | ②`end_reason=wait_start_timeout` |
| `TRACK` 收口 | 参考完成度 `progress ≥ 1.0`（★ 必须先见过一次 < 1.0） | `end_reason=traj_done` |
| 上锁 | **不是我们做的**：PX4 按 `COM_DISARM_LAND = 2.0 s` 自己上锁 ⟹ 我们只等 `armed` 降沿，那一帧**全部复位** | —— |

★ **"持续"是连续的、不是累计的**：一帧不合格就重新计时。
★★ **边沿的归属按任务态判、不数次数**：`kIdle` ⟹ 起飞、`kWaitStart` ⟹ 进 `TRACK`、**其余一律丢掉并记日志**。
用计数器数"第几次按键"是错的（落地后计数器不归零，第二个任务的第 1 次按键会被当成"第 2 次"，
飞机在操作员只想起飞的时候直接开跑轨迹）。

### 2.3 ★ 起飞的三条前置（缺一条就一帧都不发解锁请求）

| 前置 | 谁保证 | 现场含义 |
|---|---|---|
| 四条守卫全成立 | `control` | 动捕在发、`local_position` 在发、mavros 连着 |
| **已在 OFFBOARD** + setpoint 已在连续流 | PX4 模式 = `OFFBOARD` | ★ **先把 ch6 拨到第 6 档**（§H.6 原文两条） |
| **遥控在线**（`/mavros/rc/in` 新鲜） | 机器判据，不是人肉清单 | **遥控器开机、链路已建立** —— 台上没接接收机时**永远解不了锁**（刻意如此） |

★ **受理 ≠ 起飞**：前置不齐就一帧都不发解锁请求，由 `takeoff_timeout_s = 5.0 s` **兜底作废、退回
`STANDBY`**，**不自动重试**。⟹ 现场表现是"按了开始键、什么也没发生" —— 先看诊断日志缺哪条，
最常见的两条是 **ch6 不在 Offboard 档**、**遥控不在线**。
★ 解锁被拒是**预期事件**（动捕没数据、位置无效时 PX4 会拒），我们**不静默重试**：
`MAV_RESULT` 与拒绝原因会报出来。

### 2.4 失效降级：现场只需要认这几条

**总原则：任何一条关键输入坏掉 ⟹ 立即停止发 setpoint**，然后按**是否已解锁**分岔。
两类处置**完全不同，不要合并**：

| 类 | token | 现场现象 | 谁在动 |
|---|---|---|---|
| **夺权**（控制权被拿走） | `rc_override` / `mode_lost` | 我们停发、退到 `MANUAL/RC_OVERRIDE`，**连 `AUTO.LAND` 都不请求** | 飞行员（打杆 / 拨 ch6） |
| **输入坏了**（已解锁时） | `state_timeout` / `mocap_timeout` / `local_pos_timeout` / `rc_timeout` / `battery_low` / `position_invalid` / `mocap_stamp_stalled` / `ekf2_pos_lost` / `traj_timeout` | 我们停发 + **请求 `AUTO.LAND`** | PX4（自己落） |

★ 日志里一律 `degrade=<token>`，复位那一帧写 `degrade=none` ⟹
`grep -oE "degrade=[a-z0-9_]+" 日志 | grep -v none` 捞出全部降级事件。
⚠️ **但这条流水线会捞到一条"假事件"**：`TRACK：轨迹参考断了` 那条 WARN 的**说明文字里**
就写着 `再撑 N s 没回来就是 degrade=traj_timeout`（`control_node.cpp:489-492`，逐字）⟹
它**本身不是一次降级**，只是预告。★ 判读时以**行首**为准：真降级那条是**行首就是**
`degrade=<token>（持续 N s…）—— …` 的 ERROR（`control_node.cpp:911-918`），
而"参考断了"那条是 WARN、`degrade=` 出现在**句子中间**。
★ 判据细节（阈值、为什么是 2.5 s、"连续不是累计"、可信的落地上锁）**不要凭这张表猜**，
见 [`../../src/control/README.md`](../../src/control/README.md) 的「失效降级」一节。

### 2.5 降级之后想复位，只有两条路（**都不要去"帮它一把"**）

1. **落地上锁**（主路径，对四类原因都有效）—— 判据是 `connected=true ∧ armed=false`
   那次**可信的**降沿；链路静默期间 mavros 自己发的那帧合成断连**不算**（ticket 18）。
2. **夺权场景**：让**摇杆停止动作**，再把模式**离开、然后交回** OFFBOARD（判的是模式升沿）。

⚠️ 闩锁期间我们**从不自己请求 OFFBOARD** ⟹ 模式回到我们手里的那条升沿**只可能来自人**。
⚠️ **不要用 QGC 抢模式** —— 那会让"谁在控制"变得说不清（`control/README.md` 运行纪律 3）。

---

## 3. 起链：命令与顺序

### 3.1 一键（推荐）：`run/up.sh`

```bash
run/up.sh
```

把 5 个终端收成 1 个：**mavros（带 GCS 桥给 QGC）→ vrpn_listener → mocap → traj + control**，
**逐段等它真的起来**再加下一步，起完打印核对表，然后**前台守着（Ctrl-C 一起停）**。

| 步 | 起什么 | 起之前 | 起来的判据（**等**，不是"起了就算"） |
|---|---|---|---|
| 1 | mavros + **GCS 桥** | 串口被谁占着；已有 mavros 在跑就问一句"沿用还是照样再起" | `/mavros/state` 的 **`connected=True`**（≤60 s）★ 有帧 ≠ 连上飞控 |
| 2 | vrpn_listener | —— | `/vrpn/<刚体>/pose` 有帧（≤30 s；刚体名 `Z0`/`z0` 都认） |
| 3 | mocap | ★ **要你确认"飞机已摆好、静止、固定？"** | `/mocap/pose_enu` 有帧（≤30 s） |
| 4 | traj | —— | **不等**（没 `select` 就一帧不发） |
| 5 | control | —— | `/control/state` 有帧（≤30 s） |
| 6 | —— | —— | 打印核对表 ⟹ 前台守着 |

- **日志**：`run/logs/<时间戳>/01-mavros.log … 05-control.log`（判读靠它们；卡住了先看最后几行）。
- ⚠️ **别 `kill -9` 这个脚本**：`SIGKILL` 不可捕获、转发就没了，会**留下 mavros 占着串口**。
- ★ **它不代做"还差的那两步"**（§4）：选轨迹与按开始键都留给人。

### 3.2 手动分终端（等价，逐条起）

★ **命令的唯一出处是 `run/z0.py` 的 `ACTIONS`** —— 下面这几条就是它，菜单号也照它编。
在**每个**终端里都可以不手打 `source`（菜单自己带前缀）；手工敲时三条 `source` 是：

```bash
source /opt/ros/humble/setup.bash && source ~/mavros_ws/install/setup.bash && source ~/ros_ws/install/setup.bash
ros2 launch mavros px4.launch fcu_url:=/dev/ttyTHS1:921600 gcs_url:=udp-b://0.0.0.0:14555@:14550
ros2 launch vrpn_listener sync_entity_state.launch
ros2 launch mocap mocap.launch.py
ros2 launch traj traj.launch.py
ros2 launch control control.launch.py
```

| 菜单 | 起什么 | 它的坑（脚本会提醒你） |
|---|---|---|
| 菜单 **1= 环境自检（只读）** | 三条 setup / 六个包 / 串口 / 域号 / commit | 报"六个包全查不到"多半是**装好了但没 source** |
| 菜单 **2= 起链核对（只读，约 5 s）** | 每个话题有没有帧、多少 Hz | 5 s 一张表；见 §5.1 |
| 菜单 **3= 起 mavros** | 上面第 1 条 | ★ **独占** `/dev/ttyTHS1`；`/mavros/state` **1 Hz 是正常的** |
| 菜单 **4= 起 vrpn_listener** | 上面第 2 条 | 刚体名要与 `mocap.yaml` 对齐（`Z0`/`z0` 两种都出现过） |
| 菜单 **5= 起 mocap** | 上面第 3 条 | ★ 起之前要确认**飞机已摆好、静止**（阶段 A 原点此刻锁定） |
| 菜单 **6= 起 traj（单独）** | 上面第 4 条 | 只发参考、不碰飞控；一帧不发直到被 select + start |
| 菜单 **7= 起 control（单独）** | 上面第 5 条 | 台架 T3 用的就是这条；`MANUAL/INIT` 是正常起始态 |
| 菜单 **8= 起 bringup 一键（mocap+traj+control）** | 一条命令起三个（mavros **默认不起**） | 同样要确认飞机摆好；会问一句"已单独起过 vrpn 了吗" |
| 菜单 **9= 操作台 fly.py** | 选轨迹 + 看状态 | 见 §4 |
| 菜单 **10= 一键起链（mavros+GCS 桥 → vrpn → mocap → traj+control，顺序起）** | = §3.1 | 逐段等判据；`run/up.sh` 是它的一行别名 |

### 3.3 ★ 顺序不能反

`bench-runbook.md` 那一节的标题就是这句话。三条理由，都是具体的：

1. **飞机必须在起 `mocap` 之前摆好并静止** —— 阶段 A 原点在节点启动时锁定（30 帧均值），
   到解锁上升沿才切到阶段 B；**切换量 = 这段时间里动的量**（实测撞过 `0.5738 m`，
   那同量的位置跳变会被灌给 EKF2 当成真实运动）。
2. **OFFBOARD 要先于解锁** —— 见 §2.3。
3. **`traj` 与 `control` 谁先起都行**（`control` 起早了停在 `INIT` 并列出缺什么、`traj` 一帧不发），
   但**都要在起飞之前** —— 参考时钟的零点由 `control` 进 `TRACK` 那一刻的 `/traj/start` 定。

### 3.4 这条链里**没有**什么（安全边界）

**没有任何"让飞机动"的入口**：解锁 / 起飞 / 切模式 / 改参数 / 发帧，一个都没有。
理由是仓库里写死的设计：**"开始键之前不发控制指令"**（[`z0.launch.py`](../../src/bringup/launch/z0.launch.py)
头注释逐字；同一句也写在 [`fly.py`](../../src/bringup/scripts/fly.py) 与
[`bringup/README.md`](../../src/bringup/README.md) 里）⟹ 给一个程序起飞入口等于在墙上开一扇后门。
台架上那条 `ros2 service call /control/takeoff …` **照旧手打**（**不是**给现场的入口：
台上没有遥控接收机，"RC 在线"这条起飞前置本身就过不了），原文在
[`bench-runbook.md`](bench-runbook.md) **T4**。

---

## 4. 起完还差两步（**刻意不自动化**）

这就是菜单 **9= 操作台 fly.py** 那一项，等价于手打：

```bash
ros2 run bringup fly.py     # 选轨迹 + 实时看状态与完成度
```

```
命令：1 = 原地悬停   2 = 1 m 正方形   3 = 0.5 m 八字   0 = 取消选择
      h = 帮助        q = 退出          回车 = 立刻打印一次当前状态
```

1. **选轨迹**（`/traj/select`）：`1` = 原地悬停（恒为原点，`hover_duration_s` 默认 **60 s**）/
   `2` = 1 m 正方形（周长 4 m @0.5 m/s ⟹ **8.0 s**，**起点在原点**、进 `TRACK` 不跳变）/
   `3` = 0.5 m 八字（Gerono 双纽线，周长 ≈3.05 m ⟹ **≈6.1 s**）。
   ⚠️ **起飞前选好**；运行中重选也支持（`control` 会保持上一个目标并按闸门重发 `/traj/start`），
   但那是有意的例外，不是常规流程。
2. **遥控器开始键**：第 1 次上升沿 = 解锁 + 起飞，第 2 次 = 进 `TRACK`（§1.1）。

★ **为什么不自动化**：操作台**不发** `/traj/start`（参考时钟的零点必须与"开始飞"同一时刻，
谁驱动飞机只能有一个答案）、**不调解锁**（那条防线在上面）。两条入口在 `control` 里走的是
**同一个受理路径**，判据逐条相同。

---

## 5. 每一步怎么判读

### 5.1 起链核对（菜单 **2= 起链核对（只读，约 5 s）**）

| 话题 | 正常读数 | 不正常先看 |
|---|---|---|
| `/mavros/state` | **1 Hz**、`connected=True`、`armed=False`、`mode=POSCTL`（ch6 在第 1 档时；已拨到第 6 档就是 `OFFBOARD`） | ★ **有帧 ≠ 连上飞控** —— 它报的是连接心跳 |
| `/mavros/local_position/pose` | ~30 Hz（EKF2 位置，控制律的位置来源） | 没有 ⟹ 动捕没进 EKF2（那是**视觉链路**的事：[`mocap-setup.md`](../reference/mocap-setup.md) + [`bench-runbook.md`](bench-runbook.md) 的 **T1** 动捕链路目视 / **T2** 坐标轴确认） |
| `/mavros/vision_pose/pose` | 有帧、不跳 | 跳 ⟹ 动捕 WiFi / `EKF2_EVP_NOISE` |
| `/mocap/pose_enu` | 有帧（动捕实测 ~200 Hz） | `·` ⟹ `vrpn_listener` 或 `mocap` 没起、或 QoS 不兼容 |
| `/control/state` | `MANUAL/INIT`（刚起）、`MANUAL/STANDBY`（守卫齐） | 一直 `INIT` ⟹ 诊断日志里逐条写着缺什么 |
| `/traj/reference` | ★ **没选轨迹时本该是空的**（记 `·`，不算错） | 选了还是没有 ⟹ `traj` 没收到 `select`/`start` |
| `/mavros/setpoint_raw/local` | ≈50 Hz ⟹ 我们正在流 | 一帧都没有 ⟹ 守卫不成立或 `enter_offboard=false` |

★ 表里出现 **`ROS_DOMAIN_ID=73`** 时要当心：那是**台架夹具域**⟹ 读数不是真飞机（首行会标出来）。
★ 手工排查时 `ros2 topic list` 只回 `/rosout` 这种"什么都没起"的假象，多半是 **`ros2` daemon 的坑**
（同域 daemon 被先前那个环境拉起来过）—— 加 `--no-daemon`：`ros2 topic list --no-daemon --spin-time 3`。

### 5.2 ★ 第一次上电必做：ch9 通道核对（§H.8 / 验收项 6）

```bash
ros2 topic echo /mavros/rc/in     # 看 channels[8]
```

- 拨 ch9：`channels[8]` 必须**由低到高**（且低位/高位读数落在 `1000–2000` 附近 —— 若不在，
  要改的是 `start_low_us` / `start_high_us`，**不是**阈值）。
- 顺手拨 ch6：确认**模式切换只认 ch6**。
- ⚠️ **要在解锁之前做**：通道错了的话，第 1 次上升沿会直接去解锁起飞。
- ⚠️ **要试 ch8 杀车必须先拆桨**。
- 不符就**回 ticket 02 / §H 改裁定**（原文要求），不要就地改代码。

### 5.3 日志 token（都是固定形状，可 grep）

⚠️ **原文里的 `**` 是会被打印出来的**（代码里的日志字符串自带 markdown 加粗，`RCLCPP` 原样输出）
⟹ 想 `grep` 到就得**带上星号**，或者只 grep 那句里最稳的一小段（比如 `上升沿`、
`end_reason=`、`traj_origin=`）。下表写的是**原文**。

| token（原文） | 含义 |
|---|---|
| `takeoff_origin=(x, y, z)` / `traj_origin=(x, y, z)` | 起飞原点 / 轨迹原点（与动捕对账就看后者） |
| `MAV_RESULT=<n>` | 解锁应答码（`0=ACCEPTED`；其余一律"被拒"，不重试） |
| `遥控器开始键：**第 1 次上升沿**` / `遥控器开始键：**第 2 次上升沿**` | 确认过的边沿及其去向 |
| `遥控器开始键：上升沿**被丢掉**了` | ★ "按了没反应"最该看的一条（含当前任务态） |
| `TRACK：**轨迹参考已经来了**` / `TRACK：**轨迹参考断了**` | 参考流的第一帧 / 真的断了（离开 `TRACK` **不打**） |
| `TRACK 进行中：` | 节流心跳：参考年龄 + 转发出去的完成度 |
| `**任务正常走完**（end_reason=traj_done / wait_start_timeout）` | **正常收口**；是不是"降级"要看有没有 `degrade=` |
| `degrade=rc_override（持续 N s，…）—— …`（**行首**，ERROR） | 真降级事件的形状；token 的取值见 §2.4（★ 别与"参考断了"那条 WARN 里句子中间的 `degrade=` 混了） |
| `degrade=none：降级已复位` | 复位时刻（可 grep `grep -oE "degrade=[a-z0-9_]+"` 再滤掉 `none`） |
| 服务应答 `accepted` / `already_flying` | 在 `/control/takeoff` 应答 `message` 的**开头**（现场走开始键，一般看不到） |

---

## 6. 出事了先看哪

| 现象 | 先看 |
|---|---|
| **按了开始键没反应** | ① 压住够 0.3 s 吗；② 日志里有没有那两条（**"被丢掉"** ⟹ 任务态不对；**"第 1 次上升沿"** ⟹ 受理了但没起飞，去看 §2.3 的前置；**两条都没有** ⟹ 边沿根本没产生）；③ 拨一下确认是 **ch9**（§5.2） |
| **按了第 1 次，飞机不解锁** | 诊断日志的缺项：**ch6 在 Offboard 档吗**、遥控在线吗、动捕新鲜吗；`takeoff_timeout_s = 5 s` 一到命令就作废（**不自动重试**，要再按一次） |
| `control` 一直 `MANUAL/INIT` | 它的日志逐条列出缺什么：`/mavros/state` 静默 / 动捕没来 / `local_position` 没动 |
| **没有任何参考流** | `traj` 是不是**没 select**；select 了但没 `start`（`control` 进 `TRACK` 才发） |
| 飞机**位置整体偏一截** | 起 `mocap` 之前飞机就在动（阶段 A 原点漂了）⟹ 重起 `mocap`（摆好、静止之后） |
| 飞机**自己在降落** | 日志有 `degrade=` 或 `end_reason=`：前者看是哪条输入坏了，后者是正常收口（轨迹飞完 / `WAIT_START` 等超时 300 s） |
| 飞机**不听我们的了** | `MANUAL/RC_OVERRIDE` ⟹ 有人打杆或模式被切走（§2.4 的夺权）；看是谁、不要"重试一下" |
| **想立刻停任务** | §1.2 的两条软路径；☠️ ch8 是停桨，不是停任务 |
| **想终止整条链** | 在 `run/up.sh` 那个终端里 `Ctrl-C`（**别 `kill -9`**，会留下 mavros 占串口） |
| 遥控器链路断了 | 只剩 offboard 断流保护（`COM_OF_LOSS_T=1.0 s` → `COM_OBL_RC_ACT`）；⚠️ A1 未改之前会**试图返航** |

★ **排障表更全的一版**在 [`run/README.md`](../../run/README.md)（daemon 坑 / 半拉子环境 / 刚体名大小写 /
串口占用 / "在图里但没帧"的两种成因）。

---

## 7. 收尾

1. 飞机**落地上锁**之后 `/control/state` 自己回到 `MANUAL/STANDBY`（原点、轨迹原点、任务态全清）。
2. **在 `run/up.sh` 的终端里 `Ctrl-C`** —— 子进程成组（`start_new_session`），先 SIGINT 超时才 SIGKILL
   （前台那个 `Ctrl-C` 到不了子进程，必须由脚本转发）。
3. 若这一趟动了参数（§P0 那几条），**按 `bench-runbook.md` T8 恢复并回填读数**。
4. 若飞控在录 ulog：**把它存档** —— 判据全在 PX4 侧，它才是硬证据（mavros 只是观察窗）。

---

## 8. ⏳ 还没做的（诚实交代，别把这张表当成"已经飞过"）

| 项 | 为什么还没做 |
|---|---|
| ★ **验收阶梯 1–4 级**（无桨台架 / 系绳离地 20 cm / 系绳悬停 30 s / 自由飞） | 每一步都是人的物理动作（拆桨、系绳、扶飞机） |
| **EdgeTX 里给 ch9 分配开关**（§H.4） | 人工动作，是 §5.2 核对的前置 |
| **§5.2 的通道核对**（验收项 6） | 被上一条与遥控链路状态阻塞 ⟹ ticket 09 的勾选框**留空** |
| **起链那几个动作没在真飞控上跑过**（菜单 3–8、10） | 要占串口、要人在场摆飞机（ticket 22 §五） |
| **A1**（`NAV_RCL_ACT` `2 → 1`）等 4 条"已裁定、待写入"的参数 | 见 [`params-changes.md`](../reference/params-changes.md) §I.1 |
| 真 PX4 的 `MAV_RESULT` 码、`COM_DISARM_LAND` 真实上锁、1 Hz 状态延迟 | 夹具不模拟 PX4 ⟹ 只有真飞控能给（`control/README.md` 末节） |
