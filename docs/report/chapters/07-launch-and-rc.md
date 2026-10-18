# 第 7 章 启动脚本与遥控等交互（含安全/FSM）

本章是"现场那一页"的报告版：从摆好飞机、起链、选轨迹、拨开关、按开始键，一直写到降落、录制与出事后的处置。它回答的不是"代码怎么写"，而是"这一趟飞下来，人在什么时候做什么、机器在什么时候接手、每一步凭什么判断它可以往下走"。

第 5 章给出了层与层之间的接口和话题，第 6 章按包拆了代码结构；本章不重复它们，只把两条控制链的操作顺序、就绪判据、遥控通道、状态机与失效路径串起来。MPC 的公式见第 8 章，实验设计与数据处理见第 9 章，结果与未验证项见第 10 章。本章的命令与菜单号以 [`run/README.md`](../../../run/README.md) 为唯一出处，现场动作以 [`operation_manual.md`](../../../operation_manual.md) 为准，通道裁定回到 [`params-changes.md` §H](../../reference/params-changes.md)，安全结论回到 [`docs/assets/drill/`](../../assets/drill/) 的第一手日志。

---

## 7.1 术语与通道编号

本章只用几个固定词，先把它们钉死，后面不再解释。**控制权**回答"谁在驱动飞机"，只有 `MANUAL` 与 `AUTO` 两个值；**任务态**回答"这一趟走到哪一步"，有八个值，逐项见 7.7 节的表 7-7。两者是正交的，`/control/state` 把二者拼成一行，例如 `MANUAL/STANDBY` 或 `AUTO/TRACK`。**开始键**指遥控器上的 ch9，**杀车**指 ch8，**软停**指不切断动力的三种停任务方式。回退 (A) 级与 (C) 级沿用项目里的分档：(A) 是 offboard 位置控制，(C) 是 MPC 下发角速率与推力，角速率环与混控仍由 [PX4](https://github.com/PX4/PX4-Autopilot) 做。

**表 7-1　本章术语。**

| 词 | 含义 | 在哪读 |
|---|---|---|
| 控制权 | `MANUAL` / `AUTO`：飞机是否正被我们发的指令驱动 | `/control/state` 斜杠前 |
| 任务态 | `INIT`…`RC_OVERRIDE` 八个值：任务走到哪一步 | `/control/state` 斜杠后 |
| 开始键 | ch9；两次上升沿分别对应起飞与开始轨迹，长按对应自动降落 | `/mavros/rc/in` 的 `channels[8]` |
| 杀车 | ch8；高值让 PX4 立即停桨 | PX4 的 `manual_lockdown` |
| 软停 | 长按 ch9、打杆、拨 ch6 三条不切断动力的停任务路径 | 本章 7.8 节 |
| 回退 (A) 级 | 放弃 MPC，回到 offboard 位置链 | `run/z0.py up` 第 1 档 |
| 通道编号 | 遥控器侧的 ch 编号，从 1 起 | `RC_MAP_*` 参数与 `RC_CHANNELS` |

`ch9` 在这个仓库里一律指遥控器通道号，不是数组下标；代码读的是 `/mavros/rc/in` 的 `channels[8]`，两者差 1，现场核对时不要混。

<big>**一句话总结：控制权与任务态分开读，通道号从 1 起。**</big>

---

## 7.2 一键起链：两条链与一个问句

[`run/z0.py`](../../../run/z0.py) 是一张 11 项的交互菜单，本身不含任何飞行逻辑。它把命令送出去、把读数摊开，判据与记录仍在 [`bench-runbook.md`](../../control/bench-runbook.md) 里。现场最常用的是其中四项：第 8 项"一键启动（动捕+mavros，手动遥控飞）"、第 9 项"一键启动全部（起前选控制器；默认 offboard 控制）"、第 10 项"清除后台节点"、第 11 项"开始录制（选场景 1–6）"。其余各项分别做环境自检、起链核对，或单独起某一个节点。

**表 7-2　一键起链的入口。**

| 入口 | 等价于 | 起什么 | 起完之后谁在上位 |
|---|---|---|---|
| [`run/up.sh`](../../../run/up.sh) | `run/z0.py up`，菜单第 9 项 | 公共前缀 + 所选尾巴 | `control`，或 `mpc_node`（见下） |
| `run/z0.py up_manual` | 菜单第 8 项 | mavros → vrpn_listener → mocap | 没有人；飞机由遥控器手动飞 |
| [`run/down.sh`](../../../run/down.sh) | `run/z0.py down`，菜单第 10 项 | 按名字停进程 | —— |
| `run/z0.py check` | 菜单第 2 项 | 只读核对表 | —— |

两条一键链的公共前缀逐字相同：[mavros](https://github.com/mavlink/mavros)（可选带 GCS 桥给 [QGC](https://qgroundcontrol.com/)）→ vrpn_listener → mocap，每一步都等判据成立再走下一步。差别只在起链前选出的"尾巴"。

第一种是手动档。`up_manual` 只起前缀三步，不起 `traj`，也不起 `control`。这意味着整条链里没有任何程序在发布 setpoint；起完之后，操作者直接用遥控器手动起飞。它适合检查动捕与飞控链路，或者单纯把飞机飞起来。

第二种是自动档。菜单第 9 项会先问一句"这一趟谁在上位控制飞机"，输入数字选尾巴；直接回车等于第 1 档。两档的尾巴都包含 `traj`，这是操作者 2026-10-16 定下的结构事实——"轨迹必须起没得选"——不是提示语。第 1 档是 `traj` → `control`，`control` 自己发 PX4 setpoint，`output_mode` 保持默认的 `position`；第 2 档是 `traj` → `mpc` → `control`，`control` 以 `output_mode:=external` 起，`mpc_node` 带 `reference_source:=control` 与 `offboard_prestream:=true`。两档的现场动作逐字相同：选轨迹、ch6 拨到 Offboard、开始键第 1 次上升沿、判稳后第 2 次上升沿并松手、飞完自动降落。区别只在"最后给 PX4 发指令的人"是谁。

所以"谁在上位控制飞机"问的不是"谁在管任务"，而是"谁的指令最后落到 PX4 的执行器 setpoint 上"。两档里 `control` 始终是任务总管：它管开始键、解锁、切 OFFBOARD、起飞、判稳、记录轨迹原点、发 `/traj/start` 与降落。第 2 档只是把最后一段换成 `mpc_node` 求解后写姿态 setpoint。手动档则连任务总管都不起，飞机完全由人飞。

[`run/down.sh`](../../../run/down.sh) 是这条链的另一头。它按名字扫描 `DOWN_TARGETS` 白名单——mavros、vrpn_listener、mocap、traj、control、mpc_node 与 bringup 壳——先把 pid 与完整命令行摊开，问一句 y/N，再按 SIGINT → 等 5 s → SIGTERM → 等 3 s → SIGKILL 的阶梯停。它不区分这些进程是谁起的：别的会话、跑着的测试，只要名字对上都会进清单。**停进程不是降落**：setpoint 一停，PX4 在 `COM_OF_LOSS_T` 之后自己退出 OFFBOARD，按 `COM_OBL_RC_ACT` 落回 Position；飞机若还在空中，要先用 7.8 节的软停路径处置。

<big>**一句话总结：两条链只差最后发指令的人：遥控器或程序。**</big>

---

## 7.3 启动顺序与判读

自动档的起链顺序是固定的：mavros → vrpn_listener → mocap → traj →（第 2 档才有的 mpc）→ control。顺序不能反，原因分散在每一步的判据里：后面的节点消费前面的话题，前面没起来，后面即使进程在跑也只是空转。

**表 7-3　启动顺序与就绪判据。**

| 步 | 起什么 | 等什么判据 | 上限 | 现场提示 |
|---|---|---|---|---|
| 1 | mavros | `/mavros/state` 的 `connected=True` | 60 s | ★ 有帧 ≠ 连上飞控 |
| 2 | vrpn_listener | `/vrpn/<刚体>/pose` 有帧 | 30 s | `Z0` / `z0` 两种刚体名都认 |
| 3 | mocap | `/mocap/pose_enu` 有帧 | 30 s | 起之前确认"飞机已摆好、静止、固定？" |
| 4 | traj | 不等 | —— | 没被选就一帧不发，等它等于等到天荒地老 |
| 5 | mpc（第 2 档） | 它自己的启动日志行 | 120 s | 闸门关着时话题上看不出"我起来了" |
| 5′ | control（两档都有） | `/control/state` 有帧 | 30 s | 起始态是 `MANUAL/INIT` |
| 6 | 终端让给 `fly.py` | 操作者选 1/2/3/4 | —— | 起飞前必须选好轨迹 |
| 7 | 前台守着 | —— | —— | 打印核对表与起飞提示，Ctrl-C 一起停 |

第一步的判据值得单独说。`/mavros/state` 有帧只说明 mavros 在发布，而 `connected` 是消息里的一个字段，由 mavros 是否收到飞控心跳决定。看"有没有帧"会把"串口开着但飞控没回话"读成已连接；菜单里那句"有帧 ≠ 连上飞控"就是冲这个来的。第 1 步起之前还会查 `/dev/ttyTHS1` 被谁占着，因为第二个 mavros 打开串口失败的报错长得很像 launch 坏了。

第三步的确认语不是客套。`mocap` 节点在启动那一刻锁定阶段 A 的原点，飞机没摆好、还在动，原点就记歪了；后面所有相对位姿都会带这个偏差。这一步起之前必须让飞机静止、固定。

第四步的 `traj` 刻意不等。它只发参考、不碰飞控，没收到 `/traj/select` 就一帧不发；用"有没有帧"当就绪判据会永远等不到。真正的选轨迹发生在第 6 步，终端交给 `fly.py` 之后，而且必须在起飞前选好。

第 2 档的 `mpc_node` 也不能用话题判断。闸门关着的时候它不发执行器指令，只有 `offboard_prestream` 的纯悬停帧，话题面上没有"我起来了"这个事实。所以判据取它自己的启动日志行 `MPC（模型预测控制）节点启动：`，上限 120 s；等不到就打日志尾巴、把已经起的全部停掉并返回 1。这条路径还带一个环境前提：命令自带 `ACADOS_SOURCE_DIR` 与 `LD_LIBRARY_PATH`，缺一个就会在建求解器时报 `OSError: libacados.so / libhpipm.so`，2026-10-17 的起链实测就死在第 5 步。

第六步之后，`fly.py` 在同一个终端里选轨迹，选完退出；`z0.py` 打印核对表与起飞提示，然后前台守着。菜单第 2 项 `run/z0.py check` 随时可以另开一个终端跑，它把每个话题有没有帧、多少 Hz，`/mavros/state` 的 `connected/armed/mode`，以及 `/control/state` 的原文摊成一张表。

<big>**一句话总结：每步等判据成立再走；traj 不等。**</big>

---

## 7.4 输出路径：同一套任务、两个最后执行者

两档的现场动作相同，区别只在最后一跳。表 7-4 把两条输出路径并排放在一起。

**表 7-4　两条输出路径。**

| 尾巴 | 谁写 | 话题 | 消息内容 | 之后由谁算控制量 |
|---|---|---|---|---|
| 第 1 档 offboard | `control` | `/mavros/setpoint_raw/local` | 绝对 ENU 位置 + yaw，`type_mask=2552` | PX4 的位置环 → 姿态环 → 角速率环 → 混控 |
| 第 2 档 MPC | `control` | `/control/target` | 绝对 ENU 位置 + 速度 + yaw，`type_mask=2496` | 不直接到飞控；`mpc_node` 拿它当参考 |
| 第 2 档 MPC | `mpc_node` | `/mavros/setpoint_raw/attitude` | 机体三轴角速率（FLU）+ 归一化总推力，`type_mask=128` | PX4 的角速率环 → 混控 |

第 1 档里，`control` 从 `STANDBY` 起就以 50 Hz 发 `/mavros/setpoint_raw/local`，切 OFFBOARD 之前已经预流满 1 s。预流的作用不是那些帧本身——切模式前 PX4 会把它们丢掉——而是每条 setpoint 触发的 offboard 心跳，它满足 PX4 的"切模式前 setpoint 已在流"前置。切进 OFFBOARD 之后，同样的帧才真正驱动飞机。

第 2 档里，`control` 一帧都不写 `/mavros/setpoint_raw/local`。它只写内部目标 `/control/target`，`mpc_node` 订阅这个目标与原始动捕，解出总推力与机体角速率，再写 `/mavros/setpoint_raw/attitude`。`type_mask=128` 表示忽略姿态四元数，只取角速率与推力。mavros 在边界上把角速率从 FLU 转到 FRD，推力不转；PX4 只保留角速率环、混控与安全机制。这是 (C) 级的定义：我们下到角速率，不下到电机。

两条路在结构上互斥，不靠现场纪律：external 档下 `control` 不创建 local 发布者，offboard 档下不启动 `mpc_node`。因此不存在两路 setpoint 互相覆盖的窗口。

第 2 档还有一道闸门。`/control/target` 断流 0.5 s，`mpc_node` 就以 `no_control_target` 关闸并**停流**——不发布，既不冻结在最后一个目标，也不发零帧。再往后是 PX4 自己的保护：`COM_OF_LOSS_T=1.0 s` 内没有 setpoint，退出 OFFBOARD，按 `COM_OBL_RC_ACT=0` 回 Position。求解器不可用时另有 hold → hover 的兜底阶梯，但那不是这条断流闸门的一部分；最后一道始终是 PX4 的超时。

`offboard_prestream` 只负责在 `control` 请求 OFFBOARD 之前让 PX4 看见姿态流。它不 arm、不 set_mode；这两件事仍然只在 `control` 手里。

<big>**一句话总结：两条尾巴最后写的 setpoint 话题不同。**</big>

---

## 7.5 录制：六条场景与一次收口

录制由菜单第 11 项启动。它选场景 1–6，起 `ros2 bag record`，并**占住终端**：屏幕上走倒计时，录满 120 s（场景 1–5）自己收口；想提前停就按 Ctrl-C，这与倒计时到点走的是同一条 SIGINT 收口路径。

**表 7-5　六条录制场景。**

| 场景 | 名字 | 时长 | 用途 |
|---|---|---|---|
| 1 | 静止 | 120 s | 噪声基线 |
| 2 | 手飞起飞 | 120 s | 起飞瞬态 |
| 3 | 轨迹 1（悬停） | 120 s | 悬停判据 |
| 4 | 轨迹 2（正方形） | 120 s | 有机动的激励 |
| 5 | 轨迹 3（8 字） | 120 s | 另一类机动 |
| 6 | 任意录制 | 不到点 | 自由补录，只有 Ctrl-C 收口 |

录制的话题走白名单正则 `^/(mavros|mocap|traj|control|vrpn)/`，不是 `-a`；压缩用 zstd，在收口那一刻做。收口的动作是固定的：SIGINT 之后等它写缓存、压包、关库，最多等 45 s，等不到才升级到 SIGTERM / SIGKILL；然后用 `ros2 bag info` 的实测帧数逐条回对预期，写 `session.json`（时长、大小、话题、频率、sha256、缺了什么、为什么收的），再重算 [`bag/MANIFEST.md`](../../../bag/MANIFEST.md)。

这一层只订阅，不解锁、不切模式、不发帧。这条边界有机器守卫：[`test_rec.py`](../../../run/tests/test_rec.py) 用 AST 扫录制层的代码，禁止出现 `create_publisher` / `create_service` / `create_client`，也禁止那四个与"让飞机动"有关的动作。它和 [`run/z0.py`](../../../run/z0.py) 的其余部分一样，不给程序起飞入口。

现场没有"停止录制"这一项。菜单第 11 项一启动就占住终端，想提前停只能按 Ctrl-C；要看进度，另开一个终端跑 `run/z0.py rec_status`。手工拷完飞控日志（`.ulg`）之后，可以跑 `run/z0.py rec_manifest` 重算总表。飞控日志本身不由这一层拷：拔 SD 卡，把 `<卡根>/log/sessNNN/logMMM.ulg` 手工放进 `bag/<场景>/fcu/`。

录制也不是降落。飞机还在空中时按 Ctrl-C 只结束录制进程，不改变飞机状态；要停任务仍要走 7.8 节的软停路径。

<big>**一句话总结：录制只订阅、不解锁、不发帧。**</big>

---

## 7.6 遥控通道：四个开关，四条职责

遥控器的通道分配是 Phase 1 的输入，不是随手定的。`z0.params` 里非零的 `RC_MAP_*` 只有七条：四条摇杆加三个开关。其余的 `RC_MAP_*` 全为 0，ch9 不在占用表里，所以拨它不会切模式、不会解锁、不会杀车。

**表 7-6　遥控通道。**

| 通道 | 功能 | 参数 | 谁读 | 现场怎么用 |
|---|---|---|---|---|
| ch1–ch4 | 横滚 / 俯仰 / 油门 / 偏航 | `RC_MAP_ROLL/PITCH/THROTTLE/YAW = 1/2/3/4` | PX4；`control` 也读，用于打杆判据 | 飞行中别碰杆，判据看的是变化率 |
| ch5 | 解锁开关（冗余硬件路径） | `RC_MAP_ARM_SW=5` | PX4 | 现场不要用；程序解锁才是主路径 |
| ch6 | 模式开关 | `RC_MAP_FLTMODE=6` | PX4 | 第 6 档 = Offboard，整段任务停在它上面 |
| ch8 | 杀车开关 | `RC_MAP_KILL_SW=8`；`RC_KILLSWITCH_TH=0.75` | PX4 | 高值 = 立即停桨，不是"取消任务" |
| ch9 | 开始键 | `start_channel=9`，读 `channels[8]` | `control` | 两次上升沿 + 一次长按 |

ch6 的三个档位要按实测说。快照里 `COM_FLTMODE1=2`、`COM_FLTMODE4=8`、`COM_FLTMODE6=7`；2026-10-12 的通道读数里，ch6 低档 999 报 `POSCTL`（Position）、中档 1503 报 `STABILIZED`、高档 2000 报 `OFFBOARD`。现场只用第 6 档，也就是 Offboard；第 2、3、5 档未分配，拨过去没有反应。[`operation_manual.md`](../../../operation_manual.md) §1 的括号里仍写着 `COM_FLTMODE1/4/6 = 2/1/7`，其中第 4 位与快照的 `8`、实测的 `STABILIZED` 不符——这是文档里的旧值，现场动作（第 6 档）不受影响，但读参数时要按快照与实测。

ch9 的判据分三层。第一层是归一化阈值：低位 `< 0.25`、高位 `> 0.75`，去抖 200 ms。第二层是采样：`/mavros/rc/in` 是 10 Hz，200 ms 去抖等于连续 3 帧，所以每次压住至少 0.3 s 再松，压得太贴边会有一半概率按不动，而且连"被丢掉"的日志都没有。第三层是动作归属：上电即在高位不算边沿，必须先见到低位；第 1 次上升沿是解锁加起飞；第 2 次按下只"记下"，**松手**那一刻才进 `TRACK`；在飞时长按 ≥1 s 是立即自动降落，判据从电平确认起算，物理上要按住约 1.2 s，而且资格看**按下那一刻**——起飞那一下按完一直按着不放，不会让飞机刚离地就降落。

ch5 保留在参数表里，是冗余的硬件解锁路径，但现场不用它。理由有两条第一手事实：在 OFFBOARD 里拨 ch5，PX4 会以 `Arming denied: switch to manual mode first` 拒绝；即使在别的模式解开，`armed` 的变化也不是程序发起的，`control` 会停发 setpoint、告警并退回 `MANUAL/INIT`，把飞机留在飞行员手里。程序解锁与开始键那条路才是"已解锁且仍在发 setpoint"的入口。

ch8 是最后手段。它的语义链是 `RC_MAP_KILL_SW=8` → `RC_KILLSWITCH_TH=0.75` → 高值 = `ACTION_KILL` → `manual_lockdown=true`，也就是立即停桨；拨它不等于取消任务，空中拨它等于让飞机失去动力掉下来。程序侧根本不读这条通道，所以拨 ch8 不会在我们的状态机里留下任何状态变化。

`COM_RC_IN_MODE` 是这层开关的总闸，现值 `0`。它曾经是 `3`，语义是"先到的源一直用到重启"；QGC 一上来就发 `MANUAL_CONTROL`，飞控会把 MAVLink 源锁成首选源，而 `ManualControl` 只在选中源是 `SOURCE_RC` 时才评估开关。后果是 ch5、ch6、ch8 与内八手势**整层静默失效**，现场症状是"拨 ch6 模式不变"，连一条拒绝的 STATUSTEXT 都没有。2026-09-23 改成 `0`（只收 RC）后当场复现：ch6 拨档，模式跟着走。这个值不要改回 `3`。

<big>**一句话总结：四个开关职责唯一，互不复用。**</big>

---

## 7.7 状态机：两个正交维度

状态机把"谁在控制"与"在干什么"拆成两个维度，而不是一个扁平枚举。控制权是 `MANUAL` / `AUTO`，任务态是 `INIT`、`STANDBY`、`TAKEOFF`、`HOVER`、`WAIT_START`、`TRACK`、`LAND`、`RC_OVERRIDE`。`/control/state` 以 `"<控制权>/<任务态>"` 发布，例如 `MANUAL/STANDBY`、`AUTO/TAKEOFF`、`MANUAL/RC_OVERRIDE`。

**表 7-7　两个维度。**

| 维度 | 取值 | 回答的问题 |
|---|---|---|
| 控制权 | `MANUAL` / `AUTO` | 飞机是否正被我们发的指令驱动 |
| 任务态 | `INIT` / `STANDBY` / `TAKEOFF` / `HOVER` / `WAIT_START` / `TRACK` / `LAND` / `RC_OVERRIDE` | 任务走到哪一步 |

`AUTO` 的定义不是"我们连上了飞控"，而是"我们正在发真正驱动飞机的指令"——已解锁、在 OFFBOARD、setpoint 在流。已进入 OFFBOARD 但还没解锁时仍是 `MANUAL`：那时的 setpoint 不驱动飞机，飞机还在人手里。反过来，夺权时控制权与任务态会同时置位：打杆或模式被切走，`Authority` 变 `MANUAL`，`Phase` 变 `RC_OVERRIDE`，于是读到 `MANUAL/RC_OVERRIDE`。

正常路径如下。第 1 步，`INIT` 等到 `state.connected`、动捕有数据、`local_position` 有效，进 `STANDBY`；`STANDBY` 已经开始以 50 Hz 发 setpoint，目标就是当前位置与当前机头，但飞机未解锁。第 2 步，收到开始键第 1 次上升沿，或台架上的 `/control/takeoff` 服务调用，`control` 发出解锁请求；**受理不等于起飞**，对外仍是 `STANDBY`。第 3 步，`state.armed` 出现上升沿，就在这一帧锁定起飞原点与机头，进 `AUTO/TAKEOFF`，目标高度从原点沿 0.50 m/s 的斜坡升到 +1.00 m，水平两轴钉在原点。第 4 步，飞机当前位置与"原点 + 1.00 m"的差小于 0.10 m、垂直速度小于 0.10 m/s 并持续 1.0 s，进 `HOVER`；悬停判稳用的是更严的一档，位置与速度都小于 0.10 并持续 2.0 s。第 5 步，判稳后记录轨迹原点，进 `AUTO/WAIT_START`，目标就是轨迹原点；这里等开始键第 2 次上升沿，兜底超时 300 s，超时按正常任务结束请求降落，`end_reason=wait_start_timeout`。第 6 步，第 2 次上升沿按下只记下，**松手**才进 `AUTO/TRACK`，`control` 这时自己发 `/traj/start`，目标变成轨迹原点加 `/traj/reference`。第 7 步，`/traj/progress` 首次达到 1.0（必须先见过小于 1.0），进 `MANUAL/LAND`：停发 setpoint，请求 `AUTO.LAND`，`end_reason=traj_done`；PX4 在 `COM_DISARM_LAND=2.0 s` 后自己上锁，状态回到 `MANUAL/STANDBY`。

起飞这一步有三条前置，缺一条解锁就会被拒：已经进入 OFFBOARD、setpoint 已预流满 1 s、遥控在线且位置守卫成立。开始键第 1 次上升沿发出的起飞命令有 5 s 的有效期（`takeoff_timeout_s`），过期没等到 `armed` 就作废，要再按一次。在飞的四个任务态里，长按开始键 ≥1 s 都直接进 `LAND`，`end_reason=land_pressed`，这是唯一一条"仍由程序自主降落、不交还控制权"的停法。

完整的状态转移表不在这里重抄。z0 的转移条件以 [`phase1-implementation.md` §4.2](../../control/phase1-implementation.md) 为准；参考项目 XI35 的五个状态与十一条转换在 [`xi35-fsm.md`](../../reference/xi35-fsm.md)。两者的关键差别在接管语义：XI35 把摇杆位移积分进悬停点，z0 是立即停止发布，让 PX4 按 `COM_OBL_RC_ACT` 接管。

<big>**一句话总结：控制权与任务态正交，可自由组合。**</big>

---

## 7.8 失效路径与复位

失效先分成两类，处置刻意不同。**控制权被夺走**包括打杆接管 `rc_override` 与模式被外部切走 `mode_lost`；处置是立即停发 setpoint、不请求任何模式、交还控制权——既不抢回 OFFBOARD，也不请求 `AUTO.LAND`，因为那是在跟正在接管飞机的人抢。**输入坏了**包括 `state_timeout`、`mocap_timeout`、`local_pos_timeout`、`rc_timeout`、`battery_low`、`position_invalid`、`mocap_stamp_stalled`、`ekf2_pos_lost`、`traj_timeout`；处置同样是先停发，若已解锁再请求 `AUTO.LAND`，把降落交给 PX4。

**表 7-8　失效分类。**

| 类 | 原因 token | 停发 | 请求 `AUTO.LAND` | 落点 |
|---|---|---|---|---|
| 控制权被夺走 | `rc_override`、`mode_lost` | 是 | 否 | `MANUAL/RC_OVERRIDE` |
| 输入坏了 | `state_timeout`、`mocap_timeout`、`local_pos_timeout`、`rc_timeout`、`battery_low`、`position_invalid`、`mocap_stamp_stalled`、`ekf2_pos_lost`、`traj_timeout` | 是 | 已解锁时是 | `MANUAL/LAND`；未解锁时回 `INIT` |

输入超时的口径是逐条定的，不是照抄一个 0.5 s。`/mavros/state` 实测 1 Hz，超时取 2.5 s，容一次丢帧；`/mavros/local_position/pose`、`/mocap/pose_enu`、`/mavros/rc/in` 取 0.5 s；`/mavros/battery` 不做超时判据，做的是低电压值判据，因为飞控静默后最后一帧若低，它仍然低；`/mavros/estimator_status` 取 2.5 s，只作 `ekf2_pos_lost` 的新鲜度门。`/mavros/rc/in` 的超时也只在已解锁时才算失效：没解锁时"遥控没开"不会让飞机动。

现场能停任务的有三条软路径，外加一条不是软路径的杀车。

**表 7-9　停任务的三条软路径与杀车。**

| 路径 | 动作 | 程序做什么 | 飞机最后谁管 |
|---|---|---|---|
| (a) 长按开始键 | ch9 按住约 1.2 s | 停发 setpoint + 请求 `AUTO.LAND`，`end_reason=land_pressed` | 仍由程序自主降落 |
| (b) 打杆 | 摇杆变化率超过 100 µs/s，且已解锁 | 0.3 s 内停发、退到 `MANUAL/RC_OVERRIDE`、不请求任何模式 | PX4 按 `COM_OBL_RC_ACT=0` 回 Position，此后由人飞 |
| (c) 拨 ch6 | 拨离第 6 档 | 判 `mode_lost`，停发并交还，连 `AUTO.LAND` 都不请求 | PX4 用它自己的控制器，此后由人飞 |
| (d) 拨 ch8 | 拨到高值 | 程序不读这条通道，状态不变 | PX4 立即停桨；这是杀车，不是停任务 |

复位只有两条路，按键本身不是一条路。第一条是**可信的落地上锁**：`connected=true` 且 `armed=false`，并且要排除飞控断链时 mavros 合成的那一帧断连；这条会清闩锁并重置任务。第二条是**外部把模式放回 OFFBOARD**：闩锁期间我们不请求 OFFBOARD，所以那个上升沿只可能来自人；这条只清夺权闩锁。按 ch9 被明确拒绝，因为控制权不在我们手上时不该由我们复位。未解锁时的输入失效不闩锁：判据恢复就自动回 `STANDBY`，不需要任何动作。

闩锁有两种。`kTakeover` 由夺权触发，不论是否解锁，只能靠上面两条复位路清；`kArmedDegrade` 由已解锁时的输入失效触发，只有可信的落地上锁能清——电池不会因为有人切了模式就变满。闩锁期间日志每 10 s 重报一次，状态本身不会自己松动。

<big>**一句话总结：软停先停 setpoint；复位只认落地上锁或交回。**</big>

---

## 7.9 安全路径的第一手证据

这一节只写 2026-10-12 那几场演练里实际发生的事。PX4 侧的原文都能在 [`docs/assets/drill/`](../../assets/drill/) 里按时间戳找到；节点侧的收尾细节（`ps` 无输出、launch 的 `process has died`）在 [`phase4 票 08`](../../../.scratch/phase4/issues/08-safety-paths-and-fallback-drill.md) 的 Comments 里。演练的判据与记录体例在 [`safety-paths-drill.md`](../../control/safety-paths-drill.md)。

**表 7-10　2026-10-12 的安全路径实测。**

| 路径 | 时刻 | 条件 | 结果 | 原始日志 |
|---|---|---|---|---|
| 打杆接管 | 07:23:27.970 | 装桨、实飞，起飞中被夺权 | `degrade=rc_override` 命中，停发，回 Position | `drill21-probe-run9.log` |
| 拨 ch6 夺权 | 03:59:00.046 | 未解锁；模式被切走 | `degrade=mode_lost`，停发，闩锁不自恢复 | `takeover-modelost-entry.log` |
| offboard 心跳丢失（SIGINT） | 07:52:28.045 | 实飞；`control` 正常停 | PX4 `Failsafe activated`，回 Position，自动降落 | `drill23-sigint-probe-run12.log` |
| 节点崩溃（`kill -9`） | 07:40:45.279 | 实飞；进程被强杀 | 与上一行同路径，`/control/state` 不再更新 | `drill24-probe-run10.log` |
| ch8 杀车 | 08:35:49.134 | 拆桨，未解锁 | PX4 报 `Kill-switch engaged` / `Flight termination active`，程序零反应 | `drill22-ch8-killswitch-probe-run13.log` |

**打杆接管。** 07:23:26.268，ch9 从 999 到 2000，压住 0.35 s；07:23:26.554 PX4 报 `Armed by external command`；07:23:26.951 `/control/state` 到 `AUTO/TAKEOFF`，setpoint 保持 50 Hz。07:23:27.970，PX4 报 `Pilot took over using sticks`；同一拍摇杆 ch3 从 989 到 1059；07:23:27.973 `/control/state` 到 `MANUAL/RC_OVERRIDE`；07:23:27.976 日志 `degrade=rc_override`，已闩锁。随后 setpoint 从 50 Hz 掉到 10 Hz、再到 0 Hz，07:23:28.926 模式落到 `POSCTL`，07:23:20–07:24:10 之间我方发起的模式请求为 0 条。打杆到停发的时间差是 1 ms（探针 07:23:27.972，节点 07:23:27.973）。07:23:37.514 `Disarmed by landing`，07:23:39.934 回到 `MANUAL/STANDBY`，`degrade=none：降级已复位（此前 degrade=rc_override，持续 12.0 s）`。

**拨 ch6 夺权。** 03:59:00.046，模式从 `OFFBOARD` 变成 `STABILIZED`；03:59:00.057 `/control/state` 到 `MANUAL/RC_OVERRIDE`；03:59:00.059 `degrade=mode_lost`，已闩锁。setpoint 从 50 Hz 掉到 0 Hz，最后一帧停在约 +5.2 s；模式停在操作者拨到的位置，没有一条我方模式命令；窗口剩余 115 s 内每 10 s 重报"已闩锁"，状态纹丝不动。这一场全程未解锁，所以它证明的是夺权通路本身，不是打杆判据。

**offboard 心跳丢失。** 07:52:26.897 是最后一帧 setpoint（操作者对 `control` 敲 `kill -INT`）；07:52:28.045 PX4 报 `Failsafe activated`，间隔约 1.15 s，对得上 `COM_OF_LOSS_T=1.0 s` 加余量；07:52:28.905 模式落到 `POSCTL`；07:52:36.752 `Landing detected`；07:52:38.758 `Disarmed by landing`。这条路径不依赖我们的节点还活着，停发超过 1 s 由 PX4 自己接管。

**节点崩溃。** 07:40:44 最后一帧 setpoint；07:40:45.279 PX4 报 `Failsafe activated`；07:40:45.915 模式落到 `POSCTL`；07:40:48.839 `Landing detected`；07:40:50.844 `Disarmed by landing`。节点侧 `ps` 无输出，launch 报 `process has died [pid …, exit code -9]`，`/control/state` 不再更新。SIGINT 与 `kill -9` 在 PX4 侧走的是同一条路；节点日志在两种情况都停在半句，差别只剩 launch 那一行是 `process has finished cleanly` 还是 `process has died`。

**ch8 杀车。** 08:35:49.134 PX4 报 `Kill-switch engaged`，探针在同一秒的 08:35:49.164 读到 ch8 从 999 到 2000；08:35:49.188 与 08:35:49.238 连续报 `Preflight Fail: Kill switch engaged` 与 `Preflight Fail: Flight termination active`。程序侧全程 `MANUAL/STANDBY` 一次没动，setpoint 全程 50 Hz 没掉一拍，没有任何 `degrade=` 行，其他通道一帧未动。08:35:53.640 PX4 报 `Kill-switch disengaged`，ch8 回到 999，此后不再重报。这一场拆桨、未解锁，所以它证明的是"PX4 收到杀车并进入终止状态、程序不读这条通道"，**没有**看到空中真停桨。

**回退 (A) 级要按实际范围读。** 2026-10-12 走的是回退步骤里的第 1、4、5 步：确认 Phase 1 整条链仍在、随时可用。证据是 `2026-10-12-chain-up-clean.log`：`/control/state` 为 `MANUAL/STANDBY`，`/mavros/state` 为 `connected=True armed=False mode=OFFBOARD`，`/mavros/setpoint_raw/local` 约 50 Hz，`/traj/reference` 无帧。第 2、3 步——也就是**把正在跑的 (C) 链换成 (A) 链的那个切换动作本身**——从未演练过。2026-10-12 当天的理由是当时没有正在发 setpoint 的 (C) 链、没有可切的对象；到 2026-10-13，(C) 链的归属已经落地，这条理由不再成立，操作者仍裁定明确不做，理由是排期。按"接受风险 · 没有实测读数作锚"收口。同一张票里的"求解失败注入"也一样：实机链路上从未触发过求解失败，操作者按同一方式接受风险收口。

**读这批材料时要留意的过期处。** `safety-paths-drill.md` §0 与 §2.1 仍写着"打杆试过半场、判据没命中"，那描述的是 03:59 那场 `mode_lost`；07:23 那场后来按同一判据通过，记录在 [`phase4 票 08`](../../../.scratch/phase4/issues/08-safety-paths-and-fallback-drill.md) 的"实测 ③"。同一份演练页 §3.4 说 (C) 链"没有工具，只能 Ctrl-C"，那是在菜单第 9 项补上 MPC 档之前（票 14/15，2026-10-16 与 10-17）；现在 `down.sh` 的 `DOWN_TARGETS` 已经包含它。[`run/z0.py`](../../../run/z0.py) 的 `DOWN_WARNING` 里还留着一句 `NAV_RCL_ACT = 2` 会试图返航，那是旧值；飞控现值是 `1`（Hold，2026-09-28 实测），遥控丢失时 PX4 就地保持，不返航。这些都在文档清扫票的范围里，本章按现状写，不按旧句子写。

<big>**一句话总结：每条安全路径都有当天的日志与时间戳。**</big>

---

## 7.10 现场边界：这条链里没有什么

这条链的边界比它做了什么更值得记住。[`run/z0.py`](../../../run/z0.py) 没有解锁、起飞、切模式、改参数、发任何帧的入口；这不是遗漏，而是把"开始键之前不发控制指令"这条防线留在飞控与 `control` 之间。`z0.launch.py` 也不给程序起飞入口，理由写在它的文件头：那会绕开同一条防线。

**表 7-11　这条链刻意不做的事。**

| 不做 | 原因 | 谁做 |
|---|---|---|
| 自动起桨 / 自动解锁 | 开始键之前不发控制指令 | 人按 ch9 第 1 次上升沿，或台架 T4 的 `/control/takeoff` |
| 自动切模式到 Offboard | 解锁前置要求"已在 OFFBOARD" | 人拨 ch6 第 6 档 |
| 自动让飞机起飞 | 起飞由状态机在解锁后执行，不由起链脚本执行 | `control` 在 `AUTO/TAKEOFF` 里爬升 |
| 运行中在 (A) 与 (C) 之间切换 | 两条链都会写 PX4 的 OFFBOARD，同时跑就是两个驾驶员 | 不存在这条路径；档位只在起链时选 |
| 用 `down.sh` 让飞机降落 | 停进程只是停进程，PX4 还要等超时 | 软停路径 (a)/(b)/(c) |
| 用录制收口让飞机降落 | 录制只订阅 | 同上 |
| 用 QGC 抢模式 | 会与程序的状态机拉锯 | 现场不用 QGC 切模式 |
| 软件杀车通道 | 急停语义 = 停桨，只有 ch8 一条 | PX4 的 `manual_lockdown` |

还有两条容易在术语上滑过去的边界。第一，ch5 虽然保留在参数里，现场不把它当解锁手段：外部解锁会触发 `control` 的"非我方解锁"守卫，程序停发并退回 `MANUAL/INIT`，飞机留在人手里。第二，(A) 与 (C) 的互斥是结构性的：external 档下 `control` 不写 local 话题，offboard 档下不启动 `mpc_node`。所以"起链时选一次"不是提示语，是这条链唯一支持的切换时机；飞行中切换的整条路径不存在，演练页也明确不编造它的步骤。

<big>**一句话总结：起链不给程序起飞入口，停进程不等于降落。**</big>

---

## 7.11 本章引用清单

表 7-12 把本章的命令、通道、状态机与安全结论按主题归到仓库内的文件，便于回查。正文里已经随行给过链接，这里只是索引。

**表 7-12　本章引用清单。**

| 主题 | 文件 |
|---|---|
| 菜单、起链顺序、就绪判据、录制、`down` | [`run/README.md`](../../../run/README.md) |
| 现场时间轴、遥控动作、软停路径 | [`operation_manual.md`](../../../operation_manual.md) |
| 遥控通道分配、阈值、`COM_RC_IN_MODE`、`NAV_RCL_ACT` | [`params-changes.md` §H](../../reference/params-changes.md) |
| 状态、转移、降级、闩锁与复位 | [`phase1-implementation.md` §4](../../control/phase1-implementation.md) |
| 参考项目状态机与接管语义差异 | [`xi35-fsm.md`](../../reference/xi35-fsm.md) |
| 安全路径演练判据与记录 | [`safety-paths-drill.md`](../../control/safety-paths-drill.md) |
| 2026-10-12 演练日志 | [`docs/assets/drill/`](../../assets/drill/) |
| 飞控参数现值 | [`docs/assets/z0.params`](../../assets/z0.params) |
| 状态机与遥控实现 | [`fsm.hpp`](../../../src/control/include/control/fsm.hpp)、[`rc_switch.hpp`](../../../src/control/include/control/rc_switch.hpp) |
| 起链脚本与录制层 | [`run/z0.py`](../../../run/z0.py)、[`run/rec.py`](../../../run/rec.py) |
| 录制层的"只订阅"守卫 | [`run/tests/test_rec.py`](../../../run/tests/test_rec.py) |
| 起链与现场排障 | [`bench-runbook.md`](../../control/bench-runbook.md) |
| 术语 | [`CONTEXT.md`](../../../CONTEXT.md) |

<big>**一句话总结：现场结论都能回到仓库内文件。**</big>
