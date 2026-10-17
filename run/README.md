# `run/` —— z0 的**人工启动层**

**本目录只有一个给人工用的入口**：把「起链那几条命令 + 每步的核对 + 两条容易忘的前提」
收成一张菜单。它**不含任何飞行逻辑**，也**不替代** [`bench-runbook.md`](../docs/control/bench-runbook.md)
（那张表是**判据与记录**：做一遍、填读数、勾框；本脚本只是把命令送出去、把读数摊开）。

```
run/z0.py                # 交互菜单：敲数字
run/z0.py --list         # 只列菜单（不进交互）
run/z0.py env            # 环境自检（只读）
run/z0.py check          # 起链核对（只读，约 5 s 一张表）
run/z0.py up             # ★ 一键启动全部（第 9 项；run/up.sh 是它的一行别名）—— 起链前选控制器（两档）
run/z0.py up_manual      # ★ 一键启动到 mocap 为止（第 8 项）—— 手动遥控飞，**不问**
run/z0.py mpc            # 只起 MPC（模型预测控制）节点（不进菜单；第 9 项的第 2 档就是它）
run/z0.py down           # ★ 清除后台节点（第 10 项；run/down.sh 是它的一行别名）
run/down.sh --dry-run    # 只扫一眼"会停谁"，**一个信号都不发**
run/z0.py rec_start 3    # ★ 开始录制（第 11 项；场景 1–6，不给号会问一句）
                         #   ★ **它占住终端**：屏幕上是倒计时，录满 120 s 自己收口，提前停按 Ctrl-C
                         #     （场景 6「任意录制」不倒计时，只有 Ctrl-C 收口）
run/z0.py rec_status     # 现在在录什么（只读，不进菜单；在**另一个终端**看）
run/z0.py --dry-run rec_start   # 六条场景与它们会跑的命令全列出来，**一个进程都不起**
run/z0.py --dry-run up   # 把顺序、每条命令、每步判据全列出来（默认档一整块 + 另外**一**档一块），**一条都不起**
run/z0.py --dry-run mocap   # 只打印将要执行的命令，**不执行**
run/z0.py mavros|vrpn|mocap|traj|control   # 单条起（前台）
run/z0.py bringup|fly|mpc|rec_status|rec_manifest   # 五条**不进菜单**的（见下）；直接点名仍可跑
```

★ `--dry-run` 对**所有**动作都是"只打印、不执行" —— `env` / `check` 这两条只读动作**也不例外**
（它们没有命令可打印，会明说一句"只读、本进程里直接跑、`--dry-run` 不执行"）。
`--dry-run mavros` 还会把"起之前会问哪几句"一并列出来，免得现场不知道有 `gcs_url` 这条可选参数。

`env` / `check` 在没 `source` 过的 shell 里会**自己带 source 重新 exec 一次**（`Z0_SOURCED=1` 防重入）；
**交互菜单那条路在印菜单之前就先重入完** —— 它也是 `exec`，不能排在读数字之后（否则你敲的那一行会
随旧进程消失，见排障表那条）。起链的每条命令也各自带 `source` 前缀 —— 顺序与
`bench-runbook.md` 的「起链（顺序不能反）」一致：

```bash
source /opt/ros/humble/setup.bash && source ~/mavros_ws/install/setup.bash && source ~/ros_ws/install/setup.bash
```

## 菜单里有什么

| # | 项 | 形态 | 它替你做掉的判断 |
|---|---|---|---|
| 1 | 环境自检 | 只读 | 三条 `setup` 在不在、六个包**是没 build 还是没 source**、串口在不在/被谁占、域号、当前 commit |
| 2 | 起链核对 | 只读 | 每个话题**有没有帧、多少 Hz**；`/mavros/state` 的 `connected/armed/mode`；`/control/state` 原文 |
| 3 | 起 mavros | 前台 | 起之前先查 `/dev/ttyTHS1` 被谁占着（第二个 mavros 的报错**长得像 launch 坏了**）；再问一句要不要 `gcs_url` 桥给 QGC |
| 4 | 起 vrpn_listener | 前台 | 提醒刚体名要在 Nokov 侧与 `mocap.yaml` 对齐（`Z0`/`z0` 两种都出现过） |
| 5 | 起 mocap | 前台 | ★ **起之前要你确认"飞机已摆好、静止"**（阶段 A 原点在节点启动那一刻锁定） |
| 6 | 起 traj | 前台 | 只发参考、不碰飞控；一帧不发直到被 select + start |
| 7 | 起 control | 前台 | 台架 T3 用的就是这一条；`MANUAL/INIT` 是正常起始态 |
| 8 | ★ **一键启动（动捕+mavros，手动遥控飞）** | 本进程编排 + 前台守着 | **不问**（它只有一档）：mavros → vrpn → mocap **按序**起齐、每步等判据，**不起 `traj`/`control`** ⟹ 没有任何程序在上位控制飞机，起完**直接用遥控器手动起飞** |
| 9 | ★ **一键启动全部（起前选控制器；默认 offboard 控制）** | 本进程编排 + 前台守着 | 起链前问一句选档（**两档，两档都带 `traj`**，见下节）；**空回车** = 把 3→4→5→6→7 五步**按序**起齐（`bench-runbook.md` 的「起链（顺序不能反）」），每步**等判据成立**；起完把**终端让给操作台 `fly.py`** 选轨迹，然后打印核对表 + 起飞提示 |
| 10 | ★ **清除后台节点** | 本进程编排 | 与第 9 项相反的**另一头**：**按名字扫**（`DOWN_TARGETS`，含 `mpc`）在跑的节点 ⟹ 先摊开清单、问一句 y/N，再按 SIGINT→SIGTERM→SIGKILL 停；停完**再扫一遍**并报串口状态。详见下下节 |
| 11 | ★ **开始录制（选场景 1–6）** | 本进程编排（起 `ros2 bag record`，**前台占住终端**） | 只**订阅**；选之前只报一行短名单，**选定后、起录之前**把那一场的"起哪条链 + 你要做什么 + 预期话题"整张卡摊开；然后屏幕上走倒计时，**录满 120 s 自己收口**（收口 = SIGINT ⟹ 压包 ⟹ `ros2 bag info` 的**实测帧数**逐条回对预期 ⟹ 写 `session.json` + 重算 `bag/MANIFEST.md`），提前停按 Ctrl-C；首选目录已被占就**问一句**（答 y 换一份带时间戳的新目录）；场景 6 **不倒计时**，只有 Ctrl-C 收口 |

★ **五条不进菜单**（`menu=False`：不显示、不占号，但仍在动作表里 ⟹ 安全不变式一条没松）：

| key | 谁在用 | 位置 |
|---|---|---|
| `bringup` | `z0.launch.py` 一键起 mocap+traj+control —— `DOWN_TARGETS` 要从它取包名 | 照 [`bringup/README.md`](../src/bringup/README.md) 手打，或 `run/z0.py bringup`（会问一句 `use_vrpn:=false`） |
| `fly` | 操作台 —— 第 9 项两档起齐后**前台**跑它选轨迹 | 那两档里自动起；想单独再跑：`run/z0.py fly` |
| `mpc` | **模型预测控制节点**（`nmpc` 的 `mpc_node.py`）—— 第 9 项的第 2 档起它，带上 `--ros-args -p reference_source:=control` + `offboard_prestream:=true`（参考来自 `/control/target`，不再自己发 `/traj/start`）。★ **命令自带 acados 的两个环境变量**（`ACADOS_SOURCE_DIR` / `LD_LIBRARY_PATH`，`env …` 前缀）—— 缺一个它就在建求解器时 `OSError: libacados.so / libhpipm.so`**起不来**（2026-10-17 起链实测死在第 5 步）；外层已经 export 对了就听外层的 | 那一档里自动起；想单独再跑：`run/z0.py mpc`（它**会问**那四条可选参数 —— 两条参考源**互斥**；接 `control` 的目标答 `reference_source_control`） |
| `rec_status` | 排障：**现在在录什么**（只读：读状态文件 + 按名字扫一眼录制进程） | `run/z0.py rec_status` |
| `rec_manifest` | 排障：从 `bag/**/session.json` **重算总表**（每次收口都会顺手写一次；手工拷完 `.ulg` 之后也值得跑一次） | `run/z0.py rec_manifest` |

## ★ 一键（第 8 / 9 项，`run/up.sh` = 第 9 项）

```bash
run/up.sh              # 等价于 run/z0.py up（第 9 项：全部程序 + 选轨迹）
run/z0.py up_manual    # 第 8 项：动捕 + mavros，手动遥控飞
```

**两条一键的公共前缀逐字相同**：mavros（带 GCS 桥）→ vrpn_listener → mocap，
**逐段等它真的起来**再走下一步。**差别只在起链前那一句选出来的「尾巴」**（见下表）。

| 步 | 起什么 | 起之前 | 起来的判据（**等**，不是"起了就算"） |
|---|---|---|---|
| 1 | `mavros` + **`gcs_url` 桥给 QGC** | 串口被谁占着；**已经有 mavros 在跑（`connected=True`）就问一句"沿用还是照样再起"** | `/mavros/state` 的 **`connected=True`**（≤60 s）★ **有帧 ≠ 连上飞控** |
| 2 | `vrpn_listener` | — | `/vrpn/<刚体>/pose` 有帧（≤30 s；`Z0`/`z0` 都认） |
| 3 | `mocap` | ★ **要你确认"飞机已摆好、静止、固定？"** | `/mocap/pose_enu` 有帧（≤30 s） |

**尾巴**（第 9 项起链前问一句「这一趟谁在上位控制飞机」，输数字；**空回车 = 第 1 档**）：

| # | 选什么 | 尾巴 | 额外带的参数 | 第 9 项默认 |
|---|---|---|---|---|
| 1 | offboard 控制（`control` 自己发 PX4 setpoint） | `traj` → `control` | ——（`output_mode` 默认 `position`） | ★ |
| 2 | **模型预测控制**（`mpc_node.py` 发执行器指令） | `traj` → `mpc` → `control` | `control`：`output_mode:=external`；`mpc`：`reference_source:=control` + `offboard_prestream:=true` | |

★ **第 8 项不问**（它只有一档、一版之前的样子）；**第 9 项两档**，**两档的尾巴都含 `traj`**
（操作者 2026-10-16 的原话「轨迹必须起没得选」⟹ 这是**结构上的事实**，不是提示语）：
模型预测控制**没有"定点保持（`hold`）、不起 `traj`"那一档了**（一版有过）。
★ **空回车 = 今天的行为一个字不变**（第 9 项 = `+traj/control`）。两档的**操作流程完全相同**：
`control` 始终是任务总管（开始键、解锁、切 OFFBOARD、起飞、判稳、轨迹原点、`/traj/start`、降落），
第 2 档只是把「谁最后给 PX4 发指令」换成 `mpc`。选到第 2 档就按它起：

| 步 | 起什么 | 起之前 | 起来的判据 |
|---|---|---|---|
| 4 | `traj`（两档都有） | — | **不等**（没 `select` 就一帧不发，等它等于等到天荒地老） |
| 5 | `mpc`（第 2 档） | — | ★ **它自己的启动日志**那一行（≤120 s）—— 闸门关着时它**一帧都不发**（`offboard_prestream` 只发纯悬停帧），话题面上没有"我起来了"这个事实；等不到就打日志尾巴、**把已起的全部停掉**并返回 1 |
| 5′ | `control`（**两档都有**；第 2 档带 `output_mode:=external`） | — | `/control/state` 有帧（≤30 s） |
| 6 | **终端让给 `fly.py`**（两档都要，前台） | — | 你敲 1/2/3/4 选轨迹（**起飞前必须选好**），选完按它的提示退出 |
| 7 | — | — | 打印核对表 + **起飞提示** ⟹ 前台守着（**Ctrl-C 一起停**） |

★ **两档的现场动作逐字相同**：选轨迹 → ch6 拨 Offboard → 开始键第 1 次上升沿（解锁 + 起飞）
→ 判稳后开始键第 2 次上升沿、**松手**（进 `TRACK`、发 `/traj/start`）→ 飞完自动降落。
两档的区别只在**输出通路**：

- 第 1 档：`control` 直接写 `/mavros/setpoint_raw/local`；
- 第 2 档：`control` 只写内部绝对目标 `/control/target`（位置 + 速度 + yaw），
  `mpc_node` 求解后写 `/mavros/setpoint_raw/attitude`。**`control` 一帧都不写 local**，
  两路 setpoint 不会互相覆盖。`/control/target` 断流 0.5 s ⟹ `mpc_node` 以
  `no_control_target` 关闸（**零帧**，不是冻结；`hold → hover` 兜底阶梯只管求解不可用）；
  最后一道是 PX4 自己的 1 s OFFBOARD 丢失保护。

★ 第 2 档仍然保留 `offboard_prestream:=true`：`control` 请求 OFFBOARD 之前的那段预流里，
`mpc` 还没有参考也要让飞控看见姿态流。它**不 arm、不 set_mode** —— 这两件事仍在 `control`。
★ `fly.py` 的状态栏在**两档下都正常**（`/control/state`、`/control/traj_progress` 都由 `control` 发）。

- **日志**：`run/logs/<时间戳>-up/01-mavros.log …` —— 第 1 档到 **`05-control.log`**，
  第 2 档到 **`06-control.log`**（中间多一个 `05-mpc.log`；第 8 项是 `…-up_manual/`；
  已在 `.gitignore` 里）。
  某步等超时、或**子进程半路自己退了**，就把那个日志的**最后几行**打出来。
- **Ctrl-C 一起停**：子进程各自成组（`start_new_session`）⟹ 停的时候**整组一起**停
  （先 SIGINT、超时才 SIGKILL）。★ 前台那个 `Ctrl-C` **到不了**子进程（它们自成会话），
  必须由本脚本转发 —— 所以**别 `kill -9` 本脚本**：`SIGKILL` 不可捕获，转发就没了，
  会**留下 mavros 占着串口**。
- ⚠️ **把终端让出去那一段**：那时前台跑的是 `fly.py`，`Ctrl-C` 归它；脚本的后台子进程仍在，
  你退出 `fly.py` 之后脚本接着守着（`Ctrl-C` 又回到"一起停"）。
- **起飞那一下不代做**（见下节）：脚本只把"按开始键哪两下、压多久"写在屏上。
- `--dry-run up` / `--dry-run up_manual` **不进入提问**：先把**默认那一档**整条摊开，
  再另起一块（`── 其他档（起链前问一句，选到才起）──`）把另外**那一档**的命令也摊开；
  ★ 第 8 项只有一档 ⟹ **没有**那一块（明说一句"本链只有一档，起链时**不问**"）。

## ★ 清除后台节点（第 10 项 / `run/down.sh`）

```bash
run/down.sh              # 等价于 run/z0.py down
run/down.sh --dry-run    # 只扫一眼"会停谁"，**一个信号都不发**
```

**为什么不是"停 `up` 起的那些"**：`up` 记子进程的那张表是 `cmd_up` 的**局部变量**，
脚本一退（或被 `kill -9`）就跟着没了 —— 而**恰恰是那种场合最需要它**（起链的终端已经关了、
节点成了孤儿，`Ctrl-C` 那条路没有了）。所以这一条**按名字扫**，白名单在 `DOWN_TARGETS`：

| key | 扫什么（`pgrep -f` 的模式） |
|---|---|
| `mavros` | `ros2 launch mavros ` + `mavros_node` |
| `vrpn` | `ros2 launch vrpn_listener ` + `vrpn_listener` |
| `mocap` | `ros2 launch mocap ` + `mocap_rel_pose_node` |
| `traj` | `ros2 launch traj ` + `traj_node` |
| `control` | `ros2 launch control ` + `control_node` |
| `bringup` | `ros2 launch bringup ` |

★ 每条都是"**起链那条命令 + `ros2 launch` 拉起来的可执行名**"两个人（`ros2 launch` 的壳与节点本身
是**两个进程**），按 pid 去重 —— 所以清单里一个节点可能出现两次，那是正常的。
模式**刻意写窄**（不许出现 `python3` / `bash` / `.*`）：它是 `pgrep -f`，宽一寸就误伤一片。

| 它做什么 | 具体 |
|---|---|
| 先列清单 | 每条印出 **pid + 完整命令行**（`render_down`）；这时**一个信号都没发** |
| 问一句 | `ask_confirm` 一次 y/N；答 n ⟹ 直接返回（退出码 1，**什么都没动**） |
| 停 | 阶梯 **SIGINT → 等 5 s → SIGTERM → 等 3 s → SIGKILL**（`STOP_LADDER`）；与台架 `stop_dut`（[`scenario.py:298`](../src/bench/bench/scenario.py#L298)）是**同一条纪律、不是同一组数字**（它每档等 5 s、且对**进程组**发，见下） |
| 报结果 | 再扫一遍；还有剩的**如实报出来**（退出码非零），并打印 `/dev/ttyTHS1` 的占用状态 |

**两条实现上的取舍，都是踩过才知道的**：

1. **只发单个 pid，不发进程组**。`up` 那套用 `os.killpg` 是因为那些子进程**是它自己起的**
   （`start_new_session=True` ⟹ 组是干净的）；而这里扫出来的人**可能与别人终端的 shell 同组**
   ⟹ 对那个组放 `killpg` 等于连人家的终端会话一起收掉。
2. **僵尸不算"活着"**（`_pid_alive` 回 `/proc/<pid>/stat` 看状态字符）。`os.kill(pid, 0)` 对僵尸
   **是成功的**，只看它就会对着尸体把阶梯升到 SIGKILL、最后报一句假的"没停掉"、退出码非零。
   第一手：拿两个真进程试 `stop_pids`（一个忽略 `SIGINT`、一个正常）时撞上的 —— 修复前后
   分别报"两个都杀不掉（10.1 s）"与"干净（5.2 s，忽略 INT 的那个死在 SIGTERM）"。

⚠️ **它不是降落，也不解锁、不切模式、不碰飞控**：setpoint 一停，PX4 在 `COM_OF_LOSS_T`（1.0 s）后
自己退出 OFFBOARD，按 `COM_OBL_RC_ACT`（现值 `0`）落回 Position；遥控同时丢了也不要紧 ——
**A1 已落地**（`NAV_RCL_ACT = 1` Hold，★ 2026-09-28 第一手实测），PX4 **就地保持**。
飞机还在空中时先按
[`operation_manual.md`](../operation_manual.md) §1.2 处置。
⚠️ **按名字扫 ⟹ 别人会话 / 跑着的测试起的同类节点也在清单里**（现场只有一套飞机，这个代价是明说的）
—— 动手前**看一眼命令行**，不是你这套的就答 n。

★ **清除这条路不进 `ACTIONS` 的命令列**（第 10 项的 `cmd` 与第 9 项/第 8 项一样是 `None`：本进程里编排），
所以它另有两条守卫用例：`test_清除目标表按名字扫_且把起链的每一条都盖住`（`UP_CHAIN` 每一条都要被盖住、
模式得是能编译的正则、不许写宽）+ `test_清除目标表只有一张_且命令仍受安全不变式扫`
（那张表里不许出现 `ros2 service call` / `ros2 param set` / `ros2 topic pub` / `ros2 action send_goal`——
守卫扫的是"最终送进 `bash` 的东西"，这里是**另一条把字符串送出去的路径**，所以单独扫一遍）。

## ★ 录制（第 11 项，`run/rec.py`）

```bash
run/z0.py rec_start          # 菜单 11：不给场景号会问一句（1–6）
run/z0.py rec_start 3        # 直接点名场景 3
run/z0.py rec_status         # 另开一个终端：现在在录什么、录了多久
run/z0.py rec_manifest       # 手工拷完 .ulg 之后重算一次总表
```

★ **一份包的生命周期就圈在这一条命令里**（操作者 2026-09-29 现场拍定）：菜单 11
**占住终端** —— 屏幕上走倒计时，**录满 120 s 自己收口**，想提前停按 **Ctrl-C**
（转手发给录制进程同一路 SIGINT）。⚠️ 因此**没有**"停止录制"这一项了，"录着的同时去按
另一项"这件事不存在；要看进度请**另开一个终端** `run/z0.py rec_status`。

**六条场景**（操作者 2026-10-08 拍定）：`1` 静止 / `2` 手飞起飞 / `3` 轨迹 1（悬停）/
`4` 轨迹 2（正方形）/ `5` 轨迹 3（八字）/ `6` **任意录制**（自由录：不预设场景与时长，
**不倒计时**，只有 Ctrl-C 才收口，落进带 UTC 时间戳的新目录 ⟹ 想录几遍录几遍）。
⚠️ 4/5 **重新编过号**（原来 4 是八字）：`bag/4_traj3_lemniscate/` 那一份**已入库的**八字包
不改名、不动内容，靠 `rec.SLUG_ALIASES` 仍认得出它是现在的场景 5。
★ 2026-10-08 又清过一次 `bag/`（每个场景只留最新的一份）⟹ 那份现在在
`bag/old/4_traj3_lemniscate/`，`bag/` 一层里已经没有这个目录名了（见 [`bag/README.md`](../bag/README.md)）。

**为什么有这一层**：[ticket 11](../.scratch/phase2/issues/11-sitl-tier-b-seam3.md)（Phase 2 的
B 档）要的"真实动捕 rosbag"**从来没录过** —— 2026-09-28 那次真机自由飞一个包都没留。
理想轨迹替不了真实噪声与真实延迟，所以场景是按**用途**反推出来的最小集合
（噪声基线 / 起飞瞬态 / 悬停判据 / 有机动的激励 / 任意补录）。逐条见
[ticket 19](../.scratch/phase2/issues/19-record-four-flight-scenarios.md)。

| 它做什么 | 具体 |
|---|---|
| 起录 | 起 `ros2 bag record`，本进程**前台**守着（倒计时就在屏幕上）；判据：**已经在录就不起第二个**、首选目录已被占就**问一句**（答 y 换一份带时间戳的新目录，答 n 才是老行为） |
| 录什么 | 话题走白名单正则 `^/(mavros\|mocap\|traj\|control\|vrpn)/`，**不是 `-a`**；压缩 `zstd`（在**收口那一刻**压） |
| 到点 | **`Scenario.expect_s`（场景 1–5 统一 120 s）到了就自己收口**；想提前停按 **Ctrl-C**，两条来路合成同一条 SIGINT 收口路。⚠️ 场景 6 `expect_s=None` ⟹ **不到点**，只有 Ctrl-C |
| 收口 | **SIGINT**（与我们替你发的**同一条路**）⟹ 等它写缓存、压包、关库（≤45 s）⟹ 等不到才 SIGTERM / SIGKILL；然后 `ros2 bag info` 的**实测帧数**逐条回对 `Scenario.expect` ⟹ 写 `session.json`（时长/大小/话题/频率/sha256/**缺了什么**、**为什么收的**）+ 重算 `bag/MANIFEST.md` |
| 飞控日志 | **手工**（2026-09-29 拍定）：拔 SD 卡，把 `<卡根>/log/sessNNN/logMMM.ulg` 直接拷进 `bag/<场景>/fcu/` —— 本层**不拷、不删、不猜**，只在总表里把那个目录里已有的东西列出来 |

**五条第一手/设计取舍**（改之前先读）：

1. **`rosbag2` 拒绝往已经存在的目录里录**（`Output folder '…' already exists.`，**不覆盖也不接着录**）
   ⟹ 包目录**只由 `rosbag2` 自己建**，录制自己的日志因此落在包目录**旁边**，名字**跟着包目录名**
   走（`bag/<包目录名>.log`；`session.json` 的 `log` 字段就是它的实际路径）—— 重录那几份各有
   各的日志，否则第二次起录会把第一份的日志覆盖掉，而那正是排障时要看的东西。
   ★★ **目录已存在不再是一句死路**（2026-10-08 修）：先诊断是"已入账的包"还是"上次没收干净的"，
   再**问一句** —— 答 y 就录进 `bag/<场景>__<UTC 时间戳>/`（已有那份一个字节都不碰），
   答 n（默认）才是老行为。**从不替你删包、也不替你挪包。**
   ⚠️ 修之前的形状是"选哪个场景都当场退出"：`bag/` 下那四份 09-29 入库的包把**每一个**场景号
   都堵死了，现场看起来就是"菜单 11 这项录制坏了"。
2. **收口只发单个 pid、`killpg` 只在确认"这个组是我们起的"时才用**（`getpgid(pid) == pid`
   ⟸ `start_new_session=True`）。兜底那条路（按名字扫）**不碰进程组** ——
   理由与 `down` 那条**逐字相同**（扫出来的人可能与别人终端的 shell 同组）。
3. **录制进程自成一个会话，所以终端 Ctrl-C 传不到它**（`start_new_session=True` 的另一面）
   ⟹ 倒计时期间按下的 Ctrl-C 落在**本进程**，我们接住它、**转手发给录制进程**；
   收口那几秒（写缓存 → 压包）里刻意**忽略 Ctrl-C** —— 连按两下会把 `ros2 bag info` 与
   `session.json` 一起打断，而 `rosbag2` 那边正在好好收尾。
4. **`--dry-run` 对这两条也是"什么都不做"**：不起进程、不发信号、不写字节。
   `--dry-run rec_start` 会把六条场景与它们会跑的命令全摊开
   （自由录那一行按**真跑时同一把尺**印落点：带 UTC 时间戳的新目录）。
5. **它不碰 ROS 的进程内环境**（录制层不调 `ensure_sourced`）：要 `source` 的是**子进程**
   （那条命令本来就带三条 `source` 前缀），父进程的环境一点用不上。

⚠️ **它不是降落，也不解锁、不切模式、不发帧**：飞机还在空中时，先按开始键 / 拨 ch6 接管 ——
与第 10 项 `down` 同一句警告。产物目录的布局、字段、坑都在 [`bag/README.md`](../bag/README.md)。

★ 这一层**不进 `ACTIONS` 的命令列**（三条的 `cmd` 都是 `None`，本进程里编排），
所以另有守卫：[`test_rec.py`](tests/test_rec.py) 用 **AST** 扫 `create_publisher`/`create_service`/
`create_client` 与那四个禁字（"只许订阅"这条边界对录制层同样成立），另有"文件头引用的菜单号
必须与 `ACTIONS[N-1]` 对得上"那一条（与 `operation_manual.md` 的守卫同一个形状）。
★ 倒计时的判据（`watch` / `render_progress`）是**纯逻辑**：用例拿假时钟把 120 s 瞬间跑完 ——
**能离线验的判据不留到现场**（那条纪律与 `test_z0_menu.py` 同一条）。

## ★ 它**不**做什么（安全边界）

**没有任何"让飞机动"的入口**：解锁 / 起飞 / 切模式 / 改参数 / 发任何帧，一个都没有。
台上那两条路**照旧手打**——

- 现场：**遥控器开始键**（CH9 第 1 次上升沿 = 解锁 + 起飞，第 2 次 = 进 `TRACK`；压住 ≥ 0.3 s 再松）；
- 台架：`bench-runbook.md` **T4** 的 `/control/takeoff` 服务调用（**原文在那里**，本脚本只提一句"在哪"）。

理由不是洁癖，是仓库里已经写死的设计：[`z0.launch.py`](../src/bringup/launch/z0.launch.py) 的
「本包**不给**"程序起飞"的入口，因为它会绕开"开始键之前不发控制指令"这条防线」、
[`bringup/README.md`](../src/bringup/README.md) 的「起完之后**还差两步**（刻意不自动化）」。

可以多问一句的两条参数（mavros 的 `gcs_url`、bringup 的 `use_vrpn`）**做成了 y/N，值写在动作表里**：
有意**不给"随手输一段参数"**的自由 —— 那等于把任意 shell 片段送进 `bash -c`，与上面这条边界相冲。

★ **两条一键（第 8 / 9 项）都不新开一条"起链用的命令表"**：`up` / `up_manual` 的动作表项 `cmd=None`
（本进程里编排），它们起的每一条都**只写 key**（公共前缀 `MANUAL_CHAIN` + 选出来的那一档
`UP_CHOICES` / `MANUAL_CHOICES` 里选出来的那一档的 `tail`；要让终端的那两档另有
`choice.handoff` 把 `fly` 放前台），
命令取自上面那张动作表 ⟹ 下面这套守卫**不用改就盖住它们**。
★ 第 9 项第 2 档起的 `mpc` 只是**动作表里多一行**（`menu=False`：`env <acados 两个变量>
ros2 run nmpc mpc_node.py`）—— 它是第 8/9 项唯一一条**不是 `ros2 launch`** 的起链命令。
★ 那个 `env` 前缀**只设变量、后面仍然只走 `ros2`**：守卫用例专门钉住这一点（别把它放宽成
"随便什么前缀"——"命令的出口只有 `ros2`"正是那条不变式）。

★ 这条由 `run/tests/test_z0_menu.py` 的用例盯着：一条扫**动作表**（主命令**与**可选参数的取值，
不含 `ros2 service call` / `ros2 param set` / `ros2 topic pub` / `ros2 action send_goal`），
一条用 **AST** 扫**源码本身**（不许出现 `create_publisher` / `create_service` / `create_client` ——
本脚本只许**订阅**）。走 AST 而不是字符串搜索，是因为守卫表自己必须写出那些字样，
字符串搜索会把守卫表本身判成违规。
⚠️ 这一层的**威胁模型是"手滑新增一条命令"**，不是"刻意规避"：真要写成
`"ros2 service" + " call"` 拼出来，静态扫描拦不住 —— 能拦住的是"表里多了一条一眼看得见的命令"，
而那正是实际会发生的错法。

## 起链核对（`check`）读的是什么

| 话题 | 判据来自 | 备注 |
|---|---|---|
| `/mavros/state` | `control/config/control.yaml:41` | **1 Hz 是正常的**（mavros 的连接心跳） |
| `/mavros/local_position/pose` | `control.yaml:42` | EKF2 位置 —— 控制律的位置来源 |
| `/mavros/vision_pose/pose` | `mocap/config/mocap.yaml` 的 `vision_pose_topic` | 动捕 → EKF2 的载体 |
| `/mocap/pose_enu` | `mocap.yaml` 的 `pose_enu_topic` | 动捕实测 ~200 Hz |
| `/control/state` | `control.yaml:67` | `std_msgs/String`；`MANUAL/INIT` 是正常起始态 |
| `/traj/reference` | `traj.yaml` 的 `reference_topic`（50 Hz） | ★ **没选轨迹时本该是空的**（表里记 `·`，不算错） |
| `/mavros/setpoint_raw/local` | `control.yaml:63` | ≈50 Hz ⟹ 我们正在流 |
| `/vrpn/<刚体>/pose` | 图里**动态发现** | 刚体名不做大小写归一化 |

实现上有意选的两条（都在 `z0.py` 里写了理由）：
**探测走本进程的 rclpy 节点**（不经 `ros2` CLI ⟹ 绕开下面那条 daemon 坑）；
**订阅 QoS 取 `best_effort + volatile`**（对任何发布方都兼容 ⟹ 「我订上了没有」不会变成第二个静默坑）。

## 排障速查

| 现象 | 先看 |
|---|---|
| 自检说"六个包全查不到"，但 `colcon build` 明明跑过 | **装好了但没 source**（自检会这么写）。`env` 看的是**当前**环境；起链命令自己带 `source` ⟹ 那个才是准的 |
| 手工 `ros2 topic list` 只回 `/rosout`、`/parameter_events`，而节点明明活着 | ★ **`ros2` daemon 的坑**：同域内 daemon 是被**先前那个环境**拉起来的，`ROS_LOCALHOST_ONLY` 之类不一致时它**静默看不见**（第一手：夹具域 73 上实测，加变量无效、`ros2 daemon stop` 后才看到全表）。加 `--no-daemon`：`ros2 topic list --no-daemon --spin-time 3`（★ 它是 `topic list` 的**子命令参数**，`ros2 --no-daemon …` 会报 `unrecognized arguments`） |
| 起 mavros 报"打不开串口" | `/dev/ttyTHS1` 已被另一个 mavros 占着 —— 菜单 3 起之前就会告诉你占的人是谁 |
| `mocap` 起来后位置整体偏一截 | 起 `mocap` 之前飞机就在动（阶段 A 原点漂了）；菜单 5/8 会先要你确认 |
| 表里 `/traj/reference` 是 `·` | 正常：**没选轨迹本来就一帧不发**（`traj` 要 `select` + `start`） |
| `run/down.sh` 说「缺 `pgrep`/`ps`」 | 这一条**什么都没做** —— 它判活与取命令行就靠这两个命令，别把这句当成"已经清干净了"（装 procps 或手工 `ps` 核对） |
| 表里某话题"在图里但一帧都没来" | 两种可能：发布者没起，或 **QoS 不兼容**（后者**不报错**，见 [`bench/README.md`](../src/bench/README.md) 的「安全边界」） |
| 菜单里敲的数字**要敲第二次才认** | ★ **已修**（2026-09-23，ticket 22 §九）：`ensure_sourced()` 是"把本进程 `exec` 掉重开"，原先它只挂在动作里 ⟹ 你敲的那一行**随旧进程一起没掉**、菜单重印一遍。现在重入提到 `input()` 之前，菜单只印一遍。**若你的机器上还这样**，说明跑的**不是**修好之后的 commit（`git log -1 run/z0.py` 看一眼） |
| `ROS_DOMAIN_ID=73` | 那是**台架夹具域**（[`bench/pytest_support.py`](../src/bench/bench/pytest_support.py) 默认，且**硬拒 0**）⟹ 读数不是真飞机；核对表第一行会标出来 |
| `test_z0_check_live.py` 的负控用例忽然红了 | ★ **别同时跑 `colcon test`** —— 两者同在夹具域 73，`control` 的用例会把 `/control/state`、`/mavros/setpoint_raw/local` 真发出帧来，负控（"只有订阅者的话题"）随即失真。第一手：2026-09-23 撞到 2 例假失败，colcon 跑完再跑就绿 |

## 测试

```bash
python3 -m pytest run/tests/test_z0_menu.py -q          # 纯函数 + 命令行拼装 + 安全不变式 + 两条一键/清除编排（不需要 ROS，81 例）
python3 -m pytest run/tests/test_rec.py -q              # ★ 录制层：命令拼装/解析/前台倒计时/收口/核对/安全边界（不需要 ROS、不碰飞机，80 例）
python3 -m pytest run/tests/test_operation_manual.py -q   # ★ 现场那一页的守卫：菜单号/命令/话题/degrade token 都钉在代码上（不需要 ROS，5 例）

# ★ 正控：对着台架夹具（假 mavros）跑一遍 check，读数必须真的读到东西（5 例）
source /opt/ros/humble/setup.bash && source install/setup.bash
python3 -m pytest run/tests/test_z0_check_live.py -q
```

`run/` **不是 colcon 包** ⟹ 不进 `colcon test`；上面四条是它自己的入口。
（★ 夹具的域号由 `bench/pytest_support.py` 定，默认 73 —— 那条纪律的理由见它的文件头。）

⚠️ **要一次跑整个 `run/tests/`，必须先 source**（上面那两行）。没 source 时
`test_z0_check_live.py` 会在**模块级** `importorskip` 上跳掉，而 pytest 6.2.5 在那种情形下
**连这一整个目录的其余用例一起收集不到**（第一手：未 source 跑 `pytest run/tests -q` ⟹
`1 skipped`、0 例；source 后同样一条命令 ⟹ 全过）——
★ 所以"跑整个目录是绿的"这句**只有在 source 过的环境里才成立**，未 source 时那是**空收集**，不是绿。

★★ **空收集长什么样、退出码是多少**（2026-10-13 实测补记，★ 这是这一节的**唯一口径出处**）：

| 情形 | 屏幕 | 退出码 |
|---|---|---|
| 未 source，整目录 | **`1 skipped in 0.14s`**（★ 不出现任何"没跑"的字样） | **`5`** |
| 未 source，`--ignore=run/tests/test_z0_check_live.py` | `161 passed` | `0` |
| 未 source，单独点名任一其它文件 | 照常跑 | `0` |

⟹ **退出码是 `5`，不是 `0`**（pytest 的 `ExitCode.NO_TESTS_COLLECTED`）。
⚠️ 这一条与另外两处旧记录**不一致**（[`phase3/spec.md`](../.scratch/phase3/spec.md)、
[`phase2/26`](../.scratch/phase2/issues/26-recording-six-scenes-and-the-rewrite-path.md) 当时写的是"退出码仍是 0"）——
**那两处已在 2026-10-13 就地更正并指向本节**；而且这不是四舍五入的出入，是**结论方向不同**：
非零 ⟹ **自动化管道拦得住**（`&&` / CI 会停在 5），真正会栽的是**读摘要的人** ——
`1 skipped` 长得像"这儿没事"，不像"161 条凭空没了"。
复现（仓库外、两个文件、与本仓库内容无关）：目录跑 `= 5`、`--ignore` 掉那个 `importorskip` 文件 `= 0`。

★ **上面那个 `passed` 个数是快照、别照抄**：它随票增长（2026-10-08 记的是 `142 passed`，
2026-10-13 现算 **`166 passed in 13.20s`**）。要一个能引的数就现跑，别从这页里抄。

★ `test_rec.py` 有一条自己盯自己的纪律：**所有会起进程 / 发信号 / 写仓库的路径都被指到 tmp 上**
（`rec.BAG_ROOT`、`rec.STATE_FILE`、`rec.find_recorders`、`rec.process_alive`）。这不是洁癖 ——
**现场可能正录着**，而收口的判据是"给那个 pid 发 SIGINT"，指错地方就会掐掉真录制。
