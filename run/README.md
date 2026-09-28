# `run/` —— z0 的**人工启动层**

**本目录只有一个给人工用的入口**：把「起链那几条命令 + 每步的核对 + 两条容易忘的前提」
收成一张菜单。它**不含任何飞行逻辑**，也**不替代** [`bench-runbook.md`](../docs/control/bench-runbook.md)
（那张表是**判据与记录**：做一遍、填读数、勾框；本脚本只是把命令送出去、把读数摊开）。

```
run/z0.py                # 交互菜单：敲数字
run/z0.py --list         # 只列菜单（不进交互）
run/z0.py env            # 环境自检（只读）
run/z0.py check          # 起链核对（只读，约 5 s 一张表）
run/z0.py up             # ★ 一键启动全部（第 9 项；run/up.sh 是它的一行别名）
run/z0.py up_manual      # ★ 一键启动动捕+mavros（第 8 项）—— 手动遥控飞
run/z0.py down           # ★ 清除后台节点（第 10 项；run/down.sh 是它的一行别名）
run/down.sh --dry-run    # 只扫一眼"会停谁"，**一个信号都不发**
run/z0.py --dry-run up   # 把顺序、每条命令、每步判据全列出来，**一条都不起**
run/z0.py --dry-run mocap   # 只打印将要执行的命令，**不执行**
run/z0.py mavros|vrpn|mocap|traj|control   # 单条起（前台）
run/z0.py bringup|fly    # 两条**不进菜单**的（见下）；直接点名仍可跑
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
| 8 | ★ **一键启动（动捕+mavros，手动遥控飞）** | 本进程编排 + 前台守着 | mavros → vrpn → mocap **按序**起齐、每步等判据；★ **不起 `traj`/`control`** ⟹ 没有任何程序在上位控制飞机，起完**直接用遥控器手动起飞**。详见下节 |
| 9 | ★ **一键启动全部（选轨迹 → 按开始键）** | 本进程编排 + 前台守着 | 把 3→4→5→6→7 五步**按序**起齐（`bench-runbook.md` 的「起链（顺序不能反）」），每步**等判据成立**；起完把**终端让给操作台 `fly.py`** 选轨迹，然后打印核对表 + 起飞提示。详见下节 |
| 10 | ★ **清除后台节点** | 本进程编排 | 与第 9 项相反的**另一头**：**按名字扫**（`DOWN_TARGETS`）在跑的节点 ⟹ 先摊开清单、问一句 y/N，再按 SIGINT→SIGTERM→SIGKILL 停；停完**再扫一遍**并报串口状态。详见下下节 |

★ **两条不进菜单**（`menu=False`：不显示、不占号，但命令仍在动作表里 ⟹ 安全不变式一条没松）：

| key | 谁在用 | 位置 |
|---|---|---|
| `bringup` | `z0.launch.py` 一键起 mocap+traj+control —— `DOWN_TARGETS` 要从它取包名 | 照 [`bringup/README.md`](../src/bringup/README.md) 手打，或 `run/z0.py bringup`（会问一句 `use_vrpn:=false`） |
| `fly` | 操作台 —— 第 9 项起齐后**前台**跑它选轨迹（`UP_HANDOFF`） | 第 9 项里自动起；想单独再跑：`run/z0.py fly` |

## ★ 一键（第 8 / 9 项，`run/up.sh` = 第 9 项）

```bash
run/up.sh              # 等价于 run/z0.py up（第 9 项：全部程序 + 选轨迹）
run/z0.py up_manual    # 第 8 项：动捕 + mavros，手动遥控飞
```

**第 9 项**把 5 个终端收成 1 个：**mavros（带 GCS 桥）→ vrpn_listener → mocap → traj + control**，
**逐段等它真的起来**再走下一步，**然后把终端让给操作台选轨迹**，最后打印上面第 2 项那张核对表 +
起飞提示，**前台守着**。**第 8 项**就是同一张表的 **1–3 步**（到 `mocap` 为止，不起 `traj`/`control`）。

| 步 | 起什么 | 起之前 | 起来的判据（**等**，不是"起了就算"） |
|---|---|---|---|
| 1 | `mavros` + **`gcs_url` 桥给 QGC** | 串口被谁占着；**已经有 mavros 在跑（`connected=True`）就问一句"沿用还是照样再起"** | `/mavros/state` 的 **`connected=True`**（≤60 s）★ **有帧 ≠ 连上飞控** |
| 2 | `vrpn_listener` | — | `/vrpn/<刚体>/pose` 有帧（≤30 s；`Z0`/`z0` 都认） |
| 3 | `mocap` | ★ **要你确认"飞机已摆好、静止、固定？"** | `/mocap/pose_enu` 有帧（≤30 s） |
| 4 | `traj`（**只有第 9 项**） | — | **不等**（没 `select` 就一帧不发，等它等于等到天荒地老） |
| 5 | `control`（**只有第 9 项**） | — | `/control/state` 有帧（≤30 s） |
| 6 | **终端让给 `fly.py`**（**只有第 9 项**，前台） | — | 你敲 1/2/3 选轨迹（**起飞前选好**），选完按它的提示退出 |
| 7 | — | — | 打印核对表 + **起飞提示**（开始键按两下、压住 ≥0.3 s）⟹ 前台守着（**Ctrl-C 一起停**） |

- **日志**：`run/logs/<时间戳>-up/01-mavros.log … 05-control.log`（第 8 项是 `…-up_manual/`，只有 3 个；
  已在 `.gitignore` 里）。某步等超时、或**子进程半路自己退了**，就把那个日志的**最后几行**打出来。
- **Ctrl-C 一起停**：子进程各自成组（`start_new_session`）⟹ 停的时候**整组一起**停
  （先 SIGINT、超时才 SIGKILL）。★ 前台那个 `Ctrl-C` **到不了**子进程（它们自成会话），
  必须由本脚本转发 —— 所以**别 `kill -9` 本脚本**：`SIGKILL` 不可捕获，转发就没了，
  会**留下 mavros 占着串口**。
- ⚠️ **第 6 步把终端让出去那一段**：那时前台跑的是 `fly.py`，`Ctrl-C` 归它；脚本的后台子进程仍在，
  你退出 `fly.py` 之后脚本接着守着（`Ctrl-C` 又回到"一起停"）。
- **起飞那一下不代做**（见下节）：脚本只把"按开始键哪两下、压多久"写在屏上。

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
（本进程里编排），它们起的每一条都**只写 key**（`UP_CHAIN` / `MANUAL_CHAIN`，第 9 项另有 `UP_HANDOFF` 把 `fly` 放前台），
命令取自上面那张动作表 ⟹ 下面这套守卫**不用改就盖住它们**。

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
python3 -m pytest run/tests/test_z0_menu.py -q          # 纯函数 + 命令行拼装 + 安全不变式 + 两条一键/清除编排（不需要 ROS，64 例）
python3 -m pytest run/tests/test_operation_manual.py -q   # ★ 现场那一页的守卫：菜单号/命令/话题/degrade token 都钉在代码上（不需要 ROS，5 例）

# ★ 正控：对着台架夹具（假 mavros）跑一遍 check，读数必须真的读到东西（5 例）
source /opt/ros/humble/setup.bash && source install/setup.bash
python3 -m pytest run/tests/test_z0_check_live.py -q
```

`run/` **不是 colcon 包** ⟹ 不进 `colcon test`；上面三条是它自己的入口。
（★ 夹具的域号由 `bench/pytest_support.py` 定，默认 73 —— 那条纪律的理由见它的文件头。）

⚠️ **要一次跑整个 `run/tests/`，必须先 source**（上面那两行）。没 source 时
`test_z0_check_live.py` 会在**模块级** `importorskip` 上跳掉，而 pytest 6.2.5 在那种情形下
**连这一整个目录的其余用例一起收集不到**（第一手：未 source 跑 `pytest run/tests -q` ⟹
`1 skipped`、0 例；source 后同样一条命令 ⟹ **`74 passed`**）——
★ 所以"跑整个目录是绿的"这句**只有在 source 过的环境里才成立**，未 source 时那是**空收集**，不是绿。
