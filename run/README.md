# `run/` —— z0 的**人工启动层**

**本目录只有一个给人工用的入口**：把「起链那几条命令 + 每步的核对 + 两条容易忘的前提」
收成一张菜单。它**不含任何飞行逻辑**，也**不替代** [`bench-runbook.md`](../docs/control/bench-runbook.md)
（那张表是**判据与记录**：做一遍、填读数、勾框；本脚本只是把命令送出去、把读数摊开）。

```
run/z0.py                # 交互菜单：敲数字
run/z0.py --list         # 只列菜单（不进交互）
run/z0.py env            # 环境自检（只读）
run/z0.py check          # 起链核对（只读，约 5 s 一张表）
run/z0.py up             # ★ 一键起链（第 10 项；run/up.sh 是它的一行别名）
run/z0.py --dry-run up   # 把顺序、每条命令、每步判据全列出来，**一条都不起**
run/z0.py --dry-run mocap   # 只打印将要执行的命令，**不执行**
run/z0.py mavros|vrpn|mocap|traj|control|bringup|fly   # 单条起（前台）
```

★ `--dry-run` 对**所有**动作都是"只打印、不执行" —— `env` / `check` 这两条只读动作**也不例外**
（它们没有命令可打印，会明说一句"只读、本进程里直接跑、`--dry-run` 不执行"）。
`--dry-run mavros` 还会把"起之前会问哪几句"一并列出来，免得现场不知道有 `gcs_url` 这条可选参数。

`env` / `check` 在没 `source` 过的 shell 里会**自己带 source 重新 exec 一次**（`Z0_SOURCED=1` 防重入）；
起链的每条命令也各自带 `source` 前缀 —— 顺序与 `bench-runbook.md` 的「起链（顺序不能反）」一致：

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
| 8 | 起 bringup | 前台 | 一键起 mocap+traj+control；同样要确认飞机摆好；再问一句"已经单独起过 vrpn 了吗"（y ⟹ `use_vrpn:=false`）；其余参数见 [`bringup/README.md`](../src/bringup/README.md) |
| 9 | 操作台 `fly.py` | 前台 | 选轨迹 1/2/3（**起飞前选好**） |
| 10 | ★ **一键起链** | 本进程编排 + 前台守着 | 把上面 3→5→4→6→7 五步**按序**起齐（`bench-runbook.md` 的「起链（顺序不能反）」），每步**等判据成立**再加下一步；起完打印第 2 项那张核对表。详见下节 |

## ★ 一键起链（第 10 项 / `run/up.sh`）

```bash
run/up.sh          # 等价于 run/z0.py up
```

把 5 个终端收成 1 个：**mavros（带 GCS 桥）→ vrpn_listener → mocap → traj + control**，
**逐段等它真的起来**再走下一步，最后打印上面第 2 项那张核对表，然后**前台守着**。

| 步 | 起什么 | 起之前 | 起来的判据（**等**，不是"起了就算"） |
|---|---|---|---|
| 1 | `mavros` + **`gcs_url` 桥给 QGC** | 串口被谁占着；**已经有 mavros 在跑（`connected=True`）就问一句"沿用还是照样再起"** | `/mavros/state` 的 **`connected=True`**（≤60 s）★ **有帧 ≠ 连上飞控** |
| 2 | `vrpn_listener` | — | `/vrpn/<刚体>/pose` 有帧（≤30 s；`Z0`/`z0` 都认） |
| 3 | `mocap` | ★ **要你确认"飞机已摆好、静止、固定？"** | `/mocap/pose_enu` 有帧（≤30 s） |
| 4 | `traj` | — | **不等**（没 `select` 就一帧不发，等它等于等到天荒地老） |
| 5 | `control` | — | `/control/state` 有帧（≤30 s） |
| 6 | — | — | 打印核对表 ⟹ 前台守着（**Ctrl-C 一起停**） |

- **日志**：`run/logs/<时间戳>/01-mavros.log … 05-control.log`（已在 `.gitignore` 里）。
  某步等超时、或**子进程半路自己退了**，就把那个日志的**最后几行**打出来 —— 卡住了先看它。
- **Ctrl-C 一起停**：子进程各自成组（`start_new_session`）⟹ 停的时候**整组一起**停
  （先 SIGINT、超时才 SIGKILL）。★ 前台那个 `Ctrl-C` **到不了**子进程（它们自成会话），
  必须由本脚本转发 —— 所以**别 `kill -9` 本脚本**：`SIGKILL` 不可捕获，转发就没了，
  会**留下 mavros 占着串口**。
- **它不代做"还差的那两步"**（见下节）：选轨迹（`fly.py` 敲 1/2/3）与起飞（**遥控器开始键**）——
  `up` 起完把这两步**打印出来提醒**，但不做。

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

★ **一键起链（第 10 项）不新开一条"起链用的命令表"**：`up` 的动作表项 `cmd=None`（本进程里编排），
它起的每一条都**只写 key**（`UP_CHAIN`），命令取自上面那张动作表 ⟹ 下面这套守卫**不用改就盖住它**。

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
| 表里某话题"在图里但一帧都没来" | 两种可能：发布者没起，或 **QoS 不兼容**（后者**不报错**，见 [`bench/README.md`](../src/bench/README.md) 的「安全边界」） |
| `ROS_DOMAIN_ID=73` | 那是**台架夹具域**（[`bench/pytest_support.py`](../src/bench/bench/pytest_support.py) 默认，且**硬拒 0**）⟹ 读数不是真飞机；核对表第一行会标出来 |
| `test_z0_check_live.py` 的负控用例忽然红了 | ★ **别同时跑 `colcon test`** —— 两者同在夹具域 73，`control` 的用例会把 `/control/state`、`/mavros/setpoint_raw/local` 真发出帧来，负控（"只有订阅者的话题"）随即失真。第一手：2026-09-23 撞到 2 例假失败，colcon 跑完再跑就绿 |

## 测试

```bash
python3 -m pytest run/tests/test_z0_menu.py -q          # 纯函数 + 命令行拼装 + 安全不变式 + 一键起链编排（不需要 ROS，40 例）
python3 -m pytest run/tests/test_flight_runbook.py -q   # ★ 现场那一页的守卫：菜单号/命令/话题/degrade token 都钉在代码上（不需要 ROS，5 例）

# ★ 正控：对着台架夹具（假 mavros）跑一遍 check，读数必须真的读到东西（5 例）
source /opt/ros/humble/setup.bash && source install/setup.bash
python3 -m pytest run/tests/test_z0_check_live.py -q
```

`run/` **不是 colcon 包** ⟹ 不进 `colcon test`；上面三条是它自己的入口。
（★ 夹具的域号由 `bench/pytest_support.py` 定，默认 73 —— 那条纪律的理由见它的文件头。）

⚠️ **要一次跑整个 `run/tests/`，必须先 source**（上面那两行）。没 source 时
`test_z0_check_live.py` 会在**模块级** `importorskip` 上跳掉，而 pytest 6.2.5 在那种情形下
**连这一整个目录的其余用例一起收集不到**（第一手：未 source 跑 `pytest run/tests -q` ⟹
`1 skipped`、0 例；source 后同样一条命令 ⟹ **`50 passed`**）——
★ 所以"跑整个目录是绿的"这句**只有在 source 过的环境里才成立**，未 source 时那是**空收集**，不是绿。
