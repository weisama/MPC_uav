# 22: 常用启动收成一个脚本 —— `run/z0.py` 交互菜单

Status: ready-for-human
Type: task

> ⏳ **2026-09-23：脚本 + 两条测试已完成并全绿 ⟹ 交回人手里**（`Status: ready-for-human`）。
> **§三的 5 条判据全部达成**（证据在 §六）；**§五 那一项**（在真机/真飞控上把"起链"那几个动作
> 真的跑一遍）**agent 做不了** —— 起它们要占飞控串口、要人在场摆飞机 ⟹ 照实留着，**不假装已过**。
> 按 [`triage-labels.md`](../../../docs/agents/triage-labels.md)，`done` 是**操作者的裁定**（同 05/08/09 的做法）。
>
> ★★ **2026-09-23 追加：菜单加了第 10 项「一键起链」**（`run/z0.py up` / `run/up.sh`）——
> 5 个终端收成 1 个，**顺序起 + 每步等判据 + 起完给核对表**。见 **§七**（范围、边界、判据、
> 三条拍板决策都在那儿）；它**不代做**"选轨迹 / 起飞"那两步，§二 那条边界**一条没动**。

**What to build:** 把「起链」那几条**天天要手打**的命令收进仓库根的一个入口：

```
run/z0.py            # 交互菜单（不认识的直接敲数字）
run/up.sh            # ★ 一键起链（= 菜单第 10 项，见 §七）
```

**为什么单开一张**：Phase 1 的启动知识现在**散在四处**，每处都只讲自己那一段 ——

| 出处 | 讲什么 |
|---|---|
| [`bench-runbook.md`](../../../docs/control/bench-runbook.md) 的「起链（顺序不能反）」 | 三条 `source` + mavros/vrpn/mocap 三连 + 每条的核对命令 |
| [`src/bringup/README.md`](../../../src/bringup/README.md) | 一键 bringup / `fly.py` / `use_mavros` 为什么默认 `false` |
| [`z0.launch.py`](../../../src/bringup/launch/z0.launch.py) 头注释 | 「飞机必须先摆好再起 `mocap`」这条**唯一对顺序敏感**的事 |
| [`mocap-setup.md`](../../../docs/reference/mocap-setup.md) / 各包 README | 串口定案、刚体名、`colcon build` 的包选择 |

⟹ 现场每换一个终端就要重打一遍，而**错法有代价**：忘了 `source ~/mavros_ws` 会得到
"launch 坏了"的假象；起 `mocap` 之前搬过飞机 ⟹ 阶段 A 原点漂了却没人知道（`jump_warn_m` 只是告警）。

## 一、范围（做什么）

`run/` 目录（仓库根新建），**一个给人工用的交互菜单**，加一份 README：

| 菜单项 | 做什么 | 形态 |
|---|---|---|
| 环境自检 | 三条 `setup.bash` 在不在、`ros2` 能不能跑、`install/` 里有没有那 6 个包、飞控串口在不在/被谁占、`ROS_DOMAIN_ID`、当前 commit | 只读 |
| 起 mavros | `ros2 launch mavros px4.launch fcu_url:=/dev/ttyTHS1:921600`（可选 `gcs_url` 桥给 QGC） | 前台阻塞 |
| 起 vrpn_listener | `ros2 launch vrpn_listener sync_entity_state.launch` | 前台阻塞 |
| 起 mocap | `ros2 launch mocap mocap.launch.py`（★ 起之前要求人工确认"飞机已摆好、静止"） | 前台阻塞 |
| 起 traj / 起 control | 各自单独起（台架 T3 就是这么起的） | 前台阻塞 |
| 起 bringup | `ros2 launch bringup z0.launch.py`（可选 `use_vrpn:=false`） | 前台阻塞 |
| 操作台 | `ros2 run bringup fly.py` | 前台阻塞 |
| 起链核对 | 一张表：每个话题**有没有帧、多少 Hz**，`/mavros/state` 的 `connected/armed/mode`、`/control/state` 的原文 | 只读 |
| ★ 一键起链（第 10 项） | `mavros`（带 GCS 桥）→ `vrpn_listener` → `mocap` → `traj` + `control`，**逐段等它真的起来**再加下一步；起完自动给上面那张核对表 | 本进程编排（前台守着，Ctrl-C 一起停）—— 详见 **§七** |

★ 每个动作**自己带 `source` 前缀**（`bash -c 'source … && <命令>'`）⟹ 操作者不必再手打三条 `source`；
`check` 这一项还要在**自己的进程里** `import rclpy`，所以脚本在发现 `rclpy` 不在时会
**用 `bash -c` 带 source 重新 exec 自己一次**（`Z0_SOURCED=1` 防重入）。

## 二、★ 不做什么（安全边界，有测试盯着）

**本脚本不提供任何"让飞机动"的入口。** 具体地，动作表里**不得**出现：

- `ros2 service call`（含 `/control/takeoff`、`/mavros/cmd/arming`、`/mavros/set_mode`）
- `ros2 param set`（改参数）
- `ros2 topic pub`（发任何帧）

理由不是洁癖，是仓库里已经写死的两条设计：
① [`z0.launch.py`](../../../src/bringup/launch/z0.launch.py) 的「本包**不给**"程序起飞"的入口，
因为它会绕开"开始键之前不发控制指令"这条防线」；② [`bench-runbook.md`](../../../docs/control/bench-runbook.md)
的「起完之后**还差两步**（刻意不自动化）」。
⟹ 台架上的 `/control/takeoff` 那条服务调用**照旧手打**（`bench-runbook.md` T4 里就是原文），
脚本只把它当作**提示**打印出来，不给一键。

★ 这条由 `run/tests/test_z0_menu.py` 的 `test_动作表里没有任何会动飞机的入口` 与
`test_源码里不出现发布/服务调用` 两条用例盯住（对源码本身做静态检查）。

## 三、判据（★ 5 条全部达成，证据见 §六）

1. `run/z0.py --list` 打印菜单；`run/z0.py --dry-run <动作>` 只打印命令、不执行。
2. **只读动作在"什么都没起"的机器上不炸**：`env` / `check` 逐条报"缺什么"，不是抛异常。
3. **正控**：在台架夹具（假 mavros，域 73）上跑 `check`，读数必须**真的读到**那几条 ——
   用例 `run/tests/test_z0_check_live.py`（含**负控**：只有订阅者的话题必须被点名）。
4. 安全不变式两条全绿（动作表 + 源码 AST）。
5. `run/README.md` 写清"它替你做掉什么、不做什么、和 `bench-runbook.md` 谁先谁后"。

## 四、★ 两条新查到的事实（做这个脚本的直接动因，都写进了 `run/README.md` 的排障表）

### 4.1 `ros2` CLI 的 daemon 会让读数**静默为空**

- 台架夹具强设 `ROS_LOCALHOST_ONLY=1`（[`bench/pytest_support.py`](../../../src/bench/bench/pytest_support.py)）；
- 同一个域（73）上，若 `ros2` daemon **先前**是被不带该变量的进程拉起来的，
  之后 `ros2 topic list` **只回 `/rosout` 与 `/parameter_events`** —— 夹具明明活着、话题明明在发，
  **不报错、不提示**，看上去就像"什么都没起"（第一手：2026-09-23 本机）。
- 同域的 daemon 换环境**不会自己重启**（加 `ROS_LOCALHOST_ONLY=1` 仍然空），`ros2 daemon stop` 之后才看到全表。

⟹ **本脚本的图与数据探测全部走自己进程里的 rclpy 节点，一步都不经过 CLI 的 daemon**
（初版曾打算用 `ros2 topic list --no-daemon --spin-time 3`；`--no-daemon` 确实是
`topic list` / `node list` 的**子命令参数**（`ros2 --no-daemon …` 会报 `unrecognized arguments`），
但既然探测已经在本进程里，就没有理由再 fork 一个 CLI）。CLI 的那条路留在 README 的排障表里
给**手工排查**用。

### 4.2 本机存在**半拉子 ROS 环境**：「能 import rclpy」≠「source 过」

`PATH`/`PYTHONPATH` 里有 `/opt/ros/humble`，但 `AMENT_PREFIX_PATH` 里没有两个工作空间 ——
此时"缺 rclpy 才重入"的判据不成立，自检会报**六个包全查不到**，而同一台机器上
`ros2 launch control …` 本来跑得起来 ⟹ **一个会撒谎的自检比没有更坏**。
⟹ 判据改成**环境 source 过没有**（`Z0_SOURCED`），并且 `env` 表里把
「**装好了但没 source**」与「**真没 build**」分开报（这两句话对应两种完全不同的动作）。

## 五、现场待做（agent 做不了的部分）

**"起链"那几个动作没在真机/真飞控上跑过** —— 起它们要占飞控串口、要人在场摆飞机。
★ §七 的**一键起链整条**也在这条残留里（它起的正是这几条；编排逻辑有用例，**真串口那一段没有**）。
⟹ 下一次台架/飞行日**照着菜单走一遍**（起 mavros → vrpn → mocap → bringup → 核对），
把与 [`bench-runbook.md`](../../../docs/control/bench-runbook.md) 那几行的**差异**回填到本票 Comments。
★ `check` 的**读数与判读**已经在台架夹具上验过（§三.3）⟹ "命令写错 / 读数解析错"这一类不在残留里；
残留的是**真机才有的那一段**（串口、VRPN 主机、EKF2 的 `vision_pose` 链路）。

## 六、证据（2026-09-23，agent）

| 判据 | 命令 | 结果 |
|---|---|---|
| 1 + 4 | `python3 -m pytest run/tests/test_z0_menu.py -q` | **26 passed**（当时；★ §七 之后同一个文件是 **40 passed**）（纯函数 + 命令行拼装 + 两条安全不变式；不需要 ROS）。★ 复核后补了 `--list` / `--dry-run`（含"只读动作也不执行"）的用例 —— 原来这两条判据**一条用例都没有**，§六却把它们记进了"已达成" |
| 3 | `python3 -m pytest run/tests/test_z0_check_live.py -q`（对着夹具，域 73） | **5 passed** ★ 2026-09-23 §七 落地后**复跑仍 5 passed**（`Probe` 那次重构没碰它） |
| 2 | 空机器：`python3 run/z0.py check`（域 0，什么都没起） | 不抛异常；7 条话题逐条给判据，`/traj/reference` 记 `·`（本该为空），其余 6 条进「缺什么」并写明**「话题都不在图里 ⟹ 发布者没起」** |
| 3（现场读数） | 夹具在跑时：`ROS_DOMAIN_ID=73 ROS_LOCALHOST_ONLY=1 python3 run/z0.py check` | `✓ /mavros/state 4 帧 / 4 s ≈ 1 Hz connected=True armed=False mode=POSCTL`、`✓ /mocap/pose_enu 801 帧 / 4 s ≈ 200 Hz`、`✓ /mavros/local_position/pose 121 帧 / 4 s ≈ 30 Hz`；负控两条：`/control/state`（**在图里但一帧都没来**）与 `/mavros/setpoint_raw/local`（同）、`/mavros/vision_pose/pose`（**话题都不在图里**）都被点名；首行标出"台架夹具域，不是真飞机" |
| 5 | `run/README.md` | 菜单表、安全边界、核对判据的来源表、**排障表**（daemon 坑 / 半拉子环境 / 刚体名大小写 / 串口占用 / 「在图里但没帧」的两种成因）、两条测试的跑法 |

★ 一处**第一手更正**（写进 `test_z0_check_live.py` 的注释）：夹具在 `dut=None` 时
**订**了 `/control/state` 与 `/mavros/setpoint_raw/local`（`Scenario._on_control_state`、`FakeMavros`
的接收端）⟹ 这两个话题**出现在图里**却一帧不发布。判读必须把
「在图里但没有发布者」与「话题都不在图里」分开说 —— 前者是"节点没起"，
后者是"这一段链路根本不存在"。初版把两者混成一句"没数据"，**是这条现场读数把它逼出来的**。

## 七、★ 追加：一键起链（菜单第 10 项，2026-09-23）

```
run/z0.py up        # 或 run/up.sh（一行别名）
```

**为什么并进本票**（原先另开了一张 23，操作者 2026-09-23 拍定**并回来**）：它就是本票那张菜单的
**第 10 项**，用同一张动作表、同一份核对表——另立一票只会让"哪个入口是权威"变得含糊。

**它替现场做掉的**：5 个终端收成 1 个（[`bench-runbook.md`](../../../docs/control/bench-runbook.md)
那一节标题就叫「起链（**顺序不能反**）」），并且每步**等判据成立**再走下一步：

| 步 | 起什么 | 起之前 | ★ 起来的判据（**等**，不是"起了就算"） |
|---|---|---|---|
| 1 | `mavros`（+ **`gcs_url` 桥给 QGC**） | 串口被谁占着；**若已有 mavros 在跑（`connected=True`）就问"沿用还是照样再起"** | `/mavros/state` 的 **`connected=True`**（≤60 s）★ 有帧 ≠ 连上飞控 |
| 2 | `vrpn_listener` | — | `/vrpn/<刚体>/pose` 有帧（≤30 s，刚体名 `Z0`/`z0` 都认） |
| 3 | `mocap` | ★ **要人确认"飞机已摆好、静止、固定？"** | `/mocap/pose_enu` 有帧（≤30 s） |
| 4 | `traj` | — | **不等**（没被 `select` 就一帧不发，等它=等到天荒地老） |
| 5 | `control` | — | `/control/state` 有帧（≤30 s） |
| 6 | — | — | 打印 §一 那张**核对表**，然后前台守着（**Ctrl-C 一起停**） |

每步的输出进日志（`run/logs/<时间戳>/0N-<动作>.log`）—— 判读靠它们（`control` 的固定 token、
mavros 的串口报错）；某步等超时或**半路自己退了**，就把那个日志的**最后几行**打出来。
★ 子进程各自成组（`start_new_session`）⟹ 停的时候**整组一起**（先 SIGINT、超时才 SIGKILL）——
**绝不能留下 mavros 占着串口**。

**★ 不代做的两步**（三条拍板决策之一：**全留给人**）：选轨迹（`fly.py` 敲 1/2/3）与起飞
（**遥控器开始键**）。理由与 §二 同：[`z0.launch.py`](../../../src/bringup/launch/z0.launch.py)
的「本包**不给**"程序起飞"的入口，因为它会绕开"开始键之前不发控制指令"这条防线」。

**三条决策**（2026-09-23 操作者三问三答）：①自动化边界 = **全留给人**；
②"mavros 并广播" = **`gcs_url:=udp-b://0.0.0.0:14555@:14550`**（bench-runbook P0 与
[`mocap-setup.md`](../reference/mocap-setup.md) §5.4 那条）；③形态 = **并进本脚本且顺序起** + `run/up.sh` 别名。

**判据**（追加）：

1. `run/z0.py up` 进菜单第 10 项、`run/up.sh` 可跑；`--dry-run up` 把**顺序、每步命令、判据、
   要确认的话**全摊开，**一条都不起**。
2. 顺序与判据是**数据**不是散落的 `if`：`UP_CHAIN` 每项都在 `ACTION_BY_KEY` 里，
   且每项都有一条 `STEP_READY`（纯函数，喂一张 snapshot 就出答案）。
3. ★ `up` 起的命令**全部取自 `ACTIONS`**（它只写 key）⟹ §二 那条不变式**天然盖住**它们，
   不必为新代码另写一套守卫。
4. 编排可测：换掉 `start_child` / `wait_for_step` / `watch_children` 之后，用例断言
   **起哪几条、什么顺序、超时怎么办、Ctrl-C 只停自己起的那些**。

**证据**（2026-09-23，agent）：`python3 -m pytest run/tests/test_z0_menu.py -q` → **40 passed**
（其中一键起链 14 条：顺序与动作表一致 / 每步有判据或明说不等 / mavros 判据是 `connected` 不是有帧 /
刚体名不做归一化 / `--dry-run up` 一条不起 / 按序起 5 条 / 已有 mavros 时沿用且收尾不停它 /
某步超时打日志尾巴并停掉其余 / 子进程半路退了不当超时 / 守着的报错 / 串口被占先问一句 /
日志尾巴取最后几行）。★ **现场那条**（真串口、真飞控）与 §五 同一条残留，agent 做不了。

另跑了两条**只看不动的**（2026-09-23）：`python3 run/z0.py --dry-run up` 把 5 条命令、每步判据、
要确认的话全列出来、**一条没起**（顺带印出 mavros 那条**确实带了 `gcs_url` 桥**）；
`bash -n run/up.sh` 语法 OK；`python3 run/z0.py --list` 第 10 项就是它。
`run/tests/test_z0_check_live.py -q`（夹具域 73）**复跑仍 5 passed**。

## Comments

- **2026-09-23（agent，立项）**：`Status: ready-for-agent`。范围与形态是操作者拍的：
  **仓库根新建 `run/`**、**起链 + 每步核对**、**一个交互菜单脚本**（2026-09-23 三问三答）。
  相关：[`bringup/README.md`](../../../src/bringup/README.md)（一键 bringup 的现有入口）、
  [`bench-runbook.md`](../../../docs/control/bench-runbook.md)（人工清单，本脚本不替代它）、
  [`09`](09-track-mission-script.md)（`fly.py` 操作台，同样是"给人工用"的脚本）。
- **2026-09-23（agent，落地并收口）**：`Status: ready-for-human`。交付
  `run/z0.py`（菜单 8 个动作 + `env` + `check`）、`run/README.md`、
  `run/tests/`（18 + 5 例）；并在 [`README.md`](../../../README.md) 的目录图、
  [`bringup/README.md`](../../../src/bringup/README.md)、
  [`bench-runbook.md`](../../../docs/control/bench-runbook.md) 的「起链」各加了一行指针。
  ★ **`/control/takeoff` 没有被包进来**（§二）：台架那条服务照旧手打，脚本只打印"在哪"。
  ★ 与 [`09`](09-track-mission-script.md) 的分工：`fly.py` 是**飞行中的操作台**（选轨迹、看状态），
  本脚本是**起飞前的启动层**（起链、核对）—— 两者都不发速度指令、都不解锁。
- **2026-09-23（agent，`/code-review` 复核后修正）**：两轴审查各报了若干条，**逐条回代码核实**后处置如下
  （★ 按仓库纪律，驳回的也写清理由，不闷掉）：

  **核实为真、已改**
  1. ☠️ [`bench-runbook.md`](../../../docs/control/bench-runbook.md) 那条指针把菜单号写错成「**9= 起链核对**」——
     9 是 `fly.py` 操作台，起链核对是 **2**（`ACTIONS` 顺序 `z0.py:329`；`run/README.md` 的菜单表）。
     **照它敲会起到操作台**。已改。
  2. ☠️ `--dry-run` 对 `env`/`check` **不生效**（`dispatch` 里只读分支在 `dry_run` 之前命中）⟹
     §三.1 那句"只打印命令、**不执行**"名不副实（`--dry-run check` 会真起一个 5 s 探测）。
     已改成：只读动作遇 `--dry-run` 明说一句并不执行。
  3. `describe_message` 的文件注释宣称覆盖 `PositionTarget`，代码却只认 `.pose.position` ——
     第一手核对：`mavros_msgs/msg/PositionTarget.msg:28` 是 `geometry_msgs/Point position`（**顶层**）
     ⟹ `/mavros/setpoint_raw/local` 那一行只会印类名。已补 `.position` 分支 + 一条用例。
  4. §一 写的两条**可选参数**（mavros 的 `gcs_url`、bringup 的 `use_vrpn:=false`）先前**没有出口**。
     已做成起之前问一句 y/N（`OptionalArg`，值写在动作表里）；★ 有意**不做成"随手输一段参数"** ——
     那等于把任意 shell 片段送进 `bash -c`，与 §二 相冲。守卫扫描也随之扩到可选参数的取值。
  5. `--list` / `--dry-run` 两条判据**零用例**却记进 §六（见上表）—— 已补。
  6. 串口三态在 `render_snapshot` 与 `render_env` 里各写了一遍（措辞还不一样）⟹ 抽成 `serial_line()`；
     `parse_lsof_fpcf` 里收 `-Fpcf` **不产出**的 `a` 标签、以及"把格式化好的 `python3(32607)` 再
     split 回 pid"这种把自己写出去的东西读回来 —— 一起去掉；`menu_loop` 里
     `or choice in ("env","check")` 恒真（`ACTION_BY_KEY` 本就有这两条）⟹ 去掉。
  7. `bringup/README.md` 的指针文字写 `run/z0.py`、链接却指 `run/README.md` —— 已对齐。

  **核实后驳回（不采纳）**
  - 「`run/README.md` 说走 AST 就只剩"拼字符串绕过"这一条路 —— 而那条路**本来就没堵**」：
    **成立，但结论相反** —— 所以改的是**措辞**不是代码：写明这一层的威胁模型是
    **"手滑新增一条看得见的命令"**，不是"刻意规避"。要拦刻意规避得上更重的手段，
    与"给人工用的小菜单"不相称。
  - 「`cmd_env`/`cmd_check` 是 Middle Man」：**部分成立** —— 只删掉了 `dispatch` 里那次重复的
    `ensure_sourced()`（两条 `cmd_*` 自己会调），两个函数保留：它们是"只读动作"的类型化出口。
  - 「`sourced_command(setups=…)` 与 `render_snapshot(dict)` 的抽象是为测试而设」：**保留** ——
    `setups=` 是 §三.5"缺失的 setup 不能被写进命令"那条判据的**唯一**入口，
    去掉它就只能靠真的删掉机器上的 `/opt/ros`。
- **2026-09-23（agent，追加 §七 一键起链）**：操作者要"一条命令把动捕 + mavros（并广播）+ 轨迹 +
  控制都起来，起完能直接飞"。三问三答拍定 **①自动化边界 = 全留给人**、**②"并广播" = `gcs_url` 桥给
  QGC**、**③形态 = 并进本脚本且顺序起**。★ 随后明示 **"不要另开 tickets，在 ticket 22 就解决"** ——
  原先单开的 `23-one-key-startup.md` **已删除**，内容并入本票 **§七**（这就是"另立一票会让
  '哪个入口是权威'变含糊"的原因：`up` 用的就是本票那张动作表与核对表）。
  落地：`ACTIONS` 第 10 项 `up`（`cmd=None`，本进程编排）、`UP_CHAIN`/`STEP_READY`/`STEP_TIMEOUT_S`、
  `Child`/`start_child`/`stop_children`/`wait_for_step`/`watch_children`/`cmd_up`、
  `run/up.sh` 别名、`run/logs/` 进 `.gitignore`；`run/README.md` 与
  [`bench-runbook.md`](../../../docs/control/bench-runbook.md) 的指针补上第 10 项。
  ★ **`up` 只写 key、命令全取自 `ACTIONS`** ⟹ §二 那条不变式（动作表扫描 + AST 扫描）
  **无需改动就盖住了新代码**——这也是把它并进本票而不是另开票的实质理由之一。
