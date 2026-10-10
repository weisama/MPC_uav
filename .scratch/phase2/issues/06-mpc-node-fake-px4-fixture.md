# 06: MPC（模型预测控制）节点骨架 + 假 PX4 夹具（seam 1）

Status: done
Type: task

**What to build:** 节点第一次完整成型，**以及 Phase 2 / Phase 4 所有节点级测试的公共底座** —— 一个假 PX4 + 脚本化动捕输入。

⚠️ **为什么夹具必须是假 PX4 而不是 SITL**：兜底路径（求解失败、非有限值、时间戳倒退）需要"**这一帧求解失败**"这种要求 —— **真 PX4 没法被要求，假件可以**。所以夹具必须能**在指定时刻注入故障**，这不是加分项，是它存在的理由。

**Blocked by:** [04](04-offline-ocp.md)

**本张负责的改动（M7 / M8 / M9）**：M7 状态源从 PX4 话题换成动捕 ENU 话题（**因此可删掉输入侧的坐标系转换**）、M8 删掉为绕过消息版本问题而写的双订阅 hack（`_v1` 后缀话题 / `msg/versioned/` 是 **v1.16 才有**的机制，在 **z0 真身（v1.14.3 系）**里**根本不存在**）、M9 补上定义了却没使用的死变量。

> ☠️ **2026-09-23 措辞更正**：M8 原写"在 **v1.15 世代的 fork** 里根本不存在" —— **结论本身不变**（真身里同样不存在），但**对象选错了**：那是拿 fork 当参照物。真身的 `msg/` 与**上游 v1.14.3 逐字节相同**，所以这句话更贴对象的说法是"**v1.16 才有**"。见 [`refs-manifest.md`](../../../docs/reference/refs-manifest.md) §2.6 执行版。

- [x] 订阅 `/mocap/pose_enu`（**不是** NED）与 `/fmu/out/vehicle_status`（判断是否已 armed / offboard）
      —— `mpc_node.py:244-247`；`test_mpc_node.py::test_接口与_QoS_与票面一致`（夹具的动捕源与状态话题上各数到订阅者）
- [x] 发布 `/fmu/in/vehicle_rates_setpoint` 与 `/fmu/in/offboard_control_mode`（`body_rate = True`，其余 False）
      —— `test_闸门关闭时只发心跳_一帧_setpoint_都没有` 逐位断言另外五个布尔位为 False
- [x] **QoS**：pub `BEST_EFFORT + TRANSIENT_LOCAL + depth 1`，**sub 也必须显式 `BEST_EFFORT`**
      —— ⚠️ `depth` / `history` 在本套栈的发现信息里**读不回来**（见 Comments 的栈事实那条），
      守法是"常量在单测里逐字段对 + 黑盒只断言读得到的 `reliability`/`durability`"
- [x] 时间戳用**微秒** —— `test_时间戳是墙钟微秒`（夹具按"相对墙钟的比值"把 µs/ms/ns 分开）
- [x] 心跳 ≥2 Hz、实做 50 Hz；100 Hz 定时器（频率在 [09](09-sitl-tier-a-closed-loop.md) 实测最坏耗时后降档）
      —— 实测 **50.0 Hz / 100.0 Hz**（`test_实做频率_心跳_50_Hz_setpoint_100_Hz`）；降档决定仍归 09/11
- [x] **符号映射**：给定姿态与目标，断言三轴角速率的**相对符号**与 [03](03-sign-map-derivation.md) 一致
      —— `test_目标在正前方时_pitch_为负_且顶约束表现为打满`（+x 目标 ⟹ `ω_y > 0` ⟹ PX4 的 `pitch < 0`，实测 −3.8400）
- [x] **推力归一化**：给定状态与目标，断言 `thrust_body[2]` 的**数值与符号**；常数**从模型导出**而非硬编码（M2）
      —— 期望值**从 `Z0Params` 算**（不写死 0.6235）；位置保持实测 −0.6235 = `−hover_thrust`
- [x] **未解锁 / 未在 offboard 时 `不发布`**
      —— ⚠️ 读法是"不发 **setpoint**"，心跳**无条件发**（否则进不去 OFFBOARD，见 Comments）
- [x] 约束饱和时表现为**打满**，而不是被静默截断 —— 节点里**刻意没有 `clip()`**；
      `abs(abs(pitch) − PITCH_RATE_MAX) < 0.05`（对"被谁悄悄缩回去"也是红的）
- [x] **状态外推** `x + v·τ`（τ = 50 ms）一行实现；不做完整延迟补偿
      —— ⚠️ 端到端**量不了**（输入里没有 `v`，见 [15](15-mocap-velocity-source.md)）⟹ 只有 `test_node_core` 的两条 +
      一条把前提钉住的说明性用例
- [x] RViz：预测轨迹与参考点发布，能肉眼判断"模型预测控制是否在发疯"
      —— `test_RViz_的预测轨迹与参考点`（51 点 = `N+1`、起点 ≈ 当前位置、终点朝目标、参考点落在测量位置 + 偏移上）
- [x] **回退**：本张之后 Phase 1 的 `(A)` 级链仍完整可用，一条参数命令即可切回 —— Phase 2 是**加一条支路，不是替换**
      —— 本票**没有动** `offboard_smoke.py`、没有改任何 Phase 1 话题；`(A)` 链的 `ros2 run nmpc offboard_smoke.py`
      与它的三条取证开关原样可用（交接见 Comments）
- [x] **软急停接口参数化**（通道号不写死）—— ★ 通道号的**决定**由 [`.scratch/phase1/issues/02`](../../phase1/issues/02-rc-channel-decision.md) 产出；**本张不决定、不实测**（RC 夺权与失效保护的实机实测归 Phase 4）。
      ✅ **2026-09-21 该 ticket 已 resolved**，结论里有一条**改变本张前提**：**急停 = ch8 高值 = PX4 硬件杀车，程序不读急停通道**，即**没有"软急停通道"** ⟹ 本张的"软急停接口"要按 [`phase2-implementation.md`](../../../docs/control/phase2-implementation.md) §4 那张表里的**两条既有路径**（打杆接管 / ch6 模式开关）来理解，别去假设有一个专属通道
      —— 落地为 `node_core.StopConfig` 的四个条件（**全是参数、零通道号**），见
      [`mpc-node-seam1.md`](../../../docs/control/mpc-node-seam1.md) §6
- [x] 夹具能在 `colcon test` 里跑起来且至少一个冒烟场景通过；**能按指定时刻注入故障**（为 [07](07-solver-failure-fallback.md) 准备好）
      —— `bench/fake_px4.py`（注入面：`mute` / `set_armed` / `set_nav_state` / `set_pre_flight_checks_pass` + 脚本化动捕）；
      `test_fake_px4_selftest.py` **16 条**（已在 `bench/CMakeLists.txt` 注册成 `test_fake_px4_selftest`
      ⟹ **`colcon test` 真会跑它**）；seam 1 的 **14 条**集成测试全过
- [x] **不需飞机的静态自检**：一个在装桨之前就能跑的检查流程，使参数与符号错误在这一步暴露
      —— `nmpc/preflight_check.py`（快档 20 项 < 1 s / `--solve` 23 项实测 8.4 s；退出码 1 = 不过）；
      自己的测试 `test_preflight_check.py` **19 条**，**每条检查都配了负控**
- [x] M7 / M8 / M9 的改动编号、理由与第一手依据已记入 [`phase2-implementation.md`](../../../docs/control/phase2-implementation.md)
      —— §2.5 的第二张落地状态表；§4 的 S4 清单已逐条勾选

---

★ **M7 顺带干掉一整类 bug**：既然动捕原生就是 ENU/FLU、MPC（模型预测控制）内部也是 ENU/FLU，那么参考实现在输入侧那段手写 NED→ENU 四元数变换（**整个文件里最容易写错的一行**）可以直接删掉。本张删掉它，而不是把它留着"以后可能有用"。

## Comments

### 2026-09-28 包结构决定（**由操作者在本轮对话中裁定**，05 已按此建包）

[`sign-map.md`](../../../docs/control/sign-map.md) §8.1 把「实现在哪个包」甩给了本张
（原文："包名由 ticket 06 决定 —— 本票不替它定包结构"）。但 [05](05-sitl-offboard-smoke.md)
的验收清单要求「stub 与后续节点**共用同一份**符号映射实现」，所以 05 一动手就必然先把包建出来。
**故这两个决定在本轮提前做了**（操作者二选一的原文见下），本张**其余内容一概未动**：

| 决定 | 取值 | 备选与代价 |
|---|---|---|
| Phase 2 的包名 | **`nmpc`** | 备选 `mpc`（与 `MPC_uav` / `mpc-roadmap.md` 等既有标识符一致，但 `ros2 run mpc …` 会在行文里出现裸词 `MPC`，与 [`CONTEXT.md`](../../../CONTEXT.md) 首节的书写约定摩擦）；`ctbr`（用项目术语命名，但盖不住包里的 OCP / 模型 / 求解器） |
| 假 PX4 夹具落点 | **`bench` 包**（`bench/fake_px4.py`，本张实现） | 备选 `nmpc/testing/fake_px4.py`（把 `px4_msgs` 这个**仓库外**依赖锁在一个包里）。选 `bench` 的理由：它的 [`package.xml`](../../../src/bench/package.xml) 与 [README](../../../src/bench/README.md) 已自称「Phase 1/2/3 所有节点级测试的公共底座」，且那张「假什么 / 不假什么」表的 **「PX4 的行为」**一栏目前是空的 —— 补上它正是本张的活 |

**已按此建出的骨架**（05 建的，本张在此之上补节点与夹具）：

```
src/nmpc/                 ← Python 包（ament_cmake + ament_cmake_python，与 bench 同形态）
├── nmpc/sign_map.py      ← ★ 唯一一份 FLU→FRD 映射实现（纯函数，零 ROS import）—— sign-map.md §8.1
├── nmpc/commands.py      ← 「参数化指令程序」纯函数（时间 → [F, ω]），无任何模型 / 代价 / 约束
├── nmpc/offboard_smoke.py← 05 的 stub 节点
└── test/ · README.md
```

⚠️ **本张要处理的两处新增约束**（05 踩到、留给本张，别当成没看见）：

1. **`px4_msgs` 在仓库外**（`~/px4_msgs_ws`，下位工作区）。本包与 `bench` 都依赖它 ⟹
   仓库根的裸 `colcon build` 从此需要先 `source ~/px4_msgs_ws/install/setup.bash`；
   这一条要写进 [`README.md`](../../../README.md) 的构建段（**05 未动那个文件**）。
2. **05 的 stub 已经带上了「刻意错」的取证开关**（`--break {thrust-sign,thrust-scale,pitch-sign}`、
   `--timestamp-unit {us,ms,ns}`、`--qos {best_effort,reliable}`）——
   本张的 seam 1 夹具要把这些开关**收进夹具侧**（夹具负责注入故障），
   而不是继续留在 DUT 里。理由：夹具是"外部世界"，DUT 不该为了被测而自带故障注入器。
   05 里那样写是因为它**没有夹具**（取舍写在 05 的 Comments 与交付文档里）。

### 2026-09-28 收口 —— 交付物、第一手数字与交接

**交付物**（本票碰过的**全部**路径；`offboard_smoke.py` 与 Phase 1 的任何文件都没动）：

| 路径 | 是什么 |
|---|---|
| `src/nmpc/nmpc/mpc_node.py` | 节点本体（话题面 / 参数 / 收尾摘要） |
| `src/nmpc/nmpc/node_core.py` | 零 ROS、零求解器的那一半（闸门 / 外推 / 参考 / 停机条件） |
| `src/nmpc/nmpc/preflight_check.py` | 装桨前的静态自检（本票验收项之一） |
| `src/nmpc/test/test_mpc_node.py` | **seam 1** 集成测试 **14 条**（起真的 DUT 子进程） |
| `src/nmpc/test/test_preflight_check.py` | 自检脚本的测试 19 条（**每组检查都有负控**） |
| `src/bench/bench/fake_px4.py` + `src/bench/test/test_fake_px4_selftest.py` | 假 PX4 夹具 + 自检 16 条 |
| `src/bench/bench/{scenario,config,timeline}.py` + `src/bench/config/bench.yaml` | 夹具接线（起假 PX4 / 动捕发哪条 / 四个"按时刻注入"助手）—— 复核轮补进表里 |
| `src/bench/CMakeLists.txt` | 把夹具自检**注册进 `colcon test`**（复核轮补的，见下） |
| `src/nmpc/CMakeLists.txt` · 两个 `package.xml` | 三个 `install(PROGRAMS …)`、6 个 `ament_add_pytest_test`、`px4_msgs` 与 `bench` 的依赖声明 |
| `src/nmpc/nmpc/ocp.py` | `last_prediction()` + `_solves`（RViz 那条预测路径要读它） |
| `src/nmpc/test/conftest.py` | session 级 ROS 上下文（本票新增的 fixture） |
| `CONTEXT.md` · `README.md` · `docs/control/phase2-implementation.md` | 术语「假 PX4」登记、构建段（先 source `~/px4_msgs_ws`）、M7–M9 落地表 |
| `.scratch/phase2/issues/15-mocap-velocity-source.md` | 本票发现的"动捕没有速度字段"另立一票 |

跑法（`px4_msgs` 在仓库外，构建段见仓库根 `README.md`）与实测：seam 1 的 **14 条**全过（多数用例各起一次 DUT，
启动 ~8 s，见下）；夹具自检 16 条全过（`colcon test` 里）；自检脚本快档 20 项 < 1 s、`--solve` 23 项 8.4 s。

**三处票面读法，先记下来免得后来人重推**：

1. **心跳（`offboard_control_mode`）必须无条件发。** 票面"未解锁 / 未在 offboard 时**不发布**"
   读的是**不发 setpoint**：`offboardCheck.cpp:40-65` 每轮先置 `offboard_control_signal_lost = true`、
   只有收到新鲜心跳才清零，`modeCheck.cpp:126-134` 在该标志为真时把 OFFBOARD 删出"可运行" ⟹
   闸住心跳就是**死锁**（不发心跳 ⟹ 进不去 OFFBOARD ⟹ 闸门永不开）。留给 07 的接缝是参数
   `heartbeat_always`（默认 `True`）。★ 另记一条：**"不发 setpoint" ≠ "松手"** —— PX4 判失效看心跳
   （`COM_OF_LOSS_T`），松手之后飞控走它自己的失效动作，那部分兜底也归 07。
2. **饱和判据是"打满"**（`|pitch|` 顶在 `PITCH_RATE_MAX` 上，容差 0.05），所以节点里**刻意没有
   `clip()`** —— 有的话"顶约束"与"被悄悄缩回去"在话题上长得一样。
3. **`depth 1` 这条在本套栈上黑盒不可观测**（下面第 4 条），守法是"常量单测 + 只断言读得到的字段"。

**关于 05 那三个取证开关（`--break` / `--timestamp-unit` / `--qos`）**：按操作者裁定**没有动
`offboard_smoke.py`**。理由与交接：那些开关是 05 在**没有夹具**时的取证手段；夹具到位之后，
**故障注入归夹具侧**（夹具是"外部世界"：`mute` / `set_armed` / `set_nav_state` / 脚本化动捕，
能按指定时刻注入）—— DUT 自带故障注入器是"为了被测而改被测"，不该长期留着。但**删它是 05 的事**，
本票只点名交接，不越界。

**本票新查出来的两条栈级事实（都会让后来人写出恒红的断言，已各钉一个会红的用例）**：

- **`depth` / `history` 在 DDS 发现信息里读不回来**：`history` 是 `UNKNOWN`、`depth` **恒为 0**，
  与发布者实际写的值无关（`depth=1` 与 `depth=10` 读回来一样）。
- **`TRANSIENT_LOCAL` 的历史重放不发生**：晚加入的订阅者匹配得上、收得到**新**帧，但收不到任何**旧**帧。
  所以"用 `depth` 语义间接验证"这条路也不通。

**台架纪律（都是踩出来的，已写进 `test_mpc_node.py` 的模块 docstring 与 seam 1 文档 §8）**：

- **别并发跑、别把 DUT 落在后台**：话题名（`/fmu/in/*`）与 ROS 域（默认 73）都是固定的，残留进程会与
  用例**同时**发；两者还共用 `$MPC_CODEGEN_DIR`，同时判"要重新生成"会撞出与代码无关的
  `RuntimeError: Rendering of model.in.h failed!`。收工前 `pgrep -af mpc_node`；不要用 `pytest -n`。
- **DUT 启动要 ~8 s**，且 `$MPC_CODEGEN_DIR` 里的生成文件**每次启动都被重写**（mtime 证据）⟹
  代码生成 + 编 C 每次启动都在发生。**为什么每次都不复用（`is_code_reuse_possible`，`tol_code_reuse=1e-13`）没查** ——
  记为开放项；症状就是每次启动 +8 s、且启动日志里看不到编译器输出。相关票：09/11（启动耗时也吃周期预算）。
- ★★ **"发布者被发现"远早于"节点初始化完成"**：实测 `matched_publishers("mode") >= 1` 在 ~8 s 成立，
  而那一刻节点还在 `OfflineOcp()` 里、启动日志还压在 rcutils 缓冲里。**此时发 SIGINT** 的症状极具误导性：
  **日志只剩开头那行 matplotlib 警告（373 字符）、`returncode == 0`、没有 traceback** ——
  看上去像"收尾摘要没打"，其实节点压根没走到收尾路径（本票为此排查了很久）。
  ⟹ 台架判据必须是"**夹具收到了第一帧心跳**"（帧走 DDS、与日志缓冲无关）。

**交接**（本票没做、明确归别处）：兜底动作 ⟹ [07](07-solver-failure-fallback.md)；
`/traj/reference` ⟹ [09](09-sitl-tier-a-closed-loop.md)；频率降档 ⟹ 09/11；
绑核与实时优先级的**实测**（实现已在节点里）⟹ 11；动捕速度源 ⟹ [15](15-mocap-velocity-source.md)；
τ 的端到端验证要等 15 —— 在此之前 `/mocap/pose_enu` 没有 `twist`，`v` 恒为 0，
**任何 τ 都能通过台架测试**（夹具**不能**偷偷塞带 twist 的 `PoseStamped`：消息类型不匹配就是**不连接**，
那等于夹具在说谎）。

**静态自检的第一版被自己抓了一次**（值得留档）：它原本要求 `StopConfig` 的字段集**恰好等于**
文档那四个停机条件，结果把 `warmup_cycles`（首拍不发，名正言顺的第五个"不发"闸门）误伤成失败 ——
代码是对的、判据太紧。现在改成"四个**都在**"，多出来的闸门字段会列出来让人确认。

### 2026-09-28 复核轮（standards / spec 两轴）—— 逐条回代码核实后的裁定

两轴各一个子代理，**每条都回代码/源码核过**才落地（驳回的也写在下面，附理由）。
**落地 9 条**：

1. **夹具自检的 16 条没进 `colcon test`**（spec 轴，**成立**，而且正好打在本票那条验收项上）：
   `bench/CMakeLists.txt` 原来只注册了 `test_bench_selftest` / `test_timeline` ⟹ 取证用的那 16 条
   **只在手动 `pytest` 里跑过**。已注册 `test_fake_px4_selftest`（`TIMEOUT 180`）。
2. **`use_sim_time` 被我们自己的"未知参数"检查拒掉**（spec 轴，**成立，是本票引入的缺陷**）：
   实测 `ros2 run nmpc mpc_node.py --ros-args -p use_sim_time:=true` → **exit 2**
   （`未知参数：['use_sim_time']`）。09/11 用 launch 起这个节点时会带着它 ⟹ 症状是"节点起不来"、
   而报错指向参数名。已改成"只拒**我们自己**拼错的名字"，放行 `use_sim_time` 与 `qos_overrides.*` 前缀，
   并配了会红的用例（负控拿 `rate_hzz` 拼错，断言 exit 2 + 日志含"未知参数"）。
   ★ 顺带挖出一条**新事实**：`use_sim_time:=true` 而**没人发 `/clock`** 时，rclpy 时钟冻在 0，
   50/100 Hz 定时器**一次都不触发** —— 节点活着、启动日志全打、**一帧都不发**。
   本票第一版用例拿"收到第一帧心跳"当"节点起来了"的判据，白等了 120 s 才超时。
   已写进 seam 1 文档 §2.5（09/11 若在 SITL 用它，**必须有 `/clock` 源**）。
3. **文档写反了 M5 的行为**（spec 轴，**成立**）：`phase2-implementation.md` 原写"求解失败 / 非有限值 ⟹
   不发"，而代码里**退化求解照样发**（只 `warn` + `degraded_count`）。已按**三类行为**逐条订正
   （非有限值 ⟹ 闸门关；求解退化 ⟹ **照发**；映射前非有限 / 负推力 ⟹ 这一拍不发）——
   07 照错的描述接手会以为"已经不发"了。
4. **PX4 引文两处**（spec 轴，**成立**）：seam 1 文档把 `/fmu/out/*` 的 **writer** 写成
   `utilities.hpp:114-119`（那是 **reader** 块），writer 在 `:57-65` 的 datawriter QoS 块 ⟹ 已订正。
   另 `modeCheck.cpp:126-133` 少算一行（`clearCanRunBits` 在 **:134**，那正是"删出可运行"的执法行）
   ⟹ 四份文件里的引文一并改成 `126-134`。
5. **本票自己引进了死字段 `Cycle.pose_enu`**（standards 轴，**成立** —— 讽刺的是票面 M9 就是"删死变量"）：
   写了两处、**零读者**（它与 `x0[:3]` 只差一个 `v·τ`）。已删，并在 `Cycle` 注释里留一句
   "为什么会有人想加它"。
6. **裸词 `MPC` 7 处**（standards 轴，**成立**，违反 `CONTEXT.md` §0 的书写约定）：新增文本里 7 处已改
   （4 处补全称 / 加限定，3 处改述成"它/求解器"）。
7. **「假 PX4」没进术语表**（standards 轴，**成立**）：`CONTEXT.md` 是唯一术语权威、已收"假 mavros /
   假 traj / 假飞机"，本票新造的这个必须登记 ⟹ 已加一行（含注入面、以及"只假话题面不假飞控行为"
   与"假 mavros 不假 PX4 行为"那条口径的互相引用）。
8. **四个 `px4_*` timeline 助手全仓零调用者**（standards 轴的"投机性通用"）：**不删，接进用例** ——
   `_arm_and_offboard` 改走 `Timeline`（"**按指定时刻**注入"这条验收项这才真走通了），
   并新增一条 `px4_status_silence` 的用例：状态静默 > `status_timeout_s` ⟹ setpoint **一帧不发**、
   心跳**照旧 50 Hz**、恢复后重开。`px4_preflight_checks` 仍无读者（节点不看这一位 ⟹ 写不出会红的断言），
   登记进 `bench/README.md` 的「还没被任何测试用过的能力」表。
9. **收口交付表漏了路径**（spec 轴，**成立**）：原表自称"只碰了这些路径"，实际还动了
   `bench/{scenario,timeline,config}.py`、`bench.yaml`、`bench/CMakeLists.txt`、两个 `package.xml`、
   `nmpc/CMakeLists.txt`、`test/conftest.py`、`ocp.py`、`CONTEXT.md`、根 `README.md`、
   `phase2-implementation.md` 与新建的 15 号票 ⟹ 已补全（表在上面）。

10. **本票新增的两个测试文件让"不依赖 ROS"那两条断言在合跑时假红**（复核轮里跑出来的，不在两轴报告里）：
    `test_mpc_node.py`（import `px4_msgs`）与 `test_preflight_check.py`（import `nmpc.mpc_node` ⟹ `rclpy`）
    会先于 `test_node_core.py` / `test_sign_map.py` 跑（文件名序），而那两个"纯模块不许拉 ROS"的断言
    查的是**整个 pytest 进程**的 `sys.modules` ⟹ `pytest src/nmpc/test/` 一跑就红，
    `colcon test`（一个文件一个进程）照样绿 —— **同一份代码两种结论**。
    改法：判据挪进**子进程**（"只 import 那个模块，问它有没有带进 ROS"），问的是同一个契约、与谁先跑无关。
    ⚠️ **动了 `test_sign_map.py` 一处断言的取法**（结论、语义一字未改，上面的注释里写明了原因）——
    这是本票新增文件**触发**的，不是那张票自己的问题。

**驳回 / 只改注释不改行为（5 条）**：

- **"05 的三个取证开关没被收进夹具侧"**（spec 轴）—— **部分驳**。操作者裁定**不碰 `offboard_smoke.py`**，
  且收口段已点名交接给 05。更实质的一点：`--break` / `--qos` 那类开关改的是 **DUT 的行为**，
  而 seam 1 的纪律恰恰是 **DUT 里不许有故障注入器**；在夹具侧能做的、也是唯一正确的对偶物是
  **取证**（`stamp_unit_guess` 读单位、`publisher_qos` 读 QoS、符号与归一化靠**数值断言**），这三样本票都做了。
  要"注入符号错"只能改 DUT 的代码，那不是夹具的活。
- **"绑核与 `rt_priority` 是票 11 的活"**（spec 轴）—— **驳**：操作者的裁定就是"本票实现、11 实测"，
  票面那句"实现已在节点里"说的正是这件事。
- **"`tau_s` / 参数校验在 `mpc_node` 与 `preflight_check` 两处各写一遍"**（standards 轴）—— **驳**：
  自检脚本的价值**就在于它是一条独立的路径**；共用一份实现会让"自检跟着节点一起错"。
- **"`_PARAMS` / `NodeParams` / `declare_params` 三处同改"**（standards 轴）—— **驳**：
  `_PARAMS` 已经把"名字 + 默认值 + 类型"收敛成一份，剩下两处是**显式字段**，刻意不用 `**kwargs`
  （参数面要能一眼读全）。
- **"`px4_arm(…, armed=False)` 名字自相矛盾"**（standards 轴）—— **只改注释不改名**：它是"改状态"
  不是"执行动作"，改名会牵动 Phase 1 已在用的 `px4_*` 命名族；读法写进了 `bench/README.md` 的配方。
- **"07 的头号场景（这一拍求解失败）注入不了"**（spec 轴）—— **部分驳**：`VrpnSource.set_pose` /
  `set_trajectory` 可以喂 **NaN**（非有限值那次闸门关 + `sign_map` 拒绝那次都走这条），`rewind` /
  `pause` / `hold_stamp` 给时间戳路线，都是 07 要的场景；"让 `solve()` 返回非零状态"这条**不经状态输入**
  就只能在 DUT 里造假输入，接缝留给 07 定（本票不替它决定）。
