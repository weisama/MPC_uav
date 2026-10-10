# MPC（模型预测控制）节点与 seam 1 —— 话题面上发生了什么（ticket 06）

> 上游：[`phase2-implementation.md`](phase2-implementation.md) §4 的 **S4**（节点骨架）与 §2.5 的 **M7–M9**。
> 本页是那两条的**交付文档**：节点在话题面上的**行为**、`seam 1` 这个名字的**边界**、
> 以及**哪些验收项在这一层根本量不了**（那几条不写假测试，写清为什么）。

---

## 0. 先读这一段：三个"一句话"

1. **`seam 1` = "话题上出现了什么"。** 本层的断言只看外部可观测的东西（话题、QoS、时间戳、
   进程退出码），**不 import 节点的类、不读它的成员**。代价：几条验收项在这一层**量不了**
   （§5、§3 各有一条），那几条写成"为什么量不了"的一手结论，**不写成恒绿的假测试**。
2. **心跳（`offboard_control_mode`）必须无条件发。** 票面那条"未解锁 / 未在 offboard 时
   **不发布**"读的是**不发指令（setpoint）**；把心跳也闸掉就是**死锁**：不发心跳 ⟹ 进不去
   OFFBOARD ⟹ 闸门永不开。依据见 §2.2（源码路径 + 行号）。
3. **饱和必须表现为"打满"**（`|pitch|` 顶在 `PITCH_RATE_MAX` 上），节点里**刻意没有 `clip()`**
   —— 否则"它在顶约束"与"它被谁悄悄按比例缩回去"在话题上长得一样。

---

## 1. 交付物一览

| 文件 | 是什么 |
|---|---|
| `src/nmpc/nmpc/mpc_node.py` | 节点本体：话题面（订阅/发布/QoS/定时器）、参数、收尾摘要 |
| `src/nmpc/nmpc/node_core.py` | **零 ROS、零求解器**的那一半：闸门 / 外推 / 参考 / 停机条件（可一毫秒单测） |
| `src/nmpc/nmpc/preflight_check.py` | 装桨之前的**静态自检**（§7）—— 起飞流程的一步，不是测试 |
| `src/bench/bench/fake_px4.py` | 假 PX4 夹具（与 `FakeMavros` 同族的"外部世界"） |
| `src/nmpc/test/test_mpc_node.py` | **seam 1** 集成测试（条数一直在长，**别引数字**：`grep -c '^def test_' src/nmpc/test/test_mpc_node.py`），起真的 DUT 子进程 |
| `src/bench/test/test_fake_px4_selftest.py` | 夹具自检（16 条，**不起 DUT**） |

层次分工（别混，`test_mpc_node.py` 的模块 docstring 里有同一张表）：

| 文件 | 被测的接缝 | 起 DUT 吗 |
|---|---|---|
| `test_node_core.py` | 闸门 / 外推 / 参考（纯逻辑） | 不起 |
| `test_ocp_*.py` | `(状态, 参考) → u`（零 ROS 的求解器） | 不起 |
| **`test_mpc_node.py`** | **话题上出现了什么** | **起** |
| `test_preflight_check.py` | 自检脚本本身的逻辑（每条检查配负控） | 不起 |

跑法（`px4_msgs` 在仓库外，见仓库根 `README.md` 的构建段）：

```bash
source /opt/ros/humble/setup.bash
source ~/px4_msgs_ws/install/setup.bash
source install/setup.bash
export ACADOS_SOURCE_DIR=$HOME/acados
export LD_LIBRARY_PATH=$ACADOS_SOURCE_DIR/lib:$LD_LIBRARY_PATH
python3 -m pytest src/nmpc/test/test_mpc_node.py        # 多数用例各起一次 DUT（启动 ~8 s，见 §8）
```

★ **2026-10-10 起**，上面这条命令跑的是**实机冻结档**（DUT 带 `-p tau_s:=0.0`），
不再是"节点默认档" —— 见 **§11**（ticket [`phase4/07`](../../.scratch/phase4/issues/07-seam1-rerun-real-params.md)）。

---

## 2. 接口面

### 2.1 订阅 / 发布

| 方向 | 话题 | 类型 | QoS | 依据 |
|---|---|---|---|---|
| 订阅 | `/mocap/pose_enu` | `geometry_msgs/PoseStamped` | 可靠 / depth 10 | `mocap_rel_pose_node.cpp:110` 的 `QoS(10)`；**ENU/FLU 原生**（M7） |
| 订阅 | `/fmu/out/vehicle_status` | `px4_msgs/VehicleStatus` | **`BEST_EFFORT`** / depth 1 | PX4 的 writer 是 `TRANSIENT_LOCAL` + best_effort（`utilities.hpp:57-65` 的 datawriter QoS 块；`:113-120` 是 **reader**，别引错） |
| 订阅 | `/traj/reference`（**只有 `-p reference_source:=traj` 时才有这一条**，见 §9 与 [`mpc-node-traj-reference.md`](mpc-node-traj-reference.md)） | `mavros_msgs/PositionTarget` | 可靠 / depth 10 | Phase 1 的 `traj_node.cpp:108-109` 的 `rclcpp::QoS(10)`；**复用 Phase 1 的话题名与类型** ⟹ `traj` 节点零改动（ticket 17） |
| 发布 | `/fmu/in/offboard_control_mode` | `px4_msgs/OffboardControlMode` | `BEST_EFFORT + TRANSIENT_LOCAL` / depth 1 | 照抄 `px4-mpc`（§2.6） |
| 发布 | `/fmu/in/vehicle_rates_setpoint` | `px4_msgs/VehicleRatesSetpoint` | 同上 | 同上 |
| 发布 | `/mpc/predicted_path` | `nav_msgs/Path` | 默认（可靠） | 本仓自己的观测面，**不接飞控** |
| 发布 | `/mpc/reference` | `visualization_msgs/Marker` | 默认 | 同上 |

⚠️ 订阅 `/fmu/out/*` **必须显式 `BEST_EFFORT`**：写成 ROS 2 的默认 `RELIABLE` 就**一个回调都不进**、
**也不报错**（QoS 不兼容不报错，只是永不匹配）—— 唯一的症状是夹具侧的 `matched_subscribers` 数不到它
（`test_接口与_QoS_与票面一致` 断言 `== 1`）。

★ **两条条件订阅，互相正交**（都是"不给参数就不订"，但管的是两件不同的事）：
`state_source=mocap|px4` 决定**测量**那条链订哪一对话题（ticket 09）；
`reference_source=hold|traj` 决定**参考**那条链订不订 `/traj/reference`（ticket 17）。
两个 `if`、不是一个 `else` ⟹ 四种组合都合法（A 档 + 轨迹参考 = 飞控自己的状态 + 轨迹目标）。
两者的理由同源：验收项写的是"本档**不使用**某输入"，而"订阅了但不看"在话题边界上**不可证伪**
—— 条件订阅让它变成一个黑盒可断言的事实（`matched_subscribers()` 的**差值**，见
[`mpc-node-traj-reference.md`](mpc-node-traj-reference.md) §7）。

### 2.2 ★ 心跳**无条件发** —— 票面那条"不发布"的正解

票面：「未解锁 / 未在 offboard 时**不发布**」。这里的**读法是"不发 setpoint"**，心跳照发：

- `offboardCheck.cpp:40-65`：每轮先置 `offboard_control_signal_lost = true`，**只有**收到新鲜的
  `offboard_control_mode`（时间戳在 `hrt_absolute_time() + COM_OF_LOSS_T` 内）且**至少一个布尔位为真**
  才清零。
- `modeCheck.cpp:126-134`：该标志为真时 `clearCanRunBits(mode_req_offboard_signal)` ⟹ OFFBOARD
  从"可运行"里被删掉。

⟹ 把心跳闸在 `nav_state == OFFBOARD` 上会形成**死锁**（不发心跳 ⟹ 进不去 OFFBOARD ⟹ 闸门永不开），
真机上表现为"这飞机根本进不了 offboard"。

★ **"不发 setpoint" ≠ "松手"**：PX4 判失效看的是**心跳**（`COM_OF_LOSS_T`）。所以"不发"之后
  飞控会走它自己的失效动作 —— 那部分兜底归 **ticket 07**；节点侧留的接缝是
  `heartbeat_always`（默认 `True` = 无条件发；置 `False` = 心跳与 setpoint 同生死）。

对应测试：`test_闸门关闭时只发心跳_一帧_setpoint_都没有`（两条方向相反的断言合在一起 ——
心跳**在流**是正控，证明节点活着、动捕与状态都新鲜；setpoint **一帧都没有**是负断言），
配对的正控是 `test_解锁切_OFFBOARD_之后立刻开始发`。

### 2.3 频率（实测）

| 量 | 设计值 | 本机实测（2026-09-28，Orin NX，2 s 窗口） |
|---|---|---|
| 心跳 `offboard_control_mode` | 50 Hz（安全线 ≥2 Hz） | **50.0 Hz** |
| `vehicle_rates_setpoint` | 100 Hz（D6） | **100.0 Hz** |
| RViz 预测轨迹 | `viz_hz` = 10 Hz | ~10 张/秒 |

★ 两条门槛**刻意分开**断言：`≥2 Hz` 是**PX4 的安全线**（`COM_OF_LOSS_T` 的量级，票面原文），
"实测 50/100 Hz"是**设计值有没有真的落在线上** —— 把心跳写成 5 Hz 时前者照样绿。
**降档决定归 ticket 09/11**（票面：频率在实测最坏耗时后再定），所以这里只卡下限。

### 2.4 时间戳

`timestamp` 用**微秒**（墙钟）。单位错在 PX4 侧是**静默**的（`min(timestamp − time_offset, hrt)`
把它钳回"新鲜"，话题上看不出任何异常）⟹ 夹具按"相对**墙钟**的比值"把 µs/ms/ns 分开（差三个数量级）。
测试：`test_时间戳是墙钟微秒`（实测中位年龄 ~0.04 s）。

### 2.5 参数面：拼错就拒，ROS 自带的放行

- `-p 名字:=值` 里的名字**不在**本节点的参数表里 ⟹ **启动即退 exit 2**，消息里列出已知参数。
  理由：rclpy 对"多写一个参数"**完全沉默**（它静静地待着，谁也没读），台架上表现为
  "我改了参数但行为没变"，而所有断言会指向被测逻辑。这道检查排在代码生成**之前** ⟹ 毫秒级退出。
  测试：`test_拼错的参数当场拒_而_use_sim_time_放行`（负控拿 `rate_hzz` 拼错，断言 exit 2 + 日志含"未知参数"）。
- ⚠️ **ROS 自带的参数必须放行**（`use_sim_time`、`qos_overrides.*` 前缀）。第一手（2026-09-28，本机）：
  早期版本把 `-p use_sim_time:=true` 当未知参数 ⟹ **节点起不来**（exit 2），而报错指向参数名、
  不指向"你写的是 ROS 标准参数"。09/11 用 launch 起节点时带上它是常态。
- ⚠️ **`use_sim_time:=true` 而没人发 `/clock` ⟹ 这个节点一帧都不发**（实测：进程活着、启动日志全打出来了、
  50 Hz / 100 Hz 定时器**一次都不触发** —— ROS 时间冻在 0）。所以别拿"收到第一帧心跳"当"节点起来了"的判据
  （本票第一版就是这么白等了 120 s）；09/11 若要在 SITL 用 `use_sim_time`，**必须有 `/clock` 源**。

---

## 3. ★ QoS 的黑盒可观测性（两条栈级事实，都会让后来人写出恒红的断言）

本机（Humble + `rmw_fastrtps`）实测：

1. **DDS 发现信息里的 `history` 是 `UNKNOWN`、`depth` 恒为 0**，与发布者实际写的值**无关**
   （`depth=1` 与 `depth=10` 读回来一模一样）⟹ 票面那条"pub `depth 1`"在**黑盒这一层不可观测**。
   守法：① 发布方 QoS **常量**在单测里逐字段对（`test_发现信息里的_depth_读不回来_所以票面那条只能靠读常量()`，
   对象就在进程内，`depth` 在那里是能读的）；② 黑盒这一层只断言读得到的 `reliability` / `durability`。
2. **`TRANSIENT_LOCAL` 的历史重放不发生**：晚加入的 `TRANSIENT_LOCAL` 订阅者**匹配得上**
   （发布者视角 `get_subscription_count() == 1`）、也收得到**新**帧，但**收不到任何旧帧**
   ⟹ 也拿不到"depth 语义"的间接证据。

两条事实各有一个**会红的用例**钉着（`test_fake_px4_selftest.py::test_发现信息里读不回来的那两项_history_与_depth`
与 `test_mpc_node.py::test_发现信息里的_depth_读不回来_所以票面那条只能靠读常量`），
免得后来人把它当夹具 bug 反复调试。

---

## 4. 符号、推力归一化、饱和

三条都在 seam 1 上量得到（都是话题上的数）：

| 验收项 | 判据 | 实测（本机） |
|---|---|---|
| 符号与 ticket 03 一致 | 目标在 +x 远处 ⟹ 需要 `ω_y > 0`，而 PX4 的 `pitch = −ω_y` ⟹ **`pitch` 为负** | `pitch = −3.8400` |
| 推力归一化**从模型导出** | `thrust_body[2] = −F/T_max`，位置保持时 `F ≈ t_hover` ⟹ `≈ −t_hover/T_max`；期望值**从 `Z0Params` 算**（不写死） | `−0.6235` = `−hover_thrust` |
| 饱和表现为**打满** | 目标在 +x 10 m 外（1 s 时域内不可行）⟹ `abs(abs(pitch) − PITCH_RATE_MAX) < 0.05` | `|pitch| = 3.8400` |

★ **符号与幅值分开判**：符号反了会让 `pitch` 变号，模型解错了只会让幅值不对 ——
两条断言各自盯一个失败模式。

★ **热身（`warmup_cycles`）不是玄学**（`SQP_RTI` 每周期只迭代一次，全新实例的第 0 次 `solve()`
给的是"从出厂迭代点走一步"）。实测（`-p warmup_cycles:=0 -p hold_offset_x:=10`，话题上前四帧）：

```
#1 pitch=-0.0000  thrust_body[2]=-0.6235   ← 出厂迭代点（等于悬停）
#2 pitch=-3.8400  thrust_body[2]=-0.6235   ← 一步就到位（ω_y 打满）
#3 pitch=-3.8400  thrust_body[2]=-0.8483
#4 pitch=-3.8400  thrust_body[2]=-1.0000   ← 推力也顶到 T_max
```

⟹ 热身 0 会让**最需要它动的那一刻朝着悬停走**；默认 5 拍。这条被
`test_没热身时第一帧是出厂迭代点_热身之后就不再是` 端到端钉住（它的正控是第二帧）。

---

## 5. 状态外推 `x + v·τ`：本层**量不了**，原因不是夹具不够好

> ☠️ **【2026-09-29 作废 —— 完整说明在本节末的「★ 2026-09-29 更新」】**
> 本节**标题这句「本层量不了」以及下面整段结论已被推翻**（[ticket 15](../../.scratch/phase2/issues/15-mocap-velocity-source.md) 落地）：
> 动捕这一档现在**也有速度**了（独立话题 `/mocap/velocity_enu`）⟹ τ 在 B 档上**可黑盒验证**，
> 已有用例 `test_B_档_速度从_velocity_enu_进外推_表现为推力低于悬停`。
> ⚠️ 标记放在**节首**而不是只在节尾：**排序就是可见性** —— 从节首读进来的人必须先知道它作废，
> 而不是读完一整段才在末尾看到一句"上面作废"。下面原文**保留**（它是"为什么会走到这一步"的第一手推导）。

票面要求"`x + v·τ`（τ = 50 ms）一行实现；不做完整延迟补偿"。**这一条在 seam 1 上无法黑盒验证**：

- `node_core.cycle()` 的外推逻辑本身有单测（`test_node_core.py::test_外推只动位置且正好是_v_乘_tau` 等两条）；
- 但**端到端**做不了：`/mocap/pose_enu` 是 `geometry_msgs/PoseStamped`
  （`mocap_rel_pose_node.cpp:110`），而 `PoseStamped` **没有** `twist` 字段 ⟹ 节点侧的 `v` 每拍
  都退化成 0（"缺字段"那条路**一直在走**）⟹ `x + v·τ` 在地面上恒等于恒等变换，**任何** τ 都能通过台架测试。
- 速度其实一直在 `/vrpn/<刚体>/twist`（`vrpn_listener.cpp:46-65`），只是没人把它接到 Phase 2 这一侧
  ⟹ [issue 15](../../.scratch/phase2/issues/15-mocap-velocity-source.md)。
- ⚠️ 夹具**不能**偷偷塞一个带 twist 的 `PoseStamped`：ROS 2 里消息类型不匹配就是**不连接**，
  那会让夹具变成在说谎。

测试 `test_动捕那条话题没有速度字段_所以拍不出_tau_的黑盒测试` 把**前提**钉住：哪天上游换成带
twist 的类型，它会红，提醒来补这条端到端测试。

> ### ★ 2026-09-29 更新（[ticket 15](../../.scratch/phase2/issues/15-mocap-velocity-source.md) 已落地）
>
> 上面这一节的**结论已作废**（推导保留 —— 它是"为什么会走到这一步"的第一手记录）：
> 速度现在走**独立的一条话题** `/mocap/velocity_enu`（`mocap` 节点把 `/vrpn/<刚体>/twist`
> **原样** republish，`geometry_msgs/TwistStamped`，与 pose **同一个 frame** ⟹ 零变换）。
> 节点订它、`node_core.NodeCore._mocap_velocity` 用它当 10 状态里的 `v` ⟹ `x + v·τ` 在 B 档
> 上**不再**恒等变换。B 档的黑盒用例：
> `test_mpc_node.py::test_B_档_速度从_velocity_enu_进外推_表现为推力低于悬停`。
> ⚠️ **仍然**不能把速度塞进 `PoseStamped`（上面那两条 `hasattr` 断言仍守着这一点）。

---

## 6. 停机条件：四个条件 + **零通道号**（软急停接口的落地）

票面「软急停接口参数化（通道号不写死）」的**前提已被上游改动**
（[`phase1/issues/02`](../../.scratch/phase1/issues/02-rc-channel-decision.md) 已 resolved）：
**急停 = ch8 高值 = PX4 硬件杀车，程序不读急停通道** ⟹ 不存在"软急停通道"。
所以本票按 `phase2-implementation.md` §4 那张表的**两条既有路径**（打杆接管 / ch6 模式开关）落地 ——
它们在节点这一侧的**唯一表现都是"离开 OFFBOARD"**。

节点侧的四个停机条件（全部是参数，**没有一个通道号**）：

| 条件 | 默认 | 为什么 |
|---|---|---|
| 动捕静默 | `mocap_timeout_s = 0.5` | 动捕是 ~100 Hz 的上游，0.5 s 静默说明源没了 |
| 状态静默 | `status_timeout_s = 0.75` | 上游是 2 Hz ⟹ 时限必须 ≥ 一个周期（0.5 s） |
| 离开 OFFBOARD | `stop_on_nav_state_exit = True` | 打杆接管 / ch6 切模式在这里的**唯一**表现 |
| 掉 armed | `stop_on_armed_drop = True` | 飞控已经不认这条指令了，不能还在发 |

外加第五个**闸门**字段 `warmup_cycles`（首拍不发，§4）—— 它属于"不发 setpoint 的条件"，
但**不是**停机条件。★ 这个区分是**自检脚本逼出来的**：`preflight_check.py` 的第一版要求
`StopConfig` 的字段集**恰好等于**那四个，结果把 `warmup_cycles` 误伤成失败；现在的判据是
"四个**都在**"，多出来的字段会被列出来让人确认（`test_停机条件的字段名会被对账`）。

---

## 7. 装桨之前的**静态自检**

```bash
ros2 run nmpc preflight_check.py                     # 快档：24 项，< 1 s
ros2 run nmpc preflight_check.py --solve             # 再加求解器那一档：27 项，实测 ≈9 s
ros2 run nmpc preflight_check.py -p mass:=0.82 -p hover_thrust:=0.55   # 与节点同一套 -p 写法
```

★ **项数只当参考、别当契约**（2026-10-10 现跑更正：原写"20 项 / 23 项"）：Phase 2 之后自检加过检查，
两个数都会随判据增删而变 —— 判"能不能装桨"看**退出码**（`0` = 全过），不看项数。
⚠️ 现场那条流程还要带 `-p tau_s:=0.0`（不带则按节点默认 `0.05` 判），见
[`first-flight-config.md`](first-flight-config.md) §5③ —— 上面三行是**用法示例**，不是现场命令。

它回答的是**"我现在这一套参数能不能装上桨"**（测试回答的是"实现有没有被改坏"）
⟹ ① 按**运行参数**跑；② 结论打成清单；③ 任何一条不过就**退出码 1**（能直接串进起飞流程）。

| 组 | 查什么 | 怎么红（负控在 `test_preflight_check.py` 里） |
|---|---|---|
| 参数自洽 | 质量 / 悬停余量 / `T_max` 按**电池**（不是电机上限 40 N）/ 心跳 ≥2 Hz / τ / 热身 / 状态时限 / 两条安全停机 | `hover_thrust:=1.2`、`heartbeat_hz:=1`、`tau_s:=0.5`、`warmup_cycles:=0`、关掉安全停机 |
| **符号** | **三方对账**：实现 / `docs/control/sign-map.md` 的 canonical 块 / `tools/sign-map` 的独立推导 | 把 `pitch` 的符号翻过来（两条对账都红） |
| 归一化 | `−F/T_max`、悬停值 = `−hover_thrust`、打满 = −1、**换 `T_max` 归一化跟着走** | 归一化常数被硬编码（参考实现那个 `0.07`） |
| 停机条件 | 那四个字段都在、**零通道号**（`ast` 扫标识符与字符串，**注释里引用 `ch8` 不算**） | 把 `ch8 = read_rc()` 写进代码 |
| 求解器（`--solve`） | 悬停点解一次：`F ≈ m·g`、`ω ≈ 0`、**发出去的归一化推力 = `−hover_thrust`** | —— |

★ **两处设计取舍，都写死在代码里**：

- **找不到仓库根（`docs/control/sign-map.md`）= 失败，不是跳过。** 静默跳过等于给人虚假的
  安全感；`install/` 下的副本离仓库根远，那时用 `--repo <仓库根>`（本机实测：从 `/tmp` 跑也能
  沿 `__file__` 往上找到仓库根，因为 `install/` 就在仓库里）。
- **`--solve` 那一档把模型 + 代价 + 符号 + 归一化串成**一个数**：在悬停点解出来的 `u` 归一化
  之后必须等于 `−hover_thrust`（实测 `−0.62350`，`10` 次热身让 `F = 7.357500 N = m·g`）。
  这一档需要 acados，所以单列 —— 快档在没有 acados 的机器上也能跑。

---

## 8. 台架纪律（**都是踩过的**）

1. **别并发跑、别把 DUT 落在后台。** DUT 的话题名是**全固定**的（`/fmu/in/*`），ROS 域默认 73 也是
   固定的 ⟹ 任何一个"上次没杀干净的 `mpc_node`"都会和用例**同时**往同两条话题上发（症状：帧数对不上/
   频率翻倍），还共用 `$MPC_CODEGEN_DIR`（两个进程同时判"要重新生成"会撞出与代码无关的
   `RuntimeError: Rendering of model.in.h failed!`）。收工前 `pgrep -af mpc_node` 看一眼；
   **不要用 `pytest -n`（xdist）**跑这个文件。
2. **DUT 的启动要 ~8 s**（本机）：期间 `$MPC_CODEGEN_DIR` 里的生成文件**每次启动都被重写**
   （mtime 证据）⟹ 代码生成 + 编 C 在**每次**启动里都发生，不是只有第一次。日志里看不到编译器输出。
3. ★★ **"发布者被发现"远早于"节点初始化完成"。** 实测 `matched_publishers("mode") >= 1` 在 ~8 s
   就成立，而那一刻节点还在 `OfflineOcp()` 里，启动日志还压在 rcutils 的缓冲里。**此时给它发 SIGINT**
   的症状极具误导性：**日志只剩开头那行 matplotlib 警告（373 字符）、`returncode == 0`、
   没有 traceback** —— 看上去像"收尾摘要没打出来"，其实节点压根没走到收尾路径。
   ⟹ 台架的判据是**"夹具收到了第一帧心跳"**（帧走 DDS、不走 stdout，与日志缓冲无关；
   而 50 Hz 心跳定时器只有 `__init__` 全跑完才会发第一帧）。`test_mpc_node.py::_wait_dut_up` 就是这么写的。
4. **读 DUT 日志要等进程退出 + 读线程 join 完**（`scenario.py` 的 `stop_dut` 里有这条修复）——
   少那一句，调用方紧跟的 `dut_log()` 可能只拿到**前半段**（就是上面第 3 条那个 373 字符的症状之一）。

---

## 9. 交接清单（本票**没做**、明确归别处的）

| 事项 | 归哪 |
|---|---|
| 求解失败 / 非有限值的**兜底动作**（谁接管、怎么降级） | ✅ **2026-09-28 已由 [ticket 07](../../.scratch/phase2/issues/07-solver-failure-fallback.md) 关闭** —— 三档阶梯（`hold` 10 拍 → 安全悬停）、发布前全量校验、失败事件的 `error` 上报 + JSONL `fallback` 列，见 [`mpc-node-fallback.md`](mpc-node-fallback.md)。★ 那条交接的措辞（"`heartbeat_always` 是留给它的接缝"）**已作废**：心跳照旧无条件发，兜底换的是 **rates setpoint 的内容**（"不发"在飞控侧 = 沿用陈旧命令，见该文 §1.1） |
| 真正的轨迹接口 `/traj/reference`（本票只有参数化参考点偏移） | ✅ **2026-09-30 已由 [ticket 17](../../.scratch/phase2/issues/17-traj-reference-interface.md) 关闭** —— `-p reference_source:=traj` 订阅 Phase 1 那条话题（同类型同话题名 ⟹ `traj` 零改动），目标点 = **锚点 + 参考给的相对量**、参考自己的速度进时域外推、超时**冻结**。口径、判据与"没证明什么"见 [`mpc-node-traj-reference.md`](mpc-node-traj-reference.md)。⚠️ 中间那句"等操作者裁定归哪张票"**已经作废**：[ticket 09](../../.scratch/phase2/issues/09-sitl-tier-a-closed-loop.md) 的 Comments #7 当场就**另开了 [ticket 17](../../.scratch/phase2/issues/17-traj-reference-interface.md)** 承载这条交接 —— 缺的不是"归哪张票"，而是那张票的 `Status` 升级（2026-09-30 操作者授权开工并推标签）。★ **仍未做的**是"三条轨迹真的飞完" —— 那归 ticket 11 的 §S5 判据② |
| 100 Hz 要不要降档（实测最坏耗时 → D6/回填） | ✅ **2026-09-28 已由 [ticket 09](../../.scratch/phase2/issues/09-sitl-tier-a-closed-loop.md) 回答**：空机稳态最坏 **3.995 ms** ⟹ **100 Hz 保档**（冷启动 15.956 ms 落在 `warmup` 闸门里）。⚠️ 新加一条前置条件"**机上别跑别的东西**"（同二进制多几个进程 ⟹ 11.297 ms ⟹ 50 Hz 档）。读数见 [`tier-a-sitl-hover.md`](tier-a-sitl-hover.md) §5，**整周期**（含订阅/发布、状态转换、日志）仍归 ticket 11 |
| 绑核 + 可选实时优先级的**实测**（实现已在节点里：`cpu` / `rt_priority`） | ticket 11 |
| 动捕速度源（`v` 从哪来 ⟹ τ 才可能端到端验证） | [issue 15](../../.scratch/phase2/issues/15-mocap-velocity-source.md) |
| 05 那个 stub 的**取证开关**（`--break` / `--timestamp-unit` / `--qos`） | **刻意没动** `offboard_smoke.py`：那些开关在 05 里是"没有夹具时的取证手段"，夹具到位之后**故障注入归夹具侧**（夹具是"外部世界"：`mute` / `set_armed` / `set_nav_state` / 脚本化动捕）。DUT 自带故障注入器是"为了被测而改被测"，不该继续留着 —— 但**删它是 05 的事**，本票只点名交接 |
| 仓库根 `README.md` 的构建段（`px4_msgs` 在仓库外） | ticket 06 已补（05 未动那个文件） |

## 10. 本页能证明 / 不能证明

**能证明**：话题名与 QoS（读得到的那些字段）、心跳与 setpoint 的频率、闸门开合、符号的正负号、
推力归一化的**数值**、饱和打满、时间戳单位、收尾路径不打 traceback、RViz 两条观测面。
**不能证明**：`thrust_body` 的符号在**真 PX4** 上会被怎么解释（那是 SITL 闭环 / seam 3 的事，
本票的符号判据只对着 `sign-map.md` 那张权威表）；`depth` / `history`（§3）；
τ 的端到端正确性（§5）；**飞起来之后**的任何动态品质（那是 S5 之后的事）。

★ **2026-09-28 补（ticket 09）**：这里面**有一条已经被回答了**，引用时请引对地方 ——
"`thrust_body` 的符号在真 PX4 上会被怎么解释"：A 档闭环已用姿态真值锚出**方向正确**
（⑨a 残差 +0.082 rad），见 [`tier-a-sitl-hover.md`](tier-a-sitl-hover.md) §6。
本页自己的界限**一条没变**（它仍是"离线 + 假 PX4"的证据）。另，**节点侧有一处已改**：
本票当时喂 `u_ref = 上一帧解`（走 M6 §3.2 的缺省），在闭环里是正反馈，ticket 09 改喂
**参考自己的解析前馈** —— 同一票 Comments #3。

---

## 11. Phase 4：按**首飞冻结档**重跑（ticket [`phase4/07`](../../.scratch/phase4/issues/07-seam1-rerun-real-params.md)）

> 本节是 **seam 1 的第三次应用**（形态 A：喂脚本化输入 + 断言发出话题 + 假 peer）。第一次是
> Phase 1（[`phase1/spec.md`](../../.scratch/phase1/spec.md) 的 Testing Decisions 把它定为基准），
> 第二次是**本页 §1** 那一批。**没有新增任何 seam 形态** —— 同一条缝隙、同一组用例，
> 只把 DUT 的**起参数**换成实机冻结档。

### 11.1 冻结档长什么样（= `test_mpc_node.py` 的 `FROZEN_PARAMS`）

出处 [`first-flight-config.md`](first-flight-config.md) **§2**（`N1`–`N5`，逐条带理由与等级）。
冻结的**形式**是「**除 `N2` 外全部取默认值**」（§2 原话）⟹ 首飞命令行里只有**一个** `-p`：

| 票面点名要"实机参数"的四个量 | 冻结值 | 落在哪 |
|---|---|---|
| 频率 | `rate_hz 100` / `heartbeat_hz 50` / `viz_hz 10` | **节点默认**（`mpc_node.py` 的 `_PARAMS` 表） |
| 悬停油门 | `0.1589` | **模型默认**（`src/nmpc/nmpc/params.py` 的 `Z0Params`，2026-09-29 实测回填） |
| 推力归一化常数 | `t_max = 46.3027 N`（= `m·g / hover_thrust` **导出**） | 同上 |
| τ | **`0.0`**（不补偿） | ★ **与节点默认 `0.05` 不同** ⟹ 命令行那**唯一**一个 `-p tau_s:=0.0` |

⟹ **这次"换实机参数"在台架上只动一个旋钮**。前三个量**不写出来 ≠ 没管** —— 它们是
"已经等于默认值"，而"默认值有没有漂"由
`test_preflight_check.py::test_参数表与_NodeParams_的默认值不许漂移` 守着（那是另一个 seam 的事，
本节只登记）。

**代码落点**（`src/nmpc/test/test_mpc_node.py`，三处）：

| # | 改动 | 为什么 |
|---|---|---|
| ① | 新增模块常量 `FROZEN_PARAMS = {"tau_s": 0.0}` | 冻结档的**唯一**定义处 ⟹ 换机 / 换档只改这一行 |
| ② | `_scenario()` 起 DUT 时合并 `{**FROZEN_PARAMS, **params}` | 本文件从此就是「**实机档**的 seam 1」；用例显式写的键**优先** |
| ③ | 两条"推力低于悬停"的探针**显式钉** `tau_s=0.05` | 见下 §11.4 那条**静默**——**不钉也全绿**，但会不再测外推 |

### 11.2 跑法（可整段复制；未来换机照做）

```bash
source /opt/ros/humble/setup.bash
source ~/px4_msgs_ws/install/setup.bash
source install/setup.bash
export ACADOS_SOURCE_DIR=$HOME/acados
export LD_LIBRARY_PATH=$ACADOS_SOURCE_DIR/lib:$LD_LIBRARY_PATH
python3 -m pytest src/nmpc/test/test_mpc_node.py -q      # 2026-10-10 实测 ≈7m40s（本机 Orin NX）
```

★ **跑之前先看一眼 `pgrep -af mpc_node`**（本文件开头 ★★ 那条：别人的 DUT 会发在**同一对固定
话题**上，症状是数字不对而不是报错）。**不要**用 `pytest -n`。
★ **换机 = 改 `FROZEN_PARAMS` 那一行 + 重跑这条命令**。（本票的口径是"节点默认即实机值"，
所以对一台参数不同的新机，真正要改的是它自己那份 `first-flight-config` 的 `N*` 表 + 这里的常量。）

### 11.3 结论（2026-10-10，本机 Orin NX）

**`45 passed, 0 failed`（整文件，7m38s）** —— 冻结档下**一条都没红**。

票面点名的八项断言范围**逐项对得上**（这是"与 Phase 2 的 seam 1 一致"的判据）：

| 票面点名的验收项 | 落在哪条用例 | 冻结档 |
|---|---|---|
| 符号映射表 | `test_目标在正前方时_pitch_为负_且顶约束表现为打满`（+ `test_sign_map.py`，seam 2） | ✅ |
| 推力归一化 | `test_位置保持时推力归一化等于悬停油门_角速率近零` | ✅ |
| NaN **整帧丢弃** | seam 1：`test_求解失败时先保持上一帧_再切安全悬停_且一帧都不带非有限值`（全帧扫描）；seam 2：`test_node_core.py::test_整帧校验认得非有限值` | ✅ |
| 求解失败兜底 | 同上（`-p inject_solve_fail_after_frames` 注入） | ✅ |
| 心跳频率 | `test_实做频率_心跳_50_Hz_setpoint_100_Hz` | ✅ |
| QoS 可通 | `test_接口与_QoS_与票面一致` | ✅ |
| 时间戳单位 | `test_时间戳是墙钟微秒` | ✅ |
| 未解锁 / 未在 OFFBOARD 时**不发布** | `test_闸门关闭时只发心跳_一帧_setpoint_都没有`（正控 `test_解锁切_OFFBOARD_之后立刻开始发`） | ✅ |

★ **条数不是契约**（同 §7 那条）：`grep -c '^def test_'` 会随判据增删而变，**判"过没过"看退出码**。
★ **对照**：改动**前**（同一台机器、同一晚、τ=0.05 那套）跑出 `43 passed, 1 failed` ——
红的是 `test_SIGINT_之后打出收尾摘要而不是_traceback`，根因是订阅回调里抛
`RuntimeError: Unable to convert call argument to Python object`（DDS 反序列化），**单跑必过 ⟹ 偶发**；
冻结档这一跑它**过了**。⟹ 那一条**不是** τ 引起的，也**不是**本票引入的（基线跑用的是改动前的文件）。

### 11.4 ★★ 一处**静默**被本票挡下来了（唯一的实质性发现）

`test_A_档_垂直速度的外推_表现是推力低于悬停` 与 `test_B_档_速度从_velocity_enu_进外推_表现是推力低于悬停`
两条用例，**原来靠的是"节点默认 τ=0.05"**（它们没写 `tau_s`）。冻结档把默认改成 `0.0` 之后：

- 它们**照样绿** —— 因为 `v ≠ 0` 时"OCP 要把 v 刹到 0"那一项**与 τ 无关**，单独就够撑过
  阈值（`MIN_DELTA_THRUST_N = 0.590 N`）；
- 但它们**不再测外推那条通路**了：τ=0 时 `x + v·τ` 是恒等变换，**把外推整段删掉也照样绿**。

⟹ 处置 = **显式钉 `tau_s=0.05`**（探机制就该把机制的前提写出来）。★ 已实测确认：把这两条的
`tau_s` 临时改成 `0.0`，**两条都过**（2 passed）—— 也就是这个问题**不会自己报红**，
正是本仓最恨的那一类。（登记：`test_B_档` 那条的 **τ 已经由 `test_冻结档的_tau_…` 与
`test_A_档_有速度之后_tau_…配对差分` 两条显式探针覆盖**，所以这里钉值不损失覆盖面。）

**同一晚还补了一条正控**：`test_冻结档的_tau_真的是_0_而不是节点默认的_0_05` ——
把冻结档（**不写** `tau_s`）与**显式 `tau_s=0.05`** 做配对差分，要求两者的差落在"外推该有的方向"上。
**变异记录**（"新断言要主动弄坏一次"）：把 `FROZEN_PARAMS` 改成 `{"tau_s": 0.05}` ⟹
两次跑程**逐位相同**、差 `[0,0,0]`、`k = +0.0000` ⟹ **该用例变红** ✓；改回 `0.0` ⟹ 绿。
（少了这条，`FROZEN_PARAMS` 被改回 `0.05` 会**全绿** —— 那就正是票面要挡的"搬到真机后参数搬错"。）

### 11.5 本节**没证明**什么

- **没有**证明"冻结档比别的档更好" —— 它只证明"这一套起得来、且话题面行为与 Phase 2 那套**同构**"。
- **没有**证明 `thrust_body` 的符号在**真 PX4** 上会被怎么解释（§10 那条界限**一条没变**；
  它靠 SITL 闭环 / seam 3）。
- **没有**碰任何控制律、任何飞控参数、任何 `-p` 之外的东西；**没有**改 `params.py` / `mpc_node.py` /
  `node_core.py`（本票改的**只有** `src/nmpc/test/test_mpc_node.py` 这一个测试文件）。
- **没有**在真机上跑过（那是阶梯 1–4 的事，归 [`phase4/09`](../../.scratch/phase4/issues/09-ladder-ground-and-tether20.md)
  / [`phase4/10`](../../.scratch/phase4/issues/10-ladder-closed-loop-and-free-flight.md)）。
