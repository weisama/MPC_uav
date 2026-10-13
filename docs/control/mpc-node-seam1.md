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

> ☠️☠️ **本节整节按旧载体（`px4_msgs` / uXRCE-DDS 的 `/fmu/*`）写，2026-10-12 起不再是这样**
> （[ticket 13](../../.scratch/phase4/issues/13-mavros-downlink-switch.md) §二：下行换到 mavros）。
> **逐小节点名**，别拿下面任何一张表当现状：
>
> | 小节 | 今天的状态 |
> |---|---|
> | §2.1 订阅 / 发布表 | ☠️ **两张发布行（`/fmu/in/offboard_control_mode`、`/fmu/in/vehicle_rates_setpoint`）与状态订阅那一行都不再成立**：发布变成一条 `/mavros/setpoint_raw/attitude`，订阅变成 `/mavros/state`，心跳**整条删掉**。新话题名/QoS 的**权威在代码**（`mpc_node.py` 的 `TOPIC_*` 表） |
> | §2.1 那句"订阅 `/fmu/out/*` 必须显式 `BEST_EFFORT`" | ⚠️ **结论不变**（"QoS 不匹配是**静默**的"）且更重要了，但**具体档位两条路各换了一个**：新发布的那条要**逐项对齐 mavros 的收端** `SensorDataQoS()`（best_effort / depth 5 / volatile；`setpoint_raw.cpp:62`），新订阅的那条要**逐项对齐** `mavros::StateQoS()`（reliable / depth 10 / **transient_local**；`qos.hpp:60-63`）。★ 两个档位**不同**、而且都不是 ROS 默认值；权威在代码（`mpc_node.py` 的 `QOS_PUB_ATT` / `QOS_SUB_MAVROS_STATE`，各带依据） |
> | §2.2 心跳无条件发 | ☠️ **前提没了**：心跳流已删（§二④ 裁定收成单频），setpoint 自己就是心跳；`heartbeat_always` 参数一并删除 ⟹ 本节那段"不发 setpoint ≠ 松手"的**机制**仍值得读，但**参数面已经不存在** |
> | §2.3 频率表 | ☠️ 心跳那两行作废；`vehicle_rates_setpoint` 100 Hz 与 RViz 10 Hz 两行仍对（换的是载体，不是频率） |
> | §2.4 时间戳 | ☠️ **整节作废**：新载体上飞控那个 handler 不读 `time_boot_ms`，`timestamp` 这个字段根本没有对应物（§四 的映射表里明写"删除或标记为不再可测"） |
>
> ★ **本节为什么不当场重写成新表**：新话题名/QoS/频率的**逐条证据与映射表是本票 §四 的交付物**
> （票面原话："映射表本身就是交付物，写进 `mpc-node-seam1.md`"）⟹ 这里只**作废**，不**代写**。
> ⛔ 也别把这里当成"§2 已经被 §四 改过了"的凭证 —— §四 落地时回来把本节整节换掉。

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

> ☠️ **本节被 2026-10-12 的下行换载体部分作废**（[ticket 13](../../.scratch/phase4/issues/13-mavros-downlink-switch.md)
> §二：`/fmu/*` → mavros）。本节是那次改动**之前**（2026-10-10）的读数，**逐项点名**哪些还站得住：
>
> | 本节里的东西 | 今天还能不能照读 |
> |---|---|
> | §11.1 冻结档表的**除频率外三行**（悬停油门 / `t_max` / `τ=0.0`） | ✅ 与下行无关，照旧 |
> | §11.1 频率行的 `heartbeat_hz 50` | ☠️ **参数已删**（§二④ 裁定"收成单频"）。★ 那条**用例名**（`test_实做频率_心跳_50_Hz_setpoint_100_Hz`）与 §11.3 的"心跳频率"行**已标在 §11.3 表里**，改写归 §四 |
> | §11.1 的**代码落点**三处（`FROZEN_PARAMS` 那三条） | ✅ 与下行无关（§四 也明说冻结档保留） |
> | §11.2 那条命令 | ☠️ **已就地改**（加了 `~/mavros_ws` 一行，见下） |
> | §11.3 的读数（`45 passed`）与逐项对照表 | ⚠️ **是旧载体的读数**：`45 passed` 这个数、以及"心跳频率 / 时间戳单位 / 闸门"三行的**用例名**都属于换载体之前。⛔ **不要拿它当"今天也是 45 条全绿"** —— 真实条数与红色数以 `pytest` 退出码为准（同 §7 那条），重跑归 §四 |
> | §11.4 那条静默（τ 与两条探针） | ✅ 与下行无关 |
>
> ★ **为什么不全就地改写**：票面把 seam 1 **用例本身**的改写明确归 §四（"映射表本身就是交付物"）
> ⟹ 本节只把**读起来像还能照抄的**地方（冻结档表 + 命令）改成与新事实一致，
> 用例名与读数留给 §四 一次改干净（票面 §四⑤ 点名这两处不许留悬空）。

### 11.1 冻结档长什么样（= `test_mpc_node.py` 的 `FROZEN_PARAMS`）

出处 [`first-flight-config.md`](first-flight-config.md) **§2**（`N1`–`N5`，逐条带理由与等级）。
冻结的**形式**是「**除 `N2` 外全部取默认值**」（§2 原话）⟹ 首飞命令行里只有**一个** `-p`：

| 票面点名要"实机参数"的四个量 | 冻结值 | 落在哪 |
|---|---|---|
| 频率 | `rate_hz 100` / `viz_hz 10` —— ☠️ **本行 2026-10-12 改过**：原写 `rate_hz 100` / `heartbeat_hz 50` / `viz_hz 10`，`heartbeat_hz` 那个参数**已删**（见下） | **节点默认**（`mpc_node.py` 的 `_PARAMS` 表） |
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
| ③ | 那条"推力低于悬停"的探针**显式钉** `tau_s=0.05` —— ☠️ **本行 2026-10-13 改过**：原写"两条"，A 档那条已随 A 档整个删除、B 档那条改了名 ⟹ 现在**只剩一处**（`test_垂直速度的外推_表现是推力低于悬停`，源码 `_scenario(tau_s=0.05)`）。映射见 §13.1 | 见下 §11.4 那条**静默**——**不钉也全绿**，但会不再测外推 |

### 11.2 跑法（可整段复制；未来换机照做）

```bash
source /opt/ros/humble/setup.bash
source ~/mavros_ws/install/setup.bash     # ← ☠️ 2026-10-12 新增（ticket 13 §二⑥）：下行换成 mavros
source ~/px4_msgs_ws/install/setup.bash   # ← ☠️ 2026-10-13 改注：**仍必须**，但理由换了（见下）
source install/setup.bash
export ACADOS_SOURCE_DIR=$HOME/acados
export LD_LIBRARY_PATH=$ACADOS_SOURCE_DIR/lib:$LD_LIBRARY_PATH
python3 -m pytest src/nmpc/test/test_mpc_node.py -q      # 2026-10-10 实测 ≈7m40s（本机 Orin NX）
```

☠️ **本条命令 2026-10-12 动过、2026-10-13 又核过一遍**（[ticket 13](../../.scratch/phase4/issues/13-mavros-downlink-switch.md) §二⑥/⑦ → §四）：

- `~/mavros_ws` 那一行（2026-10-12 加）：**仍然必须**，拉的是 `mavros_msgs`（`nmpc/package.xml` 里已有那条 `<depend>`）。
- `~/px4_msgs_ws` 那一行：**仍然必须，但理由换了**。
  ☠️ 2026-10-12 写的理由是"`test/test_mpc_node.py` 仍 `from px4_msgs.msg import VehicleStatus`"——
  **§四 落地后这句话不再成立**（该文件里已经没有 `px4_msgs` 的 import 了）。
  现在的理由是**夹具**：`import bench` ⟹ `bench/scenario.py:38` 的**模块级**
  `from .fake_px4 import FakePx4` ⟹ `fake_px4.py:46` 的 `px4_msgs.msg`。
  **第一手**（2026-10-13）：漏这一行 ⟹ `ModuleNotFoundError: No module named 'px4_msgs'`，
  在 `import bench` 就炸，**跟 DUT 或 A 档毫无关系**。
  ⟹ 这行什么时候能删，取决于 `fake_px4.py` 的去留（**待裁**，见 §13.1 末与 §13.3）。

★ **两处（本节 + §11.1）已于 2026-10-13 核过一遍**，逐项结果见 **§13.4**；⛔ 别在别处再各改一半。

★ **跑之前先看一眼 `pgrep -af mpc_node`**（本文件开头 ★★ 那条：别人的 DUT 会发在**同一对固定
话题**上，症状是数字不对而不是报错）。**不要**用 `pytest -n`。
★ **换机 = 改 `FROZEN_PARAMS` 那一行 + 重跑这条命令**。（本票的口径是"节点默认即实机值"，
所以对一台参数不同的新机，真正要改的是它自己那份 `first-flight-config` 的 `N*` 表 + 这里的常量。）

### 11.3 结论（2026-10-10，本机 Orin NX）

**`45 passed, 0 failed`（整文件，7m38s）** —— 冻结档下**一条都没红**。

> ☠️ **这一行的读数属于 2026-10-10 那套载体，不是今天的**：下行换到 mavros 之后
> （2026-10-12，ticket 13 §二）下面这张表里带 ☠️ 的三行**都要改写或作废**，改写归 §四；
> ⛔ 别把 `45 passed` 抄进任何新文档当现状（**条数与红色现算**，同下面那条"条数不是契约"）。

票面点名的八项断言范围**逐项对得上**（这是"与 Phase 2 的 seam 1 一致"的判据）：

| 票面点名的验收项 | 落在哪条用例 | 冻结档 |
|---|---|---|
| 符号映射表 | `test_目标在正前方时_pitch_为负_且顶约束表现为打满`（+ `test_sign_map.py`，seam 2） | ✅ |
| 推力归一化 | `test_位置保持时推力归一化等于悬停油门_角速率近零` | ✅ |
| NaN **整帧丢弃** | seam 1：`test_求解失败时先保持上一帧_再切安全悬停_且一帧都不带非有限值`（全帧扫描）；seam 2：`test_node_core.py::test_整帧校验认得非有限值` | ✅ |
| 求解失败兜底 | 同上（`-p inject_solve_fail_after_frames` 注入） | ✅ |
| 心跳频率 | ☠️ `test_实做频率_心跳_50_Hz_setpoint_100_Hz` —— **心跳流已删**（§二④ 收成单频）⟹ 这条不在今天的事实里；改写归 §四 | ✅（2026-10-10） |
| QoS 可通 | `test_接口与_QoS_与票面一致` | ✅ |
| 时间戳单位 | ☠️ `test_时间戳是墙钟微秒` —— 新载体上**没有对应物**（飞控那个 handler 不读 `time_boot_ms`）⟹ 该删或标"不再可测"，**不许留成绿着但不测任何东西**；归 §四 | ✅（2026-10-10） |
| 未解锁 / 未在 OFFBOARD 时**不发布** | ☠️ `test_闸门关闭时只发心跳_一帧_setpoint_都没有` —— **闸门源换成 `/mavros/state`、心跳流没了**，用例名与判据都要改；归 §四 | ✅（2026-10-10） |

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

---

## 12. 真机链：成员、谁做什么、**三个未决**（2026-10-10 补）

> **这一节为什么存在**：`run/z0.py` 起的全是 **Phase 1** 那条链（mavros → vrpn → mocap → traj + control），
> **(C) 级的 `mpc_node` 没有入口**（`grep -rn 'nmpc\|mpc_node' run/` ⟹ 0 命中；`src/nmpc/` 下也没有 launch 文件）。
> 于是 `phase4/08` 的「求解失败注入」「回退 (A) 级」两条演练、以及阶梯 1–4，都缺一份"怎么把它摆到台上"的说明。
> **本节只写第一手能证的那部分**；三条查出来的空缺**点名留白**（它们是**查询结果**，不是我的裁定）。
>
> **判据不在这里** —— 判据在 [`../reference/px4-interface.md`](../reference/px4-interface.md) **§8「实飞前检查清单」**
> （(C) 方案的 **12** 条勾选框：动捕 → Position 模式悬停 → `UXRCE_DDS_CFG` → 心跳 → 推力 `[-1,1]` 范围 → 角速率符号…）。
> 本节补的是它没写的那一半：**谁起什么、按什么次序、以及哪些前置今天还不成立**。

### 12.1 链的成员（与 Phase 1 的链**不是同一条**）

> ☠️ **本表有三行被 2026-10-12 的下行换载体作废**（ticket 13 §二）：`MicroXRCEAgent` 那一行
> （**不再需要它**——(C) 级不再走 `/fmu/*`）、`mavros` 那一行（**节点自己现在也需要它**，
> 它就是新的下行载体）、`mpc_node.py` 那一行（下发的是 `/mavros/setpoint_raw/attitude`，
> 且**没有了心跳那条**）。下面那张 ★ 话题面同样作废，逐条见紧跟着的 ☠️。
> ★ **票面 `§一②` 只点名了 §12.3**；这三处是**同一事实的另一个出口**，不一起标会自相矛盾
> （本节说"不需要 mavros"、§12.3 说"改走 mavros"）。若判越权请当场驳。

| 进程 | 起法 | 它在这条链里干什么 |
|---|---|---|
| `MicroXRCEAgent` | `/usr/local/bin/MicroXRCEAgent …`（SITL 用 `udp4 -p 8888`，见 `tools/px4-sitl/run_sitl.sh:108`；端口默认 8888 在 `:24`） | ☠️ 原文："`/fmu/*` 的桥。**节点与 PX4 之间唯一的下行通路**，缺它则帧发不出去"。**2026-10-12 起 (C) 级下行不经它** ⟹ 这条链里**不再有它**（`tools/px4-sitl/` 那条 SITL 链仍要用，那是另一回事） |
| `vrpn_listener` | `ros2 launch vrpn_listener sync_entity_state.launch` | `/vrpn/<刚体>/pose` |
| `mocap` | `ros2 launch mocap mocap.launch.py` | 同**一个**节点（`MocapRelPoseNode`）发 **三条**：`/mocap/pose_enu` 与 `/mocap/velocity_enu`（→ 给**节点**当测量，`mocap_rel_pose_node.cpp:132-134`）、`/mavros/vision_pose/pose`（→ 给 **EKF2**，`:130-131`）。★ 它还**订阅** `/mavros/state`（`:126`）—— 解锁上升沿切原点要用它 |
| `mavros` | `ros2 launch mavros px4.launch fcu_url:=/dev/ttyTHS1:921600` | ☠️ 原文："节点**自己不**需要它；但 **EKF2 需要**（视觉位姿那条），且 `mocap` 的解锁边沿要靠它"。**改**：节点**自己就要它了** —— 它就是 (C) 级的新下行载体（`/mavros/setpoint_raw/attitude` 出、`/mavros/state` 进）。⛔ 别再把 mavros 当成"只是给 EKF2 用的" |
| `mpc_node.py` | `ros2 run nmpc mpc_node.py --ros-args -p tau_s:=0.0`（首飞冻结档，见 §11.1） | ☠️ 原文："`/fmu/in/vehicle_rates_setpoint` + `/fmu/in/offboard_control_mode`（心跳）"。**改**：`/mavros/setpoint_raw/attitude`（**单频**，setpoint 自己就是心跳） |
| （前置闸门） | `ros2 run nmpc preflight_check.py -p tau_s:=0.0` | **必须 `exit 0` 才装桨**（`first-flight-config.md` §5 ③）。★ 2026-10-12 起它多三条**下行前置**（`MAV_FWDEXTSP` / `thrust_scaling` / 那份 `px4_config.yaml` 被加载），且最后那条**要 source 过 `~/mavros_ws`** 才验得了（验不了时**报红**，不是跳过） |

★ 节点的**完整**话题面（第一手；☠️ 下面这个行号区间是 2026-10-10 那个版本的，
**换载体后已不指同一处** ⟹ 要读清单请看 `mpc_node.py` 的 `TOPIC_*` 与 `QOS_*` 两张表）。
**两处订阅是有条件的**，都得看参数档：

> ☠️ **下面这一节按旧载体写，2026-10-12 作废**。新的权威在代码里那两张表
> （`mpc_node.py` 的 `TOPIC_*` 与 `QOS_*`，各带依据），或 §2 节首那个作废块。
> 逐条差异：订阅侧 `/fmu/out/vehicle_status` → **`/mavros/state`**（无条件）；
> `state_source:=px4` 那一档**整档拒绝**（不再是"换两条话题"）；
> 发布侧**少一条**（心跳删除）、`/fmu/in/vehicle_rates_setpoint` → `/mavros/setpoint_raw/attitude`。

- ☠️ **订阅**：`/mocap/pose_enu` + `/mocap/velocity_enu`（默认档 `state_source:=mocap`）；
  **`/fmu/out/vehicle_status`（无条件，与两个档位都无关）**；
  `/traj/reference` **只在 `reference_source:=traj` 时**才订 —— 而**默认是 `hold`**（`mpc_node.py:328`）
  ⟹ **首飞冻结档（只给 `-p tau_s:=0.0`）下这条不订**。
  （`state_source:=px4` 时把测量换成 `/fmu/out/vehicle_local_position` + `/fmu/out/vehicle_attitude`。）
- ☠️ **发布**：`/fmu/in/vehicle_rates_setpoint`、`/fmu/in/offboard_control_mode`、`/mpc/predicted_path`、`/mpc/reference`。

### 12.2 ★★ 节点**不**做两件事（第一手，别指望它）

1. **不解锁、不切模式**。`mpc_node.py` 里**没有**任何命令通路
   （`grep -n 'VehicleCommand\|create_client\|/mavros/cmd\|/mavros/set_mode' src/nmpc/nmpc/mpc_node.py`
   ⟹ 0 命中，2026-10-12 复核；对照 `src/nmpc/nmpc/offboard_smoke.py` 有）。
   ⟹ 它**等别人**把 PX4 弄到「已解锁 + OFFBOARD」；在那之前一帧都不发
   （`node_core.py:213-214`：`warmup_cycles` 的定义就是「收到 armed + OFFBOARD **之后**先解这么多次，**不发**」）。
   ☠️ **原文接着写的是"⟹ 它只发**心跳**，等**别人**…"** —— 那半句 2026-10-12 作废：
   心跳那条流已删（ticket 13 §二④），现在它发的就是 setpoint 本身（**单频**）。
   ⚠️ 另加一条限定（换载体带来的）：闸门源 `/mavros/state` 是 **1 Hz**，
   所以"确认已进 OFFBOARD"会比操作者的动作**晚最多一个上报周期**。
2. **不认"任务"**。没有 Phase 1 那种状态机 —— "这一趟飞行"从哪开始、到哪结束，它不知道
   （`node_core.py:204-206`：「本节点**不是**任务状态机，它不认识"这一趟"是什么；"任务放弃"归 `control`（Phase 1）那一侧」）。

### 12.3 三个未决（每条都是查出来的空缺，**归属要人裁定**）

> ☠️☠️ **本节 2026-10-12 逐条改写过**（[ticket 13](../../.scratch/phase4/issues/13-mavros-downlink-switch.md) §一②）：
> 下行从 uXRCE-DDS 的 `/fmu/*` 换到 **mavros** ⟹ 三条未决的**前提**都变了。
> 体例如下：每条**先写新状态**，原文**原样留在**「原文（2026-10-10，uXRCE-DDS 前提）」块里 ——
> ⛔ 原文是**起因证据**（`MAV_*_CONFIG` 那些读数就是"为什么换载体"的第一手），不许删。
> **逐条**：① 消解（不再是未决）；② **仍未决**，但换成 mavros 形态；③ 仍成立且**更难办**。

**① ☠️ 已消解：不走 uXRCE-DDS ⟹ 不占串口、不加硬件。**

2026-10-12 起（ticket 13 §二 落地）(C) 级下行走 **mavros**（`/mavros/setpoint_raw/attitude`
+ `/mavros/state`），即**复用 Phase 1 已经在跑的那条链**（`~/mavros_ws`，`fcu_url:=/dev/ttyTHS1:921600`）。
⟹ 原文那条"要找一个串口给 XRCE、再把 `UXRCE_DDS_CFG` 打开"的**未决**从"归属待定"变成
"**此路不通，改走 mavros**"，**不需要任何新的串口或硬件**。
★ 下面原文里那三个 `MAV_*_CONFIG` 读数**保留**：它们正是这个决定的**起因**（不是已作废的结论）。

**原文（2026-10-10，uXRCE-DDS 前提）**：

- 第一手：`UXRCE_DDS_CFG = 0`（禁用），**今天**的导出 `bag/z0-params-2026-10-10.params:913` 与
  `docs/assets/z0.params:949` 两次都读 `0`；且**没有任何串口分给它**
  （[`params-changes.md`](../reference/params-changes.md) §E1：`MAV_0_CONFIG = 101` TELEM1、
  `MAV_1_CONFIG = 102` Onboard **被 mavros 占着**、`MAV_2_CONFIG = 0`）。
- 后果：`mpc_node` 发出去的 `vehicle_rates_setpoint` **到不了 PX4**；连它订阅的 `/fmu/out/vehicle_status`
  也没有 ⟹ 闸门永远不开。
- ⟹ **`phase4/08` 的「求解失败注入」在真机上今天做不了**（它要的正是这条链）；「回退 (A) 级」里
  "Phase-1 链还在"那一半可以验，(C) 级那一半不行。
- ⚠️ §E1 当年把它标成"★ Phase 2 阻塞"，但**全仓 `.scratch/` 里没有任何一张票认领它**（`grep -rn UXRCE_DDS_CFG .scratch/` ⟹ 只有 `phase1/15` 的一张对账表提到）。**归属待定。**

**② ☠️ 仍然是未决，但接口换成了 mavros 形态**（三种候选一个没少）。

- 接口：`/fmu/in/vehicle_command` → **`/mavros/cmd/arming`**（解锁）+ **`/mavros/set_mode`**（切 OFFBOARD）。
  ★ 这两条现在**已经有一份可用实现**：[`offboard_smoke.py`](../../src/nmpc/nmpc/offboard_smoke.py)
  在 ticket 13 §三 里已按新载体改写（两个服务客户端 + 应答回调，被拒时把 `result` / `mode_sent`
  打进日志），且它的等待间隔已重核过（`/mavros/state` 是 **1 Hz** ⟹ 最小间隔必须 **> 1 s**，
  默认 1.1 s）⟹ 原文那句"搬出来的活已经有人干过大半"现在是"**已经干完了**"。
- 候选**一个没少**：(a) QGC 手动解锁 + 手动切；(b) 写一个小的 arm 助手；(c) 直接用
  `offboard_smoke.py` 自己那套。
- ⚠️ **票面 `§一②` 曾写"归属由 §二 裁"—— 但 §二 落地时（2026-10-12）**没有**裁这件事
  （§二 的裁定只涉及 ⑤ 的 A 档与 ④ 的心跳）。⟹ 这一条**仍是未决**，归属待定。

**原文（2026-10-10，uXRCE-DDS 前提）**：

- 由 §12.2，节点不干这件事；而 SITL 里干这件事的是一个**测试夹具**里的节点
  （`tools/px4-sitl/verify_tier_a_hover.py:279` 的 `ArmAgent`：先 `ARM_DISARM`、`armed` 之后 `DO_SET_MODE`）。
- 真机上两条候选：**(a)** QGC 手动解锁 + 手动切 OFFBOARD；**(b)** 写一个小的 arm 助手（把 `ArmAgent` 搬出来）。
- ★ **但"搬出来"的活已经有人干过大半**：`src/nmpc/nmpc/offboard_smoke.py`（ticket 05 的冒烟节点，见其文件头）
  **本身就是**一条「先发心跳 ≥1 s → 解锁 → 切 OFFBOARD」的完整实现（`:289` / `:294`，
  publisher 在 `:112`），且它与 `ArmAgent` 的机制**逐字同源**（`ArmAgent` 的 docstring 自己这么写的）。
  ⟹ ② 不是"从零写"，而是"**挑哪一份、放在链的哪个位置、由谁起**"——这个仍然没人裁定。
- ⚠️ **注意与 Phase 1 的冲突**：Phase 1 的 `control` 把"QGC 手动解锁"判成**外部解锁 ⟹ 停发**
  （`bench-runbook.md` 的「已删：台架专用缝 `own_arming`」）。那是 `control` 自己的语义，**与 (C) 级链无关** ——
  但两条链共用同一架飞机，**别把两套纪律混着记**。
- **归属待定**（阶梯 1 要"只给 ±小角速率、目视电机转速差"，那一步同样需要先解锁 + OFFBOARD）。

**③ ☠️ 仍然成立，而且「更难办」了：两条链现在共用同一组 mavros 话题。**

2026-10-12 起 (C) 级也走 mavros（ticket 13 §二）⟹ 互斥不再只是"两个驾驶员抢 OFFBOARD"：

| | 下行话题 | 谁在发 | 闸门/模式话题 |
|---|---|---|---|
| (A)/(B) 级（Phase 1） | `/mavros/setpoint_raw/local` | `control`（+ `traj` 给参考） | `/mavros/state` |
| (C) 级（Phase 4） | **`/mavros/setpoint_raw/attitude`** | `mpc_node` | **`/mavros/state`** |
| ☠️ 共用（**不是假设，两边都配了**） | —— | —— | **`/mavros/state`**、**`/mavros/cmd/arming`**、**`/mavros/set_mode`** |

★ 上表第三行是**读配置核出来的**（`src/control/config/control.yaml:41` 的 `state_topic`、
`:64` 的 `set_mode_service`、`:66` 的 `arming_service`；`offboard_smoke.py` 侧是同名两条），
⛔ 不是"以后可能会"。⟹ 除了"两个驾驶员"，还多出两类**新**的互踩面：
**谁去切模式**（同一个 `set_mode` 服务，两条链各切一次会互相打断，而**两条链都会去切**）与
**闸门判读**（两条链都盯 `/mavros/state`，一条把飞机切走，另一条会把它读成"离开 OFFBOARD"
⟹ 触发各自的停机路径）。

**原文（2026-10-10，uXRCE-DDS 前提）**：

| | 下行话题 | 谁在发 |
|---|---|---|
| (A)/(B) 级（Phase 1） | `/mavros/setpoint_raw/local` | `control`（+ `traj` 给参考） |
| (C) 级（Phase 4） | `/fmu/in/vehicle_rates_setpoint` | `mpc_node` |

话题不同，但**两条都喂 PX4 的 OFFBOARD** ⟹ 同时跑 = 两个驾驶员。
**「回退 (A) 级」的**切换动作**就落在这里**：停 `mpc_node` → 起 Phase-1 链（`run/z0.py up`）→
盯 `/control/state` 回来。⚠️ **这一步本身没有成文流程**（`phase4/08` 要的"切换动作演练到位"就是它）。

### 12.4 本节**没证明**什么

- **没有**起过、没有跑过真机：上面的命令是**从仓库里现读出来的**（launch 文件、节点源码、`run/README.md`），
  不是实测过的次序。**首次起链时要按 §8 的清单逐条对，而不是照抄本节**。
- §12.3 的三条是**查询结果**（"仓库里没有"），**不是**"这件事做不到"的证明 ——
  ① 只要给一个串口 + 把 `UXRCE_DDS_CFG` 指过去就成立；② 只要有人裁定做法就成立。

---

## 13. Phase 4 §四：夹具与 seam 1 换成 mavros 形状（2026-10-13）

> **这一节为什么存在**：[ticket 13](../../.scratch/phase4/issues/13-mavros-downlink-switch.md) §四 把 seam 1 从
> uXRCE-DDS 的 `/fmu/*` 搬到 **mavros**（接口由 §二 定，夹具与用例由 §四 换）。票面把「**映射表本身
> 就是交付物**」点名写进了 §四③ ⟹ 本节体例照 §11.3 / §12.4：**先写换的是什么 → 逐条映射 → 读数 →
> 「没证明什么」**。
> ★ 本节读数全部来自 **2026-10-13 本机 Orin NX** 的实跑；**条数与红色现算**（同 §7 / §11.3 那条），
> ⛔ 别把下面任何一个数抄进别的文档当现状。

### 13.0 换的是什么：三条静默符号（N1/N2/N3）+ 心跳

| # | 旧载体（uXRCE-DDS / `px4_msgs`） | 新载体（mavros） | 错了会长什么样 |
|---|---|---|---|
| **N1** | `VehicleRatesSetpoint.thrust_body_z`，**负**号（`−z` 向上） | `AttitudeTarget.thrust`，**正**号（`0…1` 归一化） | 反号 ⟹ 推力图**上下颠倒**、一解锁就砸地；台架上表现为"推力**高于**悬停" |
| **N2** | 我们自己把 FLU 转成 FRD 再发 | **FLU 原样**发（mavros 自己转，`setpoint_raw.cpp:321-331`） | 我们若再取负 = **转两次** ⟹ `pitch`/`yaw` 静默反向。⚠️ 在"飞控最终收到的物理方向"上**看不出来** |
| **N3** | 闸门源 `/fmu/out/vehicle_status`（**2 Hz**），超时 `0.75 s` 够用 | 闸门源 `/mavros/state`（**1 Hz**，跟 MAVLink 心跳走），超时改 **`2.5 s`** | `0.75 < 1.0` ⟹ 两帧之间**必然**越线 ⟹ `status_stale` **恒真**、闸门一直关、**一帧都不发**（而日志看起来像"飞控失联"） |
| **心跳** | 独立流 `/fmu/in/offboard_control_mode`（50 Hz，uORB 看门狗要求 ≥2 Hz） | **删除**（mavros 的 `setpoint_raw` 插件替我们维持 OFFBOARD 需求） | 留着 = 测一条我们已经不发的流（"绿着但不测任何东西"） |

★ 这三条**只有在一个地方能逐条证伪**：断言落在 "**我们发出去的字段原值**" 上（票面 §四① 的原话）。
落到"飞控收到的物理方向"上时，**N2 会被双重取负抹平**（✋ [判据全绿也可能是机理反了] 那一族）。
⟹ 夹具侧新增 `AttitudeSetpoint` 记录（`type_mask` / `body_rate.x/y/z` / `thrust` / `stamp_ns`），
并进 `FakeMavros.evidence()`。

### 13.1 映射表：HEAD `6f69add` 的 **42** 条 → 现在的 **38** 条

**总账**：`-5`（删）+ `-1`（并）+ `+2`（增）= **−4**。

#### 按票面 §四③ 的分类看处置

| 票面点名的那一类 | 处置 | 落点 |
|---|---|---|
| 话题名 / QoS | **逐条换**（含"QoS 不匹配就收不到"的负控） | `test_接口与_QoS_与票面一致` |
| 符号 + 推力归一化 | **重写断言对象**；★ 新增一条**专测 N2** | `test_目标在正前方时_body_rate_y_为正_…`、`test_N2_下行里_body_rate_是_FLU_原样_没有预先取负`、`test_垂直速度的外推_表现是推力低于悬停` |
| 闸门（未解锁 / 非 OFFBOARD / 源过期） | 数据源换 `/mavros/state`；★ 超时那条**把变异写进用例** | `test_未解锁时一帧_setpoint_都没有`、`test_闸门源静默超过阈值之后停发_恢复之后重开`、`test_闸门源超时阈值_2_5_下正常跑不报_stale_而_0_75_下会` |
| **时间戳** | ☠️ **删**（新载体上**没有对应物**，理由见下） | `test_时间戳是墙钟微秒` ⟹ **不存在** |
| 频率与 RViz | 按 §二④ 的裁定**收成单频** | `test_实做频率_setpoint_100_Hz` |
| A 档（`state_source="px4"`） | 按 §二⑤ 裁定 (b)：**入口当场拒绝** | `test_A_档_state_source_px4_入口当场拒绝` |
| 求解失败兜底 / NaN 整帧丢弃 / 进程收尾 | **语义不变**，只换消息类型 | 三条同名未动 |
| 轨迹参考 / 输入合理性 | **不动**（与下行无关） | 七条同名未动 |

#### 逐条（左边是 HEAD 的 42 条，右边是现在的 38 条）

| HEAD | 现状 | 处置与依据 |
|---|---|---|
| 1 `test_接口与_QoS_与票面一致` | 同名，**整体换载体** | 断言方向也换了：DUT 在 `/mavros/setpoint_raw/attitude` 上是**发布方**、在 `/mavros/state` 上是**订阅方** |
| 2 `test_闸门关闭时只发心跳_一帧_setpoint_都没有` | → `test_未解锁时一帧_setpoint_都没有` | 心跳流删了（N3 的老家）；判据从"只发心跳"变成"**一帧 setpoint 都没有**" |
| 3 `test_解锁切_OFFBOARD_之后立刻开始发` | 同名 | 只换载体；`/mavros/state` 的 `transient_local` ⟹ 起机不必空等 1 s |
| 4 `test_状态静默超过阈值之后_停发_setpoint_但心跳照发` | → `test_闸门源静默超过阈值之后停发_恢复之后重开` | 去掉心跳那一半；补上"**恢复之后重开**"（静默那条流现在也会恢复） |
| —— | **新增** `test_闸门源超时阈值_2_5_下正常跑不报_stale_而_0_75_下会` | ★★ 票面 §二③ 点名要的**变异测试**，把变异本身写成用例的第二半 ⟹ 不靠人记得手改一次 |
| 5 `test_拼错的参数当场拒_…` | 同名 | 不动 |
| 6 `test_tau_取_0_起得来_…` | 同名 | 不动 |
| 7 `test_位置保持时推力归一化等于悬停油门_角速率近零` | 同名 | 归一化常数不变（`t_max` 是模型侧的量）；只有**符号**变了 |
| 8 `test_目标在正前方时_pitch_为负_且顶约束表现为打满` | → `test_目标在正前方时_body_rate_y_为正_且顶约束表现为打满` | FLU 原样发 ⟹ 名字里的"为负"是旧 FRD 口径；见 §13.2 那条轴的话 |
| 9 `test_没热身时第一帧是出厂迭代点_…` | 同名 | 不动 |
| 10 `test_时间戳是墙钟微秒` | ☠️ **删除** | 见下面 §13.1bis |
| —— | **新增** `test_N2_下行里_body_rate_是_FLU_原样_没有预先取负` | ★★ 票面 §四③ 点名要的"专测 N2" |
| 11 `test_SIGINT_…` | 同名 | **改了前置**（见 §13.2 的 +167 ms） |
| 12 `test_实做频率_心跳_50_Hz_setpoint_100_Hz` | → `test_实做频率_setpoint_100_Hz` | 心跳那一半随 §二④ 作废；剩下的仍然只卡下限 |
| 13–16（RViz / 求解失败 / 时间戳倒退 / 位姿跳变） | 同名 | 只换载体 |
| 17 `test_A_档只订_px4_那两条_动捕一帧都不订` | → `test_A_档_state_source_px4_入口当场拒绝` | A 档**整个不可用**了 ⟹ 这条从"它能订到东西"变成"它**起不来**" |
| 18 `test_A_档_两条状态各判各的龄(key, reason)` | ☠️ **删除** | 覆盖面**部分**由新那条"超时阈值"用例接（现在只有**一个**闸门源，谈不上"各判各的龄"） |
| 19 `test_A_档_同一条正前方任务_走哪一根轴由_NED_到_ENU_的换算决定(offset, d_enu)` | ☠️ **删除** | 覆盖面**部分**由第 8 条 + `test_位置保持锚点让位置换算第一次在话题层可观测` 接 |
| 20 `test_A_档_锚点让位置换算在话题层可观测` | ☠️ **删除** | 同上（那条不带 A 的同名用例还在） |
| 21 `test_A_档_注入的倾斜_指令方向是把机身拉平的那一边(axis, deg, expect_flat)` | ☠️ **删除，★ 无接替** | ⚠️ **本轮唯一的净覆盖损失**：那个注入面（`px4_arm` / `px4_nav_state` 改 `fake_px4` 的 `/fmu/out/vehicle_status`）随 `fake_px4` 一起没了，而 mavros 档没有对应的"改假飞控姿态"入口。**登记，归待裁** |
| 22 `test_A_档_有速度之后_tau_第一次在话题上可观测_配对差分` | → `test_速度让_tau_在话题上可观测_配对差分` | 去掉 A 档那一半；差分机制不变 |
| 23 `test_冻结档的_tau_真的是_0_…` | 同名 | 不动 |
| 24 `test_A_档_垂直速度的外推_…` + 25 `test_B_档_速度从_velocity_enu_进外推_…` | → **并成** `test_垂直速度的外推_表现是推力低于悬停` | 只剩动捕一条源（`/mocap/velocity_enu`）⟹ 两条变一条；**符号方向也反了**（见 §13.2） |
| 26 `test_A_档的逐帧日志把状态源_求解耗时_饱和标志都记下来了(tmp_path)` | → `test_逐帧日志把状态源_求解耗时_饱和标志都记下来了(tmp_path)` | 只改名（字段名/来源换载体） |
| 27–33（垂直偏置七条） | 同名 | ★ 名字与语义**不动**（与下行无关），但其中**两条**（`test_垂直偏置不跟着解爬` / `test_垂直偏置积的是实测位置误差`）因**轴变了**要改下标 `u[1]→u[2]`，见 §13.2④ |
| 34–42（掩码常量 / 轨迹 / 发现信息 / τ 量不了 / 锚点） | 同名 | 不动 |

#### 13.1bis ☠️ 为什么"时间戳"那一类**整条删除**（不许留成"绿着但不测任何东西"）

旧用例断言的是 uXRCE-DDS 那条路的 `VehicleRatesSetpoint.timestamp`（恒 `0` = "DUT 没填"）
与 `OffboardControlMode.timestamp`：**µs 单位错在 PX4 侧是静默的**
（`min(timestamp − time_offset, hrt)` 把它钳回"新鲜"），所以当时只能靠
`FakePx4.stamp_unit_guess` 按"相对墙钟的比值"把 µs/ms/ns/零值分开。

**新载体上这条通路整个不存在**：mavros 的 `AttitudeTarget.header.stamp` 进
`setpoint_raw.cpp` 之后是**给人看的**（转发 MAVLink 时另盖时间戳），DUT 填不填、填什么单位，
**飞控都不读** ⟹ 留着它最多能断言"我们填了个墙钟微秒"，而那**不是任何东西的判据**。
★ 仍然被别处覆盖的：`header.stamp` **在话题面上**由 `AttitudeSetpoint.stamp_ns` 记下来了
（`evidence()` 会回读最后一帧）—— 那只是"这一帧带了个戳"，**不是**单位判据。

#### 13.1ter `fake_px4.py` 与 `px4_msgs` 的去留：★ **待裁**（本轮只登记）

- **事实（2026-10-13 现查）**：seam 1 / 本仓测试里**已经没有 DUT 说 `px4_msgs` 了**
  （`test_mpc_node.py` 里 `px4_msgs` 的 import 全部消失，只剩注释）。
- **但它不能删**：`import bench` ⟹ `bench/scenario.py:38` 的**模块级** `from .fake_px4 import FakePx4`
  ⟹ `fake_px4.py:46` 的 `px4_msgs.msg`。⟹ `~/px4_msgs_ws` 那一行**仍然必须 source**。
- **判据与候选**（票面 §四④）：`tools/px4-sitl/` 那一串（`mocap_replay` / `verify_tier_a_hover` /
  `verify_tier_b_seam3` / `verify_offboard_smoke` …）**仍然在用** `px4_msgs`（`grep -rln px4_msgs tools/`）
  ⟹ 它们服务的是 **SITL 那条链**，与本节点无关。
- ⚠️ 本轮**没有**动 `scenario.py`（不在改动范围），所以 `px4=True` 那个档位与
  `DEFAULTS["px4"]` 都**还在原地**（`config.py` 里已用 ☠️ 块登记）。
  ⟹ 票面 §四④ 要的"**显式判、不许留着不管**"目前只做到了"登记 + 写明没有 DUT"，**裁定留给人**。

### 13.2 读数（2026-10-13，本机 Orin NX 8GB / Humble / mavros 2.15.1）

**① 整文件**：`38 passed, 1 warning in 413.31s (0:06:53)` —— 冻结档（`FROZEN_PARAMS`）下**一条都没红**。
（唯一那条 warning 是 matplotlib 的 `Unable to import Axes3D`，**与本节无关**、长期存在。）

**② 解锁之后那一场（`test_逐帧日志…` 打出的 DUT 收尾摘要，逐字，2026-10-13 现场重跑）**：

```
[mpc_node] 收尾：共发 16 帧 setpoint（单频：setpoint 自己就是心跳，无独立心跳流），跨 0.14s
⟹ 实际 105.2 Hz；定时器 67 拍、求解 21 次、求解耗时 p50 4.216 ms / p99 17.501 ms /
最坏 19.878 ms（21 次）、退化 0 拍、拒绝 0 拍；约束饱和：F_max×11, F_min×1, wy×20；
闸门：not_armed×45, warmup×5；兜底：保持 0 拍 / 安全悬停 0 拍 / 触发 0 次
```

⚠️ **这是"一场"，不是契约**：同一个用例两次跑分别读到过 `跨 0.15s / 101.7 Hz / 定时器 58 拍 /
p50 3.385 ms / not_armed×36` ⟹ **耗时与拍数逐场在动**（窗口只有 ~0.15 s）。
⛔ 有判据价值的只有下面三条**定性**读法，别去比小数：

⟹ 三条与载体相关的读法：
- **"单频"是真的**（N3 的收口）：没有独立心跳流，`~105 Hz` 量的就是 setpoint 自己。
- **闸门理由只有 `not_armed` / `warmup`** —— 没有 `no_status` / `status_stale`
  ⟹ 1 Hz 闸门源 + `2.5 s` 阈值这一对**在实跑里不误报**（这正是 N3 要挡的那条静默）。
- **饱和集中在 `wy`**（两次跑都是 `×20`），`wx` 一次都没有 —— 见下面 ④。

**③ `_wait_dut_up` 判据回来之后，还差多远才走完 `__init__`？（第一手，2026-10-13 现场探针，**两场**）**

判据 = 「夹具看到 DUT 订上 `/mavros/state`」；下表两个"差"都是**相对判据那一刻**：

| 事件 | 第 1 场 | 第 2 场 |
|---|---|---|
| 判据成立（订上 `/mavros/state`） | `+8.174 s` | `+8.224 s` |
| DUT 打出第一条启动 info 日志（走 `/rosout`） | `+156 ms` | `+170 ms` |
| **`_tick` 跑第一拍**（证据：第一条"闸门关闭"日志） | `+167 ms` | `+183 ms` |

⟹ 判据回来之后**还有 ~0.16–0.18 s 节点才进入 `spin()`**（`create_timer` 在 `mpc_node.py:747`，
排在 `create_subscription`（`:640`）**之后**）。**后果**：这段窗口里发 SIGINT，
`main()` 里 `node` 还是 `None` ⟹ 那条 `if node is not None` 让收尾摘要**不打**、
而退出码是 **`0`**（症状是"静默地没打"，不是报错）。
⟹ 处置：`test_SIGINT_…` 现在**先解锁 + 等到 ≥1 帧 setpoint**（setpoint 只可能来自 `_tick`
⟹ 那才是"`__init__` 已返回、已进 `spin()`"的黑盒证据）再发 SIGINT。
⛔ 别改成"多睡一会儿" —— 那种修法在换机上会随启动耗时一起失效（本项两场就差 14 ms）。

**④ A 档 / 动捕档的**机头朝向不同** ⟹ 同一条任务顶的是**另一根轴**（第一手）**：

`fake_px4` 的 `quat_ned_frd = (1,0,0,0)` = **机头朝北**；而 `/mocap/pose_enu` 是**单位姿态** = **机头朝东**。
同一段"机头方向 +x 追 10 m"的任务里：

- **A 档**（旧 `fake_px4`）⟹ 用 ENU 的 +x 表示 ⟹ 顶满的是 **`u[1]` / `wx`（roll）**；
- **动捕档**（现在唯一那档）⟹ ENU 的 +x 正好是机头方向 ⟹ 顶满的是 **`u[2]` / `wy`（pitch）**。

⟹ **三条用例**要跟着换轴：`test_逐帧日志…` 的饱和标志（`"wx"` → `"wy"`）与两条垂直偏置
（`test_垂直偏置不跟着解爬` / `test_垂直偏置积的是实测位置误差`，`u[1]` → `u[2]`）。
★ **独立佐证**（不是推理）：运行时摘要自己打了 `约束饱和：F_max×11, F_min×1, **wy×20**`
—— 若轴没变，这里会全是 `wx`。

**⑤ 推力符号（N1）在第一手读数里的样子**（2026-10-13 现场配对量，同一台机器、相邻两场）：

| 场 | `thrust`（归一化） | 与 `hover` 之差 |
|---|---|---|
| 悬停（动捕速度 `0`） | `+0.1589` | `−0.0000` |
| **外推**（`/mocap/velocity_enu` 向上 `1 m/s`，τ=0.05） | **`+0.0975`** | **`−0.0614`** |

`hover = +z0.t_hover / z0.t_max = +0.1589`（**正号**）；判据线 `MIN_DELTA_THRUST_N = 0.590 N`
折到归一化是 `0.0127` ⟹ 实测的 `0.0614` 有 **~4.8 倍余量**（不是刚好过线）。
☠️ **旧口径是负号**（`thrust_body_z`），那时"该减推力"写成 `>` ⟹ 同一个物理事实在两套载体上
**判断方向相反**。这条断言已按新口径改写成 `<`，注释里写明"为什么符号反了"。
★ 悬停那一格量到 `−0.0000` 也顺手证了另一件事：**FROZEN_PARAMS 之外没有别的偏置**
（悬停 ⇔ `thrust` 恰好等于 `t_hover / t_max`）。

**⑥ QoS 的四格矩阵（第一手，本机 Humble + `rmw_fastrtps`）** —— §3 那条"黑盒可观测性"的补全：

| 发布方提供 | 订阅方要求 | 实收帧数 | `get_publishers_info_by_topic` |
|---|---|---|---|
| RELIABLE | RELIABLE | 100 | 1 |
| RELIABLE | **BEST_EFFORT** | **100** | 1 |
| **BEST_EFFORT** | RELIABLE | **0** | **1** |
| BEST_EFFORT | BEST_EFFORT | 100 | 1 |

⟹ ① RxO 就是 `offered ≥ requested`（第 2 行是"夹具收端是 best_effort ⟹ 接得住 reliable 发端"的
第一手来源）；② **不兼容那一格（第 3 行）发现库里照样有 1 条** ⟹
`get_publishers_info_by_topic` **不做 QoS 过滤**，"有几个发布者"≠"有几个**接得上**"；
③ ☠️ **失配不是完全静默的**：不兼容时**两端各打一条 rmw `WARN`**
（`… requesting incompatible QoS. No messages will be sent to it. Last incompatible policy:
RELIABILITY`）⟹ 它会落到进程日志（也就是 `dut_log()`）里。**但**它只在**发现**那一刻打一次，
`ros2 topic list/hz` 照样看不出，且它不是判据 ⟹ 本仓"夹具照抄真 QoS + 用例直接读 profile"
的做法不变，⛔ 只是**别再用"日志里完全看不出原因"当理由**。
☠️☠️ **这张表的第一版是错的**：第一版探针忘了 `rclpy.spin_once`，回调从不执行 ⟹ **四格全是 0**
⟹ "RELIABLE 发 → BEST_EFFORT 收 = 0 帧"被误读成"不兼容"。⚠️ 教训：**判"收没收到"之前，
先确认那个计数器会动** —— 一个恒为 0 的计数器既像"不兼容"也像"没 spin"。

### 13.3 本节**没证明**什么

- ☠️ **`MAV_FWDEXTSP`（硬前置，`=1` 才收 setpoint）夹具模拟不了** —— 它是**真飞控的参数**，
  台架上没有它的对应物。⟹ 这一条**只能在实飞前检查清单里兜**（归票面 §一③ / §二⑤bis）。
  ⛔ **不许让夹具"顺手"把它模拟掉** —— 那会把一条**真实存在的静默失效点**变成一个绿的用例。
- **没有**证明 `thrust` / `body_rate` 到了真 PX4 之后被解释成什么。本节断的**全部**是
  "**我们发出去的那条消息里的字段值**"（这正是票面 §四① 的设计），§10 那条界限**一条没变**。
- **没有**证明真机上的时间行为：`/mavros/state` 在这里是**夹具按 1 Hz 发的**，
  与真 mavros 的 1 Hz（`sys_status.cpp:962-985`）只是**同源抄写**，不是同一次实测。
- **没有**碰任何控制律、任何飞控参数、任何 `params.py` / `mpc_node.py` / `node_core.py`；
  本轮改的只有：夹具（`fake_mavros.py`）、台架配置（`config.py`）、seam 1 用例（`test_mpc_node.py`）、
  以及本页。
- **没有**在真机上跑过（阶梯 1–4 的事）。
- ⚠️ **有净覆盖损失**：A 档那条"注入倾斜 ⟹ 指令方向是把机身拉平的那一边"**没有接替**
  （见 §13.1 第 21 行）。它挡的是"角度符号整体反了"这一类错误 —— 现在这一类**只剩 N2 那条断言**
  （它同样只看我们发出的值）。**登记，归待裁**。
- **没有**让 `px4=True` 变成一个"mavros 档位"（票面 §四③ 想要的是**新增档位**）；
  本轮用的是"**不起 `fake_px4`**"（`Scenario` 的 `px4` 默认就是 `False`），
  因为改签名要动 `scenario.py`，而它**不在本轮的改动范围**里 ⟹ **登记为待收口项**。

### 13.4 回头核 §11.1 与 §11.2（票面 §四⑤：两处都不许留悬空）

按票面"**先核再改，别各改一半**"，本节落地时把这**两处**逐行核了一遍：

| 处 | 原文的问题 | 本轮的处置 |
|---|---|---|
| §11.1 冻结档表 **③** | 写"**两条**'推力低于悬停'的探针显式钉 `tau_s=0.05`" —— A 档那条**已删**、B 档那条**已改名** ⟹ "两条"不再成立 | **就地改**成"只剩**一处**"并点名新用例名（源码 `_scenario(tau_s=0.05)`） |
| §11.2 命令行 `~/px4_msgs_ws` | 注里写"`test/test_mpc_node.py` 仍 `from px4_msgs.msg import VehicleStatus`" —— **已不成立**（该文件里 `px4_msgs` 的 import 全没了）；但那一行**仍然必须 source** | **就地改**：理由换成**夹具**（`scenario.py:38` → `fake_px4.py:46`），并附第一手 `ModuleNotFoundError` 的爆炸位置 |
| §11.2 注里那句"要等 §四 把夹具换过去" | §四 已落地 ⟹ 悬空 | 一并改掉 |
| §11.3 的 `45 passed` 与其三行 ☠️ | 原本就带 ☠️ 指向 §四 | **本轮未改**（它已经写着"⛔ 别抄"，且表里三条的归宿已在 §13.1 逐条给了） |
| §11.4 / §11.5 | 讲 τ 那条静默与"没证明什么" | **本轮未改**：τ 与下行无关，两条探针的**改名**已在 §11.1③ 与 §13.1 第 22 行登记 |

#### ☠️ 本轮**查出但没改**的悬空（⛔ 不在票面点名的两处之内 ⟹ **只登记，等人裁**）

四条，**都在本页**。★ 排序即优先级：第一条在「先读这一段」里，可见性最高。

| # | 处（行号 = 2026-10-13 落笔时） | 现状 | 为什么它已经不对 | 归属 |
|---|---|---|---|---|
| 1 | **§0 第 2 条**（`:14-16`） | 「**心跳（`offboard_control_mode`）必须无条件发**」—— 全页"先读这一段"的第二句话，**没有 ☠️** | 心跳流**整条删了**（§二④ 裁定单频）⟹ 这条现在是**对一条不存在的流**的断言。⚠️ 但**它下面那句"把心跳也闸掉就是死锁"的机理在新载体上仍值得读**（新载体的"心跳"就是 setpoint 自己 ⟹ **闸掉 setpoint 一样进不去 OFFBOARD**）⟹ 该改的是**主语**，不是整条删 | `§一④` 清单已登记，路由到「§二④ 的连带 / **§四**（映射表）」 |
| 2 | **§8 第 3 条**（`:303-308`） | 「台架的判据是**"夹具收到了第一帧心跳"**」 | 判据已换成「**DUT 订上了 `/mavros/state`**」（第一手见 §13.2③；`_wait_dut_up` 的 docstring 里写明"§二④ 换过判据"）。⚠️ 同段 `:303` 的 `matched_publishers("mode")` 也是**已删的 API** | `§一④` 清单**已登记**，路由到 `§四⑤` |
| 3 | **§6 停机条件表**，行「状态静默」（`:248`） | `status_timeout_s = 0.75`，理由"上游是 **2 Hz**" | 现行默认 **`2.5`**（一手：`node_core.py:261`、`mpc_node.py:479` / `:1495`），源是 **1 Hz** ⟹ **两个数都过期**，而这是一张"照着填参数"的表 | **无人登记（本轮新查出）** ⟹ 待裁 |
| 4 | **§1 表**（`:67`）与 **§2.1**（`:89`） | 「QoS 不匹配是**静默**的」/「**也不报错**（QoS 不兼容不报错，只是永不匹配）」 | 2026-10-13 第一手（§13.2⑥）：不兼容时**两端各打一条 rmw `WARN`** ⟹ "静默"是**过强的说法**。⚠️ 结论**不变**（仍要逐项对齐 QoS、仍要靠读 profile 判），只是**理由要换**；⛔ 别反过来读成"那就不用管 QoS 了" | **无人登记（本轮新查出）** ⟹ 待裁 |

⚠️ 上表四条**没有**顺手改掉：票面只点名了 §11.1 / §11.2 两处不许留悬空，
"**改别的节**"不在本轮的允许范围里（同票 13 §一④ 立的"**只登记、不改**"那个机制）。
★ 其中 #1 / #2 是**清单里已经路由给本票**的（§四 / §四⑤）—— 本轮**没能收**，理由是"文件内的改动
只允许落在 §11.1/§11.2 与本节"；**请裁定是否由本轮补做**（改法是就地加 ☠️，不删原文）。

⚠️ **本节与上表是一次"只改读起来像还能照抄的地方"的编辑**，票面原本把 seam 1 用例的改写归 §四、
把文档改写归本节 ⟹ 除上表两处外，**本页别的地方一个字没动**（⛔ 尤其没动 §3 与 §12 的结论区）。

