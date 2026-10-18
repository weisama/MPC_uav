# SITL 最小 offboard 冒烟（ticket 05）—— 六项判据、四类静默失败与一处文档订正

> **一句话**：PX4 在 SITL 里**第一次真的照着我们发的指令动起来** —— 用一个
> **不含任何 MPC 数学**的 stub 节点（只有符号映射 + 指令时间轴 + 发布/订阅），
> 把"中间件 / 符号 / 归一化"这三件事在**真 PX4** 上逐项验掉，给后面所有闭环失败留一份归因基线。
>
> 被控对象是 ticket 08 的 z0 同构机（[`sitl-z0-plant.md`](sitl-z0-plant.md)）；
> 上游基线（SITL 怎么来的、`px4_msgs` 怎么钉的）在 [`px4-sitl-baseline.md`](px4-sitl-baseline.md)。
> 符号的**数值权威**在 [`sign-map.md`](sign-map.md) §2，本页只引用、不复述。
>
> ⚠️ 本张是**开环单项**判据，**不是**闭环（闭环是 S5 / ticket 09/11）。
> 本张排除掉的四项，正是闭环失败时用来归因的基线。
>
> ☠️ **基线已换（2026-10-09，ticket 21 / phase3 的 [`04`](../../.scratch/phase3/issues/04-rebuild-sitl-plant-to-measured-baseline.md)）**：
> 本页**全部读数**出自 ticket 08 那台**重搭前**的同构机 —— 旧机型常数 **`SIH_T_MAX = 2.95 N`
> （总 `4 × 2.95 = 11.8 N`，按电池）**。重搭后被控对象按**电机**改成 **`SIH_T_MAX = 11.5757 N`
> （总 46.3027 N）** ⟹ 本页 **§3.5 那一组（悬停请求 `0.6233`、电机均值 `0.6230`、反推推力 `7.351 N`、
> 末段高度 `−1.60 m`）与 §4 那张表都要按新基线重跑**。⚠️ 本页**不重跑**（那是 ticket 05 的重跑），
> `04` 只做"标作废"。
>
> ★★ **另有一条比"读数过期"更重的后果**（`04` 顺带查出，**推算、未实跑**，见 §7 新加那条）：
> `offboard_smoke.py` 的归一化常数 `t_max_n` 是**该 stub 自带的默认 `11.8`（按电池）**，
> 独立于节点、也不随本页的口径变；而重搭后机型是 `46.3027 N` ⟹ **同一条悬停指令会让 SIH 出 3.92 倍推力**，
> `⑤` 那两条判据（"只给悬停推力 ⟹ 不掉高度"、"从电机反推的推力 ≈ 机重"）**会红**。
> 修它属 ticket 05 / 10 的域，`04` **不擅自改**。
>
> ☠️ **载体已换（2026-10-12，[`phase4/13`](../../.scratch/phase4/issues/13-mavros-downlink-switch.md) §三）**：
> 本页描述的 `offboard_smoke.py` 走的是 **px4_msgs + uXRCE-DDS**
> （`/fmu/in/vehicle_rates_setpoint` + `OffboardControlMode` 心跳 + `/fmu/in/vehicle_command` 解锁/切模式）；
> 该节点**已改成 mavros 载体**（`/mavros/setpoint_raw/attitude` + `/mavros/cmd/arming` +
> `/mavros/set_mode`，闸门源换成 `/mavros/state`）。⟹ 本页以下内容**一律按"旧载体"读**，
> 其中**这几处在新载体上没有对应物**（逐项点名，理由见 [`sign-map.md`](sign-map.md) §2.3.1 与 `13` §三③）：
>
> 1. **§1 那个跑法块** —— 缺 `--t-max`（该参数 2026-10-12 起**必须显式给**，不给就拒绝启动，
>    今天的正确值 46.3027 N = `m·g/hover_thrust`），且 `--qos` 的取值集合已按新载体重核；
> 2. **§3.2「QoS 双向可通」** —— 新载体的闸门源 `/mavros/state` 由 mavros 以 **reliable** 发布，
>    "订阅声明错 ⟹ 零回调"那一档**造不出来**（两个取值都收得到）；
> 3. **§3.3「时间戳：一条生成代码，三个后果」整节** —— `SET_ATTITUDE_TARGET` 的 `time_boot_ms`
>    在飞控那段 handler（`mavlink_receiver.cpp:1509-1599`）里**一次都没被读** ⟹
>    单位写成零/毫秒/纳秒**注入效果逐位相同**，`--timestamp-unit` / `--timestamp-shift-us`
>    **已从节点里删掉**（留着会让操作者以为"这一档也测过了"，而这一场里它**再也取不到第二个值**）；
> 4. **§4 那张 12 场表里 `ts-ms` / `ts-ns` / `ts-zero` / `ts-lag-1s` / `ts-lag-2s` 五场**，
>    以及 `rate-step-broken` 那两场的 `--break pitch-sign` —— 后者已改名 `pitch-preconvert`，
>    含义从"符号写反"变成 **N2**（我们自己多做了一次预转换 ⟹ 转两次 = 不转，
>    飞控最终收到的分毫不差，"这不是一个会被飞控行为测出来的错"）；
> 5. **§3.5 ⑤ 那两条判据**的另一半（归一化）—— 读数同时受上面 2026-10-09 那条基线作废影响。
>
> ★★ **驱动脚本本身也还是旧载体的**：[`verify_offboard_smoke.py`](../../tools/px4-sitl/verify_offboard_smoke.py)
> 的场景表（`:544-564`）照样传 `--timestamp-unit` / `--timestamp-shift-us` / `--break pitch-sign`，
> 闸门源与话题也钉在 `/fmu/*` 上 ⟹ **本页的命令今天跑不通**（argparse 直接拒），
> 不是"跑出来红了"。同一条也适用于 [`test_verify_judges.py`](../../tools/px4-sitl/test_verify_judges.py):123
> 与 `src/bench/bench/fake_px4.py`。这三处的归属是 `13` **§四**（夹具与 seam 1），
> ⚠️ **本页不重跑、也不改驱动**（那是 §四 / 05 的重跑），本段只做"标作废"。

---

## 0. 先读这一段：三个"一句话"

1. **判据是"PX4 真的照做"，不是"话题在"**：`ros2 topic list` 里有 `px4_msgs` 的话题
   什么都证明不了 —— QoS 声明错时话题照样列出来、`listener` 照样有数据，但**回调一条都不进**。
   所以六项判据全部落在**可观测量**上：PX4 的 `nav_state`、SIH 真值高度、PX4 解出的电机指令、
   桌面上那架飞机的姿态。**"跑起来没报错"不算过。**
2. **四个静默失败各有一次独立观测**（都不崩、不报错，只表现为"没反应"或"朝错的方向加速"）：
   订阅 QoS 声明错 ⟹ **零回调**；时间戳单位错 ⟹ **一声不吭照飞**；时间戳往过去挪够多 ⟹
   **进不了 OFFBOARD 且不报错**；`thrust_body[2]` 符号反 ⟹ **电机全零、原地不动**（PX4 把正推力夹到 0）。
3. **本张订正了一处文档结论**：[`phase2-implementation.md`](phase2-implementation.md) §6.5 写
   "传成纳秒……**PX4 可能直接丢弃**"，实测**不会丢弃、也不会提示**（§6）。

---

## 1. 交付物一览

| 交付物 | 路径（入库） | 说明 |
|---|---|---|
| 被测节点（stub） | [`src/nmpc/nmpc/offboard_smoke.py`](../../src/nmpc/nmpc/offboard_smoke.py) | 50 Hz 发心跳 + 角速率 setpoint；解锁 / 切模式；**无模型、无代价、无约束** |
| 符号映射（唯一实现） | [`src/nmpc/nmpc/sign_map.py`](../../src/nmpc/nmpc/sign_map.py) | 内部量 → PX4 四字段；ticket 03 的代码侧契约，**后续节点共用这一份** |
| 指令时间轴 | [`src/nmpc/nmpc/commands.py`](../../src/nmpc/nmpc/commands.py) | 分段给 `(悬停倍数, ω)`；纯函数、不读状态、不求解 |
| 验收驱动 | [`tools/px4-sitl/verify_offboard_smoke.py`](../../tools/px4-sitl/verify_offboard_smoke.py) | 12 场：起 SITL + agent、起 DUT、采 pxh、判据、报告、`--table` 出汇总表 |
| 取证工具 | [`tools/px4-sitl/px4_console.py`](../../tools/px4-sitl/px4_console.py) | 用 pty 驱动 pxh；本张给它加了**字节游标** `mark()` / `text_since()` |
| 取证工具的离线单测 | [`tools/px4-sitl/test_px4_console.py`](../../tools/px4-sitl/test_px4_console.py) | 4 例，不启 SITL（为什么必须有：§5.3） |
| **判据**的离线单测 | [`tools/px4-sitl/test_verify_judges.py`](../../tools/px4-sitl/test_verify_judges.py) | 14 例，不启 SITL：把**能脱离飞机判的**判据（⑤ 的高度、报告层两条跨档）喂人造数。判据本身也要有判据 —— 跑一次 1 分钟，没人会去试判据错没错 |
| 逻辑单测（零 ROS） | [`src/nmpc/test/test_sign_map.py`](../../src/nmpc/test/test_sign_map.py)、[`test_commands.py`](../../src/nmpc/test/test_commands.py) | 52 例；`test_sign_map` 解析的是 **`sign-map.md` 的 `sign-map:canonical` 块** |

> ☠️ **本块按旧载体写，今天跑不通**：`ros2 run … offboard_smoke.py` 少了 `--t-max`
> （2026-10-12 起必填）⟹ 节点会拒绝启动；`verify_offboard_smoke.py` 仍按 `/fmu/*` 起 DUT。见本页顶的载体作废块。

```bash
# 一次跑完 12 场（每场一套全新 SITL + agent，约 1 分钟/场），末了打一张汇总表
source /opt/ros/humble/setup.bash && source ~/px4_msgs_ws/install/setup.bash && source install/setup.bash
python3 tools/px4-sitl/verify_offboard_smoke.py --json /tmp/offboard-smoke/obs.json --table

# 只跑某几场；或从观测文件**重放**（不起 SITL，几秒）—— 改判据时用这个，别重跑飞机
python3 tools/px4-sitl/verify_offboard_smoke.py --only baseline,flow-break
python3 tools/px4-sitl/verify_offboard_smoke.py --replay /tmp/offboard-smoke/obs.json --table

# 零 SITL 的单测（取证工具 + 判据本身）
python3 tools/px4-sitl/test_px4_console.py
python3 -m pytest tools/px4-sitl/test_verify_judges.py -q
python3 -m pytest src/nmpc/test/test_sign_map.py src/nmpc/test/test_commands.py -q

# 自己起了 SITL 之后，单独把 stub 接上去（不带夹具）
ros2 run nmpc offboard_smoke.py --phases lift:1.2:1.2:0,0,0 --phases hover:8:1.0:0,0,0
```

> ★ **文档里那张表是 `--table` 的输出**（§4），不是手抄的 —— 手抄必然与 JSON 漂移。
> `--table` 是**判据报告之外再多打一张表**（报告与退出码照旧），要只取表就挑 `|` 开头的行。
> 原始观测（含每一帧的 `timestamp`、每一次采样的墙钟）在 `--json` 的那个文件里。
> ⚠️ `--replay … --table` 一度**绕过报告层**（表照打、跨档判据一条不跑、退出码恒 0）——
> 那正是本节的命令，已修：两条路共用一个出口，退出码一律来自 `report()`。

---

## 2. 被测节点：**刻意没有模型**

每一拍（50 Hz）只做三件事：

1. `commands.command_at(程序, OFFBOARD 生效后的秒) → (推力 N, ω_FLU)`；
2. `sign_map.px4_rates_setpoint(ω, F, T_max) → 四个字段`；
3. 发 `/fmu/in/vehicle_rates_setpoint` + `/fmu/in/offboard_control_mode(body_rate=True)`。

**"没有模型"是拿 grep 验的**，不是声称 —— 在三个交付文件里搜
`acados|casadi|ocp|cost|constraint|solve|model|jacob`：

```
src/nmpc/nmpc/offboard_smoke.py:36:    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
```

**唯一一处命中是 `Path.resolve()` 里的 "solve"。** 没有状态、没有代价、没有约束、没有求解器
（也不需要 acados / `ACADOS_SOURCE_DIR`）—— 这样"符号反了"和"模型错了"才可能被分开归因。

三条**刻意的**设计（写在 `commands.py` 的模块 docstring 里，此处只列结论）：

- **时间是"OFFBOARD 生效后"的相对秒**：模式切换花掉的几秒否则会吃掉第一段指令；
- **走完不停发**：停发会触发 `COM_OF_LOSS_T` 失效保护，而那是**断流实验**要单独观测的事，
  不能混进别的场景里；
- **闸门开在"已发布时长"上**（`--pre-stream`，默认 1.5 s）：判据要的是
  "切 OFFBOARD 之前 setpoint 已以 ≥2 Hz 发了 ≥1 s"，量的是**发出去的东西**。
  ⚠️ 这个旋钮本张**曾经没接线**（`Config` 里有、`--pre-stream` 能改、没人读），
  于是实测 0.78 s 就进了 OFFBOARD —— 见 §5.6。

---

## 3. 六项判据（每一条都是**可观测量**）

### 3.1 ① `px4_msgs` 与固件对得上 —— 判据是"PX4 真的照做"

DUT 以 50 Hz 发心跳 + rates setpoint，PX4 收到后把 `nav_state` 置为 14（OFFBOARD），
**电机真转、飞机真离地**：baseline 从地面真值 z=+0.0002 m 升到 **−1.60 m**（离地 1.60 m），
末段 vz=+0.013 m/s。★ "话题在"什么都证明不了 —— 见 3.2 的零回调。

票里附带的那条（"在此之前 setpoint 已以 ≥2 Hz 持续发布 ≥1 s"）单独判：
**DUT 自报首帧 → PX4 回话 `nav_state=14` 隔了 2.06 s、其间 103 帧 ⟹ 50.0 Hz**
（闸门 `--pre-stream` 默认 1.5 s，其余是解锁 + 切模式的往返）。

### 3.2 ② QoS 双向可通：订阅 `/fmu/out/*` 必须显式 BEST_EFFORT

> ☠️ **这一档在 mavros 载体上没有对应物**（2026-10-12，`13` §三③）：闸门源 `/mavros/state`
> 由 mavros 以 reliable 发布 ⟹ `--qos reliable|best_effort` 两个取值**都收得到**，
> "声明错 ⟹ 零回调"造不出来。本节的证据只对旧载体成立。

PX4 在 DDS 边界上给 `/fmu/out/*` 建的是 best_effort 流（`dds_topics.h.em:62`）。
把订阅侧写成 RELIABLE（**ROS 2 的默认值**）的后果是 —— **一条回调都不进**：

| | 订阅侧 | 收到的 `vehicle_status` | DUT 侧见过的 `nav_state` | 解锁 / 模式命令 | 结果 |
|---|---|---|---|---|---|
| `baseline` | BEST_EFFORT | **25 条** | `[-1, 4, 14]` | 发出去、成功 | 进 OFFBOARD、起飞 |
| `qos-reliable` | RELIABLE | **0 条** | `[-1]` | **0 条 / 0 条** | 收不到状态 ⟹ 不解锁、不切模式 |

而这两场里 `ros2 topic list` 都列得出来、`listener` 也都有数据。**这就是"静默"的含义。**

★ 这张对照表是**报告层的一条真判据**（`report()` 里跨档核对"写对时 >0 条、写错时 ==0 条"），
不是靠在 qos 那一场里写一句"baseline 收到过 250+ 条" —— 恒真的话撑不起结论，
而且那个 250+ 是复核揪出来的**假数**（真值是 25 条）。
（顺带：`qos-reliable` 那场飞机一直贴地（真值 z=+0.0002 m），**EKF 的 z 却报到 −0.46 m** —— 见 §5.7。）

### 3.3 ③ 时间戳：一条生成代码，三个后果

> ☠️ **整节只对旧载体成立**（2026-10-12，`13` §三③）：本节测的三档（单位错、挪到过去、零）
> 全靠 `time_boot_ms` 被飞控读；`SET_ATTITUDE_TARGET` 那段 handler **一次都没读它**
> ⟹ 那三档在新载体上**注入效果逐位相同**，两个开关已从节点里删掉。理由见 [`sign-map.md`](sign-map.md) §2.3.1。

PX4 收下我们的 `timestamp` 时**不是原样存**，而是过一遍生成的 uCDR 反序列化器
（心跳走 `uORB/ucdr/offboard_control_mode.h:87-88`，rates 走 `vehicle_rates_setpoint.h:82-84`，
同一份代码）：

```c
if (topic.timestamp == 0) topic.timestamp = hrt_absolute_time();
else topic.timestamp = math::min(topic.timestamp - time_offset, hrt_absolute_time());
```

`timestamp` 是 uint64、`time_offset` 是 int64 ⟹ `ts − time_offset` 是**无符号**运算。
于是 PX4 记下的永远是 `min(ts − time_offset, hrt)` —— **我们要么被"改写"，要么被这个 `min` 救回来**：

| 这一档 | 我们发出去的 `ts`（实测） | `ts − time_offset` | `min()` 挑中 | PX4 自报的 age | 后果 |
|---|---|---|---|---|---|
| 正确（µs） | `1790595705603052` | 比 `hrt` **大** ≈ Δ | `hrt` | 0.004 s | 正常 |
| 毫秒 | `1790595742193` | **uint64 下溢**成 `1.84e19` | `hrt` | 0.004 s | **静默无害** |
| 纳秒 | `1790595759189081591` | `1.79e18`（不下溢，比 `hrt` 大 12 个数量级） | `hrt` | 0.008 s | 静默无害 |
| `0` | `0` | 走**显式**分支，连 `min()` 都不过 | `hrt` | 0.016 s | 静默无害 |
| 往过去挪 1 s | `…308564` | `hrt − 0.92 s` | `ts − time_offset` | **0.916 s** | 仍 < `COM_OF_LOSS_T`(1 s) ⟹ **贴边进得去**（余量只有 Δ=0.084 s） |
| 往过去挪 2 s | `…323769` | `hrt − 1.82 s` | `ts − time_offset` | **1.823 s** | > 1 s ⟹ `data_is_recent` 永假 ⟹ **进不了** |

（表里是**复核后那次跑**的读数。第一版跑出来的对应两行是 age = 0.793 / 1.815 s —— 同一套代码、
同一组开关，差的正是 Δ 每次重新握手的取值。）

**Δ 是两条独立路径量出来的**（同一件事：会话的 `time_offset` 比真值小一截，0.1 ~ 0.24 s）：

- **正向**（我们发的）：挪 `lag` 秒后 age = `lag − Δ` ⟹ 两次跑分别量到
  **Δ = 0.207 / 0.185 s**（第一版）与 **0.084 / 0.177 s**（复核后重跑）。
  ⚠️ **别把 Δ 当成一个应当逐位相同的常数**：它是"每个场景重新握手时的那次同步滞后"，
  跨场景实测落在 **0.08 ~ 0.24 s**；报告层那条判据只要求"两个 lag 的 Δ 同量级"
  （阈值 0.20 s，同一跑里实测 |差| = 0.094）。
  ⚠️ **那条判据的分辨力有限，别把它当机制的铁证**：只有两个 lag 的点，另一种模型
  `age = c·lag`（c ≈ 0.92，即"根本没有 Δ 这回事"）在这两点上拟合得一样好
  （0.916 / 1.832 对实测 0.916 / 1.823）。机制的铁证是**源码**（`ucdr/<话题>.h` 的
  `min(ts − time_offset, hrt)`）加三条单场判据：age 随 lag 线性、挪 2 s 时越过门槛、
  三档错单位**永远换不了支**。报告层那条只挡"Δ 大幅漂移"这一种大错。
  ⚠️ 只用**同一帧**里的两个数（挪的秒数、`listener` 自报的 age）；跨帧相减会得到一个
  "与挪几秒无关"的假数 —— 那个假数本身就是算法错的自证（§5.4）。
- **反向**（PX4 发的）：`vehicle_status.timestamp` 与**收到时刻**的墙钟差 = **−0.203 s**
  （中位；11 场，范围 −0.176 … −0.242）。
  第 12 场（`ts-lag-2s`）是 −1.174 s —— 那一条 status 是窗口关闭前 1.17 s 收到的，
  量的是"最后一条消息有多旧"而不是同步误差，原因未查明。

⟹ **门槛不是 `COM_OF_LOSS_T`，而是 `COM_OF_LOSS_T + Δ`**：实测"往过去挪 1 s 进得去、挪 2 s 进不去"，
分界落在 **1.1 ~ 1.3 s**（按 Δ 每次的取值）。这也说明**"往过去挪"是个可达的静默失败**：
它不报错、不崩，只表现为"怎么都不进 OFFBOARD"。

### 3.4 ④ 心跳频率远高于最低要求

判据 ≥2 Hz，**实做 50 Hz**：12 场实测 `49.9999 ~ 50.0006 Hz`（例：baseline 566 帧 / 11.30 s ⟹ 50.00008 Hz）。
心跳（`offboard_control_mode`）与 rates setpoint 是**两个**话题，"每拍都发"是它们的共同来源。

### 3.5 ⑤ `thrust_body[2]` 的**符号**与**归一化** —— 一件事一起验两遍

归一化在 `sign_map.py` 里是 `−F / T_max`，`T_max` 按**电池**取 11.80 N（= 4 × 2.95）。

- **悬停请求** = m·g / (4·`SIH_T_MAX`) = 7.355 / 11.80 = **0.6233**；
  PX4 **原样**解出四路电机均值 **0.6230**（差 −0.05 %，那个差是姿态环在分配力矩，所以只比均值）；
- 从电机反推推力：T = 4 × 0.6230 × 2.95 = **7.351 N** vs 机重 m·g = **7.355 N**
  ⟹ "掉不下来"是**有数**的，不是只看高度；
- **高度**（尺子是 SIH 真值）：+0.0002 m ⟹ −1.60 m，末段 vz=**+0.018 m/s**；
  末三个采样点 z = −1.612 / −1.620 / −1.612 m（2 s 内带宽 **8 mm** ⟹ 既不掉高、也不缓升）。

> ⚠️ 这里的读数与 §4 那张表来自**同一次跑**（`/tmp/obs_all2.json`）。上一次跑是
> 均值 0.6228、末段 vz +0.013 m/s、T=7.348 N —— **逐次差在 1 % 量级**，属于噪声，
> 判据用的是容差（±2 % / 带宽）不是逐位相等。

两个**独立**的破坏性对照各占一场（都只改一件事）：

| 场景 | 我们发出去的 | PX4 解出的电机 | 真值后果 |
|---|---|---|---|
| `thrust-sign-broken`（符号反） | `+0.6233`（正的） | **全 0.000**（被夹到下界） | z 恒 +0.0002 m ⟹ **离不了地** |
| `thrust-scale-broken`（`T_max` 只当 75 % 用） | `−0.8311`（= 0.6233 × 4/3） | 均值 0.8305 | 一路爬到 **−8.42 m**、vz = −2.44 m/s（在爬） |

> ★ 归一化常数的**后果**是"同一个悬停指令变成猛爬"—— 这正是参考实现把 `1/T_max`
> 硬编码成 `0.07` 时的静默错误类型（换质量就错 33 %），所以 `sign_map.py` 里
> `t_max_n <= 0` 直接 `ValueError`。

### 3.6 ⑥ 小角速率步进的倾斜方向与 [`sign-map.md`](sign-map.md) §3 一致

指令：FLU `ω_y = +0.5 rad/s` 持续 0.25 s 后回 0（推力保持 1.15 × 悬停）。

| | 线上 `pitch`（DUT 自报） | 末段机头均值 | 末段真值 vx |
|---|---|---|---|
| `rate-step-ok` | `-0.5` ⟹ `pitch = −ω_y` ✓ | **+0.1469 rad**（>0 = 低头） | **+1.186 m/s**（沿机头加速） |
| `rate-step-broken`（`--break pitch-sign`） | `+0.5` | **−0.1480 rad**（抬头） | **−1.221 m/s** |

两场的推力指令完全相同（`−0.7168`）、`ω` 完全相同，**只有符号不同** ⟹ 方向确实由符号决定。
末段真值 z 也几乎相同（−3.32 / −3.26 m）⟹ 两场都正常飞着，只是朝相反方向加速。

⚠️ **这一条不能用 `listener` 读 `vehicle_rates_setpoint` 来验**（§5.3）；
也**不能取 max/min**：SIH 的加速度计噪声 σ=1.7/1.4 m/s² 折算成倾角是 0.1 rad 量级，
而阶跃只给 0.125 rad —— 取极值等于在读噪声，正反两个场景都能"测"出想要的方向（那是自欺）。
取**末段 2 s 均值**：倾角是持续的，噪声是零均值的。

### 3.7 断流实验：心跳是**唯一**的维持机制

形式：悬停中 `SIGKILL` 节点（不是优雅退出）。实测：**杀掉后 1.095 s** PX4 退出 OFFBOARD，
去向 `nav_state=2`（POSCTL，由 `COM_OBL_RC_ACT` 决定）；`COM_OF_LOSS_T` 从 pxh 读出 = **1.00 s**
（`param show`），退出发生在阈值之后。

源码：`modules/commander/HealthAndArmingChecks/checks/offboardCheck.cpp:44-51`

```c
bool data_is_recent = hrt_absolute_time() < offboard_control_mode.timestamp
		      + static_cast<hrt_abstime>(_param_com_of_loss_t.get() * 1_s);
bool offboard_available = (… || offboard_control_mode.body_rate …) && data_is_recent;
```

⟹ **心跳一停，offboard 立刻不可用**；这也解释了为什么 3.3 里"记下的时间戳变旧"能把飞机挡在门外 ——
`data_is_recent` 看的就是**经反序列化器改写后**的那个 `timestamp`。

---

## 4. 12 场一张表（`--table` 的逐字输出，从这个输出里挑 `|` 开头的行）

> ☠️ **表里这七行随载体一起作废**（2026-10-12，`13` §三③）：`ts-ms` / `ts-ns` / `ts-zero` /
> `ts-lag-1s` / `ts-lag-2s` 五档的开关已删除（没有对应物），`rate-step-broken` 那两行的
> `--break pitch-sign` 已改名 `pitch-preconvert` 且**含义换了**（从"符号写反"变成 N2）。
> ★ 表里其余各行是**旧载体**的读数（并且另有 2026-10-09 那条基线作废）—— 本页不重跑。

（2026-09-28 一次跑完，`exit 0`，**12 场 38 条判据全过**（每场另有 0~1 条不计数的说明），
外加报告层 2 条**跨档**判据：QoS 阳性对照 25 vs 0、Δ 一致性；原始观测 `/tmp/obs_all2.json`，
含每场 DUT 的每一帧、控制台采样原文、每条判据与它的 detail。）

> ⚠️ 这张表是**复核之后那次跑**的表。复核前它写的是"42 项全过" —— 那 42 里有 4 条是
> 恒真的说明行（`ck(..., True, ...)`），撑不起"判据"这个说法，复核指出后改成了
> "说明不计入分母、阳性对照挪到报告层做"，所以数字从 42 变成 **38 条判据 + 4 条说明 + 2 条跨档**。

| 场景 | 这一场的开关 | 通过 | DUT 自报：首帧→OFFBOARD | 窗口口径进入 | 末段真值 z / vz（m, m/s） | PX4 解出的电机（末段） | 本场要点 |
|---|---|---|---|---|---|---|---|
| `baseline` | `（默认开关）` | 10/10 | 2.06 s（50.0 Hz） | 4.23 s | -1.599 / +0.018 | `m0=+0.647 m1=+0.596 m2=+0.623 m3=+0.626` | 566 帧 / 11.3 s ⟹ 50.0 Hz |
| `qos-reliable` | `--qos reliable` | 3/3 | **没进** | — | +0.000 / +0.000 | `—` | 订阅 QoS 声明错 ⟹ **零回调**、不解锁、不切模式 |
| `ts-ms` | `--timestamp-unit ms` | 2/2 | 2.06 s（50.0 Hz） | 4.23 s | — | `—` | 单位错但**静默无害**：照进、照飞 |
| `ts-ns` | `--timestamp-unit ns` | 2/2 | 2.08 s（50.0 Hz） | 4.23 s | — | `—` | 单位错但**静默无害**：照进、照飞 |
| `ts-zero` | `--timestamp-unit zero` | 2/2 | 2.06 s（50.0 Hz） | 17.68 s | — | `—` | 单位错但**静默无害**：照进、照飞 |
| `ts-lag-1s` | `--timestamp-shift-us -1000000` | 3/3 | 2.04 s（50.0 Hz） | 4.18 s | — | `—` | 挪 1 s ⟹ 记下的 age≈0.8 s **仍在** `COM_OF_LOSS_T` 之内 ⟹ 贴边进得去（余量只剩 Δ） |
| `ts-lag-2s` | `--timestamp-shift-us -2000000` | 3/3 | **没进** | — | — | `—` | 记下的心跳 age > `COM_OF_LOSS_T` ⟹ 进不了 |
| `rate-step-ok` | `（默认开关）` | 3/3 | 2.04 s（50.0 Hz） | 4.21 s | -3.321 / -1.007 | `—` | 机头末段均值 +0.1469 rad ⟹ 真值 vx +1.189 m/s |
| `rate-step-broken` | `--break pitch-sign` | 3/3 | 2.08 s（50.0 Hz） | 4.26 s | -3.259 / -1.001 | `—` | 机头末段均值 -0.1480 rad ⟹ 真值 vx -1.230 m/s |
| `thrust-sign-broken` | `--break thrust-sign` | 3/3 | 2.06 s（50.0 Hz） | 4.24 s | +0.000 / +0.000 | `m0=+0.000 m1=+0.000 m2=+0.000 m3=+0.000` | 符号反 ⟹ 电机**全零**、原地不动 |
| `thrust-scale-broken` | `--break thrust-scale --break-scale 0.75` | 2/2 | 2.04 s（50.0 Hz） | 4.20 s | -8.416 / -2.437 | `m0=+0.862 m1=+0.803 m2=+0.822 m3=+0.835` | 归一化常数错 ⟹ 推力过大、一路爬到真值 8.4 m |
| `flow-break` | `（默认开关）` | 2/2 | 2.06 s（50.0 Hz） | 4.22 s | — | `—` | 断流后 1.095 s 退出 ⟹ 2(POSCTL) |

> **怎么读这张表**（三处容易误读的地方）：

1. **"DUT 自报：首帧→OFFBOARD"才是可靠的进入时刻**（= 节点自己的日志里，
   第一帧 → 第一帧看见 `nav_state=14`）。票里那条"≥1 s"判的就是它。
2. **"窗口口径进入"那一列里有 17 s 这种数，是假象**：窗口时钟从 `ros2 run` 起进程
   **之前**就起算，而"起进程 → 第一帧"这一段偶发地会涨到 **15 ~ 18 s**（这一场是 `ts-zero`）。
   它**不是** PX4 慢、也**不是**那几个 `--timestamp-*` 开关的锅：同一批里另外三场 ts-* 只有
   **4.2 s**，上一批里却四场全是 15 s —— 同一代码、同一开关、逐场不同 ⟹ 环境侧（§5.6 有三次对照）。
   同一场的"DUT 自报"始终是 **2.0 ~ 2.9 s**，判据只用它。
3. `qos-reliable` 与 `ts-lag-2s` 的"**没进**"是**判据要求的结果**，不是夹具失败：
   前者收不到状态所以不解锁、不切模式，后者的心跳被永久判为不新鲜（§3.2 / §3.3）。
   两场都**不报错**，这正是"静默失败"的定义。

> 表里"末段真值 z / vz"这一列，`rate-step-*` 两场的 z ≈ −3.3 m **不是**在掉高度而是**在爬**
> （NED 的 z 负向上，vz 也是负的 ⟹ 向上）；两场都正常飞着，差别只在水平方向（§3.6）。

---

## 5. 本张踩出来的坑（都不是抄来的，都有第一手症状）

### 5.1 夹具自己的三处假象 —— 每一条都先弄清机制、再改根因

**（a）上一轮的日志文件还在盘上** ⟹ `wait_first_frame` 立刻返回"看见第一帧"，
采样站其实站在**十几秒之后的空气里**：baseline 第一个真值点的 z 是 **−1.5615 m**
（前一轮飞机在天上留下的数），看着像"还没起飞就已经飞着"。
修法：`Dut.start()` 先 `unlink(missing_ok=True)` 那个 JSONL。修完同一处采样落在 t=0.02 s、
z=−0.071 m（真地面）。**没有放松任何判据。**

**（b）上一套世界拆掉时 PX4 临终那条 `vehicle_status`**（它当时还在 OFFBOARD、还锁着）
可能刚进中间件队列，直到新窗口开完才被收到 ⟹ qos-reliable 窗口 +0.02 s 处凭空出现
`nav_state=14, armed`，把"收不到状态 ⟹ 不解锁、不进 OFFBOARD"这条判据**伪证掉**
（该场景的 DUT 一条状态都没收到，"进 OFFBOARD"只可能是上一场的鬼影）。
修法：开窗口**之前**先 `spin` 0.5 s 把队列放干。

**（c）等控制台输出的游标**：第一版用"当时的渲染文本长度"当游标，而渲染**不是增量的**
（pxh 的 `ESC[2K\r` 常跨两次 `read()` 到达）。实测：喂完 `…TOPIC: foo\n` 时长度 35，
再喂半截 `ESC[` 变 37，再喂 `2K\rTOPIC: bar\n` —— 两截合成一个完整序列一起被吃掉，
从 37 起切片只剩 `'PIC: bar\n'`，**新打印的 `TO` 被整个跳过**，锚行首的 `^TOPIC:` 永远等不到
（症状：白等满超时、重试才成功；baseline 第一处采样就是这样站到 16 s 之后的）。
修法：改成**字节游标**（`_raw` 只增不减），并把这条钉成离线单测
（`test_px4_console.py` 里有一条**故意重现这个 bug** 的反例）。

### 5.2 `pxh` 的 `listener` 有三个行为陷阱（验收脚本靠它取证，必须记下来）

- **选项必须在话题名之后**：`listener foo -n 1` 对，`listener -n 1 foo` 会被当成别的东西；
- **一个实例都没有时只印 `never published`，而且没有 `TOPIC:` 头**
  （`topic_listener/listener_main.cpp:87-89`）⟹ 等 `^TOPIC:` 的采样会白等到超时，
  看起来像"话题没有数据"，其实是"连实例都没建"；
- 多实例分支有 `MESSAGE_TIMEOUT_S = 2.0` 的放弃时限。

### 5.3 `vehicle_rates_setpoint` 是**共用**话题 —— OFFBOARD 之前那帧不是我们的

`mc_att_control_main.cpp:319-324` 在**姿态模式**下每一拍都在发 `vehicle_rates_setpoint`；
`mc_rate_control/MulticastRateControl.cpp:174` 只在 acro/manual 下重发。
于是 OFFBOARD 生效**之前**，话题上最新的帧是 `mc_att_control` 的（≈0），
不是我们发的。实测：⑥ 想从 `listener` 读我们那 0.25 s 的阶跃，读到的是
`pitch=+0.024`、`thrust_body[2]=−0.001` —— **看着像"我们的 setpoint 根本没进去"**。
修法：⑥ 的"线上值"判据改看**节点自己的日志**（那是它真的发出去的）+ 实测倾角。

### 5.4 混时钟相减（一处**算法**错，症状是"两个不同档位测出同一个数"）

ts-lag 的机制判据第一版拿"整段日志**中位数那一帧**的 `ts`"减"**采样那一帧**读回的 `hrt`"
—— 两帧隔好几秒，等于在比两个不同时刻：挪 1 s 与挪 2 s 都量出 **−2.27 s**。
那个"**与挪几秒无关**"本身就是算法错的自证。
修法：只用量**同一帧**里的两个数（`listener` 自报的 age 与程序里挪的秒数），
"两档的 Δ 一致"改成在**报告层**对一次（单场景看不到跨档）。

### 5.5 NED 的符号滑了一次（我自己，不是代码）

⑥ 里真值 `z=−4.49 m`、`vz=−1.02 m/s`，我一度当成"飞机在掉" —— NED 的 z 负向上，
它其实**在爬**，而且与"低头 0.13 rad"完全自洽。数字与判据都没错（判据本来就按 NED 写），
错的是我读报告时的那一眼。

### 5.6 `--pre-stream` 是个**没接线**的旋钮

`Config.pre_stream_s = 1.5`（注释写着"判据要求 ≥1 s"）、`--pre-stream` 能改它，
但**没有任何代码读它**。于是实测：首帧后 **0.78 s** PX4 就进了 OFFBOARD ——
票里的"在此之前 setpoint 已以 ≥2 Hz 持续发布 ≥1 s"**其实没被满足**，而报告全绿。
修法：闸门开在 `streamed_s = (now − first_publish_ns)` 上（解锁与切模式两个分支都用它），
并在验收脚本里加一条判据，量"首帧 → PX4 回话 nav_state=14"的秒数与其间帧数。
★ 这条判据**只能用 DUT 自己的日志**量：窗口时钟从 `ros2 run` 起进程**之前**就起算
（`t_before = rec.now()` 紧接着启动 DUT），混进去会把"进入 OFFBOARD"读成 17.7 s
—— 那是"起进程 → 第一帧"的耗时，不是 PX4 慢。

**这一段耗时没能找到确定性解释，把三次对照摆在这儿**（免得后面有人再从头猜一遍）：

| 量的是什么 | 数（秒） |
|---|---|
| 批次里"窗口 → 第一帧"，12 场中的 8 场 | **2.12 ~ 2.19** |
| 批次里同样这一段，ts-* 那 4 场 | **14.97 ~ 15.53** |
| 复核后**同一批重跑**，ts-* 那 4 场 | 三场 **2.1 ~ 2.2**、一场（`ts-zero`）**15.6** ⟹ 逐场不同 |
| `--only ts-ms` 单独跑（同一场、同一开关） | **2.2**（窗口口径 4.24 − 自报 2.04）⟹ **不是开关的锅** |
| `--only baseline,qos-reliable,ts-ms` 里的 ts-ms | 15.21 ⟹ 前一场是"收不到状态那场"时会慢 |
| `--only baseline,rate-step-ok,ts-ms` | baseline **15.02**、rate-step-ok **2.15**、ts-ms **14.92** ⟹ **同一批次里逐场不同** ⟹ 既不是开关、也不是位次 |
| 同一台机器上 `python3 -c "import rclpy; from px4_msgs.msg import …; create_node; create_publisher"` ×6 | **0.77 ~ 0.79**（极稳）⟹ **不是 import/建进程的固定成本** |

⚠️ 所以**别**把它当成 python 慢或 PX4 慢 —— 唯一站得住的用法还是"窗口口径不可用于定时"。
可疑方向只剩环境侧（**被 SIGKILL 的 DUT 留下的 Fast-DDS 共享内存段**／与 SITL 启动抢 CPU），
本张**没有证据**，留作夹具侧的悬案（[06](../../.scratch/phase2/issues/06-mpc-node-fake-px4-fixture.md) 建夹具时会再撞上）。

### 5.7 仪器结论：**EKF 的 z 不能当尺子**

z0 同构机上没有气压计、没有位置源（§SIH 那套只发真值，而真值话题不导出到 DDS），
所以 EKF 的高度是把 σ=1.4 m/s² 的加速度计噪声积分出来的**随机游走**：地面静息十几秒就能漂
**−0.5 ~ −0.78 m**。拿它当尺子 = 把尺子自己的漂移记成飞机的高度变化。
⟹ **⑤ 判"不掉高"只用两样东西**：SIH 真值 + **PX4 自己解出的电机指令**。
（同一场里 EKF 的**水平** vx 却与真值吻合得很好：+1.115 vs +1.105 m/s —— 所以这不是"EKF 坏"，
而是"**没有那一路观测量**"。）

---

## 6. 与文档的偏差：§6.5 的「会被丢弃」被推翻

[`phase2-implementation.md`](phase2-implementation.md) §6.5 原文：

> `Clock().now().nanoseconds / 1000`。PX4 全用微秒。传成纳秒的话时间戳会大到离谱，
> PX4 可能直接丢弃。**这个错误不会报错，只会"没反应"。**

**实测：不会丢弃，也没有任何提示。** 三档（毫秒 / 纳秒 / 0）**全部照进 OFFBOARD、照飞**。

原因是反序列化器**根本没用我们发的那个数**：`uORB/ucdr/vehicle_rates_setpoint.h:82-85`

```c
memcpy(&topic.timestamp, buf.iterator, sizeof(topic.timestamp));
if (topic.timestamp == 0) topic.timestamp = hrt_absolute_time();
else topic.timestamp = math::min(topic.timestamp - time_offset, hrt_absolute_time());
```

- 毫秒值（≈1.79e12）减 `time_offset`（≈1.79e15）在 **uint64 下溢**成大得离谱的正数；
- 纳秒值（≈1.79e18）这次不下溢，但比 `hrt` 大 12 个数量级；
- 0 走**显式**那一支。

三种情形下 `min(·, hrt)` 挑的都是 **`hrt`**（或直接赋值 `hrt`）⟹ PX4 记下的时间戳
**比"正确单位"还新鲜**（实测 age = 0.000 ~ 0.004 s）。**单位错的后果不是被丢弃，而是被无声改写。**

> ★ 该段另外两处结论（微秒是 DDS 边界的硬约定、`msg/VehicleRatesSetpoint.msg:1` 逐字含
> `(microseconds)`）**仍然成立**，本张只是把"丢弃"这一句改掉了 —— **不是推翻整节**。

---

## 7. 未测 / 残留 / 交给下一张

- **未测**：本张没有闭环 —— 没有位置/速度反馈、没有轨迹跟踪、没有风扰与参数失配。
  闭环归 S5（ticket 09/11）。
- **未测**：`MPC_TILTMAX_AIR` 之类的限幅在**本张的指令范围内**（|ω| ≤ 0.5 rad/s、倾角 ≤ 0.14 rad）
  根本没被碰到 —— 大机动下的推力饱和不在本张判据里。
- **☠️ 基线已换 ⟹ 本张的工具会红（2026-10-09，ticket 21 / phase3 的
  [`04`](../../.scratch/phase3/issues/04-rebuild-sitl-plant-to-measured-baseline.md) 查出，**推算未实跑**）**：
  [`src/nmpc/nmpc/offboard_smoke.py:75`](../../src/nmpc/nmpc/offboard_smoke.py#L75) 的 `t_max_n` 默认
  `11.8`（注释写"按电池"）是该 stub **自己的常数**，不随被控对象变。重搭前它与机型**巧合相等**，
  所以本张的 `⑤` 两条判据是"两错相消"才过的；重搭后机型是 46.3027 N ⟹ 同一条悬停指令出 3.92 倍推力，
  `altitude_held(...)`（`verify_offboard_smoke.py:944`：要求 `vz > −0.05`）与"反推推力 ≈ 机重"**都会红**。
  ⚠️ 这是**从算式推出来的**，没有实跑 —— 真要落地得先跑一场。修它要动 ticket 05 / 10 的文件，
  `04` **没有顺手改**（改别的票的文件要单独提），在此点名并交操作者裁定。
- **残留（已定位、未修）**：[`tools/px4-sitl/verify_z0_plant.py`](../../tools/px4-sitl/verify_z0_plant.py)
  仍在用**文本偏移**口径的 `wait_for(since=)`（§5.1c 的那个毛病）。它是 **ticket 08 的文件**，
  本张**没有顺手改**（改别的票的文件要单独提），在此点名。
- **交给 ticket 06（假 PX4 夹具）**：`offboard_smoke.py` 上的
  `--break none|thrust-sign|thrust-scale|roll-sign|pitch-sign|yaw-sign`、
  `--timestamp-unit us|ms|ns|zero`、`--timestamp-shift-us`、`--qos`、
  `--pre-stream` 这一组**故障注入开关**，本质是**夹具**的能力，不是节点的业务；
  它们应该搬到夹具那侧（真 PX4 那条路只保留 `--pre-stream` 与 `--phases`）。
  本张把它们留在节点里是**为了在真 PX4 上做对照实验**，代价是节点带了一身"取证用的旋钮"。
- **交给 ticket 09/11（S5 闭环）**：本张排掉的四项（QoS / 时间戳 / 符号 / 归一化）是闭环失败时的
  第一批嫌疑对象；**先看本页的表**再怀疑模型。
