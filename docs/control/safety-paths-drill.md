# 安全路径演练流程 —— 六条实测 + 回退 (A) 级（Phase 4 / ticket 08）

> **归属**：ticket [`phase4/08`](../../.scratch/phase4/issues/08-safety-paths-and-fallback-drill.md)（`ready-for-human`）。
> 本页把票面**上半六条实测**与**「回退 (A) 级」那个动作本身**写成可执行流程 —— 后者是
> [`mpc-node-seam1.md`](mpc-node-seam1.md) **§12.3③** 点名的空缺（"这一步本身没有成文流程"）。
> **本页不新增判据、不改代码**：判据全部回链（见下表）。
>
> | 要什么 | 去哪（本页**不复述**） |
> |---|---|
> | (C) 级链的成员表、起法、**三条未决** | [`mpc-node-seam1.md`](mpc-node-seam1.md) §12 |
> | 实飞前的 12 条检查项 | [`px4-interface.md`](../reference/px4-interface.md) §8 |
> | "这级算不算过" / 何时**直接回退** | [`ladder-acceptance-criteria.md`](ladder-acceptance-criteria.md) §1、§5.1、§5.2 |
> | 通道分配的**结论**与依据 | [`params-changes.md`](../reference/params-changes.md) §H |
> | 求解失败的机制、台架配方、证据表 | [`mpc-node-fallback.md`](mpc-node-fallback.md) §2/§5/§6/§7 |
> | 装桨前必过的自检 | [`first-flight-config.md`](first-flight-config.md) §5 ③ |
> | 现场怎么起链、怎么停任务、日志 token | [`operation_manual.md`](../../operation_manual.md) §1.2/§3/§5.3 |

---

## 0. 这一页怎么读

**四条口径，按重要性排**：

1. ★★ **命令是现读出来的，不是跑过的。** 每一条都从仓库里的第一手来源读出
   （`run/` 层、launch/节点源码、`run_sitl.sh`、参数快照），每条**都在「动作」里带出处**。
   **本页一次都没起过、没飞过** —— 首次上场时按
   [`px4-interface.md`](../reference/px4-interface.md) §8 的 12 条逐条对，
   **不要照抄本页的次序**。
2. ★★ **读数的唯一出处是快照，且被守卫钉住。** 本页凡写出参数读数的，只有两种形状：
   表格行 `| `参数名` | `数` | …` 或散文里的 `` `参数名` = `数` ``（★ 等号两边各一个空格，
   是这个约定的形状，别改写）。全部取自
   [`docs/assets/z0.params`](../assets/z0.params)（**2026-10-10 现场导出**），并逐条被
   `run/tests/test_safety_paths_drill.py` 钉在那份文件上 ⟹ **谁改了快照里的值，这一页就会变红**。
   正文因此**只引用不重写**这些数。
3. **判据不在这里**：每条只写"什么算过"这一栏，取值与阈值回链 §5.2 / §8 的那两份。
4. ★ **今天 (C) 级那一半全都做不了**：整条 (C) 级下行在 z0 上**不存在**（§1 的"今天不成立"那格）
   ⟹ **§2.1 / §2.3 / §2.4 各自的 (C) 侧**、**§2.5 整条**、**§3.1 的第 2/3 步**今天都只读得到
   "怎么做"，**走不了一遍**。今天真能走的：(A) 级那三条 ＋ §3.1 的第 1/4/5 步（§2.6、§3.1 末）。

**为什么要有这一页**：票面原话是「**没演练过的回退路径不算回退路径**」，而"演练过的那个操作"
在此之前**只存在于会话记忆里** —— [`ladder-acceptance-criteria.md`](ladder-acceptance-criteria.md)
§5.2 末尾把这一步指向 ticket 08，而 ticket 08 又指向"那个操作"，**两头都没有正文**。
本页补的就是这条边。

---

## 1. 演练之前：两条链互斥

| | 下行话题 | 谁在发 | 起法 | 今天 |
|---|---|---|---|---|
| **(A) 级**（Phase 1） | `/mavros/setpoint_raw/local` | `control`（参考由 `traj` 给） | `run/z0.py up` / [`operation_manual.md`](../../operation_manual.md) §3 | ✅ 成立 |
| **(C) 级**（本阶段） | `/fmu/in/vehicle_rates_setpoint` | `mpc_node` | 成员表见 [`mpc-node-seam1.md`](mpc-node-seam1.md) §12.1 | ⛔ **不成立**（§12.3①） |

**话题不同，但两条都喂 PX4 的 OFFBOARD ⟹ 同时跑 = 两个驾驶员。**
⟹ **任何一条演练开始之前，先确认另一条不在发 setpoint**（判据见 §3.1 第 1 步）。

**两条链共用一条前缀**（这一点决定了切换的代价，第一手 §12.1 的成员表）：
`mavros` / `vrpn_listener` / `mocap` 三者**两条链都要**；
**只有两端不同** —— (A) 级多 `traj` + `control`，(C) 级多 `MicroXRCEAgent` + `mpc_node`。

---

## 2. 六条逐条

> 每条的固定五栏：**动作**（谁做什么、命令、出处）/ **期望与出处** / **什么算过** /
> **留什么证据** / **今天能不能做**。
> ⚠️ **六条里没有一条是"跑一遍没事"就算过** —— 票面原话："每条实测留下证据（读数 / 日志 / 现象），不是"试过了没问题""。

### 2.0 本页引用的参数读数（**汇总表** —— 正文各处就地引用的是**同一批数**，不另起一套）

| 参数 | 快照值 | 这一页为什么引它 |
|---|---|---|
| `RC_MAP_FLTMODE` | `6` | §2.2：模式开关在 ch6；开始通道（ch9）**不得**与它共用 |
| `RC_MAP_KILL_SW` | `8` | §2.2：急停 = ch8 高值 = **停桨**，不是停任务 |
| `RC_MAP_ARM_SW` | `5` | §2.2：保留的冗余硬件解锁路径 |
| `COM_RC_IN_MODE` | `0` | §2.2：★ 它要是 `3`，**整个遥控开关层静默失效**（[`params-changes.md`](../reference/params-changes.md) §I.3）⟹ 打杆与 ch6/ch8 全是空的 |
| `COM_RC_OVERRIDE` | `3` | §2.1：打杆能踢出 OFFBOARD 的参数基础 |
| `COM_RC_STICK_OV` | `20` | §2.1：★ 语义是**变化率**（%/s），不是"打杆 20%"（[`params-changes.md`](../reference/params-changes.md) §H.11） |
| `COM_OBL_RC_ACT` | `0` | §2.1/§2.3：离开 OFFBOARD 之后落回 **Position** |
| `COM_OF_LOSS_T` | `1.0` | §2.3/§2.4：心跳断流多久之后 PX4 自己退出 OFFBOARD |
| `NAV_RCL_ACT` | `1` | §2.2/§5：遥控一丢是**就地保持（Hold）**、不返航（[`params-changes.md`](../reference/params-changes.md) §A1 已落地） |
| `UXRCE_DDS_CFG` | `0` | §1/§2.5：(C) 级下行**今天不存在** |

★ 出处全部是 [`docs/assets/z0.params`](../assets/z0.params)（2026-10-10 导出）。
⚠️ **引用这份文件时，可查的是参数名，不是行号**（每次重导出行号整体平移，其文件头的说明）。

### 2.1 打杆接管 —— 立刻停发、且**不自己切模式**

**动作**：飞机在 OFFBOARD、程序在发 setpoint 时，**快速推任一根杆**。
出处：[`operation_manual.md`](../../operation_manual.md) §1.2 (b)（现场那一版）；
通道与参数见 §2.0。

**期望与出处**：

- **(A) 级**：我们 **0.3 s 内停发**、退到 `MANUAL/RC_OVERRIDE`、**不请求任何模式**；
  PX4 按 `COM_OBL_RC_ACT` = `0` 落回 **Position**。出处：
  [`operation_manual.md`](../../operation_manual.md) §1.2 (b)；
  代码侧 `src/control/include/control/fsm.hpp`（`RC_OVERRIDE` 那一支）；
  用例 `src/control/test/test_control_failsafe.py::test_打杆立刻停发_退到_RC_OVERRIDE_且不抢回_OFFBOARD`。
- **(C) 级**：`mpc_node` **只认状态、不认通道** —— `vehicle_status.nav_state` 一离开 OFFBOARD
  就停发 setpoint（`StopConfig.stop_on_nav_state_exit`，`src/nmpc/nmpc/node_core.py`）。
  它**没有** `VehicleCommand` 发布器、**不发**任何模式命令（§12.2）。
  ⚠️ **两处必须知道的不对称**（都是第一手的代码事实，别当成实机读数）：
  1. **心跳照发**：`heartbeat_always` 默认 `True` ⟹ 闸门关了以后
     `/fmu/in/offboard_control_mode` **一帧不少**，停的只有 `/fmu/in/vehicle_rates_setpoint`
     （`src/nmpc/nmpc/mpc_node.py` 的 `_tick` ①）。⟹ **拿"话题还有帧"当"还在发指令"会读错**。
  2. **"不发 setpoint" ≠ "飞机松手"**：`COM_OF_LOSS_T` 那条判据看的是**心跳**，
     而 PX4 对最后一帧 setpoint 是**无限期锁存**的（[`mpc-node-fallback.md`](mpc-node-fallback.md) §1.1）。

**什么算过**：`/control/state` 落到 `MANUAL/RC_OVERRIDE`（(A) 级）；PX4 的模式落回 Position；
**日志里没有任何一条"我们自己发的模式命令"**。

**留什么证据**：`/control/state` 的**原文**（不是转述）+ 日志行首的 `degrade=rc_override`
（形状与 `**` 的坑见 [`operation_manual.md`](../../operation_manual.md) §5.3）
+ 打杆到停发的**时间差**。

**今天能不能做**：✅ 能（要遥控器 + 飞控上电）。
⚠️ "落回 Position" 那一半要求 PX4 **真的在 OFFBOARD 里**，而进 OFFBOARD 要有程序在发心跳
⟹ 这一条与 §2.3/§2.4 是**同一场**里做掉的。
⚠️ **(C) 级那一半今天做不了**（§1 的"今天不成立"），它在台架上验过（§12.2 的引用链）。

### 2.2 急停 / 开始通道的**实测读数**

**动作**（出处 [`operation_manual.md`](../../operation_manual.md) §5.2，本页只搬命令）：

```bash
ros2 topic echo /mavros/rc/in     # 看 channels[8]（ch9，0-based）
```

拨 ch9：`channels[8]` 必须**由低到高**；顺手拨 ch6，确认**模式切换只认 ch6**。

**期望与出处**：

- ch9 = **开始**信号，判据是**上升沿**（低 → 高，去抖 200 ms）——
  出处 [`params-changes.md`](../reference/params-changes.md) §H.1（2026-09-21 操作者裁定）。
- **不与模式开关共用通道**：`RC_MAP_FLTMODE` = `6`、`RC_MAP_KILL_SW` = `8`，
  未被任何 `RC_MAP_*` 占用的是 ch7 与 ch9…ch16（§H.2 的逐条第一手表）。
- **急停 = ch8 高值 = PX4 硬件杀车 = 停桨**，**不是**"取消任务"，程序侧**不读**急停通道
  （§H.1 的「急停」行 + §H.7）。
- ⚠️ **前置**：`COM_RC_IN_MODE` = `0`。若它是 `3`，**整个遥控开关层静默失效**
  （[`params-changes.md`](../reference/params-changes.md) §I.3，2026-09-23 实测）——
  那时打杆、ch6、ch8 **全都是空的**，而"通道读数"这一步
  看起来仍然正常。**这一条是本条演练里最值得盯的一格。**

**什么算过**：ch9 的低 / 高两个读数记下来了，且 ch6 的拨动**只**改模式、**不**产生 ch9 的边沿。
⚠️ 通道错了的话，**第 1 次上升沿会直接去解锁起飞** ⟹ 这一步必须在**解锁之前**做。

**留什么证据**：ch9 的**低/高两个原始读数**各一行 + ch6 拨动时 PX4 模式的**前后**读数。
⚠️ **要试 ch8 必须先拆桨**（[`params-changes.md`](../reference/params-changes.md) §H.8）。

**今天能不能做**：✅ 能（要遥控器 + 飞控上电，**不装桨**）。
⏳ **现状**（照实记，不是我量的）：2026-09-28 那一趟**功能这一半过了**（两次上升沿都受理了），
但**读数从来没被记下来过** —— 操作者裁定**不补量、目测为准**（
[`operation_manual.md`](../../operation_manual.md) §5.2 末尾的状态行）。⟹ 本票要的"实测读数"
这一格**据操作者裁定以目测为准**，但下次上电顺手记一行是一秒的事。

### 2.3 offboard 心跳丢失 → PX4 按预配置动作执行

**动作**：让**发心跳的那个进程**停掉（(A) 级 = 停 `control`；(C) 级 = 停 `mpc_node`），
看 PX4 自己怎么处置。★ 停进程的命令与次序见 §3.1 第 2 步（同一套，别两处各写一遍）。

**期望与出处**：

- 判据是**心跳**、不是 setpoint：`COM_OF_LOSS_T` = `1.0` s 之内没有
  `offboard_control_mode` ⟹ PX4 退出 OFFBOARD、按 `COM_OBL_RC_ACT` = `0` 落回 **Position**。
  出处：`src/nmpc/nmpc/mpc_node.py` 模块 docstring 引的
  `offboardCheck.cpp:40-65` + `modeCheck.cpp:126-134`（z0 真身 `4a0e65f2`，第一手）；
  取舍的现场表述在 [`params-changes.md`](../reference/params-changes.md) §H.7 与
  [`operation_manual.md`](../../operation_manual.md) §1.2 末段。
- ⚠️ **"停发 setpoint" 触发不了这条**：默认档心跳无条件发（§2.1 的不对称 ①）
  ⟹ 只闸 setpoint 的话，PX4 会**一直沿用最后一帧命令**
  （[`mpc-node-fallback.md`](mpc-node-fallback.md) §1.1，`MulticopterRateControl.cpp:177-184`）。
  **本条的"丢失"只有一个来源：进程不在了**（或显式 `-p heartbeat_always:=false`，另一档，
  见 `mpc_node.py` 模块 docstring）。

**什么算过**：心跳停之后 **≤ `COM_OF_LOSS_T` + 一点余量**内，PX4 的 `nav_state` 离开 OFFBOARD、
模式落回 Position，飞机**不撒手飞走**、也不进入任何"程序自己兜"的状态。

**留什么证据**：停进程的**时刻** + `nav_state` / 模式的**前后读数** + 时间差。
(A) 级这一场还应当有 `mavros` 侧的观察窗（[`operation_manual.md`](../../operation_manual.md) §5.1 的核对表）。

**今天能不能做**：**(A) 级 ✅ / (C) 级 ⛔**。(A) 级那条链今天起得来；
(C) 级今天整条不存在（§1）⟹ 这一条只有 (A) 级那一半能真的走一遍。

### 2.4 节点崩溃注入 —— **不打招呼地**杀掉它

**动作**：在飞机**已闭环**的状态下把程序进程 **SIGKILL**（不是 `Ctrl-C`）：

```bash
pgrep -af control          # (A) 级；先看清楚要杀的是哪一个
kill -9 <pid>              # ★ 硬杀：不给它任何收尾机会
```

⚠️ **按名字扫会连带别人会话起的同类进程** —— 与 [`run/README.md`](../../run/README.md)
「清除后台节点」那条一句警告同源；先摊开清单再动手。

**期望与出处**：与 §2.3 同一条 PX4 路径（心跳消失 ⟹ `COM_OF_LOSS_T` ⟹ 退出 OFFBOARD ⟹ Position）。
**本条与 §2.3 的区别只在"怎么让它死"**：§2.3 走**正常停**（`Ctrl-C`，节点还会打一行收尾摘要，
见 `src/nmpc/test/test_mpc_node.py::test_SIGINT_之后打出收尾摘要而不是_traceback`），
本条走**崩溃**（没有收尾、没有最后一行日志）。

**什么算过**：与 §2.3 相同，**外加**一条：(A) 级下 `/control/state` 的话题要么消失、
要么**不再更新**（崩溃的进程不会替你写"我停了"）⟹ "飞机为什么不动了"必须能从 **PX4 侧**
回答，而不是从我们的日志回答。

**留什么证据**：`kill -9` 的 pid 与时刻 + PX4 侧的前后读数 + **一句"我们的日志到哪一句为止"**。

**今天能不能做**：**(A) 级 ✅ / (C) 级 ⛔**（同 §2.3）。

### 2.5 求解失败注入 —— ⛔ **今天做不了**

**动作**（约定手法，第一手 [`mpc-node-fallback.md`](mpc-node-fallback.md) §5）：

```bash
ros2 run nmpc mpc_node.py --ros-args -p tau_s:=0.0 -p inject_solve_fail_after_frames:=<N>
```

语义：**已发布 ≥ N 帧之后**，每一拍的求解结果都被换成坏解（状态码非零 + `u` 全 NaN + 耗时 NaN）。

**期望与出处**：与 SITL 一致 —— 先**保持上一帧可行指令** `fallback_hold_frames` = `10` 拍，
连续失败到阈值再切**安全悬停**（悬停推力 + 零角速率）；**任何一帧都不带非有限值**；
这个事件在日志里**可检索**（`grep '"fallback": "hover"'` 那一段，列名见 `mpc_node.py` 的日志面）。
机制与"为什么不能一直保持"： [`mpc-node-fallback.md`](mpc-node-fallback.md) §1.1/§2。

**什么算过**：行为与 SITL **一致**（保持 → 悬停），且日志里检索得到那一次。
⚠️ "和 SITL 一致"是**比较**判据，不是绝对值 —— SITL 那一场的读数在
[`ladder-acceptance-criteria.md`](ladder-acceptance-criteria.md) §4 的判据④ 行。

**留什么证据**：注入参数原文（含 N）+ 逐帧 JSONL 的 `fallback` 列那一段 + 收尾摘要。

⛔ **今天做不了**（第一手，2026-10-10 核）：`UXRCE_DDS_CFG` = `0` 且**没有任何串口分给它**
（`MAV_0_CONFIG`/`MAV_1_CONFIG`/`MAV_2_CONFIG` 的现状见
[`params-changes.md`](../reference/params-changes.md) §E1）⟹
节点发出的 `vehicle_rates_setpoint` **到不了 PX4**，连它订阅的 `/fmu/out/vehicle_status` 都没有
⟹ **闸门永远不开、一帧都不会发**（§12.3①，那一节把它点名为**无人认领的空缺**）。

⚠️ **两条护栏（装桨前必看）**：

1. 注入**默认 0 = 关**；开着的时候启动会打 `error` 级告警；
2. `preflight_check.py` 的「兜底」组里有一条 **`故障注入关着`**
   （`src/nmpc/nmpc/preflight_check.py`）⟹ 装桨前的静态自检会当场变红。
   ★ 装桨前那一步的完整口径见 [`first-flight-config.md`](first-flight-config.md) §5 ③。

### 2.6 回退 (A) 级 —— **动作本身**见 §3

票面要的是"一次确定的、**演练过的**操作"。§3 给的是那个操作的**成文版本**；
"演练过"这一半只能由人在台上走一遍。
⚠️ **而今天能走的只有它的一半**（判据在 §3.1 末）：第 1/4/5 步，也就是票面那半句
「**确认 Phase 1 整条链仍在、随时可用**」；另外半句「**切换动作本身**」要有一条**真的在发的
(C) 链**才谈得上"切"，而今天没有（§12.3①）。

---

## 3. 回退 (A) 级：成文的切换动作

### 3.1 (C) → (A) —— **这就是"回退 (A) 级"**

顺着 §1 的"两条链共用一条前缀"，切换**只动两端**，五步、每步都有**可读的判据**：

| # | 动作 | 命令（出处） | 这一步的判据（★ 不是"敲了就算"） |
|---|---|---|---|
| 1 | 确认 (A) 链此刻**不在发** | `run/z0.py check`（[`run/README.md`](../../run/README.md) 菜单 **2= 起链核对（只读，约 5 s）**） | `/control/state` 没有在跑的任务态；`/mavros/setpoint_raw/local` 不是我们的帧 |
| 2 | 停 (C) 级的**发指令那一个** | `mpc_node` 是**前台**进程 ⟹ 在它那个终端 `Ctrl-C`（会打一行收尾摘要）；终端不在了见下面的 ⚠️ | 进程真的退了（不是"终端关了"） |
| 3 | 确认 (C) 链**停了** | `ros2 topic hz /fmu/in/vehicle_rates_setpoint` | **没有帧**。⚠️ 心跳 `/fmu/in/offboard_control_mode` **同时**没了 —— 那才是 PX4 的判据（§2.3）⟹ 它会在 `COM_OF_LOSS_T` = `1.0` s 后自己退出 OFFBOARD |
| 4 | 起 (A) 级的**两端** | `run/z0.py up`（菜单 **9= 一键启动全部（选轨迹 → 按开始键）**，= `run/up.sh`）；⚠️ 它**也会去起 mavros**，已经有一个在跑时会先问一句 —— 只补缺的两端就用 `run/z0.py traj` / `run/z0.py control`（各自前台） | 见第 5 步（它才是判据） |
| 5 | **确认 (A) 链真的回来了** | `run/z0.py check`（菜单 **2= 起链核对（只读，约 5 s）**） | `/mavros/state` 的 `connected/armed/mode` + **`/control/state` 的原文** + 每个话题有没有帧/多少 Hz。起始态是 `MANUAL/INIT`，守卫齐了是 `MANUAL/STANDBY`（出处：[`operation_manual.md`](../../operation_manual.md) §5.1 那张核对表的 `/control/state` 行 ＋ 菜单 **7= 起 control（单独）** 那一行）；**落地上锁之后**自己回 `MANUAL/STANDBY`（同页 §7 第 1 条） |

**今天能不能做**（★ 这是本页最容易被误读的一格）：

| 步 | 今天 | 为什么 |
|---|---|---|
| 1 / 4 / 5 | ✅ | 只碰 (A) 级那一侧 ⟹ **今天就能走**，且这正是票面要的"**确认 Phase 1 整条链仍在、随时可用**" |
| 2 / 3 | ⛔ | 它们的前提是"**有一条正在发 setpoint 的 (C) 链**"，而 (C) 级下行今天不存在（§1、§12.3①）—— **没有可停的对象**，所以"切换动作本身"这一半**今天演练不了** |

⟹ **今天走 §3.1，就是把第 1/4/5 步走一遍**（起 (A) 链 → 核对 → 读到 `/control/state`）。
第 2/3 步等 `UXRCE_DDS_CFG` 有归属（§12.3①）之后再补演练 —— 那时它才是一条**真的**切换。

**⚠️ 第 2 步的两个坑**（第一手，都是代码/脚本层的事实）：

- **`run/down.sh` 不认 (C) 链**：它的白名单 `DOWN_TARGETS`（`run/z0.py`）只有
  `mavros` / `vrpn` / `mocap` / `traj` / `control` / `bringup` **六条** ——
  **没有 `mpc_node`，也没有 `MicroXRCEAgent`**。⟹ 节点是在别的终端/别的会话起的、
  那个终端已经不在了的话，`down.sh` **帮不上忙**，只能按名字找：
  `pgrep -af mpc_node`，然后照 `run/down.sh` 的**同一套顺序**
  **先 SIGINT ⟹ 超时才 SIGTERM ⟹ 再 SIGKILL**。
  ⚠️ 而按名字扫会**连带别人会话的同类进程**（`run/down.sh` 自己那句警告同源）⟹
  敲之前先看一眼命令行。
- **`MicroXRCEAgent` 可以留着**：它是 `/fmu/*` 的桥，**自己不产生任何指令**
  ⟹ 留着不影响 (A) 链。要停只是省资源，**但它不是回退的一部分**。

### 3.2 (A) → (C) —— 上场路径（**今天不成立**）

反向的步骤就是把 §3.1 的 2/4 两步对调，但**两处未决卡在这里**
（[`mpc-node-seam1.md`](mpc-node-seam1.md) §12.3②）：

- **谁解锁、谁切 OFFBOARD 没有归属**。`offboard_smoke.py` **本身就是**一条完整的
  「心跳 ≥1 s ⟹ 解锁 ⟹ 切 OFFBOARD」实现（`:289` / `:294`，publisher 在 `:112`），
  但它与 `mpc_node` **都发 `/fmu/in/vehicle_rates_setpoint`** ⟹ 不能"用一个拉起、再交给另一个"。
- ⛔ **整条 (C) 级下行今天不存在**（§2.5 的同一处事实）⟹ §3.2 的步骤**现在写出来也只是草案**。
  本页**不编**：等 §12.3①② 有归属之后再补这一段。

### 3.3 空中**不许**做这件事

**停进程不是降落。** 这句话在这条链上是**双重**的（第一手）：

1. (A) 级：`run/z0.py` 的 `DOWN_WARNING` 原文 —— 清除节点之后，setpoint 一停，
   PX4 在 `COM_OF_LOSS_T` 后自己退出 OFFBOARD、按 `COM_OBL_RC_ACT` 落回 Position；
   **飞机若还在空中，先按 [`operation_manual.md`](../../operation_manual.md) §1.2 的软路径处置
   （打杆接管 / 拨 ch6），别指望回退。**
2. (C) 级：**同一条**，只是停的是 `mpc_node`。

⟹ **§3.1 那五步的前提是"飞机已经在地面"**。在飞的时候要交还控制权，用的是 §2.1（打杆）
或 ch6，**不是** §3.1。

### 3.4 两条链的"停"**不对称**（一张表记住）

| | 停它 | 它停了之后 PX4 怎么办 |
|---|---|---|
| (A) 级 | `run/down.sh` / `run/z0.py down`（按名字扫，先清单再 y/N） | 心跳停 ⟹ `COM_OF_LOSS_T` ⟹ 退出 OFFBOARD ⟹ `COM_OBL_RC_ACT` = `0` → Position |
| (C) 级 | **没有工具**，只能 `Ctrl-C`（前台）或 `pgrep` + 手动 kill | 同上（★ 但闸门关着的时候**心跳照发**，见 §2.1 的不对称 ①） |

---

## 4. 证据格式（每次演练填一行）

体例与 [`ladder-acceptance-criteria.md`](ladder-acceptance-criteria.md) §6 **一致**（同一把尺）：

```
条 | 日期时间 | 链（A/C） | 条件（参数档 / τ / 频率 / 电池） | 判据 | 实测值 / 原文读数 | 过/不过 | 不过⟹归因
```

**三条硬规矩**：

1. **"过"要能指到 §2 的具体那一条，"不过"要写实际数值** —— 不写"基本正常"。
2. **现象也算证据，但必须写成可复核的形状**（"电机转速差方向与 §1.1 三行表一致"），
   **不写"看着对"**。
3. ★ **一手证据当场落仓库** —— `/tmp` 里的东西**过不了重启**（ext4 根盘 + tmpfiles 的
   `D /tmp`，出处 [`ladder-acceptance-criteria.md`](ladder-acceptance-criteria.md) §6）。

---

## 5. 本页**没证明**什么

- **没起过、没跑过任何一条**。命令是从仓库里**现读**的（`run/` 层、节点源码、参数快照），
  示意的次序**没有在台上验过**。
- **没写 §3.2（(A) → (C)）的步骤**：它被 §12.3①② 两条未决挡着，写了也是草案。
- **没回答**"今天做不了的那几条**什么时候**能做" —— 那取决于
  `UXRCE_DDS_CFG` 指到哪个口（§E1 的"开在哪个口上"），以及 §12.3② 的归属裁定。★ 这两件事
  **`.scratch/` 里今天没有任何一张票认领**（§12.3① 的原话："归属待定"）。
- **没动任何代码、没改任何参数** —— 本票是演练票，不是改动票。

---

## 6. 登记：与本页相关的**两处过期读法**（★ 只登记，不就地改）

演练现场会**同时**用到这两处，而它们印出来的话与 §2.0 的读数对不上。
按本仓的登记制（不就地改别的票的产物）**只记在这里**，归属由操作者裁定：

| 位置 | 它现在写着 | 与哪一条对不上 |
|---|---|---|
| `run/z0.py` 的 `DOWN_WARNING`（`run/z0.py:1195`） | "而 **A1 还没落实**，遥控同时丢失时 `NAV_RCL_ACT = 2` 会让它**试图返航**" | `NAV_RCL_ACT` = `1`（§2.0），且 [`params-changes.md`](../reference/params-changes.md) §A1 的 A1 行写的是**现值 `1`（Hold，2026-09-28 实测）** ＋ **阻塞列"✅ 已落地"** ⟹ 这句描述的是**当时**的状态 |
| [`operation_manual.md`](../../operation_manual.md) §6 排障表末行 | "⚠️ A1 未改之前会**试图返航**" | 同上（**同一处事实**，两处印字） |

★ **为什么值得记这一行**：这两处都是**在"想立刻停任务"的现场才读到**的那几行 ——
那时读到一句与真身相反的读数，代价不是"读错"，是**照它去判断**。
⚠️ 另一处同族但**不必改**的：[`params-changes.md`](../reference/params-changes.md) §H.7 里
"本机 … `NAV_RCL_ACT = 2` = Return" —— 它**已经**在 §H.7.1 的 ☠️ 标题里就地说明了"已过时"，
全文照实留档是有意的。

★ **第三处同族、但方向相反**：两处**指向"这个动作还没有正文"**的指针，在本页落地之后**已经过时** ——
[`mpc-node-seam1.md`](mpc-node-seam1.md) `:507`（"⚠️ **这一步本身没有成文流程**（`phase4/08` 要的
"切换动作演练到位"就是它）"）与 [`ladder-acceptance-criteria.md`](ladder-acceptance-criteria.md) `:453`
（§5.2 末段"用 ticket 08 演练过的那个操作"）。**它们没有说错事实，只是现在有正文可指了。**
本页**只登记，不回填** —— 那两份属于别的票的产物，回不回填、归谁回填由操作者裁定。
