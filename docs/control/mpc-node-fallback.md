# 求解失败与数值兜底（M5 / ticket 07）

> **一句话**：**"我们不发" ≠ "飞控松手"**。PX4 会把最后一帧 `vehicle_rates_setpoint`
> **无限期锁存**下去（§1.1，源码引用），所以"求解不可用"时必须由我们**主动换成**一条安全的帧：
> 先 `hold`（保持上一帧可行指令），连续超过阈值再 `hover`（悬停推力 + 零角速率）并**置标志上报**。

本文是 ticket 07 的落地文档，回答 ticket 06 交接过来的那条
（[`mpc-node-seam1.md`](mpc-node-seam1.md) §9 第一行：「求解失败 / 非有限值的兜底动作 —— 谁接管、怎么降级」）。

| 节 | 回答什么 |
|---|---|
| §1 | **第一手**：PX4 侧到底会发生什么（锁存 / 上电初值） |
| §2 | 三档阶梯：每档发什么、阈值多少、为什么不是别的做法 |
| §3 | 三道判据收口在**唯一**发布点；与 06 记录的"三类行为"的分界线 |
| §4 | 输入合理性：两种"数值正常、但不是这一秒的"帧 |
| §5 | 故障注入接缝（**为什么必须开在 DUT 里**）与它的三道护栏 |
| §6 | 台架 / SITL 配方 |
| §7 | 证据：哪个测试钉住了哪一条 |

代码落点：`src/nmpc/nmpc/node_core.py`（`FallbackLadder` + 两条判据，零 ROS）、
`src/nmpc/nmpc/mpc_node.py`（接线、注入、日志）、`src/nmpc/nmpc/preflight_check.py`（装桨前的三道）。

---

## 1. 第一手：PX4 侧会发生什么

### 1.1 最后一帧会被**无限期锁存**

```cpp
// src/modules/mc_rate_control/MulticopterRateControl.cpp:177-184（z0 真身 4a0e65f2）
} else if (_vehicle_rates_setpoint_sub.update(&vehicle_rates_setpoint)) {
        if (_vehicle_rates_setpoint_sub.copy(&vehicle_rates_setpoint)) {
                _rates_setpoint(0) = PX4_ISFINITE(vehicle_rates_setpoint.roll)  ? vehicle_rates_setpoint.roll  : rates(0);
                _rates_setpoint(1) = PX4_ISFINITE(vehicle_rates_setpoint.pitch) ? vehicle_rates_setpoint.pitch : rates(1);
                _rates_setpoint(2) = PX4_ISFINITE(vehicle_rates_setpoint.yaw)   ? vehicle_rates_setpoint.yaw   : rates(2);
                _thrust_setpoint = Vector3f(vehicle_rates_setpoint.thrust_body);
        }
}
```

`_rates_setpoint` / `_thrust_setpoint` 是**成员**（`MulticopterRateControl.hpp:124/127`，
上一行 `:122` 就是上游自己的注释 `// keep setpoint values between updates`）
⟹ 没有新消息时它们**原样留着上一次的值**。于是：

- **不发 setpoint 的效果 = 让飞机继续执行一条陈旧命令**（可能正打着满角速率）；
- 三个角速率分量有 `PX4_ISFINITE` 回退（退回实测角速率），**推力没有** ——
  一个 NaN 推力会直达混控器（这条是票面判据 2 的第一手依据）。

### 1.2 上电之后、收到第一条 setpoint 之前：零推力

```cpp
// MulticopterRateControl.hpp:124,127（同一 commit）
matrix::Vector3f _rates_setpoint{};        // ← 零初始化
matrix::Vector3f _thrust_setpoint{};
```

⟹ "从来没有发过"与"发了零"在飞控侧是同一件事 = **掉下来**。
这一条决定了一个边界行为：**第一拍就不可用时不能沉默** —— 那时没有"上一帧可行指令"可保持，
所以 `hold` 退化为 `hover`。★ 这个判断由**阶梯**做（`FallbackLadder.on_fail(…, can_hold=)`，
节点把"`_last_sp` 在不在"告诉它），理由是**档位与实际发出的帧必须同源** —— 否则
JSONL 打 `hover`、收尾摘要却说"此刻仍在 hold"，`holds` 还会把悬停帧记成保持帧，
而这一次事件也要等 streak 超过阈值才上报。单测：
`test_node_core.py::test_兜底阶梯_没有可保持的帧时直接进悬停_且算一次事件`。

---

## 2. 兜底阶梯

| 档位 | 什么时候 | 发什么 | 埋点 |
|---|---|---|---|
| `normal` | 求解可用（§3） | 它的解 | JSONL `fallback: "normal"` |
| `hold` | 连续不可用 ≤ `fallback_hold_frames`（默认 **10** 拍） | **上一帧可行指令**（内容原样、**时间戳换新**） | JSONL `fallback: "hold"` |
| `hover` | 连续不可用 > 阈值 | **悬停推力 + 零角速率**（`t_hover` / `T_max` ⟹ 默认 −0.6235） | `error` 级日志 + JSONL `fallback: "hover"` + 收尾摘要 |

**★ 为什么不是"一路保持到底"**：由 §1.1，一直不发 = 让飞控无限期沿用一条**陈旧**命令。
一拍的解刚不可用时，上一条命令还是**可行**的（几毫秒前算出来的）⟹ 先保持；
但一直不可用的话，"可行"这个前提就没了，必须由我们主动换成安全悬停。

**★ 为什么不是"一失败就悬停"**：一次抖动（一盘数据里的单点、一次抢占超时）就换姿态，
比保持一条 10 ms 前的可行命令更糟。两条合起来给出阈值的量级：

```
100 Hz 一拍 10 ms；实测求解 max 3.58 ms（offline-ocp.md §5）+ 若干个求解预算的余量
⟹ FALLBACK_HOLD_FRAMES = 10 拍 = 100 ms ≈ 28 个求解预算
```

**★ `hold` 为什么必须"发出去"而不是"不发"**：由 §1.1，两者在飞控侧等价
（都表现为"没有新命令"），但在**话题上**完全不同 —— 不发的话，"它在保持"这件事
**谁都看不见**（夹具、日志、复盘全都看不见）。所以保持 = **内容原样、`timestamp` 换新**：
于是"同一组 `roll/pitch/yaw/thrust_body[2]` 连着出现 N 次、而时间戳一直在前进"是可断言的。

**★ 安全悬停为什么在"状态不可信"时仍然可用**：它**不依赖任何状态** ——
推力沿机体系 z、角速率置零。这条性质是它能在 `stamp_regress` / `pose_jump` 之外
单独成立的原因（那两条判据关的是闸门，不是这一档）。

**★ 一个边界，`hold` 档不存在的那种**：**第一拍就不可用**（此前一直在热身或闸门关着）时，
"上一帧可行指令"根本不存在 ⟹ `hold` 直接跳过、进 `hover`，并且**照样算一次事件**（要上报）。
判据由调用方给（`on_fail(…, can_hold=)`），见 §1.2。

**累计量与复位**：`holds` / `hovers` / `events` 是**病历**，只累计、只在收尾摘要里读；
闸门关闭时 `reset()` 清档位与连续计数，但**不清累计量**（`FallbackLadder.reset` 的 docstring）。
`hold` ↔ `hover` 的判据是**连续**失败（中间成功过一拍就清零）—— 票面写的是"**连续**"。

---

## 3. 三道判据收口在唯一发布点

`mpc_node._normal_frame(sol)`（"这一拍的正常帧能不能用"），顺序就是票面判据的顺序：

| # | 判据 | 拦什么 | 依据 |
|---|---|---|---|
| ① | `node_core.solution_usable(sol)` | acados 状态码非零（含 `qp_stat`）／`u` 形状不是 `(4,)`／`u` 非有限 | 票面判据 1、判据 2 的解这一半 |
| ② | `sign_map.px4_rates_setpoint` 自身的入参检查 | `ω`/`F`/`T_max` 非有限、负推力 | `sign-map.md` §2 的实现 |
| ③ | `node_core.frame_is_finite(sp)` | **凡是要写进那一帧的四个数**里任一个非有限 | 票面判据 2「发布前 `np.isfinite()` **全量校验**」 |

**★ 与 ticket 06 记录的"三类行为"怎么接上**（06 那条警示点名要 07 接手）：

| 06 记录的现象 | 07 的分界线 | 这一拍 |
|---|---|---|
| 动捕非有限值 | `node_core._hold("non_finite_mocap")` | **闸门关**（不进阶梯：没有可信状态就没有"这一拍"） |
| **求解退化**（`SolveInfo.degraded`） | `degraded` **但不是** `ok=False`（如 `u` 越界 1e-3 N） | **照样发** + `warn` + `degraded_count`（06 记录的现行行为，本票**不动**它） |
| **求解失败**（状态码非零）／`u` 非有限／映射拒绝 | `solution_usable` 非 `None` 或 ②③ 拦下 | **走兜底阶梯** |

⭐ 这条分界线由 `solution_usable` 只看 `info.ok`（状态码）而不看 `info.degraded` 来实现，
在 `test_node_core.py::test_解可用吗_退化与不可用是两件事` 里逐格钉死。
**为什么这么切**：`degraded` 里最松的那一半（`u` 越界一个小量、代价读数非有限）
解本身仍然有限、仍然能用，把它读成"失败"会让兜底在**良性**读数上乱触发；
而状态码非零意味着求解器自己说"我没解出来"。

**簿记的两处污染防护**（同一次改动里修的，都是"一次抖动变成永久失效"的形状）：

- `_u_ref`（下一拍的前馈）**只在解可用时更新** —— 否则一次非有限解会污染后面每一拍的
  `yref`，求解器从此每次都失败（**持续失败**，不是一拍）；
- `solve_ms_max` 只在 `time_tot` 有限时取 `max` —— `max(nan, x)` 恒为 `nan`
  ⟹ 收尾摘要里"最坏耗时"那一行会从那一刻起永久失去意义。

---

## 4. 输入合理性：两种"数值正常、但不是这一秒的"帧

超时判据（`mocap_timeout_s` / `status_timeout_s`）拦的是"**没有数据**"，
拦不住"**数据是错的**"。这一档拦的就是后者，取向是 **宁可漏判，不可误判**：

> 误判的代价是**无故停发** —— 飞机在空中，我们却因为一次正常机动停了指令流。

| 判据 | 拦什么 | 阈值（默认） | 取法依据 |
|---|---|---|---|
| `stamp_regress` | 源自己的 `header.stamp` 往回走 | 容差 **0.01 s** | 节点 100 Hz 取一拍而动捕 200 Hz ⟹ **同一个 stamp 会被读到两次**；DDS 也不保证到达顺序单调。**容差不是余量，是必需** |
| `pose_jump` | 两帧之间的位移超过"走得完"的量 | **20 m/s** = 1.7 × `MPC_XY_VEL_MAX`(12 m/s) | 给真实机动留余量（[`params-analysis.md`](../reference/params-analysis.md):477） |
| `attitude_jump` | 两帧之间的**测地角**（`model.quat_angle`）超过"转得完"的量 | **12.6 rad/s** = 2 × `YAW_RATE_MAX`(6.28) | 同上；6.28 是本项目模型里的偏航角速率上限（`params.py:51`，与飞控 `MC_*RATE_MAX` 对账过） |

三条共同的实现细节（都在 `node_core.NodeCore`）：

- **`dt` 下限** `JUMP_DT_FLOOR_S = 0.01 s`：`允许 = 速率 × max(dt, 下限)`。
  没有它的话，DDS 投递抖动把 `dt` 压到 1 ms 时允许位移只剩 0.02 m，
  一次正常机动就被误判 —— **判据会变成频率与抖动的函数**，而它与被测量无关。
- **基准只在被接受的帧上更新**，且**不随闸门关闭清零**：
  被拒的那一帧若把基准搬走，"跳变来的那一帧"立刻成为新基准、闸门下一拍就开 ⟹ 判据等于不存在。
  实测（`VrpnSource.rewind`）重放窗口结束后 stamp 会**一次性向前跳回来** ⟹ 那时放行。
- **判据不是闭锁**：源回到基准附近（或 `dt` 把上限撑开）就恢复。两条 seam-1 用例都断言了
  "停发 **和** 恢复"两半 —— 只断"停发"的话，"节点自己死了"也满足它。

**读数进日志**：`Cycle.detail`（如「位移 100.000 m > 允许 0.200 m（dt=10.0 ms、上限 20 m/s）」）
在闸门**刚关上**的那一拍以 `info` 级立即打出（不等 `log_every` 抽帧 —— 这种帧可能只出现一两拍）。
调阈值的人第一个要问的就是这个数。

**⚠️ 未决项**（已记入 [`phase2-implementation.md`](phase2-implementation.md) §9 的 O7）：
按票面，这两条判据的选择是"**只关闸门、一帧不发**"（严格照票面执行）。由此产生一个诚实的缺口：
**长时间不信任状态下，飞控最终仍然落在"锁存最后一帧"上**（§1.1）—— 与我们主动换成安全悬停相比，
哪个更安全**本票没有拍板**，需要 SITL/实测定。

---

## 5. 故障注入接缝（为什么必须开在 DUT 里）

```bash
ros2 run nmpc mpc_node.py --ros-args -p inject_solve_fail_after_frames:=200
```

语义：**已发布 ≥ N 帧之后，每一拍的求解结果都被换成一个坏解**
（状态码非零 + `u` 全 NaN + `time_tot` NaN；实现 `mpc_node._poisoned`）。
三样一起换不是保险：`u` 全 NaN 走判据 2、状态码非零走判据 1、耗时 NaN 走簿记的污染防护 ——
只换一样，另两条路径就永远没被覆盖。

**★ 为什么这是"必须在 DUT 里开口子"的少数情况**：
[`mpc-node-seam1.md`](mpc-node-seam1.md) §9 已经定过一条纪律 ——
「DUT 自带故障注入器是『为了被测而改被测』，不该继续留着」（针对 05 那个 stub 的 `--break`）。
本条**不违反**那条纪律，因为两者的性质不同：

- 05 的 `--break` 造的是**外部世界能造**的失效（少发一条、发错一条）⟹ 夹具侧造即可，DUT 不该带；
- **"求解失败"发生在 DUT 进程内部**：话题面上只看得到帧，**夹具站在边界上造不出来**；
  而 SITL 上"注入故障之后飞机不撒手"这条判据（ticket 11）只能靠它。

它的三道护栏（防止"注入开着就上桨"）：

1. **默认 0 = 关**；
2. 启动时 `error` 级显著告警（`mpc_node.__init__`），并把"装桨前必须设回 0"写在同一行；
3. `preflight_check` 的「兜底」组里有一条 **`故障注入关着`**，装桨前的静态自检当场变红。

---

## 6. 台架 / SITL 配方

**台架（`test_mpc_node.py`，seam 1）** —— 注入 + 两条输入合理性判据要**分开**造：

| 要造什么 | 怎么造 | 用例 |
|---|---|---|
| 求解失败 | `-p inject_solve_fail_after_frames:=20`（+ `-p fallback_hold_frames:=5` 缩短阶梯） | `test_求解失败时先保持上一帧_再切安全悬停_且一帧都不带非有限值` |
| 时间戳倒退 | `bench.timeline.mocap_stamp_rewind(source, at_s, seconds, hold_s)` | `test_动捕时间戳倒退时不发指令_追平之后恢复` |
| 位姿跳变 | `bench.timeline.mocap_pose_jump(source, at_s, position=(100,0,0), hold_s)` | `test_位姿跳变时不发指令_回到原位之后恢复` |
| 非有限值 | **不用夹具**：动捕喂全零四元数（归一化只剩 NaN） | `test_node_core.py::test_坏掉的四元数不发_而好的发`（06 留下的）+ 本票的 `u` 非有限那几条单测 |

★ 两个配方刻意**只动一个量**：`mocap_stamp_rewind` 位置不动（于是闸门只可能是 `stamp_regress`
关的），`mocap_pose_jump` 的 stamp 正常前进。两个都发的话，"是哪一条判据在起作用"就分不出来。

**SITL（ticket 11 用）**：同一个开关，N 取"已经稳定悬停之后"的帧数
（例如悬停稳定后 `-p inject_solve_fail_after_frames:=<已发帧数 + 200>`），
判据是"**飞机保持安全悬停而不是掉下来，且日志里检索得到那次事件**"：

```bash
grep '"fallback": "hover"' mpc.jsonl      # 逐帧 JSONL：兜底档位与原因
grep '兜底：连续' <(ros2 run … 2>&1)       # 上报那一次（error 级）
```

---

## 7. 证据：哪条测试钉住了哪一条

| 票面验收项 | 钉在哪 |
|---|---|
| ① 求解失败绝不 `raise`；保持上一帧 / 安全悬停 | `test_mpc_node.py::test_求解失败时先保持上一帧_再切安全悬停…`（`dut_alive` + 无 `Traceback` + 两段形状可分） |
| ② 发布前全量校验、任一项非有限则整帧丢弃 | `test_node_core.py::test_整帧校验认得非有限值`、`test_解可用吗_退化与不可用是两件事`；端到端：上述 seam-1 用例里的"**任何一帧都不带非有限值**"扫描 |
| ③ 连续失败达阈值 → 置标志并上报、可检索 | `test_node_core.py::test_兜底阶梯*`（4 条）+ JSONL `fallback` 列 + 收尾摘要 `触发 N 次` |
| ④ 时间戳倒退 / 位姿跳变 ⟹ **不发指令** | `test_node_core.py` 的输入合理性一组（**10 条**，`test_真实机动不会被当成跳变` … `test_位姿判据的_dt_下限把频率与抖动挡在外面`）+ 两条 seam-1 用例 |
| ⑤ 全部走夹具的故障注入用例 | 本文 §6 那张表：**四条路径两两分开**（三条走夹具、非有限值走已有的单测/夹具来源） |
| ⑥ SITL 的注入手段可用 | §5 的开关（判据 4 由 ticket 11 跑） |
| ⑦ M5 的编号与依据记入 `phase2-implementation.md` | 同文件 §2.5 的 M5 落地段 + §4 S4/S8 |
