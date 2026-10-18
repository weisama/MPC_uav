# Phase 2 实施清单 —— MPC 接管 PID（`(C)` 级）

> **本文件是 Phase 2 的施工图**，与 [`phase1-implementation.md`](phase1-implementation.md)（Phase 1 实施清单）平行。
>
> | 文件 | 性质 |
> |---|---|
> | [`mpc-roadmap.md`](mpc-roadmap.md) | **资料汇总** —— 摆证据、给倾向，**不是决策**（它自己第 9 节列了 D1–D10 待决） |
> | **本文件** | **决策落地 + 实施清单** —— 记录已定决策、解决 D1–D10、给出可执行步骤 |
>
> 上级文件：[`../PROJECT.md`](../PROJECT.md) §4 与里程碑表。
> 事实来源：[`../reference/px4-interface.md`](../reference/px4-interface.md)、[`../reference/mocap-setup.md`](../reference/mocap-setup.md)、[`../reference/hardware.md`](../reference/hardware.md)。
>
> ⚠️ **本文件中标注「第一手」的结论，都是从源码/仓库直接读出来的**，不是从搜索结果或别人总结里转述的。路径与行号可复核。

---

## 0. 先读这一段：Phase 2 的三个"一句话"

1. **Phase 2 = MPC 输出「角速率 + 推力」，PX4 只跑角速率环 + 混控。** 这是 `(C)` 级。<br>☠️ **2026-10-13（票 13 收口轮）：本句原文是"消息是 `/fmu/in/vehicle_rates_setpoint`（uXRCE-DDS），**不是** mavros" —— 这半句**今天整个反过来**：`(C)` 级下行走的**就是 mavros**（`/mavros/setpoint_raw/attitude`，`body_rate` + 标量 `thrust`），**不是** uXRCE-DDS。** 定义不变、载体变了；定案与起因见 [`../reference/px4-interface.md`](../reference/px4-interface.md) §4.3。
2. **代码骨架取 [`DISCOWER/px4-mpc`](https://github.com/DISCOWER/px4-mpc)**（BSD 3-Clause，可自由改）。但它是为 x500 写的，有 **6 处必须改**（§2.5），其中 2 处改错会直接摔机。
3. **Phase 2 引入 4 个新东西**：acados 求解器、~~uXRCE-DDS 中间件~~、MPC 节点、`px4_msgs` 版本绑定。**它们在 SITL 里全部跑通之前，实机一行都不许上。**<br>☠️ **2026-10-13**：第 2 个（uXRCE-DDS 中间件）**最终没有引入**（(C) 级下行改走 mavros，见 [`../reference/px4-interface.md`](../reference/px4-interface.md) §4.3）⟹ 这份清单实为 **3 个**。

---

## 1. 已定决策（从对话中固化，不再重开）

| # | 决策 | 结论 | 依据 |
|---|---|---|---|
| G1 | 控制层级 | **`(C)` 角速率 + 推力** | roadmap §2 四条论据 |
| G2 | 中间件 | ☠️ **2026-10-13 改判：全 mavros**（原写"**混合**：mavros2 收动捕 / uXRCE-DDS 发指令"）—— uXRCE-DDS **最终没上**，见 [`../reference/px4-interface.md`](../reference/px4-interface.md) §4.3 | 用户 Q7：「Phase 1 全走 mavros2，uXRCE-DDS 推迟到 Phase 2」☠️ **推迟到了，最终没上** |
| G3 | MPC 状态输入 | **动捕直出**（不经 EKF2） | 用户 Q9 + roadmap D8；延迟更低 |
| G4 | 求解器 | **acados** + `SQP_RTI` + `PARTIAL_CONDENSING_HPIPM` | roadmap §4.5，与 px4-mpc 一致 |
| G5 | 代码骨架 | **`DISCOWER/px4-mpc`**（复制改造，不引依赖） | 第一手读过三个核心文件，见 §2 |
| G6 | 固件版本 | ★ **版本基准 = z0 固件真身 `Minderring/PX4-Autopilot` @ `4a0e65f2`**（**v1.14.3 系**的 MicoAir 厂商构建，飞控自报 `1.14.3`）。⟹ **`px4_msgs` = `release/1.14` @ `ffb6e80e`**（与真身共有的 184 个 `msg/` blob 全同，✅ 用户签字）；**SITL 被控对象 = 上游 v1.14.3（tag `1dacb4c`）**。☠️ **原写「版本基准统一到 fork `5ba2e91f`（Q31=a）」与「固件是「v1.15 世代」」—— 两句都反了，`5ba2e91f` 与 z0 无关**。见 [`refs-manifest.md`](../reference/refs-manifest.md) §2.5(d) 与 §2.6 执行版。⚠️ **「不刷固件」这条决定本身不变** | 用户 Q12 + [`refs-manifest.md`](../reference/refs-manifest.md) §2.5(d) / §2.6 |
| G7 | 上线顺序 | **先 SITL 验证 MPC，再实机** | 用户 Q8(b) |
| G8 | INDI | **第一版不做**，作为步骤 2 | roadmap D3 |
| G9 | 参考轨迹 | **第一阶段手写**（悬停 / 正方形 / 八字），与 Phase 1 同一套 | 用户 Q15 |
| G10 | 动捕原点 | **未起飞时静止帧为原点**，yaw 只归零偏航 | 用户 Q16 / Q21(b) |
| G11 | 动捕输出 | **只发 `/mocap/pose_enu`（ENU / 原生）**；`/mocap/pose_ned` **不发布**（原写会发 —— Q14=a 已删） | 本文 §3.2 + 用户 Q14 |

---

## 2. 参考实现 `DISCOWER/px4-mpc` 第一手解读

> 仓库 `https://github.com/DISCOWER/px4-mpc`，HEAD `3e541af`（2026-06-15），**BSD 3-Clause**，Python `ament_python` 包，40 个文件。
> 依赖 `PX4/px4_msgs` + `Jaeyoung-Lim/px4-offboard`。
> 三个核心文件本文件作者已**逐行读完**：`models/multirotor_rate_model.py`（110 行）、`controllers/multirotor_rate_mpc.py`（164 行）、`mpc_quadrotor.py`（279 行）。

> ### ✅ 2026-09-20 复核结果
>
> 本轮重新克隆并**逐项核对**，本文件 §2 的内容断言**全部成立**：
> commit `3e541afb05350e30a0867420a5df40dbdfa92c13` ✅、日期 2026-06-15 ✅、**40 个文件** ✅、
> BSD 3-Clause ✅（`Copyright (c) 2022, JaeyoungLim` —— 署名也佐证它不属 PX4 组织）、
> `px4_mpc/px4_mpc/models/multirotor_rate_model.py` ✅、`px4_mpc/px4_mpc/controllers/multirotor_rate_mpc.py` ✅、
> `mpc_msgs/srv/SetPose.srv` ✅。
>
> ⚠️ **一处已修正的错误**：本文原先把仓库写成 **`PX4/px4-mpc`，该 URL 不存在**
> （`git ls-remote` 实测回 "Repository not found"）。真身是 **`DISCOWER/px4-mpc`**。
> 按原 URL 去找会一无所获。文档内 4 处引用已全部改正。
>
> 本地克隆（Q18 决策，仓库外持久路径）：**`~/refs/px4-mpc`**。
>
> ### ✅ 2026-09-20 Q25 分支挖掘结果：**两条分支都无需再挖，前提是错的**
>
> 按 Q25=a 挖了 `dev-ned_enu_conversion` 与 `dev-attitude_setpoints`。**结论与预期相反：两者都已被
> `master` 完全包含**，且不是靠肉眼看 diff —— 是用 **tree hash** 判定的（`git diff master origin/<branch>`
> 看的是 master 自己的新工作在倒放，不是分支的贡献，这个坑会骗人）：
>
> | 分支 | HEAD | 与 master 的关系 | 实际有无独有内容 |
> |---|---|---|---|
> | `dev-ned_enu_conversion` | `82d46e9`（2025-06-24） | tree 与 `7f208fa` **逐字节相同**，二者同父 `9cdaec7`；`7f208fa` 就是已合并的 PR #33 | **无**。master 在其之上又叠了 32 个 commit |
> | `dev-attitude_setpoints` | `d3275b3`（2024-12-17） | **0 个独有 commit**（`rev-list --left-right --count master...branch` → `54  0`），是 master 的祖先 | **无** |
>
> **★ 最重要的更正：`dev-attitude_setpoints` 里根本没有姿态 setpoint 代码。**
> 决定性检查：`git log -S "VehicleAttitudeSetpoint" origin/dev-attitude_setpoints` **为空**；
> `git grep -i attitudesetpoint` **无匹配**。该分支发的仍是 `VehicleRatesSetpoint`
> + `offboard_msg.attitude = False / body_rate = True` —— 与 master **同级（C）**。
> 分支名的 "attitude setpoint" 指的是**航天器那条路径让 MPC 在代价里跟踪一个姿态四元数_参考值_**
> （`mpc_spacecraft.py:179` 的 `self.setpoint_attitude` / `:316-339` 的 `error_attitude`），
> 是**状态**，不是输出消息。→ 本文件原先"它比 master 更贴近 (B) 级"的推测**作废**。
>
> **推论（影响 Phase 2/3 设计）**：**`px4-mpc` 全部 24 条远端分支里都不存在 `VehicleAttitudeSetpoint`
> 的参考实现。** 所以若 Phase 2/3 需要 (B) 级姿态接口做回退/对照，**没有现成代码可抄，是全新工作**。
>
> **附带收获：§2.3a 的帧变换被数值复核为「正确」。** 用真值关系 `R_FLU→ENU = T · R_FRD→NED · S`
> （`T = [[0,1,0],[1,0,0],[0,0,-1]]`，`S = diag(1,-1,-1)`）在单位四元数、90° yaw、一个混合四元数上
> 逐点比对，四元数那行完全吻合。**且紧跟其后的 `q_enu /= np.linalg.norm(q_enu)` 是承重的** ——
> 那个半角映射只在输入为单位四元数时才正交，少了这行会漂。
>
> **同一份代码里两条帧变换的"不对称"，别误判成 bug**：
> - **位置/速度**（世界系 NED 向量）：`[msg.y, msg.x, -msg.z]` —— **既有 x↔y 交换，也有 z 取反**
> - **角速度**（PX4 发的是**机体系**）：`[xyz[0], -xyz[1], -xyz[2]]` —— **只取反 y/z，不交换**
> - 二者口径不同是**故意的**，因为一个在世界系、一个在机体系。
>
> **反向教训（`dev-attitude_setpoints` 的唯一价值）**：它带着**错的**帧约定（`[x, -y, -z]` + `# TODO`），
> 是 90° yaw 误差，正是后来 PR #33 修掉的东西。留作**反面参照**。
>
> **⚠️ 对我们而言，这份帧变换大部分是死代码。** px4-mpc 是「把 PX4 吐出来的 NED 转进控制器」；
> 而 Q14 已定 mocap **直出 ENU** 作为状态源 —— 所以**输入侧 (a) 用不上**，
> 我们真正要借鉴的是**输出侧 (b)**（`pitch`/`yaw` 取反、推力取负），那部分 master 已有。
>
> **两个从 master 继承的毛刺**（写代码前记着）：
> 1. 推力标度是**硬编码 `0.07`**，而 `max_thrust = 9.81/0.73 = 13.4384`
>    （`multirotor_rate_model.py:44`），其倒数 `0.0744` —— **比 `0.07` 高约 6%**，即代码里自带约 6% 的低估偏置。
>    （`mpc_spacecraft.py:298` 那条路径则是显式 `F_scaling = 1/(2*1.5)`，两者写法不一致。）
> 2. 发出去的 `thrust_body[2]` 相对求解器的 `F` 是**取负**的 —— 这对 PX4 的 FRD 约定是对的，
>    但**移植时极像 bug**，见 §2.3b。

### 2.1 ★ 最重要的一处：它的 OCP 只有 10 个状态

```python
# models/multirotor_rate_model.py
p = cs.MX.sym('p', 3); v = cs.MX.sym('v', 3); q = cs.MX.sym('q', 4)
x = cs.vertcat(p, v, q)          # nx = 10
F = cs.MX.sym('F'); w = cs.MX.sym('w', 3)
u = cs.vertcat(F, w)             # nu = 4
g = cs.vertcat(0.0, 0.0, -9.81)
force = cs.vertcat(0.0, 0.0, F)
a_thrust = v_dot_q(force, q) / self.mass
f_expl = cs.vertcat(v, a_thrust + g, 1/2 * cs.mtimes(skew_symmetric(w), q))
```

**`ω` 不是状态。** 姿态运动学里**直接用 `w`（角速率输入）当真实角速率**：

```
q̇ = ½ · skew(w) ⊗ q        ← 用 ω_des 代替真实 ω，即「内环完美跟踪」假设
```

⚠️ **这与 [`mpc-roadmap.md`](mpc-roadmap.md) §4.1 写的 13 状态（含 `ω`、含 `ω̇ = k_ω(ω_des − ω)`）不一致。**

**这个不一致不是错误，而是路线图 §3 说的「步骤 1 → 步骤 2」的分界**：

| | 状态数 | 姿态方程 | 含义 | 谁实现了 |
|---|---|---|---|---|
| **步骤 1** | **10** | `q̇ = ½·skew(ω_des)⊗q` | 假设内环瞬时，无延迟 | **px4-mpc（可直接抄）** |
| **步骤 2** | **13** | 加 `ω̇ = k_ω(ω_des − ω)` | 显式建模内环一阶延迟，需辨识 `k_ω` | 要自己写 |

→ **结论：第一版就抄 px4-mpc 的 10 状态模型。** 它更简单、已被别人飞过、且正好是路线图规划的步骤 1。`k_ω` 留到步骤 2。

### 2.2 ★ 第二重要：它的世界系是 **ENU / z 朝上**，机体是 **FLU**

```python
g = cs.vertcat(0.0, 0.0, -9.81)      # 重力朝 −z  ⇒ z 朝上 = ENU
force = cs.vertcat(0.0, 0.0, F)      # 推力沿机体 +z ⇒ 机体 z 朝上 = FLU
```

**不是 PX4 的 NED/FRD。** 它靠节点里的符号翻转把 ENU 结果喂回 NED 的 PX4（见 §2.3）。

> ⚠️ **顺带发现 [`mpc-roadmap.md`](mpc-roadmap.md) §4.2 有一处自相矛盾的公式**：该节写「`T` 沿机体 **−z** 方向（FRD）」，公式却写 `v̇ = g + (1/m)·R(q)·e_z·T`。
> 这个 `+e_z` 形式**只在 FLU/ENU 下成立**（e_z 朝上、g = [0,0,−9.81]）。在 FRD/NED 下正确写法是 `v̇ = g − R(q)·e_z·T/m`，`g = [0,0,+9.81]`。
> **两行只能对一个。** 本文 §3.2 给出决议（选 FLU/ENU），届时 roadmap 那两行需要一起改。

### 2.3 ★ 符号映射（**整个 Phase 2 最容易摔机的地方**）

> ★★ **本项目自己的权威表在 [`sign-map.md`](sign-map.md)，不在本节。**
> 本节只做一件事：**逐行读参考实现是怎么写的**，作为那份文件的交叉核对材料。
> 代码一律**回链 `sign-map.md`**、不复写数值。

节点 `mpc_quadrotor.py` 里的三个转换：

**(a) 状态进：PX4 NED → MPC 内部 ENU**

```python
# 位置
self.vehicle_local_position[0] = msg.y      # NED(n,e,d) → ENU(e,n,u)
self.vehicle_local_position[1] = msg.x
self.vehicle_local_position[2] = -msg.z
# 姿态（手写 NED→ENU 四元数）
q_enu = 1/np.sqrt(2) * np.array([msg.q[0]+msg.q[3],   # w
                                 msg.q[1]+msg.q[2],   # x
                                 msg.q[1]-msg.q[2],   # y
                                 msg.q[0]-msg.q[3]])  # z
```

**(b) 指令出：MPC 内部 FLU → PX4 FRD**

```python
thrust_rates = self.mpc.solve(...)
thrust_command = -(thrust_rates[0] * 0.07 + 0.0)
setpoint_msg.roll  = float( thrust_rates[1])   # 不变号
setpoint_msg.pitch = float(-thrust_rates[2])   # 变号
setpoint_msg.yaw   = float(-thrust_rates[3])   # 变号
setpoint_msg.thrust_body[2] = float(thrust_command)
```

> ⚠️ 上面引的是**参考实现**的写法 —— 它自带的 `0.07` 魔数**我们不抄**（§2.4 坑 1）。
> **本项目该用哪组数，见 [`sign-map.md`](sign-map.md) §2**（唯一权威）。

**(c) 归一化对照表 —— ☠️ 已迁走**

本小节原先在这里复写了一张「MPC 内部 ↔ PX4」的对照表。**已按 [ticket 03](../../.scratch/phase2/issues/03-sign-map-derivation.md)
的要求迁到唯一权威处：[`sign-map.md`](sign-map.md) §2**（原文见本文件的 git 历史）。
本节保留 (a)(b) 两段，因为它们是**参考实现的源码引用**（属"逐行解读"）；
但"**我们**该用哪组数"不在本节定义。

**与 (b) 的对照结论**：逐条一致，**无分歧点** ——
比对表与判定过程见 [`sign-map.md`](sign-map.md) §4。（这是那张票**唯一的失败模式**：两个来源都自称对，
而代码不会报错，所以对照本身必须留档。）

**★ 但 `0.07` 是个魔法数字** —— 见 §2.4 第 1 条。

### 2.4 它的五个坑（照抄会中招）

#### 坑 1 ★★ `0.07` 硬编码，且**静默**

```python
thrust_command = -(thrust_rates[0] * 0.07 + 0.0)
```

`0.07 ≈ 1/F_max`，其中 `F_max = mass·9.81/hover_thrust = 1·9.81/0.73 = 13.44 N`，`1/13.44 = 0.0744`。

**所以这行等价于 `thrust_body[2] = −F/F_max`。** 逻辑对，但：

> ⚠️ **如果你改了模型里的 `mass` 或 `hover_thrust`，却忘了同步改这个 `0.07`，推力增益会按比例错掉，而代码不会报任何错。**
> 对 z0 而言 `mass = 0.75 kg`（不是 1 kg），直接抄 `0.07` 会让推力**偏大 33 %**。

**修正：必须从模型导出，禁止硬编码。**

```python
self.thrust_scale = 1.0 / self.model.max_thrust     # max_thrust = mass*g/hover_thrust
...
thrust_body_z = -(F * self.thrust_scale)
```

> 符号与量纲的权威写法见 [`sign-map.md`](sign-map.md) §6（`T_max` 按电池、由 ticket 10 定稿）。

#### 坑 2 ★★ `solve()` 失败时直接 `raise`

```python
status = ocp_solver.solve()
if status != 0:
    raise Exception(f'acados returned status {status}.')
```

在 SITL 里这是好习惯；**在天上这是让节点崩掉**。节点一崩，~~`OffboardControlMode` 心跳~~ **下行 setpoint**停发 → PX4 触发 offboard 丢失失效保护。☠️ **2026-10-13**：换到 mavros 后**没有独立心跳流了**（**setpoint 自己就是心跳**）⟹ 机理换了名字、结论不变（见 [`../reference/px4-interface.md`](../reference/px4-interface.md) §4.3）。**必须有兜底**：求解失败时保持上一帧的可行指令（或切到悬停推力 + 零角速率），并计数上报。

#### 坑 3 ★ NaN 会**穿透推力通道**

第一手证据（PX4 `v1.14.3` `src/modules/mc_rate_control/MulticopterRateControl.cpp`，offboard 分支）：

```cpp
_rates_setpoint(0) = PX4_ISFINITE(vehicle_rates_setpoint.roll)  ? vehicle_rates_setpoint.roll  : rates(0);
_rates_setpoint(1) = PX4_ISFINITE(vehicle_rates_setpoint.pitch) ? vehicle_rates_setpoint.pitch : rates(1);
_rates_setpoint(2) = PX4_ISFINITE(vehicle_rates_setpoint.yaw)   ? vehicle_rates_setpoint.yaw   : rates(2);
_thrust_setpoint = Vector3f(vehicle_rates_setpoint.thrust_body);   // ← 没有 ISFINITE 保护
```

**三个角速率有 NaN 回退（退回实测角速率），推力没有。** 一个 NaN 推力会直达混控器。

**修正：MPC 节点发布前必须 `np.isfinite()` 全量校验，任何一项不过就整帧丢弃并走坑 2 的兜底。**

#### 坑 4 ★ `R` 罚的是**绝对**角速率，不是增量

```python
R_mat = 2*np.diag([1e1, 5e2, 5e2, 5e2])
ocp.cost.yref = [0,0,0, 0,0,0, 1,0,0,0, 0,0,0,0]   # u 的目标是 0
```

代价是 `‖y − y_ref‖²_Q,R`，而 `y = [x; u]`、`y_ref` 的 u 部分是**零**。
→ **它惩罚的是「角速率本身」，即把所有时刻的 ω 往 0 拉。** 这会系统性地压制机动性，并在需要持续角速率时（如绕圈）产生稳态偏差。

**修正：代价里应罚 `u − u_ref`（前馈/上一帧解），而不是 `u − 0`。** 这是 acados 里 `cost_type_e` / 自定义 `cost_y_expr` 的标准做法。

#### 坑 5 ★ 约束范围与 z0 不符

```python
lbu = np.array([0.0,      -wmax,      -wmax,      -0.5*wmax])
ubu = np.array([+Fmax,     wmax,       wmax,      +0.5*wmax])
```

- `max_rate = 0.5` 是**占位符**，不是 z0 的值。
- **偏航上限写成别人的一半（`0.5*wmax`）**，而 z0 的偏航上限**比横滚/俯仰更大**（6.28 vs 3.84 rad/s，见 [`params-analysis.md`](../reference/params-analysis.md)）。
- 推力下界 `0` 在 FLU 约定下**是对的**（`F ≥ 0`）。

**修正：全部换成 z0 实测参数，偏航上限单独给。**

### 2.5 相对 px4-mpc 的改动清单

| # | 改什么 | 为什么 | 危险度 |
|---|---|---|---|
| M1 | `mass` → **0.75 kg**，`hover_thrust` → **实测值** | 原值 1 kg / 0.73 是 x500 的 | 🔴 |
| M2 | 删掉 `0.07` 魔数，改成 `1/max_thrust` | 坑 1 | 🔴 |
| M3 | `max_rate` → **3.84 / 3.84 / 6.28**，且 yaw 上限不再取一半 | 坑 5 | 🔴 |
| M4 | `F_max` 按**电池**（11.8 N），不按电机 —— ✅ **2026-09-29 解耦后按原意落地**：`F` 上限改用 `Z0Params.f_max` = **11.8 N**（`ocp.py` 的 `ubu[0]`），归一化常数 `t_max`（实测 46.3 N）**不再兼任上限**。★ 回填 `hover_thrust` 时正因为两者共用一个常数，上限被悄悄抬到 46.3 N（差 3.9 倍、测试全绿），抓出它的是一条 OCP 用例与 `preflight_check` 的那条守卫 —— 见 §3.3.3 | §3.3 | ✅ |
| M5 | `solve()` 失败 + NaN 的兜底路径 | 坑 2、坑 3 | 🔴 |
| M6 | `R` 改罚 `u − u_ref` | 坑 4 | 🟡 |
| M7 | 状态源从 PX4 话题换成 `/mocap/pose_enu` | G3；且**因此不需要 (a) 那两行转换** | 🟡 |
| M8 | 删掉 `_v1` 后缀话题的双订阅 hack | `_v1` / `msg/versioned/` 是 **v1.16 才有**的机制，**z0 真身（v1.14.3 系）里根本不存在** → 这段兼容代码对我们纯属死代码（⚠️ 该结论与 09-21 那轮的措辞「fork（v1.15 世代）里不存在」**等价且更贴对象**：真身的 `msg/` 与上游 v1.14.3 逐字节相同） | 🟢 |
| M9 | 补上 `MPC_HORIZON = 15` 这个**定义了却没用的死变量** | 代码卫生 | 🟢 |

> ✅ **落地状态（2026-09-28，ticket 04 —— 只动了离线 OCP 那半）** —— 逐条的**落地位置 + 一手依据**：
>
> | 编号 | 落地位置 | 一手依据 |
> |---|---|---|
> | **M1** | `src/nmpc/nmpc/params.py`：`Z0Params.mass = 0.75` | [`hardware.md`](../reference/hardware.md) **§9 汇总表**（`:288`，标「用户实测」） |
> | **M2** | `src/nmpc/nmpc/sign_map.py:69` 的 `thrust_n / t_max_n`（**形参**，不是常数）；`t_max` 由 `hover_thrust` 导出 | 参考实现的魔数在 `px4_mpc/px4_mpc/mpc_quadrotor.py:235`（`* 0.07`），且与它**自己的模型**对不上：`1/(1.0·9.81/0.73) = 0.0744` ⟹ `0.07` 低 ~6 %（`models/multirotor_rate_model.py:42-44`） |
> | **M3** | `params.py` 三轴上限 + `ocp.py` 的 `ubu = [T_max, 3.84, 3.84, 6.28]` | [`params-analysis.md`](../reference/params-analysis.md) 的 `MC_*RATE_MAX`（`z0.params` 与飞控真身对账过 ⟹ **飞控现状**，不是占位值）；**行为**证据：4.0 rad/s 的匀速偏航被原样跟住，错接成 1.92 就卡住且**不报错**（[`offline-ocp.md`](offline-ocp.md) §3） |
> | **M4** | `params.py`：`t_max = m·g / hover_thrust`（只有 `hover_thrust` 自由） | 本文 **§3.3** 的电池 Mooch 实测放电能力 |
> | **M6** | `ocp.py`：`u_ref` 进 `yref` 控制段 | 对照实测：前馈 ⟹ `ω_z = 4.0000`；`u_ref = 0` ⟹ `0.0014`（ticket 16 后的默认口径；A 档权重下曾是 `0.0000`，一度是 `0.0139`）—— 结论"持续角速率被 `u_ref=0` 压制"两者都成立（[`offline-ocp.md`](offline-ocp.md) §3.3） |
>
> ★★ **补记（2026-09-28，ticket 09）——在线节点不再用 §3.2 的那个"缺省"**：`mpc_node` 原来喂
> `u_ref = 上一帧解`，SITL 闭环里它是一条**正反馈**并放大成 runaway（`u[3]` 爬到 +4.696 rad/s、
> 机头 13.6 圈）。现在改喂**参考自己的解析前馈**（`node_core.hold_feedforward` ⟹ 静止锚点就是
> `[T_hover,0,0,0]`），这正是 §3.2 **第二行**"调用方给的前馈（轨迹的解析前馈）"，不是"悄悄退回
> 悬停前馈"（那句禁的是 `solve()` **缺省**路径的静默退化）。第一手读数、三处一致的证据链
> 与回归测试见 [`tier-a-sitl-hover.md`](tier-a-sitl-hover.md) §3。⚠️ `ocp.py` 的**离线缺省**
> 一个字没动（`_shifted_previous` 与它那两条测试照旧）。
>
> **M5** 的"求解失败不抛异常、由健康度如实返回"那一半在 `ocp.py`（`SolveInfo.degraded`、`ok`）；
> **谁接管、怎么降级**归 ticket 06/07。**M7–M9 是节点侧的事**，本票未动。

> ✅ **落地状态（2026-09-28，ticket 06 —— 节点侧 M7–M9）** —— 逐条的**落地位置 + 一手依据**：
>
> ☠️ **2026-10-13（票 13 收口轮）**：下表 **M7 / M8 两行里的闸门话题 `/fmu/out/vehicle_status` 已作废** —— 今天 `TOPIC_STATUS` = **`/mavros/state`**（票 13 §二③），节点也不再发任何 `/fmu/in/*`；行号随之整体平移，⛔ 别照抄。见 [`../reference/px4-interface.md`](../reference/px4-interface.md) §4.3。
>
> | 编号 | 落地位置 | 一手依据 |
> |---|---|---|
> | **M7** | `mpc_node.py:284-287` 只订阅 `/mocap/pose_enu`（`geometry_msgs/PoseStamped`）+ `/fmu/out/vehicle_status`；状态由 `node_core.pose_to_state` 直接组装，**没有一行坐标系转换** | `/mocap/pose_enu` 是 `mocap_rel_pose_node.cpp:110` 用 `QoS(10)` 发的 **ENU/FLU**（节点里的引文见 `mpc_node.py:155-158`）⟹ §2.3(a) 那段手写 NED→ENU 四元数变换**整段删掉**。测试：`test_mpc_node.py::test_接口与_QoS_与票面一致`（夹具的动捕源上有订阅者）、`test_node_core.py::test_pose_to_state_不做任何坐标系转换` |
> | **M8** | `mpc_node.py:286-287` 只有**一个** `VehicleStatus` 订阅（`TOPIC_STATUS`），全仓没有 `_v1` 后缀话题 | z0 真身的 `msg/` 与**上游 v1.14.3 逐字节相同**，而 `_v1` / `msg/versioned/` 是 **v1.16 才有**的机制 ⟹ 那段兼容代码在我们这里是死代码（[`refs-manifest.md`](../reference/refs-manifest.md) §2.6）。测试：`test_接口与_QoS_与票面一致` 里的 `matched_subscribers("status") == 1` —— **恰好一个**订阅者，多一个 `_v1` 变体这条就红 |
> | **M9** | 全仓**没有**定义了却不用的 horizon 常量：`OcpConfig.N = 50`（`ocp.py:51`）在 `ocp.py:225/256/281-283`（内步长、`N_horizon`、参考的 reshape）与节点侧的 RViz 点数上**都在用** | 参考实现那个死变量：`mpc_quadrotor.py:142` 的 `MPC_HORIZON = 15` —— 它只被**注释掉**的那段代码（`:149` `N=MPC_HORIZON,`）引用，真正 spawn 的控制器（`:145`）用的是自己的缺省 horizon ⟹ 定义了却不起作用。测试：`test_mpc_node.py::test_RViz_的预测轨迹与参考点` 断言的 51 点 = `N+1`，把"用了没用的 N"这件事钉在话题上 |
>
> ⚠️ **M5 的兜底路径本票只做到"如实回报"** —— 而且**"发不发"分三类**，别当成一件事
> （**ticket 07 接手前先读这三条**，都是本票实测的行为）：
> · **非有限值**（动捕位置 / 四元数非有限）⟹ `node_core._hold("non_finite_mocap")` ⟹ **闸门关、不发**；
> · **求解退化**（`SolveInfo.degraded`：acados 状态码非零 / `u` 越出约束）⟹ **照样发**！只 `warn` + 计数
>   （`degraded_count`，进收尾摘要）—— 因为解仍有限，退化的判据在 `ocp.py` 而不是"能不能用"；
> · **映射前拒绝**（`sign_map` 对非有限的 `u` / 负推力抛 `ValueError`）⟹ **这一拍不发**（`rejected` 计数）。
> **"谁来接管、怎么降级"（悬停？切回 `(A)` 链？）归 ticket 07**（`heartbeat_always` 那个接缝就是给它的）。
> ★ 这一条是 ticket 06 的复核抓出来的：本文件原写"求解失败 + 非有限值 ⟹ 不发"，
> 而代码里**退化求解照样发** —— 07 照着错的描述接手，会以为"已经不发"了。

> 📌 **M7 顺带干掉了一整类 bug**：既然动捕原生就是 ENU/FLU，MPC 内部也用 ENU/FLU，那么 §2.3(a) 那段手写四元数转换（`1/√2·[w+z, x+y, x−y, w−z]`，**整个文件里最容易写错的一行**）**可以直接删掉**。详见 §3.2。

> ✅ **落地状态（2026-09-28，ticket 07 —— M5 的兜底路径）** —— 设计、第一手依据与配方全在
> **[`mpc-node-fallback.md`](mpc-node-fallback.md)**；这里只记 **M5 的编号与落地位置**（票面判据 7 要的就是这一行）：
>
> | 编号 | 落地位置 | 一手依据 |
> |---|---|---|
> | **M5-①** 分级兜底 | `node_core.FallbackLadder`（`normal` / `hold` / `hover`，零 ROS）；`mpc_node._normal_frame` + `_fallback_frame` 收口 | **"不发 ≠ 松手"**：`MulticopterRateControl.cpp:177-184` 把最后一帧存在**成员**里（`.hpp:124,127`）⟹ 不发 = 飞控无限期沿用陈旧命令；且 `_thrust_setpoint{}` **零初始化** ⟹ 从未发过 = 零推力（`_rates_setpoint` 三个分量有 `PX4_ISFINITE` 回退，**推力没有**） |
> | **M5-②** 发布前全量校验 | `node_core.frame_is_finite`（写进那一帧的四个数）+ `node_core.solution_usable`（状态码 / `u` 形状 / `u` 有限） | 坑 3；`u` 全 NaN 与帧非有限的注入见 `mpc_node._poisoned` |
> | **M5-③** 连续失败阈值 + 可检索上报 | `FALLBACK_HOLD_FRAMES = 10`（100 ms ≈ 28 个求解预算）；`error` 级日志 + JSONL `fallback` 列 + 收尾摘要 | 求解实测 max 3.58 ms（[`offline-ocp.md`](offline-ocp.md) §5） |
> | **M5-④** 输入合理性（时间戳倒退 / 位姿跳变 ⟹ 不发） | `node_core.NodeCore` 的两条判据 + `StopConfig` 的 `mocap_stamp_regress_s` / `mocap_jump_speed_max` / `mocap_jump_rate_max` | 阈值按飞控的 `MPC_XY_VEL_MAX = 12 m/s` 与模型的 `YAW_RATE_MAX = 6.28 rad/s` 放宽（[`params-analysis.md`](../reference/params-analysis.md):477 与消歧见 [`CONTEXT.md`](../../CONTEXT.md) §0）；★ 取向是**宁可漏判不可误判** |
>
> ⭐ **对上面那条 ⚠️ 的收口**：06 记录的"三类行为"里，**只有"求解失败（状态码非零）"这一类进兜底阶梯**；
> **退化求解（`degraded` 但 `ok`）照样发** —— 分界线定在 `SolveInfo.ok`，由
> `test_node_core.py::test_解可用吗_退化与不可用是两件事` 逐格钉住（07 没有改这条现行行为）。
> ⚠️ **新开一条开放项**：按票面，时间戳倒退 / 位姿跳变是**只关闸门、一帧不发**；
> 长时段不信任下飞控最终仍落在"锁存最后一帧"上 —— 见 §9 的 **O7**。

### 2.6 可以直接抄的部分

| 抄什么 | 位置 | 价值 |
|---|---|---|
| `skew_symmetric(q)`、`v_dot_q`、`q_to_rot_mat` | `models/multirotor_rate_model.py` | 四元数工具，是对的，省去自己推导 |
| OCP 装配骨架（`AcadosOcp` → `NONLINEAR_LS` → `SQP_RTI` → ERK 4/3） | `controllers/multirotor_rate_mpc.py` | 与 roadmap §4.5 完全一致 |
| ☠️ ~~`OffboardControlMode` 心跳（`body_rate=True`，其余 False，50 Hz）~~ **已删除**（2026-10-13，票 13 §二①：换到 mavros 后**不需要我们发心跳**） | `mpc_quadrotor.py` | ⛔ 这一行**今天无从"照抄"** |
| QoS 设置：pub `BEST_EFFORT + TRANSIENT_LOCAL + depth 1`，sub `BEST_EFFORT + VOLATILE + depth 1` | `mpc_quadrotor.py` | ⚠️ **用默认 RELIABLE QoS 会静默收不到 PX4 话题** —— 见 §6.4 |
| 预测轨迹 `Path` + 参考点 `Marker` 发布 | `mpc_quadrotor.py` | RViz 里肉眼验证 MPC 是否发疯，实飞前极有用 |

---

## 3. z0 的架构决策

### 3.1 D1–D10 决议表

| # | 决策 | 状态 | 结论 |
|---|---|---|---|
| **D1** | `(A)/(B)/(C)/(D)` | ✅ **已定** | **`(C)`** —— 见 G1 |
| **D2** | `(D)` 何时做 | ✅ **已定** | `(C)+INDI` 稳定之后，本项目范围外 |
| **D3** | INDI 是否第一版就做 | ✅ **已定** | **否**，作为步骤 2（G8） |
| **D4** | 中间件 | ✅ **已定** | ☠️ **2026-10-13 改判：全 mavros**（原写"**混合**：uXRCE-DDS 发指令 + mavros2 收动捕（G2）"）—— 见 G2 与 [`../reference/px4-interface.md`](../reference/px4-interface.md) §4.3 |
| **D5** | `T_max` 取值 | ✅ **2026-09-29 解耦后已定**：原裁定「**按电池**（11.8 N）」**保留**，但只作用于 **`F` 上限这一个角色**（`Z0Params.f_max`）；**归一化**那侧改由 P1 的实测值导出（`m·g/hover_thrust` = 46.3 N，满电口径）。★ 两者共用一个常数的时代结束（那时回填会把 `F` 上限抬到 46.3 N）。⚠️ 电池"瓶颈"的实质是**可持续电流**（15 A），不是推力归一化 —— 见 §3.3.3 与 §3.3.4 的两层表 |
| **D6** | MPC 频率 | ✅ **已定：100 Hz** | ✅ **2026-09-28 实测**（ticket 04，[`offline-ocp.md`](offline-ocp.md) §5、`python3 -m nmpc.timing --runs 1000 --cpu 4`）：**纯求解 max 3.58 ms**（三轮 3.447 / 3.584 / 3.447，取最大）、整周期 median 3.9 / p99 4.7 ms ⟹ 保 100 Hz（余量 2.8×）。⚠️ **判定只覆盖"求解器"那一列**：整周期墙钟出现过 **1/1000 次的 52–56 ms 抢占**（同拍求解只用 3.4 ms），**它越过了本行"最坏 >20 ms"那条线** —— 保留 100 Hz 是"差值是被调度出去而非求解开销"的**有理由的例外，不是满足了判据**。要真闭环在 100 Hz 上，节点侧得绑核/定优先级（归 ticket 06/07）。原验收线（≤10 ms 保 100 Hz；10–20 ms 降 50 Hz；>20 ms 停下优化）**继续作为复测判据**。★ **闭环复测（2026-09-28，ticket 09，z0 同构 SITL 真 PX4）**：空机稳态最坏 **3.995 ms** ⟹ **100 Hz 复现**（冷启动 15.956 ms 落在 `warmup` 闸门里，有专门判据盯着）；⚠️ **同一份二进制、机上多跑几个进程**时稳态最坏就变成 **11.297 ms**（⟹ 50 Hz 档）⟹ 这条判据**带"机上别跑别的东西"这个前提**，Phase 4 定实机频率必须把 Jetson 上还跑着什么算进去。另：逐拍间隔最坏的 82–93 ms 与上面那条"1/1000 次 52–56 ms 抢占"**同源**（同拍求解只用 3–4 ms）。数字与判据见 [`tier-a-sitl-hover.md`](tier-a-sitl-hover.md) §5 |
| **D7** | MPC 内部世界系 | ✅ **已定** | **ENU/FLU**（世界 z 朝上 + 机体 x 前 / z 沿推力）—— **用户选定「参考论文里的坐标系」，两篇论文一致**，见 §3.2 |
| **D8** | 状态源 | ✅ **已定** | **动捕直出**（G3） |
| **D9** | 延迟补偿 | 🔶 **仍开放** | 建议**第二版再做**。步骤 1 先用「状态外推 `x + v·τ`」这一行代码顶住，够用且零风险。★ **τ 已在 z0 上实测 ≈ 50 ms**（[`../reference/mocap-setup.md`](../reference/mocap-setup.md) **§6.3**，ticket 03）；**实飞起始值仍取 `0`**（过补偿比不补偿更危险），实测值用于确定量级与作为 ticket 10 扫 τ 的起点参照 |
| **D10** | 参考轨迹来源 | ✅ **已定** | **手写**（G9）；`ego-planner` 不在本项目范围 |

### 3.2 ★ D7 决议：**ENU / FLU**（依据：两篇论文的坐标系）

**用户决定：「参考论文里的坐标系」。** 本项目控制路线的两篇直接依据 `Sun2022`（`(C)`+INDI）与 `Foehn2022`（CTBR），**它们用的坐标系完全一致，且就是 ENU/FLU**。

**第一手证据（原文引用）**：

> **`Sun2022_TRO_NMPC_vs_DFBC` §III.A Notations**：
> *"Two right-handed coordinate frames are used in this paper: the inertial frame **`F_I`**: `{x_I, y_I, z_I}` with **`z_I` pointing upward opposite to the gravity**, and the body-frame **`F_B`**: `{x_B, y_B, z_B}` with **`x_B` pointing forward** and **`z_B` aligned with the collective thrust direction**."*

> **`Foehn2022_SciRob_agilicious` §D.1 Pilot**：
> *"...we use a right-handed coordinate system located in the center of gravity, with the **`e_g` pointing in body-relative upward thrust direction**, and **`e_x` pointing along the drone's forward direction**. Motion is represented with respect to an inertial world frame with **`e_z` pointing against the gravity direction**, where **translational derivatives (e.g. velocity) are expressed in the world frame and rotational derivatives (e.g. bodyrate) are expressed in the body frame**."*

**两张表读出来的约定**：

| | 论文 | PX4 | z0 的 MPC 内部 |
|---|---|---|---|
| **世界系** | 惯性系 **z 朝上**（逆重力） | NED，z 朝**下** | **ENU**（z 朝上） |
| **机体系** | x 朝前、**z 沿推力方向** | FRD，z 朝**下** | **FLU**（x 前 / z 上 = 推力） |
| **平移量** | 世界系 | 世界系 | ENU |
| **旋转量（ω）** | **机体系** | 机体系（FRD） | 机体系（FLU） |
| **重力** | `g = [0,0,−9.81]` | — | **`g = [0,0,−9.81]`** |
| **推力** | 沿机体 **+z** | 沿机体 −z | **`force = [0,0,F]`** |

**这三条恰好就是 §2.2 里 px4-mpc 用的那一套**（`g=[0,0,−9.81]`、`force=[0,0,F]`）。**论文、px4-mpc、动捕原生输出，三者同一约定** —— 所以模型代码可以逐字复用，输入侧零转换。

**与本文原先的建议一致的另外三条理由（保留供追溯）**：

1. **动捕原生就是 ENU/FLU。** 状态源是 `/mocap/pose_enu`（G3/G11），**输入侧零转换**。若选 NED/FRD，反而要在输入侧做 ENU→NED + FLU→FRD，其中包括**最难写对的那一行四元数变换**（px4-mpc 的 `1/√2·[w+z, x+y, x−y, w−z]`）。
2. **模型代码可以逐字复用。** px4-mpc 的 `multirotor_rate_model.py` 在 ENU/FLU 下**一个字都不用改**。选 NED/FRD 就要改重力符号和推力符号 —— 这正是 §2.2 指出的、roadmap 自己都写矛盾了的地方。
3. **转换总数相同，但风险不同。** 两种方案都恰好 1 处物理转换：
   - ENU/FLU：输出侧 —— **4 行里 3 行纯符号翻转、1 行取反 + 归一化，肉眼可验**（表与推导：`sign-map.md` §2/§4）。
   - NED/FRD：输入侧 ENU→NED 四元数 + FLU→FRD 姿态 —— **需要矩阵/四元数推导，出错概率高一个数量级**。

> **结论：把误差风险挪到「纯符号翻转」那一侧，而不是「四元数变换」那一侧。**
> **代价**：`[`mpc-roadmap.md`](mpc-roadmap.md) §4.1/§4.2 的 NED/FRD 表述需要同步改写 —— **已完成**（见该文件 §4.2 的更正注记）。

**★ 顺带一个对 §2.1 的旁证**：**两篇论文都把 `ω` 作为状态**。
`Sun2022` 的 NMPC 状态是 `x = [ξ, ξ̇, q, Ω^B]`；`Foehn2022` 的状态是 `x = [p, q, v, ω, a, τ, s, b_ω, b_a, f_d, f]`。
→ **本文 §2.1 的「步骤 1 用 10 状态（ω 不作状态）」是简化，不是论文的标准做法。** 这不改变结论（先抄 px4-mpc 跑通、再上 13 状态），但**说明步骤 1→步骤 2 的升级方向与论文一致**，不是我们在自创。

### 3.3 ★ 推力上限：为什么必须按电池，以及它怎么与 `hover_thrust` 自洽

> **本节 2026-09-28 由 [phase2/10](../../.scratch/phase2/issues/10-param-measure-backfill.md) 重写**
> （原"倾角收到 30°"那版已被操作者裁定作废，见 §3.3.4）。

#### 3.3.1 第一手证据：推力通路在三处都不重标定

PX4 `v1.14.3`：

1. **角速率环原样透传** —— `MulticopterRateControl.cpp:182` offboard 分支：
   ```cpp
   _thrust_setpoint = Vector3f(vehicle_rates_setpoint.thrust_body);   // 原样使用
   ```
2. **分配器只在"归一化推力"这个域里做线性混合** —— `ControlAllocation/ControlAllocationPseudoInverse.cpp:134`：
   ```cpp
   _control_allocation_scale(3 + axis_idx) = norm_sum / num_non_zero_thrust;
   ```
   推力轴按**各执行器 `ct` 的均值**归一化 ⟹ **`thrust_body = 1.0` ≡ 所有执行器满输出**。
   （`lib/mixer_module/functions/FunctionMotors.hpp:77-101` 那条 `THR_MDL_FAC` 曲线只改
   「归一化推力 ↔ 电机信号」的**映射形状**，端点不动（`x = 1 → 1`），也**不涉及悬停推力** ——
   分配器与我们的模型工作在**同一个域**里。）
3. **没有电池电压缩放** —— `MC_BAT_SCALE_EN = 0`（z0 实测值，保持默认），`thrust_body` 不被缩放。

→ **MPC 输出的归一化推力直接就是混控器的输入。** 所以 `F/N → thrust_body` 的换算常数（= `1/T_max`）**必须反映飞机真实能出的最大推力**，否则模型与真实被控对象不符。

#### 3.3.2 ★ 自洽条件（本张的硬要求）

`hover_thrust` 与 `1/T_max` 必须**出自同一次测量**，因为它们本来就是同一条等式的两端：

```
hover_thrust ≡ T_hover / T_max = m·g / T_max   ⟺   T_max = m·g / hover_thrust
```

右边就是 `Z0Params.t_max` 那条 property（`params.py`）；左边的当场换算在
[`tools/hover-thrust`](../../tools/hover-thrust/README.md) 的 `t_max_for()`。
**两边不能各自独立取数**：一边填实测的 `hover_thrust`、一边填理论 `T_max`，推力标度就错了。

★ **反例（不要这么写）**：把模型里的 `hover_thrust` 填成 `MPC_THR_HOVER = 0.5`。
那是 **PX4 位置控制器**自己的增益（缩写撞脸，与 MPC 无关），z0 上它是 `0.5`；
而实测真值是 **`0.1589`**（§3.3.3）—— 两者无关，填错就是 §6.3 那类"静默的推力标度错误"。

☠️ **而且这个坑真踩过**：本张 2026-09-29 之前一直用的是占位 `hover_thrust = 0.6235`（⟹ 隐含 `T/W = 1.60`），
实测是 `0.1589` ⟹ **占了 3.9 倍的推力标度错误**，而且在任何测试里都不报错
（节点按参数表飞、测试按同一个模型算预期 ⟹ 全绿而飞机是错的）。见 §3.3.3 与 [ticket 10 的 Comments](../../.scratch/phase2/issues/10-param-measure-backfill.md)。

#### 3.3.3 z0 的数（质量 750 g 实测）

★ **2026-09-29 起这一行是实测的**（P1 回填，[ticket 10](../../.scratch/phase2/issues/10-param-measure-backfill.md)）。
口径：**满电 25.2 V**（O8 的裁定），推导链条的第一手事实见该票 Comments 第二轮。

| 量 | 值 | 来源 |
|---|---|---|
| 悬停推力 `T_hover` | 0.75 × 9.81 = **7.36 N** | 质量实测（工具侧用 9.80665 ⟹ 7.355 N，差 0.03%，各自内部自洽） |
| **`hover_thrust`（P1）** | **`0.1589`**（满电口径） | ✅ **实测**：两段可用悬停日志读数 `0.221992` / `0.220884`（@ **20.474 / 20.526 V**，段内电压中位数），按**就地标定**的 `thrust ∝ V^1.608`（两段共 4056 样本合并，R² 0.79）归一到 25.2 V ⟹ `0.15896` / `0.15882`（互差 0.09 %）⟹ 均值 0.15889。工具与判据见 §5 与 [`tools/hover-thrust`](../../tools/hover-thrust/README.md) |
| **归一化上限 `T_max`（= 模型里那个）** | 7.36 / 0.1589 = **46.3 N**（`T/W = 6.29`） | ⚠️ **不是独立取的数** —— 它就是上面那条自洽条件的另一半（`params.py` 的 `t_max` 是 property） |
| ⚠️ 电池**可持续**推力上限 | ≈ 11.8 ~ 18.7 N（T/W 1.60–2.54）<br>★ **取保守端 11.8 N** ⟹ `f_max` | [`hardware.md`](../reference/hardware.md) §6.2 的**电流瓶颈**外推 —— ★ **这是另一件事，见下面那段**。取 15 A（Mooch 实测）而非 30 A（厂商标称）：飞控自己的 `BAT_AVRG_CURRENT = 15.0` 也是这个口径 |
| ⚠️ 电机 + 桨的参考上限 | 40.0 N（T/W 5.44） | 同上。⚠️ 实测的 46.3 N 已**超过它 16 %** ⟹ §6.2 那条外推偏保守（它自己 §6.4 就声明了"必须实测替换"）。★ 但折到**每电机**看就不紧张：46.3/4 = **11.6 N/电机 @25.2 V**，按同一个 `V^1.608` 折回 24 V 是 **10.7 N**，与台架点的 10.0 N/电机**差 7 %**（而台架用的还是**另一支桨** GFD90-3）⟹ 量级自洽，46.3 N 不是外推跑飞 |

**三个后果**（满电、自洽）：

1. `thrust_scale = 1/46.3 = 0.0216`；**悬停归一化推力 = −7.36/46.3 = −0.1589**（= `hover_thrust` ✓ 自洽）。
2. **悬停只占满量程的 15.9 %** ⟹ 归一化意义上"上行裕度"有 0.841；净上行加速度
   `(46.3 − 7.36)/0.75 = 51.9 m/s² ≈ 5.3 g` —— ⚠️ **但这个数只在"电池能供出这么多电流"时成立**，见下。
3. **45° 倾角下所需推力 = 7.36/cos45° = 10.4 N，只是 46.3 N 的 22.5 %** —— 见下一节。
   ⚠️ **这 22.5 % 是"归一化"那一层的说法，别当成"还有 77.5 % 余量"**：同一个 10.4 N 占 `F` 上限（`f_max` = 11.8 N）的 **88.2 %**。哪一层在约束 `F` 见下一节的两层表。

★★ **P1 回填暴露出来一个结构问题（比换个数重要）：`T_max` 当初兼了两个角色**，
而这两个角色**没法用同一个常数表达**。**✅ 2026-09-29 操作者裁定：解耦**（见下）：

| 角色 | 该取什么 | 现在是谁 |
|---|---|---|
| ① **归一化**常数（`F/N → thrust_body` 的换算） | **必须 = 实测的满量程推力**（`m·g/hover_thrust` = 46.3 N）—— 这就是 §3.3.2 的自洽条件，填别的值悬停前馈就错 | ✅ `Z0Params.t_max`（属性，由 `hover_thrust` 导出） |
| ② **OCP 的 `F` 上限**（`ocp.py` 的 `ubu[0]`） | M4/D5 要求"按**电池**"（**11.8 N** = Mooch 持续放电 15 A 口径，`T/W` 1.60）：命令行不出来的推力不会被执行器/电池兑现 | ✅ **`Z0Params.f_max`**（新字段，2026-09-29 拆出来） |

★ **这两个角色差得不只是"口径"，还有一层物理**：①是**悬停工况**下量出来的
（第一手：两段悬停日志的电流中位 **4.305 A / 4.075 A**，掉压可忽略），而"电池能供出多少"要在
**满油门工况**下问 —— 那时电芯内阻（VTC6 6S1P 13 mΩ/cell ⟹ 78 mΩ/包，§6.2）会把电压拽下去：
§6.2 的台架点 **87.2 A ⟹ 掉 ~6.8 V**，25.2 V 的包只剩 ≈18.4 V，同一个 `V^1.608` 下满量程推力
就不是 46.3 N 而是 ≈28 N 了。⚠️ 这个 6.8 V 是从 §6.2 的外推电流算的**量级估计**（不是实测），
但它说明一件事：**"满量程推力"本身随负载变**，所以拿它当 `F` 上限就等于假设电池在任何负载下
都保持空载电压。

☠️ **合用一个常数的代价踩过一次**：回填 `hover_thrust` 之前，`m·g/0.6235 = 11.80 N` 恰好≈"按电池"那档
（**巧合** —— 同构机当初照占位搭的），所以没人发现 ① 和 ② 是同一个数。回填把 ① 抬到 46.3 N，
`ocp.py` 的 `F ≤ T_max` 就**悄悄跟着**从"按电池"漂成"按电机"（差 **3.9 倍**），
而且**测试全绿**（节点按参数表飞、测试按同一个模型算预期）。抓出它的是
[`test_ocp_constraints.py`](../../src/nmpc/test/test_ocp_constraints.py) 里那条
"又远又高的目标该把推力顶到上限"—— 它以 `38.515 == 46.303` 红了一次，**该红**。

⚠️ **那条守卫也抓到了它**：[`preflight_check.py`](../../src/nmpc/nmpc/preflight_check.py) 的
"推力上限按**电池**取值"（`ok = z0.t_max < 40.0`）⟹ 回填后 `preflight_check` **红 1 项**。
✅ **解耦后这条守卫跟着改了判据对象**（现在看 `f_max`，并新增一条"`f_max` 夹在悬停与满量程之间"），
`preflight_check` **24/24 全绿**。⚠️ 措辞上要说清这次改的是**判据对象**、不是把判据放宽 ——
原来的红是**指错了东西**（它拦的是归一化常数，而那个按定义就该 46.3 N）。

★ **解耦后"倾角撑不撑得住"要分两层看，两层的答案完全不同** —— 这一条比上面那张表更容易被误用：

| | 45° 需要 10.40 N，占多少 | 物理天花板 |
|---|---|---|
| ①归一化层（46.3 N） | 22.5 %，余量 77.5 % | 80.9° |
| ②**电池层（`f_max` = 11.8 N）** | **88.2 %，余量只剩 11.8 %**（30° 占 72.0 %、60° 占 124.7 % ⟹ 不可行） | **51.4°**（= 早先那个 51.3°） |

⟹ §3.3.4 那句"45° 已经很勉强的张力**在归一化这一层上不存在**"是对的，
但**在电池这一层它仍然成立**：2026-09-28 裁"两边都回 45"时看到的 88.4 % 占用，
口径当时算错了（算在归一化头上），**结论本身在电池那一侧站得住**。

#### 3.3.4 ★ 倾角的量纲依据（`hover_thrust` 就是 `1/(T/W)`）

因为 `hover_thrust = T_hover/T_max = m·g/T_max = 1/(T/W)`，倾角问题可以完全在量纲上算清楚：
**悬停倾角 θ 需推力 `m·g/cos θ ≤ T_max`**（两边同除以 `T_max/cos θ`）⟹

```
cos θ ≥ hover_thrust   ⟹   物理最大可持续倾角 = acos(hover_thrust)
```

★ **方向不能写反**：倾角越大 `cos θ` 越小、越难满足。以实测的**满电**口径 `hover_thrust = 0.1589` 为例：
45° 时 `cos45° = 0.707 ≥ 0.1589` 成立 ⟹ 撑得住；**80°** 时 `cos80° = 0.174 ≥ 0.1589` ⟹ 还撑得住（余量只剩 8.5 %）；
**85°** 时 `cos85° = 0.087 < 0.1589` ⟹ 撑不住。
⚠️ 早先这里用的是 `45° / 60°` 那对例子 —— 那是按**占位** `0.6235` 算的，回填后 60°（`cos60° = 0.500`）**也撑得住**了，
例子会失去分辨力，故换到 80°/85°。这与 [`mpc-roadmap.md`](mpc-roadmap.md) 里那条
`R(q)·e_z · e_z ≥ cos(θ_max)` **同向**（那里的 `R(q)·e_z · e_z` 就是 `cos θ`）。

| 电池状态 | `V` | `hover_thrust` | `T_max` | `T/W` | **物理天花板 `acos(hover_thrust)`** | 30° 占用 | **45° 占用** | 60° |
|---|---|---|---|---|---|---|---|---|
| **满电（O8 口径 = 模型用的那个）** | 25.2 V | **0.1589** | 46.3 N | 6.29 | **80.9°** | 8.50 N = 18.3 % | **10.40 N = 22.5 %**（余量 **77.5 %**） | 14.71 N = 31.8 % |
| 标称 | 22.2 V | 0.195 | 37.8 N | 5.13 | 78.8° | 22.5 % | 27.6 % | 39.0 % |
| 实用截止 | 18.0 V | 0.273 | 27.0 N | 3.66 | 74.2° | 31.5 % | 38.6 % | 54.6 % |

> 表格按 `m·g = 7.36 N`（`0.75 × 9.81`）与**实测律** `hover_thrust(V) = 0.1589 × (25.2/V)^1.608` 算。
> ⚠️ **25.2 V 与 22.2 V 两行是对 3.4 % 电压跨度的外推**（拟合区间是 20.2–21.3 V，R² 0.79）——
> 拟合出的指数 `n = 1.608` 本身有区间（同一方法的复算 1.608 ~ 1.611、用滤波电压 1.50、理想值 2.0）
> ⟹ 满电那行的 `0.1589` 换成 `n ∈ [1.5, 2.0]` 会在 `0.163 ~ 0.147` 之间动（±5 %），
> 换成 `n = 1.61` 则是 `0.1588` —— **这两种取整差 0.06 %**，不改变任何结论。
> **口径要能当场复算**：`hover_thrust(V) = 读数 × (V_窗口 / V)^1.608`，
> 例：`0.221992 × (20.474 / 25.2)^1.608 = 0.15896`、`0.220884 × (20.526 / 25.2)^1.608 = 0.15882`。
> ★ 早先这张表是 `T/W` 1.60 / 2.54 两行（给出 51.3° / 88.4 %）—— 那组数**随占位作废**（2026-09-29 重算），
> 换高 C 电池那一行不再是"把 `T/W` 从 1.60 提到 2.54"：电池的约束落在**可持续电流**上，不在归一化上（见下）。
> ⚠️ **这里只算"归一化意义上撑不撑得住"**（= 推力够不够）。"电池能不能**持续**供出这么多功率"是**另一层**，
> 由 §6.2 的电流瓶颈管 —— 上表任何一行都**没有**把这个算进去。

★★ **第二层**（2026-09-29 解耦后它有了自己的常数）：把 `F` 上限换成 `f_max = 11.8 N`（按电池，
Mooch 15 A），同一张表的结论**完全不同**：

| 约束 | 45° 需要 10.40 N，占多少 | 天花板 |
|---|---|---|
| ①归一化 `t_max` = 46.3 N（**推力够不够**） | 22.5 % | 80.9° |
| ②电池 `f_max` = 11.8 N（**电池给不给得出**） | **88.2 %**（余量 11.8 %；30° 72.0 %、60° 124.7 % ⟹ 不可行） | **51.4°** |

⚠️ **别只看第一层就宣布"45° 轻松"** —— 第二层才是真正的工程约束：按 11.8 N 算，
45° 已经把电池可持续推力用掉 88 %，60° 直接不可行。这与 §3.3.3 那张两层表是同一件事。

★ **这张表就是"最大倾角"的量纲依据**，也是换电池那件事的量化收益：

- **实测**（满电）`T/W = 6.29` ⟹ **归一化侧物理天花板 80.9°**，取 45° 只占 22.5%、
  **余量 77.5%**。☠️ 早先那版说"当前 T/W = 1.60 ⟹ 天花板 51.3°，45° 用掉 88.4%、余量只剩 11.6%" ——
  那是按**占位** `hover_thrust 0.6235` 算的，**随实测回填作废**。
  ★ **推论**：那个"45° 已经很勉强"的张力**在归一化这一层上不存在**；
  ⚠️ **但它在电池那一层仍然存在**（上表第二层：45° 用掉 `f_max` 的 88.2 %、天花板 51.4°）。
  换句话说，2026-09-28 那次裁定看到的 88.4 % **不算错，只是算在了错的层上**。
- 曾想收到 **30°**（按旧占位算能把占用从 88.4% 降到 72.2%）来缓解那条张力。
  ☠️ **2026-09-28 操作者裁定「两边都回 45」**（SITL 侧原计划的 30° 一并作废）——
  见 [`sitl-z0-plant.md`](sitl-z0-plant.md) §5.3 与 [`params-changes.md`](../reference/params-changes.md) §E6.4。
  **本节按裁定保留 45°**。⚠️ 裁定后这段曾被写成"当初缓解张力的那个理由已经不成立"——
  **那句只在归一化这一层对**：按 `f_max` 算，45° 占 88.2 %、30° 占 72.0 %，
  30° 仍能把占用降 16 个百分点 ⟹ **那条张力在电池层是实测意义上的真约束**，
  只是它现在由 `f_max` 表达、而不是由"归一化上限"表达（45° 仍是 PX4 的现值与裁定值，**不改它**）。
- **换高 C 电池（D5 / O6）**：★ **口径要摆对** —— 电池**不是归一化上限的瓶颈**
  （实测归一化上限 46.3 N **高于** §6.2 给的电机参考上限 40 N ⟹ "电池把 `T/W` 压到 1.60"这个说法
  在**归一化**这一层作废）。⚠️ **但它在 `F` 上限那一层仍然是瓶颈**：解耦后 `f_max` 取的就是
  电池可持续口径（11.8 N，≈电机参考上限 40 N 的 **29 %**）⟹ 换电池真正影响的是 `f_max`。
  电池真正的约束是**可持续电流**（VTC6 6S1P：Mooch 实测 15 A，§6.2）。
  ⟹ 换高 C 电池的收益要按 §6.2 那套**功率**分析重估；⚠️ 量级上把 §6.2 的 `P ∝ T^1.5`
  锚到**实测悬停点**（89 W / 750 gf）重算，可持续 `T/W ≈ 2.4`（**是外推，且电流计不可靠，只当量级**）。
  **这是硬件决策，与实施步骤无关 —— 参数化隔离即可**：
  改动只落在 `Z0Params.mass / hover_thrust`（⟹ `t_max`）与 `f_max` 以及将来若加的倾角约束，
  其余代码不动。
  ⚠️ 但**归一化侧不要跟着电池动**（§3.3.2）：`hover_thrust` 已按**实测满量程**定稿，
  电池降额该进 `f_max`（已解耦，见 §3.3.3）。

⚠️ **两点必须说清楚（否则上面那张表会被误用）**：

1. **`MPC_TILTMAX_AIR` 不在本项目的通路上。** 它是 PX4 **位置控制器**的参数（缩写撞脸，
   与 MPC 无关）。☠️ **2026-10-13**：本句原文写"本项目的节点发的是 `offboard_control_mode.body_rate = True`（`mpc_node.py`）"—— **换到 mavros 后那条心跳流已删**（票 13 §二①），今天走 `/mavros/setpoint_raw/attitude` 的 `body_rate` 字段，PX4 侧同样只置 `flag_control_rates_enabled` **⟹ 本条结论不变**。原文留存：本项目的节点发的是 `offboard_control_mode.body_rate = True`（`mpc_node.py`），
   按 `commander/ModeUtil/control_mode.cpp:144-146` 只置 `flag_control_rates_enabled`，
   于是 `:169-175` 把 `flag_multicopter_position_control_enabled` 判为 **false**
   ⟹ **位置控制器（含它的倾角限幅）根本不参与**。所以 45° 这个数**不代表**MPC 被限制在 45°。
2. ⚠️ **而且模型侧目前没有任何倾角约束。** OCP 的约束只有
   `lbu/ubu = [F ∈ [0, f_max]] × [ω ∈ ±ω_max]`（`ocp.py`；★ 2026-09-29 解耦后上限是 `f_max`）。
   [`mpc-roadmap.md`](mpc-roadmap.md) 里那条 `R(q)·e_z · e_z ≥ cos(θ_max)` **是计划，不是实现**。
   ⟹ 当前**唯一**的实际天花板就是 `F ≤ f_max`（= 上表第二层的 **51.4°**；
   归一化那一层是 80.9°，但它**不是**可行域边界 —— `F > f_max` 的推力根本规划不出来）。
   ⚠️ 早先这里把两者混成一个数，于是得出"天花板 51.3°"（占位时代 11.8 N 那套）——
   **数值碰巧对，理由错了**；现在它由 `f_max` 明确表达。
   **要不要把倾角约束真的加进 OCP 是另一张票的事**：本张只把量纲关系写清楚，**不趁机加约束**（那会改模型）。

#### 3.3.5 ★ 核对记录：PX4 `MPC_THR_HOVER` 与模型 `hover_thrust`（改动 `phase4/02-C1`）

> 本节是 [phase4/02](../../.scratch/phase4/issues/02-hover-thrust-backfill.md) 的交付物。票面要四件事：
> **两个值各是什么**、**从哪个实测数字来**、**为什么必须相等**，外加两条复核（与 `THR_MDL_FAC` 的关系、
> 与理论 `42.9 %` 的差异）。下面逐条答。⚠️ **飞控侧那一处还没写** —— 它要在 QGC 里做、是**人的动作**，
> 票面就是因此定为 `ready-for-human`（落地状态见 ⑦）。

**① 两个值各是什么（必须是同一个物理量）**

| 位置 | 名字 | 在哪 | 现值 | 应取 |
|---|---|---|---|---|
| 飞控 | `MPC_THR_HOVER` | `z0.params:628` | **`0.5`** ← PX4 默认值，看不出被改过 | **`0.1589`** |
| 模型 | `hover_thrust` | `params.py:62`（`HOVER_THRUST`） | **`0.1589`** | 不变 |

两者都是「**悬停需要的那份推力，占满量程的比例**」，都工作在**归一化推力域**（= `thrust_body` 那个域）。
第一手链条见 §3.3.1：`PositionControl.cpp:210` 的
`collective_thrust = _acc_sp(2)·(_hover_thrust/g) − _hover_thrust` ⟹ 悬停时 `thrust_body[2] = −MPC_THR_HOVER`；
而模型侧 `hover_thrust ≡ T_hover/T_max` 里的 `T_max` 正是 `sign_map.px4_rates_setpoint(..., t_max_n)`
那个归一化常数。**PX4 对 `thrust_body` 不做任何重标定**（§3.3.1）⟹ 这两个数只有相等，前馈才与模型自洽。

**② 从哪个实测数字来**

| 步 | 数 | 出处 |
|---|---|---|
| 两段悬停日志的工具读数 | **`0.22199`**（@ 20.479 V）/ **`0.22088`**（@ 20.526 V） | `bag/old/3_traj1_hover/fcu/log_2_2026-9-29-15-29-26.ulg`（窗口 122.338–192.758 s）、`bag/old/4_traj3_lemniscate/fcu/log_3_2026-9-29-15-32-40.ulg`；工具 [`tools/hover-thrust`](../../tools/hover-thrust/README.md) |
| 按实测律 `thrust ∝ V^1.608` 归一到**满电 25.2 V**（O8 口径） | `0.15896` / `0.15882`（互差 0.09 %） | 同上；两段共 4056 样本合并拟合，R² 0.79 |
| **取 `0.1589`** | 0.15889 的取整 | ⟹ 模型侧已回填（§5 的 P1）；**飞控侧 2026-10-13 已写入并读回** |

★ 窗口内电压中位数本次复算为 **20.4786 V**（§3.3.3 与 ticket 10 记 20.474 V，差 0.02 %，同一窗口的两种取样口径）。
口径可当场复算：`0.221992 × (20.479 / 25.2)^1.608 = 0.1590`。

**③ 为什么必须相等 —— 以及这条等式在模型侧怎么被钉住**

模型侧**不存第二个常数**：`t_max` 是 `t_hover/hover_thrust` 这条 property（`params.py:116-127`）
⟹ `7.3575 / 0.1589 = 46.3027 N`。归一化常数与悬停推力**出自同一条等式**，不可能各自独立取值（§3.3.2）。
`preflight_check` 当场把它合成一个数（2026-10-09 复跑，**24/24 全绿**）：

```
✅ 悬停归一化 = −hover_thrust：−F/T_max = -0.158900，−hover_thrust = -0.158900
✅ F 上限夹在悬停与满量程之间：t_hover = 7.3575 N ≤ f_max = 11.800 N < t_max = 46.303 N
```

☠️ **只改一处的后果，量级在 z0 上已经量到过**：`MPC_USE_HTE = 1` 时位置控制器用的是 HTE，
而现值 `0.5` 把 HTE 的窗口压成 `[0.300, 0.700]` ⟹ 实测估计值**整段贴在下限 0.300**、`valid` 占 97.3 %，
即那几场飞行里位置控制器**一直按 0.300 前馈**，而真实需要 ≈0.222 —— **36 % 的差由竖直积分器在补**
（[ticket 10](../../.scratch/phase2/issues/10-param-measure-backfill.md) 的第三条交叉验证）。
模型侧那一边同样是静默的：`thrust_body` 语法完全合法，只有从飞行日志里才分辨得出是哪一环错了。

**④ `THR_MDL_FAC = 0.45` 复核（票面第 4 项）**

先答**关系**：`THR_MDL_FAC` 在**更下游**（`FunctionMotors.hpp:77-101` 决定「归一化推力 ↔ 电机信号」的
**形状**），**不在 `thrust_body` 这个域里** ⟹ 改 `MPC_THR_HOVER` **不需要**动它（§3.3.1 第 2 条）。
但它决定「悬停**推力**」与「悬停**油门**」怎么互换，所以复核推力模型时**不能跳过**（⑤ 就要用）。

再答**这个值可不可信**。2026-10-09 新增一把只读的尺子
[`tools/hover-thrust/thrust_curve_check.py`](../../tools/hover-thrust/thrust_curve_check.py)：
把 `fac` 当**自由参数**扫 `[0, 1]`（201 点），看飞行数据把 r² 的**峰**顶在哪里 ——
判据只吃曲线**形状**（输出域那把 DShot 仿射是自由的 ⟹ 不需要任何绝对标定，自检里有一条钉着这点）：

| 日志 | `fac` 的 argmax | 平均 r² | 对照 `fac=0`（无曲线） | 对照 `fac=1`（`T ∝ 油门²`） |
|---|---|---|---|---|
| `bag/old/3_traj1_hover/…log_2…ulg` | **0.455** | 0.999823 | 0.995218 | 0.987114 |
| `bag/old/4_traj3_lemniscate/…log_3…ulg` | **0.450** | 0.999940 | 0.995464 | 0.983590 |

⟹ **`z0.params:947` 的 `0.449999988` 被飞行数据支持**（网格步长 0.005，故分辨到 ±0.005）。
★ 顺带**独立复现**了 D7 的电机序：脚本**不硬编码**对应关系，两条日志都自己推出
`out0←ctrl[2]、out1←ctrl[1]、out2←ctrl[0]、out3←ctrl[3]`（且是个双射），与
`PWM_MAIN_FUNC1..4 = 103, 102, 101, 104` 相符。
⚠️ **顺带更正一条算式**：ticket 10 写「DShot 值 ≈ 640/1999 = 0.318」—— 数没写错
（电机信号的均值确实是 0.3187），但**算错了**：日志里那把仿射是 `output ≈ 1889·x + 107`（**有零偏**），
正确写法是 `(641 − 107)/1889 = 0.283`。**这种错下次复算会得出不同的数。**
原始输出与复跑命令：[`tools/hover-thrust/evidence/thrust_curve_2026-10-09.txt`](../../tools/hover-thrust/evidence/thrust_curve_2026-10-09.txt)。

**⑤ 与理论推算 `√(187.5/1020) ≈ 42.9 %` 的差异 —— 记录，不取平均（票面第 5 项）**

★ **这两个数根本不在同一个域里**，所以"差得多"要先拆开：

- 42.9 % 在 [`hardware.md`](../reference/hardware.md) **§6.3①** 里写在**「悬停油门」**名下，
  由 `√0.1838` 得来 ⟹ 隐含 `T ∝ 油门²`，即 `fac = 1`。而 ④ 已经把 `fac = 1` 否掉了
  （r² 0.9871 / 0.9836）⟹ **42.9 % 是电机信号域的数，从来就不是 `MPC_THR_HOVER` 的候选值。**
  ⚠️ `params-changes.md` §C1 原先把它当作本参数的"理论值"，那是**域写错**（该行 2026-10-09 已改）。
- **同域对照**（把理论放回**归一化推力域**，并折到实测那一段的电压）：

| 量 | 值 | 怎么来 |
|---|---|---|
| 理论推力比（台架点 24 V） | `187.5/1020 =` **0.1838** | `hardware.md` §6.3①（每电机悬停 187.5 g ÷ 满量程 1020 g） |
| 同一个理论折到 20.479 V | `187.5/(1020 × 0.7748) =` **0.2373** | `(20.479/24)^1.608 = 0.7748`，用实测律 `V^1.608` |
| **实测**（同一段日志、同一电压） | **0.22199** | ② 的工具读数 |
| ⟹ 差 | **理论高 6.9 %**（`0.2373/0.22199 = 1.069`） | —— |
| 折到满电口径 | 理论 `0.1700` vs 模型 **`0.1589`** | 同样是 ~7 % |

**结论**：同域差 **6.9 %**（理论偏高），与 `hardware.md` §6.4 自己声明的三条精度限制同量级
（台架用的是**另一支桨** GFD90-3、实测点在 24 V、`P ∝ T^1.5` 是经验式）。
**`hover_thrust` 取实测 `0.1589`，不与理论平均**；这条 6.9 % 保留为已知偏差、记在案。

**⑥ 改这一处的连带效应（都要记账，别只记主效应）**

`MPC_THR_HOVER` 在 PX4 里不只喂位置控制器。**这不影响离板 / MPC 那条通路**
（我们发 `body_rate`，位置控制器不参与，§3.3.4 第 1 条），但会影响**手动模式**与**降落检测**：

| # | 连带 | 现值（`0.5`） | 改成 `0.1589` 后 |
|---|---|---|---|
| a | **HTE 的窗口**（`MulticopterHoverThrustEstimator.cpp:90-93` 造窗 + `zero_order_hover_thrust_ekf.cpp:124` 做夹钳） | `[0.300, 0.700]`；实测估计值**整段贴在下限**、`valid` 97.3 % | `[0.000, 0.3589]` ⟹ **不再夹住**，可收敛到真实值（那几场里 ≈0.222）。⚠️ 下界被钳到 `0`（因为 `0.1589 < HTE_THR_RANGE = 0.2`），无害但不再是 ±0.2 的对称窗 |
| b | **手动油门的杆曲线**（`MPC_THR_CURVE = 0`，`z0.params:627`） | PX4 文档：居中杆 = `MPC_THR_HOVER`（`mc_att_control_main.cpp:99-109`）。现值 0.5 ⟹ 居中杆给 **50 % 推力**（≈ 2.25× 悬停） | 居中杆仍是悬停位，但**推力**从 0.5 掉到 0.1589 ⟹ 手动模式的**悬停点回到杆居中**（这正是本参数的本意，也改善"手动 ↔ 位置模式切换"的衔接）；代价是**上半程杆吃掉 `0.1589→1.0` 共 84 % 的推力范围** —— PX4 文档警告的正是这条（"if hover thrust is set to 20 %, 80 % of the upper thrust range is squeezed into the upper half"）。★ **建议保留 `MPC_THR_CURVE = 0`**（`= 1` 会让悬停落在 **8.6 %** 杆位，更难飞），但这是**手感变化**，见 §9 的 **O10** |
| c | **降落检测的两个阈值**（`MulticopterLandDetector.cpp:101-108`、`:216-219`、`:262`） | `thr_pct_hover = 0.6`、`hoverThrottle` = HTE 的 0.300 ⟹ `sys_low_throttle = 0.12 + 0.18×0.6 =` **0.228**；而稳态悬停推力是 **0.222** ⟹ **稳态悬停就落进"低油门"**（`ground_contact` 跟着为真。完整 `landed` 还要求"几乎无水平/垂直运动"，所以**不一定**误判，但余量很薄） | ⚠️ **`hoverThrottle` 与 `thr_pct_hover` 是同一支在切换**（`:101-108` 只在 `hte.valid` 时才把 `hoverThrottle` 换成 HTE；`:217` 的系数 `valid ? 0.6 : 0.3`）⟹ 有**三个**值：HTE **无效** `0.12+0.039×0.3 =` **0.132**；HTE **有效但未收敛**（估计值 ≈ 参数）`0.12+0.039×0.6 =` **0.143**；HTE **有效且收敛到真实 0.222** `0.181` ⟹ 三种情形稳态悬停**都不再**触发。"maybe-landed"那条从 `0.138` 降到 `0.130` / `0.124` |
| d | **`MPC_THR_MIN` 的余量** | `0.5` 距下限 `0.12`（`z0.params:630`）很远 | `0.1589` 只比它高 **32 %** ⟹ 低压 / 重载时更容易够到下限夹钳。⚠️ **只是提示，不建议改 `MPC_THR_MIN`** |

**⑦ 落地状态（诚实交代）**

- ✅ **模型侧已回填**（P1，2026-09-29；`params.py:62`）。
- ⬜ **飞控侧还没写** —— 在 **QGC** 的 Parameters 里（或 mavros 的 `param set`，§I.1 第 1–2 条就是这么写的）
  把 `MPC_THR_HOVER` 由 `0.5` 改成 `0.1589`，
  然后 **re-dump 对账**（§I.1 那套写参数的纪律同样适用）。这是**人的动作**，也是本票定为
  `ready-for-human` 的原因。
  ★ 值域检查：PX4 该参数自报 `@min 0.1 / @max 0.8`，启动时还会被夹进
  `[MPC_THR_MIN, MPC_THR_MAX] = [0.12, 1.0]`（`MulticopterPositionControl.cpp:238-241`）——
  **`0.1589` 两边都在域内** ✓。
- ⚠️ **两条待验**（不影响写参数，但别当成已验证）：① HTE 由位置控制器的输入驱动，
  而我们自己的离板飞行里位置控制器不参与（§3.3.4）—— **离板时 HTE 是否有效没实测过**，
  若无效，降落检测回落用参数本身 `0.1589`、系数取 0.3 ⟹ ⑥c 的 **`0.132`** 那一支；② ⑥b 那条手感变化**没在实机上飞过**。

### 3.4 控制流总览

```
 Nokov 动捕 ──VRPN──> vrpn_listener ──> mocap 节点 ──┬──> /mavros/vision_pose/pose ──> PX4 EKF2 (Phase 1)
  10.1.1.198:3883                                    │      （ENU，mavros 转 NED）
                                                     └──> /mocap/pose_enu ──> 模型预测控制节点   ← Phase 2 新增
                                                                                     │
                        ┌────────────────────────────────────────────────────────────┘
                        ▼
                   acados OCP (ENU/FLU, 10 状态)
                        │  u = [F, ωx, ωy, ωz]
                        ▼
           符号映射（权威：sign-map.md §2）⟹ roll / pitch / yaw / thrust_body[2]
                        │
                        ▼
              /mavros/setpoint_raw/attitude  (body_rate + thrust)
                        │
                        ▼
                     PX4 角速率环 (1 kHz) ──> 混控 ──> 电机
```

☠️ **2026-10-13**：上面那张图里那行**原写** `/fmu/in/vehicle_rates_setpoint  (uXRCE-DDS)  +  /fmu/in/offboard_control_mode`，**已就地改成 mavros 那条**。(C) 级下行 = `/mavros/setpoint_raw/attitude`（`body_rate` + 标量 `thrust`），独立心跳流**已删除**（见 [`../reference/px4-interface.md`](../reference/px4-interface.md) §4.3）。

**注意 Phase 2 是「加一条支路」，不是「替换」**：mavros2 那条链（`vision_pose` → EKF2 → `setpoint_raw/local`）
**保持原样不动**，随时可以切回 Phase 1 的 `(A)` 级。
★ **Q14=a 删掉 `/mocap/pose_ned` 不影响这个回退** —— 因为 `(A)` 级用的是
`/mavros/setpoint_raw/local` + EKF2 的 `/mavros/local_position/*`，**两者都不经过 `pose_ned`**。
（旧版这里写「`/mocap/pose_ned` 与 mavros2 那条链保持原样」，是把两条不相关的链并在一起说了。）

---

## 4. 分步实施清单

> **原则：每一步都有独立的、可证伪的验收判据。不通过不进下一步。**
> 步骤 S1–S5 全程在 SITL / 台架上，**不碰飞机**。

### S1 —— 环境：SITL + `px4_msgs`

> ★ **2026-09-28 已收口（[ticket 01](../../.scratch/phase2/issues/01-sitl-px4-msgs-baseline.md)）**。
> **构建步骤 / 版本与 commit / 五个坑 / 验收证据 / 给 Phase 4 的交接**全在
> [`px4-sitl-baseline.md`](px4-sitl-baseline.md)，脚本在 [`tools/px4-sitl/`](../../tools/px4-sitl/)。
> **本节只留结论**；下面三处的旧口径已被实测推翻，就地改正。

| 项 | 值 | 备注 |
|---|---|---|
| PX4 源码 | ★ **上游 `PX4/PX4-Autopilot` @ tag `v1.14.3`（`1dacb4c`）**（§2.6 执行版） | 已用 `~/refs/px4-v1.14.3-build/`（`--recursive --single-branch`）编通。**submodule 是 17 个，不是 23 个**（`grep -c '^\[submodule' .gitmodules`）。`~/refs/px4-v1.14.3` 是 `--filter=blob:none`，**不能直接构建**。✅ 机型已实测：`ROMFS/.../init.d-posix/airframes/` 共 **56 个条目**（55 airframe + CMakeLists）。☠️ **原写「fork `Peize-Liu/PX4-Autopilot @ 5ba2e91f`」—— 作废，飞控不跑 fork** |
| SITL 仿真后端 | ★ **SIH（`sihsim_quadx`，`SYS_AUTOSTART=10040`）** —— 2026-09-28 操作者裁定 | ☠️ **原写「gazebo-classic 或 gz-sim 都可用」—— 两条在本机都不成立**：gazebo-classic 的 dev 库 apt **无候选**（jammy/arm64）；gz-sim 只支持 Garden（`gz_bridge/CMakeLists.txt` 明写 `IGN (Fortress and earlier) no longer supported`），要加 OSRF 源 + ~500 MB + 写 SDF。SIH 零外部依赖。**代价已知情并接受**：SIH 是与我们控制器同层次的简化模型，**证明不了"模型够真"**，只能证明约定对得上 —— 详见交付文档 §3 |
| `px4_msgs` | ★ **钉 `release/1.14` @ `ffb6e80e`**（✅ **用户签字**） | ✅ 与真身 `4a0e65f2` 在**共有的 184 个 `msg/` 文件上 blob 全同**（唯一集合差异 `msg/CMakeLists.txt`，那是 PX4 的构建文件）；并已补证 **真身 ↔ 上游 v1.14.3 在 185 个文件上 blob 亦全同、无集合差异** ⟹ 拿上游当被控对象对兼容性无差别。☠️ **原写「从 fork 的 `msg/` + `srv/` 生成（Q31=a）」与「`release/1.14` 已被判为不兼容（28/184 字段不同）」—— 两条都作废**（那是拿 fork 比出来的）。详见 §6.1 与 [`refs-manifest.md`](../reference/refs-manifest.md) §2.5(d)/§2.6 执行版 |
| 中间件代理 | ★ **`MicroXRCEAgent udp4 -p 8888`**（钉 `v2.4.2`） | ☠️ **原写「`micro-xrce-dds-agent`」—— 二进制名没有小写别名**，写不对就是「命令找不到」。PX4 SITL 默认端口 8888 |

**⚠️ 必须做的版本校验**（roadmap 与 [`open-source-survey.md`](../reference/open-source-survey.md) §10 都强调过）：

```
PX4 侧：  python3 tools/px4-sitl/check_msg_compat.py \
              --px4-tree ~/refs/px4-v1.14.3-build --px4-msgs ~/refs/px4_msgs \
              --interface-lib ~/refs/px4-ros2-interface-lib
ROS 侧：  ros2 interface show px4_msgs/msg/VehicleRatesSetpoint
```

☠️ **原写「`python3 Tools/check-message-compatibility.py`」—— 这个路径是错的，而且照着改也还是会错**：
该脚本**不在 PX4-Autopilot 里**（v1.14.3 / v1.15.0 / v1.16.0 / fork / `px4_msgs` 逐棵树查过，全无），
它住在 **`auterion/px4-ros2-interface-lib`** 的 `scripts/` 下；**直接跑它对本项目会报假 FAILED**
（它按自己仓库 main 的类型清单查，而那 20 个类型 v1.14 里根本不存在 —— 报错原文里**没有** `Some files differ` 那一段，
即**零个字段不符**）。本项目改用 `check_msg_compat.py`：**复用官方那份 `message_hash` 算法**，
但把类型换成两侧 `msg/` 的**并集（184 个全覆盖）**，**严格强于官方脚本的子集检查**。
⚠️ `PROJECT.md:450` 与 [`px4-interface.md`](../reference/px4-interface.md):178、`:284` 仍写着那个错路径 ——
**本张不动别的票**（开票点名即可，`done` 的纪律不允许靠"留给以后"掩盖），已另开
[ticket 14](../../.scratch/phase2/issues/14-stale-check-msg-compat-path.md)。

**判据**：`px4_msgs` 与固件的消息定义一致。**v1.15 及以前版本不匹配会静默失败** —— 话题能订阅、能发布，但 PX4 收不到，且**不报任何错**。（px4-mpc 里那段同时订阅 `vehicle_status` 和 `vehicle_status_v1` 的代码，就是在绕这个版本问题。我们锁定版本后可以删掉。）

**验收**（2026-09-28 全过，证据见交付文档 §2.4）：
① `check_msg_compat.py` **PASS** —— 共有的 **184/184** 字段布局全同，`OffboardControlMode` 与 `VehicleRatesSetpoint` 均在共有集内；
② `ros2 topic list | grep fmu` 看到 **`/fmu/in/*` 13 条 + `/fmu/out/*` 11 条**（双向前缀）；
③ 类型能解析成 `[px4_msgs/msg/…]`，且 `vehicle_status` **有真实数据在流**（显式 `BEST_EFFORT`，即 §6.4 那个坑的现场）。
⚠️ **但本步到此为止**：以上都**不是「通了」的判据** —— 那正是版本不匹配骗人的地方。真判据在 S5 / [ticket 05](../../.scratch/phase2/issues/05-sitl-offboard-smoke.md)：**PX4 是不是真的照做了**。

### S2 —— acados 在 Jetson Orin NX 上编译

见 roadmap §5 的完整要点。**三条动作不变，但 2026-09-28 在本机逐条实测后，三条的理由都改了口径**
（实测数据、报错原文、复现命令：[`acados-orin-build.md`](acados-orin-build.md)）：

1. **`ACADOS_WITH_OPENMP=OFF`** —— ☠️ 旧话「Orin 上 OpenMP 与 acados 有已知冲突」**未能复现**。
   开 OMP 也编译通过、`solve()` 返回 0；绑单核时两者几乎无差。**真实的理由**：不绑核时 OMP 线程跨簇调度
   把**尾部抖动**放大（max 4.39 → 6.72 ms，med 反而更低）。⭐ 实时环看尾部，所以仍取 OFF
   （另：0.6.0 的默认值本来就是 OFF，显式写只是防上游改默认）。
2. **BLASFEO target = `ARMV8A_ARM_CORTEX_A76`** —— Orin NX 是 Cortex-**A78AE**，A76 确实是可用集中最接近的
   （上游在 aarch64 上的默认是 **A57**，`CMakeLists.txt:62-66`；`X64_AUTOMATIC` 在 aarch64 上压根不是可选值）。
   ☠️ 但「用 `GENERIC` 会掉一个量级」**不对**：实测只慢 **~21%**（median 2.83 → 3.43 ms）。
   ⭐ 同一批实测里**更值钱的一条是绑核**：median 3.96 → 2.83 ms（**−29%**）、max 4.39 → 2.96 ms（**−33%**），
   比换 BLASFEO 目标影响大得多 —— 频率定档（D6）必须写明这个前提（⚠️ 百分比要连着 median/max 一起说）。
3. **`t_renderer`（Tera）必须装** —— ☠️ 旧话「缺了报 `EOFError`」**在我们用的 v0.6.0 上不成立**：
   那是 v0.5.x 及以前**交互式提问**（`input(...)`）在非交互下读到的 Python 内建异常，
   commit `a26c7d0ce`（首发于 **v0.6.0**）已删掉该提问。**真正要记的是三种落点，其中一种完全静默**：
   缺失+有网 → 自己下载（只是慢）；缺失+无网 → `RuntimeError`，信息完整**直指根因**；
   **在但渲染不出东西 → 不报错**，只因 `utils.py:396-397` 用 `os.system` 且**只查退出码、不校验产物**，
   一路静默到两层之后才炸成 `OSError: …libacados_ocp_solver_….so: cannot open shared object file`。

**验收**：编译 `px4-mpc` 自带的例子（`mass=1, hover_thrust=0.73` 的 x500 配置）能 `AcadosOcpSolver` 创建成功并 `solve()` 返回 0。
**✅ 已通过（2026-09-28）**，最坏纯求解 **4.39 ms**（不绑核）/ **2.96 ms**（绑核 4）——
100 Hz 的 10 ms 预算够。⚠️ 但**整条 `controller.solve(x0)` 是 10.70 ms**：参考实现每周期**重建 2 次 CasADi 模型**
（`multirotor_rate_mpc.py:152-153`），移植时必须把它提出循环，否则 100 Hz 无从谈起（交 ticket 04/06）。

### S3 —— OCP 建模 + **离线**验证（不接 ROS）

> ✅ **2026-09-28 收口（ticket 04）** —— 交付物：`src/nmpc/nmpc/{ocp,model,params,timing}.py` +
> `src/nmpc/test/test_ocp_*.py`（**零 ROS**）。交付文档：[`offline-ocp.md`](offline-ocp.md)
> （含 M1–M4/M6 的落地、三条 RTI 读数的实测、最坏耗时与 D6 的判定）。
> 下表是当时的**计划**；表内两处已被实测修正，改在**行内**（不删原文），别照旧值引用。

先用脚本把 OCP 单独跑起来，喂**构造的**状态，看输出合不合理。这一步把「模型/代价/约束」的 bug 和「ROS/通信」的 bug 隔离开。

**要做的离线测试**（每个都是几行代码）：

| 测试 | 输入 | 期望输出 |
|---|---|---|
| 悬停配平 | 状态 = 原点，`v=0`，`q=identity`，目标 = 原点 | `F ≈ T_hover = 7.36 N`，`ω ≈ 0`。★ 实测**取决于 `u_ref`**：给悬停前馈 ⟹ 精确 `7.3575`；给 `u_ref = 0` ⟹ 稳态停在 **7.8111（+6.16 %）**，那是 RTI 单步迭代的**不动点偏置**，不是冷启动（[`offline-ocp.md`](offline-ocp.md) §4.1） |
| 差一个重力 | 同上，但目标在 `+z 0.5 m` | `F > T_hover`（实测 `9.5362`），`ω ≈ 0` |
| 姿态响应符号 | 同上，但目标在 `+x 0.5 m` | 产生**朝 +x 倾斜**的角速率 —— ⚠️ 方向**不要猜**：推导与结论见 [`sign-map.md`](sign-map.md) §3。实测 `ω_y = +0.3239`（`u_ref = 0`）。⚠️ **缺省 `u_ref`（上一帧解）时它不收敛**：0.3225 → … → 第 11 次 **1.3487** rad/s，一路上爬 —— ⟹ 这条测试**必须显式给 `u_ref`**，且缺省路径归 ticket 06/07 在 SITL 里验（[`offline-ocp.md`](offline-ocp.md) §4.4） |
| 约束生效 | 目标在 `+x 10 m` | `ω` 打满 `ω_max`（实测一拍即贴 3.84 并保持），**不越界**。原写「`F` 打满 `T_max`」← **实测：不会**。纯水平目标只需倾斜、垂直分量仍≈悬停，实测 `F = 11.4721`（上限 `f_max` = 11.8）；要顶到上限得**同时要求爬升**（`[10,0,10]` ⟹ 精确 `11.8`）。见 [`offline-ocp.md`](offline-ocp.md) §4.2 |
| 求解耗时 | 重复 1000 次 | 报**最坏**耗时（不是平均）。★ 实测：**必须分两列**（acados 自报 vs 整周期墙钟）—— 合成一个数会得出错误结论（§5、[`offline-ocp.md`](offline-ocp.md) §5） |

> ⚠️ **第 3 行的「姿态响应符号」是整个项目最容易搞错的一步。** 在 FLU/ENU 下，要往 +x（东）飞，需要绕 **−y** 轴倾斜……还是 +y？**不要靠记忆。**
> 答案、完整推导、以及"参考实现是这么写的"三方对照，现在都在
> [`sign-map.md`](sign-map.md) §3/§4 —— 而且**有可执行复核**（`tools/sign-map/`）。
> 那里给的是一组**推导出来**的数，照抄之前先读推导。

### S4 —— MPC 节点骨架

按 §2.6 复制 `px4-mpc`，按 §2.5 的 M1–M9 改造。**必须同时具备**：

> ✅ **落地状态（2026-09-28，ticket 06）** —— 逐条勾选 + 依据。**逐条的读法、第一手数字与
> 交接项在** [`mpc-node-seam1.md`](mpc-node-seam1.md)（节点与 seam 1 的交付文档），这里只留索引。
>
> - [x] 订阅 `/mocap/pose_enu`（**不是** NED）—— `mpc_node.py:284`；`test_接口与_QoS_与票面一致`
> - [x] 订阅 `/mocap/velocity_enu`（`geometry_msgs/TwistStamped`，**ENU**，与 pose 同一个 frame）——
>       ★ **ticket 15（2026-09-29）**：`PoseStamped` **没有** `twist` 字段 ⟹ 10 状态模型的 `v`
>       必须走这条**独立话题**（`mocap` 节点把 `/vrpn/<刚体>/twist` **原样** republish，
>       同 frame ⟹ 零变换）⟹ `x + v·τ` 在 B 档上不再恒等变换。
>       落点：`mocap_rel_pose_node.cpp` 的 `on_twist` / `mpc_node.py` 的 `TOPIC_MOCAP_VEL`；
>       用例 `test_node_core.py::test_动捕速度从独立话题进外推`、
>       `test_mpc_node.py::test_B_档_速度从_velocity_enu_进外推_表现为推力低于悬停`
> - [x] 订阅 `/fmu/out/vehicle_status` —— `mpc_node.py:286`；同一条用例里的 `matched_subscribers("status") == 1`
>       （⚠️ 这两行的行号**2026-09-28 被 ticket 07 的改动下移了 ~40 行**：07 在 `NodeParams` / `__init__` / `_tick` 里都加了东西，订阅那几行的位置跟着走）
> - [x] 发布 `/fmu/in/offboard_control_mode`（`body_rate = True`，其余 False），**≥ 2 Hz，实做 50 Hz** —— `test_闸门关闭时只发心跳_一帧_setpoint_都没有`（≥2 Hz）+ `test_实做频率_心跳_50_Hz_setpoint_100_Hz`（实测 50.0 Hz）
> - [x] 发布 `/fmu/in/vehicle_rates_setpoint` —— 同两条用例；`test_解锁切_OFFBOARD_之后立刻开始发`
> - [x] QoS：pub `BEST_EFFORT + TRANSIENT_LOCAL + depth 1`，**sub 也必须 `BEST_EFFORT`** —— `test_接口与_QoS_与票面一致`（⚠️ `depth` 黑盒读不回来，守法见 seam 1 文档 §3）
> - [x] `timestamp` 用微秒 —— `test_时间戳是墙钟微秒`（夹具按"相对墙钟的比值"把 µs/ms/ns 分开）
> - [x] 求解失败 / 非有限值的兜底路径（M5）—— ✅ **2026-09-28 由 ticket 07 补齐**：三档兜底阶梯
>       （`hold` 10 拍 → 安全悬停）、发布前全量校验、连续失败的可检索上报、两条输入合理性判据。
>       设计与第一手依据全在 [`mpc-node-fallback.md`](mpc-node-fallback.md)；M5 的编号落点见 §2.5 的 07 那段。
>       ⚠️ **票面拒掉的那一类不动**："退化求解照样发"仍是现行行为（分界线 = `SolveInfo.ok`）
> - [x] 100 Hz 定时器（D6 已定 100 Hz）—— 实测落在 100.0 Hz；**降档决定归 ticket 09/11**
> - [x] RViz：预测轨迹 `Path` + 参考 `Marker` —— `test_RViz_的预测轨迹与参考点`（51 点、起点≈当前位置、终点朝目标、参考点落位）
> - [x] 另外三项票面要求：**饱和表现为打满**（`test_目标在正前方时_pitch_为负_且顶约束表现为打满`）、
>       **软急停接口参数化**（零通道号，见 seam 1 文档 §6）、**装桨前的静态自检**
>       （`nmpc/preflight_check.py`，见 seam 1 文档 §7）

### S5 —— SITL 闭环 ★ **Phase 2 的真正关卡**

> ⚠️ **被控对象不是 `gz_x500`（Q9 决策）**：`gz_x500` 只是"能跑起来"的现成平台，
> **SITL 要重新生成一份符合 z0 的模型** —— 按 Q11 用**故意失配**的参数（质量 / `kt` / 转动惯量 / τ 按单参数 ±30% 扫），
> 并保留一份**同参数变体**作为对照；按 Q13 用 **Phase 1 的真实动捕 rosbag 回放**驱动，
> 地面真值重映射那份作为对照。
>
> 🚫 **`gz_x500` 仅可作为最早期冒烟目标 —— 不得用于 SITL 验收。** 理由不是"精度不够"
> 而是**量级差得远**。上游那些机型的实数（第一手：`simulator_sih/sih_params.c` 的
> `PARAM_DEFINE_FLOAT` 默认值 + `init.d-posix/airframes/10040_sihsim_quadx`）：
> `SIH_MASS 1.0` / `SIH_L_ROLL 0.2` / `SIH_T_MAX 5.0`（单电机）/ `SIH_IXX 0.025`，
> 而机型里 `CA_ROTORn_PX/PY = ±0.15`（**与 SIH 的 0.2 自相矛盾**）。
> 对 z0 的 0.75 / 0.0537 / **11.5757** / 0.002166：**力臂 2.8 倍、`IXX` 11.5 倍**
> （★ 第三个值 2026-10-09 基线已换：`SIH_T_MAX` 由按电池的 `2.95` 改成按电机的 `11.5757`，
> ticket 21 / phase3 的 `04`；那两条比值不涉及 `T_MAX`，故不变）。
> "在 x500 上能飞"因此**不蕴含**"在 z0 上能飞"，拿它做 SITL 验收等于把本步的判据换成另一台飞机的。
> 验收用的被控对象是 `sihsim_z0`（ticket 08 交付），参数、切换与复原见
> [`sitl-z0-plant.md`](sitl-z0-plant.md)。

> ★ **2026-09-28：本步的"被控对象"这一半已单独收口（[ticket 08](../../.scratch/phase2/issues/08-sitl-z0-airframe.md)）**。
> z0 同构机 `sihsim_z0`（`SYS_AUTOSTART=10046`）已能起，
> 参数、±30 % 失配开关、复原方式、验收脚本全在 [`sitl-z0-plant.md`](sitl-z0-plant.md)。
> **S5 本身（闭环）当时仍未开始** —— 收口的是"拿什么当被控对象"，不是"闭环跑通了"。
> ★ **2026-09-29 起已跑**（[ticket 11](../../.scratch/phase2/issues/11-sitl-tier-b-seam3.md) `done`）：五条判据的结果见本步末尾的表（其中第 1 条垂直误差、第 4 条失配兜底**未达标**，别读成"全绿"）。
> 一句话记值：`SIH_MASS 0.75` / `SIH_T_MAX 11.5757` / 力臂 `0.0537401` / `MPC_TILTMAX_AIR 45`（**与飞机一致** —— 2026-09-28 裁定「两边都回 45」，原打算在 SITL 侧收成 30° 的那版**已作废**）。
> ☠️ **`SIH_T_MAX` 这格 2026-10-09 已换**：按电池的 `2.95` → **按电机的 `11.5757`**
> （ticket 21 / phase3 的 [`04`](../../.scratch/phase3/issues/04-rebuild-sitl-plant-to-measured-baseline.md)）；
> 本段其余结论（"验收用 `sihsim_z0`"、"五个旋钮"）不受影响。
> 失配开关是**五个**旋钮（质量 / 推力系数 / 转动惯量 / 作动器滞后 `tau` / 内环闭环带宽 `rate`），其中 `rate` 动的是**控制器**、其余四个动**被控对象** —— 与上一段 Q11 记的四个不完全是一回事，见 `sitl-z0-plant.md` §5.4。

在（z0 参数的）SITL 被控对象上跑通完整链路。**SITL 里 PX4 状态话题可用，所以这一步有两个可选的输入源**：

| 输入源 | 用途 |
|---|---|
| `/fmu/out/vehicle_local_position` + `vehicle_attitude` | **先跑通 A 档**：验证 MPC→PX4 这条链、符号、推力标定 |
| 模拟的 `/mocap/pose_enu`（可以用 SITL 真值 + 人为延迟/噪声伪造） | **再跑通 B 档**：验证动捕输入这条链 |

> **★ 为什么分 A/B 两档：** 如果一上来就用伪造的动捕，符号错了你分不清是「MPC 的模型错了」还是「ENU/NED 转换错了」。先 A 后 B，**每次只有一个新变量**（这是本项目的指导思想，README 里也写了）。

**S5 验收判据**（缺一不可）：

1. 起飞到 1 m 悬停，**稳态位置误差 < 0.15 m**，无持续振荡。
2. 走完 Phase 1 的三条轨迹（悬停 / 1 m 正方形 / r=1 m 八字），**不撞地、不失控**。
   ★ 2026-10-08：轨迹 3 的 8 字由 0.5 m 放大到 **1.0 m**（原本的 2 倍，操作者裁定，见
   [ticket 27](../../.scratch/phase2/issues/27-lemniscate-doubled-and-downstream-labels.md)）。
   ⚠️ **下面 2026-09-29 那轮 B 档判的不是这条**：SITL 的轨迹档**重放的是包里那条 0.5 m 的
   `/traj/reference`**（`bag/4_traj3_lemniscate` 是数据、不随参数改；那份 0.5 m 的包
   2026-10-08 已搬进 `bag/old/4_traj3_lemniscate/`）⟹ 放大后要拿 1.0 m 的证据，
   得**重录一次八字场**或让夹具按新参数重新生成参考。
   ★ **2026-10-09 订正：那两条去向都已经落地** —— 1.0 m 的八字场 2026-10-08 录成
   `bag/5_traj3_lemniscate`、回放模型 `mocap-model-5.json` 已入库，SITL 的 `T0` 那一场
   （判据②·轨迹2）**已接上这两份**（[ticket 05](../../.scratch/phase3/issues/05-repoint-t0-to-lemniscate-1m-and-model5.md)）
   ⟹ "重放的是 0.5 m 那条"**只对 2026-09-29 那一轮成立**，不再是当前接线。
3. 人为注入 50 ms 延迟后**仍稳定**（可以变差，但不能发散）。
4. 人为让 `solve()` 返回非零，**飞机保持安全悬停而不是崩**（验证 M5 兜底）。
5. **实测最坏求解耗时** ← **这个数字直接决定 D6。**

> ### ★★ 2026-09-29：B 档跑完（[ticket 11](../../.scratch/phase2/issues/11-sitl-tier-b-seam3.md)）—— **四条判据两条过、两条没过**
>
> 判在 **SITL 真值帧**（操作者裁定），被控对象是 z0 同构机、**参数按票面要求刻意失配 ±30 %**。
> 五场矩阵、全部读数与条件见 [`tier-b-sitl-seam3.md`](tier-b-sitl-seam3.md)。
>
> | S5 判据 | 结果 | 一手数字 |
> |---|---|---|
> | 1. 1 m 悬停稳态误差 < 0.15 m | ❌ **未达标** | 水平 **0.0112 m** ✅ / 垂直 **−0.6406 m** ❌（界的 4.3 倍）；「无振荡」单独 ✅ |
> | 2. 三条轨迹 | ⛔ **没跑**（记未达标） | 正方形那条**没有包**；轨迹跟踪能力归 [17](../../.scratch/phase2/issues/17-traj-reference-interface.md) |
> | 3. 注入 50 ms 延迟 | ✅ **通过** | 总延迟 55.34 ms 下末段垂直速度 +0.024 m/s、离地段最低点 +0.154 m |
> | 4. 求解失败后安全悬停 | ❌ **未达标** | 阶梯 ✅；兜底**标称口径 ✅**（+0.038 m）、**±30 % 失配下 ❌**（沉到地面） |
> | 5. 最坏求解耗时 | ✅ **已测** | 空机稳态最坏 **5.652 ms** ⟹ 100 Hz 余量 1.77× |
>
> **两条缺口的性质**：第 1 条是**模型与被控对象的垂直口径差**（模型 `m = 0.75` vs 被控对象 `0.975 kg`，
> 悬停前馈少 2.207 N，而垂直通道没有积分作用 ⟹ 稳态停在成比例的高度偏差上）；
> 第 4 条的兜底**以「模型里的 `hover_thrust` 是对的」为前提**，失配恰好打破它 ——
> **兜底能对付"求解器抽风"，对付不了"模型与被控对象的口径差"。**
> ⚠️ **上面这四条判据用的是 SITL 机型，不是 z0 真机**；本步"过了"不等于真机能飞。
> ⛔ ~~状态停在 `ready-for-human`：**这是 Phase 2 的最高关卡、上面没有兜底**，未达标之后怎么走要人定。~~
> ☠️ **2026-10-09 订正**：那句说的状态**已经变了** —— 操作者当天裁定把 Phase 2 那 9 张票**全部收 `done`**
> （见 [`.scratch/phase3/spec.md`](../../.scratch/phase3/spec.md) 的 Comments #3）。
> ★ 但"最高关卡、上面没有兜底"这个**性质没变**：`11` 的 `done` 不等于"四判据全绿" ——
> ④-B 的 `mass+30` 那一档**真 ❌**（兜底帧只给到所需的 0.769 倍），已作为**读数**留在票面。
> ⟹ 读这一步时，**别把 `done` 读成"这里安全了"**；未达标之后怎么走**仍然要人定**。

### S6 —— 实机步骤 1（10 状态模型）

**前置：Phase 1 已完全达标**，且 §S5 五项全过。

**台架预检**（不装机、不通电桨）：

- [x] `T_hover` 标定：从 Phase 1 的悬停日志里取 PX4 实际输出的归一化推力（`/fmu/out/vehicle_thrust_setpoint` 或 mavros 侧等效量），反推模型里的 `hover_thrust`。**这是 M1 的实测输入。** ✅ **2026-09-29 完成**（ticket 10）：`hover_thrust = 0.1589`（满电口径）⟹ `T_max = 46.3 N`，已回填三处代码；口径与推导见 §3.3.3。
- [x] `ω_max` 与 `z0.params` 的 `MC_ROLLRATE_MAX` / `MC_PITCHRATE_MAX` / `MC_YAWRATE_MAX` 逐项对齐（M3）。✅ 3.84 / 3.84 / 6.28。
- [ ] 推重比复核：地面拉力台或 Phase 1 悬停时的油门裕度 ⚠️ **改口径了** —— 原来的待验证值 `T_max ≈ 11.8 N` 现在叫 **`f_max`**（`F` 上限，2026-09-29 解耦后它就是这个角色，取 §6.2 的 Mooch 15 A 外推值）；而**归一化**上限是另一个数（实测 46.3 N，§3.3.3）。**要测的还是那一件事**：电池**可持续**能供出多少推力/功率（VTC6 6S1P 的 15 A 瓶颈，§6.2）—— 它决定 `f_max` 取 11.8 N 保不保守。

**首飞阶梯**（每级都要停下来看日志）：

| 级 | 做什么 | 过不了怎么办 |
|---|---|---|
| 1 | **桨叶拆掉**，地面看电机响应：给 ±小角速率，目视电机转速差方向对不对 | 方向反了 → 就是 §2.3 的符号表错了 |
| 2 | **系绳 + 离地 20 cm**，只给悬停推力，看能否站住 | 缓慢上飘/下坠 → `T_hover` 标定不准 |
| 3 | 系绳，闭环悬停 30 s | 振荡 → 降 `Q/R` 比例或降频 |
| 4 | 自由飞，悬停 → 正方形 → 八字 | 按 Phase 1 同样的判据 |

### S7 —— 实机步骤 2（13 状态 + `k_ω`）

**仅在步骤 1 稳定后做。** 内容：

1. **辨识 `k_ω`**：给角速率阶跃，从 IMU 日志拟合内环一阶时间常数 `1/k_ω`（台架或低空，桨叶装）。
2. 模型加 `ω` 三状态 + `ω̇ = k_ω(ω_des − ω)`，`nx = 13`。
3. **重新调 `Q`/`R`** —— 加状态后原权重不再适用。
4. SITL 重跑 → 实机阶梯重跑。

> 📌 **这一步是本项目的论文价值点之一**：同一套 MPC，"内环建模 vs 不建模"的对比实验。

### S8 —— 安全与回退（**必须和 S4 同时做，不是最后做**）

| 机制 | 实现 |
|---|---|
| **遥控器夺权** | RC 任一杆动 → PX4 自动退出 offboard。**这是最后一道防线，飞前必须实测有效** |
| **失效保护** | offboard 心跳丢失 → PX4 触发 `COM_OBL_RC_ACT` / failsafe。**确认参数已配且测得** |
| **软急停** | 遥控器按键（用户 Q22 第 8 条）→ 节点停发指令。★ **2026-09-21：Phase 1 已定案 —— 没有"软急停通道"这回事**（急停 = **ch8 高值 = PX4 硬件杀车**，程序不读急停通道）⟹ "节点停发"只能走**两条既有路径**：打杆接管（`COM_RC_OVERRIDE=3` → PX4 退 offboard）或 ch6 模式开关。☠️ 原先那句「只剩 ch7 空闲」**已作废**（未映射通道有 9 个）。完整结论见 [`../reference/params-changes.md`](../reference/params-changes.md) **§H.7** |
| **求解器兜底** | ✅ 连续 N 次求解失败 → 切悬停推力 + 零角速率，并置标志。**2026-09-28（ticket 07）**：N = `fallback_hold_frames`（默认 10 拍 = 100 ms）；N 拍之内是 `hold`（保持上一帧可行指令、时间戳换新），超过才切 `hover`。依据与第一手见 [`mpc-node-fallback.md`](mpc-node-fallback.md) §2 |
| **数值守卫** | ✅ 发布前 `np.isfinite()` 全量校验（坑 3）。**2026-09-28（ticket 07）**：`node_core.frame_is_finite` 只看写进那一帧的四个数；`u` 的非有限由 `solution_usable` 在映射前拦一次；任一项非有限 ⟹ **整帧丢弃**走兜底 |
| **输入合理性**（ticket 07 新增） | 超时判据拦的是"没有数据"，拦不住"数据是错的"：`stamp_regress`（源自己的 stamp 往回走，容差 0.01 s）/ `pose_jump` / `attitude_jump` ⟹ **只关闸门、一帧不发**。见 [`mpc-node-fallback.md`](mpc-node-fallback.md) §4 |
| **回退到 Phase 1** | **两个节点都在跑，只是不发布** —— 一个 `ros2 param set` 就能切回 `(A)` 级 |

---

## 5. 待实测参数清单（★ 卡住进度的硬依赖）

> 这些**必须在实机上测**，任何文档都给不出。按依赖顺序排：

| # | 参数 | 怎么测 | 卡住谁 | 有没有替代 |
|---|---|---|---|---|
| P1 | **`hover_thrust`**（模型里） | Phase 1 悬停日志反推 —— ✅ **2026-09-29 已实测**（ticket 10）：**`0.1589`**（满电 25.2 V 口径，由 20.474 V 上的 `0.221992` 与 20.526 V 上的 `0.220884` 按实测 `V^1.608` 归一，互差 0.09 %）。工具 [`tools/hover-thrust`](../../tools/hover-thrust/README.md)；推导与三条交叉验证见 ticket 10 的 Comments 第二轮 | S6 全部 | ✅ **已回填**（`params.py` / `mpc_node.py` ×2）。★ 占位 `0.6235` 比它大 **3.9 倍** —— 这就是"测不准飞不起来"的实例 |
| P2 | **`T_max`**（归一化侧） | ⚠️ **不必单独测**：它由 P1 与 §3.3.2 的自洽条件 `T_max = m·g/hover_thrust` 唯一确定（`t_max_for()`）⟹ 实测值 = **46.3 N** | M4、§3.3 | ✅ 已随 P1 定稿。★ **M4 要的"按电池"是另一件事**（OCP 的 `F` 上限 ⟹ `f_max`），已于 2026-09-29 **解耦**并取 11.8 N，见 §3.3.3 |
| P3 | **最坏求解耗时** | ✅ **已实测（离线）** 1000 次取 max，见下 | **D6（频率）** | ✅ 已回填，D6 定 **100 Hz** |
| P4 | **`k_ω`** | 角速率阶跃 + IMU 拟合 | S7 | 步骤 1 完全不需要 |
| P5 | **动捕实测延迟 τ** | ✅ **已在 z0 上实测 ≈ 50 ms**（2026-09-28，飞控 ulog × 交叉相关） | D9 | 出处与方法：[`../reference/mocap-setup.md`](../reference/mocap-setup.md) **§6.3**。⚠️ 旧机那个"50 ms"只有一句结论、无方法，**对 z0 只是参照** |
| P6 | 遥控器**开始键**（ch9）的实读行程 | `/mavros/rc/in` 看 `channels[8]` | S8 | **不影响飞行，但影响安全**。★ **2026-09-21：通道分配已定 = ch9**（[ticket 02](../../.scratch/phase1/issues/02-rc-channel-decision.md)）—— 本行只剩"把 ch9 的实读行程读出来"，用来校准 `start_threshold`。⚠️ **前置**：先在 EdgeTX 里给 CH9 配一个开关，否则读出来永远是常数 |
| P7 | **`MPC_THR_HOVER`**（**飞控侧**，P1 是同一个量的另一处） | ⚠️ **不用测** —— 它**就是 P1**（`0.1589`）；要做的只是把它**写进飞控**。两个值各是什么、从哪来、为什么必须相等：**§3.3.5**（改动 `phase4/02-C1`） | 起飞前（手动/位置模式的悬停前馈；也定 HTE 的窗口与降落检测阈值） | ✅ **2026-10-13 已写入并读回**（mavros `param set`；`force_pull` 后读回 `0.15889999270439148`，见 [phase4/02](../../.scratch/phase4/issues/02-hover-thrust-backfill.md)）。★ 与 P1 **必须同值**：只改一处 = 静默的推力增益错误 |

> ★ **Q19（2026-09-20）→ ✅ 2026-09-21 resolved**（ticket [02](../../.scratch/phase1/issues/02-rc-channel-decision.md)）：
> **开始 = ch9 上升沿；急停 = ch8 高值（硬件杀车，不新增软急停通道）；解锁 = 程序解锁（触发 = ch9 第 1 次上升沿）。**

> ★ **P3 回填（2026-09-28，ticket 04）** —— 原文写的是"SITL 跑 1000 次"，实际是在**离线 OCP**
> 上量的（seam 2，无 ROS / 无 SITL / 无话题收发）。绑核 `--cpu 4`、机器空载，两种口径都记：
>
> | 口径 | median | p99 | max |
> |---|---|---|---|
> | acados 自报 `time_tot`（**求解器**） | 2.682 | 3.423 | **3.447 ms** |
> | 整周期墙钟（**能不能闭环**） | 3.945 | 4.675 | 56.143 ms ⚠️ |
>
> ⚠️ 墙钟那个 max **不是求解开销**：同一拍的 acados 自报只有 3.4 ms，差的 52.7 ms 是这段 Python
> 被抢占（1000 次里 1 次）。三次复测的自报 max 分别是 3.447 / 3.584 / 3.447 ms。
> **⟹ D6 定 100 Hz**（纯求解余量 2.8×）；**遗留条件**：节点侧要处理那个尾（绑核/优先级，归 06/07）。
> ★ **当时仍未测**：SITL 闭环下的整周期（含订阅/发布/状态转换/日志）—— 归 seam 3（ticket 11）复核（★ 2026-09-29 已复核，见下）。
> ★★ **2026-09-29 已复核（ticket 11）**：B 档（真动捕回放驱动、100 Hz、空机）五场矩阵，
> **逐拍求解耗时稳态最坏 5.652 ms**（去 `warmup` 后的逐拍最大值；冷启动 12.9–16.6 ms **五场都落在 `warmup` 段内**
> ⟹ 不计入"最坏"）。⟹ **100 Hz 复现**（对 10 ms 预算余量 1.77×），逐拍间隔 p50 10.0 ms、
> **实发 99.5–99.6 Hz**。⚠️ 与 A 档同一条前提：**空机**（机上多几个进程能把它抬到 11 ms）。
> 逐场表与口径见 [`tier-b-sitl-seam3.md`](tier-b-sitl-seam3.md) §9。
> 方法与原始输出：[`offline-ocp.md`](offline-ocp.md) §5；命令 `python3 -m nmpc.timing --runs 1000 --cpu 4`。
> 结论、依据与连带要求见 [`../reference/params-changes.md`](../reference/params-changes.md) **§H**；
> 约束与完整占用表见 [`../reference/params-analysis.md`](../reference/params-analysis.md) §3.15（其中「只剩 ch7」已更正）。

> ★ **P1/P2 回填（2026-09-28 起，[ticket 10](../../.scratch/phase2/issues/10-param-measure-backfill.md)）**
> —— 这两条**绑在一起**，不能各测各的（§3.3.2）。做法：
>
> 1. 取一段**悬停**日志（只读取日志，不涉及解锁/起飞）；
> 2. `python3 tools/hover-thrust/hover_thrust_from_ulog.py <悬停日志.ulg> --model-hover 0.1589`
>    ⟹ 直接给出 `hover_thrust`，并当场算出它对应的 `T_max`；
> 3. 若两者与模型当前值差得远，**改的是参数，不是判据** ——
>    工具只回答"日志里的悬停推力是多少"（本次就是这样：占位 `0.6235` 与实测 `0.22199` 差 64 %）。
>
> ✅ **2026-09-29 已回填**（四段真机日志，两段可用）：`hover_thrust = 0.1589`、`T_max = 46.3 N`。
> ⚠️ **O8 的口径（满电 vs 低压）已由操作者裁定为「满电」** —— 但注意实测数据**全部来自低电段**
> （起飞时 SOC 只有 46–60 %），满电值是**外推**：由 20.474 V 上的 `0.221992`（另一段 20.526 V / `0.220884`）
> 按**实测律** `thrust ∝ V^1.608` 归一到 25.2 V ⟹ `0.15896` / `0.15882`（互差 0.09 %）。
> ★ 若要把这条外推换成直接读数，只需**在满电包上再悬停 ~30 s**（PX4 自己的推荐做法）。
> 完整推导、三条独立交叉验证与电压律的来处见 ticket 10 的 Comments 第二轮与 §3.3.3。

**✅ P1/P2 已定稿**（2026-09-29）。回填**带出的那个结构问题也已解决**：`T_max` 原来同时是
归一化常数（实测 46.3 N）与 OCP 的 `F` 上限（M4/D5 要求"按电池"）—— **同日裁定解耦**，
`F` 上限改由 `Z0Params.f_max = 11.8 N` 承担，归一化侧只留 `t_max`；`preflight_check` **24/24 全绿**
（那条守卫改成看 `f_max`，并新增"`f_max` 夹在悬停与满量程之间"）。见 §3.3.3。
其余待测项都有可用初值。

★ **P7 是 P1 的"另一处"**（2026-10-09 加，[phase4/02](../../.scratch/phase4/issues/02-hover-thrust-backfill.md)）：
模型侧的 `hover_thrust` 已回填，但**飞控侧的 `MPC_THR_HOVER` 还是 PX4 默认的 `0.5`** ——
它要在 QGC 里改，是**人的动作**。两个值是同一个物理量，**只改一处 = 静默的推力增益错误**；
核对记录（含 `THR_MDL_FAC` 复核、与理论 42.9 % 的差异、以及三条连带效应）全在 **§3.3.5**。

---

## 6. 六个必须提前知道的坑

### 6.1 ★★ `px4_msgs` 版本不匹配 → **静默失败**

v1.15 及以前，`px4_msgs` 与固件不匹配时：话题存在、能发布、**无报错**，但 PX4 不理会。
**排查时你会怀疑 MPC、怀疑符号、怀疑一切，唯独不会怀疑消息定义。**

> ### ☠️ Q24 那次验收**判反了** —— 2026-09-21 定案：`release/1.14` **兼容，已签字采用**
>
> 2026-09-20 执行 Q24 验收时写的结论是「**不兼容**：184 个共用消息中 28 个字段布局不同」。
> **该结论作废** —— 它是拿 **fork `5ba2e91f`**（**不是 z0 的固件**）的 `msg/` 比出来的。
> 换成**真身 `4a0e65f2`**（v1.14.3 系的 MicoAir 厂商构建）后重比：**共有的 184 个 `msg/` 文件 blob 全同**
> （唯一集合差异是 `msg/CMakeLists.txt`，那是 PX4 自己的构建文件，`px4_msgs` 本就不带）。
>
> **⟹ `px4_msgs` = `release/1.14` @ `ffb6e80e`**（✅ **用户已签字**）。**不要从任何 fork 生成。**
>
> ☠️ 那次验收"附带推翻的项目前提"**同样作废**：原写「`5ba2e91f` 的 `OffboardControlMode` 与上游 v1.15.0 一致
> ⟹ z0 固件是「v1.15 世代」、「1.14.x」这个标签是错的」。
> `OffboardControlMode` 带 `thrust_and_torque` 是 **fork 的事实**；真身那一栏是 **`bool actuator`**
> （与上游 v1.14.3 相同）。⟹ **「z0 是 1.14.x」这个标签一直是对的**，G6 已按定案重写。
>
> ⚠️ **本节描述的坑本身仍然真实存在**（`msg/versioned/` 是 **v1.16 才有**，没有任何自动保护）
> —— 只是**我们并没有踩到它**：这一次的"不匹配"是误报。
>
> **→ S1 的第一步不再是"定案怎么得到与 fork 逐字节一致的 `px4_msgs`"，而是
> 「用 `release/1.14` @ `ffb6e80e` + 上游 v1.14.3 跑 SITL，并跑一遍**消息一致性比对** **复验**」。**
> ✅ **2026-09-28 已复验通过**（ticket 01）：比对脚本用 `tools/px4-sitl/check_msg_compat.py`
> —— ☠️ **不写 `check-message-compatibility.py`**（原写法），那个路径**在 PX4-Autopilot 里不存在**且直接跑会给假 FAILED，
> 理由见 §S1 与 [`px4-sitl-baseline.md`](px4-sitl-baseline.md) §2.3。
>
> 完整对照表见 [`refs-manifest.md`](../reference/refs-manifest.md) §2.5(d) 与 §2.6 执行版。
>
> ### ✅ 载重结论已**对着真身**复验（2026-09-22 重做）
>
> 原注写的是"已重新对着 **fork** `5ba2e91f` 复核" —— **对策对象又选错了**（fork 不是 z0 的固件）。
> 现已**直接对着真身 `4a0e65f2` 的树**重读一遍（`git show <sha>:<path>`，对象在 `~/refs/px4-z0/` 里）：
>
> §3.3 / §6.3 的「offboard 分支对 `thrust_body` **不做任何 hover-thrust 重标定，原样透传**」——
> `src/modules/mc_rate_control/MulticopterRateControl.cpp:182`（**真身内，逐字**）：
>
> ```cpp
> } else if (_vehicle_rates_setpoint_sub.update(&vehicle_rates_setpoint)) {
> 	if (_vehicle_rates_setpoint_sub.copy(&vehicle_rates_setpoint)) {
> 		_rates_setpoint(0) = PX4_ISFINITE(vehicle_rates_setpoint.roll)  ? ... : rates(0);
> 		_rates_setpoint(1) = PX4_ISFINITE(vehicle_rates_setpoint.pitch) ? ... : rates(1);
> 		_rates_setpoint(2) = PX4_ISFINITE(vehicle_rates_setpoint.yaw)   ? ... : rates(2);
> 		_thrust_setpoint = Vector3f(vehicle_rates_setpoint.thrust_body);   // ← 与 v1.14.3 完全一致
> 	}
> }
> ```
>
> **与 v1.14.3 那一行逐字相同，行号也相同（都是 `:182`）。→ §3.3、§6.2、§6.3 三条结论成立。**
>
> ### ✅ 复验清单（**2026-09-22 全部改对着真身 `4a0e65f2` 重做**）
>
> 原清单（2026-09-20，Q33=a）是对着 **fork `5ba2e91f`** 做的 —— **对策对象错了**，
> 已逐条对着**真身**重验。**结论全部成立**；下表行号是**真身内**的行号（与 fork 有出入，已更正）：
>
> | 条目 | 复验结果 | 真身内证据 |
> |---|---|---|
> | §3.3 / §6.2 / §6.3 `thrust_body` 原样透传、不受 `MPC_THR_HOVER` 影响 | ✅ **成立** | `src/modules/mc_rate_control/MulticopterRateControl.cpp:182` `_thrust_setpoint = Vector3f(vehicle_rates_setpoint.thrust_body);`（**与 v1.14.3 逐字相同**） |
> | §3.3 `MC_BAT_SCALE_EN` 默认 **0**（关闭） | ✅ **成立** | `src/modules/mc_rate_control/mc_rate_control_params.c:400` `PARAM_DEFINE_INT32(MC_BAT_SCALE_EN, 0);`（原注 `:294` 是 fork 的行号，**已失效**）—— **含义**：角速率环**不做**电池电压推力补偿 → `(C)` 级的推力映射**不需要**考虑电压 sag |
> | §6.4 QoS（订阅 `fmu/out/*` 必须 `BEST_EFFORT`） | ✅ **成立** | `src/modules/uxrce_dds_client/dds_topics.h.em:69`（`uxr_prepare_output_stream(session, best_effort_stream_id, …)`，原注 `:109` 已失效）、`utilities.hpp:114-119`（datareader QoS = `VOLATILE` + `BEST_EFFORT` + `KEEP_LAST`，原注 `:120-125` 已失效） |
> | §6.5 时间戳单位微秒 | ✅ **成立**（两条独立证据） | `msg/VehicleRatesSetpoint.msg:1` 逐字含 `(microseconds)`；`dds_topics.h.em:52` 与 `:103` `int64_t time_offset_us = session->time_offset / 1000; // ns -> us`（原注 `:91/:146` 已失效） |
> | §6.1 `px4_msgs` 版本 | ❌ **原判被推翻 → 改判「`release/1.14` 兼容」** | 见上 |
> | S1 SITL 机型可用性 | ✅ **上游 v1.14.3 也有 `4001_gz_x500`** | 见 [`refs-manifest.md`](../reference/refs-manifest.md) §2.6 可行性表（原注"fork 63 个"是对 fork 数的，已不相关） |
>
> **★ 一条仍然成立的范围澄清**：**§2.4 的「五个坑」不需要按真身复验** ——
> 那五条针对的是 **px4-mpc 自己的 Python 代码**（`0.07` 魔数、`raise` 兜底、代价罚 `u−0`、约束范围），
> **与 PX4 固件版本无关**。它们已在 Q25 的那一轮里对着 px4-mpc `master` 核过。
>
> **→ 所有依赖「PX4 版本」的载重结论，现在都建立在真身 `4a0e65f2`（v1.14.3 系）之上。**
> ✅ **本节原写「仍未复验的一项：SITL 机型条目数」—— 已复验**：`ROMFS/px4fmu_common/init.d-posix/airframes/`
> 实测 **56** 个条目（55 airframe + `CMakeLists`），见上面 §S1 表与 [`px4-sitl-baseline.md`](px4-sitl-baseline.md)。

### 6.2 `thrust_body` 是 `[-1, 1]`，**不是** `[0,1]`、**不是**牛顿

第一手证据见 §3.3：`_thrust_setpoint = Vector3f(vehicle_rates_setpoint.thrust_body)` 原样透传。
FRD 下 z 朝下 ⇒ **向上推力是负值**。写成 `+F/T_max` 会直接**全速砸地**
（符号、量纲与推导：[`sign-map.md`](sign-map.md) §5.3）。

### 6.3 `thrust_body` 不受 `MPC_THR_HOVER` 影响

同一个 offboard 分支里没有任何 hover-thrust 重标定。所以**模型里的 `hover_thrust` 必须和 `thrust_scale` 自洽**（§2.4 坑 1）。改了其中一个不改另一个 = 静默的推力增益错误。

### 6.4 ★ QoS 用默认值会收不到 PX4 话题

PX4 的 `fmu/*` 话题是 **BEST_EFFORT**。ROS 2 默认 `RELIABLE` 的订阅者**与 BEST_EFFORT 发布者不兼容**，表现为「话题列出来了，但一个回调都不进」。
→ 订阅者必须显式声明 `BEST_EFFORT`。（px4-mpc 里已写对，**照抄别改**。）

> ✅ **2026-09-22 对着真身 `4a0e65f2` 复验通过**（原注是对着 fork 做的，已改对象并更正行号），把两个方向拆开说清楚了 ——
> **原表述把「订阅」和「发布」当成一回事，实际不对称**：
>
> | 方向 | 固件侧的事实（第一手） | 我们该怎么做 |
> |---|---|---|
> | **订阅 `fmu/out/*`** | PX4 **所有**出向话题都走 **best-effort 流**（`dds_topics.h.em:69`：`uxr_prepare_output_stream(session, best_effort_stream_id, ...)`） | **必须声明 `BEST_EFFORT`** —— RELIABLE 订阅者**匹配不上**。这就是"话题在、回调不进"的成因 |
> | **发布 `fmu/in/*`** | PX4 建的 DataReader 是 **`VOLATILE` + `BEST_EFFORT` + `KEEP_LAST`**（`utilities.hpp:114-119`，`uxrQoS_t qos = {...}` 逐字）；且 `create_data_reader` 用的输入流是 `best_effort_in_stream_id`（`dds_topics.h.em:131`） | `BEST_EFFORT` 即可。⚠️ 注意 PX4 的 reader 是 **`VOLATILE`**，所以 pub 侧用 `TRANSIENT_LOCAL` **也兼容**（volatile reader 接受 transient-local writer，只是拿不到历史样本）—— px4-mpc 的 `TRANSIENT_LOCAL` 因此没问题，**不是 bug** |
>
> **`depth` 也不对称**：PX4 的 reader `depth = orb_get_queue_size(...) * 2`，px4-mpc 的 pub `depth = 1` —— 兼容，不用对齐。

### 6.5 时间戳单位是**微秒**

`Clock().now().nanoseconds / 1000`。PX4 全用微秒。传成纳秒的话时间戳会大到离谱，PX4 可能直接丢弃。**这个错误不会报错，只会"没反应"。**

> ✅ **2026-09-22 对着真身 `4a0e65f2` 复验通过（两条独立证据）**（原注是对着 fork 做的，已改对象并更正行号）：
> ① `msg/VehicleRatesSetpoint.msg:1` 逐字 `uint64 timestamp	# time since system start (microseconds)`；
> ② 固件在 DDS 边界上自己做 `ns → us` 换算 —— `dds_topics.h.em:52` 与 `:103`：
> `int64_t time_offset_us = session->time_offset / 1000; // ns -> us`。
> **→ 微秒是 DDS 边界的硬约定，不是习惯。**

### 6.6 求解失败必须活着

`raise` 是天上的自杀（§2.4 坑 2）。**S5 验收第 4 项就是专门测这个。**

---

## 7. 与里程碑的对应

| [`../PROJECT.md`](../PROJECT.md) 里程碑 | 本文件 |
|---|---|
| 里程碑 0（环境就绪） | **S1**（SITL + `px4_msgs`）、**S2**（acados） |
| 里程碑 2（系统辨识） | **P1 / P2 / P4**（§5） |
| 里程碑 3（仿真闭环） | **S3**（离线 OCP）、**S4**（节点）、**S5**（SITL 闭环） |
| **里程碑 4（实机 `(C)`）** | **S6** |
| 里程碑 5（INDI / 内环建模） | **S7** |
| 里程碑 7（`(D)`） | 范围外 |

---

## 8. 实施顺序一览（贴墙用）

```
S1 环境 ──> S2 acados ──> S3 离线 OCP ──> S4 节点 ──> S5 SITL ★
                                                        │
                                        ┌───────────────┘
                                        ▼
                              【Phase 1 必须已达标】
                                        │
                                        ▼
                            P1 标定 hover_thrust ★阻塞项
                                        │
                                        ▼
                        S6 实机（拆桨→系绳→自由飞）★
                                        │
                                        ▼
                                  S7 内环建模
```

**S5 之前不碰飞机。P1 没测出来不飞自由悬停。**

---

## 9. 本文的开放项（需确认 / 待实测）

| # | 开放项 | 本文主张 | 谁来定 |
|---|---|---|---|
| ~~O1~~ | ~~D7：内部坐标系~~ | ✅ **已关闭** —— **ENU/FLU**，用户选定「参考论文里的坐标系」，`Sun2022` + `Foehn2022` 一致（§3.2） | 已定 |
| ~~O2~~ | ~~`MPC_TILTMAX_AIR` 45°→30°~~ | ☠️ **2026-09-28 操作者推翻**：**两边都回 45**。✅ **正文已按裁定重写**（§3.3.4）。⚠️ **2026-09-29 P1 回填 + 解耦后，这组数字按哪一层算要重新说清**：原写"天花板 51.3°、45° 用掉 88.4 %"是按 `m·g/0.6235 = 11.80 N` 算的 —— 那**恰好就是电池层的 `f_max`**（11.8 N，T/W 1.60），所以**那组数字本身站得住**（88.4 % 是 `F` 预算的占用率，精确值见 §3.3.4）。而实测 `hover_thrust = 0.1589` 给出的是**另一层**：归一化满量程 46.30 N ⟹ 45° 只占 **22.5 %**、天花板 **80.9°**。★ 两层别混（§3.3.3）：**约束 `F` 的是电池层 88.4 %**，归一化层那 22.5 % 不代表有余量。★ 裁定本身**保留**（45° 仍成立）；§3.3.4 已重写。⚠️ 顺带查清：`MPC_TILTMAX_AIR` **不在本项目通路上**（我们发 `body_rate`，`flag_multicopter_position_control_enabled = false`），且**模型侧目前没有倾角约束** | 已定（理由已换） |
| ~~O3~~ | ~~D6：MPC 频率~~ | ✅ **已关闭** —— **100 Hz**（P3 实测：纯求解 max 3.58 ms；见 D6 行与 [`offline-ocp.md`](offline-ocp.md) §5）。⚠️ 附带条件：节点侧要处理墙钟尾延迟（1/1000 次 52–56 ms 抢占）。★ **闭环复测（2026-09-28，ticket 09）**：空机稳态最坏 **3.995 ms** ⟹ 100 Hz 复现；⚠️ 新增前置条件"**机上别跑别的东西**"（同二进制多跑了几个进程就变 11.297 ms ⟹ 50 Hz），见 [`tier-a-sitl-hover.md`](tier-a-sitl-hover.md) §5 | 已定 |
| O4 | D9：延迟补偿 | 步骤 1 只用一行状态外推；完整补偿留第二版 | 用户 |
| ~~O5~~ | ~~`R` 改罚 `u − u_ref`（M6）的具体参考量~~ | ✅ **已关闭（2026-09-28，ticket 04）**：约定写死在 [`offline-ocp.md`](offline-ocp.md) **§3.2** —— 缺省 = **上一帧解平移一拍**（首帧用悬停前馈）；调用方可给解析前馈；`prediction=False` 时**必须显式给**（缺省直接报错，不许悄悄退回悬停前馈）。行为证据见同文 §3.3。★★ **闭环复测（2026-09-28，ticket 09，SITL 真 PX4）**：那个"缺省 = 上一帧解"在**闭环里不成立** —— 它是正反馈，会把外生的小扰动放大成 runaway（`u[3]` 爬到 +4.696 rad/s）。**离线缺省一个字没动**；**在线节点**改喂 §3.2 第二行"调用方给的解析前馈"，见上方 M6 补记与 [`tier-a-sitl-hover.md`](tier-a-sitl-hover.md) §3 | 已定（在线用法已由 ticket 09 修正）|
| O6 | D5：是否换高 C 电池 | 与本文实施步骤无关，参数化即可 | 用户（硬件决策） |
| **O7** | **长时段"不信任"期间该发什么** | 票面（ticket 07）对时间戳倒退 / 位姿跳变的要求是**只关闸门、一帧不发** —— 已照做。但由 §2.5 的第一手，**不发 = 飞控无限期锁存最后一帧**（可能是几秒前的、带满角速率的命令）；"一直沉默"与"主动切安全悬停"哪个更安全**本票没有拍板**（票面没要求，就没自作主张改）。⚠️ 触发条件苛刻（动捕要连续坏 ≥ 闸门重开的时间）但**真实存在**：遮挡、上游重放、VRPN 时钟回拨都会造成 | SITL/实测定；结论进 ticket 11 |
| ~~O8~~ | ~~**P1/P2 的电池口径**（满电 vs 低压）~~ | ✅ **2026-09-29 操作者裁定：满电口径** —— 已按此**回填**（`hover_thrust = 0.1589`、`T_max = 46.3 N`，三处同值）。⚠️ **诚实交代**：四段真机日志**全是低电段**（起飞 SOC 46–60 %），满电值是**外推** —— 由 20.474 / 20.526 V 上的 `0.221992` / `0.220884` 按实测律 `thrust ∝ V^1.608` 归一到 25.2 V（得 `0.15896` / `0.15882`，互差 0.09 %）。★ 若要把外推换成直接读数，**在满电包上再悬停 ~30 s** 即可。★ 裁定只解决"选哪个口径"；M4/D5 的解耦问题**同日另行裁定**（`F` 上限改成独立的 `f_max`，见 §3.3.3）。**保留原「建议」的理由**（为什么是满电）：把模型锚在"每次飞行都从这儿开始"的那个状态；误差随电量下降朝"**模型高估能力**"漂，而这一侧**有可观测信号**（`sat` 饱和标志 + 位置环会自己补），反方向没有 | ✅ **已裁定（2026-09-29）** |
| **O9** | **OCP 代价里的 `q_diag[6]`（姿态四元数 `qw` 的权重）与 `r_diag[3]`（`R_yaw`）** —— ticket 04 的交付物 | ticket 09 在闭环里量化了它的一个**结构性后果**：`qw = 0` ⟹ 代价**分不出**参考机头 ψ 与它的镜像 `2π − ψ`（两者相差 `w = 2cos(ψ/2)`，实测代价**都恰好 = 0.000000**，同文 [`offline-ocp.md`](offline-ocp.md) §2）。于是偏航通道在参考附近**既不收敛也不漏**（0.40 rad 偏差 ⟹ −1.9e-5 rad/s；0.15 rad ⟹ −7e-6 rad/s —— 实测**死通道**），而单次 RTI 迭代下还带一点**依赖热启动**的抖动（roll 0.08 ⟹ −0.018378 rad/s）。**已开 ticket 16 并裁定落地**：`q_diag` 四元数段 **`(0.0, 0.1, 0.1, 0.1) → (0.1, 2.0, 2.0, 2.0)`**（镜像代价 `0.000000 → 9.528`，只由 `qw` 决定），**`r_diag` 不动**（☠️ 票内一度把 `R_yaw` 也降到 50，当天用逐拍对齐 DUT 的离线重放查出它会让 10 m 不可行步进下的 RTI 迭代**收敛极慢**（每 ~100 拍才衰减 10×，发指令的窗口里就是逐拍乱跳），撤回）；"偏航是死通道"**没有被这一项解决**（纯 0.4 rad 机头误差的稳态 `ω_z` 只从 `2e-5` 抬到 `3.9e-4 rad/s`，仍 ~1000 s）；闭环复飞待 ticket 16 验收项 3 | ✅ 已定（2026-09-29，ticket 16） |
| **O10** | **`MPC_THR_HOVER` 写下去之后，手动模式的手感变了** —— `phase4/02-C1` 的连带效应（§3.3.5 ⑥b） | 不算问题，但**要知道**：`MPC_THR_CURVE = 0` 的语义是"居中杆 = `MPC_THR_HOVER`"，所以把 0.5 改成 0.1589 ⟹ **居中杆给出的推力从 0.5 掉到 0.1589**（手动悬停点回到杆居中，这是本意、也改善手动↔位置模式的衔接），但上半程杆要吃掉 84 % 的推力范围。**建议保留 `MPC_THR_CURVE = 0`**（改成 1 会让悬停落在 8.6 % 杆位）。⚠️ **不影响离板/MPC 通路**（位置控制器不在通路上）。★ 本条**没有实测过**：手感变化要在**上电后手动悬停**时确认一眼 | 写参数的人（上电时顺手看） |

---

## 附：本文中「第一手」结论的证据索引

| 结论 | 证据 |
|---|---|
| px4-mpc 的 OCP 是 10 状态、ω 不是状态 | `models/multirotor_rate_model.py`，`x = cs.vertcat(p, v, q)`，`nx=10` |
| px4-mpc 内部是 ENU/FLU | 同上，`g=[0,0,-9.81]`、`force=[0,0,F]` |
| **参考实现**的输出侧依次取 `roll/pitch/yaw = +ωx, −ωy, −ωz`、推力取反（★ 这是对**上游源码的引用**，不是本项目的规定；**本项目自己的权威表在 [`sign-map.md`](sign-map.md) §2**） | `mpc_quadrotor.py`，`setpoint_msg.roll/pitch/yaw` 与 `thrust_command` |
| `0.07 ≈ 1/F_max` | `multirotor_rate_model.py` `max_thrust = mass*9.81/hover_thrust` ⇒ `1/13.44 = 0.0744` |
| `R` 罚绝对角速率 | `multirotor_rate_mpc.py` `yref` 的 u 部分全零 + `R_mat` |
| `lbu/ubu` 的 yaw 取半 | `multirotor_rate_mpc.py` `-0.5*wmax` / `+0.5*wmax` |
| `thrust_body` 原样透传、不受 `MPC_THR_HOVER` 影响 | PX4 `v1.14.3` `src/modules/mc_rate_control/MulticopterRateControl.cpp`，offboard 分支 `_thrust_setpoint = Vector3f(vehicle_rates_setpoint.thrust_body);` |
| **推力轴按各执行器 `ct` 的均值归一化** ⟹ `thrust_body = 1.0` ≡ 所有执行器满输出 | PX4 `v1.14.3` `src/modules/control_allocator/ControlAllocation/ControlAllocationPseudoInverse.cpp:134`：`_control_allocation_scale(3 + axis_idx) = norm_sum / num_non_zero_thrust;`（`:116-141` 整段；`norm_sum` 是 `fabsf(_mix(i, 3 + axis_idx))` 之和） |
| `THR_MDL_FAC` 曲线**不涉及悬停推力**（只改「归一化推力 ↔ 电机信号」的形状，端点不动） | 同上 `src/lib/mixer_module/functions/FunctionMotors.hpp:77-101`：`rel_thrust = factor*x² + (1-factor)*x`，`x=1 → 1`；定义在 `src/lib/mixer_module/motor_params.c:73`（默认 `0.0f`） |
| **`_hover_thrust` 在「加速度 → 推力」里只是个增益**，稳态推力由积分器逼到 `m·g` ⟹ 读数反映被控对象、不反映该参数 | PX4 `v1.14.3` `src/modules/mc_pos_control/PositionControl/PositionControl.cpp:210`：`collective_thrust = _acc_sp(2) * (_hover_thrust / CONSTANTS_ONE_G) - _hover_thrust;`；积分项 `:87`（累积）/`:143`（限幅 ±g） |
| `MPC_USE_HTE = 1` 时位置控制器用的是 HTE ⟹ 它**不是独立口径**（且仅 `hte.valid` 时生效） | 同上 `src/modules/mc_pos_control/MulticopterPositionControl.cpp:361-368`：`if (_param_mpc_use_hte.get()) { ... if (hte.valid) { _control.updateHoverThrust(hte.hover_thrust); } }` |
| 我们发 `body_rate` ⟹ PX4 **位置控制器（含倾角限幅）不在通路上** | ☠️ **2026-10-13（票 13 收口轮）：结论不变，引证换了一处** —— 原来引的是 `mpc_node.py` 设 `offboard_control_mode.body_rate = True`，**那条流今天已删**；现在对应的是 `AttitudeTarget` 的 `type_mask = 128`（`IGNORE_ATTITUDE` ⟹ 只给 `body_rate`）。飞控侧那两条**照旧**：PX4 `src/modules/commander/ModeUtil/control_mode.cpp:144-146`（`body_rate` ⟹ 只置 `flag_control_rates_enabled`）+ `:169-175`（只有 altitude/climb_rate/position/velocity/acceleration 之一才置 `flag_multicopter_position_control_enabled`）|
| 角速率有 NaN 回退、推力没有 | 同上，`PX4_ISFINITE(...)` 只出现在三行角速率上 |
| `MC_BAT_SCALE_EN` 默认 **0（关闭）** | PX4 `v1.14.3` `src/modules/mc_rate_control/mc_rate_control_params.c`，`PARAM_DEFINE_INT32(MC_BAT_SCALE_EN, 0);` |
| `gz_x500` 在 v1.14.3 存在 | `git ls-tree v1.14.3:ROMFS/px4fmu_common/init.d-posix/airframes` → `4001_gz_x500`（★ 但**存在 ≠ 本机可用**，见 §S1） |
| `px4_msgs` 有 `release/1.14` 分支与 `v1.14.0` 标签 | `git ls-remote --heads --tags https://github.com/PX4/px4_msgs.git` |
| SIH 可用、机型是 `sihsim_{airplane,quadx,xvert}` | v1.14.3 `src/modules/simulation/simulator_sih/CMakeLists.txt:51` `set(models …)`、`:94` `add_custom_target(sihsim_${model} …)` |
| v1.14.3 的 airframe 共 56 个条目（SIH 3 / gz 6 / gazebo-classic 35 / jsbsim+flightgear 9 / jmavsim 1 / none 1 + `CMakeLists.txt`） | `ls ROMFS/px4fmu_common/init.d-posix/airframes/`，按后缀 `grep -c` |
| gz-sim 在 v1.14.3 **只支持 Garden** | `src/modules/simulation/gz_bridge/CMakeLists.txt:38-39`：`#ignition-transport11 # IGN (Fortress and earlier) no longer supported` + `gz-transport12` |
| submodule 是 **17** 个（不是 23） | `grep -c '^\[submodule' ~/refs/px4-v1.14.3-build/.gitmodules` |
| **`MPC_THR_HOVER` 与模型 `hover_thrust` 在同一个域里**（⟹ 两处必须同值，`phase4/02-C1`） | `PositionControl.cpp:210`（`collective_thrust = _acc_sp(2)·(_hover_thrust/g) − _hover_thrust` ⟹ 悬停时 `thrust_body[2] = −MPC_THR_HOVER`）+ 上面的「推力轴按均值归一化」条 + `MulticopterRateControl.cpp:182`（原样透传）。核对记录见 **§3.3.5** |
| **HTE 的窗口 = `MPC_THR_HOVER ± HTE_THR_RANGE`**，且 EKF 会把它夹住 | PX4 `v1.14.3` `src/modules/mc_hover_thrust_estimator/MulticopterHoverThrustEstimator.cpp:90-93`：`setMinHoverThrust(constrain(MPC_THR_HOVER − HTE_THR_RANGE, 0, 0.8))`、`setMaxHoverThrust(constrain(..., 0.2, 0.9))`；夹钳在 `zero_order_hover_thrust_ekf.cpp:124`：`_hover_thr = constrain(_hover_thr + K·innov, _hover_thr_min, _hover_thr_max)`。z0 实测估计值**整段贴在下限 0.300**、`valid` 97.3 %（ticket 10 的第三条交叉验证） |
| **降落检测的两个阈值由 `MPC_THR_MIN` 与 `MPC_THR_HOVER`（HTE 有效时用 HTE）合成** | 同 `v1.14.3` `src/modules/land_detector/MulticopterLandDetector.cpp:79`（`minThrottle ← MPC_THR_MIN`）、`:81`（`hoverThrottle ← MPC_THR_HOVER`）、`:101-110`（HTE 有效时覆盖）、`:216-219`（`sys_low_throttle = minThrottle + (hoverThrottle − minThrottle)·(0.6 或 0.3)`）、`:262`（maybe-landed 用 `×0.1`） |
| **手动模式的杆曲线语义 = "居中杆 ↔ `MPC_THR_HOVER`"** | 同 `v1.14.3` `src/modules/mc_att_control/mc_att_control_main.cpp:99-109`（`throttle_curve()`：`case 1` 走 `interpolate`，默认走 `interpolateN({MPC_MANTHR_MIN, MPC_THR_HOVER, MPC_THR_MAX})`）+ `mc_pos_control_params.c:96-107` 的参数文档（含 "if hover thrust is set to 20 %, 80 % of the upper thrust range is squeezed into the upper half"） |
| **`THR_MDL_FAC = 0.45` 被飞行数据支持**（argmax r²，非取默认值） | `tools/hover-thrust/thrust_curve_check.py` 在两条真机上跑出 argmax `0.455` / `0.450`（对照 `fac=0` 得 0.9952/0.9955、`fac=1` 得 0.9871/0.9836）；原始输出 [`tools/hover-thrust/evidence/thrust_curve_2026-10-09.txt`](../../tools/hover-thrust/evidence/thrust_curve_2026-10-09.txt) |
| 真身 `4a0e65f2` ↔ 上游 `v1.14.3` 的 `msg/`：185 个文件 blob **全同**、**无集合差异** | `git -C ~/refs/px4-z0 ls-tree -r 4a0e65f2 -- msg/` vs `git -C ~/refs/px4-v1.14.3 ls-tree -r HEAD -- msg/`，按文件名 join 比 blob |
| `Tools/check-message-compatibility.py` **不在 PX4-Autopilot 里** | 逐棵树 `ls`/`find`：v1.14.3 / v1.15.0 / v1.16.0 / fork `5ba2e91f` / `px4_msgs` 全无；`Tools/` 下只有三个 `*msg*` 脚本，仓库根无 `scripts/` |
| 消息比对的官方算法在 `auterion/px4-ros2-interface-lib`，且直接跑会给假 FAILED | `scripts/check-message-compatibility.py`；其类型清单来自同仓 `message_compatibility_check.hpp`，该仓**无 `release/1.14` 分支** ⟹ 报 20 个 v1.14 根本不存在的类型为「缺失」，但报错里**没有** `Some files differ` |
| `px4_msgs` 与上游 v1.14.3 的 184 个共有 `msg/` 文件**字段布局全同** | `tools/px4-sitl/check_msg_compat.py`（复用官方 `message_hash`），**RESULT: PASS** |
| `MicroXRCEAgent` 跑通后 `/fmu/in/*` 13 条 + `/fmu/out/*` 11 条，且有真实数据 | `run_sitl.sh --agent` + `ros2 topic list \| grep fmu` + `ros2 topic echo --qos-reliability best_effort --once /fmu/out/vehicle_status` |
| Agent superbuild 会 checkout 上游**已删除**的分支 `2.12.x` | `git ls-remote --heads https://github.com/eProsima/Fast-DDS.git` 无 `2.12.x`；报错 `fatal: invalid reference: 2.12.x`（**不指向根因**） |

> 📌 **顺带一条对用户有利的发现**：`MC_BAT_SCALE_EN` 默认**关闭**，所以 [`phase1-implementation.md`](phase1-implementation.md) §8.2 里记的 ADC 电池电流采样 bug **在默认配置下不会污染推力/力矩指令**（该参数打开时才会把 `thrust` 和 `torque` 同时乘上由电池测量算出的 `battery_status.scale`）。
> **但如果将来有人图"电量低时性能一致"把 `MC_BAT_SCALE_EN` 打开，这个 ADC bug 就会从"遥测读数不对"升级成"控制环里的乘性干扰"。** 建议在 z0 上**保持该参数为 0**。
