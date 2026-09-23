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

1. **Phase 2 = MPC 输出「角速率 + 推力」，PX4 只跑角速率环 + 混控。** 消息是 `/fmu/in/vehicle_rates_setpoint`（uXRCE-DDS），**不是** mavros。这是 `(C)` 级。
2. **代码骨架取 [`DISCOWER/px4-mpc`](https://github.com/DISCOWER/px4-mpc)**（BSD 3-Clause，可自由改）。但它是为 x500 写的，有 **6 处必须改**（§2.5），其中 2 处改错会直接摔机。
3. **Phase 2 引入 4 个新东西**：acados 求解器、uXRCE-DDS 中间件、MPC 节点、`px4_msgs` 版本绑定。**它们在 SITL 里全部跑通之前，实机一行都不许上。**

---

## 1. 已定决策（从对话中固化，不再重开）

| # | 决策 | 结论 | 依据 |
|---|---|---|---|
| G1 | 控制层级 | **`(C)` 角速率 + 推力** | roadmap §2 四条论据 |
| G2 | 中间件 | **混合**：mavros2 收动捕 / uXRCE-DDS 发指令 | 用户 Q7：「Phase 1 全走 mavros2，uXRCE-DDS 推迟到 Phase 2」 |
| G3 | MPC 状态输入 | **动捕直出**（不经 EKF2） | 用户 Q9 + roadmap D8；延迟更低 |
| G4 | 求解器 | **acados** + `SQP_RTI` + `PARTIAL_CONDENSING_HPIPM` | roadmap §4.5，与 px4-mpc 一致 |
| G5 | 代码骨架 | **`DISCOWER/px4-mpc`**（复制改造，不引依赖） | 第一手读过三个核心文件，见 §2 |
| G6 | 固件版本 | ★ **版本基准 = z0 固件真身 `Minderring/PX4-Autopilot` @ `4a0e65f2`**（**v1.14.3 系**的 MicoAir 厂商构建，飞控自报 `1.14.3`）。⟹ **`px4_msgs` = `release/1.14` @ `ffb6e80e`**（与真身共有的 184 个 `msg/` blob 全同，✅ 用户签字）；**SITL 被控对象 = 上游 v1.14.3（tag `1dacb4c`）**。☠️ ~~版本基准统一到 fork `5ba2e91f`（Q31=a）~~ + ☠️ ~~固件是「v1.15 世代」~~ —— **两句都反了，`5ba2e91f` 与 z0 无关**。见 [`refs-manifest.md`](../reference/refs-manifest.md) §2.5(d) 与 §2.6 执行版。⚠️ **「不刷固件」这条决定本身不变** | 用户 Q12 + [`refs-manifest.md`](../reference/refs-manifest.md) §2.5(d) / §2.6 |
| G7 | 上线顺序 | **先 SITL 验证 MPC，再实机** | 用户 Q8(b) |
| G8 | INDI | **第一版不做**，作为步骤 2 | roadmap D3 |
| G9 | 参考轨迹 | **第一阶段手写**（悬停 / 正方形 / 八字），与 Phase 1 同一套 | 用户 Q15 |
| G10 | 动捕原点 | **未起飞时静止帧为原点**，yaw 只归零偏航 | 用户 Q16 / Q21(b) |
| G11 | 动捕输出 | **只发 `/mocap/pose_enu`（ENU / 原生）**；~~`/mocap/pose_ned`~~ **不发布**（Q14=a 已删） | 本文 §3.2 + 用户 Q14 |

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

### 2.3 ★ 符号映射表（**整个 Phase 2 最容易摔机的地方**）

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

**(c) 归一化对照表**

| 量 | MPC 内部 | PX4 `VehicleRatesSetpoint` | 变换 |
|---|---|---|---|
| roll rate | `ω_x` (FLU) | `roll` (FRD) | **不变** |
| pitch rate | `ω_y` (FLU) | `pitch` (FRD) | **取反** |
| yaw rate | `ω_z` (FLU) | `yaw` (FRD) | **取反** |
| 推力 | `F` [N]，≥0，沿机体 +z | `thrust_body[2]` ∈ [−1,0]，FRD | `−F / T_max` |

> ✅ 这套符号本身是**正确的**：FRD 与 FLU 的关系就是「x 不变、y 和 z 取反」。推力取负号也对（FRD 下 z 朝下，向上推力 = 负 z）。

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

#### 坑 2 ★★ `solve()` 失败时直接 `raise`

```python
status = ocp_solver.solve()
if status != 0:
    raise Exception(f'acados returned status {status}.')
```

在 SITL 里这是好习惯；**在天上这是让节点崩掉**。节点一崩，`OffboardControlMode` 心跳停发 → PX4 触发 offboard 丢失失效保护。**必须有兜底**：求解失败时保持上一帧的可行指令（或切到悬停推力 + 零角速率），并计数上报。

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
| M4 | `F_max` 按**电池**（≈11.8 N），不按电机 | §3.3 | 🔴 |
| M5 | `solve()` 失败 + NaN 的兜底路径 | 坑 2、坑 3 | 🔴 |
| M6 | `R` 改罚 `u − u_ref` | 坑 4 | 🟡 |
| M7 | 状态源从 PX4 话题换成 `/mocap/pose_enu` | G3；且**因此不需要 (a) 那两行转换** | 🟡 |
| M8 | 删掉 `_v1` 后缀话题的双订阅 hack | `_v1` / `msg/versioned/` 是 **v1.16 才有**的机制，**z0 真身（v1.14.3 系）里根本不存在** → 这段兼容代码对我们纯属死代码（⚠️ 该结论与 09-21 那轮的措辞「fork（v1.15 世代）里不存在」**等价且更贴对象**：真身的 `msg/` 与上游 v1.14.3 逐字节相同） | 🟢 |
| M9 | 补上 `MPC_HORIZON = 15` 这个**定义了却没用的死变量** | 代码卫生 | 🟢 |

> 📌 **M7 顺带干掉了一整类 bug**：既然动捕原生就是 ENU/FLU，MPC 内部也用 ENU/FLU，那么 §2.3(a) 那段手写四元数转换（`1/√2·[w+z, x+y, x−y, w−z]`，**整个文件里最容易写错的一行**）**可以直接删掉**。详见 §3.2。

### 2.6 可以直接抄的部分

| 抄什么 | 位置 | 价值 |
|---|---|---|
| `skew_symmetric(q)`、`v_dot_q`、`q_to_rot_mat` | `models/multirotor_rate_model.py` | 四元数工具，是对的，省去自己推导 |
| OCP 装配骨架（`AcadosOcp` → `NONLINEAR_LS` → `SQP_RTI` → ERK 4/3） | `controllers/multirotor_rate_mpc.py` | 与 roadmap §4.5 完全一致 |
| `OffboardControlMode` 心跳（`body_rate=True`，其余 False，50 Hz） | `mpc_quadrotor.py` | 照抄 |
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
| **D4** | 中间件 | ✅ **已定** | **混合**：uXRCE-DDS 发指令 + mavros2 收动捕（G2） |
| **D5** | `T_max` 取值 | ✅ **方向已定** | **按电池**（≈11.8 N）。是否换高 C 电池 → **本文给出量化依据，见 §3.3，仍可再议** |
| **D6** | MPC 频率 | 🔶 **待实测** | 先按 **100 Hz** 写，但**以实测最坏耗时为准**降档。验收线：最坏 ≤ 10 ms 保 100 Hz；10–20 ms 降 50 Hz；> 20 ms 停下优化 |
| **D7** | MPC 内部世界系 | ✅ **已定** | **ENU/FLU**（世界 z 朝上 + 机体 x 前 / z 沿推力）—— **用户选定「参考论文里的坐标系」，两篇论文一致**，见 §3.2 |
| **D8** | 状态源 | ✅ **已定** | **动捕直出**（G3） |
| **D9** | 50 ms 延迟补偿 | 🔶 **仍开放** | 建议**第二版再做**。步骤 1 先用「状态外推 `x + v·τ`, τ=50 ms」这一行代码顶住，够用且零风险 |
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
   - ENU/FLU：输出侧 `[+ω_x, −ω_y, −ω_z]` —— **纯符号翻转，4 行，肉眼可验**。
   - NED/FRD：输入侧 ENU→NED 四元数 + FLU→FRD 姿态 —— **需要矩阵/四元数推导，出错概率高一个数量级**。

> **结论：把误差风险挪到「纯符号翻转」那一侧，而不是「四元数变换」那一侧。**
> **代价**：`[`mpc-roadmap.md`](mpc-roadmap.md) §4.1/§4.2 的 NED/FRD 表述需要同步改写 —— **已完成**（见该文件 §4.2 的更正注记）。

**★ 顺带一个对 §2.1 的旁证**：**两篇论文都把 `ω` 作为状态**。
`Sun2022` 的 NMPC 状态是 `x = [ξ, ξ̇, q, Ω^B]`；`Foehn2022` 的状态是 `x = [p, q, v, ω, a, τ, s, b_ω, b_a, f_d, f]`。
→ **本文 §2.1 的「步骤 1 用 10 状态（ω 不作状态）」是简化，不是论文的标准做法。** 这不改变结论（先抄 px4-mpc 跑通、再上 13 状态），但**说明步骤 1→步骤 2 的升级方向与论文一致**，不是我们在自创。

### 3.3 ★ 推力上限：为什么必须按电池

**第一手证据**：PX4 `v1.14.3` 的 `MulticopterRateControl.cpp` offboard 分支**对 `thrust_body` 不做任何 hover-thrust 重标定**，直接透传：

```cpp
} else if (_vehicle_rates_setpoint_sub.update(&vehicle_rates_setpoint)) {
    ...
    _thrust_setpoint = Vector3f(vehicle_rates_setpoint.thrust_body);   // 原样使用
}
```

→ **MPC 输出的归一化推力直接就是混控器的输入。** 所以 `F/N → thrust_body` 的换算常数（= `1/T_max`）**必须反映飞机真实能出的最大推力**，否则 MPC 的模型与真实被控对象不符。

**z0 的数**（质量 750 g 实测）：

| 量 | 值 | 来源 |
|---|---|---|
| 悬停推力 `T_hover` | 0.75 × 9.81 = **7.36 N** | 质量实测 |
| **电池限制** `T_max` | 1.60 × 7.36 ~ 2.54 × 7.36 = **11.8 ~ 18.7 N** | [`hardware.md`](../reference/hardware.md) §6.2，T/W 1.60–2.54 |
| 电机限制 `T_max` | 5.44 × 7.36 = **40.0 N** | 同上 |

**★ 按电池取 `T_max = 11.8 N`（保守端）的三个后果**：

1. `thrust_scale = 1/11.8 = 0.0847`；**悬停归一化推力 = −7.36/11.8 = −0.624**。
2. **可用上行裕度只剩 0.376** → 净上行加速度 `(11.8 − 7.36)/0.75 = 5.9 m/s² ≈ 0.6 g`。
3. **⚠️ 45° 倾角下所需推力 = 7.36/cos45° = 10.4 N，已是 11.8 N 的 88 %。**
   → **「`MPC_TILTMAX_AIR = 45°`」与「电池限制的 T_max」几乎是冲突的**：在电量偏低时做 45° 倾斜机动会逼近饱和。

**若按电机 `T_max = 40 N` 归一化**（即抄 px4-mpc 的推导方式），悬停值变成 −0.184，**MPC 会以为自己有大量剩余推力，而电池给不出来** → 期望加速度与实际严重不符，表现为"指令下去飞机不动"甚至振荡。

> **✅ 决议（已确认）：`T_max` 按电池（11.8 N）设，`MPC_TILTMAX_AIR` 从 45° 收到 30°**（30° 时需 8.5 N，占 72 %）。
> **D5 的后续问题**：换高 C 电池能把 T/W 提到 2.54（`T_max` = 18.7 N），倾角就能放回 40°+。**但换电池是硬件改动，不影响本文件的实施步骤** —— 参数化即可。

### 3.4 控制流总览

```
 Nokov 动捕 ──VRPN──> vrpn_listener ──> mocap 节点 ──┬──> /mavros/vision_pose/pose ──> PX4 EKF2 (Phase 1)
  10.1.1.198:3883                                    │      （ENU，mavros 转 NED）
                                                     └──> /mocap/pose_enu ──> MPC 节点   ← Phase 2 新增
                                                                                  │
                        ┌─────────────────────────────────────────────────────────┘
                        ▼
                   acados OCP (ENU/FLU, 10 状态)
                        │  u = [F, ωx, ωy, ωz]
                        ▼
                   符号映射 [+ωx, −ωy, −ωz],  thrust_body[2] = −F/T_max
                        │
                        ▼
              /fmu/in/vehicle_rates_setpoint  (uXRCE-DDS)  +  /fmu/in/offboard_control_mode
                        │
                        ▼
                     PX4 角速率环 (1 kHz) ──> 混控 ──> 电机
```

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

| 项 | 值 | 备注 |
|---|---|---|
| PX4 源码 | ★ **上游 `PX4/PX4-Autopilot` @ tag `v1.14.3`（`1dacb4c`）**（§2.6 执行版；用户未单独表态，取默认） | 需**带 `--recursive` 的完整 clone**（23 个 submodule）；`~/refs/px4-v1.14.3` 是 `--depth 1 --filter=blob:none`，**不能直接构建** → 建议新目录 `~/refs/px4-v1.14.3-build/`。⚠️ 动手前先实测 `ROMFS/.../init.d-posix/airframes/` 条目数，确认要用的机型在（v1.14.3 这里条目比 fork 少）。☠️ ~~fork `Peize-Liu/PX4-Autopilot @ 5ba2e91f`~~ —— **作废，飞控不跑 fork** |
| SITL 仿真后端 | **gazebo-classic 或 gz-sim 都可用（Q12 的 gazebo-classic 选择不变）** | ✅ 第一手（对 fork 核过；上游 v1.14.3 的 `4001_gz_x500` 亦已核到，见 [`refs-manifest.md`](../reference/refs-manifest.md) §2.6 可行性表） |
| `px4_msgs` | ★ **钉 `release/1.14` @ `ffb6e80e`**（✅ **用户签字**） | ✅ 与真身 `4a0e65f2` 在**共有的 184 个 `msg/` 文件上 blob 全同**（唯一集合差异 `msg/CMakeLists.txt`，那是 PX4 的构建文件）。☠️ ~~从 fork 的 `msg/` + `srv/` 生成（Q31=a）~~ + ☠️ ~~`release/1.14` 已被判为不兼容（28/184 字段不同）~~ —— **两条都作废**（那是拿 fork 比出来的）。详见 §6.1 与 [`refs-manifest.md`](../reference/refs-manifest.md) §2.5(d)/§2.6 执行版 |
| 中间件代理 | `micro-xrce-dds-agent udp4 -p 8888` | PX4 SITL 默认端口 |

**⚠️ 必须做的版本校验**（roadmap 与 [`open-source-survey.md`](../reference/open-source-survey.md) §10 都强调过）：

```
PX4 侧：  python3 Tools/check-message-compatibility.py
ROS 侧：  ros2 interface show px4_msgs/msg/VehicleRatesSetpoint
```

**判据**：`px4_msgs` 与固件的消息定义一致。**v1.15 及以前版本不匹配会静默失败** —— 话题能订阅、能发布，但 PX4 收不到，且**不报任何错**。（px4-mpc 里那段同时订阅 `vehicle_status` 和 `vehicle_status_v1` 的代码，就是在绕这个版本问题。我们锁定版本后可以删掉。）

**验收**：`ros2 topic list | grep fmu` 能看到 `/fmu/in/*` 与 `/fmu/out/*`。

### S2 —— acados 在 Jetson Orin NX 上编译

见 roadmap §5 的完整要点。关键三条（**任一漏掉都会编译失败或性能崩**）：

1. **`ACADOS_WITH_OPENMP=OFF`** —— Orin 上 OpenMP 与 acados 的已知冲突。
2. **BLASFEO target = `ARMV8A_ARM_CORTEX_A76`** —— Orin NX 是 Cortex-**A78AE**。**A76 是可用集中最接近的**；用通用 `X64_AUTOMATIC` 或 `GENERIC` 会让性能掉一个量级。
3. **`t_renderer`（Tera）必须装** —— 缺了会在生成代码时报 `EOFError`（这个报错信息误导性极强，不指向根因）。

**验收**：编译 `px4-mpc` 自带的例子（`mass=1, hover_thrust=0.73` 的 x500 配置）能 `AcadosOcpSolver` 创建成功并 `solve()` 返回 0。

### S3 —— OCP 建模 + **离线**验证（不接 ROS）

先用脚本把 OCP 单独跑起来，喂**构造的**状态，看输出合不合理。这一步把「模型/代价/约束」的 bug 和「ROS/通信」的 bug 隔离开。

**要做的离线测试**（每个都是几行代码）：

| 测试 | 输入 | 期望输出 |
|---|---|---|
| 悬停配平 | 状态 = 原点，`v=0`，`q=identity`，目标 = 原点 | `F ≈ T_hover = 7.36 N`，`ω ≈ 0` |
| 差一个重力 | 同上，但目标在 `+z 0.5 m` | `F > T_hover`，`ω ≈ 0` |
| 姿态响应符号 | 同上，但目标在 `+x 0.5 m` | 产生**朝 +x 倾斜**的角速率（在 FLU 下是 `ω_y`... ⚠️ **必须手推一遍确认，不许猜**） |
| 约束生效 | 目标在 `+x 10 m` | `ω` 打满 `ω_max`，`F` 打满 `T_max`，**不越界** |
| 求解耗时 | 重复 1000 次 | 报**最坏**耗时（不是平均） |

> ⚠️ **第 3 行的「姿态响应符号」是整个项目最容易搞错的一步。** 在 FLU/ENU 下，要往 +x（东）飞，需要绕 **−y** 轴倾斜……还是 +y？**不要靠记忆，推导或者用已知正确的仿真器验证。** px4-mpc 的输出侧翻转表（§2.3c）已经给了正确自洽的一组，照抄即可，但要**理解而不是盲抄**。

### S4 —— MPC 节点骨架

按 §2.6 复制 `px4-mpc`，按 §2.5 的 M1–M9 改造。**必须同时具备**：

- [ ] 订阅 `/mocap/pose_enu`（**不是** NED）
- [ ] 订阅 `/fmu/out/vehicle_status`（判断是否已 armed / offboard）
- [ ] 发布 `/fmu/in/offboard_control_mode`（`body_rate = True`，其余 False），**≥ 2 Hz，实做 50 Hz**
- [ ] 发布 `/fmu/in/vehicle_rates_setpoint`
- [ ] QoS：pub `BEST_EFFORT + TRANSIENT_LOCAL + depth 1`，**sub 也必须 `BEST_EFFORT`**（§6.4）
- [ ] `timestamp` 用 `Clock().now().nanoseconds / 1000`（**PX4 用微秒**）
- [ ] 求解失败 / 非有限值的兜底路径（M5）
- [ ] 100 Hz 定时器（D6 待实测后调）
- [ ] RViz：预测轨迹 `Path` + 参考 `Marker`

### S5 —— SITL 闭环 ★ **Phase 2 的真正关卡**

> ⚠️ **被控对象不是 `gz_x500`（Q9 决策）**：`gz_x500` 只是"能跑起来"的现成平台，
> **SITL 要重新生成一份符合 z0 的模型** —— 按 Q11 用**故意失配**的参数（质量 / `kt` / 转动惯量 / τ 按单参数 ±30% 扫），
> 并保留一份**同参数变体**作为对照；按 Q13 用 **Phase 1 的真实动捕 rosbag 回放**驱动，
> 地面真值重映射那份作为对照。`gz_x500` 仅可作为**最早期**的冒烟目标。

在（z0 参数的）SITL 被控对象上跑通完整链路。**SITL 里 PX4 状态话题可用，所以这一步有两个可选的输入源**：

| 输入源 | 用途 |
|---|---|
| `/fmu/out/vehicle_local_position` + `vehicle_attitude` | **先跑通 A 档**：验证 MPC→PX4 这条链、符号、推力标定 |
| 模拟的 `/mocap/pose_enu`（可以用 SITL 真值 + 人为延迟/噪声伪造） | **再跑通 B 档**：验证动捕输入这条链 |

> **★ 为什么分 A/B 两档：** 如果一上来就用伪造的动捕，符号错了你分不清是「MPC 模型错」还是「ENU/NED 转换错」。先 A 后 B，**每次只有一个新变量**（这是本项目的指导思想，README 里也写了）。

**S5 验收判据**（缺一不可）：

1. 起飞到 1 m 悬停，**稳态位置误差 < 0.15 m**，无持续振荡。
2. 走完 Phase 1 的三条轨迹（悬停 / 1 m 正方形 / r=0.5 m 八字），**不撞地、不失控**。
3. 人为注入 50 ms 延迟后**仍稳定**（可以变差，但不能发散）。
4. 人为让 `solve()` 返回非零，**飞机保持安全悬停而不是崩**（验证 M5 兜底）。
5. **实测最坏求解耗时** ← **这个数字直接决定 D6。**

### S6 —— 实机步骤 1（10 状态模型）

**前置：Phase 1 已完全达标**，且 §S5 五项全过。

**台架预检**（不装机、不通电桨）：

- [ ] `T_hover` 标定：从 Phase 1 的悬停日志里取 PX4 实际输出的归一化推力（`/fmu/out/vehicle_thrust_setpoint` 或 mavros 侧等效量），反推模型里的 `hover_thrust`。**这是 M1 的实测输入。**
- [ ] `ω_max` 与 `z0.params` 的 `MC_ROLLRATE_MAX` / `MC_PITCHRATE_MAX` / `MC_YAWRATE_MAX` 逐项对齐（M3）。
- [ ] 推重比复核：地面拉力台或 Phase 1 悬停时的油门裕度，验证 `T_max ≈ 11.8 N`（§3.3）。

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
| **求解器兜底** | 连续 N 次求解失败 → 切悬停推力 + 零角速率，并置标志 |
| **数值守卫** | 发布前 `np.isfinite()` 全量校验（坑 3） |
| **回退到 Phase 1** | **两个节点都在跑，只是不发布** —— 一个 `ros2 param set` 就能切回 `(A)` 级 |

---

## 5. 待实测参数清单（★ 卡住进度的硬依赖）

> 这些**必须在实机上测**，任何文档都给不出。按依赖顺序排：

| # | 参数 | 怎么测 | 卡住谁 | 有没有替代 |
|---|---|---|---|---|
| P1 | **`hover_thrust`**（模型里） | Phase 1 悬停日志反推 | S6 全部 | 无。**测不准飞不起来** |
| P2 | **`T_max`**（按电池） | 拉力台，或从悬停油门裕度估 | M4、§3.3 | 理论值 11.8 N 可**先顶用** |
| P3 | **最坏求解耗时** | SITL 跑 1000 次取 max | **D6（频率）** | 先按 100 Hz 写 |
| P4 | **`k_ω`** | 角速率阶跃 + IMU 拟合 | S7 | 步骤 1 完全不需要 |
| P5 | **动捕实测延迟** | 已知：**50 ms** | D9 | 已有数 |
| P6 | 遥控器**开始键**（ch9）的实读行程 | `/mavros/rc/in` 看 `channels[8]` | S8 | **不影响飞行，但影响安全**。★ **2026-09-21：通道分配已定 = ch9**（[ticket 02](../../.scratch/phase1/issues/02-rc-channel-decision.md)）—— 本行只剩"把 ch9 的实读行程读出来"，用来校准 `start_threshold`。⚠️ **前置**：先在 EdgeTX 里给 CH9 配一个开关，否则读出来永远是常数 |

> ★ **Q19（2026-09-20）→ ✅ 2026-09-21 resolved**（ticket [02](../../.scratch/phase1/issues/02-rc-channel-decision.md)）：
> **开始 = ch9 上升沿；急停 = ch8 高值（硬件杀车，不新增软急停通道）；解锁 = 程序解锁（触发 = ch9 第 1 次上升沿）。**
> 结论、依据与连带要求见 [`../reference/params-changes.md`](../reference/params-changes.md) **§H**；
> 约束与完整占用表见 [`../reference/params-analysis.md`](../reference/params-analysis.md) §3.15（其中「只剩 ch7」已更正）。

**⚠️ P1 是唯一真正的阻塞项。** 其余都有可用的初值。

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
> ☠️ 那次验收"附带推翻的项目前提"**同样作废**：~~`5ba2e91f` 的 `OffboardControlMode` 与上游 v1.15.0 一致
> ⟹ z0 固件是「v1.15 世代」、「1.14.x」这个标签是错的~~。
> `OffboardControlMode` 带 `thrust_and_torque` 是 **fork 的事实**；真身那一栏是 **`bool actuator`**
> （与上游 v1.14.3 相同）。⟹ **「z0 是 1.14.x」这个标签一直是对的**，G6 已按定案重写。
>
> ⚠️ **本节描述的坑本身仍然真实存在**（`msg/versioned/` 是 **v1.16 才有**，没有任何自动保护）
> —— 只是**我们并没有踩到它**：这一次的"不匹配"是误报。
>
> **→ S1 的第一步不再是"定案怎么得到与 fork 逐字节一致的 `px4_msgs`"，而是
> 「用 `release/1.14` @ `ffb6e80e` + 上游 v1.14.3 跑 SITL，并跑一遍 `check-message-compatibility.py` **复验**」。**
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
> | §3.3 `MC_BAT_SCALE_EN` 默认 **0**（关闭） | ✅ **成立** | `src/modules/mc_rate_control/mc_rate_control_params.c:400` `PARAM_DEFINE_INT32(MC_BAT_SCALE_EN, 0);`（~~原注 `:294` 是 fork 的行号~~）—— **含义**：角速率环**不做**电池电压推力补偿 → `(C)` 级的推力映射**不需要**考虑电压 sag |
> | §6.4 QoS（订阅 `fmu/out/*` 必须 `BEST_EFFORT`） | ✅ **成立** | `src/modules/uxrce_dds_client/dds_topics.h.em:69`（`uxr_prepare_output_stream(session, best_effort_stream_id, …)`，~~原注 `:109`~~）、`utilities.hpp:114-119`（datareader QoS = `VOLATILE` + `BEST_EFFORT` + `KEEP_LAST`，~~原注 `:120-125`~~） |
> | §6.5 时间戳单位微秒 | ✅ **成立**（两条独立证据） | `msg/VehicleRatesSetpoint.msg:1` 逐字含 `(microseconds)`；`dds_topics.h.em:52` 与 `:103` `int64_t time_offset_us = session->time_offset / 1000; // ns -> us`（~~原注 `:91/:146`~~） |
> | §6.1 `px4_msgs` 版本 | ❌ **原判被推翻 → 改判「`release/1.14` 兼容」** | 见上 |
> | S1 SITL 机型可用性 | ✅ **上游 v1.14.3 也有 `4001_gz_x500`** | 见 [`refs-manifest.md`](../reference/refs-manifest.md) §2.6 可行性表（~~原注"fork 63 个"~~ 是对 fork 数的，已不相关） |
>
> **★ 一条仍然成立的范围澄清**：**§2.4 的「五个坑」不需要按真身复验** ——
> 那五条针对的是 **px4-mpc 自己的 Python 代码**（`0.07` 魔数、`raise` 兜底、代价罚 `u−0`、约束范围），
> **与 PX4 固件版本无关**。它们已在 Q25 的那一轮里对着 px4-mpc `master` 核过。
>
> **→ 所有依赖「PX4 版本」的载重结论，现在都建立在真身 `4a0e65f2`（v1.14.3 系）之上。**
> ⚠️ 仍未复验的一项：S1 要用的 **SITL 机型条目数**（v1.14.3 比 fork 少）—— 记在 S1 表里，动手前实测。

### 6.2 `thrust_body` 是 `[-1, 1]`，**不是** `[0,1]`、**不是**牛顿

第一手证据见 §3.3：`_thrust_setpoint = Vector3f(vehicle_rates_setpoint.thrust_body)` 原样透传。
FRD 下 z 朝下 ⇒ **向上推力是负值**。写成 `+F/T_max` 会直接**全速砸地**。

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
| ~~O2~~ | ~~`MPC_TILTMAX_AIR` 45°→30°~~ | ✅ **已关闭** —— 收到 30°（§3.3） | 已定 |
| O3 | D6：MPC 频率 | 待 P3 实测 | 实测数据 |
| O4 | D9：延迟补偿 | 步骤 1 只用一行状态外推；完整补偿留第二版 | 用户 |
| O5 | `R` 改罚 `u − u_ref`（M6）的具体参考量 | 用上一帧解或前馈 | 实现时定 |
| O6 | D5：是否换高 C 电池 | 与本文实施步骤无关，参数化即可 | 用户（硬件决策） |

---

## 附：本文中「第一手」结论的证据索引

| 结论 | 证据 |
|---|---|
| px4-mpc 的 OCP 是 10 状态、ω 不是状态 | `models/multirotor_rate_model.py`，`x = cs.vertcat(p, v, q)`，`nx=10` |
| px4-mpc 内部是 ENU/FLU | 同上，`g=[0,0,-9.81]`、`force=[0,0,F]` |
| 输出侧符号为 `[+ωx, −ωy, −ωz]`，推力取反 | `mpc_quadrotor.py`，`setpoint_msg.roll/pitch/yaw` 与 `thrust_command` |
| `0.07 ≈ 1/F_max` | `multirotor_rate_model.py` `max_thrust = mass*9.81/hover_thrust` ⇒ `1/13.44 = 0.0744` |
| `R` 罚绝对角速率 | `multirotor_rate_mpc.py` `yref` 的 u 部分全零 + `R_mat` |
| `lbu/ubu` 的 yaw 取半 | `multirotor_rate_mpc.py` `-0.5*wmax` / `+0.5*wmax` |
| `thrust_body` 原样透传、不受 `MPC_THR_HOVER` 影响 | PX4 `v1.14.3` `src/modules/mc_rate_control/MulticopterRateControl.cpp`，offboard 分支 `_thrust_setpoint = Vector3f(vehicle_rates_setpoint.thrust_body);` |
| 角速率有 NaN 回退、推力没有 | 同上，`PX4_ISFINITE(...)` 只出现在三行角速率上 |
| `MC_BAT_SCALE_EN` 默认 **0（关闭）** | PX4 `v1.14.3` `src/modules/mc_rate_control/mc_rate_control_params.c`，`PARAM_DEFINE_INT32(MC_BAT_SCALE_EN, 0);` |
| `gz_x500` 在 v1.14.3 存在 | `git ls-tree v1.14.3:ROMFS/px4fmu_common/init.d-posix/airframes` → `4001_gz_x500` |
| `px4_msgs` 有 `release/1.14` 分支与 `v1.14.0` 标签 | `git ls-remote --heads --tags https://github.com/PX4/px4_msgs.git` |

> 📌 **顺带一条对用户有利的发现**：`MC_BAT_SCALE_EN` 默认**关闭**，所以 [`phase1-implementation.md`](phase1-implementation.md) §8.2 里记的 ADC 电池电流采样 bug **在默认配置下不会污染推力/力矩指令**（该参数打开时才会把 `thrust` 和 `torque` 同时乘上由电池测量算出的 `battery_status.scale`）。
> **但如果将来有人图"电量低时性能一致"把 `MC_BAT_SCALE_EN` 打开，这个 ADC bug 就会从"遥测读数不对"升级成"控制环里的乘性干扰"。** 建议在 z0 上**保持该参数为 0**。
