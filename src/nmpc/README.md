# nmpc —— Phase 2 的 MPC 包

把 PX4 的内置 PID 换成我们自己的控制律。控制层级是 **(C) 级 CTBR**：
我们只发「总推力 + 机体角速率」，**姿态环与混控仍留在飞控里**
（理由与备选见 [`docs/control/phase2-implementation.md`](../../docs/control/phase2-implementation.md)）。

> 包名为什么是 `nmpc` 而不是 `mpc` / `ctbr`，见
> [`.scratch/phase2/issues/06`](../../.scratch/phase2/issues/06-mpc-node-fake-px4-fixture.md) 的 Comments。

## 分层（这不是风格问题，是**可测性**问题）

| 模块 | 干什么 | 有没有 ROS |
|---|---|---|
| `nmpc/sign_map.py` | ★ FLU → 下行字段的**唯一一份**实现。☠️ **2026-10-13（票 13 收口轮）**：原写"→ PX4 `VehicleRatesSetpoint`"，**只写了两条路里的一条**；今天有**两个具名入口**：`px4_rates_setpoint`（→ `VehicleRatesSetpoint`，uXRCE-DDS 那条**已废**）与 `mavros_attitude_target`（→ `AttitudeTarget` 的 `body_rate` + `thrust`，**今天在用的那条**，`/mavros/setpoint_raw/attitude`）。口径以 [`nmpc/__init__.py`](nmpc/__init__.py) `:10` 为准 | **无** |
| `nmpc/commands.py` | 参数化指令程序：时间 → `[推力, 角速率]`，无模型 / 无代价 / 无约束 | **无** |
| `nmpc/params.py` | z0 的模型参数（实测值 / 占位值分栏写明） | **无** |
| `nmpc/model.py` | 10 状态四旋翼模型（`ω` 不作状态）＋ 四元数工具 | **无** |
| `nmpc/ocp.py` | ★ 离线 OCP：`(状态, 参考) → u = [F, ωx, ωy, ωz]`（ticket 04 的交付物） | **无** |
| `nmpc/timing.py` | `python3 -m nmpc.timing`：量**最坏**周期耗时（回填 D6） | **无** |
| `nmpc/offboard_smoke.py` | ticket 05 的最小 offboard 冒烟节点（把 `sign_map` + `commands` 接到话题上） | 有 |

**上表标「无」的那几个模块**（核心层）不许 import ROS，也不许 import 任何**具体消息包**
（`px4_msgs` / `mavros_msgs` / …）。理由有二：

1. 离线 OCP（ticket 04）与台架夹具（ticket 06 起）都要直接拿它们用，不该被拖去起 DDS；
2. `test/test_sign_map.py` 与 `test/test_ocp_health.py` 里各有一条断言盯着"`rclpy` 没被带进来" ——
   零依赖**只有靠断言才守得住**。

☠️ **原文是"除 `offboard_smoke` 外都不许 import `rclpy` / `px4_msgs`"（2026-10-12 改）** ——
那句话**表述本身已经不准**：有 ROS 的不止 `offboard_smoke`，`mpc_node.py` 也是入口脚本、
同样 import `rclpy` 与消息包，它**从来就在那条约束之外**。**原意不变**（核心层不依赖具体消息包），
改的只是把"点名两个包 / 点名一个文件"换成"点名那件事、点名那些模块"。
★ **准确度补一句**（本票发现，账在 [ticket 13](../../.scratch/phase4/issues/13-mavros-downlink-switch.md) 的 Comments）：
上面那两条断言**只盯 `rclpy` 与 `px4_msgs` 这两个名字**，**没有一条**盯 `mavros_msgs` ⟹
"不许 import 任何具体消息包"目前是**写在这里的口径**，不是被断言守住的事实。

★ **`ocp` 与 `sign_map` 的分界不许模糊**：`ocp` 出的 `u` 是 **MPC 内部的 FLU 量**；
把它变成下行字段是 `sign_map` 的事。`ocp.py` 里**没有一行**涉及那些字段名 —— 混在一起，
"符号错了"和"模型解错了"就再也分不开了。

☠️ **2026-10-13（票 13 收口轮）**：上面那个括注（"`VehicleRatesSetpoint`（roll 不变 / pitch 取反 / yaw 取反 /
`thrust_body` 归一化）"）**只描述了已废的那条路**。今天在用的是 `mavros_attitude_target`，
它**一个字都不转**（FLU 原样交出去，`body_rate` 的 FLU→FRD 由 mavros 做），
归一化写成 **`thrust = F / T_max`（正号 `0…1`，负号由飞控加）**。
⟹ **两条路的字段名与符号规则不同**，细节见 [`docs/control/sign-map.md`](../../docs/control/sign-map.md) §2.2 / §2.3
与 [`mpc-node-seam1.md`](../../docs/control/mpc-node-seam1.md) §N1。⛔ 别把这两套规则混着读。

## ★ 数值权威不在本包里

符号与归一化的权威是 [`docs/control/sign-map.md`](../../docs/control/sign-map.md) §2，
代码侧契约在同文件 §8。`sign_map.py` 里那句 `# ↓↓↓ 这四行就是 …§2 的表 ↓↓↓` 是**指针**，
不是副本 —— 改表就要同时改那里，两边一起动（§8.4 的防漂移要求）。

## 构建

`mavros_msgs` **不在本仓库里**（在下位工作区 `~/mavros_ws`，Phase 1 起就在用），所以先 source 它：

```bash
source ~/mavros_ws/install/setup.bash   # 仓库外依赖，裸 colcon build 会找不到 mavros_msgs
colcon build --packages-up-to nmpc
colcon test  --packages-select nmpc && colcon test-result --verbose
```

☠️ **这一段 2026-10-12 换过**（ticket 13 §二⑥）。原文是
"`px4_msgs` 不在本仓库里（在下位工作区 `~/px4_msgs_ws`），所以先 source 它：
`source ~/px4_msgs_ws/install/setup.bash`"——**构建 `nmpc` 本身已不再需要它**
（`package.xml` 里那条 `<depend>px4_msgs</depend>` 已撤，见那里的 ☠️）。
⚠️ **但下面这条测试命令仍要 source `~/px4_msgs_ws`**：`test/test_mpc_node.py` 现在还
`from px4_msgs.msg import VehicleStatus`（seam 1 的夹具还是假 PX4）⟹ 完整的两行是
`source ~/mavros_ws/install/setup.bash && source ~/px4_msgs_ws/install/setup.bash`。
那份夹具的去留归 ticket 13 §四，**它落地之前别把 `~/px4_msgs_ws` 那一行删掉**。

## 跑一下（SITL 冒烟，详见交付文档）

```bash
ros2 run nmpc offboard_smoke.py --phases lift:1.2:1.2:0,0,0 --phases hover:8:1.0:0,0,0 \
    --log-file /tmp/smoke.jsonl
python3 src/nmpc/nmpc/offboard_smoke.py --help    # 不经 colcon 也能跑
```

场景、判据与第一手证据见 [`docs/control/sitl-offboard-smoke.md`](../../docs/control/sitl-offboard-smoke.md)。

## 参考源：`hold` / `traj` / `control`（[票 15](../../.scratch/phase4/issues/15-trajectory-flight-roles.md)）

`-p reference_source:=…` 决定**目标位置从哪来**，三档互斥：

| 档 | 参考从哪来 | 加不加锚点 | 谁在用 |
|---|---|---|---|
| `hold`（默认） | `hold_offset`（参数化的固定偏移） | 加 | 台架 / 冒烟 |
| `traj` | `/traj/reference`（相对量） | 加 | 旧独立档：菜单 9 的两档都**不再选它**；只有单独跑 `run/z0.py mpc` 时仍可手动打开（`reference_source_traj`） |
| **`control`** | **`/control/target`（绝对量，位置 + 速度 + yaw）** | **不加** | `z0.py` 菜单 9 第 2 档 —— `control` 是任务总管，`mpc` 只做输出后端 |

`control` 档的口径（`node_core.py` 的 `absolute_reference` / `on_control_target`）：

- `/control/target` 是 **ENU / FLU 数值**（与仓库里 `/traj/reference` 一样，`coordinate_frame`
  字段只是占位，**不做 NED 转换**）；`type_mask` 必须是 **2496**（位置 + 速度 + yaw，
  忽略加速度与 yaw_rate），与 `control` 侧同一个常量。
- **目标就是绝对值**：`x_ref = 目标位置`、速度按目标速度线性外推（MPC 的前馈）。
  **不再加锚点** —— 轨迹原点、起飞悬停点、判稳全在 `control` 里算好了。
- 目标断流 **0.5 s**（`control_target_timeout_s`）⟹ 闸门以 `no_control_target` 关闭，
  **不是冻结**：冻结等于拿旧目标继续飞，而目标流断掉通常意味着上游任务态变了。
  **关闸 = 零帧**（`_hold` 返回 `publish=False`），不是"阶梯接管"：`hold → hover` 那条
  兜底阶梯只处理**求解不可用**，不在这条路上。最后一道是 PX4 的 1 s OFFBOARD 丢失保护。
- `_validate` 在 `control` 档下要求 `hold_offset` 全 0、拒绝 `start_traj_reference`
  （后者只在 `reference_source=traj` 有意义）。

★ **`mpc` 仍然不 arm、不 set_mode、不读遥控器**：开始键、解锁、切 `OFFBOARD`、起飞、
`/traj/start`、降落全部由 `control` 做。第 2 档的链是 `traj → mpc → control`，
`control` 以 `output_mode=external` 运行（只发 `/control/target`，见
[`src/control/README.md`](../control/README.md)）。

## `mpc_node.py` 的两条新参数（[票 14](../../.scratch/phase4/issues/14-chain-controller-choice.md) 二定，**默认都关**）

两条都是 2026-10-16 加的，`_PARAMS` 里默认值均为 `False` ⟹ **不传参数时行为与今天逐字一致**
（"默认关"的活证据是既有用例照旧绿，尤其 `test_未解锁时一帧_setpoint_都没有` —— 不是口头保证）。
★ 票 15 起 `z0.py` 的 MPC 档只打开 `offboard_prestream`；`start_traj_reference` 不再由
`z0.py` 打开（`/traj/start` 归 `control`），但仍保留给 `reference_source=traj` 的旧链：

| 参数 | 默认 | 打开后它做什么 | 谁打开 |
|---|---|---|---|
| `offboard_prestream` | `false` | 闸门关着时，**只有** `not_armed` / `not_offboard` 这两个原因下发**一帧纯悬停**（零角速率 + 悬停推力，就是 `__init__` 里那个 `_hover_sp`）；其余关闸原因**照旧零帧** | `z0.py` 的 MPC 档 ⟹ `--ros-args -p offboard_prestream:=true` |
| `start_traj_reference` | `false` | 参考源是 `traj` 时，闸门因 `no_traj_reference` 关着那一拍发**一帧** `/traj/start`（`std_msgs/Empty`），**发一次**；理由回到 `no_status` / `not_armed` / `not_offboard` 才重新武装 | **只给旧 `traj` 档手打**；`z0.py` 不再打开它（`control` 档下 `_validate` 直接拒） |

★ **`offboard_prestream` 的权限边界**（第一手，PX4 1.14.3 `mavlink_receiver.cpp`）：不在 `OFFBOARD` 时
飞控**只**消费这帧里的 `offboard_control_mode`（`:1530-1536`，`:1535` 是那句 `publish`），
姿态/角速率那两处都写着 "Publish … only once in `OFFBOARD`"（`:1562` / `:1592`）
⟹ 预流**动不了飞机**，它唯一的权限是"让飞控肯收下我们进 `OFFBOARD` / 解锁"
（`COM_OF_LOSS_T` 要求进 `OFFBOARD` 前 1 s 内持续有 setpoint）。

⛔ **别把预流那张名单放宽**（`REASONS_PRESTREAM`）：其余每个关闸原因（`status_stale` / `no_mocap` /
`stamp_regress` / `pose_jump` / `attitude_jump` / `no_traj_reference` …）都意味着"**输入不可信**"，
那时**停流**、让 PX4 在 1 s 后把飞机踢出 `OFFBOARD`，正是我们要保住的那道兜底。
`no_status` 也不在名单里（连"解锁没有、什么模式"都不知道，发帧是盲发）。预流帧在 JSONL 里的
`fallback` 是 `"prestream"` —— 它**不是**兜底阶梯的一档（阶梯是 `normal` / `hold` / `hover`）。

★ `start_traj_reference` **只在 `reference_source=traj` 时有意义**（触发原因 `no_traj_reference`
在 `hold` 档下永远不会出现）⟹ `_validate` 在 `hold` 档下**当场拒**，不让它静静地待在那里
（与 `traj` 档拒 `hold_offset` 同一条纪律）。

⚠️ **两条都只发 setpoint，不 arm、不 set_mode、不调服务。** 在 `z0.py` 第 2 档里，这两件事由
`control` 的 FSM 做（开始键第 1 次上升沿）；`mpc` 自己永远不碰。`run/z0.py` 里不许出现
arming / set_mode 入口（`FORBIDDEN_IN_COMMANDS` 挡的正是 `ros2 service call`）。

## 离线 OCP（ticket 04，零 ROS）

```python
from nmpc import OfflineOcp

ocp = OfflineOcp()
sol = ocp.solve(ocp.hover_state(), x_ref)   # x_ref: (N+1, 10)
print(sol.u)                                # [F, ωx, ωy, ωz] —— 只取第一拍，那才是要下发的
print(sol.info.line())                      # 数值健康度
```

需要 acados 的两个环境变量（构建方式见 [`docs/control/acados-orin-build.md`](../../docs/control/acados-orin-build.md)）：

```bash
export ACADOS_SOURCE_DIR=$HOME/acados
export LD_LIBRARY_PATH=$ACADOS_SOURCE_DIR/lib:$LD_LIBRARY_PATH
```

★★ **两个缺一不可，且失败长得像"代码坏了"**（2026-10-17 起链实测，票 14）：
缺 `ACADOS_SOURCE_DIR` ⟹ `acados_template` 从**它自己被装在哪**去猜（本机 = `~/refs/acados`，
那是**源码树**、没有 `lib/`）⟹ `OSError: …/libacados.so: cannot open shared object file`；
只设它、缺 `LD_LIBRARY_PATH` ⟹ `libacados.so` 打得开、但它依赖的 `libhpipm.so` 找不到
（同一个 `OSError`，换个文件名）。两次都发生在 `mpc_node.py` 起来的**头一秒**。
★ **走 `run/z0.py` 起链时不用你 export**：`mpc` 那条动作的命令自带 `env ACADOS_SOURCE_DIR=…
LD_LIBRARY_PATH=…` 前缀（见 [`run/README.md`](../../run/README.md)）—— 上面这两行是给
**手打 `ros2 run nmpc mpc_node.py`** 与**直接跑 pytest** 用的。

第一次 `OfflineOcp()` 会**代码生成 + 编译 C**（几十秒）；之后复用 `$MPC_CODEGEN_DIR`
（缺省 `~/.cache/mpc-ocp-codegen`，**刻意不放进源码树**）。不带 ROS 直接跑测试：

```bash
cd src/nmpc && python3 -m pytest test/test_ocp_*.py test/conftest.py -q
```

### 耗时：报**最坏**，不报平均

```bash
python3 -m nmpc.timing --runs 1000 --cpu 4
```

这个数字直接决定控制频率（D6）。两条**第一手**的前提，缺了读数就是假的：

1. **绑核**：`--cpu 4` 固定到一个核上，实测中位数 −29 %、最大值 −33 %。
2. **机器必须空载**：这台机器有并行会话，别人一编译，同一个命令的 max 会从 4.4 ms 涨到 23 ms，
   判定行直接翻成"100 Hz 不够"的**假**结论。脚本自己会报 `load1` 并在有负载时给判定行打失真标记。

判据分两列，别混（实测：墙钟 max 54 ms 的那一拍，acados 自报只有 3.4 ms —— 差值是被抢占）：

| 列 | 管什么 | 实测（绑核 cpu4，1000 次，**三轮取最坏**） |
|---|---|---|
| 整周期墙钟 | **能不能闭环** —— D6 就是问这个 | median 3.91–3.95 / p99 4.66–4.68 ms，**max 54–56 ms** ⚠️ |
| acados 自报 | **求解器有没有退化** —— 与 Python 侧无关 | median 2.68 / max **3.58** ms |

三轮逐轮的原始数与"哪个数出自哪一轮"，见
[`docs/control/offline-ocp.md`](../../docs/control/offline-ocp.md) §5 的复测表。
⚠️ 墙钟那一列的 max **不是求解开销**（同一拍自报 3.4 ms）；它越过了 D6 的 >20 ms 线，
要闭环在 100 Hz 上得靠节点侧绑核/定优先级 —— 归 ticket 06/07。

### 数值健康度：`sol.info`

⚠️ **`status == 0` 单独不足以说明健康** —— `SQP_RTI` 每周期只做一次迭代，解还很差时也返回 0。
一起看：状态码（`status` / `qp_stat`）、迭代数（`sqp_iter` / `qp_iter`）、自算的替代残差
（`u_bound_violation` / `quat_norm_drift`），总开关是 `info.degraded`。

四条**实测**的坑，别照抄普通 SQP 的读法：

| 读数 | 实测行为 | 怎么办 |
|---|---|---|
| `get_stats("residuals")` | `SQP_RTI` 下**恒为全 0** | 别当健康依据；字段保留只为"看得到它确实是 0" |
| `get_stats("alpha")` | 抛 `ValueError: alpha values are not available for SQP_RTI` | 不读 |
| `solver.get_cost()` | 同一次求解：读数 516.07，按定义重算 49501.7（**差 96 倍**），也不是"只算了某一段"（第一步就已 1220.25） | 本包**自己算** `info.cost`；`info.cost_acados` 只把原始读数留证 |
| 第一帧 | 全新实例的第 0 次 solve 拿不到像样的解（+x 10 m 时 ω_y = 0、F = 7.81，跟悬停一样）—— 单次 RTI 迭代需要上一帧做起点 | 测试先热身（`test/conftest.py` 的 `solve_warm`）；节点侧归 ticket 06/07 判断 |

### 参数：哪些是实测、哪些是占位

见 [`nmpc/params.py`](nmpc/params.py) 的模块 docstring。一句话：`mass` 是实测；
`hover_thrust` / `t_max` / 三轴角速率上限是**占位值**，且 `t_max` 由 `hover_thrust` **导出** ——
两者不自洽就是一个**静默的推力增益错误**（PX4 对 `thrust_body` 不做任何悬停油门重标定）。
回填口 = [ticket 10](../../.scratch/phase2/issues/10-param-measure-backfill.md)。
