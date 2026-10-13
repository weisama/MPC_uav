# nmpc —— Phase 2 的 MPC（模型预测控制）包

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

★ **`ocp` 与 `sign_map` 的分界不许模糊**：`ocp` 出的 `u` 是 **MPC（模型预测控制）内部的 FLU 量**；
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
