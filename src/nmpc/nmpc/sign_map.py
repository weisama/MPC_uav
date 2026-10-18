"""MPC 内部量 → 下行字段的**唯一一处实现**（2026-10-12 起**两个具名入口**）。

两个入口对应两条下行通路（口径见 [`docs/control/sign-map.md`](../../../docs/control/sign-map.md)
**§2.2 / §2.3** —— "谁取负"那张表）：

- [`px4_rates_setpoint`][]: 直接发 `/fmu/in/vehicle_rates_setpoint`（uXRCE-DDS 路径）。
  取负**我们自己**做。
- [`mavros_attitude_target`][]: 发 `/mavros/setpoint_raw/attitude`（mavros 路径）。
  取负**别人**做（mavros 换 FLU→FRD、飞控 `fill_thrust` 加推力负号）⟹ 我们交出去的是
  **原样的 FLU 角速率**与**正号的**归一化推力。

★★ **为什么是两个函数 + 两个返回类型，而不是一个 `path=` 开关**：两条路上"同一个物理量"
落在**名字与符号都不同**的字段上（`thrust_body_z` 是负的、`thrust` 是正的）。
塞进同一个返回类型 ⟹ 字段名开始撒谎，而"字段名自证"正是这条通路切换要挡的那类静默
（[`13`](../../../.scratch/phase4/issues/13-mavros-downlink-switch.md) §二②）。

★ **数值权威不在本文件**，在 [`docs/control/sign-map.md`](../../../docs/control/sign-map.md) §2
（该文件 §8 是代码侧契约）。本模块只把内部量转成 PX4 的四个字段，
**不在注释或 docstring 里复述那张表的数值** —— 那是该文件 §8.4 的防漂移要求。

**为什么是纯函数、且不 import ROS**（该文件 §8.1）：离线 OCP（ticket 04）与假 PX4 夹具
（ticket 06）都要能直接用它，不必起节点。所以这里只 import 标准库。

**输入 / 输出**（单位与坐标系都写死在这里，不要靠调用方猜）：

- `omega_flu`：机体相对惯性系的角速度，分量按**机体系 FLU** 读，单位 **rad/s**，顺序 `[x, y, z]`。
- `thrust_n`：**总推力**，单位 **N**，**沿机体 +z（FLU）**，`>= 0`（推力只能往上，不能当刹车用）。
- `t_max_n`：**归一化**常数，单位 **N** —— ★ 它必须 = **实测的满量程推力**（`m·g/hover_thrust`），
  出自"§3.3.2 的自洽条件"：PX4 对 `thrust_body` 不做重标定，所以"模型里的 `hover_thrust`"与
  "这里的 `1/T_max`"是**同一个物理量**，填别的值悬停前馈就错。
  ⚠️ 它**不是** `F` 上限 —— 那个是另一个量 `Z0Params.f_max`（按电池），**数值不在本文件复述**，
  见 `params.py` 的 `F_MAX_BATTERY`。早先这里写"按电池取值"是**把两个角色混成一个常数**（2026-09-29 订正）。
- 返回：PX4 的四个字段 —— `roll` / `pitch` / `yaw`（rad/s）与 `thrust_body[2]`（无量纲）。
"""

from __future__ import annotations

import math
from dataclasses import dataclass

__all__ = ["MavrosAttitudeTarget", "Px4RatesSetpoint",
           "mavros_attitude_target", "px4_rates_setpoint"]


@dataclass(frozen=True)
class Px4RatesSetpoint:
    """`px4_msgs/msg/VehicleRatesSetpoint` 里我们用到的那四个字段。

    字段名与消息里的名字逐字一致 —— 这样节点里那四行赋值是**抄的**，不是**译的**。
    不 import `px4_msgs`（见模块 docstring）：本类只是个数据结构，不含任何序列化。
    """

    roll: float
    pitch: float
    yaw: float
    thrust_body_z: float


def _checked_inputs(omega_flu, thrust_n, t_max_n):
    """两个入口**共用**的入参检查 —— 返回 `(omega, thrust_n, t_max_n)`（`omega` 已转成 `float`）。

    ★ 两个入口拦的是同样的东西（`T_max` 非正 / 推力为负 / 角速率不是三个有限值），
    所以只有这一处实现：**同一条拒绝条件不许在两个入口里各写一遍**
    （否则改一处、漏一处，而漏掉的那条恰恰是"宁可炸掉也不静默发错"的那一类）。

    ⚠️ 与 `node_core.py` 自己的入参检查是**故意重复**的（守在另一个门口），
    那条理由写在 `node_core.py:504`。
    """
    if not math.isfinite(t_max_n) or t_max_n <= 0.0:
        raise ValueError(f"T_max 必须是正的有限值，收到 {t_max_n!r}")
    if not math.isfinite(thrust_n) or thrust_n < 0.0:
        raise ValueError(f"推力必须是非负的有限值（沿机体 +z），收到 {thrust_n!r}")

    omega = tuple(float(w) for w in omega_flu)
    if len(omega) != 3 or not all(math.isfinite(w) for w in omega):
        raise ValueError(f"角速率必须是三个有限值 [x, y, z]（FLU, rad/s），收到 {omega_flu!r}")
    return omega, thrust_n, t_max_n


def px4_rates_setpoint(omega_flu, thrust_n, t_max_n) -> Px4RatesSetpoint:
    """把 `[ω_x, ω_y, ω_z]`（FLU）与总推力 `F`（N）映射成 PX4 的四个字段。

    符号与量纲见 [`docs/control/sign-map.md`](../../../docs/control/sign-map.md) §2 ——
    **本函数的每一行都在实现那张表**，改表就要改这里，两处必须同时动。

    值域与拒绝条件（都是"宁可炸掉也不静默发错"的那一类）：

    - `t_max_n <= 0` → `ValueError`（归一化常数没有意义；参考实现把 `1/T_max` 硬编码成
      `0.07`，改质量时静默错 33 % —— 那种事不能重演）。
    - `thrust_n < 0` → `ValueError`（推力沿机体 +z，反向推力不是这台飞机的控制量）。
    - 任一项非有限 → `ValueError`。⚠️ **推力那一路必须在这里拦住**：
      PX4 的角速率三轴有 NaN 回退（退回实测角速率），**推力没有**
      （`MulticopterRateControl.cpp:182` 原样透传给混控器）。
    """
    omega, thrust_n, t_max_n = _checked_inputs(omega_flu, thrust_n, t_max_n)

    return Px4RatesSetpoint(
        # ↓↓↓ 这四行就是 docs/control/sign-map.md §2.1 的端到端映射 ↓↓↓
        #     （px4_msgs 路径上"端到端"四个负号全由我们做 ⟹ 这一列 = §2.2 的左列）
        roll=omega[0],
        pitch=-omega[1],
        yaw=-omega[2],
        thrust_body_z=-thrust_n / t_max_n,
    )


@dataclass(frozen=True)
class MavrosAttitudeTarget:
    """`mavros_msgs/msg/AttitudeTarget` 里我们用到的那四个字段（**mavros 路径**）。

    ⛔ **别把它与 `Px4RatesSetpoint` 混用**：`thrust` 与 `thrust_body_z` 是**同一个物理量在两条路上
    反号的两个字段**（`sign-map.md` §2.2），而 `body_rate_*` 是 **FLU 原样**、`roll/pitch/yaw` 是
    **FRD 已取负**（px4_msgs 路径）—— 数值看起来"就差几个负号"，把它当同一个东西用就是 N1/N2 那个静默。

    字段名与消息里的名字逐字一致（`body_rate.x` / `body_rate.y` / `body_rate.z` / `thrust`），
    这样节点里那几行赋值是**抄的**，不是**译的**。`type_mask` **不在这里**：它是消息格式的常量
    （不是符号映射），落在节点里（`offboard_smoke.py`）。
    """

    body_rate_x: float
    body_rate_y: float
    body_rate_z: float
    thrust: float


def mavros_attitude_target(omega_flu, thrust_n, t_max_n) -> MavrosAttitudeTarget:
    """把 `[ω_x, ω_y, ω_z]`（FLU）与总推力 `F`（N）映射成**发往 `/mavros/setpoint_raw/attitude`** 的字段。

    ★★ **本函数的"什么都不做"就是它的正确实现**：交出去的角速率是 **FLU 原样**，
    取负由 mavros 在 `attitude_cb` 里做（`setpoint_raw.cpp:323-324` 的
    `transform_frame_baselink_aircraft`，常量 `ftf_frame_conversions.cpp:40` 的 `R_x(π)`）；
    推力交**正号**，取负由飞控的 `fill_thrust` 做（`mavlink_receiver.cpp:1475`）。
    逐行对照见 [`docs/control/sign-map.md`](../../../docs/control/sign-map.md) **§2.2 / §2.3**。

    ⛔ **不许在这里"顺手转一次 FLU→FRD"**（那是最容易犯、也最看不出来的错）：
    转两次 = 不转 —— **roll 不受影响、pitch/yaw 静默反向**，而飞控最终收到的东西分毫不差。
    复现这一档：`offboard_smoke.py --break pitch-preconvert`。

    值域与拒绝条件：与 [`px4_rates_setpoint`][] **同一个检查器**（`_checked_inputs`）。
    ⚠️ 唯一的差别是**推力那一路在这里多一个上界**：`thrust = F/T_max` 超过 1 就说明归一化常数
    填错了（`T_max` 该是实测满量程）。⛔ **本函数不拦它** —— 拦了就等于把
    `--break thrust-scale` 那一档故障注入变成崩溃（注入的**目的**正是发出一个错量级的推力）。
    ★ 归一化常数"填错了"这件事由**启动自检**当场拦掉：
    [`params.check_hover_consistency`](params.py)（不依赖任何一帧的读数）。
    """
    omega, thrust_n, t_max_n = _checked_inputs(omega_flu, thrust_n, t_max_n)

    return MavrosAttitudeTarget(
        # ↓↓↓ 这三行**没有负号** —— 取负在别人那里（§2.3）↓↓↓
        body_rate_x=omega[0],
        body_rate_y=omega[1],
        body_rate_z=omega[2],
        thrust=thrust_n / t_max_n,
    )
