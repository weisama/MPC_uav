"""MPC（模型预测控制）内部量 → PX4 `VehicleRatesSetpoint` 的**唯一一处实现**。

★ **数值权威不在本文件**，在 [`docs/control/sign-map.md`](../../../docs/control/sign-map.md) §2
（该文件 §8 是代码侧契约）。本模块只把内部量转成 PX4 的四个字段，
**不在注释或 docstring 里复述那张表的数值** —— 那是该文件 §8.4 的防漂移要求。

**为什么是纯函数、且不 import ROS**（该文件 §8.1）：离线 OCP（ticket 04）与假 PX4 夹具
（ticket 06）都要能直接用它，不必起节点。所以这里只 import 标准库。

**输入 / 输出**（单位与坐标系都写死在这里，不要靠调用方猜）：

- `omega_flu`：机体相对惯性系的角速度，分量按**机体系 FLU** 读，单位 **rad/s**，顺序 `[x, y, z]`。
- `thrust_n`：**总推力**，单位 **N**，**沿机体 +z（FLU）**，`>= 0`（推力只能往上，不能当刹车用）。
- `t_max_n`：推力上限，单位 **N**，按**电池**取值而非按电机（理由见那份文档 §6）。
- 返回：PX4 的四个字段 —— `roll` / `pitch` / `yaw`（rad/s）与 `thrust_body[2]`（无量纲）。
"""

from __future__ import annotations

import math
from dataclasses import dataclass

__all__ = ["Px4RatesSetpoint", "px4_rates_setpoint"]


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
    if not math.isfinite(t_max_n) or t_max_n <= 0.0:
        raise ValueError(f"T_max 必须是正的有限值，收到 {t_max_n!r}")
    if not math.isfinite(thrust_n) or thrust_n < 0.0:
        raise ValueError(f"推力必须是非负的有限值（沿机体 +z），收到 {thrust_n!r}")

    omega = tuple(float(w) for w in omega_flu)
    if len(omega) != 3 or not all(math.isfinite(w) for w in omega):
        raise ValueError(f"角速率必须是三个有限值 [x, y, z]（FLU, rad/s），收到 {omega_flu!r}")

    return Px4RatesSetpoint(
        # ↓↓↓ 这四行就是 docs/control/sign-map.md §2 的表 ↓↓↓
        roll=omega[0],
        pitch=-omega[1],
        yaw=-omega[2],
        thrust_body_z=-thrust_n / t_max_n,
    )
