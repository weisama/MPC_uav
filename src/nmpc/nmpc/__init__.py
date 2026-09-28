"""`nmpc` —— Phase 2 的 MPC（模型预测控制）包：把飞控的内置 PID 换成我们自己的控制律。

> 包名为什么是 `nmpc` 而不是 `mpc`，见
> [`.scratch/phase2/issues/06`](../../../.scratch/phase2/issues/06-mpc-node-fake-px4-fixture.md) 的 Comments。

模块分工（**按"有没有 ROS"分层，这是刻意的**）：

| 模块 | 干什么 | 有没有 ROS |
|---|---|---|
| `sign_map`    | ★ FLU → PX4 `VehicleRatesSetpoint` 的**唯一一份**映射实现（纯函数） | **无** |
| `commands`    | 参数化指令程序：时间 → `[推力, 角速率]`，无模型 / 无代价 / 无约束 | **无** |
| `params`      | z0 的模型参数（实测值 + 占位值，两者分栏写明） | **无** |
| `model`       | 10 状态四旋翼模型（`ω` 不作状态）＋ 四元数工具 | **无** |
| `ocp`         | ★ 离线 OCP：`(状态, 参考) → u = [F, ωx, ωy, ωz]`（ticket 04 的交付物） | **无** |
| `timing`      | `python3 -m nmpc.timing`：量**最坏**周期耗时（回填 D6） | **无** |
| `offboard_smoke` | ticket 05 的最小 offboard 冒烟节点（把 `sign_map` + `commands` 接到话题上） | 有 |

★ 除 `offboard_smoke` 外**都不许** import `rclpy` / `px4_msgs`：离线 OCP 与假 PX4 夹具都要能
直接拿它们用，而"能 import"这件事只有靠零 ROS 依赖才守得住（`test/test_sign_map.py` 与
`test/test_ocp_health.py` 里各有一条断言盯着它）。

★ 分层不是洁癖：`ocp` 出的 `u` 是 **MPC（模型预测控制）内部的 FLU 量**，把它变成 PX4 的 `VehicleRatesSetpoint`
是 `sign_map` 的事 —— **两件事不许混在一个模块里**（否则"符号错了"和"模型解错了"再也分不开）。

符号与归一化的**数值权威不在本包里**，在 [`docs/control/sign-map.md`](../../../docs/control/sign-map.md)。

---

离线 OCP 的用法（零 ROS）：

    from nmpc import OfflineOcp

    ocp = OfflineOcp()
    sol = ocp.solve(x0, x_ref)      # u = [F, ωx, ωy, ωz]，ENU/FLU
    print(sol.info.line())          # 数值健康度
"""

from .model import (NU, NX, Z0MultirotorRateModel, normalize_quat, quat_from_rotvec,
                    quat_mul, yaw_quat)
from .ocp import DEFAULT_CODEGEN_DIR, OcpConfig, OcpSolution, OfflineOcp, SolveInfo
from .params import (DEFAULT, G, HOVER_THRUST, PITCH_RATE_MAX, ROLL_RATE_MAX, YAW_RATE_MAX,
                     Z0Params)

__all__ = [
    # 模型与四元数工具（造参考轨迹 / 闭环仿真要用）
    "NU", "NX", "Z0MultirotorRateModel",
    "normalize_quat", "quat_from_rotvec", "quat_mul", "yaw_quat",
    # 离线 OCP
    "OcpConfig", "OcpSolution", "OfflineOcp", "SolveInfo", "DEFAULT_CODEGEN_DIR",
    # 参数
    "DEFAULT", "G", "Z0Params", "HOVER_THRUST",
    "ROLL_RATE_MAX", "PITCH_RATE_MAX", "YAW_RATE_MAX",
]
