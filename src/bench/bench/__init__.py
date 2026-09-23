"""台架夹具 —— 本项目**唯一**的节点级测试底座（seam 1）。

    from bench import Scenario, timeline

    with Scenario(dut=("control", "control_node")) as sc:
        sc.wait_for_subscribers("state")
        sc.hold(3.0)
        assert len(sc.setpoints()) > 0

模块分工：

| 模块 | 干什么 | 有没有 ROS |
|---|---|---|
| `scenario`   | 把下面几件拼成一个 `with`：起 DUT 子进程、推进时间、收集证据 | 有 |
| `fake_mavros`| 假 mavros：上游话题 + set_mode/arming 服务 + 记录 DUT 的 setpoint | 有 |
| `vrpn_source`| 脚本化动捕：`/vrpn/<刚体>/pose` 或 `/mocap/pose_enu` | 有 |
| `fake_traj`  | 假 `traj` 节点：`/traj/reference` + `/traj/progress` + `/traj/start`（ticket 09） | 有 |
| `plant`      | 台架上的"假飞机"：把 DUT 的 setpoint 变成假 EKF 报出来的位置与速度 | 间接 |
| `timeline`   | 故障注入时序（第几秒干什么）—— **纯逻辑** | **无** |
| `config`     | 默认频率/话题（= 真 mavros 的实测值），`config/bench.yaml` 是它的副本 | 无 |
| `pytest_support` | pytest 的会话级 ROS 上下文 + domain 隔离（安全边界） | 有 |

★ 它**不**模拟 PX4：OFFBOARD 超时退出、EKF2 融合、模式需求检查都在真飞控/SITL 里，
  夹具只负责"让节点以为自己在跟一台 mavros 说话"。两者的分工见 .scratch/phase1/spec.md。

★ 它的**两半**分得很清楚：`scenario` / `fake_mavros` / `vrpn_source` 是"外部世界"
  （飞控 + 动捕，由 `Scenario` 自动起）；`fake_traj` 则是**被 DUT 指挥的同伴节点**
  （`control` 给它发 `/traj/start`），所以由用例自己起、自己配 —— 与
  "traj 不在时 `control` 要怎么活"（`degrade=traj_timeout`）是同一件事的两面。
⚠️ `plant` 是"间接"：它自己**不** import ROS、也不建话题，但它的每次 `step()` 都要读
  `sc.fake.setpoints`、写 `sc.fake.set_position/velocity` ⟹ 仍然需要一个活着的 `Scenario`。
"""

from .config import DEFAULTS, load
from .fake_mavros import (
    ArmingRequest,
    FakeMavros,
    ModeRequest,
    Setpoint,
    mavros_default_qos,
    mavros_sensor_qos,
    mavros_state_qos,
)
from .fake_traj import FakeTraj
from .plant import Plant, fly, fly_until
from .scenario import BenchTimeout, Scenario
from .timeline import Event, Timeline
from .vrpn_source import VrpnSource, mocap_qos

__all__ = [
    "ArmingRequest",
    "BenchTimeout",
    "DEFAULTS",
    "Event",
    "FakeMavros",
    "FakeTraj",
    "ModeRequest",
    "Plant",
    "Scenario",
    "Setpoint",
    "Timeline",
    "VrpnSource",
    "fly",
    "fly_until",
    "load",
    "mavros_default_qos",
    "mavros_sensor_qos",
    "mavros_state_qos",
    "mocap_qos",
]
