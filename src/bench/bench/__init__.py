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
| `timeline`   | 故障注入时序（第几秒干什么）—— **纯逻辑** | **无** |
| `config`     | 默认频率/话题（= 真 mavros 的实测值），`config/bench.yaml` 是它的副本 | 无 |
| `pytest_support` | pytest 的会话级 ROS 上下文 + domain 隔离（安全边界） | 有 |

★ 它**不**模拟 PX4：OFFBOARD 超时退出、EKF2 融合、模式需求检查都在真飞控/SITL 里，
  夹具只负责"让节点以为自己在跟一台 mavros 说话"。两者的分工见 .scratch/phase1/spec.md。
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
from .scenario import BenchTimeout, Scenario
from .timeline import Event, Timeline
from .vrpn_source import VrpnSource, mocap_qos

__all__ = [
    "ArmingRequest",
    "BenchTimeout",
    "DEFAULTS",
    "Event",
    "FakeMavros",
    "ModeRequest",
    "Scenario",
    "Setpoint",
    "Timeline",
    "VrpnSource",
    "load",
    "mavros_default_qos",
    "mavros_sensor_qos",
    "mavros_state_qos",
    "mocap_qos",
]
