"""夹具的默认参数 —— 与真 mavros 的实测值对齐，改这里等于改所有场景。

`config/bench.yaml` 是这份 DEFAULTS 的可编辑副本（测试里传 `config_file=` 用它，
不传就用代码里的默认值）。**两份保持一致**，改一处要同时改另一处。
"""

from __future__ import annotations

import copy
from pathlib import Path
from typing import Any, Dict, Optional

import yaml

# ★ 频率这一栏是"照抄真 mavros"，不是"随手选一个好测的数"：
#   · state 1.0 Hz  —— 实测 1.000 Hz。**不能调快**：调快等于给 DUT 一个真机上没有的灵敏度，
#                      会让"解锁上升沿最多晚 1 s"这类真问题在测试里消失。
#   · local_position / velocity_local 30 Hz —— 真 mavros 的 local_position 插件是 30 Hz 级。
#   · vrpn 200 Hz   —— 动捕实测 ~196 Hz（ticket 12 的读数：190.9–197.0 Hz）。
#   · estimator_status 1.0 Hz —— 实测 1.000 Hz（2026-09-22 z0 读数，ticket 16）。
#     ⚠️ 与 DUT 的 `ekf2_pos_lost_samples`（帧数！）是一对：1 Hz 下 2 帧 = 2 s。
DEFAULTS: Dict[str, Any] = {
    "rates_hz": {
        "state": 1.0,
        "local_position": 30.0,
        "local_velocity": 30.0,
        "rc": 10.0,
        "battery": 1.0,
        "estimator_status": 1.0,
        "vrpn": 200.0,
    },
    "topics": {
        "state": "/mavros/state",
        "local_position": "/mavros/local_position/pose",
        "local_velocity": "/mavros/local_position/velocity_local",
        "rc": "/mavros/rc/in",
        "battery": "/mavros/battery",
        # ticket 16：EKF2 位置失效的载体（MAVLink ESTIMATOR_STATUS）。
        "estimator_status": "/mavros/estimator_status",
        "setpoint": "/mavros/setpoint_raw/local",
        "set_mode": "/mavros/set_mode",
        "arming": "/mavros/cmd/arming",
    },
    "initial": {
        # 假 EKF 的位置（**ROS/ENU，z 向上为正** —— 它进的是 `/mavros/local_position/pose`，
        # 那条话题在 mavros 出向已转过 NED→ENU）。默认在原点 ——
        # 台架上的飞机就在地上，"原点"是最不该引起歧义的值；
        # 需要辨认坐标轴时由测试显式 `set_position(...)` 搬到三个轴都不同的位置。
        "position": [0.0, 0.0, 0.0],
        "connected": True,
        "armed": False,
        "mode": "POSCTL",
        # ★ 25.0 V = 6S 满电附近（4.17 V/cell）。这个默认值必须**明显高于** DUT 的
        #   `low_voltage_v`（默认 20.0 V），否则每个用到 armed 的场景都会立刻降级到
        #   `AUTO.LAND`/停发 —— 而"低压"是那些测试**没打算**注入的东西（注入它请显式
        #   调 `voltage_drop(...)`）。⚠️ 改 DUT 阈值时回头看一眼这里。
        "voltage": 25.0,
    },
    "mocap": {
        # 'mocap' = 直接发 /mocap/pose_enu（测 control 时用，旁路 mocap 节点）
        # 'vrpn'  = 发 /vrpn/<刚体>/pose（测 mocap 节点、或测整链时用）
        "target": "mocap",
        "body_name": "z0",
        "position": [0.0, 0.0, 0.0],
    },
    "scenario": {
        # 起夹具时是否同时起 DUT（False = 只要夹具，"我想先看看假 mavros 自己发的对不对"时用）
        "start_dut": True,
        # 要不要起脚本化动捕源（测的节点不依赖动捕时可以关掉）
        "mocap": True,
        # ★ 要不要起**假 PX4**（Phase 2 的 `/fmu/*` 那一半）。默认**关** ——
        #   Phase 1 的 `(A)` 链不需要它，而多起一份"外部世界"会让场景在测哪条链变得含糊。
        #   ⚠️ 一份夹具里 `FakeMavros` 与 `FakePx4` **可以同时开**（Phase 2 的"回退"验收项
        #   要的就是"两条链并存"），但它们发的是**不同的话题族**（`/mavros/*` vs `/fmu/*`），
        #   不会像"两份假 mavros"那样互相打架。
        "px4": False,
    },
    # 假 PX4 的初始状态与话题名（`bench/fake_px4.py` 的 `OUT_TOPICS` / `IN_TOPICS`）。
    "px4": {
        # 假 PX4 报的 armed / nav_state。默认 **未解锁 + POSCTL** —— 真机上"上电后、
        # 切 OFFBOARD 之前"的样子；要测"未解锁不发"这类判据就必须从这里开始。
        "armed": False,
        # 2 = POSCTL（`VehicleStatus.msg` 的 NAVIGATION_STATE_POSCTL，逐字一致）
        "nav_state": 2,
        "pre_flight_checks_pass": True,
        # 话题名照抄 PX4 的 uXRCE-DDS 命名（`utilities.hpp` 的 `rt/fmu/{in,out}/<名>`，
        # ROS 2 侧去掉 `rt/` 前缀）。不要为了"测试方便"改名。
        "topics": {
            "status": "/fmu/out/vehicle_status",
            "rates": "/fmu/in/vehicle_rates_setpoint",
            "mode": "/fmu/in/offboard_control_mode",
        },
    },
}


def load(path: Optional[str] = None) -> Dict[str, Any]:
    """读 `bench.yaml` 并与 DEFAULTS 深合并（yaml 里只写要改的键）。

    合并而不是替换：这样 yaml 里可以只写 `rates_hz: {state: 1.0}`，
    其余键照旧 —— 一份只写"我关心什么"的配置，比抄一整份更容易读出差在哪。
    """
    cfg = copy.deepcopy(DEFAULTS)
    if path is None:
        return cfg

    data = yaml.safe_load(Path(path).read_text(encoding="utf-8")) or {}
    _deep_update(cfg, data)
    return cfg


def default_config_file() -> Optional[str]:
    """安装后的 `share/bench/config/bench.yaml`（找不到就返回 None，用代码默认值）。"""
    try:
        from ament_index_python.packages import get_package_share_directory
    except ImportError:
        return None
    try:
        return str(Path(get_package_share_directory("bench")) / "config" / "bench.yaml")
    except Exception:
        return None


def _deep_update(base: Dict[str, Any], override: Dict[str, Any]) -> None:
    for key, value in override.items():
        if isinstance(value, dict) and isinstance(base.get(key), dict):
            _deep_update(base[key], value)
        else:
            base[key] = value
