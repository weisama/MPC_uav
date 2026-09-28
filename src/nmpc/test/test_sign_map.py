"""符号映射实现的断言 —— **对着权威文档那张表**断言，不是对着实现复述的口径。

[`docs/control/sign-map.md`](../../../docs/control/sign-map.md) §8.5 的原话：
「测试断言的是本文件的那张表，不是实现里复述的口径 —— 否则实现错了、测试跟着错，两边一起通过」。

所以这里做**三方对账**（三条路径互不相干，任何一条红了都说明有东西被动过）：

1. **实现**（`nmpc.sign_map`）—— 节点真正用的那一份；
2. **文档**：`sign-map.md` 里的 `sign-map:canonical` 块 —— 数值的唯一权威，测试**解析**它，不复写；
3. **独立推导**：`tools/sign-map/sign_map_core.py` 的 `derived_signs()` ——
   只用「FRD 的 y/z 是 FLU 的 y/z 的反向」这个定义把同一张表**再推一遍**。

⚠️ 本文件**不需要** ROS / 飞机 / SITL：三个模块都是纯函数，`pytest` 一毫秒跑完。
"""

from __future__ import annotations

import math
import sys
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO / "tools" / "sign-map"))  # 复核模块（独立推导 + 权威块解析）

import sign_map_core  # noqa: E402  （tools/sign-map 下的复核库，不是本包的实现）

from nmpc.sign_map import px4_rates_setpoint  # noqa: E402

#: 断言用的样例：(ωx, ωy, ωz)[rad/s]，F[N]，T_max[N] —— 刻意带上 0 与负值
CASES = [
    ((0.0, 0.0, 0.0), 0.0, 11.8),
    ((0.0, 0.0, 0.0), 7.355, 11.8),          # 悬停
    ((0.7, -0.4, 0.25), 9.1, 11.8),
    ((-1.2, 0.0, -0.9), 11.8, 11.8),         # 打满
    ((0.0, 0.5, 0.0), 0.001, 3.0),           # 另一个 T_max（换电池）
]


def _doc_coefficients():
    """权威文档里的四个系数 —— **解析**出来，不在测试里复写一份。"""
    return sign_map_core.parse_canonical_block(REPO / "docs" / "control" / "sign-map.md")


@pytest.mark.parametrize("omega,thrust,t_max", CASES)
def test_matches_the_authoritative_document(omega, thrust, t_max):
    """① 实现 == 文档那张表（四项逐条）。"""
    c = _doc_coefficients()
    sp = px4_rates_setpoint(omega, thrust, t_max)
    assert sp.roll == pytest.approx(c["roll_rate"] * omega[0], abs=1e-12)
    assert sp.pitch == pytest.approx(c["pitch_rate"] * omega[1], abs=1e-12)
    assert sp.yaw == pytest.approx(c["yaw_rate"] * omega[2], abs=1e-12)
    assert sp.thrust_body_z == pytest.approx(c["thrust_body_2"] * thrust / t_max, rel=1e-12, abs=1e-15)


@pytest.mark.parametrize("omega,thrust,t_max", CASES)
def test_matches_the_independent_derivation(omega, thrust, t_max):
    """② 实现 == 独立推导（`tools/sign-map` 那条互不相干的路径）。"""
    d = sign_map_core.derived_signs()
    sp = px4_rates_setpoint(omega, thrust, t_max)
    assert sp.roll == pytest.approx(d["roll_rate"] * omega[0], abs=1e-12)
    assert sp.pitch == pytest.approx(d["pitch_rate"] * omega[1], abs=1e-12)
    assert sp.yaw == pytest.approx(d["yaw_rate"] * omega[2], abs=1e-12)
    assert sp.thrust_body_z == pytest.approx(d["thrust_body_2"] * thrust / t_max, rel=1e-12, abs=1e-15)


def test_the_two_references_agree_with_each_other():
    """③ 文档与独立推导彼此一致 —— 这一条保证"②"不是空转（两边一起错时它才会红）。"""
    assert _doc_coefficients() == sign_map_core.derived_signs()


def test_thrust_is_negative_and_normalized():
    """推力那一项：向上为负、按 T_max 归一化、值域 [−1, 0]。"""
    assert px4_rates_setpoint((0, 0, 0), 0.0, 11.8).thrust_body_z == 0.0
    assert px4_rates_setpoint((0, 0, 0), 11.8, 11.8).thrust_body_z == pytest.approx(-1.0)
    hover = px4_rates_setpoint((0, 0, 0), 0.75 * 9.80665, 11.8).thrust_body_z
    assert -1.0 < hover < 0.0
    # 同一个物理推力，换成"一半的 T_max"⟹ 归一化值的**模**翻倍
    # （这正是参考实现把 1/T_max 硬编码成 0.07 会静默错掉的那件事）
    small = abs(px4_rates_setpoint((0, 0, 0), 3.0, 11.8).thrust_body_z)
    half_t = abs(px4_rates_setpoint((0, 0, 0), 3.0, 5.9).thrust_body_z)
    assert half_t == pytest.approx(2.0 * small)


def test_zero_omega_gives_zero_rates():
    sp = px4_rates_setpoint((0.0, 0.0, 0.0), 7.0, 11.8)
    assert (sp.roll, sp.pitch, sp.yaw) == (0.0, 0.0, 0.0)


@pytest.mark.parametrize("t_max", [0.0, -1.0, float("nan"), float("inf")])
def test_bad_t_max_is_rejected(t_max):
    """归一化常数不合法就炸掉，不静默发一个按比例错掉的推力。"""
    with pytest.raises(ValueError):
        px4_rates_setpoint((0, 0, 0), 5.0, t_max)


@pytest.mark.parametrize("thrust", [-0.1, float("nan"), float("inf")])
def test_bad_thrust_is_rejected(thrust):
    """★ 推力这一路的非有限值必须在**发布之前**拦住：PX4 对角速率有 NaN 回退、对推力没有。"""
    with pytest.raises(ValueError):
        px4_rates_setpoint((0, 0, 0), thrust, 11.8)


@pytest.mark.parametrize("omega", [(0.0, 0.0), (0.0, 0.0, 0.0, 0.0), (0.0, float("nan"), 0.0)])
def test_bad_omega_is_rejected(omega):
    with pytest.raises(ValueError):
        px4_rates_setpoint(omega, 5.0, 11.8)


def test_no_ros_import_leaks_in():
    """纯函数模块**不得**拉进 ROS：离线 OCP 与假 PX4 夹具都要能直接 import 它（§8.1）。"""
    for module in ("rclpy", "px4_msgs"):
        assert module not in sys.modules, f"sign_map 把 {module} 带进来了 —— 它必须是纯函数"
