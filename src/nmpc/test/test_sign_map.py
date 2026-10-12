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
import subprocess
import sys
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO / "tools" / "sign-map"))  # 复核模块（独立推导 + 权威块解析）

import sign_map_core  # noqa: E402  （tools/sign-map 下的复核库，不是本包的实现）

from nmpc.sign_map import mavros_attitude_target, px4_rates_setpoint  # noqa: E402

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


# ─────────────────── 2026-10-12 换通路：mavros 路径的第二个入口 ───────────────────
#
# 口径见 `sign-map.md` **§2.2 / §2.3**（"谁取负"那张表）。★ 下面这几条的落点**必须**是
# "**我们发出去的那个值**"：mavros 会在 `attitude_cb` 里再转一次 FLU→FRD
# （`setpoint_raw.cpp:323-324`）⟹ 若我们自己先转一次，**飞控最终收到的东西分毫不差**，
# 任何落在"飞控收到了什么"上的断言都会绿（`spec.md` §★★ 的 N2）。


def test_mavros_entry_emits_the_flu_values_verbatim():
    """★ N2 的断言：`body_rate` 三轴都是 **FLU 原样**，我们一个负号都不许加。"""
    for omega, thrust, t_max in CASES:
        mav = mavros_attitude_target(omega, thrust, t_max)
        assert mav.body_rate_x == pytest.approx(omega[0], abs=1e-12)
        assert mav.body_rate_y == pytest.approx(omega[1], abs=1e-12)
        assert mav.body_rate_z == pytest.approx(omega[2], abs=1e-12)


def test_the_two_entries_differ_only_in_who_applies_the_sign():
    """两条路是**同一张 §2.1 的表**，差别只在"负号由谁加"（§2.2 / §2.3）。

    roll 两边一样（mavros 那一步不换 x 分量）；pitch / yaw / 推力**互反**（别人替我们取负）。
    ⛔ 这条同时是"两条路不能混用"的护栏：把 px4 那一版的结果塞进 mavros 的消息里，
    飞控最终拿到的是**反的** pitch/yaw 与**朝下的**推力 —— 而两个入口谁也不会报错。
    """
    for omega, thrust, t_max in CASES:
        px4 = px4_rates_setpoint(omega, thrust, t_max)
        mav = mavros_attitude_target(omega, thrust, t_max)
        assert mav.body_rate_x == pytest.approx(px4.roll, abs=1e-12)       # roll：都不取负
        assert mav.body_rate_y == pytest.approx(-px4.pitch, abs=1e-12)     # pitch：mavros 取负
        assert mav.body_rate_z == pytest.approx(-px4.yaw, abs=1e-12)       # yaw：同上
        assert mav.thrust == pytest.approx(-px4.thrust_body_z, rel=1e-12, abs=1e-15)


def test_mavros_thrust_is_positive_and_normalized():
    """`AttitudeTarget.thrust`：**正号**、按 `T_max` 归一化（负号由飞控 `fill_thrust` 加）。

    归一化的**模**必须与另一条路一致（同一个物理量、同一个 `T_max`）—— 差别只在符号。
    """
    assert mavros_attitude_target((0, 0, 0), 0.0, 11.8).thrust == 0.0
    assert mavros_attitude_target((0, 0, 0), 11.8, 11.8).thrust == pytest.approx(1.0)
    assert mavros_attitude_target((0, 0, 0), 3.0, 11.8).thrust > 0.0
    # 同一个物理推力、`T_max` 减半 ⟹ 归一化值翻倍（这件事两条路上都必须成立）
    one = mavros_attitude_target((0, 0, 0), 3.0, 11.8).thrust
    two = mavros_attitude_target((0, 0, 0), 3.0, 5.9).thrust
    assert two == pytest.approx(2.0 * one)


@pytest.mark.parametrize("t_max", [0.0, -1.0, float("nan"), float("inf")])
def test_mavros_entry_rejects_bad_t_max(t_max):
    """两个入口**共用一个检查器**（`_checked_inputs`）—— 拒绝条件不许只有一条路拦得住。"""
    with pytest.raises(ValueError):
        mavros_attitude_target((0, 0, 0), 5.0, t_max)


@pytest.mark.parametrize("thrust", [-0.1, float("nan"), float("inf")])
def test_mavros_entry_rejects_bad_thrust(thrust):
    with pytest.raises(ValueError):
        mavros_attitude_target((0, 0, 0), thrust, 11.8)


@pytest.mark.parametrize("omega", [(0.0, 0.0), (0.0, 0.0, 0.0, 0.0), (0.0, float("nan"), 0.0)])
def test_mavros_entry_rejects_bad_omega(omega):
    with pytest.raises(ValueError):
        mavros_attitude_target(omega, 5.0, 11.8)


def test_no_ros_import_leaks_in():
    """纯函数模块**不得**拉进 ROS：离线 OCP 与假 PX4 夹具都要能直接 import 它（§8.1）。

    ★★ 判据在**子进程**里取（ticket 06 复核轮改的，只改取法、不改结论）：`sys.modules` 是
      整个 pytest 进程的，而同目录的 `test_mpc_node.py` / `test_preflight_check.py`
      （ticket 06 新增）会 import `px4_msgs` / `rclpy` ⟹ 直接查 `sys.modules` 时本断言
      在 `pytest src/nmpc/test/` 下**假红**，而在 `colcon test`（一个文件一个进程）下绿。
      子进程里"只 import sign_map"问的是同一个契约，且与谁先跑无关。
    """
    code = ("import sys, nmpc.sign_map;"
            " assert 'rclpy' not in sys.modules, 'sign_map 把 rclpy 带进来了';"
            " assert 'px4_msgs' not in sys.modules, 'sign_map 把 px4_msgs 带进来了'")
    done = subprocess.run([sys.executable, "-c", code], capture_output=True, text=True)
    assert done.returncode == 0, f"sign_map 不再是纯模块：{done.stderr.strip()[-300:]}"
