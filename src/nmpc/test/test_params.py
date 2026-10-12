"""归一化自洽（`T_max = m·g / hover_thrust`）的断言 —— 纯 Python，零 ROS、零飞机。

★ 为什么值得单独一组：[`offboard_smoke.py`](../nmpc/offboard_smoke.py) 的 `--t-max` 是**必须显式给**的
参数（2026-10-12 操作者裁定），它曾经默认 `11.8` —— 那是 `f_max`（**按电池**能持续兑现多少
推力），**另一个物理量**。拿它当**归一化常数**，悬停那一档发出去的数是正确值的 ≈3.92 倍，
**不报错、不崩溃、判据全绿**，只是飞机上飘。这组断言就是把它当场变红的那一个东西
（[`13`](../../../.scratch/phase4/issues/13-mavros-downlink-switch.md) §三②）。

★ 与 `test_sign_map.py` 的分工：那边断言的是**符号与归一化公式**（`F/T_max` 这一行对不对），
这边断言的是**归一化常数本身取没取对**（`T_max` 这个 N 数）。两件事会各自单独错。
"""

from __future__ import annotations

import pytest

from nmpc import commands, node_core, params, sign_map


# ───────────────────────────── 那条导出关系 ─────────────────────────────

def test_default_t_max_is_the_derived_one():
    """`Z0Params.t_max` 走的是 `t_max_from_hover` —— 默认那套必须仍是 `m·g / hover_thrust`。

    ⛔ 这一条挡的是"把 `t_max` 改成一个独立的常数（或改回 `f_max`）"：
    那样它就与 `hover_thrust` 脱钩，而 PX4 对 `thrust_body` **不做任何重标定**
    （`MulticopterRateControl.cpp:182`），脱钩 = 悬停前馈静默错。
    """
    z0 = params.DEFAULT
    assert z0.t_max == pytest.approx(params.t_max_from_hover(z0.mass, z0.hover_thrust), rel=1e-15)
    # 口径：悬停力 / 悬停油门（**不是** `f_max` —— 那是按电池的 F 上限，另一个量）
    assert z0.t_max == pytest.approx(z0.t_hover / z0.hover_thrust, rel=1e-15)
    # 生产里那个数就长这样（46.3027…）—— 写死在票面与 `--help` 里，这里对一次账
    assert z0.t_max == pytest.approx(46.3027, rel=1e-4)
    # 反向：反算回来必须是实测的那个悬停油门
    assert params.hover_thrust_implied(z0.mass, z0.t_max) == pytest.approx(z0.hover_thrust, rel=1e-12)


def test_the_property_reads_while_this_door_guards():
    """★ 同一公式两处：**属性是"读"、函数是"守门口"** —— 分工不许被"顺手统一"掉。

    ★★ 这条来自一次真实的回归（2026-10-12 本票施工中）：把 `Z0Params.t_max` 改成调
    `t_max_from_hover()`（"只有一个实现"看着更干净）⟹ **坏参数读不出来了** ⟹
    `preflight_check` 从"按名字点名"退化成"抛异常"：

        NodeParams(mass=0.0)            → 该报「质量为正」，实际 raise
        NodeParams(hover_thrust=1.2)    → 该报「悬停有上余量」，实际 raise

    而自检**必须**能对着坏参数把话说出来 —— 它存在的意义就是告诉操作者哪里不对。
    ⛔ 所以这里两个方向都钉：读得出来（不 raise）＋ 门守得住（raise）。
    """
    # 读：坏参数照样算得出一个数（自检随后拿它跟 `f_max` 比、写进消息里）
    assert params.Z0Params(mass=0.0).t_max == 0.0
    assert params.Z0Params(hover_thrust=1.2).t_max > 0.0
    # 守：同一对入参走门口就炸（命令行那条路）
    with pytest.raises(ValueError):
        params.t_max_from_hover(0.0)
    with pytest.raises(ValueError):
        params.t_max_from_hover(0.75, hover_thrust=1.2)
    # 好参数上两处必须同值，否则"读"和"守"就开始各说各话
    z0 = params.DEFAULT
    assert z0.t_max == params.t_max_from_hover(z0.mass, z0.hover_thrust)


def test_normalization_tolerance_matches_production():
    """容差口径必须与生产那一个**同值**（`node_core.SAT_TOL_REL`）。

    ★ 两个常量分处两个模块是**故意**的（`node_core` 反过来 import 本模块，合并会成环），
    所以只能靠这条断言盯着漂移 —— 改一个忘另一个，这里红。
    """
    assert params.NORM_TOL_REL == node_core.SAT_TOL_REL


# ───────────────────────────── 那条启动自检 ─────────────────────────────

def test_hover_consistency_accepts_the_production_pair():
    """正确的那一对通过，且**返回值**就是隐含的悬停油门（调用方拿它去打印，用容差断言）。"""
    z0 = params.DEFAULT
    implied = params.check_hover_consistency(z0.mass, z0.t_max)
    assert implied == pytest.approx(z0.hover_thrust, rel=params.NORM_TOL_REL)
    # `--help` 里给操作者抄的那个 46.3027 也要过
    params.check_hover_consistency(0.75, 46.3027)


def test_hover_consistency_rejects_the_battery_limit_constant():
    """★★ **变异测试**（票面点名要的那一次）：把 `T_max` 填回旧默认 `11.8`（= `f_max`）⟹ **必须红**。

    红不了就说明这条自检没有鉴别力 —— 而它挡的正是"飞起来不报错、只是上飘"那一档。
    报错话术还必须能让人**当场照抄出正确值**，否则操作者看到的只是"对不上"。
    """
    with pytest.raises(ValueError) as exc:
        params.check_hover_consistency(0.75, 11.8)
    msg = str(exc.value)
    assert "46.3027" in msg      # 正确值照抄得到
    assert "3.92" in msg         # 差多少倍也说出来（这是"上飘"的量级）


def test_hover_consistency_tolerance_is_the_production_one():
    """容差是**相对** 1e-3：`46.30`（照 `--help` 抄三位小数）过、`46`（随手一写）不过。

    ★ 分界要卡在这里：松到让 `46` 过去，这条自检就只剩"填 11.8 才红"，
    而 `46` 这种量级的错（悬停多发 0.66 %）恰恰是操作者最容易写出来的。
    """
    assert params.check_hover_consistency(0.75, 46.30) > 0.0
    with pytest.raises(ValueError):
        params.check_hover_consistency(0.75, 46.0)


@pytest.mark.parametrize("t_max", [0.0, -1.0, float("nan"), float("inf")])
def test_hover_consistency_rejects_meaningless_t_max(t_max):
    """0 / 负数 / 非有限：这里就抛，不留到归一化那一步（那一步会变成除以 0 或全 NaN）。"""
    with pytest.raises(ValueError):
        params.check_hover_consistency(0.75, t_max)


@pytest.mark.parametrize("mass", [0.0, -0.75, float("nan")])
def test_t_max_from_hover_rejects_bad_mass(mass):
    with pytest.raises(ValueError):
        params.t_max_from_hover(mass)


# ────────────────── 断言落在**发出去的那个数**上 ──────────────────

def test_the_hover_phase_emits_the_hover_thrust():
    """★ 悬停那一档（悬停倍数 = 1.0）归一化之后**必须等于悬停油门** —— 走**真入口**，不另写公式。

    两条路都断言：同一个物理量、同一个 `T_max`，差别只有符号（`sign-map.md` §2.2）。
    ⚠️ 这里两处 `g` 不同源：`commands.G = 9.80665`（与 SITL 被控对象同常数）、
    `params.G = 9.81`（与参考实现同常数）⟹ 残差 ~4.5e-4 相对，落在生产容差内但**不是** 0。
    这正是不能用 `==` 的原因之一。
    """
    z0 = params.DEFAULT
    hover_n = commands.hover_force_n(z0.mass)
    mav = sign_map.mavros_attitude_target((0.0, 0.0, 0.0), hover_n, z0.t_max)
    assert mav.thrust == pytest.approx(z0.hover_thrust, rel=params.NORM_TOL_REL)
    px4 = sign_map.px4_rates_setpoint((0.0, 0.0, 0.0), hover_n, z0.t_max)
    assert px4.thrust_body_z == pytest.approx(-z0.hover_thrust, rel=params.NORM_TOL_REL)


def test_a_wrong_t_max_scales_the_hover_command_visibly():
    """★ 反面：`T_max` 填成 `f_max` ⟹ 悬停那一档发出去的是 **≈3.92 倍**。

    这条把"为什么它静默"量化下来：错值**不越界**（仍落在 [0, 1] 内）⟹
    没有任何一处拒绝条件拦得住它，只是**大一档** —— 现象就是飞机上飘。
    """
    z0 = params.DEFAULT
    hover_n = commands.hover_force_n(z0.mass)
    right = sign_map.mavros_attitude_target((0.0, 0.0, 0.0), hover_n, z0.t_max)
    wrong = sign_map.mavros_attitude_target((0.0, 0.0, 0.0), hover_n, z0.f_max)
    assert right.thrust == pytest.approx(z0.hover_thrust, rel=params.NORM_TOL_REL)
    assert wrong.thrust / right.thrust == pytest.approx(z0.t_max / z0.f_max)
    assert wrong.thrust / right.thrust == pytest.approx(3.92, rel=1e-2)
    # ⟹ 值域上没人拦得住（`mavros_attitude_target` 刻意**不**拦 `thrust > 1`，见它的 docstring）
    assert 0.0 < wrong.thrust < 1.0
