"""seam 2（**零 ROS**）：约束 —— 打满可以，越界不行。

覆盖 ticket 04 的验收项 (e)：**不可行目标（+x 10 m）→ ω 与 F 饱和但不越界**。
外加 M3 的落地证据：**偏航上限是独立的**（6.28 rad/s），不是参考实现写的"横滚的一半"（1.92）。

★ 不可行目标下的行为要说清楚，否则断言一定会写歪（实测）：
`SQP_RTI` 单次迭代 + 约束打满时，**角速率一拍就贴上限并保持**（第 1 次 solve 就 ω_y = 3.84），
但**推力会一直晃**（实测 12 次迭代：0 → 7.92 → 2.43 → 8.66 → … → 11.47，仍在漂）。
所以这里**只断言角速率饱和与边界不越**，**不断言推力收敛** —— 后者本来就不成立。
"""

from __future__ import annotations

import numpy as np
import pytest

from nmpc import YAW_RATE_MAX, Z0Params, yaw_quat
from nmpc.ocp import OfflineOcp, SolveInfo

#: ★ **别把 QP 的输出拿 `==` 断言**（本文件踩过一次）：一个"要往下降"的目标实测解出
#: `F = 8.05e-18` 而不是 `0.0` —— 断言写 `== 0.0` 就会**偶发**变红（第一次跑还是绿的）。
#: 判"落在边界上"用容差；判"不越界"直接用模块自己的 `SolveInfo.BOUND_TOL`（生产阈值，
#: 而不是测试另造一个数）。
ZERO_TOL = 1e-9


def test_unreachable_target_saturates_rate_without_violating(solve_warm, const_ref, u_zero,
                                                             ocp: OfflineOcp):
    """(e) +x 10 m 在 1 s 内够不着（倾满 45° 也才 ~4.9 m）⟹ ω_y 贴上限，且**一步不越**。"""
    p = Z0Params()
    x0 = ocp.hover_state()
    sol = solve_warm(x0, const_ref([10.0, 0.0, 0.0]), u_zero)

    assert sol.u[2] == pytest.approx(p.pitch_rate_max, rel=1e-9), \
        f"够不着的目标该把 ω_y 打满，实际 {sol.u[2]:.6f}"
    assert sol.info.u_bound_violation <= SolveInfo.BOUND_TOL, \
        "打满 ≠ 越界：RTI 的 QP 解必须落在 [lbu, ubu] 内"
    assert sol.info.ok, sol.info.line()
    assert not sol.info.degraded, sol.info.line()


def test_thrust_pins_at_both_bounds_without_violating(solve_warm, const_ref, u_zero,
                                                      ocp: OfflineOcp):
    """推力的两个边界都**够得着**，且都不越：要"又远又高"⟹ `F = T_max`；要往下降 ⟹ `F = 0`。

    ★ 实测纠正了一个想当然（`phase2-implementation.md` §4 S3 的表原来写的是"目标 +x 10 m ⟹
    F 打满 T_max"）：**纯水平**的远目标**不会**把推力顶到上限 —— 它只要倾斜，
    垂直分量仍≈悬停，实测只到 **11.4044 N**（上限 11.8003）。要顶到上限得**同时要求爬升**
    （`[10, 0, 10]` 与 `[10, 0, 5]` 都精确停在 11.8003）。
    """
    p = Z0Params()
    x0 = ocp.hover_state()

    sol = solve_warm(x0, const_ref([10.0, 0.0, 10.0]), u_zero)
    assert sol.u[0] == pytest.approx(p.t_max, rel=1e-9), "又远又高的目标该把推力顶到上限"
    assert sol.u[2] == pytest.approx(p.pitch_rate_max, rel=1e-9), "同时角速率也打满"
    assert sol.info.u_bound_violation <= SolveInfo.BOUND_TOL, sol.info.line()

    sol = solve_warm(x0, const_ref([0.0, 0.0, -5.0]), u_zero)
    assert sol.u[0] == pytest.approx(0.0, abs=ZERO_TOL), "要快速下降 ⟹ 推力落到非负下界"
    assert sol.info.u_bound_violation <= SolveInfo.BOUND_TOL, sol.info.line()

    # 对照组：纯水平的远目标**不会**顶到推力上限（11.40 < 11.80）
    sol = solve_warm(x0, const_ref([10.0, 0.0, 0.0]), u_zero)
    assert sol.u[0] < p.t_max, "纯水平目标只需倾斜，垂直分量仍≈悬停"
    assert sol.u[2] == pytest.approx(p.pitch_rate_max, rel=1e-9)


def test_wired_bounds_use_per_axis_rate_limits(ocp: OfflineOcp):
    """M3 的**接线**证据：`[lbu, ubu]` 的三根轴速率上限 = `Z0Params.rate_max`，偏航不被折半。

    参考实现写的是 `[0, -w, -w, -0.5*w]` / `[Fmax, w, w, +0.5*w]`（`w = 0.5` 占位符），
    而 z0 的偏航上限（6.28）比横滚（3.84）**更大** —— 折半会把它压成 1.92，
    也就是把偏航机动砍掉三分之二，而且**没有任何东西会报错**。
    """
    p = Z0Params()
    lb = np.asarray(ocp.acados_ocp.constraints.lbu)
    ub = np.asarray(ocp.acados_ocp.constraints.ubu)
    assert lb[0] == 0.0 and ub[0] == pytest.approx(p.t_max, rel=1e-12), "F 的上下界"
    assert np.allclose(lb[1:], -np.array(p.rate_max), rtol=1e-12)
    assert np.allclose(ub[1:], np.array(p.rate_max), rtol=1e-12)
    assert ub[3] == pytest.approx(p.yaw_rate_max, rel=1e-12)
    # 参考实现的写法是 `0.5 * w`（横滚的一半）：那是 M3 要修的 bug，这里把它钉成"不等于"
    assert abs(ub[3] - ub[1] / 2) > 1.0, "偏航上限**不是**横滚的一半 —— 折半就是把偏航机动砍掉三分之二"


def test_yaw_rate_above_half_roll_cap_is_actually_reachable(solve_warm, ocp: OfflineOcp):
    """M3 的**行为**证据：4.0 rad/s 的持续偏航转得出来（> 横滚一半 1.92，< 偏航上限 6.28）。

    做法：参考本身就是"以 ω_d 匀速偏航"的那条轨迹，`u_ref` 给对应的前馈
    （这正是 M6 要的用法）。若上限被错接成横滚的一半，QP 会在 1.92 处把它卡住。
    """
    p = Z0Params()
    omega_d = 4.0
    assert p.roll_rate_max / 2 < omega_d < YAW_RATE_MAX, "这个用例的存在前提就是这条不等式"

    x0 = ocp.hover_state()
    x_ref = np.tile(x0, (ocp.config.N + 1, 1))
    for i in range(ocp.config.N + 1):
        x_ref[i, 6:10] = yaw_quat(omega_d * i * ocp.dt)
    u_ref = np.tile([p.t_hover, 0.0, 0.0, omega_d], (ocp.config.N, 1))

    sol = solve_warm(x0, x_ref, u_ref)
    assert sol.u[3] == pytest.approx(omega_d, abs=1e-3), \
        f"前馈一致的匀速偏航应该被原样跟住，实际 ω_z={sol.u[3]:.6f}"
    assert sol.info.u_bound_violation <= SolveInfo.BOUND_TOL, sol.info.line()
