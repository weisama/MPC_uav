"""seam 2（**零 ROS**）：求解器数值健康度**可观测**。

覆盖 ticket 04 的验收项 (g)：残差 / 迭代数 / 状态码都要拿得到、且**分得清**含义。
外加两条本模块的第一手结论，它们是**设计决策的证据**（不是"测试 acados"）：

| 读数 | 实测行为 | 本包怎么办 |
|---|---|---|
| `get_stats("residuals")` | `SQP_RTI` 下**恒为全 0** | 不信它；自算 `u_bound_violation` / `quat_norm_drift` 顶上 |
| `get_stats("alpha")` | 抛 `ValueError`（RTI 没有这个量） | 不读 |
| `solver.get_cost()` | 不是返回轨迹的代价（差 ~96 倍） | 自算 `info.cost`（见 `test_ocp_cost_ref.py`） |

⚠️ 这几条**钉的是 acados 0.6.0 的行为**：哪天上游改了（比如 RTI 也开始填残差），
这些用例会红 —— 那不是坏事，那正是"回去改文档"的信号。
"""

from __future__ import annotations

import sys

import numpy as np
import pytest

from nmpc.ocp import OfflineOcp, SolveInfo


def test_normal_solve_exposes_status_and_iteration_counts(solve_warm, ocp: OfflineOcp,
                                                          const_ref, u_zero):
    """正常一次求解：状态码、两类迭代数都能读出来，且取值合理。"""
    sol = solve_warm(ocp.hover_state(), const_ref([0.5, 0.0, 0.0]), u_zero)
    info = sol.info

    assert info.status == 0, info.line()
    assert info.sqp_iter >= 1, "SQP_RTI 每周期至少走一次迭代"
    assert info.qp_iter.shape == (2,), "两个 QP 阶段（condensing 前后）各一个迭代数"
    assert np.all(info.qp_stat == 0), f"QP 状态码必须是 0，实际 {info.qp_stat}"
    assert info.ok and not info.degraded, info.line()


def test_residuals_are_zero_under_rti_and_we_do_not_rely_on_them(solve_warm, ocp: OfflineOcp,
                                                                const_ref, u_zero):
    """★ 第一手：`SQP_RTI` 下残差**恒为 0** —— 所以它不能当健康依据（本包改用自算量）。

    断言"它是 0"是为了让这条结论**可被证伪**：如果哪天它不再恒为 0，这个用例会红。
    """
    sol = solve_warm(ocp.hover_state(), const_ref([0.5, 0.0, 0.0]), u_zero)
    assert np.all(sol.info.residuals == 0.0), \
        f"实测 RTI 下残差恒为 0；现在变成了 {sol.info.residuals} —— 请回去改 SolveInfo 的说明"
    assert sol.info.residual_max == 0.0

    with pytest.raises(ValueError, match="alpha"):
        ocp.solver.get_stats("alpha")


def test_self_computed_health_metrics_are_meaningful(solve_warm, ocp: OfflineOcp, const_ref,
                                                     u_zero):
    """自算的两个替代量在正常解上都是 0（而不是"读不到"或"nan"）。"""
    sol = solve_warm(ocp.hover_state(), const_ref([0.5, 0.0, 0.0]), u_zero)
    assert sol.info.u_bound_violation <= SolveInfo.BOUND_TOL
    assert np.isfinite(sol.info.quat_norm_drift)


def test_quaternion_drift_stays_tiny_even_when_the_rate_is_saturated(solve_warm,
                                                                    ocp: OfflineOcp,
                                                                    const_ref, u_zero):
    """`|q|² − 1` 的实测量级：**正常时就该接近 0**（模型没有 `|q|=1` 的约束，`R(q)` 也不保证正交）。

    实测（ω 打满 3.84 rad/s、持续 1 s）：**1.0e-08**；悬停 1.8e-15、+x 2 m 2.4e-11。
    所以这个读数的用法是"**涨起来才是异常**"（轨迹发散会把它顶上去），而不是"越大越健康"。
    ⚠️ 参考值：连解 30 s（自己 `step_rk4` 滚 1500 步）才到 6.7e-08 —— 它涨得慢，别指望它当灵敏的哨兵。
    """
    sol = solve_warm(ocp.hover_state(), const_ref([10.0, 0.0, 0.0]), u_zero)
    assert sol.u[2] == pytest.approx(3.84, rel=1e-9), "这条用例的前提是角速率打满"
    assert sol.info.quat_norm_drift < 1e-6, \
        f"打满时实测 1e-8 量级，出现了 {sol.info.quat_norm_drift:.3e} —— 轨迹可能已经不对了"
    assert not sol.info.degraded, "漂移是模型精度问题，不该把求解器报成退化"


def test_degraded_flag_logic_is_decided_by_the_fields():
    """`ok` / `degraded` 的判定逻辑（直接构造 `SolveInfo`，不依赖某个具体的求解结果）。"""
    good = dict(status=0, sqp_iter=1, qp_iter=np.array([0.0, 15.0]), qp_stat=np.array([0.0, 0.0]),
                residuals=np.zeros(8), u_bound_violation=0.0, quat_norm_drift=1e-8,
                cost=1.0, cost_acados=1.0, time_tot=1e-3, time_cycle=2e-3)
    assert SolveInfo(**good).ok
    assert not SolveInfo(**good).degraded, "四元数漂移不进 degraded（见 SolveInfo 的说明）"

    # ① 状态码非 0 ② QP 状态码非 0 ③ u 越界 ④ 耗时非有限
    assert SolveInfo(**{**good, "status": 2}).degraded
    assert SolveInfo(**{**good, "qp_stat": np.array([0.0, 3.0])}).degraded
    assert SolveInfo(**{**good, "u_bound_violation": 1e-3}).degraded
    assert SolveInfo(**{**good, "time_tot": np.nan}).degraded


def test_prediction_false_reports_no_cost_instead_of_a_fake_one(ocp: OfflineOcp):
    """`prediction=False` 不读轨迹 ⟹ 算不了代价：如实报 nan，且**不能**被误判成退化。"""
    x0 = ocp.hover_state()
    sol = ocp.solve(x0, np.tile(x0, (ocp.config.N + 1, 1)), np.zeros((ocp.config.N, 4)),
                    prediction=False)
    assert sol.u_pred is None and sol.x_pred is None
    assert np.isnan(sol.info.cost) and not sol.info.cost_available
    assert not sol.info.degraded, "没算代价 ≠ 退化（degraded 只在真算过时才看它）"
    assert "cost=—(未取轨迹)" in sol.info.line(), "日志行要说清「代价没算」，不能印个 0 糊弄"


def test_no_ros_import_leaks_in():
    """OCP 单元必须是**零 ROS** 的（`nmpc/__init__.py` 的分层承诺）——
    离线 OCP 与假 PX4 夹具都要能直接 import 它，而"能 import"只有靠零依赖才守得住。

    与 `test_sign_map.py::test_no_ros_import_leaks_in` 同一手法、同一理由；分在两个文件里
    是因为两边测的是不同的模块。
    """
    for module in ("rclpy", "px4_msgs"):
        assert module not in sys.modules, f"nmpc.ocp 把 {module} 带进来了 —— 它必须是零 ROS 的"


def test_prediction_false_without_explicit_u_ref_is_rejected(ocp: OfflineOcp):
    """`prediction=False` + 不给 `u_ref` ⟹ 直接报错，**不许**悄悄退回悬停前馈。

    缺省的"上一帧解平移一拍"就是**从预测轨迹里来的**；不读轨迹就没有它。若悄悄退回悬停前馈，
    M6 的参考量会永远停在 `[T_hover,0,0,0]` —— 那是**静默**的持续角速率抑制，
    正是 M6 要修的东西换了个壳。
    """
    x0 = ocp.hover_state()
    with pytest.raises(ValueError, match="prediction=False"):
        ocp.solve(x0, np.tile(x0, (ocp.config.N + 1, 1)), prediction=False)
