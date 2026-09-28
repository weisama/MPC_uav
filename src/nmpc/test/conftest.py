"""seam 2（**零 ROS**）的公共夹具 —— 见 [`../README.md`](../README.md)。

★ **每个断言之前都要先"热身"，这是本目录最容易踩的坑**（第一手，2026-09-28 本机实测）：

`SQP_RTI` 每个周期只做**一次**迭代，所以 `solve()` 返回的**不是**"这次问题的最优解"，
而是"从上一帧的迭代点出发走一步"的结果。在**全新实例**上，那个迭代点还是出厂初值 ⟹
**第 0 次 solve 基本没动**：

    目标 +x 10 m（不可行，该把 ω_y 打满）：
      第 0 次 → ω_y = 0.0000、F = 7.8111   ← 跟悬停一模一样，纯初始迭代点
      第 1 次 → ω_y = 3.8400（打满）
      目标 +x 0.5 m（`u_ref = 0`）：第 0 次 ω_y = 0.0000 → 第 1 次 +0.3268 → 第 2 次 +0.3244
                                        → …稳定在 +0.3239
      ⚠️ 同一条参考、**不给 `u_ref`** 时它**不收敛**：0.3225 → … → 第 11 次 1.3487（一路上爬）。
        见 `test_ocp_trim.py::test_default_u_ref_on_a_static_reference_ramps` 与 §4.4

★ **这不是缺陷，是 RTI 的定义**：在线闭环时"上一帧的解"就是起点，节点从第二个周期起
拿到的是稳态解。但它有两个硬后果，别踩：

1. **测试必须把"首帧行为"和"稳态行为"分开** —— 拿第 0 次的结果去断言任何东西，
   测到的都是 acados 的初始迭代点，不是我们的模型/代价/约束。用 `solve_warm`。
2. **节点侧不能信第一拍**（归 ticket 06 / 07 判断，这里只留证据与数字）。

★ `WARMUP = 10` 的来历：0.5 m 那组第 2–3 次就稳定、10 次后小数位不动；不可行目标例外
（推力一直在晃，见 `test_ocp_constraints.py` 里写明的那条）。
"""

from __future__ import annotations

import numpy as np
import pytest

from nmpc import NU, OfflineOcp

#: 断言前先连解多少次 —— 理由见模块 docstring（RTI 的单次迭代需要上一帧做起点）
WARMUP = 10


@pytest.fixture(scope="session")
def ros_context():
    """会话级的 ROS 上下文 —— **只有 seam 1（起 DUT 子进程）的用例请求它**。

    ★ 为什么不做成 `autouse`：本目录里绝大多数用例是**零 ROS** 的（`sign_map`、离线 OCP），
      它们不该因为"同目录还有一个集成测试"就去 init 一个 DDS 上下文 —— 那会顺手改掉
      进程的环境（`ROS_DOMAIN_ID` / `ROS_LOCALHOST_ONLY`），也会让 `ROS_DOMAIN_ID=0`
      这一条安全网在这里无端生效（`bench/pytest_support.py` 会**硬拒** domain 0）。
      所以 `test_mpc_node.py` 用 `pytestmark = pytest.mark.usefixtures("ros_context")` 显式要它。

    ★ 夹具（`Scenario` / `FakePx4` / 假 mavros）用 rclpy 直接在**本进程**收发 ⟹ 必须在
      任何 `Scenario()` 之前 init 一次，而且是**会话级**（反复 init/shutdown 是抖动之源，
      见 `bench/pytest_support.py`）。`from bench…` 放在函数里：`bench` 没装时，
      零 ROS 的用例不该因为这个 import 而整个目录收集失败。
    """
    from bench.pytest_support import init_ros_for_tests, shutdown_ros_after_tests

    init_ros_for_tests()
    yield
    shutdown_ros_after_tests()


@pytest.fixture(scope="module")
def ocp() -> OfflineOcp:
    """一个模块一个求解器实例 —— 建实例要代码生成 + C 编译，别每个用例都建。"""
    return OfflineOcp()


@pytest.fixture
def solve_warm(ocp: OfflineOcp):
    """连解 `n` 次，返回**最后一次** —— 测试要断言的是稳态行为（见模块 docstring）。"""

    def _run(x0, x_ref, u_ref=None, n: int = WARMUP, **kwargs):
        sol = None
        for _ in range(n):
            sol = ocp.solve(x0, x_ref, u_ref, **kwargs)
        return sol

    return _run


@pytest.fixture
def u_zero(ocp: OfflineOcp) -> np.ndarray:
    """全零的 `u_ref`（形状跟着 `N`/`nu` 走，不在用例里写死数字）。"""
    return np.zeros((ocp.config.N, NU))


@pytest.fixture
def const_ref(ocp: OfflineOcp):
    """常数偏移参考：`hover_state()` 平移 `offset` 后铺满整条时域（参考实现也是这么用的）。"""

    def _build(offset) -> np.ndarray:
        x = ocp.hover_state()
        x[:3] += np.asarray(offset, dtype=float)
        return np.tile(x, (ocp.config.N + 1, 1))

    return _build
