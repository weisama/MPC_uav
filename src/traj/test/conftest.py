"""traj 的节点级测试 —— 会话级 ROS 上下文（见 bench/pytest_support.py）。

★ 与 control/test/conftest.py 完全同款：夹具（假 mavros + 脚本化动捕）来自 `bench` 包，
  它把 ROS context 挂成**会话级**。反复 init/shutdown 换 DDS context 是测试抖动的常见来源。
"""

import pytest

from bench.pytest_support import init_ros_for_tests, shutdown_ros_after_tests


@pytest.fixture(scope="session", autouse=True)
def _ros_context():
    init_ros_for_tests()
    yield
    shutdown_ros_after_tests()
