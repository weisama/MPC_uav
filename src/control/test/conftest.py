"""control 的节点级测试 —— 会话级 ROS 上下文（见 bench/pytest_support.py）。

★ 夹具（假 mavros + 脚本化动捕）来自 `bench` 包；它是 Phase 1/2/3 所有节点级测试的公共底座。
  本文件里唯一要做的是把 ROS context 挂成**会话级**：反复 init/shutdown 换 DDS context
  是测试抖动的常见来源。
"""

import pytest

from bench.pytest_support import init_ros_for_tests, shutdown_ros_after_tests


@pytest.fixture(scope="session", autouse=True)
def _ros_context():
    init_ros_for_tests()
    yield
    shutdown_ros_after_tests()
