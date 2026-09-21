"""台架夹具自己的测试 —— 会话级 ROS 上下文（见 bench/pytest_support.py 的说明）。"""

import pytest

from bench.pytest_support import init_ros_for_tests, shutdown_ros_after_tests


@pytest.fixture(scope="session", autouse=True)
def _ros_context():
    init_ros_for_tests()
    yield
    shutdown_ros_after_tests()
