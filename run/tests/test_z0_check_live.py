"""`run/z0.py check` 的**正控**：对着台架夹具（假 mavros）跑一遍，读数必须真的读到东西。

跑法（★ 先 source，夹具要 `bench` 包在环境里）：

    source /opt/ros/humble/setup.bash && source install/setup.bash
    python3 -m pytest run/tests/test_z0_check_live.py -q

★ 为什么必须有这一条：`check` 的解析逻辑（纯函数）已经在 `test_z0_menu.py` 里测过了，
但它们**测不出"订上了没有"** —— 话题名拼错、QoS 不兼容、发现不匹配，全都会表现为
"帧数为 0"，而 `render_snapshot` 只会照实说"无帧"（它无从知道那是脚本的错）。
⟹ 这条用例让**真的帧**流过去（夹具在域 73 上发 `/mavros/state` 与 `/mocap/pose_enu`），
再断言读数落地。**负控**（`/mavros/setpoint_raw/local` 没有发布者 ⟹ 必须被点名）也在里面，
免得"满屏 ✓"的假绿。

⚠️ 域号由 `bench/pytest_support.py` 定（默认 73，**硬拒 0**）—— 那条纪律的理由见它自己的文件头。
"""

import pathlib
import sys

import pytest

RUN_DIR = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(RUN_DIR))

pytest.importorskip("rclpy", reason="要 source 过 ROS 才有 rclpy")
pytest.importorskip("bench", reason="台架夹具：先 `colcon build --packages-up-to bench` 并 source install")

import z0  # noqa: E402
from bench import Scenario  # noqa: E402
from bench.pytest_support import init_ros_for_tests, shutdown_ros_after_tests  # noqa: E402


@pytest.fixture(scope="session", autouse=True)
def _ros_context():
    # 会话级：context 一个进程一次（bench/pytest_support.py 的说明）
    init_ros_for_tests()
    yield
    shutdown_ros_after_tests()


@pytest.fixture(scope="module")
def snapshot():
    """夹具就位（`dut=None, start_dut=False` = **只起夹具**，不拉 DUT），再采一次读数。"""
    with Scenario(dut=None, start_dut=False) as sc:
        sc.hold(1.0)                      # 让夹具把第一帧发出去
        return z0.collect_snapshot(window_s=4.0)


def test_读到假飞控的状态原文(snapshot):
    assert snapshot["counts"].get("/mavros/state", 0) >= 2, "4 s 里 1 Hz 的 state 至少该有两帧"
    assert z0.describe_message  # 表是从 last 里拼的，这里只看原文
    text = snapshot["last"]["/mavros/state"]
    assert "connected=" in text and "armed=" in text and "mode=" in text


def test_动捕那条真的在流(snapshot):
    count = snapshot["counts"].get("/mocap/pose_enu", 0)
    assert count > 100, f"夹具的动捕 ~200 Hz ⟹ 4 s 里远不止 {count} 帧"


def test_只有订阅者的话题被判成没有发布者(snapshot):
    """★ 第一手：夹具（`dut=None`）**订**了 `/control/state` 与 `/mavros/setpoint_raw/local`
    （`Scenario._on_control_state`、`FakeMavros` 的接收端）⟹ 这两个话题**出现在图里**，
    但一帧发布都没有（`counts` 全 0）。判读必须把「在图里但没有发布者」与
    「话题都不在图里」分开说 —— 前者是"节点没起"，后者是"这个链路的这一段根本不存在"。"""
    assert snapshot["counts"].get("/control/state") == 0
    assert "/control/state" in snapshot["topics"], "它应当在图里（夹具订了它）"


def test_没起的那些被点名_而不是满屏勾(snapshot):
    text = z0.render_snapshot(snapshot)
    assert "✓ /mavros/state" in text
    assert "✓ /mocap/pose_enu" in text
    missing = text.split("缺什么")[1]
    # 负控 ①：只有订阅者的话题 ⟹ 说得比"没数据"具体（含"QoS 不兼容"这条不报错的坑）
    assert "/mavros/setpoint_raw/local（在图里但一帧都没来 ⟹ 发布者没起，或 QoS 不兼容）" in missing
    # 负控 ②：动捕节点没起 ⟹ `/mavros/vision_pose/pose` 连话题都不在图里
    assert "/mavros/vision_pose/pose（话题都不在图里 ⟹ 发布者没起）" in missing


def test_域号是台架域而不是真机域(snapshot):
    text = z0.render_snapshot(snapshot)
    assert "台架" in text, "域 73 的读数必须显眼地标成夹具造的 —— 否则会被当成真飞机"
