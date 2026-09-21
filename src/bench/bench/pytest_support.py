"""pytest 的会话级 ROS 上下文 —— 用它的包在自己的 `test/conftest.py` 里挂一次。

★ 为什么是**会话级**而不是每个用例一次：`rclpy.init()` / `rclpy.shutdown()` 每轮都换一个
  DDS context，反复轮换是测试抖动的常见来源（participant 泄漏、日志引用计数翻动）。
  正确的分法是：**context 一个进程一次**，节点/executor 每个用例一套。

★ `ROS_DOMAIN_ID` 是**安全边界，不是洁癖**：本机 domain 0 上跑着真的 mavros
  （对着一台真的飞控，`/dev/ttyTHS1`），而夹具会提供 `/mavros/cmd/arming`。
  跑进同一个域 ⟹ 测试里的"假解锁"可能打到真飞机上。所以：
    · 默认 domain 73（可用 `BENCH_ROS_DOMAIN_ID` 覆盖，并行跑多份测试时用）
    · **拒绝 domain 0**（硬失败，不给"我以为没事"留口子）
    · `ROS_LOCALHOST_ONLY=1`：本机多网卡（动捕 WiFi + 校园网），别把测试组播漏出去
    · DUT 子进程显式传 `env=`，不靠"碰巧继承了环境"
"""

from __future__ import annotations

import os

DOMAIN_ENV = "BENCH_ROS_DOMAIN_ID"
DEFAULT_DOMAIN = 73

#: ★ 夹具与 DUT **必须**用同一个取值。写死成常量而不是 `setdefault`：本机环境里
#:   `ROS_LOCALHOST_ONLY=0` 是有人设过的，若夹具只 setdefault、DUT 却强设，两边就会用不同的
#:   DDS 传输配置 —— 表现为"进程都活着，但彼此发现不了"，而且没有任何报错。
LOCALHOST_ONLY = "1"


def bench_domain() -> int:
    """本进程用的 ROS 域（`BENCH_ROS_DOMAIN_ID`，默认 73）。"""
    return int(os.environ.get(DOMAIN_ENV, DEFAULT_DOMAIN))


def init_ros_for_tests(domain: int = None) -> int:
    """初始化 rclpy 的默认 context；返回实际使用的 domain。重复调用是安全的（只 init 一次）。"""
    import rclpy

    domain = bench_domain() if domain is None else int(domain)
    if domain == 0:
        raise RuntimeError(
            "拒绝在 ROS_DOMAIN_ID=0 上跑台架夹具：真 mavros（连着真飞控）就在 domain 0 里，"
            f"夹具会提供 /mavros/cmd/arming ⟹ 可能误发解锁。请设 {DOMAIN_ENV} 为一个非 0 的值。")

    # 环境变量必须在 init 之前设好：C++ 侧的 DUT 子进程只能靠它拿到 domain。
    os.environ["ROS_DOMAIN_ID"] = str(domain)
    os.environ["ROS_LOCALHOST_ONLY"] = LOCALHOST_ONLY

    if not rclpy.ok():
        rclpy.init(args=[], domain_id=domain)
    return domain


def shutdown_ros_after_tests() -> None:
    import rclpy

    rclpy.try_shutdown()


def dut_env(extra: dict = None) -> dict:
    """DUT 子进程的环境：显式带上 domain 与"日志走 stdout"（好让 ctest 抓到日志）。"""
    env = dict(os.environ)
    env["ROS_DOMAIN_ID"] = str(bench_domain())
    env["ROS_LOCALHOST_ONLY"] = LOCALHOST_ONLY
    env["RCUTILS_LOGGING_USE_STDOUT"] = "1"
    env["RCUTILS_LOGGING_BUFFERED_STREAM"] = "1"
    env["PYTHONUNBUFFERED"] = "1"
    if extra:
        env.update(extra)
    return env
