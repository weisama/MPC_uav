"""Scenario —— 一次"被控对象在环"的节点级测试。

它把四件事拼成一个上下文管理器：

    ┌── 假 mavros（`FakeMavros`）：提供 PX4 的"外部世界"，并记录 DUT 发来的每一帧 setpoint
    ├── 假 PX4（`FakePx4`，**可选**，`px4=True`）：Phase 2 的 `/fmu/*` 外部世界
    ├── 脚本化动捕（`VrpnSource`）：按秒表喂出"动捕说有数据"
    ├── 故障注入时序（`Timeline`）：第几秒掐掉哪条话题、改哪个状态
    └── DUT 子进程：真的把节点当进程启起来（★ 不是"import 它的类"）

★★ 为什么 DUT 必须是**子进程**而不是 import：本张要验的判据里有一条是
   "进程被杀 ⟹ setpoint 流停止"（ticket 05 验收项），而"杀一个对象"不是那件事。
   另外子进程的隔离让"被测节点的崩溃"变成可观测的 `returncode`，而不是 pytest 进程一起死。

★ 只断言**外部可观测的东西**：话题上出现了什么、服务的调用时刻、进程还活着没有。
  不 import DUT 的类、不读它的成员 —— 那是 seam 1 的定义（见 .scratch/phase1/spec.md）。
"""

from __future__ import annotations

import itertools
import os
import shutil
import signal
import subprocess
import threading
import time
from pathlib import Path
from typing import Callable, Dict, List, Optional, Tuple

from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from . import config as config_mod
from .config import default_config_file
from .fake_mavros import FakeMavros
from .fake_px4 import FakePx4
from .timeline import Timeline
from .vrpn_source import VrpnSource

DUT_STATE_TOPIC = "/control/state"

#: 同一个 pytest 进程里会创建多个 Scenario ⟹ 节点名要唯一（重名会让 ROS graph 里出现两个同名节点）
_INDEX = itertools.count()


class BenchTimeout(AssertionError):
    """等待判据超时。消息里带上证据（记录本身），而不只是"断言失败"。"""


class Scenario:
    """用法：

        with Scenario(dut=("control", "control_node"),
                      params={"offboard_prestream_s": 1.0}) as sc:
            assert sc.wait_for_subscribers("state"), "DUT 没接上 /mavros/state"
            sc.timeline.extend(silence_for(sc.fake, "state", at_s=4.0, duration_s=3.0))
            sc.hold(7.0)
            assert not sc.dut_alive() or sc.fake.setpoints  # 等

    也可以不用 with，自己 `close()`（别依赖 `__del__`）。
    """

    def __init__(
        self,
        dut: Optional[object] = None,
        params: Optional[Dict[str, object]] = None,
        config: Optional[Dict] = None,
        config_file: Optional[str] = None,
        start_dut: Optional[bool] = None,
        mocap: Optional[bool] = None,
        px4: Optional[bool] = None,
        dut_env: Optional[Dict[str, str]] = None,
        node_name: Optional[str] = None,
    ) -> None:
        #: DUT 的定位方式：`("包名", "可执行名")`，或一个可执行文件路径，或 None（只起夹具）。
        self.dut = dut
        self.params = dict(params or {})
        self._config_file = config_file if config_file is not None else default_config_file()
        self.config = config if config is not None else config_mod.load(self._config_file)
        # 这几个开关的默认值来自配置（`config/bench.yaml` 的 scenario 段），显式传参则覆盖。
        sc_cfg = self.config.get("scenario", {})
        self._start_dut_immediately = sc_cfg.get("start_dut", True) if start_dut is None else start_dut
        self._want_mocap = sc_cfg.get("mocap", True) if mocap is None else mocap
        # ★ 假 PX4 **默认关**：Phase 1 的 `(A)` 链（走 mavros）不需要它，而多起一个往
        #   `/fmu/*` 上发状态的夹具会让"这个场景到底在测哪条链"变得含糊 ——
        #   Phase 2 的用例显式写 `px4=True`（或配 `scenario.px4: true`）。
        self._want_px4 = sc_cfg.get("px4", False) if px4 is None else px4
        self._dut_env_extra = dict(dut_env or {})
        self._node_name = node_name
        self._index = next(_INDEX)

        self.node = None
        self.fake: Optional[FakeMavros] = None
        self.px4: Optional[FakePx4] = None
        self.mocap_source: Optional[VrpnSource] = None
        self.executor: Optional[MultiThreadedExecutor] = None
        self.timeline = Timeline()
        self.dut_proc: Optional[subprocess.Popen] = None

        self._spin_thread: Optional[threading.Thread] = None
        self._dut_lines: List[str] = []
        self._dut_reader: Optional[threading.Thread] = None
        self._t0 = time.monotonic()
        self._dut_started_at_s: Optional[float] = None
        self._closed = False

    # ─────────────────────────── 生命周期 ───────────────────────────

    def __enter__(self) -> "Scenario":
        import rclpy

        if not rclpy.ok():
            raise RuntimeError(
                "rclpy 还没初始化。台架夹具要求**会话级**地初始化一次 ROS —— "
                "请在包的 test/conftest.py 里挂上：\n"
                "    from bench.pytest_support import init_ros_for_tests, shutdown_ros_after_tests\n"
                "（别在每个用例里 init/shutdown，反复换 DDS context 是抖动的来源。）")

        # 安全边界：domain 0 上跑着真 mavros（对着真飞控），见 pytest_support 的说明。
        if int(os.environ.get("ROS_DOMAIN_ID", "0")) == 0:
            raise RuntimeError(
                "拒绝在 ROS_DOMAIN_ID=0 上跑夹具：那里有真的 mavros 与真飞控。"
                "见 bench/pytest_support.py。")

        self._t0 = time.monotonic()
        self.node = rclpy.create_node(
            self._node_name or f"bench_fixture_{os.getpid()}_{self._index}")

        initial = self.config["initial"]
        self.fake = FakeMavros(
            self.node,
            clock=self,
            rates_hz=self.config["rates_hz"],
            topics=self.config["topics"],
            connected=initial["connected"],
            armed=initial["armed"],
            mode=initial["mode"],
            position=initial["position"],
            voltage=initial["voltage"],
        )
        if self._want_px4:
            px4_cfg = self.config.get("px4", {})
            self.px4 = FakePx4(
                self.node,
                clock=self,
                topics=px4_cfg.get("topics"),
                armed=px4_cfg.get("armed", False),
                nav_state=px4_cfg.get("nav_state", 2),   # POSCTL：切 OFFBOARD 之前的常态
                pre_flight_checks_pass=px4_cfg.get("pre_flight_checks_pass", True),
            )
        if self._want_mocap:
            mocap_cfg = self.config["mocap"]
            self.mocap_source = VrpnSource(
                self.node,
                target=mocap_cfg["target"],
                body_name=mocap_cfg["body_name"],
                rate_hz=self.config["rates_hz"]["vrpn"],
                position=mocap_cfg["position"],
            )

        # DUT 的输出之一：当前状态串。transient_local ⟹ 晚订阅也能立刻拿到当前值。
        self._control_states: List[Tuple[float, str]] = []
        self._state_sub = self.node.create_subscription(
            String, DUT_STATE_TOPIC, self._on_control_state,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL))

        self.executor = MultiThreadedExecutor()
        self.executor.add_node(self.node)
        self._spin_thread = threading.Thread(
            target=self.executor.spin, name="bench-spin", daemon=True)
        self._spin_thread.start()

        if self._start_dut_immediately:
            self.start_dut()
        return self

    def __exit__(self, exc_type, exc, tb) -> bool:
        self.close()
        return False  # 绝不吞异常

    def close(self) -> None:
        """收尾。**任何路径都要走到这里**（测试失败、断言抛出、用例结束）。"""
        if self._closed:
            return
        self._closed = True

        if self.dut_proc is not None and self.dut_proc.poll() is None:
            self.stop_dut()
        if self._dut_reader is not None:
            self._dut_reader.join(timeout=2.0)

        if self.executor is not None:
            self.executor.shutdown(timeout_sec=2.0)
        if self._spin_thread is not None:
            self._spin_thread.join(timeout=5.0)
        if self.node is not None:
            self.node.destroy_node()
        self.executor = None
        self.node = None

    # ─────────────────────────── 时刻 ───────────────────────────

    def t(self) -> float:
        """相对**夹具就位**的秒数。`Timeline` 的 `at_s`、记录的 `at_s` 都是这把尺子。

        DUT 在夹具就位后立刻启动（`dut_started_at_s` 一般 < 0.1 s），它自己还要 ~0.2 s 才出数
        ⟹ 涉及 DUT 的断言请用**相对量**（例如"首条 setpoint"到"切模式"的差），
        不要在绝对时刻上卡边界 —— 那是在卡"DUT 多快启动"，不是在测判据。
        """
        return time.monotonic() - self._t0

    @property
    def dut_started_at_s(self) -> Optional[float]:
        return self._dut_started_at_s

    def advance_to(self, t_s: float, tick_s: float = 0.02) -> None:
        """推进到 `t_s`，途中按时序触发事件。不要用 `time.sleep` —— 事件不会被触发。"""
        if t_s < self.t():
            raise ValueError(f"时间不能倒退：现在 {self.t():.2f} s，要求 {t_s:.2f} s")
        while True:
            self._fire_due()
            now = self.t()
            if now >= t_s:
                return
            time.sleep(min(tick_s, t_s - now))

    def hold(self, seconds: float, tick_s: float = 0.02) -> None:
        """从当前时刻再推进 `seconds` 秒。"""
        self.advance_to(self.t() + seconds, tick_s=tick_s)

    def _fire_due(self) -> None:
        for ev in self.timeline.due(self.t()):
            self.log(f"[t={self.t():.2f}s] {ev.label or '(未命名事件)'}")
            ev.action()

    def log(self, message: str) -> None:
        print(f"[bench] {message}", flush=True)

    # ─────────────────────────── DUT 子进程 ───────────────────────────

    def start_dut(self) -> subprocess.Popen:
        from .pytest_support import dut_env  # 避免 import 期就碰 rclpy/os.environ

        exe = self._resolve_dut()
        args = [exe]
        ros_args: List[str] = []
        for key, value in sorted(self.params.items()):
            ros_args += ["-p", f"{key}:={_param_value(value)}"]
        if ros_args:
            args += ["--ros-args"] + ros_args

        self._dut_started_at_s = self.t()
        self.log(f"[t={self._dut_started_at_s:.2f}s] 启动 DUT：{' '.join(args)}")
        self.dut_proc = subprocess.Popen(
            args,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            env=dut_env(self._dut_env_extra),
            # ★ 独立进程组：收尾时能整组 SIGINT/SIGKILL，不会漏掉 DUT 自己拉起来的子进程。
            start_new_session=True,
            text=True,
            bufsize=1,
        )
        self._dut_reader = threading.Thread(
            target=self._read_dut_output, name="bench-dut-log", daemon=True)
        self._dut_reader.start()
        return self.dut_proc

    def _resolve_dut(self) -> str:
        if self.dut is None:
            raise ValueError("没有指定 DUT（`dut=('包名', '可执行名')` 或一个可执行文件路径）")
        if isinstance(self.dut, str):
            path = self.dut
            if not Path(path).exists():
                found = shutil.which(path)
                if found is None:
                    raise FileNotFoundError(f"DUT 不存在：{path}")
                return found
            return path
        package, executable = self.dut  # type: ignore[misc]
        try:
            from ament_index_python.packages import get_package_prefix
        except ImportError as exc:  # pragma: no cover
            raise RuntimeError("找不到 ament_index_python（要先 source install/setup.bash）") from exc
        try:
            prefix = get_package_prefix(package)
        except Exception as exc:
            raise FileNotFoundError(
                f"包 {package} 不在环境里（prefix 查不到）。先 `colcon build --packages-up-to "
                f"{package}` 并 source install/setup.bash —— "
                "否则这个场景会「静默地什么都没测」。") from exc
        path = Path(prefix) / "lib" / package / executable
        if not path.exists():
            raise FileNotFoundError(
                f"DUT 可执行文件不存在：{path}（包 {package} 的 prefix 是 {prefix}）")
        return str(path)

    def _read_dut_output(self) -> None:
        assert self.dut_proc is not None and self.dut_proc.stdout is not None
        for line in self.dut_proc.stdout:
            self._dut_lines.append(line)
        self.dut_proc.stdout.close()

    def dut_alive(self) -> bool:
        return self.dut_proc is not None and self.dut_proc.poll() is None

    def dut_returncode(self) -> Optional[int]:
        return self.dut_proc.poll() if self.dut_proc is not None else None

    def dut_log(self) -> str:
        return "".join(self._dut_lines)

    def stop_dut(self, sig: int = signal.SIGINT, timeout_s: float = 5.0) -> None:
        """给 DUT 发 `sig`（默认 SIGINT，rclcpp 会干净退出），不退就让位给 SIGTERM/SIGKILL。

        ★ SIGINT 优先是刻意的：验收项是"节点**主动**停止发布之后，流真的停了"，
          而 SIGKILL 什么都证明不了（那只是进程没了）。所以先礼后兵。
        """
        if self.dut_proc is None or self.dut_proc.poll() is not None:
            return
        for signum in (sig, signal.SIGTERM, signal.SIGKILL):
            self.log(f"[t={self.t():.2f}s] 给 DUT 发 {_sig_name(signum)}")
            _signal_group(self.dut_proc, signum)
            try:
                self.dut_proc.wait(timeout=timeout_s)
                self.log(f"[t={self.t():.2f}s] DUT 已退出，returncode = {self.dut_proc.returncode}")
                # ★ 读完输出**再**返回：进程退出 ⟹ 管道 EOF ⟹ 读线程随即结束，join 一下就够了。
                #   少了这一句，调用方紧跟的 `dut_log()` 可能只拿到**前半段**（第一手：ticket 06
                #   的收尾摘要明明打出来了 —— returncode 也是 0 —— 日志里却只有开头那行
                #   matplotlib 警告，看上去像"节点根本没打摘要"）。
                if self._dut_reader is not None:
                    self._dut_reader.join(timeout=2.0)
                return
            except subprocess.TimeoutExpired:
                continue
        self.log("⚠️ DUT 发了 SIGKILL 还没退出")

    # ─────────────────────────── 等待 ───────────────────────────

    def wait_for(
        self,
        predicate: Callable[[], bool],
        timeout_s: float = 10.0,
        message: str = "",
        tick_s: float = 0.02,
    ) -> None:
        """等判据成立；超时抛 `BenchTimeout`，消息里带上**证据**（记录与时序）。

        等待期间照常触发时序事件 ⟹ "等某条故障注入之后的后果"是安全的写法。
        """
        deadline = self.t() + timeout_s
        while True:
            self._fire_due()
            if predicate():
                return
            if self.t() >= deadline:
                raise BenchTimeout(
                    (message or "等待判据超时") + f"（等了 {timeout_s:g} s）\n\n" + self.evidence())
            time.sleep(tick_s)

    def poll(self, predicate: Callable[[], bool], timeout_s: float = 10.0) -> bool:
        """`wait_for` 的不抛异常版（给"断言它**不**发生"用）。"""
        try:
            self.wait_for(predicate, timeout_s=timeout_s, message="poll")
            return True
        except BenchTimeout:
            return False

    def wait_for_subscribers(self, topic_key: str, count: int = 1, timeout_s: float = 10.0) -> bool:
        """等 DUT 在某条话题上**匹配**上（含 QoS 兼容性）。见 `FakeMavros.matched_subscribers`。

        ★ 超时且 DUT **已经死了** ⟹ 当场抛出它的日志，而不是返回 `False`。
          两者在断言里长得一模一样（"DUT 没有接上 /mavros/xxx"），但根因完全不同：
          一个是接口不对，另一个是**节点根本没起来**（参数名/类型写错、启动即崩）。
          不查活着与否，第二条会被读成第一条 —— 排查方向正好岔开，成本全在最后那一步。
        """
        assert self.fake is not None
        if self.fake.wait_for_subscribers(topic_key, count=count, timeout_s=timeout_s):
            return True
        if self.dut_proc is not None and not self.dut_alive():
            raise AssertionError(
                f"DUT 进程在接上 /mavros 的 {topic_key} 之前就退出了"
                f"（returncode = {self.dut_returncode()}）—— 这不是「话题名不对」，是节点没起来；"
                f"参数名/类型写错会让 rclcpp 在构造参数时直接退进程。\n\n"
                f"---- DUT 输出（末尾）----\n{self.dut_log()[-2000:]}")
        return False

    def wait_for_setpoints(self, count: int = 1, timeout_s: float = 10.0) -> bool:
        assert self.fake is not None
        return self.poll(lambda: len(self.fake.setpoints) >= count, timeout_s=timeout_s)

    def wait_for_control_state(self, text: str, timeout_s: float = 10.0) -> bool:
        """等 `/control/state` 变成 `text`（形如 `MANUAL/STANDBY`）。"""
        return self.poll(lambda: self.control_state() == text, timeout_s=timeout_s)

    # ─────────────────────────── 取记录 ───────────────────────────

    def _on_control_state(self, msg: String) -> None:
        self._control_states.append((self.t(), msg.data))

    def control_state(self) -> Optional[str]:
        """最近一次 `/control/state`（一帧都没到过时返回 None）。"""
        return self._control_states[-1][1] if self._control_states else None

    def control_state_history(self) -> List[Tuple[float, str]]:
        return list(self._control_states)

    def setpoints(self) -> List:
        assert self.fake is not None
        return list(self.fake.setpoints)

    def setpoint_rate_hz(self, from_s: Optional[float] = None, to_s: Optional[float] = None) -> float:
        """某段窗口内实测的 setpoint 频率。

        ★ 只在**窗口上**算频率，不要断言相邻两帧的间隔：rcl 的定时器是"从回调开始时刻
          重新计时"（`rcl/timer.h`），回调偶尔超时就会立刻补跑 ⟹ 50 Hz 的流里出现
          0 ms 的间隔与 40 ms 的空洞都是正常的，而平均频率仍然对。
        """
        sps = self.setpoints()
        if not sps:
            return 0.0
        t_from = sps[0].at_s if from_s is None else from_s
        t_to = sps[-1].at_s if to_s is None else to_s
        n = sum(1 for s in sps if t_from <= s.at_s <= t_to)
        span = t_to - t_from
        return (n / span) if span > 0 else 0.0

    # ─────────────────────────── 证据 ───────────────────────────

    def evidence(self) -> str:
        """人话证据：DUT 状态、夹具记录、故障时序、DUT 日志尾部。失败时打在断言消息里。"""
        parts = [
            "## 台架场景证据",
            f"- DUT：{self.dut}；进程{'存活' if self.dut_alive() else f'已退出（returncode={self.dut_returncode()}）'}"
            f"；启动于 t={self._dut_started_at_s if self._dut_started_at_s is None else round(self._dut_started_at_s, 2)} s",
            f"- 时钟：现在 t={self.t():.2f} s（★ 所有时刻都相对「夹具就位」）",
            f"- `/control/state`：{self.control_state()!r}"
            f"（{len(self._control_states)} 条："
            + ", ".join(f"{t:.2f}s={s}" for t, s in self._control_states[-6:]) + "）",
            "",
        ]
        if self.fake is not None:
            parts += [self.fake.evidence(), ""]
        if self.px4 is not None:
            parts += [self.px4.evidence(), ""]
        if self.mocap_source is not None:
            parts += [
                f"- 动捕源：{self.mocap_source.topic}，匹配到的订阅者 "
                f"{self.mocap_source.matched_subscribers()}；"
                f"速度源：{self.mocap_source.velocity_topic}，匹配到的订阅者 "
                f"{self.mocap_source.matched_velocity_subscribers()}；"
                f" paused={self.mocap_source.paused} held_stamp={self.mocap_source.held_stamp}",
                "",
            ]
        parts += [self.timeline.describe(), ""]
        tail = self.dut_log().strip().splitlines()[-25:]
        parts += ["### DUT 日志（尾 25 行）", "```", *tail, "```"]
        return "\n".join(parts)

    def dump(self, path: str) -> str:
        """把证据写成文件（出事时手动留着看）。沿用 tools/prereq-evidence 的惯例：不自动写文档。"""
        Path(path).write_text(self.evidence(), encoding="utf-8")
        return path


# ─────────────────────────── 小工具 ───────────────────────────

def _param_value(value: object) -> str:
    """ROS 参数的命令行写法。bool 必须写成 `true`/`false`（rclcpp 认这两个字面量）。"""
    if isinstance(value, bool):
        return "true" if value else "false"
    if isinstance(value, float):
        return repr(value)
    return str(value)


def _signal_group(proc: subprocess.Popen, sig: int) -> None:
    """给整个进程组发信号（`start_new_session=True` ⟹ 组号 == pid）。"""
    try:
        os.killpg(os.getpgid(proc.pid), sig)
    except (ProcessLookupError, PermissionError):
        try:
            proc.send_signal(sig)
        except ProcessLookupError:
            pass


def _sig_name(sig: int) -> str:
    try:
        return signal.Signals(sig).name
    except ValueError:  # pragma: no cover
        return str(sig)
