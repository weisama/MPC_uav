#!/usr/bin/env python3
"""在**真 PX4** 上验 ticket 05 的六件事 —— 证据采集器。

跑的是一套固定场景，每个场景起一次 `nmpc/offboard_smoke.py`，把
「话题上的字节」与「飞机的物理状态」同时记下来，事后按场景给出判据。

**为什么要有这个脚本，而不是手敲命令**：ticket 05 要排除的四个静默失败
（`px4_msgs` 版本不匹配 / QoS 不兼容 / 时间戳单位错 / 推力符号与归一化错）
**都不报错、不崩溃**。区别只在"飞机有没有动、往哪动、多久不动"——
那需要**同时**有 PX4 侧的真值、DUT 侧的发送记录和一份可复核的时间线，
人眼盯控制台是拿不到的。

三路证据（**互不替代**）：

| 路 | 从哪来 | 用来回答 |
|---|---|---|
| DUT 侧 | `--log-file` 的 JSONL（每帧的**发出值**，含时间戳字段原值） | 我们到底发了什么 |
| PX4 侧 | pxh 控制台：`listener -n 1 <话题>`、`param show`、`uxrce_dds_client` 的启动行 | PX4 收到了什么（**反序列化之后**的值） |
| 物理 | `/fmu/out/vehicle_local_position`、`/fmu/out/vehicle_attitude` | 飞机照做之后变成了什么样 |

★ 「DUT 发的是 1.79e15、PX4 记下的是 2.1e7」这种对不上，**只有同时看前两路才看得出来**。
但**结论不是"单位错会被丢弃"**（那是本仓库早先的猜测，实测被推翻）：PX4 记下的值 =
`min(我们发的 − time_offset, hrt)`，`time_offset` 从 uXRCE 的时钟同步拿到（每秒刷新一次）。
于是

* 发**毫秒** ⇒ `uint64 − int64` **下溢**成 1.84e19 这种巨大正数 ⇒ 被 `min` 钳到 `hrt`；
* 发**纳秒**（1.79e18）⇒ 差不下去，但比 `hrt` 大 12 个数量级 ⟹ 同样被 `min` 钳到 `hrt`；
* 发 **0** ⇒ 走的是**显式**那一支（`if (timestamp == 0) timestamp = hrt`，连 `min` 都不过）；
  ⟹ 上面三档**记下的都恰好 = `hrt`**，心跳比正确单位还"新鲜" ⇒ **单位错在这一侧是静默的**；
* 发**微秒**（正确）⇒ 反序列化算出的 `ts − time_offset` 比 `hrt` 只**大**一个 Δ（Δ = 时钟同步滞后，
  实测每场 0.08 ~ 0.24 s）⟹ `min` 挑的还是 `hrt`（实测 age 0.004 s）。
  ★ 所以四档里**没有哪一档记下过我们发的那个数**：正确的 µs 也是被改写成 `hrt` 的，
  它跟三档错单位的区别不在"记下什么"，而在**把值往过去挪之后还能不能被挑中**（下一行）；
* 把微秒值**往过去挪** lag 秒 ⇒ 那个差值变成 `hrt + Δ − lag`：lag 还没 Δ 大时照样挑 `hrt`（age ≈ 0），
  **lag > Δ 才换支**去挑 `ts − time_offset` ⟹ age = lag − Δ（实测挪 1 s 得 0.916 s、挪 2 s 得 1.823 s）。
  于是挪 1 s **仍 < `COM_OF_LOSS_T`** ⟹ 照样进，只是**贴边**（余量就是 Δ 那一口气）；
  挪 **2 s** ⇒ age ≈ 1.8 s > 1 s ⇒ **进不了 OFFBOARD，且不报错** ——
  有效门槛是 `COM_OF_LOSS_T + Δ ≈ 1.1 ~ 1.3 s`，**不是 1 s**。三档错单位则**永远换不了支**
  （下溢值 1.84e19 / 纳秒 1.79e18 恒大于 `hrt`，`0` 连 `min` 都不过）⟹ 挪多少都进得去。
  ⚠️ 这几行曾被写成"挪 1 s 就进不去"（凭 Δ 之前的估算写的），且一度把正确 µs 写成"记下的是
  `ts − time_offset` 那一支"—— 两处都是实测推翻的，见 `docs/control/sitl-offboard-smoke.md` §3.3 的六行表。

用法（**先 source 好工作区**：`px4_msgs` 在仓库外）：

    source /opt/ros/humble/setup.bash && source ~/px4_msgs_ws/install/setup.bash
    source install/setup.bash
    python3 tools/px4-sitl/verify_offboard_smoke.py --only baseline,flow-break
    python3 tools/px4-sitl/verify_offboard_smoke.py --list
    python3 tools/px4-sitl/verify_offboard_smoke.py --json /tmp/offboard_smoke.json
    python3 tools/px4-sitl/verify_offboard_smoke.py --replay /tmp/offboard_smoke.json

⚠️ 这会**起一整套 SITL + MicroXRCEAgent**（仿真，不碰飞机），约 6–10 分钟。
"""

from __future__ import annotations

import argparse
import collections
import json
import os
import re
import shutil
import signal
import subprocess
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[1]
sys.path.insert(0, str(HERE / "z0-plant"))

from px4_console import Px4Console, Px4ConsoleError, parse_overlay_into_env  # noqa: E402

import rclpy  # noqa: E402
from px4_msgs.msg import (TimesyncStatus, VehicleAttitude,  # noqa: E402
                          VehicleLocalPosition, VehicleStatus)
from rclpy.node import Node  # noqa: E402
from rclpy.qos import (HistoryPolicy, QoSProfile,  # noqa: E402
                       ReliabilityPolicy)

NAV_STATE_OFFBOARD = 14
ARMING_STATE_ARMED = 2
#: 报告里把 nav_state 的数字翻成人话（取值见 `px4_msgs/VehicleStatus` 的枚举）
NAV_NAMES = {0: "MANUAL", 2: "POSCTL", 3: "AUTO_MISSION", 4: "AUTO_LOITER", 5: "AUTO_RTL",
             14: "OFFBOARD", 17: "AUTO_TAKEOFF", 18: "AUTO_LAND", 19: "AUTO_FOLLOW_TARGET"}


def nav_name(state):
    return f"{state}({NAV_NAMES.get(state, '?')})" if state is not None else "—"
#: PX4 的 `/fmu/out/*` 是 BEST_EFFORT；driver 侧也照 BEST_EFFORT 订，才有得看。
QOS_BE = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=50,
                    reliability=ReliabilityPolicy.BEST_EFFORT)


def fmt(v, spec=".3f"):
    return format(v, spec) if isinstance(v, (int, float)) else "—"


# ---------------------------------------------------------------- 记录器


class Recorder(Node):
    """把 PX4 报出来的状态按墙钟记成时间线 —— **判据全靠它**。

    ⚠️ 刻意**只存时间线、不存"当前值"**：回调时序与场景起止不是一回事，
    缓存"最后一次"会让判据跟实际发生顺序脱钩。
    """

    def __init__(self):
        super().__init__("verify_offboard_smoke")
        self.t0 = time.time()
        self.status = []      # (t, nav_state, arming_state)
        self.pos = []         # (t, x, y, z, vx, vy, vz)  ★ NED：z 向下为正
        self.nose = []        # (t, nose_down) —— 机头在水平线以下多深
        self.timesync = []    # (t, estimated_offset_ns)
        self.status_ts = []   # (t, PX4 发的 timestamp µs) —— 判"PX4→ROS 是 epoch µs"用
        self.counts = collections.Counter()
        self.create_subscription(VehicleStatus, "/fmu/out/vehicle_status", self._on_status, QOS_BE)
        self.create_subscription(VehicleLocalPosition, "/fmu/out/vehicle_local_position",
                                 self._on_pos, QOS_BE)
        self.create_subscription(VehicleAttitude, "/fmu/out/vehicle_attitude", self._on_att, QOS_BE)
        self.create_subscription(TimesyncStatus, "/fmu/out/timesync_status",
                                 self._on_timesync, QOS_BE)

    def now(self) -> float:
        return time.time() - self.t0

    def _on_status(self, m):
        self.counts["vehicle_status"] += 1
        self.status.append((self.now(), int(m.nav_state), int(m.arming_state)))
        # ★ PX4 发出来的 timestamp 在 DDS 边界上被加过 time_offset（`+ time_offset`，
        #   `uORB/ucdr/vehicle_status.h` 的序列化侧）⟹ 到我们手上就是 **epoch µs**。
        #   这是"时间戳用微秒"唯一**可证伪**的方向（反方向会被 PX4 改写，见模块 docstring）。
        self.status_ts.append((self.now(), int(m.timestamp)))

    def _on_pos(self, m):
        self.counts["vehicle_local_position"] += 1
        # ★ 一律 `float()` 落地：ROS 消息里的字段是 float32，直接塞进 JSON 会炸
        self.pos.append((self.now(), float(m.x), float(m.y), float(m.z),
                         float(m.vx), float(m.vy), float(m.vz)))

    def _on_att(self, m):
        self.counts["vehicle_attitude"] += 1
        # q 是「FRD 机体 → NED 世界」的四元数（w-first）。机体 x 轴在世界 NED 里的 z 分量
        # = 2(xz − wy)（旋转矩阵的 [2][0]）：>0 表示机头**低于**水平线（低头）。
        w, x, y, z = (float(v) for v in m.q)
        self.nose.append((self.now(), 2.0 * (x * z - w * y)))

    def _on_timesync(self, m):
        self.counts["timesync_status"] += 1
        self.timesync.append((self.now(), int(m.estimated_offset)))

    # ---- 查询（都按时间线回答） ----

    def first_nav(self, state, since=0.0):
        return next((t for t, nav, _a in self.status if t >= since and nav == state), None)

    def first_not_nav_after(self, state, t_from):
        return next(((t, nav) for t, nav, _a in self.status if t >= t_from and nav != state), None)

    def armed_ever(self, t_from, t_to):
        """★ **必须带窗口**：每个场景一套 PX4，而记录器是整个跑程共用的一个实例 ——
        不加窗口就会把上一个场景的"解锁过"算到这一个头上（第一版就是这么错的）。"""
        return any(a == ARMING_STATE_ARMED for _t, _n, a in self.window(self.status, t_from, t_to))

    def window(self, series, t_from, t_to):
        """时间窗内的**原样**样本（元组长度随话题不同，这里不做解包）。"""
        return [s for s in series if t_from <= s[0] <= t_to]

    def status_changes(self, t_from, t_to):
        """把状态时间线压成**变化点**（0.1 s 内连续同值算一次）—— 判据里"整场没见过 X"
        的说法要能被复核，所以每一处跳变都得留下时刻与去向。"""
        out = []
        for t, nav, arm in self.window(self.status, t_from, t_to):
            if out and (nav, arm) == out[-1][1:]:
                continue
            if out and t - out[-1][0] < 0.05:
                out[-1] = (t, nav, arm)
                continue
            out.append((t, nav, arm))
        return out

    def physics(self, t_from, t_to, hz=10.0):
        """抽稀的物理时间线 `[t, z_NED, vx, nose_down]` —— 判据背后的**原始轨迹**。"""
        step = 1.0 / hz
        pos = self.window(self.pos, t_from, t_to)
        nose = {round(t, 2): v for t, v in self.window(self.nose, t_from, t_to)}
        out, last = [], -1e9
        for t, _x, _y, z, vx, _vy, _vz in pos:
            if t - last < step:
                continue
            last = t
            out.append([round(t, 3), round(z, 4), round(vx, 4),
                        round(nose.get(round(t, 2), 0.0), 5)])
        return out

    def z_range(self, t_from, t_to):
        zs = [p[3] for p in self.pos if t_from <= p[0] <= t_to]
        return (min(zs), max(zs)) if zs else (None, None)

    def vx_mean(self, t_to, window_s=1.0):
        vs = [p[4] for p in self.pos if t_to - window_s <= p[0] <= t_to]
        return sum(vs) / len(vs) if vs else None

    def nose_mean(self, t_to, window_s=1.0):
        """窗口里"机头在水平线以下多深"的**均值** —— 判倾角方向只能用它，不能用极值。

        ★ 加速度计噪声 σ=1.7/1.4 m/s²（`sih.cpp:440`，`noiseGauss3f(0.5, 1.7, 1.4)`），
        折算成倾角就是 0.1 rad 量级；而阶跃只给 0.25 s × 0.5 rad/s = **0.125 rad** 的倾角。
        取 `max/min` 等于在取噪声的极值 —— 正反两个场景都能"测"出想要的方向（假的）。
        均值把零均值噪声压下去，留下**持续**的倾角（阶跃之后 PX4 保着姿态飞，倾角一直在）。
        """
        vs = [p[1] for p in self.window(self.nose, t_to - window_s, t_to)]
        return sum(vs) / len(vs) if vs else None


# ---------------------------------------------------------------- DUT 进程


class Dut:
    """被测节点：`ros2 run nmpc offboard_smoke.py …`，独立进程组（要能整组 SIGKILL）。"""

    def __init__(self, argv, log_file):
        self.argv = argv
        self.log_file = Path(log_file)
        self.proc = None

    def start(self):
        # ★ 先把上一场的日志删掉：`wait_first_frame` 判"节点发第一帧了没"就是读这个文件，
        #   而节点要等 rclpy 起来、参数解析完才 `open(..., "w")`（实测 1~2 s）——
        #   这期间盘上还躺着**上一场的旧文件**（1300 多行），`wait_first_frame` 会当场返回，
        #   采样随即在"发布者还没 advertise"时发出：`listener offboard_control_mode -n 1`
        #   只会回 `never published`（没有 `TOPIC:` 头），白等 15 s 超时。
        #   实测事故：baseline 的第一处采样站在 t=0.02 s，16 s 后才拿到数据 —— 那时飞机已经起飞，
        #   所谓"地面静息对照"其实是悬停读数。
        self.log_file.unlink(missing_ok=True)
        cmd = ["ros2", "run", "nmpc", "offboard_smoke.py"] + self.argv
        self.proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                     text=True, start_new_session=True)
        return self

    def __enter__(self):
        return self.start()

    def __exit__(self, *exc):
        """★ 一定要兜住：场景中途报错时若把节点留在盘上，它下一套世界还会连上来发心跳。"""
        if self.alive():
            self._signal_group(signal.SIGKILL)
            try:
                self.proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                pass

    def _signal_group(self, sig):
        if self.proc is None or self.proc.poll() is not None:
            return
        try:
            os.killpg(os.getpgid(self.proc.pid), sig)
        except (ProcessLookupError, PermissionError):
            pass

    def sigint(self, timeout=12.0):
        """正常收尾（节点会打印 summary）。返回它的 stdout。"""
        self._signal_group(signal.SIGINT)
        try:
            out, _ = self.proc.communicate(timeout=timeout)
            return out or ""
        except subprocess.TimeoutExpired:
            self._signal_group(signal.SIGKILL)
            out, _ = self.proc.communicate(timeout=5)
            return (out or "") + "\n[收尾超时，已 SIGKILL]"

    def sigkill(self):
        """★ 断流实验用：**不给收尾机会**。返回发出信号的墙钟时刻。"""
        t = time.time()
        self._signal_group(signal.SIGKILL)
        try:
            self.proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            pass
        return t

    def alive(self):
        return self.proc is not None and self.proc.poll() is None

    def frames(self):
        """读 JSONL。节点被 SIGKILL 掉也照样有 —— 每帧都 flush。"""
        if not self.log_file.exists():
            return []
        out = []
        for line in self.log_file.read_text(encoding="utf-8").splitlines():
            try:
                out.append(json.loads(line))
            except json.JSONDecodeError:
                pass
        return out


# ---------------------------------------------------------------- 辅助


def spin(px, rec, seconds):
    """推进墙钟：**既**喂 PX4 控制台、**也**转 ROS 回调（两个世界都得动）。"""
    end = time.time() + seconds
    while time.time() < end:
        px.pump(0.01)
        rclpy.spin_once(rec, timeout_sec=0.0)


def drain_both(px, rec, seconds):
    """同上，但短时间版：`sample_topic` 里读控制台的那 0.4~1.2 s 也要转 ROS 回调。

    ★ 踩过一次：采样期间只 `px.drain()`（不转 ROS），回调积压到采样结束才被处理，
    于是**记录器记下的"收到时刻"比真实晚了一整个采样窗口** ——
    "PX4→我们 的时间戳是 epoch µs"（判据 ③）当场从 −0.19 s 变成 −1.39 s，看起来像固件
    没加 time_offset，其实是我自己把回调饿着了。（采样是仪器的一部分，仪器不能改变被观测的东西。）
    """
    end = time.time() + seconds
    while time.time() < end:
        px.pump(0.01)
        if rec is not None:
            rclpy.spin_once(rec, timeout_sec=0.0)


def spin_until(px, rec, deadline):
    """把墙钟推到 `deadline`（已经在它之后就直接返回）。"""
    while time.time() < deadline:
        px.pump(0.01)
        rclpy.spin_once(rec, timeout_sec=0.0)
    return time.time()


def wait_first_frame(px, rec, dut, timeout=30.0):
    """等 DUT **发出第一帧**（也就是它的发布者已经 advertise 了）再往下走。

    ★ 踩过两次，代价各 15 秒（都是"前段采样跑到 15 s 之后"——那时飞机**已经起飞**，
    所谓"地面静息对照"其实是悬停读数，成对证据的两个状态就不再是两个状态了）：
      · 来源一（本次实测的机理）：`Dut.start()` 里没删旧日志时，这个函数读到的还是
        **上一场**的文件 ⟹ 当场返回、采样在发布者 advertise 之前就发出；
      · 来源二：pxh 的 `listener` 在"这个话题一个实例都没有"时**不打印 `TOPIC:` 头**，
        只打 `never published`（`listener_main.cpp:87-89`：先数 `orb_exists` 的实例数，
        为 0 就直接打印返回）⟹ `wait_for("^TOPIC: …")` 白等满超时。
    两处都堵上了：`Dut.start()` 先删旧文件，这里等第一帧落地，采样前话题必然已有发布者。
    """
    end = time.time() + timeout
    while time.time() < end:
        if dut.frames():
            return time.time()
        px.pump(0.01)
        rclpy.spin_once(rec, timeout_sec=0.0)
    return None


def wait_offboard(px, rec, t_before, timeout=20.0):
    """等到 PX4 **真的**进了 OFFBOARD，返回那个墙钟时刻（等不到返回 None）。

    ★ 指令程序的时间轴是"OFFBOARD 生效后"的秒（见 `nmpc/commands.py` 的第 1 条语义），
    所以真值采样要按**这一刻**对齐，而不是按场景开始的时刻。
    ★ 返回的是**记录到的那一刻**（`rec.t0 + t`），不是"我这一刻才看见"（`time.time()`）：
    前段几处控制台采样要花 2~4 s，进 OFFBOARD 往往发生在这中间（实测 2.98 s），
    用"我发现的时刻"会把整条真值轨迹往后推 4 s，请求的 0.3 s 那一点落到 +4 s 上。
    """
    end = time.time() + timeout
    while time.time() < end:
        px.pump(0.01)
        rclpy.spin_once(rec, timeout_sec=0.0)
        t = rec.first_nav(NAV_STATE_OFFBOARD, since=t_before)
        if t is not None:
            return rec.t0 + t
    return None


def truth_snapshot(px, rec=None):
    """SIH 的**真值** + PX4 **解出的**电机指令 —— 判据只认这两个，不认估计器。

    ★ 为什么不用 `/fmu/out/vehicle_local_position`：那套 SITL 里没有气压计、没有位置源
    （SIH 只发加速度计/陀螺/空速/测距，见 `sih.cpp:440-492`），z 是积分 σ=1.4 m/s² 的
    加速度计得来的；实测**地面静息**（没解锁、没推力）时真值恒为 +0.0002 m，EKF 却报
    −0.73 m。拿它判"掉不掉高度"会被估计器自己的漂移骗到。
    """
    gt = sample_topic(px, "vehicle_local_position_groundtruth", drain=0.4, rec=rec)
    mo = sample_topic(px, "actuator_outputs_sim", drain=0.4, rec=rec)
    # ★ 连**速度**一起取（`vx` 判"低头 ⟹ 向北加速"用）：这一套 SITL 没有位置源，
    #   EKF 的水平速度也是加速度计积出来的 —— 有真值就没理由拿估计器当尺子。
    return {"z": field(gt, "z"), "vz": field(gt, "vz"),
            "vx": field(gt, "vx"), "vy": field(gt, "vy"),
            "motors_mean": motor_mean(mo), "motors": motor_line(mo)}


def sample_topic(px, topic, timeout=15.0, attempts=2, drain=1.2, rec=None):
    """`listener <话题> -n 1` 抓**一帧**（`-n 1` 且不带 `-i` 时它打完就返回）。

    ⚠️ 选项必须写在**话题名之后**：`listener -n 1 offboard_control_mode` 会被 pxh 的
    `px4_getopt` 当成"话题名叫 `-n`"，回的是 `Topic -n did not match any known topics`
    （实测，第一版就是这么写的，表现是"控制台采样永远是空"）。
    ⚠️ 必须带 `-n 1`：不带的话它会一直刷（setpoint 是 50 Hz），把控制台淹掉。
    ⚠️ 拿的是「命令回显之后」的**整段**，不是 `TOPIC:` 行之后 —— 后者实测会拿到空串。
    ⚠️ 前提是**话题已经有过发布者**：`orb_exists` 数到 0 个实例时 pxh 只打
    `never published`（`listener_main.cpp:87-89`），**没有 `TOPIC:` 头** ⟹ 这里会白等到超时。
    所以调用方必须先 `wait_first_frame`（见那里记的 15 秒事故）。
    ⚠️ 游标用 `px.mark()`（**字节**游标）而不是 `len(px.text())`（渲染长度）：
    pxh 的 `ESC[2K\\r` 可以跨两次 `read()` 到达，渲染长度当游标会让新输出被整个跳过
    —— 症状同样是"超时、重试才成功"（见 `px4_console.text_since()`）。
    """
    for _ in range(attempts):
        mark = px.mark()
        px.send(f"listener {topic} -n 1", settle=0.05)
        try:
            px.wait_for_mark(rf"(?m)^TOPIC: {re.escape(topic)}", mark, timeout=timeout)
        except Px4ConsoleError:
            continue
        drain_both(px, rec, drain)
        chunk = px.text_since(mark)
        if field(chunk, "timestamp") is not None:
            return chunk
    return ""


def field(text, name):
    m = re.search(rf"^\s*{name}:\s*(-?[0-9.eE+-]+)", text or "", re.M)
    return float(m.group(1)) if m else None


def ts_age_s(text):
    """`listener` 自报的 `(x seconds ago)` —— 它算的是 `hrt_now − timestamp`。

    ★ 这一行是"心跳新不新鲜"的**第一手读数**，比我们自己推 `COM_OF_LOSS_T` 直接。
    """
    m = re.search(r"timestamp:\s*-?\d+\s*\((-?[0-9.eE+-]+)\s*seconds ago\)", text or "")
    return float(m.group(1)) if m else None


def flags(text):
    """`listener` 里那几个 bool 位，压成一行 `body_rate=True position=False …`。"""
    out = []
    for name in ("position", "velocity", "acceleration", "attitude", "body_rate", "actuator"):
        m = re.search(rf"^\s*{name}:\s*(\w+)", text or "", re.M)
        if m:
            out.append(f"{name}={m.group(1)}")
    return " ".join(out) or "—"


def rates_line(text):
    out = []
    for name in ("roll", "pitch", "yaw"):
        v = field(text, name)
        if v is not None:
            out.append(f"{name}={v:+.3f}")
    tb = re.search(r"^\s*thrust_body:\s*\[([^\]]*)\]", text or "", re.M)
    if tb:
        vals = [float(v) for v in tb.group(1).split(",")]
        out.append(f"thrust_body[2]={vals[2]:+.4f}")
    return " ".join(out) or "—"


def motor_values(text):
    """`actuator_outputs_sim` 的 `output: [...]`（16 个，四旋翼只用前 4 个）。"""
    m = re.search(r"^\s*output:\s*\[([^\]]*)\]", text or "", re.M)
    if not m:
        return None
    try:
        return [float(v) for v in m.group(1).split(",")]
    except ValueError:
        return None


def motor_line(text):
    """`actuator_outputs_sim` 的四个电机值 —— PX4 **解出来**的量（不是我们的请求）。"""
    vals = motor_values(text)
    if not vals:
        return "—"
    return " ".join(f"m{i}={v:+.3f}" for i, v in enumerate(vals[:4]))


def motor_mean(text):
    """四个电机指令的**均值**：垂向的总推力就是这个均值的 4 倍（`sih.cpp:303`）。

    ★ 均值才是"总推力"，四个值的差异是姿态控制在分配力矩（滚转/俯仰/偏航），
    所以判"推力给对了没有"只能看均值 —— 单看某一个电机会把力矩也算进去。
    """
    vals = motor_values(text)
    if not vals or len(vals) < 4:
        return None
    return sum(vals[:4]) / 4.0


def bool_field(text, name):
    """uORB 里的 bool 字段（`field()` 只认数字，`true/false` 认不出来）。"""
    m = re.search(rf"^\s*{name}:\s*(True|False|true|false)", text or "", re.M)
    if not m:
        return None
    return m.group(1).lower() == "true"


# ---------------------------------------------------------------- 场景表

#: 「悬停段」：起飞 1.2 s × 1.2 倍悬停推力，然后 5 s 只用悬停推力。
PROG_HOVER = ["lift:1.2:1.2:0,0,0", "hover:5:1.0:0,0,0"]
#: 「角速率阶跃」：0.25 s × ω_y = +0.5 rad/s（= §3 的低头方向），推力 1.15 倍保证还在空中。
PROG_STEP = ["step:0.25:1.15:0,0.5,0", "hover:3:1.15:0,0,0"]
PROG_ONLY_HOVER = ["hover:4:1.0:0,0,0"]
PROG_LONG = ["lift:1.2:1.2:0,0,0", "hover:20:1.0:0,0,0"]

SCENARIOS = {
    # ① 基准：µs 时间戳 + best_effort 订阅 + 正确符号（后面所有对照都是跟它比）
    #   ★ 高度判据读 **SIH 的真值**（`vehicle_local_position_groundtruth`，SIH 直接发的
    #   `_p_I(2)`），不读 EKF 的 `vehicle_local_position.z` —— 后者在**没解锁、没推力**的
    #   场景里也会漂到 −0.5…−0.7 m（实测），拿它判"掉不掉高"会被估计器自己的假象骗到。
    "baseline": dict(program=PROG_HOVER, args=[], watch=9.0,
                     samples=("offboard_control_mode", "vehicle_local_position_groundtruth",
                              "actuator_outputs_sim"),
                     samples_end=("vehicle_local_position_groundtruth",
                                  "actuator_outputs_sim"),
                     # ★ 真值轨迹：按"OFFBOARD 生效后的秒"采（指令程序的同一个时间轴）。
                     #   采一次要 ~0.5 s，所以"请求时刻"总早于"真实时刻"，
                     #   记录里存的是**真实**时刻。0～1.2 s 是段 `lift`（1.2 倍推力），
                     #   之后是 5 s 的 `hover`（1.0 倍）⇒ 想看清"升上去之后停住"的形状。
                     truth_track=(0.3, 0.9, 1.5, 2.5, 4.0, 6.0)),
    # ② QoS：订阅侧写成 RELIABLE。
    #   ★ 顺手做"高度仪器"的对照：同一时刻读 EKF 的 `vehicle_local_position` 和 SIH 的真值。
    #   这一场**没解锁、没推力**，两个数差多少 = 估计器自己能漂多少。
    "qos-reliable": dict(program=PROG_HOVER, args=["--qos", "reliable"], watch=6.0,
                         samples=("vehicle_local_position",
                                  "vehicle_local_position_groundtruth"),
                         samples_end=("vehicle_local_position",
                                      "vehicle_local_position_groundtruth")),
    # ③ 时间戳：单位错（ms/ns，**静默**）与往过去挪 1 s（**可达的**静默失败）
    "ts-ms": dict(program=PROG_HOVER, args=["--timestamp-unit", "ms"], watch=6.0,
                  samples=("offboard_control_mode",)),
    "ts-ns": dict(program=PROG_HOVER, args=["--timestamp-unit", "ns"], watch=9.0,
                  samples=("offboard_control_mode",)),
    # ts=0 走的是反序列化里**显式**的那一支（`if (timestamp == 0) timestamp = hrt`）
    "ts-zero": dict(program=PROG_HOVER, args=["--timestamp-unit", "zero"], watch=9.0,
                    samples=("offboard_control_mode",)),
    # 时间戳"往过去挪"：这才是**可达的**静默失败，而且门槛不是 1 s 而是 1 s − Δ
    # （Δ = uXRCE 时钟同步的滞后，0～1 s，实测 ≈0.18 s）。两个点一起跑，把这条边界钉住：
    "ts-lag-1s": dict(program=PROG_HOVER, args=["--timestamp-shift-us", "-1000000"], watch=9.0,
                      samples=("offboard_control_mode",)),
    "ts-lag-2s": dict(program=PROG_HOVER, args=["--timestamp-shift-us", "-2000000"], watch=9.0,
                      samples=("offboard_control_mode",)),
    # ④ 角速率阶跃：符号对 / 符号反（ticket 03 的判据）
    #   ★ 末段同时取 **SIH 真值**（`vx` 判"低头 ⟹ 向北加速"）与 EKF 的
    #   `vehicle_local_position`（交叉核对：没有位置源时 EKF 的水平速度可不可信，当场看）。
    "rate-step-ok": dict(program=PROG_STEP, args=[], watch=5.0,
                         samples=("vehicle_rates_setpoint",),
                         samples_end=("vehicle_local_position_groundtruth",
                                      "vehicle_local_position")),
    "rate-step-broken": dict(program=PROG_STEP, args=["--break", "pitch-sign"], watch=5.0,
                             samples=("vehicle_rates_setpoint",),
                             samples_end=("vehicle_local_position_groundtruth",
                                          "vehicle_local_position")),
    # ⑤ 推力符号 / 归一化常数。★ 同时读 PX4 **解出来的电机指令**：
    #    `thrust_body[2]` 是"请求"，`actuator_outputs_sim` 才是"真给电机的量"。
    "thrust-sign-broken": dict(program=PROG_ONLY_HOVER, args=["--break", "thrust-sign"],
                               watch=6.0, samples=("actuator_outputs_sim",
                                                   "vehicle_local_position_groundtruth"),
                               samples_end=("vehicle_local_position_groundtruth",
                                            "actuator_outputs_sim")),
    "thrust-scale-broken": dict(program=PROG_ONLY_HOVER,
                                args=["--break", "thrust-scale", "--break-scale", "0.75"],
                                watch=4.0, samples=("actuator_outputs_sim",
                                                    "vehicle_local_position_groundtruth"),
                                samples_end=("vehicle_local_position_groundtruth",
                                             "actuator_outputs_sim")),
    # ⑥ 断流：飞到一半把节点杀掉
    "flow-break": dict(program=PROG_LONG, args=[], watch=9.0, kill_at=4.5),
}


# ---------------------------------------------------------------- 单个场景


def run_scenario(name, spec, px, rec, args):
    log_file = Path(args.outdir) / f"{name}.jsonl"
    log_file.parent.mkdir(parents=True, exist_ok=True)
    argv = ["--log-file", str(log_file), "--cmd-interval", "0.5"]
    for ph in spec["program"]:
        argv += ["--phases", ph]
    argv += spec["args"]

    print(f"[verify] 场景 {name} ← ros2 run nmpc offboard_smoke.py {' '.join(argv)}",
          file=sys.stderr)
    obs = {"name": name, "argv": argv, "program": spec["program"],
           "console_samples": {}, "console_samples_end": {}, "sample_times": {},
           "truth_track": [], "checks": []}

    # ★ 开窗口前先把 ROS 队列放干：上一套世界拆掉时（`World.__exit__` 先杀 PX4、再杀 agent）
    #   PX4 临终那条 `vehicle_status`（**它当时还在 OFFBOARD、还锁着**）可能刚进中间件队列，
    #   而我们直到这一套世界起完才转 ROS 回调 —— 它就在"第一次 spin"时才被收到，
    #   那一刻已经在窗口里了。实测：qos-reliable 窗口 +0.02 s 处凭空多出一条 `nav_state=14,
    #   armed`，把"收不到状态 ⟹ 不解锁、不进 OFFBOARD"这条判据伪证掉（本场景的 DUT
    #   一条状态都没收到，"进 OFFBOARD"只可能是上一场的鬼影）。
    spin(px, rec, 0.5)
    t_before = rec.now()
    with Dut(argv, log_file) as dut:
        # 等它发出第一帧：既是"进程真起来了"的阳性对照，也是 `listener` 能拿到 `TOPIC:`
        # 头的前提（见 `wait_first_frame`）。这一段飞机还没解锁，仍是"地面静息"。
        if wait_first_frame(px, rec, dut) is None:
            print(f"[verify] ⚠ {name}：30 s 内没等到 DUT 第一帧，采样/判据大概率跟着失真",
                  file=sys.stderr)

        kill_rel = None
        stdout = ""
        if spec.get("kill_at"):
            spin(px, rec, spec["kill_at"])
            kill_rel = dut.sigkill() - rec.t0
            obs["kill_wall"] = kill_rel
            spin(px, rec, max(0.5, spec["watch"] - spec["kill_at"]))
        else:
            for topic in spec.get("samples", ()):
                # ★ 采样的**墙钟**要当场记下来：判据说"PX4 读到的时间戳 = 现在"，
                #   而在 `evaluate()` 里取 `time.time()` 是几分钟之后，差得离谱。
                #   逐个话题都记：采样是要花时间的（每个 ~0.5~1.5 s），三个采样摊开
                #   一两秒，"这一帧是 liftoff 前还是 lift 段里采的"必须能事后核对
                #   —— 第一版只记了最后一个，结果把"lift 段里的电机指令"当成了地面静息值。
                obs["sample_wall_us"] = time.time() * 1e6
                obs["sample_times"][topic] = round(rec.now() - t_before, 2)
                obs["console_samples"][topic] = sample_topic(px, topic, rec=rec)
            track = spec.get("truth_track", ())
            if track:
                # ★ 真值轨迹：按"OFFBOARD 生效后的秒"对齐（= 指令程序的时间轴）。
                #   每采一次要 ~0.5 s，所以"请求时刻"必然早于"真实时刻"：
                #   存进记录的是**真实**时刻（`t`），请求值只用来排序/定序。
                t_entry = wait_offboard(px, rec, t_before)
                # ★ 一律换算到记录器的相对时钟（`rec.now()` 口径）再存：
                #   `wait_offboard` 返回的是 `time.time()` **绝对**墙钟，
                #   上一版直接拿它减 `t_before`（相对）⟹ 记下来是 1.79e9 这种鬼数。
                obs["offboard_enter_driver"] = (round(t_entry - rec.t0, 2)
                                                if t_entry else None)
                for t_rel in track:
                    spin_until(px, rec, (t_entry or time.time()) + t_rel)
                    snap = truth_snapshot(px, rec)
                    snap["t"] = round(rec.now() - t_before, 2)
                    snap["t_req"] = t_rel
                    obs["truth_track"].append(snap)
            else:
                spin(px, rec, spec["watch"])
            # ★ 末段再采一次：同一条话题在**起飞前**（地面静息）与**飞完之后**各一个读数，
            #   配对起来就是"高度没掉"的直接证据（指令程序走完不停发，末段一直保持）。
            for topic in spec.get("samples_end", ()):
                obs["console_samples_end"][topic] = sample_topic(px, topic, rec=rec)
                obs["sample_end_wall_us"] = time.time() * 1e6
            stdout = dut.sigint()
        frames = dut.frames()

    t_end = rec.now()
    nav_seen = sorted({f["nav_state"] for f in frames})
    dut_obs = {
        "stdout_tail": "\n".join(stdout.strip().splitlines()[-14:]),
        "frames": len(frames),
        "status_cb": frames[-1]["status_cb"] if frames else None,
        "nav_seen": nav_seen,
        "thrust_body_z_seen": sorted({round(f["thrust_body_z"], 4) for f in frames}),
        "pitch_seen": sorted({round(f["pitch"], 3) for f in frames}),
        "ts_sent": frames[len(frames) // 2]["ts"] if frames else None,
        "ts_unit": frames[len(frames) // 2]["ts_unit"] if frames else None,
        "armed_cmd": None, "mode_cmd": None,
    }
    if frames:
        span = (frames[-1]["wall_ns"] - frames[0]["wall_ns"]) / 1e9
        dut_obs["span_s"] = span
        dut_obs["actual_hz"] = (len(frames) - 1) / span if span > 0 else None
        # ★ 判据"切 OFFBOARD 之前 setpoint 已以 ≥2 Hz 发了 ≥1 s"：只用 **DUT 自己的日志**
        #   量 —— 首帧 → "第一帧看见 nav_state=14"之间的秒数与帧数。
        #   ⚠️ 别改用窗口时钟：窗口从 `ros2 run` 起进程**之前**就起算，冷启动慢的那一次
        #   （实测 ts-ns 14.6 s、ts-lag-1s 14.9 s）会让"进入 OFFBOARD"看起来像 17.7 s ——
        #   那是 python 起进程 + import 的耗时，不是 PX4 慢（同场 DUT 自报 2.8 s）。
        for i, f in enumerate(frames):
            if f["nav_state"] == NAV_STATE_OFFBOARD:
                pre = (f["wall_ns"] - frames[0]["wall_ns"]) / 1e9
                dut_obs["pre_stream_s"] = round(pre, 3)
                dut_obs["pre_stream_frames"] = i
                dut_obs["pre_stream_hz"] = round(i / pre, 1) if pre > 0 else None
                break
    for line in stdout.splitlines():
        if "解锁命令" in line:
            m = re.search(r"解锁命令 (\d+) 条、模式命令 (\d+) 条", line)
            if m:
                dut_obs["armed_cmd"], dut_obs["mode_cmd"] = int(m.group(1)), int(m.group(2))
    obs["dut"] = dut_obs
    # ★ 阳性对照里的阳性对照：**DUT 自己有没有跑起来**。
    #   踩过一次：`install/` 里的 `nmpc` 是旧的，`ros2 run` 因为"参数不认识"立刻退出，
    #   于是整场"什么都没发生" —— 判据一条条失败，看起来像被控对象的毛病，
    #   其实是夹具的。没有这一条，任何"没观察到 X"都能被这个错误伪造出来。
    if not frames:
        print(f"[verify] ☠ {name}：DUT 一帧都没发（进程没跑起来？）—— 末段输出：\n"
              + "\n".join("      " + ln for ln in dut_obs["stdout_tail"].splitlines()),
              file=sys.stderr)
        obs["dut_dead"] = True

    # ★ 一切时间比较都在**记录器的相对时钟**里做：`Dut.sigkill()` 返回的是绝对墙钟
    #   （`time.time()`），第一版把它直接喂给 `first_not_nav_after` / `z_range`，
    #   于是"断流后退出 OFFBOARD"永远判成"没观察到"（差 1.79e9 秒）。
    # ★ 本场窗口的上下界。它不只是给取证看的：下面 `last_in()` 取"最后一帧"依据的就是它
    #   （窗口口径必须只有**一处定义**，否则"这一场"与"取到的数"会各说各话）。
    obs["t_window"] = [t_before, t_end]
    obs["armed_ever"] = rec.armed_ever(t_before, t_end)
    obs["status_seen"] = sorted({n for _t, n, _a in rec.window(rec.status, t_before, t_end)})
    obs["status_changes"] = [[round(t - t_before, 2), n, a]
                             for t, n, a in rec.status_changes(t_before, t_end)]
    t_enter = rec.first_nav(NAV_STATE_OFFBOARD, since=t_before)
    obs["offboard_enter"] = t_enter
    # ★ 同一件事的两个口径，都得存：`offboard_enter` 跟记录器时钟（判据、`kill_rel` 用），
    #   `offboard_enter_rel` 跟**窗口**时钟（`truth_track` 的 `t`、`sample_times` 用）。
    #   只存一个的话，报告里"进入 OFFBOARD 于 22.47 s"和轨迹第一点"21.79 s"会互相打脸。
    obs["offboard_enter_rel"] = round(t_enter - t_before, 2) if t_enter is not None else None

    if kill_rel is not None:
        left = rec.first_not_nav_after(NAV_STATE_OFFBOARD, kill_rel)
        obs["offboard_exit_after_kill"] = (left[0] - kill_rel) if left else None
        obs["offboard_exit_state"] = left[1] if left else None

    if t_enter is not None:
        win_end = kill_rel or t_end
        obs["z_min"], obs["z_max"] = rec.z_range(t_enter, win_end)
        obs["nose_max"] = max((p[1] for p in rec.window(rec.nose, t_enter, win_end)),
                              default=None)
        obs["nose_min"] = min((p[1] for p in rec.window(rec.nose, t_enter, win_end)),
                              default=None)
        obs["vx_end"] = rec.vx_mean(win_end)
        obs["nose_mean_end"] = rec.nose_mean(win_end, window_s=2.0)
        obs["physics"] = rec.physics(t_enter, win_end)
    else:
        obs["z_min"], obs["z_max"] = rec.z_range(t_before, t_end)
        obs["nose_max"] = max((p[1] for p in rec.window(rec.nose, t_before, t_end)),
                              default=None)
        obs["nose_min"] = min((p[1] for p in rec.window(rec.nose, t_before, t_end)),
                              default=None)
        obs["vx_end"] = rec.vx_mean(t_end)
        obs["nose_mean_end"] = rec.nose_mean(t_end, window_s=2.0)
        obs["physics"] = rec.physics(t_before, t_end)

    # ★ 这两个必须**按本场窗口**取，不能用 `rec.xxx[-1]`：`rec` 是整个跑程共用的一个
    #   记录器（`main` 里只建一次），`[-1]` 会把**上一场**留下的那条算进本场 ——
    #   qos-reliable 一条 status 都没收到，却会带着 baseline 的时间戳字段进 `--json`，
    #   取证时看起来一切正常。`armed_ever` 当初加窗口就是为了同一个坑。
    def last_in(series):
        """本场窗口内这条序列的最后一帧（窗口取自 `obs["t_window"]`，见那里的注释）。"""
        rows = rec.window(series, *obs["t_window"])
        return rows[-1] if rows else None

    tsy = last_in(rec.timesync)
    obs["timesync_offset_ns"] = tsy[1] if tsy else None
    # ★ 反方向（PX4 → 我们）的时间戳：实测是 epoch µs（PX4 在序列化时加过 time_offset）。
    last_ts = last_in(rec.status_ts)
    obs["status_ts_us"] = last_ts[1] if last_ts else None
    obs["status_ts_vs_wall_s"] = ((last_ts[1] - (last_ts[0] + rec.t0) * 1e6) / 1e6
                                  if last_ts else None)
    # PX4 侧对同一话题的读数（listener 是**反序列化之后**的值，所以时间戳改写在这里现形）
    s = obs["console_samples"].get("offboard_control_mode")
    obs["px4_ts_seen"] = field(s, "timestamp")
    obs["px4_ts_age_s"] = ts_age_s(s)
    obs["px4_flags"] = flags(s)
    obs["px4_rates"] = rates_line(obs["console_samples"].get("vehicle_rates_setpoint"))
    # 数值单拎出来：判"FLU ω_y>0 ⟹ 线上 pitch 取反为负"（符号映射表 §2）要能比较
    obs["px4_pitch"] = field(obs["console_samples"].get("vehicle_rates_setpoint"), "pitch")
    obs["px4_motors"] = motor_line(obs["console_samples"].get("actuator_outputs_sim"))
    obs["px4_motors_mean"] = motor_mean(obs["console_samples"].get("actuator_outputs_sim"))
    # ★ 高度**真值**（SIH 发的 `_p_I(2)`，NED、相对起飞点）与同一时刻 EKF 的估计值。
    #   两个都留着：判"掉不掉高"用真值，EKF 那个只用来证明"为什么不能拿它判"。
    gt = obs["console_samples"].get("vehicle_local_position_groundtruth")
    obs["gt_z"] = field(gt, "z")
    obs["gt_vz"] = field(gt, "vz")
    ekf = obs["console_samples"].get("vehicle_local_position")
    obs["ekf_z"] = field(ekf, "z")
    obs["ekf_z_valid"] = bool_field(ekf, "z_valid")
    obs["gt_z_valid"] = bool_field(gt, "z_valid")
    # 末段（悬停若干秒之后）的同一批读数
    end = obs["console_samples_end"]
    obs["gt_z_end"] = field(end.get("vehicle_local_position_groundtruth"), "z")
    obs["gt_vz_end"] = field(end.get("vehicle_local_position_groundtruth"), "vz")
    obs["gt_vx_end"] = field(end.get("vehicle_local_position_groundtruth"), "vx")
    obs["ekf_vx_end"] = field(end.get("vehicle_local_position"), "vx")
    obs["ekf_z_end"] = field(end.get("vehicle_local_position"), "z")
    obs["px4_motors_end"] = motor_line(end.get("actuator_outputs_sim"))
    obs["px4_motors_mean_end"] = motor_mean(end.get("actuator_outputs_sim"))
    return obs, dut


class World:
    """一个场景的**整套外部世界**：PX4 SITL + MicroXRCEAgent + 临时参数覆盖层。

    ★ **每个场景一套**（不是一套跑到底）。第一版想省时间复用一套，结果被实测打回来：
    上一个场景收尾用的 `commander land` 把 PX4 留在 **AUTO_LAND（nav_state 18）**，
    而 PX4 **不允许从这个模式切进 OFFBOARD** —— 于是后面每个场景都"进不去 OFFBOARD"，
    看起来像"符号错了 / 时间戳错了"，其实是上一个场景的残留。
    （教训与 `verify_z0_plant.py` 里"每次先删 parameters.bson"是同一条：
    结果要可归因，前提是起手状态一样。）
    """

    def __init__(self, args):
        self.args = args
        self.agent = None
        self.px = None
        self.tmpdir = None

    #: 参数覆盖层用哪一份（相对 `z0-plant/params/`）。**子类改这一个属性就能换机型**
    #: —— B 档要跑"参数故意不对"的机型（`z0-mass+30.param` 等），而改 `_enter` 的代价是
    #: 把"起手清干净"那套逻辑复制一份（那份逻辑踩过一次坑，见类 docstring）。默认值保持
    #: 逐字不变 ⟹ A 档与冒烟档的行为一个字节都不动。
    plant_param = "z0.param"

    def _overlay_path(self) -> Path:
        """覆盖层文件的绝对路径（`Path`）。"""
        return HERE / "z0-plant" / "params" / self.plant_param

    def __enter__(self):
        # ★ `__enter__` 自己失败时（最常见：`wait_ready` 超时）`with` **不会**调 `__exit__`，
        #   于是 agent 进程、PX4 进程组、覆盖层临时目录全留在盘上，下一场起手就不干净
        #   （本类 docstring 那条"起手状态必须一样"的教训）。所以这里自己收尾再抛。
        try:
            return self._enter()
        except BaseException:
            self.__exit__()
            raise

    def _enter(self):
        self.agent = subprocess.Popen([self.args.agent_bin, "udp4", "-p", self.args.agent_port],
                                      stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT)
        self.tmpdir, env = parse_overlay_into_env(str(self._overlay_path()))
        env["PX4_SIM_MODEL"] = "sihsim_z0"
        env["PX4_SIMULATOR"] = "sihsim"
        self.px = Px4Console(cwd=self.args.build_dir, env=env)
        self.px.start()
        self.px.wait_ready(self.args.ready_timeout)
        self.px.drain(self.args.boot_settle)
        self.com_of_loss_t = self._read_param("COM_OF_LOSS_T")
        # ★ uXRCE 的时钟偏移：`session.time_offset / 1000`（µs）。它是反序列化时
        #   `min(ts − time_offset, hrt)` 里那个数，也是"我们发的 µs 会被记成什么"的
        #   唯一来源 —— 启动行里 PX4 自己会把它打出来（`uxrce_dds_client.cpp:311`）。
        found = re.search(r"synchronized with time offset\s+(-?\d+)\s*us", self.px.text())
        self.time_offset_us = int(found.group(1)) if found else None
        return self

    def _read_param(self, name):
        """`param show <名字>` 读一个参数。⚠️ 必须**真的发命令**：pxh 只在被问的时候才打印
        （第一版直接在启动输出里正则找 `COM_OF_LOSS_T`，拿到的一直是 `None`）。

        ★ 实测的原文长这样（`/tmp/probe_pxh.py` 打出来的 repr）：

            x   COM_OF_LOSS_T [281,483] : 1.0000

        ⟹ 两处坑，第一版两条都踩了：
          · 行首不是名字，是符号列（`x ` = 已使用、`+` = 已保存、`* ` = 未保存）；
          · 值在 `[281,483]` **之后**，而那个方括号里是**参数哈希与序号** ——
            上一版的正则把 `[281,483]` 里的 281 当成了值，于是"`COM_OF_LOSS_T` = 281 秒"。
        """
        mark = self.px.mark()          # ★ 字节游标，不是 `len(text())`：见 `text_since()`
        try:
            self.px.send(f"param show {name}", settle=0.5)
            self.px.drain(0.5)
        except Px4ConsoleError:
            return None
        chunk = self.px.text_since(mark)
        # ⚠️ 候选行里**必然**有一行是命令回显（`param show COM_OF_LOSS_T`），它没有值；
        #    只认"先出现 `] : 值`"的那一行，别拿回显当答案。
        cands = [ln for ln in chunk.splitlines() if re.search(rf"\b{re.escape(name)}\b", ln)]
        print(f"[verify] param show {name} ⟶ " + " | ".join(c.strip() for c in cands)
              if cands else f"[verify] param show {name} ⟶ （没打印）", file=sys.stderr)
        for text in cands:
            m = (re.search(r"\]\s*:\s*(-?[0-9.eE+-]+)", text)
                 or re.search(rf"{re.escape(name)}\s*[:=]\s*(-?[0-9.eE+-]+)", text))
            if m:
                return float(m.group(1))
        return None

    def __exit__(self, *exc):
        # 三件都要能容忍"还没轮到建它就失败了"（`__enter__` 失败时会从这里收尾）
        if self.px is not None:
            self.px.close()
        if self.agent is not None:
            self.agent.terminate()
            try:
                self.agent.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self.agent.kill()
        if self.tmpdir:
            shutil.rmtree(self.tmpdir, ignore_errors=True)


# ---------------------------------------------------------------- 判据

#: 判据里的常数都**从 z0 机型文件读**（不是抄进代码的魔数）：`SIH_MASS`、`SIH_T_MAX` 是
#: SITL 起飞时真正加载的那一份，改机型文件这里的判据跟着变。
AIRFRAME = HERE / "z0-plant" / "airframes" / "10046_sihsim_z0"
G0 = 9.80665


def airframe_param(name):
    """从入库的 z0 机型文件里读一个 `param set-default`。取不到返回 None。"""
    m = re.search(rf"(?m)^\s*param set-default\s+{re.escape(name)}\s+(-?[0-9.eE+-]+)",
                  AIRFRAME.read_text(encoding="utf-8"))
    return float(m.group(1)) if m else None


def thrust_from_motors(motors_text):
    """四个电机指令 ⟹ 总推力（N）。

    ★ 权威是 `sih.cpp:303`：`_T_B = (0, 0, −T_MAX·(u0+u1+u2+u3))`，
    其中 `u[i] = actuators_out.output[i]`（`sih.cpp:290`）——
    所以"总推力 = 四个电机指令之和 × 单电机最大推力"，均值×4 就是那条式子。
    """
    mean = motor_mean(motors_text)
    t_max = airframe_param("SIH_T_MAX")
    return None if mean is None or t_max is None else mean * 4.0 * t_max


def track_line(track):
    """真值轨迹压成一行（判据的 detail 用）。"""
    return "  ".join(f"t={s['t']:.1f}s z={fmt(s['z'], '+.2f')}" for s in track) or "—"


def altitude_held(gt_z_end, gt_vz_end):
    """⑤ 的判据本体：**在地面之上，且没有在掉**。

    ★ 为什么单拎成一个函数（而不是写在 `ck(...)` 的参数里）：这里有个**假的零**。
    `vz == 0.0` 是**完全合法**的读数 —— 它正是"稳住了"最好的那个数，但 `x or 缺省`
    这种写法会把它当成"没测到"（`0.0` 是假值），于是本该 ✅ 的一帧被判 ❌，
    而且只在**飞得最稳**的那一次出现。判据里的 None 必须与 0.0 分开写。
    （`test_verify_judges.py` 把这两条都钉住：`vz=0.0` 要通过、`vz=None` 要不通过。）
    """
    if gt_z_end is None or gt_vz_end is None:
        return False
    return gt_z_end < -0.25 and gt_vz_end > -0.05


def evaluate(obs):
    """把观测翻成判据。**仪器与判据分开**：JSON 里存的是观测，改判据不必重跑 SITL。

    ★ 高度一律看 **SIH 真值**（`vehicle_local_position_groundtruth`）与 **PX4 解出的电机指令**，
    不看 `/fmu/out/vehicle_local_position` 的 z —— 这套 SITL 没有气压计/位置源，EKF 的 z
    是积分 σ=1.4 m/s² 的加速度计得来的，**地面静息**时就能漂到 −0.7 m（见 qos-reliable 的
    "仪器体检"）。用估计器当尺子，会把尺子自己的漂移记成飞机的高度变化。
    """
    c = []

    def ck(name, ok, detail, counts=True):
        c.append({"name": name, "ok": bool(ok), "detail": detail, "counts": counts})

    def note(name, detail):
        """一行**不参与计数**的说明 —— `ok` 恒真，所以它不是证据，只是"为什么要看这一场"。

        ★ 别用 `ck(name, True, detail)` 代替：报告与表格里的"N/M 通过"是按**能失败的**判据
        数的，混进恒真项那个数就不再是证据量（`--table` 的那张表是交付文档的一部分，
        数字虚高会被当成"多验了几条"）。
        """
        c.append({"name": name, "ok": True, "detail": detail, "counts": False})

    n = obs["name"]
    if obs.get("dut_dead"):
        # 夹具坏了就别谈判据了：整场的"什么都没发生"都是这个错误伪造的
        ck("☠ 本场景无效：DUT 一帧都没发（`ros2 run nmpc` 没跑起来）", False,
           "末段输出：" + (obs["dut"].get("stdout_tail") or "").replace("\n", " ⏎ ")[-300:])
        return c

    enter = obs["offboard_enter"]
    dut = obs["dut"]
    hz = dut.get("actual_hz")
    truth = obs.get("truth_track") or []
    gt_z, gt_vz = obs.get("gt_z"), obs.get("gt_vz")
    gt_z_end, gt_vz_end = obs.get("gt_z_end"), obs.get("gt_vz_end")
    # ⚠️ 只用**末段**那一处电机读数：前段那处是控制台采样的顺序决定的，实测落在
    #   `OFFBOARD 生效后 +0.34 s`（还在 lift 段里），把它当地面静息值会读出 1.2×悬停 ——
    #   判"悬停推力 ⟹ 不掉高"必须用末段（飞机已稳定悬停）+ 真值轨迹。
    motors_end = obs.get("px4_motors_end")
    # 指令程序给过的几档：最小的那一档是悬停（1.0），最大的是"抬升"（1.2）
    sent = [abs(v) for v in dut.get("thrust_body_z_seen", [])]
    req_hover = min(sent) if sent else None
    req_max = max(sent) if sent else None
    tmax_total = 4.0 * (airframe_param("SIH_T_MAX") or 0.0)
    weight = (airframe_param("SIH_MASS") or 0.0) * G0
    # ★ 反序列化器**会**算的那个数：`(ts − time_offset) mod 2^64`（`timestamp` 是 uint64、
    #   `time_offset` 是 int64 ⇒ 无符号运算），随后再被 `min(·, hrt)` 挑一次。
    #   ⚠️ **别**把它跟 `listener` 读回来的 `timestamp` 相减来"量挪了几秒"：`dut["ts_sent"]`
    #   是整段日志**中位数那一帧**的发送值，`px4_ts_seen` 是**采样那一帧**记下的值，两帧
    #   隔好几秒，相减等于在比两个不同时刻（上一版就这么算的，"挪 1 s"量出 −2.27 s，且挪
    #   1 s 与挪 2 s 量出同一个数 —— 那个"与挪几秒无关"本身就是算法错的自证）。
    #   ✔ 只用量**同一帧**里的两个数：`listener` 自报的 age（`px4_ts_age_s`）与挪的秒数。
    #     挪 lag 秒后 age = lag − Δ（见 ts-lag 分支），Δ 才是反序列化器的时钟误差。

    if n == "baseline":
        ck("① px4_msgs 与固件对得上：**PX4 真的进了 OFFBOARD**", enter is not None,
           f"首次 nav_state=14 于 t={fmt(enter, '.2f')}s" if enter is not None
           else f"整场没见过 14 —— 只见过 {obs['status_seen']}")
        # ★ 只从 DUT 自己的日志量（首帧 → 第一帧看见 14）：见 `dut_obs` 里记的窗口时钟坑。
        pre, pre_hz = dut.get("pre_stream_s"), dut.get("pre_stream_hz")
        ck("① 拉进 OFFBOARD 之前，setpoint 已以 ≥2 Hz 持续发布 ≥1 s",
           pre is not None and pre >= 1.0 and (pre_hz or 0) >= 2.0,
           f"DUT 自报：首帧 → PX4 回话 nav_state=14 隔了 {fmt(pre, '.3f')} s，"
           f"其间 {dut.get('pre_stream_frames')} 帧 ⟹ {fmt(pre_hz, '.1f')} Hz"
           f"（闸门 `--pre-stream` 默认 1.5 s；★ 这一条只能这样量：窗口时钟从"
           f"`ros2 run` 起进程之前起算，冷启动慢的那次会把它读成 17.7 s）")
        ck("④ 心跳速率：判据 ≥2 Hz，实做 ≈50 Hz", (hz or 0) >= 45.0,
           f"{dut['frames']} 帧 / {fmt(dut.get('span_s'), '.2f')}s ⟹ {fmt(hz, '.1f')} Hz")
        ck("② 订阅 QoS 对上了：收到 vehicle_status", (dut.get("status_cb") or 0) > 0,
           f"DUT 收到 {dut.get('status_cb')} 条（订阅侧 BEST_EFFORT）")
        ck("③ 我们→PX4：发的是 epoch **微秒**，PX4 记下的却是它自己的 hrt，心跳新鲜",
           (dut.get("ts_sent") or 0) > 1e14 and obs["px4_ts_seen"] is not None
           and obs["px4_ts_seen"] < 1e9 and (obs["px4_ts_age_s"] if obs["px4_ts_age_s"] is not None else 1.0) < 0.05,
           f"DUT 发 timestamp={dut.get('ts_sent')}（epoch µs）；PX4 记下 "
           f"{fmt(obs['px4_ts_seen'], '.0f')} µs、自报 "
           f"{fmt(obs['px4_ts_age_s'], '.6f')} s 前 —— ★ 记下的**不是我们发的那个数**，"
           f"是反序列化器改写后的值（`uORB/ucdr/<话题>.h`: "
           f"`ts = min(ts − time_offset, hrt)`；本地实例的 `time_offset`≈"
           f"{fmt(obs.get('time_offset_us'), '.0f')} µs）")
        ck("③ PX4→我们：`/fmu/out/*` 的时间戳是 **epoch 微秒**",
           obs.get("status_ts_vs_wall_s") is not None
           and abs(obs["status_ts_vs_wall_s"]) < 1.0,
           f"vehicle_status.timestamp 与墙钟差 {fmt(obs.get('status_ts_vs_wall_s'), '+.3f')} s"
           f"（若序列化没加 time_offset，这个差会是 ≈ −epoch ≈ −1.79e9 s）")
        ck("心跳内容：body_rate=True，其余位 False", obs["px4_flags"] ==
           "position=False velocity=False acceleration=False attitude=False "
           "body_rate=True actuator=False", obs["px4_flags"])
        ck("⑤ 悬停那一档的请求，PX4 **原样**解成了电机指令（中间没有偏置）",
           req_hover is not None and obs.get("px4_motors_mean_end") is not None
           and abs(req_hover - obs["px4_motors_mean_end"]) / req_hover < 0.02,
           f"我们发 |thrust_body[2]|={fmt(req_hover, '.4f')}（抬升那档 {fmt(req_max, '.4f')}）；"
           f"PX4 解出 {motors_end}，四路均值 {fmt(obs.get('px4_motors_mean_end'), '.4f')}"
           f"（差 {fmt(100 * (obs.get('px4_motors_mean_end') or 0) / req_hover - 100 if req_hover else None, '+.2f')} %）"
           f"；四路的差异是姿态环在分配力矩，所以只比均值")
        ck("⑤ 只给悬停推力 ⟹ **不掉高度**（尺子是 SIH 真值）",
           altitude_held(gt_z_end, gt_vz_end),
           f"真值 z：起飞前 {fmt(gt_z, '+.4f')} m ⟹ 末段 {fmt(gt_z_end, '+.3f')} m"
           f"（**在地面之上 {fmt(-(gt_z_end or 0), '.2f')} m**）、末段 vz="
           f"{fmt(gt_vz_end, '+.4f')} m/s。轨迹：{track_line(truth)}")
        ck("⑤ 从电机指令反推的推力 ≈ 机重 ⟹ 「掉不下来」是有数的（不是只看高度）",
           motors_end is not None and abs((thrust_from_motors(obs['console_samples_end'].get('actuator_outputs_sim')) or 0) - weight) < 0.1 * weight,
           f"末段电机 {motors_end} ⟹ T={fmt(thrust_from_motors(obs['console_samples_end'].get('actuator_outputs_sim')), '.3f')} N，"
           f"机重 m·g={fmt(weight, '.3f')} N（0.75 kg × 9.80665；T_max={fmt(tmax_total, '.2f')} N = 4×{fmt(airframe_param('SIH_T_MAX'), '.2f')}）")

    elif n == "qos-reliable":
        ck("② 订阅侧写 RELIABLE ⟹ **一条 vehicle_status 都收不到**",
           (dut.get("status_cb") or 0) == 0, f"DUT 收到 {dut.get('status_cb')} 条")
        note("  ★ 阳性对照：baseline 同一个订阅**收到过**回调 —— 所以这不是「本来就没数据」",
             f"本场 {dut.get('status_cb')} 条；对照本身在 `report()` 里跨档做（那一处是能失败的），"
             f"**但得本批同时跑了 baseline 才做得成** —— 只跑本场时报告会明说"
             f"「这条对照**没做**」（`cross_qos_control` 的 `None` 那一支），本行只是把话写在旁边")
        ck("于是也拉不进 OFFBOARD（收不到状态 ⟹ 不解锁、不切模式）", enter is None,
           f"DUT 侧见过的 nav_state = {dut['nav_seen']}（−1 = 从没收到过状态）；"
           f"它一共发了 解锁命令 {dut.get('armed_cmd')} 条、模式命令 {dut.get('mode_cmd')} 条")
        ck("  ★ 仪器体检：这一场**没解锁、没推力**，真值说飞机一直贴地，EKF 的 z 却在漂",
           gt_z_end is not None and abs(gt_z_end) < 0.05
           and obs.get("ekf_z_end") is not None and abs(obs["ekf_z_end"]) > 0.2,
           f"两处同一对比（看的是**同一时刻**的真值 vs EKF）："
           f"刚开机 z 真值 {fmt(gt_z, '+.4f')} m、EKF {fmt(obs.get('ekf_z'), '+.3f')} m"
           f"（还没漂开，所以只在前段量是量不到的）；"
           f"末段 z 真值 {fmt(gt_z_end, '+.4f')} m（z_valid={obs.get('gt_z_valid')}）、"
           f"EKF {fmt(obs.get('ekf_z_end'), '+.3f')} m（z_valid={obs.get('ekf_z_valid')}）"
           f"⟹ 差 {fmt((obs.get('ekf_z_end') or 0) - (gt_z_end or 0), '+.2f')} m。"
           f"★ 没气压计/位置源，EKF 的 z 只能把 σ=1.4 m/s² 的加速度计噪声积成**随机游走**"
           f"（越久越大，静止十几秒就到半米）；拿它当尺子 = 把尺子自己的漂移记成飞机的高度变化。"
           f"这就是 ⑤ 判高度只看 SIH 真值 + PX4 解出的电机指令的理由")

    elif n in ("ts-ms", "ts-ns", "ts-zero"):
        unit = n.split("-")[1]
        sent_ts = dut.get("ts_sent")
        expect_sent = {"ms": (1e11, 1e13), "ns": (1e17, 1e19), "zero": (0, 0)}[unit]
        why = {
            "ms": "1.79e12（ms）− 1.79e15（time_offset）< 0 ⟹ **uint64 下溢**成大得离谱的正数",
            "ns": "1.79e18（ns）− 1.79e15 > 0（这次没下溢），但比 hrt 大 12 个数量级",
            "zero": "0 走的是**显式**那一支：`if (timestamp == 0) timestamp = hrt`（连 `min()` 都不过）",
        }[unit]
        ck("③ 这一档是**静默无害**的：PX4 照样进 OFFBOARD，天上一切正常", enter is not None,
           f"首次 nav_state=14 于 t={fmt(enter, '.2f')}s；真值末段 z={fmt(gt_z_end, '+.3f')} m")
        ck("  ↑ 机制：反序列化器挑中的不是我们发的数，而是 hrt", 
           (obs.get("px4_ts_seen") or 0) < 1e9 and (obs["px4_ts_age_s"] if obs["px4_ts_age_s"] is not None else 1.0) < 0.05
           and sent_ts is not None and expect_sent[0] <= sent_ts <= expect_sent[1],
           f"DUT 发 timestamp={sent_ts}（{unit}）；{why}；`min(它, hrt)` 于是挑了 hrt = "
           f"{fmt(obs.get('px4_ts_seen'), '.0f')} µs，自报 {fmt(obs.get('px4_ts_age_s'), '.6f')} s 前 "
           f"—— **单位错比正确单位还「新鲜」**，PX4 一声不吭")
        note("  ★ 订正：交付文档 §6.5 原写「发错单位会被丢弃」，实测**不会** —— 没有丢弃，也没有提示",
             "证据就是本场那两条（进 OFFBOARD + 记下的 ts = hrt）：单位错被静默无害化了，"
             "不是被挡住了；本行是结论，不单独算一条判据")

    elif n in ("ts-lag-1s", "ts-lag-2s"):
        lag = float(n.split("-")[-1][0])
        tol = obs.get("com_of_loss_t")
        age = obs.get("px4_ts_age_s")
        ck("订阅与解锁都正常 —— **问题只在时间戳**",
           (dut.get("status_cb") or 0) > 0 and obs["armed_ever"],
           f"status 回调 {dut.get('status_cb')} 条，armed={obs['armed_ever']}，"
           f"DUT 侧见过 nav_state {dut['nav_seen']}")
        refused = enter is None and NAV_STATE_OFFBOARD not in dut["nav_seen"]
        if lag == 1:
            # ★ 实测**推翻了**"挪 1 s 就进不去"：记下的 age = 1 s − Δ（Δ 是同步滞后，实测 ≈0.18 s）
            #   ⟹ 还在 `COM_OF_LOSS_T`(1 s) 之内。余量只剩 Δ 这一口气。
            ck("③ 往过去挪 1 s ⟹ 心跳 age = 1 s − Δ ≈ 0.8 s，**仍 < `COM_OF_LOSS_T` ⟹ 照样进 OFFBOARD**",
               (not refused) and age is not None and tol is not None and age < tol,
               f"PX4 记下 {fmt(obs.get('px4_ts_seen'), '.0f')} µs、自报 age={fmt(age, '.3f')} s "
               f"（`COM_OF_LOSS_T`={fmt(tol, '.2f')} s ⟹ 余量只有 {fmt(tol - age, '.3f')} s = Δ）；"
               f"首次 nav_state=14 于 t={fmt(enter, '.2f')}s ⟹ 能进能飞，只是**贴边**")
        else:
            ck("③ 往过去挪 2 s ⟹ 心跳被永久判「不新鲜」⟹ **进不了 OFFBOARD**（全程不报错）",
               refused and age is not None and tol is not None and age > tol,
               f"整场没见过 14（PX4 侧只见 {obs['status_seen']}）；PX4 记下 "
               f"{fmt(obs.get('px4_ts_seen'), '.0f')} µs、自报 age={fmt(age, '.3f')} s > "
               f"`COM_OF_LOSS_T`={fmt(tol, '.2f')} s ⟹ `offboardCheck.cpp:46` 的 "
               f"`data_is_recent` 永假")
        # ★ 机制判据只用**同一帧**里的两个数：挪的秒数 lag 与 `listener` 自报的 age。
        #   反序列化器写回去的是 `min(ts − time_offset, hrt)` 里的较小者：
        #     挑 `hrt` 那一支 ⟹ 记下的 ts == hrt ⟹ age ≈ 0（ts-ms/ns/zero 三档就是这样）；
        #     挑 `ts − time_offset` 那一支 ⟹ 记下的 ts 落在 hrt **之前** ⟹ age > 0。
        #   本档 age 明显 > 0（且两档都 ≈ lag − 0.18 s）⟹ 挑中的正是后一支，没有歧义。
        delta = (lag - age) if age is not None else None
        if delta is not None:
            obs["ts_age_delta_s"] = delta
        ck("  ↑ 机制：记下的 ts 落在 hrt 之前 ⟹ `min()` 挑的是 `ts − time_offset` 那一支；"
           "age = 挪的秒数 − Δ（Δ = 时钟偏移误差，≈0.18 s）",
           delta is not None and 0.05 < delta < 0.5,
           f"DUT 发 timestamp={dut.get('ts_sent')}（epoch µs − {lag:.0f}e6）；PX4 记下 "
           f"{fmt(obs.get('px4_ts_seen'), '.0f')} µs ⟹ age={fmt(age, '.3f')} s > 0，"
           f"**不是** hrt 那一支（那一支 age 会 ≈0，见 ts-ms/ns/zero 三档）；"
           f"于是 Δ = 挪的秒数 − age = {fmt(delta, '.3f')} s —— 两档挪法量出的 Δ 一致，"
           f"说明它是**同步滞后**这一个常数，不是挪法带来的")

    elif n in ("rate-step-ok", "rate-step-broken"):
        # ★ 倾角方向只看**末段 2 s 的均值**，不看 max/min：加速度计噪声 σ=1.7/1.4 m/s²
        #   折算成倾角就是 0.1 rad 量级，而阶跃只给 0.125 rad —— 取极值等于在读噪声，
        #   正反两个场景都能"测"出自己想要的方向（那是自欺）。阶跃之后 PX4 保着姿态飞，
        #   倾角是**持续**的，均值刚好把它留下、把零均值噪声压掉。
        down = n == "rate-step-ok"
        nose, gt_vx = obs.get("nose_mean_end"), obs.get("gt_vx_end")
        ck("⑥ 角速率阶跃让飞机**低头**（FLU ω_y=+0.5，sign-map §3）" if down
           else "⑥（刻意把线上 pitch 写反）同一个 FLU ω_y=+0.5 让飞机**抬头**",
           (nose > 0.03) if down else (nose < -0.03),
           f"末段 2 s 机头均值 {fmt(nose, '+.4f')} rad（>0 = 机头在水平线以下）。"
           f"★ 不用 max/min：噪声 σ 折算 0.1 rad 量级、阶跃只有 0.125 rad，取极值就是在读噪声")
        ck("  ↑ 世界系后果：低头 ⟹ 沿机头方向（初始朝北）加速" if down
           else "  ↑ 世界系后果：抬头 ⟹ 向反方向加速",
           (gt_vx or 0) > 0.3 if down else (gt_vx or 0) < -0.3,
           f"末段 SIH 真值 vx={fmt(gt_vx, '+.3f')} m/s（同一时刻 EKF 的 vx="
           f"{fmt(obs.get('ekf_vx_end'), '+.3f')} —— 这套 SITL 没位置源，EKF 的速度是"
           f"加速度计积出来的，判方向只当真值用）")
        pitch_seen = dut.get("pitch_seen") or []
        ck("  ↑ 线上那一帧：FLU ω_y=+0.5 写成 PX4 的 `pitch` = −0.5（§2 的「取反」那一行）" if down
           else "  ↑ 线上那一帧（故障注入后）：同一个 FLU ω_y 写成 `pitch` = +0.5",
           (min(pitch_seen) < -0.3) if down else (max(pitch_seen) > 0.3),
           f"节点自己 log 里 `VehicleRatesSetpoint.pitch` 的取值集合 {pitch_seen}"
           f"（另一档是悬停段的 0.0）；§2 权威表：`pitch_rate = −1 × ω_y_FLU`。"
           f"★ 这一条**不用** pxh 读回的那一帧：OFFBOARD 之前这个 uORB 话题的「最新帧」是"
           f"**mc_att_control** 发的（`mc_att_control_main.cpp:319-324` 在姿态模式下每轮都发，"
           f"`mc_rate_control` 只在 acro 手动档回写，`:174`），`listener` 拿到的就是它；"
           f"而步进只持续 0.25 s，进 OFFBOARD 之后再去采已经采不到了"
           f"（实测读到 `{obs['px4_rates']}` —— 那是进 OFFBOARD 前姿态环的残帧）")

    elif n == "thrust-sign-broken":
        t_from_motors = thrust_from_motors(obs["console_samples_end"].get("actuator_outputs_sim"))
        ck("① 中间件那一层是好的：**照样进了 OFFBOARD**（故障只在符号）",
           enter is not None, f"首次 nav_state=14 于 t={fmt(enter, '.2f')}s")
        ck("⑤ 推力符号写反 ⟹ 请求变成**正**的 ⟹ 电机被压到最小，真值高度一直贴地",
           gt_z_end is not None and abs(gt_z_end) < 0.1
           and obs.get("px4_motors_mean_end") is not None
           and obs["px4_motors_mean_end"] < 0.05,
           f"我们发的 thrust_body[2] 是**正**的（{dut.get('thrust_body_z_seen')}）；"
           f"PX4 解出 {motors_end}（均值 {fmt(obs.get('px4_motors_mean_end'), '.4f')} ⟹ "
           f"T={fmt(t_from_motors, '.3f')} N）；真值 z：起飞前 {fmt(gt_z, '+.4f')} ⟹ 末段 "
           f"{fmt(gt_z_end, '+.4f')} m ⟹ **离不了地**")
        ck("  ↑ 判据用的是真值与电机指令 —— EKF 的 z 这一场会漂到 −0.6 m 上下，"
           "单看它会以为「升了一点」", gt_z is not None,
           f"真值末段 z={fmt(gt_z_end, '+.4f')} m，EKF 末段 z={fmt(obs.get('ekf_z_end'), '+.3f')} m")

    elif n == "thrust-scale-broken":
        ck("① 中间件那一层是好的：照样进了 OFFBOARD", enter is not None,
           f"首次 nav_state=14 于 t={fmt(enter, '.2f')}s")
        ck("⑤ 归一化常数错（T_max 少算 25%）⟹ 同一个悬停指令变成**猛爬**",
           req_hover is not None and req_hover > 0.75 and gt_z_end is not None and gt_z_end < -2.5,
           f"我们发 |thrust_body[2]|={fmt(req_hover, '.4f')}（正确应是 0.6233 = m·g/T_max，"
           f"因为 T_max 只当 75% 用，请求被放大 4/3）；PX4 解出 {motors_end}；"
           f"真值末段 z={fmt(gt_z_end, '+.3f')} m —— baseline 同一段时间只到 −1.58 m")

    elif n == "flow-break":
        ck("⑥ 断流前正常在 OFFBOARD 里悬停",
           enter is not None and obs.get("kill_wall") is not None,
           f"t={fmt(enter, '.2f')}s 进 OFFBOARD，t={fmt(obs.get('kill_wall'), '.2f')}s SIGKILL")
        d = obs.get("offboard_exit_after_kill")
        tol = obs.get("com_of_loss_t") or 1.0
        ck(f"⑥ 杀掉节点 ⟹ PX4 在 `COM_OF_LOSS_T`（{fmt(tol, '.2f')} s）之后退出 OFFBOARD",
           d is not None and tol * 0.8 <= d <= tol + 0.8,
           f"杀掉后 {fmt(d)} s 观察到 nav_state≠14，去向 "
           f"nav_state={nav_name(obs.get('offboard_exit_state'))}"
           f"（`COM_OBL_RC_ACT` 决定的失效动作）"
           if d is not None else "杀掉后再没观察到 nav_state 变化")

    return c


def counted(checks):
    """只留**能失败**的那些（`counts` 缺省 True，所以老 JSON 重放也照旧）。

    "N/M 通过"里的 M 是证据条数；`note()` 那种恒真的说明行不算 —— 它进不了分母。
    """
    return [c for c in checks if c.get("counts", True)]


def cross_qos_control(cb_by_name):
    """跨档 QoS 阳性对照 —— 返回 `(判定, 说明)`；缺对照时判定是 `None`。

    ★ 为什么这条得在报告层做：单场那句"写错 QoS ⟹ 0 条回调"只有在**同一个订阅写对时
    确实收得到**的前提下才是证据（否则可能只是这台机器上本来就没有状态）。
    ★ `None`（本批没同时跑那两场）与 `False`（对照不成立）**刻意分开**：前者是"这条
    判据没做"，后者是"做了、没过"。`--only qos-reliable` 那种跑法属于前者 —— 报告里
    要把这句话说出来，不能让它静悄悄地什么也不打。
    """
    be, rel = cb_by_name.get("baseline"), cb_by_name.get("qos-reliable")
    if be is None or rel is None:
        missing = "baseline" if be is None else "qos-reliable"
        return None, (f"跨档 QoS 阳性对照：本批**没有 `{missing}`** ⟹ 这条对照**没做**"
                      f"（缺的是对照那一半，别把它的缺席读成通过）")
    same = be > 0 and rel == 0
    return same, (f"跨档 QoS 阳性对照：同一个 `vehicle_status` 订阅 —— baseline（BEST_EFFORT）"
                  f"收到 {be} 条、qos-reliable（RELIABLE）收到 {rel} 条 ⟹ "
                  f"{'有/无 的差别确实来自 QoS 声明' if same else '❌ 对照不成立：差距不是 QoS 造成的'}")


def cross_delta(d1, d2):
    """跨档 Δ 一致性 —— 返回 `(判定, 说明)`；缺一档时判定是 `None`。

    ★ 判的是"**Δ 与挪几秒无关**"（`age = 挪的秒数 − Δ` 里的 Δ 是同步滞后这一个量）。
    ⚠️ 这条**分辨力有限，别当它是机制的铁证**：只有两个 lag 的点，另一种模型
    `age = c·lag`（c≈0.92，即「没有 Δ 这回事」）在这两点上拟合得一样好
    （0.916 / 1.832 对实测 0.916 / 1.823）。真正钉机制的是源码（`ucdr/<话题>.h` 的
    `min(ts − time_offset, hrt)`）加三条单场判据（age 随 lag 线性、挪 2 s 越门槛、
    三档错单位永远换不了支）；这里只挡住"Δ 大幅漂移"这一种大错。
    阈值 0.20 s：实测跨场景 0.08 ~ 0.24 s，同一跑里两个 lag 的 |差| 见过 0.022 / 0.093
    ⟹ 留了 2 倍余量 —— 它是**同步滞后的两次取样**，本来就不该逐位相同。
    """
    if d1 is None or d2 is None:
        missing = "ts-lag-1s" if d1 is None else "ts-lag-2s"
        return None, (f"跨档 Δ 一致性：本批**没有 `{missing}`** ⟹ 这条**没做**"
                      f"（不能拿它当「Δ 是个常数」的证据）")
    same = abs(d1 - d2) < 0.20
    return same, (f"跨档 Δ 一致性：挪 1 s ⟹ Δ={fmt(d1, '.3f')} s；挪 2 s ⟹ Δ={fmt(d2, '.3f')} s"
                  f"（|差|={fmt(abs(d1 - d2), '.3f')} < 0.20）⟹ "
                  f"{'一致 —— Δ 与「挪几秒」无关（量级 0.1 s 上下），是 uXRCE 的同步滞后' if same else '❌ 不一致：模型有问题'}"
                  f"（★ 这条分辨力有限，见函数 docstring）")


def report(results):
    if not results:
        # ★ 别把"一场都没跑成"报成"全部场景通过"：世界起不来时每一场都会被 `continue`
        #   掉，`bad` 恒为 0 —— 那是这份报告最丢人的一种错法（什么都没验却报绿）。
        print("⚠️ 一场都没跑完（每一场都在「起世界」那一步就失败了）⟹ 没有任何判据可判，"
              "**这不是「全部场景通过」**")
        return 1
    bad = 0
    for r in results:
        print(f"\n=== 场景 {r['name']} ===")
        print(f"  指令：{' '.join(r['argv'])}")
        d = r.get("dut") or {}
        if d.get("actual_hz"):
            print(f"  ·  DUT 发了 {d['frames']} 帧 / {fmt(d.get('span_s'), '.2f')}s ⟹ "
                  f"{fmt(d['actual_hz'], '.1f')} Hz；status 回调 {d.get('status_cb')} 条；"
                  f"DUT 侧见过的 nav_state {d['nav_seen']}")
        if r.get("offboard_enter") is not None:
            print(f"  ·  OFFBOARD 于 t={fmt(r['offboard_enter_rel'], '.2f')}s（窗口口径；"
                  f"= 记录器口径 {fmt(r['offboard_enter'], '.2f')}s − 窗口起点）生效；"
                  f"机头最低 {fmt(r.get('nose_min'))} 最高 {fmt(r.get('nose_max'))}"
                  f"（EKF 的 z 区间 [{fmt(r.get('z_min'))}, {fmt(r.get('z_max'))}] m —— "
                  f"★ 只是估计器读数，判高度不用它）")
        for s in r.get("truth_track") or []:
            print(f"  ·  真值 t={s['t']}s（= OFFBOARD 后 "
                  f"{fmt((s['t'] - (r.get('offboard_enter_rel') or 0)), '+.2f')}s，"
                  f"请求 {s['t_req']}s）：z={fmt(s['z'], '+.4f')} m、"
                  f"vz={fmt(s['vz'], '+.4f')} m/s、电机 {s['motors']}（均值 "
                  f"{fmt(s['motors_mean'], '.4f')}）")
        if r.get("gt_z_end") is not None:
            print(f"  ·  末段真值：z={fmt(r['gt_z_end'], '+.4f')} m、vz="
                  f"{fmt(r['gt_vz_end'], '+.4f')} m/s；PX4 解出的电机 {r.get('px4_motors_end')}")
        if r.get("offboard_exit_after_kill") is not None:
            print(f"  ·  断流后 {fmt(r['offboard_exit_after_kill'])}s 退出 OFFBOARD，"
                  f"去向 nav_state={nav_name(r.get('offboard_exit_state'))}")
        if r.get("px4_rates") and r["px4_rates"] != "—":
            print(f"  ·  PX4 侧的 vehicle_rates_setpoint：{r['px4_rates']}")
        for chk in r["checks"]:
            if not chk.get("counts", True):
                print(f"  ℹ️ {chk['name']}：{chk['detail']}")
                continue
            mark = "✅" if chk["ok"] else "❌"
            print(f"  {mark} {chk['name']}：{chk['detail']}")
            bad += 0 if chk["ok"] else 1
    # ★ 跨档判据两条：单场景里看不到对照，所以在报告层对一次。
    #   两条都**能失败**（缺对照时返回 None ⟹ 明说"没做"，不算通过也不算不通过）。
    cb = {r["name"]: (r.get("dut") or {}).get("status_cb") for r in results}
    ok, text = cross_qos_control(cb)
    print(f"  ·  {text}")
    bad += 1 if ok is False else 0

    lag_delta = {r["name"]: r.get("ts_age_delta_s") for r in results}
    ok, text = cross_delta(lag_delta.get("ts-lag-1s"), lag_delta.get("ts-lag-2s"))
    print(f"  ·  {text}")
    bad += 1 if ok is False else 0

    print(f"\n{'全部场景通过' if bad == 0 else f'{bad} 项判据不通过'}")
    return 0 if bad == 0 else 1


def switches_of(obs):
    """从 `argv` 里剥掉每场都有的那一截（日志路径 / `--cmd-interval` / `--phases`），
    只留**这一场的开关** —— 表格里"凭什么它是这一档"要一眼看得见。"""
    argv, out, skip = list(obs.get("argv") or []), [], False
    for a in argv:
        if skip:
            skip = False
            continue
        if a in ("--log-file", "--cmd-interval", "--phases"):
            skip = True
            continue
        out.append(a)
    return " ".join(out) or "（默认开关）"


def emit(results, args, write_json=False):
    """出口：判据报告（**总是打**，退出码从它来）→ 可选表格 → 可选落盘。

    ★ 为什么真跑与 `--replay` 要共用这一个出口：`--replay --table` 曾经直接
    `return table(...)`，于是报告层的两条跨档判据一次都不跑、退出码恒 0，
    而交付文档里生成那张表的命令正是它 —— 表里的数看着对，判据却没判。
    """
    rc = report(results)
    if args.table:
        print()
        table(results)
    if write_json and args.json:
        Path(args.json).write_text(
            json.dumps(results, indent=2, ensure_ascii=False, default=float),
            encoding="utf-8")
        print(f"[verify] 观测已写到 {args.json}", file=sys.stderr)
    return rc


def table(results):
    """把观测压成一张 markdown 表 —— **交付文档里的那张表就是这段的输出**，
    别手抄（手抄必然与 JSON 漂移）。配合 `--json` 用：`--table --replay obs.json`。

    ★ 它**不产生退出码**（那件事归 `report()`）：`--table` 是"再多打一张表"，
    判据报告照常打 —— 想只要表就从输出里挑 `|` 开头的行。
    ⚠️ 单元格里的 `|` 要转义：`vehicle_rates_setpoint` 那一格本来就是 `roll=… pitch=…`，
    没这个的话列会错位（第一版就这么错位了）。
    """
    def cell(s):
        return str(s).replace("|", "\\|")

    print("| 场景 | 这一场的开关 | 通过 | DUT 自报：首帧→OFFBOARD | 窗口口径进入 | "
          "末段真值 z / vz（m, m/s） | PX4 解出的电机（末段） | 本场要点 |")
    print("|---|---|---|---|---|---|---|---|")
    for r in results:
        d = r.get("dut") or {}
        ok = sum(1 for c in counted(r["checks"]) if c["ok"])
        pre = d.get("pre_stream_s")
        pre_hz = d.get("pre_stream_hz")
        pre_txt = ("**没进**" if pre is None else
                   f"{fmt(pre, '.2f')} s（{fmt(pre_hz, '.1f')} Hz）" if pre_hz is not None
                   else f"{fmt(pre, '.2f')} s")
        enter = (fmt(r.get("offboard_enter_rel"), ".2f") + " s"
                 if r.get("offboard_enter_rel") is not None else "—")
        gt = (f"{fmt(r.get('gt_z_end'), '+.3f')} / {fmt(r.get('gt_vz_end'), '+.3f')}"
              if r.get("gt_z_end") is not None else "—")
        mot = r.get("px4_motors_end") or "—"
        if r.get("offboard_exit_after_kill") is not None:
            note = (f"断流后 {fmt(r['offboard_exit_after_kill'])} s 退出 ⟹ "
                    f"{nav_name(r.get('offboard_exit_state'))}")
        elif pre is None and r["name"].startswith("qos"):
            note = "订阅 QoS 声明错 ⟹ **零回调**、不解锁、不切模式"
        elif pre is None and r["name"].startswith("ts-lag"):
            note = "记下的心跳 age > `COM_OF_LOSS_T` ⟹ 进不了"
        elif r["name"].startswith("ts-lag"):
            note = ("挪 1 s ⟹ 记下的 age≈0.8 s **仍在** `COM_OF_LOSS_T` 之内 ⟹ 贴边进得去"
                    "（余量只剩 Δ）")
        elif r["name"] == "thrust-sign-broken":
            note = "符号反 ⟹ 电机**全零**、原地不动"
        elif r["name"] == "thrust-scale-broken":
            note = (f"归一化常数错 ⟹ 推力过大、一路爬到真值 "
                    f"{fmt(abs(r.get('gt_z_end') or 0), '.1f')} m")
        elif r["name"].startswith("rate-step"):
            note = (f"机头末段均值 {fmt(r.get('nose_mean_end'), '+.4f')} rad ⟹ "
                    f"真值 vx {fmt(r.get('gt_vx_end'), '+.3f')} m/s")
        elif r["name"] == "baseline":
            note = (f"{d.get('frames')} 帧 / {fmt(d.get('span_s'), '.1f')} s ⟹ "
                    f"{fmt(d.get('actual_hz'), '.1f')} Hz")
        else:
            note = "单位错但**静默无害**：照进、照飞"
        print(f"| `{r['name']}` | `{cell(switches_of(r))}` | {ok}/{len(counted(r['checks']))} | "
              f"{pre_txt} | {enter} | {gt} | `{cell(mot)}` | {note} |")


# ---------------------------------------------------------------- 主流程


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0],
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--list", action="store_true")
    ap.add_argument("--only", default="", help="逗号分隔的场景名（默认全部）")
    ap.add_argument("--replay", default=None,
                    help="★ 不跑 SITL，直接拿上次 --json 的**观测**重算判据")
    ap.add_argument("--outdir", default="/tmp/offboard-smoke")
    ap.add_argument("--json", default=None)
    ap.add_argument("--table", action="store_true",
                    help="判据报告之外**再**打一张 markdown 汇总表（配合 --json/--replay 用；"
                         "交付文档里的那张表就是它的输出，别手抄。★ 报告与退出码照旧 —— "
                         "想只要表就从输出里挑 `|` 开头的行）")
    ap.add_argument("--src", default=str(Path.home() / "refs/px4-v1.14.3-build"))
    ap.add_argument("--build-dir", default=None)
    ap.add_argument("--ready-timeout", type=float, default=90.0)
    ap.add_argument("--boot-settle", type=float, default=4.0)
    ap.add_argument("--keep-params", action="store_true")
    ap.add_argument("--agent-bin", default="/usr/local/bin/MicroXRCEAgent")
    ap.add_argument("--agent-port", default="8888")
    ap.add_argument("--debug-dump", default="/tmp/verify_offboard_smoke_console.log")
    args = ap.parse_args(argv)

    if args.list:
        for name, spec in SCENARIOS.items():
            print(f"{name:22} {' '.join(spec['args']) or '（默认开关）':42} {spec['program']}")
        return 0

    if args.replay:
        results = json.loads(Path(args.replay).read_text(encoding="utf-8"))
        print(f"[verify] 回放 {args.replay}：{len(results)} 场"
              f"（{'、'.join(r['name'] for r in results) or '空'}）"
              f" —— 判据只覆盖列出的这些场", file=sys.stderr)
        for r in results:
            r["checks"] = evaluate(r)
        # ★ 两条路（真跑 / 回放）必须**共用同一个出口**：`--replay --table` 曾经在
        #   `table()` 就 return 了，于是报告层的跨档判据一次不跑、退出码恒 0 ——
        #   而交付文档里生成那张表的命令正是它（"表里的数看着对"和"判据真的过了"是两回事）。
        return emit(results, args)

    args.build_dir = args.build_dir or str(Path(args.src) / "build/px4_sitl_default")
    names = [n for n in (args.only.split(",") if args.only else SCENARIOS) if n]
    unknown = [n for n in names if n not in SCENARIOS]
    if unknown:
        raise SystemExit(f"没有这些场景：{unknown}（--list 看全部）")

    if not (Path(args.build_dir) / "etc/init.d-posix/airframes/10046_sihsim_z0").exists():
        raise SystemExit("z0 机型没装 —— 先跑 install_z0_airframe.sh")

    Path(args.outdir).mkdir(parents=True, exist_ok=True)
    if not args.keep_params:
        # 从零起手，否则 rootfs/parameters.bson 的历史残留会让结果不可归因
        for f in ("parameters.bson", "parameters_backup.bson"):
            (Path(args.build_dir) / "rootfs" / f).unlink(missing_ok=True)

    rclpy.init()
    rec = Recorder()
    results = []
    failed = []
    try:
        for name in names:
            # ★ 先清掉上一轮的名字：`World.__enter__` 自己抛错时 `with … as world` 不赋值，
            #   于是 `world` 要么**未绑定**（第一场 ⟹ `NameError` 把真正的错误顶掉），
            #   要么还是**上一场**那个（`__exit__` 已经关掉的控制台）—— 两种都会把
            #   `--debug-dump` 写成"上一套世界"或干脆什么都不写。
            world = None
            try:
                with World(args) as world:
                    print(f"[verify] COM_OF_LOSS_T = {world.com_of_loss_t}", file=sys.stderr)
                    obs, _dut = run_scenario(name, SCENARIOS[name], world.px, rec, args)
                    obs["com_of_loss_t"] = world.com_of_loss_t
                    obs["time_offset_us"] = world.time_offset_us
            except (Px4ConsoleError, OSError) as e:
                # ★ `OSError` 也要接：`--agent-bin` / `--build-dir` 指到不存在的路径时，
                #   `World.__enter__` 抛的是 `FileNotFoundError`（真见过：agent 不在这台
                #   机器上），它不属于 `Px4ConsoleError` —— 于是整个跑程被一个 traceback
                #   打断，`--json` 也不写了，而上面那句"世界没起来"的话一次都没打出来。
                px = world.px if world is not None else None
                print(f"[verify] {name}: 控制台错误 —— {e!r}", file=sys.stderr)
                if px is None:
                    print(f"[verify] ⚠ {name}: 世界没起来（`World.__enter__` 就失败了），"
                          f"没有控制台可 dump —— 上面那行错误就是全部线索", file=sys.stderr)
                else:
                    Path(args.debug_dump).write_text(px.text() or "", encoding="utf-8")
                failed.append(name)
                if isinstance(e, OSError):
                    # 起世界这一步就 OSError（agent 二进制不在 / 构建目录不对）⟹ 后面每一场
                    # 都会一模一样地失败，继续跑只是把同一条错误刷 N 遍。
                    print(f"[verify] ⚠ 起世界这一步就是 OSError（多半是 `--agent-bin` / "
                          f"`--build-dir` 路径不对）⟹ 后面的场景注定同样失败，就此打住",
                          file=sys.stderr)
                    break
                continue
            obs["checks"] = evaluate(obs)
            results.append(obs)
            print(f"[verify] {name} ⟹ "
                  f"{sum(1 for c in counted(obs['checks']) if c['ok'])}/"
                  f"{len(counted(obs['checks']))} 通过 （另有 "
                  f"{len(obs['checks']) - len(counted(obs['checks']))} 条说明，不计）",
                  file=sys.stderr)
    finally:
        rec.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    rc = emit(results, args, write_json=True)
    if failed:
        # ★ "某一场没跑成"不等于"它通过了"：报告里没有它，`全部场景通过` 那行也照样会打。
        #   把缺席的场次点名，并且让退出码非 0 —— 一次 12 场的验收里少跑一场，
        #   不是"少验一条"，是**这次验收不完整**。
        print(f"\n⚠️ 有 {len(failed)} 场**根本没跑成**：{'、'.join(failed)} ⟹ "
              f"这是一次**不完整的验收**（它们不在上面的报告里，不等于它们通过）")
        rc = 1
    return rc


if __name__ == "__main__":
    sys.exit(main())
