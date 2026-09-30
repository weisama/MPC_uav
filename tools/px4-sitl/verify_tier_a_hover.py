#!/usr/bin/env python3
"""A 档闭环（ticket 09）—— 在 **z0 同构 SITL** 上让 `mpc_node` 真的飞，并把**数字**带回来。

本工具回答三个互相独立的问题（每个问题有各自的仪器）：

| 问题 | 仪器 | 为什么是这台仪器 |
|---|---|---|
| 飞机**真的**稳在 1 m 吗 | SIH 真值 `vehicle_local_position_groundtruth`，**稀疏采样 ~4 Hz** | 这套 SITL 里唯一不是估计器算出来的高度（`sih.cpp:497-552` 直接发 `_p_I`）。见下面 ★★★ |
| 我们发出去的是什么、**求解多慢** | DUT 自己的 JSONL（`-p log_file:=…`，100 Hz） | 逐帧 `u` / `solve_ms` / `sat` / `fallback` / `x0`；D6 那个数字只从这一列来。**振动判据也在这条 100 Hz 的日志上**（真值采样只有 4 Hz） |
| PX4 收下之后做了什么 | pxh：`actuator_outputs_sim`、`estimator_status_flags`、`param show` | 电机是 PX4 **解出来**的量（不是我们的请求）；`estimator_status_flags` 是"EKF 到底在用哪几个源"的权威读数 |

★★★ **为什么真值不能走连续流**（2026-09-28 两次实跑换来；第一版就是连续流，两场作废）：
  `listener <话题> -n N`（N ≥ 2）走的是**流式分支**（`listener_main.cpp:96-146`），
  逐样本打一整屏（~1.1 KB）。而 `-r` 限不住它：实测请求 50 Hz、39.5 s 墙钟里收到 12050 块
  （3339 Hz）⟹ 那条流是 **3.7 MB/s**。PX4 的 print 同步写 stdout ⟹ 主循环（含 SIH 步进）
  被自己的输出堵住：那 39.5 s 里**被控对象的时间只走了 3.6 s（9.1%）**，
  一场"稳在 1 m"的飞行变成在时间流速差 10 倍的被控对象上做闭环（飞机一路爬到 30 m）。
  ⟹ 本工具改成**稀疏采样**（`listener -n 2`，周期 0.25 s，~9 KB/s）**并把这件事做成硬自检**：
  样本自带的被控对象时间跨度与墙钟跨度之比必须 ≈1（判据⓪），不是靠"应该没问题"。
  ⚠️ 采样期间控制台**不能**发别的命令（`listener` 把 stdin 当退出键读，其余字符读掉就扔，
     `listener_main.cpp:128-146`）⟹ 解锁走 ROS，见 `ArmAgent`。

⚠️ 本工具**不会**替你把状态源换成真值（那件事要改 `dds_topics.yaml` 并重编 PX4，见
  `--source px4-gt` 的说明与 `docs/control/tier-a-sitl-hover.md`）。默认就是票面写的那一对：
  `/fmu/out/vehicle_local_position` + `/fmu/out/vehicle_attitude`。

用法（**必须先 source 工作空间**，与其它 tools 一致）：

    source /opt/ros/humble/setup.bash
    source ~/px4_msgs_ws/install/setup.bash
    source install/setup.bash

    python3 tools/px4-sitl/verify_tier_a_hover.py --plant-check          # 只体检被控对象，不跑节点
    python3 tools/px4-sitl/verify_tier_a_hover.py --plant-check --start-sims
    python3 tools/px4-sitl/verify_tier_a_hover.py --hold-s 30 --json /tmp/tier-a/obs.json
    python3 tools/px4-sitl/verify_tier_a_hover.py --replay /tmp/tier-a/obs.json   # 不跑 SITL，重放判据
"""

from __future__ import annotations

import argparse
import json
import math
import os
import re
import statistics
import subprocess
import sys
import time
import traceback
from pathlib import Path

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[1]
sys.path.insert(0, str(HERE))
# ★ 判据里的常数从**源码**读，不读 `install/` 的副本 —— 本仓库的老坑：
#   `install/` 是拷贝，改了源码没 `colcon build` 时它还是旧值，而报告上看不出来。
sys.path.insert(0, str(REPO / "src/nmpc"))

import rclpy  # noqa: E402
from px4_msgs.msg import VehicleCommand, VehicleStatus  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,  # noqa: E402
                       ReliabilityPolicy)

import verify_offboard_smoke as v  # noqa: E402  （同目录，复用世界/记录器/控制台采样）
from nmpc.params import (HOVER_THRUST, YAW_RATE_MAX, ROLL_RATE_MAX,  # noqa: E402
                         PITCH_RATE_MAX)

#: 发布侧 QoS：`dds_topics.h.em` 给 PX4 的 reader 建的是 best_effort 流 ⟹ 我们也发 best_effort
#: （+ TRANSIENT_LOCAL 让后起的订阅者也能拿到最后一帧）。与 `offboard_smoke.py` 同一份理由。
QOS_PUB = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                     reliability=ReliabilityPolicy.BEST_EFFORT,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)

MAV_MODE_FLAG_CUSTOM_MODE_ENABLED = 1.0
PX4_CUSTOM_MAIN_MODE_OFFBOARD = 6.0

#: 判据里的常数一律**从 z0 机型文件读**（`verify_offboard_smoke` 的老规矩）：
#: 悬停推力那一式 = `m·g`，归一化后 = `m·g / (4·SIH_T_MAX)`（`sih.cpp:290-303`）。
MASS_KG = v.airframe_param("SIH_MASS")
SIH_T_MAX = v.airframe_param("SIH_T_MAX")            # 单个电机的最大推力（N）
G0 = v.G0
HOVER_N = None if MASS_KG is None else MASS_KG * G0
T_MAX_TOTAL = None if not SIH_T_MAX else 4.0 * SIH_T_MAX      # 四轴的总推力上限（N）
HOVER_NORM = None if (HOVER_N is None or T_MAX_TOTAL is None) else HOVER_N / T_MAX_TOTAL

#: 真值流的话题名（`msg/VehicleLocalPosition.msg:75` 的 `# TOPICS` 行——
#: `vehicle_local_position_groundtruth` 与 `vehicle_local_position` **共用同一个消息定义**，
#: 所以 `px4_msgs` 里不需要新的 `.msg`；能不能在 DDS 上看到它只取决于 `dds_topics.yaml`）。
GT_TOPIC = "vehicle_local_position_groundtruth"
GT_ATT_TOPIC = "vehicle_attitude_groundtruth"

#: 体检时问 PX4 的参数（EKF 到底有没有高度/位置源，是 A 档能不能成立的前提）
PLANT_PARAMS = ("SYS_HITL", "EKF2_HGT_MODE", "EKF2_GPS_CTRL", "EKF2_EV_CTRL", "GPS_1_CONFIG",
                "SENS_EN_GPSSIM", "SENS_EN_BAROSIM", "SENS_EN_MAGSIM", "SENS_IMU_MODE",
                "COM_ARM_WO_GPS", "EKF2_BARO_CTRL")

#: `EstimatorStatusFlags` 里"哪个源真的在融合"的那几位（`msg/EstimatorStatusFlags.msg`）
EST_FLAGS = ("cs_tilt_align", "cs_yaw_align", "cs_gps", "cs_gps_hgt", "cs_baro_hgt",
             "cs_rng_hgt", "cs_ev_pos", "cs_ev_hgt", "cs_mag_3d", "cs_vehicle_at_rest",
             "cs_in_air", "cs_wind")


# ------------------------------------------------------------------ 真值流


#: 流式 `listener` 的逐样本表头（`listener_main.cpp:140`）：
#: `TOPIC: <话题> instance <i> #<第几个样本>`
_SAMPLE_HEAD = re.compile(r"^TOPIC:\s*(\S+)\s+instance\s+\d+\s+#(\d+)\s*$", re.M)


def parse_stream(chunk, topic, fields=("timestamp", "x", "y", "z", "vx", "vy", "vz"),
                 arrays=()):
    """把 `listener <topic> -n N` 的整段输出切成**逐帧**样本。

    ★ 用输出里自带的 `#<n>` 表头切块，而不是按行数/空行猜：pxh 的块里字段数会随
      消息定义变（多一个字段就错位），而表头是 `listener` 自己打的。
    ★ 只认 `topic` 匹配的表头：同一次跑程里控制台上可能来过别的 `listener`（体检那几发），
      它们的残块会混在同一段字节里（字节游标是按"我发命令那一刻"记的）。
    ★ `arrays` 是**数组字段**（`q: [w, x, y, z]`）：`v.field()` 只认标量，
      对 `q:` 后面那个 `[` 返回 `None`（实测 —— 姿态真值这一路就靠它，见 `GT_ATT_TOPIC`）。
    """
    heads = [m for m in _SAMPLE_HEAD.finditer(chunk) if m.group(1) == topic]
    out = []
    for i, m in enumerate(heads):
        end = heads[i + 1].start() if i + 1 < len(heads) else len(chunk)
        body = chunk[m.end():end]
        row = {k: v.field(body, k) for k in fields}
        for k in arrays:
            mm = re.search(rf"^\s*{k}:\s*\[([^\]]*)\]", body, re.M)
            row[k] = ([float(t) for t in mm.group(1).split(",")] if mm else None)
        row["n"] = int(m.group(2))
        if row.get("timestamp") is None:
            continue
        out.append(row)
    # ★ 按 `#<n>` 排序后再去重：pxh 的行内重画（`ESC[2K\\r`）偶尔会把同一块的头部
    #   重复渲染一次（`text_since()` 的字节游标挡掉了大部分，但**流式输出的边界**上
    #   仍可能重来一次）—— 去重键用表头里的样本号，比"时间戳相等"可靠。
    seen, uniq = set(), []
    for r in sorted(out, key=lambda r: r["n"]):
        if r["n"] in seen:
            continue
        seen.add(r["n"])
        uniq.append(r)
    return uniq


class TruthSampler:
    """真值的**稀疏**采样器：一条命令换一对样本（`listener <话题> -n 2`），不做连续流。

    ★★★ 为什么不是"连续流"（2026-09-28 两次实跑换来的结论，代价是两场作废的飞行）：

      1. `listener <topic> -n N` 的**流式打印不受 `-r` 约束**。`-r 50` 在源码里是
         `orb_set_interval(sub, 1000/50 = 20)`（`listener_main.cpp:212-215`），
         单位 ms 是对的（`uORBManagerUsr.cpp:193` 里 `interval * 1000` → `set_interval_us`），
         但它只约束 `orb_check` 那一类消费者，**管不住 `poll()` 唤醒的打印循环**
         （流式模式走的是 `fds[1]` 就绪即打印，`listener_main.cpp:118-146`）。
         实测：请求 50 Hz 时，39.5 s 墙钟里收到 **12050 块**（= 我们给的 `-n`）⟹ 3339 Hz。
      2. 真正的伤害不是"数据多"，而是**被控对象被自己的输出堵死**：PX4 的 print 是
         同步写 stdout，那条流是 3.7 MB/s ⟹ 主循环（含 SIH 步进）被压在写调用上。
         实测那 39.5 s 里**被控对象的时间只走了 3.6 s（9.1%）**——于是"稳在 1 m"
         变成在一台时间流速差 10 倍的被控对象上做闭环：飞机一路爬到 30 m、
         日志里 `u[0]` 恒在 8.3 N（悬停只要 7.36 N），整场读数全部作废。
         ⟹ 症状很好认：`truth_hz` 会是"请求值的几十倍"，所以**判据⓪**把它做成硬自检。

    ★ 采样周期取 0.25 s（~4 Hz）：控制台流量 ~9 KB/s（比流式的 3.7 MB/s 低 400 倍），
      而高度/漂移这类判据要的是"轨迹"，4 Hz 足够（振动判据不用它 —— 那一层判在
      DUT 自己 100 Hz 的逐帧日志上，见 `frame_track`）。
    """

    def __init__(self, px, topic, rec, period_s=0.25, timeout_s=2.0,
                 fields=("x", "y", "z", "vx", "vy", "vz"), arrays=()) -> None:
        self.px, self.topic, self.rec = px, topic, rec
        self.period_s, self.timeout_s = period_s, timeout_s
        #: ★ 要取哪些列 —— 位置那一路是 `x/y/z/vx/vy/vz`，姿态真值那一路是 `q`（数组）。
        #:  参数化是为了**同一台稀疏采样器**能同时干这两件事（两条链的坑完全一样：
        #:  都得躲开 `listener` 的流式输出，见模块 docstring ★★★）。
        self.fields, self.arrays = tuple(fields), tuple(arrays)
        self.samples = []          # 每行：{"veh_us", "wall", <fields…>, <arrays…>}
        self.sent = self.missed = 0
        self._mark = None
        self._sent_at = 0.0
        self._next = 0.0

    def poll(self) -> None:
        """推进一拍：到点就发采样命令；有回音就收样本。**非阻塞**（要能在 spin 循环里调）。"""
        now = time.time()
        if self._mark is None:
            if now < self._next:
                return
            self._mark = self.px.mark()
            # ⚠️ settle 给 0：紧接着就 `pump`，不在这里白等
            self.px.send(f"listener {self.topic} -n 2", settle=0.0)
            self._sent_at, self.sent = now, self.sent + 1
            return
        chunk = self.px.text_since(self._mark)
        rows = parse_stream(chunk, self.topic, fields=("timestamp", *self.fields),
                            arrays=self.arrays)
        if rows:
            s = rows[-1]                       # `-n 2` ⟹ 取第二帧（时间戳更新）
            self.samples.append({"veh_us": s["timestamp"], "wall": self.rec.now(),
                                 # ★ `wall` 是**相对**的（`Recorder.now()` = `time.time() − t0`），
                                 #   而帧日志的 `wall_ns` 是**绝对**的 ROS 纪元 ns ⟹ 配对要用
                                 #   这个绝对量（判据⑨；理由见 `pair_by_time` 的 docstring）。
                                 "wall_epoch": time.time(),
                                 **{k: s.get(k) for k in (*self.fields, *self.arrays)}})
            self._mark = None
            self._next = time.time() + self.period_s
        elif now - self._sent_at > self.timeout_s:
            self._mark = None                  # 没回音就放弃这一拍（`missed` 记账，别卡住）
            self.missed += 1
            self._next = time.time() + self.period_s

    def track(self) -> tuple:
        """压成 `([[t_rel_s, x, y, z, vx, vy, vz], …], meta)`。

        ★ `t` 一律用**样本自带的被控对象时间**（µs，相对第一帧）——本工具所有高度/漂移判据
          都用这一把尺子；"OFFBOARD 生效"那一刻也从**真值自己**认（第一次爬升），
          不跨时钟换算（两个时钟之间隔着 ~0.18 s 的 `time_offset`，见 ticket 08）。
        ★ `meta` 里同时给出**被控对象时间跨度**与**墙钟跨度**：这两个数的比就是判据⓪
          （仪器有没有把被控对象卡住）。
        ★★ **残缺样本在这里就丢掉**（`partial` 记账，不静默）：`listener` 的一屏可能被
          `text_since` 从中间截断（实测出现 `[t, x, y, None, None, None, None]` 这种行，
          150 个样本里 28 个），而下游 `segments()` 拿 `None` 与阈值比会直接 `TypeError`
          —— 第一版就是这么在**一场飞完之后**把报告打崩的（obs.json 因为写在 `finally` 里
          才没一起丢）。丢在源头 + 留计数，比在每个消费者里各写一遍判空更不容易漏。
        """
        if not self.samples:
            return [], {"n": 0, "sent": self.sent, "missed": self.missed, "partial": 0}
        t0 = self.samples[0]["veh_us"]
        fields = ("x", "y", "z", "vx", "vy", "vz")
        rows, partial = [], 0
        for s in self.samples:
            if any(s.get(k) is None for k in fields):
                partial += 1
                continue
            rows.append([(s["veh_us"] - t0) / 1e6] + [s[k] for k in fields])
        if not rows:
            return [], {"n": 0, "sent": self.sent, "missed": self.missed, "partial": partial}
        veh_span = rows[-1][0]
        wall_span = self.samples[-1]["wall"] - self.samples[0]["wall"]
        return rows, {"n": len(rows), "sent": self.sent, "missed": self.missed,
                      "partial": partial,
                      "vehicle_span_s": round(veh_span, 2),
                      "wall_span_s": round(wall_span, 2),
                      "hz": round((len(rows) - 1) / veh_span, 2) if veh_span > 0 else None}

    def payload_rows(self, keys) -> tuple:
        """轻量整理：只做"丢掉残缺样本 + 换成相对时间"，返回 `[{t, <key>…}, …]`。

        ★ 与 `track()` 的区别只是**输出形状**（这里是字典、那边是给数值判据用的定长表）。
        两者共用同一条纪律：**残缺样本丢在源头并计数**（理由见 `track()` 的 ★★）。

        ★★ `wall_epoch`（绝对墙钟）**必定随行输出**，它不在 `keys` 里也不由调用方决定：
          它是 `pair_by_time` 唯一能用的配对键。第一版漏了它 ⟹ 33 个姿态真值样本
          全部在配对时被静默跳过、判据⑨ 报成"没采到姿态真值"——**仪器坏了却说得像没数据**
          （2026-09-28，`test_tier_a_judges.py::test_payload_rows_带出配对键` 盯住这一条）。
        """
        t0 = self.samples[0]["veh_us"] if self.samples else 0
        rows, partial = [], 0
        for s in self.samples:
            if any(s.get(k) is None for k in keys):
                partial += 1
                continue
            if not isinstance(s.get("wall_epoch"), (int, float)):
                partial += 1                      # 没有配对键的行等于半条记录，别当完整的发出去
                continue
            rows.append({"t": round((s["veh_us"] - t0) / 1e6, 3), "veh_us": s["veh_us"],
                         "wall_epoch": float(s["wall_epoch"]),
                         **{k: s[k] for k in keys}})
        return rows, {"n": len(rows), "sent": self.sent, "missed": self.missed,
                      "partial": partial}


# ------------------------------------------------------------------ 解锁 / 切模式


class ArmAgent(Node):
    """把 PX4 拉进 OFFBOARD：先 `ARM_DISARM`，armed 之后 `DO_SET_MODE`（custom + OFFBOARD）。

    ★ 为什么用节点而不是控制台 `commander arm`：控制台这一场是**真值记录仪**
      （`listener … -n N` 会占住它几十秒、还会把我们的字符吃掉）⟹ 解锁走 ROS、
      控制台只读数，两件事不抢同一个 pty。机制与 `offboard_smoke.py` 的
      `_send_command` 逐字一致（同一组 param、同一个 QoS）。
    """

    def __init__(self, interval_s: float = 0.5) -> None:
        super().__init__("tier_a_arm")
        self.pub_cmd = self.create_publisher(VehicleCommand, "/fmu/in/vehicle_command", QOS_PUB)
        self.create_subscription(VehicleStatus, "/fmu/out/vehicle_status", self._on_status, v.QOS_BE)
        self.status = None
        self.arm_sent = 0
        self.mode_sent = 0
        self.create_timer(interval_s, self._tick)

    def _on_status(self, m) -> None:
        self.status = m

    @property
    def armed(self) -> bool:
        return (self.status is not None
                and int(self.status.arming_state) == v.ARMING_STATE_ARMED)

    @property
    def nav(self) -> int:
        return -1 if self.status is None else int(self.status.nav_state)

    def _send(self, command: int, param1: float, param2: float = 0.0) -> None:
        msg = VehicleCommand()
        msg.timestamp = int(time.time_ns() // 1000)   # ★ epoch µs（ticket 08 的结论）
        msg.command = command
        msg.param1 = float(param1)
        msg.param2 = float(param2)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self.pub_cmd.publish(msg)

    def _tick(self) -> None:
        if self.status is None:
            return
        if not self.armed:
            self._send(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
            self.arm_sent += 1
            return
        if self.nav != v.NAV_STATE_OFFBOARD:
            self._send(VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
                       MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, PX4_CUSTOM_MAIN_MODE_OFFBOARD)
            self.mode_sent += 1

    def describe(self) -> str:
        return (f"解锁命令 {self.arm_sent} 条、模式命令 {self.mode_sent} 条；"
                f"末态 armed={int(self.armed)} nav_state={self.nav}")


def dut_env() -> dict:
    """DUT 子进程的额外环境变量：acados 的**安装目录**（与 `test_mpc_node._dut_env` 同一条规则）。

    ★ 没有它会静悄悄失败成"节点一帧都不发"：`OfflineOcp` 里 `ctypes` 去开
      `$ACADOS_SOURCE_DIR/lib/../lib/libacados.so`，而本工具起子进程时**没带**操作者 shell 里的
      那套环境（实测 `OSError: …/acados/lib/../lib/libacados.so: cannot open shared object file`
      ⟹ 首场 A 档跑成了"中间件归因"，白花 80 s）。
    ★ 只在外面真的没设时才补 `~/acados`（`docs/control/acados-orin-build.md` 坑 4：
      拿 `ACADOS_SOURCE_DIR` 指源码树是另一个坑）——不覆盖操作者给的值。
    """
    if os.environ.get("ACADOS_SOURCE_DIR"):
        return {}
    root = Path.home() / "acados"
    if not (root / "lib").is_dir():
        return {}
    return {"ACADOS_SOURCE_DIR": str(root),
            "LD_LIBRARY_PATH": f"{root / 'lib'}:{os.environ.get('LD_LIBRARY_PATH', '')}"}


class MpcDut(v.Dut):
    """被测节点换成 `mpc_node.py`（A 档）。

    ★ 只覆盖 `start()`：`v.Dut.start()` 把入口脚本**写死**成 `offboard_smoke.py`（ticket 05 的
      冒烟节点），而 A 档要测的是另一个可执行文件、参数走 `--ros-args -p`。
      `__exit__` / `sigint` / `frames` 全部照用 —— 那些是"进程组 + JSONL"的通用部分。
    """

    def start(self) -> "MpcDut":
        # ★ 先删旧日志，理由与 `v.Dut.start()` 里那条一样：`frames()` 读到上一场的文件，
        #   会让"节点起来了没"判据当场为真（实测代价 15 s 级）。
        self.log_file.unlink(missing_ok=True)
        cmd = ["ros2", "run", "nmpc", "mpc_node.py"] + self.argv
        env = dict(os.environ)
        env.update(dut_env())
        self.proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                     text=True, start_new_session=True, env=env)
        return self


# ------------------------------------------------------------------ 推进 / 等待


# ---- 转节点：**一个进程共用一个 SingleThreadedExecutor**（2026-09-29 改，ticket 11）
#
# ☠️ 不要再退回 `for n in nodes: rclpy.spin_once(n, timeout_sec=0.0)`。
#    那个写法**每次调用都 add_node/remove_node**，实测会让**某一个**节点的回调
#    被永久饿死 —— 饿死谁是不确定的（与建节点的时间、话题、QoS、有没有回放、DUT 在不在
#    都无关）。B 档上撞到的两种表现：
#      · `--no-replay`：`Recorder` 计数从 t=5 s 起冻住（35 s 内一动不动，同时别的节点照涨）
#      · `--with-replay`：换成裸 `vehicle_status` 订阅者饿死，`ArmAgent.status` 收不到
#        ⟹ `_tick()` 提前返回 ⟹ **解锁命令 0 条**⟹ PX4 不进 OFFBOARD ⟹ 整场废掉
#    探针（`/tmp/probe_b_arm.py`，四订阅者分两批建 + 单/多执行器开关）逐条钉住了上面这段；
#    换成共享执行器后**每个节点都收得到**，A 档同一条链路也一并受益。
#
# ☠️ `SPIN_DRAIN` 不是"多转几圈好看"：`SingleThreadedExecutor.spin_once` 一次只处理
#    **一个**回调，5 个节点共享时每拍只 drain 1 次会把每个话题的速率压到 11 Hz 上下
#    （真值流尤其吃亏：判据① 的统计窗就建在它上面）。30 次/拍实测把真值流拉到 231 Hz
#    （探针 30 s：tap 6928 点），比旧的 round-robin 还快 3 倍。
SPIN_DRAIN = 30

_EXEC = None
_EXEC_NODES: set = set()


def _executor_for(nodes):
    """进程内共用的执行器；新节点只 add 一次（`add_node` 幂等，集合只是省点开销）。"""
    global _EXEC
    if _EXEC is None:
        from rclpy.executors import SingleThreadedExecutor
        _EXEC = SingleThreadedExecutor()
    for n in nodes:
        if id(n) not in _EXEC_NODES:
            _EXEC.add_node(n)
            _EXEC_NODES.add(id(n))
    return _EXEC


def _spin(nodes) -> None:
    ex = _executor_for(nodes)
    for _ in range(SPIN_DRAIN):
        ex.spin_once(timeout_sec=0.0)


def spin_all(px, nodes, seconds: float, tick=None) -> None:
    """推进墙钟：喂控制台 + 转**所有**节点的回调（记录器与解锁器都要转）。

    ★ 只转记录器会饿着解锁器：`ArmAgent` 的定时器不转就不发命令，
      表现是"等 OFFBOARD 等满超时"，看起来像 PX4 不认我们的模式命令。
    ★ `tick` 是**顺手要推进的东西**（A 档就是真值采样器）：它必须在同一个循环里被调，
      不然要么采样频率跟着 spin 的节奏漂、要么得为它单开一个线程去抢同一个 pty。
    """
    end = time.time() + seconds
    while time.time() < end:
        px.pump(min(0.01, max(0.001, end - time.time())))
        _spin(nodes)
        if tick is not None:
            tick()


def wait_until(px, nodes, predicate, timeout_s, message: str = "", tick=None):
    """等到 `predicate()` 为真，返回 True/False（**不抛异常**：本工具的判据要能"没等到"）。"""
    end = time.time() + timeout_s
    while time.time() < end:
        px.pump(0.01)
        _spin(nodes)
        if tick is not None:
            tick()
        if predicate():
            return True
    if message:
        print(f"[verify] ⚠ 等超时：{message}", file=sys.stderr)
    return False


# ------------------------------------------------------------------ 体检


#: 控制台采样的等待上限（比 `verify_offboard_smoke` 的 15 s 短）：本工具要连采七八条话题，
#: 每条都白等 15 s 的话一场体检要多花两分钟，而"话题不存在"这件事
#: 恰恰由**返回空**来表达（`listener` 数到 0 个实例只打 `never published`）。
SAMPLE_TIMEOUT_S = 8.0

POSE_FIELDS = ("x", "y", "z", "vx", "vy", "vz", "heading")
VALID_FIELDS = ("xy_valid", "z_valid", "v_xy_valid", "v_z_valid", "heading_good_for_control")


def sample_pose(px, topic, rec=None, with_valid=False):
    """采一条 `VehicleLocalPosition` 系的位姿话题 ⟹ `(字段, 有效性标志)`。

    ★ `vehicle_local_position_groundtruth` 与 `vehicle_local_position` **共用同一个消息定义**
      （`msg/VehicleLocalPosition.msg:75` 的 `# TOPICS` 行），所以两者的字段名逐个对得上 ——
      "真值 vs EKF"这类对照才有共同语言。
    """
    chunk = v.sample_topic(px, topic, timeout=SAMPLE_TIMEOUT_S, rec=rec)
    fields = {k: v.field(chunk, k) for k in POSE_FIELDS}
    if not with_valid:
        return fields
    return fields, {k: v.bool_field(chunk, k) for k in VALID_FIELDS}


def sample_flags(px, topic, rec=None) -> dict:
    """采一条话题并抽 `bool` 字段（`EstimatorStatusFlags` 那几位）。"""
    chunk = v.sample_topic(px, topic, timeout=SAMPLE_TIMEOUT_S, rec=rec)
    return {name: v.bool_field(chunk, name) for name in EST_FLAGS}


def run_plant_check(world, rec, args) -> dict:
    """**不跑 DUT**，只问被控对象：传感器的源在哪、EKF 在融合什么、估计器靠不靠得住。

    ★ 这一节是 A 档的**前提检查**，不是可选项：票面要求"读 PX4 自己的
      `/fmu/out/vehicle_local_position` + `vehicle_attitude` 把飞机稳在 1 m"，
      而那两条话题是 **EKF2 的输出**。如果这台 SITL 里 EKF2 没有任何位置/高度源，
      那么"稳不住"这件事与我们的模型、符号映射、中间件都无关 —— 得先把归因这条路堵死，
      再去谈控制器的表现（否则第一场失败会引来一堆无用的调参）。
    """
    px = world.px
    obs = {"params": {}, "flags": {}, "ros_topics": [], "t": {}}
    for name in PLANT_PARAMS:
        obs["params"][name] = world._read_param(name)

    # 地面对照：真值与 EKF 各读一次（**同一条跑道、同一台仪器口径**）
    obs["gt_rest"] = sample_pose(px, GT_TOPIC, rec)
    obs["t"]["gt_rest"] = round(rec.now(), 2)
    obs["ekf_rest"], obs["ekf_rest_valid"] = sample_pose(px, "vehicle_local_position", rec,
                                                         with_valid=True)
    obs["t"]["ekf_rest"] = round(rec.now(), 2)
    obs["flags"] = sample_flags(px, "estimator_status_flags", rec=rec)
    obs["att_seen"] = v.field(v.sample_topic(px, "vehicle_attitude", rec=rec,
                                             timeout=SAMPLE_TIMEOUT_S), "timestamp") is not None

    # ★★ 地面静息**待一会儿**：一次快照分不出"误差是有界的噪声"还是"在爬"。
    #   这个区别对 A 档是决定性的 —— 有界 ⟹ 控制器追 EKF 是合理的；在爬 ⟹ 闭环追的是
    #   一个越走越远的量，真值高度判据必然不过（而这与我们的模型/符号无关）。
    if args.plant_dwell_s > 0:
        obs["dwell"] = dwell_estimator_vs_truth(px, rec, args.plant_dwell_s)

    if args.start_sims:
        # ★ 「手动把三个传感器仿真模块起起来」——★ **实测：它们本来就在跑**：
        #   三条命令的应答都是 `ERROR [sensor_*_sim] Task already running`。
        #   机制不在 `rcS` 的 HITL 分支（`rcS:346-353` 只管 `SYS_HITL == 2`），
        #   而在 POSIX 专用的 `ROMFS/px4fmu_common/init.d-posix/px4-rc.simulator:21-31`：
        #   `simulator_sih start` 之后按 `SENS_EN_*SIM` 逐个起；z0 机型把三个都设了 1。
        #   （早先"三件套不在跑"的结论是**错的**，见文档的归因一节。）
        replies = {}
        for mod in ("sensor_baro_sim", "sensor_mag_sim", "sensor_gps_sim"):
            m = px.mark()
            px.send(f"{mod} start", settle=0.25)
            replies[mod] = " ".join(px.text_since(m).strip().splitlines()[-2:])
        obs["sims_reply"] = replies
        spin_all(px, [rec], args.settle_after_sims)
        obs["gt_rest2"] = sample_pose(px, GT_TOPIC, rec)
        obs["ekf_rest2"], obs["ekf_rest2_valid"] = sample_pose(
            px, "vehicle_local_position", rec, with_valid=True)
        obs["flags2"] = sample_flags(px, "estimator_status_flags", rec=rec)

    # DDS 侧到底有没有那两条真值话题（`--source px4-gt` 能不能成立的前提）
    try:
        out = subprocess.run(["ros2", "topic", "list"], capture_output=True, text=True,
                             timeout=20.0)
        obs["ros_topics"] = sorted(t for t in out.stdout.split() if "groundtruth" in t)
    except (OSError, subprocess.TimeoutExpired) as e:      # pragma: no cover - 环境问题
        obs["ros_topics_error"] = repr(e)
    return obs


def evaluate_plant(obs) -> list:
    """体检的判据：**能不能指望 EKF 的高度**。"""
    c = []

    def ck(name, ok, detail, counts=True):
        c.append({"name": name, "ok": bool(ok) if ok is not None else None,
                  "detail": detail, "counts": counts})

    p = obs.get("params") or {}
    ck("被控对象是 SITL（而非 HITL 模式）", p.get("SYS_HITL") == 0.0,
       f"`SYS_HITL` = {v.fmt(p.get('SYS_HITL'))} —— ★ 这一条**不**决定三件套在不在跑"
       f"（那条支路 `rcS:346-353` 只管 `SYS_HITL == 2`）：SITL 走的是 "
       f"`px4-rc.simulator:21-31`，按 `SENS_EN_*SIM` 起；见下面「传感器仿真的源」那两条")
    ck("地面静息：真值高度 ≈ 0（仪器零点）",
       obs.get("gt_rest", {}).get("z") is not None and abs(obs["gt_rest"]["z"]) < 0.05,
       f"真值 `z` = {v.fmt((obs.get('gt_rest') or {}).get('z'), '+.4f')} m、"
       f"`vz` = {v.fmt((obs.get('gt_rest') or {}).get('vz'), '+.4f')} m/s"
       f"（NED：z 向下为正，所以「在地面」就是 0）")
    dz = _diff(obs.get("ekf_rest"), obs.get("gt_rest"), "z")
    ck("★ 地面静息：EKF 的高度与真值一致（差 < 0.25 m）", dz is not None and abs(dz) < 0.25,
       f"EKF `z` = {v.fmt((obs.get('ekf_rest') or {}).get('z'), '+.4f')} m、"
       f"真值 {v.fmt((obs.get('gt_rest') or {}).get('z'), '+.4f')} m ⟹ 差 "
       f"{v.fmt(dz, '+.3f')} m。★ 这一条**不过**就意味着：A 档「稳在 1 m」这个验收项"
       f"在这台被控对象上首先取决于估计器，而不是控制器")
    dxy = _diff_xy(obs.get("ekf_rest"), obs.get("gt_rest"))
    ck("★ 地面静息：EKF 的水平位置与真值一致（差 < 0.25 m）",
       dxy is not None and dxy < 0.25,
       f"EKF (x,y) = ({v.fmt((obs.get('ekf_rest') or {}).get('x'), '+.3f')}, "
       f"{v.fmt((obs.get('ekf_rest') or {}).get('y'), '+.3f')})、真值 "
       f"({v.fmt((obs.get('gt_rest') or {}).get('x'), '+.3f')}, "
       f"{v.fmt((obs.get('gt_rest') or {}).get('y'), '+.3f')}) ⟹ 差 {v.fmt(dxy, '.3f')} m")
    f = obs.get("flags") or {}
    aids = [k for k in ("cs_baro_hgt", "cs_rng_hgt", "cs_gps_hgt", "cs_ev_hgt") if f.get(k)]
    ck("EKF2 至少有一个**高度源**在融合（baro / 测距 / GPS / 外部视觉）", bool(aids),
       f"`estimator_status_flags`：高度源 = {aids or '**一个都没有**'}；"
       f"完整读数 " + " ".join(f"{k}={f.get(k)}" for k in EST_FLAGS if f.get(k) is not None))
    ck("EKF2 有水平位置源（GPS / 外部视觉）",
       bool([k for k in ("cs_gps", "cs_ev_pos") if f.get(k)]),
       f"cs_gps={f.get('cs_gps')} cs_ev_pos={f.get('cs_ev_pos')} "
       f"⟹ 水平位置{'有' if f.get('cs_gps') or f.get('cs_ev_pos') else '**没有**'}源"
       f"（没有的话 EKF 的 x/y 只能靠加速度计二次积分）")
    ck("★ DDS 上能看到真值话题（`--source px4-gt` 的前提）", bool(obs.get("ros_topics")),
       f"`ros2 topic list` 里带 groundtruth 的：{obs.get('ros_topics') or '**没有**'}"
       f"（`src/modules/uxrce_dds_client/dds_topics.yaml` 里没有这几条 ⟹ 没桥接就看不到）"
       + (f"；list 失败：{obs.get('ros_topics_error')}" if obs.get("ros_topics_error") else ""))
    # ★ 归因的关键：**SITL 里那两条"助航"是从 EKF 自己的输出喂回去的**（源码一手证据）——
    #   气压计仿真用 `vehicle_global_position.alt` 反算气压（`SensorBaroSim.cpp:107-118`），
    #   GPS 仿真用 EKF 的 lat/lon/alt + 速度加噪（`SensorGpsSim.cpp:117-121`）。
    #   ⟹ `cs_gps=True` / `cs_baro_hgt=True` **不能**当作"估计器有真值可依"来读：
    #   它是自反馈，估偏了不会有任何一路把它拉回来。这一条不判通过与否，但**必须印出来**，
    #   否则"EKF 差 0.4 m"会被一路读成控制器的问题。
    ck("★ 传感器仿真的源（源码一手证据）：喂的是 EKF 自己的输出，不是真值", None,
       "`sensor_baro_sim`：气压 = f(EKF `vehicle_global_position.alt`) + 1 Pa RMS 噪声"
       "（`src/modules/simulation/sensor_baro_sim/SensorBaroSim.cpp:107-118`、"
       "`:154-155`；`_baro_drift_pa_per_sec` 默认 0）；"
       "`sensor_gps_sim`：lat/lon = EKF + N(0, 0.2 m)、alt = EKF + N(0, 0.5 m)、"
       "v = EKF v + σ(0.06, 0.077, 0.158)"
       "（`src/modules/simulation/sensor_gps_sim/SensorGpsSim.cpp:117-121`）。"
       "⟹ 这台 SITL 上，EKF 的位置/高度**没有任何一路来自 SIH 真值**："
       "助航是自反馈，误差不落地、也没人能纠它", counts=False)

    if obs.get("dwell"):
        d = dwell_summary(obs["dwell"])
        # 「在爬」的判据：净变化明显超过这段里的**噪声带**（用首末差与整体标准差比）。
        # ★ 阈值只是**读法**（2σ），不是验收线；真正的判据是上面那两条 ❌/✅。
        drifting = (d.get("dz_net") is not None and d.get("dz_std") is not None
                    and d["dz_std"] > 0 and abs(d["dz_net"]) > 2.0 * d["dz_std"])
        ck("★ 地面静息：EKF 高度误差是「有界噪声」还是「在爬」（决定闭环追的是什么）",
           None,
           f"{v.fmt(d.get('span_s'), '.1f')}s / {d.get('n')} 次采样：EKF−真值 `z` 从 "
           f"{v.fmt(d.get('dz_first'), '+.3f')} 到 {v.fmt(d.get('dz_last'), '+.3f')} m"
           f"（净 {v.fmt(d.get('dz_net'), '+.3f')} m）、标准差 {v.fmt(d.get('dz_std'), '.3f')} m、"
           f"|最大| {v.fmt(d.get('dz_abs_max'), '.3f')} m；水平误差最大 "
           f"{v.fmt(d.get('dxy_max'), '.3f')} m、净 {v.fmt(d.get('dxy_net'), '+.3f')} m ⟹ "
           + ("**在爬**（净变化 > 2× 噪声带）：闭环追的是一个越走越远的量，"
              "真值高度判据不过与模型/符号无关"
              if drifting else
              "**有界**（净变化 ≤ 2× 噪声带）：误差量级 = 这台 SITL 的噪声带，"
              "高度判据的容差得按它定，不是按 0"),
           counts=False)

    if obs.get("gt_rest2") is not None or obs.get("sims_reply"):
        dz2 = _diff(obs.get("ekf_rest2"), obs.get("gt_rest2"), "z")
        f2 = obs.get("flags2") or {}
        aids2 = [k for k in ("cs_baro_hgt", "cs_rng_hgt", "cs_gps_hgt", "cs_ev_hgt") if f2.get(k)]
        ck("★ 三件套本来就在跑（`--start-sims` 的应答就是证据）", bool(obs.get("sims_reply")),
           f"三条命令的应答：{obs.get('sims_reply') or '（没发）'}；之后 EKF 认的高度源 = "
           f"{aids2 or '**仍然没有**'}、EKF-vs-真值的高度差 {v.fmt(dz, '+.3f')} ⟹ "
           f"{v.fmt(dz2, '+.3f')} m")
    return c


def _diff(a, b, key):
    if not a or not b or a.get(key) is None or b.get(key) is None:
        return None
    return a[key] - b[key]


def _diff_xy(a, b):
    if not a or not b or a.get("x") is None or b.get("x") is None:
        return None
    return math.hypot(a["x"] - b["x"], a["y"] - b["y"])


def dwell_estimator_vs_truth(px, rec, duration_s, period_s=2.0):
    """地面静息**反复**采同一对读数（EKF / 真值），把"误差在爬"与"误差是噪声"分开。

    ★ 一次快照做不到这件事：静息时 EKF 的误差 0.32 m 与 0.43 m 看起来是同一个数
      （都在 0.4 上下），而前者是"自反馈随机游走"、后者是"稳定的偏置"—— 对 A 档的含义完全相反：
      偏置可以在验收时扣掉，游走不能。
    ★ 采样按 `period_s` 走**墙钟**（`rec.now()`），不是按帧数：`sample_pose` 每发一次
      `listener` 就要等它回一屏，单次耗时不是常数。
    """
    rows = []
    t0 = rec.now()
    while True:
        t_before = rec.now()
        gt = sample_pose(px, GT_TOPIC, rec)
        ekf, valid = sample_pose(px, "vehicle_local_position", rec, with_valid=True)
        rows.append({"t": round(rec.now() - t0, 2), "gt": gt, "ekf": ekf,
                     "ekf_valid": valid})
        if rec.now() - t0 >= duration_s or len(rows) >= 40:
            break
        spin_all(px, [rec], max(0.0, period_s - (rec.now() - t_before)))
    return rows


def dwell_summary(rows) -> dict:
    """把静息采样压成：首/末/净、标准差（噪声带）、水平误差的最大与净。"""
    dz = [d for d in (_diff(r.get("ekf"), r.get("gt"), "z") for r in rows) if d is not None]
    dxy = [d for d in (_diff_xy(r.get("ekf"), r.get("gt")) for r in rows) if d is not None]
    out = {"n": len(rows), "span_s": rows[-1]["t"] if rows else None}
    # ★ `with_valid=True` 的第二个返回值是**一份标志字典**（不是布尔）⟹ 要先去重再排序：
    #   直接塞进 set 会 TypeError（本工具第一版就是这么把一整个 36 s 的静息段丢掉的）。
    seen = []
    for r in rows:
        s = json.dumps(r.get("ekf_valid"), sort_keys=True, default=str)
        if s not in seen:
            seen.append(s)
    out["ekf_valid_seen"] = seen
    if dz:
        out.update({"dz_first": dz[0], "dz_last": dz[-1], "dz_net": dz[-1] - dz[0],
                    "dz_std": statistics.pstdev(dz), "dz_abs_max": max(abs(x) for x in dz)})
    if dxy:
        out.update({"dxy_first": dxy[0], "dxy_last": dxy[-1], "dxy_net": dxy[-1] - dxy[0],
                    "dxy_max": max(dxy)})
    return out


# ------------------------------------------------------------------ 飞一场


def dut_params(cfg) -> list:
    """给 `mpc_node.py` 的参数（`--ros-args -p 名:=值`）。

    ★ 频率**不改**（票面 100 Hz 是候选上限）：D6 就是"先按 100 Hz 量最坏耗时、
      再按实测降档"——先降档再量等于把问题绕过去。

    ★★ `hold_yaw_deg` 是 **ticket 16 验收第 3 项唯一的旋钮**（2026-09-30 补）：
      它是**参考机头**（绝对、ENU，`mpc_node.py` 的 `hold_yaw_deg`），而 SITL 里车辆解锁时
      机头 ≈ 0 ⟹ 传 `20` 就等于给出一条**已知大小**的机头误差（≈ 20°），不必另造扰动注入。
      ⚠️ 这个参数**一直存在**于 `mpc_node`，只是这份参数表**从来没传过**它 ⟹ 从前每一场都在
      `hold_yaw_deg = 0°`（节点默认）下飞，机头那一列因此**没有误差可看**。少这一行，
      那一项就只能靠"没人推它"来通过 —— 正是验收正文点名不要的那种通过。

    ★★★ **它旁边还有一个闸门，漏了同样是静默的**（2026-09-30 复核抓出，本函数第一版就漏了）：
      `mpc_node` 的 `keep_yaw` **默认 True**，而 `control_config()` 里那一行是
      `yaw = None if keep_yaw else radians(hold_yaw_deg)`（`mpc_node.py`）⟹ **默认档下
      `hold_yaw_deg` 被整条忽略**，`ControlConfig.hold_yaw` 是 `None`，于是
      `node_core.hold_reference` 取的是**锚点那一拍的机头**（`node_core.py` 的
      `yaw = config.hold_yaw if ... is not None else yaw_from_quat(x[6:10])`）——
      参考机头 ≈ 实测机头，**误差恒为噪声**。飞出来一场"全绿"，而 ⑩ 只报"没做"。
      ⟹ `hold_yaw` 非 0 时**必须**同时把它关掉。★ 只在非 0 时传：`hold_yaw_deg = 0` 的场次
      保持"锚点机头"这个**老行为**一个字不变（绝对 0° 与"解锁机头"在真机上不是一回事）。
    """
    params = [f"log_file:={cfg.log_file}",
              "state_source:=px4",
              f"hold_offset_x:={cfg.hold_x}",
              f"hold_offset_y:={cfg.hold_y}",
              f"hold_offset_z:={cfg.hold_z}",
              f"hold_yaw_deg:={cfg.hold_yaw}",
              f"rate_hz:={cfg.rate_hz}"]
    if cfg.hold_yaw:
        # ★ `keep_yaw:=false` 是 `hold_yaw_deg` 的**使能位**，不是可选优化：见 docstring ★★★
        params.insert(5, "keep_yaw:=false")
    if cfg.source == "px4-gt":
        params += [f"px4_position_topic:=/fmu/out/{GT_TOPIC}",
                   f"px4_attitude_topic:=/fmu/out/{GT_ATT_TOPIC}"]
    return params


def run_flight(px, rec, dut, cfg, agent) -> dict:
    """一场闭环：采样真值 → 解锁 → OFFBOARD → 悬停 `hold_s` 秒 → 收尾采样。

    时序上只有一条硬约束：**真值采样必须从解锁之前就开始**（地面静息是后面所有高度判据的
    零点，而且它证明这台仪器在起飞前读数是对的）。解锁走 ROS 就是为了把控制台留给采样器。
    """
    obs = {"name": cfg.name, "source": cfg.source, "params": dut_params(cfg),
           "rate_hz": cfg.rate_hz, "truth_hz_want": cfg.truth_hz,
           "gt_att_hz_want": cfg.gt_att_hz, "hold_s": cfg.hold_s, "t": {}}
    nodes = [rec, agent]
    sampler = TruthSampler(px, GT_TOPIC, rec, period_s=1.0 / max(0.5, cfg.truth_hz))
    # ★ 姿态真值的**第二条链**（判据⑨）：慢一拍（默认 1 Hz）就够 —— 要的是"转了多
    #   少、朝哪边转"这种累积量，不是振动；而控制台流量是这场仪器的稀缺资源
    #   （`TruthSampler` 的 ★★★ 就是流量把被控对象堵死换来的）。
    att = TruthSampler(px, GT_ATT_TOPIC, rec, period_s=1.0 / max(0.2, cfg.gt_att_hz),
                       fields=(), arrays=("q",))
    def tick():                 # 两条链在同一个 spin 循环里推（各自非阻塞、各自记账）
        sampler.poll()
        att.poll()
    obs["t"]["sampler_start"] = round(rec.now(), 2)

    if not wait_until(px, nodes, lambda: dut.frames(), cfg.dut_timeout_s,
                      "DUT 一帧都没发（节点没起来？）", tick=tick):
        obs["dut_dead"] = True
    obs["t"]["first_frame"] = round(rec.now(), 2)

    # 解锁 + 切 OFFBOARD：等到记录器**看见** nav_state=14（记录器的时钟，另一个口径）
    t_before = rec.now()
    wait_until(px, nodes, lambda: rec.first_nav(v.NAV_STATE_OFFBOARD, since=t_before) is not None,
               cfg.offboard_timeout_s, "PX4 没进 OFFBOARD", tick=tick)
    t_enter = rec.first_nav(v.NAV_STATE_OFFBOARD, since=t_before)
    obs["t"]["offboard_enter"] = None if t_enter is None else round(t_enter - t_before, 2)
    obs["armed_ever"] = rec.armed_ever(t_before, rec.now())
    obs["agent"] = agent.describe()

    # 悬停窗口（墙钟）：OFFBOARD 生效起算
    if t_enter is not None:
        spin_all(px, nodes, cfg.hold_s, tick=tick)
    obs["t"]["flight_end"] = round(rec.now(), 2)

    rows, meta = sampler.track()
    obs["truth"], obs["truth_meta"] = rows, meta
    obs["gt_att"], obs["gt_att_meta"] = att.payload_rows(("q",))
    obs["truth_n"] = meta["n"]
    obs["truth_hz"] = meta.get("hz")
    obs["truth_span_s"] = meta.get("vehicle_span_s")
    # ★ 判据⓪ 的两个读数：**同一对样本**的两种时间。比 ≈1 才说明这台仪器没把被控对象卡住。
    obs["stream_wall_s"] = meta.get("wall_span_s")
    obs["truth_missed"] = meta.get("missed")

    # 收尾时的几个"同一时刻"读数：电机（PX4 解出来的推力）、EKF 的位置、融合标志、姿态。
    # ★ 这些都在**采样器停了之后**采（采样期间控制台腾不出来，见 `TruthSampler` 的说明）；
    #   飞机还在悬停（节点走完不停发），所以末段读数与"悬停段"是同一档工况。
    mo = v.sample_topic(px, "actuator_outputs_sim", timeout=SAMPLE_TIMEOUT_S, rec=rec)
    obs["motors_end"] = v.motor_line(mo)
    obs["motors_mean_end"] = v.motor_mean(mo)
    obs["thrust_n_end"] = v.thrust_from_motors(mo)
    obs["ekf_end"], obs["ekf_end_valid"] = sample_pose(
        px, "vehicle_local_position", rec, with_valid=True)
    obs["gt_end"] = sample_pose(px, GT_TOPIC, rec)
    obs["flags_end"] = sample_flags(px, "estimator_status_flags", rec=rec)
    obs["att_end_ts"] = v.field(v.sample_topic(px, "vehicle_attitude", timeout=SAMPLE_TIMEOUT_S,
                                               rec=rec), "timestamp")
    st = v.sample_topic(px, "vehicle_status", timeout=SAMPLE_TIMEOUT_S, rec=rec)
    obs["status_end"] = {"nav_state": v.field(st, "nav_state"),
                         "arming_state": v.field(st, "arming_state"),
                         "pre_flight_checks_pass": v.bool_field(st, "pre_flight_checks_pass")}

    frames = dut.frames()
    obs["dut"] = summarise_frames(frames)
    obs["solve"] = solve_stats(frames)
    obs["yaw_pair_diag"] = {}
    obs["yaw_pairs"] = yaw_pairs(frames, obs.get("gt_att") or (), diag=obs["yaw_pair_diag"])
    # ★★ 判据⑨ 的**别名闸门**的原料：DUT 帧是 100 Hz 的（每拍 10 ms），它自己的机头/指令
    #   两路都可信；真值那一路只有 1 Hz。飞机转得快时（实测 4.9 rad/s）1 Hz 的样本
    #   **每个间隔转过 ~4.9 rad > π** ⟹ 最短路径展开把角**记反/记少**（−54 记成 −26.5），
    #   判据⑨a 就报出"指令把飞机往反方向转"这种**归因错误**的 ❌（2026-09-28 实测）。
    #   ⚠️ 必须在**采集侧**算好塞进 obs：判据是 obs 的纯函数（`--replay` 不重跑 SITL）。
    obs["yaw_rate_bound"] = yaw_rate_bound(frames)
    stdout = dut.sigint()
    obs["stdout_tail"] = "\n".join(stdout.strip().splitlines()[-8:])
    for line in stdout.splitlines():
        if "收尾：" in line:
            obs["summary"] = line.strip()
    return obs


def summarise_frames(frames) -> dict:
    """DUT 逐帧日志的**汇总**（帧数、实际频率、状态源、档位、饱和、指令）。"""
    if not frames:
        return {"frames": 0}
    span = (frames[-1]["wall_ns"] - frames[0]["wall_ns"]) / 1e9
    fallback = {}
    for f in frames:
        fallback[f.get("fallback")] = fallback.get(f.get("fallback"), 0) + 1
    normal = [f for f in frames if f.get("fallback") == "normal"]
    sat_any = sorted({k for f in frames for k in (f.get("sat") or [])})
    return {
        "frames": len(frames),
        "span_s": round(span, 2),
        "actual_hz": round((len(frames) - 1) / span, 2) if span > 0 else None,
        "nav_seen": sorted({int(f.get("nav_state", -1)) for f in frames}),
        "state_source": frames[-1].get("state_source"),
        "fallback": fallback,
        "sat_any": sat_any,
        "sat_count": {k: sum(1 for f in frames if k in (f.get("sat") or []))
                      for k in sat_any},
        #: ★ **饱和标志这条判据要的是"能力在日志里"，不是"这一场真的饱和过"**（票据原话：
        #: "日志记录 … 约束饱和标志，供飞行后复盘"）。第一版把它写成了 `bool(sat_any)`
        #: ⟹ 一场**没有饱和**的飞行被判成 ❌（判据⑧），而那一场恰恰是"没顶约束"的证据。
        #: 改成"每一帧都带 `sat` 这一列"，出现次数照旧报（`sat_count`）。
        "sat_logged": all("sat" in f for f in frames),
        #: 位置保持锚点的事件列（`""` = 常态）：`["anchor"]` 是本场该有的样子。
        "ref_events": sorted({str(f.get("ref_anchor") or "") for f in frames} - {""}),
        "ages_ok": all(isinstance(f.get("age_pos_s"), (int, float))
                       and isinstance(f.get("age_att_s"), (int, float)) for f in frames),
        "u_end": normal[-1].get("u") if normal else None,
        "thrust_body_z_seen": sorted({round(float(f["thrust_body_z"]), 4) for f in frames
                                      if f.get("thrust_body_z") is not None})[:6],
        #: ★ 末段（最后 100 个正常帧）的 `thrust_body_z` 中位数 —— 判"悬停段的推力落在哪"
        #: 要看**末段**，不是整场的取值集合：起飞那一拍本来就该接近 −1（要抬起来），
        #: 混在一起看会把"起飞"的读数当成"悬停"的读数（第一版就印错过一行）。
        # ⚠️ `statistics.median([])` 会**抛异常**（`StatisticsError`）：这里先挡一下。
        #   理由与 `segments()` 里那次一样 —— 报告层崩在"飞完一整场之后"，
        #   丢掉的是 80 s 的实测（那一次是靠 `finally` 里落盘 obs.json 才救回来的）。
        "thrust_body_z_tail": (statistics.median(
            [float(f["thrust_body_z"]) for f in normal[-100:]
             if f.get("thrust_body_z") is not None])
            if normal and any(f.get("thrust_body_z") is not None for f in normal[-100:])
            else None),
        # ⚠️ 同样先挡住空集：`fmean` 空序列也是抛异常（理由同上一条）。
        "u_mean_normal": ([statistics.fmean(f["u"][i] for f in normal) for i in range(4)]
                          if normal and all(isinstance(f.get("u"), list) and len(f["u"]) >= 4
                                            for f in normal)
                          else None),
        "ekf_flags_seen": sorted({(f.get("ekf_xy_valid"), f.get("ekf_z_valid"))
                                  for f in frames}),
        "track": frame_track(frames, tail_s=3.0),
    }


def frame_track(frames, tail_s=3.0, head_n=20):
    """从逐帧日志里取**节点自己看到的那一条高度**（`x0`，ENU，`[2]` = up）——"状态源说飞机在哪"。

    ★ 为什么必须单开这一路：闭环里只有**一个**高度是控制器看得见的（EKF 的），
      真值是**被控对象**那一侧的量，两者在这台 SITL 上相差 0.4 m 量级
      （源码一手证据：气压计/GPS 仿真喂的是 EKF 自己的输出，见 `evaluate_plant`）。
      ⟹ "稳在 1 m"这件事只能在 `x0` 这一路上判成"控制器的功劳/锅"；
      真值那一路另行报告（它是被控对象的读数，不是控制器的成绩单）。
    ★ `head` 取**最前面** `head_n` 帧（还没离地）；`tail` 取最后 `tail_s` 秒（悬停段）。
      与真值那一路的分段**刻意不同源**：这里没有"从零识别起飞"的问题 —— 日志的第一帧
      必然是地面（节点一开始就发帧，解锁在之后）。
    """
    normal = [f for f in frames
              if f.get("fallback") == "normal" and isinstance(f.get("x0"), list)
              and len(f["x0"]) >= 6]
    if len(normal) < head_n + 5:
        return {}
    t_end = normal[-1]["wall_ns"] / 1e9

    def col(idx, rows):
        return [float(f["x0"][idx]) for f in rows if isinstance(f["x0"][idx], (int, float))]

    tail = [f for f in normal if f["wall_ns"] / 1e9 >= t_end - tail_s]
    up_head, up_tail = col(2, normal[:head_n]), col(2, tail)
    if not up_tail or not up_head:
        return {}
    span = (tail[-1]["wall_ns"] - tail[0]["wall_ns"]) / 1e9
    ex, ey = col(0, tail), col(1, tail)
    # ★ 参考（`x_ref0`）自己的轨迹 —— 判"目标有没有钉住"（见下面 `ref_track` 的说明）。
    ref = [f for f in normal if isinstance(f.get("x_ref0"), list) and len(f["x_ref0"]) >= 3]
    ref_z = [float(f["x_ref0"][2]) for f in ref]
    ref_xy = [(float(f["x_ref0"][0]), float(f["x_ref0"][1])) for f in ref]
    # 参考误差（`x0 − x_ref0`，即"要它走的修正量"）：头 20 帧 vs 末 100 帧。
    err_head = [float(f["x0"][2]) - float(f["x_ref0"][2]) for f in ref[:head_n]]
    err_tail = [float(f["x0"][2]) - float(f["x_ref0"][2]) for f in ref[-100:]]
    # 机头（EKF 帧）：逐帧相位序列 → 展开成累计转角（自转超过 ±π 也不丢圈数）
    psi_t = [(f["wall_ns"] / 1e9, yaw_enu_from_quat(f["x0"][6:10])) for f in normal
             if isinstance(f.get("x0"), list) and len(f["x0"]) >= 10]
    psi = unwrap([p for _, p in psi_t]) if psi_t else []
    yt = [(t, p) for (t, _), p in zip(psi_t, psi) if t >= t_end - tail_s]
    u3_tail = (statistics.fmean(float(f["u"][3]) for f in normal[-100:])
               if normal and all(isinstance(f.get("u"), list) and len(f["u"]) >= 4
                                 for f in normal[-100:]) else None)
    # ── 机头误差 vs 角速率指令（判据⑩ 的原料，ticket 16 验收第 3 项）──
    # ★ 两路都取**同一帧**：参考机头 `x_ref0_q` 与实测机头 `x0[6:10]`（都是 EKF 帧、同一拍），
    #   配那一帧的 `u[3]`。★ 取**末段**（悬停段）：那时位置已在参考附近 ⟹ `u[3]` 里
    #   占比最大的就是机头这一项，不会被位置误差那条路串进来。
    # ★ 用**中位数**而不是均值：RTI 逐拍带一点依赖热启动的抖动，千帧中位数把它的方差压掉
    #   约 √1000（判据要问的是"这个偏差下指令往哪边走、走多大"，不是逐拍噪声）。
    pairing = []
    for f in tail:
        q_ref = f.get("x_ref0_q")
        if not (isinstance(q_ref, list) and len(q_ref) >= 4
                and isinstance(f.get("x0"), list) and len(f["x0"]) >= 10
                and isinstance(f.get("u"), list) and len(f["u"]) >= 4):
            continue
        pairing.append((wrap_pi(yaw_enu_from_quat(q_ref) - yaw_enu_from_quat(f["x0"][6:10])),
                        float(f["u"][3])))
    # ★ 只留**真有误差**的那些帧：没注入误差（`--hold-yaw 0`）时 `pairing` 是满的、
    #   而误差全是噪声 ⟹ 那不算"判过"，算"没判"（`evaluate` 记 `ok=None`）。
    big = [(dq, u3) for dq, u3 in pairing if abs(dq) >= YAW_RESTORE_MIN_RAD]
    return {"n": len(normal),
            "up_head": statistics.fmean(up_head), "up_tail": statistics.fmean(up_tail),
            "climb": statistics.fmean(up_tail) - statistics.fmean(up_head),
            "up_tail_min": min(up_tail), "up_tail_max": max(up_tail),
            "up_tail_p2p": max(up_tail) - min(up_tail),
            "up_tail_std": statistics.pstdev(up_tail),
            "up_tail_osc_hz": crossings_series(up_tail, span),
            "vup_head": statistics.fmean(col(5, normal[:head_n]) or [float("nan")]),
            "vup_tail": statistics.fmean(col(5, tail) or [float("nan")]),
            "xy_drift_tail": (math.hypot(ex[-1] - ex[0], ey[-1] - ey[0])
                              if ex and ey else None),
            "tail_s": round(span, 2),
            #: ★★ 参考自身的跨度 —— **这一项才是"目标有没有钉住"的测量**，而不是高度误差。
            #: 首飞那场（参考每拍 = 测量 + 偏移）它一定等于**飞机走的距离**（实测 32 m），
            #: 锚定之后它该接近 0（锚点 + 偏移是个常量）。
            #: ⚠️ 判"误差关没关掉"要看 `referr_tail`，判"目标钉没钉住"看 `ref_span_z`——
            #: 两者是**不同**的失效：参考钉住而飞机没到 = 控制器/模型的事（判据③④）；
            #: 参考在跟着测量走 = 参考取法的事（本轮就是栽在这里）。
            "ref_span_z": (max(ref_z) - min(ref_z)) if ref_z else None,
            "ref_span_xy": (max(math.hypot(x, y) for x, y in ref_xy)
                            - min(math.hypot(x, y) for x, y in ref_xy)) if ref_xy else None,
            "referr_head": statistics.median(err_head) if err_head else None,
            "referr_tail": statistics.median(err_tail) if err_tail else None,
            "ref_n": len(ref),
            # ── 机头（判据⑥b）──
            # ★ 用**节点自己那一帧的 `x0[6:10]`**（= PX4 `vehicle_attitude` 经我们的换算，
            #   也就是 EKF 帧）—— 与操作者裁定的"验收判在 EKF 帧上"一致。真值姿态是**背景**，
            #   它只用来给判据⑨ 做归因（见 `yaw_pairs`）。
            # ★★ 为什么要单列这一路：悬停时机头不该持续转。实测那几场它在自转（累计 31 rad，
            #   约 5 圈/38 s），而**没有一条判据会因此变红** —— 位置那几条全是 ✅，
            #   因为位置参考是锚死的、机头自转在位置上只表现为小幅漂移。这个洞是判据⑥b 补的。
            "yaw_ekf_span": (psi[-1] - psi[0]) if psi else None,
            "yaw_ekf_tail_span": (yt[-1][1] - yt[0][1]) if yt else None,
            "yaw_ekf_tail_rate": ((yt[-1][1] - yt[0][1]) / tail_s) if yt else None,
            # ── 机头回正（判据⑩）──
            "yaw_restore_n": len(big),                       # 末段里"真有误差"的帧数
            "yaw_restore_n_all": len(pairing),               # 末段里配得上对的帧数（分母）
            "yaw_restore_err": (statistics.median([dq for dq, _ in big]) if big else None),
            "yaw_restore_u3": (statistics.median([u3 for _, u3 in big]) if big else None),
            "u3_tail": u3_tail}


#: D6 的档位（票面原话）：**实测最坏**求解耗时 ≤10 ms ⟹ 100 Hz 有余；
#: 10–20 ms ⟹ 降 50 Hz；>20 ms ⟹ **停下优化，不硬上**。
D6_LADDER = ((10.0, "100 Hz（最坏 ≤10 ms，余量够）"),
             (20.0, "50 Hz（10–20 ms，降档）"),
             (math.inf, "**停下优化**（最坏 >20 ms，不硬上）"))

#: 「参考锚定」那条判据的名字前缀（`evaluate` 起名、`attribute` 按前缀找它）。
#: ★ 它**不属于**票面那三条归因（模型 / 符号映射 / 中间件）里的任何一条 —— 它是"参考怎么取"，
#:   也是 ticket 09 首飞用的第一手：那一场飞机以 0.9 m/s 爬了 37 s、看起来像模型发散。
CK_ANCHOR = "★ 参考锚定"

#: 偏航那两条判据的名字前缀（`attribute` 按前缀找它们）。
CK_YAW_MAP = "⑨ 偏航：指令方向与被控对象真的转的方向一致"
CK_YAW_CONV = "⑨ 偏航：我们换算出来的机头朝向 == 教科书那一个"
CK_YAW_RESTORE = "⑩ 偏航：有已知机头误差时指令把它往参考拉（符号 + 量级）"

#: ⑩ 用的常数 —— **都不是判据自己的发明**，是把已经量到的东西写成常数：
#: ① `R_YAW` = 落地权重 `r_diag[3]`（`OcpConfig` 的默认，`src/nmpc/nmpc/ocp.py`）。
#:    ★ 它由 `test_tier_a_judges.py::test_偏航回正判据的权重常数没跟着_OcpConfig_漂` 钉住 ——
#:      权重一改而这里没跟，这条判据量的就是**别的东西**，而且**照样绿**（本仓库的老坑）。
#: ② `YAW_TAU_PER_R = 2.05`：闭环机头时间常数与 `R_yaw` 的实测比（`τ ≈ 2.05 · R_yaw`），
#:    逐档量出来的原始 stdout 在 ticket 16 Comments #5 ⑤。
R_YAW = 5e2
YAW_TAU_PER_R = 2.05
#: 期望的角速率指令 = `Δψ / τ`，`τ = YAW_TAU_PER_R · R_YAW`（落地权重 ⟹ ≈ 1025 s）。
YAW_TAU_S = YAW_TAU_PER_R * R_YAW
#: 判"这一场真的注入了机头误差"的下限 [rad]：小于它**不判**（记"没做"）——
#: 误差太小时 `u[3]` 被别的项盖住，判了也说明不了什么。
YAW_RESTORE_MIN_RAD = math.radians(5.0)
#: 量级判带的乘法宽度：期望值的 `1/4 … 4` 倍。
#: ★ **只有符号是锐的**；幅值这一带是"别差一个数量级"的信封 —— 真 SITL 多了姿态环滞后，
#:   实测增益只会比离线预测**小**，所以**下限比上限要紧**（下限抓"权威被压到 ≈0"）。
YAW_RESTORE_BAND = (0.25, 4.0)


def yaw_ned_from_quat(q_ned_frd) -> float:
    """**NED/FRD** 四元数 → NED 偏航角 ψ [rad]（绕 z 朝下、从北往东为正）。

    ⚠️ 刻意**不调** `node_core.yaw_from_quat`：那条路吃的是**已经换算过**的 ENU/FLU
    四元数。判据⑨ 要问的正是"我们的换算对不对"，拿被检验的东西去检验自己是循环论证。
    这里只用一个**教科书公式 + PX4 自己的定义**（ψ 绕 z_down、北→东为正，右手法），
    与被检验的代码无关。

    与 `node_core.yaw_from_quat` 的代数形式相同（同一个 ZYX 取法），但**输入是不同的东西**：
    这里是 NED/FRD 原样、那边是 `px4_quat_to_enu()` 的产物 —— 两者的差别恰好就是待检验的
    FRD→FLU 换标签。所以这一路能当"独立参照"用。
    """
    w, x, y, z = (float(t) for t in q_ned_frd)
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def wrap_pi(a: float) -> float:
    """把一个角包进 `(−π, π]`。"""
    return (float(a) + math.pi) % (2.0 * math.pi) - math.pi


def yaw_enu_from_quat(q_enu_flu) -> float:
    """`x0[6:10]`（**已经换算过**的 ENU/FLU 四元数）→ 机头朝向 [rad，从 +x 逆时针]。

    ★ 与 `node_core.yaw_from_quat` 同一个 ZYX 取法 —— 这里**故意**再写一遍而不是 import：
      A 档这台仪器要能脱离 `nmpc` 包单独跑（它已经在 import `nmpc.params` 拿悬停推力，
      但那是**数据**、不是判据）。取法本身是教科书公式，重复不会漂移；
      真要漂移的是"那个公式用在哪一路输入上"，而那正是判据⑨ 要盯的。
    """
    w, x, y, z = (float(t) for t in q_enu_flu)
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def unwrap(seq) -> list:
    """把一串角（rad）按**最短路径**展开成连续量：判"转了多少、朝哪边转"必须用它。

    ⚠️ 少了这一步会得出"偏航在 −π/+π 之间跳"这种读数，而下面两条判据问的都是**累计转角**。
    """
    out = [float(seq[0])]
    for a in seq[1:]:
        d = (float(a) - out[-1] + math.pi) % (2.0 * math.pi) - math.pi
        out.append(out[-1] + d)
    return out


#: 别名闸门阈值 [rad]：**一个采样间隔**里转过的角到这个量级，最短路径展开就再也解不出来
#: （`unwrap` 假定 `|Δ| < π`）。取 `0.9π` 而不是 `π`：留一成余量给"转速本身还在变"。
ALIAS_STEP_RAD = 0.9 * math.pi

#: ⑨a（累计转角）的**判别力下限** [rad]：指令累计转角小于它，这条判据**证不了方向**。
#:
#: 判据是 `|resid| < 0.25·|cmd_span| + 0.35`。另一个世界（指令**反了**）给出的读数是
#: `|psi_span − cmd_span|`，而健康时 `|psi_span| ≈ |cmd_span|` ⟹ 那个世界约是 `2·|cmd_span|`。
#: 只有当 `2·|cmd_span| > 0.25·|cmd_span| + 0.35`，即 **`|cmd_span| > 0.2`** 时，反了的指令
#: **才会**被判 ❌。★ 第一手（2026-09-28）：第 8 场 `cmd_span = −0.0022` ⟹ 阈值 0.3505，
#: 就算 yaw 的符号整个反了，resid 也只会是 0.086 —— **照样 ✅**。那一场**没有**证明方向，
#: 它只是"没矛盾"；真正有力的读数是第 5 场（`cmd_span = +7.94`，反了会给出 −15.7）。
#: ⚠️ 阈值写成常数而不是从 `0.35/1.75` 算出来：那两个数都来自经验（判据自己的噪声地板），
#: 这里要的是"这条判据什么时候**有劲**"的一个可读门槛。
YAW_POWER_RAD = 0.2


def yaw_rate_bound(frames) -> Optional[float]:
    """"这段飞行到底转得多快"的**上界** [rad/s] —— 判据⑨ 别名闸门的原料。

    取两路的**较大者**，两路都出自 100 Hz 的 DUT 帧（每拍 10 ms，`|ω| < 300 rad/s` 都不会
    别名 —— 而真值那一路只有 1 Hz）：

    | 路 | 是什么 | 为什么它可信 |
    |---|---|---|
    | `x0` 的机头序列 | 展开后的**平均**转速（只用**转角大小**） | 同一段旋转在任何偏航约定下**转角大小都一样**，只有符号会翻 ⟹ 与判据⑨b 不循环论证 |
    | `u[3]` | 指令角速率的**峰值** | 就算符号映射整个反了，**幅值**也还是对的 |

    ★ 两路都**稠密**，所以"没转"与"转得快"都不会看错 —— 这与真值那一路（1 Hz、转快了就
      记反记少）正好相反。取较大者是为了**保守**：闸门只会把一条判据降级成"没做"，
      不会把 ❌ 变成 ✅，所以宁可高估转速。
    · 返回 `None`（**不是 0**）只有当帧少到算不出转速 —— 0 会被读成"没转"，那是**有信息
      含量的错答案**；`None` 在判据那边表示"闸门不开"。
    """
    ts, psi, us = [], [], []
    for f in frames or ():
        x0, w = f.get("x0"), f.get("wall_ns")
        if not isinstance(w, (int, float)):
            continue
        u = f.get("u")
        if isinstance(u, list) and len(u) >= 4 and isinstance(u[3], (int, float)):
            us.append(abs(float(u[3])))
        if not isinstance(x0, list) or len(x0) < 10:
            continue
        ts.append(w / 1e9)
        psi.append(yaw_enu_from_quat(x0[6:10]))
    rate = None
    if len(ts) >= 3 and ts[-1] > ts[0]:
        steps = [abs(wrap_pi(psi[k] - psi[k - 1])) for k in range(1, len(psi))]
        if not (steps and max(steps) >= ALIAS_STEP_RAD):   # 帧自己就近别名了 ⟹ 别拿它当基准
            psi_u = unwrap(psi)
            rate = abs((psi_u[-1] - psi_u[0]) / (ts[-1] - ts[0]))
    cands = [c for c in (rate, max(us) if us else None) if c is not None]
    return max(cands) if cands else None


def yaw_pairs(frames, gt_att, tol_s=0.25, diag=None) -> list:
    """把姿态真值样本与 DUT 帧**配对**，并把三样东西放在同一行（判据⑨ 的原料）：

    | 列 | 是什么 | 从哪来 |
    |---|---|---|
    | `psi_ned` | 真值机头朝向（NED 偏航，绕 z 朝下、北→东为正） | `vehicle_attitude_groundtruth.q`，**教科书公式** |
    | `yaw_node` | 节点**换算之后**看到的机头朝向（ENU/FLU） | 逐帧日志的 `x0[6:10]`（即 `px4_quat_to_enu()` 的产物） |
    | `yaw_cmd` | 从第一对起到这一对为止，**指令**让飞机绕 z_FLU 转过的累计角 | 逐帧日志的 `u[3]`（`∫u[3]dt`，梯形法） |

    ★ 为什么要在这里算、而不是在判据里算：**判据必须是 obs 的纯函数**（`--replay` 不重跑
      SITL，见 `evaluate` 的 docstring）。配对要用到帧，所以配对在采集侧做完，判据只读结果。

    ★★ `diag`（可选 dict，会被就地填上计数）：本函数有 **4 条**"返回空"的路，而它们对判据
      的含义完全不同（没采到真值 / 采到了但配不上 / 配上了但帧里缺 `u` / 缺 `x0`）。
      没有 `diag` 时这四条全部塌成"长度为 0 的列表"，判据只能猜 —— 2026-09-28 就猜错过一次
      （漏了配对键，33 个样本一条没配上，报告却写成"没采到姿态真值"）。判据读 `obs["yaw_pair_diag"]`。
    """
    if diag is not None:
        diag.update({"n_gt": len(gt_att or ()), "u_missing": 0, "x0_missing": 0})
    pairs = pair_by_time(gt_att, ("q",), frames, tol_s=tol_s, diag=diag)
    if not pairs:
        return []
    # 同一段配对序列上的指令累计角（用帧自带的 `ts`，µs）
    its = [p["frame"]["ts"] for p in pairs]
    us = [float(p["frame"]["u"][3]) for p in pairs
          if isinstance(p["frame"].get("u"), list) and len(p["frame"]["u"]) >= 4]
    if len(us) != len(pairs):
        if diag is not None:
            diag["u_missing"] = len(pairs) - len(us)
        return []
    cum = [0.0]
    for k in range(1, len(pairs)):
        dt = (its[k] - its[k - 1]) / 1e6
        cum.append(cum[-1] + 0.5 * (us[k] + us[k - 1]) * dt)
    out = []
    for k, p in enumerate(pairs):
        x0 = p["frame"].get("x0") or []
        if len(x0) < 10:
            if diag is not None:
                diag["x0_missing"] = sum(1 for q in pairs
                                         if len(q["frame"].get("x0") or []) < 10)
            return []
        out.append({"t": p["t"], "psi_ned": yaw_ned_from_quat(p["q"]),
                    "yaw_node": yaw_enu_from_quat(x0[6:10]), "yaw_cmd": cum[k]})
    return out


def yaw_verdict(pairs, rate_hint=None) -> dict:
    """两条独立读数（判据⑨ 的两个数）：**指令方向**与**换算本身**。

    · `psi_span` = 真值机头朝向的**累计转角**（解锁展开后头尾之差）
      `cmd_span` = 同一段的**指令累计转角** `∫u[3]dt`
      ⟹ 物理锚点：`dψ_ned/dt == −ω_z_FLU`（`sign-map.md` §5.2 的"yaw 取反"）。
      这一条**完全不经过我们的换算**（ψ 直接从真值四元数按 PX4 自己的定义取），
      所以它能判"指令方向对不对"。
      ⚠️ **但它的判别力与 `|cmd_span|` 同阶**：`cmd_span` 很小（悬停得好）时它**证不了方向**，
      只能说明"没矛盾" ⟹ 用 `yaw_had_power()` 再问一句，别把 `✅` 一律读成"方向被证实"
      （推导与第一手见 `YAW_POWER_RAD`）。
    · `conv_err` = `(π/2 − ψ_ned) − yaw_node`（包到 ±π）的中位数：
      `π/2 − ψ_ned` 是**教科书**给出的"同一台飞机在 ENU 里的机头朝向"
      ⟹ 这一条判"我们的 FRD→FLU 换算对不对"。

    ★★ `rate_hint`（`obs["yaw_rate_bound"]`，可选）：**别名闸门**，只闸**累计转角**那一路。
      真值只有 1 Hz，飞机转得快时相邻两样本转过 > π，`unwrap` 走最短路径会**把角记反/记少**
      ⟹ ⑨a 的 `psi_span` / `resid` 失去意义，而且会给出一个**归因错误**的 ❌
      （2026-09-28：4.9 rad/s 的真值序列把 −53.9 rad 记成 −26.5，报告写成
      "指令把飞机往反方向转 ⟹ 符号映射"）。闸门判据是
      `|转速| × 真值采样间隔 ≥ ` `ALIAS_STEP_RAD`：一个采样间隔里转过的角**解不出来**了。

      ⚠️ ⑨b（`conv_err`）**不闸**：它是**逐样本**比出来的（配对取的是最近那帧，10 ms 量级），
      不做真值序列的展开 ⟹ 那场转着 4.9 rad/s 时它的读数依然干净（中位数 −0.064 rad、
      跨度 0.138 rad，实测）。把两条一起闸会**白白丢掉一条能过的判据**。
      `rate_hint=None`（没有独立的稠密转速参考）⟹ 闸门**不开**，只降级成"照旧判"，
      免得把一次仪器缺失变成一张空白报告。
      ⚠️ **旧观测重放时闸门开不了**：`yaw_rate_bound` 是**采集时**算好存进 obs 的
      （判据必须是 obs 的纯函数），而闸门是 2026-09-28 那场之后才加的 ⟹ 更早的 obs 里
      没有这个键。重放它们会**原样复现那个假 ❌**（`/tmp/tier-a/obs6.json` 就是）——
      报告会在那一行明说"没有别名自检的原料，以重飞为准"，别把它读成新结论。
    """
    out = {"n": len(pairs)}
    if len(pairs) < 3:
        return out
    ts = [p["t"] for p in pairs]
    dts = [ts[k] - ts[k - 1] for k in range(1, len(ts)) if ts[k] > ts[k - 1]]
    out["dt_sample"] = statistics.median(dts) if dts else None
    out["rate_hint"] = rate_hint
    if rate_hint is not None and out["dt_sample"]:
        out["alias_turn"] = abs(float(rate_hint)) * out["dt_sample"]
        out["alias"] = out["alias_turn"] >= ALIAS_STEP_RAD
    else:
        out["alias_turn"] = None
        out["alias"] = None                     # 判不了 ⟹ 不闸（见 docstring 末段）
    psi = unwrap([p["psi_ned"] for p in pairs])
    out["psi_span"] = psi[-1] - psi[0]
    out["cmd_span"] = pairs[-1]["yaw_cmd"]
    out["resid"] = out["psi_span"] + out["cmd_span"]      # 该接近 0（ψ̇ = −ω_z）
    wrapped = [wrap_pi((math.pi / 2.0 - p["psi_ned"]) - p["yaw_node"]) for p in pairs]
    wrapped[0] = wrap_pi(wrapped[0])                      # 起点归一化到 (−π, π]，读数才有意义
    err = unwrap(wrapped)
    out["conv_err_med"] = statistics.median(err)
    out["conv_err_span"] = max(err) - min(err)
    return out


def yaw_had_power(yv) -> bool:
    """⑨a 这一场**有没有劲**判方向：`|∫u[3]dt|` 过不过 `YAW_POWER_RAD`（推导见那里）。

    ★ 从 `cmd_span` **现算**，而不是让 `yaw_verdict` 存一个 `discriminating` 位：
      `--replay` 重放的是**旧观测** —— 那里面没有那一位 ⟹ 读它会得到 `None`，
      于是"第 5 场 +7.94 rad 那一场"会被读成"没劲"。判据是 **obs 的纯函数**，
      那就只认 obs 里**确实有**的键。`False` 不改变通过与否，它管的只是**话怎么说**。
    """
    cs = (yv or {}).get("cmd_span")
    return cs is not None and abs(float(cs)) > YAW_POWER_RAD


def pair_by_time(samples, keys, frames, tol_s=0.25, diag=None) -> list:
    """把真值样本按**墙钟**配到最近的 DUT 帧上。

    ⚠️ **不能拿两边的"时间戳"直接配**（第一版就是这么写错的，症状是"采到 25 点、
      配出 **0** 对"，判据⑨ 静悄悄变成"没做"）：控制台那一路 `listener` 打的是
      **PX4 原始的 `hrt`（开机起算 µs，实测 18.7e6）**，而帧日志的 `ts` 是
      **DDS 桥出来的 Unix 纪元 µs（实测 1.79e15）** —— 两个数**都是 µs、零点差了 56 年**。
      一手依据两头都有：产生侧是 `sih.cpp:513`（`attitude.timestamp = hrt_absolute_time()`），
      桥接侧是 `verify_offboard_smoke.py:132-134`（"timestamp 在 DDS 边界上被加过
      `time_offset` ⟹ 到我们手上就是 epoch µs"）。差别不在话题里，在**两个出口**上。

    ⚠️ 也不能用 `Recorder.now()`：它是**相对**的（`time.time() − t0`，见
      `verify_offboard_smoke.py:126-127`），而帧的 `wall_ns` 是绝对纪元 ns。
      ⟹ 用采样时另记的 `wall_epoch`（`time.time()`）与 `wall_ns` 配对 ——
      同一台机器上的同一个系统时钟，`tol_s` 就是"两个记录时刻差多少"。
    """
    if diag is not None:
        diag.update({"n_samples": len(samples or ()), "n_frames": len(frames or ()),
                     "frames_no_stamp": 0, "no_wall_epoch": 0, "too_far": 0, "n_out": 0})
    if not samples or not frames:
        return []
    idx = [k for k, f in enumerate(frames) if isinstance(f.get("wall_ns"), (int, float))]
    if diag is not None:
        # ★ 这一格是"帧那边没有绝对墙钟"——它与"样本那边没有配对键"是**两种不同的仪器故障**，
        #   而且修法不同（前者看 DUT 帧日志，后者看采样器），所以分开计数、不合并成"配不上"。
        diag["frames_no_stamp"] = len(frames) - len(idx)
    if not idx:
        return []
    out = []
    for s in samples:
        if not isinstance(s.get("wall_epoch"), (int, float)):
            if diag is not None:
                diag["no_wall_epoch"] += 1
            continue
        j = min(idx, key=lambda k: abs(frames[k]["wall_ns"] / 1e9 - s["wall_epoch"]))
        if abs(frames[j]["wall_ns"] / 1e9 - s["wall_epoch"]) > tol_s:
            if diag is not None:
                diag["too_far"] += 1
            continue
        d = {"t": s["t"], "frame": frames[j]}
        d.update({k: s[k] for k in keys})
        out.append(d)
    if diag is not None:
        diag["n_out"] = len(out)
    return out


def solve_stats(frames) -> dict:
    """D6 那个数字：求解耗时的 **p50 / p99 / 最坏**，以及**逐拍的间隔**（实际频率）。

    ★ 分位数与最坏**都要**（`mpc_node.solve_ms_desc()` 里记了理由）：p50 会把
      30 ms 的尾巴整个藏掉，而票面写的判据是"实测**最坏**求解耗时"。
    ★ 逐拍间隔（`wall_ns` 的差）是另一件事：它是"这台机器在**这个负载**下真跑出多少 Hz"，
      求解耗时只是它的下界。两者一起看才知道降档该降几档。

    ★★ **冷启动要单独记账**（2026-09-28 实测定下来的口径，见
      `docs/control/tier-a-sitl-hover.md`）：第一场有效飞行里全集最坏 **16.123 ms**、而它
      就是**第 0 拍**（`fallback == "warmup"`）；第 1 拍 5.238 ms；再往后 3741 拍**没有一拍
      > 5 ms**（p99 3.14、稳态最坏 3.55）。

      那是 acados 求解器第一次进内存的一次性开销（代码生成产物 + 动态库分页），**不是**
      持续负载 —— 拿它把整场频率腰斩，等于让 Phase 3 白让出一半裕度。所以：

      · `ladder`      = 按票面**字面**（全集最坏）算的档位，照样给出来；
      · `ladder_steady` = 去掉 `warmup` 那几拍之后的档位 —— **D6 定在它上面**；
      · `cold_in_warmup` = 第 0 拍是不是落在 `warmup` 闸门里。
        ⚠️ 这一条**必须为真**，否则那 16 ms 会落在**正在发指令**的那一拍上：
          `warmup_cycles=0` 时 `ladder_steady` 会把冷启动偷偷排掉，那是自欺。
    """
    solve = [float(f["solve_ms"]) for f in frames if isinstance(f.get("solve_ms"), (int, float))]
    out = {"n": len(solve)}
    if not solve:
        return out
    arr = sorted(solve)
    out["p50"] = round(arr[len(arr) // 2], 3)
    out["p99"] = round(arr[min(len(arr) - 1, int(0.99 * len(arr)))], 3)
    out["max"] = round(arr[-1], 3)
    out["min"] = round(arr[0], 3)
    out["over_10"] = sum(1 for x in arr if x > 10.0)
    out["over_20"] = sum(1 for x in arr if x > 20.0)
    for limit, verdict in D6_LADDER:
        if out["max"] <= limit:
            out["ladder"] = verdict
            break
    # —— 冷启动 / 稳态的分账 ——
    warm = [f for f in frames if f.get("fallback") == "warmup"]
    steady = sorted(float(f["solve_ms"]) for f in frames
                    if isinstance(f.get("solve_ms"), (int, float))
                    and f.get("fallback") != "warmup")
    out["warmup_n"] = len(warm)
    out["cold_ms"] = round(float(frames[0]["solve_ms"]), 3) \
        if isinstance(frames[0].get("solve_ms"), (int, float)) else None
    out["cold_in_warmup"] = (frames[0].get("fallback") == "warmup")
    out["cold_fallback"] = frames[0].get("fallback")  # 给判据⑦报出"第 0 拍处在哪个档"
    if steady:
        out["steady_n"] = len(steady)
        out["steady_p50"] = round(steady[len(steady) // 2], 3)
        out["steady_p99"] = round(steady[min(len(steady) - 1, int(0.99 * len(steady)))], 3)
        out["steady_max"] = round(steady[-1], 3)
        for limit, verdict in D6_LADDER:
            if out["steady_max"] <= limit:
                out["ladder_steady"] = verdict
                break
    ticks = [f["wall_ns"] for f in frames]
    dts = [round((ticks[i + 1] - ticks[i]) / 1e6, 3) for i in range(len(ticks) - 1)]
    if dts:
        d = sorted(dts)
        out["tick_p50_ms"] = d[len(d) // 2]
        out["tick_p99_ms"] = d[min(len(d) - 1, int(0.99 * len(d)))]
        out["tick_max_ms"] = d[-1]
    return out


# ------------------------------------------------------------------ 判据（纯函数）


def segments(truth, liftoff_cross=-0.15, hover_s=3.0, ground_s=1.0, rising=False):
    """把真值轨迹切成 `ground / climb / hover` 三段（**只用真值自己的时钟**）。

    ★ `rising`：判"起飞"的方向。A 档真值是 **NED（z 向下为正）** ⟹ 起飞 = 第一帧
      `z < liftoff_cross`（默认，A 档行为一字不改）。B 档（`verify_tier_b_seam3`）真值
      直接取自 DDS 真值帧、是 **ENU（z 向上为正）** ⟹ 起飞 = 第一帧 `z > liftoff_cross`，
      得传 `rising=True`。
      ⚠️ 不传就是**静默取反**：ENU 下 `z < 0.15` 取到的是**第一帧地面**，
      `liftoff_t` 记成 0、`climb` 恒等于"开场头 5 s" —— 而 B 档真值从回放启动就开始录、
      起飞在 OFFBOARD 之后好几秒，于是**起飞晚于 5 s 就会判出一个假的 ❌**
      （报告会写成"真值高度没升上去"，指向控制器）。这正是 `test_tier_b_judges.py`
      里 `test_起飞晚于五秒也认得出来` 钉住的那条。

    ★ "起飞"是从**真值**认出来的（第一帧 z < `liftoff_cross`），不是从另一个时钟换算的：
      两个时钟之间隔着 `time_offset`（ticket 08 实测 ≈0.18 s），换算一次就够把
      "起飞前那 0.2 s"分错，而"地面静息"恰恰是要拿来当零点的。
    ★ 分段的边界一律用**秒**（不是"取最后 N 帧"）：真值流的频率是本工具量出来的
      （`--stream-hz` 只是估长度用），按帧数切片会把 40 Hz 与 50 Hz 的运行切成不同的段。
    ★ 残缺行（任一字段是 `None`）在这里再滤一遍：源头（`TruthSampler.track`）已经滤过，
      但**旧 obs.json 的重放**也要能跑 —— 判据函数崩掉一次，就要再飞一场才能把报告拿回来。
    """
    seg = {"ground": [], "climb": [], "hover": [], "liftoff_t": None}
    truth = [r for r in (truth or []) if all(v is not None for v in r)]
    if not truth:
        return seg
    if rising:
        t_first_climb = next((r[0] for r in truth if r[3] > liftoff_cross), None)   # ENU
    else:
        t_first_climb = next((r[0] for r in truth if r[3] < liftoff_cross), None)   # NED
    seg["liftoff_t"] = t_first_climb
    if t_first_climb is not None:
        seg["ground"] = [r for r in truth
                         if t_first_climb - 0.05 - ground_s <= r[0] < t_first_climb - 0.05]
        seg["climb"] = [r for r in truth if t_first_climb <= r[0] < t_first_climb + 5.0]
    else:
        seg["ground"] = [r for r in truth if r[0] >= truth[-1][0] - ground_s]
    t_end = truth[-1][0]
    seg["hover"] = [r for r in truth if r[0] >= t_end - hover_s]
    return seg


def z_stats(rows, idx=3):
    vals = [r[idx] for r in rows]
    return {"n": len(vals), "mean": statistics.fmean(vals) if vals else None,
            "min": min(vals) if vals else None, "max": max(vals) if vals else None,
            "p2p": (max(vals) - min(vals)) if vals else None}


def crossings_series(vals, span_s):
    """一串**等间隔**样本穿过自己均值的频率（Hz）——"无持续振荡"的量化形式。

    ★ 分辨力上限 = 采样率/2（奈奎斯特）：比采样间隔还快的抖，这台仪器看不见 ——
      报告里要**把这句话说出来**，不能把它读成"没有抖"。
    """
    if len(vals) < 8 or not span_s or span_s <= 0:
        return None
    mean = statistics.fmean(vals)
    sign = [1 if x > mean else -1 for x in vals]
    n = sum(1 for i in range(1, len(sign)) if sign[i] != sign[i - 1])
    return n / 2.0 / span_s


def crossings(rows, idx=3):
    """真值流末段里曲线穿过自己均值的频率 —— `crossings_series` 的"按段"包装。

    ★ 时长取**这段自己的时间跨度**（`rows` 的首末时刻），不是请求的窗口：真值流的
      末尾可能没采满（`-n` 用完、或收流早了一点），拿请求值去除会低估频率。
    """
    vals = [r[idx] for r in rows]
    if len(vals) < 8:
        return None
    return crossings_series(vals, rows[-1][0] - rows[0][0])


def evaluate(obs) -> list:
    """把观测翻成判据。**仪器与判据分开**：JSON 里存观测，改判据不必重跑 SITL。"""
    c = []

    def ck(name, ok, detail, counts=True):
        c.append({"name": name, "ok": bool(ok) if ok is not None else None,
                  "detail": detail, "counts": counts})

    dut = obs.get("dut") or {}
    tr = dut.get("track") or {}     # ⚠️ 锚定那条判据（排在①之后）就要用它，所以在这里取
    solve = obs.get("solve") or {}
    truth = obs.get("truth") or []
    seg = segments(truth, hover_s=obs.get("hover_s", 3.0))
    hover = z_stats(seg["hover"])
    ground = z_stats(seg["ground"])
    hover_xy = [r[1:3] for r in seg["hover"]]
    climb = seg["climb"]

    # ⓪ 仪器自检 —— ★ 排在所有判据**之前**：这一条不过的话，后面每一行读数都是在
    #    一台被自己的输出卡住的被控对象上量的，整场不作数（第一版就没这条，
    #    于是"真值流 3444 Hz"这种一眼假的读数被当成了正常输出）。
    span, wall = obs.get("truth_span_s"), obs.get("stream_wall_s")
    hz, want = obs.get("truth_hz"), obs.get("truth_hz_want")
    ck("⓪ 仪器自检：真值采样没把被控对象卡住（被控对象时间跨度 ≈ 墙钟跨度）",
       span is not None and wall is not None and wall > 0 and span >= 0.8 * wall,
       f"被控对象时间 {v.fmt(span, '.1f')}s / 墙钟 {v.fmt(wall, '.1f')}s"
       f"（{v.fmt(None if (span is None or not wall) else 100 * span / wall, '.1f')}%）；"
       f"采样 {obs.get('truth_n')} 点 ⟹ {v.fmt(hz, '.2f')} Hz（目标 {v.fmt(want, '.1f')} Hz）、"
       f"命令 {obs.get('truth_meta', {}).get('sent')} 条 / 白等 {obs.get('truth_missed')} 次。"
       f"★ 不过 ⟹ **本场不作数**（不是控制器的锅）：控制台输出把 PX4 的主循环（含 SIH 步进）"
       f"堵住，闭环就在一台时间流速不对的被控对象上跑 —— 见模块 docstring ★★★")

    # ① 中间件层：节点活着、PX4 认了我们、进了 OFFBOARD
    ck("① 中间件：DUT 发得出帧、PX4 解锁并进了 OFFBOARD",
       dut.get("frames", 0) > 0 and obs.get("armed_ever")
       and obs.get("t", {}).get("offboard_enter") is not None,
       f"DUT {dut.get('frames', 0)} 帧（{v.fmt(dut.get('actual_hz'), '.1f')} Hz）、"
       f"armed={obs.get('armed_ever')}、OFFBOARD 于 t="
       f"{v.fmt(obs.get('t', {}).get('offboard_enter'), '.2f')}s；{obs.get('agent')}")
    if not dut.get("frames"):
        ck("☠ DUT 一帧都没发 —— 后面所有判据都无从判（先看 stdout 尾巴）", False,
           (obs.get("stdout_tail") or "")[-400:])
        return c

    # ★ 参考锚定（ticket 09 首飞栽在这一条上，所以它排在"行为/高度"之前）
    #   它是**控制器输入**的一件事，不是控制器的事：参考若每拍 = 测量 + 偏移，位置误差
    #   恒不收敛，后面所有高度/振荡读数都会是一条"追不上的爬升"，看着却像模型不行。
    ref_span = tr.get("ref_span_z")
    ck(f"{CK_ANCHOR}：目标全程没跟着测量走（`x_ref0` 钉在锚点上）",
       ref_span is not None and ref_span < 0.3
       and bool(dut.get("ref_events")) and "anchor" in (dut.get("ref_events") or []),
       f"`x_ref0[2]` 全程跨度 {v.fmt(ref_span, '.4f')} m（该接近 0：锚点 + 偏移是个常量）、"
       f"水平跨度 {v.fmt(tr.get('ref_span_xy'), '.4f')} m；锚点事件 {dut.get('ref_events')}；"
       f"参考误差（`x0[2] − x_ref0[2]`）头 {v.fmt(tr.get('referr_head'), '+.3f')} m ⟹ "
       f"末段 {v.fmt(tr.get('referr_tail'), '+.3f')} m。"
       f"★★ 若这一条不过（跨度 ≈ 飞机自己飞过的距离、误差一行不变）⟹ 归因「**参考取法**」："
       f"节点在追一个每拍都跟着自己移动的目标，与模型/符号映射都无关")

    # ⑨ 偏航：两条**互相独立**的读数（票面"符号映射在真 PX4 上最终确认"的那一条）
    #    ★ 为什么单列：2026-09-28 那场飞机在自转（5 圈/38 s），而**节点自己的日志里
    #      "测量反着走"与"指令反着发"长得一模一样** —— 位置那几条判据全是 ✅。
    #      只有把真值姿态拉进来，才能把"换算反了"与"映射反了"分开。
    yv = yaw_verdict(obs.get("yaw_pairs") or [], rate_hint=obs.get("yaw_rate_bound"))
    obs["yaw"] = yv
    if yv.get("alias"):
        # ★★ 别名：真值采样太稀，**这一条判不出来**（不是不通过）。2026-09-28 就吃过一次：
        #   4.9 rad/s 的真值序列被最短路径展开记成一半，⑨a 给出 ❌ 并把归因写成
        #   "指令让飞机往反方向转 ⟹ 符号映射" —— 方向其实是**判据自己错了**。
        #   归因行（见 `attribute`）也会因为 ok=None（不计数）而自动不再提这一条。
        ck("⑨ 前置：真值采样密到能解出这段旋转（**别名自检**；判 ❌ = 本场**不能算通过**，"
           "但归因**不指向**控制器）", False,
           f"飞机转速**上界** {v.fmt(obs.get('yaw_rate_bound'), '.3f')} rad/s（自 100 Hz 的 "
           f"`x0`/`u[3]` 两路取较大者），而姿态真值只有 "
           f"{v.fmt(obs.get('gt_att_hz_want'), '.1f')} Hz（采样间隔 "
           f"{v.fmt(yv.get('dt_sample'), '.3f')} s）⟹ 一个间隔转过 "
           f"{v.fmt(yv.get('alias_turn'), '.2f')} rad ≥ {ALIAS_STEP_RAD:.2f} "
           f"（`unwrap` 走最短路径，超过 π 就记反/记少 —— 2026-09-28 那场就是这样把 "
           f"−53.9 rad 记成 −26.5，⑨a 于是报「指令反了」）⟹ **累计转角**这一条读不出来。"
           f"★ 这不是「飞机没转」，也不是「符号映射反了」：是**真值采样跟不上转速**。"
           f"★★ 这一条**计入**「不通过」（`counts` 默认为真）而**不是**当读数放过 —— "
           f"理由是它出现的场次**本来就不可能是一次合格悬停**（飞机转速上界已经过 "
           f"{ALIAS_STEP_RAD / (yv.get('dt_sample') or 1):.2f} rad/s），"
           f"所以本场也不该在报告里读成「全部通过」；但归因**明说**原因在真值采样率、"
           f"不在控制器。要判这一条得提高真值采样率（`--gt-att-hz`）—— 或者把飞机先停稳再判")
        ck(CK_YAW_MAP, None, "真值采样太稀（见上一条 ⚠️，别名）⟹ 这一条**没做**"
                             "（「本场不能算通过」由**上一条前置**表达，这里不重复计数）",
           counts=False)
    elif (yv.get("n") or 0) < 3:
        # ★ 三分法：**没有数据**（ok=None，不计数）与**仪器坏了**（ok=False，计数）不是一回事。
        #   旧观测/这一档根本没采 ⟹ 前者；采到了却一条没配上 ⟹ 后者（2026-09-28 的教训：
        #   漏了配对键，33 个样本一条没配上，报告把它写成了"没采到姿态真值"）。
        #   `verify_offboard_smoke` 的跨档判据用同一个三分法。
        dg = obs.get("yaw_pair_diag") or {}
        if (dg.get("n_gt") or 0) == 0 and not obs.get("gt_att"):
            for name in (CK_YAW_MAP, CK_YAW_CONV):
                ck(name, None, "本场**没有姿态真值样本**（旧观测、或 `--gt-att-hz 0`）⟹ "
                               "这一条**没做**，不当作通过也不当作不通过", counts=False)
        else:
            ck("⑨ 前置：姿态真值配得上 DUT 帧（**仪器自检**；判 ❌ = 本场**不能算通过**，"
               "但归因**不指向**控制器）", False,
               f"采到 {dg.get('n_gt')} 点 `vehicle_attitude_groundtruth`、配出 "
               f"{dg.get('n_out')} 对 ⟹ 判据⑨ 的原料没接上。分项：帧无绝对墙钟 "
               f"{dg.get('frames_no_stamp')} 条、样本无配对键 `wall_epoch` "
               f"{dg.get('no_wall_epoch')} 点、时间差超 {0.25}s 容差 {dg.get('too_far')} 点、"
               f"帧里缺 `u` {dg.get('u_missing')} 对、缺 `x0` {dg.get('x0_missing')} 对。"
               f"★ 这不是「飞机没转」，是**我们的两条链没对上**（见 `pair_by_time` 的 ⚠️："
               f"`hrt` 与 epoch 差 56 年、`Recorder.now()` 是相对的）。"
               f"★★ 与上面那条别名自检同一口径：**计入**「不通过」（仪器坏了 ⟹ 这一场"
               f"本来就说不出「全部通过」），但归因明说原因在仪器")
            for name in (CK_YAW_MAP, CK_YAW_CONV):
                ck(name, None, "仪器没接上（见上一条 ⚠️）⟹ 这一条**没做**"
                               "（「本场不能算通过」由**上一条前置**表达）", counts=False)
    else:
        ck(f"{CK_YAW_MAP}（真值锚点：`dψ_ned/dt == −ω_z_FLU`，**不经过我们的换算**）",
           abs(yv["resid"]) < 0.25 * abs(yv.get("cmd_span") or 0.0) + 0.35,
           f"配对 {yv.get('n')} 点（`vehicle_attitude_groundtruth` @ "
           f"{v.fmt(obs.get('gt_att_hz_want'), '.1f')} Hz）："
           f"真值机头累计转角 {v.fmt(yv.get('psi_span'), '+.3f')} rad、指令累计转角 `∫u[3]dt` "
           f"{v.fmt(yv.get('cmd_span'), '+.3f')} rad ⟹ 残差 `ψ̇ + ω_z` 的积分 "
           f"{v.fmt(yv.get('resid'), '+.3f')} rad（该 ≈ 0）。"
           + (f"★ **判别力够**（`|∫u[3]dt|` = {v.fmt(yv.get('cmd_span'), '+.3f')} rad > "
              f"{YAW_POWER_RAD}）⟹ 这一场**真的判了方向**：指令反了的话这里会是 "
              f"`|ψ_span − ∫u[3]dt|`，判 ❌。"
              if yaw_had_power(yv) else
              f"⚠️ **判别力不足**（`|∫u[3]dt|` = {v.fmt(yv.get('cmd_span'), '+.3f')} rad ≤ "
              f"{YAW_POWER_RAD}）⟹ 这一场的 `✅` **只说明没矛盾，不能证明方向**"
              f"（悬停得好 ⟹ 指令累计转角本来就小，反过来也过）。"
              f"**方向证据要引真在转的场次**（第 5 场：`∫u[3]dt` = +7.94 rad、真值 −7.73 rad，"
              f"反了会给出 −15.7），或者 seam 1 的话题层用例")
           + ("" if yv.get("alias") is not None else
              f"⚠️ 本场**没有别名自检的原料**（obs 里缺 `yaw_rate_bound` —— 闸门是 2026-09-28 "
              f"之后才加的，**旧观测**重放时都没有它）⟹ 飞机若转得快，这一条可能是个**假 ❌**"
              f"（真值 1 Hz 记不下 > π 的步长）；那种情况**以重飞为准**。")
           + f"★ 若这一条不过：**指令把飞机往反方向转** ⟹ 归因「符号映射」（`sign-map.md` §5.2 的 yaw 取反）")
        ck(f"{CK_YAW_CONV}（`π/2 − ψ_ned` == 节点换算出来的 ENU 机头朝向）",
           abs(yv["conv_err_med"]) < 0.15 and (yv.get("conv_err_span") or 9.9) < 0.30,
           f"换算误差中位数 {v.fmt(yv.get('conv_err_med'), '+.3f')} rad、全程跨度 "
           f"{v.fmt(yv.get('conv_err_span'), '.3f')} rad（该 ≈ 0 且不随航向变）。"
           f"★ 若这一条不过（误差 ≈ 2ψ_ned 或随航向线性变）⟹ 归因「符号映射」的**换算那半边**："
           f"`px4_quat_to_enu` 的 yaw 分量反了 —— 那时位置环仍可能看着正常")

    # ② 行为符合预期（不是"没崩"）：真值高度**升上去**了
    ck("② 行为：真值高度真的升上去了（起飞后 5 s 内 z < −0.4 m）",
       seg["liftoff_t"] is not None and bool(climb)
       and any(r[3] < -0.4 for r in climb),
       f"真值第一点 z={v.fmt(truth[0][3], '+.4f')} m（地面）⟹ 第一次越过 −0.15 m 于 "
       f"t={v.fmt(seg['liftoff_t'], '.2f')}s；起飞段最低 "
       f"{v.fmt(min((r[3] for r in climb), default=None), '+.3f')} m。"
       f"★ 若这一条不过（高度不动或往下扎）⟹ 归因「符号映射/推力方向」，与模型无关")

    # ③ 悬停高度：1 m（NED 里是 −1）—— ★ 判在**状态源那一帧**上（理由见下）
    climb_ekf = tr.get("climb")
    ck("③ 悬停高度 1 m ± 0.15（★ **EKF 帧**：节点日志里 `x0[2]` 的抬升量，末段 3 s 均值）",
       climb_ekf is not None and 0.85 < climb_ekf < 1.15,
       f"节点侧：起飞前 {v.fmt(tr.get('up_head'), '+.3f')} m ⟹ 末段 {v.fmt(tr.get('up_tail'), '+.3f')} m"
       f"（抬升 {v.fmt(climb_ekf, '+.3f')} m，末段 up 速度 {v.fmt(tr.get('vup_tail'), '+.3f')} m/s）、"
       f"末段区间 [{v.fmt(tr.get('up_tail_min'), '+.3f')}, {v.fmt(tr.get('up_tail_max'), '+.3f')}] m、{tr.get('n')} 帧。"
       f"★ 为什么判在这一帧：票面的状态源就是 EKF 这两条话题，**「1 m」只有在那一帧里才有定义**"
       f"（节点没法知道自己相对真值差多少）。真值帧的读数见下一条")
    ck("③（真值帧，ℹ️ 不计数）：真值抬升了多少 —— 与 EKF 帧的差就是**被控对象自己**的误差", None,
       f"真值：地面段均值 {v.fmt(ground['mean'], '+.4f')} m ⟹ 末段 {v.fmt(hover['mean'], '+.4f')} m"
       f"（抬升 {v.fmt(None if (hover['mean'] is None or ground['mean'] is None) else ground['mean'] - hover['mean'], '.3f')} m）；"
       f"末段 {hover['n']} 帧区间 [{v.fmt(hover['min'], '+.3f')}, {v.fmt(hover['max'], '+.3f')}] m。"
       f"体检里地面静息的 EKF−真值 高度差见 `--plant-check` 的 dwell 一行（0.4 m 量级、"
       f"噪声带 ~0.15 m、|最大| ~0.7 m）⟹ 两个帧的读数差里，**主要部分是这台 SITL 的估计器误差**，"
       f"不是控制器的误差", counts=False)

    # ④ 无持续振荡 / 无漂移（★ 也判在 EKF 帧：节点日志 100 Hz，分辨力比真值流高一倍）
    rate = obs.get("truth_hz") or 0.0
    osc = tr.get("up_tail_osc_hz")
    ck("④ 无持续振荡（★ EKF 帧，末段 3 s）：高度峰峰值 < 0.35 m 且穿均值频率 < 2 Hz",
       tr.get("up_tail_p2p") is not None and tr["up_tail_p2p"] < 0.35
       and osc is not None and osc < 2.0,
       f"节点侧末段 {v.fmt(tr.get('tail_s'), '.1f')}s：峰峰值 {v.fmt(tr.get('up_tail_p2p'), '.4f')} m、"
       f"标准差 {v.fmt(tr.get('up_tail_std'), '.4f')} m、穿均值 {v.fmt(osc, '.2f')} Hz。"
       f"★ 这一路是 **100 Hz** 的逐拍日志 ⟹ 判得了 50 Hz 以内的抖（真值采样只有 "
       f"{v.fmt(rate, '.1f')} Hz，见下一条）")
    osc_gt = crossings(seg["hover"])
    ck("④（真值帧，ℹ️ 不计数）：真值那一路的峰峰值与穿均值频率", None,
       f"末段峰峰值 {v.fmt(hover['p2p'], '.4f')} m、穿均值 {v.fmt(osc_gt, '.2f')} Hz"
       f"（真值采样 {v.fmt(rate, '.2f')} Hz / {hover['n']} 点 ⟹ 只判得了 2 Hz 以上的抖，"
       f"★ 这是这台仪器的分辨力上限；而且它把估计器自身的漂移也算进峰峰值里了）",
       counts=False)
    drift = (math.hypot(hover_xy[-1][0] - hover_xy[0][0], hover_xy[-1][1] - hover_xy[0][1])
             if len(hover_xy) > 1 else None)
    ck("④ 无持续漂移：末段 3 s 水平位移 < 0.5 m（EKF 帧与真值帧各报一个）",
       tr.get("xy_drift_tail") is not None and tr["xy_drift_tail"] < 0.5
       and drift is not None and drift < 0.5,
       f"EKF 帧水平净位移 {v.fmt(tr.get('xy_drift_tail'), '.3f')} m；"
       f"真值帧 {v.fmt(drift, '.3f')} m（x 从 {v.fmt((hover_xy[0][0] if hover_xy else None), '+.3f')} "
       f"到 {v.fmt((hover_xy[-1][0] if hover_xy else None), '+.3f')} m）")

    # ⑤ 推力标定：**在真 PX4 上**确认（票面验收项）
    #   ⚠️ 只在**配平**（末段垂直速度≈0）时判 —— 第一版没这道闸，于是"飞机以 0.9 m/s
    #      爬升"那一场里电机均值 0.6978 与悬停 0.6233 差 0.0745 < 0.08 也判了 ✅：
    #      判据太宽 + 场景根本不在悬停 ⟹ **一场爬升被判成"推力标定通过"**。
    #      推力标定只在"停得住"这件事上才有意义（否则量到的是爬升所需推力）。
    m_mean = obs.get("motors_mean_end")
    u_mean = dut.get("u_mean_normal")
    vup_tail, in_trim = tr.get("vup_tail"), None
    in_trim = vup_tail is not None and abs(vup_tail) < 0.1
    ck("⑤ 推力标定：PX4 解出的电机均值 ≈ m·g/(4·T_max)（★ 只在末段配平时才算数）",
       None if not in_trim else (m_mean is not None and HOVER_NORM is not None
                                 and abs(m_mean - HOVER_NORM) < 0.05),
       f"末段电机均值 {v.fmt(m_mean, '.4f')} ⟹ 总推力 {v.fmt(obs.get('thrust_n_end'), '.3f')} N；"
       f"z0 机型真值 m·g={v.fmt(HOVER_N, '.3f')} N、归一化悬停 = {v.fmt(HOVER_NORM, '.4f')}"
       f"（= m·g/(4·`SIH_T_MAX`)）。★ 电机是 **PX4 解出来**的量，不是我们的请求。"
       + ("" if in_trim else
          f" ⚠️ **末段不在配平**（up 速度 {v.fmt(vup_tail, '+.3f')} m/s ⟹ 判据不成立，"
          f"此读数只作背景）：飞机没停在高度上时，电机读数反映的是「爬/降需要多少推力」，"
          f"拿它当悬停标定的证据会把一场爬升读成「标定通过」"),
       counts=in_trim)
    ck("⑤ 归一化常数一致：节点自己的 `HOVER_THRUST` == 机型的 m·g/(4·T_max)",
       HOVER_NORM is not None and abs(HOVER_THRUST - HOVER_NORM) < 0.01,
       f"节点参数 `HOVER_THRUST`={HOVER_THRUST:.4f}、机型推出 {v.fmt(HOVER_NORM, '.4f')}"
       f"（两个数来自**各自独立的**来源：`nmpc/params.py` 与 z0 机型文件）")
    u0 = (u_mean or [None])[0]
    ck("⑤ 悬停段的推力指令 ≈ 悬停所需（|u[0]| 与 m·g 差 < 25%）",
       u0 is not None and HOVER_N is not None and abs(abs(u0) - HOVER_N) < 0.25 * HOVER_N,
       f"正常帧 `u[0]`（FLU 总推力，**牛顿**）均值 {v.fmt(u0, '+.3f')} N，悬停需 "
       f"{v.fmt(HOVER_N, '.3f')} N ⟹ 归一化 {v.fmt(None if (u0 is None or T_MAX_TOTAL is None) else u0 / T_MAX_TOTAL, '+.4f')}"
       f"（分母是总推力上限 {v.fmt(T_MAX_TOTAL, '.2f')} N）；线上那一列 `|thrust_body_z|`"
       f"**末段**中位数 = {v.fmt(dut.get('thrust_body_z_tail'), '.4f')}，悬停该是 "
       f"{v.fmt(HOVER_NORM, '.4f')}（整场的取值集合 {dut.get('thrust_body_z_seen')} —— "
       f"起飞那一拍本来就该贴 −1，别拿它当悬停读数）")

    # ⑥ 符号映射：lateral 指令没有长期打满（打满 ⟹ 追不上参考 ⟹ 状态源/模型的问题）
    u_mean_list = u_mean or [None, None, None, None]
    ck("⑥ 符号映射：末段滚转/俯仰指令没有长贴上限（|u[1]|,|u[2]| < 上限的 40%）",
       all(x is not None and abs(x) < 0.4 * lim for x, lim in
           ((u_mean_list[1], ROLL_RATE_MAX), (u_mean_list[2], PITCH_RATE_MAX))),
       f"正常帧指令均值 roll={v.fmt(u_mean_list[1], '+.4f')} pitch={v.fmt(u_mean_list[2], '+.4f')} "
       f"yaw={v.fmt(u_mean_list[3], '+.4f')} rad/s（上限 {ROLL_RATE_MAX}/{PITCH_RATE_MAX}/{YAW_RATE_MAX}）；"
       f"饱和计数 {dut.get('sat_count') or '无'}。★ 长期贴上限说明参考追不上 —— 那是**输入**的问题，"
       f"不是符号映射的问题（方向错会表现为反着飞，②会先不过）")

    # ⑥b 悬停段的机头 —— ★ 它不在票面的三条归因里，但"稳定悬停、无持续振荡"有一半是它。
    #    第一手：2026-09-28 那几场飞机在自转（累计 31 rad ≈ 5 圈/38 s），而**其余判据全绿**
    #    —— 位置参考是锚死的，机头自转在位置上只表现为小幅漂移 ⟹ 那些 ✅ 盖不住这个失效。
    #    判在 `x0[6:10]`（= PX4 `vehicle_attitude` 经我们的换算，即**EKF 帧**），
    #    与操作者裁定的"验收判在 EKF 帧上"一致；真值姿态只给判据⑨ 做归因。
    ck("⑥b 悬停段机头保持（EKF 帧）：末段机头转速 ≈ 0 且全程没转圈",
       tr.get("yaw_ekf_tail_rate") is not None
       and abs(tr["yaw_ekf_tail_rate"]) < 0.1
       and abs(tr.get("yaw_ekf_span") or 9.9) < 0.5,
       f"末段（{v.fmt(tr.get('tail_s'), '.2f')}s）机头转速 "
       f"{v.fmt(tr.get('yaw_ekf_tail_rate'), '+.4f')} rad/s、该段转角 "
       f"{v.fmt(tr.get('yaw_ekf_tail_span'), '+.3f')} rad；**全程累计转角** "
       f"{v.fmt(tr.get('yaw_ekf_span'), '+.3f')} rad；末段 `u[3]` 均值 "
       f"{v.fmt(tr.get('u3_tail'), '+.4f')} rad/s。★「机头在转而位置那几条全绿」是"
       f"**位置参考锚死**的副作用 ⟹ 别拿位置 ✅ 当成机头正常；归因见判据⑨")

    # ⑩ 机头回正（**ticket 16 验收第 3 项**）—— ★ 判**指令**，不判机头走到哪。
    #   落地权重下 τ ≈ 2.05·R_yaw ≈ 1025 s ⟹ 一场 25 s 里机头只走 ~0.5°，而 EKF 机头的噪声
    #   就是这个量级 ⟹ "机头回到位"这个量在短场里**不可观测**；可观测的是 `u[3]`。
    #   ★ 符号规则不是推的（推理有踩坑记录）：离线钉住状态、反复解到不动点实测（2026-09-30）——
    #     实测机头 `+23°`（Δψ = −0.4014 rad）⟹ `u[3] = −3.8319e-4`；`−23°` ⟹ `+3.8319e-4`，
    #     **反号等幅** ⟹ `u[3]·Δψ > 0` 即回正，且 τ = 1047.6 s（与 `2.05·R_yaw` 差 2%）。
    #   ⚠️ 本场没注入机头误差时这条**没做**（`ok=None`、不计数）—— 它不是通过，
    #     投它票的是"这一场用 `--hold-yaw` 飞"这件事本身（验收正文就这么要求）。
    #   ⚠️ 也别拿上面 ⑥b 的 ✅ 当替代：它的门槛 `0.1 rad/s` 比这条要看的 `3.4e-4` 高
    #     **三个数量级** ⟹ 偏航权威再小它也是绿的（这就是 ticket 25 的"死通道"）。
    r_err, r_u3 = tr.get("yaw_restore_err"), tr.get("yaw_restore_u3")
    if r_err is None or r_u3 is None:
        # ★ 两种成因**分开说**（复核意见）：配不上对（分母 0）与"配上了但误差全在噪声里"
        #   是两条不同的排查路，混成一句"没注入误差"会把人往错的方向引。
        n_all = tr.get("yaw_restore_n_all") or 0
        why = (f"末段一对都没配成（分母 = {n_all}）⟹ 先查逐帧日志里有没有 `x_ref0_q` 这一列"
               f"（老节点的日志没有它）"
               if n_all == 0 else
               f"末段配到 {v.fmt(n_all, 'd')} 帧（参考机头 `x_ref0_q` 与实测机头 `x0[6:10]` "
               f"同一帧配对），但其中 `|Δψ| ≥ {math.degrees(YAW_RESTORE_MIN_RAD):.0f}°` 的"
               f"有 **0 帧** ⟹ 参考机头与实测机头分不开（`--hold-yaw` 没传；"
               f"⚠️ 传了但没关 `keep_yaw` 也是如此，见 `dut_params()` 的 ★★★）")
        ck("⑩ 偏航：有已知机头误差时指令把它往参考拉（**没做**：本场没有机头误差）",
           None,
           why + "。★ 这**不是通过**：验收它得用 `--hold-yaw 20` 重飞一场"
                 "（见 `dut_params()` 的 ★★）",
           counts=False)
    else:
        want = r_err / YAW_TAU_S
        lo, hi = YAW_RESTORE_BAND[0] * abs(want), YAW_RESTORE_BAND[1] * abs(want)
        same_sign = (r_u3 * r_err) > 0
        in_band = lo <= abs(r_u3) <= hi
        ck(CK_YAW_RESTORE, same_sign and in_band,
           f"末段 {v.fmt(tr.get('yaw_restore_n'), 'd')} 帧里 Δψ = 参考机头 − 实测机头 "
           f"= {v.fmt(r_err, '+.4f')} rad（{math.degrees(r_err):+.1f}°，中位数），"
           f"同段 `u[3]` = {v.fmt(r_u3, '+.8f')} rad/s（中位数）⟹ "
           f"{'**同号**（回正）' if same_sign else '☠ **反号**（把机头往远处推）'}；"
           f"期望 `Δψ/τ` = {v.fmt(want, '+.2e')} rad/s（`τ = 2.05·R_yaw = {YAW_TAU_S:.0f} s`，"
           f"`R_yaw = {R_YAW:g}`），实测/期望 = "
           f"{v.fmt((abs(r_u3)/abs(want)) if want else None, '.2f')}×"
           f"（判带 {YAW_RESTORE_BAND[0]:g}…{YAW_RESTORE_BAND[1]:g}×）"
           f"{'' if in_band else ' ☠ 量级不在带里'}。"
           f"★ 只有**符号**是锐的（反了必红）；幅值那一带是「别差一个数量级」的信封 —— "
           f"真 SITL 多了姿态环滞后，实测增益只会比离线预测小。"
           f"⚠️ 这一条**判不了**「机头多久回到位」（那是 τ ≈ 17 min 的事，短场里不可观测），"
           f"它判的是「指令在不在往参考拉、拉得对不对」；"
           f"偏航权威本身够不够是 ticket 25 的事")

    # ⑦ D6：票面那个数字（结论行，不判"通过"）
    ck("⑦ D6（结论行，不是通过/不通过）：求解耗时 p50/p99/最坏 —— 冷启动与稳态分账",
       None,
       f"n={solve.get('n')} ⟹ p50 {v.fmt(solve.get('p50'), '.3f')} ms / "
       f"p99 {v.fmt(solve.get('p99'), '.3f')} ms / **全集最坏 {v.fmt(solve.get('max'), '.3f')} ms**"
       f"（查最坏 10 ms 的 {solve.get('over_10')} 拍、超 20 ms 的 {solve.get('over_20')} 拍）；"
       f"去掉 {solve.get('warmup_n')} 拍 `warmup` 之后（n={solve.get('steady_n')}）："
       f"p50 {v.fmt(solve.get('steady_p50'), '.3f')} / p99 {v.fmt(solve.get('steady_p99'), '.3f')}"
       f" / **稳态最坏 {v.fmt(solve.get('steady_max'), '.3f')} ms**。"
       f"按票面字面（全集）⟹ {solve.get('ladder') or '（没有样本）'}；"
       f"按稳态 ⟹ {solve.get('ladder_steady') or '（没有样本）'}。"
       f"★ 全集最坏那 {v.fmt(solve.get('cold_ms'), '.3f')} ms 是**第 0 拍**（acados 第一次进内存的"
       f"一次性开销，不是持续负载）；见 `solve_stats` 的 ★★ 与 `docs/control/tier-a-sitl-hover.md`",
       counts=False)
    ck("⑦ 冷启动落在 `warmup` 闸门里（否则那 {:.1f} ms 会砸在**正在发指令**的一拍上）"
       .format(solve.get("cold_ms") or 0.0),
       bool(solve.get("cold_in_warmup")),
       f"第 0 拍 fallback={solve.get('cold_fallback')!r}、`solve_ms`={solve.get('cold_ms')}；"
       f"`warmup_cycles` 那几拍（本场 {solve.get('warmup_n')} 拍）不发指令 ⟹ "
       f"冷启动的尾巴落在闸门里。★ 稳态档位（D6 定在它上面）的前提就是这一条")
    ck("⑦ 这台机器真跑出多少 Hz（逐拍间隔）—— D6 的另一个读数",
       solve.get("tick_p50_ms") is not None and solve["tick_p50_ms"] < 12.0,
       f"逐拍间隔 p50 {v.fmt(solve.get('tick_p50_ms'), '.3f')} ms / "
       f"p99 {v.fmt(solve.get('tick_p99_ms'), '.3f')} ms / 最坏 {v.fmt(solve.get('tick_max_ms'), '.3f')} ms "
       f"⟹ 请求 {obs.get('rate_hz')} Hz、实发 {v.fmt(dut.get('actual_hz'), '.1f')} Hz")

    # ⑧ 复盘面：状态源/分龄/饱和标志都进了日志（票面验收项）
    ck("⑧ 复盘面：逐帧日志里有状态源、两条状态的**分龄**、饱和标志与档位",
       dut.get("state_source") == "px4" and bool(dut.get("ages_ok")) and bool(dut.get("sat_logged")),
       f"state_source={dut.get('state_source')}、分龄齐全={dut.get('ages_ok')}、"
       f"每帧都带 `sat` 列={dut.get('sat_logged')}（本场出现过的饱和标志 "
       f"{dut.get('sat_any') or '无' }，计数 {dut.get('sat_count')}）、"
       f"档位分布 {dut.get('fallback')}。★ 判的是「**能力在日志里**」而不是「这一场饱和过」"
       f"—— 没有饱和的一场恰恰是「没顶约束」的证据，判 ❌ 就是把它读反了")
    # ★★ 这一条**在 SITL 这一层判不了**，所以它是 ℹ️ 读数（`ok=None` + `counts=False`），
    #   不是判据 —— 早先那版写的是 `ck(..., True, ...)`：一个**永远为真**的常量，
    #   源码上看着像在判、其实什么也没判（2026-09-28 复核抓出来的）。
    #   真正能证伪的位置只有一个：**话题边界**（DDS 里那条订阅在不在）。
    #   `fallback` 列证不了任何事 —— 它的取值只可能是 normal/hold/hover/warmup，
    #   "没有一帧提到 mocap"是**定义上**成立的空话。
    ck("⑧ 本档**不用动捕**（票面约束）—— SITL 这一层**不做**这条，判据在话题边界", None,
       f"本场 `-p state_source:=px4`（⑧ 上一行判的就是它）、档位分布 {dut.get('fallback')}；"
       f"★ **证伪位置只有一个**：`state_source=px4` 时 `/mocap/pose_enu` 的 "
       f"`matched_subscribers()` 必须 == 0（「订阅了但不看」在话题上不可证伪，见 `mpc_node` 的 ★）"
       f"—— 那条在 **seam 1**：`test_mpc_node.py::test_A_档只订_px4_那两条_动捕一帧都不订`。"
       f"本行**不计入**通过与否（`fallback` 的取值面里本来就没有 mocap 这一项）",
       counts=False)
    return c


#: 归因表（票面："若本档失败，归因到**模型 / 符号映射 / 中间件**三者之一，不是笼统地再调调参数"）
def attribute(obs, checks) -> str:
    """从**已经量到的数**里读出归因，而不是靠印象。

    ★ 顺序刻意如此：先看"飞控那一层有没有通"（中间件），再看"**参考怎么取的**"（见下），
      再看"方向对不对"（符号映射），再看"输入追不追得上"（状态源），最后才轮到"控制器本身"。
      反过来做的话，一个中间件的毛病会一路被读成"模型不行"。
    ★ "参考取法"这一条插在中间件之后、符号映射之前 —— **两头都不能越位**：
      排在中间件之前的话，一场"节点一帧没发"会被读成参考取法（`ref_span` 也是 None）；
      排在符号映射之后的话，参考跟着测量走造成的爬升会先撞上②/⑥。
    ★★ **票面的三条盖不住第四条**（这是本票实测撞出来的，见 `docs/control/tier-a-sitl-hover.md`）：
      "参考取法"既不是模型、也不是符号映射、也不是中间件 —— 参考定义在**我们自己的代码**里
      （`hold_reference` 拿哪一帧去加 `hold_offset`），而票面那三条都是"外部东西接错了"。
      所以它单列一条，且排在②之前：目标本身就跟着测量走的话，飞机"稳不住"与符号映射无关，
      ②却会因为它一直在爬而跟着挂 —— 那条路走过一次，归因给成了"模型"。
    ★ 高度/振荡都判在 **EKF 帧**（节点只看得见那一帧）⟹ 真值帧的差**不**作为归因的触发条件：
      它是**被控对象**的读数（这台 SITL 的气压计/GPS 仿真是 EKF 的自反馈，见 `evaluate_plant`），
      所以一律附在结论后面当背景，不让它去触发"状态源"那条归因。
    """
    def failed(prefix):
        return [c for c in checks if c.get("ok") is False and c["name"].startswith(prefix)]

    dut = obs.get("dut") or {}
    tr = dut.get("track") or {}
    seg = segments(obs.get("truth") or [], hover_s=obs.get("hover_s", 3.0))
    hov, gnd = z_stats(seg["hover"]), z_stats(seg["ground"])
    gt_climb = (None if (hov["mean"] is None or gnd["mean"] is None)
                else gnd["mean"] - hov["mean"])
    tail = (f"；★ 被控对象那一侧的读数（不参与通过与否）：末段 EKF-vs-真值 高度差 "
            f"{v.fmt(_diff(obs.get('ekf_end'), obs.get('gt_end'), 'z'), '+.3f')} m、"
            f"真值抬升 {v.fmt(gt_climb, '.3f')} m —— 这台 SITL 的估计器自带 0.4 m 量级误差"
            f"（气压计/GPS 仿真喂的是 EKF 自己的输出）")

    if failed("⓪"):
        return ("**本场不作数（不是控制器的锅）**：真值采样把被控对象卡住了（仪器自检没过，"
                "见判据⓪）⟹ 先修测量再谈归因")
    if dut.get("frames", 0) == 0 or obs.get("t", {}).get("offboard_enter") is None:
        return ("**中间件**：节点没发帧 / PX4 没进 OFFBOARD ⟹ 在谈控制器之前先修这一层"
                f"（{obs.get('agent')}；stdout 尾巴见下）")
    # ★ 锚定那一条要排在②之前，**但要排在上面那条之后**：帧数不够时 `ref_span` 也是 None，
    #   判据同样会红（"没测到"≠"没跟着走"）—— 那种场面还没轮到谈参考取法。
    #   `ref_n` 这个门槛就是为了不让"帧太少"被读成"参考没锚住"（两者都会让那条判据变红）。
    if failed(CK_ANCHOR) and (tr.get("ref_n") or 0) >= 10:
        return ("**参考取法（票面三条之外）**：目标自己跟着测量走 ⟹ 位置误差被做成了恒等于 "
                "`hold_offset`、永远关不小；飞机只会沿一个方向匀速走（本票实测：**0.9 m/s "
                "爬升 37 s**，`x0 − x_ref0` 全程卡在 −0.955 m ≈ `v·τ − hold_offset`）。"
                "这不是模型/符号映射/中间件的毛病，是 `hold_reference` 拿哪一帧去加偏移的问题："
                "必须钉在**闸门第一次打开那一拍**的锚点上（`NodeCore._update_anchor`）。"
                "本场的爬升/饱和读数在修好它之前**一概不作数**")
    if failed("②"):
        return ("**符号映射 / 推力方向**：进了 OFFBOARD 但飞机没往上去 ⟹ "
                "看 `u[0]` 与线上下发的 `thrust_body_z` 的符号（`sign-map.md` §1）")
    if failed(CK_YAW_MAP):
        return ("**符号映射 / 偏航方向**：指令让飞机**往反方向**转（真值累计转角与 `∫u[3]dt` 同号）"
                "⟹ 问题是 yaw 那一路的取反（`sign-map.md` §5.2）——"
                "★ 这一条**位置判据抓不到**：位置环照样能稳，飞机只是边稳边自转")
    if failed(CK_YAW_CONV):
        return ("**符号映射 / 姿态换算**：NED/FRD → ENU/FLU 的**偏航那半边**反了"
                "（换算误差 ≈ 2ψ_ned 或随航向线性变）⟹ 看 `node_core.px4_quat_to_enu` 的"
                "`Q_NED_TO_ENU` / `Q_FRD_TO_FLU` 复合顺序；"
                "★ 位置环可能仍然正常，所以这条只能靠真值姿态抓")
    if failed(CK_YAW_RESTORE):
        # ★ 排在 ⑥b 之前：⑩ 红时 ⑥b **可能**也红（权威够大时符号反会让机头朝远处转、转角/转速
        #   超门槛），两条同时红该先说到因上。⚠️ 但**落地权重下 ⑥b 看不见它**：`|u[3]| ≈ 3.8e-4`
        #   在 3 s 里只转 `1.1e-3 rad`，`0.1 rad/s` 与 `0.5 rad` 两个门槛都过得去 ——
        #   这正是下面正文说的那个洞，别把这两句读成互相矛盾。
        return ("**偏航通道（票面三条之外；ticket 16 验收第 3 项）**：有已知机头误差时，`u[3]` "
                "**没有把机头往参考拉**（符号反），或者**幅值差了一个数量级**（权威被压到 ≈0）。"
                "★ 这一条**独立于** ⑥b：⑥b 的门槛是 `0.1 rad/s`，比这里要看的 `3.4e-4` 高"
                "**三个数量级** ⟹ 权威再小 ⑥b 也是绿的（那就是个洞，见下）。"
                "· **符号反** ⟹ 先查 `u[3]` 这一路的取反（`sign-map.md` §5.2）与 `q_diag` 的 `qw`"
                "（`qw = 0` 时代价对朝向几乎无回正力、且**分不出镜像机头**，见 `offline-ocp.md` §2）；"
                "· **幅值 ≈ 0（但符号对）** ⟹ 这是**死通道**，落地权重下 `τ ≈ 2.05 · R_yaw ≈ 17 min`、"
                "指令量级 `1e-4 rad/s`（**票 25**）—— 那不是这条判据的锅，"
                "它只说明「指令在往对的方向拉，只是很轻」")
    if failed("⑥b"):
        # ★★ 这里**不许**一口咬定"⑨ 已经证明两个方向都没反"：走到这一条时 ⑨a 可能是
        #   **没做**（别名闸门 / 仪器没接上，`ok=None`），也可能做了但**判别力不足**
        #   （`|∫u[3]dt|` 太小，见 `YAW_POWER_RAD`）—— 两种情况下方向**都没有**被证实。
        #   2026-09-28 复核抓出来的：原文在别名那一场上照样说"已证明"，把话说满了。
        yv_here = obs.get("yaw") or {}
        if yv_here.get("alias"):
            yaw_note = ("⚠️ **⑨a 本场没做**（别名闸门：真值采样跟不上转速）⟹ **方向这一条"
                        "没有被排除**，别把本场读成「符号映射已证实」；")
        elif not (yv_here.get("n") or 0) >= 3:
            yaw_note = ("⚠️ **⑨a 本场没做**（没有配得上的姿态真值）⟹ **方向这一条没有被排除**；")
        elif yaw_had_power(yv_here):
            yaw_note = ("⑨a 本场**真的判到了方向**（`|∫u[3]dt|` 超过判别力门槛，指令反了会判 ❌）"
                        "且它过了 ⟹ 不是符号映射；")
        else:
            yaw_note = (f"⚠️ ⑨a 本场 `✅` 但**判别力不足**（`|∫u[3]dt|` = "
                        f"{v.fmt(yv_here.get('cmd_span'), '+.3f')} rad ≤ {YAW_POWER_RAD}）"
                        f"⟹ 它只是「没矛盾」，**方向仍需另证**（引真在转的第 5 场）；")
        return ("**代价 / 前馈（票面三条之外）**：指令在**单调上爬**（棘轮），飞机边稳边自转，"
                + yaw_note +
                "模型也已被排除（同一条参考在无 ROS 的闭环里能悬停，见 "
                "`docs/control/tier-a-sitl-hover.md`）。真凶是**我们自己写的那两处约定**："
                "① `u_ref = 上一帧解` 对前馈构成正反馈 —— `offline-ocp.md` §4.4 离线就记过"
                "「单调上爬、不收敛」，本票在**真 PX4 闭环**里把它复现了（`u[3]`：0 ⟹ "
                "+4.696 rad/s，EKF 帧累计 13.6 圈）；② `q_diag` 的 `qw = 0`（同文 §2）让代价对"
                "「朝向」几乎无回正力，且**镜像机头 ψ = 2π − ψ_ref 的代价实测 = 0.000000** —— "
                "代价在参考与它的镜像之间**完全不区分**。★ 调这两处要动 ticket 04 的 M6 约定与"
                "权重（**不在 M1–M9 里**）⟹ 见本票 Comments、等操作者裁定，别自己改"
                "\n\n☠️ **2026-09-29 状态更新**：那一条**已经裁定并落地**了（ticket 16："
                "`q_diag` 的 `qw` 0.1、`qatt` 2.0；`R_yaw` 保持参考实现的 5e2 —— 同日一度"
                "降到 50，因会让 RTI 迭代在大误差下**收敛极慢**（发指令的窗口里逐拍乱跳）而**撤回**"
                " —— 见 `offline-ocp.md` §2 与 "
                "`.scratch/phase2/issues/16-ocp-yaw-weights-mirror-well.md`）"
                "⟹ 上面那句「等操作者裁定、别自己改」**已过期**（保留为当时记录）。"
                "⚠️ 本段读数（`u[3]`：0 ⟹ +4.696 rad/s、镜像代价 = 0.000000）是**当时那几场**"
                "（`qw = 0`）的历史读数 —— **别**按现在的默认权重去重读它们。")
    if failed("⑥"):
        return ("**输入（状态源）**：进了 OFFBOARD、方向也对，但滚转/俯仰长期贴上限 ⟹ "
                f"参考追不上（饱和计数 {dut.get('sat_count')}）：不是模型解不出，"
                "是喂进去的状态/参考里有东西对不上（谁给的、多旧 ⟹ 日志的 `state_source` / `age_*`）"
                + tail)
    if failed("③") or failed("④"):
        return ("**模型**：方向对、输入追得上（没有长期饱和）而 **EKF 帧内**的高度/振荡仍不过 ⟹ "
                "轮到「预测模型与代价」这一层" + tail)
    return (f"本场没有失败项：EKF 帧内抬升 {v.fmt(tr.get('climb'), '.3f')} m、"
            f"末段峰峰值 {v.fmt(tr.get('up_tail_p2p'), '.3f')} m ⟹ 控制器在**它能看见的那一帧**里"
            "把飞机稳住了；归因表不适用" + tail)


# ------------------------------------------------------------------ 报告


def print_plant_rows(plant) -> list:
    """体检那几行的**渲染**（与 `report()` 同一套口径：`counts=False` 的行是读数，不是判据）。

    ★ 这条小函数是为了修一个真出现过的错读：两条 `ok=None` 的**结论行**（"传感器仿真的源"、
      "误差在爬还是有界"）被 `'✅' if chk['ok'] else '❌'` 印成了 ❌ —— 看起来像"体检挂了 5 条"，
      其实只有 3 条判据没过。`None` 既不是通过也不是不通过。
    """
    out = []
    for chk in plant:
        if chk.get("ok") is None or not chk.get("counts", True):
            out.append(f"  ℹ️ {chk['name']}：{chk['detail']}")
        else:
            out.append(f"  {'✅' if chk['ok'] else '❌'} {chk['name']}：{chk['detail']}")
    return out


def report(obs, checks, plant=None) -> int:
    bad = 0
    print(f"\n=== A 档闭环：{obs.get('name')}（状态源 {obs.get('source')}）===")
    d = obs.get("dut") or {}
    print(f"  ·  真值采样 {obs.get('truth_n')} 点 / 被控对象时间 {v.fmt(obs.get('truth_span_s'), '.1f')}s"
          f"（墙钟 {v.fmt(obs.get('stream_wall_s'), '.1f')}s）⟹ "
          f"{v.fmt(obs.get('truth_hz'), '.2f')} Hz")
    print(f"  ·  DUT {d.get('frames')} 帧 / {v.fmt(d.get('span_s'), '.1f')}s ⟹ "
          f"{v.fmt(d.get('actual_hz'), '.1f')} Hz；档位 {d.get('fallback')}；"
          f"饱和 {d.get('sat_count') or '无'}")
    am = obs.get("gt_att_meta") or {}
    print(f"  ·  姿态真值 {am.get('n')} 点（命令 {am.get('sent')} 条 / 白等 {am.get('missed')} 次 / "
          f"残缺 {am.get('partial')}）—— 判据⑨ 的独立参照")
    print(f"  ·  OFFBOARD 于 t={v.fmt(obs.get('t', {}).get('offboard_enter'), '.2f')}s；"
          f"{obs.get('agent')}")
    s = obs.get("solve") or {}
    print(f"  ·  ★ D6：求解 p50 {v.fmt(s.get('p50'), '.3f')} / p99 {v.fmt(s.get('p99'), '.3f')} / "
          f"全集最坏 {v.fmt(s.get('max'), '.3f')} ms ⟹ {s.get('ladder')}；"
          f"去掉 warmup 后（稳态 n={s.get('steady_n')}）最坏 {v.fmt(s.get('steady_max'), '.3f')} ms "
          f"⟹ {s.get('ladder_steady')}")
    yv = obs.get("yaw") or {}
    if yv.get("n"):
        print(f"  ·  ★ 偏航（真值锚点）：真值累计转角 {v.fmt(yv.get('psi_span'), '+.2f')} rad / "
              f"指令 {v.fmt(yv.get('cmd_span'), '+.2f')} rad（残差 {v.fmt(yv.get('resid'), '+.2f')}）；"
              f"换算误差中位数 {v.fmt(yv.get('conv_err_med'), '+.3f')} rad")
    print(f"  ·  末段：电机 {obs.get('motors_end')}（均值 {v.fmt(obs.get('motors_mean_end'), '.4f')}"
          f" ⟹ {v.fmt(obs.get('thrust_n_end'), '.3f')} N）；EKF z="
          f"{v.fmt((obs.get('ekf_end') or {}).get('z'), '+.3f')} m；真值 z="
          f"{v.fmt((obs.get('gt_end') or {}).get('z'), '+.3f')} m")
    for chk in checks:
        if not chk.get("counts", True):
            print(f"  ℹ️ {chk['name']}：{chk['detail']}")
            continue
        print(f"  {'✅' if chk['ok'] else '❌'} {chk['name']}：{chk['detail']}")
        bad += 0 if chk["ok"] else 1
    print(f"  → 归因：{attribute(obs, checks)}")
    if plant is not None:
        print("\n--- 被控对象体检 ---")
        for chk in print_plant_rows(plant):
            print(chk)
    print(f"\n{'A 档闭环判据全部通过' if bad == 0 else f'{bad} 项判据不通过'}")
    return 0 if bad == 0 else 1


# ------------------------------------------------------------------ 主流程


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0],
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--plant-check", action="store_true",
                    help="只体检被控对象（不跑 DUT）：传感器的源、EKF 在融合什么、真值话题在不在 DDS 上")
    ap.add_argument("--start-sims", action="store_true",
                    help="体检时顺手 `sensor_baro_sim/mag_sim/gps_sim start`（本档唯一的被控对象侧动作）")
    ap.add_argument("--settle-after-sims", type=float, default=8.0)
    ap.add_argument("--plant-dwell-s", type=float, default=0.0,
                    help="地面静息反复采样多少秒（分「误差在爬」与「有界噪声」，默认 0 = 不采）")
    ap.add_argument("--source", default="px4", choices=("px4", "px4-gt"),
                    help="状态源：px4 = 票面那一对话题；px4-gt = 指向 groundtruth 对（需要先桥接）")
    ap.add_argument("--hold-s", type=float, default=25.0, help="OFFBOARD 生效后悬停多少秒")
    ap.add_argument("--rate-hz", type=float, default=100.0, help="给节点的 rate_hz（D6 的候选上限）")
    ap.add_argument("--truth-hz", type=float, default=4.0,
                    help="真值**稀疏采样**的目标频率（Hz；模块 docstring ★★★ 说明为什么不是连续流）")
    ap.add_argument("--gt-att-hz", type=float, default=1.0,
                    help="姿态真值（`vehicle_attitude_groundtruth`）的采样频率（判据⑨；"
                         "要的是累计转角，1 Hz 够，且它和控制台流量成正比）")
    ap.add_argument("--preflight-s", type=float, default=6.0, help="起飞前留多少秒（地面零点）")
    ap.add_argument("--hold-x", type=float, default=0.0)
    ap.add_argument("--hold-y", type=float, default=0.0)
    ap.add_argument("--hold-z", type=float, default=1.0, help="悬停高度（ENU，米）")
    ap.add_argument("--hold-yaw", type=float, default=0.0,
                    help="悬停参考机头（deg，透传给节点的 `hold_yaw_deg`，非 0 时**同时**传 "
                         "`keep_yaw:=false` —— 少了它参考机头取锚点机头、误差造不出来）。"
                         "★ 造一条**已知大小**的机头误差：SITL 里解锁时机头 ≈ 0 ⟹ 误差 ≈ 这个值。"
                         "见 `dut_params()` 的 ★★/★★★")
    #: ★ 默认给 **150 s**（不是 40）：`OfflineOcp()` 首次会代码生成 + 编译 C（`test_mpc_node`
    #:   的 `STARTUP_S` 也按 120 s 留）；卡太短会把"节点还在编译"读成"中间件没通"。
    #:   缓存命中时实测 ~8–12 s。
    ap.add_argument("--dut-timeout-s", type=float, default=150.0)
    ap.add_argument("--offboard-timeout-s", type=float, default=40.0)
    ap.add_argument("--outdir", default="/tmp/tier-a")
    ap.add_argument("--json", default=None)
    ap.add_argument("--replay", default=None)
    ap.add_argument("--src", default=str(Path.home() / "refs/px4-v1.14.3-build"))
    ap.add_argument("--build-dir", default=None)
    ap.add_argument("--ready-timeout", type=float, default=90.0)
    ap.add_argument("--boot-settle", type=float, default=4.0)
    ap.add_argument("--keep-params", action="store_true")
    ap.add_argument("--agent-bin", default="/usr/local/bin/MicroXRCEAgent")
    ap.add_argument("--agent-port", default="8888")
    ap.add_argument("--debug-dump", default="/tmp/verify_tier_a_console.log")
    args = ap.parse_args(argv)

    if args.replay:
        obs = json.loads(Path(args.replay).read_text(encoding="utf-8"))
        if "checks" not in obs:
            obs["checks"] = evaluate(obs)
        rc = report(obs, obs["checks"], obs.get("plant_checks"))
        return rc

    args.build_dir = args.build_dir or str(Path(args.src) / "build/px4_sitl_default")
    Path(args.outdir).mkdir(parents=True, exist_ok=True)
    if not (Path(args.build_dir) / "etc/init.d-posix/airframes/10046_sihsim_z0").exists():
        raise SystemExit("z0 机型没装 —— 先跑 install_z0_airframe.sh")
    if not args.keep_params:
        for f in ("parameters.bson", "parameters_backup.bson"):
            (Path(args.build_dir) / "rootfs" / f).unlink(missing_ok=True)

    rclpy.init()
    rec = v.Recorder()
    obs = {"name": "tier-a", "source": args.source, "rate_hz": args.rate_hz,
           "hover_s": 3.0, "plant_checks": None}
    rc = 0
    world = None
    try:
        with v.World(args) as world:
            print(f"[verify] COM_OF_LOSS_T = {world.com_of_loss_t}", file=sys.stderr)
            if args.plant_check:
                plant = run_plant_check(world, rec, args)
                obs["plant"] = plant
                obs["plant_checks"] = evaluate_plant(plant)
                for line in print_plant_rows(obs["plant_checks"]):
                    print(line)
                rc = 0 if all(c["ok"] or not c.get("counts", True)
                              for c in obs["plant_checks"]) else 1
            else:
                # ★ 飞之前先体检：它给出"EKF 到底有没有高度源"这个**前提**，
                #   没有它的话，一场失败会被读成控制器的问题（归因表也是靠它才判得动）。
                plant = run_plant_check(world, rec, args)
                obs["plant"] = plant
                obs["plant_checks"] = evaluate_plant(plant)
                cfg = argparse.Namespace(**vars(args))
                cfg.name = f"hover-{args.hold_z:g}m"
                # ★ 机头参考非零才加后缀：DUT 的 JSONL 与观测文件都按 `cfg.name` 命名，
                #   不加就会让 `--hold-yaw 20` 的那一场**覆盖**同高度的常规场（两场的证据
                #   互相踩掉、而文件名一样 ⟹ 事后分不清哪一场是哪一场）。
                if args.hold_yaw:
                    cfg.name += f"-yaw{args.hold_yaw:g}"
                cfg.log_file = str(Path(args.outdir) / f"{cfg.name}-{args.source}.jsonl")
                argv_dut = ["--ros-args"]
                for p in dut_params(cfg):
                    argv_dut += ["-p", p]
                agent = ArmAgent()
                with MpcDut(argv_dut, Path(cfg.log_file)) as dut:
                    obs.update(run_flight(world.px, rec, dut, cfg, agent))
                # ★ 判据层自己炸了也要把这一场说完：观测已经在手上了（`finally` 落盘），
                #   `--replay` 能把报告补出来。已经栽过两次（`segments` 的 TypeError、
                #   `evaluate` 里的一个 `UnboundLocalError`），两次都是**飞完一整场之后**
                #   崩在报告层 —— 那种损失本来一次都不该有。
                try:
                    obs["checks"] = evaluate(obs)
                except Exception as e:  # noqa: BLE001 —— 判据层的任何异常都不该吃掉一场飞行
                    traceback.print_exc()
                    rc = 3
                    obs["checks"] = [{
                        "name": "☠ 判据层自己抛异常（本场的观测仍然完整，可 `--replay` 重判）",
                        "ok": False, "counts": True,
                        "detail": f"{type(e).__name__}: {e}"}]
                rc = report(obs, obs["checks"], obs.get("plant_checks")) or rc
                agent.destroy_node()
    except (v.Px4ConsoleError, OSError) as e:
        print(f"[verify] ☠ 世界没起来：{e!r}", file=sys.stderr)
        rc = 2
    finally:
        if world is not None and world.px is not None:
            Path(args.debug_dump).write_text(world.px.text(), encoding="utf-8")
            print(f"[verify] 控制台全文写到 {args.debug_dump}", file=sys.stderr)
        # ★ 观测在 `finally` 里落盘（不是走完流程才落）：一场 SITL 是 3–5 分钟，
        #   判据函数里一个笔误就能把这几分钟的证据一起丢掉（已经发生过一次 ——
        #   `dwell_summary` 把标志字典塞进 set 触发 TypeError）。
        if args.json:
            Path(args.json).parent.mkdir(parents=True, exist_ok=True)
            Path(args.json).write_text(
                json.dumps(obs, indent=2, ensure_ascii=False, default=float), encoding="utf-8")
            print(f"[verify] 观测写到 {args.json}", file=sys.stderr)
        rec.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return rc


if __name__ == "__main__":
    sys.exit(main())
