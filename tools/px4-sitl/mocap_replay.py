#!/usr/bin/env python3
"""B 档的**动捕回放夹具**（ticket 11）—— 把 SITL 真值按真实包的脾气变成 `/mocap/pose_enu`。

**接法由操作者 2026-09-29 裁定**（[11 号票](../../.scratch/phase2/issues/11-sitl-tier-b-seam3.md) Comments 四·2）：

    /mocap/pose_enu 上放的 = SITL 真值(t − D) ⊕ 包里量出来的噪声
    发帧节奏             = 包里那一串**真实间隔**（逐帧复现，不是抽分布）

★ **为什么不是"把包里的帧原样放出去"**（那是"状态源 = 包回放"的字面读法）：那会**断开反馈** ——
  `/mocap/pose_enu` 上是录好的帧，飞机怎么动控制器看不见，MPC（模型预测控制）会对着一段
  既定轨迹使劲。票面判据 3（注入 50 ms 延迟后仍稳定）在这种情况下**失去意义**。
  ⟹ 真值必须来自**当前这一场**；包里出的是**噪声与时序**，不是轨迹。

**为什么真值要绕 DDS**（`/fmu/out/vehicle_*_groundtruth`）：这台 SITL 里唯一不是估计器算出来的
  位姿就是 SIH 的真值（`sih.cpp:495-564` 直接发刚体积分出来的 `_p_I`）；EKF 那一路的助航是
  **自反馈**（气压/GPS 仿真喂的是 EKF 自己的输出），实测与真值差 0.4 m 量级
  （见 `verify_tier_a_hover.py --plant-check` 的读数）。拿 EKF 当"动捕"会把估计器的偏差
  记到控制器账上。桥接那两条话题由 `enable_groundtruth_dds.sh` 负责。

**三样输入**（本节点**不自己量包** —— 量包是 `mocap_model.py` 的事，那样"跑的这一场"与
  "留档的数"才是同一份）：

| 参数 | 是什么 |
|---|---|
| `--model` | `mocap_model.py dump` 出来的 JSON（唯一输入） |
| `--extra-delay-ms` | 在模型自带的基延迟上**再加**多少（判据 3 用 50） |
| `--seed` | 噪声随机种子（要可复现） |

**输出**：`/mocap/pose_enu`（`geometry_msgs/PoseStamped`、RELIABLE depth 10、`frame_id='world'`
—— 与真实那条逐字一致，见 `mpc_node.QOS_MOCAP` 与 `mocap_rel_pose_node.cpp:93`）
+ 一份 JSONL（每帧记一行：发了什么、取的是哪一帧真值、延迟多少、实际间隔）。

用法（**先 source 工作空间**）：

    python3 tools/px4-sitl/mocap_replay.py --model /tmp/mocap-model-3.json --out /tmp/replay.jsonl
    python3 tools/px4-sitl/mocap_replay.py --model … --extra-delay-ms 50     # 判据 3
"""

from __future__ import annotations

import argparse
import json
import signal
import sys
import threading
import time
from pathlib import Path
from typing import Optional, Sequence

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from px4_msgs.msg import VehicleAttitude, VehicleLocalPosition
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

# `/fmu/out/*` 是 best_effort（PX4 侧 `utilities.hpp:114-119`）⟹ 订阅方必须不**强于**它
QOS_TRUTH = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=10,
                       reliability=ReliabilityPolicy.BEST_EFFORT)
#: 与 `mpc_node.QOS_MOCAP` **逐字一致**（可靠、depth 10）：真实那条是 `QoS(10)` 发的
#: （`mocap_rel_pose_node.cpp:93`、`bench/vrpn_source.py:mocap_qos()`）
QOS_MOCAP = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=10,
                       reliability=ReliabilityPolicy.RELIABLE)

TOPIC_MOCAP = "/mocap/pose_enu"
#: ☠️ 这三个与 `verify_tier_b_seam3.py` 的同名常量**是两份**（不是同一份）。
#: 两份是**有意的**：本夹具要能**单独跑**（`python3 mocap_replay.py …`，不经过验证器），
#: 反过来验证器 import 本模块 ⟹ 从本模块 import 验证器就是**循环依赖**。
#: 但两份就有漂移的可能：改话题名/QoS 时**两处一起改**（改了这里不改那里，
#: 症状是"真值流一条都收不到"，而验证器判出来的只会是"夹具没发帧"）。
TOPIC_GT_POS = "/fmu/out/vehicle_local_position_groundtruth"
TOPIC_GT_ATT = "/fmu/out/vehicle_attitude_groundtruth"

#: 真值缓冲留多长 [s]：只要盖过最大延迟（基延迟 + 注入）再加一点
BUFFER_S = 3.0


def nlerp(q0: np.ndarray, q1: np.ndarray, w: float) -> np.ndarray:
    """四元数线性插值 + 归一化（`w∈[0,1]`）。**先对齐符号**：`q` 与 `−q` 是同一个姿态，
    不对齐的话插值会走"反方向绕远路"（姿态上看不出来，转过的角却是错的）。"""
    q1 = q1 if float(np.dot(q0, q1)) >= 0.0 else -q1
    q = (1.0 - w) * np.asarray(q0, dtype=float) + w * np.asarray(q1, dtype=float)
    n = float(np.linalg.norm(q))
    return q / n if n > 1e-9 else np.asarray(q0, dtype=float)


class TruthBuffer:
    """按**到达时刻**（本进程的墙钟）存真值帧 —— 延迟就是"取多老的那一帧"。

    ★ 用到达时刻而不是消息里的 `timestamp`（那是飞控上电起算的，与墙钟差着一个
      `time_offset`，`mpc_node` 自己也只在**同源自比**时才用它）。量的是"多久之前的那一帧"，
      到达时刻才是正确的量。
    """

    def __init__(self, span_s: float = BUFFER_S) -> None:
        self.span_s = span_s
        self._lock = threading.Lock()
        self._t: list = []
        self._p: list = []          # NED 位置
        self._q: list = []          # FRD→NED 四元数（**wxyz**，与 PX4 `VehicleAttitude.q` 同序）

    def push(self, t_s: float, pos_ned, quat_wxyz) -> None:
        with self._lock:
            self._t.append(float(t_s))
            self._p.append(np.asarray(pos_ned, dtype=float))
            self._q.append(np.asarray(quat_wxyz, dtype=float))
            cut = self._t[-1] - self.span_s
            i = 0
            while i < len(self._t) and self._t[i] < cut:
                i += 1
            if i:
                del self._t[:i], self._p[:i], self._q[:i]

    def stats(self) -> dict:
        with self._lock:
            n = len(self._t)
            if n < 2:
                return {"n": n}
            span = self._t[-1] - self._t[0]
            return {"n": n, "span_s": round(span, 3),
                    "rate_hz": round((n - 1) / span, 2) if span > 0 else None,
                    "newest_age_s": round(time.perf_counter() - self._t[-1], 4)}

    def sample_at(self, t_target: float):
        """取 `t_target` 时刻的 `(位置 NED, 四元数 xyzw)`；插值，样本不够时返回 `None`。

        返回的第三项是"实际用的那一段有多旧"（`t_target − t_newer`，负数是正常范围）。
        """
        with self._lock:
            if len(self._t) < 2:
                return None, None, None
            t = self._t
            if t_target <= t[0] or t_target >= t[-1]:
                return None, None, None                    # 窗口外：**不猜**（不夹到端点）
            lo, hi = 0, len(t) - 1
            while hi - lo > 1:                             # 二分：t 是有序的
                mid = (lo + hi) // 2
                if t[mid] <= t_target:
                    lo = mid
                else:
                    hi = mid
            w = (t_target - t[lo]) / max(t[hi] - t[lo], 1e-9)
            pos = (1.0 - w) * self._p[lo] + w * self._p[hi]
            quat = nlerp(self._q[lo], self._q[hi], w)
            return pos, quat, t_target - t[lo]


def ned_frd_to_enu_flu(pos_ned, quat_wxyz):
    """NED/FRD → ENU/FLU —— **调用 `nmpc.node_core` 里那一处**，不在这里重写符号。

    ★ 这是刻意的：`px4_pose_to_enu` 是"读错一个号"的唯一防波堤（`node_core.py` 那段 ★★），
      夹具自己再推一遍符号，就等于在**被测链路的另一侧**复制了一份可能同样的错 ——
      而夹具与被测代码共享同一个错时，测试会**一起错并且通过**（票面 §35 否决那个方案
      的第一手理由）。

    ☠☠ 四元数分量序：入参必须是 **wxyz**（`px4_quat_to_enu` 的 `Q_NED_TO_ENU =
      [0, √.5, √.5, 0]` 就是按 wxyz 写的），返回值也是 **wxyz**。
      2026-09-29 这一处曾经错成"进 xyzw、出当 xyzw"，**两个错不抵消**：
      水平机（NED/FRD 下单位四元数）本该出 **绕 z 转 90°**（`(0, 0, ±√.5, √.5)`），
      实际出的是 **绕 x 转 −90°**（`(∓√.5, 0, 0, √.5)`）。
      后果不是"歪一点"，是**姿态误差满 90°**：B 档首飞飞机全程趴在地面
      （`thrust_body[2] = −0.0000`、`F_min` 饱和 1543/1544、真值倾角 0.00°）——
      控制器一直在"把 90° 滚转扳回来"，代价就是把推力丢到下限。
      `test_mocap_replay.py` 现在用**矩阵恒等式**（`R_enu = L·R_ned·M`）钉着这一处。
    """
    from nmpc.node_core import px4_pose_to_enu                      # noqa: PLC0415
    return px4_pose_to_enu(pos_ned, quat_wxyz)


def orientation_xyzw(q_wxyz):
    """`wxyz` → `PoseStamped.orientation` 的 `(x, y, z, w)`。

    ★★ **全链路唯一一次换序**，单独成函数就是为了能单测它 —— 写反了不报错、不 NaN，
      只让姿态整体偏 90°（见 `ned_frd_to_enu_flu` 的 ☠☠）。
    """
    w, x, y, z = (float(c) for c in q_wxyz)
    return (x, y, z, w)


class MocapReplay(Node):
    def __init__(self, model: dict, args) -> None:
        super().__init__("mocap_replay")
        self.model = model
        self.args = args
        self.pub = self.create_publisher(PoseStamped, TOPIC_MOCAP, QOS_MOCAP)
        self.buf = TruthBuffer()
        self.rng = np.random.default_rng(args.seed)
        self.sigma = np.asarray(model["noise"]["sigma_m"], dtype=float)
        self.seq = [float(x) for x in model["dt_seq_s"]]
        self.base_delay_s = (args.base_delay_ms * 1e-3 if args.base_delay_ms is not None
                             else float(model["lag"]["p50_ms"]) * 1e-3)
        self.delay_s = self.base_delay_s + args.extra_delay_ms * 1e-3
        self._stop = threading.Event()
        self._log = None
        self._log_path = Path(args.out) if args.out else None
        self.frames = 0
        self.t_first = None
        self.t_last = None
        self.missed_window = 0
        self.dt_actual: list = []
        self.create_subscription(VehicleLocalPosition, TOPIC_GT_POS, self._on_pos, QOS_TRUTH)
        self.create_subscription(VehicleAttitude, TOPIC_GT_ATT, self._on_att, QOS_TRUTH)
        self.get_logger().info(
            f"回放夹具起来了：模型={model['bag']}、窗口 {model['segment']['t0_s']:.1f}–"
            f"{model['segment']['t1_s']:.1f} s、{len(self.seq)} 个间隔；"
            f"延迟 = 基 {self.base_delay_s * 1e3:.2f} ms + 注入 {args.extra_delay_ms:.1f} ms "
            f"= {self.delay_s * 1e3:.2f} ms；σ = "
            + ", ".join(f"{s * 1e3:.4f}" for s in self.sigma) + " mm")

    # ── 真值入缓冲（两个回调各写自己那一半，位置与姿态**分开存不算数**：一起存）──
    def _on_pos(self, msg) -> None:
        self._last_pos = (float(msg.x), float(msg.y), float(msg.z))
        self._push_if_paired()

    def _on_att(self, msg) -> None:
        # ★ 原样存 `msg.q`（PX4 就是 wxyz）—— 缓冲、转换、判据三处**同一序**，
        #   全链路只在与 `PoseStamped` 交界的那一行换成 xyzw（见 `_publish_frame`）。
        self._last_att = (float(msg.q[0]), float(msg.q[1]), float(msg.q[2]), float(msg.q[3]))
        self._push_if_paired()

    def _push_if_paired(self) -> None:
        """两条流**都到过**才入缓冲。

        ★ 为什么必须成对：动捕给的是**一个刚体的位姿**（位置与姿态同一帧）。若各推各的，
          位置用 1 号的、姿态用 2 号的，就去造了一个**真实系统里不存在**的混合帧。
          两条流在 SIH 里同频（同一次 `publish_ground_truth()` 里发），配对误差 ≤1 拍。
        """
        p, a = getattr(self, "_last_pos", None), getattr(self, "_last_att", None)
        if p is None or a is None:
            return
        self.buf.push(time.perf_counter(), p, a)

    # ── 发帧线程 ──
    def run(self) -> None:
        if self._log_path:
            self._log_path.parent.mkdir(parents=True, exist_ok=True)
            self._log = self._log_path.open("w", encoding="utf-8")
        th = threading.Thread(target=self._loop, name="replay", daemon=True)
        th.start()
        try:
            while not self._stop.is_set():
                time.sleep(0.1)
        finally:
            self._stop.set()
            th.join(timeout=2.0)
            if self._log:
                self._log.close()

    def stop(self, *_a) -> None:
        self._stop.set()

    def _loop(self) -> None:
        """按包里那一串真实间隔发帧。**到点才发**，不发就空转。

        ★ 时间基准是 `time.perf_counter()`（单调），不是墙钟：墙钟会被 NTP 拽。
        ★ 近处的等待用**自旋**（最后 0.4 ms），远处的用 `sleep` —— 包里的间隔中位数是
          0.98 ms（真实流是"爆发式"的），用 `sleep` 的粒度根本表达不出来。
          代价：每帧最多自旋 0.4 ms，按 ~190 帧/s 算 ≈ 7.6% 的一个核，可接受。
        """
        i = 0
        t_next = time.perf_counter()
        while not self._stop.is_set():
            now = time.perf_counter()
            if now + 4e-4 < t_next:
                time.sleep(min(0.05, max(0.0, t_next - now - 4e-4)))
                continue
            if now < t_next:
                continue                                     # 自旋最后一小段
            self._publish_frame()
            dt = self.seq[i % len(self.seq)] if self.seq else 0.005
            i += 1
            t_next += dt
            if t_next < now - 0.05:                          # 落后太多 ⟹ 重新对齐（别一路追赶）
                t_next = now

    def _publish_frame(self) -> None:
        t_now = time.perf_counter()
        pos, quat, dt_since = self.buf.sample_at(t_now - self.delay_s)
        if pos is None:
            self.missed_window += 1
            return
        p_enu, q_flu, _ = ned_frd_to_enu_flu(pos, quat)
        noise = self.rng.normal(0.0, self.sigma)
        p_enu = tuple(float(c) + float(n) for c, n in zip(p_enu, noise))
        msg = PoseStamped()
        stamp_ns = time.time_ns()
        msg.header.stamp.sec = int(stamp_ns // 1_000_000_000)
        msg.header.stamp.nanosec = int(stamp_ns % 1_000_000_000)
        msg.header.frame_id = "world"
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = (float(c) for c in p_enu)
        ox, oy, oz, ow = orientation_xyzw(q_flu)
        msg.pose.orientation.x = ox
        msg.pose.orientation.y = oy
        msg.pose.orientation.z = oz
        msg.pose.orientation.w = ow
        self.pub.publish(msg)
        self.frames += 1
        if self.t_first is None:
            self.t_first = t_now
        else:
            self.dt_actual.append(t_now - self.t_last)
        self.t_last = t_now
        if self._log:
            self._log.write(json.dumps({
                "wall_ns": stamp_ns, "t_perf": round(t_now, 6),
                "delay_ms": round(self.delay_s * 1e3, 3),
                "truth_dt_s": None if dt_since is None else round(float(dt_since), 6),
                "pos": [round(float(c), 6) for c in p_enu],
                # 按**消息字段序**记（x, y, z, w），这样"日志里写的"与"话题上发的"
                # 逐字对齐 —— 复盘时不必再想一次序。
                "quat_xyzw": [round(float(c), 8) for c in orientation_xyzw(q_flu)],
            }) + "\n")
            if self.frames % 64 == 0:
                self._log.flush()

    def summary(self) -> dict:
        dt = np.asarray(self.dt_actual, dtype=float)
        out = {"frames": self.frames,
               "span_s": (None if self.t_first is None or self.t_last is None
                          else round(self.t_last - self.t_first, 3)),
               "delay_ms": round(self.delay_s * 1e3, 3),
               "base_delay_ms": round(self.base_delay_s * 1e3, 3),
               "extra_delay_ms": self.args.extra_delay_ms,
               "no_signal_frames": self.missed_window,
               "sigma_m": [float(x) for x in self.sigma],
               "truth": self.buf.stats()}
        if dt.size:
            out["dt_actual"] = {"n": int(dt.size), "mean_ms": round(float(dt.mean() * 1e3), 3),
                                "p50_ms": round(float(np.percentile(dt, 50) * 1e3), 3),
                                "p99_ms": round(float(np.percentile(dt, 99) * 1e3), 3),
                                "max_ms": round(float(dt.max() * 1e3), 3)}
        return out


def main(argv: Optional[Sequence[str]] = None) -> int:
    ap = argparse.ArgumentParser(description="B 档动捕回放夹具（ticket 11）")
    ap.add_argument("--model", required=True, help="mocap_model.py dump 出来的 JSON")
    ap.add_argument("--extra-delay-ms", type=float, default=0.0,
                    help="在模型基延迟上再加多少（判据 3 用 50）")
    ap.add_argument("--base-delay-ms", type=float, default=None,
                    help="覆盖模型的基延迟（默认取模型里 stamp→记录器 的中位值）")
    ap.add_argument("--seed", type=int, default=20260929)
    ap.add_argument("--out", default=None, help="逐帧 JSONL 写到哪")
    ap.add_argument("--summary", default=None, help="收尾汇总写到哪（JSON）")
    args = ap.parse_args(argv)

    model = json.loads(Path(args.model).read_text(encoding="utf-8"))
    if not model.get("dt_seq_s"):
        print(f"模型 {args.model} 里没有 dt_seq_s —— 先跑 mocap_model.py dump", file=sys.stderr)
        return 2
    # 老掉牙的 numpy 兼容不是问题；这里只要求 numpy 在
    rclpy.init()
    node = MocapReplay(model, args)
    signal.signal(signal.SIGINT, node.stop)
    signal.signal(signal.SIGTERM, node.stop)

    def spin():
        while not node._stop.is_set() and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)

    th = threading.Thread(target=spin, name="spin", daemon=True)
    th.start()
    try:
        node.run()
    finally:
        s = node.summary()
        print(json.dumps(s, ensure_ascii=False, indent=1))
        if args.summary:
            Path(args.summary).parent.mkdir(parents=True, exist_ok=True)
            Path(args.summary).write_text(json.dumps(s, ensure_ascii=False, indent=1),
                                          encoding="utf-8")
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
