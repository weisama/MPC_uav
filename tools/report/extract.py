#!/usr/bin/env python3
"""把输入读成数组：六份话题包 + 三趟 SITL summary。

这一层**只做读取与清洗**，不做坐标换算（那是 `align.py`）、不算指标（`metrics.py`）。
所有时间轴都保留**包记录时间戳（epoch 秒）**；减 `t0_abs` 是 `build.py` 的事 ——
同一份夹具既能给"相对时间"用，也能给"与别的传感器对表"用。

★ 读包走 `sqlite3` 直查而不是 `rosbag2_py` 的顺序读：包是 SQLite 存储，
  只取六七个话题时直接 `where topic_id=? order by timestamp`，不必顺序解码整包；
  反序列化仍用 ROS 的 `deserialize_message`，口径与 `rosbag2_py` 一致。

⚠️ 本模块的顶层**不 import ROS**：`extract` 在没 source 环境的测试里也要能 import
  （仿真摘要那几条用例不需要 ROS）。ROS 的 import 放在 `_ros()` 里，第一次读包时才发生。
"""

from __future__ import annotations

import hashlib
import json
import sqlite3
import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[1]
SITL_DIR = HERE / "evidence" / "sitl"

sys.path.insert(0, str(REPO / "tools" / "px4-sitl"))
from mocap_model import prepare_bag  # noqa: E402  （只依赖 numpy/subprocess，不 import ROS）

from align import (  # noqa: E402
    completion_from_progress,
    completion_from_reference,
    find_state_window,
    origin_from_command,
    position_at,
)

WAIT_START = "AUTO/WAIT_START"

#: 各话题的读取口径。键 = 话题名，值 = 需要的字段（`PositionTarget` 等按类型取）。
OFFBOARD_TOPICS = (
    "/traj/start",
    "/control/state",
    "/traj/reference",
    "/traj/progress",
    "/mavros/setpoint_raw/local",       # offboard 的原点 / 指令流
    "/mavros/vision_pose/pose",         # 报告用"实际轨迹"（姿态也从这里）
    "/mavros/local_position/velocity_local",
    "/mavros/local_position/pose",      # 仅作原点回退（口径 2）
)
MPC_TOPICS = (
    "/traj/start",
    "/control/state",
    "/traj/reference",
    "/traj/progress",
    "/control/target",                  # 模型预测控制的原点 / 前馈
    "/mavros/setpoint_raw/attitude",    # 推力（归一化）× t_max、角速率
    "/mavros/vision_pose/pose",
    "/mavros/local_position/velocity_local",
    "/mavros/local_position/pose",      # 仅作原点回退（口径 2）
)


# ------------------------------------------------------------------ 哈希


def sha256_file(path: Path) -> str:
    h = hashlib.sha256()
    with path.open("rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def sha256_of_bag(bag_dir: Path) -> str:
    """包内容 sha256：各 `*.db3*` 文件哈希的**十六进制串**依次喂进 sha256。

    与 `run/rec.py:922` 完全同口径，`metadata.yaml` 不算（它带时间戳，逐次会变）。
    没有报文段时返回空串，不返回 `sha256("")`（那长得像一个正经哈希）。
    """
    files = sorted(bag_dir.glob("*.db3*"))
    if not files:
        return ""
    h = hashlib.sha256()
    for p in files:
        h.update(sha256_file(p).encode())
    return h.hexdigest()


# ------------------------------------------------------------------ 读包


def _ros():
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    return deserialize_message, get_message


@dataclass
class RawTopic:
    name: str
    typ: str
    t: np.ndarray                      # 包记录时间戳（epoch 秒）
    msgs: list = field(repr=False)


def _single_db(root: Path, bag_dir: Path) -> Path:
    """解包目录里**唯一**的 `.db3`。

    多分片（`*.db3` 多个）直接抛错：当前实现只读第一个，静默截断会得到
    "短了一截但哈希完整"的数据。要支持分片得先合并读取顺序（未做）。
    """
    files = sorted(Path(root).glob("*.db3"))
    if not files:
        raise FileNotFoundError(f"{bag_dir} 里找不到 .db3（先确认 metadata.yaml 完好）")
    if len(files) > 1:
        raise ValueError(
            f"{bag_dir} 有 {len(files)} 个 .db3 分片（{[p.name for p in files]}）；"
            "本工具只支持单分片，直接失败而不是静默截断")
    return files[0]


def read_topics(bag_dir: Path, workdir: Path, topics: Sequence[str],
                *, missing_ok: bool = False) -> Dict[str, RawTopic]:
    """把 `topics` 里出现的话题整个读出来。

    `missing_ok=False`（默认）时缺任何话题都抛错；`True` 时只返回存在的话题，
    由调用方决定哪些是必需的（原点话题的回退路径要用）。
    """
    root = prepare_bag(Path(bag_dir), Path(workdir))
    db = _single_db(root, bag_dir)
    con = sqlite3.connect(str(db))
    try:
        id_map = {name: (tid, typ) for tid, name, typ in
                  con.execute("select id, name, type from topics").fetchall()}
        missing = [t for t in topics if t not in id_map]
        if missing and not missing_ok:
            raise ValueError(f"{bag_dir.name} 缺话题：{missing}")
        deserialize_message, get_message = _ros()
        out: Dict[str, RawTopic] = {}
        for name in topics:
            if name not in id_map:
                continue
            tid, typ = id_map[name]
            rows = con.execute(
                "select timestamp, data from messages where topic_id=? order by timestamp",
                (tid,)).fetchall()
            if not rows:
                raise ValueError(f"{bag_dir.name} 的话题 {name} 一条报文都没有")
            cls = get_message(typ)
            msgs = [deserialize_message(bytes(r[1]), cls) for r in rows]
            ts = np.asarray([r[0] for r in rows], dtype=np.int64) * 1e-9
            out[name] = RawTopic(name=name, typ=typ, t=ts, msgs=msgs)
    finally:
        con.close()
    return out


def _bag_end_abs(db_dir: Path) -> float:
    db = _single_db(Path(db_dir), Path(db_dir))
    con = sqlite3.connect(str(db))
    try:
        t_max = con.execute("select max(timestamp) from messages").fetchone()[0]
    finally:
        con.close()
    if t_max is None:
        raise ValueError(f"{db_dir} 是空包")
    return float(t_max) * 1e-9


def _posestamped(topic: RawTopic) -> Tuple[np.ndarray, np.ndarray]:
    p = np.array([[m.pose.position.x, m.pose.position.y, m.pose.position.z]
                  for m in topic.msgs], dtype=float)
    # ROS 的 `orientation` 是 xyzw；本项目内部一律 wxyz。
    q = np.array([[m.pose.orientation.w, m.pose.orientation.x,
                   m.pose.orientation.y, m.pose.orientation.z]
                  for m in topic.msgs], dtype=float)
    return p, q


def _first_stamp_abs(topic: RawTopic) -> float:
    """首条报文的 `header.stamp`（epoch 秒）。

    仿真回放按**报文头时间戳**重发参考，所以 sim 的 t=0 要据此还原到
    `/traj/start`（两者差 7.7–15.6 ms，见 spec §3）。
    """
    h = topic.msgs[0].header.stamp
    return float(h.sec) + float(h.nanosec) * 1e-9


def _twist_linear(topic: RawTopic) -> np.ndarray:
    return np.array([[m.twist.linear.x, m.twist.linear.y, m.twist.linear.z]
                     for m in topic.msgs], dtype=float)


def _position_target(topic: RawTopic) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    p = np.array([[m.position.x, m.position.y, m.position.z] for m in topic.msgs], dtype=float)
    v = np.array([[m.velocity.x, m.velocity.y, m.velocity.z] for m in topic.msgs], dtype=float)
    yaw = np.array([m.yaw for m in topic.msgs], dtype=float)
    mask = np.array([m.type_mask for m in topic.msgs], dtype=np.int64)
    return p, v, yaw, mask


def _floats(topic: RawTopic) -> np.ndarray:
    return np.array([float(m.data) for m in topic.msgs], dtype=float)


def _strings(topic: RawTopic) -> np.ndarray:
    return np.array([str(m.data) for m in topic.msgs], dtype=object)


# ------------------------------------------------------------------ 数据类


@dataclass
class BagExtract:
    """一份实飞包读出来的全部数组。

    时间列（`*_t`）都是**包记录时间戳（epoch 秒）**；`*_s` 才是相对 `t0_abs` 的秒数。
    """

    name: str
    kind: str                          # "offboard" | "mpc"
    bag_dir: str
    t0_abs: float
    origin_abs: np.ndarray
    origin_spread_m: float
    origin_window_abs: Tuple[float, float]
    origin_source: str                 # 实际取原点的话题（含回退说明）
    completion_s: float
    completion_reason: str             # "progress" | "ref_end"
    package_end_s: float
    ref_first_stamp_abs: float         # 首条 /traj/reference 的 header.stamp
    ref_t: np.ndarray
    ref_p: np.ndarray
    ref_v: np.ndarray
    ref_yaw: np.ndarray
    progress_t: np.ndarray
    progress: np.ndarray
    cmd_t: np.ndarray
    cmd_p: np.ndarray
    actual_t: np.ndarray
    actual_p_abs: np.ndarray
    actual_q_wxyz: np.ndarray
    actual_v: np.ndarray
    vel_t: np.ndarray
    u_t: Optional[np.ndarray] = None
    u_thrust_n: Optional[np.ndarray] = None
    u_rate: Optional[np.ndarray] = None
    feedback_source: str = ""
    velocity_source: str = "/mavros/local_position/velocity_local（ENU）"


def read_offboard_bag(bag_dir: Path, workdir: Path) -> BagExtract:
    """offboard 包：原点取 `/mavros/setpoint_raw/local` 的 WAIT_START 恒定值。"""
    return _read_bag(bag_dir, workdir, kind="offboard")


def read_mpc_bag(bag_dir: Path, workdir: Path) -> BagExtract:
    """模型预测控制实飞包：原点取 `/control/target` 的 WAIT_START 恒定值。"""
    return _read_bag(bag_dir, workdir, kind="mpc")


def _origin_from_pose(pose_t: np.ndarray, pose_p: np.ndarray,
                      window: Tuple[float, float]) -> Tuple[np.ndarray, float, str]:
    """口径 2 的回退：首个 `AUTO/WAIT_START` 时 `/mavros/local_position/pose` 的位置。

    只取单点（不做均值），`spread` 记 0；回退原因写进返回值，最终落进 JSON。
    """
    idx = np.flatnonzero(np.asarray(pose_t, dtype=float) >= window[0])
    if idx.size:
        origin = np.asarray(pose_p, dtype=float)[idx[0]]
        why = "单点：首个 ≥ WAIT_START 起点的样本"
    else:
        origin = position_at(pose_t, pose_p, window[0])
        why = "窗口内没有位姿样本，顶住端点"
    return origin, 0.0, (
        f"/mavros/local_position/pose（回退：指令话题缺失；{why}）")


def _read_bag(bag_dir: Path, workdir: Path, kind: str) -> BagExtract:
    topics = OFFBOARD_TOPICS if kind == "offboard" else MPC_TOPICS
    raw = read_topics(bag_dir, workdir, topics, missing_ok=True)
    required = ("/traj/start", "/control/state", "/traj/reference", "/traj/progress",
                "/mavros/vision_pose/pose", "/mavros/local_position/velocity_local")
    missing = [t for t in required if t not in raw]
    if missing:
        raise ValueError(f"{bag_dir.name} 缺话题：{missing}")

    start = raw["/traj/start"]
    if start.t.size != 1:
        raise ValueError(f"{bag_dir.name}: /traj/start 应恰好 1 条，实际 {start.t.size}")
    t0 = float(start.t[0])

    state = raw["/control/state"]
    states = _strings(state)
    wait = find_state_window(state.t, states, WAIT_START)

    origin_topic = "/mavros/setpoint_raw/local" if kind == "offboard" else "/control/target"
    cmd_t = cmd_p = None
    origin = spread = None
    origin_source = ""
    if origin_topic in raw:
        cmd_p, _cmd_v, _cmd_yaw, _cmd_mask = _position_target(raw[origin_topic])
        cmd_t = raw[origin_topic].t
        try:
            origin, spread = origin_from_command(cmd_t, cmd_p, wait)
            origin_source = origin_topic
        except ValueError as exc:
            # 只有"窗口内没样本"才回退；指令存在但不恒定是坏数据，必须炸。
            if "没有指令样本" not in str(exc):
                raise
    if origin is None:
        if "/mavros/local_position/pose" not in raw:
            raise ValueError(
                f"{bag_dir.name}: 原点话题 {origin_topic} 与回退话题 "
                "/mavros/local_position/pose 都不可用")
        pose_p, _pose_q = _posestamped(raw["/mavros/local_position/pose"])
        origin, spread, origin_source = _origin_from_pose(
            raw["/mavros/local_position/pose"].t, pose_p, wait)

    ref_topic = raw["/traj/reference"]
    ref_p, ref_v, ref_yaw, _ref_mask = _position_target(ref_topic)
    ref_t = ref_topic.t
    progress = _floats(raw["/traj/progress"])
    progress_t = raw["/traj/progress"].t
    try:
        completion, reason = completion_from_progress(progress_t, progress, t0)
    except ValueError:
        completion, reason = completion_from_reference(ref_t, t0)

    actual_p, actual_q = _posestamped(raw["/mavros/vision_pose/pose"])
    actual_t = raw["/mavros/vision_pose/pose"].t
    actual_v = _twist_linear(raw["/mavros/local_position/velocity_local"])

    u_t = u_thrust = u_rate = None
    if kind == "mpc":
        att = raw["/mavros/setpoint_raw/attitude"]
        u_t = att.t
        u_thrust = np.array([float(m.thrust) for m in att.msgs], dtype=float)
        u_rate = np.array([[m.body_rate.x, m.body_rate.y, m.body_rate.z]
                           for m in att.msgs], dtype=float)

    # `prepare_bag` 返回的就是解包目录；包尾用全库最大记录时间。
    package_end = _bag_end_abs(prepare_bag(Path(bag_dir), Path(workdir))) - t0

    return BagExtract(
        name=Path(bag_dir).name,
        kind=kind,
        bag_dir=str(Path(bag_dir).relative_to(REPO)),
        t0_abs=t0,
        origin_abs=origin,
        origin_spread_m=spread,
        origin_window_abs=wait,
        origin_source=origin_source,
        completion_s=completion,
        completion_reason=reason,
        package_end_s=package_end,
        ref_first_stamp_abs=_first_stamp_abs(ref_topic),
        ref_t=ref_t, ref_p=ref_p, ref_v=ref_v, ref_yaw=ref_yaw,
        progress_t=progress_t, progress=progress,
        cmd_t=cmd_t, cmd_p=cmd_p,
        actual_t=actual_t, actual_p_abs=actual_p,
        actual_q_wxyz=actual_q, actual_v=actual_v,
        vel_t=raw["/mavros/local_position/velocity_local"].t,
        u_t=u_t, u_thrust_n=u_thrust, u_rate=u_rate,
        # 票 01 要求这里记的是**反馈**来源，不是指令话题。
        feedback_source=("/mavros/local_position/pose（PX4 内置 PID 的反馈，EKF 30 Hz）"
                         if kind == "offboard" else
                         "/mocap/pose_enu（模型预测控制的反馈，原始动捕）"),
    )


# ------------------------------------------------------------------ SITL summary


def sim_reference_start(summary: dict, ref_delay_s: float) -> Tuple[float, str]:
    """仿真 t=0 的绝对时刻（epoch 秒）。

    回放夹具把参考分成 `pre` / `bag` / `hold` 三段，**只有 `bag` 段对应真实那一场**。
    `bag` 段起点 = 参考首帧时刻 + `ref_hold_s`（`pre` 段顶住首点 30 s）。
    参考首帧的**墙钟**时刻 = 首帧时刻 - `t.first_frame` + `reference_timeline.first_s`。

    ★ 还要再减 `ref_delay_s`：模型参考的时间轴以**首条 `/traj/reference` 的 header**
      为零（`mocap_model.read_reference` 做 `t - t[0]`），而票 01 的 t=0 是
      `/traj/start` 的**记录**时间戳，两者差 7.7–15.6 ms。不减就相当于把
      模型预测控制仿真整体后移约 10 ms（0.5 m/s 下约 5 mm，且与 spec §3 不符）。
      `ref_delay_s` 由同一场景的实飞包给出：`ref_first_stamp_abs - t0_abs`。

    ⚠️ 不用 `move_first_s`（第一帧在动）—— 参考在 `bag` 段前段本来就可以不动
      （悬停整段都不动），拿它当零点会让悬停那趟直接没有窗口。
    """
    wall0 = float(summary["frames"][0]["wall_ns"]) * 1e-9
    t = summary["t"]
    first_ref_abs = wall0 - float(t["first_frame"]) + float(summary["reference_timeline"]["first_s"])
    t0 = first_ref_abs + float(summary["ref_hold_s"]) - float(ref_delay_s)
    why = ("t0 = 首帧墙钟 - t.first_frame + reference_timeline.first_s + ref_hold_s"
           " - 参考发布延迟（首条 /traj/reference header - /traj/start 记录时间）")
    return t0, why


@dataclass
class SimExtract:
    name: str
    summary_path: str
    t0_abs: float
    t0_reason: str
    origin_abs: np.ndarray
    origin_spread_m: float
    ref_t: np.ndarray                 # 绝对
    ref_p: np.ndarray                 # 相对（model 的 p，z ≡ 0）
    ref_v: np.ndarray
    ref_yaw: np.ndarray
    frame_t: np.ndarray
    frame_p_abs: np.ndarray
    frame_v: np.ndarray
    frame_q_wxyz: np.ndarray
    thrust_n: np.ndarray
    rate: np.ndarray
    completion_s: float
    completion_reason: str
    span_s: float
    t_max_n: float
    ref_first_abs: float
    ref_delay_s: float


def read_sim(summary_path: Path, t_max_n: float, ref_delay_s: float) -> SimExtract:
    """读一趟 SITL summary 与其模型参考。

    `x0` 是 DUT 实际吃到的合成动捕（`state_source=mocap`），也就是仿真源的"实际"；
    `truth` 只用于验证器判据，这里不用（两者在 nominal 下同源）。

    `ref_delay_s` 见图 `sim_reference_start`：仿真回放的是同一份包的参考，
    发布延迟要从 t=0 里扣掉。
    """
    summary_path = Path(summary_path)
    d = json.loads(summary_path.read_text(encoding="utf-8"))
    model_path = REPO / d["model"]
    model = json.loads(model_path.read_text(encoding="utf-8"))

    t0, why = sim_reference_start(d, ref_delay_s)
    ref = model["reference"]
    ref_t_rel = np.asarray(ref["t_s"], dtype=float)
    ref_p = np.asarray(ref["p"], dtype=float)
    ref_v = np.asarray(ref["v"], dtype=float)
    ref_yaw = np.asarray(ref.get("yaw") or [0.0] * len(ref_p), dtype=float)
    if ref_p.shape[0] != ref_t_rel.size:
        raise ValueError(f"{summary_path.name}: 参考位置 {ref_p.shape} 与时间轴 {ref_t_rel.shape} 不一致")
    span = float(ref["span_s"])

    frames = d["frames"]
    have = [f for f in frames if f.get("x_ref0") is not None]
    if not have:
        raise ValueError(f"{summary_path.name}: frames 里没有一帧带 x_ref0，无法重建原点")
    x_ref0_first = np.asarray(have[0]["x_ref0"], dtype=float)
    # `pre` 段顶住参考首点：x_ref0 应当恒定。原点 = x_ref0 - p_first。
    pre = np.array([f["x_ref0"] for f in have
                    if float(f["wall_ns"]) * 1e-9 < t0], dtype=float)
    if pre.size == 0:
        pre = x_ref0_first[None, :]
    spread = float(np.abs(pre - x_ref0_first).max())
    origin = x_ref0_first - ref_p[0]

    frame_list = [f for f in frames if f.get("x0") is not None]
    if not frame_list:
        raise ValueError(f"{summary_path.name}: frames 里没有 x0")
    frame_t = np.array([float(f["wall_ns"]) * 1e-9 for f in frame_list], dtype=float)
    x0 = np.array([f["x0"] for f in frame_list], dtype=float)
    if x0.shape[1] < 10:
        raise ValueError(f"{summary_path.name}: x0 只有 {x0.shape[1]} 维，需要 10")
    thrust = np.array([f.get("thrust") for f in frame_list], dtype=float)
    if np.any(~np.isfinite(thrust)):
        raise ValueError(f"{summary_path.name}: frames 缺 thrust 列（旧版 summary？）")
    rate = np.array([[f.get("body_rate_x"), f.get("body_rate_y"), f.get("body_rate_z")]
                     for f in frame_list], dtype=float)
    if np.any(~np.isfinite(rate)):
        raise ValueError(f"{summary_path.name}: frames 缺 body_rate_* 列（旧版 summary？）")

    return SimExtract(
        name=d["name"],
        summary_path=str(summary_path.relative_to(REPO)),
        t0_abs=t0,
        t0_reason=why,
        origin_abs=origin,
        origin_spread_m=spread,
        # 模型参考的 t=0 在首条参考报文头上，比 t0 晚 ref_delay_s。
        ref_t=t0 + float(ref_delay_s) + ref_t_rel,
        ref_p=ref_p, ref_v=ref_v, ref_yaw=ref_yaw,
        ref_first_abs=t0 + float(ref_delay_s),
        frame_t=frame_t,
        frame_p_abs=x0[:, :3],
        frame_v=x0[:, 3:6],
        frame_q_wxyz=x0[:, 6:10],
        thrust_n=thrust * float(t_max_n),
        rate=rate,
        completion_s=span + float(ref_delay_s),
        completion_reason="ref_end",
        span_s=span,
        t_max_n=float(t_max_n),
        ref_delay_s=float(ref_delay_s),
    )


# ------------------------------------------------------------------ 归一化常数（推力）


def parse_launch_line(line: str) -> Dict[str, float]:
    """从 `mpc_node` 启动行里抠 `mass` / `hover_thrust` / `T_max`。

    形状是**固定的**（`mpc_node.py` 的启动日志）：`模型 mass=0.75 kg hover_thrust=0.1589
    ⇒ T_max=46.3027 N`。解析失败直接抛错 —— 静默用错常数会让推力整列偏 0.02%。
    """
    import re
    m = re.search(r"mass=([0-9.]+)\s*kg\s*hover_thrust=([0-9.]+)\s*⇒\s*T_max=([0-9.]+)\s*N",
                  line)
    if not m:
        raise ValueError(f"启动行里找不到 mass/hover_thrust/T_max：{line[:120]}…")
    return {"mass_kg": float(m.group(1)),
            "hover_thrust": float(m.group(2)),
            "t_max_n": float(m.group(3))}


def read_normalization(path: Path) -> dict:
    return json.loads(Path(path).read_text(encoding="utf-8"))
