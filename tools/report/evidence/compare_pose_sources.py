#!/usr/bin/env python3
"""票 14：对照 `/mavros/vision_pose/pose`（VP）与 `/mavros/local_position/pose`（LP）。

第 9 章要回答一个容易混的问题：报告里的“实际轨迹”统一用 VP，但两组控制器的
**反馈**不是同一条 —— offboard 档用 LP（EKF，30 Hz），MPC 档用
`/mocap/pose_enu`（原始动捕，约 190 Hz）。本脚本量出 VP 与 LP 的位置差、滚转差
与四元数符号关系，避免把“反馈源差异”算成“算法差异”。

口径：

- 窗口：与票 01 相同 —— `[t0, 首个 /traj/progress >= 1.0)`；没有 1.0 时截止到参考流末尾
  （悬停 offboard 包被截断）。`t0` = `/traj/start` 的包记录时间戳。
- 位置：LP 线性插值到 VP 的时间戳，误差 = VP − LP（ENU，米）；RMS 分开给水平（xy 范数）
  与三维。
- 姿态：欧拉角按 ZYX 内旋；`roll_mean` = VP roll − LP roll。四元数点积 `dot` 记
  `dot < 0` 的比例 —— 同一个姿态允许 `q` 与 `−q` 两种写法。
- 另外记两个时间基准读数：录包起点到 `/traj/start` 的间隔、首条 `/traj/reference`
  的 header 相对 `t0` 的发布延迟。

用法：

    source /opt/ros/humble/setup.bash
    python3 tools/report/evidence/compare_pose_sources.py
"""

from __future__ import annotations

import json
import re
import sys
import tempfile
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[2]
sys.path.insert(0, str(REPO / "tools" / "report"))
sys.path.insert(0, str(REPO / "tools" / "px4-sitl"))

import extract as E  # noqa: E402
from align import quat_to_euler, resample_quat  # noqa: E402

OUT = HERE / "pose-source-comparison.json"
TOPICS = (
    "/traj/start",
    "/traj/progress",
    "/traj/reference",
    "/mavros/vision_pose/pose",
    "/mavros/local_position/pose",
)

#: 场景 ↔ 包。与票 01 的 `build.py` 的 `SCENES` 同一组路径。
CASES = (
    ("hover", "offboard", "bag/3_traj1_hover"),
    ("hover", "mpc_real", "bag/6_free__20261017-070103"),
    ("square", "offboard", "bag/4_traj2_square"),
    ("square", "mpc_real", "bag/6_free__20261017-070446"),
    ("lemniscate", "offboard", "bag/5_traj3_lemniscate"),
    ("lemniscate", "mpc_real", "bag/6_free__20261017-070935"),
)


def _bag_start_abs(bag_dir: Path) -> float:
    """`metadata.yaml` 的 `starting_time`（epoch 秒）。"""
    text = (bag_dir / "metadata.yaml").read_text(encoding="utf-8")
    m = re.search(r"starting_time:\s*\n\s*nanoseconds_since_epoch:\s*(\d+)", text)
    if not m:
        raise ValueError(f"{bag_dir}: metadata.yaml 里找不到 starting_time")
    return int(m.group(1)) * 1e-9


def _track_end(ref_t: np.ndarray, progress_t: np.ndarray, progress: np.ndarray,
               t0: float) -> tuple[float, str]:
    hit = np.flatnonzero(np.asarray(progress) >= 1.0)
    if hit.size:
        return float(progress_t[hit[0]] - t0), "progress"
    return float(ref_t[-1] - t0), "ref_end"


def _interp_columns(t: np.ndarray, x: np.ndarray, tq: np.ndarray) -> np.ndarray:
    return np.column_stack([np.interp(tq, t, x[:, i]) for i in range(x.shape[1])])


def _stats(d: np.ndarray, roll_diff: np.ndarray, dot: np.ndarray) -> dict:
    h = np.linalg.norm(d[:, :2], axis=1)
    d3 = np.linalg.norm(d, axis=1)
    return {
        "n": int(d.shape[0]),
        "position": {
            "h_rms_m": float(np.sqrt(np.mean(h ** 2))),
            "d3_rms_m": float(np.sqrt(np.mean(d3 ** 2))),
            "h_mean_m": float(h.mean()),
            "d3_max_m": float(d3.max()),
            "xyz_rms_m": [float(v) for v in np.sqrt(np.mean(d ** 2, axis=0))],
            "what": "VP − LP（ENU）；LP 线性插值到 VP 的时间戳",
        },
        "attitude": {
            "roll_diff_mean_rad": float(roll_diff.mean()),
            "roll_diff_std_rad": float(roll_diff.std()),
            "quat_dot_negative_fraction": float(np.mean(dot < 0.0)),
            "quat_abs_dot_mean": float(np.mean(np.abs(dot))),
            "what": "roll 差 = VP − LP（ZYX 内旋）；四元数点积在 LP 时间戳上取",
        },
    }


def compare_case(scene: str, kind: str, bag: str, workdir: Path) -> dict:
    bag_dir = REPO / bag
    raw = E.read_topics(bag_dir, workdir, TOPICS)
    if raw["/traj/start"].t.size != 1:
        raise ValueError(f"{bag}: /traj/start 应恰好 1 条，实际 {raw['/traj/start'].t.size}")
    t0 = float(raw["/traj/start"].t[0])
    end, end_reason = _track_end(raw["/traj/reference"].t, raw["/traj/progress"].t,
                                 E._floats(raw["/traj/progress"]), t0)

    vp = raw["/mavros/vision_pose/pose"]
    lp = raw["/mavros/local_position/pose"]
    p_vp, q_vp = E._posestamped(vp)
    p_lp, q_lp = E._posestamped(lp)

    # 位置：LP 插值到 VP 的时间戳。
    p_lp_on_vp = _interp_columns(lp.t, p_lp, vp.t)
    d_pos = p_vp - p_lp_on_vp

    # 姿态：两套时间戳各算一次，避免把插值误差混进时间对齐。
    r_vp = quat_to_euler(q_vp)
    r_lp = quat_to_euler(q_lp)
    r_lp_on_vp = _interp_columns(lp.t, r_lp, vp.t)
    roll_diff = r_vp[:, 0] - r_lp_on_vp[:, 0]
    q_vp_on_lp = resample_quat(vp.t, q_vp, lp.t)
    dot = np.sum(q_vp_on_lp * q_lp, axis=1)

    in_track = (vp.t >= t0) & (vp.t <= t0 + end)
    all_mask = np.ones(vp.t.shape, dtype=bool)
    track_mask_lp = (lp.t >= t0) & (lp.t <= t0 + end)

    return {
        "scene": scene,
        "source": kind,
        "bag": bag,
        "bag_sha256": E.sha256_of_bag(bag_dir),
        "bag_start_to_traj_start_s": t0 - _bag_start_abs(bag_dir),
        "reference_publish_delay_ms": (E._first_stamp_abs(raw["/traj/reference"]) - t0) * 1e3,
        "track_window_s": [0.0, end],
        "track_end_reason": end_reason,
        "track": _stats(d_pos[in_track], roll_diff[in_track], dot[track_mask_lp]),
        "whole_bag": _stats(d_pos[all_mask], roll_diff[all_mask], dot),
    }


def main() -> int:
    out = {
        "what": "VP 与 LP 的对照（票 14，第 9 章）；口径见本脚本文件头。",
        "generated_by": "python3 tools/report/evidence/compare_pose_sources.py",
        "cases": [],
    }
    with tempfile.TemporaryDirectory(prefix="pose-sources-") as tmp:
        for scene, kind, bag in CASES:
            case = compare_case(scene, kind, bag, Path(tmp) / f"{scene}-{kind}")
            out["cases"].append(case)
            t, w = case["track"], case["whole_bag"]
            print(f"[{scene:10s} {kind:9s}] TRACK {case['track_window_s'][1]:7.3f} s | "
                  f"位置 h_rms {t['position']['h_rms_m'] * 100:4.2f} cm / "
                  f"3d_rms {t['position']['d3_rms_m'] * 100:4.2f} cm | "
                  f"roll 差 {np.degrees(t['attitude']['roll_diff_mean_rad']):+5.2f}° "
                  f"(全程 {np.degrees(w['attitude']['roll_diff_mean_rad']):+5.2f}°) | "
                  f"dot<0 {t['attitude']['quat_dot_negative_fraction'] * 100:.0f}%")
    OUT.write_text(json.dumps(out, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(f"已写 {OUT.relative_to(REPO)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
