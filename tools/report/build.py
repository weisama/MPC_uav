#!/usr/bin/env python3
"""票 01：把六份包 + 三趟 SITL 变成 `docs/report/data/<scene>.{csv,json}`。

四源的口径（为什么这么定，见 `docs/report/data/README.md`）：

| 源 | 数据 | 时间零点 | 原点 |
|---|---|---|---|
| `target` | offboard 包的 `/traj/reference`（相对量） | 该包 `/traj/start` | 该包 WAIT_START 指令值 |
| `offboard` | offboard 包 `/mavros/vision_pose/pose` | 同上 | 同上 |
| `mpc_sim` | SITL `x0`（合成动捕） | `bag` 段起点 − 参考发布延迟 | `x_ref0(前摇) − 模型首点` |
| `mpc_real` | 实飞包 `/mavros/vision_pose/pose` | 该包 `/traj/start` | 该包 WAIT_START 的 `/control/target` |

公共窗 = 四源完成时刻的**最小值**；网格 100 Hz；位置/速度线性插值、四元数 slerp、
控制量零阶保持。所有源都减去各自原点（CSV 同时保留绝对列）。

用法：

    source /opt/ros/humble/setup.bash
    python3 tools/report/build.py --scene all
    python3 tools/report/build.py --scene square

推力归一化常数**不跨趟抄**：默认读 `tools/report/evidence/mpc-normalization.json`
（提交版，含每趟启动行原文与日志 sha256）。`run/logs/` 还在时 build 会核对哈希；
要重新采集用 `--refresh-normalization`。
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import sys
import tempfile
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional, Sequence

import numpy as np

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[1]
sys.path.insert(0, str(HERE))

from align import (  # noqa: E402
    HZ,
    absolute_target,
    build_grid,
    common_end,
    euler_to_quat,
    interp_columns,
    quat_to_euler,
    resample_linear,
    resample_quat,
    resample_zoh,
)
from extract import (  # noqa: E402
    BagExtract,
    SimExtract,
    parse_launch_line,
    read_mpc_bag,
    read_normalization,
    read_offboard_bag,
    read_sim,
    sha256_file,
    sha256_of_bag,
)
from metrics import attitude_error_deg, control_stats, position_error  # noqa: E402

OUT_DIR = REPO / "docs" / "report" / "data"
NORM_PATH = HERE / "evidence" / "mpc-normalization.json"
SITL = HERE / "evidence" / "sitl"

COLUMNS = [
    "source", "t_s",
    "x_m", "y_m", "z_m", "x_abs_m", "y_abs_m", "z_abs_m",
    "vx_mps", "vy_mps", "vz_mps",
    "roll_rad", "pitch_rad", "yaw_rad",
    "qw", "qx", "qy", "qz",
    "u_thrust_n", "wx_radps", "wy_radps", "wz_radps", "valid",
]

SOURCE_ORDER = ("target", "offboard", "mpc_sim", "mpc_real")


@dataclass(frozen=True)
class Scene:
    key: str
    label: str
    offboard_bag: str
    mpc_bag: str
    sim_tag: str
    norm_key: str


SCENES = {
    "hover": Scene("hover", "悬停", "bag/3_traj1_hover",
                   "bag/6_free__20261017-070103", "R-hover", "hover"),
    "square": Scene("square", "正方形", "bag/4_traj2_square",
                    "bag/6_free__20261017-070446", "R-square", "square"),
    "lemniscate": Scene("lemniscate", "8 字", "bag/5_traj3_lemniscate",
                        "bag/6_free__20261017-070935", "R-lemniscate", "lemniscate"),
}


# ------------------------------------------------------------------ 归一化证据


def collect_normalization() -> dict:
    """从 `run/logs/` 现场采集三趟的启动行；`run/logs/` 会被 gitignore，故必须落库。"""
    runs = {}
    for key, scene in SCENES.items():
        log = REPO / "run" / "logs" / _log_dir(scene) / "05-mpc.log"
        if not log.is_file():
            raise FileNotFoundError(f"找不到 {log}，无法采集归一化常数")
        lines = log.read_text(encoding="utf-8").splitlines()
        hit = [ln for ln in lines if "模型 mass=" in ln and "T_max=" in ln]
        if not hit:
            raise ValueError(f"{log} 里没有 mpc_node 启动行")
        launch = hit[0].strip()
        runs[key] = {
            "log": str(log.relative_to(REPO)),
            "log_sha256": sha256_file(log),
            "launch_line": launch,
            **parse_launch_line(launch),
        }
    sim_tmax = _sim_t_max("R-hover")
    return {
        "什么是": "模型预测控制源的 `u_thrust_n = thrust × t_max` 里那个 `t_max` 的出处。"
                 "`run/logs/` 被 gitignore，所以把启动行**原文**与日志 sha256 摘到这里。",
        "口径": "t_max 取**控制器启动行**（`mpc_node` 算出来的 T_max），"
               "不取被控对象 `plant_hover.t_max_total_n`（两者差 1e-4 N）。",
        "runs": runs,
        "sim": {"t_max_n": sim_tmax,
                "source": "tools/report/evidence/sitl/R-*/harness-identity.json 的"
                          "「控制器参数（DUT 实跑值）」"},
    }


def _log_dir(scene: Scene) -> str:
    return {
        "hover": "20261017-070004-up",
        "square": "20261017-070344-up",
        "lemniscate": "20261017-070834-up",
    }[scene.key]


def _sim_t_max(tag: str) -> float:
    ident = json.loads((SITL / tag / "harness-identity.json").read_text(encoding="utf-8"))
    return float(ident["控制器参数（DUT 实跑值；DUT 未用命令行覆盖任何一项）"]["t_max_n"])


def load_normalization(refresh: bool) -> dict:
    if refresh:
        fresh = collect_normalization()
        NORM_PATH.parent.mkdir(parents=True, exist_ok=True)
        NORM_PATH.write_text(json.dumps(fresh, ensure_ascii=False, indent=2) + "\n",
                             encoding="utf-8")
        return fresh
    if not NORM_PATH.is_file():
        raise FileNotFoundError(
            f"缺 {NORM_PATH}。`run/logs/` 还在时跑一次 `--refresh-normalization` 生成它。")
    norm = read_normalization(NORM_PATH)
    # 日志还在 ⟹ 核对它没被改过；不在 ⟹ 用提交版（这正是摘录的理由）。
    for key, run in norm["runs"].items():
        log = REPO / run["log"]
        if log.is_file():
            got = sha256_file(log)
            if got != run["log_sha256"]:
                raise ValueError(
                    f"归一化证据与现场日志不一致：{run['log']}\n"
                    f"  证据 {run['log_sha256']}\n  现场 {got}")
    return norm


# ------------------------------------------------------------------ 源装配


@dataclass
class SourceSeries:
    """一个源在公共网格上的全部列。"""

    name: str
    t_abs: np.ndarray            # 网格的绝对时刻
    p_abs: np.ndarray
    v: np.ndarray
    q_wxyz: np.ndarray
    u_thrust_n: np.ndarray       # 可为 NaN
    rate: np.ndarray             # 可为 NaN
    valid: np.ndarray
    info: dict


def _grid_abs(t0_abs: float, grid_rel: np.ndarray) -> np.ndarray:
    return t0_abs + grid_rel


def _valid_mask(grid_abs: np.ndarray, t_first: float, t_last: float) -> np.ndarray:
    return ((grid_abs >= t_first) & (grid_abs <= t_last)).astype(int)


def _attitude_from_yaw(yaw: np.ndarray) -> np.ndarray:
    e = np.column_stack([np.zeros_like(yaw), np.zeros_like(yaw), yaw])
    return euler_to_quat(e)


def build_target(e: BagExtract, grid_abs: np.ndarray) -> SourceSeries:
    """目标源：`/traj/reference`（相对）→ 绝对 = 原点 + 参考。"""
    p_abs = resample_linear(e.ref_t, absolute_target(e.origin_abs, e.ref_p), grid_abs)
    v = resample_linear(e.ref_t, e.ref_v, grid_abs)
    q = _attitude_from_yaw(resample_zoh(e.ref_t, e.ref_yaw[:, None], grid_abs)[:, 0])
    nan = np.full(grid_abs.shape, np.nan)
    return SourceSeries(
        name="target", t_abs=grid_abs, p_abs=p_abs, v=v, q_wxyz=q,
        u_thrust_n=nan, rate=np.full((grid_abs.size, 3), np.nan),
        valid=_valid_mask(grid_abs, e.ref_t[0], e.ref_t[-1]),
        info={
            "feedback_source": "无（这是目标，不是测量）",
            "position_source": f"{e.bag_dir}/traj/reference（相对量）+ 原点",
            "attitude_source": "由 /traj/reference.yaw 生成的水平姿态（roll=pitch=0）",
            "velocity_source": "/traj/reference.velocity",
        })


def build_offboard(e: BagExtract, grid_abs: np.ndarray) -> SourceSeries:
    p_abs = resample_linear(e.actual_t, e.actual_p_abs, grid_abs)
    q = resample_quat(e.actual_t, e.actual_q_wxyz, grid_abs)
    # 速度与位姿是两条话题，时间轴不同 —— 各自插值。
    v = resample_linear(e.vel_t, e.actual_v, grid_abs)
    return SourceSeries(
        name="offboard", t_abs=grid_abs, p_abs=p_abs, v=v, q_wxyz=q,
        u_thrust_n=np.full(grid_abs.shape, np.nan),
        rate=np.full((grid_abs.size, 3), np.nan),
        valid=_valid_mask(grid_abs, e.actual_t[0], e.actual_t[-1]),
        info={
            "feedback_source": e.feedback_source,
            "position_source": "/mavros/vision_pose/pose",
            "attitude_source": "/mavros/vision_pose/pose（约 190 Hz；不是 30 Hz 的 EKF 姿态）",
            "velocity_source": e.velocity_source,
        })


def build_mpc_real(e: BagExtract, grid_abs: np.ndarray, t_max_n: float) -> SourceSeries:
    p_abs = resample_linear(e.actual_t, e.actual_p_abs, grid_abs)
    q = resample_quat(e.actual_t, e.actual_q_wxyz, grid_abs)
    v = resample_linear(e.vel_t, e.actual_v, grid_abs)
    u = resample_zoh(e.u_t, (e.u_thrust_n * float(t_max_n))[:, None], grid_abs)[:, 0]
    rate = resample_zoh(e.u_t, e.u_rate, grid_abs)
    return SourceSeries(
        name="mpc_real", t_abs=grid_abs, p_abs=p_abs, v=v, q_wxyz=q,
        u_thrust_n=u, rate=rate,
        valid=_valid_mask(grid_abs, e.actual_t[0], e.actual_t[-1]),
        info={
            "feedback_source": e.feedback_source,
            "position_source": "/mavros/vision_pose/pose",
            "attitude_source": "/mavros/vision_pose/pose",
            "velocity_source": e.velocity_source,
            "u_conversion": {"formula": "u_thrust_n = thrust × t_max",
                             "t_max_n": float(t_max_n),
                             "source": "该趟 mpc_node 启动行（见 mpc-normalization.json）"},
        })


def build_mpc_sim(e: SimExtract, grid_abs: np.ndarray) -> SourceSeries:
    p_abs = resample_linear(e.frame_t, e.frame_p_abs, grid_abs)
    q = resample_quat(e.frame_t, e.frame_q_wxyz, grid_abs)
    v = resample_linear(e.frame_t, e.frame_v, grid_abs)
    u = resample_zoh(e.frame_t, e.thrust_n[:, None], grid_abs)[:, 0]
    rate = resample_zoh(e.frame_t, e.rate, grid_abs)
    return SourceSeries(
        name="mpc_sim", t_abs=grid_abs, p_abs=p_abs, v=v, q_wxyz=q,
        u_thrust_n=u, rate=rate,
        valid=_valid_mask(grid_abs, e.frame_t[0], e.frame_t[-1]),
        info={
            "feedback_source": "SITL 合成 mocap（state_source=mocap），即 x0",
            "position_source": "summary.frames[].x0[0:3]",
            "attitude_source": "summary.frames[].x0[6:10]（wxyz）",
            "velocity_source": "summary.frames[].x0[3:6]",
            "u_conversion": {"formula": "u_thrust_n = thrust × t_max",
                             "t_max_n": e.t_max_n,
                             "source": "harness-identity.json「控制器参数」"},
        })


# ------------------------------------------------------------------ 公共窗 / 指标


def _target_identity(e: BagExtract) -> dict:
    """在 TRACK 窗内核对 `指令 ≈ 原点 + 参考`，返回误差读数（米）。"""
    lo, hi = e.t0_abs, e.t0_abs + e.completion_s
    mask = (e.cmd_t >= lo) & (e.cmd_t <= hi)
    ct = e.cmd_t[mask]
    if ct.size == 0:
        raise ValueError(f"{e.name}: TRACK 窗内没有指令样本")
    ref_at = interp_columns(e.ref_t, e.ref_p, ct)
    err = np.linalg.norm(e.cmd_p[mask] - absolute_target(e.origin_abs, ref_at), axis=1)
    return {
        "n": int(ct.size),
        "mean_m": float(err.mean()),
        "p99_m": float(np.percentile(err, 99)),
        "max_m": float(err.max()),
        "口径": "指令话题（offboard=/mavros/setpoint_raw/local，模型预测控制=/control/target）"
                "与「原点 + /traj/reference」逐样本比；参考流有缺口时用线性插值补齐。",
    }


def _sim_target_identity(e: SimExtract) -> dict:
    """仿真：TRACK 窗内，`x_ref0`（DUT 收到的绝对参考）与「原点 + 模型参考」比。

    ⚠️ 只取 `[t0, t0 + completion_s]`：`pre` / `hold` 是回放夹具加的前摇与尾段，
      把它们混进来会让读数带上不相干相位（票 01 只要求在 TRACK 窗内验证）。
    """
    frames = [f for f in json.loads((REPO / e.summary_path).read_text(encoding="utf-8"))["frames"]
              if f.get("x_ref0") is not None]
    ft_all = np.array([float(f["wall_ns"]) * 1e-9 for f in frames])
    xr_all = np.array([f["x_ref0"] for f in frames], dtype=float)
    mask = (ft_all >= e.t0_abs) & (ft_all <= e.t0_abs + e.completion_s)
    ft, xr = ft_all[mask], xr_all[mask]
    if ft.size == 0:
        raise ValueError(f"{e.name}: TRACK 窗内没有带 x_ref0 的帧")
    ref_at = interp_columns(e.ref_t, e.ref_p, ft)
    err = np.linalg.norm(xr - absolute_target(e.origin_abs, ref_at), axis=1)
    return {
        "n": int(ft.size),
        "win_s": [0.0, float(e.completion_s)],
        "mean_m": float(err.mean()),
        "p99_m": float(np.percentile(err, 99)),
        "max_m": float(err.max()),
        "口径": "TRACK 窗内 x_ref0 与「原点 + 模型参考」逐帧比（帧间线性插值）。",
    }


#: `/traj/reference` 的标称周期（50 Hz）。20 ms 附近的抖动是常态，不算“没发”。
REF_NOMINAL_S = 0.02
#: 超过标称周期的这个倍数才记成缺口。
REF_GAP_FACTOR = 2.0


def _ref_gaps(t: np.ndarray) -> dict:
    """参考流相邻样本的时间间隔（秒）。

    标称 50 Hz（`REF_NOMINAL_S`），所以 20 ms 附近的抖动是常态；`n_over_40ms`
    只数**两个标称周期以上**的缺口，才是指令真的“没发”的那几处。
    """
    d = np.diff(np.asarray(t, dtype=float))
    if d.size == 0:
        return {"n": 0}
    return {
        "n": int(d.size),
        "p50_s": float(np.percentile(d, 50)),
        "p99_s": float(np.percentile(d, 99)),
        "max_s": float(d.max()),
        "n_over_40ms": int(np.sum(d > REF_NOMINAL_S * REF_GAP_FACTOR)),
    }


def _rel(series: SourceSeries, origin: np.ndarray) -> np.ndarray:
    return series.p_abs - origin[None, :]


def _source_metrics(series: SourceSeries, origin: np.ndarray,
                    target: SourceSeries, target_origin: np.ndarray) -> dict:
    rel = _rel(series, origin)
    tgt = _rel(target, target_origin)
    out = {"position": position_error(tgt, rel)}
    out["attitude"] = attitude_error_deg(target.q_wxyz, series.q_wxyz)
    if np.any(np.isfinite(series.u_thrust_n)):
        out["control"] = control_stats(series.u_thrust_n, series.rate)
    return out


# ------------------------------------------------------------------ 窗口口径


def _completion_reason(name: str, off: BagExtract, mpc: BagExtract, sim: SimExtract) -> str:
    return {"target": off.completion_reason, "offboard": off.completion_reason,
            "mpc_real": mpc.completion_reason, "mpc_sim": sim.completion_reason}[name]


def _source_trim_reason(name: str, off: BagExtract, mpc: BagExtract,
                        sim: SimExtract) -> str:
    """每个源为什么在这个时刻结束 —— 写进 JSON，别让人只从 README 猜。"""
    if name in ("target", "offboard"):
        if off.completion_reason == "ref_end":
            return (f"录制截断：progress 终值 {float(off.progress[-1]):.3f} < 1.0；"
                    f"截止到参考流末尾 {off.completion_s:.6f} s"
                    f"（包尾 {off.package_end_s:.6f} s）")
        return f"`/traj/progress` 首次 ≥ 1.0：{off.completion_s:.6f} s"
    if name == "mpc_real":
        return f"`/traj/progress` 首次 ≥ 1.0：{mpc.completion_s:.6f} s"
    return (f"模型参考末帧：{sim.completion_s:.6f} s"
            f"（= span {sim.span_s:.6f} s + 参考发布延迟 {sim.ref_delay_s * 1e3:.2f} ms）")


# ------------------------------------------------------------------ 场景构建


def build_scene(scene: Scene, norm: dict, out_dir: Path, work_root: Path) -> dict:
    work = Path(work_root) / scene.key
    off = read_offboard_bag(REPO / scene.offboard_bag, work / "offboard")
    mpc = read_mpc_bag(REPO / scene.mpc_bag, work / "mpc")
    sim_summary = SITL / scene.sim_tag / f"{scene.sim_tag}-summary.json"
    t_max_real = float(norm["runs"][scene.norm_key]["t_max_n"])
    sim_t_max = float(norm["sim"]["t_max_n"])
    if abs(sim_t_max - _sim_t_max(scene.sim_tag)) > 1e-12:
        raise ValueError(f"{scene.sim_tag}: 归一化证据的 t_max 与 harness-identity 不一致")
    # 仿真回放的是同一份包的 /traj/reference；用实飞包量出的发布延迟把
    # sim 的 t=0 从"首条参考 header"退回"/traj/start 记录时间戳"。
    ref_delay_s = float(off.ref_first_stamp_abs - off.t0_abs)
    sim = read_sim(sim_summary, sim_t_max, ref_delay_s)

    # 目标的 TRACK 窗与 offboard 同包同语义（不是参考流末帧：progress=1.0 之后
    # 参考还会继续发零，拿末帧会把 target 的窗口拉长到降落段）。
    ends = {
        "target": off.completion_s,
        "offboard": off.completion_s,
        "mpc_sim": sim.completion_s,
        "mpc_real": mpc.completion_s,
    }
    end_s = common_end(ends)
    winner = min(ends, key=ends.get)
    grid_rel = build_grid(end_s, HZ)

    sources: Dict[str, SourceSeries] = {}
    origins = {}
    # 目标与 offboard 共用同一个 t0 / 原点（同一份包）。
    sources["target"] = build_target(off, _grid_abs(off.t0_abs, grid_rel))
    origins["target"] = off.origin_abs
    sources["offboard"] = build_offboard(off, _grid_abs(off.t0_abs, grid_rel))
    origins["offboard"] = off.origin_abs
    sources["mpc_sim"] = build_mpc_sim(sim, _grid_abs(sim.t0_abs, grid_rel))
    origins["mpc_sim"] = sim.origin_abs
    sources["mpc_real"] = build_mpc_real(mpc, _grid_abs(mpc.t0_abs, grid_rel), t_max_real)
    origins["mpc_real"] = mpc.origin_abs

    _check_columns(sources)
    readings = {
        "target_identity": {
            "offboard_bag": _target_identity(off),
            "mpc_bag": _target_identity(mpc),
            "mpc_sim": _sim_target_identity(sim),
        },
        "relative_at_t0": {
            name: [float(v) for v in (_rel(s, origins[name])[0])] for name, s in sources.items()
        },
        "reference_gaps_s": {
            "offboard_bag": _ref_gaps(off.ref_t),
            "mpc_bag": _ref_gaps(mpc.ref_t),
            "mpc_sim": _ref_gaps(sim.ref_t),
        },
    }

    meta = {
        "scene": scene.key,
        "label": scene.label,
        "generated_by": f"python3 tools/report/build.py --scene {scene.key}",
        "grid": {"hz": HZ, "n": int(grid_rel.size),
                 "common_end_s": float(end_s),
                 "common_end_reason": f"四源完成时刻取最小：{winner} = {end_s:.6f} s",
                 "window_trim_reason": (
                     f"公共窗右端取各源最小：{winner} = {end_s:.6f} s；"
                     + _source_trim_reason("offboard", off, mpc, sim)),
                 "source_ends_s": ends},
        "sources": {},
        "inputs": {
            "offboard_bag": _bag_inputs(REPO / scene.offboard_bag),
            "mpc_bag": _bag_inputs(REPO / scene.mpc_bag),
            "sim_summary": _file_inputs(sim_summary),
            "sim_model": _file_inputs(REPO / json.loads(
                sim_summary.read_text(encoding="utf-8"))["model"]),
            "normalization": _file_inputs(NORM_PATH),
        },
        "readings": readings,
    }

    for name in SOURCE_ORDER:
        s = sources[name]
        entry = {
            "t0_abs": _source_t0(name, off, mpc, sim),
            "origin_abs": [float(v) for v in origins[name]],
            "completion_s": float(ends[name]),
            "completion_reason": _completion_reason(name, off, mpc, sim),
            "window_trim_reason": _source_trim_reason(name, off, mpc, sim),
            "valid_fraction": float(s.valid.mean()),
            **s.info,
        }
        if name == "target":
            entry["origin_source"] = f"{off.origin_source}（散布 {off.origin_spread_m:.1e} m）"
        elif name == "offboard":
            entry["origin_source"] = f"{off.origin_source}（散布 {off.origin_spread_m:.1e} m）"
            entry["origin_window_abs"] = [float(v) for v in off.origin_window_abs]
        elif name == "mpc_real":
            entry["origin_source"] = f"{mpc.origin_source}（散布 {mpc.origin_spread_m:.1e} m）"
            entry["origin_window_abs"] = [float(v) for v in mpc.origin_window_abs]
        else:
            entry["origin_source"] = ("x_ref0（前摇段恒定值）− 模型参考首点；"
                                      "等价于锚点状态 + 参考 z 偏移（见 README）")
            entry["origin_spread_m"] = float(sim.origin_spread_m)
            entry["t0_reason"] = sim.t0_reason
        if name != "target":
            entry["metrics_vs_target"] = _source_metrics(
                s, origins[name], sources["target"], origins["target"])
        meta["sources"][name] = entry

    # CSV
    rows = []
    for name in SOURCE_ORDER:
        s = sources[name]
        euler = quat_to_euler(s.q_wxyz)
        rel = _rel(s, origins[name])
        for i in range(grid_rel.size):
            rows.append([
                name, float(grid_rel[i]),
                *rel[i], *s.p_abs[i], *s.v[i], *euler[i],
                *s.q_wxyz[i], float(s.u_thrust_n[i]), *s.rate[i],
                int(s.valid[i]),
            ])
    out_dir.mkdir(parents=True, exist_ok=True)
    _write_csv(out_dir / f"{scene.key}.csv", rows)
    (out_dir / f"{scene.key}.json").write_text(
        json.dumps(meta, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    return meta


def _source_t0(name: str, off: BagExtract, mpc: BagExtract, sim: SimExtract) -> float:
    return {"target": off.t0_abs, "offboard": off.t0_abs,
            "mpc_sim": sim.t0_abs, "mpc_real": mpc.t0_abs}[name]


def _check_columns(sources: Dict[str, SourceSeries]) -> None:
    """缺列/缺源直接失败 —— 不静默补 0。"""
    missing = [n for n in SOURCE_ORDER if n not in sources]
    if missing:
        raise ValueError(f"缺源：{missing}")
    for name, s in sources.items():
        n = s.t_abs.size
        for field_name in ("p_abs", "v", "q_wxyz", "valid"):
            arr = getattr(s, field_name)
            if arr.shape[0] != n:
                raise ValueError(f"{name}: {field_name} 行数 {arr.shape[0]} ≠ {n}")
        if s.p_abs.shape[1] != 3 or s.v.shape[1] != 3 or s.q_wxyz.shape[1] != 4:
            raise ValueError(f"{name}: 列宽不对 {s.p_abs.shape}/{s.v.shape}/{s.q_wxyz.shape}")
        # 允许 NaN（目标源没有 u、offboard 没有控制量），但不允许整列形状错。
        if s.rate.shape != (n, 3):
            raise ValueError(f"{name}: rate 形状 {s.rate.shape} 应为 {(n, 3)}")


def _write_csv(path: Path, rows: List[list]) -> None:
    with path.open("w", encoding="utf-8", newline="") as f:
        w = csv.writer(f)
        w.writerow(COLUMNS)
        for row in rows:
            w.writerow([_fmt(v) for v in row])


def _fmt(v) -> str:
    if isinstance(v, str):
        return v
    if v is None:
        return ""
    f = float(v)
    if not math.isfinite(f):
        return ""
    return f"{f:.9g}"


def _file_inputs(path: Path) -> dict:
    path = Path(path)
    return {"path": str(path.relative_to(REPO)), "sha256": sha256_file(path),
            "size": path.stat().st_size}


def _bag_inputs(path: Path) -> dict:
    path = Path(path)
    files = {str(p.relative_to(REPO)): {"sha256": sha256_file(p), "size": p.stat().st_size}
             for p in sorted(path.iterdir()) if p.is_file()}
    return {"path": str(path.relative_to(REPO)), "sha256": sha256_of_bag(path), "files": files}


# ------------------------------------------------------------------ README


def write_readme(metas: Dict[str, dict], out_dir: Path) -> None:
    lines = [
        "# 报告数据（Phase 5 票 01）",
        "",
        "> 本目录由 `tools/report/build.py` 生成，**不要手改**。",
        "> 生成命令：`source /opt/ros/humble/setup.bash && python3 tools/report/build.py --scene all`",
        "",
        "每条数据都能回到同名的 `.json`：窗口、原点、输入 sha256、读数都在那里。",
        "",
        "## 四源",
        "",
        "| `source` | 是什么 | 数据来源 | 控制器反馈 |",
        "|---|---|---|---|",
        "| `target` | 参考目标 | offboard 包 `/traj/reference`（相对量）+ 轨迹原点 | 无（目标） |",
        "| `offboard` | PX4 内置 PID 实飞 | offboard 包 `/mavros/vision_pose/pose` | `/mavros/local_position/pose`（EKF 30 Hz） |",
        "| `mpc_sim` | 模型预测控制在 SITL 里闭环 | 票 02 的 `R-*` summary，`x0`（合成动捕） | SITL 合成 mocap |",
        "| `mpc_real` | 模型预测控制实飞 | 实飞包 `/mavros/vision_pose/pose` | `/mocap/pose_enu`（原始动捕） |",
        "",
        "⚠️ 报告里的“实际轨迹”统一用 `/mavros/vision_pose/pose`，但 offboard 与模型预测控制两组的",
        "**控制器反馈不是同一条话题**（前者 EKF 30 Hz，后者原始动捕）。",
        "",
        "## 窗口",
        "",
        "| 场景 | 公共窗终点 | 由谁决定 | 网格点数 |",
        "|---|---|---|---|",
    ]
    for key, m in metas.items():
        g = m["grid"]
        winner = min(m["grid"]["source_ends_s"], key=m["grid"]["source_ends_s"].get)
        lines.append(f"| {m['label']}（`{key}`） | {g['common_end_s']:.4f} s | `{winner}` | {g['n']} |")
    lines += [
        "",
        "公共窗 = 四源完成时刻的**最小值**。完成时刻的口径：",
        "",
        "- 实飞：`/traj/progress` **首次** ≥ 1.0（之后参考继续发，不能取末帧）；",
        "- 悬停 offboard 包录到 TRACK 中途截断（progress 未到 1.0）⟹ 截止到参考流末尾；",
        "- 仿真：模型参考的 `span_s`（`bag` 段；`pre`/`hold` 是回放夹具加的，不进窗口）。",
        "",
        "每个源为什么在这个时刻结束，见 `.json` 的 `sources.<源>.window_trim_reason`；",
        "仿真 t=0 还扣掉了参考发布延迟（`ref_t[0] − t0_abs` 为 7.7–10.2 ms）。",
        "",
        "## 列",
        "",
        "| 列 | 单位 | 口径 |",
        "|---|---|---|",
        "| `source` | — | `target` / `offboard` / `mpc_sim` / `mpc_real` |",
        "| `t_s` | s | 相对各自 `/traj/start`（仿真 = 回放 `bag` 段起点 − 参考发布延迟） |",
        "| `x_m`,`y_m`,`z_m` | m | 已减各自轨迹原点（ENU） |",
        "| `x_abs_m`,`y_abs_m`,`z_abs_m` | m | 未减原点（便于回包核对） |",
        "| `vx_mps`,`vy_mps`,`vz_mps` | m/s | 目标=参考速度；实飞=EKF `velocity_local`；仿真=`x0[3:6]` |",
        "| `roll_rad`,`pitch_rad`,`yaw_rad` | rad | ZYX 内旋，由 `qw..qz` 换算 |",
        "| `qw`,`qx`,`qy`,`qz` | — | **wxyz**（与 PX4 一致） |",
        "| `u_thrust_n` | N | 仅模型预测控制源：`thrust × t_max`；`t_max` 逐趟见 `.json` |",
        "| `wx_radps`,`wy_radps`,`wz_radps` | rad/s | 仅模型预测控制源：机体角速率指令 |",
        "| `valid` | 0/1 | 该网格点是否落在该源原始数据的时间范围内；0 = 端点保持值 |",
        "",
        "网格 100 Hz：位置/速度线性插值，四元数沿最短弧 slerp，控制量**零阶保持**。",
        "",
        "## 原点（最容易搞错的一条）",
        "",
        "轨迹原点取 **`AUTO/WAIT_START` 期间指令的恒定值**，不是 `/traj/start` 时刻的飞机位置：",
        "FSM 在稳定判据成立时就锁存原点，两者实测差 6.5–10.5 cm。⛔ 用后者会引入系统偏差。",
        "",
        "- offboard：`/mavros/setpoint_raw/local`；模型预测控制：`/control/target`；",
        "- 仿真：`x_ref0（前摇）− 模型参考首点`，等价于「DUT 锚点状态 + 参考 z 偏置 1.0 m」；",
        "- 所有源减去各自原点后再写 `x_m` 等相对列。",
        "",
        "## 已知坑",
        "",
        "1. **目标不是逐点等于指令**：`/traj/reference` 标称 50 Hz，20 ms 附近的抖动是常态；",
        "   缺口内指令保持上一帧。中断多长见 `.json` 的 `readings.reference_gaps_s`",
        "   （看 `n_over_40ms` / `max_s`，别看 20 ms 那个）；`readings.target_identity`",
        "   记了 p99 / max，不要把 max 当成“公式不成立”。",
        "2. **仿真 `pre` 段是我们加的**：回放夹具多放了 30 s 前摇（顶住参考首点）。",
        "   它不进入公共窗；仿真 t=0 取 `bag` 段起点再减参考发布延迟。",
        "3. **`run/logs/` 会被 gitignore**：推力归一化常数与启动行原文摘在",
        "   `tools/report/evidence/mpc-normalization.json`；日志还在时 build 会核对 sha256。",
        "4. **推力常数不跨趟抄**：每趟的 `t_max` 来自该趟 `mpc_node` 启动行；",
        "   仿真用 `harness-identity.json` 的控制器参数。",
        "5. **悬停 offboard 包是截断录制**：没有 `progress=1.0`，完成时刻=参考流末尾；",
        "   终值与包尾差多少见 `.json` 的 `sources.offboard.window_trim_reason`。",
        "6. **目标姿态是“水平 + 零偏航”**（控制器自己的约定，`node_core.py` 的 `traj_reference`），",
        "   不是由加速度反解的“理想倾角”。飞机为了加速/抗阻会倾，所以实际倾角与 0 的差",
        "   **不是**姿态控制误差；第 8 章解释了为什么这个模型不需要倾角前馈。",
        "7. **仿真偏航约 90°，目标 0°**：仿真源首帧 `x0` 的偏航就在 90° 附近，整场没转到 0。",
        "   这是数据里的真实读数，**本票不判定原因**；姿态图 yaw 那一行会看到一条常值偏置，",
        "   别把它读成控制失效。`metrics_vs_target.attitude` 已把总夹角、倾角、偏航差**分开**给了。",
        "8. **offboard 实测误差可与验收文档对表**（独立工具、50 Hz、`/mocap/pose_enu`）：",
        "   `docs/control/ladder-acceptance-criteria.md` §2.2 的悬停 0.1049/0.1610、",
        "   正方形 0.5682/0.7552、8 字 0.5886/0.8160（本次生成值见下表，差 < 3%）。",
        "",
        "| 场景 | offboard 水平均值 / 最大（m） |",
        "|---|---|",
    ]
    for key, m in metas.items():
        pos = m["sources"]["offboard"]["metrics_vs_target"]["position"]
        lines.append(f"| {m['label']}（`{key}`） | {pos['h_mean_m']:.3f} / {pos['h_max_m']:.3f} |")
    lines += [
        "",
    ]
    (out_dir / "README.md").write_text("\n".join(lines), encoding="utf-8")


# ------------------------------------------------------------------ CLI


def main(argv: Optional[Sequence[str]] = None) -> int:
    ap = argparse.ArgumentParser(description="Phase 5 报告数据管线（票 01）")
    ap.add_argument("--scene", default="all",
                    choices=["all", *SCENES], help="场景（默认 all）")
    ap.add_argument("--out-dir", default=str(OUT_DIR), help="输出目录")
    ap.add_argument("--refresh-normalization", action="store_true",
                    help="从 run/logs/ 重新采集推力归一化常数（日志已清空时不要用）")
    args = ap.parse_args(argv)

    norm = load_normalization(args.refresh_normalization)
    out_dir = Path(args.out_dir)
    keys = list(SCENES) if args.scene == "all" else [args.scene]
    metas = {}
    # 解包目录放 /tmp 并在结束时删掉：仓库里只新增 docs/report/data/**。
    with tempfile.TemporaryDirectory(prefix="report-data-") as tmp:
        for key in keys:
            meta = build_scene(SCENES[key], norm, out_dir, Path(tmp))
            metas[key] = meta
            g = meta["grid"]
            print(f"[{key}] {g['n']} 点 / {g['common_end_s']:.4f} s，"
                  f"终点由 {min(g['source_ends_s'], key=g['source_ends_s'].get)} 决定")
    # README 始终列**全部已生成**的场景：单场景重跑不该让另外两行消失。
    for key in SCENES:
        path = out_dir / f"{key}.json"
        if key not in metas and path.is_file():
            metas[key] = json.loads(path.read_text(encoding="utf-8"))
    ordered = {k: metas[k] for k in SCENES if k in metas}
    write_readme(ordered, out_dir)
    print(f"已写 {out_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
