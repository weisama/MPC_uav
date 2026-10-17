#!/usr/bin/env python3
"""指标：位置误差、姿态误差、控制量统计。纯函数，输入都是公共网格上的数组。

口径（必须与 `docs/report/data/README.md` 一致）：

- 位置误差 = **实际 − 目标**，ENU，逐点；水平误差取 xy 范数，垂直取 |z|；
- 姿态误差 = 两个四元数之间的**最小旋转角**（度），`2·acos(|Δw|)`；
  `|Δw|` 是为了让 `q` 与 `-q` 同姿态、结果相同；
- 控制量统计只对模型预测控制源有意义（offboard 的 PX4 PID 不暴露推力/角速率）。
"""

from __future__ import annotations

import numpy as np

from align import quat_to_euler


def position_error(target_p: np.ndarray, actual_p: np.ndarray) -> dict:
    """位置误差统计（米）。`target_p` / `actual_p` 都是 `(n, 3)` 且同网格。"""
    target_p = np.asarray(target_p, dtype=float)
    actual_p = np.asarray(actual_p, dtype=float)
    if target_p.shape != actual_p.shape or target_p.ndim != 2 or target_p.shape[1] != 3:
        raise ValueError(f"位置数组形状不一致：{target_p.shape} / {actual_p.shape}")
    err = actual_p - target_p
    h = np.linalg.norm(err[:, :2], axis=1)
    z = np.abs(err[:, 2])
    d3 = np.linalg.norm(err, axis=1)
    return {
        "n": int(err.shape[0]),
        "h_mean_m": float(h.mean()),
        "h_rms_m": float(np.sqrt(np.mean(h ** 2))),
        "h_p95_m": float(np.percentile(h, 95)),
        "h_max_m": float(h.max()),
        "z_mean_abs_m": float(z.mean()),
        "z_max_abs_m": float(z.max()),
        "d3_rms_m": float(np.sqrt(np.mean(d3 ** 2))),
        "d3_max_m": float(d3.max()),
    }


def attitude_error_deg(q_ref_wxyz: np.ndarray, q_act_wxyz: np.ndarray) -> dict:
    """姿态误差（度）：总夹角，外加**拆开的倾角与偏航差**。

    ★ 为什么要拆：目标姿态是「水平 + 零偏航」，而实际飞机会为加速/抗阻而倾、SITL 还可能
      带着起机偏航不动。只报一个总夹角，会把「倾角 0.1°、偏航差 90°」读成「姿态控制坏了」；
      拆开后倾角与偏航差各自可与它们的成因分开讨论（成因不在本模块判定）。
    """
    q_ref = _normalize(np.asarray(q_ref_wxyz, dtype=float))
    q_act = _normalize(np.asarray(q_act_wxyz, dtype=float))
    if q_ref.shape != q_act.shape or q_ref.ndim != 2 or q_ref.shape[1] != 4:
        raise ValueError(f"四元数形状应为同形 (n, 4)：{q_ref.shape} / {q_act.shape}")
    # Δq = q_act ⊗ conj(q_ref)；w 分量为 cos(θ/2)。
    w = (q_act * q_ref * np.array([1.0, -1.0, -1.0, -1.0])).sum(axis=1)
    ang = 2.0 * np.arccos(np.clip(np.abs(w), 0.0, 1.0))
    deg = np.degrees(ang)

    # 机体系 z 轴与世界的夹角 = acos(|cos(roll)·cos(pitch)|)；它与偏航无关。
    r_ref, p_ref, y_ref = quat_to_euler(q_ref).T
    r_act, p_act, y_act = quat_to_euler(q_act).T
    # 机体系 z 轴在**去掉偏航**后的表达：R = Rz(yaw)Ry(pitch)Rx(roll)，
    # R·e_z = Rz(yaw)·[cos r·sin p, −sin r, cos r·cos p]。
    # 去掉 Rz 之后，比较的就是"各自机头方向下的倾角向量"，不会被偏航差污染。
    z_ref = np.column_stack([np.cos(r_ref) * np.sin(p_ref), -np.sin(r_ref),
                             np.cos(r_ref) * np.cos(p_ref)])
    z_act = np.column_stack([np.cos(r_act) * np.sin(p_act), -np.sin(r_act),
                             np.cos(r_act) * np.cos(p_act)])
    cos_tilt = np.clip((z_ref * z_act).sum(axis=1), -1.0, 1.0)
    tilt = np.degrees(np.arccos(cos_tilt))
    dyaw = np.degrees(np.abs(np.arctan2(np.sin(y_act - y_ref), np.cos(y_act - y_ref))))

    def _stats(x):
        return {"mean": float(x.mean()), "p95": float(np.percentile(x, 95)),
                "max": float(x.max())}

    return {
        "n": int(deg.size),
        "angle_mean_deg": float(deg.mean()),
        "angle_p95_deg": float(np.percentile(deg, 95)),
        "angle_max_deg": float(deg.max()),
        "tilt": _stats(tilt),
        "yaw_diff": _stats(dyaw),
    }


def control_stats(thrust_n: np.ndarray, rate_radps: np.ndarray) -> dict:
    """推力（N）与机体角速率（rad/s）的统计；NaN 行会被剔除。"""
    thrust = np.asarray(thrust_n, dtype=float).ravel()
    rate = np.asarray(rate_radps, dtype=float)
    if rate.ndim != 2 or rate.shape[1] != 3 or rate.shape[0] != thrust.size:
        raise ValueError(f"控制量形状不一致：thrust {thrust.shape} / rate {rate.shape}")
    ok = np.isfinite(thrust) & np.all(np.isfinite(rate), axis=1)
    thrust, rate = thrust[ok], rate[ok]
    if thrust.size == 0:
        raise ValueError("控制量全是 NaN")
    rate_norm = np.linalg.norm(rate, axis=1)
    return {
        "n": int(thrust.size),
        "thrust_mean_n": float(thrust.mean()),
        "thrust_max_n": float(thrust.max()),
        "thrust_min_n": float(thrust.min()),
        "rate_norm_mean_radps": float(rate_norm.mean()),
        "rate_norm_max_radps": float(rate_norm.max()),
        "rate_absmax_radps": [float(v) for v in np.abs(rate).max(axis=0)],
    }


def _normalize(q: np.ndarray) -> np.ndarray:
    n = np.linalg.norm(q, axis=1, keepdims=True)
    return q / np.where(n > 0, n, 1.0)
