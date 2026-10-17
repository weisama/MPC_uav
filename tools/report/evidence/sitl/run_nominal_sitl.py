#!/usr/bin/env python3
"""跑票 02 的三趟 nominal SITL —— `verify_tier_b_seam3.py` 的**最小包装**。

**为什么需要包装**：SITL 驱动器 `tools/px4-sitl/verify_tier_b_seam3.py` 最后一次修改在
Phase 3（`7dd7133`，2026-10-09），**早于** Phase 4 票 13 把 DUT 的下行从 px4_msgs/uXRCE-DDS
换到 mavros（2026-10-13）。换载体之后，**命令面**要另起真 mavros（外部前置，不在本脚本里），
另有**三处接线/列名**要在包装里补（本脚本只做这三处）：

1. **命令面（外部前置）**：DUT 发 `/mavros/setpoint_raw/attitude`，由 mavros 转成
   `SET_ATTITUDE_TARGET`。所以跑 SITL 时必须**另起一个真 mavros**
   （`ros2 launch bringup mavros.launch.py fcu_url:=udp://:14540@127.0.0.1:14580`，
   见 `run/z0.py` 的 `mavros` 动作）。本脚本不负责起它。
2. **进 OFFBOARD 的前置流**：PX4 只认 `SET_ATTITUDE_TARGET`（由它自己派生
   OffboardControlMode），而 DUT 的闸门又要求 `mode == "OFFBOARD"` ⟹ 两边互等。
   实飞那一档靠 `offboard_prestream:=true` 破局（`run/z0.py` 的模型预测控制档就是这么起的；
   见 `operation_manual.md` §3.1″ 与 `src/nmpc/README.md` 的 `offboard_prestream` 行）。
   本包装只把这一条**补进 DUT 参数**，与实飞一致；不改闸门、不改控制器。
3. **逐帧 `nav_state` 这一列**：票 13 换载体后 DUT 的 JSONL 写的是 `mode`（**字符串**，
   `"OFFBOARD"`）而不是 `nav_state`（整数 14）—— 这正是 `mpc_node.py:1460-1465` 的 ☠️
   警告过的那一类。`verify_tier_b_seam3.py:913` 的「不失控」主判读 `f.get("nav_state")`
   ⟹ 在新日志里**恒为 None**，判据② 会结构性地读成「不失控 False」（2026-10-17 首跑实测：
   窗口内 793 帧、`mode` 全是 `"OFFBOARD"`，而 `offboard_kept=False`）。
   本包装在 `Dut.frames()` 后把 `mode`/`armed` **合成**回 `nav_state`/`arming_state`
   两个旧列（`OFFBOARD` ⟹ 14），只补判据层要读的键，不改原始 JSONL。
4. **summary 里少掉的新载体列**：`FRAME_KEYS` 还写着旧载体的 `thrust_body_z`，而票 13 之后
   DUT 记的是 `thrust`（归一化）+ `body_rate_x/y/z` ⟹ summary 里这三列**全空**，
   而票 01 的 `u_thrust_n` 正是要从 `thrust` 换算（`.scratch/phase5/issues/01-data-pipeline.md`
   「CSV 列」那节）。本包装把 `thrust` / `body_rate_*` / `mode` **追加**进 `FRAME_KEYS`，
   让 summary 自带这些列；不改 `verify_tier_b_seam3.py` 本身。

⚠️ 本包装**不改** `verify_tier_b_seam3.py`：那份文件被归档证据与别的票共用，本轮不在它的
修改面上（票 02 清单也只允许新增 `tools/report/evidence/sitl/**`）。`obs["params"]` 里会留下
补进去的那一条 ⟹ 事后可核。

用法（先 source 工作空间，并在**另一个终端**起好 mavros）：

    source /opt/ros/humble/setup.bash
    source ~/mavros_ws/install/setup.bash
    source ~/px4_msgs_ws/install/setup.bash
    source install/setup.bash
    python3 tools/report/evidence/sitl/run_nominal_sitl.py \
        --tag R-hover --model tools/report/evidence/sitl/models/R-hover-model.json \
        --reference-source traj --mismatch none --hold-s 78 --ref-hold-s 30 \
        --outdir /tmp/phase5-sitl/R-hover \
        --json tools/report/evidence/sitl/R-hover/R-hover-summary.json

其余参数逐个透传给 `verify_tier_b_seam3.py`。
"""

from __future__ import annotations

import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(REPO / "tools" / "px4-sitl"))

import verify_tier_b_seam3 as vb  # noqa: E402

_orig_dut_params = vb.dut_params_b


def _dut_params_with_prestream(cfg):
    """与实飞同口径：闸门关着时也发悬停预流（否则进不去 OFFBOARD，见模块 docstring）。"""
    params = _orig_dut_params(cfg)
    if not any(p.startswith("offboard_prestream:=") for p in params):
        params.append("offboard_prestream:=true")
    return params


vb.dut_params_b = _dut_params_with_prestream

#: `px4_msgs/VehicleStatus.nav_state` 的两个枚举值（与 `verify_offboard_smoke.NAV_STATE_OFFBOARD`
#: 同值）。旧载体的整数枚举 → 新载体的字符串之间的映射只服务于**判据层**。
_NAV_STATE_OFFBOARD = vb.v.NAV_STATE_OFFBOARD
_ARMING_STATE_ARMED = vb.v.ARMING_STATE_ARMED

_orig_frames = vb.v.Dut.frames


def _frames_with_nav_state(self):
    """把 mavros 载体的 `mode`/`armed` 合成回旧判据要读的 `nav_state`/`arming_state`。

    只**补键**（原始 JSONL 不动、`mode` 原样保留）：判据② 的「不失控」判的是
    `f.get("nav_state") == 14`，而票 13 之后这一列在日志里**根本不存在** ——
    不补的话每一场都会读成「不失控 False」，而那看起来完全像控制器的问题。
    """
    frames = _orig_frames(self)
    for f in frames:
        if "nav_state" not in f:
            f["nav_state"] = (_NAV_STATE_OFFBOARD
                              if str(f.get("mode") or "") == "OFFBOARD" else -1)
        if "arming_state" not in f:
            f["arming_state"] = _ARMING_STATE_ARMED if f.get("armed") else 1
    return frames


vb.v.Dut.frames = _frames_with_nav_state

#: 票 13 之后 DUT JSONL 里才有的列（旧 `FRAME_KEYS` 里只有 `thrust_body_z`，新载体没有它）。
#: `thrust` 是**归一化推力**（`u[0] / t_max`），票 01 的 `u_thrust_n` 从它换算。
_EXTRA_FRAME_KEYS = ("thrust", "body_rate_x", "body_rate_y", "body_rate_z", "mode")
vb.FRAME_KEYS = tuple(vb.FRAME_KEYS) + _EXTRA_FRAME_KEYS

if __name__ == "__main__":
    raise SystemExit(vb.main(sys.argv[1:]))
