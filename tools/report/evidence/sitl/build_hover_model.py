#!/usr/bin/env python3
"""生成 `R-hover` 的回放模型（`models/R-hover-model.json`）—— 票 02 第 1 步的悬停一支。

**为什么需要这一层包装**：`tools/px4-sitl/mocap_model.py` 的 `track_window()` 从
`/control/state` 的迁移里取 `AUTO/TRACK` 那一窗。`bag/3_traj1_hover`（2026-10-08 那次）
**是录到 TRACK 中途截断的**（参考只到 39.65 s / 60 s），`/control/state` 里 TRACK 就是
**最后一个状态** ⟹ `track_window()` 走 `(t0, states[-1][0])` 的退路，返回 **t0 == t1 的零长窗口**
⟹ `noise_sigma()` 拿到空数组、`reshape(0, -1)` 抛 `ValueError`。

本包装只做一件事：把窗口终点改成**包尾**（`tr.stamp_s[-1]`），其余逐字复用
`build_model()`（时序 / 延迟 / 噪声 / 实测 / 参考全是它算的）。没有改 `mocap_model.py`：
它被归档证据与别的票共用，本轮不在它的修改面上。

用法（**先 source 工作空间**，在**仓库根**跑）：

    source /opt/ros/humble/setup.bash && source ~/px4_msgs_ws/install/setup.bash
    source install/setup.bash
    python3 tools/report/evidence/sitl/build_hover_model.py
"""

from __future__ import annotations

import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(REPO / "tools" / "px4-sitl"))

import mocap_model as mm  # noqa: E402

BAG = Path("bag/3_traj1_hover")
NOISE_BAG = Path("bag/1_static")
WORKDIR = Path("/tmp/mocap-model-rmodels-hover")
OUT = Path("tools/report/evidence/sitl/models/R-hover-model.json")


def main() -> int:
    tr = mm.read_trace(BAG, workdir=WORKDIR)
    win = mm.track_window(tr.states)
    if win is None:
        raise SystemExit(f"{BAG} 里没有 AUTO/TRACK，无法按轨迹档建模")
    t0, t1 = win
    if t0 == t1:
        # TRACK 是最后一个状态（录包在 TRACK 中截断）⟹ 终点取包尾。
        t1 = float(tr.stamp_s[-1])
        print(f"[hover-model] TRACK 是最后一个状态 ⟹ 窗口终点取包尾 "
              f"{t1:.3f} s（{t1 - t0:.2f} s）")
    orig = mm.track_window
    mm.track_window = lambda states, state=mm.TRACK_STATE: (t0, t1)  # noqa: ARG005
    try:
        model = mm.build_model(BAG, NOISE_BAG, win_s=0.2, workdir=WORKDIR)
    finally:
        mm.track_window = orig
    model["segment"]["how"] += (
        "；★ TRACK 是包里最后一个状态（录包在 TRACK 中途截断）⟹ 终点取包尾，"
        "见 tools/report/evidence/sitl/build_hover_model.py")
    print(mm.render(model, NOISE_BAG))
    OUT.parent.mkdir(parents=True, exist_ok=True)
    import json
    OUT.write_text(json.dumps(model, indent=1, ensure_ascii=False), encoding="utf-8")
    print(f"\n模型写到 {OUT}（{OUT.stat().st_size / 1024:.0f} KiB）")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
