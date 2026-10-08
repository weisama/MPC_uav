#!/usr/bin/env python3
"""B 档闭环 + **seam 3 量化验收**（ticket 11）—— Phase 2 最高的那道闸，上面没有退路。

票面四条判据，本工具逐条落地（**判在 SITL 真值帧**，操作者 2026-09-29 裁定）：

| 判据 | 落地方式 |
|---|---|
| ① 起飞到 1 m 悬停、稳态位置误差 < 0.15 m、无持续振荡 | `判据①` + `判据①·振荡` |
| ② 跑完 Phase 1 的轨迹（**悬停 / r=0.5 m 八字** 两条） | `判据②·轨迹1` + `判据②·轨迹2`（**定性判**，见下） |
| ③ 注入 50 ms 动捕延迟后仍稳定（可变差、不可发散） | `判据③`（`--extra-delay-ms 50`） |
| ④ 故意让求解器失败，飞机保持安全悬停而不是坠机 | `判据④`（`--inject-solve-fail-after-frames`） |

★★ **判据② 现在怎么判**（操作者裁定，2026-09-30；原话在 11 号票 Comments）：

  1. **口径去掉「1 m 正方形」**，只留两条：悬停 / **r = 0.5 m 八字**。
     正方形那条从来没跑过，理由与"没有跟踪能力"一起作废 —— `bag/` 里本来也没有那份包。
  2. **判定口径 = 定性判：跑完 + 不撞地 + 不失控**；**跟踪误差只留档、不设界**。
     依据：spec（`docs/control/phase2-implementation.md`）对这一条本来就只有
     「不撞地、不失控」，**没有数值界** ⟹ **不编阈值**。
  3. **两条各由一档场次判**：悬停那条由 `--reference-source hold` 的场次判（F0–F4 就是它），
     八字那条只有 `--reference-source traj` 的场次才可能判（夹具重发包里的 `/traj/reference`）。
     一场跑不出两条 ⟹ 另一条当场记「**本场不做**」（`ok=None`、**不计数**）：
     既不冒充通过也不冒充不通过。**汇总时别把"本场不做"读成"通过"。**

  ⚠️ 2026-09-30 之前，这一行是**硬编码的 `False`**（"未达标，缺口不在本票手里"）。
     那是 17 号票还没落地时的写法：当时节点没有消费 `/traj/reference` 的口（见
     [17 号票](../../.scratch/phase2/issues/17-traj-reference-interface.md)，**已 `done`**），
     八字物理上不可能跑；而正方形那条的包在 `bag/` 里也不存在。
     ⟹ 现在换成真判。一行恒 ❌ 的判据在矩阵里会**掩盖**真实结果（真跑绿了也看不出来），
     属于最坏的那类静默失败。

**状态源怎么接的**（操作者裁定「包当参考 + 噪声/延迟模型」）：
  `/mocap/pose_enu` ← `mocap_replay.py` = **本场 SITL 真值(t − D) ⊕ 包里量出来的噪声**，
  发帧节奏 = 包里那一串**真实间隔**。真值经 DDS 的
  `/fmu/out/vehicle_local_position_groundtruth` 上来（桥接见 `enable_groundtruth_dds.sh`）。
  ⟹ **闭环没有断**：飞机怎么动，控制器看得见；包里出的是"这台动捕的脾气"，不是轨迹。

**被控对象**：`--mismatch mass+30` ⟹ `z0-mass+30.param`（ticket 08 那版机型 + 参数故意差 ±30%）。
  票面要的"错参数也能飞稳"就是这一条。

☠☠ **一处必须先说的航电口径**（操作者 2026-09-29 裁定「用 0.6235」，全文见 11 号票 Comments）：
  SITL 机型自己推出来的悬停归一化推力是 **0.6235**（`4 × SIH_T_MAX = 11.8 N`，
  `m·g/11.8 = 0.6235`；A 档实飞也量到同一个数 0.6233，见 `tier-a-sitl-hover.md:214`）。
  而 `nmpc/params.py` 在 ticket 10 回填后是 **0.1589**（真机满电口径）—— 两者差 **3.92×**。
  `thrust_body_z = -thrust_n / t_max`（`sign_map.py:69`）而 `t_max = m·g/hover_thrust`
  ⟹ 用 0.1589 时模型的控制效能高估 3.92 倍，飞机在悬停点上会以 ~7.3 m/s² 往下掉，
  **那量的是参数域错误、不是控制器**。所以本工具默认 `--hover-thrust 0.6235`
  （= 与 SITL 被控对象对得上的那个值），A/B 两场都从它起算；
  这个 3.92× 单独记账，**不动 `params.py`**（操作者：「11 补、10 不动」）。

**两组读数各自回答什么**（★ 别混）：

    --mismatch mass+30   ⟹ 票面要的**失配**实验（模型标称 vs 被控对象 +30% 质量）
    --mismatch none      ⟹ **归因对照**（模型与被控对象对得上）。判据④ 尤其需要它：
                            M5 兜底发的悬停推力是**模型参数**（`mpc_node.py:405`，恒 = −hover_thrust），
                            失配机型需要 1.30 倍 ⟹ 兜底只能保证"指令有界"，保不住高度。
                            这一条要**两个被控对象各跑一场**才能把"兜底机制坏了"与
                            "兜底的推力口径在被控对象上是错的"分开。

用法（**先 source 工作空间**）：

    source /opt/ros/humble/setup.bash && source ~/px4_msgs_ws/install/setup.bash
    source install/setup.bash

    # 判据①（失配被控对象）
    python3 tools/px4-sitl/verify_tier_b_seam3.py --tag F1 --mismatch mass+30 --json /tmp/tier-b/F1.json
    # 判据③（+50 ms）
    python3 tools/px4-sitl/verify_tier_b_seam3.py --tag F2 --mismatch mass+30 --extra-delay-ms 50 \\
        --json /tmp/tier-b/F2.json
    # 判据④（注入求解失败）
    python3 tools/px4-sitl/verify_tier_b_seam3.py --tag F3 --mismatch mass+30 \\
        --inject-solve-fail-after-frames 700 --json /tmp/tier-b/F3.json

    # 只重判（不重跑 SITL）
    python3 tools/px4-sitl/verify_tier_b_seam3.py --replay /tmp/tier-b/F1.json

☠️ **`--model` 可以不传**（默认 = 本仓库归档里那份 `evidence/tier-b/mocap-model-3.json`）。
   五场当时用的是 `/tmp/mocap-model-3.json` —— 而 **`/tmp` 是 tmpfs，机器 2026-09-29 10:40
   重启后那份就没了**（原始逐帧同样只在 `/tmp`）。模型现在**以归档里这份为准**，
   内容与当时那份一致：归档里的 `F*-replay.json` 记的 `sigma_m`（0.164/0.290/0.105 mm，
   逐位相同）与 `base_delay_ms = 5.34` 都是夹具**直接读模型**算出来并落盘的
   （`mocap_replay.py:188` ⟹ `:325`）⟹ 可逐位核对。
"""

from __future__ import annotations

import argparse
import bisect
import collections
import json
import math
import os
import statistics
import subprocess
import sys
import time
import traceback
from pathlib import Path

HERE = Path(__file__).resolve().parent
if str(HERE) not in sys.path:                      # 同目录复用（`ros2 run` 之外直接跑时）
    sys.path.insert(0, str(HERE))

import numpy as np  # noqa: E402
import rclpy  # noqa: E402
from px4_msgs.msg import VehicleAttitude, VehicleLocalPosition  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.qos import (HistoryPolicy, QoSProfile,  # noqa: E402
                       ReliabilityPolicy)

# ★ NED/FRD → ENU/FLU 走 `node_core` 里**唯一的那一处**（`px4_pose_to_enu`）——
#   夹具（`mocap_replay.py`）也走它。两边共用同一个换算，才谈得上"被测的是什么"。
#   ⚠️ `verify_offboard_smoke` **没有**转发这个名字（实测 `grep` 为空），所以直接从包里取。
from nmpc.node_core import px4_pose_to_enu, px4_quat_to_enu  # noqa: E402

import verify_offboard_smoke as v  # noqa: E402
import verify_tier_a_hover as ta  # noqa: E402

#: 真值话题的**DDS 全名**。⚠️ 与 A 档的 `ta.GT_TOPIC`（控制台话题名，无前导 `/fmu/out/`）
#: 不是同一个字符串 —— 一个是 `listener <名字>`，一个是 DDS 的 topic name。
TOPIC_GT_POS = "/fmu/out/vehicle_local_position_groundtruth"
TOPIC_GT_ATT = "/fmu/out/vehicle_attitude_groundtruth"
TOPIC_MOCAP = "/mocap/pose_enu"

#: `/fmu/out/*` 由 PX4 以 best_effort 发（`utilities.hpp:114-119`）⟹ 订阅方不能强于它。
#: depth 给 10 而不是 1：本节点只把样本往列表里塞（微秒级），但**丢一帧就是真值流上一个洞**，
#: 而判据①/③ 的读数全部建立在这条流的连续性上。真值流是 ~250 Hz，depth 10 = 40 ms 的缓冲。
QOS_TRUTH = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=10,
                       reliability=ReliabilityPolicy.BEST_EFFORT)

#: 稳态窗口的长度 [s]（判据①/③）：与 A 档 `evaluate` 的 `hover_s=3.0` 是**同一把尺子**，
#: 只是 B 档的真值流是全速的（250 Hz 而不是 4 Hz），所以窗口能取长一点而不丢分辨力。
STEADY_S = 8.0

#: 稳态误差的判据阈值 [m]（票面原话：「稳态位置误差 < 0.15 m」）
ERR_TOL_M = 0.15
#: 无持续振荡：峰峰值上界 [m] / 穿均值频率上界 [Hz] —— **沿用 A 档 `evaluate` 的④**
#: （`up_tail_p2p < 0.35` 与 `osc < 2.0`）。★ 沿用而不是另立一套：两档的读数要能并排看。
OSC_P2P_M = 0.35
OSC_HZ = 2.0
#: 判据③/④ 的"不发散"界 —— 票面只说"可以变差，但不能发散"，所以这里是**实现方的判断**，
#: 明说出来：末段垂直速度 < 0.5 m/s（还在悬停，不是在爬/在掉）、末段高度 p2p < 0.6 m、
#: 全程最低点 > 0.05 m（没触地）。
DIVERGE_VZ_MS = 0.5
DIVERGE_P2P_M = 0.6
GROUND_M = 0.05
#: 判据④ 的"安全悬停"界：末段平均高度与目标之差 < 0.35 m（比判据① 宽 —— 它要回答的是
#: "有没有保持住"，不是"控得多准"）
HOVER_TOL_M = 0.35

#: 判据② 的"不失控"界 [°]：**姿态翻过垂直就是失控了** —— 这是个物理分界（倾斜 90° 时升力
#: 全指向水平、飞机不再被支撑），不是调出来的数。⚠️ 它不是判据② 的主判（主判是窗口内
#: DUT 帧全在 OFFBOARD），而是**旁证**：见 `_judge_traj` 的 ★。
LOSS_TILT_DEG = 90.0
#: 判据② 的八字那条：真值行与 DUT 帧配对的时间容差 [s]。真值 ~250 Hz、DUT ~50 Hz，
#: 两条流各自带抖动；0.03 s 是"同一时刻"的宽松解释 —— 够把两条流配起来，
#: 又不会把远处那一拍拉进来冒充。
PAIR_TOL_S = 0.03

#: 进 obs 的逐帧列。判据层要用的**全部**在这里；其余（`qp_stat`/`quat_drift`/…）只在
#: DUT 自己那份 JSONL 里留着。少一列的症状是"判据层读不到数"，多一列的症状是 obs.json 变大
#: —— 前者更危险，所以宁可宽一点。
FRAME_KEYS = ("wall_ns", "tick", "fallback", "fallback_note", "x_ref0", "x_ref0_q", "x0",
              "u", "sat", "thrust_body_z", "ref_anchor", "anchor_yaw",
              "age_pos_s", "age_att_s", "age_state_s", "state_source",
              "ekf_xy_valid", "ekf_z_valid", "nav_state", "arming_state")

#: 真值行 `[t_rel, x, y, z, vx, vy, vz, wall]` 的**列号**（形状由 `TruthTap.enu_rows` 定义，
#: 与 A 档 `TruthSampler.track()` 同形 ⟹ 两档是同一把尺子）。
#: ☠️ **别凭记忆读列**：`r[2]` 是 **y**、`r[3]` 才是 **z**。本票自己在这上面栽过一次 ——
#: 拿 `r[2]` 当高度去算"最后 8 s 窗口的峰峰值"，算出来的是**水平漂移**，差点把一个错的
#: 数写进文档。列号有名字就不会再犯。
T_X, T_Y, T_Z, T_WALL = 1, 2, 3, 7


def tail_vz(rows, frac=5):
    """末段的 `(全段 z, 末段点, 末段相邻点差分出来的垂直速度)`。

    ★ **抽出来是因为它原来在两处逐字重复**（发散判据 与 M5 兜底判据）：两处都拿"末段差分速度"
      当"飞机还在不在动"的证据，而"末段"的口径一旦要改，两份**会悄悄不一致** —— 那一类不一致
      不会报错，只会让两条判据对同一场给出不同的结论。抽成一处就没有这个可能。
    """
    zs = [r[T_Z] for r in rows]
    tail = rows[-max(1, len(rows) // frac):]
    vz = [(tail[i][T_Z] - tail[i - 1][T_Z])
          / max(tail[i][T_WALL] - tail[i - 1][T_WALL], 1e-9)
          for i in range(1, len(tail))]
    return zs, tail, vz


# ------------------------------------------------------------------ 全速真值记录器


class TruthTap(Node):
    """把 SITL 真值**全速**记下来（DDS，不经控制台）。

    ★★ 为什么 B 档**不用** A 档那台 `TruthSampler`（`verify_tier_a_hover.py:149`）：
      A 档只能走控制台（`listener … -n 2`），原因是它要的 `vehicle_local_position` 是
      **EKF 的输出**，默认就在 `dds_topics.yaml` 里；而 A 档想拿真值就得额外桥接，
      于是它选了"稀疏采样 + 判据⓪ 自检"这条折中路。B 档的真值**本来就必须桥到 DDS**
      （它要合成 `/mocap/pose_enu`），所以这里直接全速收 ——

      · 判据① 的"无持续振荡"直接受益：A 档真值只有 4 Hz（分辨力 2 Hz，
        连判据自己的门槛都够不到，只能退回 EKF 帧判），B 档是全速 ⟹ **判据与仪器同一帧**。
      · 控制台**完全空着**：B 档不在控制台上抢任何流量，A 档那条"仪器把被控对象堵住"
        的失效模式（A 档判据⓪）在 B 档结构上不存在。

    ⚠️ 但**位置与姿态分开存**：动捕给的是**一个刚体的位姿**，而 DDS 上是两条独立话题。
      这里逐条记各自的到达时刻，**不做跨话题的配对** —— 判据①/③/④ 都只用位置那一路，
      姿态留着做"没失控"的旁证（倾斜角）。配对的活由 `mocap_replay.py` 干
      （它必须成对，因为它要合成一帧位姿）。
    """

    def __init__(self, out_path: Path) -> None:
        super().__init__("tier_b_truth_tap")
        self.out_path = out_path
        self.out_path.parent.mkdir(parents=True, exist_ok=True)
        self._log = self.out_path.open("w", encoding="utf-8")
        self.pos = []      #: (t_wall_s, x, y, z, vx, vy, vz) —— **NED**（真值原样，换算只在一处做）
        self.att = []      #: (t_wall_s, w, x, y, z)
        self.create_subscription(VehicleLocalPosition, TOPIC_GT_POS, self._on_pos, QOS_TRUTH)
        self.create_subscription(VehicleAttitude, TOPIC_GT_ATT, self._on_att, QOS_TRUTH)

    def _on_pos(self, msg) -> None:
        t = time.time()
        self.pos.append((t, float(msg.x), float(msg.y), float(msg.z),
                         float(msg.vx), float(msg.vy), float(msg.vz)))
        self._log.write(json.dumps({"topic": "pos", "t": t, "x": float(msg.x),
                                    "y": float(msg.y), "z": float(msg.z),
                                    "vx": float(msg.vx), "vy": float(msg.vy),
                                    "vz": float(msg.vz)}) + "\n")

    def _on_att(self, msg) -> None:
        t = time.time()
        q = msg.q                                   # PX4 是 [w, x, y, z]
        self.att.append((t, float(q[0]), float(q[1]), float(q[2]), float(q[3])))
        self._log.write(json.dumps({"topic": "att", "t": t, "q": [float(c) for c in q]}) + "\n")

    def close(self) -> None:
        self._log.close()

    def enu_rows(self):
        """`[[t_rel_s, x, y, z, vx, vy, vz, wall_s], …]`，**ENU**。

        ★ 列 0–6 与 A 档 `TruthSampler.track()` 的**完全同形** ⟹ `ta.segments` / `ta.z_stats`
          / `ta.crossings` 直接拿来用，两档的判据是**同一把尺子**（这条是有意的：
          "B 档是不是比 A 档好"这种话只有在同一把尺子上才说得出口）。
        ★★ 第 7 列 `wall_s` 是**绝对墙钟**，它是**配对键**（与 DUT 逐帧日志的 `wall_ns`
          同一个时基：`time.time()` 就是 ROS 的 epoch ns ÷1e9）。
          把它塞进同一行而不是另开一个平行数组：平行数组**会错位**，而错位在配对阶段
          只表现为"一条也没配上"，看着像"没数据"—— A 档 `payload_rows` 的 ★★ 记过这一次。

        ⚠️ 速度列直接取**消息自己的 `vx/vy/vz`**（与位置同一帧、同一个 NED 帧 ⟹
          与 A 档那一路的行完全同源），**不另做差分** —— 差分会把"速度"变成另一个量，
          而 A/B 两档的行同形才有意义。
        """
        if len(self.pos) < 1:
            return []
        t0 = self.pos[0][0]
        rows = []
        for t, x, y, z, vx, vy, vz in self.pos:
            p_enu, _q, v_enu = px4_pose_to_enu((x, y, z), (1.0, 0.0, 0.0, 0.0), (vx, vy, vz))
            rows.append([round(t - t0, 6), float(p_enu[0]), float(p_enu[1]), float(p_enu[2]),
                         float(v_enu[0]), float(v_enu[1]), float(v_enu[2]), float(t)])
        return rows

    def tilt_deg_max(self, t_lo: float, t_hi: float):
        """窗口内**离竖直方向**的最大夹角 [°] —— "没失控"的旁证读数。

        ★ 为什么要它：位置那几条判据在"机头在自转 / 姿态在乱摆"时**可能全绿**
          （A 档 2026-09-28 实测：飞机自转 5 圈而位置判据全过，见 `evaluate` 的⑥b）。
          倾角是姿态那一路唯一一个**与朝向无关**的读数（自转不改变倾角），
          所以它恰好补上那个盲区。
        """
        vals = [r for r in self.att if t_lo <= r[0] <= t_hi]
        if not vals:
            return None
        out = []
        for _t, w, x, y, z in vals:
            q = np.array([x, y, z, w], dtype=float)          # xyzw
            n = float(np.linalg.norm(q))
            if n < 0.5:
                continue
            q /= n
            # 机体 +z 在惯性系里的分量 = R[2,2] ⟹ 与竖直的夹角
            r22 = 1.0 - 2.0 * (q[0] ** 2 + q[1] ** 2)
            out.append(math.degrees(math.acos(max(-1.0, min(1.0, r22)))))
        return max(out) if out else None


# ------------------------------------------------------------------ 回放夹具进程


class ReplayProc:
    """`mocap_replay.py` 的子进程（**同目录脚本，不是 ROS 包** ⟹ 不能 `ros2 run`）。

    ★ 用 `sys.executable` 而不是 `python3`：操作者的环境里 `python3` 可能是系统解释器
      （没有 `px4_msgs`），而本工具自己就跑在**对的那个**解释器上。
    """

    def __init__(self, args, out_jsonl: Path, summary: Path) -> None:
        self.args, self.out_jsonl, self.summary = args, out_jsonl, summary
        self.proc = None

    def start(self) -> "ReplayProc":
        self.out_jsonl.parent.mkdir(parents=True, exist_ok=True)
        self.out_jsonl.unlink(missing_ok=True)          # 别读到上一场的文件（同 `v.Dut.start` 的 ★）
        cmd = [sys.executable, str(HERE / "mocap_replay.py"),
               "--model", self.args.model,
               "--extra-delay-ms", str(self.args.extra_delay_ms),
               "--seed", str(self.args.seed),
               "--out", str(self.out_jsonl),
               "--summary", str(self.summary)]
        if self.args.base_delay_ms is not None:
            cmd += ["--base-delay-ms", str(self.args.base_delay_ms)]
        if getattr(self.args, "reference_source", "hold") == "traj":
            # ★ 参考那一路的逐帧留档单独一份：判层要拿它算"轨迹到底在不在 OFFBOARD 窗口里"，
            #   与动捕那一路（`--out`）混在一起就分不出谁是谁。
            self.ref_out = self.out_jsonl.parent / (self.out_jsonl.stem + "-reference.jsonl")
            self.ref_out.unlink(missing_ok=True)
            cmd += ["--publish-reference",
                    "--ref-z-offset-m", str(ref_z_offset_m(self.args)),
                    "--ref-hold-s", str(self.args.ref_hold_s),
                    "--ref-out", str(self.ref_out)]
        self.proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                     text=True, start_new_session=True)
        return self

    def alive(self) -> bool:
        return self.proc is not None and self.proc.poll() is None

    def stdout(self) -> str:
        if self.proc is None:
            return ""
        if self.proc.poll() is None:
            return ""
        return self.proc.stdout.read() or ""

    def frames_published(self) -> int:
        """从逐帧 JSONL 数发出去多少帧 —— "夹具在发吗"的**一手**读数（不猜、不问话题）。"""
        if not self.out_jsonl.exists():
            return 0
        with self.out_jsonl.open("rb") as fh:
            return sum(1 for _ in fh)

    def sigint(self, timeout=10.0) -> str:
        """正常收尾（它会打印自己的汇总 JSON）。"""
        if self.proc is None or self.proc.poll() is not None:
            return self.stdout()
        try:
            os.killpg(os.getpgid(self.proc.pid), 2)     # SIGINT
        except ProcessLookupError:
            pass
        try:
            out, _ = self.proc.communicate(timeout=timeout)
            return out or ""
        except subprocess.TimeoutExpired:
            self.kill()
            return (self.stdout() or "") + "\n[回放夹具收尾超时]"

    def kill(self) -> None:
        if self.proc is None or self.proc.poll() is not None:
            return
        try:
            os.killpg(os.getpgid(self.proc.pid), 9)
        except ProcessLookupError:
            pass
        try:
            self.proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            pass

    def ref_timeline(self, t0: float) -> dict:
        """夹具那份参考逐帧日志 → **与记录器同一根墙钟**上的时间线（判层只能吃 obs）。

        ★★ 为什么要在采集侧算好：判据层是 obs 的**纯函数**（`--replay` 不重跑 SITL），
          让判据去读文件就等于"重判时读到的可能是另一次跑的文件"。
        ★ 为什么不能只看"夹具在发帧"：参考流**照着时间表**发，与 DUT 起没起来、
          进没进 OFFBOARD **完全无关**。若 DUT 起得慢，整条八字会在**地面上**放完，
          而夹具的汇总里一切正常（帧数、节奏、掩码全对）⟹
          "跑完了三条轨迹"会变成一句**没被验证**的话。这个时间线就是用来判它的。
        """
        path = getattr(self, "ref_out", None)
        if path is None or not path.exists():
            return {"n": 0, "why": "没有参考逐帧日志（这一场不是 traj 档？）"}
        rows = []
        for line in path.read_text(encoding="utf-8").splitlines():
            if line.strip():
                rows.append(json.loads(line))
        if not rows:
            return {"n": 0, "why": f"{path} 是空的"}
        by_phase = collections.Counter(r["phase"] for r in rows)
        #: "在动"的帧 = 水平位置非零。八字的移动段就是这么认出来的（包 4：帧 0..304）；
        #: 用 `--ref-z-offset-m` 叠出来的 z 不算"动"（那是高度，不是轨迹）。
        move = [r for r in rows if abs(r["position"][0]) > 1e-6 or abs(r["position"][1]) > 1e-6]
        out = {"n": len(rows), "by_phase": dict(by_phase), "n_moving": len(move),
               "first_s": round(rows[0]["wall_ns"] * 1e-9 - t0, 3),
               "last_s": round(rows[-1]["wall_ns"] * 1e-9 - t0, 3),
               "log": str(path)}
        if move:
            out["move_first_s"] = round(move[0]["wall_ns"] * 1e-9 - t0, 3)
            out["move_last_s"] = round(move[-1]["wall_ns"] * 1e-9 - t0, 3)
            out["move_span_s"] = round(move[-1]["wall_ns"] * 1e-9
                                       - move[0]["wall_ns"] * 1e-9, 3)
        return out


# ------------------------------------------------------------------ 被控对象（换机型）


class BWorld(v.World):
    """B 档的被控对象：只把**参数覆盖层**换成失配变体，其余（起手清理、时钟偏移、
    `Companion`/`sihsim_z0`、agent 收尾）一个字都不动。

    ★ 靠的是 `v.World.plant_param` 这个加法式的钩子（默认为 `z0.param`，行为逐字不变）
      —— 而不是把 `_enter()` 抄一份：那份逻辑里有"起手必须一样"那条教训
      （类 docstring：上一场残留的 AUTO_LAND 会让下一场进不去 OFFBOARD），抄一份就等于
      让它有两处会漂。
    """

    def __init__(self, args) -> None:
        super().__init__(args)
        self.plant_param = plant_param_file(args.mismatch)


#: 失配变体的名字 → 覆盖层文件名（`z0-plant/params/`）。`none` = 标称机型。
#: 单参数、±30%：与 `z0-plant/plant_spec.py` 的生成器一一对应。
MISMATCHES = {
    "none": "z0.param",
    "mass+30": "z0-mass+30.param", "mass-30": "z0-mass-30.param",
    "thrust+30": "z0-thrust+30.param", "thrust-30": "z0-thrust-30.param",
    "inertia+30": "z0-inertia+30.param", "inertia-30": "z0-inertia-30.param",
    "tau+30": "z0-tau+30.param", "tau-30": "z0-tau-30.param",
}


def plant_param_file(mismatch: str) -> str:
    """`mass+30` → `z0-mass+30.param`；**不认识的名字当场炸**并列出可选值。

    ★ 不静默退回标称：那样"我跑了失配实验"与"我跑了标称"在盘上长得一模一样，
      而这正是本票最贵的一次实验（票面：「错参数也能飞稳」是判据的一部分）。
    """
    if mismatch not in MISMATCHES:
        raise SystemExit(f"不认识的失配名 {mismatch!r}；可选：{sorted(MISMATCHES)}")
    path = HERE / "z0-plant" / "params" / MISMATCHES[mismatch]
    if not path.exists():
        raise SystemExit(f"{path} 不存在 —— 先跑 tools/px4-sitl/z0-plant/plant_spec.py")
    return MISMATCHES[mismatch]


def plant_hover_thrust(param_file: str):
    """从**被控对象自己的参数文件**里读 `(SIH_MASS, SIH_T_MAX×4, 悬停归一化推力)`。

    ★ 从机型文件读、不在代码里抄常数 —— 与 A 档「判据里的常数都从 z0 机型文件读」同一条纪律
      （`verify_offboard_smoke.py:875`）。判据④ 的**预测下降率**就建立在这个数上，
      而它必须与被控对象**同源**，否则"预测"与"实测"对不上时无法判断是谁错了。
    """
    mass = tmax_total = None
    for line in (HERE / "z0-plant" / "params" / param_file).read_text(encoding="utf-8").splitlines():
        p = line.split()
        if len(p) >= 4 and p[0] == "param" and p[1] == "set":
            if p[2] == "SIH_MASS":
                mass = float(p[3])
            elif p[2] == "SIH_T_MAX":
                tmax_total = 4.0 * float(p[3])
    if mass is None or tmax_total is None:
        return None
    t_hover = mass * 9.81
    return {"mass_kg": mass, "t_max_total_n": tmax_total,
            "hover_thrust": t_hover / tmax_total, "t_hover_n": t_hover}


# ------------------------------------------------------------------ DUT 参数


def ref_z_offset_m(cfg) -> float:
    """`traj` 档下夹具叠在参考 `position.z` 上的高度（默认 = `--hold-z`）。

    ★ 单独成函数：**"高度由谁叠"是这一档最容易接错的一根线**（见 `dut_params_b` 的 ☠️），
      而两个取用点（命令行、obs 留档）必须是同一个数 —— 各写一遍就会出现
      "命令行叠了 1 m、留档里写 0"这种事后谁也判不清的场面。
    """
    z = getattr(cfg, "ref_z_offset_m", None)
    return float(cfg.hold_z if z is None else z)


def dut_params_b(cfg) -> list:
    """给 `mpc_node.py` 的参数（`--ros-args -p 名:=值`）。

    ★ `state_source:=mocap` 是 B 档的**全部意义**：状态源换成 `/mocap/pose_enu`
      （默认就是 mocap，但**显式写出来** —— 这条参数就是本档的身份，让它出现在日志里）。
    ★ `hover_thrust` 显式给：见模块 docstring 的 ☠☠（SITL 机型是 0.6235，不是 0.1589）。
    ★ `rate_hz` 不改：D6 是"先按 100 Hz 量最坏耗时、再按实测降档"。
    ★★ `--reference-source traj`（判据② 的八字那条）下 **`hold_offset` 必须全 0** —— 不是我们
      客气，是 `mpc_node._validate` 会**拒跑**非零（`mpc_node.py:1415`）。所以那一档：
      高度改由夹具叠在参考上（`--ref-z-offset-m`），DUT 只收到"相对锚点的轨迹"。
      ☠️ 两条路是**互斥**的：谁叠高度是二选一。写反了（DUT 拿 `hold_offset_z=1` 又收相对参考）
      会**起不来**（拒跑），比静默错好 —— 但要是两边都置 0，飞机就停在地面，
      而判层只会看到"它没动"，看不出是配置错。
    """
    traj = getattr(cfg, "reference_source", "hold") == "traj"
    params = [f"log_file:={cfg.log_file}",
              "state_source:=mocap",
              f"hover_thrust:={cfg.hover_thrust}",
              f"hold_offset_x:={0.0 if traj else cfg.hold_x}",
              f"hold_offset_y:={0.0 if traj else cfg.hold_y}",
              f"hold_offset_z:={0.0 if traj else cfg.hold_z}",
              f"reference_source:={getattr(cfg, 'reference_source', 'hold')}",
              f"rate_hz:={cfg.rate_hz}"]
    if cfg.inject_solve_fail_after_frames > 0:
        params += [f"inject_solve_fail_after_frames:={cfg.inject_solve_fail_after_frames}"]
    return params


# ------------------------------------------------------------------ 飞一场


def run_flight_b(px, rec, tap, dut, replay, cfg, agent) -> dict:
    """一场 B 档闭环：夹具发流 → 解锁 → OFFBOARD → 悬停 `hold_s` 秒 → 收尾。

    ★ 起手顺序是有讲究的（**夹具先于 DUT**）：`mpc_node` 起手就 `preflight_check`，
      而 B 档的闸门里有"动捕流多久没来"这一条（`mocap_timeout_s = 0.5 s`）。
      夹具在 DUT 之后起的话，DUT 头几拍的 `x0` 是空的，日志开头会是一段
      `hold`/`hover` —— 那会把"判据④ 的注入"和"起手没数据"混成同一列，
      而它们是完全不同的两件事。
    """
    obs = {"name": cfg.tag, "plant": cfg.mismatch, "plant_param": cfg.plant_param,
           "mismatch": cfg.mismatch, "state_source": "mocap",
           "reference_source": getattr(cfg, "reference_source", "hold"),
           "params": dut_params_b(cfg), "rate_hz": cfg.rate_hz, "hold_s": cfg.hold_s,
           "extra_delay_ms": cfg.extra_delay_ms, "model": cfg.model,
           "inject_solve_fail_after_frames": cfg.inject_solve_fail_after_frames,
           "plant_hover": plant_hover_thrust(cfg.plant_param), "t": {}}
    if obs["reference_source"] == "traj":
        #: 高度**由夹具叠**（DUT 那一档收不到 `hold_offset`，见 `dut_params_b` 的 ☠️）
        obs["ref_z_offset_m"] = ref_z_offset_m(cfg)
        obs["ref_hold_s"] = cfg.ref_hold_s
    nodes = [rec, agent, tap]

    # ① 夹具先起来，**并且真的在发帧**（判在它自己的逐帧 JSONL 上，不问话题）
    replay.start()
    ok = ta.wait_until(px, nodes, lambda: replay.frames_published() >= 30, 30.0,
                       "回放夹具没在发帧（模型文件坏了？看它的 stdout）")
    obs["replay_up"] = bool(ok)
    obs["t"]["replay_frames_at_dut_start"] = replay.frames_published()
    obs["t"]["dut_start"] = round(rec.now(), 2)

    # ② DUT 起来
    if not ta.wait_until(px, nodes, lambda: dut.frames(), cfg.dut_timeout_s,
                         "DUT 一帧都没发（节点没起来？）"):
        obs["dut_dead"] = True
    obs["t"]["first_frame"] = round(rec.now(), 2)

    # ③ 解锁 + OFFBOARD（等**记录器**看见 nav_state=14 —— 另一个时钟口径）
    t_before = rec.now()
    ta.wait_until(px, nodes, lambda: rec.first_nav(v.NAV_STATE_OFFBOARD, since=t_before) is not None,
                  cfg.offboard_timeout_s, "PX4 没进 OFFBOARD")
    t_enter = rec.first_nav(v.NAV_STATE_OFFBOARD, since=t_before)
    obs["t"]["offboard_enter"] = None if t_enter is None else round(t_enter - t_before, 2)
    #: OFFBOARD 起点在**记录器墙钟**（`rec.t0` 起算）上的值：`traj` 档靠它与参考时间线对表。
    #: ⚠️ 上面那个 `offboard_enter` 是相对 `t_before` 的**秒数**，与夹具的墙钟不同源，
    #:   拿它去和参考的 `wall_ns` 比就是拿两个零点不同的量相减。
    obs["t"]["offboard_enter_t"] = None if t_enter is None else round(t_enter, 3)
    obs["armed_ever"] = rec.armed_ever(t_before, rec.now())
    obs["agent"] = agent.describe()

    # ④ 悬停窗口（墙钟）
    if t_enter is not None:
        ta.spin_all(px, nodes, cfg.hold_s)
    t_end = rec.now()
    obs["t"]["flight_end"] = round(t_end, 2)
    obs["t"]["flight_end_t"] = round(t_end, 3)

    # ⑤ 收尾：把 DUT 与夹具都停下来，各自的汇总各自报
    frames = dut.frames()
    obs["dut"] = ta.summarise_frames(frames)
    obs["solve"] = ta.solve_stats(frames)
    # ★★ 逐帧日志要**进 obs**：判据是 obs 的纯函数（`--replay` 不重跑 SITL），
    #    而 `summarise_frames` 只给汇总 —— 兜底阶梯的**迁移顺序**、稳态窗口的边界、
    #    `x_ref0` 的逐帧值都只有原始帧才有。只留判据用得上的列，控制 obs.json 的体积。
    obs["frames"] = [{k: f.get(k) for k in FRAME_KEYS} for f in frames]
    replay_out = replay.sigint()
    obs["replay_stdout_tail"] = "\n".join(replay_out.strip().splitlines()[-12:])
    if obs["reference_source"] == "traj":
        # ★ 放在**夹具停了之后**读：发帧线程还在跑时读，读到的是半条时间线，
        #   而"八字跑完没有"正是拿这条时间线的**末尾**判的。
        obs["reference_timeline"] = replay.ref_timeline(rec.t0)
    if cfg.replay_summary.exists():
        try:
            obs["replay"] = json.loads(cfg.replay_summary.read_text(encoding="utf-8"))
        except (OSError, ValueError) as e:                       # noqa: BLE001
            obs["replay"] = {"读不出汇总": repr(e)}
    stdout = dut.sigint()
    obs["stdout_tail"] = "\n".join(stdout.strip().splitlines()[-8:])
    for line in stdout.splitlines():
        if "收尾：" in line:
            obs["summary"] = line.strip()

    # ⑥ 真值（全速，ENU + 绝对墙钟配对键）
    rows = tap.enu_rows()
    obs["truth"] = rows
    obs["truth_n"] = len(rows)
    if len(rows) >= 2:
        span = rows[-1][0] - rows[0][0]
        obs["truth_hz"] = round((len(rows) - 1) / span, 2) if span > 0 else None
    obs["truth_att_n"] = len(tap.att)
    obs["truth_pos_n"] = len(tap.pos)
    #: "没失控"的旁证（★ 在采集侧算好塞进 obs：判据是 obs 的纯函数）
    obs["tilt_deg_max"] = tap.tilt_deg_max(0.0, float("inf"))
    _lb = steady_bounds(frames)
    obs["tilt_deg_max_steady"] = None if _lb is None else tap.tilt_deg_max(*_lb)
    #: ⓪b 仪器自检要用它在**判据层**里配对（`att_err_deg`）—— 判据只吃 obs，
    #: 所以这一路真值随 obs 走（不进 JSON：它是活对象，序列化由 `_jsonable` 兜）。
    obs["_tap"] = tap

    # ⑦ 末段几个"同一时刻"的读数（B 档控制台是空的 ⟹ 想采就采，没有 A 档那层抢流量的问题）
    st = v.sample_topic(px, "vehicle_status", timeout=ta.SAMPLE_TIMEOUT_S, rec=rec)
    obs["status_end"] = {"nav_state": v.field(st, "nav_state"),
                         "arming_state": v.field(st, "arming_state"),
                         "pre_flight_checks_pass": v.bool_field(st, "pre_flight_checks_pass")}
    mo = v.sample_topic(px, "actuator_outputs_sim", timeout=ta.SAMPLE_TIMEOUT_S, rec=rec)
    obs["motors_mean_end"] = v.motor_mean(mo)
    obs["thrust_n_end"] = v.thrust_from_motors(mo)
    return obs


# ------------------------------------------------------------------ 判据①/③ 的读数


def frame_at(frames, *, wall_lo=None, wall_hi=None, normal_only=False):
    out = []
    for f in frames or []:
        t = (f.get("wall_ns") or 0) / 1e9
        if wall_lo is not None and t < wall_lo:
            continue
        if wall_hi is not None and t > wall_hi:
            continue
        if normal_only and f.get("fallback") != "normal":
            continue
        out.append(f)
    return out


def steady_bounds(frames, steady_s: float = STEADY_S):
    """稳态窗口的**绝对墙钟**上下界 —— 到"最后一次正常帧"为止。

    ★ 为什么到"最后一次正常帧"而不是"整场结束"：判据① 要的是**稳态**读数，
      而判据④ 那场里注入之后**全程**都是兜底帧。若窗口取到整场末尾，
      一场坏掉的兜底会被当成"悬停精度不好"记进判据①，两件事就混了。
      ⟹ 窗口一律**止于最后一次正常帧**；判据④ 自己另有一段读数（见 `fallback_report`）。
    """
    fs = frames or []
    if not fs:
        return None
    normal = [f for f in fs if f.get("fallback") == "normal" and f.get("x_ref0")]
    t_hi = (normal[-1]["wall_ns"] / 1e9) if normal else (fs[-1].get("wall_ns", 0) / 1e9)
    if not t_hi:
        return None
    return t_hi - steady_s, t_hi


def ref_target(frames, t_lo: float, t_hi: float):
    """窗口内的**目标位置**（ENU）= 节点自己 `x_ref0` 的逐轴中位数。

    ★ 为什么用**中位数**而不是均值：`x_ref0` 在锚点被重钉的那一拍照例会跳一下，
      中位数对这种孤立跳变不敏感（重钉那件事另有 `ref_anchor` 那一列专门记账）。
    ★ 为什么用**节点自己的** `x_ref0` 当目标、而不是把 `--hold-z 1.0` 写死：
      真目标是"锚点 + 偏移"，锚点是**起飞前那一刻的测量**，它不一定是 (0,0,0)。
      拿 (0,0,1) 当目标会把"锚点偏了多少"算进控制器的误差里。这一点在 B 档尤其要小心：
      B 档的锚点来自**合成流**，它带着回放噪声 σ 的那一次抽样。
    """
    vals = [f["x_ref0"] for f in frame_at(frames, wall_lo=t_lo, wall_hi=t_hi) if f.get("x_ref0")]
    if not vals:
        return None
    return [statistics.median([v[i] for v in vals]) for i in range(3)]


def att_err_deg(tap, frames, delay_s: float = 0.0) -> dict:
    """DUT **报出来的姿态**与同一时刻**真值**的夹角 [°] —— 仪器自检（判据① 的前置）。

    ★★ 为什么非有这一条：B 档首飞（2026-09-29）飞机全程没离地，而逐帧日志看着"很正常"
      —— 真身是**夹具**把四元数的**分量序**写反了（`px4_pose_to_enu` 给的是 wxyz，
      而 `PoseStamped.orientation` 的字段是 x,y,z,w），水平机于是变成"绕 x 转 −90°"。
      控制器整场都在扳这个并不存在的 90° 滚转，代价是把推力丢到下限（`F_min` 饱和
      1543/1544）。**位置判据、饱和日志、闸门日志全都正常**，只有拿"DUT 看到的姿态"
      与"真值姿态"对撞才看得见 —— 实测那一场读出来是 **120°**：错的姿态是「绕 x 转 90°」，
      对的应该是「绕 z 转 90°」，两个转轴正交 ⟹ 相对角 120°（`2·acos(|⟨q1,q2⟩|)`）。
      夹具那侧有 `test_mocap_model.py` 的矩阵恒等式钉着，
      这一条钉的是**接线**：夹具 → 话题 → 节点 → `x0`，任何一环写反都会在这里变成几十度。

    ★ 判在**中位数**而不是最大值：这条问的是"两路看的是不是同一个物理姿态"，
      是**整体**属性。取最大值会被两件本来就存在的事顶起来 —— 回放的固有延迟里
      姿态在动（判据③ 那场注入 50 ms 后更是如此）、以及两路各自的时间戳抖动。
      ⟹ 按**已知延迟**把真值回退 `delay_s` 再配对（这样判据③ 那场也照样说得通），
      判中位数、报最大值。真串了序的那种错是**整体**偏 90°，中位数一样看得见。
    """
    att = getattr(tap, "att", None) or []
    fs = [f for f in (frames or []) if f.get("x0") and len(f["x0"]) >= 10 and f.get("wall_ns")]
    if not att or not fs:
        return {"n": 0}
    ts = [r[0] for r in att]
    errs = []
    for f in fs:
        t = f["wall_ns"] / 1e9 - delay_s
        i = bisect.bisect_left(ts, t)
        best = None
        for j in (i - 1, i):
            if 0 <= j < len(att) and abs(ts[j] - t) <= 0.015:      # 15 ms 内才配对
                if best is None or abs(ts[j] - t) < abs(ts[best] - t):
                    best = j
        if best is None:
            continue
        # 真值是 NED/FRD（PX4 原生），DUT 的 `x0` 是 ENU/FLU ⟹ 先过那**唯一一处**换算
        q_truth = px4_quat_to_enu(att[best][1:])
        q_dut = np.asarray(f["x0"][6:10], dtype=float)
        n1, n2 = float(np.linalg.norm(q_truth)), float(np.linalg.norm(q_dut))
        if n1 < 0.5 or n2 < 0.5:
            continue
        dot = abs(float(np.dot(q_truth / n1, q_dut / n2)))
        errs.append(math.degrees(2.0 * math.acos(max(-1.0, min(1.0, dot)))))
    if not errs:
        return {"n": 0}
    errs.sort()
    return {"n": len(errs), "median": statistics.median(errs),
            "p95": errs[min(len(errs) - 1, int(0.95 * len(errs)))], "max": errs[-1]}


def err_metrics(truth, target, t_lo: float, t_hi: float, tag: str) -> dict:
    """窗口内**真值 − 目标** 的误差读数（水平 / 垂直 / 3D）。

    ★ 三个口径都报，判据只判**均值**（票面"稳态位置误差"）：p2p 与 max 交给"无持续振荡"、
      std 交给"发不发散"。把它们混成一个数就再也分不出"偏了 0.2 m"与"在 ±0.2 m 上抖"。
    """
    rows = [r for r in truth if t_lo <= r[7] <= t_hi]
    if not rows or target is None:
        return {"n": 0, "tag": tag}
    hx = [r[1] - target[0] for r in rows]
    hy = [r[2] - target[1] for r in rows]
    hz = [r[3] - target[2] for r in rows]
    horiz = [math.hypot(a, b) for a, b in zip(hx, hy)]
    out = {"n": len(rows), "tag": tag, "window_s": round(t_hi - t_lo, 2),
           "span_s": round(rows[-1][0] - rows[0][0], 2),
           "target_enu": [round(c, 4) for c in target],
           # 水平误差（标量范数）与垂直误差（带符号）
           "h_mean": statistics.fmean(horiz), "h_max": max(horiz),
           "z_mean": statistics.fmean(hz), "z_absmean": statistics.fmean(abs(c) for c in hz),
           "z_max": max(abs(c) for c in hz),
           # 3D 范数（最保守的口径，报告用；判据仍按水平/垂直分开判）
           "d3_mean": statistics.fmean(math.hypot(h, z) for h, z in zip(horiz, hz)),
           "d3_max": max(math.hypot(h, z) for h, z in zip(horiz, hz)),
           "h_std": statistics.pstdev(horiz) if len(horiz) > 1 else 0.0,
           "z_std": statistics.pstdev(hz) if len(hz) > 1 else 0.0,
           "z_p2p": max(hz) - min(hz), "h_p2p": max(horiz) - min(horiz),
           "z_min_abs": min(r[3] for r in rows),
           "vz_tail": statistics.fmean(r[6] for r in rows[-max(1, len(rows) // 10):]),
           }
    return out


def traj_err_metrics(frames, truth, t_lo: float, t_hi: float) -> dict:
    """窗口内**真值 − DUT 自己那一刻的 `x_ref0`** 的误差读数（判据② 的**留档**项）。

    ★★ 为什么拿节点自己的 `x_ref0` 当"该在哪儿"，而不是把参考包的坐标再算一遍：
      节点算的是 `目标 = 锚点 + p_rel`，锚点是**起飞前那一刻的实测**（B 档还带一次回放
      噪声抽样）。在外面重算就得自己复刻"锚点取哪一拍"，一旦取错，**锚点的偏差会被
      算成跟踪误差** —— 与 `ref_target` 那条 ★ 同一个坑。`x_ref0` 是控制器当时真正在
      追的那个点，拿它做基准等于直接问"追上了没有"。
      ⚠️ 代价说清楚：这条读数是「真值 vs 控制器**以为**的目标」，不是「真值 vs 包里那条
      理想八字」。两者差值 = 锚点误差 + 参考路自身的误差，本读数**分不开**它们。
    ★ 窗口按**真值行**截、DUT 帧按最近邻配对（±`PAIR_TOL_S`）—— 两条流各自有抖动，
      不配对就只能比"窗口均值 vs 窗口均值"，而那会把相位差抹平。
    ★★ 这一块**只留档、不设界**（操作者 2026-09-30 裁定：spec 原文对这一条只有
      「不撞地、不失控」，**没有数值界** ⟹ **不编阈值**）。所以它返回读数、不返回 `ok`。
    """
    rows = [r for r in (truth or []) if t_lo <= r[T_WALL] <= t_hi]
    fs = [f for f in frame_at(frames, wall_lo=t_lo - PAIR_TOL_S, wall_hi=t_hi + PAIR_TOL_S)
          if f.get("x_ref0") and f.get("wall_ns")]
    out = {"n": len(rows), "n_paired": 0, "tag": "参考跟踪（留档）",
           "window_s": round(t_hi - t_lo, 2), "pair_tol_s": PAIR_TOL_S}
    if not rows or not fs:
        return out
    fw = [f["wall_ns"] / 1e9 for f in fs]
    hz, dz, d3 = [], [], []
    for r in rows:
        i = bisect.bisect_left(fw, r[T_WALL])
        best = None
        for j in (i - 1, i):
            if 0 <= j < len(fs) and abs(fw[j] - r[T_WALL]) <= PAIR_TOL_S:
                if best is None or abs(fw[j] - r[T_WALL]) < abs(fw[best] - r[T_WALL]):
                    best = j
        if best is None:
            continue
        tgt = fs[best]["x_ref0"]
        h = math.hypot(r[T_X] - tgt[0], r[T_Y] - tgt[1])
        z = r[T_Z] - tgt[2]
        hz.append(h)
        dz.append(z)
        d3.append(math.hypot(h, z))
    if not hz:
        return out
    out.update({"n_paired": len(hz),
                "pair_frac": round(len(hz) / len(rows), 3),
                "h_mean": statistics.fmean(hz), "h_max": max(hz),
                "z_mean": statistics.fmean(dz), "z_absmean": statistics.fmean(abs(c) for c in dz),
                "d3_mean": statistics.fmean(d3), "d3_max": max(d3)})
    return out


def _window_health(frames, truth, t_lo: float, t_hi: float) -> dict:
    """窗口内判据② 要的三条**定性**读数：跑到了没有、撞没撞地、失控没失控。

    ★ 三个零件的口径（都会被写进判据行的 detail，看得见才谈得上复核）：
      · `ran`        —— 窗口内 DUT 帧 ≥ 10 **且** 真值点 ≥ 10（数据够不够判）
      · `air`        —— 窗口内真值最低点 > `GROUND_M`（不撞地）
      · `offboard_kept` —— 窗口内**每一帧** DUT 报的 `nav_state` 都是 14
    ★★ 为什么"不失控"的主判取 `nav_state` 而不是倾角：`nav_state` 在**每一帧**日志里
      都有（`--replay` 也读得到），而倾角只有整场/稳态两个口径、且**含 DUT 停机之后
      那一段**（真值一直录到收尾）—— 拿整场倾角当窗口判据，一次"停完自己摔了"就会
      把整个窗口读成失控。倾角因此降级为**旁证**（`_judge_traj` 里照报不设界），
      这与判据① 那条「没失控的旁证：真值倾角」本来就不设阈值的写法是同一条。
    """
    fs = frame_at(frames, wall_lo=t_lo, wall_hi=t_hi)
    nav = [f.get("nav_state") for f in fs]
    rows = [r for r in (truth or []) if t_lo <= r[T_WALL] <= t_hi]
    zs = [r[T_Z] for r in rows]
    return {"window": [round(t_lo, 3), round(t_hi, 3)],
            "n_frames": len(fs),
            "n_offboard": sum(1 for n in nav if n == v.NAV_STATE_OFFBOARD),
            "n_truth": len(rows),
            "z_min": (min(zs) if zs else None),
            "z_max": (max(zs) if zs else None),
            "ran": len(fs) >= 10 and len(rows) >= 10,
            "offboard_kept": bool(fs) and all(n == v.NAV_STATE_OFFBOARD for n in nav),
            "air": bool(zs) and min(zs) > GROUND_M}


def _health_detail(h: dict) -> str:
    """把 `_window_health` 的三条读成一句话 —— 判据行的 detail 与没做行共用。"""
    return (f"窗口 [{v.fmt(h['window'][0], '.3f')}, {v.fmt(h['window'][1], '.3f')}] s（墙钟）；"
            f"DUT 帧 {h['n_frames']} 帧（其中 OFFBOARD {h['n_offboard']} 帧）、"
            f"真值 {h['n_truth']} 点；真值高度区间 "
            f"[{v.fmt(h['z_min'], '+.3f')}, {v.fmt(h['z_max'], '+.3f')}] m"
            f"（最低点 > {GROUND_M} ⟹ 不撞地={h['air']}）")


def fallback_report(frames) -> dict:
    """兜底阶梯的**现场**：档位分布、切换顺序、注入生效点、hover 那一档的推力。

    ★ 判据④ 的"M5 兜底生效"就是这一张表 —— 而且它有一个**结构上不可伪造**的证据：
      悬停帧的 `thrust_body_z` **恒等于 −hover_thrust**（`mpc_node.py:405`：
      `px4_rates_setpoint((0,0,0), t_hover, t_max)`）⟹ 日志里那一列一旦变成这个常数，
      就只能来自兜底（正常解不可能恰好钉在它上面）。
    """
    fs = frames or []
    dist = {}
    for f in fs:
        dist[f.get("fallback")] = dist.get(f.get("fallback"), 0) + 1
    order, prev = [], None
    for f in fs:
        cur = f.get("fallback")
        if cur != prev:
            order.append(str(cur))
            prev = cur
    hold = [f for f in fs if f.get("fallback") == "hold"]
    hover = [f for f in fs if f.get("fallback") == "hover"]
    tz = [f.get("thrust_body_z") for f in hover if f.get("thrust_body_z") is not None]
    notes = sorted({str(f.get("fallback_note") or "") for f in (hover or hold)} - {""})
    out = {"dist": dist, "order": order, "n_hold": len(hold), "n_hover": len(hover),
           "hover_thrust_body_z_seen": sorted({round(float(x), 5) for x in tz})[:4],
           "notes": notes[:3]}
    if hover:
        t0 = hover[0]["wall_ns"] / 1e9
        out["hover_start_wall_s"] = round(t0, 3)
        out["hover_span_s"] = round((hover[-1]["wall_ns"] - hover[0]["wall_ns"]) / 1e9, 2)
    if hold:
        out["hold_span_s"] = round((hold[-1]["wall_ns"] - hold[0]["wall_ns"]) / 1e9, 3)
    return out


def divergence(truth, t_lo: float, t_hi: float, t_air_abs=None) -> dict:
    """"发不发散"的读数（判据③/④）：末段垂直速度、端段高度峰峰值、离地段最低点、漂移。

    ★ 票面只说"不能发散"，没有给数 ⟹ 这里**把实现方的判断写出来**（阈值见模块顶部
      `DIVERGE_*` / `GROUND_M`），而不是含糊地写"基本稳定"。

    ★★ `t_air_abs`（离地那一刻的**绝对墙钟**）不是可有可无的：判"没触地"要拿**离地段**
      的最低点，不能拿整条真值的（`z_min_all`）。真值是从回放启动就开录的，飞机那会儿
      **正停在地面上**（z ≈ 0）⟹ 用 `z_min_all` 判 `> GROUND_M` ，**每一场都必然 ❌**
      （连飞得最好的那一场也是），而报告会把它写成"触地了"。这正是
      `test_tier_b_judges.py::test_判据③的好飞机不该被地面段判成触地` 钉住的那一条。
      两个数都留在读数里：`z_min_air` 用来判，`z_min_all` 只作上下文。
    """
    rows = [r for r in truth if t_lo <= r[T_WALL] <= t_hi]
    if len(rows) < 5:
        return {"n": len(rows)}
    zs, _tail, vz = tail_vz(rows)
    hxy = [math.hypot(r[T_X] - rows[0][T_X], r[T_Y] - rows[0][T_Y]) for r in rows]
    all_z = [r[T_Z] for r in truth] if truth else zs
    air_z = [r[T_Z] for r in truth if t_air_abs is None or r[T_WALL] >= t_air_abs]
    return {"n": len(rows), "span_s": round(t_hi - t_lo, 2),
            "vz_mean": statistics.fmean(vz) if vz else 0.0,
            "vz_absmax": max(abs(x) for x in vz) if vz else 0.0,
            "z_p2p": max(zs) - min(zs), "z_mean": statistics.fmean(zs),
            "h_drift": hxy[-1] - hxy[0], "h_max": max(hxy),
            "z_min_all": min(all_z), "z_max_all": max(all_z),
            # 离地段（真值里绝对墙钟 ≥ 离地时刻的那些点）
            "air_n": len(air_z),
            "z_min_air": min(air_z) if air_z else None,
            "z_max_air": max(air_z) if air_z else None}


# ------------------------------------------------------------------ 判据层


def _judge_traj(ck, obs, seg) -> dict:
    """判据② —— 两条轨迹，**定性判**：跑完 + 不撞地 + 不失控；跟踪误差**只留档**。

    ★★ 口径的来历（操作者 2026-09-30 裁定，原话在 11 号票 Comments）：
      · **去掉「1 m 正方形」**：`bag/` 里没有那份包（四份场景逐条见 `bag/README.md`），
        spec 那一行本来也只剩「悬停 / r=0.5 m 八字」两条；
      · spec 原文（`docs/control/phase2-implementation.md`）对这一条只有
        「**不撞地、不失控**」，**没有数值界** ⟹ **不编阈值**。跟踪误差照样算、
        照样进 `obs.json`（`traj_err_metrics`），但它**不当判据**。

    ⚠️ 这里原来是一行**硬编码的 `False`**（"未达标 —— 缺口是两个，都不在本票手里"）。
      那是 **17 号票还没落地**时的写法：当时节点没有消费 `/traj/reference` 的口，
      八字那条物理上不可能跑。17 号票收口（`83422f9`）之后那条口接通了，这行必须换成
      **真判** —— 一行恒 ❌ 的判据在矩阵里会**掩盖**真实结果（真跑绿了也看不出来），
      而同一条判据在 `--replay` 下又永远不会变，属于最坏的那类静默失败。

    ★★ 为什么**两条轨迹各由一档场次来判**、另一条当场记「本场不做」：
      悬停那条由 `--reference-source hold` 的场次判 —— F0–F4 就是它，也就是 Phase 1
      轨迹 1 的真身（那几场的参考恒为锚点、飞机原地悬停）；
      八字那条只有 `--reference-source traj` 的场次才**可能**判。
      一场跑不出两条 ⟹ 若两条都 `counts=True`，**每一场都会凭空多一条不通过**，
      而那是"这一场没跑它"、不是"它不通过"。所以另一条 `counts=False`（`ok=None`）：
      既不冒充通过、也不冒充不通过，与判据③「这一条没做」是同一条三分法。
    ★ 「本场不做」的那条**不进 `counts`**，但**进报告**：矩阵汇总时要靠它认出
      "两条轨迹各由哪几场覆盖"，否则「判据② 全绿」会与「判据② 只跑了一条」长得一样。
    """
    frames = obs.get("frames") or []
    truth = obs.get("truth") or []
    src = obs.get("reference_source") or "hold"
    tt = obs.get("t") or {}
    lo = seg.get("liftoff_t") if isinstance(seg, dict) else None
    #: 离地的**绝对墙钟**。★★ 窗口必须从这一刻起：OFFBOARD 起手那一刻飞机**还在地面**
    #: （`offboard_enter_t` 是"模式生效"、不是"离地"），而判据② 的"不撞地"是
    #: `窗口内最低点 > GROUND_M` —— 窗口若从 OFFBOARD 起算，**任何一场都会读成撞地**。
    #: 真值第 7 列（`T_WALL`）才是绝对墙钟，第 0 列是流自己的时基：两者相加不能写反。
    t_air_abs = (truth[0][T_WALL] + lo) if (truth and lo is not None) else None
    out = {}

    def _flight_window():
        """控制住的那段窗口（绝对墙钟）—— 判据② 只在窗口里读，不读停机后的自由落体。"""
        ov, end = tt.get("offboard_enter_t"), tt.get("flight_end_t")
        if all(isinstance(x, (int, float)) for x in (ov, end)) and end > ov:
            return float(ov), float(end), "OFFBOARD 起 → 收手前"
        fs = [f for f in frames if f.get("wall_ns")]
        if len(fs) < 2:
            return None
        return (fs[0]["wall_ns"] / 1e9, fs[-1]["wall_ns"] / 1e9,
                "DUT 首帧 → 末帧（☠️ 旧 obs 没有 `offboard_enter_t` 时的退路；"
                "它含停机后那一段，读数偏悲观）")

    # ── 轨迹 1：原地悬停（Phase 1 轨迹 1；由 `hold` 档的场次判）
    if src != "hold":
        ck("★判据②·轨迹1 原地悬停（定性：跑完 + 不撞地 + 不失控）", None,
           f"本场 `--reference-source {src}` ⟹ **这一条没做**（不是通过也不是不通过）。"
           f"★ 判它的场次是 `--reference-source hold` 那几场 —— 那一档的参考恒为锚点，"
           f"飞机原地悬停，就是 Phase 1 轨迹 1 的真身。", counts=False)
        out["hover"] = {"ran": False, "why": "本场是 traj 档"}
    else:
        w = _flight_window()
        if w is None:
            ck("★判据②·轨迹1 原地悬停（定性：跑完 + 不撞地 + 不失控）", None,
               "没有逐帧日志也没有真值 ⟹ **这一条判不了**，不当作通过也不当作不通过",
               counts=False)
        else:
            h = _window_health(frames, truth, max(w[0], t_air_abs or w[0]), w[1])
            out["hover"] = h
            tilt = (f"全程最大倾角 {v.fmt(obs.get('tilt_deg_max'), '.2f')}°（|·| < "
                    f"{LOSS_TILT_DEG:g}° 才是「没翻过去」⇔ 未失控，**旁证，不设界**）")
            if not h["ran"]:
                ck("★判据②·轨迹1 原地悬停（定性：跑完 + 不撞地 + 不失控）", None,
                   f"{_health_detail(h)}；窗口（{w[2]}）里数据不够 ⟹ **这一条判不了**"
                   f"（仪器没覆盖到窗口，不是飞机飞得不好）", counts=False)
            else:
                ok = bool(h["offboard_kept"] and h["air"] and lo is not None)
                ck("★判据②·轨迹1 原地悬停（定性：跑完 + 不撞地 + 不失控）", ok,
                   f"{_health_detail(h)}；起飞于 t={v.fmt(lo, '.2f')}s（真值流时基）；"
                   f"窗口取法 {w[2]}；{tilt}。"
                   f"★ 判的是**三条定性**（跑完 {h['offboard_kept'] and h['ran']}、"
                   f"不撞地 {h['air']}、不失控 {h['offboard_kept']}）；"
                   f"精度那条在 **★判据①**，不在这里重复。")

    # ── 轨迹 2：r = 0.5 m 八字（由 `traj` 档的场次判；参考 = 夹具重发包里的 `/traj/reference`）
    tl = obs.get("reference_timeline") or {}
    if src != "traj":
        ck("★判据②·轨迹2 r = 0.5 m 八字（定性：跑完 + 不撞地 + 不失控；跟踪误差只留档）",
           None,
           f"本场 `--reference-source {src}` ⟹ **这一条没做**（不是通过也不是不通过）。"
           f"★ 判它的场次是 `--reference-source traj` 那几场（夹具按 `bag/"
           f"4_traj3_lemniscate` 里的 `/traj/reference` 逐点重发；高度由 `--ref-z-offset-m` "
           f"叠在夹具侧）。", counts=False)
        out["lemniscate"] = {"ran": False, "why": "本场是 hold 档"}
    elif not tl.get("n"):
        ck("★判据②·轨迹2 r = 0.5 m 八字（定性：跑完 + 不撞地 + 不失控；跟踪误差只留档）",
           None,
           f"本场是 traj 档，但**参考时间线读不到**（{tl.get('why') or '没有日志'}）"
           f"⟹ **这一条判不了** —— 「八字到底发出去没有、发到哪一刻」全靠这条日志，"
           f"没有它就只剩一句「应该发了」。★ 不当作通过也不当作不通过", counts=False)
        out["lemniscate"] = {"ran": False, "why": tl.get("why") or "没有日志"}
    elif not tl.get("n_moving"):
        ck("★判据②·轨迹2 r = 0.5 m 八字（定性：跑完 + 不撞地 + 不失控；跟踪误差只留档）",
           None,
           f"参考时间线里**没有一帧在动**（共发 {tl.get('n')} 帧，全在原点）⟹ **这一条没做**。"
           f"★ 这本身就是夹具的**征兆**：包里那条参考的移动段在 `bag/4_traj3_lemniscate` 里"
           f"是前 305 帧\n（t ≤ 6.10 s），发成不动只有两种可能 —— 模型文件里 `reference` "
           f"块是空的，或者 `--publish-reference` 没接上。", counts=False)
        out["lemniscate"] = {"ran": False, "why": "参考全程不动"}
    else:
        #: `r_lo/r_hi` = 参考**自己**的移动段（判"跑完"用），`m_lo/m_hi` = 判据**读数的窗口**
        #: （从离地那一刻起，见上）。两者分开：把窗口下界抬到离地会让"参考在离地前就动了"
        #: 这件事**看不出来**，而那正是"八字没跑完"的一种。
        r_lo, r_hi = tl["move_first_s"], tl["move_last_s"]
        m_lo, m_hi = max(r_lo, t_air_abs or r_lo), r_hi
        h = _window_health(frames, truth, m_lo, m_hi)
        er = traj_err_metrics(frames, truth, m_lo, m_hi)
        out["lemniscate"] = {**h, "ref": tl, "err": er}   # ← 误差读数随判据块留档
        ov, end = tt.get("offboard_enter_t"), tt.get("flight_end_t")
        # 跑完的**接线**判据：移动段必须整段落在 OFFBOARD 窗口里。
        # ★ 只有 ov/end 都在才判它（旧 obs 没有 ⟹ 这一项 `None`，不冒充通过）。
        fits = (None if not all(isinstance(x, (int, float)) for x in (ov, end))
                else bool(r_lo >= ov and r_hi <= end))
        tilt = (f"全程最大倾角 {v.fmt(obs.get('tilt_deg_max'), '.2f')}°"
                f"（旁证，不设界；★ 它含停机后那一段，别拿它当窗口判据）")
        head = (f"参考移动段 [{v.fmt(r_lo, '.3f')}, {v.fmt(r_hi, '.3f')}] s，"
                f"长 {v.fmt(tl.get('move_span_s'), '.2f')} s、共 {tl['n_moving']} 帧"
                f"（时间线共 {tl['n']} 帧，相位分布 {tl.get('by_phase')}）；"
                f"OFFBOARD 窗口 [{v.fmt(ov, '.3f')}, {v.fmt(end, '.3f')}] s ⟹ "
                f"参考移动段整段落在窗口内={fits}（★ 判的是**参考**那一段，不是窗口；"
                f"读数窗口另从离地那一刻算，见下）；{_health_detail(h)}；{tilt}。")
        if not h["ran"]:
            ck("★判据②·轨迹2 r = 0.5 m 八字（定性：跑完 + 不撞地 + 不失控；跟踪误差只留档）",
               None,
               f"{head}★ 移动段里数据不够 ⟹ **这一条判不了**（仪器没覆盖到窗口，"
               f"不是飞机飞得不好）", counts=False)
        else:
            ck("★判据②·轨迹2 r = 0.5 m 八字（定性：跑完 + 不撞地 + 不失控；跟踪误差只留档）",
               bool(h["offboard_kept"] and h["air"] and fits is not False),
               f"{head}**跟踪误差（只留档，spec 没有数值界 ⟹ 不设判据）**："
               f"真值 − DUT 自己的 `x_ref0`，水平均值 "
               f"{v.fmt(er.get('h_mean'), '.4f')} m / 最大 {v.fmt(er.get('h_max'), '.4f')} m、"
               f"3D 均值 {v.fmt(er.get('d3_mean'), '.4f')} m（配对上 "
               f"{er.get('n_paired')}/{er.get('n')} 点，容差 ±{PAIR_TOL_S:g} s）。"
               f"★ 判的是**三条定性**（跑完 {fits}、不撞地 {h['air']}、"
               f"不失控 {h['offboard_kept']}）；误差那一列**只是留档**，"
               f"高不影响 ok、低也不加分 —— 操作者 2026-09-30 裁定「不编阈值」。")
    return out


def evaluate_b(obs) -> list:
    """把 B 档的观测翻成 seam 3 的四条判据。**仪器与判据分开**（`--replay` 不重跑 SITL）。

    ⚠️ **它不是纯函数，尽管名字像**：判的过程要把几个派生块**写回 `obs`**
    （`fallback_report` / `att_err_deg` / `steady` / `osc` / `diverge` / `m5` / `traj`），
    因为 `obs.json` 要留档的正是这些块（文档 §5/§7/§8 的数字就是从它们来的），
    而判据行只回答"过没过"、不重复装数字。
    两个后果，用之前先看一眼：
      · **同一个 obs 判两次，第二次读的是第一次写下的块**（`att_err_deg` 这条会直接复用，
        不再重算）⟹ 想拿"改动后重判"的干净结果，**先 `obs.pop("checks")` 并把派生块删掉**，
        或者重新跑一场；
      · 传进来的 dict 会被改。要留原件就别把原件递进来。
    ★ 这条**偏离**了本仓"判据函数写成纯函数"的惯例，是有意的（约定 vs 留档口径打架时选了留档），
      所以在这里点名写清，而不是让下一个人从行为里去猜。
    """
    c = []

    def ck(name, ok, detail, counts=True):
        c.append({"name": name, "ok": bool(ok) if ok is not None else None,
                  "detail": detail, "counts": counts})

    frames = obs.get("frames") or []          # ★ 逐帧日志（不是 `dut.frames` 那个**计数**）
    truth = obs.get("truth") or []
    solve = obs.get("solve") or {}
    fb = fallback_report(frames)
    obs["fallback_report"] = fb
    ph = obs.get("plant_hover") or {}

    # ⓪ 仪器自检：夹具真的在发、真值流真的在流、目标是锚死的
    ck("⓪ 仪器自检：回放夹具在发帧、真值流在流、DUT 的每一帧都认得状态源",
       bool(obs.get("replay_up")) and obs.get("truth_n", 0) > 100
       and (obs.get("dut") or {}).get("state_source") == "mocap",
       f"夹具发出 {obs.get('t', {}).get('replay_frames_at_dut_start')} 帧（DUT 起手那一刻）、"
       f"本场共 {obs.get('replay', {}).get('frames')} 帧；"
       f"真值 {obs.get('truth_n')} 点 @ {v.fmt(obs.get('truth_hz'), '.1f')} Hz"
       f"（姿态 {obs.get('truth_att_n')} 点）；"
       f"回放延迟 基 {v.fmt((obs.get('replay') or {}).get('base_delay_ms'), '.2f')} ms + "
       f"注入 {v.fmt(obs.get('extra_delay_ms'), '.1f')} ms = "
       f"{v.fmt((obs.get('replay') or {}).get('delay_ms'), '.2f')} ms；"
       f"DUT 的 state_source={(obs.get('dut') or {}).get('state_source')}")

    # ⓪b 仪器自检：DUT **报出来的姿态**与同一时刻真值一致 —— 钉住"夹具→话题→节点→x0"
    #   这一整条接线。分量序写反不会报错、不会 NaN，只会让姿态整体偏 90°（见
    #   `att_err_deg` 的 ★★ 与首飞那次）。按已知回放延迟回退真值再配对。
    # `_tap` 在 ⟹ 现算；不在但有**存下来的读数** ⟹ 用存下来的（`--replay` 复判那条路，
    # 读数随 JSON 一起落盘就是为了让 JSON 自给自足）；两样都没有 ⟹ 判不了。
    if "_tap" not in obs and obs.get("att_err_deg"):
        ae = obs["att_err_deg"]
        ck("⓪b 仪器自检：DUT 报出的姿态与同一时刻真值一致（中位数 ≤ 10°）",
           ae.get("n", 0) > 0 and ae["median"] <= 10.0,
           f"**取自本场落盘的读数**（`--replay` 复判：配对用的真值流不在，但读数在）；"
           f"配到 {ae.get('n')} 帧；夹角 中位数 {v.fmt(ae.get('median'), '.2f')}°、"
           f"p95 {v.fmt(ae.get('p95'), '.2f')}°、最大 {v.fmt(ae.get('max'), '.2f')}°")
    elif "_tap" in obs:
        ae = att_err_deg(obs["_tap"], frames,
                         delay_s=(obs.get("replay") or {}).get("delay_ms", 0.0) / 1e3)
        obs["att_err_deg"] = ae
        ck("⓪b 仪器自检：DUT 报出的姿态与同一时刻真值一致（中位数 ≤ 10°）",
           ae.get("n", 0) > 0 and ae["median"] <= 10.0,
           f"配到 {ae.get('n')} 帧（按已知延迟 "
           f"{v.fmt((obs.get('replay') or {}).get('delay_ms'), '.2f')} ms 回退真值）；"
           f"夹角 中位数 {v.fmt(ae.get('median'), '.2f')}°、p95 {v.fmt(ae.get('p95'), '.2f')}°、"
           f"最大 {v.fmt(ae.get('max'), '.2f')}°。"
           f"★ **整场偏 120°** 就是**分量序**写反了（wxyz ↔ xyzw）：错出来的是「绕 x 转 90°」，"
       f"而对的应该是「绕 z 转 90°」，两个转轴正交 ⟹ 相对角正是 120°（不是 90°）。"
       f"这一条与控制器无关")
    else:
        # `--replay` 复判那条路：观测是从 JSON 读回来的，配对用的真值不在了。
        # ⟹ 判不了，**不计数**（与"不通过"分开，见 docstring 的三分法）。
        ck("⓪b 仪器自检：DUT 报出的姿态与同一时刻真值一致（中位数 ≤ 10°）", None,
           "本场是**从 JSON 复判**的：配对用的真值流不在，这一条判不了"
           "（要它就跑真场次，别用 `--replay` 复判）", counts=False)

    # ① 中间件
    ck("① 中间件：DUT 发得出帧、PX4 解锁并进了 OFFBOARD",
       (obs.get("dut") or {}).get("frames", 0) > 0 and obs.get("armed_ever")
       and obs.get("t", {}).get("offboard_enter") is not None,
       f"DUT {(obs.get('dut') or {}).get('frames', 0)} 帧"
       f"（{v.fmt((obs.get('dut') or {}).get('actual_hz'), '.1f')} Hz）、"
       f"armed={obs.get('armed_ever')}、OFFBOARD 于 t="
       f"{v.fmt(obs.get('t', {}).get('offboard_enter'), '.2f')}s；{obs.get('agent')}")
    if not (obs.get("dut") or {}).get("frames"):
        ck("☠ DUT 一帧都没发 —— 后面所有判据都无从判（先看 stdout 尾巴）", False,
           (obs.get("stdout_tail") or "")[-400:])
        return c

    # 参考锚定（★ 与 A 档同一条，排在行为之前）：B 档的锚点来自**合成流**，
    # 所以"目标没跟着测量走"这件事在 B 档要**再证一遍** —— 回放噪声一旦漏进参考，
    # 参考就会变成"测量 + 偏移"，位置误差恒不收敛。
    tr = (obs.get("dut") or {}).get("track") or {}
    ref_span = tr.get("ref_span_z")
    ck("① 参考锚定：目标全程没跟着测量走（`x_ref0` 钉在锚点上）",
       ref_span is not None and ref_span < 0.3,
       f"`x_ref0[2]` 全程跨度 {v.fmt(ref_span, '.4f')} m（该接近 0）、"
       f"水平跨度 {v.fmt(tr.get('ref_span_xy'), '.4f')} m；锚点事件 "
       f"{(obs.get('dut') or {}).get('ref_events')}；参考误差头 "
       f"{v.fmt(tr.get('referr_head'), '+.3f')} m ⟹ 末段 {v.fmt(tr.get('referr_tail'), '+.3f')} m。"
       f"★ B 档尤其要盯着这一条：参考若漏进回放噪声，误差就会一直不收敛，"
       f"而症状看起来像「控制器不行」")

    # ② 行为：真值高度真的升上去了（ENU ⟹ 阈值取正）
    # ★ `rising=True`：B 档真值是 ENU（z 向上为正），而 `ta.segments` 的默认判法是给
    #   A 档的 NED 写的（第一帧 `z < liftoff_cross`）。不传就是静默取反 —— 见那里 docstring。
    seg = ta.segments(truth, liftoff_cross=0.15, hover_s=STEADY_S, rising=True)
    ck("② 行为：真值高度真的升上去了（起飞后 5 s 内 z > +0.4 m，ENU）",
       seg["liftoff_t"] is not None
       and any(r[3] > 0.4 for r in seg["climb"]),
       f"真值首点 z={v.fmt((truth[0][3] if truth else None), '+.4f')} m（地面）⟹ 第一次越过 "
       f"+0.15 m 于 t={v.fmt(seg['liftoff_t'], '.2f')}s；起飞段最高 "
       f"{v.fmt(max((r[3] for r in seg['climb']), default=None), '+.3f')} m。"
       f"★ ENU 的 z 是**向上为正**，所以判据的符号与 A 档（NED）相反")

    # ③ 判据①：稳态位置误差 < 0.15 m（票面原话）
    bounds = steady_bounds(frames)
    if bounds is None:
        ck("★判据① 稳态位置误差 < 0.15 m", None,
           "没有逐帧日志（`frames` 空）⟹ **这一条没做**，不当作通过也不当作不通过",
           counts=False)
        return c
    t_lo, t_hi = bounds
    target = ref_target(frames, t_lo, t_hi)
    em = err_metrics(truth, target, t_lo, t_hi, "判据①稳态窗口")
    obs["steady"] = em

    # ★★ 前置（仪器自检，**计数**）——与 A 档 ⑨ 的前置同一个口径：真值流在稳态窗口里够密。
    #   为什么要单列而不是并进判据①：数据不够与"控制器没控住"是**两件**事。并进判据① 的话，
    #   真值流断掉一场会被读成「判据① 不通过」，而那条最容易被当成本档的闸的结论。
    #   所以：仪器这一条**计入**（本场不能读成"全部通过"），判据① 本身记「没做」不计数。
    dense = (em.get("n") or 0) >= 10
    ck("★判据① 前置：真值流在稳态窗口里够密（**仪器自检**；判 ❌ = 本场不能算通过，"
       "但归因**不指向**控制器）", dense,
       f"稳态窗口 [{v.fmt(t_lo, '.3f')}, {v.fmt(t_hi, '.3f')}] s（墙钟）里有 "
       f"{em.get('n')} 点真值（该 ≥ 10；本场真值 {obs.get('truth_n')} 点 @ "
       f"{v.fmt(obs.get('truth_hz'), '.1f')} Hz ⟹ {STEADY_S:g}s 该有 "
       f"{v.fmt((obs.get('truth_hz') or 0) * STEADY_S, '.0f')} 点）。"
       f"★ 不过 ⟹ 真值流没覆盖到窗口（录得比 DUT 早停？或 `truth` 列被裁过），"
       f"本场的判据①/③ 读数**都不作数**；这与飞机飞得好不好无关")
    # ★★ 前置三：稳态窗口必须**完全落在正常帧里**。
    #   窗口是"最后一次正常帧往前 8 s"；判据④ 那两场在 700 帧（7 s）处注入 ⟹ 正常帧总长
    #   只有 ~7 s < 8 s ⟹ 窗口**必然把起飞爬升段圈进来**。实测后果：F4 那一场判据① 读成
    #   `z_mean −0.175 m / z_p2p 1.507 m`，看着像"悬停精度很差"，真相是"窗口里有半段在爬升"
    #   —— 状态机与采样**都对**，是**这一场本来就不是悬停精度场**。
    #   ⟹ 按既有的三分法记「没做」+ **不计数**（与判据③ 在没加延迟那场的处置同一条），
    #     否则一个设计使然的窗口会被读成控制器的罪证。
    normal_fs = [f for f in frames if f.get("fallback") == "normal" and f.get("wall_ns")]
    norm_span_s = ((normal_fs[-1]["wall_ns"] - normal_fs[0]["wall_ns"]) / 1e9
                   if len(normal_fs) >= 2 else 0.0)
    window_fits = norm_span_s >= STEADY_S
    if not dense:
        ck("★判据① 稳态位置误差 < 0.15 m", None,
           f"真值在窗口里只有 {em.get('n')} 点（见上一条 ★判据① 前置）⟹ **这一条没做**，"
           f"不当作通过也不当作不通过", counts=False)
    elif not window_fits:
        ck(f"★判据① 稳态位置误差 < {ERR_TOL_M} m", None,
           f"正常帧总长只有 {v.fmt(norm_span_s, '.2f')}s < {STEADY_S:g}s ⟹ 稳态窗口必然含"
           f"起飞爬升段，本场的判据① 读数**不作数**（不是通过也不是不通过）。"
           f"★ 本场是**判据④ 的注入场**（`inject_solve_fail_after_frames="
           f"{obs.get('inject_solve_fail_after_frames')}` ⟹ 悬停只跑了 "
           f"{v.fmt(norm_span_s, '.2f')}s 就注入）—— 这是**这一场的设计**，不是控制器的事。"
           f"要看悬停精度请读 `--mismatch mass+30`/`none` 那两场（正常帧 ~15 s）",
           counts=False)
    else:
        ck(f"★判据① 稳态位置误差 < {ERR_TOL_M} m"
           f"（真值帧；窗口 = 最后一次正常帧往前 {STEADY_S:g} s）",
           em["h_mean"] < ERR_TOL_M and em["z_absmean"] < ERR_TOL_M,
           f"窗口 {v.fmt(t_hi - t_lo, '.2f')}s / {em['n']} 点；"
           f"水平误差 均值 {v.fmt(em.get('h_mean'), '.4f')} m、最大 {v.fmt(em.get('h_max'), '.4f')} m；"
           f"垂直误差 均值 {v.fmt(em.get('z_mean'), '+.4f')} m（绝对值均值 "
           f"{v.fmt(em.get('z_absmean'), '.4f')} m）、最大 {v.fmt(em.get('z_max'), '.4f')} m；"
           f"3D 均值 {v.fmt(em.get('d3_mean'), '.4f')} m。目标（节点自己 `x_ref0` 的中位数）"
           f"{em.get('target_enu')}。★ 判**均值**（稳态二字就是这个意思）：抖动用下一条判")

    # ④ 无持续振荡（票面"无持续振荡"）—— 沿用 A 档 `evaluate` 的④ 阈值，两档读数能并排看
    hov = seg["hover"]
    osc_z = ta.crossings(hov, idx=3)
    # ★ 水平那一路在**悬停段内**算（不是全场）：全场的水平误差被起飞段主导，
    #   穿均值频率量到的是"起飞那一下"，不是"悬停抖不抖"。
    tgt = target or [0.0, 0.0, 0.0]
    osc_h = (ta.crossings_series([math.hypot(r[1] - tgt[0], r[2] - tgt[1]) for r in hov],
                                 hov[-1][0] - hov[0][0]) if len(hov) > 8 else None)
    obs["osc"] = {"z_hz": osc_z, "h_hz": osc_h, "hz_limit": (obs.get("truth_hz") or 0) / 2}
    OSC_NAME = f"★判据①·振荡 无持续振荡：末段高度峰峰值 < {OSC_P2P_M} m 且穿均值频率 < {OSC_HZ} Hz"
    if osc_z is None:
        # 同样三分法：`crossings` 要 8 点以上才算得出频率。点不够是**仪器**的事
        # （真值没覆盖到末段），不是"抖得厉害" —— 记 ❌ 会把它读成后者的反面。
        ck(OSC_NAME, None,
           f"末段只有 {len(hov)} 点真值（`crossings` 要 ≥ 8 点）⟹ **这一条没做**；"
           f"见 ★判据① 前置那一行的仪器自检", counts=False)
    else:
        z_p2p = max(r[3] for r in hov) - min(r[3] for r in hov)
        ck(OSC_NAME, z_p2p < OSC_P2P_M and osc_z < OSC_HZ,
           f"末段 {len(hov)} 点 / {v.fmt(hov[-1][0] - hov[0][0], '.2f')}s："
           f"高度峰峰值 {v.fmt(z_p2p, '.4f')} m、"
           f"穿均值 {v.fmt(osc_z, '.2f')} Hz；水平误差穿均值 {v.fmt(osc_h, '.2f')} Hz。"
           f"★★ 本档真值流是**全速**（{v.fmt(obs.get('truth_hz'), '.1f')} Hz ⟹ 分辨力 "
           f"{v.fmt((obs.get('truth_hz') or 0) / 2, '.1f')} Hz）—— A 档真值只有 4 Hz、"
           f"连自己 2 Hz 的门槛都够不到，只能退回 EKF 帧判；B 档**判据与仪器同一帧**")
    ck("① 「没失控」的旁证：真值倾角（与朝向无关 ⟹ 补上「机头在自转而位置判据全绿」那个盲区）",
       None,
       f"全程最大倾角 {v.fmt(obs.get('tilt_deg_max'), '.2f')}°；"
       f"稳态窗口内最大 {v.fmt(obs.get('tilt_deg_max_steady'), '.2f')}°"
       f"（姿态真值 {obs.get('truth_att_n')} 点）。★ A 档 2026-09-28 实测过"
       f"「飞机自转 5 圈而位置判据全过」，所以这一列与位置那几条是**互相独立**的两条证据；"
       f"本行不设阈值（悬停时该接近 0°，判据① 的位置误差已经覆盖了「稳不稳」这件事）",
       counts=False)

    # ⑤ 判据③：注入 50 ms 延迟后仍稳定（可变差、不可发散）
    # ★ 离地时刻换到**绝对墙钟**：真值第 0 列是流自己的相对时基，第 7 列才是配对键。
    t_air_abs = (truth[0][7] + seg["liftoff_t"]) if (truth and seg.get("liftoff_t") is not None) else None
    di = divergence(truth, t_lo, t_hi, t_air_abs=t_air_abs)
    obs["diverge"] = di
    if obs.get("extra_delay_ms", 0) <= 0:
        ck("★判据③ 注入 50 ms 动捕延迟后仍稳定（可变差、不可发散）", None,
           f"本场 `--extra-delay-ms {v.fmt(obs.get('extra_delay_ms'), '.1f')}` ⟹ **这一条没做**"
           f"（不是通过也不是不通过）。要判它就跑 `--extra-delay-ms 50`", counts=False)
    elif di.get("n", 0) < 5:
        ck("★判据③ 注入延迟后仍稳定（可变差、不可发散）", None,
           f"这一场做了延迟（{v.fmt(obs.get('extra_delay_ms'), '.1f')} ms），但稳态窗口里真值只有 "
           f"{di.get('n')} 点 ⟹ **这一条判不了**（仪器没覆盖到窗口，见 ★判据① 前置）；"
           f"不当作通过也不当作不通过", counts=False)
    else:
        ck(f"★判据③ 注入 {v.fmt(obs.get('extra_delay_ms'), '.1f')} ms 延迟后仍稳定"
           f"（可变差、不可发散）",
           abs(di["vz_mean"]) < DIVERGE_VZ_MS
           and di["z_p2p"] < DIVERGE_P2P_M and di["z_min_air"] is not None
           and di["z_min_air"] > GROUND_M and abs(di["h_drift"]) < 0.5,
           f"延迟 基 {v.fmt((obs.get('replay') or {}).get('base_delay_ms'), '.2f')} + 注入 "
           f"{v.fmt(obs.get('extra_delay_ms'), '.1f')} = "
           f"{v.fmt((obs.get('replay') or {}).get('delay_ms'), '.2f')} ms；"
           f"末段垂直速度均值 {v.fmt(di.get('vz_mean'), '+.3f')} m/s（|·| < {DIVERGE_VZ_MS}）、"
           f"末段高度峰峰值 {v.fmt(di.get('z_p2p'), '.4f')} m（< {DIVERGE_P2P_M}）；"
           f"**离地段**高度区间 [{v.fmt(di.get('z_min_air'), '+.3f')}, "
           f"{v.fmt(di.get('z_max_air'), '+.3f')}] m（{di.get('air_n')} 点，最低点 > {GROUND_M} "
           f"= 没触地）；末段水平净漂移 {v.fmt(di.get('h_drift'), '+.3f')} m。"
           f"★ 判**离地段**的最低点而不是整条真值的（整条含起飞前停在地面那几秒，"
           f"那个 `z_min_all` = {v.fmt(di.get('z_min_all'), '+.3f')} m 恒 ≈ 0 ⟹ 拿它判就等于"
           f"每场必 ❌）。"
           f"★ 阈值是**实现方的判断**（票面只说「不能发散」），写在这里以便复核")

    # ⑥ 判据④：故意让求解器失败，飞机保持安全悬停而不是坠机
    if obs.get("inject_solve_fail_after_frames", 0) <= 0:
        ck("★判据④ 求解器失败后保持安全悬停（M5 兜底）", None,
           f"本场 `inject_solve_fail_after_frames="
           f"{obs.get('inject_solve_fail_after_frames')}` ⟹ **这一条没做**。"
           f"要判它就跑 `--inject-solve-fail-after-frames N`（N 的含义 = 「已发布 N 帧之后」）",
           counts=False)
    else:
        _judge_m5(ck, obs, fb, ph, truth)

    # ⑦ D6：票面那个数字（结论行，不判通过）
    ck("⑦ D6（结论行）：求解耗时 p50/p99/最坏 —— 冷启动与稳态分账",
       None,
       f"n={solve.get('n')} ⟹ p50 {v.fmt(solve.get('p50'), '.3f')} ms / "
       f"p99 {v.fmt(solve.get('p99'), '.3f')} ms / **全集最坏 {v.fmt(solve.get('max'), '.3f')} ms**"
       f"；去掉 {solve.get('warmup_n')} 拍 `warmup` 之后（n={solve.get('steady_n')}）："
       f"p50 {v.fmt(solve.get('steady_p50'), '.3f')} / p99 {v.fmt(solve.get('steady_p99'), '.3f')}"
       f" / **稳态最坏 {v.fmt(solve.get('steady_max'), '.3f')} ms**。"
       f"按票面字面 ⟹ {solve.get('ladder') or '（没有样本）'}；"
       f"按稳态 ⟹ {solve.get('ladder_steady') or '（没有样本）'}。"
       f"★ **交给 Phase 3 的「实测最坏求解时间」取 `steady_max`**"
       f"（冷启动那一拍是 acados 的一次性开销，不是持续负载）",
       counts=False)
    ck("⑦ D6 的另一半：这台机器真跑出多少 Hz（逐拍间隔）",
       solve.get("tick_p50_ms") is not None and solve["tick_p50_ms"] < 12.0,
       f"逐拍间隔 p50 {v.fmt(solve.get('tick_p50_ms'), '.3f')} ms / "
       f"p99 {v.fmt(solve.get('tick_p99_ms'), '.3f')} ms / 最坏 "
       f"{v.fmt(solve.get('tick_max_ms'), '.3f')} ms ⟹ 请求 {obs.get('rate_hz')} Hz、"
       f"实发 {v.fmt((obs.get('dut') or {}).get('actual_hz'), '.1f')} Hz")

    # ⑧ 复盘面
    # ☠☠ 分龄这一列**不能照抄 A 档的 `ages_ok`**：那是按 A 档"两路 EKF 状态"写的
    #    （`age_pos_s` + `age_att_s`，位置与姿态各来自一条话题）。B 档只有**一个**状态源
    #    （`/mocap/pose_enu` 一条 `PoseStamped`，位置与姿态同帧）⟹ 分龄只有**一条**
    #    （`age_state_s`）。2026-09-29 第一版直接复用 A 档那个 `ages_ok`，于是五场全红，
    #    理由是"`age_pos_s`/`age_att_s` 不是数" —— 而这两列在 B 档**本就不存在**。
    #    症状是"仪器自检报缺口"，真身是**判据抄错了档**。
    ages_b = all(isinstance(f.get("age_state_s"), (int, float)) for f in frames)
    n_no_age = sum(1 for f in frames if not isinstance(f.get("age_state_s"), (int, float)))
    ck("⑧ 复盘面：逐帧日志里有状态源、分龄、饱和标志与档位",
       (obs.get("dut") or {}).get("state_source") == "mocap"
       and ages_b and bool((obs.get("dut") or {}).get("sat_logged")),
       f"state_source={(obs.get('dut') or {}).get('state_source')}、"
       f"分龄齐全={ages_b}（缺 {n_no_age} 帧；B 档的分龄只有 `age_state_s` **一条** —— "
       f"位置与姿态同一条 `PoseStamped` ⟹ A 档那两条 `age_pos_s`/`age_att_s` 在 B 档"
       f"**本就不存在**，不是缺口）、"
       f"每帧带 `sat` 列={(obs.get('dut') or {}).get('sat_logged')}、"
       f"档位分布 {fb.get('dist')}")

    # ⑨ 判据②：两条轨迹（悬停 / r=0.5 m 八字）—— 定性判，误差只留档
    obs["traj"] = _judge_traj(ck, obs, seg)
    return c


def _judge_m5(ck, obs, fb, ph, truth) -> None:
    """判据④ 的两层读数：**兜底机制**（阶梯有没有按 M5 走）与**兜底后果**（高度保没保住）。

    ★★ 为什么必须分两层、而且要**两个被控对象各跑一场**：

      M5 兜底发的悬停帧推力**恒等于模型的 `hover_thrust`**（`mpc_node.py:405`），
      而被控对象需要的悬停推力是 `m_plant·g / T_max_plant`。票面要求被控对象故意
      差 ±30 %，于是**这两个数本来就不相等**：

          模型 0.6235  vs  被控对象 mass+30 需要 0.8106  ⟹  净下坠 (1 − 0.6235/0.8106)·g ≈ 2.3 m/s²

      ⟹ 在失配被控对象上，M5 **必然**保不住高度 —— 那是**兜底口径的前提被打破**
      （"模型 hover_thrust 正确"这一条本来就是 M5 的假设），**不是阶梯坏了**。
      要证阶梯本身是好的，只能用**对得上的被控对象**（`--mismatch none`）再跑一场。
      把这两件事混成一条 ❌ 就会写出一个**归因错误**的结论。
    """
    ladder_ok = fb["n_hover"] > 0 and fb["n_hold"] > 0 and "normal" in fb["order"]
    t_hover_start = fb.get("hover_start_wall_s")
    post = [r for r in truth if t_hover_start is not None and r[T_WALL] >= t_hover_start]
    m5 = {"n_post": len(post)}
    if len(post) >= 5:
        zs, _tail, vz = tail_vz(post)
        m5.update({"z_start": zs[0], "z_end": zs[-1], "z_min": min(zs), "z_max": max(zs),
                   "descent_mean": (zs[0] - zs[-1])
                   / max(post[-1][T_WALL] - post[0][T_WALL], 1e-9),
                   "vz_tail": statistics.fmean(vz) if vz else None})
    obs["m5"] = m5

    want = ph.get("hover_thrust")
    note = ""
    if want is not None and obs.get("model_hover_thrust") is not None:
        # 归一化推力之比 = 实际推力之比（两者共用同一个 T_max 分母口径）：
        #   推力比 = 模型给的 / 被控对象需要的，不足 1 ⟹ 净向下加速度 = (1 − 比)·g
        model_ht = float(obs["model_hover_thrust"])
        pred = (1.0 - model_ht / float(want)) * 9.81   # **正 = 推力不足 ⟹ 下坠**
        m5["predicted_descent_ms2"] = pred
        note = (f"★ **口径对不上是预期的**：模型 hover_thrust "
                f"{v.fmt(model_ht, '.4f')} vs 被控对象需要 "
                f"{v.fmt(want, '.4f')}（= m·g/(4·SIH_T_MAX)，从 {obs.get('plant_param')} 读）"
                f"⟹ 固定悬停帧给出的推力只有所需的 {v.fmt(model_ht / float(want), '.3f')} 倍 "
                f"⟹ 预测净加速度 {v.fmt(pred, '+.3f')} m/s²"
                f"（正 = 下坠、负 = 上飘）。")

    ck("★判据④-A M5 阶梯真的按设计走了（`hold` → `hover` 的迁移在日志里可检索）",
       ladder_ok,
       f"档位分布 {fb['dist']}；迁移顺序 {'→'.join(fb['order'])}；"
       f"`hold` {fb['n_hold']} 拍（{v.fmt(fb.get('hold_span_s'), '.3f')}s，"
       f"`fallback_hold_frames` 默认 10 拍）、`hover` {fb['n_hover']} 拍"
       f"（{v.fmt(fb.get('hover_span_s'), '.2f')}s）；悬停帧的 `thrust_body_z` 实测 "
       f"{fb['hover_thrust_body_z_seen']} ⟹ 恒等于 −hover_thrust ⟹ **结构上不可伪造**"
       f"（`mpc_node.py:405`）。理由一栏：{(fb['notes'] or ['（无）'])[0][:180]}")

    if not ladder_ok:
        ck("★判据④-B 兜底后飞机保持安全悬停（高度保持住）", False,
           "阶梯没走起来（见上一条）⟹ 这一条不可判")
        return

    tgt_z = ((obs.get("steady") or {}).get("target_enu") or [0, 0, 0])[2]
    err_end = None if m5.get("z_end") is None else m5["z_end"] - tgt_z
    # ⚠️ 这一列**不能写 `vz_tail or 9.9`**：悬停保持得**完美**时它正好是 `0.0`，
    #    而 `0.0` 是假值 ⟹ `or` 会把它换成 9.9，把"稳如磐石"判成"发散"。
    #    （`test_判据4B_对得上的被控对象_兜底保住了高度就通过` 就是这条抓出来的。）
    vz_tail = m5.get("vz_tail")
    ok_hover = (err_end is not None and abs(err_end) < HOVER_TOL_M
                and vz_tail is not None and abs(vz_tail) < DIVERGE_VZ_MS
                and m5.get("z_min", -1) > GROUND_M)
    ck("★判据④-B 兜底后飞机保持安全悬停（高度保持住、不坠机、不发散）",
       ok_hover,
       f"切入 `hover` 之后 {m5.get('n_post')} 点：高度 "
       f"{v.fmt(m5.get('z_start'), '+.3f')} ⟹ **{v.fmt(m5.get('z_end'), '+.3f')} m**"
       f"（目标 {v.fmt(tgt_z, '+.3f')} ⟹ 末段误差 {v.fmt(err_end, '+.3f')} m，"
       f"界 ±{HOVER_TOL_M}）；全程区间 [{v.fmt(m5.get('z_min'), '+.3f')}, "
       f"{v.fmt(m5.get('z_max'), '+.3f')}] m；平均下降率 "
       f"{v.fmt(m5.get('descent_mean'), '+.3f')} m/s、末段垂直速度 "
       f"{v.fmt(m5.get('vz_tail'), '+.3f')} m/s。{note}"
       f"★★ 这一条不过是**有信息**的：它说明「兜底 = 发一条固定推力的安全帧」这个设计"
       f"**以『模型的 `hover_thrust` 正确』为前提** —— 而票面要求的 ±30 % 参数失配恰好打破它。"
       f"要判「阶梯本身好不好」看 `--mismatch none` 那一场")
    matched = str(obs.get("mismatch") or "none") == "none"
    ck("★判据④-C 归属：这一场的读数该记在谁头上",
       None,
       ("被控对象与模型**对得上**（`--mismatch none`）⟹ 这一场是 M5 的**对照**："
        "它若不达标，归因就是兜底机制本身" if matched else
        f"被控对象是**失配**变体 `{obs.get('mismatch')}` ⟹ 这一场同时含两个因素："
        f"（a）M5 阶梯本身（看 ④-A）、（b）兜底悬停推力的口径在被控对象上偏了多少"
        f"（见上面的预测值）。两者要靠 `--mismatch none` 那一场分开"),
       counts=False)


# ------------------------------------------------------------------ 报告


def report_b(obs, checks) -> int:
    print("=" * 100)
    print(f"B 档 seam 3 —— {obs.get('name')}（{obs.get('plant_param')}，"
          f"延迟 {v.fmt((obs.get('replay') or {}).get('delay_ms', obs.get('extra_delay_ms')), '.2f')} ms，"
          f"注入 {obs.get('inject_solve_fail_after_frames')} 帧后）")
    print("=" * 100)
    bad = 0
    for c in checks:
        mark = "✅" if c["ok"] else ("ℹ️ " if c["ok"] is None else "❌")
        if c["ok"] is False and c.get("counts", True):
            bad += 1
        print(f"{mark} {c['name']}")
        for line in str(c["detail"]).splitlines():
            print(f"     {line}")
        print()
    n_count = sum(1 for c in checks if c.get("counts", True))
    print(f"———— 计入的判据 {n_count} 条，其中不通过 {bad} 条 ————")
    return 1 if bad else 0


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0],
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--tag", default="B", help="这一场的名字（进文件名与报告标题）")
    ap.add_argument("--model", default=str(HERE / "evidence" / "tier-b" / "mocap-model-3.json"),
                    help="`mocap_model.py dump` 出来的模型 JSON（回放夹具的唯一输入）。"
                         "默认 = 本仓库归档里那份（☠️ `/tmp` 是 tmpfs，2026-09-29 10:40 重启后 "
                         "`/tmp/mocap-model-3.json` 已不存在，见模块 docstring）")
    ap.add_argument("--mismatch", default="none", choices=sorted(MISMATCHES),
                    help="被控对象的失配变体（`mass+30` = 票面要的那一档；`none` = 归因对照）")
    ap.add_argument("--hover-thrust", type=float, default=0.6235,
                    help="DUT 模型的悬停油门。★ 默认 0.6235 = **SITL 机型自己推出来的值**"
                         "（m·g/(4·SIH_T_MAX)），不是 `params.py` 的 0.1589 —— 理由见模块 docstring ☠☠")
    ap.add_argument("--extra-delay-ms", type=float, default=0.0,
                    help="在模型基延迟上再叠多少（判据③ 用 50）")
    ap.add_argument("--base-delay-ms", type=float, default=None,
                    help="覆盖模型里的基延迟（默认取模型里 stamp→记录器 的中位值）")
    ap.add_argument("--seed", type=int, default=20260929)
    ap.add_argument("--inject-solve-fail-after-frames", type=int, default=0,
                    help="故障注入（判据④）：已发布 N 帧之后每拍的解都换成坏解；0 = 关")
    ap.add_argument("--hold-s", type=float, default=25.0, help="OFFBOARD 生效后再飞多少秒")
    ap.add_argument("--rate-hz", type=float, default=100.0)
    ap.add_argument("--hold-x", type=float, default=0.0)
    ap.add_argument("--hold-y", type=float, default=0.0)
    ap.add_argument("--hold-z", type=float, default=1.0,
                    help="**目标高度**（ENU，米）—— 两种参考源共用这一个旋钮：`hold` 档它是 DUT 的 "
                         "`hold_offset_z`；`traj` 档它变成夹具的 `--ref-z-offset-m`，"
                         "而 DUT 的 `hold_offset` 全 0（那一档拒收非零）")
    ap.add_argument("--reference-source", default="hold", choices=("hold", "traj"),
                    help="DUT 的参考源（ticket 17）：`hold` = 锚点+常量偏移（老行为）；"
                         "`traj` = 夹具重发包里的 `/traj/reference`（判据② 的八字那条）")
    ap.add_argument("--ref-hold-s", type=float, default=30.0,
                    help="`traj` 档：夹具开场顶住首点多少秒（等 DUT 起来 + 解锁 + OFFBOARD + 爬到参考高度）")
    ap.add_argument("--ref-z-offset-m", type=float, default=None,
                    help="`traj` 档：叠在参考 position.z 上的高度；默认取 `--hold-z`")
    ap.add_argument("--dut-timeout-s", type=float, default=150.0)
    ap.add_argument("--offboard-timeout-s", type=float, default=40.0)
    ap.add_argument("--outdir", default="/tmp/tier-b")
    ap.add_argument("--json", default=None)
    ap.add_argument("--replay", default=None, help="只重判一份旧 obs.json（不重跑 SITL）")
    ap.add_argument("--src", default=str(Path.home() / "refs/px4-v1.14.3-build"))
    ap.add_argument("--build-dir", default=None)
    ap.add_argument("--ready-timeout", type=float, default=90.0)
    ap.add_argument("--boot-settle", type=float, default=4.0)
    ap.add_argument("--keep-params", action="store_true")
    ap.add_argument("--agent-bin", default="/usr/local/bin/MicroXRCEAgent")
    ap.add_argument("--agent-port", default="8888")
    ap.add_argument("--debug-dump", default="/tmp/verify_tier_b_console.log")
    args = ap.parse_args(argv)

    if args.replay:
        obs = json.loads(Path(args.replay).read_text(encoding="utf-8"))
        if "checks" not in obs:
            obs["checks"] = evaluate_b(obs)
        return report_b(obs, obs["checks"])

    if not Path(args.model).exists():
        raise SystemExit(f"模型 {args.model} 不存在 —— 先跑：\n"
                         f"  python3 tools/px4-sitl/mocap_model.py dump bag/3_traj1_hover "
                         f"--noise-bag bag/1_static --json {args.model}")
    if args.reference_source == "traj":
        # ☠️ 两条静默错法各堵一道。写反了不会报错，只会让报告说"飞机没动"：
        #    ① DUT 侧收到非零 `hold_offset` ⟹ `mpc_node._validate` **拒跑**（好，但很难懂）；
        #    ② 两边都不出高度 ⟹ 锚点在地面 ⟹ 飞机**压根不打算离地**，判层只看到"没动"。
        if args.hold_x or args.hold_y:
            raise SystemExit(f"--reference-source traj 时 hold_x/hold_y 必须是 0（收到 "
                             f"{args.hold_x}/{args.hold_y}）—— 那一档的 `hold_offset` 全 0，"
                             f"它只是锚点相对量，不是目标点")
        if ref_z_offset_m(args) == 0.0:
            raise SystemExit("--reference-source traj 且高度 0 ⟹ 参考全在**地面**上"
                             "（包里的参考 z ≡ 0，锚点又定在地面那一拍）—— "
                             "用 --hold-z（或 --ref-z-offset-m）给一个起飞高度")
        if args.hold_s < args.ref_hold_s + 10.0:
            raise SystemExit(f"--hold-s={args.hold_s} 太短：traj 档要先用 --ref-hold-s="
                             f"{args.ref_hold_s} 顶住首点等飞机起来，八字本身还要 ~6.1 s "
                             f"（包 4 的移动段），合计至少 {args.ref_hold_s + 10.0:.0f} s")
    args.plant_param = plant_param_file(args.mismatch)
    args.model_hover_thrust = args.hover_thrust
    args.build_dir = args.build_dir or str(Path(args.src) / "build/px4_sitl_default")
    args.replay_summary = Path(args.outdir) / f"{args.tag}-replay.json"
    outdir = Path(args.outdir)
    outdir.mkdir(parents=True, exist_ok=True)
    if not (Path(args.build_dir) / "etc/init.d-posix/airframes/10046_sihsim_z0").exists():
        raise SystemExit("z0 机型没装 —— 先跑 install_z0_airframe.sh")
    if not args.keep_params:
        for f in ("parameters.bson", "parameters_backup.bson"):
            (Path(args.build_dir) / "rootfs" / f).unlink(missing_ok=True)

    # 夹具与 DUT 都要能把 `nmpc` / `px4_msgs` import 进来：本进程已经做到了（它们就在
    # 上面 import 成功），但子进程继承的是 `os.environ` —— 操作者 source 过就够。
    obs = {"name": args.tag, "plant_param": args.plant_param}
    rc = 0
    world = replay = tap = None
    rec = agent = None
    rclpy.init()
    try:
        rec = v.Recorder()
        with BWorld(args) as world:
            print(f"[verify] 被控对象 = {args.plant_param}"
                  f"（COM_OF_LOSS_T = {world.com_of_loss_t}）", file=sys.stderr)
            print(f"[verify] 模型 hover_thrust = {args.hover_thrust}；"
                  f"被控对象需要 {plant_hover_thrust(args.plant_param)}", file=sys.stderr)
            tap = TruthTap(outdir / f"{args.tag}-truth.jsonl")
            replay = ReplayProc(args, outdir / f"{args.tag}-replay-frames.jsonl",
                                args.replay_summary)
            cfg = argparse.Namespace(**vars(args))
            cfg.log_file = str(outdir / f"{args.tag}-dut.jsonl")
            argv_dut = ["--ros-args"]
            for p in dut_params_b(cfg):
                argv_dut += ["-p", p]
            agent = ta.ArmAgent()
            with ta.MpcDut(argv_dut, Path(cfg.log_file)) as dut:
                obs.update(run_flight_b(world.px, rec, tap, dut, replay, cfg, agent))
            obs["model_hover_thrust"] = args.hover_thrust
            obs["plant_param"] = args.plant_param
            # 判据层自己炸了也要把这一场说完（观测已在手上，`finally` 落盘 + `--replay`）
            try:
                obs["checks"] = evaluate_b(obs)
            except Exception as e:  # noqa: BLE001
                traceback.print_exc()
                rc = 3
                obs["checks"] = [{
                    "name": "☠ 判据层自己抛异常（本场观测仍然完整，可 `--replay` 重判）",
                    "ok": False, "counts": True, "detail": f"{type(e).__name__}: {e}"}]
            rc = report_b(obs, obs["checks"]) or rc
            agent.destroy_node()
    except (v.Px4ConsoleError, OSError) as e:
        print(f"[verify] ☠ 世界没起来：{e!r}", file=sys.stderr)
        rc = 2
    finally:
        if replay is not None:
            replay.kill()
        if tap is not None:
            tap.close()
            tap.destroy_node()
        if world is not None and world.px is not None:
            Path(args.debug_dump).write_text(world.px.text(), encoding="utf-8")
            print(f"[verify] 控制台全文写到 {args.debug_dump}", file=sys.stderr)
        if args.json:
            # ★ `_tap` 是**活节点**（⓪b 在判据层里拿它配对），不是观测值 —— 不进 JSON
            obs.pop("_tap", None)
            Path(args.json).parent.mkdir(parents=True, exist_ok=True)
            Path(args.json).write_text(
                json.dumps(obs, indent=2, ensure_ascii=False, default=float), encoding="utf-8")
            print(f"[verify] 观测写到 {args.json}", file=sys.stderr)
        if rec is not None:
            rec.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return rc


if __name__ == "__main__":
    sys.exit(main())
