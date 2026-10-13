#!/usr/bin/env python3
"""装桨之前的**静态自检** —— 让参数与符号错误在这一步暴露（ticket 06 的验收项）。

    python3 src/nmpc/nmpc/preflight_check.py            # 快档：参数 + 符号 + 归一化
    python3 src/nmpc/nmpc/preflight_check.py --solve    # 再加"建一次求解器，在悬停点解一次"
    ros2 run nmpc preflight_check.py -p mass:=0.82 -p hover_thrust:=0.55

★ 与 `colcon test` 的分工：测试回答"实现有没有被改坏"，本脚本回答
  **"我现在这一套参数能不能装上桨"** —— 所以它①按**运行参数**跑（`-p` 与节点同一套写法），
  ②把结论打成一张清单，③任何一条不过就退出码 1（能直接串在起飞前的流程里）。

★ 四类检查，各自回答一个问题：

| 组 | 问题 | 不需要 |
|---|---|---|
| 参数自洽 | 悬停有没有上余量？τ / 频率 / 停机时限是不是票面那几个量级？ | 飞机、acados |
| 符号 | **三方对账**：实现 / `docs/control/sign-map.md` 的 canonical 块 / `tools/sign-map` 的独立推导 | 飞机、ROS |
| 归一化 | 推力归一化是不是**从模型导出**的（`−F/T_max`、悬停值 = `−hover_thrust`） | 飞机、ROS |
| 停机条件 | 四个条件**参数化**、且程序里**没有通道号**（`ast` 扫标识符，注释里的引用不算） | 飞机、ROS |
| 兜底 | 兜底保持拍数可解释、位姿合理性阈值比真机动**宽**、**故障注入关着** | 飞机、ROS |
| 下行前置 | 换 mavros 载体之后**两个"整条下行静默死掉"的开关**：飞控的 `MAV_FWDEXTSP = 1`、mavros 的 `thrust_scaling = 1.0` | 飞机；前者读仓库里的导出快照，后者要 **mavros 起着**（★ **不需要飞控通电**，见 `read_live_thrust_scaling`） |

`--solve` 那一档才需要 acados：它把"模型 + 代价 + 符号 + 归一化"合成**一个数** ——
在悬停点解出来的 `u` 归一化之后必须等于 `−hover_thrust`。
"""

from __future__ import annotations

import argparse
import ast
import math
import sys
from dataclasses import dataclass, fields
from pathlib import Path
from typing import Callable, Dict, FrozenSet, List, Optional, Sequence, Tuple

#: 直接 `python3 src/nmpc/nmpc/preflight_check.py` 跑时（不经 colcon）的引导，
#: 与 `offboard_smoke.py` 同一形态：把 `src/nmpc` 塞进 sys.path 好让 `nmpc` 包能 import。
if __package__ in (None, ""):  # pragma: no cover - 只在直接执行时走
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from nmpc.node_core import (STATUS_SOURCE_PERIOD_S, TAU_MAX_S, TAU_MIN_S,  # noqa: E402
                            StopConfig)
from nmpc.params import G, Z0Params  # noqa: E402
from nmpc.sign_map import px4_rates_setpoint  # noqa: E402

#: **必须存在**的四个停机条件（= `docs/control/phase2-implementation.md` §4 那两条既有路径的节点侧
#: 表现：打杆接管 / ch6 切模式在节点这一侧都表现为"离开 OFFBOARD"）。
#: ★ 判据是"这四个**都在**"，不是"字段集恰好等于这四个"：`StopConfig` 的语义是
#:   **"不发 setpoint 的条件"**，`warmup_cycles`（首拍不发）是名正言顺的第五个闸门 ——
#:   要求字段集相等会把它误伤成失败（第一手：本检查的第一版就是那样红的）。
#:   多出来的字段会被列进 `detail`，请当场确认它确实属于闸门。
STOP_FIELDS: FrozenSet[str] = frozenset(
    {"mocap_timeout_s", "status_timeout_s", "stop_on_nav_state_exit", "stop_on_armed_drop"})

#: 通道号的形状（`ch6` / `ch8` / `rc_channel` 之类）。★ 用 `ast` 扫**标识符与字符串**，
#: 不扫注释与 docstring —— 那些地方引用 `phase1/issues/02` 的结论（"急停 = ch8 高值"）
#: 是正确的行文，把它算成违规就没人敢写注释了。
_CHANNEL_RE = r"(?i)^(ch\d+|rc_?chan\w*|channel\w*)$"

#: 符号对账用的样例：`(ω_flu, F, T_max)`。刻意覆盖 0 / 悬停 / 混合 / 打满 / 换电池。
SIGN_CASES: Sequence[tuple] = (
    ((0.0, 0.0, 0.0), 0.0, 11.8),
    ((0.0, 0.0, 0.0), 0.75 * G, 11.8),
    ((0.7, -0.4, 0.25), 9.1, 11.8),
    ((-1.2, 0.0, -0.9), 11.8, 11.8),
    ((0.0, 0.5, 0.0), 0.001, 3.0),
)


@dataclass(frozen=True)
class Check:
    """一条检查的结论。`detail` 里放**数**（不是"通过"两个字），失败时要能一眼看出差多少。"""

    group: str
    name: str
    ok: bool
    detail: str


def find_repo_root(start: Optional[Path] = None) -> Optional[Path]:
    """往上找带 `docs/control/sign-map.md` 的那一级 —— 那是**数值权威**所在。

    ★ 找不到就**算失败**，不算"跳过"：符号对账是这套检查的核心，静默跳过等于没查。
      （`install/` 下的副本离仓库根很远，那时要显式 `--repo`。）
    """
    base = Path(start) if start is not None else Path(__file__).resolve()
    for cand in (Path.cwd(), base, base.parent):
        for p in (cand, *cand.parents):
            if (p / "docs" / "control" / "sign-map.md").is_file():
                return p
    return None


# ───────────────────────────── ① 参数自洽 ─────────────────────────────

def check_params(p, z0: Optional[Z0Params] = None) -> List[Check]:
    """参数自洽 —— 全是**量纲关系**，不抄 `params.py` 里的具体数值（那些是占位/实测，会回填）。"""
    z0 = z0 or p.z0_params()
    g = "参数自洽"
    out: List[Check] = []

    out.append(Check(g, "质量为正", p.mass > 0.0, f"mass = {p.mass} kg"))

    ok = 0.0 < p.hover_thrust < 1.0
    out.append(Check(g, "悬停有上余量", ok,
                     f"hover_thrust = {p.hover_thrust} ⟹ T_max = {z0.t_max:.4f} N、"
                     f"悬停 F = {z0.t_hover:.4f} N"
                     + ("" if ok else "（≥1 表示悬停就要打满：没有爬升余量）")))

    # ★ 判据对象是 **`f_max`（`F` 上限，按电池）**，不是 `t_max` —— 后者是**归一化**常数，
    #   按定义必须 = 实测满量程（46.3 N），拿它比会一直红，而那红**指错了东西**。
    #   2026-09-29 解耦前这条守卫报的 `T_max = 46.3 N` 正是"两个角色被混成一个数"的症状。
    ok = z0.f_max < 40.0
    out.append(Check(g, "F 上限按**电池**取值", ok,
                     f"f_max = {z0.f_max:.3f} N（电机上限 40 N 是 M4 说的那个**不该**用的数；"
                     f"走到那儿说明又按了电机）。★ 归一化常数是另一回事："
                     f"t_max = {z0.t_max:.3f} N（须 = 实测满量程）"))

    # ★ 这条把"两个角色"钉在一起：`F` 上限够悬停（否则连平飞都规划不出来），
    #   且**严格小于**归一化满量程。
    # ⚠️ 右端**必须是严格的不等号**（2026-09-29 复核订正）：写成 `<=` 时，把两个角色**合回一个
    #   常数**（`f_max = t_max`）照样绿 —— 而那正是本张要防的那个错误。物理上也不可能相等：
    #   `t_max` 是电机/桨的满量程，`f_max` 是电池可持续的那一档，电池永远先见底。
    ok = z0.t_hover <= z0.f_max < z0.t_max
    out.append(Check(g, "F 上限夹在悬停与满量程之间（且严格小于满量程）", ok,
                     f"t_hover = {z0.t_hover:.4f} N ≤ f_max = {z0.f_max:.3f} N < "
                     f"t_max = {z0.t_max:.3f} N（f_max 低于悬停 ⟹ 规划不出平飞；"
                     f"不小于 t_max ⟹ 两个角色又被合回一个常数了 —— 那是本张修掉的那个错）"))

    # ☠️ 这里原有两条与**独立心跳流**有关的检查：
    #   · 「心跳 ≥2 Hz」（`heartbeat_hz ≥ 2.0`，理由是 PX4 的 `COM_OF_LOSS_T` = 1 s）；
    #   · 「定时器频率」里同时报 `rate_hz` / `heartbeat_hz` / `viz_hz`。
    #   ticket 13 §二④（操作者裁定 (a) 收成单频）把心跳那条流与 `heartbeat_hz` / `heartbeat_always`
    #   两个参数一起删了 ⟹ **前一条整个删除**（它守的量不存在了）。
    #   ⚠️ 它原来守的那条性质**没有消失、只是换了主语**：现在"心跳"就是"这一条 setpoint 流"，
    #   而它的频率就是 `rate_hz`（100 Hz ≫ 1 s 的 `COM_OF_LOSS_T`）⟹ 见下面那条。
    ok = p.rate_hz > 0.0 and p.viz_hz > 0.0
    out.append(Check(g, "定时器频率", ok,
                     f"rate = {p.rate_hz} Hz / viz = {p.viz_hz} Hz"
                     "（★ 只有这两条定时器：心跳那条流随换载体删除，"
                     "setpoint 自己就是心跳 ⟹ 它的频率就是 `rate_hz`，"
                     "远快于 PX4 的 `COM_OF_LOSS_T` = 1 s）"))

    # ★★ 下界是 **0**（含），不是"正的"（ticket `phase4/05`，2026-10-09）：
    #   τ = 0 = **不补偿**，是**首飞冻结值**（ticket `phase4/03`：过补偿比不补偿更危险）。
    #   ⚠️ 两个方向都留着负控：负数 = **反向外推**（拿"还没走到"的状态喂控制器），
    #   `≥ TAU_MAX_S` = 比一拍还大，那不再是执行延迟的量级。
    #   端点用的是 `node_core` 里的常量 ⟹ 与 `mpc_node._validate` **同一份**，
    #   不会再出现"两处各写一遍 `0.2`、差一个端点"那种漂移（本票第一版踩过）。
    ok = TAU_MIN_S <= p.tau_s < TAU_MAX_S
    out.append(Check(g, "外推 τ 在合理区间", ok,
                     f"tau_s = {p.tau_s} s（首飞冻结值 0 = 不补偿；z0 实测 ≈0.05 s；"
                     f"负数反向外推，τ 大过一拍就会明显过冲）"))

    ok = p.warmup_cycles >= 1
    out.append(Check(g, "热身拍数 ≥1", ok,
                     f"warmup_cycles = {p.warmup_cycles}（SQP_RTI 的第 0 次 solve 给的是出厂迭代点："
                     f"实测第 0 次 ω_y = 0.0000、第 1 次 3.8400 ⟹ 0 会把那一帧发出去）"))

    ok = p.mocap_timeout_s > 0.0 and p.status_timeout_s > 0.0
    out.append(Check(g, "停机时限为正", ok,
                     f"mocap > {p.mocap_timeout_s} s / status > {p.status_timeout_s} s"))

    # ☠️ **这条的判据整个换过**（ticket 13 §二③）：旧判据是 `status_timeout_s >= 0.5`，
    #   理由是"假 PX4 与真 commander 都是 **2 Hz**"。换载体后真的闸门源是 `/mavros/state`，
    #   它跟着 PX4 的 MAVLink HEARTBEAT **1 Hz** 走 ⟹ **旧的 0.5 s 判据在真机上恒绿而恒错**：
    #   `0.75`（旧默认值）也满足 `>= 0.5`，而 `0.75 < 1.0` 会让"状态超时"这条判据**恒真**
    #   （闸门一直关、一帧都不发）。★ 判据现在是"**阈值 vs 源周期**"这条物理关系本身，
    #   而不是一个拍出来的下界 —— 周期唯一写一遍在 `node_core.STATUS_SOURCE_PERIOD_S`。
    period = STATUS_SOURCE_PERIOD_S
    # ★ 门槛取 **2 个周期**（不是"大于 1 个周期"）：只比一个周期多一点的话，
    #   源正常地丢一拍就会被判成"状态过期"（抖动误判），而这类误判的症状同样是"不发"。
    ok = p.status_timeout_s >= 2 * period
    if ok:
        why = ""
    elif p.status_timeout_s <= period:
        why = ("　⟹ 阈值不大于源周期 ⟹ 超时判据**恒真**、闸门一直关着"
               "（症状是「一帧都不发」，不是报错）")
    else:
        why = "　⟹ 余量不到 2 个周期 ⟹ 源正常丢一拍就会被判成丢状态（抖动误判）"
    out.append(Check(g, "状态时限 ≥ 闸门源 2 个周期", ok,
                     f"status_timeout_s = {p.status_timeout_s} s vs 闸门源 `/mavros/state` "
                     f"周期 {period} s（= {p.status_timeout_s / period:.2f} 个周期）" + why))

    ok = bool(p.stop_on_armed_drop) and bool(p.stop_on_nav_state_exit)
    out.append(Check(g, "两条安全停机都开着", ok,
                     f"掉 armed = {p.stop_on_armed_drop} / 离开 offboard = {p.stop_on_nav_state_exit}"
                     + ("" if ok else "（关掉它 = 飞控已经不认这条指令了，我们还在发）")))
    return out


# ───────────────────────────── ② 符号三方对账 ─────────────────────────────

def check_signs(repo: Path, impl: Callable = px4_rates_setpoint) -> List[Check]:
    """符号：**实现 / 文档 / 独立推导**三边互相印证（任何一边被动过都会红）。

    与 `test/test_sign_map.py` 是同一个口径，区别是这里的"实现"可以换（`impl=`），
    好让**负控**真的能验证"符号写反会被抓到"。
    """
    g = "符号对账"
    sys.path.insert(0, str(repo / "tools" / "sign-map"))
    import sign_map_core  # noqa: PLC0415  （`tools/sign-map` 下的复核库，不是本包的实现）

    doc = sign_map_core.parse_canonical_block(repo / "docs" / "control" / "sign-map.md")
    derived = sign_map_core.derived_signs()
    out = [Check(g, "文档 == 独立推导", doc == derived,
                 f"docs/control/sign-map.md 的 canonical 块 vs tools/sign-map：{doc} / {derived}")]

    for label, ref in (("实现 == 文档", doc), ("实现 == 独立推导", derived)):
        bad = []
        for omega, thrust, t_max in SIGN_CASES:
            sp = impl(omega, thrust, t_max)
            got = (sp.roll, sp.pitch, sp.yaw, sp.thrust_body_z)
            want = (ref["roll_rate"] * omega[0], ref["pitch_rate"] * omega[1],
                    ref["yaw_rate"] * omega[2], ref["thrust_body_2"] * thrust / t_max)
            if not all(math.isclose(a, b, rel_tol=1e-12, abs_tol=1e-12) for a, b in zip(got, want)):
                bad.append(f"ω={omega} F={thrust} T_max={t_max}：得到 {got}，应为 {want}")
        out.append(Check(g, label, not bad,
                         f"{len(SIGN_CASES)} 组样例全部一致" if not bad else "；".join(bad[:3])))
    return out


# ───────────────────────────── ③ 归一化从模型导出 ─────────────────────────────

def check_normalization(z0: Z0Params, impl: Callable = px4_rates_setpoint) -> List[Check]:
    """推力归一化：**悬停那一点**是"模型里的 hover_thrust"与"归一化常数 1/T_max"的交汇处。

    第一手依据（`MulticopterRateControl.cpp:182`）：PX4 对 `thrust_body` **原样透传、不做任何
    悬停油门重标定** ⟹ 模型里的 `hover_thrust` 与我们发出去的归一化推力**必须是同一个物理量**
    （`phase2-implementation.md` §6.3）。这一组就是把那条关系变成等式。

    ★ `impl=` 是给**负控**用的：把它换成"硬编码归一化常数"的那一版（参考实现里的 `0.07`），
      "换 T_max 归一化跟着走"那一条就会红 —— 这正是本组存在的理由。
    """
    g = "推力归一化"
    out: List[Check] = []

    hover = impl((0.0, 0.0, 0.0), z0.t_hover, z0.t_max)
    want = -z0.hover_thrust
    out.append(Check(g, "悬停归一化 = −hover_thrust", math.isclose(hover.thrust_body_z, want, rel_tol=1e-12),
                     f"−F/T_max = {hover.thrust_body_z:+.6f}，−hover_thrust = {want:+.6f}"))

    ok = -1.0 < hover.thrust_body_z < 0.0
    out.append(Check(g, "悬停落在值域 (−1, 0)", ok, f"thrust_body[2] = {hover.thrust_body_z:+.6f}"))

    full = impl((0.0, 0.0, 0.0), z0.t_max, z0.t_max)
    out.append(Check(g, "打满 = −1（不静默截断）", math.isclose(full.thrust_body_z, -1.0, rel_tol=1e-12),
                     f"F = T_max ⟹ thrust_body[2] = {full.thrust_body_z:+.6f}"))

    # 归一化常数只能有**一处**：`sign_map.px4_rates_setpoint(..., t_max_n)` 的形参。
    # 参考实现把 1/T_max 硬编码成 0.07（改质量时静默错 33 %）—— 这一条盯的就是那个坑。
    hard = impl((0.0, 0.0, 0.0), z0.t_hover, 2.0 * z0.t_max)
    out.append(Check(g, "换 T_max 归一化跟着走", math.isclose(hard.thrust_body_z, want / 2.0, rel_tol=1e-12),
                     f"同一个物理推力、T_max 翻倍 ⟹ {hard.thrust_body_z:+.6f}（= 悬停值的一半）"))
    return out


# ───────────────────────── ③′ 兜底与输入合理性（ticket 07 / M5）─────────────────────────

#: 自检里当**下限**用的"飞控最大水平速度" [m/s] —— 它伺候的是那条"位姿合理性阈值比真机动宽"：
#: 我们自己那个跳变阈值（`mocap_jump_speed_max`，默认 `20`）必须**大于**飞控能飞出的速度，
#: 否则一次正常机动会被读成"跳变"而**无故停发**（飞机在空中收不到指令）。
#: ★ 它**不在代码里**（是飞控参数，不是我们的模型参数）⟹ 本文件是它唯一的落点，
#:   与上面那个 40 N 的电机上限同一种用法。★ 运行时真正用的是 `mocap_jump_speed_max`（`20`），
#:   **不是**这个常量 —— 这个常量只在自检里当对照。
#: ⚠️ **2026-10-10 更正注释**：原写"z0 飞控**现状**里的最大水平速度"—— **不实**。
#:   飞控上现读 **`2.0`**（2026-09-23 写入；现值按名字查 `docs/assets/z0.params`，
#:   事实链见 `docs/reference/params-changes.md` §I 与 `first-flight-config.md` §F7）。
#:   `12.0` 是**刻意保留的宽松值**（比真值宽 6×）：换成 `2.0` 不会改变当前结论
#:   （`20 > 2` 照样过），只会让这条"下限"**放宽 6×** —— 即允许有人把运行时那个阈值
#:   配到刚过 `2.0`，而那在真实机动上会**无故停发**。
#:   ⟹ 别把这个数说成现状；**也别顺手改成 `2.0`** —— 改它等于推翻
#:   `.scratch/phase2/issues/07` 处置 ③ 那个已复核的决定。
MPC_XY_VEL_MAX = 12.0


def check_fallback(p, z0: Optional[Z0Params] = None) -> List[Check]:
    """兜底阶梯与输入合理性判据 —— 全是**参数之间的大小关系**，不重复代码里的逻辑。"""
    z0 = z0 or p.z0_params()
    g = "兜底"
    out: List[Check] = []

    ok = int(p.fallback_hold_frames) >= 0
    out.append(Check(g, "兜底保持拍数 ≥0", ok,
                     f"fallback_hold_frames = {p.fallback_hold_frames} 拍"
                     f"（0 = 第一次失败就切安全悬停；默认 10 拍 = 100 ms ≈ 28 个求解预算）"))

    fastest = max(z0.rate_max)
    ok = p.mocap_jump_speed_max > MPC_XY_VEL_MAX and p.mocap_jump_rate_max > fastest
    out.append(Check(g, "位姿合理性阈值比真机动宽", ok,
                     f"位移上限 {p.mocap_jump_speed_max:g} m/s（须 > {MPC_XY_VEL_MAX:g} = 飞控的 "
                     f"`MPC_XY_VEL_MAX`）、转角上限 {p.mocap_jump_rate_max:g} rad/s"
                     f"（须 > {fastest:g} = 模型里最快的角速率）—— ★ 反过来的错是**无故停发**："
                     "一次正常机动被读成跳变，飞机在空中就收不到指令了"))

    ok = int(p.inject_solve_fail_after_frames) == 0
    out.append(Check(g, "故障注入关着", ok,
                     f"inject_solve_fail_after_frames = {p.inject_solve_fail_after_frames}"
                     + ("" if ok else "　⟹ **装桨前必须设回 0**：它会让每一拍的求解结果"
                                      "都被换成坏解，飞机收到的全是兜底帧")))
    return out


# ───────────────────────── ④ 停机条件参数化（零通道号）─────────────────────────

def check_stop_conditions(p, sources: Sequence[Path] = ()) -> List[Check]:
    """停机条件：四个字段**参数化**，且程序里**没有通道号**。

    ★ 为什么零通道号：`phase1/issues/02` 已 resolved —— **急停 = ch8 高值 = PX4 硬件杀车，
      程序不读急停通道**；节点侧的停机只能走两条既有路径（打杆接管 / ch6 切模式），
      而它们在节点这一侧**都表现为"离开 OFFBOARD"** ⟹ 一旦代码里出现通道号，
      就说明有人把"读遥控器"这条已经被否掉的路又写回来了。
    """
    g = "停机条件"
    out: List[Check] = []

    have = {f.name for f in fields(StopConfig)}
    missing = sorted(STOP_FIELDS - have)
    extra = sorted(have - STOP_FIELDS)
    out.append(Check(g, "文档那四个停机条件都在", not missing,
                     (f"四个条件齐（{len(STOP_FIELDS)} 个）" if not missing
                      else f"缺 {missing} ⟹ 先去改 `phase2-implementation.md` §4 那张表")
                     + (f"；另有闸门字段 {extra}（确认它们确实属于'不发'的条件）" if extra else "")))

    declared = {name for name, _d, _t in _param_table()}
    bad_names = [n for n in declared if _looks_like_channel(n)]
    out.append(Check(g, "没有通道类参数", not bad_names,
                     f"声明的参数里没有通道类名字（扫到 {bad_names}）" if bad_names
                     else f"{len(declared)} 个参数，无 channel/rc_ 之类"))

    hits = _channel_identifiers(sources)
    out.append(Check(g, "源码里没有通道号标识符", not hits,
                     "扫描 " + ", ".join(x.name for x in sources) + "："
                     + ("干净（注释里引用 `phase1/issues/02` 的 ch8 不算）" if not hits
                        else f"命中 {hits}")))
    return out


def _param_table():
    """节点的参数表 —— **从 `mpc_node` 里拿**，不在本文件里复写一份。"""
    from nmpc.mpc_node import _PARAMS  # noqa: PLC0415  （要 ROS/acados，故延后）

    return _PARAMS


# ───────────── ⑤ 下行前置：两个"整条下行静默死掉"的开关（ticket 13 §二⑤bis/⑥）─────────────
#
# ★ 为什么这两条值得进自检：它们**都不在我们仓库里**，而且失败时**飞控侧不说话**——
#   话题上照发、`ros2 topic hz` 正常、进程不报错，飞机只是**什么都不做**（甚至掉高度）。
#   与本文件其它组的区别是：那些组查的是"我们自己的参数对不对"，这两条查的是
#   **别人的两项配置有没有把我们的下行通道关掉** —— 属于"装桨前必须知道"的事。

#: 飞控参数快照（`docs/assets/z0.params`）里数据行的列序：
#: `默认值 <TAB> ? <TAB> 参数名 <TAB> 值 <TAB> 类型`（导出件，第一手见文件头的说明段）。
#: ⚠️ **按名字找，不按行号**：那份文件自己写着"每次重导出，行号都整体平移"。
FCU_SNAPSHOT_NAME_COL = 2
FCU_SNAPSHOT_VALUE_COL = 3

#: 收 `SET_ATTITUDE_TARGET` 的硬前置（`mavlink_receiver.cpp:1516-1518` 的
#: `if (_mavlink->get_forward_externalsp() && …)`；参数定义 `mavlink_params.c:112-121`）。
FCU_FWDEXTSP = "MAV_FWDEXTSP"

#: mavros 的 `setpoint_raw` 插件里那个缩放（`setpoint_raw.cpp:58` 声明、`:294-313` 使用）。
MAVROS_THRUST_SCALING = "thrust_scaling"

#: 那个插件作为**节点**跑时用的全名（mavros 给每个插件建一个节点）。
#: ★ 必须读**这个活节点**上的参数 —— 读 `px4_config.yaml` 会得到假绿，理由见
#:   `read_live_thrust_scaling` 的 docstring。
MAVROS_SETPOINT_RAW_NODE = "/mavros/setpoint_raw"


def read_fcu_snapshot(path: Path, name: str) -> Optional[str]:
    """从飞控参数快照里按**名字**取一个值（找不到 ⟹ `None`，不猜）。

    ★ 用 UTF-8 显式解码 + `split("\\n")`（**不用 `splitlines()`**）：那份文件里夹着
      `★` 之类的多字节字符，按 latin-1 解再 `splitlines()` 会凭空多切出几行
      —— 这个坑文件头自己记着（本仓核对时踩过一次）。
    """
    for line in path.read_text(encoding="utf-8").split("\n"):
        if not line or line.startswith("#"):
            continue
        cols = line.split("\t")
        if len(cols) > FCU_SNAPSHOT_VALUE_COL and cols[FCU_SNAPSHOT_NAME_COL] == name:
            return cols[FCU_SNAPSHOT_VALUE_COL]
    return None


def read_live_thrust_scaling(node_fqn: str = MAVROS_SETPOINT_RAW_NODE,
                             timeout_s: float = 5.0) -> Tuple[Optional[float], str]:
    """从**活着的** mavros 节点上读 `thrust_scaling` ⟹ `(值 或 None, 说明)`。

    ★★ **为什么不能读 `px4_config.yaml`**（2026-10-13 实机定的案，票 13 第六轮）：
      mavros 的插件节点是**运行期动态建**的，`plugin.cpp:34-37` 给它们的 NodeOptions 写死
      `use_global_arguments(false)` ⟹ 进程级 `--params-file` **到不了它们**。
      ⟹ `px4_config.yaml` 里那 27 个 `/**/<插件>:` 块**全是死文本**：实机
      `ros2 param get /mavros/setpoint_raw thrust_scaling` = `nan`（= `setpoint_raw.cpp:58`
      的声明默认值），而同一份 yaml 里写着 `1.0`。
      用**精确节点名** `/mavros/setpoint_raw:` 再单独喂一份 params-file，**仍然是 `nan`**
      ⟹ 不是"通配符没匹配上"，是插件节点根本不读任何 params-file。
      **所以"读文件"得到的是一个假绿**：链已经死了，文件里还是 1.0。

    ⛔ **与飞机无关**：飞控**不通电**、`fcu_url` 指着一个不存在的设备（`link[1000] open failed`），
      64 个插件节点照样建出来、这个参数照样读得到（2026-10-13 实测）。
      ⟹ 这一条仍然满足本组的纪律「**不需要飞控通电**」，只需要 **mavros 起着**。

    ⛔ **真的读不到就报"验不了"（红），没有"跳过"这一档** —— 与飞控快照那条同一条纪律：
      这一组守的是"整条下行静默死掉"，而静默的死法没有别的检测点。
    """
    try:
        import rclpy  # noqa: PLC0415
        from rcl_interfaces.msg import ParameterType  # noqa: PLC0415
        from rcl_interfaces.srv import GetParameters  # noqa: PLC0415
    except ImportError:  # pragma: no cover - 没 source ROS 时走这里
        return None, "导不进 rclpy / rcl_interfaces ⟹ 这一条**验不了**（先 source ROS 环境）"

    owned = not rclpy.ok()
    node = None
    try:
        if owned:
            rclpy.init()
        node = rclpy.create_node("preflight_thrust_scaling_probe")
        cli = node.create_client(GetParameters, f"{node_fqn}/get_parameters")
        if not cli.wait_for_service(timeout_sec=timeout_s):
            return None, (f"**{node_fqn} 不在** ⟹ mavros 没起（或没起全）。"
                          "★ 它**可以在飞控不通电时**单独起来做这一条："
                          "`ros2 launch bringup mavros.launch.py`（fcu_url 指哪都行）")
        req = GetParameters.Request()
        req.names = [MAVROS_THRUST_SCALING]
        fut = cli.call_async(req)
        rclpy.spin_until_future_complete(node, fut, timeout_sec=timeout_s)
        if not fut.done() or fut.result() is None:  # pragma: no cover - 服务在但不应答
            return None, f"{node_fqn} 上这个参数**没答**（超时 {timeout_s:.0f} s）"
        pv = fut.result().values[0]
        #: `PARAMETER_NOT_SET` = 节点上没声明它 —— 与"声明了、值是 NAN"是两回事，
        #: 但对本检查是同一个结论（都不是 1.0），所以只在说明文字里区分。
        if pv.type == ParameterType.PARAMETER_NOT_SET:
            return None, f"{node_fqn} 上**没有声明** {MAVROS_THRUST_SCALING} 这个参数"
        if pv.type == ParameterType.PARAMETER_DOUBLE:
            return float(pv.double_value), f"{node_fqn} 活值"
        return None, (f"{node_fqn} 上 {MAVROS_THRUST_SCALING} 的类型是 {pv.type}"
                      "（不是 double）⟹ 认不出，**不猜**")
    finally:
        if node is not None:
            node.destroy_node()
        if owned and rclpy.ok():  # pragma: no cover - 只在真起过 rclpy 时走
            rclpy.shutdown()


def check_downlink_prereqs(
        repo: Optional[Path],
        live_reader: Optional[Callable[[], Tuple[Optional[float], str]]] = None) -> List[Check]:
    """换到 mavros 载体之后的两个**硬前置**（ticket 13 §二⑤bis/⑥）。

    | 项 | 是什么 | 失败时现象 |
    |---|---|---|
    | `MAV_FWDEXTSP = 1` | **飞控参数**：`SET_ATTITUDE_TARGET` handler 的守卫 | 我们发的每一条 setpoint 被飞控丢掉，**飞控侧没有日志** |
    | `thrust_scaling = 1.0` | **mavros 参数**（`setpoint_raw` 插件） | 缺失 ⟹ `thrust != 0` 的帧**整帧丢**（只打一句节流 ERROR）；`0` ⟹ 推力恒 0、掉高度 |

    ★ 两者都不是我们的参数 ⟹ 本组**在快档里也跑**（一个是仓库里的快照，一个从**活着的**
      mavros 节点上读；**都不需要飞机通电** —— 飞控不通电时插件节点照样在，实测见
      `read_live_thrust_scaling`）。⛔ **没有"跳过"这一档**：读不到就红，并说清怎么让它是绿的 ——
      它们守的是"整条下行静默死掉"，而静默的死法没有别的检测点。

    `live_reader` 只为**测试**注入（默认真读 `/mavros/setpoint_raw` 活节点）。
    """
    g = "下行前置"
    out: List[Check] = []

    # ① 飞控参数（读**导出快照**：现场没有别的离线来源）
    snap = (repo / "docs" / "assets" / "z0.params") if repo is not None else None
    if snap is None or not snap.is_file():
        out.append(Check(g, f"{FCU_FWDEXTSP} = 1", False,
                         f"找不到飞控参数快照（{snap or '仓库根没找到'}）⟹ 这一条**验不了**。"
                         "它不是「可跳过」的项：值为 0 时整条下行被飞控静默丢掉"))
    else:
        value = read_fcu_snapshot(snap, FCU_FWDEXTSP)
        ok = value == "1"
        out.append(Check(g, f"{FCU_FWDEXTSP} = 1（飞控参数）", ok,
                         f"快照里 {FCU_FWDEXTSP} = {value!r}（{snap.name}，按名字查不按行号）"
                         + ("" if ok else "　⟹ **=0 时 `SET_ATTITUDE_TARGET` 整条被飞控丢掉**"
                                          "（`mavlink_receiver.cpp:1516-1518`），表现与「话题名"
                                          "写错」几乎一样")
                         + "；★ 实机起飞前请再读一次**现值**（快照是导出时的值）："
                           "QGC 的 Parameters 里搜这个名字，或 `ros2 param list | grep -i "
                           + FCU_FWDEXTSP + "` 找到它挂在哪个节点上再 `ros2 param get`"))

    # ② mavros 参数（★ 读**活节点**，不读 yaml —— 读 yaml 会得到假绿，理由见
    #    `read_live_thrust_scaling` 的 docstring；这是票 13 第六轮按实机读数改的口径）
    reader = live_reader if live_reader is not None else read_live_thrust_scaling
    value, where = reader()
    ok = (isinstance(value, (int, float)) and not isinstance(value, bool)
          and math.isclose(float(value), 1.0, rel_tol=0.0, abs_tol=1e-12))
    out.append(Check(g, f"{MAVROS_THRUST_SCALING} = 1.0（mavros **活值**）", ok,
                     f"{where}：{MAVROS_THRUST_SCALING} = {value!r}"
                     + ("（`setpoint_raw.cpp:58` 的声明默认就是 `NAN`）" if value is None or
                        (isinstance(value, float) and math.isnan(value)) else "")
                     + ("" if ok else "　⟹ **`nan`/缺失 ⟹ `thrust != 0` 的帧整帧被丢"
                                      "（连 body_rate 一起，`setpoint_raw.cpp:294`）**；"
                                      "**=0 ⟹ 推力恒 0（掉高度）**；只有 1.0 是我们要的口径。"
                                      "　修法：`ros2 param set /mavros/setpoint_raw "
                                      f"{MAVROS_THRUST_SCALING} 1.0`")))
    return out


def _looks_like_channel(name: str) -> bool:
    import re  # noqa: PLC0415

    return bool(re.match(_CHANNEL_RE, name))


def _channel_identifiers(sources: Sequence[Path]) -> List[str]:
    """用 `ast` 扫标识符与字符串常量（**不扫注释/docstring**）。"""
    import re  # noqa: PLC0415

    pat = re.compile(_CHANNEL_RE)
    hits: List[str] = []
    for src in sources:
        tree = ast.parse(src.read_text(encoding="utf-8"))
        for node in ast.walk(tree):
            names = []
            if isinstance(node, ast.Name):
                names = [node.id]
            elif isinstance(node, ast.Attribute):
                names = [node.attr]
            elif isinstance(node, ast.keyword) and node.arg:
                names = [node.arg]
            elif isinstance(node, ast.Constant) and isinstance(node.value, str):
                names = [node.value]
            hits += [f"{src.name}:{node.lineno} {n}" for n in names if pat.match(n)]
    return hits


# ───────────────────────────── ⑤ --solve：合成一个数 ─────────────────────────────

def check_solver(z0: Z0Params, warmup: int = 10) -> List[Check]:
    """在悬停点解一次（**需要 acados**）：模型 + 代价 + 符号 + 归一化合成一个数。

    ★ 热身是必须的：`SQP_RTI` 每周期只迭代一次，全新实例的第 0 次 `solve()` 给的是
      "从出厂迭代点走一步"，不是最优解（实测悬停点第 0 次 F = 7.81 N、10 次后 = m·g）。
    """
    import numpy as np  # noqa: PLC0415

    from nmpc.ocp import OfflineOcp  # noqa: PLC0415

    g = "求解器（--solve）"
    ocp = OfflineOcp(params=z0)
    x0 = ocp.hover_state()
    x_ref = np.tile(x0, (ocp.config.N + 1, 1))
    sol = None
    for _ in range(max(1, warmup)):
        sol = ocp.solve(x0, x_ref)
    assert sol is not None
    f_n, rates = float(sol.u[0]), [float(v) for v in sol.u[1:]]

    out = [Check(g, f"悬停配平 F ≈ m·g（{warmup} 次热身）", math.isclose(f_n, z0.t_hover, rel_tol=1e-3),
                 f"F = {f_n:.6f} N，m·g = {z0.t_hover:.6f} N（内步长 {ocp.config.tf / ocp.config.N * 1e3:.1f} ms、"
                 f"N = {ocp.config.N}）")]
    out.append(Check(g, "悬停不产生角速率", all(abs(w) < 1e-6 for w in rates),
                     f"ω = {['%+.2e' % w for w in rates]} rad/s"))

    sp = px4_rates_setpoint((0.0, 0.0, 0.0), f_n, z0.t_max)
    ok = math.isclose(sp.thrust_body_z, -z0.hover_thrust, rel_tol=5e-3)
    out.append(Check(g, "发出去的归一化推力 = −hover_thrust", ok,
                     f"thrust_body[2] = {sp.thrust_body_z:+.5f}（−hover_thrust = {-z0.hover_thrust:+.5f}）"
                     " —— 这一条把模型、代价、符号表与归一化常数**串成一个数**"))
    return out


# ───────────────────────────── CLI ─────────────────────────────

def build_node_params(overrides: Dict[str, str]):
    """按节点的参数表造一份 `NodeParams`（`-p name:=value` 与节点同一套写法）。"""
    from nmpc.mpc_node import NodeParams  # noqa: PLC0415

    values: Dict[str, object] = {}
    kinds = {}
    for name, default, _type in _param_table():
        values[name] = default
        kinds[name] = type(default)
    for name, raw in overrides.items():
        if name not in values:
            raise SystemExit(f"✗ 没有这个参数：{name}（拼错的名字 rclpy 是不会报的，所以这里当场退）")
        kind = kinds[name]
        try:
            if kind is bool:
                values[name] = raw.strip().lower() in ("1", "true", "yes")
            else:
                values[name] = kind(raw)
        except ValueError as exc:
            raise SystemExit(f"✗ 参数 {name} 的值 {raw!r} 转不成 {kind.__name__}：{exc}") from exc

    known = {f.name for f in fields(NodeParams)}
    kw = {k: v for k, v in values.items() if k in known}
    kw["hold_offset"] = tuple(values[f"hold_offset_{a}"] for a in "xyz")
    mapped = set(values) - (set(kw) | {"hold_offset_x", "hold_offset_y", "hold_offset_z"})
    if mapped:  # 参数表与字段表脱节 —— 宁可炸掉，别悄悄少查一个
        raise SystemExit(f"✗ 参数表里有字段映射不上：{sorted(mapped)}")
    return NodeParams(**kw)


def _parse_overrides(argv: Sequence[str]) -> tuple:
    overrides, rest, i = {}, [], 0
    while i < len(argv):
        if argv[i] == "-p":
            if i + 1 >= len(argv) or ":=" not in argv[i + 1]:
                raise SystemExit("✗ `-p` 的写法是 `-p name:=value`")
            name, raw = argv[i + 1].split(":=", 1)
            overrides[name] = raw
            i += 2
            continue
        rest.append(argv[i])
        i += 1
    return overrides, rest


def _print(checks: Sequence[Check]) -> int:
    group = None
    for c in checks:
        if c.group != group:
            group = c.group
            print(f"\n── {group} ──")
        print(f"  {'✅' if c.ok else '❌'} {c.name}：{c.detail}")
    bad = [c for c in checks if not c.ok]
    print()
    if bad:
        print(f"✗ {len(checks)} 项里 {len(bad)} 项不过 ⟹ **先修掉再装桨**："
              + "、".join(c.name for c in bad))
        return 1
    print(f"✓ {len(checks)} 项全部通过。")
    return 0


def main(argv: Optional[Sequence[str]] = None, *,
         live_reader: Optional[Callable[[], Tuple[Optional[float], str]]] = None) -> int:
    """`live_reader` 只为**测试**注入（命令行不暴露它；默认真读活节点）。"""
    ap = argparse.ArgumentParser(description="装桨之前的静态自检（ticket 06）")
    ap.add_argument("--solve", action="store_true",
                    help="额外建一次求解器并在悬停点解一次（需要 acados，几十秒）")
    ap.add_argument("--repo", type=Path, default=None, help="仓库根（默认往上找 docs/control/sign-map.md）")
    # ☠️ 2026-10-13：`--mavros-config <px4_config.yaml>` 这一项**删了**（票 13 第六轮）。
    #    它喂的是"读 yaml"那条路，而实机读数证明那份 yaml 里的 `/**/setpoint_raw:` 块
    #    **根本没进 mavros**（`plugin.cpp:35` 的 `use_global_arguments(false)`）⟹
    #    照它判会得到一个**假绿**。现在这一条改成读活节点，没有可指的文件了。
    args, rest = ap.parse_known_args(argv)
    overrides, extra = _parse_overrides(rest)
    if extra:
        raise SystemExit(f"✗ 不认识的参数：{extra}")

    p = build_node_params(overrides)
    pkg = Path(__file__).resolve().parent
    checks: List[Check] = []
    checks += check_params(p)
    repo = Path(args.repo) if args.repo else find_repo_root()
    doc = (repo / "docs" / "control" / "sign-map.md") if repo else None
    if doc is None or not doc.is_file():
        checks.append(Check("符号对账", "找得到仓库根", False,
                            f"{doc or '往上都没找到'} 不存在 ⟹ 符号对账做不了。这一条**算失败、"
                            "不算跳过**（它是本脚本的核心，静默跳过等于给人虚假的安全感）；"
                            "在 `install/` 里跑就用 `--repo <仓库根>`。"))
    else:
        checks += check_signs(repo)
    checks += check_normalization(p.z0_params())
    checks += check_fallback(p, p.z0_params())
    checks += check_stop_conditions(p, sorted(pkg.glob("*.py")))
    checks += check_downlink_prereqs(repo, live_reader)
    if args.solve:
        checks += check_solver(p.z0_params())

    print(f"z0 起飞前静态自检 | {p.describe()}")
    return _print(checks)


if __name__ == "__main__":  # pragma: no cover
    raise SystemExit(main())
