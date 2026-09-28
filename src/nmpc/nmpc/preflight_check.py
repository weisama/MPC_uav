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
from typing import Callable, Dict, FrozenSet, List, Optional, Sequence

#: 直接 `python3 src/nmpc/nmpc/preflight_check.py` 跑时（不经 colcon）的引导，
#: 与 `offboard_smoke.py` 同一形态：把 `src/nmpc` 塞进 sys.path 好让 `nmpc` 包能 import。
if __package__ in (None, ""):  # pragma: no cover - 只在直接执行时走
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from nmpc.node_core import StopConfig  # noqa: E402
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

    ok = z0.t_max < 40.0
    out.append(Check(g, "推力上限按**电池**取值", ok,
                     f"T_max = {z0.t_max:.3f} N（电机上限 40 N 是 M4 说的那个**不该**用的数；"
                     f"走到那儿说明又按了电机）"))

    ok = p.heartbeat_hz >= 2.0
    out.append(Check(g, "心跳 ≥2 Hz", ok,
                     f"heartbeat_hz = {p.heartbeat_hz} Hz（PX4 的 `COM_OF_LOSS_T` 是 1 s；"
                     f"低于 2 Hz 就可能被判 offboard 信号丢失 ⟹ 进不去 OFFBOARD）"))

    ok = p.rate_hz >= p.heartbeat_hz > 0.0 and p.viz_hz > 0.0
    out.append(Check(g, "定时器频率", ok,
                     f"rate = {p.rate_hz} Hz / 心跳 = {p.heartbeat_hz} Hz / viz = {p.viz_hz} Hz"))

    ok = 0.0 < p.tau_s < 0.2
    out.append(Check(g, "外推 τ 在合理区间", ok,
                     f"tau_s = {p.tau_s} s（票面 50 ms；τ 大过一拍就会明显过冲）"))

    ok = p.warmup_cycles >= 1
    out.append(Check(g, "热身拍数 ≥1", ok,
                     f"warmup_cycles = {p.warmup_cycles}（SQP_RTI 的第 0 次 solve 给的是出厂迭代点："
                     f"实测第 0 次 ω_y = 0.0000、第 1 次 3.8400 ⟹ 0 会把那一帧发出去）"))

    ok = p.mocap_timeout_s > 0.0 and p.status_timeout_s > 0.0
    out.append(Check(g, "停机时限为正", ok,
                     f"mocap > {p.mocap_timeout_s} s / status > {p.status_timeout_s} s"))

    ok = p.status_timeout_s >= 0.5
    out.append(Check(g, "状态时限 ≥ 上游一个周期", ok,
                     f"status_timeout_s = {p.status_timeout_s} s（假 PX4 与真 commander 都是 2 Hz "
                     f"⟹ 短于 0.5 s 会把正常的上游判成静默）"))

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


def main(argv: Optional[Sequence[str]] = None) -> int:
    ap = argparse.ArgumentParser(description="装桨之前的静态自检（ticket 06）")
    ap.add_argument("--solve", action="store_true",
                    help="额外建一次求解器并在悬停点解一次（需要 acados，几十秒）")
    ap.add_argument("--repo", type=Path, default=None, help="仓库根（默认往上找 docs/control/sign-map.md）")
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
    checks += check_stop_conditions(p, sorted(pkg.glob("*.py")))
    if args.solve:
        checks += check_solver(p.z0_params())

    print(f"z0 起飞前静态自检 | {p.describe()}")
    return _print(checks)


if __name__ == "__main__":  # pragma: no cover
    raise SystemExit(main())
