#!/usr/bin/env python3
"""±30 % 单参数失配扫描（ticket 12）—— 把"这个 MPC（模型预测控制）依赖精确模型吗"
从**假设**变成**一条随参数变化的曲线**。

工具形状：**夹具 + 纯函数**。

| 层次 | 在哪 | 判什么 |
|---|---|---|
| 飞行 | 复用 **A 档**那套仪器（`verify_tier_a_hover.py`，真 PX4、状态源 = PX4 的 EKF 帧） | 每次只换**覆盖层**：`z0.param` ⟹ `z0-mass+30.param` 等 |
| 判读 | 本文件的纯函数（`extract` / `divergence` / `summarise`） | 发散 / 退化 / 稳定，以及与**基准散度**的差 |

★ **为什么复用 A 档而不是新写一套闭环**：`verify_tier_a_hover.py` 的判据是 ticket 09
  一条一条踩出来的（棘轮、镜像井、别名闸门、⓪ 仪器自检）。另起一套 = 把那些坑重踩一遍，
  而且**两张票的读数不可比**。这里只加两样它没有的东西：**换覆盖层**与**逐场读回参数**。

★ **为什么发散要自己判**：A 档判据压根判不了发散 —— 第 6 场（`docs/control/tier-a-sitl-hover.md`）
  自转 13.6 圈时"位置那几条判据**全是 ✅**"。所以 `divergence()` 单列三条硬线
  （真值高度越过 3 m / 水平飞出 3 m / 机头累计转过一圈）。

⚠️ **本张是仿真结论，不能外推**：SIH 的推力曲线是线性的（`THR_MDL_FAC = 0` 是**刻意**的背离，
  见 `sitl-z0-plant.md` §5.2）、电池不掉压、动捕时延在 A 档根本不存在（状态源是 EKF 帧）。
  sim-to-real 的这三条 gap **本阶段未被覆盖**，见 `docs/control/mismatch-sweep.md` §5。

用法（**先 source 工作空间**，与其它 tools 一致）：

    source /opt/ros/humble/setup.bash && source ~/px4_msgs_install/setup.bash
    source install/setup.bash

    # 单个变体（调试用；一场 ~3–5 min）
    python3 tools/px4-sitl/mismatch_sweep.py --variants z0 --repeats 1 --outdir /tmp/sweep
    # 全扫（11 个变体 × 1 + 基准重复）
    python3 tools/px4-sitl/mismatch_sweep.py --repeats 1 --outdir /tmp/sweep
    # 只汇总（不飞）：吃 outdir 里的 obs-*.json
    python3 tools/px4-sitl/mismatch_sweep.py --summarize /tmp/sweep --json /tmp/sweep/summary.json
    # 重探控制器代码身份（修旧版仪器记的"驱动视图"，原值留 `controller_code_prev`）
    python3 tools/px4-sitl/mismatch_sweep.py --outdir /tmp/sweep --fix-controller-code --dry-run

    # 未知参数原样透传给 A 档工具（`--hold-s` / `--rate-hz` / `--truth-hz` …）
    python3 tools/px4-sitl/mismatch_sweep.py --variants z0-mass+30 -- --hold-s 25
"""

from __future__ import annotations

import argparse
import bisect
import hashlib
import json
import math
import os
import re
import statistics
import subprocess
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[1]
for _p in (HERE, HERE / "z0-plant", REPO / "src/nmpc"):
    # ★ `src/nmpc` 也塞进去（与 `verify_tier_a_hover.py` 同一条理由）：判据与口径里的常数
    #   要读**源码**那一份，不读 `install/` 的拷贝 —— 后者改了源码没 `colcon build` 时是旧值，
    #   而报告上看不出来。
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

import plant_spec  # noqa: E402  （变体与参数值的**单一真源**，不另抄一份）

#: 票面的 5 个旋钮（旋钮名 → 动哪些参数）**直接引用**真源 —— 抄一份的话，
#: 改旋钮的人改了 `plant_spec`、本文件还在读旧名字，而报告上看不出来。
KNOBS = plant_spec.KNOBS

#: 扫描的变体后缀（`±30 %`）。★ `z0-thrcurve` **不在**扫描里：它是 ticket 08 的
#: 验收工具（非扫描项），混进来会让"±30 % 的边界"这句话多出一个不属于它的点。
SIGNS = ("+", "-")
PCT = int(plant_spec.MISMATCH_PCT * 100)

#: 发散的三条硬线（运算定义，不是形容词）。★ 取值理由：都在**票面工况**（1 m 悬停、
#: 位置保持）的一个数量级之外，够宽 ⟹ 不会把"抖"读成"散"；够窄 ⟹ 真散了一定抓到。
DIV_Z_M = 3.0        # 真值高度越过起飞点 3 m（爬飞 / 掉下去）
DIV_XY_M = 3.0       # 真值水平飞出 3 m
DIV_YAW_RAD = 2 * math.pi   # EKF 帧机头累计转过一圈（自转；ticket 09 第 6 场是 13.6 圈）
DIV_HOLD_M = 0.4     # 末段真值抬升不足这个数 ⟹ 末段不在空中（掉高/坠地）

#: 汇总表里逐读数与基准比的量（名字即 `extract()` 的键）。
METRIC_KEYS = ("climb_ekf", "e_z_ekf", "p2p_ekf", "drift_ekf", "yaw_span_ekf",
               "solve_steady_max", "solve_p50", "climb_gt", "p2p_gt")

#: A 档判据"③ 悬停高度 1 m ± 0.15"的目标高度（ENU，米）。判据在 `verify_tier_a_hover` 里，
#: 这里只用来算"稳态误差"这个读数（`climb_ekf − HOLD_Z_M`）。
HOLD_Z_M = 1.0

#: 同一条判据的**容差带**（米）。票面把它写进结论里 —— "±30 % 会挂"太粗，
#: 有 `vertical_gain()` 反推的刚度就能翻译成"**悬停口径容许多少**"。改判据时这里要跟着改。
HOVER_BAND_M = 0.15

#: 重力（与 `verify_offboard_smoke.G0` 同值：判据里的常数一律从机型文件推）。
G0 = 9.80665


def plant_hover_thrust() -> float:
    """SITL 被控对象的**标称**归一化悬停推力 = `m·g / (4·SIH_T_MAX)`（从 `plant_spec` 真源推）。

    ★★ **2026-10-09 重搭后它与 `nmpc/params.py` 的 `HOVER_THRUST` 同档**：
      被控对象在 ticket 21（Phase 3 的 `04`）里按**真机实测口径**重搭，
      `T_MAX_TOTAL = 46.3027 N`（**按电机**，不按电池）⟹ 解析悬停 `≈0.158846`，
      节点默认 `0.1589` —— 差 0.03 %（就是 `g` 取 9.80665 vs 9.81 的差）。
      ⚠️ **重搭前**两者差 **3.92 倍**（被控对象解析值 `0.6233` vs 节点 `0.1589`），
      拿 `0.1589` 去飞那时的机型会让飞机不离地 —— 那段历史账**已随重搭归零**，别再引。
    ★ 保留这个函数是为了 `--hover-thrust auto` 的语义：从**被控对象真源**现算 ⟹ 永远与它同源。
    """
    nominal = expected_params("z0")
    return nominal["SIH_MASS"] * G0 / (4.0 * nominal["SIH_T_MAX"])


#: `ros2 run nmpc <这个名字>`（A 档夹具的 argv 里写死的那个）⟹ 入口脚本的文件名。
ENTRY_SCRIPT = "mpc_node.py"

#: ★★ **本进程内的 `import nmpc` 不能用来判"正在飞的是哪一份"** ——
#:    本文件第 53 行**故意**把 `REPO/"src/nmpc"` 插到了 `sys.path` 最前面
#:    （为了读**源码**那份常数，不读 `install/` 的拷贝）。于是本进程里 `import nmpc`
#:    **必然**落到 `src/`；而节点子进程拿到的是**环境变量 `PYTHONPATH`**，那次
#:    `sys.path.insert` 传不过去 ⟹ 节点飞的是 `install/`。两者**恰好相反**。
#:    2026-09-29 10:14 实测：本进程 `src/nmpc/nmpc`，干净子进程
#:    `install/nmpc/local/lib/python3.10/dist-packages/nmpc`。
#:    第一版 `controller_code()` 就是这么把自己的视图记成了节点的身份（**假身份**，
#:    而且打印出来完全像真的），故改成探针子进程。
#:    ⚠️ 反过来的直觉是错的：入口脚本自己会插 `parents[1]`（`mpc_node.py:150`），
#:    但那**不代表"谁被执行谁赢"** —— `install/nmpc/lib/nmpc/` **没有 `__init__.py`**，
#:    只是 **PEP 420 的命名空间段**，会被 `PYTHONPATH` 上那个**有 `__init__.py` 的正规包**
#:    盖过 ⟹ 最终仍由 `PYTHONPATH` 顺序决定（`run_matrix.sh:49` 的
#:    `export PYTHONPATH=.../src/nmpc` 就是这么生效的）。⟹ 探身份只能**测**，不能推。
_PROBE_SRC = "import nmpc;print('NMPC_PKG=' + nmpc.__file__)"


def sha256_16(path: Path) -> str:
    """16 位十六进制 = 64 bit，够把"是不是同一份文件"钉死，且读起来不糊。"""
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()[:16]


def resolve_flying_package() -> Path:
    """**干净子进程**里 `import nmpc` 落到哪个目录 = 节点真正 import 的那一份。

    为什么值得多起一个 `python -c`：这是"这一轮 13 场飞的是不是同一份代码"这句话的
    **唯一**第一手来源；用本进程的 `import` 顶替它，就会得到一个**看不出来的**假身份。
    """
    if not os.environ.get("PYTHONPATH"):
        # 不是硬错误（也可能靠 `.pth` 生效），但没 source 工作空间时**十有八九**探针会失败。
        print("[sweep] ⚠️ 环境里没有 PYTHONPATH，控制器代码探针可能落到非预期的一份",
              file=sys.stderr)
    p = subprocess.run([sys.executable, "-c", _PROBE_SRC],
                       capture_output=True, text=True, env=dict(os.environ))
    for line in reversed(p.stdout.splitlines()):
        if line.startswith("NMPC_PKG="):
            return Path(line.split("=", 1)[1].strip()).resolve().parent
    tail = (p.stderr or p.stdout or "").strip().splitlines()[-1:]
    raise RuntimeError(f"探针没报出路径（rc={p.returncode}）：{tail[0] if tail else '无输出'}")


def resolve_entry_script(entry: str = ENTRY_SCRIPT) -> Path | None:
    """`ros2 run` 真正 exec 的那个文件：`<pkg prefix>/lib/nmpc/<entry>`。

    ★ 它与**包内同名文件**不一定同文（`colcon` 会往 `lib/<pkg>/` 里也拷一份入口脚本），
      所以两份都要记。拿不到就返回 `None`（不猜）。
    """
    try:
        p = subprocess.run(["ros2", "pkg", "prefix", "nmpc"],
                           capture_output=True, text=True, env=dict(os.environ), timeout=30)
    except (OSError, subprocess.TimeoutExpired):
        return None
    lines = [ln for ln in p.stdout.splitlines() if ln.strip()]
    if p.returncode != 0 or not lines:
        return None
    f = Path(lines[-1].strip()) / "lib" / "nmpc" / entry
    return f if f.exists() else None


def _hash_py_dir(pkg_dir: Path) -> dict:
    """目录里**全部** `.py` 的哈希（不给名单，免得漏掉新加的文件）。"""
    return {f.name: {"path": str(f), "sha256": sha256_16(f)}
            for f in sorted(Path(pkg_dir).glob("*.py"))}


def _node_default_hover_thrust() -> float:
    """`nmpc/params.py` 里那份**未被打补丁**的 `HOVER_THRUST`（= z0 真机满电口径）。

    ⚠️ `run_variant()` 会在导入 A 档工具**之前**把这个模块常量改成 SITL 口径
      ⟹ 那个进程里再调本函数拿到的是**改过的**值（`run_variant` 因此自己先取一份存着）。
      本函数是给**恢复**路径用的：`recover_sweep` 跑在干净进程里，读到的就是真默认值。
    """
    import nmpc.params as np_params
    return float(np_params.HOVER_THRUST)


def controller_code() -> dict:
    """**这一轮飞的到底是哪一份控制器代码** —— 路径 + 逐文件 sha256（第一手留档）。

    ★ 为什么非有不可（本张撞出来的）：`ros2 run nmpc` 用的是 **ament 装出来的那一份**
      （`install/nmpc/...`，非 symlink 安装 ⟹ 是**拷贝**），而 `src/nmpc/` 里可能
      正被别人编辑。同一轮扫描里若"某一半飞的是 08:59 的构建、另一半飞的是刚改过的 src"，
      这张表就不是一个实验了 —— 而报告上**看不出来**（它照样把 13 行印得整整齐齐）。
      ⟹ 逐场把**实际解析到的包路径与各文件哈希**记进 `obs["sweep"]["controller_code"]`。

    返回里三样东西各有用处：
      `package` / `files`  —— 身份本身（节点 import 的那一份）；
      `entry`             —— `ros2 run` exec 的入口脚本（另一份文件，落档备查）；
      `driver_view` + `polluted` —— **本进程** `import nmpc` 落到哪、与节点是否不同。
        故意留着：`polluted=True` 是**常态**（见上面 `_PROBE_SRC` 那段），
        它不是错误，而是"**别拿驱动进程的 import 当节点的身份**"这条教训的常驻读数。
    """
    out: dict = {"probe": "干净子进程（`python -c 'import nmpc'`，环境变量与节点一致）",
                 "package": None, "files": {}, "entry": None,
                 "driver_view": None, "polluted": None, "note": None}
    try:
        pkg_dir = resolve_flying_package()
    except RuntimeError as exc:
        out["note"] = (f"★ **探针失败 ⟹ 这一场的控制器代码身份不明**：{exc}。"
                       "别把本进程的 `import nmpc` 当退路（它会被第 53 行的 "
                       "`sys.path.insert` 顶到 `src/` 上）。")
        return out
    out["package"] = str(pkg_dir)
    out["files"] = _hash_py_dir(pkg_dir)

    entry = resolve_entry_script()
    if entry is not None:
        out["entry"] = {"path": str(entry), "sha256": sha256_16(entry)}
        inside = pkg_dir / entry.name
        if inside.exists():
            out["entry"]["same_as_package_copy"] = (sha256_16(inside) == out["entry"]["sha256"])

    inproc = sys.modules.get("nmpc")
    if inproc is None:
        try:
            import nmpc as inproc                     # noqa: PLC0415  —— 就是要它落到 src
        except ImportError:
            inproc = None
    if inproc is not None:
        out["driver_view"] = str(Path(inproc.__file__).resolve().parent)
        out["polluted"] = Path(out["driver_view"]) != pkg_dir
        if out["polluted"]:
            out["note"] = (f"驱动进程 `import nmpc` 落在 {out['driver_view']}，"
                           f"节点落在 {pkg_dir} ⟹ **以节点那份为准**（`sys.path.insert` "
                           "传不进子进程）。本字段是这台仪器的自检读数，不是异常。")
    return out


#: A 档 run log 里**PX4 自己**回读参数的那一行（tier A 的 plant check 打的）：
#: `[verify] param show SIH_MASS ⟶ pxh> param show SIH_MASS | x   SIH_MASS [843,1330] : 0.7500`
#: ★ 注意方括号里还有两个数字（`[843,1330]` = 参数索引），所以不能简单取最后一个数。
PARAM_LINE_RE = re.compile(r"param show\s+(?P<name>[A-Z][A-Z0-9_]*)\s+⟶.{0,80}?:\s*"
                           r"(?P<val>[-+]?\d+(?:\.\d+)?(?:[eE][-+]?\d+)?)\s*$")


def readback_from_log(log_path: Path) -> dict:
    """从 A 档的 run log 里捞**PX4 自己回读**的那 10 个参数。

    ★ 这条证据比"仪器在场时读的那一份"还硬一点点：这是 PX4 的 `param show` **输出文本**，
      是被控对象**自己报的**，不是我们请求的值。
    ★ 为什么要它：2026-09-29 10:15 之后起的 8 场因为仪表缺陷（`controller_code()` 抛异常，
      在写 `obs["sweep"]` **之前**就死了）没有落盘读回；这 8 场的覆盖层证据就靠这个函数
      从 run log 里**恢复**出来（见 `recover_sweep`），而不是从文档里手抄。
    """
    want = set(readback_names())
    out: dict = {}
    for line in Path(log_path).read_text(encoding="utf-8", errors="replace").splitlines():
        m = PARAM_LINE_RE.search(line)
        if m and m.group("name") in want:
            out.setdefault(m.group("name"), float(m.group("val")))
    return out


def recover_sweep(outdir: Path, dry_run: bool = False) -> list[dict]:
    """给**缺 `obs["sweep"]`** 的场次补出那一块（读回从 run log 恢复，其余按现况重算）。

    ★ 只补不覆盖：已经有 `sweep` 的场次**一个字节都不动** —— 那是仪表在场时的第一手落盘。
    ★ 补出来的那一块带 `recovered` 字段（时间、来源、为什么），并**逐条说明哪几个字段是
      "重算/重推"而非当场捕获**（`tier_a_rc`、`loadavg` 这类当场读数恢复不出来，就**留空**，
      不拿"大概是多少"去填）。
    """
    probe = controller_code()
    rows = []
    for path in sorted(Path(outdir).glob("obs-*.json")):
        obs = json.loads(path.read_text(encoding="utf-8"))
        if isinstance(obs.get("sweep"), dict):
            rows.append({"file": path.name, "changed": False, "why": "已有 `sweep` ⟹ 不动"})
            continue
        m = re.match(r"^obs-(?P<v>.+)-(?P<r>\d+)\.json$", path.name)
        if not m:
            rows.append({"file": path.name, "changed": False, "why": "文件名对不上，跳过"})
            continue
        variant, rep = m.group("v"), int(m.group("r"))
        log = Path(outdir) / f"run-{variant}-{rep}.log"
        if not log.exists():
            rows.append({"file": path.name, "changed": False,
                         "why": f"找不到 {log.name} ⟹ 没有可恢复的读回"})
            continue
        readback = readback_from_log(log)
        if len(readback) < len(readback_names()):
            rows.append({"file": path.name, "changed": False,
                         "why": f"{log.name} 里只读出 {len(readback)}/"
                                f"{len(readback_names())} 个参数 ⟹ 证据不全，**不补**"})
            continue
        ok, detail = overlay_verdict(variant, readback)
        overlay = overlay_path(variant)
        rows.append({"file": path.name, "changed": True,
                     "why": f"从 {log.name} 恢复 {len(readback)} 个参数、覆盖层 "
                            f"{'✅' if ok else '❌'}"})
        if dry_run:
            continue
        obs["sweep"] = {
            "variant": variant, "rep": rep, "overlay": overlay.name,
            "overlay_sha256": sha256_16(overlay),
            "readback": readback, "overlay_check": {"ok": ok, "detail": detail},
            "hover_thrust": plant_hover_thrust(),
            "hover_thrust_node_default": _node_default_hover_thrust(),
            "hover_thrust_plant": plant_hover_thrust(),
            "controller_code": probe,
            "recovered": {
                "at": time.strftime("%Y-%m-%d %H:%M:%S"),
                "from": log.name,
                "why": "仪表缺陷（`controller_code()` 抛 `AttributeError`，发生在写 `sweep` 之前）"
                       "⟹ 本场没落盘 `sweep`；此处从 run log 恢复",
                "recomputed": ["hover_thrust / hover_thrust_node_default（当场值未落盘，"
                               "按口径重算 —— 驱动这一轮一律 `--hover-thrust auto`）"],
                "unavailable": ["tier_a_rc", "loadavg", "nproc", "tier_a_argv"],
            },
        }
        path.write_text(json.dumps(obs, ensure_ascii=False, indent=1), encoding="utf-8")
    return rows


# ---------------------------------------------------------------- 逐帧轨迹（判据③的"可重判"原料）
TRACE_HZ = 5.0          # 降采样到多少 Hz 入库（原始 ~100 Hz）
TRACE_JZ = 2            # `x0[2]` 就是 EKF 帧的高度（判据③判的那一维）


def trace_from_jsonl(jsonl_path: Path, hz: float = TRACE_HZ) -> dict:
    """把节点的逐帧流降采样成**可入库的小轨迹**，让判据③的窗口将来能被重判。

    ★ 为什么需要它（2026-09-29，由并行会话 `ros-ws-8b` 的教训回推）：obs 里落的是
      **聚合量**（`dut.track`：`up_tail` / `up_tail_min/max/std` / `tail_s`），
      它**绑死了"末段 3 s"这个窗口口径**。窗口口径一旦要改（换帧、换末段长度、
      均值换中位数），聚合量**重算不出来** ⟹ 只能 13 场**整场重飞**。
      而判据③恰好是本页结论的**承重行**（`k` 与"稳到哪儿"全从它来）。
    ★ 原料在**原始流**里（每帧都有 `x0`，节点自己那份 EKF 帧状态），但原始流留 `/tmp`（一场 3.3 MB）。
      ⟹ 这里只把**重判需要的那几列**降采样入库（~8 KB/场，比聚合量贵不了多少），
      原始流仍留 `/tmp` —— **不是**把整条流搬进仓库。
    ★ 记录源文件的 sha256：原始流会随重启消失，这个 hash 是"轨迹确实来自那一份流"的凭据。
    """
    rows: list[dict] = []
    n_total = 0
    src_sha = sha256_16(jsonl_path)
    step: float | None = None
    t0_us: float | None = None
    last_kept = None
    for line in Path(jsonl_path).open(encoding="utf-8", errors="replace"):
        line = line.strip()
        if not line:
            continue
        try:
            r = json.loads(line)
        except json.JSONDecodeError:
            continue
        x0 = r.get("x0")
        ts = r.get("ts")
        if not isinstance(x0, list) or len(x0) <= TRACE_JZ or not isinstance(ts, (int, float)):
            continue
        n_total += 1
        if t0_us is None:
            t0_us = float(ts)
            step = 1.0 / hz if hz > 0 else None   # ★ `t_rel` 是**秒**（`ts` 是微秒，下面除过）
            last_kept = -float("inf")
        t_rel = (float(ts) - t0_us) / 1e6
        if step is not None and (t_rel - last_kept) < step:
            continue           # 降采样：每 1/hz 秒留一点
        last_kept = t_rel
        u = r.get("u") or []
        rows.append({
            "t": round(t_rel, 3),
            "x0_z": round(float(x0[TRACE_JZ]), 4),
            "x0_x": round(float(x0[0]), 4),
            "x0_y": round(float(x0[1]), 4),
            "u0": round(float(u[0]), 4) if len(u) > 0 else None,
            "tbz": (round(float(r["thrust_body_z"]), 4)
                    if isinstance(r.get("thrust_body_z"), (int, float)) else None),
            "fb": r.get("fallback"),
        })
    return {
        "source": str(jsonl_path),
        "source_sha256": src_sha,
        "hz": hz,
        "n_raw": n_total,
        "n_kept": len(rows),
        "t_span_s": rows[-1]["t"] if rows else None,
        "cols": ["t", "x0_x", "x0_y", "x0_z", "u0", "tbz", "fb"],
        "how": "从原始流逐帧 `x0` 降采样；`x0_z` 就是判据③判的那一维（EKF 帧）。"
               "★ 抬升量 = `x0_z − x0_z[0]` 的窗口统计 —— **窗口由重判的人自己定**，"
               "本块不预设末段长度（预设了就等于把口径又焊死一次）。",
        "rows": rows,
    }


def attach_trace(outdir: Path, logdir: Path | None = None,
                 hz: float = TRACE_HZ, dry_run: bool = False) -> list[dict]:
    """给每一场 obs 挂上降采样轨迹（**只补不覆盖**，已有 `trace` 的一字节不动）。

    原始流路径**优先从 obs 自己的 `sweep.tier_a_argv` 里读**（那是当场记下来的真路径），
    读不到才退回 `<logdir>/logs/<变体>-<rep>/hover-1m-px4.jsonl`。
    """
    paths = sorted(Path(outdir).glob("obs-*.json"))
    rows = []
    for path in paths:
        obs = json.loads(path.read_text(encoding="utf-8"))
        if isinstance(obs.get("trace"), dict):
            rows.append({"file": path.name, "changed": False, "why": "已有 `trace` ⟹ 不动"})
            continue
        m = re.match(r"^obs-(?P<v>.+)-(?P<r>\d+)\.json$", path.name)
        if not m:
            rows.append({"file": path.name, "changed": False, "why": "文件名对不上，跳过"})
            continue
        variant, rep = m.group("v"), int(m.group("r"))
        # ① obs 自己记的 tier A 原始流目录（最靠谱）
        cands: list[Path] = []
        argv = ((obs.get("sweep") or {}).get("tier_a_argv")) or []
        for i, a in enumerate(argv):
            if a == "--outdir" and i + 1 < len(argv):
                cands.append(Path(argv[i + 1]) / "hover-1m-px4.jsonl")
        # ② 退回 --logdir 约定
        if logdir is not None:
            cands.append(Path(logdir) / "logs" / f"{variant}-{rep}" / "hover-1m-px4.jsonl")
        jsonl = next((c for c in cands if c.exists()), None)
        if jsonl is None:
            rows.append({"file": path.name, "changed": False,
                         "why": "原始流不在（/tmp 被清或换机）⟹ 补不了；"
                                "这一场判据③若要换窗口重判，**得重飞**"})
            continue
        tr = trace_from_jsonl(jsonl, hz=hz)
        if tr["n_kept"] == 0:
            rows.append({"file": path.name, "changed": False,
                         "why": f"{jsonl} 里没有带 `x0` 的帧 ⟹ 不挂空轨迹"})
            continue
        rows.append({"file": path.name, "changed": True,
                     "why": f"从 {jsonl} 降采样 {tr['n_raw']} → {tr['n_kept']} 点（{hz} Hz）"})
        if dry_run:
            continue
        tr["attached_by"] = "attach_trace"
        obs["trace"] = tr
        path.write_text(json.dumps(obs, ensure_ascii=False, indent=1), encoding="utf-8")
    return rows


def judge_controller_record(cc: dict, probe: dict, obs_mtime: float) -> tuple[bool, str]:
    """旧版仪器写进 `obs` 的 `controller_code` 该不该用**现在的探针**覆盖 —— 只认可复核的情形。

    ★ 为什么会有"旧版仪器写的"这回事：`controller_code()` 的第一版用**本进程**的
      `import nmpc` 判身份，被第 53 行的 `sys.path.insert` 顶到 `src/` 上，
      于是 2026-09-29 10:14 之前落盘的每一场都记着一个**指向 `src/` 的假身份**。
    ★ 为什么不能直接改：改证据文件本身得有据。这里只允许两种情形之一：
      （a）记录的包路径 == 探针的包路径 ⟹ 没什么可改的；
      （b）不一致，**且探针那份的每个文件都早于本场落盘时刻** ⟹ 期间没重建过，
           现在的探针结果对那一场同样成立（`colcon build` 会刷新 mtime，所以这个检查有牙）。
      探针里有任何一个文件比本场新 ⟹ **拒绝改**（宁可留着旧记录并标出来，也不伪造一个更准的身份）。
    """
    pkg = (cc or {}).get("package")
    if not pkg or not probe.get("files"):
        return False, "记录或探针缺一半 ⟹ 不动"
    if Path(pkg) == Path(probe["package"]):
        return False, "两边指向同一份 ⟹ 不动"
    newer = [v["path"] for v in probe["files"].values()
             if Path(v["path"]).stat().st_mtime > obs_mtime]
    if newer:
        return False, (f"探针那份里有 {len(newer)} 个文件**比本场落盘还新**（如 "
                       f"{Path(newer[0]).name}）⟹ 期间多半重建过，不能拿现在的探针去改这一场")
    return True, (f"记录说 `{pkg}`、探针说 `{probe['package']}`，且探针那份**全部早于本场** "
                  "⟹ 判为旧版仪器的**驱动进程视图**（`sys.path.insert` 顶到 `src/`，"
                  "传不进节点子进程），改用探针那份")


def refix_controller_codes(outdir: Path, dry_run: bool = False) -> list[dict]:
    """逐场重探控制器代码身份；改了的场次把**原值**留在 `controller_code_prev` 里备查。

    只动 `obs["sweep"]["controller_code"]` 这一个字段，别的字节不碰（其余读数由仪器当场落盘）。
    """
    probe = controller_code()
    out = []
    for path in sorted(Path(outdir).glob("obs-*.json")):
        obs = json.loads(path.read_text(encoding="utf-8"))
        sw = obs.get("sweep")
        if not isinstance(sw, dict):
            continue
        cc = sw.get("controller_code") or {}
        ok, why = judge_controller_record(cc, probe, path.stat().st_mtime)
        out.append({"file": path.name, "changed": ok, "why": why})
        if not ok or dry_run:
            continue
        sw["controller_code_prev"] = {**cc,
                                      "note": "旧版仪器（进程内 `import nmpc`）写的**驱动视图**，"
                                              "非节点身份 ⟹ 已被 `controller_code` 取代"}
        sw["controller_code"] = {**probe,
                                 "correction": {"at": time.strftime("%Y-%m-%d %H:%M:%S"),
                                                "why": why, "tool": "refix_controller_codes"}}
        path.write_text(json.dumps(obs, ensure_ascii=False, indent=1), encoding="utf-8")
    return out


def _resolve_hover_thrust(text) -> float | None:
    """`auto` / `none` / 数字 → 给节点的 `hover_thrust`（`None` = 不覆盖）。

    ★ 重搭后被控对象口径与节点默认**同档**（`plant_hover_thrust()` ≈ 0.1588、节点 0.1589）
      ⟹ `auto` 与 `none` 实质上等价；保留 `auto` 是为让"跟被控对象同源"这件事显式、
      且在有人改了机型常数时**自动跟随**（这正是 `auto` 该有的行为）。
    """
    s = str(text).strip().lower()
    if s in ("none", "", "null"):     # 驱动往子进程传 `str(ht)`，`None` 会变成字面量
        return None
    if s == "auto":
        return plant_hover_thrust()
    return float(s)


# ---------------------------------------------------------------- 变体表


def variant_order() -> list[str]:
    """票面的 11 个：`z0` + 5 个旋钮 × ±30 %（`z0` 排第一 = 参数一致对照组）。"""
    return ["z0"] + [f"z0-{knob}{tag}{PCT}" for knob in KNOBS for tag in SIGNS]


def knob_of(variant: str) -> str | None:
    """`z0-mass+30` → `mass`；`z0` → `None`（基准没有旋钮）。

    ⚠️ 不认识的变体**抛 `KeyError`**（如 `z0-thrcurve`）：把它当"没有旋钮"静默放过，
      会让一个非扫描项混进扫描表的结论里。
    """
    if variant == "z0":
        return None
    for knob in KNOBS:
        if variant.startswith(f"z0-{knob}"):
            return knob
    raise KeyError(variant)


def knob_param_names(variant: str) -> list[str]:
    """这个变体该动的参数名（基准 = 空表）。"""
    knob = knob_of(variant)
    return [] if knob is None else list(KNOBS[knob]["params"])


def readback_names() -> list[str]:
    """逐场读回哪些参数 = **全部旋钮动过的参数并集**（去重、定序）。

    ★ 为什么不是"只读本变体那一个"：并集能同时验证**两件事** ——
      ① 该动的那个动了；② **其余一个都没动**（"单参数"这个前提本身）。
      只读前者的话，"扫描器顺手把别的也改了"看不出来。
    """
    out = []
    for spec in KNOBS.values():
        for name in spec["params"]:
            if name not in out:
                out.append(name)
    return out


def overlay_path(variant: str) -> Path:
    """覆盖层文件的绝对路径（真源 `plant_spec --emit` 的产物）。"""
    path = HERE / "z0-plant" / "params" / f"{variant}.param"
    if not path.exists():
        raise SystemExit(f"没有这个覆盖层：{path}（`plant_spec.py --emit` 生成）")
    return path


def expected_params(variant: str) -> dict:
    """该变体下每个参数的**期望值** —— 直接问 ticket 08 的验收工具要（它认这个真源）。"""
    import verify_z0_plant  # 延迟导入：只用它这一条，不让本模块多背一个依赖

    return verify_z0_plant.expected_params(variant)


def overlay_verdict(variant: str, readback: dict) -> tuple[bool, str]:
    """逐场核对"这一场真的跑在**这个**失配机型上吗、而且只动了这一个旋钮吗"。

    ★ 为什么这条非有不可：整张票的结论都挂在这个前提上。若覆盖层没生效（跑的还是标称机型），
      扫描会**一路全绿**并给出"±30 % 完全无影响"的结论 —— 一个方向完全相反的错误结论。
      ticket 08 已经证明过"改参数 → 被控对象真的按新参数出力"，但那只在**它那一轮**上；
      本张每一场都得自己带着读数（`obs["sweep"]["readback"]`）。
    """
    import verify_z0_plant as vzp

    want = expected_params(variant)
    names = [n for n in readback_names() if n in want]
    missing = [n for n in names if readback.get(n) is None]
    if missing:
        return False, f"没读到：{', '.join(missing)}（{len(names)} 项里缺 {len(missing)} 项）"
    bad = []
    for name in names:
        tol = vzp.PARAM_TOL_ABS + vzp.PARAM_TOL_REL * abs(want[name])
        if abs(readback[name] - want[name]) > tol:
            bad.append(f"{name} 期望 {want[name]:.6g}、读到 {readback[name]:.6g}")
    if bad:
        return False, "; ".join(bad)
    knob = knob_of(variant)
    return True, (f"{len(names)} 项逐条与 `plant_spec` 一致"
                  + (f"（本场动的是 `{' / '.join(knob_param_names(variant))}` ⟹ 旋钮 {knob}）"
                     if knob else "（基准：没有旋钮要动）"))


# ---------------------------------------------------------------- 判读（纯函数）


def _ta():
    """A 档工具（判据的**唯一**来源）。延迟导入：`--summarize` 也想在没 source ROS 时能跑。"""
    import verify_tier_a_hover

    return verify_tier_a_hover


def divergence(obs) -> tuple[bool, str | None]:
    """本场是否**发散** + 理由（多条用 `；` 连）。纯函数。

    ★ 三条硬线都取**真值帧**或**EKF 帧自己**的累计量，不取"末段均值好看"这类读数：
      ticket 09 第 6 场就是"位置全绿而机头自转 13.6 圈"。
    ★ "节点一帧没发"**不算**发散（`harness_dead`），它是夹具/兜底的读数 ——
      混进发散计数会让"±30 % 内没有一条发散"这句话既对又没用。
    """
    why: list[str] = []
    truth = [r for r in (obs.get("truth") or []) if all(v is not None for v in r)]
    if truth:
        zs = [float(r[3]) for r in truth]
        xy = [math.hypot(float(r[1]), float(r[2])) for r in truth]
        if min(zs) < -DIV_Z_M:
            why.append(f"真值高度 z 越过 −{DIV_Z_M:g} m（最低 {min(zs):+.2f} m，NED 向上为负）"
                       f"⟹ 爬飞")
        if max(zs) > DIV_Z_M:
            why.append(f"真值高度 z 越过 +{DIV_Z_M:g} m（最高 {max(zs):+.2f} m）")
        if xy and max(xy) > DIV_XY_M:
            why.append(f"水平飞出 {DIV_XY_M:g} m（最远 {max(xy):.2f} m）")
        ta = _ta()
        seg = ta.segments(truth, hover_s=obs.get("hover_s", 3.0))
        hov, gnd = ta.z_stats(seg["hover"]), ta.z_stats(seg["ground"])
        if hov.get("mean") is not None and gnd.get("mean") is not None:
            lift = gnd["mean"] - hov["mean"]
            if lift < DIV_HOLD_M:
                why.append(f"末段真值抬升只有 {lift:+.3f} m（< {DIV_HOLD_M:g} m ⟹ 末段不在空中）")
    yaw = ((obs.get("dut") or {}).get("track") or {}).get("yaw_ekf_span")
    if yaw is not None and abs(yaw) > DIV_YAW_RAD:
        why.append(f"EKF 帧机头累计转过 {yaw:+.2f} rad（> 一圈 ⟹ 自转）")
    return (bool(why), "；".join(why) if why else None)


def extract(obs) -> dict:
    """一场观测 → 本张要报的读数（纯函数）。缺什么给 `None`，**不抛**。

    ★ "缺键给 None"是刻意的：夹具坏掉的时候要能说"这一项没测到"，
      不能因为一个键没取到就把整张表带崩（`test_extract_读的是同一批键_...` 钉住这条）。
    """
    dut = obs.get("dut") or {}
    tr = dut.get("track") or {}
    solve = obs.get("solve") or {}
    checks = [c for c in (obs.get("checks") or []) if c.get("counts", True)]
    failed = [c["name"] for c in checks if c.get("ok") is False]
    div, why = divergence(obs)
    ta = _ta()
    seg = ta.segments([r for r in (obs.get("truth") or []) if all(v is not None for v in r)],
                      hover_s=obs.get("hover_s", 3.0))
    hov, gnd = ta.z_stats(seg["hover"]), ta.z_stats(seg["ground"])
    climb_gt = (None if (hov.get("mean") is None or gnd.get("mean") is None)
                else gnd["mean"] - hov["mean"])
    climb_ekf = tr.get("climb")
    try:
        attribution = ta.attribute(obs, obs.get("checks") or [])
    except Exception as e:  # noqa: BLE001 —— 归因不可用不该吃掉一整场
        attribution = f"（归因不可用：{type(e).__name__}: {e}）"
    return {
        "passed": not failed,
        "failed_checks": failed,
        "harness_dead": (dut.get("frames") or 0) == 0,
        "diverged": div,
        "diverge_why": why,
        "attribution": attribution,
        # —— 判在**状态源那一帧**（EKF）的读数：票面验收线就在这一帧上（操作者裁定）——
        "climb_ekf": climb_ekf,
        "e_z_ekf": (None if climb_ekf is None else climb_ekf - HOLD_Z_M),
        "p2p_ekf": tr.get("up_tail_p2p"),
        "drift_ekf": tr.get("xy_drift_tail"),
        "referr_tail": tr.get("referr_tail"),
        "yaw_span_ekf": tr.get("yaw_ekf_span"),
        "u3_tail": tr.get("u3_tail"),
        # —— 真值帧（**被控对象**的读数，只当背景：这台 SITL 的估计器自带 0.4 m 量级误差）
        "climb_gt": climb_gt,
        # 末段 EKF 高度（**绝对**，含 EKF 起机零点）—— 不进 `METRIC_KEYS`（不参与排序）。
        # ☠️ **它不是帧差、也不能与 `climb_gt` 相减**：`climb_gt` 是**真值相对抬升**
        #   （地面为零点），两者 datum 不同 ⟹ 差里主要是起机零点（本轮 +0.65…+0.79 m）。
        #   要量两帧的分歧请用 `climb_ekf − climb_gt`（**同口径**，都是相对抬升）。
        #   （2026-09-29 自查抓到：这里原先写着"与 `climb_gt` 同口径"，与前半句自相矛盾。）
        "up_tail": tr.get("up_tail"),
        "p2p_gt": hov.get("p2p"),
        # —— 求解耗时（D6 那一列）
        "solve_p50": solve.get("p50"),
        "solve_p99": solve.get("p99"),
        "solve_max": solve.get("max"),
        "solve_steady_max": solve.get("steady_max"),
        "ladder_steady": solve.get("ladder_steady"),
        "frames": dut.get("frames"),
        "actual_hz": dut.get("actual_hz"),
    }


# ------------------------------------------------- 真值离线重取（只读控制台归档，不占 SITL 窗口）
#
# ★ 为什么需要它：归档的 `obs["truth"]` 是**抽稀**过的一路（A 档 `TruthSampler` 每拍只留到几帧），
#   而 A 档控制台**全文**里真值块密得多（本批实测 ~5.1 Hz / 末段 22 帧，vs obs 的 5–11 帧）。
#   有全文就能**离线**答"某个真值读数的散度是采样太稀、还是真的散"，不必再占一次 SITL 窗口。
#
# ☠️ 它**替代不了实跑**，而且只能**证伪**：重取值与归档值**相等不认证干净**（一个坏窗口
#   完全可能给出同一个值）。写成结论只能说"重取后差值 ≤ N m"，不能说"检查过了、没问题"。

GT_TOPIC = "TOPIC: vehicle_local_position_groundtruth"

#: **同一个字符类纪律**（`verify_offboard_smoke.field()` 2026-09-29 那次崩点）：**至少要有一位数字**。
#: 控制台是**按块**渲染的，块边界可以落在数值中间 —— 旧写法（`-?[0-9.eE+-]+`）会把半截的 `-`
#: 当成一个数 ⟹ `float('-')` 抛 ValueError ⟹ **整场作废**（加密档第一场就是这么死的）。
_GT_NUM = r"-?(?:\d+\.?\d*|\.\d+)(?:[eE][+-]?\d+)?"


def parse_console_gt(text) -> list:
    """控制台全文 → `[[t_s, x, y, z], …]`（按 px4 时间戳排序）。

    **半截/非数值的那一整块丢掉**，不抛 —— 这是取数纪律不是宽容：丢一条只少一帧，
    抛出去会让整场作废（2026-09-29 加密档第一场就是这么死的）。

    ★ 取值一律**行锚定**（`[ \\t\\r]*$`）：`z: 1.0e` / `z: -0.0140-` / `z: -` 三样都判不合格、
      整块丢掉。**这比 `verify_offboard_smoke.field()` 严一档** —— 那个只要求"至少一位数字"，
      于是 `z: 1.0e` 会被**静默截短**成 `1.0`（§附.8 明记那条分支**没关掉**）。
      这里没有那个分支，**但只关掉了"行尾带杂质"这一种**：
      若上游渲染出来的是一条**看着完整、其实被截短**的行（`z: -0.0140` 而真值是 `-0.01408`），
      本函数**看不出来**，也无法看出来。
    """
    out = []
    for blk in (text or "").split(GT_TOPIC)[1:]:
        ts = re.search(r"^\s*timestamp:\s*(\d+)", blk, re.M)
        cols = [re.search(rf"^\s*{k}:\s*({_GT_NUM})[ \t\r]*$", blk, re.M) for k in ("x", "y", "z")]
        if not ts or not all(cols):
            continue
        out.append([int(ts.group(1)) / 1e6] + [float(c.group(1)) for c in cols])
    out.sort(key=lambda r: r[0])
    return out


def match_flight(obs_zs, snaps, tol=0.002) -> tuple:
    """`obs["truth"]` 的 z 序列 → `(快照名, 命中率, 快照序列)`，取命中率最高的那个。"""
    best = (0.0, None)
    for name, gt in snaps.items():
        if not gt:
            continue
        pool = sorted(r[3] for r in gt)
        hit = 0
        for z in obs_zs:
            i = bisect.bisect_left(pool, z)
            for j in (i, i - 1):
                if 0 <= j < len(pool) and abs(pool[j] - z) <= tol:
                    hit += 1
                    break
        rate = hit / len(obs_zs) if obs_zs else 0.0
        if rate > best[0]:
            best = (rate, name)
    return (best[1], best[0], snaps.get(best[1]))


def slice_between(gt, z_first, z_last, tol=0.002) -> list:
    """把控制台序列切成与 obs **同跨度**（首个/末个 z 各配一次）⟹ 窗口定义一致，可比。"""
    i0 = next((i for i, r in enumerate(gt) if abs(r[3] - z_first) <= tol), None)
    i1 = next((i for i in range(len(gt) - 1, -1, -1) if abs(gt[i][3] - z_last) <= tol), None)
    if i0 is None or i1 is None or i1 < i0:
        return []
    return gt[i0:i1 + 1]


def truth_lift(rows, hover_s=3.0) -> dict:
    """`[[t,x,y,z], …]` → 地面/悬停段的 n 与均值、抬升 —— **用 A 档自己的**窗口函数，不另写一套。"""
    ta = _ta()
    seg = ta.segments([[r[0], r[1], r[2], r[3], 0.0, 0.0, 0.0] for r in rows], hover_s=hover_s)
    hov, gnd = ta.z_stats(seg["hover"]), ta.z_stats(seg["ground"])
    lift = (None if hov.get("mean") is None or gnd.get("mean") is None
            else gnd["mean"] - hov["mean"])
    return {"gnd_n": gnd.get("n"), "gnd_mean": gnd.get("mean"),
            "hov_n": hov.get("n"), "hov_mean": hov.get("mean"), "lift": lift}


def load_console_snaps(console_dir) -> dict:
    """`<dir>/*.log` → `{文件名: [[t,x,y,z], …]}`（**只读**；空文件也留着，让调用方能报出来）。"""
    return {p.name: parse_console_gt(p.read_text(errors="replace"))
            for p in sorted(Path(console_dir).glob("*.log"))}


def reread_truth(outdir, snaps) -> list:
    """逐场：`obs["truth"]`（抽稀）vs 控制台全文重取（全速）的同一读数对照。

    变体名从文件名 `obs-<variant>-<rep>.json` 取（`obs` 里没有变体字段）。
    """
    rows = []
    for f in sorted(Path(outdir).glob("obs-*.json")):
        obs = json.loads(f.read_text())
        truth = [r for r in (obs.get("truth") or []) if all(v is not None for v in r)]
        if not truth:
            continue
        m = re.match(r"obs-(.+)-(\d+)\.json$", f.name)
        slim = [[r[0], r[1], r[2], r[3]] for r in truth]
        name, hit, gt = match_flight([r[3] for r in slim], snaps)
        sub = slice_between(gt or [], slim[0][3], slim[-1][3])
        hover_s = obs.get("hover_s", 3.0)
        rows.append({
            "file": f.name, "variant": (m.group(1) if m else f.stem),
            "rep": (int(m.group(2)) if m else 0),
            "snap": name, "hit": hit, "snap_n": 0 if not gt else len(gt),
            "obs_hz": (len(slim) / (slim[-1][0] - slim[0][0])) if slim[-1][0] > slim[0][0] else None,
            "obs": truth_lift(slim, hover_s),
            "full": truth_lift(sub, hover_s) if sub else None,
            "sub_n": len(sub),
            "full_hz": ((len(sub) - 1) / (sub[-1][0] - sub[0][0])
                        if len(sub) > 1 and sub[-1][0] > sub[0][0] else None),
        })
    return rows


def render_reread(rows) -> str:
    """上面那批 → markdown 表 + 极值 + 同参数重复对。**不判通过与否**（它只报读数）。"""
    out = ["| 场次 | 快照 | 命中率 | obs：Hz / 末段 n / 抬升 | 全速：Hz / 末段 n / 抬升 | Δ抬升 |",
           "|---|---|---|---|---|---|"]
    deltas = []
    for r in rows:
        o, g = r["obs"], r["full"]
        def cell(d):
            if d is None or d["lift"] is None:
                return f"{'—' if d is None else round(d['hov_n'] or 0)} / **算不出**"
            return f"{_f(d['hov_n'], '.0f')} / {_f(d['lift'])}"
        hz = "—" if not r["obs_hz"] else f"{r['obs_hz']:.2f}"
        fhz = "—" if not r["full_hz"] else f"{r['full_hz']:.2f}"
        d = (None if (o["lift"] is None or (g or {}).get("lift") is None)
             else g["lift"] - o["lift"])
        if d is not None:
            deltas.append(abs(d))
        out.append(f"| `{r['variant']}` rep{r['rep']} | {r['snap']} | {r['hit']:.0%} | "
                   f"{hz} / {cell(o)} | {fhz} / {cell(g)} | {'—' if d is None else _f(d)} |")
    if deltas:
        out.append(f"\n**重取与归档的最大差：{max(deltas):.3f} m**（{len(deltas)} 场两版都算得出来）")
    pairs = {}
    for r in rows:
        pairs.setdefault(r["variant"], []).append(r)
    for v, rs in pairs.items():
        full = [r["full"]["hov_mean"] for r in rs if r["full"] and r["full"]["hov_mean"] is not None]
        if len(full) >= 2:
            out.append(f"- `{v}` 同参数 {len(full)} 场：全速悬停平台 "
                       f"{_f(min(full))} … {_f(max(full))}（**差 {max(full) - min(full):.3f} m**）")
    return "\n".join(out)


# ---------------------------------------------------------------- 汇总（纯函数）


def _stat(values) -> dict | None:
    """一组读数的中位数/极值/跨度（`None` 一律剔掉）。"""
    xs = sorted(float(v) for v in values if v is not None)
    if not xs:
        return None
    return {"n": len(xs), "median": statistics.median(xs), "min": xs[0], "max": xs[-1],
            "span": xs[-1] - xs[0]}


def summarise(runs: list[dict]) -> dict:
    """运行记录 → 逐变体的表 + **基准散度**（纯函数）。

    ★ 为什么基准要单独算一份：票面要的是"**干净对比**"。失配那几场与基准的差，
      只有放进"基准自己重复几次散多大"这个尺子里才有意义 ——
      否则 `±0.01 m` 的差会被读成"失配的影响"，而它可能只是这台仪器的重复性。
    """
    by_variant: dict[str, list[dict]] = {}
    for r in runs:
        e = extract(r["obs"])
        e["variant"] = r["variant"]
        e["rep"] = r.get("rep", 1)
        e["knob"] = knob_of(r["variant"])
        e["overlay_ok"] = ((r["obs"].get("sweep") or {}).get("overlay_check") or {}).get("ok")
        e["status"] = ("发散" if e["diverged"]
                       else "退化" if not e["passed"] else "稳定")
        by_variant.setdefault(r["variant"], []).append(e)

    base_rows = by_variant.get("z0", [])
    baseline = {"n": len(base_rows),
                # ⚠️ 没有基准场次时（`--variants z0-mass+30` 这种单变体跑法）要说出来：
                #    "与基准的差"整列会是 None，而这**不是**"没有差别"。
                "present": bool(base_rows),
                "warn": (None if len(base_rows) > 1 else
                         ("**这次没有跑基准（`z0`）** ⟹ 整张表没有对照，"
                          "「±30 % 是否有影响」这个问题**答不了**"
                          if not base_rows else
                          "基准只有 1 场 ⟹ 「与基准的差」里分不清失配与重复性；"
                          "判边界时以**票面验收线**为准，别引这个差"))}
    for key in METRIC_KEYS:
        baseline[key] = _stat([row[key] for row in base_rows])

    for variant, rows in by_variant.items():
        for row in rows:
            row["delta"] = {}
            for key in METRIC_KEYS:
                b, v = baseline.get(key), row.get(key)
                row["delta"][key] = (None if (b is None or v is None) else v - b["median"])

    counts = {"发散": 0, "退化": 0, "稳定": 0}
    for rows in by_variant.values():
        for row in rows:
            counts[row["status"]] += 1

    # ★ 控制器代码留档：一轮扫描里出现**两份**不同的代码 ⟹ 这不是一个实验。
    #   签名 = **入口脚本** + 包内全部 `.py` 的哈希（入口脚本由 `ros2 run` 直接 exec，
    #   而 import 的模块来自 dist-packages，两者是两份文件 ⟹ 都要进签名）。
    sigs: dict[str, list[str]] = {}
    pkgs: dict[str, set] = {}
    for r in runs:
        cc = (r["obs"].get("sweep") or {}).get("controller_code") or {}
        files = cc.get("files")
        if not files:
            sig = "（未留档）"
        else:
            blob = "|".join([f"{k}:{v.get('sha256')}" for k, v in sorted(files.items())]
                            + [f"entry:{(cc.get('entry') or {}).get('sha256')}"])
            sig = hashlib.sha256(blob.encode()).hexdigest()[:12]
        sigs.setdefault(sig, []).append(f"{r['variant']}-{r.get('rep', 1)}")
        pkgs.setdefault(sig, set()).add(cc.get("package") or "（路径未记）")
    controller = {"n_sigs": len(sigs), "sigs": sigs,
                  "packages": {k: sorted(v) for k, v in pkgs.items()},
                  "warn": (None if len(sigs) <= 1 else
                           f"★ **本轮出现了 {len(sigs)} 份不同的控制器代码** ⟹ 这 {len(runs)} 场"
                           f"**不是同一个实验**：{ {k: v for k, v in sigs.items()} }。"
                           "多半是 `src/` 被人改过之后再 `colcon build`（`ros2 run nmpc` 跑的是"
                           "**装出来**的那一份，不是 `src/`）⟹ 重跑，或把一趟固定在一份构建上")}
    # ★ **扫描器自己的身份**：`controller_code` 管的是"被测对象"，这里管"量它的尺子"。
    #   没有这一块时，"两批之间改过工具"只能靠口头凭据 —— 而这正是本轮发生过的事。
    hsig: dict[str, list[str]] = {}
    for r in runs:
        h = (r["obs"].get("sweep") or {}).get("harness") or {}
        hsig.setdefault(h.get("sha256") or "（未留档）", []).append(
            f"{r['variant']}-{r.get('rep', 1)}")
    if list(hsig) == ["（未留档）"]:
        h_warn = ("★ **本轮没有留扫描器自己的身份**（`harness` 字段是之后才加的）⟹ "
                  "**归档里查不出「前后两批是不是同一把尺子」**。"
                  "作者当时的口供是「改动落在两批之间、只往 `run_variant` 里加了 `trace` 的"
                  "自动落盘」，但**这条口供本身不可复核**（事后 `--attach-trace` 又把前三场的 "
                  "`trace` 补齐，连字段差别都抹平了）⟹ 引用本轮那 13 场时**按「可能换过尺子」处理**："
                  "需要严格同尺的结论，用提交后的同一份代码重跑，别拿这两批硬凑。")
    elif len(hsig) > 1:
        h_warn = (f"★ **一轮里出现了 {len(hsig)} 份不同的扫描器代码** ⟹ 这 {len(runs)} 场"
                  f"**不是同一个实验**（量尺被改过）：{dict(hsig)}")
    else:
        h_warn = None
    harness = {"n_sigs": len(hsig), "sigs": hsig, "warn": h_warn}

    # ★ 逐帧轨迹的覆盖率：**聚合量重算不回来**，所以"哪几场将来能换窗口重判"必须写在表里，
    #   不能让读表的人以为"表在 ⟹ 什么都能重算"。
    trace_hz = sorted({(r["obs"].get("trace") or {}).get("hz") for r in runs
                       if isinstance(r["obs"].get("trace"), dict)
                       and (r["obs"].get("trace") or {}).get("hz") is not None})
    by_how: dict[str, int] = {}
    for r in runs:
        t = r["obs"].get("trace")
        if isinstance(t, dict):
            by_how[t.get("attached_by") or "（未记）"] = by_how.get(
                t.get("attached_by") or "（未记）", 0) + 1
    trace = {"n": sum(by_how.values()), "n_runs": len(runs), "by": by_how,
             "hz": trace_hz[0] if len(trace_hz) == 1 else (trace_hz or None)}
    return {"n_runs": len(runs), "baseline": baseline, "by_variant": by_variant,
            "counts": counts, "variants": list(by_variant), "controller": controller,
            "harness": harness, "trace": trace}


def _f(value, fmt="+.3f") -> str:
    return "—" if value is None else format(float(value), fmt)


#: "谁先退化"排序里用的读数。★ **刻意不含求解耗时**（`solve_*`）：
#: 它对**机器负载**敏感（同一台机器空机 3.995 ms / 机上跑 pytest 时 11.297 ms，
#: ticket 09 §5.1），进排序的话"扫描时机器忙了一下"会被读成"这个参数让求解变慢了" ——
#: 两个方向完全不同的结论。它照旧印在表里（D6 那一列），只是不参与排序。
#: 参与排序的读数 —— **从 `METRIC_KEYS` 派生**，不另抄一份。
#: ★ 为什么派生：这两张表曾经各写一份，只差两个 `solve_*`；将来往 `METRIC_KEYS` 加一条读数、
#:   忘了同步这里，报告会**静默地少排一条**（不报错，只是那一列看不见）。派生让这种漏改不可能。
#: ★ 为什么排除 `solve_*`：求解耗时**随机器负载变**，不是被控对象的性质 ——
#:   拿它排"谁先退化"会把"当时谁在抢 CPU"读成"哪个参数更敏感"。它照旧进明细表，只是不参与排序。
RANK_KEYS = tuple(k for k in METRIC_KEYS if not k.startswith("solve"))


def rank_frame(key: str | None) -> str | None:
    """读数属于**哪一帧** —— `_ekf` / `_gt` 不是后缀装饰，是**两台不同的仪器**。

    ★ 为什么必须区分（2026-09-29 本轮实测撞到）：A 档判在 **EKF 帧**（操作者裁定），
      而 `climb_gt` 来自 SIH **真值**帧。两者之间夹着这台 SITL 自己的估计器误差，
      它的量级**会随场次整块漂**、且**跨零**（具体范围见告警文案，那里是**现算的**）。
      ⟹ 用"离开基准散度带的倍数"排序时，**真值帧那几条读数的倍数不能与 EKF 帧的并排比**：
      它们的散度带是**两台仪器各自的**重复性，不是同一个尺子上的刻度。
    """
    if key is None:
        return None
    if key.endswith("_ekf"):
        return "ekf"
    if key.endswith("_gt"):
        return "truth"
    return None


def rank_degradation(s: dict) -> dict:
    """按"**读数离开基准散度带的倍数**"给旋钮排序 —— 票面那句"从哪个参数先退化"的答案。

    ★ 为什么不能只靠 `status` 列（发散/退化/稳定）：那一列是一根**固定的绝对线**
      （如"1 m ± 0.15"）。**两个都还过线的档之间**，票面这句话就没有答案了 ——
      而"哪个参数先开始变差"恰恰是问在这些档之间的。所以尺子换成**基准自己重复几场的散度带**：

          x = |读数 − 基准中位数| ÷ (基准跨度 / 2)

      倍数无量纲 ⟹ 米、弧度、秒可以并排比。每个旋钮取其**最大的那个倍数**（最差读数），
      并把"是哪一条读数、哪一场把它顶上去的"一起回指出来。

    ★ 两类**不给倍数**的：
      · **发散**的档 —— 它不是"更严重的退化"，是另一个类别（拿一个已经飞天的高度差
        去跟"抖了一点"比大小没有意义）⟹ 单列在最前，`x = None`；
      · **夹具没跑起来**的档（节点 0 帧）⟹ 本档根本没读数。
      · 基准散度带为 0（重复场读数一模一样）或基准只有 1 场 ⟹ **整页拒绝给倍数**：
        那种情况下任何差都是"无穷倍"，印出来会让人以为"这个参数一碰就散"。
    """
    base = s["baseline"]
    #: 每个读数的散度带半宽；`None` = 这条读数**不可分辨**（基准重复不够 / 跨度为 0）
    half = {k: (None if (b is None or b["n"] < 2 or b["span"] <= 0) else b["span"] / 2.0)
            for k in RANK_KEYS for b in (base.get(k),)}
    resolvable = any(h is not None for h in half.values())

    rows = []
    for variant, vruns in s["by_variant"].items():
        knob = knob_of(variant)
        if knob is None:
            continue                                    # 基准自己不进排序
        if any(r["diverged"] for r in vruns):
            rows.append({"knob": knob, "x": None, "flag": "发散（不参与倍数）",
                         "variant": variant, "rep": None, "key": None,
                         "value": None, "base_median": None})
            continue
        if all(r["harness_dead"] for r in vruns):
            rows.append({"knob": knob, "x": None, "flag": "夹具没跑起来（节点 0 帧）⟹ 本档无读数",
                         "variant": variant, "rep": None, "key": None,
                         "value": None, "base_median": None})
            continue
        best = {"knob": knob, "x": None, "flag": "不可分辨（基准散度带为 0 或只有 1 场）",
                "variant": variant, "rep": None, "key": None,
                "value": None, "base_median": None}
        for r in vruns:
            for key in RANK_KEYS:
                h, v = half.get(key), r.get(key)
                if h is None or v is None:
                    continue
                x = abs(v - base[key]["median"]) / h
                if best["x"] is None or x > best["x"]:
                    best.update({"x": x, "key": key, "rep": r["rep"], "value": v,
                                 "base_median": base[key]["median"], "flag": None})
        rows.append(best)

    rows.sort(key=lambda r: (0 if str(r["flag"] or "").startswith("发散") else
                             1 if r["x"] is not None else 2,
                             -(r["x"] if r["x"] is not None else 0.0)))

    if not base["present"]:
        note = ("**这次没有跑基准（`z0`）** ⟹ 「谁先退化」**答不了**："
                "没有散度带就没有尺子，只能看每档的绝对读数")
    elif not resolvable:
        note = ("基准的散度带是 0（或只有 1 场）⟹ 这台仪器分辨不出 ±30 % 的差，"
                "**不按倍数排序**；判边界只能以票面验收线为准")
    else:
        note = None

    # ★ 跨帧告警：排序按"每条读数各自的倍数"取赢家 ⟹ **一张表里可能混着两台仪器**。
    #   本轮实测就是这么发生的：`inertia` / `tau` / `rate` 的倍数全来自 `climb_gt`（真值帧），
    #   而 `mass` / `thrust` 的来自 `climb_ekf`（EKF 帧）—— 两条读数的"散度带"是两台仪器
    #   各自的重复性，**不是同一把尺子上的刻度** ⟹ 这两组的倍数**不能并排读**，必须点名。
    #
    # ★★ 为什么这段散文里的数要**现算**、不许写死：2026-09-29 本轮就栽在这上面 ——
    #   文案里的范围是从**已丢的第一批**抄来的，13 场归档之后**没有人再回算过**，
    #   于是工具印的、文档引的、票面写的三处数字**互不一致**，而它们看上去都"像真的"。
    #   索性改成从手上这批 run 现算：换了场次它自己会变，**再也不会与数据脱钩**。
    frames = {rank_frame(r.get("key")) for r in rows if r.get("x") is not None}
    frames.discard(None)
    if len(frames) > 1 and note is None:
        # ☠️ 2026-09-29 自查抓到的**第三类缺陷**：这里原本印 `up_tail − climb_gt`，并称之为
        #   "两帧差"。**它不是**：`up_tail` 是 **EKF 末段绝对高度**（含 EKF 起机零点，
        #   本轮 +0.65…+0.79 m），`climb_gt` 是**真值相对抬升**（地面为零点）——
        #   减出来那 0.51…0.96 m 主要是**起机零点**，不是两帧的分歧，而且因此**恒同号**。
        #   同口径的差是 `climb_ekf − climb_gt`（都是"相对抬升、各自地面为零点"），
        #   本轮实测 −0.29…+0.39 m，**并且会换号**。
        #   ⟹ 注释里那句"与 `climb_gt` 同口径"**自相矛盾**（同句前半句说它是绝对量），已改。
        #   教训与本文件 §2.7.1"身份探针在说假话"同类：**一个量看着像帧差，不等于它是帧差**。
        gaps = [r["climb_ekf"] - r["climb_gt"]
                for vr in s["by_variant"].values() for r in vr
                if isinstance(r.get("climb_ekf"), (int, float))
                and isinstance(r.get("climb_gt"), (int, float))]
        if len(gaps) >= 2:
            # ⚠️ "跨零"也**必须现判**：范围全是同一号的时候还说"跨零"就是编的
            #    （这条是被本文件自己的单测抓出来的，不是想出来的）。
            crosses = min(gaps) < 0 < max(gaps)
            gap_txt = (f"（本批实测 **同口径**的 `climb_ekf − climb_gt` 在 "
                       f"{min(gaps):+.2f} … {max(gaps):+.2f} m 之间，{len(gaps)} 场两帧都取得到"
                       + ("，且**跨零**" if crosses else "，**同号**（没跨零）") + "）")
        else:
            gap_txt = "（本批取不到两帧都有的场次 ⟹ 这个差**没量**，别引任何数）"
        note = ("★ **这张排序里混了两帧**：`" + "` / `".join(sorted(frames)) +
                "` 的倍数分别来自不同的仪器 —— A 档判在 **EKF 帧**（操作者裁定），"
                "`climb_gt`/`p2p_gt` 来自 SIH **真值帧**，两者之间夹着这台 SITL 自己的估计器误差，"
                "它的量级**会随场次整块漂**" + gap_txt +
                "。⟹ **跨帧的倍数不可比**，别把它们排成一条线；"
                "判「谁先退化」请只看与判层同帧的那几条（`*_ekf`）。")
    return {"rows": rows, "resolvable": resolvable, "note": note, "frames": sorted(frames)}


def render_rank(r: dict, limit: int | None = None) -> str:
    """排序结果的 markdown 片段（给文档直接引）。"""
    out = ["| 名次 | 旋钮 | 离开基准散度带的倍数 | 由哪条读数 | **帧** | 场次 | 读数 | 基准中位数 |",
           "|---|---|---|---|---|---|---|---|"]
    for i, row in enumerate((r["rows"] if limit is None else r["rows"][:limit]), 1):
        if row["x"] is None:
            out.append(f"| {i} | `{row['knob']}` | — | {row['flag']} | — | — | — | — |")
        else:
            fr = rank_frame(row["key"])
            out.append(f"| {i} | `{row['knob']}` | **{row['x']:.2f}×** | `{row['key']}` | "
                       f"{'**EKF**' if fr == 'ekf' else '真值' if fr == 'truth' else '—'} | "
                       f"`{row['variant']}` rep{row['rep']} | {_f(row['value'])} | "
                       f"{_f(row['base_median'])} |")
    if r.get("note"):
        out.append("")
        out.append(f"⚠️ {r['note']}")
    return "\n".join(out)


#: 会改变"悬停需要多少推力"的旋钮 —— 只有这两个能把垂直通道推开。
#: `inertia` / `tau` / `rate` 不动悬停所需的推力（它们动的是姿态/带宽），
#: 所以**不参与**刚度反推：把它们算进去等于拿"没推动高度的档"去除，得到的是噪声。
GAIN_KNOBS = ("mass", "thrust")


def vertical_gain(s: dict, frame: str = "ekf") -> dict:
    """从 `mass±30` / `thrust±30` 的**实测高度偏差**反推垂直通道的等效刚度 `k` [N/m]。

    ★ 为什么这个数比"±30 % 会挂"有用得多：这几个档挂判据③，**不是发散、也不是姿态问题**，
      机理是"模型里的**悬停前馈**与实际所需不一致"——位置环只能用**误差**去补这个缺，
      于是稳态偏差 `Δz = −ΔF / k`。`k` 一出来，判据③ 的 ±0.15 m 就翻译成一句**能算的容差**：
      "悬停口径最多能错 `0.15·k / (m·g)`"。票面要的"边界"因此是个**百分比**，不是"某档挂/不挂"。

    ★★ **哪一列是量的、哪一列是算的**（这条不说清，整张表会被误引）：
      · `ΔF` = **算的** —— 从机型口径 `m·g/(4·T_max)` 解析算出来的"悬停所需推力差"，
        **不是**从飞行读数里量出来的。⟹ 它与别的票（如 ticket 20 §8.4）用同一个式子算出的
        数**不是"互为旁证"**，是**同一个数**。
      · `Δz` = **量的** —— 逐场 A 档仪表量到的稳态高度差。
      · `k` = 量的 `Δz` 除算的 `ΔF` ⟹ `k` 本身是**一次测量**，它的可信度来自
        "**另一套仪器（B 档、另一台被控对象档位）量到了同一个斜率**"（ticket 20 从 F1 的
        −0.6406 m 反解得 ≈3.4 N/m，与本页 3.57 N/m 差 3.5 %）—— 这才是可引的旁证。

    ★ **符号约定**：`k := 恢复力 ÷ 位移 = −ΔF / Δz`，`k > 0` 才叫"弹簧"——
      悬停所需变大（`ΔF > 0`）⟹ 实际下沉（`Δz < 0`）⟹ 两者异号。
      写成 `ΔF / Δz` 会得到一个恒为负的数（第一版就是这么错的），
      而那会让下面那条"`k ≤ 0` ⟹ 机理不符"的守卫把**正确的**数据全判成异常。

    机理与三个独立读数的一致性见 `docs/control/mismatch-sweep.md` §3.3。
    这里刻意**逐档各算一个 k 再取中位数**：四个档给出四个 k，它们散得开就说明"线性弹簧"
    这个模型本身不成立，那时 `resolvable=False`，不要报一个看着很精确的数。

    `ΔF` 一律换算到**基准被控对象**的推力满量程 `4·SIH_T_MAX(z0)`（**换算基准**，不是 `f_max`：
      `f_max` 是约束层按电池的 11.8 N，被控对象的满量程按电机、重搭后是 46.30 N —— 两者解耦）：
      `mass`：`ΔF = Δm·g`（同高度悬停所需的力差）；
      `thrust`：`ΔF = (frac_变体 − frac_基准)·4·T_max`，`frac = m·g/(4·T_max)`
        （悬停所需**没变**，但模型按基准的油门比例去命令 ⟹ 多命令的那部分）。
    ⚠️ **`ΔF` 的数值对满量程尺度不敏感**：`mass` 档两边约掉 `4·T_max`、`thrust` 档只看比值
      `T0/T1` ⟹ 重搭（`T_MAX_TOTAL` 11.8→46.3）**不改 `ΔF`**，只改 `f_scale` 那一列的显示值。

    `frame`：`ekf`（票 09 操作者裁定判在 EKF 帧上）或 `gt`。
    """
    key = f"climb_{frame}"
    base = (s.get("baseline") or {}).get(key)
    if not base or base.get("median") is None:
        return {"k": None, "rows": [], "resolvable": False,
                "note": "没有基准（`z0`）读数 ⟹ **反推不出刚度**"}
    p0 = expected_params("z0")
    frac0 = p0["SIH_MASS"] * G0 / (4.0 * p0["SIH_T_MAX"])
    f_scale = 4.0 * p0["SIH_T_MAX"]                 # = 基准被控对象的推力满量程（换算基准，牛顿）

    rows = []
    for variant, rs in sorted((s.get("by_variant") or {}).items()):
        if not rs or rs[0].get("knob") not in GAIN_KNOBS:
            continue
        p = expected_params(variant)
        frac = p["SIH_MASS"] * G0 / (4.0 * p["SIH_T_MAX"])
        d_f = (frac - frac0) * f_scale
        for row in rs:
            if row["status"] == "发散" or row.get(key) is None:
                rows.append({"variant": variant, "knob": row["knob"], "d_f_n": d_f,
                             "d_z_m": None, "k": None, "flag": "无读数/发散"})
                continue
            d_z = row[key] - base["median"]
            k = None if abs(d_z) < 1e-6 else -d_f / d_z      # `k := −ΔF/Δz`（见 docstring）
            rows.append({"variant": variant, "knob": row["knob"], "d_f_n": d_f,
                         "d_z_m": d_z, "k": k, "flag": None})
    ks = [r["k"] for r in rows if r["k"] is not None]
    k = statistics.median(ks) if ks else None
    if k is None:
        note = "没有任何一档既改了悬停推力、又有可用读数 ⟹ 算不出来"
    elif k <= 0:
        note = ("★ 反推出的 `k ≤ 0` ⟹ **机理与数据不符**（改了悬停推力却没把高度推开、"
                "或推反了）⟹ 别引这个数，先查夹具")
    elif len(ks) >= 2 and (max(ks) - min(ks)) / k > 0.25:
        note = (f"★ 逐档 k 散得太开（{min(ks):.2f} … {max(ks):.2f} N/m，"
                f"中位数 {k:.2f}）⟹ 「线性弹簧」这个模型在这一档不成立，**别当容差用**")
    else:
        note = None
    tol_n = HOVER_BAND_M * k if k else None
    return {"k": k, "k_all": ks, "rows": rows, "note": note, "frame": frame,
            "f_scale_n": f_scale, "hover_force_n": p0["SIH_MASS"] * G0,
            "resolvable": bool(k and k > 0 and not note),
            "tol_n": tol_n,
            # 判据③ 的 ±0.15 m 换算成"悬停推力口径最多能错百分之多少"
            "tol_frac": (tol_n / (p0["SIH_MASS"] * G0)) if tol_n else None}


def render_gain(g: dict) -> str:
    """刚度反推的 markdown 片段。"""
    # ★ 表头必须写清哪一列是**算出来的**、哪一列是**量出来的** —— 否则读的人会把 `ΔF`
    #   当成实测（它是从机型口径 `m·g/(4·T_max)` 解析算的），进而以为 `ΔF` 与别的票"互为旁证"。
    #   真正**互不相干地量出来的只有 `Δz`**；`k` 是"量的 Δz ÷ 算的 ΔF"，所以 `k` 的**可比性**
    #   来自"两台不同的仪器量到了同一个 Δz 对应的斜率"（见 docstring ★）。
    out = [f"| 变体 | 旋钮 | ΔF（**算**：按**基准被控对象**满量程 `4·SIH_T_MAX(z0)`"
           f"={g['f_scale_n']:.2f} N，N） | Δz（**量**：相对基准中位数，m） | "
           "k = −ΔF/Δz（N/m） |",
           "|---|---|---|---|---|"]
    for r in g["rows"]:
        out.append(f"| `{r['variant']}` | `{r['knob']}` | {_f(r['d_f_n'], '+.3f')} | "
                   + ("— | — |" if r["k"] is None else
                      f"{_f(r['d_z_m'], '+.3f')} | **{r['k']:.2f}** |"))
    if g.get("k"):
        out.append("")
        out.append(f"中位数 **k = {g['k']:.2f} N/m**（{g['frame']} 帧，"
                   f"推力口径 {g['f_scale_n']:.2f} N、悬停 {g['hover_force_n']:.3f} N）")
        if g.get("tol_frac"):
            out.append(f"⟹ 判据③ 的 ±{HOVER_BAND_M:g} m 相当于**悬停口径最多错 "
                       f"±{100 * g['tol_frac']:.1f} %**"
                       f"（{_f(g.get('tol_n'), '+.3f')} N）")
    if g.get("note"):
        out.append(f"⚠️ {g['note']}")
    return "\n".join(out)


def render_table(s: dict) -> str:
    """逐场一行（markdown）。★ 印在**文档里的那张表就是这张**，不是手抄的。"""
    cols = ("| 变体 | 旋钮 | rep | 状态 | EKF 抬升 m | 稳态误差 m | 峰峰值 m | 水平漂移 m | "
            "机头 rad | 求解稳态最坏 ms | 真值抬升 m | 覆盖层 |")
    out = [cols, "|" + "---|" * 12]
    order = (["z0"] if "z0" in s["by_variant"] else []) \
        + [v for v in s["by_variant"] if v != "z0"]
    for variant in order:
        for row in sorted(s["by_variant"][variant], key=lambda r: r["rep"]):
            ov = row.get("overlay_ok")
            out.append(
                f"| `{variant}` | {row.get('knob') or '—'} | {row['rep']} | {row['status']} | "
                f"{_f(row['climb_ekf'])} | {_f(row['e_z_ekf'])} | {_f(row['p2p_ekf'])} | "
                f"{_f(row['drift_ekf'])} | {_f(row['yaw_span_ekf'])} | "
                f"{_f(row['solve_steady_max'], '.3f')} | {_f(row['climb_gt'])} | "
                f"{'✅' if ov else '❌' if ov is False else '—'} |")
    base = s["baseline"]
    out.append("")
    out.append(f"基准（`z0`）{base['n']} 场；逐读数散度：")
    for key in METRIC_KEYS:
        b = base.get(key)
        if b:
            out.append(f"- `{key}`：中位数 {_f(b['median'])}、跨度 {_f(b['span'])}"
                       f"（{_f(b['min'])} … {_f(b['max'])}，n={b['n']}）")
    if base.get("warn"):
        out.append(f"- ⚠️ {base['warn']}")
    if any(r.get("overlay_ok") is False for rows in s["by_variant"].values() for r in rows):
        out.append("- ⚠️ **有场次覆盖层没生效**（读回与真源不符）⟹ 那几场的失配结论**不成立**")
    ctl = s.get("controller", {})
    for sig, pkgs in (ctl.get("packages") or {}).items():      # ★ 一轮的身份：**总是**印出来
        n = len(ctl["sigs"].get(sig, []))
        where = "、".join(f"`{p}`" for p in pkgs)
        tail = "" if sig == "（未留档）" else f"，签名 `{sig}`"
        out.append(f"- 控制器代码：{where}（{n} 场{tail}）")
    if ctl.get("warn"):
        out.append(f"- ⚠️ {ctl['warn']}")
    hs = s.get("harness") or {}
    if hs.get("warn"):
        out.append("- ⚠️ " + hs["warn"])
    elif hs.get("n_sigs") == 1:
        sig = next(iter(hs["sigs"]))
        out.append(f"- 扫描器自身代码：签名 `{sig}`（{len(hs['sigs'][sig])} 场）")
    tr = s.get("trace") or {}
    if tr:
        miss = tr["n_runs"] - tr["n"]
        how = "、".join(f"{k} {v} 场" for k, v in sorted((tr.get("by") or {}).items()))
        line = (f"- 逐帧轨迹（判据③换窗口重判用）：{tr['n']}/{tr['n_runs']} 场已入库"
                f"（{tr.get('hz')} Hz 降采样；来源：{how}）")
        if miss:
            line += (f"；⚠️ **{miss} 场没有**（原始流不在了）⟹ 这几场若换窗口口径重判"
                     f"**只能重飞**")
        out.append(line)
    c = s["counts"]
    out.append("")
    out.append(f"合计 {s['n_runs']} 场：发散 {c['发散']}、退化 {c['退化']}、稳定 {c['稳定']}。")
    return "\n".join(out)


def load_runs(dirpath: Path) -> list[dict]:
    """把 `obs-<变体>-<rep>.json` 读回来（驱动与汇总之间**只有文件**这一条接口）。"""
    dirpath = Path(dirpath)
    runs = []
    for path in sorted(dirpath.glob("obs-*.json")):
        m = re.match(r"^obs-(?P<v>.+)-(?P<r>\d+)\.json$", path.name)
        if not m:
            continue
        runs.append({"variant": m.group("v"), "rep": int(m.group("r")),
                     "path": str(path),
                     "obs": json.loads(path.read_text(encoding="utf-8"))})
    return runs


# ---------------------------------------------------------------- 夹具（单场）


def run_variant(variant: str, rep: int, outdir: Path, extras: list[str],
                hover_thrust: float | None, rawdir: Path | None = None) -> int:
    """**一场**：换覆盖层 → 跑 A 档那套仪器 → 把"覆盖层真的生效"的读数并进观测。

    ★ 换覆盖层的做法（**不动任何别人的文件**）：`verify_offboard_smoke.World._enter`
      通过**模块全局** `parse_overlay_into_env` 取覆盖层路径，所以把它换成
      "无视传进来的路径、一律用本变体那一份"即可。本进程只跑一个变体 ⟹ 全局补丁是安全的。
      ⚠️ 不要改成"临时覆盖 `z0-plant/params/z0.param`"那种写法：那会**改到入库产物**，
        而扫描中途崩掉就会把它留在盘上。
    """
    # ★ 推力口径：**必须在 import A 档工具之前**把 `nmpc.params.HOVER_THRUST` 改掉 ——
    #   它是 `from nmpc.params import HOVER_THRUST`（导入期绑值），改晚了那条判据读的还是旧值。
    import nmpc.params as np_params

    node_default = float(np_params.HOVER_THRUST)
    if hover_thrust is not None:
        np_params.HOVER_THRUST = float(hover_thrust)

    import verify_offboard_smoke as v
    import verify_tier_a_hover as ta

    if hover_thrust is not None:
        # ★ 另一处：`dut_params()` 只给 `log_file/state_source/hold_*/rate_hz`，
        #   **不传** `hover_thrust` ⟹ 节点会用 `mpc_node.py` 自己的默认值（= `params.py` 的
        #   `HOVER_THRUST`，重搭后与 `auto` 同档；重搭前它才是那个会掉飞机的真机口径）。
        #   这里把开关补上（`mpc_node` 本来就有这个参数：`mpc_node.py` 的 `_PARAMS`）。
        #   ⚠️ 用**包装**而不是改 `verify_tier_a_hover.py`：那是 ticket 11 正在动的文件，
        #     本张不碰它（改法记进文档，属"该做进工具里"的后续）。
        orig_dut_params = ta.dut_params

        def dut_params(cfg, _orig=orig_dut_params, _ht=float(hover_thrust)):
            return [*_orig(cfg), f"hover_thrust:={_ht}"]

        ta.dut_params = dut_params

    overlay = overlay_path(variant)
    orig = v.parse_overlay_into_env
    v.parse_overlay_into_env = lambda _path=None: orig(str(overlay))

    readback: dict = {}

    class SweepWorld(v.World):
        """A 档的 World + **起机后读回旋钮参数**。"""

        def _enter(self):
            world = super()._enter()
            for name in readback_names():
                readback[name] = self._read_param(name)
            return world

    v.World = SweepWorld

    # ★ 原始流（A 档的 `hover-1m-px4.jsonl`，一场 ~3 MB）与**入库的摘要**分开落盘：
    #   `--logdir` 指到 `/tmp` 时，仓库里只留 `obs-*.json` + `run-*.log`（一场 ~40 KB）。
    #   2026-09-29 撞出来的理由：`/tmp` 是 tmpfs，**一次重启就把整轮证据清空**；
    #   原始流不适合入库（13 场 ~43 MB），摘要必须入库。
    run_dir = Path(rawdir or outdir) / "logs" / f"{variant}-{rep}"
    run_dir.mkdir(parents=True, exist_ok=True)
    json_path = Path(outdir) / f"obs-{variant}-{rep}.json"
    argv = [*extras, "--outdir", str(run_dir), "--json", str(json_path)]
    rc = ta.main(argv)

    ok, detail = overlay_verdict(variant, readback)
    if json_path.exists():
        obs = json.loads(json_path.read_text(encoding="utf-8"))
        obs["sweep"] = {
            "variant": variant, "rep": rep, "overlay": overlay.name,
            "overlay_sha256": hashlib.sha256(overlay.read_bytes()).hexdigest()[:16],
            "readback": readback, "overlay_check": {"ok": ok, "detail": detail},
            "tier_a_argv": argv, "tier_a_rc": rc,
            "hover_thrust": hover_thrust, "hover_thrust_node_default": node_default,
            "hover_thrust_plant": plant_hover_thrust(),
            "controller_code": controller_code(),
            "loadavg": list(os.getloadavg()), "nproc": os.cpu_count(),
            # ★ **扫描器自己的身份**（2026-09-29 由并行会话 `ros-ws-72` 指出该洞后补）：
            #   `controller_code` 记的是**被测对象**，但"量它的那把尺子"当时没留痕 ——
            #   而本轮的 `mismatch_sweep.py` 恰好在两批之间被改过一次（11:32:31）。
            #   尺子变了 ⟹ 前后两批就未必是同一个实验，**这件事必须能从归档里查出来**，
            #   不然只能靠"我记得当时只改了 X"这种口头凭据。
            "harness": {"file": Path(__file__).name, "sha256": sha256_16(Path(__file__)),
                        "mtime": time.strftime(
                            "%Y-%m-%d %H:%M:%S",
                            time.localtime(Path(__file__).stat().st_mtime))},
        }
        # ★ **当场**挂逐帧轨迹：这一刻原始流一定还在盘上（它就在 `run_dir`）。
        #   事后补（`--attach-trace`）是给**已经跑完、原始流还没被清**的场次用的兜底 ——
        #   一旦 `/tmp` 被清，判据③就再也换不了窗口口径（只能整场重飞）。
        raw_jsonl = run_dir / "hover-1m-px4.jsonl"
        if raw_jsonl.exists():
            try:
                tr = trace_from_jsonl(raw_jsonl)
                if tr["n_kept"]:
                    # ★ 记"谁挂的"：`run_variant` = 当场挂（原始流一定在）；
                    #   `attach_trace` = 事后补。两者在归档里**必须能分开** ——
                    #   本轮就是"事后补"把"前后两批的差别"抹平、导致旁证不可复核（见文档 §附）。
                    tr["attached_by"] = "run_variant"
                    obs["trace"] = tr
            except Exception as exc:                       # noqa: BLE001 —— 不留档不该掀掉整场
                print(f"[trace] {variant} rep{rep}：挂轨迹失败（{exc!r}）⟹ 这一场"
                      f"将来换窗口口径重判只能重飞", file=sys.stderr)
        else:
            print(f"[trace] {variant} rep{rep}：{raw_jsonl} 不在 ⟹ 本场没挂轨迹",
                  file=sys.stderr)
        json_path.write_text(json.dumps(obs, ensure_ascii=False, indent=1), encoding="utf-8")
    print(f"[sweep] {variant} rep{rep}：A 档 rc={rc}；覆盖层读回 "
          f"{'✅' if ok else '❌'} {detail}", file=sys.stderr)
    return rc


# ---------------------------------------------------------------- 驱动


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0],
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--variants", default="",
                    help=f"逗号分隔；默认全部 {len(variant_order())} 个")
    ap.add_argument("--repeats", type=int, default=1, help="每个变体跑几场（基准建议 ≥2）")
    ap.add_argument("--outdir", default="/tmp/mismatch-sweep")
    ap.add_argument("--summarize", default=None, help="只汇总这个目录里的 obs-*.json，不飞")
    ap.add_argument("--dry-run", action="store_true", help="配合 `--fix-controller-code`：只判不改")
    ap.add_argument("--logdir", default=None,
                    help="A 档**原始流**（`hover-1m-px4.jsonl`，一场 ~3 MB）落到哪；"
                         "默认 = `--outdir`。`--outdir` 指进仓库时请把它指到 `/tmp`，"
                         "否则 13 场 ~43 MB 原始流会被一起入库")
    ap.add_argument("--attach-trace", action="store_true",
                    help="给 obs-*.json 补降采样逐帧轨迹（判据③换窗口可重判，不用整场重飞）")
    ap.add_argument("--trace-hz", type=float, default=TRACE_HZ,
                    help=f"轨迹降到多少 Hz 入库（默认 {TRACE_HZ}）")
    ap.add_argument("--recover-sweep", action="store_true",
                    help="给缺 `obs[\"sweep\"]` 的场次从 A 档 run log 恢复读回（只补不覆盖）")
    ap.add_argument("--reread-truth", default=None, metavar="DIR",
                    help="**离线**把 `--outdir` 那批的 A 档真值从控制台归档（DIR 里的 *.log）"
                         "按全速重取，与 `obs[\"truth\"]`（抽稀）逐场对照；只读、不飞、不改盘。"
                         "用来判'真值读数的散度是采样太稀还是真散'")
    ap.add_argument("--fix-controller-code", action="store_true",
                    help="重探控制器代码身份并修正 obs 里的旧记录（原值留在 "
                         "`controller_code_prev`；判据见 `judge_controller_record`）")
    ap.add_argument("--json", default=None, help="汇总结果写到哪（默认 <outdir>/summary.json）")
    ap.add_argument("--md", default=None, help="汇总表（markdown）写到哪")
    ap.add_argument("--timeout-s", type=float, default=1200.0, help="单场墙钟上限")
    ap.add_argument("--hover-thrust", default="auto",
                    help="给节点的 `hover_thrust`（**推力口径**）。`auto`（默认）= 按 SITL 机型现算 "
                         "`m·g/(4·SIH_T_MAX)`；`none` = 不覆盖（用节点自己的默认值）。"
                         "★ 重搭后两者同档（≈0.1588 / 0.1589），`auto` 只是让「与被控对象同源」显式化 "
                         "—— 见 `plant_hover_thrust()`")
    ap.add_argument("--run", default=None, help=argparse.SUPPRESS)   # 内部：单场模式
    ap.add_argument("--rep", type=int, default=1, help=argparse.SUPPRESS)
    args, extras = ap.parse_known_args(argv)

    if args.summarize:
        return _summarise_main(Path(args.summarize), args.json, args.md)

    if args.recover_sweep:
        for row in recover_sweep(Path(args.outdir), dry_run=args.dry_run):
            print(f"[recover] {'补' if row['changed'] else '留'} {row['file']}：{row['why']}")
        return 0

    if args.fix_controller_code:
        for row in refix_controller_codes(Path(args.outdir), dry_run=args.dry_run):
            mark = "改" if row["changed"] else "留"
            print(f"[recode] {mark} {row['file']}：{row['why']}")
        return 0

    if args.attach_trace:
        ld = Path(args.logdir) if args.logdir else None
        for row in attach_trace(Path(args.outdir), logdir=ld,
                                hz=args.trace_hz, dry_run=args.dry_run):
            mark = "挂" if row["changed"] else "留"
            print(f"[trace] {mark} {row['file']}：{row['why']}")
        return 0

    if args.reread_truth is not None:
        snaps = load_console_snaps(args.reread_truth)
        empty = [n for n, g in snaps.items() if not g]
        if not snaps:
            raise SystemExit(f"--reread-truth：{args.reread_truth} 里没有 *.log")
        rows = reread_truth(Path(args.outdir), snaps)
        if not rows:
            raise SystemExit(f"--reread-truth：{args.outdir} 里没有带 truth 的 obs-*.json")
        missed = [r for r in rows if r["hit"] < 1.0]
        print(render_reread(rows))
        print(f"\n[重读] {len(snaps)} 个快照（{len(empty)} 个空的没取到块）、{len(rows)} 场对照；"
              f"命中率 100 % 的 {len(rows) - len(missed)} 场"
              + (f"、**低于 100 % 的 {len(missed)} 场**（配错快照 ⟹ 该行读数不可用）" if missed else ""))
        return 0

    outdir = Path(args.outdir)
    outdir.mkdir(parents=True, exist_ok=True)

    ht = _resolve_hover_thrust(args.hover_thrust)

    if args.run:                                   # —— 单场模式（驱动 spawn 的就是它）
        for bad in ("--outdir", "--json", "--summarize"):
            if bad in extras:
                raise SystemExit(f"{bad} 由扫描器自己管，别透传")
        return run_variant(args.run, args.rep, outdir, extras, ht,
                           rawdir=Path(args.logdir) if args.logdir else None)

    variants = [s for s in args.variants.split(",") if s] or variant_order()
    for variant in variants:
        knob_of(variant)                            # 认不出来的变体**当场炸**，别跑完才发现
        overlay_path(variant)
    if args.repeats < 1:
        raise SystemExit("--repeats 至少 1")

    print(f"[sweep] {len(variants)} 个变体 × {args.repeats} 场 = "
          f"{len(variants) * args.repeats} 场 SITL；推力口径 "
          f"hover_thrust = {ht if ht is not None else '（不覆盖）'}；"
          f"透传参数 {extras or '（无）'}")
    t0 = time.time()
    for variant in variants:
        for rep in range(1, args.repeats + 1):
            cmd = [sys.executable, str(Path(__file__).resolve()), "--run", variant,
                   "--rep", str(rep), "--outdir", str(outdir),
                   "--hover-thrust", str(ht), *extras]
            if args.logdir:
                cmd += ["--logdir", str(args.logdir)]
            log = outdir / f"run-{variant}-{rep}.log"
            t1 = time.time()
            with open(log, "w", encoding="utf-8") as fh:
                try:
                    p = subprocess.run(cmd, stdout=fh, stderr=subprocess.STDOUT,
                                       timeout=args.timeout_s)
                    rc = p.returncode
                except subprocess.TimeoutExpired:
                    rc = -9
            print(f"[sweep] {variant} rep{rep}：rc={rc}、{time.time() - t1:.0f}s、"
                  f"日志 {log}", flush=True)
    print(f"[sweep] 全部跑完，{time.time() - t0:.0f}s ⟹ 汇总")
    return _summarise_main(outdir, args.json, args.md)


def _summarise_main(dirpath: Path, json_out, md_out) -> int:
    runs = load_runs(dirpath)
    if not runs:
        raise SystemExit(f"{dirpath} 里没有 obs-*.json")
    s = summarise(runs)
    table = render_table(s)
    rank = rank_degradation(s)
    gain = vertical_gain(s)
    print(table)
    print()
    print(render_rank(rank))
    print()
    print(render_gain(gain))
    s["rank"] = rank
    s["vertical_gain"] = gain
    json_path = Path(json_out) if json_out else dirpath / "summary.json"
    json_path.write_text(json.dumps(s, ensure_ascii=False, indent=1, default=str),
                         encoding="utf-8")
    print(f"\n[sweep] 汇总写到 {json_path}")
    if md_out:
        Path(md_out).write_text(table + "\n\n" + render_rank(rank) + "\n\n"
                                + render_gain(gain) + "\n", encoding="utf-8")
        print(f"[sweep] 表（含退化排序与刚度反推）写到 {md_out}")
    return 0 if s["counts"]["发散"] == 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
