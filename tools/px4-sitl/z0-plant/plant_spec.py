#!/usr/bin/env python3
"""z0 同构 SITL 被控对象（SIH）的**单一真源**：机型文件与参数覆盖层都由这里生成。

为什么要有这个文件：[ticket 08](../../../.scratch/phase2/issues/08-sitl-z0-airframe.md)
要求"参数值、切换方式、如何复原"可复现，而 [ticket 12](../../../.scratch/phase2/issues/12-mismatch-sweep.md)
要扫 5 个旋钮 × 2 个方向（原始清单 4 个，`rate` 是 2026-09-28 加的）。若靠手抄 12 份参数文件，文档与文件必然漂移。

- `--emit`   生成 `airframes/10046_sihsim_z0` 与 `params/<变体>.param`（入库产物）
- `--check`  只校验已入库产物与本文件一致（不写盘）
- `--table`  打印 markdown 参数表（文档直接引用，避免手抄漂移）
- `--list`   列出全部变体名

失配旋钮 **5** 个（ticket 12 的原始清单是 4 个 —— 质量 / 推力系数 / 转动惯量 / 内环时间常数；
第 5 个 `rate` 是 2026-09-28 操作者裁定「两者都做」时加的：原 `tau` 旋钮扫的是**作动器滞后**，
而 MPC 所用模型里的 `k_ω` 是**闭环**时间常数，两者不是同一个量，故补一个直接改速率环 P 增益的旋钮）。

数值分**五档**诚实标注（前两档是本项目文档的既有约定；后三档是 ticket 08 加的必要区分 ——
"照抄上游"与"项目决定"**既不是 z0 的读数、也不是由 z0 读数推出来的**，
把它们混进前两档就是**假证据**）：
`实测`（z0 自己的第一手读数）/ `推得`（由实测值经明写的算式得到）/
`照抄`（**z0 侧根本没有对应物**，照抄上游同构文件或上游默认值）/ `决定`（**项目决定**，刻意偏离 z0）/
`占位`（无 z0 数据，取默认值或量级估，待 [ticket 10](../../../.scratch/phase2/issues/10-param-measure-backfill.md) 回填）。
"""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent

# z0 的第一手参数导出（**已入库**：`docs/assets/z0.params`）—— 凡是"z0 值"的出处都指它
Z0_PARAMS = "`docs/assets/z0.params`"

# ---------------------------------------------------------------- 几何与标称值

DIAG = 0.152          # m，实测轴距（对角线）—— hardware.md
ARM = DIAG / 2.0      # m，机臂（对角半长）= 0.076
LEVER = ARM / math.sqrt(2.0)   # m，单轴投影力臂 = 0.0537403…
MASS = 0.750          # kg，实测（750 g）
# N，**按电机**的可用总推力满量程（真机实测 46.3027 N，= m·g(9.81)/hover_thrust(0.1589)）。
# ★★ **这里按电机、不按电池** —— 被控对象 = 机体的**物理能力**（桨/电机瞬时能给多少），
#    电池"能可持续供多少"是**约束层**的事（`Z0Params.f_max` = 11.8 N），不写进被控对象。
#    两件事合用一个常数的时代在 ticket 21 之前（`T_MAX_TOTAL = 11.8`）—— 那让 SIH 自带一个
#    不存在的电流限制，且"电池上限"被烘进被控对象 ⟹ 解耦在 SITL 里测不出来。
#    见 phase2-implementation.md §3.3.3 / 票 `.scratch/phase2/issues/21-rebuild-sitl-plant-to-measured.md`。
T_MAX_TOTAL = 46.3027
G = 9.80665

MOTOR_T_MAX = T_MAX_TOTAL / 4.0        # SIH 的 T_MAX 是**单电机**
KM = 0.05                              # = CA_ROTORn_KM（z0 值 = PX4 默认）
T_HOVER = MASS * G                     # 7.355 N
# 点质量上界：把整机质量放在力臂处 —— 真实转动惯量必小于它，故是**上界**
I_POINT = MASS * LEVER ** 2

# ---------------------------------------------------------------- 参数表

# (名字, 值, 档次, 出处/理由, 写进哪边)
#   "airframe" → 机型文件里的 `param set-default`（机器身份）
#   "overlay"  → 覆盖层里的 `param set`（会被扫/会被改的那部分）
# PWM_MAIN_FUNC 的理由四条共用（`identity` vs z0 的 `3-2-1-4`）
_FUNC_WHY = (
    "★ **刻意偏离 z0**：z0 是 `3-2-1-4`（`103/102/101/104`，`docs/assets/z0.params`）。"
    "SIH 的混控式把**输出通道 i 当作第 i 个旋翼**（`sih.cpp:304-306`，它从不读 `CA_ROTOR*`），"
    "所以 SITL 里只有 identity 映射能让「分配器以为的旋翼位置」与「SIH 摆的旋翼位置」对上；"
    "照抄 z0 那组会让 1↔3 错位 ⟹ 俯仰反号。ESC 接线标签在 SITL 里没有对应物"
)

ROWS = [
    # ---- 被控对象（SIH 刚体 + 一阶电机）—— 两处都写：机型给安全默认，覆盖层强制现值
    ("SIH_MASS", MASS, "实测",
     "z0 质量 750 g（hardware.md §6.1）", "both"),
    ("SIH_T_MAX", MOTOR_T_MAX, "推得",
     f"★ **按电机的**可用总推力满量程 {T_MAX_TOTAL} N ÷ 4（**不按电池** —— 被控对象 = 机体物理能力，"
     f"电池可持续上限是约束层 `f_max`；两个角色见 phase2-implementation.md §3.3.3）", "both"),
    ("SIH_Q_MAX", KM * MOTOR_T_MAX, "推得",
     "= CA_ROTOR_KM × SIH_T_MAX ⟹ 被控对象的偏航力矩/推力比 = 分配器以为的 KM", "both"),
    ("SIH_L_ROLL", LEVER, "推得",
     "152 mm 对角线 → 机臂 0.076 m → 单轴投影 0.0537 m（与 CA_ROTOR_PX/PY 必须相等）", "both"),
    ("SIH_L_PITCH", LEVER, "推得", "同上", "both"),
    ("SIH_IXX", I_POINT, "占位",
     "点质量上界 m·l²；真值未辨识（见 sitl-z0-plant.md §8）", "both"),
    ("SIH_IYY", I_POINT, "占位", "同 IXX", "both"),
    ("SIH_IZZ", 2.0 * I_POINT, "占位", "= 2·IXX（细长体量级估）", "both"),
    ("SIH_IXY", 0.0, "占位", "对称假设（无实测）", "both"),
    ("SIH_IXZ", 0.0, "占位", "对称假设（无实测）", "both"),
    ("SIH_IYZ", 0.0, "占位", "对称假设（无实测）", "both"),
    ("SIH_T_TAU", 0.05, "占位",
     "SIH 默认值；z0 内环时间常数未辨识（phase2 §5 的 P4 才是它）", "both"),
    ("SIH_KDV", 1.0, "占位", "SIH 默认；保持默认以免给被控对象偷偷加阻尼", "both"),
    ("SIH_KDW", 0.025, "占位", "SIH 默认；同上", "both"),

    # ---- 机型身份：构型 / 几何 / 输出分配 / 检查位
    ("SIH_VEHICLE_TYPE", 0, "实测",
     "z0 是多旋翼（hardware.md §6.1）；值 0 = SIH 的 VehicleType::MC", "airframe"),
    ("CA_ROTOR_COUNT", 4, "实测", "z0 是四旋翼", "airframe"),
    ("CA_ROTOR0_PX", +LEVER, "推得",
     "X 形；★ **刻意不取 z0.params 的 `0.15`**（那是 PX4 通用 X 型默认值、比实测几何大 2.8 倍，"
     "见 hardware.md:38 / params-analysis.md §3.1）。取实测 152 mm 推得的真值；"
     "与 SIH 混控式（sih.cpp:304-306）逐项同号", "airframe"),
    ("CA_ROTOR0_PY", +LEVER, "推得", "", "airframe"),
    ("CA_ROTOR0_KM", +KM, "实测", f"z0 值 = PX4 默认（{Z0_PARAMS}）", "airframe"),
    ("CA_ROTOR1_PX", -LEVER, "推得", "", "airframe"),
    ("CA_ROTOR1_PY", -LEVER, "推得", "", "airframe"),
    ("CA_ROTOR1_KM", +KM, "实测", f"同 ROTOR0（{Z0_PARAMS}）", "airframe"),
    ("CA_ROTOR2_PX", +LEVER, "推得", "", "airframe"),
    ("CA_ROTOR2_PY", -LEVER, "推得", "", "airframe"),
    ("CA_ROTOR2_KM", -KM, "实测", f"z0 值（{Z0_PARAMS}）；符号 = 桨的转向", "airframe"),
    ("CA_ROTOR3_PX", -LEVER, "推得", "", "airframe"),
    ("CA_ROTOR3_PY", +LEVER, "推得", "", "airframe"),
    ("CA_ROTOR3_KM", -KM, "实测", f"同 ROTOR2（{Z0_PARAMS}）", "airframe"),
    ("CA_ROTOR0_CT", 6.5, "实测",
     f"z0 值 = PX4 默认（{Z0_PARAMS}）；★ 绝对值会被伪逆归一化掉，不必等于 SIH_T_MAX", "airframe"),
    ("PWM_MAIN_FUNC1", 101, "决定", _FUNC_WHY, "airframe"),
    ("PWM_MAIN_FUNC2", 102, "决定", "同上；identity 映射", "airframe"),
    ("PWM_MAIN_FUNC3", 103, "决定", "同上；identity 映射", "airframe"),
    ("PWM_MAIN_FUNC4", 104, "决定", "同上；identity 映射", "airframe"),
    ("SENS_EN_GPSSIM", 1, "照抄", f"z0 侧**没有对应物**（{Z0_PARAMS} 里查无此参数）；照抄上游 10040，EKF 要有传感器可吃", "airframe"),
    ("SENS_EN_BAROSIM", 1, "照抄", "同上", "airframe"),
    ("SENS_EN_MAGSIM", 1, "照抄", "同上", "airframe"),
    ("CBRK_SUPPLY_CHK", 894281, "实测", f"z0 值（{Z0_PARAMS}，恰与上游 10040 同值）；仿真里没有电池", "airframe"),
    ("CBRK_IO_SAFETY", 22027, "实测", f"z0 值（{Z0_PARAMS}，恰与上游 10040 同值）；仿真里没有安全开关", "airframe"),

    # ---- PX4 侧（只写覆盖层）
    ("THR_MDL_FAC", 0.0, "决定",
     "★ **故意不等于 z0 的 0.45**，理由**不是**「SIH 是线性的所以别叠非线性」（那太弱），"
     "而是**端到端等价**：混控器把**推力**反解成电机信号（`FunctionMotors.hpp:93`），"
     "SIH 的被控对象对电机信号线性（`sih.cpp:303`）⟹ 取 0 时「命令推力 → 实得推力」是**恒等**，"
     "而真机恰好也是恒等（真机的物理曲线与 0.45 模型**相消**）。取 0.45 反而在 SITL 里造出"
     "一条**真机上不存在**的失真（同一个 z0-thrcurve 变体实测：§6.2 那个起飞工作点上是"
     "命令 `0.1166` → 信号 `0.1842`，即**同命令多出约 58 % 推力**；旧机型同一工作点是 `0.6325` → `0.7227`，"
     "约 14 % —— 这个百分比**随工作点变**，因为曲线非线性）。"
     "⚠️ 代价写在这儿：取 0 时 SITL 的**电机信号数值**与真机不同（真机命令 0.1166 → 信号 0.1842，"
     "SITL 则是 0.1166 → 0.1166）—— 若将来比较「MPC 输出的电机指令大小」或研究电机饱和边界要记得。"
     "`0.45` 是标定值还是残留值**未知**（归 phase4/02）。见 sitl-z0-plant.md §5.2",
     "overlay"),
    ("MC_ROLLRATE_MAX", 220, "实测", f"z0 值（{Z0_PARAMS}；角速率环约束）", "overlay"),
    ("MC_PITCHRATE_MAX", 220, "实测", "z0 值（同上）", "overlay"),
    ("MC_YAWRATE_MAX", 360, "实测", f"z0 值（{Z0_PARAMS}；≠ PX4 默认 200）", "overlay"),
    ("MC_ROLLRATE_P", 0.15, "实测",
     f"z0 值（{Z0_PARAMS}；恰等于 PX4 默认，params-analysis.md §3.1 已坐实）；"
     "★ 被 `rate` 旋钮扫（内环闭环带宽失配），故基线里也要显式写",
     "overlay"),
    ("MC_PITCHRATE_P", 0.15, "实测", "同 ROLLRATE_P（z0 = PX4 默认）", "overlay"),
    ("MC_YAWRATE_P", 0.2, "实测", "z0 值（z0 = PX4 默认 0.2；偏航**单独给**，不取横滚的值）", "overlay"),
    ("MPC_TILTMAX_AIR", 45, "实测",
     f"z0 现值 = PX4 默认 45（{Z0_PARAMS}）。★★ **本行原写成 `决定` + 取 30**（想靠收小倾角给被控对象"
     "留推力裕度：30° 需 8.5 N = 电池可用推力的 72 %，45° 需 10.4 N = 88 %）——"
     "**2026-09-28 操作者裁定「两边都回 45」**，于是它成了本表里**唯一一处「想过偏离但最终没偏」**的参数。"
     "⚠️ 提醒留着：45° 的 88 % 意味着留给姿态控制的余量只有 12 %，大机动时注意推力饱和。"
     "⚠️ **这是 PX4 位置控制器参数，与 MPC 毫无关系**（Multicopter Position Controller）。"
     "见 sitl-z0-plant.md §5.3 / params-changes.md §E6.4",
     "overlay"),
    ("MC_BAT_SCALE_EN", 0, "实测", f"z0 值 = 默认（{Z0_PARAMS}；不要电池电压补偿，仿真里没有电池模型）", "overlay"),
    ("MC_AIRMODE", 0, "实测", "z0 值 = 默认", "overlay"),
]

# ---------------------------------------------------------------- 失配旋钮

# ticket 12 的旋钮：**单参数** ±30%（一个变体只动一个旋钮；原始清单 4 个 + `rate` 共 5 个）
KNOBS = {
    "mass": {
        "params": ["SIH_MASS"],
        "why": "挂载/换电池；只动质量、不动转动惯量 —— 这才是'单参数'的含义",
    },
    "thrust": {
        "params": ["SIH_T_MAX", "SIH_Q_MAX"],
        "why": "推力系数；Q_MAX 跟着动以保持偏航/推力比（= CA_ROTOR_KM）不变，"
               "所以这个旋钮只改推力总量，不改偏航通道",
    },
    "inertia": {
        "params": ["SIH_IXX", "SIH_IYY", "SIH_IZZ"],
        "why": "转动惯量辨识误差；三轴同比例动（一个旋钮）",
    },
    "tau": {
        "params": ["SIH_T_TAU"],
        "why": "**作动器滞后**辨识误差（`SIH_T_TAU` 是电机的一阶滞后，`sih.cpp:293-294`；"
               "官方描述是 0→100 % 用时约 4τ）。★ **它 ≠ phase2 §5 里的 P4 `k_ω`** —— "
               "后者是**角速率环的闭环**时间常数，从 IMU 日志拟合。要扫真正的闭环带宽失配用 `rate` 旋钮",
    },
    "rate": {
        "params": ["MC_ROLLRATE_P", "MC_PITCHRATE_P", "MC_YAWRATE_P"],
        "why": "**内环闭环带宽失配**（≈ MPC 所用模型里 `k_ω` 的失配）：三个轴的速率环 P 增益同比例 ±30 %。"
               "★ 与其余四个旋钮**性质不同** —— 那四个动的是**被控对象**，这个动的是**控制器**；"
               "P 与闭环带宽在低增益下近似成正比，故 ±30 % P ≈ ±30 % `k_ω`（近似，精确值要实测）。"
               "被控对象本身不变 ⟹ 本旋钮用 `verify_z0_plant.py` **量不到**（它只验被控对象），"
               "要闭环阶跃响应才量得出（ticket 09/11 的仪表）",
    },
}

MISMATCH_PCT = 0.30

# 非扫描项的额外变体：用来量化"推力曲线失配"有多大（本张的取证用，不属 12 的旋钮）
EXTRA_VARIANTS = {
    "z0-thrcurve": ({"THR_MDL_FAC": 0.45}, "复现 z0 固件的 `THR_MDL_FAC`，用于量化曲线失配"),
}

# ---------------------------------------------------------------- 加密档（ticket 12 §附.1）

#: 边界附近的**加密档**：±30 % 只答了"哪一档越线了"，没答**线在哪**。这几档把边界夹紧。
DENSE_PCTS = (0.10, 0.05)

#: 只对**探到边界的那两个**旋钮加密。另外三个（`inertia`/`tau`/`rate`）在 ±30 % 上都没退化
#: ⟹ 往更小幅度扫没有信息量（退化只会更不可能），扫了也只是多烧机时。
DENSE_KNOBS = ("mass", "thrust")


def build_dense_variants() -> dict:
    """边界附近的加密档 —— **刻意不并进** `build_variants()`。

    ★ **为什么不改 `MISMATCH_PCT` 了事**：`build_variants()` 把它**编进了变体名**
      （`z0-{knob}{tag}{int(MISMATCH_PCT*100)}`）。把它改成 0.10 ⟹ **11 个变体全体改名**
      （`z0-mass+30` → `z0-mass+10`）、被控对象跟着变，而 ticket 12 那 13 场归档
      （文件名与读数都挂在这套名字上）**当场孤立**。那不是"加一档"，是**换掉基线**。
      ⟹ 加密档**只增不改**，`MISMATCH_PCT` 永远是 `0.30`。

    ★ **为什么与 `build_variants()` 分开**：那 11 个变体是 08/09 的产物与 12 的归档所挂的
      **基线**；加密档是**另一组实验**。混进同一个函数 ⟹ "默认变体集"悄悄长大
      （`--variants-all`、`--check` 都跟着变），而报告上看不出来。
    """
    base = {n: v for n, v, _s, _w, scope in ROWS if scope in ("overlay", "both")}
    out = {}
    for knob in DENSE_KNOBS:
        spec = KNOBS[knob]
        for pct in DENSE_PCTS:
            pct_tag = int(round(pct * 100))
            for sign, tag in ((+1, "+"), (-1, "-")):
                vals = dict(base)
                factor = 1.0 + sign * pct
                for p in spec["params"]:
                    vals[p] = base[p] * factor
                out[f"z0-{knob}{tag}{pct_tag}"] = vals
    return out


def all_variants() -> dict:
    """**基线 ∪ 加密档**。凡是"按名字查一个变体"的地方都用它。

    ★ 别直接用 `build_variants()[name]` —— 那样加密档会 KeyError
      （`verify_z0_plant.expected_params` 就是靠这个查找，ticket 12 的判据⑤ 又靠它）。
    """
    out = build_variants()
    out.update(build_dense_variants())
    return out


def build_variants() -> dict:
    """变体名 → {参数名: 值}（**全量**，不是差量 —— 每个覆盖层都自洽可读）。"""
    base = {n: v for n, v, _s, _w, scope in ROWS if scope in ("overlay", "both")}

    out = {"z0": dict(base)}
    for knob, spec in KNOBS.items():
        for sign, tag in ((+1, "+"), (-1, "-")):
            vals = dict(base)
            factor = 1.0 + sign * MISMATCH_PCT
            for p in spec["params"]:
                vals[p] = base[p] * factor
            out[f"z0-{knob}{tag}{int(MISMATCH_PCT * 100)}"] = vals

    for name, (overrides, _why) in EXTRA_VARIANTS.items():
        vals = dict(base)
        vals.update(overrides)
        out[name] = vals
    return out


# ---------------------------------------------------------------- 渲染

def _knob_of(variant: str) -> str:
    """`z0-mass+30` → `mass`。"""
    for knob in KNOBS:
        if variant.startswith(f"z0-{knob}"):
            return knob
    raise KeyError(variant)


def fmt(value) -> str:
    if isinstance(value, int):
        return str(value)
    return f"{value:.6g}"


def render_airframe() -> str:
    """机型文件：机器身份部分（几何/构型/输出分配/检查位 + SIH 标称默认值）。"""
    lines = [
        "#!/bin/sh",
        "#",
        "# @name z0 同构四旋翼 SITL（SIH，含参数失配开关）",
        "#",
        "# @type Quadrotor",
        "#",
        "# ⚠️ **本文件由 tools/px4-sitl/z0-plant/plant_spec.py 生成，不要手改** ——",
        "#    改参数请改那份 spec 再 `--emit`，否则文档表格与这里的值会漂移。",
        "#",
        "# 由 ticket 08 新增。上游 10040_sihsim_quadx 是 x500 量级的通用四旋翼（质量 1 kg、",
        "# 力臂 0.15/0.2 m 互相矛盾），与 z0（750 g、152 mm 轴距）差得远 ——",
        "# 「在 10040 上能飞」证明不了「在 z0 上能飞」。",
        "#",
        "# 参数分两处：这里是**机器身份**（构型、几何、输出分配、检查位）+ SIH 标称默认值；",
        "# 会被扫/会被改的那部分（SIH_* 现值、THR_MDL_FAC、MC_*RATE_MAX、MC_*RATE_P、MPC_TILTMAX_AIR）",
        "# 走覆盖层（z0-plant/params/*.param，经 rcS 的 `. px4-rc.params` 钩子按名取自 PATH）。",
        "# 理由与复现步骤见 docs/control/sitl-z0-plant.md。",
        "#",
        "",
        ". ${R}etc/init.d/rc.mc_defaults",
        "",
        "PX4_SIMULATOR=${PX4_SIMULATOR:=sihsim}",
        "PX4_SIM_MODEL=${PX4_SIM_MODEL:=z0}",
        "",
    ]
    for name, value, _strength, _why, scope in ROWS:
        if scope in ("airframe", "both"):
            lines.append(f"param set-default {name} {fmt(value)}")
    lines.append("")
    # SIH_VEHICLE_TYPE 是在 SIH init 时读的，必须无条件写（照抄上游 10040 的做法）
    lines.append("# SIH 在 init 时读它，必须 `param set`（`set-default` 会被已持久化的值压住）")
    lines.append("param set SIH_VEHICLE_TYPE 0")
    lines.append("")
    return "\n".join(lines)


def render_overlay(variant: str, values: dict) -> str:
    if variant == "z0":
        head = "# z0 同构机的**参数一致对照组**（ticket 12 的基准侧）"
    elif variant in EXTRA_VARIANTS:
        head = f"# 非扫描变体：{EXTRA_VARIANTS[variant][1]}"
    else:
        knob = _knob_of(variant)
        head = (f"# 失配变体：{knob} ×{variant.split(knob)[1]}"
                f"（{KNOBS[knob]['why']}）")
    lines = [
        head,
        "#",
        "# ⚠️ 由 tools/px4-sitl/z0-plant/plant_spec.py 生成，不要手改。",
        "# 全量参数（不是差量）：无条件 `param set` ⟹ 结果与 rootfs/parameters.bson 里的",
        "# 历史残留无关，这一条是 ticket 12 能复现的前提。",
        "",
    ]
    for name, _v, _s, _w, scope in ROWS:
        if scope in ("overlay", "both"):
            lines.append(f"param set {name} {fmt(values[name])}")
    lines.append("")
    return "\n".join(lines)


def render_table() -> str:
    out = ["| 参数 | 值 | 档次 | 出处 / 理由 |", "|---|---|---|---|"]
    for name, value, strength, why, _scope in ROWS:
        out.append(f"| `{name}` | `{fmt(value)}` | {strength} | {why or '—'} |")
    out.append("")
    # 变体表按**旋钮**合并成一行（± 两个变体同义，只是方向不同）——
    # 逐变体列会把每个旋钮的理由抄两遍，人读不动；变体名仍逐个写全，不丢信息
    out.append("| 变体 | 动的参数 | 含义 |")
    out.append("|---|---|---|")
    out.append("| `z0` | — | 参数一致的对照组（基准） |")
    for knob, spec in KNOBS.items():
        names = " / ".join(f"`z0-{knob}{tag}{int(MISMATCH_PCT * 100)}`" for tag in ("+", "-"))
        out.append(f"| {names} | "
                   f"{', '.join('`'+p+'`' for p in spec['params'])} | "
                   f"{spec['why']} |")
    for variant, (overrides, why) in EXTRA_VARIANTS.items():
        out.append(f"| `{variant}` | "
                   f"{', '.join('`'+p+'`' for p in overrides)} | {why} |")
    # 加密档：它们是**另一组**（不改 `MISMATCH_PCT` 那套基线），故单列一行并写明理由，
    # 免得有人以为"变体表里怎么多了几档 ±10 %/±5 %"就是基线动了
    for knob in DENSE_KNOBS:
        names = " / ".join(
            f"`z0-{knob}{tag}{int(round(pct * 100))}`"
            for pct in DENSE_PCTS for tag in ("+", "-"))
        out.append(f"| {names} | "
                   f"{', '.join('`'+p+'`' for p in KNOBS[knob]['params'])} | "
                   f"**边界加密档**（ticket 12 §附.1）：把 ±30 % 探到的边界夹紧；"
                   f"**不改**上方 ±30 % 那套基线 |")
    return "\n".join(out)


# ---------------------------------------------------------------- 落盘 / 校验

def targets() -> dict:
    out = {"airframes/10046_sihsim_z0": render_airframe()}
    for variant, values in all_variants().items():
        out[f"params/{variant}.param"] = render_overlay(variant, values)
    return out


def main(argv=None):
    ap = argparse.ArgumentParser(description="z0 同构 SIH 机型/参数的单一真源")
    g = ap.add_mutually_exclusive_group(required=True)
    g.add_argument("--emit", action="store_true", help="写盘生成（幂等）")
    g.add_argument("--check", action="store_true", help="校验已入库产物与本文件一致")
    g.add_argument("--table", action="store_true", help="打印 markdown 参数表")
    g.add_argument("--list", action="store_true", help="列出变体名")
    args = ap.parse_args(argv)

    if args.table:
        print(render_table())
        return 0
    if args.list:
        for name in all_variants():
            print(name)
        return 0

    bad = 0
    for rel, content in sorted(targets().items()):
        path = HERE / rel
        if args.emit:
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(content, encoding="utf-8")
            print(f"写入 {path.relative_to(HERE.parent.parent.parent)}")
        else:
            if not path.exists():
                print(f"缺失：{rel}", file=sys.stderr)
                bad += 1
            elif path.read_text(encoding="utf-8") != content:
                print(f"**与 spec 不一致**：{rel}（跑 --emit 重新生成）", file=sys.stderr)
                bad += 1
    if args.check:
        if bad:
            return 1
        print(f"OK：{len(targets())} 份产物与 spec 一致")
    return 0


if __name__ == "__main__":
    sys.exit(main())
