"""符号映射表的**独立推导与交叉核对** —— 纯逻辑，不 import `rclpy` / `px4_msgs` / `numpy.linalg` 之外的东西。

★ **本模块不是权威，也不产生权威数值。** 权威在 [`docs/control/sign-map.md`](../../docs/control/sign-map.md)。
本模块的作用是：**用一条与那份文档互不相干的路径把同一张表推一遍**，再回头对账 ——
包括物理闭环（同一个指令在 ENU/FLU 与 NED/FRD 两套约定下必须给出同一个世界系加速度）。
换句话说，这里是**复核**，不是"第二份实现"。

三处"互不相干"的路径：

1. **定义 → 符号**：只用「FRD 的 y/z 是 FLU 的 y/z 的反向」这个定义，
   把角速度当**物理矢量**投影到两套基上（`rates_flu_to_frd`）。表里的 `[+1, −1, −1]` 是**算出来的**。
2. **物理闭环**：同一条 MPC（模型预测控制）内部指令，分别走
   ① ENU/FLU 直接积分、② 映射成 PX4 的四个字段后按 NED/FRD 积分，断言世界系加速度相同。
   这一条抓的是"符号表自洽但物理方向相反"——**光靠第 1 条抓不到**。
3. **三方对账**：推导结果 vs 文档里的 `sign-map:canonical` 块 vs `~/refs/px4-mpc` 与
   `~/refs/px4_msgs` 的实际文本。外部仓库不在时**明确报 SKIP**，绝不静默通过。

术语：`ω` 恒指**机体相对惯性系**的角速度，分量按所在机体系读（FLU 或 FRD）。
`F` 是总推力，单位 N，`F ≥ 0`，方向沿机体 **+z（FLU）**。
"""

from __future__ import annotations

import re
import sys
from pathlib import Path

import numpy as np

__all__ = [
    "DEFAULT_DOC",
    "DEFAULT_PX4_MPC",
    "DEFAULT_PX4_MSGS",
    "skew",
    "rot_from_rotvec",
    "enu_to_ned_matrix",
    "flu_to_frd_matrix",
    "rates_flu_to_frd",
    "thrust_flu_to_frd_z",
    "thrust_body_2",
    "accel_enu_from_flu",
    "accel_enu_from_px4",
    "parse_canonical_block",
    "derived_signs",
    "read_px4_mpc_signs",
    "read_px4_msgs_thrust_comment",
    "Skip",
]


# ---------------------------------------------------------------- 路径

_REPO = Path(__file__).resolve().parents[2]
DEFAULT_DOC = _REPO / "docs" / "control" / "sign-map.md"
DEFAULT_PX4_MPC = Path.home() / "refs" / "px4-mpc"
DEFAULT_PX4_MSGS = Path.home() / "refs" / "px4_msgs"


class Skip(Exception):
    """外部依赖不在场 —— 明确报 SKIP，不静默通过。"""


# ---------------------------------------------------------------- 线性代数小工具


def skew(v):
    """反对称矩阵：`skew(a) @ b == cross(a, b)`。"""
    x, y, z = (float(c) for c in v)
    return np.array([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]])


def rot_from_rotvec(v):
    """罗德里格斯公式：把旋转矢量（轴 × 角，弧度）变成旋转矩阵。

    `v` 很小的时候直接用级数，避免 `v/‖v‖` 把噪声放大 —— 自检里 `ω·dt` 就在 1e-6 量级。
    """
    v = np.asarray(v, dtype=float)
    theta = float(np.linalg.norm(v))
    if theta < 1e-9:
        K = skew(v)
        return np.eye(3) + K + 0.5 * (K @ K)
    K = skew(v / theta)
    return np.eye(3) + np.sin(theta) * K + (1.0 - np.cos(theta)) * (K @ K)


# ---------------------------------------------------------------- 两套基（只由定义构造）

#: 世界系 ENU → NED 的基变换：ENU 的 (e, n, u) 在 NED 里是 (n, e, −u)。
#: 这是一个**真旋转**（det = +1），不是镜像。
_ENU_TO_NED = np.array([[0.0, 1.0, 0.0],
                        [1.0, 0.0, 0.0],
                        [0.0, 0.0, -1.0]])

#: 机体系 FLU → FRD 的基变换：x 不变，y 与 z 取反（`CONTEXT.md` §一）。
#: det = +1 ⟹ 等价于"绕机体 x 转 180°"，**不是镜像** ⟹ 角速度按普通矢量变换，没有额外负号。
_FLU_TO_FRD = np.diag([1.0, -1.0, -1.0])


def enu_to_ned_matrix():
    """世界系 ENU → NED 的基变换矩阵（由定义构造，不是查表得来的）。"""
    return _ENU_TO_NED.copy()


def flu_to_frd_matrix():
    """机体系 FLU → FRD 的基变换矩阵（`diag(1, −1, −1)`，自逆）。

    ⚠️ 名字必须与方向一致：它自逆，所以**改反了也照样算对**，
    但那样一来名字就在一个"专治方向写反"的模块里撒了谎。
    """
    return _FLU_TO_FRD.copy()


# ---------------------------------------------------------------- 推导 2：FLU → FRD


def rates_flu_to_frd(omega_flu):
    """把 FLU 分量表示的角速度**物理矢量**，投影到 FRD 的三根轴上。

    这是推导，不是抄：先把分量还原成矢量，再逐轴投影 ——
    所以 `[+1, −1, −1]` 是这条式子**算出来**的结果，不是写死的常数。
    """
    omega = np.asarray(omega_flu, dtype=float)
    # `_FLU_TO_FRD` 的行就是 FRD 三根轴在 FLU 基里的方向 ⟹ 逐行点乘 = 投影。
    return _FLU_TO_FRD @ omega


def thrust_flu_to_frd_z(F_n):
    """推力沿机体 +z(FLU)，求它在 FRD 的 z 轴上的分量 —— 结果恒为 `−F`。"""
    return float(_FLU_TO_FRD[2, 2] * float(F_n))


def thrust_body_2(F_n, t_max_n):
    """归一化推力，值域 `[−1, 0]`（向上推力为负）。

    ★ **量纲与精确表达式以 [`docs/control/sign-map.md`](../../docs/control/sign-map.md) §2/§6 为准**，
    此处只记不变量：输入 `[N] / [N]` ⟹ **无量纲**；`F=0 ⟹ 0`；`F=T_max ⟹ −1`。
    不在 docstring 里复写那条式子 —— 数值只有一处（该文件 §8.4 的防漂移要求）。
    """
    if t_max_n <= 0:
        raise ValueError("T_max 必须为正")
    return thrust_flu_to_frd_z(F_n) / float(t_max_n)


# ---------------------------------------------------------------- 物理闭环


def accel_enu_from_flu(omega_flu, F_n, mass_kg, dt_s, g=9.81):
    """① ENU/FLU 约定：直接积分一个小步长，返回**世界系（ENU）**加速度。

    前提：初始姿态为单位阵（机头朝东、水平）。推力沿机体 +z，四旋翼没有别的控制力。
    """
    R = rot_from_rotvec(np.asarray(omega_flu, dtype=float) * dt_s)
    thrust_dir_world = R @ np.array([0.0, 0.0, 1.0])          # 机体 +z 在世界系里的方向
    return F_n / mass_kg * thrust_dir_world + np.array([0.0, 0.0, -g])


def accel_enu_from_px4(roll, pitch, yaw, thrust_body_z, t_max_n, mass_kg, dt_s, g=9.81):
    """② 把 PX4 的四个字段按 **PX4 自己的语义**（机体 FRD、世界 NED）积分，再换算回 ENU。

    不用 `docs/control/sign-map.md` 里的任何数值 —— 这是**另一条路径**，
    物理闭环就靠"两条路径给的 ENU 加速度必须相同"来判定。

    PX4 语义的三条来源（都是第一手，见文档附录）：
      · `roll/pitch/yaw` 是机体角速率，单位 rad/s；
      · `thrust_body[2]` 是归一化推力，**沿机体 z(FRD，朝下)**，向上推力为负；
      · 机体水平、机头朝东时，FRD 三轴在 NED 里是 `(东, 南, 下)`。
    """
    # 同一个物理状态（机头朝东、水平）在 NED/FRD 里的姿态：C · I · S。
    R0 = enu_to_ned_matrix() @ np.eye(3) @ _FLU_TO_FRD
    omega_frd = np.array([roll, pitch, yaw], dtype=float)
    R = R0 @ rot_from_rotvec(omega_frd * dt_s)                 # 机体角速率 ⟹ 右乘
    body_z_ned = R @ np.array([0.0, 0.0, 1.0])                 # 机体 z(FRD) = 朝下那根轴
    force_ned = thrust_body_z * t_max_n * body_z_ned
    a_ned = force_ned / mass_kg + np.array([0.0, 0.0, g])
    return np.array([a_ned[1], a_ned[0], -a_ned[2]])           # NED → ENU


# ---------------------------------------------------------------- 对账：文档 / 外部源码


def parse_canonical_block(doc_path=DEFAULT_DOC):
    """从权威文档里读 `sign-map:canonical` 块 —— **数值的唯一来源就是它**。

    返回 `{"roll_rate": +1.0, "pitch_rate": -1.0, "yaw_rate": -1.0, "thrust_body_2": -1.0}`，
    其中 `thrust_body_2` 是 `F / T_max` 的系数。
    """
    text = Path(doc_path).read_text(encoding="utf-8")
    m = re.search(r"<!--\s*sign-map:canonical.*?-->\s*```text\n(.*?)```", text, re.S)
    if not m:
        raise AssertionError(
            f"{doc_path} 里找不到 `sign-map:canonical` 块 —— 权威块被改名/删掉了吗？")
    body = m.group(1)
    out = {}
    for key in ("roll_rate", "pitch_rate", "yaw_rate", "thrust_body_2"):
        mm = re.search(rf"^\s*{key}\s*=\s*([+-]?\d+(?:\.\d+)?)\s*\*\s*([A-Za-z_0-9 /]+?)\s*$",
                       body, re.M)
        if not mm:
            raise AssertionError(f"权威块里读不出 `{key}`（原文：{body!r}）")
        out[key] = float(mm.group(1))
    return out


def derived_signs():
    """★ 独立推导出来的符号 —— 这条路径**不读**权威文档、也不读任何参考实现。

    做法：给三轴各喂一个单位角速率，看它在 FRD 里落到哪根轴上、什么符号；
    推力同理（喂 1 N，看它在 FRD 的 z 分量）。
    """
    signs = {}
    for name, i in (("roll_rate", 0), ("pitch_rate", 1), ("yaw_rate", 2)):
        e = np.zeros(3)
        e[i] = 1.0
        w_frd = rates_flu_to_frd(e)
        # 只该有第 i 个分量非零；它的符号就是映射的符号。
        off = np.delete(w_frd, i)
        if np.any(np.abs(off) > 1e-12):
            raise AssertionError(f"{name} 的映射混进了别的轴：{w_frd}")
        signs[name] = float(np.sign(w_frd[i]))
    signs["thrust_body_2"] = float(np.sign(thrust_flu_to_frd_z(1.0)))
    return signs


def read_px4_mpc_signs(repo=DEFAULT_PX4_MPC):
    """从 `~/refs/px4-mpc` 的**实际源码文本**里读出它用的符号。

    参考实现的 `thrust_rates = [F, ωx, ωy, ωz]`，所以下标 1/2/3 就是三轴角速率。
    仓库不在时抛 `Skip`。
    """
    path = Path(repo) / "px4_mpc" / "px4_mpc" / "mpc_quadrotor.py"
    if not path.exists():
        raise Skip(f"参考实现不在场：{path}")
    text = path.read_text(encoding="utf-8")
    pats = {
        "roll_rate": r"setpoint_msg\.roll\s*=\s*(?:float\()?\s*([+-]?)thrust_rates\[1\]",
        "pitch_rate": r"setpoint_msg\.pitch\s*=\s*(?:float\()?\s*([+-]?)thrust_rates\[2\]",
        "yaw_rate": r"setpoint_msg\.yaw\s*=\s*(?:float\()?\s*([+-]?)thrust_rates\[3\]",
        "thrust_body_2": r"thrust_command\s*=\s*-\(\s*thrust_rates\[0\]",
    }
    out = {}
    for key, pat in pats.items():
        m = re.search(pat, text)
        if not m:
            raise AssertionError(f"参考实现里匹配不到 `{key}` 的写法（{path}:{pat}）—— 上游改过？")
        out[key] = -1.0 if (key == "thrust_body_2" or m.group(1) == "-") else 1.0
    return out


def read_px4_msgs_thrust_comment(repo=DEFAULT_PX4_MSGS):
    """`px4_msgs` 的 `VehicleRatesSetpoint.msg` 里那句"向上推力为负"的原话。仓库不在时抛 `Skip`。"""
    path = Path(repo) / "msg" / "VehicleRatesSetpoint.msg"
    if not path.exists():
        raise Skip(f"px4_msgs 不在场：{path}")
    text = path.read_text(encoding="utf-8")
    if "negative throttle demand" not in text:
        raise AssertionError(f"{path} 里没有 'negative throttle demand' 那句 —— 上游改过措辞？")
    return "negative throttle demand"


def _main(argv=None):
    """直接 `python3 sign_map_core.py` 时给一句指路的，**故意不返回 0**。

    本模块是库，没有 CLI。之前跑它什么都不打印、退出码 0 —— 看上去像"复核通过了"，
    而其实一个用例都没跑。对一个专门用来防"静默错"的工具来说这是最刺眼的自相矛盾。
    """
    print(
        "本文件是被导入的库，没有命令行入口。\n"
        "要跑复核，请用：\n"
        "    python3 tools/sign-map/test_sign_map_core.py --self-test\n"
        "或：\n"
        "    python3 -m pytest tools/sign-map/test_sign_map_core.py -q\n",
        file=sys.stderr,
    )
    return 2


if __name__ == "__main__":
    sys.exit(_main())
