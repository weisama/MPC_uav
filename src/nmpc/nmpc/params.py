"""z0 的模型参数 —— 10 状态 MPC（模型预测控制）用。

★ **本文件里的数分两类，别混**（改之前先看这一行）：

| 类 | 有哪些 | 出处 |
|---|---|---|
| **实测** | `mass` | [`hardware.md`](../../../docs/reference/hardware.md) §9 汇总表「用户实测」，0.750 kg |
| **已对账的飞控参数** | 三轴角速率上限 | `z0.params` 与飞控真身对账过（[`params-analysis.md`](../../../docs/reference/params-analysis.md) 的 `MC_*RATE_MAX`）—— 是**飞控现状**，不是我们量的，但也不是占位 |
| **占位值** | `hover_thrust`、`t_max`（由它导出） | 见下 |

⚠️ 归一化常数（参考实现里硬编码的 `0.07`）**不在本文件里**：它在
`nmpc/sign_map.py` 的 `px4_rates_setpoint(..., t_max_n)` 里以**形参**出现，由调用方传 `t_max`。
本文件只负责把 `t_max` 与 `hover_thrust` 锁成一条导出关系（见下）。

**占位关系必须自洽**（[`phase2-implementation.md`](../../../docs/control/phase2-implementation.md) §6.3）：
PX4 的 offboard 分支对 `thrust_body` **不做任何悬停油门重标定、原样透传**（第一手：
`MulticopterRateControl.cpp:182`），所以"模型里的 `hover_thrust`"与"推力归一化常数 `1/T_max`"
**描述的是同一个物理量**。两者不一致就是一个**静默的推力增益错误**——
这里用 `T_max = m·g / hover_thrust` 这条**导出**关系把它锁死：只有 `hover_thrust` 是自由的，
`T_max` 跟着它走，改一个不可能忘改另一个。

**回填口 = [ticket 10](../../../.scratch/phase2/issues/10-param-measure-backfill.md)**：
`hover_thrust` 是 §5 里 P1（唯一"测不准就飞不起来"的参数，从 Phase 1 悬停日志反推 PX4 实际输出的
归一化推力）。P1 出数后**只改这一个字段**，`t_max` 自动跟着变。
"""

from __future__ import annotations

from dataclasses import dataclass

#: 重力加速度 [m/s²] —— 与参考实现 `multirotor_rate_model.py` 的 `g` 同一个值
G = 9.81

#: 悬停油门**占位值**（无量纲，[0, 1]）。
#:
#: ★ 它是**从电池限制的 `T_max` 反推**出来的，不是独立量：
#:     `hover_thrust = T_hover / T_max = (0.75 × 9.81) / 11.8 = 0.6235`
#: 取 11.8 N 的理由与推法见 [`phase2-implementation.md`](../../../docs/control/phase2-implementation.md) §3.3
#: （电池 Mooch 实测的持续放电能力；**不是**电机上限 40 N）。
#: ⚠️ 它与 PX4 参数 `MPC_THR_HOVER`（写的是 0.5）**不是一回事** —— 那个是 PX4 位置控制器的悬停油门，
#: 在 `(C)` 级下完全不参与推力映射（见 `CONTEXT.md` 开头对 `MPC_*` 的辨析）。
HOVER_THRUST = 0.6235

#: 角速率上限 [rad/s] —— ★ **偏航单独给，不是横滚的一半**（参考实现写成了 `0.5*wmax`，见 M3）。
#:
#: 出处：[`params-analysis.md`](../../../docs/reference/params-analysis.md) 的 z0 参数表
#: —— `MC_ROLLRATE_MAX` / `MC_PITCHRATE_MAX` = 220 °/s = 3.8397 rad/s、`MC_YAWRATE_MAX` = 360 °/s = 6.2832 rad/s。
#: 这里取三位有效数字（3.84 / 6.28），与 `z0.params` 里那两个数的差别 < 0.01 %。
ROLL_RATE_MAX = 3.84
PITCH_RATE_MAX = 3.84
YAW_RATE_MAX = 6.28


@dataclass(frozen=True)
class Z0Params:
    """一套模型参数。**可变**（`dataclasses.replace`）—— [ticket 12](../../../.scratch/phase2/issues/12-mismatch-sweep.md)
    的 ±30 % 失配扫描要拿它造变体，所以刻意做成不可变数据类而不是一堆模块常量。"""

    #: 起飞质量 [kg] —— **实测值**，出处 `hardware.md` §9 汇总表
    mass: float = 0.75

    #: 悬停油门（无量纲）—— 占位值，回填口见模块 docstring（P1 / ticket 10）
    hover_thrust: float = HOVER_THRUST

    #: 三轴角速率上限 [rad/s]
    roll_rate_max: float = ROLL_RATE_MAX
    pitch_rate_max: float = PITCH_RATE_MAX
    yaw_rate_max: float = YAW_RATE_MAX

    @property
    def t_hover(self) -> float:
        """悬停所需总推力 [N]。"""
        return self.mass * G

    @property
    def t_max(self) -> float:
        """推力上限 [N] —— ★ **按电池**（≈11.8 N），不按电机（40 N）。见 M4 与 §3.3。

        由 `hover_thrust` 导出 ⟹ 与它自洽；这也是参考实现的算法（`max_thrust = mass*9.81/hover_thrust`）。"""
        return self.t_hover / self.hover_thrust

    @property
    def rate_max(self) -> tuple[float, float, float]:
        """三轴角速率上限 [rad/s]，顺序 = `[ωx, ωy, ωz]` = `[roll, pitch, yaw]`。"""
        return (self.roll_rate_max, self.pitch_rate_max, self.yaw_rate_max)


#: 默认那一套（z0 实测 + 占位）
DEFAULT = Z0Params()
