"""z0 的模型参数 —— 10 状态 MPC（模型预测控制）用。

★ **本文件里的数分两类，别混**（改之前先看这一行）：

| 类 | 有哪些 | 出处 |
|---|---|---|
| **实测** | `mass`、`hover_thrust`（⟹ `t_max`） | `mass`： [`hardware.md`](../../../docs/reference/hardware.md) §9 汇总表「用户实测」，0.750 kg。`hover_thrust`：**从 z0 自己的悬停日志反推**（[ticket 10](../../../.scratch/phase2/issues/10-param-measure-backfill.md)，2026-09-29），见下 |
| **已对账的飞控参数** | 三轴角速率上限 | `z0.params` 与飞控真身对账过（[`params-analysis.md`](../../../docs/reference/params-analysis.md) 的 `MC_*RATE_MAX`）—— 是**飞控现状**，不是我们量的，但也不是占位 |
| **电池口径的引数** | `f_max` | `hardware.md` §6.2 的 Mooch 持续放电 15 A 外推（≈11.8 N）—— 是**引数**，不是我们量的 |
| ~~占位值~~ | ~~`hover_thrust`、`t_max`~~ | ✅ **2026-09-29 已全部实测**（ticket 10 的 P1/P2 回填），本表已无占位 |

⚠️ 归一化常数（参考实现里硬编码的 `0.07`）**不在本文件里**：它在
`nmpc/sign_map.py` 的 `px4_rates_setpoint(..., t_max_n)` 里以**形参**出现，由调用方传 `t_max`。
本文件只负责把 `t_max` 与 `hover_thrust` 锁成一条导出关系（见下）。

**这条导出关系必须自洽**（[`phase2-implementation.md`](../../../docs/control/phase2-implementation.md) §6.3）：
PX4 的 offboard 分支对 `thrust_body` **不做任何悬停油门重标定、原样透传**（第一手：
`MulticopterRateControl.cpp:182`），所以"模型里的 `hover_thrust`"与"推力归一化常数 `1/T_max`"
**描述的是同一个物理量**。两者不一致就是一个**静默的推力增益错误**——
这里用 `T_max = m·g / hover_thrust` 这条**导出**关系把它锁死：只有 `hover_thrust` 是自由的，
`T_max` 跟着它走，改一个不可能忘改另一个。

★★ **`t_max` 与 `f_max` 是两个物理量，别再合回去**（2026-09-29 裁定解耦）：
`t_max` 是**归一化**常数（必须 = 实测满量程 46.3 N，悬停工况量的），`f_max` 是 **`F` 上限**
（电池可持续 ≈11.8 N，满油门工况问的）。合用一个常数的后果已经踩过一次 —— 回填 `hover_thrust`
把 `ocp.py` 的 `F ≤ T_max` 从"按电池"悄悄抬成"按电机"（差 3.9 倍，测试全绿）。

**回填口 = [ticket 10](../../../.scratch/phase2/issues/10-param-measure-backfill.md)**：
`hover_thrust` 是 §5 里 P1（唯一"测不准就飞不起来"的参数，从 Phase 1 悬停日志反推 PX4 实际输出的
归一化推力）。P1 出数后**只改这一个字段**，`t_max` 自动跟着变（`f_max` 不跟着动 —— 它是另一件事）。
"""

from __future__ import annotations

from dataclasses import dataclass

#: 重力加速度 [m/s²] —— 与参考实现 `multirotor_rate_model.py` 的 `g` 同一个值
G = 9.81

#: 悬停油门 —— ★ **实测值**（无量纲，[0, 1]）。2026-09-29 由 ticket 10 回填，替换占位 `0.6235`。
#:
#: ★ 它是**从 z0 自己的悬停日志反推**出来的，不是独立量、也不是抄来的常数：
#:     工具读数（两段可用日志）`0.221992` @ **20.474 V**、`0.220884` @ **20.526 V**（段内电压**中位数**）；
#:     按就地标定的 `thrust ∝ V^1.608`（两段共 4056 个样本合并拟合，n = 1.608、R² = 0.79）
#:     归一化到**满电 25.2 V** ⟹ `0.15896` / `0.15882`（互差 0.09 %）⟹ 均值 0.15889 ⟹ 取 **0.1589**。
#:     ★ **口径要能当场复算**：`hover_thrust(25.2) = 读数 × (V_窗口 / 25.2)^1.608`
#:       —— 上面每个数都是这么来的，拿计算器就能验（早先只印两位小数的电压，那一对复算不出来）。
#:     ⚠️ 指数本身只在 1.61 附近定得下来（同一方法的复算落在 **1.608 ~ 1.611**；单窗 1.590 / 2.330，
#:       换成滤波电压 1.500）⟹ 终值随它在 **0.1588 / 0.1589** 之间动 0.06 % —— 这是同一个测量的两种取整，
#:       远小于 n 本身的不确定区间（1.5–2.0 ⟹ ±5 %）。本张取 0.1589，因为它与 1.608 成对。
#: 方法与判据见 [`tools/hover-thrust`](../../../tools/hover-thrust/README.md)，
#: 口径裁定（★ 满电口径）与原始分析见 [`phase2-implementation.md`](../../../docs/control/phase2-implementation.md) §3.3 与该票 Comments。
#:
#: ★★ **电池口径是这份参数的一部分**：`MC_BAT_SCALE_EN = 0` ⟹ PX4 不按电压缩放 `thrust_body`，
#: 而真实推力随电压掉（实测 `V^1.6`）⟹ 一个常数**必然**在某段电量上是错的：
#: 满电 0.159 / 标称 0.195 / 实用截止 0.273（T/W 6.3 / 5.1 / 3.7）。取满电是本票的裁定。
#:
#: ⚠️ 它与 PX4 参数 `MPC_THR_HOVER`（z0 上是 0.5）**不是一回事** —— 那个是 PX4 位置控制器的悬停油门，
#: 在 `(C)` 级下完全不参与推力映射（见 `CONTEXT.md` 开头对 `MPC_*` 的辨析）。
#: ⚠️ 另一条别搞混的：`hover_thrust` 是**推力**域的数，与**电机信号**域差一层 `THR_MDL_FAC` 曲线
#: （实测悬停推力 0.221 对应悬停油门信号 0.318）。
HOVER_THRUST = 0.1589

#: 角速率上限 [rad/s] —— ★ **偏航单独给，不是横滚的一半**（参考实现写成了 `0.5*wmax`，见 M3）。
#:
#: 出处：[`params-analysis.md`](../../../docs/reference/params-analysis.md) 的 z0 参数表
#: —— `MC_ROLLRATE_MAX` / `MC_PITCHRATE_MAX` = 220 °/s = 3.8397 rad/s、`MC_YAWRATE_MAX` = 360 °/s = 6.2832 rad/s。
#: 这里取三位有效数字（3.84 / 6.28），与 `z0.params` 里那两个数的差别 < 0.01 %。
ROLL_RATE_MAX = 3.84
PITCH_RATE_MAX = 3.84
YAW_RATE_MAX = 6.28

#: ★★ **`F` 上限** [N] —— 电池**可持续**口径（M4/D5），**与上面的归一化常数是两个物理量**
#: （2026-09-29 操作者裁定**解耦**：`hover_thrust`/`t_max` 只负责归一化，这里负责"能要多少推力"）。
#:
#: 出处 [`hardware.md`](../../../docs/reference/hardware.md) §6.2：Mooch 实测持续放电 **15 A** ⟹ 单电机
#: 299.5 gf ⟹ 4 × 299.5 g = 1198 g ⟹ **11.75 N**（`T/W` 1.60）；本文件取三位 **11.8 N**，与票面口径一致。
#: ⚠️ 旧占位时代那个 `m·g/0.6235 = 11.8003 N` 与它只差 ~0.4 % —— **是巧合**（SITL 同构机当初照占位搭的），
#: **不要**因为"对得上"就把两个角色又混回一个常数。
#:
#: ⚠️ 它**不是**"飞机最多能推出多少力"——那是归一化那一侧的事（`t_max` = 实测满量程 46.3 N）。
#: 它回答的是另一个问题：**电池能持续兑现多少**。规划出超过它的推力，飞控/电池兑现不了，
#: 现象是"指令下去飞机不动"。所以它进 `ocp.py` 的 `ubu`（`F ≤ f_max`），**不进**归一化。
#: ⚠️ 它随**电量**掉（满电 → 截止：`hover_thrust(V) = 0.1589×(25.2/V)^1.608`，见 `t_max` 那段），
#: 现在是个定值 —— 按电压缩放它要等倾角/电池约束那张票（本张只把它从归一化里拆出来）。
F_MAX_BATTERY = 11.8


@dataclass(frozen=True)
class Z0Params:
    """一套模型参数。**可变**（`dataclasses.replace`）—— [ticket 12](../../../.scratch/phase2/issues/12-mismatch-sweep.md)
    的 ±30 % 失配扫描要拿它造变体，所以刻意做成不可变数据类而不是一堆模块常量。"""

    #: 起飞质量 [kg] —— **实测值**，出处 `hardware.md` §9 汇总表
    mass: float = 0.75

    #: 悬停油门（无量纲）—— **实测值**（满电口径），出处的完整交代见模块 docstring
    hover_thrust: float = HOVER_THRUST

    #: 三轴角速率上限 [rad/s]
    roll_rate_max: float = ROLL_RATE_MAX
    pitch_rate_max: float = PITCH_RATE_MAX
    yaw_rate_max: float = YAW_RATE_MAX

    #: `F` 上限 [N] —— ★ **电池可持续口径**（M4/D5），进 `ocp.py` 的 `ubu`；
    #: **不是**归一化常数（那是 `t_max`）。两者的解耦是 2026-09-29 的裁定，
    #: 出处的完整交代见模块常量 `F_MAX_BATTERY` 那段。
    f_max: float = F_MAX_BATTERY

    @property
    def t_hover(self) -> float:
        """悬停所需总推力 [N]。"""
        return self.mass * G

    @property
    def t_max(self) -> float:
        """**归一化**上限 [N] —— 满电口径下实测 **46.3 N**（T/W 6.29）。
        由 `hover_thrust` 导出 ⟹ 与它自洽；这也是参考实现的算法（`max_thrust = mass*9.81/hover_thrust`）。

        ★ **它只负责一件事：把 `F` 换算成 `thrust_body`**（`sign_map.px4_rates_setpoint` 的 `t_max_n`）。
        §3.3.2 的自洽条件要求它**等于实测满量程**，填别的值悬停前馈就错。
        ⚠️ **别拿它当 `F` 上限** —— 那是 `f_max`（按电池 ≈11.8 N）。
        2026-09-29 之前这两个角色兼在一个常数上：回填后它变成 46.3 N，
        于是 `ocp.py` 的 `F ≤ T_max` 悄悄从"按电池"漂成了"按电机"（差 3.9 倍，且不报错）。
        ★ 也**别**为了让它 < 40 N 去动 `hover_thrust`（那是把测量改小去迁就判据）。
        """
        return self.t_hover / self.hover_thrust

    @property
    def rate_max(self) -> tuple[float, float, float]:
        """三轴角速率上限 [rad/s]，顺序 = `[ωx, ωy, ωz]` = `[roll, pitch, yaw]`。"""
        return (self.roll_rate_max, self.pitch_rate_max, self.yaw_rate_max)


#: 默认那一套 —— **全部实测**（2026-09-29 起再无占位）
DEFAULT = Z0Params()
