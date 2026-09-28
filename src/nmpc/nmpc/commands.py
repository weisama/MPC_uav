"""**参数化的指令程序** —— ticket 05 的 stub 用它，没有模型、没有代价、没有约束。

一句话：给一条时间轴，它给出"此刻该给多大推力、该绕哪根轴转多快"。
它**不是控制器**（不读状态、不求解、不闭环），只是一个能被人手写出来的指令串。
本模块存在的理由就是让"中间件 / 符号 / 归一化"这三件事**在没有模型预测控制数学的环境里**
也能被真 PX4 检验 —— 出问题时才能确定不是模型算错了。

**为什么单独一个模块、而不是塞在节点里**：它是纯函数，`pytest` 里一毫秒跑完，
不需要 ROS、不需要飞机、不需要 SITL。节点只负责把它接到话题上。

程序语义（三条，都是刻意的）：

1. **时间是"OFFBOARD 生效后"的相对秒**，不是墙钟 —— 否则模式切换花掉的那几秒会吃掉第一段。
2. **走完不停发**：最后一段一直保持到进程退出。停发会触发 PX4 的 offboard 丢失失效保护
   （`COM_OF_LOSS_T`），而那是**断流实验**专门要单独观测的事，不能混进别的实验里。
3. **`thrust_scale` 是悬停推力的倍数**，不是归一化推力、也不是牛顿 ——
   归一化那一步是 `sign_map` 的事（除以 `T_max`），两者分开才能分别验。
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Iterable, Sequence, Tuple

__all__ = ["Phase", "parse_phase", "parse_program", "hover_force_n", "command_at", "describe"]

G = 9.80665  # m/s²，与 tools/px4-sitl/z0-plant/plant_spec.py 同一个常数


def _positive(value, what: str) -> float:
    """必须**有限且为正** —— 光写 `<= 0` 拦不住 `nan`（`nan <= 0` 是假）。

    ★ 为什么值得单拎出来：`nan` 时长不会报错，`command_at` 里 `t < elapsed + nan` 恒假
    ⟹ 那一段**永远轮不到**，后面的段被 `elapsed += nan` 一起吃掉 ⟹ 整条程序塌成最后一段，
    飞机从第一拍起就在执行另一套指令，而报告上什么都看不出来。
    本模块宁可在这里炸掉，也不静默发一条不是人写的指令。
    """
    if isinstance(value, float) and not math.isfinite(value):
        raise ValueError(f"{what}必须是有限数，收到 {value!r}")
    if value <= 0:
        raise ValueError(f"{what}必须为正，收到 {value!r}")
    return float(value)


def _non_negative(value, what: str) -> float:
    """同上，但允许 0（0 是"不给推力 / 不转"这一档）。"""
    if isinstance(value, float) and not math.isfinite(value):
        raise ValueError(f"{what}必须是有限数，收到 {value!r}")
    if value < 0:
        raise ValueError(f"{what}不能为负，收到 {value!r}")
    return float(value)


def _finite(value, what: str) -> float:
    """不限符号，但必须是有限数 —— 角速率可以取负（那是反向转），但不能是 `nan`。"""
    if isinstance(value, float) and not math.isfinite(value):
        raise ValueError(f"{what}必须是有限数，收到 {value!r}")
    return float(value)


@dataclass(frozen=True)
class Phase:
    """一个恒定段：`duration_s` 秒内，推力 = `thrust_scale` × 悬停推力，角速率 = `omega_flu`。"""

    name: str
    duration_s: float
    thrust_scale: float
    omega_flu: Tuple[float, float, float]

    def __post_init__(self) -> None:
        if not self.name:
            raise ValueError("每一段要有个名字（打印与取证都要用）")
        _positive(self.duration_s, f"段 {self.name!r} 的时长")
        _non_negative(self.thrust_scale, f"段 {self.name!r} 的推力倍数")
        for i, w in enumerate(self.omega_flu):
            _finite(w, f"段 {self.name!r} 的角速率 ω[{i}]")


def parse_phase(spec: str) -> Phase:
    """`名字:时长秒:悬停推力倍数:ωx,ωy,ωz`（ω 单位 rad/s，机体 FLU）。

    例：`lift:1.2:1.20:0,0,0` ⟹ 以 1.2 倍悬停推力上升 1.2 s，不转。
    """
    parts = spec.split(":")
    if len(parts) != 4:
        raise ValueError(f"段格式应为 `名字:时长:推力倍数:ωx,ωy,ωz`，收到 {spec!r}")
    name, duration, scale, omega = parts
    try:
        omega_flu = tuple(float(v) for v in omega.split(","))
    except ValueError as exc:
        raise ValueError(f"段 {name!r} 的角速率解析不了：{omega!r}") from exc
    if len(omega_flu) != 3:
        raise ValueError(f"段 {name!r} 的角速率要是三个数（FLU 的 x,y,z），收到 {omega!r}")
    try:
        duration_s, thrust_scale = float(duration), float(scale)
    except ValueError as exc:
        raise ValueError(f"段 {name!r} 的时长/倍数解析不了：{duration!r} / {scale!r}") from exc
    # ★ 构造**留在 try 外面**：`Phase.__post_init__` 的校验话术（nan / inf / 负值各说各的）
    #   比这里那句"解析不了"具体得多，被裹进同一个 `except ValueError` 就等于把
    #   `lift:nan:…` 报成"解析不了"—— 明明是能解析、但不许用的值。
    return Phase(name=name, duration_s=duration_s,
                 thrust_scale=thrust_scale, omega_flu=omega_flu)


def parse_program(specs: Iterable[str]) -> list:
    """把若干段串成一条时间轴（顺序即时间顺序）。空程序是**错误**，不是"什么都不发"。"""
    program = [parse_phase(s) for s in specs]
    if not program:
        raise ValueError("指令程序不能为空 —— 空程序意味着不发 setpoint，那是断流实验而不是本用法")
    return program


def hover_force_n(mass_kg: float, g: float = G) -> float:
    """悬停所需的**总推力**（N）：`m·g`。

    ⚠️ 这是**本模块唯一的"物理"**，而且只是 `m·g` 这一个乘法 —— 不是模型。
    真值里的 `mass` 由调用方给（SITL 里就是 `SIH_MASS`；z0 实机是 750 g）。
    """
    _positive(mass_kg, "质量")
    return mass_kg * g


def command_at(program: Sequence[Phase], t_s: float, hover_n: float) -> Tuple[float, Tuple[float, float, float]]:
    """`t_s` 时刻的 `(推力[N], (ωx, ωy, ωz)[rad/s])`。

    `t_s < 0`（时钟抖动）取第一段；超出总时长取**最后一段**（见模块 docstring 第 2 条）。
    """
    if t_s < 0:
        phase = program[0]
    else:
        phase = program[-1]
        elapsed = 0.0
        for candidate in program:
            if t_s < elapsed + candidate.duration_s:
                phase = candidate
                break
            elapsed += candidate.duration_s
    return phase.thrust_scale * hover_n, phase.omega_flu


def describe(program: Sequence[Phase], hover_n: float) -> str:
    """一行一段的可读时间轴（日志 / 证据里用），带每段的起止秒。"""
    out = []
    elapsed = 0.0
    for phase in program:
        out.append(f"{elapsed:6.1f}s +{phase.duration_s:<5.1f}s {phase.name:<10}"
                   f" F={phase.thrust_scale * hover_n:.3f}N ({phase.thrust_scale:.2f}×悬停)"
                   f" ω={phase.omega_flu}")
        elapsed += phase.duration_s
    return "\n".join(out)
