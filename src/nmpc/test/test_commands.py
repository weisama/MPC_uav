"""指令程序的纯逻辑单测 —— 零 ROS、零 SITL，一毫秒跑完。

这里验的是**时间轴本身**（哪一段、推力多大、走完怎么办），不是飞行。
凡是"该报错"的地方都要真报错：本包的取舍是**宁可炸掉也不静默发错**，
而 SITL 里一个静默的错误指令表现为"飞机往错的方向飞"，那比崩掉难查得多。
"""

from __future__ import annotations

import pytest

from nmpc import commands
from nmpc.commands import Phase

PROGRAM = commands.parse_program(["lift:1.2:1.2:0,0,0", "hover:8:1.0:0,0,0"])
HOVER_N = commands.hover_force_n(0.75)  # ≈ 7.355 N


# ---------------------------------------------------------------- 解析

def test_parse_phase_reads_all_four_fields():
    p = commands.parse_phase("tilt:2.5:1.05:0,0.3,0")
    assert p.name == "tilt"
    assert p.duration_s == 2.5
    assert p.thrust_scale == 1.05
    assert p.omega_flu == (0.0, 0.3, 0.0)


def test_parse_program_keeps_order():
    assert [p.name for p in PROGRAM] == ["lift", "hover"]


@pytest.mark.parametrize("spec", [
    "lift:1.2:1.2",                 # 少一段
    "lift:1.2:1.2:0,0,0:extra",     # 多一段
    "lift:1.2:1.2:0,0",             # ω 只有两个
    "lift:1.2:1.2:a,b,c",           # ω 不是数
    "lift:abc:1.2:0,0,0",           # 时长不是数
    ":1.2:1.2:0,0,0",               # 没名字
    "lift:0:1.2:0,0,0",             # 时长必须为正
    "lift:1.2:-1:0,0,0",            # 推力倍数不能为负
    # ★ 下面四个是"光写 `<= 0` 拦不住"的那一类：`nan <= 0` 与 `inf <= 0` **都是假**。
    #   `nan` 时长尤其阴：`command_at` 里 `t < elapsed + nan` 恒假 ⟹ 那一段永远轮不到，
    #   `elapsed += nan` 之后整条程序塌成最后一段 —— 飞机从第一拍起就在执行**另一套**指令，
    #   而日志、报告、天上的姿态全都"正常"。本模块宁可在这里炸掉。
    "lift:nan:1.2:0,0,0",           # 时长 nan
    "lift:inf:1.2:0,0,0",           # 时长 inf
    "lift:1.2:nan:0,0,0",           # 推力倍数 nan
    "lift:1.2:1.2:nan,0,0",         # 角速率 nan
])
def test_bad_specs_raise(spec):
    with pytest.raises(ValueError):
        commands.parse_phase(spec)


def test_bad_specs_say_which_rule_they_broke():
    """坏 spec 的**话术**也是接口的一部分：`nan` 是能解析、但不许用的值。

    ☠️ 这条是复核逮到的：`Phase(...)` 的构造原本裹在 `except ValueError` 里（那是给
    `float()` 用的），于是 `__post_init__` 那些**具体的**校验话术（"必须是有限数"）
    被替换成一句"时长/倍数解析不了" —— 明明解析得了。查故障时那句话会把人往错的
    方向带（去找拼写错误，而真正的问题是发了一条 `nan` 指令）。
    """
    with pytest.raises(ValueError) as excinfo:
        commands.parse_phase("lift:nan:1.2:0,0,0")
    msg = str(excinfo.value)
    assert "有限数" in msg and "解析不了" not in msg
    # 阳性对照：真正解析不了的时候，反过来说的才是"解析不了"
    with pytest.raises(ValueError) as excinfo:
        commands.parse_phase("lift:abc:1.2:0,0,0")
    assert "解析不了" in str(excinfo.value)


def test_nan_duration_would_collapse_the_program():
    """把上一条的**后果**钉住：没有那层校验时，`nan` 时长本身就是一条静默错误。

    （这里用等价的正常段拼出"塌成最后一段"的样子，说明判据为什么值得存在：
    整条时间轴只剩一段在发 —— 起飞段凭空消失，而任何"发了多少帧/多大推力"的报告都正常。）
    """
    with pytest.raises(ValueError):
        commands.parse_program(["lift:nan:1.2:0,0,0", "hover:8:1.0:0,0,0"])
    # 对照：时长合法时，t=0.5 s 落在**第一段**（1.2×悬停），不是最后一段
    assert commands.command_at(PROGRAM, 0.5, HOVER_N)[0] == pytest.approx(1.2 * HOVER_N)


def test_empty_program_is_an_error():
    """空程序 = 不发 setpoint = 断流实验，不是本用法 —— 必须炸，不能被当成"无声地什么都不做"。"""
    with pytest.raises(ValueError):
        commands.parse_program([])


@pytest.mark.parametrize("mass", [0.0, -1.0, float("nan"), float("inf")])
def test_bad_mass_raises(mass):
    """`nan` 质量也会一路穿到推力上（`nan * g = nan`）—— 同样在源头拦。"""
    with pytest.raises(ValueError):
        commands.hover_force_n(mass)


# ---------------------------------------------------------------- 时间轴

def test_liftoff_force_is_scale_times_hover():
    f, omega = commands.command_at(PROGRAM, 0.5, HOVER_N)
    assert f == pytest.approx(1.2 * HOVER_N)
    assert omega == (0.0, 0.0, 0.0)
    # 阳性对照：同一时刻如果真是"悬停段"，力就不该是这个值
    assert f != pytest.approx(1.0 * HOVER_N)


def test_boundary_switches_to_the_next_phase():
    """边界属于**下一段**（`t < elapsed + duration`）—— 差一帧就换段，这里把口径钉死。"""
    assert commands.command_at(PROGRAM, 1.1999, HOVER_N)[0] == pytest.approx(1.2 * HOVER_N)
    assert commands.command_at(PROGRAM, 1.2, HOVER_N)[0] == pytest.approx(1.0 * HOVER_N)


def test_last_phase_is_held_past_the_end():
    """走完**不停发**：停发会触发 PX4 的 offboard 丢失保护，而那是断流实验专门要单独观测的事。"""
    assert commands.command_at(PROGRAM, 60.0, HOVER_N)[0] == pytest.approx(1.0 * HOVER_N)
    # 阳性对照：确实"过了总时长"（9.2 s），不是还落在 hover 段里
    assert 60.0 > sum(p.duration_s for p in PROGRAM)


def test_negative_time_gives_the_first_phase():
    """`t<0` 只可能来自时钟抖动；取第一段而不是崩掉 —— 但绝不允许它悄悄跳过起飞段。"""
    assert commands.command_at(PROGRAM, -0.01, HOVER_N) == commands.command_at(PROGRAM, 0.0, HOVER_N)


def test_omega_is_passed_through_untouched():
    """本模块**不做**符号映射、也不做归一化 —— 那是 `sign_map` 的事（分开才能分别验）。"""
    program = commands.parse_program(["tilt:1:1.0:0.1,-0.2,0.3"])
    f, omega = commands.command_at(program, 0.5, HOVER_N)
    assert omega == (0.1, -0.2, 0.3)
    assert f == pytest.approx(HOVER_N)


def test_describe_lists_every_phase_with_start_times():
    text = commands.describe(PROGRAM, HOVER_N)
    assert "lift" in text and "hover" in text
    assert "0.0s" in text and "1.2s" in text          # 两段的起始秒
    assert f"{1.2 * HOVER_N:.3f}N" in text            # 起飞段的力
    assert text.index("lift") < text.index("hover")   # 顺序即时间顺序


def test_phase_is_frozen():
    """段是不可变的：取证时改一段会静默改掉别的实验的基准。"""
    p = Phase("x", 1.0, 1.0, (0.0, 0.0, 0.0))
    with pytest.raises(Exception):
        p.duration_s = 2.0
