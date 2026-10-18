"""`sign_map_core` 的自检 —— **不需要飞机、不需要 ROS、不需要通电**。

```bash
python3 tools/sign-map/test_sign_map_core.py --self-test
python3 -m pytest tools/sign-map/test_sign_map_core.py -q
```

三类判据：

| 类 | 判的是什么 | 为什么必须有 |
|---|---|---|
| **推导 vs 文档** | 独立推导出的符号 == 权威文档里的 `sign-map:canonical` 块 | 抓"文档被改成另一个口径而没人发现" |
| **物理闭环** | 同一条指令走 ENU/FLU 与 NED/FRD 两条路，世界系加速度必须相同 | ★ 抓"符号表自洽、但物理方向相反"—— 前一类**抓不到**这种 |
| **反例钉死** | 故意把某个符号写反 ⟹ 断言飞机真的朝反方向/砸地 | 证明这些错法**真的会摔机**，而不是"看着别扭" |

外部仓库（`~/refs/px4-mpc`、`~/refs/px4_msgs`）不在场时**打印 SKIP 并仍然通过** ——
但 SKIP 是可见的，不会伪装成通过。
"""

from __future__ import annotations

import re
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))

from sign_map_core import (  # noqa: E402
    DEFAULT_DOC,
    accel_enu_from_flu,
    accel_enu_from_px4,
    derived_signs,
    enu_to_ned_matrix,
    flu_to_frd_matrix,
    parse_canonical_block,
    rates_flu_to_frd,
    read_px4_mpc_signs,
    read_px4_msgs_thrust_comment,
    Skip,
    thrust_body_2,
)

MASS = 0.75          # z0 实测质量（kg）；本自检只用它保持量纲自洽，数值本身不影响符号判定
G = 9.81
T_MAX = 11.8         # z0 电池限制的推力上限（N）；最终值由 ticket 10 定稿
DT = 1e-3            # 积分小步长


def _hover_F():
    return MASS * G


# ---------------------------------------------------------------- 一、定义与推导


def test_basis_changes_are_proper_rotations():
    """★ 「FLU→FRD 是绕 x 转 180°，不是镜像」这个论断，就钉在这两个行列式上。

    若哪天真成了镜像（det = −1），角速度作为轴矢量就要多一个负号 —— 那是另一张表。
    """
    assert np.isclose(np.linalg.det(enu_to_ned_matrix()), 1.0)
    assert np.isclose(np.linalg.det(flu_to_frd_matrix()), 1.0)
    # 自逆：来回换两次要回到原处
    S = flu_to_frd_matrix()
    assert np.allclose(S @ S, np.eye(3))


def test_rates_mapping_does_not_mix_axes():
    """三轴各喂单位角速率，落到 FRD 后**只该有一根轴非零** —— 纯符号翻转，无交叉项。"""
    for i in range(3):
        e = np.zeros(3)
        e[i] = 1.0
        w = rates_flu_to_frd(e)
        assert np.count_nonzero(np.abs(w) > 1e-12) == 1
        assert np.isclose(abs(w[i]), 1.0)


def test_derived_signs_match_authoritative_doc():
    """独立推导 == 权威文档。不一致就**停**：这说明文档与推导有一个是错的。

    ⚠️ 第三行那个**字面量是故意写死的，不是"忘了回链"**（别顺手删）。
    `sign-map.md` §8.4 要求代码不得复写数值，指的是**实现模块**；测试是例外，理由如下：
    前两行只证明"推导与文档互相一致"—— 若有人把文档的块和推导矩阵**一起**改反，
    前两行会**同时**通过；第三行是那颗锚，把两者一起漂移的情况钉死。
    （对"非空转"的证明见 `test_harness_catches_mutated_doc`。）
    """
    derived = derived_signs()
    documented = parse_canonical_block(DEFAULT_DOC)
    assert derived == documented, f"推导 {derived} != 文档 {documented}"
    assert derived == {"roll_rate": 1.0, "pitch_rate": -1.0, "yaw_rate": -1.0, "thrust_body_2": -1.0}


def test_thrust_normalisation_is_negative_and_normalised():
    """`thrust_body[2] = −F / T_max`，值域 `[−1, 0]`（向上推力为负）。"""
    assert np.isclose(thrust_body_2(_hover_F(), T_MAX), -_hover_F() / T_MAX)
    assert -1.0 <= thrust_body_2(_hover_F(), T_MAX) < 0.0
    assert np.isclose(thrust_body_2(0.0, T_MAX), 0.0)          # 零推力 ⟹ 0，不是 −0 之外的东西
    assert np.isclose(thrust_body_2(T_MAX, T_MAX), -1.0)       # 拉满 ⟹ −1


# ---------------------------------------------------------------- 二、物理闭环（★ 承重的一条）


def _derived_px4_command(omega_flu, F_n, t_max_n=T_MAX):
    """把 MPC 内部量按**推导出来的**映射变成 PX4 的四个字段。

    这里刻意**不 import 节点里的实现** —— 闭环要能独立成立。
    """
    w = rates_flu_to_frd(omega_flu)
    return dict(roll=w[0], pitch=w[1], yaw=w[2], thrust_body_z=thrust_body_2(F_n, t_max_n))


def test_physics_closure_random_states():
    """★ 主判据：随机姿态变化 + 随机推力，两条约定给出的世界系加速度必须相同。"""
    rng = np.random.default_rng(20260928)
    for _ in range(200):
        omega_flu = rng.uniform(-2.0, 2.0, size=3)
        F_n = float(rng.uniform(0.0, T_MAX))
        a_flu = accel_enu_from_flu(omega_flu, F_n, MASS, DT)
        a_px4 = accel_enu_from_px4(**_derived_px4_command(omega_flu, F_n), t_max_n=T_MAX,
                                   mass_kg=MASS, dt_s=DT)
        assert np.allclose(a_flu, a_px4, rtol=1e-9, atol=1e-9), f"{a_flu} != {a_px4}"


def test_physics_closure_each_axis_alone():
    """三根轴**各自单独**闭环 —— 这样某个轴反了时，失败信息能直接指出是哪一根。"""
    for name, i in (("roll(ω_x)", 0), ("pitch(ω_y)", 1), ("yaw(ω_z)", 2)):
        omega = np.zeros(3)
        omega[i] = 5e-4
        F_n = _hover_F()
        a_flu = accel_enu_from_flu(omega, F_n, MASS, DT)
        a_px4 = accel_enu_from_px4(**_derived_px4_command(omega, F_n), t_max_n=T_MAX,
                                   mass_kg=MASS, dt_s=DT)
        assert np.allclose(a_flu, a_px4, rtol=1e-9, atol=1e-9), f"{name} 两侧不一致：{a_flu} vs {a_px4}"


def test_hover_equilibrium_is_zero_acceleration():
    """悬停配平：零角速率 + 推力 = m·g ⟹ 世界系加速度 ≈ 0（三轴都是）。"""
    a = accel_enu_from_flu(np.zeros(3), _hover_F(), MASS, DT)
    assert np.allclose(a, np.zeros(3), atol=1e-9)


def test_forward_flight_direction_is_plus_x():
    """★ 票面第 1 件事：`ω_y > 0`（绕机体 +y 正向、机头下俯）⟹ 朝 **+x（东）** 加速。"""
    eps = 5e-4
    a = accel_enu_from_flu(np.array([0.0, eps, 0.0]), _hover_F(), MASS, DT)
    assert a[0] > 0, f"应当朝 +x 加速，实际 {a}"
    assert abs(a[1]) < 1e-9 and abs(a[2]) < 1e-9, f"不该有 y/z 分量：{a}"


def test_lateral_flight_direction_is_minus_y():
    """侧飞对照：`ω_x > 0`（绕机体 +x 正向滚转 = 右滚）⟹ 朝 **−y（南）** 加速。

    机头朝东时 +y 是北（左手边），所以"右滚 ⟹ 往南"是对的：
    `R_x(θ)·e_z = (0, −sinθ, cosθ)`，θ>0 把推力拽向 −y。
    """
    a = accel_enu_from_flu(np.array([5e-4, 0.0, 0.0]), _hover_F(), MASS, DT)
    assert a[1] < 0 and abs(a[0]) < 1e-9, f"应当朝 −y 加速，实际 {a}"


def test_general_rule_omega_is_z_cross_d():
    """★ 票面第 1 件事的**一般形式**（航向无关）：`ω = κ·(ẑ × d̂)`，`κ > 0`。

    `d̂` 是期望加速度在**机体系**里的方向（水平分量）；`ẑ` 是机体 +z（推力轴）。
    机头朝东时，机体系方向与世界系 ENU 方向重合，所以直接比即可。
    """
    for angle in np.linspace(0.0, 2.0 * np.pi, 9)[:-1]:
        d_hat = np.array([np.cos(angle), np.sin(angle), 0.0])
        omega = np.cross(np.array([0.0, 0.0, 1.0]), d_hat) * 1e-3
        a = accel_enu_from_flu(omega, _hover_F(), MASS, DT)
        h = a[:2] / np.linalg.norm(a[:2])
        assert np.allclose(h, d_hat[:2], atol=1e-6), f"d̂={d_hat[:2]} 却朝 {h} 加速"


# ---------------------------------------------------------------- 三、反例钉死


def test_counterexample_pitch_not_flipped_flies_backwards():
    """不翻 `pitch` ⟹ 机头**上仰**、朝 **−x** 加速（"越飞越倒退"）。"""
    eps = 5e-4
    omega = np.array([0.0, eps, 0.0])
    F_n = _hover_F()
    a_right = accel_enu_from_flu(omega, F_n, MASS, DT)
    a_wrong = accel_enu_from_px4(roll=omega[0], pitch=+omega[1], yaw=-omega[2],
                                 thrust_body_z=thrust_body_2(F_n, T_MAX),
                                 t_max_n=T_MAX, mass_kg=MASS, dt_s=DT)
    assert a_right[0] > 0
    assert a_wrong[0] < 0, f"符号写反却没飞反：{a_wrong}"


def test_counterexample_thrust_sign_flipped_crashes():
    """推力写成 `+F/T_max` ⟹ 力朝机腹（FRD 的 +z）⟹ **全速砸地**。"""
    F_n = _hover_F()
    a_wrong = accel_enu_from_px4(roll=0.0, pitch=0.0, yaw=0.0,
                                 thrust_body_z=-thrust_body_2(F_n, T_MAX),
                                 t_max_n=T_MAX, mass_kg=MASS, dt_s=DT)
    assert a_wrong[2] < -0.5 * G, f"符号写反却没有明显下坠：{a_wrong}"


def test_counterexample_yaw_not_flipped_turns_the_wrong_way():
    """不翻 `yaw` ⟹ 同样的指令会**朝反方向偏航**。"""
    eps = 5e-4
    omega = np.array([0.0, 0.0, eps])
    F_n = _hover_F()
    a_right = accel_enu_from_flu(omega, F_n, MASS, DT)
    a_wrong = accel_enu_from_px4(roll=-omega[0], pitch=-omega[1], yaw=+omega[2],
                                 thrust_body_z=thrust_body_2(F_n, T_MAX),
                                 t_max_n=T_MAX, mass_kg=MASS, dt_s=DT)
    # 偏航本身不产生平移加速度，改看**机头转向**：世界系里机体 x 轴该往哪转。
    assert np.allclose(a_right, a_wrong, atol=1e-6), "偏航不该产生平移加速度（两条路都一样）"
    # 用一条独立的姿态积分把"转向"判出来
    from sign_map_core import rot_from_rotvec
    R_right = rot_from_rotvec(omega * DT)                       # ENU/FLU
    R0 = enu_to_ned_matrix() @ np.eye(3) @ flu_to_frd_matrix()
    R_wrong = R0 @ rot_from_rotvec(np.array([0.0, 0.0, +eps]) * DT)   # PX4，yaw 没翻
    nose_right = np.array([R_right[0, 0], R_right[1, 0]])
    nose_wrong_ned = R_wrong @ np.array([1.0, 0.0, 0.0])
    nose_wrong = np.array([nose_wrong_ned[1], nose_wrong_ned[0]])     # NED → ENU
    # 右侧：机头朝 +y 转（左转）；写反：朝 −y 转
    assert nose_right[1] > 0 and nose_wrong[1] < 0, f"{nose_right} / {nose_wrong}"


# ---------------------------------------------------------------- 四、与外部仓库对账


def test_authoritative_doc_block_is_parseable():
    """权威块读得出、且恰好四个符号 —— 改坏了这里立刻红。"""
    assert set(parse_canonical_block(DEFAULT_DOC)) == {
        "roll_rate", "pitch_rate", "yaw_rate", "thrust_body_2"}


def test_matches_reference_implementation_source():
    signs = read_px4_mpc_signs()
    assert signs == derived_signs(), f"参考实现 {signs} != 推导 {derived_signs()}"


def test_matches_px4_msgs_comment():
    assert read_px4_msgs_thrust_comment() == "negative throttle demand"


# ---------------------------------------------------------------- 五、判据本身不是空转


def test_harness_catches_mutated_doc():
    """★ 把权威文档里的一个符号改反，"推导 == 文档"这条**必须**红。

    没有这一条，就分不清"对账通过"与"对账根本没在看"。
    """
    import tempfile

    text = DEFAULT_DOC.read_text(encoding="utf-8")
    mutated = re.sub(r"(pitch_rate\s*=\s*)-1", r"\g<1>+1", text)
    assert mutated != text, "变异没生效（权威块的写法变了？）"
    with tempfile.NamedTemporaryFile("w", suffix=".md", delete=False, encoding="utf-8") as fh:
        fh.write(mutated)
        tmp = Path(fh.name)
    try:
        assert parse_canonical_block(tmp) != derived_signs()
    finally:
        tmp.unlink()


def test_harness_catches_mutated_map():
    """★ 把映射改错（`pitch` 不翻），物理闭环**必须**红 —— 证明闭环判据不是空转。"""
    omega = np.array([0.0, 5e-4, 0.0])
    F_n = _hover_F()
    right = accel_enu_from_flu(omega, F_n, MASS, DT)
    wrong = accel_enu_from_px4(roll=omega[0], pitch=+omega[1], yaw=-omega[2],
                               thrust_body_z=thrust_body_2(F_n, T_MAX),
                               t_max_n=T_MAX, mass_kg=MASS, dt_s=DT)
    assert not np.allclose(right, wrong, rtol=1e-9, atol=1e-9)


# ---------------------------------------------------------------- 跑法

_TESTS = [
    ("定义与推导 · 基变换是真旋转（不是镜像）", test_basis_changes_are_proper_rotations),
    ("定义与推导 · 角速率映射无交叉项", test_rates_mapping_does_not_mix_axes),
    ("定义与推导 · 推导的符号 == 权威文档", test_derived_signs_match_authoritative_doc),
    ("定义与推导 · 推力归一化 −F/T_max", test_thrust_normalisation_is_negative_and_normalised),
    ("物理闭环 · 随机状态两条约定一致", test_physics_closure_random_states),
    ("物理闭环 · 三根轴各自单独闭环", test_physics_closure_each_axis_alone),
    ("物理闭环 · 悬停配平加速度为零", test_hover_equilibrium_is_zero_acceleration),
    ("物理闭环 · 前飞方向 = +x（东）", test_forward_flight_direction_is_plus_x),
    ("物理闭环 · 侧飞方向 = −y（南）", test_lateral_flight_direction_is_minus_y),
    ("物理闭环 · 一般规则 ω = ẑ × d̂", test_general_rule_omega_is_z_cross_d),
    ("反例钉死 · pitch 不翻 ⟹ 倒着飞", test_counterexample_pitch_not_flipped_flies_backwards),
    ("反例钉死 · 推力符号反 ⟹ 砸地", test_counterexample_thrust_sign_flipped_crashes),
    ("反例钉死 · yaw 不翻 ⟹ 反着偏航", test_counterexample_yaw_not_flipped_turns_the_wrong_way),
    ("对账 · 权威块可解析", test_authoritative_doc_block_is_parseable),
    ("对账 · 与 ~/refs/px4-mpc 源码逐条一致", test_matches_reference_implementation_source),
    ("对账 · 与 ~/refs/px4_msgs 注释一致", test_matches_px4_msgs_comment),
    ("非空转 · 文档被改反时对账会红", test_harness_catches_mutated_doc),
    ("非空转 · 映射被改错时闭环会红", test_harness_catches_mutated_map),
]


def main():
    if "--self-test" not in sys.argv:
        print("用法：python3 tools/sign-map/test_sign_map_core.py --self-test")
        return 2
    print(f"权威文档：{DEFAULT_DOC}")
    print(f"（存在：{DEFAULT_DOC.exists()}）\n")
    passed = skipped = failed = 0
    for name, fn in _TESTS:
        try:
            fn()
        except Skip as exc:
            skipped += 1
            print(f"  SKIP  {name}  —— {exc}")
        except Exception as exc:  # noqa: BLE001 —— 自检要的是"全过或全红"，不挑异常类型
            failed += 1
            print(f"  FAIL  {name}\n        {type(exc).__name__}: {exc}")
        else:
            passed += 1
            print(f"  ok    {name}")
    print(f"\n{passed} passed, {skipped} skipped, {failed} failed")
    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(main())
