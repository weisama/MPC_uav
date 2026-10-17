#!/usr/bin/env python3
"""票 01（Phase 5）：数据提取与对齐管线的**口径**测试。

这里测的不是控制器，而是"四源能不能相减"的那几条约定：

| 组 | 回答的问题 | 数据 |
|---|---|---|
| 合成用例 | 原点锁存早于 `/traj/start`、`target = origin + reference`、`progress >= 1.0` 截断、四元数往返、公共窗取最小 | 手造 |
| 黄金值 | 六份包的 `/traj/start`、原点、首帧参考、完成时刻 | `bag/` 实录 |
| 变异自检 | 把原点换成 `/traj/start` 时刻的位置、把截断点移到包尾，黄金值断言**会红** | `bag/` 实录 |
| 仿真 | 三趟 SITL 的 `x0` 四元数、参考时间线、与包的参考逐字一致 | `tools/report/evidence/sitl/` |

★ 黄金值测试需要能 `import rosbag2_py`（先 `source /opt/ros/humble/setup.bash`）；
  取不到就 **skip 并写明原因**，不假装通过。合成用例与仿真摘要测试不需要 ROS。
"""

from __future__ import annotations

import json
import math
import sys
from pathlib import Path

import numpy as np
import pytest

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[1]
sys.path.insert(0, str(HERE))

from align import (  # noqa: E402
    absolute_target,
    build_grid,
    common_end,
    completion_from_progress,
    completion_from_reference,
    euler_to_quat,
    find_state_window,
    origin_from_command,
    position_at,
    quat_to_euler,
    resample_linear,
    resample_quat,
    resample_zoh,
)

STATE_WAIT = "AUTO/WAIT_START"
STATE_TRACK = "AUTO/TRACK"


def _ros_available():
    try:
        import rclpy.serialization  # noqa: F401
        import rosidl_runtime_py.utilities  # noqa: F401
        return True
    except Exception:
        return False


needs_ros = pytest.mark.skipif(
    not _ros_available(),
    reason="读包需要 ROS 2 Python（先 source /opt/ros/humble/setup.bash）")


# ─────────────────────────── 合成用例 ───────────────────────────


def test_origin_latches_at_wait_start_not_at_traj_start():
    """原点取 `AUTO/WAIT_START` 期间的指令值，**不是** `/traj/start` 时刻的飞机位置。

    这是 spec 的核心口径：FSM 更早锁存原点，两者实测差 6.5–10.5 cm。
    合成数据把差值放大到 0.2 m，确认函数读的是指令、不是位置。
    """
    t = np.array([-2.0, 0.0, 2.0, 4.0])
    states = np.array([STATE_WAIT, STATE_WAIT, STATE_TRACK, STATE_TRACK])
    wait = find_state_window(t, states, STATE_WAIT)
    assert wait == (-2.0, 2.0)

    cmd_t = np.array([-3.0, -2.0, 0.0, 1.9, 2.1, 4.0])
    cmd_p = np.array([[1.0, 1.0, 1.0]] * 4 + [[1.2, 1.2, 1.2]] * 2)
    origin, spread = origin_from_command(cmd_t, cmd_p, wait)
    assert np.allclose(origin, [1.0, 1.0, 1.0])
    assert spread < 1e-12

    # 飞机在 /traj/start 时刻已经飘到 1.2 m；拿它当原点就是那笔假偏差。
    pos_t = np.array([0.0, 4.0])
    pos_p = np.array([[1.2, 1.2, 1.2], [1.2, 1.2, 1.2]])
    assert np.linalg.norm(position_at(pos_t, pos_p, 2.0) - origin) > 0.19


def test_origin_rejects_non_constant_wait_start_command():
    """WAIT_START 期间指令不恒定 ⟹ 当场失败，不取均值。"""
    t = np.array([-2.0, 0.0, 2.0])
    states = np.array([STATE_WAIT, STATE_WAIT, STATE_TRACK])
    wait = find_state_window(t, states, STATE_WAIT)
    cmd_t = np.array([-1.0, 0.0, 1.0])
    cmd_p = np.array([[1.0, 0.0, 0.0], [1.1, 0.0, 0.0], [1.2, 0.0, 0.0]])
    with pytest.raises(ValueError, match="不恒定"):
        origin_from_command(cmd_t, cmd_p, wait)


def test_origin_falls_back_to_local_position_pose():
    """指令话题缺失时，原点回退到首个 `AUTO/WAIT_START` 时刻的位姿。"""
    from extract import _origin_from_pose

    t = np.array([0.5, 1.5, 2.5])
    p = np.array([[1.0, 2.0, 3.0], [4.0, 5.0, 6.0], [7.0, 8.0, 9.0]])
    origin, spread, why = _origin_from_pose(t, p, (1.0, 2.5))
    assert np.allclose(origin, [4.0, 5.0, 6.0])   # 首个 ≥ 窗口起点的样本
    assert spread == 0.0
    assert "回退" in why

    # 窗口起点早于所有样本：顶住端点，不抛错、也不外推。
    origin, _spread, _why = _origin_from_pose(t, p, (0.0, 0.2))
    assert np.allclose(origin, [1.0, 2.0, 3.0])


def test_target_is_origin_plus_relative_reference():
    """绝对目标 = 原点 + `/traj/reference`（相对量）；这条关系只在 TRACK 窗内成立。"""
    origin = np.array([0.3, -0.2, 1.0])
    ref = np.array([[0.0, 0.0, 0.0], [1.0, 0.5, 0.0]])
    target = absolute_target(origin, ref)
    assert target.shape == (2, 3)
    assert np.allclose(target[0], origin)
    assert np.allclose(target[-1], [1.3, 0.3, 1.0])
    with pytest.raises(ValueError, match="n, 3"):
        absolute_target(origin, np.array([0.0, 0.0, 0.0]))


def test_completion_truncates_at_first_progress_one():
    """完成时刻 = **首个** `progress >= 1.0`；之后参考继续发零，不能拿末帧判完成。"""
    t = np.array([0.0, 5.0, 8.0, 12.0, 20.0])
    p = np.array([0.0, 0.61, 1.0, 1.0, 1.0])
    end, reason = completion_from_progress(t, p, t0=0.0)
    assert end == pytest.approx(8.0)
    assert reason == "progress"


def test_completion_without_progress_one_uses_reference_end():
    """悬停包没有 1.0（录制截断）⟹ 截止到参考流末尾，并写明理由。"""
    t = np.array([0.01, 20.0, 39.6501])
    end, reason = completion_from_reference(t, t0=0.0)
    assert end == pytest.approx(39.6501)
    assert reason == "ref_end"


def test_common_window_takes_minimum():
    """公共窗取各源完成时刻的最小值 —— 悬停由 39.64 s 的仿真/包尾决定。"""
    assert common_end({"target": 39.6501, "offboard": 39.6501,
                       "mpc_sim": 39.6417, "mpc_real": 60.0163}) == pytest.approx(39.6417)
    assert common_end({"target": 8.0213, "offboard": 8.0213,
                       "mpc_sim": 17.8832, "mpc_real": 8.0096}) == pytest.approx(8.0096)


def test_quaternion_euler_roundtrip_below_1e9():
    """四元数 ↔ 欧拉角往返误差 < 1e-9 rad（票面检查清单）。"""
    rng = np.random.default_rng(20261017)
    q = rng.normal(size=(2000, 4))
    q /= np.linalg.norm(q, axis=1, keepdims=True)
    # 统一为 w >= 0，避开 q 与 -q 同姿态带来的符号歧义。
    q[q[:, 0] < 0] *= -1.0
    e = quat_to_euler(q)
    q2 = euler_to_quat(e)
    q2[q2[:, 0] < 0] *= -1.0
    err = np.abs(q - q2).max()
    assert err < 1e-9, f"往返误差 {err:.3e} rad"


def test_linear_resample_holds_before_first_sample():
    """参考流比 `/traj/start` 晚 10 ms；网格 t=0 处**顶住首帧**，不外推出假值。"""
    t = np.array([0.0101, 0.0301, 0.0501])
    x = np.array([[0.005, 0.0, 0.0], [0.015, 0.0, 0.0], [0.025, 0.0, 0.0]])
    grid = build_grid(0.06, hz=100.0)
    out = resample_linear(t, x, grid)
    assert np.allclose(out[0], x[0])       # t=0 顶住首帧
    assert np.allclose(out[1], x[0])       # t=0.01 仍在首帧之前
    # t=0.03 落在 0.0101 与 0.0301 之间：0.005 + (0.03-0.0101)/0.02 × 0.01
    assert out[3, 0] == pytest.approx(0.01495, abs=1e-12)


def test_zoh_keeps_command_between_ticks():
    """控制量是离散指令：用零阶保持，不做线性插值（插值会造出没发过的值）。"""
    t = np.array([0.0, 0.05, 0.10])
    u = np.array([[7.0], [8.0], [9.0]])
    grid = np.array([0.0, 0.02, 0.049, 0.05, 0.09])
    assert np.allclose(resample_zoh(t, u, grid)[:, 0], [7.0, 7.0, 7.0, 8.0, 8.0])


def test_quaternion_resample_uses_slerp_and_shortest_arc():
    """四元数 slerp：跨 ±180° 时走短弧，不出现符号翻转造成的假跳变。"""
    q = np.array([[1.0, 0.0, 0.0, 0.0],
                  [-1.0, 0.0, 0.0, 0.0]])   # 同一姿态，符号相反
    t = np.array([0.0, 1.0])
    out = resample_quat(t, q, np.array([0.5]))
    assert np.allclose(np.abs(out[0]), [1.0, 0.0, 0.0, 0.0], atol=1e-12)


def test_resample_refuses_short_series():
    with pytest.raises(ValueError, match="至少"):
        resample_linear(np.array([0.0]), np.array([[0.0, 0.0, 0.0]]),
                        np.array([0.0, 0.1]))


# ─────────────────────────── 指标（合成） ───────────────────────────


def test_position_error_splits_horizontal_and_vertical():
    """水平误差取 xy 范数、垂直取 |z|，不能合成一个数（报告里分开讲）。"""
    from metrics import position_error

    target = np.zeros((2, 3))
    actual = np.array([[3.0, 4.0, 0.0], [0.0, 0.0, -2.0]])
    m = position_error(target, actual)
    assert m["h_mean_m"] == pytest.approx((5.0 + 0.0) / 2)
    assert m["h_max_m"] == pytest.approx(5.0)
    assert m["z_mean_abs_m"] == pytest.approx(1.0)
    assert m["z_max_abs_m"] == pytest.approx(2.0)


def test_attitude_error_separates_tilt_from_yaw():
    """纯偏航 90°：总夹角 90°，但**倾角 0°**、偏航差 90°。

    这三个数必须分开报 —— 只报总夹角会把"倾角小、偏航差大"读成"姿态控制坏了"；
    两者成因不同（倾角用于加速/抗阻，偏航可能是起机朝向），本模块不判定成因。
    """
    from metrics import attitude_error_deg

    q_ref = np.array([[1.0, 0.0, 0.0, 0.0]])
    q_act = euler_to_quat(np.array([[0.0, 0.0, np.pi / 2]]))
    m = attitude_error_deg(q_ref, q_act)
    assert m["angle_mean_deg"] == pytest.approx(90.0, abs=1e-6)
    assert m["tilt"]["max"] == pytest.approx(0.0, abs=1e-6)
    assert m["yaw_diff"]["mean"] == pytest.approx(90.0, abs=1e-6)

    # `q` 与 `-q` 是同一姿态：误差必须是 0，不是 180°。
    assert attitude_error_deg(q_ref, -q_ref)["angle_max_deg"] == pytest.approx(0.0)


def test_control_stats_drops_nan_rows():
    from metrics import control_stats

    u = np.array([1.0, np.nan, 3.0])
    rate = np.array([[0.0, 0.0, 0.0], [9.0, 9.0, 9.0], [3.0, 4.0, 0.0]])
    m = control_stats(u, rate)
    assert m["n"] == 2
    assert m["thrust_max_n"] == pytest.approx(3.0)
    assert m["rate_norm_max_radps"] == pytest.approx(5.0)


# ─────────────────────────── 仿真摘要（无 ROS） ───────────────────────────

SIM = REPO / "tools" / "report" / "evidence" / "sitl"


def _summary(tag):
    return json.loads((SIM / tag / f"{tag}-summary.json").read_text(encoding="utf-8"))


@pytest.mark.parametrize("tag,span,ref_start_wall,ref_delay_s", [
    ("R-hover", 39.64167761802673, 1792246130.8921504, 0.007737636566162109),
    ("R-square", 17.88318920135498, 1792245962.7535696, 0.009085655212402344),
    ("R-lemniscate", 26.22361469268799, 1792246042.5870135, 0.010177850723266602),
])
def test_sim_reference_window_starts_after_pre_hold(tag, span, ref_start_wall,
                                                    ref_delay_s):
    """仿真 t=0 = 回放 `bag` 段起点 − 参考发布延迟。

    `bag` 段起点 = 参考首帧 + `ref_hold_s`；`pre` 段是**我们**加的（真实那场没有前摇）。
    `ref_delay_s` 是该场景实飞包里「首条 `/traj/reference` 的 header −
    `/traj/start` 的记录时间」，7.7–10.2 ms，与 spec §3 的 7.7–15.6 ms 同量级。
    """
    from extract import sim_reference_start

    d = _summary(tag)
    t0, why = sim_reference_start(d, ref_delay_s)
    assert t0 == pytest.approx(ref_start_wall - ref_delay_s, abs=1e-6)
    assert d["ref_hold_s"] == 30.0
    assert why
    assert d["reference_source"] == "traj" and d["mismatch"] == "none"
    # 仿真源自己的完成时刻：参考末帧（模型 span）+ 发布延迟；公共窗要跟实飞取最小。
    model = json.loads((SIM / "models" / f"{tag}-model.json").read_text(encoding="utf-8"))
    assert model["reference"]["span_s"] == pytest.approx(span, abs=1e-9)


@needs_ros
@pytest.mark.parametrize("key,delay", [
    ("hover_offboard", 0.007737636566162109),
    ("square_offboard", 0.009085655212402344),
    ("lemniscate_offboard", 0.010177850723266602),
])
def test_reference_publication_delay_matches_sim_anchor(extracts, key, delay):
    """仿真 t=0 用的延迟必须等于实飞包里量出来的「首条参考 header − /traj/start」。"""
    e = extracts[key]
    assert e.ref_first_stamp_abs - e.t0_abs == pytest.approx(delay, abs=1e-6)


@pytest.mark.parametrize("tag", ["R-hover", "R-square", "R-lemniscate"])
def test_sim_checks_have_no_false(tag):
    """三趟 nominal SITL 的判据里没有一个 `ok=False`（票 02 的交接条件）。"""
    d = _summary(tag)
    bad = [c["name"] for c in d["checks"] if c["ok"] is False]
    assert bad == [], bad


@pytest.mark.parametrize("tag,bag", [
    ("R-hover", "bag/3_traj1_hover"),
    ("R-square", "bag/4_traj2_square"),
    ("R-lemniscate", "bag/5_traj3_lemniscate"),
])
def test_sim_reference_envelope_matches_scene(tag, bag):
    """模型参考的包络与场景一致（形状与运动范围）。"""
    model = json.loads((SIM / "models" / f"{tag}-model.json").read_text(encoding="utf-8"))
    assert model["bag"] == bag
    ref = np.asarray(model["reference"]["p"], dtype=float)
    # 形状与量级：三个场景的包络都与报告里写死的一致。
    assert ref.shape[1] == 3 and len(ref) == model["reference"]["n"]
    if tag == "R-hover":
        assert np.allclose(ref, 0.0)
    elif tag == "R-square":
        assert np.isclose(ref[:, :2].max(), 1.0, atol=2e-3)
        assert np.isclose(ref[:, :2].min(), 0.0, atol=2e-3)
    else:
        assert np.isclose(ref[:, 0].max(), 1.0, atol=2e-3)
        assert np.isclose(ref[:, 1].max(), 0.5, atol=2e-3)


def test_sim_quaternion_is_wxyz():
    """`x0[6:10]` 是 `[w, x, y, z]`（与 `model.py:70` 一致）—— 当成 xyzw 会读出 90° 假滚转。"""
    d = _summary("R-square")
    f = next(f for f in d["frames"] if f["x0"] is not None)
    q = np.asarray(f["x0"][6:10], dtype=float)
    e_wxyz = quat_to_euler(q[None, :])[0]
    e_xyzw = quat_to_euler(q[[3, 0, 1, 2]][None, :])[0]
    assert abs(e_wxyz[0]) < 0.1 and abs(e_wxyz[1]) < 0.1
    assert abs(e_xyzw[0]) > 1.0 or abs(e_xyzw[1]) > 1.0


# ─────────────────────────── 黄金值 / 变异（需要 ROS 读包） ───────────────────────────

BAGS = {
    "hover_offboard": ("bag/3_traj1_hover", 1791452468.0047069,
                       (-0.05775393545627594, -0.05549931153655052, 1.0115019083023071),
                       39.65007972717285, "ref_end"),
    "square_offboard": ("bag/4_traj2_square", 1791452671.9687545,
                        (-0.08052615821361542, -0.028158580884337425, 1.0096334218978882),
                        8.021292209625244, "progress"),
    "lemniscate_offboard": ("bag/5_traj3_lemniscate", 1791454734.7870278,
                            (-0.04193028807640076, -0.03807908296585083, 0.9904073476791382),
                            12.213040828704834, "progress"),
    "hover_mpc": ("bag/6_free__20261017-070103", 1792220485.6721663,
                  (-0.06903056055307388, -0.0308650154620409, 1.0866360664367676),
                  60.0162935256958, "progress"),
    "square_mpc": ("bag/6_free__20261017-070446", 1792220707.6739805,
                   (-0.002021434251219034, -0.10534180700778961, 1.157146692276001),
                   8.009598731994629, "progress"),
    "lemniscate_mpc": ("bag/6_free__20261017-070935", 1792221003.3192825,
                       (-0.032465845346450806, -0.02981523610651493, 1.1866655349731445),
                       12.202110528945923, "progress"),
}


@pytest.fixture(scope="module")
def extracts(tmp_path_factory):
    if not _ros_available():
        pytest.skip("读包需要 ROS 2 Python")
    from extract import read_offboard_bag, read_mpc_bag

    work = tmp_path_factory.mktemp("bags")
    out = {}
    for key, (bag, *_rest) in BAGS.items():
        bag_dir = REPO / bag
        fn = read_mpc_bag if "mpc" in key else read_offboard_bag
        out[key] = fn(bag_dir, work / key)
    return out


@needs_ros
@pytest.mark.parametrize("tag,key", [
    ("R-hover", "hover_offboard"),
    ("R-square", "square_offboard"),
    ("R-lemniscate", "lemniscate_offboard"),
])
def test_sim_reference_is_verbatim_from_bag(extracts, tag, key):
    """仿真参考的**值**逐字取自对应包的 `/traj/reference`（票 02 的“逐字”承诺）。

    比位置、速度、偏航；时间轴不比 —— 模型参考用 header 时间戳，包里存的是
    记录时间戳，两者差的是发布延迟（另有用例专门量它）。
    """
    model = json.loads((SIM / "models" / f"{tag}-model.json").read_text(encoding="utf-8"))
    ref = model["reference"]
    e = extracts[key]
    assert ref["n"] == e.ref_p.shape[0]
    assert np.array_equal(np.asarray(ref["p"], dtype=float), e.ref_p)
    assert np.array_equal(np.asarray(ref["v"], dtype=float), e.ref_v)
    assert np.array_equal(np.asarray(ref["yaw"], dtype=float), e.ref_yaw)


@needs_ros
@pytest.mark.parametrize("key", list(BAGS))
def test_golden_time_zero(extracts, key):
    bag, t0, *_ = BAGS[key]
    assert extracts[key].t0_abs == pytest.approx(t0, abs=1e-6)


@needs_ros
@pytest.mark.parametrize("key", list(BAGS))
def test_golden_origin_millimetre(extracts, key):
    """原点与 2026-10-17 探针值一致（mm 级）—— 它必须来自 WAIT_START 的指令值。"""
    _, _, origin, _c, _r = BAGS[key]
    got = extracts[key].origin_abs
    assert np.allclose(got, origin, atol=1e-3), (got, origin)
    # WAIT_START 期间指令恒定这一条也要有锚：散布不是"大概很小"。
    assert extracts[key].origin_spread_m < 1e-6, extracts[key].origin_spread_m
    expected_topic = ("/mavros/setpoint_raw/local" if "offboard" in key
                      else "/control/target")
    assert extracts[key].origin_source == expected_topic


@needs_ros
@pytest.mark.parametrize("key", list(BAGS))
def test_golden_completion(extracts, key):
    _, _, _o, end, reason = BAGS[key]
    assert extracts[key].completion_s == pytest.approx(end, abs=0.02)
    assert extracts[key].completion_reason == reason


@needs_ros
@pytest.mark.parametrize("key", list(BAGS))
def test_mutation_origin_at_traj_start_exceeds_golden_tolerance(extracts, key):
    """变异自检：把原点换成 `/traj/start` 时刻的位置，黄金值的 mm 级断言一定红。"""
    e = extracts[key]
    wrong = position_at(e.actual_t, e.actual_p_abs, e.t0_abs)
    assert np.linalg.norm(wrong - e.origin_abs) > 0.06, (
        f"{key}: 位置原点与指令原点只差 {np.linalg.norm(wrong - e.origin_abs):.3f} m "
        "—— 这条变异就鉴别不了两种口径")


@needs_ros
@pytest.mark.parametrize("key", ["square_offboard", "lemniscate_offboard",
                                "square_mpc", "lemniscate_mpc"])
def test_mutation_completion_at_package_end_differs_by_over_1s(extracts, key):
    """变异自检：把截断点移到包尾，完成时刻断言一定红（差 > 1 s）。"""
    e = extracts[key]
    assert e.package_end_s - e.completion_s > 1.0


def test_committed_offboard_metrics_match_published_baseline():
    """交叉核对：入库 JSON 的 offboard 误差必须对得上验收文档里已发表的基线。

    ★ 这是**独立来源**：`docs/control/ladder-acceptance-criteria.md` §2.2 的读数
      由另一套工具（`tools/ladder-baseline/`，50 Hz 网格、`/mocap/pose_enu`）算出，
      与本管线（100 Hz、`/mavros/vision_pose/pose`）不是同一份代码。
      两边能对上，才说明"原点 + 相对参考"这套口径没有把坐标系搞错。
    """
    published = {
        "hover": (0.1049, 0.1610),
        "square": (0.5682, 0.7552),
        "lemniscate": (0.5886, 0.8160),
    }
    for key, (h_mean, h_max) in published.items():
        path = REPO / "docs" / "report" / "data" / f"{key}.json"
        assert path.is_file(), f"缺 {path}（先跑 build.py --scene all）"
        m = json.loads(path.read_text(encoding="utf-8"))
        pos = m["sources"]["offboard"]["metrics_vs_target"]["position"]
        assert pos["h_mean_m"] == pytest.approx(h_mean, rel=0.03), key
        assert pos["h_max_m"] == pytest.approx(h_max, rel=0.03), key


@needs_ros
def test_hover_offboard_is_truncated_not_completed(extracts):
    """悬停 offboard 包是**录到 TRACK 中途截断**的：progress 从未到 1.0。

    所以这趟的完成时刻只能取参考流末尾，且它离包尾不到 20 ms ——
    "把截断点移到包尾"这一条变异在悬停上是**空操作**，鉴别不了任何东西；
    它与 `progress` 那条变异不是同一类，别拿它自检。
    """
    e = extracts["hover_offboard"]
    assert e.completion_reason == "ref_end"
    assert not np.any(e.progress >= 1.0)
    assert e.package_end_s - e.completion_s < 0.05
