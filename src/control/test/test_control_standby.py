"""control 节点的节点级测试（**seam 1**：只断言话题边界上发生了什么）。

判据全部来自 ticket 05 的验收清单：
  1. 三条守卫（飞控已连接 ∧ 动捕有数据 ∧ local_position 有效）**逐条**缺失 ⟹ 一帧 setpoint 都不发
  2. 守卫成立 ⟹ 50 Hz 持续发 setpoint，且字段正确（type_mask = 2552 / FRAME_LOCAL_NED / 位置 = 当前位姿 / yaw）
  3. 切 OFFBOARD 之前 setpoint 必须已连续流够 offboard_prestream_s
  4. 运行中前置失效 ⟹ **立刻停止**发 setpoint（真机上由 PX4 的 COM_OF_LOSS_T 兜底）
  5. 进程被杀 ⟹ 流停止

★★ 本文件刻意**不** import `control` 的任何类、不读它的成员：DUT 是一个**子进程**。
   理由是验收项 5（"节点被杀之后流停止"）在"import 它的类"的写法里根本表达不出来，
   而"只看外部可观测的话题"这条纪律让测试可以在实现重写之后继续有效。

★ 关于负向断言：本文件每一条"没有发 setpoint"都配了一个**正控**（"修好之后必须发"），
  否则"DUT 启动就崩了"会让所有负向断言假通过。
"""

import pytest

from bench import Scenario
from bench.timeline import silence_for

#: DUT =（包名，可执行名）—— 由 `ament_index_python` 解析成 <prefix>/lib/control/control_node
DUT = ("control", "control_node")

#: 与 `config/control.yaml` 一致的参数。测试里显式写出来，是为了让"这一条判据在什么口径下成立"，
#: 在断言旁边就能读到；改 yaml 时这里也要改（两侧不一致时，README 里说明了以哪个为准）。
BASE_PARAMS = {
    "setpoint_rate_hz": 50.0,
    "enter_offboard": True,
    "offboard_prestream_s": 2.0,
    "offboard_retry_s": 1.0,
    # ★ 2.5 s 而不是 px4ctrl 的 0.5 s：/mavros/state 实测 1 Hz（夹具也照 1 Hz 发）。
    "state_timeout_s": 2.5,
    "local_pos_timeout_s": 0.5,
    "mocap_timeout_s": 0.5,
}


def _reach_standby(sc, min_setpoints: int = 20, timeout_s: float = 15.0):
    """把场景推到"守卫全成立、正在流 setpoint"。

    ★ 这里等的第一件东西不是时间而是**匹配**：`matched_subscribers` 计的是已经匹配上的订阅者，
      于是 QoS 不兼容会在这里立刻失败，而不是几分钟后表现为"莫名其妙没数据"。
    """
    assert sc.wait_for_subscribers("state"), "DUT 没有接上 /mavros/state"
    assert sc.wait_for_subscribers("local_position"), "DUT 没有接上 /mavros/local_position/pose"
    assert sc.wait_for_subscribers("rc"), "DUT 没有接上 /mavros/rc/in"
    assert sc.wait_for_subscribers("battery"), "DUT 没有接上 /mavros/battery"
    assert sc.poll(lambda: sc.mocap_source.matched_subscribers() >= 1), "DUT 没有接上动捕话题"
    assert sc.wait_for_setpoints(min_setpoints, timeout_s=timeout_s), \
        "守卫成立之后应当开始以 50 Hz 流 setpoint"


# ───────────────────────────── 冒烟：进 STANDBY 并正确流 setpoint ─────────────────────────────

def test_冒烟_进_STANDBY_并按_50Hz_发正确的_setpoint():
    with Scenario(dut=DUT, params=BASE_PARAMS) as sc:
        _reach_standby(sc)
        assert sc.control_state() == "MANUAL/STANDBY", (
            f"守卫成立后任务态应为 STANDBY，实际 {sc.control_state()!r}")

        # ★ 位置 = **当前位姿**（不是常量、不是轨迹）：先记下原地保持的那一段，
        #   再把假 EKF 搬到一个三个轴都不同的位置，看 setpoint 是否跟着变。
        #   三轴取不同的值 ⟹ 轴装反/装错会被抓出来（只测一个轴是抓不出来的）。
        first = sc.setpoints()[0]
        assert (first.msg.position.x, first.msg.position.y, first.msg.position.z) == \
            pytest.approx((0.0, 0.0, 0.0))

        sc.fake.set_position(0.12, -0.34, 0.56)
        sc.hold(1.0)

        last = sc.setpoints()[-1]
        assert (last.msg.position.x, last.msg.position.y, last.msg.position.z) == \
            pytest.approx((0.12, -0.34, 0.56)), "setpoint 的位置必须跟随当前位姿（原地保持）"

        # —— 字段正确性（ticket 05 的三条硬指标）——
        assert last.msg.type_mask == 2552, (
            "type_mask 必须是 2552：忽略速度/加速度/yaw_rate，保留位置 + yaw；"
            "不含 FORCE(512) 与 IGNORE_YAW(1024)")
        assert last.msg.coordinate_frame == 1, "coordinate_frame 必须是 FRAME_LOCAL_NED(1)"
        assert last.msg.header.stamp.sec > 0 or last.msg.header.stamp.nanosec > 0, \
            "stamp 不能是 0（mavros 会把它换算成 time_boot_ms）"
        assert last.msg.header.frame_id == "map", "frame_id 应当沿用 local_position 的 frame"

        # ★ yaw：**本张起不再是常量 0，而是"当前机头"**（ticket 05 的 `yaw` 参数已删，
        #   见 control/include/control/fsm.hpp 的文件头 ★）。
        #   转一下假 EKF 的机头 ⟹ setpoint 的 yaw 必须跟着转，否则测的是"恰好等于 0"。
        #   ⚠️ 这里只断言 STANDBY 段（跟随）；"进入 TAKEOFF 后锁定不再跟随"在
        #   `test_control_takeoff.py` 里。
        sc.fake.set_yaw(0.7)
        sc.hold(0.5)
        assert sc.setpoints()[-1].msg.yaw == pytest.approx(0.7, abs=1e-6), (
            "STANDBY 段 setpoint 的 yaw 必须 = 当前机头（这样飞机原地不动、不转向）")
        sc.fake.set_yaw(0.0)
        sc.hold(0.3)

        # —— 频率：只在**窗口**上算，绝不断言相邻两帧的间隔（见 Scenario.setpoint_rate_hz）——
        rate = sc.setpoint_rate_hz(from_s=first.at_s + 0.3, to_s=last.at_s - 0.05)
        assert rate >= 0.8 * 50.0, f"setpoint 实测只有 {rate:.1f} Hz，应 ≈50 Hz"

        # —— OFFBOARD：等够前置流才切，且切了之后**控制权仍是 MANUAL** ——
        sc.wait_for(lambda: sc.fake.mode == "OFFBOARD", timeout_s=8.0,
                    message="DUT 没有把（假的）PX4 拉进 OFFBOARD")
        assert sc.control_state() == "MANUAL/STANDBY", (
            "切进 OFFBOARD 模式**不等于**交出控制权：飞机还没解锁 ⟹ 控制权仍是 MANUAL。"
            "这条语义见 control/include/control/fsm.hpp 对 Authority 的定义")
        assert sc.dut_alive(), "DUT 进程必须还活着（否则上面的断言可能是在测一个空场景）"


# ───────────────────────────── 三条守卫逐条缺失 ─────────────────────────────

def _blocked_then_recovered(break_it, fix_it, why: str):
    """`break_it` 掐掉一条前置 ⟹ 必须**一帧都不发**；`fix_it` 恢复 ⟹ 必须开始发。

    ★ 两个方向缺一不可：只有负向断言的话，"DUT 启动就崩了"也会让测试通过。
    """
    with Scenario(dut=DUT, params=BASE_PARAMS, start_dut=False) as sc:
        break_it(sc)
        sc.start_dut()
        sc.hold(4.0)  # 远超 longest guard 的判据（state 2.5 s），足够让"该发"的场面发生

        assert sc.control_state() == "MANUAL/INIT", (
            f"{why} ⟹ 任务态应当是 INIT，实际 {sc.control_state()!r}")
        assert not sc.setpoints(), f"{why} ⟹ 一帧 setpoint 都不该发（实际发了 {len(sc.setpoints())} 帧）"
        assert not sc.fake.mode_requests, f"{why} ⟹ 不该请求 OFFBOARD"
        assert sc.dut_alive(), "DUT 必须还活着 —— 否则上面那些「没有」说明不了任何事"

        fix_it(sc)
        assert sc.wait_for_setpoints(20, timeout_s=10.0), (
            f"恢复「{why}」之后仍然不流 setpoint ⟹ 上面那些「没有」不能算数"
            "（多半是场景本身没跑起来，而不是守卫在起作用）")


def test_守卫_动捕没数据时不发_setpoint():
    _blocked_then_recovered(
        break_it=lambda sc: sc.mocap_source.pause(),
        fix_it=lambda sc: sc.mocap_source.resume(),
        why="动捕没有数据",
    )


def test_守卫_local_position_失效时不发_setpoint():
    _blocked_then_recovered(
        break_it=lambda sc: sc.fake.mute("local_position"),
        fix_it=lambda sc: sc.fake.unmute("local_position"),
        why="/mavros/local_position/pose 静默",
    )


def test_守卫_飞控未连接时不发_setpoint():
    _blocked_then_recovered(
        break_it=lambda sc: sc.fake.mute("state"),
        fix_it=lambda sc: sc.fake.unmute("state"),
        why="/mavros/state 静默（飞控未连接）",
    )


def test_已解锁时不发_setpoint():
    """★ 外部解锁 = 异常（`params-changes.md` §H.6）：停发 setpoint + 告警。

    ★ 本用例用的是**外部解锁**的注入面（直接 `set_state(armed=True)`，DUT 没发过解锁请求）
      —— 这条路径在 ticket 08 之后仍然成立：DUT 会区分"这次 armed 上升沿是不是我请求的"
      （`px4_interface.cpp::on_state` 的归属判定）⟹ 没请求过的上升沿一律按外部解锁处理。
      我们自己解锁那一路（`/control/takeoff`）在 `test_control_takeoff.py` 里。"""
    with Scenario(dut=DUT, params=BASE_PARAMS) as sc:
        _reach_standby(sc)
        sc.fake.set_state(armed=True)
        sc.wait_for(lambda: sc.control_state() == "MANUAL/INIT", timeout_s=5.0,
                    message="检测到外部解锁之后应当退回 INIT")

        frozen = len(sc.setpoints())
        sc.hold(0.6)
        assert len(sc.setpoints()) == frozen, "外部解锁之后必须停发 setpoint"


# ───────────────────────────── 运行中前置失效 ─────────────────────────────

def test_STANDBY_中途飞控状态静默_流在_state_timeout_内停止():
    """★ 这一条与上一个用例测的是**不同的路径**：
    上一个是"状态变了"，这个是"状态**不再来了**"—— 后者才是实飞最危险的那种失效
    （飞控挂了/串口掉了，而最后一个 state 看起来一切正常）。"""
    with Scenario(dut=DUT, params=BASE_PARAMS) as sc:
        _reach_standby(sc)

        t_silence = sc.t() + 0.5
        sc.timeline.extend(
            silence_for(sc.fake, "state", at_s=t_silence, duration_s=3.0))

        # 静默 2.9 s（= state_timeout_s 2.5 + 0.4 余量）之后：必须已经退回 INIT
        sc.advance_to(t_silence + 2.9)
        assert sc.control_state() == "MANUAL/INIT", (
            f"/mavros/state 静默超过 state_timeout_s(2.5 s) 之后应当退回 INIT，"
            f"实际 {sc.control_state()!r}")

        frozen = len(sc.setpoints())
        sc.advance_to(t_silence + 2.98)   # 停在恢复（3.0 s）之前
        assert len(sc.setpoints()) == frozen, (
            "守卫失效之后必须**立刻**停发 setpoint（真机上 PX4 会在 COM_OF_LOSS_T 后自行退出 OFFBOARD）")

        # 正控：恢复之后必须重新流起来
        sc.advance_to(t_silence + 8.0)
        assert sc.control_state() == "MANUAL/STANDBY", "state 恢复之后应当回到 STANDBY"
        assert len(sc.setpoints()) > frozen, "恢复之后应当重新开始流"


# ───────────────────────────── OFFBOARD 前置流 ─────────────────────────────

def test_切_OFFBOARD_不早于前置流时长():
    """★ 两侧都断言：**不能更早**（PX4 会因断流/流太短拒绝或立刻掉出 OFFBOARD）
    也**不能晚太多**（晚太多说明判据没在工作，只是碰巧没犯规）。

    前置流设成 3.0 s（比冒烟用的 2.0 s 更长），这样"到没到点"不会被启动抖动掩盖。"""
    params = dict(BASE_PARAMS, offboard_prestream_s=3.0)
    with Scenario(dut=DUT, params=params) as sc:
        _reach_standby(sc)
        sc.wait_for(lambda: bool(sc.fake.mode_requests), timeout_s=10.0,
                    message="从来没有请求切 OFFBOARD")

        first_setpoint = sc.setpoints()[0].at_s
        first_request = sc.fake.mode_requests[0].at_s
        delta = first_request - first_setpoint

        assert delta >= 3.0 - 0.05, (
            f"切 OFFBOARD 只在前置流 {delta:.2f} s 后就发生了，要求 ≥3.0 s。"
            "依据：断流 PX4 会自动退出 OFFBOARD（COM_OF_LOSS_T），所以「先流够了再切」是前提")
        assert delta <= 3.0 + 1.5, (
            f"前置流够了之后 {delta:.2f} s 才切 —— 太晚了。"
            "（若正好卡在边界上，多半是 set_mode 服务还没发现 ⟹ 重试间隔在起作用）")

        # 未成则重试（本条只在夹具回答"拒绝"时有意义），这里只要求：切成功之后不再重复请求
        sc.wait_for(lambda: sc.fake.mode == "OFFBOARD", timeout_s=8.0)
        n_after = len(sc.fake.mode_requests)
        sc.hold(2.5)
        assert len(sc.fake.mode_requests) == n_after, (
            "已经切进 OFFBOARD 之后不该再重复请求（/mavros/state 是 1 Hz，"
            "重复请求说明 in_offboard 的判据没生效）")


def test_关掉_enter_offboard_就只流不切():
    """`enter_offboard = false` 是台架上想手动切模式时用的开关（用户 2026-09-21 裁定）。"""
    params = dict(BASE_PARAMS, enter_offboard=False)
    with Scenario(dut=DUT, params=params) as sc:
        _reach_standby(sc)
        sc.hold(3.0)  # 远超前置流时长；若开关没生效，早就切了
        assert not sc.fake.mode_requests, "enter_offboard = false 时不该请求任何模式切换"
        assert sc.fake.mode == "POSCTL", "假 PX4 的模式不该被改动"
        assert len(sc.setpoints()) > 20, "但 setpoint 必须照流（开关只管切不切模式）"


# ───────────────────────────── 诊断日志：能指认到"是谁" ─────────────────────────────

def test_降级证据里带发布者的节点名():
    """★ 「话题上有帧 / 没帧」之外，日志还必须能回答「**是谁**在发」。

    缘起是一条没查出来的悬案（2026-09-22 台架）：`/mocap/pose_enu` 上收到过帧，而它唯一的合法
    发布者当时**没有任何发布路径**；当时日志只写"发布者：N 个（QoS）"⟹ **事后指认不出是谁发的**，
    十三条候选只能一条条手工排除。数量回答"有没有"，**节点名才回答"是谁"**。

    ★ 本用例刻意走**降级告警的证据**那条路径，而不是 INIT 的诊断块：后者每个周期只打一次，
      若第一拍的诊断**早于 DDS 发现完成**，它打的就是"0 个"（实测见过），断言会随发现快慢翻脸。
      这里先在 `_reach_standby` 里等订阅者**匹配上**（= 发现已完成），再制造静默 ⟹ 不依赖时序。
    """
    with Scenario(dut=DUT, params=BASE_PARAMS) as sc:
        _reach_standby(sc)  # ★ 这一句同时是"发现已完成"的屏障（见上面的说明）
        sc.fake.mute("local_position")  # 有发布者、但没有帧 —— 正是当初排不出来的那一类

        # ★ `wait_for` 自己断言（不返回布尔），别写成 `assert sc.wait_for(...)` —— 那会 `assert None`。
        sc.wait_for(lambda: "degrade=local_pos_timeout" in sc.dut_log(), timeout_s=6.0,
                    message="local_position 静默超过 local_pos_timeout_s 之后应当打出降级告警")

        log = sc.dut_log()
        chunk = log[log.index("degrade=local_pos_timeout"):][:400]
        assert f"/{sc.node.get_name()}" in chunk, (
            "降级证据里没有出现发布者的**节点名**："
            f"夹具这个节点叫 /{sc.node.get_name()}，静默期间它仍然挂在话题图上。\n"
            "只有数量没有名字 ⟹ 幽灵发布者那类事件事后指认不出来（本条测试的由来）。\n"
            + sc.evidence())
        # 正控：QoS 那一列不能被节点名挤掉 —— 它是"静默但没人报错"那类故障的唯一线索。
        assert "best_effort" in chunk, (
            "降级证据里丢了 QoS —— local_position 在真 mavros 上是 best_effort，"
            "这一列是发现「best_effort 发布方配 reliable 订阅方」那种静默故障的仅有线索\n"
            + sc.evidence())


# ───────────────────────────── 断流实验（夹具这一半） ─────────────────────────────

def test_杀掉_DUT_之后_setpoint_流停止():
    """★ ticket 05 验收项「断流实验」的**夹具这一半**。

    真机那一半（真 PX4 在 `COM_OF_LOSS_T` 之后自行退出 OFFBOARD、按 `COM_OBL_RC_ACT` 回 Position）
    需要飞控空闲时手动跑一次 —— 原因见本包 README「还没做的验收项」。
    """
    with Scenario(dut=DUT, params=BASE_PARAMS) as sc:
        _reach_standby(sc)

        # 故障按时序注入：让"DUT 在某个时刻死掉"与时序里其它事件是同一种东西
        sc.timeline.add(sc.t() + 0.5, sc.stop_dut, label="杀掉 DUT（SIGINT）")
        sc.advance_to(sc.t() + 1.0)

        assert not sc.dut_alive(), "DUT 应当已经退出"
        frozen = len(sc.setpoints())
        sc.hold(1.0)
        assert len(sc.setpoints()) == frozen, (
            "DUT 进程已经退出，不该再有 setpoint 到达"
            f"（冻结后又来了 {len(sc.setpoints()) - frozen} 帧）")
