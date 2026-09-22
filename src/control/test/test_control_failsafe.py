"""control 节点的**失效路径**测试（**seam 1**：只断言话题/服务边界上发生了什么）—— ticket 06。

判据全部来自 ticket 06 的验收清单：
  ① 打杆 ⟹ **立刻停发 setpoint**（不是"不更新目标"）+ 退到 `RC_OVERRIDE`
  ② 动捕 / `/mavros/state` / 遥控器**各一条**超时判据，任一条超时 ⟹ `LAND`
  ③ 电压低于 `low_voltage_v` ⟹ 主动中止任务 ⟹ `LAND`（**不依赖电流读数**）
  ④ EKF2 位置失效 ⟹ `LAND`
  ⑤ 降级**可复位**（不是单向 latch），原因在日志里**可检索**（`degrade=<token>`）
  ⑥ 故障注入全部在夹具里验证过 —— **本文件就是那一条的现场**：三条输入的超时（②）、
     动捕时间戳冻住与倒退（本文件末尾两条）、电压跌落（③），全在指定时刻注入。
  ⑦ 任何失效下进程**不崩**

★ 本文件与 `test_control_standby.py` 的分工（ticket 05 那一半）：那份问"守卫成立时发了什么"，
  这份问"守卫/判据失效时**停止**了什么"。两者都只看外部可观测的东西，DUT 是**子进程**。

★ 关于负向断言：每一条"不再发 setpoint / 不再请求模式"都配了**正控**（此前确实在发/在请求），
  否则"DUT 启动就崩了"会让所有负向断言假通过。每条用例末尾都查 `sc.dut_alive()`（验收项 ⑦）。

★★ **ticket 08 之后本文件多了一条前置**：②③④⑥ 这几条判据都只在**已解锁**时成立，
  而 ticket 08 给了真实的解锁路径 ⟹ 那些用例的基线不再能靠注入 `armed` 造出来，
  必须真的走一次 `/control/takeoff`（见 `_reach_auto`）。基线状态因此是 `AUTO/TAKEOFF`
  （一解锁就开始爬升）而不是 `AUTO/STANDBY`。

★★ 一条**没做到的事**（诚实记账）：① 的后半句"实测确认 PX4 按 `COM_OBL_RC_ACT` 接管"是
  **真机实验**，不能在台架上做（台架里没有真 PX4）。它和 ticket 05 的"断流实验真机那一半"
  一样，属于本包 README 里列出的"还没做的验收项"。本文件能证明的是"我们这边确实停发了"，
  而那正是那个真机实验的**前提**。
"""

import re

import pytest
from std_srvs.srv import Trigger

from bench import Scenario
from bench.timeline import hold_mocap_stamp, rc_change, voltage_drop

#: DUT =（包名，可执行名）—— 由 `ament_index_python` 解析成 <prefix>/lib/control/control_node
DUT = ("control", "control_node")

#: 起飞命令的服务名（ticket 08）。★ 本文件用它把飞机送到"已解锁"那个基线态。
TAKEOFF_SRV = "/control/takeoff"

#: 与 `config/control.yaml` 一致的参数。逐条写出来，是为了让"这一条判据在什么口径下成立"
#: 在断言旁边就能读到（与 `test_control_standby.py` 同一条纪律）。
BASE_PARAMS = {
    "setpoint_rate_hz": 50.0,
    "enter_offboard": True,
    "offboard_prestream_s": 1.0,
    "offboard_retry_s": 1.0,
    # ★ 2.5 s 而不是 px4ctrl 的 0.5 s：/mavros/state 实测 1 Hz（夹具也照 1 Hz 发）。
    "state_timeout_s": 2.5,
    "local_pos_timeout_s": 0.5,
    "mocap_timeout_s": 0.5,
    "rc_timeout_s": 0.5,
    "battery_timeout_s": 2.0,
    # —— ticket 06 的失效判据阈值 ——
    "low_voltage_v": 20.0,
    "rc_stick_ov_us_per_s": 100.0,
    "mocap_stamp_stall_s": 0.5,
    # —— ticket 16：EKF2 位置失效的载体（`/mavros/estimator_status`）——
    # ★ 上游实测 1.000 Hz（夹具也照 1 Hz 发）⟹ 超时取 2.5 s；消抖是**帧数**（2 帧 ≈ 2 s）。
    "estimator_timeout_s": 2.5,
    "ekf2_pos_lost_samples": 2,
    "land_mode": "AUTO.LAND",
    # ☠️ 这里原本有一个 `own_arming: True` —— ticket 06 的**台架缝**（本张没有解锁路径，
    #   而"已解锁时的失效该请求 AUTO.LAND"只在 armed 时成立）。
    #   **ticket 08 把它删掉了**：解锁路径已经真实存在（`/control/takeoff` +
    #   `/mavros/cmd/arming`），归属由应答 + `armed` 上升沿真实追踪（`px4_interface.cpp::on_state`）。
    #   留着那个参数会**打掉** §H.6 的"外部解锁"告警 —— 它会把别人拨的解锁开关也记成我们做的。
    #   ⟹ 本文件里所有"已解锁"的场面改走**真实解锁路径**（见 `_reach_auto`）。
}


# ───────────────────────────────── 工具 ─────────────────────────────────

def _reach_auto(sc):
    """把场景推到"**已解锁**、正在 OFFBOARD 里流 setpoint"（= `/control/state` 为 `AUTO/TAKEOFF`）。

    ★ 顺序是刻意的：先等**模式真的切进 OFFBOARD**（这同时证明 set_mode 那条链路通），
      再走**真实的解锁路径** `/control/takeoff` —— ticket 08 之后没有"台架专属的解锁缝"
      （`own_arming` 参数已删）：`armed` 的归属由 `/mavros/cmd/arming` 的应答真实追踪，
      所以"这次解锁是我们自己发起的"这件事**只能**靠真的解一次来实现。

    ★ 基线状态是 `AUTO/TAKEOFF`（**不是** `AUTO/STANDBY`）：一旦解锁，状态机就开始往上爬。
      夹具里飞机**不动**（本文件不装 `test_control_takeoff.py` 里那层假飞机）⟹ 到高判据
      永远不成立 ⟹ 会稳定地停在 TAKEOFF。这些用例注入的是别的东西，不关心飞得多高。

    ★ 第一件事不是等时间而是等**匹配**：`matched_subscribers` 计的是已经匹配上的订阅者，
      于是 QoS 不兼容会在这里立刻失败，而不是表现为"莫名其妙没数据"。
    """
    assert sc.wait_for_subscribers("state"), "DUT 没有接上 /mavros/state"
    assert sc.wait_for_subscribers("local_position"), "DUT 没有接上 /mavros/local_position/pose"
    assert sc.wait_for_subscribers("rc"), "DUT 没有接上 /mavros/rc/in"
    assert sc.wait_for_subscribers("battery"), "DUT 没有接上 /mavros/battery"
    # ★ ticket 16：这一条是**必须**在这里等的 —— DUT 侧对这条话题用的是易失订阅，而夹具
    #   照抄真 mavros 用 transient_local 发 ⟹ 两者兼容但**不回放历史**。若哪天有人把其中
    #   一边的 QoS 改错（两边就都不匹配），`matched_subscribers` 会在这里当场变成 0，
    #   而不是让后面的用例以"判据没反应"的样子失败。
    assert sc.wait_for_subscribers("estimator_status"), "DUT 没有接上 /mavros/estimator_status"
    assert sc.poll(lambda: sc.mocap_source.matched_subscribers() >= 1), "DUT 没有接上动捕话题"
    assert sc.wait_for_setpoints(20, timeout_s=10.0), "守卫成立之后应当开始流 setpoint"

    sc.wait_for(lambda: sc.fake.mode == "OFFBOARD", timeout_s=8.0,
                message="DUT 没有把（假的）PX4 拉进 OFFBOARD ⟹ 后面的判据没有基线")
    # ★ `/mavros/state` 是 1 Hz ⟹ DUT 最多还要 1 s 才知道"已在 OFFBOARD"，
    #   而那是解锁的前置之一。不等的话这里会变成"解锁被前置判据挡住"的假故障。
    sc.hold(1.2)

    client = sc.node.create_client(Trigger, TAKEOFF_SRV)
    assert client.wait_for_service(timeout_sec=6.0), f"DUT 没有提供 {TAKEOFF_SRV}"
    future = client.call_async(Trigger.Request())
    assert sc.poll(lambda: future.done(), timeout_s=6.0), f"{TAKEOFF_SRV} 没有回应"
    assert future.result().success, f"起飞命令被拒：{future.result().message}"
    sc.node.destroy_client(client)

    assert sc.wait_for_control_state("AUTO/TAKEOFF", timeout_s=10.0), (
        f"受理 + 解锁之后应当进 AUTO/TAKEOFF（我们正在发会驱动飞机的指令），"
        f"实际 {sc.control_state()!r}")


def _degrade_tokens(sc):
    """DUT 日志里出现过的 `degrade=<token>`（ticket 06 要求"原因在日志里**可检索**"）。

    token 由 `fsm.hpp::to_string(Degrade)` 给 ⟹ 日志里搜到的原因与代码里的判据同源。

    ⚠️ 词表是 `[a-z0-9_]+`，**含数字** —— ticket 16 的 `ekf2_pos_lost` 是第一个带数字的 token，
      而这里的正则当时写的是 `[a-z_]+` ⟹ 它把 token 截成 `ekf`，测试报"日志里没有这条降级"
      而**日志里明明有**（现场用 README 里那条 `grep -oE` 方子的人会遇到同一件事）。
      ★ 同一条词表现在由 `test_fsm.cpp::DegradeNames` 逐个字符断言 ⟹ 以后再加 token 时，
        "文档里的方子捞不出来"这种事会在 seam 2 上先红，而不是留到现场。
    """
    return re.findall(r"degrade=([a-z0-9_]+)", sc.dut_log())


def _wait_degrade(sc, token, timeout_s=3.0):
    """等某条降级原因出现在 DUT 日志里（stdout 管道有一点延迟，不硬断言）。"""
    sc.wait_for(lambda: token in _degrade_tokens(sc), timeout_s=timeout_s,
                message=f"DUT 日志里没出现 degrade={token}；出现过：{_degrade_tokens(sc)}")


def _stopped(sc, hold_s=0.8):
    """断言 setpoint 流**真的停了**（配正控：调用前必须已经流过）。"""
    assert sc.setpoints(), "正控失败：此前一帧 setpoint 都没发过 ⟹ 下面的「停了」不成立"
    frozen = len(sc.setpoints())
    sc.hold(hold_s)
    assert len(sc.setpoints()) == frozen, (
        f"应当停止发布 setpoint，但冻结之后又来了 {len(sc.setpoints()) - frozen} 帧")


def _mode_requests_after(sc, n_before):
    return [r.custom_mode for r in sc.fake.mode_requests[n_before:]]


def _watch_mocap_stamps(sc):
    """开始记录 `/mocap/pose_enu` 上的时间戳，返回 `(stamps, 订阅句柄)`。

    ★ 给"注入真的生效了"当正控用：**负向**断言（"短促倒退不降级"）在注入静默失效时会**假绿**
      —— 那种绿比红危险。所以先用话题上的**向后跳变**证明注入到达了（`bench/README.md` 的运行纪律）。

    ★★ 返回前要过**两道**关，两道都缺一不可（第一版两道都没过，红得莫名其妙，两处都是探针查出来的）：

      1. **等自己这个旁路订阅匹配上**（发现屏障，`bench/README.md` 的同名条目）。新建的订阅要
         几十~上百 ms 才匹配，而这类用例的注入窗口只有 0.2 s（**必须**短于 `mocap_stamp_stall_s`，
         否则就不是"短促"了）⟹ 不等匹配，窗口会在订阅还没接上时开完、关完。
      2. **等基线的几帧真的到了**再返回调用方去注入。★ 这条更隐蔽：倒退是**相对前一帧**的，
         如果调用方拿到句柄就立刻 `rewind`，我们收到的**第一帧就是倒退后的那一帧**
         ⟹ `_stamp_deltas` 里根本没有那个负值可看（探针实测：窗口里全是 +5 ms 的小步进，
         只有窗口结束时的 **+5.0 s** 一次前跳可见）—— 于是"没看见向后跳变"会误报，
         而注入其实是好的。
    """
    from bench.vrpn_source import mocap_qos
    from geometry_msgs.msg import PoseStamped

    before = sc.mocap_source.matched_subscribers()
    stamps: list = []
    sub = sc.node.create_subscription(
        PoseStamped, sc.mocap_source.topic,
        lambda m: stamps.append(m.header.stamp.sec + m.header.stamp.nanosec * 1e-9),
        mocap_qos())
    assert sc.poll(lambda: sc.mocap_source.matched_subscribers() > before, timeout_s=2.0), (
        f"旁路订阅没接上动捕话题（匹配数一直停在 {sc.mocap_source.matched_subscribers()}）"
        f"⟹ 下面这个 0.2 s 的注入窗口太短，正控会假绿")
    assert sc.poll(lambda: len(stamps) >= 3, timeout_s=2.0), (
        f"旁路订阅匹配上了却只收到 {len(stamps)} 帧 ⟹ 没有基线帧，下面的「向后跳变」判不出来")
    return stamps, sub


def _stamp_deltas(stamps):
    """相邻两帧的戳差（s）。**倒退**表现为一个大幅负值。"""
    return [b - a for a, b in zip(stamps, stamps[1:])]


def _assert_requests_recorded(sc, timeout_s=6.0):
    """**负向断言的正控**：假 mavros 确实在记模式请求（`OFFBOARD` 已经到过）。

    ★ 没有它，"之后没再请求任何模式"这句话可能只是"假 mavros 压根没记到东西"（服务没接上、
      测试过早）—— 那种假绿比红更危险。本包对"未收到/没变化"的断言一律配正控（README 的运行纪律）。

    ⚠️ 这里必须用 `sc.poll()` 而**不是** `sc.wait_for()`：`wait_for()` 成功时返回 `None`
      （超时抛 `BenchTimeout`），把它写进 `assert` 会**永远失败** —— 初版就是这么写的，
      一跑就红（2026-09-22 全量套件抓到的）。`poll()` 才是"不抛异常版的布尔"。
    """
    assert sc.poll(lambda: "OFFBOARD" in [r.custom_mode for r in sc.fake.mode_requests],
                   timeout_s=timeout_s), (
        "正控失败：假 mavros 一条 OFFBOARD 请求都没记到 ⟹ 下面「不该再请求」的断言不成立")


# ─────────────────────── ① 打杆接管（RC override） ───────────────────────

def test_打杆立刻停发_退到_RC_OVERRIDE_且不抢回_OFFBOARD():
    """① 打杆 ⟹ **立刻停发 setpoint**（不是"不更新目标"——那等于让飞机悬停在原目标，
    操作员打杆也拉不回来），并交还控制权。

    "立刻"的量级：摇杆判据看的是**变化率**（夹具 10 Hz 的 rc 帧），一帧之内就能算出来，
    所以从注入到停发应当远小于一秒。这里给 0.3 s 的宽限（远超链路本身的开销，
    又足以抓住"一直不停发"这种真故障）。
    """
    with Scenario(dut=DUT, params=BASE_PARAMS) as sc:
        _reach_auto(sc)
        _assert_requests_recorded(sc)
        n_requests = len(sc.fake.mode_requests)

        t_stick = sc.t() + 0.3
        sc.timeline.extend(rc_change(sc.fake, t_stick, 1, 1900))   # 通道 1（roll）一步推到 1900
        assert sc.wait_for_control_state("MANUAL/RC_OVERRIDE", timeout_s=5.0), (
            f"打杆之后应当退到 RC_OVERRIDE（控制权被夺走），实际 {sc.control_state()!r}")

        fired = [f for f in sc.timeline.fired if f.label.startswith("ch1")]
        assert fired, "注入事件没触发 ⟹ 下面的时序断言不成立"
        last = sc.setpoints()[-1].at_s
        assert last <= fired[0].fired_at_s + 0.3, (
            f"打杆之后 {last - fired[0].fired_at_s:.2f} s 还在发 setpoint —— 要求「立刻停发」")

        _wait_degrade(sc, "rc_override")
        assert _degrade_tokens(sc)[0] == "rc_override", (
            f"第一条降级原因应当就是打杆接管，实际 {_degrade_tokens(sc)}")
        _stopped(sc)

        # ★ 本张最关键的一条"不做"：不抢回 OFFBOARD（否则与飞行员一秒一次地拉锯）。
        sc.hold(2.0)
        assert _mode_requests_after(sc, n_requests) == [], (
            f"夺权之后不该再请求任何模式，实际请求了 {_mode_requests_after(sc, n_requests)}")
        assert sc.fake.mode == "OFFBOARD", "（假的）PX4 的模式不该被我们改动"
        assert sc.dut_alive(), "验收项 ⑦：任何失效下节点都不许崩"


def test_外部把模式切走_退到_RC_OVERRIDE_且不抢回_然后交回即复位():
    """①的另一种夺权：**模式被外部切走**（飞行员拨了模式开关 / QGC 切了模式）。

    ★ 这条同时是验收项 ⑤ 的一半（可复位）：夺权闩锁的复位条件是"有人把模式放回 OFFBOARD"
      —— 那是**外部动作**，我们从不自己请求 OFFBOARD 去抢（所以这条只可能是人干的）。
    """
    with Scenario(dut=DUT, params=BASE_PARAMS) as sc:
        _reach_auto(sc)
        n_requests = len(sc.fake.mode_requests)

        sc.fake.set_state(mode="POSCTL")   # 外部夺权：/mavros/state 是 1 Hz ⟹ 最多晚 1 s 被检出
        assert sc.wait_for_control_state("MANUAL/RC_OVERRIDE", timeout_s=5.0), (
            f"模式被切走之后应当退到 RC_OVERRIDE，实际 {sc.control_state()!r}")
        _wait_degrade(sc, "mode_lost")
        _stopped(sc)

        sc.hold(2.0)
        assert _mode_requests_after(sc, n_requests) == [], "夺权之后不该再请求任何模式"
        assert sc.fake.mode == "POSCTL", "模式被切走之后我们不许把它抢回来"

        # —— ⑤ 复位：外部把模式交回 OFFBOARD ⟹ 立刻恢复（不需要重启节点，也不需要"清故障"命令）——
        sc.fake.set_state(mode="OFFBOARD")
        assert sc.wait_for_control_state("AUTO/TAKEOFF", timeout_s=5.0), (
            f"外部把模式放回 OFFBOARD 之后应当自动复位，实际 {sc.control_state()!r}"
            "（任务层**不**因夺权而复位：飞机还解锁着 ⟹ 应当在 TAKEOFF 里继续飞）")
        assert sc.wait_for_setpoints(len(sc.setpoints()) + 20, timeout_s=5.0), \
            "复位之后必须重新开始流 setpoint（否则「可复位」是空的）"
        assert sc.dut_alive()


# ─────────────────────── ② 三条输入超时 ⟹ LAND ───────────────────────

def _timeout_case(inject, expected_token: str):
    """共用的骨架：已解锁 ⟹ 某条输入失效 ⟹ 停发 + 请求 `AUTO.LAND` + 日志可检索。"""
    with Scenario(dut=DUT, params=BASE_PARAMS) as sc:
        _reach_auto(sc)
        _assert_requests_recorded(sc)
        n_requests = len(sc.fake.mode_requests)

        inject(sc)
        assert sc.wait_for_control_state("MANUAL/LAND", timeout_s=8.0), (
            f"{expected_token} ⟹ 应当进 LAND（已解锁 ⟹ 主动中止任务），实际 {sc.control_state()!r}")
        _wait_degrade(sc, expected_token)
        assert _degrade_tokens(sc)[0] == expected_token, (
            f"第一条降级原因应当是 {expected_token}，实际 {_degrade_tokens(sc)}")
        _stopped(sc)

        # 请求了中止任务的模式（`AUTO.LAND`），且**此后不再请求 OFFBOARD**（那等于一边降落一边抢）。
        sc.wait_for(lambda: "AUTO.LAND" in [r.custom_mode for r in sc.fake.mode_requests],
                    timeout_s=5.0, message="没有请求 AUTO.LAND ⟹ 主动中止任务这一半没生效")
        sc.hold(2.0)
        assert set(_mode_requests_after(sc, n_requests)) <= {"AUTO.LAND"}, (
            f"降级之后的模式请求只该是 AUTO.LAND，实际 {_mode_requests_after(sc, n_requests)}")

        # ★★ 回归：**我们自己请求的 AUTO.LAND 不是"被夺权"**。
        #    若把 OFFBOARD→AUTO.LAND 误判成夺权，状态机会变成 MANUAL/RC_OVERRIDE、
        #    并且连降落都不再请求（见 `fsm.hpp::mode_is_ours` 的说明）。
        assert sc.control_state() == "MANUAL/LAND", (
            f"请求 AUTO.LAND 成功之后应当仍是 LAND，实际 {sc.control_state()!r}"
            "（变成 RC_OVERRIDE 说明我们自己的降落请求被误判成了夺权）")
        # 而且不再重发（`in_land` 为真 ⟹ 决定"要不要重试"的是状态机）。
        n_land = len([r for r in sc.fake.mode_requests if r.custom_mode == "AUTO.LAND"])
        sc.hold(2.5)
        assert len([r for r in sc.fake.mode_requests if r.custom_mode == "AUTO.LAND"]) == n_land, (
            "已经在我们请求的降落模式里了，不该再按重试间隔重发（纯噪声）")
        assert sc.dut_alive(), "验收项 ⑦"


@pytest.mark.parametrize("label,inject,token", [
    ("动捕停止发布", lambda sc: sc.mocap_source.pause(), "mocap_timeout"),
    ("/mavros/state 静默（飞控失联）",
     lambda sc: sc.timeline.extend(_silence(sc, "state")), "state_timeout"),
    ("遥控器静默", lambda sc: sc.timeline.extend(_silence(sc, "rc")), "rc_timeout"),
], ids=["mocap", "state", "rc"])
def test_三条输入超时各有一条判据_任一条超时进_LAND(label, inject, token):
    """② 的正面：三条输入**各有一条**超时判据，任一条成立 ⟹ `LAND`（已解锁时）。

    ⚠️ 这三条的上游频率差两个数量级（动捕 ~196 Hz、rc 10 Hz、state **1 Hz**），
      所以判据也必须是逐条参数化的 —— 见 `control.yaml` 里 `state_timeout_s` 为什么是 2.5 s。
    """
    _timeout_case(inject, token)


def _silence(sc, topic_key: str):
    """静默到"判据必然成立"的时长（比对应超时多留 0.5 s，避免卡在边界上）。

    ★ 不用 `silence_for` 的固定时长：state 的判据是 2.5 s，而 rc / mocap 是 0.5 s ——
      用同一个数字会让其中一条测的其实是别的东西。
    """
    timeout = {"state": 2.5, "rc": 0.5, "mocap": 0.5}[topic_key]
    from bench.timeline import silence_for
    return silence_for(sc.fake, topic_key, at_s=sc.t() + 0.2, duration_s=timeout + 0.5)


# ─────────────────────── ③ 低压 ⟹ LAND（+ ⑤ 复位） ───────────────────────

def test_低压主动中止任务_且落地上锁之后复位():
    """③ + ⑤：电压跌破阈值 ⟹ 停发 + 请求 `AUTO.LAND`；**复位条件是落地上锁**。

    ★ "不依赖电流读数"是刻意的（ticket 06 明确要求）：夹具把 `current` / `percentage`
      都发成 NaN（真 mavros 在只有 `SYS_STATUS` 时也是这样），判据只看 `voltage`。
    """
    with Scenario(dut=DUT, params=BASE_PARAMS) as sc:
        _reach_auto(sc)
        _assert_requests_recorded(sc)
        n_requests = len(sc.fake.mode_requests)

        t_drop = sc.t() + 0.2
        sc.timeline.extend(voltage_drop(sc.fake, t_drop, 19.5))   # < low_voltage_v = 20.0
        assert sc.wait_for_control_state("MANUAL/LAND", timeout_s=6.0), (
            f"低压应当主动中止任务，实际 {sc.control_state()!r}")
        _wait_degrade(sc, "battery_low")
        assert _degrade_tokens(sc)[0] == "battery_low", f"实际 {_degrade_tokens(sc)}"
        _stopped(sc)
        sc.wait_for(lambda: "AUTO.LAND" in [r.custom_mode for r in sc.fake.mode_requests],
                    timeout_s=5.0, message="低压之后没有请求 AUTO.LAND")

        # ★ 电压自己恢复**不算**复位：飞机还在天上，电池不会自己变满 ⟹ 不许空中自动接管。
        sc.fake.set_voltage(25.0)
        sc.hold(2.5)
        assert sc.control_state() == "MANUAL/LAND", (
            f"电压读数恢复不该让飞机在空中自动接管，实际 {sc.control_state()!r}")
        _stopped(sc, hold_s=0.5)

        # —— ⑤ 复位：落地上锁（armed → false）。这是"可复位、不是单向 latch"的主路径 ——
        sc.fake.set_state(armed=False)
        assert sc.wait_for_control_state("MANUAL/STANDBY", timeout_s=6.0), (
            f"落地上锁之后应当自动复位回 STANDBY，实际 {sc.control_state()!r}"
            "（单向 latch 的话这里会永远停在 LAND，只能重启节点）")
        assert "none" in _degrade_tokens(sc), \
            f"复位也要在日志里可检索（degrade=none），实际 {_degrade_tokens(sc)}"
        assert sc.wait_for_setpoints(len(sc.setpoints()) + 20, timeout_s=5.0), \
            "复位之后必须重新开始流 setpoint"
        assert _mode_requests_after(sc, n_requests) == ["AUTO.LAND"], \
            f"复位之后不该再有别的模式请求，实际 {_mode_requests_after(sc, n_requests)}"
        assert sc.dut_alive()


# ─────────────────────── ④ EKF2 位置失效 ───────────────────────

def test_位置估计给出_NAN_时停发并请求_LAND():
    """④ EKF2 位置失效 ⟹ `LAND`。注入面是 `local_position` 上的 NaN/Inf。

    ⚠️ **这条判据目前是"防御性"的，不是真机现象的直接复现**（见夹具自检里同名那段与
      ticket 06 的 Comments）：第一手核实过 z0 固件 `4a0e65f2` 那条链路 ——
      `EKF2.cpp::PublishLocalPosition` 不发 NaN、`LOCAL_POSITION_NED.hpp` 不带 `xy_valid`、
      mavros `local_position.cpp` 原样转 —— **位置真失效时表现成"话题照发、值不可信"，
      而不是 NaN**。所以 ④ 在本栈里的**实际**防线是"动捕失效 ⟹ 0.5 s 就 LAND"（动捕是本栈
      唯一的位置源），这条 NaN 判据是同一件事的第二道便宜保险。
    """
    with Scenario(dut=DUT, params=BASE_PARAMS) as sc:
        _reach_auto(sc)
        n_requests = len(sc.fake.mode_requests)

        sc.fake.set_position(float("nan"), 0.0, 0.0)   # EKF2 报了 NaN/Inf 的位置
        assert sc.wait_for_control_state("MANUAL/LAND", timeout_s=6.0), (
            f"位置出现 NaN 应当立刻停发并中止任务，实际 {sc.control_state()!r}")
        _wait_degrade(sc, "position_invalid")
        assert _degrade_tokens(sc)[0] == "position_invalid", (
            f"NaN 是根因，应当排在 local_pos_timeout 前面，实际 {_degrade_tokens(sc)}")
        _stopped(sc)
        sc.wait_for(lambda: "AUTO.LAND" in [r.custom_mode for r in sc.fake.mode_requests],
                    timeout_s=5.0, message="没有请求 AUTO.LAND")
        assert sc.dut_alive()


# ──────────── ④ 之二：EKF2 位置失效的**真载体**（ticket 16） ────────────
#
# ④ 原来只挂在"位置是 NaN"上，而第一手核实过：**那条判据在真机上恒假** ——
# PX4 的 `EKF2.cpp::PublishLocalPosition` 不合成 NaN、`LOCAL_POSITION_NED` 又把 `xy_valid`
# 丢掉，mavros 原样转 ⟹ 位置真失效时表现成"话题照发、值不可信"（见上方 NaN 用例的说明，
# 以及 `docs/control/phase1-implementation.md` §4.4）。ticket 16 找到了真能读到的载体：
# `/mavros/estimator_status.pos_horiz_abs`（PX4 侧 `EKF/ekf_helper.cpp:761` 每次发布现算 ⟹ 能自清）。

def test_EKF2自报没有绝对位置时停发并请求_LAND():
    """④ 的正面（已解锁）：`pos_horiz_abs` 连续为假 ⟹ 停发 + `AUTO.LAND`，原因可检索。

    ★ 这条注入**不碰** `local_position` 的任何读数（真 PX4 也不碰）：它造的是
      "动捕帧正常到、位置话题照发，而 EKF2 说它的水平位置没有绝对参考" ——
      那正是 `mocap_timeout` / `mocap_stamp_stalled` 看不见的一类失效。
    """
    with Scenario(dut=DUT, params=BASE_PARAMS) as sc:
        # ★ 发现屏障（`src/bench/README.md` 纪律 1）：下面"**单帧**为假还没降级"是否定断言，
        #   必须先把"DUT 真的接上了这个话题"钉死，否则"没降级"与"压根没收到"在日志里长得一样。
        assert sc.wait_for_subscribers("estimator_status"), "DUT 没有接上 /mavros/estimator_status"
        _reach_auto(sc)
        _assert_requests_recorded(sc)
        n_requests = len(sc.fake.mode_requests)

        sc.fake.set_pos_horiz_abs(False)
        # ★★ 消抖的正面判据（这是本张"连续 2 帧"裁定的**唯一**直接证据）：
        #    先只让它发**一帧**假，确认此刻还没降级；第二帧到齐之后才准降级。
        #    ★ 用夹具的帧计数对齐，不用 sleep 猜 —— 上游 1 Hz，猜必然又慢又飘。
        n0 = sc.fake.estimator_status_frames
        assert sc.poll(lambda: sc.fake.estimator_status_frames > n0, timeout_s=3.0), \
            "夹具没有继续发 estimator_status（频率被改过？）"
        sc.hold(0.4)   # 留出 DUT 处理这一帧的时间（下一帧还在 ~0.6 s 之后）
        assert "ekf2_pos_lost" not in _degrade_tokens(sc), (
            f"**单帧**为假就降级了 ⟹ 消抖（`ekf2_pos_lost_samples`）没生效，"
            f"实际 {_degrade_tokens(sc)}")
        assert sc.control_state() == "AUTO/TAKEOFF", (
            f"单帧为假不该改变状态，实际 {sc.control_state()!r}")

        # 第二帧为假 ⟹ 判据成立。
        assert sc.wait_for_control_state("MANUAL/LAND", timeout_s=6.0), (
            f"EKF2 自报位置没有绝对参考 ⟹ 应当停发并中止任务，实际 {sc.control_state()!r}")
        _wait_degrade(sc, "ekf2_pos_lost")
        assert _degrade_tokens(sc)[0] == "ekf2_pos_lost", f"实际 {_degrade_tokens(sc)}"
        _stopped(sc)
        sc.wait_for(lambda: "AUTO.LAND" in [r.custom_mode for r in sc.fake.mode_requests],
                    timeout_s=5.0, message="没有请求 AUTO.LAND ⟹ 主动中止任务这一半没生效")
        assert set(_mode_requests_after(sc, n_requests)) <= {"AUTO.LAND"}, (
            f"降级之后只该请求 AUTO.LAND，实际 {_mode_requests_after(sc, n_requests)}")

        # ★ 状态位自己恢复**不算**复位：这是"已解锁的输入失效"，闩到落地上锁
        #   （与低压那条同一个语义：飞机还在天上，不许因为读数变好就自动重新接管）。
        sc.fake.set_pos_horiz_abs(True)
        sc.hold(3.0)
        assert sc.control_state() == "MANUAL/LAND", (
            f"状态位恢复不该让飞机在空中自动接管，实际 {sc.control_state()!r}")
        _stopped(sc, hold_s=0.5)

        # —— ⑤ 复位：落地上锁 ——
        sc.fake.set_state(armed=False)
        assert sc.wait_for_control_state("MANUAL/STANDBY", timeout_s=6.0), (
            f"落地上锁之后应当自动复位回 STANDBY，实际 {sc.control_state()!r}")
        assert sc.wait_for_setpoints(len(sc.setpoints()) + 20, timeout_s=5.0), \
            "复位之后必须重新开始流 setpoint"
        assert sc.dut_alive(), "验收项 ⑦"


def test_未解锁时_EKF2自报位置失效不停发也不降级():
    """这条判据**只在已解锁时**算失效（用户 2026-09-22 裁定）。

    ★ 理由不是"未解锁时不需要保护"，而是**代价不对称**：本判据的代价是**停发 setpoint**，
      而冷启动时"EKF2 还没把位置锚到动捕上"（`pos_horiz_abs` 从假变真）是**常态**
      ⟹ 不加这道门会把台架上每次冷启动的正常窗口变成"节点不发东西了"。
    ⚠️ 代价（刻意接受）：解锁前就坏掉的情形只能等解锁后才被抓住（见 `fsm.hpp::first_degrade`）。
    """
    with Scenario(dut=DUT, params=BASE_PARAMS) as sc:
        assert sc.wait_for_subscribers("estimator_status"), "DUT 没有接上 /mavros/estimator_status"
        assert sc.wait_for_setpoints(20, timeout_s=10.0), "正控：未解锁时也该在流 setpoint"
        assert sc.control_state() == "MANUAL/STANDBY"
        _assert_requests_recorded(sc)
        n_requests = len(sc.fake.mode_requests)
        n_setpoints = len(sc.setpoints())

        sc.fake.set_pos_horiz_abs(False)
        # 1 Hz ⟹ 3.5 s 必然够 2 帧（判据若没有解锁门，这段时间里早就降级了）。
        sc.hold(3.5)

        assert sc.control_state() == "MANUAL/STANDBY", (
            f"未解锁时不该降级，实际 {sc.control_state()!r}")
        assert "ekf2_pos_lost" not in _degrade_tokens(sc), (
            f"未解锁时不该报这条降级，实际 {_degrade_tokens(sc)}")
        assert sc.wait_for_setpoints(n_setpoints + 20, timeout_s=3.0), "不该停发 setpoint"
        assert _mode_requests_after(sc, n_requests) == [], "不该有任何模式请求"
        assert sc.dut_alive()


def test_EKF2位置判据数的是连续帧_不是累计帧():
    """消抖的**另一半**：真帧必须把计数归零（否则判据会变成"累计够 2 帧就报"）。

    ★ 这不是多余的一条：`连续 2 帧` 与 `累计 2 帧` 在"状态位偶尔抖一下"的真实场面上
      行为完全不同 —— 后者会让每次抖动都累加、最终在什么都没坏的时候请求降落。
      两者在**一次**注入下表现一致（都会降级），必须用"假-真-假-真"交替才分得开。
    """
    with Scenario(dut=DUT, params=BASE_PARAMS) as sc:
        _reach_auto(sc)
        n_requests = len(sc.fake.mode_requests)

        for _ in range(3):
            n0 = sc.fake.estimator_status_frames
            sc.fake.set_pos_horiz_abs(False)
            assert sc.poll(lambda: sc.fake.estimator_status_frames > n0, timeout_s=3.0)
            # ★ 两次 `hold(0.35)` **不是**在"等判据成立"（判据数帧、上面已经在数帧了），
            #   而是保证这一轮**恰好一帧**：0.35 s < 上游周期 1 s ⟹ 这段里至多发 1 帧假
            #   （留 0.65 s 余量）。若这轮发到 2 帧，它就变成"连续 2 帧"、降级是对的，
            #   这条用例反而会**假红**。所以这个数**不能**往 1 s 以上调。
            sc.hold(0.35)
            n1 = sc.fake.estimator_status_frames
            sc.fake.set_pos_horiz_abs(True)
            assert sc.poll(lambda: sc.fake.estimator_status_frames > n1, timeout_s=3.0)
            sc.hold(0.35)

        assert sc.control_state() == "AUTO/TAKEOFF", (
            f"3 次单帧抖动不该降级（判据数的是连续帧），实际 {sc.control_state()!r}")
        assert "ekf2_pos_lost" not in _degrade_tokens(sc), f"实际 {_degrade_tokens(sc)}"
        assert _mode_requests_after(sc, n_requests) == [], "不该有任何模式请求"
        assert sc.dut_alive()


def test_estimator_status静默时_陈旧的坏读数不作数():
    """`/mavros/estimator_status` 静默 ⟹ **不许**拿最后一帧的读数说事（`health()` 里的新鲜度门）。

    ★ 构造手法（这道门只在"已解锁且还没闩锁"的窗口里可观测，所以顺序是刻意的）：
      1. **未解锁**时注入 `pos_horiz_abs=False` 并等够 2 帧 —— 计数已经够了，但解锁门挡着，
         此刻不会降级（这正是上一条用例证明的）；
      2. 让话题**静默**（> `estimator_timeout_s` = 2.5 s）；
      3. 走**真实的解锁路径**（`_reach_auto`，全程 ≥ 3 s ⟹ 到时读数必然已经过期）；
      4. 断言没有报 `ekf2_pos_lost`。
    ★ 判据的意义：这条话题里那个状态位在 PX4 侧是**每次发布现算**的（`ekf/ekf_helper.cpp:761`）
      ⟹ 它只在**帧还在到**的时候才自清。让一个陈旧的"假"一直挂着，等于把"我们不知道"
      说成"我们知道它坏了"（与"没收到电池不是低压"同一条纪律）。
    ⚠️ 代价（刻意接受）：这条话题单独静默时本判据**失明** —— 那时没有更好的信号，
      真兜底是动捕那两条判据（它们不依赖这个话题）。
    """
    with Scenario(dut=DUT, params=BASE_PARAMS) as sc:
        assert sc.wait_for_subscribers("estimator_status"), "DUT 没有接上 /mavros/estimator_status"
        assert sc.wait_for_setpoints(20, timeout_s=10.0), "正控：未解锁时也该在流 setpoint"

        # ① 未解锁时攒够 2 帧假（解锁门挡着 ⟹ 不降级）。
        sc.fake.set_pos_horiz_abs(False)
        n0 = sc.fake.estimator_status_frames
        assert sc.poll(lambda: sc.fake.estimator_status_frames >= n0 + 2, timeout_s=5.0), \
            "1 Hz 上游没能在 5 s 内发出 2 帧"
        assert sc.control_state() == "MANUAL/STANDBY", (
            f"未解锁时不该降级，实际 {sc.control_state()!r}")

        # ② 话题静默 —— 从这一刻起"最后一帧的假"就是陈旧读数。
        #    先静默到**必然过期**（> estimator_timeout_s = 2.5 s）再往下走：否则后面
        #    "解锁那一刻读数还新鲜"会让这条用例变得看时序（夹具的 1 Hz 与判据只差 1 s 出头）。
        sc.fake.mute("estimator_status")
        sc.hold(3.0)

        # ③ 真实解锁（耗时 ≥ 3 s > estimator_timeout_s ⟹ 读数必然过期）。
        _reach_auto(sc)

        # ④ 陈旧读数不许再被拿来降级。
        sc.hold(2.0)
        assert "ekf2_pos_lost" not in _degrade_tokens(sc), (
            f"话题已静默，判据不该拿陈旧的读数说事，实际 {_degrade_tokens(sc)}")
        assert sc.control_state() == "AUTO/TAKEOFF", (
            f"陈旧读数不该让我们中止任务，实际 {sc.control_state()!r}")
        assert sc.dut_alive()


# ─────────────────────── ⑤ 未解锁时的失效：只停发，不请求 LAND ───────────────────────

def test_未解锁时输入失效只停发_判据恢复即自动复位():
    """⑤ 的另一半，也是本张最反直觉的一条：**未解锁**时的失效只停发、**不请求 `AUTO.LAND`**。

    两条第一手依据（用户 2026-09-22 裁定，见 `fsm.hpp` 文件头）：
      · `AUTO.LAND` 在 `mode_req_prevent_arming` 名单里（`mode_requirements.cpp:142`）
        ⟹ 未解锁时切进去会让台架上**解不了锁**；
      · 未解锁时"飞机没在飞"，停发就是全部该做的事。

    ★ 而且未解锁的失效**不闩锁**：判据一恢复就自动回 STANDBY（不必落地上锁，因为没有"落地"）。
    """
    with Scenario(dut=DUT, params=BASE_PARAMS) as sc:
        # 未解锁、守卫全成立 ⟹ 已经进 STANDBY 并在流（正控）。
        assert sc.wait_for_subscribers("state")
        assert sc.wait_for_setpoints(20, timeout_s=10.0), "正控：未解锁时也该在流 setpoint"
        assert sc.control_state() == "MANUAL/STANDBY"
        _assert_requests_recorded(sc)
        n_requests = len(sc.fake.mode_requests)

        sc.mocap_source.pause()
        assert sc.wait_for_control_state("MANUAL/INIT", timeout_s=5.0), (
            f"动捕失效之后应当停发并退回 INIT，实际 {sc.control_state()!r}")
        _wait_degrade(sc, "mocap_timeout")
        _stopped(sc)
        sc.hold(1.0)
        assert "AUTO.LAND" not in [r.custom_mode for r in sc.fake.mode_requests], (
            "未解锁时**不该**请求 AUTO.LAND（它会阻止解锁，台架上就解不了锁了）")
        assert sc.fake.mode != "AUTO.LAND"

        # —— 复位：判据恢复 ⟹ 自动回 STANDBY（不需要任何人为动作）——
        sc.mocap_source.resume()
        assert sc.wait_for_control_state("MANUAL/STANDBY", timeout_s=6.0), (
            f"动捕恢复之后应当自动复位，实际 {sc.control_state()!r}")
        assert sc.wait_for_setpoints(len(sc.setpoints()) + 20, timeout_s=5.0), "复位之后要重新流"
        assert "none" in _degrade_tokens(sc), f"复位要可检索，实际 {_degrade_tokens(sc)}"
        assert _mode_requests_after(sc, n_requests) == [], "整个过程不该有新的模式请求"
        assert sc.dut_alive()


# ─── ⑤ 复位判据的边界：mavros 的**合成断连帧**不算"落地上锁"（ticket 18） ───

def _latch_then_synth_disconnect(sc):
    """把场景推到"已解锁 ⟹ 链路静默 ⟹ 已闩锁在 LAND"，然后让夹具发**那一帧**。

    这条时间线是**真机的形状**（ticket 18 的证据链，mavros 2.15.1 @ `22ae5b7`，第一手核实）：
      · HEARTBEAT 停 ⟹ `/mavros/state` 只剩最后一帧（mavros 只在收到 HEARTBEAT 时发这个话题）；
      · +`state_timeout_s`（2.5 s）⟹ 本节点闩锁 `kArmedDegrade`，停发 + 请求 `AUTO.LAND`；
      · +`conn_timeout`（mavros 参数，默认 **10.0 s**）⟹ mavros **自己**发一帧
        `connected=false, armed=false, guided=false, manual_input=false, mode=""` —— `sys_status.cpp:903-911` 的
        `publish_disconnection()`，由 `:1411` 的 `connection_cb(false)` 触发。

    ★ 台架上把 10 s 直接压成"请夹具发这一帧"（`fake_mavros.synth_disconnect()`）：夹具**不假扮**
      mavros 的 `conn_timeout` 计时逻辑，只假扮那一帧的**内容与时机**。所以本文件证明的是"收到
      这一帧之后本节点怎么走"，不是"mavros 会在 10 s 后发它"（后者只能第一手读源码，票面有）。

    返回"请求 AUTO.LAND 之后"的模式请求条数（给后面断言"没有新的 OFFBOARD"当基线）。

    ⚠️ **这一帧到达时无法断言"闩锁还在"** —— 那帧之后状态话题上唯一可见的是
      `connected=false ∧ armed=false`，而按 ⑤ 的处置，当时的对外状态就是 `MANUAL/INIT`：
      **改前改后同形**（改前 = 被这帧清掉闩锁后回 INIT；改后 = 闩锁还在但 `armed` 为假 ⟹ 也停在
      INIT）。区别只在**链路恢复那一刻**才外显 ⟹ 判别性断言全在恢复之后，见下面两条用例。
    """
    _reach_auto(sc)
    _assert_requests_recorded(sc)
    n_before = len(sc.fake.mode_requests)

    # ① 链路静默 ⟹ 停发 + 请求 AUTO.LAND。这一段是 ticket 06 已验的行为，这里当**正控**用：
    #    没有它，后面那句"仍然在 LAND"可能只是"压根没闩锁过"（假绿比红危险）。
    sc.fake.mute("state")
    assert sc.wait_for_control_state("MANUAL/LAND", timeout_s=8.0), (
        f"链路静默之后应当进 LAND（已解锁 ⟹ 主动中止任务），实际 {sc.control_state()!r}")
    _wait_degrade(sc, "state_timeout")
    _stopped(sc)
    assert "AUTO.LAND" in [r.custom_mode for r in sc.fake.mode_requests[n_before:]], (
        "正控：闩锁期间应当请求过 AUTO.LAND")
    n_requests_at_latch = len(sc.fake.mode_requests)

    # ② mavros 自己发的那一帧（真机上是 10 s 之后）。它只发一帧，此后 state 继续静默。
    sc.fake.synth_disconnect()
    sc.hold(2.0)
    return n_requests_at_latch


def test_合成断连帧之后链路恢复且_PX4_仍_armed_仍走_LAND_不重新接管():
    """ticket 18 的本体：合成断连帧**不算落地上锁** ⟹ 链路恢复时闩锁还在、任务仍在中止中。

    链路回来时 PX4 很可能**仍然 armed**（我们停发之后它自己退出了 OFFBOARD，但没落地）——
    这正是"已解锁的输入失效要落地上锁才复位"那条闩锁存在的理由：此刻回 STANDBY 并重新请求
    OFFBOARD，就等于**我们的失效保护在失效之后自己撤销了**。判别力在头两条断言上
    （`MANUAL/LAND` 仍然在 + 继续请求 `AUTO.LAND`；改回原状 ⟹ 节点停在 `MANUAL/INIT`）。

    ⚠️ 下面"没有请求 OFFBOARD"那一条**没有判别力**（三种实现都成立）：拦住自主接管的是
      `kArmingByUs` 守卫 —— 合成帧那个假的 `armed=false` 顺手清掉了归属标记
      （`px4_interface.cpp:160-164`）⟹ 链路恢复时的上升沿被记成"别人解锁"。那是**撞上的**
      安全网、不是本票设计的；这条断言钉的是**性质**（以后别让它漏出去），不是"修好了"的证据。

    ★ 与下面那条**成对**：只有两条都绿，"合成断连帧不算落地上锁，但真落地上锁仍然算"这句话
      才是完整的（只拦合成帧 ⟹ 真上锁也复位不了 = 单向 latch，ticket 06 验收项 ⑤ 禁止）。
    """
    with Scenario(dut=DUT, params=BASE_PARAMS) as sc:
        n_requests_at_latch = _latch_then_synth_disconnect(sc)

        # ③ 链路恢复：mavros 重连、HEARTBEAT 回来，而飞机**还没上锁**。
        #    ★ 模式显式写 POSCTL：真机上我们那几帧 `AUTO.LAND` 请求**根本没到** PX4（链路是断的），
        #      而假 mavros 的 set_mode 服务在静默期间照样答应成功（它不假扮"链路断"）⟹ 必须以
        #      "重连时 PX4 的真实模式"为准把它覆盖一次，否则测的是夹具的一厢情愿。
        sc.fake.reconnect_state(connected=True, armed=True, mode="POSCTL")

        assert sc.wait_for_control_state("MANUAL/LAND", timeout_s=6.0), (
            f"链路恢复时闩锁必须还在 ⟹ 仍是 LAND（任务已中止），实际 {sc.control_state()!r}"
            "—— 回 MANUAL/INIT 或 MANUAL/STANDBY 都说明闩锁被那帧合成断连帧清掉了")
        # 还在 LAND 的实现细节之一：PX4 此刻在 POSCTL ⟹ 不重新请求就没人让它降落。
        # ★ 用 `sc.poll()` 而**不是** `sc.wait_for()`：后者成功时返回 `None` ⟹ 写进 `assert`
        #   恒假（本文件里 `wait_for` 一律当语句用，`poll` 才是断言用的那个）。
        assert sc.poll(lambda: "AUTO.LAND" in [r.custom_mode
                                               for r in sc.fake.mode_requests[n_requests_at_latch:]],
                       timeout_s=6.0), "闩锁还在 ⟹ 应当继续请求 AUTO.LAND（PX4 此刻在 POSCTL）"
        _stopped(sc, hold_s=1.0)
        # ★ 而**从头到尾**不许再请求 OFFBOARD —— 见用例开头那条 ⚠️：这条**没有判别力**
        #   （改回原状也成立），它钉的是"别让自主接管漏出去"这个性质。
        #   ★ 放在 `_stopped()` 之后：那 1 s 是按"复位后多久会重新请求"的量级取的
        #     （前置流 `offboard_prestream_s` = 1 s），所以这条负断言真的能看见它。
        assert "OFFBOARD" not in [r.custom_mode
                                  for r in sc.fake.mode_requests[n_requests_at_latch:]], (
            f"不许自主重新接管，实际请求了 {_mode_requests_after(sc, n_requests_at_latch)}")
        # 不是昙花一现：静了 1 s 之后**还是** LAND（闩锁不会被"等一会儿"熬掉）。
        assert sc.control_state() == "MANUAL/LAND", (
            f"恢复之后应当**稳定**停在 LAND，实际 {sc.control_state()!r}")
        assert sc.dut_alive(), "验收项 ⑦"


def test_合成断连帧之后链路恢复且已落地上锁_仍然复位():
    """★ 另一半，也是**防单向 latch** 的那一半（ticket 06 验收项 ⑤）。

    同一条时间线，只是链路回来时飞机**真的已经在地面上并已上锁**（`connected=true ∧ armed=false`
    —— 这才是"可信的落地上锁"）。闩锁必须在这里复位：回 STANDBY、重新流 setpoint、重新请求
    OFFBOARD，等下一次起飞命令。**只能重启节点才能退出去的降级是本项目的禁忌。**

    ⚠️ 它拦的是**半边修复**：只在复位判据里加 `connected`、却不同时冻结 `was_armed_`
      （那帧合成状态里的 `armed=false` 会把降沿吃掉），恢复时"真的上锁"这个降沿就**对不上边**
      ⟹ 闩锁永不复位，节点永远停在 `MANUAL/INIT`。seam 2 有一条同名用例（`test_fsm.cpp`），
      改一半时就是它先红。
      ★ 注意：**整个修复都撤掉**时这条反而是绿的（那帧伪造的降沿把闩锁清了，于是"照常复位"）
        —— 所以判别性最强的是上面那条；这条钉的是"别把另一半弄丢"。
    """
    with Scenario(dut=DUT, params=BASE_PARAMS) as sc:
        n_requests_at_latch = _latch_then_synth_disconnect(sc)

        sc.fake.reconnect_state(connected=True, armed=False, mode="POSCTL")
        assert sc.wait_for_control_state("MANUAL/STANDBY", timeout_s=6.0), (
            f"链路恢复且飞机已上锁 ⟹ 必须复位回 STANDBY，实际 {sc.control_state()!r}"
            "（停在 MANUAL/INIT = 闩锁变成单向的了，只能重启节点）")
        assert sc.wait_for_setpoints(len(sc.setpoints()) + 20, timeout_s=6.0), "复位之后要重新流"
        assert "none" in _degrade_tokens(sc), f"复位要可检索，实际 {_degrade_tokens(sc)}"
        # 正控：节点**真的**回到了正常工作（前置流 1 s 之后重新请求 OFFBOARD），
        # 不是"停在 STANDBY 但已经不干活了"。
        assert sc.poll(lambda: "OFFBOARD" in [r.custom_mode
                                              for r in sc.fake.mode_requests[n_requests_at_latch:]],
                       timeout_s=8.0), "复位之后应当重新请求 OFFBOARD（正常路径回来了）"
        assert sc.dut_alive(), "验收项 ⑦"


# ─────────────────────── ⑥ 动捕时间戳：冻住 vs 单次倒退 ───────────────────────

def test_动捕时间戳冻住会降级_而短促倒退不会():
    """⑥ 的"注入时间戳倒退"。

    ★★ 这两半必须一起测，因为它们证明的是**判据的粗细**：
      · `hold_stamp`（帧照发、戳不前进 ⟹ mavros 与 EKF2 都会丢弃）**必须**降级；
      · **短促**倒退（0.2 s，短于 `mocap_stamp_stall_s` = 0.5 s）**不该**降级 ——
        把一次 NTP 校时判成失效，就会在真机上因为一次无害的时钟跳变把飞机打下来。
    ★ 判据的基准是**历史最大戳**（与 EKF2 同一把尺子，`EKF/estimator_interface.cpp:393`）⟹
      倒退窗口里**每一帧都没超过最大戳**（哪怕它自己在一帧帧地前进）。所以窗口一拖长就会降级，
      那条用例紧挨在这条后面（`rewind` 的窗口形状值得单独看一眼，见 `vrpn_source.py:135`）。
    """
    with Scenario(dut=DUT, params=BASE_PARAMS) as sc:
        _reach_auto(sc)
        n_requests = len(sc.fake.mode_requests)
        assert "mocap_stamp_stalled" not in _degrade_tokens(sc), "正控：注入之前不该有这条降级"

        # ① 短促倒退：窗口 0.2 s < 0.5 s 判据 ⟹ 只丢几帧，不降级（窗口一结束戳就跳回当前时刻）。
        #    ★ 先起一个旁路订阅当**正控**：只有话题上真出现了向后跳变，
        #      下面"不降级"这句话才是在说我们的判据，而不是在说"注入没生效"。
        stamps, sub = _watch_mocap_stamps(sc)
        sc.mocap_source.rewind(5.0, hold_s=0.2)
        sc.hold(1.5)
        sc.node.destroy_subscription(sub)
        deltas = _stamp_deltas(stamps)
        assert deltas and min(deltas) < -1.0, (
            f"正控失败：注入之后话题上没出现向后跳变（戳差最小 {min(deltas) if deltas else '没收到帧'}）"
            f"⟹ 注入没生效，下面「不降级」的断言不成立")
        assert "mocap_stamp_stalled" not in _degrade_tokens(sc), (
            f"短促时间戳倒退不该判成失效（它只让 mavros 丢几帧）；实际降级：{_degrade_tokens(sc)}")
        assert sc.control_state() == "AUTO/TAKEOFF", "倒退之后应当仍在正常流（任务没被打断）"

        # ② 冻住（戳一直不前进）⟹ 0.5 s 之后降级。
        sc.timeline.extend(hold_mocap_stamp(sc.mocap_source, sc.t() + 0.2, held=True))
        assert sc.wait_for_control_state("MANUAL/LAND", timeout_s=6.0), (
            f"时间戳冻住应当降级中止任务，实际 {sc.control_state()!r}")
        _wait_degrade(sc, "mocap_stamp_stalled")
        assert _degrade_tokens(sc)[0] == "mocap_stamp_stalled", f"实际 {_degrade_tokens(sc)}"
        _stopped(sc)
        assert sc.wait_for_subscribers("state"), "（正控：DUT 进程仍在）"
        assert sc.dut_alive()


def test_动捕时间戳倒退到旧纪元_持续超过判据即降级():
    """⑥ 的另一半 —— 这条才是"倒退"真正的危险形状（2026-09-22 复核补的用例）。

    `rewind` 的窗口里戳**仍然在一帧帧地前进**，只是整段落在 5 s 之前 ⟹ 拿"上一帧的戳"当基准
    的判据**永远说健康**；而 PX4 那边每一帧都在丢 —— EKF2 只收"比已缓冲的最新样本还新"的样本
    （`EKF/estimator_interface.cpp:393`，丢弃时只打一条 `ECL_WARN("EV data too fast")`）。
    于是话题层面**每一帧都新鲜**、估计器却**一帧都没用上**，这正是本判据存在的理由。
    ★ 基准改成历史最大戳之后，窗口 > `mocap_stamp_stall_s` 就必然降级 —— 这条钉住它。

    ★★ 这条的**判别力是实测过的**（2026-09-22，不是推演）：把 `on_mocap` 临时改回
      "基准 = 上一帧的戳、每帧无条件更新"，本节两条用例一个失败一个通过 ——
      本条失败（DUT 全程停在 `AUTO/TAKEOFF`，一条降级都没有），冻住那条仍通过。
      同时用探针量到窗口内的真实戳：跳变 −4.9955 s 之后**每帧前进 +5 ms**，
      而旧基准就停在倒退前的最大戳上不动 ⟹ 旧判据在窗口里每一帧都判"健康"。
    """
    with Scenario(dut=DUT, params=BASE_PARAMS) as sc:
        _reach_auto(sc)
        assert "mocap_stamp_stalled" not in _degrade_tokens(sc), "正控：注入之前不该有这条降级"

        # 倒退窗口 1.0 s > 判据 0.5 s：中途每一帧都在前进，但一帧都没超过历史最大戳。
        sc.mocap_source.rewind(5.0, hold_s=1.0)
        assert sc.wait_for_control_state("MANUAL/LAND", timeout_s=6.0), (
            f"戳停在旧纪元应当降级（这些帧在 EKF2 眼里一帧都收不到），实际 {sc.control_state()!r}")
        _wait_degrade(sc, "mocap_stamp_stalled")
        assert _degrade_tokens(sc)[0] == "mocap_stamp_stalled", (
            f"根因应当是动捕戳不前进（不是 mocap_timeout —— 帧一直在到），实际 {_degrade_tokens(sc)}")
        _stopped(sc)
        assert sc.dut_alive()


# ─────────────────────── ⑦ 节点在"什么都坏掉"时不崩 ───────────────────────

def test_假_mavros_的输入同时静默_节点不崩且报出最根本的原因():
    """⑦ 验收项：任何失效下节点**不崩**（崩了会停发心跳，进而触发 PX4 自己的失效保护）。

    ★ 一次把假 mavros 的**五条**输入全部掐掉，同时把 `set_mode` 服务也撤走
      （真机上"mavros 重启"就是这个样子）：这是最接近"整条链路没了"的场面，
      节点只该不停地报判据，不该退出。
    ⚠️ 动捕**不在这五条里**（`FakeMavros.SILENCEABLE` 覆盖不到它 —— 它是另一个发布方
      `VrpnSource`）⟹ 动捕那一条判据由 ② 的用例单独负责，这里测的是另外四条。
    ★ 顺带钉住两条判据的**时序与优先级**：0.5 s 的判据先到（local_position），
      2.5 s 的 state 后到，而一旦飞控失联成立，它每轮都排在最前面（`first_degrade` 的顺序）。
    """
    from bench.fake_mavros import FakeMavros

    with Scenario(dut=DUT, params=BASE_PARAMS) as sc:
        assert sc.wait_for_subscribers("state")
        assert sc.wait_for_setpoints(20, timeout_s=10.0), "正控：此前在流"

        sc.fake.set_mode_policy("absent")
        sc.fake.mute(*FakeMavros.SILENCEABLE)
        sc.hold(4.0)   # 远超最长的判据（state 2.5 s）

        assert sc.dut_alive(), "所有输入同时消失时节点**不许退出**"
        assert sc.dut_returncode() is None
        assert sc.control_state() == "MANUAL/INIT", (
            f"守卫全失效 ⟹ 应当退回 INIT，实际 {sc.control_state()!r}")
        _stopped(sc)
        assert _degrade_tokens(sc) == ["local_pos_timeout", "state_timeout"], (
            f"应当先报 0.5 s 的 local_pos_timeout、再让 2.5 s 的 state_timeout 接管"
            f"（它更根本，所以在 `first_degrade` 的顺序里排在前面），实际 {_degrade_tokens(sc)}")
        # ⚠️ 这里**故意不断言** `mode_requests`：「未解锁 ⟹ 不请求 LAND」在本用例里**不可证** ——
        #   服务已经被撤走，我们根本发不出任何模式请求（假 mavros 自然什么都没记到），
        #   断言它等于断言"零 == 零"。有服务在、口径相同的那条在
        #   `test_未解锁时输入失效只停发_判据恢复即自动复位` 里。本用例负责的是 ⑦ 本身：
        assert "静默" in sc.dut_log(), (
            "全链路死掉时节点仍应**持续产出证据**（谁静默了多久）—— 否则 ⑦ 只证明了「没崩」")


def test_通道数不足的遥控帧不崩():
    """⑦ 的一个具体角落：**接收机的通道数可以比我们读的通道号少**。

    真机换接收机 / 改 `RC_MAP_*` 之后很容易出现这种情况 ⟹ 读通道号时要"跳过"而不是"越界"。
    这条把那个 `continue` 变成会红的一行断言（否则它的下场是"天上第一次 segfault"）。
    """
    with Scenario(dut=DUT, params=BASE_PARAMS) as sc:
        assert sc.wait_for_setpoints(20, timeout_s=10.0), "正控：此前在流"

        sc.fake.set_rc([1500, 1500])       # 只有 2 通道，而 rc_map_throttle/yaw 指的是 3/4
        sc.hold(2.0)
        assert sc.dut_alive(), "通道数不足时节点崩了 ⟹ 必须修（真机上会因此停发心跳）"
        assert sc.setpoints(), "而且应当照常流 setpoint（少几个通道不影响位置控制）"
        assert sc.control_state() == "MANUAL/STANDBY"


# ─────────────────────── §H.6 外部解锁（ticket 08 起是**真的**在追踪归属） ───────────────────────

def test_外部解锁只停发不请求_LAND():
    """★ 这条守的是 `Guard::kArmingByUs` 的反面：

    **没经过** `/control/takeoff` 的 `armed` 上升沿 ⟹ 是外面干的（QGC / 遥控器拨杆 / 别人）
    ⟹ 按 `params-changes.md` §H.6 停发 + 告警，而**不是**当作"我们飞起来了"去请求降落
    （飞机可能是别人控制着的，我们不该动它的模式）。

    ★★ ticket 08 之前这条是靠 `own_arming=false` 这个**参数**表达的（一条台架缝）；
      现在是**真实追踪**：`/mavros/cmd/arming` 的肯定应答才产生归属，之后观察到的
      第一个 `armed` 上升沿才算我们的（`px4_interface.cpp::on_state`）。
      所以本用例的注入面从"改参数"变成了"**不调用**那个服务"—— 语义完全一样，
      但它现在测的是生产路径上真的会跑的那段代码。
    """
    with Scenario(dut=DUT, params=BASE_PARAMS) as sc:
        assert sc.wait_for_setpoints(20, timeout_s=10.0), "正控：未解锁时在流 setpoint"
        _assert_requests_recorded(sc)
        n_requests = len(sc.fake.mode_requests)

        sc.fake.set_state(armed=True)
        assert sc.wait_for_control_state("MANUAL/INIT", timeout_s=6.0), (
            f"外部解锁应当停发并退回 INIT，实际 {sc.control_state()!r}")
        _stopped(sc)
        sc.hold(2.0)
        assert _mode_requests_after(sc, n_requests) == [], (
            f"外部解锁时不该请求任何模式，实际 {_mode_requests_after(sc, n_requests)}")
        assert "外部解锁" in sc.dut_log(), "应当有一条能让人看懂的告警（「这是谁解锁的？」）"
        assert sc.dut_alive()
