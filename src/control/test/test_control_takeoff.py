"""control 节点的起飞 / 悬停 / 自动降落（**seam 1**）—— ticket 08 的验收清单。

★★ 本文件与 `test_control_standby.py` 的**唯一**差别是：**飞机开始动了**。
   ticket 05 验的是"发出去的帧对不对"，位置目标就是当前位姿，所以夹具**不许**动飞机；
   ticket 08 的判据（爬到 1 m、位置误差 < 10 cm、速率 < 0.1 m/s）**全是关于飞机状态的**
   ⟹ 必须在夹具上再补一层"假飞机"（见本文件末尾的 `_Plant`）。没有它：位置恒为 0
   ⟹ "爬到 1 m"永远不会发生；速度恒为 0 ⟹ `|v| < 0.10` **恒真** ⟹ 测试会全绿而毫无意义。
   ⚠️ 这一层刻意留在**本文件**里，没有塞进 `bench`：它不是"某个接口假得像不像"，
     而是"这一轮想让飞机怎么飞"的场景逻辑（与 `Timeline` 的时机编排同类）。
     ticket 09/10 真复用时再提升进 `bench` —— 那时它才够格有自己的自检。

验收项 → 用例（工单 `.scratch/phase1/issues/08-takeoff-hover-land.md`）：
  · 解锁（守卫放行后、**一次**、被拒不重试）  → 冒烟 / 遥控不在线 / 解锁被拒 / 任务在跑
  · 以 ≤0.5 m/s 线性爬到 1 m                → 冒烟 / 爬升速率 / （水平钉住见爬升速率那条）
  · 稳定判据 `位置误差 ∧ 速率` 两条都要        → 速率那一半 / 位置那一半（各带正控）
  · 稳定成立那一刻记**轨迹原点**且日志可复核   → 冒烟（解析 `traj_origin=` 与目标对比）
  · 任务结束自动 `AUTO.LAND`，上锁交给 PX4     → 冒烟
  · 起飞不转向（yaw = 起飞机头，锁死）        → 机头锁定

★ 关于负向断言：每一条"没发生"都配了一个**正控**（"改回来之后必须发生"）。
  只有负向断言的用例，"DUT 启动就崩了"或"场景根本没跑起来"都会让它变绿。
"""

import re

import pytest
from std_srvs.srv import Trigger

from bench import Scenario

#: DUT =（包名，可执行名）—— 由 `ament_index_python` 解析成 <prefix>/lib/control/control_node
DUT = ("control", "control_node")

#: 与 `config/control.yaml` 一致的参数（这里显式写出来，让"这条判据在什么口径下成立"
#: 在断言旁边就能读到）。⚠️ 只把**时间**相关的改成小值好让用例跑快些，
#: 判据阈值（10 cm / 0.1 m/s / 0.5 m/s）一律照工单原值 —— 改它们就不是在验工单了。
BASE_PARAMS = {
    "setpoint_rate_hz": 50.0,
    "enter_offboard": True,
    "offboard_prestream_s": 1.0,
    "offboard_retry_s": 1.0,
    "state_timeout_s": 2.5,
    "local_pos_timeout_s": 0.5,
    "mocap_timeout_s": 0.5,
    # —— ticket 08 的任务判据 ——
    "takeoff_altitude_m": 1.0,
    "climb_speed_mps": 0.5,
    "takeoff_alt_tol_m": 0.10,
    "takeoff_vel_tol_mps": 0.10,
    "takeoff_settle_s": 1.0,
    "hover_pos_tol_m": 0.10,
    "hover_vel_tol_mps": 0.10,
    "hover_settle_s": 2.0,
    # ★ 工单的自由飞（阶梯第 4 级）要求保持 60 s；台架只验"保持满 ⟹ 自动降落"这条**逻辑**，
    #   60 s 那一条是人的物理步骤（见工单的 Comments）。
    "hover_hold_s": 3.0,
    "takeoff_timeout_s": 5.0,
}

TAKEOFF_SRV = "/control/takeoff"


# ───────────────────────────── 小工具 ─────────────────────────────

def _reach_offboard(sc, timeout_s=20.0):
    """把场景推到"守卫全成立、正在流 setpoint、**并且**已在 OFFBOARD"。

    ★ 最后那个 `hold(1.2)` 是必须的，不是保险：解锁的前置之一是 `guards.in_offboard`，
      而它来自 `/mavros/state`（**1 Hz**）⟹ 夹具切完模式之后，DUT 那边最多还要 1 s 才看得见。
      不等的话，"解锁请求没发出去"会被误读成"前置判据写错了"（现场最贵的就是这个方向搞反）。
    """
    assert sc.wait_for_subscribers("state"), "DUT 没有接上 /mavros/state"
    assert sc.wait_for_subscribers("local_position"), "DUT 没有接上 /mavros/local_position/pose"
    assert sc.wait_for_subscribers("local_velocity"), \
        "DUT 没有接上 /mavros/local_position/velocity_local（稳定判据的速率来源）"
    assert sc.wait_for_subscribers("rc"), "DUT 没有接上 /mavros/rc/in"
    assert sc.wait_for_subscribers("battery"), "DUT 没有接上 /mavros/battery"
    assert sc.poll(lambda: sc.mocap_source.matched_subscribers() >= 1), "DUT 没有接上动捕话题"
    assert sc.wait_for_setpoints(20, timeout_s=timeout_s), \
        "守卫成立之后应当开始以 50 Hz 流 setpoint"
    sc.wait_for(lambda: sc.fake.mode == "OFFBOARD", timeout_s=8.0,
                message="DUT 没有把（假的）PX4 拉进 OFFBOARD")
    sc.hold(1.2)


def _takeoff(sc, timeout_s=6.0):
    """调一次 `/control/takeoff`，返回服务应答（`std_srvs/Trigger`，请求无字段）。"""
    client = sc.node.create_client(Trigger, TAKEOFF_SRV)
    assert client.wait_for_service(timeout_sec=timeout_s), (
        f"DUT 没有提供 {TAKEOFF_SRV}（服务名或类型对不上？）")
    future = client.call_async(Trigger.Request())
    sc.wait_for(lambda: future.done(), timeout_s=timeout_s,
                message=f"{TAKEOFF_SRV} 没有回应")
    resp = future.result()
    sc.node.destroy_client(client)
    return resp


def _fly(sc, plant, seconds: float, dt: float = 0.02) -> None:
    """推进 `seconds` 秒，途中让假飞机跟着 setpoint 动。★ 别用 `sc.hold` —— 那个不动飞机。"""
    end = sc.t() + seconds
    while sc.t() < end:
        plant.step()
        sc.advance_to(min(sc.t() + dt, end))


def _fly_until(sc, plant, predicate, timeout_s: float = 15.0, message: str = "") -> None:
    """一边让假飞机飞、一边等判据成立（等价于 `Scenario.wait_for`，但飞机在动）。

    超时与 `BenchTimeout` 一样带上 `evidence()`：台架上"为什么没到"的答案几乎总在记录里。
    """
    deadline = sc.t() + timeout_s
    while True:
        if predicate():
            return
        if sc.t() >= deadline:
            raise AssertionError(
                (message or "等待判据超时") + f"（等了 {timeout_s:g} s）\n\n" + sc.evidence())
        plant.step()
        sc.advance_to(min(sc.t() + 0.02, deadline))


_ORIGIN_RE = re.compile(r"(takeoff_origin|traj_origin)=\(([-0-9.]+), ([-0-9.]+), ([-0-9.]+)\)")


def _origins_in_log(sc):
    """把 DUT 日志里的 `takeoff_origin=(x, y, z)` / `traj_origin=(x, y, z)` 都解析出来。

    ★ **这条解析本身就是一条验收项**（工单："轨迹原点必须能在日志里复核"）：
      `format_vec3` 用固定形状 `(%.3f, %.3f, %.3f)` 就是为了让现场脚本能直接抓。
      解析不出来 ⟹ 那条验收项不成立，而"日志里看着有"不算数。
    """
    out = {}
    for m in _ORIGIN_RE.finditer(sc.dut_log()):
        out.setdefault(m.group(1), []).append(
            (float(m.group(2)), float(m.group(3)), float(m.group(4))))
    return out


def _climb_start(sc) -> int:
    """第一个"目标 z 比上一帧高"的 setpoint 的下标减一（即爬升的**起点**帧）。

    ★ 断言里带 `evidence()`：这条失败的信息量全在"到底发了什么 setpoint"上。
    """
    sps = sc.setpoints()
    for i in range(1, len(sps)):
        if sps[i].msg.position.z > sps[i - 1].msg.position.z + 1e-9:
            return i - 1
    raise AssertionError(
        "setpoint 的目标 z 从来没有上升过 ⟹ 爬升根本没发生（这条断言本身也是正控）\n\n"
        + sc.evidence())


def _climb_rate_mps(sc, min_window_s: float = 0.3) -> float:
    """爬升段里**目标 z** 相对爬升起点的最大平均速率（m/s）。

    ★★ 只在**跨度足够大的窗口**上算，绝不看相邻两帧：rcl 的定时器是"从回调开始重新计时"
      （`rcl/timer.h`）⟹ 50 Hz 的流里本来就会出现 0 ms 的间隔与 40 ms 的空洞，
      相邻两帧算出来的速率可以偏好几倍，而**平均**速率仍然准（同 `Scenario.setpoint_rate_hz`）。
      用 `at_s`（到达时刻）当尺子时这一点更严重：它比 DUT 自己的时钟多一层抖动。
    """
    sps = sc.setpoints()
    start = _climb_start(sc)
    t0, z0 = sps[start].at_s, sps[start].msg.position.z
    rates = [(s.msg.position.z - z0) / (s.at_s - t0)
             for s in sps[start:] if (s.at_s - t0) >= min_window_s]
    assert rates, f"爬升段太短（只有 {sps[-1].at_s - sps[start].at_s:.2f} s），算不出速率"
    return max(rates)


def _climb_span(sc):
    """爬升段 setpoint 的目标 z 区间 `(起点, 最高)`。"""
    sps = sc.setpoints()
    start = _climb_start(sc)
    zs = [s.msg.position.z for s in sps[start:]]
    return sps[start], max(zs)


# ───────────────────────────── 冒烟：整条链 ─────────────────────────────

def test_冒烟_解锁_爬升_悬停_判稳记轨迹原点_保持后自动降落():
    """一次把整条链走完：受理 → 解锁 → TAKEOFF → HOVER → WAIT_START → LAND。

    ★ 为什么值得一条用例走完全程（而不是拆成五条）：这条链上**每一环的输出都是下一环的输入**
      （解锁晚 1 s ⟹ 爬升起点变、判稳基准变），拆开测会各自用"手工摆好的初态"绕过这些耦合。
      单点的反例由下面那几条专门的用例负责造。
    """
    with Scenario(dut=DUT, params=BASE_PARAMS) as sc:
        _reach_offboard(sc)
        plant = _Plant(sc)
        assert sc.control_state() == "MANUAL/STANDBY"

        # —— ① 受理 ≠ 起飞 ——
        resp = _takeoff(sc)
        assert resp.success, f"起飞命令被拒：{resp.message}"
        assert sc.control_state() == "MANUAL/STANDBY", (
            "受理之后、**解锁之前**不该进 TAKEOFF：飞机还没被我们驱动，"
            "而 `AUTO` 的定义是「正在发会驱动飞机的指令」")
        # ★ 受理这一帧的目标仍是**当前位置**（原地保持）—— 飞机不该在解锁前就被指令去动。
        #   （解锁请求本身**可以**在受理后的下一帧就发出去：这一刻前置已经齐了，
        #     那正是我们要的"手不停在等"；这条断言管的是"命令 ≠ 动作"。）
        target = sc.setpoints()[-1].msg.position
        assert (target.x, target.y, target.z) == pytest.approx(sc.fake.position, abs=1e-9), (
            "受理之后、解锁之前，setpoint 的目标必须仍是当前位置（原地保持）")

        # —— ② 解锁 ⟹ TAKEOFF（起飞机头 / 原点在那一刻锁定）——
        _fly_until(sc, plant, lambda: sc.control_state() == "AUTO/TAKEOFF", timeout_s=10.0,
                   message="解锁之后没有进 TAKEOFF")
        origins = _origins_in_log(sc)
        assert "takeoff_origin" in origins, (
            "TAKEOFF 的日志里没有 `takeoff_origin=(x, y, z)` ⟹ 起飞原点不可复核"
            "（工单的验收方式就是 grep 这个令牌）")
        tox, toy, toz = origins["takeoff_origin"][-1]
        assert (tox, toy, toz) == pytest.approx((0.0, 0.0, 0.0), abs=0.05), (
            f"起飞原点应当 ≈ 假 EKF 此刻的位置 (0, 0, 0)，实际 {origins['takeoff_origin'][-1]}")

        # —— ③ 爬升：≤0.5 m/s，水平两轴钉在原点 ——
        _fly_until(sc, plant, lambda: sc.control_state() == "AUTO/HOVER", timeout_s=10.0,
                   message="没有进 HOVER（爬到 1 m 并稳住 1 s 之后应当进）")
        first, zmax = _climb_span(sc)
        assert zmax - first.msg.position.z >= 0.95, (
            f"爬升只走了 {zmax - first.msg.position.z:.2f} m，要求 ≥0.95 m（≈ takeoff_altitude_m）")
        assert _climb_rate_mps(sc) <= 0.55, (
            f"爬升速率 {_climb_rate_mps(sc):.3f} m/s 超过 climb_speed_mps = 0.5")
        xs = {round(s.msg.position.x, 9) for s in sc.setpoints() if s.at_s >= first.at_s}
        ys = {round(s.msg.position.y, 9) for s in sc.setpoints() if s.at_s >= first.at_s}
        assert len(xs) == 1 and len(ys) == 1, (
            f"爬升期间水平两轴必须**钉在原点**（垂直起飞，不斜着走），实际 x 有 {len(xs)} 个值、"
            f"y 有 {len(ys)} 个值")

        # —— ④ 悬停 ⟹ 判稳 ⟹ 记录**轨迹原点** ——
        _fly_until(sc, plant, lambda: sc.control_state() == "AUTO/WAIT_START", timeout_s=10.0,
                   message="稳定判据始终没有成立（位置误差 <10 cm ∧ 速率 <0.1 m/s 持续 2 s）")
        origins = _origins_in_log(sc)
        assert "traj_origin" in origins, (
            "判稳的日志里没有 `traj_origin=(x, y, z)` ⟹ 工单那条"
            "「轨迹原点必须能在日志里复核」不成立")
        tx, ty, tz = origins["traj_origin"][-1]
        # ★ 轨迹原点 = 判稳那一刻的位置 ≈ 悬停点（= 起飞原点正上方 1 m）。
        #   两者相差不超过一个 `hover_pos_tol_m`（判据保证），所以容差取它。
        assert tz == pytest.approx(toz + 1.0, abs=0.10), \
            f"轨迹原点的 z 应当 ≈ 起飞原点 +1 m，实际 {tz} vs {toz + 1.0}"
        assert tx == pytest.approx(tox, abs=0.10) and ty == pytest.approx(toy, abs=0.10), \
            f"轨迹原点的水平位置应当 ≈ 起飞原点，实际 ({tx}, {ty}) vs ({tox}, {toy})"

        # ★★ 工单的 ★ 核心设计：之后一切目标都以**轨迹原点**为 0 ——
        #   所以 WAIT_START 的 setpoint 目标必须**逐字等于**它（不是悬停点、不是当前位置）。
        target = sc.setpoints()[-1].msg.position
        assert (target.x, target.y, target.z) == pytest.approx((tx, ty, tz), abs=1e-9), (
            "WAIT_START 的目标必须**就是**轨迹原点（停在原地）。"
            "若沿用悬停点，飞机就会慢慢挪回去，而那个「轨迹原点」成了它没去过的点")

        # —— ⑤ 保持满 hover_hold_s ⟹ 自动 AUTO.LAND ——
        _fly_until(sc, plant, lambda: sc.control_state() == "MANUAL/LAND", timeout_s=10.0,
                   message="保持满 hover_hold_s 之后没有自动请求降落")
        sc.wait_for(lambda: any(r.custom_mode == "AUTO.LAND" for r in sc.fake.mode_requests),
                    timeout_s=6.0,
                    message="任务走完却没有请求 AUTO.LAND（上锁交给 PX4，但降落必须是我们请求的）")
        frozen = len(sc.setpoints())
        sc.hold(1.0)
        assert len(sc.setpoints()) == frozen, (
            f"请求降落之后必须**停止**发 setpoint（冻结后又来了 {len(sc.setpoints()) - frozen} 帧）")

        # —— ⑥ 上锁**不是我们发的**：PX4 的 COM_DISARM_LAND 会做，我们只等 armed 降沿 ——
        assert sc.fake.armed, "此刻还没到上锁的时候（PX4 会在它自己的着陆检测之后才上锁）"
        sc.fake.set_state(armed=False)
        sc.wait_for(lambda: sc.control_state() == "MANUAL/STANDBY", timeout_s=8.0,
                    message="落地上锁之后应当自动复位回 STANDBY（不需要重启节点）")

        # ★ 解锁只发一次（工单：不重试）。
        assert len(sc.fake.arming_requests) == 1, (
            f"解锁请求应当只发**一次**，实际 {len(sc.fake.arming_requests)} 次")

        # 正控：复位之后**再按一次**必须能起飞（否则上面那些"复位了"都是空话）。
        assert _takeoff(sc).success, "落地上锁之后应当可以再起飞一次"


# ───────────────────────────── 受理 ≠ 起飞 ─────────────────────────────

def test_遥控不在线时_受理了但永不解锁_命令超时作废():
    """解锁的前置之一是**遥控在线**（§H.7.1「起飞前遥控必须开机且链路已建立」）。

    ★ 这条前置被刻意做成了**机器判据**：遥控不在场时解锁，紧接的 `rc_timeout` 降级
      （只在已解锁时算失效）会判定失效并请求降落 ⟹ 飞机**起飞后立刻降落**，
      比一开始就不解锁难查得多。
    ⚠️ 代价（刻意接受）：台上没有遥控接收机时**永远解不了锁**。见 fsm.hpp ⑦ 的说明。

    ★ 一个**受理 ≠ 起飞**的可观测证据：命令作废之前第二次调用会回 `already_flying`，
      作废之后又能受理 —— 这两次应答把"命令在飞/已作废"这件事变成了接缝 1 能看见的东西。
    """
    params = dict(BASE_PARAMS, takeoff_timeout_s=3.0)
    with Scenario(dut=DUT, params=params, start_dut=False) as sc:
        sc.fake.mute("rc")     # ★ 遥控静默。注意它**不是** STANDBY 的守卫（未解锁时链路断着
                               #   不影响任何事），而是**解锁的前置** —— 这条区别正是本用例的题眼。
        sc.start_dut()
        _reach_offboard(sc)

        resp = _takeoff(sc)
        assert resp.success, f"前置不齐时也应当**受理**命令（受理 ≠ 起飞）：{resp.message}"
        assert resp.message.startswith("accepted"), (
            "★ 应答的词表由 `to_string(TakeoffReply)` 定，且必须在**开头**（现场脚本要能 grep）—— "
            f"`CONTEXT.md` 与 `README.md` 按它写词表，这里就是那个契约的可执行出处。实际：{resp.message}")
        assert "不在线" in resp.message, (
            f"应答里必须报出「遥控不在线」这个缺项，实际：{resp.message}")

        sc.hold(1.5)   # 远短于 takeoff_timeout_s，命令应当仍在飞
        assert not sc.fake.arming_requests, (
            "遥控不在线时**一帧解锁请求都不该发**（否则飞控拒绝、而我们还以为在等生效）")
        assert sc.control_state() == "MANUAL/STANDBY"
        resp2 = _takeoff(sc)
        assert not resp2.success and resp2.message.startswith("already_flying"), (
            f"第一条命令还在飞的时候第二次调用应当被拒，实际：{resp2}")

        # 等过 takeoff_timeout_s ⟹ 命令作废、退回 STANDBY（**不自动重试**）。
        sc.hold(3.0)
        resp3 = _takeoff(sc)
        assert resp3.success, (
            "命令超时作废之后应当可以重新受理（复位不需要重启节点 / 不需要清故障命令）。"
            f"实际：{resp3.message}")
        assert not sc.fake.arming_requests, "遥控仍然不在线 ⟹ 仍然不该有解锁请求"


def test_解锁被拒_报原因且不自动重试():
    """被拒这条路径的完整要求（用户 2026-09-22 裁定）：**报原因 + 退回 STANDBY，不自动重试**。

    ★ `'fail'` 造的是飞控**明确拒绝**（`CommandBool.result = 1` = TEMPORARILY_REJECTED），
      与"没应答"（服务不存在）是两条不同的路。
    ★ 重试的**反面**必须被验，因为静默重试的后果很具体：它会把"解锁参数没配对"这种
      确定性拒绝变成一串没人看的日志，而操作员看到的现象只是"按了键，飞机不动"。
    """
    params = dict(BASE_PARAMS, takeoff_timeout_s=3.0)
    with Scenario(dut=DUT, params=params) as sc:
        _reach_offboard(sc)
        plant = _Plant(sc)
        sc.fake.set_arming_policy("fail")

        assert _takeoff(sc).success
        sc.wait_for(lambda: bool(sc.fake.arming_requests), timeout_s=6.0,
                    message="解锁请求始终没有发出去（前置判据把人挡住了？）")
        sc.hold(2.0)   # 覆盖 offboard_retry_s(1.0) ⟹ 若在重试，这里会多出几次
        assert len(sc.fake.arming_requests) == 1, (
            f"解锁被拒之后**不许重试**，实际发了 {len(sc.fake.arming_requests)} 次")
        assert not sc.fake.armed, "被拒绝的请求不该让飞机解锁"
        assert sc.control_state() == "MANUAL/STANDBY", (
            f"被拒之后应当退回 STANDBY，实际 {sc.control_state()!r}")
        # 原因要进日志：`MAV_RESULT` 码是唯一能区分"被拒"与"没反应"的东西
        # （固定形状的令牌，与 `degrade=` / `traj_origin=` 同一惯例）。
        assert "MAV_RESULT" in sc.dut_log(), (
            "被拒的原因没有进日志 —— 操作员只会看到「按了键飞机不动」")

        # —— 正控：修好之后**再按一次**必须能起飞（失败没有把状态机卡死）——
        # ★ 被拒**不必**等 takeoff_timeout_s：`note_arming_failed()` 当场就把任务复位了，
        #   所以命令立刻可以重来（超时那条兜底管的是"没应答"，不是"被拒"）。
        sc.fake.set_arming_policy("ok")
        assert _takeoff(sc).success, (
            "解锁被拒之后命令应当**当场**可以重来（退回 STANDBY 就是复位，不需要等超时）")
        _fly_until(sc, plant, lambda: sc.control_state() == "AUTO/TAKEOFF", timeout_s=10.0,
                   message="第二次起飞没有进 TAKEOFF")
        assert sc.fake.armed
        assert len(sc.fake.arming_requests) == 2, (
            "第二次起飞应当**新发一次**解锁请求（是新一轮任务，不是重试）")


def test_任务在跑时第二次起飞被拒():
    """`kAlreadyFlying` —— 命令的生命周期绑在任务态上（见 `Fsm::Task` 的说明）。"""
    with Scenario(dut=DUT, params=BASE_PARAMS) as sc:
        _reach_offboard(sc)
        plant = _Plant(sc)
        assert _takeoff(sc).success
        _fly_until(sc, plant, lambda: sc.control_state() == "AUTO/TAKEOFF", timeout_s=10.0,
                   message="第一次起飞没有进 TAKEOFF")

        resp = _takeoff(sc)
        assert not resp.success, "任务在跑的时候第二次起飞必须被拒"
        assert resp.message.startswith("already_flying") and "任务态" in resp.message, (
            f"拒绝的理由要报出来（含当前任务态，方便现场判断在等什么），实际：{resp.message}")
        sc.hold(1.0)
        assert len(sc.fake.arming_requests) == 1, "被拒的命令不许带来第二次解锁请求"


# ───────────────────────────── 爬升速率 ─────────────────────────────

def test_爬升速率不超过_climb_speed_mps():
    """工单："以 ≤0.5 m/s 线性爬升到 1 m"。★ 单独立一条，因为它是**独立的验收项**。

    ★ 用**窗口**上的平均速率（见 `_climb_rate_mps`），不看相邻两帧：rcl 的定时器会补跑，
      50 Hz 的流里本来就有 0 ms 间隔与 40 ms 空洞，逐帧算速率会得到 5 m/s 这种假故障。
    """
    with Scenario(dut=DUT, params=BASE_PARAMS) as sc:
        _reach_offboard(sc)
        plant = _Plant(sc)
        assert _takeoff(sc).success
        _fly_until(sc, plant, lambda: sc.fake.position[2] > 0.95, timeout_s=10.0,
                   message="假飞机没有爬到 0.95 m")

        first, zmax = _climb_span(sc)
        climbed = zmax - first.msg.position.z
        assert climbed >= 0.95, (
            f"假飞机爬到了，但 setpoint 的目标只走了 {climbed:.2f} m ⟹ 目标与飞机不是一回事")
        rate = _climb_rate_mps(sc)
        assert rate <= 0.55, (
            f"爬升速率 {rate:.3f} m/s 超过 climb_speed_mps = 0.5（容差 10% 是给时间戳抖动的，"
            "不是给斜坡本身留的）")
        # 也不算太慢：斜坡真的在按 climb_speed_mps 走（否则"没超标"是"几乎没动"的另一种说法）。
        assert rate >= 0.40, (
            f"爬升速率只有 {rate:.3f} m/s，远低于 climb_speed_mps = 0.5 —— "
            "「没超过上限」不能靠「几乎不爬」来满足")


# ─────────────────── 稳定判据的两半，各造一个反例 ───────────────────

def test_稳定判据的速率那一半_不成立就不记轨迹原点():
    """★★ 本张最容易漏的一条：`位置误差 < 10 cm` **∧** `|v| < 10 m/s…` 是**两条都要**。

    ★ 反例的造法（这一步是关键）：假飞机**已经停在悬停点上**（位置那半妥妥成立），
      只把它**报出来的速度**做坏。夹具的位置与速度是两条独立话题 ⟹
      "位置不动、速度很大"这种内部不自洽的场面在台架上是造得出来的
      （真 mavros 里两者同一个回调发出，物理上自洽；DUT 订的是两条话题）。
      没有这条反例，判据可以只写位置那一半而测试全绿。
    ★ 带正控：把速度改回静止 ⟹ **必须**判稳成立。
    """
    params = dict(BASE_PARAMS, hover_settle_s=1.0)
    with Scenario(dut=DUT, params=params) as sc:
        _reach_offboard(sc)
        plant = _Plant(sc)
        assert _takeoff(sc).success
        _fly_until(sc, plant, lambda: sc.control_state() == "AUTO/HOVER", timeout_s=12.0,
                   message="没有进 HOVER（爬不上去？）")

        # ★ 先证明"位置那一半此刻是妥妥成立的"，否则下面测的就不是速率那一半：
        #   假飞机稳态跟踪（plant 的限速 1.5 > 爬升 0.5）⟹ 它就在悬停点上，
        #   而悬停点 = 起飞原点正上方 1 m（起飞原点已在②里确认过 ≈ 0）。
        assert sc.fake.position[2] == pytest.approx(1.0, abs=0.02), (
            f"假飞机没停在悬停点高度上（{sc.fake.position[2]:.3f} m）⟹ 这条用例测不出"
            "「速率那一半」是不是真的在起作用")
        assert sc.setpoints()[-1].msg.position.z == pytest.approx(
            sc.fake.position[2], abs=0.05), "setpoint 的目标不在飞机附近 ⟹ 位置误差那一半也不成立"

        plant.freeze()                              # 别让 plant 每帧把速度写回 0
        sc.fake.set_velocity(0.4, 0.0, 0.4)         # |v| = 0.566 m/s > 0.10
        sc.hold(3.0)                                # 远超 hover_settle_s(1.0)

        assert sc.control_state() == "AUTO/HOVER", (
            "位置误差 < 10 cm 但三维速率 0.566 m/s ⟹ 判稳**不该**成立。"
            "实际进了 WAIT_START ⟹ 判据只看了位置那一半")
        assert "traj_origin" not in _origins_in_log(sc), (
            "判稳没成立却已经记录了轨迹原点 —— 那后面一切目标都会锚在一个没稳住的点上")

        # —— 正控 ——
        sc.fake.set_velocity(0.0, 0.0, 0.0)
        sc.hold(3.0)
        assert sc.control_state() == "AUTO/WAIT_START", (
            "速度恢复静止之后判稳**必须**成立 —— 否则上面那条「没进」是别的原因造成的")
        assert "traj_origin" in _origins_in_log(sc)


def test_稳定判据的位置那一半_不成立就不记轨迹原点():
    """位置反例：速率稳稳为 0，只把假 EKF 的位置**搬偏** 30 cm（> `hover_pos_tol_m`）。

    ★ 与上一条合起来才说明两条判据**各自**都是必需的（少任何一条反例，
      实现里都可以把对应那一半删掉而测试全绿）。
    """
    params = dict(BASE_PARAMS, hover_settle_s=1.0)
    with Scenario(dut=DUT, params=params) as sc:
        _reach_offboard(sc)
        plant = _Plant(sc)
        assert _takeoff(sc).success
        _fly_until(sc, plant, lambda: sc.control_state() == "AUTO/HOVER", timeout_s=12.0,
                   message="没有进 HOVER（爬不上去？）")

        plant.freeze()
        hover = sc.fake.position
        sc.fake.set_position(hover[0] + 0.30, hover[1], hover[2])   # 速度仍是 0
        sc.hold(3.0)
        assert sc.control_state() == "AUTO/HOVER", (
            "速率 < 0.10 m/s 但离悬停点 0.30 m ⟹ 判稳**不该**成立"
            f"（hover_pos_tol_m = {params['hover_pos_tol_m']}）")
        assert "traj_origin" not in _origins_in_log(sc), "判稳没成立却记录了轨迹原点"

        # —— 正控：搬回悬停点上 ⟹ 必须判稳成立 ——
        sc.fake.set_position(*hover)
        sc.hold(3.0)
        assert sc.control_state() == "AUTO/WAIT_START", \
            "位置恢复之后判稳**必须**成立，否则上面的「没进」是别的原因"


# ───────────────────────────── 机头 ─────────────────────────────

def test_进入_TAKEOFF_后机头锁定_不跟随():
    """工单 §3.4：起飞**不该顺带转机头** ⟹ setpoint 的 yaw = **起飞机头**（锁一次）。

    ★ 这条推翻了 ticket 05 的 `yaw` 参数（已删）：那个参数默认 0，而 0 在 ROS/ENU 口径下
      经 mavros 换算后是 PX4 的 +90°（机头朝东）⟹ 一旦在 OFFBOARD 里解锁，PX4 会**真的转机头**。
    ★ 反例的造法：把假 EKF 的机头**转到别处**，setpoint 的 yaw 不许动。
      不这么做的话，"锁定"与"跟随"在台架上无法区分（都恒等于 0）。
    """
    with Scenario(dut=DUT, params=BASE_PARAMS) as sc:
        _reach_offboard(sc)
        plant = _Plant(sc)

        # STANDBY：yaw = **当前机头**（这样飞机原地不动、不转向）。
        sc.fake.set_yaw(0.6)
        sc.hold(0.5)
        assert sc.setpoints()[-1].msg.yaw == pytest.approx(0.6, abs=1e-6), (
            "STANDBY 段 setpoint 的 yaw 必须 = 当前机头")

        assert _takeoff(sc).success
        _fly_until(sc, plant, lambda: sc.control_state() == "AUTO/TAKEOFF", timeout_s=10.0,
                   message="没有进 TAKEOFF")
        locked = sc.setpoints()[-1].msg.yaw
        assert locked == pytest.approx(0.6, abs=1e-6), (
            f"起飞机头应当=解锁那一刻的机头（0.6 rad），实际 {locked}")

        # ★ 反例：把机头转到别处 —— setpoint 的 yaw 不许跟着动。
        sc.fake.set_yaw(-1.1)
        _fly(sc, plant, 1.5)
        assert sc.setpoints()[-1].msg.yaw == pytest.approx(0.6, abs=1e-6), (
            "进入 TAKEOFF 之后机头被锁死，不该跟随假 EKF 报的机头（起飞不转向）"
            "—— 实际跟着动了 ⟹ 飞机在起飞过程中会自己扭一圈")


# ───────────────────────────── 假飞机（场景侧） ─────────────────────────────

class _Plant:
    """台架上的"假飞机"：把 DUT 发的 setpoint 变成**假 EKF 报出来的位置与速度**。

    ★★ 为什么必须有这一层：`FakeMavros` 刻意**不含任何动力学**（见 `src/bench/README.md`
      §「它假的是什么、不假什么」）—— "你发你的 setpoint，它报它的位置"是它的全部行为。
      那个选择在 ticket 05 是对的（当时验的是"帧对不对"），但 ticket 08 的判据全是关于
      飞机状态的：没有这一层，位置恒为 0 ⟹ "爬到 1 m"永远不会发生；速度恒为 0
      ⟹ `|v| < 0.10` **恒真**（判据的一半没有反例，测试会在此前提下变绿）。

    ★ 模型 = **限速逼近**（`rate_mps` > 爬升的 0.5 ⟹ 稳态跟踪误差为 0，
      不会用"假的跟踪偏差"污染"位置误差 < 10 cm"那一条）+ **速度 = 实际位移 / dt**。
      速度是**真的**：爬升段 0.5 m/s、到位后 0，所以判据的速率那一半有反例可造。
    ★ 只在**有新 setpoint 到达**时才动一格：否则夹具的轮询比 DUT 的 50 Hz 快时，
      两格之间没有新指令 ⟹ 位移为 0 ⟹ 报出来的速度会在 0 与 2× 之间跳
      （真 mavros 不会这样，那是夹具自己造出来的假故障）。
    ⚠️ 它**不碰机头**：yaw 一律由用例自己 `set_yaw`。"锁定不跟随"那条判据恰恰要求
      机头能被人为改掉而 setpoint 的 yaw 不动 —— 让 plant 跟着 setpoint 转 yaw 会把那条判据测没。
    """

    def __init__(self, sc, rate_mps: float = 1.5):
        self.sc = sc
        self.rate_mps = rate_mps
        self.x, self.y, self.z = (float(v) for v in sc.fake.position)
        self._last_t = sc.t()
        self._last_n = len(sc.fake.setpoints)
        self.frozen = False

    def freeze(self) -> None:
        """停手：之后 `step()` 什么都不做（用来造"飞机没跟着目标走"的场面）。"""
        self.frozen = True

    def step(self) -> None:
        now = self.sc.t()
        sps = self.sc.fake.setpoints
        if self.frozen or not sps or len(sps) == self._last_n:
            return
        dt = now - self._last_t
        if dt <= 0.0:
            return
        self._last_n = len(sps)
        self._last_t = now

        sp = sps[-1].msg
        limit = self.rate_mps * dt
        dx = _clamp(sp.position.x - self.x, limit)
        dy = _clamp(sp.position.y - self.y, limit)
        dz = _clamp(sp.position.z - self.z, limit)
        self.x += dx
        self.y += dy
        self.z += dz
        # 位置与速度是两条**独立**话题 ⟹ 写回顺序无关（见 `FakeMavros.set_velocity` 的 ⚠️）。
        self.sc.fake.set_velocity(dx / dt, dy / dt, dz / dt)
        self.sc.fake.set_position(self.x, self.y, self.z)


def _clamp(delta: float, limit: float) -> float:
    return max(-limit, min(limit, delta))
