"""故障注入时序 —— **纯函数**，零 ROS 依赖（所以它能被最便宜的测试盯住）。

★ 为什么单独成模块并且不 import rclpy：这个文件回答的是"第几秒该干什么"，
  这类逻辑的错误（早触发、漏触发、重复触发）在集成测试里表现为"偶尔抽风"，
  而它本身可以在一毫秒内被单测钉死。见 `test/test_timeline.py`（纯 pytest，无 ROS）。

★ 时刻的尺子：`at_s` 一律是 **`Scenario.t()` 那把秒表**（相对「夹具就位」的秒数）。
  DUT 在夹具就位后立刻启动（`dut_started_at_s` 一般 < 0.1 s），所以写时刻时可以近似当成
  "DUT 起来 3 秒后把 state 掐掉"；但**别在绝对时刻上卡边界**，涉及 DUT 的断言请用相对量
  （见 `Scenario.t()` 的说明）。
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Callable, Iterable, List


@dataclass
class Event:
    """某个时刻要执行的一次动作。`label` 只用于日志/证据（出事时能看出"哪一条没按预期发生"）。"""
    at_s: float
    action: Callable[[], None]
    label: str = ""

    def __post_init__(self) -> None:
        if self.at_s < 0:
            raise ValueError(f"事件时刻不能为负：{self.at_s}（{self.label}）")


@dataclass
class Fired:
    """已触发的事件 —— 留给证据输出。"""
    at_s: float          # 计划时刻
    fired_at_s: float    # 实际时刻（抖动 = 二者的差）
    label: str


class Timeline:
    """一串按时刻排队的动作。`due(t)` 取出并移除所有 `at_s <= t` 的事件（幂等、不重复触发）。"""

    def __init__(self, events: Iterable[Event] = ()) -> None:
        self._events: List[Event] = sorted(events, key=lambda e: e.at_s)
        self.fired: List[Fired] = []

    def add(self, at_s: float, action: Callable[[], None], label: str = "") -> Event:
        return self.extend([Event(at_s=at_s, action=action, label=label)])[0]

    def extend(self, events: Iterable[Event]) -> List[Event]:
        added = list(events)
        self._events.extend(added)
        self._events.sort(key=lambda e: e.at_s)
        return added

    def due(self, t_s: float) -> List[Event]:
        """取出所有到点的事件（并记账）。同一事件只会被取出一次。"""
        ready: List[Event] = []
        while self._events and self._events[0].at_s <= t_s:
            ready.append(self._events.pop(0))
        for ev in ready:
            self.fired.append(Fired(at_s=ev.at_s, fired_at_s=t_s, label=ev.label))
        return ready

    def pending(self) -> List[Event]:
        return list(self._events)

    def describe(self) -> str:
        """给证据输出用：计划了什么、实际什么时候发生。"""
        lines = ["### 故障注入时序", "", "| 计划 (s) | 实际 (s) | 动作 |", "|---|---|---|"]
        if not self.fired:
            lines.append("| — | — | （没有任何事件触发） |")
        for f in self.fired:
            lines.append(f"| {f.at_s:.2f} | {f.fired_at_s:.2f} | {f.label or '（未命名）'} |")
        for ev in self._events:
            lines.append(f"| {ev.at_s:.2f} | **未触发** | {ev.label or '（未命名）'} |")
        return "\n".join(lines)


# ───────────────────────── 常用的故障配方 ─────────────────────────
#
# 全是"返回事件列表"的纯函数：不碰 ROS、不碰时间，只描述"什么时候对谁做什么"。
# 用 `scenario.timeline.extend(silence_for(fake, 'state', at_s=3.0, duration_s=3.0))` 挂上去。

def silence_for(
    fake, key: str, at_s: float, duration_s: float, label: str = "",
) -> List[Event]:
    """让 `fake` 的某条话题静默 `duration_s` 秒，然后恢复。

    ★ 时长要按被测判据选，不是随便填：
      · `/mavros/state` 实测 1.000 Hz ⟹ 想触发"飞控失联"（默认判据 2.5 s），
        `duration_s` 必须 **≥ 3 s**；掐 0.5 s 落在正常间隔内，什么也测不出来。
      见 docs/control/phase1-implementation.md §4.3-3 与 ticket 05 的 Comments。
    """
    tag = label or f"{key} 静默 {duration_s:g} s"
    return [
        Event(at_s=at_s, action=lambda: fake.mute(key), label=f"开始：{tag}"),
        Event(at_s=at_s + duration_s, action=lambda: fake.unmute(key), label=f"恢复：{key}"),
    ]


def state_change(fake, at_s: float, label: str = "", **fields) -> List[Event]:
    """改 `/mavros/state` 的字段，例如 `state_change(fake, 3.0, connected=False)`。"""
    tag = label or "state 改为 " + ", ".join(f"{k}={v}" for k, v in fields.items())
    return [Event(at_s=at_s, action=lambda: fake.set_state(**fields), label=tag)]


def voltage_drop(fake, at_s: float, voltage: float, label: str = "") -> List[Event]:
    """电压跌落（ticket 06 的失效路径要能表达）。"""
    tag = label or f"电压 → {voltage} V"
    return [Event(at_s=at_s, action=lambda: fake.set_voltage(voltage), label=tag)]


def rc_change(fake, at_s: float, channel: int, value: int, label: str = "") -> List[Event]:
    """改一路遥控通道（通道号从 1 起算）。例如 `rc_change(fake, 4.0, 9, 2000)` = 拨动 ch9。"""
    tag = label or f"ch{channel} → {value}"
    return [Event(at_s=at_s, action=lambda: fake.set_rc_channel(channel, value), label=tag)]


def set_mode_policy(fake, at_s: float, policy: str, label: str = "") -> List[Event]:
    """改 `/mavros/set_mode` 的应答策略：`'ok'` / `'fail'` / `'absent'`。"""
    tag = label or f"set_mode 应答 → {policy}"
    return [Event(at_s=at_s, action=lambda: fake.set_mode_policy(policy), label=tag)]


def px4_status_silence(px4, at_s: float, duration_s: float, label: str = "") -> List[Event]:
    """假 PX4 的 `/fmu/out/vehicle_status` **停止发布** `duration_s` 秒。

    ★ 时长要按被测判据选（与 `silence_for` 同一条纪律）：真 PX4 的这条话题是
      **2 Hz 定频 + 变化即刻发**（`Commander.cpp:1790`）⟹ 只掐 0.5 s 落在正常间隔内，
      什么也测不出来；要触发"状态多久没到"的判据，`duration_s` 必须**明显大于 2.5 个周期**。
    """
    tag = label or f"假 PX4 状态静默 {duration_s:g} s"
    return [
        Event(at_s=at_s, action=lambda: px4.mute("status"), label=f"开始：{tag}"),
        Event(at_s=at_s + duration_s, action=lambda: px4.unmute("status"), label="恢复：假 PX4 状态"),
    ]


def px4_arm(px4, at_s: float, armed: bool = True, label: str = "") -> List[Event]:
    """改假 PX4 的解锁状态。★ 变化**不超过一拍（10 ms）就发出去**（见 `fake_px4.STATUS_PERIOD_S`），
    所以"注入 armed 之后等它反映到话题上"是毫秒级的事，不需要留 0.5 s 的余量。"""
    tag = label or f"假 PX4 armed → {armed}"
    return [Event(at_s=at_s, action=lambda: px4.set_armed(armed), label=tag)]


def px4_nav_state(px4, at_s: float, nav_state: int, label: str = "") -> List[Event]:
    """改假 PX4 的当前模式。

    ⚠️ `nav_state` 传**整数**（`NAV_STATE_OFFBOARD = 14` 等）。常量只在
    `bench/fake_px4.py` 里有一份 —— 本模块**零 ROS 依赖**（见文件头），所以不 import 它，
    由调用方传值（测试里 `from bench.fake_px4 import NAV_STATE_OFFBOARD` 即可）。
    """
    tag = label or f"假 PX4 nav_state → {nav_state}"
    return [Event(at_s=at_s, action=lambda: px4.set_nav_state(nav_state), label=tag)]


def px4_preflight_checks(px4, at_s: float, ok: bool, label: str = "") -> List[Event]:
    """改假 PX4 的 `pre_flight_checks_pass`（解锁前的门槛之一）。"""
    tag = label or f"假 PX4 pre_flight_checks_pass → {ok}"
    return [Event(at_s=at_s, action=lambda: px4.set_pre_flight_checks_pass(ok), label=tag)]


def px4_state_silence(px4, at_s: float, duration_s: float,
                      key: str = "local_position", label: str = "") -> List[Event]:
    """假 PX4 的**一条状态话题**停止发布 `duration_s` 秒（`key`：`local_position` / `attitude`）。

    ★ `key` 必须能单独选：位置与姿态是**两条消息**（实测 32.5 / 31.6 Hz，各判各的龄）——
      "只掐姿态、位置照发"考的是"节点有没有把两条都当必需输入"，
      而"一起掐"考的是 `px4_stale`。两条判据不同，配方也就不该合成一个。

    ★ 时长按 `px4_timeout_s`（默认 0.5 s）选：这条流的周期只有 ~31 ms，
      所以要**明显大于** 0.5 s 才跨得过闸门（与 `px4_status_silence` 同一条纪律）。
    """
    tag = label or f"假 PX4 状态 {key} 静默 {duration_s:g} s"
    return [
        Event(at_s=at_s, action=lambda: px4.mute(key), label=f"开始：{tag}"),
        Event(at_s=at_s + duration_s, action=lambda: px4.unmute(key), label=f"恢复：{key}"),
    ]


def px4_state(px4, at_s: float, position_ned=(), velocity_ned=(), yaw_deg=None,
              label: str = "") -> List[Event]:
    """把假 PX4 注入的状态搬到某个**NED**位姿上（只给的那几样会变，其余保持）。

    ★ 参数刻意用 NED 的**原始名字**（`position_ned=(北, 东, 下)`）：这里是"被测系统的输入"，
      用 ENU 写会让人以为夹具替 DUT 做过转换（见 `fake_px4` 里那段 ★）。
    ⚠️ `yaw_deg` 是**绕 NED 的 z 轴**（正 = 机头从北转向东），不是 ENU 的偏航角。
    """
    def action() -> None:
        if position_ned:
            px4.set_position_ned(*position_ned)
        if velocity_ned:
            px4.set_velocity_ned(*velocity_ned)
        if yaw_deg is not None:
            px4.set_yaw_ned(math.radians(float(yaw_deg)))

    tag = label or (f"假 PX4 状态 → NED p={tuple(position_ned) or '不变'} "
                    f"v={tuple(velocity_ned) or '不变'} yaw={yaw_deg if yaw_deg is not None else '不变'}")
    return [Event(at_s=at_s, action=action, label=tag)]


def hold_mocap_stamp(source, at_s: float, held: bool = True, label: str = "") -> List[Event]:
    """动捕的**时间戳冻住**（照发，但 stamp 不前进）—— mavros 会丢弃这类消息。"""
    tag = label or ("动捕时间戳冻住" if held else "动捕时间戳恢复")
    return [Event(at_s=at_s, action=lambda: source.hold_stamp(held), label=tag)]


def pause_mocap(source, at_s: float, duration_s: float, label: str = "") -> List[Event]:
    """动捕**停止发布** `duration_s` 秒（与"冻住"是两种不同的失效，见 `VrpnSource.pause`）。"""
    tag = label or f"动捕停止发布 {duration_s:g} s"
    return [
        Event(at_s=at_s, action=lambda: source.pause(), label=f"开始：{tag}"),
        Event(at_s=at_s + duration_s, action=lambda: source.resume(), label="恢复：动捕发布"),
    ]


def mocap_pose_jump(source, at_s: float, position=(), orientation=None,
                    hold_s: float = 0.5, label: str = "") -> List[Event]:
    """动捕**瞬移** `hold_s` 秒再回到原位 —— 造"刚体认错 / 遮挡恢复"那一类坏帧。

    ★ 与 `rewind` 是两种不同的失效，`NodeCore` 也是两条判据：这里位置/姿态**跳**了
      （`pose_jump` / `attitude_jump`），stamp 是**正常前进**的；rewind 反过来。
      两条都发的话，"判据是哪一条在起作用"就分不出来了 —— 所以本配方**只动位姿**。
    ★ `hold_s` 默认 0.5 s：动捕 200 Hz ⟹ 100 帧都停在被拒的那一档，够节点跑完
      `warmup_cycles` 之外的判据；太短（比如 5 ms）会让"恢复"与"跳变"混在同一拍里。
    ★ 回归原位用的是**当初记下的**位姿（不是硬编码原点）：夹具的初始位姿可以被场景改过。
    """
    pos0, quat0 = source.position, source.orientation
    tag = label or f"动捕位姿跳变 {tuple(position) if position else ''}，持续 {hold_s:g} s"
    return [
        Event(at_s=at_s,
              action=lambda: source.set_pose(position=position or None,
                                             orientation=orientation),
              label=f"开始：{tag}"),
        Event(at_s=at_s + hold_s,
              action=lambda: source.set_pose(position=pos0, orientation=quat0),
              label="恢复：动捕位姿回到跳变前"),
    ]


def mocap_stamp_rewind(source, at_s: float, seconds: float, hold_s: float = 0.5,
                       label: str = "") -> List[Event]:
    """动捕的 **stamp 往回走** `seconds` 秒，`hold_s` 之后一次性跳回来。

    ★ 这是 `VrpnSource.rewind` 的时序化：上游重放 / VRPN 时钟回拨在话题上就是这个形状
      （向后的跳变 + 一段时间后向前的跳变），而 `NodeCore` 的 `stamp_regress` 判据
      拦的正是它。**位置不动** —— 于是"闸门关了"只可能是 stamp 那条判据干的。
    """
    tag = label or f"动捕 stamp 倒退 {seconds:g} s，持续 {hold_s:g} s"
    return [Event(at_s=at_s, action=lambda: source.rewind(seconds, hold_s=hold_s), label=tag)]
