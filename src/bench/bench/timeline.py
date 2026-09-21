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
