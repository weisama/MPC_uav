"""时序的纯逻辑单测 —— **零 ROS**（沿用 mocap 的 seam 2 先例：能纯函数测的就别上集成）。

为什么值得单独测：时序的错误（漏触发、重复触发、早触发）在集成测试里表现为"偶尔抽风"，
而在这里是一毫秒就能钉死的确定性问题。
"""

from bench.timeline import (
    Event,
    Timeline,
    hold_mocap_stamp,
    pause_mocap,
    rc_change,
    silence_for,
    state_change,
    voltage_drop,
)


class _Fake:
    """记录"被谁在什么时候动过"的最小替身 —— 不碰 ROS。"""

    def __init__(self):
        self.calls = []

    def mute(self, key):
        self.calls.append(("mute", key))

    def unmute(self, key):
        self.calls.append(("unmute", key))

    def set_state(self, **kw):
        self.calls.append(("set_state", kw))

    def set_voltage(self, v):
        self.calls.append(("set_voltage", v))

    def set_rc_channel(self, ch, v):
        self.calls.append(("set_rc_channel", ch, v))

    def hold_stamp(self, held):
        self.calls.append(("hold_stamp", held))

    def pause(self):
        self.calls.append(("pause",))

    def resume(self):
        self.calls.append(("resume",))


def test_due_按时刻顺序取出且只取一次():
    log = []

    def fire(events):
        for ev in events:
            ev.action()

    tl = Timeline()
    tl.add(2.0, lambda: log.append("b"), "第二条")
    tl.add(1.0, lambda: log.append("a"), "第一条")

    assert [e.label for e in tl.due(0.5)] == []
    due = tl.due(1.0)                                    # 恰好到点也算
    assert [e.label for e in due] == ["第一条"]
    fire(due)
    assert [e.label for e in tl.due(1.9)] == []
    due = tl.due(9.9)
    assert [e.label for e in due] == ["第二条"]
    fire(due)
    assert tl.due(99.0) == []                            # ★ 取出过就不再触发
    assert log == ["a", "b"]


def test_同刻事件保持加入顺序():
    log = []
    tl = Timeline()
    tl.add(1.0, lambda: log.append(1))
    tl.add(1.0, lambda: log.append(2))
    for ev in tl.due(1.0):
        ev.action()
    assert log == [1, 2]


def test_负时刻被拒绝():
    import pytest

    with pytest.raises(ValueError):
        Event(at_s=-0.1, action=lambda: None)


def test_describe_把没触发的事件标出来():
    tl = Timeline()
    tl.add(1.0, lambda: None, "会触发")
    tl.add(5.0, lambda: None, "不会触发")
    tl.due(2.0)
    text = tl.describe()
    assert "会触发" in text
    assert "未触发" in text


def test_silence_for_掐掉再恢复():
    fake = _Fake()
    events = silence_for(fake, "state", at_s=3.0, duration_s=3.0)
    assert [e.at_s for e in events] == [3.0, 6.0]
    for e in events:
        e.action()
    assert fake.calls == [("mute", "state"), ("unmute", "state")]


def test_配方真的调到替身上():
    """每个配方都触发一遍：**动作真的落到了对象上**，顺序也对。

    这条与 `test_silence_for_掐掉再恢复` 一起，保证"时序"与"故障"之间的接线没接错 ——
    接线错了的表现是"注入了一条看上去毫无效果的故障"，而这在集成测试里极难看出来。
    """
    fake = _Fake()
    batch = []
    for events in (
        state_change(fake, 1.0, connected=False),
        voltage_drop(fake, 2.0, 13.5),
        rc_change(fake, 3.0, channel=9, value=2000),
        hold_mocap_stamp(fake, 4.0),
        pause_mocap(fake, 5.0, duration_s=1.0),
    ):
        assert len(events) in (1, 2)
        batch += events

    tl = Timeline(batch)
    for ev in tl.due(99.0):
        ev.action()

    assert fake.calls == [
        ("set_state", {"connected": False}),
        ("set_voltage", 13.5),
        ("set_rc_channel", 9, 2000),
        ("hold_stamp", True),
        ("pause",),
        ("resume",),
    ]
