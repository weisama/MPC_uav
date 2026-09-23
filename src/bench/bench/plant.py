"""台架上的"假飞机" —— 把 DUT 发的 setpoint 变成假 EKF 报出来的位置与速度。

★★ 为什么需要这一层：`FakeMavros` 刻意**不含任何动力学**（见 `src/bench/README.md`
  §「它假的是什么、不假什么」）—— "你发你的 setpoint，它报它的位置"是它的全部行为。
  那个选择在 ticket 05 是对的（当时验的是"帧对不对"），但 ticket 08 起 DUT 的判据全是关于
  **飞机状态**的：没有这一层，位置恒为 0 ⟹ "爬到 1 m"永远不会发生；速度恒为 0
  ⟹ `|v| < 0.10` **恒真**（判据有一半没有反例，测试会在那个前提下变绿）。

★ 它原来住在 `src/control/test/test_control_takeoff.py` 里，注释写着"ticket 09/10 真复用时
  再提升进 `bench`"。**ticket 09 就是那次复用**（轨迹跟踪同样要"飞机真的在动"），
  于是搬到这里 —— 两个测试文件各留一份"假飞机"必然走散，而两者断言的物理是同一套。

★ 模型 = **限速逼近**（`rate_mps` 大于任何一条指令速率 ⟹ 稳态跟踪误差为 0，
  不会用"夹具自己造出来的跟踪偏差"污染"位置误差 < 10 cm"那类判据）
  + **速度 = 实际位移 / dt**（速度是**真的**：爬升段 0.5 m/s、到位后 0）。
★ 只在**有新 setpoint 到达**时才动一格：否则夹具的轮询比 DUT 的 50 Hz 快时，
  两格之间没有新指令 ⟹ 位移为 0 ⟹ 报出来的速度会在 0 与 2× 之间跳
  （真 mavros 不会这样，那是夹具自己造出来的假故障）。
⚠️ 它**不碰机头**：yaw 一律由用例自己 `set_yaw`。"锁定不跟随"那条判据恰恰要求
  机头能被人为改掉而 setpoint 的 yaw 不动 —— 让 plant 跟着 setpoint 转 yaw 会把那条判据测没。
"""

from typing import Callable

__all__ = ["Plant", "fly", "fly_until"]


def _clamp(delta: float, limit: float) -> float:
    return max(-limit, min(limit, delta))


class Plant:
    """限速逼近的假飞机。用法：每个推进步调一次 `step()`（或直接用 `fly()` / `fly_until()`）。

    ⚠️ `Scenario.hold()` **不会**推进它（`hold` 只走场景时钟）—— 用 `fly` 系列。
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


def fly(sc, plant: Plant, seconds: float, dt: float = 0.02) -> None:
    """推进 `seconds` 秒，途中让假飞机跟着 setpoint 动。★ 别用 `sc.hold` —— 那个不动飞机。"""
    end = sc.t() + seconds
    while sc.t() < end:
        plant.step()
        sc.advance_to(min(sc.t() + dt, end))


def fly_until(
    sc,
    plant: Plant,
    predicate: Callable[[], bool],
    timeout_s: float = 15.0,
    message: str = "",
    dt: float = 0.02,
) -> None:
    """一边让假飞机飞、一边等判据成立（等价于 `Scenario.wait_for`，但飞机在动）。

    超时的报错与 `BenchTimeout` 一样带上 `evidence()`：台架上"为什么没到"的答案几乎总在记录里。
    """
    deadline = sc.t() + timeout_s
    while True:
        if predicate():
            return
        if sc.t() >= deadline:
            raise AssertionError(
                (message or "等待判据超时") + f"（等了 {timeout_s:g} s）\n\n" + sc.evidence())
        plant.step()
        sc.advance_to(min(sc.t() + dt, deadline))
