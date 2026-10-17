"""接缝 1（话题边界）单测：traj 节点**真的作为一个进程**跑起来，只断言它对外发了什么。

分工（见 .scratch/phase1/spec.md 的「接缝 1 / 接缝 2」）：

  · test/test_trajectories.cpp（seam 2）：几何、尺寸、限速对不对 —— 那里有精确解可对。
  · 本文件（seam 1）：**"选 / 开始 / 清空 / 重选"这四条时序规则**对不对，以及
    参考**真的按 50 Hz 流出来了**。这些在纯函数层不存在（那里没有时间、没有话题）。

★★ 本文件里每条"没有发生"的断言，都必须配一条"确实发生了"的阳性对照。
   理由：`assert rec.refs == []` 在"话题名写错了""节点根本没起来""QoS 不兼容"
   这三种情况下**同样通过** —— 那种断言不是证据，是安慰。
   所以下面每条负向断言旁边都有一条正向断言（收到 marker / DUT 日志里那句话 / 后续帧数恢复）。

★ 为什么用本地 recorder 而不是往 `bench` 里加记录器：bench 是 Phase 1/2/3 的公共底座、
  自己也有自检（`test/test_bench_selftest.py`）；为了三条 traj 话题去动它，风险大于收益。
  本地 recorder 只订阅、不碰 DUT 内部，符合 seam 1 的定义。
"""

from __future__ import annotations

import math

import pytest
from mavros_msgs.msg import PositionTarget
from std_msgs.msg import Empty, Float32, UInt8
from visualization_msgs.msg import Marker

from bench.scenario import Scenario

DUT = ("traj", "traj_node")

REF_TOPIC = "/traj/reference"
PROGRESS_TOPIC = "/traj/progress"
MARKER_TOPIC = "/traj/marker"
SELECT_TOPIC = "/traj/select"
START_TOPIC = "/traj/start"

#: `type_mask`（置 1 = 忽略）：位置 + 速度 + yaw 有效 ⟹ 只忽略加速度(448)与 yaw_rate(2048)。
#: ★ 在这里把数字钉死是刻意的：改掩码会改变参考的语义（例如开启速度前馈，§8.6），
#:   必须是一次**有意的**修改，不能悄悄漂过去。
TYPE_MASK_POS_VEL_YAW = 2496
FRAME_LOCAL_NED = 1

#: §5.2 的设计限速。参考帧之间用有限差分量的速度也不允许超过它。
MAX_SPEED_MS = 0.5
#: 采样与网络抖动带来的余量（50 Hz 采样 + 有限差分 ⟹ 卡在阈值上会假阳性）。
SPEED_TOL = 0.02


# ─────────────────────────── 本地 recorder / commander ───────────────────────────


class TrajRecorder:
    """把 traj 的三条输出录下来。**只订阅**，不 import DUT 的类、不读它的成员。"""

    def __init__(self, node, clock):
        self._t = clock
        self.refs = []       # [(at_s, PositionTarget)]
        self.progress = []   # [(at_s, float)]
        self.markers = []    # [(at_s, Marker)]
        # QoS 用默认（reliable + volatile + depth 10），与 traj 的 rclcpp::QoS(10) 兼容。
        node.create_subscription(PositionTarget, REF_TOPIC, self._on_ref, 10)
        node.create_subscription(Float32, PROGRESS_TOPIC, self._on_progress, 10)
        node.create_subscription(Marker, MARKER_TOPIC, self._on_marker, 10)

    def _on_ref(self, msg):
        self.refs.append((self._t(), msg))

    def _on_progress(self, msg):
        self.progress.append((self._t(), msg.data))

    def _on_marker(self, msg):
        self.markers.append((self._t(), msg))

    # —— 取用（一律带 `since`：同一个 recorder 会跨"重选/清空"，必须按段看）——

    def refs_since(self, n):
        """第 `n` 帧之后的 `(at_s, 消息)` 对。

        ★ 保留时刻而不是只给消息：**位置差分算速度**、**窗口算频率**都要用到它，
          丢掉时刻就没法做那两条最关键的独立验证了。
        """
        return list(self.refs[n:])

    def msgs_since(self, n):
        """只要消息（不关心时刻时用）。"""
        return [m for _, m in self.refs[n:]]

    def progress_since(self, n):
        return [v for _, v in self.progress[n:]]

    def latest_ref(self):
        return self.refs[-1][1] if self.refs else None

    def rate_hz(self, from_s, to_s):
        """窗口内实测的参考频率。

        ★ 只算窗口，**不断言相邻两帧的间隔**：rcl 的定时器是"从回调开始时刻重新计时"
          （`rcl/timer.h`），回调偶尔超时就会立刻补跑 ⟹ 50 Hz 的流里出现 0 ms 间隔与
          40 ms 空洞都是正常的，而窗口平均频率仍然对。
        """
        stamps = [at for at, _ in self.refs if from_s <= at <= to_s]
        span = to_s - from_s
        return len(stamps) / span if span > 0 else 0.0

    def markers_of(self, marker_id):
        return [m for _, m in self.markers if m.id == marker_id]

    def wait_for_marker(self, sc, marker_id, timeout_s=10.0):
        return sc.poll(lambda: len(self.markers_of(marker_id)) > 0, timeout_s=timeout_s)


#: DUT 日志里"start 被接受"与"start 被丢弃"的两句话。**判据是它们，不是时间**：
#: 这两个字符串同时也是 `test_没选轨迹就_start_被忽略且只告警` 的阳性对照。
START_LOG_OK = "★ 开始"
START_LOG_IGNORED = "还没有选轨迹"


class TrajCommander:
    """发 `/traj/select` 与 `/traj/start`。"""

    def __init__(self, node):
        self._select = node.create_publisher(UInt8, SELECT_TOPIC, 10)
        self._start = node.create_publisher(Empty, START_TOPIC, 10)

    def select(self, value: int):
        msg = UInt8()
        msg.data = value
        self._select.publish(msg)

    def start(self, sc=None, timeout_s=6.0):
        """发 `/traj/start`（`Empty`）。

        ★★ 为什么带 `sc` 时要"重发到看见结果"：`/traj/select` 与 `/traj/start` 是**两个 DDS
          writer**，彼此**没有顺序保证** —— 我们先 publish `select` 不等于 DUT 先处理它。
          DUT 对"还没选轨迹就 start"只 WARN 一句然后丢弃（这个行为本身是对的：手打命令时
          确实可能敲反），而 `Empty` 没有序号、**补不了**。所以"选 + 开始"这类流程用例必须按
          重发处理，否则就是一条**随机红**的测试（实测踩到过：同一份代码全量跑时偶发失败）。

          ★ 这不是"盲重发"：只在 DUT **明说了**它丢掉一次（`START_LOG_IGNORED` 计数 +1）时
            才补发一帧，并在看到 `START_LOG_OK` 计数上升时立刻收工 —— 判据全部来自 DUT 的日志，
            重发次数有界（`timeout_s`）。两个计数都取**增量**：同一个 recorder/commander
            在一次用例里会发多次 start（例如"飞行中重选"），历史日志里的旧 `★ 开始`
            绝不能被当成本次已生效。

        不传 `sc` 时退化为**一次性**发布 —— 负向用例 `test_没选轨迹就_start_被忽略`
          用的就是它（那条要的恰恰是"这一次 start 被忽略"，重发会把判据毁掉）。

        超时**不抛异常**：让调用方自己那句带判据的 `wait_for` 去失败，报出来的才是
          "参考没流出来"而不是"补发次数用完了"。
        """
        done = ignored = 0
        if sc is not None:
            done = sc.dut_log().count(START_LOG_OK)
            ignored = sc.dut_log().count(START_LOG_IGNORED)
        self._start.publish(Empty())
        if sc is None:
            return
        deadline = sc.t() + timeout_s
        while sc.t() < deadline:
            sc.hold(0.05)
            log = sc.dut_log()
            if log.count(START_LOG_OK) > done:
                return
            n_ignored = log.count(START_LOG_IGNORED)
            if n_ignored > ignored:
                ignored = n_ignored
                sc.log("start 被 DUT 丢了一次（还没选轨迹）⟹ 补发一帧")
                self._start.publish(Empty())


# ─────────────────────────── 小工具 ───────────────────────────


def wait_dut_up(sc, timeout_s=15.0):
    """等 DUT 起来，**两个方向都等到**。

    ★ 判据是 graph 里那五条话题都到齐了，而不是"我们 sleep 了 1 秒"。
      时机很关键，而且**方向要分开看**：
        · 它的发布者（reference/progress/marker）—— 我们不等到它就断言"没收到参考"，
          是在测"DDS 发现快不快"，不是在测 traj。
        · 它的订阅者（select/start）—— ★ 这一条是实测踩出来的：只等发布者的话，
          我们那条**只发一次**的 `/traj/start` 可能在 DUT 的订阅被匹配之前就发出去了，
          然后**没有任何重发**（Empty 没有 id，补不了）⟹ 表现为"start 发了但参考不动"，
          而节点日志里其实什么都没收到。等订阅者到齐，这条竞态就没了。
    """
    sc.wait_for(
        lambda: sc.node.count_publishers(REF_TOPIC) >= 1
        and sc.node.count_publishers(PROGRESS_TOPIC) >= 1
        and sc.node.count_publishers(MARKER_TOPIC) >= 1
        and sc.node.count_subscribers(SELECT_TOPIC) >= 1
        and sc.node.count_subscribers(START_TOPIC) >= 1,
        timeout_s=timeout_s,
        message="traj 节点没起来：graph 里看不到它的发布者/订阅者（可执行名/包名对了吗？）")


def norm(v):
    return math.sqrt(v.x * v.x + v.y * v.y + v.z * v.z)


def stamp_s(msg):
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


def speed_from_refs(refs, i):
    """用相邻两帧的**位置差分**量速度 —— 与节点报告的 `velocity` 是两条独立证据。

    ★ 这条差分是必要的：若只在"θ 均匀"而不是"弧长均匀"的轨迹上发布，节点报告的
      `|velocity|` 仍然写着 0.5，而**位置实际走多快**会冲到 ~1.4×。差分才拦得住它。

    ★★ 必须用消息里的 `header.stamp`，**不能**用我们收到的时刻（第一手踩出来的）：
      参考是按**真实时间**参数化的（节点用 `now() - start_time` 算 t），而 rcl 的定时器是
      "从回调开始时刻重新计时"（`rcl/timer.h`）—— 一次调度延迟之后紧接着的几个 tick 会
      **背靠背补跑**，于是两帧的**接收时刻差**可能只有 0.1 ms，而位置差是 0.1 s 的量。
      实测在正方形上因此假报了 4.5 m/s（= 9× 限速）。position 是 stamp 的函数，
      用 stamp 差分出来的才是真速度（残余误差 ~1e-4，见 trajectories.hpp 的波纹说明）。
    """
    if i <= 0 or i >= len(refs) - 1:
        return 0.0
    dt = stamp_s(refs[i + 1][1]) - stamp_s(refs[i - 1][1])
    if dt <= 0:
        return 0.0
    p0, p1 = refs[i - 1][1].position, refs[i + 1][1].position
    return math.dist((p0.x, p0.y, p0.z), (p1.x, p1.y, p1.z)) / dt


# ─────────────────────────── 用例 ───────────────────────────


def test_选轨迹后不发参考_必须等_start():
    """★★ 本节点最重要的一条时序规则。

    若"选中"就是零点，那么从选中到 control 进 TRACK 之间的那几秒里参考已经跑到轨迹中段，
    control 一进 TRACK 就要把飞机猛地拽过去 —— Phase 1 最怕的就是这种指令跳变。
    """
    with Scenario(dut=DUT, mocap=False) as sc:
        rec, cmd = TrajRecorder(sc.node, sc.t), TrajCommander(sc.node)
        wait_dut_up(sc)

        # ① 什么都没发之前：一帧参考都不该有（此时 DUT 的发布者已在 graph 里 ⟹ 不是"没发现"）
        sc.hold(0.5)
        assert rec.refs == [], f"还没选轨迹就收到参考帧：{rec.refs[:3]}"

        # ② 选一个正方形（阳性对照：折线 marker 必须回来 —— 证明 DUT 收到了 select、
        #    并且它的发布者与我们的订阅**双向**发现成功。没有这条，①的"空"就毫无意义）
        cmd.select(2)
        assert rec.wait_for_marker(sc, 0), (
            "选了轨迹但连折线 marker 都没回来 ⟹ DUT 没收到 /traj/select，"
            "或在 /traj/marker 上没对上\n" + sc.evidence())

        # ③ 选了但**没开始**：仍然一帧参考都不发（这才是本条要测的判据）
        sc.hold(1.0)
        assert rec.refs == [], (
            "选了轨迹、还没发 /traj/start，参考就已经在流了 —— "
            "参考时钟零点错了，control 进 TRACK 时会被甩一下\n" + sc.evidence())
        assert rec.progress == [], "完成度也不该在开始之前发\n" + sc.evidence()

        # ④ 发 start：参考立刻开始流（同一套 recorder，证明①③的空是判据而非故障）
        cmd.start(sc=sc)
        # ★ message 只写"判断句"，**不要在这里拼 sc.evidence()**：它是**急求值**的，
        #   每次调用都会跑一遍（而且一旦里面出问题，测试会以那个问题失败，
        #   完全盖住真正要报的判据）。bench 的 wait_for 在超时时自己会附上证据。
        sc.wait_for(lambda: len(rec.refs) >= 5, timeout_s=5.0,
                    message="发了 /traj/start 但参考还是没流出来")


def test_没选轨迹就_start_被忽略且只告警():
    with Scenario(dut=DUT, mocap=False) as sc:
        rec, cmd = TrajRecorder(sc.node, sc.t), TrajCommander(sc.node)
        wait_dut_up(sc)

        # ★ 这里**刻意**用不带 `sc` 的一次性形式：本条要测的正是"这一次 start 被忽略"，
        #   `start(sc=...)` 的补发会把判据补没了（见 `TrajCommander.start` 的说明）。
        cmd.start()
        sc.hold(0.8)
        assert rec.refs == [], "还没选轨迹，start 就把参考放出来了\n" + sc.evidence()
        # ★ 阳性对照：DUT 必须**明确说过**它忽略了（否则"没反应"与"没收到"分不开）
        assert START_LOG_IGNORED in sc.dut_log(), (
            "start 在没有轨迹时既没发参考、也没有告警 —— 分不清是忽略了还是没收到\n"
            + sc.evidence())

        # 之后正常选 + start 仍然能用
        cmd.select(1)
        cmd.start(sc=sc)
        sc.wait_for(lambda: len(rec.refs) >= 5, timeout_s=5.0,
                    message="补选轨迹后参考没流出来")


def test_参考消息的口径_掩码_坐标系_frame_与_yaw():
    """载体是 `mavros_msgs/PositionTarget`：这三项一旦漂了，参考的语义就变了。"""
    with Scenario(dut=DUT, mocap=False) as sc:
        rec, cmd = TrajRecorder(sc.node, sc.t), TrajCommander(sc.node)
        wait_dut_up(sc)
        cmd.select(1)
        cmd.start(sc=sc)
        sc.wait_for(lambda: len(rec.refs) >= 5, timeout_s=5.0, message="参考没流出来")

        msg = rec.latest_ref()
        assert msg.type_mask == TYPE_MASK_POS_VEL_YAW, (
            f"type_mask = {msg.type_mask}，期望 {TYPE_MASK_POS_VEL_YAW}"
            "（位置+速度+yaw）。掩码决定哪些分量有意义，改了必须是有意的。")
        assert msg.coordinate_frame == FRAME_LOCAL_NED
        # ★ frame_id 是**标注**：mavros 的 setpoint_raw 只读 coordinate_frame 与 type_mask，
        #   从不读 frame_id（mavros/src/plugins/setpoint_raw.cpp:209-249）。
        #   写 traj_origin 是为了让 ros2 topic echo 的人一眼看出"这不是 map 系"。
        assert msg.header.frame_id == "traj_origin", msg.header.frame_id
        assert msg.yaw == pytest.approx(0.0), "Phase 1 恒定 yaw（§8.6：yaw 前馈先不做）"


def test_悬停_位置恒在原点_完成度按时长走完():
    # 时长压到 2 s：悬停的完成度靠 hover_duration_s 兜，测试里没必要等 60 s。
    with Scenario(dut=DUT, mocap=False, params={"hover_duration_s": 2.0}) as sc:
        rec, cmd = TrajRecorder(sc.node, sc.t), TrajCommander(sc.node)
        wait_dut_up(sc)
        cmd.select(1)
        cmd.start(sc=sc)
        sc.wait_for(lambda: any(v >= 1.0 for v in rec.progress_since(0)), timeout_s=12.0,
                    message="悬停的完成度没走到 1.0")

        refs = rec.refs_since(0)
        for _, m in refs:
            assert norm(m.position) == pytest.approx(0.0, abs=1e-9), (
                f"悬停的位置不是原点：({m.position.x}, {m.position.y}, {m.position.z})")
            assert norm(m.velocity) == pytest.approx(0.0, abs=1e-9)
        # 完成度单调不减，且确实经过了 1.0
        vals = rec.progress_since(0)
        assert vals == sorted(vals), "完成度回退了"
        assert max(vals) == pytest.approx(1.0)
        # 跑完之后**继续发**（参考停在终点，由 control 自己按 progress ≥ 1.0 收尾，§4.2）
        n = len(rec.refs)
        sc.hold(0.4)
        assert len(rec.refs) > n, "跑完之后参考就停发了 —— control 会失去参考流"


def test_起飞_在话题上是一条斜坡_到顶后停住():
    """★ 轨迹 4（ticket 14 二定）：从**轨迹原点**升到 `takeoff_height_m`，然后停在那儿。

    ★ 速度一律用**位置差分**独立量（同 `speed_from_refs` 的理由）：节点报告出来的 `velocity`
      与位置是同一个东西的两种说法，只有差分是独立证据。
    ★ 时长压到 2 s：本条要看的"到顶那一刻还没跑完"没必要等 60 s。
    """
    height, speed, duration = 0.2, 0.2, 2.0
    with Scenario(dut=DUT, mocap=False,
                  params={"takeoff_height_m": height, "takeoff_speed_ms": speed,
                          "hover_duration_s": duration}) as sc:
        rec, cmd = TrajRecorder(sc.node, sc.t), TrajCommander(sc.node)
        wait_dut_up(sc)
        cmd.select(4)
        cmd.start(sc=sc)
        sc.wait_for(lambda: len(rec.refs) >= 5, timeout_s=10.0,
                    message="选了 4 也发了 start，参考还是没流出来（4 进 parse_kind 了吗？）")
        # ★★ 必须等**整条**跑完再分析：只等"来了 5 帧"就下手，拿到的是 0.08 s 的数据 ——
        #    那时还在斜坡脚下，下面每一段判据都会**空着通过**（爬升段一帧都没有）。
        sc.wait_for(lambda: any(v >= 1.0 for v in rec.progress_since(0)), timeout_s=12.0,
                    message="完成度没走到 1.0（时长只给了 2 s）")

        refs = rec.refs_since(0)
        zs = [m.position.z for _, m in refs]

        # ① 零点 = 轨迹原点：第一帧必须还在**起点附近**，不是一上来就在高度上。
        #    容差 0.02 m = 斜坡里 0.1 s 的量 —— start 到第一帧之间不可能有更长的空档
        #    （50 Hz，而且 recorder 的订阅在 cmd.start 之前就已匹配，见 wait_dut_up）。
        assert zs[0] == pytest.approx(0.0, abs=0.1 * speed), (
            f"起飞的第一帧 z = {zs[0]}，离轨迹原点太远 —— 零点错了 ⟹ "
            "「起飞 20 cm」会变成别的意思")
        assert zs == sorted(zs), "起飞过程中 z 回退了（那不是斜坡，是抖动）"

        # ② 爬升段（只取斜坡内部，避开两个折点）：差分速度 ≈ takeoff_speed_ms
        climbing = [i for i, z in enumerate(zs) if 0.02 < z < height - 0.02]
        assert len(climbing) >= 3, (
            f"斜坡内部只有 {len(climbing)} 帧 —— 要么没在爬，要么这条斜坡短到看不见"
            f"（z 尾部：{zs[-5:]}）")
        for i in climbing:
            v = speed_from_refs(refs, i)
            assert v == pytest.approx(speed, abs=SPEED_TOL), (
                f"第 {i} 帧（z = {zs[i]:.4f}）的实际爬升速度是 {v:.3f} m/s，不是 {speed}")
            # ★ 差分只是位置的说法；**上报的 velocity** 是另一份、且是下游真正吃进去的那份
            #   （模型预测控制档把它当速度参考）⟹ 两份都要对，不能只对一份。
            assert refs[i][1].velocity.z == pytest.approx(speed, abs=SPEED_TOL), (
                f"第 {i} 帧上报的 velocity.z = {refs[i][1].velocity.z}，不是 {speed}"
                "（位置对了、速度参考却是 0 或别的值）")

        # ③ 到顶：正好停在 height（**不越界**），之后一直停在那儿、速度归 0
        top = next(i for i, z in enumerate(zs) if z >= height - 1e-9)
        assert max(zs) == pytest.approx(height, abs=1e-9), (
            f"z 的最大值是 {max(zs)}，越过了 takeoff_height_m = {height}")
        for i in range(top + 1, len(zs) - 1):
            assert zs[i] == pytest.approx(height, abs=1e-9), f"第 {i} 帧掉出了高度：{zs[i]}"
            assert speed_from_refs(refs, i) == pytest.approx(0.0, abs=SPEED_TOL), (
                f"到顶之后第 {i} 帧还在动：{speed_from_refs(refs, i):.4f} m/s")
            assert refs[i][1].velocity.z == pytest.approx(0.0, abs=SPEED_TOL), (
                f"到顶之后第 {i} 帧仍在上报 velocity.z = {refs[i][1].velocity.z}")

        # ④ 完成度**按时长**走：到顶那一刻只走了斜坡那一截（≈ h/v/duration），远没有跑完。
        #    完成度走的是**另一条话题**（`/traj/progress`），按**到达时刻**配对 ——
        #    同一个 tick 里两条 payload 前后脚发出，0.1 s 的窗口足够圈住那一拍。
        top_at = refs[top][0]
        near = [v for at, v in rec.progress if abs(at - top_at) <= 0.1]
        assert near, f"到顶那一拍（t = {top_at:.2f} s）前后没有完成度帧"
        assert max(near) == pytest.approx((height / speed) / duration, abs=0.15), (
            f"到顶那一刻完成度是 {max(near)}，按\"时长 2 s、斜坡 1 s\"应该是 "
            f"{(height / speed) / duration} —— 1.0 意味着完成度被做成按斜坡走"
            "（`control` 那条 `progress ≥ 1 ⟹ LAND` 会在起飞刚结束时就触发），"
            "0.0 意味着完成度压根没在走")


def test_正方形_四条边与限速_在话题上真的成立():
    with Scenario(dut=DUT, mocap=False) as sc:
        rec, cmd = TrajRecorder(sc.node, sc.t), TrajCommander(sc.node)
        wait_dut_up(sc)
        cmd.select(2)
        cmd.start(sc=sc)
        # 8.0 s 跑完（4 m / 0.5 m/s）；★ 余量给得很大：轨迹是**实时**参数化的，
        #   Jetson 被别的进程压住时 DUT 走得比墙钟慢（实测全量跑时红过一次），
        #   而超时放宽只花"真出问题时的等待"，不影响绿的那条路径。
        sc.wait_for(lambda: any(v >= 1.0 for v in rec.progress_since(0)), timeout_s=20.0,
                    message="正方形没跑完")

        refs = rec.refs_since(0)
        ps = [m.position for _, m in refs]

        # ① 起点在原点（= 悬停点，所以进 TRACK 时不会有跳变）
        assert norm(ps[0]) == pytest.approx(0.0, abs=0.02), (
            f"正方形不是从原点出发：({ps[0].x}, {ps[0].y}, {ps[0].z})")

        # ② 全程在 [0, 1] 这个 1 m 见方的格子里，且**四个角都真的够到了**
        assert all(-0.02 <= p.x <= 1.02 and -0.02 <= p.y <= 1.02 for p in ps), (
            "参考跑出了边长 1 m 的正方形："
            f"x ∈ [{min(p.x for p in ps):.3f}, {max(p.x for p in ps):.3f}]，"
            f"y ∈ [{min(p.y for p in ps):.3f}, {max(p.y for p in ps):.3f}]")
        assert max(p.x for p in ps) > 0.98 and max(p.y for p in ps) > 0.98, "远角 (1,1) 没到"
        assert min(p.x for p in ps) < 0.02 and min(p.y for p in ps) < 0.02, "近边没回到 0"

        # ③ 报告的 |v| 与**位置差分**量的速度，两条独立证据都不超过限速
        for _, m in refs:
            assert norm(m.velocity) <= MAX_SPEED_MS + SPEED_TOL, "报告的 |v| 超速"
        # ★ 差分跨过拐角时量到的是**弦长** < 弧长 ⟹ 只会**低估**，不会假报超速。
        #   所以这里容差可以很紧：真出问题（比如限速没缩放）会差好几倍，不是几个百分点。
        worst = max(speed_from_refs(refs, i) for i in range(1, len(refs) - 1))
        assert worst <= MAX_SPEED_MS + SPEED_TOL, (
            f"位置差分量的速度到过 {worst:.3f} m/s，超过 {MAX_SPEED_MS} m/s")

        # ④ 完成度到 1.0，且跑完停在起点（正方形闭合）
        assert max(rec.progress_since(0)) == pytest.approx(1.0)
        assert norm(rec.latest_ref().position) == pytest.approx(0.0, abs=0.02), (
            "正方形跑完之后参考没停在起点")
        assert norm(rec.latest_ref().velocity) == pytest.approx(0.0, abs=1e-9), (
            "跑完之后还在报速度：飞机会被继续推着走")


def test_正方形_参考按_50_Hz_发():
    with Scenario(dut=DUT, mocap=False) as sc:
        rec, cmd = TrajRecorder(sc.node, sc.t), TrajCommander(sc.node)
        wait_dut_up(sc)
        cmd.select(2)
        cmd.start(sc=sc)
        sc.wait_for(lambda: len(rec.refs) >= 100, timeout_s=10.0, message="参考帧太少")

        # 只在**流中间**的窗口上算频率（避开 start 瞬间的发现抖动与结束边界）
        t_lo, t_hi = rec.refs[20][0], rec.refs[-1][0] - 0.2
        rate = rec.rate_hz(t_lo, t_hi)
        assert 45.0 <= rate <= 55.0, (
            f"[{t_lo:.2f}, {t_hi:.2f}] s 窗口内实测 {rate:.1f} Hz，"
            f"期望 50 Hz（与 control 的 setpoint 同频，§2）")


def test_八字_幅度_偏扁_与限速():
    with Scenario(dut=DUT, mocap=False) as sc:
        rec, cmd = TrajRecorder(sc.node, sc.t), TrajCommander(sc.node)
        wait_dut_up(sc)
        cmd.select(3)
        cmd.start(sc=sc)
        # a = 1.0 ⟹ 周长 ≈6.10 m @0.5 m/s ≈ 12.2 s（余量同上：实时参数化 + 机器可能被压住）
        sc.wait_for(lambda: any(v >= 1.0 for v in rec.progress_since(0)), timeout_s=30.0,
                    message="8 字没跑完")

        refs = rec.refs_since(0)
        ps = [m.position for _, m in refs]
        xs = [p.x for p in ps]
        ys = [p.y for p in ps]

        # ★ Gerono 双纽线是**偏扁**的：x 半幅 1.0、**y 半幅只有 0.5**。
        #   这是规格不是缺陷（§5.2 写了），所以在话题层把它钉住。
        #   ★ 2026-10-08 幅度从 0.5 放大到 1.0（原本的 2 倍）—— 见 .scratch/phase2/issues/27。
        assert min(xs) == pytest.approx(-1.0, abs=0.02) and max(xs) == pytest.approx(1.0, abs=0.02), (
            f"8 字的 x 幅度不对：x ∈ [{min(xs):.3f}, {max(xs):.3f}]，期望 ±1.0")
        assert min(ys) == pytest.approx(-0.5, abs=0.02) and max(ys) == pytest.approx(0.5, abs=0.02), (
            f"8 字的 y 幅度不对：y ∈ [{min(ys):.3f}, {max(ys):.3f}]，期望 ±0.5（= x 的一半）")
        # x 与 y 的幅度比 ≈ 2 —— 这条把"偏扁"直接表述成不变式
        assert max(xs) / max(ys) == pytest.approx(2.0, abs=0.15)

        assert all(-1.0 - SPEED_TOL <= p.x <= 1.0 + SPEED_TOL for p in ps)
        assert all(-0.5 - SPEED_TOL <= p.y <= 0.5 + SPEED_TOL for p in ps)
        worst = max(speed_from_refs(refs, i) for i in range(1, len(refs) - 1))
        assert worst <= MAX_SPEED_MS + SPEED_TOL, (
            f"8 字的位置差分速度到过 {worst:.3f} m/s —— 弧长不均匀（过中心那一段冲快了）；"
            "若它只在 0.5 附近摆几十个百分点以外，就是 θ 匀速而不是弧长匀速")
        # 跑完回到原点（曲线闭合）
        assert norm(rec.latest_ref().position) == pytest.approx(0.0, abs=0.02)


def test_折线_marker_不开始也能看到形状与尺寸():
    """RViz 那条验收判据的机器可查版本。

    ★ 折线在**选中时**就发（不必等 start）：否则"离线核对轨迹形状"这件事就得先起飞。
    """
    with Scenario(dut=DUT, mocap=False) as sc:
        rec, cmd = TrajRecorder(sc.node, sc.t), TrajCommander(sc.node)
        wait_dut_up(sc)

        # 正方形：5 个顶点、四条 1.0 m 的边、闭合
        cmd.select(2)
        assert rec.wait_for_marker(sc, 0), "选了轨迹但折线 marker 没来\n" + sc.evidence()
        strip = [m for m in rec.markers_of(0) if m.type == Marker.LINE_STRIP][-1]
        assert strip.action == Marker.ADD
        assert strip.header.frame_id == "map", (
            f"marker 的 frame 是 {strip.header.frame_id!r}：必须是 RViz 有 TF 的那个（默认 map），"
            "写 traj_origin 会让 RViz 直接把它丢掉")
        pts = [(p.x, p.y) for p in strip.points]
        assert len(pts) == 5, f"正方形折线应有 5 个顶点（闭合），实际 {len(pts)}"
        for i in range(1, len(pts)):
            edge = math.dist(pts[i - 1], pts[i])
            assert edge == pytest.approx(1.0, abs=1e-6), f"第 {i} 条边长 {edge:.4f} m，期望 1.0"
        assert pts[0] == pytest.approx(pts[-1]), "折线没闭合"

        # ★★ 折线必须**周期重发**（yaml 的 marker_period_s）。Marker 是 volatile 的：
        #   "选中时发一次"对**晚起的 RViz** 等于没发（实测踩到过 —— 重发语句被 tick() 里
        #   的 `!running_` 提前返回挡住，于是"选了但没开始"时折线只存在一瞬间，
        #   而"选了没开始"恰恰是离线核对形状的用法）。
        n_strips = len([m for m in rec.markers_of(0) if m.type == Marker.LINE_STRIP])
        sc.wait_for(lambda: len([m for m in rec.markers_of(0)
                                 if m.type == Marker.LINE_STRIP]) > n_strips,
                    timeout_s=3.0,
                    message="折线没有周期重发 —— 晚起的 RViz 会永远看不到它（尤其是没开始的时候）")

        # 8 字：x 幅度 ±1.0、y 幅度 ±0.5（换轨迹**不需要重启节点**）
        n_before = len(rec.refs)
        cmd.select(3)

        # ★ 按**内容**认折线，不按"又来了几条"：正方形的**周期重发**随时可能插进来，
        #   `[-1]` 于是拿到重发的正方形（实测踩到过：5 个点的折线去比 ±1.0）。
        #   点数就是不变量：正方形 5 个顶点、8 字 201+1。
        def lem_strips():
            return [m for m in rec.markers_of(0)
                    if m.type == Marker.LINE_STRIP and len(m.points) == 202]

        assert sc.poll(lambda: len(lem_strips()) >= 1, timeout_s=5.0), (
            "重选轨迹后折线没更新 —— 换轨迹应当不需要重启节点\n" + sc.evidence())
        strip = lem_strips()[-1]
        xs = [p.x for p in strip.points]
        ys = [p.y for p in strip.points]
        # ★ 容差 1e-3 而不是 1e-6：折线是按 **θ 均匀**采样的（刻意的，见 trajectories.hpp），
        #   而 x 的极值在 θ=3π/2 —— 201 个采样点未必正好落在那儿。这是采样的误差，
        #   不是形状的误差；1e-3 远小于"形状不对"会有的偏差（换错公式至少差 1e-1）。
        assert min(xs) == pytest.approx(-1.0, abs=1e-3) and max(xs) == pytest.approx(1.0, abs=1e-3)
        assert min(ys) == pytest.approx(-0.5, abs=1e-3) and max(ys) == pytest.approx(0.5, abs=1e-3)
        assert len(strip.points) == 202, f"8 字折线点数 {(len(strip.points))}，期望 201+1"
        # ★ 阳性对照：换轨迹**不等于**开始 —— 参考仍然一帧都不发
        assert len(rec.refs) == n_before, "只是换了个轨迹，参考就自己流起来了"


def test_飞行中重选_必须重新_start_且完成度归零():
    with Scenario(dut=DUT, mocap=False) as sc:
        rec, cmd = TrajRecorder(sc.node, sc.t), TrajCommander(sc.node)
        wait_dut_up(sc)
        cmd.select(2)
        cmd.start(sc=sc)
        sc.wait_for(lambda: len(rec.refs) >= 50, timeout_s=5.0, message="正方形没开始流")

        # 重选（模拟"操作者改主意"）
        n_refs = len(rec.refs)
        cmd.select(3)

        # ★★ 两个窗口，缺一不可（这是**竞态**，不是"等一下就好了"）：
        #   ① 抽干窗口 —— `select` 到达 DUT 之前，可能有**已经上路**的参考帧（50 Hz ⟹ 一帧
        #      20 ms）。把它当成"重选没生效"是**假阳性**：判据是"重选**生效之后**停流"，
        #      而"哪一帧是生效后的第一帧"在测试里看不见。容忍 5 帧（0.1 s）是把
        #      "DDS 把 select 送到 DUT 回调"的延迟算进去，不是允许它慢慢停。
        #   ② 静默窗口 —— 只看①的话，"停得很慢"会被容忍度放过去。② 才是真正的判据：
        #      抽干之后必须**一帧都没有**。
        sc.hold(0.6)
        n_drained = len(rec.refs)
        assert n_drained - n_refs <= 5, (
            f"重选之后参考还在流（{n_refs} → {n_drained} 帧）—— 若此时 control 还在 TRACK，"
            "飞机会被从旧轨迹拽到新轨迹上\n" + sc.evidence())
        sc.hold(0.3)
        assert len(rec.refs) == n_drained, (
            f"重选之后参考没有真正停流（静默窗口里又来了 {len(rec.refs) - n_drained} 帧）\n"
            + sc.evidence())

        # ★ 必须再发一次 start；而且新轨迹是**从 0 开始**的（完成度归零，而不是接着旧的一起跑）
        n_resume = len(rec.refs)
        cmd.start(sc=sc)
        sc.wait_for(lambda: len(rec.refs) > n_resume, timeout_s=5.0,
                    message="重选后 start 没生效")
        # ★ 取**静默窗口之后**的第一帧（`n_resume`，不是 `n_refs`）：重选前的旧帧不属于新轨迹。
        t_resume = rec.refs[n_resume][0]
        first = rec.refs_since(n_resume)[0][1]
        assert norm(first.position) == pytest.approx(0.0, abs=0.02), (
            "重选后第一帧不在原点 —— 说明参考时间轴没有真正归零")
        # 按**时刻**取重选之后的完成度（不按下标：两张表的帧数在边界上可能差一帧）
        new_prog = [v for at, v in rec.progress if at >= t_resume]
        assert new_prog, "重选后没有完成度上报\n" + sc.evidence()
        assert min(new_prog) < 0.1, (
            f"重选后的完成度最小只有 {min(new_prog):.3f}，没有从 0 重新开始 —— "
            "参考时间轴还在用旧轨迹的 t")


def test_清空_参考停发_marker_删除_完成度报零():
    with Scenario(dut=DUT, mocap=False) as sc:
        rec, cmd = TrajRecorder(sc.node, sc.t), TrajCommander(sc.node)
        wait_dut_up(sc)
        cmd.select(2)
        cmd.start(sc=sc)
        sc.wait_for(lambda: len(rec.refs) >= 50, timeout_s=5.0, message="正方形没开始流")

        n_refs, n_prog = len(rec.refs), len(rec.progress)
        cmd.select(0)

        # 抽干 + 静默两个窗口，理由与 `test_飞行中重选…` 同一份（那里的注释写全了）：
        # 清空到达 DUT 之前可能有已经上路的参考帧，但**静默窗口里一帧都不许有**。
        sc.hold(0.6)
        n_drained = len(rec.refs)
        assert n_drained - n_refs <= 5, (
            f"清空之后参考还在流（{n_refs} → {n_drained} 帧）\n" + sc.evidence())
        sc.hold(0.3)

        # ① 参考与完成度都停发
        assert len(rec.refs) == n_drained, (
            f"清空之后参考没有真正停流（静默窗口里又来了 {len(rec.refs) - n_drained} 帧）\n"
            + sc.evidence())
        # ② 阳性对照：清空的**最后一帧完成度必须是 0.0**（"我清空了"这件事要被显式说出来，
        #    而不是让下游靠"话题静默"去猜）。
        #    ★ 只看"末帧"而不是"恰好一帧"：清空生效前可能还差一帧在飞 —— 那不是判据的一部分。
        tail = rec.progress_since(n_prog)
        assert tail and tail[-1] == pytest.approx(0.0), (
            f"清空后没有以 0.0 收尾，完成度尾巴是 {tail[-5:]}")
        # ③ marker 被删除（否则 RViz 里会留一条已经不存在的轨迹，很误导）
        assert sc.poll(lambda: any(
            m.action == Marker.DELETE for m in rec.markers_of(0)), timeout_s=2.0), (
            "清空之后折线 marker 没被 DELETE，RViz 里会留一条幽灵轨迹")
        # ④ 清空后进程仍然活着（清空是正常操作，不是"崩了所以停发"）
        assert sc.dut_alive(), "清空把节点搞崩了\n" + sc.dut_log()


def test_无效编号被忽略且保持当前轨迹():
    with Scenario(dut=DUT, mocap=False) as sc:
        rec, cmd = TrajRecorder(sc.node, sc.t), TrajCommander(sc.node)
        wait_dut_up(sc)
        cmd.select(2)
        cmd.start(sc=sc)
        sc.wait_for(lambda: len(rec.refs) >= 50, timeout_s=5.0, message="正方形没开始流")

        progress_before = rec.progress_since(0)[-1]
        n = len(rec.refs)
        cmd.select(9)  # 越界值
        sc.hold(0.6)

        # ★ 选"保持不动"而不是"清空"：突然清空比没反应更危险（多半是敲错了）。
        assert len(rec.refs) > n, "发了个无效编号，参考就停了\n" + sc.evidence()
        assert "忽略无效的" in sc.dut_log(), (
            "无效编号既没有告警、也没有停 —— 操作者无从知道自己的命令被丢弃了\n"
            + sc.dut_log())
        assert rec.progress_since(0)[-1] > progress_before, (
            "无效编号把完成度重置了 ⟹ 轨迹被重建了，应当原地不动")
