#!/usr/bin/env python3
"""操作台 —— 选轨迹 + 实时看状态与完成度（ticket 09 的"启动脚本"）。

    ros2 run bringup fly.py

    1 = 原地悬停   2 = 1 m 正方形   3 = 1 m 八字
    0 = 取消选择   h = 帮助   q = 退出   （回车 = 立刻打印一次当前状态）

★★ 本脚本**只做两件事**：发 `/traj/select`（`std_msgs/UInt8`）、显示 `/control/state` 与
   `/control/traj_progress`。它**不**发 `/traj/start`、**不**调起飞服务、**不**碰 setpoint——
   这三件事各有各的主人，抢着做只会让"谁在驱动飞机"变成一笔糊涂账：

   · `/traj/start` **由 `control` 在进 `TRACK` 那一刻自己发**（ticket 09 Comments ②）：
     参考时钟的零点必须与"开始飞"同一时刻。这里再发一次 = 把正在跑的参考时钟归零，
     飞机当场掉头飞回轨迹起点 —— 且现场看不出是谁干的。
   · 起飞 = **遥控器开始键的第 1 次上升沿**（§H.5）。这条防线是刻意的：
     "开始键按下之前不发任何控制指令"。
     ⚠️ 注意别把这句话读成"程序解锁不存在" —— `/control/takeoff` 服务**是**存在的，
     它是**拆桨台架**用的（台上没有遥控接收机，"RC 在线"这个起飞前置条件本身就过不了），
     现场走的是开始键。本脚本**刻意不调它**：现场多一个程序入口，就多一倍"是谁让它起飞的"
     的排查面（`control/README.md` 里两条入口共用同一个受理路径 `Fsm::request_takeoff`）。
   · 轨迹**必须在起飞前就选好**（`traj` 的 `select` 只是记下选哪条，不发任何参考），
     但**运行中重选也是支持的**：`control` 会发现参考流断了、保持上一个目标、
     并按 `traj_stream_gap_s`（0.5 s）的闸门重发 `/traj/start` —— 这一条它会自己处理，
     操作员不用管（`degrade=traj_timeout` 只在 3 s 内一直没恢复时才判）。
     所以脚本在"已在飞行"时重选会多打一句提示，提醒你飞机接下来会拐向新轨迹。

★ 显示粒度按**完成度**走（每 5%）而不是按时间：悬停 60 s 不会刷 60 行，
   而 6 s 的八字照样看得见它在动。想立刻看一次读数就按回车。
"""
import select
import sys
import time

import rclpy
from mavros_msgs.msg import State
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float32, String, UInt8

#: 三条轨迹的**名字**（几何与时长在 `traj` 侧：`src/traj/config/traj.yaml` 与它的 README）。
#  ★ 这里只印名字与时长量级，**不印几何** —— 几何改了不会让这份脚本说谎。
KINDS = {
    1: ("原地悬停", "60 s（= §5.3 的验收时长）"),
    2: ("正方形 1 m", "≈8 s"),
    3: ("八字 a=1.0（偏扁：2.0 m × 1.0 m）", "≈12.2 s"),
}

SELECT_TOPIC = "/traj/select"
STATE_TOPIC = "/control/state"
PROGRESS_TOPIC = "/control/traj_progress"
#: 飞控侧的解锁/模式（1 Hz）。★ 与上面三条不同，这条**不是我们的话题** —— 它归 mavros。
MAVROS_STATE_TOPIC = "/mavros/state"

#: 状态串是 `transient_local` 发的（`control_node.cpp`）⟹ 订阅端也要 durable 才拿得到
#: **当前**那一帧；用默认的 volatile 会一直黑屏，直到下一次状态变化（可能是几分钟后）。
STATE_QOS = QoSProfile(
    depth=10,
    history=HistoryPolicy.KEEP_LAST,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


def _bar(value: float, width: int = 20) -> str:
    filled = int(round(max(0.0, min(1.0, value)) * width))
    return "█" * filled + "░" * (width - filled)


class FlyCli(Node):
    def __init__(self) -> None:
        super().__init__("fly")
        self._t0 = time.monotonic()
        self.state = None          # /control/state（"AUTHORITY/PHASE"）
        self.progress = None       # /control/traj_progress（0–1）
        self.armed = None          # /mavros/state.armed
        self.mode = None           # /mavros/state.mode
        self.kind = None           # 我们**发出去**的那条选择（不是从 traj 读回来的）
        self._last_line = ""

        self._select_pub = self.create_publisher(UInt8, SELECT_TOPIC, 10)
        self.create_subscription(String, STATE_TOPIC, self._on_state, STATE_QOS)
        self.create_subscription(Float32, PROGRESS_TOPIC, self._on_progress, 10)
        # ⚠️ `/mavros/state` 只有 **1 Hz**（见 control.yaml 里那段）⟹ armed/mode 最多晚 1 s
        self.create_subscription(State, MAVROS_STATE_TOPIC, self._on_mavros_state, 10)

    # ─────────────────────────── 回调 ───────────────────────────

    def _on_state(self, msg: String) -> None:
        self.state = msg.data

    def _on_progress(self, msg: Float32) -> None:
        self.progress = float(msg.data)

    def _on_mavros_state(self, msg: State) -> None:
        self.armed = bool(msg.armed)
        self.mode = msg.mode

    # ─────────────────────────── 动作 ───────────────────────────

    def select(self, kind: int) -> None:
        """把选择发给 `traj`（0 = 取消选择）。**不发 start** —— 见文件头。"""
        msg = UInt8()
        msg.data = kind
        self._select_pub.publish(msg)
        if kind == 0:
            self.kind = None
            print("→ 已发出：取消选择（traj 会停流，并把完成度清零）")
            return
        name, duration = KINDS[kind]
        self.kind = kind
        print(f"→ 已发出：选轨迹 {kind} = {name}（{duration}）")
        if self.armed:
            print("   ⚠️ 飞机**现在是解锁的**：轨迹会在下一次进 TRACK 时换掉；"
                  "control 会保持上一个目标并按 0.5 s 的闸门重发 /traj/start，"
                  "3 s 内一直没恢复才会判 traj_timeout 并降落。")

    # ─────────────────────────── 显示 ───────────────────────────

    def status_line(self, coarse: bool = False) -> str:
        """`coarse=True` = **比较键**（只用来判"要不要重画"，不打印）：去掉时间戳、
        完成度按 5% 取整。

        ★★ 时间戳**必须**排除在比较键之外 —— 台架实测抓到的第一条缺陷：`[t=…]` 每 0.1 s
           都在变 ⟹ 键帧帧不同 ⟹ 去重形同虚设，悬停 60 s 照样刷 600 行。
        """
        parts = []
        if not coarse:
            parts.append(f"[t={time.monotonic() - self._t0:5.1f}s]")
        parts.append(f"状态={self.state or '（还没收到 /control/state）'}")
        if self.progress is None:
            parts.append("完成度=—")
        else:
            shown = round(self.progress * 20) / 20 if coarse else self.progress
            parts.append(f"完成度={shown:5.3f} {_bar(self.progress)}")
        parts.append(f"已选={'—' if self.kind is None else KINDS[self.kind][0]}")
        if self.armed is not None:
            parts.append(f"飞控={'已解锁' if self.armed else '未解锁'}/{self.mode or '?'}")
        return "  ".join(parts)

    def show_if_changed(self) -> None:
        """只在**看得出变化**时打印：状态变、选择变、或完成度跨过 5% 的格子。

        ★ 否则 50 Hz 的完成度会把终端刷成瀑布（那等于没有显示）。
        """
        key = self.status_line(coarse=True)
        if key != self._last_line:
            self._last_line = key
            print(self.status_line(), flush=True)


def _stdin_ready(timeout_s: float = 0.0) -> bool:
    """stdin 上有没有一整行可读（不阻塞）。管道/重定向进来的输入同样有效。"""
    try:
        return bool(select.select([sys.stdin], [], [], timeout_s)[0])
    except (OSError, ValueError):
        return False


def _print_help() -> None:
    print(
        "\n命令：1 = 原地悬停   2 = 1 m 正方形   3 = 1 m 八字   0 = 取消选择\n"
        "      h = 帮助        q = 退出          回车 = 立刻打印一次当前状态\n"
        "★ 起飞与开始轨迹**不在这个脚本里**：遥控器开始键的第 1 次上升沿 = 解锁起飞，\n"
        "  第 2 次 = 进 TRACK（§H.5）。开始键按下之前，control 不会发任何控制指令。\n"
        "★ 本脚本**不发** /traj/start —— 那是 control 进 TRACK 时自己发的（见脚本头部说明）。\n",
        flush=True)


def main() -> int:
    rclpy.init()
    node = FlyCli()
    print("bringup fly —— 选轨迹 + 看状态（Ctrl-C 或 q 退出）")
    _print_help()
    print("已选轨迹：" + "／".join(f"{k}={v[0]}" for k, v in KINDS.items()), flush=True)
    node.show_if_changed()      # 首帧：顺手把"上一次打印过的那行"种下去（否则下面会重复打印一次）

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            if _stdin_ready():
                line = sys.stdin.readline()
                if not line:      # EOF（例如 `echo 2 | ros2 run bringup fly.py`）
                    break
                cmd = line.strip().lower()
                if cmd in ("q", "quit", "exit"):
                    break
                if cmd in ("h", "?", "help"):
                    _print_help()
                elif cmd in ("", "p", "s"):
                    print(node.status_line(), flush=True)   # 回车 = 立刻看一次
                elif cmd in ("0", "1", "2", "3"):
                    node.select(int(cmd))
                else:
                    print(f"? 不认识 {cmd!r} —— 按 h 看帮助", flush=True)
            node.show_if_changed()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
