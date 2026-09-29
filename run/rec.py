#!/usr/bin/env python3
"""`run/rec.py` —— **录制层**：4 个场景的 rosbag，落到 `bag/`（ticket 19）。

    run/z0.py rec_start 1      # 起录场景 1（菜单 **11= 开始录制（选场景 1–4）**）
    run/z0.py rec_status       # 现在在录什么（不进菜单）
    run/z0.py rec_manifest     # 重算 `bag/MANIFEST.md`（不进菜单）

★ **它不碰飞机。** 只做两件事：`ros2 bag record`（**纯订阅**）与拷文件 ——
不解锁、不起飞、不切模式、不改参数、不发任何一帧。这条与 `z0.py` 的边界是同一条，
由 `run/tests/test_rec.py` 的 AST 用例盯着（不许出现 `create_publisher` / `create_service` / `create_client`）。

★★ **录制是前台的一条流程，到点自动收口**（操作者 2026-09-29 拍定）：印场景卡 ⟹
起 `ros2 bag record` ⟹ **倒计时就摆在眼前** ⟹ 到 `Scenario.expect_s`（四份统一 120 s）
自己收口 ⟹ 打印核对表 + 重算清单。想提前停就按 **Ctrl-C**，走的是**同一条** SIGINT 收口路
（见 `watch` / `finalize`）。⚠️ 因此菜单里**没有**"停止录制"这一项 ——
一份包的生命周期就圈在这一条命令里，"录着的同时去按另一项"这件事不存在了。

★ **飞控日志（`.ulg`）不在本层。** 操作者 2026-09-29 拍定飞控日志"**拔 SD 卡直接拷**"，
所以归档是**手工动作**（拷进 `bag/<日期>/fcu/`）；`render_manifest` 只负责把那个目录里
已经有的东西列进总表 —— 它不拷、不删、不猜。

★★ **为什么要有这一层**（第一手，不是"顺手录一下"）：
[`ticket 11`](../.scratch/phase2/issues/11-sitl-tier-b-seam3.md) 的输入是「Phase 1 实飞录下的真实动捕 rosbag」，
而 2026-09-29 全盘搜包（`.bag`/`.mcap`/`*.db3`）**零命中** —— 2026-09-28 那次真机自由飞
**一个包都没留**。B 档要的是"真实噪声与真实延迟"，理想轨迹替不了它。所以 4 个场景是
**按用途反推出来的最小集合**：静止（噪声基线）/ 手动起飞（起飞瞬态）/ 轨迹 1（悬停判据）/
轨迹 3（有机动 ⟹ 延迟 τ 要的激励）。逐条见 `SCENARIOS`。

★ **话题口径是"白名单正则 + 事后核对"，不是 `-a`**：`-a` 会把 `/rosout`、`/parameter_events`
以及**同域里别人起的节点**一起吸进来（本仓库有并行会话）；而写死一串话题名又会在
刚体名（`Z0` / `z0` 都出现过）上静默漏采。所以取 `INCLUDE_REGEX` 这一条前缀正则，
再用 `ros2 bag info` 的**实测帧数**回对 `Scenario.expect` —— "该来的没来"必须看得见，
而不是等人事后发现包里少了东西。第一手（2026-09-29，域 42，没碰飞机）：
`-e "^/(probe_a|rosout)/"` 只订到 `/probe_a/one`，`-e "/probe_"` + `-x "skip"` 把
`/probe_b/skip_me` 滤掉了 ⟹ 两个开关都是 `search` 语义、按预期工作。

★ **压缩**取 `file` + `zstd`（`ros-humble-rosbag2-compression-zstd` 0.15.17 已装，第一手：
`ls /opt/ros/humble/lib/librosbag2_compression_zstd.so`）。压缩发生在**收口那一刻**
（录制自己的日志 `bag/<日期>/<场景>.log` 里是 `Compressing file: …`）⟹ `STOP_GRACE_S` 按它定。
⚠️ 压出来的 `*.db3.zstd` 用 `ros2 bag info` / `play` 读都没问题（第一手已验），
但**拿 sqlite3 直接开 `.db3` 的第三方脚本读不了** —— 要用就走 `rosbag2_py`。

★ **收口怎么收**（第一手：`bash -c 'source … && ros2 bag record …'` 起的进程树里
**只有 python 那个** —— bash 会把最后一条命令 `exec` 掉，`Popen.pid` 就是录制进程本身，
`pgrep -af "ros2 bag record"` 实见 `/usr/bin/python3 /opt/ros/humble/bin/ros2 bag record …`）。
所以收口 = **给那个 pid 发 SIGINT**，与操作者自己按 Ctrl-C 是同一条路（`rosbag2` 收到后
写缓存 → 压包 → 关库，`metadata.yaml` 落地）。
⚠️ 倒计时期间操作者按下的 Ctrl-C 落在**本进程**身上（录制进程自成一个会话，
`start_new_session=True` ⟹ 终端的 SIGINT 传不到它）⟹ `start` 接住 `KeyboardInterrupt`、
**转手把它发给录制进程**。两条来路（到点 / 人按）最终都汇到 `finalize` 一处 ——
所以"提前停"与"录满 120 s"在产物上**没有区别**。
⚠️ 兜底那条路（按名字扫出来的 pid）**只发单个 pid、不发进程组** —— 理由与
`z0.DOWN_TARGETS` 同一条：扫出来的人可能与别人终端的 shell 同组，对那个组放 `killpg`
等于把人家终端一起收掉。
"""

from __future__ import annotations

import contextlib
import datetime as _dt
import hashlib
import json
import os
import re
import shlex
import signal
import subprocess
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Callable, Dict, List, Optional, Sequence, Tuple

#: 仓库根 = `run/` 的上一级（与 `z0.py` 同一个口径，独立写一份免得循环 import）
REPO = Path(__file__).resolve().parents[1]

#: 录制产物**都落在这里**（仓库根的 `bag/`，操作者 2026-09-29 拍定**随 git 入库**）
BAG_ROOT = REPO / "bag"

#: 录制中的那一份状态（**不入库**：它是本机现场的瞬时态，`.gitignore` 挡着）
STATE_FILE = BAG_ROOT / ".recorder.json"

#: 白名单正则 —— `rosbag2` 的 `-e` 是**在话题名上做 `search`**。带 `^` 锚定是为了
#: 不误伤将来某个叫 `/foo/mocap/bar` 的话题。
INCLUDE_REGEX = r"^/(mavros|mocap|traj|control|vrpn)/"

#: 排除正则（空 = 不加 `-x`）。**现在一条都不排除** —— 高频的 `/mavros/imu/data*`
#: 也照录，理由：包是**一次性的现场产物**，"事后发现少了某条"的代价远大于多存几十 MB。
#: 留这个出口只是给"某条话题大到录不动"那一天用的。
EXCLUDE_REGEX = ""

#: 压缩（见文件头）。改之前先确认 `librosbag2_compression_zstd.so` 在。
COMPRESSION: Tuple[str, str] = ("file", "zstd")

#: 收口等待上限：`rosbag2` 要在这一段时间里「写缓存 → 压 zstd → 关库」。
#: 一个 100 MB 级的包在 Orin NX 上压几秒就完；45 s 是留给"包比预期大得多"的余量。
STOP_GRACE_S = 45.0

#: 倒计时刷新间隔。0.5 s 是为了让"剩 N 秒"看起来是连续的（1 s 会跳得很明显），
#: 又不至于把 CPU 花在刷屏上。
TICK_S = 0.5

#: **不是交互终端**时每隔多久打一行（重定向进日志的场合）—— 往文件里灌 `\r` 只会
#: 得到一坨连成一行的东西，所以那条路上改成"每 10 s 一整行"。
PROGRESS_EVERY_S = 10.0


# ─────────────────────────── 4 个场景（票面定义） ───────────────────────────


@dataclass(frozen=True)
class Scenario:
    """一个录制场景：**起哪条链 + 人要做什么 + 包里该有什么**。

    `expect` 的每一条都是**正则**（`search` 语义）—— 刚体名那类动态话题只能这么写。
    收口时逐条回对 `ros2 bag info` 的实测话题，**缺一条就报出来**。
    """

    n: int
    slug: str
    title: str
    chain: str          # 起链用哪条：`up_manual`（动捕+mavros）/ `up`（全链）
    do: str             # 操作者要做的动作（打印给他看）
    expect: Tuple[str, ...]
    expect_s: float     # ★ **到点自动收口**的时长（不再是"提示"—— 见 `start` 的前台倒计时）


# ★ **四个场景统一 120 s，到点自动收口**（操作者 2026-09-29 现场拍定）。
#   演变：最初 1/2 是 90、3 是 150、4 是 120 ⟹ 改为四份统一 90 ⟹ 再改为**四份统一 120**。
#   为什么最后落在 120：场景 3 的轨迹 1 本身要**悬停 60 s**，前面还有爬升到 1 m +
#   判稳 2 s + 两次按开始键，后面还有自动降落 —— 90 s 会把人逼到"赶时间"，而这一趟
#   最看重的是**缓慢**（手飞那段）与**完整**（起飞到落地整段）。120 s 留出余量。
#   ⚠️ 到点是**自动收口**：想提前停就 Ctrl-C（同一条 SIGINT 收口路，见 `start`）。



SCENARIOS: Tuple[Scenario, ...] = (
    Scenario(1, "s1-static", "静止基线（动捕+mavros，飞机不动）", "up_manual",
             "什么都不做：飞机摆好、**静止**（别碰、别挡动捕）；录满 120 s 自动收口",
             ("/mocap/pose_enu", "/vrpn/.*/pose", "/mavros/vision_pose/pose",
              "/mavros/local_position/pose", "/mavros/state"),
             120.0),
    Scenario(2, "s2-manual-takeoff", "定点模式缓慢推摇杆起飞", "up_manual",
             "ch6 拨 **Position** 档 → 摇杆手势解锁 → **缓慢**推油门到 ~1 m →"
             "稳住十几秒 → 缓慢落回、等自动上锁",
             ("/mocap/pose_enu", "/vrpn/.*/pose", "/mavros/rc/in",
              "/mavros/local_position/pose", "/mavros/state"),
             120.0),
    Scenario(3, "s3-traj1-hover", "执行轨迹 1（原地悬停 60 s）", "up",
             "`fly.py` 选 **1** → ch6 拨第 6 档（Offboard）→ 开始键①（解锁+起飞）→"
             "判稳后开始键②松手（进 TRACK）→ 跑完自动降落",
             ("/mocap/pose_enu", "/vrpn/.*/pose", "/traj/reference", "/traj/progress",
              "/control/state", "/mavros/setpoint_raw/local", "/mavros/state"),
             120.0),
    Scenario(4, "s4-traj3-lemniscate", "执行轨迹 3（0.5 m 八字）", "up",
             "`fly.py` 选 **3** → 开始键①（解锁+起飞）→ 判稳后开始键②松手 ——"
             "同一趟里继场景 3 之后再来一次，**不用重启任何节点**",
             ("/mocap/pose_enu", "/vrpn/.*/pose", "/traj/reference", "/traj/progress",
              "/control/state", "/mavros/setpoint_raw/local", "/mavros/state"),
             120.0),
)

SCENARIO_BY_N: Dict[int, Scenario] = {s.n: s for s in SCENARIOS}

#: 两条一键的菜单标题（打印场景卡用）。⚠️ 它们**就是**从 `z0.ACTIONS` 抄下来的字面量
#: —— 抄一份就会各自漂移，所以 `test_rec.py` 有一条用例把它**逐字钉回动作表**
#: （改动作表标题而不同步这里，那条用例会红）。
CHAIN_MENU_TITLE: Dict[str, str] = {
    "up_manual": "菜单 **8= 一键启动（动捕+mavros，手动遥控飞）**",
    "up": "菜单 **9= 一键启动全部（选轨迹 → 按开始键）**",
}


def scenario_choices() -> str:
    """菜单里那一行「1=…  2=…」—— 场景表的**唯一出处**（提示语与打印共用一份）。"""
    return "  ".join(f"{s.n}={s.title.split('（')[0]}" for s in SCENARIOS)


# ─────────────────────────── 纯逻辑：命令与路径 ───────────────────────────


def record_argv(out_dir: Path, include: str = INCLUDE_REGEX,
                exclude: str = EXCLUDE_REGEX,
                compression: Tuple[str, str] = COMPRESSION) -> List[str]:
    """`ros2 bag record` 的 argv（**纯函数**：不碰文件系统、不起进程）。

    ★ 有意**不写 `-a`**：见文件头。`-o` 传的是**目录**，`rosbag2` 会在它下面建
    `metadata.yaml` + `<名字>_0.db3[.zstd]`。
    """
    argv = ["ros2", "bag", "record", "-o", str(out_dir), "-e", include]
    if exclude:
        argv += ["-x", exclude]
    argv += ["--compression-mode", compression[0], "--compression-format", compression[1]]
    return argv


def record_command(out_dir: Path, include: str = INCLUDE_REGEX,
                   exclude: str = EXCLUDE_REGEX,
                   compression: Tuple[str, str] = COMPRESSION) -> str:
    """上一条的**一整条 shell 命令**（逐项 `shlex.quote`）。

    ★ 必须 quote：默认白名单正则自带 `^` 与 `|`，不 quote 送进 `bash -c` 会被当成**管道**
    —— 那时的表现是"包起来了、里面一条话题都没有"，而且不报错。
    """
    return " ".join(shlex.quote(a) for a in record_argv(out_dir, include, exclude, compression))


def day_dir(now: Optional[_dt.datetime] = None, root: Optional[Path] = None) -> Path:
    """当天那一层：`bag/<YYYY-MM-DD>/`（**本地日期** —— 现场人对表用的是墙上那个钟）。

    ★ `root` 的默认值写成 `None` 再在这里取 `BAG_ROOT`（**不是**写成默认参数值）：
    默认参数在 `def` 那一刻就绑死了，用例把 `rec.BAG_ROOT` 指到 tmp 上时它不会跟着动 ——
    那意味着"测试里跑一下收口"有可能去动**真的**那份录制状态。
    """
    now = now or _dt.datetime.now()
    return (root or BAG_ROOT) / now.strftime("%Y-%m-%d")


def session_dir(scenario: Scenario, now: Optional[_dt.datetime] = None,
                root: Optional[Path] = None) -> Path:
    """一份包一个目录：`bag/<日期>/<slug>/`（同名再来一次会被 `rec_start` 拒掉）。"""
    return day_dir(now, root) / scenario.slug


def utc_stamp(now: Optional[_dt.datetime] = None) -> str:
    """UTC 的 ISO 串 —— ★ **与飞控日志对表用的就是它**：飞控的时钟由 mavros 的
    MAVLink timesync 拨过来，所以飞控自报的 `utc` 与这里的 UTC 可以直接比
    （第一手：2026-09-28 那次拉日志时，飞控报的已经是真日期而不是 1970）。

    ⚠️ 传进来的 `now` **是朴素的本地时间**（`_dt.datetime.now()` 就是这个形状）⟹ 必须
    `astimezone` 换算，**不能直接 `strftime` 贴个 `Z`**：那样得到的是"本地时间冒充 UTC"，
    在本地时区不是 UTC 的机器上这个字段就是错的 —— 而它整个存在的理由就是**能对上飞控**。
    （第一手：复核 2026-09-29 发现；本机恰好本地==UTC，所以一直没露出来。）
    """
    stamp = _dt.datetime.now(_dt.timezone.utc) if now is None else now.astimezone(
        _dt.timezone.utc)
    return stamp.strftime("%Y-%m-%dT%H:%M:%SZ")


def parse_bag_info(text: str) -> Dict:
    """`ros2 bag info` 的原文 → 结构化读数（**纯函数**）。

    Humble 的形状（**真输出**见 `run/tests/test_rec.py` 里那份样本）：

        Files:             x_0.db3.zstd
        Bag size:          5.0 KiB
        Storage id:        sqlite3
        Duration:          4.410736213s
        Start:             Sep 29 2026 02:42:23.538870652 (1790649743.538870652)
        End:               …
        Messages:          175
        Topic information: Topic: /probe_a/one | Type: std_msgs/msg/String | Count: 85 | Serialization Format: cdr

    ⚠️ **认不出的行不猜**：缺哪个字段就留 `None`/空，由 `render_record_table`
    把"没读到"如实写出来（`control` 里 `认不出的消息不猜` 是同一条纪律）。
    """
    out: Dict = {"files": [], "size_text": "", "duration_s": None,
                 "messages": None, "topics": []}
    for line in text.splitlines():
        line = line.strip()
        # ⚠️ 用 `search` 且**带上前缀 `Topic:`**：那一行真正的开头是
        # `Topic information: Topic: /chatter | Type: … | Count: 2 | …`，
        # 写成 `^Topic:` 会一位都匹配不上，于是"每条话题都读不到"而**不报错**
        # （第一手：2026-09-29 拿真 `ros2 bag info` 输出回代时撞到）。
        m = re.search(r"Topic:\s+(\S+)\s+\|\s+Type:\s+(\S+)\s+\|\s+Count:\s+(\d+)", line)
        if m:
            out["topics"].append({"topic": m.group(1), "type": m.group(2),
                                  "count": int(m.group(3))})
            continue
        if line.startswith("Files:"):
            out["files"] += line.split(":", 1)[1].split()
        elif line.startswith("Bag size:"):
            out["size_text"] = line.split(":", 1)[1].strip()
        elif line.startswith("Duration:"):
            m = re.match(r"^Duration:\s+([\d.]+)s", line)
            out["duration_s"] = float(m.group(1)) if m else None
        elif line.startswith("Messages:"):
            m = re.search(r"(\d+)", line.split(":", 1)[1])
            out["messages"] = int(m.group(1)) if m else None
    return out


def missing_expected(topics: Sequence[Dict], scenario: Scenario) -> List[str]:
    """`Scenario.expect` 里**一帧都没到**的那些（纯函数）。

    判据是"有没有**这条话题**"，不是"帧数够不够" —— 频率列由 `render_record_table`
    按实测时长算出来给人看。
    """
    names = [t["topic"] for t in topics]
    return [pat for pat in scenario.expect if not any(re.search(pat, n) for n in names)]


def rate_hz(count: int, duration_s: Optional[float]) -> Optional[float]:
    return (count / duration_s) if duration_s else None


def render_scenario_card(scenario: Scenario, out_dir: Path,
                         log_path: Optional[Path] = None) -> str:
    """起录之前摊开的那张卡（**人要在按下去之前看见它**）。"""
    lines = [
        f"场景 {scenario.n}：{scenario.title}",
        f"  起链：{CHAIN_MENU_TITLE.get(scenario.chain, scenario.chain)}",
        f"  你要做的：{scenario.do}",
        f"  录到 {scenario.expect_s:.0f} s **自动收口**（想提前停就 Ctrl-C —— 同一条收口路）",
        f"  落盘：{_rel(out_dir)}",
    ]
    if log_path is not None:
        lines.append(f"  日志：{_rel(log_path)}（万一没录上，先看它）")
    return "\n".join(lines)


def render_record_table(session: Dict) -> str:
    """收口之后的核对表（纯函数，吃 `session.json` 那一份 dict）。

    ★ 这一屏就是"包到底录到了什么"的**唯一证据**：每条话题的帧数与实测频率 +
    **预期里缺了什么**。缺一条就 `✗`，并且**不写"基本成功"**。
    """
    lines: List[str] = []
    lines.append(f"场景 {session.get('scenario')}：{session.get('title')}")
    lines.append(f"  目录 {session.get('dir')}")
    dur = session.get("duration_s")
    # ★ 判"读没读到"用 `is None`，**不许用真值**：真值会把一个**真的 `0.0 s`**
    #   （收口正常、但消息时间跨度为零）印成"没读到" —— 那是把"量到了 0"说成"没量到"，
    #   正好犯本仓为"静默为空"交过三次学费的那个毛病。
    lines.append("  时长 **没读到**" if dur is None else f"  时长 {dur:.1f} s")
    lines.append(f"  大小 {session.get('size_text') or '**没读到**'}"
                 f"   消息 {session.get('messages') if session.get('messages') is not None else '**没读到**'}")
    lines.append(f"  报文段 {', '.join(session.get('files') or []) or '**没读到**'}")
    lines.append(f"  sha256 {session.get('sha256') or '**没读到**'}")
    if session.get("log"):
        lines.append(f"  录制的输出：{session['log']}")
    lines.append(f"  收口 {session.get('stop_signal') or '?'}"
                 + ("" if session.get("closed_cleanly") else
                    "  ← ⚠️ 不是干净收口：`metadata.yaml` 是 `ros2 bag reindex` 补的"))

    topics = session.get("topics") or []
    lines.append(f"  话题（{len(topics)} 条）：")
    if not topics:
        lines.append("    **一条都没有** —— 先看包目录**旁边**那份 `<场景>.log`"
                     "（`ros2 bag record` 自己的输出）")
    for t in sorted(topics, key=lambda x: -x["count"]):
        rate = rate_hz(t["count"], dur)
        lines.append(f"    {f'{rate:7.1f} Hz' if rate else '     ?  Hz'}  "
                     f"{t['count']:>9,} 帧  {t['topic']}")

    scenario = SCENARIO_BY_N.get(session.get("scenario"))
    if scenario is not None:
        miss = session.get("missing_expected") or []
        lines.append("  预期话题：")
        for pat in scenario.expect:
            suffix = "   ← **一帧都没有，这份包不完整**" if pat in miss else ""
            lines.append(f"    {'✗' if pat in miss else '✓'} {pat}{suffix}")
    return "\n".join(lines)


def _rel(path: Path) -> str:
    try:
        return str(Path(path).relative_to(REPO))
    except ValueError:
        return str(path)


# ─────────────────────────── 状态文件（现场那一份） ───────────────────────────


def write_state(state: Dict, path: Optional[Path] = None) -> None:
    path = path or STATE_FILE                  # ★ 同 `day_dir`：默认值在调用时取，不在 def 时绑
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(state, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


def read_state(path: Optional[Path] = None) -> Optional[Dict]:
    """读状态文件；**读不出/坏了都返回 None**，由调用方走"按名字扫"那条兜底路。"""
    try:
        return json.loads((path or STATE_FILE).read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return None


def process_alive(pid: int) -> bool:
    """活着吗 —— ★ **僵尸不算活着**（`os.kill(pid, 0)` 对僵尸是成功的；
    与 `z0._pid_alive` 同一条纪律，第一手理由见 `run/README.md` 的「清除后台节点」）。"""
    try:
        with open(f"/proc/{pid}/stat", "rb") as fh:
            fields = fh.read().rsplit(b")", 1)[1].split()
        return fields[0] not in (b"Z", b"X")
    except (OSError, IndexError):
        return False


def find_recorders() -> List[Tuple[int, str]]:
    """按名字扫录制进程 `[(pid, cmdline)]` —— 状态文件丢了时的**兜底**。

    ★ **只认录制进程本身**：`pgrep -f` 会顺手扫到两条不是它的东西 —— ① 包着它的
    `bash -c 'source … && ros2 bag record …'`（真机上 bash 会把最后一条 `exec` 掉、
    通常根本没这个壳，但别的起法会有）；② **正在跑这条 `pgrep` 的那个 shell**，
    因为它的命令行里就写着 `ros2 bag record`（第一手：在工具的 eval shell 里实测到，
    它把自己列了出来）。两条都按"命令行以 `bash` / `/bin/bash` 开头"剔掉。
    """
    try:
        out = subprocess.run(["pgrep", "-af", "ros2 bag record"],
                             capture_output=True, text=True, timeout=10).stdout
    except (OSError, subprocess.SubprocessError):
        return []
    hits: List[Tuple[int, str]] = []
    for line in out.splitlines():
        pid_text, _, cmdline = line.partition(" ")
        if not pid_text.isdigit() or not cmdline:
            continue
        if cmdline.split(" ", 1)[0].rsplit("/", 1)[-1] == "bash":
            continue
        hits.append((int(pid_text), cmdline))
    return hits


# ─────────────────────────── 起录 ───────────────────────────


def git_head() -> Tuple[str, int]:
    """`(短 hash, 未提交改动数)` —— 包要能与代码对上（与 `z0.collect_env` 同口径）。"""
    head = subprocess.run(["git", "-C", str(REPO), "rev-parse", "--short", "HEAD"],
                          capture_output=True, text=True).stdout.strip()
    dirty = subprocess.run(["git", "-C", str(REPO), "status", "--porcelain"],
                           capture_output=True, text=True).stdout.strip().splitlines()
    return head or "(git 读不到)", len(dirty)


def start(n: int, now: Optional[_dt.datetime] = None) -> int:
    """菜单 **11= 开始录制（选场景 1–4）** 的**一整条前台流程**：
    查 ⟹ 印卡 ⟹ 起录 ⟹ 倒计时 ⟹ 收口 ⟹ 核对。

    ★ **前台**（操作者 2026-09-29 拍定）：屏幕上就是那个倒计时，录满 `expect_s`
    **自己收口**。旧版把录制丢到后台、只印一个 pid，人从屏幕上根本看不出
    "录到哪了、还差多久"。想提前停按 **Ctrl-C**（见 `_run_countdown`）。

    返回码：`0` = 收好口且录到了东西；`1` = 有东西不对（包不干净 / 一条消息都没有 /
    录制进程自己退了 / 已经在录 / 目录已存在）；`2` = 场景号不认识。
    """
    import z0                                  # ⟵ 函数级 import：z0 在模块级 import 本模块

    scenario = SCENARIO_BY_N.get(n)
    if scenario is None:
        print(f"没有场景 {n}（只有 {', '.join(str(s.n) for s in SCENARIOS)}）")
        return 2

    live = find_recorders()
    if live:
        print("已经在录了 —— 一次只录一份。正在录的那条在它自己的终端里跑，Ctrl-C 停它：")
        for pid, cmd in live:
            print(f"  {pid}  {cmd}")
        return 1

    now = now or _dt.datetime.now()
    out_dir = session_dir(scenario, now)
    # ⚠️ **`rosbag2` 拒绝往已经存在的目录里录**（第一手 2026-09-29，离线排练时撞到：
    #   `[ERROR] [ros2bag]: Output folder '…' already exists.` 而它**不覆盖、也不接着录**）。
    #   所以：① 这个目录**不能由我们预先建**（日志因此挪到旁边去，见下）；
    #   ② 它存在就一定是"上一份还在"或"上一次没收干净"，两种都**只报不动**。
    if out_dir.exists():
        print(f"{_rel(out_dir)} **已经存在** —— `rosbag2` 拒绝往已有的目录里录。")
        if (out_dir / "metadata.yaml").exists():
            print("  里面有一份**收好口**的包 ⟹ 不覆盖已入账的包；要重录先把那份挪走。")
        else:
            print("  里面**没有** `metadata.yaml` ⟹ 上一次没收干净（或被 SIGKILL 掉了）。"
                  "先看一眼它的日志，确认没用了再手工 `mv` 走 —— **本脚本不替你删包**。")
        return 1
    day = day_dir(now)
    day.mkdir(parents=True, exist_ok=True)
    # ★ 日志落在**包目录旁边**（`bag/<日期>/<场景>.log`）：它不能放进包里 ——
    #   一旦预先建了那个目录，`rosbag2` 就一句 "already exists" 直接退出（上面那条）。
    log_path = day / f"{scenario.slug}.log"

    # ★ 卡片印在**起之前**：那几句"你要做的"是给人**照着做**的，
    #   印在起之后等于"都录起来了，他才开始读"—— 而那几秒正是要紧的那几秒。
    print(render_scenario_card(scenario, out_dir, log_path))

    # ★ 起链命令一律带 source 前缀（与 `z0.sourced_command` **同一出处**）。
    # ⚠️ 这里**不调 `z0.ensure_sourced()`**：要 source 的是**子进程**（`bash -c` 里那一串），
    #   父进程的环境一点都用不上 —— 而它是"把本进程 exec 掉重开"，在这里调只会凭空重入一次。
    argv = record_argv(out_dir)
    cmd = record_command(out_dir)
    with log_path.open("w", encoding="utf-8") as fh:
        proc = subprocess.Popen(["bash", "-c", z0.sourced_command(cmd)],
                                stdout=fh, stderr=subprocess.STDOUT, start_new_session=True)

    head, dirty = git_head()
    state = {
        "scenario": scenario.n, "slug": scenario.slug, "title": scenario.title,
        "chain": scenario.chain, "dir": _rel(out_dir),
        "started_utc": utc_stamp(now), "started_local": now.strftime("%Y-%m-%d %H:%M:%S"),
        "git_head": head, "git_dirty": dirty,
        "ros_domain_id": os.environ.get("ROS_DOMAIN_ID"),
        "include_regex": INCLUDE_REGEX, "exclude_regex": EXCLUDE_REGEX,
        "compression": "/".join(COMPRESSION),
        "argv": argv, "pid": proc.pid, "log": _rel(log_path),
    }
    write_state(state)

    print(f"  已起：pid {proc.pid}（`rec_status` 能从别的终端看它录到哪了）")
    print("  ⚠️ 起录后约 1 s 才开始订阅话题（`rosbag2` 要先发现一次）"
          "—— 要紧的动作稍等一下再开始")
    outcome = _run_countdown(scenario, proc.pid, log_path)
    # ⚠️ **不传 `now`**：收口时刻由 `finalize` 自己取。`start` 手里这个是**起录**时刻，
    #   传下去会让 `stopped_utc == started_utc`（见 `finalize` 的 ★）。
    session = finalize(out_dir, state=state, outcome=outcome)
    _reap(proc)
    return 0 if session.get("closed_cleanly") and (session.get("messages") or 0) else 1


# ─────────────────────────── 倒计时（前台那一段） ───────────────────────────


def _mmss(seconds: float) -> str:
    seconds = max(0, int(seconds))
    return f"{seconds // 60:02d}:{seconds % 60:02d}"


def render_progress(scenario: Scenario, elapsed_s: float, *, tty: bool = True,
                    width: int = 24) -> str:
    """倒计时的**那一行**（纯函数）。`tty=False` 给的是**不带 `\r` 的整行**版本。

    ★ 两种形状是有理由的：交互终端上原地刷新（一条线慢慢长满），
      重定向进日志的场合每 10 s 一整行 —— 往文件里灌 `\r` 只会得到一坨连成一行的东西。
    """
    total = float(scenario.expect_s) or 1.0
    elapsed = min(max(float(elapsed_s), 0.0), total)
    left = total - elapsed
    clock = f"[{_mmss(elapsed)} / {_mmss(total)}]"
    if not tty:
        return f"  {clock} 剩 {left:.0f} s"
    filled = int(round(width * elapsed / total))
    return (f"  场景 {scenario.n}  {clock}  剩 {left:4.0f} s  "
            f"[{'█' * filled}{'░' * (width - filled)}]")


def watch(scenario: Scenario, *, alive: Callable[[], bool], clock=None, sleep=None,
          stream=None, tty: Optional[bool] = None, started: Optional[float] = None,
          every_s: float = PROGRESS_EVERY_S) -> str:
    """把倒计时摆在眼前，直到**到点**或**录制进程自己退了**。返回原因（纯逻辑，可离线测）。

    返 `"timeup"` / `"gone"` —— 收口动作（发信号、核对、写清单）在 `finalize` 里，
    这里**只负责显示与判定**，所以拿假 `clock`/`sleep` 就能把 120 s 一秒跑完。

    ★ `alive` 是**回调**而不是 pid：进程死没死的判据只有一处（`process_alive`），
      这里不重复实现一遍；用例也就没必要去起真进程。
    """
    clock = clock or time.monotonic
    sleep = sleep or time.sleep
    stream = stream if stream is not None else sys.stdout
    if tty is None:
        try:
            tty = bool(stream.isatty())
        except (AttributeError, ValueError):
            tty = False                       # 不是文件对象（用例里的假流）一律按非交互走
    t0 = clock() if started is None else started
    total = float(scenario.expect_s)
    shown_at: Optional[float] = None
    while True:
        elapsed = clock() - t0
        if tty:
            stream.write("\r" + render_progress(scenario, elapsed, tty=True))
            stream.flush()
        elif shown_at is None or elapsed - shown_at >= every_s:
            stream.write(render_progress(scenario, elapsed, tty=False) + "\n")
            stream.flush()
            shown_at = elapsed
        if elapsed >= total:
            if tty:
                stream.write("\n")
                stream.flush()
            return "timeup"
        if not alive():
            if tty:
                stream.write("\n")
                stream.flush()
            return "gone"
        sleep(TICK_S)


def _run_countdown(scenario: Scenario, pid: int, log_path: Path) -> str:
    """把倒计时跑完并把**为什么收口**说清楚；返回 `watch` 那个原因。

    ★ **Ctrl-C 在这里被接住并转交**：录制进程自成一个会话（`start_new_session=True`）
      ⟹ 终端发的 SIGINT **传不到它**。我们不接住的话，按一下 Ctrl-C 只会把本进程打死、
      留下一份**没有任何人收口**的包（它还在后台写着）。接住、转手发给它，
      "到点"与"人按"两条来路就汇到同一条收口路上，产物上没有区别。
    """
    print(f"  ▶ 录满 {scenario.expect_s:.0f} s **自动收口**；想提前停按 **Ctrl-C**"
          f"（同一条收口路）")
    try:
        outcome = watch(scenario, alive=lambda: process_alive(pid))
    except KeyboardInterrupt:
        outcome = "ctrl-c"
    print()                     # ★ 倒计时是原地刷新的那一行，收口这几行得另起一行
    if outcome == "timeup":
        print(f"  ⏱ 录满 {scenario.expect_s:.0f} s —— 自动收口。")
    elif outcome == "ctrl-c":
        print("  ⏹ 你按了 Ctrl-C —— 收口。")
    else:
        print(f"  ⚠️ 录制进程**自己退了**（没到 {scenario.expect_s:.0f} s）"
              f"—— 先看它的输出：{_rel(log_path)}")
    return outcome


# ─────────────────────────── 收口 ───────────────────────────


#: 收口那一刻发的是什么信号 —— 写进 `session.json` 的 `stop_signal`，
#: 现场回查时"这一份是被我们收的，还是它自己没的"要一眼看得出。
STOP_REASON: Dict[str, str] = {
    "timeup": "SIGINT（录满到点，自动收口）",
    "ctrl-c": "SIGINT（操作者按的 Ctrl-C）",
    "gone": "（录制进程自己退的 —— 不是我们收的）",
}


@contextlib.contextmanager
def _sigint_ignored():
    """收口这一小段里**忽略 Ctrl-C**。

    ★ 不是洁癖：告诉操作者"想停就按 Ctrl-C"之后，他多半会**连按两下**
    （第一下之后屏幕上有好几秒在"收口中（写缓存 → 压 zstd）"，看起来像没反应）。
    第二下若把我们打断，`ros2 bag info` 与 `session.json` 就都不会跑了 ——
    而 `rosbag2` 那边其实正在好好收尾。代价（丢掉那几秒的可见性）远小于收益。
    ⚠️ 只包住"等它退出"这一段；核对与写清单不包，那时 Ctrl-C 想打断就打断。
    """
    try:
        old = signal.signal(signal.SIGINT, signal.SIG_IGN)
    except (ValueError, OSError):
        yield                              # 非主线程之类装不上：不装，照跑
        return
    try:
        yield
    finally:
        signal.signal(signal.SIGINT, old)


def finalize(out_dir: Path, *, state: Optional[Dict], outcome: str,
             now: Optional[_dt.datetime] = None) -> Dict:
    """收口：SIGINT ⟹ 等它压完包 ⟹ `ros2 bag info` 核对 ⟹ 写 `session.json` + 清单。

    ★ 顺序**先 SIGINT 再等**：`rosbag2` 的 SIGINT 处理就是"把缓存写完 + 压包 + 关库"，
    与操作者自己按 Ctrl-C 完全同一条路。等不到才升级（SIGTERM → SIGKILL，见 `_stop_pid`）；
    被 SIGKILL 掉的包**不是废的** —— `summarize` 会跑 `ros2 bag reindex` 把
    `metadata.yaml` 补回来，并在核对表里**如实标出"不是干净收口"**。

    ★ `now` **由收口这一刻自己取**，绝不从 `start()` 那里传进来。（第一手：复核 2026-09-29
    揪出 `start` 把自己那份**起录**时刻一路传到这里 ⟹ `stopped_utc` 与 `started_utc`
    **逐字相同**，而 `bag/README.md` 正是拿这两个时间戳去和飞控日志对表 —— 照它读会得出
    "这份包只有 0 秒长"。参数留着只为用例能钉住收口时刻。）
    """
    now = now or _dt.datetime.now()
    pid = int((state or {}).get("pid") or 0) or None
    if pid is not None and process_alive(pid):
        print("  收口中（写缓存 → 压 zstd）… ", end="", flush=True)
        with _sigint_ignored():
            used = _stop_pid(pid)
        print(f"好（{used}）")
        signal_text = STOP_REASON.get(outcome, "SIGINT")
        if used != "SIGINT":               # 升级过就得如实说，别让表上只留一句"SIGINT"
            signal_text += f"，但它没理 SIGINT —— 升级到 {used}"
    else:
        signal_text = STOP_REASON["gone"]
    session = summarize(out_dir, state=state, now=now, stop_signal=signal_text)
    STATE_FILE.unlink(missing_ok=True)
    print()
    print(render_record_table(session))
    if (session.get("messages") or 0) == 0:
        print(f"\n⚠️ **一条消息都没有** —— 这份包是空的：看 "
              f"{session.get('log') or '包目录旁边那份 <场景>.log'}"
              "（话题名对不对、起链起来没有）。别把它当「录过了」。")
    write_manifest()
    print(f"\n清单已更新：{_rel(BAG_ROOT / 'MANIFEST.md')}")
    return session


def _reap(proc) -> None:
    """把子进程收掉（不收就是个僵尸，一直挂到本进程退出 —— 菜单是循环的）。

    ★ 它**不影响判据**：`process_alive` 本来就不认僵尸。这里只是别留垃圾。
    """
    try:
        proc.wait(timeout=5)
    except (OSError, subprocess.SubprocessError, AttributeError):
        pass


def _stop_pid(pid: int) -> str:
    """SIGINT ⟹ （等不到就）SIGTERM ⟹ （再等不到就）SIGKILL。返回用掉的那一档。

    ★ `killpg` **只在能确认"这个组是我们自己起的"时才用**（`getpgid(pid) == pid`
    ⟸ `start_new_session=True` 的产物）；否则只发单个 pid —— 按名字扫出来的进程
    可能与别人终端的 shell 同组，对那个组放 `killpg` 会把人家终端一起收掉
    （同一条纪律与第一手理由见 `run/README.md` 的「清除后台节点」）。
    """
    for sig, name in ((signal.SIGINT, "SIGINT"), (signal.SIGTERM, "SIGTERM"),
                      (signal.SIGKILL, "SIGKILL")):
        try:
            if os.getpgid(pid) == pid:
                os.killpg(pid, sig)
            else:
                os.kill(pid, sig)
        except ProcessLookupError:
            return name
        if _wait_gone(pid, STOP_GRACE_S if sig is signal.SIGINT else 5.0):
            return name
    return "SIGKILL（没等它退干净）"


def _wait_gone(pid: int, limit_s: float) -> bool:
    deadline = time.monotonic() + limit_s
    while time.monotonic() < deadline:
        if not process_alive(pid):
            return True
        time.sleep(0.2)
    return False


def summarize(out_dir: Path, *, state: Optional[Dict], now: Optional[_dt.datetime] = None,
              stop_signal: Optional[str] = None) -> Dict:
    """把一个收好口的包读成一份 `session.json`（**唯一的写入口**）。"""
    metadata = out_dir / "metadata.yaml"
    clean = metadata.exists()
    if not clean:                      # 被 SIGKILL 掉 / 没收干净：先把索引补回来再读
        subprocess.run(["ros2", "bag", "reindex", str(out_dir)],
                       capture_output=True, text=True, timeout=300)
        clean = metadata.exists()
    info_text = ""
    try:
        info_text = subprocess.run(["ros2", "bag", "info", str(out_dir)],
                                   capture_output=True, text=True, timeout=120).stdout
    except (OSError, subprocess.SubprocessError):
        info_text = ""
    info = parse_bag_info(info_text)
    scenario = SCENARIO_BY_N.get((state or {}).get("scenario"))
    session = dict(state or {})
    session.update({
        "dir": _rel(out_dir),
        "stopped_utc": utc_stamp(now),
        "stop_signal": stop_signal,
        "closed_cleanly": clean,
        "duration_s": info["duration_s"],
        "size_text": info["size_text"],
        "messages": info["messages"],
        "files": info["files"],
        "topics": info["topics"],
        "missing_expected": missing_expected(info["topics"], scenario) if scenario else [],
        "sha256": sha256_of_bag(out_dir),
        "info_text": info_text.strip(),
    })
    (out_dir / "session.json").write_text(
        json.dumps(session, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    return session


def sha256_of_bag(out_dir: Path) -> str:
    """包内容的 sha256（`metadata.yaml` **不算**：它带时间戳字段，逐次都会变）。

    ★ 与 `tools/tau-analysis/evidence/` 那套同一纪律：**数要能被别人回验** ——
    拿同一份目录重算一遍即可对齐。

    ⚠️ 一个报文段都没有时返回**空串**，不返回 `sha256("")` 那个 `e3b0c442…` ——
    那玩意儿**长得就像一个正经哈希**，摆在表里会被当成"这份包算过了"。
    """
    files = sorted(out_dir.glob("*.db3*"))
    if not files:
        return ""
    digest = hashlib.sha256()
    for path in files:
        digest.update(_sha256_file(path).encode())
    return digest.hexdigest()


def _sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as fh:
        for chunk in iter(lambda: fh.read(1 << 20), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _abs(rel: Optional[str]) -> Optional[Path]:
    if not rel:
        return None
    path = Path(rel)
    return path if path.is_absolute() else REPO / path


def status() -> int:
    """现在在录什么（只读）。"""
    state = read_state()
    live = find_recorders()
    if not state and not live:
        print("没在录。（`rec_start N` 起录 —— N 见菜单 **11= 开始录制（选场景 1–4）**）")
        return 0
    if state:
        n = state.get("scenario")
        scenario = SCENARIO_BY_N.get(n)
        pid = state.get("pid")
        print(f"场景 {n}：{state.get('title')}")
        print(f"  目录 {state.get('dir')}")
        print(f"  起点 {state.get('started_local')}（UTC {state.get('started_utc')}）"
              f"   已录 {_elapsed_s(state):.0f} s"
              + (f"   （录满 {scenario.expect_s:.0f} s 自动收口）" if scenario else ""))
        print(f"  pid {pid}：{'**在跑**' if process_alive(int(pid or -1)) else '**已经退出了**'}"
              f"   落盘 {_dir_size(_abs(state.get('dir')))}")
        if state.get("log"):
            print(f"  它的输出：{state['log']}（「已经退出了」时先看它）")
    for pid, cmd in live:
        print(f"  按名字扫到录制进程 {pid}  {cmd}")
    return 0


def _elapsed_s(state: Dict) -> float:
    try:
        started = _dt.datetime.strptime(state["started_local"], "%Y-%m-%d %H:%M:%S")
    except (KeyError, ValueError, TypeError):
        return 0.0
    return (_dt.datetime.now() - started).total_seconds()


def _dir_size(path: Optional[Path]) -> str:
    if path is None or not path.exists():
        return "**目录不在**"
    total = sum(p.stat().st_size for p in path.rglob("*") if p.is_file())
    return f"{total / 1e6:.1f} MB"
# ─────────────────────────── 清单（入库的那一份） ───────────────────────────


def render_manifest(root: Optional[Path] = None) -> str:
    """把 `bag/**/session.json` 与 `bag/**/fcu/*.ulg` 收成一张表（纯函数，可离线测）。

    ★ 这张表是**给人回查的入口**：哪一份包、多长、多大、sha256、缺了什么。
    """
    root = root or BAG_ROOT
    sessions: List[Dict] = []
    for path in sorted(root.glob("*/*/session.json")):
        try:
            sessions.append(json.loads(path.read_text(encoding="utf-8")))
        except (OSError, ValueError):
            continue
    # ★ **刻意不写"生成于 <now>"**：这份表是入库的，而验收里有一条是"`rec_manifest`
    #   重算得一致"——带时间戳就每算一次脏一次，`git diff` 永远对不上。
    #   每份包自己的起点时间在表里（`started_utc`），回查不缺东西。
    lines = ["# `bag/` 清单（自动生成 —— 别手改；`run/z0.py rec_manifest` 重算）", "",
             "★ 本表是**确定性输出**（不含生成时间）：重算一次必须逐字相同，好拿 `git diff` 当判据。", ""]
    if not sessions:
        lines.append("（还没有收好口的包。）")
    else:
        lines += ["| 场景 | 目录 | 起点（UTC） | 时长 | 大小 | 消息 | 话题 | sha256（包内容） |",
                  "|---|---|---|---|---|---|---|---|"]
        for s in sorted(sessions, key=lambda x: str(x.get("started_utc"))):
            dur = "?" if s.get("duration_s") is None else f"{s['duration_s']:.1f} s"  # 同上：不许真值
            lines.append(f"| {s.get('scenario')} {s.get('title')} | `{s.get('dir')}` | "
                         f"{s.get('started_utc') or '?'} | {dur} | {s.get('size_text') or '?'} | "
                         f"{s.get('messages')} | {len(s.get('topics') or [])} | "
                         f"`{s.get('sha256') or '?'}` |")
        lines += ["", "## 缺了什么（预期里一帧都没到的）", ""]
        gaps = [(s.get("dir"), m) for s in sessions for m in (s.get("missing_expected") or [])]
        if gaps:
            lines += [f"- `{d}`：**{m}**" for d, m in gaps]
        else:
            lines.append("（一份都不缺。）")

    fcu = sorted(root.glob("*/fcu/*.ulg"))
    lines += ["", "## 飞控日志（`.ulg`）", ""]
    if fcu:
        lines += ["| 文件 | 大小 | sha256 |", "|---|---|---|"]
        for path in fcu:
            lines.append(f"| `{_rel(path)}` | {path.stat().st_size / 1e6:.2f} MB |"
                         f" `{_sha256_file(path)}` |")
    else:
        lines.append("（还没有 `.ulg` —— 拔 SD 卡、手工拷进 `bag/<日期>/fcu/`，这里就会列出来）")
    lines.append("")
    return "\n".join(lines)


def write_manifest(root: Optional[Path] = None) -> Path:
    root = root or BAG_ROOT
    root.mkdir(parents=True, exist_ok=True)
    path = root / "MANIFEST.md"
    path.write_text(render_manifest(root), encoding="utf-8")
    return path


# ─────────────────────────── 菜单入口（`run/z0.py` 那两条） ───────────────────────────
#
# ★ 这三个函数**不带 `ensure_sourced()`** —— 录制层不碰 ROS：
#   `ros2 bag record` 跑在 `z0.sourced_command` 包出来的**子进程**里（`start` 里那一句），
#   本进程只写状态文件、发信号、读 `ros2 bag info` 的输出。（`ros2 bag info` 也不需要
#   source —— 它是 CLI 自带子命令；真缺环境它自己会报错，那时屏幕上是它的原话。）


def _ask(prompt: str, default: str) -> str:
    """问一句（非交互时给默认值）—— 与 `z0.ask_confirm` 同一条纪律：**不抛异常**。"""
    try:
        return input(prompt).strip() or default
    except EOFError:
        return default


def cmd_start(argv: Sequence[str] = ()) -> int:
    """菜单 **11= 开始录制（选场景 1–4）**；`rec_start 3` 也行（不给就问一句）。

    ⚠️ 这条**阻塞整个终端**直到收口（前台倒计时，见 `start`）—— 这是有意的：
    录制期间操作者要做的事都在飞机上，屏幕上留一个"录到哪了"比留一个菜单有用。
    """
    if argv and argv[0].isdigit():
        return start(int(argv[0]))
    # ★ 选之前只给一行短名单；**完整那张卡（起链 + 你要做的）在 `start` 里、起录之前印** ——
    #   那几句是"人要照着做"的，必须出现在**他开始动手之前**，而不是录完之后。
    print(f"录哪个场景？{scenario_choices()}")
    reply = _ask("场景号 [1]：", "1")
    if not reply.isdigit():
        print(f"`{reply}` 不是一个场景号。")
        return 2
    return start(int(reply))


def cmd_status(argv: Sequence[str] = ()) -> int:
    """现在在录什么（**不进菜单**）—— 录制在那条前台命令占着的终端里跑，
    所以这条是给**另一个终端**看的（`run/z0.py rec_status`）。"""
    return status()


def cmd_manifest(argv: Sequence[str] = ()) -> int:
    """重算 `bag/MANIFEST.md`（**不进菜单**：每次收口都会顺手写一次）。"""
    print(f"清单重算完毕：{_rel(write_manifest())}")
    return 0


#: 本模块提供的**进程内动作**名（与 `z0.ACTIONS` 的 key 逐字对齐）—— `z0.in_process_command`
#: 就是照这几个名字取的；`z0.TAKES_ARGS` 里那一条吃位置参数。
IN_PROCESS_KEYS: Tuple[str, ...] = ("rec_start", "rec_status", "rec_manifest")


def _sourced(cmd: str) -> str:
    """带 source 前缀的那条命令 —— 走 `z0.sourced_command`（**同一出处**，别另抄一份）。"""
    import z0
    return z0.sourced_command(cmd)


def dry_run_note(key: str, argv: Sequence[str] = ()) -> str:
    """`--dry-run` 对录制这两条说什么 —— ★ 判据只有一条：**一个进程都不起、一个信号都不发、
    一个字节都不写**（与 `z0.dispatch` 里那条"`--dry-run` 对所有进程内动作都是不执行"配套）。"""
    if key == "rec_start":
        lines = ["（`--dry-run`：**一个进程都不起**，状态文件也不写。四条场景与它们会跑的命令："]
        for scenario in SCENARIOS:
            lines.append(f"  场景 {scenario.n}：{scenario.title}")
            lines.append(f"    $ {_sourced(record_command(session_dir(scenario)))}")
            lines.append(f"    录到 {scenario.expect_s:.0f} s 自动收口；"
                         f"预期话题：{'、'.join(scenario.expect)}")
        lines.append("）")
        return "\n".join(lines)
    if key == "rec_manifest":
        return (f"（`--dry-run`：清单**不写**。真跑会读 {_rel(BAG_ROOT)} 下的 `session.json` 与 "
                f"`fcu/*.ulg`，重算 `{_rel(BAG_ROOT / 'MANIFEST.md')}`。）")
    return f"（`{key}` 是只读动作：本进程里直接跑，没有命令可打印；`--dry-run` 不执行）"

