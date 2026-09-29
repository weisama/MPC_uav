"""`run/rec.py`（录制层，ticket 19）的用例 —— **不需要 ROS、不需要飞机、不碰真飞机一根线**。

跑法：

    python3 -m pytest run/tests/test_rec.py -q

★ 这里测的是**判读与拼装**（解析 / 白名单 / 核对 / 归档 / 安全边界），不是"把常量再抄一遍"。
现场那条真数据路径（真起 `ros2 bag record` 录四个场景）在 ticket 19 的现场记录里，
理由与 `test_z0_menu.py` 那条一样：**能离线验的判据不留到现场**。

★★ **两条纪律写在最前面，后面每条用例都从它们派生**：

  ① **不许动真东西**：所有会起进程 / 发信号 / 写仓库的路径在用例里都被指到 `tmp_path` 或
     换掉（`rec.BAG_ROOT`、`rec.STATE_FILE`、`rec.find_recorders`、`rec.process_alive`）。
     这不是洁癖 —— 现场可能正录着，而收口的判据是"给那个 pid 发 SIGINT"，指错地方就会
     **掐掉真录制**。⟹ 这也是 `day_dir`/`read_state` 那几个默认值写成 `None`、
     在**调用时**才取 `BAG_ROOT` 的原因。
  ② **认不出的行不猜**：解析类用例里都有一条"给空输入 / 脏输入，别抛异常、别编数字"。
"""

import ast
import datetime
import io
import json
import os
import pathlib
import re
import shlex
import signal
import subprocess
import sys
import time
import types

import pytest

RUN_DIR = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(RUN_DIR))

import rec  # noqa: E402  （路径注入了才 import 得到）
import z0   # noqa: E402

#: 真实输出（2026-09-29 离线录的一份小包，域 42，没碰飞机）—— **逐字照抄**，
#: 解析用例就吃它。⚠️ 那次录到的关键一行是 `Topic information: Topic: /chatter | …`：
#: 行首是 `Topic information:` 而**不是** `Topic:`，写成 `^Topic:` 会一位都匹配不上。
BAG_INFO = """Files:             baginfo_0.db3
Bag size:          24.9 KiB
Storage id:        sqlite3
Duration:          1.000009935s
Start:             Sep 29 2026 02:49:54.737157214 (1790650194.737157214)
End:               Sep 29 2026 02:49:55.737167149 (1790650195.737167149)
Messages:          2
Topic information: Topic: /chatter | Type: std_msgs/msg/String | Count: 2 | Serialization Format: cdr
"""


# ─────────────────────────── 场景表 ───────────────────────────


def test_四个场景就是票面那四个():
    assert [s.n for s in rec.SCENARIOS] == [1, 2, 3, 4]
    assert len({s.slug for s in rec.SCENARIOS}) == 4, "落盘目录名不许重"
    assert set(rec.SCENARIO_BY_N) == {1, 2, 3, 4}
    # 场景 1/2 没有 control（手动遥控飞那条链），3/4 才是全链
    assert (rec.SCENARIO_BY_N[1].chain, rec.SCENARIO_BY_N[2].chain) == ("up_manual", "up_manual")
    assert (rec.SCENARIO_BY_N[3].chain, rec.SCENARIO_BY_N[4].chain) == ("up", "up")


def test_每个场景都写清楚要人做什么_预期话题也能当正则用():
    for scenario in rec.SCENARIOS:
        assert scenario.title and scenario.slug and scenario.do and scenario.expect
        assert scenario.expect_s > 0
        assert scenario.chain in rec.CHAIN_MENU_TITLE, f"场景 {scenario.n} 的链没有菜单出处"
        for pattern in scenario.expect:
            re.compile(pattern)                     # ★ 写错的正则要在这里炸，不是收口时


def test_四个场景的时长统一_120s_且它是自动收口的判据():
    """★ 操作者 2026-09-29 现场拍定：**四份统一 120 s，到点自动收口**
    （演变：原先 1/2 是 90、3 是 150、4 是 120 ⟹ 统一 90 ⟹ 统一 **120**）。
    钉在用例里，改一个就得四个一起改 —— 它现在是**收口的判据**（`watch` 拿它比时间），
    不再是"提示"：四个不一样 ⟹ 现场每个人心里的"录够了没有"都不一样。

    ⚠️ 实际时长以 `session.json` 的 `duration_s` 为准（秒级调度 ⟹ 差个零点几秒是正常的）。
    """
    assert {s.expect_s for s in rec.SCENARIOS} == {120.0}


def test_场景选择那一行与场景表同源():
    line = rec.scenario_choices()
    for scenario in rec.SCENARIOS:
        assert f"{scenario.n}={scenario.title.split('（')[0]}" in line


# ─────────────────────────── 录制的命令行 ───────────────────────────


def test_录制的命令里没有_a_只有白名单正则(tmp_path):
    """★ `-a` 会把 `/rosout`、`/parameter_events` 与**同域里别人起的节点**一起吸进来
    （本仓库有并行会话）⟹ 取白名单正则 + 事后核对（见 `rec.py` 文件头）。"""
    argv = rec.record_argv(tmp_path / "x")
    assert argv[:3] == ["ros2", "bag", "record"]
    assert "-a" not in argv and "--all" not in argv
    assert argv[argv.index("-e") + 1] == rec.INCLUDE_REGEX
    assert "--compression-mode" in argv and "zstd" in argv


def test_白名单正则必须被引号包住_否则_shell_会把它当管道(tmp_path):
    """★ 默认正则自带 `^` 与 `|`：不 quote 送进 `bash -c` ⟹ 表现是"包起来了、
    里面一条话题都没有"，而且**不报错**（这类静默失败本仓库交过两次学费）。"""
    command = rec.record_command(tmp_path / "x")
    assert shlex.quote(rec.INCLUDE_REGEX) in command
    assert rec.INCLUDE_REGEX in shlex.split(command)


def test_排除正则为空时不许出现_x(tmp_path):
    argv = rec.record_argv(tmp_path / "x", exclude="")
    assert "-x" not in argv
    assert "-x" in rec.record_argv(tmp_path / "x", exclude="^/mavros/imu/")


def test_四份场景落盘目录各不相同_且都在_bag_当天目录下(tmp_path):
    moment = datetime.datetime(2026, 9, 29, 10, 30)
    dirs = [rec.session_dir(s, moment, tmp_path) for s in rec.SCENARIOS]
    assert len(set(dirs)) == 4
    for scenario, path in zip(rec.SCENARIOS, dirs):
        assert path == tmp_path / "2026-09-29" / scenario.slug
        assert rec.record_argv(path)[4] == str(path)         # `-o` 就是那个目录


# ─────────────────────────── 解析 `ros2 bag info` ───────────────────────────


def test_解析真的_bag_info_输出():
    info = rec.parse_bag_info(BAG_INFO)
    assert info["files"] == ["baginfo_0.db3"]
    assert info["size_text"] == "24.9 KiB"
    assert info["duration_s"] == pytest.approx(1.000009935)
    assert info["messages"] == 2
    assert info["topics"] == [{"topic": "/chatter", "type": "std_msgs/msg/String", "count": 2}]


def test_解析多种话题行():
    """一个包里有好几条话题时逐条读出来（真输出里这几行是重复的同一形状）。"""
    text = BAG_INFO + (
        "Topic information: Topic: /mocap/pose_enu | Type: geometry_msgs/msg/PoseStamped"
        " | Count: 1800 | Serialization Format: cdr\n"
        "Topic information: Topic: /vrpn/Z0/pose | Type: geometry_msgs/msg/PoseStamped"
        " | Count: 3600 | Serialization Format: cdr\n")
    topics = rec.parse_bag_info(text)["topics"]
    assert [t["topic"] for t in topics] == ["/chatter", "/mocap/pose_enu", "/vrpn/Z0/pose"]
    assert topics[2]["count"] == 3600


def test_解析不出的输入不猜也不炸():
    """★ 空输入、脏输入都要返回"没读到"，**不许编数字**（`control` 的
    「认不出的消息不猜」是同一条纪律）。"""
    empty = rec.parse_bag_info("")
    assert empty["files"] == [] and empty["topics"] == []
    assert empty["duration_s"] is None and empty["messages"] is None
    noise = rec.parse_bag_info("随便一行\nDuration: 不是数字\nMessages: 也没有\n")
    assert noise["duration_s"] is None and noise["messages"] is None


def test_算频率_没读到时长就不给数字():
    assert rec.rate_hz(1800, 60.0) == pytest.approx(30.0)
    assert rec.rate_hz(0, 60.0) == 0.0
    assert rec.rate_hz(1800, None) is None      # ★ 宁可空着，也别拿"没读到"当 0


# ─────────────────────────── 收口核对 ───────────────────────────


def _session(**over):
    base = {
        "scenario": 3, "title": rec.SCENARIO_BY_N[3].title, "dir": "bag/2026-09-29/s3-traj1-hover",
        "duration_s": 150.0, "size_text": "12.3 MiB", "messages": 12345,
        "files": ["s3-traj1-hover_0.db3.zstd"], "sha256": "deadbeef",
        "stop_signal": "SIGINT", "closed_cleanly": True,
        "topics": [{"topic": "/mocap/pose_enu", "type": "geometry_msgs/msg/PoseStamped",
                    "count": 30000},
                   {"topic": "/traj/reference", "type": "traj/msg/Reference", "count": 7500}],
        "missing_expected": [],
    }
    base.update(over)
    return base


def test_核对表把每条话题的实测频率摊开():
    text = rec.render_record_table(_session())
    assert "场景 3" in text
    assert "150.0 s" in text and "12.3 MiB" in text
    assert "200.0 Hz" in text and "/mocap/pose_enu" in text
    assert "SIGINT" in text


def test_核对表按场景逐条报缺了什么():
    """★ 票面要的"包够不够用"就落在这一屏：预期里有、实测一帧都没到的**必须点名**。"""
    missing = ["/control/state", "/vrpn/.*/pose"]
    text = rec.render_record_table(_session(missing_expected=missing))
    for pattern in rec.SCENARIO_BY_N[3].expect:
        mark = "✗" if pattern in missing else "✓"
        assert f"{mark} {pattern}" in text
    assert "这份包不完整" in text
    # 一条不缺时**不许**出现那句话
    assert "这份包不完整" not in rec.render_record_table(_session())


def test_不是干净收口要明说():
    """被 SIGKILL / 没收干净时 `metadata.yaml` 是 `ros2 bag reindex` 补的 ⟹ 表上要看得见。"""
    text = rec.render_record_table(_session(closed_cleanly=False))
    assert "不是干净收口" in text


def test_缺的话题用正则匹_刚体名不算死():
    """刚体名 `Z0`/`z0` 都出现过 ⟹ 预期里只能写正则，判据也得按正则走。"""
    topics = [{"topic": "/vrpn/z0/pose", "type": "x", "count": 1}]
    assert rec.missing_expected(topics, rec.SCENARIO_BY_N[1]) != []      # 缺 mocap 那条
    assert "/vrpn/.*/pose" not in rec.missing_expected(topics, rec.SCENARIO_BY_N[1])
    assert rec.missing_expected([{"topic": "/vrpn/Z0/pose", "type": "x", "count": 1},
                                 {"topic": "/mocap/pose_enu", "type": "x", "count": 1},
                                 {"topic": "/mavros/vision_pose/pose", "type": "x", "count": 1},
                                 {"topic": "/mavros/local_position/pose", "type": "x", "count": 1},
                                 {"topic": "/mavros/state", "type": "x", "count": 1}],
                                rec.SCENARIO_BY_N[1]) == []


# ─────────────────────────── 状态文件 ───────────────────────────


def test_状态文件往返(tmp_path):
    path = tmp_path / "state.json"
    state = {"scenario": 2, "pid": 4321, "started_local": "2026-09-29 10:00:00"}
    rec.write_state(state, path)
    assert rec.read_state(path) == state


def test_状态文件坏了就当没有(tmp_path):
    path = tmp_path / "state.json"
    path.write_text("{这不是 json", encoding="utf-8")
    assert rec.read_state(path) is None
    assert rec.read_state(tmp_path / "不存在.json") is None


def test_僵尸进程不算活着():
    """★ 与 `z0._pid_alive` 同一条纪律：`os.kill(pid, 0)` 对**僵尸**是成功的 ⟹
    只看它就等于对着尸体等 45 s，最后还报一句假的"没收干净"。"""
    alive = subprocess.Popen(["sleep", "300"])
    dead = subprocess.Popen(["sleep", "300"])
    try:
        os.kill(dead.pid, signal.SIGKILL)
        deadline = time.time() + 5.0
        while time.time() < deadline and not _is_zombie(dead.pid):
            time.sleep(0.05)
        assert _is_zombie(dead.pid), "造不出僵尸态，这条用例本身没验到东西"
        assert rec.process_alive(alive.pid) is True
        assert rec.process_alive(dead.pid) is False
        assert rec.process_alive(2 ** 22 + 7) is False        # 不存在的 pid
    finally:
        alive.kill()
        dead.wait()
        alive.wait()


def _is_zombie(pid: int) -> bool:
    try:
        with open(f"/proc/{pid}/stat") as fh:
            return fh.read().rsplit(") ", 1)[-1].split(" ", 1)[0] == "Z"
    except OSError:
        return False


def test_按名字扫要剔掉_shell_壳(monkeypatch):
    """★ 第一手：`pgrep -af "ros2 bag record"` 会捞到两种**不是录制进程**的东西 ——
    ① 包着它的 `bash -c 'source … && ros2 bag record …'`（往它发信号不会传到录制进程）；
    ② **正在跑这条 pgrep 的那个 shell**，它的命令行里就写着 `ros2 bag record`。"""
    canned = ("9546 bash -c 'source /opt/ros/humble/setup.bash && ros2 bag record -o /tmp/x'\n"
              "9556 /usr/bin/python3 /opt/ros/humble/bin/ros2 bag record -o /tmp/x\n"
              "9999 /bin/bash -c ros2 bag record --help\n"
              "不是pid ros2 bag record -o /tmp/y\n")
    monkeypatch.setattr(rec.subprocess, "run",
                        lambda *a, **k: subprocess.CompletedProcess(a, 0, canned, ""))
    assert [pid for pid, _ in rec.find_recorders()] == [9556]


def test_起录时已经在录就先报出来(monkeypatch, capsys, tmp_path):
    monkeypatch.setattr(rec, "BAG_ROOT", tmp_path / "bag")
    monkeypatch.setattr(rec, "find_recorders", lambda: [(4321, "ros2 bag record -o /x")])
    assert rec.start(1) == 1
    out = capsys.readouterr().out
    assert "已经在录了" in out and "4321" in out
    assert not (tmp_path / "bag").exists(), "报了「已经在录」就**不许**再起第二个"


def test_不认识的场景号要报出来不炸(capsys):
    assert rec.start(99) == 2
    assert "没有场景 99" in capsys.readouterr().out


class _FakeProc:
    pid = 4321

    def wait(self, timeout=None):        # `_reap` 会调它
        return 0


#: 收口之后那份 `session.json` 的替身（**收口本身**另有用例，这里只求 `start` 能跑完）
_FAKE_SESSION = {
    "scenario": 1, "title": rec.SCENARIO_BY_N[1].title, "dir": "bag/2026-09-29/s1-static",
    "duration_s": 120.0, "size_text": "1.0 MiB", "files": ["s1-static_0.db3.zstd"],
    "sha256": "deadbeef", "topics": [], "missing_expected": [],
    "messages": 123, "closed_cleanly": True, "stop_signal": "SIGINT（测试）",
}


def _fake_start(monkeypatch, tmp_path, session=None):
    """把"起进程 + 倒计时 + 收口"换成假的：`start` 的其余部分
    （状态文件 / 日志落点 / 命令拼装 / 印卡）照跑。返回 `(Popen 调用, 交给收口的状态)`。

    ★ 这三处**必须**一起换：
      · `Popen` —— 真起 `ros2 bag record` 会去连真的域（而用例可能正在飞机旁边跑）；
      · `watch` —— 不换就要**真等 120 s**（它是按墙上时钟转的）；
      · `process_alive` / `_stop_pid` / `summarize` / `write_manifest` —— 假 pid 4321
        要么不存在、要么（更糟）**撞上某个真进程**，`_stop_pid` 会真发 SIGINT 给它。

    ⚠️ 状态文件**在收口那一刻就被删了**（那是设计：收完口就不该再有"在录"的状态）⟹
    想检查它的内容只能从收口那一侧截（`states` 里拿），**不能事后 `read_state()`**。
    ⚠️ 换的是 `rec.subprocess` 这**一个名字**（而不是 `subprocess.Popen` 本身）：
    `subprocess.run` 内部就是用 `Popen` 实现的，直接换后者会把 `git_head()` 一起打断
    （第一手：这条用例第一次跑就炸在 `git_head` 里）。
    """
    monkeypatch.setattr(rec, "BAG_ROOT", tmp_path / "bag")
    monkeypatch.setattr(rec, "STATE_FILE", tmp_path / "bag" / ".recorder.json")
    monkeypatch.setattr(rec, "find_recorders", lambda: [])
    monkeypatch.setattr(rec, "watch", lambda *a, **k: "timeup")
    monkeypatch.setattr(rec, "process_alive", lambda pid: False)
    monkeypatch.setattr(rec, "_stop_pid", lambda pid: pytest.fail("进程是假的，不该真有信号"))
    seen, states = [], []

    def fake_summarize(out_dir, **kw):
        states.append(kw.get("state"))
        return dict(session or _FAKE_SESSION)

    monkeypatch.setattr(rec, "summarize", fake_summarize)
    monkeypatch.setattr(rec, "write_manifest", lambda *a, **k: tmp_path / "bag" / "MANIFEST.md")
    shim = types.SimpleNamespace(
        Popen=lambda argv, **kw: (seen.append((argv, kw)), _FakeProc())[1],
        STDOUT=subprocess.STDOUT, run=subprocess.run)
    monkeypatch.setattr(rec, "subprocess", shim)
    return seen, states


def test_起录不预先建包目录_日志落在它旁边(tmp_path, monkeypatch, capsys):
    """★ 回归（2026-09-29 离线排练抓到的真事）：`rosbag2` **拒绝往已经存在的目录里录** ——
    `[ERROR] [rosbag2]: Output folder '…' already exists.`，**不覆盖也不接着录**。
    原先 `start` 为了放 `record.log` 先 `mkdir` 了包目录 ⟹ 每一份都起不来、日志里只有那一句。
    判据因此有两条：① 包目录**只能由 `rosbag2` 自己建**；② 日志**必须在包目录之外**。"""
    seen, states = _fake_start(monkeypatch, tmp_path)
    assert rec.start(1) == 0
    (argv, kw), = seen
    bag_dir = rec.session_dir(rec.SCENARIO_BY_N[1], datetime.datetime.now(), tmp_path / "bag")
    assert not bag_dir.exists(), "预先建了包目录 ⟹ rosbag2 会一句 already exists 直接退出"
    assert kw.get("start_new_session") is True, "它自成一个会话 ⟹ 终端 Ctrl-C 传不到它（要转交）"
    assert argv[:2] == ["bash", "-c"] and shlex.quote(rec.INCLUDE_REGEX) in argv[2]
    (state,) = states
    assert state["pid"] == 4321
    assert pathlib.Path(state["log"]).parent == bag_dir.parent, "日志跑进包里去了"
    assert pathlib.Path(state["log"]).parent.exists(), "要写日志，它那一层得先在"
    assert state["started_utc"] and state["git_head"], "对表要用的字段不能少"
    assert "静止基线" in capsys.readouterr().out, "起之前要把那张卡摊开给人看"


def test_包目录已经存在就只报不动(tmp_path, monkeypatch, capsys):
    """★ 两种成因都要说清，而且**都不许删**：一份是已经入账的包，另一份是没收干净的。"""
    _fake_start(monkeypatch, tmp_path)
    bag_dir = rec.session_dir(rec.SCENARIO_BY_N[1], datetime.datetime.now(), tmp_path / "bag")
    bag_dir.mkdir(parents=True)
    PRECIOUS = "已经录好的东西".encode()
    (bag_dir / "s1-static_0.db3").write_bytes(PRECIOUS)
    (bag_dir / "metadata.yaml").write_text("收好口了\n", encoding="utf-8")
    assert rec.start(1) == 1
    assert "不覆盖已入账的包" in capsys.readouterr().out
    assert (bag_dir / "s1-static_0.db3").read_bytes() == PRECIOUS

    (bag_dir / "metadata.yaml").unlink()            # 换一个成因：没收干净的那种
    assert rec.start(1) == 1
    out = capsys.readouterr().out
    assert "没有" in out and "本脚本不替你删包" in out
    assert bag_dir.exists()


def test_包不好看就返回非零(tmp_path, monkeypatch):
    """★ 返回码也有判据：`0` 只给"收好口**且**录到了东西"。空包 / 没收干净都不算成功 ——
    现场一句 `echo $?` 就该看得出这一趟作废了，而不是等人事后翻 `session.json`。"""
    _fake_start(monkeypatch, tmp_path, session={"messages": 0, "closed_cleanly": True})
    assert rec.start(1) == 1, "一条消息都没有的包不算成功"
    _fake_start(monkeypatch, tmp_path, session={"messages": 99, "closed_cleanly": False})
    assert rec.start(1) == 1, "不是干净收口（`metadata.yaml` 是补的）也不算成功"


# ─────────────────────────── 倒计时（前台那一段） ───────────────────────────


class _FakeClock:
    """假时钟 + 假 `sleep`（`sleep` 就是把时间往前拨）—— 120 s 的倒计时瞬间跑完。

    ★ 不这么写，这条链上的用例要么真等两分钟，要么就只能盯着常量看。
    """

    def __init__(self, t0: float = 1000.0):
        self.t = t0
        self.slept = 0

    def clock(self) -> float:
        return self.t

    def sleep(self, seconds: float) -> None:
        self.t += seconds
        self.slept += 1


def _run_watch(*, alive=lambda: True, tty=False, every_s=rec.PROGRESS_EVERY_S):
    fake = _FakeClock()
    stream = io.StringIO()
    outcome = rec.watch(rec.SCENARIO_BY_N[1], alive=alive, clock=fake.clock, sleep=fake.sleep,
                        stream=stream, tty=tty, started=fake.t, every_s=every_s)
    return outcome, fake, stream.getvalue()


def test_到点就自动收口():
    """★ 这一条就是操作者要的那个行为："录满 120 s **自己停掉**"。"""
    outcome, fake, text = _run_watch()
    assert outcome == "timeup"
    assert fake.t - 1000.0 >= rec.SCENARIO_BY_N[1].expect_s, "没到 120 s 就不许收口"


def test_录制进程先死了就不必干等到点():
    """★ 起录起失败了（域号不对、串口被占…）时，操作者不该对着倒计时站满两分钟 ——
    它死了就当场说，别把"等满 120 s"演成一个正常收口。"""
    fake_box = {}

    def alive() -> bool:
        return fake_box["fake"].t - 1000.0 < 3.0

    fake_box["fake"] = _FakeClock()          # 先造出来，好让 alive 闭包拿到它
    stream = io.StringIO()
    outcome = rec.watch(rec.SCENARIO_BY_N[1], alive=alive,
                        clock=fake_box["fake"].clock, sleep=fake_box["fake"].sleep,
                        stream=stream, tty=False, started=fake_box["fake"].t)
    assert outcome == "gone"
    assert fake_box["fake"].t - 1000.0 < rec.SCENARIO_BY_N[1].expect_s


def test_倒计时两种形状_交互的原地刷新_重定向的整行():
    """★ 往文件里灌 `\\r` 只会得到一坨连成一行的东西 ⟹ 非交互那条路上改成"每 10 s 一整行"。"""
    scenario = rec.SCENARIO_BY_N[1]
    tty_line = rec.render_progress(scenario, 60.0, tty=True)
    assert "\r" not in tty_line and "01:00" in tty_line and "60" in tty_line
    assert "█" in tty_line and "░" in tty_line
    assert "\r" not in rec.render_progress(scenario, 60.0, tty=False)

    _outcome, _fake, plain = _run_watch(tty=False)
    assert "\r" not in plain, "非交互流上不许出现回车符（会把日志糊成一行）"
    lines = plain.splitlines()
    assert len(lines) == 13, f"120 s / 每 10 s 一行 = 13 行（含 0 与 120），实际 {len(lines)}"
    assert lines[0].startswith("  [00:00 / 02:00]") and lines[-1].startswith("  [02:00 / 02:00]")

    _outcome, _fake, inplace = _run_watch(tty=True)
    assert "\r" in inplace and inplace.endswith("\n"), "原地刷新的那一行最后要收尾"


def test_倒计时的判据就是录制进程本身(monkeypatch, tmp_path):
    """★ `alive` 是回调、判据只有 `process_alive` 一处 —— 这里把两者接起来看：
    `watch` 拿到的那个回调，问的就是**我们起的那个 pid**。"""
    asked = []

    def fake_watch(scenario, *, alive, **kw):
        asked.append(alive())
        return "timeup"

    monkeypatch.setattr(rec, "watch", fake_watch)
    monkeypatch.setattr(rec, "process_alive", lambda pid: (asked.append(pid), True)[1])
    assert rec._run_countdown(rec.SCENARIO_BY_N[1], 4321, tmp_path / "x.log") == "timeup"
    assert asked == [4321, True], f"回调问的应当是 pid 4321：{asked}"


def test_ctrl_c_与到点走同一条收口路(monkeypatch, capsys, tmp_path):
    """★ 为什么要在 `_run_countdown` 里接住 Ctrl-C：录制进程自成一个会话
    （`start_new_session=True`），终端发的 SIGINT **传不到它** —— 不接住的话，
    按一下只会把我们打死、留下一份**没人收口**的包（它还在后台写着）。"""
    def boom(*a, **k):
        raise KeyboardInterrupt

    monkeypatch.setattr(rec, "watch", boom)
    assert rec._run_countdown(rec.SCENARIO_BY_N[1], 4321, tmp_path / "x.log") == "ctrl-c"
    assert "Ctrl-C" in capsys.readouterr().out


# ─────────────────────────── 收口 ───────────────────────────


def _fake_finalize_env(monkeypatch, tmp_path, *, session=None, used="SIGINT", alive=True):
    """`finalize` 的替身环境：假进程、假核对、假清单，返回 `(收到的 pid, summarize 的关键字)`。"""
    monkeypatch.setattr(rec, "BAG_ROOT", tmp_path / "bag")
    monkeypatch.setattr(rec, "STATE_FILE", tmp_path / "bag" / ".recorder.json")
    rec.write_state({"scenario": 1, "title": "静止基线", "pid": 4321,
                     "dir": "bag/2026-09-29/s1-static"})
    killed, seen = [], {}

    def fake_summarize(out_dir, **kw):
        seen.update(kw)
        return dict(session or _FAKE_SESSION)

    monkeypatch.setattr(rec, "process_alive", lambda pid: alive)
    monkeypatch.setattr(rec, "_stop_pid",
                        lambda pid: (killed.append(pid), used)[1])
    monkeypatch.setattr(rec, "summarize", fake_summarize)
    monkeypatch.setattr(rec, "write_manifest", lambda *a, **k: tmp_path / "bag" / "MANIFEST.md")
    return killed, seen


def test_收口发给录制进程的是信号_并且记下来为什么(tmp_path, monkeypatch, capsys):
    """★ 收口只有一条路：SIGINT（与操作者自己按 Ctrl-C 同一条）。
    表里还要看得见**为什么收的** —— "到点" / "人按的" / "它自己没的"在现场含义完全不同。"""
    killed, seen = _fake_finalize_env(monkeypatch, tmp_path)
    rec.finalize(tmp_path / "bag/2026-09-29/s1-static",
                 state=rec.read_state(), outcome="timeup")
    assert killed == [4321], "收口要发给**录制进程自己**那个 pid"
    assert seen["stop_signal"] == rec.STOP_REASON["timeup"]
    assert not rec.STATE_FILE.exists(), \
        "收完口要清掉状态文件 —— 不然 `rec_status` 会一直报一份早就没了的录制"


def test_进程已经退了的收口不假装是我们收的(tmp_path, monkeypatch):
    killed, seen = _fake_finalize_env(monkeypatch, tmp_path, alive=False)
    rec.finalize(tmp_path / "bag/2026-09-29/s1-static",
                 state=rec.read_state(), outcome="gone")
    assert killed == [], "它已经死了，不许再发信号"
    assert seen["stop_signal"] == rec.STOP_REASON["gone"]


def test_升级到_SIGKILL_要如实写在表里(tmp_path, monkeypatch):
    """★ SIGINT 被无视、升级到 SIGTERM/SIGKILL 时，表上不能只剩一句 "SIGINT" ——
    那种包**不是干净收口**，回查的人必须一眼看出差别。"""
    _killed, seen = _fake_finalize_env(monkeypatch, tmp_path, used="SIGKILL")
    rec.finalize(tmp_path / "bag/2026-09-29/s1-static",
                 state=rec.read_state(), outcome="timeup")
    assert "SIGKILL" in seen["stop_signal"] and "升级" in seen["stop_signal"]


def test_空包必须当场喊出来(tmp_path, monkeypatch, capsys):
    _killed, _seen = _fake_finalize_env(
        monkeypatch, tmp_path, session={"messages": 0, "closed_cleanly": True})
    rec.finalize(tmp_path / "bag/2026-09-29/s1-static",
                 state=rec.read_state(), outcome="timeup")
    assert "一条消息都没有" in capsys.readouterr().out


def test_收口那一段忽略_ctrl_c():
    """★ 告诉人"想停就按 Ctrl-C"之后，他多半会**连按两下**（第一下之后屏幕上有几秒
    在"收口中（写缓存 → 压 zstd）"）。第二下若把我们打断，`ros2 bag info` 与
    `session.json` 就都不会跑了 —— 而 `rosbag2` 那边正在好好收尾。"""
    before = signal.getsignal(signal.SIGINT)
    with rec._sigint_ignored():
        assert signal.getsignal(signal.SIGINT) is signal.SIG_IGN
    assert signal.getsignal(signal.SIGINT) is before, "出了这一小段要装回去"


def test_起录不把自己的时刻塞给收口(tmp_path, monkeypatch):
    """★ 回归（2026-09-29 复核揪出的真事）：`start` 手里那个 `now` 是**起录**时刻，
    原先一路传进 `finalize` ⟹ `session.json` 里 `stopped_utc` 与 `started_utc`
    **逐字相同**。而 `bag/README.md` 正是拿这两个时间戳去和飞控日志对表 ——
    照它读会得出"这份包只有 0 秒长"，而那个字段整个存在的理由就是能对上飞控。

    判据：`start` **不许**把 `now` 传给 `finalize`（收口时刻由收口那一刻自己取）。
    """
    _fake_start(monkeypatch, tmp_path)
    got: Dict = {}
    monkeypatch.setattr(rec, "finalize",
                        lambda out_dir, **kw: (got.update(kw), dict(_FAKE_SESSION))[1])
    assert rec.start(1) == 0
    assert "now" not in got, "起录时刻被当成收口时刻传下去了 ⟹ `stopped_utc` 会说谎"


def test_收口时刻由收口这一刻自己取(tmp_path, monkeypatch):
    """`finalize` 没拿到 `now` 就自己取一个 —— 起录在很久以前时，收口时刻**必须**晚于它。"""
    _killed, seen = _fake_finalize_env(monkeypatch, tmp_path)
    state = rec.read_state()
    state["started_utc"] = "2020-01-01T00:00:00Z"       # 假装这份包是 2020 年起录的
    rec.finalize(tmp_path / "bag/2026-09-29/s1-static", state=state, outcome="timeup")
    stamped = rec.utc_stamp(seen["now"])
    assert stamped != state["started_utc"], "收口时刻被起录时刻顶替了"
    assert stamped > state["started_utc"], f"收口应当晚于起录（{stamped}）"


def test_utc_stamp_把本地时间换算成真_UTC():
    """★ 回归（2026-09-29 复核揪出）：传进来的 `now` 是**朴素的本地时间**
    （`_dt.datetime.now()` 就是这个形状），原先直接 `strftime` 贴个 `Z` ⟹
    本地时区不是 UTC 的机器上，这个**专门用来和飞控对表**的字段是错的。
    本机恰好本地==UTC，所以一直没露出来。

    ★ 判据必须**与本机时区无关** —— 本机恰好本地==UTC，拿"朴素值"当判据的话
    旧代码也能过（两边都是 12:00:00Z）。所以主判据用**一个明确的非 UTC 时区**：
    东九区的 21:00 就是 UTC 的 12:00，贴 `Z` 而不换算会写成 `21:00:00Z`。
    """
    tokyo = datetime.timezone(datetime.timedelta(hours=9))
    assert rec.utc_stamp(datetime.datetime(2026, 9, 29, 21, 0, 0, tzinfo=tokyo)) \
        == "2026-09-29T12:00:00Z", "带时区的时刻要**换算**到 UTC，不能照着墙上钟贴个 Z"
    assert rec.utc_stamp(datetime.datetime(2026, 9, 29, 12, 0, 0,
                                           tzinfo=datetime.timezone.utc)) \
        == "2026-09-29T12:00:00Z", "本来就是 UTC 的，不许再挪一次"
    naive = datetime.datetime(2026, 9, 29, 12, 0, 0)     # 朴素值：按本机时区解释
    assert rec.utc_stamp(naive) == rec.utc_stamp(naive.astimezone()), \
        "朴素值要按本地时间转 UTC —— 与「同一个时刻、带本机时区」必须盖出同一个戳"


def test_没有报文段时摘要不冒充哈希(tmp_path):
    """★ 复核 2026-09-29：原先对**一个报文段都没有**的目录返回 `sha256("")`，
    也就是 `e3b0c442…` —— 那玩意儿**长得就像一个正经哈希**，摆在表里会被读成
    "这份包算过了"。没有东西可算就说没有。"""
    bag = tmp_path / "bag/2026-09-29/s1-static"
    bag.mkdir(parents=True)
    assert rec.sha256_of_bag(bag) == "", "空目录不许返回那个「空输入的哈希」"
    (bag / "s1-static_0.db3").write_bytes(b"x")
    assert len(rec.sha256_of_bag(bag)) == 64, "有报文段了才该给出真摘要"


def test_零时长的包不许说成没读到():
    """★ 复核 2026-09-29：时长用**真值**判断 ⟹ 一个真的 `0.0 s`（收口正常、只是消息
    时间跨度为零）会被印成"没读到" —— 那是把"量到了 0"说成"没量到"，
    正是本仓为"静默为空"交过三次学费的那个毛病。判据：`0.0` 与"没有"分开说。
    """
    out = rec.render_record_table(dict(_FAKE_SESSION, duration_s=0.0, sha256=""))
    dur_line = [line for line in out.splitlines() if "时长" in line][0]
    assert "0.0 s" in dur_line, f"真的 0.0 s 被说成没读到了：{dur_line!r}"
    missing = [line for line in rec.render_record_table(
        dict(_FAKE_SESSION, duration_s=None)).splitlines() if "时长" in line][0]
    assert "没读到" in missing, "真的没读到还是要说没读到"


# ─────────────────────────── 菜单入口 ───────────────────────────


def test_不给场景号时问一句并默认_1(monkeypatch, capsys):
    seen = []
    monkeypatch.setattr(rec, "start", lambda n: (seen.append(n), 0)[1])
    monkeypatch.setattr("builtins.input", lambda *a: "")
    assert rec.cmd_start([]) == 0
    assert seen == [1]
    assert "静止基线" in capsys.readouterr().out      # 四条都摊出来给人选


def test_给了场景号就不问(monkeypatch):
    seen = []
    monkeypatch.setattr(rec, "start", lambda n: (seen.append(n), 0)[1])
    monkeypatch.setattr("builtins.input", lambda *a: pytest.fail("给了号就不该再问"))
    assert rec.cmd_start(["3"]) == 0
    assert seen == [3]


def test_场景号打成别的字要报出来(monkeypatch, capsys):
    monkeypatch.setattr(rec, "start", lambda n: pytest.fail("不该起"))
    monkeypatch.setattr("builtins.input", lambda *a: "一")
    assert rec.cmd_start([]) == 2
    assert "不是一个场景号" in capsys.readouterr().out


# ─────────────────────────── `--dry-run` ───────────────────────────


def test_dry_run_把四条场景的命令全摊开(tmp_path, monkeypatch, capsys):
    """★ `--dry-run` 的判据是**一个进程都不起**（`z0.dispatch` 那条）—— 这条用例只读文本，
    真去起录的路径在上面那几条里已经证明它是"先查场景号、再查有没有在录"。"""
    monkeypatch.setattr(rec, "BAG_ROOT", tmp_path / "bag")
    text = rec.dry_run_note("rec_start")
    for scenario in rec.SCENARIOS:
        assert scenario.title in text
    assert shlex.quote(rec.INCLUDE_REGEX) in text
    assert "一个进程都不起" in text
    assert not (tmp_path / "bag").exists()
def test_文件头引用的菜单号与动作表对得上():
    """★ 与 `operation_manual.md` 那条守卫同一个形状：**菜单号不许各写各的**——
    `rec.py` 里凡是写"菜单 **N= 标题**"的地方（文件头用法、场景卡的起链条、运行提示），
    标题都必须与 `z0.ACTIONS` 第 N 项对得上。

    ⚠️ 12/13 连功能一起删掉之后（2026-09-29），本文件里还剩 **8 / 9 / 11** 三处引用
    （8、9 是起链那两条，11 是它自己）。下面那句 `len(...) >= 3` 是**数量下限**，
    真正咬人的是逐条 `startswith` 对表 —— 少写一处菜单号它不会红，
    写错一个菜单号它一定红。
    """
    text = pathlib.Path(rec.__file__).read_text(encoding="utf-8")
    refs = [(int(n), title.strip())
            for n, title in re.findall(r"菜单\s*\*\*(\d+)=\s*([^*]+?)\*\*", text)]
    assert len(refs) >= 3, f"`rec.py` 里引用的菜单号太少（只认出 {refs}）—— 是不是漏写了？"
    for number, cited in refs:
        title = z0.ACTIONS[number - 1].title
        assert _plain(title).startswith(_plain(cited)), \
            f"菜单 **{number}= {cited}** 与动作表对不上：第 {number} 项是「{title}」"


def _plain(text: str) -> str:
    return re.sub(r"\s+", " ", text.replace("*", " ")).lstrip("★ ").strip()


def test_录制那几条与_z0_的进程内动作表对齐():
    assert set(rec.IN_PROCESS_KEYS) <= set(z0.IN_PROCESS_KEYS), \
        "`z0.in_process_command` 里必须有这几条（否则 `run/z0.py rec_start` 会报不认识）"
    assert set(z0.TAKES_ARGS) <= set(rec.IN_PROCESS_KEYS)
    for key in z0.TAKES_ARGS:
        assert callable(getattr(rec, f"cmd_{key.removeprefix('rec_')}"))
# ─────────────────────────── 清单（`bag/MANIFEST.md`） ───────────────────────────


def test_清单把每份包与缺了什么摊开(tmp_path):
    day = tmp_path / "2026-09-29"
    rec.write_state(_session(), day / "s3-traj1-hover" / "session.json")
    rec.write_state(_session(scenario=4, title=rec.SCENARIO_BY_N[4].title,
                             dir="bag/2026-09-29/s4-traj3-lemniscate",
                             missing_expected=["/traj/progress"]),
                    day / "s4-traj3-lemniscate" / "session.json")
    (day / "fcu").mkdir(parents=True)
    (day / "fcu" / "sess001-log001.ulg").write_bytes(b"ULOG" * 10)
    text = rec.render_manifest(tmp_path)
    assert "s3-traj1-hover" in text and "s4-traj3-lemniscate" in text
    assert "`deadbeef`" in text                      # sha256 要能被人回验
    assert "/traj/progress" in text                  # ★ 缺的那条点名，不是一句"基本通过"
    assert "sess001-log001.ulg" in text


def test_一份包都没有时清单说实话(tmp_path):
    text = rec.render_manifest(tmp_path)
    assert "还没有收好口的包" in text
    assert "还没有 `.ulg`" in text
    assert "手工拷进 `bag/<日期>/fcu/`" in text, "空的时候要说清**怎么才能有**（飞控日志是手工动作）"


def test_清单是确定性输出_重算逐字相同(tmp_path):
    """★ 回归（2026-09-29 复核抓到）：原先表头带一行 `生成于 <now>`，
    而这张表是**入库**的 ⟹ 每录完一份就脏一次，验收那条"`rec_manifest` 重算得一致"
    用 `git diff` 永远对不上。起点时间在表里（`started_utc`），回查不缺东西。
    """
    day = tmp_path / "2026-09-29"
    (day / "s1-static").mkdir(parents=True)
    (day / "s1-static" / "session.json").write_text(json.dumps({
        "scenario": 1, "title": "静止基线", "dir": "2026-09-29/s1-static",
        "started_utc": "2026-09-29T01:00:00", "duration_s": 90.0, "size_text": "1.0 MB",
        "messages": 100, "topics": [], "sha256": "deadbeef", "missing_expected": [],
    }, ensure_ascii=False), encoding="utf-8")
    first = rec.render_manifest(tmp_path)
    assert first == rec.render_manifest(tmp_path), "同一份输入重算两次必须逐字相同"
    assert "生成于" not in first, "★ 入库的表里不许有时间戳 —— 它会让每次重算都脏"
    assert "2026-09-29T01:00:00" in first, "回查要靠每份包自己的起点时间"


def test_清单里零时长与没读到分开说(tmp_path):
    """同上那条，只是换到**入库的那张表**上（`MANIFEST.md`）：`?` 只能代表"没读到"。"""
    for slug, duration in (("s1-static", 0.0), ("s2-manual-takeoff", None)):
        day = tmp_path / "2026-09-29" / slug
        day.mkdir(parents=True)
        (day / "session.json").write_text(json.dumps({
            "scenario": 1, "title": "静止基线", "dir": f"2026-09-29/{slug}",
            "started_utc": "2026-09-29T01:00:00", "duration_s": duration,
            "size_text": "1.0 MB", "messages": 0, "topics": [], "sha256": "",
            "missing_expected": [],
        }, ensure_ascii=False), encoding="utf-8")
    row_zero = [ln for ln in rec.render_manifest(tmp_path).splitlines()
                if "s1-static" in ln]
    # 每个 slug 各一行；0.0 那种要写成 "0.0 s"，不能是 "?"
    assert any("| 0.0 s |" in ln for ln in row_zero), f"真的 0.0 s 被写成问号：{row_zero}"
    assert "| ? |" in rec.render_manifest(tmp_path), "没读到的才写问号"


def test_写清单会建目录(tmp_path, monkeypatch):
    monkeypatch.setattr(rec, "BAG_ROOT", tmp_path / "bag")
    path = rec.write_manifest()
    assert path.exists() and path.name == "MANIFEST.md"


# ─────────────────────────── ★ 安全边界 ───────────────────────────


def test_源码里不出现发布与服务调用():
    """★ 走 AST（不是字符串搜索）：守卫表自己写着那些字样，字符串搜索会把守卫表当成违规
    —— 与 `test_z0_menu.py` 那条同一个理由。

    **录制层只许订阅**：`ros2 bag record` 是纯订阅，多一个 `create_publisher` 就多一条
    能碰飞机的路（本仓库的安全边界是"没有任何会动飞机的入口"）。"""
    tree = ast.parse(pathlib.Path(rec.__file__).read_text(encoding="utf-8"))
    attributes = {node.attr for node in ast.walk(tree) if isinstance(node, ast.Attribute)}
    forbidden_api = {"create_publisher", "create_service", "create_client"}
    assert not (forbidden_api & attributes), f"录制层只许订阅：{forbidden_api & attributes}"

    literals = [node.value for node in ast.walk(tree)
                if isinstance(node, ast.Constant) and isinstance(node.value, str)]
    for token in z0.FORBIDDEN_IN_COMMANDS:
        hits = {text for text in literals if token in text.lower()} - {token}
        assert not hits, f"`{token}` 出现在了守卫表之外：{hits}"


def test_录制层只发_ros2_那一条命令():
    """`ACTIONS` 那条不变式（"命令只许是 `ros2 …`"）对录制层同样成立：
    它送进 `bash -c` 的东西**只有** `ros2 bag record` 与 z0 那几条 source 前缀。"""
    argv = rec.record_argv(pathlib.Path("/tmp/x"))
    assert argv[0] == "ros2"
    assert rec.record_command(pathlib.Path("/tmp/x")).startswith("ros2 bag record")
