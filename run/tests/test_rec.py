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


def test_六个场景就是票面那六个():
    """★ 操作者 2026-10-08 拍定的那张表：`1` 静止 / `2` 手飞起飞 / `3` 轨迹 1 /
    `4` 轨迹 2（正方形）/ `5` 轨迹 3（八字）/ `6` 任意录制。

    ⚠️ **4 与 5 相对 09-29 那一版调过个儿**：那时 4 是八字。票面写的是"轨迹 3=轨迹 1，
    4=轨迹 2，5=轨迹 3"⟹ 轨迹号对齐本来那张表，八字从 4 挪到 5。
    """
    assert [s.n for s in rec.SCENARIOS] == [1, 2, 3, 4, 5, 6]
    assert len({s.slug for s in rec.SCENARIOS}) == 6, "落盘目录名不许重"
    assert set(rec.SCENARIO_BY_N) == {1, 2, 3, 4, 5, 6}
    # 场景 1/2 没有 control（手动遥控飞那条链），3/4/5 才是全链，6 是自由录（链由人自己定）
    assert (rec.SCENARIO_BY_N[1].chain, rec.SCENARIO_BY_N[2].chain) == ("up_manual", "up_manual")
    assert tuple(rec.SCENARIO_BY_N[n].chain for n in (3, 4, 5)) == ("up", "up", "up")
    assert rec.SCENARIO_BY_N[6].chain == "any"


def test_轨迹号与飞控那边的两条轨迹一一对上():
    """★ 票面那句话（"3=轨迹 1，4=轨迹 2，5=轨迹 3"）指的是**飞行程序那边**的轨迹号
    （`fly.py` 的 1/2/3）—— 场景号与轨迹号是**两套编号**，别把它们混成一套。
    这里的判据取自场景自己的 `do` 文字，而不是另抄一张对照表（抄了就会各自漂移）。
    """
    for scene_n, kind in ((3, "1"), (4, "2"), (5, "3")):
        do = rec.SCENARIO_BY_N[scene_n].do
        assert f"**{kind}**" in do, \
            f"场景 {scene_n} 的 `do` 里应当点名让飞手选轨迹 {kind}（现在是：{do!r}）"


def test_每个场景都写清楚要人做什么_预期话题也能当正则用():
    for scenario in rec.SCENARIOS:
        assert scenario.title and scenario.slug and scenario.do
        assert scenario.chain in rec.CHAIN_MENU_TITLE, f"场景 {scenario.n} 的链没有菜单出处"
        for pattern in scenario.expect:
            re.compile(pattern)                     # ★ 写错的正则要在这里炸，不是收口时
    # ★ 有倒计时的那几条（1–5）必须同时给出预期话题 —— "到点收口"与"该收到什么"是一套的
    for scenario in rec.SCENARIOS:
        if scenario.expect_s is not None:
            assert scenario.expect_s > 0, f"场景 {scenario.n}：倒计时没写成 `None`，就得是正数"
            assert scenario.expect, f"场景 {scenario.n} 有倒计时却没写预期话题"


def test_五个场景的时长统一_120s_且它是自动收口的判据():
    """★ 操作者 2026-09-29 现场拍定：**统一 120 s，到点自动收口**
    （演变：原先 1/2 是 90、3 是 150、4 是 120 ⟹ 统一 90 ⟹ 统一 **120**）。
    钉在用例里，改一个就得一起改 —— 它现在是**收口的判据**（`watch` 拿它比时间），
    不再是"提示"：几个不一样 ⟹ 现场每个人心里的"录够了没有"都不一样。

    ⚠️ 场景 6（任意录制）**不在这一条里**：它 `expect_s is None`（见下一条用例），
    因此**不取 `0.0`** —— 0 会被读成"立刻到点"。
    ⚠️ 实际时长以 `session.json` 的 `duration_s` 为准（秒级调度 ⟹ 差个零点几秒是正常的）。
    """
    ticked = [s for s in rec.SCENARIOS if s.expect_s is not None]
    assert [s.n for s in ticked] == [1, 2, 3, 4, 5]
    assert {s.expect_s for s in ticked} == {120.0}


def test_场景6是自由录_不倒计时_也不预设预期话题():
    """★ 操作者 2026-10-08 拍定："不预设场景/时长，起录后我按 Ctrl-C 停"。

    ⚠️ 两个坑都钉在这里：
      ① `expect_s` 必须是 **`None`**，不能是 `0.0` —— `watch` 拿 `expect_s` 比时间，
         0 会在第一帧就判"到点"（而自由录**永远**不该自己收口）；
      ② `expect = ()` **合法**（`render_scenario_card` / `render_record_table` 都按
         "自由录 —— 不预设"这一支印），但**不能是别的场景的副本** —— 那会让人以为
         "这一场要录到那些话题才算数"。
    """
    free = rec.SCENARIO_BY_N[6]
    assert free.expect_s is None
    assert free.expect == ()
    assert free.chain == "any", "自由录的链由人自己定（手动遥控飞 or 程序飞）"
    assert "Ctrl-C" in free.do, "自由录的 `do` 必须写清**怎么停**（屏幕上没有倒计时可以看）"


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


def test_六份场景落盘目录各不相同_且都直接在_bag_下(tmp_path):
    """★ 布局是**扁平**的（操作者 2026-09-29 现场拍定）：`bag/<slug>/`，**没有日期层**。
    日期没丢 —— 每份包自己的 `started_utc` 写在 `session.json` 与 `MANIFEST.md` 里，
    现场按场景找东西时目录名就是要记的那一个。
    ⚠️ 代价：同名场景再录一次会撞上已有目录（`rosbag2` 拒绝往已有目录里录）⟹ 那时录进
    `unique_dir` 那份带时间戳的（见下面的用例）。
    """
    dirs = [rec.session_dir(s, tmp_path) for s in rec.SCENARIOS]
    assert len(set(dirs)) == 6
    for scenario, path in zip(rec.SCENARIOS, dirs):
        assert path == tmp_path / scenario.slug
        assert rec.record_argv(path)[4] == str(path)         # `-o` 就是那个目录
        assert rec.scenario_by_slug(path.name) is scenario, \
            "★ 目录名必须能回认出场景号 —— 收编（`adopt`）只有这一条线索"


# ─────────────────────────── 重录：带时间戳的新目录 ───────────────────────────


def test_重录落进带时间戳的新目录_且认得回场景号(tmp_path):
    """★ 2026-10-08 现场那条主诉的修法：**目录已存在不再是一句死路**。

    判据三条：
      ① `unique_dir` 与首选目录**不同名**、且**前缀就是 slug**（人能一眼看出是哪个场景）；
      ② 它**不在**首选目录里（是兄弟，不是子目录）—— 免得 `rosbag2` 把两者混起来；
      ③ 名字还得能回认出场景号（`adopt` 只有目录名这一条线索）。
    """
    for scenario in rec.SCENARIOS:
        preferred = rec.session_dir(scenario, tmp_path)
        fresh = rec.unique_dir(scenario, tmp_path)
        assert fresh != preferred
        assert fresh.parent == tmp_path
        assert fresh.name.startswith(scenario.slug + "__"), fresh.name
        assert rec.scenario_by_slug(fresh.name) is scenario
        assert rec.dir_stamp() in fresh.name


def test_同一秒里连来两次也不覆盖(tmp_path):
    """⚠️ 理论上要两次起录落在同一秒才会走到这一支 —— 但那一支**不许**是"覆盖"或"报错"：
    加 `-2` / `-3` 往后排，且**仍然认得出场景号**。"""
    now = datetime.datetime(2026, 10, 8, 7, 15, 30)
    first = rec.unique_dir(rec.SCENARIO_BY_N[6], tmp_path, now=now)
    first.mkdir(parents=True)
    second = rec.unique_dir(rec.SCENARIO_BY_N[6], tmp_path, now=now)
    assert second != first and not second.exists()
    assert rec.scenario_by_slug(second.name) is rec.SCENARIO_BY_N[6]
    second.mkdir(parents=True)
    third = rec.unique_dir(rec.SCENARIO_BY_N[6], tmp_path, now=now)
    assert third not in (first, second) and not third.exists()


def test_时间戳是_UTC_且不带冒号(tmp_path):
    """★ 与 `utc_stamp` 同一把尺（UTC）—— 现场要拿它和 `session.json` 的 `started_utc`、
    飞控日志的 `utc` 对表。⚠️ 有意**不带冒号**：目录名是要被人手打、被 `git` 记住的东西。"""
    stamp = rec.dir_stamp()
    assert re.fullmatch(r"\d{8}-\d{6}", stamp), stamp
    assert ":" not in stamp
    # 传一个**本地朴素时间**进去，取出来必须是换算过的 UTC（同一个坑 `utc_stamp` 踩过）
    naive = datetime.datetime(2026, 10, 8, 7, 15, 30)
    assert rec.dir_stamp(naive) == naive.astimezone(datetime.timezone.utc).strftime(
        "%Y%m%d-%H%M%S")


def test_老编号的八字目录仍认得出场景号():
    """★★ `bag/4_traj3_lemniscate/` 是 **09-29 入库的那份真数据**（八字，那时它是场景 4）。
    2026-10-08 把八字挪到场景 5、4 让给正方形 ⟹ 那份包**不改名、不动内容**（它是数据），
    所以"认目录名"这一步必须把它认回**当前的场景 5**，否则收编与清单会把它当成认不出的包。"""
    assert rec.SLUG_ALIASES == {"4_traj3_lemniscate": 5}
    scenario = rec.scenario_by_slug("4_traj3_lemniscate")
    assert scenario is rec.SCENARIO_BY_N[5]
    # ★ 别名也要能接住"带时间戳的那一份"（`<老目录名>__<stamp>`）
    assert rec.scenario_by_slug("4_traj3_lemniscate__20261008-071530") is scenario
    # ⚠️ 别名**不是**"所有旧编号都认"：只列了这一条，别的照样认不出（不许瞎猜）
    assert rec.scenario_by_slug("9_nonexistent") is None


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
        "scenario": 3, "title": rec.SCENARIO_BY_N[3].title, "dir": "bag/3_traj1_hover",
        "duration_s": 150.0, "size_text": "12.3 MiB", "messages": 12345,
        "files": ["3_traj1_hover_0.db3.zstd"], "sha256": "deadbeef",
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
    "scenario": 1, "title": rec.SCENARIO_BY_N[1].title, "dir": "bag/1_static",
    "duration_s": 120.0, "size_text": "1.0 MiB", "files": ["1_static_0.db3.zstd"],
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
    bag_dir = rec.session_dir(rec.SCENARIO_BY_N[1], tmp_path / "bag")
    assert not bag_dir.exists(), "预先建了包目录 ⟹ rosbag2 会一句 already exists 直接退出"
    assert kw.get("start_new_session") is True, "它自成一个会话 ⟹ 终端 Ctrl-C 传不到它（要转交）"
    assert argv[:2] == ["bash", "-c"] and shlex.quote(rec.INCLUDE_REGEX) in argv[2]
    (state,) = states
    assert state["pid"] == 4321
    assert pathlib.Path(state["log"]).parent == bag_dir.parent, "日志跑进包里去了"
    assert pathlib.Path(state["log"]).parent.exists(), "要写日志，它那一层得先在"
    assert state["started_utc"] and state["git_head"], "对表要用的字段不能少"
    assert "静止基线" in capsys.readouterr().out, "起之前要把那张卡摊开给人看"


def _occupied(tmp_path, *, closed: bool):
    """造出"首选目录已经被占"那种情形（`closed=True` = 有一份收好口的包）。"""
    bag_dir = rec.session_dir(rec.SCENARIO_BY_N[1], tmp_path / "bag")
    bag_dir.mkdir(parents=True)
    (bag_dir / "1_static_0.db3").write_bytes(PRECIOUS)
    if closed:
        (bag_dir / "metadata.yaml").write_text("收好口了\n", encoding="utf-8")
    return bag_dir


PRECIOUS = "已经录好的东西".encode()


def test_首选目录被占_答_n_就只报不动(tmp_path, monkeypatch, capsys):
    """★ **默认那一条**（空回车 ⟹ n）：两种成因都要说清，而且**都不许删、不许挪**。"""
    _fake_start(monkeypatch, tmp_path)
    monkeypatch.setattr(rec, "ask_confirm", lambda *a, **k: False)
    bag_dir = _occupied(tmp_path, closed=True)
    assert rec.start(1) == 1
    assert "不覆盖已入账的包" in capsys.readouterr().out
    assert (bag_dir / "1_static_0.db3").read_bytes() == PRECIOUS

    (bag_dir / "metadata.yaml").unlink()            # 换一个成因：没收干净的那种
    assert rec.start(1) == 1
    out = capsys.readouterr().out
    assert "没有" in out and "本脚本不替你删包" in out
    assert bag_dir.exists()


def test_首选目录被占_答_y_就录进带时间戳的新目录(tmp_path, monkeypatch, capsys):
    """★★ 2026-10-08 那条主诉的正面判据：**答 y 之后这一趟真的录起来**。

    ⚠️ 现场那条主诉的形状是"一选场景就说目录已存在，什么都没录" —— 四个目录全是 09-29
    入库的那几份，所以**每一个场景号都会撞上** ⟹ 从操作者眼里看就是"这项录制坏了"。
    修法不是"允许覆盖"（那是拿入库的数据换方便），而是**另开一份新目录**。
    """
    seen, states = _fake_start(monkeypatch, tmp_path)
    monkeypatch.setattr(rec, "ask_confirm", lambda *a, **k: True)
    bag_dir = _occupied(tmp_path, closed=True)
    assert rec.start(1) == 0, "答了 y 还返回非零 ⟹ 现场看到的就是主诉里那句「什么都没录」"
    (argv, _kw), = seen
    (state,) = states
    recorded = pathlib.Path(state["dir"])          # 落在 `tmp_path` 下 ⟹ `_rel` 给的是绝对路径
    assert recorded != bag_dir and not recorded.exists(), "rosbag2 自己建，我们不能预先建"
    assert recorded.name.startswith("1_static__"), state["dir"]
    assert recorded.parent == tmp_path / "bag"
    assert rec.scenario_by_slug(recorded.name) is rec.SCENARIO_BY_N[1]
    # ★ 已有那份**一个字节都不碰**（这是这条修法的全部意义）
    assert (bag_dir / "1_static_0.db3").read_bytes() == PRECIOUS
    assert (bag_dir / "metadata.yaml").read_text(encoding="utf-8") == "收好口了\n"
    # ★ 日志跟着**包目录名**走，否则第二次起录会盖掉第一份的日志（排障要看的就是它）
    assert pathlib.Path(state["log"]).name == recorded.name + ".log"
    assert str(recorded) in argv[2], "真跑的命令得指向那份新目录"


def test_场景6永远落进带时间戳的新目录(tmp_path, monkeypatch):
    """★ 自由录**没有"首选目录"**这个概念 ⟹ 它连问都不问一句，天生就是一份新目录
    （⟹ 同一个自由录录几遍就几份，互不相干）。"""
    seen, states = _fake_start(monkeypatch, tmp_path)
    monkeypatch.setattr(rec, "ask_confirm",
                        lambda *a, **k: pytest.fail("自由录不该问 —— 它本来就没有首选目录"))
    # ★ 自由录那一档 `watch` **永远不返 `"timeup"`**（见 `watch` 的 ★★）⟹ 替身也得照这个
    #   来，否则这条用例测的就不是"自由录会怎样"，而是"替身编出来的一个不可能的状态"。
    monkeypatch.setattr(rec, "watch", lambda *a, **k: "ctrl-c")
    assert rec.start(6) == 0
    (state,) = states
    assert pathlib.Path(state["dir"]).name.startswith("6_free__"), state["dir"]
    (argv, _kw), = seen
    assert "6_free__" in argv[2]


def test_ask_confirm_非交互时不抛异常也走默认(capsys):
    """★ 与 `z0.ask_confirm` 同一条纪律：管道 / 重定向 / 关掉 stdin 时**不许抛** —— 走默认。
    ⚠️ 默认是 **N**（不换目录）：一条"没人看着"的路径不该自己决定多录一份包出来。"""
    def eof(_prompt):
        raise EOFError

    import builtins
    monkeypatch_ok = builtins.input
    builtins.input = eof
    try:
        assert rec.ask_confirm("换一份？") is False
    finally:
        builtins.input = monkeypatch_ok


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
                     "dir": "bag/1_static"})
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
    rec.finalize(tmp_path / "bag/1_static",
                 state=rec.read_state(), outcome="timeup")
    assert killed == [4321], "收口要发给**录制进程自己**那个 pid"
    assert seen["stop_signal"] == rec.STOP_REASON["timeup"]
    assert not rec.STATE_FILE.exists(), \
        "收完口要清掉状态文件 —— 不然 `rec_status` 会一直报一份早就没了的录制"


def test_进程已经退了的收口不假装是我们收的(tmp_path, monkeypatch):
    killed, seen = _fake_finalize_env(monkeypatch, tmp_path, alive=False)
    rec.finalize(tmp_path / "bag/1_static",
                 state=rec.read_state(), outcome="gone")
    assert killed == [], "它已经死了，不许再发信号"
    assert seen["stop_signal"] == rec.STOP_REASON["gone"]


def test_升级到_SIGKILL_要如实写在表里(tmp_path, monkeypatch):
    """★ SIGINT 被无视、升级到 SIGTERM/SIGKILL 时，表上不能只剩一句 "SIGINT" ——
    那种包**不是干净收口**，回查的人必须一眼看出差别。"""
    _killed, seen = _fake_finalize_env(monkeypatch, tmp_path, used="SIGKILL")
    rec.finalize(tmp_path / "bag/1_static",
                 state=rec.read_state(), outcome="timeup")
    assert "SIGKILL" in seen["stop_signal"] and "升级" in seen["stop_signal"]


def test_空包必须当场喊出来(tmp_path, monkeypatch, capsys):
    _killed, _seen = _fake_finalize_env(
        monkeypatch, tmp_path, session={"messages": 0, "closed_cleanly": True})
    rec.finalize(tmp_path / "bag/1_static",
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


def test_收口的每一件事都在忽略_ctrl_c_的窗口里(tmp_path, monkeypatch):
    """★★ 回归（2026-09-29 现场丢了东西）：旧版的窗口**只包住"等它退出"那一段**，
    核对与写清单在外面 ⟹ 操作者第二下 Ctrl-C 打断的就是那几秒 ⟹ **3 份包没有
    `session.json`**（`rosbag2` 那边其实收得好好的，丢的全是收口记录）。

    判据：收口这条路里的**每一步**（停进程 / 量包 / 写清单 / 清状态文件）都在窗口里 ——
    逐点取 `signal.getsignal` 来看，而不是只测那个上下文管理器（那样测不出范围）。
    """
    monkeypatch.setattr(rec, "BAG_ROOT", tmp_path / "bag")
    monkeypatch.setattr(rec, "STATE_FILE", tmp_path / "bag" / ".recorder.json")
    rec.write_state({"scenario": 1, "title": "静止基线", "pid": 4321, "dir": "bag/1_static"})
    seen: Dict[str, object] = {}
    note = lambda name: seen.__setitem__(name, signal.getsignal(signal.SIGINT))  # noqa: E731

    monkeypatch.setattr(rec, "process_alive", lambda pid: True)
    monkeypatch.setattr(rec, "_stop_pid", lambda pid: (note("停录制进程"), "SIGINT")[1])
    monkeypatch.setattr(rec, "summarize",
                        lambda out_dir, **kw: (note("量包核对"), dict(_FAKE_SESSION))[1])
    monkeypatch.setattr(rec, "write_manifest",
                        lambda *a, **k: (note("写清单"), tmp_path / "MANIFEST.md")[1])
    rec.finalize(tmp_path / "bag/1_static", state=rec.read_state(), outcome="timeup")
    assert set(seen) == {"停录制进程", "量包核对", "写清单"}, f"收口这几步没全走到：{set(seen)}"
    for step, handler in seen.items():
        assert handler is signal.SIG_IGN, \
            f"「{step}」不在忽略 Ctrl-C 的窗口里 —— 第二下就会把它打断（2026-09-29 就是这么丢的）"
    assert not rec.STATE_FILE.exists(), "清状态文件也在同一段里，不该留着"
    assert signal.getsignal(signal.SIGINT) is not signal.SIG_IGN, "收完口要装回去"


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
    rec.finalize(tmp_path / "bag/1_static", state=state, outcome="timeup")
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
    bag = tmp_path / "bag/1_static"
    bag.mkdir(parents=True)
    assert rec.sha256_of_bag(bag) == "", "空目录不许返回那个「空输入的哈希」"
    (bag / "1_static_0.db3").write_bytes(b"x")
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


def test_dry_run_把六条场景的命令全摊开(tmp_path, monkeypatch, capsys):
    """★ `--dry-run` 的判据是**一个进程都不起**（`z0.dispatch` 那条）—— 这条用例只读文本，
    真去起录的路径在上面那几条里已经证明它是"先查场景号、再查有没有在录"。

    ★ 场景 6 那一行也要与真跑的落点**同一把尺**（带时间戳的新目录）—— 否则这一屏会显示一个
      "真跑时根本不会用"的目录名，而 `--dry-run` 的全部意义就是那一条。
    """
    monkeypatch.setattr(rec, "BAG_ROOT", tmp_path / "bag")
    text = rec.dry_run_note("rec_start")
    for scenario in rec.SCENARIOS:
        assert scenario.title in text
    assert shlex.quote(rec.INCLUDE_REGEX) in text
    assert "一个进程都不起" in text
    assert "6_free__" in text, "自由录的落点要按带时间戳的新目录印"
    assert rec.session_dir(rec.SCENARIO_BY_N[6], tmp_path / "bag").name + "__" in text
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


# ─────────── 收编：给缺 `session.json` 的包补一份（2026-09-29 现场，事后重建） ───────────
#
# ★ 现场第一手：收口后段原先不挡 Ctrl-C，操作者在那几秒里又按了一下 ⟹ 四份包里
#   **3 份的 `session.json` 没写成**（包本身是好的）。丢的正好是"这份包是什么、
#   缺没缺话题、sha256 是多少" ⟹ `MANIFEST.md` 只认 `session.json`，那三份整个隐形。

#: `bag/2026-09-29/s4-traj3-lemniscate.log` 里那两行**原文照抄**（`grep -a` 抄的，
#: 时间戳就是 `rosbag2` 自己报的 epoch 秒）。
_REAL_LOG_OPENED = ("[INFO] [1790667079.353559012] [rosbag2_storage]: Opened database "
                    "'/home/chen/ros_ws/bag/2026-09-29/s4-traj3-lemniscate/"
                    "s4-traj3-lemniscate_0.db3' for READ_WRITE.")
_REAL_LOG_STOPPED = "[INFO] [1790667119.462906380] [rosbag2_recorder]: Recording stopped"
_REAL_LOG_OPENED_EPOCH = 1790667079.353559012
_REAL_LOG_STOPPED_EPOCH = 1790667119.462906380


def test_从_rosbag2_自己的日志里读出起停时刻():
    """★ 这两个时刻是**包真的开始/停止写盘**的时刻（`rosbag2` 自己报的），
    比 `metadata.yaml` 的 `starting_time`（那是**第一条消息**的时刻）更贴近"起录"。"""
    got = rec.parse_record_log("\n".join(["别的行", _REAL_LOG_OPENED, "", _REAL_LOG_STOPPED]))
    assert got["opened_epoch"] == 1790667079.353559012
    assert got["stopped_epoch"] == 1790667119.462906380
    assert got["stopped_epoch"] - got["opened_epoch"] == pytest.approx(40.11, abs=0.01)


def test_日志里读不出时刻就写_None_不拿别的东西顶():
    """★ 本仓为"静默为空"交过三次学费：取不到就说取不到。**不许**拿"现在"或不完整的行
    冒充 —— 那会让重建出来的 `started_utc` 看起来是真的。"""
    for text in ("", "\n", "这根本不是 rosbag2 的日志\n",
                 "[1790667079.353559012] 少了正经的那一段头\n",
                 "[INFO] [1790667079.353559012] [rosbag2_recorder]: Recording st\n"):
        assert rec.parse_record_log(text) == {"opened_epoch": None, "stopped_epoch": None}, \
            f"凭空造出了数字：{text!r}"


def test_认得的那一行不要求写全路径():
    """★ 刻意**只认到 `Opened database` 为止**：我们要的是那一行的 epoch，不是路径
    （路径在包目录旁边、还随布局改过名）。认死整行的话，`rosbag2` 换个引号写法就全读不出来了。"""
    got = rec.parse_record_log("[INFO] [1790667079.353559012] [rosbag2_storage]: Opened database")
    assert got["opened_epoch"] == _REAL_LOG_OPENED_EPOCH


def _fake_adopt_env(monkeypatch, tmp_path, *, log=""):
    """`adopt` 的假环境：量包（`summarize`）换成记录下来的空跑，返回收编时**交给它**的东西。"""
    monkeypatch.setattr(rec, "BAG_ROOT", tmp_path / "bag")
    root = tmp_path / "bag"
    root.mkdir(parents=True, exist_ok=True)
    if log:
        (root / "4_traj3_lemniscate.log").write_text(log, encoding="utf-8")
    seen = []
    monkeypatch.setattr(rec, "summarize",
                        lambda out_dir, **kw: (seen.append((out_dir, kw)), {})[1])
    return root, seen


def test_收编只碰没有_session_json的包(tmp_path, monkeypatch):
    """★ 一条纪律：`session.json` 在 = 那份包**已经入账**了，而那是**收口那一刻的实测读数**
    —— 比事后重建可信 ⟹ 收编一个字节都不许碰它。"""
    root, seen = _fake_adopt_env(monkeypatch, tmp_path)
    (root / "1_static").mkdir()
    rec.write_state(_session(), root / "1_static" / "session.json")
    keep = (root / "1_static" / "session.json").read_bytes()
    (root / "2_manual_takeoff").mkdir()
    (root / "README.md").write_text("x", encoding="utf-8")      # 文件不是包目录，别管它
    assert rec.adopt(root=root) == 0
    assert (root / "1_static" / "session.json").read_bytes() == keep, "已入账的包被动了"
    assert [p.name for p, _ in seen] == ["2_manual_takeoff"]


def test_收编认不出场景的目录不动_并如实报出来(tmp_path, monkeypatch, capsys):
    """★ 目录名不是任何 `Scenario.slug` 时**不猜**（认错场景比不收编更坏：那份包会被
    贴上别人的场景名、别人的预期话题）。返回非零，并把它点名。"""
    root, seen = _fake_adopt_env(monkeypatch, tmp_path)
    (root / "随手建的东西").mkdir()
    assert rec.adopt(root=root) == 1
    assert seen == [], "认不出的目录不许量、更不许写"
    out = capsys.readouterr().out
    assert "随手建的东西" in out and "认不出" in out


def _adopt_log(opened_epoch: float, stopped_epoch=None) -> str:
    """一份最小可用的 `.log`（epoch 可指定）—— 用来验"那句来路的日期是从这份包现算的"。"""
    text = ("[INFO] [%.9f] [rosbag2_storage]: Opened database 'x' for READ_WRITE.\n"
            % opened_epoch)
    if stopped_epoch is not None:
        text += "[INFO] [%.9f] [rosbag2_recorder]: Recording stopped\n" % stopped_epoch
    return text


def test_来路里的日期是现算的不许写死(tmp_path, monkeypatch):
    """★ 2026-10-08 这一课：那句来路里原本**写死**了「2026-09-29 那次收口」，可 10-08 又收编过
    两份 —— 写死的日期会把它们也栽到 9-29 头上。来路那句话存在的全部意义就是**能回查**，
    **一句错的日期比没有日期更坏** ⟹ 日期只能从这份包自己的 `.log` 现算。"""
    opened = datetime.datetime(2026, 10, 8, 10, 18, 31).timestamp()
    root, seen = _fake_adopt_env(
        monkeypatch, tmp_path, log=_adopt_log(opened, opened + 49.6))
    (root / "4_traj3_lemniscate").mkdir()
    rec.adopt(root=root)
    text = seen[0][1]["state"]["reconstructed"]
    assert "2026-10-08" in text, f"来路没写这份包自己的日期：{text!r}"
    assert "2026-09-29" not in text, f"写死了别的日期 —— 这正是这次要修掉的那个毛病：{text!r}"
    assert "缺的是**收口之后**写 `session.json` 那一段" in text, \
        "`Recording stopped` 在 ⟹ 缺的是收口之后那一段，原因要按这条证据写"


def test_来路里缺的是收口那一段还是录到一半_按日志分开说(tmp_path, monkeypatch):
    """★ 两种缺法的**原因不一样**，不许共用一句话：`.log` 里有 `Recording stopped` = 录是录完了、
    缺的是收口之后写 `session.json`；**没有**它 = 录到一半就断了。同一句"收口后段被打断"
    会让后一种（包本身就是残的）看上去跟前者一样好。"""
    opened = datetime.datetime(2026, 10, 8, 10, 18, 31).timestamp()
    root, seen = _fake_adopt_env(monkeypatch, tmp_path, log=_adopt_log(opened))   # 没有 stopped
    (root / "4_traj3_lemniscate").mkdir()
    rec.adopt(root=root)
    text = seen[0][1]["state"]["reconstructed"]
    assert "录到一半就断了" in text, f"少了 `Recording stopped` 该这么说：{text!r}"
    assert "收口之后" not in text, f"这两句不许混用：{text!r}"


def test_归档目录按名字跳过_不算认不出(tmp_path, monkeypatch, capsys):
    """★ `bag/old/` 是**按目录名认出来的**归档目录（已被后一次录制取代的旧包）⟹ 跳过它
    **不算"认不出场景"**：只有它一个时返回码必须是 0（不是 1）。

    ⚠️ 返回码那一列**必须有鉴别力**：把跳过那三行删掉再跑，`old/` 会落进"认不出"那条路，
    而它**同时也返回 1** —— 所以只断言"有个 1"或"印出了 old"都验不出这条功能
    （★ 这条用例第一版就是这么写的，变异测试时没红，才改成现在这样）。"""
    root, seen = _fake_adopt_env(monkeypatch, tmp_path)
    (root / rec.ARCHIVE_DIRNAME / "3_traj1_hover").mkdir(parents=True)
    assert rec.adopt(root=root) == 0, (
        "归档目录不是「认不出场景」的目录 —— 只有它一个时，该收编的确实一个都没有 ⟹ 0")
    assert seen == [], "归档目录里的包不许量、更不许写"
    out = capsys.readouterr().out
    assert "跳过" in out, f"跳过归档目录这件事要说在屏幕上，否则像被静默吞了：{out!r}"


def test_归档目录不吞别的认不出的目录(tmp_path, monkeypatch, capsys):
    """★ 归档**只按那一个目录名**认，不许放宽成"凡认不出就归档"：那样拼错的目录名会被
    悄悄吞掉，而"认不出就报红、不动它"正是这一层的纪律（见旁边那条用例）。"""
    root, seen = _fake_adopt_env(monkeypatch, tmp_path)
    (root / rec.ARCHIVE_DIRNAME / "3_traj1_hover").mkdir(parents=True)
    (root / "随手建的东西").mkdir()
    assert rec.adopt(root=root) == 1, "归档目录**之外**的认不出的目录，照样要报红"
    assert seen == []
    out = capsys.readouterr().out
    assert "随手建的东西" in out and "认不出" in out, f"它该被点名：{out!r}"
    assert "3_traj1_hover" not in out, f"归档目录**里面**的东西不该被点名：{out!r}"


def test_收编的起录时刻取自日志里_rosbag2_自己报的那一行(tmp_path, monkeypatch):
    """★ 判据要**与本机时区无关**：拿日志里的 epoch 秒经 `utc_stamp` 换算，
    不许直接把 epoch 贴个 `Z`（本机恰好本地==UTC，那种写法在别的机器上就是错的）。"""
    root, seen = _fake_adopt_env(
        monkeypatch, tmp_path, log="\n".join([_REAL_LOG_OPENED, _REAL_LOG_STOPPED]) + "\n")
    (root / "4_traj3_lemniscate").mkdir()
    assert rec.adopt(root=root) == 0
    _path, kw = seen[0]
    state = kw["state"]
    # `dir` / `log` 走 `_rel`（相对仓库根）—— 用例跑在 `tmp_path` 里，所以是绝对路径
    # ★ 这份目录名是**老编号**（9-29 入库的那份八字包，那时它是场景 4）⟹ `scenario` 报
    #   **当前编号 5**，而 `slug` 如实写**目录名**（两者不一样是真的，不抹平）。
    assert state["scenario"] == 5, "老编号的目录要按 `SLUG_ALIASES` 认回当前的场景号"
    assert state["slug"] == "4_traj3_lemniscate", "`slug` 是落盘时那份目录的名字，如实记"
    assert state["dir"].endswith("bag/4_traj3_lemniscate")
    assert state["started_utc"] == rec.utc_stamp(
        datetime.datetime.fromtimestamp(_REAL_LOG_OPENED_EPOCH)), "起录时刻没落到那一步"
    assert kw["stopped_utc"] == rec.utc_stamp(
        datetime.datetime.fromtimestamp(_REAL_LOG_STOPPED_EPOCH)), "收口时刻没落到那一步"
    assert "反推" in kw["stop_signal"], \
        "收编出来的 `stop_signal` 要写明是**事后反推**的 —— 表里不能看上去像收口那一刻记的"
    assert "事后重建" in state["reconstructed"]
    # ★ 日志按**包目录名**找（与 `start` 同一把尺）：照 `scenario.slug`（现在是
    #   `5_traj3_lemniscate`）找会找不到旁边那份，于是跨度取不到、`stop_signal` 变成
    #   "看不出来" —— 而那正是**唯一**能把"到点"与"人按"分开的一手证据。
    assert state["log"].endswith("bag/4_traj3_lemniscate.log")


def test_收编取不到的字段一律写_null_不猜(tmp_path, monkeypatch):
    """★ `pid` / `git_head` / `git_dirty` / `ros_domain_id` 只在**起录那一刻**存在，
    包里没有它们的影子 ⟹ 一律 `null`。留一个"看着像真的"的值比留 null 坏得多
    （回查的人分不出哪个数是实测的）。"""
    root, seen = _fake_adopt_env(
        monkeypatch, tmp_path, log="\n".join([_REAL_LOG_OPENED, _REAL_LOG_STOPPED]) + "\n")
    (root / "4_traj3_lemniscate").mkdir()
    rec.adopt(root=root)
    state = seen[0][1]["state"]
    for key in rec.ADOPT_UNKNOWN:
        assert state[key] is None, f"`{key}` 取不到，不许拿别的东西顶"
    assert set(rec.ADOPT_UNKNOWN) == {"pid", "git_head", "git_dirty", "ros_domain_id"}


def test_收编时日志不在就说没有起录时刻_不是编一个(tmp_path, monkeypatch):
    """★ 日志被删/被移走时，`started_utc` 只能是 `None` —— 不许退回"现在"，
    否则重建出来的表会声称一份三天前的包是刚刚起录的。"""
    root, seen = _fake_adopt_env(monkeypatch, tmp_path)          # 不写 log
    (root / "4_traj3_lemniscate").mkdir()
    rec.adopt(root=root)
    state = seen[0][1]["state"]
    assert state["started_utc"] is None and state["started_local"] is None
    assert state["log"] is None
    assert "没有" in state["reconstructed"], "取不到起录时刻这件事要写在来路里"
    assert seen[0][1]["stopped_utc"] is None


def test_收编从日志跨度反推是怎么停的():
    """★ 这是**推导**不是猜：`expect_s` 是本仓自己定的倒计时（场景 1–5 统一 120 s），跨度是
    `rosbag2` 自己报的 ⟹ 跨度贴近 120 s = 到点自己收的，明显短一截 = 人按的 Ctrl-C。

    ★ 判据拿**有实测记录的那一份**校准：场景 1 的 `session.json` 是当时活着写下的，
      写着"录满到点，自动收口"，而它的 `.log` 跨度是 119.5 s ⟹ 这条判据必须把 119.5 判成"到点"。

    ★ 自由录（场景 6）**没有这条推导可做**：它本来就不该有"到点" ⟹ 只能说"人按的 Ctrl-C"，
      且**不许**印出"比到点应有的 ~None s 短一截"那种话（`None` 也不该被拿去做减法）。
    """
    one = rec.SCENARIO_BY_N[1]
    assert "到点" in rec.adopt_stop_signal(119.5, one), \
        "场景 1 实际就是到点收的（有当时活着写下的 session.json 作证），这条判据得把它判对"
    for span, want in ((74.6, "Ctrl-C"), (106.6, "Ctrl-C"), (40.1, "Ctrl-C")):
        got = rec.adopt_stop_signal(span, one)
        assert want in got and f"{span:.1f} s" in got, f"{span} s 该判成「{want}」：{got}"
    assert rec.adopt_stop_signal(None, one) == rec.ADOPT_STOP_SIGNAL, \
        "跨度取不到时要说「看不出来」，不许挑一个"

    free = rec.SCENARIO_BY_N[6]
    got = rec.adopt_stop_signal(300.0, free)
    assert "Ctrl-C" in got and "300.0 s" in got and "不倒计时" in got, got
    assert "None" not in got, f"说了句 `None` 出来 —— 那是没防住自由录：{got}"
    assert rec.adopt_stop_signal(None, free) == rec.ADOPT_STOP_SIGNAL, \
        "连跨度都没有时，照样只能说「看不出来」"


def test_收编的_stop_signal_写清是根据什么反推的(tmp_path, monkeypatch):
    """★ 推导得出的结论必须**带着依据**落地 —— 回查的人要能自己复核这条反推。"""
    root, seen = _fake_adopt_env(
        monkeypatch, tmp_path, log="\n".join([_REAL_LOG_OPENED, _REAL_LOG_STOPPED]) + "\n")
    (root / "4_traj3_lemniscate").mkdir()
    rec.adopt(root=root)
    _path, kw = seen[0]
    assert "40.1 s" in kw["stop_signal"], "跨度这个数要写在结论里：%r" % kw["stop_signal"]
    assert "Ctrl-C" in kw["stop_signal"], "40.1 s 远短于 120 s ⟹ 人按的"
    assert "40.1 s" in kw["state"]["reconstructed"], "来路那段也要写清跨度"


def test_收编会把清单重算(tmp_path, monkeypatch):
    """★ `MANIFEST.md` 只认 `session.json` ⟹ 补完必须重算，否则那几份包还是隐形。"""
    root, _seen = _fake_adopt_env(monkeypatch, tmp_path)
    (root / "2_manual_takeoff").mkdir()
    called = []
    monkeypatch.setattr(rec, "write_manifest", lambda *a, **k: called.append(a) or root)
    rec.adopt(root=root)
    assert called == [(root,)]


# ─────────────────────────── 清单（`bag/MANIFEST.md`） ───────────────────────────


def test_清单把每份包与缺了什么摊开(tmp_path):
    """★ 飞控日志归到**该场景自己的** `fcu/` 下（扁平布局），清单要能一并列出来。"""
    rec.write_state(_session(), tmp_path / "3_traj1_hover" / "session.json")
    rec.write_state(_session(scenario=4, title=rec.SCENARIO_BY_N[4].title,
                             dir="bag/4_traj3_lemniscate",
                             missing_expected=["/traj/progress"]),
                    tmp_path / "4_traj3_lemniscate" / "session.json")
    (tmp_path / "3_traj1_hover" / "fcu").mkdir(parents=True)
    (tmp_path / "3_traj1_hover" / "fcu" / "sess001-log001.ulg").write_bytes(b"ULOG" * 10)
    text = rec.render_manifest(tmp_path)
    assert "3_traj1_hover" in text and "4_traj3_lemniscate" in text
    assert "`deadbeef`" in text                      # sha256 要能被人回验
    assert "/traj/progress" in text                  # ★ 缺的那条点名，不是一句"基本通过"
    assert "sess001-log001.ulg" in text
    assert rec.ARCHIVE_DIRNAME not in text, "没有归档目录时别印那句说明"


def test_清单说清旧包去哪了(tmp_path):
    """★ 归档之后**同一个场景在磁盘上有两份、而本表只剩一份** —— 不写清楚，回查的人会以为
    某个场景的数据被删了。（清单只扫一层 ⟹ `bag/old/` 里的包天然进不了表，这是要的；
    这里管的是**别让人误读成"没了"**。）"""
    rec.write_state(_session(), tmp_path / "3_traj1_hover" / "session.json")
    archived = tmp_path / rec.ARCHIVE_DIRNAME / "3_traj1_hover"
    archived.mkdir(parents=True)
    rec.write_state(_session(dir=f"bag/{rec.ARCHIVE_DIRNAME}/3_traj1_hover"),
                    archived / "session.json")
    text = rec.render_manifest(tmp_path)
    assert "旧包" in text and rec.ARCHIVE_DIRNAME in text, f"得说清归档的旧包去哪了：{text!r}"
    # ★ 归档那份**自己**不许进表：表里出现它的 `dir` 就说明"只扫一层"这条被破坏了
    assert f"bag/{rec.ARCHIVE_DIRNAME}/3_traj1_hover" not in text


def test_一份包都没有时清单说实话(tmp_path):
    text = rec.render_manifest(tmp_path)
    assert "还没有收好口的包" in text
    assert "还没有 `.ulg`" in text
    assert "手工拷进那个场景自己的" in text, "空的时候要说清**怎么才能有**（飞控日志是手工动作）"
    assert "bag/<场景>/fcu/" in text


def test_清单是确定性输出_重算逐字相同(tmp_path):
    """★ 回归（2026-09-29 复核抓到）：原先表头带一行 `生成于 <now>`，
    而这张表是**入库**的 ⟹ 每录完一份就脏一次，验收那条"`rec_manifest` 重算得一致"
    用 `git diff` 永远对不上。起点时间在表里（`started_utc`），回查不缺东西。
    """
    day = tmp_path / "1_static"
    day.mkdir(parents=True)
    (day / "session.json").write_text(json.dumps({
        "scenario": 1, "title": "静止基线", "dir": "bag/1_static",
        "started_utc": "2026-09-29T01:00:00", "duration_s": 90.0, "size_text": "1.0 MB",
        "messages": 100, "topics": [], "sha256": "deadbeef", "missing_expected": [],
    }, ensure_ascii=False), encoding="utf-8")
    first = rec.render_manifest(tmp_path)
    assert first == rec.render_manifest(tmp_path), "同一份输入重算两次必须逐字相同"
    assert "生成于" not in first, "★ 入库的表里不许有时间戳 —— 它会让每次重算都脏"
    assert "2026-09-29T01:00:00" in first, "回查要靠每份包自己的起点时间"


def test_清单里零时长与没读到分开说(tmp_path):
    """同上那条，只是换到**入库的那张表**上（`MANIFEST.md`）：`?` 只能代表"没读到"。"""
    for slug, duration in (("1_static", 0.0), ("2_manual_takeoff", None)):
        day = tmp_path / slug
        day.mkdir(parents=True)
        (day / "session.json").write_text(json.dumps({
            "scenario": 1, "title": "静止基线", "dir": f"bag/{slug}",
            "started_utc": "2026-09-29T01:00:00", "duration_s": duration,
            "size_text": "1.0 MB", "messages": 0, "topics": [], "sha256": "",
            "missing_expected": [],
        }, ensure_ascii=False), encoding="utf-8")
    row_zero = [ln for ln in rec.render_manifest(tmp_path).splitlines()
                if "1_static" in ln]
    # 每个 slug 各一行；0.0 那种要写成 "0.0 s"，不能是 "?"
    assert any("| 0.0 s |" in ln for ln in row_zero), f"真的 0.0 s 被写成问号：{row_zero}"
    assert "| ? |" in rec.render_manifest(tmp_path), "没读到的才写问号"


def test_写清单会建目录(tmp_path, monkeypatch):
    monkeypatch.setattr(rec, "BAG_ROOT", tmp_path / "bag")
    path = rec.write_manifest()
    assert path.exists() and path.name == "MANIFEST.md"


# ─────────────────────── ★ 入库产物的完整性（别靠人眼） ───────────────────────


def test_参数存档与拷进_docs_的那份正文逐字相同():
    """★ 回归（2026-09-29 现场抓到，差点静默入库）：`docs/assets/z0.params` 是
    **中文头 + 那份从飞控导出的原文件**，而原文件**不是 UTF-8**
    （`# Vehicle:` 那行是 GBK 的"多旋翼"）。

    ⟹ 任何"把整个文件解码成字符串、改一处、再写回"的工具都会把那 6 个字节换成 6 个 U+FFFD，
    而 `file`（照样报 Non-ISO）、参数条数（照样 911）、UTF-8 校验（照样合法）**全都不报错**。
    当时就是这么坏的，隔了一次 `endswith` 复核才发现。这条用例把它钉在源码树上：
    **正文必须与 `bag/z0-params-2026-09-29.params` 逐字相同**。
    """
    repo = RUN_DIR.parent
    head = (repo / "docs/assets/z0.params").read_bytes()
    body = (repo / "bag/z0-params-2026-09-29.params").read_bytes()
    assert head.endswith(body), (
        "`docs/assets/z0.params` 的**正文**必须与 `bag/z0-params-2026-09-29.params` 逐字相同"
        "（后者是那份原文件的存档）—— 多半是被哪个编辑器按 UTF-8 往返了一遍，"
        "把那句 GBK 车名换成了 U+FFFD")
    assert b"# Vehicle: \xb6\xe0\xd0\xfd\xd2\xed" in body, (
        "GBK 的 `# Vehicle: 多旋翼` 是这份原文件的特征字节 —— 不许「顺手」改成 UTF-8")
    assert len(head) > len(body), "前面还要有那段中文头（只拷正文 = 出处没了）"
    prefix = head[:len(head) - len(body)]
    assert "真身" in prefix.decode("utf-8"), "中文头要在（它写着出处与那次 914→911 的差别）"
    assert b"\r\n" not in prefix, "★ 中文头是 LF —— 与正文的 CRLF 混着，这是**有意**的（正文逐字）"
    assert b"\xef\xbf\xbd" not in head, "U+FFFD 一个都不许有：有就是坏了（正文原样是 GBK）"


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
