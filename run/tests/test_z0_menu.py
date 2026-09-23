"""`run/z0.py` 的**纯函数**用例 —— 不需要 ROS、不需要 source（模块顶层不 import rclpy）。

跑法：

    python3 -m pytest run/tests/test_z0_menu.py -q

★ 这里测的是**有逻辑的那部分**（解析 / 判读 / 安全不变式），不是"把常量再抄一遍"。
现场那条真数据路径在 `test_z0_check_live.py`（对着台架夹具跑）。
"""

import pathlib
import sys
from types import SimpleNamespace

import pytest

RUN_DIR = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(RUN_DIR))

import z0  # noqa: E402  （路径注入了才 import 得到）


# ─────────────────────────── 串口占用 ───────────────────────────


def test_解析_lsof_字段模式():
    # 第一手格式（本机实测）：p<pid> / c<command> / f<fd>
    assert z0.parse_lsof_fpcf("p32607\ncpython3\nf3\n") == ["python3(32607)"]
    assert z0.parse_lsof_fpcf("") == []          # 没人占用时 lsof 什么都不输出
    assert z0.parse_lsof_fpcf("p1\ncmavros\nf3\nf4\nf5\n") == ["mavros(1)"]   # 多 fd 只算一个持有者
    assert z0.parse_lsof_fpcf("p1\ncmavros\nf3\np2\ncros2\nf9\n") == ["mavros(1)", "ros2(2)"]


# ─────────────────────────── 刚体名 ───────────────────────────


def test_刚体名从_vrpn_话题里取():
    topics = ["/vrpn/z0/pose", "/vrpn/wand/pose", "/mavros/state", "/vrpn/z0/accel"]
    assert z0.vrpn_body_names(topics) == ["wand", "z0"]


def test_大小写只差一个字母时要提醒():
    # ★ 2026-09-21 实测到 Z0/z0 两个值（mocap.yaml 文件头），这里保证不会静默选错
    note = z0.case_variant_note(["Z0", "z0"])
    assert note and "Z0" in note and "z0" in note
    assert z0.case_variant_note(["z0"]) is None
    assert z0.case_variant_note([]) is None


# ─────────────────────────── 域号 ───────────────────────────


def test_域号提示_0是真机_73是台架():
    assert "真机域" in z0.domain_note("0")
    assert "真机域" in z0.domain_note(None)      # 不设 = 默认 0
    bench = z0.domain_note("73")
    assert "台架" in bench and "不是真飞机" in bench
    assert "非默认域" in z0.domain_note("42")


def test_localhost_only_会写进提示():
    assert "只在本机发现" in z0.domain_note("73", "1")
    assert "只在本机发现" not in z0.domain_note("73", "0")


# ─────────────────────────── 消息压成一行 ───────────────────────────


def test_读出_state_的三个字段():
    msg = SimpleNamespace(connected=True, armed=False, mode="POSCTL")
    assert z0.describe_message(msg) == "connected=True armed=False mode=POSCTL"


def test_string_与_float32_走_data():
    assert z0.describe_message(SimpleNamespace(data="AUTO/TAKEOFF")) == "AUTO/TAKEOFF"
    assert z0.describe_message(SimpleNamespace(data=0.75)) == "0.75"


def test_posestamped_读位置():
    position = SimpleNamespace(x=0.12, y=-0.34, z=0.56)
    msg = SimpleNamespace(pose=SimpleNamespace(position=position))
    assert z0.describe_message(msg) == "x=+0.120 y=-0.340 z=+0.560"


def test_positiontarget_的位置在顶层():
    """★ 第一手：`mavros_msgs/msg/PositionTarget.msg:28` 是 `geometry_msgs/Point position`
    —— `/mavros/setpoint_raw/local` 的载荷**没有** `.pose`。只认 `.pose.position`
    的话这一行只会印出类名（等于没读）。"""
    msg = SimpleNamespace(position=SimpleNamespace(x=1.0, y=-2.0, z=0.5))
    assert z0.describe_message(msg) == "x=+1.000 y=-2.000 z=+0.500"


def test_认不出的消息不猜():
    assert z0.describe_message(SimpleNamespace(foo=1)) == "SimpleNamespace"


# ─────────────────────────── 帧率与表 ───────────────────────────


def test_无帧与有帧是两种说法():
    assert z0.rate_text(0, 4.0) == "无帧"          # 不是 "0 Hz" —— 现场含义不同
    assert z0.rate_text(800, 4.0) == "800 帧 / 4 s ≈ 200 Hz"


def test_全空的一张图不炸_并且指名缺什么():
    text = z0.render_snapshot({"window_s": 4.0, "domain": "0", "topics": {}, "counts": {},
                               "last": {}, "serial": {"path": "/dev/ttyTHS1", "exists": True,
                                                      "holders": []}})
    assert "缺什么" in text
    assert "/mavros/state" in text            # 缺的那条要**点名**，不是一句"没数据"
    assert "/traj/reference" in text          # 可选话题仍然列出来
    assert "✗ /traj/reference" not in text    # 但没帧不算错（·）


def test_读到读数时报_ok_并带上原文():
    probes = [topic for topic, _ in z0.PROBES]
    snap = {
        "window_s": 4.0, "domain": "73", "localhost_only": "1",
        "topics": {topic: "mavros_msgs/msg/State" for topic in probes},
        "counts": {topic: 4 for topic in probes},
        "last": {"/mavros/state": "connected=True armed=False mode=POSCTL"},
        "serial": {"path": "/dev/ttyTHS1", "exists": True, "holders": ["mavros(1)"]},
    }
    snap["counts"]["/mocap/pose_enu"] = 800
    text = z0.render_snapshot(snap)
    assert "✓ /mavros/state" in text
    assert "connected=True armed=False mode=POSCTL" in text      # 原文照抄，不改写
    assert "≈ 200 Hz" in text
    assert "台架" in text                                        # 域号那句必须显眼
    assert "缺什么" not in text                                  # 齐了就不该再报缺
    assert "选轨迹" in text                                      # 但"下一步"仍要说


def test_缺的话题要逐个点名():
    snap = {
        "window_s": 4.0, "domain": "0", "localhost_only": None,
        "topics": {"/mavros/state": "mavros_msgs/msg/State"},      # 只发了一个，其余不在图里
        "counts": {"/mavros/state": 4},
        "last": {}, "serial": {"path": "/dev/ttyTHS1", "exists": True, "holders": []},
    }
    text = z0.render_snapshot(snap)
    assert "缺什么" in text
    assert "发布者没起" in text                                   # 不在图里 ⟹ 说得比"没数据"具体
    assert "/mocap/pose_enu" in text


def test_串口被占要说清代价():
    snap = {"window_s": 4.0, "domain": "0", "topics": {}, "counts": {}, "last": {},
            "serial": {"path": "/dev/ttyTHS1", "exists": True, "holders": ["mavros(1)"]}}
    text = z0.render_snapshot(snap)
    assert "被占用" in text and "mavros(1)" in text
    assert "长得像 launch 坏了" in text


# ─────────────────────────── ★ 安全不变式 ───────────────────────────


def test_动作表里没有任何会动飞机的入口():
    violations = z0.forbidden_commands(z0.ACTIONS)
    assert violations == [], f"动作表里出现了不该有的命令：{violations}"
    # 判据的两条设计依据要在文件里留着（改这些命令前先读它们）
    assert all(a.cmd is None or a.cmd.startswith("ros2 ") for a in z0.ACTIONS)


def test_源码里不出现发布与服务调用():
    """★ 走 AST 而不是字符串搜索：**守卫表自己**要写出那些字样（`FORBIDDEN_IN_COMMANDS`），
    字符串搜索会把那张表本身当成违规 —— 于是判据要么放水，要么逼人去拼字符串绕过它。"""
    import ast

    tree = ast.parse((RUN_DIR / "z0.py").read_text(encoding="utf-8"))
    literals = [node.value for node in ast.walk(tree)
                if isinstance(node, ast.Constant) and isinstance(node.value, str)]
    for token in z0.FORBIDDEN_IN_COMMANDS:
        hits = {text for text in literals if token in text.lower()} - {token}
        assert not hits, f"`{token}` 出现在了守卫表之外：{hits}"

    attributes = {node.attr for node in ast.walk(tree) if isinstance(node, ast.Attribute)}
    forbidden_api = {"create_publisher", "create_service", "create_client"}
    assert not (forbidden_api & attributes), \
        f"本脚本只许**订阅**：{forbidden_api & attributes}"


def test_每条起链命令都带_source_前缀():
    command = z0.sourced_command("ros2 launch mocap mocap.launch.py")
    assert command.endswith("ros2 launch mocap mocap.launch.py")
    if z0.ROS_SETUP.exists():                     # 本机有 ROS：那它必须在前缀里
        assert f"source {z0.ROS_SETUP}" in command
        assert z0.existing_setups(), "一条 setup 都没有 ⟹ `env` 会报出来，不该静默"
    # 不存在的 setup **不能**被写进命令（否则 `source` 直接失败，看上去像脚本坏了）
    command = z0.sourced_command("true", setups=[pathlib.Path("/nope/setup.bash")])
    assert command == "source /nope/setup.bash && true"


def test_菜单与_动作表对得上():
    text = z0.menu_text()
    for action in z0.ACTIONS:
        assert action.title in text
    assert text.index("1  环境自检") < text.index("0  退出")   # 数字与动作表的顺序一致
    assert z0.ACTION_BY_KEY["mocap"].confirm          # 起 mocap 必须要求确认
    assert z0.ACTION_BY_KEY["bringup"].confirm        # bringup 里有 mocap ⟹ 也要
    assert z0.ACTION_BY_KEY["mavros"].warn_serial     # mavros 起之前先看串口


# ─────────────────────────── 命令行拼装（--dry-run / 可选参数） ───────────────────────────


def test_list_只列菜单且返回_0(capsys):
    assert z0.main(["--list"]) == 0
    out = capsys.readouterr().out
    for action in z0.ACTIONS:
        assert action.title in out


def test_dry_run_只打印不执行(capsys, monkeypatch):
    def boom(_cmd):                      # 被执行到就该炸
        raise AssertionError("--dry-run 执行了命令")

    monkeypatch.setattr(z0, "run_foreground", boom)
    assert z0.main(["--dry-run", "mocap"]) == 0
    out = capsys.readouterr().out
    assert "ros2 launch mocap mocap.launch.py" in out


def test_dry_run_也要把可选参数亮出来(capsys):
    assert z0.main(["--dry-run", "mavros"]) == 0
    assert "gcs_url" in capsys.readouterr().out       # 不亮 ⟹ 现场不知道它存在


def test_dry_run_对只读动作也是不执行(capsys, monkeypatch):
    """★ 判据（ticket 22 §三.1）说的是"只打印命令、**不执行**" —— 只读动作也不例外，
    否则 `--dry-run check` 会真起一个 5 s 的探测。"""
    ran = []
    monkeypatch.setattr(z0, "cmd_check", lambda: ran.append("check") or 0)
    assert z0.main(["--dry-run", "check"]) == 0
    assert ran == []
    assert "只读" in capsys.readouterr().out


def test_可选参数只有答_y_才追加():
    mavros = z0.ACTION_BY_KEY["mavros"]
    assert z0.optionals_suffix(mavros, [False]) == ""              # 回车 ⟹ 一个都不加
    assert z0.optionals_suffix(mavros, [True]) == f" gcs_url:={z0.GCS_URL}"
    assert z0.optionals_suffix(z0.ACTION_BY_KEY["mocap"], []) == ""  # 没有可选参数的动作
    assert z0.ACTION_BY_KEY["bringup"].optionals[0].suffix == "use_vrpn:=false"


def test_答_y_之后命令尾巴真的进了_bash_c(capsys, monkeypatch):
    seen = []
    monkeypatch.setattr(z0, "_serial_holders", lambda _path: [])       # 别去动真的串口
    monkeypatch.setattr(z0, "ask_confirm", lambda _q: True)
    monkeypatch.setattr(z0, "run_foreground", lambda cmd: seen.append(cmd) or 0)
    assert z0.launch_action(z0.ACTION_BY_KEY["mavros"]) == 0
    assert seen == [f"ros2 launch mavros px4.launch fcu_url:={z0.FCU_URL} gcs_url:={z0.GCS_URL}"]


def test_可选参数的值也受安全不变式扫查():
    """可选参数的 `suffix` 与主命令走同一条 `bash -c` ⟹ 必须一起扫。"""
    assert z0.forbidden_commands(z0.ACTIONS) == []
    forged = z0.Action("伪", "伪动作", None, "伪",
                       optionals=(z0.OptionalArg("伪参数", "问一句", "&& ros2 service call /x"),))
    assert z0.forbidden_commands([forged]) != []


# ─────────────────────────── 一键起链（ticket 22 §七） ───────────────────────────


def test_起链的顺序就是_runbook_那条():
    assert z0.UP_CHAIN == ("mavros", "vrpn", "mocap", "traj", "control")
    assert set(z0.UP_CHAIN) <= set(z0.ACTION_BY_KEY), "起链里出现了动作表里没有的东西"
    assert set(z0.UP_OPTIONALS) <= set(z0.ACTION_BY_KEY)
    for key, names in z0.UP_OPTIONALS.items():
        have = {opt.name for opt in z0.ACTION_BY_KEY[key].optionals}
        assert set(names) <= have, f"{key} 默认要带的可选参数不在它的表项里：{set(names) - have}"


def test_起链里没有会动飞机的入口():
    """★ `up` 起的命令**全部**来自 `ACTIONS`（它只写 key）⟹ 22 号票那条不变式盖住它。"""
    assert z0.forbidden_commands([z0.ACTION_BY_KEY[k] for k in z0.UP_CHAIN]) == []
    assert "control" in z0.UP_CHAIN          # 控制节点在链上，但**起飞**不在（下面那条）
    assert z0.ACTION_BY_KEY["up"].cmd is None
    assert not z0.ACTION_BY_KEY["up"].read_only, \
        "`up` 会起进程，不能算只读 —— 否则 `--dry-run` 与核对表都会撒谎"


def test_每一步要么有起来的判据要么明说不等():
    for key in z0.UP_CHAIN:
        if key == "traj":
            assert key not in z0.STEP_READY and z0.STEP_TOPIC[key] is None
            assert "不等" in z0.wait_hint(key)
        else:
            assert key in z0.STEP_READY and key in z0.STEP_TIMEOUT_S
    assert set(z0.STEP_READY) <= set(z0.UP_CHAIN)


def test_mavros_的判据是_connected_不是有帧():
    """★ 第一手：`/mavros/state` 的 1 Hz 心跳在飞控链路通之前就在发 —— **有帧 ≠ 连上了**。"""
    ready = z0.STEP_READY["mavros"]
    assert not ready({"counts": {"/mavros/state": 3},
                      "last": {"/mavros/state": "connected=False armed=False"}})
    assert ready({"counts": {"/mavros/state": 1},
                  "last": {"/mavros/state": "connected=True armed=False mode=POSCTL"}})
    assert not ready({"counts": {}, "last": {}})


def test_动捕判据认任意刚体名():
    """刚体名不做归一化（`Z0`/`z0` 都出现过）⟹ 有**任何一个**在发就算起来。"""
    ready = z0.STEP_READY["vrpn"]
    assert ready({"counts": {"/vrpn/Z0/pose": 40}})
    assert ready({"counts": {"/vrpn/z0/pose": 1}})
    assert not ready({"counts": {"/vrpn/Z0/pose": 0, "/mocap/pose_enu": 100}})


def test_mocap_与_control_的判据看帧数():
    assert z0.STEP_READY["mocap"]({"counts": {"/mocap/pose_enu": 12}})
    assert not z0.STEP_READY["mocap"]({"counts": {"/mocap/pose_enu": 0}})
    assert z0.STEP_READY["control"]({"counts": {"/control/state": 3}})
    assert not z0.STEP_READY["control"]({"counts": {}})


def test_dry_run_up_把顺序与判据全摊开_且一条都不起(capsys, monkeypatch):
    def boom(*_args, **_kwargs):
        raise AssertionError("`--dry-run up` 起了东西")

    monkeypatch.setattr(z0, "start_child", boom)
    monkeypatch.setattr(z0, "cmd_up", boom)
    assert z0.main(["--dry-run", "up"]) == 0
    out = capsys.readouterr().out
    for key in z0.UP_CHAIN:
        assert z0.ACTION_BY_KEY[key].cmd in out, f"{key} 的命令没摊出来"
    assert "gcs_url" in out            # ★ 默认带上的那条要看得见（否则现场不知道它在）
    assert "connected=True" in out     # 判据也摊开
    assert "遥控器开始键" in out        # 还差的那两步写清楚了
    assert "飞机已摆好" in out          # `mocap` 那条人工确认


# ── 编排（把外部世界全换成假的：探测 / 起进程 / 等 / 确认 / 日志目录）──


@pytest.fixture
def fake_child():
    class FakeChild:
        def __init__(self, key, cmd, log):
            self.key, self.cmd, self.log = key, cmd, log
            self.running, self.stopped, self.closed = True, False, False

        def alive(self):
            return self.running

        def tail(self, lines=12):
            return f"      （{self.key} 的假日志尾）"

        def stop(self, grace_s=5.0):
            self.stopped, self.running = True, False

        def close_log(self):
            self.closed = True

    return FakeChild


@pytest.fixture
def up_world(monkeypatch, tmp_path, fake_child):
    """把 `cmd_up` 的外部世界全换成假的；返回 (喂探测的快照表, 起过的子进程表)。"""
    snapshots: list = [{}]
    started: list = []

    monkeypatch.setattr(z0, "ensure_sourced", lambda: None)     # 否则会真的带 source 重 exec
    monkeypatch.setattr(z0, "RUN_DIR", tmp_path)
    monkeypatch.setattr(z0, "Probe", lambda *a, **k: _Probe(snapshots))
    monkeypatch.setattr(z0, "watch_children", lambda *a, **k: 0)

    def start_child(key, log_path):
        child = fake_child(key, "假命令", log_path)
        started.append(child)
        return child

    monkeypatch.setattr(z0, "start_child", start_child)
    monkeypatch.setattr(z0, "ask_confirm", lambda *a, **k: True)
    monkeypatch.setattr(z0, "render_snapshot", lambda snap: "（假核对表）")
    return snapshots, started


class _Probe:
    """假探测：按脚本喂的快照依次返回，用完之后一直返回最后一个。"""

    def __init__(self, snapshots):
        self._snapshots, self._calls, self.closed = snapshots, 0, False

    def snapshot(self, window_s=4.0):
        self._calls += 1
        return self._snapshots[min(self._calls - 1, len(self._snapshots) - 1)]

    def close(self):
        self.closed = True


def test_一键起链按顺序起五条_每步都等到判据(up_world, monkeypatch):
    snapshots, started = up_world
    waited = []
    monkeypatch.setattr(z0, "wait_for_step",
                        lambda probe, key, **k: (waited.append(key), (True, {}))[1])

    assert z0.cmd_up() == 0
    assert [c.key for c in started] == list(z0.UP_CHAIN)
    assert [c.log.name for c in started] == ["01-mavros.log", "02-vrpn.log", "03-mocap.log",
                                             "04-traj.log", "05-control.log"]
    # `traj` 没有判据（没 select 就一帧不发）⟹ 不空等它
    assert waited == ["mavros", "vrpn", "mocap", "control"]


def test_mavros_已经在跑就问沿用_并且收尾不停它(up_world, monkeypatch, capsys):
    snapshots, started = up_world
    snapshots[0] = {"counts": {"/mavros/state": 1},      # 第一口探测就看到 mavros 在跑
                    "last": {"/mavros/state": "connected=True armed=False mode=POSCTL"}}
    monkeypatch.setattr(z0, "wait_for_step", lambda *a, **k: (True, {}))

    assert z0.cmd_up() == 0
    assert [c.key for c in started] == ["vrpn", "mocap", "traj", "control"]   # 没有第二个 mavros
    assert "沿用" in capsys.readouterr().out
    # ★ 收尾只停**本脚本起的**：别人的 mavros 根本不在 `started` 里，也就停不到它
    assert all(c.log.name != "01-mavros.log" for c in started)


def test_某一步超时_打日志尾巴_停掉其余的_返回_1(up_world, monkeypatch, capsys):
    snapshots, started = up_world
    monkeypatch.setattr(z0, "wait_for_step",
                        lambda probe, key, **k: (key != "mocap", {}))   # 卡在 mocap

    assert z0.cmd_up() == 1
    out = capsys.readouterr().out
    assert "等超时" in out and "假日志尾" in out
    assert [c.key for c in started] == ["mavros", "vrpn", "mocap"]      # 没往下起 traj/control
    assert all(c.stopped for c in started), "中断后不能留下任何子进程（尤其是 mavros）"
    assert all(c.closed for c in started)


def test_子进程半路退了_不当成超时(up_world, monkeypatch, capsys):
    snapshots, started = up_world

    def wait(probe, key, *, alive=None, **k):
        started[-1].running = False        # 模拟它自己退了
        assert alive is not None and not alive()
        return False, {}

    monkeypatch.setattr(z0, "wait_for_step", wait)
    assert z0.cmd_up() == 1
    assert "子进程自己退了" in capsys.readouterr().out


def test_守着时某个节点自己退了要报出来(fake_child, monkeypatch, capsys, tmp_path):
    class Dying(fake_child):
        def __init__(self):
            super().__init__("control", "假命令", tmp_path / "05-control.log")
            self.calls = 0

        def alive(self):
            self.calls += 1
            return self.calls < 3          # 第三次问的时候它已经没了

    assert z0.watch_children([Dying()], interval_s=0.01) == 1
    assert "自己退了" in capsys.readouterr().out


def test_串口被占时先问一句(monkeypatch, capsys):
    monkeypatch.setattr(z0, "_serial_holders", lambda path: ["mavros(1)"])
    monkeypatch.setattr(z0, "ask_confirm", lambda *a, **k: False)
    assert z0.serial_gate(z0.ACTION_BY_KEY["mavros"]) is False
    assert "被占用" in capsys.readouterr().out
    assert z0.serial_gate(z0.ACTION_BY_KEY["mocap"]) is True     # 不起串口的动作不问


def test_日志尾巴读的是最后几行(tmp_path):
    log = tmp_path / "x.log"
    log.write_text("\n".join(f"第{i}行" for i in range(1, 21)), encoding="utf-8")
    child = z0.Child(key="k", cmd="c", proc=None, log=log, fh=None)
    tail = child.tail(lines=3)
    assert "第20行" in tail and "第18行" in tail and "第17行" not in tail
