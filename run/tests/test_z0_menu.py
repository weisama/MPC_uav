"""`run/z0.py` 的**纯函数**用例 —— 不需要 ROS、不需要 source（模块顶层不 import rclpy）。

跑法：

    python3 -m pytest run/tests/test_z0_menu.py -q

★ 这里测的是**有逻辑的那部分**（解析 / 判读 / 安全不变式），不是"把常量再抄一遍"。
现场那条真数据路径在 `test_z0_check_live.py`（对着台架夹具跑）。
"""

import os
import pathlib
import re
import signal
import subprocess
import sys
import time
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
    # ★ 2026-10-17：`mpc` 那条前面挂了 `env ACADOS_SOURCE_DIR=… LD_LIBRARY_PATH=…`
    #   （缺它起链死在第 5 步）⟹ 这里认「`env` 只设变量、然后仍以 `ros2` 走」这一种前缀。
    #   ⛔ **不要把它放宽成"随便什么前缀"** —— 这条断言的用处就是"命令的出口只有 ros2"。
    for action in z0.ACTIONS:
        cmd = action.cmd
        if cmd is None or cmd.startswith("ros2 "):
            continue
        assert cmd.startswith("env "), f"{action.key}: 命令的出口不是 ros2：{cmd}"
        head, sep, tail = cmd[4:].partition(" ros2 ")
        assert sep, f"{action.key}: `env` 前缀后面不是 ros2：{cmd}"
        for word in head.split():
            assert "=" in word, f"{action.key}: `env` 前缀里混进了非赋值项 {word!r}：{cmd}"


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


# ─────────────── 术语纪律（`CONTEXT.md` 第一节「最危险的一个歧义」） ───────────────

#: ★ 2026-10-18 起术语规则反转（`CONTEXT.md`）：正文一律写 `MPC`，不再写旧全称。
#:   `run/tests/test_safety_paths_drill.py` 里有一份同样的守卫。
RE_OLD_TERM = re.compile("模型预测控制")
#: 唯一豁免：代码里的启动日志字面串，它是运行时判据（`run/z0.py` 的 `STEP_LOG_READY`）。
OLD_TERM_ALLOW = "MPC（模型预测控制）节点启动："

#: ★ **改到哪就守到哪**：这三个文件是操作者现场直接读的措辞面。
TERM_DISCIPLINE_FILES = ("run/z0.py", "run/README.md", "operation_manual.md")


def test_三个文件里不用旧术语_模型预测控制():
    """★ PX4 的 `MPC_*` 是**多旋翼位置控制器**（级联 PID）的参数，与本项目 MPC 无关
    （`CONTEXT.md` 第一节）⟹ 正文一律写 `MPC`，不再写旧全称。

    ★ 代价很低、收益很实：漏了不会让飞机摔，但会让"这句话说的是哪个 MPC"每年重问一次。
    """
    hits = []
    for rel in TERM_DISCIPLINE_FILES:
        path = RUN_DIR.parent / rel
        for number, line in enumerate(path.read_text(encoding="utf-8").split("\n"), start=1):
            if RE_OLD_TERM.search(line) and OLD_TERM_ALLOW not in line:
                hits.append(f"{rel}:{number}：{line.strip()}")
    assert not hits, ("正文出现旧术语「模型预测控制」，应统一写 `MPC`：\n  "
                      + "\n  ".join(hits))


def test_每条起链命令都带_source_前缀():
    command = z0.sourced_command("ros2 launch mocap mocap.launch.py")
    assert command.endswith("ros2 launch mocap mocap.launch.py")
    if z0.ROS_SETUP.exists():                     # 本机有 ROS：那它必须在前缀里
        assert f"source {z0.ROS_SETUP}" in command
        assert z0.existing_setups(), "一条 setup 都没有 ⟹ `env` 会报出来，不该静默"
    # 不存在的 setup **不能**被写进命令（否则 `source` 直接失败，看上去像脚本坏了）
    command = z0.sourced_command("true", setups=[pathlib.Path("/nope/setup.bash")])
    assert command == "source /nope/setup.bash && true"


def test_source_重入等子进程时不动_SIGINT_处置(monkeypatch):
    """★ 回归第一层（2026-10-17 现场：一次 Ctrl-C 就丢掉了一份 `session.json`）。

    机制：Ctrl-C 发给**整个前台进程组** ⟹ 外壳（`ensure_sourced` 里等重开菜单的那个
    进程）与菜单进程同时收到。Python 3 的 `subprocess.call` 被 `KeyboardInterrupt`
    打断时会 `p.kill()`（SIGKILL）子进程（3.10.12 `subprocess.py:347`）⟹ 菜单进程
    在"收口中"那几个字刚打印完就被打死，`ros2 bag info` / `session.json` 全没跑。

    ★ 修法**不许**是"先 `SIG_IGN` 再起子进程"——那会被子进程继承（见下一条用例）。
    这里钉住：等待期间**不改任何信号处置**；`wait()` 被 `KeyboardInterrupt` 打断后
    要**接着等**（而不是走掉），返回值照旧透传。
    """
    seen = {"handlers": [], "waits": 0}

    class FakeProc:
        def wait(self):
            seen["waits"] += 1
            seen["handlers"].append(signal.getsignal(signal.SIGINT))
            if seen["waits"] == 1:
                raise KeyboardInterrupt
            return 7

    monkeypatch.setattr(z0.subprocess, "Popen", lambda argv, env=None: FakeProc())
    before = signal.getsignal(signal.SIGINT)
    assert z0.call_ignoring_sigint(["bash", "-c", "true"]) == 7
    assert seen["waits"] == 2, "被打断之后必须接着等，不能走"
    assert all(h is before for h in seen["handlers"]), "等待期间不许改 SIGINT 处置"
    assert signal.getsignal(signal.SIGINT) is before


def test_source_重入的子进程不能继承_SIGINT_忽略(tmp_path):
    """★ 回归第二层（2026-10-17 当天现场：录制挂着，按 Ctrl-C **毫无反应**）。

    上一版修法在起子进程**之前**把 SIGINT 设成 `SIG_IGN`。被忽略的信号会被
    `fork`+`exec` **继承**，而 CPython 只有当启动时 SIGINT 不是 `SIG_IGN` 才装
    `KeyboardInterrupt` 处理器 ⟹ 内层菜单从启动起就永远看不见 Ctrl-C。
    现场一手：`/proc/<内层 pid>/status` 的 `SigIgn` 带 `…0002`（SIGINT 被忽略）、
    `SigCgt` 是 0（连处理器都没装）。

    ⟹ 让子进程自己把"启动时 SIGINT 是否被忽略"写出来：必须是 `0`。
    """
    seen = tmp_path / "sigint_ignored.txt"
    child = tmp_path / "child.py"
    child.write_text(
        "import signal\n"
        f"open({str(seen)!r}, 'w').write("
        "'1' if signal.getsignal(signal.SIGINT) is signal.SIG_IGN else '0')\n",
        encoding="utf-8")
    helper = tmp_path / "helper.py"
    helper.write_text(
        "import sys\n"
        f"sys.path.insert(0, {str(RUN_DIR)!r})\n"
        "import z0\n"
        f"sys.exit(z0.call_ignoring_sigint([sys.executable, {str(child)!r}]))\n",
        encoding="utf-8")

    rc = subprocess.run([sys.executable, str(helper)], timeout=30).returncode
    assert rc == 0
    assert seen.exists(), "子进程没跑起来 ⟹ 这条用例没测到东西"
    assert seen.read_text() == "0", (
        "子进程继承了 SIGINT=SIG_IGN ⟹ 它里面的 Python 不会再装 KeyboardInterrupt，"
        "Ctrl-C 永远是死的（2026-10-17 现场就是这么挂住的）")


def test_一次_SIGINT_不能打死正在等它的子进程(tmp_path):
    """端到端：外壳只收到**一次** SIGINT 时，它等着的子进程必须**还活着**。

    ⛔ 反面就是裸 `subprocess.call` —— 子进程会被 SIGKILL 掉（本机复现过：child 直接消失，
    外层退出码 130）。这里把 SIGINT **只发给外壳**（不是发给进程组），这样"子进程死了"
    就只可能是外壳干的，不可能是终端那一下顺手带走的。
    """
    child = tmp_path / "child.py"
    pid_file = tmp_path / "child.pid"
    child.write_text(
        "import os, time\n"
        f"open({str(pid_file)!r}, 'w').write(str(os.getpid()))\n"
        "time.sleep(30)\n", encoding="utf-8")
    helper = tmp_path / "helper.py"
    helper.write_text(
        "import sys\n"
        f"sys.path.insert(0, {str(RUN_DIR)!r})\n"
        "import z0\n"
        f"sys.exit(z0.call_ignoring_sigint([sys.executable, {str(child)!r}]))\n",
        encoding="utf-8")

    proc = subprocess.Popen([sys.executable, str(helper)], start_new_session=True)
    try:
        deadline = time.monotonic() + 10
        while not pid_file.exists() and time.monotonic() < deadline:
            time.sleep(0.02)
        assert pid_file.exists(), "子进程没起来 ⟹ 这条用例没测到东西"
        child_pid = int(pid_file.read_text())

        os.kill(proc.pid, signal.SIGINT)        # ★ 只发外壳：等于"一次 Ctrl-C"
        time.sleep(0.5)
        try:
            os.kill(child_pid, 0)
            alive = True
        except ProcessLookupError:
            alive = False
        assert alive, "外壳把正在等它的子进程打死了 ⟹ 菜单进程也会这样死在收口途中"

        os.kill(child_pid, signal.SIGKILL)      # 收尾；返回码因此非 0
        assert proc.wait(timeout=10) != 0
    finally:
        if proc.poll() is None:
            proc.kill()


def _foreground_helper(tmp_path, child_source: str, argv_extra: str = ""):
    """起一个"外壳 → `z0.run_foreground`"的假前台，返回 (helper 进程, 子进程 pid 文件)。"""
    child = tmp_path / "child.py"
    pid_file = tmp_path / "child.pid"
    child.write_text(child_source.replace("PIDFILE", repr(str(pid_file))), encoding="utf-8")
    helper = tmp_path / "helper.py"
    helper.write_text(
        "import sys\n"
        f"sys.path.insert(0, {str(RUN_DIR)!r})\n"
        "import z0\n"
        f"{argv_extra}"
        f"print('rc=%d' % z0.run_foreground(f'\"{{sys.executable}}\" \"{child}\"'), flush=True)\n",
        encoding="utf-8")
    proc = subprocess.Popen([sys.executable, str(helper)], start_new_session=True,
                            stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    deadline = time.monotonic() + 15
    while not pid_file.exists() and time.monotonic() < deadline:
        time.sleep(0.02)
    assert pid_file.exists(), "前台子进程没起来 ⟹ 这条用例没测到东西"
    return proc, int(pid_file.read_text())


def test_前台_Ctrl_C_不立刻_SIGKILL_子进程(tmp_path):
    """前台动作（launch / `fly.py`）也**不能**被 `subprocess.call` 的 `p.kill()` 带走。

    ★ 修之前：Ctrl-C ⟹ `subprocess.call` 里的 `except: p.kill()` ⟹ 子进程被 SIGKILL，
      它自己的收摊逻辑（`fly.py` 的 `rclpy.shutdown()`、launch 的关串口）根本没机会跑。
    ★ 现在：只等；子进程自己退（2 s 后退 0），外壳把它的返回码交回来。
    """
    proc, child_pid = _foreground_helper(
        tmp_path, "import os, time\nopen(PIDFILE, 'w').write(str(os.getpid()))\ntime.sleep(2)\n")
    try:
        os.kill(proc.pid, signal.SIGINT)        # ★ 只发外壳
        time.sleep(0.5)
        try:
            os.kill(child_pid, 0)
            alive = True
        except ProcessLookupError:
            alive = False
        assert alive, "Ctrl-C 把前台子进程直接 SIGKILL 了 ⟹ 它没机会自己收摊"
        out, err = proc.communicate(timeout=15)
        assert proc.returncode == 0, err
        assert "rc=0" in out, out
    finally:
        if proc.poll() is None:
            proc.kill()


def test_前台_Ctrl_C_等不到就升级_TERM_再_KILL(tmp_path):
    """子进程**不理** SIGINT 时不能永远等：`FOREGROUND_GRACE_S` 到点 ⟹ SIGTERM → SIGKILL。

    ★ 这一条同时钉住"升级只发给直接子进程"之前的那一步：只要 SIGTERM 能收掉它，
      就用不到 SIGKILL（读数 `rc=-15`）。
    """
    proc, child_pid = _foreground_helper(
        tmp_path,
        "import os, signal, time\n"
        "signal.signal(signal.SIGINT, signal.SIG_IGN)\n"
        "open(PIDFILE, 'w').write(str(os.getpid()))\n"
        "time.sleep(30)\n",
        argv_extra="z0.FOREGROUND_GRACE_S = 0.3\n")
    try:
        os.kill(proc.pid, signal.SIGINT)        # ★ 只发外壳
        out, err = proc.communicate(timeout=15)
        assert proc.returncode == 0, err
        assert "rc=-15" in out, out             # SIGTERM 收掉的（不是 SIGKILL 的 -9）
        try:
            os.kill(child_pid, 0)
            alive = True
        except ProcessLookupError:
            alive = False
        assert not alive, "升级之后子进程还活着"
    finally:
        if proc.poll() is None:
            proc.kill()


def test_菜单与_动作表对得上():
    text = z0.menu_text()
    for action in z0.MENU_ACTIONS:
        assert action.title in text
    assert z0.MENU_ACTIONS == z0.ACTIONS[:len(z0.MENU_ACTIONS)], \
        "菜单项应当是动作表的**前若干条**（否则菜单号与表里顺序会各说各话）"
    assert text.index("1  环境自检") < text.index("0  退出")   # 数字与动作表的顺序一致
    assert "操作台 fly.py" not in text, "`fly` 不进菜单（它是一键流程里那一步，`menu=False`）"
    assert z0.ACTION_BY_KEY["mocap"].confirm          # 起 mocap 必须要求确认
    assert z0.ACTION_BY_KEY["mavros"].warn_serial     # mavros 起之前先看串口
    # ★ 一键那两项要覆盖 mocap（`confirm` 由 `start_and_wait` 逐步照做，不是在这里另判）
    for key in ("up", "up_manual"):
        assert z0.ACTION_BY_KEY[key].cmd is None and not z0.ACTION_BY_KEY[key].read_only


# ─────────────────────────── 命令行拼装（--dry-run / 可选参数） ───────────────────────────


def test_list_只列菜单且返回_0(capsys):
    assert z0.main(["--list"]) == 0
    out = capsys.readouterr().out
    for action in z0.MENU_ACTIONS:
        assert action.title in out


def test_进菜单前先把_source_重入走完(monkeypatch):
    """★ 回归（2026-09-23 现场反馈：**数字要敲第二次才认**）。

    `ensure_sourced()` 是"把本进程 `exec` 掉重开"。菜单是**先读完一行再派发**的 ——
    重入若发生在派发里，用户已经敲进缓冲区的那一行就随旧进程一起没了、菜单重印一遍。
    第一手：未 source 的 shell 里 `printf "1\\nq\\n" | python3 run/z0.py` ⟹ 菜单印**两遍**、
    第一行凭空消失。⟹ 重入必须排在 `input()` **之前**（即 `menu_loop` 之前）。
    """
    calls = []
    monkeypatch.setattr(z0, "ensure_sourced", lambda: calls.append("ensure"))
    monkeypatch.setattr(z0, "menu_loop", lambda: calls.append("menu") or 0)
    assert z0.main([]) == 0
    assert calls == ["ensure", "menu"]


def test_只列菜单不重入_source(monkeypatch):
    """`--list` 只打印菜单，不碰 rclpy ⟹ 不必为它白等一次 `source`（有意省掉）。"""
    calls = []
    monkeypatch.setattr(z0, "ensure_sourced", lambda: calls.append("ensure"))
    monkeypatch.setattr(z0, "menu_loop", lambda: calls.append("menu") or 0)
    assert z0.main(["--list"]) == 0
    assert calls == []


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


def test_互斥的参考源不能同时打开():
    """两条 `reference_source` 同时入命令行时 ROS 只取最后一个、**不报错** ⟹
    必须在拼装处一声响，不能让它静默变成"后答的胜出"。"""
    mpc = z0.ACTION_BY_KEY["mpc"]
    both = [opt.name in ("reference_source_traj", "reference_source_control")
            for opt in mpc.optionals]
    with pytest.raises(ValueError, match="reference_source"):
        z0.optionals_suffix(mpc, both)
    # 只开一条（菜单 9 第 2 档的情形）照常拼出尾巴
    only_control = [opt.name == "reference_source_control" for opt in mpc.optionals]
    assert "reference_source:=control" in z0.optionals_suffix(mpc, only_control)


def test_答_y_之后命令尾巴真的进了_bash_c(capsys, monkeypatch):
    seen = []
    monkeypatch.setattr(z0, "_serial_holders", lambda _path: [])       # 别去动真的串口
    monkeypatch.setattr(z0, "ask_confirm", lambda _q: True)
    monkeypatch.setattr(z0, "run_foreground", lambda cmd: seen.append(cmd) or 0)
    assert z0.launch_action(z0.ACTION_BY_KEY["mavros"]) == 0
    # ★ 这一行**故意写成字面量**（不从 `ACTION_BY_KEY` 取）：它是安全相关的那条命令，
    #   改它就该让这条用例红一次。2026-10-13（票 13 第九轮）从上游 `px4.launch` 改指
    #   本仓的 `mavros.launch.py` —— 理由见 `z0.py` 文件头那条口径澄清。
    assert seen == [f"ros2 launch bringup mavros.launch.py fcu_url:={z0.FCU_URL} gcs_url:={z0.GCS_URL}"]


def test_可选参数的值也受安全不变式扫查():
    """可选参数的 `suffix` 与主命令走同一条 `bash -c` ⟹ 必须一起扫。"""
    assert z0.forbidden_commands(z0.ACTIONS) == []
    forged = z0.Action("伪", "伪动作", None, "伪",
                       optionals=(z0.OptionalArg("伪参数", "问一句", "&& ros2 service call /x"),))
    assert z0.forbidden_commands([forged]) != []


# ─────────────────────────── 一键起链（ticket 22 §七） ───────────────────────────


def _chain_choice_keys(chain, *, with_handoff: bool = False) -> list:
    """这条链**各档合起来**能起（或要让终端）的 key（去重、保序）—— 判据要盖住全部档。"""
    keys: list = []
    for choice in chain.choices:
        tail = z0.MANUAL_CHAIN + choice.tail
        for key in (tail + choice.handoff) if with_handoff else tail:
            if key not in keys:
                keys.append(key)
    return keys


def test_起链的顺序就是操作手册那条():
    assert z0.UP_CHAIN == ("mavros", "vrpn", "mocap", "traj", "control")
    # 第 8 项：动捕 + mavros（手动遥控飞）—— **没有** traj/control（没人上位控制飞机）
    assert z0.MANUAL_CHAIN == ("mavros", "vrpn", "mocap")
    assert z0.UP_HANDOFF == ("fly",)
    assert tuple(chain.key for chain in z0.CHAINS) == ("up_manual", "up")
    for chain in z0.CHAINS:
        keys = list(chain.steps) + list(chain.handoff)
        assert set(keys) <= set(z0.ACTION_BY_KEY), \
            f"{chain.key} 里出现了动作表里没有的东西：{set(keys) - set(z0.ACTION_BY_KEY)}"
        assert chain.label and chain.closing, f"{chain.key}: 起完要说的话不能是空的"
        # 两条链的公共前缀必须**逐字相同**（否则"第 8 项就是第 9 项的前三条"这句话是假的）
        assert chain.steps[:len(z0.MANUAL_CHAIN)] == z0.MANUAL_CHAIN
    assert set(z0.UP_OPTIONALS) <= set(z0.ACTION_BY_KEY)
    for key, names in z0.UP_OPTIONALS.items():
        have = {opt.name for opt in z0.ACTION_BY_KEY[key].optionals}
        assert set(names) <= have, f"{key} 默认要带的可选参数不在它的表项里：{set(names) - have}"


def test_第八项不再问那一句_且与一版之前起的一样(monkeypatch, capsys):
    """★ 票 14 二定的第一条（2026-10-16 操作者：「z0.py 的 8 复原」）：

    第 8 项**只有一档** ⟹ 起链时**不问**，起的东西与一版之前（`85bde0d^`）**逐字一致**
    —— 三条命令 `mavros/vrpn/mocap`、不让终端、收尾说「直接用遥控器起飞」。

    ★ 判据挑的是**「问没问」这件事本身**（`input` 换成抛异常的假件）：把那一问留下的
      症状不是报错，是**多占操作者一次手**（还会让人以为能选别的档）。
    """
    def boom(*_a, **_k):
        raise AssertionError("第 8 项**不许**提问（它只有一档）")

    monkeypatch.setattr("builtins.input", boom)
    chain = z0.CHAIN_BY_KEY["up_manual"]
    assert len(chain.choices) == 1, "第 8 项就是「只有一档」（多一档 ⟹ 那一句又回来了）"
    assert chain.choice(1).key == "none"
    chosen = z0.ask_chain_choice(chain)          # ★ 不问也要有返回值，且不碰 `input`
    assert chosen.key == "none"
    assert "不问" in capsys.readouterr().out, "不问 ≠ 不说：屏上照旧要把这一档起什么打出来"
    assert list(chain.steps) == list(z0.MANUAL_CHAIN)
    assert chain.default_choice == 1
    assert not chain.handoff, "第 8 项不选轨迹 ⟹ 不把终端让给操作台"
    assert "遥控器" in "\n".join(chain.closing)


def test_两档的尾巴都起_traj_且MPC档带三条参数():
    """★ 票 15 之后的表：第 9 项**只有两档**、**两档的尾巴都含 `traj` 与 `control`**
    （操作者原话「轨迹必须起没得选」—— 这是**结构上的事实**，不是提示语）。

    ★★ 票 15 的核心：MPC 档的尾巴变成 `traj → mpc → control` —— `control` 仍是
      **唯一任务总管**（开始键/解锁/OFFBOARD/起飞/判稳/轨迹原点/`/traj/start`/降落），
      `mpc` 只是它的输出后端。额外三条参数缺一条，这条链就跑不起来：
        · `reference_source_control` —— mpc 订 `/control/target`（绝对目标）；
        · `offboard_prestream` —— control 这档不写 PX4 setpoint，进 OFFBOARD 前那条流由 mpc 给；
        · `output_mode_external` —— control **只发** `/control/target`，不写
          `/mavros/setpoint_raw/local`（两路 setpoint 会互相覆盖）。
      ⛔ `start_traj_reference` **不再带**：`/traj/start` 由 control 的 TRACK 分支发
      （两边都发会让参考时钟归零两次）。

    ★ 顺便钉住"`hold` 那一档没有了"：一版第 3 档是**唯一**不起 `traj` 的档，
      它要是偷偷回来，下面这条 `all("traj" in …)` 立刻红。
    """
    want = {  # 档位号 → (尾巴, 要不要让终端, 额外带的参数)
        1: (("traj", "control"), True, ()),
        2: (("traj", "mpc", "control"), True,
            ("reference_source_control", "offboard_prestream", "output_mode_external")),
    }
    chain = z0.CHAIN_BY_KEY["up"]
    assert len(chain.choices) == 2, "第 9 项只剩两档（多一档少一档都要在这里现形）"
    for number, (tail, handoff, extra) in want.items():
        choice = chain.choice(number)
        assert choice.tail == tail, f"第 {number} 档的尾巴不对"
        assert bool(choice.handoff) is handoff, f"第 {number} 档「要不要把终端让出去」不对"
        assert choice.extra_optionals == extra, f"第 {number} 档额外带的参数不对"
        assert choice.title and choice.closing, f"第 {number} 档：标题/起完要说的话不能空"
        # 尾巴里**不许**重复公共前缀那几条（重复 ⟹ 同一条会被起两遍）
        assert not set(choice.tail) & set(z0.MANUAL_CHAIN), \
            f"第 {number} 档的尾巴里混进了公共前缀：{set(choice.tail) & set(z0.MANUAL_CHAIN)}"
    assert all("traj" in choice.tail for choice in chain.choices), \
        "★ 「轨迹必须起」= 每一档的尾巴里都有 `traj`（MPC**没有** hold 那一档）"
    # ★★ 票 15：两档的尾巴都含 `control`（它是唯一任务总管）；第 2 档的 control **只发**
    #   `/control/target`，第 1 档的 control 照旧直接写 PX4 setpoint。
    assert all("control" in choice.tail for choice in chain.choices), \
        "两档都必须有 control —— 开始键/解锁/起飞/轨迹原点/降落只写一遍"
    assert _cmd_for("control", chain.choice(1)).endswith("ros2 launch control control.launch.py"), \
        f"默认档的 control 不许带 output_mode（默认就是 position）：{_cmd_for('control', chain.choice(1))}"
    assert _cmd_for("control", chain.choice(2)).endswith(" output_mode:=external"), \
        ("MPC 档的 control 必须切到 external —— 否则它会和 mpc 同时写 PX4 setpoint："
         f"{_cmd_for('control', chain.choice(2))}")
    assert chain.default_choice == 1, "★ 空回车 = offboard 控制 = 今天的行为，一个字不变"
    # ★ 空回车 = 默认档 ⟹ 默认档下的顺序与这一版之前**逐字相同**
    assert list(z0.CHAIN_BY_KEY["up_manual"].steps) == list(z0.MANUAL_CHAIN)
    assert list(z0.CHAIN_BY_KEY["up"].steps) == list(z0.UP_CHAIN)


def test_MPC档没有_hold_那一条():
    """★ 二定删掉的正是那条路：`mpc` 那一档的参考源**只能是 `traj`** —— 名字里不许有 `hold`，
    屏幕上（收尾）也不许再提"这一档的参考是 hold"那类话（留着会把现场指向一条不存在的路）。"""
    chain = z0.CHAIN_BY_KEY["up"]
    keys = [choice.key for choice in chain.choices]
    assert "mpc_hold" not in keys, f"`hold` 那一档还在：{keys}"
    mpc = chain.choice(2)
    text = "\n".join((mpc.title,) + mpc.closing)
    assert "hold" not in text, f"MPC 那一档的屏上还在说 hold：{text}"
    # 而下面这条是"它还真的带上了正确的参考源"的**正面**判据（只查名字只能证明没写错字）。
    # ★★ 票 15：参考源从 `traj` 换成 `control`（绝对目标），`start_traj_reference` **不再带**
    #   —— `/traj/start` 由 control 的 TRACK 分支发，两边都发会让参考时钟归零两次。
    assert _mpc_cmd(mpc).endswith(
        " --ros-args -p reference_source:=control"
        " --ros-args -p offboard_prestream:=true"), \
        f"MPC 档的命令不对：{_mpc_cmd(mpc)}"
    assert "start_traj_reference" not in _mpc_cmd(mpc), \
        "control 在场时 mpc 不许再自己发 /traj/start（参考时钟会被归零两次）"
    assert _mpc_cmd(chain.choice(1)).endswith("ros2 run nmpc mpc_node.py"), \
        f"offboard 控制档**不许**带这些参数（它根本不起 MPC）：{_mpc_cmd(chain.choice(1))}"


def test_默认档的提示与默认档一致():
    """菜单提示里写的"默认是哪一档"必须由 `Chain.default_choice` 现算 —— 不许另抄一份编号。

    ★ 判据直接从 `CHAINS` 现取（**不另设**一份 `{key: 默认档}` 的常量：那种副本没有生产读者，
      却正是"两份编号会分叉"的入口）。第 8 项 = 1、第 9 项 = 1 写死在这里**是故意的**
      —— 它就是"空回车 = 今天的行为"那句话的锚。
    ★ 只有一档的链（第 8 项）的提示里**根本不列档号** ⟹ 这一条对它无从适用，跳过并**明说**。
    """
    assert {c.key: c.default_choice for c in z0.CHAINS} == {"up_manual": 1, "up": 1}
    for chain in z0.CHAINS:
        note = z0.ACTION_BY_KEY[chain.key].note
        # 提示里多档用 `/` 隔开、每档形如 `N 标题`；标默认的那一档写着 `（**默认**`
        marked = re.findall(r"(\d) [^/]*?（\*\*默认\*\*", note)
        if len(chain.choices) == 1:
            assert marked == [] and "档" not in note, \
                f"{chain.key}: 只有一档，提示里不该出现档号/选档的话：{note!r}"
            continue
        assert marked == [str(chain.default_choice)], (
            f"{chain.key} 的菜单提示里标默认的档位是 {marked}，"
            f"而 `default_choice` 是 {chain.default_choice}")


def test_起链问的那一句_两档都列出来_且标出默认(monkeypatch, capsys):
    """★ 这是操作者真正看到的屏：第 9 项两档都在、**正好一处**"（默认）"、且标在默认那一档上。"""
    monkeypatch.setattr("builtins.input", lambda *a: "")
    chain = z0.CHAIN_BY_KEY["up"]
    z0.ask_chain_choice(chain)
    lines = capsys.readouterr().out.splitlines()
    numbered = [ln for ln in lines if re.match(r"\s*[1-2] = ", ln)]
    assert len(numbered) == 2, f"两档没列全：{lines}"
    marked = [ln for ln in numbered if "（默认）" in ln]
    assert len(marked) == 1, f"「（默认）」应当正好一处：{marked}"
    assert marked[0].strip().startswith(f"{chain.default_choice} = "), \
        f"默认标错档了：{marked[0]!r}"
    assert all(choice.title in "\n".join(numbered) for choice in chain.choices)


def _answers(replies: list):
    """按脚本给答案的假 `input`：**给完就抛**（不是固定返回同一个值）。

    ★ 固定返回值有一个安静的坑：被测的那条路要是"一直打错就一直问"，用例会**死循环**
      —— 挂住而不是变红，`-q` 下看着像"跑得慢"。给完就抛，等于把"问的次数"也钉住了。
    """
    pending = list(replies)

    def fake(*_a):
        assert pending, "问得比预期多（`ask_chain_choice` 该在拿到答案后就返回）"
        return pending.pop(0)

    return fake


def test_起链问的那一句_空回车走默认_打错的再问一遍(monkeypatch, capsys):
    """★ 纪律与 `ask_confirm` 一条：**不猜、不吞** —— 打错的再问一遍，空回车才是默认。"""
    chain = z0.CHAIN_BY_KEY["up"]
    monkeypatch.setattr("builtins.input", _answers(["7", "mpc", "", ""]))
    assert z0.ask_chain_choice(chain).key == "offboard"          # 第 9 项的默认档
    out = capsys.readouterr().out
    assert out.count("不认识的选项") == 2, "打错两次就该问两遍（`mpc` 那种非数字也算打错）"

    # ★ `input` 一律走"按脚本给答案、给完就**炸**"的假件（**不是**固定返回同一个值）：
    #   问不完的那条路（一直打错）在固定返回值下会变成**死循环**——测试挂住而不是变红，
    #   现场看起来只是"跑得慢"。给完就抛，等于把这一条也钉住了。
    monkeypatch.setattr("builtins.input", _answers(["2"]))
    assert z0.ask_chain_choice(chain).key == "mpc"
    monkeypatch.setattr("builtins.input", _answers(["3", "9", "2"]))
    assert z0.ask_chain_choice(chain).key == "mpc", \
        "★ 第 9 项只有两档 ⟹ 3 与 9 都算打错，要一直问到认识的数字为止（不许悄悄退回默认档）"


def test_起链问的那一句_非交互时走默认(monkeypatch, capsys):
    """管道/重定向（`input` 抛 `EOFError`）⟹ 走默认，别抛异常 —— 否则脚本在这儿炸。"""
    def eof(*_a):
        raise EOFError

    monkeypatch.setattr("builtins.input", eof)
    assert z0.ask_chain_choice(z0.CHAIN_BY_KEY["up"]).key == "offboard"
    assert "非交互" in capsys.readouterr().out


def test_档位越界要报错_不许悄悄退回默认():
    with pytest.raises(ValueError):
        z0.CHAIN_BY_KEY["up"].choice(0)
    with pytest.raises(ValueError):
        z0.CHAIN_BY_KEY["up"].choice(3)      # ★ 二定之后第 9 项只有两档
    with pytest.raises(ValueError):
        z0.CHAIN_BY_KEY["up_manual"].choice(2)   # 第 8 项只有一档


def _cmd_for(key: str, choice) -> str:
    """按 `start_child` 的同一条路（`optionals_suffix`）拼出这一档**真正会跑**的那条命令。"""
    action = z0.ACTION_BY_KEY[key]
    chosen = tuple(z0.UP_OPTIONALS.get(key, ())) + tuple(choice.extra_optionals)
    return (action.cmd or "") + z0.optionals_suffix(
        action, [o.name in chosen for o in action.optionals])


def _mpc_cmd(choice) -> str:
    return _cmd_for("mpc", choice)


def test_MPC档的收尾要说清_开始键与_offboard_档一样():
    """★★ 票 15 之后这一档**不再是"起链 ≠ 能飞"**：`control` 在场，开始键/解锁/OFFBOARD/
    起飞/判稳/轨迹原点/`/traj/start`/降落全由它负责，`mpc` 只是输出后端 ⟹ 屏上必须
    明确写出"**手上的动作与 offboard 档完全一样**"。

    ★ 反面判据同样重要：不能再出现一版的「没有 control」「解锁/切模式要手打」
      「`start_traj_reference` 替你发 start」—— 那三句现在**全是假的**，留着会把现场
      引到一条不存在的路上（尤其 `/traj/start`，手打一次就是第二次归零参考时钟）。
    """
    text = "\n".join(z0.CHAIN_BY_KEY["up"].choice(2).closing)
    assert "开始键与 offboard 档完全一样" in text, \
        "必须明说两种控制只是输出后端不同（否则操作员会以为这一档还要额外做点什么）"
    assert "唯一任务总管" in text and "control" in text
    assert "/control/target" in text and "绝对目标" in text, \
        "要说清 mpc 在这条链上吃什么（否则现场会去 mpc 里找轨迹参考）"
    assert "/traj/start" in text, "要说明 start 由 control 的 TRACK 分支发"
    assert "失流保护" in text, "control 死了之后的兜底必须写清楚（0.5 s 关闸 + PX4 1 s）"
    # 反面：三句旧话不许留。
    for stale in ("没有 `control`", "手打", "start_traj_reference", "safety-paths-drill"):
        assert stale not in text, f"MPC 档的收尾里还留着一版的旧话：{stale!r}"
    # ★ 状态栏在这一档**确实可用**（`control` 在场）—— 与一版的"会是空的"正好相反。
    assert "/control/state" in text and "照常可用" in text
    # offboard 控制档的收尾里不该出现 MPC 的概念。
    offboard = "\n".join(z0.CHAIN_BY_KEY["up"].choice(1).closing)
    assert "/control/target" not in offboard and "mpc_node" not in offboard


def test_起链里没有会动飞机的入口():
    """★ 一键起的命令**全部**来自 `ACTIONS`（两条链都只写 key）⟹ 22 号票那条不变式盖住它们。"""
    for chain in z0.CHAINS:
        keys = _chain_choice_keys(chain)
        assert z0.forbidden_commands([z0.ACTION_BY_KEY[k] for k in keys]) == []
        assert z0.ACTION_BY_KEY[chain.key].cmd is None
        assert not z0.ACTION_BY_KEY[chain.key].read_only, \
            f"`{chain.key}` 会起进程，不能算只读 —— 否则 `--dry-run` 与核对表都会撒谎"
    assert "control" in z0.UP_CHAIN          # 控制节点在链上，但**起飞**不在（起飞只有遥控器开始键）
    assert "control" not in z0.MANUAL_CHAIN  # ★ 手动飞那条**根本不起** control
    assert z0.ACTION_BY_KEY["fly"].cmd, "起完要前台跑操作台 ⟹ 它必须有命令"


def test_每一步要么有起来的判据要么明说不等():
    for chain in z0.CHAINS:
        for key in _chain_choice_keys(chain):
            if key in ("up", "up_manual", "down", "fly"):
                continue                      # 编排/前台那几条不是"起的节点"
            if key == "traj":
                assert key not in z0.STEP_READY and z0.STEP_TOPIC[key] is None
                assert "不等" in z0.wait_hint(key)
            elif key in z0.STEP_LOG_READY:
                # ★ `mpc` 这一类：闸门关着时**一帧都不发** ⟹ 判据只能落在它自己的日志上
                assert key not in z0.STEP_READY, f"{key}: 两条判据不许都写（会分不清哪条在判）"
                assert key in z0.STEP_TIMEOUT_S
                assert z0.STEP_LOG_READY[key] in z0.wait_hint(key)
            else:
                assert key in z0.STEP_READY and key in z0.STEP_TIMEOUT_S
    assert set(z0.STEP_READY) <= set(z0.UP_CHAIN)
    assert set(z0.STEP_LOG_READY) <= set(_chain_choice_keys(z0.CHAIN_BY_KEY["up"])), \
        "日志判据表里出现了没有哪一档会起的节点 ⟹ 那是死文本"


def test_判据的分类只有一处_三种情形都分对():
    """★ `wait_kind` 是"这一步按哪种判据等"的**唯一**分类处：`wait_hint`（说什么）、
    `start_and_wait`（真去等）、`_render_steps`（摊开时印不印等待上限）都问它。

    三处各写一遍 `if/elif` 时，漏改一处的表现是**静默的**（屏上承诺的等待与真等的不一致）
    ⟹ 这里把三种情形都钉住，另外用 `--dry-run` 的屏做一次**跨层**核对（见下一条用例）。
    """
    assert z0.wait_kind("mavros") == "ready"     # 话题/状态判据
    assert z0.wait_kind("mpc") == "log"          # 只在 `STEP_LOG_READY` 里
    assert z0.wait_kind("traj") is None          # 谁也不等（它本来就不发东西）
    for key in z0.STEP_READY:
        assert z0.wait_kind(key) == "ready", f"{key} 应当按话题判据等"
    for key in z0.STEP_LOG_READY:
        assert z0.wait_kind(key) == "log", f"{key} 应当按日志判据等"
    assert not set(z0.STEP_READY) & set(z0.STEP_LOG_READY), \
        "同一个 key 不许两种判据都写（会分不清哪条在判）"


def test_mpc_那一步的日志判据仍出现在_mpc_node_源码里():
    """★ 判据是**正向**的（等它自己打那一行）⟹ 那句话被改掉时这里必须当场红，
    不然现场表现是"等 120 s 超时"（红，但看不出为什么）。"""
    src = (RUN_DIR.parent / "src/nmpc/nmpc/mpc_node.py").read_text(encoding="utf-8")
    assert z0.STEP_LOG_READY["mpc"] in src, (
        f"`mpc_node.py` 里找不到那句启动日志 {z0.STEP_LOG_READY['mpc']!r} —— "
        "改措辞可以，但要同步改 `STEP_LOG_READY`")
    assert z0.ACTION_BY_KEY["mpc"].cmd.endswith("ros2 run nmpc mpc_node.py"), \
        f"`mpc` 那条命令的可执行文件改了：{z0.ACTION_BY_KEY['mpc'].cmd}"


def test_mpc_那条命令自带_acados_的两个环境变量():
    """★★ **2026-10-17 起链实测**：第 9 项死在第 5 步，`OSError: …/libacados.so: cannot open
    shared object file` —— shell 里两个变量都没设，`acados_template` 去猜了 `~/refs/acados`（源码树）。

    ★ 这条用例钉的是"**操作者不用记着先 export**"：两个变量必须在**命令里**，
      且 `--dry-run` 摊出来的同一条命令也带得上（它就是从 `cmd` 拼的）。
    ★ 判据取"两个名字都在前缀里"而不是比整串：真正的价值是**缺一个就起不来**
      （缺 `LD_LIBRARY_PATH` ⟹ `libhpipm.so` 找不到），所以两个都要在。
    """
    cmd = z0.ACTION_BY_KEY["mpc"].cmd
    assert cmd.startswith("env "), f"acados 的环境变量必须挂在命令前面：{cmd}"
    prefix = cmd.split(" ros2 ")[0]
    for name in ("ACADOS_SOURCE_DIR", "LD_LIBRARY_PATH"):
        assert name in prefix, f"{name} 没进 `mpc` 那条命令的环境前缀：{prefix}"
    # ★ 外层已经指对了就听外层的（同 `src/nmpc/test` 的 `_dut_env()`），别写死一个绝对路径
    assert "$HOME/acados" in prefix and "${ACADOS_SOURCE_DIR:-" in prefix, \
        f"默认值应当是 `$HOME/acados`、且不覆盖外层已设的值：{prefix}"
    # ★ 末尾不许留一个空路径元素（空 = 当前目录，ld.so 的规矩）
    assert "${LD_LIBRARY_PATH:+:" in prefix, \
        f"`LD_LIBRARY_PATH` 没设时会拼出一个尾部空元素（= 当前目录）：{prefix}"


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
    monkeypatch.setattr(z0, "run_foreground", boom)
    assert z0.main(["--dry-run", "up"]) == 0
    out = capsys.readouterr().out
    for key in z0.UP_CHAIN:
        assert z0.ACTION_BY_KEY[key].cmd in out, f"{key} 的命令没摊出来"
    assert z0.ACTION_BY_KEY["fly"].cmd in out, "起完要让终端给操作台选轨迹 —— 那条命令也要摊出来"
    assert "gcs_url" in out            # ★ 默认带上的那条要看得见（否则现场不知道它在）
    assert "connected=True" in out     # 判据也摊开
    assert "遥控器开始键" in out        # 起飞那两步写清楚了
    assert "飞机已摆好" in out          # `mocap` 那条人工确认
    assert "0.3 s" in out              # ★ 现场那条压键要点（去抖：压得太准会按不动）


def _default_block(out: str) -> str:
    """`--dry-run` 输出里**默认那一档**那一块（有"其他档"的链：分隔标题以上；
    ★ 只有一档的链（第 8 项）根本没有第二块 ⟹ 整份输出就是它）——"""
    head, sep, _tail = out.partition(z0.OTHER_CHOICES_HEADER)
    if not sep:
        return out                       # 只有一档 ⟹ 没有分隔标题（`render_chain` 明说"不问"）
    return head


def test_dry_run_up_manual_只到_mocap_且不起_control(capsys, monkeypatch):
    """第 8 项（手动遥控飞）与第 9 项只差后两条 + 选轨迹那一步 ——
    摊开的**默认那一档**必须正好是这些，而且**没有第二块**（复原：它只有一档、不问）。"""
    def boom(*_args, **_kwargs):
        raise AssertionError("`--dry-run up_manual` 起了东西")

    monkeypatch.setattr(z0, "start_child", boom)
    monkeypatch.setattr(z0, "cmd_up_manual", boom)
    monkeypatch.setattr(z0, "run_foreground", boom)
    monkeypatch.setattr("builtins.input", boom)     # ★ `--dry-run` **不许**进入提问
    assert z0.main(["--dry-run", "up_manual"]) == 0
    out = capsys.readouterr().out
    default = _default_block(out)
    for key in z0.MANUAL_CHAIN:
        assert z0.ACTION_BY_KEY[key].cmd in default, f"{key} 的命令没摊出来"
    assert z0.ACTION_BY_KEY["control"].cmd not in out, "手动飞那条**不许**起 control"
    assert z0.ACTION_BY_KEY["traj"].cmd not in out, "手动飞那条**不许**起 traj"
    assert z0.ACTION_BY_KEY["mpc"].cmd not in out, "手动飞那条**不许**起 MPC"
    assert z0.ACTION_BY_KEY["fly"].cmd not in out, "手动飞那条不选轨迹（没有 traj 可选）"
    assert "遥控器" in default          # 起完是"你自己用遥控器飞"
    # ★ 复原的判据：**没有**「其他档」那一块（一版之前它就没有），且明说"不问"
    assert z0.OTHER_CHOICES_HEADER not in out, "第 8 项只有一档 ⟹ 不该有「其他档」那一块"
    assert "不问" in out


def test_起链先重入_source_再问选档(up_world, monkeypatch, capsys):
    """★ 顺序有判据：`ensure_sourced()` 会把本进程 **exec 掉重开** ⟹ 先问就把答案丢了
    （现象是"问了等于没问"、**不报错**）。这与 README 里那条"敲的数字要敲第二次才认"同源。"""
    order: list = []
    monkeypatch.setattr(z0, "ensure_sourced", lambda: order.append("sourced"))
    monkeypatch.setattr(z0, "ask_chain_choice",
                        lambda chain: (order.append("asked"), chain.choice(2))[1])
    monkeypatch.setattr(z0, "wait_for_step", lambda *a, **k: (True, {}))
    monkeypatch.setattr(z0, "wait_for_log", lambda *a, **k: (True, ""))

    assert z0.cmd_up() == 0
    assert order[:2] == ["sourced", "asked"], f"先问了才重入 ⟹ 答案会被 exec 丢掉：{order}"


def test_dry_run_把两档的命令都摊开且默认那档排在前面(capsys, monkeypatch):
    def boom(*_a, **_k):
        raise AssertionError("`--dry-run up` 不该提问、也不该起东西")

    monkeypatch.setattr("builtins.input", boom)
    monkeypatch.setattr(z0, "start_child", boom)
    monkeypatch.setattr(z0, "run_foreground", boom)
    assert z0.main(["--dry-run", "up"]) == 0
    out = capsys.readouterr().out
    default = _default_block(out)
    other = out.partition(z0.OTHER_CHOICES_HEADER)[2]
    # 默认档（第 9 项 = offboard 控制）那一块里**只有**它的尾巴；MPC 档那三条
    # `--ros-args` 参数不在里头
    assert z0.ACTION_BY_KEY["control"].cmd in default
    assert "output_mode:=external" not in default, \
        "★ external 只有 MPC 档才带 ⟹ 不许出现在默认那一块"
    assert "--ros-args -p reference_source:=control" not in default, \
        "★ MPC 的那几条参数只有第 2 档才带"
    assert "--ros-args -p reference_source:=control" in other, "第 2 档那几条参数要摊出来"
    assert "offboard_prestream:=true" in other and "output_mode:=external" in other
    assert "start_traj_reference:=true" not in other, \
        "control 在场时 mpc 不许再自己发 /traj/start"
    assert z0.ACTION_BY_KEY["mpc"].cmd in other
    assert z0.ACTION_BY_KEY["control"].cmd in other, "第 2 档的尾格里也有 control（任务总管）"
    # 第 2 档（MPC）在"其他档"那一块；第 1 档是默认档 ⟹ 它**不**该出现在那里
    # （★ 默认那一块摊的是它的**命令与判据**，不是它的标题 —— 标题只在提问那一屏上）
    assert z0.CHAIN_BY_KEY["up"].choice(2).title in other
    assert z0.CHAIN_BY_KEY["up"].choice(1).title not in other
    assert f"第 {z0.CHAIN_BY_KEY['up'].default_choice} 档" in default
    # ★ **跨层核对**：摊开的等待上限必须与真等的那一处同源（都问 `wait_kind`）——
    #   `mpc` 那一步（日志判据）要带超时，`traj` 那一步要明说"不等"
    assert f"最多 {z0.STEP_TIMEOUT_S['mpc']:.0f} s" in other, \
        "`mpc` 那一步是日志判据 ⟹ 摊开时也要把等待上限写出来（与 `start_and_wait` 同源）"
    assert "不等（它本来就不发东西）" in other, "`traj` 那一步谁也不等 ⟹ 摊开时要明说"


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
    """把一键流程的外部世界全换成假的；返回 (喂探测的快照表, 起过的子进程表, 前台跑过的命令表)。

    ★ 前台那一步（`handover`）也换掉：它会真去跑 `fly.py` 并把终端抢走。
    ★ 起链前那一问（`ask_chain_choice`）换成**直接给默认档** —— 与 `ask_confirm` 同一个套路；
      要试别的档，用例自己再 `monkeypatch` 一次（见 `test_选到_MPC档_先起_traj_再起_mpc_且带上三条参数`）。
    ★ `启动参数`那一栏用的是 `choice.extra_optionals` ⟹ 假 `start_child` 也必须收它，
      否则"第 4 档带没带 `--ros-args`"这件事在编排这一层就没有任何观测点。
    """
    snapshots: list = [{}]
    started: list = []
    foreground: list = []
    optionals: list = []

    monkeypatch.setattr(z0, "ensure_sourced", lambda: None)     # 否则会真的带 source 重 exec
    monkeypatch.setattr(z0, "RUN_DIR", tmp_path)
    monkeypatch.setattr(z0, "Probe", lambda *a, **k: _Probe(snapshots))
    monkeypatch.setattr(z0, "watch_children", lambda *a, **k: 0)
    monkeypatch.setattr(z0, "run_foreground", lambda cmd: foreground.append(cmd) or 0)

    def start_child(key, log_path, extra_names=()):
        child = fake_child(key, "假命令", log_path)
        started.append(child)
        optionals.append((key, tuple(extra_names)))
        return child

    monkeypatch.setattr(z0, "start_child", start_child)
    monkeypatch.setattr(z0, "ask_confirm", lambda *a, **k: True)
    monkeypatch.setattr(z0, "ask_chain_choice", lambda chain: chain.choice(chain.default_choice))
    monkeypatch.setattr(z0, "render_snapshot", lambda snap: "（假核对表）")
    return snapshots, started, foreground, optionals


class _Probe:
    """假探测：按脚本喂的快照依次返回，用完之后一直返回最后一个。"""

    def __init__(self, snapshots):
        self._snapshots, self._calls, self.closed = snapshots, 0, False

    def snapshot(self, window_s=4.0):
        self._calls += 1
        return self._snapshots[min(self._calls - 1, len(self._snapshots) - 1)]

    def close(self):
        self.closed = True


def test_一键起链按顺序起五条_每步都等到判据_最后把终端让给操作台(up_world, monkeypatch, capsys):
    snapshots, started, foreground, _optionals = up_world
    waited = []
    monkeypatch.setattr(z0, "wait_for_step",
                        lambda probe, key, **k: (waited.append(key), (True, {}))[1])

    assert z0.cmd_up() == 0
    assert [c.key for c in started] == list(z0.UP_CHAIN)
    assert [c.log.name for c in started] == ["01-mavros.log", "02-vrpn.log", "03-mocap.log",
                                             "04-traj.log", "05-control.log"]
    # `traj` 没有判据（没 select 就一帧不发）⟹ 不空等它
    assert waited == ["mavros", "vrpn", "mocap", "control"]
    # ★ 选轨迹那一步：**前台**跑操作台（人要在里面敲 1/2/3），且它在起齐**之后**
    assert foreground == [z0.ACTION_BY_KEY["fly"].cmd]
    out = capsys.readouterr().out
    assert out.index("05-control") < out.index(z0.ACTION_BY_KEY["fly"].cmd)
    assert "起飞与执行轨迹" in out, "起完要把「还差的两步」写在屏上（含开始键那两下）"


def test_手动飞那条只起三条_不占前台_也没有_traj_control(up_world, monkeypatch, capsys):
    snapshots, started, foreground, _optionals = up_world
    waited = []
    monkeypatch.setattr(z0, "wait_for_step",
                        lambda probe, key, **k: (waited.append(key), (True, {}))[1])

    assert z0.cmd_up_manual() == 0
    assert [c.key for c in started] == list(z0.MANUAL_CHAIN)
    assert [c.log.name for c in started] == ["01-mavros.log", "02-vrpn.log", "03-mocap.log"]
    assert waited == ["mavros", "vrpn", "mocap"]      # 三条都有判据（没有 traj）
    assert foreground == [], "手动飞那条不选轨迹 ⟹ 不把终端让出去（前台一直守着）"
    assert "遥控器" in capsys.readouterr().out


def test_选到_MPC档_先起_traj_再起_mpc_再起_control_且各带参数(up_world, monkeypatch, capsys):
    """★★ 第 9 项第 2 档（票 15 之后的形状）：`traj → mpc → control`。

    三条额外参数缺一条，这条链就断在某一处：
    · `reference_source:=control` —— 否则 mpc 还在等 `/traj/reference`，而 control 这档不发它；
    · `offboard_prestream:=true` —— control 这档不写 PX4 setpoint，进 OFFBOARD 前那条流由 mpc 给；
    · `control output_mode:=external` —— 否则 control 仍写 `/mavros/setpoint_raw/local`，
      与 mpc 的 attitude setpoint **两路同时驱动飞控**。
    ⛔ `start_traj_reference` 不带：`/traj/start` 由 control 的 TRACK 分支发。
    """
    snapshots, started, foreground, optionals = up_world
    monkeypatch.setattr(z0, "ask_chain_choice", lambda chain: chain.choice(2))
    monkeypatch.setattr(z0, "wait_for_step", lambda *a, **k: (True, {}))
    monkeypatch.setattr(z0, "wait_for_log", lambda *a, **k: (True, ""))

    assert z0.cmd_up() == 0
    assert [c.key for c in started] == ["mavros", "vrpn", "mocap", "traj", "mpc", "control"]
    assert [c.log.name for c in started][-2:] == ["05-mpc.log", "06-control.log"]
    # ⚠️ 这里记的是**这一档带的名字**（`start_child` 再按各 action 自己的 `optionals` 过滤）
    #   ⟹ mpc 那条里也会看到 `output_mode_external`（它属于 control）。
    assert optionals[-2] == ("mpc", ("reference_source_control", "offboard_prestream",
                                    "output_mode_external")), \
        "★ mpc 那两条少一条这条链就断在某一处（见 docstring）"
    assert optionals[-1] == ("control", ("reference_source_control", "offboard_prestream",
                                        "output_mode_external")), \
        "★ control 必须切 external，否则两路 setpoint 同时写飞控"
    assert foreground == [z0.ACTION_BY_KEY["fly"].cmd], "选轨迹那一步照旧（`fly.py` 的选择那一半可用）"
    out = capsys.readouterr().out
    assert "开始键与 offboard 档完全一样" in out, \
        "起完要说明：这一档手上的动作与 offboard 档完全相同"
    assert "/control/target" in out and "唯一任务总管" in out, \
        "要说清这一档谁管任务、谁只是输出后端"


def test_选到_offboard_控制档_起四条_一条参数都不带(up_world, monkeypatch, capsys):
    """第 9 项第 1 档（默认）= 今天的行为：`traj → control`，**不**起 MPC、
    也不带任何 `--ros-args` 参数。"""
    snapshots, started, foreground, optionals = up_world
    monkeypatch.setattr(z0, "ask_chain_choice", lambda chain: chain.choice(1))
    monkeypatch.setattr(z0, "wait_for_step", lambda *a, **k: (True, {}))

    assert z0.cmd_up() == 0
    assert [c.key for c in started] == ["mavros", "vrpn", "mocap", "traj", "control"]
    assert all(names == () for _key, names in optionals), \
        f"这一档一条可选参数都不带（`mavros` 的 `gcs_url` 是另一条路）：{optionals}"
    assert foreground == [z0.ACTION_BY_KEY["fly"].cmd]
    assert "遥控器开始键" in capsys.readouterr().out


def test_mpc_那一步等的是它自己的日志行_等不到就报红并停掉其余的(up_world, monkeypatch, capsys):
    """★ `mpc` 的判据只能落在日志上（闸门关着时它一帧都不发）；等不到 ⟹
    打日志尾巴 + **停掉前面那几条** + 退出码 1（不许留下一半的链）。"""
    snapshots, started, _foreground, _optionals = up_world
    monkeypatch.setattr(z0, "ask_chain_choice", lambda chain: chain.choice(2))
    monkeypatch.setattr(z0, "wait_for_step", lambda *a, **k: (True, {}))
    seen = []
    monkeypatch.setattr(z0, "wait_for_log",
                        lambda child, marker, **k: (seen.append((child.key, marker)), (False, "等超时（120 s）"))[1])

    assert z0.cmd_up() == 1
    assert seen == [("mpc", z0.STEP_LOG_READY["mpc"])], "等错了行（或没等）"
    out = capsys.readouterr().out
    assert "等超时" in out and "假日志尾" in out
    assert [c.key for c in started] == ["mavros", "vrpn", "mocap", "traj", "mpc"], \
        "★ 这一档 `traj` 在 `mpc` **前面**（两档的尾巴都含 traj）—— 起齐到 mpc 为止"
    assert all(c.stopped and c.closed for c in started), "中断后不能留下任何子进程"


def test_mavros_已经在跑就问沿用_并且收尾不停它(up_world, monkeypatch, capsys):
    snapshots, started, _foreground, _optionals = up_world
    snapshots[0] = {"counts": {"/mavros/state": 1},      # 第一口探测就看到 mavros 在跑
                    "last": {"/mavros/state": "connected=True armed=False mode=POSCTL"}}
    monkeypatch.setattr(z0, "wait_for_step", lambda *a, **k: (True, {}))

    assert z0.cmd_up() == 0
    assert [c.key for c in started] == ["vrpn", "mocap", "traj", "control"]   # 没有第二个 mavros
    assert "沿用" in capsys.readouterr().out
    # ★ 收尾只停**本脚本起的**：别人的 mavros 根本不在 `started` 里，也就停不到它
    assert all(c.log.name != "01-mavros.log" for c in started)


def test_某一步超时_打日志尾巴_停掉其余的_返回_1(up_world, monkeypatch, capsys):
    snapshots, started, foreground, _optionals = up_world
    monkeypatch.setattr(z0, "wait_for_step",
                        lambda probe, key, **k: (key != "mocap", {}))   # 卡在 mocap

    assert z0.cmd_up() == 1
    out = capsys.readouterr().out
    assert "等超时" in out and "假日志尾" in out
    assert [c.key for c in started] == ["mavros", "vrpn", "mocap"]      # 没往下起 traj/control
    assert all(c.stopped for c in started), "中断后不能留下任何子进程（尤其是 mavros）"
    assert all(c.closed for c in started)
    assert foreground == [], "起链都没起齐 ⟹ 不许把终端让出去"


def test_子进程半路退了_不当成超时(up_world, monkeypatch, capsys):
    snapshots, started, _foreground, _optionals = up_world

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


# ─────────────── 清除后台节点（第 10 项 `down`，ticket 22 §八，2026-09-23） ───────────────
# ⚠️ 这个节标题原先写"第 11 项"（**早就写错了**，`down` 一直是第 10 项）；2026-09-29 复核时
#    顺手改成对的 —— 因为同一趟里第 11 项变成了 `rec_start`，留着它会指着别人。


def _launch_package(key: str) -> str:
    """从动作表那条命令里取出包名（`ros2 launch <包> <文件>` / `ros2 run <包> <可执行>`）——
    **不另抄一份**。★ ticket 14 起链里多了 `mpc`（`ros2 run nmpc mpc_node.py`）⟹ 这里认两种写法。
    ★ 2026-10-17：`mpc` 那条命令前面多了 `env ACADOS_SOURCE_DIR=… LD_LIBRARY_PATH=…`
      （起链实测缺它则 `OSError: libacados.so`）⟹ 这里**从 `ros2` 那一段开始取**，
      不管前面挂了几个 `env` 赋值。"""
    cmd = z0.ACTION_BY_KEY[key].cmd or ""
    parts = cmd.split()
    assert "ros2" in parts, f"{key} 那条命令里没有 `ros2` 这一截：{cmd}"
    i = parts.index("ros2")
    assert parts[i + 1:i + 2] in (["launch"], ["run"]), \
        f"{key} 那条命令既不是 launch 也不是 run：{cmd}"
    return parts[i + 2]


def test_清除目标表按名字扫_且把起链的每一条都盖住():
    """★ 判据来自 ticket 22 §八：起链能起的每一条，清除都必须能扫到 ——
    否则"清了之后串口还被占着"这类残留只能靠人肉 `ps`。
    ★ ticket 14：判据从"默认那五条"扩到**各档合起来**能起的全部（含 `mpc`）。"""
    keys = [t.key for t in z0.DOWN_TARGETS]
    assert keys[0] == "mavros", "串口的持有者应当排在最前面（最先要看它有没有放开）"
    for key in _chain_choice_keys(z0.CHAIN_BY_KEY["up"]):
        assert key in keys, f"起链里有 `{key}`，清除表里没有它 ⟹ 清了也留着"
    assert "mpc" in keys, "★ MPC 两档会起 `mpc_node.py` ⟹ 清除表必须扫得到它"
    for target in z0.DOWN_TARGETS:
        assert target.label and target.pattern
        re.compile(target.pattern)                      # ★ 模式得是能编译的正则（写错要在这里炸）
        assert _launch_package(target.key) in target.pattern, \
            f"{target.key}: 模式里没有它自己的包名 ⟹ 太宽或写错了：{target.pattern}"
        for too_wide in ("python3", "bash", "sudo", "/opt/ros", ".*"):
            assert too_wide not in target.pattern, f"{target.key}: 模式里有 {too_wide} ⟹ 会误伤"


def test_清除目标表只有一张_且命令仍受安全不变式扫():
    """清除走的是 `pgrep`/`kill`，不进 `ACTIONS` 的命令列；但**不许**因此出现
    `ros2 service call` 那类字样（守卫表扫的是"最终送进 bash 的东西"，这里是另一条路径）。"""
    text = " ".join(t.pattern for t in z0.DOWN_TARGETS)
    for token in z0.FORBIDDEN_IN_COMMANDS:
        assert token not in text


def test_解析_pgrep_输出():
    assert z0.parse_pgrep("") == []
    assert z0.parse_pgrep("412\n7\n412\n") == [412, 7]     # 去重、保序
    assert z0.parse_pgrep("不是数字\n9\n\n") == [9]          # 脏行丢掉，不炸


def _fake_scan(monkeypatch, pids):
    """把扫描的**两个外部读数**换成假的：pgrep 的结果 + "这个 pid 还在不在"。
    造出来的 pid 是假的 ⟹ 不换 `_pid_alive` 的话它们会被当成僵尸剔掉（那另有专门用例）。"""
    monkeypatch.setattr(z0, "_pgrep_ids", lambda pattern: list(pids))
    monkeypatch.setattr(z0, "_pid_cmdline", lambda pid: f"假命令行 {pid}")
    monkeypatch.setattr(z0, "_pid_alive", lambda pid: True)


def test_扫描时把本脚本自己剔掉(monkeypatch):
    """★ 把自己列进清单就是"清除脚本把清除脚本杀了"。"""
    _fake_scan(monkeypatch, [os.getpid(), 4200])
    hits = z0.scan_down_targets()
    assert [h.pid for h in hits] == [4200]
    assert hits[0].cmdline == "假命令行 4200"


def test_同一个进程被两条模式扫到也只报一次(monkeypatch):
    """`ros2 launch` 的父进程、bash 壳、节点本身**都会**命中 ⟹ 不去重就会停它三遍。"""
    _fake_scan(monkeypatch, [4200, 4200, 7])
    hits = z0.scan_down_targets()
    assert [h.pid for h in hits] == [4200, 7]
    assert [h.key for h in hits] == [z0.DOWN_TARGETS[0].key] * 2   # 归属 = 第一条命中的模式


def test_停进程的顺序是先_SIGINT_再_SIGTERM_最后_SIGKILL():
    """★ 与 `bench` 的 `stop_dut` 同一条纪律（`scenario.py:298`）：先礼后兵 ——
    先 SIGINT 让节点走自己的收摊逻辑，超时才升级。"""
    sent = []
    alive = {7}

    def fake_send(pid, sig):
        sent.append(sig)
        if sig == signal.SIGKILL:
            alive.discard(pid)                          # 只有 SIGKILL 才杀得掉它

    survivors = z0.stop_pids([7], alive=lambda pid: pid in alive,
                             send=fake_send, sleep=lambda _s: None)
    assert sent == [signal.SIGINT, signal.SIGTERM, signal.SIGKILL]
    assert survivors == []


def test_在_SIGINT_就退了的进程不会被补刀():
    sent = []

    def fake_send(pid, sig):
        sent.append(sig)
        if sig == signal.SIGINT:
            alive.discard(pid)

    alive = {7}
    assert z0.stop_pids([7], alive=lambda pid: pid in alive,
                        send=fake_send, sleep=lambda _s: None) == []
    assert sent == [signal.SIGINT]


def test_杀不掉的要如实报出来():
    assert z0.stop_pids([7], alive=lambda pid: True,
                        send=lambda pid, sig: None, sleep=lambda _s: None) == [7]


def test_绝不向自己与_init_放信号():
    """安全不变式：pid 0/1 与自己一律跳过（放信号给它们不是"清除"，是事故）。"""
    sent = []
    assert z0.stop_pids([0, 1, os.getpid()], alive=lambda pid: True,
                        send=lambda pid, sig: sent.append(pid), sleep=lambda _s: None) == []
    assert sent == []


def _proc_state(pid: int) -> str:
    """`/proc/<pid>/stat` 里那个状态字符（`Z` = 僵尸）。用例自己判"造出来的真是僵尸"。"""
    try:
        with open(f"/proc/{pid}/stat") as fh:
            return fh.read().rsplit(") ", 1)[-1].split(" ", 1)[0]
    except OSError:
        return ""


def _make_zombie() -> "subprocess.Popen":
    """造一个**真**僵尸：杀掉子进程但**不收尸**（不 `wait()`）⟹ 它留在进程表里等父进程。"""
    proc = subprocess.Popen(["sleep", "300"], start_new_session=True)
    os.kill(proc.pid, signal.SIGINT)
    deadline = time.time() + 5.0
    while time.time() < deadline:
        if _proc_state(proc.pid) == "Z":
            return proc
        time.sleep(0.05)
    proc.wait()
    pytest.fail("造不出僵尸态，这条用例本身没验到东西")


def test_僵尸进程不算活着():
    """★ 真事（2026-09-23 拿两个真进程试 `stop_pids` 时撞上的）：`os.kill(pid, 0)` 对**僵尸**
    **是成功的** ⟹ 只看它就等于对着尸体连升三级，最后还报一句假的"没停掉"、退出码非零。
    判据只能回 `/proc/<pid>/stat` 看状态字符。（僵尸在 `down` 的常见成因：对面那个
    `run/z0.py up` 正在 `watch_children` 里收尸，中间那一小段窗口。）
    """
    proc = _make_zombie()
    try:
        assert z0._pid_alive(proc.pid) is False
    finally:
        proc.wait()


def test_扫描不把僵尸算进清单(monkeypatch):
    """僵尸**已经死了**，只是父进程还没收尸 —— 列进"要停谁"只会让人以为没清干净。"""
    proc = _make_zombie()
    try:
        monkeypatch.setattr(z0, "_pgrep_ids", lambda pattern: [proc.pid])
        monkeypatch.setattr(z0, "_pid_cmdline", lambda pid: "[sleep] <defunct>")
        assert [h.pid for h in z0.scan_down_targets()] == []
    finally:
        proc.wait()


def _hit(pid: int = 4242, key: str = "mavros") -> object:
    return z0.DownHit(key=key, label=f"{key} 的假说明", pid=pid,
                      cmdline=f"ros2 launch {_launch_package(key)} 假文件")


def test_没有可清的节点时如实说没有(capsys, monkeypatch):
    monkeypatch.setattr(z0, "ensure_sourced", lambda: None)
    monkeypatch.setattr(z0, "scan_down_targets", lambda **k: [])
    monkeypatch.setattr(z0, "_serial_holders", lambda path: [])
    assert z0.cmd_down() == 0
    out = capsys.readouterr().out
    assert "没有" in out and "串口" in out


def test_缺了扫不动的工具就不许说没有可清的(capsys, monkeypatch):
    """★ "扫不动"与"没有可清的"在屏幕上长得一样、后果却相反（仓库为"静默为空"交过两次学费：
    `check` 的 `ros2 daemon` 坑、半拉子环境）⟹ 缺 `pgrep`/`ps` 时必须**明说 + 非零退出**。"""
    monkeypatch.setattr(z0, "ensure_sourced", lambda: None)
    monkeypatch.setattr(z0, "_missing_scan_tools", lambda: ["pgrep"])
    monkeypatch.setattr(z0, "scan_down_targets", lambda **k: [])   # 就算硬扫也是空 —— 正是危险形状
    assert z0.cmd_down() != 0
    out = capsys.readouterr().out
    assert "pgrep" in out and "什么都没做" in out
    assert "没有扫到任何在跑的节点" not in out      # ★ 不许把它说成"没什么可清的"


def test_清单里的命令行一个字都不截():
    """★ 按名字扫会捞到别人会话的节点，唯一的缓解手段就是人**看一眼命令行**
    （复核 2026-09-23 抓到的真事：原先 `[:88]` 把 mavros 那条正好切在 `fcu_url:=…` 上）。"""
    long_cmd = "ros2 launch mavros px4.launch fcu_url:=/dev/ttyTHS1:921600 gcs_url:=udp-b://0.0.0.0:14555@:14550"
    text = z0.render_down([z0.DownHit(key="mavros", label="mavros（独占串口）", pid=38663,
                                      cmdline=long_cmd)])
    assert long_cmd in text, "命令行被截断了 —— 那就只剩 pid 可看，人认不出是不是自己那套"


def test_进程内动作就是_cmd_为空的那几条():
    """判据是**表里 `cmd` 为空**（与 `READ_ONLY_KEYS` 同一套路）⟹ 加一项菜单不用记着改名单。
    这条用例把"推导成立"本身钉住：非进程内的动作**必须**有命令可跑。"""
    assert z0.IN_PROCESS_KEYS == tuple(a.key for a in z0.ACTIONS if a.cmd is None)
    for action in z0.ACTIONS:
        assert (action.cmd is None) == (action.key in z0.IN_PROCESS_KEYS), \
            f"{action.key}: `cmd` 空不空与它在不在进程内跑必须一致"


def test_清除要先问一句_答_n_一个信号都不发(capsys, monkeypatch):
    sent = []
    monkeypatch.setattr(z0, "ensure_sourced", lambda: None)
    monkeypatch.setattr(z0, "scan_down_targets", lambda **k: [_hit()])
    monkeypatch.setattr(z0, "ask_confirm", lambda *a, **k: False)
    monkeypatch.setattr(z0, "_send_signal", lambda pid, sig: sent.append((pid, sig)))
    assert z0.cmd_down() == 1
    assert sent == []
    assert "已取消" in capsys.readouterr().out


def test_答_y_才动手_并把结果与串口状态报出来(capsys, monkeypatch):
    rounds = [[_hit()], []]                     # 第一扫：找到 1 个；停完再扫：一个不剩
    monkeypatch.setattr(z0, "ensure_sourced", lambda: None)
    monkeypatch.setattr(z0, "scan_down_targets", lambda **k: rounds.pop(0))
    monkeypatch.setattr(z0, "ask_confirm", lambda *a, **k: True)
    monkeypatch.setattr(z0, "stop_pids", lambda pids, **k: [])   # 都停掉了
    monkeypatch.setattr(z0, "_serial_holders", lambda path: [])
    assert z0.cmd_down() == 0
    out = capsys.readouterr().out
    assert "4242" in out                        # 动手前先让人看清是谁
    assert "不会让飞机降落" in out                # ★ 在空中按它的后果，写在同一屏上
    assert "串口" in out


def test_停完还剩着就返回非零(capsys, monkeypatch):
    monkeypatch.setattr(z0, "ensure_sourced", lambda: None)
    monkeypatch.setattr(z0, "scan_down_targets", lambda **k: [_hit()])
    monkeypatch.setattr(z0, "ask_confirm", lambda *a, **k: True)
    monkeypatch.setattr(z0, "stop_pids", lambda pids, **k: [4242])   # 杀不掉
    monkeypatch.setattr(z0, "_serial_holders", lambda path: ["mavros(4242)"])
    assert z0.cmd_down() == 1
    out = capsys.readouterr().out
    assert "4242" in out and "串口" in out


def test_dry_run_down_只看不停(capsys, monkeypatch):
    monkeypatch.setattr(z0, "scan_down_targets", lambda **k: [_hit()])
    monkeypatch.setattr(z0, "_serial_holders", lambda path: [])
    monkeypatch.setattr(z0, "stop_pids",
                        lambda *a, **k: pytest.fail("--dry-run 停进程了"))
    assert z0.main(["--dry-run", "down"]) == 0
    out = capsys.readouterr().out
    assert "4242" in out                        # 会停谁要看得见
    assert "不" in out


def test_菜单十一条_号与表里的顺序一致_隐藏的四条不占号():
    """★ 菜单号被跨文件钉死了（`operation_manual.md` 逐条引用、
    `test_operation_manual.py` 拿 `ACTIONS[N-1].title` 对）⟹ 动号要三处一起动。

    形状（三个时点，都是操作者拍定的）：
    · 2026-09-23：菜单里**去掉** 8= bringup / 9= 操作台 `fly.py`，
      8= 一键启动动捕+mavros（手动遥控飞），9= 一键启动全部，10= 清除后台节点；
    · 2026-09-29（ticket 19 落地）：11= 开始录制 / 12= 停止录制并收口 / 13= 归档飞控日志（从 SD 卡）；
    · 2026-09-29（同日现场复盘后）：★ **12 与 13 连功能一起删掉** —— 录制改成**前台一条流程**
      （菜单 11 占住终端、倒计时到 120 s 自己收口），"停"不再是独立动作；飞控日志改回
      **拔 SD 卡手工拷**。菜单从此 **11 项**。
    ★ 隐藏的那几条**命令/功能**没删，只是 `menu=False`（不显示、不占号）：`fly` 是一键流程让出去的
    那一步，`bringup` 是 `DOWN_TARGETS` 取包名的地方，`rec_status`/`rec_manifest`/`rec_adopt`
    是排障时的命令行入口（`rec_adopt` = 给缺 `session.json` 的包补一份事后重建的记录），
    ★ **ticket 14 新增 `mpc`**：第 8/9 项的第 3/4 档会起它 ⟹ 它进 `ACTIONS`（命令只写一处）、
    但**不占菜单号**（菜单照旧 11 项）——
    删了它们，"唯一入口是 `ACTIONS`"就破了个口。
    """
    assert [a.key for a in z0.MENU_ACTIONS] == [
        "env", "check", "mavros", "vrpn", "mocap", "traj", "control", "up_manual", "up", "down",
        "rec_start"]
    assert [a.key for a in z0.ACTIONS] == [a.key for a in z0.MENU_ACTIONS] + [
        "rec_status", "rec_manifest", "rec_adopt", "mpc", "bringup", "fly"], \
        "菜单项应当排在前面、`menu=False` 的排在后面（号 = 位置 ⟹ 别让隐藏项插队）"
    for key in ("bringup", "fly", "rec_status", "rec_manifest", "rec_adopt", "mpc"):
        assert not z0.ACTION_BY_KEY[key].menu
        assert z0.ACTION_BY_KEY[key].title not in z0.menu_text(), f"{key} 不该出现在菜单里"
    assert not z0.MENU_ACTIONS[9].read_only, "它会停进程 ⟹ 不能算只读（否则 --dry-run 会撒谎）"
    assert "down" in z0.IN_PROCESS_KEYS and "up_manual" in z0.IN_PROCESS_KEYS
    # ★ 录制这一项也**不是**只读：它会起进程、发信号、写文件 —— `--dry-run` 对它
    #   必须真的什么都不做（判据在 test_rec.py 的 `dry_run_note` 逐条用例里）。
    assert not z0.ACTION_BY_KEY["rec_start"].read_only, "它会起录制进程，不能算只读"
    assert z0.ACTION_BY_KEY["rec_status"].read_only, "它只读状态文件 + 扫一眼进程"


# ─────────────── 位置参数管道（`rec_start N` 必须真的收得到） ───────────────


def test_rec_start_收得到场景号(capsys, monkeypatch):
    """★ 回归（2026-09-29 复核抓到，第一手复现）：`main` 原先用 `parse_args`，
    于是动作自带的 `--` 参数会被 argparse 拒掉（当时撞上的是 `rec_fcu … --since <ISO>`）。
    改成 `parse_known_args` 之后，多出来的参数**原样**交给动作自己（`rec.cmd_start`）。
    ⚠️ 这一条现在守着的是那个**口子**本身：`TAKES_ARGS` 缩到只剩 `rec_start` 之后，
    将来任何带 `--flag` 的动作都靠它才收得到参数。
    """
    seen = []
    monkeypatch.setattr(z0, "in_process_command",
                        lambda key: (lambda argv: (seen.append((key, list(argv))), 0)[1]))
    assert z0.main(["rec_start", "3"]) == 0
    assert seen == [("rec_start", ["3"])], "场景号应当原样落到 `rec.cmd_start` 手里"


def test_不吃参数的动作多给了就报错(capsys):
    """★ `parse_known_args` 的代价要挡住：打错字不许被静默吞掉。
    只有 `TAKES_ARGS` 那一条能收下"不认识"的参数，别的动作多给一个就当场报错。"""
    assert z0.TAKES_ARGS == ("rec_start",)
    assert "check" not in z0.TAKES_ARGS
    with pytest.raises(SystemExit) as excinfo:
        z0.main(["check", "--since", "2026-09-29T00:00:00"])
    assert excinfo.value.code == 2                     # argparse 的用法错误
    assert "--since" in capsys.readouterr().err        # 得把多出来的那个原样报出来
