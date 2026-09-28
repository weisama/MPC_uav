#!/usr/bin/env python3
"""用假终端（pty）驱动 PX4 SITL 的 pxh 控制台 —— ticket 08 的取证工具。

为什么不能直接拿管道喂 `./bin/px4`（实测，2026-09-28）：

1. **非 tty 的 stdin 会让 pxh 退化成一个只重画提示符的忙循环**：管道喂 3 条命令、
   跑 20 s，stdout 就产生 **64 MB**（全是 `\\x1b[2Kpxh> ` 的重复），日志没法看；
2. **`\\x03`（Ctrl-C）送不进去**：它不是被行规程转成 SIGINT，只是一个普通字节 ——
   这是让 SITL 干净退出的正常办法，管道里做不到；
3. 回显与换行行为与真终端不同，`listener` 之类的交互命令容易卡住。

pty 之后它就是一个正常终端：逐字符回显、`\\x03` → SIGINT、窗口宽度可控（列宽设大
避免折行）。**只用标准库**（pty/termios/fcntl），不引入 pexpect。

用法（脚本内）：

    from px4_console import Px4Console
    px = Px4Console(cwd=BUILD_DIR, env={"PX4_SIM_MODEL": "sihsim_quadx"})
    px.start()
    px.wait_ready()                      # 等 "Startup script returned"
    px.send("listener sensor_accel")
    px.wait_for(r"z: *(-?[0-9.]+)", 5)   # 匹配就返回，否则超时抛错
    px.close()                           # 送 SIGINT

用法（手工探查）：

    python3 px4_console.py --cmd-file /tmp/cmds.txt            # 每行一条命令
    python3 px4_console.py --cmd-file /tmp/cmds.txt --plant z0 # 带 z0 覆盖层
"""

from __future__ import annotations

import argparse
import fcntl
import os
import pty
import re
import select
import shutil
import signal
import struct
import subprocess
import sys
import termios
import time

# pxh 会用 ANSI 转义做行内重画；解析前一律剥掉，否则正则会被 `\x1b[2K` 打断。
ANSI_RE = re.compile(r"\x1b\[[0-9;?]*[A-Za-z]|\x1b[()][A-Z0-9]")


class Px4ConsoleError(RuntimeError):
    pass


class Px4Console:
    """一个 PX4 SITL 进程 + 它的 pty 控制台。"""

    def __init__(self, cwd, binary="./bin/px4", env=None, cols=200, rows=40):
        self.cwd = str(cwd)
        self.binary = binary
        self.extra_env = dict(env or {})
        self.cols = cols
        self.rows = rows
        self.proc = None
        self._master = None
        self._raw = bytearray()
        self.exited = False

    # ---------- 生命周期 ----------

    def start(self):
        if self.proc is not None:
            raise Px4ConsoleError("已经启动过了")

        env = dict(os.environ)
        env.update(self.extra_env)

        master, slave = pty.openpty()
        # 窗口宽度是 200 列：默认 0 列会让 pxh 的行编辑行为异常，且小宽度会把
        # listener 的输出折行，正则反而难写。
        # 回显不用我们管：pxh 自己在 `_setup_term()` 里就 `~ICANON ~ECHO`，
        # 然后逐字符 `fprintf` + `ESC[2K\r` 重画 —— 那些重画由 text() 收掉。
        fcntl.ioctl(slave, termios.TIOCSWINSZ,
                    struct.pack("HHHH", self.rows, self.cols, 0, 0))

        def _child_setup():
            os.setsid()
            # 让 pty 成为控制终端 —— 否则 ^C 不会变成 SIGINT。
            fcntl.ioctl(0, termios.TIOCSCTTY, 0)

        self.proc = subprocess.Popen(
            [self.binary], cwd=self.cwd, env=env,
            stdin=slave, stdout=slave, stderr=slave,
            preexec_fn=_child_setup, close_fds=True,
        )
        os.close(slave)
        self._master = master
        return self

    def close(self, timeout=20.0):
        """送 ^C 让 SITL 干净退出；退不掉再 SIGTERM/SIGKILL。"""
        if self.proc is None:
            return
        if self.proc.poll() is None:
            try:
                os.write(self._master, b"\x03")
            except OSError:
                pass
            deadline = time.time() + timeout
            while self.proc.poll() is None and time.time() < deadline:
                self.pump(0.2)
        if self.proc.poll() is None:
            self.proc.terminate()
            try:
                self.proc.wait(timeout=10)
            except subprocess.TimeoutExpired:
                self.proc.kill()
                self.proc.wait(timeout=10)
        try:
            os.close(self._master)
        except OSError:
            pass
        self._master = None

    def __enter__(self):
        return self.start()

    def __exit__(self, *exc):
        self.close()

    # ---------- 读写 ----------

    def pump(self, timeout=0.2):
        """读一次 pty；返回新读到的字节数。"""
        if self._master is None:
            return 0
        try:
            ready, _, _ = select.select([self._master], [], [], timeout)
        except InterruptedError:
            return 0
        if not ready:
            return 0
        try:
            chunk = os.read(self._master, 65536)
        except OSError:
            # EIO = 从端已关闭（进程退出）
            self.exited = True
            return 0
        if not chunk:
            self.exited = True
            return 0
        self._raw += chunk
        return len(chunk)

    def text(self):
        """剥掉 ANSI 转义、并把 pxh 的行内重画收成一行之后的全文。

        pxh 的行编辑每读一个字符就 `_clear_line()`（`ESC[2K\\r`）+ 重画整行，
        所以一条 40 字符的命令会在原始流里留下 40 段回显。终端上它们互相覆盖，
        但字节流里全是噪声（ticket 01 那次管道喂命令刷出 64 MB 就是这个）。
        这里按 `\\n` 切行、每行只留最后一个 `\\r` 之后的那段 —— 正则和日志就都干净了。
        """
        plain = ANSI_RE.sub("", self._raw.decode("utf-8", "replace"))
        # 注意 ONLCR：pty 输出侧把 `\n` 补成 `\r\n`，所以行尾会多一个空段 ——
        # 取"最后一个**非空**段"，不能直接取最后一个。
        lines = []
        for line in plain.split("\n"):
            lines.append(next((seg for seg in reversed(line.split("\r")) if seg), ""))
        return "\n".join(lines)

    def clear(self):
        self._raw.clear()

    def send(self, cmd, settle=0.15):
        """发一条命令并回车。"""
        if self._master is None:
            raise Px4ConsoleError("还没启动")
        os.write(self._master, cmd.encode() + b"\r")
        self.pump(settle)

    def wait_for(self, pattern, timeout=10.0, since=0):
        """等正则出现（在 `since` 之后的文本里找）；返回 Match，超时抛错。"""
        rx = re.compile(pattern)
        deadline = time.time() + timeout
        while True:
            m = rx.search(self.text()[since:])
            if m:
                return m
            if self.proc is not None and self.proc.poll() is not None:
                raise Px4ConsoleError(
                    f"SITL 提前退出（找 {pattern!r}）:\n{self.tail()}")
            if time.time() > deadline:
                raise Px4ConsoleError(
                    f"超时 {timeout}s 没等到 {pattern!r}，最后输出:\n{self.tail()}")
            self.pump(0.2)

    def wait_ready(self, timeout=60.0):
        """等 rcS 跑完（"Startup script returned successfully"）。"""
        return self.wait_for(r"Startup script returned successfully", timeout)

    def tail(self, n=4000):
        t = self.text()
        return t[-n:]

    def drain(self, seconds=1.0):
        """空转一段时间，把输出读干净。"""
        deadline = time.time() + seconds
        while time.time() < deadline:
            self.pump(min(0.2, max(0.01, deadline - time.time())))


def parse_overlay_into_env(overlay):
    """把覆盖层文件放到一个临时目录里当 `px4-rc.params`，返回 (临时目录, 环境变量)。

    rcS 用 `. px4-rc.params` **按名字**找文件（PATH 里搜，`rcS:236`），而 PATH 尾部才是
    `${R}etc/init.d-posix` —— 所以只要把一个目录前置到 PATH，就能让 SITL 读出
    我们指定的那一套参数。

    ⚠️ 这条规则**另有一份 bash 实现**（`run_sitl.sh:94-96`），改规则要两处一起改。
    详见 docs/control/sitl-z0-plant.md §3。
    """
    import tempfile

    if overlay is None:
        return None, None
    tmp = tempfile.mkdtemp(prefix="px4-rc-params.")
    shutil.copyfile(overlay, os.path.join(tmp, "px4-rc.params"))
    return tmp, {"PATH": tmp + os.pathsep + os.environ.get("PATH", "")}


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--cmd-file", required=True,
                    help="每行一条要送进 pxh 的命令；`#` 开头是注释，空行是空转 0.5 s")
    ap.add_argument("--src", default=os.path.expanduser("~/refs/px4-v1.14.3-build"))
    ap.add_argument("--build-dir", default=None)
    ap.add_argument("--model", default="sihsim_quadx")
    ap.add_argument("--plant", default=None,
                    help="z0-plant 下的参数覆盖层文件名（如 z0.param）")
    ap.add_argument("--ready-timeout", type=float, default=60.0)
    ap.add_argument("--total-timeout", type=float, default=180.0)
    args = ap.parse_args(argv)

    build_dir = args.build_dir or os.path.join(args.src, "build/px4_sitl_default")
    env = {"PX4_SIM_MODEL": args.model, "PX4_SIMULATOR": "sihsim"}

    overlay_path = None
    if args.plant:
        overlay_path = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                    "z0-plant", "params", args.plant)
    tmpdir, overlay_env = parse_overlay_into_env(overlay_path)
    if overlay_env:
        env.update(overlay_env)

    px = Px4Console(cwd=build_dir, env=env)
    t0 = time.time()
    try:
        px.start()
        px.wait_ready(args.ready_timeout)
        print(f"[px4_console] SITL 起来了（{time.time() - t0:.1f}s）", file=sys.stderr)
        with open(args.cmd_file, encoding="utf-8") as fh:
            for line in fh:
                line = line.strip()
                if not line or line.startswith("#"):
                    px.drain(0.5)
                    continue
                print(f"[px4_console] >>> {line}", file=sys.stderr)
                px.send(line)
                px.drain(1.0)
                if time.time() - t0 > args.total_timeout:
                    print("[px4_console] 到总时限，停", file=sys.stderr)
                    break
        px.drain(2.0)
    finally:
        px.close()
        if tmpdir:
            shutil.rmtree(tmpdir, ignore_errors=True)

    sys.stdout.write(px.text())
    return 0


if __name__ == "__main__":
    sys.exit(main())
