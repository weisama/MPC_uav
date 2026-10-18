#!/usr/bin/env python3
"""z0 起链菜单 —— 把「天天要手打的那几条」收进一个入口（ticket 22）。

    run/z0.py                 # 交互菜单（敲数字）
    run/z0.py --list          # 只列菜单，不进入交互（给脚本/记录用）
    run/z0.py --dry-run mavros  # 只打印将要执行的命令，不执行
    run/z0.py env|check|mavros|vrpn|mocap|traj|control|up|up_manual|down
                              # ↑ 菜单里前 10 项（`up` = 全部程序、`up_manual` = 手动遥控飞）
    run/z0.py rec_start 5     # ↑ 第 11 项：开始录制（场景 1–6）
                              #   （实现在 `run/rec.py` —— **只订阅**，见那一份的文件头）
                              #   ⚠️ 它**占住终端**：屏幕上是倒计时，场景 1–5 录满 120 s 自己
                              #      收口，想提前停按 Ctrl-C。选 **6（任意录制）不预设场景/时长**，
                              #      只有 Ctrl-C 收口。**没有**"停止录制"这个菜单项了。
    run/z0.py bringup|fly|rec_status|rec_manifest
                              # ↓ 四条**不进菜单**的（命令仍在 ACTIONS 里，见下面的注释）

★ 它替你做掉的三件事：

  ① **`source` 前缀**：每个动作都跑在 `bash -c 'source … && <命令>'` 里，
     三条 `setup.bash` 的顺序与 [`bench-runbook.md`](../docs/control/bench-runbook.md) 的「起链」一致；
     `env` / `check` 这两项要在**本进程**里跑（rclpy / ament_index），所以在没 source 过的环境里
     会**带 source 重新 exec 自己**一次（`Z0_SOURCED=1` 防重入）—— 判据是"环境 source 过没有"，
     不是"能不能 import rclpy"：本机存在**半拉子环境**（`PATH`/`PYTHONPATH` 有 `/opt/ros`、
     `AMENT_PREFIX_PATH` 里却没有工作空间），那时自检会报"六个包全查不到"而 launch 其实跑得起来。
  ② **每步的核对**：`check` 一张表给出"哪个话题有帧、多少 Hz、`/mavros/state` 里
     `connected/armed/mode` 是什么" —— 判据来自各包的 `config/*.yaml`，不是猜的。
  ③ **两条容易忘的前提**：起 `mocap` 之前**飞机必须已摆好并静止**（阶段 A 原点在节点启动那一刻锁定，
     见 [`z0.launch.py`](../src/bringup/launch/z0.launch.py) 头注释）；`/dev/ttyTHS1` 只能有一个 mavros。
     另外两条**可选**参数的出口（`gcs_url` 桥给 QGC、`use_vrpn:=false`）做成了起之前问一句 y/N，
     值写在动作表里（`OptionalArg`）—— 有意**不给"随手输一段参数"**的自由，那等于把任意 shell
     片段送进 `bash -c`，与下面这条边界相冲。

★ **两条「一键」**（第 8 / 9 项，ticket 22 §七 / §九）—— 命令仍只从 `ACTIONS` 取（见下）：

   · **第 9 项 `up`**（`run/up.sh`）＝ mavros（带 GCS 桥）→ vrpn → mocap **+ 起链前选出来的尾巴**
     **按序起 + 每步等判据**，起完**把终端让给操作台 `fly.py`** 选轨迹，然后打印核对表 +
     起飞提示并前台守着。★ 起什么**只写 key**（`MANUAL_CHAIN` + 选中的那一档的 `tail`）。
   · **第 8 项 `up_manual`** ＝ 只到 `mocap` 为止、**不问**（2026-10-16 复原，见
     `.scratch/phase4/issues/14-*.md`）⟹ 没有任何程序在上位控制飞机，起完**直接用遥控器手动飞**。
   · ★ 两条**都不代做"起飞"**：选轨迹是在让出去的前台里敲 1/2/3/4，起飞是**遥控器开始键**。
   · 第 9 项的两档（**两档都带 `traj`** —— "轨迹必须起"是结构上的事实，不是提示语）：
     **1 offboard 控制**（`control` 自己发 PX4 setpoint，`traj → control`，**默认档**）/
     **2 MPC**（`mpc_node.py` 作为 `control` 的输出后端，`traj → mpc → control`）。
     ★ 这个词有歧义（见 `CONTEXT.md` 第一节：PX4 的 `MPC_*` 参数是**位置控制器**的，不是这个）
     ⟹ 指代本项目的控制器时一律写 `MPC`（首次出现可写 `Model Predictive Control（MPC）`）。
     ★★ **两档的现场动作逐字相同**：`control` 始终是任务总管（开始键、解锁、切 OFFBOARD、起飞、
     判稳、轨迹原点、`/traj/start`、降落）；第 2 档只是把「谁最后给 PX4 发指令」换成 `mpc_node`。
     第 2 档的命令带三条开关（`reference_source_control` / `offboard_prestream` /
     `output_mode_external`）；`start_traj_reference` **不再带**（`/traj/start` 归 `control`）。
     理由与一手依据见 `.scratch/phase4/issues/15-trajectory-flight-roles.md`。

★★ **它不提供任何"让飞机动"的入口 —— 解锁 / 起飞 / 切模式 / 改参数 / 发帧，一个都没有。**
   ★ **2026-10-13 的一处口径澄清（票 13 第九轮，操作者裁定）**：菜单里**现在仍然没有**这一类入口，
   但第 1 项「起 mavros」从今天起走**本仓的** `bringup/launch/mavros.launch.py`，而那一条
   **在起链时会顺手把 mavros 自己的 `thrust_scaling` 设成 `1.0`**
   （`plugin.cpp:35` 的 `use_global_arguments(false)` 让 `px4_config.yaml` 到不了插件节点 ⟹
   不补这一刀整条下行是死的：`thrust != 0` 的帧被**整帧丢掉**、零操纵、几乎无声）。
   ⟹ 它改的是 **mavros 的一个缩放系数**（不是飞控参数、不发帧、不动模式），
   且**是链路能工作的前提** ⟹ 归在"起链"里，不归在"改参数"里。**这个边界是操作者裁的，不是自解释的。**
   ⛔ 因此**不要**再把这条命令改回上游 `px4.launch`（改回去 = 悄没声地把下行关掉）。
   两条理由写在仓库里，不是洁癖：`z0.launch.py` 的「本包**不给**"程序起飞"的入口，因为它会绕开
   "开始键之前不发控制指令"这条防线」、`bench-runbook.md` 的「起完之后**还差两步**（刻意不自动化）」。
   ⟹ 台架上的 `/control/takeoff` 照旧手打（`bench-runbook.md` T4 有原文），本脚本只提一句"在哪"。
   这条由 `run/tests/test_z0_menu.py` 的两条用例盯住（扫动作表 + 扫本文件源码）。

⚠️ **本脚本不是 `bench-runbook.md` 的替代品**：那张表是**判据与记录**（做一遍、填读数、勾框），
   本脚本只是**把命令送出去**并把读数摊开给人看。谁先谁后见 `run/README.md`。

★ **第 10 项「清除后台节点」（`down` / `run/down.sh`，ticket 22 §八）**：与第 8/9 项对称的另一头 ——
   `up` 起、`down` 停。它**按名字扫**（`DOWN_TARGETS` 那张白名单，`pgrep -f`），
   所以**别人会话 / 跑着的测试**起的同类节点也会在清单里（现场只有一套飞机，这个代价是明说过的）；
   顺序是**先列清单 ⟹ 一次 y/N ⟹ SIGINT →（超时）SIGTERM →（超时）SIGKILL**，只发**单个 pid**
   不发进程组（按名字扫出来的进程可能与别人终端的 shell 同组）。⚠️ **停进程不是降落**：
   空中先按 `operation_manual.md` **§1.2**（两条软路径）处置，理由写在 `DOWN_WARNING` 里。

★ **第 11 项「录制」（ticket 19）**：开始录制（选场景 1–6）。实现在 [`rec.py`](rec.py)，
   是**订阅侧**的动作 —— `ros2 bag record`，**没有多出任何"让飞机动"的入口**（不解锁、
   不切模式、不发帧、不改参数），所以上面那条安全边界原样成立。为什么要有这一层写在
   `rec.py` 的文件头：B 档（ticket 11）要的那份真实动捕 bag **从来没录过**，理想轨迹替不了它。
   ★ **它占住终端**（操作者 2026-09-29 拍定）：屏幕上是倒计时，场景 1–5 录满 120 s 自己收口，
   想提前停按 Ctrl-C —— 因此菜单里**没有**"停止录制"这一项；录制期间也不该在同一个终端里
   敲别的菜单项。看进度请在**另一个终端** `run/z0.py rec_status`。
   ★ **六条场景**（操作者 2026-10-08 拍定）：`1` 静止 / `2` 手飞起飞 / `3` 轨迹 1（悬停）/
   `4` 轨迹 2（正方形）/ `5` 轨迹 3（八字）/ `6` **任意录制**。场景 6 就是原来那句"自己想录什么
   录什么"：**不预设场景、不预设时长**，起录之后**只有 Ctrl-C 才收口**，落进一份带 UTC 时间戳的
   新目录（⟹ 同一个场景想录几遍就录几遍，早已入账的那几份一个字节都不碰）。
   ⚠️ 场景 4/5 **重新编过号**：`bag/4_traj3_lemniscate/` 那份 09-29 入库的八字包**照旧叫这个名字**
   （数据不许改名），靠 `rec.SLUG_ALIASES` 仍认得出它是现在的场景 5。
   ★ 2026-10-08 又清过一遍 `bag/`（每个场景只留最新的一份）⟹ 那份搬进了
   `bag/old/4_traj3_lemniscate/`，**`bag/` 一层里已经没有这个目录名了**（见 `bag/README.md`）。
   ★ 有意**不让本脚本自己压包 / 自己拉飞控日志**：压缩是 `ros2 bag record` 收口时自己做的事，
   飞控日志由操作者**拔 SD 卡**直接拷（mavros 独占串口 ⟹ 本机没有第二条 MAVLink 通道走 FTP），
   拷进 `bag/<场景>/fcu/` 之后总表会自己列出来。
"""

from __future__ import annotations

import argparse
import importlib
import os
import shutil
import signal
import subprocess
import sys
import time
import unicodedata
from dataclasses import dataclass
from pathlib import Path
from typing import IO, Callable, Dict, Iterable, List, Optional, Sequence, Tuple

# ─────────────────────────── 常量（都是第一手的路径/值） ───────────────────────────

#: 仓库根 = `run/` 的上一级。★ 本脚本**必须在仓库里**跑：install/ 与 docs 都相对它。
REPO = Path(__file__).resolve().parents[1]

#: `run/` 自己 —— 两条一键的日志写在它下面（`run/logs/<时间戳>-<链名>/`，已被 .gitignore 挡住）。
RUN_DIR = Path(__file__).resolve().parent

ROS_DISTRO = os.environ.get("ROS_DISTRO", "humble")
ROS_SETUP = Path(f"/opt/ros/{ROS_DISTRO}/setup.bash")

#: mavros 装在**另一个工作空间**（源码构建，`bringup/README.md` 第 3 条）。换机器用 `Z0_MAVROS_WS` 覆盖。
MAVROS_WS = Path(os.environ.get("Z0_MAVROS_WS", "~/mavros_ws")).expanduser()

#: 飞控串口。★ 定案（ticket 01 ② / 13）：z0 只有 THS1/THS2，心跳只在 **THS1 @ 921600** 上探到。
#: ⚠️ 设备名是每台机器各自实测出来的 —— 换机器要重扫（上游那台用的是 ttyTHS3）。
SERIAL_PORT = "/dev/ttyTHS1"
FCU_URL = SERIAL_PORT + ":921600"

#: 把 MAVLink 桥给笔记本上的 QGC 时用的默认地址（第一手：`docs/reference/mocap-setup.md` §5.4 实测口径）。
#: 现场网段不同就改这里，或用 `Z0_GCS_URL` 覆盖。
GCS_URL = os.environ.get("Z0_GCS_URL", "udp-b://0.0.0.0:14555@:14550")

#: ★★ acados 的两个环境变量 —— `mpc_node.py` 建 OCP 求解器时**当场要**它们（`ocp.py` 的
#: `_build_solver` ⟹ `AcadosOcpSolver` ⟹ `ctypes.CDLL(<ACADOS_SOURCE_DIR>/lib/libacados.so)`）。
#: **2026-10-17 实测**：第 9 项起链死在第五步，两个都在 shell 里没设 ——
#:
#: · 缺 `ACADOS_SOURCE_DIR` ⟹ `acados_template` 从**它自己被装在哪**去猜（本机是 `~/refs/acados`，
#:   那是 **acados 源码树**、`lib/libacados.so` 根本不在）⟹
#:   `OSError: /home/chen/refs/acados/lib/../lib/libacados.so: cannot open shared object file`；
#: · 只设它、缺 `LD_LIBRARY_PATH` ⟹ `libacados.so` 打得开，但它依赖的 `libhpipm.so` 找不到
#:   （同一个 `OSError`，只是文件名不同 —— 这条是票 13 踩过的）。
#:
#: ⟹ **写进命令本身**，不靠操作者记着先 export（起链是"一键"，那就得真的是一键）。
#: `${ACADOS_SOURCE_DIR:-$HOME/acados}`：外层**已经**指对了就听外层的（同
#: `src/nmpc/test/test_mpc_node.py` 的 `_dut_env()`）；`${LD_LIBRARY_PATH:+:…}` 是为了
#: **不**在末尾留一个空路径元素（空元素 = 当前目录，ld.so 的规矩）。
ACADOS_ENV = ("ACADOS_SOURCE_DIR=\"${ACADOS_SOURCE_DIR:-$HOME/acados}\" "
              "LD_LIBRARY_PATH=\"${ACADOS_SOURCE_DIR:-$HOME/acados}/lib"
              "${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}\"")

#: `install/` 里应当有的包（全部是 Phase 1 的：动捕源 + 三个节点 + 夹具 + 启动层）。
EXPECTED_PACKAGES = ("vrpn_listener", "mocap", "traj", "bench", "control", "bringup")

#: 核对时订阅的话题。`(话题, 期望读数的一句话)`；`expected` 里的"本该是空的"会被单独标出来。
#: 话题名全部来自各包的 `config/*.yaml`（`control.yaml:41-88`、`mocap.yaml`、`traj.yaml`、`mocap.yaml` 的
#: `vision_pose_topic`），期望频率来自 `traj.yaml:rate_hz=50` 与实测的动捕 ~200 Hz。
PROBES: Tuple[Tuple[str, str], ...] = (
    ("/mavros/state", "1 Hz 是**正常**的（mavros 的连接心跳）"),
    ("/mavros/local_position/pose", "EKF2 位置 —— 控制律的位置来源"),
    ("/mavros/vision_pose/pose", "动捕 → EKF2 的载体；断了 EKF2 就没有视觉"),
    ("/mocap/pose_enu", "mocap 节点的相对位姿输出（动捕实测 ~200 Hz）"),
    ("/control/state", "control 状态机；`MANUAL/INIT` 是**正常**的起始态，不是故障"),
    ("/traj/reference", "参考流 50 Hz —— **没选轨迹时本该是空的**"),
    ("/mavros/setpoint_raw/local", "我们发出去的 setpoint，≈50 Hz ⟹ 说明正在流"),
)

#: 期望为空的话题：没帧不算错（`PROBES` 里那句"本该是空的"就是判据）。
OPTIONAL_TOPICS = ("/traj/reference",)


# ─────────────────────────── 纯函数（被 run/tests 直接测） ───────────────────────────


def display_width(text: str) -> int:
    """串里有多少**终端列**。中文/全角字符占两列 —— `str.ljust` 只数**字符**，用它排版必然错位。"""
    return sum(2 if unicodedata.east_asian_width(ch) in ("W", "F") else 1 for ch in text)


def pad(text: str, width: int) -> str:
    """按终端列宽左侧补齐（`ljust` 的中日韩版本）。"""
    return text + " " * max(0, width - display_width(text))


def parse_lsof_fpcf(text: str) -> List[str]:
    """把 `lsof -Fpcf <设备>` 的输出解成 `["python3(32607)"]`。

    第一手（2026-09-23）：`lsof -Fpcf /dev/ttyTHS1` 的输出形如
    `p32607\\ncpython3\\nf3\\n` —— `p<pid>` / `c<command>` / `f<fd>` 三类行，
    一个进程可能有多行 `f`（多个 fd）。
    """
    holders: List[str] = []
    seen: set = set()
    pid = cmd = None
    for line in text.splitlines():
        line = line.strip()
        if not line:
            continue
        tag, value = line[0], line[1:]
        if tag == "p":
            pid, cmd = value, None
        elif tag == "c":
            cmd = value
        elif tag == "f" and pid is not None and pid not in seen:
            # 一个进程可能有多行 `f`（多个 fd）⟹ 按 pid 去重，别把同一个人报两遍
            seen.add(pid)
            holders.append(f"{cmd or '?'}({pid})")
    return holders


def vrpn_body_names(topics: Iterable[str]) -> List[str]:
    """从 `/vrpn/<刚体>/pose` 里取出刚体名（原样，**不做大小写归一化**）。"""
    names = []
    for topic in topics:
        parts = topic.strip("/").split("/")
        if len(parts) == 3 and parts[0] == "vrpn" and parts[2] == "pose":
            names.append(parts[1])
    return sorted(set(names))


def case_variant_note(bodies: Sequence[str]) -> Optional[str]:
    """同名不同大小写时给一句提醒。

    ★ 第一手（`mocap/config/mocap.yaml` 的文件头）：2026-09-21 一天之内实测到 `Z0`（大写）
    与 `z0`（小写）两个刚体名 —— 话题名原样取自 VRPN 的 sender 名，
    **Nokov 侧重命名一次，`mocap.yaml` 的 `body_name` 就得跟着改**。
    """
    lowered: Dict[str, set] = {}
    for name in bodies:
        lowered.setdefault(name.lower(), set()).add(name)
    for variants in lowered.values():
        if len(variants) > 1:
            return (f"⚠️ 图里同时有 {'、'.join(sorted(variants))} —— "
                    "刚体名不做大小写归一化，`mocap.yaml` 的 `body_name` 只认其中一个")
    return None


def domain_note(domain: Optional[str], localhost_only: Optional[str] = None) -> str:
    """ROS 域的一句话解读。

    ★ 台架夹具（`bench/pytest_support.py`）默认 **73**、**硬拒 0**，且强设 `ROS_LOCALHOST_ONLY=1` ——
    两条都是为了"别把测试里的假解锁打到真飞机上"。⟹ 域号不是洁癖，是安全边界，
    读数前先说清"这是哪一个域"。
    """
    if domain in (None, ""):
        domain = "0"
    if domain == "0":
        note = "真机域（默认）"
    elif domain == "73":
        note = "⚠️ **台架夹具域**（bench 默认）—— 这里的读数是夹具造的，不是真飞机"
    else:
        note = "⚠️ 非默认域（既不是 0 也不是 73）"
    if localhost_only == "1":
        note += "；`ROS_LOCALHOST_ONLY=1` ⟹ **只在本机发现**，跨机（动捕主机）的话题看不见"
    return f"ROS_DOMAIN_ID={domain} —— {note}"


def describe_message(msg: object) -> str:
    """把一帧消息压成一行给人看的字。

    只认三族（覆盖 `PROBES` 里的全部话题）：
    `mavros_msgs/State`（`connected/armed/mode`）、带 `.data` 的（`std_msgs/String`/`Float32`）、
    带位置字段的 —— `PoseStamped` 是 `.pose.position`，`PositionTarget`
    （`/mavros/setpoint_raw/local` 的载荷）**直接就是 `.position`**（第一手：
    `mavros_msgs/msg/PositionTarget.msg:28`）。认不出就报类名 —— **不猜**。
    """
    if hasattr(msg, "connected") and hasattr(msg, "mode"):
        return (f"connected={getattr(msg, 'connected')} "
                f"armed={getattr(msg, 'armed')} mode={getattr(msg, 'mode')}")
    if hasattr(msg, "data"):
        return str(getattr(msg, "data"))
    position = getattr(getattr(msg, "pose", None), "position", None)
    if position is None:
        position = getattr(msg, "position", None)
    if position is not None:
        return (f"x={position.x:+.3f} y={position.y:+.3f} z={position.z:+.3f}"
                if all(isinstance(getattr(position, a, None), (int, float))
                       for a in ("x", "y", "z")) else "有位姿（字段不全）")
    return type(msg).__name__


def rate_text(count: int, window_s: float) -> str:
    """一帧没收到时说"无帧"，而不是 `0.00 Hz` —— 两者在现场的含义完全不同。"""
    if count <= 0:
        return "无帧"
    return f"{count} 帧 / {window_s:.0f} s ≈ {count / window_s:.0f} Hz"


def serial_line(serial: dict, *, occupied: str, idle: str = "") -> str:
    """串口那一行的三种状态 —— **一处写、两处用**（判据只有一份，别再抄一遍）。

    `occupied` / `idle` 是两种场合的补充说明：`check` 要讲清"第二个 mavros 为什么打不开"，
    `env` 只要讲"是谁占着"。
    """
    path = serial.get("path")
    holders = serial.get("holders") or []
    if not serial.get("exists"):
        return f"✗ 串口 {path} 不存在 —— mavros 起不来（换机器要重扫，见 ticket 01/13）"
    if holders:
        return f"⚠️ 串口 {path} 被占用：{'、'.join(holders)} —— {occupied}"
    return f"· 串口 {path} 空闲" + (f"（{idle}）" if idle else "")


def render_snapshot(snap: dict) -> str:
    """把 `collect_snapshot()` 的读数摊成一张表（**纯函数**，可离线测）。

    表里三档记号：
    `✓` 有帧 ｜ `✗` 无帧（非可选话题 ⟹ 缺东西）｜ `·` 无帧但**本该是空的**。
    """
    lines: List[str] = []
    window_s = snap.get("window_s", 0.0)
    lines.append(domain_note(snap.get("domain"), snap.get("localhost_only")))
    lines.append(serial_line(snap.get("serial") or {},
                             occupied="第二个 mavros 打不开它，而那个报错**长得像 launch 坏了**",
                             idle="mavros 还没起，或已经起好了"))

    topics: Dict[str, str] = snap.get("topics") or {}
    counts: Dict[str, int] = snap.get("counts") or {}
    last: Dict[str, str] = snap.get("last") or {}
    vrpn_topics = [t for t in counts if t.startswith("/vrpn/")]
    missing: List[str] = []

    lines.append("")
    lines.append(pad("话题", 34) + pad("读数", 24) + "说明")
    for topic, expected in list(PROBES) + [(t, "动捕刚体的原始位姿（~200 Hz）") for t in vrpn_topics]:
        if topic not in counts:
            if topic.startswith("/vrpn/"):
                continue
            silent_is_fine = topic in OPTIONAL_TOPICS
            if topic not in topics:
                reason = f"{topic}（话题都不在图里 ⟹ 发布者没起）"
            else:
                reason = f"{topic}（在图里但没有发布者）"
            if not silent_is_fine:
                missing.append(reason)
            mark = "·" if silent_is_fine else "✗"
            lines.append(f"{mark} " + pad(topic, 32) + pad("—", 24) + expected)
            continue
        count = counts.get(topic, 0)
        optional = topic in OPTIONAL_TOPICS or topic.startswith("/vrpn/")
        mark = "✓" if count > 0 else ("·" if optional else "✗")
        if count <= 0 and not optional:
            # 订得上、一帧不来 ⟹ 要么发布者没起，要么 QoS 不兼容（后者**不报错**，
            # 见 bench/README 的「QoS 不一致会静默不匹配」）。两种都说出来，别让人少查一条。
            missing.append(f"{topic}（在图里但一帧都没来 ⟹ 发布者没起，或 QoS 不兼容）")
        detail = last.get(topic) or expected
        lines.append(f"{mark} " + pad(topic, 32) + pad(rate_text(count, window_s), 24) + detail)

    note = case_variant_note(vrpn_body_names(topics))
    if note:
        lines.append("")
        lines.append(note)

    lines.append("")
    if missing:
        lines.append("缺什么：")
        for item in missing:
            lines.append(f"  ✗ {item}")
    else:
        lines.append("这一侧齐了 —— 但**齐了不等于对**：判据仍按 `bench-runbook.md` 走。")
    lines.append("")
    lines.append("下一步（★ 脚本刻意不代劳，见文件头）：")
    lines.append("  ① 选轨迹：`ros2 run bringup fly.py` 敲 1/2/3（★ 起飞前选好）")
    lines.append("  ② 遥控器开始键第 1 次上升沿 = 解锁 + 起飞，第 2 次 = 进 TRACK")
    lines.append("     （每次压住 ≥ 0.3 s 再松；台架上没有接收机时走 bench-runbook.md T4 的服务）")
    lines.append("")
    lines.append("两件**本脚本查不到、但会拦住解锁**的事（都在 bench-runbook.md P0）：")
    lines.append("  · SD 卡根目录的 `fault_*.log`：PX4 会拒绝解锁（`vehicle needs service`），"
                 "且**删档不当场生效**。")
    lines.append("    ⚠️ 本脚本读不了它：mavros **独占**串口 ⟹ 本机没有第二条 MAVLink 通道走 FTP，")
    lines.append("    看它只能靠 QGC（或把卡拔下来）。⚠️ 转储**同名覆盖** ⟹「只有一个」推不出故障次数。")
    lines.append("  · 参数（`NAV_RCL_ACT` 等）：`params-changes.md` 的 A1/§I 那张表，用 QGC 或 mavros 对账。")
    return "\n".join(lines)


# ─────────────────────────── 环境：`source` 与前台执行 ───────────────────────────


def existing_setups() -> List[Path]:
    """按 `bench-runbook.md`「起链」的顺序，列出**真实存在**的三条 `setup.bash`。

    缺哪条就跳过哪条（`env` 会把它报出来）—— 不假装 source 过。
    """
    candidates = [ROS_SETUP, MAVROS_WS / "install/setup.bash", REPO / "install/setup.bash"]
    return [p for p in candidates if p.exists()]


def sourced_command(inner: str, setups: Optional[Sequence[Path]] = None) -> str:
    """把一条命令包成「先 source 再跑」——这是本脚本存在的主要理由之一。"""
    prefixes = [f"source {p}" for p in (existing_setups() if setups is None else setups)]
    return " && ".join(prefixes + [inner])


#: 前台子进程收到 Ctrl-C 后给它的**收摊时间**；到点才升级 SIGTERM → SIGKILL。
FOREGROUND_GRACE_S = 10.0


def run_foreground(inner: str) -> int:
    """在前台跑（Ctrl-C 停它，本脚本收得住 —— 见 `menu_loop`）。

    ★ Ctrl-C **不走 `subprocess.call`**：它被 `KeyboardInterrupt` 打断时会
      `p.kill()`（SIGKILL）子进程（见 [`call_ignoring_sigint`](run/z0.py) 那段第一手）。
      子进程与我们同一个前台进程组 ⟹ 终端那一下它**自己**也收到了，让它走自己的收摊逻辑
      （`fly.py` 就是靠这个把节点销毁、`rclpy.shutdown()` 的）；这里先等
      `FOREGROUND_GRACE_S`，等不到才 SIGTERM → SIGKILL。

    ★ 收摊这段时间里 SIGINT 被挡住（同 `run/rec.py` 的 `_sigint_ignored`）：否则操作者
      再按一下就会把本进程打死，留下一棵半死不活的子树。⚠️ 升级只发给**直接子进程**
      （它和我们在同一个进程组里，对整个组发信号会连菜单一起收掉）—— 与修之前
      `p.kill()` 的范围一样，没有变差。
    """
    proc = subprocess.Popen(["bash", "-c", sourced_command(inner)])
    try:
        return proc.wait()
    except KeyboardInterrupt:
        pass
    print(f"\n（Ctrl-C：等它自己收摊，最多 {FOREGROUND_GRACE_S:.0f} s）")
    try:
        old = signal.signal(signal.SIGINT, signal.SIG_IGN)
    except (ValueError, OSError):              # 非主线程之类：挡不住也照常走下面的逻辑
        old = None
    try:
        try:
            return proc.wait(timeout=FOREGROUND_GRACE_S)
        except subprocess.TimeoutExpired:
            pass
        for sig, name in ((signal.SIGTERM, "SIGTERM"), (signal.SIGKILL, "SIGKILL")):
            print(f"  {name} … ", end="", flush=True)
            try:
                proc.send_signal(sig)
            except ProcessLookupError:
                print("（它已经退了）")
                break
            try:
                return proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                continue
        return proc.returncode if proc.returncode is not None else 1
    finally:
        if old is not None:
            signal.signal(signal.SIGINT, old)


def call_ignoring_sigint(argv, *, env=None) -> int:
    """等一个子进程；等待期间收到的 Ctrl-C **只在外壳这一侧被吞掉** —— 只给"外壳等真正的自己"用。

    ⛔ **别退回裸 `subprocess.call`**：Python 3 的 `subprocess.call` 被 `KeyboardInterrupt`
    打断时会 **`p.kill()`（SIGKILL）子进程**（本机 3.10.12 的 `subprocess.py:347` 就是
    `with Popen(...) as p:` + `except: p.kill()`）。而 Ctrl-C 发的是**整个前台进程组**
    ⟹ 外壳与真正的菜单进程会同时收到；菜单进程正要收口，外壳却反手把它 SIGKILL 掉。

    ★★ 第一手（2026-10-17 现场，真的丢了一份 `session.json`）：`rec_start 6` 录到一半
    按了**一次** Ctrl-C，屏幕上是

        收口中（写缓存 → 压 zstd）… chen@orin-nx:~/ros_ws/run$

    —— 那行后面紧跟着 shell 提示符，`好（SIGINT）`／核对表／`清单已更新` 一行都没有；
    `bag/<包>.recorder.json` 没被删、录制进程成了孤儿（PPID=1）。包本身是好的
    （录制进程收到了菜单进程终死前发出的 SIGINT），丢的只是收口记录。
    离线复现：外层 `subprocess.call(["bash", "-c", "exec python3 child.py"])`，
    给外层发**一次** SIGINT ⟹ child **直接消失**，外层退出码 130。

    ⛔⛔ **也别退回"先 `signal.signal(SIGINT, SIG_IGN)` 再起子进程"** —— 那是本函数的
    **上一版**，2026-10-17 当天就在同一台机器上翻车，比它要修的那个 bug 更凶：
    **被忽略的信号会被子进程继承**（`fork`+`exec` 一路带着），而 CPython **只有当启动时
    SIGINT 不是 `SIG_IGN` 才装** `KeyboardInterrupt` 处理器 ⟹ 内层菜单从启动起就一直
    "看不见" Ctrl-C，整个会话都停不下来。现场：`rec_start 6` 已挂两分多钟，操作者按
    Ctrl-C **毫无反应**；`/proc/<内层 pid>/status` 是 `SigIgn: …0002`（bit 1 = SIGINT
    被忽略）、`SigCgt: 0`（连 Python 的处理器都没装）—— 与旁边那个正常菜单的
    `…1001000` / `…4002` 一比就现形。

    ⟹ 先把子进程 **`Popen` 出来**（它拿到**默认**处置），再在父进程这一侧把
    `KeyboardInterrupt` 吞掉、接着等。父进程不动任何处置，子进程就不可能继承到坏状态；
    菜单、录制收口、起链的 `Ctrl-C 一起停` 行为都不变（子进程仍在同一个前台进程组）。
    """
    proc = subprocess.Popen(argv, env=env)
    while True:
        try:
            return proc.wait()
        except KeyboardInterrupt:
            # 外壳这侧收到了；真正的菜单进程**也**收到了（同一个前台进程组）——
            # 它该收口就收口、该停就停。外壳只负责**别走、更别杀它**。
            print("\n（Ctrl-C：等它自己收口，别急）", end="", flush=True)
            continue


def ensure_sourced() -> None:
    """带 source **重新 exec 自己**一次，让本进程的环境与"起链"那几条命令一致。

    为什么不是"只在缺 rclpy 时才重入"（第一手教训，2026-09-23）：本机存在**半拉子环境** ——
    `PATH`/`PYTHONPATH` 里有 `/opt/ros/humble`（所以 `rclpy`/`ament_index_python` 都能 import），
    但 `AMENT_PREFIX_PATH` 里**没有**两个工作空间。那时 `env` 会报"六个包全查不到"，
    而同一台机器上 `ros2 launch control …` 本来跑得起来 —— 一个会撒谎的自检比没有更坏。
    ⟹ 判据改成**环境是不是 source 过的**（`Z0_SOURCED`），不是"能不能 import rclpy"。
    重入是幂等的：`source` 只往前提路径，不动 `ROS_DOMAIN_ID` 之类的取值。

    ⚠️ **它是"把自己 exec 掉"，所以调用点必须排在"读 stdin"之前。** 交互菜单那条路
    （`main` ⟹ `menu_loop`）在进循环**之前**先调一次，就是为了这个 —— 放在动作里做的话，
    用户已经敲进缓冲区的那一行会随旧进程一起没掉（详见 `main` 里那段第一手记录）。
    其余调用点都在各自 `cmd_*` 的**第一句**（早于那里的 `ask_confirm`）⟹ 没有 stdin 可丢；
    而 `launch_action` **根本不需要**它：那条路要 source 的是 `run_foreground` 起的子进程，
    父进程只碰 `lsof` 与 y/N。
    """
    if os.environ.get("Z0_SOURCED") == "1" or not existing_setups():
        return
    env = dict(os.environ, Z0_SOURCED="1")
    code = sourced_command(f'exec "{sys.executable}" "{Path(__file__).resolve()}" '
                           + " ".join(f'"{a}"' for a in sys.argv[1:]))
    # ★ 必须走 `call_ignoring_sigint`（不是裸 `subprocess.call`）：Ctrl-C 同时到外壳与
    #   子进程，外壳的 `subprocess.call` 会把**正在收口的**子进程 SIGKILL 掉。见那个函数。
    sys.exit(call_ignoring_sigint(["bash", "-c", code], env=env))


# ─────────────────────────── 动作表 ───────────────────────────


@dataclass(frozen=True)
class OptionalArg:
    """一条**可选**参数：起之前问一句 y/N，答 y 才追加到命令行尾巴。

    ★ 有意**不做成"随便输一段参数"** —— 那样等于把任意 shell 片段送进 `bash -c`，
    与本脚本"命令出口只有 `ACTIONS` 那几条"的边界相冲（每条的 `suffix` 都写在表里，测试扫得到）。
    `name` 只用于"一键默认带上哪几条"（`UP_OPTIONALS`）与测试指认，不参与命令拼装。
    """

    name: str
    prompt: str
    suffix: str


@dataclass(frozen=True)
class Action:
    """一个菜单项。`cmd=None` ⟹ 在**本进程**里做（只读检查、或像 `up` 那样做编排）。

    `menu=False` ⟹ **不进菜单**（也不占菜单号），但仍在表里 —— 这样"命令的唯一出处是
    `ACTIONS`"这条仍然成立：一键流程要前台调用的 `fly` 就是靠它拿到命令的（见 `UP_HANDOFF`）。
    """

    key: str
    title: str
    cmd: Optional[str]
    note: str
    confirm: Optional[str] = None      # 起之前必须人工确认的那句话
    warn_serial: bool = False          # 起之前先看串口有没有被别人占着
    optionals: Tuple[OptionalArg, ...] = ()   # 起之前逐条问的可选参数（y/N）
    read_only: bool = False            # ★ 只读（`--dry-run` 与"什么都不起"的判据都靠它）
    menu: bool = True                  # ★ 进不进菜单（`fly` 进一键流程、不单独占一项）


#: ★ 这张表是**唯一的**命令出口 —— `run/tests/test_z0_menu.py` 扫的就是它。
#: 加一条之前先读文件头的「不提供任何让飞机动的入口」。
#: ★ **菜单号 = 本表里 `menu=True` 的那些项的顺序**（`MENU_ACTIONS`）⟹ 插队会动号，
#: 跨文件引用菜单号的地方（`operation_manual.md` + 它的守卫）会跟着红。
ACTIONS: Tuple[Action, ...] = (
    Action("env", "环境自检（只读）", None,
           "三条 setup / 六个包 / 串口 / 域号 / 当前 commit",
           read_only=True),
    Action("check", "起链核对（只读，约 5 s）", None,
           "每个话题有没有帧、多少 Hz；state 的 connected/armed/mode",
           read_only=True),
    Action("mavros", "起 mavros", f"ros2 launch bringup mavros.launch.py fcu_url:={FCU_URL}",
           "★ 它**独占**串口；`/mavros/state` **1 Hz 是正常的**；另开终端 `ros2 topic hz /mavros/state` 核对",
           warn_serial=True,
           optionals=(OptionalArg(
               "gcs_url",
               f"要把 MAVLink 桥给笔记本上的 QGC 吗？（y ⟹ 追加 gcs_url:={GCS_URL}）",
               f"gcs_url:={GCS_URL}"),)),
    Action("vrpn", "起 vrpn_listener", "ros2 launch vrpn_listener sync_entity_state.launch",
           "动捕数据源；核对刚体名：`ros2 topic list | grep vrpn`（`Z0`/`z0` 两种都出现过）"),
    Action("mocap", "起 mocap", "ros2 launch mocap mocap.launch.py",
           "★ 阶段 A 原点在**节点启动那一刻**锁定：起之前飞机要摆好、静止；起完别再碰飞机",
           confirm="飞机已摆好、静止、固定？"),
    Action("traj", "起 traj（单独）", "ros2 launch traj traj.launch.py",
           "只发参考、不碰飞控；一帧不发直到被 select + start（start 由 control 发）"),
    Action("control", "起 control（单独）", "ros2 launch control control.launch.py",
           "台架 T3 就是这么起的；起早了停在 `MANUAL/INIT` 并逐条列出缺什么 —— 那是正常状态",
           # ★★ 票 15：只开 `external` 那一条。它是**输出后端**的开关，不动任务流程：
           #   开始键、解锁、切 OFFBOARD、起飞、判稳、轨迹原点、/traj/start、降落全在本节点。
           #   ⚠️ 传的是 **launch 参数**（不是 `--ros-args -p`）：这条命令走的是 `ros2 launch`。
           optionals=(OptionalArg(
               "output_mode_external",
               "要让外部控制器（MPC）发执行器指令吗？"
               "（y ⟹ 追加 output_mode:=external；本节点只发 /control/target）",
               "output_mode:=external"),)),
    Action("up_manual", "★ 一键启动（动捕+mavros，手动遥控飞）", None,
           "mavros（带 GCS 桥）→ vrpn → mocap，逐段等判据；★ **不起 traj/control** ⟹ "
           "没有任何程序在上位控制飞机，起完**直接遥控器手动起飞**"),
    Action("up", "★ 一键启动全部（起前选控制器；默认 offboard 控制）", None,
           "mavros → vrpn → mocap 逐段等判据，**尾巴由起链前那一句选**："
           "1 offboard 控制（**默认**，+traj+control）/ 2 MPC（+traj+mpc+control，"
           "mpc 当 control 的输出后端）；两档**都带 traj 与 control**，起完把**终端让给操作台**"
           "选轨迹，然后你按**遥控器开始键**（第 1 次 = 解锁起飞、第 2 次 = 进 TRACK）；"
           "Ctrl-C 一起停"),
    Action("down", "★ 清除后台节点（扫 mavros/vrpn/mocap/traj/control/mpc 并停掉）", None,
           "按**名字**扫（`DOWN_TARGETS`）⟹ 别人会话起的同类节点也在清单里；先列清单再问一句 y/N；"
           "先 SIGINT 超时才升级 ⟹ **它不是降落**，空中先按 runbook §1.2 处置"),

    # ★★ 录制一项（第 11 项，ticket 19）—— 实现在 `run/rec.py`。
    #   它是**订阅侧**的动作：`ros2 bag record`，**一个会动飞机的入口都没多**
    #   （不解锁、不切模式、不发帧；`run/tests/test_rec.py` 有一条 AST 用例盯着）。
    #   ★ **没有"停止录制"这一项了**（操作者 2026-09-29 拍定删掉 12/13 两项）：
    #     录制是**前台**的一条流程 —— 这一项要**占住终端**，倒计时摆在屏幕上，
    #     录满 120 s 自己收口，想提前停按 Ctrl-C（同一条 SIGINT 收口路）。
    #     飞控日志（`.ulg`）同理改成**手工**：拔 SD 卡直接拷进该场景自己的 `bag/<场景>/fcu/`。
    Action("rec_start", "★ 开始录制（选场景 1–6）", None,
           "★ **它占住终端**：屏幕上是倒计时，录满 120 s **自己收口**（不用再敲「停」），"
           "提前停按 Ctrl-C —— 同一条收口路（SIGINT ⟹ 压包 ⟹ `ros2 bag info` 逐条回对 ⟹ 写 "
           "`session.json` + 更新 `bag/MANIFEST.md`；**收口那十几秒里 Ctrl-C 无效**，"
           "为的是别再丢收口记录）；只**订阅**（`ros2 bag record`，"
           "话题走白名单正则）⟹ 包落 `bag/<场景>/`（**扁平，没有日期层**）；"
           "起录后约 1 s 才订上话题，要紧的动作稍等一下。"
           "★ 六条场景：1 静止 / 2 手飞起飞 / 3 轨迹 1 / 4 轨迹 2 / 5 轨迹 3 / "
           "**6 任意录制**（不预设场景与时长，**只有 Ctrl-C 收口**，落进带 UTC 时间戳的新目录）；"
           "⚠️ 首选目录已被占（例如 09-29 那几份已入账的包）时**不再当场退出**，"
           "而是问一句 y/N，答 y 就录进一份带时间戳的新目录 —— 已有那份一个字节都不碰"),
    # ★ 这三条**不进菜单**：录制的入口就 11 那一项，而"现在在录什么"在它那一屏里
    #   已经摆着了（前台倒计时）；"重算清单"每次收口都会顺手做一次；
    #   "收编"只在出事之后用一次（给缺 `session.json` 的包补一份事后重建的记录）。
    #   留着它们是为了**另一个终端**（`run/z0.py rec_status`）与排障。
    Action("rec_status", "现在在录什么（只读）", None,
           "读状态文件 + 按名字扫一眼录制进程；**不发信号、不动文件**",
           read_only=True, menu=False),
    Action("rec_manifest", "重算 bag/MANIFEST.md", None,
           "从 `bag/*/session.json` 与 `bag/*/fcu/*.ulg` 重算清单（纯读盘 + 写一个 md）",
           menu=False),
    Action("rec_adopt", "★ 收编缺 session.json 的包（事后重建）", None,
           "只对**没有 `session.json`** 的包动手：从包自己的 `metadata.yaml` 与旁边那份 "
           "`<包目录名>.log` 反推，补一份**事后重建**的 `session.json`（取不到的字段写 null，"
           "**不猜**）并重算清单；已入账的包一个字节都不碰，包内容也不改",
           menu=False),

    # ★★ MPC 那一条（ticket 14，2026-10-16）—— **不进菜单**（`menu=False`，不占菜单号）：
    #   它的入口是第 9 项起链前那一句「谁在上位控制飞机」的第 2 档（二定：只剩两档）。
    #   ★ 为什么**能**进这张表（"没有任何让飞机动的入口"那条边界）：闸门关着时它**一帧都不发**
    #     —— `_tick` 在 `cycle.x0 is None` 时直接 return（不求解、不发 setpoint，见
    #     `mpc_node.py` 的 `_gate_closed`），而闸门要求 `armed + OFFBOARD`，这两件事
    #     **本脚本一件都不做**（`FORBIDDEN_IN_COMMANDS` 挡的正是那个入口）。
    #   ★ `--ros-args -p` 这个写法照抄 `docs/control/first-flight-config.md` 那条
    #     （`ros2 run nmpc mpc_node.py --ros-args -p tau_s:=0.0`）；⚠️ `preflight_check.py` 的
    #     `-p` 是它**自己** argparse 的选项，两个不是一回事，别互相搬。
    # ★ `env ACADOS_ENV` 那个前缀**必须在**：`mpc_node.py` 起来的头一秒就建求解器，
    #   两个变量缺一个就是 `OSError: lib*.so: cannot open shared object file`（见 `ACADOS_ENV`）。
    #   它是"起不来"，不是"飞错了"——所以放在**命令里**，`--dry-run` 也看得见。
    Action("mpc", "起 MPC 节点 —— nmpc 的 mpc_node.py",
           f"env {ACADOS_ENV} ros2 run nmpc mpc_node.py",
           "★ 四条可选参数（默认一条都不带）—— 两条定参考源（互斥），后两条是开关。"
           "第 9 项的 MPC 档打开 `reference_source_control` + `offboard_prestream`；"
           "`output_mode_external` 加在 **control** 那一条上，不在本动作里。"
           "★ 它在 **armed + OFFBOARD 之前一帧都不发**（闸门关着连心跳都没了 —— setpoint 就是心跳）"
           "⟹ 起它 ≠ 飞；解锁/切 OFFBOARD **由 control 做**（本动作里没有这类入口），见 `operation_manual.md`",
           optionals=(OptionalArg(
               "reference_source_traj",
               "要追轨迹吗？（y ⟹ 追加 --ros-args -p reference_source:=traj）"
               "—— 没有 `control` 的独立档用它（参考是 `/traj/reference`，相对轨迹原点）",
               "--ros-args -p reference_source:=traj"),
               OptionalArg(
               "reference_source_control",
               "参考由 control 给吗？（y ⟹ 追加 --ros-args -p reference_source:=control）"
               "—— 第 9 项的 MPC 档用它：`/control/target` 是**绝对目标**，"
               "开始键/解锁/轨迹原点/降落全部由 control 负责",
               "--ros-args -p reference_source:=control"),
               OptionalArg(
               "offboard_prestream",
               "要发前置流吗？（y ⟹ 追加 --ros-args -p offboard_prestream:=true）"
               "—— 只在未解锁或不在 OFFBOARD 时发纯悬停帧，只为让飞控肯放我们进 OFFBOARD",
               "--ros-args -p offboard_prestream:=true"),
               OptionalArg(
               "start_traj_reference",
               "要让它自己发 /traj/start 吗？（y ⟹ 追加 --ros-args -p start_traj_reference:=true）"
               "—— **只给 reference_source=traj 的旧独立档**：在「已解锁 + OFFBOARD + 状态新鲜"
               "但还没有参考」那一拍发一次 /traj/start（⛔ 不是遥控器开始键 —— 那归 control）；"
               "第 9 项两档都不打开它",
               "--ros-args -p start_traj_reference:=true"),),
           menu=False),

    # ★★ 下面两条**不进菜单**（`menu=False`，不占菜单号）。操作者 2026-09-23 拍定把菜单里
    #   原来那两项（8= bringup / 9= 操作台 fly.py）去掉，换成两条一键流程；但它们的**命令**
    #   仍留在表里，理由各一条：
    #   · `fly` —— 第 9 项起齐之后要**前台**把它跑起来选轨迹（`UP_HANDOFF`）；
    #   · `bringup` —— `DOWN_TARGETS` 要从它身上取包名（`z0.launch.py` 照 runbook 手打仍可行，
    #     留下的那个 launch 壳得扫得到）。两条都受同一套不变式扫查。
    Action("bringup", "起 bringup 一键（mocap+traj+control）", "ros2 launch bringup z0.launch.py",
           "★ 起之前飞机要摆好（内含 mocap）；mavros **默认不起**（独占串口、要人看一眼）",
           confirm="飞机已摆好、静止、固定？（bringup 里有 mocap）",
           optionals=(OptionalArg(
               "use_vrpn",
               "已经单独起过 vrpn_listener 了吗？（y ⟹ 追加 use_vrpn:=false）",
               "use_vrpn:=false"),),
           menu=False),
    Action("fly", "操作台 fly.py", "ros2 run bringup fly.py",
           "选轨迹 1/2/3；它**不发** `/traj/start`、不解锁 —— 那两条都在开始键上",
           menu=False),
)

ACTION_BY_KEY: Dict[str, Action] = {a.key: a for a in ACTIONS}

#: **菜单里看得见的那些**（编号就是它们的顺序）。`menu=False` 的仍在 `ACTIONS` 里，
#: 但既不显示、也不占号 —— 数字键 → key 的映射只走这张表。
MENU_ACTIONS: Tuple[Action, ...] = tuple(a for a in ACTIONS if a.menu)

#: 只读动作（`env` / `check`）—— 判据是表里那个 `read_only`，不是"`cmd` 是不是空"：
#: `up` 的 `cmd` 也是空，但它会**起进程**（本进程里做编排）。
READ_ONLY_KEYS: Tuple[str, ...] = tuple(a.key for a in ACTIONS if a.read_only)


def optionals_suffix(action: Action, answers: Sequence[bool]) -> str:
    """把选中的可选参数拼成命令行尾巴（**纯函数**，可离线测）。

    ★ 同一个 `-p <名>:=` 只许出现一次：两条参考源是**互斥**的，而 ROS 对命令行里
      重复的参数只取最后一处、**不报错** ⟹ 两个都答 y 会变成一次静默的"后答的胜出"。
      入口处一声响（`ValueError`），比现场猜"到底用了哪个参考源"便宜。
    """
    chosen = [opt.suffix for opt, yes in zip(action.optionals, answers) if yes]
    names = [s.split("-p ", 1)[1].split(":=", 1)[0] for s in chosen if "-p " in s]
    dup = sorted({name for name in names if names.count(name) > 1})
    if dup:
        raise ValueError(f"同一参数被指定了不止一次：{', '.join(dup)}（互斥开关只能开一个）")
    return (" " + " ".join(chosen)) if chosen else ""

#: 出现这些字样就说明"这个脚本能碰到飞机" —— 安全不变式（测试断言动作表里一条都没有）。
#: 判据不是"这些命令坏"，而是"它们**不该藏在一个菜单里**"：`z0.launch.py` 与
#: `bench-runbook.md` 的「还差两步（刻意不自动化）」都把这条路留给人。
FORBIDDEN_IN_COMMANDS: Tuple[str, ...] = (
    "ros2 service call", "ros2 param set", "ros2 topic pub", "ros2 action send_goal",
)


def forbidden_commands(actions: Iterable[Action]) -> List[str]:
    """返回动作表里**违反安全不变式**的命令（正常应当返回空表）。

    ★ 扫的是**最终会送进 `bash -c` 的全部文本**：主命令 + 每条可选参数的 `suffix`
    （可选参数拼进的是同一条命令行，不扫它就等于留了个后门）。
    """
    bad: List[str] = []
    for action in actions:
        pieces = [action.cmd or ""] + [opt.suffix for opt in action.optionals]
        for piece in pieces:
            lowered = piece.lower()
            for token in FORBIDDEN_IN_COMMANDS:
                if token in lowered:
                    bad.append(f"{action.key}: {token} ← {piece}")
    return bad


# ─────────────────────────── 只读：环境自检 ───────────────────────────


def serial_state() -> dict:
    """串口三态那一份 dict（`serial_line` 的入参形状）—— **只有这一处写**。

    ★ `serial_line` 的 docstring 就是"一处写、两处用（判据只有一份，别再抄一遍）"：
    到 2026-09-23 这份 dict 已经有三个用户（`collect_env` / `Probe.snapshot` / `cmd_down`），
    复核时三份字面量一模一样 ⟹ 收成这个函数，别再抄第四遍。
    """
    return {"path": SERIAL_PORT, "exists": Path(SERIAL_PORT).exists(),
            "holders": _serial_holders(SERIAL_PORT)}


def _serial_holders(path: str) -> List[str]:
    if shutil.which("lsof") is None:
        return []
    try:
        out = subprocess.run(["lsof", "-Fpcf", path], capture_output=True, text=True, timeout=10)
    except (OSError, subprocess.SubprocessError):
        return []
    return parse_lsof_fpcf(out.stdout)


def collect_env() -> dict:
    """环境自检的原始读数（纯数据 ⟹ 表由 `render_env` 拼）。"""
    from ament_index_python.packages import get_package_prefix  # source 过才有

    packages: Dict[str, str] = {}
    for name in EXPECTED_PACKAGES:
        try:
            packages[name] = get_package_prefix(name)
        except Exception:
            packages[name] = ""
    git_head = subprocess.run(["git", "-C", str(REPO), "rev-parse", "--short", "HEAD"],
                              capture_output=True, text=True).stdout.strip()
    git_dirty = subprocess.run(["git", "-C", str(REPO), "status", "--porcelain"],
                               capture_output=True, text=True).stdout.strip().splitlines()
    return {
        "setups": [str(p) for p in (ROS_SETUP, MAVROS_WS / "install/setup.bash",
                                    REPO / "install/setup.bash")],
        "existing_setups": [str(p) for p in existing_setups()],
        "ros2_cli": shutil.which("ros2") or "",
        "packages": packages,
        "serial": serial_state(),
        "domain": os.environ.get("ROS_DOMAIN_ID"),
        "localhost_only": os.environ.get("ROS_LOCALHOST_ONLY"),
        "ament_prefix_path": os.environ.get("AMENT_PREFIX_PATH", ""),
        "git": {"head": git_head or "(git 读不到)", "dirty": len(git_dirty)},
    }


def render_env(env: dict) -> str:
    """环境自检的表（纯函数）。"""
    lines: List[str] = []
    existing = set(env.get("existing_setups") or [])
    lines.append("工作空间：")
    for path in env.get("setups") or []:
        mark = "✓" if path in existing else "✗"
        lines.append(f"  {mark} {path}")
    if len(existing) < 3:
        lines.append("  ⟹ 缺的那条 source **不会**被本脚本补上（它不假装）—— 先按缺的那条去装/去 build")
    prefix_path = env.get("ament_prefix_path", "")
    for ws in ("/opt/ros", str(MAVROS_WS), str(REPO)):
        if ws and ws not in prefix_path:
            lines.append(f"  ⚠️ `AMENT_PREFIX_PATH` 里没有 {ws} ⟹ 那个工作空间的包看不见")

    lines.append("")
    lines.append("包（`install/` 里在不在）：")
    for name, prefix in (env.get("packages") or {}).items():
        mark = "✓" if prefix else "✗"
        if prefix:
            detail = prefix
        elif (REPO / "install" / name).exists():
            # ★ 这一条是第一手教训：**装好了**但当前 shell 没 source 它，报成
            # "先 colcon build" 会把人送去重编一份本来就好的东西（本机 2026-09-23 就撞到过）。
            detail = "**装好了但没 source**（本脚本的子进程会自己 source；`env`/`check` 看的是当前环境）"
        else:
            detail = "查不到 ⟹ 先 `colcon build --packages-up-to control`"
        lines.append(f"  {mark} " + pad(name, 14) + detail)

    lines.append("")
    lines.append(serial_line(env.get("serial") or {}, occupied="已经是 mavros 就该是它"))
    lines.append(f"· `ros2` CLI：{env.get('ros2_cli') or 'PATH 里没有 ⟹ 三条 setup 没 source 上'}")
    lines.append("· " + domain_note(env.get("domain"), env.get("localhost_only")))

    git = env.get("git") or {}
    dirty = git.get("dirty", 0)
    lines.append(f"· 代码：{git.get('head')}" + (f" + **{dirty} 处未提交改动**" if dirty else "（工作区干净）"))
    return "\n".join(lines)


def cmd_env() -> int:
    ensure_sourced()
    print(render_env(collect_env()))
    return 0


# ─────────────────────────── 只读：起链核对 ───────────────────────────


def _msg_class(type_string: str):
    """`"mavros_msgs/msg/State"` → 那个消息类（没有就返回 None，**不抛**）。"""
    parts = type_string.split("/")
    if len(parts) != 3:
        return None
    try:
        module = importlib.import_module(f"{parts[0]}.{parts[1]}")
    except ImportError:
        return None
    return getattr(module, parts[2], None)


class Probe:
    """一个**只订阅**的 rclpy 节点 —— 同一张图上可以反复采（一键要一步一步等读数）。

    ★ 用**本进程的 rclpy 节点**，不经过 `ros2` CLI：CLI 的 daemon 会让图**静默为空**
    （第一手：同域内 daemon 与夹具的 `ROS_LOCALHOST_ONLY` 不一致时，
    `ros2 topic list` 只回 `/rosout` 与 `/parameter_events`，见 `run/README.md` 排障表）。
    ★ QoS 取 **best_effort + volatile**：它对任何发布方都兼容（durability/reliability
    只要求 offered ≥ requested）⟹ 「我订上了没有」不会变成第二个静默坑。
    **本节点只订阅，从不发布**（安全不变式由测试盯着）。
    """

    def __init__(self, discover_s: float = 1.5):
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

        self._rclpy = rclpy
        self._started_here = not rclpy.ok()
        if self._started_here:
            rclpy.init(args=[])
        self._node = Node(f"z0_probe_{os.getpid()}")
        self._qos = QoSProfile(depth=200,
                               reliability=ReliabilityPolicy.BEST_EFFORT,
                               durability=DurabilityPolicy.VOLATILE,
                               history=HistoryPolicy.KEEP_LAST)
        self._discover_s = discover_s
        self._subs: Dict[str, object] = {}
        self._counts: Dict[str, int] = {}      # 累计帧数（对外报的是**窗口内**的增量）
        self._last: Dict[str, str] = {}        # ★ 累计：最后一个窗口不一定有帧，等待判据要靠它

    def _spin(self, seconds: float) -> None:
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            self._rclpy.spin_once(self._node, timeout_sec=0.1)

    def _discover(self) -> Dict[str, str]:
        """按图里**真实存在**的话题订。

        ★ 每次都重看一遍图：一键里话题是**逐步出现**的（mavros 起了才有 `/mavros/state`），
        一次性的发现会让后面的步子对着不存在的话题空等。
        """
        topics = {name: types[0] for name, types in self._node.get_topic_names_and_types() if types}
        wants = [t for t, _ in PROBES] + [
            t for t in topics if t.startswith("/vrpn/") and t.endswith("/pose")]
        for topic in wants:
            if topic in self._subs or topic not in topics:
                continue
            cls = _msg_class(topics[topic])
            if cls is None:
                continue
            self._counts.setdefault(topic, 0)
            self._subs[topic] = self._node.create_subscription(
                cls, topic, self._on_msg(topic), self._qos)
        return topics

    def _on_msg(self, topic: str) -> Callable[[object], None]:
        def callback(msg: object) -> None:
            self._counts[topic] += 1
            self._last[topic] = describe_message(msg)
        return callback

    def snapshot(self, window_s: float = 4.0) -> dict:
        """采**一段窗口**，返回**纯数据**（表由 `render_snapshot` 拼）。

        `counts` 是**这段窗口内**的帧数（`rate_text` 按它算 Hz）；
        `last` 是**到目前为止**最后一帧的原文（跨窗口累积 —— 等待判据靠它，
        否则 1 Hz 的 `/mavros/state` 会有一半窗口读不到）。
        """
        if not self._subs:                      # 先给发现一点时间（只在第一次）
            self._spin(self._discover_s)
        self._discover()
        base = dict(self._counts)
        self._spin(window_s)
        topics = self._discover()
        return {
            "window_s": window_s,
            "domain": os.environ.get("ROS_DOMAIN_ID"),
            "localhost_only": os.environ.get("ROS_LOCALHOST_ONLY"),
            "topics": topics,
            "nodes": self._node.get_node_names(),
            "counts": {t: n - base.get(t, 0) for t, n in self._counts.items()},
            "last": dict(self._last),
            "serial": serial_state(),
        }

    def close(self) -> None:
        self._node.destroy_node()
        if self._started_here:
            self._rclpy.try_shutdown()


def collect_snapshot(window_s: float = 4.0) -> dict:
    """采一张（用完就把节点收掉）—— `check` 与现场用例走这条。"""
    probe = Probe()
    try:
        return probe.snapshot(window_s)
    finally:
        probe.close()


def cmd_check() -> int:
    ensure_sourced()
    print(render_snapshot(collect_snapshot()))
    return 0


# ─────────────────── 一键（第 8 / 9 项，ticket 22 §七 / §九） ───────────────────
#
# ★ 它**不代做**"还差的那两步"（选轨迹 `fly.py` / 起飞 = 遥控器开始键）—— 见 ticket 22 §二：
#   `z0.launch.py` 的「本包**不给**"程序起飞"的入口，因为它会绕开"开始键之前不发控制指令"这条防线」。


#: 起链的顺序 —— 与 `bench-runbook.md` 的「起链（顺序不能反）」一致。
#: ★ 每一项都是 `ACTIONS` 里的 key（**不是另抄一份命令**）⟹ ticket 22 那条不变式天然盖住它们。
UP_CHAIN: Tuple[str, ...] = ("mavros", "vrpn", "mocap", "traj", "control")

#: 「动捕 + mavros，手动遥控飞」那条 —— 前三条同上，**到 mocap 为止**：
#: 不起 traj/control ⟹ 没有任何程序在上位控制飞机（遥控器怎么飞就怎么飞）。
MANUAL_CHAIN: Tuple[str, ...] = ("mavros", "vrpn", "mocap")

#: 起齐之后**把终端让给**谁：`up` 要选轨迹，而选轨迹是在操作台里敲 1/2/3（前台）。
#: ★ 与 `UP_CHAIN` 分开是因为**形态不同**：那几条后台起 + 等判据，这一条要占着 stdin/stdout。
UP_HANDOFF: Tuple[str, ...] = ("fly",)

#: 一键**默认带上**的可选参数（按 `OptionalArg.name` 指认；值仍写在 `ACTIONS` 里）。
UP_OPTIONALS: Dict[str, Tuple[str, ...]] = {"mavros": ("gcs_url",)}


# ───────────── 链尾那一格：这一趟「谁在上位控制飞机」（ticket 14，2026-10-16） ─────────────
#
# ★ **第 9 项**起链前**问一句**（两档，输数字，空回车 = offboard 控制）；**第 8 项不问**
#   （它只有一档，2026-10-16 复原）。两条链的**公共前缀是 `MANUAL_CHAIN`**
#   （mavros → vrpn → mocap），尾巴由选中的那一档给。
# ★ 每一档的 `tail` **只写 key**（命令仍取自 `ACTIONS`）⟹ ticket 22 那条不变式天然盖住它们；
#   `closing` 是**提示**（起完那几步怎么做），不是命令 —— 它不含任何能送进 `bash -c` 的东西。


@dataclass(frozen=True)
class ControllerChoice:
    """起链前那一句的一个答案：**尾巴起哪几条** + 起完要不要让终端 + 起完说哪几句。

    `tail` 只写 `MANUAL_CHAIN` **之外**的那几条 —— 公共前缀那条不变式（两条链的前三条
    逐字相同）照旧成立。`extra_optionals` 是这一档要**额外带上**的可选参数名
    （按 `OptionalArg.name` 指认，值仍写在 `ACTIONS` 里）。
    """

    key: str                                   # 打印与测试指认用，不参与命令拼装
    title: str
    tail: Tuple[str, ...]
    closing: Tuple[str, ...]
    handoff: Tuple[str, ...] = ()
    extra_optionals: Tuple[str, ...] = ()


#: 菜单 8（`up_manual`）**只有一档** —— 它**不问**（2026-10-16 复原）：起 `mavros → vrpn → mocap`，
#: 没有任何程序在上位控制飞机。★ 留成"一格档"而不是特判，是为了让 `ask_chain_choice` /
#: `render_chain` 只有一条代码路（"只有一档就不问"），菜单 8 与菜单 9 的差别只剩**表里有几行**。
MANUAL_CHOICES: Tuple[ControllerChoice, ...] = (
    ControllerChoice("none", "不起控制器 —— 只到动捕 + mavros，起完直接用遥控器手动飞", (), (
        "★ 现在**直接用遥控器起飞**就行 —— 本流程**没有** traj/control：",
        "  没有任何程序在上位控制飞机，飞成什么样全看你手上的杆。",
        "  动捕经 mavros 变成 EKF2 的位置估计（`/mavros/vision_pose/pose`）⟹ 位置信息在，"
        "但**没人替你把杆**。",
    )),
)


#: MPC 档那几句（票 15 之后：**`control` 在场，开始键照常有效**）。
#: ⛔ **不含任何命令**（"在哪手打"只指文档）：`z0.py` 里不许出现 `ros2 service call` 那类字样
#: （`FORBIDDEN_IN_COMMANDS` + 那条 AST 用例扫的正是全文件的字符串字面量）。
#: ★★ 票 15 把这一档从"`control` 不在场、解锁/切模式/开始键全要手打"改成**与 offboard 档同流程**：
#:   `control` 是唯一任务总管（开始键 / 解锁 / OFFBOARD / 起飞 / 判稳 / 轨迹原点 / `/traj/start` /
#:   降落），`mpc_node` 只是它的**输出后端** —— 订 `/control/target` 求解，发姿态 setpoint。
#: ⟹ 操作员看到的两条链**只剩输出那一层不同**，手上的动作完全一样。
_MPC_CLOSING: Tuple[str, ...] = (
    "★★ 选到 MPC 档：**开始键与 offboard 档完全一样** —— 两种控制只是输出后端不同：",
    "   · `control` 仍是唯一任务总管：开始键 → 解锁 + 切 OFFBOARD + 起飞 + 判稳 + 轨迹原点"
    " + 发 `/traj/start` + 轨迹超时/降落，全部由它负责；",
    "   · `mpc_node` 只做一件事：订 `/control/target`（**绝对目标**，位置 + 速度 + yaw）并求解，"
    "把 `/mavros/setpoint_raw/attitude` 发给飞控；它是本档唯一写 PX4 setpoint 的节点。",
    "★ 起飞与执行轨迹**都在你手上**（本脚本刻意不代做 —— ticket 22 §二）：",
    "   ① 选轨迹：刚把终端让给操作台那一步（没选成的话再跑 `ros2 run bringup fly.py`）；",
    "      ★ **必须起飞前选好** —— `control` 进 `TRACK` 那一下要拿去发 `/traj/start`。",
    "   ② 起飞/开始轨迹：**遥控器开始键**（开始通道，现场是 CH9）",
    "      第 1 次上升沿 = 解锁 + 起飞、第 2 次 = 进 `TRACK`（开始执行轨迹）。",
    "      ⚠️ **每次压住 ≥ 0.3 s 再松**（200 ms 去抖对上 10 Hz 采样，压得太准会按不动）；",
    "      ⚠️ 上电时它若已停在**高位**，那**不算边沿**（必须先见低位）；",
    "      ⚠️ 起飞前置里有一条「遥控在线」⟹ 台上没接遥控接收机时**永远解不了锁**（刻意如此）。",
    "   ③ 操作台 `fly.py` 的状态栏照常可用（`/control/state` 与 `/control/traj_progress`"
    "仍由 `control` 发）。",
    "★ 安全边界（两条链一样）：`control` 一旦不再发目标（0.5 s），`mpc_node` **关闸停发** ——"
    "由 PX4 的 OFFBOARD 失流保护（1 s）接管；它**不会**拿着最后一帧目标一直飞。",
)


#: 菜单 9（`up`）**两档** —— ★★ **两档的尾巴都含 `traj`**：操作者 2026-10-16 的原话是
#: 「轨迹必须起没得选」⟹ "必起 traj"是**结构上的事实**（写在 `tail` 里），不是提示语；
#: 一版那条"MPC + `hold`、不起 traj"的路（第 3 档）按同一次裁定**删掉**。
#: ★★ 票 15 起**两档的尾巴也都含 `control`**（第 2 档是 `traj → mpc → control`）：
#:   任务流程只写一遍，`mpc` 退化成 `control` 的一个输出后端 —— 将来的新控制方式照同样形状接。
UP_CHOICES: Tuple[ControllerChoice, ...] = (
    ControllerChoice("offboard", "offboard 控制（control，(A) 级）—— 起 traj；起完选轨迹 + 按开始键",
                     ("traj", "control"), (
        "★ 起飞与执行轨迹**都在你手上**（本脚本刻意不代做 —— ticket 22 §二）：",
        "   ① 选轨迹：刚把终端让给操作台那一步（没选成的话再跑 `ros2 run bringup fly.py`）；",
        "      ★ **必须起飞前选好** —— `control` 进 `TRACK` 那一下要拿去发 `/traj/start`。",
        "   ② 起飞/开始轨迹：**遥控器开始键**（开始通道，现场是 CH9）",
        "      第 1 次上升沿 = 解锁 + 起飞、第 2 次 = 进 `TRACK`（开始执行轨迹）。",
        "      ⚠️ **每次压住 ≥ 0.3 s 再松**（200 ms 去抖对上 10 Hz 采样，压得太准会按不动）；",
        "      ⚠️ 上电时它若已停在**高位**，那**不算边沿**（必须先见低位）；",
        "      ⚠️ 起飞前置里有一条「遥控在线」⟹ 台上没接遥控接收机时**永远解不了锁**（刻意如此）。",
    ), handoff=UP_HANDOFF),
    ControllerChoice("mpc",
                     "MPC—— mpc_node.py 作为 control 的输出后端；"
                     "**起 traj + mpc + control**，起完选轨迹"
                     "（阶梯 4 飞轨迹用这档）",
                     ("traj", "mpc", "control"), _MPC_CLOSING,
                     handoff=UP_HANDOFF,
                     # ★★ 票 15 的三条（全部**默认关**，只有这一档打开）：
                     #   · `reference_source_control` —— mpc 的参考是 `/control/target`（绝对目标）；
                     #   · `offboard_prestream` —— 进 OFFBOARD 前由 mpc 先发悬停帧（control 这档不写
                     #     PX4 setpoint，这条流必须有人给）；
                     #   · `output_mode_external` —— control **只发** `/control/target`，
                     #     不写 `/mavros/setpoint_raw/local`（两路 setpoint 会互相覆盖）。
                     # ⛔ `start_traj_reference` **不再带**：`/traj/start` 由 `control` 的 TRACK 分支发
                     #   （与 offboard 档逐字相同）。两边都发会让参考时钟被归零两次。
                     extra_optionals=("reference_source_control", "offboard_prestream",
                                      "output_mode_external")),
)


@dataclass(frozen=True)
class Chain:
    """一条一键流程（= 菜单第 8 / 9 项）：**公共前缀 + 起链前选出来的那一档尾巴**。

    ★ `steps` / `handoff` / `closing` 都是**默认那一档**的（`--dry-run` 与既有用例看的就是它）；
      真正起什么由 `cmd_chain` 里那一句问出来的编号决定（`choice(n)`）。
    """

    key: str                          # 与 `ACTIONS` 的 key 对齐（第 8 / 第 9 项那两条）
    label: str                        # 打印用
    choices: Tuple[ControllerChoice, ...]    # ★ 每条链**各自一张表**（第 8 项 1 行、第 9 项 2 行）
    default_choice: int = 1           # 空回车算哪一档（1 起；编号 = 本链 `choices` 里的位次）

    def choice(self, number: int) -> ControllerChoice:
        """第 `number` 档（1 起）。越界 ⟹ 报错，**不许**悄悄退回默认。"""
        if not 1 <= number <= len(self.choices):
            raise ValueError(f"{self.key}: 没有第 {number} 档（只有 1–{len(self.choices)}）")
        return self.choices[number - 1]

    @property
    def steps(self) -> Tuple[str, ...]:
        """**默认那一档**的完整起链顺序（含公共前缀）。"""
        return MANUAL_CHAIN + self.choice(self.default_choice).tail

    @property
    def handoff(self) -> Tuple[str, ...]:
        return self.choice(self.default_choice).handoff

    @property
    def closing(self) -> Tuple[str, ...]:
        return self.choice(self.default_choice).closing


CHAINS: Tuple[Chain, ...] = (
    # ★ 第 8 项**复原**成一版之前那样：只有一档、不问，起的东西与那时**逐字一致**。
    Chain("up_manual", "一键启动动捕 + mavros（手动遥控飞）", MANUAL_CHOICES, default_choice=1),
    # ★ 第 9 项两档，默认 = offboard 控制（= 它今天的样子）⟹ 空回车的行为与这一版之前**逐字一致**。
    Chain("up", "一键启动全部（起前选控制器；默认 offboard 控制）", UP_CHOICES, default_choice=1),
)

CHAIN_BY_KEY: Dict[str, Chain] = {c.key: c for c in CHAINS}

#: 每一步"起来了没有"等的那个话题。`None` ⟹ **不等**。
#: ★ `traj` 是 `None` 不是偷懒：它没被 `select` 就一帧都不发（等它等于等到天荒地老）。
STEP_TOPIC: Dict[str, Optional[str]] = {
    "mavros": "/mavros/state",
    "vrpn": None,                     # 话题名带刚体名 ⟹ 动态发现，判据见 STEP_READY
    "mocap": "/mocap/pose_enu",
    "traj": None,
    "control": "/control/state",
}

#: "起来了没有"的判据 —— **纯函数**（喂一张 snapshot 就出答案，可离线测）。
STEP_READY: Dict[str, Callable[[dict], bool]] = {
    # ★ 有帧 ≠ 连上飞控：`/mavros/state` 的 1 Hz 心跳在飞控链路通之前就在发 ⟹ 判据是 connected
    "mavros": lambda snap: "connected=True" in (snap.get("last") or {}).get("/mavros/state", ""),
    # 刚体名不做归一化（`Z0`/`z0` 都出现过）⟹ 任何一个 `/vrpn/<刚体>/pose` 在发就算
    "vrpn": lambda snap: any(t.startswith("/vrpn/") and n > 0
                             for t, n in (snap.get("counts") or {}).items()),
    "mocap": lambda snap: (snap.get("counts") or {}).get("/mocap/pose_enu", 0) > 0,
    "control": lambda snap: (snap.get("counts") or {}).get("/control/state", 0) > 0,
}

#: 每一步最多等多久（mavros 最慢：开串口 + 与飞控握手）。
STEP_TIMEOUT_S: Dict[str, float] = {"mavros": 60.0, "vrpn": 30.0, "mocap": 30.0, "control": 30.0,
                                    "mpc": 120.0}

#: 「起来了没有」的**日志判据**：子进程**自己的日志**（`<log_dir>/NN-<key>.log`）里要出现这个串。
#:
#: ★ `mpc` 落在这里不是偷懒，是**话题面上根本没有这个事实**：闸门关着（未 `armed` / 未
#:   `OFFBOARD`）时它**不求解也不发 setpoint** —— `_tick` 在 `cycle.x0 is None` 时直接
#:   `_gate_closed` 后 return，而 `_viz_tick` 又要求 `last_prediction()` 非 `None`
#:   （`src/nmpc/nmpc/mpc_node.py`）⟹ `/mpc/predicted_path` 在解锁之前**一声不响**。
#:   ⟹ 判据取它启动时那一行（`mpc_node.py:724-725` 的 `get_logger().info(...)`），**正向**的：
#:   节点因参数校验 / acados 起不来时这一条当场红，并把日志尾巴打出来（不是"起了就当成功"）。
#: ★ 超时给 120 s 的理由：冷 `$MPC_CODEGEN_DIR`（默认 `~/.cache/mpc-ocp-codegen`）时要
#:   **代码生成 + 编译 C**，那是几十秒到分钟级的；热的时候只有几秒。
#:   ⚠️ 这一行的措辞若改了，这里会**红**（等超时 + 日志尾巴），不会静默 —— 用例另外钉了一条
#:   （那个 marker 必须仍出现在 `mpc_node.py` 的源码里）。
STEP_LOG_READY: Dict[str, str] = {"mpc": "MPC（模型预测控制）节点启动："}


def wait_kind(key: str) -> Optional[str]:
    """这一步按**哪种判据**等：`"ready"`（话题/状态）/ `"log"`（它自己的日志）/ `None`（不等）。

    ★ 这个分类**只此一处** —— `wait_hint`（说什么）、`start_and_wait`（真去等）、
      `_render_steps`（摊开时要不要印那一行）都问它。★ 再加一种判据只改这里；
      原先这三处各写一遍 `if key in STEP_READY: … elif key in STEP_LOG_READY: …`，
      漏改一处的表现是**静默的**（屏上说得对、实际没等，或反过来）。
    """
    if key in STEP_READY:
        return "ready"
    if key in STEP_LOG_READY:
        return "log"
    return None


def wait_hint(key: str) -> str:
    """这一步在等什么（人话）—— 打印出来，别让人对着一动不动的屏猜。"""
    if key == "mavros":
        return "等 `/mavros/state` 的 **connected=True**（★ 有帧 ≠ 连上飞控：心跳在链路通之前就在发）"
    if wait_kind(key) == "log":
        return (f"等它**自己的日志**里出现 `{STEP_LOG_READY[key]}`"
                "（★ 闸门关着时它一帧都不发 ⟹ 话题面上没有「我起来了」这个事实）")
    if key == "vrpn":
        return "等 `/vrpn/<刚体>/pose` 有帧（刚体名 `Z0`/`z0` 都认）"
    topic = STEP_TOPIC.get(key)
    return f"等 `{topic}` 有帧" if topic else "不等（它本来就不发东西）"


@dataclass
class Child:
    """本脚本起的**一个后台子进程**，以及它的日志。"""

    key: str
    cmd: str
    proc: subprocess.Popen
    log: Path
    fh: IO[str]

    def alive(self) -> bool:
        return self.proc.poll() is None

    def tail(self, lines: int = 12) -> str:
        """日志的最后几行 —— 等超时或半路退了，人要看的正是它。"""
        try:
            text = self.log.read_text(encoding="utf-8", errors="replace").splitlines()
        except OSError:
            return ""
        return "\n".join("      " + line for line in text[-lines:])

    def close_log(self) -> None:
        if not self.fh.closed:
            self.fh.close()

    def stop(self, grace_s: float = 5.0) -> None:
        """停掉它**以及 `ros2 launch` 拉起来的整棵子树**。

        ★ 它自成一个进程组（`start_new_session=True`）—— 好的一面是停整组不会误伤别人，
        代价是操作者在前台按的 Ctrl-C **不会**传给它，必须由本脚本转发（`stop_children`）。
        ★ 先 SIGINT（与操作者自己按 Ctrl-C 同一个信号 ⟹ 各节点走自己的收摊逻辑），
        超时才 SIGKILL —— **不能留下 mavros 占着串口**。
        """
        try:
            if self.proc.poll() is None:
                try:
                    os.killpg(os.getpgid(self.proc.pid), signal.SIGINT)
                    self.proc.wait(timeout=grace_s)
                except (ProcessLookupError, subprocess.TimeoutExpired):
                    try:
                        os.killpg(os.getpgid(self.proc.pid), signal.SIGKILL)
                    except ProcessLookupError:
                        pass
                    try:
                        self.proc.wait(timeout=grace_s)
                    except subprocess.TimeoutExpired:
                        pass
        finally:
            self.close_log()


def stop_children(children: Sequence[Child]) -> None:
    """收尾：把本脚本起的都停掉。★ **绝不能留下 mavros 占着串口。**"""
    for child in children:
        if child.alive():
            print(f"  停 {child.key} … ", end="", flush=True)
            child.stop()
            print("好")
    for child in children:              # 已经自己退了的，句柄也要关
        child.close_log()


def start_child(key: str, log_path: Path, extra_names: Sequence[str] = ()) -> Child:
    """起一条动作的后台子进程，输出**进日志文件**（★ 不丢日志：判读靠它们）。

    `extra_names` = 这一趟**额外**要带上的可选参数名（按 `OptionalArg.name` 指认）——
    一键里由**选中的那一档**给（`ControllerChoice.extra_optionals`），
    默认那几条仍由 `UP_OPTIONALS` 给。★ 拼装仍然只走 `optionals_suffix`（唯一一处），
    所以"可选参数也受安全不变式扫查"那条照旧成立。
    """
    action = ACTION_BY_KEY[key]
    chosen = tuple(UP_OPTIONALS.get(key, ())) + tuple(extra_names)
    cmd = (action.cmd or "") + optionals_suffix(
        action, [opt.name in chosen for opt in action.optionals])
    log_path.parent.mkdir(parents=True, exist_ok=True)
    fh = log_path.open("w", encoding="utf-8")
    proc = subprocess.Popen(["bash", "-c", sourced_command(cmd)],
                            stdout=fh, stderr=subprocess.STDOUT, start_new_session=True)
    return Child(key=key, cmd=cmd, proc=proc, log=log_path, fh=fh)


def wait_for_step(probe: Probe, key: str, *, window_s: float = 2.0,
                  alive: Optional[Callable[[], bool]] = None,
                  echo: Optional[Callable[[dict], None]] = None,
                  timeout_s: Optional[float] = None) -> Tuple[bool, dict]:
    """反复采一小口，直到这一步**真的起来**（`STEP_READY`）或超时。

    返回 `(成不成, 最后一次读数)` —— **超时也把读数交回去**：人要看的正是"它现在到哪一步了"。
    """
    limit = STEP_TIMEOUT_S.get(key, 30.0) if timeout_s is None else timeout_s
    deadline = time.monotonic() + limit
    ready = STEP_READY[key]
    snap = probe.snapshot(window_s)
    while True:
        if ready(snap):
            return True, snap
        if time.monotonic() >= deadline:
            return False, snap
        if alive is not None and not alive():
            return False, snap          # 子进程半路退了 ⟹ 不要再空等
        if echo is not None:
            echo(snap)
        snap = probe.snapshot(window_s)


def wait_for_log(child: Child, marker: str, *, timeout_s: float) -> Tuple[bool, str]:
    """等子进程**自己的日志**里出现 `marker`（那几条话题面上没有判据的步骤，见 `STEP_LOG_READY`）。

    返回 `(成不成, 不成的原因)`。判据与 `wait_for_step` 同一条纪律：
    ① 到了就成；② **子进程自己退了**立刻判否（不空等到超时）；③ 超时判否。

    ★ 读的是子进程 stdout/stderr 那个文件（`start_child` 把两者并进同一个文件），**边跑边读**。
      ⚠️ 这一手成立**不是**因为"反正会 flush"：是**同一条判据已经在别处跑通了** ——
      `src/nmpc/test/test_mpc_node.py:561` 就是 `sc.wait_for(lambda: "节点启动" in sc.dut_log() …)`，
      读的同样是重定向出来的日志文件。那一行是 `self.get_logger().info(...)`（节点启动日志），
      按记录 flush ⟹ 不会卡在缓冲里等 4 KiB。
      ⛔ 换成 `print()` 打的话，`stdout` 重定向到文件时是**块缓冲**（本仓栽过：探针不带 `-u`，
      中途读文件是空的）⟹ 这条判据就会变成"每次都等满 120 s"。改 `mpc_node` 那一行时先看这点。
    """
    deadline = time.monotonic() + timeout_s
    while True:
        try:
            text = child.log.read_text(encoding="utf-8", errors="replace")
        except OSError:                     # 文件还没建出来 / 被删了 —— 当作"还没到"，不炸
            text = ""
        if marker in text:
            return True, ""
        if not child.alive():
            return False, "子进程自己退了"
        if time.monotonic() >= deadline:
            return False, f"等超时（{timeout_s:.0f} s）"
        time.sleep(0.2)


def echo_waiter(key: str, every_s: float = 5.0) -> Callable[[dict], None]:
    """等的时候每隔几秒报一句"现在的读数" —— 否则长时间不动看着像卡死了。"""
    last = [0.0]

    def echo(snap: dict) -> None:
        now = time.monotonic()
        if now - last[0] < every_s:
            return
        last[0] = now
        if key == "vrpn":
            live = [t for t, n in (snap.get("counts") or {}).items()
                    if t.startswith("/vrpn/") and n > 0]
            print(f"        · 还没到：{'、'.join(live) if live else '一个 /vrpn/<刚体>/pose 都还没发'}")
        else:
            topic = STEP_TOPIC.get(key)
            if topic:
                print(f"        · 还没到：`{topic}` " + (snap.get("last", {}).get(topic) or "无帧"))

    return echo


def serial_gate(action: Action) -> bool:
    """起之前先看串口有没有被别人占着；占着就问一句。返回 True = 可以继续。

    ★ 理由是第一手的：第二个 mavros 打不开串口，而**那个报错长得像 launch 坏了**。
    """
    if not action.warn_serial:
        return True
    holders = _serial_holders(SERIAL_PORT)
    if not holders:
        return True
    print(f"⚠️ {SERIAL_PORT} 已被占用：{'、'.join(holders)}")
    print("   第二个 mavros 打不开串口，而那个报错**长得像 launch 坏了** —— 先确认真的是同一个。")
    return ask_confirm("仍然要起？")


def start_and_wait(probe: Probe, key: str, log_dir: Path, started: List[Child],
                   *, index: int, total: int,
                   extra_names: Sequence[str] = ()) -> Tuple[bool, dict]:
    """起一步 + **等它真的起来**（按 `wait_kind` 分派：话题判据或它自己的日志判据）。
    失败就把日志最后几行打出来。"""
    action = ACTION_BY_KEY[key]
    if not serial_gate(action):
        return False, {}
    if action.confirm and not ask_confirm(action.confirm):
        print("已取消。")
        return False, {}
    log_path = log_dir / f"{index:02d}-{key}.log"
    child = start_child(key, log_path, extra_names)
    started.append(child)
    chosen = tuple(UP_OPTIONALS.get(key, ())) + tuple(extra_names)
    extra = [opt.suffix for opt in action.optionals if opt.name in chosen]
    print(f"[{index}/{total}] {action.title}")
    print(f"      命令：{child.cmd}" + (f"（带上 {'、'.join(extra)}）" if extra else ""))
    print(f"      日志：{log_path}")
    kind = wait_kind(key)
    if kind is None:                               # 没有"起来了"的判据 ⟹ 起了就往下走
        print(f"      不等：{wait_hint(key)}")
        return True, {}
    print(f"      {wait_hint(key)}（最多 {STEP_TIMEOUT_S[key]:.0f} s）")
    t0 = time.monotonic()
    if kind == "ready":
        ok, payload = wait_for_step(probe, key, alive=child.alive, echo=echo_waiter(key))
        reason = "子进程自己退了" if not child.alive() else f"等超时（{STEP_TIMEOUT_S[key]:.0f} s）"
    else:
        ok, reason = wait_for_log(child, STEP_LOG_READY[key], timeout_s=STEP_TIMEOUT_S[key])
        payload = {}
    if ok:
        print(f"      ✓ 起来了（{time.monotonic() - t0:.1f} s）")
        return True, payload
    print(f"      ✗ {reason} —— 它的日志最后几行：")
    print(child.tail())
    if not child.alive():
        print(f"      （看完整日志：`cat {log_path}`）")
    return False, payload              # ★ 失败也把最后一帧带回去（与这一版之前一样；调用方现在不看它）


def watch_children(children: Sequence[Child], interval_s: float = 1.0) -> int:
    """前台守着：谁**自己退了**就报出来（连同它的日志尾巴）。

    ★ 这是"起完别撒手"的那一步：中间某个节点悄悄挂了没人知道，是现场最贵的错法之一
    （`control` 一挂，飞机就停在最后一次 setpoint 上）。
    """
    while True:
        time.sleep(interval_s)
        dead = next((c for c in children if not c.alive()), None)
        if dead is not None:
            print(f"⚠️ `{dead.key}` 自己退了（不是 Ctrl-C）—— 它的日志最后几行：")
            print(dead.tail())
            return 1


def handover(key: str) -> int:
    """**把终端让给**一条动作（前台跑）—— 操作者要在里面敲键（操作台选轨迹）。

    ★ 与后台那几条分开：它们的输出进日志、人不必看着；这一条**必须占着 stdin/stdout**
    （选轨迹就是敲 1/2/3）。可测性仍然只有一个出口（`run_foreground`）。
    """
    action = ACTION_BY_KEY[key]
    cmd = (action.cmd or "") + optionals_suffix(action, [False] * len(action.optionals))
    print(f"★ 下面把终端让给「{action.title}」—— 敲 1/2/3 选轨迹（**起飞前选好**），"
          "选完按它的提示退出，本脚本接着守着。")
    print(f"$ {sourced_command(cmd)}")
    return run_foreground(cmd)


#: 摊开"另外几档"时那一行分隔标题。★ 用例拿它把两块切开：默认那一块里**不许**混进
#: 别的档才会起的命令（`--dry-run up_manual` 里出现 `control` 的命令就是这条判据要抓的）。
OTHER_CHOICES_HEADER = "── 其他档（起链前问一句，选到才起）──"


def _render_steps(steps: Sequence[str], handoff: Sequence[str], closing: Sequence[str],
                  *, extra_names: Sequence[str] = ()) -> List[str]:
    """把一条链摊成行（含最后那段 `closing`）—— `render_chain` 的**两块共用**这一份
    （免得第二份实现跟第一份漂）。★ `closing` 由调用方给全：**默认那一块**末尾还有
    "怎么收尾"那一句，**另外几块**只列命令、说完就停（`closing=()`）。"""
    lines: List[str] = []
    for index, key in enumerate(steps, start=1):
        action = ACTION_BY_KEY[key]
        chosen = tuple(UP_OPTIONALS.get(key, ())) + tuple(extra_names)
        cmd = (action.cmd or "") + optionals_suffix(
            action, [opt.name in chosen for opt in action.optionals])
        lines.append(f"{index}. {action.title}")
        lines.append(f"   $ {sourced_command(cmd)}")
        has_criterion = wait_kind(key) is not None
        wait = f"（最多 {STEP_TIMEOUT_S[key]:.0f} s）" if has_criterion else ""
        lines.append(f"   等：{wait_hint(key)}{wait}")
        if action.warn_serial:
            lines.append(f"   起之前先看 `{SERIAL_PORT}` 被谁占着（占了就问一句）")
        if action.confirm:
            lines.append(f"   起之前会问：{action.confirm}")
    for key in handoff:
        action = ACTION_BY_KEY[key]
        lines.append(f"{len(steps) + 1}. ★ 把终端让给「{action.title}」（**前台**，人要敲键）")
        lines.append(f"   $ {sourced_command(action.cmd or '')}")
    if closing:
        lines += ["", *closing]
    return lines


def render_chain(chain_key: str) -> str:
    """`--dry-run up` / `--dry-run up_manual` 打印的东西：**默认那一档**的顺序、每步命令、判据、
    要人确认的话、起完那几句；**再另起一块**把另外几档的命令也摊开 —— **一条都不起**。

    ★ 两块分开写是有判据的：默认那一块说的是"起链时**不问**也会发生的事"，
      另外几块是"选到才发生的事"（`OTHER_CHOICES_HEADER` 就是那条分界线）。
    ★ **只有一档的链（菜单 8）没有第二块** —— 摊出来说"选到别的档"是没有的事。
    """
    chain = CHAIN_BY_KEY[chain_key]
    default = chain.choice(chain.default_choice)
    lines = [f"{chain.label}（`run/z0.py {chain_key}`）—— 本模式**一条都不起**，只是把要发生的事摊开：",
             f"★ 下面摊开的是**默认那一档（第 {chain.default_choice} 档）**"
             + ("（本链只有这一档）。" if len(chain.choices) == 1 else "，空回车走的就是它。"), ""]
    lines += _render_steps(chain.steps, chain.handoff,
                           chain.closing + ("★ 收尾：跑起来之后前台守着，**Ctrl-C 一起停**"
                                            "（只停它起的那些）。",),
                           extra_names=default.extra_optionals)
    others = [(number, choice) for number, choice in enumerate(chain.choices, start=1)
              if number != chain.default_choice]
    if not others:                            # ★ 只有一档 ⟹ **没有**「其他档」那一块（菜单 8）
        lines += ["", f"★ 本链只有一档（第 {chain.default_choice} 档），起链时**不问**。"]
        return "\n".join(lines)
    lines += ["", OTHER_CHOICES_HEADER, "",
              f"★ 起链前会问一句「谁在上位控制飞机」（{len(chain.choices)} 档）；"
              f"**空回车 = 第 {chain.default_choice} 档**"
              "（上面那一块）。选到别的档时，尾巴换成下面这些（起完那几句在起链时印）："]
    for number, choice in others:
        lines += ["", f"  [{number}] {choice.title}"]
        lines += ["  " + line for line in _render_steps(
            MANUAL_CHAIN + choice.tail, choice.handoff, (), extra_names=choice.extra_optionals)]
    return "\n".join(lines)


def cmd_chain(chain_key: str) -> int:
    """一键：**先问这一趟谁在上位控制飞机**，再按选中的那一档的顺序逐段起，
    **每段等到它的判据成立**再走下一步；然后（`handoff`）把终端让出去，
    最后打印核对表 + 起飞提示并前台守着。"""
    chain = CHAIN_BY_KEY[chain_key]
    ensure_sourced()          # ★ 必须在提问**之前**：它会把本进程 exec 掉重开，先问就白问了
    choice = ask_chain_choice(chain)
    steps = MANUAL_CHAIN + choice.tail
    log_dir = RUN_DIR / "logs" / f"{time.strftime('%Y%m%d-%H%M%S')}-{chain_key}"
    started: List[Child] = []
    probe = Probe()
    total = len(steps)
    try:
        print(f"{chain.label}：{' → '.join(steps)}")
        print(f"日志目录：{log_dir}/")
        print("★ 起 `mocap` 之前飞机必须**已摆好并静止**（阶段 A 原点在它启动那一刻锁定）。")
        print()
        for index, key in enumerate(steps, start=1):
            if key == "mavros" and STEP_READY["mavros"](probe.snapshot(2.0)):
                # 现场常态：mavros 在别的终端里长期跑着 ⟹ 别起第二个（串口只有一个）
                if ask_confirm("★ 检测到 mavros **已经在跑**（`/mavros/state` connected=True）—— 沿用它？"
                               "（n = 照样再起一个，多半会因串口被占而失败）", default=True):
                    print(f"[{index}/{total}] mavros：**沿用已经在跑的那个** —— 不起新的，"
                          "收尾时也不会停它。")
                    continue
            ok, _ = start_and_wait(probe, key, log_dir, started, index=index, total=total,
                                   extra_names=choice.extra_optionals)
            if not ok:
                print()
                print(f"✗ 起链中断 —— 先把这一步的问题解决掉（日志在上面），再重跑 `run/z0.py {chain_key}`。")
                return 1
            print()

        for key in choice.handoff:      # ★ 是**选出来的那一档**要不要让终端，不是默认那档
            print()
            handover(key)               # ★ 前台：Ctrl-C 归上面那个 handler（= 一起停）

        print("=" * 72)
        print(render_snapshot(probe.snapshot(4.0)))
        print("=" * 72)
        for line in choice.closing:
            print(line)
        print(f"★ 日志：{log_dir}/（例如 `tail -f {log_dir}/{total:02d}-{steps[-1]}.log`）")
        print("★ 现在前台守着 —— **Ctrl-C 一起停**（只停本脚本起的那些）。")
        return watch_children(started)
    except KeyboardInterrupt:
        print("\n（Ctrl-C：停本脚本起的那些 —— 先 SIGINT 让各节点自己收摊，最多等 5 s）")
        return 0
    finally:
        probe.close()
        stop_children(started)


def cmd_up_manual() -> int:
    """第 8 项：动捕 + mavros（手动遥控飞）。"""
    return cmd_chain("up_manual")


def cmd_up() -> int:
    """第 9 项：全部程序（含把终端让给操作台选轨迹）。"""
    return cmd_chain("up")


# ─────────────── 清除后台节点（第 10 项，ticket 22 §八，2026-09-23） ───────────────
# ★ 编号 2026-09-23 顺延过一次（原第 11 项）：菜单收成 10 项，两条一键占 8/9（ticket 22 §九）。
#
# ★ 为什么不是"停 `up` 起的那些"：`up` 的子进程表（`started`）是 `cmd_up` 的**局部变量**，
#   脚本一退就没了 —— 恰恰在"起链的终端被关了 / 被 `kill -9` 了、节点成了孤儿"这种最需要
#   清除的场合，那份表也不在了。所以这一条**按名字扫**（操作者 2026-09-23 拍定），
#   代价写在 `DOWN_WARNING` 里：别人会话/测试起的同类节点也会被扫进来。


@dataclass(frozen=True)
class DownTarget:
    """一类要清除的进程。`pattern` 是 `pgrep -f` 的正则（**白名单，写在表里，不接受人输**）。

    `key` 与 `ACTIONS` / `UP_CHAIN` 的 key 对齐 —— 不是另起一套命名；`label` 只用于打印。
    """

    key: str
    label: str
    pattern: str


#: 要扫的进程。**逐条对着起链那几条命令写**（`ros2 launch <包> …`）加上它们拉起来的**可执行名**
#: （`ros2 launch` 的父进程与节点本身是两个人 ⟹ 两种都要，`scan_down_targets` 按 pid 去重）。
#: ⚠️ 模式**不许写宽**（不许出现 `python3` / `bash` / `.*` 这类）：它是 `pgrep -f`，宽一寸就误伤一片。
#: ★ **`bringup` 那条只有前半**：它本身就是个 launch 壳（`ros2 launch bringup z0.launch.py`），
#:   它拉起来的 mocap / traj / control 三个节点由**各自那三行**兜住 ⟹ 这一条故意不写自己的可执行名
#:   （写 `z0.launch.py` 也只是重复匹同一个进程）。其余五行都是"命令 + 可执行名"两个人。
DOWN_TARGETS: Tuple[DownTarget, ...] = (
    #: ★ 2026-10-13：起链改走本仓的 `bringup mavros.launch.py`（见文件头那条口径澄清），
    #: 所以模式**必须同时认这两条**：新写法（带包名 `bringup`）与旧的上游写法
    #: （现场可能仍有终端按老 runbook 起的）—— 再加裸的 `mavros_node`。
    #: ⛔ 别只留新的：那会让"按老命令起的那条"清不掉，串口一直占着。
    DownTarget("mavros", "mavros（独占串口）",
               r"ros2 launch bringup mavros.launch.py|ros2 launch mavros |(/| )mavros_node( |$)"),
    DownTarget("vrpn", "vrpn_listener（动捕数据源）",
               r"ros2 launch vrpn_listener |(/| )vrpn_listener( |$)"),
    DownTarget("mocap", "mocap（相对位姿）",
               r"ros2 launch mocap |(/| )mocap_rel_pose_node( |$)"),
    DownTarget("traj", "traj（参考轨迹）",
               r"ros2 launch traj |(/| )traj_node( |$)"),
    DownTarget("control", "control（状态机）",
               r"ros2 launch control |(/| )control_node( |$)"),
    #: ★ ticket 14：第 9 项的第 2 档（MPC）会起它（`ros2 run nmpc mpc_node.py`）⟹
    #: 起链能起的，清除就必须能扫到（判据是下面那条用例：`UP_CHAIN` 每一条都在这里）。
    #: ★ 它是个**可执行文件**（`install(PROGRAMS)`），不是 launch ⟹ 模式里认的是
    #: `lib/nmpc/mpc_node.py` 那个路径尾巴（`(/| )mpc_node.py( |$)`），与 control 那行同形。
    DownTarget("mpc", "mpc 节点",
               r"ros2 run nmpc mpc_node.py|(/| )mpc_node.py( |$)"),
    DownTarget("bringup", "bringup 一键（launch 壳）",
               r"ros2 launch bringup "),
)

#: 停之前**必须让人看见**的那句话。事实来源是 `params-changes.md` §H.7（"急停 = 停桨"那张表）/
#: `operation_manual.md` §1.2（"想停任务，只有两条软路径"，操作者那一版）—— **屏上要指人去看的那一处
#: 写 runbook §1.2**：这行字是给手上拿着遥控器的人看的，他手边那一页就是 runbook。
DOWN_WARNING = (
    "⚠️ 清除 = **停进程**，它**不会让飞机降落**：setpoint 一停，PX4 会在 `COM_OF_LOSS_T`（1.0 s）后\n"
    "   自己退出 OFFBOARD，按 `COM_OBL_RC_ACT`（现值 `0`）落回 Position；而 **A1 还没落实**，\n"
    "   遥控同时丢失时 `NAV_RCL_ACT = 2` 会让它**试图返航**（室内没有 GPS）。\n"
    "   飞机若还在空中，先按 `operation_manual.md` **§1.2 的两条软路径**处置（打杆接管 / 拨 ch6），\n"
    "   别指望这一条 —— 依据在 `params-changes.md` §H.7。\n"
    "★ 这是**按名字**扫的（`DOWN_TARGETS`）：**别人会话、跑着的测试**起的同类节点也会在清单里。"
)

#: 先礼后兵的信号阶梯与各自的等待上限 —— 与 `bench` 的 `stop_dut`（`scenario.py:298`）同一条纪律：
#: 先 SIGINT 让节点走自己的收摊逻辑，超时才升级。★ 最后一档才是 SIGKILL。
STOP_LADDER: Tuple[Tuple[int, float], ...] = (
    (signal.SIGINT, 5.0), (signal.SIGTERM, 3.0), (signal.SIGKILL, 2.0),
)


def parse_pgrep(text: str) -> List[int]:
    """`pgrep -f` 的输出（一行一个 pid）→ pid 表：**去重、保序**，脏行丢掉。"""
    pids: List[int] = []
    for line in text.splitlines():
        line = line.strip()
        if line.isdigit():                 # 非数字行（警告之类）不是 pid ⟹ 丢掉，不炸
            pid = int(line)
            if pid not in pids:
                pids.append(pid)
    return pids


def _pgrep_ids(pattern: str) -> List[int]:
    """按模式找 pid（`pgrep -f`：匹的是**整条命令行**）。找不到就是空表（pgrep 退 1，不是错误）。"""
    if shutil.which("pgrep") is None:
        return []
    try:
        out = subprocess.run(["pgrep", "-f", "--", pattern],
                             capture_output=True, text=True, timeout=10)
    except (OSError, subprocess.SubprocessError):
        return []
    return parse_pgrep(out.stdout)


def _pid_cmdline(pid: int) -> str:
    """这个 pid 的命令行（打印给人看 ⟹ 空白压成单空格）。取不到就空串。"""
    try:
        out = subprocess.run(["ps", "-o", "args=", "-p", str(pid)],
                             capture_output=True, text=True, timeout=10)
    except (OSError, subprocess.SubprocessError):
        return ""
    return " ".join(out.stdout.split())


def _pid_state(pid: int) -> str:
    """`/proc/<pid>/stat` 里的状态字符（`Z` = 僵尸），取不到就空串。

    ★ 解析用 `rsplit(") ", 1)`：命令行（`comm`）**可能自带空格甚至括号**，
    但从**最后一个** `") "` 往后切永远是对的。
    """
    try:
        with open(f"/proc/{pid}/stat") as fh:
            return fh.read().rsplit(") ", 1)[-1].split(" ", 1)[0]
    except (OSError, IndexError):
        return ""


def _pid_alive(pid: int) -> bool:
    """只探存在性，**不发信号**。权限不足也算"活着"（存在、但我们动不了它）。

    ⚠️ **僵尸要单独判**：`os.kill(pid, 0)` 对僵尸进程**是成功的**（它还在进程表里），
    只看它就会对着尸体把阶梯升到 SIGKILL，最后报一句假的"没停掉"、退出码非零
    —— 2026-09-23 拿两个真进程试 `stop_pids` 时撞上的（`/proc/<pid>/stat` 的状态是 `Z`）。
    """
    if _pid_state(pid) == "Z":
        return False
    try:
        os.kill(pid, 0)
    except ProcessLookupError:
        return False
    except PermissionError:
        return True
    return True


def _send_signal(pid: int, sig: int) -> None:
    """给**单个 pid** 发信号。★ 有意不用 `os.killpg` —— 理由见 `cmd_down`。"""
    try:
        os.kill(pid, sig)
    except (ProcessLookupError, PermissionError):
        pass


@dataclass(frozen=True)
class DownHit:
    """扫到的一个进程：它属于哪一条（`key`/`label`）+ 快照下来的命令行。"""

    key: str
    label: str
    pid: int
    cmdline: str


def scan_down_targets() -> List[DownHit]:
    """按 `DOWN_TARGETS` 逐条扫，**按 pid 去重**（同一个人会被几条模式同时命中）。

    ★ 亲手把自己（以及父进程）剔掉 —— 否则"清除脚本把清除脚本杀了"。
    ★ 僵尸也算"不在了"（`_pid_alive`）：它已经死了，只是父进程还没收尸；
      列进"要停谁"只会让人以为没清干净。
    """
    hits: List[DownHit] = []
    seen: set = set()
    untouchable = {os.getpid(), os.getppid()}
    for target in DOWN_TARGETS:
        for pid in _pgrep_ids(target.pattern):
            if pid in seen or pid <= 1 or pid in untouchable or not _pid_alive(pid):
                continue
            seen.add(pid)
            hits.append(DownHit(key=target.key, label=target.label, pid=pid,
                                cmdline=_pid_cmdline(pid)))
    return hits


def render_down(hits: Sequence[DownHit]) -> str:
    """把清单摊开（**纯函数**）—— 动手之前，人得先看清"要停谁"。

    ★ **命令行一个字都不截**：按名字扫会捞到别人会话的同类节点，而唯一的缓解手段就是
    人**看一眼命令行**（不是你这套的就答 n）⟹ 截断等于把唯一的判据切掉。
    第一手（2026-09-23 复核抓到）：原来写的 `[:88]` 把 mavros 那条正好切在
    `fcu_url:=/dev/ttyTHS1`（尾巴 `:921600` 没了）、把 `mavros_node` 那条切在 `--param` 中间。
    长就让它换行 —— 终端会折，判据不能丢。
    """
    if not hits:
        return "没有扫到任何在跑的节点 —— 没什么可清的。"
    lines = [f"扫到 {len(hits)} 个进程（按**名字**扫，模式在 `DOWN_TARGETS` 里）："]
    for index, hit in enumerate(hits, start=1):
        lines.append(f"  {index:2d}  {pad(hit.label, 26)}  PID {hit.pid:<7}  {hit.cmdline}")
    return "\n".join(lines)


def stop_pids(pids: Iterable[int], *, alive: Optional[Callable[[int], bool]] = None,
              send: Optional[Callable[[int, int], None]] = None,
              sleep: Optional[Callable[[float], None]] = None) -> List[int]:
    """按信号阶梯停掉这些 pid，返回**还活着的**（杀不掉要如实报出来，不许假装成功）。

    ★ 只发**单个 pid**，不发进程组：按名字扫出来的进程可能与**别人终端**的 shell 同组，
      对那个组放 `killpg` 等于连人家的终端会话一起收掉。
    ★ pid `0`/`1` 与自己一律跳过（给它们发信号不是"清除"，是事故）。
    """
    alive = alive or _pid_alive
    send = send or _send_signal
    sleep = sleep or time.sleep
    targets = [pid for pid in dict.fromkeys(pids) if pid > 1 and pid != os.getpid()]
    for sig, grace_s in STOP_LADDER:
        pending = [pid for pid in targets if alive(pid)]
        if not pending:
            break
        for pid in pending:
            send(pid, sig)
        remaining = grace_s               # ★ 用倒计时而不是看墙上的钟：注入的假 sleep 也能跑完
        while remaining > 0 and any(alive(pid) for pid in pending):
            sleep(0.1)
            remaining -= 0.1
    return [pid for pid in targets if alive(pid)]


def _missing_scan_tools() -> List[str]:
    """扫描靠的两个外部命令（`pgrep` 找 pid、`ps` 取命令行）缺了哪个。

    ★ **缺工具与"没有可清的"在屏幕上长得一样**，而后果相反 —— 这个仓库为"静默为空"
    已经交过两次学费（`check` 的 `ros2 daemon` 坑、半拉子环境那两条），所以单拎出来问一句。
    """
    return [name for name in ("pgrep", "ps") if shutil.which(name) is None]


def cmd_down() -> int:
    """清除后台节点：扫 ⟹ 列清单 ⟹ 一次 y/N ⟹ 先礼后兵停掉 ⟹ 再扫一遍看结果。

    ★ **不给"按名字杀"的自由**：模式是 `DOWN_TARGETS` 里那张白名单（与人输的参数无关）。
    """
    ensure_sourced()
    missing = _missing_scan_tools()
    if missing:
        print(f"⚠️ 本机缺 {'、'.join(missing)} —— **扫不出清单来**，所以这一条什么都没做。\n"
              "   别把这句当成「已经清干净了」：`down` 判活的依据就在这两个命令上。")
        return 1
    hits = scan_down_targets()
    print(render_down(hits))
    if not hits:
        print("  " + serial_line(serial_state(), occupied="（上面没扫到，可能不是本脚本认得的名字）"))
        return 0
    print()
    print(DOWN_WARNING)
    if not ask_confirm(f"把这 {len(hits)} 个进程停掉？（先 SIGINT，超时才 SIGTERM / SIGKILL）",
                       default=False):
        print("已取消。")
        return 1
    print("  信号阶梯：SIGINT → 等 5 s → SIGTERM → 等 3 s → SIGKILL（与 `bench` 的 `stop_dut` 同一纪律）")
    survivors = stop_pids([hit.pid for hit in hits])
    print()
    left = scan_down_targets()
    if survivors:
        # ★ 这里**不会**再报僵尸（`_pid_alive` 已经把 `Z` 判成死了）⟹ 剩下的两条真实成因是：
        #   不响应信号的（D 状态、卡在驱动里）与**我们没权限动**的（别人的会话、别的 uid）。
        print(f"⚠️ 这 {len(survivors)} 个没停掉（不响应信号，或权限不够 —— `ps -o pid,stat,user,args -p …`）："
              f"{'、'.join(str(pid) for pid in survivors)}")
    if left:
        print(f"⚠️ 再扫一遍还剩 {len(left)} 个：\n{render_down(left)}")
    print("  " + serial_line(serial_state(),
                             occupied="还占着它的大概率是没停掉的 mavros —— 再看一眼上面的清单",
                             idle="mavros 停了它就该空出来"))
    return 1 if (survivors or left) else 0


# ─────────────────────────── 交互菜单 ───────────────────────────


def menu_text() -> str:
    lines = [
        "z0 起链菜单（`run/z0.py`）—— 命令、核对、前提，一个入口",
        "",
        "  ★ 本脚本**不提供**任何会动飞机的入口：解锁 / 起飞 / 切模式 / 改参数 / 发帧都没有。",
        "    起飞只有两条路：遥控器开始键（CH9 第 1 次上升沿）或台架的 `/control/takeoff` 服务",
        "    （后者照旧手打，原文在 `docs/control/bench-runbook.md` T4）。",
        "  ★ 起 `mocap` 之前，飞机必须**已摆好并静止**（阶段 A 原点在启动那一刻锁定）。",
        "",
    ]
    for index, action in enumerate(MENU_ACTIONS, start=1):
        lines.append(f"  {index}  " + pad(action.title, 36) + action.note)
    lines.append("  0  退出")
    return "\n".join(lines)


def ask_confirm(question: str, default: bool = False) -> bool:
    """问一句 y/N。`default` 决定**空回车**算什么（提示符里看得出来）。"""
    try:
        reply = input(f"{question} {'[Y/n]' if default else '[y/N]'} ").strip().lower()
    except EOFError:                    # 非交互（管道/重定向）⟹ 走默认，别抛异常
        return default
    return default if not reply else reply in ("y", "yes")


def ask_chain_choice(chain: "Chain") -> "ControllerChoice":
    """起链前那一句：「这一趟谁在上位控制飞机」—— 输数字，**空回车 = 该菜单项的默认档**。

    ★ **只有一档就不问**（第 8 项，2026-10-16 复原）：直接返回它，屏上照旧把"这一档起什么"
      打出来（不问 ≠ 不说）。这样菜单 8 的屏与一版之前**逐字一致**，也免得问一句只有一个答案的话。
    ★ 这一问必须在 `ensure_sourced()` **之后**：那一步在没 source 时会把本进程 `exec` 掉重开，
      先问就会把答案丢掉（现象是"问了等于没问"，而且不报错）。
    ★ 打错的**再问一遍**（不猜、不吞）；`EOFError`（管道/重定向）⟹ 走默认 ——
      与 `ask_confirm` 同一条纪律。
    """
    default = chain.default_choice
    if len(chain.choices) == 1:               # ★ 只有一档 ⟹ 不占操作者的手
        only = chain.choices[0]
        print()
        print(f"★ {chain.label}：这一趟**谁在上位控制飞机**？只有一档，**不问** —— {only.title}")
        return only
    print()
    print(f"★ {chain.label}：这一趟**谁在上位控制飞机**？")
    for number, option in enumerate(chain.choices, start=1):
        print(f"   {number} = {option.title}" + ("（默认）" if number == default else ""))
    while True:
        try:
            reply = input(f"选择（数字，回车 = {default}）：").strip()
        except EOFError:
            print(f"（非交互 ⟹ 走默认第 {default} 档）")
            return chain.choice(default)
        if not reply:
            return chain.choice(default)
        if reply.isdigit() and 1 <= int(reply) <= len(chain.choices):
            return chain.choice(int(reply))
        print(f"  ⚠️ 不认识的选项：{reply!r} —— 请输 1–{len(chain.choices)}")


def launch_action(action: Action) -> int:
    """跑一个启动动作：先看串口、再要确认，最后前台跑（Ctrl-C 回到菜单）。"""
    if not serial_gate(action):
        return 1
    if action.confirm and not ask_confirm(action.confirm):
        print("已取消。")
        return 1
    answers = [ask_confirm(opt.prompt) for opt in action.optionals]
    try:
        cmd = (action.cmd or "") + optionals_suffix(action, answers)
    except ValueError as exc:
        print(f"  ⛔ {exc} —— 这一趟没有起链。")
        return 1
    print(f"$ {sourced_command(cmd)}")
    print("（前台跑；Ctrl-C 停它，停完回到菜单）")
    try:
        return run_foreground(cmd)
    except KeyboardInterrupt:
        print("\n（已 Ctrl-C）")
        return 0


#: **在本进程里做**的动作（不起子进程）：两条只读的 + 两条一键 / 清除的编排。
#: ★ 判据 = **表里 `cmd` 是空的**（不是另抄一份名单）：今天正好就是 env/check/up/down 四条，
#: 上面那条 `READ_ONLY_KEYS` 用的是同一个套路 ⟹ 加一项菜单不用记着来这里改。
IN_PROCESS_KEYS: Tuple[str, ...] = tuple(a.key for a in ACTIONS if a.cmd is None)

#: 进程内动作里**吃位置参数**的那一条 —— `run/z0.py rec_start 3`（不给就问一句）。
#: ★ 只有它被喂 `extra`：其余几条（含 `env`/`check`/`up`/`down`）保持**零参数签名**，
#:   因为既有用例拿 `lambda: …` 当假替身（`test_z0_menu.py`）—— 改签名会连带打红一片，
#:   而"加一条菜单"不该动那些。
TAKES_ARGS: Tuple[str, ...] = ("rec_start",)


def in_process_command(key: str) -> Callable[[], int]:
    """按名字**现取**（不是 import 期就绑死）—— 这样用例能把它们替换掉。

    ★ 录制那四条在 `rec.py` 里，**函数里 import**（不是模块顶层）：这样 `z0.py`
    本身不依赖录制层，顶层的 import 面保持不变（`test_z0_menu.py` 只 import z0 就能跑）。
    """
    import rec                                     # noqa: PLC0415（理由见上）
    return {"env": cmd_env, "check": cmd_check, "up": cmd_up, "up_manual": cmd_up_manual,
            "down": cmd_down,
            "rec_start": rec.cmd_start, "rec_status": rec.cmd_status,
            "rec_manifest": rec.cmd_manifest, "rec_adopt": rec.cmd_adopt}[key]


def dry_run_note(key: str, extra: Sequence[str] = ()) -> str:
    """`--dry-run` 对**在本进程里做**的动作说什么 —— 各有各的话。"""
    if key in CHAIN_BY_KEY:
        return render_chain(key)
    if key == "down":
        # ★ 这一条**只看不停**：`--dry-run` 在这里格外要紧 —— 看一眼会停谁，然后什么都不发。
        return render_down(scan_down_targets()) + "\n（`--dry-run`：只扫一眼，**一个信号都不发**）"
    if key.startswith("rec_"):
        import rec                                 # noqa: PLC0415（同上）
        return rec.dry_run_note(key, extra)
    return f"（`{key}` 是只读动作：本进程里直接跑，没有命令可打印；`--dry-run` 不执行）"


def dispatch(key: str, dry_run: bool = False, extra: Sequence[str] = ()) -> int:
    if key in IN_PROCESS_KEYS:
        # ★ `--dry-run` 对**所有在本进程里做**的动作都是"不执行" —— 否则它就名不副实了
        # （`--dry-run check` 会真起一个 5 s 的探测，`--dry-run up` 会真把五个进程拉起来）。
        if dry_run:
            print(dry_run_note(key, extra))
            return 0
        fn = in_process_command(key)
        return fn(extra) if key in TAKES_ARGS else fn()
    action = ACTION_BY_KEY.get(key)
    if action is None:
        print(f"不认识的动作：{key}（用 `run/z0.py --list` 看有哪些）")
        return 2
    if dry_run:
        print(f"$ {sourced_command(action.cmd or '')}")
        for opt in action.optionals:
            print(f"   （起之前会问一句：{opt.prompt}）")
        return 0
    return launch_action(action)


def menu_loop() -> int:
    while True:
        print()
        print(menu_text())
        print()
        try:
            choice = input("选择（数字 / q 退出）：").strip()
        except EOFError:
            print()
            return 0
        if choice in ("0", "q", "quit", "exit"):
            return 0
        if choice.isdigit() and 1 <= int(choice) <= len(MENU_ACTIONS):
            key = MENU_ACTIONS[int(choice) - 1].key
        elif choice in ACTION_BY_KEY:
            key = choice
        else:
            print("不认识的选项。")
            continue
        try:
            dispatch(key)
        except KeyboardInterrupt:
            print("\n（已中断，回到菜单）")


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = argparse.ArgumentParser(
        prog="run/z0.py",
        description="z0 起链菜单 —— 常用启动 + 每步核对（不含任何会动飞机的入口）")
    parser.add_argument("action", nargs="?",
                        help="动作名（env/check/mavros/vrpn/mocap/traj/control/up_manual/up/down/"
                             "rec_start，另有不进菜单的 fly 与 rec_status/"
                             "rec_manifest）；不给就进交互菜单")
    parser.add_argument("extra", nargs="*",
                        help="动作的参数（`rec_start 3`）—— 除它以外都不吃参数")
    parser.add_argument("--list", action="store_true", help="只列菜单，不进入交互")
    parser.add_argument("--dry-run", action="store_true", help="只打印将要执行的命令，不执行")
    # ★ `parse_known_args` 而不是 `parse_args`：这条放行是给"动作自带 `--` 参数"留的口子。
    #   第一手（2026-09-29）：`z0.py --dry-run rec_fcu /tmp/x --since <ISO>` 撞到
    #   `error: unrecognized arguments: --since` —— 那一刻 `TAKES_ARGS` 里还有 `rec_fcu`。
    #   现在吃位置参数的只剩 `rec_start`，但这个口子**留着**（删了它，将来任何带
    #   `--flag` 的动作都会重蹈覆辙，而那种报错出现在**动作根本没法开始**的时候）。
    #   ⚠️ 只对 `TAKES_ARGS` 那一条放行：别的动作多给一个参数仍然要报错（打错字不许静默吞掉）。
    args, unknown = parser.parse_known_args(argv)

    if args.list:
        print(menu_text())
        return 0
    if args.action:
        if unknown and args.action not in TAKES_ARGS:
            parser.error(f"`{args.action}` 不吃参数，但收到了：{' '.join(unknown)}")
        return dispatch(args.action, dry_run=args.dry_run, extra=list(args.extra) + unknown)
    # ★ **进菜单之前**就把 source 重入走完（2026-09-23 现场反馈，第一手复现见下面这段）。
    #   `ensure_sourced()` 是把**本进程 `exec` 掉**重开 ⟹ 它必须发生在**读 stdin 之前**。
    #   原先它只在动作里做，而菜单是"先读完一行再派发"的 —— 于是用户敲的那一行
    #   随旧进程一起消失、菜单重印一遍，现场感受就是"**数字要敲第二次才认**"。
    #   第一手（未 source 的 shell，`printf "1\nq\n" | python3 run/z0.py`）：
    #   菜单印**两遍**、第一行凭空不见；提到这里之后只印一遍。
    ensure_sourced()
    return menu_loop()


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        sys.exit(130)
