#!/usr/bin/env python3
# =============================================================================
# z0 前置条件取证 —— MAVLink 探针
#
# 对应 ticket：.scratch/phase1/issues/01-hardware-prereqs.md 的 ②③④
#
# 它回答三件事：
#   ② 飞控挂在 /dev/ttyTHS1 还是 /dev/ttyTHS2   → --scan
#   ③ 飞控实际报出的固件版本号**原文**           → --port /dev/ttyTHSx
#   ④ 未解锁时的电池电流读数                     → --port /dev/ttyTHSx
#
# ⚠️ 安全纪律（本脚本**只读**，永不违反）：
#   * 只发两种指令，都**与飞行状态无关**：
#       - MAV_CMD_SET_MESSAGE_INTERVAL (511)：设遥测下行速率
#       - MAV_CMD_REQUEST_MESSAGE      (512)：请求重发某条消息
#   * **绝不**发解锁 / 起飞 / 模式切换 / 参数写入 / 任何 COMMAND_LONG 之外的指令。
#   * 不用 MAV_MODE_FLAG_SAFETY_ARMED 之类去"探测"——没那个必要。
#
# 为什么用 pymavlink 而不是手搓解码器：MAVLink v2 的 CRC 带 per-message
# `CRC_EXTRA` 种子，种子写错不会报错、只会**静默丢帧**。手搓等于赌一张背下来的
# 种子表。pymavlink 是权威实现，不赌。
#
# 依赖：pymavlink **和** pyserial —— ⚠️ **两个都要装**。
#   pymavlink 2.4.49 的 Requires-Dist 只有 `lxml` 与 `fastcrc`，**不含 pyserial**
#   （2026-09-21 在 z0 的 Jetson 上第一手核实：只装 pymavlink 会通过 import 检查、
#    却在打开串口时炸 `ModuleNotFoundError: No module named 'serial'`）。
#   装法（NxtPX4v2 上实测可用）：
#     python3 -m pip install --user pymavlink pyserial
# =============================================================================
from __future__ import annotations

import argparse
import os
import struct
import sys
import time

# 候选端口与波特率：PX4 的 TELEM 默认 57600；机载电脑链路常被改成 115200/921600
# FC_SCAN_PORTS 可覆盖（逗号分隔）—— 主要给自动化测试用，也可在非 THS 的机器上复用
#
# ⚠️ 为什么把 /dev/ttyAMA0 也列上（2026-09-21 补）：
#   Jetson Orin 上有**第三个 UART** —— `serial@31d0000`，`arm,sbsa-uart` 驱动，
#   设备名是 `ttyAMA0`（ARM PL011/SBSA 系）。**前六轮全把它漏了。**
#   它的 DT 节点**没有 `dmas`/`dma-names`**，因此**不受**那个接收 DMA 缺陷影响，
#   是个潜在绕开方案。
#   ⚠️ 它**不认** 8250 的 TIOCM_LOOP，所以 `uart_loopback.py --internal` 对它不适用，
#   只能靠物理短接判定。
DEFAULT_PORTS = [
    p for p in os.environ.get("FC_SCAN_PORTS", "").split(",") if p.strip()
] or ["/dev/ttyTHS1", "/dev/ttyTHS2", "/dev/ttyAMA0"]
DEFAULT_BAUDS = [57600, 115200, 921600]

FC_SYS_DEFAULT = 1
FC_COMP_DEFAULT = 1

# --- 心跳来源筛除：★ 2026-09-21 实测（ticket 01 ④ / 02 期间）------------------
# 这条链路上**同时**有两个心跳源：飞控 `(sysid=1, compid=1, type=2, autopilot=12)`
# 与另一个地面站 `(255, 190, 6, 8)`（255/190 是地面站系软件的署名）。
# 2026-09-21 实测：25 s 内**各 25 条**，都是 1 Hz。
# ⚠️ 而 `conn.wait_heartbeat()` 只认「收到一条 HEARTBEAT」，**不筛来源** ——
# 先到哪条就返回哪条。于是后续指令会发给 255 号系统（飞控从没被要求推消息，
# 表现是「读得到 HEARTBEAT，却一条遥测都收不到」），GCS 的 base_mode 还会被
# 当成飞控状态读出来。09-21 的两次误判都属于这个形状。
# ⚠️ 那条 `255/190` 心跳**不是我们自己发的**：pymavlink 的自署名是 compid=**0**
# （`mavutil.py` → `mavfile.__init__(source_system=255, source_component=0)`），
# 实测 compid=190 ⟹ 是**另一台地面站（QGC 系）经飞控转发**到这个口上的。
# 转发还是环回，对本判据**无影响**：它不是飞控，就必须筛掉。
GCS_SYSID = 255             # 地面站系软件的惯用 sysid（也是 pymavlink 的自署名）
# 后两个是 MAVLink 标准枚举，不是本地魔数：`MAV_TYPE_GCS = 6`、`MAV_AUTOPILOT_INVALID = 8`，
# 出自 `common.xml` 的 enum `MAV_TYPE` / `MAV_AUTOPILOT`（pymavlink 生成在
# `dispatches`/`dialects/v20/common.py`）。★ 自检里有一条**直接跟 pymavlink 的方言对值**，
# 免得哪天标准改了这里静默失配。
MAV_TYPE_GCS = 6
MAV_AUTOPILOT_INVALID = 8   # 地面站用的 autopilot 值；真飞控不会是它

# 我们只发这两条指令（见文件头安全纪律）
MAV_CMD_SET_MESSAGE_INTERVAL = 511
MAV_CMD_REQUEST_MESSAGE = 512
MSG_ID_BATTERY_STATUS = 147
MSG_ID_AUTOPILOT_VERSION = 148


def _fail_dependency() -> None:
    print("  ❌ 未安装 pymavlink —— 无法读飞控。")
    print("     装法：python3 -m pip install --user pymavlink")
    print("     装完重跑本脚本。②③④ 都需要它。")


def _open(port: str, baud: int):
    """打开端口。返回 (conn, None) 或 (None, 错误说明)。"""
    from pymavlink import mavutil

    try:
        return mavutil.mavlink_connection(port, baud=baud, source_system=255), None
    except Exception as exc:  # 权限 / 占用 / 设备不存在
        msg = f"{type(exc).__name__}: {exc}"
        hint = ""
        if "Permission" in msg or "errno 13" in msg.lower():
            hint = ("\n     → **权限问题**：当前用户不在 `dialout` 组。"
                    "\n       修：`sudo usermod -aG dialout $(id -un)` 然后**重新登录**"
                    "（仅加入组不重登不生效）。")
        elif "device reports readiness" in msg or "busy" in msg.lower():
            hint = "\n     → 端口被占用（mavros 在跑？先停掉再采）。"
        return None, msg + hint


# ---------------------------------------------------------------------------
# 原始流刻画 —— 区分「波特率错」和「线上根本不是数据」
#
# `--scan` 报「无心跳」时，背后其实是三种情况，而它们对操作者的指示**完全相反**：
#   (a) 读不到任何字节        → 线路静默：飞控没在发 / 这口没接线
#   (b) 有字节，但成不了帧    → 数据被切碎（**先怀疑接收通路把数据打坏了**）→ 先跑 uart_loopback.py
#   (c) 有字节且是完整帧块    → 是数据，只是波特率不对            → 去换波特率
#
# 判据是**非零字节的最长连续段**：MAVLink v2 最短的帧（心跳，无 payload）也有 17 字节，
# 任何真数据流必然出现 ≥17 的非零长段；轮到 (b) 这一档时，非零段总是被切得很碎，
# **无论成因是什么**（接收通路被打坏、噪声、接线问题……）。
#
# 2026-09-21 在 z0 的 Jetson 上第一手实测到 (b) 这种最误导的形状：
# ttyTHS1 @ 921600 读到 ~13 kB/s、**帧错误为 0**（接收器锁住了位时钟，看着"完全正常"），
# 但非零字节最长只有 **9 字节** —— 一个 MAVLink 帧都不可能有。
# 当时只看"有没有字节"就报了"有信号、线接对了"，是**误判**；本函数就是为此加的。
#
# ⚠️⚠️ 但 (b) 内部还藏着**两种成因**，本函数**分不开**（必须靠 uart_loopback.py 分开）：
#    b1. 线上真的是噪声 / 串扰 / RX 没被驱动          → 查接线
#    b2. **线上就是真数据，但被接收通路的驱动缺陷打坏了** → 查驱动（★ z0 实际命中的是这条）
# 第一手证明 b2 存在：`uart_loopback.py --internal` 的长模式回环
#   —— 完全不需要接线 —— 发 16 字节收回 16 个 0x00。
#   既然接收通路会把 ≥8 字节打成 0，那么**任何**真实数据流在 (b) 的形状下
#   都可能是被它打坏的，而不是"线上没有数据"。
#   ⟹ **看到 (b)，先跑 `uart_loopback.py --internal`，再谈接线。**
# ---------------------------------------------------------------------------
TIOCGICOUNT = 0x545D          # /usr/include/asm-generic/ioctls.h:99
_ICOUNT_FMT = "4i 2i 4i i 9i"  # struct serial_icounter_struct, /usr/include/linux/serial.h:102
_ICOUNT_SZ = struct.calcsize(_ICOUNT_FMT)

MAVLINK_MIN_FRAME = 17        # v2 最短帧：12 B 包头 + 2 B CRC + 心跳 payload 3 B


def _raw_characterize(port: str, baud: int, seconds: float = 1.0):
    """无 MAVLink 帧时，看线上到底是什么。返回 dict；取不到则 None。"""
    try:
        import fcntl
        import serial
    except ImportError:
        return None
    try:
        s = serial.Serial(port, baud, timeout=0.2)
    except Exception:
        return None

    def _ic():
        v = struct.unpack(_ICOUNT_FMT,
                          fcntl.ioctl(s.fileno(), TIOCGICOUNT, b"\x00" * _ICOUNT_SZ)[:_ICOUNT_SZ])
        return v[4], v[6], v[9]          # rx, frame, brk
    try:
        s.reset_input_buffer()
        rx0, fr0, bk0 = _ic()
        data = b""
        deadline = time.time() + seconds
        while time.time() < deadline:
            data += s.read(65536)
        rx1, fr1, bk1 = _ic()
    except Exception:
        return None
    finally:
        try:
            s.close()
        except Exception:
            pass

    longest = cur = 0
    for b in data:
        if b:
            cur += 1
            longest = max(longest, cur)
        else:
            cur = 0
    return {
        "bytes": len(data),
        "frame": fr1 - fr0,
        "brk": bk1 - bk0,
        "null_pct": (data.count(0) / len(data)) if data else 0.0,
        "max_run": longest,
        "seconds": seconds,
    }


def _explain_raw(c: dict) -> None:
    """把「无心跳」细化成三种可操作的情况。"""
    if c["bytes"] == 0:
        print(f"        └ {c['seconds']:.1f}s 内**一个字节都没有** ⟹ 线路静默")
        print("          → 这个口没接线，或飞控没在发。**先确认线接的是哪个口**。")
        return

    print(f"        └ 读到 {c['bytes']} 字节（{c['null_pct']:.0%} 是 0x00；帧错误 {c['frame']}，"
          f"break {c['brk']}；非零最长连续 **{c['max_run']} 字节**）")
    if c["max_run"] < MAVLINK_MIN_FRAME:
        print(f"          ⚠️ 看不到 ≥{MAVLINK_MIN_FRAME} 字节的非零连续段（最长只有 {c['max_run']}）——")
        print("             要么线上不是数据，要么**数据被驱动打坏了**。两者症状一样，必须分开：")
        print("             ★ **先排驱动缺陷**（本机第一手实测：JetPack 6.x 的 "
              "`nvidia,tegra194-hsuart` 接收通路 DMA 有缺陷，")
        print("               **≥8 字节的接收数据会被打成 0**）：")
        print("                 python3 tools/prereq-evidence/uart_loopback.py --internal")
        print("               → 若报「长模式回环数据被打坏」，**问题就在这里，与接线无关**。")
        print("                 修法：设备树删掉该串口节点的 `dmas`/`dma-names` 退回 PIO 后重启。")
        print("               ⚠️ 该缺陷**只影响接收**时也足以让 MAVLink 一个帧都解不出来。")
        print("             └ 若自检通过，才轮到接线：Jetson 的 **RX 要接飞控的 TX**（必须交叉），")
        print("               并确认 **GND 共地**。")
        print("             ⚠️ **两种情况下都不要去改波特率** —— 波特率错会给出长帧块，不是这个形状。")
    else:
        print(f"          → 像**真数据**（存在 ≥{MAVLINK_MIN_FRAME} 字节的长段）"
              " ⟹ 大概率只是波特率不匹配，继续试别的波特率。")


def _decode_flight_sw_version(raw: int) -> str:
    """PX4 把版本号打包成 (major<<24)|(minor<<16)|(patch<<8)。

    解出来只作参考 —— 真正的判据是飞控报出的 raw 十六进制原文，
    所以两样都打印，不让人只看到"解释过的"结果。
    """
    major = (raw >> 24) & 0xFF
    minor = (raw >> 16) & 0xFF
    patch = (raw >> 8) & 0xFF
    return f"{major}.{minor}.{patch}"


def _encode_git_hash_bytes(sha_hex: str, vendor: int = 0) -> bytes:
    """把 sha（十六进制字符串）按 PX4 的规则编成 `flight_custom_version` 的 8 字节。

    这是 `_decode_git_hash_bytes` 的**逆函数**，两边互为交叉验证（见 `--self-test`）。
    ⚠️ 只吃 `sha_hex` 的**前 16 个字符** —— 传完整的 40 位亦可（多余的会被掩码砍掉），
    但别指望它区分前 10 位相同的两个 commit。

    规则出自运行固件自身的源码（commit `4a0e65f2331b23b9c8352656e9ccb6edeaea5d6c`）：

      * `px_update_git_header.py:93` → `PX4_GIT_VERSION_BINARY = 0x` + sha 的**前 16 个字符**
      * `version.c:343`             → `px4_firmware_version_binary()` 原样返回它
      * `mavlink_main.cpp:1074`     → `& 0xFFFFFFFFFF000000`：**只留高 5 字节**（= 前 10 个字符）
      * `mavlink_main.cpp:1077`     → 小端 memcpy 整个 8 字节
      * `mavlink_main.cpp:1078`     → 前 3 字节再被厂商版本号覆盖
    """
    v = int(sha_hex[:16], 16) & 0xFFFFFFFFFF000000
    b = bytearray(v.to_bytes(8, "little"))
    b[0:3] = (vendor & 0xFFFFFF).to_bytes(3, "little")
    return bytes(b)


def _decode_git_hash_bytes(raw):
    """从 flight_custom_version 解出 git hash 前缀。

    ⚠️ **不是 ASCII 字符串** —— 第一手核实用的是**运行固件自己那棵树**
    （commit `4a0e65f2331b23b9c8352656e9ccb6edeaea5d6c`，`mavlink_main.cpp:1073-1078`，
    行号与 `_encode_git_hash_bytes` 的 docstring 一致）。
    上游 v1.15.0（tag `30e763b`）同一处是 `mavlink_main.cpp:1082-1087`，**编码相同**，可作交叉验证：

        /* use only first 5 bytes of git hash for firmware version */
        const uint64_t fw_git_version_binary = px4_firmware_version_binary() & 0xFFFFFFFFFF000000;
        const uint64_t fw_vendor_version = px4_firmware_vendor_version() >> 8;
        memcpy(&msg.flight_custom_version, &fw_git_version_binary, 8);
        memcpy(&msg.flight_custom_version, &fw_vendor_version, 3);

    所以三件事：
      1. 塞进去的是 **uint64 的二进制镜像（小端）**，不是 "4a0e65f2" 这样的字符串；
      2. `& 0xFFFFFFFFFF000000` 已经砍掉了低 3 字节 → 只保留 hash 的**前 5 字节**；
      3. 随后**前 3 字节又被厂商版本覆盖** → 只有 `[3:8]` 这 5 个字节还是 hash。

    `[3:8]` 逆序（小端还原）后得到 hash 的**前 10 个十六进制字符**。

    ⚠️⚠️ **2026-09-21 的修订史（别再把这道闸门加回来）**：
    本函数中间那版曾断言 `b[3:8]` 应当是 **ASCII** 的 `0-9a-f`，并因此把 z0 的真实读数
    `00 00 00 33 f2 65 0e 4a` 判成「取不到 git hash」。**那个断言是错的**：
    `b[3:8]` 是 hash 的**二进制**，本来就不该是 ASCII，所以那道闸门会把**任何**真固件都挡掉。
    解出来就是 `4a0e65f233` —— 与仓库里 commit `4a0e65f2331b23b9c8352656e9ccb6edeaea5d6c`
    （`Minderring/PX4-Autopilot`，"add micoair743-aio to PX4-Autopilot v1.14.3"）逐字对上。
    教训：**拿"我以为的编码"去否证"实测的字节"，而不是去查源码** —— 与 §0 那两条同源。

    返回 (原始 8 字节的 hex, 解出的 hash 前缀或 None)。
    """
    try:
        b = bytes(raw)
    except Exception:
        return "（无法按字节解）", None
    if not b:
        return "", None
    if set(b) == {0}:
        # 全 0 = 该字段没被填充（非 PX4 固件 / 未定义），不要当成 hash
        return b.hex(), None
    prefix = b[3:8][::-1].hex()          # 小端还原 → hash 的前 10 个十六进制字符
    if set(prefix) == {"0"}:
        # `[3:8]` 全 0 = 掩码后那几个字节本来就是 0（非 PX4 固件 / 厂商版本号占了整段），
        # 这不是 hash。⚠️ 注意这跟上面「整段全 0」不是同一回事，报错措辞要分开。
        return b.hex(), None
    return b.hex(), prefix


def is_foreign_heartbeat(hb) -> bool:
    """（纯函数）这条 HEARTBEAT 是不是「我们自己 / 别的地面站」，而不是飞控。

    ⚠️ 必须**先筛掉**再拿去当飞控用 —— 见文件上方 `GCS_SYSID` 那段踩坑记录。
    取不到字段时返回 True（**宁可跳过**：不可信的来源不能当飞控）。
    """
    try:
        if hb.get_srcSystem() == GCS_SYSID:
            return True
        if hb.type == MAV_TYPE_GCS:
            return True
        if hb.autopilot == MAV_AUTOPILOT_INVALID:
            return True
    except Exception:
        return True
    return False


def wait_fc_heartbeat(conn, timeout: float):
    """等到一条**飞控**心跳（跳过自己/别的地面站的心跳）。返回 `(hb, 跳过的条数)`。

    超时或对端静默 → `(None, 跳过的条数)`。
    ⚠️ 不要退回裸 `conn.wait_heartbeat()` —— 那正是 09-21 两次误判的成因。
    读串口的异常照旧向上抛，由调用方按各自的话术描述（probe_port 与 scan_ports 措辞不同）。
    """
    deadline = time.monotonic() + timeout
    skipped = 0
    while True:
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            return None, skipped
        hb = conn.wait_heartbeat(timeout=remaining)
        if hb is None:
            return None, skipped
        if is_foreign_heartbeat(hb):
            skipped += 1
            continue
        return hb, skipped


def probe_port(port: str, baud: int, wait: float, samples: int = 6) -> int:
    """读单个端口的 ③ 与 ④。返回 0 = 至少拿到一项。"""
    conn, err = _open(port, baud)
    if conn is None:
        print(f"  ❌ 打不开 {port} @ {baud}：{err}")
        return 1

    try:
        print(f"  端口 {port} @ {baud} baud —— 等心跳 …")
        # 打开成功 ≠ 能读到。pymavlink 用 timeout=0 + nonblocking() 打开，
        # 所以"口被别人占着"不会在 open 时抛错，而是在这里读不到东西。
        try:
            hb, skipped = wait_fc_heartbeat(conn, wait)
        except Exception as exc:
            print(f"     ⚠️ 读串口出错：{type(exc).__name__}: {exc}")
            print("       最可能：**mavros 正占着这个口**（同一台 Jetson 上跑 mavros 会抢字节流）"
                  "→ 先停掉 mavros 再采。")
            return 1
        if skipped:
            print(f"     ℹ️ 已跳过 {skipped} 条**非飞控**心跳"
                  "（sysid=255 / type=6 = 另一台地面站，本机飞控不是它）")
        if hb is None:
            print("     ⚠️ 未收到**飞控** HEARTBEAT —— 这个口上没听到飞控（或波特率不对）")
            print("       也可能是 mavros 正占着这个口把字节流抢走了 —— 确认它没在跑。")
            return 1

        # 组件号要从心跳本身取：mavutil 只自动填 target_system，
        # target_component 会留在 0（= 广播），那样指令可能没人应答。
        sysid = hb.get_srcSystem()
        compid = hb.get_srcComponent()
        print(f"     ✅ 收到 HEARTBEAT：system={sysid} component={compid} "
              f"type={hb.type} autopilot={hb.autopilot} "
              f"state=0x{hb.base_mode:02x}")
        print(f"        （autopilot=12 即 PX4；type=2 即四旋翼）")

        # --- 让飞控多吐我们要的两条消息（纯遥测速率，安全）-------------------
        for msg_id in (MSG_ID_AUTOPILOT_VERSION, MSG_ID_BATTERY_STATUS):
            try:
                conn.mav.command_long_send(
                    sysid, compid, MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                    float(msg_id), 1_000_000.0, 0, 0, 0, 0, 0,  # 1 Hz
                )
            except Exception:
                pass
        # 再直接点名要一次版本号
        try:
            conn.mav.command_long_send(
                sysid, compid, MAV_CMD_REQUEST_MESSAGE, 0,
                float(MSG_ID_AUTOPILOT_VERSION), 0, 0, 0, 0, 0, 0,
            )
        except Exception:
            pass

        got = 0

        # --- ③ 版本 ---------------------------------------------------------
        print()
        print("  【③ 固件版本】")
        ver = conn.recv_match(type="AUTOPILOT_VERSION", blocking=True, timeout=wait)
        if ver is None:
            print("     ⚠️ 未收到 AUTOPILOT_VERSION —— ③ **未采集**，"
                  "请改用 QGC 读（ticket 原文即如此）")
        else:
            raw = ver.flight_sw_version
            print(f"     flight_sw_version 原文 = {raw} (0x{raw:08x})")
            print(f"       按 PX4 编码解出 ≈ {_decode_flight_sw_version(raw)}"
                  f"   ← 仅供参考，判据是上面的原文")
            raw_bytes_hex, git_prefix = _decode_git_hash_bytes(ver.flight_custom_version)
            print(f"     flight_custom_version 原始字节 = {raw_bytes_hex}")
            if git_prefix is None:
                rawb = bytes(ver.flight_custom_version) if ver.flight_custom_version else b""
                if not rawb:
                    why = "该字段为空（长度 0）"
                elif set(rawb) == {0}:
                    why = "该字段**整段 8 字节全 0**（非 PX4 固件 / 未填充）"
                elif set(rawb[3:8]) == {0}:
                    why = "掩码该留 hash 那 5 个字节（`[3:8]`）**全是 0**"
                else:
                    why = "字节形态无法解释（见上）"
                print(f"       → **取不到 git hash** —— {why}。")
                print("         → 那就只以 flight_sw_version 的原文为准，**不要**硬凑 hash。")
            else:
                print(f"       → git hash 前 10 位 = `{git_prefix}`"
                      f"   （`[3:8]` 小端还原；前 3 字节 `{raw_bytes_hex[0:6]}` 是**厂商版本号**，不是 hash）")
                print(f"       ★ 拿 `{git_prefix[:8]}` 去查：`git log --oneline -1 {git_prefix[:8]}`，"
                      "或 `https://github.com/<owner>/<repo>/commit/" + git_prefix[:8] + "`")
            print(f"     vendor_id={ver.vendor_id} product_id={ver.product_id}")
            print(f"     capabilities=0x{ver.capabilities:x}")
            print()
            # 期望值 = refs-manifest.md §2.5(d) 的定案（2026-09-21），不是那个已作废的 fork。
            EXPECTED_SHA = "4a0e65f2331b23b9c8352656e9ccb6edeaea5d6c"
            print("     ★ 与 refs-manifest.md §2.5(d) 的对照：定案的 z0 固件真身是 "
                  "`Minderring/PX4-Autopilot` @ `4a0e65f2`（**v1.14.3 系**的 MicoAir 厂商构建）。")
            print("       ⚠️ fork `Peize-Liu/PX4-Autopilot @ 5ba2e91f`（v1.15 世代）**与 z0 无关** —— "
                  "本脚本 2026-09-23 之前的版本把它当成了期望值，会给出**反的**判定。")
            if git_prefix is None:
                print("       ⚠️ 取不到 hash，这一栏无法核对 —— **不要**据此改动 §2.5。")
                print("          但注意：**上面 `flight_sw_version` 的原文仍是一个独立证据** ——")
                print("          若它解出的世代与 §2.5 的结论不符，那本身就要记进 §2.5。")
            elif EXPECTED_SHA.startswith(git_prefix[:8]):
                print(f"       ✅ `{git_prefix[:8]}` 与定案真身 `4a0e65f2` **一致** "
                      "→ §2.5(d) 的判定得到实测支持。")
            else:
                print(f"       ⚠️ `{git_prefix[:8]}` 与定案真身 `4a0e65f2` **不一致** —— "
                      "**以飞控报出的为准**：去查这个 commit 属于哪个仓库/分支，")
                print("          它才是真正的「z0 固件真身」；§2.5(d)/§2.6 要以它为准。")
            got += 1

        # --- ④ 电流 ---------------------------------------------------------
        print()
        print("  【④ 电池 / 电流】")
        print(f"     采 {samples} 个样本 —— A4 的判据是「**恒**为 0 / **恒**为怪值」，"
              f"单个样本不足以判定")
        cur_vals = []
        last = None
        deadline = time.time() + max(wait, 2.0) * 3
        while len(cur_vals) < samples and time.time() < deadline:
            m = conn.recv_match(type="BATTERY_STATUS", blocking=True,
                                timeout=max(0.5, wait / 2))
            if m is None:
                break
            last = m
            cur_vals.append(m.current_battery)

        if last is None:
            print("     ⚠️ 未收到 BATTERY_STATUS —— ④ **未采集**")
            print("       （可能：没接电池/电源模块；或换用 QGC 看）")
        else:
            volts = [v for v in last.voltages if v not in (0, 65535)]
            v0 = volts[0] / 1000.0 if volts else None
            # current_battery 单位 10mA，-1 = 未知
            lo, hi = min(cur_vals), max(cur_vals)
            print(f"     current_battery 样本 = {cur_vals}  (单位 10mA, -1 = 未知)")
            print(f"       范围：{lo} … {hi}   变化量 = {hi - lo}"
                  f"{'  ← **恒定不变**' if hi == lo else '  ← 有变化'}")
            print(f"     voltages[0]       = "
                  f"{'%.3f V' % v0 if v0 is not None else '（无有效电芯电压）'}")
            print(f"     current_consumed  = {last.current_consumed} mAh")
            print(f"     battery_remaining = {last.battery_remaining} %"
                  f"{'  (-1 = 未知)' if last.battery_remaining == -1 else ''}")
            print()
            print("     ★ A4 判定（**只有「未解锁时」才成立**）：")
            if len(cur_vals) < 3:
                print(f"       ⚠️ **样本不足（{len(cur_vals)} 个）—— 不给判定。**")
                print("          A4 的判据是「恒」，1–2 个样本无法区分"
                      "「缺陷导致的恒定值」与「正常的安静值」。")
                print("          → 重跑并加大 --samples，或改用 QGC 连续观察。")
            elif lo == -1:
                print("       → 飞控自报电流**不可用**（全 -1）⇒ **疑似中招**。")
            elif hi == lo and hi == 0:
                print(f"       → 电流恒为 0（{len(cur_vals)} 个样本全 0）"
                      " ⇒ **疑似中招**：ADC 通道缺陷。")
            elif hi == lo:
                print(f"       → 电流**恒为怪值** {hi}（≈{hi / 100.0:.2f} A，"
                      f"{len(cur_vals)} 个样本完全不变）⇒ **疑似中招**。")
            else:
                print(f"       → 电流**有变化**（{lo}…{hi}）⇒ **未中招**。")
                print("          未解锁时它应≈0；把「一个有代表性的读数 + 范围 + "
                      "**当时未解锁**」一起记进 A4。")
            print()
            print("     ⚠️ 电压通道未受该缺陷影响，所以电压读数可作 Phase 1 兜底。")
            print("     ⚠️ 该缺陷的后果：电流/电量保护全是空转，**参数无法绕过**"
                  "（编译期宏）。")
            print("     ⚠️⚠️ 但 2026-09-21 查明：那个 `CH(5)` 缺陷属于 fork 的 "
                  "`boards/hkust/nxt-dual`，")
            print("        而 z0 飞控实际跑的固件（v1.14.3 系，见 §2.5(d)）里没有 `hkust/` 这块板；")
            print("        它的 `boards/micoair/h743-*` 把电流通道写在 "
                  "`/* PC1 */ ADC1_CH(11)`，**与引脚相符、无该缺陷**。")
            print("        ⟹ 「未中招」不是因为运气，也不是「本该中招却没事」；"
                  "**别把它当成一个待修的缺陷**。")
            got += 1

        return 0 if got else 1
    finally:
        try:
            conn.close()
        except Exception:
            pass


def scan_ports(ports: list[str], bauds: list[int], wait: float) -> int:
    """② —— 逐个口 × 逐个波特率找 HEARTBEAT。"""
    print("  扫描：找哪个串口上有飞控心跳")
    print(f"  候选端口 = {', '.join(ports)}")
    print(f"  候选波特率 = {', '.join(str(b) for b in bauds)}")
    print()
    hits = []
    for port in ports:
        for baud in bauds:
            conn, err = _open(port, baud)
            if conn is None:
                print(f"    {port} @ {baud:<7} ❌ {err}")
                continue
            read_err = None
            skipped = 0
            try:
                hb, skipped = wait_fc_heartbeat(conn, wait)
            except Exception as exc:
                # 别让一个口读崩就掀翻整个扫描 —— 记下来继续试下一个
                hb = None
                read_err = f"{type(exc).__name__}: {exc}"
            finally:
                try:
                    conn.close()
                except Exception:
                    pass
            if read_err is not None:
                print(f"    {port} @ {baud:<7} ⚠️ 读取出错：{read_err}")
                print("      （若 mavros 正跑着，它会占住串口 —— 先停掉）")
                continue
            if hb is None:
                # ⚠️ 「只有 GCS 心跳」与「完全静默」是**两种不同的故障**（前者说明本机
                # 自己那半条链路是通的，后者说明线/口/电有问题），措辞必须分开。
                if skipped:
                    print(f"    {port} @ {baud:<7} — **只有地面站心跳**（{skipped} 条），没有飞控心跳")
                else:
                    print(f"    {port} @ {baud:<7} — 无心跳")
                char = _raw_characterize(port, baud)
                if char is not None:
                    _explain_raw(char)
            else:
                # 从心跳本身取 sysid —— conn 上面已经 close 了，别再依赖它的属性
                note = f"，已跳过 {skipped} 条 GCS 心跳" if skipped else ""
                print(f"    {port} @ {baud:<7} ✅ **飞控在此** "
                      f"(sysid={hb.get_srcSystem()}, type={hb.type}{note})")
                hits.append((port, baud))
            print()

    print("  ===== ② 结论 =====")
    if not hits:
        print("  未在任何候选端口/波特率上听到飞控。")
        print("  **每一格的判据在上面**，四种情况对应完全不同的动作：")
        print("    · 静默          → 飞控没上电，或线没接到这个口")
        print("    · 帧块残破      → ★ **先跑 uart_loopback.py --internal 排驱动缺陷**，")
        print("                       再查 TX/RX 是否交叉、GND 是否共地（**别去调波特率**）")
        print("    · 只是波特率不对→ 换波特率再试")
        print("  ★ 本机已知：JetPack 6.x 的 nvidia,tegra194-hsuart **接收通路 DMA 有缺陷**，")
        print("    ≥8 字节的接收数据会被打成 0 ⟹ MAVLink 一帧都解不出来。")
        print("  另：当前用户在不在 dialout 组？不在就打不开串口（见上面报错）。")
        print("  若 mavros 正在跑，它会占住串口 —— 先停掉再扫。")
        # 机器可读结果行：capture.sh 用它把 ② 的结果喂给 ③④，避免两边端口不一致
        print("RESULT: none")
        return 1
    if len(hits) == 1:
        port, baud = hits[0]
        print(f"  ✅ 飞控在 **{port}**，波特率 **{baud}**。")
        print(f"     → ticket 01 的 ② 可以就此定案。")
        print(f"RESULT: {port} {baud}")
        return 0
    print("  ⚠️ 命中多个（真串口上不该发生）："
          + ", ".join(f"{p}@{b}" for p, b in hits))
    print("     真 UART 的波特率必须匹配，所以同一个口通常只会命中一个。")
    print("     命中多个一般意味着：端口是 pty / 被转发 / 被镜像（例如测试假飞控）。")
    print(f"  → 暂取最低波特率那个：{hits[0][0]} @ {hits[0][1]}")
    # 仍然吐 RESULT：让 ③④ 至少用上与 ② 一致的端口，而不是退回"盲猜第一个 THS"
    print(f"RESULT: {hits[0][0]} {hits[0][1]}")
    return 1


# ---------------------------------------------------------------------------
# 自检 —— **不需要飞控、不需要串口**：
#   python3 tools/prereq-evidence/mavlink_probe.py --self-test
#
# 用真值做交叉验证：z0 的飞控 2026-09-21 报出的 8 字节是
#   `00 00 00 33 f2 65 0e 4a`
# 而它的固件 commit 是 `4a0e65f2331b23b9c8352656e9ccb6edeaea5d6c`
# （`Minderring/PX4-Autopilot`，"add micoair743-aio to PX4-Autopilot v1.14.3"，2024-11-18）。
# 编码函数按源码规则从 sha 推出字节，解码函数再从字节推回 sha 前缀 —— 两者必须互逆，
# 且**必须与飞控实测的字节逐字相同**。这条断言就是 09-21 那次误判的反例保护：
# 若有人再往里加"必须是 ASCII"之类的闸门，这里会立刻红。
# ---------------------------------------------------------------------------
SELFTEST_FW_SHA = "4a0e65f2331b23b9c8352656e9ccb6edeaea5d6c"
SELFTEST_FW_BYTES = bytes([0x00, 0x00, 0x00, 0x33, 0xF2, 0x65, 0x0E, 0x4A])


def _selftest() -> int:
    bad = 0

    def check(name, got, want):
        nonlocal bad
        ok = got == want
        bad += 0 if ok else 1
        print(f"  {'✅' if ok else '❌'} {name}：{got!r}" + ("" if ok else f"   ← 期望 {want!r}"))

    print("  【自检】flight_custom_version 的编解码（不需要飞控 / 串口）")
    check("按源码规则由 sha 编出的 8 字节 == 飞控实测字节",
          _encode_git_hash_bytes(SELFTEST_FW_SHA), SELFTEST_FW_BYTES)
    check("解码：原始 8 字节 hex", _decode_git_hash_bytes(SELFTEST_FW_BYTES)[0],
          SELFTEST_FW_BYTES.hex())
    check("解码：hash 前 10 位", _decode_git_hash_bytes(SELFTEST_FW_BYTES)[1],
          SELFTEST_FW_SHA[:10])
    check("厂商版本号只覆盖前 3 字节、不污染 hash 段",
          _decode_git_hash_bytes(_encode_git_hash_bytes(SELFTEST_FW_SHA, 0x010E03))[1],
          SELFTEST_FW_SHA[:10])
    check("全 0 字节 → 取不到 hash（不许硬凑）", _decode_git_hash_bytes(bytes(8))[1], None)
    check("`[3:8]` 全 0 但前面非 0 → 也取不到 hash（这是**另一条** None 路径）",
          _decode_git_hash_bytes(bytes([0xAA, 0xBB, 0xCC, 0, 0, 0, 0, 0]))[1], None)
    check("encode → decode 往返",
          _decode_git_hash_bytes(_encode_git_hash_bytes("30e763b" + "0" * 33))[1],
          "30e763b000")

    print("  【自检】心跳来源筛除（不需要飞控 / 串口）")
    # 这几条是 2026-09-21 那两次误判的回归保护：链路上同时有飞控心跳和一条
    # 地面站心跳（1 Hz 各一条），而 `wait_heartbeat()` 不筛来源、先到先返回。

    class _HB:
        """假心跳。（compid 只是为了让 fixture 读起来跟实测一致，判据不看它。）"""

        def __init__(self, sysid, compid, mtype, autopilot):
            self._sysid = sysid
            self.type, self.autopilot = mtype, autopilot

        def get_srcSystem(self):
            return self._sysid

    class _Conn:
        """假连接：按顺序吐心跳，吐完就静默。`calls` 用来验「有没有真去取」。"""

        def __init__(self, seq):
            self._seq = list(seq)
            self.calls = 0

        def wait_heartbeat(self, timeout=None):
            self.calls += 1
            return self._seq.pop(0) if self._seq else None

    gcs = _HB(255, 190, 6, 8)   # 实测链路上那条地面站心跳
    px4 = _HB(1, 1, 2, 12)      # 真飞控（z0 实测：sysid=1 / type=2 / autopilot=12）
    check("地面站心跳（255/190/6/8）→ 判为「非飞控」（必须跳过）", is_foreign_heartbeat(gcs), True)
    check("真 PX4 心跳 → 不跳过", is_foreign_heartbeat(px4), False)
    check("别的地面站（sysid=250 / type=6 / autopilot=8）→ 也跳过",
          is_foreign_heartbeat(_HB(250, 190, 6, 8)), True)
    check("取不到字段 → 跳过（不可信来源不当飞控用）", is_foreign_heartbeat(object()), True)

    # 常数不是魔数：直接跟 pymavlink 的方言对值（装不上 pymavlink 就跳过，不算失败）
    try:
        from pymavlink.dialects.v20 import common as _common
        check("MAV_TYPE_GCS 与 pymavlink 方言一致", MAV_TYPE_GCS, _common.MAV_TYPE_GCS)
        check("MAV_AUTOPILOT_INVALID 与 pymavlink 方言一致",
              MAV_AUTOPILOT_INVALID, _common.MAV_AUTOPILOT_INVALID)
    except ImportError:
        print("  ⏭  未装 pymavlink —— 跳过「枚举对值」2 项（不算失败）")

    hb, skipped = wait_fc_heartbeat(_Conn([gcs, gcs, px4]), 1.0)
    check("前面 N 条 GCS 心跳：跳过并等到真飞控", (hb is px4, skipped), (True, 2))
    hb, skipped = wait_fc_heartbeat(_Conn([gcs]), 1.0)
    check("只有 GCS 心跳（对端静默）→ None，但报出跳过条数", (hb, skipped), (None, 1))
    hb, skipped = wait_fc_heartbeat(_Conn([]), 1.0)
    check("全程静默 → None，跳过 0 条", (hb, skipped), (None, 0))

    conn = _Conn([gcs])   # 预算已耗尽：必须立刻返回，**不取也不消费**那条心跳
    hb, skipped = wait_fc_heartbeat(conn, 0.0)
    check("时间预算已耗尽（0 s）→ 立即返回，且没动过那条心跳",
          (hb, skipped, conn.calls, len(conn._seq)), (None, 0, 0, 1))

    print("  ✅ 自检全部通过" if not bad else f"  ❌ 自检有 {bad} 项失败")
    return 1 if bad else 0


def main() -> int:
    ap = argparse.ArgumentParser(
        description="z0 前置条件取证 —— 只读 MAVLink 探针",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    ap.add_argument("--self-test", action="store_true",
                    help="只验 flight_custom_version 的编解码 —— 不需要飞控，也不需要串口")
    ap.add_argument("--scan", action="store_true",
                    help="② 扫描候选端口/波特率，找出飞控挂在哪个串口")
    ap.add_argument("--port", help="③④ 直接指定串口，如 /dev/ttyTHS1")
    ap.add_argument("--baud", type=int, help=f"波特率（默认按 {'/'.join(map(str, DEFAULT_BAUDS))} 依次试）")
    ap.add_argument("--wait", type=float, default=4.0, help="每次等待秒数（默认 4）")
    ap.add_argument("--samples", type=int, default=6,
                    help="④ 电池电流采几个样本（默认 6）；A4 判据是「恒」，样本太少不给判定")
    args = ap.parse_args()

    # 自检不需要 pymavlink（也不碰串口），所以放在依赖检查之前
    if args.self_test:
        return _selftest()

    try:
        import pymavlink  # noqa: F401
    except ImportError:
        _fail_dependency()
        return 1

    if args.scan:
        return scan_ports(DEFAULT_PORTS, DEFAULT_BAUDS, max(args.wait, 2.5))

    if not args.port:
        ap.print_help()
        return 2

    bauds = [args.baud] if args.baud else DEFAULT_BAUDS
    for baud in bauds:
        if probe_port(args.port, baud, args.wait, max(args.samples, 1)) == 0:
            return 0
        print()
    print(f"  ❌ {args.port} 在 {bauds} 里都没有可用读数。")
    return 1


if __name__ == "__main__":
    sys.exit(main())
