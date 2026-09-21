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
import sys
import time

# 候选端口与波特率：PX4 的 TELEM 默认 57600；机载电脑链路常被改成 115200/921600
# FC_SCAN_PORTS 可覆盖（逗号分隔）—— 主要给自动化测试用，也可在非 THS 的机器上复用
DEFAULT_PORTS = [
    p for p in os.environ.get("FC_SCAN_PORTS", "").split(",") if p.strip()
] or ["/dev/ttyTHS1", "/dev/ttyTHS2"]
DEFAULT_BAUDS = [57600, 115200, 921600]

FC_SYS_DEFAULT = 1
FC_COMP_DEFAULT = 1

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


def _decode_flight_sw_version(raw: int) -> str:
    """PX4 把版本号打包成 (major<<24)|(minor<<16)|(patch<<8)。

    解出来只作参考 —— 真正的判据是飞控报出的 raw 十六进制原文，
    所以两样都打印，不让人只看到"解释过的"结果。
    """
    major = (raw >> 24) & 0xFF
    minor = (raw >> 16) & 0xFF
    patch = (raw >> 8) & 0xFF
    return f"{major}.{minor}.{patch}"


def _decode_git_hash_bytes(raw):
    """从 flight_custom_version 解出 git hash 前缀。

    ⚠️ **不是 ASCII 字符串** —— 这是第一手核实过的（上游 v1.15.0，tag `30e763b`，
    `src/modules/mavlink/mavlink_main.cpp:1082-1087`）::

        /* use only first 5 bytes of git hash for firmware version */
        const uint64_t fw_git_version_binary = px4_firmware_version_binary() & 0xFFFFFFFFFF000000;
        const uint64_t fw_vendor_version = px4_firmware_vendor_version() >> 8;
        memcpy(&msg.flight_custom_version, &fw_git_version_binary, 8);
        memcpy(&msg.flight_custom_version, &fw_vendor_version, 3);

    所以三件事：
      1. 塞进去的是 **uint64 的二进制镜像（小端）**，不是 "5ba2e91f" 这样的字符串；
      2. `& 0xFFFFFFFFFF000000` 已经砍掉了低 3 字节 → 只保留 hash 的**前 5 字节**；
      3. 随后**前 3 字节又被厂商版本覆盖** → 只有 `[3:8]` 这 5 个字节还是 hash。

    `[3:8]` 逆序（小端还原）后得到 hash 的**前 10 个十六进制字符**。

    返回 (原始 8 字节的 hex, 解出的 hash 前缀或 None)。
    """
    try:
        b = bytes(raw)
    except Exception:
        return "（无法按字节解）", None
    prefix = b[3:8][::-1].hex()
    # 全 0 = 该字段没被填充（非 PX4 固件 / 老版本 / 未定义），不要当成 hash
    if not b or set(b) == {0} or set(prefix) == {"0"}:
        return b.hex(), None
    return b.hex(), prefix


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
            hb = conn.wait_heartbeat(timeout=wait)
        except Exception as exc:
            print(f"     ⚠️ 读串口出错：{type(exc).__name__}: {exc}")
            print("       最可能：**mavros 正占着这个口**（同一台 Jetson 上跑 mavros 会抢字节流）"
                  "→ 先停掉 mavros 再采。")
            return 1
        if hb is None:
            print("     ⚠️ 未收到 HEARTBEAT —— 这个口上没听到飞控（或波特率不对）")
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
                print("       → 该字段全 0：**取不到 git hash**（非 PX4 固件，或该版本未填充）")
                print("         → 那就只以 flight_sw_version 的原文为准。")
            else:
                print(f"       → git hash 前 10 位 ≈ `{git_prefix}`"
                      f"   （二进制的**前 5 字节**，**前 3 字节已被厂商版本覆盖**）")
            print(f"     vendor_id={ver.vendor_id} product_id={ver.product_id}")
            print(f"     capabilities=0x{ver.capabilities:x}")
            print()
            print("     ★ 与 refs-manifest.md §2.5 的对照：文档判 z0 固件真身是 fork "
                  "`5ba2e91f`（v1.15 世代）。")
            if git_prefix is None:
                print("       ⚠️ 取不到 hash，这一栏无法核对 —— **不要**据此改动 §2.5。")
            elif "5ba2e91f".startswith(git_prefix) or git_prefix.startswith("5ba2e91f"):
                print(f"       ✅ `{git_prefix}` 与 `5ba2e91f` **一致** → §2.5 的判定得到实测支持。")
            else:
                print(f"       ⚠️ `{git_prefix}` 与 `5ba2e91f` **不一致** → "
                      "**以飞控报出的为准，§2.5 的结论要重开**。")
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
            print("        修复 = 重编重刷 fork 的 `pr_fix_hkust_nxt_dual` @ `8161c4cd`"
                  "（相对 `main` 只差一行 `CH(5)`→`CH(8)`）。")
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
            try:
                hb = conn.wait_heartbeat(timeout=wait)
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
                print(f"    {port} @ {baud:<7} — 无心跳")
            else:
                # 从心跳本身取 sysid —— conn 上面已经 close 了，别再依赖它的属性
                print(f"    {port} @ {baud:<7} ✅ **飞控在此** "
                      f"(sysid={hb.get_srcSystem()}, type={hb.type})")
                hits.append((port, baud))
            print()

    print("  ===== ② 结论 =====")
    if not hits:
        print("  未在任何候选端口/波特率上听到飞控。")
        print("  可能：① 飞控没上电；② 线没接；③ 波特率不在这三个里")
        print("        ④ 当前用户在不在 dialout 组？不在就打不开串口（见上面报错）。")
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


def main() -> int:
    ap = argparse.ArgumentParser(
        description="z0 前置条件取证 —— 只读 MAVLink 探针",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    ap.add_argument("--scan", action="store_true",
                    help="② 扫描候选端口/波特率，找出飞控挂在哪个串口")
    ap.add_argument("--port", help="③④ 直接指定串口，如 /dev/ttyTHS1")
    ap.add_argument("--baud", type=int, help=f"波特率（默认按 {'/'.join(map(str, DEFAULT_BAUDS))} 依次试）")
    ap.add_argument("--wait", type=float, default=4.0, help="每次等待秒数（默认 4）")
    ap.add_argument("--samples", type=int, default=6,
                    help="④ 电池电流采几个样本（默认 6）；A4 判据是「恒」，样本太少不给判定")
    args = ap.parse_args()

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
