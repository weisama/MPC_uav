#!/usr/bin/env python3
# =============================================================================
# z0 前置条件取证 —— UART 回环自检
#
# 对应 ticket：.scratch/phase1/issues/01-hardware-prereqs.md 的 ②
#
# 【它填的是哪个洞】
# ② 的原始观测是「两条 UART 都读不到任何字节」。这个现象至少有三条成因，
# 而它们**在软件侧看起来一模一样**：
#
#   A. 飞控侧根本没在发（没上电 / 没启动完 / **那个口上没有 MAVLink 实例**）
#   B. 接线没落在 Jetson 的 UART RX 上（接错针 / TX-RX 没交叉）
#   C. 排针的 UART 功能没被 pinmux 打开（Tegra234 的 pinmux 由 BPMP 配置，
#      内核 DT 里查不到 pinmux 节点，**用户态无法核实**）
#
# 本脚本用**回环**把 C（以及"针脚认错"）从 A/B 里摘出来：
#   把**你接飞控的那两针**（TELEM2 的 TX/RX 对应到 Jetson 的两针）短接，
#   然后本脚本从 TX 发一串已知模式、看 RX 能否原样收回。
#
#   收到 ⟹ Jetson 侧一路全通（引脚 + pinmux + 驱动 + 电平）
#            ⟹ 问题必在 A 或 B，**不要再怀疑 Jetson**
#   收不到 ⟹ 问题在 C 或针脚认错，**与飞控无关**
#
# ⚠️ 做回环时**请先把飞控侧那两根线从排针上拔掉**，否则：
#   你发出去的字节会进飞控的 RX（PX4 会当垃圾丢弃，无害，但回环测不准）。
#
# 【安全性】本脚本只做一件事：**往 TX 发一小段固定模式并读回**。
#   不发任何 MAVLink 指令，不涉及解锁 / 起飞 / 模式切换 / 参数写入。
#   即使线还接在飞控上，飞出去的也只是几个无意义的字节。
#
# 用法：
#   python3 tools/prereq-evidence/uart_loopback.py                     # 两口都测，921600
#   python3 tools/prereq-evidence/uart_loopback.py --baud 57600
#   python3 tools/prereq-evidence/uart_loopback.py --port /dev/ttyTHS2
#   python3 tools/prereq-evidence/uart_loopback.py --no-loopback       # 对照组：不短接时跑
#
# 退出码：0 = 至少一个口回环成功；1 = 都没成功；2 = 参数错误
# =============================================================================
from __future__ import annotations

import argparse
import fcntl
import struct
import sys
import time

try:
    import serial
except ImportError:
    print("❌ 未安装 pyserial。修：python3 -m pip install --user pyserial")
    print("   ⚠️ 注意：pymavlink **不自带** pyserial（它不是 pymavlink 的依赖）。")
    sys.exit(1)

DEFAULT_PORTS = ["/dev/ttyTHS1", "/dev/ttyTHS2"]

# 已知模式：0x55 是 0101…，在任意波特率下都最容易看出"收到了但速率不对"
PATTERN = bytes([0x55, 0xAA] * 32)

# ---------------------------------------------------------------------------
# 内部回环（不需要任何跳线）
#
# 8250 系 UART 的 MCR bit4 = LOOP，把 TX **在芯片内部**接回 RX。
# Linux 通过 ioctl(TIOCMBIS, TIOCM_LOOP) 暴露。
# 数值取自本机头文件，**不凭记忆**：
#   TIOCM_LOOP = 0x8000  ← /usr/include/asm-generic/termios.h:46
#   TIOCMBIS   = 0x5416  ← /usr/include/asm-generic/ioctls.h:41
#   TIOCMBIC   = 0x5417  ← 同上 :42
#
# ⚠️ **它能证明什么、不能证明什么**（关键）：
#   内部回环在**芯片内部**成环，**完全绕过引脚**。所以
#     通过 ⟹ 控制器 + 驱动 + 时钟活着；**仍然完全不知道引脚有没有被 pinmux 接出去**
#     失败 ⟹ 控制器/驱动本身有问题，**与接线无关**
#   要判定引脚/pinmux，**只有物理短接那一条路**。
# ---------------------------------------------------------------------------
TIOCMBIS, TIOCMBIC, TIOCM_LOOP = 0x5416, 0x5417, 0x8000

# ⚠️ 实测已知行为（2026-09-21，z0 的 Jetson）—— **已查清，不是"未知行为"**：
#   发 1–7 字节  → 原样收回 ✅
#   发 8–15 字节 → **前 8 字节被抹成 0，其余正确**
#                  （发 01..0c 收回 0000000000000000 09 0a 0b 0c）
#   发 ≥16 字节  → **全部抹成 0**（发 0x55×64 收回 64 个 0x00）
#
# ★★ 2026-09-21 晚些时候更新：**上面这段是 ttyTHS1 修复前的读数，现已作废。**
#   在设备树里删掉 `serial@3100000` 的 `dmas`/`dma-names` 并重启后（内核日志
#   `RX in PIO mode` / `TX in PIO mode`），该口长模式 16 字节、物理回环 64 字节**均原样收回**。
#   ⚠️ 但 `ttyTHS2`（`serial@3140000`）**没做同样的改动**，它仍然带着 DMA 缺陷 ⟹
#   跑 `--internal` 时它仍会报长模式被打坏，**那是预期的，不是回归**。
#
#   机制：**接收通路的 DMA 缺陷**。该 SoC UART 的 DT 节点带
#   `dmas = <…>; dma-names = "rx tx";`（`compatible = nvidia,tegra194-hsuart`），
#   JetPack 6.x 上这条 DMA 通路会损坏数据。官方/社区修法：
#   **删掉该节点的 `dmas` / `dma-names`，强制退回 PIO**（内核日志会打印 "RX in PIO mode"）。
#
# ★★ 这不是回环自检的边角料 —— **它是能直接掐断飞控链路的真故障**：
#   MAVLink 帧最短 17 字节，在 ≥16 档会被**全部打成 0** ⟹ 一个心跳都解不出来。
#   症状极具误导性：物理回环"看起来成功"（字节数分毫不差），但内容全是 0。
#
#   ⟹ 所以本函数**故意去测它**，而不是像初版那样用 ≤7 字节绕开。
SHORT_PATTERN = bytes(range(1, 8))    # 7 字节：只验控制器 + 驱动 + 时钟
LONG_PATTERN = bytes(range(1, 17))    # 16 字节：进 DMA 档，验接收数据通路


def _roundtrip(s, pattern: bytes, wait: float = 1.5) -> bytes:
    """在已开好内部回环的端口上发 pattern 并读回。"""
    s.reset_input_buffer()
    s.write(pattern)
    s.flush()
    got = b""
    deadline = time.time() + wait
    while len(got) < len(pattern) and time.time() < deadline:
        got += s.read(len(pattern) - len(got))
    return got


def internal_loopback(port: str, baud: int) -> dict:
    """**不需要跳线**：用 UART 内部回环验控制器与接收数据通路。

    返回 {"controller": bool, "dma_ok": bool}。
      controller —— 控制器/驱动/时钟是否活着（**不代表引脚是通的**）
      dma_ok     —— 接收数据通路在 ≥8 字节时是否保真（**这一条才决定飞控链路能不能用**）
    """
    result = {"controller": False, "dma_ok": False}
    try:
        s = serial.Serial(port, baud, timeout=0.2)
    except Exception as exc:
        print(f"  ❌ 打不开 {port}：{type(exc).__name__}: {exc}")
        return result

    fd = s.fileno()
    try:
        try:
            fcntl.ioctl(fd, TIOCMBIS, struct.pack("I", TIOCM_LOOP))
        except OSError as exc:
            print(f"  ⚠️ {port}：ioctl(TIOCMBIS, TIOCM_LOOP) 被拒 —— "
                  f"{exc.strerror}（errno {exc.errno}）")
            print("     ⟹ 该驱动不支持内部回环，此法不适用 → 改做物理短接")
            return result

        time.sleep(0.15)

        # ---- ① 短模式：控制器活着吗 ----
        got = _roundtrip(s, SHORT_PATTERN)
        if got == SHORT_PATTERN:
            print(f"  ✅ {port}：短模式回环成功（{len(SHORT_PATTERN)} 字节原样收回）"
                  " ⟹ 控制器 + 驱动 + 时钟正常")
            result["controller"] = True
        else:
            print(f"  ❌ {port}：短模式{len(SHORT_PATTERN)} 字节回环"
                  f"{'内容不符：' + got.hex() if got else '**一个字节都没收回**'}")
            print("     ⟹ 控制器/驱动有问题（或该驱动只是接受了 ioctl 却没真接内部回环）")
            print("     ⚠️ 但内部回环**绕过引脚** —— 它**不能**说明引脚/pinmux 是通的。")
            return result

        # ---- ② 长模式：接收数据通路保真吗（>8 字节才触发 DMA）----
        got = _roundtrip(s, LONG_PATTERN)
        if got == LONG_PATTERN:
            print(f"  ✅ {port}：长模式回环成功（{len(LONG_PATTERN)} 字节原样收回）"
                  " ⟹ 接收数据通路保真")
            result["dma_ok"] = True
        else:
            zeroed = got.count(0)
            print(f"  ❌❌ {port}：**长模式回环数据被打坏** —— "
                  f"发 {len(LONG_PATTERN)} 字节，收回 {len(got)} 字节，其中 {zeroed} 个是 0x00")
            print(f"      发出: {LONG_PATTERN.hex(' ')}")
            print(f"      收回: {got.hex(' ') if got else '(空)'}")
            print("      ⟹ ★★ **接收通路的 DMA 缺陷**（JetPack 6.x / nvidia,tegra194-hsuart 已知问题）。")
            print(f"         MAVLink 帧最短 17 字节 ⟹ 在 ≥16 档被全部打成 0 ⟹ **一个心跳都解不出来**。")
            print("         修法：在设备树里**删掉该串口节点的 `dmas` / `dma-names`** 强制退回 PIO，")
            print("         然后重启。⚠️ 症状极具误导性：物理回环的**字节数分毫不差**，")
            print("         只看「收没收到」会误判成「线接对了」。")
        return result
    finally:
        try:
            fcntl.ioctl(fd, TIOCMBIC, struct.pack("I", TIOCM_LOOP))
        except Exception:
            pass
        try:
            s.close()
        except Exception:
            pass


def test_port(port: str, baud: int, expect_loopback: bool, wait: float) -> bool:
    """返回 True = 该口通过（与期望一致）。"""
    try:
        s = serial.Serial(port, baud, timeout=0.2)
    except Exception as exc:
        print(f"  ❌ 打不开 {port}：{type(exc).__name__}: {exc}")
        if "Permission" in str(exc) or "errno 13" in str(exc).lower():
            print("     → 权限。修：`sudo usermod -aG dialout $(id -un)` 后重新登录，"
                  "或临时 `sudo setfacl -m u:$(id -un):rw " + port + "`")
        return False

    try:
        # 冲掉可能残留的字节，避免把"上电噪声"当成回环成功
        s.reset_input_buffer()
        s.reset_output_buffer()
        time.sleep(0.2)
        s.reset_input_buffer()

        s.write(PATTERN)
        s.flush()

        got = b""
        deadline = time.time() + wait
        while len(got) < len(PATTERN) and time.time() < deadline:
            got += s.read(len(PATTERN) - len(got))

        if got == PATTERN:
            print(f"  ✅ {port} @ {baud}：**回环成功** —— 发 {len(PATTERN)} 字节，"
                  f"原样收回 {len(got)} 字节")
            print("     ⟹ Jetson 侧全通（引脚 + pinmux + 驱动 + 电平都正常）")
            print("     ⟹ ② 的问题**不在 Jetson**，去查飞控侧（上电？参数？接线交叉？）")
            return True

        if len(got) > 0:
            print(f"  ⚠️ {port} @ {baud}：收到 {len(got)} 字节但**内容不符**")
            print(f"     发出: {PATTERN[:16].hex()} …")
            print(f"     收回: {got[:16].hex()} …")
            print("     ⟹ 有信号但不成回环 —— 常见于：线还接在飞控上（那几字节被飞控吃掉了），"
                  "或波特率在某一侧不匹配。")
            return False

        print(f"  ✗ {port} @ {baud}：发 {len(PATTERN)} 字节，**一个都没收回**（{wait:.0f} s 内）")
        return not expect_loopback
    finally:
        try:
            s.close()
        except Exception:
            pass


def main() -> int:
    ap = argparse.ArgumentParser(
        description="UART 回环自检 —— 把「Jetson 侧坏了」从「飞控侧静默」里摘出来")
    ap.add_argument("--port", action="append",
                    help="要测的串口（可重复）；默认测 /dev/ttyTHS1 与 /dev/ttyTHS2")
    ap.add_argument("--baud", type=int, default=921600,
                    help="波特率（默认 921600 —— z0 的 SER_TEL2_BAUD）")
    ap.add_argument("--wait", type=float, default=2.0, help="每口等待秒数（默认 2）")
    ap.add_argument("--no-loopback", action="store_true",
                    help="对照组：**未短接**时跑。此时「收不到」才算通过")
    ap.add_argument("--internal", action="store_true",
                    help="**不需要跳线**：用 UART 内部回环（TIOCM_LOOP）验控制器。"
                         "⚠️ 它绕过引脚，**不能**判定 pinmux/接线")
    args = ap.parse_args()

    ports = args.port or DEFAULT_PORTS
    expect = not args.no_loopback

    if args.internal:
        print("=" * 72)
        print("UART **内部**回环自检 —— 不需要任何跳线")
        print("=" * 72)
        print()
        print(f"  波特率 {args.baud}　端口 {', '.join(ports)}")
        print()
        results = [internal_loopback(p, args.baud) for p in ports]
        print()
        print("=" * 72)
        dma_bad = [p for p, r in zip(ports, results) if r["controller"] and not r["dma_ok"]]
        if dma_bad:
            print("结论：★★ **接收通路的 DMA 缺陷**（" + ", ".join(dma_bad) + "）")
            print("      控制器活着，但 **≥8 字节的接收数据会被打成 0**。")
            print("      ⟹ MAVLink 帧最短 17 字节，会被**全部打成 0** ⟹ 一个心跳都解不出来。")
            print("      ⟹ **这很可能就是「连不上飞控」的真正原因**，与接线无关。")
            print("      修法：设备树里删掉该串口节点的 `dmas` / `dma-names` 强制退回 PIO，然后重启。")
            print("      ⚠️ 别再折腾波特率或接线 —— 物理回环的字节数会分毫不差，看「收没收到」会误判。")
            return 1
        if all(r["controller"] for r in results):
            print("结论：**控制器 + 接收数据通路都正常** ⟹ 芯片侧无罪。")
            print("      ⚠️ 但内部回环**绕过引脚**，所以**下一步必须物理短接**才能判定引脚：")
            print("        把你接飞控的那两针短接，再跑 `uart_loopback.py`（不加 --internal）。")
            return 0
        print("结论：至少一个口的内部回环没过 —— 先查该口的驱动/控制器，这一条与接线无关。")
        return 1

    print("=" * 72)
    if expect:
        print("UART 回环自检 —— 请确认：**已把你接飞控的那两针短接**（飞控侧两线已拔掉）")
    else:
        print("UART 回环自检（对照组：**未短接**）—— 预期「收不到」")
    print("=" * 72)
    print()
    print(f"  波特率 {args.baud}　等待 {args.wait} s　端口 {', '.join(ports)}")
    print()

    results = [test_port(p, args.baud, expect, args.wait) for p in ports]

    print()
    print("=" * 72)
    if all(results):
        if expect:
            print("结论：**回环通过** ⟹ Jetson 侧无罪。② 卡在飞控侧 ——")
            print("      去确认：① 飞控真的上电了？② 那个 UART 上真的配了 MAVLink（MAV_1_CONFIG）？")
            print("      ③ TX/RX 是否交叉（Jetson 的 TX 要接飞控的 RX）。")
        else:
            print("结论：未短接时收不到 ✓（对照组符合预期）—— 说明之前的『收不到』不是自激噪声。")
            print("      可放心去做真回环：短接后再跑一次（不加 --no-loopback）。")
        return 0
    print("结论：**回环未通过** ⟹ 问题在 Jetson 侧或针脚 —— 与飞控无关。")
    print("      依次怀疑：① 短接的是不是这两个口对应的针？② 排针 UART 功能没被 pinmux 打开？")
    print("      ③ 短接没接实。")
    return 1


if __name__ == "__main__":
    sys.exit(main())
