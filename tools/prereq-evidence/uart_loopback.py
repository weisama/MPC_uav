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
    args = ap.parse_args()

    ports = args.port or DEFAULT_PORTS
    expect = not args.no_loopback

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
