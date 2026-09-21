#!/usr/bin/env bash
# =============================================================================
# z0 —— 设备树改动：让 serial@3100000（串口1 / ttyTHS1）退回 PIO，修掉接收 DMA 缺陷
#
# 对应 ticket：.scratch/phase1/issues/01-hardware-prereqs.md 的 ②（第八轮）
#
# 【为什么改这一处】
#   serial@3100000 的 DT 节点带 `dmas` / `dma-names = "rx tx"`，
#   JetPack 6.x 的 nvidia,tegra194-hsuart 接收通路会损坏数据：
#   **≥8 字节的接收内容被打成 0**（第一手实测：发 16 字节 01..10，收回 16 个 0x00）。
#   MAVLink 帧最短 17 字节 ⟹ 全部被打成 0 ⟹ 一个心跳都解不出来 = 「连不上飞控」。
#   删掉这两个属性 ⟹ 驱动退回 PIO ⟹ 缺陷不触发（内核日志会打印 "RX in PIO mode"）。
#
# 【官方留存方法】把改好的 dtb 用 `/boot/extlinux/extlinux.conf` 里的 `FDT` 行指定。
#   官方文档（Jetson Linux Developer Guide · Jetson Orin NX and Nano Series）原文：
#   UEFI 默认 extlinux 启动，找 extlinux.conf 里的 FDT 条目；其中指定的 image/dtb
#   **优先级高于**烧录在分区里的 dtb；没写或文件不存在才回落到分区。
#   ⟹ 持久（重启/断电都在）、不需要主机、不需要 recovery、不需要重刷。
#   ⚠️ 例外：安全启动开着时 FDT 行会被忽略。本机实测 SecureBoot efivar = 0（未启用）。
#
# 【子命令】
#   build     生成改好的 dtb + **自校验**（不需要 sudo，不碰 /boot）
#   install   装进 /boot/dtb/ 并往 extlinux.conf 加 FDT 行（需要 sudo，会备份）
#   rollback  还原 extlinux.conf 与 dtb（需要 sudo）
#   status    只读：现在的 FDT 行 + 内核里该节点还有没有 dmas
#
# ⚠️⚠️ 诚实交代：**本脚本截至 2026-09-21 只跑过 `build` 与 `status`。**
#   `install` / `rollback` 的路径逻辑是照官方文档写的，但**没有在真机上执行过**。
#   ⚠️ 它的 build 干跑已验证「拆→编→再拆」无损往返、且除目标节点外全文零差异。
# =============================================================================
set -euo pipefail

DTB_BASE="kernel_tegra234-p3768-0000+p3767-0001-nv"
SRC_DTB="/boot/dtb/${DTB_BASE}.dtb"
[ -f "$SRC_DTB" ] || SRC_DTB="/boot/kernel_${DTB_BASE}.dtb"
DST_DTB="/boot/dtb/${DTB_BASE}-pio.dtb"
EXTLINUX="/boot/extlinux/extlinux.conf"
LABEL="primary-pio"

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
WORK="$REPO/tools/prereq-evidence/dtb-work"
NODE="serial@3100000"

die() { echo "❌ $*" >&2; exit 1; }

# 只删指定节点里的 dmas / dma-names，别处一律不动
_patch_dts() {
    awk '
      /^\t\tserial@3100000 \{/ { ins = 1 }
      ins && /dmas = /          { next }
      ins && /dma-names = /     { next }
      ins && /^\t\t\};/         { ins = 0 }
      { print }
    ' "$1"
}

# 把 dts 里某个节点的原文抓出来
_node_text() {
    awk -v n="$2" '$0 ~ "^\t\t" n " \\{" {p=1} p{print} p && /^\t\t\};/{exit}' "$1"
}

cmd_build() {
    [ -f "$SRC_DTB" ] || die "找不到源 dtb：$SRC_DTB"
    command -v dtc >/dev/null || die "没装 dtc。修：sudo apt install device-tree-compiler"

    rm -rf "$WORK"; mkdir -p "$WORK"
    echo "源 dtb：$SRC_DTB"
    md5sum "$SRC_DTB"

    echo
    echo "--- 1) 拆开 ---"
    dtc -I dtb -O dts -o "$WORK/orig.dts" "$SRC_DTB" 2>/dev/null

    echo "--- 2) 删属性 ---"
    _patch_dts "$WORK/orig.dts" > "$WORK/patched.dts"

    echo "--- 3) 重编 ---"
    dtc -I dts -O dtb -o "$WORK/${DTB_BASE}-pio.dtb" "$WORK/patched.dts" 2>/dev/null

    echo "--- 4) 自校验 ---"
    dtc -I dtb -O dts -o "$WORK/roundtrip.dts" "$WORK/${DTB_BASE}-pio.dtb" 2>/dev/null

    local before after
    before="$(_node_text "$WORK/orig.dts" "$NODE")"
    after="$(_node_text "$WORK/roundtrip.dts" "$NODE")"
    echo "  改动前 $NODE："; echo "$before" | sed 's/^/    /'
    echo "  改动后 $NODE："; echo "$after"  | sed 's/^/    /'

    # 断言 1：两个属性真的没了
    echo "$after" | grep -q 'dmas'      && die "自校验失败：dmas 还在"
    echo "$after" | grep -q 'dma-names' && die "自校验失败：dma-names 还在"
    # 断言 2：节点仍是 okay（别把口弄没了）
    echo "$after" | grep -q 'status = "okay"' || die "自校验失败：status 不是 okay"
    # 断言 3：compatible 没被动过（否则换了驱动，行为完全不同）
    echo "$after" | grep -q 'nvidia,tegra194-hsuart' || die "自校验失败：compatible 变了"
    # 断言 4：除目标节点外，全文一字未动
    if diff -q \
        <(awk -v n="$NODE" '$0 ~ "^\t\t" n " \\{" {p=1} p && /^\t\t\};/{p=0;next} !p' "$WORK/roundtrip.dts") \
        <(awk -v n="$NODE" '$0 ~ "^\t\t" n " \\{" {p=1} p && /^\t\t\};/{p=0;next} !p' "$WORK/orig.dts") \
        >/dev/null; then
        echo "  ✅ 除 $NODE 外全文零差异"
    else
        die "自校验失败：目标节点之外也有改动"
    fi
    echo "  ✅ 体积：$(stat -c%s "$SRC_DTB") → $(stat -c%s "$WORK/${DTB_BASE}-pio.dtb") 字节"

    echo
    echo "✅ 生成完毕：$WORK/${DTB_BASE}-pio.dtb"
    echo "   下一步（需要人确认）：bash $0 install"
}

cmd_install() {
    [ -f "$WORK/${DTB_BASE}-pio.dtb" ] || die "还没有产物，先跑：bash $0 build"
    [ "$(id -u)" -eq 0 ] || die "需要 root：sudo bash $0 install"
    [ -f "$EXTLINUX" ] || die "找不到 $EXTLINUX"

    local ts; ts="$(date +%Y%m%d-%H%M%S)"
    cp -a "$EXTLINUX" "${EXTLINUX}.bak-${ts}"
    echo "✅ 已备份：${EXTLINUX}.bak-${ts}"

    install -m 644 "$WORK/${DTB_BASE}-pio.dtb" "$DST_DTB"
    echo "✅ 已装 dtb：$DST_DTB"

    if grep -q "^ *FDT " "$EXTLINUX"; then
        die "$EXTLINUX 里已经有 FDT 行，本脚本不覆盖 —— 请人工处理"
    fi
    if grep -q "^ *LABEL ${LABEL}\$" "$EXTLINUX"; then
        die "已存在 LABEL ${LABEL}，本脚本不覆盖 —— 请人工处理"
    fi

    # 新 LABEL 插在最前面；原有的 LABEL primary 一字不动，作为菜单里的回退项
    awk -v lbl="$LABEL" -v fdt="$DST_DTB" '
      NR == 1 {
        print "LABEL " lbl
        print "      MENU LABEL primary-pio (UART PIO, 删 dmas)"
        print "      LINUX /boot/Image"
        print "      FDT " fdt
        print "      INITRD /boot/initrd"
        print "      APPEND ${cbootargs}"
        print ""
      }
      /^DEFAULT / { print "DEFAULT " lbl; next }
      { print }
    ' "$EXTLINUX" > "${EXTLINUX}.new"
    mv "${EXTLINUX}.new" "$EXTLINUX"
    echo "✅ 已加 LABEL ${LABEL} 并把 DEFAULT 指过去（原 LABEL primary 保留在菜单里）"
    echo
    echo "回退方式（两种，任选）："
    echo "  A. 开机时在 extlinux 菜单（TIMEOUT 30 秒）里选 'primary kernel'"
    echo "  B. sudo cp ${EXTLINUX}.bak-${ts} ${EXTLINUX} && sudo rm -f ${DST_DTB}"
    echo
    echo "下一步：sudo reboot"
    echo "重启后第一件事（验修复）："
    echo "  python3 $REPO/tools/prereq-evidence/uart_loopback.py --internal --port /dev/ttyTHS1"
    echo "  期望：**长模式 16 字节原样收回**（修好前是 16 个 0x00）"
}

cmd_rollback() {
    [ "$(id -u)" -eq 0 ] || die "需要 root：sudo bash $0 rollback"
    local bak; bak="$(ls -1t "${EXTLINUX}".bak-* 2>/dev/null | head -1)"
    [ -n "$bak" ] || die "找不到备份文件 ${EXTLINUX}.bak-*"
    cp -a "$bak" "$EXTLINUX"
    rm -f "$DST_DTB"
    echo "✅ 已还原 $EXTLINUX（来自 $bak）并删除 $DST_DTB"
    echo "   重启即回到原状：sudo reboot"
}

cmd_status() {
    echo "=== extlinux.conf 的启动项 ==="
    grep -nE "^(DEFAULT|LABEL| *FDT| *LINUX)" "$EXTLINUX" 2>/dev/null || echo "  （读不到 $EXTLINUX）"
    echo
    echo "=== 内核此刻跑的 DT 里，$NODE 还有没有 dmas ==="
    local n="/proc/device-tree/bus@0/${NODE}"
    if [ -d "$n" ]; then
        for p in dmas dma-names status compatible; do
            if [ -e "$n/$p" ]; then
                case "$p" in
                    # dmas 是 4 字节 phandle 的二进制数组，转字符串只会出乱码 ⟹ 打 hex
                    dmas) printf "  %-12s = %s (hex)\n" "$p" \
                              "$(od -An -tx1 "$n/$p" | tr -s ' ' | tr -d '\n' | sed 's/^ //')" ;;
                    *)    printf "  %-12s = %s\n" "$p" \
                              "$(tr -d '\0' < "$n/$p" | tr '\n' ' ')" ;;
                esac
            else
                printf "  %-12s = （不存在）\n" "$p"
            fi
        done
        [ -e "$n/dmas" ] \
            && echo "  ⟹ **还在** ⇒ 改动没生效（重启了吗？FDT 行对吗？）" \
            || echo "  ⟹ **已消失** ⇒ 改动生效，应该在跑 PIO"
    else
        echo "  ⚠️ 找不到 $n"
    fi
    echo
    echo "=== 内核日志里的 PIO 迹象 ==="
    dmesg 2>/dev/null | grep -iE "3100000.serial|PIO mode" | tail -5 || echo "  （无相关行 / 需要 sudo 才能读 dmesg）"
}

case "${1:-}" in
    build)    cmd_build ;;
    install)  cmd_install ;;
    rollback) cmd_rollback ;;
    status)   cmd_status ;;
    *) sed -n '2,40p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'; exit 1 ;;
esac
