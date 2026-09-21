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
#   build                   生成改好的 dtb + **自校验**（不需要 sudo，不碰 /boot）
#   install                 装进 /boot/dtb/ 并往 extlinux.conf 加 FDT 行（需要 sudo，会备份）
#   DRY_RUN=1 ... install   同一条路径的空跑：产物落 dtb-work/，**不碰 /boot、不要 root**
#   rollback                还原 extlinux.conf 与 dtb（需要 sudo）
#   status                  只读：现在的 FDT 行 + 内核里该节点还有没有 dmas
#
# 【install 的两个防炸机要点】（2026-09-21 施工时补，此前只有 build 被实跑过）
#   1. 新 LABEL 的 APPEND **照抄原 LABEL 原文**，不写 `${cbootargs}` ——
#      原行带着 root=PARTUUID=…，自己写会把 root= 弄丢 ⟹ 开不了机。
#   2. 新 LABEL **追加到文件末尾**，不做「插到最前面」—— 免得全局项
#      （DEFAULT / TIMEOUT）落到 LABEL 块之后，解析行为靠赌。
#   两者都有自校验断言看着（改动行数 ≤1、原 LABEL primary 必须还在）。
#
# ⚠️⚠️ 诚实交代（**2026-09-21 第九轮更新，此前那句"install 没在真机上跑过"已作废**）：
#   - `build`    ✅ 已实跑（含全部断言）
#   - `install`  ✅ **2026-09-21 已在 z0 上真装成功** —— `/boot/extlinux/extlinux.conf`
#                已改成 `DEFAULT primary-pio` + 末尾 `LABEL primary-pio`（含 `FDT` 行），
#                备份在 `extlinux.conf.bak-20260921-082331`。
#                ⚠️ **但"装上去"≠"验证过"** —— 改动是否生效（能否开机、`dmas` 是否真没了）
#                要等重启后 `status` + `uart_loopback.py --internal` 才算数。
#   - `rollback` ❌ **仍未在真机上执行过** —— 照官方文档写的，首次 rollback 请注意有无报错。
#   - 空跑/自校验：`DRY_RUN=1` + `EXTLINUX_OVERRIDE=<副本>` 可在副本上完整练一遍 install
#                （真跑过三条：正常路径 / 幂等挡板 / 缺 APPEND 拒绝）。
# =============================================================================
set -euo pipefail

DTB_BASE="kernel_tegra234-p3768-0000+p3767-0001-nv"
SRC_DTB="/boot/dtb/${DTB_BASE}.dtb"
[ -f "$SRC_DTB" ] || SRC_DTB="/boot/kernel_${DTB_BASE}.dtb"
DST_DTB="/boot/dtb/${DTB_BASE}-pio.dtb"
# EXTLINUX 允许被环境变量覆盖 —— **只为在副本上验证本脚本**（配合 DRY_RUN=1）。
# 正常施工不要设它。
EXTLINUX="${EXTLINUX_OVERRIDE:-/boot/extlinux/extlinux.conf}"
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
    # DRY_RUN=1 ⟹ 走完全相同的代码路径，但产出落在 $WORK，**一个字节都不碰 /boot**。
    # 用途：这条路径在真机上从没跑过，而它直接决定能不能开机 —— 先在副本上看清楚产物。
    local dry="${DRY_RUN:-0}"
    local ext="$EXTLINUX" dtb_out="$DST_DTB"

    [ -f "$WORK/${DTB_BASE}-pio.dtb" ] || die "还没有产物，先跑：bash $0 build"
    if [ "$dry" = "1" ]; then
        ext="$WORK/extlinux.conf.preview"
        dtb_out="$WORK/${DTB_BASE}-pio.preview.dtb"
        cp -a "$EXTLINUX" "$ext"
        cp -a "$WORK/${DTB_BASE}-pio.dtb" "$dtb_out"
        echo "⚠️  DRY_RUN=1：产物写到 $WORK，不碰 /boot，也不需要 root"
    else
        [ "$(id -u)" -eq 0 ] || die "需要 root：sudo bash $0 install"
    fi
    [ -f "$ext" ] || die "找不到 $ext"

    # ⚠️ **挡板必须在「备份 + 装 dtb」之前**。反过来的话，重复跑一次 install 会先
    #   把**已经被改过**的 extlinux.conf 备份成最新一份，然后才 die ⟹ 最新备份不再是原件
    #   ⟹ 之后 `rollback`（取最新备份）会把「改过的配置」当成原件还原回去，
    #   而它引用的 -pio.dtb 已被 rollback 删掉 ⟹ **回退路径静默失效**。
    if grep -q "^ *FDT " "$ext"; then
        die "$ext 里已经有 FDT 行，本脚本不覆盖 —— 请人工处理（先 rollback 再重装）"
    fi
    if grep -q "^ *LABEL ${LABEL}\$" "$ext"; then
        die "已存在 LABEL ${LABEL}，本脚本不覆盖 —— 请人工处理（先 rollback 再重装）"
    fi

    if [ "$dry" != "1" ]; then
        local ts; ts="$(date +%Y%m%d-%H%M%S)"
        cp -a "$EXTLINUX" "${EXTLINUX}.bak-${ts}"
        echo "✅ 已备份：${EXTLINUX}.bak-${ts}"
        install -m 644 "$WORK/${DTB_BASE}-pio.dtb" "$DST_DTB"
        echo "✅ 已装 dtb：$DST_DTB"
    fi

    # ★★ 必须照抄原 LABEL 的 APPEND 行原文，**不能自己写 `${cbootargs}`**。
    #   本机原 LABEL primary 的 APPEND 带着 `root=PARTUUID=1f0266a5-… rw rootwait
    #   rootfstype=ext4 …` 一长串参数。若新启动项只写 `${cbootargs}`，而 UEFI 的
    #   cbootargs 里不含 root=，内核**找不到根文件系统 ⟹ 开不了机**。
    #   照抄原文则与「已知能启动的那一份」逐字一致，唯一差别就是多一行 FDT。
    local append_line
    append_line="$(awk '/^[[:space:]]*APPEND[[:space:]]/{print; exit}' "$ext")"
    [ -n "$append_line" ] || die "读不到原 LABEL 的 APPEND 行 —— 拒绝生成新启动项（怕开不了机）"
    echo "✅ 复用原 APPEND 行：$append_line"

    # ★ 新 LABEL **追加到文件末尾**，而不是插在最前面。
    #   理由：syslinux/extlinux 里 `DEFAULT` / `TIMEOUT` 是全局项，惯例写在所有
    #   `LABEL` 块之前。插到开头会让全局项落在 LABEL 块之后，解析行为不必赌 ——
    #   追加到末尾则文件其余部分只改动 `DEFAULT` 那一行，最不容易出意外。
    awk -v lbl="$LABEL" -v fdt="$dtb_out" -v append="$append_line" '
      /^[[:space:]]*DEFAULT[[:space:]]/ { print "DEFAULT " lbl; next }
      { print }
      END {
        print ""
        print "# ★ 以下 LABEL 由 dtb_pio_fix.sh 追加：serial@3100000 删 dmas/dma-names 退回 PIO，"
        print "#   修掉 JetPack 6.x 的接收 DMA 缺陷（≥8 字节被打成 0）。"
        print "#   除 FDT 一行外，与原 LABEL primary 逐字一致。"
        print "#   回退：把上面的 DEFAULT 改回 primary，或删掉本块。"
        print "LABEL " lbl
        print "      MENU LABEL primary-pio (UART PIO, no dmas)"
        print "      LINUX /boot/Image"
        print "      FDT " fdt
        print "      INITRD /boot/initrd"
        print append
      }
    ' "$ext" > "${ext}.new"
    mv "${ext}.new" "$ext"

    # ---- 自校验：新文件必须与**改动前的原件**只差「DEFAULT 一行」+「末尾新增块」----
    # ⚠️ 基准必须是改动前的副本。真装时 $ext 就是 $EXTLINUX 本身，
    #    拿它当基准等于文件跟自己比 ⟹ 统计恒为 0/0，断言空转（初版踩过）。
    local baseline="$EXTLINUX"
    [ "$dry" = "1" ] || baseline="${EXTLINUX}.bak-${ts}"
    local n_added n_removed
    n_removed="$(diff "$baseline" "$ext" 2>/dev/null | grep -c '^<')" || true
    n_added="$(diff "$baseline" "$ext" 2>/dev/null | grep -c '^>')" || true
    echo "   diff 统计（基准 $(basename "$baseline")）：被删 $n_removed 行，新增 $n_added 行"
    # 被删的应当**只有** DEFAULT 那一行
    [ "$n_removed" -le 1 ] || die "自校验失败：改动波及了 $n_removed 行（应 ≤1，只有 DEFAULT）"
    # 新 LABEL 块必须齐活
    for k in "LABEL $LABEL" "FDT $dtb_out" "LINUX /boot/Image" "INITRD /boot/initrd"; do
        grep -qF "$k" "$ext" || die "自校验失败：新文件里找不到「$k」"
    done
    grep -qF "$append_line" "$ext" || die "自校验失败：APPEND 行没进去（开机会挂）"
    grep -qE "^[[:space:]]*DEFAULT[[:space:]]+$LABEL\$" "$ext" || die "自校验失败：DEFAULT 没指到 $LABEL"
    # 原 LABEL primary 必须完好（回退项）
    grep -qE "^LABEL primary\$" "$ext" || die "自校验失败：原 LABEL primary 不见了（回退路径没了）"
    echo "  ✅ 自校验通过"

    if [ "$dry" = "1" ]; then
        echo
        echo "=== 预览产物：$ext ==="
        cat "$ext"
        echo
        echo "=== 与现网文件的 diff ==="
        diff "$EXTLINUX" "$ext" || true
        echo
        echo "⚠️  DRY_RUN 结束，/boot 未被触碰。要真装：sudo bash $0 install"
        return 0
    fi

    echo "✅ 已加 LABEL ${LABEL} 并把 DEFAULT 指过去（原 LABEL primary 保留在菜单里）"
    echo
    echo "回退方式（两种，任选）："
    echo "  A. 开机时在 extlinux 菜单（TIMEOUT 30 秒）里选 'primary kernel'"
    echo "  B. sudo cp ${EXTLINUX}.bak-${ts} ${EXTLINUX} && sudo rm -f ${DST_DTB}"
    echo
    echo "下一步：sudo reboot"
    echo "重启后按顺序验（三条都要看）："
    echo "  a) bash $0 status                              # DT 里 dmas 是否真的没了"
    echo "  b) python3 $REPO/tools/prereq-evidence/uart_loopback.py --internal --port /dev/ttyTHS1"
    echo "     期望：**长模式 16 字节原样收回**（修好前是 16 个 0x00）"
    echo "  c) python3 $REPO/tools/prereq-evidence/uart_loopback.py --port /dev/ttyTHS1   # 需短接在位"
    echo "     期望：**64 字节原样收回**（修好前是 64 个 0x00）"
}

cmd_rollback() {
    [ "$(id -u)" -eq 0 ] || die "需要 root：sudo bash $0 rollback"
    # ⚠️ 末尾的 `|| true` 不能省：`set -euo pipefail` 下，没有备份文件时 `ls` 返回 2，
    #   pipefail 把管道置 2 ⟹ 赋值语句失败 ⟹ set -e 直接终止脚本，
    #   **下面那句友好报错永远打不出来**，应急回退会静默非零退出。
    local bak; bak="$(ls -1t "${EXTLINUX}".bak-* 2>/dev/null | head -1)" || true
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
    # 只打注释头：从第 2 行打到 `set -euo pipefail` 之前为止。
    # ⚠️ 不要写死行号（`sed -n '2,40p'`）—— 头部注释一变长就会把代码行也打出来（踩过）。
    *) awk 'NR > 1 && /^set -euo pipefail$/ { exit } NR > 1 { print }' "${BASH_SOURCE[0]}" \
           | sed 's/^# \{0,1\}//'; exit 1 ;;
esac
