#!/usr/bin/env bash
# =============================================================================
# z0 前置条件取证脚本
#
# 对应 ticket：.scratch/phase1/issues/01-hardware-prereqs.md
# 对应 spec  ：.scratch/phase1/spec.md §「阻塞项：4 件必须由人物理完成的事」
#
# 它做的事：把「四件必须由人物理完成的事」里**能自动采集的部分**一次采完，
#           输出一段可直接粘贴进取证文档的 markdown。
#
# ⚠️ 三条纪律：
#   1. **脚本不自动修改任何文档** —— 取证文档（refs-manifest / params-changes）是
#      第一手证据的归宿，必须由人确认后手工落笔，避免脚本把噪声写进取证链。
#   2. **采不到就写「未采集」，绝不编造、绝不用默认值填充。**
#   3. **只读** —— 不写飞控、不解锁、不发起任何会改变飞行状态的指令。
#
# 用法：
#   bash tools/prereq-evidence/capture.sh                  # 打印 markdown 到 stdout
#   bash tools/prereq-evidence/capture.sh --out FILE       # 同时写入 FILE
#   bash tools/prereq-evidence/capture.sh --append-evidence # 追加到 .scratch/phase1/evidence/
#   bash tools/prereq-evidence/capture.sh --scan-serial     # 额外扫描哪个 THS 口有 MAVLink
#   bash tools/prereq-evidence/capture.sh --unarmed         # 声明采集时未解锁（④ 的判定前提）
#   bash tools/prereq-evidence/capture.sh --armed           # 声明采集时已解锁（④ 将作废）
#
# 退出码：0 = 采集完成（不代表四项都过）；1 = 写盘失败（--out / --append-evidence）；2 = 参数错误
# =============================================================================
set -uo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"

# ---- 动捕链路参数（取自 docs/reference/mocap-setup.md）-----------------------
# server: 10.1.1.198   VRPN 端口 3883（mocap-setup.md §5 / systemd 单元里的 nc -z）
MOCAP_HOST="${MOCAP_HOST:-10.1.1.198}"
MOCAP_PORT="${MOCAP_PORT:-3883}"
PING_COUNT="${PING_COUNT:-5}"

OUT_FILE=""
APPEND_EVIDENCE=0
SCAN_SERIAL=0
ASSUME_UNARMED="（未声明）"
# 由 §②b 的扫描填入；§③④ 直接复用，避免"② 刚说是 THS2，③④ 却去开 THS1"
SCANNED_PORT=""
SCANNED_BAUD=""

while [ $# -gt 0 ]; do
  case "$1" in
    --out)
      if [ $# -lt 2 ] || [ -z "${2:-}" ]; then
        echo "❌ --out 需要一个文件名" >&2; exit 2
      fi
      OUT_FILE="$2"; shift 2 ;;
    --append-evidence)  APPEND_EVIDENCE=1; shift ;;
    --scan-serial)      SCAN_SERIAL=1; shift ;;
    --unarmed)          ASSUME_UNARMED="是（采集时未解锁）"; shift ;;
    --armed)            ASSUME_UNARMED="否（采集时已解锁 —— 电流读数不可用于 A4 判定）"; shift ;;
    -h|--help)          sed -n '2,30p' "$0"; exit 0 ;;
    *) echo "未知参数：$1" >&2; exit 2 ;;
  esac
done

# ---- 找不到就写「未采集」，不要静默留空 --------------------------------------
have() { command -v "$1" >/dev/null 2>&1; }
NA="未采集（本机无此命令）"

# =============================================================================
# 渲染
# =============================================================================
render() {
  local now iface ipaddr gw route_line cur_ssid
  now="$(date '+%Y-%m-%d %H:%M:%S %Z')"

  echo "# z0 前置条件取证 —— 自动采集"
  echo
  echo "> 采集时间：\`${now}\`　主机：\`$(hostname)\`　内核：\`$(uname -r)\`　架构：\`$(uname -m)\`"
  echo "> 采集脚本：\`tools/prereq-evidence/capture.sh\`（只读；不自动修改任何文档）"
  echo "> ⚠️ 本文是**自动采集的原始读数**。要落进 \`docs/reference/\` 的取证文档，必须由人复核后手工落笔。"
  echo
  echo "---"
  echo

  # ---------------------------------------------------------------------------
  # 前置体检：这两件事不做，③④ 一定采不到。放在最前面，免得人接上飞控才发现。
  echo "## 开跑前先做这两件事"
  echo
  local in_dialout pymav_ok have_mav have_ser
  if id -nG 2>/dev/null | tr ' ' '\n' | grep -qx dialout; then
    in_dialout="✅ 已在 \`dialout\` 组"
  else
    in_dialout="❌ **不在 \`dialout\` 组** —— 打不开 \`/dev/ttyTHS*\`（权限 \`root:dialout 660\`）。
   修：\`sudo usermod -aG dialout $(id -un)\` 然后**必须重新登录**（仅加入组不重登不生效）"
  fi
  echo "1. **串口权限**：${in_dialout}"
  echo
  # ⚠️ 必须**两个都查**：pymavlink 不自带 pyserial。
  # 2026-09-21 在 z0 的 Jetson 上第一手核实：pymavlink 2.4.49 的 Requires-Dist
  # 只有 `lxml` / `fastcrc`。只查 pymavlink 会**误报「已装」**，然后在 ③④ 阶段
  # 炸 `ModuleNotFoundError: No module named 'serial'` —— 正是"人到现场才发现"的坑。
  python3 -c 'import pymavlink' 2>/dev/null && have_mav=1 || have_mav=0
  python3 -c 'import serial'    2>/dev/null && have_ser=1 || have_ser=0
  if [ "$have_mav" = 1 ] && [ "$have_ser" = 1 ]; then
    pymav_ok="✅ 已装（\`pymavlink\` + \`pyserial\`）"
  elif [ "$have_mav" = 1 ]; then
    pymav_ok="❌ **缺 \`pyserial\`** —— \`pymavlink\` 已装，但**它不自带 pyserial**，
   开串口时会炸 \`ModuleNotFoundError: No module named 'serial'\`。
   修：\`python3 -m pip install --user pyserial\`"
  elif [ "$have_ser" = 1 ]; then
    pymav_ok="❌ **缺 \`pymavlink\`**。修：\`python3 -m pip install --user pymavlink\`"
  else
    pymav_ok="❌ **两个都没装** —— ③④ 读不到。修：\`python3 -m pip install --user pymavlink pyserial\`"
  fi
  echo "2. **MAVLink 工具**：${pymav_ok}"
  echo
  echo "> 另需：**飞控已上电**，且串口线接在 Jetson 的 UART 上（本机没有 USB 串口设备，"
  echo "> 所以飞控不会以 \`/dev/ttyACM*\` 形式出现）。"
  echo
  echo "---"
  echo

  # ---------------------------------------------------------------------------
  echo "## ① 网络：Jetson 是否已接入动捕网段"
  echo
  echo "**判据（ticket 原文）**：Jetson 在动捕网段内，\`ping ${MOCAP_HOST}\` **0% 丢包**（不再走校园网关）。"
  echo
  echo '```'
  echo "---- 本机接口 ----"
  ip -brief addr 2>&1 | sed 's/^/  /'
  echo
  echo "---- 去 ${MOCAP_HOST} 的路由（关键：看 dev 与 src）----"
  ip route get "${MOCAP_HOST}" 2>&1 | sed 's/^/  /'
  echo
  echo "---- 默认网关 ----"
  ip route 2>&1 | grep -E '^default' | sed 's/^/  /' || echo "  （无默认路由）"
  echo
  echo "---- WiFi 连接与 AP 扫描 ----"
  if have nmcli; then
    nmcli -t -f DEVICE,TYPE,STATE,CONNECTION dev 2>&1 | grep -E 'wifi|ethernet' | sed 's/^/  /'
    echo
    echo "  [在范围内的 AP —— ★ 动捕那个 WiFi 叫什么，需要人到现场指认]"
    # stderr 必须分开收：混进来会被当成一行扫描数据，进而被渲染成"候选 SSID"
    scan_err="$(mktemp)"
    scan_out="$(timeout 40 nmcli -f SSID,SIGNAL,CHAN,SECURITY dev wifi list --rescan yes 2>"${scan_err}")"
    if [ -z "${scan_out}" ]; then
      echo "  ⚠️ **扫描未产出数据（未采集）** —— 不要把空当成「没有候选网络」"
      echo "     nmcli 报：$(head -3 "${scan_err}" | tr '\n' ' ')"
    else
      printf '%s\n' "${scan_out}" | sed 's/^/  /'
      echo
      echo "  [机械过滤：已剔除校园 SSID（SUSTech-* / eduroam）与隐藏名 —— 动捕网大概率在这几个里]"
      printf '%s\n' "${scan_out}" \
        | awk 'NR>1 && $1!="" {print $1}' \
        | grep -vxE 'SUSTech-.*|eduroam|--' | sort -u | sed 's/^/    → /'
      echo "  ↑ 若这里为空，说明扫描到了但是**全被过滤掉了**（即只有校园网/隐藏 AP），不是没扫到。"
    fi
    rm -f "${scan_err}"
  else
    echo "  （无 nmcli）"
  fi
  echo
  echo "---- ping ${MOCAP_HOST} ----"
  timeout $((PING_COUNT * 2 + 8)) ping -c "${PING_COUNT}" -W 1 "${MOCAP_HOST}" 2>&1 | sed 's/^/  /'
  echo
  echo "---- VRPN 端口 ${MOCAP_PORT} TCP 可达性（真正的业务端口）----"
  if have nc; then
    if timeout 5 nc -z -w 3 "${MOCAP_HOST}" "${MOCAP_PORT}" 2>&1; then
      echo "  ✅ TCP ${MOCAP_PORT} 可连"
    else
      echo "  ❌ TCP ${MOCAP_PORT} 不可连（注意：VRPN 实际走 UDP，此检查不通不必然等于 VRPN 不通）"
    fi
  else
    echo "  $NA"
  fi
  echo '```'
  echo
  echo "**第一手结论**："
  echo
  # SSID 可以含 ':'（nmcli -t 会转义成 '\:'）。AWK -F: 取 $2 会把含冒号的 SSID 截断 ——
  # 而 SSID 正是 ① 唯一缺的那个字段，截断了等于白采。改为保留首字段之后的**全部**再还原转义。
  cur_ssid=""
  if have nmcli; then
    cur_ssid="$(nmcli -t -f ACTIVE,SSID dev wifi 2>/dev/null \
      | grep '^yes:' | cut -d: -f2- | sed 's/\\:/:/g')"
  fi
  echo "- 当前 WiFi：${cur_ssid:-$NA}"
  echo "- 当前有地址且 UP 的接口：\`$(ip -brief addr show scope global up 2>/dev/null | awk '$3!="" && $1!="docker0" && $1!="l4tbr0"{print $1" "$3}' | tr '\n' ' ')\`"
  echo "- 去 ${MOCAP_HOST} 的出口：\`$(ip route get "${MOCAP_HOST}" 2>/dev/null | head -1)\`"
  echo "- → **判定**：$(if timeout 8 ping -c 2 -W 1 "${MOCAP_HOST}" >/dev/null 2>&1; then echo '✅ 通（0% 丢包）'; else echo '❌ **不通** —— ① 未完成'; fi)"
  echo
  echo "> 落笔位置：[\`docs/reference/mocap-setup.md\`](../../docs/reference/mocap-setup.md) 的「动捕网段」一节，以及 ticket 01 的勾选项。"
  echo
  echo "---"
  echo

  # ---------------------------------------------------------------------------
  echo "## ② 飞控接在哪个串口设备节点"
  echo
  echo "**判据（ticket 原文）**：记录飞控实际使用的串口设备节点，并确认它可用。"
  echo
  echo '```'
  echo "---- /dev/tty* 里的串口设备 ----"
  ls -l /dev/ttyTHS* /dev/ttyACM* /dev/ttyUSB* 2>&1 | sed 's/^/  /'
  echo
  echo "---- 稳定别名 /dev/serial/by-id/（若飞控走 USB 才会有）----"
  ls -l /dev/serial/by-id/ 2>&1 | sed 's/^/  /'
  echo
  echo "---- lsusb（飞控走 USB 的话会在这里出现）----"
  if have lsusb; then lsusb 2>&1 | sed 's/^/  /'; else echo "  $NA"; fi
  echo
  echo "---- 内核认到的 UART（SoC 侧，设备名与 MMIO 的对应关系）----"
  if have journalctl; then
    # 必须限量：这台 Jetson 已开机数周，全量 -k 会解出整本内核日志，只为了显示 10 行
    journalctl -k -n 2000 --no-pager 2>/dev/null | grep -iE 'ttyTHS|ttyACM|ttyUSB' | tail -10 | sed 's/^/  /'
  else
    echo "  （无 journalctl）"
  fi
  echo
  echo "---- 当前用户与串口组权限 ----"
  id 2>&1 | sed 's/^/  /'
  echo "  dialout 组成员：$(getent group dialout 2>/dev/null || echo '(无 dialout 组)')"
  echo '```'
  echo
  echo "**第一手结论**："
  echo
  # 注意：ls 会因为部分 glob 不存在而返回非零，配合 pipefail 会误触发兜底，
  # 所以先取进变量、再用参数展开兜底，不走 ||。
  serial_nodes="$(ls /dev/ttyTHS* /dev/ttyACM* /dev/ttyUSB* 2>/dev/null | tr '\n' ' ' | sed 's/ *$//')"
  echo "- 可用串口节点：${serial_nodes:-$NA}"
  echo "- 飞控是否走 USB：$(if have lsusb && lsusb 2>/dev/null | grep -qiE 'px4|3d robotics|ardupilot|ftdi|silicon labs|cp210|acm'; then echo '是（lsusb 里有疑似飞控设备）'; else echo '**否** —— `lsusb` 里没有飞控，所以 `/dev/ttyACM*`、`/dev/ttyUSB*` 本来就不会存在；飞控只能接在 SoC UART（`ttyTHS1` / `ttyTHS2`）上'; fi)"
  echo "- 用户是否在 \`dialout\` 组：$(if id -nG 2>/dev/null | tr ' ' '\n' | grep -qx dialout; then echo '✅ 在'; else echo '❌ **不在** —— `/dev/ttyTHS*` 权限是 `root:dialout 660`，**当前用户打不开串口**。需 `sudo usermod -aG dialout '"$(id -un)"'` 后**重新登录**（仅加入组不重登不生效）'; fi)"
  echo
  echo "> ★ **「是 THS1 还是 THS2」这一问，\`--scan-serial\` 能自动答**：它会对两个口依次发 HEARTBEAT 并听回包，"
  echo "> 哪个口收到飞控的心跳，哪个就是。见下方 §②b。"
  echo
  echo "> 落笔位置：ticket 01 勾选项；**并且 \`dialout\` 那条要同步进 ticket 05**（\`control\` 骨架要开串口）。"
  echo
  echo "---"
  echo

  # ---------------------------------------------------------------------------
  if [ "${SCAN_SERIAL}" -eq 1 ]; then
    echo "## ②b 串口扫描 —— 哪个口上挂着飞控"
    echo
    echo '```'
    probe_py="${REPO_ROOT}/tools/prereq-evidence/mavlink_probe.py"
    if [ -f "${probe_py}" ]; then
      scan_raw="$(python3 "${probe_py}" --scan 2>&1)"
      printf '%s\n' "${scan_raw}" | sed 's/^/  /'
      # 吃掉机器可读结果行，喂给 §③④ —— 免得"② 刚说是 THS2，③④ 却去开 THS1"
      detected="$(printf '%s\n' "${scan_raw}" | sed -n 's/^RESULT: //p' | head -1)"
      if [ -n "${detected}" ] && [ "${detected}" != "none" ]; then
        SCANNED_PORT="${detected%% *}"
        SCANNED_BAUD="${detected##* }"
      fi
    else
      echo "  找不到 mavlink_probe.py"
    fi
    echo '```'
    echo
    echo "---"
    echo
  fi

  # ---------------------------------------------------------------------------
  echo "## ③④ 飞控版本 / 电池电流（需要飞控上电 + 串口可达）"
  echo
  echo "**判据**：③ = QGC 读到的版本号**原文**；④ = **未解锁时**的电流读数（应 ≈0）。"
  echo
  echo '```'
  PORT="${FC_PORT:-}"
  BAUD_ARG=""
  PORT_SRC=""
  if [ -n "${PORT}" ]; then
    PORT_SRC="（来自 FC_PORT 环境变量）"
  elif [ -n "${SCANNED_PORT}" ]; then
    # ★ 优先用 §②b 扫出来的那个口，而不是"第一个存在的 /dev/ttyTHS*"
    PORT="${SCANNED_PORT}"
    BAUD_ARG="${SCANNED_BAUD}"
    PORT_SRC="（**来自 §②b 的扫描结果**，波特率 ${SCANNED_BAUD}）"
  else
    for cand in /dev/ttyTHS1 /dev/ttyTHS2; do
      [ -e "${cand}" ] && PORT="${cand}" && break
    done
    PORT_SRC="（**盲猜**：第一个存在的 /dev/ttyTHS* —— 未做扫描，不保证是飞控那个口。建议加 --scan-serial）"
  fi
  if [ -z "${PORT}" ]; then
    echo "  未发现任何候选串口，跳过。"
  else
    echo "  使用端口：${PORT} ${PORT_SRC}"
    echo "  （可用 FC_PORT=/dev/ttyTHSx 覆盖）"
    echo
    if [ -n "${BAUD_ARG}" ]; then
      python3 "${REPO_ROOT}/tools/prereq-evidence/mavlink_probe.py" \
        --port "${PORT}" --baud "${BAUD_ARG}" 2>&1 | sed 's/^/  /'
    else
      python3 "${REPO_ROOT}/tools/prereq-evidence/mavlink_probe.py" \
        --port "${PORT}" 2>&1 | sed 's/^/  /'
    fi
  fi
  echo '```'
  echo
  echo "**第一手结论**："
  echo
  echo "- 采集时是否未解锁：**${ASSUME_UNARMED}**"
  echo "  （A4 的判定前提就是「未解锁时电流应 ≈0」。**若当时已解锁，这一栏作废、必须重采。**）"
  echo
  echo "> ⚠️ **④ 的判定规则（ticket 原文）**：未解锁时应 ≈0；**恒为 0 或恒为怪值 = 中招**。"
  echo "> 中招的后果是电流/电量保护全是空转，且**参数无法绕过**（编译期宏），只能重编重刷固件。"
  echo "> 修复途径 = fork 自带的 \`pr_fix_hkust_nxt_dual\` @ \`8161c4cd\`（相对 \`main\` 只差一行 \`CH(5)\`→\`CH(8)\`）。"
  echo
  echo "> 落笔位置：③ → [\`docs/reference/refs-manifest.md\`](../../docs/reference/refs-manifest.md) §2.5；"
  echo "> ④ → [\`docs/reference/params-changes.md\`](../../docs/reference/params-changes.md) A4。"
  echo
  echo "---"
  echo
  echo "## 落笔对照表"
  echo
  echo "| 采集项 | 归宿 | 说明 |"
  echo "|---|---|---|"
  echo "| ① 网络 | \`docs/reference/mocap-setup.md\` | 顺便补上「动捕那个 WiFi 的 SSID 是什么」——文档至今没写 |"
  echo "| ② 串口 | ticket 01 勾选项 + ticket 05 | 含 \`dialout\` 组那条 |"
  echo "| ③ 版本 | \`docs/reference/refs-manifest.md\` §2.5 | 版本号的**原文**，不要转述 |"
  echo "| ④ 电流 | \`docs/reference/params-changes.md\` A4 | 必须带「是否未解锁」这个前提 |"
  echo
  echo "> **本脚本不代写上述文档。** 自动采集的读数只是原料；取证文档要求的是人复核过的、"
  echo "> 带上下文的第一手结论。"
}

# =============================================================================
# 输出
# =============================================================================
REPORT="$(render)"

# 写盘失败必须体现在退出码上 —— 否则调用方（人或 CI）只看退出码，
# 会以为原始取证已经归档，其实什么都没写。
rc=0

if [ -n "${OUT_FILE}" ]; then
  if ! printf '%s\n' "${REPORT}" > "${OUT_FILE}"; then
    echo "❌ 写入 --out 文件失败：${OUT_FILE}（文件未落盘）" >&2
    rc=1
  fi
fi

if [ "${APPEND_EVIDENCE}" -eq 1 ]; then
  EV_DIR="${REPO_ROOT}/.scratch/phase1/evidence"
  if ! mkdir -p "${EV_DIR}"; then
    echo "❌ 无法创建取证目录：${EV_DIR}" >&2
    rc=1
  else
    EV_FILE="${EV_DIR}/$(date '+%Y-%m-%d-%H%M%S')-capture.md"
    if printf '%s\n' "${REPORT}" > "${EV_FILE}"; then
      echo "已追加原始取证：${EV_FILE}" >&2
    else
      echo "❌ 写入取证文件失败：${EV_FILE}" >&2
      rc=1
    fi
  fi
fi

printf '%s\n' "${REPORT}"
exit "${rc}"
