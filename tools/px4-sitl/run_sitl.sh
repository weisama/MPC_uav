#!/usr/bin/env bash
# 起 PX4 SITL（SIH 后端），可选一并起 micro-XRCE-DDS Agent —— ticket 01 的验收工具，
# ticket 08 起支持 z0 同构机与 ±30% 参数失配覆盖层。
#
# ⚠️ 这不是 `run/z0.py` 那条**真机**链的替代品，也**不碰飞机**：它跑的是仿真。
# ⚠️ **`--plant z0` 之外的一切机型（`sihsim_quadx` / `gz_x500` …）只作最早期冒烟，
#    不得用于 SITL 验收** —— 它们的动力学与 z0 差得远（x500 量级：1 kg / 力臂 0.15 m）。
#
# 用法：
#   bash tools/px4-sitl/run_sitl.sh --plant z0            # z0 同构机（参数一致对照组）
#   bash tools/px4-sitl/run_sitl.sh --plant z0 --mismatch mass+30   # 质量 +30% 失配
#   bash tools/px4-sitl/run_sitl.sh --plant z0 --reset-params       # 先清掉持久化参数
#   bash tools/px4-sitl/run_sitl.sh --list-variants                 # 列出全部变体
#   bash tools/px4-sitl/run_sitl.sh                       # 上游 10040（仅冒烟）
#   bash tools/px4-sitl/run_sitl.sh --agent               # 前台 SITL + 后台 Agent
#
# 切换机制与取值见 docs/control/sitl-z0-plant.md；参数真源是
# tools/px4-sitl/z0-plant/plant_spec.py（改参数改那里，不要手改产物）。
set -euo pipefail

SRC="${SRC:-$HOME/refs/px4-v1.14.3-build}"
BUILD_DIR="${BUILD_DIR:-$SRC/build/px4_sitl_default}"
MODEL="${MODEL:-sihsim_quadx}"          # 默认保持 ticket 01 的行为：上游 SIH 四旋翼
AGENT_PORT="${AGENT_PORT:-8888}"
AGENT_BIN="${AGENT_BIN:-/usr/local/bin/MicroXRCEAgent}"
WITH_AGENT=0
PLANT=""
MISMATCH=""
OVERLAY=""
RESET_PARAMS=0
HERE="$(cd "$(dirname "$0")" && pwd)"
OVERLAY_DIR=""
AGENT_PID=""

usage() { sed -n '2,20p' "$0"; }

while [[ $# -gt 0 ]]; do
  case "$1" in
    --agent) WITH_AGENT=1; shift ;;
    --plant) PLANT="${2:?--plant 要一个值}"; shift 2 ;;
    --mismatch) MISMATCH="${2:?--mismatch 要一个值，如 mass+30}"; shift 2 ;;
    --overlay) OVERLAY="${2:?--overlay 要一个值}"; shift 2 ;;
    --reset-params) RESET_PARAMS=1; shift ;;
    --list-variants) python3 "$HERE/z0-plant/plant_spec.py" --list; exit 0 ;;
    -h|--help) usage; exit 0 ;;
    *) echo "未知参数：$1" >&2; usage; exit 1 ;;
  esac
done

[[ -x "$BUILD_DIR/bin/px4" ]] || { echo "找不到 $BUILD_DIR/bin/px4 —— 先跑 build_px4_sitl.sh" >&2; exit 1; }

# ---- 选覆盖层 ---------------------------------------------------------------
if [[ -n "$MISMATCH" || -n "$PLANT" || -n "$OVERLAY" ]]; then
  # --mismatch 蕴含 --plant z0（失配是相对 z0 标称值说的）
  [[ -n "$PLANT" ]] || PLANT=z0
  if [[ -n "$OVERLAY" ]]; then
    OVERLAY_SRC="$OVERLAY"
  elif [[ -n "$MISMATCH" ]]; then
    OVERLAY_SRC="$HERE/z0-plant/params/${PLANT}-${MISMATCH}.param"
  else
    OVERLAY_SRC="$HERE/z0-plant/params/${PLANT}.param"
  fi
  [[ -f "$OVERLAY_SRC" ]] || {
    echo "找不到覆盖层 $OVERLAY_SRC；可用变体：" >&2
    python3 "$HERE/z0-plant/plant_spec.py" --list >&2
    exit 1
  }
  if [[ "$PLANT" != "z0" ]]; then
    echo "未知 --plant $PLANT（目前只定义 z0）" >&2; exit 1
  fi
  MODEL="sihsim_z0"
fi

# ---- 机型是否装好 -----------------------------------------------------------
if [[ "$MODEL" == "sihsim_z0" ]]; then
  [[ -f "$BUILD_DIR/etc/init.d-posix/airframes/10046_sihsim_z0" ]] || {
    echo "z0 同构机型还没装 —— 先跑：bash $HERE/install_z0_airframe.sh" >&2; exit 1
  }
else
  echo "⚠️  当前机型是 $MODEL（**不是** z0 同构机）。它只能用来确认 SITL 起得来，"
  echo "    **不得用于 SITL 验收** —— 要跑真验收：bash $0 --plant z0"
fi

cleanup() {
  [[ -n "$AGENT_PID" ]] && kill "$AGENT_PID" 2>/dev/null || true
  [[ -n "$OVERLAY_DIR" ]] && rm -rf "$OVERLAY_DIR" || true
}
trap cleanup EXIT INT TERM

# ---- 覆盖层落地：rcS 用 `. px4-rc.params` **按名字**在 PATH 里找 ------------------
# PATH 尾部才是 ${R}etc/init.d-posix（${R} 在 posix 下为空），所以前置一个目录即可遮蔽。
# 该目录里只放 px4-rc.params 一个文件，避免遮住 px4-rc.simulator / px4-rc.mavlink。
if [[ -n "${OVERLAY_SRC:-}" ]]; then
  OVERLAY_DIR="$(mktemp -d "${TMPDIR:-/tmp}/px4-z0-overlay.XXXXXX")"
  cp "$OVERLAY_SRC" "$OVERLAY_DIR/px4-rc.params"
  export PATH="$OVERLAY_DIR:$PATH"
  echo "==> 覆盖层：$(basename "$OVERLAY_SRC")（经 PATH 前置的 px4-rc.params 生效）"
fi

if [[ "$RESET_PARAMS" == 1 ]]; then
  rm -f "$BUILD_DIR/rootfs/parameters.bson" "$BUILD_DIR/rootfs/parameters_backup.bson"
  echo "==> 已清掉持久化参数（rootfs/parameters*.bson）—— 回到全默认起手"
fi

if [[ "$WITH_AGENT" == 1 ]]; then
  [[ -x "$AGENT_BIN" ]] || { echo "找不到 $AGENT_BIN —— 先跑 build_xrce_agent.sh" >&2; exit 1; }
  echo "==> MicroXRCEAgent udp4 -p $AGENT_PORT（日志 /tmp/xrce_agent.log）"
  "$AGENT_BIN" udp4 -p "$AGENT_PORT" > /tmp/xrce_agent.log 2>&1 &
  AGENT_PID=$!
  sleep 1
fi

echo "==> PX4 SITL：PX4_SIM_MODEL=$MODEL PX4_SIMULATOR=sihsim"
echo "    另一个终端跑：source /opt/ros/humble/setup.bash \\"
echo "                  && source \$HOME/px4_msgs_ws/install/setup.bash \\"
echo "                  && ros2 topic list | grep fmu"
echo "    自检参数：在 pxh 里敲 param show SIH_MASS（z0 应为 0.75，上游 10040 是 1.0）"
echo
cd "$BUILD_DIR"
PX4_SIM_MODEL="$MODEL" PX4_SIMULATOR=sihsim ./bin/px4
