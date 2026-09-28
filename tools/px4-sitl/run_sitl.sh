#!/usr/bin/env bash
# 起 PX4 SITL（SIH 后端），可选一并起 micro-XRCE-DDS Agent —— ticket 01 的验收工具。
#
# ⚠️ 这不是 `run/z0.py` 那条**真机**链的替代品，也**不碰飞机**：
#    它跑的是仿真里的 z0 同构机（SIH，`SYS_AUTOSTART=10040`），只有 CPU。
#
# 验收那一条命令（两个终端分别起，或本脚本加 --agent）：
#     source /opt/ros/humble/setup.bash && ros2 topic list | grep fmu
#   → `/fmu/in/*` 与 `/fmu/out/*` **双向前缀都要有**。只有一半 = agent 通了但 PX4 侧没连上。
#
# 用法：
#   bash tools/px4-sitl/run_sitl.sh --agent        # 前台跑 SITL，agent 在后台
#   bash tools/px4-sitl/run_sitl.sh                # 只跑 SITL（agent 你自己起）
set -euo pipefail

SRC="${SRC:-$HOME/refs/px4-v1.14.3-build}"
BUILD_DIR="${BUILD_DIR:-$SRC/build/px4_sitl_default}"
MODEL="${MODEL:-sihsim_quadx}"          # SIH 的三个模型：quadx / airplane / xvert
AGENT_PORT="${AGENT_PORT:-8888}"
AGENT_BIN="${AGENT_BIN:-/usr/local/bin/MicroXRCEAgent}"
WITH_AGENT=0
AGENT_PID=""

for a in "$@"; do
  case "$a" in
    --agent) WITH_AGENT=1 ;;
    -h|--help) sed -n '2,20p' "$0"; exit 0 ;;
    *) echo "未知参数：$a" >&2; exit 1 ;;
  esac
done

[[ -x "$BUILD_DIR/bin/px4" ]] || { echo "找不到 $BUILD_DIR/bin/px4 —— 先跑 build_px4_sitl.sh" >&2; exit 1; }

cleanup() {
  [[ -n "$AGENT_PID" ]] && kill "$AGENT_PID" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

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
echo
cd "$BUILD_DIR"
PX4_SIM_MODEL="$MODEL" PX4_SIMULATOR=sihsim ./bin/px4
