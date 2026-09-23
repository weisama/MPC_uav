#!/usr/bin/env bash
# 清除后台节点的一行别名 —— 等价于 `run/z0.py down`（见 .scratch/phase1/issues/22-run-menu-script.md §八）。
# 与 `run/up.sh` 对称的另一头：按**名字**扫（DOWN_TARGETS 那张白名单，pgrep -f）mavros / vrpn_listener /
# mocap / traj / control / bringup，先列清单，问一句 y/N，再按 SIGINT →（超时）SIGTERM →（超时）SIGKILL 停。
#
# ⚠️ 两件必须先知道的事：
#   ① **它不是降落**：setpoint 一停，PX4 在 COM_OF_LOSS_T（1.0 s）后自己退出 OFFBOARD ⟹ 空中先按
#      flight-runbook.md **§1.2「想停任务，只有两条软路径」**处置（依据在 params-changes.md §H.7）。
#   ② **按名字扫**⟹ 别人会话 / 跑着的测试起的同类节点也会在清单里（现场只有一套飞机，代价是明说过的）。
set -euo pipefail

exec python3 "$(dirname "$(readlink -f "$0")")/z0.py" down "$@"
