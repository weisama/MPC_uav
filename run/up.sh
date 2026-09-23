#!/usr/bin/env bash
# 一键起链的一行别名 —— 等价于 `run/z0.py up`（见 .scratch/phase1/issues/22-run-menu-script.md §七）。
# 起 mavros（带 GCS 桥）→ vrpn_listener → mocap → traj + control，逐段等判据成立再加下一步，
# 起完打印核对表并前台守着（Ctrl-C 一起停）。
#
# ★ 它**不代做**"还差的那两步"：选轨迹（fly.py 敲 1/2/3）与起飞（遥控器开始键）—— 留给人。
set -euo pipefail

exec python3 "$(dirname "$(readlink -f "$0")")/z0.py" up "$@"
