#!/usr/bin/env bash
# 把 docs/control/*-spec.md 发布为 GitHub Issue。
#
#   ./scripts/publish-spec.sh              # 发布 phase1-spec
#   ./scripts/publish-spec.sh phase2       # 发布 phase2-spec（Phase 2 完成后才有）
#
# 前置：gh auth login --hostname github.com --git-protocol ssh --web
# （交互式设备码流程，必须由人在自己的终端完成，脚本代劳不了。）
set -euo pipefail

REPO=weisama/MPC_uav
PHASE="${1:-phase1}"
SPEC="docs/control/${PHASE}-spec.md"

case "$PHASE" in
  phase1) TITLE="[Phase 1] offboard 位置定点飞行" ;;
  phase2) TITLE="[Phase 2] MPC（模型预测控制）接管 PID" ;;
  *)      TITLE="[${PHASE}] spec" ;;
esac

command -v gh >/dev/null || { echo "gh 未安装。见 https://github.com/cli/cli/releases"; exit 1; }
gh auth status >/dev/null 2>&1 || { echo "未登录 GitHub。先跑：gh auth login --hostname github.com --git-protocol ssh --web"; exit 1; }
[ -f "$SPEC" ] || { echo "找不到 $SPEC"; exit 1; }

# label 不存在时 gh issue create --label 会直接报错，所以先补上（幂等）。
gh label create ready-for-agent --repo "$REPO" \
  --description "Fully specified, ready for an AFK agent" --color 0E8A16 2>/dev/null || true

gh issue create --repo "$REPO" \
  --title "$TITLE" \
  --body-file "$SPEC" \
  --label ready-for-agent
