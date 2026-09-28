#!/usr/bin/env bash
# micro-XRCE-DDS Agent（`MicroXRCEAgent`）的构建脚本 —— ticket 01 的可复现形式。
#
# 它是 PX4 与 ROS 2 之间**唯一**的桥：PX4 侧的 `uxrce_dds_client`（编在 SITL 二进制里）
# 主动连到本机的 UDP 8888，agent 再把这些话题以 DDS 暴露给 ROS 2（`/fmu/in/*`、`/fmu/out/*`）。
# 没有它，`ros2 topic list | grep fmu` 一条都不会有 —— 而**不会有任何报错**。
#
# ★ 版本为什么是 v2.4.2：PX4 v1.14（2023-11）同期配对的就是 Agent 2.4.x
#   （v3.x 是配 PX4 v1.15+ 的）。docs 里没钉版本，这是本机取的默认（见交付文档 §5）。
#
# ★★ 本机实测的坑：superbuild 会去 checkout **分支 `2.12.x`**，而 eProsima 上游
#    **已经把这个分支删了**（第一手：`git ls-remote --heads` 里没有 2.12.x，
#    只剩 tag `v2.12.0/1/2`）⟹ 报错是 `fatal: invalid reference: 2.12.x` /
#    `Failed to checkout tag: '2.12.x'`，**这个报错不指向"上游删了分支"这个根因**。
#    本脚本把那一行改钉到 **tag `v2.12.2`**（2.12 系列最后一个）。
#
# 用法：
#   bash tools/px4-sitl/build_xrce_agent.sh              # 编 + 装到 /usr/local
#   INSTALL_PREFIX=$HOME/.local bash tools/px4-sitl/build_xrce_agent.sh
set -euo pipefail

SRC="${SRC:-$HOME/refs/micro-xrce-dds-agent}"
BUILD_DIR="${BUILD_DIR:-$SRC/build}"
INSTALL_PREFIX="${INSTALL_PREFIX:-/usr/local}"
JOBS="${JOBS:-$(nproc)}"

PINNED_TAG="v2.4.2"
PINNED_COMMIT="57d086216d01ec43121845d385894a25987f8a2c"
# 上游已删除的分支 → 改钉的 tag（理由见文件头）
DEAD_BRANCH="2.12.x"
REPLACEMENT_TAG="v2.12.2"

if [[ ! -d "$SRC/.git" ]]; then
  echo "==> clone $PINNED_TAG 到 $SRC"
  git clone --recursive --branch "$PINNED_TAG" https://github.com/eProsima/Micro-XRCE-DDS-Agent.git "$SRC"
fi
actual="$(git -C "$SRC" rev-parse HEAD)"
if [[ "$actual" != "$PINNED_COMMIT" ]]; then
  echo "⚠️  $SRC 现在在 $actual，不是钉住的 $PINNED_COMMIT" >&2
fi

# ── 打补丁：把已消失的分支换成仍存在的 tag ────────────────────────────────
if grep -q "set(_fastdds_tag ${DEAD_BRANCH})" "$SRC/CMakeLists.txt"; then
  echo "==> 改钉 fastdds: $DEAD_BRANCH → $REPLACEMENT_TAG（上游删了那个分支）"
  sed -i "s|^\( *set(_fastdds_tag \)${DEAD_BRANCH})$|\1${REPLACEMENT_TAG})|" "$SRC/CMakeLists.txt"
fi
grep -n "set(_fastdds_tag" "$SRC/CMakeLists.txt"

# ── 构建（superbuild 会自动拉 Fast-CDR / foonathan_memory / Fast-DDS）──────
cmake -S "$SRC" -B "$BUILD_DIR" \
  -DCMAKE_INSTALL_PREFIX="$INSTALL_PREFIX" \
  -DUAGENT_BUILD_EXECUTABLE=ON \
  -DUAGENT_SUPERBUILD=ON \
  -DCMAKE_BUILD_TYPE=Release
make -C "$BUILD_DIR" -j"$JOBS"

echo
echo "==> 安装到 $INSTALL_PREFIX（需要写权限）"
cmake --install "$BUILD_DIR" --prefix "$INSTALL_PREFIX"
echo "✅ $INSTALL_PREFIX/bin/MicroXRCEAgent"
echo "   起它：$INSTALL_PREFIX/bin/MicroXRCEAgent udp4 -p 8888"
