#!/usr/bin/env bash
# acados 在 z0 机载电脑（Jetson Orin NX，aarch64）上的构建脚本 —— ticket 02 的可复现形式。
#
# 为什么是这三个开关（每一条都有第一手证据，见 docs/control/acados-orin-build.md）：
#   BLASFEO_TARGET=ARMV8A_ARM_CORTEX_A76   上游在 aarch64 上的默认值是 CORTEX_**A57**，
#                                          不是 A76；不显式指定就吃不到 A76 的 NEON 调度
#   ACADOS_WITH_OPENMP=OFF                 0.6.0 的默认值已是 OFF，这里显式写出是防上游改默认
#   HPIPM_TARGET=GENERIC                   hpipm 无 ARM 专属目标；上游默认即 GENERIC
#
# 用法：
#   bash tools/acados-orin/build_acados.sh                 # 落到 $HOME/acados
#   INSTALL_DIR=$HOME/acados-generic BLASFEO_TARGET=GENERIC \
#       bash tools/acados-orin/build_acados.sh             # 对照构建（量「掉一个量级」用）
#
# 源码取自仓库外的只读副本 ~/refs/acados（钉 commit，见该脚本 --print-pin）。
set -euo pipefail

SRC="${SRC:-$HOME/refs/acados}"
INSTALL_DIR="${INSTALL_DIR:-$HOME/acados}"
BUILD_DIR="${BUILD_DIR:-${INSTALL_DIR}-build}"
BLASFEO_TARGET="${BLASFEO_TARGET:-ARMV8A_ARM_CORTEX_A76}"
HPIPM_TARGET="${HPIPM_TARGET:-GENERIC}"
ACADOS_WITH_OPENMP="${ACADOS_WITH_OPENMP:-OFF}"
JOBS="${JOBS:-$(nproc)}"
# ★ 钉住的上游 commit（重建时先 git -C "$SRC" checkout 这个值）
PINNED_COMMIT="00947ddc394b30c956d9a7794d0bbcbc44c7c050"

if [[ "${1:-}" == "--print-pin" ]]; then
  echo "acados 源码 $SRC @ $PINNED_COMMIT"
  # t_renderer 的版本与 sha256 只写在 install_t_renderer.sh 一处（那边才是核对它的地方）
  echo "t_renderer: 见 tools/acados-orin/install_t_renderer.sh（v0.2.1 / 带 sha256 核对）"
  exit 0
fi

[[ -d "$SRC" ]] || { echo "找不到 acados 源码：$SRC（先 git clone --recursive）" >&2; exit 1; }
actual="$(git -C "$SRC" rev-parse HEAD)"
if [[ "$actual" != "$PINNED_COMMIT" ]]; then
  echo "⚠️  $SRC 现在在 $actual，不是钉住的 $PINNED_COMMIT" >&2
  echo "    要复现本次取证结果：git -C $SRC checkout $PINNED_COMMIT" >&2
fi

# ★ out-of-source 构建：~/refs/acados 是「只读副本」，构建产物不落在它里面
cmake -S "$SRC" -B "$BUILD_DIR" \
  -DACADOS_INSTALL_DIR="$INSTALL_DIR" \
  -DBLASFEO_TARGET="$BLASFEO_TARGET" \
  -DHPIPM_TARGET="$HPIPM_TARGET" \
  -DACADOS_WITH_OPENMP="$ACADOS_WITH_OPENMP" \
  -DCMAKE_BUILD_TYPE=Release
cmake --build "$BUILD_DIR" --target install -j"$JOBS"

# ★ 上游漏装的两个文件（第一手：CMakeLists.txt 里**没有**它们的 install 规则）
#   `lib/link_libs.json`（acados_template 硬依赖，缺了 AcadosOcpSolver 直接 FileNotFoundError）
#   `lib/git_commit_hash`（缺了只是警告）
#   它们是 `file(WRITE lib/...)` 在 **configure 时写进源码树**的，所以只在
#   「构建目录 = 安装目录 = 源码目录」的官方流程里恰好就在位；out-of-source 构建必须自己搬。
#   ⚠️ 内容随构建开关变化（openmp 那一行），所以要在**每次构建之后**搬，不能只搬一次。
mkdir -p "$INSTALL_DIR/lib"
for f in link_libs.json git_commit_hash; do
  install -m 0644 "$SRC/lib/$f" "$INSTALL_DIR/lib/$f"
done

echo
echo "已安装到 $INSTALL_DIR（含上游漏装的 lib/link_libs.json、lib/git_commit_hash）"
echo "接下来："
echo "  bash tools/acados-orin/install_t_renderer.sh      # 缺它的三种落点见文档 §4.1"
echo "  # ⚠️ 本机 pypi.org 不可达（会静默卡十分钟），必须带镜像："
echo "  python3 -m pip install --user -i https://pypi.tuna.tsinghua.edu.cn/simple \\"
echo "      -e $SRC/interfaces/acados_template"
