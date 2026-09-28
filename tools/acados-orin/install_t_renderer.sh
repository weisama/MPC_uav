#!/usr/bin/env bash
# 装 t_renderer（Tera）—— acados 代码生成时 shell 出去的那个 Rust CLI。
#
# 为什么单独一个脚本：acados 0.6.0 检测到 $ACADOS_SOURCE_DIR/bin/t_renderer 缺失时会
# **自己去 GitHub 下载**。机身有网时这很省事，但 Phase 2 的验收要求可复现 ——
# 所以这里把它固定成 v0.2.1 的 linux-arm64 二进制并核对 sha256。
# 缺它的真实症状（不是文档里流传的 EOFError）见 docs/control/acados-orin-build.md §4.1 ——
# 三种落点里有一种**完全不报错**（在、但渲染不出东西），那才是真正坑人的那种。
#
# 用法：bash tools/acados-orin/install_t_renderer.sh [安装目录，默认 $HOME/acados]
set -euo pipefail

INSTALL_DIR="${1:-${ACADOS_SOURCE_DIR:-$HOME/acados}}"
TERA_VERSION="v0.2.1"
ASSET="t_renderer-${TERA_VERSION}-linux-arm64"
URL="https://github.com/acados/tera_renderer/releases/download/${TERA_VERSION}/${ASSET}"
SHA256="17db270e2f21f25bfb994d0eaea9e0481f3c2dd081030f959c254b8862df4453"

[[ -d "$INSTALL_DIR" ]] || { echo "安装目录不存在：$INSTALL_DIR（先跑 build_acados.sh）" >&2; exit 1; }
mkdir -p "$INSTALL_DIR/bin"
tmp="$(mktemp -d)"
trap 'rm -rf "$tmp"' EXIT

echo "下载 $URL"
curl -sSL -o "$tmp/$ASSET" "$URL"
echo "$SHA256  $tmp/$ASSET" | sha256sum -c -

install -m 0755 "$tmp/$ASSET" "$INSTALL_DIR/bin/t_renderer"
"$INSTALL_DIR/bin/t_renderer" --version
echo "已装到 $INSTALL_DIR/bin/t_renderer"
