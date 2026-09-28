#!/usr/bin/env bash
# 把 z0 同构机型（10046_sihsim_z0）装进**已构建**的 SITL —— **不需要重新编译**。
#
# 为什么不用改 ROMFS 再 make：posix SITL 的 rcS 是按**名字**在 PATH 里找参数的
# （`PATH="$PATH:${R}etc/init.d-posix"`，而 posix 下 `${R}` 是空的），机型也一样：
#     REQUESTED_AUTOSTART=$(ls "${R}etc/init.d-posix/airframes" | sed -n 's/^\([0-9]*\)_'${PX4_SIM_MODEL}'$/\1/p')
# 也就是 build/px4_sitl_default/etc/init.d-posix/airframes/ 这个**宿主目录**（不是只读
# ROMFS 镜像）。所以放一个文件进去就生效 —— 见 docs/control/sitl-z0-plant.md §取证。
#
# ⚠️ 它是 build 目录里的运行期文件：**重新编译 / clean 之后要重跑本脚本**。
#    ROMFS 的条目表（airframes/CMakeLists.txt）确实没列我们这份机型，所以 make 不保证留住它。
set -euo pipefail

SRC="${SRC:-$HOME/refs/px4-v1.14.3-build}"
BUILD_DIR="${BUILD_DIR:-$SRC/build/px4_sitl_default}"
HERE="$(cd "$(dirname "$0")" && pwd)"
DEST="$BUILD_DIR/etc/init.d-posix/airframes"
AIRFRAME="10046_sihsim_z0"
MODEL="sihsim_z0"

die() { echo "错误：$*" >&2; exit 1; }

[[ -x "$BUILD_DIR/bin/px4" ]] || die "找不到 $BUILD_DIR/bin/px4 —— 先跑 tools/px4-sitl/build_px4_sitl.sh"
[[ -f "$HERE/z0-plant/airframes/$AIRFRAME" ]] || die "找不到机型文件 $HERE/z0-plant/airframes/$AIRFRAME"

# 产物与本文件无关，但必须与 spec 一致，否则装进去的是"另一种机"
python3 "$HERE/z0-plant/plant_spec.py" --check >/dev/null || {
  echo "生成产物与 plant_spec.py 不一致，先跑：python3 $HERE/z0-plant/plant_spec.py --emit" >&2
  exit 1
}

mkdir -p "$DEST"
cp "$HERE/z0-plant/airframes/$AIRFRAME" "$DEST/$AIRFRAME"

# 自检：完全按 rcS 的那条 sed 复算一遍机型 ID（不假设它找得到）
found="$(ls "$DEST" | sed -n "s/^\([0-9][0-9]*\)_${MODEL}\$/\1/p")"
[[ "$found" == "10046" ]] || die "自检失败：rcS 的查找式在 $DEST 里算出 '$found'，期望 '10046'"

echo "✅ 已装入：$DEST/$AIRFRAME（PX4_SIM_MODEL=$MODEL，SYS_AUTOSTART=10046）"
echo "   起：bash tools/px4-sitl/run_sitl.sh --plant z0"
echo "   ⚠️ 重新编译后要重跑本脚本。"
