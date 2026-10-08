#!/usr/bin/env bash
# PX4 SITL（上游 v1.14.3）在 z0 机载电脑（Jetson Orin NX，aarch64）上的构建脚本 ——
# ticket 01 的可复现形式。**Phase 4 重编固件要复用的就是这个构建环境**（Q37=a）。
#
# ⚠️ 一句话挡住最常见的误读：**在这里编 SITL ≠ 刷飞控**（Q12 的「不刷固件」决策不变）。
#    本脚本只编给仿真器跑的那份 posix 二进制（`build/px4_sitl_default/bin/px4`），
#    飞控里烧着的仍然是出厂那份 `Minderring/PX4-Autopilot` @ `4a0e65f2`，一个字都没动。
#
# 三个坑（都是本机实测踩到的，不是抄来的）：
#   1. `Tools/setup/requirements.txt` 里的 `matplotlib>=3.0.*` **新版 pip 直接拒收**
#      （`.*` 后缀只能配 `==`/`!=`）⟹ 本脚本过滤掉该项再装。
#      构建 PX4 不需要 matplotlib/pandas/lxml，滤掉不影响。
#   2. **empy 必须钉 3.3.4**：PX4 v1.14.3 的模板调用是 `em.Interpreter(..., options={em.RAW_OPT: True, ...})`，
#      那是 **empy 3.x 的 API**；`requirements.txt` 只写 `empy>=3.3`，照着装会拿到 4.x 并炸。
#   3. ★ 本机 `pip` 直连 pypi 会**静默卡住**（`pypi.org` 不可达而 `files.pythonhosted.org` 可达）
#      ⟹ 一律加 `-i https://pypi.tuna.tsinghua.edu.cn/simple`。
#
# 用法：
#   bash tools/px4-sitl/build_px4_sitl.sh            # clone（若缺）+ 建 venv + 编 SITL
#   bash tools/px4-sitl/build_px4_sitl.sh --print-pin
#
# 产物：
#   $SRC/build/px4_sitl_default/bin/px4   —— SITL 可执行文件
#   $VENV/bin/python3                     —— 构建期用的 python（含 empy 3.3.4）
set -euo pipefail

SRC="${SRC:-$HOME/refs/px4-v1.14.3-build}"
VENV="${VENV:-$HOME/refs/px4-v1.14.3-venv}"
JOBS="${JOBS:-$(nproc)}"
PIP_INDEX="${PIP_INDEX:-https://pypi.tuna.tsinghua.edu.cn/simple}"

# ★ 钉住的上游 tag 与它解引用到的 commit（tag 是 annotated，两者不同，别混）
PINNED_TAG="v1.14.3"
PINNED_COMMIT="1dacb4cdef2d7145754fc788fa8dc482eed74b40"

if [[ "${1:-}" == "--print-pin" ]]; then
  echo "PX4 源码 $SRC @ tag $PINNED_TAG（commit $PINNED_COMMIT）"
  echo "仿真后端: SIH（sihsim_quadx，SYS_AUTOSTART=10040）—— 见 docs/control/px4-sitl-baseline.md §3"
  exit 0
fi

# ── 1. 源码 ────────────────────────────────────────────────────────────────
if [[ ! -d "$SRC/.git" ]]; then
  echo "==> clone $PINNED_TAG（带 submodule）到 $SRC"
  git clone --recursive --branch "$PINNED_TAG" --single-branch \
    https://github.com/PX4/PX4-Autopilot.git "$SRC"
fi
actual="$(git -C "$SRC" rev-parse HEAD)"
if [[ "$actual" != "$PINNED_COMMIT" ]]; then
  echo "⚠️  $SRC 现在在 $actual，不是钉住的 $PINNED_COMMIT" >&2
fi
# submodule 缺一个就编不过，且报错不指向根因
if git -C "$SRC" submodule status | grep -q '^-'; then
  echo "==> 补 submodule"
  git -C "$SRC" submodule update --init --recursive
fi

# ── 2. python 构建依赖（独立 venv，不碰 ROS 的 system python）────────────────
if [[ ! -x "$VENV/bin/python3" ]]; then
  echo "==> 建 venv $VENV"
  python3 -m venv "$VENV"
fi
"$VENV/bin/pip" install -q -U pip
# 坑 1：滤掉新版 pip 拒收的那一行；坑 3：走国内源
grep -vE '^matplotlib' "$SRC/Tools/setup/requirements.txt" > "$VENV/requirements.filtered.txt"
"$VENV/bin/pip" install -q -i "$PIP_INDEX" "numpy<2" -r "$VENV/requirements.filtered.txt"
# 坑 2：empy 钉 3.x —— 必须在 requirements 之后装，否则会被 >=3.3 覆盖成 4.x
"$VENV/bin/pip" install -q -i "$PIP_INDEX" "empy==3.3.4"
"$VENV/bin/python3" -c 'import em; assert hasattr(em, "RAW_OPT"), "empy 不是 3.x"'

# ── 3. 编译 ────────────────────────────────────────────────────────────────
echo "==> make px4_sitl_default（-j$JOBS）"
export PATH="$VENV/bin:$PATH"
make -C "$SRC" px4_sitl_default -j"$JOBS"

echo
echo "✅ 产物：$SRC/build/px4_sitl_default/bin/px4"
echo "   起 SITL：bash tools/px4-sitl/run_sitl.sh"
