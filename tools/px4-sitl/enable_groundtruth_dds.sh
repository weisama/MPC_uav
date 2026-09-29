#!/usr/bin/env bash
# 把 SIH 的两条**真值**话题桥到 DDS（uXRCE）—— 给 B 档（ticket 11）当"真值仪器"用。
#
# 为什么需要它（B 档的硬前提）：
#   B 档的状态源是 `/mocap/pose_enu`，而那个流要**由 SITL 真值合成**
#   （操作者裁定：包当参考 + 噪声/延迟模型）。SITL 里唯一不是估计器算出来的姿态/位置
#   就是 `vehicle_local_position_groundtruth` / `vehicle_attitude_groundtruth`
#   （`sih.cpp:495-564` 直接发刚体积分出来的 `_p_I`）。
#   而这两条**默认不在** `src/modules/uxrce_dds_client/dds_topics.yaml` 里 ⟹ DDS 上看不到。
#   A 档早就把这条路写在它自己的 docstring 里（`verify_tier_a_hover.py:23-24`：
#   "要改 `dds_topics.yaml` 并重编 PX4"），本脚本就是那句话的可复现形式。
#
# 为什么**不能**用 `/fmu/out/vehicle_local_position`（EKF）顶替：
#   这台 SITL 里 EKF 的助航（baro/GPS 仿真）是从 **EKF 自己的输出**喂回去的
#   （`SensorBaroSim.cpp:107-118`、`SensorGpsSim.cpp:117-121`）⟹ 自反馈，误差不落地；
#   实测它与真值有 0.3 m 量级的高度差。拿它当"动捕"会把估计器的偏差记到控制器账上。
#
# 机制（**只加 yaml 两行，不动任何 C++**）：
#   1. `dds_topics.yaml` 由 `generate_dds_topics.py` 在构建期读，生成 `dds_topics.h`；
#      它按 `topic` 取 ORB_ID、按 `type` 取序列化器 ⟹ 别名话题（同一 .msg 的第 2 个名字）
#      直接可用：`msg/VehicleLocalPosition.msg:75` 的 TOPICS 行里就有
#      `vehicle_local_position_groundtruth`，`vehicle_attitude_groundtruth` 同理
#      （`msg/VehicleAttitude.msg:11`），生成的头文件里确实有
#      `ORB_DECLARE(vehicle_local_position_groundtruth)`。
#   2. 所以 ROS 侧收到的**消息类型就是** `px4_msgs/msg/VehicleLocalPosition`
#      （不需要 px4_msgs 里有 Groundtruth 变体 —— 它没有，也不需要）。
#
# ⚠️ 两条后果，用之前必须知道：
#   * 改的是 **`$SRC` 那棵树**（`~/refs/px4-v1.14.3-build`，钉在 v1.14.3 = `1dacb4cdef`）。
#     它是 ticket 01 钉住的基线 ⟹ 这次改动**必须留档**（交付文档里记 commit + 本脚本）。
#     回退：`git -C $SRC checkout -- src/modules/uxrce_dds_client/dds_topics.yaml` 再重编。
#   * 重编会动 build 目录 ⟹ **`install_z0_airframe.sh` 要重跑**（它自己声明了这件事）。
#     本脚本编完会自动替你重跑一次。
#
# 用法：
#   bash tools/px4-sitl/enable_groundtruth_dds.sh --edit-only   # 只改 yaml，不编
#   bash tools/px4-sitl/enable_groundtruth_dds.sh               # 改 yaml + 增量重编 + 重装机型
set -euo pipefail

SRC="${SRC:-$HOME/refs/px4-v1.14.3-build}"
VENV="${VENV:-$HOME/refs/px4-v1.14.3-venv}"
JOBS="${JOBS:-$(nproc)}"
HERE="$(cd "$(dirname "$0")" && pwd)"
YAML="$SRC/src/modules/uxrce_dds_client/dds_topics.yaml"
BUILD_DIR="$SRC/build/px4_sitl_default"
GEN_H="$BUILD_DIR/src/modules/uxrce_dds_client/dds_topics.h"

# 要桥的两条：`锚点` 是已有的那个块，紧跟在它后面插入 `新增` 块。
# 用 python 做**逐字插入**（不用 yaml 往返 —— 那会把整个文件重排版，diff 没法看）。
read -r -d '' INSERT_SPEC <<'PY' || true
import sys
path = sys.argv[1]
pairs = [
    ("/fmu/out/vehicle_attitude\n    type: px4_msgs::msg::VehicleAttitude\n",
     "  - topic: /fmu/out/vehicle_attitude_groundtruth"
     "\n    type: px4_msgs::msg::VehicleAttitude\n"),
    ("/fmu/out/vehicle_local_position\n    type: px4_msgs::msg::VehicleLocalPosition\n",
     "  - topic: /fmu/out/vehicle_local_position_groundtruth"
     "\n    type: px4_msgs::msg::VehicleLocalPosition\n"),
]
text = open(path, encoding="utf-8").read()
added = []
for anchor, new in pairs:
    if new.splitlines()[0].strip() in text:
        continue                      # 幂等：已经在里面了
    i = text.index(anchor)            # 找不到就抛 ValueError，不静默跳过
    # 锚点必须是一条**没被注释掉**的条目（前面正好是 "  - topic: "）
    assert text[i - 11:i] == "  - topic: ", f"锚点前面不像条目头：{text[i-11:i]!r}"
    text = text[:i + len(anchor)] + new + text[i + len(anchor):]
    added.append(new.splitlines()[0].strip())
open(path, "w", encoding="utf-8").write(text)
print("\n".join(added))
PY

die() { echo "错误：$*" >&2; exit 1; }

[[ -f "$YAML" ]] || die "找不到 $YAML —— 先跑 tools/px4-sitl/build_px4_sitl.sh"

echo "==> 改 $YAML"
added="$(python3 -c "$INSERT_SPEC" "$YAML")"
if [[ -n "$added" ]]; then
  echo "    新增：$added"
else
  echo "    两条都已经在了（幂等）"
fi

echo "==> 现在的 diff（本脚本的全部改动）"
git -C "$SRC" --no-pager diff --stat -- src/modules/uxrce_dds_client/dds_topics.yaml || true
git -C "$SRC" --no-pager diff -- src/modules/uxrce_dds_client/dds_topics.yaml || true

if [[ "${1:-}" == "--edit-only" ]]; then
  echo
  echo "只改了 yaml。要生效还得："
  echo "    export PATH=$VENV/bin:\$PATH && make -C $SRC px4_sitl_default -j$JOBS"
  echo "    bash tools/px4-sitl/install_z0_airframe.sh    # 重编后要重装机型"
  exit 0
fi

echo "==> 增量重编（make px4_sitl_default -j$JOBS）"
export PATH="$VENV/bin:$PATH"
make -C "$SRC" px4_sitl_default -j"$JOBS"

echo "==> 自检：生成的 dds_topics.h 里有没有那两条"
# 生成式：`uORB::Subscription <话题>_sub{ORB_ID(<话题>)};` 与
# `create_data_writer(..., "px4_msgs::msg::dds_::<类型>_", ...)`（`dds_topics.h.em`）
for t in vehicle_local_position_groundtruth vehicle_attitude_groundtruth; do
  pat="${t}_sub{ORB_ID(${t})};"          # ★ 别在模式里写行首空白：脚本里的制表符会被吃掉
  grep -q "$pat" "$GEN_H" || die "$GEN_H 里找不到 $t 的订阅（生成器没吃到新条目？）"
  echo "    ✅ $t：$(grep -m1 "$pat" "$GEN_H" | sed 's/^[[:space:]]*//')"
done

echo "==> 重装机型（重编会清掉 build 目录里的运行期机型文件）"
bash "$HERE/install_z0_airframe.sh"

cat <<EOF

✅ 完成。起机后验：
    ros2 topic list | grep groundtruth
    # 期望两条：/fmu/out/vehicle_attitude_groundtruth、/fmu/out/vehicle_local_position_groundtruth
⚠️ 记住了：这条改动让 $SRC 与钉住的 v1.14.3 基线出现一处差异（只多了两条 DDS 发布，
   不动动力学）。它必须写进交付文档 —— 见 docs/control/tier-b-sitl-seam3.md。
EOF
