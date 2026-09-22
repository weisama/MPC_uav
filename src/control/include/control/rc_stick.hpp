// 打杆判据 —— 把 `/mavros/rc/in` 的通道值（µs）翻成"飞行员正在打杆吗"。
//
// ★★ 为什么单独一个文件、并且**照抄 PX4 的算法**：`COM_RC_STICK_OV` 的语义很容易读错。
//   第一手（z0 固件 `4a0e65f2331b23b9c8352656e9ccb6edeaea5d6c`，v1.14.3 线）：
//     · `src/modules/manual_control/ManualControl.cpp:163-168`
//         minimum_stick_change = 0.01f * COM_RC_STICK_OV;
//         sticks_moving = |_roll_diff.update(roll, dt)| > minimum_stick_change || …（俯仰/偏航/油门）
//     · `src/modules/manual_control/MovingDiff.hpp:40-58`
//         new_diff = (value - _last_value) / dt_s;  _difference_filter.update(new_diff);   // 再低通
//     · `src/lib/mathlib/math/filter/AlphaFilter.hpp:62-77,121`（`setParameters(dt, 0.1f)`）
//         alpha = dt / (0.1 + dt);  state = (1 - alpha) * state + alpha * sample
//   ⟹ **它比的是"摇杆的每秒变化率"，不是"摇杆偏移量"。** `COM_RC_STICK_OV = 20` 是 0.2/s
//   （归一化摇杆单位，满行程 = ±1），**不是"打杆 20%"** —— 本项目早期文档里那句口径已更正。
//   行为上的差别是真的：**任何一次有意的打杆都会触发**，而"摇杆推到头再保持不动"不会
//   （变化率为 0，低通之后衰减到阈值以下）。
//
// ★ 单位换算：PX4 的阈值在**归一化摇杆**上，本项目读到的是 µs。z0 实测校准
//   `RC1_MIN = 989 / RC1_MAX = 2012`（`docs/assets/z0.params`）⟹ 半行程 ≈ 511 µs ⟹
//   0.2/s ≈ 102 µs/s。默认取 `100 µs/s`，并做成参数（`rc_stick_ov_us_per_s`）——
//   **改了 PX4 的 `COM_RC_STICK_OV` 就必须同步改它**（两者必须同源，否则节点与固件各说各话）。
//
// ★ 本文件**零 ROS 依赖**（同 `fsm.hpp` / `mocap_rel_pose.hpp`）：判据能被最便宜的测试钉住，
//   不用起节点、不用 DDS。它的用量极小（4 个通道 @ 20 Hz），所以直接照抄固件的算法，不图省事。

#ifndef CONTROL__RC_STICK_HPP_
#define CONTROL__RC_STICK_HPP_

#include <cmath>

namespace control
{

/// 单通道的"在动吗"判据 —— PX4 `MovingDiff` 的等价物。
/// 用法：每收到一帧 `/mavros/rc/in`，对每个摇杆通道调一次 `update(值, 距上一帧的秒数)`。
class StickMoveDetector
{
public:
  /// @param threshold_us_per_s 变化率阈值（µs/s）。默认 100 ≈ `COM_RC_STICK_OV = 20`
  ///        × z0 实测校准（见文件头）。
  explicit StickMoveDetector(double threshold_us_per_s = 100.0)
  : threshold_us_per_s_(threshold_us_per_s) {}

  /// 返回"这一帧之后该通道算不算在动"。
  /// ⚠️ 三种情况**一律返回 false**（不作判断，而不是"当成在打杆"）：
  ///   · 第一帧（没有上一帧，变化率无从算起 —— 与 `MovingDiff` 的 `_last_value` 初值为 NaN 一致）
  ///   · `dt_s` 不是正数（同一时刻到了两帧）
  ///   · 值不是有限数（NaN/Inf）
  bool update(double value_us, double dt_s)
  {
    if (!std::isfinite(value_us) || !(dt_s > 1e-9)) {
      return false;
    }
    if (has_last_) {
      const double alpha = dt_s / (kTimeConstantS + dt_s);   // AlphaFilter::setParameters(dt, 0.1)
      rate_us_per_s_ += alpha * ((value_us - last_us_) / dt_s - rate_us_per_s_);
    }
    last_us_ = value_us;
    has_last_ = true;
    return std::fabs(rate_us_per_s_) > threshold_us_per_s_;
  }

  /// 丢帧/长时间无数据之后重新开始（下一帧不会被当成"从旧值跳过来"）。
  void reset()
  {
    has_last_ = false;
    last_us_ = 0.0;
    rate_us_per_s_ = 0.0;
  }

  /// 观测口（日志/测试用）：当前滤波后的变化率（µs/s，带符号）。
  double rate_us_per_s() const {return rate_us_per_s_;}

  double threshold_us_per_s() const {return threshold_us_per_s_;}

private:
  /// PX4 `MovingDiff` 的滤波器时间常数（`_difference_filter.setParameters(dt_s, .1f)`）。
  static constexpr double kTimeConstantS = 0.1;

  double threshold_us_per_s_;
  bool has_last_{false};
  double last_us_{0.0};
  double rate_us_per_s_{0.0};
};

}  // namespace control

#endif  // CONTROL__RC_STICK_HPP_
