// 遥控器"开始键"判据（ticket 09）—— 把 `/mavros/rc/in` 上的**一档开关**翻成"操作员按了开始键"。
//
// ★★ 语义的**唯一出处**是 `docs/reference/params-changes.md` §H.1 / §H.5（用户 2026-09-21 裁定），
//   逐字引在下面 —— 改代码前先读那两节，不要凭这一份注释改判据：
//     · **上升沿**触发，不是电平：`先 < 0.25` **再** `> 0.75`（归一化，与 PX4 的 `*_TH` 同一把尺），
//       **去抖 200 ms**；
//     · **第 1 次上升沿 = 解锁 + 起飞**（§H.5 的 D3 = 程序解锁），**第 2 次 = 开始轨迹**；
//     · ★★ **上电即在高位不算边沿**（必须先见低位）—— 否则"开机时开关恰好停在上位"会让飞机
//       在操作员什么都没做的情况下被受理起飞。
//
// ★ 为什么单独一个文件、且是**零 ROS 依赖**（同 `rc_stick.hpp` / `fsm.hpp`）：
//   这条判据的三个坑都在**纯逻辑**上，而它们在话题层面完全同形（"一次边沿"与"三次边沿"、
//   "忽略抖了一下"与"确认了一个电平"在 `/control/state` 上都只表现为"进了 TRACK"或"没进"）：
//     ① 上电即高位 / ② 抖动没有去抖 / ③ 一次边沿被消费两次。
//   放在这里，三条各自有一条最便宜的用例钉着（`test/test_rc_switch.cpp`）。
//
// ★ 归一化的方向由**两个参数**定义（`low_us` ↔ 0.0、`high_us` ↔ 1.0），不由名字里的 min/max 定义：
//   z0 的 `RC9_REV = 1`（`docs/assets/z0.params`）指"若有人把 ch9 映射进 PX4，PX4 会把它的
//   读数反向归一化"。§H.2 已确认 ch9 **没有被任何 `RC_MAP_*` 占用** ⟹ PX4 不会用这条通道，
//   方向只由本文件这两个参数说了算。⚠️ 一旦哪天把 ch9 映射给 PX4 用，这两个数必须与那一边
//   对齐（否则同一条开关在两边含义相反）。

#ifndef CONTROL__RC_SWITCH_HPP_
#define CONTROL__RC_SWITCH_HPP_

#include <cmath>

namespace control
{

/// 去抖判定在**边界上**的容差（s）。
///
/// ★★ 为什么必须有一个：`debounce_s` 与**帧间隔**极易正好整除 —— §H.1 的 200 ms 对
///   `/mavros/rc/in` 实测的 10 Hz 就是 2×100 ms ⟹ "第 3 帧正好到点"这件事在浮点累加下算出来是
///   0.19999999999999996，于是**同一组输入、同一条判据，时灵时不灵**
///   （现场症状是"同样的操作，有时按得动、有时按不动"，而这是最查不下去的一类报告）。
/// ★ 取 1e-9（≈1 纳秒）：它只吸收舍入误差，**不改变任何有物理意义的判定** ——
///   10 Hz 的帧间隔比它大 8 个数量级，所以"少喂一帧"永远是明确的不确认。
constexpr double kDebounceEpsilonS = 1e-9;

/// 一档**两位置开关**（`/mavros/rc/in` 的一个通道）的上升沿检测器。
///
/// 用法：每收到一帧 `/mavros/rc/in`，对**同一个通道**调一次 `update(该通道的原始读数, now_s)`；
/// 然后每帧问一次 `take_edge()`（它**消费**边沿：一次边沿只会被取走一次）。
///
/// 判据的形状（H.1）：读数折算到 [0,1]（`low_us` → 0、`high_us` → 1）后
///   · `> start_threshold` = 候选**高位**；`< release_threshold` = 候选**低位**；
///   · 两者之间 = **死区**：不作判断（保持当前候选与它的计时）——这一条就是回差，
///     它让"开关停在阈值附近微抖"不会产生一串边沿；
///   · 候选变了就重新计时，**连续**保持 `debounce_s` 才算"电平确认"；
///   · 确认的电平从低位翻到高位 ⟹ 记一条边沿。
///
/// ⚠️ 上电时电平是"未知"，从**未知**直接确认到高位**不算**边沿（H.5.2）—— 不需要额外判断，
///   它是"必须先从低位翻上来"的自然结果（`released_` 那一位）。
class StartSwitchDetector
{
public:
  /// @param start_threshold   判"高位"的折算阈值（H.1：0.75）
  /// @param release_threshold 判"低位"的折算阈值（H.1：0.25）
  /// @param debounce_s        电平要连续保持多久才算数（H.1：200 ms ⟹ 去抖）
  /// @param low_us  折算到 0.0 的原始读数（µs；默认 1000）
  /// @param high_us 折算到 1.0 的原始读数（µs；默认 2000）
  StartSwitchDetector(double start_threshold, double release_threshold, double debounce_s,
    double low_us, double high_us)
  : start_threshold_(start_threshold), release_threshold_(release_threshold),
    debounce_s_(debounce_s), low_us_(low_us), high_us_(high_us) {}

  /// 喂一帧读数。`now_s` 必须单调递增（与 `Fsm::update()` 同一个时钟）。
  /// ⚠️ 非有限读数**一律不动状态**（同 `StickMoveDetector` 的纪律）：它是"这一帧没有信息"，
  ///   不是"电平到低位了"—— 把它当成低位会凭空造出一条边沿。
  void update(double value_us, double now_s)
  {
    if (!std::isfinite(value_us)) {
      return;
    }
    normalized_ = folded(value_us);
    sample_valid_ = true;

    const Level raw = raw_level_of(normalized_);
    if (raw == Level::kUnknown) {
      // 死区：不动 `pending_`、也不动它的计时。读数既不能确认高位也不能确认低位。
      return;
    }
    if (raw != pending_) {
      pending_ = raw;
      pending_since_s_ = now_s;
      return;
    }
    if (pending_ == level_) {
      return;   // 这个电平早就确认过了，没有新的边沿可记
    }
    // ★ 边界算确认（"正好保持满 debounce_s"）⟹ 容差加在**已过时间**这一边，
    //   让"门槛就是 debounce_s"这句话在代码里读得出来。理由见 `kDebounceEpsilonS`。
    if ((now_s - pending_since_s_ + kDebounceEpsilonS) < debounce_s_) {
      return;   // 还在去抖窗里
    }

    // ── 电平确认 ──
    level_ = pending_;
    if (level_ == Level::kLow) {
      // ★ H.5.2 的那一条：只有在确认过低位之后，"高位"才是一次**边沿**。
      released_ = true;
      return;
    }
    if (released_) {
      edge_ = true;
      ++edges_;
    }
  }

  /// 取走一条边沿（**消费**语义：一次边沿只会被取走一次）。
  /// ★ 用"取走"而不是"读当前是否有边沿"，是因为调用方每帧问一次、而边沿是**事件**：
  ///   可读的边沿会在下一帧被再读一次 ⟹ 一次按键被当成两次（第 1 次解锁、第 2 次开轨迹）。
  bool take_edge()
  {
    const bool e = edge_;
    edge_ = false;
    return e;
  }

  /// 链路断过之后的复位（同 `StickMoveDetector::reset()` 的纪律）。
  /// ★★ 为什么必须有：断链期间开关**动没动过我们不知道**，而"已确认低位"这一位
  ///   （H.5.2 的判据）在那之前是真的 —— 于是"链路断了、开关恰好停在上位"会在链路恢复的
  ///   第一帧凭空产生一条边沿，而那正是 H.5.2 要挡的形状，只是触发它的是断链而不是上电。
  ///   复位之后必须**重新看见一次低位**才承认下一次高位是边沿。
  /// ⚠️ 代价（刻意接受）：链路反复抖动时，操作员每次都要把开关放下来再拨上去
  ///   （"按了没反应"）—— 安全侧的这一边是对的，而诊断面能看到断链次数（`edge_count` 不动
  ///   而按键无效时，先查 `/mavros/rc/in` 的 `age_seconds`）。
  void reset()
  {
    level_ = Level::kUnknown;
    pending_ = Level::kUnknown;
    pending_since_s_ = 0.0;
    released_ = false;
    edge_ = false;
    // ★ `edges_` **不复位**：它是"上电以来确认过几条边沿"的诊断计数，只增不减 ——
    //   复位它会让现场看到的条数与实际发生过的按键对不上（那正是它的用途）。
  }

  // ───────────────────────── 观测口（日志/测试用） ─────────────────────────

  /// 已确认的电平是不是高位。`false` 包含"未知"与"已确认低位"两种情形 ——
  /// 与真值语义（"操作员按着开始键"）一致：没确认过的都不算按着。
  bool high() const {return level_ == Level::kHigh;}

  /// 上电以来**确认过**低位（H.5.2 的诊断面：没确认过低位 ⟹ 永远不会产生边沿）。
  bool released() const {return released_;}

  /// 最近一帧的折算值（带符号；< 0 或 > 1 表示原始读数在配置的两个端点之外）。
  double normalized() const {return normalized_;}

  /// 收过至少一帧读数。
  bool sample_valid() const {return sample_valid_;}

  /// 上电以来**确认过**的边沿条数（诊断：与任务态的推进次数对不上就说明有边沿被忽略/漏掉）。
  int edge_count() const {return edges_;}

private:
  enum class Level
  {
    kUnknown,   ///< 还没看到越过任何一条阈值的读数（上电初值，也是"死区里从没出过界"的值）
    kLow,
    kHigh,
  };

  /// 折算：`low_us` → 0.0、`high_us` → 1.0（**不钳位** —— 越界是有信息量的）。
  /// ★ `span` **允许是负数**（`low_us > high_us`）—— 那是一份**合法配置**，不是配置错误：
  ///   方向由"哪个数写在哪个名字上"定义（见文件头那条 ★），换一个方向相反的接收机通道
  ///   只要把两个数对调，**不改代码**。⚠️ 只有 `span == 0`（两个端点相等）才是退化的：
  ///   那时方向无从判定，折算恒为 0 ⟹ 永远产生不了边沿（安全侧）。
  ///   ← 这里原本写的是 `if (!(span > 0.0))`，**把"反着配"和"配成相等"当成同一回事**，
  ///     于是反着配的通道永远按不动（`test_rc_switch.cpp::折算方向由参数决定` 抓到的）。
  double folded(double us) const
  {
    const double span = high_us_ - low_us_;
    if (span == 0.0) {
      // ★ 不在运行时钳位或告警：本仓的数值参数只在 `control.yaml` 受控，节点启动时会把它们
      //   打印出来（`control_node.cpp` 的启动清单），配错在那里看得见。
      return 0.0;
    }
    return (us - low_us_) / span;
  }

  Level raw_level_of(double normalized) const
  {
    if (normalized > start_threshold_) {return Level::kHigh;}
    if (normalized < release_threshold_) {return Level::kLow;}
    return Level::kUnknown;
  }

  double start_threshold_;
  double release_threshold_;
  double debounce_s_;
  double low_us_;
  double high_us_;

  Level level_{Level::kUnknown};     ///< 已确认的电平
  Level pending_{Level::kUnknown};   ///< 当前候选电平（读数给的，还没确认）
  double pending_since_s_{0.0};      ///< 候选电平**首次**出现的时刻（去抖窗的起点）
  bool released_{false};
  bool edge_{false};
  int edges_{0};
  bool sample_valid_{false};
  double normalized_{0.0};
};

}  // namespace control

#endif  // CONTROL__RC_SWITCH_HPP_
