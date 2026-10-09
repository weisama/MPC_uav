# `hover-thrust` —— 从飞控日志反推模型里的 `hover_thrust`

给 [phase2 ticket 10](../../.scratch/phase2/issues/10-param-measure-backfill.md) 的 **P1** 用：
`hover_thrust` **不是**从参考机架抄来的常数，而是**用 z0 自己的悬停日志反推**出来的。
本目录就是那把尺子 —— 一条等式、一组判据、三个自检、一份证据。

★ **2026-10-09 起本目录还有第二把尺子**：[`thrust_curve_check.py`](thrust_curve_check.py) ——
从**同一批日志**反解 `THR_MDL_FAC`（推力曲线因子），见 **§8**。
两者量的是**两个不同的域**（归一化推力 vs 电机信号），**不能互相替代**。

**只读本地 ulog 文件。不碰飞控、不发任何 MAVLink。**

## 1. 量的是什么（一条等式）

```
hover_thrust ≡ T_hover / T_max = m·g / T_max
```

稳态悬停时，PX4 打在 `vehicle_thrust_setpoint.xyz[2]` 上的那个数，**就是**它：

| | |
|---|---|
| 悬停需要多少推力 | `F = m·g`（受力平衡），而 NED/FRD 下机体 z 朝**下** ⟹ `thrust_body[2]` 为**负** |
| `thrust_body` 怎么变成推力 | `MulticopterRateControl.cpp:182` **原样透传** → `ControlAllocator` 伪逆混合，推力轴按**各执行器 `ct` 的均值**归一化（`ControlAllocationPseudoInverse.cpp:134`）⟹ `1.0` ≡ 所有执行器满输出 |
| 满输出是多少牛顿 | **实测的归一化上限**（满电口径 46.3 N，2026-09-29 回填），见 [`phase2-implementation.md`](../../docs/control/phase2-implementation.md) §3.3。⚠️ 它**不再**按"电池可持续"那个口径 —— 那两个角色要解耦，见 §3.3.3 末段 |
| 所以 | `hover_thrust = m·g / T_max` = 满输出时的那**一个归一化数** |

### 为什么这不是"把参数又念了一遍"

因为**稳态时实际推力会收敛到 `m·g`，而飞控自己相信的那个数是什么并不重要**。
第一手（PX4 v1.14.3）：

```cpp
// PositionControl.cpp:210 —— _hover_thrust 在"加速度 → 推力"里是个纯**增益**
float collective_thrust = _acc_sp(2) * (_hover_thrust / CONSTANTS_ONE_G) - _hover_thrust;
```

增益错了，竖直速度积分器 `_vel_int(2)`（`:87` 累积、`:143` 限幅 ±g）就会一直积，
**直到实际推力真的把重力扛住为止**。所以稳态读数反映的是**被控对象的物理**，
不是控制器手里的 `MPC_THR_HOVER`（z0 上是 `0.5`）。

⭐ **这条推论在 SITL 上被实测证实**（见 [`evidence/README.md`](evidence/README.md)）：
同一份日志里，飞控的 `MPC_THR_HOVER = 0.5`、`hover_thrust_estimate` 读数是 `0.5` 且 `valid = 0`，
而被控对象的解析真值是 `m·g/(4·SIH_T_MAX) = 0.623304` ——
**本工具读到 0.6233040**，与 `0.5` 无关。这就是"用日志量，而不是读参数"的实证。

## 2. 两个容易搞错的地方

1. **符号**：悬停时 `thrust_body[2]` **必须为负**。为正说明约定变了——
   工具**不会**悄悄取绝对值，它把符号单独报出来并把结果判为不可用。
2. **`hover_thrust_estimate` 不是独立口径**：`MPC_USE_HTE = 1` 时位置控制器用的就是它
   （`MulticopterPositionControl.cpp:361-368`，且仅 `hte.valid` 时生效），两者一致是同义反复。
   它只回答"**估计器收敛了没有**"。工具把它打印出来并标注警告，但**不**参与读数。

## 3. 判据与误差界

一段样本算"稳态悬停"，要同时满足（[`hover_thrust_core.py`](hover_thrust_core.py) `steady_hover_mask`）：

| 判据 | 默认 | 挡掉什么 |
|---|---|---|
| `|v| ≤ v_eps` | **0.30 m/s** | 机动/巡航（悬停时速度停在零附近，但**不是**严格为零）。★ `|v|` 是**三维**速度模 `sqrt(vx²+vy²+vz²)`，**不是**水平速度 —— 垂直爬升/下降时水平分量是 0，只取水平就会被误判成悬停 |
| 倾角 `≤ tilt_eps` | **10°** | 侧倾着顶风/顶位差——推力被拿去抵消水平分量，不再等于 `m·g` |
| `0.15 ≤ |thrust_z| ≤ 0.98` | —— | 地面怠速、以及已经打满的段 |
| 段长 `≥ min_duration` | **2 s** | 太短 ⟹ 位置环的积分还没把差补上 |
| 段内 / 段间一致 | 5% / 10% | 这一段里"悬停推力"不是**一个**数（电池掉压、还在爬升、夹着瞬态） |

★ **精度不是靠 `v_eps` 保证的**，是两个**有量纲依据**的界：

- 侧倾 `θ` 时竖直分量按 `cos θ` 拿 ⟹ `tilt_eps = 10°` 给出 **`1 − cos 10° = 1.5%`**；
- 掩码逐样本判 `|v| ≤ v_eps` ⟹ 连通段内速度变化 ≤ `2·v_eps` ⟹
  整段**持续**偏掉的上界 `δ ≤ 2·v_eps / (g·T)`，默认值给出 **3.1%**（`sustained_error_bound()`）。

⚠️ 这三个数（`v_eps` / `min_duration` / `tilt_eps`）由上面两条界**绑在一起**，不能各改各的；
自检里有用例钉着它们。改任一个，请同时改本节与 §3.3。

## 4. 怎么用

```bash
# 1) 先从飞控取悬停日志（★ 取日志是只读操作，不涉及解锁/起飞）
python3 tools/prereq-evidence/mavlink_probe.py --help     # 见该工具的纪律说明

# 2) 反推（--model-hover 只用来打印与模型当前值的差距）
python3 tools/hover-thrust/hover_thrust_from_ulog.py <悬停日志.ulg> --model-hover 0.1589

# 若手上有独立已知值（SITL 同构机那种），当场对一下
python3 tools/hover-thrust/hover_thrust_from_ulog.py <log.ulg> --expect 0.62330

# 一批日志：跨日志复现 + 判据敏感性（反"卡在分界上挑数据"）
python3 tools/hover-thrust/corpus_check.py <日志目录> --expect 0.62330 --v-eps 0.15 0.30 0.40 0.60

# 3) 同一批日志的**第二把尺子**：反解 THR_MDL_FAC（推力曲线因子，见 §8）
python3 tools/hover-thrust/thrust_curve_check.py <悬停日志.ulg>
```

退出码：`0` 可用 / `2` 算出了但不可用（附原因）/ `3` 与 `--expect` 不符 / `4` 没有可分析的数据。
`--json` 出完整结构（判据、各候选段、HTE、输入 sha256）。

## 5. 自检（不需要飞机、不需要日志、不需要飞控通电）

```bash
python3 tools/hover-thrust/test_hover_thrust_core.py --self-test      # 24 条：判据与读数
python3 tools/hover-thrust/test_hover_thrust_from_ulog.py --self-test # 12 条：适配层与坏日志
python3 tools/hover-thrust/thrust_curve_check.py --self-test          # 11 条：推力曲线反解（§8）
pytest tools/hover-thrust/ -q
```

★ **第三把尺子（§8）由 [`test_thrust_curve_check.py`](test_thrust_curve_check.py) 接进 pytest** ——
它**不重抄**那 11 条判据，只是把 `thrust_curve_check.self_check()` 转成 pytest 用例，
否则上面那条 `pytest tools/hover-thrust/ -q` **收不到** §8 的尺子（`self_check()` 是现场报告式的）。

`test_hover_thrust_from_ulog.py` 用一份**假 ULog**（真值已知的合成日志）驱动适配层。
它存在的直接原因：这批判据第一次在 175 份真日志上批跑时，暴露出**崩溃**而不是干净结论的路径 ——
而"崩溃"在批跑脚本里会被 `except` 吞掉，看起来和"这份日志没数据"一模一样。
目前钉住三条：空在飞窗口、报错路径里引用了 pyulog 并不存在的 `file_name`、
以及 topic 在但**字段**不在（`vehicle_control_mode` 没有 `flag_armed`）时的 `KeyError`。
★ 这三条的共同形状是"**报错路径自己炸掉**，把真因盖住"：
所以 `_load(..., optional=True)` 把两类结局分开 ——
topic 整个不在 = 「这份日志没数据」（干净结论）；topic 在而字段不在 = 固件/机型不对（大声报错）。

## 6. 验证状态

| | 状态 |
|---|---|
| 合成信号上的判据与读数 | ✅ 36 条自检 |
| SITL 同构机（解析真值已知） | ✅ 单份端到端偏差 **2.6 × 10⁻⁸**；175 份批跑 **50 份**可用读数极差 **0.0000%**；4 档判据下**只出现过 1 个读数** |
| **真机 z0 悬停日志** | ✅ **已跑并已回填（2026-09-29，四段真机 ulog）**：两段可用 **`0.221992`**（@ 20.474 V）/ **`0.220884`**（@ 20.526 V），互差 0.50 %，倾角 ≤ 1.75°。★ 电池口径由操作者裁定为**满电**（[ticket 10](../../.scratch/phase2/issues/10-param-measure-backfill.md) 的 O8）⟹ 按实测律 `thrust ∝ V^1.608` 归一到 25.2 V 得 `0.15896` / `0.15882` ⟹ **`0.1589`**，已回填 `params.py` / `mpc_node.py` ×2。原始分析记在该票 Comments 第二 ~ 四轮 |
| **`THR_MDL_FAC` 反解**（§8） | ✅ **2026-10-09 在同样两条真机日志上跑过**：扫 `fac` 的 argmax r² 落在 **0.450 / 0.455**（两个独立飞行日），对照档 `fac=0` 得 0.9952 / 0.9955、`fac=1` 得 0.9871 / 0.9836 ⟹ **`0.45` 这个值被飞行数据支持**（网格步长 0.005，故分辨到 ±0.005）。支撑 [phase4/02](../../.scratch/phase4/issues/02-hover-thrust-backfill.md) 验收项 4；原始输出见 [`evidence/thrust_curve_2026-10-09.txt`](evidence/thrust_curve_2026-10-09.txt) |

## 7. 它能与不能证明什么

- ✅ 能把被控对象**自己的**悬停油门从日志里读回来，且读数不随判据移动。
- ⚠️ SITL 那份真值 `0.6233` 是**同构机自己的解析真值**（`m·g/(4·SIH_T_MAX)`），
  它当初与 §3.3 的**占位值**一致纯属巧合（同构机就是照占位搭的）⟹ 那份验证的是**管线**，
  **不是** z0 的电机/电池能力。★ 2026-09-29 真机回填后**两者不再相等**（0.1589 vs 0.6233），
  这恰好把"读数来自日志、不是抄参数"这件事反证了一遍。
- ❌ **不修正竖直加速度**（`vehicle_local_position` 的 `ax/ay/az` 不读，自检里有用例钉住）。
- ⚠️ SITL 上"干净得过分"（无气动阻力、无掉压、无振动）；**真机已跑过四段并已回填**（见 §6 与 ticket 10 的 Comments），
  但真机读数**还没经拉力台复核**，且**满电那一点是外推的**（实测点都在 20.5 V 附近）——
  换成直接读数只需在满电包上再悬停 ~30 s。
- ⚠️ 回填 0.1589 后 **SITL 同构机（`SIH_T_MAX` 那套）不再与模型同构**：它自己的悬停油门仍是解析的 `0.6233`。
  本目录的 SITL 证据不受影响（它证明的是工具管线），但**基于同构机的 SITL 测试会被这个失配影响** —— 见 ticket 10 的 Comments。

完整交代、原始输出与复跑命令：[`evidence/README.md`](evidence/README.md)。

---

## 8. ★ 第二把尺子：[`thrust_curve_check.py`](thrust_curve_check.py) —— 反解 `THR_MDL_FAC`

> 2026-10-09 加，给 [phase4/02](../../.scratch/phase4/issues/02-hover-thrust-backfill.md) 的验收项 4
> （「与 `THR_MDL_FAC = 0.45` 的关系一并复核」）用。

**为什么它和上面那把尺子不能合并**：`MPC_THR_HOVER` / `hover_thrust` 工作在**归一化推力域**，
而 `THR_MDL_FAC` 描述的曲线在**更下游** —— 它决定「归一化推力 ↔ 电机信号」这条映射的**形状**
（PX4 `v1.14.3` `lib/mixer_module/functions/FunctionMotors.hpp:77-101`）。所以：

- 换 `MPC_THR_HOVER` **不需要**动 `THR_MDL_FAC`（两者不同域）；
- 但"悬停**油门**是多少"这个量**只有**在曲线已知时才和"悬停**推力**是多少"互换 ——
  ⟹ 复核推力模型时两者必须**各自独立量一次**，不能用一个去推另一个。

**它怎么测**（方法只有一句）：

```
分配器给 rel_thrust（= actuator_motors.control[]），混控器反解曲线给出电机信号
（= actuator_outputs.output[] 的线性前身）：
    正: rel = fac·x² + (1−fac)·x
    反: x = −t1 + sqrt(t1² + rel/fac),   t1 = (1−fac)/(2·fac)
```

把 `fac` 当**自由参数**扫一遍 `[0, 1]`，看飞行数据把 `r²` 的**峰**顶在哪里。
判据只吃**形状**：输出域那把仿射（DShot 的 scale/offset）是自由的、与 `fac` 无关 ⟹
不需要知道量程、不需要绝对标定，也就不会把一个标定假设偷偷塞进结论里
（自检里有一条钉着"`output` 做仿射变换后 r² 不变"）。

**2026-10-09 的读数**（两条真机日志，详见 [`evidence/thrust_curve_2026-10-09.txt`](evidence/thrust_curve_2026-10-09.txt)）：

| 日志 | `fac` 的 argmax | 平均 r² | 对照 `fac=0` | 对照 `fac=1` |
|---|---|---|---|---|
| `bag/old/3_traj1_hover/fcu/log_2_2026-9-29-15-29-26.ulg` | **0.455** | 0.999823 | 0.995218 | 0.987114 |
| `bag/old/4_traj3_lemniscate/fcu/log_3_2026-9-29-15-32-40.ulg` | **0.450** | 0.999940 | 0.995464 | 0.983590 |

⟹ **`z0.params:947` 的 `THR_MDL_FAC = 0.449999988` 被飞行数据支持**（网格步长 0.005 ⟹ 分辨到 ±0.005）。
★ 顺带**独立复现**了 `PWM_MAIN_FUNC1..4 = 103, 102, 101, 104`（电机序 `3-2-1-4`）：
脚本**不硬编码**这个对应，它按 r² 自己推，两条日志都推出
`out0←ctrl[2]、out1←ctrl[1]、out2←ctrl[0]、out3←ctrl[3]`，与 `D7` 相符且是个双射。

⚠️ **两条容易误用的**：

1. 输出域那把仿射是 **DShot 编码**（两条日志都落在 `output ≈ 1889·x + 107` ⟹ `x=0→107`、`x=1→1996`），
   **不是** `1999·x`（有零偏）。**别拿 `output/1999` 当电机信号** —— 那会差一个常数。
   ★ 这也修正了 [ticket 10](../../.scratch/phase2/issues/10-param-measure-backfill.md) Comments 里
   「日志里该段 DShot 值 ≈ 640/1999 = **0.318**」那条**算式**：那一段 `out1` 中位数是 **641**、
   `ctrl[1]` 中位数是 **0.1908**，按本日志的编码反解得 `(641 − 107)/1889 = 0.283 ≈ inv(0.1908) = 0.281`。
   而 0.318 那个数其实是 **`inv(ctrl 的均值)`**（`inv(0.2210) = 0.3187`，即"四个电机的平均电机信号"）——
   `640/1999` 恰好凑近它，纯属巧合。**数没写错，算错了**；这种算式的麻烦在于**下次复算会得出不同的数**。
2. ⚠️ 本脚本给出的是**形状**结论，**不是** `hover_thrust`。`fac` 反解出来了也**不能**反推悬停推力
   （那需要绝对标定，本脚本刻意不引入）。悬停推力归
   [`hover_thrust_from_ulog.py`](hover_thrust_from_ulog.py)。
3. ⚠️ 分析窗口是本脚本**自己**定的（`control > 0.02` 的那一段），**不是**
   `hover_thrust_from_ulog.py` 的"`flag_armed ∧ ¬landed`"窗口，更不是它**再往下挑的稳态悬停段** ——
   别把两条命令的窗口当成同一个。同一份 `log_2` 上三者分别是
   **`119.779 – 196.279`**（本脚本）/ **`120.599 – 196.398`**（`flag_armed ∧ ¬landed`）/
   **`122.338 – 192.758`**（稳态悬停段，`hover_thrust = 0.22199` 就是它）⟹
   首尾差到**零点几秒**（不是几十毫秒）量级，换个窗口读数就会动。
4. ⚠️ 它测的是**混控器→驱动器**这一段。它**不能**证明"电机实际出了多少推力"
   （那要拉力台，是 [`battery-thrust`](../battery-thrust/README.md) 那类工具与硬件的事）。
