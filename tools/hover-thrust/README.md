# `hover-thrust` —— 从飞控日志反推模型里的 `hover_thrust`

给 [phase2 ticket 10](../../.scratch/phase2/issues/10-param-measure-backfill.md) 的 **P1** 用：
`hover_thrust` **不是**从参考机架抄来的常数，而是**用 z0 自己的悬停日志反推**出来的。
本目录就是那把尺子 —— 一条等式、一组判据、两个自检、一份证据。

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
```

退出码：`0` 可用 / `2` 算出了但不可用（附原因）/ `3` 与 `--expect` 不符 / `4` 没有可分析的数据。
`--json` 出完整结构（判据、各候选段、HTE、输入 sha256）。

## 5. 自检（不需要飞机、不需要日志、不需要飞控通电）

```bash
python3 tools/hover-thrust/test_hover_thrust_core.py --self-test      # 24 条：判据与读数
python3 tools/hover-thrust/test_hover_thrust_from_ulog.py --self-test # 12 条：适配层与坏日志
pytest tools/hover-thrust/ -q
```

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
