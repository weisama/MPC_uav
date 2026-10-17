# `ladder-baseline` —— 从包里出「同口径跟踪误差」

服务于 ticket [`../../.scratch/phase4/issues/06-ladder-acceptance-criteria.md`](../../.scratch/phase4/issues/06-ladder-acceptance-criteria.md)
（起飞前写定四级阶梯的量化判据）。判据表本身在
[`docs/control/ladder-acceptance-criteria.md`](../../docs/control/ladder-acceptance-criteria.md)，
**结论写在那边，不在本文件里**。

**它量的是**：在「控制器正在跟轨迹」的那段窗口里，**实际位置偏离它自己的指令多少**。

**为什么必须先量 (A) 级**：阶梯 4 的判据是「MPC（模型预测控制）的读数**不差于** (A) 级同一套判据」。
"不差于"要给一个数作为线，那个数只能从 **(A) 级的实飞**里出 —— 拿 SITL 的数当线，
比的是仿真与实机，答不了"MPC（模型预测控制）比 PID 如何"。

★ 更基本的一条：**两侧必须由同一份代码、同一条规则算出来**。
两个各写各的分析脚本、各自"看起来对"的读数相减，减掉的不是控制器差异，是口径差异。
本工具的存在就是为了让这件事有一个可执行的唯一定义。

## 口径（唯一定义）

| 量 | 定义 |
|---|---|
| **窗口** | `/control/state` 里**最长的一段 `AUTO/TRACK`**，取该段**首拍到末拍**。`t-anchored` 在包里没有 `/control/state`（模型预测控制自发起步、没起 control）时取 `/traj/reference` 首末帧；有的话取参考覆盖区间 ∩ `AUTO/TRACK` |
| **实际** | `/mocap/pose_enu`（`geometry_msgs/PoseStamped`，ENU） |
| **指令（口径 a）** | `/mavros/setpoint_raw/local`（`mavros_msgs/PositionTarget`）—— 飞的是 PX4 内置级联 PID |
| **指令（口径 c）** | `/mpc/reference`（`visualization_msgs/Marker`）—— MPC（模型预测控制）当拍喂给求解器的**绝对**参考 |
| **指令（口径 c-target）** | `/control/target`（`mavros_msgs/PositionTarget`）—— `reference_source=control` 时模型预测控制 **原样消费**的绝对目标 |
| **指令（口径 t-anchored）** | **锚点 + `/traj/reference`** —— `reference_source=traj` 时模型预测控制实际算出的绝对目标；锚点按 `node_core._update_anchor` 的规则从包里重建 |
| **误差** | `e(t) = p_实际(t) − p_指令(t)`；水平报 **`hypot(e_x, e_y)`**，垂直报**带符号的** `e_z` |
| **采样** | 两条序列都线性插值到**同一个** 50 Hz 网格（`--hz` 可改） |

三条容易搞错、且**错了不会报错**的地方：

1. **窗口只由 `/control/state` 定**。不许用"动捕动了没有"去猜 —— 那是启发式，阈值一改窗口就变
   （`tools/tau-analysis/README.md`「读分段」里记着同一条坑）。
2. **零点两侧相同**。`/mpc/reference` 与 `/control/target` 是**绝对**位置；
   `/traj/reference` **不是**（是相对轨迹原点的偏移，悬停时恒为 0），
   **不能直接拿它当指令** —— 必须先加锚点（`--caliber t-anchored`）。
   ★ 锚点 = **闸门第一次打开那一拍已收到的最新一帧动捕**，不是轨迹起点、也不是"起飞瞬间"。
   2026-10-17 系绳那趟：闸门首开 t 的动捕 z = **0.1267 m**，`/traj/start` 那一刻是 0.1220 m
   —— 差约 **5 mm**（4.8 mm 按插值、5.0 mm 按最近帧；两者相差 9.2 ms）；拿后者当锚点会让整段 `e_z` 平移这个量，**看着仍像正常跟踪误差**。
   ★★ `control` 照发 `/control/target`，**不代表模型预测控制消费它** —— 同一个 `control` 进程可以
   `output_mode=external` 而模型预测控制仍用 `reference_source=traj`。**以模型预测控制启动日志的 `参考=` 为准**。
   2026-10-17 悬停那趟反过来踩过：启动行明写 `参考=/control/target`，若误按 traj 模式重建锚点，
   两个"原点"差 **9.2 cm（x，−0.1611 对 −0.0690）**，水平均值会从正确的 **0.0861** 误报成
   **0.0537**（低 38%）—— 两个数都"看起来正常"，没有任何报错。
3. **`coordinate_frame` 的标注不能拿来判正方向**。见下方「踩过的坑」第 3 条。

## 快速开始

```bash
# 自检 —— 不需要包、不需要飞机、不需要开 ROS
python3 tools/ladder-baseline/test_ladder_core.py --self-test
python3 -m pytest tools/ladder-baseline/test_ladder_core.py -q

# 读一份包（口径 a = (A) 级）
python3 tools/ladder-baseline/ladder_from_bag.py bag/4_traj2_square --caliber a

# 留一份机器可读的结果（另存 `evidence/` 那份就是这么来的）
python3 tools/ladder-baseline/ladder_from_bag.py bag/4_traj2_square --caliber a \
  --json tools/ladder-baseline/evidence/A-4_traj2_square.json

# (C) 级：按模型预测控制启动日志的 `参考=` 选口径
python3 tools/ladder-baseline/ladder_from_bag.py bag/6_free__20261017-070446 \
  --caliber c-target --json tools/ladder-baseline/evidence/C-target-6_free__20261017-070446.json
python3 tools/ladder-baseline/ladder_from_bag.py bag/6_free__20261017-070103 \
  --caliber c-target   # ★ 该趟模型预测控制启动行写的是 参考=/control/target

# 阶梯 3 的「稳态悬停 30 s」：先按窗口规则切出，再只留末尾 30 s（切法进 JSON）
python3 tools/ladder-baseline/ladder_from_bag.py bag/6_free__20261017-020757 \
  --caliber t-anchored --window-last 30
```

脚本**只读本地文件**，不碰飞控、不发任何 MAVLink、不启动任何节点。

## (A) 级基线（2026-10-09 快照）

数**现算**，别抄 —— 跑上面那条 `--json` 命令，或

```bash
for f in tools/ladder-baseline/evidence/A-*.json; do python3 -c "
import json,sys; d=json.load(open('$f')); u=d['unaligned']; a=d.get('aligned',{})
print(d['bag'], d['window_len_s'], u['xy_mean'], u['xy_max'], a.get('dt_s'))"; done
```

| 包 | 窗长 s | 水平均值 | 水平最大 | e_z 均值 | \|e_z\| 均值 | 对齐 dt* s | 对齐后均值 | 对齐后最大 |
|---|---|---|---|---|---|---|---|---|
| `bag/3_traj1_hover` | 39.20 | 0.1049 | 0.1610 | -0.0018 | 0.0032 | n/a（曲线是平的） | 0.1049 | 0.1610 |
| `bag/4_traj2_square` | 8.00 | 0.5682 | 0.7552 | -0.0118 | 0.0118 | -1.42 | 0.0972 | 0.2342 |
| `bag/5_traj3_lemniscate` | 12.02 | 0.5886 | 0.8160 | -0.0106 | 0.0106 | -1.42 | 0.0798 | 0.1829 |
| `bag/old/3_traj1_hover` | 59.20 | 0.0506 | 0.1114 | -0.0051 | 0.0067 | n/a（曲线是平的） | 0.0506 | 0.1114 |
| `bag/old/4_traj3_lemniscate` | 6.08 | 0.3802 | 0.6019 | -0.0133 | 0.0148 | -1.32 | 0.1124 | 0.2313 |

**口径可信度**：`bag/old/` 那两行复现了 [`bag/README.md`](../../bag/README.md) 已发表的读数
（悬停 0.0506/0.1114 对 **5.1/10.7 cm**、`e_z` −5.1/20.2 mm 对 **−5/19 mm**；
0.5 m 八字 0.3802/0.6019 对 **0.371/0.592 m**）—— 差在窗口边缘几十毫秒，量级与结论一致。
⟹ 本工具的口径与那份文档的口径**是同一个**，不是另起一套。

**表里读得出、也是这一轮最重要的两件事**：

- ★★ **对齐后残差（0.08–0.11 m）远小于未对齐读数（0.38–0.59 m）** ⟹ (A) 级在这三条轨迹上的
  误差**主要与一个约 1.3–1.4 s 的等效滞后同向**，不是稳态偏差。对齐后的**最大**误差
  （0.18–0.23 m）留在拐角/极值处，那里是**幅值压缩**（0.5 m 八字的包络 0.755×0.189 对参考
  1.000×0.500），与滞后无关。两样合起来是**限幅/带宽**的特征。
- ★★ **同一场景两次飞行可以差 2 倍**：1 m 原地悬停 09-29 是 **0.0506**、10-08 是 **0.1049**。
  分段看不是起机瞬态 —— 09-29 全程 0.043–0.068、10-08 全程 0.084–0.122，**是常驻偏差**。
  ⟹ 单次飞行的数当不了基线，判据表必须写明取哪一趟（见判据表的「基线取值」一节）。

## (C) 级实飞读数（2026-10-17）

四份包的 `evidence/` JSON 是本节数字的来源。**口径按各自的模型预测控制启动行选**：

| 包 | 口径 | 窗长 s | 水平均值 | 水平最大 | e_z 均值 | \|e_z\| 均值 | \|e_z\| 最大 |
|---|---|---|---|---|---|---|---|
| `6_free__20261017-070103`（轨迹 1 悬停） | `c-target` | 59.14 | 0.0861 | 0.1468 | −0.0035 | 0.0052 | 0.0189 |
| `6_free__20261017-070446`（轨迹 2 正方形） | `c-target` | 7.13 | 0.2628 | 0.3681 | −0.0061 | 0.0132 | 0.0390 |
| `6_free__20261017-070935`（轨迹 3 八字） | `c-target` | 12.00 | 0.2668 | 0.3892 | −0.0012 | 0.0166 | 0.0317 |
| `6_free__20261017-020757`（系绳，**末 30 s**） | `t-anchored` | 30.00 | 0.0652 | 0.1273 | −0.0059 | 0.0079 | 0.0222 |

与 (A) 级线并排看：悬停 0.0861/0.1468 对 **0.1049/0.1610**；正方形 0.2628/0.3681 对
**0.5682/0.7552**；八字 0.2668/0.3892 对 **0.5886/0.8160**（均为未对齐验收口径）。
★ 系绳那趟的**整窗**（55.54 s，含起飞斜坡）是 0.0797/0.3464；末 30 s 才是稳态读数。
★★ **四个包的口径都按各自的模型预测控制启动行选的**：系绳 `参考=/traj/reference` ⟹ `t-anchored`；
悬停/正方形/八字 `参考=/control/target` ⟹ `c-target`。**不是**按 `control` 的 `output_mode` 猜。

★ 两个自由飞包的 `aligned.dt_s` 都是 −0.5 s 量级（正方形 −0.54、八字 −0.52），
比 (A) 级的 −1.42 s 小；悬停那趟曲线平，`dt_s` 无意义。

★★ **系绳那趟的起飞段是真的有瞬态**（用户肉眼看到的"飞偏了"）：水平误差在
`/traj/start` **+1.04 s** 达到 **0.3464 m**，+4 s 时落到 0.0413 m；之后稳态误差在
0.05–0.08 m 之间（末 30 s 均值 0.0652）。垂直误差在 **+2.02 s** 达 **−0.2206 m**
（该时刻实际 z=0.106 m、指令 0.327 m；整窗实际最低 z=0.0458 m @ +0.61 s），
**+13.60 s** 首次回到 `|e_z| ≤ 0.02 m`；**+35.00…+35.72 s** 还有一次 0.0222 m，
此后再未超过（`readings.z_abs_settle_t_rel`）。
末 30 s 的读数**不包含**这段（窗口切掉了它）；**整窗**的 `|e_z|` 最大 **0.2206 m**
（> 判据 ③ 的 0.20 m，见票 10 的记录）。

## 输出里的每个量是干什么的

| 字段 | 干什么用 |
|---|---|
| `unaligned.{xy_mean,xy_max,xy_p95,z_mean,z_absmean,z_max}` | **验收口径**。判据表里的线只认这一组 |
| `aligned.{dt_s,metrics}` | **诊断口径，非验收**。回答"这 0.57 m 是跟不上还是只是慢起机" |
| `aligned.flat` | 曲线平 ⟹ 窗内**没有可对齐的激励**（悬停时指令是常量），`dt_s` 是噪声挑的一格、不是滞后 |
| `aligned.coverage_ok` | 偏移扫到了指令序列覆盖范围之外 ⟹ 端点被**静默夹取**，对齐退化 |
| `segments[]` | 等分各段。**分辨瞬态与常驻偏差**必需 —— 全窗均值会把它俩混在一起 |
| `path_len_actual` / `path_len_cmd` | 纯滞后 ⟹ 两者接近；走得慢 ⟹ 实际明显更短。指令是常量时比值为 `n/a` |
| `envelope_xy_*` | 幅值压缩（"画小了"）只在这一栏露头 |
| `readings.progress_max` | 轨迹是否走完（判据 ①）。<1 ⟹ 录制先停、轨迹没跑完 |
| `readings.actual_z_min` / `_t_rel` | 判据 ② 的最低高度。**不是**误差峰值时刻的实际 z |
| `readings.state_window_{armed_offboard,total}` | 判据 ② 的"窗内全程 `OFFBOARD+armed`" |
| `readings.battery_v_{min,max}` | 「条件」栏的电池区间 |
| `readings.peak_{xy,z_abs}` / `_t_rel` | 误差峰值与其时刻（瞬态 vs 常驻的分界） |
| `readings.z_abs_settle_t_rel` | `\|e_z\|` **最后一次** >0.02 m 的时刻（此后一直在带内） |
| `readings.tail10_{dx,dy,net}` | **阶梯 3** ②（不持续漂移）的末 10 s 水平净位移；阶梯 4 ② 是高度/状态，**不看这一栏**；窗短于 10 s 时没有 |

## 踩过的坑（★ 都是这一轮真踩到的）

### 1. ★★ 解压目录只用包名 ⟹ 两个包读同一份数据（**静默**）

`bag/old/3_traj1_hover` 与 `bag/3_traj1_hover` 的 `.name` **都是** `3_traj1_hover`。
第一版把解压目录定成 `/tmp/z0-ladder/<name>`，于是后者复用了前者解出来的 `.db3`
（`prepare_bag` 见文件在就跳过解压）——**两个不同日期的包给出逐位相同的读数，没有任何报错**。
它刚好会把"同场景两次飞行差 2 倍"这个真实现象**抹平**，正撞在这一轮最要紧的那条结论上。

修法：解压目录由**整个相对路径**决定（`bag/old/3_traj1_hover` → `bag__old__3_traj1_hover`）。

### 2. ★★ `_GAP_S` 必须明显大于 `/control/state` 的周期

`/control/state` 实测只有 **1 Hz**（中位 1000 ms、最大 1038 ms，五份包一致，
见 [`evidence/control-state-rate.txt`](evidence/control-state-rate.txt)）。
断档阈值第一版取 0.5 s ⟹ 每一拍都成了独立的一小段（长度 0）⟹ 窗口塌成一个点，
报错是 `make_grid` 的"窗口非法"，**看着像网格的问题**。取 2.5 s（容忍连丢两拍）。
自检里两条用例分别钉住这个常数的上界与下界。

⚠️ **这个 1 Hz 是窗口边界精度的上限**：两端各 ±1 拍。对 8 s 的正方形窗口就是 ±12%。
两侧用同一条规则 ⟹ 口径一致，但**别把窗长当成精确到毫秒的数**。

### 3. ★★ `coordinate_frame = FRAME_LOCAL_NED` 是**标注**，数值其实是 ENU

(A) 侧发的 `coordinate_frame` 是 `1`（`FRAME_LOCAL_NED`），而动捕是 ENU。
谁"照标注把 z 取反"，`e_z` 会从毫米级变成 **2 m** 级、而 x/y 一切正常 —— 这类错只在这一栏露头。

第一手（2026-10-08 悬停包，同一时刻）：动捕 `z = +1.011`、setpoint `z = +1.012`。
⟹ 数值是 **ENU（z 朝上）**，标注不作数。代码里的闸门是 `DATUM_GATE_M = 2.0`：
两侧 z 平均差超过它就**直接拒答**，并提示"先查 `coordinate_frame` / 锚点，别去调阈值"。

### 4. ★ 对齐搜索会把端点"夹"出来

`np.interp` 在网格越界时**静默夹取端点**。偏移一超出指令序列的覆盖范围，
"对齐"就退化成"拿一个静止点去凑"，误差假性下降。`coverage_ok` 就是那条闸门。
自检里同时钉住两档：指令只覆盖窗口时**必须**报 `False`。

### 5. ★ 自检一开始咬不住"把网格裁到共同区间"

`best_shift` 若逐档把评估网格裁到 `指令 ∩ 窗口`，报出的 `n` 会变小、误差也变低。
第一版自检**在宽覆盖的合成信号上 14 项全过**（裁掉的量是 0，缺陷完全静默）。
补了一条**窄覆盖**用例（指令只覆盖窗口）之后才咬得住。
⟹ 教训是"合成信号要把被测的那条边界**造在会被碰到的地方**"。

### 6. ★★ "两个原点"不是同一个 —— 悬停那趟差 9.2 cm（**静默**）

2026-10-17 悬停包：`control` 是 `output_mode=external`，照发 `/control/target`；
模型预测控制启动行写的是 `参考=/control/target` ⟹ **模型预测控制确实消费了它**，本包的正确口径是 `c-target`。
两侧的"轨迹原点"仍是分别抓的：`control` 的轨迹原点来自它自己的稳定判据（EKF，1 m 高度上），
模型预测控制的锚点来自闸门首开那一拍的**动捕**。实测 x 差 **0.0920 m**（−0.0690 对 −0.1611）。
若误以为这趟走的是 traj 模式、按锚点重建，水平均值会从 **0.0861** 变成 **0.0537**（低 38%），
而两个数**都"看起来正常"**，没有任何报错。那份选错口径的 JSON 保留为
`evidence/C-anchored-6_free__20261017-070103-WRONG-CALIBER.json`（**只作反例，不进任何结论**）。

⟹ 口径选择必须是**每次飞行查模型预测控制启动日志**的一条动作，不是"这个包是 external 就选 c-target"。
⟹ 反向也成立：`reference_source=traj` 时**必须**用 `t-anchored`，且锚点按
`闸门首开那一拍已收到的最新动捕` 重建 —— 用 `/traj/start` 那一刻会差约 **5 mm**（4.8–5.0 mm，系绳包实测）。

## 自检

```bash
python3 tools/ladder-baseline/test_ladder_core.py --self-test
```

判据是**合成信号上的真值对**：给一个已知的滞后看能不能收回来、给一个已知的常量偏移看指标是不是它。
另有若干条**钉住被否掉的方案与已修的坑**的用例（窗口断档上下界、网格右端点、对齐不缩网格、
扫描不跳档、`dt` 的符号、`e_z` 的符号、路径长度分不分得开慢与滞后）—— 改动它们会立刻红。

`ladder_core.py` **不 import** `rclpy` / `rosbag2_py`，也不读文件：它只吃「时刻 + 位置」两条序列。
所以自检**不需要包、不需要飞机、不需要开 ROS**。读包的适配层在 `ladder_from_bag.py`。

## 验证状态（诚实交代）

| 部分 | 状态 |
|---|---|
| `test_ladder_core.py --self-test` | ✅ **19 项全过**（新增锚点 2 项、窗口裁剪 1 项） |
| **变异测试**（主动弄坏，看会不会红） | ✅ 5 条变体逐个验过：偏移符号反、窗口首末取差、`e_z` 取绝对值、网格丢右端点、对齐裁网格 —— **各自都有用例变红** |
| 在**真机包**上端到端 | ✅ 2026-10-09 五份 (A) 级包；✅ 2026-10-17 四份 (C) 级包 |
| 口径与已发表读数一致 | ✅ `bag/old/` 两行复现了 `bag/README.md` 的 5.1/10.7 cm 与 0.371/0.592 m |
| 口径 **c**（`/mpc/reference`）在真数据上 | ⚠️ **仍未验** —— 录包正则 `^/(mavros\|mocap\|traj\|control\|vrpn)/` **不录 `/mpc/`**，实飞包里没有这个话题。代码路径在，等录包正则放开后跑 |
| 口径 **c-target**（`/control/target`）在真数据上 | ✅ 2026-10-17 悬停 / 正方形 / 八字三份包 |
| 口径 **t-anchored**（锚点 + `/traj/reference`）在真数据上 | ✅ 2026-10-17 系绳一份包；锚点按 `node_core._update_anchor` 规则重建 |
| **锚点重建**自洽性 | ✅ 系绳包末 30 s：`e_z` 均值 −5.9 mm、最大 22 mm —— 与自由飞悬停的 −3.5 mm 同量级；锚点取错约 5 mm 会让它整体平移 |
| `DATUM_GATE_M` 抓到过真错 | ⚠️ **未**。它是**事前**设的闸门，按上面第 3 条的机理写的，没在真数据上触发过 |
| 结论的**原始证据** | ✅ 已入 git：`evidence/*.json`（用**本版代码**跑出的完整读数）+ `evidence/control-state-rate.txt` + `evidence/solver-summary-20261017.json`（四趟日志收尾行逐字摘录） |

> ★ `evidence/` 的 **10 份有效 JSON**（5 份 A + 3 份 C-target + 2 份 C-anchored 系绳）是
> **逐份现算**的（2026-10-17 收尾时统一用本版代码重跑，含 `readings` 补充读数），
> 读数是本文表格的来源；另有 **1 份反例**
> （`C-anchored-6_free__20261017-070103-WRONG-CALIBER.json`，选错口径，**不进任何结论**）。
> ⚠️ 反例 JSON 是**修正前那版代码**的产物（`anchor` 里多一个 `window_rule` 键）——
> 它的价值就是"当时的错误输出长什么样"，**不要**拿它复现本文读数。
> 想复跑要先把对应的包解出来（`/tmp/z0-ladder/` 过不了重启）。
> ★★ 口径**必须按模型预测控制启动日志的 `参考=` 选**（见上文第 2 条），不能按 control 的
> `output_mode` —— 悬停那趟用错会把水平均值从 0.0861 报成 0.0537（低 38%）。

## 依赖

`numpy`；读包那一侧要 `rosbag2_py` + `rclpy` + `rosidl_runtime_py`（ROS 2 Humble，机上已有），
以及 `zstd` 命令行（`prepare_bag` 解 `.db3.zstd` 用）。`prepare_bag` 复用
[`tools/px4-sitl/mocap_model.py`](../px4-sitl/mocap_model.py) 里那份 —— 不重写，
解压 + 改 `metadata.yaml` 这三件事是已知会咬人的地方，只该有一份实现。

`ladder_core.py` 只需要 `numpy`。
