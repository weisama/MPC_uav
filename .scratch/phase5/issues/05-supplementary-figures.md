# 05: 补充图（位置误差–时间、控制量）

Status: ready-for-human
Type: task

**What to build:** 两张补充图族：位置误差–时间（四源）与控制量（模型预测控制仿真 vs 实飞两源）。

**Blocked by:** 01, 02

## 图 1：位置误差–时间

- 每场景一张：`docs/report/figures/fig-error-{hover,square,lemniscate}.png/.svg`；
- 3 行：ex、ey、ez；每行四源叠加；
- 误差 = 实际 − 目标（轨迹原点相对系）；单位 m；窗口 = 公共窗；
- 图注写目标定义与窗口。

## 图 2：控制量

- 每场景一张：`docs/report/figures/fig-control-{hover,square,lemniscate}.png/.svg`；
- 4 行：推力（N）、ωx、ωy、ωz；
- **只画两源**：模型预测控制仿真、模型预测控制实飞；
- ⛔ 不把 offboard 的 `/mavros/setpoint_raw/local`（位置设定点）混进来——它和推力/角速率不是同一层，
  画在一起会造成"看起来可比"的假象；
- 实飞推力换算口径写进图注（`thrust` / `thrust_body_z` 与归一化常数，按该趟日志）。

## 实现

- `tools/report/plot_error.py`、`tools/report/plot_control.py`；
- 只读 CSV；共用 `style.py`；
- 图注进 `docs/report/generated/figure-captions.md`。

## 检查清单

- [x] 误差图四源齐全，符号约定写清（实际 − 目标）
- [x] 控制量图只有两源，并在图注说明为什么 offboard 不在图内
- [x] 推力单位与换算口径可追溯
- [x] 图内英文标签、图题中文；PNG/SVG 非空
- [x] 与轨迹/姿态图共用窗口与相对系

## 验收

- 两张图族能直接支撑第 10 章的"跟踪误差"与"控制量"两节；
- 完成后切 `ready-for-human`，附图 + 口径摘要 + 任何缺失源。

---

## Comments

### 2026-10-18 —— 两个图族已生成，转 ready-for-human

**产物**

| 图族 | 文件 | 脚本 | 测试 |
|---|---|---|---|
| E1–E3 位置误差–时间 | `docs/report/figures/fig-error-{hover,square,lemniscate}.{png,svg}` | `tools/report/plot_error.py` | `test_report_supplementary_figures.py` |
| C1–C3 控制量 | `docs/report/figures/fig-control-{hover,square,lemniscate}.{png,svg}` | `tools/report/plot_control.py` | 同上（共 19 条） |

- 重跑：`python3 tools/report/plot_error.py --scene all`、
  `python3 tools/report/plot_control.py --scene all`。
- 图注：`docs/report/generated/figure-captions.md` 的 `phase5-error:<scene>` 与
  `phase5-control:<scene>` 块；分块替换，部分重跑不会覆盖轨迹/姿态图注。
- 测试：`python3 -m pytest tools/report/test_report_supplementary_figures.py -q` → **19 passed**；
  报告工具全量 **144 passed**（2026-10-18 快照）。

**读数口径**

1. 误差 = 实际 − 目标，逐轴 `ex/ey/ez`，单位 m；四个源是 target、offboard、
   mpc_sim、mpc_real。target 行的误差恒为 0（黑虚线），这是符号约定的自检线。
2. 控制量只画 mpc_sim 与 mpc_real。offboard 的
   `/mavros/setpoint_raw/local` 是位置设定点，与推力/角速率不是同一层，
   混画会让人误以为两组控制器在同一层可比；图注已写明。
3. 推力单位 N，由日志里的归一化推力乘 `u_conversion.t_max_n` 得到；本批三场景三趟
   均读自同名 JSON，值见各图图注。图上是零阶保持台阶，不是插值曲线。
4. 窗口、时间零点、原点相对系与轨迹/姿态图完全一致。

**数据里值得注意的读数（截图自 CSV，2026-10-18）**

- 正方形与 8 字的误差以**滞后型**为主：offboard 的 ex 极值约 ±0.7 m、
  mpc_sim 的 ey 到约 −0.83 m，正负号随运动方向翻转；悬停则贴着零。
- mpc_sim 在正方形/8 字的垂直误差明显大于 mpc_real（8 字 ez 到 0.25 m），
  与第 10 章边界一节记录的仿真时间轴/求解条件差异一致，未做归因。
- 控制量里 mpc_real 的推力均值比 mpc_sim 高约 1.1 N；这是悬停与两场景一致的**读数**，
  不等于控制器给出的力不同——实飞的推力换算常数与机体质量口径需按第 9 章读。

**变异自检**

- 把误差符号改成目标 − 实际 ⟹ 符号对照测试变红；
- 把推力换算常数 `t_max` 改掉 ⟹ 与 JSON 动态比对的测试变红；
- 往控制图里加 offboard 源 ⟹ “只有两源/无蓝色”测试变红。

**未做/边界**

- 只读 `docs/report/data/*.csv` 与同名 JSON，不读原始包、不重算管线口径。
- 没有改动票 01/02 的数据与指标；没有生成 3D 图。
