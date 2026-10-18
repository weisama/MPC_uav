# 03: 3 张轨迹图（XY 俯视 + Z–t，含 3D 候选）

Status: ready-for-human
Type: task

**What to build:** 三张轨迹对比图，每张两个子图；外加三张 3D 候选图（不进正文）。

**Blocked by:** 01, 02

## 图的内容

每张图（悬停 / 正方形 / 8 字）：

- **左：XY 俯视**，等比例；四源叠加：
  目标（虚线）、offboard 话题包、模型预测控制仿真、模型预测控制话题包；
- **右：Z–t**，四源叠加，t 相对 `/traj/start`；
- 起点用同一标记；图例英文；单位 m；
- 窗口 = 票 01 的公共窗（悬停 0–39.65 s；正方形/8 字到完成）。

### 2026-10-18 追加：12 张 pitch/roll 图（操作者要求）

3 个场景 × 4 个源 = **12 种情况，每种情况一张图**：

- 每张图含 **Pitch、Roll 两条曲线 + 一条 0 deg 零线**（不是一张只画一个量，也不是 24 张）；
- 单位 deg；横轴 `t [s]`，相对 `/traj/start`；图内标签英文；
- **纵轴 12 张统一**：由全部 12 组数据现算，以 0 为中心、向上取整后留 1 deg 余量
  （当前数据快照为 −14 到 +14 deg；换数据重跑会自动变，不写死）；
- **横轴同场景 4 张统一**：`0` 到该场景公共窗终点；不同场景窗口本来就不同；
- 目标姿态按控制器约定恒 0，所以 `*-target` 图的两条线与零线重合。

文件：

- `docs/report/figures/fig-traj-{hover,square,lemniscate}.png`（300 dpi）
- 同名 `.svg`
- `docs/report/figures/candidates/fig-traj-{scene}-3d.png/.svg`：四源 3D 轨迹，**不放进正文**
- `docs/report/figures/fig-pitchroll-{scene}-{source}.png/.svg`：12 张 pitch/roll 图
  （`scene` = hover/square/lemniscate；`source` = target/offboard/mpc_sim/mpc_real）

## 实现

- 脚本 `tools/report/plot_traj.py`，只读 `docs/report/data/*.csv`；
- 样式集中在一个 `style.py`（线型、颜色、字号、图幅），四类图共用；
- 图注生成到 `docs/report/generated/figure-captions.md`，含场景、窗口、来源、生成命令；
- 3D 候选单独目录，脚本里用 `--candidates` 开关，默认不生成。
- pitch/roll 图脚本 `tools/report/plot_pitch_roll.py`；12 张共用同一纵轴；
  图注按 `phase5-pitchroll:<scene>:<source>` 分块，与轨迹图注互不覆盖。

## 检查清单

- [x] 每张图的四源齐全；目标用虚线、实际用实线；XY 等比例
- [x] 悬停图注明"offboard 包截止 39.65 s（录制截断，不是控制失败）"
- [x] 图内标签英文、图题中文；不依赖中文字体
- [x] PNG 300 dpi、SVG 可解析；文件非空
- [x] 生成脚本只读 CSV，不读原始包（图注另读票 01 的同名 JSON 取来源与窗口；未读任何包）
- [x] 3D 候选没有被任何章节文件引用
- [x] 图注含来源与窗口
- [x] 12 张 pitch/roll 图齐全（3 场景 × 4 源），每张含 pitch、roll 与零线
- [x] 12 张图纵轴范围完全一致；同场景 4 张横轴一致
- [x] 目标 4 张图的 pitch/roll 恒 0；图注写明统一范围与姿态口径
- [x] pitch/roll 图注与轨迹图注分块共存，部分重跑不互相覆盖

## 验收

- 操作者能只看图判断四源是否同口径（起点、窗口、相对系一致）；
- 12 张 pitch/roll 图翻看时**不需要重新读纵轴数值**：范围一致、零线位置一致；
- 3D 候选交操作者看图后决定去留；去留决定写进票的 `## Comments`；
- 完成后切 `ready-for-human`，附三张图 + 候选图 + 图注摘要。

---

## Comments

### 2026-10-17 —— 三张图 + 三张候选已生成，转 ready-for-human

**产物**

| 图 | 文件 | 网格点/源 | 公共窗 |
|---|---|---|---|
| T1 悬停 | `docs/report/figures/fig-traj-hover.{png,svg}` | 3966 | 0 - 39.6494 s |
| T2 正方形 | `docs/report/figures/fig-traj-square.{png,svg}` | 802 | 0 - 8.0096 s |
| T3 8 字 | `docs/report/figures/fig-traj-lemniscate.{png,svg}` | 1222 | 0 - 12.2021 s |
| T1-3D / T2-3D / T3-3D | `docs/report/figures/candidates/fig-traj-*-3d.{png,svg}` | 同上 | 同上 |

- 图注：`docs/report/generated/figure-captions.md`（正文）；
  `docs/report/generated/figure-captions-candidates.md`（候选，单独成文件）。
- 脚本：`tools/report/plot_traj.py`；共用样式 `tools/report/style.py`。
- 重跑：`python3 tools/report/plot_traj.py --scene all --candidates`（约 7 s）。
- 测试：`python3 -m pytest tools/report -q` → **78 passed**（票 01 的 61 条 + 本票 17 条）。
- 同一输入连跑两次，`docs/report/figures/**` 与 `docs/report/generated/**` 的 sha256 逐字节一致。

**读图口径**（验收时先看这四条）

1. 四源都画在**轨迹原点相对系**：各自减去 `AUTO/WAIT_START` 期间的指令值；
   浅灰 x=0 / y=0 / z=0 线就是各自原点。四源起点用同一种空心圆。
2. 目标虚线、三条实际轨迹实线；左图 `set_aspect("equal")`，右图横轴从 0 开始。
3. 时间零点统一为 `/traj/start`；窗口右端取四源完成时刻最小值。
4. 悬停图注写明：offboard 包截止 39.65 s（录制截断，不是控制失败）。

**需要操作者裁定的两件事**

1. **3D 候选去留**：三张候选图已生成，但**没有**进正文，也没有被任何章节文件引用
   （测试会扫 `docs/report/chapters/*.md`）。请看完决定保留/删除/进正文；决定写回本票。
2. **`Blocked by: 01` 的状态**：票 01 目前仍是 `ready-for-human`。本票的图用的是票 01
   已提交的 CSV/JSON（输入 sha256 在 `data/<scene>.json`）；若票 01 复审后改口径，图需要重跑。

**与票面字面不一致的三处（请过目）**

1. 票面写「只读 CSV」；实现上**画图只读 CSV**，图注另读票 01 同目录的 `<scene>.json`
   取窗口与来源（包路径、sim summary、sha256）。没有读任何原始包。若坚持字面只读 CSV，
   来源字符串只能写死在脚本里，会随票 01 重跑过期。
2. 3D 候选图注**单独一个文件**，没有混进 `figure-captions.md`：装配票 16 会内联后者，
   混在一起等于把候选带进正文。
3. 本机有两套 matplotlib（Debian 3.5.1 + pip 3.10.9），Debian 的 `mpl_toolkits` 会遮住
   pip 的 namespace package，导致 3D 投影不可用。`plot_traj.py` 里加了一个**只影响本进程**的
   `sys.modules` 修补（`_install_mpl_toolkits_shim`），没有改环境、没有装依赖。

**未做/边界**

- 没改票 01/02 的数据与口径，没碰控制代码，没读原始包。
- 3D 候选的三个坐标轴按各自数据范围缩放（不保证等比例），只用于看形状；图注已写明。
- `figure-captions.md` 按场景分块（`phase5-traj:<scene>`），部分重跑不会抹掉其他场景的图注。

### 2026-10-18 —— 追加 12 张 pitch/roll 图，坐标轴统一

**口径（按操作者选择）**

- 12 种情况 = 3 场景 × 4 源，每种情况**一张图**；每张图两条线（Pitch / Roll）+ 零线，
  不是一张只画一个量，也不是 24 张。
- **纵轴**：12 张完全共用。范围由全部 12 组 pitch/roll 数据现算：
  `limit = ceil(max|angle|) + 1 deg`，对称到 `[-limit, +limit]`。
  当前数据快照（2026-10-18）为 **−14 到 +14 deg**；代码里没有写死这个数，
  换数据重跑会自动变。最大绝对值出现在 8 字仿真的 pitch（−12.05 deg）。
- **横轴**：同场景 4 张共用 `0` 到公共窗终点；三场景分别为 39.6494 / 8.0096 /
  12.2021 s。窗口取自票 01 的 `<scene>.json`，绘图仍只读 CSV。
- 单位 deg，图内英文（`Pitch` / `Roll` / `Zero` / `angle [deg]`）。

**产物**

- 图：`docs/report/figures/fig-pitchroll-{hover,square,lemniscate}-{target,offboard,mpc_sim,mpc_real}.{png,svg}`（12 × 2 个文件）
- 图注：`docs/report/generated/figure-captions.md` 的 `phase5-pitchroll:<scene>:<source>` 块；
  每块含窗口、来源、统一纵轴范围、横轴范围、姿态口径与生成命令。
- 脚本：`tools/report/plot_pitch_roll.py`；图注分块写入抽到 `tools/report/captions.py`
  （轨迹图脚本改为共用，行为不变）。
- 重跑：`python3 tools/report/plot_pitch_roll.py --scene all`（约 6 s）；
  只重画一个场景时纵轴仍用全部 12 组数据定，不会缩成该场景自身范围。
- 测试：`source /opt/ros/humble/setup.bash && python3 -m pytest tools/report -q`
  → **96 passed**，其中 `test_report_pitch_roll_figures.py` 新增 **14 条**。
- 连跑两次，12 张图与图注的 sha256 逐字节一致。

**读图口径（验收时先看这三条）**

1. 纵轴刻度 12 张完全相同（−10/−5/0/5/10）；零线永远在正中。翻图不必重读刻度。
2. `*-target` 图 pitch/roll 恒 0，两条线与零线重合（目标姿态按控制器约定水平 +
   零偏航），这是预期，不是画错或数据缺失。
3. 实飞 roll 有约 +0.03 rad（≈1.7 deg）的系统零偏（VP 约 190 Hz vs EKF 30 Hz），
   图注已写明；不要把这条零偏读成跟踪误差。

**需要操作者看的两件事**

1. 12 张图的切分与"每张两条线 + 零线"是否符合预期；
2. 纵轴 ±14 deg 是否合适：它由当前数据最大绝对值 12.05 deg 外扩而来；
   若希望更紧凑（例如 ±13）或留更多余量，改 `compute_angle_limits` 一处即可。

**未做/边界**

- 没有扩展票 01/02：CSV 里四源的 `roll_rad`/`pitch_rad` 本来就齐全，本次只动绘图与图注。
- `plot_traj.load_scene` 的必填列扩为 `t,x,y,z,roll,pitch,yaw`；CSV 不需重生成。
- 轨迹图与 3D 候选未改动；3D 去留仍待操作者裁定。
