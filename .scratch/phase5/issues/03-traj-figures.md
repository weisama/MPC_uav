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

文件：

- `docs/report/figures/fig-traj-{hover,square,lemniscate}.png`（300 dpi）
- 同名 `.svg`
- `docs/report/figures/candidates/fig-traj-{scene}-3d.png/.svg`：四源 3D 轨迹，**不放进正文**

## 实现

- 脚本 `tools/report/plot_traj.py`，只读 `docs/report/data/*.csv`；
- 样式集中在一个 `style.py`（线型、颜色、字号、图幅），四类图共用；
- 图注生成到 `docs/report/generated/figure-captions.md`，含场景、窗口、来源、生成命令；
- 3D 候选单独目录，脚本里用 `--candidates` 开关，默认不生成。

## 检查清单

- [x] 每张图的四源齐全；目标用虚线、实际用实线；XY 等比例
- [x] 悬停图注明"offboard 包截止 39.65 s（录制截断，不是控制失败）"
- [x] 图内标签英文、图题中文；不依赖中文字体
- [x] PNG 300 dpi、SVG 可解析；文件非空
- [x] 生成脚本只读 CSV，不读原始包（图注另读票 01 的同名 JSON 取来源与窗口；未读任何包）
- [x] 3D 候选没有被任何章节文件引用
- [x] 图注含来源与窗口

## 验收

- 操作者能只看图判断四源是否同口径（起点、窗口、相对系一致）；
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
