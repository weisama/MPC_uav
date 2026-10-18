# 04: 3 张姿态角图（roll / pitch / yaw × 四源）

Status: done
Type: task

**What to build:** 三条轨迹各一张姿态角图，3 行 × 四源叠加。

**Blocked by:** 01, 02

## 图的内容

每张图（悬停 / 正方形 / 8 字）：

- 3 行：roll、pitch、yaw；
- 每行四源：目标、offboard 话题包、MPC 仿真、MPC 话题包；
- 单位 deg；t 相对 `/traj/start`；窗口 = 票 01 的公共窗；
- 目标姿态按控制器约定：**roll ≡ 0、pitch ≡ 0、yaw ≡ 0**；
- 图注写明：实飞姿态取 `/mavros/vision_pose/pose`；它与 30 Hz EKF 姿态有约 **+0.03 rad**
  的滚转系统偏差，不要把实际滚转均值误读成跟踪误差；
- 图注写明：目标姿态是水平 + 纯偏航，理由见第 8 章（无阻力模型里匀速平动不需要倾角）。

文件：

- `docs/report/figures/fig-att-{hover,square,lemniscate}.png`（300 dpi）
- 同名 `.svg`

## 实现

- 脚本 `tools/report/plot_attitude.py`，只读 CSV；
- 四元数 → 欧拉角用与票 01 同一份实现，⛔ 不要在图脚本里另写一套；
- 共用 `style.py`；
- 图注进 `docs/report/generated/figure-captions.md`。

## 检查清单

- [x] 三个姿态角都有；四源齐全；单位与刻度清楚
- [x] 目标线为 0（或接近 0 的常值），并在图注说明
- [x] 仿真实姿态来自 `x0[6:10]` 四元数，⛔ 不是旧日志的 `roll/pitch/yaw` 角速率字段
- [x] 图注含 VP/EKF 偏差说明
- [x] PNG/SVG 非空；图内英文标签
- [x] 生成脚本不读原始包、不重算口径

## 验收

- 姿态图与轨迹图使用同一窗口、同一相对系、同一源；
- 完成后切 `ready-for-human`，附三张图 + 图注摘要 + 任何异常（如某源姿态缺失）。

---

## Comments

### 2026-10-18 —— 三张姿态图已生成，转 ready-for-human

**产物**

| 图 | 文件 | 网格点/源 | 公共窗 |
|---|---|---|---|
| A1 悬停 | `docs/report/figures/fig-att-hover.{png,svg}` | 3966 | 0 - 39.6494 s |
| A2 正方形 | `docs/report/figures/fig-att-square.{png,svg}` | 802 | 0 - 8.0096 s |
| A3 8 字 | `docs/report/figures/fig-att-lemniscate.{png,svg}` | 1222 | 0 - 12.2021 s |

- 脚本：`tools/report/plot_attitude.py`；只读 `docs/report/data/*.csv`，
  图注的来源与窗口读同名 `.json`；四元数 → 欧拉角复用 `align.quat_to_euler`，没有另写一套。
- 图注：`docs/report/generated/figure-captions.md` 的 `phase5-att:<scene>` 块（每场景一块）；
  与轨迹/误差/控制量图注分块共存，部分重跑不互相覆盖。
- 重跑：`python3 tools/report/plot_attitude.py --scene all`（约 3 s）。
- 测试：`python3 -m pytest tools/report/test_report_attitude_figures.py -q` → **10 passed**；
  报告工具全量 `python3 -m pytest tools/report -q` → **144 passed**（2026-10-18 快照）。

**读图口径**

1. 三行分别是 roll / pitch / yaw，单位 deg；目标恒 0（水平 + 纯偏航），黑色虚线。
2. 实飞姿态取 `/mavros/vision_pose/pose`；它与 30 Hz EKF 有约 +0.03 rad 的滚转零偏，
   图注已写明，不要把 roll 均值读成跟踪误差。
3. 仿真姿态取 `x0[6:10]` 四元数解算；旧日志里叫 `roll/pitch/yaw` 的字段是角速率，未被使用。
4. 窗口与相对系与轨迹图完全一致（票 01 的公共窗、`/traj/start` 时间零点、各自轨迹原点）。

**需要操作者看的两件事**

1. 仿真源的 yaw 从首帧起就在 90° 附近、整场没有转到 0；票 01 已把它记为数据读数、未判定原因。
   图里按要求保留，没有扣掉。若希望把这条偏置从图或指标里剔除，需要另行裁定口径。
2. 实飞 roll 的 +0.03 rad 零偏同样保留在原始读数里；是否在姿态图里扣掉，也请一并裁定。

**变异自检**

- 把目标线改成实线 ⟹ `test_target_is_a_dashed_zero_line` 变红；
- 把四源颜色改成同一色 ⟹ `test_four_sources_are_drawn` 变红；
- 图注删掉 `+0.03 rad` ⟹ `test_captions_state_attitude_conventions` 变红。

**未做/边界**

- 没有改票 01 的数据与口径，没有碰控制代码，没有读原始包。
- 只生成票面要求的三张图；9 张 pitch/roll 图由票 03 负责，本票未改。

### 2026-10-18 —— 操作者裁定收工

操作者 2026-10-18 指令「全部 tickets 收工」，并对 README 修订与装配结果放行；本票按「做完了」义把 `Status` 转 `done`。收口过程见 [票 20](20-closeout-and-v5.md)。
