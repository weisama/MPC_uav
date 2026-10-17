# 图注（自动生成）

> 本文件由报告图脚本生成，**不要手改**。脚本只替换对应的
> `<!-- BEGIN ... -->` / `<!-- END ... -->` 块，不会动别的票写的块。

<!-- BEGIN phase5-traj:hover -->
### T1 悬停（hover）轨迹：四源对比

- 文件：`figures/fig-traj-hover.png`、`figures/fig-traj-hover.svg`（300 dpi）
- 数据：`data/hover.csv`（同名 `.json` 记输入 sha256 与窗口口径）
- 窗口：0.0000 - 39.6494 s（四源完成时刻取最小：mpc_sim = 39.649415 s）
- 来源：目标与 offboard 来自 `bag/3_traj1_hover`；模型预测控制仿真来自 `tools/report/evidence/sitl/R-hover/R-hover-summary.json`；模型预测控制实飞来自 `bag/6_free__20261017-070103`。
- 坐标：四源各自减去自身的轨迹原点（`AUTO/WAIT_START` 期间的指令值），单位为 m；时间零点为 `/traj/start` 的包记录时间戳。
- 实际轨迹：实飞两源取 `/mavros/vision_pose/pose`；仿真源取 SITL 的 `x0`（合成动捕）。offboard 与模型预测控制两组控制器看到的反馈源不同，详见 `data/README.md`。
- 样式：目标为虚线，实际轨迹为实线；四源起点用同一种空心圆标记；左图 XY 等比例俯视，右图为高度-时间。
- 生成：`python3 tools/report/plot_traj.py --scene all --candidates`
- 注意：offboard 包截止 39.65 s（录制截断，不是控制失败）。
- 裁剪口径：公共窗右端取各源最小：mpc_sim = 39.649415 s；录制截断：progress 终值 0.661 < 1.0；截止到参考流末尾 39.650080 s（包尾 39.661156 s）
<!-- END phase5-traj:hover -->

<!-- BEGIN phase5-traj:square -->
### T2 正方形（square）轨迹：四源对比

- 文件：`figures/fig-traj-square.png`、`figures/fig-traj-square.svg`（300 dpi）
- 数据：`data/square.csv`（同名 `.json` 记输入 sha256 与窗口口径）
- 窗口：0.0000 - 8.0096 s（四源完成时刻取最小：mpc_real = 8.009599 s）
- 来源：目标与 offboard 来自 `bag/4_traj2_square`；模型预测控制仿真来自 `tools/report/evidence/sitl/R-square/R-square-summary.json`；模型预测控制实飞来自 `bag/6_free__20261017-070446`。
- 坐标：四源各自减去自身的轨迹原点（`AUTO/WAIT_START` 期间的指令值），单位为 m；时间零点为 `/traj/start` 的包记录时间戳。
- 实际轨迹：实飞两源取 `/mavros/vision_pose/pose`；仿真源取 SITL 的 `x0`（合成动捕）。offboard 与模型预测控制两组控制器看到的反馈源不同，详见 `data/README.md`。
- 样式：目标为虚线，实际轨迹为实线；四源起点用同一种空心圆标记；左图 XY 等比例俯视，右图为高度-时间。
- 生成：`python3 tools/report/plot_traj.py --scene all --candidates`
<!-- END phase5-traj:square -->

<!-- BEGIN phase5-traj:lemniscate -->
### T3 8 字（lemniscate）轨迹：四源对比

- 文件：`figures/fig-traj-lemniscate.png`、`figures/fig-traj-lemniscate.svg`（300 dpi）
- 数据：`data/lemniscate.csv`（同名 `.json` 记输入 sha256 与窗口口径）
- 窗口：0.0000 - 12.2021 s（四源完成时刻取最小：mpc_real = 12.202111 s）
- 来源：目标与 offboard 来自 `bag/5_traj3_lemniscate`；模型预测控制仿真来自 `tools/report/evidence/sitl/R-lemniscate/R-lemniscate-summary.json`；模型预测控制实飞来自 `bag/6_free__20261017-070935`。
- 坐标：四源各自减去自身的轨迹原点（`AUTO/WAIT_START` 期间的指令值），单位为 m；时间零点为 `/traj/start` 的包记录时间戳。
- 实际轨迹：实飞两源取 `/mavros/vision_pose/pose`；仿真源取 SITL 的 `x0`（合成动捕）。offboard 与模型预测控制两组控制器看到的反馈源不同，详见 `data/README.md`。
- 样式：目标为虚线，实际轨迹为实线；四源起点用同一种空心圆标记；左图 XY 等比例俯视，右图为高度-时间。
- 生成：`python3 tools/report/plot_traj.py --scene all --candidates`
<!-- END phase5-traj:lemniscate -->
