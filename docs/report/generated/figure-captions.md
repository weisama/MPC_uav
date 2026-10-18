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

<!-- BEGIN phase5-pitchroll:hover:target -->
### PR-hover-target 悬停 · 目标：pitch 与 roll

- 文件：`figures/fig-pitchroll-hover-target.png`、`figures/fig-pitchroll-hover-target.svg`（300 dpi）
- 数据：`data/hover.csv`
- 窗口：0.0000 - 39.6494 s（公共窗右端取各源最小：mpc_sim = 39.649415 s；录制截断：progress 终值 0.661 < 1.0；截止到参考流末尾 39.650080 s（包尾 39.661156 s））
- 统一坐标轴：纵轴 -14 到 14 deg（12 张图共用，以 0 为中心）；横轴 0 - 39.6494 s（同场景 4 张共用）；零线为 0 deg。
- 来源：`bag/3_traj1_hover` 的 `/traj/reference`（相对量 + 轨迹原点；姿态按控制器约定取水平 + 零偏航）。
- 姿态口径：目标姿态按控制器约定取水平 + 零偏航，因此 pitch 与 roll 恒为 0，两条曲线与零线重合。
- 生成：`python3 tools/report/plot_pitch_roll.py --scene all`
- 注意：offboard 包截止 39.65 s（录制截断，不是控制失败）。
<!-- END phase5-pitchroll:hover:target -->

<!-- BEGIN phase5-pitchroll:hover:offboard -->
### PR-hover-offboard 悬停 · offboard 实飞：pitch 与 roll

- 文件：`figures/fig-pitchroll-hover-offboard.png`、`figures/fig-pitchroll-hover-offboard.svg`（300 dpi）
- 数据：`data/hover.csv`
- 窗口：0.0000 - 39.6494 s（公共窗右端取各源最小：mpc_sim = 39.649415 s；录制截断：progress 终值 0.661 < 1.0；截止到参考流末尾 39.650080 s（包尾 39.661156 s））
- 统一坐标轴：纵轴 -14 到 14 deg（12 张图共用，以 0 为中心）；横轴 0 - 39.6494 s（同场景 4 张共用）；零线为 0 deg。
- 来源：`bag/3_traj1_hover` 的 `/mavros/vision_pose/pose`。
- 姿态口径：实飞姿态取 `/mavros/vision_pose/pose`；它与 30 Hz EKF 姿态有约 +0.03 rad 的滚转系统偏差，不要把滚转零偏读成跟踪误差。
- 生成：`python3 tools/report/plot_pitch_roll.py --scene all`
- 注意：offboard 包截止 39.65 s（录制截断，不是控制失败）。
<!-- END phase5-pitchroll:hover:offboard -->

<!-- BEGIN phase5-pitchroll:hover:mpc_sim -->
### PR-hover-mpc_sim 悬停 · 模型预测控制仿真：pitch 与 roll

- 文件：`figures/fig-pitchroll-hover-mpc_sim.png`、`figures/fig-pitchroll-hover-mpc_sim.svg`（300 dpi）
- 数据：`data/hover.csv`
- 窗口：0.0000 - 39.6494 s（公共窗右端取各源最小：mpc_sim = 39.649415 s；录制截断：progress 终值 0.661 < 1.0；截止到参考流末尾 39.650080 s（包尾 39.661156 s））
- 统一坐标轴：纵轴 -14 到 14 deg（12 张图共用，以 0 为中心）；横轴 0 - 39.6494 s（同场景 4 张共用）；零线为 0 deg。
- 来源：`tools/report/evidence/sitl/R-hover/R-hover-summary.json` 的 `x0` 四元数。
- 姿态口径：仿真姿态由 `x0[6:10]` 四元数解算；旧日志里名叫 roll/pitch/yaw 的字段是角速率，不能当姿态角。
- 生成：`python3 tools/report/plot_pitch_roll.py --scene all`
- 注意：offboard 包截止 39.65 s（录制截断，不是控制失败）。
<!-- END phase5-pitchroll:hover:mpc_sim -->

<!-- BEGIN phase5-pitchroll:hover:mpc_real -->
### PR-hover-mpc_real 悬停 · 模型预测控制实飞：pitch 与 roll

- 文件：`figures/fig-pitchroll-hover-mpc_real.png`、`figures/fig-pitchroll-hover-mpc_real.svg`（300 dpi）
- 数据：`data/hover.csv`
- 窗口：0.0000 - 39.6494 s（公共窗右端取各源最小：mpc_sim = 39.649415 s；录制截断：progress 终值 0.661 < 1.0；截止到参考流末尾 39.650080 s（包尾 39.661156 s））
- 统一坐标轴：纵轴 -14 到 14 deg（12 张图共用，以 0 为中心）；横轴 0 - 39.6494 s（同场景 4 张共用）；零线为 0 deg。
- 来源：`bag/6_free__20261017-070103` 的 `/mavros/vision_pose/pose`。
- 姿态口径：实飞姿态取 `/mavros/vision_pose/pose`；它与 30 Hz EKF 姿态有约 +0.03 rad 的滚转系统偏差，不要把滚转零偏读成跟踪误差。
- 生成：`python3 tools/report/plot_pitch_roll.py --scene all`
- 注意：offboard 包截止 39.65 s（录制截断，不是控制失败）。
<!-- END phase5-pitchroll:hover:mpc_real -->

<!-- BEGIN phase5-pitchroll:square:target -->
### PR-square-target 正方形 · 目标：pitch 与 roll

- 文件：`figures/fig-pitchroll-square-target.png`、`figures/fig-pitchroll-square-target.svg`（300 dpi）
- 数据：`data/square.csv`
- 窗口：0.0000 - 8.0096 s（公共窗右端取各源最小：mpc_real = 8.009599 s；`/traj/progress` 首次 ≥ 1.0：8.021292 s）
- 统一坐标轴：纵轴 -14 到 14 deg（12 张图共用，以 0 为中心）；横轴 0 - 8.0096 s（同场景 4 张共用）；零线为 0 deg。
- 来源：`bag/4_traj2_square` 的 `/traj/reference`（相对量 + 轨迹原点；姿态按控制器约定取水平 + 零偏航）。
- 姿态口径：目标姿态按控制器约定取水平 + 零偏航，因此 pitch 与 roll 恒为 0，两条曲线与零线重合。
- 生成：`python3 tools/report/plot_pitch_roll.py --scene all`
<!-- END phase5-pitchroll:square:target -->

<!-- BEGIN phase5-pitchroll:square:offboard -->
### PR-square-offboard 正方形 · offboard 实飞：pitch 与 roll

- 文件：`figures/fig-pitchroll-square-offboard.png`、`figures/fig-pitchroll-square-offboard.svg`（300 dpi）
- 数据：`data/square.csv`
- 窗口：0.0000 - 8.0096 s（公共窗右端取各源最小：mpc_real = 8.009599 s；`/traj/progress` 首次 ≥ 1.0：8.021292 s）
- 统一坐标轴：纵轴 -14 到 14 deg（12 张图共用，以 0 为中心）；横轴 0 - 8.0096 s（同场景 4 张共用）；零线为 0 deg。
- 来源：`bag/4_traj2_square` 的 `/mavros/vision_pose/pose`。
- 姿态口径：实飞姿态取 `/mavros/vision_pose/pose`；它与 30 Hz EKF 姿态有约 +0.03 rad 的滚转系统偏差，不要把滚转零偏读成跟踪误差。
- 生成：`python3 tools/report/plot_pitch_roll.py --scene all`
<!-- END phase5-pitchroll:square:offboard -->

<!-- BEGIN phase5-pitchroll:square:mpc_sim -->
### PR-square-mpc_sim 正方形 · 模型预测控制仿真：pitch 与 roll

- 文件：`figures/fig-pitchroll-square-mpc_sim.png`、`figures/fig-pitchroll-square-mpc_sim.svg`（300 dpi）
- 数据：`data/square.csv`
- 窗口：0.0000 - 8.0096 s（公共窗右端取各源最小：mpc_real = 8.009599 s；`/traj/progress` 首次 ≥ 1.0：8.021292 s）
- 统一坐标轴：纵轴 -14 到 14 deg（12 张图共用，以 0 为中心）；横轴 0 - 8.0096 s（同场景 4 张共用）；零线为 0 deg。
- 来源：`tools/report/evidence/sitl/R-square/R-square-summary.json` 的 `x0` 四元数。
- 姿态口径：仿真姿态由 `x0[6:10]` 四元数解算；旧日志里名叫 roll/pitch/yaw 的字段是角速率，不能当姿态角。
- 生成：`python3 tools/report/plot_pitch_roll.py --scene all`
<!-- END phase5-pitchroll:square:mpc_sim -->

<!-- BEGIN phase5-pitchroll:square:mpc_real -->
### PR-square-mpc_real 正方形 · 模型预测控制实飞：pitch 与 roll

- 文件：`figures/fig-pitchroll-square-mpc_real.png`、`figures/fig-pitchroll-square-mpc_real.svg`（300 dpi）
- 数据：`data/square.csv`
- 窗口：0.0000 - 8.0096 s（公共窗右端取各源最小：mpc_real = 8.009599 s；`/traj/progress` 首次 ≥ 1.0：8.021292 s）
- 统一坐标轴：纵轴 -14 到 14 deg（12 张图共用，以 0 为中心）；横轴 0 - 8.0096 s（同场景 4 张共用）；零线为 0 deg。
- 来源：`bag/6_free__20261017-070446` 的 `/mavros/vision_pose/pose`。
- 姿态口径：实飞姿态取 `/mavros/vision_pose/pose`；它与 30 Hz EKF 姿态有约 +0.03 rad 的滚转系统偏差，不要把滚转零偏读成跟踪误差。
- 生成：`python3 tools/report/plot_pitch_roll.py --scene all`
<!-- END phase5-pitchroll:square:mpc_real -->

<!-- BEGIN phase5-pitchroll:lemniscate:target -->
### PR-lemniscate-target 8 字 · 目标：pitch 与 roll

- 文件：`figures/fig-pitchroll-lemniscate-target.png`、`figures/fig-pitchroll-lemniscate-target.svg`（300 dpi）
- 数据：`data/lemniscate.csv`
- 窗口：0.0000 - 12.2021 s（公共窗右端取各源最小：mpc_real = 12.202111 s；`/traj/progress` 首次 ≥ 1.0：12.213041 s）
- 统一坐标轴：纵轴 -14 到 14 deg（12 张图共用，以 0 为中心）；横轴 0 - 12.2021 s（同场景 4 张共用）；零线为 0 deg。
- 来源：`bag/5_traj3_lemniscate` 的 `/traj/reference`（相对量 + 轨迹原点；姿态按控制器约定取水平 + 零偏航）。
- 姿态口径：目标姿态按控制器约定取水平 + 零偏航，因此 pitch 与 roll 恒为 0，两条曲线与零线重合。
- 生成：`python3 tools/report/plot_pitch_roll.py --scene all`
<!-- END phase5-pitchroll:lemniscate:target -->

<!-- BEGIN phase5-pitchroll:lemniscate:offboard -->
### PR-lemniscate-offboard 8 字 · offboard 实飞：pitch 与 roll

- 文件：`figures/fig-pitchroll-lemniscate-offboard.png`、`figures/fig-pitchroll-lemniscate-offboard.svg`（300 dpi）
- 数据：`data/lemniscate.csv`
- 窗口：0.0000 - 12.2021 s（公共窗右端取各源最小：mpc_real = 12.202111 s；`/traj/progress` 首次 ≥ 1.0：12.213041 s）
- 统一坐标轴：纵轴 -14 到 14 deg（12 张图共用，以 0 为中心）；横轴 0 - 12.2021 s（同场景 4 张共用）；零线为 0 deg。
- 来源：`bag/5_traj3_lemniscate` 的 `/mavros/vision_pose/pose`。
- 姿态口径：实飞姿态取 `/mavros/vision_pose/pose`；它与 30 Hz EKF 姿态有约 +0.03 rad 的滚转系统偏差，不要把滚转零偏读成跟踪误差。
- 生成：`python3 tools/report/plot_pitch_roll.py --scene all`
<!-- END phase5-pitchroll:lemniscate:offboard -->

<!-- BEGIN phase5-pitchroll:lemniscate:mpc_sim -->
### PR-lemniscate-mpc_sim 8 字 · 模型预测控制仿真：pitch 与 roll

- 文件：`figures/fig-pitchroll-lemniscate-mpc_sim.png`、`figures/fig-pitchroll-lemniscate-mpc_sim.svg`（300 dpi）
- 数据：`data/lemniscate.csv`
- 窗口：0.0000 - 12.2021 s（公共窗右端取各源最小：mpc_real = 12.202111 s；`/traj/progress` 首次 ≥ 1.0：12.213041 s）
- 统一坐标轴：纵轴 -14 到 14 deg（12 张图共用，以 0 为中心）；横轴 0 - 12.2021 s（同场景 4 张共用）；零线为 0 deg。
- 来源：`tools/report/evidence/sitl/R-lemniscate/R-lemniscate-summary.json` 的 `x0` 四元数。
- 姿态口径：仿真姿态由 `x0[6:10]` 四元数解算；旧日志里名叫 roll/pitch/yaw 的字段是角速率，不能当姿态角。
- 生成：`python3 tools/report/plot_pitch_roll.py --scene all`
<!-- END phase5-pitchroll:lemniscate:mpc_sim -->

<!-- BEGIN phase5-pitchroll:lemniscate:mpc_real -->
### PR-lemniscate-mpc_real 8 字 · 模型预测控制实飞：pitch 与 roll

- 文件：`figures/fig-pitchroll-lemniscate-mpc_real.png`、`figures/fig-pitchroll-lemniscate-mpc_real.svg`（300 dpi）
- 数据：`data/lemniscate.csv`
- 窗口：0.0000 - 12.2021 s（公共窗右端取各源最小：mpc_real = 12.202111 s；`/traj/progress` 首次 ≥ 1.0：12.213041 s）
- 统一坐标轴：纵轴 -14 到 14 deg（12 张图共用，以 0 为中心）；横轴 0 - 12.2021 s（同场景 4 张共用）；零线为 0 deg。
- 来源：`bag/6_free__20261017-070935` 的 `/mavros/vision_pose/pose`。
- 姿态口径：实飞姿态取 `/mavros/vision_pose/pose`；它与 30 Hz EKF 姿态有约 +0.03 rad 的滚转系统偏差，不要把滚转零偏读成跟踪误差。
- 生成：`python3 tools/report/plot_pitch_roll.py --scene all`
<!-- END phase5-pitchroll:lemniscate:mpc_real -->
