# 3D 候选图注（自动生成，不进正文）

> 对应 `figures/candidates/`。**本文件不进入正文**；3D 候选是否采用
> 由操作者看图后裁定（票 03 验收）。

<!-- BEGIN phase5-traj-candidates:hover -->
### T1-3D 悬停（hover）3D 轨迹候选

- 文件：`figures/candidates/fig-traj-hover-3d.png`、`figures/candidates/fig-traj-hover-3d.svg`（300 dpi）
- 状态：**未进入正文**，待操作者看图后裁定去留。
- 数据、窗口、坐标与来源同 T1：`data/hover.csv`，窗口 0.0000 - 39.6494 s（四源完成时刻取最小：mpc_sim = 39.649415 s）。
- 坐标轴统一：三张候选与正文图共用同一组范围——XY 两轴 -1.7 到 1.7 m；3D 视图的 z 轴 -0.3 到 0.3 m；盒体按该范围成比例，z 只有厘米级，因此盒体偏薄。
- 注意：3D 透视视角不用于读数，只用于观察轨迹形状。
- 生成：`python3 tools/report/plot_traj.py --scene all --candidates`
<!-- END phase5-traj-candidates:hover -->

<!-- BEGIN phase5-traj-candidates:square -->
### T2-3D 正方形（square）3D 轨迹候选

- 文件：`figures/candidates/fig-traj-square-3d.png`、`figures/candidates/fig-traj-square-3d.svg`（300 dpi）
- 状态：**未进入正文**，待操作者看图后裁定去留。
- 数据、窗口、坐标与来源同 T2：`data/square.csv`，窗口 0.0000 - 8.0096 s（四源完成时刻取最小：mpc_real = 8.009599 s）。
- 坐标轴统一：三张候选与正文图共用同一组范围——XY 两轴 -1.7 到 1.7 m；3D 视图的 z 轴 -0.3 到 0.3 m；盒体按该范围成比例，z 只有厘米级，因此盒体偏薄。
- 注意：3D 透视视角不用于读数，只用于观察轨迹形状。
- 生成：`python3 tools/report/plot_traj.py --scene all --candidates`
<!-- END phase5-traj-candidates:square -->

<!-- BEGIN phase5-traj-candidates:lemniscate -->
### T3-3D 8 字（lemniscate）3D 轨迹候选

- 文件：`figures/candidates/fig-traj-lemniscate-3d.png`、`figures/candidates/fig-traj-lemniscate-3d.svg`（300 dpi）
- 状态：**未进入正文**，待操作者看图后裁定去留。
- 数据、窗口、坐标与来源同 T3：`data/lemniscate.csv`，窗口 0.0000 - 12.2021 s（四源完成时刻取最小：mpc_real = 12.202111 s）。
- 坐标轴统一：三张候选与正文图共用同一组范围——XY 两轴 -1.7 到 1.7 m；3D 视图的 z 轴 -0.3 到 0.3 m；盒体按该范围成比例，z 只有厘米级，因此盒体偏薄。
- 注意：3D 透视视角不用于读数，只用于观察轨迹形状。
- 生成：`python3 tools/report/plot_traj.py --scene all --candidates`
<!-- END phase5-traj-candidates:lemniscate -->
