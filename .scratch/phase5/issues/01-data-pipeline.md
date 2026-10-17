# 01: 数据提取与对齐管线

Status: ready-for-human
Type: task

**What to build:** 一条可重跑的数据管线，把六份话题包与三趟 SITL 的四种来源变成统一 CSV/JSON。
它是第 9/10 章以及所有图的地基；**没有这条管线，图上的四源不可相减**。

**Blocked by:** 02（`mpc_sim` 源要读票 02 的 summary；真实三源可以先落，最终验收要等 02）

## 范围

输入：

| 场景 | 目标 | offboard | 模型预测控制仿真 | 模型预测控制实飞 |
|---|---|---|---|---|
| 悬停 | `bag/3_traj1_hover` | `bag/3_traj1_hover` | 票 02 的 `R-hover` | `bag/6_free__20261017-070103` |
| 正方形 | `bag/4_traj2_square` | `bag/4_traj2_square` | 票 02 的 `R-square` | `bag/6_free__20261017-070446` |
| 8 字 | `bag/5_traj3_lemniscate` | `bag/5_traj3_lemniscate` | 票 02 的 `R-lemniscate` | `bag/6_free__20261017-070935` |

输出：

- `docs/report/data/<scene>.csv`：长表，列见下；
- `docs/report/data/<scene>.json`：窗口、原点、输入 sha256、生成命令、读数；
- `docs/report/data/README.md`：数据字典（单位、坐标系、来源、窗口、已知坑）；
- `tools/report/`：`extract.py`、`align.py`、`metrics.py`、`build.py`、`test_report_data.py`。

## 口径（必须写死）

1. **时间零点**：`/traj/start`（`std_msgs/Empty`，无 header）的**包记录时间戳**。
2. **轨迹原点**：
   - offboard：首个 `AUTO/WAIT_START` 期间 `/mavros/setpoint_raw/local` 的恒定值；
   - 模型预测控制：首个 `AUTO/WAIT_START` 期间 `/control/target` 的恒定值；
   - 回退：首个 `AUTO/WAIT_START` 时 `/mavros/local_position/pose` 的位置。
   - ⛔ 不用 `/traj/start` 时刻的飞机位置（会引入 6.5–10.5 cm 假偏差）。
3. **TRACK 窗**：`[t0, 首个 /traj/progress >= 1.0]`；悬停 offboard 包没有 1.0，
   截止到包尾（约 39.65 s / progress 0.661），并在 JSON 里记 `window_trim_reason`。
4. **绝对目标**：`origin + /traj/reference.position`；只在 TRACK 窗内成立。
5. **相对系**：所有源减去各自原点后再写入 CSV（同时保留绝对列，便于核对）。
6. **公共网格**：100 Hz；位置/速度线性插值；四元数 slerp 后转欧拉角。
7. **姿态源**：实飞用 `/mavros/vision_pose/pose`（~190 Hz）；仿真用 `x0[6:10]` 四元数。
   ⛔ 不用 `/mavros/local_position/pose` 的姿态（30 Hz EKF，与前者约 +0.03 rad 滚转偏差）。
8. **完成时刻**：实飞看 `progress`；仿真看参考末帧/注入时间线。公共窗取各源最小完成时刻。

## CSV 列

`source`（`target` / `offboard` / `mpc_sim` / `mpc_real`）、`t_s`、`x_m`、`y_m`、`z_m`、
`x_abs_m`、`y_abs_m`、`z_abs_m`、`vx_mps`、`vy_mps`、`vz_mps`、
`roll_rad`、`pitch_rad`、`yaw_rad`、`qw`、`qx`、`qy`、`qz`、
`u_thrust_n`、`wx_radps`、`wy_radps`、`wz_radps`、`valid`。

- 目标源的 `u_*` 为空；offboard 源没有推力/角速率，只填位置与姿态；
- 模型预测控制源的 `u_thrust_n` 由 `thrust` / `thrust_body_z` 按该趟日志的归一化常数换算，
  换算口径写进 JSON；**不跨趟抄常数**。
  ⚠️ `run/logs/` 被 gitignore：必须把该趟启动行的 `mass`/`hover_thrust`/`t_max` **原文**摘进 JSON，
  不能只留一个会消失的路径。
- JSON 的每个源要记 `feedback_source`：offboard = `/mavros/local_position/pose`（EKF 30 Hz）、
  模型预测控制 = `/mocap/pose_enu`（原始动捕）、仿真 = SITL 的合成 mocap。
  报告里的"实际轨迹"统一用 `/mavros/vision_pose/pose`，但两组的控制器反馈不是同一条。

## 检查清单

- [x] `tools/report/build.py --scene all` 能从六份包 + 三份 SITL summary 重跑出全部 CSV/JSON
- [x] 每份包的 sha256、每份 SITL summary 的 sha256 写进 JSON
- [x] 原点重建与探针值一致（mm 级）；`target = origin + reference` 在 TRACK 窗内逐包验证
- [x] 悬停 offboard 窗截止 39.65 s；正方形/8 字截止 `progress=1.0`
- [x] 四源都有完整列；缺列/缺源直接失败，不静默补 0
- [x] 四元数 ↔ 欧拉角往返误差 < 1e-9 rad（合成用例）
- [x] 公共窗取最小值；窗口与裁剪理由写进 JSON
- [x] 临时解包目录清理；仓库只新增 `docs/report/data/**` 与 `tools/report/**`
- [x] `python3 -m pytest tools/report/test_report_data.py -q` 全绿
- [x] 变异自检：把原点换成 `/traj/start` 时刻位置、把截断点移到包尾，断言分别会红

## 验收

- 数据字典能让一个没读过包的人按列复算误差；
- 重跑结果与提交版一致（浮点容差内）；
- 每个数都能回到 `docs/report/data/<scene>.json` 的读数与输入 sha256；
- 完成后切 `ready-for-human`，附"四源读数摘要 + 已知坑 + 未确定项"给操作者审。

## 证据锚

- `/traj/reference` 相对轨迹原点：`src/traj/config/traj.yaml`、`src/control/src/px4_interface.cpp:385-390`；
- FSM 原点锁存：`src/control/include/control/fsm.hpp:1447-1451`；
- 话题与完成语义：2026-10-17 六份包的数值探针。

## Comments

### 2026-10-17 —— 实现完成，转 ready-for-human

**重跑**：`source /opt/ros/humble/setup.bash && python3 tools/report/build.py --scene all`
（约 17 s，临时解包目录建在 `/tmp` 并在退出时删除）。同一输入连跑两次，
`docs/report/data/**` 的 sha256 逐字节一致。测试：

```
$ python3 -m pytest tools/report/test_report_data.py -q
61 passed in 20.91s
```

**四源读数摘要**（公共窗内、相对各自原点；详见 `docs/report/data/<scene>.json`）：

| 场景 | 源 | 水平误差均值 / 最大（m） | 倾角均值（°） | 偏航差均值（°） |
|---|---|---|---|---|
| 悬停 | offboard | 0.1047 / 0.1610 | 2.43 | 7.48 |
| 悬停 | mpc_sim | 0.0132 / 0.0415 | 0.21 | 90.51 |
| 悬停 | mpc_real | 0.0694 / 0.1333 | 0.71 | 1.81 |
| 正方形 | offboard | 0.5720 / 0.7608 | 3.73 | 3.25 |
| 正方形 | mpc_sim | 0.4073 / 0.6253 | 7.66 | 90.05 |
| 正方形 | mpc_real | 0.2753 / 0.3802 | 3.25 | 1.41 |
| 8 字 | offboard | 0.5935 / 0.8241 | 3.02 | 5.25 |
| 8 字 | mpc_sim | 0.5204 / 0.8650 | 8.91 | 85.08 |
| 8 字 | mpc_real | 0.2691 / 0.3950 | 3.34 | 1.90 |

公共窗：悬停 `0–39.6494 s`（由 mpc_sim 决定）、正方形 `0–8.0096 s`（mpc_real）、
8 字 `0–12.2021 s`（mpc_real）。原点黄金值与探针值逐份一致到 1 mm 内；
`target = origin + reference` 在 TRACK 窗内逐样本核对（offboard 均值 0–4.7 mm，
模型预测控制实飞 1.4–10.1 mm，仿真 ≤ 36 mm——差的是参考流 50 Hz 重发与
DUT 100 Hz 取样的对齐，不是公式不成立）。

**已知坑**（完整版在 `docs/report/data/README.md` §已知坑）：

1. `/traj/reference` 标称 50 Hz，20 ms 附近抖动是常态；缺口看 `readings.reference_gaps_s`
   的 `n_over_40ms` / `max_s`（正方形/8 字 offboard 包最长 110/154 ms）。
2. 悬停 offboard 包是**录制截断**：progress 终值 0.661，未到 1.0；完成时刻取参考流末尾
   `39.6501 s`（包尾 `39.6612 s`，差 11 ms）。裁剪理由已写进 JSON 的
   `sources.offboard.window_trim_reason` 与 `grid.window_trim_reason`。
3. 推力常数逐趟取该趟 `mpc_node` 启动行，摘录与日志 sha256 在
   `tools/report/evidence/mpc-normalization.json`；`run/logs/` 被 gitignore。
4. 仿真 t=0 = 回放 `bag` 段起点 **− 参考发布延迟**（7.7–10.2 ms，由同场景实飞包量出）。
   不减会把仿真源整体后移约 10 ms，与 spec §3 不符。
5. 原点回退路径（`/mavros/local_position/pose`）已实现并有合成用例；六份包都走主路径，
   回退**没有被实物输入执行过**。

**未确定项 / 留给后续票**：

- 仿真源偏航从首帧起就在 90° 附近、整场没转到 0。这是数据读数，本票**未判定原因**；
  `metrics_vs_target.attitude` 已把总夹角、倾角、偏航差分开给，第 9/10 章可直接引用。
- 逐轴 RMSE、悬停末 10 s 稳态误差、角速率饱和计数按票 15（第 10 章）实现，本票未做。
- 多分片 `.db3` 不支持：`extract._single_db` 遇到多个分片会**报错**而不是静默只读第一个。
- 验收文档 §2.2 的独立基线（offboard 悬停 0.1049/0.1610、正方形 0.5682/0.7552、
  8 字 0.5886/0.8160）与本次生成值差 < 3%，已写成回归用例
  `test_committed_offboard_metrics_match_published_baseline`。

**追加（2026-10-17 15:09 票 02 重跑 R-square）**：`R-square-summary.json` 被替换
（sha256 `d687277a…` → `637a9082…`），`square.json` 的 `mpc_sim` 读数随之变化：
水平误差均值/最大 0.4160/0.6452 → **0.4073/0.6253 m**，倾角均值 7.77° → 7.66°，
偏航差 90.07° → 90.05°。已重跑 `build.py --scene square` 对齐当前证据；
`hover` / `lemniscate` 未受影响。
