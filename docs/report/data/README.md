# 报告数据（Phase 5 票 01）

> 本目录由 `tools/report/build.py` 生成，**不要手改**。
> 生成命令：`source /opt/ros/humble/setup.bash && python3 tools/report/build.py --scene all`

每条数据都能回到同名的 `.json`：窗口、原点、输入 sha256、读数都在那里。

## 四源

| `source` | 是什么 | 数据来源 | 控制器反馈 |
|---|---|---|---|
| `target` | 参考目标 | offboard 包 `/traj/reference`（相对量）+ 轨迹原点 | 无（目标） |
| `offboard` | PX4 内置 PID 实飞 | offboard 包 `/mavros/vision_pose/pose` | `/mavros/local_position/pose`（EKF 30 Hz） |
| `mpc_sim` | MPC 在 SITL 里闭环 | 票 02 的 `R-*` summary，`x0`（合成动捕） | SITL 合成 mocap |
| `mpc_real` | MPC 实飞 | 实飞包 `/mavros/vision_pose/pose` | `/mocap/pose_enu`（原始动捕） |

⚠️ 报告里的“实际轨迹”统一用 `/mavros/vision_pose/pose`，但 offboard 与 MPC 两组的
**控制器反馈不是同一条话题**（前者 EKF 30 Hz，后者原始动捕）。

## 窗口

| 场景 | 公共窗终点 | 由谁决定 | 网格点数 |
|---|---|---|---|
| 悬停（`hover`） | 39.6494 s | `mpc_sim` | 3966 |
| 正方形（`square`） | 8.0096 s | `mpc_real` | 802 |
| 8 字（`lemniscate`） | 12.2021 s | `mpc_real` | 1222 |

公共窗 = 四源完成时刻的**最小值**。完成时刻的口径：

- 实飞：`/traj/progress` **首次** ≥ 1.0（之后参考继续发，不能取末帧）；
- 悬停 offboard 包录到 TRACK 中途截断（progress 未到 1.0）⟹ 截止到参考流末尾；
- 仿真：模型参考的 `span_s`（`bag` 段；`pre`/`hold` 是回放夹具加的，不进窗口）。

每个源为什么在这个时刻结束，见 `.json` 的 `sources.<源>.window_trim_reason`；
仿真 t=0 还扣掉了参考发布延迟（`ref_t[0] − t0_abs` 为 7.7–10.2 ms）。

## 列

| 列 | 单位 | 口径 |
|---|---|---|
| `source` | — | `target` / `offboard` / `mpc_sim` / `mpc_real` |
| `t_s` | s | 相对各自 `/traj/start`（仿真 = 回放 `bag` 段起点 − 参考发布延迟） |
| `x_m`,`y_m`,`z_m` | m | 已减各自轨迹原点（ENU） |
| `x_abs_m`,`y_abs_m`,`z_abs_m` | m | 未减原点（便于回包核对） |
| `vx_mps`,`vy_mps`,`vz_mps` | m/s | 目标=参考速度；实飞=EKF `velocity_local`；仿真=`x0[3:6]` |
| `roll_rad`,`pitch_rad`,`yaw_rad` | rad | ZYX 内旋，由 `qw..qz` 换算 |
| `qw`,`qx`,`qy`,`qz` | — | **wxyz**（与 PX4 一致） |
| `u_thrust_n` | N | 仅 MPC 源：`thrust × t_max`；`t_max` 逐趟见 `.json` |
| `wx_radps`,`wy_radps`,`wz_radps` | rad/s | 仅 MPC 源：机体角速率指令 |
| `valid` | 0/1 | 该网格点是否落在该源原始数据的时间范围内；0 = 端点保持值 |

网格 100 Hz：位置/速度线性插值，四元数沿最短弧 slerp，控制量**零阶保持**。

## 原点（最容易搞错的一条）

轨迹原点取 **`AUTO/WAIT_START` 期间指令的恒定值**，不是 `/traj/start` 时刻的飞机位置：
FSM 在稳定判据成立时就锁存原点，两者实测差 6.5–10.5 cm。⛔ 用后者会引入系统偏差。

- offboard：`/mavros/setpoint_raw/local`；MPC：`/control/target`；
- 仿真：`x_ref0（前摇）− 模型参考首点`，等价于「DUT 锚点状态 + 参考 z 偏置 1.0 m」；
- 所有源减去各自原点后再写 `x_m` 等相对列。

## 已知坑

1. **目标不是逐点等于指令**：`/traj/reference` 标称 50 Hz，20 ms 附近的抖动是常态；
   缺口内指令保持上一帧。中断多长见 `.json` 的 `readings.reference_gaps_s`
   （看 `n_over_40ms` / `max_s`，别看 20 ms 那个）；`readings.target_identity`
   记了 p99 / max，不要把 max 当成“公式不成立”。
2. **仿真 `pre` 段是我们加的**：回放夹具多放了 30 s 前摇（顶住参考首点）。
   它不进入公共窗；仿真 t=0 取 `bag` 段起点再减参考发布延迟。
3. **`run/logs/` 会被 gitignore**：推力归一化常数与启动行原文摘在
   `tools/report/evidence/mpc-normalization.json`；日志还在时 build 会核对 sha256。
4. **推力常数不跨趟抄**：每趟的 `t_max` 来自该趟 `mpc_node` 启动行；
   仿真用 `harness-identity.json` 的控制器参数。
5. **悬停 offboard 包是截断录制**：没有 `progress=1.0`，完成时刻=参考流末尾；
   终值与包尾差多少见 `.json` 的 `sources.offboard.window_trim_reason`。
6. **目标姿态是“水平 + 零偏航”**（控制器自己的约定，`node_core.py` 的 `traj_reference`），
   不是由加速度反解的“理想倾角”。飞机为了加速/抗阻会倾，所以实际倾角与 0 的差
   **不是**姿态控制误差；第 8 章解释了为什么这个模型不需要倾角前馈。
7. **仿真偏航约 90°，目标 0°**：仿真源首帧 `x0` 的偏航就在 90° 附近，整场没转到 0。
   这是数据里的真实读数，**本票不判定原因**；姿态图 yaw 那一行会看到一条常值偏置，
   别把它读成控制失效。`metrics_vs_target.attitude` 已把总夹角、倾角、偏航差**分开**给了。
8. **offboard 实测误差可与验收文档对表**（独立工具、50 Hz、`/mocap/pose_enu`）：
   `docs/control/ladder-acceptance-criteria.md` §2.2 的悬停 0.1049/0.1610、
   正方形 0.5682/0.7552、8 字 0.5886/0.8160（本次生成值见下表，差 < 3%）。

| 场景 | offboard 水平均值 / 最大（m） |
|---|---|
| 悬停（`hover`） | 0.105 / 0.161 |
| 正方形（`square`） | 0.572 / 0.761 |
| 8 字（`lemniscate`） | 0.594 / 0.824 |
