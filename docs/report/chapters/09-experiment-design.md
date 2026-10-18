# 第 9 章 实验设计与数据处理

本章说明第 10 章的数据从哪里来、怎样组织，以及怎样重跑。四个源——参考目标、offboard 实飞、MPC 仿真、MPC 实飞——来自六份话题包和三趟 SITL；本章给出四源的定义、包与场景的映射，以及复现命令。

全章只讲数据，不讲结果。四个源由同一套管线生成 100 Hz 长表；数据窗口、重采样与指标的具体口径以 [`docs/report/data/README.md`](../data/README.md) 与 [`tools/report/`](../../../tools/report) 的实现为准。

话题、状态机与坐标系的分工见第 5、7 章；模型、参考姿态与控制器约定见第 8 章。本章不重复推导，只说明数据来源与组织方式。

---

## 9.1 数据组织

### 9.1.1 四个源

表 9-1 给出四个源的定义。目标与 offboard 来自同一份包：`/traj/reference` 是相对轨迹原点的参考，加上该包的原点才是绝对目标；offboard 的实际轨迹取 `/mavros/vision_pose/pose`。MPC 仿真取 SITL summary 的 `x0`，MPC 实飞取实飞包的 VP。

**表 9-1　四个源。**

| `source` | 是什么 | 数据 | 时间零点 | 轨迹原点 | 控制器反馈 |
|---|---|---|---|---|---|
| `target` | 参考目标 | offboard 包 `/traj/reference`（相对量）+ 原点 | 同一份 offboard 包 | 同一份 offboard 包 | 无（目标不是测量） |
| `offboard` | PX4 内置 PID 实飞 | `/mavros/vision_pose/pose` | 同一份 offboard 包 | `/mavros/setpoint_raw/local` | EKF2 位置估计（ROS 侧 LP，约 30 Hz） |
| `mpc_sim` | MPC 在 SITL 闭环 | summary `frames[].x0`（合成动捕） | 由墙钟反推（[`tools/report/extract.py`](../../../tools/report/extract.py)） | `x_ref0`（前摇段）− 模型参考首点 | SITL 合成 mocap |
| `mpc_real` | MPC 实飞 | `/mavros/vision_pose/pose` | 同一份实飞包 | `/control/target` | `/mocap/pose_enu`（原始动捕） |

<big>**一句话总结：四个源分别来自同场实飞包与 SITL 状态，是后续所有对比的输入。**</big>

### 9.1.2 三份产物

三份产物是 `docs/report/data/<scene>.csv`、同名的 `<scene>.json` 与数据字典 [`docs/report/data/README.md`](../data/README.md)。CSV 是长表：一行一个源、一个网格点，列固定为 `source, t_s, x_m, y_m, z_m, x_abs_m, y_abs_m, z_abs_m, vx_mps, vy_mps, vz_mps, roll_rad, pitch_rad, yaw_rad, qw, qx, qy, qz, u_thrust_n, wx_radps, wy_radps, wz_radps, valid`。相对列 `x_m` 等已减各自原点；绝对列 `x_abs_m` 等保留原值，便于回包核对。`valid=0` 表示该网格点落在该源原始时间范围之外，值是端点保持的结果，不是新测到的样本。

JSON 存的是方法元数据：窗口、原点、完成原因、输入 sha256、参考发布缺口与逐源读数。CSV 只存网格上的数，不重复这些口径。换包或改窗口后，两份产物必须一起重生成；只重画图不会更新 JSON。

<big>**一句话总结：四个源统一落成一张 100 Hz 长表，相对列与绝对列并存，口径元数据在同名 JSON。**</big>

---

## 9.2 数据集

### 9.2.1 六份话题包与场景映射

三个场景各用两份包：一份 offboard 实飞包，同时提供 `/traj/reference` 与 PX4 内置 PID 的实际轨迹；一份 MPC 实飞包。表 9-2 是本章与 [`tools/report/build.py`](../../../tools/report/build.py) 的 `SCENES` 字典逐项对应的映射，改路径必须两处一起改。

**表 9-2　六份话题包。**

| 场景 | 控制器 | 包 | 包内首条消息（UTC） | 包时长 | TRACK 情况 |
|---|---|---|---|---|---|
| 悬停 | PX4 内置 PID（offboard） | [`bag/3_traj1_hover`](../../../bag/3_traj1_hover) | 2026-10-08T09:39:51.222Z | 116.4 s | 录到 39.65 s 截断，`progress` 终值 0.661 |
| 悬停 | MPC 实飞 | [`bag/6_free__20261017-070103`](../../../bag/6_free__20261017-070103) | 2026-10-17T07:01:06.495Z | 90.0 s | 完整，60.02 s |
| 正方形 | PX4 内置 PID（offboard） | [`bag/4_traj2_square`](../../../bag/4_traj2_square) | 2026-10-08T09:44:12.981Z | 36.9 s | 完整，8.02 s |
| 正方形 | MPC 实飞 | [`bag/6_free__20261017-070446`](../../../bag/6_free__20261017-070446) | 2026-10-17T07:04:49.276Z | 38.0 s | 完整，8.01 s |
| 8 字 | PX4 内置 PID（offboard） | [`bag/5_traj3_lemniscate`](../../../bag/5_traj3_lemniscate) | 2026-10-08T10:18:31.477Z | 49.6 s | 完整，12.21 s |
| 8 字 | MPC 实飞 | [`bag/6_free__20261017-070935`](../../../bag/6_free__20261017-070935) | 2026-10-17T07:09:38.175Z | 46.3 s | 完整，12.20 s |



<big>**一句话总结：3条飞行轨迹 X 2种控制方式。**</big>

### 9.2.2 三趟 nominal 仿真

仿真格不是另造一条参考，而是把实飞包的参考逐字回放。三趟都用同一个被控对象（`z0.param`，`--mismatch none`）与同一套控制器参数（`mass=0.75 kg`、`hover_thrust=0.1589`、`t_max=46.3027 N`），参考分别取自 `bag/3`、`bag/4`、`bag/5`。表 9-3 给出三趟的配置。

**表 9-3　三趟 nominal SITL。**

| tag | 参考包 | 参考形状 | `span` | `hold_s` | 产物 |
|---|---|---|---|---|---|
| `R-hover` | [`bag/3_traj1_hover`](../../../bag/3_traj1_hover) | 悬停，参考恒为 0 | 39.6417 s | 78 | `R-hover-summary.json`、`solver-summary.json` |
| `R-square` | [`bag/4_traj2_square`](../../../bag/4_traj2_square) | 1 m 正方形 | 17.8832 s | 56 | `R-square-summary.json`、`solver-summary.json` |
| `R-lemniscate` | [`bag/5_traj3_lemniscate`](../../../bag/5_traj3_lemniscate) | 8 字，$a=1.0$ m | 26.2236 s | 65 | `R-lemniscate-summary.json`、`solver-summary.json` |

三趟的 `ref_hold_s` 都是 30 s，`ref_z_offset_m` 都是 1.0 m，`hold_s` 按 `ref_hold_s + span + 8.0` 向上取整到秒（8.0 s 是验证器留的稳态窗口）。`span` 是模型参考的 `bag` 段长度；正方形与 8 字的参考在完成之后还会继续发零，所以 `span` 比 TRACK 时长长（17.88 s 对 8.02 s、26.22 s 对 12.21 s）。悬停那趟只录到 39.65 s，模型参考也按 39.64 s 回放，没有补到 60 s。

参考模型的生成命令与 sha256 在 [`tools/report/evidence/sitl/README.md`](../../../tools/report/evidence/sitl/README.md)。正方形与 8 字用 `mocap_model.py dump` 从对应包导出；悬停的 TRACK 窗口在包尾截断，`mocap_model.py` 的 `track_window()` 会返回零长窗口，因此改用 `build_hover_model.py`，只把窗口终点改成包尾，其余逐字复用 `build_model()`。三份模型的 sha256 已复算过，重跑逐字节一致。

已有的 Tier A（`hold` 参考）与 Tier B 的 `T0`（质量 +30% 的失配对象）不作为四源图的仿真格。它们的参考或对象与实飞不同，放进四源图会把“失配鲁棒性”混进“同口径跟踪”；第 10 章只在边界一节说明这一点，不把它们的数值并进结果表，失配问题另见 [`docs/control/tier-b-sitl-seam3.md`](../../control/tier-b-sitl-seam3.md)。

<big>**一句话总结：三趟 nominal 仿真复用实飞参考与同一套控制器参数，只补上缺失的仿真格。**</big>

---

## 9.3 复现命令

以下命令都从仓库根目录执行；数据管线需要 [ROS 2 Humble](https://docs.ros.org/en/humble/) 的 Python 环境，先 source 再跑：

```bash
source /opt/ros/humble/setup.bash
python3 tools/report/build.py --scene all
python3 tools/report/plot_traj.py --scene all --candidates
python3 tools/report/plot_pitch_roll.py --scene all
python3 -m pytest tools/report -q
```

`build.py` 的输入与逐字节可复现说明见 [`docs/report/data/README.md`](../data/README.md)。第 10 章误差、姿态与控制量图的命令以 [`docs/report/generated/figure-captions.md`](../generated/figure-captions.md) 的图注为准；三趟 SITL 的模型导出与闭环参数见 [`tools/report/evidence/sitl/README.md`](../../../tools/report/evidence/sitl/README.md)。全量测试需要设置 `ACADOS_SOURCE_DIR`。

<big>**一句话总结：数据、图与测试都能从仓库根目录用给定命令重跑，仿真需另起 PX4 与 mavros。**</big>

---

## 9.4 本章引用清单

表 9-4 是本章结论的落点。路径都相对仓库根；带行号的源码位置是写作时的快照，改代码后以实际文件为准。

**表 9-4　引用与证据。**

| 路径 | 内容 | 用在哪 |
|---|---|---|
| [`docs/report/data/README.md`](../data/README.md) | 数据字典：四源、窗口、列、原点、已知坑 | 全章 |
| `docs/report/data/<scene>.json` | 窗口、原点、完成原因、输入 sha256、逐源读数 | 9.1、9.2 |
| [`tools/report/build.py`](../../../tools/report/build.py) | 管线入口、`SCENES` 映射与产物 | 9.1–9.3 |
| [`tools/report/extract.py`](../../../tools/report/extract.py) | 读包、原点提取、仿真 t0 反推 | 9.1、9.2 |
| [`tools/report/evidence/sitl/README.md`](../../../tools/report/evidence/sitl/README.md) | 三趟 nominal SITL 的配置、命令、产物 | 9.2.2、9.3 |
| [`tools/report/evidence/mpc-normalization.json`](../../../tools/report/evidence/mpc-normalization.json) | 三趟启动行原文、`t_max` 与日志哈希 | 9.2.2 |
| [`bag/README.md`](../../../bag/README.md)、[`bag/MANIFEST.md`](../../../bag/MANIFEST.md) | 包来源、会话起点、时长、轨迹归属 | 9.2.1 |
| [`src/traj/config/traj.yaml`](../../../src/traj/config/traj.yaml) | 参考频率、相对量、形状与尺度 | 9.2.1 |
| [`docs/control/tier-b-sitl-seam3.md`](../../control/tier-b-sitl-seam3.md) §1 | B 档状态源、真值判据、失配对象 | 9.2.2 |

本章的数值快照取自 2026-10-17/18 生成的三份 `<scene>.json`。包、模型或控制器参数一变，窗口与逐源读数都要重算；不要把本章的表当成永久常量。
