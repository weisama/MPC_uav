# 07: `traj` —— 三条参考轨迹

Status: done
Type: task

> ✅ **2026-09-22 用户裁定：收 `done`**（原为 `ready-for-human`；7/7 勾选框本就已对，`done` 的判据
> 按 `triage-labels.md` 满足）。
> ★ **一处残留仍记在这里、不藏在 `done` 下面**：验收项「RViz 里能看出三条几何的形状与尺寸对不对」
> **只在消息层核过**（顶点数 / 边长 / 幅度 / 闭合 / `frame_id` / 周期重发），本机 `DISPLAY` 为空、
> **没真的开 RViz 看一眼**。⟹ 台架时若有屏幕，顺手 `rviz2` 加一条 `/traj/marker` 目视即可
> （`marker_frame_id` 默认 `map`，理由见 §二.5）；没看也不影响控制逻辑。

**What to build:** 三条在**轨迹原点**下定义、按时间采样的**解析**参考轨迹，以 **50 Hz** 输出，与 setpoint 同频。

因为 Phase 1 是 `(A)` 级、**只发位置**，动力学全部由 PX4 的 `MPC_*` 级联 PID 处理 —— **轨迹发生器不需要算任何加速度或姿态**。这正是 Phase 1 能很快做出来的原因。

| # | 轨迹 | 定义（单位 m） | 时长 |
|---|---|---|---|
| 1 | 原地悬停 | 目标恒为原点，高度 1.0 m | — |
| 2 | 正方形 | 边长 1.0 m：`(0,0) → (1,0) → (1,1) → (0,1) → (0,0)` | 8.0 s（周长 4 m，@0.5 m/s） |
| 3 | 八字 | Gerono 双纽线 a = 0.5 m：`x = a·sin t`，`y = a·sin t·cos t`，`t: 0→2π` | ≈6.1 s（周长 ≈3.05 m，@0.5 m/s） |

限速 **≤0.5 m/s**。

★ **注意八字的实际尺寸**（数值积分算出来的，不是估的）：`x ∈ [−0.5, +0.5]`（宽 1.0 m），`y ∈ [−0.25, +0.25]`（高 0.5 m）—— Gerono 的 y 幅度**只有 x 的一半**，所以 8 字偏扁。**若想要两个方向都是 ±0.5 m 的对称 8 字**，改用 Bernoulli 双纽线 `x = a·cos t/(1+sin²t)`、`y = a·sin t·cos t/(1+sin²t)`。这是**显示偏好，不影响控制逻辑** —— 先按 Gerono 做，飞完不满意再换。

**一张可以完全并行开工的 ticket**：它不碰动捕、不碰 PX4、不碰 FSM，只依赖工作区骨架。

**Blocked by:** 03（只需工作区骨架）

- [x] 三条轨迹各自的几何与上表一致（可数值核对：正方形周长 4 m、八字周长 ≈3.05 m、x/y 幅值）
- [x] 按 `max_speed` 缩放参数推进速率，任一时刻速度 **≤0.5 m/s**
- [x] 在**轨迹原点**下定义（不是动捕棚原点），与相对位姿的口径一致
- [x] 50 Hz 输出，与 setpoint 同频；轨迹完成度可上报（0–1）
- [x] 生成逻辑是**纯函数**、可单测，零 ROS 依赖
- [x] RViz 里能看出三条几何的**形状与尺寸**对不对 —— ⚠️ **本机无 `DISPLAY`，没真的开 RViz 看**；验的是 `/traj/marker` 的**消息内容**（顶点数、边长、幅度、闭合、`frame_id`、周期重发），见下方注释三
- [x] 选择方式支持脚本输入 **1/2/3**，不需要改代码或重新 launch

## Comments

### 2026-09-22 —— 代码完成并提交（agent，在 z0 的 Jetson 上）。7/7 勾选框已对，**RViz 那条只在消息层核对**

#### 一、产出

| 文件 | 说明 |
|---|---|
| `src/traj/include/traj/trajectories.hpp` | ★ 三条轨迹的**纯函数**（`Hover` / `Square` / `Lemniscate` + `make(Kind)`），**零 ROS 依赖**（自带 POD `Vec3`），~400 行 |
| `src/traj/src/traj_node.cpp` | 节点 = ROS 边界：参数、五条话题、定时器、RViz 折线。**不发 setpoint、不碰 mavros** |
| `src/traj/test/test_trajectories.cpp` | ★ 纯函数单测（seam 2），20 例，**含一条独立的 Simpson 法周长交叉核对** |
| `src/traj/test/test_traj_node.py` | ★ 节点级（seam 1），11 例，DUT 是子进程（`bench.Scenario`） |
| `src/traj/config/traj.yaml` | 参数逐条带理由；多机复制一份改话题名即复用 |
| `src/traj/launch/traj.launch.py` | `params_file` + `namespace` 两个 launch 参数（照 `mocap.launch.py`） |
| `src/traj/README.md` | 🆕 运行纪律 + 三条轨迹 + **为什么是弧长匀速** + 参数表 + 怎么用 1/2/3 |
| `src/traj/{package.xml,CMakeLists.txt}` | `mavros_msgs` 只为 `PositionTarget` 一个消息；**刻意不含 `ament_lint_auto`**（与 mocap/bench/control 一致，不增加 lint 基线） |

`colcon test --packages-up-to traj` = **31 例全绿**（seam 2 的 20 + seam 1 的 11）。
⚠️ 要 `--packages-up-to`：**seam 1 依赖 `bench` 夹具**。另外 `colcon test-result` 汇总显示的 33 是把它
xunit 里的汇总节点也数了一遍，真实用例数是 31（两个 `grep -c` 可复核）。
测试**不需要飞控、不需要动捕、不需要起飞**。

#### 二、五条实现决定（都写进了代码注释 / yaml / README，以免后人重新论证）

1. **载体 = `mavros_msgs/PositionTarget`，口径 = 相对轨迹原点。**
   ★ **加原点这件事只在 `control` 里发生一次**：`traj` 根本不知道飞机在哪，只回答"第 t 秒该在哪"。
   好处是它能在没飞控/没动捕/没起飞的情况下把三条几何跑一遍核对（seam 1 就是这么做的），
   且"重复加原点 / 一次都没加"这一类 bug **只有一个可能出错的地方**。
   `type_mask = 2496`（位置+速度+yaw；忽略加速度与 yaw_rate，见 §8.6）、`coordinate_frame = FRAME_LOCAL_NED`、
   `header.frame_id = "traj_origin"`。⚠️ 后两者不矛盾：**mavros 的 `setpoint_raw` 只读 `coordinate_frame`，
   从不读 `header.frame_id`**（第一手：`mavros/src/plugins/setpoint_raw.cpp:209-249`）——`frame_id` 纯粹是给人看的标注。
2. **「选」与「开始」是两条话题**（`/traj/select`、`/traj/start`）。
   只 select 不发参考（但折线一选就出，可离线核对形状）；start 才是参考时钟的零点。
   **运行中重选 ⟹ 立刻停流，且必须再发一次 start**（新时间轴从 0 起）——否则 `control` 还在 TRACK 时
   会按旧轨迹的惯性把飞机拽到新轨迹上。先例：XI35 的 `px4ctrl` 在 AUTO_HOVER 发 `/traj_start_trigger`。
3. **速度上限的正确实现是"弧长匀速"，不是"参数匀速"。**
   正方形按 `s = clamp(t)·v` 取边；八字要建 `θ → 弧长` 的数值表再反查。
   定量理由：`x = a·sinθ` 的速度因子 `√(cos²θ + cos²2θ) ∈ [0.661, 1.414]`、均值 0.970，
   **按 θ 匀速时过中心那一点是平均速度的 1.46 倍** —— 把"平均"压到 0.5 m/s，峰值就到 **0.73 m/s**，直接违反限速。
   弧长表太稀会引入速度波纹（2000 点时有限差分能测到 5.3e-4 的抖动）⟹ 默认 **20000** 点。
4. **"高度 1.0 m"属于轨迹原点，不属于轨迹**（本张最容易踩的坑）。
   轨迹原点是 control 在 HOVER 判稳时记的位置，**那时飞机已经在 1 m 上** ⟹ 悬停的目标就是 `(0,0,0)`，
   正方形四个角 `z = 0`，`height_offset_m` **默认 0.0**。在这条参数里再填 1.0 是**叠加**（= 2 m），不是设定高度。
   已在 `trajectories.hpp` 文件头、`traj.yaml`、README 与 §5.2 四处写死。
5. **`marker_frame_id` 默认 `"map"`（不是 `traj_origin`），且折线必须周期重发。**
   RViz 的 Fixed Frame 要有 TF 才算得出来，而 `traj_origin` **不在任何 TF 树里**（它是 control 记的一个数）；
   写它会得到"RViz 直接把 marker 丢掉"。于是折线的**形状与尺寸对、落点相对动捕原点** ——
   校形状够用（本张的判据），"落点对不对"要在 control 里看。另外 **Marker 是 volatile 的**：后起的 RViz
   看不到已发的那条 ⟹ 按 `marker_period_s`（默认 1 s）周期重发，**包括"选了但没 start"的时候**（那正是离线核对形状的用法）。

#### 三、验收项逐条怎么验的

| 验收项 | 证据 |
|---|---|
| 几何与上表一致 | seam 2：正方形周长 =4.0（解析）、八字周长与**独立 Simpson 积分**互校（≈3.05）、`x ∈ ±0.5`、`y ∈ ±0.25`、四个角、闭合；seam 1 在话题层再核一遍 |
| 速度 ≤0.5 m/s | seam 2 扫过三条轨迹（含有限差分算出的真实速度）；seam 1 用 `header.stamp` 差分核话题上的位置 |
| 在轨迹原点下定义 | 参考是**相对量**（悬停 = `(0,0,0)`），节点不读任何位置输入 ——"不知道飞机在哪"是设计而不是缺陷 |
| 50 Hz + 完成度 | seam 1：窗口频率 45–55 Hz；`progress` 单调、到 1.0、**跑完继续发**（参考停了 control 就会失去参考流） |
| 纯函数、零 ROS | `trajectories.hpp` 不含任何 ROS 头；seam 2 直接 gtest 链进去 |
| RViz 形状与尺寸 | ⚠️ **只核到消息层**（顶点数/边长/幅度/闭合/`frame_id`/周期重发）。**本机 `DISPLAY` 为空，没真的开 RViz 看一眼** —— 留给人 |
| 脚本选 1/2/3 | `ros2 topic pub --once /traj/select std_msgs/msg/UInt8 "{data: 2}"`，**不重启节点**；非法值（如 4）告警并保持当前轨迹 |

#### 四、⟹ 给 [`ticket 09`](09-track-mission-script.md) 的三条接线要求

1. **加轨迹原点**：`setpoint = traj_origin + /traj/reference`（高度也一并来自原点）。⚠️ 绝不能把
   `/traj/reference` 直接 remap 到 `/mavros/setpoint_raw/local`。
2. **进 `TRACK` 那一刻发一次 `/traj/start`**（`std_msgs/Empty`）—— 参考时钟的零点必须与"开始飞"同一时刻，
   否则参考已经在跑而飞机还停在起点。**重选轨迹后要再发一次。**
3. **`/control/traj_progress` 直接转发 `/traj/progress`**（`Float32`），`TRACK → LAND` 用 `progress ≥ 1.0`（§4.2）——
   自己重算弧长等于把同一个数算两遍。另外 **`/traj/marker` 只是可视化**，control 不必订阅。

#### 五、两条测试纪律 + 一个夹具 bug（都写进了 README / bench README）

- ★ **差分速度必须用 `header.stamp`，不能用收到消息的时刻。** rcl 的定时器从**回调开始**重新计时，
  回调一超时就立刻补跑 ⟹ 接收时刻上会出现相邻仅 0.1 ms 的两帧，而它们之间隔着 0.1 s 的运动。
  实测症状：正方形"飞到 4.5 m/s"（限速的 9 倍）、八字 0.527 m/s —— 全是假的。频率断言同理，只在**窗口**上算。
- ★ 两次"看起来是节点 bug"的现象，根因都在测试侧：`[−1]` 取折线时可能拿到**上一条轨迹的周期重发帧**
  （改成按内容认：点数就是不变量，正方形 5、八字 202）；`/traj/start` 是**一次性**消息，
  订阅未匹配上就被丢掉 ⟹ 断言前必须同时等三个发布者与两个订阅者都上齐。
- ⚠️ **顺带修了 `bench` 的一个真 bug**：`fake_mavros.py` 的 `evidence()` 调了 rclpy **不存在**的
  `Service.get_num_clients()`（Humble 没有服务客户端计数 API），于是**任何 bench 超时都会在打印证据时崩掉**，
  把真正的断言失败盖住（本张排查 `/traj/start` 时被它骗过一次）。已删掉该方法与 evidence 里的那一句，
  更正记录留在 `fake_mavros.py` 与 `src/bench/README.md`。

#### 六、没做的（明确记下，免得被当成遗漏）

- **Bernoulli 双纽线**（对称 8 字）：Gerono 的偏扁是规格（题面写了"先按 Gerono 做，飞完不满意再换"）。
- **沿轨迹切向的 yaw**：8 字中心交叉处会要求 180° 掉头（飞机做不到，会甩）；§8.6 的 yaw 前馈本来就"先不做"。
- **加速度前馈**：Phase 1 是 `(A)` 级位置控制，动力学全交给 PX4 的 `MPC_*` 级联 PID ⟹ 轨迹发生器不需要算任何加速度或姿态。
