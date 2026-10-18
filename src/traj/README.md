# `traj` —— 参考轨迹发生器

只回答一个问题：**「第 t 秒该在哪、以多快的速度经过」**。

```
/traj/select (UInt8) ─┐
/traj/start  (Empty) ─┤→  traj_node  ─┬→ /traj/reference (PositionTarget, 50 Hz)
                      │   （纯查表）  ├→ /traj/progress  (Float32, 0–1)
   config/traj.yaml ──┘               └→ /traj/marker    (Marker, RViz 折线)
```

## ★ 运行纪律（比代码重要）

1. **它不发 setpoint、不碰 mavros、不判稳。** 参考要变成 setpoint、并且**把轨迹原点加回去**，
   是 `control` 的事（ticket 09 已落地：`control/README.md` 的「轨迹跟踪」一节）。
   ⚠️ **绝不能把 `/traj/reference` 直接 remap 到
   `/mavros/setpoint_raw/local`**：轨迹原点是 1 m 高度上的悬停点，直接转发 = 先往地面附近扎。
2. **参考是"相对轨迹原点"的，`traj` 不知道飞机在哪。** 这正是它能在**没有飞控、没有动捕、没起飞**
   的情况下把三条几何跑一遍核对的原因（seam 1 测试就是这么做的）。
3. **「选」与「开始」是两次动作。** 只 `select` 不发参考（但 RViz 折线一选就出，可以离线看形状）；
   `start` 才是参考时钟的零点。**运行中重选会立刻停流，且必须再发一次 `start`** ——
   否则 `control` 还在 TRACK 时，飞机会被从旧轨迹拽到新轨迹上。
4. **改 `traj.yaml` 不用改代码**；多台无人机复制一份 yaml 改话题名即可（`namespace` + `params_file`）。

## 四条轨迹（都在**轨迹原点**下定义，见 `docs/control/phase1-implementation.md` §5.2）

| # | 轨迹 | 定义（m） | 时长 |
|---|---|---|---|
| 1 | 原地悬停 | 恒为原点（**`z = 0`，不是 1.0** —— 见下） | `hover_duration_s`，默认 60 s |
| 2 | 正方形 | 边长 1.0 m：`(0,0)→(1,0)→(1,1)→(0,1)→(0,0)`，**起点在原点**（= 悬停点，所以进 TRACK 不跳变） | 8.0 s（周长 4 m @0.5 m/s） |
| 3 | 八字 | Gerono 双纽线 `a = 1.0`：`x = a·sinθ`、`y = a·sinθ·cosθ`，`θ: 0→2π` | ≈12.2 s（周长 ≈6.10 m @0.5 m/s） |
| 4 | **起飞** | 从轨迹原点**匀速直上** `takeoff_height_m`（默认 **0.2 m**），到顶后**停在那儿** | 到顶用 `takeoff_height_m / takeoff_speed_ms`（默认 0.2/0.2 = **1.0 s**），之后走完 `hover_duration_s` |

★★ **"高度 1.0 m"不属于轨迹，属于轨迹原点。** 原点由 `control` 在 HOVER 判稳那一刻记下，
**那时飞机已经在 1 m 上** ⟹ 轨迹里的 `z` 就该是 0。参数 `height_offset_m` 默认 **0.0**；
填 1.0 是**叠加**（变成 2 m），不是设定高度。

★ **轨迹 4 是 2026-10-16 新加的**（票 14 二定），它和上面三条有三处**故意**不一样：

- `z(0) = 0` ⟹ **零点就是轨迹原点**。对 MPC 那一路，原点 = 闸门第一次打开那一拍
  （飞机此刻停在**地面**）⟹ 目标 = 地面 + 0.2 m，这就是"起飞 20 cm"。
  ⚠️ **不要**拿它去接 `control` 的 TRACK：`control` 的轨迹原点是**1 m 高度上的悬停点**，
  在原点（地面）上起飞等于"往下扎 1 m"。
- **`height_offset_m` 不参与**这条（其余三条照旧参与）：从轨迹原点往上爬再加偏移没有意义。
- 速度用 `takeoff_speed_ms`（爬升速率），**不是** `max_speed_ms`（那条是全速上限）——
  所以 `traj_node` 打印这一条时说的是"速度"而不是"限速"。
- **斜坡**而不是一步到位：升速有界（= `takeoff_speed_ms`）。一步到位等于把爬升速率交给闭环自己定。

★ **编号取 4 不是 0**：`/traj/select` 的 **0 是「清空」**（既有语义，`fly.py` 的「取消选择」也用它）
⟹ 新轨迹用下一个空号。

★ **8 字是偏扁的**：`x ∈ [−1.0, +1.0]`（宽 2.0 m）、`y ∈ [−0.5, +0.5]`（高 1.0 m）——
Gerono 双纽线的 y 幅度**只有 x 的一半**。这是**规格不是缺陷**（§5.2 写了）；想要两个方向都 ±a 要换
Bernoulli 双纽线。测试里有一条不变式 `max(x)/max(y) ≈ 2` 直接钉住"偏扁"这件事。

> ★★ **2026-10-08：轨迹 3 放大到原本的 2 倍**（`a`: 0.5 → 1.0）。操作者裁定，动捕棚容得下 2 m 级。
> 这是一条**相似变换**：宽 1.0→2.0 m、高 0.5→1.0 m、周长 3.05→6.10 m、一圈时长 ≈6.1→≈12.2 s；
> **限速一个字没改**（弧长匀速，仍 0.5 m/s）⟹ 只有一圈变长。轨迹 1/2 不受影响。
> 记录：[`.scratch/phase2/issues/27`](../../.scratch/phase2/issues/27-lemniscate-doubled-and-downstream-labels.md)。

## ★ 速度上限是"弧长匀速"，不是"参数匀速"

- 正方形：按弧长 `s = clamp(t)·v` 推进，`s / 边长` 取第几条边，拐角处不减速（不要求停）。
- 八字：先用 **20000 点的数值表**建 `θ → 弧长` 的映射再反查。
  原因是 `x = a·sinθ` 的速度因子 `√(cos²θ + cos²2θ) ∈ [0.661, 1.414]`（均值 0.970）：
  **按 θ 匀速时过中心那一点是平均速度的 1.46 倍** —— 把"平均"压到 0.5 m/s，峰值就到 0.73 m/s，**违反上限**。
  弧长表太稀会让速度出现**波纹**：分段线性反查使每段内 `dθ/ds` 是常数，而真值在段内是变的。
  实测（中心差分 `dt=1e-5`、步长 0.01 s）：**2000 点 = 1.09e-3 m/s，20000 点（默认）= 9.8e-5 m/s**，
  200000 点 = 3.7e-6 m/s ⟹ 线性收敛，默认值比 §5.3 的 15 cm 判据小四个数量级。

## 参数（`config/traj.yaml`）

| 参数 | 默认 | 说明 |
|---|---|---|
| `reference_topic` / `progress_topic` / `select_topic` / `start_topic` / `marker_topic` | `/traj/*` | 话题名一律走参数（多机复用） |
| `reference_frame_id` | `"traj_origin"` | **纯标注**：mavros 的 `setpoint_raw` 只读 `coordinate_frame` 与 `type_mask`，**从不读 `frame_id`**（第一手：`mavros/src/plugins/setpoint_raw.cpp:209-249`）。写它只为了 `ros2 topic echo` 时一眼看出"这是轨迹原点系" |
| `marker_frame_id` | `"map"` | ★ **必须是 RViz 有 TF 的 frame**（`/mavros/local_position/pose` 的 frame），写 `traj_origin` 会让 RViz 直接丢掉它。所以折线**形状与尺寸对、落点相对动捕原点**——校形状够用，"落点对不对"在 `control` 里看 |
| `max_speed_ms` | `0.5` | §5.2 的硬要求。调大之前先想清楚失控后果 |
| `hover_duration_s` | `60.0` | 悬停没有终点而完成度必须是 0–1 ⟹ 用时长兜。默认 60 = §5.3 的验收时长 |
| `square_side_m` / `lemniscate_a_m` | `1.0` / `1.0` | 尺度。★ 8 字的 `a` 2026-10-08 由 0.5 放大到 1.0（见上） |
| `takeoff_height_m` / `takeoff_speed_ms` | `0.2` / `0.2` | ★ **只给轨迹 4 用**（起飞高度与爬升速率）：0.2 m 用 1.0 s 到顶。⚠️ 它**不**受 `max_speed_ms` 管，也**不**参与 `height_offset_m`（见上） |
| `height_offset_m` | `0.0` | ★ 见上，**改它之前先读那段** |
| `rate_hz` | `50.0` | 与 `control` 的 setpoint **同频**：同频才能一拍一换 |
| `marker_period_s` / `marker_points` | `1.0` / `201` | Marker **是 volatile 的**（后起的 RViz 看不到已发的那条）⟹ 周期重发；1 s 是"够 RViz 起来"与"不刷屏"的折中 |
| `startup_kind` | `0` | 启动即选（离线核对形状用）。**飞的时候保持 0**：选轨迹是操作者的动作，不该藏在 yaml 里 |
| `yaw_rad` | `0.0` | **恒定 yaw**。若改成"沿轨迹切向"，8 字中心交叉处会要求 180° 掉头（飞机做不到，会甩）；§8.6 的 yaw 前馈本来就"先不做" |

## 怎么用

```bash
# 只起 traj：看参考与折线，不需要飞控、不需要动捕、不需要 control
ros2 launch traj traj.launch.py
# 一键起 mocap+traj+control（+ 可选 mavros）—— ticket 09 落地，用法见 src/bringup/README.md
ros2 launch bringup z0.launch.py

# 选轨迹：1=悬停 2=正方形 3=8 字 4=起飞 0=清空（**不需要重新 launch**）
ros2 topic pub --once /traj/select std_msgs/msg/UInt8 "{data: 2}"
ros2 topic pub --once /traj/start  std_msgs/msg/Empty "{}"

ros2 topic hz   /traj/reference          # 应稳定 50 Hz
ros2 topic echo /traj/progress           # 0 → 1
rviz2                                     # 加 Marker 显示，看形状与尺寸（Fixed Frame = map）
```

⚠️ 手打这两条时**顺序不能反**：先 `select` 后 `start`（只 `start` 会收到一句 WARN「还没有选轨迹」）。
★ 而"顺序"在**程序里**是不能靠先后两次 `publish` 保证的 —— 两条话题是两个 DDS writer，
彼此没有顺序保证；所以 `select` 之后若 `start` 被先处理，DUT 就只 WARN 一句然后静默下去。
测这类流程要么等一个**可观测的副作用**，要么按"重发到看见结果"处理（见 `test/test_traj_node.py` 的
`TrajCommander.start`）。

## 测试（两层，都不需要飞控/动捕）

```bash
colcon test --packages-up-to traj         # 38 例（seam 2 的 26 + seam 1 的 12）
# ⚠️ 必须 `--packages-up-to`：seam 1 依赖 `bench` 夹具。若 bench 已经 build 过、且
#    `install/setup.bash` 已 source，`--packages-select traj` 也能跑。
# 📌 `colcon test-result` 的汇总会比真实用例数**多几个** —— 它把 xunit 里的**汇总节点**也数了一遍，
#    真实用例数是 38（`grep -c '^TEST' test/test_trajectories.cpp` 与
#    `grep -c '^def test' test/test_traj_node.py`）。
```

- **seam 2**（`test/test_trajectories.cpp`，零 ROS，26 例）：几何、闭合、弧长均匀性、限速扫四条轨迹、
  起飞的斜坡（`z(0)=0`、到顶 = `h`、斜坡内 `ż = v`、到顶后 `ż = 0`、`height_offset_m` 不参与）、
  周长（含一条独立 Simpson 法交叉核对——**用另一条算法算同一个数**，不重复实现里的公式），
  以及**放大到 2 倍**那条相似变换（周长/时长恰好翻倍、限速不动）。
- **seam 1**（`test/test_traj_node.py`，12 例，用 `bench` 夹具把节点当子进程起）：
  `select`/`start` 语义（**选了不 start ⟹ 一帧都不发**，并配了正控）、消息口径
  （`type_mask = 2496`、`FRAME_LOCAL_NED`、`frame_id`、恒定 yaw）、50 Hz **窗口**频率、
  正方形四条边且跑完停回原点、8 字幅度与偏扁不变式、RViz 折线的形状/尺寸/**周期重发**、
  起飞那条在话题上真的是一条斜坡且到顶后停住（连 `progress` 按时长走一起钉）。

> ★ **差分速度必须用 `header.stamp`，不能用"收到消息的时刻"。** rcl 的定时器从**回调开始**
> 重新计时，回调一超时就立刻补跑 ⟹ 接收时刻上会出现相邻仅 0.1 ms 的两帧，而它们之间隔着
> 0.1 s 的运动 —— 用接收时刻算出来正方形"飞到 4.5 m/s"（限速的 9 倍），全是假的。
> 频率断言同理，只在**窗口**上算（`src/bench/README.md` 的纪律 2）。

> ⚠️ **别在同一 ROS 域上同时跑两份 bench 测试** —— 两个 `Scenario` 的 DUT 会互相看见：
> 一份测试发的 `/traj/select` 会到达另一份的 DUT，症状是**负向断言随机变红**（"选了没 start 却收到参考"这种）。
> 实测踩到过：一边 `colcon test` 一边手动跑同一个文件，3 条"不该发生"的断言红了；换
> `BENCH_ROS_DOMAIN_ID=78 python3 -m pytest ...` 立刻全绿。**同机并行跑就各给一个域号**
> （夹具默认 73，见 `src/bench/README.md` 的安全边界）。

## 目录

```
src/traj_node.cpp             只做接线：参数、话题、定时器、RViz 折线
include/traj/trajectories.hpp ★ 四条轨迹的**纯函数**，零 ROS（seam 2）；弧长参数化都在这
test/test_trajectories.cpp    纯函数单测（seam 2，26 例）
test/test_traj_node.py        节点级（seam 1，12 例）
config/traj.yaml              参数（逐条带理由，改参数前先读注释）
launch/traj.launch.py
```
