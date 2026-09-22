# 19: `/mocap/pose_enu` 上出现过**来历不明的发布者** —— 它喂饱了 `control` 的 mocap 守卫

Status: needs-info
Type: task

**为什么单开一张：** [ticket 13](13-mavros-bench-vision-pose.md) 台架时抓到一条**说不通的事实**：
某个 `control` 进程从 `/mocap/pose_enu` 收到了消息，而**那个话题唯一的合法发布者（`mocap` 节点）当时没有任何发布路径**。
它不是 [ticket 04](04-mocap-relative-pose.md) 的代码缺陷（`mocap` 节点源码已通读，发布点只有一个），
也不是 13 的验收项 —— 它是一条**独立的安全相关线索**：**"动捕有数据"这条守卫可以被一个来历不明的发布者满足。**

## 第一手事实（2026-09-22）

`~/.ros/log/control_node_58161_1790060699586.log`（进程起于 `07:04:59.62`）：

| 本机时刻 | 行（节选） |
|---|---|
| `07:04:59.64` | `→ MANUAL/INIT … 动捕无数据（/mocap/pose_enu 静默 从未收到过）` |
| **`07:05:00.58`** | `→ MANUAL/STANDBY … 前置条件全成立（飞控已连接 ∧ **动捕有数据** ∧ local_position 有效 ∧ arm 守卫成立）` |
| `07:05:03` | 解锁 `MAV_RESULT=0` → `07:05:04` TAKEOFF → `07:05:07` HOVER 已到高 → `07:05:11` WAIT_START → `07:05:13` `SIGINT` |

**这比 `mocap` 节点自己记录的恢复时刻早 22.4 s**：
`~/.ros/log/mocap_rel_pose_node_48895_1790058454153.log` 里，
该节点 WARN `动捕数据已恢复（中断了约 800.2 s）` 的时间戳是 **`1790060722.957` = `07:05:22.957`**，
且这条 WARN 的**打印位置就在 `on_pose` 内**（`now() - last_pose_time_`），
即"输入恢复后的第一帧"，与 ERROR 行里的 `最后一帧在本机时刻 1790059922.805` 自洽（差 800.152 s）。

**`mocap` 节点那段时间没有任何发布路径**（源码通读，`src/mocap/src/mocap_rel_pose_node.cpp`）：

- 唯一发布点 `:196-197`（`publish(vision_pose_pub_, …)` / `publish(pose_enu_pub_, …)`），**在 `on_pose` 内**；
- `on_pose` 只由 `/vrpn/z0/pose` 的订阅（`:84-86`）触发 ⟹ **输入断流 = 零发布**；
- `on_state`（`:126-130`）只写 `has_state_/armed_`，**一个字节都不发**；
- 全节点只有两个订阅：`/vrpn/z0/pose`（`:84-86`）、`/mavros/state`（`:87-89`）；
- `last_pose_time_` 只有一个写点（`:169`，也在 `on_pose` 内），无遮蔽、无定时器重发。

## 已排除的候选（每条都有第一手证据，2026-09-22 `07:29`–`07:31` 实跑）

| 候选 | 排除证据 |
|---|---|
| 第二个发布者 | `ros2 topic info -v /mocap/pose_enu` → `Publisher count: 1`，`Node name: mocap_rel_pose` |
| mavros 顺带发 | `ros2 topic list` 里 mavros 只有 `/mavros/mocap/pose`、`/mavros/mocap/tf`、`/mavros/fake_gps/mocap/tf`；且**新** mavros 现在活着而话题上仍只有 1 个发布者 |
| `control` 节点自己 | 它的发布者只有 `setpoint_pub_`（`px4_interface.cpp:124`）与 `state_pub_`（`control_node.cpp:185`） |
| 订阅/槽错位（A→B 喂） | `mocap_sub_` 建在 `topics_.mocap` 上、`on_mocap` 写 `slot(topics_.mocap)`（`px4_interface.cpp:117-119`、`:228`），逐行核对无错位 |
| QoS 错配（静默收不到） | `mocap` 节点的订阅与 QoS 自 `06:27:33` 起未变，同一订阅在断流**前后**都收到 ~200 Hz |
| `use_sim_time` / `/clock` | `mocap` 节点日志前缀与 `date` 一致（`1790058454` = `06:27:34`，与 launch 目录名 `2026-09-22-06-27-34` 吻合）⟹ 无仿真时钟 |
| 第二份工作区 / 第二个 `mocap` 进程 | `ps -eo pid,lstart,cmd` 全量过滤：只有一个 `mocap_rel_pose_node`（pid `48895`，`06:27:33` 起） |
| rosbag 回放 | `find` 全盘 `.db3/.mcap/.bag`：无 |
| 本仓库的 bench fixture | `src/bench/bench/pytest_support.py:41-42` 与 `scenario.py:114-116` **硬拒** `ROS_DOMAIN_ID=0`（真 mavros 在 0 域） |
| cron / systemd / rc.local / 自启 | 都查过：无 |
| 远程 ROS 2 参与者 | AF_PACKET 抓包（`/tmp/z0_dds_peers.py`）：DDS 端口段（7400–7600）**无非本机流量** |
| 用户的工具 | 已当面问过操作者：「我不知道，没弄过这个东西」 |

**仍然排除不掉的一条**：某个**只在那段时间存在**的进程（旧 mavros `47747` 是 `07:06:28` 被杀的唯一"窗口内存在、现在没有"的 ROS 进程，
但它的插件表里只有 `mocap_pose_estimate`，话题名是 `/mavros/mocap/*`，与 `/mocap/pose_enu` 不同名）。

## 为什么这是缺陷级的

- **守卫可被伪造**：`control` 的"动捕有数据"判据（`guards().mocap_fresh` ⟸ `slot(topics_.mocap).ever/last_rx`）
  会被这个话题上**任何**发布者满足，不问来路。今天那个 `58161` 就是靠它进了 STANDBY 并走完一次任务；
- **但不污染控制律**：位置来自 EKF2 的 `/mavros/local_position/pose`，mocap 只喂 EKF2 的视觉输入；
  同一时刻 `/mavros/vision_pose/pose` 上 `Publisher count: 1`（仍是 `mocap_rel_pose`）
  ⟹ **EKF2 那 800.2 s 确实没有视觉**（这也是 `degrade=mocap_timeout` 想表达的）；
- **观测能力不足**：`control` 的日志只写"发布者：**N 个**（QoS）"，**不写节点名**
  （`control_node.cpp:835-870` 的 `publisher_summary`）⟹ 事后无法从日志指认。这是可改的。

## 需要什么信息（这就是 `needs-info` 的原因）

**下一次台架**（动捕输入**必须是通的**，或至少与今天同样的"输入断流而话题上仍有帧"的状态），按顺序做：

1. **先把观测能力补上**（agent 可做，不需要硬件）：让 `publisher_summary` 打印
   `get_publishers_info_by_topic` 返回的 **`node_name()` / `node_namespace()`**（不只数量与 QoS）；
   同时 `mocap` 节点侧加一条"发布前的自检"日志（或在 `ros2 topic info -v` 里留档）。
2. **复现**：输入断流时（`ros2 topic hz /vrpn/z0/pose` 零帧）跑一个**只读**订阅者
   （`ros2 topic hz /mocap/pose_enu`，QoS 取 `reliable` 与 `control` 节点一致）。
   - **有帧** ⟹ 幽灵还在 ⟹ `ros2 topic info -v /mocap/pose_enu` 会直接给出**节点名**，当场指认；
   - **零帧** ⟹ 那个发布者是**瞬态**的，需要**在断流窗口内持续抓**
     （建议：`ros2 topic info -v` / `ros2 node list` 每 5 s 落一份带时间戳的文件，事后与 `mocap` 节点日志对齐）。
3. 若复现不到：把这条降级为"**已记录、未能复现**"，并把第 1 项的观测能力改动**照做**
   （它让下一次发生时可指认，成本很低）。

★ **不要**为了复现去改 `control` 的守卫语义 —— 先有指认能力，再谈判据该不该收紧。

## Comments

- **2026-09-22（agent，triage）**：`Status: needs-info` —— 事实充分、成因未知，
  **下一步需要一次台架窗口**（复现 + 指认发布者），而第 1 项的观测能力改动 agent 可以先做。
  相关：[ticket 13](13-mavros-bench-vision-pose.md) §实测 2026-09-22（异常的时间线原文）、
  [ticket 04](04-mocap-relative-pose.md)（`mocap` 节点的发布路径）。
