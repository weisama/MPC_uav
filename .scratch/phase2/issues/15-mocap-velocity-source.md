# 15: 10 状态模型的 `v` 没有来源 —— `/mocap/pose_enu` 是 `PoseStamped`，没有任何速度字段

Status: needs-triage
Type: task

**发现于** [ticket 06](06-mpc-node-fake-px4-fixture.md) 实现期间（2026-09-28）。**不是** 06 的验收项缺失，
是 06 落地"状态外推 `x + v·τ`"这条时，顺着 `v` 往上找，发现**上游没有这个量**。

## 事实（逐条第一手）

1. **模型的初值里有 `v`，且它不是可选的**：`x = [p(3), v(3), q(4)]`，`nx = 10`
   （[`model.py:49`](../../../src/nmpc/nmpc/model.py) 的 `NX`，推导见
   [`phase2-implementation.md`](../../../docs/control/phase2-implementation.md) §2.4 的 `x = vertcat(p, v, q)`）。
2. **状态源 `/mocap/pose_enu` 只有位姿**：
   `create_publisher<geometry_msgs::msg::PoseStamped>(pose_enu_topic_, rclcpp::QoS(10))`
   —— [`mocap_rel_pose_node.cpp:93`](../../../src/mocap/src/mocap_rel_pose_node.cpp)。
   `PoseStamped` 里**没有** `twist`。
3. **于是节点侧只能拿到 `v ≡ 0`**：`node_core.cycle()` 用 `getattr(mocap, "twist", None)` 取速度，
   缺字段就退化成一串 0（[`node_core.py`](../../../src/nmpc/nmpc/node_core.py) 里 `velocity` 那三行）。
   这个退化是**刻意**的（缺字段不许崩、不许瞎猜），但它现在**每拍都发生**。
4. **速度其实一直都在，只是没被 republish**：`vrpn_listener` 同时发布
   `/vrpn/<刚体>/{pose,twist,accel}`，其中 twist 是 `geometry_msgs/TwistStamped`、
   `frame_id = "world"`（与 pose 同一个 frame）、`header.stamp = listener->now()`
   —— [`vrpn_listener.cpp:46-65`](../../../src/vrpn_listener/src/vrpn_listener.cpp) 与
   `:190` 的 `loadParam("frame_id", "world", …)`。文档侧同样记着它"动捕自算的，最干净"
   （[`mocap-setup.md`](../../../docs/reference/mocap-setup.md) §6 的速度路由表）。
   ⚠️ 该文档里"本项目不用"指的是**不要 relay 进 mavros/EKF2**（那是另一条理由，见 §6 的警告），
   **不是**"Phase 2 不能用它"。

## 为什么值得单开一张（后果不是"少一点精度"）

- **每拍的初值都在说"没在动"**：`v0 = 0` 在 1 m/s 时就是 **100 % 的 `v` 误差**，
  而 OCP 的预测正是从这个初值往前滚的。位置保持（`hold_offset`）时它表现为对扰动的
  响应偏钝/相位错；轨迹跟踪（ticket 09 的 B 档起）时会变成系统性的前馈缺失。
- **D9 的那"一行外推"现在是死的**：`x + v·τ` 里 `v = 0` ⟹ 外推等于恒等变换。
  票面 [06](06-mpc-node-fake-px4-fixture.md) 要求"τ = 50 ms 一行实现"，
  实现与单测都在（`test_node_core.py` 里 τ 的往返与正控各一条），但**整条链上它是惰性的**。
- **它同时也是"夹具不可能覆盖"的那一类**：`bench` 的 `VrpnSource` 必须照抄真身的消息类型
  （`PoseStamped`，见 [`vrpn_source.py`](../../../src/bench/bench/vrpn_source.py) 的文件头注释）
  —— ROS 2 里消息类型不匹配就是**不连接**，所以夹具**不能**偷偷塞一个带 twist 的 PoseStamped。
  ⟹ seam 1 上**无法**黑盒验证 τ 那条路（[06](06-mpc-node-fake-px4-fixture.md) 的
  `test_mpc_node.py` 里已写明"这条测试为什么不存在，等哪张票"）。

## 候选取法（**要人裁定**，别让 agent 自己挑）

| # | 取法 | 代价 / 风险 |
|---|---|---|
| **A** | `mocap` 节点**多订阅一条** `/vrpn/<刚体>/twist`，原样 republish 成 `/mocap/velocity_enu`（`TwistStamped`，frame 不变 = `"world"`） | ✅ **同一 frame ⟹ 零变换**（与 M7 的"输入侧零转换"一致）；改动 ~10 行；与 pose 同步（`vrpn_listener` 的 synchronizer 已经把 pose/twist 对齐过）⚠️ 需确认"同步"是否也覆盖到新话题的时刻 |
| **B** | MPC（模型预测控制）节点直接订阅 `/vrpn/<刚体>/twist` | ❌ 让 Phase 2 越过 `mocap` 节点直达 VRPN 层（`mocap` 包存在的意义就是隔离它）；且刚体名要进 MPC 的参数 |
| **C** | 在节点里对 `p` 做数值差分 | ❌ 差分放大噪声（`mocap-setup.md` §「`twist` 的 angular 全 0、linear 有噪声级读数」说明动捕侧读数本身就带噪）；且与"动捕已经算好了"重复劳动 |
| **D** | 用 PX4 的 `/fmu/out/vehicle_local_position`（EKF2 融合速度） | ❌ 那是 **NED**，要在输入侧做坐标变换 —— 正是 M7 花力气删掉的东西；且把 Phase 2 的状态源又绑回飞控 |

**倾向 A**（理由：它在**既有架构**里就位、零变换、且不动 MPC（模型预测控制）节点的接口），但这条改变了
`mocap` 包的输出面、也让 Phase 2 的状态源从"一条话题"变成"两条话题"，属于
[`phase2-implementation.md`](../../../docs/control/phase2-implementation.md) §4 那张接口表的改动 ⟹ 归人。

## 与 [06](06-mpc-node-fake-px4-fixture.md) 的边界（别把这张并回去）

06 只需交付"给定 `v` 时外推正确"（已有单测）与"缺 `v` 时不崩"（已有单测 + 集成测试断言
`v=0` 的行为）。**这张票要解决的是"从哪里拿到 `v`"**，那是接口决定，不是 06 的实现细节。
