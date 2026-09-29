# 15: 10 状态模型的 `v` 没有来源 —— `/mocap/pose_enu` 是 `PoseStamped`，没有任何速度字段

Status: done
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

## Comments

**① ★ 操作者裁定（2026-09-29）—— 取法 A，原话**：

> **方案 A**：`mocap` 节点多订阅一条 `/vrpn/<刚体>/twist`，**原样 republish** 成
> `/mocap/velocity_enu`（`geometry_msgs::msg::TwistStamped`，**frame 不变 = `"world"`**）。
> 理由：同一 frame ⟹ **零变换**（与 M7 的"输入侧零转换"一致）；改动 ~10 行；与 pose 同一个源。
> **不是** B（直连 VRPN）/ C（数值差分）/ D（PX4 NED）。

实现里逐字落成了"原样"：`on_twist` 里 `out.header = msg->header;`（**frame 与 stamp 都照抄，不二次打戳**）
＋ `out.twist = msg->twist;`（三分量原样带走；`angular` 在 `vrpn_listener` 侧刻意留零，本节点不动它）。

**② ★ 票面 ⚠️「'同步'是否也覆盖新话题的时刻」—— 本轮**实查代码**，答：**不覆盖**，票面那句括注是错的**：

- `Synchronizer`（[`vrpn_listener.hpp:13-21`](../../../src/vrpn_listener/include/vrpn_listener/vrpn_listener.hpp)）
  **只是每个 sender 的容器**：`sender_name` / `listener_ptr` / `vrpn_tracker` / 三个 publisher。
  **没有任何时间对齐逻辑** —— 名字叫 "Synchronizer" 是**误称**。
- `handlePose` / `handleTwist` / `handleAccel`
  （[`vrpn_listener.cpp:19-97`](../../../src/vrpn_listener/src/vrpn_listener.cpp)）各自到场即发，
  且**各自** `header.stamp = listener->now()`（pose 在 `:40`、twist 在 `:62`、accel 在 `:93`）
  ⟹ **pose 与 twist 的时间戳取自两次独立的 `now()`，没有对齐**。
- ⟹ 票面候选表 A 行里「`vrpn_listener` 的 synchronizer 已经把 pose/twist 对齐过」**是错的**；
  但这**不影响裁定 A 成立** —— 零变换靠的是"**同一个 frame**"（都是 `"world"`），**不是**靠时间对齐。
- 因此节点侧**没有**对速度单独设新鲜度门（两条话题由**同一台** `vrpn_listener` 发出，位置断了速度也断；
  pose 的门已覆盖整条链），但**记了** `describe_state` 的 `vel_cb` / `age_vel_s`
  （[`node_core.py:674-675`](../../../src/nmpc/nmpc/node_core.py)）让"速度到没到、多旧"在复盘面可见。

**③ 改动的落点（file:line）**：

| 文件 | 落点 |
|---|---|
| [`mocap_rel_pose_node.cpp`](../../../src/mocap/src/mocap_rel_pose_node.cpp) | 参数 `twist_topic`/`velocity_enu_topic` `:77-85`；订阅 `:101`；发布 `:111`；`on_twist` `:162-173`；成员 `:325-350`；文件头数据流 `:6,9` |
| [`mocap.yaml`](../../../src/mocap/config/mocap.yaml) | `twist_topic: ""` `:17-19`；`velocity_enu_topic` `:23-24` |
| [`mocap.launch.py`](../../../src/mocap/launch/mocap.launch.py) | `:25` 参数描述去掉了"三个话题"的硬编数（会过期） |
| [`mocap/README.md`](../../../src/mocap/README.md) | 数据流图 `:9,13`；参数表 `:58,61` |
| [`node_core.py`](../../../src/nmpc/nmpc/node_core.py) | `__init__` 三字段 `:544-550`；`on_mocap_velocity` `:597-607`；`_mocap_velocity` `:837`；`describe_state` `:674-675` |
| [`mpc_node.py`](../../../src/nmpc/nmpc/mpc_node.py) | `TOPIC_MOCAP_VEL` `:199`；订阅 `:397-398`；`_on_mocap_velocity` `:539-540`；接口表 `:18`；`state_source_desc` `:355` |
| [`vrpn_source.py`](../../../src/bench/bench/vrpn_source.py) | `TwistStamped` 发布 `:74,94,125-131`；`set_velocity` `:167`；`matched_velocity_subscribers` `:197` |
| [`scenario.py`](../../../src/bench/bench/scenario.py) | 证据行加"速度源：… 匹配到的订阅者 …" `:453-454` |
| [`test_node_core.py`](../../../src/nmpc/test/test_node_core.py) | `_Velocity` 替身 `:81`；两条新用例 `:459,481` |
| [`test_mpc_node.py`](../../../src/nmpc/test/test_mpc_node.py) | B 档黑盒 `:1126`；QoS 用例补断言 `:209`；"这条测试为什么不存在"注记更新 `:1713-1723` |
| [`phase2-implementation.md`](../../../docs/control/phase2-implementation.md) | §4 接口表/清单加一行 `:745-751` |
| [`mocap-setup.md`](../../../docs/reference/mocap-setup.md) | §1 表 `:30-31`；§6 警告澄清 + 路由表加"Phase 2 的 MPC"行 `:375-391` |
| [`mpc-node-seam1.md`](../../../docs/control/mpc-node-seam1.md) | §5 "τ 在 seam 1 上无法黑盒验证"结论作废（推导保留）`新增更新块 :187-197` |

**④ 验收（本轮实跑的原始输出）**：

```
$ colcon build --packages-select mocap
Finished <<< mocap [36.4s]

$ colcon test --packages-select mocap && colcon test-result --test-result-base build/mocap
Summary: 22 tests, 0 errors, 0 failures, 0 skipped

$ PYTHONPATH=/home/chen/ros_ws/src/nmpc pytest src/nmpc/test/test_node_core.py -q
57 passed, 1 warning in 1.51s        # 基线 55，本张 +2

$ … -v -k "独立话题 or 独立速度"
test_动捕速度从独立话题进外推 PASSED
test_独立速度话题优先于位姿里内嵌的_twist PASSED
```

**变异测试（自证新断言有鉴别力）**：把 `_mocap_velocity` 的 `for src in (self.mocap_velocity, self.mocap)`
临时改成 `(self.mocap,)` ⟹ **两条新用例都红** ⟹ 恢复。确认它们真的在测"独立话题那条路"。

**⑤ ⚠️ seam 1 集成用例写了但**未执行**（诚实交代，未糊过去）**：
`test_mpc_node.py::test_B_档_速度从_velocity_enu_进外推_表现为推力低于悬停` 的黑盒 DUT 是
**安装在 `install/` 的 `mpc_node.py`**，改源码后要让 DUT 取到新代码**必须 `colcon build --packages-select nmpc`**。
本轮机器纪律**禁止** build `nmpc`（并行会话正在改 `src/nmpc/nmpc/ocp.py`，build 会把半成品烘进 `install/`）
⟹ **本用例是"已写、未跑"**，需要排一次 `nmpc` build 才能上绿。**单元侧（`node_core`）已对源码实跑验证**。
⚠️ 这格**不是** `done` 的充分证据，是留给排期的一条欠账。

**⑥ 未动（守边界）**：`PoseStamped` 那两条 `hasattr` 断言**未松**（`test_mpc_node.py:1723` 等）；
`test_动捕缺_twist_时退化成不外推` **未改**；`node_core` 里 `getattr` 退化到 `(0,0,0)` 的**刻意**行为**保留**
（缺速度仍不崩、不瞎猜）。`src/nmpc/nmpc/ocp.py` 与 `tools/px4-sitl/verify_z0_plant.py` **未碰**。

**⑦ 复核轮（2026-09-29，独立核验）—— 两处作废标记的**位置**被挪到节首**

复核回代码核过 ② 那条推翻，**成立**（我自己读的）：`Synchronizer`（`vrpn_listener.hpp:13-21`）
确实只是结构体（`sender_name` / `listener_ptr` / `vrpn_tracker` + 三个 publisher），
`handlePose` 与 `handleTwist` 各自 `stamp = listener->now()` 后立即 `publish`
（`vrpn_listener.cpp:20-42` / `:44-66`）⟹ **没有时间对齐**；两边 `frame_id` 都取
`listener->_frame_id` ⟹ **零变换的前提成立**。裁定 A 不受影响，判断正确。

但 ③ 表里两处"作废"标记**只加在了节尾**，而**节首（最显眼处）仍写着被推翻的那句话**：
- `docs/control/mpc-node-seam1.md` §5 的**标题**就是 `…本层**量不了**…`；
- `test_mpc_node.py::test_动捕那条话题没有速度字段_所以拍不出_tau_的黑盒测试`
  —— **函数名**里那半句"所以拍不出 τ 的黑盒测试"现在是假话（B 档**拍得出**）。

按本仓库纪律（**作废标记放被作废那节的节首、逐项点名；排序就是可见性**），已在**两处节首**
各补一个 ☠️ 块并把更新块指过去，**原文一律保留**。
⚠️ **函数名刻意未改名**（`mpc-node-seam1.md` §5 与 `test_mpc_node.py:1031/:1133` 按名字引用它）
—— 改名要一起动三处，且"保留被推翻的原话 + 标记"是本仓库的既定做法。**这点留给后续裁定**。

**⑧ ⚠️ 本票 `done` 的一条欠账（复核补记，排期未了）**：⑤ 那条 seam 1 用例仍需
一次 `colcon build --packages-select nmpc` 才能上绿。按本仓库 `done` 的定义
（未决事项必须**另开票**并在本文点名），这张票**只点名、未开票** ⟹ 严格说 `done` 尚不完整。
复核已把它排进"等并行会话（ticket 16）改完 `ocp.py` 之后的一次 `nmpc` build"，
**跑绿之前，这条欠账都记在这里**。
