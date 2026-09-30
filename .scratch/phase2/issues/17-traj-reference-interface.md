# 17: 真正的轨迹接口 `/traj/reference`（06 交接、09 未做）

Status: done
Type: task

**What to build**：把参考从"参数化的悬停锚点 + `hold_offset`"换成一条**真正的轨迹接口**。

**来历**：[06](06-mpc-node-fake-px4-fixture.md) 的交接清单 §9 把"真正的轨迹接口 `/traj/reference`
（06 只有参数化参考点偏移）"归给了 [09](09-sitl-tier-a-closed-loop.md)；**09 没有做它**，
走的是**最小锚点路径**（`hold_reference` + 参数化 `hold_offset` + `anchor_yaw`），
理由写在 09 的 Comments #7：A 档只要"稳定悬停"一个工况，且 A 档的原则是"**每次只有一个
新变量**"—— 引入一条新话题链会让 09 最干净的那组对照（第 6 场 vs 第 8 场，只差 `u_ref` 一处）
不再干净。⟹ **这条交接仍然挂着**，需要先定归哪张票。

**为什么不能只换一条话题**（09 留下的两处 ⚠️，都是踩过的）：

1. **前馈要按参考自己的导数重算**：09 把 `u_ref` 从"上一帧解"改成
   `node_core.hold_feedforward`（**静止锚点**的解析前馈 `[T_hover, 0, 0, 0]`）。
   接运动参考后它必须换成那条参考自己的加速度/角速度前馈，否则
   `offline-ocp.md` §3.3 的实测（**持续角速率被压成 0**）会原样复发。
2. **判据要跟着换**：09 的判据（③④⑦⑨…）锚在 `hold_offset` 与"悬停段末 3 s"上。

**还要定的事**（2026-09-30 全部裁定，见 Comments #1）：

- [x] 参考轨迹的**来源**：**复用 Phase 1 的 `mavros_msgs/PositionTarget` 与话题名**
      （操作者裁定）。⚠️ 票面原先那句"直接用这个类型意味着把 mavros 拉进 Phase 2 的环里"
      **是错的**：`mavros_msgs` 的 `CMakeLists.txt` 只调 `rosidl_generate_interfaces`，
      不依赖 `mavros` 节点/插件/`libmavros`（⚠️ `package.xml` 里那条 `libmavconn` 是悬空声明）⟹ 见 Comments #2
- [x] 参考的**速度/加速度**要不要一起给：**速度要（掩码里那三位本来就是有效的），加速度不要**
      （默认掩码 `2496` 忽略它；且当前模型下"匀速"这一族**不需要**加速度）⟹ 见 Comments #3
- [x] 轨迹原点的接入方式：**节点加原点**（Phase 2 的锚点 = 闸门第一次打开那一拍的测量），
      与 Phase 1 把 `traj_origin` 定在 `control` 里同构 ⟹ 见 Comments #4
- [x] M6 的 `u_ref` 约定：**"调用方给的解析前馈"那一行，且 `hold_feedforward` 一个字都不用改**
      —— 当前这一族的参考本身就是模型的精确解 ⟹ 见 Comments #3

相关：[`mpc-node-seam1.md`](../../../docs/control/mpc-node-seam1.md) §9 的交接行、
[`tier-a-sitl-hover.md`](../../../docs/control/tier-a-sitl-hover.md) §3.4 的 ⚠️、
交付文档 [`mpc-node-traj-reference.md`](../../../docs/control/mpc-node-traj-reference.md)。

---

## Comments

### #1 ★ 开工授权与标签（2026-09-30，**操作者保留撤销权**）

本票的 `Status` 原为 `needs-triage`。**2026-09-30 操作者裁定"开工，标签我替你推"** ⟹
由会话代推为 `ready-for-agent`，收工时置 `done`。⚠️ **这次推标签是代操作者执行的**，
若操作者认为归属裁定不对，直接改回 `needs-triage` 即可（代码与文档不受影响）。

同批裁定的另外两条，边界写在这里免得后来人重新猜：

- **参考载体** = 复用 Phase 1 的 `mavros_msgs/PositionTarget` + `/traj/reference`（推荐档）。
  ⟹ `src/traj` **零改动**。
- **本票边界** = **能力为主，真飞归 11**：交付"轨迹跟踪能力 + 夹具级证据"；
  "三条轨迹在 SITL/真机上飞完"（§S5 判据②）**仍在** [11](11-sitl-tier-b-seam3.md)。
  ⛔ 本票**没有任何一条飞行读数**。

### #2 ☠️ 票面那条"会把 mavros 拉进环里"的担心**不成立**（第一手）

票面"还要定的事"第一条写着：Phase 2 走 uXRCE-DDS + 机体系角速率、**没有 mavros 那半边** ⟹
直接用这个类型意味着把 mavros 拉进 Phase 2 的环里。**查过 `mavros_msgs` 的构建：
它的 `CMakeLists.txt` 只调 `rosidl_generate_interfaces`**，全文件没有
`find_package(mavros)` / `libmavros` / 插件 ⟹ 依赖它只把**几个 `.msg` 的类**拉进来，
不拉任何 mavros 侧的运行时，与"DDS 走哪条"完全无关。

⚠️ **一处要说准**（否则就是把话说满了）：它的 `package.xml:34` 里有一条
`<build_export_depend>libmavconn</build_export_depend>`，而它自己的构建**从不使用**它 ——
全仓没有 `find_package(libmavconn)`，本机 Humble 里 `/opt/ros/humble/share/libmavconn`
**根本不存在**。⟹ 那是一条**悬空声明**，作用只到"下游在**构建依赖图**里排在它后面"。
★ 且 `libmavconn` 与 `mavros` 节点**不是同一个包**；运行环里仍然没有任何 mavros 侧二进制。

另两条同向的证据：① 仓里的回放包（bags 3/4）**已经**把 `/traj/reference` 记成
`mavros_msgs/msg/PositionTarget` ⟹ 换类型就要为回放另写一层转换，而那种转换**没有真值可对**；
② 复用之后 `traj` 节点一个字都不用改 ⟹ 轨迹形状与 `traj_origin` 语义与 Phase 1 是同一份定义。

### #3 ★★ ⚠️ #1 的正解：**当前这一族参考本身就是模型的精确解**，前馈不用改

票面 ⚠️ #1 要求"前馈要按参考自己的导数重算"。**对着模型算过之后，结论是这条要求在本族参考下
不需要** —— 但**理由是新的**，不是"忘了改"：

本仓模型没有阻力（`v̇ = R(q)·[0,0,F]/m + g`）⟹ "**匀速直线 + 水平姿态 + `F = m·g`**"
**本身就是模型的一条精确解**。于是参考族取

    p_ref(t_k) = 锚点 + p_rel + v_ref·(k·dt)，  其余分量逐点相同，  u_ref = [m·g, 0, 0, 0]

时 `f(x_ref, u_ref) = ẋ_ref` **恒成立** ⟹ `hold_feedforward` 一个字都不用改。
判据是 `test_node_core.py::test_轨迹参考是模型的一条精确解`（拿模型的动力学往前推）。

⚠️ **"位置按 `v·t` 外推"不是锦上添花，它是让上面那条恒等式成立的那一项**：
换成零阶保持（位置钉在当拍参考上、速度却填 `v`）会得到一个**自相矛盾**的参考
—— 代价要它停在原地、速度项又要它以 `v` 前进，而它在话题面上**照样是一串正常的角速率**。
变异测试确认过：改成零阶保持，3 条用例当场变红。

☠️ **这条订正有边界**：它只对"当前这一族"成立。一旦参考带上**持续加速度**或**持续角速率**
（`IGNORE_A*` 被解开 / `yaw_rate` 被用），`hold_feedforward` 里那张表立刻作废，
那时换的是**参考族**（连同外推项），不只是 `u_ref`（`node_core.py` 的 ☠️ 段已写明）。
⚠️ `traj_node.cpp` 默认掩码 `2496` 忽略的正是加速度与 `yaw_rate` —— 所以这一族**恰好**是安全的，
而这个"恰好"是**协议的属性**，不是我们选的。

### #4 锚点 = 轨迹原点（判据口径的落点）

`reference_abs = 锚点 + p_rel`，锚点仍是**闸门第一次打开那一拍的测量**（`hold` 档已有的口径）。
第一手依据：Phase 1 的 `traj_origin` 就是 `control` 在 HOVER 判稳那一刻记的**自己当前位置**
（`phase1-implementation.md` §3.3 第一手表），**不是**动捕原点，理由是"偏移被自动吸收 ——
在同一个坐标系里以当前点为 0 发指令"。**加原点这件事发生在下游这一侧**。

⚠️ **由此的已知限制（写死口径，留给 11 实飞确认或推翻）**：`p_rel` 给的是**形状**，
绝对落点由"闸门打开时飞机在哪"决定 ⟹ 与"按动捕原点摆好的绝对轨迹"差一个**常值平移**。

### #5 参考断流 = **冻结**（位置守住 + **速度归零**），不是停发

依据 Phase 1 已有的裁定"缺一帧参考就保持上一个目标，不回退到原点"（`phase1-implementation.md` §3.3）。
新参数 `traj_stream_gap_s`（默认 `0.5 s`）。

☠️ **参数名一度是 `traj_timeout_s`，已改掉**：Phase 1 的 `control` 有**两级**
（`src/control/README.md:119`）—— `traj_stream_gap_s`（0.5 s ⟹ 保持上一个目标）与
`traj_timeout_s`（3.0 s ⟹ `degrade=traj_timeout`，放弃这一趟）。本节点做的是**第一级**，
沿用第一级的名字即可；用第二级的名字会让同一个名字在两栈里指**两个阈值、两种后果**
（0.5 s 守住 vs 3.0 s 放弃），现场照文档读数会读错一档。
⚠️ **本节点没有第二级**：冻住之后一直守着，直到别的停机条件把闸门关上 ——
理由是**本节点不是任务状态机**，它不认识"这一趟"是什么，"任务放弃"归 `control` 那一侧。

☠️ **只冻位置、不冻速度是错的**：外推项 `v_ref·(k·dt)` 会继续把参考往前推，参考自己飞走、
飞机照着追 —— 一个**不会报错**的漂移源。所以冻结必须把速度一起归零，且这一条单独有用例钉着。

★ 与"停发"是两条不同的路：停发 = 交给兜底阶梯，那是 [07](07-solver-failure-fallback.md) 的地盘，
**本票不碰**。台架判据是"**发着指令的冻结**"（setpoint 照发 + `ref_frozen` 为真 + `x_ref0_v` 归零 +
目标点守住），见 `test_mpc_node.py::test_参考断流之后冻结_而不是停发`。

### #6 ★ 夹具保真度的一处修正：`FakeTraj` 原先**漏发了 `velocity`**

真身 `publish_reference`（`traj_node.cpp:311-329`）填 `position` + `velocity` + `yaw`，
而本仓的 `FakeTraj` 原先把 `velocity` **整个漏了**（三格恒 0）。对只看位置的消费者
（Phase 1 的 `control`）**不可见**；对 Phase 2 的 MPC（模型预测控制）**不是** —— 时域外推要读参考自己的速度，
漏掉它 = **每一条轨迹都被当成"停在那儿"**，且完全不报错。已补 `velocity_fn`（默认为 0，旧用例不受影响）。

★ 这条与 `FakeMavros` 的"QoS 抄错只会永远收不到"同类：**夹具的"漏"比夹具的"错"更难查** ——
它不会让任何断言变红，只会让被测逻辑在一个**与真机不同**的世界里被验证。

### #7 台架纪律：domain 73 是全机公用，条件订阅那条**必须判差值**

第一手（2026-09-30）：`/traj/reference` 的**正常**订阅者里就有 Phase 1 的 `control_node`，
而台架默认跑在 domain 73 上。另一个会话跑 `src/control/test` 时它的 `control_node`
会让 `matched_subscribers()` **恒 ≥ 1** ⟹ 直觉写法 `== 0`（负向）**假红**、`>= 1`（正向）**假绿**。
本票的写法是取"**起 DUT 前后的订阅者数差**"（`start_dut=False` + 手工 `start_dut()`），
外来订阅者被同时计入前后两边而消掉。

★ 另外两条：① `start_dut` 是**台架**的开关、不是 DUT 的 ROS 参数，
一把抓塞进 `params` 会让 DUT 以"未知参数"当场退出（这条报错是设计出来的，见
`mpc-node-seam1.md` §2.5）；② 要一次干净的重跑：`BENCH_ROS_DOMAIN_ID=74 python3 -m pytest …`。
**方向是好的**：这类污染只会让断言变红、不会假绿，所以判据是"重跑一次"而不是"改断言"。

### #8 交付物

| 文件 | 改了什么 |
|---|---|
| [`src/nmpc/nmpc/node_core.py`](../../../src/nmpc/nmpc/node_core.py) | `REFERENCE_SOURCE_*` 与 7 位 `IGNORE_*` 常量、`traj_reference_fields`（逐轴读掩码）、`traj_reference`（锚点 + 相对量 + 按参考速度外推 + `freeze`）、`ControlConfig.reference_source`、`StopConfig.traj_stream_gap_s`、`cycle()` 新增 `no_traj_reference` 闸门、`describe_state` 的 4 个 `ref_*` 键 |
| [`src/nmpc/nmpc/mpc_node.py`](../../../src/nmpc/nmpc/mpc_node.py) | `/traj/reference` 的**条件订阅**（与 `state_source` 正交）、`reference_source` / `traj_stream_gap_s` 两个参数与校验（`traj` 档拒非 0 `hold_offset`）、启动日志列出该话题、逐帧日志加 `x_ref0_v` |
| `src/nmpc/package.xml` | `<depend>mavros_msgs</depend>`（附"纯消息包"的说明） |
| [`src/nmpc/test/test_node_core.py`](../../../src/nmpc/test/test_node_core.py) | **9 条**新用例（掩码逐位 / 精确解 / 外推 / 冻结 / 坏帧不刷新龄 / 没参考时闸门 / `hold` 档无视参考） |
| [`src/nmpc/test/test_mpc_node.py`](../../../src/nmpc/test/test_mpc_node.py) | **5 条**新用例（掩码对真类 / 条件订阅判差值 / 目标点被参考驱动 / 断流冻结 / 没参考时闸门 + 理由） |
| [`src/bench/bench/fake_traj.py`](../../../src/bench/bench/fake_traj.py) | #6 的保真度修正（`velocity_fn`） |
| [`docs/control/mpc-node-traj-reference.md`](../../../docs/control/mpc-node-traj-reference.md) | **新增**：本票的交付文档（口径 / 判据 / 能证明什么 / 不能证明什么） |
| [`docs/control/mpc-node-seam1.md`](../../../docs/control/mpc-node-seam1.md) | §2.1 补条件订阅那一行 + 两条正交条件订阅的 ★、§9 交接行收口 |

### #9 ★★ ⚠️ #2 的落点：**A 档判据在 traj 档逐条点名**（本票只点名，不换判据）

票面 ⚠️ #2 那条"判据要跟着换"原先在交付文档里**没有落点**（复核指出）。已补
[`mpc-node-traj-reference.md`](../../../docs/control/mpc-node-traj-reference.md) **§8.1**，
对 `tools/px4-sitl/verify_tier_a_hover.py` 里那几条**逐条**给出 traj 档下的结论：

| 判据 | traj 档下 |
|---|---|
| ③ 悬停高度 1 m ± 0.15（末段 3 s） | ☠️ **失效** —— 高度由 `锚点 + p_rel.z` 决定，没有"1 m"这个目标 ⟹ 改成**对参考 `x_ref0` 的误差** |
| ④ 无持续漂移（水平位移 < 0.5 m / 3 s） | ☠️ **失效** —— 参考本来就要它平移，这条会把**预期的机动**判成漂移 ⟹ 改成"误差的漂移" |
| ④ 无持续振荡（峰峰值 / 穿均值频率） | ⚠️ **半失效** —— 量应是**误差**的振荡，不是绝对高度 |
| ⑦ 冷启动砸在哪一拍（`warmup` 闸门） | ✅ **不变**，与参考从哪来无关 |
| ⑨ 偏航方向 / 换算 | ✅ **不变**，但要在**参考带 yaw**的场次上重跑一遍别名闸门 |

⛔ **换判据这件事本票没做，归 [11](11-sitl-tier-b-seam3.md) 的 §S5 判据②**。理由写死了两条：
① 阈值该取多少**现在没有任何飞行读数可依据**，拍一个数就是编；② 那些判据住在一个**属于 A 档的脚本**里，
而它当前**有别的会话的在制品**（同工作树），本票不碰。

### #10 ★ 本票**没有任何飞行读数**，别把它读成"轨迹跟踪已经能飞"

交付的是**能力 + 夹具级证据**（`test_node_core.py` 10 条、`test_mpc_node.py` 5 条）。
能证明的是：掩码逐位对得上真类、参考真的驱动了目标点与 `x_ref0_v`、断流冻结、
没参考时闸门不开且理由是 `no_traj_reference`、锚点+外推是模型的精确解。
**不能**证明的是：真机/SITL 上跟得住（那是 [11](11-sitl-tier-b-seam3.md) 的地盘，见 #9）。
