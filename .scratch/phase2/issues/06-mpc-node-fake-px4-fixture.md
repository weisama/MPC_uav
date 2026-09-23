# 06: MPC（模型预测控制）节点骨架 + 假 PX4 夹具（seam 1）

Status: ready-for-agent
Type: task

**What to build:** 节点第一次完整成型，**以及 Phase 2 / Phase 3 所有节点级测试的公共底座** —— 一个假 PX4 + 脚本化动捕输入。

⚠️ **为什么夹具必须是假 PX4 而不是 SITL**：兜底路径（求解失败、非有限值、时间戳倒退）需要"**这一帧求解失败**"这种要求 —— **真 PX4 没法被要求，假件可以**。所以夹具必须能**在指定时刻注入故障**，这不是加分项，是它存在的理由。

**Blocked by:** [04](04-offline-ocp.md)

**本张负责的改动（M7 / M8 / M9）**：M7 状态源从 PX4 话题换成动捕 ENU 话题（**因此可删掉输入侧的坐标系转换**）、M8 删掉为绕过消息版本问题而写的双订阅 hack（`_v1` 后缀话题 / `msg/versioned/` 是 **v1.16 才有**的机制，在 **z0 真身（v1.14.3 系）**里**根本不存在**）、M9 补上定义了却没使用的死变量。

> ☠️ **2026-09-23 措辞更正**：M8 原写"在 **v1.15 世代的 fork** 里根本不存在" —— **结论本身不变**（真身里同样不存在），但**对象选错了**：那是拿 fork 当参照物。真身的 `msg/` 与**上游 v1.14.3 逐字节相同**，所以这句话更贴对象的说法是"**v1.16 才有**"。见 [`refs-manifest.md`](../../../docs/reference/refs-manifest.md) §2.6 执行版。

- [ ] 订阅 `/mocap/pose_enu`（**不是** NED）与 `/fmu/out/vehicle_status`（判断是否已 armed / offboard）
- [ ] 发布 `/fmu/in/vehicle_rates_setpoint` 与 `/fmu/in/offboard_control_mode`（`body_rate = True`，其余 False）
- [ ] **QoS**：pub `BEST_EFFORT + TRANSIENT_LOCAL + depth 1`，**sub 也必须显式 `BEST_EFFORT`**
- [ ] 时间戳用**微秒**
- [ ] 心跳 ≥2 Hz、实做 50 Hz；100 Hz 定时器（频率在 [09](09-sitl-tier-a-closed-loop.md) 实测最坏耗时后降档）
- [ ] **符号映射**：给定姿态与目标，断言三轴角速率的**相对符号**与 [03](03-sign-map-derivation.md) 一致
- [ ] **推力归一化**：给定状态与目标，断言 `thrust_body[2]` 的**数值与符号**；常数**从模型导出**而非硬编码（M2）
- [ ] **未解锁 / 未在 offboard 时 `不发布`**
- [ ] 约束饱和时表现为**打满**，而不是被静默截断 —— 否则分不清"MPC 在顶约束"与"MPC 在正常工作"
- [ ] **状态外推** `x + v·τ`（τ = 50 ms）一行实现；不做完整延迟补偿
- [ ] RViz：预测轨迹与参考点发布，能肉眼判断"MPC 是否在发疯"
- [ ] **回退**：本张之后 Phase 1 的 `(A)` 级链仍完整可用，一条参数命令即可切回 —— Phase 2 是**加一条支路，不是替换**
- [ ] **软急停接口参数化**（通道号不写死）—— ★ 通道号的**决定**由 [`.scratch/phase1/issues/02`](../../phase1/issues/02-rc-channel-decision.md) 产出；**本张不决定、不实测**（RC 夺权与失效保护的实机实测归 Phase 3）。
      ✅ **2026-09-21 该 ticket 已 resolved**，结论里有一条**改变本张前提**：**急停 = ch8 高值 = PX4 硬件杀车，程序不读急停通道**，即**没有"软急停通道"** ⟹ 本张的"软急停接口"要按 [`phase2-implementation.md`](../../../docs/control/phase2-implementation.md) §4 那张表里的**两条既有路径**（打杆接管 / ch6 模式开关）来理解，别去假设有一个专属通道
- [ ] 夹具能在 `colcon test` 里跑起来且至少一个冒烟场景通过；**能按指定时刻注入故障**（为 [07](07-solver-failure-fallback.md) 准备好）
- [ ] **不需飞机的静态自检**：一个在装桨之前就能跑的检查流程，使参数与符号错误在这一步暴露
- [ ] M7 / M8 / M9 的改动编号、理由与第一手依据已记入 [`phase2-implementation.md`](../../../docs/control/phase2-implementation.md)

---

★ **M7 顺带干掉一整类 bug**：既然动捕原生就是 ENU/FLU、MPC（模型预测控制）内部也是 ENU/FLU，那么参考实现在输入侧那段手写 NED→ENU 四元数变换（**整个文件里最容易写错的一行**）可以直接删掉。本张删掉它，而不是把它留着"以后可能有用"。
