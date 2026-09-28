# 17: 真正的轨迹接口 `/traj/reference`（06 交接、09 未做）

Status: needs-triage
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

**还要定的事**：

- [ ] 参考轨迹的**来源**：Phase 1 的 `/traj/reference` 是 `mavros_msgs/PositionTarget` +
      `frame_id = "traj_origin"`、**相对轨迹原点**（`phase1-implementation.md` §3.3 / §1079）。
      ⚠️ Phase 2 走 uXRCE-DDS + 机体系角速率，**没有 mavros 那半边** ⟹ 直接用这个类型意味着
      把 mavros 拉进 Phase 2 的环里
- [ ] 参考的**速度/加速度**要不要一起给（10 状态模型 + τ 外推需要 `v`；见
      [15](15-mocap-velocity-source.md) 对"`v` 从哪来"的同一类问题）
- [ ] 轨迹原点的接入方式是"节点加原点"还是"上游发绝对量"（Phase 1 的裁定是**由 `control`
      加原点**，理由见 `phase1-implementation.md` §3.3 的 ⚠️：轨迹原点是 1 m 高度上的悬停点，
      直接转发 = 先掉到地面附近）
- [ ] M6 的 `u_ref` 约定在**运动参考**下怎么落地（`offline-ocp.md` §3.2 的三行里，
      只有"调用方给的解析前馈"这一行能用）

相关：[`mpc-node-seam1.md`](../../../docs/control/mpc-node-seam1.md) §9 的交接行、
[`tier-a-sitl-hover.md`](../../../docs/control/tier-a-sitl-hover.md) §3.4 的 ⚠️。
