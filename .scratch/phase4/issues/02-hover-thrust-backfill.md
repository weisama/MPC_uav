# 02: 悬停油门回填 PX4 并与 MPC（模型预测控制）模型核对自洽

Status: ready-for-human
Type: task

**What to build:** 把**已经测出来的**悬停油门同时落到 PX4 与 MPC（模型预测控制）模型两处，并留下一份「两处确实一致」的核对记录。

**Blocked by:** 跨 feature 硬依赖 —— [`phase2/issues/10-param-measure-backfill.md`](../../phase2/issues/10-param-measure-backfill.md)。悬停油门只能从 Phase 1 的悬停日志反推，那一步在 Phase 2 的 `ready-for-human` 票里。**本票不重新测量，只做落地与核对。**

**为什么必须同时落两处**：PX4 对 `thrust_body` **不做任何重标定**。只改一处 = 一个**静默的推力增益错误** —— 语法完全合法的消息，PX4 照做，我只有从飞行日志里才能分辨是哪一环错了。

- [ ] `MPC_THR_HOVER` 写成**实测值**（现值 `0.5` 恰好是 PX4 默认值，看不出被改过）
- [ ] 模型里的 hover thrust 与推力归一化常数**逐项一致**（改动清单里的 M4 那一类），核对记录留档
- [ ] 核对记录写明：两个值各是什么、从哪个实测数字来、**为什么它们必须相等**
- [ ] 与 `THR_MDL_FAC = 0.45` 的关系一并复核 —— 它是非默认值，说明这组参数被人动过，而它是推力模型的关键参数
- [ ] 若实测值与理论推算（`√(187.5/1020) ≈ 42.9%`）差得多，**记录差异**而不是取平均
- [ ] 改动带编号与理由，写进 [`phase2-implementation.md`](../../../docs/control/phase2-implementation.md) 并从本文件回链

---

**为什么是 `ready-for-human`**：改飞控参数要在 QGC 里做，且「哪个读数才是可信的」是人的判断。

*⚠️ spec 层约束：Phase 4 不与 Phase 1 / Phase 2 并行开工。*
