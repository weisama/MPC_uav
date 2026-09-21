# 10: 参数实测定稿并回填

Status: ready-for-human
Type: task

**What to build:** 把 [04](04-offline-ocp.md) 里的**占位参数**换成**实测值**，并复验模型自洽。

`hover_thrust` 是**唯一**"测不准就飞不起来"的参数 —— 其余待测参数都有可用初值可以先顶住。

**Blocked by:** Phase 1 的悬停实飞日志（**跨 feature 硬依赖**，不是本 feature 的 ticket：[`.scratch/phase1/issues/08`](../../phase1/issues/08-takeoff-hover-land.md) 与 [`09`](../../phase1/issues/09-track-mission-script.md)）。⚠️ 但它不阻塞 [01](01-sitl-px4-msgs-baseline.md)–[09](09-sitl-tier-a-closed-loop.md) 的开工，只阻塞 [11](11-sitl-tier-b-seam3.md) 的**量化判据定稿**。

- [ ] `hover_thrust` 从 Phase 1 悬停日志**反推**（取 PX4 实际输出的归一化推力），不是抄参考机架的值 —— 本张的实测输入就是这条
- [ ] `T_max` 按**电池**复核（拉力台，或从悬停油门裕度估）。理论值 ≈ 11.8 N 可**先顶用**，但定稿要有实测依据
- [ ] `ω_max` 三项与 `z0.params` 的 `MC_ROLLRATE_MAX` / `MC_PITCHRATE_MAX` / `MC_YAWRATE_MAX` **逐项对齐**；偏航**单独给**，不取横滚的一半
- [ ] ★ 模型里的 `hover_thrust` 与推力归一化常数**互相自洽** —— PX4 对 `thrust_body` 不做任何重标定，两者不一致就是**静默的增益错误**
- [ ] 最大倾角收到 30° 的**量纲依据**（电池限制的 `T_max` 下，不同倾角所需推力占比）写进文档 —— 使未来换电池时知道该改哪一步
- [ ] 实测值回填 [`phase2-implementation.md`](../../../docs/control/phase2-implementation.md) §5 的待测参数清单，并同步 04 的代码
- [ ] ★ **换高 C 电池是独立的硬件决策** —— 参数化隔离即可，**不阻塞软件实施**
- [ ] ⚠️ 本张只定稿 **z0 的真实参数**；"这个 MPC（模型预测控制）是否依赖精确模型"由 [12](12-mismatch-sweep.md) 回答 —— **不要在本张顺手调参去掩盖失配**

---

**为什么是 `ready-for-human`**：需要真飞机与拉力台/悬停日志，且核心是"哪一条读数才是可信的"这种人的判断，不是给定条件下的推导。

**P6（遥控器急停通道的实测读数）不在这里** —— 通道分配的决定在 [`.scratch/phase1/issues/02`](../../phase1/issues/02-rc-channel-decision.md)，实机安全项归 Phase 3。
