# 02: 决定遥控通道分配 —— 开始 / 急停 / 解锁

Status: ready-for-human
Type: grilling

**What to build:** 一张**写死的决定**：开始信号用哪个通道、急停用哪个通道、两者**是否复用同一通道**、以及**解锁走遥控器还是程序**。

这是 spec 里**唯一一处刻意不在 spec 层解决的设计决定**（Q19），点名交给 ticket 化步骤产出。它必须在 Phase 1 实施到 FSM 的**「开始信号」之前** resolved —— 否则安全机制缺一条腿。

**已固定的约束（不可协商）：**
- 急停/开始信号**不得**与 PX4 模式开关（`RC_MAP_FLTMODE`）共用通道 —— 否则按下去会同时切换 PX4 飞行模式，把我们踢出 OFFBOARD
- 已知 `RC_MAP_FLTMODE` 已占用一个通道；[`params-analysis.md`](../../../docs/reference/params-analysis.md) §3.15 的完整占用表（含 ch5 解锁、ch8 杀车）显示**实际只剩 ch7 空闲**
- **在此之前，实现一律参数化**（`start_channel` + `start_threshold` + 200 ms 去抖），不写死任何通道号

**Blocked by:** 无（可立即开工）—— 占用表已足以支撑决策。

⚠️ **但结论必须在台架上用 `/mavros/rc/in` 核对一次**（那一步在 [`09`](09-track-mission-script.md) 的台架环节）。若核对结果与决定不符，回到本张修改决定，而不是就地改代码。

- [ ] 开始信号：通道 + 阈值 + 去抖参数写定
- [ ] 急停信号：通道写定，**或**明确说明为何与开始信号复用同一通道
- [ ] 解锁方式（遥控解锁 vs 程序解锁）写定
- [ ] 显式确认与 `RC_MAP_FLTMODE` 不冲突
- [ ] 结论写入文档（[`params-changes.md`](../../../docs/reference/params-changes.md) 新增一节，或 [`CONTEXT.md`](../../../CONTEXT.md)），并从本文件回链

---

**为什么这张不是 `ready-for-agent`**：需要真遥控器在手，且核心是人的取舍（复用还是分开、解锁交给谁），不是给定条件下的推导。
