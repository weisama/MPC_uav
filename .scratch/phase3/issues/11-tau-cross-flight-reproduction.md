# 11: τ 的跨飞行复现 + B 口径验证（需另一段日志）

Status: ready-for-agent
Type: task

**What to build:** 给 [03](03-latency-tau-from-logs.md) 那个 **τ ≈ 50 ms** 补上**第二条独立读数**，
并把 03 里**没验成**的 B 口径验掉。

**Blocked by:** 需要**给飞控通电**，再从 SD 卡拉到**另一段带机动的飞行日志** ——
与 03 完全同一条**只读**取数链路（原生 MAVLink 日志协议，`/dev/ttyTHS1`），
见 [`docs/reference/mocap-setup.md`](../../../docs/reference/mocap-setup.md) **§6.3.2**。

**为什么必须有这一票**：03 的结论**只有一次机动**支撑（323.6—327.2 s 那一次偏航），
这是它自己写在 §6.3.5 里的**最大不确定性**。相关峰是平顶的，单次估计分辨率约 ±10 ms ——
所以现在只能说「≈ 50 ms 这个量级」，说不出更多。**第二条跨飞行的读数才把这个量级钉住。**

- [ ] 再取**至少一段**带机动的飞行日志（log 210 的剩余部分，或 SD 卡上另一条 id）
- [ ] **A 口径复现**：新日志上独立量一次 τ，看是否仍落在 ≈ 50 ms 这个量级；**给原始读数**，不要只给结论
- [ ] **验 B 口径**（加速度计 × `quat_body_down`）：需要一段**近乎悬停**的飞行 ——
      03 那段是**偏航为主**，而 `quat_body_down` 按构造对偏航免疫（激励σ 仅 0.02），所以 B 在 03 上是**无效**、不是错
- [ ] **建立 `id ↔ SD 路径` 的对应关系**：补一次 FTP 列表，把 03 里标着「SD 路径未建立」的那一行填上
      （MAVLink 日志协议**只给 id、不给路径**，这条路绕不过去）
- [ ] 两段飞行若给出的 τ **差得多**，写清是方法、窗口还是飞行本身的差别 —— **不要取平均**
- [ ] 结论回填 §6.3 与 [`tools/tau-analysis/evidence/`](../../../tools/tau-analysis/evidence/)，并从本文件回链

---

**要接着用的东西**（03 已经留好了）：

- 工具：[`tools/tau-analysis/`](../../../tools/tau-analysis/)（20 项自检；新日志直接喂给它就行）
- 读法：[`mocap-setup.md`](../../../docs/reference/mocap-setup.md) §6.3.5 的**不确定性表**
  与 [`tools/tau-analysis/README.md`](../../../tools/tau-analysis/README.md) 的「读分段」——
  **极差大不等于方法不稳**，先看每段的激励σ
- ⚠️ **不要为了这条去改 `EKF2_EV_DELAY`**。它属 [`params-changes.md`](../../../docs/reference/params-changes.md)
  **§D5「不要动」**，03 的实测只是**加强**了那条理由，不是推翻

**为什么是 `ready-for-agent`**：纯离线数据处理。通电与拉取那一步由操作者做（与 03 相同），
拿到日志之后的活不需要飞机、不需要人盯。

*⚠️ spec 层约束：Phase 3 不与 Phase 1 / Phase 2 并行开工。*
