# 02: acados 在 Orin NX 上编译通过

Status: ready-for-agent
Type: task

**What to build:** 一个在 z0 的机载电脑上**真跑得起来**的 acados —— 三个"任一漏掉就编译失败或性能掉一个量级"的配置项全部到位，且**解得动**（不只是装上了）。

**Blocked by:** 无（可立即开工）

- [ ] `ACADOS_WITH_OPENMP=OFF`（Orin 上 OpenMP 与 acados 的已知冲突）
- [ ] BLASFEO target = `ARMV8A_ARM_CORTEX_A76` —— Orin NX 实为 Cortex-**A78AE**，A76 是可用集中最接近的；用 `X64_AUTOMATIC` 或 `GENERIC` 会**掉一个量级**
- [ ] `t_renderer`（Tera）可用 —— 缺它会在生成代码时报 `EOFError`，而**这个报错不指向根因**（本张的价值之一就是把它的真因记下来）
- [ ] 用参考实现自带的例子（质量 1 kg / 悬停油门 0.73 的 x500 配置）能创建 `AcadosOcpSolver` 且 `solve()` 返回 0
- [ ] 记录本机实测求解耗时的**量级**（不是验收线本身；验收线在 [04](04-offline-ocp.md) 与 [09](09-sitl-tier-a-closed-loop.md)）
- [ ] 安装步骤与三个坑的报错原文写入文档，使后来者不必重踩

---

**无阻塞，且与 [01](01-sitl-px4-msgs-baseline.md) 完全并行** —— 两者一个是仿真/中间件侧，一个是求解器侧，不共享任何前置。
