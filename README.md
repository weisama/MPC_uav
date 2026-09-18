# MPC_uav — z0 无人机 MPC 控制

> **本仓库是 z0（OddityRC XI35 Pro 152 mm 涵道四旋翼）自主飞行栈的工作副本。**
> 飞控 MicoAir **NxtPX4v2**（PX4 1.14.x，飞控出厂自带）+ 机载 Jetson Orin NX 8GB + Nokov 动捕（VRPN）。
> 最终目标是 **MPC 接管飞控内置 PID**；分两阶段落地，见下。

---

## 两阶段路线

| 阶段 | 做什么 | 控制接口 | 是否完成 |
|---|---|---|---|
| **Phase 1** | **offboard 位置定点飞行** —— 用动捕让 z0 稳定悬停并跟踪简单轨迹，**PX4 跑全部内环 PID** | **(A) 级**：`/mavros/setpoint_raw/local`（位置 + yaw） | ⬜ 未开始 |
| **Phase 2** | **MPC 接管 PID** —— MPC 输出角速率 + 推力，PX4 只跑角速率环 | **(C) 级**：`/fmu/in/vehicle_rates_setpoint` | ⬜ 未开始，方案见 [docs/control/phase2-implementation.md](docs/control/phase2-implementation.md) |

**指导思想：每个阶段只引入一个新变量。** Phase 1 全走 mavros2（**完全不碰 uXRCE-DDS**），把"动捕链路 + 状态机 + offboard"先跑通；Phase 2 再加 MPC + acados + uXRCE-DDS，并且**先在 SITL 验证再上实机**。

---

## 目录

```
├── docs/    资料与文档
│   ├── PROJECT.md                    ★ 总纲（先读它）
│   ├── CONTEXT.md                    术语表（含 MPC 歧义澄清）
│   ├── LOCAL-NOTES.md                ⚠️ 本地凭据，已 gitignore，不入库
│   ├── control/
│   │   ├── phase1-implementation.md  ★★ Phase 1 实施清单
│   │   ├── phase2-implementation.md  ★★ Phase 2 实施清单
│   │   └── mpc-roadmap.md            Phase 2 资料汇总（含 D1–D10）
│   ├── reference/
│   │   ├── code-inventory.md         ★★ 代码盘点与取舍
│   │   ├── open-source-survey.md     开源项目调查（第一手 clone 核实）
│   │   ├── mocap-setup.md            ★ 动捕链路搭建手册
│   │   ├── px4-interface.md          动捕 / PX4 / Offboard 接口事实
│   │   ├── params-analysis.md        z0.params 逐项分析
│   │   └── hardware.md               硬件规格
│   └── assets/
│       ├── z0.params                 z0 飞控参数（928 行，待上机核实）
│       └── papers/                   6 篇参考文献 + 中文索引
└── src/     ROS2 Humble 代码（Phase 1 待建）
```

**先读哪个**：`docs/PROJECT.md` → `docs/reference/code-inventory.md`（代码取舍）→ `docs/control/phase1-implementation.md`（动手写代码前必读）→ `docs/CONTEXT.md`（查术语）。

---

## 关键约束（摘要，每条都有第一手证据）

| 约束 | 后果 |
|---|---|
| ⚠️ **z0 不在 PX4 官方支持矩阵内** —— `hkust_nxt-dual` 板级目标只在 **PX4 ≥ 1.15.0** 存在，z0 是 1.14.x | **不刷固件、不追上游，把 1.14 当既定事实** |
| ⚠️ **电池电流采样大概率读错通道** —— `nxt-dual` 的 `ADC12_CH(5)` 是错的，上游 `129c0f188a`（2024-11-19）才改成 `ADC12_CH(8)`，**v1.15.0 仍带 bug，v1.16.0/main 才修**。z0 是 1.14.x，**连这个板级目标都还不存在** | 上电后**在 QGC 核对电流读数**（恒为 0 或怪值 = 中招）；中招则电流/电量保护全是假的 |
| ⚠️ **动捕时间戳 `stamp = now()` 是刻意设计，不要改** | 动捕主机与 Jetson 有 **~4 秒**时钟差；改成 `msg_time` 会毁掉 EKF2 融合 |
| ⚠️ **`TrajectorySetpoint` 没有 `thrust` 字段** | 要发姿态+推力得用 `VehicleAttitudeSetpoint` |
| ⚠️ **推力是 `[-1, 1]` 归一化，不是 `0–1`、不是牛顿** | 数值约定错了，MPC 算得再对飞机也不会飞 |
| ⚠️ **`px4_msgs` 版本必须与固件严格匹配** | v1.15 及以前**不匹配会静默失败**（不报错，只是消息没人收） |

---

## 许可证与致谢

- 本仓库引用的第三方项目各自的许可证见 `docs/reference/open-source-survey.md`。
  ★ **注意**：`uzh-rpg/agilicious` 的许可证**禁止**再分发其代码，本项目**只引其论文与设计思想，不引用任何代码**。
- 参考论文索引见 [`docs/assets/papers/README.md`](docs/assets/papers/README.md)。
