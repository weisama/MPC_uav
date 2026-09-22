# 16: 「EKF2 位置失效」的**可用载体** —— ticket 06 ④ 在本链条上抓不到

Status: needs-triage
Type: task

**为什么单开一张：** ticket [06](06-control-failsafe-paths.md) 的验收项 ④ 是
「EKF2 位置失效 → `LAND`」。实现时按字面接了判据（`position_invalid` = `local_position` 出现 NaN/Inf），
但**第一手核实发现这条判据在本项目的链路上永远为假**（详见下），所以 ④ 实际上**没有兜住任何东西**。
这不能靠"判据存在"糊过去，也不能在本张里悄悄换设计 —— 换设计要动守卫清单与降级表，**必须单独立票**。

**第一手证据（2026-09-22，z0 固件 `Minderring/PX4-Autopilot` @ `4a0e65f2331b23b9c8352656e9ccb6edeaea5d6c`）：**

1. `src/modules/ekf2/EKF2.cpp::PublishLocalPosition` 直接发 `_ekf.getPosition()` ——
   `lpos.x/y/z` **没有任何 NaN 合成**（只有全局原点无效时 `ref_lat/lon/alt` 是 NaN，与局部位置无关）。
2. `src/modules/mavlink/streams/LOCAL_POSITION_NED.hpp` 把 x/y/z 原样发出，
   **不带 `xy_valid`**（这个 flag 在 MAVLink 这一层就丢了）。
3. mavros 2.15.1（commit `22ae5b7cc7cdb4cb9c2070a8213c72dae445a23e`，
   `/home/chen/mavros_ws/src/mavros`）`src/plugins/local_position.cpp::handle_local_position_ned`
   原样 NED→ENU 拷过去，不校验也不合成。

⟹ 结论：**"EKF2 位置失效"不会以 NaN 的形式出现在 `/mavros/local_position/pose` 上。**

**现状（不是零）：** 动捕是本项目**唯一**的位置源，所以 EKF2 位置失效**必然**先表现为
**动捕输入失效**（超时 / 时间戳不前进）—— 那两条判据（ticket 06 已实现）是**当下的真实兜底**。
本票要处理的是"有没有更早、更直接的载体"。

**候选载体（都未核实，别当结论用）：**

| 候选 | 出处 | 要核实什么 |
|---|---|---|
| `/mavros/estimator_status` 的 `pos_horiz_abs_status_flag` | `mavros/src/plugins/sys_status.cpp:1296` 一带 | ① 该 flag 在 PX4 侧由谁置/清（`estimator_status` 的 flag 语义在 1.14 里改过）；② **是否有粘滞性** —— 若失效解除后 flag 不会自己清，直接拿它做判据会变成一个单向 latch（正是 ticket 06 明确要避免的） |
| `EKF2` 的 `control_status_changes` / `cs_*` 位 | `EKF2.cpp` 发布的 `estimator_status` | 同上；另确认 z0 的 `EKF2_*` 参数下这些位会不会常置 |
| `/mavros/local_position/pose` 的 `header.frame_id` 或协方差 | mavros `local_position.cpp` | 协方差在 `LOCAL_POSITION_NED` 路径上是否被填充（很可能恒 0 ⟹ 不可用） |

- [ ] 核实上表中**至少一条**载体在 z0 的固件 + mavros 版本上**真的可读、且能自清**
- [ ] 若找到可用载体 ⟹ 决定它属于哪一类降级（输入类 ⟹ 已解锁时 `LAND`）、进 `fsm.hpp` 的守卫/降级表并补 seam 2 单测
- [ ] 若**找不到**可用载体 ⟹ 把结论写进 `docs/control/phase1-implementation.md` §4.4，
      并明确"④ 的兜底就是动捕那两条判据"，同时删掉或降级 `position_invalid` 的对外说法（避免读文档的人以为它兜住了 EKF2）
- [ ] 无论走哪条，`src/control/README.md` 与 `CONTEXT.md` 的「失效降级」条要同步（当前措辞已注明"防御性"）

**Blocked by:** 无（ticket 06 已落地，本票只动判据与文档）
**与 ticket 08 的关系：** 起飞之后才有"位置失效要中止任务"的后果；本票可以独立做，但**结论要在 08 之前落地**
（否则首飞时对"位置失效怎么办"的答案会是一句没兜住的判据）。
