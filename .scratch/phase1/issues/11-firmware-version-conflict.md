# 11: 飞控自报 1.14.3 vs 文档的「v1.15 世代」—— 冲突未决

Status: needs-triage
Type: research

**要从哪来：** ticket [01](01-hardware-prereqs.md) 的 ③ 第一次拿到**飞控自己报出的版本号**
（第十一轮，2026-09-21）。它是 §2.5 里被指定为「唯一终审」的那个读数 —— 而**终审与 (b) 不符**。

**冲突（两边都是第一手，不是转述）：**

| 来源 | 结论 | 依据 |
|---|---|---|
| 飞控 `AUTOPILOT_VERSION` | **`1.14.3`** | `flight_sw_version = 17695488 = 0x010e0300`，按 PX4 编码 `(major<<24)\|(minor<<16)\|(patch<<8)` |
| [`refs-manifest.md`](../../../docs/reference/refs-manifest.md) §2.5(b) | **v1.15 世代** | fork `5ba2e91f` 的**源码** `msg/`：`OffboardControlMode` 带 `thrust_and_torque` + `direct_actuator`，与 v1.15.0 一致、与 v1.14.3 不一致 |

**逐字读数与三种未验证的假设已在 [`refs-manifest.md`](../../../docs/reference/refs-manifest.md) §2.5(c) 记全**，
不在本文件重复。**该节刻意没有挑一个假设当结论。**

**为什么它不能就这么悬着：**

- §2.6 的决策（版本基准统一到 fork `5ba2e91f`）以 §2.5(b) 为前提
- **Phase 2 的 [phase2/01](../../phase2/issues/01-sitl-px4-msgs-baseline.md) `px4_msgs` 版本基准锁定**整张 ticket 的地基就是它 ——
  §2.5(a) 那张「`release/1.14` 不兼容」的表是拿 **fork 源码**比的；
  若飞控实际跑的是 1.14.3 系固件，**该结论可能反过来**
  （1.14 系下 `OffboardControlMode` 是 `bool actuator` —— 而它是 offboard 心跳必需的那条消息）

**建议的第一步（§2.5(c) 里列的代价最低、信息量最大的一条）：**

- [ ] **用 MAVLink 读飞控的完整参数表**，与 [`docs/assets/z0.params`](../../../docs/assets/z0.params) 对照
  - 该文件头自标「**尚未在 z0 的飞控上核实**」⟹ 这一步能把那句话**第一手结掉**
  - 顺带第一手核实 `MAV_1_CONFIG = 102` 与 `SER_TEL2_BAUD = 921600`（至今只有"强证据、非第一手"，见 ticket 01 第三轮）
  - 判世代：看参数表里有没有只属于某一代的参数
  - ⚠️ **只读**（`PARAM_REQUEST_LIST`），**不写任何参数**
- [ ] 分支源码侧：看 `version.h` / `px4_firmware_vendor_version()` 的语义（假设 2 与 3 要它才能验）
- [ ] 定案，并按结果回头改 §2.5 / §2.6 / phase 2 的 [phase2/01](../../phase2/issues/01-sitl-px4-msgs-baseline.md)
  - 若结论是「1.14 系」⟹ **Phase 2 的 `px4_msgs` 选版要整个重判**，且 §2.5(a) 的表要重做
  - 若结论是「1.15 世代、版本号是厂商覆写」⟹ §2.5(b) 保住，但要在 §2.5(c) 里写明**为什么**版本号不可信

**Blocked by:** 无 —— 需要飞控上电并接在串口1 上（Phase 1 ticket 01 已完成，链路已通）

---

**为什么不是 `ready-for-agent`：** 第一步（读参数表）agent 能做；
但**末了一步是判断**（这批参数表到底对应哪一代、要不要推翻 §2.5/§2.6），
属于 `needs-triage` —— 先由人决定按哪条路走、以及本张要不要拆成「取证」+「决策」两张。
