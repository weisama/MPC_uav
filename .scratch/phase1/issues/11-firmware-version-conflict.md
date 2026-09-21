# 11: 飞控自报 1.14.3 vs 文档的「v1.15 世代」—— **2026-09-21 定案：文档错了，真身是 v1.14.3 系的厂商构建**

Status: done
Type: research

> ✅ **2026-09-21 收口**：证据链已定案并落进 §2.5(d)；两件待拍板的事 ① 硬件 = **MicoAir NxtPX4v2**、
> ② §2.6 改法**已签字**（见下）。
>
> ⚠️ **不过 `done` 不掩盖两件事，都已点名**：
> 1. **跨文档清扫** = 票 [10](10-docs-1-14-sweep.md)（它的作业方向已反转）；
> 2. **本票带出一个新的 Phase 3 任务**：板名（NxtPX4v2）与飞控里跑的固件树（只有 `micoair/h743-*`，
>    **无 `hkust/`**）对不上 ⟹ **Phase 3 重编固件该编哪个板级目标仍未定**，
>    且 §2.7 的 ADC 修复途径对这个固件不适用。**此项目前只在本文与 §2.6 里记着，
>    `.scratch/phase3/` 尚无 issue 集 —— Phase 3 建票时必须包含它。**

**定案一句话：** z0 飞控里跑的固件 = commit
**`4a0e65f2331b23b9c8352656e9ccb6edeaea5d6c`**（`Minderring/PX4-Autopilot`，
"add micoair743-aio to PX4-Autopilot v1.14.3"，2024-11-18），
**其 `msg/` 与上游 v1.14.3 逐字节相同，也与 `px4_msgs release/1.14` 逐字节相同。**

⟹ [`refs-manifest.md`](../../../docs/reference/refs-manifest.md) §2.5 的 (a)(b) **作废**（参照物选成了 fork），
§2.6 的决策前提被推翻。**完整证据链在 §2.5(d)，本文件不重复。**

**它是怎么定下来的**（原先计划的"第一步 = 读参数表"**用不上了** —— 没读参数表就定案了）：

| 环节 | 结果 |
|---|---|
| `AUTOPILOT_VERSION.flight_custom_version` 的 8 字节 | `00 00 00 33 f2 65 0e 4a` |
| 解出 git hash 前 10 位 | **`4a0e65f233`**（源码链见 §2.5(d)①，**用的是运行固件自己那棵树**） |
| 反向验证 | 由该 sha 前 16 字符正向推出的字节 **逐字等于**飞控实测字节 |
| 该 commit 的 `msg/`（185 个） vs 上游 v1.14.3 | **0 个不同**（blob SHA 全量比对） |
| 该 commit 的 `msg/` vs `px4_msgs release/1.14` @ `ffb6e80e` | **0 个不同**（唯一差异是 `msg/CMakeLists.txt`，那是 PX4 的构建文件） |
| 该 commit 的 `boards/` | 有 `micoair/{h743, h743-v2, h743-aio}`，**没有 `hkust/`** |

---

## ✅ 两件要人拍板的事 —— **2026-09-21 用户已裁定**

- [x] **① 硬件 = MicoAir NxtPX4v2**（用户看实物/订单，**第一手**）。
      ⚠️ **但这没有消掉张力，反而把它逼到台面上**：NxtPX4v2 的上游目标名是 `hkust_nxt-dual`，
      而飞控里跑的固件树**只有 `micoair/h743-*`、没有 `hkust/`**。所以要么
      「**板名 ≠ 固件目标名**」（厂商出厂固件用自己的 `micoair/*` 目标编），要么实物标签与固件不符。
      **⟹ Phase 3 重编固件时必须先把这件事查清**：按 `hkust_nxt-dual` 编 = **换板级目标**
      （引脚/传感器/ADC 通道全变），**不是**"打一行补丁"，且有引入 §2.7 那个 ADC 缺陷的风险。

      ☠️ **同时更正我自己之前的一个错判据**：下面那张表原先写 `micoair/h743-aio` 是「IMU **BMI270**」，
      并拿它当"不是 NxtPX4v2"的证据 —— **这是错的**。实测三块 micoair 板的 `default.px4board`：

      | 板 | IMU | 气压计 |
      |---|---|---|
      | `micoair/h743` | **BMI088** + BMI270 | DPS310 |
      | `micoair/h743-v2` | **BMI088** + ICM42688P | （由 `COMMON_BAROMETERS` 带，未见具体驱动行） |
      | `micoair/h743-aio` | **BMI088** + BMI270 | DPS310 |

      **三块都带 BMI088**（NxtPX4v2 是"双 BMI088"）⟹ 我原先那条"IMU 对不上"**不成立，作废**；
      **只剩气压计（DPS310 vs SPL06）还算是差异，但单凭它也不足以定板**。

      **若要真把 ① 钉死，唯一便宜的硬办法**（仍是**只读**）：读飞控参数表，看
      `CAL_ACC0_ID` / `CAL_ACC1_ID` / `CAL_ACC2_ID`（有几个加速度计、器件 ID 是什么）
      与 `SER_*` 端口的集合 —— 传感器**器件 ID** 比"板名"硬。**这条尚未做。**

- [x] **② §2.6 改法 = 用户已签字**：`px4_msgs` 钉 **`release/1.14` @ `ffb6e80e`**（已执行，见 §2.6 执行版）；
      SITL 源取默认**上游 v1.14.3**（用户未单独表态，已在该处标注为"我取的默认、可改"）。

## 原始待决记录（留存）

- [ ] ~~**① 硬件到底是哪块板？**~~
  固件那棵树里只有 `micoair/h743-*`，而本项目文档（含 `CLAUDE.md`）一直写「飞控 = **MicoAir NxtPX4v2**」，
  而 NxtPX4v2 的 PX4 目标名是 **`hkust_nxt-dual`** —— 两者对不上：
  - `hkust/nxt-dual`（NxtPX4v2）：TEL2=`/dev/ttyS3`、气压计 SPL06、IMU 双 BMI088
  - ~~`micoair/h743-aio`：TEL2=`/dev/ttyS1`、气压计 DPS310、IMU **BMI270**~~
    ☠️ **2026-09-21 更正：这行是错的** —— 实测 `micoair/h743-aio` 的 `default.px4board` 是
    **BMI088 + BMI270**（三块 micoair 板**全都带 BMI088**），所以"IMU 对不上"**不成立**；
    真正还剩的差异只有**气压计**（DPS310 vs SPL06）。见上面 ① 的更正表。
  - 飞控报 `vendor_id=7052 / product_id=54`，但**上游 5 块板**（`hkust/*`、`micoair/*`）的
    `default.px4board` 都不写这两个 id（由厂商分支的 Kconfig 决定，上游看不到）⟹ 这条查不出来

  **→ 看实物 / 看订单 / 看机架上的丝印**（最快）—— ✅ **用户已看：NxtPX4v2**。
  若要**把①钉死到能指导 Phase 3**，唯一便宜的硬办法是读飞控参数表的
  `CAL_ACC*_ID` / `CAL_BARO0_ID`（器件 ID 比"板名"硬）——**只读**（`PARAM_REQUEST_LIST`），不写任何参数。
  ⚠️ **这条不是学究**：它决定 §2.7 那个 ADC 缺陷**到底该不该修**、以及 Phase 3 重编固件时该编哪块板。

  **已经试过、判定为「判不出来」的两条（别重复劳动）**：
  - `vendor_id=7052 / product_id=54` —— 上游 5 块板（`hkust/*`、`micoair/*`）的 `default.px4board`
    都不写这两个 id（由厂商分支的 Kconfig 决定），**查不出来**。
  - [`docs/assets/z0.params`](../../../docs/assets/z0.params) 的 `SER_*` 集合只有
    `SER_GPS1_BAUD` / `SER_TEL1_BAUD` / `SER_TEL2_BAUD`，与三块 `micoair/*` 及 `hkust/nxt-dual`
    **都不吻合** —— 但该文件是上游 `Sim2real-UAV-Params-` 的存档、**不是 z0 飞控导出的**
    （自标「尚未在 z0 的飞控上核实」），所以它**不能用来定板**；且参数文件只记**已配置**的端口，
    缺席本身也不构成证据。**⟹ 只能靠实物/订单，或真去读一次飞控的参数表。**

- [x] ~~**② §2.6 怎么改（用户决策）**~~ ✅ **2026-09-21 用户已签字**（`px4_msgs` 钉 `release/1.14` @ `ffb6e80e`），
  已落进 [`refs-manifest.md` §2.6 执行版](../../../docs/reference/refs-manifest.md)。
  原建议存此备查：`px4_msgs` = `release/1.14` @ `ffb6e80e`；SITL = 上游 v1.14.3（或直接编 `4a0e65f2`）。

## 连带要改的地方（★ 2026-09-21 逐条复核过，标注已按实际状态校正）

- [x] [`refs-manifest.md`](../../../docs/reference/refs-manifest.md)：§2.5 头部判定（**已改**）、§2.5(d)（**已写**）、
      **§2.5(c) 的作废标记（已加 —— 本条原先漏了，是 code-review 查出来的）**、
      §2.6 的决议表（只加了 ⚠️，**表本身还没改** —— 那是 ② 签字后的事）、
      §2.7 的 ADC 修复途径（**已加 ⚠️**）
- [x] [`phase2/01`](../../phase2/issues/01-sitl-px4-msgs-baseline.md)：Blocked-by 与「为什么必须是 fork」
      那一段**已重写**；`Status` 已从 `ready-for-agent` 改成 **`needs-info`**（等 ② 签字）
- [x] [`params-changes.md`](../../../docs/reference/params-changes.md) A4：「修复 = 重编重刷 fork 的
      `pr_fix_hkust_nxt_dual`」**已划掉并标作废**（对这块固件不适用）
- [x] `CLAUDE.md` 的「PX4 1.14.x」标签 —— **复查后确认它一直是对的**（09-20/21 一度判它错，是误判），
      **故不改**。但「MicoAir NxtPX4v2」这个**板名**待 ① 定
- [x] [`phase1/01`](01-hardware-prereqs.md)：它的 ③ 一节与头部摘要**已按本票更正**
      （原先写着「解不出 hash」「未决冲突」「工具已补第二道闸」——三处都已标作废）
- [x] [`mavlink_probe.py`](../../../tools/prereq-evidence/mavlink_probe.py)：错误的「必须是 ASCII」闸门已移除，
      补了 `--self-test`（用上面那对真值做交叉验证）
- [ ] ⚠️ **跨文档清扫 = 票 [10](10-docs-1-14-sweep.md) 的活，本票不代劳**：条「v1.15 世代 / 基准统一到 fork」
      还写进了 `README.md`(4,78,79)、`docs/PROJECT.md:437`、`docs/control/phase2-implementation.md:587`、
      `.scratch/phase2/spec.md:298`、`.scratch/phase2/issues/06:12`、`docs/reference/params-changes.md:14`。
      **票 10 的作业方向已按其实际状态反转**（它原先会指挥 agent 往错的方向扫）——
      详见该票顶部那段 ☠️ 更正。**本票只负责把证据与票据层面的事实定死。**

**Blocked by:** 无 —— 需要人看一眼实物/拍一个板（①），以及同意 §2.6 的改法（②）

---

**为什么不是 `ready-for-agent`：** 取证部分**已经做完并定案**；剩下两件都是
「agent 判不了」的（① 要接触实物，② 是用户已定决策的改法）。
