# 18: 用 QGC 执行器测试落实「物理输出口 ↔ 机体角」那张表

Status: ready-for-human
Type: task

**What to build:** 把「**机体角 → 转子序号 → 电机序号 → 物理输出口**」这条链**在硬件上逐个验一遍**，
填出一张有出处的表，并把 [`ladder-acceptance-criteria.md`](../../../docs/control/ladder-acceptance-criteria.md)
§1.1 里那条 ⚠️（"**物理口→机体角**要现场落实"）消解掉。

**Blocked by:** 无。★ **与走哪条通路无关**（px4_msgs 还是 mavros 都改不了接线），
也**不需要 uXRCE-DDS**、不需要解锁 —— 是**唯一一条现在就能做**的独立项。

---

## 为什么这张表现在还是空的

阶梯 1 的判据是"**目视电机转速差方向**"，而那条判据的前提是
"**我知道哪个物理口是哪个角**"。现在这条前提只有一半：

- **角点**是有的（[`ladder-acceptance-criteria.md`](../../../docs/control/ladder-acceptance-criteria.md) §1.1 `:83-100`，从
  `CA_ROTOR0..3_PX/PY` 与 `AZ = −1` 推出：**0 = 前右、1 = 后左、2 = 前左、3 = 后右**）；
- **物理口**是空的 —— §1.1 `:103-107` 明写"**上表是按角点写的，现场要确认的正是哪个角是哪个物理口**"，
  且这一行已经被登记进 §7.2 底部那张"**已登记、未处置的相邻事项**"表（`:536`）。
- ⚠️ **而且 `PWM_MAIN_FUNC1..4 = 103, 102, 101, 104` 是刻意改过的非默认电机序**
  （[`params-changes.md`](../../../docs/reference/params-changes.md) **D7**），
  ⟹ **"输出口 3" 驱动的才是 Motor1**，绝不能按"Motor n → 输出 n"想当然。

**不做这一步，阶梯 1 的六段就是在一张错位的表上判方向。**

---

## 已经推出来的部分（**第一手，链条完整**）

| 机体角 | 转子序号 | 电机序号 | 物理输出口 |
|---|---|---|---|
| **前右** | `CA_ROTOR0` | **Motor1** | **MAIN3** |
| **后左** | `CA_ROTOR1` | **Motor2** | **MAIN2** |
| **前左** | `CA_ROTOR2` | **Motor3** | **MAIN1** |
| **后右** | `CA_ROTOR3` | **Motor4** | **MAIN4** |

**每一列怎么来的**：

1. **`PWM_MAIN_FUNC1..4 = 103, 102, 101, 104`**（`docs/assets/z0.params:699-702`；
   同值见 `bag/z0-params-2026-10-10.params:663-666`）。★ 这四个数是**输出功能 id**，
   而 `Motor` 这一族 id 从 **101** 起、共 12 个（`~/refs/px4-v1.14.3-build/src/lib/mixer_module/output_functions.yaml`）
   ⟹ `101 = Motor1`、`102 = Motor2`、`103 = Motor3`、`104 = Motor4`
   ⟹ **输出口 1 挂 Motor3、口 2 挂 Motor2、口 3 挂 Motor1、口 4 挂 Motor4**。
2. **转子序号 ↔ 电机序号**：`CA_ROTORn`（n 从 0 起）与 **Motor(n+1)** 直接对应 ——
   分配器按**下标顺序**读 `CA_ROTOR%u_PX/PY/…/KM` 并依次登记执行器
   （`.../src/modules/control_allocator/ActuatorEffectiveness/ActuatorEffectivenessRotors.cpp:54-77`；
   且 **v1.14.3 的 `src/` 里查不到 `MOT_ORDERING`**，`grep -rn 'MOT_ORDERING' src/` 零命中）。
3. **角点**：§1.1 `:85-88`（`CA_ROTOR0_PX/PY = +0.15/+0.15`、`1 = −/−`、`2 = +/−`、`3 = −/+`）。

★★ **但上表的第 3 列与第 4 列是"读参数读出来的"** ——
**本票的全部意义就是把它换成"看着它转出来的"**。⛔ 上表**不是**结论，是**待验的假设**。

---

## 怎么做（`MAV_CMD_ACTUATOR_TEST`，QGC 执行器测试页）

**前置**（全在 `docs/assets/z0.params` 现读）：

| 条件 | z0 现值 | 出处 |
|---|---|---|
| **未解锁**（飞控自己拒） | —— | `Commander.cpp:1433`（`_arm_state_machine.isArmed()` ⟹ `DENIED`） |
| `COM_MOT_TEST_EN = 1` | **1** ✓ | `z0.params:286`；飞控侧判据 `Commander.cpp:1437` |
| `CA_AIRFRAME = 0`（多旋翼） | **0** ✓ | `z0.params:115` |
| `CA_ROTOR_COUNT = 4` | **4** ✓ | `z0.params:226` |
| 安全开关（若有）必须已解除 | —— | `Commander.cpp:1433`（`_safety.isButtonAvailable() && !isSafetyOff()` ⟹ `DENIED`） |

**实测方式**：QGC → 执行器测试，逐个点动，**目视哪个物理插头在转**。
命令落到飞控的哪个字段（**第一手**，`Commander.cpp:1431-1469`）：

- `param1 = value`（`actuator_test.value = cmd.param1`，`:1463`）；
- `param2 = timeout`（**秒**，`:1465` 乘 1000；`≤ 0` ⟹ `ACTION_RELEASE_CONTROL`，
  且**上限被强制到 3000 ms**，`:1476-1478`）；
- `param5` = **`actuator_test.function`**（`:1442`）—— 若 `< 1000`，
  按 **MAVLink 的 `ACTUATOR_OUTPUT_FUNCTION`** 编号解释：**1..32 = 电机**
  （映射成 PX4 的 `FUNCTION_MOTOR1 + n − 1`，即 **1→Motor1=101**）、**33..64 = 舵机**；
  若 `≥ 1000` 则 `−= 1000`（直接用 PX4 的内部 id）。

★★ **一处本票开票时没核、必须现场钉住的**：**QGC 那个页面实际送的 `param5` 是哪个编号**
（电机序号 1..4？输出序号 1..4？还是 PX4 的 101..104？）—— **QGC 不在本仓库里，也没有读过它的源码**。
**不许猜。** 用飞控控制台看真值：

```
listener actuator_test      # 每次点动时看 function 落到哪个数
```

期望看到 **101 / 102 / 103 / 104**（= Motor1..4）。**若看到的是别的数，先把它记下来再判断** ——
本票的结论依赖"哪个电机真的被点着了"，不依赖"QGC 上那个标签叫什么"。

---

## 要填的表（本票的主交付）

> ★ **默认拆桨**（和阶梯 1 同一纪律）。只量电机转向，**不装桨**。

| QGC 点动项 | `listener` 看到的 `function` | 哪个物理口在转（MAIN1..4） | 上位机上的位置（前左/前右/后左/后右） | 转向（CW/CCW） |
|---|---|---|---|---|
| 1 | | | | |
| 2 | | | | |
| 3 | | | | |
| 4 | | | | |

**然后反核上表**：四行都要与"已推得的链条"一致；
**任一不一致 ⟹ 停**（⛔ 不许靠调参数去把它凑一致 —— 不一致这件事本身就是结论，
它可能是接线、参数、或我这条推导链里某一环错了，三种归因完全不同）。

**顺手一并记（很便宜，但要说清它不是什么）**：
- **转向**：`CA_ROTOR0/1_KM = +0.05`、`CA_ROTOR2/3_KM = −0.05`
  （`z0.params:134 / :158 / :166 / :174`），而 PX4 自己的文档写着
  "**Use a positive value for a rotor with CCW rotation. Use a negative value for a rotor with CW rotation.**"
  （`.../control_allocator/module.yaml:211-214`，第一手）
  ⟹ **期望：Motor1 / Motor2 一个转向、Motor3 / Motor4 另一个转向**。
  ★ **但这不是 KM 的判据**：KM 的判据是**阶梯 1 的 yaw 那一行**（推力起来之后才有效）。
  这里只是**提前看一眼**，不符时**先记录**，⛔ **别当场改 `CA_ROTORn_KM`**。
  （PX4 那两句没写视角 —— 实操按"从机身上方俯视"理解；现场判不出就**原文照抄**，别自己补一个视角。）
- **桨叶正反**（不转，纯目视）：正桨/反桨装错**在拆桨状态下看不出来**，
  顺手核对每个电机上桨的标记与它的**转向**是否配对。★ 这一条同样**只记录**。

---

## 写回哪里（三处，**都在别的票的范围里 ⟹ 本票只点、各自归位**）

| 位置 | 改什么 | 归属 |
|---|---|---|
| `ladder-acceptance-criteria.md` §1.1 的 ⚠️（`:103-107`） | 把"现场要确认"换成**填好的表**（☠️ 原 ⚠️ 就地标掉，按 §7.1 在 §7.2 增一行） | ★ 见下 |
| 同页 §7.2 底部"已登记、未处置的相邻事项"表（`:536`） | 把那一行标为**已处置** | ★ 见下 |
| [`params-changes.md`](../../../docs/reference/params-changes.md) **D7** | 它自己写着"仍需现场确认'物理口→机体角'的完整映射" ⟹ 补上实测结果 | D7 的归属票 |

★★ **与 [17](17-ladder-criteria-trace.md) 的分工要写死，别两票同时改一行**：
- **[17](17-ladder-criteria-trace.md) ⛔ 不碰** §1.1 的 ⚠️ 与 §7.2 底部那一行（它的票面已注明）；
- **本票**负责把实测结果落进这三处。若 [17](17-ladder-criteria-trace.md) 在本票之前落地，
  它会在 §7.2 里留下一行**互不相干**的修订（下行载体），**两行并存是对的**（一行一个证据）。

---

## 验收

- [ ] 四行表**填满**，每行都有"`listener actuator_test` 看到的 `function` 值"（⛔ 不是"QGC 上写的是几"）
- [ ] `param5` 的编号空间**有第一手结论**（101..104？还是 1..4？写清核法）
- [ ] 四行与"已推得的链条"**逐行核对**：一致 / 不一致（不一致 ⟹ 停，并写归因）
- [ ] 转向逐条记录（**并写明它不构成 KM 的判据**）
- [ ] 桨叶正反目视记录（不转状态下）
- [ ] 全程**拆桨**、**未解锁**（★ 车侧动作由操作者亲手做）
- [ ] 结果写进 §1.1 的 ⚠️（☠️ + §7.2 增行）、§7.2 底部登记表那一行标为已处置、`params-changes.md` D7
- [ ] **没有**改任何飞控参数（`CA_ROTORn_KM` / `PWM_MAIN_FUNC*` 都**只读不写**）
- [ ] **没有**碰 [17](17-ladder-criteria-trace.md) 的落点（下行载体那几处）

---

**为什么是 `ready-for-human`**：

1. **它要动飞机**（通电、点电机）—— 而 **车侧/飞机侧动作由操作者亲手做**，
   agent 只报时机与读数，⛔ 不代跑；
2. 判据里唯一需要"**眼睛**"的部分（**哪个口在转**）**没有自动化替代物** ——
   飞控能把 `function` 值报出来（`listener`），但"哪个插头在转"只有人看得到。

★ 本票的**推导部分**（上面那条链）已经做完且是第一手的；
剩下的是**上机验一眼**。**别把它和 [09](09-ladder-ground-and-tether20.md) 的阶梯 1 混成一件事** ——
阶梯 1 要用这张表，所以这张表**必须是阶梯 1 之前**的第一步。

---

## Comments

### 2026-10-12 —— 开票时已核过的

| 事实 | 锚点 |
|---|---|
| `PWM_MAIN_FUNC1..4 = 103, 102, 101, 104` | `docs/assets/z0.params:699-702`；bag 版 `:663-666` |
| `Motor` 族 id 从 **101** 起、共 12 个 | `~/refs/px4-v1.14.3-build/src/lib/mixer_module/output_functions.yaml` |
| `CA_ROTORn` 按**下标顺序**读、转子 n ↔ Motor(n+1) | `.../control_allocator/ActuatorEffectiveness/ActuatorEffectivenessRotors.cpp:54-77` |
| v1.14.3 的 `src/` 里**没有** `MOT_ORDERING` | `grep -rn 'MOT_ORDERING' src/` **零命中** |
| 角点：0 前右 / 1 后左 / 2 前左 / 3 后右 | `docs/assets/z0.params:130-177`（`PX/PY`）+ `AZ = −1`；推导在判据页 §1.1 `:85-88` |
| `KM`：0/1 = +0.05、2/3 = −0.05 | `z0.params:134 / :158 / :166 / :174` |
| KM 正 ⟹ CCW、负 ⟹ CW（PX4 自己的话） | `.../control_allocator/module.yaml:211-214` |
| 执行器测试的拒绝条件与参数空间 | `.../commander/Commander.cpp:1431-1469`（`:1442` 取 `param5`、`:1463` `param1`、`:1465` `param2`、`:1476-1478` 上限 3000 ms） |
| z0 的四个前置条件现值 | `z0.params:286`（`COM_MOT_TEST_EN = 1`）、`:115`（`CA_AIRFRAME = 0`）、`:226`（`CA_ROTOR_COUNT = 4`） |

**未核（现场第一件事）**：

1. ★★ **QGC 执行器测试页实际送的 `param5` 编号空间**（QGC 不在本仓库里，源码没读过）。
   核法：`listener actuator_test` 看 `function` 的真值。
2. **`MAV_CMD_ACTUATOR_TEST` 的完整参数定义**（本票只读了飞控侧的**消费端**：
   `param1/param2/param5` 的用法是**从消费端反推**的，没读 MAVLink 消息定义那一侧的文档）。
   现场若发现某个参数对不上（例如超时行为与预期不同），**以飞控侧那段代码为准**，并把它记下来。
3. **真机上没做过**：四行表是空的，⛔ 别把上面"已推得的链条"当成实测结果引用。
