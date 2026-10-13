# `z0.params` 逐项分析

> 对 [`../assets/z0.params`](../assets/z0.params) 的调查记录。
> 标注规则：✅ **可直接用** / ⚠️ **需核实或修改** / ❌ **必须改** / 📌 **对 MPC（模型预测控制）有直接价值**
> 相关文件：[`../PROJECT.md`](../PROJECT.md) 第 5.1 节、[`px4-interface.md`](px4-interface.md)
>
> ★ **2026-09-21 已对账 —— 本文所有 ⚠️「需核实」项的总答复**：`z0.params` 与 z0 飞控真身
> **914/915 条名字全对上**（唯一多出的是飞控内部的 `_HASH_CHECK`），其中 **21 条值不同**、
> **其余 893 条值一致** ⟹ **本文写的每个"现值"都是飞控现状**。
> 于是 ⚠️ 这个标注的**含义变了**：**不再意味着"怀疑文件过时"，而是"飞机上真有的待改项"**。
> 对账全文见 [`.scratch/phase1/issues/15-fcu-param-reconciliation.md`](../../.scratch/phase1/issues/15-fcu-param-reconciliation.md)；
> 逐项处置（改/不改/何时改）见 [`params-changes.md`](params-changes.md) **§I**。
> ⚠️ **本文是取证正文，不是行动清单** —— 行动清单看 `params-changes.md`（它已取代本文 §6）。
>
> ☠️ **2026-09-23 —— 本文的引用基准已从 fork 换成真身**：本文多处曾按
> `Peize-Liu/PX4-Autopilot @ 5ba2e91f` 引行号，**该 fork 与 z0 固件无关**。
> z0 固件**真身 = `Minderring/PX4-Autopilot` @ `4a0e65f2`**（v1.14.3 系的 MicoAir 厂商构建）。
> 全文已逐处按真身重新核对，「行号变了」的都已就地标注；**判词被推翻的只有一处**：
> §3.5.1（ADC 电流通道）—— 由"中招/必须重刷"**改为「未中招」**。
> ⚠️ **2026-10-09 又更正一次（仅改理由，不改观测）**：z0 跑的板级目标**就是** `hkust_nxt-dual`
> （飞控自报 `ver_hw = HKUST_NXT_DUAL`）—— 本文件里那些"真身树里没有 `hkust/`、用的是 `micoair/*`"
> 的说法**作废**；「未中招」是因为 z0 那份板级配置**本来就带修好的 `ADC12_CH(8)`**。
> 权威判定与分支对照见 [`refs-manifest.md`](refs-manifest.md) §2.5(d)④ 的 2026-10-09 块 / §2.6 / §2.7。

---

## 1. 文件元信息

| 项 | 值 |
|---|---|
| 来源 | [`Huan-qun/Sim2real-UAV-Params-`](https://github.com/Huan-qun/Sim2real-UAV-Params-) → `S2R/assets/param/uav1.params` |
| 大小 | 928 行（**其中 6 行为文件头出处说明**），914 条参数 |
| 格式 | `Vehicle-Id <TAB> Component-Id <TAB> Name <TAB> Value <TAB> Type`（QGC 导出格式） |
| 头部声明 | `Stack: PX4 Pro` / `Vehicle: 多旋翼` / **`Version: 1.14.3 dev`** / `Git Revision: 4a0e65f233000000` |
| 注释 | `# Onboard parameters for Vehicle 1` —— **"Vehicle 1"，即多机系统里的 1 号机** |
| 来历 | 用户确认：**另一台用同一套配件搭建、且用动捕实飞验证过的无人机** |

> 📌 **版本吻合**：`1.14.3 dev` 与用户的目标版本 `1.14.x` 一致，因此参数名与枚举无需跨版本翻译。**这是该文件最大的价值。**

---

## 2. 参数分布总览

按模块前缀统计（共 914 条）：

| 模块 | 条数 | 关注度 |
|---|---|---|
| `EKF2_` | **129** | ★★★ 状态估计，动捕链路的核心 |
| `CA_` | **121** | ★★★ 控制分配，含几何与电机映射 |
| `COM_` | 67 | ★★ 安全逻辑（RC 接管、失控保护） |
| `MPC_` | 59 | ★★ PX4 **位置**控制器（**不是**模型预测控制） |
| `CAL_` | 49 | — 传感器标定值，**与具体硬件绑定，不可移植** |
| `RC_`/`RC1..18_` | 46 + 90 | ★ 遥控器通道映射 |
| `PWM_` | 44 | ★★ 执行器输出配置 |
| `MC_` | 41 | ★★★ 角速率环增益（模型预测控制的内环） |
| `SENS_`/`IMU_` | 40 + 19 | ★ 传感器使能与滤波 |
| `MAV_` | 23 | ★★ MAVLink/通信口分配 |
| `SYS_` | 20 | ★★ 机型选择与估计器选择 |
| `NAV_`/`RTL_`/`MIS_`/`GF_`/`FLW_`/`FW_`/`VT*`/`WV_` | 约 60 | — 任务/返航/固定翼，**本项目不用** |
| `BAT1_`/`BAT_`/`CBRK_` | 11+9+7 | ★★ 电池与安全旁路 |
| `DSHOT_`/`MOT_` | 5+2 | ★★ 电调协议 |
| `SDLOG_` | 7 | ★ 日志配置 |
| 其余 | — | `LNDMC_`/`HTE_`/`MBE_`/`CP_`/`TC_`/`TEL_`/`GF_` 等 |

> ⚠️ **`CAL_*`（49 条）与 `RC*` 通道映射（90 条）是硬件绑定的**：陀螺零偏、加速度计标定、遥控器行程都跟具体那台机器/那个遥控器有关。**照抄可能比不抄更糟。** 这两组应当用**你自己飞控上已有的值**。

---

## 3. ★ 必改项与核实项（重点）

这一节是本文件的结论部分。

### 3.1 ❌ `CA_ROTORx_PX/PY = 0.15` —— 几何与你的机架不符

**事实**（文件中的原始值）：

| 参数 | 值 |
|---|---|
| `CA_ROTOR0_PX` / `_PY` | **0.15** / **0.15** |
| `CA_ROTOR1_PX` / `_PY` | **-0.15** / **-0.15** |
| `CA_ROTOR2_PX` / `_PY` | **0.15** / **-0.15** |
| `CA_ROTOR3_PX` / `_PY` | **-0.15** / **0.15** |
| `CA_ROTOR_COUNT` | 4 |
| `CA_METHOD` | **2（Auto）** —— ⚠️ 旧版本文档写「2（归一化分配）」是**错的**，见下方更正 |

**推导**：Rotor 0 位于 `(+0.15, +0.15)`，到机体中心的距离 = `√(0.15² + 0.15²)` = **0.2121 m**，即臂长 **212 mm** → 轴距 **424 mm**。

**矛盾**：你的 XI35 Pro 机架轴距是 **152 mm**（臂长 76 mm）。**差了 2.8 倍。**

**推测原因**：这是 PX4 通用四旋翼 X 型的**默认几何**（`0.15 m` 半展）。`SYS_AUTOSTART = 4001`（Generic Quad X）会继承这套默认值，除非机型文件显式覆盖——而 `z0.params` 里没有任何覆盖的痕迹（`CA_ROTOR0_CT = 6.5`、`CA_ROTOR0_KM = 0.05` 等也全是 PX4 模块默认值，12 个 rotor 槽位完全相同，只有前 4 个有非零位置）。**（此段为推测，未在 z0 上验证——z0 的机载电脑没有 PX4 源码。）**

**影响评估（重要，分情况）**：

| 场景 | 影响 |
|---|---|
| **PX4 自带角速率环 + `(C)` 方案** | ✅ **确实无影响 —— 但本节原先给的理由是错的。** 正确理由见下方「★★ 2026-09-20 更正」。 |
| **(D) 方案（模型预测控制/INDI 直接出电机指令）** | ❌ **必须修正**。那时模型预测控制要自己建"力矩 → 电机"的映射，几何错了整个映射就错了。 |
| 模型预测控制建模中的**力矩饱和估计** | ⚠️ 有影响。若要估计"最大可输出横滚力矩"，需要真实几何。 |

**核实方法**（在飞控上执行）：

```bash
# 用 QGC 连上飞控，在 MAVLink Console 里执行：
param show CA_ROTOR0_PX
param show CA_ROTOR0_PY
# 或直接 dump 全部
param show CA_ROTOR*
```

```bash
# 或者导出你自己的参数文件，和参考文件做 diff（推荐，见第 5 节）
```

#### ★★ 2026-09-20 更正：为什么几何错了却真的没影响（原由是错的）

**原由（现已被推翻）**：「`MC_ROLLRATE_K/P` 等增益是在真机上试出来的，几何偏差已被调参吸收」。
**推翻依据（两条，均第一手）**：

**① z0 的角速率环增益全是 PX4 出厂默认值 —— 根本没有"被调过"的痕迹。**

| 参数 | z0 值 | 固件默认值（**真身** `Minderring/PX4-Autopilot @ 4a0e65f2`，`src/modules/mc_rate_control/mc_rate_control_params.c`） |
|---|---|---|
| `MC_ROLLRATE_P` | `0.15` | `PARAM_DEFINE_FLOAT(MC_ROLLRATE_P, 0.15f)` @ `:53` |
| `MC_ROLLRATE_I` | `0.2` | `PARAM_DEFINE_FLOAT(MC_ROLLRATE_I, 0.2f)` @ `:65` |
| `MC_ROLLRATE_D` | `0.003` | `PARAM_DEFINE_FLOAT(MC_ROLLRATE_D, 0.003f)` @ `:90` |
| `MC_PITCHRATE_P` | `0.15` | `PARAM_DEFINE_FLOAT(MC_PITCHRATE_P, 0.15f)` @ `:134` |
| `MC_YAWRATE_P` | `0.2` | `PARAM_DEFINE_FLOAT(MC_YAWRATE_P, 0.2f)` @ `:214` |

> ☠️ **2026-09-23 重新锚定**：本表原引 fork `Peize-Liu/PX4-Autopilot @ 5ba2e91f`，行号为
> `51 / 63 / 88 / 132 / 212`。**该 fork 与 z0 无关**（见 [`refs-manifest.md`](refs-manifest.md) §2.5(d)）。
> 已按**真身** `4a0e65f2` 逐行复核：五行**全部 +2**（上表即真身行号），**取值一字未变 ⟹ 结论不变**。

**逐条相符。** 所以"偏差被调参吸收"这个机制**不可能存在**。

**② 真正的原因：控制分配器会按几何自身的范数归一化，绝对臂长被约掉。**

`ControlAllocator::update_allocation_method()`（`src/modules/control_allocator/ControlAllocator.cpp:142-205`）
把 `CA_METHOD` 交给各 effectiveness 源；而 `CA_METHOD = 2` 的语义是 **Auto**，不是"归一化分配"
（`src/modules/control_allocator/module.yaml:33-44` 逐字：`0: Pseudo-inverse with output clipping` /
`1: Pseudo-inverse with sequential desaturation technique` / `2: Automatic`，`default: 2`）。
> ☠️ **2026-09-23 重新锚定**：原先写 `module.yaml:35-46`（fork 的行号）。已按**真身** `4a0e65f2`
> 复核 —— `CA_METHOD` 块在 **`:33-44`**（`values:` 三行在 `:41/:42/:43`，`default: 2` 在 `:44`），
> 本文件其余四处 control_allocator 引用（`ControlAllocator.cpp:142`、`ActuatorEffectivenessMultirotor.hpp:52`、
> `ActuatorEffectiveness.hpp:156`、`ControlAllocationPseudoInverse.cpp:73`）在真身中**行号与内容逐字未变**。

`ActuatorEffectivenessMultirotor::getNormalizeRPY()`
（`ActuatorEffectiveness/ActuatorEffectivenessMultirotor.hpp:52-55`）**无条件**返回
`normalize[0] = true`（基类 `ActuatorEffectiveness.hpp:156-161` 的默认值是全 `false`）：

```cpp
void getNormalizeRPY(bool normalize[MAX_NUM_MATRICES]) const override
{
	normalize[0] = true;
}
```

于是 `ControlAllocationPseudoInverse::updateControlAllocationMatrixScale()`
（`ControlAllocation/ControlAllocationPseudoInverse.cpp:73-105`）按**每一列自身的范数**缩放：

```cpp
// Same scale on roll and pitch
if (_normalize_rpy) {
	...
	roll_norm_scale = sqrtf(_mix.col(0).norm_squared() / (num_non_zero_roll_torque / 2.f));
	...
	_control_allocation_scale(0) = fmaxf(roll_norm_scale, pitch_norm_scale);
	_control_allocation_scale(1) = _control_allocation_scale(0);
```

roll 列的元素正比于各 rotor 的 `CA_ROTOR*_PY`，**所以 `roll_norm_scale` 正比于 `PY`** ——
`±0.15 → ±0.0537` 是一个**纯均匀缩放**，归一化后**完全约掉**。
**→ 对对称 X 型机架，绝对臂长不影响分配结果。**

**⚠️ 三处推论（这是本轮对 Q21 决策前提的实质修正）**：

1. **§3.1 的结论（对 (C) 无影响）成立，理由换成上面这条。**
2. **Q21=a 里「改几何必须同时重调 `MC_*RATE_P*`」的前提不成立** —— 若实测几何只是 `±0.15/±0.15`
   的**均匀缩放**，改与不改在分配器里等价，**不需要重调**。（增益本来就是默认值，也没"调好的"可弄坏。）
3. **但"实测电机位置"这件事仍然值得做，理由变了**：只有当实测几何**不是均匀缩放**
   （例如 `PX ≠ PY`、或不是正 X 布局）时，分配矩阵的**比例**才变、才需要重调。
   **→ 所以 Q21 的正确动作是「先量，再判断是否均匀缩放」，而不是「量完必改必重调」。**
   （另：`(D)` 方案与力矩饱和估算**仍然**需要真实几何，见上表。）

#### ★★ 2026-10-09 判定：**是均匀缩放 ⟹ 不改、不重调**（现场确认，非卡尺读数）

上面那条"先量再判断"的动作**已经走完**（判定全文与证据等级见
[`phase4/01`](../../.scratch/phase4/issues/01-geometry-and-extrinsics.md) 的「★ 判定（2026-10-09）」）：

| 项 | 值 | 证据等级 |
|---|---|---|
| 电机位置（机体系 x 前 / y 右） | **`(±0.0537401, ±0.0537401)` m** | **推得**，不是卡尺读数 |
| 轴距（电机对电机对角线） | **152 mm** | 机架规格 + 操作者 2026-10-09 目视复认，**两来源一致** |
| 布局形状 | **正方形**、正 X（`PX == PY`） | 操作者直接确认 |

⚠️ **证据等级如实标注**：全票**没有**逐轴卡尺数字，`a = 76/√2 = 53.7401 mm` 是**推得**的
（对角线 `2a² = 76²` 把它钉死）；「正方形」是操作者确认的**定性**结论。
★ **不能只靠对角线**：一条对角线分不出正方形与长方形 —— 形状那一步是**独立**的一项。

**判定**：`±0.15 → ±0.0537401` 是**纯均匀缩放**（因子 `0.35827`，即现值偏大 `2.79×`）
⟹ 按上面那条归一化论证，**`CA_ROTOR0..3_PX/PY` 不改、`MC_*RATE_*` 不重调**（`(C)` 方案）。
真值 `±0.0537401` 登记给 **(D) 方案与力矩饱和估算**用 —— 那两处**必须**用真值，且**从来不是"改参数"**。
⚠️ **别把这里读成「少做了一步」**：这是**判定结果**，不是遗漏。

### 3.2 ⚠️★ `PWM_MAIN_FUNC1..4 = 103, 102, 101, 104` —— 电机序 `3-2-1-4` ✅

**事实**：`PWM_MAIN_FUNC5..8` 全为 `0`（未使用），只有前 4 路有分配。

**推导**：PX4 的 actuator function 编号中 `101=Motor1`、`102=Motor2`、`103=Motor3`、`104=Motor4`。`PWM_MAIN_FUNCx` 问的是"物理输出口 x 上放哪个功能"：

```
物理输出口 1 → 功能 103 → Motor 3
物理输出口 2 → 功能 102 → Motor 2
物理输出口 3 → 功能 101 → Motor 1
物理输出口 4 → 功能 104 → Motor 4
```

→ **电机序 `3-2-1-4`**。

**为什么可信**：这是**刻意改过的非默认值**（默认是 `1-2-3-4`）。另外在调研中发现**独立的第二个来源**给出同样的 `3-2-1-4`，两者互相印证。

**⚠️ 但仍需现场确认**：`3-2-1-4` 说的是"物理口 → 逻辑电机"的映射。**逻辑电机号 1/2/3/4 又被 `CA_ROTORx_PX/PY` 映射到几何位置**，而上面 3.1 已指出几何是错的。所以"哪个物理口对应机体的哪个角"这件事，**必须对着 3.1 的 `CA_ROTOR` 表一起看**，不能只看电机序。

### 3.3 ⚠️★ `CA_ROTORx_KM` 的符号 —— 不能用来推断桨的转向

**事实**：`CA_ROTOR0_KM = +0.05`、`CA_ROTOR1_KM = +0.05`、`CA_ROTOR2_KM = -0.05`、`CA_ROTOR3_KM = -0.05`。**全部 12 个 rotor 槽位的 `CT` 都是 `6.5`。**

**⚠️ 这是一个容易掉进去的坑**：看到 `KM = ±0.05` 很自然会以为"正负号就是螺旋桨的物理转向"。**不是。** 这些是 PX4 `control_allocator` 的**模块默认值**（`CT=6.5`、`KM=±0.05` 对全部 12 个槽位一视同仁），反映的是**该机型在 PX4 里的约定**，不保证等于你机架上桨的实际旋向。

→ **props-in / props-out 必须靠肉眼/实飞来确认**，不能靠读参数。（见 [`../PROJECT.md`](../PROJECT.md) 第 7 节 O1）

### 3.4 ⚠️ `BAT1_R_INTERNAL = 0.005 Ω` —— 比 VTC6 的真实内阻小 15 倍

**事实**：`BAT1_R_INTERNAL = 0.004999999888241291`。

**对比**：VTC6 单节内阻 ≈ 13 mΩ，6S1P 整包 ≈ **78 mΩ**。参数里写的是 **5 mΩ**。

**影响**：影响 PX4 对"负载下电压跌落"的补偿估算。对纯电压显示影响不大，但**如果你想让模型预测控制观测/补偿电压 sag，这个值需要改对**。

**⚠️ 但先看 3.5 —— 电压可能根本没在读。**

### 3.5 ⚠️★ 电池监测链路可能是不完整的

**事实**（这几条放一起看）：

| 参数 | 值 | 含义 |
|---|---|---|
| `BAT1_SOURCE` | `0` | 电源 = 电源模块 (Power Module) |
| `BAT1_V_CHANNEL` | **`-1`** | 电压通道**未配置** |
| `BAT1_CAPACITY` | **`-1`** | 电池容量**未知** |
| `BAT1_V_DIV` | `10.02` | 分压比（有值，暗示配过电源模块） |
| `BAT1_A_PER_V` | `17.0` | 电流标定（有值） |
| `CBRK_SUPPLY_CHK` | **`894281`** | **禁用电源检查**（无备用电池也能解锁） |
| `COM_LOW_BAT_ACT` | `0` | **低电无动作** |
| `BAT_LOW_THR`/`BAT_CRIT_THR`/`BAT_EMERGEN_THR` | 0.15 / 0.07 / 0.05 | 阈值有设，但无动作 |

**解读**：`CBRK_SUPPLY_CHK = 894281` 这个值在用户自己的笔记里被解释为"**禁用电源检查，无备用电池也可解锁**"——这是**电压没接/没读**时的典型做法。加上 `BAT1_V_CHANNEL = -1`、`BAT1_CAPACITY = -1`、`COM_LOW_BAT_ACT = 0`，整条电池监测链看起来是**没配起来**的。

**⚠️ 如果确实如此，两个后果**：

1. **没有低电压失控保护。** 结合 [`../PROJECT.md`](../PROJECT.md) 第 2.4 节的发现（电池是功率瓶颈），这比较危险——飞机不会在电池不行时告诉你。
2. **模型预测控制拿不到电池电压。** 想做电压 sag 补偿的话，得先有一条电压观测通路。

**核实方法**（飞控上执行）：

```bash
listener battery_status -n 5
# 看 voltage_v / current_a / remaining 是否有真实数据
```

#### 3.5.1 ☠️ **已作废**：固件的 ADC 电流通道写在错误的引脚上（**对 z0 不成立**）

> ☠️ **标题判词已反 —— 对 z0 是「未中招」**（原写「★★★ 更深的病根……（本轮第一手验证）」）。
> **当前有效结论见下方页首更正块。**

> ### ☠️ **2026-09-23 本节前提已被推翻 —— 结论是「未中招」，不是「配错了」**
>
> **本节整节（§3.5.1）建立在「fork `Peize-Liu/PX4-Autopilot @ 5ba2e91f` 就是 z0 的固件」这个前提上，
> 而该前提是错的。** fork 与 z0 **无关**（它是 v1.15 世代；`Nxt-FC` 的 submodule 指认是一条**错的线索**）。
>
> z0 固件**真身** = `Minderring/PX4-Autopilot` @ `4a0e65f2`（v1.14.3 系的 MicoAir 厂商构建）。
> ☠️ ~~**真身那棵树里根本没有 `boards/hkust/`** —— 只有 `boards/micoair/{h743, h743-v2, h743-aio}`。
> 而 `boards/micoair/h743-aio` 写的是 `/* PC1 */ ADC1_CH(11)`，**与引脚相符、无此缺陷**。~~
> —— **2026-10-09 更正：这段理由错了。** 飞控**自报** `ver_hw = HKUST_NXT_DUAL`（8 份飞行日志一致）
> ⟹ **z0 跑的就是 `hkust/nxt-dual`**；`4a0e65f2` 那棵树里看不到 `boards/hkust/`，是因为
> **板级目录来自它的子提交 `33bdd21f`**（2024-11-19）—— 版本头只记裸 `HEAD`、**不带 dirty 标记**。
> 而那行 ADC：z0 那份 `board_config.h:98` **早就是 `ADC12_CH(8)`**（与 fork 修复提交 `8161c4cd` 同一 blob）。
>
> **→ 2026-09-21 实测判定：z0 「未中招」**（读数与判据见 [`params-changes.md`](params-changes.md) §A4）。
> ⟹ **正确的解释是「这份构建本来就带修好的值」**，不是「缺陷不在固件里」。
> 下面整节保留作为**取证过程与版本边界的存档**，但其结论（"必须重编重刷固件"）**对 z0 不成立**；
> Phase 4 的「重编重刷 `pr_fix_hkust_nxt_dual`」这个修法也因此**没有对象**（要修的那行早修好了），
> 且 2026-10-09 操作者已裁定**不重刷固件**。
> 权威判定见 [`refs-manifest.md`](refs-manifest.md) §2.5(d)④ 的 2026-10-09 块 / §2.7。

上面说的是"链路没配起来"。☠️ **原写「2026-09-20 这一轮查到了更硬的事实：z0 固件里电流采样读的是错误的 ADC 通道 —— 这不是"没配"，是"配错了"」—— 已作废**：**对 fork 成立，对 z0 不成立**（见上方更正块）。

**取证**（全部第一手，非二手总结）：

☠️ **原写「第一步，先找到 z0 固件的真身」—— 方向对，但这一步走错了路**（见下）：
`HKUST-Aerial-Robotics/Nxt-FC` **不是固件源码仓库**（只有 51 个文件，
是硬件 + 预览版二进制 + 两个 submodule），这是对的；但**它的 `.gitmodules` 指的不是 z0 的固件**：

```
url = https://github.com/Peize-Liu/PX4-Autopilot.git    branch = develop
Nxt-FC/main 钉在 commit  5ba2e91ff227fe4da0133371e88dc81b038bb81e   (2024-07-02)
```

> ☠️ **该 commit 与 z0 固件无关。** `Nxt-FC` 的 `.gitmodules` 描述的是 **Nxt-FC 这个仓库自己的**
> 上游依赖，**不等于飞控里烧录的内容**。z0 飞控自报的 `flight_custom_version` 直接给出真身 hash
> **`4a0e65f233`**（见 [`refs-manifest.md`](refs-manifest.md) §2.5(d)）。

**第二步，读该 commit 的板级配置**（`boards/hkust/nxt-dual/src/board_config.h:89-102`，逐字）：

```c
#define ADC12_CH(n)		(n)

#define PX4_ADC_GPIO  \
	/* PC4  */  GPIO_ADC12_INP4,   \
	/* PC5  */  GPIO_ADC12_INP8        ← 电流采样物理接在 PC5，即通道 8

#define ADC_BATTERY_VOLTAGE_CHANNEL             ADC12_CH(4)
#define ADC_BATTERY_CURRENT_CHANNEL             ADC12_CH(5)   ← 宏却写通道 5

#define ADC_CHANNELS \
	((1 << ADC_BATTERY_VOLTAGE_CHANNEL) | \
	 (1 << ADC_BATTERY_CURRENT_CHANNEL))
```

**硬件层的根因清清楚楚**：电流采样线接在 **PC5**，而 PC5 是 `ADC12_INP8`（**通道 8**）；
固件却让电流去读**通道 5**。板内注释与通道号自相矛盾 —— 这就是 bug。

**第三步，确认版本边界**（对照 PX4 上游三个 tag 的同一文件）：

| 固件 | `boards/hkust/nxt-dual` 的 `ADC_BATTERY_CURRENT_CHANNEL` |
|---|---|
| `Peize-Liu/PX4-Autopilot @ 5ba2e91f`（**☠️ 不是 z0 的固件，仅作对照**） | `ADC12_CH(5)` |
| **z0 固件真身** `Minderring/PX4-Autopilot @ 4a0e65f2` | ★ **2026-10-09 重查**：这一棵**没有 `boards/hkust/`**，但**不是构建时用的那一棵** —— 板级目录来自它的**子提交 `33bdd21f`**（厂商分支 `micoair743-v1.14.3`，2024-11-19）。**那一棵里 `nxt-dual` 的电流通道已经是 `ADC12_CH(8)`** ⟹ **z0 未中招是因为这份配置已修好**。（☠️ ~~其 `boards/micoair/h743-aio` 写 `/* PC1 */ ADC1_CH(11)`，无此缺陷 ⟹ 未中招~~ —— 那段理由作废） |
| **z0 用的那棵树** `Minderring/… @ 33bdd21f`（`4a0e65f2` 的子提交，只加 `boards/hkust/`） | `ADC12_CH(8)` ← **已修**（`board_config.h:98`，blob `91fca7fd18`，与下面 fork 修复提交 `8161c4cd` **同一 blob**） |
| PX4 上游 **v1.14.3**（tag `1dacb4c`） | 无 `boards/hkust`（该板级目标尚不存在） |
| PX4 上游 **v1.15.0** | `ADC12_CH(5)` ← **仍带 bug** |
| PX4 上游 **v1.16.0** | `ADC12_CH(8)` ← **已修** |

电压通道全程为 `ADC12_CH(4)` 未动，**上游的修复只针对电流通道**。
（附带一条旧文档遗漏的对照：`nxt-v1` 的电压通道是 `ADC12_CH(9)`，**与 `nxt-dual` 的 `CH(4)` 不同**，
两者不可互相照抄。）

**第四步，能否靠参数绕过？—— 不能。** 曾有一条诱人的推测：`analog_battery.cpp` 的
`get_current_channel()`（`src/modules/battery_status/analog_battery.cpp:109-117`）是

```cpp
if (_analog_params.i_channel >= 0) return _analog_params.i_channel;
else return DEFAULT_I_CHANNEL[_index - 1];   // 板级宏 → ADC12_CH(5)
```

即 `BAT1_I_CHANNEL` 只要给非负值就能**覆盖**板级默认，而 z0 现在正是 `-1`（回落到错值）。
**但这条路被编译期掩码堵死了**（`src/drivers/adc/board_adc/ADC.hpp:58-67`，逐字）：

```cpp
#ifndef ADC_CHANNELS
#error "board needs to define ADC_CHANNELS to use this driver"
#endif
ADC(uint32_t base_address = SYSTEM_ADC_BASE, uint32_t channels = ADC_CHANNELS, bool publish_adc_report = true);
```

`ADC_CHANNELS` 是**编译期宏**（`#ifndef`/`#error` 守卫即证明），作为构造参数默认值传入，
`_samples[]` 由它定容填充，**该驱动模块内无任何参数、无运行期覆盖**。
所以 `board_adc` **只采样通道 4 和 5**，而 `analog_battery` 只能从 `adc_report` 里已采样的通道中挑。
**通道 8 永远不会出现在 `adc_report` 里 → 把 `BAT1_I_CHANNEL` 设成 `8` 拿不到任何有效读数。**

> **→ 结论：** ☠️ **原写「这个 bug 无法用参数绕过，必须重编重刷固件」—— 已作废**；
> **该结论只对 fork 成立。对 z0：这份板级配置里那一行本来就是 `ADC12_CH(8)`，
> 无需绕、也无需重刷**（☠️ ~~这个 bug 不在它的固件里~~ —— **2026-10-09 更正**：bug 在 z0 的板级目标
> `hkust/nxt-dual` 里，只是**这份构建已修好**；见本节页首更正块）。
> ⚠️ 因此下面那条"待台架验证的次要猜测"**没有对象**（`BAT1_I_CHANNEL` 不必改）。
> 附带一个**待台架验证的次要猜测**（**不是修复方案**）：把 `BAT1_I_CHANNEL` 设成 `8`
> 也许能把"读到错误引脚的天晓得的垃圾值"变成"干净的 0"，即把**不可判定**的坏读数变成
> **可判定**的"无电流采样"。这可能比现在好（假电流会骗过库仑计），但**只能在台架上试**，
> 且改之前要确认它不会让 `battery_status` 整条报错。

**对 Q15 的影响**：用户已在 Q15 接受了「加一条电压下限」的修正 ——
☠️ **原写「这条修正的必要性被第一手证实了 —— 电流/库仑计数这条路是假的」—— 已作废**（见下方更正）。

> ☠️ **2026-09-23 更正**：**"电流这条路是假的"不成立。** 2026-09-21 实测（[`params-changes.md`](params-changes.md) §A4）
> 未解锁状态读到的 `current_battery` 是 `[37, 38, 36, 37, 36, 36]`（10 mA 单位，≈0.37 A，**有变化**），
> `current_consumed = 23 mAh` —— **电流是能读的**。
> 「加一条电压下限」这条修正**仍然要做**，但理由是**电压下限本来就是独立的安全带**
> （`BAT_LOW_THR = 0.15` / `CRIT = 0.07` 都已设值、`COM_LOW_BAT_ACT = 0` 却无动作），
> **不再**靠"电流不可信"来支撑。

### 3.6 `EKF2_EV_POS_X/Y/Z = 0` —— 外参落在 **`mocap` 节点**里（这个 `0` 于是是**对的**；曾判为"漏标"）

> ★★ **2026-10-09 改判（本节结论的落点变了，先读这段）**：操作者裁定**外参做在 `mocap` 节点里**
> （`body_to_imu_x/y/z = 0/0/−0.080` m，机体系 FLU，按该帧姿态旋转），
> ⟹ **飞控上 `EKF2_EV_POS_X/Y/Z` 保持 `0/0/0`，而且现在这个 `0` 是「正确的」**（由构造保证），
> **不再是**本节下面推出来的「漏标」。⛔ **不要再往飞控填数** —— 两边各做一次 = 偏移**双计**（错一倍），
> 且两边都"看起来对"。
>
> ★ **下面这段（判定「是没标不是对」+ 推出 `≈ −0.0885 m`）没有被删掉，它仍然有用**：
> ① 它是"节点里该填多少"的**依据**（同一件事的两个落点，只是符号约定不同：
> 节点用 **FLU z 上 ⟹ `−0.080`**，飞控参数用 **FRD z 下 ⟹ `−0.0885`**）；
> ② 它带的**承载前提与两个未决项**（下面「两条还不能当结论的部分」）**对节点侧同样成立**。
> 代码里落的数是操作者给的 **80 mm**，与推导的 88.5 mm 差 8.5 mm（来源：「离地 20 mm」是估的）——
> **两个数都记着**，差在哪写清楚了，但没有证据判谁对。落地清单见
> [`phase4/01`](../../.scratch/phase4/issues/01-geometry-and-extrinsics.md) 的「★★ 落地」节。

**事实**：三个都是 `0.000`。

**两种情况**：

- **可能没问题**：如果动捕刚体原点被**刻意定义在 IMU 位置**上，或者你用了"相对初始位姿转发"的手法（用户的 `vrpn_relative_pose` 节点正是干这个的），那么 `0` 是**有意为之**。
- **可能是漏标**：如果是随手放上去没量偏移，那么位置估计会有一个**常值偏移**。

**核实方法**：量出动捕软件里刚体原点标记 → 飞控 IMU 中心的实际偏移（机体系 x 前 / y 右 / z 下，单位 m），填进去。

#### ★★ 2026-10-09 判定：**是「没标」，不是「对」** —— z 分量已定 `≈ −0.0885 m`（xy 仍未做）

操作者 2026-10-09 给了一条不用卡尺的路子：**读飞机停在地面时的动捕原始 z**。结果：

| 项 | 读数 / 依据 |
|---|---|
| `/vrpn/z0/pose` 原始 z（`bag/1_static`，2026-09-29，飞机在地面、未解锁） | **`+0.10850 m`**（std 0.06 mm，20346 帧 / 117 s） |
| 刚体原点是什么点 | **5 颗球的无权质心**（五路 `z0_Marker1..5` 的均值逐轴等于 `/vrpn/z0/pose`，5 位小数内相同） |
| 参考点是 IMU，不是地面 | 第一手：EKF2 每个外部位置源都按 `<源>_pos_body − imu_pos_body` 修正（`ev_pos_control.cpp:57` / `ev_height_control.cpp:51` / `ev_vel_control.cpp:55` / `gps_control.cpp:56` / `range_height_control.cpp:61`），且 z0 的 **`EKF2_IMU_POS_* = 0/0/0`**（[`z0.params`](../assets/z0.params):379-381） |
| 符号 | 机体系 z **向下**为正（`ekf2_params.c:1080` 参数原文 "down axis"）⟹ 刚体在 IMU **上方** ⟹ 取**负** |
| 结果 | **`EKF2_EV_POS_Z = 0.020 − 0.10850 ≈ −0.0885 m`**（`0.020` = 操作者给的"飞控离地 20 mm 左右"，是**估计**；每差 5 mm 结果差 5 mm） |

⟹ 上一段那条「可能没问题」的分支**被推翻**：刚体原点在**地面上方 108.5 mm**，IMU 在约 20 mm 处，
差一个量级，**不是同一个点** ⟹ 曾据此判定 **`0/0/0` 属于「漏标」**。
（★ 2026-10-09 改判见节首：这个偏移**改由 `mocap` 节点承担**，于是飞控这个 `0` 变成**正确**的；
「不是同一个点」这个事实本身没变，变的是"谁来补这个差"。）

⚠️ **两条还不能当结论的部分**（★ 对**节点侧**同样成立 —— 它们说的是这个数本身可不可信）：

1. **承载前提「动捕 z=0 平面就是地面」尚未证**（仓库里没有一手证据；这条读法默认了它）。
   验法：拿一颗反光球**平放在飞机当前位置的地上**读 pose z，应当 ≈ 球半径。
   ★ 原话是"验完才有资格写进飞控"—— **现在改成"验完这个 80 mm 才算数"**（落点变了，前提没变）。
2. **这是每场次现读的数**：2026-09-21 那次的同一读数（见 [`mocap-setup.md`](mocap-setup.md) §10 的 ticket 12 复测表）
   是 `+0.0883`，本次 `+0.10850` —— **z 差 20.2 mm** ⟹ 标记点装法变了，**不能把这个数当常量**。
   ★ 这条对节点侧**更致命**：`mocap.yaml` 里的 `−0.080` 是个**写死的默认值**，
   每场次装态一变它就不再对 ⟹ 启动日志每次都把外参打出来，就是为了让人当场复核。

★ **顺带纠正本节的判据用法**（免得白等信号）：这个偏移在**静止 / 水平飞行**里**看不出**
（EKF 原点在初始化那一瞬定下，参考点与测量点同时平移；实测地面静止时 `/mavros/local_position/pose`
的 z = `−0.00022 m`，与填 0 还是填真值无关）。能露头的只有**姿态变化时杠杆臂的水平投影**，
误差 ≈ `|d|·sinθ` ⟹ 80～88.5 mm 的偏移在 10° 倾角下 ≈ **14~15 mm** 水平误差。
⟹ 这条判据**必须包含一段带姿态变化的飞行**才判得了（**至今一次都没跑过**）。
完整推导、复现配方与"xy 这条路取不到"见 [`phase4/01`](../../.scratch/phase4/issues/01-geometry-and-extrinsics.md)。

### 3.7 ⚠️ `EKF2_GPS_CTRL = 7` —— 室内动捕飞行建议设 0

**事实**：`EKF2_GPS_CTRL = 7`（bit0 经纬度 + bit1 高度 + bit2 速度，**全开**），同时 `SYS_HAS_GPS = 1`、`EKF2_GPS_CHECK = 245`、`GPS_1_CONFIG` 有配置。

**矛盾点**：`SYS_HAS_MAG = 0`（明确声明"无磁罗盘"）却**没有**声明"无 GPS"。而 `COM_ARM_WO_GPS = 1`（允许无 GPS 解锁）又暗示知道可能没 GPS。

**建议**：室内纯动捕飞行时设 `EKF2_GPS_CTRL = 0` + `SYS_HAS_GPS = 0`，把 GPS 这条线彻底断干净，避免"偶尔收到几颗星"造成的估计抖动。**需确认这是否会影响你想保留的 GPS 使用场景。**

> ★ **2026-09-22 已裁定：就按上面改（`0` / `0`）** —— 见 [`params-changes.md`](params-changes.md) §I 第 1 条。
> 裁定时的状态是「**已裁定、待写入**」（飞控没上电，`/dev/ttyTHS1` 上探不到心跳）。

### 3.8 ⚠️ `UXRCE_DDS_CFG = 0` —— uXRCE-DDS **当前是禁用**的

> ☠️ **2026-10-13（票 13 收口轮）：本节"对项目的影响"整条作废 —— 那个必须改的参数，定案是不改。**
> 本节的**事实**读得对（`:462` 的 `UXRCE_DDS_CFG = 0`、`:464` 的链路走 MAVLink），
> 但由此推出的动作**反了**：`(C)` 级下行改走 **mavros**（操作者 2026-10-12 定案），
> 所以 **`UXRCE_DDS_CFG` 保持 `0` 才是定案**；`:467` 那句「**必须改这个参数**」与
> `:469` 的「这条路要自己从零配」**今天都没有对象**。定案见
> [`px4-interface.md`](px4-interface.md) §4.3 / [`13`](../../.scratch/phase4/issues/13-mavros-downlink-switch.md)。
> ⛔ 原文留存（按旧前提读）：

**事实**：`UXRCE_DDS_CFG = 0`（Disabled），全文件只有这 1 条 `UXRCE_` 参数。

**含义**：z0 的机载电脑链路走的是 **MAVLink**（`MAV_1_CONFIG = 102` / `MAV_1_MODE = 2` = TELEM2 / Onboard），**不是 uXRCE-DDS**
（原文写"这台**参考机**"—— 2026-09-21 对账后已确认 `z0.params` 就是 z0 飞控真身，见页首 ★）。

**对项目的影响**：本项目 `(C)` 方案打算用 uXRCE-DDS 发 `/fmu/in/vehicle_rates_setpoint`，**必须改这个参数**。具体配置方式见 [`px4-interface.md`](px4-interface.md)。

**⚠️ 这同时说明**：`z0.params` 里**没有**任何 uXRCE-DDS 的配置经验可参考。uXRCE-DDS 这条路要自己从零配。

### 3.9 ✅ `COM_RC_IN_MODE = 3` —— **已结案**（本轮第一手核实，原疑虑解除）

**原文的疑虑**：枚举值在不同 PX4 版本间变过，`3` 可能是 "MAVLink RC"，若是则物理遥控器被忽略，
与 `COM_RC_OVERRIDE` 的安全意图冲突。

**→ 已对照 z0 固件源码结案**（**真身** `Minderring/PX4-Autopilot @ 4a0e65f2`，
`src/modules/commander/commander_params.c:168-186`，逐字 —— ☠️ 原引 fork `5ba2e91f`，已按真身复核：
该块在真身中为 `:168-186`，**注释与 `PARAM_DEFINE_INT32(COM_RC_IN_MODE, 3)` 逐字一致**）：

```
/**
 * RC control input mode
 *
 * A value of 0 enables RC transmitter control (only). A valid RC transmitter calibration is required.
 * A value of 1 allows joystick control only. RC input handling and the associated checks are disabled.
 * A value of 2 allows either RC Transmitter or Joystick input. The first valid input is used, will fallback to other sources if the input stream becomes invalid.
 * A value of 3 allows either input from RC or joystick. The first available source is selected and used until reboot.
 * A value of 4 ignores any stick input.
 *
 * @value 0 RC Transmitter only
 * @value 1 Joystick only
 * @value 2 RC and Joystick with fallback
 * @value 3 RC or Joystick keep first
 * @value 4 Stick input disabled
 */
PARAM_DEFINE_INT32(COM_RC_IN_MODE, 3);
```

**结论**：

1. `3` = **"RC or Joystick keep first"**，**不是** MAVLink RC。**物理遥控器是通的**，原疑虑解除。
2. `3` **同时是 PX4 的默认值**（`PARAM_DEFINE_INT32(COM_RC_IN_MODE, 3)`）——所以这一条**不是**被人改过的。
3. ⚠️ **但引入一个新的操作坑**：「**第一个可用源被选中并用到重启为止**」。
   若**开机时 joystick / MAVLink 源先就绪**，遥控器就会被**锁定到重启**为止。
   这与 Nxt-FC 上游刚修的那个 bug 是**同类风险**（上游 commit `b7aeebb`：
   "宿主机开机太快导致 autostart.service 启动得太早，Wi-Fi/VRPN 是 not ready"）。
   **→ z0 的启动流程必须保证：遥控器链路先于任何 joystick 源就绪，否则安全员的接管能力会在本次上电期间失效。**
4. **仍待核实**（见 §3.10）：物理接收机到底接在哪个口、走 CRSF 还是 MAVLink。`COM_RC_IN_MODE` 只决定
   "接受哪类源"，不决定"接收机接在哪"。

☠️ **2026-09-23 更正：上面第 3 条从"预测"变成了"实测"，且处置反过来 —— 值已改成 `0`。**

第 3 条预言的场面**在真机上发生了**：QGC 先上线 ⟹ MAVLink 源被 `_first_valid_source` 锁到重启
（`ManualControlSelector.cpp:59-62`，真身 `4a0e65f2`）⟹ `ManualControl::processSwitches()` 那条
`if (data_source == SOURCE_RC)`（`ManualControl.cpp:172`）永不成立 ⟹ **ch6 模式开关 / ch5 解锁开关 /
☠️ ch8 杀车 / 内八手势 全部静默失效**（现场症状：拨 ch6 模式不变，且**没有任何 STATUSTEXT**）。
**⟹ 本节第 1/2 条（"枚举含义没问题"）仍然成立，但"不是警报"这个处置是错的。**
现役值 = **`0`**（RC only）。完整链条、复现证据与两条操作纪律见
[`params-changes.md`](params-changes.md) **§I.3**（本节不再重复）。


### 3.10 ⚠️ `MAV_0` / `SER_TEL1_BAUD` 与遥控器接收机的关系

**事实**：`MAV_0_CONFIG = 101`（TELEM1）、`MAV_0_MODE = 0`（Normal）、`SER_TEL1_BAUD = 921600`。

而**用户自己的笔记**里建议的是 `SER_TEL1_BAUD = 57600 8N1`（TELEM1 接遥控器数传）。

**两个待核实点**：

1. **波特率不一致**（921600 vs 57600）。
2. **协议更根本**：ELRS 接收机输出的是 **CRSF**，不是 MAVLink。若要把 ELRS 接收机接到 UART，通常需要 `SERIALx_PROTOCOL = 23`（RCIN）而不是把该口配成 `MAV_x_CONFIG`。**`z0.params` 把 TELEM1 配成了 MAVLink 口——这可能说明那台机器没用物理遥控器，或者用的是 MAVLink 方式的 RC。**

→ **结合 3.9 一起核实。** 这两条如果都错，会直接影响"安全员能否接管"。

### 3.11 📌 `MPC_THR_HOVER = 0.5` —— 建议实测后回填

**事实**：`MPC_THR_HOVER = 0.5`，`MPC_THR_MIN = 0.12`，`MPC_THR_MAX = 1.0`，`THR_MDL_FAC = 0.45`，`MPC_THR_CURVE = 0`。

**分析**：
- 按 [`../PROJECT.md`](../PROJECT.md) 第 2.3 节的推算，理论悬停油门 ≈ **42.9 %**（`√(187.5 / 1020)`，其中 187.5 g 是单电机承担的悬停推力）。
- `0.5` **恰好是 PX4 的默认值**，看不出被改过的痕迹。
- 但 `MPC_THR_MIN = 0.12` 和 `THR_MDL_FAC = 0.45` 都是非默认值，说明这个参数组确实被人动过——只是 `THR_HOVER` 可能没单独调。

**📌 对模型预测控制的价值**：`MPC_THR_HOVER` 是 PX4 位置控制器的**线性化工作点**，也是你模型预测控制模型里"悬停推力"的直接参考值。建议实测后同时回填到 PX4 和模型预测控制模型两处。

**实测手段**：`Fast-Drone-XI35` 的 `thrust_calibrate.py`，或 PX4 Autotune（`MC_AT_EN = 1`、`MC_AT_APPLY = 1` 已经开好了，只差 `MC_AT_START`）。

### 3.12 📌 模型预测控制必须尊重的约束上限（来自本文件）

这些值定义了 **模型预测控制输出的 ω_des 的可行域**，可以直接抄进模型预测控制的约束里：

| 约束 | 参数 | 值 |
|---|---|---|
| 最大横滚角速率 | `MC_ROLLRATE_MAX` | **220 °/s** |
| 最大俯仰角速率 | `MC_PITCHRATE_MAX` | **220 °/s** |
| 最大偏航角速率 | `MC_YAWRATE_MAX` | **360 °/s** |
| 最大倾角（空中） | `MPC_TILTMAX_AIR` | **45°** |
| 最大水平速度 | `MPC_XY_VEL_MAX` | **12 m/s** |
| 最大上升速度 | `MPC_Z_VEL_MAX_UP` | **3 m/s** |
| 最大下降速度 | `MPC_Z_VEL_MAX_DN` | **0.5 m/s** |
| 最大水平加速度 | `MPC_ACC_HOR_MAX` | **5 m/s²** |
| 推力下限 / 上限 | `MPC_THR_MIN` / `MAX` | 0.12 / 1.0 |

> 📌 **注意 `MPC_Z_VEL_MAX_DN = 0.5 m/s` 很保守**，`MPC_TILTMAX_AIR = 45°` 也偏保守。这些是**为 PID 位置环设的安全边界**，模型预测控制想要更激进的机动时**可以放宽**——但别忘了第 2.4 节的电池瓶颈才是真正的天花板。

### 3.13 ★ `COM_RC_OVERRIDE = 3` —— 保留这条安全设置

**事实**：`COM_RC_OVERRIDE = 3`，`COM_RC_STICK_OV = 20`（用户笔记里建议 30），`COM_OBL_RC_ACT = 0`（= Position 模式），`COM_OF_LOSS_T = 1.0 s`。

**含义**（用户笔记原文）："安全设置：允许打杆时强制退出 Offboard 模式，切至 Position 模式。"

→ **✅ 这是模型预测控制实飞时最重要的安全网，务必保留并理解其行为。**
→ ⚠️ 但它依赖"飞控能读到遥控器"——**回到 3.9 / 3.10 的核实**。安全网不能是假的。

### 3.14 ✅ 其他可直接复用的值

| 参数 | 值 | 说明 |
|---|---|---|
| `SYS_AUTOSTART` | `4001` | Generic Quad X |
| `SYS_MC_EST_GROUP` | `2` | 使用 **EKF2**（不是已废弃的 LPE） |
| `SYS_HAS_MAG` | `0` | ✅ 无磁罗盘（与 NxtPX4v2 硬件一致） |
| `MAV_TYPE` | `2` | 四旋翼 |
| `EKF2_EV_CTRL` | `11` | 1(水平位置) + 2(垂直位置) + 8(yaw) = 融位置与 yaw，**不融速度** |
| `EKF2_HGT_REF` | `3` | 高度基准 = **Vision** |
| `EKF2_MAG_TYPE` | `5` | 无磁罗盘模式 |
| `EKF2_EV_DELAY` | **`50.0`** | ★ **与你实测的 50 ms 完全一致** —— 强佐证 |
| `EKF2_EV_NOISE_MD` | `0` | 噪声由参数给定，不从消息读取 |
| `EKF2_EV_QMIN` | `0` | 质量门限关闭 |
| `COM_ARM_WO_GPS` | `1` | 允许无 GPS 解锁（室内必需） |
| `COM_DISARM_PRFLT` | `10 s` | 解锁后不起飞自动上锁 |
| `COM_KILL_DISARM` | `5 s` | |
| `MOT_POLE_COUNT` | `14` | ✅ 与 Spinnybois 2006 的 12槽14极 一致 |
| `DSHOT_MIN` / `DSHOT_3D_ENABLE` | `0.055` / `0` | DShot 最小值 / 非 3D 模式 |
| `BAT1_N_CELLS` | `6` | ✅ 6S |
| `BAT_AVRG_CURRENT` | **`15.0`** | ★ **恰好等于 VTC6 的实测持续放电能力** —— 说明这个值被认真设过 |
| `MOT_SLEW_MAX` | `0` | 无油门变化率限制 |
| `MPC_USE_HTE` | `1` | 悬停推力估计器开启 |

### 3.15 ★★★ 本轮新发现的参数（旧版本文档完全没有；全部第一手核实）

以下参数在旧版 `params-analysis.md` 中**一条都没有出现**，是本轮（2026-09-20）重验 `z0.params` 时挖出的。

#### (a) ★★★ 遥控器通道占用 —— 旧文档说「ch5 空闲」是**错的**

`RC_MAP_*` 的完整占用（`z0.params` 逐条）：

| 通道 | 功能 | 参数 |
|---|---|---|
| ch1 | 横滚 | `RC_MAP_ROLL = 1` |
| ch2 | 俯仰 | `RC_MAP_PITCH = 2` |
| ch3 | 油门 | `RC_MAP_THROTTLE = 3` |
| ch4 | 偏航 | `RC_MAP_YAW = 4` |
| **ch5** | **解锁开关** | **`RC_MAP_ARM_SW = 5`** |
| ch6 | 飞行模式开关 | `RC_MAP_FLTMODE = 6` |
| **ch8** | **杀车开关** | **`RC_MAP_KILL_SW = 8`** |
| ch7 | — | 未被 `RC_MAP_*` 占用 |

其余 `RC_MAP_*`（`ACRO_SW` / `MODE_SW` / `OFFB_SW` / `POSCTL_SW` / `RETURN_SW` / `LOITER_SW` / …）**全为 `0`（未分配）**。

**后果（安全相关）**：新增的「开始 / 停止」通道**不能**与**解锁（ch5）**、**模式（ch6）**、**杀车（ch8）**撞车，
否则一拨就同时干了别的事。旧文档 §8.4 只警告了 ch6，**漏了 ch5 和 ch8**。

> ☠️ **2026-09-21 更正（本节原先写「只剩 ch7 空闲」，那句是错的）**：
> 「空闲」要分两个层面看 —— **PX4 侧**未被映射的通道有 **9 个**（**ch7 与 ch9…ch16**，
> 因为 `RC_CHAN_CNT = 16`），不是 1 个；**遥控器侧**才是真正的约束：哪个物理开关被分配到了哪个通道，
> 那要读遥控器（EdgeTX 的 通道→开关 分配），**PX4 的参数表回答不了**。
> 实测读数（2026-09-21，751 条 `RC_CHANNELS`：ch7 = 2000 恒定、ch9 = 999 恒定，**但那次窗口内没人拨动开关**，
> 所以"ch7 上有开关 / ch9 上没有"只是**推断**）见
> [`params-changes.md`](params-changes.md) §H.3。
>
> ★ **2026-09-21 已定案（ticket [02](../../.scratch/phase1/issues/02-rc-channel-decision.md)）**：
> **开始 = ch9 的上升沿；急停 = ch8 高值（沿用现有硬件杀车，不新增软急停通道）；
> 解锁 = 程序解锁（触发 = ch9 第 1 次上升沿，第 2 次上升沿才开始轨迹）；`RC_MAP_ARM_SW = 5` 保留。**
> 完整结论、依据与连带要求见 [`params-changes.md`](params-changes.md) **§H**（本节不再复述）。
> 台架复核（用 `/mavros/rc/in` 核对 `channels[8]`）**至今未做** —— 它现在是
> [`params-changes.md`](params-changes.md) **§H.8 / §H.10** 的未决项（ticket 09 的验收项 6 因此**留空**），
> 见 [`../../.scratch/phase1/spec.md`](../../.scratch/phase1/spec.md) 的阻塞项第 4 条。

#### (b) ★ 飞行模式开关的实际分配（枚举值第一手来自固件 `COM_FLTMODE${i}`）

`z0.params`：`COM_FLTMODE1 = 2`、`COM_FLTMODE4 = 1`、`COM_FLTMODE6 = 7`，其余 `-1`（未分配）。

固件里的枚举（**真身** `Minderring/PX4-Autopilot @ 4a0e65f2`，
`src/modules/commander/commander_params.c:361-374` 的 `@value` 清单，逐字）：

```
-1: Unassigned          0: Manual          1: Altitude        2: Position
 3: Mission             4: Hold           10: Takeoff        11: Land
 5: Return              6: Acro             7: Offboard        8: Stabilized
12: Follow Me          13: Precision Land
```

> ☠️ **2026-09-23 重新锚定 + 一处事实更正**：
> 1. **文件引错了**：原文引 `src/modules/commander/module.yaml` —— 该文件**在真身（以及上游 v1.14.3）里根本不存在**
>    （`git show 4a0e65f2:src/modules/commander/module.yaml` 报 `does not exist`）。`COM_FLTMODE1..6`
>    实际定义在 `src/modules/commander/commander_params.c`，行号 **`:377 / :401 / :425 / :449 / :473 / :497`**；
>    其 `@value` 清单在 **`:361-374`**。
> 2. **原文表里那个 `9: Position Slow` 是多余的** —— 真身的 `@value` 清单里**没有 9**
>    （`grep '@value 9'` 在该文件无命中）。删掉它**不影响本节结论**：z0 用到的是 `2`/`1`/`7`。
> 3. 其余 **14** 个值（`-1/0/1/2/3/4/5/6/7/8/10/11/12/13`）在真身中**逐字一致**。

**所以 z0 的模式开关实际是**：

| 档位 | 模式 |
|---|---|
| 1 | **Position** |
| 4 | **Altitude** |
| 6 | **Offboard** |
| 2 / 3 / 5 | 未分配（拨过去无反应） |

**两个含义**：

1. ★ **第 6 档就是 Offboard** —— 飞行员有一条**纯硬件**路径进出 OFFBOARD，
   与 `COM_RC_OVERRIDE`/`COM_OBL_RC_ACT` 是**两套独立机制**。设计 Q5 的接管逻辑时必须把这条算进去。
2. ⚠️ **`0: Manual` 在 z0 上并未分配**（旧文档和我本人在讨论中都曾误以为手动模式可用）。
   z0 上最"手动"的退路只有 **Altitude**（第 4 档）或 **Position**（第 1 档），
   **没有一个真正的手动油门模式**。这会影响应急程序的写法。

#### (c) ★ 两条与 Q5 直接相关的安全参数（旧文档只提了名字，没给基准）

| 参数 | z0 值 | PX4 默认 | 判定 |
|---|---|---|---|
| `COM_RC_OVERRIDE` | **`3`** | `1` | **刻意改过**（放宽了打杆接管的适用范围） |
| `COM_RC_STICK_OV` | **`20`** | `30.0` | **刻意收紧**。☠️ **更正（2026-09-22）**：原句写"打杆 20% 即接管"，**读错了** —— 这个阈值比的是**归一化摇杆的每秒变化率**（`0.2/s`），不是偏移量。第一手链条见 [`params-changes.md` §H.11](params-changes.md) |

→ 两条合起来说明：**「打杆接管」在 z0 上是被人有意配强过的**，是 Q5 方案的真实参数基础。

#### (d) ★★★ `NAV_RCL_ACT` —— 遥控丢失时**返航**（旧文档完全没有）
（☠️ 标题原写 `= 2`，**那是本节写下时的值、不是现值**；**现值 = `1`（Hold）**，★ 2026-09-28 第一手实测，见下。）

> ✅ **2026-09-28 状态更新：本节的风险已消除** —— 本节的建议（改 `1 = Hold`）**已落地**，
> 第一手实测真身 `NAV_RCL_ACT = 1`（见 [`params-changes.md`](params-changes.md) §I.1′）。
> **下面保留的是"为什么必须改"的论证**，不是现值 —— 现值是 `1`。

`NAV_RCL_ACT` 的语义是「遥控链路丢失后的动作」，`2` = **Return（返航）**。

**⚠️ 对室内动捕飞行是个真实风险**：返航需要 **home 点**，而室内**无 GPS**、home 点不可靠。
遥控一掉，PX4 会尝试返航而不是就地悬停/降落。
（对照 `NAV_DLL_ACT = 0` = 数传丢失无动作。）

> ### ⛔ 本小节有一个**我自己写错的建议**，已更正
>
> 本节曾建议「改为 **`0`（无动作）**」—— **`NAV_RCL_ACT` 根本没有 `0` 这个值。**
>
> **第一手证据**（**真身** `Minderring/PX4-Autopilot @ 4a0e65f2`，`src/modules/commander/commander_params.c:789-806`，逐字
> —— ☠️ 原引 fork `5ba2e91f` 的 `:651-668`，已按真身复核为 `:789-806`，**内容逐字一致**）：
>
> ```
> /**
>  * Set RC loss failsafe mode
>  * ...
>  * @value 1 Hold mode
>  * @value 2 Return mode
>  * @value 3 Land mode
>  * @value 5 Terminate
>  * @value 6 Disarm
>  * @min 1
>  * @max 6
>  */
> PARAM_DEFINE_INT32(NAV_RCL_ACT, 2);
> ```
>
> **合法值只有 `1 / 2 / 3 / 5 / 6`（`@min 1`）** —— 与 `NAV_DLL_ACT` 不同，
> **后者才有 `@value 0 Disabled`**（`commander_params.c:769-787` —— ☠️ 原引 `:634-649`，已按真身复核）。**两者枚举不对称，我先前把它们类比错了。**
>
> **→ 修正后的正确建议：`NAV_RCL_ACT` 改为 `1`（Hold mode）** —— 室内动捕飞行时，
> 遥控丢失后**让 PX4 的位置控制器就地保持当前位置**，这是最符合 `(A)`/`(C)` 级飞行的行为。
> （`3` Land = 就地降落，也可接受，视飞行计划定。）
> **`5` Terminate / `6` Disarm` 绝对不要用** —— 那是**停桨坠落**。
>
> ⚠️ 另注：`2` 是 PX4 的**出厂默认值**（`PARAM_DEFINE_INT32(NAV_RCL_ACT, 2)`），
> 所以这不是"被人改坏的"，而是"用默认值带来的室内风险"。
> 且该参数**只在 `COM_RC_IN_MODE` 未禁用 RC 输入检查时才会触发**（见参数注释）。

#### (e) ★ 控制分配的几何：`CA_ROTOR_COUNT = 4`（一条排除性发现）

`CA_ROTOR_COUNT = 4`、`CA_METHOD = 2`（= **Auto**，见 §3.1 的更正中关于 `CA_METHOD` 语义的部分）。所以 `CA_ROTOR4..9_*` 那一串**全零是残留、无害** ——
分配器只用前 4 个。**旧文档隐含担心过这点，可以排除。**

但这**不改变** §3.1 的结论：`CA_ROTOR0..3_PX/PY = ±0.15` 仍是错的几何，且现在**可以量化**：

- `±0.15 / ±0.15` → 臂长 `√(0.15²+0.15²) = 0.212 m` → **轴距 424 mm**
- ✅ **2026-10-09 已确认**：z0 的 152 mm **就是**电机对电机对角线、且四个电机是**正方形**布局
  ⟹ 真值 `±76/√2 = ±0.0537401 m`（不是"若…则"了，见 §3.1 的「★★ 2026-10-09 判定」）
- 现值偏大 `0.15 / 0.0537401 ≈ 2.79×` —— **但这只是均匀缩放，判定为不改**（同上一行）

> ⚠️ **本小节原先的"连带后果"论述已被 §3.1 的「★★ 2026-09-20 更正」推翻**，摘要如下：
> 原来说「`MC_*RATE_P*` 是配着这个错的几何调出来的，单独改几何会把调好的姿态环弄坏」——
> **两处都不成立**：① `MC_*RATE_P*` 实测**全是 PX4 出厂默认值**（`0.15 / 0.15 / 0.2`），**没有"调好的"可弄坏**；
> ② 多旋翼控制分配器**按几何自身范数归一化**，均匀缩放**会被完全约掉**，改几何对分配结果**无影响**。
>
> **→ 修正后的动作（对 Q21 决策前提的实质修改）**：Q21 应为
> **「先实测电机位置 → 判断是否只是 `±0.15/±0.15` 的均匀缩放 → 是则无需改动；
> 不是（`PX≠PY` 或非正 X 布局）才需要改几何并重调」**。
> ✅ **2026-10-09：这个动作已经走完，判定是「均匀缩放」⟹ 不改、不重调**（见 §3.1 的
> 「★★ 2026-10-09 判定」）。真值 `±0.0537401` 已登记，`(D)` 方案与力矩饱和估算**直接用真值**。

#### (f) ⚠️ uXRCE-DDS 无串口可用（Phase 2/3 的实际障碍）

> ☠️ **2026-10-13（票 13 收口轮）：不再是障碍 —— 它反而成了定案的依据。**
> 本节下面那三行**事实**（`UXRCE_DDS_CFG = 0` / `MAV_2_CONFIG = 0` / `MAV_0`｜`MAV_1` 的分配）
> 一手读得都对，而且**正是它们**让操作者在 2026-10-12 把 (C) 级下行定成 **mavros**：
> 没有任何空闲串口 ⟹ uXRCE-DDS **不是"待解决"而是"做不到"**。
> ⟹ 本节结论「**Phase 2 落地时必须先解决的实际问题**」**今天不成立**（那个问题被绕开了，不是被解决了）。
> 定案见 [`px4-interface.md`](px4-interface.md) §4.3；⚠️ 归属：本节的**重写**属本页作者，
> 本轮只按票 13 §一④ 的口径**就地留痕**。

`UXRCE_DDS_CFG = 0`（禁用）、`MAV_2_CONFIG = 0`（无第三个 MAVLink 口）、
`MAV_0_CONFIG = 101`（TELEM1）、`MAV_1_CONFIG = 102`（TELEM2，Onboard = mavros 在用）。

→ **没有任何串口被分配给 uXRCE-DDS。** 旧文档只说"要开这个参数"，
**没说开在哪个物理口上** —— 这是 Phase 2 落地时必须先解决的实际问题。

> ### ⛔ 这里有一处已更正的事实（我先前写错了）
>
> 本文曾写「`SER_GPS1_BAUD = 0` 说明 GPS 口是关的，理论上可以挪用」—— **错。**
>
> **第一手证据**（**真身** `Minderring/PX4-Autopilot @ 4a0e65f2`，`Tools/serial/serial_params.c.jinja:11-13`，逐字
> —— ☠️ 原引 fork `5ba2e91f`，已按真身复核：**该文件该行号在真身中逐字一致，无需改动**）：
>
> ```
>  * Note: certain drivers such as the GPS can determine the Baudrate automatically.
>  *
>  * @value 0 Auto
> ```
>
> **`0 = Auto`（自动波特率），不是"关闭"。** 而 GPS 子系统在 z0 上其实是**完整开着的**：
>
> | 参数 | 值 | 含义 |
> |---|---|---|
> | `SYS_HAS_GPS` | **`1`** | 声明**有** GPS |
> | `GPS_1_CONFIG` | **`201`** | GPS1 端口**已分配** |
> | `GPS_1_PROTOCOL` | `1` | u-blox |
> | `EKF2_GPS_CTRL` | `7` | 经纬度 + 高度 + 速度**全开** |
> | `EKF2_GPS_CHECK` | `245` | 检查全开 |
> | `SENS_GPS_MASK` | `7` | — |
> | `SER_GPS1_BAUD` | `0` | **Auto**（默认值） |
>
> **→ 两个后果**：① 「GPS 口是空的、可以挪给 uXRCE-DDS」这条**线索不成立**，
> `GPS_1_CONFIG` 已经被占用；② 与 §3.14 的 `SYS_HAS_MAG = 0`（明确声明无磁罗盘）形成对比 ——
> **无磁罗盘是显式声明的，而 GPS 是显式开着的**，室内纯动捕飞行时这才是 §3.7 那句"建议设 0"的真正理由。
>
> **⚠️ 但仍有一条可查的路**：`SER_GPS1_BAUD = 0` 是**默认值**，说明这个口可能从来没接过东西
> （否则会填具体波特率）。**实际有没有 GPS 硬件，要看 `GPS_1_CONFIG = 201` 对应的物理口上接了什么** ——
> 这需要看硬件走线，不是参数能回答的。（`2026-09-20` 未核实。）

#### (g) `CBRK_SUPPLY_CHK = 894281` —— 电源检查被旁路

魔术值，**禁用电源预检**（无电池也能解锁）。对一台**真装了电源模块**的机器，这条**没有必要**，
且会**掩盖电池没接/没读**的情况 —— 与 §3.5.1 的 ADC 问题叠加，风险更高。
**建议首飞前评估是否恢复检查。**

---

## 4. ⚠️ 与你自己的笔记建议值不一致的参数

用户笔记 `~/weisama-log/25.06.09_UAV_mocap/25.06.26_mocap.md`（**本地仓库，不在本仓库内**）里有一份**自己试出来的** PX4 参数建议表（标注为 PX4 **v1.16**）。与本文件对比，**有多处不一致**：

| 参数 | `z0.params` (v1.14) | 你笔记的建议 (v1.16) | 说明 |
|---|---|---|---|
| `EKF2_EV_DELAY` | **50** | 125 | ★ **`z0.params` 的 50 与你实测的 50 ms 吻合**；笔记表里的 125 可能是另一套系统的值 |
| `EKF2_EVP_NOISE` | **0.06096** | **1.0**（并注明"实测 0.1 过小会发散"） | ⚠️ **`0.061` 比你自己标注的"会发散"的 0.1 还小** —— 这是最值得注意的一处差异 |
| `EKF2_EVA_NOISE` | 0.001745 | 10 | ⚠️ 差 4 个数量级 |
| `EKF2_EVV_NOISE` | 0.1 | — | |
| `EKF2_NOAID_NOISE` | 3.048 | 10 | 笔记说"不建议改变" |
| `EKF2_BARO_CTRL` | `1`（**开**） | `0`（**关**） | ⚠️ 见下方说明 |
| `EKF2_BARO_GATE` | `1000` | 笔记 1.14 版建议"设很大值" | ✅ 一致 —— `z0.params` 用 1.14 的手法（大 gate）变相禁用气压计 |
| `SER_TEL1_BAUD` | 921600 | 57600 | ⚠️ 见 3.10 |
| `SDLOG_MODE` | `2`（解锁时记录） | `1`（上电到掉电） | 次要 |
| `MPC_THR_HOVER` | 0.5 | — | 见 3.11 |

**关于 `EKF2_BARO_CTRL`**：`z0.params` 里 `EKF2_BARO_CTRL = 1`（启用）**但** `EKF2_BARO_GATE = 1000`。这正是 1.14 时代"变相禁用气压计"的写法（用户笔记 1.14 段落原文：*"设置为一个很大的值（例如 1000）会大幅降低或禁用气压计对高度的影响"*）。到 1.16 才有 `EKF2_BARO_CTRL = 0` 这个直截了当的开关。**所以两者不矛盾——是同一意图的两个版本写法。** ✅

**关于 `EKF2_EVP_NOISE = 0.06096`**：这个值到底能不能用，**取决于动捕的数据质量和 `EKF2_EV_DELAY` 是否准**（用户笔记第 5 条原文：*"正确的延迟补偿参数 EKF2_EV_DELAY，可以减小坐标噪声方差从而减小 EKF2_EVP_NOISE"*）。参考机延迟 50 ms 对上了，所以它敢用这么小的噪声。**你的 z0 延迟如果不同，这个值不能照抄。**

> ✅ **2026-09-28 答复这个"如果"**：z0 的延迟已从飞控 ulog **实测 ≈ 50 ms**（点估计 49.2 ms）
> （[`mocap-setup.md`](mocap-setup.md) **§6.3**）—— **与参考机的 50 ms 同一量级，没差出量级来**
> ⟹ **这个条件成立，`0.06096` 可以照抄**（下面那条"按机器调出来的"的提醒仍然有效，
> 但不再有一个"延迟未知"的悬置风险压在上面）。

> **结论：EKF2 的噪声参数（`EVP/EVA/EVV/NOAID_NOISE`）是"按机器调出来的"，不是"配置出来的"。照抄有风险，建议作为起点然后按笔记第 3/4 条的方法微调。**

---

## 5. ★ 推荐动作：与你自己飞控的参数做 diff —— ✅ **已执行（2026-09-21）**

> ✅ **结论**：**914/915 条名字全对上**（唯一多出的是飞控内部的 `_HASH_CHECK`）；
> 其中 **21 条值不同**（6 条加速度计标定 + 12 条遥控器校准 + 3 条飞行计数器），**其余 893 条值一致**。
> ⟹ **本文没有一处"文件过时"** —— 所有 ⚠️ 项都是**飞机上的真实现状**。
> **执行时走的不是下面那段 QGC 路径，而是 `ros2 param dump`**（mavros 已连通，见 ticket 14）；
> 下面的原始步骤**保留作历史记录**，因为 `diff` 的比对方法本身没变。

```bash
# ★ 实际用的（2026-09-21，mavros 已连通时）—— 915 条，~45 s
ros2 param list /mavros/param | wc -l
ros2 param dump /mavros/param > /tmp/fcu_params.yaml
# ⚠️ 读参数别用 /mavros/param/get —— ROS2 版 mavros 没这个服务（ticket 15 §五 坑 1）
```

**原步骤（QGC 路径，未走；保留）**：

```bash
# 1) 用 QGC 连上 z0 的飞控 → 参数页 → 右上角"工具" → "保存到文件"
#    存为 z0-export.params

# 2) 与参考文件对比（两边都滤掉注释行、排序后 diff，避免顺序干扰）
cd /home/chen/ros_ws/docs/assets
diff <(grep -v '^#' z0.params | sort) <(grep -v '^#' /path/to/z0-export.params | sort)
```

> 📌 **导入参数文件时的注意事项**（来自上游仓库随 `uav1.params` 附的说明）：
> **"QGC 导入 params 文件，修改 `MAV_SYS_ID` 为对应编号（默认 1）"**
> → 上游原件头部原本写的是 `# Onboard parameters for Vehicle 1`（本仓库的 `z0.params` 已把这行改成 z0 的出处说明），且 `MAV_SYS_ID = 1`。**z0 是单机，保持 1 即可**；将来多机时每台要改成不同编号。

**重点看差异里有没有出现**：

- `CA_ROTOR*` —— 几何（3.1）
- `PWM_MAIN_FUNC*` —— 电机序（3.2）
- `SYS_AUTOSTART` / `CA_AIRFRAME` —— 机型选择
- `COM_RC_*` / `MAV_0_*` / `SER_TEL1_BAUD` —— 遥控器链路（3.9 / 3.10）★★ 安全相关
- `BAT1_*` / `CBRK_SUPPLY_CHK` —— 电池链路（3.5）
- `EKF2_EV_POS_*` —— 外参（3.6）
- `UXRCE_DDS_CFG` —— 中间件（3.8）

> ✅ **实际结果（2026-09-21）**：上面这 7 组**一组都没出现在差异里**。差异只有 **21 条** ——
> `CAL_ACC0/1_XOFF/YOFF/ZOFF`（**6** 条加速度计标定）、
> `RC1_MIN` / `RC2_MAX` / `RC3_MAX` / `RC4_MAX` / `RC6_MIN+TRIM` / `RC7_MIN+TRIM` / `RC9_MIN+TRIM` / `RC10_MIN+TRIM`（**12** 条遥控器校准）、
> `COM_FLIGHT_UUID` + `LND_FLIGHT_T_HI` + `LND_FLIGHT_T_LO`（**3** 条飞行计数器）。6+12+3 = **21** ✓
> ⟹ **原来那批"⚠️ 需核实"不是文件过时，是飞机上的真实现状**。

> 💡 **原预判 vs 实际**：原文写"`CAL_*` 那 **49 条**会全部显示为差异"—— **方向对（`CAL_*` 确实会差），幅度错**：
> 实际只有 **6 条**（全是 `CAL_ACC*`；`CAL_MAG*` / `CAL_GYRO*` 在真身上与文件**一致**，21 条差异里没有它们）。
> **教训不变：不要照抄 `CAL_*`**（它跟具体硬件绑定）。

> ☠️ **顺带更正 ticket 15 的两个分类计数**：ticket 15 §二 的小标题写「飞行计数器（**4**）」「遥控器校准（**14**）」，
> 但它**自己列出的名字**分别是 **3** 条与 **12** 条，且 6+4+14 = 24 ≠ 它自己给的合计 **21**。
> **以本行的 6+12+3 = 21 为准**（与它列的参数名逐一对得上）。**总数 21 与结论都不受影响。**

---

## 6. 待办清单 —— ⚠️ **已被 [`params-changes.md`](params-changes.md) 取代**，保留作取证记录

> ★ **行动清单看 `params-changes.md`**（它有"阻塞/建议"分级，2026-09-22 起还有 **§I 的人裁定**）。
> 本表保留，是因为它是**对账前**写下的原始待办 —— 对照它的"状态"列能看出**哪些"核实"动作被对账直接替代掉了**。
> 状态由 ticket [15](../../.scratch/phase1/issues/15-fcu-param-reconciliation.md) 的对账回填（2026-09-21/22）。

| # | 事项 | 方法 | 优先级 | 2026-09 状态 |
|---|---|---|---|---|
| 1 | 导出自己的 `.params` 并 diff | 实走 `ros2 param dump`（原写「QGC → 保存到文件」，已改） | ★★★ | ✅ **已做（09-21）**：914/915 一致 |
| 2 | 核实 `CA_ROTORx_PX/PY` 实际值 | `ros2 param get /mavros/param CA_ROTOR0_PX` | ★★★ | ✅ **值已核实 = ±0.15**（真身就是它，见 `params-changes.md` §A2）；⚠️ **"实测几何"仍未做** —— 归 §I 第 3 条 |
| 3 | 核实遥控器链路（`COM_RC_IN_MODE` 枚举含义 + `MAV_0`/CRSF 协议） | QGC 参数页悬停提示 + 实机验证 | ★★★ **安全** | 🔶 **各自一半**：`COM_RC_IN_MODE = 3` 已结案（§3.9）；**`MAV_0`/CRSF 的物理接法仍未验** —— 且 09-21 实测时 **RC 链路是断的**（`params-changes.md` §H.9），没做成 |
| 4 | 核实电池监测是否工作 | `listener battery_status` | ★★ **安全** | ✅ **已做（09-21）：未中招** —— 读数与判据见 `params-changes.md` §A4 |
| 5 | 确认电机序与桨转向的**对应关系**（不是只看电机序） | 实机 + 对照 `CA_ROTOR` 表 | ★★★ | ⬜ **未做**（人的事，要上真台架） |
| 6 | 量出 `EKF2_EV_POS_X/Y/Z` 外参 | 尺子量刚体原点→IMU | ★★ | 🔶 **2026-10-09 落点在节点侧**：外参由 `mocap` 节点的 `body_to_imu_*` 承担（z = 80 mm，xy 仍按零），飞控**保持 `0/0/0`** ⟹ 本条**不再回填飞控** —— §I 第 7 条 / §3.6 节首 |
| 7 | 实测悬停油门，回填 `MPC_THR_HOVER` | `thrust_calibrate.py` 或 Autotune | ★★ | ⬜ **未做** —— §I 第 5 条；`THR_MDL_FAC` 一并定 |
| 8 | 决定 `EKF2_GPS_CTRL` 是否设 0 | 看是否需要 GPS 场景 | ★ | ★ **已裁定：现在就改 `0`**（09-22，§I 第 1 条），**待写入** |
| 9 | ~~打开 uXRCE-DDS（`UXRCE_DDS_CFG`）~~ ☠️ **2026-10-13（票 13 收口轮）：这一条取消 —— 不做，而不是没做** | 见 [`px4-interface.md`](px4-interface.md) | ~~★★（走 `(C)` 方案时）~~ | ⛔ **不适用**：`(C)` 方案**没变**，但它的载体 2026-10-12 定成 **mavros** ⟹ `UXRCE_DDS_CFG` **保持 `0` 是定案**（`params-changes.md` §E1 的旧前提随之作废）。 |
