# 15: ★ `z0.params` ↔ 飞控真身**逐条对账** —— 结论：文件就是真身；21 条差异全是标定/计数器

Status: ready-for-human
Type: task

> 📌 **2026-09-23 状态说明（为什么还是 `ready-for-human` 而不是 `done`）**：
> ★人裁定**已做**（§四 第 4 框 ✅）、**3 条参数已于 2026-09-23 写入并对账**（第 5 框 ✅）、
> 文档回填**已补齐**（第 8 框 ✅）。**还剩 2 框，都是纯人的事**：
> ① **量 `EKF2_EV_POS_*` 外参** —— **尺子量**；
> ② **`MPC_THR_HOVER` / `THR_MDL_FAC` 标定** —— **推力台**。
> ⟹ 按 [`triage-labels.md`](../../../docs/agents/triage-labels.md)，验收项**没勾完就不许标 `done`**；剩的是人的事，故维持 `ready-for-human`。

**为什么单开一张：** [`PROJECT.md`](../../../docs/PROJECT.md) 的"推荐动作"原话是
*"从 **z0 的飞控**导出参数，与 `z0.params` 做 diff —— 差异就是'从上游参考值到 z0 实际值'要改的东西"*；
`PROJECT.md` §参数与 `phase1-implementation.md` §6 也都标着 **"尚未在 z0 的飞控上核实"**。
2026-09-21 飞控上电 + mavros 连通后（[ticket 14](14-mavros-from-source.md)），**这一步做掉了** ——
结论**跟预期相反**，直接影响"参数要不要改、怎么改"，所以必须落纸。

**Blocked by:** 无（飞控已上电、mavros 已连通）
**被它影响的：** [`phase1-implementation.md`](../../../docs/control/phase1-implementation.md) §6 参数表、
[`params-analysis.md`](../../../docs/reference/params-analysis.md) 的"必须核实"各项、[`PROJECT.md`](../../../docs/PROJECT.md) 里程碑 1

---

## 一、怎么做（★ 方法本身也是产出：ROS2 版 mavros 读参数的坑）

```bash
source /opt/ros/humble/setup.bash && source ~/mavros_ws/install/setup.bash
ros2 param list /mavros/param | wc -l          # → 915（飞控参数被暴露成**标准 ROS 参数**）
ros2 param dump /mavros/param > /tmp/fcu_params.yaml   # 全量导出，实测 45 s
ros2 param get /mavros/param EKF2_EV_CTRL      # 单条读
```

⚠️ **不要用 `/mavros/param/get`** —— ROS2 版 mavros **没有这个服务**（源码 `mavros/src/plugins/param.cpp`
里那行被注释掉了：`// rclcpp::Service<mavros_msgs::srv::ParamGet>::SharedPtr get_srv;`），
只有 `~/pull`（`ParamPull`）与 `~/set`（`ParamSetV2`）。第一次摸的时候在这里白等了一分多钟。

> ⚠️ **2026-09-22 更正：上面"只有两个服务"这句不完整**（第一手重读 `mavros/src/plugins/param.cpp`）——
> 还有 **`~/get_parameters`**（`:468`）与 **`~/set_parameters`**（`:476`，`ros2 param set` 走的就是它，
> 真的会发给飞控：`:1148`）；`~/set_parameters_atomically` 则是**硬编码不支持**（`:1157-1164`）。
> **"没有 `~/get`"这个判断本身是对的，坑也是真的**，不影响本节的结论。
> ⟹ **实际影响在 §四**：写参数**不必**用 `ParamSetV2`。逐条表格见
> [`docs/reference/params-changes.md`](../../../docs/reference/params-changes.md) **§I.2**。

⚠️ **`/mavros/param/set` 的服务类型是 `ParamSetV2`**（不是 ROS1 时代的 `ParamSet`）—— 写参数时别按旧类型写。

## 二、结果：**914/915 条名字全对上**

| 项 | 数 |
|---|---|
| `z0.params` 里的参数 | **914** |
| 飞控真身上的参数 | **915** |
| 名字能对上的 | **914**（100%） |
| 只在真身上有 | **1**：`_HASH_CHECK`（飞控内部的标定哈希，参数文件不带它，正常） |
| **值不一致** | **21**（逐条见下，**全部是设备标定类 / 飞行计数器类**） |

### 21 条值差异（★ 一条"配置类"都没有）

| 类别 | 参数 | 参数文件 | 飞控真身 | 解释 |
|---|---|---|---|---|
| 加速度计标定（6） | `CAL_ACC0_XOFF/YOFF/ZOFF`、`CAL_ACC1_XOFF/YOFF/ZOFF` | （上游那台机器的标定） | z0 自己的标定值 | **设备标定**，换机必变。`_HASH_CHECK` 就是它的哈希 |
| 飞行计数器（4） | `COM_FLIGHT_UUID` | `1144` | **`0`** | 飞控**从未有过飞行记录** |
| | `LND_FLIGHT_T_HI` / `_LO` | `3` / `-616518067` | **`0`** / `0` | 同上（累计飞行时间 = 0） |
| 遥控器校准（14） | `RC1_MIN`、`RC2_MAX`、`RC3_MAX`、`RC4_MAX`、`RC6_MIN/TRIM`、`RC7_MIN/TRIM`、`RC9_MIN/TRIM`、`RC10_MIN/TRIM` | 各自的上游值 | 差 1–7 µs | **遥控器校准**（行程/微调），设备相关 |

> ☠️ **2026-09-22 更正本表的两个分类计数**（写这张时数错了，**总数 21 与全部结论不受影响**）：
> 「飞行计数器（**4**）」实为 **3** 条（`COM_FLIGHT_UUID`、`LND_FLIGHT_T_HI`、`LND_FLIGHT_T_LO`）；
> 「遥控器校准（**14**）」实为 **12** 条（`RC1_MIN` + `RC2/3/4_MAX` + `RC6/7/9/10` 各 `MIN`+`TRIM`）。
> 6 + 12 + 3 = **21** ✓ —— 原来的 6+4+14 = 24 与上一张表的合计对不上。
> 判据：**以本表自己列出的参数名逐个数为准**（`params-analysis.md` §5 已同步为 6+12+3）。

### 逐条核实了文档里声称的每一条（**全部一致**）

`phase1-implementation.md` §6 与 `PROJECT.md` §参数 里声称的 30 余条，逐条对着真身读：

| 声称 | 真身 | 声称 | 真身 |
|---|---|---|---|
| `EKF2_EV_CTRL = 11` | 11 ✅ | `COM_RC_OVERRIDE = 3` | 3 ✅ |
| `EKF2_HGT_REF = 3` | 3 ✅ | `COM_OBL_RC_ACT = 0` | 0 ✅ |
| `EKF2_MAG_TYPE = 5` | 5 ✅ | `COM_OF_LOSS_T = 1.0` | 1.0 ✅ |
| `EKF2_EV_DELAY = 50.0` | 50.0 ✅ | `COM_RC_IN_MODE = 3` | 3 ✅ |
| `EKF2_EVP_NOISE = 0.061` | 0.06095999851822853 ✅ | `RC_MAP_FLTMODE = 6` | 6 ✅ |
| `EKF2_EV_POS_X/Y/Z = 0` | 0/0/0 ✅ | `MPC_XY_VEL_MAX = 12.0` | 12.0 ✅ |
| `EKF2_GPS_CTRL = 7` | 7 ✅ | `MPC_THR_HOVER = 0.50` | 0.5 ✅ |
| `COM_ARM_WO_GPS = 1` | 1 ✅ | `MC_ROLLRATE_MAX/PITCHRATE_MAX = 220` | 220/220 ✅ |
| `SYS_AUTOSTART = 4001` | 4001 ✅ | `BAT1_R_INTERNAL = 0.005` | 0.004999999888 ✅ |
| `MAV_1_CONFIG/MODE = 102/2` | 102/2 ✅ | `THR_MDL_FAC = 0.45` | 0.449999988 ✅ |
| `SER_TEL1_BAUD`/`SER_TEL2_BAUD = 921600` | 921600 ✅ | `BAT1_N_CELLS/MOT_POLE_COUNT = 6/14` | 6/14 ✅ |
| `PWM_MAIN_FUNC1..4 = 103/102/101/104` | 103/102/101/104 ✅ | `UXRCE_DDS_CFG = 0` | 0 ✅ |

> ★ **浮点别用精确相等比**（我自己的脚本在这里报了两条假不一致：`0.45` vs `0.449999988`、
> `0.061` vs `0.060959998518`）—— 飞控参数是 **float32**，导出文本是十进制的长尾。

★ **顺带把 [ticket 02](02-rc-channel-decision.md) 的两个前提在真身上确认了**：

| 参数 | 真身值 | 对 ticket 02 的意义 |
|---|---|---|
| `RC_MAP_KILL_SW` | **`8`** | ✅ **ch8 = 硬件杀车**（急停走它，不新增软通道） |
| `RC_MAP_FLTMODE` | **`6`** | ✅ ch6 = 模式开关 |
| `RC_MAP_OFFB_SW` / `POSCTL_SW` / `LOITER_SW` | 全 `0` | ✅ **ch9 不在任何 `RC_MAP_*` 里** ⟹ "开始 = ch9 上升沿"不冲突 |
| `COM_FLTMODE1 / 4 / 6` | **`2` / `1` / `7`** | ✅ 与 `params-analysis.md` §3.15(b) 记录一致 ⟹ 档位 1 = **Position**（"遥控器 Position 模式悬停"这一步**可达**）、档位 6 = **Offboard** |

## 三、这条对账改变了什么（★ 读法要反过来）

**原判断**（`PROJECT.md`）："`z0.params` 内容是上游的，**尚未在 z0 的飞控上核实**"
—— 言下之意是"表里的可疑值**可能只是文件过时**"。

**对账后**：**914/915 条与真身一致** ⟹ **飞控上跑的就是这份配置**。
所以文档里那些"⚠️ 疑为 PX4 默认值 / 与 152 mm 机架不符 / 差 15 倍"的怀疑，
**不是文件问题，是飞机上真实存在的待改项**：

| 待处理项 | 真身值 | 为什么是问题（文档已有分析） |
|---|---|---|
| `CA_ROTOR0..3_PX/PY = ±0.15` | ±0.15000000596 | 算得臂长 212 mm、轴距 **424 mm**，与 152 mm 机架不符（疑模板默认值） |
| `BAT1_R_INTERNAL = 0.005` | 0.0049999999 | VTC6 6S1P 应约 **78 mΩ**，差 15 倍 |
| `EKF2_EV_POS_X/Y/Z = 0` | 0/0/0 | 动捕刚体 → IMU 外参**未标定**（z0 用铝柱架球，Z 向几乎必然非 0） |
| `EKF2_GPS_CTRL = 7` | 7 | 室内动捕飞行通常应设 **0** |
| `MPC_THR_HOVER = 0.5` | 0.5 | 疑 PX4 默认值，非实测；**直接决定高度控制好坏** |
| `THR_MDL_FAC = 0.45` | 0.449999988 | 与 XI35 的 `K3 = 0.6786` 不同，★ 两者至少一个错 |
| `MPC_XY_VEL_MAX = 12.0` | 12.0 | 对 152 mm 涵道机是失控速度；Phase 1 建议 1.0–2.0 |
| `EKF2_EVP_NOISE = 0.061` | 0.0609599985 | 用户笔记："1.0 可用、0.1 过小（发散）"；**0.061 比 0.1 还小** |

### ☠️ 一条必须写下来的操作警告：**不要整份烧 `z0.params`**

对账的结果正好说明：那份文件**含 6 条加速度计标定 + 12 条遥控器校准**，
而**飞控上现有的这两组值才是 z0 自己的**。整份 `load` 下去会：
**① 覆盖 z0 的加速度计标定**（参数表与真实 IMU 不匹配）、**② 覆盖遥控器行程校准**。
⟹ 若要上参数，**只挑配置类参数写**（`EKF2_*` / `MPC_*` / `CA_ROTOR*` / `BAT1_*`…），
**绝不整份烧**。（`PROJECT.md` 里"准备烧进 z0"的说法应按本条更正 —— 已在文档侧改。）

## 四、验收项

- [x] 从飞控导出全量参数（`ros2 param dump /mavros/param`，915 条）
- [x] 与 [`docs/assets/z0.params`](../../../docs/assets/z0.params) 逐条 diff，差异分类
- [x] 逐条核实 `phase1-implementation.md` §6 与 `PROJECT.md` §参数 声称的每一条
- [x] ★ **人裁定（2026-09-22，用户拍板）**：8 条里**现在就改 3 条**（`EKF2_GPS_CTRL = 0` + `SYS_HAS_GPS = 0`、
      `MPC_XY_VEL_MAX = 2.0`），其余 5 条分别留到 **系统辨识 / 推力标定 / 尺子量 / 飞行阶段**。
      逐条处置（含"谁改、怎么验"）见 [`params-changes.md`](../../../docs/reference/params-changes.md) **§I**
- [x] 若决定改：**只写配置类参数**，改完**再 dump 一次对账**，回填本张
      —— ✅ **2026-09-23 已写入**：`EKF2_GPS_CTRL` `7→0`、`SYS_HAS_GPS` `1→0`、`MPC_XY_VEL_MAX` `12.0→2.0`
      （三条都是 `Set parameter successful` + 读回确认；`ros2 param dump` 918 行对账通过）。见 Comments。
      ★ **顺带更正本条括注**：写参数**不必**用 `ParamSetV2` ——
      `ros2 param set /mavros/param <名> <值>` 就会转发给飞控（`param.cpp:1148`），见 **§I.2**；
      ⚠️ 但**必须先 `~/pull`**，否则报 `Undeclared parameter`（2026-09-23 实测，`params-changes.md` §I.1 的 ★）
- [ ] `EKF2_EV_POS_X/Y/Z` 外参：量出刚体→IMU 的实际偏移（`hardware.md` §6 的"尺子量"）后写入
- [ ] `MPC_THR_HOVER` 标定（`thrust_calibrate` 流程）与 `THR_MDL_FAC` 的取舍（两者至少一个错）
- [x] 把本张的结论回填 —— ★ **实际范围比本条写的更大**（原文只点了两处，**漏了 `params-analysis.md`**）：
      [`PROJECT.md`](../../../docs/PROJECT.md) §5.1、[`phase1-implementation.md`](../../../docs/control/phase1-implementation.md) §6、
      **`params-analysis.md`**（页首 ★ / §3.7 / §5 / §6 待办清单）、
      **`params-changes.md`**（页首警告 / F2 / §G / 新增 §I）、[`bench-runbook.md`](../../../docs/control/bench-runbook.md) P0

## 五、坑

1. ⚠️ **`/mavros/param/get` 不存在**（ROS2 版 mavros 把 `ParamGet` 注释掉了）—— 见 §一。
2. ⚠️ **float32 别用精确相等比**（见 §二 的 ★）。
3. ⚠️ **`ros2 param dump` 要 45 s**（915 条），别以为卡死了；期间别并发跑别的重活（8 GB 内存）。
4. ⚠️ **参数是飞控上的持久状态**，写下去就留在飞机上 —— 与"改代码"不同，**每次写都要有记录、能回退**
   （先 `ros2 param get` 存旧值再写）。

## Comments

### 2026-09-21 —— 对账完成（agent，在 z0 的 Jetson 上）

- 触发：飞控上电 + mavros 连通（[ticket 14](14-mavros-from-source.md)）后，
  [ticket 13](13-mavros-bench-vision-pose.md) 的"顺带记录 `EKF2_EVP_NOISE`"顺手扩成了全量对账。
- 结论一句话：**`z0.params` 就是飞控真身**（914/915 一致），差异 21 条**全是标定/计数器**；
  因此文档里所有"⚠️ 可疑值"都是**飞机上真实存在的待改项**，且**整份烧参数是错的**（会覆盖标定）。
- 原始导出与 diff 脚本落在 `/tmp`（`/tmp/fcu_params.yaml`、`/tmp/diff_params.py`），**不入库**；
  本文档 §二 已把结论与 21 条差异逐条记全。

### 2026-09-22 —— ★ 人裁定 + 文档回填补齐（agent）

**触发**：用户 `/implement #15`。

**① ★ 人裁定（用户拍板）** —— 就是 §四 第一个框，8 条待处理项的处置：

| # | 待处理项 | 裁定 | 谁来改 | 怎么验 |
|---|---|---|---|---|
| 1 | `EKF2_GPS_CTRL` + `SYS_HAS_GPS` | ★ **现在就改** `0` / `0` | agent（mavros） | re-dump 对账 |
| 2 | `MPC_XY_VEL_MAX` | ★ **现在就改** `2.0` | agent（mavros） | re-dump 对账 + 飞时看轨迹 2/3 |
| 3 | `CA_ROTOR0..3_PX/PY` | **不改**（分配器按几何范数归一化，均匀缩放被约掉；要它的是 (D) 方案） | — | — |
| 4 | `BAT1_R_INTERNAL` | 留到**系统辨识**（里程碑 2） | 人 | 只影响 sag 补偿 |
| 5 | `MPC_THR_HOVER` | 留到**推力标定**（里程碑 2） | 人 | `thrust_calibrate`，且要同时回填 **MPC（模型预测控制）** 模型 |
| 6 | `THR_MDL_FAC` | 留到**推力标定**（与 5 一并） | 人 | 与 XI35 的 `K3=0.6786` 至少一个错 |
| 7 | `EKF2_EV_POS_X/Y/Z` | 留到"**尺子量**" | 人 | 量刚体原点→IMU 偏移后写入 |
| 8 | `EKF2_EVP_NOISE` | 留到**飞行阶段**判定 | 人 | 台架 30 min 未发散 ⟹ 台架证不了它过小 |

⟹ 落纸在 [`params-changes.md` §I](../../../docs/reference/params-changes.md)。

**② ⏸ 第 1/2 条没写进去 —— 因为飞控没上电**：`mavlink_probe.py --port /dev/ttyTHS1 --baud 921600`
**无 HEARTBEAT**，mavros 也没在跑。所以状态是「**已裁定、待写入**」，不是"已改"。
命令备在 `params-changes.md` **§I.1**；`bench-runbook.md` P0 已加指针 —— **下次上电就是补写的机会**。

**③ 文档回填原先只做了一半，这次补齐**：§四 最后一条当时点的是 `PROJECT.md`（已做）与
`phase1-implementation.md` §6（已做），但**本张「被它影响的」那行还点名了 `params-analysis.md`** —— **那处漏了**。
本次补齐 5 份文档：

- `params-analysis.md`：页首加对账 ★；§5「与你自己飞控的参数做 diff」标为**已执行**
  （实际走 `ros2 param dump`，**不是** QGC 导出）；§6 待办清单加「2026-09 状态」列；§3.7 加裁定行
- `params-changes.md`：页首「所有『现值』都还没有在 z0 自己的飞控上核对过」**作废**；
  F2「`z0.params` 是另一台参考机导出的」**结案**；§G 第 1 项标已做；**新增 §I**（裁定表 + 执行清单 + §I.2）
- `bench-runbook.md` P0：加 3 条待写入参数的指针（★ 标注它们**不是**台架判据，只是共用这次上电）
- `phase1-implementation.md` §6：`MPC_XY_VEL_MAX` / `MPC_THR_HOVER` / `EKF2_EVP_NOISE` 三行加裁定注

**④ 本轮顺带查出的三处（都在上面落纸里更正了）**：

1. ☠️ **本张 §二 的两个分类计数写错了**：飞行计数器写 **4**（实 **3**）、遥控器校准写 **14**（实 **12**）——
   6+4+14 = 24 与本张自己给的合计 **21** 对不上。**更正为 6+12+3 = 21**（**总数与全部结论不变**）。
2. ★ **本张 §一「只有 `~/pull` 与 `~/set`」这一句不完整**（第一手重读 `mavros/src/plugins/param.cpp`）：
   还有 **`~/get_parameters`**（`:468`）与 **`~/set_parameters`**（`:476`）——
   后者正是 `ros2 param set` 走的路径，`set_parameters_cb` 里确实调 `send_param_set_and_wait()` 发给飞控（`:1148`）；
   而 `~/set_parameters_atomically` 是**硬编码不支持**（`:1157-1164`）。
   排除名单（`:357-369`）里**没有我们的 3 条** ⟹ §四「用 `ParamSetV2`」那句括注**可以放宽**：`ros2 param set` 就够。
   详见 `params-changes.md` **§I.2**。
3. ⚠️ **`params-analysis.md` 里 6 处引的固件真身 `Peize-Liu/PX4-Autopilot @ 5ba2e91f` 是已被推翻的那一版**
   （其中 `:270` 还明写"**z0 实际在跑的**"）—— 按 `params-changes.md` **F3**，真身应是 `4a0e65f2`。
   **不属本张范围，没有动**（本张只改参数结论，不改固件身份）—— 归
   [ticket 10](10-docs-1-14-sweep.md) 的全库清扫。**在此点名，免得它随本张的"回填完成"被静默盖过去。**

### 2026-09-23 —— ★ 3 条参数**已写入**（第 5 框勾掉）+ 顺带查出 `COM_RC_IN_MODE` 那条硬伤（agent）

**① 第 5 框（写参数）结清** —— 借着本次上电补写（`bench-runbook.md` P0 说的"这次上电就是机会"）：

| 参数 | 原值 | 写入 | 读回 | dump 对账 |
|---|---|---|---|---|
| `EKF2_GPS_CTRL` | `7` | `0` | `0` ✅ | ✅ |
| `SYS_HAS_GPS` | `1` | `0` | `0` ✅ | ✅ |
| `MPC_XY_VEL_MAX` | `12.0` | `2.0` | `2.0` ✅ | ✅ |

`ros2 param dump /mavros/param` → **918 行**；顺带复核到 **`NAV_RCL_ACT = 1`**（A1 首飞阻塞项
**已落地**）与 **`COM_RC_IN_MODE = 0`**（见下）。
⚠️ **新增一条前置**：`set` 之前必须先 `~/pull`，否则报 `Undeclared parameter`（实测）——
已补进 `params-changes.md` **§I.1** 的 ★。

**② 顺带查出的硬伤：`COM_RC_IN_MODE = 3` 会让整个遥控开关层静默失效** ——
完整第一手链条与两侧互证的复现证据在 `params-changes.md` **§I.3**。
**它推翻了本张 §二 引用的两处处置**（`params-changes.md` §D4 与 `params-analysis.md` §3.9 都判"不是警报"）
—— 已在两处落更正；那两处对**枚举含义**的核实仍然成立，推翻的是处置。
★ 直接后果：**ticket [02](02-rc-channel-decision.md) 的裁定（ch9 = 开始键）在 `3` 之下根本走不到**
——`ManualControl::processSwitches()` 在选中源不是 RC 时**压根不评估开关** ⟹
§H.8 的台架核对若在那时做，得到的是"拨了没反应"的**假结论**（当时的实测症状正是如此）。

**③ 与真身口径的对齐**：本次引用的固件位置**全部按真身 `4a0e65f2` 复核** ——
`ManualControlSelector.cpp:59-62 / :73 / :82-84`、`ManualControl.cpp:172 / :277 / :281`、
`rc_update.cpp:673-695 / :677`、`mavlink_receiver.cpp:2056-2058`。
★ 顺带**更正一条此前的读法**：真身的 `UpdateManualControlInput`（`rc_update.cpp:673-695`）
**从不给 `.valid` 赋值**，RC 输入的准入靠 `ManualControlSelector::isInputValid`（`:66-88`）的
**时间戳新鲜度 + 源匹配** —— fork 版那句 `valid = _rc_calibrated` **不适用**。

**④ 仍未动的框**（都是纯人的事，故本张维持 `ready-for-human`）：
`EKF2_EV_POS_X/Y/Z` 的**尺子量**、`MPC_THR_HOVER` / `THR_MDL_FAC` 的**推力台标定**。
