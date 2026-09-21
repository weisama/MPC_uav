# 15: ★ `z0.params` ↔ 飞控真身**逐条对账** —— 结论：文件就是真身；21 条差异全是标定/计数器

Status: ready-for-human
Type: task

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

对账的结果正好说明：那份文件**含 6 条加速度计标定 + 14 条遥控器校准**，
而**飞控上现有的这两组值才是 z0 自己的**。整份 `load` 下去会：
**① 覆盖 z0 的加速度计标定**（参数表与真实 IMU 不匹配）、**② 覆盖遥控器行程校准**。
⟹ 若要上参数，**只挑配置类参数写**（`EKF2_*` / `MPC_*` / `CA_ROTOR*` / `BAT1_*`…），
**绝不整份烧**。（`PROJECT.md` 里"准备烧进 z0"的说法应按本条更正 —— 已在文档侧改。）

## 四、验收项

- [x] 从飞控导出全量参数（`ros2 param dump /mavros/param`，915 条）
- [x] 与 [`docs/assets/z0.params`](../../../docs/assets/z0.params) 逐条 diff，差异分类
- [x] 逐条核实 `phase1-implementation.md` §6 与 `PROJECT.md` §参数 声称的每一条
- [ ] ★ **人裁定**：上表 8 条待处理项哪些现在就改、哪些留到标定/飞行阶段（改哪条 ⇒ 谁来改 ⇒ 怎么验）
- [ ] 若决定改：**只写配置类参数**（用 `/mavros/param/set`，类型 `ParamSetV2`），改完**再 dump 一次对账**，回填本张
- [ ] `EKF2_EV_POS_X/Y/Z` 外参：量出刚体→IMU 的实际偏移（`hardware.md` §6 的"尺子量"）后写入
- [ ] `MPC_THR_HOVER` 标定（`thrust_calibrate` 流程）与 `THR_MDL_FAC` 的取舍（两者至少一个错）
- [ ] 把本张的结论回填 [`PROJECT.md`](../../../docs/PROJECT.md)（"尚未核实"的那段）与
      [`phase1-implementation.md`](../../../docs/control/phase1-implementation.md) §6

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
