# 资料索引（Phase 5 票 19）

> 生成日期：2026-10-18。索引只回答「这份文件是什么、被哪一章引用」，不重复各章结论。
> 引用列由 `grep` 对 `docs/report/chapters/**` 与 `docs/report/generated/**` 现查；`generated/` 的图注与指标服务于第 10 章，故并入章号。
> 范围：`docs/assets/papers/`、`docs/assets/drill/`、`tools/*/evidence/`、`bag/`、参数导出。`docs/LOCAL-NOTES.md` 不在范围内；该文件含本机凭据，报告不引用。

## 论文

| 路径 | 是什么 | 日期/版本 | 被哪一章引用 | 备注 |
|---|---|---|---|---|
| `docs/assets/papers/Sun2022_TRO_NMPC_vs_DFBC.pdf` | 论文：A Comparative Study of Nonlinear MPC and Differential-Flatness-based Control for Quadrotor Agile Flight（非线性模型预测控制与微分平坦控制用于四旋翼敏捷飞行的对比研究） | 2022（IEEE T-RO） | 第4、8章 | 第4章按 PDF 首页复核标题；第8章引 Sun2022 §III.A |
| `docs/assets/papers/Foehn2022_SciRob_agilicious.pdf` | 论文：Agilicious: Open-Source and Open-Hardware Agile Quadrotor for Vision-Based Flight（Agilicious：面向视觉飞行的开源开放硬件敏捷四旋翼） | 2022（Science Robotics 7(67)） | 第4、8章 | 第8章引 Foehn2022 §D.1 |
| `docs/assets/papers/Torrente2021_RAL_DataDrivenMPC.pdf` | 论文：Data-Driven MPC for Quadrotors（面向四旋翼的数据驱动模型预测控制） | 2021（IEEE RA-L） | 第4章 | 高斯过程补偿气动效应 |
| `docs/assets/papers/Houska2011_Automatica_RTI.pdf` | 论文：An auto-generated real-time iteration algorithm for nonlinear MPC in the microsecond range（一种把非线性模型预测控制压到微秒级的自动生成实时迭代算法） | 2011（Automatica 47） | 第4、8章 | 第8章作为实时迭代的理论出处 |
| `docs/assets/papers/Houska2011_OCAM_ACADOToolkit.pdf` | 论文：ACADO toolkit — An open-source framework for automatic control and dynamic optimization（ACADO Toolkit：用于自动控制与动态优化的开源框架） | 2011（OCAM 32） | 第4章 | acados 的前身 |
| `docs/assets/papers/Liu2024_IROS_OmniNxt.pdf` | 论文：OmniNxt: A Fully Open-source and Compact Aerial Robot with Omnidirectional Visual Perception（OmniNxt：全开源、紧凑、带全向视觉感知的空中机器人） | 2024（IEEE/RSJ IROS） | 第4章 | 硬件平台参考（Nxt-FC + Orin NX） |
| `docs/assets/papers/README.md` | 6 篇论文的中文标注、原题、作者、出处与速查表 | 未标注 | 第4章 | 命名规则与阅读优先级见文内 |

## 演练日志

| 路径 | 是什么 | 日期/版本 | 被哪一章引用 | 备注 |
|---|---|---|---|---|
| `docs/assets/drill/2026-10-12-arm-denied-offboard.log` | 解锁被拒的只读探针原始输出（PX4 statustext、/mavros/state、遥控等 5 路订阅）；用于定位 ch5 高位未解锁的原因 | 2026-10-12 | 未引用 | 目录整体见第7章；本文件未被正文点名 |
| `docs/assets/drill/2026-10-12-ch9-takeoff-battery-low.log` | 从 run/logs 摘出的节点原文：ch9 起飞成功与被电池低压打断 | 2026-10-12 | 未引用 | 目录整体见第7章；原始 run/logs 不入库 |
| `docs/assets/drill/2026-10-12-chain-up-clean.log` | run/z0.py check 的输出：清掉孤儿进程后 (A) 级链的就绪读数 | 2026-10-12 | 第7章 | 第7章回退 (A) 级一节引用 |
| `docs/assets/drill/2026-10-12-drill21-probe-run9.log` | §2.1 合并版探针 run9（窗口 1200 s）的原始逐拍输出；判打杆接管 rc_override | 2026-10-12 | 第7章 | 表 7-10 引用为 drill21-probe-run9.log |
| `docs/assets/drill/2026-10-12-drill22-ch8-killswitch-probe-run13.log` | §2.1 合并版探针 run13（窗口 900 s）；判 ch8 杀车开关的飞控行为 | 2026-10-12 | 第7章 | 表 7-10 引用 |
| `docs/assets/drill/2026-10-12-drill23-sigint-control.log` | SIGINT 场次的 control 节点与 launch 原始日志 | 2026-10-12 | 未引用 | 与 drill23-sigint-probe-run12.log 同一场；正文只点名后者 |
| `docs/assets/drill/2026-10-12-drill23-sigint-probe-run12.log` | §2.1 合并版探针 run12（窗口 700 s）；判 offboard 心跳丢失后的 Failsafe 与自动降落 | 2026-10-12 | 第7章 | 表 7-10 引用 |
| `docs/assets/drill/2026-10-12-drill24-probe-run10.log` | §2.1 合并版探针 run10（窗口 1800 s）；判节点被 kill -9 后的行为 | 2026-10-12 | 第7章 | 表 7-10 引用 |
| `docs/assets/drill/2026-10-12-node-logs-arming-and-failsafe.log` | 从 run/logs 摘出的解锁与失效关键字原文（Arming denied、Armed by、Failsafe activated 等） | 2026-10-12 | 未引用 | 目录整体见第7章；本文件未被正文点名 |
| `docs/assets/drill/2026-10-12-rc-channels.log` | 遥控 16 通道的实测采样（基线、ch9 与 ch6 动作），用于确认通道映射 | 2026-10-12 | 未引用 | 第7章引用了同日通道读数，但未点名本文件 |
| `docs/assets/drill/2026-10-12-takeover-modelost-entry.log` | 拨 ch6 把模式从 OFFBOARD 切走的进入时刻原始输出；判 degrade=mode_lost 与停发 | 2026-10-12 | 第7章 | 表 7-10 引用 |
| `docs/assets/drill/2026-10-12-takeover-modelost.log` | 同一场夺权之后的闩锁半程（180 s）；判闩锁不自恢复 | 2026-10-12 | 未引用 | 进入时刻看 takeover-modelost-entry.log |

## 工具证据：battery-thrust

| 路径 | 是什么 | 日期/版本 | 被哪一章引用 | 备注 |
|---|---|---|---|---|
| `tools/battery-thrust/evidence/README.md` | 工具输出格式、复算命令、三条结论与取不到项的说明 | 2026-10-09 | 第3章 | 第3章链到本文件 |
| `tools/battery-thrust/evidence/summarise.py` | 汇总脚本：读本目录 JSON 现算汇总表，并核对 MANIFEST 的 ulog sha256 | 2026-10-09 | 第3章 | 第3章复算命令引用 |
| `tools/battery-thrust/evidence/summary_2026-10-09.txt` | 8 份 ulog 的汇总表 + 低电量端投影 + 指纹核对 | 2026-10-09 | 未引用 | 由 summarise.py 现算 |
| `tools/battery-thrust/evidence/10712c0f__1_static.json` | battery_from_ulog.py 对 bag/1_static/fcu/log_0_2026-9-29-15-22-20.ulg 的完整输出：可持续电流/功率、判据自检、missing 清单、sha256 | 2026-10-09 | 未引用 | 目录整体见第3章 |
| `tools/battery-thrust/evidence/10712c0f__1_static.txt` | 同一份输出的工具 stdout 人读版（对应 bag/1_static/fcu/log_0_2026-9-29-15-22-20.ulg） | 2026-10-09 | 未引用 | 目录整体见第3章 |
| `tools/battery-thrust/evidence/1f8e76cd__2_manual_takeoff.json` | battery_from_ulog.py 对 bag/2_manual_takeoff/fcu/log_1_2026-9-29-15-25-14.ulg 的完整输出：可持续电流/功率、判据自检、missing 清单、sha256 | 2026-10-09 | 未引用 | 目录整体见第3章 |
| `tools/battery-thrust/evidence/1f8e76cd__2_manual_takeoff.txt` | 同一份输出的工具 stdout 人读版（对应 bag/2_manual_takeoff/fcu/log_1_2026-9-29-15-25-14.ulg） | 2026-10-09 | 未引用 | 目录整体见第3章 |
| `tools/battery-thrust/evidence/3bfd9027__3_traj1_hover.json` | battery_from_ulog.py 对 bag/3_traj1_hover/fcu/log_0_未知日期.ulg 的完整输出：可持续电流/功率、判据自检、missing 清单、sha256 | 2026-10-09 | 未引用 | 目录整体见第3章 |
| `tools/battery-thrust/evidence/3bfd9027__3_traj1_hover.txt` | 同一份输出的工具 stdout 人读版（对应 bag/3_traj1_hover/fcu/log_0_未知日期.ulg） | 2026-10-09 | 未引用 | 目录整体见第3章 |
| `tools/battery-thrust/evidence/bb2ff47c__4_traj2_square.json` | battery_from_ulog.py 对 bag/4_traj2_square/fcu/log_1_未知日期.ulg 的完整输出：可持续电流/功率、判据自检、missing 清单、sha256 | 2026-10-09 | 未引用 | 目录整体见第3章 |
| `tools/battery-thrust/evidence/bb2ff47c__4_traj2_square.txt` | 同一份输出的工具 stdout 人读版（对应 bag/4_traj2_square/fcu/log_1_未知日期.ulg） | 2026-10-09 | 未引用 | 目录整体见第3章 |
| `tools/battery-thrust/evidence/83de1da5__5_traj3_lemniscate.json` | battery_from_ulog.py 对 bag/5_traj3_lemniscate/fcu/log_4_未知日期.ulg 的完整输出：可持续电流/功率、判据自检、missing 清单、sha256 | 2026-10-09 | 未引用 | 目录整体见第3章 |
| `tools/battery-thrust/evidence/83de1da5__5_traj3_lemniscate.txt` | 同一份输出的工具 stdout 人读版（对应 bag/5_traj3_lemniscate/fcu/log_4_未知日期.ulg） | 2026-10-09 | 未引用 | 目录整体见第3章 |
| `tools/battery-thrust/evidence/c9153f43__old-5_traj3_lemniscate.json` | battery_from_ulog.py 对 bag/old/5_traj3_lemniscate/fcu/log_2_未知日期.ulg 的完整输出：可持续电流/功率、判据自检、missing 清单、sha256 | 2026-10-09 | 未引用 | 目录整体见第3章 |
| `tools/battery-thrust/evidence/c9153f43__old-5_traj3_lemniscate.txt` | 同一份输出的工具 stdout 人读版（对应 bag/old/5_traj3_lemniscate/fcu/log_2_未知日期.ulg） | 2026-10-09 | 未引用 | 目录整体见第3章 |
| `tools/battery-thrust/evidence/ead72549__old-4_traj3_lemniscate.json` | battery_from_ulog.py 对 bag/old/4_traj3_lemniscate/fcu/log_3_2026-9-29-15-32-40.ulg 的完整输出：可持续电流/功率、判据自检、missing 清单、sha256 | 2026-10-09 | 未引用 | 目录整体见第3章 |
| `tools/battery-thrust/evidence/ead72549__old-4_traj3_lemniscate.txt` | 同一份输出的工具 stdout 人读版（对应 bag/old/4_traj3_lemniscate/fcu/log_3_2026-9-29-15-32-40.ulg） | 2026-10-09 | 未引用 | 目录整体见第3章 |
| `tools/battery-thrust/evidence/fb1a1751__old-3_traj1_hover.json` | battery_from_ulog.py 对 bag/old/3_traj1_hover/fcu/log_2_2026-9-29-15-29-26.ulg 的完整输出：可持续电流/功率、判据自检、missing 清单、sha256 | 2026-10-09 | 未引用 | 目录整体见第3章 |
| `tools/battery-thrust/evidence/fb1a1751__old-3_traj1_hover.txt` | 同一份输出的工具 stdout 人读版（对应 bag/old/3_traj1_hover/fcu/log_2_2026-9-29-15-29-26.ulg） | 2026-10-09 | 未引用 | 目录整体见第3章 |
| `tools/battery-thrust/evidence/__pycache__/summarise.cpython-310.pyc` | summarise.py 的 Python 字节码缓存 | 2026-10-09 | 未引用 | 生成物，不是证据 |

## 工具证据：hover-thrust

| 路径 | 是什么 | 日期/版本 | 被哪一章引用 | 备注 |
|---|---|---|---|---|
| `tools/hover-thrust/evidence/README.md` | hover_thrust 反推证据的说明：SITL 同构机真值对、175 份日志复现、判据敏感性、不能替代什么 | 2026-09-29 | 第3章 | 第3章链到本文件 |
| `tools/hover-thrust/evidence/sitl_log_10_42_47_2026-09-28.json` | 单份 SITL ulog 的工具完整输出（判据、候选段、读数、HTE、sha256）；与解析真值的相对偏差 2.6e-8 | 2026-09-28 | 未引用 | 目录整体见第3章 |
| `tools/hover-thrust/evidence/sitl_log_10_42_47_2026-09-28.txt` | 同一份输出的工具 stdout 人读版 | 2026-09-28 | 未引用 | 目录整体见第3章 |
| `tools/hover-thrust/evidence/sitl_corpus_175_logs_2026-09-28.txt` | 175 份 SITL 日志的批跑原始输出，含 4 档速度门的敏感性表 | 2026-09-28 | 未引用 | 目录整体见第3章 |
| `tools/hover-thrust/evidence/thrust_curve_2026-10-09.txt` | THR_MDL_FAC 反解工具的原始输出（thrust_curve_check.py） | 2026-10-09 | 未引用 | 目录整体见第3章 |

## 工具证据：tau-analysis

| 路径 | 是什么 | 日期/版本 | 被哪一章引用 | 备注 |
|---|---|---|---|---|
| `tools/tau-analysis/evidence/README.md` | τ 实测证据的说明：文件清单、三个口径、2026-10-17 三趟交叉读数与限制 | 2026-10-17 | 未引用 | 目录整体见第3章 |
| `tools/tau-analysis/evidence/fcu-log-list-2026-09-28.txt` | 飞控日志列表节选（LOG_REQUEST_LIST 原始输出），支持 log 210 的 id/utc/size | 2026-09-28 | 未引用 | 目录整体见第3章 |
| `tools/tau-analysis/evidence/fcu-log-pull-2026-09-28.txt` | 三次拉取日志的原始输出节选（停在约 18.6 MB / 37.9%），说明下载未完成 | 2026-09-28 | 未引用 | 目录整体见第3章 |
| `tools/tau-analysis/evidence/tau_z0_log210_fc2.0_seg3.json` | 飞控日志 210（2026-09-24）的 τ 完整输出：三个口径、分段、参数与 sha256 | 2026-09-28 | 未引用 | 目录整体见第3章 |
| `tools/tau-analysis/evidence/tau_z0_log212_hover_auto_fc2.0_seg3.json` | 悬停 log212、自动窗口、3 段的 τ 读数 | 2026-10-17 | 未引用 | 目录整体见第3章 |
| `tools/tau-analysis/evidence/tau_z0_log212_hover_maneuver_fc2.0_seg3.json` | 悬停 log212、手工末段机动窗口的 τ 读数 | 2026-10-17 | 未引用 | 目录整体见第3章 |
| `tools/tau-analysis/evidence/tau_z0_log213_square_auto_fc2.0_seg3.json` | 正方形 log213、自动窗口、3 段的 τ 读数 | 2026-10-17 | 未引用 | 目录整体见第3章 |
| `tools/tau-analysis/evidence/tau_z0_log213_square_maneuver_fc2.0_seg3.json` | 正方形 log213、手工窗口的 τ 读数 | 2026-10-17 | 未引用 | 目录整体见第3章 |
| `tools/tau-analysis/evidence/tau_z0_log213_takeoff_maneuver_fc2.0_seg3.json` | 正方形 log213、起飞段手工窗口的 τ 读数（VO 最大间隔 304 ms，不进结论） | 2026-10-17 | 未引用 | 目录整体见第3章 |
| `tools/tau-analysis/evidence/tau_z0_log214_lemniscate_auto_fc2.0_seg3.json` | 8 字 log214、自动窗口、3 段的 τ 读数 | 2026-10-17 | 未引用 | 目录整体见第3章 |
| `tools/tau-analysis/evidence/tau_z0_log214_lemniscate_maneuver_fc2.0_seg3.json` | 8 字 log214、手工末段机动窗口的 τ 读数 | 2026-10-17 | 未引用 | 目录整体见第3章 |

## 工具证据：ladder-baseline

| 路径 | 是什么 | 日期/版本 | 被哪一章引用 | 备注 |
|---|---|---|---|---|
| `tools/ladder-baseline/evidence/A-3_traj1_hover.json` | 口径 a（offboard 位置设定点）的跟踪误差读数；输入 bag/3_traj1_hover | 2026-10-17 | 未引用 | 工具为 ladder_from_bag.py |
| `tools/ladder-baseline/evidence/A-4_traj2_square.json` | 口径 a 的跟踪误差读数；输入 bag/4_traj2_square | 2026-10-17 | 未引用 | 工具为 ladder_from_bag.py |
| `tools/ladder-baseline/evidence/A-5_traj3_lemniscate.json` | 口径 a 的跟踪误差读数；输入 bag/5_traj3_lemniscate | 2026-10-17 | 未引用 | 工具为 ladder_from_bag.py |
| `tools/ladder-baseline/evidence/A-old_3_traj1_hover.json` | 口径 a 的跟踪误差读数；输入 bag/old/3_traj1_hover | 2026-10-17 | 未引用 | 历史包 |
| `tools/ladder-baseline/evidence/A-old_4_traj3_lemniscate.json` | 口径 a 的跟踪误差读数；输入 bag/old/4_traj3_lemniscate | 2026-10-17 | 未引用 | 历史包 |
| `tools/ladder-baseline/evidence/C-anchored-6_free__20261017-020757.json` | 口径 t-anchored 的跟踪误差读数；输入 bag/6_free__20261017-020757 | 2026-10-17 | 未引用 | 锚点按 node_core 规则重建 |
| `tools/ladder-baseline/evidence/C-anchored-6_free__20261017-020757-last30.json` | 同一趟的末 30 s 稳态窗口读数（阶梯 3 用） | 2026-10-17 | 未引用 | 窗口切法在 JSON 内 |
| `tools/ladder-baseline/evidence/C-anchored-6_free__20261017-070103-WRONG-CALIBER.json` | 故意用错口径的对照输出；证明口径选错会让读数变化而脚本不报错 | 2026-10-17 | 未引用 | 保留作反例，不是结论 |
| `tools/ladder-baseline/evidence/C-target-6_free__20261017-070103.json` | 口径 c-target 的跟踪误差读数；输入 bag/6_free__20261017-070103 | 2026-10-17 | 未引用 | 该趟启动行为 参考=/control/target |
| `tools/ladder-baseline/evidence/C-target-6_free__20261017-070446.json` | 口径 c-target 的跟踪误差读数；输入 bag/6_free__20261017-070446 | 2026-10-17 | 未引用 | 正方形 |
| `tools/ladder-baseline/evidence/C-target-6_free__20261017-070935.json` | 口径 c-target 的跟踪误差读数；输入 bag/6_free__20261017-070935 | 2026-10-17 | 未引用 | 8 字 |
| `tools/ladder-baseline/evidence/control-state-rate.txt` | 五份包 /control/state 的发布周期与 AUTO/TRACK 窗口边界（窗口精度 ±1 拍） | 2026-10-09 | 未引用 | 窗口边界口径的来源 |
| `tools/ladder-baseline/evidence/solver-summary-20261017.json` | 2026-10-17 四趟实飞求解器收尾行汇总（原始 run/logs 已 gitignore） | 2026-10-17 | 未引用 | 逐字摘录，不重算 |

## 工具证据：px4-sitl

| 路径 | 是什么 | 日期/版本 | 被哪一章引用 | 备注 |
|---|---|---|---|---|
| `tools/px4-sitl/evidence/tier-a/mismatch-dense/obs-z0-1.json` | verify_tier_a_hover.py 单场观测汇总（mismatch-dense，z0 rep1；判据 checks、EKF/真值统计） | 2026-09-29 | 未引用 | Tier A 失配证据，报告未并入四源图 |
| `tools/px4-sitl/evidence/tier-a/mismatch-dense/obs-z0-2.json` | verify_tier_a_hover.py 单场观测汇总（mismatch-dense，z0 rep2；判据 checks、EKF/真值统计） | 2026-09-29 | 未引用 | Tier A 失配证据，报告未并入四源图 |
| `tools/px4-sitl/evidence/tier-a/mismatch-dense/obs-z0-3.json` | verify_tier_a_hover.py 单场观测汇总（mismatch-dense，z0 rep3；判据 checks、EKF/真值统计） | 2026-09-29 | 未引用 | Tier A 失配证据，报告未并入四源图 |
| `tools/px4-sitl/evidence/tier-a/mismatch-dense/obs-z0-mass+10-1.json` | verify_tier_a_hover.py 单场观测汇总（mismatch-dense，z0-mass+10 rep1；判据 checks、EKF/真值统计） | 2026-09-29 | 未引用 | Tier A 失配证据，报告未并入四源图 |
| `tools/px4-sitl/evidence/tier-a/mismatch-dense/obs-z0-mass+30-1.json` | verify_tier_a_hover.py 单场观测汇总（mismatch-dense，z0-mass+30 rep1；判据 checks、EKF/真值统计） | 2026-09-29 | 未引用 | Tier A 失配证据，报告未并入四源图 |
| `tools/px4-sitl/evidence/tier-a/mismatch-dense/obs-z0-mass+5-1.json` | verify_tier_a_hover.py 单场观测汇总（mismatch-dense，z0-mass+5 rep1；判据 checks、EKF/真值统计） | 2026-09-29 | 未引用 | Tier A 失配证据，报告未并入四源图 |
| `tools/px4-sitl/evidence/tier-a/mismatch-dense/obs-z0-mass-10-1.json` | verify_tier_a_hover.py 单场观测汇总（mismatch-dense，z0-mass-10 rep1；判据 checks、EKF/真值统计） | 2026-09-29 | 未引用 | Tier A 失配证据，报告未并入四源图 |
| `tools/px4-sitl/evidence/tier-a/mismatch-dense/obs-z0-mass-30-1.json` | verify_tier_a_hover.py 单场观测汇总（mismatch-dense，z0-mass-30 rep1；判据 checks、EKF/真值统计） | 2026-09-29 | 未引用 | Tier A 失配证据，报告未并入四源图 |
| `tools/px4-sitl/evidence/tier-a/mismatch-dense/obs-z0-mass-5-1.json` | verify_tier_a_hover.py 单场观测汇总（mismatch-dense，z0-mass-5 rep1；判据 checks、EKF/真值统计） | 2026-09-29 | 未引用 | Tier A 失配证据，报告未并入四源图 |
| `tools/px4-sitl/evidence/tier-a/mismatch-dense/obs-z0-thrust+10-1.json` | verify_tier_a_hover.py 单场观测汇总（mismatch-dense，z0-thrust+10 rep1；判据 checks、EKF/真值统计） | 2026-09-29 | 未引用 | Tier A 失配证据，报告未并入四源图 |
| `tools/px4-sitl/evidence/tier-a/mismatch-dense/obs-z0-thrust+30-1.json` | verify_tier_a_hover.py 单场观测汇总（mismatch-dense，z0-thrust+30 rep1；判据 checks、EKF/真值统计） | 2026-09-29 | 未引用 | Tier A 失配证据，报告未并入四源图 |
| `tools/px4-sitl/evidence/tier-a/mismatch-dense/obs-z0-thrust+5-1.json` | verify_tier_a_hover.py 单场观测汇总（mismatch-dense，z0-thrust+5 rep1；判据 checks、EKF/真值统计） | 2026-09-29 | 未引用 | Tier A 失配证据，报告未并入四源图 |
| `tools/px4-sitl/evidence/tier-a/mismatch-dense/obs-z0-thrust-10-1.json` | verify_tier_a_hover.py 单场观测汇总（mismatch-dense，z0-thrust-10 rep1；判据 checks、EKF/真值统计） | 2026-09-29 | 未引用 | Tier A 失配证据，报告未并入四源图 |
| `tools/px4-sitl/evidence/tier-a/mismatch-dense/obs-z0-thrust-30-1.json` | verify_tier_a_hover.py 单场观测汇总（mismatch-dense，z0-thrust-30 rep1；判据 checks、EKF/真值统计） | 2026-09-29 | 未引用 | Tier A 失配证据，报告未并入四源图 |
| `tools/px4-sitl/evidence/tier-a/mismatch-dense/obs-z0-thrust-5-1.json` | verify_tier_a_hover.py 单场观测汇总（mismatch-dense，z0-thrust-5 rep1；判据 checks、EKF/真值统计） | 2026-09-29 | 未引用 | Tier A 失配证据，报告未并入四源图 |
| `tools/px4-sitl/evidence/tier-a/mismatch-dense/run-z0-1.log` | 同一场的验证器与 PX4 控制台输出（mismatch-dense，z0 rep1） | 2026-09-29 | 未引用 | Tier A 失配证据，报告未并入四源图 |
| `tools/px4-sitl/evidence/tier-a/mismatch-dense/run-z0-2.log` | 同一场的验证器与 PX4 控制台输出（mismatch-dense，z0 rep2） | 2026-09-29 | 未引用 | Tier A 失配证据，报告未并入四源图 |
| `tools/px4-sitl/evidence/tier-a/mismatch-dense/run-z0-3.log` | 同一场的验证器与 PX4 控制台输出（mismatch-dense，z0 rep3） | 2026-09-29 | 未引用 | Tier A 失配证据，报告未并入四源图 |
| `tools/px4-sitl/evidence/tier-a/mismatch-dense/run-z0-mass+10-1.log` | 同一场的验证器与 PX4 控制台输出（mismatch-dense，z0-mass+10 rep1） | 2026-09-29 | 未引用 | Tier A 失配证据，报告未并入四源图 |
| `tools/px4-sitl/evidence/tier-a/mismatch-dense/run-z0-mass+30-1.log` | 同一场的验证器与 PX4 控制台输出（mismatch-dense，z0-mass+30 rep1） | 2026-09-29 | 未引用 | Tier A 失配证据，报告未并入四源图 |
| `tools/px4-sitl/evidence/tier-a/mismatch-dense/run-z0-mass+5-1.log` | 同一场的验证器与 PX4 控制台输出（mismatch-dense，z0-mass+5 rep1） | 2026-09-29 | 未引用 | Tier A 失配证据，报告未并入四源图 |
| `tools/px4-sitl/evidence/tier-a/mismatch-dense/run-z0-mass-10-1.log` | 同一场的验证器与 PX4 控制台输出（mismatch-dense，z0-mass-10 rep1） | 2026-09-29 | 未引用 | Tier A 失配证据，报告未并入四源图 |
| `tools/px4-sitl/evidence/tier-a/mismatch-dense/run-z0-mass-30-1.log` | 同一场的验证器与 PX4 控制台输出（mismatch-dense，z0-mass-30 rep1） | 2026-09-29 | 未引用 | Tier A 失配证据，报告未并入四源图 |
| `tools/px4-sitl/evidence/tier-a/mismatch-dense/run-z0-mass-5-1.log` | 同一场的验证器与 PX4 控制台输出（mismatch-dense，z0-mass-5 rep1） | 2026-09-29 | 未引用 | Tier A 失配证据，报告未并入四源图 |
| `tools/px4-sitl/evidence/tier-a/mismatch-dense/run-z0-thrust+10-1.log` | 同一场的验证器与 PX4 控制台输出（mismatch-dense，z0-thrust+10 rep1） | 2026-09-29 | 未引用 | Tier A 失配证据，报告未并入四源图 |
| `tools/px4-sitl/evidence/tier-a/mismatch-dense/run-z0-thrust+30-1.log` | 同一场的验证器与 PX4 控制台输出（mismatch-dense，z0-thrust+30 rep1） | 2026-09-29 | 未引用 | Tier A 失配证据，报告未并入四源图 |
| `tools/px4-sitl/evidence/tier-a/mismatch-dense/run-z0-thrust+5-1.log` | 同一场的验证器与 PX4 控制台输出（mismatch-dense，z0-thrust+5 rep1） | 2026-09-29 | 未引用 | Tier A 失配证据，报告未并入四源图 |
| `tools/px4-sitl/evidence/tier-a/mismatch-dense/run-z0-thrust-10-1.log` | 同一场的验证器与 PX4 控制台输出（mismatch-dense，z0-thrust-10 rep1） | 2026-09-29 | 未引用 | Tier A 失配证据，报告未并入四源图 |
| `tools/px4-sitl/evidence/tier-a/mismatch-dense/run-z0-thrust-30-1.log` | 同一场的验证器与 PX4 控制台输出（mismatch-dense，z0-thrust-30 rep1） | 2026-09-29 | 未引用 | Tier A 失配证据，报告未并入四源图 |
| `tools/px4-sitl/evidence/tier-a/mismatch-dense/run-z0-thrust-5-1.log` | 同一场的验证器与 PX4 控制台输出（mismatch-dense，z0-thrust-5 rep1） | 2026-09-29 | 未引用 | Tier A 失配证据，报告未并入四源图 |
| `tools/px4-sitl/evidence/tier-a/mismatch-dense/summary.json` | 15 场扫描的机读汇总：基准散度、退化排序、ΔF/Δz 刚度 | 2026-09-29 | 未引用 | Tier A 失配证据 |
| `tools/px4-sitl/evidence/tier-a/mismatch-dense/SUMMARY.md` | 15 场扫描的人读汇总表（同 summary.json 的表） | 2026-09-29 | 未引用 | Tier A 失配证据 |
| `tools/px4-sitl/evidence/tier-a/mismatch-dense-w20/obs-z0-mass+30-1.json` | verify_tier_a_hover.py 单场观测汇总（mismatch-dense-w20，z0-mass+30 rep1；判据 checks、EKF/真值统计） | 2026-09-29 | 未引用 | Tier A 失配证据，报告未并入四源图 |
| `tools/px4-sitl/evidence/tier-a/mismatch-dense-w20/obs-z0-mass+30-2.json` | verify_tier_a_hover.py 单场观测汇总（mismatch-dense-w20，z0-mass+30 rep2；判据 checks、EKF/真值统计） | 2026-09-29 | 未引用 | Tier A 失配证据，报告未并入四源图 |
| `tools/px4-sitl/evidence/tier-a/mismatch-dense-w20/run-z0-mass+30-1.log` | 同一场的验证器与 PX4 控制台输出（mismatch-dense-w20，z0-mass+30 rep1） | 2026-09-29 | 未引用 | Tier A 失配证据，报告未并入四源图 |
| `tools/px4-sitl/evidence/tier-a/mismatch-dense-w20/run-z0-mass+30-2.log` | 同一场的验证器与 PX4 控制台输出（mismatch-dense-w20，z0-mass+30 rep2） | 2026-09-29 | 未引用 | Tier A 失配证据，报告未并入四源图 |
| `tools/px4-sitl/evidence/tier-a/mismatch-dense-w20/summary.json` | 2 场扫描的机读汇总：基准散度、退化排序、ΔF/Δz 刚度 | 2026-09-29 | 未引用 | Tier A 失配证据 |
| `tools/px4-sitl/evidence/tier-a/mismatch-dense-w20/SUMMARY.md` | 2 场扫描的人读汇总表（同 summary.json 的表） | 2026-09-29 | 未引用 | Tier A 失配证据 |
| `tools/px4-sitl/evidence/tier-a/mismatch-sweep/obs-z0-1.json` | verify_tier_a_hover.py 单场观测汇总（mismatch-sweep，z0 rep1；判据 checks、EKF/真值统计） | 2026-09-29 | 未引用 | Tier A 失配证据，报告未并入四源图 |
| `tools/px4-sitl/evidence/tier-a/mismatch-sweep/obs-z0-2.json` | verify_tier_a_hover.py 单场观测汇总（mismatch-sweep，z0 rep2；判据 checks、EKF/真值统计） | 2026-09-29 | 未引用 | Tier A 失配证据，报告未并入四源图 |
| `tools/px4-sitl/evidence/tier-a/mismatch-sweep/obs-z0-3.json` | verify_tier_a_hover.py 单场观测汇总（mismatch-sweep，z0 rep3；判据 checks、EKF/真值统计） | 2026-09-29 | 未引用 | Tier A 失配证据，报告未并入四源图 |
| `tools/px4-sitl/evidence/tier-a/mismatch-sweep/obs-z0-inertia+30-1.json` | verify_tier_a_hover.py 单场观测汇总（mismatch-sweep，z0-inertia+30 rep1；判据 checks、EKF/真值统计） | 2026-09-29 | 未引用 | Tier A 失配证据，报告未并入四源图 |
| `tools/px4-sitl/evidence/tier-a/mismatch-sweep/obs-z0-inertia-30-1.json` | verify_tier_a_hover.py 单场观测汇总（mismatch-sweep，z0-inertia-30 rep1；判据 checks、EKF/真值统计） | 2026-09-29 | 未引用 | Tier A 失配证据，报告未并入四源图 |
| `tools/px4-sitl/evidence/tier-a/mismatch-sweep/obs-z0-mass+30-1.json` | verify_tier_a_hover.py 单场观测汇总（mismatch-sweep，z0-mass+30 rep1；判据 checks、EKF/真值统计） | 2026-09-29 | 未引用 | Tier A 失配证据，报告未并入四源图 |
| `tools/px4-sitl/evidence/tier-a/mismatch-sweep/obs-z0-mass-30-1.json` | verify_tier_a_hover.py 单场观测汇总（mismatch-sweep，z0-mass-30 rep1；判据 checks、EKF/真值统计） | 2026-09-29 | 未引用 | Tier A 失配证据，报告未并入四源图 |
| `tools/px4-sitl/evidence/tier-a/mismatch-sweep/obs-z0-rate+30-1.json` | verify_tier_a_hover.py 单场观测汇总（mismatch-sweep，z0-rate+30 rep1；判据 checks、EKF/真值统计） | 2026-09-29 | 未引用 | Tier A 失配证据，报告未并入四源图 |
| `tools/px4-sitl/evidence/tier-a/mismatch-sweep/obs-z0-rate-30-1.json` | verify_tier_a_hover.py 单场观测汇总（mismatch-sweep，z0-rate-30 rep1；判据 checks、EKF/真值统计） | 2026-09-29 | 未引用 | Tier A 失配证据，报告未并入四源图 |
| `tools/px4-sitl/evidence/tier-a/mismatch-sweep/obs-z0-tau+30-1.json` | verify_tier_a_hover.py 单场观测汇总（mismatch-sweep，z0-tau+30 rep1；判据 checks、EKF/真值统计） | 2026-09-29 | 未引用 | Tier A 失配证据，报告未并入四源图 |
| `tools/px4-sitl/evidence/tier-a/mismatch-sweep/obs-z0-tau-30-1.json` | verify_tier_a_hover.py 单场观测汇总（mismatch-sweep，z0-tau-30 rep1；判据 checks、EKF/真值统计） | 2026-09-29 | 未引用 | Tier A 失配证据，报告未并入四源图 |
| `tools/px4-sitl/evidence/tier-a/mismatch-sweep/obs-z0-thrust+30-1.json` | verify_tier_a_hover.py 单场观测汇总（mismatch-sweep，z0-thrust+30 rep1；判据 checks、EKF/真值统计） | 2026-09-29 | 未引用 | Tier A 失配证据，报告未并入四源图 |
| `tools/px4-sitl/evidence/tier-a/mismatch-sweep/obs-z0-thrust-30-1.json` | verify_tier_a_hover.py 单场观测汇总（mismatch-sweep，z0-thrust-30 rep1；判据 checks、EKF/真值统计） | 2026-09-29 | 未引用 | Tier A 失配证据，报告未并入四源图 |
| `tools/px4-sitl/evidence/tier-a/mismatch-sweep/run-z0-1.log` | 同一场的验证器与 PX4 控制台输出（mismatch-sweep，z0 rep1） | 2026-09-29 | 未引用 | Tier A 失配证据，报告未并入四源图 |
| `tools/px4-sitl/evidence/tier-a/mismatch-sweep/run-z0-2.log` | 同一场的验证器与 PX4 控制台输出（mismatch-sweep，z0 rep2） | 2026-09-29 | 未引用 | Tier A 失配证据，报告未并入四源图 |
| `tools/px4-sitl/evidence/tier-a/mismatch-sweep/run-z0-3.log` | 同一场的验证器与 PX4 控制台输出（mismatch-sweep，z0 rep3） | 2026-09-29 | 未引用 | Tier A 失配证据，报告未并入四源图 |
| `tools/px4-sitl/evidence/tier-a/mismatch-sweep/run-z0-inertia+30-1.log` | 同一场的验证器与 PX4 控制台输出（mismatch-sweep，z0-inertia+30 rep1） | 2026-09-29 | 未引用 | Tier A 失配证据，报告未并入四源图 |
| `tools/px4-sitl/evidence/tier-a/mismatch-sweep/run-z0-inertia-30-1.log` | 同一场的验证器与 PX4 控制台输出（mismatch-sweep，z0-inertia-30 rep1） | 2026-09-29 | 未引用 | Tier A 失配证据，报告未并入四源图 |
| `tools/px4-sitl/evidence/tier-a/mismatch-sweep/run-z0-mass+30-1.log` | 同一场的验证器与 PX4 控制台输出（mismatch-sweep，z0-mass+30 rep1） | 2026-09-29 | 未引用 | Tier A 失配证据，报告未并入四源图 |
| `tools/px4-sitl/evidence/tier-a/mismatch-sweep/run-z0-mass-30-1.log` | 同一场的验证器与 PX4 控制台输出（mismatch-sweep，z0-mass-30 rep1） | 2026-09-29 | 未引用 | Tier A 失配证据，报告未并入四源图 |
| `tools/px4-sitl/evidence/tier-a/mismatch-sweep/run-z0-rate+30-1.log` | 同一场的验证器与 PX4 控制台输出（mismatch-sweep，z0-rate+30 rep1） | 2026-09-29 | 未引用 | Tier A 失配证据，报告未并入四源图 |
| `tools/px4-sitl/evidence/tier-a/mismatch-sweep/run-z0-rate-30-1.log` | 同一场的验证器与 PX4 控制台输出（mismatch-sweep，z0-rate-30 rep1） | 2026-09-29 | 未引用 | Tier A 失配证据，报告未并入四源图 |
| `tools/px4-sitl/evidence/tier-a/mismatch-sweep/run-z0-tau+30-1.log` | 同一场的验证器与 PX4 控制台输出（mismatch-sweep，z0-tau+30 rep1） | 2026-09-29 | 未引用 | Tier A 失配证据，报告未并入四源图 |
| `tools/px4-sitl/evidence/tier-a/mismatch-sweep/run-z0-tau-30-1.log` | 同一场的验证器与 PX4 控制台输出（mismatch-sweep，z0-tau-30 rep1） | 2026-09-29 | 未引用 | Tier A 失配证据，报告未并入四源图 |
| `tools/px4-sitl/evidence/tier-a/mismatch-sweep/run-z0-thrust+30-1.log` | 同一场的验证器与 PX4 控制台输出（mismatch-sweep，z0-thrust+30 rep1） | 2026-09-29 | 未引用 | Tier A 失配证据，报告未并入四源图 |
| `tools/px4-sitl/evidence/tier-a/mismatch-sweep/run-z0-thrust-30-1.log` | 同一场的验证器与 PX4 控制台输出（mismatch-sweep，z0-thrust-30 rep1） | 2026-09-29 | 未引用 | Tier A 失配证据，报告未并入四源图 |
| `tools/px4-sitl/evidence/tier-a/mismatch-sweep/summary.json` | 13 场扫描的机读汇总：基准散度、退化排序、ΔF/Δz 刚度 | 2026-09-29 | 未引用 | Tier A 失配证据 |
| `tools/px4-sitl/evidence/tier-a/mismatch-sweep/SUMMARY.md` | 13 场扫描的人读汇总表（同 summary.json 的表） | 2026-09-29 | 未引用 | Tier A 失配证据 |
| `tools/px4-sitl/evidence/tier-a/smoke-mass30-node-default/obs-z0-mass+30-1.json` | 单场观测汇总（mass+30，节点默认参数） | 2026-09-30 | 未引用 | Tier A 失配证据 |
| `tools/px4-sitl/evidence/tier-a/smoke-mass30-node-default/run-z0-mass+30-1.log` | 该场的验证器与 PX4 控制台输出 | 2026-09-30 | 未引用 | Tier A 失配证据 |
| `tools/px4-sitl/evidence/tier-a/smoke-mass30-node-default/summary.json` | 单场机读汇总（无基准，质量 +30%） | 2026-09-30 | 未引用 | Tier A 失配证据 |
| `tools/px4-sitl/evidence/tier-a/smoke-rebuild-node-default/command.txt` | 复跑命令、起止时刻与落库路径 | 2026-10-09 | 未引用 | Tier A 冒烟 |
| `tools/px4-sitl/evidence/tier-a/smoke-rebuild-node-default/hover-1m-px4.jsonl` | DUT 逐帧 JSONL（求解量、x0、fallback、耗时） | 2026-10-09 | 未引用 | Tier A 冒烟 |
| `tools/px4-sitl/evidence/tier-a/smoke-rebuild-node-default/obs-rebuild-nominal-1.json` | 单场观测汇总（重搭后被控对象，节点默认参数） | 2026-10-09 | 未引用 | Tier A 冒烟 |
| `tools/px4-sitl/evidence/tier-a/smoke-rebuild-node-default/run-rebuild-nominal-1.log` | 该场的验证器与 PX4 控制台输出 | 2026-10-09 | 未引用 | Tier A 冒烟 |
| `tools/px4-sitl/evidence/tier-a/smoke-rebuild-node-default/stdout.log` | 验证器标准输出（参数读回与告警） | 2026-10-09 | 未引用 | Tier A 冒烟 |
| `tools/px4-sitl/evidence/tier-b/README.md` | B 档矩阵归档说明：九场来源、文件口径、重跑命令与四条必知事项 | 2026-10-09 | 未引用 | Tier B 证据，读数归宿见 docs/control/tier-b-sitl-seam3.md |
| `tools/px4-sitl/evidence/tier-b/run_matrix.sh` | 跑 B 档九场的脚本（含等空机闸门） | 2026-10-09 | 未引用 | Tier B 证据，读数归宿见 docs/control/tier-b-sitl-seam3.md |
| `tools/px4-sitl/evidence/tier-b/controller-code-20260930-F0c-F1c.json` | F0c/F1c 两场的代码身份（git HEAD + 逐文件 sha256） | 2026-09-30 | 未引用 | Tier B 证据，读数归宿见 docs/control/tier-b-sitl-seam3.md |
| `tools/px4-sitl/evidence/tier-b/df-column-20260930.json` | F0b/F1b/F0c/F1c 四场逐帧 [tick, df, u[0], thrust_body_z] 与发布点残差 | 2026-09-30 | 未引用 | Tier B 证据，读数归宿见 docs/control/tier-b-sitl-seam3.md |
| `tools/px4-sitl/evidence/tier-b/oscillation-mechanism-20261009.json` | 垂直通道振荡机理读数（逐场扫描、振铃拟合、延时扫描、回归） | 2026-10-09 | 未引用 | Tier B 证据，读数归宿见 docs/control/tier-b-sitl-seam3.md |
| `tools/px4-sitl/evidence/tier-b/rejudge-20261008-judge2.json` | 判据② 换真判后对 F0c/F1c 的离线重判全文 | 2026-10-08 | 未引用 | Tier B 证据，读数归宿见 docs/control/tier-b-sitl-seam3.md |
| `tools/px4-sitl/evidence/tier-b/mocap-model-3.json` | 回放夹具模型：来自真实 Phase 1 悬停包（逐轴 σ 与成簇帧间隔） | 2026-09-29 | 未引用 | Tier B 证据，读数归宿见 docs/control/tier-b-sitl-seam3.md |
| `tools/px4-sitl/evidence/tier-b/mocap-model-4.json` | 回放夹具模型：八字档，参考包络 1.000 × 0.500 m（0.5 m 旧规格） | 2026-10-08 | 未引用 | Tier B 证据，读数归宿见 docs/control/tier-b-sitl-seam3.md |
| `tools/px4-sitl/evidence/tier-b/mocap-model-5.json` | 回放夹具模型：八字档，参考包络 2.000 × 1.000 m（1.0 m 当前规格） | 2026-10-09 | 未引用 | Tier B 证据，读数归宿见 docs/control/tier-b-sitl-seam3.md |
| `tools/px4-sitl/evidence/tier-b/u-ref-authority-F0b.json` | 离线探针产物：F0b 的 d u[0]/d u_ref[0] 通路符号与增益（不是飞行） | 2026-09-29 | 未引用 | Tier B 证据，读数归宿见 docs/control/tier-b-sitl-seam3.md |
| `tools/px4-sitl/evidence/tier-b/u-ref-authority-F1b.json` | 离线探针产物：F1b 的 d u[0]/d u_ref[0] 通路符号与增益（不是飞行） | 2026-09-29 | 未引用 | Tier B 证据，读数归宿见 docs/control/tier-b-sitl-seam3.md |
| `tools/px4-sitl/evidence/tier-b/u-ref-authority-F1b-noerr.json` | 离线探针产物：F1b 去掉误差项后的通路增益对照 | 2026-09-29 | 未引用 | Tier B 证据，读数归宿见 docs/control/tier-b-sitl-seam3.md |
| `tools/px4-sitl/evidence/tier-b/u-ref-authority-F1b-sameframes-20260930.json` | 离线探针产物：F1b 同批帧、新树的复跑（量中间提交的影响） | 2026-09-30 | 未引用 | Tier B 证据，读数归宿见 docs/control/tier-b-sitl-seam3.md |
| `tools/px4-sitl/evidence/tier-b/F0-console.log` | F0 场 PX4 SITL 控制台（pxh>） | 2026-09-29 | 未引用 | Tier B 证据，读数归宿见 docs/control/tier-b-sitl-seam3.md |
| `tools/px4-sitl/evidence/tier-b/F0-replay.json` | F0 场回放夹具收尾统计（发帧数、实测延迟、σ） | 2026-09-29 | 未引用 | Tier B 证据，读数归宿见 docs/control/tier-b-sitl-seam3.md |
| `tools/px4-sitl/evidence/tier-b/F0-summary.json` | B 档矩阵 F0 场观测汇总（标称；含 checks 判定行） | 2026-09-29 | 未引用 | Tier B 证据，读数归宿见 docs/control/tier-b-sitl-seam3.md |
| `tools/px4-sitl/evidence/tier-b/F0b-console.log` | F0b 场 PX4 SITL 控制台（pxh>） | 2026-09-29 | 未引用 | Tier B 证据，读数归宿见 docs/control/tier-b-sitl-seam3.md |
| `tools/px4-sitl/evidence/tier-b/F0b-replay.json` | F0b 场回放夹具收尾统计（发帧数、实测延迟、σ） | 2026-09-29 | 未引用 | Tier B 证据，读数归宿见 docs/control/tier-b-sitl-seam3.md |
| `tools/px4-sitl/evidence/tier-b/F0b-summary.json` | B 档矩阵 F0b 场观测汇总（标称 + 偏置加在 u_ref（落点后被证伪）；含 checks 判定行） | 2026-09-29 | 未引用 | Tier B 证据，读数归宿见 docs/control/tier-b-sitl-seam3.md |
| `tools/px4-sitl/evidence/tier-b/F0c-console.log` | F0c 场 PX4 SITL 控制台（pxh>） | 2026-09-30 | 未引用 | Tier B 证据，读数归宿见 docs/control/tier-b-sitl-seam3.md |
| `tools/px4-sitl/evidence/tier-b/F0c-replay.json` | F0c 场回放夹具收尾统计（发帧数、实测延迟、σ） | 2026-09-30 | 未引用 | Tier B 证据，读数归宿见 docs/control/tier-b-sitl-seam3.md |
| `tools/px4-sitl/evidence/tier-b/F0c-summary.json` | B 档矩阵 F0c 场观测汇总（标称 + 偏置加在发出指令（候选 A）；含 checks 判定行） | 2026-09-30 | 未引用 | Tier B 证据，读数归宿见 docs/control/tier-b-sitl-seam3.md |
| `tools/px4-sitl/evidence/tier-b/F1-console.log` | F1 场 PX4 SITL 控制台（pxh>） | 2026-09-29 | 未引用 | Tier B 证据，读数归宿见 docs/control/tier-b-sitl-seam3.md |
| `tools/px4-sitl/evidence/tier-b/F1-replay.json` | F1 场回放夹具收尾统计（发帧数、实测延迟、σ） | 2026-09-29 | 未引用 | Tier B 证据，读数归宿见 docs/control/tier-b-sitl-seam3.md |
| `tools/px4-sitl/evidence/tier-b/F1-summary.json` | B 档矩阵 F1 场观测汇总（mass+30（判据①）；含 checks 判定行） | 2026-09-29 | 未引用 | Tier B 证据，读数归宿见 docs/control/tier-b-sitl-seam3.md |
| `tools/px4-sitl/evidence/tier-b/F1b-console.log` | F1b 场 PX4 SITL 控制台（pxh>） | 2026-09-29 | 未引用 | Tier B 证据，读数归宿见 docs/control/tier-b-sitl-seam3.md |
| `tools/px4-sitl/evidence/tier-b/F1b-replay.json` | F1b 场回放夹具收尾统计（发帧数、实测延迟、σ） | 2026-09-29 | 未引用 | Tier B 证据，读数归宿见 docs/control/tier-b-sitl-seam3.md |
| `tools/px4-sitl/evidence/tier-b/F1b-summary.json` | B 档矩阵 F1b 场观测汇总（mass+30 + 偏置加在 u_ref（落点后被证伪）；含 checks 判定行） | 2026-09-29 | 未引用 | Tier B 证据，读数归宿见 docs/control/tier-b-sitl-seam3.md |
| `tools/px4-sitl/evidence/tier-b/F1c-console.log` | F1c 场 PX4 SITL 控制台（pxh>） | 2026-09-30 | 未引用 | Tier B 证据，读数归宿见 docs/control/tier-b-sitl-seam3.md |
| `tools/px4-sitl/evidence/tier-b/F1c-replay.json` | F1c 场回放夹具收尾统计（发帧数、实测延迟、σ） | 2026-09-30 | 未引用 | Tier B 证据，读数归宿见 docs/control/tier-b-sitl-seam3.md |
| `tools/px4-sitl/evidence/tier-b/F1c-summary.json` | B 档矩阵 F1c 场观测汇总（mass+30 + 偏置加在发出指令（候选 A）；含 checks 判定行） | 2026-09-30 | 未引用 | Tier B 证据，读数归宿见 docs/control/tier-b-sitl-seam3.md |
| `tools/px4-sitl/evidence/tier-b/F2-console.log` | F2 场 PX4 SITL 控制台（pxh>） | 2026-09-29 | 未引用 | Tier B 证据，读数归宿见 docs/control/tier-b-sitl-seam3.md |
| `tools/px4-sitl/evidence/tier-b/F2-replay.json` | F2 场回放夹具收尾统计（发帧数、实测延迟、σ） | 2026-09-29 | 未引用 | Tier B 证据，读数归宿见 docs/control/tier-b-sitl-seam3.md |
| `tools/px4-sitl/evidence/tier-b/F2-summary.json` | B 档矩阵 F2 场观测汇总（mass+30 + 50 ms（判据③）；含 checks 判定行） | 2026-09-29 | 未引用 | Tier B 证据，读数归宿见 docs/control/tier-b-sitl-seam3.md |
| `tools/px4-sitl/evidence/tier-b/F3-console.log` | F3 场 PX4 SITL 控制台（pxh>） | 2026-09-29 | 未引用 | Tier B 证据，读数归宿见 docs/control/tier-b-sitl-seam3.md |
| `tools/px4-sitl/evidence/tier-b/F3-replay.json` | F3 场回放夹具收尾统计（发帧数、实测延迟、σ） | 2026-09-29 | 未引用 | Tier B 证据，读数归宿见 docs/control/tier-b-sitl-seam3.md |
| `tools/px4-sitl/evidence/tier-b/F3-summary.json` | B 档矩阵 F3 场观测汇总（mass+30 + 注入求解失败（判据④）；含 checks 判定行） | 2026-09-29 | 未引用 | Tier B 证据，读数归宿见 docs/control/tier-b-sitl-seam3.md |
| `tools/px4-sitl/evidence/tier-b/F4-console.log` | F4 场 PX4 SITL 控制台（pxh>） | 2026-09-29 | 未引用 | Tier B 证据，读数归宿见 docs/control/tier-b-sitl-seam3.md |
| `tools/px4-sitl/evidence/tier-b/F4-replay.json` | F4 场回放夹具收尾统计（发帧数、实测延迟、σ） | 2026-09-29 | 未引用 | Tier B 证据，读数归宿见 docs/control/tier-b-sitl-seam3.md |
| `tools/px4-sitl/evidence/tier-b/F4-summary.json` | B 档矩阵 F4 场观测汇总（标称 + 注入（判据④对照）；含 checks 判定行） | 2026-09-29 | 未引用 | Tier B 证据，读数归宿见 docs/control/tier-b-sitl-seam3.md |
| `tools/px4-sitl/evidence/tier-b/20261009-rebuild/README.md` | 重搭后六场（F0–F4 + T0）的归档说明与三条必知事项 | 2026-10-09 | 未引用 | 读数在 phase3/06 票 Comments |
| `tools/px4-sitl/evidence/tier-b/20261009-rebuild/command.txt` | 六场的复跑命令、逐场时刻、退出码与落库口径 | 2026-10-09 | 未引用 | Tier B 重搭证据 |
| `tools/px4-sitl/evidence/tier-b/20261009-rebuild/harness-identity-2026-10-09.json` | 该批工具链身份：控制器、入口、夹具、判层、被控对象、参考包的 sha256 | 2026-10-09 | 未引用 | 跑完后补写 |
| `tools/px4-sitl/evidence/tier-b/20261009-rebuild/T0-replay-frames-reference.jsonl` | T0 场夹具发出的参考逐帧（wall_ns + 相位 + 位置/速度），供离线重判 | 2026-10-09 | 未引用 | 原始逐帧，判层的派生量来源 |
| `tools/px4-sitl/evidence/tier-b/20261009-rebuild/F0-console.log` | 重搭后 F0 场 PX4 SITL 控制台 | 2026-10-09 | 未引用 | Tier B 重搭证据；与 2026-09-29 那批不可逐位并排 |
| `tools/px4-sitl/evidence/tier-b/20261009-rebuild/F0-replay.json` | 重搭后 F0 场回放夹具收尾统计 | 2026-10-09 | 未引用 | Tier B 重搭证据；与 2026-09-29 那批不可逐位并排 |
| `tools/px4-sitl/evidence/tier-b/20261009-rebuild/F0-summary.json` | 重搭后 F0 场的观测汇总（含 frames/truth，可离线重判） | 2026-10-09 | 未引用 | Tier B 重搭证据；与 2026-09-29 那批不可逐位并排 |
| `tools/px4-sitl/evidence/tier-b/20261009-rebuild/F1-console.log` | 重搭后 F1 场 PX4 SITL 控制台 | 2026-10-09 | 未引用 | Tier B 重搭证据；与 2026-09-29 那批不可逐位并排 |
| `tools/px4-sitl/evidence/tier-b/20261009-rebuild/F1-replay.json` | 重搭后 F1 场回放夹具收尾统计 | 2026-10-09 | 未引用 | Tier B 重搭证据；与 2026-09-29 那批不可逐位并排 |
| `tools/px4-sitl/evidence/tier-b/20261009-rebuild/F1-summary.json` | 重搭后 F1 场的观测汇总（含 frames/truth，可离线重判） | 2026-10-09 | 未引用 | Tier B 重搭证据；与 2026-09-29 那批不可逐位并排 |
| `tools/px4-sitl/evidence/tier-b/20261009-rebuild/F2-console.log` | 重搭后 F2 场 PX4 SITL 控制台 | 2026-10-09 | 未引用 | Tier B 重搭证据；与 2026-09-29 那批不可逐位并排 |
| `tools/px4-sitl/evidence/tier-b/20261009-rebuild/F2-replay.json` | 重搭后 F2 场回放夹具收尾统计 | 2026-10-09 | 未引用 | Tier B 重搭证据；与 2026-09-29 那批不可逐位并排 |
| `tools/px4-sitl/evidence/tier-b/20261009-rebuild/F2-summary.json` | 重搭后 F2 场的观测汇总（含 frames/truth，可离线重判） | 2026-10-09 | 未引用 | Tier B 重搭证据；与 2026-09-29 那批不可逐位并排 |
| `tools/px4-sitl/evidence/tier-b/20261009-rebuild/F3-console.log` | 重搭后 F3 场 PX4 SITL 控制台 | 2026-10-09 | 未引用 | Tier B 重搭证据；与 2026-09-29 那批不可逐位并排 |
| `tools/px4-sitl/evidence/tier-b/20261009-rebuild/F3-replay.json` | 重搭后 F3 场回放夹具收尾统计 | 2026-10-09 | 未引用 | Tier B 重搭证据；与 2026-09-29 那批不可逐位并排 |
| `tools/px4-sitl/evidence/tier-b/20261009-rebuild/F3-summary.json` | 重搭后 F3 场的观测汇总（含 frames/truth，可离线重判） | 2026-10-09 | 未引用 | Tier B 重搭证据；与 2026-09-29 那批不可逐位并排 |
| `tools/px4-sitl/evidence/tier-b/20261009-rebuild/F4-console.log` | 重搭后 F4 场 PX4 SITL 控制台 | 2026-10-09 | 未引用 | Tier B 重搭证据；与 2026-09-29 那批不可逐位并排 |
| `tools/px4-sitl/evidence/tier-b/20261009-rebuild/F4-replay.json` | 重搭后 F4 场回放夹具收尾统计 | 2026-10-09 | 未引用 | Tier B 重搭证据；与 2026-09-29 那批不可逐位并排 |
| `tools/px4-sitl/evidence/tier-b/20261009-rebuild/F4-summary.json` | 重搭后 F4 场的观测汇总（含 frames/truth，可离线重判） | 2026-10-09 | 未引用 | Tier B 重搭证据；与 2026-09-29 那批不可逐位并排 |
| `tools/px4-sitl/evidence/tier-b/20261009-rebuild/T0-console.log` | 重搭后 T0 场 PX4 SITL 控制台 | 2026-10-09 | 未引用 | Tier B 重搭证据；与 2026-09-29 那批不可逐位并排 |
| `tools/px4-sitl/evidence/tier-b/20261009-rebuild/T0-replay.json` | 重搭后 T0 场回放夹具收尾统计 | 2026-10-09 | 未引用 | Tier B 重搭证据；与 2026-09-29 那批不可逐位并排 |
| `tools/px4-sitl/evidence/tier-b/20261009-rebuild/T0-summary.json` | 重搭后 T0 场（mass+30 重搭被控对象，1.0 m 八字参考）的观测汇总（含 frames/truth，可离线重判） | 2026-10-09 | 第8章 | Tier B 重搭证据；与 2026-09-29 那批不可逐位并排 |
| `tools/px4-sitl/evidence/z0-plant/variants-all-2026-10-09.json` | 12 个被控对象变体的参数自检（质量、推力、惯量、τ、速率、推力曲线） | 2026-10-09 | 未引用 | 参数逐项对照，不是飞行 |

## 工具证据：report

| 路径 | 是什么 | 日期/版本 | 被哪一章引用 | 备注 |
|---|---|---|---|---|
| `tools/report/evidence/compare_pose_sources.py` | 对照脚本：把 /mavros/vision_pose/pose 与 /mavros/local_position/pose 对齐后算位置/姿态差 | 2026-10-18 | 未引用 | 第9章素材，正文未点名本文件 |
| `tools/report/evidence/pose-source-comparison.json` | compare_pose_sources.py 的输出：六个场景-源组合的 VP/LP 位置 RMS 与姿态差 | 2026-10-18 | 未引用 | 第9章素材，正文未点名本文件 |
| `tools/report/evidence/mpc-normalization.json` | 模型预测控制源的推力归一化出处：三趟启动行原文、t_max、日志 sha256 | 2026-10-17 | 第9、10章 | generated 图注也引用 |
| `tools/report/evidence/topic-frequency-20261017.json` | 六份包的逐话题帧数、跨度、中位周期与最大间隔 | 2026-10-17 | 第5章 | 表 5-5 的频率读数来源 |
| `tools/report/evidence/sitl/README.md` | 三趟 nominal SITL 的配置、模型生成、复跑命令、产物与结果摘要 | 2026-10-17 | 第9章 | 第9章链到本文件 |
| `tools/report/evidence/sitl/build_hover_model.py` | 悬停参考模型构建脚本：TRACK 在包尾截断时只改窗口终点，其余复用 build_model() | 2026-10-17 | 第9章 | 第9章点名 |
| `tools/report/evidence/sitl/extract_solver_summary.py` | 从原始 DUT JSONL 抽取 solve_ms 均值/p50/p95/最坏、退化与饱和拍数 | 2026-10-17 | 未引用 | 复跑命令的一部分 |
| `tools/report/evidence/sitl/run_nominal_sitl.py` | verify_tier_b_seam3.py 的最小包装（补 offboard 预流、模式/解锁字段、FRAME_KEYS） | 2026-10-17 | 未引用 | 复跑命令的一部分 |
| `tools/report/evidence/sitl/write_harness_identity.py` | 跑完后写本次运行身份（模型/包/控制器/夹具/被控对象的 sha256） | 2026-10-17 | 未引用 | 复跑命令的一部分 |
| `tools/report/evidence/sitl/models/R-hover-model.json` | 回放夹具的参考模型（参考取自 bag/3_traj1_hover） | 2026-10-17 | 未引用 | 第9章描述生成方法，未点名文件 |
| `tools/report/evidence/sitl/R-hover/command.txt` | R-hover 的复现命令（中间件 + 本场 + 求解汇总） | 2026-10-17 | 未引用 | nominal SITL 产物 |
| `tools/report/evidence/sitl/R-hover/harness-identity.json` | R-hover 的输入 sha256（模型、包、控制器、被控对象、夹具、中间件） | 2026-10-17 | 未引用 | nominal SITL 产物 |
| `tools/report/evidence/sitl/R-hover/R-hover-console.log` | R-hover 的验证器控制台全文 | 2026-10-17 | 未引用 | nominal SITL 产物 |
| `tools/report/evidence/sitl/R-hover/R-hover-summary.json` | R-hover 的验证器 obs 原样（frames、truth、判据读数）；报告图与指标的数据源 | 2026-10-17 | 第9、10章 | generated 图注也引用 |
| `tools/report/evidence/sitl/R-hover/solver-summary.json` | R-hover 的求解耗时汇总（均值/p50/p95/最坏、退化/失败/饱和拍数） | 2026-10-17 | 第9、10章 | generated 指标也引用 |
| `tools/report/evidence/sitl/models/R-square-model.json` | 回放夹具的参考模型（参考取自 bag/4_traj2_square） | 2026-10-17 | 未引用 | 第9章描述生成方法，未点名文件 |
| `tools/report/evidence/sitl/R-square/command.txt` | R-square 的复现命令（中间件 + 本场 + 求解汇总） | 2026-10-17 | 未引用 | nominal SITL 产物 |
| `tools/report/evidence/sitl/R-square/harness-identity.json` | R-square 的输入 sha256（模型、包、控制器、被控对象、夹具、中间件） | 2026-10-17 | 未引用 | nominal SITL 产物 |
| `tools/report/evidence/sitl/R-square/R-square-console.log` | R-square 的验证器控制台全文 | 2026-10-17 | 未引用 | nominal SITL 产物 |
| `tools/report/evidence/sitl/R-square/R-square-summary.json` | R-square 的验证器 obs 原样（frames、truth、判据读数）；报告图与指标的数据源 | 2026-10-17 | 第9、10章 | generated 图注也引用 |
| `tools/report/evidence/sitl/R-square/solver-summary.json` | R-square 的求解耗时汇总（均值/p50/p95/最坏、退化/失败/饱和拍数） | 2026-10-17 | 第9、10章 | generated 指标也引用 |
| `tools/report/evidence/sitl/models/R-lemniscate-model.json` | 回放夹具的参考模型（参考取自 bag/5_traj3_lemniscate） | 2026-10-17 | 未引用 | 第9章描述生成方法，未点名文件 |
| `tools/report/evidence/sitl/R-lemniscate/command.txt` | R-lemniscate 的复现命令（中间件 + 本场 + 求解汇总） | 2026-10-17 | 未引用 | nominal SITL 产物 |
| `tools/report/evidence/sitl/R-lemniscate/harness-identity.json` | R-lemniscate 的输入 sha256（模型、包、控制器、被控对象、夹具、中间件） | 2026-10-17 | 未引用 | nominal SITL 产物 |
| `tools/report/evidence/sitl/R-lemniscate/R-lemniscate-console.log` | R-lemniscate 的验证器控制台全文 | 2026-10-17 | 未引用 | nominal SITL 产物 |
| `tools/report/evidence/sitl/R-lemniscate/R-lemniscate-summary.json` | R-lemniscate 的验证器 obs 原样（frames、truth、判据读数）；报告图与指标的数据源 | 2026-10-17 | 第9、10章 | generated 图注也引用 |
| `tools/report/evidence/sitl/R-lemniscate/solver-summary.json` | R-lemniscate 的求解耗时汇总（均值/p50/p95/最坏、退化/失败/饱和拍数） | 2026-10-17 | 第9、10章 | generated 指标也引用 |

## 工具证据：rti-convergence 与 fcu-params

| 路径 | 是什么 | 日期/版本 | 被哪一章引用 | 备注 |
|---|---|---|---|---|
| `tools/rti-convergence/evidence/scan.json` | RTI 收敛扫描表 1：目标偏移 × R_yaw 的逐拍衰减与判定（极限环/慢收敛/已收敛） | 2026-09-30 | 第8章（目录级） | 第8章引 tools/rti-convergence/ 的读数 |
| `tools/rti-convergence/evidence/r_yaw_sweep.json` | RTI 收敛扫描表 2：R_yaw 单轴 @ 10 m 的收敛边界 | 2026-09-30 | 第8章（目录级） | 第8章引 tools/rti-convergence/ 的读数 |
| `tools/fcu-params/evidence/fcu-dump-2026-10-09.raw.yaml` | 修复前的飞控参数全量 dump：整型参数未按 param_type 解，值不可用，仅留档 | 2026-10-09 | 未引用 | 文件头已标作废；修复后的口径见工具 README |

## 录制包

| 路径 | 是什么 | 日期/版本 | 被哪一章引用 | 备注 |
|---|---|---|---|---|
| `bag/1_static` | 场景 1：动捕 + mavros 起来后原地静止（噪声基线）；未解锁，无飞行 | 2026-09-29 | 第3章 | 117.3 s；metadata.yaml 在、missing_expected 为空；含 session.json、1_static.log、fcu/log_0_2026-9-29-15-22-20.ulg + INDEX.md |
| `bag/2_manual_takeoff` | 场景 2：定点模式手动推摇杆起飞到约 1.45 m 再落回 | 2026-09-29 | 未引用 | 72.8 s；包完整；session.json 为 rec_adopt 事后重建；手动遥控，无控制器 |
| `bag/3_traj1_hover` | 场景 3：轨迹 1 悬停 60 s，offboard（PX4 内置 PID） | 2026-10-08 | 第9、10章 | 116.4 s；包完整（missing_expected 为空），但 TRACK 只录到 39.65 s（progress 终值 0.661），属录制截断；含 session.json、3_traj1_hover.log、fcu/log_0_未知日期.ulg + INDEX.md |
| `bag/4_traj2_square` | 场景 4：轨迹 2 1 m 正方形，offboard（PX4 内置 PID） | 2026-10-08 | 第9、10章 | 36.9 s；包完整；TRACK 8.0 s 整段；session.json 为 rec_adopt 事后重建；含 fcu/log_1_未知日期.ulg + INDEX.md |
| `bag/5_traj3_lemniscate` | 场景 5：轨迹 3 1 m 八字，offboard（PX4 内置 PID） | 2026-10-08 | 第9、10章 | 49.6 s；包完整；TRACK 12.2 s 整段；session.json 为 rec_adopt 事后重建；含 fcu/log_4_未知日期.ulg + INDEX.md |
| `bag/6_free__20261017-020757` | 场景 6：轨迹 4 起飞（原点升到 0.2 m 后悬停），模型预测控制独立档（reference_source=traj） | 2026-10-17 | 未引用 | 64.3 s；包完整；系绳那趟；session.json 为 rec_adopt 事后重建；未用于四源图 |
| `bag/6_free__20261017-070103` | 场景 6：轨迹 1 悬停 60 s，模型预测控制（output_mode=external） | 2026-10-17 | 第9、10章 | 90.0 s；包完整；TRACK 60.02 s；含 fcu/log_212_未知日期.ulg + INDEX.md |
| `bag/6_free__20261017-070446` | 场景 6：轨迹 2 1 m 正方形，模型预测控制（output_mode=external） | 2026-10-17 | 第9、10章 | 38.0 s；包完整；TRACK 8.01 s；含 fcu/log_213_未知日期.ulg + INDEX.md |
| `bag/6_free__20261017-070935` | 场景 6：轨迹 3 1 m 八字，模型预测控制（output_mode=external） | 2026-10-17 | 第9、10章 | 46.3 s；包完整；TRACK 12.20 s；含 fcu/log_214_未知日期.ulg + INDEX.md |
| `bag/MANIFEST.md` | bag/ 当前包的自动总表：起点、时长、大小、消息数、话题数、包内容 sha256 与飞控日志 sha256 | 2026-10-17（内容快照；表本身无生成时间） | 第3、6、7、9章 | 确定性输出，不含生成时间；由 run/z0.py rec_manifest 重算 |
| `bag/README.md` | bag/ 的组织规则、六个场景、2026-09-29/10-08 两趟读数、飞控日志归档与已知坑 | 2026-10-17（含当日四份包） | 第2、6、9章 | 场景与生命周期规则的说明 |
| `bag/old/README.md` | bag/old/ 的归档规则与「谁取代了谁」 | 2026-10-08 | 未引用 | 历史/被取代包不参与主结论 |
| `bag/old/3_traj1_hover` | 历史/被取代：2026-09-29 悬停包，offboard | 2026-09-29 | 未引用 | 105.0 s；被 bag/3_traj1_hover（10-08）取代；不参与主结论；含 fcu/log_2_2026-9-29-15-29-26.ulg |
| `bag/old/4_traj3_lemniscate` | 历史/被取代：2026-09-29 的 0.5 m 八字包，offboard | 2026-09-29 | 未引用 | 39.2 s；老编号（session.json 记 scenario 4）；被 bag/5_traj3_lemniscate 取代；不参与主结论；含 fcu/log_3_2026-9-29-15-32-40.ulg |
| `bag/old/5_traj3_lemniscate` | 历史/被取代：2026-10-08 第一趟八字的空包（0 字节，rosbag2 起录几秒即死） | 2026-10-08 | 未引用 | 包为空但那一趟飞行成功；被同日第二趟 bag/5_traj3_lemniscate 取代；不参与主结论；含 fcu/log_2_未知日期.ulg |
| `bag/z0-params-2026-09-29.params` | 2026-09-29 的飞控参数原样导出（911 条；body 为 CRLF 且夹一处 GBK） | 2026-09-29 | 未引用 | 历史导出；被 2026-10-10 那份取代 |
| `bag/z0-params-2026-10-10.params` | 2026-10-10 的飞控参数原样导出（911 条，UTF-8 + LF） | 2026-10-10 | 未引用 | docs/assets/z0.params 的 body 来源 |

## 参数导出

| 路径 | 是什么 | 日期/版本 | 被哪一章引用 | 备注 |
|---|---|---|---|---|
| `docs/assets/z0.params` | z0 飞控参数快照（2026-10-10 现场 QGC 导出，911 条；文件头 + 原文件逐字内容） | 2026-10-10 | 第3、4、5、6、7、8章 | 报告按此文件引用参数现值 |

## 统计

- 覆盖文件行数：253 行（论文 7、演练日志 12、工具证据 216、录制包与说明 15、参数导出 3，其中 2 份参数导出位于 bag/）。
- 日期/版本标为「未标注」：1 行。
- 正文未引用：214 行。
- 目录级引用与文件级引用的区别写在「备注」列；目录整体被引用不等于每个文件被点名。
