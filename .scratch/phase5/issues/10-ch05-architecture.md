# 10: 第 5 章 系统架构与实验平台

Status: done
Type: task

**What to build:** `docs/report/chapters/05-architecture.md`。

**Blocked by:** —

## 内容

> **2026-10-18 操作者裁定（取代下面第 2 条）**：去掉「两条动捕路径」小节。其余条目不变。

- **三层架构**：定位层（Nokov → VRPN → mocap 节点）、机载层（ROS 2 + mavros + 节点）、飞控层（PX4）；
- **两条动捕路径**：EKF2 vision pose（安全网）与直连动捕（控制器状态源）；说清为什么两条都要；
- **硬件连接**：动捕主机 ↔ Jetson ↔ 飞控（Onboard 串口，mavros 占用）；遥控链路；
- **软件栈**：ROS 2 Humble、mavros 2.15.1、acados、PX4 1.14.3 系；
- **话题表**（正文的主干）：话题名、类型、频率（带测量时点）、生产者/消费者、坐标系；
  至少覆盖 `/traj/reference`、`/traj/progress`、`/traj/start`、`/control/state`、
  `/control/target`、`/mavros/setpoint_raw/local`、`/mavros/setpoint_raw/attitude`、
  `/mavros/vision_pose/pose`、`/mocap/pose_enu`；
- **数据流文字描述**：offboard 档 vs MPC 档的两条链，各自的控制量是什么；
- **安全边界**：谁解锁、谁切模式、谁停发 setpoint；PX4 的 offboard 失效保护；
- **不画装饰性架构图**（操作者已裁定不要）；用文字 + 表。

## 来源

- `docs/PROJECT.md` §3；
- `docs/control/phase1-implementation.md` §2–3；
- `docs/reference/px4-interface.md`；
- `docs/reference/mocap-setup.md`；
- `run/README.md`；
- `src/*/package.xml`、各包 README；
- 2026-10-17 六份包的数值探针（频率/坐标系）。

## 投影用一句话总结（2026-10-17 追加）

每个叶子内容小节（没有子标题的 `##`/`###`；独立「小结」节同样适用）**末尾**加一行，且只加一行：

`<big>**一句话总结：……**</big>`

- 一句话，建议 20–50 字、最多 60 字；只概括本节结论，能脱离上下文朗读。
- 不写“本节介绍……”；不引入本节之外的数、不引用图号/表号；不写“效果良好/基本通过”。
- 不单独成标题，不进目录。细则见 [spec.md](../spec.md) §8。

## 跨章阅读（2026-10-18 追加）

写作前**可以并建议**先读其他章节，尤其是相邻章和已落地章，用来对齐术语、符号、详略与过渡：

- 先读 `docs/report/chapters/` 下已存在的章节，再读本章票与相邻章的「内容」，明确本章边界；
- 术语/符号以 [CONTEXT.md](../../../CONTEXT.md) 为准；其他章已定下的写法优先沿用；
  发现冲突记进本票 `## Comments`，不改别人的章；
- 只借组织方式与措辞，不整段搬运；本章结论仍由本章自己给证据；
- 别的章还没写完时不等待、不阻塞；写完后再做一次交叉核对。

## 检查清单

- [ ] 写作前读过相邻章与已落地章；术语/符号与它们一致，冲突已记进 Comments

- [x] 每个叶子小节末尾有且只有一行 `<big>**一句话总结：……**</big>`，且句子不引入本节之外的数

- [x] 话题表的频率标注测量时点，不写"约 XX Hz"当永恒事实
- [x] 两条链的控制量写清：offboard = 位置 + yaw；MPC = 总推力 + 机体三轴角速率
- [x] 说清 uXRCE-DDS **不使用**、mavros 是唯一链路
- [x] 说清 FLU/FRD、ENU/NED 的转换发生在哪一侧，不重复第 8 章的推导
- [x] 不画架构图；不用 ASCII 大图代替文字
- [x] 中文母语写作，无裸词 `MPC`

## 验收

- 操作者审过才 `done`；审阅记录写进 `## Comments`。

## Comments

### 2026-10-17 —— 初稿完成，转 ready-for-human

**章节结构**（12 个叶子小节，每节末尾各一行 `<big>**一句话总结：……**</big>`）：

1. 5.1 符号与缩写（表 5-1）；5.2 三层架构（表 5-2）；5.3 两条动捕路径（表 5-3）；
2. 5.4 硬件连接（表 5-4）、软件栈（表 5-5）、为什么没有 uXRCE-DDS；
3. 5.5 话题表（表 5-6，16 行，九个指定话题全在）；
4. 5.6 offboard 档（位置 + yaw）与 MPC 档（总推力 + 机体三轴角速率）两条链；
5. 5.7 坐标系与转换发生在哪一侧（表 5-7）；5.8 安全边界（表 5-8）；5.9 本章引用清单（表 5-9，15 行）。

**话题频率快照**（2026-10-17 六份包，[`tools/report/topic_frequency.py`](../../../tools/report/topic_frequency.py) 重算，读数存 [`topic-frequency-20261017.json`](../../../tools/report/evidence/topic-frequency-20261017.json)；完整表见正文表 5-6）：

| 量 | 读数 | 口径 |
|---|---|---|
| 动捕/视觉位姿（`/vrpn/z0/pose`、`/mocap/pose_enu`、`/mavros/vision_pose/pose`） | 181–193 Hz | n/span；成簇到达，中位间隔会高估 |
| `/traj/reference`、`/traj/progress` | 中位 49.9–50.1 Hz，最大缺口 44–154 ms | 活跃期中位间隔 |
| `/mavros/setpoint_raw/attitude` | 98.8–99.8 Hz | n/span |
| `/mavros/setpoint_raw/local` | 活跃期 50 Hz；整包 40.9–50.0 Hz | LAND 停发 6.6–7.1 s |
| `/control/target` | 活跃期 50 Hz；整包 39.0–45.9 Hz | LAND 停发 7.2–7.9 s |
| EKF 局部位置/速度 | 29.9–30.1 Hz | 整包 |
| `/mavros/rc/in` / `/mavros/state` | 19.95–20.04 Hz / 1.00 Hz | 整包 |

这些是快照，不是规格；正文已把“量于何时、怎么量”写在表头与表后说明里。

**证据锚**（正文表 5-9 逐条给出路径；以下是审阅时最值得先核的几条）：

- 三层架构与两条链路解耦：`docs/PROJECT.md` §3；
- 话题语义、`type_mask`、坐标系：`docs/control/phase1-implementation.md` §2–3、`docs/reference/px4-interface.md`；
  `/mavros/setpoint_raw/local` 的 **2552** 与 `/control/target` 的 **2496** 分别核到
  `src/control/include/control/px4_interface.hpp` 的 `kTypeMaskPositionAndYaw` / `kTypeMaskPositionVelocityAndYaw`；
- MPC 输出：`src/nmpc/nmpc/mpc_node.py`（`/mavros/setpoint_raw/attitude`、`type_mask=128`、100 Hz）、
  `src/nmpc/nmpc/sign_map.py`（FLU 角速率原样、正号归一化推力）、`src/nmpc/nmpc/node_core.py`（`state_source=mocap`）；
- 安全边界：`run/README.md`、`src/control/include/control/fsm.hpp`、`docs/assets/z0.params`；
- 动捕两条路径与硬件：`docs/reference/mocap-setup.md`、`src/mocap/README.md`、`src/vrpn_listener/config/params.yaml`；
- 不使用 uXRCE-DDS：`.scratch/phase4/issues/13-mavros-downlink-switch.md`、`UXRCE_DDS_CFG=0` / `MAV_0_CONFIG=101` / `MAV_1_CONFIG=102` / `MAV_2_CONFIG=0`。

**未确定项 / 已知边界**：

- Nokov 服务端版本号未记录；正文只写“Nokov 配置说明的 200 Hz 出包率”并注明该配置值本身未测。
- 频率来自 2026-10-17 的六份包；不同场次、不同负载下会变，正文没有把它写成恒定规格。
- 两条路径的原点由不同环节定义（EKF2 局部原点 vs `mocap` 节点原点）。正文明确“不假设相同”，
  第 9 章按源各减各自原点；本章不替第 9 章做对齐结论。
- 第 3 章已展开的机体几何、推力标定、延迟与电池口径，本章只引用结论，不重复推导。
- ENU↔NED、FLU↔FRD 的公式与四元数写法留给第 8 章；本章只给“发生在哪一侧”的表。

**字数**：正文散文约 3.7 k 汉字（7.9 k 非空白字符）；含表格与引用清单共 5,314 汉字 / 17,781 非空白字符；
106 行表格；47 个仓库内链接全部可解析；裸词 `MPC` 仅出现 1 次（开头的定义处）。

### 2026-10-17 —— code review 后的修订

对初稿跑了一遍 code review，逐条回源码核过再改（核不实的保留原结论并在此说明）：

- **频率可复算性（真）**：初稿写“用 `extract.py` 重算”，但 `extract.py` 只读话题、不算频率。
  新增 `tools/report/topic_frequency.py`（只读包时间戳，不依赖 ROS）与 4 条合成用例；
  六份包的输出存 `tools/report/evidence/topic-frequency-20261017.json`。表 5-6 的每个区间都与 JSON 对得上
  （`/mavros/setpoint_raw/attitude` 98.82–99.77 Hz 也确实由包时间戳复现，不是 solver summary 的进程时长）。
- **先 OFFBOARD 后解锁（真）**：`fsm.hpp:1341` 在预流满 `offboard_prestream_s_` 后请求 OFFBOARD，
  `:1371` 才在 `in_offboard && rc_fresh` 下请求 arming；初稿把次序写反了，已改。
- **第 2 次 ch9 是松手生效（真）**：`note_start_release` 才兑现“欠一次开始”，上升沿只记待办；
  两档的流程描述都已改。
- **失效分支不是“只停发”（真，但分两类）**：`fsm.hpp:1289-1310` 在已解锁时对输入类失效请求 `AUTO.LAND`；
  打杆接管与模式被夺走 ④ 那条分支，只停发、不请求任何模式。5.6.1 第 9 步已按两类改。
- **MPC 关闸是停流（真）**：`node_core.py` 的 `_hold()` 返回 `publish=False`，不发布；
  初稿写“发零帧”，已改成“停流——不发布，也不冻结在最后一个目标”。
- **ch5 / ch6 是并行的硬件路径（真）**：`z0.params` 的 `RC_MAP_ARM_SW=5`、`RC_MAP_FLTMODE=6`；
  表 5-8 已补两行，并说明外部解锁会被 `kArmingByUs` 判为异常、`control` 停发。
- **长按 ch9 降落（真）**：`note_start_long_press` 与普通 LAND 动作相同；表 5-8 的 AUTO.LAND 行已补。
- **`/mavros/rc/in` 是原始 µs（真）**：`rc_switch.hpp` 用 `low_us=1000` / `high_us=2000` 在 `control` 侧归一化；
  表 5-6 已改。
- **`/mavros/state` 不随变化立即发（成立）**：它随 mavros 的心跳更新；初稿把两个 state 话题混在一起说，已拆开。
- **包集合（真）**：`bag/6_free__20261017-*` 会匹配到四份，正文已改成三个时刻并注明 `020757` 是轨迹 4 起飞段。
- **链接计数（真）**：初稿写 40，实际 47（修订后）；已改。

**审阅建议重点**：5.3 “两条都要”的论证是否成立；5.6 两条链的控制量是否与代码一致；
5.8 “谁能让飞机动”的边界是否还有漏项。请操作者审后把结论写在本节下方，再切 `done`。

### 2026-10-18 —— 按操作者裁定改结构

操作者指令：不要动既有的手改；去掉 5.3；5.6 的两条数据链改成对比表格，写明共有与区别，
区别用两张表分别展开。

已做：

- 删除原 5.3「两条动捕路径」整节（含原表 5-3）；后续小节与表号顺延：
  5.4→5.3、5.5→5.4、5.6→5.5、5.7→5.6、5.8→5.7、5.9→5.8；原表 5-4…5-9 → 表 5-3…5-11。
- 原 5.6 的 18 步文字改为三张表：
  - 表 5-6「两条链的共同点」：任务总管、轨迹源、操作流程、OFFBOARD 前置、轨迹原点、下行链路、结束与失效、互斥；
  - 表 5-7「offboard 档」与表 5-8「MPC 档」：同一组项目（谁发、下行话题、控制量、
    `type_mask`、指令频率、参考、状态反馈、PX4 承担、预流、停止条件），便于逐行对照。
- 章首「三个问题」改为「两个问题」，去掉已被删除的两条动捕路径。
- 未改动操作者的手改：5.2 的总结句、被删各节的解释段落、硬件表的表头分隔线等均保持原样。
- 操作者删掉了一些叶子小节的总结行（5.3.1、5.3.2、5.4、5.8）。按要求不恢复；
  若这些节仍要保留「每叶一行总结」的体例，请操作者示下。

当前字数：散文约 0.9 k 汉字（1.3 k 非空白字符）；含表格共 2,724 汉字 / 11,599 非空白字符；
136 行表格；38 个仓库内链接全部可解析；裸词 `MPC` 1 次。

注意：原定「两条动捕路径」内容已不在本章；章首与 5.2 表中仍有一处「两条动捕路径同时失去状态」
的措辞，未改，留给操作者决定是否恢复该节或改这句。上一节「审阅建议重点」里的 5.3 / 5.6 / 5.8
编号已被本次改结构取代（现在的 5.3 是硬件连接，5.5 是两条数据链，5.7 是安全边界）。

### 2026-10-18 —— 操作者裁定收工

操作者 2026-10-18 指令「全部 tickets 收工」，并对 README 修订与装配结果放行；本票按「做完了」义把 `Status` 转 `done`。收口过程见 [票 20](20-closeout-and-v5.md)。
