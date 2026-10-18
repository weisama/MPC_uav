# 代码注释审计（Phase 5 票 17）

日期：2026-10-18
范围：报告各章引用到的 `src/{bench,bringup,control,mocap,nmpc,traj,vrpn_listener}` 源文件。
只改注释/文档字符串；不改行为、默认值、参数、话题名、代码标识符，不重构，不改测试断言。

## 一、引用范围的取法

在 `docs/report/chapters/*.md` 中按**完整路径**与**文件名/可执行名**两种写法检索，
取 `src/` 下被点到的实现文件。测试文件虽在正文中被点名（第 6、8 章），本票不纳入修改
（票面禁止改断言；且它们的注释是测试实现的一部分）。

| 文件 | 来源章 | 报告中的引用形式 |
|---|---|---|
| `src/bench/bench/pytest_support.py` | 06 | 完整路径 |
| `src/bringup/launch/z0.launch.py` | 06 | 完整路径 |
| `src/bringup/launch/mavros.launch.py` | 06 | 文件名 |
| `src/bringup/scripts/fly.py` | 05、06、07 | 文件名 |
| `src/bringup/scripts/mavros_set_thrust_scaling.py` | 06 | 文件名 |
| `src/control/launch/control.launch.py` | 06 | 文件名 |
| `src/control/include/control/fsm.hpp` | 04、05、06、07 | 完整路径 |
| `src/control/include/control/px4_interface.hpp` | 05 | 完整路径 |
| `src/control/include/control/rc_switch.hpp` | 07 | 完整路径/文件名 |
| `src/control/include/control/rc_stick.hpp` | 06 | 文件名 |
| `src/control/src/px4_interface.cpp` | 05 | 完整路径 |
| `src/control/src/control_node.cpp` | 06 | 可执行名 `control_node` |
| `src/mocap/src/mocap_rel_pose_node.cpp` | 06 | 可执行名 `mocap_rel_pose_node` |
| `src/mocap/launch/mocap.launch.py` | 06 | 文件名 |
| `src/nmpc/nmpc/__init__.py` | 06 | 完整路径 |
| `src/nmpc/nmpc/model.py` | 04、08 | 完整路径/文件名 |
| `src/nmpc/nmpc/mpc_node.py` | 05、06、08 | 完整路径/文件名 |
| `src/nmpc/nmpc/node_core.py` | 04、05、06、08 | 完整路径/文件名 |
| `src/nmpc/nmpc/ocp.py` | 08 | 完整路径/文件名 |
| `src/nmpc/nmpc/params.py` | 03、08 | 完整路径/文件名 |
| `src/nmpc/nmpc/sign_map.py` | 05、06、08 | 完整路径/文件名 |
| `src/nmpc/nmpc/commands.py` | 06 | 文件名 |
| `src/nmpc/nmpc/offboard_smoke.py` | 06、08 | 文件名 |
| `src/nmpc/nmpc/preflight_check.py` | 06、08 | 文件名 |
| `src/traj/src/traj_node.cpp` | 05 | 完整路径 |
| `src/traj/config/traj.yaml` | 05、09 | 完整路径 |
| `src/traj/launch/traj.launch.py` | 06 | 文件名 |

`vrpn_listener` 只在报告中以包名出现，没有点到具体源文件；且该包是 vendored、有 sha256 锁定
（`src/vrpn_listener/VENDORED.md`），**未修改**。

## 二、逐文件审计

| 文件 | 现状/问题 | 改动 | 证据 |
|---|---|---|---|
| `src/control/include/control/fsm.hpp` | ① `traj_node.cpp:289` 已不在 `running_` 判据上；② `traj_node.cpp:302-308` 已不在"跑完仍按 50 Hz 发"那段；③ "三条轨迹"与 `trajectories.hpp:148` 均过期（现有四条轨迹；夹取在 `:146` 与 `:424`）；④ `px4_interface.cpp:160-164` 已不在清归属标记处；⑤ 两处 "ticket 09 会…" 是将来时，功能早已实现 | `:289`→`:320-322`；`:302-308`→`:328-331`；`三条`→`四条`、`:148`→`:146,424`；`:160-164`→`:242-244`；"会用/会让…来调它"→"起也由/起…也调它" | `traj_node.cpp` 当前 `if (!running_) return;` 在 320–322、每拍发布在 328–331；`trajectories.hpp:146`/`:424` 是非正时长夹取；`px4_interface.cpp:242-244` 是 `arming_by_us_ = false`；`control_node.cpp:29-31` 已写明 ch9 是现场入口 |
| `src/control/src/control_node.cpp` | ① `takeoff_service_name_` 的注释仍写 "ticket 09 会…"，是将来时；② 进度心跳注释仍按"三条轨迹"举例 | ①改为 "ticket 09 起也会…"；②改为"四条轨迹的时长是 60 s 级（悬停，起飞沿用同一时长）与几十秒（方形/8 字）" | ①同文件 `:29-31` 已记载 ch9 上升沿已实现；②`traj.yaml` 的 `select_topic` 注释列出 1–4，`trajectories.hpp:474-485` 建四条 |
| `src/nmpc/nmpc/__init__.py` | `params` 行仍写"实测值 + 占位值" | 改为"实测值 / 已对账的飞控参数 / 引数；**无占位值**，2026-09-29 起" | `src/nmpc/nmpc/params.py:10`："✅ 2026-09-29 已全部实测…本表已无占位" |
| `src/nmpc/nmpc/mpc_node.py` | ① 三处把动捕 pos 订阅行写成 `mocap_rel_pose_node.cpp:110`（该行不是 QoS 订阅）；② 两处把 `/traj/reference` 的 QoS 写成 `traj_node.cpp:108-109`（该处是 `warn_about_parameters`）；③ 一处 `control/README.md:119` 不在 `traj_stream_gap_s` 那一行 | ①→`:120`（PoseStamped `QoS(10)`）；②→`:112-113`（发布者构造）；③→`:141` | `mocap_rel_pose_node.cpp:120-121`（pose）/`:123-124`（twist）；`traj_node.cpp:112-113`；`control/README.md:141` 是 `traj_stream_gap_s` 行 |
| `src/nmpc/nmpc/node_core.py` | ① 两处 `control.yaml:120-125`；② `fake_px4.py:50/:52`；③ 一处 `control/README.md:119`；④ `control/README.md:402`；⑤ `traj_node.cpp:325`；⑥ `mocap_rel_pose_node.cpp:110`；⑦ `vrpn_listener.hpp:12-20` | ①→`:142`；②→`:56-60`；③→`:141`；④→`:429`；⑤→`:356`；⑥→`:120`；⑦→`:16-24` | `control.yaml:142` 含 `min 0.998 / max 1.001`；`fake_px4.py:56-60` 是枚举；README `:141`/`:429`；`traj_node.cpp:356` 是"忽略字段留 0"的注释；`mocap_rel_pose_node.cpp:120`；`vrpn_listener.hpp:16-24` 是 `Synchronizer` 结构体 |
| `src/nmpc/nmpc/offboard_smoke.py` | ① `px4_interface.hpp:117-119`；② `px4_interface.cpp:177`；③ `px4_interface.cpp:546-547` | ①→`:127-130`；②→`:222`；③→`:643` | `px4_interface.hpp:129` 是 `offboard{"OFFBOARD"}`；`px4_interface.cpp:222` 是 arming client；`:643` 是 `base_mode = 0` 及注释 |
| `src/nmpc/nmpc/params.py` | ① 只说归一化常数在 `px4_rates_setpoint(..., t_max_n)`；换 mavros 载体后 `mavros_attitude_target` 也收同一个形参；② `node_core.py:387` 已不在 `params.f_max` 处 | ①改为"两个下行入口（`px4_rates_setpoint` / `mavros_attitude_target`）…形参 `t_max_n`"；②→`:460` | `sign_map.py` 的两个入口共用 `_checked_inputs`；`node_core.py:460` 是 `params.f_max` |
| `src/nmpc/nmpc/sign_map.py` | `node_core.py:423` 不是"故意重复"那条理由 | →`node_core.py:504` | `node_core.py:503-508` 是该理由所在段 |
| `src/traj/src/traj_node.cpp` | `describe_shape()` 的文档注释仍写"三条轨迹的尺寸差别"（现有四条） | "三条"→"四条" | `trajectories.hpp` 的 `Kind` 与工厂含 `kTakeoff`（`:474-485`）；`traj.yaml` 的 `select_topic` 注释列出 1–4 |

以下文件已逐行审计，未发现需要改的注释/文档字符串问题（`z0.launch.py` 运行期字符串的过期项
见 §三），**未改动**（"为什么"在文件头/关键分支上均已有，不重复添加无信息量的注释）：

`src/bench/bench/pytest_support.py`、`src/bringup/launch/z0.launch.py`、
`src/bringup/launch/mavros.launch.py`、`src/bringup/scripts/fly.py`、
`src/bringup/scripts/mavros_set_thrust_scaling.py`、`src/control/launch/control.launch.py`、
`src/control/include/control/px4_interface.hpp`、`src/control/include/control/rc_switch.hpp`、
`src/control/include/control/rc_stick.hpp`、`src/control/src/px4_interface.cpp`、
`src/mocap/src/mocap_rel_pose_node.cpp`、`src/mocap/launch/mocap.launch.py`、
`src/nmpc/nmpc/model.py`、`src/nmpc/nmpc/ocp.py`、`src/nmpc/nmpc/commands.py`、
`src/nmpc/nmpc/preflight_check.py`、`src/traj/config/traj.yaml`、
`src/traj/launch/traj.launch.py`。

其中 `mocap_rel_pose_node.cpp:89` 引用的 `vrpn_listener.cpp:139-144`、`:191` 引用的
`vrpn_listener.cpp:61` 已逐条核对，仍然正确；`traj_node.cpp:23` 引用的外部
`realflight_modules/px4ctrl/src/PX4CtrlFSM.cpp:202`/`:598-603` 也在
`/home/chen/refs/Fast-Drone-XI35` 的对应源码中核对过（分别是 `publish_trigger(...)`
调用与函数定义）。

**移植出处**：在相关文件的文件头/函数注释里已标注，本票复核后未发现缺失 ——
`traj_node.cpp` 标了 px4ctrl 的 `PX4CtrlFSM.cpp`、`rc_stick.hpp` 标了 PX4 的
`ManualControl.cpp` / `MovingDiff.hpp` / `AlphaFilter.hpp`、`sign_map.py` 与 `params.py`
标了参考实现的 `multirotor_rate_model.py` / 硬编码 `0.07`、下行字段则逐条标了 PX4 与
mavros 的源码位置。没有再补重复的出处注释。

## 三、记录在案但未改的过期断言

| 位置 | 过期内容 | 为什么没改 | 建议归属 |
|---|---|---|---|
| `src/bringup/launch/z0.launch.py:131` | 启动日志写"选轨迹（1/2/3）"，`fly.py` 现有四条（`KINDS` 含 4 = 起飞） | 这是运行时日志字符串，不是注释/文档字符串；本票只允许改注释 | 退给 `bringup` 的下一次触碰，或由 Phase 5 文档清扫统一处理 |
| `src/bench/bench/vrpn_source.py:15` | 引用 `mocap_rel_pose_node.cpp:110`，实际发布者在 `:120` | 该文件未被报告引用（报告只引用了 `bench/bench/pytest_support.py`），超出本票范围 | 下一次 `bench` 注释维护 |
| `src/control/test/test_control_failsafe.py:679` | 引用 `px4_interface.cpp:160-164`，实际在 `:242-244` | 测试文件不在本票范围；票面也禁止改测试断言（此处虽只是注释） | 测试维护时顺手修 |
| `src/nmpc/test/test_mpc_node.py:2259` | 引用 `control/README.md:119`，实际在 `:141` | 同上 | 测试维护时顺手修 |
| `src/nmpc/test/test_mpc_node.py:2637` | 引用 `mocap_rel_pose_node.cpp:110`，实际在 `:120` | 同上 | 测试维护时顺手修 |
| `src/nmpc/test/test_node_core.py:461,486` | 引用 `control.yaml:120-125`，实际在 `:142` | 同上 | 测试维护时顺手修 |
| `src/nmpc/nmpc/__init__.py` 模块表 | 表中未列 `node_core`、`mpc_node`、`preflight_check` | 属"表格不完整"而非错误断言；同一 docstring 的 ☠️ 段已点名后两者，`node_core` 在正文中另有说明。补行会把审计扩成结构改写，故只记录 | 若报告第 6 章需要完整的模块表，再按正文口径补 |

## 四、验证

- `git diff` 逐行核对：所有 `+/-` 行都是 `//`、`///`、`#`、`#:` 或模块 docstring 内的行，
  没有代码/参数/字符串字面量改动（`z0.launch.py` 的日志字符串改动已撤回）。
- `git diff --numstat` 每个文件的增删行数相等（7/7、2/2、1/1、6/6、8/8、3/3、3/3、1/1、1/1）
  ⟹ 没有插入或删除整行，**没有移动任何被其它文档引用的行号**。
- 构建（2026-10-18，仓库根）：
  ```bash
  source /opt/ros/humble/setup.bash
  source ~/mavros_ws/install/setup.bash
  source ~/px4_msgs_ws/install/setup.bash
  colcon build
  ```
  结果：`Summary: 7 packages finished [1min 19s]`，退出码 0；无 warning/error。
- 构建（2026-10-18 收口轮重跑，同上四条 source）：`Summary: 7 packages finished [1min 25s]`，
  退出码 0。
- 测试（2026-10-18，补 source `install/setup.bash` 后重跑）：
  ```bash
  source /opt/ros/humble/setup.bash
  source ~/mavros_ws/install/setup.bash
  source ~/px4_msgs_ws/install/setup.bash
  source install/setup.bash
  export ACADOS_SOURCE_DIR=$HOME/acados
  export LD_LIBRARY_PATH=$ACADOS_SOURCE_DIR/lib:$LD_LIBRARY_PATH
  python3 -m pytest src/nmpc/test tools/report/test_report_data.py -q
  ```
  结果：**`338 passed, 233 warnings in 570.19s`**，退出码 0。
  首次未 source `install/setup.bash` 时在 `src/nmpc/test/conftest.py:33` 报
  `ModuleNotFoundError: No module named 'nmpc'`；这是环境问题，不是代码失败。
