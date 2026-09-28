# bench —— 台架夹具

**它不是飞行代码，是测试基础设施。** 给节点级测试（seam 1）提供"外部世界"：

```
        ┌─────────────── 本包（一个 pytest 进程内的夹具节点）───────────────┐
        │  FakeMavros  ── 发 /mavros/*（/mavros/state 1 Hz、local_position、rc、battery、estimator_status）
        │              ── 收 /mavros/setpoint_raw/local 并逐帧记录（含到达时刻）
        │              ── 服务 /mavros/set_mode、/mavros/cmd/arming
        │  FakePx4     ── 假 PX4（uXRCE-DDS 侧，`px4=True`，Phase 2）：发 /fmu/out/vehicle_status
        │              ── 收 /fmu/in/vehicle_rates_setpoint 与 /fmu/in/offboard_control_mode 并逐帧记录
        │  VrpnSource  ── 脚本化发 /mocap/pose_enu（或上游 /vrpn/<刚体>/pose）
        │  FakeTraj   ── 假 traj：收 /traj/select + /traj/start，发 /traj/reference + /traj/progress
        │  Timeline    ── "第几秒掐掉哪条话题 / 改哪个状态"
        └───────────────────────────────┬───────────────────────────────────┘
                                        │ DDS（独立 domain）
                              ┌─────────▼─────────┐
                              │  DUT 子进程（真节点）│
                              └───────────────────┘
```

存在的理由：状态机里那些**实飞才会想到的判据**（`/mavros/state` 静默、时间戳倒退、电压跌落、
服务不应答）**真 SITL 无法注入**，而它们恰恰是最容易写错、也最难在真机上复现的部分。
它是 Phase 1/2/3 所有节点级测试的公共底座。

## ★ 它假的是什么、不假什么

| 假 | 不假 |
|---|---|
| **mavros 的行为**：话题名、频率、**QoS**、`/mavros/state` 1 Hz、`mode_sent` 的语义、服务的应答、`/mavros/estimator_status` 自报的 `pos_horiz_abs` | **PX4 的行为**：`COM_OF_LOSS_T` 超时退 OFFBOARD、EKF2 融合、模式需求检查 |
| **PX4 的行为**（`FakePx4`，ticket 06，仅 `Scenario(px4=True)` 时起）：`/fmu/out/vehicle_status` 的**话题名、QoS、字段语义、2 Hz 定频 + 变化即刻发**；`/fmu/in/*` 的**订阅侧** QoS（决定 DUT 的发布方接不接得上）；`armed` / `nav_state` / `pre_flight_checks_pass` 的**注入**；PX4 反序列化器对 `timestamp` 的读法（`stamp_age_s()` 把它**报告**成一个有物理含义的数，**不据此行动**） | **飞控内部的一切**：夹具只把 DUT 发来的 `/fmu/in/*` 帧**记进列表**（`_on_frame`），没有任何动力学 ⟹ **姿态环 / 混控 / 位置控制都不假**。同样不假：`COM_OF_LOSS_T` 超时退 OFFBOARD、EKF2 **怎么**融合、模式需求检查、prearm 的**具体**检查项、时钟同步的真实滞后 Δ、`vehicle_command` 的处理（MPC（模型预测控制）节点是纯控制器，不发命令） |
| EKF2 的**位置输出**（`local_position`，可被 `set_position` 搬动） | EKF2 **怎么算出**这个位置 |
| EKF2 的**速度输出**（`local_position/velocity_local`，可被 `set_velocity` 搬动；与位置**解耦**，两条独立话题） | 速度**怎么融合**出来的（TRIAD/差动，与它的噪声） |
| PX4 **接受 SET_MODE 之后 mode 会变**（`follow_set_mode`，默认开） | PX4 **为什么可能拒绝**某个模式 |
| PX4 **解锁服务的应答**（`set_arming_policy`）与**解锁之后 `armed` 才上升沿**（`follow_arming`） | PX4 的 **prearm 检查**（哪种检查不合格对应哪个 `MAV_RESULT` 码） |
| **`traj` 的接口与时钟**（`FakeTraj`，ticket 09）：没 `select`+`start` 不发、跑完停在 1.0、`select` 静默停流并归零 | **轨迹的几何与运动学**（三条轨迹长什么样、弧长匀速）—— 那是 `traj` 自己的 seam 1 测试 |

⟹ **依赖 PX4 侧行为的验收项不能在夹具里做**（例如"杀节点 → PX4 自行退出 OFFBOARD"），
那是 SITL / 拆桨台架的职责。夹具只能证到"节点确实停止发 setpoint 了"。

★ **上面第一行与第二行的口径不冲突，是分工**：`FakeMavros` 那行右列的"不假 PX4 的行为"指的是
**飞控内部**的控制/融合逻辑；`FakePx4` 那行假的是 PX4 **暴露给 uXRCE-DDS 的接口层**
（话题名 / QoS / 字段语义 / 出版节奏 / 注入点），它没有任何动力学模型 —— (C) 级 CTBR 下
PX4 收到的是「总推力 + 机体角速率」，夹具只照抄这个接缝的**名字与 QoS**
（见 [`src/nmpc/package.xml`](../nmpc/package.xml) 的 `<description>`）。

> 📌 **夹具假的是哪个 mavros 节点**：mavros 2.15.x 是 **router 架构** —— `mavros_router`
> 持链路，`mavros` 只是 UAS 节点（`mavros_node.cpp:77-79`，第一手核实见
> [`docs/reference/mocap-setup.md`](../../docs/reference/mocap-setup.md) §5.4 的坑 1）。
> 夹具假的是**后者**：话题名全挂在 `/mavros/...` 下，与 DUT 看到的完全一致；
> `mavros_router` **不假**（DUT 不碰它）。
> 台架跑 OFFBOARD 时若想用 QGC 旁观，按 §5.4 给真 mavros 热加一条 GCS 链路即可。

★★ **QoS 是这里最容易被低估的部分**。每一条都对着真 mavros 2.15.1 的源码抄（见
`bench/fake_mavros.py` 文件头的对照表与 `include/control/px4_interface.hpp`）：
`local_position` / `battery` 是 `SensorDataQoS()`（**best_effort**），`/mavros/state` 是
`QoS(10).transient_local()`。**QoS 不兼容不会报错，只是永远不匹配** ——
`ros2 topic list`、`ros2 topic hz`、日志全都看不出原因。
`test/test_bench_selftest.py` 里有一条专门的回归测试盯着这件事。

★★ **但黑盒读不回 QoS 的哪两项**（本机实测，2026-09-28，Humble + rmw_fastrtps）：
**DDS 发现信息**里的 `history` 回来是 `UNKNOWN(3)`、`depth` **恒为 0**，且与发布者**实际写的值无关**
（`depth=1` 与 `depth=10` 读回来一模一样）⟹ 黑盒**只能断言 `reliability` / `durability`**。
这一条写在 `bench/fake_px4.py` 的 `publisher_qos()` docstring 里，
并被 `test/test_fake_px4_selftest.py::test_发现信息里读不回来的那两项_history_与_depth`
钉成会红的一行 —— 量具自己坏了必须立刻看得出来。
⚠️ 谁拿 `depth` 下断言，谁就会得到一个**恒红**的测试（而它与 DUT 毫无关系）。

★ 同一条实测的第二个推论：**`TRANSIENT_LOCAL` 的历史重放不发生** —— 晚加入的 durable 订阅者
**匹配得上**（发布者视角 `get_subscription_count() == 1`）、**新**帧也收得到，但**收不到任何旧帧**。
⟹ 连"拿重放间接反推 `depth` 语义"这条路也是堵死的（同一份实测见
[`src/nmpc/test/test_mpc_node.py`](../nmpc/test/test_mpc_node.py) 的
`test_发现信息里的_depth_读不回来_所以票面那条只能靠读常量`）。

## ★ 安全边界（不是洁癖）

本机 **domain 0 上跑着真的 mavros**（对着真飞控、`/dev/ttyTHS1`），而夹具会提供
**`/mavros/cmd/arming`**。跑进同一个域 ⟹ 测试里的"假解锁"可能打到真飞机上。所以：

- 默认 **`ROS_DOMAIN_ID=73`**（并行跑多份测试时用 `BENCH_ROS_DOMAIN_ID` 覆盖）
- **拒绝 domain 0**（`init_ros_for_tests()` 与 `Scenario.__enter__` 都硬失败，不给"我以为没事"留口子）

★★ **同一个域里只能有一份夹具** —— 这条是 2026-09-22 用一次事故换来的，症状极具误导性：

两份夹具（或"夹具 + 别包的夹具"）并存时，**两边的假 mavros 会同时往 `/mavros/state` 等话题上发**，
订阅方（DUT）收到的是**两股帧的混合** ⟹ `armed` 抖动、`mode` 乱跳、发布者计数变 2。
表现出来是**随机假红**（守卫被别的夹具的帧打翻）或**随机假绿**（判据被别的夹具的帧一直喂饱），
而日志里 DUT 自己的行为**完全正常** —— 你会去查节点，而罪魁在测试进程里。

第一手证据（同一天，同一份代码）：整仓 `colcon test` 默认**并行跑包**（`bench` 与 `control` 同时在域 73），
`control` 的 `test_control_standby` **9 条挂了 5 条**（清一色 `MANUAL/STANDBY` + "没在发 setpoint"）；
把同一份测试**单独跑在域 75** ⟹ **9 条全绿**。`test_bench_selftest` 的 `test_state_是_1Hz`
（"3.4 s 内 2–3 帧"）也在同样的场面下变成 7 帧。

做法：**每份并发运行的测试各自一个域**。
- 手动跑单文件：`BENCH_ROS_DOMAIN_ID=75 python3 -m pytest src/control/test/...`（域号随便挑，别用 0/73）
- 整仓跑：`colcon test --executor sequential`（串行跑包 ⟹ 同一时刻只有一个包在用域 73）。
  ⚠️ 用 `BENCH_ROS_DOMAIN_ID=<n>` 跑**整仓**并**不能**解决这个问题：那个变量对**所有**包都生效，
  两个包拿到同一个域，撞法不变。
- 这套"到底该给谁哪个域"的根治办法**还没做**（曾开过一张 ticket，2026-09-22 按"不影响飞行"删票归档，
  `git log --diff-filter=D -- .scratch/phase1/issues/17-*.md` 可取回）。现状就是上面这两条约定：
  单文件手动挑域号、整仓加 `--executor sequential`。
  ⚠️ 两处已知挡不住的场面：**同一包并发两轮**（两个会话）、以及**忘了加 `--executor sequential`** ——
  症状是随机假红/假绿，而 DUT 自己的日志完全正常（排查方向天然指错）。
- **`ROS_LOCALHOST_ONLY=1`**（本机多网卡：动捕 WiFi + 校园网，别把测试组播漏出去）

#### ★ 已知的间歇抖动：`test_control_failsafe`（**不阻塞，别再追**）

**2026-09-23 的整仓 `colcon test --executor sequential` 里，`control` 挂过 1 例** ——
`test_control_failsafe.py::test_动捕时间戳冻住会降级_而短促倒退不会` 的**后半**：
`ASSERT wait_for_control_state("MANUAL/LAND", 6.0)` 超时，DUT 实际停在 `AUTO/TAKEOFF` 一动不动
（`build/control/Testing/Temporary/LastTest_20260923-0346.log:409` 起）。

**判定：整仓并发下的抖动，不是缺陷**（三条第一手证据）：

1. **单独跑必过** —— 同文件 `ROS_DOMAIN_ID=73 pytest src/control/test/test_control_failsafe.py`
   ⟹ `19 passed`；只跑那一条 ⟹ `1 passed`。
2. **整包 ctest 也过** —— `ctest --test-dir build/control` ⟹ `7/7 全过`（含它自己的 172.61 s）
   ⟹ 差异不在 ctest 路径，在"整仓并发"。
3. **它在本票之前就在抖** —— `LastTestsFailed_*.log` 里 `test_control_failsafe` 在 **2026-09-22
   出现过 8 次**（0338/0355/0357/0400/0654/0734/1106/1115），那时相关代码一行都还没写。
   同一天 `test_control_standby` 4 次、`test_fsm` 8 次，也都是整仓跑时的抖动。

**症状的读法**（供将来真出问题时定位）：DUT 在 6 s 内**既没降级也没超时**、
停在 `AUTO/TAKEOFF` 不动 ⟹ 它看见的动捕戳**一直在前进** ⟹ 要么冻住那次注入没生效
（timeline 事件没被触发/被推后），要么"历史最大戳"与冻结值的关系在负载下走了另一条分支。

★ **2026-09-28 操作者裁定：不开独立票** —— 证据与读法就写在这里，
[`07`](../../.scratch/phase1/issues/07-traj-reference-trajectories.md) 的那个"整仓全绿"框按本条勾。
⚠️ 唯一**没做**的是"在无负载下复现"；`--executor sequential` + 单文件挑域号这两条约定照旧。
  ⚠️ 这一条**夹具与 DUT 必须取同一个值**。本机上 `ROS_LOCALHOST_ONLY=0` 是被人设过的；
  夹具若只 `setdefault`、DUT 却强设，两边就会用不同的 DDS 传输配置 ——
  表现为"两个进程都活着，但彼此发现不了"，且**没有任何报错**。所以两边都用常量 `LOCALHOST_ONLY` 硬写。
- DUT 子进程**显式传 `env=`**，不靠"碰巧继承了环境"

## 怎么用

### 最小例子

```python
# src/<你的包>/test/conftest.py —— 会话级，一个进程一次
import pytest
from bench.pytest_support import init_ros_for_tests, shutdown_ros_after_tests

@pytest.fixture(scope="session", autouse=True)
def _ros():
    init_ros_for_tests()
    yield
    shutdown_ros_after_tests()
```

```python
from bench import Scenario
from bench.timeline import silence_for

def test_飞控状态静默之后停止发_setpoint():
    with Scenario(dut=("control", "control_node"),
                  params={"offboard_prestream_s": 2.0}) as sc:
        # ★ 任何"没收到"的断言之前，先过发现屏障
        assert sc.wait_for_subscribers("state")
        assert sc.wait_for_setpoints(count=20, timeout_s=15.0)

        sc.timeline.extend(silence_for(sc.fake, "state", at_s=sc.t() + 1.0, duration_s=3.0))
        assert sc.wait_for_control_state("MANUAL/INIT", timeout_s=6.0)
```

### 一条完整的场景

```python
with Scenario(dut=("control", "control_node")) as sc:
    sc.wait_for_subscribers("state")
    sc.wait_for_subscribers("local_position")
    assert sc.mocap_source.matched_subscribers() >= 1     # 动捕那条也要接上
    sc.wait_for_setpoints(count=20)

    sc.fake.set_position(0.12, -0.34, 0.56)               # 搬动假 EKF
    sc.hold(0.5)                                          # 推进 0.5 s（★ 不要用 time.sleep）
    sp = sc.setpoints()[-1]
    assert (sp.msg.position.x, sp.msg.position.y, sp.msg.position.z) == (0.12, -0.34, 0.56)
    assert sp.msg.type_mask == 2552
    assert sc.control_state() == "MANUAL/STANDBY"
```

## API 速查

### `Scenario`

| 成员 | 说明 |
|---|---|
| `Scenario(dut, params, config, config_file, start_dut, mocap, dut_env, node_name)` | `dut=("包名","可执行名")` 或一个可执行文件路径，或 `None`（只起夹具）。`params` 会变成 `--ros-args -p k:=v` |
| `t()` | 秒表（相对**夹具就位**）。`Timeline` 的 `at_s`、记录的 `at_s` 都是这把尺子 |
| `advance_to(t_s)` / `hold(s)` | 推进时间并**沿途触发时序事件**。★ 不要用 `time.sleep` —— 事件不会被触发 |
| `fake` / `mocap_source` / `timeline` | 三个夹具对象 |
| `wait_for(pred, timeout_s, message)` | 等判据；超时抛 `BenchTimeout`，消息里带 `evidence()` |
| `poll(pred, timeout_s)` | `wait_for` 的不抛异常版（给"断言它**不**发生"用） |
| `wait_for_subscribers(key, count, timeout_s)` | 发现屏障 |
| `wait_for_setpoints(count, timeout_s)` | 等 DUT 发出至少 N 帧 setpoint |
| `wait_for_control_state(text, timeout_s)` | 等 `/control/state` 变成 `"MANUAL/STANDBY"` 这样的串 |
| `control_state()` / `control_state_history()` | 最近一次 / 全部 |
| `setpoints()` / `setpoint_rate_hz(from_s, to_s)` | DUT 发来的 setpoint 及窗口内的实测频率 |
| `dut_alive()` / `dut_returncode()` / `dut_log()` / `stop_dut(sig=SIGINT)` | DUT 子进程（SIGINT → SIGTERM → SIGKILL，整组发） |
| `evidence()` / `dump(path)` | 人话证据（DUT 状态、夹具记录、时序表、DUT 日志尾 25 行） |

### `FakeMavros`（`sc.fake`）

| 方法 | 说明 |
|---|---|
| `mute(*keys)` / `unmute(*keys)` | 让话题**停止发布**。可选 `state` `local_position` `local_velocity` `rc` `battery` |
| `set_state(**fields)` | 改 `connected` / `armed` / `mode` / `guided` / `manual_input` |
| `synth_disconnect()` / `reconnect_state(**fields)` | ★ ticket 18：**mavros 自己那帧合成断连状态** —— 发一帧 `connected=armed=guided=manual_input=False, mode=""` 然后**静默**（真 mavros 的 `publish_disconnection()`，`sys_status.cpp:903-911`，链路静默满 `conn_timeout` 后发出）；`reconnect_state` 是它的一对（改回字段 + 恢复发布）。★ 那帧的六个字段是**常量**（`connected`/`armed`/`guided`/`manual_input` 全 `False`、`mode=""`、`system_status=UNINIT`）、与夹具属性无关 —— 这就是"那帧的 `armed=false` 不是读数"在夹具里的落点 |
| `set_position(x, y, z)` | 搬动假 EKF（**ROS/ENU，z 向上为正**；⚠️ 不是 NED —— 见 `fake_mavros.py` 的 docstring） |
| `set_velocity(vx, vy, vz)` | 搬动假 EKF 的**速度**（ROS/ENU，与 `set_position` **互不影响** —— 它们是两条独立话题） |
| `set_yaw(yaw)` | 改假 EKF 报的**机头**（ROS/ENU，从 +x 逆时针）。★ ticket 08 加：此前 setpoint 的 yaw 是固定 0，夹具没有"会转的飞机"⟹"锁没锁住机头"在 0 上分不出来 |
| `set_voltage(v)` / `set_rc(channels)` / `set_rc_channel(ch, v)` | 电压、整组遥控通道、单路（**通道号从 1 起算**） |
| `set_mode_policy('ok'\|'fail'\|'absent')` | `set_mode` 应答：接受 / `mode_sent=False` / **服务不存在** |
| `set_arming_policy('ok'\|'fail'\|'absent')` | `/mavros/cmd/arming` 应答：`success=True,result=0` / `success=False,result=1` / **服务不存在**。★ `'fail'` 造的是"**服务当场说不**"（DUT 立刻知道被拒） |
| `follow_arming`（构造参数，默认 `true`） | `false` = **应答肯定但 `armed` 永不上升沿**。★ 这是**另一种**失效：真机上对应 prearm 检查在 ack 之后才失败 ⟹ DUT **只能等到 `takeoff_timeout_s` 超时**，与 `'fail'` 走的是两条不同的路径 |
| `set_mode_service_available(bool)` | 运行时增删服务（⚠️ 要等 DDS 发现传播，断言前用 `wait_for`） |
| `matched_subscribers(key)` | ★ 发现屏障：**QoS 不兼容时恒为 0**（服务没有对应的计数 API，见 `fake_mavros.py` 的更正） |
| `setpoints` / `mode_requests` / `arming_requests` | 记录 DUT **发来**的每一帧 / 每次 set_mode / 每次 arming（`ArmingRequest(at_s, value)`）。★ 解锁的负断言（"只请求一次"、"被拒后一帧都不补发"）就靠它 |
| `setpoints_between(from_s, to_s)` | 按窗口取 |

★ `matched_subscribers()` 是**最该先用的一个断言原语**：它把"没收到数据"一句话分成两种截然不同的原因 ——
`== 0` ⟹ 根本没接上（QoS 不兼容 / 话题名写错）；`> 0` 却收不到 ⟹ 才是 DUT 的逻辑问题。
没有这个数，两种情况在测试日志里长得一模一样。

### `FakePx4`（`sc.px4`，仅 `Scenario(px4=True)` 时存在，否则是 `None`）

Phase 2 的 `/fmu/*` 外部世界（ticket 06）。它**发** `/fmu/out/vehicle_status`、**收**
`/fmu/in/vehicle_rates_setpoint` 与 `/fmu/in/offboard_control_mode`；话题名的 key 分别是
`status` / `rates` / `mode`（`FakePx4.OUT_TOPICS` / `IN_TOPICS`，默认值在 `config/bench.yaml` 的 `px4:` 段）。

| 方法 | 说明 |
|---|---|
| `mute(*keys)` / `unmute(*keys)` | 让话题**停止发布**（不是"发空值"）。⚠️ 目前只有 `status` 可静默（`SILENCEABLE`），别的 key 抛 `ValueError` |
| `set_armed(bool)` / `set_nav_state(int)` / `set_pre_flight_checks_pass(bool)` | 三条注入面（改一个属性）。★ **下一拍（≤10 ms）就发出去** —— 夹具按 commander 主循环的 10 ms 跑，**不是**按 500 ms |
| `matched_subscribers(topic_key)` | 夹具**发**的那条（`status`）上已匹配的订阅者数。★ 发现屏障：**QoS 不兼容时恒为 0** |
| `matched_publishers(topic_key)` | 反方向的发现屏障：DUT 在那条 `/fmu/in/*` 上有几个发布者已匹配（两者都是 0 才说明 DDS 根本没发现） |
| `wait_for_publishers(topic_key, count=1, timeout_s=10.0)` / `wait_for_subscribers(topic_key, count=1, timeout_s=10.0)` | 上面两个的轮询版；**超时返回 `False`**（不抛） |
| `published_frames(topic_key)` | DUT 发到某条 `/fmu/in/*` 上的**全部帧**（只读快照，元素是 `DutFrame(at_s, msg, topic_key)`） |
| `frames_between(topic_key, from_s, to_s)` | 按窗口取（窗口两端用的都是记录的 `at_s`） |
| `rate_hz(topic_key, from_s=None, to_s=None)` | 窗口内的实测频率（★ 纪律同 `Scenario.setpoint_rate_hz`：**不要在相邻两帧的间隔上断言**） |
| `status_rate_hz(from_s=None, to_s=None)` | 夹具**自己**发状态的频率（默认应 ~2 Hz —— 真 PX4 的定频） |
| `publisher_qos(topic_key)` | DUT 发布方**实际提供**的 QoS（读不到返回 `None`，断言前先过 `wait_for_publishers`）。★★ **只能对 `reliability` / `durability` 下断言** |
| `stamp_age_s(frame, now_s=None)` | 那帧时间戳相对**墙钟**的"年龄" [s]（正 = 过去）。⚠️ `now_s` 是**墙钟秒**，**不是** `sc.t()` —— 它第一版取错过，于是正确的 µs 会被判成 `ns` |
| `stamp_unit_guess(topic_key="rates")` | 猜单位，返回 `(单位, 中位年龄秒)`：`"us"` / `"ms"` / `"ns"` / `"zero"` / `"stale"` / `"none"`。★ 它是**猜测**不是断言 —— 断言请用 `stamp_age_s()` |
| `status_frames` | 夹具**发出**的 `StatusFrame(at_s, timestamp_us, msg)` 列表 |
| `t()` / `hrt_us()` | 与 `Scenario` 同一把尺子 / 夹具这一侧的"PX4 时钟"（µs，从**夹具就位**起算 —— 与 DUT 的墙钟 µs 是**两个纪元**，相减没有意义） |
| `evidence()` | 人话证据（三个注入状态的当前值 + 两条 `/fmu/in/*` 的帧数/频率/单位猜测 + `status` 的匹配数） |

★ 与 `FakeMavros` 同一套纪律：**任何"没收到"的断言之前先过发现屏障**、"频率只在窗口上算"、
每条"不发生"的断言配一条正控。

### `VrpnSource`（`sc.mocap_source`）

| 成员 | 说明 |
|---|---|
| `target='vrpn'` | 发 `/vrpn/<刚体>/pose`（**上游**，`mocap` 节点在回路里）→ 测 `mocap`，或测整链 |
| `target='mocap'` | 直接发 `/mocap/pose_enu`（**下游**，旁路 `mocap`）→ 测 `control` |
| `pause()` / `resume()` | **停止发布**（"话题一条都没有"） |
| `hold_stamp(bool)` | 照发但**时间戳不前进**（mavros 会丢弃这类消息） |
| `rewind(seconds, hold_s)` | 时间戳**倒退**（造一次向后的跳变） |
| `set_pose(...)` / `set_trajectory(fn)` | 固定位姿 / `fn(t_s) -> (position, orientation)` |
| `matched_subscribers()` | 发现屏障 |

★ `pause` 与 `hold_stamp` 是**两种不同的失效**，别混：前者一条都不发；后者有消息但戳不动。
控制节点的判据只看"到没到"，而 mavros 的 vision_pose 会丢时间戳不前进的消息 ——
两者在 EKF2 眼里效果一样，在**话题层面完全不一样**。

### `Timeline` 与故障配方

`Timeline.add(at_s, action, label)` / `extend(events)` / `due(t)` / `pending()` / `describe()`。
配方都是"返回事件列表"的**纯函数**，挂上去即可：

```python
from bench.fake_px4 import NAV_STATE_OFFBOARD
from bench.timeline import (silence_for, state_change, voltage_drop, rc_change,
                            set_mode_policy, hold_mocap_stamp, pause_mocap,
                            px4_arm, px4_nav_state, px4_status_silence, px4_preflight_checks)

sc.timeline.extend(silence_for(sc.fake, "state", at_s=4.0, duration_s=3.0))
sc.timeline.extend(state_change(sc.fake, 6.0, connected=False))
sc.timeline.extend(voltage_drop(sc.fake, 7.0, 13.5))
sc.timeline.extend(rc_change(sc.fake, 8.0, channel=9, value=2000))
sc.timeline.extend(set_mode_policy(sc.fake, 9.0, "fail"))
sc.timeline.extend(hold_mocap_stamp(sc.mocap_source, 10.0))
sc.timeline.extend(pause_mocap(sc.mocap_source, 11.0, duration_s=2.0))
# 假 PX4（ticket 06）：`/fmu/*` 那一侧的注入 —— 全部**按时刻**注入，不在测试里直接调 setter
sc.timeline.extend(px4_arm(sc.px4, 12.0, armed=True))
sc.timeline.extend(px4_nav_state(sc.px4, 12.0, NAV_STATE_OFFBOARD))
sc.timeline.extend(px4_status_silence(sc.px4, 14.0, duration_s=3.0))  # 状态龄超阈值 ⟹ 闸门关
sc.timeline.extend(px4_preflight_checks(sc.px4, 15.0, ok=False))
sc.timeline.add(sc.t() + 0.5, sc.stop_dut, "杀掉 DUT")
```

⚠️ 假 PX4 那四个助手的 `at_s` 与 `sc.t()` 同尺；**事件在等待期间照常触发**
（`Scenario.wait_for` / `advance_to` / `hold` 都会 `_fire_due()`），所以"跑 3 s 之后注入"
不需要额外写轮询。★ 它们是 `px4_arm(…, armed=False)` 这种**状态设置**而不是"执行一个动作"，
名字读作"改假 PX4 的 armed 状态"（`armed=False` 是合法的，就是**上锁**）。

时序的错误（漏触发 / 重复触发 / 早触发）在集成测试里表现为"偶尔抽风"，
所以 `Timeline` 本身是**零 ROS 的纯逻辑**并被 `test/test_timeline.py` 单独钉死。

### ⏳ 还没被任何测试用过的能力

夹具比 ticket 05 的需求做得多 —— 下面这些**能跑但没人调过**，第一次用的人请**先写一条
自检**（`test/test_bench_selftest.py` 就是干这个的），别把"夹具没生效"当成"被测节点行为正确"：

| 能力 | 谁会用 |
|---|---|
| `VrpnSource.set_trajectory` | 仍然没人用（原以为会给 ticket 07/09，实际 ticket 09 用的是新加的 `FakeTraj`：轨迹跟踪要的是**参考位姿的流**，而不是"动捕跟着飞机动"） |
| `FakeMavros.set_arming_service_available()` | 暂时没人用（`set_arming_policy('absent')` 已覆盖"服务不存在"）；留着是为了与 `set_mode_service_available` 对称 |
| `Scenario(start_dut=False)`（只起夹具，手动起节点） | 台架排障时好用，自动化里没用过 |
| `px4_preflight_checks`（假 PX4 的第四个注入面） | **还没有读者**：`FakePx4.set_pre_flight_checks_pass` 自身有自检（真发到话题上），但被测节点**目前不看这一位**（闸门只看 `armed` + `nav_state`）⟹ 现在拿它写不出会红的断言。07/09 若开始看 prearm，再挂到 `control`/`nmpc` 的测试上 |

★ 反过来，`FakeTraj`（全部能力）/ `Plant` / `Scenario.wait_for_subscribers` 的"**DUT 已死**"分支
（它现在会把 DUT 的日志尾巴一起抛出来 —— 参数名/类型写错会让 `rclcpp` 构造参数时直接退进程，
而原来的报错长得像"话题名不对"）/ `mute` / `pause` / `set_position` / `set_state` / `wait_for_control_state` /
`setpoint_rate_hz` / `evidence` / `set_voltage` / `voltage_drop` / `set_rc` / `set_rc_channel` /
`rc_change` / `rewind` / `hold_stamp` / `set_mode_policy('absent')` / `synth_disconnect` /
`reconnect_state` 这些
**已被 `control` 的测试真实调用过**（ticket 05/06/09/18），可以照着抄。

★ ticket 08 新引入的三个注入面（`set_velocity` / `set_yaw` / `set_arming_policy`）**都已各有一条自检**，
并且**都已被 `control` 的测试真实调用**（`set_velocity` 与 `set_arming_policy` 在
`test_control_takeoff.py`、`set_yaw` 在 `test_control_standby.py`）⟹ 可以从 ② 那份清单里抄。
其中 `set_arming_policy('fail')` 与 `follow_arming=False` 是**两种不同的失效**（见 `FakeMavros` 表），
第一次用的时候别把它们当成同一件事。

★ **ticket 09 新加了两件"假体"，都按上面的纪律配了自检**：

| 假体 | 是什么 | 自检 |
|---|---|---|
| `Plant`（`bench/plant.py`，+ `fly()` / `fly_until()`） | ★ **不是夹具**，是**场景**逻辑（同 `Timeline`）：一架"速率受限地逼近 setpoint"的假飞机，速度 = 实际位移 / dt | 两条：**跟着 setpoint 动且速度是真的**、**只在有新 setpoint 时才动一格**（否则探针轮询比 DUT 的 50 Hz 快时会看到 0 与 2× 交替的假速度抖动） |
| `FakeTraj`（`bench/fake_traj.py`） | 假 `traj` 节点：收 `/traj/select` + `/traj/start`，发 `/traj/reference`（50 Hz）+ `/traj/progress` | 五条，逐条对着 `traj_node.cpp` 的**真实行为**：没 `start` 前一帧都不发、收到 `start` 后按 `duration` 推进完成度并**停在 1.0**、运行中重选**静默停流且时钟归零**、没选中轨迹时收到 `start` 不发但记一笔、**上一轮跑完留下的 1.0 能被造出来也能被 `start` 抹掉** |

⚠️ `Plant` 原本是 `control` 的 `test_control_takeoff.py` 文件末尾的私有 `_Plant`，**ticket 09 要复用它**，
于是按本文件的纪律**提升进了 `bench` 并补上自检**（原处那份当时还写着"ticket 09/10 若要复用，再提升" ——
那张纸条现在兑现了）。
★ 它住在 `bench` 但**不是夹具的一部分**这个区分很重要：夹具假的是"外部世界"，`Plant` 假的是
**这一轮的飞机**（同一个 DUT 在不同场景里配不同的飞机）。

⚠️ `FakeTraj` 的参考运动学是**假**的（它不做几何），它假的只是 `traj` 的**接口与时钟**；
"三条轨迹的几何对不对"归 `traj` 自己的 seam 1 测试，别在这里重复。

## 三条写测试的纪律

1. **任何"没收到 / 没发生"的断言之前，先过发现屏障**
   （`sc.wait_for_subscribers(key)`、`sc.mocap_source.matched_subscribers()`、`sc.wait_for_setpoints()`）。
   否则测试失败时你分不清"逻辑不对"与"DDS 还没发现"。
2. **不要在相邻两帧的间隔上断言频率**，只在**窗口**上算（`setpoint_rate_hz(from_s, to_s)`）。
   rcl 的定时器从**回调开始时刻**重新计时，回调偶尔超时就会立刻补跑 ⟹
   50 Hz 的流里出现 0 ms 的间隔与 40 ms 的空洞都是正常的，而平均频率仍然对。
3. **每条"不发生"的断言都要配一个正控**（先证明它在正常情况下**会**发生）。
   只有负断言的话，"测试通过"与"夹具根本没工作"长得一模一样。
   ★★ **正控自己也要过发现屏障 + 拿到基线帧**（2026-09-22 踩过两次，都是第一手的）：
      旁路订阅刚 `create_subscription` 时**还没匹配**（几十~上百 ms），而 `rewind(...)` 这类注入的窗口
      只有零点几秒 ⟹ 窗口会在订阅没接上时开完关完；**更隐蔽的是第二个坑**：即使匹配上了，
      若在**收到任何一帧之前**就注入，`rewind` 的那次**后退**在你这儿根本没有"前一帧"可比
      （探针实测：窗口里只剩 +5 ms 的小步进，只有窗口结束时的**前跳** +5.0 s 可见）
      ⟹ 正控会误报"注入没生效"，而注入其实是好的。
      写法见 `control/test/test_control_failsafe.py::_watch_mocap_stamps`：先等匹配数 +1，再等基线帧到齐。

## ★ 台架纪律（起 DUT 子进程的用例）

1. **收工前看一眼有没有残留的 DUT 子进程**：`pgrep -af mpc_node`。
   DUT 的话题名是**全固定**的（`/fmu/in/*`，见 `FakePx4.IN_TOPICS`），测试用的域号也是固定的
   （73）⟹ 上一个没杀干净的进程会与这一次**同时**往同两条话题上发，夹具的记录里就混进了别人的帧
   （症状是"帧数对不上 / 频率翻倍"，而断言本身照样绿或红得莫名其妙）。
   同一时刻两者还共用 `$MPC_CODEGEN_DIR`（560 KB 的生成代码）⟹ 两个进程同时判"要重新生成"
   会撞出一个与代码无关的 `RuntimeError: Rendering of model.in.h failed!`。
2. **不要用 `pytest -n`（xdist）**跑起子进程的用例 —— 同上：并发就地撞话题与生成目录。
   ★ 这条与第 1 条的第一手记录在 [`src/nmpc/test/test_mpc_node.py`](../nmpc/test/test_mpc_node.py)
   文件头（"别并发跑，也别把 DUT 落在后台"）。
3. **读 DUT 日志要等进程退出、读线程 join 完** —— `Scenario.stop_dut` 里的那条修复：
   进程退出 ⟹ 管道 EOF ⟹ 读线程随即结束，`join` 一下才返回；少了这一句，调用方紧跟的
   `dut_log()` 只拿到**前半段**（第一手：ticket 06 的收尾摘要明明打出来了、`returncode` 也是 0，
   日志里却只有开头那行 matplotlib 警告，看上去像"节点根本没打摘要"）。

## 配置

`config/bench.yaml`（安装到 `share/bench/config/bench.yaml`，用 `bench.config.load()` 读，
`Scenario` 默认就用它）：频率、话题名、初始状态、动捕刚体名、`scenario.start_dut` / `scenario.mocap` 开关。

★ 频率必须与实测一致：`state` 写的是 **1 Hz**、`vrpn` 是 **200 Hz**、`estimator_status` 是 **1 Hz**
（z0 实测 1.000 Hz，ticket 16）。
把 state 调快会让"解锁上升沿最多晚 1 s"这类真问题在测试里消失 —— **那等于用夹具说谎**。
⚠️ `estimator_status` 这一条尤其**不能**随手调快：DUT 的 `ekf2_pos_lost` 判据是"**连续 2 帧**"，
而 2 帧的真实代价是 ~2 s —— 调快会让那条判据看起来比真机灵得多。

## 构建与测试

```bash
colcon build --packages-up-to bench        # ★ 不要用 --packages-select：别的包 test_depend 了它
colcon test  --packages-up-to bench
```

夹具**自己也有测试**（`test/test_bench_selftest.py`）：验它**假得像不像** ——
频率对不对、`local_position` 是不是 best_effort、`mute` 是不是真的不发、`set_state` 是不是真的走到话题上、
`set_mode` 的三种应答，**ticket 06 新引入的四个注入面各一条**（`set_voltage`、`set_position(nan)`、
`set_rc_channel`/静置恒定、`hold_stamp`/`rewind`），以及 **ticket 08 的三个各一条**
（`set_velocity` 真的走到 `velocity_local` 上**且与 `set_position` 解耦**、`set_yaw` 真的改掉
`local_position` 的四元数**且符号对**（用 DUT 那套 `atan2` 反解出来对账）、
`set_arming_policy` 的 `'ok'`/`'fail'`/`follow_arming=False`/`'absent'` 四种形状**两两可区分**）。
**ticket 16 的两条**（`set_pos_horiz_abs`）：注入真的走到 `/mavros/estimator_status` 上且**只动那一位**
（其余位保持假，因为 DUT 只读那一位）、以及这条话题的**发布方 QoS 与真 mavros 一致**
（`mavros::StateQoS()` = 可靠 + **瞬态本地**；DUT 侧**故意用易失订阅** ⟹ 这条自检钉的是
"易失订阅者接得上、但不会拿到启动前的历史回放"这个性质 —— 见 `px4_interface.cpp` 那段 ★★）。
**ticket 09 的七条**：`Plant` 两条 + `FakeTraj` 五条（上面那张表逐条列了；`FakeTraj` 那五条的问法
是**对着 `traj_node.cpp` 的行为逐条抄的**，不是"它大概能发数据"）。
★ 这套自检里也补上了 `Scenario.wait_for_subscribers` 的**"DUT 已死"分支**（ticket 09 踩到：
传参类型不对 ⟹ 节点启动即退 ⟹ 报错长得像话题名不对）。
**ticket 18 的一条**（`synth_disconnect`）：复刻的"合成断连帧"**只发一帧**（发完把 `state` 静默，
因为真 mavros 只在收到 HEARTBEAT 时发这个话题），且那帧的六个字段是**常量**
`connected=armed=guided=manual_input=False, mode=""` —— 与夹具自己 `set_state` 写过的字段**无关**
（真 mavros 也看不到飞控，这就是"那帧的 `armed=false` 不是读数"在夹具里的落点）。
★ 这条自检必须带正控（先把 `armed=True` 放到话题上，再断言那帧是 `False`、且 `sc.fake.armed` 没被改），
否则"字段都是常量"在夹具根本没发帧时也会通过（负向断言在本包的纪律，见下面第 3 条）。
★ 理由：**夹具骗不到 DUT 的前提是它先骗过自己**；
一个把 `local_position` 发成 reliable 的假 mavros，会让**所有**依赖它的测试"莫名其妙收不到数据"，
而那种故障在日志里看不出来。

★ **ticket 06（Phase 2 的假 PX4）另起了一个文件**：`test/test_fake_px4_selftest.py`（**16 条**，
已在 `CMakeLists.txt` 里注册成 `test_fake_px4_selftest` ⟹ **`colcon test` 会跑它**——ticket 06 的验收项（"夹具能在 `colcon test` 里跑起来"）取的就是这一个文件的证；
且**全部不起 DUT** —— `start_dut=False`，需要"有个谁在发/在收"的地方由测试**自己**发，
这样夹具的量具 `publisher_qos` / `stamp_unit_guess` / `matched_*` 也被一起验了，而不是靠 DUT 恰好正确）：
两个 QoS 常量与 PX4 的 writer/reader **逐字段**一致、`vehicle_status` 真的是 2 Hz、注入的变化
≤0.1 s 就落地、`mute` 真的掐断又能恢复、四条注入面都落到话题上、夹具时钟与墙钟**是两个纪元**、
时间戳取证的三种单位 + `zero` / `stale` / `none`、`publisher_qos` **只断言得了两项**、
导航常量与 `px4_msgs` 里的一致。

⚠️ `bench` 是 Python 包：改 `bench/*.py` 之后**必须重新 `colcon build`**（代码被**复制**进
`install/bench/local/lib/python3.10/dist-packages/bench/`，**不是软链**；而 `source install/setup.bash`
把那个目录放进 `PYTHONPATH` ⟹ pytest 导入的是**安装副本**，不是 `src/bench/`），只改 `test/*.py` 则不用。
★ 这条 2026-09-22 咬过一次：`src/` 里已经修掉的 `FakeMavros.evidence()` 崩溃（`Service` 没有
`get_num_clients`）在**安装副本**里还活着 ⟹ 任何一次 `Scenario` 超时都变成 `AttributeError` 而不是打印证据，
"查不到证据"被误当成"夹具没记"。症状指向错误的方向，代价是几十行调试。

## 目录

```
bench/fake_mavros.py    ★ 假 mavros（QoS 对照表在文件头；记录 DUT 发来的每一帧）
bench/fake_px4.py       ★ 假 PX4（uXRCE-DDS 侧，ticket 06）：发 /fmu/out/vehicle_status、收 /fmu/in/*（仅 px4=True）
bench/vrpn_source.py    脚本化动捕（两种喂法 + 输入故障注入）
bench/plant.py          ★ 场景：假飞机（速率受限地逼近 setpoint）+ fly() / fly_until()
bench/fake_traj.py      假 traj：把 /traj/select + /traj/start 变成 50 Hz 的参考与完成度
bench/timeline.py       ★ 纯逻辑、零 ROS：事件时序 + 常用故障配方
bench/scenario.py       Scenario：拼装 + DUT 子进程 + 秒表 + 证据输出
bench/config.py         DEFAULTS 与 yaml 深合并
bench/pytest_support.py ★ 会话级 ROS 上下文与 domain 安全边界
config/bench.yaml       默认频率/话题/初始状态
test/                   夹具自检（26 条 + 假 PX4 的 16 条）+ 时序纯逻辑单测（6 条）
```
