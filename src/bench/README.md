# bench —— 台架夹具

**它不是飞行代码，是测试基础设施。** 给节点级测试（seam 1）提供"外部世界"：

```
        ┌─────────────── 本包（一个 pytest 进程内的夹具节点）───────────────┐
        │  FakeMavros  ── 发 /mavros/*（/mavros/state 1 Hz、local_position、rc、battery）
        │              ── 收 /mavros/setpoint_raw/local 并逐帧记录（含到达时刻）
        │              ── 服务 /mavros/set_mode、/mavros/cmd/arming
        │  VrpnSource  ── 脚本化发 /mocap/pose_enu（或上游 /vrpn/<刚体>/pose）
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
| **mavros 的行为**：话题名、频率、**QoS**、`/mavros/state` 1 Hz、`mode_sent` 的语义、服务的应答 | **PX4 的行为**：`COM_OF_LOSS_T` 超时退 OFFBOARD、EKF2 融合、模式需求检查 |
| EKF2 的**位置输出**（`local_position`，可被 `set_position` 搬动） | EKF2 **怎么算出**这个位置 |
| PX4 **接受 SET_MODE 之后 mode 会变**（`follow_set_mode`，默认开） | PX4 **为什么可能拒绝**某个模式 |

⟹ **依赖 PX4 侧行为的验收项不能在夹具里做**（例如"杀节点 → PX4 自行退出 OFFBOARD"），
那是 SITL / 拆桨台架的职责。夹具只能证到"节点确实停止发 setpoint 了"。

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
- 这套"到底该给谁哪个域"的根治办法还没做，见 ticket 17。
- **`ROS_LOCALHOST_ONLY=1`**（本机多网卡：动捕 WiFi + 校园网，别把测试组播漏出去）
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
with Scenario(dut=("control", "control_node"), params={"yaw": 0.0}) as sc:
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
| `set_position(x, y, z)` | 搬动假 EKF（**ROS/ENU，z 向上为正**；⚠️ 不是 NED —— 见 `fake_mavros.py` 的 docstring） |
| `set_voltage(v)` / `set_rc(channels)` / `set_rc_channel(ch, v)` | 电压、整组遥控通道、单路（**通道号从 1 起算**） |
| `set_mode_policy('ok'\|'fail'\|'absent')` | `set_mode` 应答：接受 / `mode_sent=False` / **服务不存在** |
| `set_mode_service_available(bool)` | 运行时增删服务（⚠️ 要等 DDS 发现传播，断言前用 `wait_for`） |
| `matched_subscribers(key)` | ★ 发现屏障：**QoS 不兼容时恒为 0**（服务没有对应的计数 API，见 `fake_mavros.py` 的更正） |
| `setpoints` / `mode_requests` / `arming_requests` | 记录（`armings` 本张没用到，ticket 08 会用） |
| `setpoints_between(from_s, to_s)` | 按窗口取 |

★ `matched_subscribers()` 是**最该先用的一个断言原语**：它把"没收到数据"一句话分成两种截然不同的原因 ——
`== 0` ⟹ 根本没接上（QoS 不兼容 / 话题名写错）；`> 0` 却收不到 ⟹ 才是 DUT 的逻辑问题。
没有这个数，两种情况在测试日志里长得一模一样。

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
from bench.timeline import (silence_for, state_change, voltage_drop, rc_change,
                            set_mode_policy, hold_mocap_stamp, pause_mocap)

sc.timeline.extend(silence_for(sc.fake, "state", at_s=4.0, duration_s=3.0))
sc.timeline.extend(state_change(sc.fake, 6.0, connected=False))
sc.timeline.extend(voltage_drop(sc.fake, 7.0, 13.5))
sc.timeline.extend(rc_change(sc.fake, 8.0, channel=9, value=2000))
sc.timeline.extend(set_mode_policy(sc.fake, 9.0, "fail"))
sc.timeline.extend(hold_mocap_stamp(sc.mocap_source, 10.0))
sc.timeline.extend(pause_mocap(sc.mocap_source, 11.0, duration_s=2.0))
sc.timeline.add(sc.t() + 0.5, sc.stop_dut, "杀掉 DUT")
```

时序的错误（漏触发 / 重复触发 / 早触发）在集成测试里表现为"偶尔抽风"，
所以 `Timeline` 本身是**零 ROS 的纯逻辑**并被 `test/test_timeline.py` 单独钉死。

### ⏳ 还没被任何测试用过的能力

夹具比 ticket 05 的需求做得多 —— 下面这些**能跑但没人调过**，第一次用的人请**先写一条
自检**（`test/test_bench_selftest.py` 就是干这个的），别把"夹具没生效"当成"被测节点行为正确"：

| 能力 | 谁会用 |
|---|---|
| `arming_requests` | ticket 08（解锁） |
| `VrpnSource.set_trajectory` | ticket 07（参考轨迹跟随） |
| `Scenario(start_dut=False)`（只起夹具，手动起节点） | 台架排障时好用，自动化里没用过 |

★ 反过来，`mute` / `pause` / `set_position` / `set_state` / `wait_for_control_state` /
`setpoint_rate_hz` / `evidence` / `set_voltage` / `voltage_drop` / `set_rc` / `set_rc_channel` /
`rc_change` / `rewind` / `hold_stamp` / `set_mode_policy('absent')` 这些
**已被 `control` 的测试真实调用过**（ticket 05/06），可以照着抄。

## 三条写测试的纪律

1. **任何"没收到 / 没发生"的断言之前，先过发现屏障**
   （`sc.wait_for_subscribers(key)`、`sc.mocap_source.matched_subscribers()`、`sc.wait_for_setpoints()`）。
   否则测试失败时你分不清"逻辑不对"与"DDS 还没发现"。
2. **不要在相邻两帧的间隔上断言频率**，只在**窗口**上算（`setpoint_rate_hz(from_s, to_s)`）。
   rcl 的定时器从**回调开始时刻**重新计时，回调偶尔超时就会立刻补跑 ⟹
   50 Hz 的流里出现 0 ms 的间隔与 40 ms 的空洞都是正常的，而平均频率仍然对。
3. **每条"不发生"的断言都要配一个正控**（先证明它在正常情况下**会**发生）。
   只有负断言的话，"测试通过"与"夹具根本没工作"长得一模一样。

## 配置

`config/bench.yaml`（安装到 `share/bench/config/bench.yaml`，用 `bench.config.load()` 读，
`Scenario` 默认就用它）：频率、话题名、初始状态、动捕刚体名、`scenario.start_dut` / `scenario.mocap` 开关。

★ 频率必须与实测一致：`state` 写的是 **1 Hz**、`vrpn` 是 **200 Hz**。
把 state 调快会让"解锁上升沿最多晚 1 s"这类真问题在测试里消失 —— **那等于用夹具说谎**。

## 构建与测试

```bash
colcon build --packages-up-to bench        # ★ 不要用 --packages-select：别的包 test_depend 了它
colcon test  --packages-up-to bench
```

夹具**自己也有测试**（`test/test_bench_selftest.py`）：验它**假得像不像** ——
频率对不对、`local_position` 是不是 best_effort、`mute` 是不是真的不发、`set_state` 是不是真的走到话题上、
`set_mode` 的三种应答，以及 **ticket 06 新引入的四个注入面各一条**（`set_voltage`、`set_position(nan)`、
`set_rc_channel`/静置恒定、`hold_stamp`/`rewind`）。★ 理由：**夹具骗不到 DUT 的前提是它先骗过自己**；
一个把 `local_position` 发成 reliable 的假 mavros，会让**所有**依赖它的测试"莫名其妙收不到数据"，
而那种故障在日志里看不出来。

⚠️ `bench` 是 Python 包：改 `bench/*.py` 之后**必须重新 `colcon build`**（代码被**复制**进
`install/bench/local/lib/python3.10/dist-packages/bench/`，**不是软链**；而 `source install/setup.bash`
把那个目录放进 `PYTHONPATH` ⟹ pytest 导入的是**安装副本**，不是 `src/bench/`），只改 `test/*.py` 则不用。
★ 这条 2026-09-22 咬过一次：`src/` 里已经修掉的 `FakeMavros.evidence()` 崩溃（`Service` 没有
`get_num_clients`）在**安装副本**里还活着 ⟹ 任何一次 `Scenario` 超时都变成 `AttributeError` 而不是打印证据，
"查不到证据"被误当成"夹具没记"。症状指向错误的方向，代价是几十行调试。

## 目录

```
bench/fake_mavros.py    ★ 假 mavros（QoS 对照表在文件头；记录 DUT 发来的每一帧）
bench/vrpn_source.py    脚本化动捕（两种喂法 + 输入故障注入）
bench/timeline.py       ★ 纯逻辑、零 ROS：事件时序 + 常用故障配方
bench/scenario.py       Scenario：拼装 + DUT 子进程 + 秒表 + 证据输出
bench/config.py         DEFAULTS 与 yaml 深合并
bench/pytest_support.py ★ 会话级 ROS 上下文与 domain 安全边界
config/bench.yaml       默认频率/话题/初始状态
test/                   夹具自检（13 条）+ 时序纯逻辑单测（6 条）
```
