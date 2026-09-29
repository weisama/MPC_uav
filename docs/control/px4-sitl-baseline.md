# PX4 SITL 基线（ticket 01）—— 上游 v1.14.3 + `px4_msgs` + micro-XRCE-DDS Agent

> **这是 Phase 2 全部后续工作的地基**，也是 **Phase 3 重编固件要复用的构建环境**（Q37=a）。
> ⚠️ **一句话挡住最常见的误读：在这里编 SITL ≠ 刷飞控。** Q12 的「不刷固件」决策不变 ——
> 本页只编了一份给仿真器跑的 posix 二进制，飞控里烧着的仍是出厂那份
> `Minderring/PX4-Autopilot` @ `4a0e65f2`，**一个字都没动**。
>
> 可复现脚本：[`tools/px4-sitl/`](../../tools/px4-sitl/)（`build_px4_sitl.sh` / `build_xrce_agent.sh` /
> `run_sitl.sh` / `check_msg_compat.py`）。**本页只讲结论与理由，命令细节以脚本为准。**

---

## 1. 交付物一览

| | 版本 / commit | 路径 | 体积 |
|---|---|---|---|
| PX4 源码（SITL 被控对象） | tag **`v1.14.3`** → commit `1dacb4cdef2d7145754fc788fa8dc482eed74b40` | `~/refs/px4-v1.14.3-build/` | 2.2 GB |
| SITL 可执行文件 | 同上 | `…/build/px4_sitl_default/bin/px4`（43 MB） | — |
| 构建用 python 环境 | empy **3.3.4**（★ 见 §5 坑 2） | `~/refs/px4-v1.14.3-venv/` | 256 MB |
| **`px4_msgs`** | `release/1.14` @ **`ffb6e80e1c17e5714395611a020c282a87af8fa4`** | `~/refs/px4_msgs/`（源） · `~/px4_msgs_ws/`（colcon 工作区） | 122 MB |
| **micro-XRCE-DDS Agent** | tag `v2.4.2` → commit `57d086216d01ec43121845d385894a25987f8a2c` | `~/refs/micro-xrce-dds-agent/` → 装到 `/usr/local` | 411 MB |
| 官方比对脚本的**真身** | `auterion/px4-ros2-interface-lib` @ `ac2e8aeecb22de8efb29738b173e1ade5d54432b` | `~/refs/px4-ros2-interface-lib/` | 3.5 MB |

`sources` 一律在**仓库外**（Q10 / Q18=a）；构建产物也**不落进只读副本**。

> ⚠️ `~/refs/px4-v1.14.3-build` 用的是 `--single-branch`（只有 `v1.14.3` 这一支）。
> 17 个 submodule 全部按 tag 记录的 commit 初始化。**实测这样能编**，不必 full clone。

---

## 2. 四份第一手证据（本页的立足点）

### 2.1 消息定义：固件真身 vs `px4_msgs` vs 上游 v1.14.3 —— 三方逐 blob 比

用 `git ls-tree -r` 直接取 blob 哈希比对（不需要网络、**也不需要检出工作树** ——
真身树 `~/refs/px4-z0` 就是个只有 `.git` 的裸仓，`git cat-file -t 4a0e65f2` 确认提交在对象库里）：

| 对比 | 共有文件 | **blob 不同的** | 集合差异 |
|---|---|---|---|
| 固件真身 `4a0e65f2`(185) ↔ `px4_msgs` `ffb6e80e`(184) | **184** | **0** | 只有 `CMakeLists.txt`（在固件那侧） |
| 固件真身 `4a0e65f2`(185) ↔ 上游 `v1.14.3`(185) | **185** | **0** | **无** |

> 括号里的 185/184 是 **`msg/` 下的文件数，含 `CMakeLists.txt`**；其中 `.msg` 恰好是 **184** 个
> （`git ls-tree -r HEAD --name-only -- msg/ | grep -c '\.msg$'`）。
> 两张表的「只有 `CMakeLists.txt` 在固件那侧」是同一件事：`px4_msgs` 不带 CMakeLists。

✅ **这独立复现了 `refs-manifest.md` §2.6 执行版的两条结论**，且把第二条也补上了：
上游 v1.14.3 的 `msg/` 与固件真身**不仅共有文件相同，文件集合也相同**（185/185，连 `CMakeLists.txt` 都在）。
⟹ **拿上游 v1.14.3 当 SITL 被控对象，对 `px4_msgs` 兼容性而言与拿真身无差别。**（这正是 §2.6 取默认的前提）

### 2.2 逐类型字段布局：**全部 184 个**都相同

`tools/px4-sitl/check_msg_compat.py`（PASS，exit 0）：

```
共有       : 184 个 → 逐个比字段布局
字段布局不同的 : 0 个
解析失败的     : 0 个
只在 px4_msgs 里 : （无）
只在 PX4 树里    : （无）
    OffboardControlMode      在共有集里，字段一致      ← ticket 01 点名要覆盖
    VehicleRatesSetpoint     在共有集里，字段一致      ← ticket 01 点名要覆盖
RESULT: PASS
```

### 2.3 ★ 官方比对脚本：**文档里的路径是错的，直接用还会给假阴性**

ticket 01（与 `PROJECT.md:450`、`px4-interface.md:178`）写的是「PX4 官方脚本
`Tools/check-message-compatibility.py`」/`./scripts/…`，**这个文件在 PX4-Autopilot 里不存在**：

- `Tools/` 下只有 `generate_msg_docs.py` / `px_generate_uorb_topic_files.py` / `px_generate_uorb_topic_helper.py`；
  仓库根**没有 `scripts/` 目录**；
- 逐棵树查过 **v1.14.3 / v1.15.0 / v1.16.0 / fork `5ba2e91f` / `px4_msgs`，全部没有**。

**它的真身是 `auterion/px4-ros2-interface-lib` 的 `scripts/check-message-compatibility.py`。**

★ 但**直接跑它对本项目会报假 FAILED** —— 它的待查类型清单取自**它自己那个仓库**的
`px4_ros2_cpp/include/px4_ros2/components/message_compatibility_check.hpp`，
而该仓库**没有 `release/1.14` 分支**（只有 `release/1.5.1` / `1.16` / `1.17` / `1.18`）。
拿 main 的清单查 v1.14 这一对，实测报 20 个类型「缺失」：

```
FAILED! The following message types were not found in one of the repositories:
 - ArmingCheckReply.msg        - GotoSetpoint.msg          - RegisterExtComponentRequest.msg
 - RoverRateSetpoint.msg       - SetpointConfig.msg        - UnregisterExtComponent.msg   …（共 20 个）
```

**那 20 个类型在 PX4 v1.14.3 里根本不存在**（逐名核过 `msg/` 目录）—— 是更新版 PX4 才加的。
**这不是不兼容，是「v1.14 里没有这个东西」。** 报错原文里**没有** `Some files differ` 那一段
（即：**零个字段不符**），这一点容易被那行 `FAILED!` 盖过去。

⟹ 本项目的比对改用 `tools/px4-sitl/check_msg_compat.py`：**复用官方那份 `message_hash` 算法**
（从上面那个 clone 里 import，不另写一份免得漂移），但把类型换成**两侧 `msg/` 的并集（184 个）**，
**严格强于官方脚本的子集检查**。

★ 而且那个「不漂移」的保证是**承重的**：`message_hash` 借自上游、会随上游改动而变，
所以脚本**默认要求 `--interface-lib` 恰好停在 `ac2e8aeecb22`**，否则 **❌ 拒绝运行**
（`--allow-unpinned-algorithm` 可降级为警告，但结论的可比性自负）；
跑通时输出会打印算法来源那一行，PASS 自证它是用哪一版算的。

### 2.4 双向话题 + 类型 + 真实数据

`MicroXRCEAgent udp4 -p 8888` + SITL 起来之后（`tools/px4-sitl/run_sitl.sh --agent`）：

```
/fmu/in/*  = 13 条        /fmu/out/* = 11 条        ← 双向前缀都在

/fmu/in/offboard_control_mode  [px4_msgs/msg/OffboardControlMode]
/fmu/in/vehicle_rates_setpoint [px4_msgs/msg/VehicleRatesSetpoint]   ← Phase 2 靠它
/fmu/out/vehicle_status        [px4_msgs/msg/VehicleStatus]          ← ticket 06 靠它
```

★ **类型名解析得出来**，说明 `px4_msgs` 与 PX4 侧对得上（不只是"话题名撞上了"）。

再加一条**数据真的在流**的证据（用显式 `BEST_EFFORT` 订阅，这本身就是 `phase2-implementation.md`
§6.4 那个坑的现场演示）：

```
$ ros2 topic echo --qos-reliability best_effort --once /fmu/out/vehicle_status
timestamp: 1790581756425178
arming_state: 1
nav_state: 4
vehicle_type: 1
```

> ⚠️ **本页刻意不把这条当成"通了"的终点**（ticket 01 的原话：那正是本坑骗人的地方）。
> 真判据在 [ticket 05](../../.scratch/phase2/issues/05-sitl-offboard-smoke.md)：**PX4 是不是真的照做**。
> 本页只负责把"消息能送到、类型对得上、数据在流"这三件事钉死。

---

## 3. 被控对象选了 **SIH**（`sihsim_quadx`），不是 gazebo（Q9 的落地方式）

ticket 01 原文写「gazebo-classic 与 gz-sim 皆可」。**在本机这两条都被实测排除了**：

| 后端 | 第一手判定 |
|---|---|
| **gazebo-classic** | ❌ **本机装不了**：`apt-cache policy gazebo libgazebo11 libgazebo-dev` **全部无候选**（jammy/arm64 只有 `gazebo-common` 这种 arch:all 数据包）。PX4 的 gazebo-classic 后端要对着 gazebo11 的 dev 库编 submodule ⟹ 得先在 Jetson 上自己编一整个 gazebo11 |
| **gz-sim** | ⚠️ **只有 Garden 可用，且要加第三方源**：v1.14.3 的 `src/modules/simulation/gz_bridge/CMakeLists.txt` 里写明 `#ignition-transport11 # IGN (Fortress and earlier) no longer supported`，只 `find_package(gz-transport … gz-transport12)`。本机 apt 里 `ignition-fortress`（= ign-transport11）有候选但**版本不对**；OSRF 官方源的 `jammy/main/binary-arm64` 里确有 `libgz-sim7-dev` + `libgz-transport12-dev` ⟹ 可行，代价是加源 + 约 500 MB + 写 SDF 模型 |
| **SIH** | ✅ **零外部依赖**：PX4 树内模块 `src/modules/simulation/simulator_sih/`，只有 CPU |

**2026-09-28 操作者裁定：用 SIH**（含接受它的证明力代价，见下）。

**机型与目标**：`10040_sihsim_quadx`（`SYS_AUTOSTART=10040`）。目标名由
`simulator_sih/CMakeLists.txt:94` 的 `add_custom_target(sihsim_${model} …)` 生成，模型表 = `airplane / quadx / xvert`。
本树 `ROMFS/px4fmu_common/init.d-posix/airframes/` 共 **56 个条目** = **55 个 airframe + `CMakeLists.txt`**，逐后缀数：

| 后端 | 个数 |
|---|---|
| `_gazebo-classic_` | **35** |
| `_gz_`（gz-sim） | **6** |
| `_jsbsim_` / `_flightgear_` | **9** |
| `_sihsim_` | **3** |
| `_jmavsim_` | **1** |
| `_none_` | **1** |

★ 三个 `_sihsim_` 就是 `simulator_sih/CMakeLists.txt:51` 里 `set(models airplane quadx xvert)` 那一行。
**要外部仿真器的后端占了其中的 50/55**（gazebo-classic + gz-sim + jsbsim/flightgear）
⟹ 「本机编得动且不装外部依赖」这一条，把可选集从 55 压到了 **3**。

**为什么 SIH 特别合 ticket 08 的胃口**：SIH 的机型全在参数里 ——
`SIH_MASS` / `SIH_IXX, IYY, IZZ, IXY, IXZ, IYZ` / `SIH_T_MAX` / `SIH_Q_MAX` /
`SIH_L_ROLL` / `SIH_L_PITCH` / `SIH_T_TAU`（电机时间常数）/ `SIH_KDV, KDW`。
「参数刻意与 z0 失配 ±30%」+「保留一套参数一致的对照组」在 SIH 里是**两份 airframe 文件 diff 一下就出来的东西**，
可入库、可复核；换成 SDF 就是两份模型的漂移风险。

> ★★ **必须写下来的代价（操作者已知情并接受）**：SIH 是 PX4 自带的**简化刚体 + 一阶电机模型**，
> 与我们要写的 10 状态控制器模型**同属一个简化层次**。所以「在 SIH 上能飞稳」**证明不了"模型够真"**；
> 它能证明的是**约定对得上真 PX4**（ENU/FLU → FRD 的符号翻转、推力归一化），这恰好是 seam 3 的判据本身。
> **"模型缺了什么"这件事，SIH 暴露不出来。** 这是本阶段证据强度上的已知天花板。

---

## 4. 复现步骤

```bash
# ① PX4 SITL（clone + venv + 编译）
bash tools/px4-sitl/build_px4_sitl.sh
bash tools/px4-sitl/build_px4_sitl.sh --print-pin     # 看钉住的 tag/commit

# ② micro-XRCE-DDS Agent（编 + 装到 /usr/local）
bash tools/px4-sitl/build_xrce_agent.sh
sudo ldconfig                                          # ★ 少这一步会 "cannot open shared object file"，见 §5 坑 4

# ③ px4_msgs（源在仓库外，工作区在家目录）
mkdir -p ~/px4_msgs_ws/src && ln -sfn ~/refs/px4_msgs ~/px4_msgs_ws/src/px4_msgs
source /opt/ros/humble/setup.bash && cd ~/px4_msgs_ws && colcon build --symlink-install

# ④ 验收
bash tools/px4-sitl/run_sitl.sh --agent               # 一个终端
source /opt/ros/humble/setup.bash && source ~/px4_msgs_ws/install/setup.bash
ros2 topic list | grep fmu                            # 另一个终端：双向前缀都要有
python3 tools/px4-sitl/check_msg_compat.py \
    --px4-tree ~/refs/px4-v1.14.3-build \
    --px4-msgs ~/refs/px4_msgs \
    --interface-lib ~/refs/px4-ros2-interface-lib
```

`colcon build` 在 **`~/px4_msgs_ws`**，**不是本仓库**。`~/px4_msgs_ws` 是**下位工作区**：
Phase 2 的节点（在本仓库的 `src/`，colcon 根 = 仓库根）用到 `px4_msgs` 时，
**先 source `~/px4_msgs_ws/install/setup.bash` 再在本仓库 `colcon build`**。

> ⚠️ **这个落点是本张取的默认、可由 [ticket 06](../../.scratch/phase2/issues/06-mpc-node-fake-px4-fixture.md) 改口径**：
> 也可以把 `px4_msgs` 作为包放进本仓库工作区。之所以不放：`px4_msgs` 是上游大包，
> 且 `README.md` 已把它的位置记成「仓库外 `~/refs/px4_msgs`」—— 与 `~/acados`、`~/refs/acados` 同一惯例。

---

## 5. 本机实测踩到的五个坑（报错原文都在，都不是抄来的）

**坑 1 —— 新版 pip 拒收 PX4 自己的 `requirements.txt`**
`matplotlib>=3.0.*` ⟹ `ERROR: Invalid requirement: 'matplotlib>=3.0.*': .* suffix can only be used with '==' or '!=' operators`。
构建 SITL 用不到 matplotlib/pandas/lxml，脚本里滤掉再装。

**坑 2 —— `empy` 必须钉 3.3.4，而 `requirements.txt` 只写 `empy>=3.3`**
PX4 v1.14.3 的模板调用是 empy **3.x** 的 API。全树用到 `em.RAW_OPT` 的**只有两处**（`grep -rl` 实测）：

- `Tools/msg/px_generate_uorb_topic_files.py:182`
- `src/modules/uxrce_dds_client/generate_dds_topics.py:133`

两处都是 `em.Interpreter(output=…, globals=…, options={em.RAW_OPT: True, em.BUFFERED_OPT: True})`，
而 `em.RAW_OPT` 这个开关是 empy **3.x** 的。照着 `>=3.3` 装会拿到 **4.2.1**，签名已变。
★ `empy==3.3.4` 必须**在 requirements 之后**装，否则 `>=3.3` 会把它覆盖掉。

> ⚠️ 别把 `src/modules/flight_mode_manager/generate_flight_tasks.py:23` 也算进来：
> 那行是 `em.Interpreter(output=…, globals=…)`，**没有 `options=`**，对 empy 版本不敏感。

**坑 3 —— 本机 `pip` 直连 pypi 会静默卡住**
`pypi.org` 不可达、`files.pythonhosted.org` 正常（同机实测，与 [ticket 02](../../.scratch/phase2/issues/02-acados-orin-build.md)
记的是同一条）。表现是**十分钟一声不响**。一律加 `-i https://pypi.tuna.tsinghua.edu.cn/simple`。

**坑 4 —— ★ Agent 装完跑不起来，而报错指向库、不指向根因**
`/usr/local/bin/MicroXRCEAgent --help` ⟹
`error while loading shared libraries: libmicroxrcedds_agent.so.2.4: cannot open shared object file`。
库**确实装了**（`/usr/local/lib/libmicroxrcedds_agent.so.2.4.2`，3014263 字节），
`/etc/ld.so.conf.d/libc.conf` 里**也确实有** `/usr/local/lib` —— 只是**没跑 `ldconfig`**，缓存没更新。
`sudo ldconfig` 即愈。

**坑 5 —— Agent 的 superbuild 去 checkout 一个**已被上游删除的分支**
`Micro-XRCE-DDS-Agent` v2.4.2 的 `CMakeLists.txt:99` 写 `set(_fastdds_tag 2.12.x)`，
而 eProsima **已经删了 `2.12.x` 分支**（`git ls-remote --heads` 里没有，只剩 tag `v2.12.0/1/2`）。
报错是 `fatal: invalid reference: 2.12.x` / `Failed to checkout tag: '2.12.x'` —— **不指向"上游删了分支"这个根因**。
`build_xrce_agent.sh` 把那一行改钉到 **tag `v2.12.2`**（2.12 系列最后一个）。

> ⚠️ 这让 `~/refs/micro-xrce-dds-agent` **不再是一份"原样只读副本"**：`git -C … diff` 有一处 1 行的改动
> （同 [ticket 02](../../.scratch/phase2/issues/02-acados-orin-build.md) 对 `~/refs/acados` 的处置）。脚本会自动打这个补丁。

---

## 6. 与现有文档的偏差（**已核实，未擅自清扫**）

| # | 文档说的 | 实测 | 处置 |
|---|---|---|---|
| D1 | `Tools/check-message-compatibility.py`（`PROJECT.md:450`、`px4-interface.md:178`、`:284`）；本张清单同上 | 真身在 `auterion/px4-ros2-interface-lib`；见 §2.3 | **本张已改** `phase2-implementation.md` §S1 与 §6.1 给 S1 的指令；**另外 2 文件 3 处未动**（不越界）⟹ **已开 [ticket 14](../../.scratch/phase2/issues/14-stale-check-msg-compat-path.md) 承接**，见 §7 遗留。★ **2026-09-29 已收口**：ticket 14 `done`，那 3 处（`PROJECT.md:455`、`px4-interface.md:183`/`:297`）已改 ⟹ 本行「**未动**」一词已过期（保留为当时记录） |
| D2 | 「23 个 submodule」 | **17** 个（`grep -c '^\[submodule' .gitmodules`） | 本页记正；`phase2-implementation.md` §S1 的「23」已一并改 |
| D3 | 命令写 `micro-xrce-dds-agent udp4 -p 8888` | 二进制名是 **`MicroXRCEAgent`**（无小写别名） | 本页记正 |
| D4 | 「gazebo-classic 与 gz-sim 皆可」 | 本机两条都不可直接用，见 §3 | 本页记正；**Q12「留在 1.14.x」的决策本身不变** |

---

## 7. 给 Phase 3 的交接

1. **构建环境已就位**：`~/refs/px4-v1.14.3-build/` + venv + 脚本三件套。
   重编固件（Q37=a）时**复用这套工具链**（`make <board>_default` 而不是 `px4_sitl_default`）。
   ⚠️ 但「**改哪一行**」仍不是已知答案 —— 原定的 ADC 一行修复指向 `boards/hkust/nxt-dual`，
   而真身树里**没有 `hkust/`**（`params-analysis.md` §3.5.1）。
2. **`px4_msgs` 的比对方法可复用**：`check_msg_compat.py` 不依赖 SITL，将来换任何 PX4 树都能跑。
3. **遗留（本张不做，但已开票点名）**：
   `PROJECT.md:450` 与 `docs/reference/px4-interface.md:178`、`:284`（**2 文件 3 处**）仍在引用不存在的脚本路径
   ⟹ **[ticket 14](../../.scratch/phase2/issues/14-stale-check-msg-compat-path.md)** 承接。
   —— 按 ticket 02 的先例，**本张只改自己这节**（`phase2-implementation.md` §S1 / §6.1 给 S1 的指令 / 证据索引），
   **不顺手清扫别的票**；开票点名才是 `done` 要求的动作。
