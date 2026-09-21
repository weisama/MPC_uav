# ★ 本包是 vendored —— 勿替换、勿修改

> **本文件取代同目录 `README.md` 的地位。**
> `README.md` 是上游 zip 里原样带来的，**内容已过时**：
> 它教人去 `git clone https://github.com/efc-robot/vrpn_client_ros2` —— **不要照做**。
> 本包**不是**那个公开仓库，那份公开版会丢精度，并且会和下游断连（理由见下 §3）。

---

## 1. 来源（钉死）

| 项 | 值 |
|---|---|
| 仓库 | `git@github.com:Huan-qun/Sim2real-UAV-Params-.git` |
| commit | **`b7aeebb71cc708445fbac70a4863c736d4674630`**（2026-08-30） |
| 路径 | `S2R/assets/vrpn_listener.zip` |
| zip md5 | `9f43d98fb1e2d03b69067ef43babeda9`（10366 B） |
| 本地核对副本 | `~/refs/Sim2real-UAV-Params-/`（仓库外，见 [`refs-manifest.md`](../../docs/reference/refs-manifest.md) §1） |
| zip 内 mtime 最晚文件 | `src/vrpn_listener.cpp` / `config/params.yaml`，2026-05-07 |

zip 在 `1aa94a98`（2026-07-21）加入后**未再改动**（`vrpn_listener.cpp` 8088 B）——
这一条由 `mocap-setup.md` §6.2 记录，2026-09-20 经 Q16=b 重新取上游复核过。

**提取方式**：`cd src && unzip -q <zip>`（zip 顶层目录就是 `vrpn_listener/`）。
**已逐文件 sha256 核对字节一致**（见 §4 表），**没有任何一行被改动**。

---

## 2. ★★ 定制点一：`stamp = now()` —— **刻意设计，不是 bug，不要改**

`src/vrpn_listener.cpp` 第 36–40 行**原样写着理由**：

```cpp
    pose_msg.header.frame_id = listener->_frame_id;
    // Use ROS receive time, not VRPN msg_time, because the mocap PC clock is
    // not NTP-synced with the Jetson (~4s offset observed). Adds ~2-5ms jitter
    // but keeps the stamp in the same clock domain as IMU/MAVROS for fusion.
    pose_msg.header.stamp = listener->now();
```

`twist`（第 62 行）与 `accel`（第 93 行）**同一处理，同一理由**。

**为什么不能改成 VRPN 的 `msg_time`**：
动捕主机与 Jetson 有 **~4 秒**时钟差（未做 NTP 同步）。
`msg_time` 会把这 4 秒带进消息 → 落在**另一个时钟域** → **毁掉 EKF2 融合**。
`now()` 取的是 **ROS 接收时刻**，与 IMU / mavros **同一时钟域**，代价只是 ~2–5 ms 抖动。

> ⚠️ **措辞纪律**：这是**上游自己发布的版本就这样**，不是本地打的补丁。
> 正因如此，它**尤其容易**被后来者当成脏改动"顺手清理" —— 这正是本节要防的事。
> ⚠️ **连带**：时间戳不前进的位姿消息**会被 mavros 静默丢弃**，
> 所以"给它一个固定时间戳"这种做法是**反效果**，不是"稳定化"。

## 2b. 定制点二：`handleTwist` 的角速度**刻意留零**

第 57–59 行的注释：多数动捕 VRPN server 不填 `vel_quat`，而由四元数差分求角速度还需要
`vel_quat_dt`。**角速度应从 IMU 取**，不要在这里补。**同属必须保留的上游行为。**

## 2c. 定制点三：`frame_id` 三处统一用参数化的 `_frame_id`

`pose` / `twist` / `accel` 三处都是 `listener->_frame_id`（来自 `config/params.yaml` 的
`frame_id`），**不要在源码里写死 `world`**。

> 上游 §2.2 另称定制了 **QoS** 与 **polling 频率**：前者在源码里体现为三个 publisher
> 都用 `create_publisher<>(topic, 10)` 的**默认 QoS（reliable，depth 10）**；
> 后者体现为 `params.yaml` 里 `mainloop_frequency: 400.0`（动捕 200 Hz 出包，留 2× 余量）。
> **这两条以源码/配置为准，本节不额外加约束。**

---

## 3. ★★ 勿替换为公开版

**不要**用 `efc-robot/vrpn_client_ros2` 的公开版本替换本包。上游明确警告，原因是公开版相对本定制版
少修了：`stamp`、QoS、polling 频率、`handleTwist`/`handleAccel` 的 `frame_id`。
其中 **QoS** 一条是硬伤：本包发的是 **reliable**，下游（如 `vrpn_relative_pose`）按 reliable 订阅
——换成 best-effort 的公开版会**直接断连**。

**同类陷阱**：`mocap-setup.md` §2.2 / `phase1-implementation.md` §8.1。

---

## 4. 复核方式（一条命令，随时可重跑）

```bash
# 与钉住的 zip 逐文件比对字节 —— 应输出 "全部字节一致"
cd /home/chen/ros_ws/src && python3 - <<'EOF'
import zipfile, hashlib
z = zipfile.ZipFile('/home/chen/refs/Sim2real-UAV-Params-/S2R/assets/vrpn_listener.zip')
bad = [n for n in z.namelist() if not n.endswith('/')
       and hashlib.sha256(z.read(n)).hexdigest()
           != hashlib.sha256(open(n,'rb').read()).hexdigest()]
print("!!! 有差异: " + str(bad) + " !!!" if bad else "全部字节一致 ✅")
EOF
```

VENDORED 时的基线（`sha256sum` 全量）：

| 文件 | sha256 |
|---|---|
| `CMakeLists.txt` | `f7c5fec1337c477a6f4cc52099cc12058822dba665a89e8e4ffca9a749f6ed55` |
| `README.md` | `65fa132d0911f62d772e03a20e62edabc5fb0d72a50582efb375f95cfd2034fb` |
| `cmake/FindVRPN.cmake` | `bb21b94ca7c693935b952c4d21f4f74f436cd4309cc583f9bd7ff924952f84e9` |
| `cmake/Findquatlib.cmake` | `3f2fd6c35e3cb9d5ab7a779f733918abceb7db6233eb2107db5d15093f5d839a` |
| `config/params.yaml` | `c811abf663c624413211f664c38f0d0fb7fa13318858203ddce5a670914994aa` |
| `include/vrpn_listener/vrpn_listener.hpp` | `7501ae11cb0179de22d98d43b1776220025c03cf2fe7bbe6310181b0fdf2e7a4` |
| `launch/sync_entity_state.launch` | `5c560b06913d839acd0330e6248940f0bcff4258fccb98263e56ff0c43154889` |
| `package.xml` | `9c78ac727e17ac9f03b47aa71e1c30916c91a54e18fbbc3a5b09ee847c53ff38` |
| `src/vrpn_listener.cpp` | `bff6b138523d6ff1d2c7d8da3344f5ae103fd95bb7e9a4e00b71e3203c435e2c` |

**`VENDORED.md` 本身是新增文件，不属于 vendored 集合**，不在上表内，可以修改。

---

## 5. 依赖与运行（环境侧，不改本包任何文件）

| 项 | 值 |
|---|---|
| VRPN 库 | **`ros-humble-vrpn` 7.35.0**（apt 装到 `/opt/ros/humble`）。⚠️ **不要**按 `mocap-setup.md` §2.1 源码编译 —— 见下"为什么" |
| 为何可用 apt 那份 | 本包自带的 `cmake/FindVRPN.cmake` + `Findquatlib.cmake` 能在 `CMAKE_PREFIX_PATH` 下直接找到它（`libvrpn.a` / `libquat.a` / `include/vrpn_*.h` / `include/quat.h`），**因此不需要改动本包的 CMake 文件** |
| ★ **还必须搭一座桥** | 本包 `CMakeLists.txt` 里另有一行 `target_link_libraries(vrpn_listener vrpn)` —— **裸名字**，链接器会展开成 `-lvrpn`，**这要求 `libvrpn` 落在 ld 的默认搜索路径里**。apt 那份在 `/opt/ros/humble/lib`，**不在**默认路径 ⟹ 报 `cannot find -lvrpn`。**这一行是 vendored 的，不能改**，所以只能在系统侧搭桥：<br>`sudo ln -sfn /opt/ros/humble/lib/libvrpn.a /usr/local/lib/libvrpn.a`<br>（`/usr/local/lib` 在 ld 的默认 `SEARCH_DIR` 里，已核实；且上游本就是 `sudo make install` 到 `/usr/local` 的布局。**只需 `libvrpn`，`libquat` 全程以绝对路径引用、不需要**。） |
| 动捕主机 | `10.1.1.198:3883`（见 `config/params.yaml`）。该参数是**唯一**要改的地方（换个场地才需要） |
| 刚体名 | ★ **自动发现，不在配置里写死** —— `refresh_trackers()` 每秒遍历 VRPN 连接的 `sender_name(i)`，发现新 sender 才建 publisher（`src/vrpn_listener.cpp:109-125`）。话题名格式 `/vrpn/<刚体名>/pose`，刚体名里的空格会被换成 `_`（`replaceSpace()`）。<br>⚠️ **大小写照抄 VRPN，不做任何规范化** —— 实测 z0 的发送名是 **`Z0`**，所以话题是 `/vrpn/Z0/pose`（见 [`mocap-setup.md`](../../docs/reference/mocap-setup.md) 文末「ticket 03 实测」§二） |

```bash
source /opt/ros/humble/setup.bash
cd /home/chen/ros_ws
colcon build --packages-select vrpn_listener
source install/setup.bash
ros2 launch vrpn_listener sync_entity_state.launch
```

---

## 6. 验收记录（2026-09-21，z0 的 Jetson `orin-nx`，全部第一手）

| 判据 | 结果 |
|---|---|
| `colcon build` 干净通过 | ✅ **0 warning / 0 error**（`-Wall -Wextra -Wpedantic` 下也干净），产物 `install/vrpn_listener/lib/vrpn_listener/vrpn_listener`（aarch64 ELF） |
| 构建产物不进 git | ✅ `build/` `install/` `log/` 均被仓库根 `.gitignore` 挡住（`git check-ignore` 实测） |
| vendored 源码一行未改 | ✅ 9 个文件逐个 sha256 与 zip 一致（§4 表） |
| 刚体名自动发现 | ✅ 实测发现 **28 个 sender** → 自动生成 **84 个话题**，`params.yaml` 里没有任何名字 |
| `ros2 topic hz` 100–400 Hz | ✅ **199.9 Hz**（在正在出数的 `U_Tracker0` 上实测） |
| ★ 对 **z0 本体**（`/vrpn/Z0/pose`）的速率 | ⏳ **未验成 —— `Z0` 当前 0 数据流**，见 [`mocap-setup.md`](../../docs/reference/mocap-setup.md) 文末 §三。**速率上限本身已由 `U_Tracker0` 证明达标**，缺的是"把 z0 放进动捕棚/在 Nokov 里开流"这个物理动作 |
