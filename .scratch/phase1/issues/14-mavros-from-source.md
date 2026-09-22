# 14: 装 mavros 2.15.1 —— ★ arm64 二进制在可达镜像里**全都没有**，改源码构建（pin tag 2.15.1）

Status: done
Type: task

**为什么单开一张：** [ticket 13](13-mavros-bench-vision-pose.md) 的第 1 步写的是
`sudo apt install ros-humble-mavros ros-humble-mavros-extras`。**这条路实测不通** ——
不是网络问题，是**上游根本没出（或已撤）humble 的 arm64 二进制**。本张记下证据、换路径、并把
mavros 这个**系统级依赖**的来源钉死（pin tag + 构建命令 + 依赖版本），免得下次换机器又从头摸一遍。

**Blocked by:** 无（飞控已上电，串口已定案 `/dev/ttyTHS1 @ 921600`）
**被它解除阻塞的是：** [ticket 13](13-mavros-bench-vision-pose.md)（QGC 侧验收）、[ticket 05](05-control-offboard-skeleton.md)

---

## 一、第一手证据（2026-09-21，z0 的 Jetson `orin-nx`）

### (a) 十个镜像的 `jammy/main` 索引与 pool 里都没有 `ros-humble-mavros` 的二进制

```bash
# 索引（binary-arm64/Packages.gz）——下面这句在 huaweicloud / tuna / ustc / aliyun / sjtu / bfsu / nju / tencent 上都是 0
curl -s https://<镜像>/ros2/ubuntu/dists/jammy/main/binary-arm64/Packages.gz | gunzip | grep -c "^Package: ros-humble-mavros$"
# pool 目录（hit.edu.cn 额外查过）
curl -s https://<镜像>/ros2/ubuntu/pool/main/r/ros-humble-mavros/ | grep -o 'ros-humble-mavros_[^"<]*arm64\.deb'
```

| 镜像 | `ros-humble-mavros` 二进制 | 备注 |
|---|---|---|
| repo.huaweicloud.com（本机在用的源） | ❌ | 索引日期 2026-09-17 |
| tuna / ustc / aliyun / sjtu / bfsu / nju / tencent / zju | ❌ | 各家索引里都没有这个包名 |
| hit.edu.cn | ❌ | pool 里有 2.1.0–2.7.0 的*.dsc/源包*，**没有 arm64.deb** |

⚠️ **一个差点骗到我的陷阱**（记下来免得重踩）：`grep -o 'ros-humble-mavros_[^"<]*\.deb'` 会**假匹配**
`ros-humble-mavros_2.15.1-1jammy.debian.tar.xz`（`.deb` 后面没锚定）⟹ 必须按 **`arm64\.deb`** 精确匹配。

### (b) 关键对照：同一个仓库、同一次构建，**只有 mavros 本体缺**

```bash
curl -s https://mirrors.aliyun.com/ros2/ubuntu/pool/main/r/ros-humble-mavros-msgs/ | grep -o 'href="[^"]*"'
#  → ros-humble-mavros-msgs_2.15.1-1jammy.20260907.223842_arm64.deb   ✅ 有
curl -s https://mirrors.aliyun.com/ros2/ubuntu/pool/main/r/ros-humble-mavros/ | grep -o 'href="[^"]*"'
#  → 只有 .dsc / .orig.tar.gz / .debian.tar.xz                              ❌ 没有二进制
```

⟹ 不是镜像同步不全，是**上游那次构建没产出/已被撤**。iron / rolling 的 mavros 是**全的**
（同一份索引里能查到）—— 只有 humble 缺。

### (c) 官方源不可达，但 GitHub 可达（源码构建的前提成立）

```bash
curl -s -o /dev/null -w "%{http_code}\n" https://packages.ros.org/ros2/ubuntu/dists/jammy/Release   # → 000（不可达）
git ls-remote --tags https://github.com/mavlink/mavros | grep 2.15.1
#  → 0755770a058e2aed558b3d4fc10e45ba68d2b117  refs/tags/2.15.1
#     22ae5b7cc7cdb4cb9c2070a8213c72dae445a23e  refs/tags/2.15.1^{}   ← 真正要 pin 的 commit
```

**版本对得上**：已装 `ros-humble-mavros-msgs` = `2.15.1-1jammy.20260907.223842` ⟹ 源码也用 **2.15.1**。

## 二、决定（★ 用户 2026-09-21 裁定）

**从源码构建 mavros 2.15.1**，放在**独立工作区** `~/mavros_ws`（**不进本仓库**）：

- 理由 ①：文档整条 Phase 1（`phase1-implementation.md` §2/§3.5、ticket 04/05）都建在 mavros 的
  接口与"边界处做 ENU→NED / FLU→FRD"这条结论上 —— 源码构建**一个结论都不用改**，
  而"改用 pymavlink 自建桥"要推翻它并重写 ticket 05 的前提。
- 理由 ②：mavros 是**系统级依赖**，不是本项目代码 —— 不入库（入库要走 ticket 03 那套
  `VENDORED.md` + 哈希核验的流程，代价不对等）。来源改由**本张的 tag + commit + 命令**钉死。

## 三、落地步骤

```bash
# 1) 源码（已在 2026-09-21 完成）
mkdir -p ~/mavros_ws/src && cd ~/mavros_ws/src
git clone --depth 1 --branch 2.15.1 https://github.com/mavlink/mavros mavros
git -C mavros rev-parse HEAD        # 必须是 22ae5b7cc7cdb4cb9c2070a8213c72dae445a23e

# 2) 跳过三个包（★ 关键决定：mavros_msgs 用 apt 的同版本，不重复建一套）
cd ~/mavros_ws/src/mavros
touch mavros_msgs/COLCON_IGNORE mavros_examples/COLCON_IGNORE test_mavros/COLCON_IGNORE

# 3) 依赖：实测只有两个包缺（其余已装，版本见下）
sudo apt install ros-humble-eigen-stl-containers ros-humble-mavlink

# 4) 构建（8 GB 内存 ⟹ 限制并行度数，避免 OOM）
cd ~/mavros_ws && source /opt/ros/humble/setup.bash
colcon build --parallel-workers 4 --cmake-args -DCMAKE_BUILD_TYPE=Release
```

**依赖实测清单（2026-09-21）**：`libmavconn` / `mavros` / `mavros_extras` 三个 `package.xml`
的 32 个依赖里，**30 个已装**（`libgeographic-dev 1.52`、`geographiclib-tools`、`libasio-dev`、
`libconsole-bridge-dev`、`libeigen3-dev`、`libyaml-cpp-dev`、`python3-empy`、`python3-click`、
以及 22 个 `ros-humble-*`），只缺 `ros-humble-eigen-stl-containers` 与 `ros-humble-mavlink`。

> ⚠️ 顺带更正一处**我自己的**查证失误：先前我把 geographiclib 的包名写成 `libgeographiclib-dev`
> 并据此认为"universe 缺失、源码构建也不通"。**Ubuntu 22.04 里它叫 `libgeographic-dev`**，
> 一直在。⟹ 教训同 ticket 04：**包名要按 `apt-cache search` 的结果写，不要凭记忆**。

## 验收项

- [x] `colcon build` 成功；`~/mavros_ws/install` 里出现 `mavros` 与 `mavros_extras` 的可执行/插件
      ⟹ ✅ **3 个包全部建成、0 失败**（`libmavconn` 37.7 s / `mavros` 7 min 2 s / `mavros_extras` 5 min 50 s）；
      另有两次前置报错已排除：缺 `ros-humble-ament-cmake-google-benchmark`（补装）、
      首次 launch 缺 GeographicLib 数据集（`install_geographiclib_datasets.sh`，见坑 1）
- [x] `ros2 launch mavros px4.launch fcu_url:=/dev/ttyTHS1:921600` 起得来，`/mavros/state` 有数据
      ⟹ ✅ 日志 `CON: Got HEARTBEAT, connected. FCU: PX4 Autopilot`；
      `/mavros/state` = `connected: true, armed: false, guided: true, mode: AUTO.LOITER, system_status: 3`
- [x] ★ **记录 `/mavros/state` 的发布频率**（`ros2 topic hz`）—— 这决定"解锁上升沿最多晚多久被检出"，
      是 [ticket 04](04-mocap-relative-pose.md) §七 第 4 条与 [ticket 13](13-mavros-bench-vision-pose.md) 坑 4 的遗留项
      ⟹ ✅ **`1.00 Hz`**（30 条 / 30 s；间隔 min 0.998 / mean 1.000 / max 1.002 s）。
      ⚠️ `ros2 topic hz` 在本机打不出读数（只打印 `Terminated`），改用 rclpy 计数脚本量的。
      结论已写回 04 §七 第 4 条与 ticket 13 坑 4
- [x] `vision_pose` 链路可用：`/mavros/vision_pose/pose` 上的消息被 mavros 收下（`mocap` 节点侧无异常）
      ⟹ ✅ 订阅者 = `mavros/vision_pose` 插件（QoS 兼容）；mavros 日志 `Plugin vision_pose initialized`；
      **且飞控 EKF2 确实在用**（`pos_horiz_abs` 反转实验）—— 证据见 [ticket 13](13-mavros-bench-vision-pose.md) §实测 ②③
- [x] **provenance 入库**：把 tag/commit、COLCON_IGNORE 的三个包、依赖清单、构建命令写进
      [`mocap-setup.md`](../../../docs/reference/mocap-setup.md)（或本张 Comments 即可，视篇幅定）
      ⟹ ✅ 已写进 [`mocap-setup.md`](../../../docs/reference/mocap-setup.md) **§5.1**（apt 那条已就地作废）
- [x] 回到 [ticket 13](13-mavros-bench-vision-pose.md) 做 QGC 侧验收
      ⏳ 余下两条**需要人到 QGC 现场**（① 目视 + ⑥ 轴向确认）与**拆桨解锁**一次；其余已就地做完
      ★ **2026-09-22 拆分（三项里两项已答回、一项没做成 ⟹ 当时没勾）**：
      - ✅ **① QGC 目视**（`VISION_POSITION_ESTIMATE` 与 `LOCAL_POSITION_NED` 都有数）—— 操作者报回；
      - ✅ **⑥ 轴向确认**（左 `x+` / 机头 `y+` / 下 `z+` ⟹ 显示的是 NED ⟹ **ENU→NED 转换确实发生了**）；
        两项的原话、agent 解读、数据窗见 [ticket 13](13-mavros-bench-vision-pose.md) §实测 2026-09-22；
      - ⏳ **拆桨解锁一次（阶段 B）上午没做成**：当天动捕输入在 `06:52:02.8→07:05:22.9` 断流 800.2 s，
        而当时的解锁都落在断流窗口里 ⟹ 边沿被错过
        （阶段 B 的触发在 `on_pose` 内，`mocap_rel_pose_node.cpp:171/190`，**不是代码缺陷**）；
        另有飞控 `crash dumps present on SD` 拦解锁一道（见 [ticket 20](20-fcu-hardfault-i2c1-crash-dump.md)）。
      ✅ **2026-09-22 08:17:53 三项全部达成**：飞控经清障后可解锁，操作者**拆桨**解锁、电机点转 4.2 s，
      `mocap` 节点打出「★ 阶段 B：解锁上升沿后的起飞前原点已锁定 —— 切换发生在此刻」。
      ⚠️ 同刻 WARN 掉了「启动到解锁之间飞机被移动」这条纪律判据 —— **按用户裁定判为已验证**
      （触发 + 阈值告警两条路径都实测到）；时刻线、判据结论、残留全在
      [ticket 13](13-mavros-bench-vision-pose.md) §实测 2026-09-22 §阶段 B（**本张不复述数字**）。

## 坑（开做前先看）

1. ⚠️ **`geographiclib` 数据集**：mavros 首次使用 `global_position` 类插件时会要
   `/usr/share/GeographicLib` 数据集（`sudo /opt/ros/humble/lib/mavros/install_geographiclib_datasets.sh`，
   源码构建时脚本在 `~/mavros_ws/src/mavros/mavros/scripts/`）。**对 vision_pose 链路不一定必需**，
   真报错再装（装它要下载，本机到外网不稳定）。
2. ⚠️ **两套 `mavros_msgs` 的版本必须同代**：我们用 apt 的 `2.15.1-1jammy.20260907`。
   若哪天 apt 把它升到 2.16.x，源码 mavros 2.15.1 与新 msgs 的 ABI 可能不匹配
   ⟹ 升级前先看本张，或把源码 mavros 也跟到同一 tag。
3. ⚠️ **8 GB 内存**：不加 `--parallel-workers` 直接构建会并发编译到 OOM（本机还要同时跑动捕链路）。
4. ⚠️ **`~/mavros_ws` 与 `/home/chen/ros_ws` 的环境要同时 source**，且**顺序**决定谁覆盖谁
   （两个工作区都有 `install/setup.bash`）。`mavros_msgs` 只在 `/opt/ros/humble` 里有一套，不冲突。

## Comments

### 2026-09-21 —— 侦察、裁定、开工（agent，在 z0 的 Jetson 上）

- 串口定案：`mavlink_probe.py --scan` → **`/dev/ttyTHS1` @ 921600**（`sysid=1, type=2`）；
  `dialout` 组已修好（`dialout:x:20:chen`，当前会话即生效）。⟹ [ticket 01](01-hardware-prereqs.md) 的 ② 结案。
- 证据见上文 §一；裁定见 §二（用户选"源码构建 2.15.1"）。
- ⚠️ 一处**修正 ticket 13**：那张的第 1 步（apt 装 mavros）与坑 1 已改为指向本张。

### 2026-09-21 —— ★ 构建完成 + 连上飞控（验收项逐条已勾）

**构建（命令见 §三）**：`git -C mavros rev-parse HEAD` 已核对 =
`22ae5b7cc7cdb4cb9c2070a8213c72dae445a23e` ✅（与 tag 2.15.1 的 `^{}` 一致）。
`colcon build` 三个包全过、**0 失败**：`libmavconn` 37.7 s、`mavros` 7 min 2 s、`mavros_extras` 5 min 50 s。

**开做后又撞上两个坑（补进下面的坑清单，原文那 4 条之外）**：

- ⚠️ **缺 `ros-humble-ament-cmake-google-benchmark`** —— `mavros` 的 `CMakeLists.txt` 在 `BUILD_TESTING`
  下会 `find_package(ament_cmake_google_benchmark)`，第一次 `colcon build` 直接失败。补装即过。
- ⚠️ **GeographicLib 数据集对源码构建是"必需"而非"可选"** —— 首次 launch 直接以
  `GeographicLib exception: File not readable /usr/share/GeographicLib/geoids/egm96-5.pgm` 中止。
  源码构建时脚本在 `~/mavros_ws/src/mavros/mavros/scripts/install_geographiclib_datasets.sh`
  （**不是**坑 1 里写的 `/opt/ros/humble/lib/mavros/...` —— 那条是 apt 安装的路径）。
  执行后装了 geoids / gravity / magnetic 三套数据集，重启 launch 即成功。
  ⟹ **本坑从"不一定必需"升格为"必需"**（与原文坑 1 的判断相反，实测为准）。

**连上飞控**：`fcu_url:=/dev/ttyTHS1:921600`（串口定案见 [ticket 13](13-mavros-bench-vision-pose.md)）。
`/mavros/state` 出数、`1.00 Hz`，`vision_pose` 链路经 EKF2 侧证实**真的在融合**
（`pos_horiz_abs` 反转实验，见 [ticket 13](13-mavros-bench-vision-pose.md) §实测 ③）。

**本张的连带产出**：读飞控参数的正确方式被摸清（`/mavros/param/get` 在 ROS2 版**不存在**，
参数走标准 ROS 参数接口；缓存 915 条）⟹ 已就地做了 `z0.params` ↔ 飞控真身的**逐条对账**，
另开 [ticket 15](15-fcu-param-reconciliation.md)。

### 2026-09-22 —— 最后一条验收：三项里 ✅ 两项，第 3 项（阶段 B）没做成 ⟹ **本张保持 open**

**mavros 这一侧的东西全部做完并已实测**：源码构建三包 0 失败（tag/commit 已核）、连飞控、
`/mavros/state` = `1.00 Hz`、`vision_pose` 链路被**飞控自己的** EKF2 证实真的在融合（`pos_horiz_abs` 反转实验）、
provenance 已入 [`mocap-setup.md`](../../../docs/reference/mocap-setup.md) §5.1。
本张唯一的开口就是验收项最后一条"回到 13 做 QGC 侧验收"—— 今天被拆成三项：

| 子项 | 结果 |
|---|---|
| ① QGC 目视（两话题有数） | ✅ 操作者报回 |
| ⑥ 轴向确认（左 `x+` / 机头 `y+` / 下 `z+` ⟹ NED ⟹ 转换确实发生） | ✅ 操作者报回 |
| **拆桨解锁一次（阶段 B）** | ⏳ **没做成** —— 当天动捕输入断流 800.2 s，解锁边沿全被错过 |

⟹ `Status: ready-for-agent → ready-for-human`（剩下的一步要人到现场，且**要先让动捕数据通**；
它本身在 [ticket 13](13-mavros-bench-vision-pose.md) 的验收项里跟踪）。
🚧 **2026-09-22 追加阻塞**：操作者随后试解锁，被**飞控自己**拒绝 ——
`crash dumps present on SD, vehicle needs service`（SD 卡根目录有硬故障转储，内容 = `wq:I2C1` 硬故障）
⟹ **清障之前"拆桨解锁一次"做不了**，见 [ticket 20](20-fcu-hardfault-i2c1-crash-dump.md)。
✅ **同日 07:57 已清障**（用户授权，删前已留档）⟹ 阻塞解除，可重做最后一步（还差动捕数据通）。
★ 这条阻塞**不属于 mavros**（mavros 侧本张已全部实测完），只是挡在最后一步前面。
★ 不把这一格勾掉的理由很直白：`done` 的定义是"验收项**全部**勾上"，而这一格的三项里有一项没发生 ——
按本仓库的规矩，不能靠 `done` 掩盖（`docs/agents/triage-labels.md`）。

### 2026-09-22 08:17 —— ★ 三项全部达成 ⟹ 本张收 `done`

操作者在飞控清障后拆桨解锁一次（`08:17:52.542 FCU: Armed by external command` → `08:17:56.741 Disarmed`，
电机无桨点转 4.2 s），`mocap` 节点于 `08:17:53.227` 打出「★ 阶段 B：解锁上升沿后的起飞前原点已锁定」。

- 一手数据（时刻线、两条判据各自的结论、旁证）**全部记在**
  [ticket 13](13-mavros-bench-vision-pose.md) §实测 2026-09-22 §阶段 B —— **本张不复述数字**。
- ⚠️ **一处如实留档**：同刻 WARN 掉了「启动到解锁之间飞机被移动」这条纪律判据（切换量超阈值），
  **用户裁定判为已验证**（触发路径与阈值告警路径**都已**实测）；「跳变 ≈ 0」这条仍是**零观测的残留**，
  已写进 [`bench-runbook.md`](../../../docs/control/bench-runbook.md) T4 的「记录」列，飞行日跑 T4 时顺手填。
- ⟹ `Status: ready-for-human → done`（验收项 5 条全勾；`mavros` 侧本张全部实测完）。
