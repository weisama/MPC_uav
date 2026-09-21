# 03: Vendor `vrpn_listener` + ROS2 工作区骨架

Status: done
Type: task

> ✅ **2026-09-21 完成**（agent 在 z0 的 Jetson 上执行）。五条验收项 **4 条全过 + 1 条"速率达标但对 z0 本体未验成"**
> （`Z0` 当前没有数据流，是动捕软件侧的物理状态，不是本张的缺陷）—— 交代见下方 Comments，
> 残留项另开 [ticket 12](12-z0-mocap-stream-check.md)。
> 顺带把 [ticket 01](../issues/01-hardware-prereqs.md) 留给本张的**未决项「VRPN 走 UDP 还是 TCP」判定了：UDP**。

**What to build:** 仓库里第一次出现 `src/`。把**上游那份定制过的** `vrpn_listener` 原样 vendor 进来并让它跑起来，使「动捕主机 → VRPN/UDP → ROS2 话题」这一段成为一条**已被验证、可复用的底座**。

同时建立 ROS2 工作区骨架（能 `colcon build`，构建产物不进 git）—— 后面每个包都建在它上面，所以它先做。

**来源是钉住的，不要换成公开版：**
必须是 `Huan-qun/Sim2real-UAV-Params-` 的 `S2R/assets/vrpn_listener.zip`，与 [`refs-manifest.md`](../../../docs/reference/refs-manifest.md) 记录的 commit（`b7aeebb`）对得上。**不是**公开的 `efc-robot/vrpn_client_ros2`。

**两件必须原样保留的事：**

1. **`stamp = now()` 是刻意设计，不是 bug。** 动捕主机与 Jetson 有 **~4 秒**时钟差，改成 VRPN 的 `msg_time` 会毁掉 EKF2 融合。这一行**连注释一起原样保留**。
2. **zip 里的 `README.md` 已过时**（它还在教人 clone 公开版）。**不要信它** —— 用 `VENDORED.md` 取代它的地位。

`VENDORED.md` 要写明：来源 zip 与 commit、`stamp = now()` 这个定制点与理由、以及**「勿替换为公开版」**。

**Blocked by:** 01（Vendor 完成后要跑起来，需要动捕 WiFi 通）

- [x] `colcon build` 干净通过；构建产物（`build/` `install/` `log/`）不在 git 里
- [x] `VENDORED.md` 存在，含来源 commit、定制点、"勿替换"三条
- [x] `ros2 topic hz /vrpn/<刚体>/pose` 稳定 **~100–400 Hz**（动捕主机在线时）
      —— ✅ **实测 199.9 Hz**。⚠️ 但**是在 `U_Tracker0` 上测的**：`Z0` 当前根本不推数据（见 Comments §三），
      速率验收**对 z0 本体**留给 [ticket 12](12-z0-mocap-stream-check.md)
- [x] 刚体名由 VRPN **自动发现**，不需要在配置里写死
      —— ✅ 实测自动发现 **28 个 sender → 84 个话题**，`params.yaml` 里没有任何名字
- [x] **没有改动 vendored 源码的任何一行**（这是 vendored 的定义）
      —— ✅ 9 个文件逐个 sha256 与 zip 一致

---

## Comments

### 2026-09-21 —— ★★★ 完成。顺带判定了 ticket 01 留给本张的 UDP/TCP 未决项，并挖出两件影响 ticket 04 的事实

> 执行者：agent，在 z0 的 Jetson（`orin-nx`）上。**全部结论为第一手读数。**
> 期间有三种系统侧改动，逐条列在 §四。

#### 一、产出

| 文件 | 说明 |
|---|---|
| `src/vrpn_listener/**`（9 个文件） | **从钉住的 zip 解出，逐个 sha256 与 zip 一致**（`VENDORED.md` §4 有全量表）。提取方式 `unzip -q <zip>`，zip 顶层目录就是 `vrpn_listener/` |
| `src/vrpn_listener/VENDORED.md` | 🆕 新增文件（**不属于 vendored 集合**）。含：来源 zip + commit `b7aeebb` + zip md5、**三个定制点**（`stamp = now()` / `handleTwist` 角速度留零 / `frame_id` 参数化）、**"勿替换为公开版"**及理由、一条可重跑的字节一致性复核命令、依赖与运行、验收记录 |
| 工作区骨架 | **colcon 根 = 仓库根**（与 `mocap-setup.md` §10 第 3 项"改成 `~/ros_ws`"一致）。`build/` `install/` `log/` 已被仓库根 `.gitignore` 挡住（`git check-ignore` 逐条实测） |
| `README.md` | 目录树里 `src/` 那行（原写"Phase 1 待建"）更新为现状 + 构建命令 |

**来源核对**：`~/refs/Sim2real-UAV-Params-/` 副本 HEAD = `b7aeebb71cc708445fbac70a4863c736d4674630`
（2026-08-30），与 ticket 钉的 `b7aeebb` **一致**；`vrpn_listener.cpp` 8088 B，
第 37–40 行的 `stamp = listener->now()` **连注释一字不差**。

#### 二、五条验收项

| # | 判据 | 结果 |
|---|---|---|
| 1 | `colcon build` 干净通过；产物不进 git | ✅ 清空 `build/ install/ log/` 后重编：**0 warning / 0 error**（编译选项含 `-Wall -Wextra -Wpedantic`）。产物 `install/vrpn_listener/lib/vrpn_listener/vrpn_listener`（aarch64 ELF） |
| 2 | `VENDORED.md` 三条齐 | ✅ |
| 3 | `ros2 topic hz` 100–400 Hz | ✅ **`average rate: 199.901` / `199.989`**（两次采样）。⚠️ **测的对象是 `U_Tracker0`，不是 `Z0`** —— 原因见 §三 |
| 4 | 刚体名自动发现 | ✅ **28 个 sender → 84 个话题**，`config/params.yaml` 里**没有任何名字**（源码 `refresh_trackers()`，`vrpn_listener.cpp:109-125`） |
| 5 | vendored 源码一行未改 | ✅ §一的 sha256 表 |

> ⚠️ **这张 ticket 原来那句"What to build"里说的「把上游那份定制过的 `vrpn_listener` **原样 vendor 进来并让它跑起来**」——
> 跑起来这件事**做成了**（进程起来、发现 sender、发布话题、速率达标），
> **但"z0 的位置真的在这条链路上流动"这一条，被 `Z0` 无流挡住了**（§三）**。不要把它读成已经飞过。**

#### 三、★★ 挖出的两件事（都影响 ticket 04）

##### (a) 传输层 = **UDP** —— ticket 01 的未决项闭合

| 证据 | 读数 |
|---|---|
| `ss -tunap`（**vrpn_listener 进程**） | **3 个 UDP** 连到 `10.1.1.198:3883` |
| 同进程的 TCP | `LISTEN 10.1.1.114:51185` + 一条 ESTAB，**对端 `10.1.1.198:52182`** ⟹ 是**动捕主机主动连进本机监听口**的，不是我们发起 |
| 抓包（`AF_PACKET`，本机 `wlP1p1s0`） | 动捕主机 5 s 内向本机 UDP `:36039`（= 该进程的 socket）推 **2997 个包**（2978 个载荷 1400 B）⟹ 服务器**在主动推** |
| 独立 VRPN 客户端（`vrpn_Tracker_Remote("robot0@10.1.1.198:3883")`） | 5 s **961 次** pose 回调 ⟹ **≈192 Hz** |

⟹ **`mocap-setup.md` §1 数据流图的 `VRPN/UDP(3883)` 是对的，不用改。**
⚠️ 原先"TCP 答 banner、UDP 无应答"**是探测方法的问题**：VRPN 的 UDP 不认裸 socket 发的随机报文
（要先做它自己的端口协商）。**"裸探针没回应"不能推出"那条路不通"** —— 与本文档第三轮那条教训同源，
这次是靠**真 `vrpn_listener`** 才判出来的，正如当初所要求的那样。

##### (b) ☠️ 刚体名是 **`Z0`（大写）**，且 **`Z0` 当前没有数据流**

`sender` 名单里与 z0 相关的：**`Z0`**（本体）+ `Z0_Marker1`…`Z0_Marker5`（**单颗反光球各自也是独立 sender**，
各自会生成 `/vrpn/Z0_MarkerN/{pose,twist,accel}`）。同服务器上还有别的项目的
`robot0/robot1` 及其 markers、`U_Tracker0..9`、`VRPN Control`。

- ⚠️ **下游话题是 `/vrpn/Z0/pose`**，而 `mocap-setup.md` §10 第 1 项原记的是 `z0`（小写）⟹ **已在该处就地更正**。
  话题名**直接取自 VRPN 的 sender 名**，`replaceSpace()` **只把空格换成 `_`、不改大小写**。
- ★ **`Z0` 出现在名单里 ≠ 它在出数**：**5 s 内 0 次回调**（独立 VRPN 客户端实测），
  而同一次运行里 `robot0` / `U_Tracker0` 各 **961 次**。名单来自服务器的对象列表，
  出数要求该刚体**正在被追踪**（通常是**飞机此刻在动捕棚有效范围内、Nokov 侧已开流**）。
- ⟹ **这是物理侧的状态，不是本张的缺陷**，但它**挡住了第 3 条验收对 z0 本体的验证** ⟹ 另开
  [ticket 12](12-z0-mocap-stream-check.md)（`ready-for-human`）。

#### 四、系统侧做了三处改动（都会影响以后的构建，必须知道）

| # | 改动 | 为什么 |
|---|---|---|
| 1 | `sudo apt install ros-humble-vrpn`（**7.35.0**，装到 `/opt/ros/humble`） | 本机原本**没有** `libvrpn`（`code-inventory.md:135` 记的"已实测本机无 `libvrpn`"仍成立）。**用户裁定走 apt，不走 `mocap-setup.md` §2.1 的源码编译** |
| 2 | `sudo ln -sfn /opt/ros/humble/lib/libvrpn.a /usr/local/lib/libvrpn.a` | ★ **不做这一步就编不过**：本包 `CMakeLists.txt` 里有一行 `target_link_libraries(vrpn_listener vrpn)` —— **裸名字**，链接器展开成 `-lvrpn`，**要求 `libvrpn` 落在 ld 默认搜索路径里**；apt 那份在 `/opt/ros/humble/lib`，不在。**那一行是 vendored 的，不能改**，所以只能在系统侧搭桥（`/usr/local/lib` 在 ld 的 `SEARCH_DIR` 里，已核实；这也正是上游 `sudo make install` 的布局）。**只链了 `libvrpn`**：`libquat` 全程以绝对路径引用 |
| 3 | 无 | 没动设备树、没动串口、没动 `params.yaml` |

> `apt install` 的尾部有一条 `Error: GDBus.Error:...UnitMasked: Unit packagekit.service is masked` ——
> 那是 Ubuntu 的**安装后触发脚本**在抱怨 `packagekit` 服务被 mask，**与本包无关**，`Setting up ros-humble-vrpn` 已经成功。

#### 五、文档落点

| 文档 | 改了什么 |
|---|---|
| [`mocap-setup.md`](../../../docs/reference/mocap-setup.md) §10 第 1 项 | 刚体名就地更正为 **`Z0`**（原写 `z0`），并指出下游话题是 `/vrpn/Z0/pose` |
| [`mocap-setup.md`](../../../docs/reference/mocap-setup.md) 文末 | 🆕 「**ticket 03 实测**」一节：传输层判定 = UDP（含 §一 四条证据）、`Z0` 大小写与无流事实、VRPN 版本 7.34↔7.35 只差小版本号可忽略 |
| [`README.md`](../../../README.md) 目录树 | `src/` 那行更新 + 构建命令 |
| `src/vrpn_listener/VENDORED.md` | 新建（§一）+ 验收记录一节 |

#### 六、遗留

- ⏳ **对 `Z0` 本体的速率验收** → [ticket 12](12-z0-mocap-stream-check.md)（人把 z0 放进动捕棚 / 在 Nokov 里开流 + 跑一条命令）
- 🔶 **本张未碰**：`mocap-setup.md` §8 的 systemd 自启动那一套（不是本张范围）；`Z0_Marker1..5` 这些话题**下游要不要屏蔽**（现在是每条都建 publisher，28 sender 全开会白占带宽）——**留给 ticket 04 权衡**
- 🔶 **`ros2 topic hz` 没有消息时不打印任何行**（含 "subscribed to"）—— 排查时别把它当成"命令挂了"

#### 七、code review 的 9 条**范围外**发现（本张未修，登记以免丢失）

收尾跑 `/code-review` 回来 **10 条**，其中 **1 条是本张改动直接造成的**（`mocap-setup.md` §2.1/§2.2
还在教源码编译 VRPN 和"从别的机器拷包"，与本张新写的 `VENDORED.md` §5 冲突）—— **已就地修掉**（§五）。
**剩下 9 条全部落在本张没碰过的文件上，属于既有问题**，按"跨文件清扫不代劳"的约定**不在本张修**，列在此处供认领：

| # | 落点 | 症状（一句话） |
|---|---|---|
| 1 | `docs/PROJECT.md:433` | ADC 缺陷那段指挥读者去重刷一个**实际不受影响**的固件 |
| 2 | `README.md:87` | 同一条 ADC 断言，方向与 PROJECT.md 相反 |
| 3 | `.scratch/phase3/spec.md:38` | 仍写 `5ba2e91f` fork 的刷写计划（已被 `4a0e65f2` 定案推翻） |
| 4 | `tools/prereq-evidence/uart_loopback.py:261` | 默认端口含**未修**的 `ttyTHS2` ⟹ 会给出误导性失败 |
| 5 | `tools/prereq-evidence/mavlink_probe.py:432` | 硬编码已作废的 `5ba2e91f` |
| 6 | `docs/reference/params-analysis.md:263` | 同一条过时断言 |
| 7 | `tools/prereq-evidence/capture.sh:56` | `sed -n '2,30p'` 会漏代码行 |
| 8 | `tools/prereq-evidence/dtb_pio_fix.sh:191` | 自校验硬编码 LINUX/INITRD |
| 9 | `tools/prereq-evidence/README.md:98` | "两个端口都受影响"已过时 |

其中 1/2/3/5/6 **同属"固件真身"与被它推翻的旧结论**一类 ⟹ 应并入
[ticket 10](10-docs-1-14-sweep.md)（跨文档清扫）；4/7/8/9 是工具本身的缺陷 ⟹ **尚无归属票**，需另开。
