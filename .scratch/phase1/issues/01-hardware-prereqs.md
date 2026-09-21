# 01: 前置人工核实 —— 动捕网络 / 飞控串口 / 固件版本 / 电流读数

Status: done
Type: task

> ✅ **2026-09-21 完成**（① 第二轮、② 第九~十一轮、③④ 第十一轮 —— 四条结论全部落进 `docs/reference/`）。
> ⚠️ **一处未决事项已另开 ticket**：[11 飞控自报 1.14.3 vs 文档的「v1.15 世代」](11-firmware-version-conflict.md) ——
> 它是 ③ 的读数带出来的冲突，**不影响本张的完成**，但**卡住 Phase 2 的 [01](../../phase2/issues/01-sitl-px4-msgs-baseline.md)**。

**What to build:** 四件必须由人物理完成、agent 代劳不了的事，全部在写任何代码之前做完 —— 因为后面每一张 ticket 的结论都以它们为前提：

1. 把 Jetson 连到**动捕所在的那个 WiFi**（不是校园 802.1x），验证能 ping 通动捕主机
2. 确认飞控接的是哪个串口设备节点
3. 在 QGC 记录**飞控实际报出的固件版本号**
4. 在 QGC 看**未解锁时的电池电流读数**

**为什么要做：**
- ①② 是整条链路的物理前提。当前实测路由走校园网关 `10.31.255.254`，`ping 10.1.1.198` **100% 丢包**；实测本机只有 `/dev/ttyTHS1` 与 `/dev/ttyTHS2` 两个串口（没有 `/dev/ttyACM*`、`/dev/ttyUSB*`），必须先确定是哪个。
- ③ 是**版本问题的唯一终审**。文档旧称 z0 跑 "PX4 1.14.x" 已被第一手判定推翻为 **v1.15 世代**（见 [`refs-manifest.md`](../../../docs/reference/refs-manifest.md) §2.5），但**以飞控自己报出的为准**。它同时是 Phase 2 选 `px4_msgs` 版的依据。
- ④ 核实 `nxt-dual` 板已知的 **ADC 电流采样通道缺陷**是否中招。未解锁时应 ≈0；**恒为 0 或恒为怪值 = 中招**，那样电流/电量保护全是空转，而该缺陷**参数无法绕过**（编译期宏），只能重编重刷固件。

⚠️ **③④ 互为前提**：QGC 只能通过串口连上飞控，所以"先看版本"这件事本身依赖 ②。

**Blocked by:** 无（可立即开工）

- [x] Jetson 在动捕网段内，`ping 10.1.1.198` **0% 丢包**（不再走校园网关）—— ✅ 2026-09-21 实测 5/5，见下方第二轮
- [x] 记录飞控实际使用的串口设备节点，并确认它可用
  - 节点**已定案**：`/dev/ttyTHS1`（= `serial@3100000` = 载板丝印**串口1**），见第五、六、七轮
  - **Jetson 侧已修好并验证通过**（第十轮）：设备树删掉 `dmas`/`dma-names` 退回 PIO 后重启，
    内部回环长模式 16 字节、物理回环 64 字节**均原样收回**，内核日志坐实 `RX in PIO mode` /
    `TX in PIO mode`。**修前这两档分别是 16 个与 64 个 `0x00`**
  - ✅ **飞控接回后收到心跳，② 闭合**（第十一轮）：`HEARTBEAT system=1 component=1 type=2
    autopilot=12`（PX4 四旋翼）——见第十一轮
- [x] 飞控版本号**原文**记入 [`refs-manifest.md`](../../../docs/reference/refs-manifest.md) §2.5(c)
  - ⚠️ **本项有一个未决冲突**：飞控自报 `flight_sw_version = 0x010e0300` ⟹ **`1.14.3`**，
    而 §2.5(b) 据 fork **源码**判的是「v1.15 世代」。§2.5(c) 记了两边读数与三种未验证的假设，
    **没有挑一个当结论**。**Phase 2 的 `px4_msgs` 选版在 S1 定案前必须先解决它**
  - ⚠️ 原 ticket 写的是「QGC 读到的」—— 实际是**直接读 MAVLink 字段**（同样第一手，且更原始）
- [x] 未解锁时的电池电流读数记录在案 + 判定 ⟹ **未中招**，记入 [`params-changes.md`](../../../docs/reference/params-changes.md) A4
  - 6 样本 `[37,38,36,37,36,36]`（36…38 **有变化**）⟹ 不满足「恒为 0 / 恒为怪值」。
    **当时未解锁**已由 `base_mode = 0x1d`（bit7 未置位）第一手确认
- [x] 四条结论**全部落进文档** —— ✅ ① `mocap-setup.md`、② 本文件 + `hardware.md`、
  ③ `refs-manifest.md` §2.5(c)、④ `params-changes.md` A4

---

**为什么这张不是 `ready-for-agent`**：物理操作（改网络、读 QGC 屏幕、看串口设备），AFK agent 做不了。
它的产出是**信息**，不是代码；产出的归宿是 `docs/reference/` 下那几份取证文档。

---

## Comments

### 2026-09-21 —— agent 在 z0 的 Jetson（`orin-nx`）上做的第一手侦察

**结论：四项 ✅ 一个都没勾。** 四件都是物理操作，agent 做不了；以下是**没能完成的原因证据**，
以及把人的现场工作压缩到最小的产出。

| # | 状态 | 第一手证据 |
|---|---|---|
| ① | ❌ **未完成** | 仍连 `SUSTech-802.1x`，`10.29.12.59/14`，默认网关 `10.31.255.254`。`ping 10.1.1.198` = **100% 丢包**（网关本身通，≈3.5 ms，所以不是网断了）。有线口 `enP8p1s0` = **NO-CARRIER**。 |
| ② | ⚠️ **定不了，但范围已缩小** | 无 `/dev/ttyACM*`、无 `/dev/ttyUSB*`、无 `/dev/serial/by-id/`；**`lsusb` 里根本没有飞控**（只有 root hub + AX200 蓝牙 + Terminus hub）→ 飞控**不是 USB 接的**，只能是 `ttyTHS1`/`ttyTHS2`（内核日志 `3100000.serial` / `3140000.serial`，Tegra SoC UART）。**而且 SoC UART 永远不会有 `/dev/serial/by-id/` 别名** —— 别再等它出现。 |
| ③ | ❌ **本机做不到** | 无 QGC、无 pymavlink / mavsdk / mavproxy。 |
| ④ | ❌ **同上**，且依赖 ③ 的串口链路。 | — |

**★ ① 缺的具体是「SSID 名字」。** 全量重扫后，剔除校园 SSID 与隐藏名，在范围内的候选有：
`407lab`、`504509D`、`508office`、`508office-5G`、`BearWifi`、`CLEARLab-2.4G`、`DIRECT-8F-HP`、
`SUST-XY-605-HUAWEI-AX6`、`TPLINK_7A79_2.4G`、`TPLINK_7A79_5G`、`Xuyang_Office`。
[`mocap-setup.md`](../../../docs/reference/mocap-setup.md) 第 5 项只写了"连到动捕主机所在的 WiFi"，
**从未写出那个 WiFi 叫什么** —— 而这是唯一剩下要人认定的信息。
⚠️ 这份名单是**机械剔除的结果，不是"动捕网就在其中"的结论**，必须人到现场指认。

**★ 附带的阻塞发现（影响 ticket 05，与本张无直接关系）**：用户 `chen` **不在 `dialout` 组**
（`dialout:x:20:` 成员为空），而 `/dev/ttyTHS*` 权限是 `root:dialout 660`。
**→ 就算 ② 定了案，串口也打不开。** 已在 05 记录。

**★ 本轮的代码产出**：新增 [`tools/prereq-evidence/`](../../../tools/prereq-evidence/)（取证脚本，
**只读**）。它不是本张 ticket 要求的产出（本张要求的是信息），是为了让人到现场后**一次采完、抄对**：
- `capture.sh` —— 网络 / 串口枚举 / 权限 / `dialout` 体检，输出**自带落笔位置提示**的 markdown
- `mavlink_probe.py` —— `--scan` 自动答 ②（逐口 × 逐波特率找心跳），并把结果
  （`RESULT: <port> <baud>`）回喂给 ③④，避免"② 刚说是 THS2、③④ 却去开 THS1"；
  `--port` 读 ③④，并从 `flight_custom_version` 解出 **git hash 前 10 位**，
  可直接核对 `refs-manifest.md` §2.5 的 `5ba2e91f`。
  ⚠️ 该字段是**二进制小端**、且前 3 字节被厂商版本覆盖（第一手核实：上游 v1.15.0
  `30e763b` 的 `mavlink_main.cpp:1082-1087`），**不是 ASCII** —— 首版实现按 ASCII 解，
  已修
- ④ 的电流**采 6 个样本**再判定：A4 的判据是「**恒**为 0 / **恒**为怪值」，
  单个样本区分不了"缺陷导致的恒定值"和"正常的安静值"，样本 <3 个时**明确不给判定**

**⚠️ 本轮刻意没做的事**：没有往 [`refs-manifest.md`](../../../docs/reference/refs-manifest.md) §2.5
和 [`params-changes.md`](../../../docs/reference/params-changes.md) A4 里填任何占位值 ——
那两个坑是 ③④ 实测读数的归宿，**填"待测"会污染取证链**。等飞控上电后再落笔。

---

### 2026-09-21 —— 第二轮：**① ✅ 闭合；② 仍未定案，但阻塞点已被精确刻画**

> 本轮由 agent 执行，其间有**一次人工授权**：`chen` 提供 sudo 口令，
> 跑了你授权的两条 —— `usermod -aG dialout chen` 与 `setfacl -m u:chen:rw /dev/ttyTHS{1,2}`。

#### ① ✅ **完成**（第一手读数，已落进 [`mocap-setup.md`](../../../docs/reference/mocap-setup.md) §10 末尾）

| 项 | 读数 |
|---|---|
| 已连 WiFi | **`TPLINK_7A79_2.4G`**（接口 `wlP1p1s0`）—— **这就是此前文档从未写出的那个 SSID** |
| 本机地址 | `10.1.1.114/24` |
| 默认网关 | `10.1.1.1`（**不再走校园网关 `10.31.255.254`**） |
| `ping -c5 10.1.1.198` | ✅ **0% 丢包**（5/5，RTT 1.7–37.1 ms） |
| ARP | `10.1.1.198 lladdr e8:9c:25:1d:b8:64 REACHABLE` |

配套：**刚体名已确定为 `z0`**（操作者确认），**飞控侧走 TELEM2**（操作者确认）。两条均已记入 `mocap-setup.md` §10 第 1/2 项。

#### ② ⛔ **仍未定案 —— 但已排除三类原因，只剩物理层**

权限打通后实测（**没有任何 permission 错误**，端口确实被打开）：

| 检查 | 读数 |
|---|---|
| `/dev/ttyTHS1` / `/dev/ttyTHS2` 打开 | ✅ 均成功 |
| `mavlink_probe.py --scan`：2 口 × 3 波特率（57600/115200/921600）找 HEARTBEAT | ❌ **6 个组合全无** |
| **原始字节嗅探（3 s，任意波特率）** | ❌ **两口均 0 字节 —— 线路完全静默** |
| 端口被占用 | ❌ 无（无 `serial-getty@ttyTHS*`、无 mavros 在跑） |
| 设备树 `status` | `serial@3100000`（ttyTHS1）/ `serial@3140000`（ttyTHS2）**均 `okay`** |

> **已排除**：权限（✅ 通）、**波特率**（波特率错也会读到乱码字节，而这里是 0 字节）、端口占用（✅ 无）。
> → **结论：没有任何信号到达 Jetson 的 RX 线。**

剩余可能，**三条都需要物理动作，agent 做不了**：

- **A 飞控没有真正在发** —— 未上电 / 未启动完 / **TELEM2 上没有 MAVLink 实例**。
  ★ **这条最可能，且有一个本项目特有的陷阱**：`z0.params` 是**参考机**的参数文件，
  README 明确标它"待上机核实"。**若 z0 实际是 `MAV_1_CONFIG = 0` 或 `SER_TEL2_BAUD = 0`，
  PX4 根本不会往 TELEM2 发任何字节** —— 与观测完全吻合。而"读参数"本身又需要先有链路（鸡生蛋）。
- **B 接线没落在 Jetson 的 UART RX 上** —— 接错针，或 TX/RX 没有交叉。
- **C 40 针排针的 UART 功能未被 pinmux 打开** —— 内核 DT 里**查不到 pinmux 节点**
  （Tegra234 的 pinmux 由 **BPMP** 单独配置，用户态无法核实）。

**★ 把 A/B/C 分开的判据（30 秒实验）**：在排针上把 UART 的 **TX 与 RX 短接**做回环，然后我发字节读回显 ——
**读得到** ⇒ Jetson 侧（引脚 + pinmux + 驱动）全通，问题必在 **A 或 B**；**读不到** ⇒ 问题在 **C** 或针脚认错。

**★ 同日追加：重启 Jetson 后重测 + **内部回环**（不需要跳线）**

- 操作者重启过 Jetson（`/proc/uptime` = **195.9 s**）。**重启对症状毫无影响**：
  `lsusb` 仍无飞控、`/dev/ttyACM*` / `ttyUSB*` / `serial/by-id/` 仍不存在、
  `--scan`（2 口 × 3 波特率）与原始嗅探（5 s，**0 字节**）结果与重启前**完全一致**。
  > 副作用：`setfacl` 随 devtmpfs 重建**失效**，但 `dialout` 组身份随新会话**生效** —— 权限反而更干净。
- **新增判据：UART 内部回环（`TIOCM_LOOP`）—— 不需要任何跳线。**
  两个口都 **1–7 字节原样回环成功** ⇒ **控制器 + 驱动 + 时钟都活着**。
  ⚠️ 但内部回环在**芯片内部**成环、**绕过引脚**，所以它**证明不了引脚/pinmux 有没有接出去** ——
  **判定引脚仍然只能靠物理短接。**
- 顺带实测到一个**未查明**的行为（**不下结论、不外推**）：发 **≥8 字节**时，
  **前 8 字节被抹成 0、第 8 字节之后的内容正确**（发 `01..0c` 收回 `0000000000000000 09 0a 0b 0c`），
  两个口行为一致。机制未知，形似 RX DMA 路径的产物。工具因此改用 7 字节短模式规避。

→ **② 的下一步没变，而且只剩一条路**：物理短接那两针，或者走飞控 USB。

#### ③④：**仍未采集**（依赖 ②）

> ⚠️ **本轮新发现、影响 ticket 03**：动捕主机 `10.1.1.198` 的 **TCP** `3883` 会答 **VRPN banner**
> （`vrpn: ver. 07.34  0`），且 135/139/445 开放 ⇒ **动捕主机是 Windows**；
> 但 **UDP `3883` 完全无应答**（8 个刚体名 × 3 种报文格式），且 banner 之后 12 s 不推任何消息。
> `mocap-setup.md` §1 的数据流图默认写的是 `VRPN/UDP(3883)` ——
> **传输层到底是 UDP 还是 TCP，待 ticket 03 用真 `vrpn_listener` 判定。**

#### 本轮代码产出：修掉工具的一个**真缺陷**

`pymavlink` **不自带 `pyserial`** —— 第一手核实：`pymavlink 2.4.49` 的 `Requires-Dist`
**只有 `lxml` 与 `fastcrc`**。旧预检只查 `import pymavlink`，会**误报「✅ 已装」**，
直到 ②③④ 采集阶段才炸 `ModuleNotFoundError: No module named 'serial'` ——
正是"人到现场才发现"的那种坑。已改 [`capture.sh`](../../../tools/prereq-evidence/capture.sh)
（分别检查两者，缺哪个报哪个）、[`mavlink_probe.py`](../../../tools/prereq-evidence/mavlink_probe.py)
（文件头依赖说明）、[`README.md`](../../../tools/prereq-evidence/README.md)（新增「依赖」节）。

---

### 2026-09-21 —— 第三轮：② 的「有信号了」是**误判**；证据指向**接线未交叉**

> ⚠️⚠️ **本轮的因果结论（"容性串扰 / RX 没被驱动"）已被第六轮推翻。**
> 那些零字节**不是串扰，是真数据被接收通路的驱动缺陷打坏了**。
> 本轮保留下来的只有**原始读数**（帧错误、非零占比、游程），**结论部分作废**。

> ⚠️ **先纠正上一轮的一句话。** 操作者把 TELEM2 接到机载电脑串口 1 之后，
> `/dev/ttyTHS1` 从「0 字节」变成了「每秒上万字节」，agent 当时据此报了
> **「有信号了！线接对了」**。**这句话是错的** —— 字节不是数据。以下是把话说死的过程。

#### ① 飞控侧配置：**排除**（本轮首次拿到第一手参数）

`z0.params` **就在仓库里**（[`docs/assets/z0.params`](../../../docs/assets/z0.params)），
只是参数名是**第三个字段**（格式 `1\t1\tPARAM\tvalue\ttype`），按行首 grep 会误判"没有"。
实际：

| 参数 | 值 | 行 |
|---|---|---|
| `MAV_1_CONFIG` | **102**（= TELEM2） | :504 |
| `SER_TEL2_BAUD` | **921600** | :893 |
| `SER_TEL1_BAUD` | 921600 | :892 |

→ 第二轮里那个「`MAV_1_CONFIG = 0` 所以 PX4 根本不往 TELEM2 发」的陷阱，**被本仓库自带的文件证伪**。
（但该文件头部自标「**尚未在 z0 的飞控上核实**」，所以它是**强证据、不是第一手**。）

#### ② ★★ 决定性判据：**非零字节的最长连续段**

`ttyTHS1` 上那些字节，用三条独立的量刻画（每档 2 s，`TIOCGICOUNT` 取内核计数器）：

| 波特率 | 读到的字节 | 非零占比 | 帧错误 | break | **非零最长连续段** |
|---|---|---|---|---|---|
| 57600 | 1041 | 58% | 396 | 0 | **6 B** |
| 115200 | 2730 | 39% | 690 | 0 | **8 B** |
| 230400 | 6351 | 10% | 827 | 0 | — |
| 460800 | 12859 | 8% | 649 | 0 | — |
| **921600** | **25854** | **6%** | **0** | 0 | **9 B** |
| 1500000 | 28846 | 4% | 853 | 0 | — |

**判据**：MAVLink v2 **最短的帧**（心跳，无 payload）也有 **17 字节**。
任何**真数据流**（哪怕波特率完全错）都必然出现 ≥17 字节的非零连续段。
而这里最长只有 **9 字节**，分布集中在 2/4/6（偶数、极短）。

> ⟹ **线上不是数据流。** 无论波特率设成什么，都不可能有 MAVLink 帧。
> 「读到很多字节」和「收到数据」是两回事 —— 这正是上一轮误判的根因。

#### ③ ★ 旁证：@921600 的形状，指向**串扰**

最误导的一格是 `@921600`：**帧错误 = 0**（接收器锁住了位时钟，看着"完全正常"），
但 **95% 是 0x00**，非零段最长 6–9 字节。这与**容性串扰**的特征吻合：

- 串扰源就**在旁边的导线上以 921600 跑**（`SER_TEL1_BAUD` / `SER_TEL2_BAUD` 都是 921600）
  ⟹ 感应波形的**位时钟与 921600 一致** ⟹ 接收器不报帧错误；
- 容性耦合**只传边沿、不传直流电平**，长串相同位会丢失 ⟹ 非零段被切得极碎，且**幅度低**，
  多数位读成 0 ⟹ 大段 0x00。

**对照**：`ttyTHS2`（没接线）**干净 0 字节** ⟹ 这些字节是 `ttyTHS1` 那条线特有的，不是软件伪影。

#### ④ 结论与动作（② 仍未闭合，但下一步是唯一且明确的）

**最可能的成因：Jetson 的 RX 引脚没被驱动** —— 即**接线没交叉**
（Jetson RX 接到了飞控的 **RX**，两端都是输入、谁都不驱动），或 **GND 没共地**。

> ⚠️ **此时不要去调波特率。** 波特率错会给出 ≥17 字节的长帧块，**不是这个形状**。
> 形状对不上，说明问题在**接线**。

**要人做的（按优先级）**：
1. **把两根信号线在**某一端**对调**（Jetson RX ↔ 飞控 TX），并**确认 GND 共地**，然后重跑 `--scan`。
2. ★ **更快的一条：把飞控的 USB（Type-C 调参口）插到 Jetson 上。**
   PX4 **在 USB 上永远跑 MAVLink**，与 `MAV_1_CONFIG` 无关 ⟹ 会直接出 `/dev/ttyACM*` ⟹
   **③④ 不需要 TELEM2 就能采到**，而且能**第一手读出 `MAV_1_CONFIG` / `SER_TEL2_BAUD`**，
   把上面「强证据但非第一手」的参数核实掉。
   > 这条把 ②③④ 一起解开，是当前**性价比最高**的动作。
3. 若走 USB，② 的答案就是「USB 那条链路」；TELEM2 那条线的交叉问题**仍要修**（Phase 1 要用它）。

#### ⑤ 本轮代码产出：把判据固化进工具

`--scan` 以前只会说「无心跳」—— 那句话**把三种情况混成一种**，正是它让我误判的。
现已在 [`mavlink_probe.py`](../../../tools/prereq-evidence/mavlink_probe.py) 里加
`_raw_characterize()` / `_explain_raw()`，每一格现在直接给出**可执行的**判据：

| 形状 | 判词 | 动作 |
|---|---|---|
| 0 字节 | 线路静默 | 确认线接的是哪个口 / 飞控上电了吗 |
| 有字节、非零段 **< 17 B** | **不是数据流**（噪声/串扰/RX 未驱动） | **查 TX/RX 交叉与 GND**，别调波特率 |
| 有字节、非零段 **≥ 17 B** | 像真数据 | 只是波特率不匹配，继续换 |

`RESULT: <port> <baud>` 机器可读行**未改动**（`capture.sh` 依赖它）。

---

### 2026-09-21 —— 第四轮：★「串口0」是**调试口**，不是 `ttyTHS1`

操作者做了物理回环（短接），但**测不到回环**。查 device tree 的 aliases 后，
原因不是接线，而是**短接的不是同一个控制器**：

| alias | 指向 | `compatible` | 对应设备 |
|---|---|---|---|
| **`serial0`** | `/serial` | **`nvidia,tegra234-tcu`** | **调试串口**（combined UART）—— `console=ttyTCU0,115200`；`/dev/ttyTCU0` 存在但权限 `600`（内核 console 占用），**不是给应用用的口** |
| `serial1` | `/bus@0/serial@3100000` | serial-tegra | **`/dev/ttyTHS1`** ← 飞控所在 |
| `serial2` | `/bus@0/serial@3140000` | serial-tegra | `/dev/ttyTHS2` |

`/proc/device-tree/chosen/stdout-path` = **`serial0:115200n8`** —— 与 `serial0` = TCU 吻合。

> ⚠️ **`serial0` 不在 `bus@0` 下**，`compatible` 也完全不同（`tegra234-tcu` vs `serial-tegra`）。
> **"串口0"和"串口1"不是同一类东西** —— 前者是调试口，后者才是接飞控的 SoC UART。
> 这与操作者的编号习惯一致（他说"串口1"时，活动的正是 `serial1` = `ttyTHS1`）。

**排除项**：`/dev/ttyS0`–`ttyS3` 是 `serial8250` 的平台占位
（`/sys/devices/platform/serial8250`，无真实硬件 backing），**不要**把它们当成候选串口。
`serial@3110000` 是 `disabled`。

**要人重做的**：把短接改到**接飞控的那个口（串口1 = 排针上对应 `ttyTHS1` 的两针）**，
飞控侧两线先拔掉，再跑 `uart_loopback.py`。这一次的结果才是有效的判据：

- **收到回显** ⟹ Jetson 侧全通 ⟹ 毛病在飞控/接线侧（回到第三轮的 TX/RX 交叉假设）
- **收不到** ⟹ 该口的引脚 / pinmux 有问题 ⟹ 与飞控无关

#### ⚠️⚠️ 上面这节（第四轮）的结论**已被推翻** —— 见第五轮

---

### 2026-09-21 —— 第五轮：★ 找到真正的根因 —— **「串口0」的控制器在设备树里是 `disabled`**

#### 先纠正第四轮的错误

第四轮把**载板丝印的「串口0」**和**设备树的 alias `serial0`** 当成了同一个东西，据此断言
「串口0 = 调试口 TCU」——**这个结论是错的**。两者是两套互不相干的编号：

| 编号体系 | `0` | `1` | `2` |
|---|---|---|---|
| **设备树 alias** | `/serial` = **TCU 调试口**（`console=ttyTCU0`，无 `/dev` 节点） | `serial@3100000` = `ttyTHS1` | `serial@3140000` = `ttyTHS2` |
| **载板丝印** | **模组 UART0，管脚 99/101** | 模组 UART1，管脚 203/205 | 模组 UART2，管脚 236/238（= Debug） |

操作者手册写明「串口0 的 RX/TX 是模组 **101 / 99** 管脚」，与 NVIDIA 官方引脚表吻合
（DG-10931-001 §12.3 Table 12-7：99 = `UART0_TXD`、101 = `UART0_RXD`）。
**丝印的「串口0」= 模组 UART0，不是 TCU。**

#### ★ 根因：模组 UART0 → `serial@3110000`，而我们这台机器上它是 `disabled`

`serial@3110000` = **uartb = 模组 UART0**（SoC 侧 UART2，引脚 PX04–PX07）。
本机设备树实测：

| 节点 | status | `/dev` 节点 |
|---|---|---|
| `serial@3100000` | `okay` | `ttyTHS1`（alias `serial1`） |
| **`serial@3110000`** | **`disabled`** | **无！连 alias 都没有** |
| `serial@3140000` | `okay` | `ttyTHS2`（alias `serial2`） |
| `serial@31d0000` | `okay` | TCU 调试口 |

> ⟹ **短接「串口0」永远测不出回环** —— 那个控制器**根本没有被实例化**，
> `/dev` 下不存在对应节点。第四轮的回环实验**方法没错，但打在了一个死口上**，
> 所以「收不到」这一结果**不能**用来推断接线或 pinmux。

**这同时解释了一个悬了很久的疑点**：`mocap-setup.md` §5.2 记着上游用的是
**`/dev/ttyTHS3`**。而 uartb 一旦启用并给它 alias `serial3`，节点名正好就是 `ttyTHS3`。
⟹ **上游很可能就是把飞控接在「串口0」（模组 UART0）上，并在自己的 DT 里启用了 uartb**；
z0 没做这一步，所以 `ttyTHS3` 不存在 —— 与本轮实测完全吻合。
（**这是强推断，不是第一手** —— 需要启用 uartb 后实测验证。）

#### 两条下一步（并行）

1. **便宜且决定性：改短「串口1」（= 飞控所在的口），重跑 `uart_loopback.py`。**
   - 通过 ⟹ 串口1 = `ttyTHS1` 且 Jetson 侧全通 ⟹ 问题在飞控/接线侧
   - 不通过 ⟹ 串口1 不是 `ttyTHS1`（或该口 pinmux 未开）
2. **可能才是真正的修法：在设备树里启用 `serial@3110000`**（并加 alias）。
   官方/社区的步骤是 `status = "okay"` + `aliases { serial3 = ... }`，编 dtb 后放到
   `/boot/dtb` 并在 `extlinux.conf` 里引用 —— **需 sudo + 重启**。
   ⚠️ **已知坑（与本机高度相关）**：JetPack 6.x 上 `serial-tegra` 走 DMA 时**数据会损坏**，
   社区做法是**删掉该节点的 `dmas`/`dma-names` 强制退回 PIO**。
   > ★ 这条坑与本文件第二轮记的那个「**≥8 字节时前 8 字节被抹成 0**」的未查明行为
   > **形状高度吻合**（DMA 路径产物），值得在启用 uartb 时一并留意。

#### 对仓库文档的修正（本轮已落地）

- `hardware.md` 第 8 节的「Jetson Orin NX 8GB」**是对的**（设备树 `nvidia,p3767-0001`
  = Orin NX 8GB 模组）；操作者一度口述为「Orin Nano」，已由本人更正为 **Orin NX**。
- ⚠️ 仍存疑：设备树自报载板为 `nvidia,p3768-0000`（**NVIDIA 官方 devkit 载板**），
  而 `hardware.md` 写的是「微空 kit-miao ORIN NX【V2】底板」。两者不一致，
  推测第三方底板是 p3768 的兼容克隆（故刷原厂 DTB 可跑）。**未核实，先记下。**
- 操作者手册确认：**串口1 = 模组 203/205**（= 模组 UART1），与 NVIDIA 官方表一致 ✅。
  飞控接的就是串口1。

---

### 2026-09-21 —— 第六轮：★★★ **② 的根因查明 —— 接收通路的 DMA 缺陷**

操作者按第四轮的要求把**串口1（= 模组 203/205 = `ttyTHS1`）短接**，飞控两线已拔。
本轮拿到的是**决定性证据**。

#### 一、证据：长度扫描（发 0x55×n，固定读 300 ms）

| 发出 n | `ttyTHS1` 收回 | 内容 | `ttyTHS2`（对照，未短接） |
|---|---|---|---|
| 1 / 2 / 4 / 7 | **正好 n** | **全 0x55 ✅ 完好** | 0 字节 |
| **8** | 8 | **全 0x00** | 0 字节 |
| 9 | 9 | 0x55×1 / **0x00×8** | 0 字节 |
| 12 | 12 | 0x55×4 / **0x00×8** | 0 字节 |
| 16 … 512 | **正好 n** | **全 0x00** | 0 字节 |

**两条铁的事实：**

1. **收回的字节数永远精确等于发出的字节数** ⟹ **物理回环 100% 成功** ⟹
   `串口1` 的**引脚 + pinmux + 驱动 + 电平全部正常**。
   ⟹ 第四轮"该口 pinmux 可能没开"、第三轮"RX 没被驱动"**都被排除**。
2. **内容在 ≥8 字节时被打成 0** ⟹ 这是**接收数据通路**的缺陷，**与接线无关**。

#### 二、机制：是 DMA

本机 DT 实测：

```
serial@3100000（= ttyTHS1 = 串口1）
  compatible = nvidia,tegra194-hsuart      ← HSUART 驱动
  dma-names  = "rx tx"                     ← 收发都走 DMA
serial@3140000（= ttyTHS2）同样带 dmas / dma-names = "rx tx"
```

已知问题（JetPack 6.x / `nvidia,tegra194-hsuart`）：**`serial-tegra` 走 DMA 时接收数据会损坏**。
官方/社区的修法是**删掉该节点的 `dmas` / `dma-names` 强制退回 PIO**
（内核日志会打印 "RX in PIO mode"）。本机正是 L4T R36.5.0 / JetPack 6.2。

#### 三、★★ 推翻第三轮：那不是串扰

第三轮把 `ttyTHS1 @921600` 的「~13 kB/s、帧错误 0、94% 是 0x00」判为**容性串扰**。**错的。**

既然接收通路会把 ≥8 字节打成 0，那么那些零字节的**更合理解释**是：
**飞控真实发出的 MAVLink 被 DMA 打坏了** —— 「帧错误 0」正因为**信号是真的 921600**，
「94% 零」正因为**帧的内容被抹掉**。

> **MAVLink 帧最短 17 字节** ⟹ 在 ≥16 档被**全部打成 0** ⟹ **一个心跳都解不出来**。
> 这就是 ② 卡了整整六轮的真正原因：**症状极具误导性** ——
> 物理回环的字节数**分毫不差**，只看「收没收到」会一路误判成「线接对了 / 波特率不对 / 串扰」。

#### 四、工具已改（两处，都是被这次误判教会的）

1. **`uart_loopback.py --internal` 现在故意去测它**（初版是用 ≤7 字节**绕开**这个行为，方向反了）。
   加了一档 **16 字节长模式**：短模式验控制器、长模式验接收通路。
   ★ **不需要任何跳线**即可复现 —— 本机两个口都报
   「长模式回环数据被打坏：发 16 字节，收回 16 个 0x00」。
2. **`mavlink_probe.py` 的「不是数据流」判词加了前提**：那一条**分不开**「线上是噪声」与
   「真数据被驱动打坏」。现在它要求**先跑 `uart_loopback.py --internal`**，再谈接线。

#### 五、下一步（② 的修法）

| 选项 | 内容 | 代价 |
|---|---|---|
| **A. 改设备树退回 PIO** | 删 `serial@3100000` 的 `dmas`/`dma-names`，编 dtb → `/boot/dtb` → `extlinux.conf` 引用 | sudo + **重启**；**这才是 Phase 1 走 TELEM2 的正解** |
| **B. 先走飞控 USB** | Type-C 调参口插 Jetson ⟹ `/dev/ttyACM*`。**完全绕过 Tegra UART** | 无重启；可立即采 ③④ |

**未定项**：`dma-names = "rx tx"` 两向都有 —— 但本次实验**只能证明接收被打坏**，
**发送是否也受影响尚未验证**（回环里 TX→RX 是同一颗芯片，分不开）。
若 TX 也坏，mavros→飞控的上行指令同样不可信 —— 修 DMA 是两向一起修的，所以不改变动作。

---

### 2026-09-21 —— 第七轮：★ 发现**第三个 UART** `/dev/ttyAMA0`（不带宽带缺陷）

#### 先纠正第五轮的一处错误

第五轮的表里把 `serial@31d0000` 写成了「TCU 调试口」—— **错了**。实测：

| 节点 | 驱动 | `/dev` 名 |
|---|---|---|
| `/serial`（`nvidia,tegra234-tcu`，mbox） | `tegra-tcu` | `ttyTCU0` ← **这才是 TCU** |
| **`serial@31d0000`**（`arm,sbsa-uart`） | **`sbsa-uart`** | **`ttyAMA0`** |

`ttyAMA` 是 **ARM PL011 / SBSA 兼容 UART** 的设备名（树莓派上的"主串口"即此名）。
**它是一颗独立的第三个 UART，此前所有轮次都没测过它。**

#### ★ 关键：它**没有** DMA 属性

| 节点 | `compatible` | `dmas` / `dma-names` |
|---|---|---|
| `serial@3100000`（ttyTHS1） | `nvidia,tegra194-hsuart` | **有**（`rx tx`）← 缺陷在此 |
| `serial@3140000`（ttyTHS2） | `nvidia,tegra194-hsuart` | **有** |
| **`serial@31d0000`（ttyAMA0）** | **`arm,sbsa-uart`** | **无** |

⟹ **ttyAMA0 大概率不受第六轮那个接收缺陷影响**，是一个潜在的**绕开方案**。
⚠️ 但**尚未实测**：`sbsa-uart`（PL011 系）**不认 8250 的 `TIOCM_LOOP` 内部回环**——
本机实测该口短模式 7 字节也收不回来，所以那条"不需要跳线"的自检对它**不适用**，
只能靠**物理短接**判定。

**未定**：这颗 UART 在载板上**是否接到了可用连接器**（p3768 devkit 上它通常不作为
通用口引出）。`--scan` 纳入它后仍无心跳，但那是因为飞控没接在它上面，**不构成否定**。

#### 本轮回答操作者的三个问题（结论固化）

| 问 | 答 | 依据 |
|---|---|---|
| 串口1 现在能正常通信吗？ | **能"通"，不能"用"** —— 物理层全好，但 ≥8 字节的接收内容被打成 0；≤7 字节短包完全正常 | 第六轮长度扫描 |
| 能和飞控 TELEM2 通信吗？ | **现在不行**。MAVLink 帧最短 17 字节 ⟹ 全部被打成 0 ⟹ 一个心跳都解不出。**修掉 DMA 后应该能**（待验证） | 第六轮 + `--scan` 全灭 |
| 串口0 能测通信吗？ | **测不了** —— `serial@3110000` 是 `disabled`，内核未创建 platform device，`/dev` 下无节点。**没有东西可以打开** | `/sys/bus/platform/devices/` 无 `3110000.serial` |

---

### 2026-09-21 —— 第八轮：★★ **设备树改动的施工图**（照此执行，不必回溯前七轮）

> **本节是给「新开一个对话来把 ② 做完」用的。** 前七轮是诊断过程，本轮是**施工件**：
> 结论、确切改法、逐条命令、验证判据、回退方式，全部自足。
> 配套脚本：[`tools/prereq-evidence/dtb_pio_fix.sh`](../../../tools/prereq-evidence/dtb_pio_fix.sh)

#### 0. 一句话现状

飞控 TELEM2 接在**串口1**（= 模组 203/205 = `/dev/ttyTHS1` = DT 的 `serial@3100000`）。
该口**物理层全通**（物理短接的字节数分毫不差），但**接收方向 ≥8 字节的内容会被内核驱动打成 0**
（第一手：发 16 字节 `01..10`，收回 16 个 `0x00`）。MAVLink 帧**最短 17 字节**
⟹ 所有帧被全部打成 0 ⟹ **一个心跳都解不出来**。这就是 ② 至今不通过的**唯一原因**，与接线无关。

**修法 = 设备树里删掉该节点的 `dmas` / `dma-names`，让驱动退回 PIO。**

#### 1. 本轮新采的第一手事实

`dtc` 解开的 DT 原文（全部串口节点，`/boot/dtb/kernel_tegra234-p3768-0000+p3767-0001-nv.dtb`，
md5 `fcc01bf9a07489a32b636443804d6bb2`，与 `/boot/kernel_…-nv.dtb` **字节相同**）：

| 节点 | `/dev` | `compatible` | 驱动 | `dmas` | `status` |
|---|---|---|---|---|---|
| `serial@3100000` | `ttyTHS1` | `nvidia,tegra194-hsuart` | `serial-tegra` | **有** | okay |
| `serial@3140000` | `ttyTHS2` | `nvidia,tegra194-hsuart` | `serial-tegra` | **有** | okay |
| `serial@31d0000` | `ttyAMA0` | `arm,sbsa-uart` | `sbsa-uart` | 无 | okay |
| `serial@3110000` | *（无节点）* | `nvidia,tegra234-uart` + `nvidia,tegra20-uart` | `8250_tegra` | 无 | **disabled** |
| `/serial`（顶层） | `ttyTCU0` | `nvidia,tegra234-tcu` | `tegra-tcu` | 无 | okay |

★★ **DT 里只有两个 hsuart 节点带 `dmas`，而恰好只有这两个口有接收缺陷。** 这条相关性把
第六轮的结论钉死了。

内核配置（`/proc/config.gz` 第一手）—— 两套驱动都在内核里，**启用任何串口都不需要重编内核**：

```
CONFIG_SERIAL_8250=y          CONFIG_SERIAL_8250_TEGRA=y
CONFIG_SERIAL_TEGRA=y         CONFIG_SERIAL_TEGRA_TCU=y
```

目标节点改动前后的确切原文：

```dts
// 改动前
serial@3100000 {
        compatible = "nvidia,tegra194-hsuart";
        reg = <0x00 0x3100000 0x00 0x10000>;
        interrupts = <0x00 0x70 0x04>;
        clocks = <0x03 0x9b>;
        resets = <0x03 0x64>;
        dmas = <0xed 0x08 0xed 0x08>;      // ← 删
        dma-names = "rx\0tx";              // ← 删
        status = "okay";
        reset-names = "serial";
        phandle = <0x21f>;
};
```

#### 2. 官方留存方法 —— 已核实，是 `extlinux.conf` 的 `FDT` 行

NVIDIA 官方文档（*Jetson Linux Developer Guide* → **Jetson Orin NX and Nano Series**）原文要点：

1. UEFI 默认就是 **extlinux 启动**，会去找 `/boot/extlinux/extlinux.conf` 里的 **`FDT` 条目**；
2. 在那里指定的 image / dtb **优先级高于**烧录在分区里的 dtb；
3. **没写、或文件不存在**时，才回落到**烧录分区**里的 dtb。

⟹ **官方留存的改法 = 在 `extlinux.conf` 里加一行 `FDT /boot/dtb/…-pio.dtb`。**
持久（重启、断电都在），**不需要主机电脑、不需要 recovery 模式、不需要重刷**。

- <https://docs.nvidia.com/jetson/archives/r36.2/DeveloperGuide/HR/JetsonModuleAdaptationAndBringUp/JetsonOrinNxNanoSeries.html>
- <https://forums.developer.nvidia.com/t/device-tree-dtbo-how-does-it-work/338605>
- <https://forums.developer.nvidia.com/t/jetson-orin-nx-r36-3-rs485/328497>（同款「缺 FDT 行」案例）

**本机现状（第一手）**：`/boot/extlinux/extlinux.conf` 里**没有 `FDT` 行**，
所以现在跑的是**分区兜底**的 dtb；而 `/boot/dtb/kernel_…-nv.dtb` 与 `/boot/kernel_…-nv.dtb` 是同一份。
**这正是"要加 FDT 行"的原因。**

⚠️ **唯一的例外**：安全启动开着时 `FDT` 行会被忽略、只能重刷签名分区。
**本机实测 `SecureBoot` efivar 值 = 0 ⟹ 未启用**，所以这条路对本机成立。

#### 3. 本轮明确**不做**的事（免得新对话再绕进去）

| 不做 | 理由 |
|---|---|
| 折腾 `/dev/ttyAMA0` | 操作者 2026-09-21 明示**放弃这条线**。它内部回环连**短模式**都收不回来（驱动不实现 `TIOCM_LOOP`），**任何结论都得不出来**，是纯粹的坑 |
| 启用 `serial@3110000`（串口0） | **它不解决任何当前问题** —— 飞控线在**串口1**上。等 ② 用串口1 打通后，如确需备用口再单开一轮（那会是**独立的一次改动**，便于区分成败） |
| 调波特率 / 再检接线 | 已被长度扫描排除：物理回环字节数分毫不差 ⟹ 接线与波特率都是对的 |

#### 4. 施工步骤（就这三步）

```bash
cd /home/chen/ros_ws

# ① 生成改好的 dtb + 自校验（**不需要 sudo，不碰 /boot**）
bash tools/prereq-evidence/dtb_pio_fix.sh build

# ② 装进 /boot/dtb/ 并往 extlinux.conf 加 FDT 行（**需要 sudo**；自动备份、保留原 LABEL）
sudo bash tools/prereq-evidence/dtb_pio_fix.sh install

# ③ 重启
sudo reboot
```

`install` 做的事：备份 `extlinux.conf` → 拷 dtb 到 `/boot/dtb/kernel_…-nv-pio.dtb`
→ **在最前面新增** `LABEL primary-pio`（含 `FDT` 行）→ 把 `DEFAULT` 指过去。
**原有的 `LABEL primary` 一字不动**，作为开机菜单里的回退项。

#### 5. 验证判据（重启后立刻跑，两条都要看）

```bash
# a) 芯片侧：长模式 16 字节能不能原样收回
python3 tools/prereq-evidence/uart_loopback.py --internal --port /dev/ttyTHS1
#    修好前：发 01..10，收回 16 个 0x00
#    ★ 修好后：**16 字节原样收回**

# b) DT 侧：dmas 属性是不是真的没了
bash tools/prereq-evidence/dtb_pio_fix.sh status

# c) 端到端：飞控心跳
python3 tools/prereq-evidence/mavlink_probe.py --port /dev/ttyTHS1 --baud 921600
```

- **a 通过 ⟹ 驱动缺陷已修好**（这是必要条件，与飞控无关）。
- **a 通过但 c 仍无心跳** ⟹ Jetson 侧无罪，问题才轮到飞控侧（上电？`MAV_1_CONFIG`？
  TX/RX 是否交叉？）—— **这时才该去查别的东西**。
- **a 不过** ⟹ 改动没生效（`status` 看 FDT 行；看 `dmesg | grep 3100000` 有没有 `PIO mode`）→ 回退。

#### 6. 回退（两种，任一即可）

```bash
# A. 开机时在 extlinux 菜单里选 "primary kernel"（TIMEOUT 30 秒，来得及）
# B. 盲操作还原：
sudo bash tools/prereq-evidence/dtb_pio_fix.sh rollback && sudo reboot
```

`LABEL primary` 与分区里的原 dtb **全程未被动过**，所以最坏情况是"回到现在这个状态"，
**不会把机器弄成不能启动**。

#### 7. 已经做过的干跑证据（本轮，零风险）

| 验的东西 | 结果 |
|---|---|
| `拆 → 编 → 再拆` 能否无损往返 | ✅ 重编前后 dts 文本完全一致 |
| 删属性后节点原文 | ✅ `dmas` / `dma-names` 消失，`status = "okay"`、`compatible` **原样保留** |
| 除目标节点外是否零改动 | ✅ 全文 `diff` 零差异 |
| 体积 | 250051 → 250003 字节（正好少了那两个属性） |
| `SecureBoot` efivar | ✅ 值 = 0 ⟹ `FDT` 行会被采用 |
| `dtb_pio_fix.sh build` | ✅ **已实跑通过**（含全部断言） |

⚠️ **诚实交代**：`install` 与 `rollback` 两条路径**尚未在真机上执行过** ——
它们是照官方文档写的，`build` 已验证。首次 `install` 前请注意有没有报错。
中间产物（dtb/dts）落在 `tools/prereq-evidence/dtb-work/`，**已加进 `.gitignore`**（可随时由脚本重建）。

#### 8. 还需人给的（不阻塞本轮施工）

- TELEM2 的 TX/RX **具体接到哪两针**（此前只确认"接在串口1 上"）。若 5-a 通过而 5-c 仍无心跳，
  这一条就是下一个要问的。
- 飞控那端的接线**是否交叉**（Jetson TX → 飞控 RX）。

---

### 2026-09-21 —— 第九轮：★★ **第八轮的施工已执行完毕，只差重启 + 验证**

> 操作者**已把串口1 重新短接**（飞控侧两线仍在拔掉状态）⟹ 本轮可以把"改动前"的指纹采干净，
> 并在重启后立刻用同一个物理条件验"改动后"。**本轮结束时的状态是「已装好、待重启」**。

#### 一、改动前基线（**短接在位**，第一手）

| 测法 | 读数 | 判词 |
|---|---|---|
| `uart_loopback.py --internal --port /dev/ttyTHS1` | 短模式 7 字节 ✅ 原样；**长模式 16 字节 → 16 个 `0x00`** | DMA 缺陷原样复现 |
| `uart_loopback.py --port /dev/ttyTHS1`（**物理**短接） | 发 64 字节，**收回 64 字节，全 `0x00`** | ★ **字节数分毫不差** ⟹ 跳线在位，**引脚 + pinmux + 驱动 + 电平全通**，坏的只有接收数据通路 |

⚠️ 注意工具对第二行的判词是「回环未通过 ⟹ 问题在 Jetson 侧或针脚」——**这句在本例里是误导**，
正是第六轮记的那个陷阱（它只看内容不看字节数）。**字节数相等这一条本身就证明引脚没问题**，
不要照着那句去查 pinmux。

#### 二、本轮执行的动作

```bash
bash tools/prereq-evidence/dtb_pio_fix.sh build          # ✅ 自校验全过，250051 → 250003 字节
DRY_RUN=1 bash tools/prereq-evidence/dtb_pio_fix.sh install   # ✅ 空跑，产出落 dtb-work/
echo 123 | sudo -S bash tools/prereq-evidence/dtb_pio_fix.sh install   # ✅ 真装
```

装完的现场（**第一手，`/boot` 实测**）：

| 项 | 值 |
|---|---|
| `/boot/extlinux/extlinux.conf` | `DEFAULT primary-pio`；末尾追加 `LABEL primary-pio`（含 `FDT /boot/dtb/kernel_…-nv-pio.dtb`） |
| 原 `LABEL primary` | **一字未动**（回退项） |
| 备份 | `/boot/extlinux/extlinux.conf.bak-20260921-082331` |
| 新 dtb | `/boot/dtb/kernel_tegra234-p3768-0000+p3767-0001-nv-pio.dtb`（250003 字节，`root:root 644`，与原件权限一致） |
| 与备份的 diff | **只差 `DEFAULT` 一行 + 末尾新增 12 行**，其余零改动 |

#### 三、★ 本轮发现并修掉了 `install` 路径的两个**会开不了机**的真缺陷

第八轮老实交代过 `install` 从没在真机上跑过。本轮首次执行前先做了空跑，抓到两条：

1. **新启动项原本写的是裸 `${cbootargs}`，会把 `root=` 弄丢。**
   本机原 `LABEL primary` 的 `APPEND` 是
   `${cbootargs} root=PARTUUID=1f0266a5-… rw rootwait rootfstype=ext4 …` 一长串。
   若新 LABEL 只写 `${cbootargs}`，而 UEFI 的 cbootargs 里不含 `root=`，
   内核**找不到根文件系统 ⟹ 开不了机**。
   → 已改成**照抄原 APPEND 行原文**，新启动项与"已知能启动的那一份"逐字一致，唯一差别是多一行 `FDT`。
2. **新 LABEL 原本插在文件最前面，会让全局项落到 `LABEL` 块之后。**
   `DEFAULT` / `TIMEOUT` 是全局项，惯例在 `LABEL` 之前；插到开头等于赌解析行为。
   → 已改成**追加到文件末尾**，文件其余部分只改动 `DEFAULT` 那一行。

配套给自校验加了**基准**：初版拿 `$EXTLINUX` 当基准，真装时它就是文件自己 ⟹ 统计恒为 `0/0`、
**断言空转**（首跑输出里那句"被删 0 行、新增 0 行"就是这个 bug 的现场）。现已改为以**备份**为基准。

**三条自校验测试（在副本上实跑，`DRY_RUN=1` + `EXTLINUX_OVERRIDE=`）**：

| 场景 | 期望 | 结果 |
|---|---|---|
| 正常路径 | 被删 1 行、新增 12 行，断言全过 | ✅ |
| 已被改过的文件（幂等） | **拒绝**，不许重复加 LABEL | ✅ 报"已经有 FDT 行，不覆盖" |
| 文件里没有 `APPEND` 行 | **拒绝**（怕开不了机） | ✅ 报"读不到原 APPEND 行" |

#### 四、⏭ **下一步就三件事**（重启后，按顺序）

> ⚠️ **重启前请人在机器旁**：extlinux 菜单 `TIMEOUT 30` 秒，万一新启动项有问题，
> 在菜单里选 **`primary kernel`** 即可回到现状。

```bash
# a) DT 侧：dmas 是不是真的没了（应打「已消失 ⇒ 改动生效，应该在跑 PIO」）
bash tools/prereq-evidence/dtb_pio_fix.sh status

# b) 芯片侧：长模式 16 字节能不能原样收回  ← 这是「缺陷已修」的必要条件，与飞控无关
python3 tools/prereq-evidence/uart_loopback.py --internal --port /dev/ttyTHS1
#    修前：16 个 0x00。★ 修后应为：01 02 … 10 原样收回

# c) 物理侧（短接还在位，正好接着用）：64 字节原样收回
python3 tools/prereq-evidence/uart_loopback.py --port /dev/ttyTHS1
```

**判词**：
- **a + b + c 全过** ⟹ ② 的 Jetson 侧**修好了**，可以**拔短接、把飞控两线接回串口1**，
  再跑 `mavlink_probe.py --port /dev/ttyTHS1 --baud 921600` 取心跳 ⟹ **② 闭合，同时解锁 ③④**
- **b 不过** ⟹ 改动没生效：`status` 看 `FDT` 行在不在、`dmesg | grep 3100000` 有没有 `PIO mode`
- **b 过了、c 不过是新情况**（不可能由 DMA 缺陷解释）⟹ 回来记一轮，别自己归因

#### 五、回退（两种，任一即可）

```bash
# A. 开机菜单（TIMEOUT 30 秒）里选 "primary kernel"
# B. 盲操作还原：
echo 123 | sudo -S bash tools/prereq-evidence/dtb_pio_fix.sh rollback && sudo reboot
```

#### 六、本轮**没做**的事

- **没有重启** —— 会话跑在 Jetson 本机上，重启会掐断它；且改动后首次开机需要人在菜单旁。
- **没有动 `serial@3110000`（串口0）与 `/dev/ttyAMA0`** —— 理由见第八轮 §3，本轮未推翻。
- **③④ 仍未采集** —— 仍卡在 ② （飞控线现在是拔掉的，且串口1 被短接占用）。

#### 七、提交前 code review 又抓出三条（均已修 + 已验）

| # | 缺陷 | 后果 | 修法 |
|---|---|---|---|
| 1 | `install` **先备份先装 dtb、后查挡板** | 重复跑一次：第 2 次会先把**已被改过**的 `extlinux.conf` 备份成「最新一份」再 die ⟹ 之后 `rollback`（取最新备份）把改过的配置当原件还原，而它引用的 `-pio.dtb` 已被 rollback 删掉 ⟹ **回退路径静默失效** | 两道挡板**移到备份/装 dtb 之前**。已用 root + 已改过的副本实测：die、退出码 1、**不产生任何新备份** |
| 2 | `rollback` 里 `ls … \| head -1` 没跟 `\|\| true` | `set -euo pipefail` 下无备份文件时 `ls` 返回 2、pipefail 让赋值失败 ⟹ **脚本静默死亡**，那句"找不到备份文件"永远打不出来 | 补 `\|\| true`（同文件别处早已这么写，就这处漏了） |
| 3 | 无参运行的帮助用 `sed -n '2,40p'` 写死行号 | 头部注释一变长就把 `set -euo pipefail` 等代码行当帮助打出来 | 改成打到 `set -euo pipefail` 之前为止，不再依赖行号。实测帮助里无代码行泄漏 |

另修了 `tools/prereq-evidence/README.md` 的验证状态表里一处**陈述过期**：
「『不是数据流』判据」那条仍把本机现象写成"正确判为**噪声/串扰**"，
而同一文件第 61–64 行早已作废"串扰"这个说法 —— 现已改为记录该档位
「既可能是噪声、**也可能是真 MAVLink 被 DMA 打坏**」这个后来才查明的双重含义。

> **review 另有 7 条落在本轮没碰的文件上**（`capture.sh` 漏枚举 `/dev/ttyAMA0`、
> `mavlink_probe.py` 的 `FC_SCAN_PORTS` 不 strip 与空 hash 前缀假阳性、
> `CLAUDE.md` 与若干 `docs/` 仍写 "PX4 1.14.x" 等），**本轮未动** —— 它们不属于 ② 的施工范围，
> 待单独开一轮或并入 ticket 10（1.14 清扫）处理。

---

### 2026-09-21 —— 第十轮：★★★ **修法生效，三条验证全过 —— ② 的 Jetson 侧彻底修好**

> 重启后（开机 `08:37:56`，uptime 173 s）跑第九轮 §4 那三条，**结果与预期完全一致**。
> **这是六轮诊断 + 两轮施工的收口**：接收通路 DMA 缺陷的「根因 → 修法 → 生效」链条闭合。

#### 一、三条验证（第一手，重启后实测）

| # | 命令 | 修前（第九轮基线） | **修后** |
|---|---|---|---|
| a | `dtb_pio_fix.sh status` | `dmas` 还在 | ✅ **`dmas` / `dma-names` 均已消失**，`status = okay`、`compatible` 未变 |
| b | `uart_loopback.py --internal`（长模式） | 发 16 字节 → **16 个 `0x00`** | ✅ **16 字节原样收回** ⟹ 接收数据通路保真 |
| c | `uart_loopback.py`（**物理**短接） | 发 64 字节 → **64 个 `0x00`** | ✅ **64 字节原样收回** ⟹ 引脚 + pinmux + 驱动 + 电平全通 |

#### 二、★ 机制坐实：内核日志原文（需要 sudo 才读得到）

```
[    2.165451] serial-tegra 3100000.serial: RX in PIO mode
[    2.165455] serial-tegra 3100000.serial: TX in PIO mode
[    2.165519] 3100000.serial: ttyTHS1 at MMIO 0x3100000 (irq = 112, base_baud = 0) is a TEGRA_UART
```

**收发两向都退回了 PIO** —— 第六轮 §5 留的那个未定项（「只能证明接收被打坏，发送是否也受影响未验证」）
就此有答案：DMA 缺陷是**收发两向一起**的，修也是两向一起修的。

#### 二·补 ★★ 同芯片 A/B 对照：唯一变量就是 `dmas`

重启后**同时**测了两个口（第一手，同一颗 SoC、同一个内核、同一个驱动）：

| 口 | DT 里有没有 `dmas` | 短模式（7 B） | 长模式（16 B） |
|---|---|---|---|
| **`ttyTHS1`**（`serial@3100000`） | **已删** | ✅ 原样 | ✅ **原样收回** |
| **`ttyTHS2`**（`serial@3140000`） | **仍在** | ✅ 原样 | ❌ **16 个 `0x00`** |

**这一对把因果钉死了** —— 两个口硬件同级、驱动同一份代码、跑同一个内核，
差异只有这一个设备树属性；而缺陷只跟着 `dmas` 走。
第六轮那条「DT 里只有这两个 hsuart 节点带 `dmas`，而恰好只有这两个口有缺陷」的相关性，
至此升级为**受控对照**。也顺带说明：`ttyTHS2` 现在**仍然是坏的**，谁要用它得做同样改动。

#### 三、顺带验证了第九轮那个「照抄 APPEND」的修法确实必要

`/proc/cmdline` 实测含 `root=PARTUUID=1f0266a5-9da8-4507-a1c4-250d6460b9ca` —— 能正常开机、
`root=` 没丢。若照初版脚本写裸 `${cbootargs}`，这一条就没了。

#### 四、⏭ ② 只差最后一步：接回飞控，取心跳

**Jetson 侧从此不再是嫌疑对象**（这正是第九轮那张判据表里的第一条分支）。
剩下三件事全在飞控侧：上电 / `MAV_1_CONFIG` / TX-RX 是否交叉。

要人做的（顺序别反）：

1. **拔掉串口1 的短接**
2. **把飞控 TELEM2 两线接回串口1**（模组 203/205）
3. 跑：
   ```bash
   python3 tools/prereq-evidence/mavlink_probe.py --port /dev/ttyTHS1 --baud 921600
   ```
   - **出心跳** ⟹ ② 闭合，**③④ 同一轮一起采**（版本号原文 → `refs-manifest.md` §2.5；
     未解锁电流 6 样本 → `params-changes.md` A4）
   - **仍无心跳** ⟹ 这时候**才**轮到查飞控侧：上电了没、`MAV_1_CONFIG` 是不是 102、
     TX/RX 有没有交叉（第八轮 §8 那两条待问项）。⚠️ 不要回头怀疑 Jetson —— 上面三条已排除它。

---

### 2026-09-21 —— 第十一轮：★★★ **② 闭合** —— 收到飞控心跳；③④ 一并采到

> 操作者接回 TELEM2，agent 直接读 MAVLink（不经过 QGC）。
> **六轮诊断 + 两轮施工，② 到此闭合。**

#### 一、② 闭合判据：心跳

```
✅ 收到 HEARTBEAT：system=1 component=1 type=2 autopilot=12 state=0x1d
   （autopilot=12 = PX4；type=2 = 四旋翼）
```

| 判定 | 依据 |
|---|---|
| 链路**双向物理通路**正常 | 心跳是飞控**主动发出**的，能被 Jetson 解出完整 MAVLink 帧 ⟹ RX 通路（本轮修复的那条）确实通了 |
| **不是**波特率碰巧 | `921600` = `SER_TEL2_BAUD`，一次命中，没有扫 |

⚠️ **一处仍未验证**：`mavlink_probe.py` **只读不发**，所以「Jetson → 飞控」方向的
**MAVLink 层**没被测过（物理层已被第九轮的物理回环证明）。
Phase 1 的 mavros 会上行发指令 —— 那是第一次真正上行，届时留意。

#### 二、③ 版本号 —— **采到了，但与文档结论冲突（未决）**

| 字段 | 读数 |
|---|---|
| `flight_sw_version` | `17695488`（`0x010e0300`）⟹ **`1.14.3`** |
| `flight_custom_version` | `00 00 00 33 f2 65 0e 4a` ⟹ **解不出 git hash**（见下） |
| `vendor_id` / `product_id` | `7052` / `54` |

★ **`flight_custom_version` 帮不上忙，而且工具原本会给出一份假证据。**
按上游源码，`[3:8]` 应是 git hash 的**前 5 个 ASCII 字符**（只可能是 `0-9a-f`）；
实测那 5 个字节是 `33 f2 65 0e 4a`，**含不可打印字符** ⟹ **不可能是 hash**。

> 工具初版只挡了"全 0"，于是把这一串**硬打成 `4a0e65f233`** 当成 hash 打了出来，
> 还配上判词「与 `5ba2e91f` 不一致 → §2.5 的结论要重开」。
> **那是一份格式完美、内容全假的证据** —— 正是本项目 §0 那两条教训的同款。
> 已给 [`mavlink_probe.py`](../../../tools/prereq-evidence/mavlink_probe.py) 补第二道闸
> （`b[3:8]` 必须全在 `0-9a-f` 内，否则报"取不到"），并复跑确认。

**冲突本身**：飞控自报 **1.14.3**，而 §2.5(b) 据 fork `5ba2e91f` 的**源码**判「v1.15 世代」。
§2.5 上一段把「读飞控实际报出的版本号」定为**唯一终审** ⟹ 终审与 (b) 不符。
**已在 [`refs-manifest.md`](../../../docs/reference/refs-manifest.md) §2.5(c) 记全**（含三种未验证的假设），
**没有挑一个当结论**。⚠️ **它直接挑战 §2.6 与 Phase 2 的 `px4_msgs` 选版前提，S1 定案前必须解决。**

#### 三、④ 电流 —— **未中招**

| 项 | 读数 |
|---|---|
| 当时是否解锁 | **未解锁** ✅（`base_mode = 0x1d`，bit7 `SAFETY_ARMED` 未置位） |
| 6 个样本 | `[37, 38, 36, 37, 36, 36]`（10 mA）→ 范围 36…38，**有变化** |
| 电压 / 已耗 / 剩余 | `20.159 V` / `23 mAh` / `16 %` |

⟹ 不满足「**恒**为 0 / **恒**为怪值」⟹ **未中招**。已记入
[`params-changes.md`](../../../docs/reference/params-changes.md) A4（含两点保留：
读数 ≈0.37 A 而非 0；本条**不推翻**上游的机制描述，只说明本机没被击中）。

#### 四、本轮产出的代码改动

| 文件 | 改动 |
|---|---|
| `mavlink_probe.py` | `_decode_git_hash_bytes` 加第二道闸（必须是 ASCII hash 字符），并改掉 `None` 分支的措辞 —— 不再可能"硬凑出一个假 hash" |

#### 五、⏭ 这张 ticket 完成后，下一个要人拍板的事

**③ 的冲突怎么收**。三条路（`refs-manifest.md` §2.5(c) 有详述）：

1. **读飞控完整参数表**，与 `docs/assets/z0.params` 对照 ——
   顺带把该文件头那句「尚未在 z0 的飞控上核实」**第一手结掉**，
   并核实 `MAV_1_CONFIG = 102` / `SER_TEL2_BAUD = 921600`（至今只有"强证据、非第一手"）。
   **代价最低、信息量最大，建议先做这一条。**
2. 拿分支源码直接看 `version.h` / `px4_firmware_vendor_version()`。
3. 等 DDS 通了直接看运行固件的话题字段布局（最硬，但要先搭 uXRCE-DDS）。

