# `prereq-evidence` —— z0 前置条件取证工具

这套脚本服务于 ticket [`../../.scratch/phase1/issues/01-hardware-prereqs.md`](../../.scratch/phase1/issues/01-hardware-prereqs.md)
（以及 spec 的「⚠️ 阻塞项：4 件必须由人物理完成的事」）。

**它不能替人做物理操作。** 它做的是：把「人到现场之后，把读数抄进文档」这一段变得机械、可复现、
且**不会抄错**。

## 为什么需要它

那 4 件事本身必须由人完成 —— 但其中「把值抄对、抄进正确的文档坑位」是可以自动化的，
而这一步恰恰最容易出错：③ 的版本号一旦转述就走样，④ 的电流读数一旦丢掉「当时未解锁」这个前提就整条作废。

## 快速开始

接上飞控、连好动捕 WiFi 之后：

```bash
# 0) 一次性准备（缺任一条 ③④ 都采不到）
sudo usermod -aG dialout "$(id -un)"     # 然后必须重新登录
python3 -m pip install --user pymavlink pyserial   # ⚠️ 两个都要；见下方「依赖」

# 1) 全量取证：网络 + 串口 + 版本 + 电流
bash tools/prereq-evidence/capture.sh --scan-serial

# 2) 想同时留一份原始档
bash tools/prereq-evidence/capture.sh --scan-serial --append-evidence
```

`capture.sh` 输出一段 markdown：**自带「落笔位置」提示**，你把里面的值人工复核后抄进对应文档即可。

## 单独用 MAVLink 探针

```bash
# ② 飞控挂在 THS1 还是 THS2 —— 逐口 × 逐波特率找心跳
python3 tools/prereq-evidence/mavlink_probe.py --scan

# ③④ 已知端口，直接读
python3 tools/prereq-evidence/mavlink_probe.py --port /dev/ttyTHS1
python3 tools/prereq-evidence/mavlink_probe.py --port /dev/ttyTHS1 --baud 921600
```

### ★ 「无心跳」不是一句话 —— 它有三种形状，动作完全相反

`--scan` 报「无心跳」时，每一格现在会**自己说出**是哪一种：

| 形状 | 判词 | 该做什么 |
|---|---|---|
| 0 字节 | 线路静默 | 确认线接的是哪个口 / 飞控上电了吗 |
| 有字节，但**非零字节最长连续段 < 17 B** | **多半是接收数据通路被打坏**（先排驱动缺陷，见下下节） | **先跑 `--internal`**：长模式全 0 ⟹ 驱动缺陷，改设备树；长模式正常 ⟹ 才去查 TX/RX 交叉、GND 共地。⚠️ **两种情况都别去调波特率** |
| 有字节，且有 **≥ 17 B** 的非零长段 | 像真数据 | 只是波特率不匹配，继续换波特率 |

> **17 的来历**：MAVLink v2 **最短的帧**（心跳、无 payload）也有 17 字节。
> 真数据流哪怕波特率全错，也必然出现 ≥17 字节的非零长段。
>
> ⚠️ **这一条是被误判教会才加的。** 2026-09-21：飞控线接到 `ttyTHS1` 后，
> 该口从 0 字节变成 ~13 kB/s，且 **@921600 的帧错误恰好为 0**（看着"完全正常"），
> 当时据此报了「有信号了」。但非零段最长只有 **9 字节** ⟹ 一个 MAVLink 帧都不可能有。
> **「读到很多字节」≠「收到数据」** —— 只看字节数就会掉进这个坑。
>
> ⚠️⚠️ **同一天稍后的更正**：当时把成因写成「**容性串扰**」—— **那是错的，已作废**。
> 长度扫描证明真相是**接收通路的 DMA 缺陷**（同上节）：发 16 字节原样收回 16 个 `0x00`，
> 短包（≤7 字节）却完全正常 —— 串扰不会这么"按长度分档"。
> **别再用"串扰"这个说法解释本机的现象。**

## 单独用 UART 回环自检

当 ② 表现为「**两条 UART 都读不到任何字节**」时，成因至少有三条，而**软件侧看起来一模一样**：

- **A** 飞控侧根本没在发（没上电 / 没启动完 / **那个口上没有 MAVLink 实例**）
- **B** 接线没落在 Jetson 的 UART RX 上（接错针 / TX-RX 没交叉）
- **C** 排针的 UART 功能没被 pinmux 打开（Tegra234 的 pinmux 由 **BPMP** 配置，
  内核 DT 里**查不到 pinmux 节点**，用户态无法核实）

`uart_loopback.py` 用**回环**把 C（以及"针脚认错"）从 A/B 里摘出来：

```bash
# ① 先做**不需要跳线**的一步：内部回环，验控制器活不活
python3 tools/prereq-evidence/uart_loopback.py --internal

# ② 把你接飞控的那两针短接（飞控侧两线先拔掉），验引脚/pinmux：
python3 tools/prereq-evidence/uart_loopback.py

# 对照组：未短接时跑，预期「收不到」—— 用来排除自激噪声
python3 tools/prereq-evidence/uart_loopback.py --no-loopback
```

> **`--internal` 通过** ⟹ 控制器 + 驱动 + 时钟正常。
> ⚠️ **但它绕过引脚**（在芯片内部成环），所以**不能**说明 pinmux/接线是通的。

### ★★ `--internal` 还会测一个**能直接掐断飞控链路**的缺陷

它跑**两档**：**7 字节**短模式（验控制器）+ **16 字节**长模式（验接收数据通路）。
长模式一失败，就是那个缺陷：

> **JetPack 6.x 的 `nvidia,tegra194-hsuart` 接收通路 DMA 有缺陷 ——
> ≥8 字节的接收数据会被打成 0。**
> 本机（z0 的 Jetson，L4T R36.5.0）**两个口都中招**：
> 发 16 字节 `01..10`，收回 **16 个 0x00**。

**为什么它致命**：MAVLink 帧**最短 17 字节** ⟹ 在 ≥16 档被**全部打成 0** ⟹
**一个心跳都解不出来**。这正是 `--scan` 报「无心跳」却「又像有信号」的真正原因。

**为什么它极难发现**：物理回环的**字节数分毫不差**（发 n 收回正好 n），
只有比内容才看得出全被抹成 0。只看「收没收到」会一路误判成
「线接对了 → 那是波特率不对 → 那是串扰」。

**修法**：设备树里删掉该串口节点的 `dmas` / `dma-names`，强制退回 PIO，然后重启
（内核日志会打印 `RX in PIO mode`）。

⚠️ 所以：**`--scan` 报「帧块残破」时，先跑 `--internal`，再谈接线。**
>
> **物理短接后收到** ⟹ Jetson 侧一路全通（引脚 + pinmux + 驱动 + 电平）⟹ 问题必在 **A 或 B**，
> **不要再怀疑 Jetson**。
> **收不到** ⟹ 问题在 **C** 或针脚认错，**与飞控无关**。

⚠️ 做回环时请先把飞控侧那两根线从排针上拔掉，否则你发的字节会进飞控的 RX（PX4 当垃圾丢弃、无害，但回环测不准）。

安全性：只往 TX 发一小段固定模式（物理回环 64 字节 `0x55 0xAA` 重复；内部回环 7 字节与 16 字节递增）并读回，
**不发任何 MAVLink 指令**。即使线还接在飞控上，飞出去的也只是几个无意义的字节。

## 修掉那个 DMA 缺陷：`dtb_pio_fix.sh`

`--internal` 报了「接收通路 DMA 缺陷」之后，**修它**用这个脚本
（完整施工图与判据见 ticket 第八轮）：

```bash
bash tools/prereq-evidence/dtb_pio_fix.sh build     # 不需要 sudo，不碰 /boot，自带自校验
DRY_RUN=1 bash tools/prereq-evidence/dtb_pio_fix.sh install   # ★ 空跑：产物落 dtb-work/，不碰 /boot
sudo bash tools/prereq-evidence/dtb_pio_fix.sh install   # 装 dtb + 往 extlinux.conf 加 FDT 行
sudo reboot
```

改的是 `serial@3100000`（`/dev/ttyTHS1`）的 `dmas` / `dma-names` —— **删掉它们**让驱动退回 PIO。

**为什么走 `extlinux.conf` 的 `FDT` 行**：这是 NVIDIA 官方的留存改法 —— UEFI 默认 extlinux 启动，
`FDT` 指定的 dtb **优先级高于**烧录分区里的；持久、不需要主机、不需要重刷。
（本机实测 `SecureBoot` efivar = 0，安全启动没开，所以这条路成立。）

| 子命令 | 要 sudo | 干什么 |
|---|---|---|
| `build` | 否 | 从 `/boot/dtb/` 的 dtb 生成改好的 dtb，断言「除目标节点外零差异」 |
| `DRY_RUN=1 install` | 否 | **同一条 install 路径的空跑**：产出落 `dtb-work/`，一个字节都不碰 `/boot` |
| `install` | 是 | 备份 `extlinux.conf` → 装 dtb → **新增** `LABEL primary-pio` 并设为默认 |
| `rollback` | 是 | 从最新备份还原 `extlinux.conf` 并删掉新 dtb |
| `status` | 否 | 现在的 `FDT` 行 + 内核跑的 DT 里还有没有 `dmas` |

**`install` 生成新启动项时的两条硬规矩**（2026-09-21 施工时补，此前 `install` 只有纸面设计）：

1. 新 LABEL 的 `APPEND` **照抄原 LABEL 原文**，不写裸 `${cbootargs}` ——
   原行带着 `root=PARTUUID=…`，自己写会把 `root=` 弄丢 ⟹ **开不了机**。
2. 新 LABEL **追加到文件末尾**，不做「插到最前面」—— 免得 `DEFAULT` / `TIMEOUT`
   这些全局项落到 `LABEL` 块之后，解析行为靠赌。

两条都有断言看着：改动行数必须 ≤1、原 `LABEL primary` 必须还在、`APPEND` 必须进得去。
原有的 `LABEL primary` 全程不动，所以最坏情况只是「回到原状」，不会把机器弄成不能启动。

> 验证这条 install 路径本身，可以拿**副本**练（不碰 `/boot`、不要 root）：
> ```bash
> T=/tmp/selftest; mkdir -p $T; cp /boot/extlinux/extlinux.conf.bak-* $T/extlinux.conf
> EXTLINUX_OVERRIDE=$T/extlinux.conf DRY_RUN=1 bash tools/prereq-evidence/dtb_pio_fix.sh install
> ```

## 输出都去哪

| 采集项 | 归宿 | 注意 |
|---|---|---|
| ① 网络 | [`docs/reference/mocap-setup.md`](../../docs/reference/mocap-setup.md) | 顺手补上「动捕那个 WiFi 的 SSID」—— 文档至今没写 |
| ② 串口 | ticket 01 勾选项 | 含 `dialout` 组那条，**要同步给 ticket 05** |
| ③ 版本 | [`docs/reference/refs-manifest.md`](../../docs/reference/refs-manifest.md) §2.5 | 版本号**原文**，不要转述 |
| ④ 电流 | [`docs/reference/params-changes.md`](../../docs/reference/params-changes.md) A4 | **必须带「当时未解锁」这个前提**，否则作废 |

### ③ 的一个额外收获：**从读数里直接读出固件真身**

`AUTOPILOT_VERSION.flight_custom_version` 里带 git hash，脚本解出来就是**固件的 commit 前 10 位**。
2026-09-21 这一步**真的定案了**：解出 `4a0e65f233` ⟹ commit
`4a0e65f2331b23b9c8352656e9ccb6edeaea5d6c`（`Minderring/PX4-Autopilot`，
"add micoair743-aio to PX4-Autopilot v1.14.3"）——
**推翻了 [`refs-manifest.md`](../../docs/reference/refs-manifest.md) §2.5 原先"fork `5ba2e91f`"的判定**
（全链见该节 §2.5(d)）。

> ⚠️ **它不是 ASCII 字符串**（第一手核实于**运行固件自己那棵树**的
> `src/lib/version/px_update_git_header.py:93`、`version.c:343`、
> `src/modules/mavlink/mavlink_main.cpp:1073-1078`）：
>
> ```c
> const uint64_t fw_git_version_binary = px4_firmware_version_binary() & 0xFFFFFFFFFF000000;
> const uint64_t fw_vendor_version = px4_firmware_vendor_version() >> 8;
> memcpy(&msg.flight_custom_version, &fw_git_version_binary, 8);
> memcpy(&msg.flight_custom_version, &fw_vendor_version, 3);
> ```
>
> 即：塞进去的是 **uint64 的二进制镜像（小端）**；低 3 字节已被掩码砍掉；
> **前 3 字节又被厂商版本号覆盖**。所以只有 `[3:8]` 这 5 个字节是 hash，
> 逆序后得到 hash 的**前 10 个十六进制字符**。
>
> ⚠️⚠️ **本脚本中途曾写反过这道判据**：一度断言 `[3:8]` 应当是 **ASCII** 的 `0-9a-f`，
> 于是把上面那份**正确**的读数判成"取不到 hash"。**那道闸门是反向的错** ——
> `[3:8]` 本来就是二进制，它会挡掉任何真固件。已移除，并补了下面的自检。

### 自检（**不需要飞控、不需要串口**）

```bash
python3 tools/prereq-evidence/mavlink_probe.py --self-test
```

用真值对做交叉验证：**由 commit `4a0e65f2…` 正向推出的 8 字节，必须逐字等于飞控实测的
`00 00 00 33 f2 65 0e 4a`**，且解码必须还原出 `4a0e65f233`。
若有人再往里加"必须是 ASCII"之类的闸门，这里会立刻红。

共 **7 项**：正向编码、解码那 8 字节、hash 前 10 位、厂商版本号只覆盖前 3 字节**不污染**
hash 段、`encode → decode` 往返，以及**两条不同的**「取不到 hash」路径
（整段全 0 / 只有 `[3:8]` 全 0）—— 后者原先没被自检覆盖，是 2026-09-21 的 code-review 查出来的。

### ④ 的判定规则

未解锁时电流应 ≈0。**恒为 0 或恒为怪值 = 中招**（`boards/hkust/nxt-dual` 的 ADC 通道缺陷）。
中招后电流/电量保护全是空转，且**参数无法绕过**（编译期宏）。

> ⚠️ **2026-09-21 修正「怎么修」**：原先写"修复 = fork 自带的 `pr_fix_hkust_nxt_dual`
> @ `8161c4cd`，只差一行" —— **对 z0 的固件不适用**。飞控跑的固件树里**没有 `hkust/`**，
> 它的 `boards/micoair/h743-*` 把电流通道写在 `/* PC1 */ ADC1_CH(11)`，**与引脚相符、无此缺陷**。
> ⟹ 本机「未中招」不是因为运气，**是这条缺陷不在这块固件里**。
> Phase 3 要重编固件时，**该编哪块板还是个待定项**（见 ticket 11 的 ①）。

**电压通道未受影响**，Phase 1 可用「电压 + 计时」兜底。

## 安全纪律

**两个脚本都是只读的。** `mavlink_probe.py` 只发两种指令，且都与飞行状态无关：

- `MAV_CMD_SET_MESSAGE_INTERVAL` (511) —— 设遥测下行速率
- `MAV_CMD_REQUEST_MESSAGE` (512) —— 请求重发某条消息

**绝不**发解锁 / 起飞 / 模式切换 / 参数写入。**采集时飞控可以不解锁，也应该不解锁**（④ 的前提）。

## 依赖（⚠️ 一个实测踩过的坑）

**`pymavlink` 不自带 `pyserial`。两个都要装。**

2026-09-21 在 z0 的 Jetson（`orin-nx`）上第一手核实：`pymavlink 2.4.49` 的
`Requires-Dist` **只有 `lxml` 与 `fastcrc`**。只装 `pymavlink` 的后果特别误导 ——
`import pymavlink` 会**成功**，直到真正打开串口才炸
`ModuleNotFoundError: No module named 'serial'`，而那已经是 ②③④ 的采集阶段了。

```bash
python3 -m pip install --user pymavlink pyserial
```

> `capture.sh` 的预检已改成**分别检查两者**（缺哪个报哪个）。旧版只查 `pymavlink`，
> 会在缺 `pyserial` 时**误报「✅ 已装」**。

## 为什么用 pymavlink 而不是手搓解码器

MAVLink v2 的 CRC 带 per-message `CRC_EXTRA` 种子。种子写错**不会报错、只会静默丢帧** ——
手搓等于赌一张背下来的种子表。pymavlink 是权威实现，不赌。

（本仓库的 `~/refs/` 里都是 fetch-only 副本，没有 mavlink 生成头，
所以本地也无法第一手核对种子表 —— 这正是不能用记忆里的表的理由。）

## 验证状态（诚实交代）

| 部分 | 状态 |
|---|---|
| Tier A：网络 / 串口枚举 / 权限 | ✅ **已在 z0 的 Jetson（`orin-nx`）上实跑验证**，含 `dialout` 权限被拒的真实报错路径。**2026-09-21 二次实跑**：复现了 `pyserial` 缺失的真实报错（见「依赖」），并暴露出旧的 `pymavlink`-only 预检会误报 —— 两处均已修 |
| Tier B：`--port` 读 ③④ | ✅ 已用 **pymavlink 在 pty 上模拟的假飞控**端到端验证（含 git hash 解码、电流为 0 的「疑似中招」判定） |
| Tier B：`--scan` 找端口 | ✅ 同上验证（假飞控 + 真实权限报错两种路径都走到） |
| Tier B：**对真飞控** | ✅ **2026-09-21 端到端成功**：`/dev/ttyTHS1` @ 921600 听到 `HEARTBEAT`（`autopilot=12`、`type=2`），并读到 `AUTOPILOT_VERSION`（③）与 6 个 `BATTERY_STATUS` 样本（④）。**首跑失败的真正原因是接收通路 DMA 缺陷**（见下两条），修完才通 |
| `mavlink_probe.py --self-test` | ✅ **2026-09-21 通过**：按源码规则由 commit `4a0e65f2…` **正向推出**的 8 字节与飞控实测逐字相同，解码还原出 `4a0e65f233`；全 0 字节仍正确报"取不到" |
| Tier B：**「不是数据流」判据** | ✅ **2026-09-21 在 z0 上第一手触发**：`ttyTHS1` 接到飞控线后读到 ~13 kB/s、帧错误 0，但非零段最长仅 **9 B** → 正确判为「**不是数据流**」，**没有**误报成「波特率不对」。这正是加这条判据的理由。<br>⚠️ **该判据当前的分档已不能只按"噪声"讲** —— 后来查明成因是**接收通路的 DMA 缺陷把真数据打坏**（见下条与 ticket 第六/九轮），所以「非零段 < 17 B」这一档既可能是噪声，**也可能是真 MAVLink 被打坏**。判词已据此加了前提（先跑 `--internal` 排除驱动缺陷，再谈接线） |
| `uart_loopback.py`：对照组（`--no-loopback`） | ✅ **已在 z0 的 Jetson 上实跑**：未短接时两口均收不到，符合预期 |
| `uart_loopback.py`：**内部回环短模式** | ✅ **已在 z0 的 Jetson 上实跑**：两口 7 字节均原样收回 ⟹ 控制器/驱动/时钟正常（⚠️ **它绕过引脚，证明不了 pinmux/接线**） |
| `uart_loopback.py`：**长模式（测 DMA 缺陷）** | ✅ **已在 z0 上第一手触发并复现**：两口发 16 字节均收回 16 个 0x00 ⟹ 接收通路 DMA 缺陷坐实 |
| `uart_loopback.py`：真回环 | ✅ **2026-09-21 在 z0 上实跑**：短接「串口1」（= `ttyTHS1`）后，**收回字节数精确等于发出字节数** ⟹ 引脚/pinmux/电平全通；而内容在 ≥8 字节被打成 0 ⟹ 坐实是驱动缺陷而非接线 |
| `uart_loopback.py --internal --port /dev/ttyAMA0` | ⚠️ **试过，作废** —— 该驱动（`sbsa-uart`）不实现 `TIOCM_LOOP`，**短模式**就收不回来，得不出任何结论。操作者已决定**放弃这条线** |
| `dtb_pio_fix.sh build` | ✅ **2026-09-21 在 z0 上实跑通过**（含全部断言）：往返无损、除 `serial@3100000` 外全文零差异、体积 `250051 → 250003` 字节 |
| `dtb_pio_fix.sh install`（空跑） | ✅ **2026-09-21 实跑通过**（含副本上的三条自校验测试：正常路径 / 幂等挡板 / 缺 `APPEND` 拒绝） |
| `dtb_pio_fix.sh install`（真装 + 重启） | ✅ **2026-09-21 在 z0 上端到端验证通过**：`DEFAULT` → `primary-pio`、末尾追加 `LABEL primary-pio`（含 `FDT`）⟹ **能正常开机**（`root=PARTUUID=…` 没丢）⟹ 重启后 `dmas`/`dma-names` **已消失**、内核日志打出 **`RX in PIO mode` / `TX in PIO mode`** ⟹ 缺陷修复被 `uart_loopback.py` 双向证实（长模式 16 字节、物理回环 64 字节均原样收回） |
| `dtb_pio_fix.sh rollback` | ❌ **尚未在真机上执行过** —— 照官方文档写的，首次 rollback 请注意有无报错 |

所以：**首次对真飞控跑时，请留意脚本有没有报错或给不出读数**，那是预期内的首跑风险，
不是"已验证过所以一定对"。
