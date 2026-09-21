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
| 有字节，但**非零字节最长连续段 < 17 B** | **不是数据流**（噪声 / 串扰 / RX 未驱动） | **查 TX/RX 是否交叉、GND 是否共地**；⚠️ **别去调波特率** |
| 有字节，且有 **≥ 17 B** 的非零长段 | 像真数据 | 只是波特率不匹配，继续换波特率 |

> **17 的来历**：MAVLink v2 **最短的帧**（心跳、无 payload）也有 17 字节。
> 真数据流哪怕波特率全错，也必然出现 ≥17 字节的非零长段。
>
> ⚠️ **这一条是被误判教会才加的。** 2026-09-21：飞控线接到 `ttyTHS1` 后，
> 该口从 0 字节变成 ~13 kB/s，且 **@921600 的帧错误恰好为 0**（看着"完全正常"），
> 当时据此报了「有信号了」。但非零段最长只有 **9 字节** ⟹ 一个 MAVLink 帧都不可能有。
> 实为**容性串扰**：串扰源就在旁边以 921600 跑，所以位时钟对得上、不报帧错误；
> 而容性耦合只传边沿不传电平，长串相同位丢失 ⟹ 非零段被切得极碎、大段 0x00。
> **「读到很多字节」≠「收到数据」** —— 只看字节数就会掉进这个坑。

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

安全性：只往 TX 发一小段固定模式（物理回环 64 字节 `0x55 0xAA` 重复；内部回环 7 字节递增）并读回，
**不发任何 MAVLink 指令**。即使线还接在飞控上，飞出去的也只是几个无意义的字节。

## 输出都去哪

| 采集项 | 归宿 | 注意 |
|---|---|---|
| ① 网络 | [`docs/reference/mocap-setup.md`](../../docs/reference/mocap-setup.md) | 顺手补上「动捕那个 WiFi 的 SSID」—— 文档至今没写 |
| ② 串口 | ticket 01 勾选项 | 含 `dialout` 组那条，**要同步给 ticket 05** |
| ③ 版本 | [`docs/reference/refs-manifest.md`](../../docs/reference/refs-manifest.md) §2.5 | 版本号**原文**，不要转述 |
| ④ 电流 | [`docs/reference/params-changes.md`](../../docs/reference/params-changes.md) A4 | **必须带「当时未解锁」这个前提**，否则作废 |

### ③ 的一个额外收获：能从读数里核对固件真身

`AUTOPILOT_VERSION.flight_custom_version` 里有 git hash，脚本能解出来，
**直接核对**它是不是 [`refs-manifest.md`](../../docs/reference/refs-manifest.md) §2.5 判定的 fork `5ba2e91f`。

> ⚠️ **它不是 ASCII 字符串**（这一点坑过一次，第一手核实于上游 v1.15.0 tag `30e763b`，
> `src/modules/mavlink/mavlink_main.cpp:1082-1087`）：
>
> ```c
> const uint64_t fw_git_version_binary = px4_firmware_version_binary() & 0xFFFFFFFFFF000000;
> const uint64_t fw_vendor_version = px4_firmware_vendor_version() >> 8;
> memcpy(&msg.flight_custom_version, &fw_git_version_binary, 8);
> memcpy(&msg.flight_custom_version, &fw_vendor_version, 3);
> ```
>
> 即：塞进去的是 **uint64 的二进制镜像（小端）**；低 3 字节已被掩码砍掉；
> **前 3 字节又被厂商版本覆盖**。所以只有 `[3:8]` 这 5 个字节是 hash，
> 逆序后得到 hash 的**前 10 个十六进制字符**。
>
> 脚本两种都打印：**原始字节 hex** + 解出的 hash 前缀。若原始字节全 0，会明说"取不到"，
> 并提示**不要**据此改动 §2.5 —— 免得把解码失败当成"版本不符"。

### ④ 的判定规则

未解锁时电流应 ≈0。**恒为 0 或恒为怪值 = 中招**（`nxt-dual` 的 ADC 通道缺陷）。
中招后电流/电量保护全是空转，且**参数无法绕过**（编译期宏），只能重编重刷固件
（修复 = fork 自带的 `pr_fix_hkust_nxt_dual` @ `8161c4cd`，相对 `main` 只差一行）。
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
| Tier B：**对真飞控** | ⚠️ **跑过，但还没成功** —— 2026-09-21 在 z0 上实跑，飞控未听到（原因见下条） |
| Tier B：**「不是数据流」判据** | ✅ **2026-09-21 在 z0 上第一手触发**：`ttyTHS1` 接到飞控线后读到 ~13 kB/s、帧错误 0，但非零段最长仅 **9 B** → 正确判为**噪声/串扰**，**没有**误报成「波特率不对」。这正是加这条判据的理由 |
| `uart_loopback.py`：对照组（`--no-loopback`） | ✅ **已在 z0 的 Jetson 上实跑**：未短接时两口均收不到，符合预期 |
| `uart_loopback.py`：**内部回环短模式** | ✅ **已在 z0 的 Jetson 上实跑**：两口 7 字节均原样收回 ⟹ 控制器/驱动/时钟正常（⚠️ **它绕过引脚，证明不了 pinmux/接线**） |
| `uart_loopback.py`：**长模式（测 DMA 缺陷）** | ✅ **已在 z0 上第一手触发并复现**：两口发 16 字节均收回 16 个 0x00 ⟹ 接收通路 DMA 缺陷坐实 |
| `uart_loopback.py`：真回环 | ✅ **2026-09-21 在 z0 上实跑**：短接「串口1」（= `ttyTHS1`）后，**收回字节数精确等于发出字节数** ⟹ 引脚/pinmux/电平全通；而内容在 ≥8 字节被打成 0 ⟹ 坐实是驱动缺陷而非接线 |

所以：**首次对真飞控跑时，请留意脚本有没有报错或给不出读数**，那是预期内的首跑风险，
不是"已验证过所以一定对"。
