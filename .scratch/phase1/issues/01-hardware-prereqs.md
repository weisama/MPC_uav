# 01: 前置人工核实 —— 动捕网络 / 飞控串口 / 固件版本 / 电流读数

Status: ready-for-human
Type: task

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

- [ ] Jetson 在动捕网段内，`ping 10.1.1.198` **0% 丢包**（不再走校园网关）
- [ ] 记录飞控实际使用的串口设备节点，并确认它可用
- [ ] QGC 读到的飞控版本号**原文**记入 [`refs-manifest.md`](../../../docs/reference/refs-manifest.md) §2.5
- [ ] 未解锁时的电池电流读数记录在案，并给出"是否中招"的判定，结论记入 [`params-changes.md`](../../../docs/reference/params-changes.md) A4
- [ ] 四条结论**全部落进文档** —— 不能只留在对话或记忆里

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
