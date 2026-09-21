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
python3 -m pip install --user pymavlink

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

## 为什么用 pymavlink 而不是手搓解码器

MAVLink v2 的 CRC 带 per-message `CRC_EXTRA` 种子。种子写错**不会报错、只会静默丢帧** ——
手搓等于赌一张背下来的种子表。pymavlink 是权威实现，不赌。

（本仓库的 `~/refs/` 里都是 fetch-only 副本，没有 mavlink 生成头，
所以本地也无法第一手核对种子表 —— 这正是不能用记忆里的表的理由。）

## 验证状态（诚实交代）

| 部分 | 状态 |
|---|---|
| Tier A：网络 / 串口枚举 / 权限 | ✅ **已在 z0 的 Jetson（`orin-nx`）上实跑验证**，含 `dialout` 权限被拒的真实报错路径 |
| Tier B：`--port` 读 ③④ | ✅ 已用 **pymavlink 在 pty 上模拟的假飞控**端到端验证（含 git hash 解码、电流为 0 的「疑似中招」判定） |
| Tier B：`--scan` 找端口 | ✅ 同上验证（假飞控 + 真实权限报错两种路径都走到） |
| Tier B：**对真飞控** | ❌ **尚未** —— 等飞控上电后首跑 |

所以：**首次对真飞控跑时，请留意脚本有没有报错或给不出读数**，那是预期内的首跑风险，
不是"已验证过所以一定对"。
