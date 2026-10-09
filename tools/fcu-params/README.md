# `fcu-params` —— 从串口**只读**读回飞控参数，与快照逐条对账

服务于 ticket [`../../.scratch/phase4/issues/05-first-flight-config-freeze.md`](../../.scratch/phase4/issues/05-first-flight-config-freeze.md)
的 **§5 ⑤**（"起飞前 dump 一次飞控参数与 `docs/assets/z0.params` 对账"）与
[`docs/control/first-flight-config.md`](../../docs/control/first-flight-config.md) §5 的 ②（D 节）、⑤（全量对账）。

**它回答的是**：`docs/assets/z0.params`（QGC 导出，2026-09-29）**只是快照** ——
"这份文件对"≠"**此刻**飞控里就是这些值"。这个工具把后半句变成**一次可核的动作**。

**它不做的事**（写清楚了才好用）：**一个字节都不写**。没有 `PARAM_SET` 的代码路径。
要写参数走 [`docs/reference/params-changes.md`](../../docs/reference/params-changes.md) §I.1 的四步纪律
（先存旧值 → 写 → 读回 → 再 dump 一次），那是**人的动作**。

## 快速开始

```bash
# 0) 自检 —— 不碰串口、不需要飞控通电、不需要 mavros
python3 tools/fcu-params/params_from_serial.py --self-test

# 1) 离线单测（同上，走 pytest）
python3 -m pytest tools/fcu-params/ -q

# 2) 真读：读回 + 与快照逐条对账
python3 tools/fcu-params/params_from_serial.py --compare docs/assets/z0.params

# 3) 只想看那几条要核的
python3 tools/fcu-params/params_from_serial.py \
  --name NAV_RCL_ACT,MC_BAT_SCALE_EN,MPC_THR_HOVER,MPC_USE_HTE,MPC_THR_MIN,MPC_XY_VEL_MAX

# 4) 留一份全量快照（对账的输入，也是"此刻"的第一手证据）
python3 tools/fcu-params/params_from_serial.py --yaml /tmp/fcu_params_2026-10-09.yaml
```

退出码：`0` = 对上 / `1` = 有差异 / `2` = 链路或文件层失败（**不是** traceback 完事，
链路错误会打印成一句可执行的排查建议）。

## 链路（为什么是这几条参数）

| 事实 | 值 | 第一手出处 |
|---|---|---|
| 走哪口 | `MAV_1_CONFIG = 102`（TELEM2） | `docs/PROJECT.md:375-376` |
| 什么模式 | `MAV_1_MODE = 2`（Onboard） | 同上 |
| 波特率 | `SER_TEL2_BAUD = 921600` | 同上 |
| 设备 | `/dev/ttyTHS1` | `src/bringup/launch/mavros.launch.py` 的 `fcu_url` 默认值 |
| 协议 | **MAVLink**（`UXRCE_DDS_CFG = 0` ⟹ 不走 uXRCE-DDS） | `docs/reference/params-analysis.md` §3.8 |

★★ **所以这个工具用 pymavlink 直连，不需要 mavros 在跑，也不需要 ROS。**
（文档里另有一条 `ros2 param dump /mavros/param` 的路，`docs/PROJECT.md:406-407`；
那条要 mavros 起链、且 ROS 2 的 mavros **没有** `/mavros/param/get` 服务 —— 拿不到"全量逐条"。）

⚠️ `--port` / `--baud` 可以覆盖，但**默认值就是 z0 那条**；改默认值前先看这张表。

## 对账的三类结果（"干净"要三条都空）

| 栏 | 含义 | 常见的**假**警报 |
|---|---|---|
| `changed` | 两边都有、值不同 | 无 —— 容差按 float32 往返定（见下） |
| `only_current` | 飞控里有、快照里没有 | 上次导出后**新加**的参数（版本不同） |
| `only_reference` | 快照里有、飞控里没有 | ⚠️ 多半是**这次没读全**，不是被删 —— 收不全时工具会**当面说出来** |

★ **容差**（`params_core.REL_TOL = 1e-6` / `ABS_TOL = 1e-4`）：QGC 导出写的是 float32 的十进制展开
（`0.45` ⟹ `0.449999988079071`、`0.2` ⟹ `0.200000002980232239`），MAVLink 读回的是**同一个** float32。
用严格相等会把一整批参数判成"变了"，而那是**表示误差，不是改动**。
双向都钉住了：表示误差**不许**报（`test_容差_float32的两种写法是同一个值`）、
真改动**必须**报（最小一档是 `0.5 → 0.1589`、`0.45 → 0.451`）。

## 踩过的坑

### 1. ★★ 自检里那个"已知值核对"，判据比模块自己的容差还严

`--self-test` 曾经是**红的**：`HTE_THR_RANGE: 期望 0.2，解出 0.20000000298023224`。
原因是那五条已知值用了 `abs(got - want) > 1e-9` —— **比 `same_value()` 还严**，
于是把自己刚设计好的容差**绕过去了**。

它的恶性在于**它红得完全没有道理**：读数是**对的**，红的只是判据。
而"判据本身写错"和"链路上真读到旧值"在现场**长得一模一样**。

★ 更要紧的是**它当时没被任何测试覆盖**：`test_params_core.py` 9 条**全绿**、`--self-test` 红。
⟹ 补了 [`test_params_from_serial.py`](test_params_from_serial.py)，钉的不是"函数返回什么"，
而是"**这条命令本身是绿的**"。这是本工具唯一一次"测试全绿但东西是坏的"。

### 2. ★ 变异 4 一开始**没被逮住**（断言没鉴别力）

把 `Diff.clean` 改成只看 `changed`（丢掉 `only_current` / `only_reference`），
跑出来 **12 passed** —— 因为那条用例的 diff 里**同时有** `changed`，
`not self.changed` 照样是 `False`，被改的那一支根本没走到。

修法是**一次只多一样**：只有 `only_current` 非空、只有 `only_reference` 非空，各钉一条。
（同族教训：`test_容差_*` 也是**双向**的，只钉一边就会漏掉另一边的错。）

### 3. ★ 别把导出文件的编码当 UTF-8

`docs/assets/z0.params` 的 header 是 **UTF-8 + LF**、body 是 **CRLF**、
body 里还夹着**一行原始 GBK 字节**。`decode_export` **逐行**解码、每行独立退回，
所以整块按任何一种编码解都炸、而数据行（纯 ASCII）一分不损。
`test_解析_中文头_CRLF_GBK字节都拦不住它` 里带了一条**负控**：先证明整块解在这份输入上**真的会炸**。

## 验证状态（诚实交代）

| 部分 | 状态 |
|---|---|
| 离线单测 `pytest tools/fcu-params/` | ✅ **12 条全过** |
| `--self-test` | ✅ 绿（`skipped` **0 行**、自己对自己干净、变异**恰好**报 1 条；条数见它自己第一行的现算读数） |
| **变异测试**（主动弄坏，看会不会红） | ✅ 4 条变体逐个验过：① 已知值判据退回严格相等 ② 容差退化成 `==` ③ 参数名正则放行小写 ④ `clean` 只看 `changed` —— **各自都有用例变红**（④ 是**补强断言后**才咬住的，见坑 2） |
| 解析器在**真文件**上 | ✅ `docs/assets/z0.params`（⚠️ 条数**现算**别抄，跑 `--self-test` 看第一行） |
| **在真飞控上跑通** | ⚠️ **未跑** —— 见下 |

⚠️ **未跑的那一格是本工具最要紧的一格**。上面全是**离线**证据：
它证明"解析器和判据是对的"，**证明不了**"这条链路能通、能读回全量"。
第一次真读会同时验证：`/dev/ttyTHS1:921600` 通不通、HEARTBEAT 到不到、
`param_count` 收不收得全、以及**飞控此刻的值是不是快照里那些**。
在那之前，本工具对 ticket 05 的那三框**一条都没闭合**。

（离线侧已核的链路事实：`pymavlink 2.4.49` + `pyserial 3.5` 机上已有；
`/dev/ttyTHS1` 属 `root:dialout`、当前用户**在 `dialout`** 组里；本次检查时**没人占着该口**。
★ 这三条都是**快照**：真的撞上"口被占"，症状是 `wait_heartbeat` 超时而不是权限错。）

## 依赖

`pymavlink`（真读那一侧，**延迟 import** —— `--self-test` 与全部单测**不需要它**，
也就不需要串口权限）。离线侧只用标准库。

- [`params_core.py`](params_core.py) —— 纯函数：解析 + 对账。**不 import** `pymavlink`、**不读文件**。
- [`params_from_serial.py`](params_from_serial.py) —— CLI。**唯一**碰硬件的两个动作：等 `HEARTBEAT`、发 `PARAM_REQUEST_LIST`。
