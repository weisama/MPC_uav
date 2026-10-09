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
#    ★ 落在**仓库里**、别落 /tmp —— /tmp 过不了重启（这条项目里吃过亏）
python3 tools/fcu-params/params_from_serial.py --yaml tools/fcu-params/evidence/fcu-dump-<日期>.yaml
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
| `changed` | 两边都有、值不同 | ⚠️ 头号来源是**整型没按 `param_type` 解**（见坑 1）；其次才是 float32 的表示误差 |
| `only_current` | 飞控里有、快照里没有 | 上次导出后**新加**的参数（版本不同）。★ 伪参数 `_HASH_CHECK` 已剔除，不会再冒充一条 |
| `only_reference` | 快照里有、飞控里没有 | ⚠️ 多半是**这次没读全**（链路上丢了），不是被删 —— 收不全时工具会报出**缺哪几个 `param_index`** |

★ **容差**（`params_core.REL_TOL = 1e-6` / `ABS_TOL = 1e-4`）：QGC 导出写的是 float32 的十进制展开
（`0.45` ⟹ `0.449999988079071`、`0.2` ⟹ `0.200000002980232239`），MAVLink 读回的是**同一个** float32。
用严格相等会把一整批参数判成"变了"，而那是**表示误差，不是改动**。
双向都钉住了：表示误差**不许**报（`test_容差_float32的两种写法是同一个值`）、
真改动**必须**报（最小一档是 `0.5 → 0.1589`、`0.45 → 0.451`）。

★ **"读全了没有"按 `param_index` 判，不按名字个数判**（`collect_params`）：
`param_count` 是飞控自报的 **used 参数个数**（`param_count_used()`），每条真参数的
`param_index` 是 `param_get_used_index(param)`（`send_param` 里那行，第一手），
取值恰好是 `[0, param_count_used())` 且不重不漏 ⟹ 缺的索引能指名道姓。
重传（同一条来两遍）**另记一栏**，不会再冒充"没收全"。

## 踩过的坑

### 1. ☠️☠️ 忽略 `param_type` ⟹ 一整批整型参数看着像"被改了"

**2026-10-09 第一次真机读，现场踩到。** 读数长这样：

```
NAV_RCL_ACT     = 1.401298464324817e-45     快照 1
MC_BAT_SCALE_EN = 0.0                       快照 0        ← 这条是对的（浮点）
MPC_USE_HTE     = 1.401298464324817e-45     快照 1
MPC_XY_VEL_MAX  = 2.0                       快照 2.00000… ← 这条也是对的
对账：相同 720，不同 189
```

一眼看像"飞控上有一两百条参数被人改过"。**其实一条都没变。**

机理是**第一手**的（`refs/px4-v1.14.3-build`，`MavlinkParametersManager::send_param`，
源码那句注释自己就写着 "MAVLink encodes float and int params in the same space during transmission"）：

```cpp
if (param_type(param) == PARAM_TYPE_INT32) {
        int32_t param_value;
        param_get(param, &param_value);
        memcpy(&msg.param_value, &param_value, sizeof(param_value));   // ← 位型塞进 float 字段
} else {
        msg.param_value = param_value;                                  // ← 数值直传
}
```

⟹ **整型参数在线上是"位型"**：`1` ⟹ 0x00000001 ⟹ 读成 `1.4e-45`；`-1` ⟹ 0xFFFFFFFF ⟹ 读成 `nan`；
`921600` ⟹ 0x000E1000 ⟹ 读成 `1.3e-39`。修法是按 `msg.param_type` 还原
（`params_core.decode_param_value`）。

**离线核账**：把那次读数按位型重解释一遍 —— **189 条"不同"里 184 条当场对上**，
剩下 5 条才是真的（见下）。这是"差异数量级本身就该可疑"的一个实例：
**一个改动不该同时动到 `PWM_MAIN_MIN1` 和 `NAV_RCL_ACT`。**

★ 这个坑**最毒的地方**是它伪装成工具唯一要回答的那个问题："飞控里的值变了没有"。
它不会报错、不会崩，只会给你一屏**看起来很像真的**差异。

### 2. ★ 伪参数 `_HASH_CHECK` 会冒充"飞控多出一条参数"

PX4 回全量的**第一条永远是它**（`send_one()`：`if (_send_all_index == PARAM_HASH) { param_index = -1;
param_type = MAV_PARAM_TYPE_UINT32; memcpy(&msg.param_value, &hash, 4); }`）——
它是**参数哈希**，不是参数，`param_index = -1`，QGC 导出里也没有。
不剔除它，对账的"飞控里有、快照里没有"那一栏就会凭空多一条。

### 3. ★★ 自检里那个"已知值核对"，判据比模块自己的容差还严

`--self-test` 曾经是**红的**：`HTE_THR_RANGE: 期望 0.2，解出 0.20000000298023224`。
原因是那五条已知值用了 `abs(got - want) > 1e-9` —— **比 `same_value()` 还严**，
于是把自己刚设计好的容差**绕过去了**。

它的恶性在于**它红得完全没有道理**：读数是**对的**，红的只是判据。
而"判据本身写错"和"链路上真读到旧值"在现场**长得一模一样**。

★ 更要紧的是**它当时没被任何测试覆盖**：`test_params_core.py` 9 条**全绿**、`--self-test` 红。
⟹ 补了 [`test_params_from_serial.py`](test_params_from_serial.py)，钉的不是"函数返回什么"，
而是"**这条命令本身是绿的**"。这是本工具第一次"测试全绿但东西是坏的"——
**第二次是坑 1**（那次 19 条单测里没有一条碰过真串口的解码路径）。

### 4. ★ 变异 4 一开始**没被逮住**（断言没鉴别力）

把 `Diff.clean` 改成只看 `changed`（丢掉 `only_current` / `only_reference`），
跑出来 **12 passed** —— 因为那条用例的 diff 里**同时有** `changed`，
`not self.changed` 照样是 `False`，被改的那一支根本没走到。

修法是**一次只多一样**：只有 `only_current` 非空、只有 `only_reference` 非空，各钉一条。
（同族教训：`test_容差_*` 也是**双向**的，只钉一边就会漏掉另一边的错。）

### 5. ★ 别把导出文件的编码当 UTF-8

`docs/assets/z0.params` 的 header 是 **UTF-8 + LF**、body 是 **CRLF**、
body 里还夹着**一行原始 GBK 字节**。`decode_export` **逐行**解码、每行独立退回，
所以整块按任何一种编码解都炸、而数据行（纯 ASCII）一分不损。
`test_解析_中文头_CRLF_GBK字节都拦不住它` 里带了一条**负控**：先证明整块解在这份输入上**真的会炸**。

## 验证状态（诚实交代）

| 部分 | 状态 |
|---|---|
| 离线单测 `pytest tools/fcu-params/` | ✅ **19 条全过** |
| `--self-test` | ✅ 绿（`skipped` **0 行**、自己对自己干净、变异**恰好**报 1 条、位型解码 7 条；条数见它自己第一行的现算读数） |
| **变异测试**（主动弄坏，看会不会红） | ✅ 8 条变体里 **7 条被逮住**：已知值判据退回严格相等 / 容差退化成 `==` / 参数名正则放行小写 / `clean` 只看 `changed` / **忽略 `param_type`**（一次红 5 条）/ 伪参数照收 / 完整性按名字个数判 —— 各有用例变红（`clean` 那条是**补强断言后**才咬住的，见坑 4）。⚠️ **第 8 条（无符号档改用 `<i` 解）被操作者拦下、没跑** —— 如实记着，别当成"验过" |
| 解析器在**真文件**上 | ✅ `docs/assets/z0.params`（⚠️ 条数**现算**别抄，跑 `--self-test` 看第一行） |
| **在真飞控上跑通** | ✅ **2026-10-09 跑通了**（HEARTBEAT、`/dev/ttyTHS1:921600`、910 条读数都回来了）—— 但那一次抓出了坑 1 的 bug |
| **修复后重读、并与快照对上** | ⚠️ **未做** —— 见下 |

⚠️ **最后一格才是"这三框闭合没有"的答案**。现在手上这次读数是**修复前**的
（`evidence/fcu-dump-2026-10-09.raw.yaml`，整型是位型，只作留档），所以：

- 那次读数**不能**用来判"飞控此刻的值是不是 `docs/assets/z0.params` 里那些"——
  189 条"不同"里 184 条已离线证明是伪影，但**离线证明是推断，不是第一手**；
- 还没闭合的三框（ticket 05 的 ① `NAV_RCL_ACT`、② `MC_BAT_SCALE_EN`、④ §D 那批）
  **要等修复后重读一次**才能划掉。

**重读后才该说的两句话**（现在只能说到这里）：
① 那 184 条伪影消失、对账变干净（或只剩能逐条解释的真差异）；
② `NAV_RCL_ACT` / `MPC_USE_HTE` / `MPC_THR_HOVER` 这几条读回的是**数值**（`1` / `1` / `0.5`），
不是位型。

★ 修复前的读数里还有 **5 条真差异**，重读时要看它们还在不在：
`CAL_ACC0_XOFF/YOFF/ZOFF`（**三条都变了**，`ZOFF` 从 `0.108` 掉到 `0.0216` ⟹ 像是**重新标定过**，
但这是**推断**，得问操作者）、`COM_FLIGHT_UUID`（每次开机自增的计数器）、
`LND_FLIGHT_T_LO`（飞行计时器）。★ 后两条**本来就该变**，不是异常。

（离线侧已核的链路事实：`pymavlink 2.4.49` + `pyserial 3.5` 机上已有；
`/dev/ttyTHS1` 属 `root:dialout`、当前用户**在 `dialout`** 组里；检查时**没人占着该口**。
★ 这几条是**快照**：真的撞上"口被占"，症状是 `wait_heartbeat` 超时而不是权限错。）

## 依赖

`pymavlink`（真读那一侧，**延迟 import** —— `--self-test` 与全部单测**不需要它**，
也就不需要串口权限）。离线侧只用标准库。

- [`params_core.py`](params_core.py) —— 纯函数：解析 + 对账。**不 import** `pymavlink`、**不读文件**。
- [`params_from_serial.py`](params_from_serial.py) —— CLI。**唯一**碰硬件的两个动作：等 `HEARTBEAT`、发 `PARAM_REQUEST_LIST`。
