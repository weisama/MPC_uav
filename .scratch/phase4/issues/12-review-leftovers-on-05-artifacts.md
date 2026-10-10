# 12: 复核遗留 —— 05（与 02）产物上的 5 处缺陷

Status: done
Type: task

**What to build:** 把 2026-10-10 给 [07](07-seam1-rerun-real-params.md) 做 `/code-review` 时**顺手扫出来、
但不属于 07** 的 5 条缺陷落地修掉。它们全在**已经收口的票**（[05](05-first-flight-config-freeze.md)）
或**别人正在做的票**（[02](02-hover-thrust-backfill.md)）的产物里，07 只**登记**、不越界去改
（仓库纪律：「别顺手改别的票」）。

**为什么值得单开一张**：[`triage-labels.md`](../../../docs/agents/triage-labels.md) 对 `done` 的定义
明写「**未决事项必须另开 ticket 并在本文里点名，不能靠 `done` 掩盖**」。这 5 条是 07 复审时**新发现**的，
05 收口时并不知道 ⟹ 归到这张新票里，而不是回头改 05 的结论。

**来源**：2026-10-10 的 `/code-review`（针对 [07](07-seam1-rerun-real-params.md) 的改动）报出 6 条，
其中第 6 条（`test_mpc_node.py` 里打印串 `τ=1e-6` 与实跑 `run(0.0)` 不符）**属于 07**、已当场改掉；
剩下 5 条**逐条回代码核过**，记在下面。★ 每条都带**核实方式**，不是转述审查意见
（仓库纪律：「复核意见也当假设验」）。

- [x] **#1** `tools/fcu-params/params_from_serial.py:120` —— 用**未去重**的 `got` 当完成条件
- [x] **#2** `tools/hover-thrust/thrust_curve_check.py:107` —— `affine_r2` 的 `ValueError` 一路冒泡成 traceback
- [x] **#3** `docs/control/first-flight-config.md:269` —— 行号引用 `preflight_check.py:240` 已失效（实为 `:252`）
- [x] **#4** `docs/control/first-flight-config.md:202` —— K2 行把本票**新增**的 `-0.01` 写成「既有的」
- [x] **#5** `tools/fcu-params/test_params_from_serial.py:23` —— 注释标「2026-09-29」，但那份基准 2026-10-10 已被替换

★ 上面各条的**行号是发现时（2026-10-10 施工前）的定位**，不是改完之后的 —— 尤其 #1：
那条 `break` 条件现在是 `params_from_serial.py:127` 的 `indices_complete(seen_idx, count)`
（原 `:120` 的 `if count and got >= count`）。
★ 追溯请按**文件 + 函数/改动**，别按这里的行号 —— 本票 #3 修的正是这一类悬空引用。

---

## 逐条

### #1 `params_from_serial.py:120`：`got` 不去重，却拿它当 break 条件

**现象**（`awk 'NR>=113&&NR<=121'` 逐字）：

```python
            count = int(msg.param_count)
            idx = int(msg.param_index)
            raw.append((pid, msg.param_value, int(msg.param_type), idx, count))
            if idx >= 0:
                got += 1                      # ← 同一 idx 再来一遍也 +1
            ...
            if count and got >= count:
                break                         # ← 于是可能"数够了但其实没收齐"
```

**核实方式**：`grep -n "duplicate\|missing\|indices"` → 同模块的 `collect_params` **明确区分**
`c.duplicates`（第 127 行）与 `c.missing_indices`（第 135 行），注释还写明"重传**必须说出来**"。
⟹ 模块自己的口径是「重传是预期事件、要跟缺项分开」，而第 120 行用的是**没有这个区分**的 `got`。

**后果**：拉全量时某条 `PARAM_VALUE` 被重传一次、同时另一条真参数在链路里晚到 ⟹ `got` 凑到
`param_count` 就 break，此刻 `indices` 里还缺一条 ⟹ 打印「⚠️ 没收全 … 缺的 param_index 是 […]」，
操作者会把这条**假警报**读成"飞控删了这个参数"。

**修法方向**（施工时确认，别照抄）：完成条件改成按**去重后的索引集合**判满
（`len({i for ... if i >= 0}) >= count`），`got` 只留着做进度打印。

### #2 `thrust_curve_check.py:107`：`affine_r2` 的 `ValueError` 没人接

**现象**：`affine_r2` 在第 106-107 行（`sxx == 0.0`）与第 113-114 行（`sst == 0.0`）抛
`ValueError("xs 是常数 ⟹ 斜率不定（这一段没有激励）")` / `ValueError("ys 是常数 ⟹ r² 无定义")`。

**核实方式**：`grep -n "def mean_r2\|def best_fac\|def analyse_ulog\|try:\|except ValueError"` →
`mean_r2`（第 127 行）直接 `vals.append(affine_r2(xs, out)[2])`、`best_fac`（第 135 行）直接
`r = mean_r2(pairs, fac)`，**两处都没有 try**；文件里唯一的 `try` 在第 265 行，是 `_load_series`
给"这份 ulog 里没有这个话题"用的，与 r² 无关。

**后果**：拿一份真飞行日志跑，窗口里恰好有一路电机 `output`（或 `control`）是常量
（未参与控制 / 卡在某个档） ⟹ `best_fac` 扫 201 个 `fac`，第一次进 `mean_r2` 就在那一路抛，
直接冒泡成 traceback。而本脚本其它失败路径（缺话题第 268 行、缺字段第 274 行）**都是干净的
`SystemExit` 消息** ⟹ 唯独这里例外，用户拿到的是栈。

**修法方向**（施工时确认）：`analyse_ulog` 里给 `best_fac`/`mean_r2` 的调用加一层，
把 `ValueError` 转成带通道号的 `SystemExit`（"第 j 路在这一段没有激励"），
或在 `mean_r2` 内跳过无激励通道并**报出跳了哪几路**。★ 别静默 `try/except: pass` ——
跳过会把 r² 的样本数悄悄改掉。

### #3 `first-flight-config.md:269`：`preflight_check.py:240` 是悬空行号

**现象**：第 269 行表格左列写 `` `src/nmpc/nmpc/preflight_check.py:240` ``。

**核实方式**：`grep -n "MPC_XY_VEL_MAX" src/nmpc/nmpc/preflight_check.py` → **252**。
不是文档写错、是**同一张票把注释块加长后行号漂了**（旧代码常量在 240）。

**同族**：第 104 行也引了 `` `preflight_check.py:236-240` ``，同一处理一并核。

**后果**：下一个人按 `:240` 去翻，落到的是注释正文而不是常量定义，会误以为引用错了或被删了。
仓库对这类悬空行号有专门纪律。

### #4 `first-flight-config.md:202`：把**新增**说成「既有的」

**现象**：第 202 行 K2 那格（"判据会变红的那条断言"栏）写
「参数化负控新增 `tau_s = 0.2` 与**既有的** `-0.01`」。

**核实方式**：`git log -L 84,90:src/nmpc/test/test_preflight_check.py` →
`4a4d6be Phase 4/05` 那个 hunk（`@@ -84,2 +84,7 @@`）里 **`-0.01` 与 `0.2` 两行都是 `+`**。
04-05 之前那一档**只有** `("tau_s", 0.5, …)` 一行（`aaeb356 ticket 06 收口` 引入）。

**后果**：复核者据「`-0.01` 既有」去 `origin/main` 找那条负控，**根本不存在** ⟹
把这次改动的**测试增量**误记成存量（真正既有的只有 `0.5` 那一档），弱化了 K2 的实际覆盖面。

### #5 `test_params_from_serial.py:23`：基准文件的日期标错

**现象**（第 23-24 行）：

```python
#: 仓库里那份真导出（2026-09-29）。自检的输入。
Z0_EXPORT = Path(__file__).resolve().parents[2] / "docs/assets/z0.params"
```

**核实方式**：`git log --oneline -2 -- docs/assets/z0.params` →
`9e3bb2a Phase 4/05 换档收口：操作者用 QGC 重导参数文件`（2026-10-10）。⟹ 路径没变、
**指向的文件变了**，注释里的 09-29 已过期。

**后果**：让人以为打负控的是旧那版；实际取值来自 10-10 导出的那份，溯源会被带偏。
（仓储里已有同族纪律：「散文里写死的数会过期」——日期/快照时点要带**可核的锚**。）

---

## 归属与边界

| # | 落点 | 归属 | 本票要不要动 |
|---|---|---|---|
| 1 | `tools/fcu-params/params_from_serial.py` | 05 的产物（`9e3bb2a`） | 要 |
| 2 | `tools/hover-thrust/thrust_curve_check.py` | **02 的产物（`c225a34 Phase 4/02`）** | ⚠️ 见下 |
| 3 | `docs/control/first-flight-config.md:269` | 05 的产物 | 要 |
| 4 | `docs/control/first-flight-config.md:202` | 05 的产物 | 要 |
| 5 | `tools/fcu-params/test_params_from_serial.py` | 05 的换档后果（`9e3bb2a`） | 要 |

⚠️ **#2 的归属有歧义，留给操作者裁定**：它所在的文件是 [02](02-hover-thrust-backfill.md) 的交付物，
而 **02 现在是 `Status: ready-for-human`（未收口）** ⟹ 它可能该记进 02 的 Comments，
而不是这张「收口票的遗留」票。07 收口时问过操作者、选了"新开一张"这条路；
这里**只登记、不代替操作者决定**（仓库纪律：「别顺手改别的票」）。
**施工前先看 02 是否还开着** —— 若还开着，先问一句再动手。

## 本票**没做**的

> ☠️ **2026-10-10 作废本节第 3 条**（下面逐项点名）—— 本节写于**登记时**，本票当日已施工：
> · 第 1 条（不碰 05 那张 `done` 票）—— **仍成立**；
> · 第 2 条（不碰 07 的产物）—— **仍成立**；
> · 第 3 条「**没有**动上面 5 个文件里的任何一个字节」—— ☠️ **不再成立**：5 条缺陷已在本次全部落地，
>   改了 7 个文件。以文末 Comments 为准。**留着原句是为了不抹掉登记时的状态**。

- **没有**改 [05](05-first-flight-config-freeze.md) 的票面去加"未决事项"回链 ——
  `triage-labels.md` 要求 `done` 票点名未决项，但那要动**已收口票**，留给操作者定。
- **没有**碰 07 的任何产物（07 自己的第 6 条已当场改完）。
- ~~**没有**动上面 5 个文件里的任何一个字节。~~ ☠️ 见上（2026-10-10 施工后已不成立）。

---

> ☠️ **2026-10-10 作废（下面这段是 07 登记时的状态，本票当日已施工并收口）**：
> 现在的 `Status` 是 `done`，收口理由见文末 Comments。

**为什么是 `ready-for-agent`**（登记时）：5 条全是纯软件（注释/文档/工具健壮性），台上可跑、不需通电。
`Status` 由 07 收口时**提议**，操作者可随时改判 —— 「归谁」是操作者保留的裁定。

---

## Comments

### 2026-10-10 —— 收口：`Status` → `done`（**操作者裁定**，本票 5 框全闭）

**裁定方式**：施工完成后交操作者裁定「翻成 done」。★ 这一格是**操作者**拍的，
不是"我做完了"自动推出来的 —— 施工与证据见下（两义已在票面分开写明）。

**收口时的票面状态**：顶层 5 个框**全 `[x]`**（`grep '^- \[ \]'` 空）；
没有第二份验收清单（本票只有顶层这一份，不存在副本分叉）。
**未决事项**（`triage-labels.md` 要求 `done` 票不得用收口掩盖）已逐条写在
「本票**没做**的」与「复核轮」两节里，不另开票：末尾那条 `Collected.complete @ expected==0`
是**既有行为、只影响非法自报值的报告侧**，已就地登记在代码注解里。

### 2026-10-10 —— 5 条全部落地（`/implement`，本机 Orin NX）

**改了 7 个文件**：5 处缺陷里 #3/#4 同一份文档，另有 2 个测试文件 + `params_core.py`（#1 的纯函数落点）。

| # | 落点 | 改法 |
|---|---|---|
| 1 | `tools/fcu-params/params_from_serial.py` ＋ `params_core.py` | 完成条件换成新纯函数 `indices_complete(seen_idx, count)`（按**去重后索引**判满，与 `Collected.complete` 同一把尺）；`got` 只留作进度打印 |
| 2 | `tools/hover-thrust/thrust_curve_check.py` | `analyse_ulog` 里在**拟合之前**加一道与既有一道对称的闸门：`control` 通道严格恒定 ⟹ 报**带通道号**的 `SystemExit`（见下节） |
| 3 | `docs/control/first-flight-config.md:269`（**含同族 `:104`**） | 悬空行号重指：`:240` ⟹ `:252`、`:236-240` ⟹ `:236-252`（行号是 05 把那段注释加长后漂的，值本身没动） |
| 4 | `docs/control/first-flight-config.md:202` | 「既有的 `-0.01`」⟹ 写明**两条都是本票新增**（`4a4d6be` 的 hunk 里两行都是 `+`；那一档此前**只有 `0.5`**） |
| 5 | `tools/fcu-params/test_params_from_serial.py:23` | 不再写死日期，改指该文件**自己的说明头**（路径一直没变、文件换过两档 ⟹ 写死日期正是上一版出错的机制） |

### #2 的票面描述有一处**与实测不符** —— 已按实测实现

票面原文：「窗口里恰好有一路电机 `output`（**或** `control`）是常量 ⟹ … 直接冒泡成 traceback」。
回代码核出**前半句不成立**：`analyse_ulog:325-327` 那道「四路输出都在动」的闸门**先**就把它拦了
（`max-min > 1.0` 判活），报的是干净的 `SystemExit`。用合成数据跑真调用实测两种形状：

| 形状 | 实测 |
|---|---|
| `control` 与 `output` **都**恒定（真日志里"某电机没参与"的自然形态） | 已被 `:325-327` 拦成 `SystemExit("✗ 只有 3 路输出在动…")` |
| `control` 恒定、`output` 抖 >1 count（要合成才造得出） | ✅ 票面这条成立：`ValueError: xs 是常数…` 从 `:107` 冒泡成栈 |

⟹ 修法按第二种来，且**必须放在拟合之前** —— 接住异常不行：常量 `control` 会让每个输出的 4 个候选
**各抛一遍**，报出来的"哪一路"取决于先扫到谁。新闸门判据取**严格恒定**（`max == min`），
比"会抛那个异常"**略宽**：`sxx` 是按均值算的，全等的一组数在浮点下 `sum/n` 未必逐位回到原值
（实测 `[0.1]*3` ⟹ `sxx ≈ 5.8e-34 ≠ 0.0`，此时不抛）。两种情形结论相同，一并拦掉。

★ **#2 的归属**：开工前 02 还是 `ready-for-human`（未收口），票面要求先问一句 ——
**2026-10-10 操作者裁定「本票就改」**，02 的票面保持不动。

### 变异测试（纪律：新断言要主动弄坏一次）

| 把生产代码改回原样 | 该变红的断言 | 实测 |
|---|---|---|
| `if indices_complete(seen_idx, count)` ⟹ `if count and got >= count` | `test_params_from_serial.py::test_pull_params_不被重传骗到而提前收手` | ✅ 红 |
| 删掉 `still = [… max == min …]` 那道闸门 | `test_thrust_curve_check.py::test_一路control没激励时_报干净的SystemExit而不是栈` | ✅ 红（冒出来的正是 `:107` 那条 `ValueError`） |
| **删掉整句 `break`** | 同上（靠 `recv_match_calls == 4` 那一半） | ✅ 红（那次跑了 5.4 s，正是空转到超时的指纹） |
| 尺子改成数个数：`not _missing(...)` ⟹ `len(seen) >= expected` | `test_params_core.py::test_读满判据_数个数会漏掉_个数够但覆盖不齐` | ✅ 红 |

★ #1 的测试**分两层**：`test_params_core.py` 钉那把**尺子**，`test_params_from_serial.py`
钉**它被用上了**（脚本化假链路，不开串口、不要串口权限）—— 只钉尺子的那层，挡不住"尺子对而没接上"这类回归。
★ 后两条是**复核轮补的**（第 ④ 条意见说前一版分不出"读完就 break"与"根本没有 break"）—— 见下。

### 全量测试（`/implement` 的最后一关）

| 面 | 命令 | 读数 |
|---|---|---|
| `tools/` 六个**收得到**的子目录（点名） | `pytest tools/battery-thrust tools/fcu-params tools/hover-thrust tools/ladder-baseline tools/sign-map tools/tau-analysis -q` | **161 passed** |
| `src/nmpc/test` | `source install/setup.bash` + `ACADOS_SOURCE_DIR=$HOME/acados` 后跑 `pytest test -q` | **221 passed**（8m42s） |
| `run/tests/` 逐文件 | `pytest run/tests/test_<名>.py -q` | 5 ＋ 80 ＋ 66 passed、1 skipped（`test_z0_check_live`） |

⚠️ **两条环境坑，与本次改动无关**，但下次跑全量会撞上（**2026-10-10 实测**）：

1. **`pytest tools`（目录形式）一条都收不到** —— 不是"全绿"：ROS 的 `launch_testing` 插件在从 `tools/`
   递归时把收集吞了（同一棵树下**逐子目录点名**收到 161 条）。`-p no:launch_testing` 也不行 ——
   它会打断 `launch_testing_ros_pytest_entrypoint`。
2. `src/nmpc/test` 不 source 工作区时 `ModuleNotFoundError: No module named 'px4_msgs'`
   （要 `source install/setup.bash`；`ACADOS_SOURCE_DIR` 另算，见仓库纪律）。

### 复核轮（`/code-review`，`high`）—— 报 6 条：**采纳 4 条、驳回 2 条**

报告收 6 条，逐条回代码核实（纪律：复核意见也当假设验；★ 我自己关于代码的断言同理）：

| 意见 | 核实 | 处置 |
|---|---|---|
| ① `indices_complete` 与 `collect_params` 各写一遍同一条规则 ⟹ docstring 说的"同尺"没有构造上的保证 | 属实（两处都是 `set(range(expected)) - seen`） | ✅ 抽出 `params_core._missing()`，**两处都调它**（同 `K3` 的先例：一处定义才是真的同一把尺） |
| ② `expected` 为负时 `indices_complete` 返回 `True`（`range(-1)` 是空集 ⟹ 差分空） | 属实：实测 `indices_complete(set(), -1) is True` | ✅ 加 `expected < 0 ⟹ False` |
| ③ docstring 声称与 `Collected.complete` 同尺，但 `expected == 0` 时两边结论**相反** | 属实：实测那边 `True`、这边 `False` | ✅ **改文档不改代码** —— 循环这侧判 `True` 会在**第一条**消息上就 `break`，所以分歧是**要**的；已在 docstring 里点名，不再说"同尺" |
| ④ 新测试分不出"读完就 `break`"与"根本没有 `break`" | 属实：删掉 `break` 后循环只是空转到超时，而收齐的参数照样齐 ⟹ 旧断言全绿 | ✅ 假连接加 `recv_match_calls` 计数并断言 `== 4`；已用"删掉 `break`"变异验红 |
| ⑤ `pytest tools`（目录形式）收不到用例 | 属实，但**与本次改动无关**（ROS `launch_testing` 插件） | ⛔ 不属本票，只在上面"全量测试"节登记读数 |
| ⑥ `indices_complete` 每次重建 `set(range(expected))` ⟹ 全读 O(params²) | 属实（z0 ≈911 条 ⟹ 全程约 83 万次集合插入），**但**常数极小、N 有界：串口那一趟是秒级，多出来的是毫秒级 | ⛔ **不改** —— 为它加缓存会把"一处定义"这条更值钱的纪律弄脏；登记在此，不装作没看见 |

★ 复核同时确认了几件正面的事（不必再验）：两处新行号 `preflight_check.py:252` / `:236-252` **指得准**、
`z0.params:649` 仍是 `2.0`、`-0.01` 与 `0.2` 确实**都是** `4a4d6be` 新加的。

### 本票**没做**的

- **没有**动 [02](02-hover-thrust-backfill.md) 的票面（#2 改在**本票**里；要不要往 02 加一条回链归操作者）。
- **没有**动 [05](05-first-flight-config-freeze.md) 那张 `done` 票去补"未决事项"回链（票面自己说留给操作者）。
- **没有**碰 07 的任何产物。
- **没有**把上面那两条环境坑写进任何 README —— 那是别处的账，只在这里登记读数。
- **没有**把 `Collected.complete` 在 `expected == 0` 上的 `True` 对齐过来（复核 ③ 顺带发现）——
  它只落在"飞控自报 0 条"这种非法自报值的**报告**那一侧，且是既有行为；改它会动 `params_core`
  的公开属性，超出本票 5 条的边界。**已在该分歧处（`indices_complete` 的 docstring）点名**。
