# 12: 复核遗留 —— 05（与 02）产物上的 5 处缺陷

Status: ready-for-agent
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

- [ ] **#1** `tools/fcu-params/params_from_serial.py:120` —— 用**未去重**的 `got` 当完成条件
- [ ] **#2** `tools/hover-thrust/thrust_curve_check.py:107` —— `affine_r2` 的 `ValueError` 一路冒泡成 traceback
- [ ] **#3** `docs/control/first-flight-config.md:269` —— 行号引用 `preflight_check.py:240` 已失效（实为 `:252`）
- [ ] **#4** `docs/control/first-flight-config.md:202` —— K2 行把本票**新增**的 `-0.01` 写成「既有的」
- [ ] **#5** `tools/fcu-params/test_params_from_serial.py:23` —— 注释标「2026-09-29」，但那份基准 2026-10-10 已被替换

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

- **没有**改 [05](05-first-flight-config-freeze.md) 的票面去加"未决事项"回链 ——
  `triage-labels.md` 要求 `done` 票点名未决项，但那要动**已收口票**，留给操作者定。
- **没有**碰 07 的任何产物（07 自己的第 6 条已当场改完）。
- **没有**动上面 5 个文件里的任何一个字节。

---

**为什么是 `ready-for-agent`**：5 条全是纯软件（注释/文档/工具健壮性），台上可跑、不需通电。
`Status` 由 07 收口时**提议**，操作者可随时改判 —— 「归谁」是操作者保留的裁定。
