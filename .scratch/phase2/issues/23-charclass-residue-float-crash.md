# 23: 字符类残留 `-?[0-9.eE+-]+` —— 半截窗口上的 `float('-')` 崩点（逐处点名）

Status: done
Type: task

**What's wrong:** 几个取数器吃的都是**有损窗口**，而它们的数值正则用的是
`-?[0-9.eE+-]+` —— 这个字符类**本身**能只匹配到**一个** `-`（或 `e`/`E`/`.`/`+`），
于是 `float('-')` 抛 `ValueError`。**崩点在"一场飞行跑完之后"**：不是值不合法，是
**窗口边界可以落在数值中间**。

**来源：** [ticket 12](12-mismatch-sweep.md) 的点名项 ① ——
[`mismatch-sweep.md` §附.8](../../../docs/control/mismatch-sweep.md) 末列了"同类字符类残留处"。
操作者 **2026-09-29 裁定"开票"**（原话见 [12 号票](12-mismatch-sweep.md) Comments 7）。
本项目另有一条纪律：**"可达" ≠ "已发生"** —— 下面逐处区分。

⚠️ **编号取 23**：`issues/` 里下一个空闲号（写入时最新为 `22-ocp-health-import-leak-judge-wrong-scope.md`）。
**若并行会话已另开同题材的票，以先入库者为准、后到者并入，别留两张。**

---

## 1. 机理（一句话）

`px4_console` 把控制台当**有损窗口**渲染两种口径：

- `_raw[mark:]`（**字节**窗口，`text_since(mark)`）—— 字节边界可以落在数值中间；
- `px.text()[marker:]`（**文本偏移**窗口）—— 用具 `text()` / `wait_for()` 的 docstring
  **自己就把"文本偏移当游标"判为无效**（渲染不是增量的，切片可能从 token 中间开始）。

两种窗口的**尾部**都可能是一个**半截数值**。旧字符类 `-?[0-9.eE+-]+` 要求"≥0 个字符"，
所以半截成 `-` 时它匹配到 `-` 并 `float('-')` 抛。**修法**：数值改成
`-?(?:\d+\.?\d*|\.\d+)(?:[eE][+-]?\d+)?`（**至少要有一个数字**），取不到一律 `None`、**绝不抛**。

> ⚠️ 这个修法**没关掉**的是它旁边那条：**静默截短**（`-0.01408` 被窗口切成 `-0.0140`，
> 仍然匹配新正则、仍给**合法但错**的值）。本条只把"抛"变成 `None`，**没把"截短"变成正确**。
> 由 ticket 12 记在 [`mismatch-sweep.md` §附.8](../../../docs/control/mismatch-sweep.md)。

## 2. 点名处（逐处）

⚠️ **按「文件 + 符号名」找，别按行号找** —— 行号会漂。下表左边"票/文档引"列是 ticket 12 文档
§附.8 / 票面当时写的号码，右边"现"列是**本条修完之后**的实际位置。

| 票/文档引 | 现（本条写入时） | 符号 | 窗口 | 崩点可达？ |
|---|---|---|---|---|
| `verify_offboard_smoke.py:422` | `:434`（`def field` 在 `:432`） | `field()` | 字节窗口 | **已修**（ticket 12 那轮，本条不重复动） |
| `verify_offboard_smoke.py:443` | `:443` | `ts_age_s()` | 字节窗口 | ❌ **不可达**（见 §4） |
| `verify_offboard_smoke.py:862/863` | `:879/:880`（`def _read_param` 在 `:849`） | `World._read_param()` | 字节窗口 | ✅ **可达** → **本条已修** |
| `verify_offboard_smoke.py:892` | `:909`（`def airframe_param` 在 `:907`） | `airframe_param()` | **静态文件**（非窗口） | ❌ **不可达**（见 §4） |
| `verify_z0_plant.py:78` | `:91` | `PARAM_RE`（喂 `param_value()`） | 文本偏移窗口 | ✅ **可达** → **本条已修** |
| `verify_z0_plant.py:92` | `:110`（`def field` 在 `:103`） | `verify_z0_plant.field()` | 文本偏移窗口 | ✅ **可达** → **本条已修** |

★ 行号为什么漂：`verify_offboard_smoke.py` 的 `:862/:863/:892` 是 **`74e938c` 之前**的号码
（ticket 12 修 `field()` 时在文件上方插了 `_NUM` 注释块 ⟹ 之后的行整体后移 +13；本条又往
`_read_param` 里插了 4 行注释 ⟹ 再后移一点）；`verify_z0_plant.py:78/:92` 在本条插入 `_NUM`
注释块后现为 `:91/:110`。**引用请连符号一起引。**

## 3. ★★ 可达 ≠ 已发生

**没有证据**说上面任一处**真跑里**被咬过 —— ticket 12 的原话（本条采纳）：
"**"可达"不是"已发生"** —— 没有证据说真跑里被咬过（212 的原话，我同意；**别把它写成"曾出错"**）"。

唯一**有第一手**的一次崩是 **`verify_offboard_smoke.field()`**（ticket 12 加密档第一场，
2026-09-29，`float('-')`、整场作废）—— 那一处**已经在 ticket 12 那轮修掉**，不在本条。
本条点名的其余处**只有"可达"这个强度**，**没有实跑证据**。

## 4. 逐处核实（本会话第一手，原始输出见 Comments）

- `verify_offboard_smoke.ts_age_s()`（`:443`）：**不可达**。它的取值组**后面必须**跟
  ` seconds ago)`，尾部一截断整条正则就不匹配 ⟹ 返回 `None`。**保留原样。**
- `verify_offboard_smoke.airframe_param()`（现 `:909`，旧号 `:892`）：**不可达**。它读的是
  入库的**静态机型文件全文**（`AIRFRAME.read_text()`），**不是**控制台窗口 ⟹ 不存在"窗口边界落在
  数值中间"这条通路。**保留原样。**
- `verify_offboard_smoke.World._read_param()`（现 `:879/:880`，旧号 `:862/:863`）：**可达**（同一机制）。
  读取前有 `send(...)` + `drain(0.5)`，尾部**通常**落在行边界（所以风险低于 `field()`），
  但 `drain` 只把概率压低、**不消除机制** ⟹ **已修**（改用模块级 `_NUM`）。
- `verify_z0_plant.param_value()` / `field()`（现 `:91`/`:110`，旧号 `:78`/`:92`）：**可达**，
  且踩的是**被用具明文弃用**的文本偏移游标（`px.text()[marker:]`）⟹ **已修**。

## 5. 验收项

- [x] `verify_z0_plant.py` 的 `param_value()` / `field()` 在半截窗口上给 `None`、不抛
- [x] `verify_offboard_smoke.World._read_param()` 同上
- [x] `ts_age_s()` / `airframe_param()` **经核实不可达 ⟹ 保留原样**（理由写进 §4）
- [x] 加单测，且**主动弄坏一次**确认它会红（证明有鉴别力）
- [x] 合法数值的取法**一个都没变**

## Comments

### 2026-09-29 —— 操作者裁定开票 + 本会话落地（含第一手验证与原始输出）

> ★ **票面状态说明**：本会话**已按本票在工作树落地**（未 `git add`/`commit`，改动见 ② 与 ④⑤）。
> 票面 `Status:` 按 2026-09-29 裁定落 **`ready-for-agent`**（**未收 `done`**）—— 上面 §5 的验收项
> 因此**不勾**，是否转 `done` 由复核 / 提交后再定。
>
> ☠️ **【2026-09-29 已作废 —— 见本文件末「⑦ 收口」】**：上面这段「`ready-for-agent`」「未收 `done`」
> **已过期**。本票已由复核验证并 `Status: done`、§5 五项全勾。原句保留为**当时记录**
> ——⚠️ 但别再照它以为"还没做"（状态词比数更毒）。

**① 开票依据**：操作者 **2026-09-29 裁定"开票"**（落地为 ticket 12 点名项 ① 的处置 —— 见
[12 号票](12-mismatch-sweep.md) Comments 7 与票头 2026-09-29 收口说明）。本票即那一张。

**② 改了哪几处（3 个函数 / 4 个正则）**

| 文件 | 符号 | 改法 |
|---|---|---|
| `tools/px4-sitl/verify_z0_plant.py` | `PARAM_RE`（喂 `param_value()`） | `(-?[0-9.eE+-]+)` → `({_NUM})` |
| `tools/px4-sitl/verify_z0_plant.py` | `field()` | 同上 |
| `tools/px4-sitl/verify_offboard_smoke.py` | `World._read_param()` 的两条候选正则 | 同上（改用模块级 `_NUM`） |

`_NUM = r"-?(?:\d+\.?\d*|\.\d+)(?:[eE][+-]?\d+)?"`（**至少一个数字**）——
与 ticket 12 修 `verify_offboard_smoke.field()` 时用的是**同一个写法**。

**③ 哪几处经核实其实不会崩 ⟹ 保留原样（不改齐）**

- `verify_offboard_smoke.ts_age_s()`（`:443`）：**不可达**。取值组后**必须**跟 ` seconds ago)`，
  尾部一截断整条正则就不匹配 ⟹ 返回 `None`。**保留原样。**
- `verify_offboard_smoke.airframe_param()`（现 `:909`）：**不可达**。它读的是**静态机型文件全文**
  （`AIRFRAME.read_text()`），**不是**控制台窗口 ⟹ 没有"窗口边界落在数值中间"这条通路。**保留原样。**

（这两处的字符类**字面上仍写着**旧写法 `-?[0-9.eE+-]+` —— 这是**刻意保留**：它们触发不了，
按"可达 ≠ 已发生、别为了改齐而改"处理。）

**④ 原始输出（第一手，本会话；不启 SITL、不飞）**

修前（旧的 `-?[0-9.eE+-]+`）：

```
param_value(" x * SIH_MASS [843,1330] : -", "SIH_MASS")
  -> ValueError: could not convert string to float: '-'
field("    z: -", "z")   -> ValueError: could not convert string to float: '-'
field("    z: e", "z")   -> ValueError: could not convert string to float: 'e'
param_value(完整行) -> 0.75   field(完整行 "    z: -0.01408") -> -0.01408
World._read_param 内层正则:  "x   COM_OF_LOSS_T [281,483] : -"  -> float('-') 抛
```

修后：

```
param_value(" x * SIH_MASS [843,1330] : -", "SIH_MASS") -> None
param_value(" x * SIH_MASS [843,1330] : e", "SIH_MASS") -> None
param_value(" x * SIH_MASS [843,1330] : 0.7500", "SIH_MASS") -> 0.75
field("    z: -", "z") -> None ; field("    z: e", "z") -> None
field("    z: -0.01408", "z") -> -0.01408 ; field("    z: -1.2e-3", "z") -> -0.0012
field("    z: .5", "z") -> 0.5 ; field("    z: 5.", "z") -> 5.0
World._read_param("x   COM_OF_LOSS_T [281,483] : -") -> None
World._read_param("x   COM_OF_LOSS_T [281,483] : 1.0000") -> 1.0
World._read_param("x   MPC_Z_VEL_P_ACC [1,2] : -2.5000") -> -2.5
# 未改的两处仍照旧：
ts_age_s("    timestamp: 1234 (-") -> None ; airframe_param("SIH_MASS") -> 0.75
```

（`World._read_param` 用一个人造 console 桩调用：`mark` / `send` / `drain` / `text_since`
返回值 = 上面那个半截窗口。纯离线，不起 PX4。）

**⑤ 单测（新增文件，并按"新断言要主动弄坏一次"自证）**

- 新增 [`tools/px4-sitl/test_charclass_residue.py`](../../../tools/px4-sitl/test_charclass_residue.py)：
  **17 条**，纯离线（`verify_z0_plant` 不依赖 ROS；`World._read_param` 那几条用
  `pytest.importorskip("rclpy"/"px4_msgs")`，没环境就 skip）。
  两条断言：**半截值给 `None` 绝不抛** + **合法数值一个都没变**（后者是"修法只影响原本会抛的输入"
  那条论断的机器可读版）。★ 断的是**解析往返**（`float("0.75") == 0.75` 这类），
  没有用 `== 0.0` 式过紧断言。
- **弄坏一次（变异测试）**：把两个 `_NUM` 临时改回旧字符类 `-?[0-9.eE+-]+`，跑本文件
  ⟹ **7 failed, 10 passed** —— 红的**恰好**是 7 条"半截"用例（param_value ×3、field ×3、
  read_param ×1），10 条"正常数值"全绿 ⟹ 有鉴别力、且不是整体假红。改回后 **17 passed**。
- **回归**：跑相关离线文件 `test_mismatch_sweep.py` + `test_verify_judges.py` +
  `test_tier_a_judges.py` + `test_tier_b_judges.py` + `test_px4_console.py` = **159 passed**。
  （按机器纪律**未跑** `pytest tools/px4-sitl/` 全量 —— 那些用例起 SITL、与并行 stream 抢 CPU；
  也未动 `src/nmpc/**`、未 `colcon build`。）

**⑥ 与 ticket 12 的边界**：本票把 §附.8 里"其余是已知潜伏"那几处**核实并处置**；
`field()` 那一处（真正崩过的那次）**属 ticket 12 那轮**，本票不重复动、也不改它的测试。

**⑦ 复核轮（2026-09-29，独立验证 ⟹ 收口 `done`）**

复核没有采信 ⑤ 的报告，**重做了一遍**（第一手）：

- **变异测试我自己跑了一遍**，结论与 ⑤ 一致：把两个 `_NUM` 同时改回 `-?[0-9.eE+-]+`
  ⟹ `6 failed, 8 passed, 3 skipped`（本 shell 无 ROS，`_read_param` 那 3 条被 `importorskip` 跳掉；
  有 ROS 就是 ⑤ 记的 **7** 条）。红的**恰好**是"半截值"用例（`param_value` ×3、`field` ×3），
  "合法数值"8 条全绿 ⟹ **有鉴别力，且不是整体假红**。复原后 `14 passed, 3 skipped`，
  `git diff --stat` 与 ② 一致（`verify_z0_plant.py` +24/−3、`verify_offboard_smoke.py` +8/−2）。
- **回归我复算过**：`test_mismatch_sweep` + `test_verify_judges` + `test_tier_a_judges` +
  `test_tier_b_judges` + `test_px4_console` + 本票新文件 = **176 passed**（= ⑤ 的 159 + 新文件的 17）。
  ⚠️ **环境要求记一笔**（⑤ 没写，踩过）：光 `source install/setup.bash` **不够** ——
  这些文件 import `px4_msgs`，必须**同时** source `/opt/ros/humble/setup.bash` 与
  `/home/chen/px4_msgs_ws/install/setup.bash`，否则整模块 `importorskip` 成
  **`1 skipped`**（看着像"全过"，其实是**一条都没跑** —— 这类假绿要认得出来）。
- **两个"不可达"判定，复核独立回代码核过，均成立**：`ts_age_s()` 的取值组**后面必须**跟
  ` seconds ago)`（尾部一截断整条正则不匹配 ⟹ `None`），结构上到不了崩点；
  `airframe_param()` 读 `AIRFRAME.read_text()` **全文**，不存在"窗口边界落在数值中间"这条通路。
  ⟹ ⑥ 的处理**维持原样，不撤回**（`_read_param()` 是 `text_since(mark)` 字节窗口，同一机制，改对了）。
- 状态词订正：票头 ② 上方那段「`ready-for-agent`／未收 `done`」**已加 ☠️ 作废标记**，
  §5 五项**已勾**，`Status: done`。★ 本仓库教训：**状态词比数更毒** —— 活儿落地了票面还写
  "待做"，下一个人会照着重做一遍。
