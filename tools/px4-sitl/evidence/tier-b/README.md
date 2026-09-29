# B 档五场矩阵的归档证据（ticket 11）

这一份是 [`docs/control/tier-b-sitl-seam3.md`](../../../../docs/control/tier-b-sitl-seam3.md)
（ticket 11 的交付文档）背后的一手产物。**裁决表与全部读数在文档里，这里只放产物本身**，
以免同一个数字在两处各写一遍、日后改一处忘一处。

## 文件

| 文件 | 是什么 |
|---|---|
| `<tag>-summary.json` | 该场的观测汇总：配置、真值统计、DUT 逐帧统计、`solve` 耗时分账、**`checks` = 判定行**（含每行的 `detail`，全部读数在文字里） |
| `<tag>-console.log` | PX4 SITL 的控制台（`pxh>`），137 行 |
| `<tag>-replay.json` | 回放夹具自己的收尾统计（发帧数、实测延迟、σ） |
| `mocap-model-3.json` | **夹具的模型**（134 KiB）：来自真实 Phase 1 包，含逐轴 σ 与成簇帧间隔 ⟹ 有了它才能重放出"同样脾气"的动捕流 |
| `run_matrix.sh` | 跑这五场的脚本（含**等空机**闸门与 `pgrep -x` 的教训） |

`<tag>` = `F0`(标称) / `F1`(mass+30，判据①) / `F2`(mass+30 + 50 ms，判据③) /
`F3`(mass+30 + 注入求解失败，判据④) / `F4`(标称 + 注入，判据④的对照)。

## ⚠️ 四件读这份归档必须知道的事

1. **逐帧数组（`frames` / `truth`，原始 48 MB）没有入库。** 因此 `checks` 是**固化下来的**判定结果 ——
   拿 `-summary.json` 单独重跑 `evaluate_b` **会少掉若干依赖逐帧数组的行**（那是判层的优雅降级，
   不是那些行不通过）。
2. **`checks` 已由当前判层重判过一次**（`checks_note` 字段里也写了）：飞行当时写下的那一份判层
   有过两个 bug —— ⑧ 误读了 A 档的分龄列（B 档只有一个状态源，只有 `age_state_s`）、
   判据① 缺「正常帧总长够不够 8 s」的前置三判定。**别用旧判层的结论复核这些数字。**
   ⚠️ 重判时 **⓪b 那一行走的是"复用落盘读数"分支**（`_tap` 不随 JSON 落盘）⟹
   它的中位数是**飞行当时**由同一份代码算出来的。这是判层 docstring 里点明的那个陷阱
   （改了口径、拿旧 JSON 重判，这一行不会跟着变）—— 要它按新口径重算，得整场重飞。
3. **`params` 只记了 ROS 参数，没记 harness 的 argv**（`--seed` 等）⟹ 复现靠 `run_matrix.sh`
   的默认值（种子默认 `20260929`）。
4. **代码身份**：每份 `<tag>-summary.json` 里的 `controller_code` 块记的是这五场实际 import 到的
   控制器源码 —— `src/nmpc` @ `cff55ae`（2026-09-29 09:40:49），逐文件 sha256 也在里面。
   为什么**包**解析到 `src/` 而不是 `install/`：`run_matrix.sh:52` 设了
   `PYTHONPATH=/home/chen/ros_ws/src/nmpc`，而 DUT 子进程继承它 —— `ta.MpcDut.start` 把
   `env = dict(os.environ)` 再叠 `dut_env()` 交给 `Popen`，而 `dut_env()`
   （[`verify_tier_a_hover.py:370`](../../verify_tier_a_hover.py#L370)、
   [`:339`](../../verify_tier_a_hover.py#L339)）**只加 acados 两个键、不动 `PYTHONPATH`**。
   ☠️ **但入口脚本（`__main__`）来自 `install/`，这与"包来自 `src`"是两件事**：
   DUT 是 `ros2 run nmpc mpc_node.py` 起的 ⟹ `ros2 run` 按 `<prefix>/lib/<包>/<名字>` 找文件
   ⟹ 跑的是 `install/nmpc/lib/nmpc/mpc_node.py`（2026-09-29 当场查
   `get_package_prefix('nmpc')` = `/home/chen/ros_ws/install/nmpc`）。
   ⚠️ 这份与 `src/` 那份**逐行 diff 只差 1 行**（`:130` 的 docstring 里 `T_max` vs `f_max`）
   ⟹ 行为一致、**本批读数不受影响**；但**"改了 `src/` 就等于改了飞的东西"是错的** ——
   `mpc_node.py` 里的改动必须 `colcon build` 之后才会飞（20 号票的落地就卡在这条上）。
   ⚠️ **`install/nmpc` 是 09-29 08:59 的构建快照、与这份不是同一份代码**
   （ticket 12 的 ±30% 扫描跑的就是那份）。
   ★ 但**这一处差异对本批的数字是数值无感的**（2026-09-29 经 12 号会话提出后逐行核实）：
   飞行路径上**唯一**的行为差异是 `ocp.py` 的 `ubu[0]` 由 `t_max` 换成 `f_max`
   （外加 `node_core` 里那个顶约束自检跟着换对象；`sign_map.py`/`mpc_node.py` 纯注释）。
   在**本批用了的 SITL 口径**下两者几乎相等：`hover_thrust = 0.6235` ⟹ `t_max = 11.8003 N`
   vs `f_max = 11.8 N`，差 **0.0027 %**（远低于 `SAT_TOL_REL = 1e-3`）。
   ⟹ **代码身份仍以 `src@cff55ae` 为准（上表已记），但它不构成"两份读数不可比"的理由** ——
   真正的可比性限制是**实验条件不同**（场次、失配档、hold 时长），不是这一行差异。
   （补一句踩过的坑：**默认口径 `0.1589` 下两者差 3.92×**（`t_max = 46.3 N`）——
   那正是 `f_max` 解耦要修的东西；本批因为显式传了 `0.6235` 才躲开。）
   ★ 这一块是事后补的（ticket 12 的会话指出「13 行照样印得整整齐齐、但可能不是同一个实验」，
   而这批归档原先确实没记代码身份）。补的只是元数据：五份 JSON 除新增 `controller_code` 外
   **逐键全等**、判定行分布仍是 10/1、9/2、10/2、10/2、11/1。

## 怎么重新跑出来

```bash
TAGS="F0 F1 F2 F3 F4" bash tools/px4-sitl/evidence/tier-b/run_matrix.sh
```

产出落在 `/tmp/tier-b/`。**必须先空机**：这套数字里有一条是"实测最坏求解耗时"，
机上多几个进程就能把它从 5.7 ms 抬到 11 ms（A 档 §5.1 量过）；脚本的等空机闸门就是干这个的。

☠️ **`/tmp` 是 tmpfs —— 重启即失，而且已经真丢过一次。** 机器 2026-09-29 10:40 重启后，
五场的原始逐帧（`/tmp/tier-b/*.jsonl`，48 MB）与当时 dump 出来的 `/tmp/mocap-model-3.json`
**都没了**；本归档里剩下的就是全部。⟹ **回放夹具的模型输入现在以本目录里那份
`mocap-model-3.json` 为准**（内容与当时同一份：`F*-replay.json` 的 `sigma_m` 与
`base_delay_ms` 是夹具直接读模型算出来落盘的，逐位对得上），验证器与 `run_matrix.sh`
的 `--model` 默认值已指向它。**判据① 那类"要按新口径重算就得整场重飞"的行，从此没有回头路。**
