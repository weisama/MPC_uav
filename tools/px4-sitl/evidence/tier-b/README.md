# B 档矩阵的归档证据（ticket 11 —— 目录内**九场**：F0–F4 五场 + ticket 20 的两次复跑四场）

这一份是 [`docs/control/tier-b-sitl-seam3.md`](../../../../docs/control/tier-b-sitl-seam3.md)
（ticket 11 的交付文档）背后的一手产物。**裁决表与全部读数在文档里，这里只放产物本身**，
以免同一个数字在两处各写一遍、日后改一处忘一处。

## 文件

| 文件 | 是什么 |
|---|---|
| `<tag>-summary.json` | 该场的观测汇总：配置、真值统计、DUT 逐帧统计、`solve` 耗时分账、**`checks` = 判定行**（含每行的 `detail`，全部读数在文字里） |
| `<tag>-console.log` | PX4 SITL 的控制台（`pxh>`），137–138 行（`mass+30` 那几场多一行质量提示） |
| `<tag>-replay.json` | 回放夹具自己的收尾统计（发帧数、实测延迟、σ） |
| `mocap-model-3.json` | **夹具的模型**（134 KiB）：来自真实 Phase 1 包，含逐轴 σ 与成簇帧间隔 ⟹ 有了它才能重放出"同样脾气"的动捕流 |
| `mocap-model-4.json` / `mocap-model-5.json` | **八字那档（`--reference-source traj`）的模型**。包 4 = 参考包络 1.000 × 0.500 m（**0.5 m 的旧规格**，2026-10-08 前录的）；包 5 = 2.000 × 1.000 m（**1.0 m，当前规格**）—— 票 05 起 `run_matrix.sh` 的 `T0)` 接的是**包 5**。⚠️ 回放节点**不自己量包**：参考 / 噪声 / 时序全部来自这份 JSON ⟹ 拿错哪一份，装备都不报错、只是跑的参考不对 |
| `run_matrix.sh` | 跑这九场的脚本（含**等空机**闸门与 `pgrep -x` 的教训） |
| `u-ref-authority-*.json` | **ticket 20 的离线探针产物**（不是飞行）：`probe_u_ref_authority.py` 的输出，见下节。★ 其中 `-sameframes-20260930` 那份是**同批帧、新树**的复跑，用来量"中间那四个提交对这档的影响"（20 号票 §13.7） |
| `controller-code-20260930-F0c-F1c.json` | `F0c`/`F1c` 两场的**代码身份**（`git HEAD` + 11 个文件 sha256 + 入口脚本 sha）—— 这两场的 `--json` 里没有 `controller_code` 块（缺席口径见下方第 4 条），故单记一份 |
| `rejudge-20261008-judge2.json` | **判据② 换成真判之后**，拿归档 obs **离线重判** `F0c`/`F1c` 的全文（不启 SITL、不启夹具，域 73）。☠️ 落盘的 `-summary.json` 里那条判据② 是**旧判层的硬编码 `False`** ⟹ 本文件是它的**唯一**订正处（机制与计数见下方第 2 条） |
| `df-column-20260930.json` | `F0b`/`F1b`/`F0c`/`F1c` 四场的**逐帧 `[tick, df, u[0], thrust_body_z]`** + 发布点闭合式残差 —— 20 号票 §13.4 的**唯一**可复核底料（`df`/`u_ref0` 两列不在 `-summary.json` 里，逐帧 jsonl 又没入库）。**生成器**：[`tools/px4-sitl/export_df_column.py`](../../export_df_column.py)（读 `/tmp/tier-b/<tag>-dut.jsonl` ⟹ 必须与飞行同一次开机内跑）。☠️ **这两场的输入已经没了**：2026-09-30 06:38 那次开机把 `/tmp` 清了 ⟹ 生成器**再跑不起来**，本文件是那四列（`df`／`u[0]`／`thrust_body_z`／`u_ref0`）**唯一**的存本（这正是"当场落仓库"那条纪律的一次现场兑现） |

`<tag>` = `F0`(标称) / `F1`(mass+30，判据①) / `F2`(mass+30 + 50 ms，判据③) /
`F3`(mass+30 + 注入求解失败，判据④) / `F4`(标称 + 注入，判据④的对照)。
★ 另有四场是 **ticket 20 的复跑**（同一套脚本 `run_matrix.sh`），**两组是两个不同的落点**：
`F0b`/`F1b` = 偏置加在 **`u_ref`** 上（**已被证伪**，见 20 号票 §10）；
`F0c`/`F1c` = 偏置加在**发出去的指令**上（裁定 4 的候选 A，本张的验收证据）—— 见下节。
☠️ **两组都要留着**：作废的是**落点**，不是那两场读数 —— 它们正是"反号"那件事的一手凭证。

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
   ☠️ **还有一处作废在落盘的 `checks` 里（2026-10-08 起的第三次口径变化）：判据② 那一行。**
   `F0c`/`F1c` 两份 `-summary.json` 里都能读到一条
   `★判据② 跑完 Phase 1 的三条轨迹（悬停 / 1 m 正方形 / r=0.5 m 8 字）` = **`false`** ——
   那是**旧判层的硬编码 `False`**，**不是**这两场的实测结论，**也不代表这两场没通过判据②**。
   现行判层把它拆成**两行**（轨迹1 悬停 / 轨迹2 八字），一场只判其中一条、另一条记
   **本场不做**（`ok=null`、**不计数**）。口径与实测读数见
   [`docs/control/tier-b-sitl-seam3.md`](../../../../docs/control/tier-b-sitl-seam3.md) **§6**；
   这两场的**离线重判全文**见本目录 `rejudge-20261008-judge2.json`（不启 SITL，域 73）。
   ★ 连带的计数订正：旧判层下这两场记 **11 / 1**，现行判层下是 **11 / 0**。
   ☠️ 重判时 **`att_err_deg` 不许 pop** —— 它是本场**落盘的读数**（不是从 `frames` 现算的），
   pop 掉会让 ⓪b 退化成「判不了」而**少一条计入**（我踩过：读出来是 `10 / 0`，看着像"口径变了"）。
3. **`params` 只记了 ROS 参数，没记 harness 的 argv**（`--seed` 等）⟹ 复现靠 `run_matrix.sh`
   的默认值（种子默认 `20260929`）。
4. **代码身份**：每份 `<tag>-summary.json` 里的 `controller_code` 块记的是那五场（`F0`–`F4`）实际 import 到的
   控制器源码 —— `src/nmpc` @ `cff55ae`（2026-09-29 09:40:49），逐文件 sha256 也在里面。
   ⚠️ **ticket 20 那四场（`F0b`/`F1b`/`F0c`/`F1c`）的 JSON 里没有这一块**（那块是事后给 F0–F4
   补的，`F*` 的生成路径不产它；**缺席口径** = `grep -c controller_code <tag>-summary.json`
   在这四份上都出 `0`）：`F0b`/`F1b` 的身份记在 20 号票 §9.6／§11（`a4e0fa8c…`），
   `F0c`/`F1c` 的记在本目录 `controller-code-20260930-F0c-F1c.json`。
   为什么**包**解析到 `src/` 而不是 `install/`：`run_matrix.sh` 里那行 `export PYTHONPATH=…/src/nmpc` 设了
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
# ticket 20 的第一次复跑（偏置落在 u_ref 上 —— 这个落点后来被证伪）：
TAGS="F0b F1b" bash tools/px4-sitl/evidence/tier-b/run_matrix.sh
# ticket 20 的第二次复跑（偏置落在发出去的指令上 = 裁定 4 的候选 A）：
TAGS="F0c F1c" bash tools/px4-sitl/evidence/tier-b/run_matrix.sh
# 定性用的离线探针（几秒，不占机器窗口 ⟹ **不用等空机**）：
python3 tools/px4-sitl/probe_u_ref_authority.py --frames /tmp/tier-b/F1b-dut.jsonl
```

### ticket 20 第一次复跑的两场（2026-09-29 13:0x）—— **落点后来被证伪**

同一脚本、同一判层、同一口径，唯一差别是**控制器带 `ki_z = 1.0 N/(m·s)` 的垂直积分作用**。
结论（全程读数见 [20 号票](../../../../.scratch/phase2/issues/20-vertical-channel-mismatch-offset.md) §10）：

| 场 | 被控对象 | 垂直误差均值 | 判据① | `ΔF` 稳态 |
|---|---|---|---|---|
| `F0b` | `mass±0` | −0.0083 m | ✅ | 0.582 N |
| `F1b` | `mass+30` | **−0.7200 m** | ❌ | **顶夹 4.4425 N** |

⟹ 积分作用**没有**消掉失配偏差，反而让两场都同向差了 ~0.01 / ~0.08 m。
**根因已离线量到**：`d u[0]/d u_ref[0] = −0.0616 N/N`（反号 ⟹ 正反馈），
而环路真正撑住高度的是 `d u[0]/d e_z = +3.45 N/m` 那条通路。
⚠️ 前四场（F0–F4）的 `controller_code` 是 `src@cff55ae`；**这两场是 `a4e0fa8c…`**（本张的新代码）——
两组**不可混在一张表里比**，只能按"同一对（有无积分器）"比。

### ticket 20 第二次复跑的两场（2026-09-30 04:4x）—— **判据① 双过**

同一脚本、同一判层、同一 `ki_z = 1.0`，**唯一差别是偏置落到了发出去的指令上**（裁定 4 的候选 A）。
两场**判据① 双过**（`mass+30` 那场从 `F1b` 的 −0.7200 m 收进 0.15 m 界内）。
☠️ **本节的数字一律不在 README 里重抄**（归档的规矩：同一个读数只写一处、改一处）——
逐场读数在 [20 号票](../../../../.scratch/phase2/issues/20-vertical-channel-mismatch-offset.md) **§13.2**、
代价在 **§13.5**、发布点闭合式（含本目录 `df-column-20260930.json` 该怎么用）在 **§13.4**。
这里只记三条**归档层面**、看这个目录的人会踩的：

1. **`ΔF` 现在只能从 `df` 列读** —— `u_ref0[0]` 在候选 A 上**恒为常量**（两场去重后只有一个值）⟹
   照旧落点那样去读 `u_ref0[0]` 会读出"`ΔF = 0`"，把一次成功的实验误判成"积分作用没生效"（§13.4）。
2. **代价落在振荡峰峰值那一列**（标称那一场也涨了，而它的 `ΔF` 只有 mN 量级）⟹ 逐项见 §13.5。
3. **离线复算的峰峰值那一列不许用** —— 被实飞证伪（§13.6）。

⚠️ 代码身份：`F0c`/`F1c` 走的是 `src@5c27f0e`（`src/nmpc` 干净 ⟹ 控制器 == 该提交内容），
入口脚本与 `src/` 那份 **同 sha `463df55e…`**（已 build）；逐文件 sha 见本目录
`controller-code-20260930-F0c-F1c.json`。



产出落在 `/tmp/tier-b/`。**必须先空机**：这套数字里有一条是"实测最坏求解耗时"，
机上多几个进程就能把它从 5.7 ms 抬到 11 ms（A 档 §5.1 量过）；脚本的等空机闸门就是干这个的。

☠️ **`/tmp` 过不了重启，而且已经真丢过一次。**（2026-09-29 订正：**机制不是 tmpfs** ——
`findmnt -T /tmp` 显示它就是根盘 `ext4` 上的普通目录，没有独立挂载；真原因是
`/usr/lib/tmpfiles.d/tmp.conf:11` 的 `D /tmp 1777 root root -`，即 **systemd-tmpfiles 每次开机清空它**。
⟹ **同一次开机内 `/tmp` 是持久的**，重启才没。原因写对要紧：照"tmpfs"去理解的人会以为
`df /tmp` 一看在根盘上就安全了。）机器 2026-09-29 10:40 重启后，
五场的原始逐帧（`/tmp/tier-b/*.jsonl`，48 MB）与当时 dump 出来的 `/tmp/mocap-model-3.json`
**都没了**；本归档里剩下的就是全部。⟹ **回放夹具的模型输入现在以本目录里那份
`mocap-model-3.json` 为准**（内容与当时同一份：`F*-replay.json` 的 `sigma_m` 与
`base_delay_ms` 是夹具直接读模型算出来落盘的，逐位对得上），验证器与 `run_matrix.sh`
的 `--model` 默认值已指向它。**判据① 那类"要按新口径重算就得整场重飞"的行，从此没有回头路。**

## 另有一批：2026-10-09 重搭之后那六场（`20261009-rebuild/`）

票 04 重搭了被控对象（`T_MAX_TOTAL` 11.8 → 46.3027 N）、票 05 把 `T0` 的参考接到
`mocap-model-5.json`（1.0 m 八字）之后，[票 06](../../../../.scratch/phase3/issues/06-fly-t0-seam3-four-criteria.md)
又跑了六场（`F0`–`F4` + `T0`）。**它们不并进上面那张表** —— 被控对象与参考都换过，
与 `F0`–`F4`（2026-09-29）**不是同一台机器上的读数**，混进一张表就会被当成可比的。

★ 好处是它们**带 `frames`/`truth` 入库**。本目录里带没带是**逐份**不同的：
`F0`–`F4` 那五份（2026-09-29 10:10）**没带**（`frames` 是空列表）⟹ 那五场
"要按新口径重算就得整场重飞"；票 20 的 `F0b`/`F1b`/`F0c`/`F1c` **带了**
（2026-10-08 的离线重判正是靠这一点）。☠️ 判据别写成 `grep -c '"frames"'` ——
**它区分不出来**（没带的那几份也出 `2`：`checks`/`dut` 的文字里也出现过这两个词）。
能区分的是长度：

```bash
python3 -c "import json,sys;print([len(json.load(open(f)).get('frames') or []) for f in sys.argv[1:]])" \
    F0-summary.json F0c-summary.json        # ⟹ [0, 1545]
```
逐场产物、时刻、工具链身份与三条必知事项见
[`20261009-rebuild/README.md`](20261009-rebuild/README.md)。
