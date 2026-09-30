# `tools/rti-convergence/` —— 冻结状态下的 RTI 迭代收敛扫描

[ticket 16](../../.scratch/phase2/issues/16-ocp-yaw-weights-mirror-well.md) 裁定 `R_yaw` 的第一手证据。

## 它在测什么

把节点的**每拍行为**单独拿出来、把测量与参考**钉死**。于是"拿上一帧解当 warm start"这件事
就退化成一个**不动点迭代**：

```
u_{k+1} = RTI(x0, x_ref, u_k)          # x0 / x_ref 全程不变
```

这正是台架 A 档黑盒（`src/nmpc/test/test_mpc_node.py` 的假 PX4 状态静止）跑的那个东西 ——
离线重放已与 DUT 自己 JSONL **逐拍对上**（DUT tick 9 = 离线第 5 次：`F=8.9336, ω_y=0.104645, ω_z=0.185475`）。

⚠️ 它**只描述冻结状态**下的迭代，不证明实机行为 —— 真机上 `x0` 每拍都在变。

## 为什么需要它：一个踩过的坑

判"不收敛"有个陷阱：**探针截断太早，会把慢收敛读成极限环**。

本工具初版只跑 60 拍，看到 10 m + `R_yaw = 50` 的 `|Δu| = 0.888`，写下"没有不动点、逐拍乱跳"；
跑到 400 拍才知道它是 `0.00026` 的**几何衰减**（每 ~100 拍小 10×）——**有**不动点，只是要 ~1000 拍。
同一版还把 `R_yaw = 5e2` 在 5–7 m 的**未 settled 瞬态**读成了"同样发散"，重跑后同样翻。

⟹ 本工具一律**按拍记录整条衰减曲线**，并按曲线判 `极限环 / 慢收敛 / 已收敛`。
`DEFAULT_TICKS = 400`，**别调小**。

## 跑法

```bash
source install/setup.bash
export ACADOS_SOURCE_DIR=$HOME/acados LD_LIBRARY_PATH=$ACADOS_SOURCE_DIR/lib:$LD_LIBRARY_PATH
# 一次跑出两张表并各自落 JSON（★ 每张表只算一次，打印与落盘共用同一份结果）
python3 tools/rti-convergence/rti_convergence.py \
    --out       tools/rti-convergence/evidence/scan.json \
    --sweep-out tools/rti-convergence/evidence/r_yaw_sweep.json
python3 -m pytest tools/rti-convergence -q                           # 5 条判据
```

**两张表**：表 1 = 偏移 × `R_yaw`（哪个权重在哪个工况有极限环）；表 2 = `R_yaw` 单轴 @ 10 m
不可行步进（**收敛判据落在哪两档之间**，`ocp.py` 引用它的那个结论就靠这张）。

⚠️ 表里 **"极限环"后面括号里的数与"末段"列是 `classify` 的 `end` = 后 10 拍的 `|Δu|` 最大值**，
**不是末拍那一次** —— 两者差得不多但定义不同，引用时别混（第一手：`R_yaw=50` @10 m 的
`end = 3.31e-4`、末拍 `= 2.69e-4`）。

## 读数小结（2026-09-30，400 拍，`qw=0.1 / qatt=2.0`，`evidence/scan.json`）

| 目标偏移 | `R_yaw = 5e2`（落地） | `R_yaw = 50`（已撤回） |
|---|---|---|
| 2 m | 已收敛 | 已收敛 |
| 3 m | 已收敛 | **极限环**（`|Δu| = 3.26`） |
| 4 m | 已收敛 | **极限环**（2.63） |
| 5 m | 已收敛 | **极限环**（1.37） |
| 6 m | **极限环**（0.595，`ω_x`） | **极限环**（0.739） |
| 7 m | **极限环**（1.622，`F`） | 已收敛 |
| 8 m | 已收敛 | 已收敛 |
| 10 m | 已收敛（第 50 拍起 `|Δu| = 0`） | **慢收敛**（拍 50 = 0.888、拍 200 = 0.027、**末拍 = `2.7e-4`**） |
| 12 m | 已收敛 | 未定（`|Δu|` 在 4…12 之间**不衰减 —— 单调上升**，4.28 → 10.30） |

⟹ **两个权重各有各的坏区**，稳定区逐工况变。这里选 `5e2` 的理由是
"**黑盒探的那个点（10 m）+ 参考实现的默认值**"，**不是**"500 更稳"—— 见 6 m / 7 m 两行。
（"未定"那一格是这个粗糙判据的诚实出口：它既没收敛、又不满足"末段不衰减"的极限环判据。
要定性得另画相图，本工具不做。）

## 文件

| 文件 | 作用 |
|---|---|
| `rti_convergence.py` | 迭代扫描核心 + CLI（`iterate` / `step_sizes` / `classify` / `scan` / `sweep_r_yaw`） |
| `test_rti_convergence.py` | 5 条判据：落地值 10 m 收敛 / 撤回值 10 m 未 settled / 两组判得开 / `Weights` 不漂 / 收敛边界在 50…55 |
| `evidence/scan.json` | 表 1 的原始读数（偏移 × `R_yaw`） |
| `evidence/r_yaw_sweep.json` | 表 2 的原始读数（`R_yaw` 单轴 @ 10 m） |
