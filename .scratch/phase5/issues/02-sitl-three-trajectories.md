# 02: 三趟统一 nominal SITL（悬停 / 正方形 / 8 字）

Status: done
Type: task

**What to build:** 用同一被控对象、同一控制器参数、逐字取自三份 offboard 包的参考，跑三趟
**nominal** SITL 闭环，补上"正方形从未在仿真里跑过"的缺口，并把三张图的仿真格统一到同一口径。

**Blocked by:** —（无）

## 为什么必须新跑

- 正方形**从未在 SITL 里跑过**：`tools/px4-sitl/verify_tier_b_seam3.py:13-16` 明确写过判据里删掉了它；
- 已有的悬停是 `hold` 参考（Tier A），八字 T0 是 `mass+30%` 失配；
- 直接拿这两趟当四源图的"仿真"格，**被控对象与参考口径都不一样**，读数不可横比。

## 做法

1. 用 `tools/px4-sitl/mocap_model.py` 从 `bag/3_traj1_hover`、`bag/4_traj2_square`、
   `bag/5_traj3_lemniscate` 各提取一份参考模型（落 `tools/report/evidence/sitl/models/`）。
2. 用 `tools/px4-sitl/verify_tier_b_seam3.py --reference-source traj --model <model>` 跑三趟：
   - tag：`R-hover`、`R-square`、`R-lemniscate`；
   - **不加** `--mismatch`（nominal）；
   - 控制器参数与实飞一致：质量 0.75 kg、`hover_thrust=0.1589`、`T_max≈46.3027 N`、
     速率限制 3.84 / 3.84 / 6.28 rad/s、推力上限 11.8 N；
   - 悬停窗口与实飞一致（参考包只到 39.65 s）。
3. 产物落 `tools/report/evidence/sitl/<tag>/`：
   - `<tag>-summary.json`（含 `frames` 与 `truth`）；
   - `solver-summary.json`：从原始 DUT JSONL 抽取 `solve_ms` 的均值/p95/最坏、退化拍数、饱和计数
     （⚠️ Tier B 的 summary 投影**不含** `solve_ms`；在删原始 JSONL 前抽）；
   - `command.txt`、`harness-identity.json`（模型/包 sha256、控制器/被控对象哈希）。
4. 三趟的 `truth` 与 `frames` 交给票 01 提取成 `mpc_sim` 源；原始 DUT JSONL 不入库。

## 检查清单

- [x] 三趟都完成，`reference_source = traj`，truth 与 frames 非空
- [x] 三趟的参考分别逐字来自 `bag/3`、`bag/4`、`bag/5`（模型 sha256 可核）
- [x] 三趟都是 nominal；`harness-identity.json` 里没有 `mass+30` 之类的失配标记
- [x] `solver-summary.json` 有均值/p95/最坏/退化拍数/饱和计数
- [x] 飞行段/仿真段的高度范围合理，没有触地、没有持续发散
- [x] 不新增依赖、不改控制器代码；只新增 `tools/report/evidence/sitl/**`
- [x] 跑完后确认没有残留 SITL 进程、没有占用真机链路

## 验收

- 三趟 summary 能被票 01 的提取器直接读入；
- 每趟的参考、被控对象、控制器参数、运行命令可追溯；
- 完成后切 `ready-for-human`，附"三趟窗口/高度范围/求解汇总/异常"给操作者审。

## 注意

- 这是**仿真**，不需要装桨或现场动作；但会吃 Jetson CPU，不要在真机通电/飞行时跑。
- 全量测试/部分 acados 脚本需要 `ACADOS_SOURCE_DIR`；按仓库既有注意事项设置。
- 已有 Tier A 与 T0 **不改、不删**，作为第 8/10 章的失配鲁棒性证据引用。

---

## Comments

### 2026-10-17 —— 三趟已跑完，产物在 `tools/report/evidence/sitl/`，请操作者审

**先说结论**：三趟都跑通、两处判据全绿（每份 summary 的 `checks` 16 条里 `ok=False` **0 条**，
三份控制台 `❌` 计数 **0**）。但为了让票 13（换 mavros 载体）之后的 DUT 能在这套 **Phase 3 的**
SITL 驱动器上跑起来，本轮在 `tools/report/evidence/sitl/` 下加了一组脚本（核心是
`run_nominal_sitl.py` 这个最小包装），对共享 harness **一行未改**。这几处适配请操作者过目（见 §2）。

#### 1. 三趟读数（2026-10-17，同一被控对象 `z0.param`、`--mismatch none`）

| tag | DUT 帧 / 真值点 | 参考 span | OFFBOARD 窗口（记录器钟） | 离地段高度 | 稳态误差 水平均值/最大；垂直绝对值均值 | 求解 p50/p95/最坏（ms） |
|---|---|---|---|---|---|---|
| `R-hover` | 7774 / 20410 | 39.64 s（参考恒 0） | —（traj 档不判轨迹） | [0.151, 1.736] m | 0.0114 / 0.0203；0.0002 | 3.154 / 3.624 / **20.138** |
| `R-square` | 5602 / 15352 | 17.88 s（1×1 m） | [14.92, 70.93] s，参考移动段 [36.62, 44.60] 整段在内 | [0.154, 1.736] m | 0.0121 / 0.0246；0.0002 | 3.071 / 3.313 / **12.916**（稳态最坏 6.462） |
| `R-lemniscate` | 6529 / 17251 | 26.22 s（2×1 m） | [15.24, 80.25] s，参考移动段 [36.96, 49.14] 整段在内 | [0.154, 1.776] m | 0.0130 / 0.0317；0.0003 | 3.160 / 3.612 / **10.738** |

- 三趟 `fallback` 分布只有 `prestream` / `warmup` / `normal`，**没有** `hold`/`hover` 兜底；
- 退化拍数 **0**、求解失败 **0**、饱和拍数 **0**（`sat` 全是空列表）；
- 移动段跟踪误差（**只留档，spec 无数值界**）：正方形水平均值 0.398 m / 最大 0.615 m；
  8 字 0.505 / 0.819 m。**没有**拿它判成败，操作者 2026-09-30 的「不编阈值」裁定照旧。
- 每份 `harness-identity.json` 里 `mismatch="none"`、`mismatch_markers=[]`；
  被控对象是标称 `z0.param`（sha256 `94e64ec8…`），不是 `mass+30`。

#### 2. 与票面步骤不一致的地方（**这是本轮最需要审的部分**）

票面写的是「用 `verify_tier_b_seam3.py --reference-source traj --model <model>` 跑三趟」。
那份驱动器最后一次修改在 Phase 3（`7dd7133`，2026-10-09），**早于**票 13（2026-10-13）把 DUT
下行从 px4_msgs/uXRCE-DDS 换到 mavros。直接照跑会**静默**得到 0 帧或结构性假红。
本轮**没有改 `verify_tier_b_seam3.py`**（它被归档证据与别的票共用）：外部前置一项 +
在 `tools/report/evidence/sitl/run_nominal_sitl.py`（最小包装）里做三处**代码**适配。

0. **（外部前置，不在脚本里）另起真 mavros**：DUT 发 `/mavros/setpoint_raw/attitude`，
   SITL 驱动器的 `BWorld` 里**没有** mavros。本轮在另一个终端起
   `bringup mavros.launch.py fcu_url:=udp://:14540@127.0.0.1:14580`（`ROS_DOMAIN_ID=55`），
   DUT 才算有命令面。首跑（没起 mavros 的那次）实测 DUT 0 帧、闸门日志 `not_offboard×17997`。

以下三项是**脚本里的代码适配**（`run_nominal_sitl.py`）：

1. **`offboard_prestream:=true`**：PX4 要先收到 `SET_ATTITUDE_TARGET` 才进 OFFBOARD，
   DUT 的闸门又要求 `mode=="OFFBOARD"` 才发 ⟹ 互等死锁。实飞档同样用 `offboard_prestream`
   破局（`run/z0.py` 的模型预测控制档）。包装把它补进 DUT 参数，`obs["params"]` 里有原文。
2. **`mode` → `nav_state` 合成**：票 13 之后 DUT 的逐帧 JSONL 写 `mode`（字符串）而**没有**
   `nav_state`（整数）。`verify_tier_b_seam3.py:913` 的「不失控」主判读 `f.get("nav_state")==14`
   ⟹ 第一版包装没补这一列时，窗口内 793 帧 `mode` 全是 `"OFFBOARD"`、而 `offboard_kept=False`。
   包装在 `Dut.frames()` 上**只补判据层的键**，原始 JSONL 不动。
3. **`FRAME_KEYS` 追加 `thrust` / `body_rate_x/y/z` / `mode`**：旧键 `thrust_body_z` 在新载体里
   **不存在**，summary 里推力列会全空 —— 而票 01 的 `u_thrust_n` 正要读它。

另有一处只在**悬停那支**：`bag/3_traj1_hover` 是录到 TRACK 中途截断的，
`mocap_model.track_window()` 返回 `t0==t1` 的零长窗口 ⟹ `noise_sigma()` 抛
`ValueError: cannot reshape array of size 0`。`build_hover_model.py` 只把窗口终点改成包尾，
其余逐字复用 `build_model()`；`models/R-hover-model.json` 的 `segment.how` 里留了这句。

**未改的东西**（可核）：`git status` 里 `src/`、`tools/px4-sitl/` 均无改动；本轮新增只在
`tools/report/evidence/sitl/**`。没有新增依赖。

#### 3. 参考的可追溯性（复算过）

三份模型用 README 里的命令重跑，sha256 **逐字节相同**：

- `R-hover-model.json` `c3566acc…23a8707`（`build_hover_model.py` 重跑后不变）
- `R-square-model.json` `e8b7b884…0f238641`（`mocap_model.py dump bag/4_traj2_square
  --noise-bag bag/1_static --win-s 0.2` 重跑后不变）
- `R-lemniscate-model.json` `8493bf72…34dd1bb`（同法重跑后不变）

⚠️ `--noise-bag` 必须**显式给相对路径** `bag/1_static`：默认值是绝对路径，会写进模型的
`noise.static_bag` 字段并改变文件哈希（其余内容逐字节相同）。每份身份文件的
`参考包（逐字来源）` 里记了 `.db3.zstd` 与 `metadata.yaml` 的 sha256。

#### 4. 已知异常（请重点看第 1、2 条）

1. **起飞爬升欠阻尼**：三趟都在离地后 2–5 s 冲到 1.7–1.9 m（目标 1.0 m），再回落到 0.4–0.7 m，
   约 15 s 收敛。archived Phase 3 的 F0/F4 峰值也有 1.767/1.761 m ⟹ **不是本批新引入的**，
   但本批回落更深。另有**一次未入库**的 `R-lemniscate` 试跑（同配置，在补 `FRAME_KEYS` 之前）
   在 t≈14.4 s 跌到 **z=0.012 m**（1.2 cm AGL）；入库这一趟最低 0.154 m。**同配置两趟差这么多**
   ⟹ 起飞段对时序/负载敏感，报告里别拿单趟最低点当结论。
2. **求解最坏值受本机负载污染（已重跑最坏那趟）**：首轮三趟跑的同时，另一个会话在跑全量
   pytest（`/tmp/pytest_ch03_full.log`，13:56:37 起，load average ≈ 4.7）。p95 只有
   3.6–4.1 ms、`solve_ok` 全真、退化 0，但最坏出现 20.1 / 23.3 ms 的**单拍**尖峰
   （`over_10` 7/11/2 拍）。按操作者裁定「只重跑最坏的那趟」，2026-10-17 15:09 在机器空闲
   （load ≈ 0.16）时重跑了 `R-square`：`over_10` 11 → **1** 拍、稳态最坏 23.287 → **6.462 ms**、
   p95 4.050 → **3.313 ms**；入库读数已换成本轮。`R-hover` / `R-lemniscate`（最坏 20.1 / 10.7 ms）
   保留首轮读数。旧一轮的 summary 只在 `/tmp` 备份，未入库。
3. `summary.json` 的 `dut.ages_ok=false` 是 **A 档旧口径**：B 档只有 `age_state_s` 一条分龄，
   A 档的 `age_pos_s`/`age_att_s` 本就不存在；控制台判据⑧ 已按 B 档口径判过（通过）。
4. `判据②·轨迹1`（原地悬停）在这三趟里是「**没做**」：`--reference-source traj` 下不判；
   `R-hover` 的参考恒为 0，所以 `traj.lemniscate.ran=false` 是预期，不是夹具故障。

#### 5. 给票 01 的口径提醒

- `frames` 里 `u = [F(N), wx, wy, wz]`；**实际发出去**的推力是 `thrust × 46.3027`。
  它与同一帧 `u[0]` 差一个 `df`（垂直积分偏置），而且是**前一拍**的 `df`
  （逐帧实测：`thrust×46.3027 − u[0] = df_prev`，见 `mpc_node.py:1424-1440`）。
  只用 `u[0]` 会漏掉这一项。
- `x0[6:10]` 是四元数；`truth` 每行 8 列 `[t_rel, x, y, z, vx, vy, vz, wall_s]`，**ENU**。
- `summary.json` 的 `frames` 是验证器停机前抓的快照，比原始 JSONL 少收尾 ~100 帧；
  原始帧数见 `solver-summary.json` 的 `n_frames`。

#### 6. 收尾检查

- 两轮跑完后均已停掉本轮起的 mavros（含 15:09 的 `R-square` 重跑）；`ps` 里没有
  `px4_sitl` / `MicroXRCE` / `mocap_replay`，也没有 14540/14580/8888 的 UDP listener。
- **没有碰真机链路**：全程 `ROS_DOMAIN_ID=55`（真机 0、bench 73）。
- 另一个会话的 pytest 子进程（`mpc_node.py -p tau_s:=0.0`，cwd `~/.cache/mpc-ocp-codegen`）
  仍在跑；**不是本轮起的，没有杀**。
- 相关测试：`pytest tools/px4-sitl/test_tier_b_judges.py tools/px4-sitl/test_mocap_model.py -q`
  ⟹ **103 passed**。全量套件由另一个会话在跑，本轮没有重复起一份。

#### 7. 操作者裁定（2026-10-17）与落地

1. **起飞欠阻尼**：**接受为 nominal 仿真的已知形态**。报告如实描述，引 archived Phase 3 的
   F0/F4 同现象作旁证，不再深挖。
2. **求解最坏值**：**只重跑最坏的那趟**（`R-square`）。已在机器空闲时重跑，读数已替换
   （见 §4.2 与 §1 表）。
3. **包装式适配**（另起 mavros + 脚本内三处补丁）：**认，作为 Phase 5 正式口径**。
   后续票（含票 01 取数）沿用 `tools/report/evidence/sitl/run_nominal_sitl.py` 这条路径；
   `README.md` 已按此口径记录，共享 `verify_tier_b_seam3.py` 仍不改。
4. **悬停模型绕过**（`build_hover_model.py` 把噪声窗口终点改到包尾）：**认，属数据缺陷的
   合理绕行**；绕过写在 `models/R-hover-model.json` 的 `segment.how` 与各
   `harness-identity.json` 里。

本票的人工裁定项已全部落定。操作者 2026-10-17 说「收工」⟹ 按「做完了」义转 `done`
（不是「裁定不做了」那种收口；四条裁定即上面 §7 的内容，均已落地）。

#### 8. 重跑与票 01 的时序冲突（**请操作者转给票 01**）

15:09 的 `R-square` 重跑**替换了** `tools/report/evidence/sitl/R-square/R-square-summary.json`
（旧文件 sha256 `d687277a…`，大小 9,119,717 B）。但票 01 已在 15:0x 提交（`045615b`），
其 `docs/report/data/square.json` 的 `inputs.sim_summary.sha256` 仍是旧快照
（`d687277a…`），而当前文件已不同。⟹ **票 01 的 `square.csv` / `square.json` 需要重跑
`python3 tools/report/build.py --scene square`**（`build.py` 的 sha256 核对会报这一条，
不会静默通过）。本票**没有**改票 01 的任何文件（并行会话的在制品），只在此记录。
`hover` / `lemniscate` 两源的 summary 未变，不受影响。
