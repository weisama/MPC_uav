# 票 02：三趟统一 nominal SITL（`R-hover` / `R-square` / `R-lemniscate`）

这里是**第 5 阶段报告**用的三趟**仿真**闭环：同一被控对象（`z0.param`，`--mismatch none`）、
同一控制器参数、参考**逐字**取自三份 offboard 包。它们补上"正方形从未在仿真里跑过"的缺口，
并让四源图的"仿真"格与"实飞/offboard/目标"格同口径。

时间：2026-10-17；环境：Jetson Orin NX（PX4 SITL `~/refs/px4-v1.14.3-build`，SIH 机型
`10046_sihsim_z0`；中间件 = **mavros 2.15.1**，`ROS_DOMAIN_ID=55`）。

## 三趟的配置

| tag | 参考包 | 模型 sha256 | 参考时长 span | `hold_s` | `ref_hold_s` | `ref_z_offset_m` |
|---|---|---|---|---|---|---|
| `R-hover` | `bag/3_traj1_hover` | `c3566acc…23a8707` | 39.64 s（参考恒为 0） | 78 | 30 | 1.0 |
| `R-square` | `bag/4_traj2_square` | `e8b7b884…0f238641` | 17.88 s（1×1 m） | 56 | 30 | 1.0 |
| `R-lemniscate` | `bag/5_traj3_lemniscate` | `8493bf72…34dd1bb` | 26.22 s（2×1 m） | 65 | 30 | 1.0 |

`hold_s` 的口径：`ref_hold_s + span + 8.0`（8.0 = 验证器的 `STEADY_S`，稳态窗口从最后一次
正常帧往前 8 s）。悬停那趟与实飞一致：**参考包只录到 39.65 s**（TRACK 中途截断），不是 60 s。

控制器参数**没有在命令行覆盖**：节点默认 `mass=0.75 kg`、`hover_thrust=0.1589`、
`rate_hz=100`，与实飞档一致。被控对象从 `z0.param` 现算：`t_max_total = 46.3028 N`、
`hover_thrust = 0.1588997`；速率上限 3.84/3.84/6.28 rad/s、推力上限 11.8 N 均来自
`src/nmpc/nmpc/params.py`（同一份被 DUT import 的代码，sha256 见各 `harness-identity.json`）。

## 模型（参考）怎么来的

```bash
# 正方形 / 8 字：与 mocap_model.py 的确定性输出逐字节可复现（sha256 见上表）
python3 tools/px4-sitl/mocap_model.py dump bag/4_traj2_square \
    --noise-bag bag/1_static --win-s 0.2 \
    --json tools/report/evidence/sitl/models/R-square-model.json
python3 tools/px4-sitl/mocap_model.py dump bag/5_traj3_lemniscate \
    --noise-bag bag/1_static --win-s 0.2 \
    --json tools/report/evidence/sitl/models/R-lemniscate-model.json

# 悬停：TRACK 是包里最后一个状态（录包在 TRACK 中途截断）⟹ track_window() 返回零长窗口、
# mocap_model.py 的 noise_sigma() 会抛 ValueError。build_hover_model.py 只把窗口终点改成包尾，
# 其余逐字复用 build_model()；同样可复现（重跑后 sha256 不变）。
python3 tools/report/evidence/sitl/build_hover_model.py
```

`--noise-bag bag/1_static --win-s 0.2` 必须显式给：`mocap_model.py` 的默认 noise-bag 是
**绝对路径**，会写进模型的 `noise.static_bag` 字段并改变文件哈希（内容其余逐字节相同）。
2026-10-17 复算过：三份模型按上面命令重跑，sha256 与入库文件**完全一致**。

## 怎么跑的

中间件（**另一个终端**先起，三趟共用）：

```bash
ROS_DOMAIN_ID=55 ros2 launch bringup mavros.launch.py fcu_url:=udp://:14540@127.0.0.1:14580
```

每趟（`R-square` 为例；另两趟只换 tag/模型/hold_s，见 `R-*/command.txt`）：

```bash
source /opt/ros/humble/setup.bash && source ~/mavros_ws/install/setup.bash
source ~/px4_msgs_ws/install/setup.bash && source install/setup.bash
export ROS_DOMAIN_ID=55
python3 tools/report/evidence/sitl/run_nominal_sitl.py --tag R-square \
    --model tools/report/evidence/sitl/models/R-square-model.json \
    --reference-source traj --mismatch none --hold-s 56 --ref-hold-s 30 \
    --outdir /tmp/phase5-sitl/R-square \
    --json tools/report/evidence/sitl/R-square/R-square-summary.json \
    --debug-dump tools/report/evidence/sitl/R-square/R-square-console.log
# 跑完、删原始 JSONL 之前：
python3 tools/report/evidence/sitl/extract_solver_summary.py \
    --jsonl /tmp/phase5-sitl/R-square/R-square-dut.jsonl --tag R-square \
    --out tools/report/evidence/sitl/R-square/solver-summary.json
```

`run_nominal_sitl.py` 是 `verify_tier_b_seam3.py` 的**最小包装**，补三处票 13（换 mavros 载体）
之后失效的接线；它**不改**共享的 `verify_tier_b_seam3.py`：

1. 给 DUT 追加 `offboard_prestream:=true`（与实飞同口径）。不补则 PX4 等 setpoint 才进
   OFFBOARD、DUT 等 OFFBOARD 才发 setpoint ⟹ 死锁（首跑实测 0 帧）。
2. 把新版 JSONL 的 `mode`（字符串）/ `armed` 合成回旧判据要读的 `nav_state`（14=OFFBOARD）/
   `arming_state`。不补则判据②「不失控」**结构性**读成 False（首跑实测：窗口内 793 帧全是
   `mode="OFFBOARD"`，而 `n_offboard=0`）。
3. 在 `FRAME_KEYS` 后追加 `thrust` / `body_rate_x/y/z` / `mode`。旧键 `thrust_body_z` 在新载体
   里**不存在**，不补则 summary 里推力列全空（票 01 要用它换 `u_thrust_n`）。

## 产物（每趟）

| 文件 | 内容 |
|---|---|
| `<tag>-summary.json` | 验证器 obs 原样：`frames`（逐帧）+ `truth`（ENU 真值）+ 判据读数。**票 01 的输入** |
| `solver-summary.json` | 从原始 DUT JSONL 抽的 `solve_ms` 均值/p50/p95/最坏、退化/失败拍数、饱和拍数 |
| `<tag>-console.log` | 验证器控制台全文（`--debug-dump`） |
| `command.txt` / `harness-identity.json` | 复现命令 + 全部输入的 sha256（模型/包/控制器/被控对象/夹具/中间件） |

原始 DUT JSONL（`/tmp/phase5-sitl/<tag>/<tag>-dut.jsonl`，5–9 MB）**不入库**；`/tmp` 开机即清，
所以求解汇总必须当场抽出来。`summary.json` 里的 `frames` 是**验证器停机前**抓的快照，
比原始 JSONL 少收尾那 ~100 帧（原始 JSONL 的帧数见 `solver-summary.json` 的 `n_frames`）。

### `frames` 里票 01 要用的列

- `wall_ns`（epoch ns，配 `truth` 第 7 列）、`x0`（10 维，`x0[6:10]` = 四元数）、
  `x_ref0`（3 维绝对参考）、`u` = `[F(N), wx, wy, wz]`、`thrust`（归一化 0–1）、
  `body_rate_x/y/z`（rad/s）、`nav_state`（由 `mode` 合成，14=OFFBOARD）、`fallback`。
- **`u_thrust_n` 的换算口径**：`thrust × 46.3027` = **实际发出去**的推力；它与同一帧的 `u[0]`
  差一个 `df`（垂直积分偏置），且是**前一拍**的 `df`（逐帧实测：`thrust×46.3027 − u[0]
  = df_prev`，见 `mpc_node.py:1424-1440` 的 ☠️）。票 01 若只用 `u[0]`，会漏掉这一项。
- `truth` 每行 8 列：`[t_rel_s, x, y, z, vx, vy, vz, wall_s]`，**ENU**（z 向上为正）。

## 结果摘要（2026-10-17 这三趟）

| tag | DUT 帧 / 真值点 | 稳态误差（水平均值/最大，垂直绝对值均值） | 稳态高度峰峰值 | 求解 p50 / p95 / 最坏（ms） | 退化 / 失败 / 饱和 |
|---|---|---|---|---|---|
| `R-hover` | 7774 / 20410 | 0.0114 / 0.0203 m；0.0002 m | 0.0008 m | 3.154 / 3.624 / **20.138** | 0 / 0 / 0 |
| `R-square` | 5602 / 15352 | 0.0121 / 0.0246 m；0.0002 m | 0.0010 m | 3.071 / 3.313 / **12.916**（稳态最坏 6.462） | 0 / 0 / 0 |
| `R-lemniscate` | 6529 / 17251 | 0.0130 / 0.0317 m；0.0003 m | 0.0015 m | 3.160 / 3.612 / **10.738** | 0 / 0 / 0 |

（稳态误差的水平**均值** `h_mean` 从各自的 `steady` 块现读；三趟都 < 0.02 m。移动段的跟踪误差
`h_mean` 分别为 — / 0.398 / 0.505 m，spec 没有数值界，**只留档**。）

三趟的验证器判据：**没有一条判为假**（每份 `checks` 16 条里 `ok=False` 0 条、控制台 `❌` 0 个），
但**不是 16 条全判过** —— `ok=True` 分别 10 / 11 / 11 条，其余 `ok=None` 是**本场没做**：

- `R-square` / `R-lemniscate` 各 5 条：判据③（`--extra-delay-ms 0` 未注入 50 ms 延迟）、
  判据④（未注入求解失败）、判据②·轨迹1 原地悬停（`--reference-source traj` 下不判）、
  以及两条**本来就不设判据**的行（真值倾角旁证、⑦ D6 结论行）；
- `R-hover` 多 1 条：判据②·轨迹2 八字 —— 该趟参考恒为 0（悬停），时间线里没有一帧在动。

已判过的那些：

- `reference_source="traj"`、`mismatch="none"`，`frames`/`truth` 非空；
- 离线段高度区间 `R-hover [0.151, 1.736]`、`R-square [0.154, 1.736]`、
  `R-lemniscate [0.154, 1.776]` m —— 最低点 > 0.05 m（未触地）；
- 参考移动段整段落在 OFFBOARD 窗口内（正方形/8 字），且窗口内**每一帧**都是 OFFBOARD；
- `z_min_all ≈ 0` 只是起飞前停在地面，不是触地。

## 已知异常 / 口径注意

1. **起飞爬升有明显欠阻尼**：三趟都在离地后 2–5 s 冲到 1.7–1.9 m（目标 1.0 m），
   再回落到 0.4–0.7 m，约 15 s 后收敛。archived Phase 3 的 F0/F4 也有同样现象
   （峰值 1.767/1.761 m），所以**不是本批新引入的**；但本批回落更深。报告若画高度曲线，
   这一段会非常显眼。
   - 另有一次**未入库**的 `R-lemniscate` 试跑（同一配置，在修 `FRAME_KEYS` 之前）在 t≈14.4 s
     跌到 **z=0.012 m**（1.2 cm AGL，持续约几十毫秒）；入库这一趟最低 0.154 m。
     同一个配置两次结果差这么多 ⟹ 起飞段对时序/负载敏感，**别拿单趟最低点当结论**。
2. **求解耗时的最坏值受本机负载影响**：首轮三趟跑的同时，另一个会话在跑全量 pytest
   （13:56:37 起，load average ≈ 4.7）。p95 只有 3.6–4.1 ms、`solve_ok` 全真、退化 0，
   但最坏值出现 20.1/23.3 ms 的**单拍**尖峰（当时的 `over_10` 分别 7/11/2 拍）。
   **2026-10-17 15:09 在机器空闲（load ≈ 0.16）时只重跑了最坏的 `R-square`**：
   `over_10` 从 11 拍降到 1 拍，稳态最坏 23.287 → **6.462 ms**，p95 4.050 → **3.313 ms**；
   素材已替换为本轮读数（旧一轮的 summary 只在 `/tmp` 备份，未入库）。
   `R-hover` / `R-lemniscate` 的 `over_10` 只有 7 / 2 拍、p95 3.6 ms，保留首轮读数。
3. **`summary.json` 的 `dut.ages_ok=false` 是旧口径**：B 档只有 `age_state_s` 一条分龄，
   A 档的 `age_pos_s`/`age_att_s` 本就不存在；控制台判据⑧ 已按 B 档口径判过（通过）。
4. **`判定②·轨迹1`（原地悬停）在这三趟里是「没做」**：`--reference-source traj` 下不判；
   悬停的真身是 `hold` 档的场次。`R-hover` 这一趟的参考恒为 0（也不动），所以
   `traj.lemniscate.ran=false` 是**预期**，不是夹具故障。
