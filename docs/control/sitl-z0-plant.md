# z0 同构 SITL 被控对象（ticket 08）—— 机型、参数、失配开关与验收

> **一句话**：SITL 里那架飞机现在**是 z0**（0.75 kg / 力臂 53.7 mm / 单电机 2.95 N），
> 不再是 x500 量级的通用四旋翼；四个参数旋钮各能单独偏 ±30 %，用来给
> [ticket 12](../../.scratch/phase2/issues/12-mismatch-sweep.md) 扫"错参数下还能不能飞稳"。
>
> 上游基线（怎么编出这个 SITL、`px4_msgs` 怎么钉的）在
> [`px4-sitl-baseline.md`](px4-sitl-baseline.md)（ticket 01）；**本页只讲 z0 这架被控对象。**
>
> 单一真源：[`tools/px4-sitl/z0-plant/plant_spec.py`](../../tools/px4-sitl/z0-plant/plant_spec.py)
> —— 机型文件与 10 份参数覆盖层**全部由它生成**，文档里的表也是它打印的。
> 改参数**只改那份 spec 再 `--emit`**，不要手改产物（手改必然与文档漂移）。

---

## 0. 先读这一段：三个"一句话"

1. **为什么不用现成机型**：上游那些机型（`10040_sihsim_quadx`、`gz_x500`）的动力学与 z0 差得远。
   上游 SIH 四旋翼**一个 `SIH_*` 都不设**，全取默认值（`sih_params.c` 的 `PARAM_DEFINE_FLOAT`）：
   `SIH_MASS 1.0` / `SIH_L_ROLL 0.2` / `SIH_T_MAX 5.0`（单电机）/ `SIH_IXX 0.025`；
   而机型文件里 `CA_ROTORn_PX/PY = ±0.15`（**与 SIH 的 0.2 自相矛盾**）。
   对 z0 的 `0.75` / `0.0537401` / `2.95` / `0.002166`：**力臂差 2.8 倍、`IXX` 差 11.5 倍**。
   **"在 x500 上能飞"不蕴含"在 z0 上能飞"**，所以拿它们做 SITL 验收等于把判据换成另一台飞机。
   它们**只准当最早期冒烟目标**（`run_sitl.sh` 不选机型时会自己把这条警告打出来）。
2. **被控对象是 SIH（`PX4_SIMULATOR=sihsim`），不是 Gazebo**：SIH 是个纯刚体 + 一阶电机
   的小模型（`src/modules/simulation/simulator_sih/sih.cpp`），参数表全在 `SIH_*` 里
   —— 这正是"按 z0 取值"能落地的原因。**Gazebo 那条路本张不走。**
3. **失配是交付物，不是副作用**：`±30 %` 的开关是**刻意留的**，
   [ticket 11](../../.scratch/phase2/issues/11-sitl-tier-b-seam3.md) 的判据里就有
   "错参数也能飞稳"这一条 —— 所以开关在**本张**就建好，不等到 12 现搭。

---

## 1. 交付物一览

| 交付物 | 路径（入库） | 说明 |
|---|---|---|
| 单一真源 | `tools/px4-sitl/z0-plant/plant_spec.py` | 参数表 + 变体生成 + 一致性校验 |
| 机型文件 | `tools/px4-sitl/z0-plant/airframes/10046_sihsim_z0` | 机器身份：构型 / 几何 / 输出分配 / 检查位 |
| 参数覆盖层 ×10 | `tools/px4-sitl/z0-plant/params/*.param` | `z0`（对照组）+ 4 旋钮 × 2 方向 + `z0-thrcurve` |
| 装机脚本 | `tools/px4-sitl/install_z0_airframe.sh` | 装进已构建的 SITL，**不需要重编** |
| 起机脚本 | `tools/px4-sitl/run_sitl.sh --plant z0 [--mismatch …]` | 选变体 / 清持久化参数 / 列变体 |
| 验收脚本 | `tools/px4-sitl/verify_z0_plant.py` | 45 项静态 + 5 项动态检查，见 §6 |
| 取证工具 | `tools/px4-sitl/px4_console.py` | 用 pty 驱动 pxh 控制台（为什么要 pty 见它自己的 docstring） |

---

## 2. 参数表（`plant_spec.py --table` 的**逐字输出**）

数值分**五档**诚实标注（`plant_spec.py` 的模块 docstring 里是同一份定义）：

- **实测** —— z0 自己的第一手读数（出处多为 [`docs/assets/z0.params`](../assets/z0.params)）；
- **推得** —— 由实测值经**明写的算式**得到；
- **照抄** —— **z0 侧根本没有对应物**，照抄上游同构文件或上游默认值；
- **决定** —— **项目决定**，刻意偏离 z0（不是读数、也不是推论 —— 单独成档是为了不冒充证据）；
- **占位** —— 没有 z0 数据，取默认值或量级估，**待 [ticket 10](../../.scratch/phase2/issues/10-param-measure-backfill.md) 回填**。

⚠️ 后三档是 ticket 08 复核时**补**出来的：原先只有三档，于是"照抄上游"被记成 `实测`、
"项目决定"被记成 `推得` —— 那是**假证据**，也是本页第一版的一处真错误。

| 参数 | 值 | 档次 | 出处 / 理由 |
|---|---|---|---|
| `SIH_MASS` | `0.75` | 实测 | z0 质量 750 g（hardware.md §6.1） |
| `SIH_T_MAX` | `2.95` | 推得 | 按电池的可用总推力 11.8 N ÷ 4（phase2-implementation.md §3.3） |
| `SIH_Q_MAX` | `0.1475` | 推得 | = CA_ROTOR_KM × SIH_T_MAX ⟹ 被控对象的偏航力矩/推力比 = 分配器以为的 KM |
| `SIH_L_ROLL` | `0.0537401` | 推得 | 152 mm 对角线 → 机臂 0.076 m → 单轴投影 0.0537 m（与 CA_ROTOR_PX/PY 必须相等） |
| `SIH_L_PITCH` | `0.0537401` | 推得 | 同上 |
| `SIH_IXX` | `0.002166` | 占位 | 点质量上界 m·l²；真值未辨识（见 sitl-z0-plant.md §8） |
| `SIH_IYY` | `0.002166` | 占位 | 同 IXX |
| `SIH_IZZ` | `0.004332` | 占位 | = 2·IXX（细长体量级估） |
| `SIH_IXY` | `0` | 占位 | 对称假设（无实测） |
| `SIH_IXZ` | `0` | 占位 | 对称假设（无实测） |
| `SIH_IYZ` | `0` | 占位 | 对称假设（无实测） |
| `SIH_T_TAU` | `0.05` | 占位 | SIH 默认值；z0 内环时间常数未辨识（phase2 §5 的 P4 才是它） |
| `SIH_KDV` | `1` | 占位 | SIH 默认；保持默认以免给被控对象偷偷加阻尼 |
| `SIH_KDW` | `0.025` | 占位 | SIH 默认；同上 |
| `SIH_VEHICLE_TYPE` | `0` | 实测 | z0 是多旋翼（hardware.md §6.1）；值 0 = SIH 的 VehicleType::MC |
| `CA_ROTOR_COUNT` | `4` | 实测 | z0 是四旋翼 |
| `CA_ROTOR0_PX` | `0.0537401` | 推得 | X 形；★ **刻意不取 z0.params 的 `0.15`**（那是 PX4 通用 X 型默认值、比实测几何大 2.8 倍，见 hardware.md:38 / params-analysis.md §3.1）。取实测 152 mm 推得的真值；与 SIH 混控式（sih.cpp:304-306）逐项同号 |
| `CA_ROTOR0_PY` | `0.0537401` | 推得 | — |
| `CA_ROTOR0_KM` | `0.05` | 实测 | z0 值 = PX4 默认（`docs/assets/z0.params`） |
| `CA_ROTOR1_PX` | `-0.0537401` | 推得 | — |
| `CA_ROTOR1_PY` | `-0.0537401` | 推得 | — |
| `CA_ROTOR1_KM` | `0.05` | 实测 | 同 ROTOR0（`docs/assets/z0.params`） |
| `CA_ROTOR2_PX` | `0.0537401` | 推得 | — |
| `CA_ROTOR2_PY` | `-0.0537401` | 推得 | — |
| `CA_ROTOR2_KM` | `-0.05` | 实测 | z0 值（`docs/assets/z0.params`）；符号 = 桨的转向 |
| `CA_ROTOR3_PX` | `-0.0537401` | 推得 | — |
| `CA_ROTOR3_PY` | `0.0537401` | 推得 | — |
| `CA_ROTOR3_KM` | `-0.05` | 实测 | 同 ROTOR2（`docs/assets/z0.params`） |
| `CA_ROTOR0_CT` | `6.5` | 实测 | z0 值 = PX4 默认（`docs/assets/z0.params`）；★ 绝对值会被伪逆归一化掉，不必等于 SIH_T_MAX |
| `PWM_MAIN_FUNC1` | `101` | 决定 | ★ **刻意偏离 z0**：z0 是 `3-2-1-4`（`103/102/101/104`，`docs/assets/z0.params`）。SIH 的混控式把**输出通道 i 当作第 i 个旋翼**（`sih.cpp:304-306`，它从不读 `CA_ROTOR*`），所以 SITL 里只有 identity 映射能让「分配器以为的旋翼位置」与「SIH 摆的旋翼位置」对上；照抄 z0 那组会让 1↔3 错位 ⟹ 俯仰反号。ESC 接线标签在 SITL 里没有对应物 |
| `PWM_MAIN_FUNC2` | `102` | 决定 | 同上；identity 映射 |
| `PWM_MAIN_FUNC3` | `103` | 决定 | 同上；identity 映射 |
| `PWM_MAIN_FUNC4` | `104` | 决定 | 同上；identity 映射 |
| `SENS_EN_GPSSIM` | `1` | 照抄 | z0 侧**没有对应物**（`docs/assets/z0.params` 里查无此参数）；照抄上游 10040，EKF 要有传感器可吃 |
| `SENS_EN_BAROSIM` | `1` | 照抄 | 同上 |
| `SENS_EN_MAGSIM` | `1` | 照抄 | 同上 |
| `CBRK_SUPPLY_CHK` | `894281` | 实测 | z0 值（`docs/assets/z0.params`，恰与上游 10040 同值）；仿真里没有电池 |
| `CBRK_IO_SAFETY` | `22027` | 实测 | z0 值（`docs/assets/z0.params`，恰与上游 10040 同值）；仿真里没有安全开关 |
| `THR_MDL_FAC` | `0` | 决定 | ★ **故意不等于 z0 的 0.45**：SIH 的被控对象对电机信号是**线性**的，取 0 才能让混控器的推力曲线与被控对象自洽（否则控制→推力多出一段与工作点有关的非线性，且它不在 ticket 12 的四个旋钮里）。要复现 z0 的 0.45 用 `z0-thrcurve` 变体。见 sitl-z0-plant.md §5.2 |
| `MC_ROLLRATE_MAX` | `220` | 实测 | z0 值（`docs/assets/z0.params`；角速率环约束） |
| `MC_PITCHRATE_MAX` | `220` | 实测 | z0 值（同上） |
| `MC_YAWRATE_MAX` | `360` | 实测 | z0 值（`docs/assets/z0.params`；≠ PX4 默认 200） |
| `MPC_TILTMAX_AIR` | `30` | 决定 | ★ **项目决定，不是 z0 现值**（z0 = PX4 默认 45）：30° 需 8.5 N = 电池可用推力的 72 %，45° 需 10.4 N = 88 %（phase2-implementation.md §3.3）。⚠️ **这是 PX4 位置控制器参数，与模型预测控制毫无关系**。见 sitl-z0-plant.md §5.3 / params-changes.md §E6.4 |
| `MC_BAT_SCALE_EN` | `0` | 实测 | z0 值 = 默认（`docs/assets/z0.params`；不要电池电压补偿，仿真里没有电池模型） |
| `MC_AIRMODE` | `0` | 实测 | z0 值 = 默认 |

变体表（4 个旋钮 × ±30 %）在 **§5**。

**参数写在两处，这一分为二是刻意的**（`plant_spec.ROWS` 每行的最后一列决定）：

- **机型文件**（`param set-default`）= **机器身份** —— 构型、几何、输出分配、检查位。
  它决定"这是哪台机"，ticket 12 **不会**动它。
- **覆盖层**（`param set`）= **会被扫/会被改的那部分** —— `SIH_*` 现值、`THR_MDL_FAC`、
  `MC_*RATE_MAX`、`MPC_TILTMAX_AIR`。12 扫的就是这一层。

> ★ **覆盖层是全量而不是差量**（每个变体都无条件 `param set` 全部 21 条）：
> 这样结果与 `rootfs/parameters.bson` 里的**历史残留无关** ——
> 上一次跑 `--mismatch mass+30` 会把这个值**持久化**到 bson 里，只写差量的话
> 下一次跑 `z0` 就会被上一轮的残留压住。**这是 12 能复现的前提。**

---

## 3. 机型与覆盖层是怎么生效的（第一手取证）

两件事都**不需要重新编译**，因为 posix SITL 的 rcS 是按**名字**找文件的，
而 `${R}` 在 posix 下展开为空 ⟹ 找的就是**工作目录**（`build/px4_sitl_default/`）下的相对路径。
源码为证（`~/refs/px4-v1.14.3-build/ROMFS/px4fmu_common/init.d-posix/rcS`）：

```sh
: 14   PATH="$PATH:${R}etc/init.d-posix"          # 搜索路径尾部 = 工作目录下的 etc/init.d-posix
: 56   REQUESTED_AUTOSTART=$(ls "${R}etc/init.d-posix/airframes" \
                              | sed -n 's/^\([0-9][0-9]*\)_'${PX4_SIM_MODEL}'$/\1/p')
: 212  for f in ${R}etc/init.d-posix/airframes/"$(param show -q SYS_AUTOSTART)"_*
: 236  . px4-rc.params                            # ★ 注释原文："user defined params for instances can be in PATH"
```

**两条机制据此建立**：

1. **机型**：`ls … | sed 's/^\([0-9]*\)_sihsim_z0$/\1/p'` ⟹ 文件名以 `<ID>_sihsim_z0` 的形式
   放进 `build/px4_sitl_default/etc/init.d-posix/airframes/` 即可，
   `install_z0_airframe.sh` 就是 `cp` 一个文件进去（并**复算一遍那条 sed** 自检得到 `10046`）。
   ⚠️ 它是 **build 目录里的运行期文件**：**重新编译 / clean 之后必须重跑装机脚本**
   （ROMFS 的 `airframes/CMakeLists.txt` 没列我们这份，`make` 不保证留住）。
2. **覆盖层**：`. px4-rc.params` 是在 **PATH 里按名字**找的，而 PATH 尾部才是
   `${R}etc/init.d-posix` —— 所以只要**把只含 `px4-rc.params` 的临时目录前置到 PATH**，
   就能按名字遮蔽掉 build 目录里那份（实测那份 `px4-rc.params` 全文只有注释，
   是**空操作**；遮蔽它无副作用）。
   临时目录里**只放 `px4-rc.params` 一个文件**，`px4-rc.simulator` / `px4-rc.mavlink`
   仍从 build 目录取 —— 前置目录不会把它们一起遮掉。
   这条规则**有两份实现，是手抄的**：Python 侧 `px4_console.parse_overlay_into_env()`
   （`px4_console.py:212`，被 `verify_z0_plant.py:157` 与 `px4_console.py:251` 调用），
   bash 侧 `run_sitl.sh:94-96`（`mktemp -d` + `cp` + `export PATH`）。
   ⚠️ 改了一条规则要**两处一起改** —— 这里没抽公共层，因为跨语言的共享层得不偿失。

> ★ `${R}` 在 posix 下为空，**证据是行为本身**：rcS 里那两处用的是相对路径写法，
> 而 SITL 实际找到并执行了 `etc/init.d-posix/airframes/10046_sihsim_z0`
> （否则 `PX4_SIM_MODEL=sihsim_z0` 会直接报 "Unknown model"）。

---

## 4. 怎么跑 / 怎么复原

```sh
# 0) 一次性：把机型装进已构建的 SITL（重编之后要重跑）
bash tools/px4-sitl/install_z0_airframe.sh

# 1) 起 z0 同构机（参数一致对照组）
bash tools/px4-sitl/run_sitl.sh --plant z0

# 2) 起某个失配变体
bash tools/px4-sitl/run_sitl.sh --plant z0 --mismatch mass+30

# 3) 从"全默认"起手（清掉持久化参数）—— 怀疑被子进程的残留污染时用
bash tools/px4-sitl/run_sitl.sh --plant z0 --reset-params

# 4) 列全部变体
bash tools/px4-sitl/run_sitl.sh --list-variants

# 5) 不选机型 = 上游 10040（**只能冒烟，不得验收**，脚本会自己警告）
bash tools/px4-sitl/run_sitl.sh
```

**怎么复原**：覆盖层是**每次起机现铺现用**的（`mktemp -d`，退出时 `trap` 清掉），
所以"复原"= **不带 `--mismatch` 再起一次**（用 `z0` 覆盖层）。
剩下的唯一残留是 `build/px4_sitl_default/rootfs/parameters*.bson`
（PX4 自己把参数持久化在那里）—— 但覆盖层是**全量 `param set`**，
所以它压不住我们关心的那 21 条（§2 末尾那条 ★）。要彻底回到出厂默认才需要
`--reset-params` 或 `verify_z0_plant.py` 默认的"起机前删 bson"行为。

---

## 5. 失配开关（ticket 12 的输入）

**四个旋钮，每个只动一个物理量**，名即变体名后缀（`±30 %`，`MISMATCH_PCT = 0.30`）：


### 5.1 为什么"质量"旋钮不动转动惯量

因为要的是**单参数**失配。真实世界里挂载会同时改变质量和惯量，
但那正是 ticket 12 要拿去**区分**的东西 —— 如果两个一起动，扫出来的
"这个控制器对质量敏感还是对惯量敏感"就没有答案了。

### 5.2 `THR_MDL_FAC = 0` —— ★ 一处**故意的**背离

z0 固件上 `THR_MDL_FAC = 0.45`（电机推力曲线 `v = a·u² + (1−a)·u`；
第一手出处 `docs/assets/z0.params:920` = `0.449999988079071045`），
**SITL 侧取 `0`**。理由：SIH 的被控对象对**电机信号**是**线性**的
（`sih.cpp:303` 里 `_T_B = (0,0,−_T_MAX·Σu)`），若混控器再叠一条非线性曲线，
"控制量 → 真实推力"之间就多出一段**与工作点有关**的非线性 ——
那是夹在验证链里的**又一个**未建模误差，且**不在 ticket 12 的四个旋钮里**。

两面的取舍都摆在桌面上：

- 取 `0`：验证链干净；代价是**偏离了 z0 的固件配置**，
  "SITL 里能飞"到"z0 上能飞"之间隔了这条曲线。
- 取 `0.45`：更贴近 z0；代价是把一个**不属于本张交付范围**的失配塞进了被控对象。

**当前取 `0`，但这是可推翻的** —— 想要哪种失配，`z0-thrcurve` 变体就在那里
（它只改 `THR_MDL_FAC`，其余与 `z0` 完全一致），12 扫的时候可以把这两个变体并排比。
**如果用户裁定要"以 z0 固件为准"，改 `plant_spec.ROWS` 里那条的 `0.0` 再 `--emit` 即可。**

### 5.3 `MPC_TILTMAX_AIR = 30` —— 是 PX4 位置控制器参数

⚠️ **与模型预测控制毫无关系**，只是缩写撞脸（Multicopter **P**osition **C**ontroller）。
收它的理由是**电池限制下的推力裕度**：30° 需 8.5 N = 电池可用推力的 72 %，
45° 需 10.4 N = 88 %（`phase2-implementation.md` §3.3）。
**飞机上仍是 45**（`params-changes.md` §E3/E4 未改）—— 两边**故意不一致**，
别把 SITL 的 30 抄回飞机（理由与被推翻的风险见 `params-changes.md` §E6.4）。

---

## 6. 验收：`verify_z0_plant.py`

```sh
python3 tools/px4-sitl/verify_z0_plant.py --variant z0            # 单变体
python3 tools/px4-sitl/verify_z0_plant.py --variants-all         # 全部 10 个（~13 min）
python3 tools/px4-sitl/verify_z0_plant.py --variant z0 --static-only
python3 tools/px4-sitl/verify_z0_plant.py --variant z0 --json /tmp/z0.json   # 机读证据
```

**两类检查**：

- **静态 45 项**：用 `param show <前缀>*` 批量读回（`SIH_ CA_ROTOR PWM_MAIN_FUNC MC_ MPC_ SENS_EN_ CBRK_ THR_MDL_FAC`），
  逐条与 `plant_spec.expected_params(变体)` 比对（绝对容差 `5e-5` / 相对 `2e-3`
  —— PX4 参数是 float32，且 `param show` 只打印 4 位小数）。
  **它同时验证了"覆盖层真的生效"和"机型文件真的被加载"**（后者的 `CA_ROTOR_*` / `PWM_MAIN_FUNC*`
  只写在机型里）。
- **动态 5 项**：真机式地**给电机下指令、量它到底出多少推力**，见 §6.1。

### 6.1 动态配方（以及每一处为什么这么取）

1. **悬停附近取工作点**：`A = m·g / SIH_T_MAX`（= 2.4932，即"4 台电机信号之和"的悬停值），
   令**每台电机**的电机信号 `u = min(A/3.45, 0.97)`。
   关键在 `3u < A < 4u`：**三台电机还压在地面上**（接地时 SIH 把 `_w_B` 清零，
   见 `sih.cpp:385` 的假地面钳位），只有第四台补上时才起飞 ——
   所以中间那几个"三台转、一台停"的不平衡态**翻不了**；
   而 4 台齐了是 `4u ≈ 1.16·A`，只比悬停高 16 %。
   （第一版没用这条，中间态直接把飞机掀到 72 rad/s²。）
2. **`-v` 的语义是"相对推力"，不是电机信号** —— 而且曲线方向**反直觉**（§7 坑 7）。
   `FunctionMotors::updateValues` 喂进去的是**推力**、吐出来的是**反解出的电机信号**
   （`FunctionMotors.hpp:93` 那行 `values[i] = -tmp1 + sqrtf(tmp2 + (control / a))`；注释在 `:81`），
   `actuator_test` 把 `-v` 当推力交给它。所以想得到电机信号 `u`，要下发的是
   `curve(u, a) = a·u² + (1−a)·u`；混控器再反解回 `curve_inv(curve(u)) = u`。
   脚本据此把"**我算的期望电机信号**"与"**读回的电机信号**"对比 ——
   三方（我 / 混控器 / 被控对象）对得上，测量才成立。
3. **读的是电机信号本身**：`actuator_outputs_sim.output[i]` 是**过完曲线之后**的 `u`
   （`pwm_out_sim` 发布），所以"命令到位没有"是**直接读出来的**，不靠推断。
4. **量推力用无噪真值话题**：`vehicle_local_position_groundtruth`
   （`x/y/z` NED + `vx/vy/vz` + `ax/ay/az`）与 `vehicle_attitude_groundtruth`（w-first 四元数）。
   ⚠️ **不要用 `sensor_accel`** —— 它被加了 `noiseGauss3f(0.5, 1.7, 1.4)` 的高斯噪声（`sih.cpp:440`），
   拿来算推力纯属自找误差。
5. **竖直下式**：离地后 `az = [m·g − T·cosθ − KDV·vz] / m`
   ⟹ `T = [m·(g − az) − KDV·vz] / cosθ`，`cosθ = 1 − 2(qx²+qy²)`。
   `az` 与 `θ` 取**同一帧**，只做 `cosθ` 修正（所以**倾斜本身不致命**，落地或倾到 15° 以上才判失败）。
6. **按仿真时间推进，不按墙钟**：SIH 是 lockstep，同一段墙钟里仿真走过的步数是浮动的
   —— 控制台一忙就慢得多，**实测能慢到 ~0.3×**（出处：`verify_z0_plant.py` 模块 docstring 坑 3）。
   所以"等 1 s 再看"是错的 —— 脚本等的是 `timestamp_sample` 前进够 `--settle-sim` 秒。

### 6.2 结果（2026-09-28，`--variants-all` 一次跑完 10 个变体）

**500 项检查全过**（每变体 45 项静态 + 5 项动态 = 50 项，共 10 个变体）。下表是动态那 5 项的实测量。

> ⚠️ **原始机读证据没入库**（一次 `--variants-all` 的 JSON 有 75 KB，且 13 分钟就能重跑）：
> 本节这组数出自 **提交版的复核重跑**，JSON 在 `/tmp/z0_all_v2.json`（500/500）；
> 要重跑用文末配方。同目录另有 `/tmp/z0_all.json` —— 那是**修 §7 坑 7 之前**的旧跑，
> `thrcurve` 那行读数是错的（0.852），**别拿它当证据**。
>
> 表里的 **倾角（`cosθ` 那列）是逐跑抖动的**：同一变体不同次跑能落在 0°–10°，
> 因为它取决于起飞后那几百毫秒里"补发的第四台电机"把机体推歪多少。
> **它不影响结论** —— `T` 的算式里 `cosθ` 是**同时刻**修正过的，且推力偏差始终在 ±0.2 % 内。

| `z0` | 0.7227 | 0.7227 | 8.521 N | 8.528 N | −0.08 % | −0.857 m | 0.9984（3.2°） |
| `z0-mass+30` | 0.9395 | 0.9395 | 11.082 N | 11.086 N | −0.04 % | −0.954 m | 0.9973（4.2°） |
| `z0-mass-30` | 0.5059 | 0.5059 | 5.959 N | 5.969 N | −0.17 % | −0.980 m | 1.0000（0.0°） |
| `z0-thrust+30` | 0.5559 | 0.5559 | 8.515 N | 8.528 N | −0.15 % | −0.855 m | 0.9984（3.2°） |
| `z0-thrust-30` | 0.9700 | 0.9700 | 8.013 N | 8.012 N | +0.01 % | −0.473 m | 0.9994（2.0°） |
| `z0-inertia+30` | 0.7227 | 0.7227 | 8.520 N | 8.528 N | −0.09 % | −1.207 m | 1.0000（0.0°） |
| `z0-inertia-30` | 0.7227 | 0.7227 | 8.521 N | 8.528 N | −0.08 % | −0.864 m | 0.9981（3.5°） |
| `z0-tau+30` | 0.7227 | 0.7227 | 8.521 N | 8.528 N | −0.08 % | −0.822 m | 0.9972（4.3°） |
| `z0-tau-30` | 0.7227 | 0.7227 | 8.521 N | 8.528 N | −0.08 % | −0.929 m | 1.0000（0.0°） |
| `z0-thrcurve` | 0.7227 | **0.6325** | 8.521 N | 8.528 N | −0.08 % | −0.891 m | 1.0000（0.0°） |

`T` 期望 = `4 × SIH_T_MAX × u`，**用各自变体自己的 `SIH_T_MAX`** —— 所以
`thrust±30 %` 两行的 `u` 与基准不同（电机信号得换算过）而 `T` 期望相近，
**这正是"失配真的作用在被控对象上"的证据**。

> ★ **这张表真正证明的事**：`T` 实测与期望在每个变体上都贴着 1（±0.2 % 内），
> 说明"覆盖层改了参数 → 被控对象真的按新参数出力"这条链**逐变体**是通的 ——
> 而不只是"文件写进去了"。这是 ticket 12 能拿这些变体当**已知失配的被控对象**用的前提。
>
> 另外三个副产物：
> - `mass±30` 的 `u` 差得很远（0.94 vs 0.51），说明**质量旋钮真的只改质量**
>   （若它偷偷带动了推力，`u` 不会这么整齐地按 `m·g/T_MAX` 走）。
> - `inertia±30` / `tau±30` **四行**的 `u` 与 `T` 与基准**一致到 0.001 N**（它们只影响姿态/瞬态，
>   而本测量是准静态竖直方向的）—— 这也从反面确认了测量**没有**越界宣称自己量了惯量。
>   ⚠️ 这四行里**倾角仍是逐跑抖的**（0°–4.3°）：抖的是"起飞那几百毫秒被推歪多少"，
>   与惯量无关 —— 恰好也是"本测量没在量惯量"的另一个侧面。
> - `z0-thrcurve` 的 `-v` 是 0.6325 而不是 0.7227，却读回同一个 0.7227 ——
>   **混控器的推力曲线按 `THR_MDL_FAC = 0.45` 反解了一次，且反解结果与我的算式逐位吻合**
>   （§7 坑 7 就是这么发现的）。

---

## 7. 七个坑（都是本张踩出来的）

1. **`param show SIH_` 什么都不返回**。`do_show_print` 要求过滤串**被完整消费**
   （`*` 只能在末尾），`SIH_` 只算前缀 ⟹ 必须写 `param show SIH_*`。
   实测 45 条参数里"44 条读不到"就是这么来的。
2. **`param show` 的输出有**两列**状态**：`x * SIH_MASS [843,1330] : 0.7500`
   —— 第一列"已使用"、第二列"已存盘/未存盘"，值等于默认时第二列是空格（`x   NAME …`）。
   正则必须容忍 `[x+*]?\s+[+* ]?\s*NAME`，否则只能匹配到"值=默认"的那一半。
3. **`listener` 的锚点要 `(?m)`**。`listener actuator_outputs_sim` 的回显形如
   `TOPIC: actuator_outputs_sim`，用 `^TOPIC:` 不去多行模式永远匹配不到
   （表现为"响应明明在日志里，脚本却超时 30 s"）。
4. **pxh 的控制台在起机后一阵子会很忙**（logger / mavlink / EKF 还在刷），
   这时不下发命令、直接等，会等到超时。对策：起机后先空转 `--boot-settle`（默认 3 s），
   并且**每次读都等自己的哨兵响应**，不要用固定 sleep。
5. **同一串命令连发会被丢掉前面的**。四条 `actuator_test` 命令间隔 20 ms 连发时，
   实测第一条**没生效**（读到 `output = [0,1,1,1]`）。
   对策不是"把间隔调大"这种玄学，而是**逐条读回确认**：脚本读 `actuator_outputs_sim`，
   把偏低的通道**补发一次**（补发只会把电机往上拉，且 `ActuatorTest` 进入测试模式时的
   `reset()` 发生在 `set` **之前**，所以补发不会把已到位的通道清掉）。
   ⚠️ **这不是保险，是主路径**：最终那次 `--variants-all` 里，**10 个变体 10 次
   首读全都没到位**（每个变体的 `resent_motors` 都是 `[1,2,3,4]`），
   补发之后第二次读回才全绿。换言之"发完就信"在这条链上一次都不成立。
6. **`sih` 里有一处诱饵，而且是两处对不上的诱饵**。
   `sih.hpp:132` 的成员**初始化式**写的是 `uORB::Subscription _actuator_out_sub{ORB_ID(actuator_outputs)}`，
   `sih.cpp:688` 那段散文注释也跟着说 "subscribes to actuator_outputs"；
   而真正生效的是 `sih.cpp:81` 构造里**重新赋值**的
   `_actuator_out_sub = uORB::Subscription{ORB_ID(actuator_outputs_sim)}` ——
   **构造体覆盖了成员初始化式**，两个 `actuator_outputs` 字样全是过期的。
   `actuator_outputs` 是**不带 `_sim`** 的正常话题，在 SITL 里**根本没有发布者**；
   按那两处去找会一路查到 `pwm_out_sim` 都找不着北。（同类教训见
   `open-source-survey.md` §0。）
   ⚠️ 本张复核时**差点自己踩出第二个坑**：曾打算把 `81` 改成 `80`（凭印象觉得差一行），
   落笔前逐行 grep 才发现 `80` 是空行 —— 结论是**复核过的**：`81` 才是赋值那行。
7. **★ 推力曲线的方向是反的，而 `THR_MDL_FAC = 0` 会把它藏起来。**
   `FunctionMotors::updateValues` 的注释写的是 `rel_thrust = factor*x² + (1-factor)*x`，
   看着像"电机信号 `x` 进、推力出"；而**实际代码是反解**：

   ```cpp
   // src/lib/mixer_module/functions/FunctionMotors.hpp
   values[i] = -tmp1 + sqrtf(tmp2 + (control / a));   // control = 传进来的**推力**
   ```

   也就是说 `-v` 是**推力**，电机信号是算出来的。`THR_MDL_FAC = 0` 时曲线是恒等映射、
   两个方向一模一样，**十个变体里只有 `z0-thrcurve` 能把这个错误暴露出来** ——
   本张第一版脚本正是这么中招的：`z0` 全绿，`z0-thrcurve` 读回 0.852 而不是 0.7227、
   实测推力 10.06 N 而不是 8.53 N。**它不是被控对象的错，是我这条曲线写反了**
   （而那次的"错"反而独立证明了 SIH 对电机信号是严格线性的：`4×2.95×0.852 = 10.054 N` vs 实测 `10.056 N`）。

---

## 8. 未测 / 残留风险（不靠"跑通了"掩盖）

| # | 事项 | 现状 |
|---|---|---|
| U1 | **偏航力矩比没有动态量过** | `SIH_Q_MAX = KM × SIH_T_MAX` 只是**代数上**让它与分配器以为的 `CA_ROTOR_KM` 一致；验收只量了**推力通道**。要量偏航得让四台电机产生纯偏航不平衡（`u0=u1`、`u2=u3` 才不引入滚转/俯仰），**本张没做** |
| U2 | **转动惯量是占位值** | `SIH_IXX/IYY` 取的是**点质量上界** `m·l²`（真值必小于它），`IZZ` 取 2×IXX，`IXY/IXZ/IYZ` 取 0（对称假设）。**都不是辨识值** —— 待 ticket 10 |
| U3 | **`SIH_T_TAU = 0.05` 是 SIH 默认** | z0 的**内环**时间常数未辨识（`phase2-implementation.md` §5 的 P4 才是它）。注意这直接影响 12 的 `tau` 旋钮：**旋钮扫的是"相对这个默认值 ±30 %"** |
| U4 | **`SIH_KDV` / `SIH_KDW` 保持 SIH 默认** | 刻意：给被控对象偷偷加阻尼会让"能飞"变得没意义。真机有没有这么大的气动阻尼**未知** |
| U5 | **PX4 侧只镜像了 §2 表里那几条** | 滤波器、EKF、`MPC_*` 位置环增益、`IMU_*` 等**全部是 SITL 默认值**，没有逐条对齐 z0 的 915 条。`plant_spec.ROWS` 就是"镜像了哪些"的**完整清单** |
| U6 | ~~**几何取自 152 mm 对角线**~~ **已结案（不是"未测"，是"刻意不取"）** | 本张取 152 mm 推出的 `0.0537401`，**刻意不取** `z0.params` 里的 `CA_ROTOR0_PX = 0.15`。理由已在 §2 表中写明：`0.15` 是 PX4 通用 X 型**默认值**，比实测几何大 2.8 倍（`hardware.md:38`、`params-analysis.md` §3.1）—— 这是**上游与 z0 各自的既有矛盾**，不是本张的未测项。⚠️ 但它**顺带说明**：飞机上那套分配器几何与 z0 真几何**本来就不一致**，本张只是**没有复制这个不一致** |
| U7 | **分配器 → 力矩通道的符号没动态验证过** | `actuator_test` **绕过分配器**（直接把电机信号塞进 `actuator_outputs_sim`），所以"滚转/俯仰/偏航三个通道**符号**对不对"这条路本张**量不到** —— 量的是"覆盖层改参数 → 被控对象按新参数出力"。要量符号得喂 `vehicle_torque_setpoint`，那是 **ticket 05 / 11 的仪表** |
| U8 | **`PWM_MAIN_FUNC` 刻意偏离 z0 的 `3-2-1-4`** | 见 §2 表与 `plant_spec._FUNC_WHY`：SIH 把**输出通道 i 当作第 i 个旋翼**，SITL 里只有 identity 映射自洽；照抄 z0 那组会让 1↔3 错位 ⟹ 俯仰反号。**代价**：SITL 里验不了 z0 的 ESC 接线顺序（那个顺序要等上真机） |

---

## 附：给 ticket 12 的复现配方

```sh
python3 tools/px4-sitl/verify_z0_plant.py --variants-all --json /tmp/z0_all.json
```

或者只起机（不起 ROS；每条都会**前台阻塞**，`Ctrl-C` 退出）：

```sh
for v in z0 z0-mass+30 z0-mass-30 z0-thrust+30 z0-thrust-30 \
         z0-inertia+30 z0-inertia-30 z0-tau+30 z0-tau-30 z0-thrcurve; do
  echo "== $v"; bash tools/px4-sitl/run_sitl.sh --overlay \
      "tools/px4-sitl/z0-plant/params/$v.param"
done
```

**参数值、切换方式、复原方式三件事的答案**：
值见 §2（生成式，不会漂）；切换 = `--mismatch <旋钮>±30` 或 `--overlay <文件>`；
复原 = 不带 `--mismatch` 再起一次（§4 末）。
