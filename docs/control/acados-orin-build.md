# acados 在 z0 机载电脑上的构建（Phase 2 §S2）

> 本文件是 ticket [`02-acados-orin-build`](../../.scratch/phase2/issues/02-acados-orin-build.md) 的交付物：
> **安装步骤 + 三个配置项的实测复核 + 报错原文**。
> ⚠️ 它覆盖的是**里程碑 0 的一半**：里程碑 0（环境就绪）= **S1**（SITL + `px4_msgs`）+ **S2**（本文件）
> （对照 [`phase2-implementation.md`](phase2-implementation.md) §7）。**S1 那一半不属于本票。**
> 上级：[`phase2-implementation.md`](phase2-implementation.md) §S2（施工图）、[`mpc-roadmap.md`](mpc-roadmap.md) §5（资料汇总）。
> 脚本：[`tools/acados-orin/`](../../tools/acados-orin/)（可复现形式）。
>
> ⚠️ 本文件里标「第一手」的都是**在本机跑出来或读源码读出来的**，路径/行号/命令可复核。
> 与旧文档冲突的地方**以本文件为准**，冲突处已逐条标注（见 §3、§4）。

---

## 1. 本机实况（这一份读数是从哪来的）

| 项 | 值 | 证据 |
|---|---|---|
| 源码 | `~/refs/acados` @ **`00947ddc394b30c956d9a7794d0bbcbc44c7c050`** = `v0.6.0-41-g00947ddc3`（2026-09-25） | `git -C ~/refs/acados describe --tags` |
| 安装目录 | **`~/acados`**（`ACADOS_SOURCE_DIR` 指向它） | `ls ~/acados/{bin,lib,include,cmake}` |
| 构建目录 | `~/acados-build`（**out-of-source**，产物不落进 `~/refs/`） | — |
| BLASFEO / HPIPM / OpenMP | **`ARMV8A_ARM_CORTEX_A76` / `GENERIC` / `OFF`** | `~/acados-build/CMakeCache.txt` |
| t_renderer | **v0.2.1**（`linux-arm64`，8.5 MB）sha256 `17db270e2f21f25bfb994d0eaea9e0481f3c2dd081030f959c254b8862df4453` | `~/acados/bin/t_renderer --version` |
| Python | `/usr/bin/python3` = **3.10.12**；`acados_template` 以 `-e` 装在 user site（源码指向 `~/refs/acados/interfaces/acados_template`） | `python3 -c "import acados_template; print(...)"` |
| 依赖版本 | casadi **3.8.1**、scipy **1.15.3**、numpy **2.2.6**、matplotlib 3.10.9（全在 `~/.local`） | `pip3 list --user` |
| CPU/时钟 | 6 核（`0–3` 实测 ~1.42 GHz、`4–5` ~1.98 GHz），governor = `schedutil`，**未跑 `jetson_clocks`** | `/sys/devices/system/cpu/*/cpufreq` |
| 验收 | `AcadosOcpSolver` 建成；`solve()` → **0**；最坏纯求解 **4.39 ms**（x500 例子，n=1000） | §3 |
| ⚠️ **读数前提** | 上面所有耗时都是**在空载机器上**量的。**这台机器随时可能有别人在编译**（本次取证期间就撞上过一轮 PX4 SITL 构建），负载一上来同样的代码会量出 **max 23~27 ms**、脚本甚至报出「100 Hz 不够」的**假**结论 —— 见 §6.3 | `uptime` / `/proc/loadavg` |

**为什么源码在 `~/refs/` 而安装到 `~/acados/`**：`~/refs/` 按 Q18 是「仓库外 + 持久 + 钉 commit 的**只读副本**」，
构建产物不该落在里面。`ACADOS_SOURCE_DIR` 必须指向**安装目录**（要有 `bin/ lib/ include/ cmake/`），
所以两者分开。⚠️ 但上游在 **configure 时就会往源码树里写** `lib/link_libs.json` 与 `lib/git_commit_hash`
（`CMakeLists.txt:368/371`，见 §5 坑 1）——这两个文件在 acados 自己的 `.gitignore:108`（`lib/`）里，
所以 `git -C ~/refs/acados status` 仍然是干净的，但**「只读」这个词对这两个文件不成立**。

---

## 2. 装法（可复现的三步）

```bash
# ① 源码（钉到本次取证的 commit）
git clone --recursive https://github.com/acados/acados.git ~/refs/acados
git -C ~/refs/acados checkout 00947ddc394b30c956d9a7794d0bbcbc44c7c050

# ② 构建 + 安装到 ~/acados（脚本里含上游漏装的 link_libs.json，见 §5 坑 1）
bash tools/acados-orin/build_acados.sh

# ③ t_renderer（缺它的三种落点见 §4）
bash tools/acados-orin/install_t_renderer.sh

# ④ Python 侧 ⚠️ pypi.org 在这台机器上不可达，必须带镜像（见 §5 坑 2）
pip3 install --user -i https://pypi.tuna.tsinghua.edu.cn/simple \
     "scipy>=1.13" "matplotlib>=3.8" "casadi<=3.8.1" cython Deprecated setuptools_scm
pip3 install --user -i https://pypi.tuna.tsinghua.edu.cn/simple \
     -e ~/refs/acados/interfaces/acados_template

# ⑤ 每次开新终端都要有这两行（写进 ~/.bashrc 也行）
export ACADOS_SOURCE_DIR=$HOME/acados
export LD_LIBRARY_PATH=$ACADOS_SOURCE_DIR/lib:$LD_LIBRARY_PATH

# ⑥ 验收
python3 tools/acados-orin/check_x500_ocp.py --runs 1000
```

> ⚠️ `casadi<=3.8.1` 是 `acados_template/setup.py` 写的上限（不是我们加的）。
> ⚠️ 本机 numpy 2.2.6 与 apt 的 scipy 1.8.0 **互不兼容**（numpy 2 需要 scipy ≥ 1.13），
> 所以 scipy/matplotlib 装到 `~/.local` 覆盖 apt 版。仓库里的 `src/`、`run/`、`tools/` **不 import scipy/matplotlib**，
> 这次升级不影响 Phase 1 那条链（已 grep 核实）。

---

## 3. ★ 三个配置项：逐条实测复核（本张的主要修正）

施工图里那三条「任一漏掉就编译失败或性能掉一个量级」。**实测下来三条都要改口径**：

| # | 文档原话 | 本机实测 | 结论 |
|---|---|---|---|
| 1 | `ACADOS_WITH_OPENMP=OFF` —— 「Orin 上 OpenMP 与 acados 的**已知冲突**」 | **没能复现冲突**：开 OMP 也编译通过、`solve()` 返回 0。绑单核时开/关**几乎无差**（2.89 vs 2.83 ms）；**不绑核时开 OMP 的尾部明显变差**（p99 5.10 / max **6.72** ms vs 4.08 / 4.39 ms） | **仍然保持 OFF**，但**理由要改写**：不是「冲突」，是「**不绑核时 OMP 线程跨核/跨簇调度把尾部抖动放大**」。⚠️ 另：0.6.0 的**默认值本来就是 OFF**（`CMakeLists.txt:78`），显式写只是防上游改默认 |
| 2 | BLASFEO = `ARMV8A_ARM_CORTEX_A76` —— 「用 `X64_AUTOMATIC` 或 `GENERIC` 会**掉一个量级**」 | 同一核心交替测：A76 median **2.83 ms** vs GENERIC **3.43 ms** ⟹ **只慢 ~21%，不是 10×**。⚠️ 且 `X64_AUTOMATIC` 在 aarch64 上**不是可选值**——上游 aarch64 的默认是 `ARMV8A_ARM_CORTEX_A57`（`CMakeLists.txt:62-66`），`X64_AUTOMATIC` 是非 aarch64 分支的默认 | **仍然选 A76**（便宜就占），但**收益量级要改写**；★ 顺带一个更值钱的实测：**绑核**比换 BLASFEO 目标影响更大（median 3.96 → 2.83 ms，**快 ~29%**；max 列 4.39 → 2.96 ms，**快 ~33%**） |
| 3 | `t_renderer` 必须装 —— 「缺它会报 `EOFError`，而这个报错不指向根因」 | **在 v0.6.0 上不成立**（三种落点见 §4） | 必须装这件事不变；**症状与「真因」都要改写** —— 见 §4 |

**一句话**：三条的**动作都不变**（OFF / A76 / 装 tera），但**三条的理由原来都写错了量级或机制**，
照原话去理解会在排查时找错方向。

---

## 4. `t_renderer`：三种落点 + 「`EOFError`」的真出处

### 4.1 本机实测的三种落点（第一手）

| 情况 | 实测结果 | 报错/表现（原文摘） |
|---|---|---|
| **a. 缺失 + 有网** | **不报错**。acados 自己从 GitHub 下（8.5 MB，代码生成那一步 15.3 s vs 正常 3.6 s），随后照常编译、`solve()` → 0 | `Setting up tera renderer` / `Downloading …/t_renderer-v0.2.1-linux-arm64` / `Successfully downloaded t_renderer.` |
| **b. 缺失 + 无网** | `RuntimeError`，**信息完整、直指根因**（还附手动安装步骤） | `RuntimeError: Tera renderer executable not found, while looking in path: /home/chen/acados/bin/t_renderer … Automatic installation failed with: <urlopen error [Errno 111] Connection refused>` |
| **c. 在、但渲染不出东西** | ★ **完全不报错**：打印「`Templated solver code generated in 36.074 ms`」（正常 3.6 s ⟹ 一眼能看出什么都没渲染），失败**两层之后**才炸 | `OSError: …/mpc_codegen/libacados_ocp_solver_ocp_multirotor_rate_model_….so: cannot open shared object file: No such file or directory` |

**c 就是「报错不指向根因」的真身。** 机制（第一手读源码）：
`interfaces/acados_template/acados_template/utils.py:396-397` 是用 **`os.system` 拼字符串**跑 tera 的，
**只看退出码**；退出码为 0 就当作渲染成功，**没有产物校验**。所以「渲染出空文件」这种情况一路静默到 `ctypes` 加载 `.so` 时才炸。

> 实测用的桩：`bin/t_renderer` 换成「`--version` 答 0.2.1、其余什么都不做、exit 0」的 shell 脚本。
> 这也解释了 b 为什么**不**走 render 那条路：`get_tera()`（`utils.py:388`）在渲染之前就把缺件挡住了。

### 4.2 ★ 「`EOFError`」的真出处（第一手：acados 自己的 git 历史）

**旧版 acados 在缺 t_renderer 时会交互式提问**，非交互运行读到 EOF，于是抛出 Python 内建的 `EOFError`：

```
EOFError: EOF when reading a line
```

- 证据：commit **`a26c7d0ce`**（2026-08-01，*"Update tera renderer to v0.2.1, in Python always attempt automatic installation. (#1892)"*）
  的 diff 里**删掉**了这两行 —— `msg += 'Do you wish to set up Tera renderer automatically?\n'` 与 `if input(msg) != 'y':`。
- `git tag --contains a26c7d0ce` → **`v0.6.0`** ⟹ **v0.6.0 及以后不会再有 `EOFError`**（v0.5.x 及以前才会）。
- ⚠️ 补一条：`EOFError` 这个字符串**在 acados 的全部 git 历史里都不存在**（`git log -S EOFError` 为空）——
  它从来不是 acados 抛的，是 Python 内建的，所以**在源码里搜它永远搜不到**。

> ☠️ **要改写的旧说法**：`phase2-implementation.md` §S2 第 3 条与 `mpc-roadmap.md` §5 第 3 条写的
> 「缺它会在生成代码时报 `EOFError`」——**在 v0.5.x 及以前成立，在我们用的 v0.6.0 上不成立**。
> **该记的不是"会报 EOFError"，而是"三种落点里有一种（c）是静默的"**：真正坑人的不是异常类型，是**没有产物校验**。

---

## 5. 坑（本张新发现两条）

### 坑 1（新）★ out-of-source 构建时 `lib/link_libs.json` 缺失 → `FileNotFoundError`

```
FileNotFoundError: [Errno 2] No such file or directory: '/home/chen/acados/lib/link_libs.json'
  acados_template/acados_code_gen_options.py:362 → make_consistent()
  ← AcadosOcpSolver.__init__ 第一行就调它 ⟹ 连求解器都建不起来
```

- **根因**：`link_libs.json` 是 `CMakeLists.txt:371-381` 用 `file(WRITE lib/link_libs.json …)`
  在 **configure 时写进源码树**的；而**全仓没有任何 `install(...)` 规则安装它**（已 grep 全仓 `CMakeLists.txt`/`*.cmake` 核实）。
  同一处还写 `lib/git_commit_hash`（缺了只是 warning）。
- **为什么官方流程没暴露它**：上游文档让在 `$ACADOS_SOURCE_DIR/build` 里构建、安装到 `$ACADOS_SOURCE_DIR`，
  **构建目录 = 安装目录 = 源码目录**，于是这两个文件"恰好"在正确的位置。**只要 out-of-source 就必踩。**
- **修法**：`tools/acados-orin/build_acados.sh` 在构建后把这两个文件 `install` 到 `$ACADOS_SOURCE_DIR/lib/`。
  ⚠️ **每次构建后都要搬**——`link_libs.json` 的内容随开关变（OpenMP 那一行，实测三个变体各不同）。

### 坑 2（新）★ `pypi.org` 在这台机器上不可达

`curl https://pypi.org/simple/casadi/` 15 s 超时（exit 124），而 `files.pythonhosted.org` 正常。
**症状**：`pip3 install` 卡在收集索引这一步**十分钟一声不响**（本项目先前那次就是这么卡住的）。
**修法**：加 `-i https://pypi.tuna.tsinghua.edu.cn/simple`（实测 1.3 s 返回 200；`mirrors.aliyun.com` 亦可）。

### 坑 3 上游 install 只装 `*.h` 与 lib/cmake 配置，`bin/` 不由构建产生

`acados/CMakeLists.txt:179-205`：`install(TARGETS acados …)` + `install(DIRECTORY … PATTERN "*.h")` + cmake 配置。
⟹ `bin/t_renderer` 永远不会被构建出来，必须按 §2 第 ③ 步单独装，否则落进 §4 的 a/b/c 之一。

### 坑 4 `ACADOS_SOURCE_DIR` 指的是**安装目录**，不是源码目录

它要能提供 `bin/t_renderer`、`lib/libacados.so`、`lib/link_libs.json`、`include/`、`cmake/`（`utils.py:80-102`）。
指到 `~/refs/acados`（源码）会因为缺 `link_libs.json` 直接报坑 1 的错。

---

## 6. ★ 实测数（**上游参考配置**的量级；z0 自己那一份的 P3 仍未结）

> ⚠️ **不要把这些数当成 `phase2-implementation.md` §5 的 P3 已回填。**
> §5 的 P3（`phase2-implementation.md:571`）问的是「**z0 自己的 OCP** 在 SITL 跑 1000 次取 max」，卡 D6；
> 本节的数来自**上游参考实现的 x500 占位配置**（`mass=1`、`hover_thrust=0.73`），**不是 z0 的 OCP**
> （z0 版在 [ticket 04](../../.scratch/phase2/issues/04-offline-ocp.md)，参数定稿在
> [ticket 10](../../.scratch/phase2/issues/10-param-measure-backfill.md)）。
> 本节能提供的是**量级与瓶颈归因**；**拿它去填 P3 就是错的口径**。§5 的 P3 维持「待实测」。

**被求解的问题**：参考实现自带的例子 —— `MultirotorRateModel()`（x500 占位配置：`mass = 1 kg`、`hover_thrust = 0.73`、
`max_rate = 0.5` **占位值**），`N = 50`，`Tf = 1 s`，ERK 4/3，`SQP_RTI` +
`PARTIAL_CONDENSING_HPIPM`，每周期**一次** SQP 迭代。⚠️ **这不是 z0 的最终 OCP**（z0 版在 [ticket 04](../../.scratch/phase2/issues/04-offline-ocp.md)）。

### 6.1 表 1 —— 纯求解耗时（`ocp_solver.solve()`，各 1000 次）

| 变体 | 绑核 | median | p99 | **max** |
|---|---|---|---|---|
| **A76 / OpenMP OFF**（采用） | 不绑 | 3.96 | 4.08 | **4.39 ms** |
| **A76 / OpenMP OFF**（采用） | 核 4 | 2.83 | 2.87 | **2.96 ms** |
| A76 / OpenMP **ON** | 不绑 | 2.92 | 5.10 | **6.72 ms** |
| A76 / OpenMP **ON** | 核 4 | 2.89 | 2.91 | **2.93 ms** |
| BLASFEO `GENERIC` | 核 4 | 3.43 | 3.48 | **3.80 ms** |

**每个数是"一次空载运行"的代表值，不是唯一值。重复测的散布（同变体、同命令）**：

| 变体 | median 的范围 | max 的范围 |
|---|---|---|
| A76 / OFF / **核 4** | **2.82 ~ 2.87 ms**（4 次：2.83 / 2.82 / 2.84 / 2.87） | **2.96 ~ 3.10 ms** |
| A76 / OFF / 不绑核 | ~3.9 ~ 4.0 ms | ~4.4 ms（**最吃负载，见 §6.3**） |

- 全部 ≤ 4.4 ms ⟹ **100 Hz（10 ms 预算）在纯求解这一项上是够的**。但见下表 ——
- ★ **不绑核要吃掉 ~30%**：median 3.96 → 2.83 ms（**−29%**）、max 4.39 → 2.96 ms（**−33%**）。
  不绑核时会落到 1.42 GHz 的那一簇上。⚠️ **百分比要连着列头一起说** ——
  同一组数按 median 算是 29%、按 max 算是 33%，单说一个数就是说不清。
- ★ **OpenMP 的代价只在尾部**：不绑核时 median 甚至更低（多核并行），但 max 从 4.39 涨到 6.72 ms。**实时环看的是尾部**。
- ★ 这些数是 **governor = `schedutil`、未 `jetson_clocks`** 的「交付状态」；锁频到最高会更快，但**实机飞时也是这个状态**，所以按这个数定档。
- ⚠️ **别追第三位小数**：median 的散布有 ~0.05 ms（见上表）。**要盯的是 max 和 p99。**
- ★ **绑核那一行对负载是有抗性的**，不绑核那一行没有：本次复核期间，另一会话在编译
  PX4 SITL / `px4_msgs`（load 5~8），**绑核仍稳定拿到 median 2.87 / max 3.10 ms**，
  而**不绑核同一条命令拿到 median 3.06 ~ 3.20 / max 23.4 ~ 27.4 ms**（§6.3，3 次复测）。⟹
  **要复核这份文档的数，请一律用 `--cpu 4`**；不绑核的数只在空载窗口里可比。

### 6.2 表 2 —— ★★ 求解**不是**瓶颈：参考实现的每周期开销是它的 4 倍

| 量 | median | max |
|---|---|---|
| `ocp_solver.solve()`（真正解 QP/NLP） | **2.81 ms** | 3.00 ms |
| `model.get_acados_model()` **单次**（重建 CasADi 表达式） | **3.10 ms** | 8.39 ms |
| `controller.solve(x0)` **整条**（参考实现的一个控制周期） | **10.70 ms** | 12.33 ms |

**归因**（第一手：`px4_mpc/controllers/multirotor_rate_mpc.py:152-153`）：参考实现**每次 `solve()` 都重建 2 次 CasADi 模型**
（只为取 `nx`/`nu` 两个整数），另外逐帧 `ocp_solver.get(i, "x"/"u")` 读 51×2 个 numpy 数组也要 ~1.7 ms。

> ⚠️ **给 [04](../../.scratch/phase2/issues/04-offline-ocp.md) / [06](../../.scratch/phase2/issues/06-mpc-node-fake-px4-fixture.md) 的硬结论**：
> **照抄参考实现的控制周期是 10.7 ms ⟹ 100 Hz 预算在"还没开始算"时就花光了。**
> 移植时必须把模型构建**提出循环**（挪到构造函数），`nx`/`nu` 从 `model.x.size()` 直接取；
> 逐帧读取 `simX/simU` 也应改成"只在需要时读"。这不是优化项，是能不能跑 100 Hz 的前提。

### 6.3 ★ 读数的前提：**机器必须是空的**（与绑核并列的第二个前提）

**这台机器不是独占的。** 2026-09-28 复测时就撞上**另一个会话正在编译 PX4 SITL**
（`~/refs/px4-v1.14.3-build/…`，4+ 个 `cc1plus` + `ninja`，**不是本票起的**），6 核上 load 5.8~8：

| 同一条命令（`--runs 1000`） | median | p99 | **max** | 脚本自己的判定行 |
|---|---|---|---|---|
| **空载**（采用的那份读数） | 3.96 | 4.08 | **4.39 ms** | 100 Hz 预算**够** |
| 有编译负载（未绑核） | 3.06 ~ 3.20 | 16.6 ~ 19.5 | **23.4 ~ 27.4 ms** | 100 Hz 预算**不够** ❌ |

（负载下那一行是 **3 次独立复测**（load 5.1 / 5.8 / 6+）的区间，不是一次 —— median 几乎没变、
**变的全是尾部**，这正是"多核并行把 median 拉平、把 max 拉爆"的另一个例子。）

> ★ **差 5 倍以上，而且结论整个翻过来了。** 所以：
> 1. **量之前先看 `uptime`**。load 高于核数的一半，数就不可信。
> 2. **`check_x500_ocp.py` 自己会报 load**，并在有负载时把判定行标成「**可能失真**」——
>    看到那个标记就别拿这个数去定 D6。
> 3. 这不只是"仪器脏了"：**它是 D6 的真实风险**。真机飞的时候，如果机上还有别的进程在吃 CPU，
>    尾部就会变成这个量级。**「100 Hz 够不够」的答案依赖「机上还有没有别的东西在跑」** ——
>    这一条要连同绑核一起写进 D6 的前提（交接 [04](../../.scratch/phase2/issues/04-offline-ocp.md) / [09](../../.scratch/phase2/issues/09-sitl-tier-a-closed-loop.md)）。
> 4. 绑核（`--cpu 4`）**对负载的抗性明显更好**：同样的负载下，绑核那份仍然 median 2.82 / max 3.07 ms
>    （独立复测），因为 4 号核是 1.98 GHz 那一簇、且躲开了编译进程。

### 6.4 表 3 —— 版本错配的告警（不阻塞，但要知道）

`AcadosOcpDims.N has been migrated to AcadosOcpOptions.N_horizon` ——
参考实现（2026-06）写 `ocp.dims.N = 50`，acados 0.6.0 已把它迁到 `ocp.solver_options.N_horizon`（自动迁移 + 告警，功能正常）。
**本张只记录，不改参考实现**（改动属于 ticket 04）。

---

## 7. 复现

```bash
export ACADOS_SOURCE_DIR=$HOME/acados
export LD_LIBRARY_PATH=$ACADOS_SOURCE_DIR/lib:$LD_LIBRARY_PATH

python3 tools/acados-orin/check_x500_ocp.py --runs 1000 --node-runs 20          # 采纳的那一份
python3 tools/acados-orin/check_x500_ocp.py --runs 1000 --cpu 4                 # 绑核
# 对照变体（先构建到别的安装目录）：
INSTALL_DIR=$HOME/acados-generic BLASFEO_TARGET=GENERIC bash tools/acados-orin/build_acados.sh
ACADOS_SOURCE_DIR=$HOME/acados-generic LD_LIBRARY_PATH=$HOME/acados-generic/lib:$LD_LIBRARY_PATH \
  python3 tools/acados-orin/check_x500_ocp.py --runs 1000 --cpu 4 \
  --scratch $HOME/acados-ocp-check-generic-pin
```

脚本做的事：校验 `~/refs/px4-mpc` 在钉住的 `3e541afb…` 上 → `git archive` 出干净源码到 scratch（**不往只读副本里写生成代码**）
→ 断言这确实是 `mass=1 / hover_thrust=0.73` 的 x500 配置 → 建 `AcadosOcpSolver` → `solve()` 必须为 0 → 报**最坏**耗时。

---

## 8. 遗留 / 交接

| # | 事项 | 交给谁 |
|---|---|---|
| 1 | ★ ★ **「缺 Tera 报 `EOFError`」这句话还在 5 个文件里原样重复**（`hardware.md:278`、`code-inventory.md:239`、`PROJECT.md:408`、`PROJECT.md:453`、`open-source-survey.md:61`）。本张只改了 `phase2-implementation.md` §S2 与 `mpc-roadmap.md` §5（acados 部署的**直接出处**）—— 那几处**本张不越界改**。⚠️ 另：经 `grep` 核对，「`GENERIC` 掉一个量级」与「OpenMP 冲突」这两句**只存在于 `phase2-implementation.md:444` 一处**，已随本张改掉，**别处没有**（`hardware.md:276` 只写「没有 A78 目标 → 用 A76」，本来就是对的） | **已另开票点名**：[ticket 13](../../.scratch/phase2/issues/13-stale-acados-claims-sweep.md)（`done` 不允许靠"留给以后"掩盖未决事项，见 `docs/agents/triage-labels.md`）。⚠️ 其中 `hardware.md:278` 是「acados 要点」那一节，**是后来者最先看到的地方**，优先改 |
| 2 | z0 自己的 OCP（`mass = 0.75`、`T_max` 按电池、角速率上限换实测值）的最坏耗时 —— §5 的 P3 仍是「待实测」 | ticket [04](../../.scratch/phase2/issues/04-offline-ocp.md) |
| 3 | **频率定档（D6）必须带两个前提**：①**绑核**（不绑核 median 差 29%、max 差 33%）；②**机上负载**（有编译负载时 max 从 4.39 → 23.4~27.4 ms，差 5 倍以上、结论会反过来，见 §6.3） | ticket 04 / [09](../../.scratch/phase2/issues/09-sitl-tier-a-closed-loop.md) |
| 4 | `~/acados-omp`、`~/acados-generic` 两个对照安装目录是本张的取证产物，**不是交付物**；`~/acados` 才是 | — |
| 5 | 本机 `acados` 的 `git_commit_hash` = `00947ddc3`（第一手：`cat ~/acados/lib/git_commit_hash`）—— 与 `~/refs/acados` 的 HEAD 一致 | — |
