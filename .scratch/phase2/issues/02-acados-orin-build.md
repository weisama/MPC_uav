# 02: acados 在 Orin NX 上编译通过

Status: done
Type: task

**What to build:** 一个在 z0 的机载电脑上**真跑得起来**的 acados —— 三个"任一漏掉就编译失败或性能掉一个量级"的配置项全部到位，且**解得动**（不只是装上了）。

**Blocked by:** 无（可立即开工）

- [x] `ACADOS_WITH_OPENMP=OFF`（~~Orin 上 OpenMP 与 acados 的已知冲突~~ —— ☠️ **冲突未能复现**，见 Comments）
- [x] BLASFEO target = `ARMV8A_ARM_CORTEX_A76` —— Orin NX 实为 Cortex-**A78AE**，A76 是可用集中最接近的；~~用 `X64_AUTOMATIC` 或 `GENERIC` 会**掉一个量级**~~ ☠️ **实测只慢 ~21%**，见 Comments
- [x] `t_renderer`（Tera）可用 —— ~~缺它会在生成代码时报 `EOFError`~~ ☠️ **v0.6.0 上不成立**，真身见 Comments；**这个报错不指向根因**（本张的价值之一就是把它的真因记下来）
- [x] 用参考实现自带的例子（质量 1 kg / 悬停油门 0.73 的 x500 配置）能创建 `AcadosOcpSolver` 且 `solve()` 返回 0
- [x] 记录本机实测求解耗时的**量级**（不是验收线本身；验收线在 [04](04-offline-ocp.md) 与 [09](09-sitl-tier-a-closed-loop.md)）
- [x] 安装步骤与三个坑的报错原文写入文档，使后来者不必重踩

---

**无阻塞，且与 [01](01-sitl-px4-msgs-baseline.md) 完全并行** —— 两者一个是仿真/中间件侧，一个是求解器侧，不共享任何前置。

> ⚠️ **上方三条勾选项的正文里有三处被实测推翻**（原文已用 ☠️ 标出，**未删**，照仓库纪律保留原话）。
> 逐条实测复核、报错原文、复现命令、实测数据全部落在
> [`docs/control/acados-orin-build.md`](../../../docs/control/acados-orin-build.md)。
> **动作一个都没变**（OFF / A76 / 装 tera），变的是**理由的口径** —— 照旧话去理解会在排查时找错方向。

## Comments

- **2026-09-28（agent，立项→落地→取证，一次做完）**：`Status: ready-for-agent` → **`done`**。
  在 z0 机载电脑（Jetson Orin NX，aarch64）上 out-of-source 构建 acados
  **`00947ddc394b30c956d9a7794d0bbcbc44c7c050`**（= `v0.6.0-41-g00947ddc3`），装到 `~/acados`。
  三条验收判据全过：`AcadosOcpSolver` 建成、`solve()` → **0**、
  最坏纯求解 **4.39 ms**（不绑核）/ **2.96 ms**（绑核）。

  **产物**

  | 文件 | 作用 |
  |---|---|
  | [`docs/control/acados-orin-build.md`](../../../docs/control/acados-orin-build.md) | ★ 交付文档：安装步骤、三条配置项的逐条实测复核、坑与报错原文、测量表、复现命令 |
  | [`tools/acados-orin/build_acados.sh`](../../../tools/acados-orin/build_acados.sh) | 构建（钉 commit + 三开关 + 搬上游漏装的两个文件） |
  | [`tools/acados-orin/install_t_renderer.sh`](../../../tools/acados-orin/install_t_renderer.sh) | 装 t_renderer v0.2.1，**核对 sha256** `17db270e…` |
  | [`tools/acados-orin/check_x500_ocp.py`](../../../tools/acados-orin/check_x500_ocp.py) | 验收脚本：建求解器、断言配置、报**最坏**耗时 |

  **同步改的文档指针**：[`phase2-implementation.md`](../../../docs/control/phase2-implementation.md) §S2
  （三条正文全部改写 + 补上验收已完成）、[`mpc-roadmap.md`](../../../docs/control/mpc-roadmap.md) §5
  （表内 ☠️ 改口径，**新增第 8/9 两条坑**）、[`refs-manifest.md`](../../../docs/reference/refs-manifest.md) §1
  （新增 `acados/` 一行）。

- **2026-09-28（agent，★ 三条继承说法被实测推翻 —— 本张的主要产出）**
  「性能掉一个量级」这类话**不实测就抄下去**，等于把上一轮的口头结论当事实往下传。逐条：

  | 原话 | 实测 | 处置 |
  |---|---|---|
  | OpenMP 与 acados 在 Orin 上**已知冲突** | **没能复现**：开 OMP 也编译通过、`solve()` → 0；绑单核时开/关几乎无差（2.89 vs 2.84 ms）。**不绑核时开 OMP 只是尾部更差**（max 4.39 → **6.72** ms，median 反而更低） | 仍取 **OFF**，但理由改成「**不绑核时跨簇调度放大尾部抖动**」；且注明 0.6.0 默认本来就是 OFF（`CMakeLists.txt:78`） |
  | 用 `GENERIC` 会**掉一个量级** | 同一核心交替测：median **2.84 → 3.43 ms**，即 **~21%**，不是 10× | 仍选 **A76**（便宜就占），但收益量级改写。⚠️ 另：`X64_AUTOMATIC` 在 aarch64 上**不是可选值**，上游 aarch64 默认是 **A57**（`CMakeLists.txt:62-66`） |
  | 缺 t_renderer 报 **`EOFError`** | v0.6.0 上**不成立**。那是 v0.5.x 及以前**交互式 `input(...)`** 在非交互下读到的 Python 内建异常；`git log -S EOFError` **在全史里为空**，所以照这句话去搜源码**永远搜不到**。commit `a26c7d0ce`（首发 **v0.6.0**）删掉了该提问 | ★ **记的不是"报什么异常"，而是三种落点**：缺失+有网→自己下载（只慢）；缺失+无网→`RuntimeError` 信息完整**直指根因**；**在但渲染不出东西→完全不报错**，静默两层后炸成 `OSError: …libacados_ocp_solver_….so: cannot open shared object file` |

  **静默那一种的机制**（第一手读源码）：`acados_template/utils.py:397` 用 **`os.system` 拼字符串**调 tera，
  **只查退出码、不校验产物** ⟹ 渲染出空文件也算成功。**这才是「报错不指向根因」的真身。**

- **2026-09-28（agent，★ 比配置项更值钱的两条实测）**

  1. **绑核比换 BLASFEO 目标影响大**：不绑核 3.96 ms → 绑核 2.83 ms（**差 40%**），
     因为不绑核时会落到 1.42 GHz 的那一簇上（6 核两簇，`schedutil`，**未跑 `jetson_clocks`**）。
     ⟹ **D6 频率定档必须写明这个前提**，否则数字能差 40%（交接 [04](04-offline-ocp.md)/[09](09-sitl-tier-a-closed-loop.md)）。
  2. ★★ **参考实现照抄下来跑不到 100 Hz，而瓶颈不是求解**。同一进程内实测：

     | 量 | median |
     |---|---|
     | `ocp_solver.solve()`（真正解 QP/NLP） | **2.81 ms** |
     | `controller.solve(x0)`（参考实现的**一个控制周期**） | **10.70 ms** |

     归因（`px4_mpc/controllers/multirotor_rate_mpc.py:152-153`）：**每周期重建 2 次 CasADi 模型**，
     只为取 `nx`/`nu` 两个整数，单次就是 **3.10 ms**；另外逐帧读 51×2 个 numpy 数组 ~1.7 ms。
     ⟹ **给 [04](04-offline-ocp.md)/[06](06-mpc-node-fake-px4-fixture.md) 的硬结论**：
     移植时**必须把模型构建提出循环**、`nx`/`nu` 从 `model.x.size()` 直接取。
     **这不是优化项，是能不能跑 100 Hz 的前提。**

- **2026-09-28（agent，两个新坑 —— 原文档里没有的）**

  1. ★ **上游漏装 `lib/link_libs.json`**：它是 `CMakeLists.txt:371-381` 用 `file(WRITE …)`
     在 **configure 时写进源码树**的，**全仓没有任何 install 规则**（已 grep 全仓 `CMakeLists.txt`/`*.cmake` 核实）。
     官方流程「构建目录 = 安装目录 = 源码目录」恰好掩盖了这点；**out-of-source 必踩**：
     `AcadosOcpSolver.__init__` 第一行 `make_consistent()` 就 `FileNotFoundError`
     （`acados_code_gen_options.py:362`）。已在构建脚本里搬（⚠️ **每次构建后都要搬**，内容随开关变）。
     同处还写 `lib/git_commit_hash`（缺了只是 warning）。
     ⚠️ 这两个文件在 acados 自己的 `.gitignore:108`（`lib/`）里 ⟹
     **`git -C ~/refs/acados status` 仍然干净**，但「只读副本」这个词对它们不成立（已在 refs-manifest 里注明）。
  2. **该机 `pypi.org` 不可达**（`curl` 15 s 超时，而 `files.pythonhosted.org` 正常）⟹
     `pip3 install` **卡十分钟一声不响**（本项目先前那次就是这么卡住的）。
     加 `-i https://pypi.tuna.tsinghua.edu.cn/simple` 即可。

- **2026-09-28（agent，明确不改的边界 —— 按「别顺手改别的票」）**
  「`GENERIC` 掉一个量级」与「OpenMP 冲突」这两句话在**另外三处**也写着：
  `PROJECT.md:408`、`docs/reference/hardware.md:276`、`docs/reference/open-source-survey.md:59`。
  本张**只改了** `phase2-implementation.md` §S2 与 `mpc-roadmap.md` §5（这两处是 acados 部署的**直接出处**），
  **另外三处留待下一轮文档清扫一并对齐** —— 不是漏了，是不越界（详见交付文档 §8 遗留表）。

- **2026-09-28（agent，取证环境说明）**：为量「掉一个量级」「OpenMP 冲突」这两句话，
  额外构建了 `~/acados-generic`（BLASFEO=GENERIC）与 `~/acados-omp`（OpenMP=ON）两个**对照安装目录**，
  用同一份验收脚本交替测。**这两个目录是取证产物、不是交付物**；`~/acados` 才是采用的那一份。
  版本错配告警 `AcadosOcpDims.N has been migrated to …N_horizon`（参考实现写 `ocp.dims.N`，
  0.6.0 已迁到 `ocp.solver_options.N_horizon`）**只记录、不改参考实现**（改动属 [04](04-offline-ocp.md)）。
