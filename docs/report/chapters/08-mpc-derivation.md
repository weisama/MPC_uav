# 第 8 章 MPC 公式推导与应用

本章从刚体动力学出发，推导 z0 实际运行的十状态最优控制问题，再把它逐层落到 PX4 的接口上。全章按三层写：第一层是牛顿–欧拉方程与四元数运动学，属于物理推导；第二层是坐标系、四元数乘法顺序与符号映射，属于代码约定；第三层是权重、时域与求解器参数，属于工程选择。三层分开写，是为了让读者能判断哪一步是“推出来的”，哪一步是“选出来的”。

第 5 章已给出两条控制链的接口边界，本章不重复话题表；第 9 章讲实验设计，第 10 章给结果。本章只回答一个问题：控制器每一拍解的那个优化问题，究竟长什么样，凭什么能发到 PX4 上。

---

## 8.1 坐标系与符号

### 8.1.1 三个坐标系与转换边界

模型内部只用两套坐标系：世界系 ENU（东—北—天）与机体系 FLU（前—左—上）。PX4 内部用 NED（北—东—地）与 FRD（前—右—下）。两套体系不是同一套轴换了名字，而是轴的方向真的不同；表 8-1 给出每个量的口径与转换发生的位置。

**表 8-1　坐标系口径与转换边界。**

| 量 | 模型内部 | PX4 内部 | 转换由谁做 |
|---|---|---|---|
| 位置、速度 | ENU | NED | mavros 边界 |
| 姿态四元数 | ENU/FLU，$q$ 把机体系矢量旋到世界系（机体相对世界系的姿态） | NED/FRD | mavros 边界 |
| 机体角速率 | FLU | FRD | mavros 的 `setpoint_raw` 插件 |
| 总推力 | 标量 $F\ge 0$，沿机体 +z | `thrust_body_z`，向上为负 | PX4 的 `fill_thrust` |

模型的状态输入这一侧没有坐标转换：动捕给的就是 ENU/FLU，`pose_to_state` 原样装进状态向量。转换只发生在输出侧，而且只发生一次。这个选择的代价是本章 8.8 节必须把符号讲清楚；收益是模型内部没有第二套坐标约定，少一个静默出错的转换点。

四元数约定与代码一致：$q=[w,x,y,z]$，$q$ 把**机体系**矢量旋到**世界系**，

$$
\mathbf v_{\text{world}}=R(q)\,\mathbf v_{\text{body}}.
\tag{8-1}
$$

`q_to_rot_mat` 按这个方向展开旋转矩阵（`model.py:68-75`），`v_dot_q` 用它把推力转到世界系（`model.py:78-80`）。源码注释把 $q$ 记作“机体 ← 世界”（`model.py:25`），这里的箭头是“机体的姿态相对世界系”的意思；旋转矩阵本身的方向以式 (8-1) 为准，不要按箭头把它读成“把世界系矢量旋到机体系”。若把方向读反，推力方向会整体反过来，症状是飞机一解锁就往下扎，而求解器本身不会报任何错。

<big>**一句话总结：模型内部用 ENU 世界系与 FLU 机体系，坐标转换只在 mavros 边界发生一次。**</big>

### 8.1.2 符号与代码变量对照

表 8-2 是本章的符号总表。同一个物理量在论文记号、代码变量与 PX4 字段里往往有三个名字，读公式时以本表为准。

**表 8-2　符号、代码变量与出处。**

| 符号 | 含义 | 代码变量 | 出处 |
|---|---|---|---|
| $\mathbf p\in\mathbb R^3$ | 世界系位置 | `p`，状态 `x[0:3]` | `model.py:163-166` |
| $\mathbf v\in\mathbb R^3$ | 世界系速度 | `v`，状态 `x[3:6]` | 同上 |
| $q\in\mathbb R^4$ | 把机体系矢量旋到世界系的单位四元数（机体相对世界系的姿态） | `q`，状态 `x[6:10]` | 同上 |
| $F\in\mathbb R$ | 沿机体 +z 的总推力，N | `F`，输入 `u[0]` | `model.py:168-170` |
| $\boldsymbol\omega\in\mathbb R^3$ | 机体三轴角速率，rad/s，FLU | `w`，输入 `u[1:4]` | `model.py:169-170` |
| $m$ | 起飞质量，kg | `params.mass` | `params.py:168` |
| $g$ | 重力加速度，m/s² | `9.81`（`params.G`） | `params.py:39`、`model.py:177` |
| $N$ | 预测步数 | `OcpConfig.N` | `ocp.py:57` |
| $T_f$ | 预测时域，s | `OcpConfig.tf` | `ocp.py:58` |
| $\Delta t$ | 内部步长，s | `OfflineOcp.dt` | `ocp.py:424-427` |
| $\mathbf x,\mathbf u$ | 状态、输入向量 | `x`、`u` | `model.py:49-50` |
| $\mathbf x_0$ | 本拍初值 | `x0` | `node_core.py:539-552` |
| $\mathbf x_{\mathrm{ref}},\mathbf u_{\mathrm{ref}}$ | 参考状态、参考输入 | `x_ref`、`u_ref` | `node_core.py:646-776` |
| $Q,Q_e,R$ | 代价权重矩阵 | `q_diag`、`q_e_diag`、`r_diag` | `ocp.py:64-87` |
| $f_{\max}$ | 电池可持续推力上限，N | `params.f_max` | `params.py:96`、`181` |
| $t_{\max}$ | 推力归一化常数，N | `params.t_max` | `params.py:189-207` |
| $T_{\mathrm{hover}}$ | 悬停所需总推力，N | `params.t_hover` | `params.py:184-187` |
| $h$ | 悬停归一化推力 | `params.hover_thrust` | `params.py:72`、`171` |

$Q$、$Q_e$、$R$ 在代码里存成对角线元组，装配求解器时才乘 2 变成矩阵（`ocp.py:255-257`）。这个“乘 2”与 acados 的半系数有关，见 8.5.2。

还有两处记号容易混。其一，代码里角速率的变量名是 `w`，而四元数的标量分量记作 $q_w$；`w` 不是 $q_w$。其二，第 5 章把总推力记作 $T$，本章按代码变量 `u[0]` 记作 $F$；$t$ 只表示归一化推力 $F/t_{\max}$。读跨章公式时以这两个对照为准。


---

## 8.2 刚体动力学

### 8.2.1 平动

把飞机当成一个质量固定的刚体。世界系里，位置对时间的导数就是速度：

$$
\dot{\mathbf p}=\mathbf v.
\tag{8-2}
$$

牛顿第二定律写成

$$
m\dot{\mathbf v}=\mathbf F_{\text{total}}.
\tag{8-3}
$$

作用在飞机上的力只有两项：螺旋桨推力与重力。推力在机体系里沿 +z，大小记为 $F$：

$$
\mathbf f_{\text{body}}=\begin{bmatrix}0\\0\\F\end{bmatrix},
\qquad
\mathbf g=\begin{bmatrix}0\\0\\-9.81\end{bmatrix}.
\tag{8-4}
$$

重力已经写成“单位质量受力”，所以式 (8-3) 除以质量后，重力项不再带 $m$。推力要先从机体系转到世界系，再做除法：

$$
\dot{\mathbf v}=\frac{1}{m}R(q)\begin{bmatrix}0\\0\\F\end{bmatrix}
+\begin{bmatrix}0\\0\\-9.81\end{bmatrix}.
\tag{8-5}
$$

式 (8-2) 与式 (8-5) 就是代码里的前六行：`force = [0,0,F]`、`a_thrust = v_dot_q(force, q) / mass`、`f_expl = [v; a_thrust + g; …]`（`model.py:177-182`）。$g$ 在代码里写成 `[0,0,-9.81]`，注释明确标了“ENU：重力朝 −z”（`model.py:177`）。

这里没有阻力项。模型假设推力是唯一的非重力外力，空气阻力、桨叶挥舞、机身升力都不进模型。这个假设在低速悬停与 1 m/s 量级的轨迹上够用，但它不是“阻力很小所以可以忽略”的定量结论；第 10 章的实飞误差里包含这一项，不能全部归给参数失配。


### 8.2.2 转动：四元数运动学

转动部分要回答的是：机体以角速度 $\boldsymbol\omega$ 相对世界系转动时，表示姿态的四元数 $q$ 怎么随时间变化。设 $q(t)$ 是把机体系矢量旋到世界系的单位四元数，$R(q)$ 是它对应的旋转矩阵。四元数与旋转矩阵的关系是群同态：

$$
R(q_1\otimes q_2)=R(q_1)R(q_2).
\tag{8-6}
$$

在很短的时间 $\Delta t$ 内，机体绕机体系瞬时轴转过 $\Delta\boldsymbol\theta=\boldsymbol\omega\Delta t$。这个微小转动对应的四元数是

$$
\delta q\approx\begin{bmatrix}1\\ \tfrac12\Delta\boldsymbol\theta\end{bmatrix}
=\begin{bmatrix}1\\ \tfrac12\boldsymbol\omega\Delta t\end{bmatrix}.
\tag{8-7}
$$

姿态的更新可以写成右乘：$q(t+\Delta t)=q(t)\otimes\delta q$。代入式 (8-6)，

$$
R\big(q(t+\Delta t)\big)=R(q(t))\,R(\delta q)
\approx R(q(t))\left(I+[\Delta\boldsymbol\theta]_\times\right),
\tag{8-8}
$$

其中 $[\cdot]_\times$ 是叉乘矩阵。两边减去 $R(q(t))$ 再除以 $\Delta t$，得到旋转矩阵的微分方程

$$
\dot R=R\,[\boldsymbol\omega]_\times.
\tag{8-9}
$$

四元数这边，把 $q(t+\Delta t)=q(t)\otimes\delta q$ 展开到一阶：

$$
\dot q=\frac12\,q\otimes\begin{bmatrix}0\\ \boldsymbol\omega\end{bmatrix}.
\tag{8-10}
$$

这一步没有省掉中间代数：式 (8-10) 就是式 (8-9) 的四元数写法，右乘来自“微小转动作用在机体系”。写成 $q\otimes[0,\boldsymbol\omega]^{\mathsf T}$ 而不是 $[0,\boldsymbol\omega]^{\mathsf T}\otimes q$，是约定问题；两种写法对应不同的物理含义，差一个符号。

代码里的矩阵形式是

$$
M(\boldsymbol\omega)=
\begin{bmatrix}
0&-\omega_x&-\omega_y&-\omega_z\\
\omega_x&0&\omega_z&-\omega_y\\
\omega_y&-\omega_z&0&\omega_x\\
\omega_z&\omega_y&-\omega_x&0
\end{bmatrix},
\qquad
\dot q=\frac12 M(\boldsymbol\omega)q.
\tag{8-11}
$$

把式 (8-10) 按四元数乘法逐项展开，并与式 (8-11) 对照，可以确认 $M(\boldsymbol\omega)q=q\otimes[0,\boldsymbol\omega]^{\mathsf T}$——也就是说，代码实现的是**右乘**。这一点在源码里容易读反：`skew_symmetric` 的 docstring 把它简写成 $\tfrac12\,\mathrm{skew}(\boldsymbol\omega)\otimes q$（`model.py:54-57`），但同一段紧接着说明“排布是为四元数右乘写的”。本章按实际实现写式 (8-10)；若按左乘展开，$y$、$z$ 两个分量的符号会与代码相反。

逐分量写出，就是求解器实际积分的那四行：

$$
\begin{aligned}
\dot q_w&=-\tfrac12(\omega_xq_x+\omega_yq_y+\omega_zq_z),\\
\dot q_x&=\tfrac12(\omega_xq_w+\omega_yq_z-\omega_zq_y),\\
\dot q_y&=\tfrac12(\omega_yq_w-\omega_xq_z+\omega_zq_x),\\
\dot q_z&=\tfrac12(\omega_zq_w+\omega_xq_y-\omega_yq_x).
\end{aligned}
\tag{8-12}
$$

式 (8-12) 与 `model.py:182` 的 `1/2 * mtimes(skew_symmetric(w), q)` 逐项一致。写成时另做过一次数值对拍：随机取单位四元数与角速率，`quat_mul(q, [0, ω])` 与 `M(ω)q` 的差在 $10^{-15}$ 量级。这不是“公式看起来对”，而是两条独立的计算路径给出同一个结果；仓库里没有常驻的这条对拍用例，判据只能由写作者当场核。


### 8.2.3 三条建模假设及其边界

式 (8-5) 与式 (8-12) 不是无条件成立的。模型显式或隐式地用了表 8-3 里的假设；把它们与“参数不准”分开，是读第 10 章误差的前提。

**表 8-3　建模假设与失效边界。**

| 假设 | 写在哪里 | 真实系统里被什么破坏 | 参数扫描能否发现 |
|---|---|---|---|
| 内环瞬时：输入的角速率被当成实际角速度（指令即真值） | `model.py:182` 直接拿输入 `w` 进运动学 | PX4 角速率环有有限带宽、延迟与饱和 | 不能；`rate`/`inertia` 旋钮只探动态品质 |
| 无气动阻力 | 式 (8-5) 没有阻力项 | 速度越高阻力越大 | 不能；它不是某个可调参数 |
| 推力沿机体 +z、质量常值 | `model.py:178`、`params.py:168` | 桨叶挥舞、机架变形、电池消耗 | 质量扫描只覆盖静态质量 |
| 四元数保持单位长度 | 模型没有 $\lVert q\rVert=1$ 的约束 | 数值积分漂移 | 不能；漂移是模型精度指标 |

第一条最关键。$q$ 的运动学里吃的是**指令角速率**；模型把它直接当成真实角速度，等于假设 PX4 的角速率环一收到指令就精确跟上。这个假设在 100 Hz 的外环与 1 kHz 的内环分层下是合理的工程近似，但它不是“参数误差”，无法靠把 `mass` 或 `hover_thrust` 调准来消除。13 状态模型要显式建模这条内环，见 8.12。

第三条里的质量常值也需要说清楚：`Z0Params.mass` 是起飞质量，飞行中电池放电会让质量缓慢下降。这个变化远小于本文讨论的 ±30%，但在长航时任务里不能一直当成零。

<big>**一句话总结：模型把内环当理想环节，并忽略阻力、变质量与非 +z 推力，这些是结构假设。**</big>

---

## 8.3 十状态模型

### 8.3.1 状态与输入

把式 (8-2)、式 (8-5)、式 (8-12) 拼成一个一阶系统。状态取

$$
\mathbf x=\begin{bmatrix}\mathbf p\\ \mathbf v\\ q\end{bmatrix}\in\mathbb R^{10},
\qquad
\mathbf u=\begin{bmatrix}F\\ \boldsymbol\omega\end{bmatrix}\in\mathbb R^{4}.
\tag{8-13}
$$

完整的连续动力学是

$$
\dot{\mathbf x}=f(\mathbf x,\mathbf u)=
\begin{bmatrix}
\mathbf v\\
\dfrac{1}{m}R(q)\begin{bmatrix}0\\0\\F\end{bmatrix}+\begin{bmatrix}0\\0\\-9.81\end{bmatrix}\\
\dfrac12\,q\otimes\begin{bmatrix}0\\\boldsymbol\omega\end{bmatrix}
\end{bmatrix}.
\tag{8-14}
$$

代码里的维度断言是 `NX = 10`、`NU = 4`（`model.py:49-50`），状态切片 `Q_SLICE = slice(6, 10)` 对应四元数（`model.py:51`）。式 (8-14) 的右端就是 `f_expl`（`model.py:182`），`dynamics()` 与 `step_rk4()` 都调用同一个 [CasADi](https://web.casadi.org/) 表达式（`model.py:194-208`）。

状态里没有 $\boldsymbol\omega$。角速率是输入，不是状态；这是“内环理想”假设的直接后果，也是十状态与十三状态的分界。十三状态版本会再加三个状态与一行 $\dot{\boldsymbol\omega}=k_\omega(\boldsymbol\omega_{\mathrm{des}}-\boldsymbol\omega)$，但那需要先辨识 $k_\omega$，当前没有做。

<big>**一句话总结：模型状态是位置、速度、四元数共十维，输入是推力与三轴角速率共四维。**</big>

### 8.3.2 输入为什么是“角速率指令”而不是力矩

输入选 $\boldsymbol\omega$ 而不是力矩，是接口决定模型的典型例子。z0 采用 (C) 级架构：控制器只下到角速率，PX4 保留 1 kHz 角速率环、控制分配与混控。模型如果以力矩为输入，就必须知道转动惯量、电机时间常数与分配矩阵；这些量在 z0 上没有全部辨识，而且一旦换桨或换电池就会变。以角速率为输入，模型把内环当成一个已经调好的执行器，只负责外环的预测。

这也是参考实现 `px4-mpc` 的选择（[`docs/control/phase2-implementation.md`](../../control/phase2-implementation.md) §2.1），并与 `Sun2022` §III.A、`Foehn2022` §D.1 的分层一致：论文里的高带宽内环承担姿态跟踪，外环只规划推力与角速率。z0 没有实现论文里的 INDI 内环，PX4 的角速率环就是这里的“内环”。

这个选择的代价写在表 8-3 第一行：模型看不到内环的延迟与饱和。收益是模型维度低、求解快，且在 (C) 级接口上不需要额外的执行器辨识。

<big>**Sun2022 是本项目控制器内（PID角度环）/外环（MPC）的选型依据。**</big>

### 8.3.3 这套控制能不能称为 NMPC

本章标题用“MPC”作统称。按预测模型与优化问题的性质，z0 这套控制器属于**非线性 MPC（NMPC）**，不是线性 MPC。

判据在模型，不在求解器。式 (8-14) 右端有两处非线性：推力先经 $R(q)$ 旋转，四元数导数含 $q\otimes[0,\boldsymbol\omega]$ 的双线性项（`model.py:180-182`）。线性 MPC 要求预测模型能写成 $\mathbf x_{k+1}=A\mathbf x_k+B\mathbf u_k$，这里不存在这样的常数矩阵 $A,B$。

求解器的内部线性化不改变分类。[acados](https://github.com/acados/acados) 把问题装配成非线性 OCP（`NONLINEAR_LS`，`ocp.py:258-263`），`SQP_RTI` 每拍做一次 SQP 迭代（`ocp.py:287-293`）；每步迭代都要线性化，这是求解非线性问题的常规手段，不等于对被控对象做线性化。术语表把 NMPC 与线性 MPC 区分开（[CONTEXT.md](../../../CONTEXT.md)），Phase 4 的接口表也把 `mpc_node` 直接写作非线性 MPC 控制器（[15-trajectory-flight-roles.md](../../../.scratch/phase4/issues/15-trajectory-flight-roles.md)）。

需要保留的边界：内环理想、无阻力、十状态都是**结构近似**，不是线性化；称 NMPC 不表示模型更精确，也不表示每拍都收敛到最优解。

<big>**一句话总结：预测模型含姿态旋转与四元数双线性项，这套控制属于非线性 MPC。**</big>

---

## 8.4 离散化

### 8.4.1 连续模型的离散形式

求解器不能直接处理连续微分方程，要在时域上离散。取步长

$$
\Delta t=\frac{T_f}{N}=\frac{1.0}{50}=0.02\ \mathrm{s},
\tag{8-15}
$$

并假设每一步内输入不变（零阶保持）。离散动力学写成

$$
\mathbf x_{k+1}=f_d(\mathbf x_k,\mathbf u_k),\qquad k=0,1,\dots,N-1.
\tag{8-16}
$$

$N=50$、$T_f=1.0$ 写在 `OcpConfig` 里（`ocp.py:57-58`），$\Delta t$ 由 `tf / N` 算出（`ocp.py:424-427`）。50 步、20 ms 的组合意味着预测窗口覆盖 1 s；这个长度要大于外环的主要时间常数，又要短到模型误差不会把远端预测毁掉。它是工程选择，不是从动力学推出来的。

式 (8-16) 里的 $f_d$ 不是解析矩阵指数，而是数值积分。acados 在每一步内部用显式 Runge–Kutta 把式 (8-14) 从 $t_k$ 积到 $t_{k+1}$；节点与测试看到的离散模型就是“积分器 + 连续右端”的组合。

<big>**一句话总结：连续模型按 20 ms 步长离散成 50 步，覆盖 1 s 时域，由求解器内部积分。**</big>

### 8.4.2 RK4 与 ERK 的分工

`model.py` 里有一个 `step_rk4`：

$$
\mathbf x_{k+1}=\mathbf x_k+\frac{\Delta t}{6}\left(k_1+2k_2+2k_3+k_4\right),
\tag{8-17}
$$

其中 $k_1=f(\mathbf x_k,\mathbf u_k)$、$k_2=f(\mathbf x_k+\tfrac{\Delta t}{2}k_1,\mathbf u_k)$，$k_3$、$k_4$ 同理（`model.py:198-208`）。这个函数只给测试与闭环数值仿真用，源码注释明确写了“**不是** OCP 里的积分器”。

最优控制问题内部用的是 acados 的 ERK 积分器，配置为 4 级、3 个子步（`ocp.py:289-292` 的 `sim_method_num_stages = 4`、`sim_method_num_steps = 3`）。两者都是显式 Runge–Kutta 家族，但不是同一个实现：`step_rk4` 是单步四级的定步长 RK4，ERK 是在每个 20 ms 区间里再分 3 个子步的显式积分。把 `step_rk4` 的公式当成 OCP 的离散化写进论文，会把“测试工具”误写成“求解器配置”；本章把这两件事分开。

<big>**一句话总结：acados 的四级三步 ERK 大幅优化求解器计算实机。**</big>

---

## 8.5 最优控制问题

### 8.5.1 一般形式

每一拍，控制器解一个有限时域最优控制问题：

$$
\begin{aligned}
\min_{\mathbf u_0,\dots,\mathbf u_{N-1}}\quad & J(\mathbf x_{0:N},\mathbf u_{0:N-1})\\
\text{s.t.}\quad & \mathbf x_{k+1}=f_d(\mathbf x_k,\mathbf u_k),\quad k=0,\dots,N-1,\\
& \mathbf x_0=\hat{\mathbf x},\\
& \mathbf u_k\in\mathcal U,\quad k=0,\dots,N-1.
\end{aligned}
\tag{8-18}
$$

$\hat{\mathbf x}$ 是本拍的状态估计。它不是直接取最近一帧动捕，而是把位置按速度外推一个固定延迟 $\tau=50$ ms：

$$
\hat{\mathbf p}=\mathbf p_{\text{mocap}}+\mathbf v_{\text{mocap}}\tau,
\qquad
\hat{\mathbf v}=\mathbf v_{\text{mocap}},
\qquad
\hat q=q_{\text{mocap}}.
\tag{8-19}
$$

`extrapolate_state` 只外推位置，速度与四元数原样保留（`node_core.py:539-552`），默认 $\tau=50$ ms（`node_core.py:110` 的 `TAU_DEFAULT_S`，经 `ControlConfig.tau_s` 下发，`node_core.py:309-337`）。这个外推补偿的是动捕到控制器的端到端延迟；它没有补偿姿态，因为角速率积分需要陀螺仪，而动捕链路没有给角速率。

式 (8-18) 的初值等式在求解器里不是软约束，而是把第 0 级的上下界都设成同一个值：`solver.set(0, "lbx", x0)` 与 `"ubx"`（`ocp.py:339-340`）。因此 $\mathbf x_0$ 不参与优化，只参与代价计算。

<big>**一句话总结：优化问题在动力学、初值等式与输入盒约束下最小化有限时域代价。**</big>

### 8.5.2 代价函数与权重

代价采用 acados 的 `NONLINEAR_LS` 形式。最小二乘残差取 $\mathbf y=[\mathbf x;\mathbf u]$，终端残差只取 $\mathbf x$：

$$
\mathbf y_k=\begin{bmatrix}\mathbf x_k\\ \mathbf u_k\end{bmatrix},
\qquad
\mathbf y_{e}=\mathbf x_N.
\tag{8-20}
$$

代价写成

$$
J=\frac12\sum_{k=0}^{N-1}\left\|\mathbf y_k-\mathbf y_{\mathrm{ref},k}\right\|_{W}^{2}
+\frac12\left\|\mathbf x_N-\mathbf x_{\mathrm{ref},N}\right\|_{W_e}^{2},
\tag{8-21}
$$

其中 $W=\operatorname{diag}(2Q,2R)$、$W_e=2Q_e$。acados 的 `NONLINEAR_LS` 定义里带 $\tfrac12$，装配时把权重乘 2（`ocp.py:255-261`），两者正好抵消，所以按对角线权重手算的代价就是

$$
J=\sum_{k=0}^{N-1}\left(
\sum_{i=0}^{9}q_i\,e_{x,k,i}^{2}
+\sum_{j=0}^{3}r_j\,e_{u,k,j}^{2}
\right)
+\sum_{i=0}^{9}q_{e,i}\,e_{x,N,i}^{2}.
\tag{8-22}
$$

`OcpConfig.cost()` 就是按式 (8-22) 实现的（`ocp.py:93-110`）。它单独存在的原因见 8.7.1：`SQP_RTI` 下求解器自报的代价不可信。

表 8-4 给出三组对角线权重与它们的作用。

**表 8-4　代价权重（`ocp.py:64-87`）。**

| 组 | 顺序 | 取值 | 含义 |
|---|---|---|---|
| `q_diag` | $[p(3),v(3),q(4)]$ | $(10,10,10,\ 10,10,10,\ 0.1,2,2,2)$ | 阶段状态代价 |
| `q_e_diag` | 同上 | $(300,300,300,\ 100,100,100,\ 0,0,0,0)$ | 终端状态代价 |
| `r_diag` | $[F,\omega_x,\omega_y,\omega_z]$ | $(10,\ 500,500,500)$ | 输入代价 |

**位置与速度。** 阶段权重都是 10，终端位置 300、终端速度 100。终端位置是阶段的 30 倍、终端速度是 10 倍，作用是把误差往终点压；终端姿态权重为 0，终点只约束位置与速度。

**姿态。** 阶段四元数段是 $(0.1,2,2,2)$，终端是 0。$q_w=0.1$ 不是随手填的：若 $q_w=0$，参考航向 $\psi$ 与它的镜像 $2\pi-\psi$ 只差 $w$ 的符号，代价对两者完全相同，偏航就成了一条死通道。旧权重下实测“参考朝向与镜像朝向的代价都恰好为 0”；把 $q_w$ 提到 0.1 后，镜像代价变成有限值，这条死通道才被打开（`ocp.py:60-67`、[phase2 票 16](../../../.scratch/phase2/issues/16-ocp-yaw-weights-mirror-well.md)）。$q_{\text{att}}=2.0$ 是相对参考实现的 0.1 提高的，方向是增加姿态误差的代价，不是“调参调出来的最优值”。

终端四元数段仍是 0，这是照抄参考实现。镜像井由阶段项消掉，终端不再重复罚姿态；代价是终点没有显式的姿态收敛要求，姿态靠阶段项与动力学传递。

**输入。** 推力权重 10，三轴角速率权重都是 500。角速率权重大，是为了压指令抖动；它并不改变约束，只改变解在可行域里的位置。

三组权重的绝对值照搬参考实现，z0 只改过四元数段（`offline-ocp.md` §2）；本章不把 10、300、100 这些绝对值包装成推导结果。能讲出理由的是相对大小：终端高于阶段（把误差往终点压），角速率高于推力（压抖动），阶段姿态略高于 0（打开偏航通道）。

关于偏航权重 500，有一条容易被误读的实测：2026-09-29 曾把 $\omega_z$ 的权重降到 50（“方案 E”），当天撤回。原因不是“50 不稳定”，而是 10 m 不可行步进下 RTI 迭代收敛极慢：逐拍变化按几何衰减，60 拍的探针里看起来像逐拍乱跳，跑到 400 拍才衰减到 $2.6\times10^{-4}$；权重 500 在同一场景第 50 拍就精确收敛。但也不能反过来说“500 更稳”：同一张扫描里 500 在 6 m（$\omega_x$）与 7 m（$F$）上各有真极限环，50 在 3/4/5/6 m 上也有。权重是代价，不是增益旋钮；稳定区逐工况变，见 `ocp.py:68-87` 与 [`tools/rti-convergence/`](../../../tools/rti-convergence) 的读数。

**偏航通道仍然弱。** $q_w=0.1$、$q_{\text{att}}=2.0$ 只解决了“代价分不出镜像”，没有把偏航变成高权威通道。0.4 rad 的机头误差在 $\omega_z$ 权重 500 下只换来约 $3.9\times10^{-4}$ rad/s 的稳态指令，时间常数在 100–1000 s 量级；这不是本章权重能解决的问题，票 16 的收口记录把它留作已知限制。

<big>**一句话总结：代价用状态与输入相对参考的加权平方和，终端只罚位置与速度。**</big>

### 8.5.3 参考与前馈

参考状态由 `node_core` 生成，分三档：保持、轨迹、绝对目标。三档的共同点是姿态都取“水平 + 纯偏航”，位置按参考自己的速度在时域上线性外推：

$$
\mathbf p_{\mathrm{ref}}(t_k)=\mathbf p_{\text{anchor}}+\mathbf p_{\text{rel}}+\mathbf v\,t_k,
\qquad
\mathbf v_{\mathrm{ref}}=\mathbf v,
\qquad
q_{\mathrm{ref}}=q_{\text{yaw}}(\psi).
\tag{8-23}
$$

`traj_reference` 先把基准状态铺满 $N+1$ 个点，再给位置加上 $\tau_k\mathbf v$（`node_core.py:646-684`）；速度与姿态逐点相同。参考输入固定为

$$
\mathbf u_{\mathrm{ref}}=\begin{bmatrix}m g\\0\\0\\0\end{bmatrix},
\tag{8-24}
$$

由 `hold_feedforward` 返回（`node_core.py:713-776`）。

式 (8-24) 不是“悬停时随便填的数”，它让参考自己成为模型的一条精确解。把式 (8-23)、式 (8-24) 代入式 (8-14)：$q$ 常量、$\boldsymbol\omega=0$ 给出 $\dot q=0$；推力项 $R(q_{\text{yaw}})[0,0,mg]^{\mathsf T}/m=[0,0,g]^{\mathsf T}$，与重力抵消，给出 $\dot{\mathbf v}=0$；而 $\dot{\mathbf p}=\mathbf v$ 正是式 (8-23) 的时间导数。于是

$$
f(\mathbf x_{\mathrm{ref}},\mathbf u_{\mathrm{ref}})=\dot{\mathbf x}_{\mathrm{ref}}.
\tag{8-25}
$$

这条等式对“保持”与“轨迹”两档都成立，判据是 `test_node_core.py::test_轨迹参考是模型的一条精确解`（拿 `step_rk4` 逐步往前推）。它成立的前提是参考族只有匀速平动与常量偏航；一旦参考带持续加速度或持续角速率，式 (8-25) 立刻失效。

**$\mathbf u_{\mathrm{ref}}$ 是代价参考，不是指令。** 真正发出去的是求解器输出的 $\mathbf u_0$。这一点在垂直通道上被实测放大过：把质量失配的积分偏置加进 $\mathbf u_{\mathrm{ref}}$，实测灵敏度

$$
\frac{\partial u_0[0]}{\partial u_{\mathrm{ref}}[0]}=-0.0616\ \mathrm{N/N}
\tag{8-26}
$$

是负的，积分作用会变成正反馈：越沉，偏置越大，解出的推力反而越小。该落点已被证伪并删除（`node_core.py:713-776` 的 ☠️ 段）。现在的垂直偏置加在**发出去的指令**上：

$$
u_{\mathrm{cmd}}[0]=\operatorname{clip}\big(u_0[0]+\Delta F,\ 0,\ f_{\max}\big),
\tag{8-27}
$$

$\Delta F$ 是纯积分状态，$e_z=z_{\mathrm{ref}}-z_{\mathrm{meas}}$，增益 $k_i=1\ \mathrm{N/(m\cdot s)}$，状态夹在 $[-T_{\mathrm{hover}},f_{\max}-T_{\mathrm{hover}}]$ 内（`mpc_node.py:1094-1120`、`1241`、`484`）。夹状态是为了“退得回来”，夹输出是为了“不越界”，两者不能合并。式 (8-27) 不属于优化问题，它是发布前的附加项；本章把它写在这里，是为了让读者知道十状态模型之外还有一层慢积分。

<big>**一句话总结：参考前馈取悬停推力与零角速率，参考本身因此是模型的一条精确解。**</big>

---

## 8.6 约束

### 8.6.1 推力上下限

优化问题对输入只有盒约束。推力那一路是

$$
0\le F\le f_{\max}=11.8\ \mathrm{N}.
\tag{8-28}
$$

代码里写成 `lbu = [0, -ω_max]`、`ubu = [f_max, ω_max]`、`idxbu = [0,1,2,3]`（`ocp.py:272-278`）。下界取 0，是因为 FLU 下推力沿机体 +z，反向推力不是这台飞机的控制量；负推力会让模型规划出“吸住”飞机的解，而 PX4 的混控器兑现不了。

上界 $f_{\max}=11.8$ N 是**电池可持续**口径：VTC6 持续放电 15 A，按 22.2 V 与台架效率外推，四电机合计约 11.8 N（`params.py:83-96`、[`docs/report/chapters/03-hardware.md`](03-hardware.md) §3.4）。它不是电机与桨的满量程。

**$f_{\max}$ 与 $t_{\max}$ 是两个量。** 归一化常数

$$
t_{\max}=\frac{T_{\mathrm{hover}}}{h}=\frac{m g}{0.1589}=46.3027\ \mathrm{N}
\tag{8-29}
$$

只负责把牛顿换算成 PX4 的归一化推力；它由悬停点反推，是满电口径的满量程。$t_{\max}$ 是 $f_{\max}$ 的 3.92 倍。把 $t_{\max}$ 当 $F$ 的上界，求解器会规划出电池兑现不了的推力，而且不会报错：节点照参数表飞、测试按同一个模型算预期，可能全绿而飞机掉高。这个坑真实踩过——回填 `hover_thrust` 后 `ocp.py` 的 `F ≤ T_max` 悄悄从“按电池”漂成“按电机”，最后是“又远又高的目标该把推力顶到上限”那条测试以 `38.515 == 46.303` 红了一次（[`docs/control/phase2-implementation.md`](../../control/phase2-implementation.md) §3.3.3）。

<big>**一句话总结：推力下界为零、上界取电池可持续的 11.8 N，归一化常数不参与约束。**</big>

### 8.6.2 角速率上下限

三轴角速率约束是

$$
-3.84\le\omega_x\le 3.84,\qquad
-3.84\le\omega_y\le 3.84,\qquad
-6.28\le\omega_z\le 6.28\ \mathrm{rad/s}.
\tag{8-30}
$$

取值来自飞控参数：`MC_ROLLRATE_MAX` 与 `MC_PITCHRATE_MAX` 是 220 °/s，换算为 3.8397 rad/s；`MC_YAWRATE_MAX` 是 360 °/s，换算为 6.2832 rad/s。代码取三位有效数字 3.84 与 6.28（`params.py:74-81`）。

偏航上限比横滚、俯仰更大，这是四旋翼的常见配置，也是参考实现写错的地方之一：参考实现把偏航写成横滚的一半，z0 改成独立给（`ocp.py:277-280`）。约束加在**指令**上，不是实测角速率上；PX4 的角速率环可能饱和或滞后，模型对此没有建模。

<big>**一句话总结：角速率约束取飞控参数换算后的横滚俯仰 3.84、偏航 6.28 rad/s。**</big>

### 8.6.3 没有实现的约束

当前 OCP 的约束只有式 (8-28) 与式 (8-30) 的输入盒，加上式 (8-18) 的初值等式。以下约束**没有实现**：

- **倾角约束。** 路线图 §4.3 规划了 $R(q)\mathbf e_z\cdot\mathbf e_z\ge\cos45^\circ$，但 `ocp.py` 里只有 `lbu`/`ubu`，没有任何状态约束。当前模型可以规划出任意倾角，只要推力与角速率在盒内。
- **位置、速度硬约束。** 位置与速度只进代价，不是硬约束。`MPC_XY_VEL_MAX=2.0` 是 PX4 位置控制器的软限速，不是这个优化问题的约束。
- **四元数单位长度约束。** 模型没有 $\lVert q\rVert=1$ 的等式约束。正常时漂移在 $10^{-8}$ 量级，涨起来才是轨迹发散的信号；它不进 `degraded` 判据（[`docs/control/offline-ocp.md`](../../control/offline-ocp.md) §4.3）。
- **终端约束集与障碍物约束。** 都没有。

没有倾角约束的后果不是“飞机一定会翻”，而是“模型不会主动阻止大倾角”。实际飞行里先撞上的是 $f_{\max}$ 与角速率上限：45° 悬停所需推力约 $mg/\cos45^\circ=10.4$ N，已经占 $f_{\max}$ 的 88%。这说明推力上限间接限制了可持续倾角，但它不是几何硬约束，瞬时解仍可能规划出更大的倾角。把 45° 写成现状，是本章明确避免的错误。

<big>**一句话总结：倾角、位置与四元数范数约束都未实现，当前只有输入盒约束。**</big>

---

## 8.7 求解器

### 8.7.1 acados 配置与实时迭代语义

表 8-5 是 `_build_solver()` 实际下发的配置（`ocp.py:243-302`）。

**表 8-5　求解器配置。**

| 项 | 取值 | 出处 |
|---|---|---|
| 预测步数 $N$ | 50 | `ocp.py:57` |
| 时域 $T_f$ | 1.0 s | `ocp.py:58` |
| 代价 | `NONLINEAR_LS`，阶段与终端 | `ocp.py:258-263` |
| NLP 求解器 | `SQP_RTI` | `ocp.py:290` |
| QP 求解器 | `PARTIAL_CONDENSING_HPIPM` | `ocp.py:287` |
| Hessian 近似 | `GAUSS_NEWTON` | `ocp.py:288` |
| 积分器 | `ERK`，4 级 3 步 | `ocp.py:289-292` |
| 部分凝聚步数 | `qp_solver_cond_N = N` | `ocp.py:293` |

`SQP_RTI` 是实时迭代（real-time iteration）：它把 SQP 的一步拆成“准备”和“反馈”两半，每个控制周期只做一次迭代，用上一周期的解作为热启动。它追求的是**低延迟**，不是每拍的最优性。单次迭代的解是迭代映射的一步，不是收敛解；第一拍用的是求解器的出厂初值，不能当控制指令。实测中目标 +x 10 m、全新实例的第 0 次 solve 给出 $\omega_y=0$、$F=7.8111$ N，与悬停初值一样，从第 1 次起才开始动（[`docs/control/offline-ocp.md`](../../control/offline-ocp.md) §4.1）。

这条语义解释了节点里的两个设计：预热 5 拍不发（`WARMUP_CYCLES=5`，`node_core.py:127`），以及测试必须先热身再断言（`conftest.py` 的 `solve_warm`）。它也是 `SQP_RTI` 下三条原厂读数不可信的原因：残差恒为 0、`alpha` 直接抛异常、`get_cost()` 与返回轨迹的代价差两个数量级。节点因此自己按式 (8-22) 重算代价，不读 `solver.get_cost()`。

为什么选这套组合：部分凝聚把长时域 QP 的变量规模压下来，与 10 状态、50 步的规模匹配；Gauss–Newton 用雅可比近似 Hessian，牺牲一点收敛速度换每拍耗时；ERK 对这类非刚性动力学足够，且显式积分不需要线性求解。这些是照着参考实现与 acados 的常见配置选的，不是本章推导出来的。

<big>**一句话总结：求解器每周期只做一次实时迭代，因此单拍解是迭代步而不是最优解。**</big>

### 8.7.2 实时预算与实测耗时

控制器的目标频率是 100 Hz（`mpc_node.py:444` 的 `rate_hz = 100.0`），也就是每拍 10 ms。表 8-6 是三趟 2026-10-17 仿真闭环的求解耗时，数据来自 DUT 的 JSONL，`solve_ms` 取 acados 自报的 `time_tot`（`mpc_node.py:1385`）。

**表 8-6　三趟仿真的求解耗时（ms）。**

| 场景 | 有效求解帧 | 均值 | p50 | p95 | 稳态最坏 | 预热首解 | 退化帧 | 求解失败 | 饱和帧 |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| `R-hover` | 7797 | 3.127 | 3.154 | 3.624 | 20.138 | 11.44 | 0 | 0 | 0 |
| `R-square` | 5679 | 2.985 | 3.071 | 3.313 | 6.462 | 12.916 | 0 | 0 | 0 |
| `R-lemniscate` | 6572 | 3.133 | 3.160 | 3.613 | 10.738 | 9.435 | 0 | 0 | 0 |

读法是：中位约 3.1 ms，p95 约 3.3–3.6 ms，占 10 ms 预算的三分之一；稳态最坏在 6.5–20.1 ms 之间，偶尔超过一拍预算。三趟的退化帧、求解失败帧与饱和帧都是 0。

**“最坏超过 10 ms”不等于“丢帧”。** 节点跑在 ROS 定时器上，不是硬实时系统；一次超时会让这一拍晚到，下一拍继续，上一帧 setpoint 仍被 PX4 锁存。真正要担心的是持续超时把有效频率拉低，而不是单次 20 ms。表里 `R-hover` 的 20.138 ms 是稳态窗口里的单帧；这三个汇总只给最大值，回答不了“它是否连续出现”。`R-square` 的 12.916 ms 落在预热首解，稳态最坏只有 6.462 ms；`R-lemniscate` 的 10.738 ms 也在稳态窗口内。

这些是**仿真**读数，不是真机保证。仿真跑在机载电脑上，被控对象是 SIH，没有真实电调的电磁干扰、没有动捕 WiFi 的抖动，也没有并行会话抢 CPU。真机的耗时证据应在第 10 章按同一口径给出；本章不把仿真最坏值写成“实飞最坏值”。

<big>**一句话总结：三趟仿真的求解中位约 3 ms、p95 约 3.6 ms，未出现求解失败或退化。**</big>

---

## 8.8 应用到 PX4

### 8.8.1 CTBR 接口

解出来的第一拍是 $\mathbf u_0=[F,\omega_x,\omega_y,\omega_z]$。它通过 `mavros_msgs/AttitudeTarget` 发到 `/mavros/setpoint_raw/attitude`，字段映射是

$$
\text{body\_rate}_x=\omega_x,\quad
\text{body\_rate}_y=\omega_y,\quad
\text{body\_rate}_z=\omega_z,\quad
\text{thrust}=\frac{F}{t_{\max}},
\tag{8-31}
$$

`type_mask=128`，即忽略姿态四元数，只使用角速率与推力（`mpc_node.py:411-418`、`1296-1306`）。这就是 CTBR（collective thrust + body rate）：总推力加机体三轴角速率。

PX4 收到后保留 1 kHz 角速率环与控制分配（`docs/control/mpc-roadmap.md:330`），把角速率跟踪成电机指令。位置环与姿态环由 MPC 承担；这是 (C) 级接口的定义。`AttitudeTarget` 的 `orientation` 字段不填，因为掩码已经声明忽略它；填了也不会被使用，但会在日志里制造一个看似有意义的姿态量。

只有正常帧更新 `_last_sp`（`mpc_node.py:1304-1305`）。兜底帧与正常帧走同一个发布函数、同样的字段，区别只在内容来自兜底阶梯；这样“保持”在话题上是可观测的（同一组数值连续出现，时间戳在前进），而不是“节点什么都不发”的黑箱。

<big>**一句话总结：下行只发总推力与机体三轴角速率，姿态四元数由掩码忽略。**</big>

### 8.8.2 ENU/FLU 到 FRD 的符号映射

FLU 与 FRD 的关系是绕机体 x 轴转 180°：

$$
\mathbf e_x^{\mathrm{FRD}}=\mathbf e_x^{\mathrm{FLU}},\qquad
\mathbf e_y^{\mathrm{FRD}}=-\mathbf e_y^{\mathrm{FLU}},\qquad
\mathbf e_z^{\mathrm{FRD}}=-\mathbf e_z^{\mathrm{FLU}}.
\tag{8-32}
$$

这不是镜像：对角矩阵 $\operatorname{diag}(1,-1,-1)$ 的行列式是 $+1$，它是一个真旋转。把同一个角速度矢量投影到两套基上，得到

$$
\omega_x^{\mathrm{FRD}}=\omega_x^{\mathrm{FLU}},\qquad
\omega_y^{\mathrm{FRD}}=-\omega_y^{\mathrm{FLU}},\qquad
\omega_z^{\mathrm{FRD}}=-\omega_z^{\mathrm{FLU}}.
\tag{8-33}
$$

也就是“roll 不变、pitch 与偏航取反”，没有额外的轴矢量负号。推力同理：物理推力是 $F\mathbf e_z^{\mathrm{FLU}}$，它在 FRD 的 z 轴上的分量是 $-F$，归一化后

$$
\text{thrust\_body}_z=-\frac{F}{t_{\max}}.
\tag{8-34}
$$

表 8-7 给出端到端的所有权。关键点是：我们发出的 `body_rate` 是 FLU 原样、`thrust` 是正号；FLU→FRD 由 mavros 在 `setpoint_raw.cpp:323-324` 用 $R_x(\pi)$ 做一次；推力的负号由飞控的 `fill_thrust` 在 `mavlink_receiver.cpp:1475` 做一次。

**表 8-7　CTBR 的端到端符号映射。**

| 量 | 我们发出的字段 | mavros | PX4 最终字段 |
|---|---|---|---|
| roll 角速率 | `body_rate.x = +ωx` | FLU→FRD，x 不变 | `roll = +ωx` |
| pitch 角速率 | `body_rate.y = +ωy` | FLU→FRD，y 取反 | `pitch = −ωy` |
| yaw 角速率 | `body_rate.z = +ωz` | FLU→FRD，z 取反 | `yaw = −ωz` |
| 总推力 | `thrust = +F/t_max` | 不转换 | `thrust_body_z = −F/t_max` |

**不要在节点里抢先转一次。** 如果我们在 `mavros_attitude_target` 里先把 FLU 转成 FRD，mavros 会再转一次，两次 $R_x(\pi)$ 互相抵消，飞控最终收到的数值与写对时完全一样。症状极隐蔽：roll 不受影响，pitch 与偏航静默反向。所以断言不能落在“PX4 收到了什么”上，只能落在我们发出的字段上（`sign_map.py:126-154`、[`docs/control/sign-map.md`](../../control/sign-map.md) §5）。想复现这一档，可以用 `offboard_smoke.py --break pitch-preconvert`。

<big>**一句话总结：FLU 到 FRD 是绕机体 x 轴转 180°，角速率取反两根轴，推力在飞控侧取负。**</big>

### 8.8.3 推力归一化链

PX4 的 offboard 角速率分支对 `thrust_body` 不做重标定，而是原样交给混控器（`MulticopterRateControl.cpp:182`；`MC_BAT_SCALE_EN=0`）。因此模型里的牛顿值必须经过一个固定常数换成归一化推力：

$$
t=\frac{F}{t_{\max}},\qquad
t_{\max}=\frac{T_{\mathrm{hover}}}{h}=\frac{m g}{h}.
\tag{8-35}
$$

代入 z0 的数：$T_{\mathrm{hover}}=0.75\times9.81=7.3575$ N，$h=0.1589$，得到 $t_{\max}=46.3027$ N（`params.py:184-207`）。悬停时 $F=T_{\mathrm{hover}}$，于是

$$
t_{\mathrm{hover}}=\frac{T_{\mathrm{hover}}}{t_{\max}}=h=0.1589.
\tag{8-36}
$$

式 (8-36) 就是自洽条件：`hover_thrust` 与 $1/t_{\max}$ 必须出自同一次测量，因为它们是同一条等式的两端。代码用 `t_max = m*g/hover_thrust` 把两者锁死，并用 `check_hover_consistency` 在启动时检查（`params.py:130-160`）。若一边填实测 $h$、一边填一个独立取的 $t_{\max}$，悬停前馈会整体偏，而求解器不会报错。

**`MPC_THR_HOVER` 不在这条链上。** 它是 PX4 位置控制器与悬停推力估计器的参数，z0 现值仍是默认的 0.5，不是实测的 0.1589。在 (C) 级接口下，我们发的是 `thrust_body`，不经过 PX4 位置控制器的悬停油门重标定；把 `hover_thrust` 填成 0.5 会让归一化链整体错约 3.15 倍（$0.5/0.1589$）；[`docs/control/phase2-implementation.md`](../../control/phase2-implementation.md) §3.3.2 把这一项记作 3.9 倍；3.9 与 $t_{\max}/f_{\max}=3.92$ 同值，不是这里的悬停油门比，按 $t_{\max}=mg/h$ 现算应为 3.15。两者的名字撞脸，但消费者不同。

链上还有一个运行时前置：mavros 的 `thrust_scaling` 必须是 1.0。缺失时 `thrust != 0` 的帧会被整帧丢弃，值为 0 时推力恒为 0；两种都是静默的。`preflight_check` 会读活着的 mavros 节点上的这个参数，而不是读启动文件（`preflight_check.py:365-462`）。

<big>**一句话总结：PX4飞控不能精准控制推力大小，只能通过拟合油门-推力曲线，来通过控制油门从而控制推力**</big>

---

## 8.9 参考姿态为什么是水平 + 纯偏航

参考姿态取水平加纯偏航，不是为了让图好看，而是因为这一族参考在无阻力模型里是精确解。

设参考位置按常量速度前进，参考姿态是常量偏航：

$$
\mathbf p_{\mathrm{ref}}(t)=\mathbf p_0+\mathbf v t,\qquad
\mathbf v_{\mathrm{ref}}=\mathbf v,\qquad
q_{\mathrm{ref}}=q_{\text{yaw}}(\psi).
\tag{8-37}
$$

把式 (8-37) 与式 (8-24) 代入式 (8-14)：

$$
\dot{\mathbf v}_{\mathrm{ref}}
=\frac{1}{m}R(q_{\text{yaw}})\begin{bmatrix}0\\0\\mg\end{bmatrix}
+\begin{bmatrix}0\\0\\-g\end{bmatrix}
=\begin{bmatrix}0\\0\\g\end{bmatrix}
+\begin{bmatrix}0\\0\\-g\end{bmatrix}
=\mathbf 0.
\tag{8-38}
$$

同时 $\dot q_{\mathrm{ref}}=0$（因为 $\boldsymbol\omega=0$），$\dot{\mathbf p}_{\mathrm{ref}}=\mathbf v$。三项都与参考自己的导数一致，所以式 (8-25) 成立。

物理上，这个结果只说了一件事：**倾角是产生水平加速度的手段，不是维持水平速度的手段。** 在无阻力模型里，匀速平动不需要任何水平力，水平姿态加悬停推力就够了。真实飞机有阻力，维持匀速确实需要一点倾角；这一点被模型忽略，表现为稳态跟踪误差，而不是参考姿态写错。

`/traj/reference` 只给位置与速度，加速度字段在掩码里被忽略（`type_mask=2496`，`node_core.py:588-634`）。参考源没有加速度信息，控制器就算想算“维持速度所需的倾角”也无从算起；它只能选择与模型自洽的水平姿态。若将来解开加速度掩码，参考族就要换成带加速度的形式，`u_ref` 也要按参考的导数重算，式 (8-25) 不再对当前形式成立。

这与第 10 章姿态图的目标线是同一件事。第 10 章将画姿态误差；按这里写定的约定，roll 与 pitch 的参考线取 0，因为参考姿态是水平，偏航的参考线是该段恒定参考航向，误差目标同样是 0。图上的 0 不是“实测姿态恰好为 0”，而是“参考姿态的水平分量与当前航向就是比较基准”。

<big>**一句话总结：无阻力模型里匀速平动不需要倾角，所以参考姿态是水平加纯偏航。**</big>

---

## 8.10 模型失配与鲁棒性

### 8.10.1 参数失配的证据现状

讨论失配前必须先说证据的有效期。[`docs/control/mismatch-sweep.md`](../../control/mismatch-sweep.md) 页首有一条 ☠️：该页全部读数出自 2026-10-09 之前的旧同构被控对象，旧机型常数 `SIH_T_MAX = 2.95 N`（总 11.8 N），重搭后 `T_MAX_TOTAL = 46.3027 N`。因此那张 13 场扫描表、§3–§4 的结论以及“等效刚度 $k=3.57$ N/m”全部作废。作废的是读数，不是方法：五个旋钮、夹具与判据层的设计仍然有效。

重搭之后，目前只有一场质量失配的证据：[`tools/px4-sitl/evidence/tier-b/20261009-rebuild/T0-summary.json`](../../../tools/px4-sitl/evidence/tier-b/20261009-rebuild/T0-summary.json)。被控对象质量 0.975 kg（比控制器模型高 30%），悬停推力 9.56475 N；控制器模型仍是 0.75 kg、`hover_thrust=0.1589`。判定结果：

- 判据①稳态窗口内，水平误差均值 0.0107 m、最大 0.0169 m；垂直误差均值 $-0.00013$ m、绝对均值 0.00035 m。
- 倾角最大 15.46°（起飞瞬态），稳态最大 0.55°。
- 求解稳态 p50 2.931 ms、最坏 5.932 ms；退化 0、求解失败 0、兜底保持 0、安全悬停 0。
- 全程未发散。

这一场说明：质量加三成，在这条轨迹、这个控制器参数、开启垂直偏置积分的条件下，稳态误差仍远小于 0.15 m 的判据界。它**不能**外推成“所有 ±30% 参数都鲁棒”：推力 −30%、惯量、内环时间常数、角速率增益都没有重搭后的读数；旧机型上的读数已随页首 ☠️ 作废，不再引用其结论。本章因此只写这一场的结论，不复制旧表的“只有 mass 与 thrust 过线”那句话。

<big>**一句话总结：代码通过质量加三成误差小于判据，来判断参数是否失配。**</big>

### 8.10.2 结构假设与参数误差不是一回事

表 8-8 把“模型结构”和“模型参数”分开。参数失配扫描只能扫右列，扫不到左列。

**表 8-8　结构假设与参数误差。**

| 类别 | 具体内容 | 能不能靠 ±30% 扫描发现 |
|---|---|---|
| 结构假设 | 内环瞬时、无阻力、推力沿 +z、无执行器动态、无电池电压跌落 | 不能 |
| 参数误差 | 质量、推力标度、惯量、角速率环增益、延迟 | 部分能；旧扫描覆盖过，重搭后只重跑过质量 |

“内环理想”是最容易被误当成参数误差的一条。模型把 $\boldsymbol\omega$ 当实际角速率，真实系统里它是指令，PX4 的角速率环有带宽、延迟与饱和。把 `rate` 旋钮调 ±30% 只能探“内环增益变了以后闭环品质如何”，不能回答“模型少了一个状态”这个问题；后者的修法是十三状态模型加 $k_\omega$，不是把某个参数调准。

工程上的鲁棒性来自三处：反馈本身、1 s 的短时域、以及保守的 $f_{\max}$。短时域让模型误差来不及把预测推得很远；反馈每 10 ms 修正一次；$f_{\max}$ 按电池而不是电机取值，避免规划出兑现不了的推力。这些是设计上的余量，不是形式化的鲁棒性保证；本章不写“对 ±30% 参数鲁棒”这种没有边界的话。

<big>**一句话总结：参数失配扫描探不到结构误差，内环理想假设需要十三状态模型才能显式建模。**</big>

---

## 8.11 求解失败兜底

### 8.11.1 “可用”与“退化”的判据

求解器返回后，`solution_usable` 先判这一拍能不能发（`node_core.py:516-537`）：状态码成功、解形状为 $(4,)$、四个分量全有限；任一不满足即不可用。

“退化”与“不可用”分开：状态码非零、$u$ 越界、代价或耗时非有限而解本身仍有限时，节点照样发、打警告、计数。合并两者，会让一次数值噪声触发不必要的姿态切换。

发布前再由 `frame_is_finite` 检查 `body_rate_x/y/z` 与 `thrust` 四个数（`node_core.py:490-514`）。顺序是 `solution_usable` → 垂直偏置 → `sign_map` 入参检查 → 整帧校验（`mpc_node.py:1206-1250`）。PX4 对角速率有 NaN 回退、对推力没有，最后一道不能省。

<big>**一句话总结：求解状态码异常、形状不对或含非有限值时判为不可用，不进入发布。**</big>

### 8.11.2 兜底阶梯

求解不可用时走三档阶梯（`FallbackLadder`，`node_core.py:1438-1526`）：解可用就发解；连续不可用不超过 10 拍，保持上一帧可行指令、只换时间戳；超过 10 拍，切悬停推力加零角速率并置事件标志。

不能一路保持：PX4 会无限期锁存最后一条 setpoint（`MulticopterRateControl.cpp:177-184`），“我们不发”与“飞控沿用旧帧”是同一件事，陈旧指令可能正打满角速率。也不能一失败就悬停：10 拍在 100 Hz 下约 100 ms，够跨过单次超时，又不会让旧指令持续太久（`FALLBACK_HOLD_FRAMES=10`，`node_core.py:135`）。安全悬停不依赖状态；第一拍就没有“上一帧”时，`can_hold=False` 直接进悬停。

阶梯只处理“求解不可用”，不处理“闸门关闭”：未解锁、不在 OFFBOARD、状态或测量超时，节点不发帧、阶梯复位，PX4 在 `COM_OF_LOSS_T=1.0 s` 后自行退出 Offboard（`docs/assets/z0.params:290`）。不抛异常是刻意的：进程崩掉后 PX4 仍锁存最后一条指令，切到已知安全指令比崩溃更可控。

<big>**一句话总结：不可用时先保持上一帧可行指令十拍，再切到不依赖状态的安全悬停。**</big>

---

## 8.12 已实现与规划中

表 8-9 把本章涉及的机制按“已实现”与“规划中”分开。这张表是本章的边界声明：正文里的公式只描述左列。

**表 8-9　已实现与规划中的机制。**

| 机制 | 状态 | 证据 / 规划出处 |
|---|---|---|
| 十状态模型 $[\mathbf p,\mathbf v,q]$、输入 $[F,\boldsymbol\omega]$ | 已实现 | `model.py:49-51`、`162-191` |
| 四元数运动学右乘约定 | 已实现 | `model.py:54-65`、`182` |
| 50 步、1 s、20 ms 离散 | 已实现 | `ocp.py:57-58`、`424-427` |
| `NONLINEAR_LS` 代价与三组权重 | 已实现 | `ocp.py:64-87`、`255-263` |
| 推力 $[0,f_{\max}]$ 与角速率盒约束 | 已实现 | `ocp.py:272-278` |
| 悬停前馈 $\mathbf u_{\mathrm{ref}}=[mg,0,0,0]$ | 已实现 | `node_core.py:713-776` |
| 发布前垂直偏置积分 | 已实现 | `mpc_node.py:1094-1120`、`1241` |
| 求解失败三档兜底 | 已实现 | `node_core.py:1438-1526` |
| 十三状态模型与 $\dot{\boldsymbol\omega}=k_\omega(\boldsymbol\omega_{\mathrm{des}}-\boldsymbol\omega)$ | **未实现** | [`docs/control/mpc-roadmap.md`](../../control/mpc-roadmap.md) §4.2；[`src/nmpc/`](../../../src/nmpc) 无 `k_omega` 代码（仅 `model.py:36` 的方向声明）；$k_\omega$ 未辨识 |
| 45° 倾角约束 | **未实现** | [`docs/control/mpc-roadmap.md`](../../control/mpc-roadmap.md) §4.3；`ocp.py` 只有 `lbu/ubu` |
| INDI 内环 | **未实现** | [`docs/control/mpc-roadmap.md`](../../control/mpc-roadmap.md) §3.1；[`src/nmpc/`](../../../src/nmpc) 无 `INDI` 实现 |
| 四元数单位长度约束 | **未实现** | 模型无该约束；漂移作精度指标 |
| 加速度参考与前馈导数 | **未实现** | `/traj/reference` 的加速度掩码被忽略 |
| 位置、速度硬约束、障碍物约束 | **未实现** | `ocp.py` 无状态约束 |

十三状态版本的公式在路线图 §4.2 里已经写出，但它不是现状：$k_\omega$ 需要给角速率阶跃、看 IMU 响应来辨识，这一步没有做。倾角约束同理，45° 只是规划值，当前 OCP 不会阻止模型规划更大的倾角。把这两项写成“已实现”，会让第 10 章的误差解释失真。

<big>**一句话总结：已实现的是十状态模型与前馈参考，十三状态、倾角约束与 INDI 是未来可拓展工作。**</big>

---

## 8.13 本章引用清单

表 8-10 把本章的公式、常数与判据按主题归到仓库内的文件与论文。正文里已经随行给过 `file:line`，这里只是便于回查。

**表 8-10　本章引用清单。**

| 主题 | 文件 / 论文 |
|---|---|
| 十状态动力学、四元数工具、RK4 | [`src/nmpc/nmpc/model.py`](../../../src/nmpc/nmpc/model.py) |
| OCP 装配、代价、约束、求解器配置 | [`src/nmpc/nmpc/ocp.py`](../../../src/nmpc/nmpc/ocp.py) |
| 质量、悬停推力、归一化常数、电池上限 | [`src/nmpc/nmpc/params.py`](../../../src/nmpc/nmpc/params.py) |
| 参考生成、前馈、初值外推、兜底阶梯 | [`src/nmpc/nmpc/node_core.py`](../../../src/nmpc/nmpc/node_core.py) |
| 发布路径、垂直偏置、消息字段 | [`src/nmpc/nmpc/mpc_node.py`](../../../src/nmpc/nmpc/mpc_node.py) |
| FLU/FRD 映射与推力符号 | [`src/nmpc/nmpc/sign_map.py`](../../../src/nmpc/nmpc/sign_map.py)、[`docs/control/sign-map.md`](../../control/sign-map.md) §5 |
| 离线求解器行为、RTI 首帧与代价读数 | [`docs/control/offline-ocp.md`](../../control/offline-ocp.md) §2、§4 |
| 规划中的十三状态、倾角约束、INDI | [`docs/control/mpc-roadmap.md`](../../control/mpc-roadmap.md) §3–4 |
| 十状态来源与推力上限推导 | [`docs/control/phase2-implementation.md`](../../control/phase2-implementation.md) §2.1、§3.3 |
| 偏航镜像井与权重讨论 | [`phase2 票 16`](../../../.scratch/phase2/issues/16-ocp-yaw-weights-mirror-well.md) |
| 失配扫描的作废声明与方法 | [`docs/control/mismatch-sweep.md`](../../control/mismatch-sweep.md) 页首、§1.2 |
| 质量加三成的重搭后证据 | [`T0-summary.json`](../../../tools/px4-sitl/evidence/tier-b/20261009-rebuild/T0-summary.json) |
| 三趟仿真的求解耗时 | [`R-hover`](../../../tools/report/evidence/sitl/R-hover/solver-summary.json)、[`R-square`](../../../tools/report/evidence/sitl/R-square/solver-summary.json)、[`R-lemniscate`](../../../tools/report/evidence/sitl/R-lemniscate/solver-summary.json) |
| 四旋翼动力学与 (C) 级分层 | `Sun2022` §III.A、`Foehn2022` §D.1 |
| 实时迭代 | `Houska2011`（Automatica 2011） |

<big>**一句话总结：本章的公式、权重与判据都能回到源码、证据文件或论文。**</big>
