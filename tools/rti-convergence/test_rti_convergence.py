"""`rti_convergence.py` 的判据 —— 钉住 ticket 16 收回 `R_yaw = 50` 的**那条区别**。

本文件钉四件事，前两件是裁定的支点：

1. **10 m 不可行步进 + `R_yaw = 5e2`（落地值）⟹ 第 50 拍就已收敛**；
2. **同一场景 + `R_yaw = 50`（已撤回）⟹ 在节点实际跑的那段窗口里还在大幅摆动**；
3. **收敛边界落在 `50` 与 `55` 之间**（`ocp.py` 引用这句 ⟹ 它得有人守）；
4. **`Weights` 的默认与 `OcpConfig` 的默认不漂**（它是手抄的；漂了整张表量的就是别的东西，且全绿）。

★ 两条是**同一条用例的一对**：单看第 2 条只能说"50 很慢"，配上第 1 条才说明
"换回 5e2 是有效的"；反过来只有第 1 条则说明不了"当初为什么要收回 50"。

⚠️ 本文件的教训写在被测模块的 docstring 里：**判"不收敛"必须先跑够**。初版探针只跑 60 拍，
把 `R_yaw = 50` 的慢收敛读成了"没有不动点"，还顺带把 `R_yaw = 5e2` 在 5–7 m 的**瞬态**
读成了发散 —— 两条都随 400 拍重跑而翻。所以这里第 1 条的拍数**不能调小**。

跑法：

    source install/setup.bash
    export ACADOS_SOURCE_DIR=$HOME/acados LD_LIBRARY_PATH=$ACADOS_SOURCE_DIR/lib:$LD_LIBRARY_PATH
    python3 -m pytest tools/rti-convergence -q
"""

from __future__ import annotations

import numpy as np
import pytest

pytest.importorskip("nmpc")
pytest.importorskip("acados_template")

import rti_convergence as R  # noqa: E402  （同目录，pytest 会把它放进 sys.path）

#: 黑盒那条用例探的目标偏移 —— 1.0 s 时域内走 10 m 是**不可行步进**（正是要压的工况）。
OFF_10M = (10.0, 0.0, 0.0)


@pytest.fixture(scope="module")
def u_landed() -> np.ndarray:
    """落地权重（`R_yaw = 5e2`）下连解 400 拍。"""
    return R.iterate(OFF_10M, weights=R.Weights(r_yaw=5e2))


@pytest.fixture(scope="module")
def u_withdrawn() -> np.ndarray:
    """已撤回的 `R_yaw = 50`（"方案 E"）下连解 400 拍。"""
    return R.iterate(OFF_10M, weights=R.Weights(r_yaw=50.0))


def test_落地权重在_10_m_不可行步进下已收敛(u_landed):
    """★ 支点 ①：`R_yaw = 5e2` 下这条迭代**精确**到不动点。

    实测（2026-09-30，`qw=0.1, qatt=2.0, R_yaw=5e2`）：`|Δu|` 第 50 拍就是 `0.00000`，
    末拍 `ω_y = +0.000841`、`ω_z = −0.001818`（交叉轴 ≪ 黑盒阈值 `0.2`）。
    ⟹ 黑盒那条 `roll = +0.2032` 的读出**不再出现**。
    """
    d = R.step_sizes(u_landed)
    assert d[50] < 1e-6, f"第 50 拍就该收敛，实测 |Δu| = {d[50]:.6f}"
    assert d[-1] < 1e-6, f"末拍仍未收敛，实测 |Δu| = {d[-1]:.6f}"


def test_已撤回的_50_在同一场景还在摆(u_withdrawn):
    """★ 支点 ②：`R_yaw = 50` 下这条迭代**慢**到"发指令的那段窗口"里全是大幅摆动。

    实测（2026-09-30）：`|Δu|` 拍 50 = `0.8878`、拍 100 = `0.2740`、拍 200 = `0.0268`、
    末拍 = `0.00026` —— **几何衰减**（每 ~100 拍小 10×）⟹ 它**有**不动点，只是要 ~1000 拍。
    节点每拍只做 1 次迭代 ⟹ 头几十拍发出的就是 `ω_z ∈ ±0.85…1.19 rad/s` 的乱跳指令。

    ⚠️ 这条**能红**：若哪天 `R_yaw = 50` 变得"第 60 拍就收敛"，说明有别的机制在压制它，
      那时"收回 50"的理由就得重新论证（本条会先红）。
    """
    d = R.step_sizes(u_withdrawn)
    assert d[50] > 0.5, f"R_yaw=50 的第 50 拍该还在大幅摆动，实测 |Δu| = {d[50]:.5f}"
    assert d[200] < d[50], "它该是**单调衰减**（慢收敛），不是极限环"


def test_两条的判别力来自_R_yaw_本身(u_landed, u_withdrawn):
    """★ 变异检查：两组只差 `R_yaw` 一个数，读数就必须分得开。

    没有这条，上面两条可以同时"因为别的原因"通过（例如两次都在空跑）。
    实测：第 50 拍 `|Δu|` 是 `0.00000` vs `0.88779` —— 差三个数量级以上。
    """
    a = R.step_sizes(u_landed)[50]
    b = R.step_sizes(u_withdrawn)[50]
    assert a < 1e-6 < b, f"两组该判得开，实测 {a:.6f} vs {b:.5f}"


def test_Weights_的默认就等于_OcpConfig_的默认():
    """★ `Weights` 的 `q_diag`/`r_diag` 是**手抄** `OcpConfig` 的排布（本模块不 import 它，
    见 `Weights` 的 docstring）⟹ 它可能**静静地漂**。

    漂的后果是**静默的错答案**：`Weights()` 号称"落地权重"，一旦 `OcpConfig` 的默认改了而这里
    没跟，整张扫描表量的就是**另一个东西**，而所有断言照样绿。所以这条把它们钉在一起
    （不起求解器，纯比较，几乎不花时间）。

    ⚠️ 它变红**不等于**有人改错了 —— 也可能只是"默认权重又动了"，那时**先改 `Weights` 的默认**，
    再重跑 `--out` / `--sweep-out` 刷新两张表，最后才回来看这条。
    """
    from nmpc.ocp import OcpConfig

    d = OcpConfig()
    assert R.Weights().q_diag() == d.q_diag, \
        f"Weights 的 q_diag 与 OcpConfig 默认漂了：{R.Weights().q_diag()} vs {d.q_diag}"
    assert R.Weights().r_diag() == d.r_diag, \
        f"Weights 的 r_diag 与 OcpConfig 默认漂了：{R.Weights().r_diag()} vs {d.r_diag}"


def test_收敛边界落在_50_与_55_之间(u_withdrawn):
    """★ 钉 `ocp.py` 里那句「判据在 50 与 55 之间」—— 剔除它的**那个数**由这条守。

    上一版这句话引用的是一串**只落在对话里、没进仓库**的读数（表 2 落地前的 `rysweep`）。
    现在它由 `evidence/r_yaw_sweep.json` 背书，而这条用例只把**边界那两档**再钉一遍
    （整条曲线跑起来太重，边界才是会被改动的那个数）。

    第一手（2026-09-30，10 m 不可行步进，400 拍）：`50 ⟹ 慢收敛`（末段 `3.3e-4`）、
    `55 ⟹ 已收敛`（中段/末段都是 `0.00000`），再往上 `60…500` 全 `已收敛`
    ⟹ 边界**恰在**这两档之间，不是"差不多在这附近"。
    """
    assert R.classify(u_withdrawn)["verdict"] != "已收敛", \
        "已撤回的 50 该判**不**收敛 —— 它一收敛，'收回 50' 的理由就没了"
    row = R.sweep_r_yaw((55.0,))[0]
    assert row["verdict"] == "已收敛", \
        f"55 该已收敛（边界就在 50 与 55 之间），实测判定 = {row['verdict']}、末段 {row['end']:.5f}"
