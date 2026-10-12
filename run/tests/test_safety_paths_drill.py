"""`docs/control/safety-paths-drill.md`（安全路径演练流程，ticket 08）的守卫。

跑法：

    python3 -m pytest run/tests/test_safety_paths_drill.py -q

★ **为什么这一页要单开一份守卫**：这一页的**全部价值**就是"现场照着它敲"，
而它自己**一次都没跑过**（口径 §0 第 1 条白纸黑字写着）。没跑过的文档只有两类东西
能出腐坏，而这两类**肉眼复核都抓不住**：

| 文档里写的 | 钉在哪 |
|---|---|
| 参数读数（`COM_OF_LOSS_T` = `1.0` 这一族） | `docs/assets/z0.params` 的**现读值**（改快照 ⟹ 这一页变红） |
| 命令里的入口点（`run/z0.py <key>` / `ros2 run nmpc <x>.py` / `run/*.sh`） | `z0.ACTION_BY_KEY` 与文件系统 |
| 相对链接 | 文件系统（别页改名时这一页不该静默烂掉） |
| 术语（禁裸词 `MPC`） | `CONTEXT.md` 那句书写约定 |

★★ **有意只扫这一份文档**（不是 `docs/**`）：本仓有并行会话，别人的在制品混在同一个工作树里，
扫全 `docs/` 会让**他们**改文档时红在**我们**这里（同 `test_operation_manual.py` 的 ★★）。

★ **约定即判据**：这一页里参数读数只有两种形状 —— 表格行 `` | `名` | `数` | `` 与
散文里的 `` `名` = `数` ``（等号两边各一个空格）。**两种形状必须都留**：只认一种，
另一种就是"守卫看不见的自由散文"。改这一页时**别把等号写成别的样子**，
写成别的样子不会报错，只会**静默离开守卫的视野** —— 把这件事变成会红的断言的是
`test_读数不会静默离开守卫的视野`（★ **不是** `test_约定本身认得出来`：那一条只在
**整种形状消失**时才红，单条读数漂移它抓不到）。
"""

import pathlib
import re
import sys

RUN_DIR = pathlib.Path(__file__).resolve().parents[1]
REPO = RUN_DIR.parent
sys.path.insert(0, str(RUN_DIR))

import z0  # noqa: E402  （路径注入了才 import 得到）

DOC = REPO / "docs" / "control" / "safety-paths-drill.md"
PARAMS = REPO / "docs" / "assets" / "z0.params"
NMPC_PKG = REPO / "src" / "nmpc" / "nmpc"

#: 参数读数的**两种**形状（见模块 docstring 的 ★ 约定即判据）。
RE_TABLE_READING = re.compile(r"\|\s*`([A-Z][A-Z0-9_]*)`\s*\|\s*`([0-9.]+)`\s*\|")
RE_PROSE_READING = re.compile(r"`([A-Z][A-Z0-9_]*)`\s=\s`([0-9.]+)`")

RE_NMPC_ENTRY = re.compile(r"ros2 run nmpc ([\w.]+\.py)")
#: ⚠️ 大小写也要认进来（`[A-Za-z_]+`）：写成 `run/z0.py Check` 时若只认小写，
#: 那个词**根本不会被取出来**，也就无从查表 ⟹ 静默逃逸（复核时的变异测试抓到的）。
RE_Z0_ACTION = re.compile(r"run/z0\.py ([A-Za-z_]+)")
RE_RUN_SH = re.compile(r"run/([\w.-]+\.sh)")
RE_LINK = re.compile(r"\[[^\]]*\]\(([^)]+)\)")

#: `CONTEXT.md` 的全仓清扫命令（逐字搬，不自创）。
RE_BARE_MPC = re.compile(r"(?<![A-Za-z_/\-])MPC(?![A-Za-z_（\-\d])")


def _doc() -> str:
    assert DOC.exists(), f"演练流程那一页不在：{DOC}"
    return DOC.read_text(encoding="utf-8")


def _snapshot() -> dict:
    """`docs/assets/z0.params` 的 `{参数名: 现读值字符串}`。

    ⚠️ 显式 `encoding="utf-8"` + `split("\\n")`：该文件头的说明里写着两件事 ——
    `★` 的 UTF-8 末字节是 `0x85`（按 latin-1 解再 `splitlines()` 会凭空多切几行），
    以及**行号会随每次重导出整体平移** ⟹ 这里按**参数名**取值，不碰行号。
    """
    text = PARAMS.read_text(encoding="utf-8")
    out = {}
    for line in text.split("\n"):
        if line.startswith("#"):
            continue
        parts = line.split("\t")
        if len(parts) >= 5 and parts[0].strip().isdigit():
            out[parts[2]] = parts[3]
    return out


def _readings(text: str):
    """文档里声明的参数读数：`[(行号, 参数名, 文档里写的数, 形状)]`。"""
    found = []
    for shape, pat in (("表格", RE_TABLE_READING), ("散文", RE_PROSE_READING)):
        for m in pat.finditer(text):
            found.append((text[:m.start()].count("\n") + 1, m.group(1), m.group(2), shape))
    return found


#: ★ **"差一点就是读数"**的三种形状 —— 用来找"看着像读数、却没被上面两条约定形状认出"的地方。
#: 每种都只取**参数名**（值那一格故意放宽：值写坏了正是要被抓的情形）。
#:
#: 第一手（复核时的变异测试）：只盯"两边都带反引号 + 值是数字"是不够的 ——
#: 下面三种漂移**全都不命中**任何一种约定形状，于是静默离开快照的视野：
#:   · 值不带反引号：`` `名` = 1.0 ``
#:   · 值非数字：`` `名` = `一秒` ``
#:   · 值那一格裸写（表格）：`` | `名` | 1.0 | ``
RE_NEAR_PROSE = re.compile(r"`([A-Z][A-Z0-9_]*)`\s*=\s*`?([^\s`）)、，。]+)`?")
RE_NEAR_TABLE = re.compile(r"\|\s*`([A-Z][A-Z0-9_]*)`\s*\|\s*`?([^\s`|]+)`?\s*\|")
RE_BARE_NAME_EQ = re.compile(r"(?<![A-Za-z0-9_`])([A-Z][A-Z0-9_]*)\s*=\s*`?([0-9.]+)`?")


def _strip_readings(line: str) -> str:
    """把这一行里**已经被认出来**的读数抠掉，剩下的才是"没被认出的那部分"。

    ⚠️ 按行判"整行认没认出来"是不够的：一行里可以**前半句是合规读数、后半句是漂移读数**
    （本页 §6 那张登记表就是这种行）—— 整行判会放过后半句。
    """
    for pat in (RE_TABLE_READING, RE_PROSE_READING):
        line = pat.sub(" ", line)
    return line


# ─────────────────────────── 参数读数 ───────────────────────────


def test_约定本身认得出来():
    """★ 守卫的**自我**判据：两种形状各自至少一条命中。

    没有这一条，把等号写错（或整张表换成别的排版）会让 `test_参数读数与快照一致` **静默变空**
    —— 那时它仍然"全绿"，而它盯的东西一个都不在了。
    ⚠️ 它只盯"**这一种形状整体消失**"；单条读数改了标点不至于把它清空 ⟹ 那种漂移由
    `test_读数不会静默离开守卫的视野` 兜（★ 兜它的**是那一条**，不是这一条 —— 这一条只在
    整种形状消失时才红）。
    """
    shapes = {shape for _, _, _, shape in _readings(_doc())}
    assert shapes == {"表格", "散文"}, (
        f"两种读数形状至少要各命中一条，实际只认得：{sorted(shapes)}。"
        "改这一页的排版时请照模块 docstring 的 ★ 约定（别把 `名` = `数` 写成别的样子）。")


def test_读数不会静默离开守卫的视野():
    """★ "差一点就是一条读数"、却没被约定形状认出 ⟹ 红。

    这是 `test_约定本身认得出来` **盖不住**的那一半：那种漂移不会让整种形状消失，
    只是把**这一条**挪到视野外（第一手：`` `X` = `0` `` 改成 `` `X` is `0` `` 时，那条照样全绿）。
    三种漂移形状见 `RE_NEAR_PROSE` / `RE_NEAR_TABLE` / `RE_BARE_NAME_EQ` 上面的注释；
    变红的修法就是把那一处写回 `名` = `数`（或表格 `` | `名` | `数` | ``）。

    ⚠️ **只判三种**，不做"凡是出现参数名就必须是读数"的宽判据 —— 本页大量正文**只引用参数名、
    不写值**（值回链别页），那种写法是合规的：不写值 = 不宣称读数。
    """
    snap = _snapshot()
    bad = []
    for i, line in enumerate(_doc().split("\n"), start=1):
        rest = _strip_readings(line)
        for pat in (RE_NEAR_PROSE, RE_NEAR_TABLE, RE_BARE_NAME_EQ):
            for name, value in pat.findall(rest):
                if name in snap:
                    bad.append(f"第 {i} 行：{line.strip()}    ← `{name}` = `{value}` 不是读数形状")
    assert not bad, (
        "这些地方引了快照里的参数、数值也写出来了，但**不在两种约定形状里**"
        "（表格 `| `名` | `数` |`，或散文 `` `名` = `数` ``）⟹ 它们**没有被快照钉住**：\n  "
        + "\n  ".join(bad))


def test_读数表每一行都是读数形状():
    """§2.0 那张表**被定义为**"本页唯一一处参数读数" ⟹ 它的每一行都必须是一条读数。

    兜的是上面两条都够不着的形状：把某一格的值写成非数字（`` | `NAV_RCL_ACT` | `一` | ``）
    —— 那一行在两种约定形状下**都不命中**，于是一声不响地离开了快照。
    """
    text = _doc()
    lines = text.split("\n")
    start = next((i for i, l in enumerate(lines) if l.startswith("### 2.0")), None)
    assert start is not None, "找不到 §2.0（那一段的标题是读数表的锚）"
    end = next((i for i in range(start + 1, len(lines)) if lines[i].startswith("### ")), len(lines))
    bad = []
    for i in range(start, end):
        line = lines[i].strip()
        if not line.startswith("|"):
            continue
        cells = [c.strip() for c in line.strip("|").split("|")]
        if cells[0] in ("参数", "---") or set(cells[0]) <= {"-"}:
            continue                      # 表头 / 分隔行
        if not RE_TABLE_READING.search(line):
            bad.append(f"第 {i + 1} 行：{line}")
    assert not bad, (
        "§2.0 的这几行不是读数形状（`| `名` | `数` | …`）⟹ 它们没有被快照钉住：\n  "
        + "\n  ".join(bad))


def test_参数读数与快照一致():
    """★ 这一页引的每一个读数都必须等于 `docs/assets/z0.params` 里的现读值。

    鉴别力来自"**同一个数两个出处**"：页面里的值是**手抄**的，快照是**导出**的。
    快照一换（现场重导出参数），对不上的那几条当场变红 —— 那正是这一页该被复核的时刻。
    """
    snap = _snapshot()
    bad = []
    for lineno, name, value, shape in _readings(_doc()):
        if name not in snap:
            bad.append(f"第 {lineno} 行（{shape}）：`{name}` 不在这份快照里 —— "
                       "要么参数名写错，要么这一页引了一个快照没有的量")
        elif float(snap[name]) != float(value):
            bad.append(f"第 {lineno} 行（{shape}）：`{name}` 文档里写 `{value}`，"
                       f"快照里是 `{snap[name]}`")
    assert not bad, "参数读数与 docs/assets/z0.params 对不上：\n  " + "\n  ".join(bad)


# ─────────────────────────── 命令里的入口点 ───────────────────────────


def test_nmpc_的入口脚本都存在():
    """`ros2 run nmpc X.py` ⟹ `src/nmpc/nmpc/X.py` 必须真的在。

    这一页教人"怎么把它摆到台上"，而它自己没起过 ⟹ 脚本名写错**现场才会发现**，
    且症状是"命令找不到"，与"包没 build / 没 source"分不开（本仓踩过这一类）。
    """
    missing = [name for name in set(RE_NMPC_ENTRY.findall(_doc()))
               if not (NMPC_PKG / name).is_file()]
    assert not missing, f"这些入口脚本在 src/nmpc/nmpc/ 下不存在：{sorted(missing)}"


def test_run_z0_的动作名都在动作表里():
    """`run/z0.py <key>` ⟹ key 必须是 `z0.ACTION_BY_KEY` 里真实存在的那一个。

    ★ 这是 `test_operation_manual.py` 那条教训的同族（菜单号写成 9 会起到操作台）：
    "文档引用代码里的名字"这一类腐坏**肉眼复核抓不住**（名字看着都对）。
    """
    known = set(z0.ACTION_BY_KEY)
    unknown = sorted({key for key in RE_Z0_ACTION.findall(_doc()) if key not in known})
    assert not unknown, (
        f"`run/z0.py <key>` 引了动作表里没有的名字：{unknown}；"
        f"动作表里有：{sorted(known)}")


#: 菜单引用的**写法是约定**（与 `test_operation_manual.py` 同一把尺）：`菜单 **N= 标题**`。
RE_MENU_REF = re.compile(r"菜单\s*\*\*(\d+)=\s*([^*]+?)\*\*")


def _normalize(text: str) -> str:
    """去掉加粗星号与多余空白 —— 只用来比对，不改文档本身。"""
    return re.sub(r"\s+", " ", text.replace("*", " ")).strip()


def test_菜单引用与动作表对得上():
    """`菜单 **N= 标题**` ⟹ 第 N 项必须是 `z0.ACTIONS[N-1]`，且标题照抄它。

    ★ 与 [`test_operation_manual.py`](test_operation_manual.py) **同一个坑、同一把尺**：
    "文档引用代码里的编号"这一类腐坏**肉眼复核抓不住**（`2= 起链核对` 看着就对，
    而它指向的是**第 N 项**这个位置）。反过来，动作表一重排，这一页立刻变红。

    ⚠️ 与那边一样：**只有这一种写法在守卫视野里**。散文里裸写"菜单 2"管不着 ⟹
    这一页里凡是要指菜单，就写成这种形状。

    ★ **查的是 `MENU_ACTIONS`，不是 `ACTIONS`**：`run/z0.py` 的 `:426` 自己写着
    「菜单号 = `MENU_ACTIONS` 的顺序」（`menu=False` 的动作不进菜单、不占号）。
    今天两者前 11 项恰好一致（5 个 `menu=False` 都排在末尾），所以用 `ACTIONS` 也**碰巧**对 ——
    但哪天有人把一条不进菜单的动作插到前面，用 `ACTIONS` 就会**静默查错行**。
    （同族的 `test_operation_manual.py` 用的是 `z0.ACTIONS` —— 那是另一张票的产物，**不在本票范围内改**。）
    """
    refs = [(int(n), title) for n, title in RE_MENU_REF.findall(_doc())]
    assert len(refs) >= 2, f"这一页至少要显式点名菜单项（只认出 {len(refs)} 条）"
    for number, cited in refs:
        assert 1 <= number <= len(z0.MENU_ACTIONS), f"菜单里没有第 {number} 项"
        title = _normalize(z0.MENU_ACTIONS[number - 1].title).lstrip("★ ").strip()
        assert title.startswith(_normalize(cited).lstrip("★ ").strip()), (
            f"菜单 **{number}= {cited}** 与动作表对不上："
            f"第 {number} 项是「{z0.MENU_ACTIONS[number - 1].title}」")


def test_run_下的脚本都存在():
    """`run/*.sh` 必须真的在（两个一键脚本是这一页里最常被照着敲的两条）。"""
    missing = [name for name in set(RE_RUN_SH.findall(_doc()))
               if not (RUN_DIR / name).is_file()]
    assert not missing, f"run/ 下没有这些脚本：{sorted(missing)}"


# ─────────────────────────── 链接与术语 ───────────────────────────


def test_相对链接都解得到():
    """文档里的相对链接逐个落到真实文件上（别页改名 / 挪窝时这一页不该静默烂掉）。

    跳过外链（`http`）与纯锚点；`#` 之后的部分不算路径。
    """
    bad = []
    for raw in RE_LINK.findall(_doc()):
        target = raw.split("#", 1)[0].strip()
        if not target or target.startswith(("http://", "https://", "mailto:")):
            continue
        if not (DOC.parent / target).exists():
            bad.append(f"{raw} ⟹ 解不到 {(DOC.parent / target)}")
    assert not bad, "相对链接解不到：\n  " + "\n  ".join(bad)


def test_不裸用_MPC():
    """★ 术语纪律（`CONTEXT.md` 开头那条）：指代本项目的控制器时**禁止单独用裸词 `MPC`**。

    正则逐字搬自 `CONTEXT.md` 的全仓清扫命令；`MPC（模型预测控制）` / `MPC_*` 参数名
    都不算命中（否定环视已经把这两种排除掉了）。
    """
    hits = [(i + 1, line) for i, line in enumerate(_doc().split("\n"))
            if RE_BARE_MPC.search(line)]
    assert not hits, "裸用 `MPC`（要写成 `MPC（模型预测控制）` 或全称）：\n  " + "\n  ".join(
        f"第 {n} 行：{line.strip()}" for n, line in hits)
