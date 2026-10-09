#!/usr/bin/env python3
"""把 `evidence/*.json` 汇成表 —— **票面结论里的每个数都从数据现算**，不手抄。

为什么要有这个脚本：结论里那些数（可持续电流、悬停功率、外推电流、占额定比例）
一旦手抄进散文，就会随下一次数据更新而过期，且没人知道它们是从哪一版 JSON 来的。
这里全部现算，并把**口径**与**源文件指纹**一起打出来。

跑法（在仓库根）::

    python3 tools/battery-thrust/evidence/summarise.py

另外做两件事：
1. 用 `bag/MANIFEST.md` 里的 sha256 核对每份 ulog 的指纹 ——
   证明工具读的就是清单登记的那份字节，不是"同名的另一份"；
2. 按 `P = V·I` 把悬停工作点投影到**实用截止 18.0 V**（票面问题 ① 要的"低电量端"）——
   ⚠️ 这是**投影**，不是测量：归档里最低的一段也只到 ~19.3 V，见输出末尾的"不许当测量"一节。
"""

from __future__ import annotations

import glob
import json
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", "..", ".."))
V_CUTOFF = 18.0  # hardware.md §6.1 的"实用截止"


def load_all():
    out = []
    for p in sorted(glob.glob(os.path.join(HERE, "*.json"))):
        with open(p, encoding="utf-8") as f:
            out.append((os.path.basename(p), json.load(f)))
    return out


def scene(rep):
    """从路径里取 `<场景>`（`bag/<场景>/fcu/*.ulg` 或 `bag/old/<场景>/fcu/*.ulg`）。"""
    parts = rep["file"].split(os.sep)
    return parts[-3] if parts[-4] != "old" else "old/" + parts[-3]


def check_manifest(reps):
    """用 `bag/MANIFEST.md` 登记的 sha256 核对工具算出来的 —— 对不上就是读错了文件。"""
    p = os.path.join(ROOT, "bag", "MANIFEST.md")
    if not os.path.exists(p):
        print("⚠️ 找不到 bag/MANIFEST.md，跳过指纹核对")
        return
    text = open(p, encoding="utf-8").read()
    table = dict(re.findall(r"`(bag/[^`]+\.ulg)` \| [\d.]+ MB \| `([0-9a-f]{64})`", text))
    checked = bad = 0
    for name, rep in reps:
        rel = os.path.relpath(rep["file"], ROOT)
        if rel not in table:
            continue  # old/ 里的不在清单里（清单只登记当前该用的那一份）
        checked += 1
        if table[rel] != rep["sha256"]:
            bad += 1
            print(f"❌ 指纹不符 {rel}\n   清单 {table[rel]}\n   实算 {rep['sha256']}")
    print(f"指纹核对：清单里登记的 {checked} 份，逐字节一致"
          + ("（全对）" if not bad else f"，**{bad} 份不符**"))


def row(name, rep):
    if rep.get("error"):
        return [scene(rep), "—", "—", "—", "—", "—", "—", "—", "—", "—",
                "无在飞段" if "在飞段" in rep["error"] else rep["error"][:24]]
    fw = rep["flight_window"]
    s = rep["current"]["sustained"]
    lh = (rep["current"].get("sustained_last_half") or {}).get("sustained_a", {})
    a = rep.get("hover_anchor")
    e = rep.get("extrapolation")
    v = rep.get("verdict", {})
    sig = rep.get("signature", {})
    return [
        scene(rep),
        f"{fw['span_s']:.0f}",
        f"{rep['voltage']['median_v']:.2f}",
        f"{s['value']:.2f}" if s["value"] is not None else "—",
        # ★ 门票面点名排除的那一列：全段最大落在在飞窗口起点 = 解锁/起飞那一瞬
        "★起" if s.get("at_flight_start") else "·",
        f"{lh['value']:.2f}" if lh.get("value") is not None else "—",
        f"{s['raw_max']:.2f}",
        f"{a['voltage_v']:.2f} / {a['current_a']:.2f} / {a['power_w']:.0f}" if a else "—",
        f"{e['current_a']:.2f}" if e else "—",
        f"{v['fraction_of_rating']:.0%}" if v.get("fraction_of_rating") else "—",
        f"{sig.get('signature_ratio', float('nan')):.1f}×" if sig.get("signature_ratio") else "—",
    ]


def main() -> int:
    reps = load_all()
    if not reps:
        print("没有 JSON —— 先按 README 跑一遍 battery_from_ulog.py --json")
        return 1

    print("## 逐份日志")
    cols = ["场景", "在飞s", "V中位", "全段maxA", "起点", "放电末A", "单点max",
            "锚点 V/I/W", "f_max处A", "占额定", "印记比"]
    # 按场景名排（JSON 文件名是 sha 前缀，按它排是乱的）——`old/` 排在后面
    rows = sorted((row(n, r) for n, r in reps), key=lambda r: (r[0].startswith("old/"), r[0]))
    width = [max(len(c), *(len(r[i]) for r in rows)) for i, c in enumerate(cols)]
    print("  " + "  ".join(c.ljust(w) for c, w in zip(cols, width)))
    for r in rows:
        print("  " + "  ".join(x.ljust(w) for x, w in zip(r, width)))
    print("  ★起 = 「全段maxA」取到最大值的那一段落在**在飞窗口起点**（解锁/起飞那一瞬）——"
          "正是票面问 ① 时**点名排除**的那一列，不能当 ① 的答案。")
    print("       问 ① 的「电量低端」口径看**放电末A**那一列。")

    print("\n## 票面问题 ①：低电量端（按 `P = V·I` 把悬停工作点投影到实用截止电压）")
    cand = [(r, n) for n, r in reps if r.get("hover_anchor")]
    if not cand:
        print("  （没有一份日志取到悬停锚点）")
    else:
        # 用**电压最低**的那份当"低电量端"的代理（不是用 remaining：那是电压法且被压降压低）
        r, n = min(cand, key=lambda t: t[0]["hover_anchor"]["voltage_v"])
        a, e = r["hover_anchor"], r["extrapolation"]
        p_hov, p_max = a["power_w"], e["power_w"]
        print(f"  最低电压的一份 = {scene(r)}（锚点 {a['voltage_v']:.3f} V，悬停 {p_hov:.1f} W = {a['current_a']:.3f} A）")
        print(f"  同一 log 的电压范围：{r['voltage']['min_v']:.2f}–{r['voltage']['max_v']:.2f} V"
              f"（最低单点 {r['voltage']['min_v']:.2f} V = {r['voltage']['min_v'] / 6:.2f} V/节；截止 = {V_CUTOFF:.1f} V = 3.00 V/节）")
        print()
        print("  | 母线电压 | 悬停所需电流 | f_max 所需电流 | 占额定 15 A |")
        print("  |---|---|---|---|")
        for vbus, tag in ((a["voltage_v"], "锚点实测"),
                          (r["voltage"]["min_v"], "本份最低单点"),
                          (V_CUTOFF, "**实用截止（投影）**")):
            i_h = p_hov / vbus
            i_f = p_max / vbus
            print(f"  | {vbus:.2f} V（{tag}） | {i_h:.2f} A | {i_f:.2f} A | {i_f / 15.0:.0%} |")
        print(f"\n  ⚠️ 18 V 那一行是**投影**：归档里没有任何一段飞到 18 V，"
              f"所以它是「若电压掉到截止，按实测功率反推要抽多少电流」，")
        print(f"     既没有包含低 SOC 时内阻上升，也没包含电压崩塌 —— **不是测量**。")

    print("\n## 不许当测量")
    for line in (
        "· 归档里**没有一段接近实用截止电压**（最低单点 "
        f"{min(r['voltage']['min_v'] for n, r in reps if r.get('voltage')):.2f} V），"
        "所以「低电量端能不能顶住」在归档里**没被触发过**。",
        "· 所有以安培为单位的数都继承 `BAT1_A_PER_V = 17.0` 这个**未经日志验证**的标度；",
        "  而 `f_max = 11.8 N` 本身就是从 15 A 反推出来的 ⟹ 「f_max 保不保守」与这个标度**同生共死**。",
    ):
        print(line)

    print()
    check_manifest(reps)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
