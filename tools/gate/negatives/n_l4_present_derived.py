#!/usr/bin/env python3
"""反例 L4（审计 F-L4-1，2026-09-13）—— 工位在场全集少写一个障碍必须红。

以前 motions[].present 完全自报，唯一交叉核对是同一个人写的 required_motion_groups：少写一个已装件，
扫掠对着残缺的障碍集合照样 0 mm³ → "条件绿"。现在层按 assembly.yaml 推导每个动作的**最小**在场集合：
    present ⊇ movers ∪ {早于本位置进入装配、所属子总成 ∈ closure(本动作 subassembly_of) 的实体}
（实体进入装配 = 首次作为 mover，或首次出现在某步 parts 里的台面底件；按动作所在侧过滤。）

经层的 `run_declared_motions`（run() 对每个带 motions 的步调用的就是它；run() 把 asm/names 挂在 res 上，这里照做）：
  · 坏：步 2 的动作 present 漏掉步 1 装上的 a（同子总成）→ step02:present_covers_derived:m2 FAIL(BLOCK)，measured=1
  · 好：present 补上 a → PASS
  · 顺序矛盾：步 1 的 present 里出现步 2 才装的 b → step01:present_only_installed:m1 FAIL(WARN)
  · 坏声明：步没有 subassembly_of / subassembly 不在 subassemblies 里 → unknown（另计）
几何：a、b、base 都是互不相交的小方块（扫掠本身全绿），只有 present 声明在变 —— 证明红的是"少写了障碍"这件事。
"""
from __future__ import annotations
import json
import sys
from pathlib import Path

_HERE = Path(__file__).resolve()
GATE = _HERE.parents[1]
for _p in (str(_HERE.parent), str(GATE), str(GATE.parents[1])):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from _harness import FakeCtx, core, findings, assert_red, assert_green, result, record   # noqa: E402
from negatives.n_l4_split_direction import _write                                        # noqa: E402
import layers.l4_assembly as L4                                                          # noqa: E402

NAME = "工位在场全集 ⊇ 推导集合：漏写已装件红（present_covers_derived）、补上绿、顺序矛盾 WARN、缺子总成声明 unknown"
EXPECT = ("L4/step02:present_covers_derived:m2 FAIL(BLOCK) measured=1；补上 a 后 PASS；"
          "step01:present_only_installed:m1 FAIL(WARN)；无 subassembly_of → unknown")


def _motion(mid, movers, present):
    return dict(id=mid, movers=movers, present=present, workspace="bench",
                sense="withdrawal_from_assembled", outward_world=[0, 0, 1], len_mm=3, step_mm=.5)


def _asm(present2, present1=("a", "base"), sub2="bench", subs=None):
    m1 = _motion("m1", ["a"], list(present1))
    m2 = _motion("m2", ["b"], list(present2))
    s1 = {"step": 1, "id": "step01", "seq": 1, "subassembly_of": "bench", "parts": ["a", "base"],
          "required_motion_groups": {"m1": {k: m1[k] for k in ("movers", "present", "workspace")}}, "motions": [m1]}
    s2 = {"step": 2, "id": "step02", "seq": 2, "parts": ["b", "base"],
          "required_motion_groups": {"m2": {k: m2[k] for k in ("movers", "present", "workspace")}}, "motions": [m2]}
    if sub2:
        s2["subassembly_of"] = sub2
    return {"subassemblies": subs if subs is not None else {"bench": {"members": []}},
            "assembly_order": [s1, s2], "tool_states": {}}


def _run_step2(geo, ctx, asm, tol):
    res = core.LayerResult(4, "neg")
    res._asm, res._names = asm, L4._Names(ctx, geo)
    for st in asm["assembly_order"]:
        L4.run_declared_motions(res, geo, st, tol)
    return res


def run():
    boxes = {"a": ((-3, -.5, 0), (-2, .5, 1)), "b": ((2, -.5, 0), (3, .5, 1)), "base": ((-5, -5, -2), (5, 5, -1))}
    directory = _write(boxes, "present_derived")
    old = L4.PLACED
    try:
        L4.PLACED = directory
        ctx = FakeCtx(core.load_data(), {}, {p.stem: p for p in directory.glob("*.stl")})
        geo = L4._Geo(ctx)
        tol = ctx.data["tolerances"]["feature_check_tolerances"]["static_intersection_mm3"]["max"]

        bad = _run_step2(geo, ctx, _asm(present2=["b", "base"]), tol)               # 漏了步 1 装的 a
        good = _run_step2(geo, ctx, _asm(present2=["b", "base", "a"]), tol)
        order = _run_step2(geo, ctx, _asm(present2=["b", "base", "a"], present1=["a", "base", "b"]), tol)
        nosub = _run_step2(geo, ctx, _asm(present2=["b", "base", "a"], sub2=None), tol)
        badsub = _run_step2(geo, ctx, _asm(present2=["b", "base", "a"], sub2="ghost"), tol)
    finally:
        L4.PLACED = old

    f_bad = findings(bad, subject="step02", check="present_covers_derived:m2")
    f_good = findings(good, subject="step02", check="present_covers_derived:m2")
    f_ord = findings(order, subject="step01", check="present_only_installed:m1")
    f_nosub = findings(nosub, subject="step02", check="present_covers_derived:m2")
    f_badsub = findings(badsub, subject="step02", check="present_covers_derived:m2")
    sweep_ok = all(f.state == "PASS" for r in (bad, good) for f in findings(r, check="motion:m2"))

    ok_red, why_red, red = assert_red(f_bad, severity="BLOCK", require_measured=True)
    ok_green, why_green = assert_green(f_good)
    ok_ord, why_ord, red_ord = assert_red(f_ord, severity="WARN", require_measured=True)
    unk = [f for f in f_nosub + f_badsub if f.state == "FAIL" and f.measured is None]
    ok_unk = len(unk) == 2
    passed = ok_red and ok_green and ok_ord and ok_unk and sweep_ok
    return result(NAME, EXPECT, passed,
                  got=(f"漏 a：{'红' if ok_red else why_red}(measured={f_bad[0].measured if f_bad else None}) / "
                       f"补上：{'绿' if ok_green else why_green} / 顺序矛盾：{'WARN 红' if ok_ord else why_ord} / "
                       f"缺子总成声明 unknown {len(unk)}/2 / 扫掠本身全绿={sweep_ok}"),
                  red=red, expect_severity="BLOCK",
                  detail=(f"坏样本：{(f_bad[0].detail if f_bad else '')[:160]}｜顺序：{(f_ord[0].detail if f_ord else '')[:120]}"
                          f"｜unknown：{[u.detail[:60] for u in unk]}"))


if __name__ == "__main__":
    print(json.dumps(run(), ensure_ascii=False, indent=1))
