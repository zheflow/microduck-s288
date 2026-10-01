#!/usr/bin/env python3
"""反例（09-13 F-L7-4）—— 质量清单有缺口时，整机质量汇总判据不许 PASS。

修前：15 类元件无质量、4 条 component_body 红、2 条 world_placement_for_mass 红，`com_in_support_home` /
`joint:*:static_torque` 照样 PASS(BLOCK)，detail 里才写"缺头部电子件"。缺的是力臂最长的头部件，绿是下界不是结论。

坏样本：`_l7_fixture.complete_inventory` 把清单堵完整之后，**只删一个数** —— battery_3s.mass_g.v → null
（66 g，仓在躯干）。修前：battery 进不了清单但 com_in_support_home / static_torque 仍 PASS；
修后：`_stability:com_in_support_home`、14 条 `joint:*:static_torque`、`joint:*:static_torque_home`、
`_load:head_mass_budget` 全部 unknown（measured=None），detail 列出缺口清单（含 battery_3s）并给
"仅含已知质量的下界值"文本。
对照：完整清单 → 上述判据 PASS（evidence_n > 0）。
用了 negatives/_l7_fixture.py 的夹具（只删不编，见该文件头）。
"""
from __future__ import annotations
import json
import sys
from copy import deepcopy
from pathlib import Path

_HERE = Path(__file__).resolve()
GATE = _HERE.parents[1]
for _p in (str(_HERE.parent), str(GATE), str(GATE / "layers"), str(GATE.parents[1])):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import core                                                                      # noqa: E402
from _harness import findings, assert_red, assert_green, result, record, summarize  # noqa: E402
import _l7_fixture as FX                                                         # noqa: E402

NAME = "L7：质量清单缺一个元件的质量 → com_in_support_home / static_torque 必须 unknown，不许拿下界当绿"
EXPECT = ("L7/_stability:com_in_support_home FAIL(measured=None)、L7/joint:*:static_torque 全 FAIL(measured=None)，"
          "detail 点名 battery_3s 并给下界文本；对照完整清单全 PASS")
GAP_ID = "battery_3s"


def run() -> dict:
    base = core.load_data()
    ctrl, drop = FX.complete_inventory(base)
    pd = FX.placed_stand_in("gap", drop)
    bad = deepcopy(ctrl)
    c = next(x for x in bad["components"]["components"] if x.get("id") == GAP_ID)
    removed = c["mass_g"]["v"]
    c["mass_g"]["v"] = None

    r_bad = FX.run_l7(bad, placed_dir=pd)
    r_good = FX.run_l7(ctrl, placed_dir=pd)

    com = findings(r_bad, subject="_stability", check="com_in_support_home")
    tor = [f for f in r_bad.findings if f.check == "static_torque"]
    torh = [f for f in r_bad.findings if f.check == "static_torque_home"]
    budget = findings(r_bad, subject="_load", check="head_mass_budget")
    ok_c, why_c, red_c = assert_red(com, severity="BLOCK", require_measured=False)
    ok_t, why_t, red_t = assert_red(tor, severity="BLOCK", require_measured=False)
    all_unknown = (ok_c and ok_t and all(f.state == "FAIL" and f.measured is None for f in com + tor + torh)
                   and len(tor) == 14 and bool(budget) and all(f.state == "FAIL" and f.measured is None for f in budget))
    named = all(GAP_ID in (f.detail or "") for f in com + tor)
    lower = all(("下界" in (f.detail or "")) for f in com + tor)

    g_com = findings(r_good, subject="_stability", check="com_in_support_home")
    g_tor = [f for f in r_good.findings if f.check == "static_torque"]
    g_bud = findings(r_good, subject="_load", check="head_mass_budget")
    okg1, whyg1 = assert_green(g_com)
    okg2, whyg2 = assert_green(g_tor)
    okg3, whyg3 = assert_green(g_bud)

    passed = ok_c and ok_t and all_unknown and named and lower and okg1 and okg2 and okg3
    return result(NAME, EXPECT, passed,
                  got=(f"删 {GAP_ID}.mass_g（{removed} g）→ com_in_support_home {summarize(com, n=1)}；"
                       f"static_torque {sum(1 for f in tor if f.state == 'FAIL' and f.measured is None)}/{len(tor)} unknown、"
                       f"static_torque_home {sum(1 for f in torh if f.state == 'FAIL' and f.measured is None)}/{len(torh)} unknown、"
                       f"head_mass_budget {summarize(budget, n=1)}；detail 点名缺口 {named}、给下界 {lower}；"
                       f"对照 com {'绿' if okg1 else whyg1}（余量 {g_com[0].measured if g_com else None}）、"
                       f"static_torque {'绿' if okg2 else whyg2}、head_mass_budget {'绿' if okg3 else whyg3}"),
                  red=(red_c if ok_c else []) + (red_t[:2] if ok_t else []), expect_severity="BLOCK",
                  allow_unknown_red="质量清单缺口 = 整机量算不全：层必须判 unknown（measured=None），detail 只给下界文本",
                  detail=f"坏={[record(f) for f in com]}；{summarize(tor, n=2)}；对照={summarize(g_com + g_tor[:1], n=2)}")


if __name__ == "__main__":
    r = run()
    print(json.dumps(r, ensure_ascii=False, indent=1))
    sys.exit(0 if r["passed"] else 1)
