#!/usr/bin/env python3
"""反例 ③ —— 交集在声明弹性区内，但体积超出"过盈量 × 接触面积上限"：BLOCK。

声明区只说明"这里允许弹性变形"，不说明"这里多大的干涉都行"。上限 = 声明过盈量 δ × 面积，
面积 = features 声明的 elastic_contact.contact_area_mm2；缺则取该区核心盒（两特征 bbox 之交，不含外扩）最大面面积。
几何见 _l4_elastic_scene.py：
  A  case overcap：rib z 0.6..1.3，交集 0.5×2×0.4 = 0.4 mm³，全在 P01-F01 ∩ O01-F01 里；
     δ=0.12，核心盒最大面 0.5×2 = 1.0 → 上限 0.12 → elastic_contact_declared:lid FAIL(BLOCK) measured=0.4
  B  case ok 的几何（0.1 mm³，本来 WARN），但声明 contact_area_mm2 = 0.2 → 上限 0.024 → BLOCK
     （声明的面积优先于 bbox 面积；声明得更小就更严）
"""
from __future__ import annotations
import json
import sys
from pathlib import Path

_HERE = Path(__file__).resolve()
for _p in (str(_HERE.parent), str(_HERE.parents[1]), str(_HERE.parents[3])):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from _harness import findings, assert_red, result, record                    # noqa: E402
import _l4_elastic_scene as S                                                 # noqa: E402

NAME = "声明弹性区内但体积超过 过盈量×面积上限 → BLOCK"
EXPECT = "L4/step01:elastic_contact_declared:lid FAIL(BLOCK) measured≈0.4（上限 0.12）；声明小面积时 0.1 也 BLOCK；motion:lid BLOCK"


def run():
    asserts, reds = [], []
    ra, _ = S.run_motion("overcap", rib_decl=S.decl())
    el = findings(ra, subject="step01", check="elastic_contact_declared:lid")
    mo = findings(ra, subject="step01", check="motion:lid")
    ok_e, why_e, red_e = assert_red(el, severity="BLOCK", require_measured=True)
    ok_m, why_m, red_m = assert_red(mo, severity="BLOCK", require_measured=True)
    reds += red_e + red_m
    asserts.append({"claim": "A elastic_contact_declared:lid FAIL(BLOCK)", "ok": ok_e})
    asserts.append({"claim": "A measured≈0.4（区内体积，超上限 0.12）", "ok": bool(
        ok_e and abs(float(el[0].measured) - 0.4) < 1e-6)})
    asserts.append({"claim": "A 说明里写出超上限（0.12）", "ok": bool(el and "超上限" in el[0].detail and "0.12" in el[0].detail)})
    asserts.append({"claim": "A motion:lid FAIL(BLOCK)", "ok": ok_m})

    rb, _ = S.run_motion("ok", rib_decl=S.decl(area=0.2))
    eb = findings(rb, subject="step01", check="elastic_contact_declared:lid")
    ok_b, why_b, red_b = assert_red(eb, severity="BLOCK", require_measured=True)
    reds += red_b
    asserts.append({"claim": "B 声明 contact_area_mm2=0.2 → 0.1 mm³ 超上限 0.024 → BLOCK", "ok": bool(
        ok_b and abs(float(eb[0].measured) - 0.1) < 1e-6)})

    passed = all(a["ok"] for a in asserts)
    got = "；".join(f"{'✓' if a['ok'] else '✗'}{a['claim']}" for a in asserts)
    out = result(NAME, EXPECT, passed, got, reds, expect_severity="BLOCK",
                 detail=f"A={[record(f) for f in el + mo]} {why_e}{why_m} | B={[record(f) for f in eb]} {why_b}")
    out["assertions"] = asserts
    return out


if __name__ == "__main__":
    r = run()
    print(json.dumps(r, ensure_ascii=False, indent=1))
    sys.exit(0 if r["passed"] else 1)
