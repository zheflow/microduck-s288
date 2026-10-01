#!/usr/bin/env python3
"""反例 ④ —— 声明区里缺过盈量或材料（或材料不是 PETG / 与 parts.yaml 不一致 / 过盈量无 src）：仍 BLOCK。

与 ① 完全相同的几何（交集 0.1 mm³ 全在两侧特征 bbox 之交里），只改声明：
  A  只写 material、缺 interference_mm            → 说明里点名 "interference_mm"
  B  只写 interference_mm、缺 material            → 点名 "material"
  C  material = PLA                                → 点名 "PLA"
  D  interference_mm 没有 src                      → 点名 "src"
  E  特征写 PETG，但 parts.yaml 该件 material=PLA  → 点名 "parts.yaml"（两处材料不一致 = 说不清，不猜）
  F  pla_snap 组在、两侧都没有 elastic_contact     → 点名 "elastic_contact"
每种：motion:lid FAIL(BLOCK) measured=峰值 0.1（刚体仍然红），elastic_contact_declared:lid FAIL(BLOCK) measured=None
（声明不全 = unknown，不是"抓到超量"），说明里写明缺什么。
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

NAME = "声明区缺过盈量/材料（或材料不是 PETG、与 parts 不一致、过盈量无 src）→ 仍 BLOCK 并写明缺什么"
EXPECT = "每种声明缺陷：L4/step01:motion:lid FAIL(BLOCK) measured≈0.1 + elastic_contact_declared:lid FAIL(BLOCK) unknown，detail 点名缺项"

CASES = [
    ("A 缺过盈量", dict(rib_decl=S.decl(drop=("interference_mm",))), "O01-F01 缺 elastic_contact.interference_mm"),
    ("B 缺材料", dict(rib_decl=S.decl(drop=("material",))), "O01-F01 缺 elastic_contact.material"),
    ("C 材料 PLA", dict(rib_decl=S.decl(material="PLA")), "O01-F01 elastic_contact.material='PLA' 不在"),
    ("D 过盈量无 src", dict(rib_decl=S.decl(src=None)), "O01-F01 elastic_contact.interference_mm 缺 src"),
    ("E 与 parts.yaml 材料不一致", dict(rib_decl=S.decl(), rib_part_material="PLA"), "parts.yaml:O01.material='PLA' 不一致"),
    ("F 两侧都没有 elastic_contact", dict(), "O01-F01 缺 elastic_contact（"),
]


def run():
    asserts, reds, det = [], [], []
    for tag, kw, token in CASES:
        r, _ = S.run_motion("ok", **kw)
        mo = findings(r, subject="step01", check="motion:lid")
        el = findings(r, subject="step01", check="elastic_contact_declared:lid")
        ok_m, why_m, red_m = assert_red(mo, severity="BLOCK", require_measured=True)
        reds += red_m
        e = el[0] if len(el) == 1 else None
        ok_e = bool(e and e.state == "FAIL" and e.severity == "BLOCK" and e.measured is None and token in e.detail)
        asserts.append({"claim": f"{tag}：motion:lid FAIL(BLOCK) 峰值≈0.1", "ok": bool(
            ok_m and abs(float(mo[0].measured) - 0.1) < 1e-6)})
        asserts.append({"claim": f"{tag}：elastic_contact_declared:lid FAIL(BLOCK) unknown 且点名 {token!r}", "ok": ok_e})
        det.append(f"{tag}: {[record(f) for f in mo + el]} {why_m} | el.detail={(e.detail[:160] if e else None)!r}")
    passed = all(a["ok"] for a in asserts)
    got = "；".join(f"{'✓' if a['ok'] else '✗'}{a['claim']}" for a in asserts)
    out = result(NAME, EXPECT, passed, got, reds, expect_severity="BLOCK", detail=" || ".join(det))
    out["assertions"] = asserts
    return out


if __name__ == "__main__":
    r = run()
    print(json.dumps(r, ensure_ascii=False, indent=1))
    sys.exit(0 if r["passed"] else 1)
