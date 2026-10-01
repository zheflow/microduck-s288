#!/usr/bin/env python3
"""反例 ② —— 同样体积的交集落在声明弹性区外：仍 BLOCK，声明不许变成"整件免检"。

几何见 _l4_elastic_scene.py。与 ① 同一对件、同一 pla_snap 组、同样合规的 PETG + 过盈声明，交集同为 0.1 mm³：
  A  case outside：rib 在 lid **底面**（z −0.3..0.1），lid 的弹性特征 P01-F01 只登记在顶部 z 0.5..1.0
     → 交集不在"移动件特征 ∩ 障碍件特征"里 → elastic_contact_declared:lid FAIL(BLOCK) measured=区外体积≈0.1，
       motion:lid FAIL(BLOCK) measured=峰值，step_sweep FAIL(BLOCK)
  B  case ok 的几何，但 rib 的登记 bbox 写在 x −3.5..−3.0（登记位置 ≠ 真实凸棱位置）
     → 区外 ≈0.1 → 同样 BLOCK（区由数据声明，不由"反正在这两件之间"推）
  C  case ok 的几何，但没有任何 pla_snap 组关联这两件 → 纯刚体 BLOCK，不发弹性结果
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

NAME = "声明弹性区外的同体积交集仍 BLOCK（区外 = 普通刚体判据）"
EXPECT = ("L4/step01:elastic_contact_declared:lid FAIL(BLOCK) measured=区外体积≈0.1；motion:lid / step_sweep FAIL(BLOCK)；"
          "登记 bbox 偏位同样 BLOCK；无卡扣组 → 纯刚体 BLOCK 且无弹性结果")


def run():
    asserts, reds = [], []
    ra, tol = S.run_motion("outside", rib_decl=S.decl())
    el = findings(ra, subject="step01", check="elastic_contact_declared:lid")
    mo = findings(ra, subject="step01", check="motion:lid")
    ss = findings(ra, subject="step01", check="step_sweep")
    ok_e, why_e, red_e = assert_red(el, severity="BLOCK", require_measured=True)
    ok_m, why_m, red_m = assert_red(mo, severity="BLOCK", require_measured=True)
    reds += red_e + red_m
    asserts.append({"claim": "A elastic_contact_declared:lid FAIL(BLOCK) 带区外体积", "ok": ok_e})
    asserts.append({"claim": "A 区外体积≈0.1（与 ① 同体积）", "ok": bool(
        ok_e and abs(float(el[0].measured) - 0.1) < 1e-6)})
    asserts.append({"claim": "A motion:lid FAIL(BLOCK) 峰值≈0.1", "ok": bool(
        ok_m and abs(float(mo[0].measured) - 0.1) < 1e-6)})
    asserts.append({"claim": "A step_sweep FAIL(BLOCK)", "ok": bool(
        len(ss) == 1 and ss[0].state == "FAIL" and ss[0].severity == "BLOCK")})

    rb, _ = S.run_motion("ok", rib_decl=S.decl(), rib_bbox=((-3.5, -1.0, 0.9), (-3.0, 1.0, 1.3)))
    eb = findings(rb, subject="step01", check="elastic_contact_declared:lid")
    mb = findings(rb, subject="step01", check="motion:lid")
    ok_eb, why_eb, red_eb = assert_red(eb, severity="BLOCK", require_measured=True)
    reds += red_eb
    asserts.append({"claim": "B 登记 bbox 偏位 → elastic BLOCK 带区外体积", "ok": ok_eb})
    asserts.append({"claim": "B motion:lid BLOCK", "ok": bool(len(mb) == 1 and mb[0].severity == "BLOCK" and mb[0].state == "FAIL")})

    rc, _ = S.run_motion("ok", rib_decl=S.decl(), with_group=False)
    mc = findings(rc, subject="step01", check="motion:lid")
    ok_mc, why_mc, red_mc = assert_red(mc, severity="BLOCK", require_measured=True)
    reds += red_mc
    asserts.append({"claim": "C 无 pla_snap 组 → motion:lid BLOCK", "ok": ok_mc})
    asserts.append({"claim": "C 无 pla_snap 组 → 不发 elastic_contact_declared",
                    "ok": not findings(rc, subject="step01", check_prefix="elastic_contact_declared")})

    passed = all(a["ok"] for a in asserts)
    got = "；".join(f"{'✓' if a['ok'] else '✗'}{a['claim']}" for a in asserts)
    detail = (f"A={[record(f) for f in el + mo + ss]} {why_e}{why_m} | B={[record(f) for f in eb + mb]} {why_eb} | "
              f"C={[record(f) for f in mc]} {why_mc}")
    out = result(NAME, EXPECT, passed, got, reds, expect_severity="BLOCK", detail=detail)
    out["assertions"] = asserts
    return out


if __name__ == "__main__":
    r = run()
    print(json.dumps(r, ensure_ascii=False, indent=1))
    sys.exit(0 if r["passed"] else 1)
