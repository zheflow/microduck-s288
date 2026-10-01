#!/usr/bin/env python3
"""反例 ① —— 声明弹性区内的小交集：只能 WARN（elastic_contact_declared），不许判 PASS，也不再判 BLOCK。

来源：hr39f B03 顶盖开盖路径（后移 3.5 再上提 30）在 dx=−1.25 处与两壳前舌凸点各接触 0.65 mm³，
这是设计的 PETG 弹性过盈 0.25。用户问"要不要给卡扣豁免"，决定：**不豁免，改成声明式**（hr41g）。
刚体 L4 以前只能 BLOCK；现在"交集全部落在同一 pla_snap 组两侧特征 bbox 的交里、且 ≤ 过盈量×面积上限、
且声明了 PETG + 过盈量" → FAIL(WARN)"声明弹性接触，保持力/寿命待实测 BN19/BN20"。

几何见 _l4_elastic_scene.py（case ok：lid 顶面扫过 rib，交集 0.1 mm³；过盈 0.12 × 面积 1.0 = 0.12 上限）。
  A  rib 侧声明 elastic_contact → step01:elastic_contact_declared:lid FAIL(WARN) measured≈0.1，文字含 BN19/BN20；
     motion:lid FAIL(WARN)（不是 PASS，也不是 BLOCK）；step_sweep FAIL(WARN)
  B  lid 侧声明（另一侧不声明）→ 同样 WARN（声明放在任一侧的弹性特征上都认）
  C  对照 case clean（不接触）→ motion:lid PASS，且不发 elastic_contact_declared（分类器不许对干净路径开火）
  D  整层 run(ctx) 的拆卸序：disasm01 disassembly_sweep FAIL(WARN) + elastic_contact_declared FAIL(WARN)；
     lid 按"声明弹性下可拆"迁出障碍集合，disasm02 不挂 BLOCK 的 disassembly_prereq，
     但必须挂 disassembly_prereq_elastic FAIL(WARN)（后续结论以卡扣实物可挠为前提，不静默）
"""
from __future__ import annotations
import json
import sys
from pathlib import Path

_HERE = Path(__file__).resolve()
for _p in (str(_HERE.parent), str(_HERE.parents[1]), str(_HERE.parents[3])):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from _harness import findings, assert_red, assert_green, result, record      # noqa: E402
import _l4_elastic_scene as S                                                 # noqa: E402

NAME = "声明弹性区内小交集只判 WARN（elastic_contact_declared），不判 PASS/BLOCK [WARN]"
EXPECT = ("L4/step01:elastic_contact_declared:lid FAIL(WARN) measured≈0.1 含 BN19/BN20；motion:lid 与 step_sweep FAIL(WARN)；"
          "clean 对照 PASS 且无弹性结果；拆卸序 disasm01 WARN、disasm02 disassembly_prereq_elastic WARN 无 BLOCK prereq")


def _one(fs):
    return fs[0] if len(fs) == 1 else None


def run():
    asserts, reds = [], []

    # A：rib 侧声明
    res, tol = S.run_motion("ok", rib_decl=S.decl())
    el = findings(res, subject="step01", check="elastic_contact_declared:lid")
    mo = findings(res, subject="step01", check="motion:lid")
    ss = findings(res, subject="step01", check="step_sweep")
    ok_a, why_a, red_a = assert_red(el, severity="WARN", require_measured=True)
    reds += red_a
    e1, m1, s1 = _one(el), _one(mo), _one(ss)
    asserts.append({"claim": "A elastic_contact_declared:lid 单条 FAIL(WARN) measured≈0.1", "ok": bool(
        ok_a and e1 and abs(float(e1.measured) - 0.1) < 1e-6)})
    asserts.append({"claim": "A 文字含『声明弹性接触』与 BN19/BN20", "ok": bool(
        e1 and "声明弹性接触" in e1.detail and "BN19/BN20" in e1.detail)})
    asserts.append({"claim": "A motion:lid FAIL(WARN)，不是 PASS/BLOCK", "ok": bool(
        m1 and m1.state == "FAIL" and m1.severity == "WARN" and m1.measured is not None)})
    asserts.append({"claim": "A step_sweep FAIL(WARN)", "ok": bool(s1 and s1.state == "FAIL" and s1.severity == "WARN")})

    # B：lid 侧声明，rib 侧不声明
    rb, _ = S.run_motion("ok", lid_decl=S.decl())
    eb = _one(findings(rb, subject="step01", check="elastic_contact_declared:lid"))
    mb = _one(findings(rb, subject="step01", check="motion:lid"))
    asserts.append({"claim": "B 声明在移动件侧特征上也认 → WARN", "ok": bool(
        eb and eb.state == "FAIL" and eb.severity == "WARN" and mb and mb.severity == "WARN")})

    # C：对照 —— 不接触
    rc, _ = S.run_motion("clean", rib_decl=S.decl())
    ok_c, why_c = assert_green(findings(rc, subject="step01", check="motion:lid"))
    no_el = not findings(rc, subject="step01", check_prefix="elastic_contact_declared")
    asserts.append({"claim": "C clean 对照 motion:lid PASS", "ok": ok_c})
    asserts.append({"claim": "C clean 对照不发 elastic_contact_declared", "ok": no_el})

    # D：整层 run(ctx) 的拆卸序
    rl = S.run_layer("ok", rib_decl=S.decl())
    d1 = _one(findings(rl, subject="disasm01", check="disassembly_sweep"))
    d1e = _one(findings(rl, subject="disasm01", check_prefix="elastic_contact_declared"))
    d2p = findings(rl, subject="disasm02", check="disassembly_prereq")
    d2e = _one(findings(rl, subject="disasm02", check="disassembly_prereq_elastic"))
    d2s = findings(rl, subject="disasm02", check="disassembly_sweep")
    asserts.append({"claim": "D disasm01 disassembly_sweep FAIL(WARN)", "ok": bool(
        d1 and d1.state == "FAIL" and d1.severity == "WARN")})
    asserts.append({"claim": "D disasm01 elastic_contact_declared FAIL(WARN)", "ok": bool(
        d1e and d1e.state == "FAIL" and d1e.severity == "WARN")})
    asserts.append({"claim": "D disasm02 无 BLOCK 的 disassembly_prereq", "ok": not d2p})
    asserts.append({"claim": "D disasm02 disassembly_prereq_elastic FAIL(WARN)", "ok": bool(
        d2e and d2e.state == "FAIL" and d2e.severity == "WARN")})
    ok_d2s, why_d2s = assert_green(d2s)
    asserts.append({"claim": "D disasm02 自身扫掠 PASS（lid 已按声明迁出）", "ok": ok_d2s})

    passed = all(a["ok"] for a in asserts)
    got = "；".join(f"{'✓' if a['ok'] else '✗'}{a['claim']}" for a in asserts)
    detail = (f"A={[record(f) for f in el + mo + ss]} | B={[record(f) for f in (eb, mb) if f]} | "
              f"C={'绿' if ok_c else why_c} | D1={record(d1) if d1 else None} D2prereq={[record(f) for f in d2p]} "
              f"D2el={record(d2e) if d2e else None} D2sweep={'绿' if ok_d2s else why_d2s}")
    out = result(NAME, EXPECT, passed, got, reds, expect_severity="WARN", detail=detail)
    out["assertions"] = asserts
    return out


if __name__ == "__main__":
    r = run()
    print(json.dumps(r, ensure_ascii=False, indent=1))
    sys.exit(0 if r["passed"] else 1)
