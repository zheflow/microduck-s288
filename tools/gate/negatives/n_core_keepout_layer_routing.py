#!/usr/bin/env python3
"""反例 审计 L3 MINOR（2026-09-13）—— 扫掠/运动类禁入体的义务按 keepouts.yaml:checked_in_layer 派层，
covered_by_subjects 的格子汇总成 L{层}/KOxx；不再是第 3 层五条永久 NOT_RUN → 永久 INCOMPLETE。

assertions（走 core.obligations / core.build_scorecard，不经过层）：
  1. checked_in_layer=4 的 KO 不在 L3 义务里、在 L4 义务里；没写的默认 L3。
  2. covered_by_subjects=[stepA, stepB]，stepA PASS / stepB FAIL(BLOCK) → L4/KOX 格 FAIL，check=covered_by_cells。
  3. 全 PASS → L4/KOX PASS，evidence_n = 两格之和。
  4. 少一格（stepB 没有 finding）→ L4/KOX NOT_RUN（缺格不算过）。
"""
from __future__ import annotations
import copy
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from core import LayerResult, build_scorecard, load_data, obligations, PASS, FAIL, NOT_RUN, BLOCK   # noqa: E402

NAME = "禁入体义务按 checked_in_layer 派层 + covered_by_subjects 汇总格：红传红、缺格 NOT_RUN、不写默认 L3"
EXPECT = "L4/KOX 义务存在、L3/KOX 不存在；stepB 红 → L4/KOX FAIL；全绿 → PASS；缺 stepB → NOT_RUN"


def _data():
    d = load_data()
    d = {"keepouts": {"keepouts": [{"id": "KOX", "name": "x", "checked_in_layer": 4, "covered_by_subjects": ["stepA", "stepB"]},
                                   {"id": "KOY", "name": "y"}]},
         "parts": {"parts": []}, "components": {"components": []}, "fasteners": {"fasteners": []},
         "relations": {"relations": []}, "frozen": {"joint_axes": []}, "assembly": d.get("assembly") or {}, "waivers": {"waivers": []}}
    return d


def _l4(states):
    r = LayerResult(4, "装得进去")
    for subj, st in states.items():
        r.add(subject=subj, check="sweep", state=st, severity=BLOCK, measured=0.0 if st == PASS else 3.0, criterion="t", evidence_n=7)
    return r


def run() -> dict:
    d = _data()
    ob = obligations(d)
    a1 = ("L4/KOX" in ob) and ("L3/KOX" not in ob) and ("L3/KOY" in ob)
    sc = build_scorecard([_l4({"stepA": PASS, "stepB": FAIL})], {}, [], 0.0, data=d, layers_run=[4])
    c = sc["cells"].get("L4/KOX") or {}
    a2 = c.get("state") == FAIL and c.get("checks", [{}])[0].get("check") == "covered_by_cells" and c["checks"][0].get("severity") == BLOCK
    sc2 = build_scorecard([_l4({"stepA": PASS, "stepB": PASS})], {}, [], 0.0, data=d, layers_run=[4])
    c2 = sc2["cells"].get("L4/KOX") or {}
    a3 = c2.get("state") == PASS and c2.get("evidence_n") == 14
    sc3 = build_scorecard([_l4({"stepA": PASS})], {}, [], 0.0, data=d, layers_run=[4])
    c3 = sc3["cells"].get("L4/KOX") or {}
    a4 = c3.get("state") == NOT_RUN and "stepB" in str(c3.get("checks", [{}])[0].get("detail"))
    asserts = [{"claim": "checked_in_layer=4 的 KO 义务在 L4 不在 L3；没写的默认 L3", "ok": a1},
               {"claim": "covered_by_subjects 里一格 FAIL → L4/KOX FAIL(BLOCK) covered_by_cells", "ok": a2},
               {"claim": "全 PASS → L4/KOX PASS，evidence_n = 两格之和 14", "ok": a3},
               {"claim": "缺一格 → L4/KOX NOT_RUN，detail 点名缺格", "ok": a4}]
    return {"name": NAME, "passed": all(x["ok"] for x in asserts), "expect": EXPECT,
            "got": f"a1={a1} a2={a2}({c.get('state')}) a3={a3}({c2.get('state')},{c2.get('evidence_n')}) a4={a4}({c3.get('state')})",
            "expect_severity": "BLOCK", "red": [], "assertions": asserts,
            "detail": f"L4/KOX checks: {c.get('checks')}"}


if __name__ == "__main__":
    import json
    print(json.dumps(run(), ensure_ascii=False, indent=1))
