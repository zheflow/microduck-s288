#!/usr/bin/env python3
"""A scanned box cannot cover an unscannable solid in the same feature."""
from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from _harness import FakeCtx, assert_green, assert_red, base_data, box, export_stl, findings, result

PART, FID = "B03", "B03-NEG-PARTIAL"
NAME = "混合实体特征：只扫描一个box不能掩盖另一形状的保留区未知"
EXPECT = f"{FID}:solid_retention_coverage FAIL/WARN/None；两个可扫描box对照正常"


def _observe(unknown):
    import l2_features
    l2_features._GEOM_CACHE.clear()
    first = {"shape": "box", "bbox_mm": [[-4, -3, -.5], [-1, 3, .5]], "pos": [-2.5, 0, 0]}
    second = {"shape": "box", "bbox_mm": [[1, -3, -.5], [4, 3, .5]], "pos": [2.5, 0, 0]}
    if unknown:
        second["shape"] = "box_clipped_by_outer_envelope"
        second["nominal_box_mm"] = second.pop("bbox_mm")
    feature = {"id": FID, "part": PART, "kind": "plate", "check_class": "plate", "count": 2,
               "spec_verbatim": "两个实体区域，第二个可只给代表点但必须显式报告保留区未知",
               "geom": {"shape": "composite", "frame": "export_local", "instances": [first, second]}}
    part = {"id": PART, "inventory_id": "B03_partial", "material": "PETG",
            "print_orientation": "任意", "mirrored_copy": None}
    data = base_data([part], [feature], fasteners=[], relations=[])
    stl = export_stl(box(12, 10, 4), "partial_solid_" + str(unknown))
    return l2_features.run(FakeCtx(data, {PART: stl}))


def run():
    bad, good = _observe(True), _observe(False)
    missing = findings(bad, subject=PART, check=f"{FID}:solid_retention_coverage")
    good_missing = findings(good, subject=PART, check=f"{FID}:solid_retention_coverage")
    bad_retention = findings(bad, subject=PART, check=f"{FID}:solid_retention")
    good_retention = findings(good, subject=PART, check=f"{FID}:solid_retention")
    red_ok, red_why, red = assert_red(missing, severity="WARN", require_measured=False)
    old_ok, old_why = assert_green(bad_retention)
    good_ok, good_why = assert_green(good_retention)
    explicit_unknown = len(missing) == 1 and missing[0].measured is None
    passed = red_ok and explicit_unknown and old_ok and good_ok and not good_missing
    return result(NAME, EXPECT, passed,
                  f"mixed coverage={[(f.state, f.measured) for f in missing]}; "
                  f"scannable retention={[f.state for f in bad_retention]}; "
                  f"full-box retention={[f.state for f in good_retention]}", red,
                  expect_severity="WARN", detail=" | ".join(filter(None, [red_why, old_why, good_why])),
                  allow_unknown_red="此反例要证明未扫描的实体保留区必须显式unknown，不能借已检区域通过。")


if __name__ == "__main__":
    import json
    outcome = run()
    print(json.dumps(outcome, ensure_ascii=False, indent=2))
    sys.exit(0 if outcome["passed"] else 1)
