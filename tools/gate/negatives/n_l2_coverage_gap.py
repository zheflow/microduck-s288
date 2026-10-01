#!/usr/bin/env python3
"""反例（09-13 F-L2-1）—— 零判据的特征不许记 covered；shape=sweep 无处可探的特征必须 unknown。

坑的形状：l2_features.run 在特征循环一进来就 `res.covered.add(fid)`，然后
`if cyl_u:` / `if box_u and not cyl_u:` 两支都空时**什么都不发**——12 条 sweep/corridor/
shell_clearance_cut 类特征一条判据没有，却被 core.COVERAGE_ITEMS 算成"已覆盖"，
`L2/_coverage 171/171` 假绿。

正确做法：
  · 两支都空 → res.unknown(pid, f"{fid}:geometry", …)（未知=失败，元规则 4）；
  · covered 只登记**真量到东西**（至少一条带 measured 的判据）的特征 —— unknown 不算覆盖；
  · geom.box_export_local {lo,hi}（滑配走廊等扫掠体的 export_local 外包盒，L1 已经这么读）
    也算可探的 bbox，让 L01-F13 / L03-F05 这种有盒的走廊能落到 occupancy 判据上。

三条特征同一个件：
  A  sweep_cut，geom 只有 sweep_range/swept_body（照 features.yaml:L01-F11 造）→ 必须有 unknown、且 **不在** covered
  B  Ø2.2 通孔（nominal_d_mm）→ present 正常发、在 covered（对照）
  C  slide_corridor，shape=sweep 但有 box_export_local {lo,hi} 落在一个真空腔上 → occupancy 发、在 covered（对照）
"""
from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from _harness import FakeCtx, assert_green, assert_red, base_data, box, cyl, export_stl, findings, result  # noqa: E402

PART = "L01"
FID_SWEEP, FID_HOLE, FID_CORR = "L01-NEG-SWEEP", "L01-NEG-HOLE", "L01-NEG-CORR"
NAME = "零判据特征不记 covered：sweep 无探针 → unknown 且不覆盖；孔 / 带盒走廊正常覆盖"
EXPECT = (f"L2/{PART}:{FID_SWEEP}:geometry FAIL(unknown) 且 {FID_SWEEP} ∉ res.covered；"
          f"{FID_HOLE}:present PASS ∈ covered；{FID_CORR}:occupancy PASS ∈ covered")

BLK = (24.0, 20.0, 10.0)
HOLE_X = 7.0
POCKET_C, POCKET_S = (-6.0, 0.0, 6.0), (6.0, 6.0, 8.0)     # 从顶面挖下去的空腔（z 2..10）


def _mesh():
    m = box(*BLK, center=(0.0, 0.0, BLK[2] / 2))
    m = m.difference(cyl(1.1, 40.0, center=(HOLE_X, 0.0, BLK[2] / 2)))
    return m.difference(box(*POCKET_S, center=POCKET_C))


def _data():
    part = {"id": PART, "inventory_id": f"{PART}_neg_cov", "material": "PLA",
            "print_orientation": "任意", "mirrored_copy": None}
    sweep = {"id": FID_SWEEP, "part": PART, "kind": "sweep_cut", "check_class": "sweep_cut", "count": 1,
             "spec_verbatim": "sweep_of(build_hip, hip_l, ±24°, n9, grow0.4)（反例）",
             "geom": {"shape": "sweep", "frame": "export_local", "confidence": "derived", "count": 1,
                      "count_kind": "instances", "through": None,
                      "depth_mm": {"v": None, "src": None, "unknown_class": "not_applicable"},
                      "instances": [{"grow_mm": {"v": 0.4, "src": "assumed"}, "sweep_range_deg": [-24, 24],
                                     "sweep_samples": 9, "swept_body": "hip_l"}],
                      "note": "扫掠让位刀，无解析直径/深度（反例：故意没有任何可探的 pos/bbox）"}}
    hole = {"id": FID_HOLE, "part": PART, "kind": "screw_hole", "check_class": "screw_hole", "count": 1,
            "spec_verbatim": "1×Ø2.2 通",
            "geom": {"shape": "cylinder", "frame": "export_local",
                     "nominal_d_mm": {"v": 2.2, "src": "measured"},
                     "depth_mm": {"v": BLK[2], "src": "assumed"}, "depth_kind": "cutter_length",
                     "through": True, "axis": "z", "pos": [HOLE_X, 0.0, BLK[2] / 2],
                     "axial_span_mm": [[HOLE_X, 0.0, 0.0], [HOLE_X, 0.0, BLK[2]]]}}
    lo = [POCKET_C[i] - POCKET_S[i] / 2 for i in range(3)]
    hi = [POCKET_C[i] + POCKET_S[i] / 2 for i in range(3)]
    corr = {"id": FID_CORR, "part": PART, "kind": "slide_corridor", "check_class": "slide_corridor", "count": 1,
            "spec_verbatim": "servo_slide 走廊（反例：盒 = 顶面空腔）",
            "geom": {"shape": "sweep", "frame": "export_local", "confidence": "derived", "count": 1,
                     "axis": "z", "through": None, "zone_shape": "box_hull_of_sweep",
                     "depth_mm": {"v": None, "src": None, "unknown_class": "not_applicable"},
                     "box_export_local": {"lo": lo, "hi": hi, "src": "measured"},
                     "sweep_len_mm": {"v": 8.0, "src": "assumed"}}}
    return base_data([part], [sweep, hole, corr])


def run() -> dict:
    import l2_features
    l2_features._GEOM_CACHE.clear()
    stl = export_stl(_mesh(), "l2_coverage_gap")
    res = l2_features.run(FakeCtx(_data(), {PART: stl}))
    fs_sweep = [f for f in findings(res, subject=PART) if f.check.startswith(FID_SWEEP + ":")]
    fs_hole = findings(res, subject=PART, check=f"{FID_HOLE}:present")
    fs_corr = findings(res, subject=PART, check=f"{FID_CORR}:occupancy")
    ok_red, why_red, red = assert_red(fs_sweep, severity="BLOCK", require_measured=False)
    has_unknown = any(f.state == "FAIL" and f.measured is None for f in fs_sweep)
    ok_hole, why_hole = assert_green(fs_hole)
    ok_corr, why_corr = assert_green(fs_corr)
    cov = set(res.covered)
    checks = [
        (f"{FID_SWEEP} 发了 unknown（measured=None 的 FAIL）", ok_red and has_unknown),
        (f"{FID_SWEEP} 不在 res.covered 里", FID_SWEEP not in cov),
        (f"{FID_HOLE}:present PASS", ok_hole),
        (f"{FID_HOLE} 在 res.covered 里", FID_HOLE in cov),
        (f"{FID_CORR}:occupancy PASS（box_export_local 被认作可探的盒）", ok_corr),
        (f"{FID_CORR} 在 res.covered 里", FID_CORR in cov),
    ]
    failed = [c for c, ok in checks if not ok]
    got = (f"sweep 判据 {[(f.check, f.state, f.measured) for f in fs_sweep]}；covered={sorted(cov)}；"
           f"hole present={[f.state for f in fs_hole]}；corr occupancy={[(f.state, f.measured) for f in fs_corr]}")
    return result(NAME, EXPECT, not failed, got + (f"｜未成立：{failed}" if failed else ""), red,
                  expect_severity="BLOCK",
                  allow_unknown_red="验的正是『无处可探的特征必须 unknown、且不许记 covered』这条设计路径",
                  detail=" | ".join(x for x in (why_red, why_hole, why_corr) if x))


if __name__ == "__main__":
    import json
    r = run()
    print(json.dumps(r, ensure_ascii=False, indent=1))
    sys.exit(0 if r["passed"] else 1)
