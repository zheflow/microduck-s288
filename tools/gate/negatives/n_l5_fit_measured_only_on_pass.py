#!/usr/bin/env python3
"""反例（09-13 F-L5-3）—— 关系量了但没过，配合桶的 `fit_measured_geometry` 不许 PASS。

坑的形状：l5_function 在 relation_contact 发出**之后**无条件 `measured_buckets[path] = rid`，
于是桶级 `fit_measured_geometry` = PASS(BLOCK)，而同一条关系的 relation_contact 是红的
（记分卡上 keyhole / bearing_axial_retention / plain_slide_head_roll / back_clamp 四个桶都是这样：
"有实测几何撑着" 被读成 "实测几何通过了"）。

正确判据：只有 relation_contact 的 state==PASS 才登记；量了但没过 → 该桶
`fit_measured_geometry` FAIL(BLOCK, measured=rid, detail="R.. 量了但没过")；没人量过 → unknown（不变）。

几何复用 n_l5_fg13_pad_contact_label_only.py（H02 垫柱退 0.30 什么都没夹住）：
  坏样本  退 0.30 → relation_contact FAIL → fits.servo.pad_contact:fit_measured_geometry 必须 FAIL 且 measured=RID
  对照    贴合   → relation_contact PASS → fit_measured_geometry PASS
"""
from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from _harness import FakeCtx, assert_green, assert_red, base_data, box, cyl, export_stl, findings, result  # noqa: E402

RID = "NG53_pad_contact"
BUCKET = "fits.servo.pad_contact"
CHECK = "fit_measured_geometry"
NAME = "关系量了但没过：配合桶 fit_measured_geometry 必须 FAIL(measured=关系 id)；贴合对照必须 PASS"
EXPECT = f"L5/{BUCKET}:{CHECK} FAIL(BLOCK, measured={RID}) 当 {RID}:relation_contact 为 FAIL；贴合时 PASS"

SERVO_FACE_X = -14.85
GAP = 0.30


def _pad(bad: bool):
    face = SERVO_FACE_X - (GAP if bad else 0.0)
    a = cyl(2.5, 10.0, center=(face - 5.0, 4.0, 0.0), axis="x")
    b = cyl(2.5, 10.0, center=(face - 5.0, -4.0, 0.0), axis="x")
    return a.union(b)


def _servo():
    return box(10.0, 20.0, 12.0, center=(SERVO_FACE_X + 5.0, 0.0, 0.0))


def _data():
    parts = [{"id": "H02", "inventory_id": "H02_pad", "material": "PLA",
              "print_orientation": "背板外面朝下", "mirrored_copy": None},
             {"id": "H01", "inventory_id": "H01_servoblock", "material": "PLA",
              "print_orientation": "脚底朝下", "mirrored_copy": None}]
    rel = {"id": RID, "kind": "clamp",
           "between": ["H02 垫柱端面", "横滚舵机薄段背面 -14.85"],
           "declared_target": "接触", "target_ref": f"tolerances.yaml:{BUCKET}",
           "actual": "按 T=19.9 名义接触", "actual_src": "measured", "status": "ok",
           "contact": {"axis": "x", "approach": "+x",
                       "parties": [{"part": "H02", "role": "mover"}, {"part": "H01", "role": "fixed"}],
                       "probe_bbox_mm": [[-16.5, -8.0, -4.0], [-13.5, 8.0, 4.0]]},
           "provenance": ["negatives/n_l5_fit_measured_only_on_pass"]}
    return base_data(parts, [], fasteners=[], relations=[rel])


def _observe(bad: bool):
    import l2_features
    import l5_function
    l2_features._GEOM_CACHE.clear()
    tag = "gap" if bad else "touch"
    pa = export_stl(_pad(bad), f"f53_pad_{tag}")
    pb = export_stl(_servo(), f"f53_servo_{tag}")
    ctx = FakeCtx(_data(), {"H02": pa, "H01": pb}, placed_map={"pad": pa, "servoblock": pb})
    res = l5_function.run(ctx)
    return (findings(res, subject=RID, check="relation_contact"),
            findings(res, subject=BUCKET, check=CHECK))


def run() -> dict:
    bad_rel, bad_fit = _observe(True)
    good_rel, good_fit = _observe(False)
    ok_rel_red, why_rel, _ = assert_red(bad_rel, severity="BLOCK")
    ok_fit_red, why_fit, red = assert_red(bad_fit, severity="BLOCK")
    fit_names_rid = bool(bad_fit) and RID in str(bad_fit[0].measured)
    ok_rel_g, why_rg = assert_green(good_rel)
    ok_fit_g, why_fg = assert_green(good_fit)
    checks = [("坏样本 relation_contact FAIL（前提）", ok_rel_red),
              (f"坏样本 {BUCKET}:{CHECK} FAIL(BLOCK) 且 measured 非空", ok_fit_red),
              (f"坏样本 {CHECK}.measured 点名 {RID}", fit_names_rid),
              ("对照 relation_contact PASS", ok_rel_g),
              (f"对照 {CHECK} PASS", ok_fit_g)]
    failed = [c for c, ok in checks if not ok]
    fmt = lambda fs: f"{fs[0].state}(measured={fs[0].measured})" if fs else "ABSENT"
    got = (f"退 0.30：relation_contact {fmt(bad_rel)} / {CHECK} {fmt(bad_fit)}；"
           f"贴合：relation_contact {fmt(good_rel)} / {CHECK} {fmt(good_fit)}")
    return result(NAME, EXPECT, not failed, got + (f"｜未成立：{failed}" if failed else ""), red,
                  expect_severity="BLOCK",
                  detail=" | ".join(x for x in (why_rel, why_fit, why_rg, why_fg) if x))


if __name__ == "__main__":
    import json
    r = run()
    print(json.dumps(r, ensure_ascii=False, indent=1))
    sys.exit(0 if r["passed"] else 1)
