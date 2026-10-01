#!/usr/bin/env python3
"""反例（09-13 F-L5-4）—— 阈值的 src 必须进判据：没来源的阈值不许判，拍脑袋的阈值判了要在记分卡上看得见。

坑的形状：tolerances.yaml 里 `fits.*.*.target_range_mm` 与 `feature_check_tolerances.*` 这些**阈值**节点很多没有
`src`；层读阈值时只看 min/max，"nominal 落在区间内 → PASS"对着一个没人知道哪来的区间也绿。

正确判据：层读阈值时把 src 写进 criterion（"阈值 src=measured/datasheet/assumed/缺"）；阈值节点**没有 src** →
该判据 unknown（"阈值没有来源"）；src=assumed → 判据照跑，但另发一条 `threshold_assumed:<桶或键>` FAIL(WARN)，
measured = 阈值本身。

  A src=assumed（真数据现状）  fits.servo.pad_contact:fit_value PASS 且 criterion 含 "src=assumed"；
                               threshold_assumed:fits.servo.pad_contact.target_range_mm FAIL(WARN) measured 非空；
                               l5_screwhead 的 head_seat_flatness PASS + threshold_assumed:feature_check_tolerances.seat_flatness_spread_mm FAIL(WARN)
  B 删掉 src                    fit_value unknown（"阈值没有来源"），不发 threshold_assumed；head_seat_flatness unknown
  C src=measured               fit_value PASS，不发 threshold_assumed
"""
from __future__ import annotations

import copy
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from _harness import FakeCtx, assert_green, assert_red, base_data, box, cyl, export_stl, findings, result  # noqa: E402

BUCKET = "fits.servo.pad_contact"
FCT_KEY = "feature_check_tolerances.seat_flatness_spread_mm"
PART, FID, GID = "N03", "N03-NEG-SRC", "F_NEG_threshold_src"
NAME = "阈值 src 进判据：assumed → threshold_assumed FAIL(WARN)；无 src → fit_value / head_seat_flatness unknown；measured → 不发 WARN"
EXPECT = (f"A：L5/{BUCKET}:threshold_assumed:{BUCKET}.target_range_mm FAIL(WARN) + L5/_thresholds:threshold_assumed:{FCT_KEY} FAIL(WARN)；"
          f"B：{BUCKET}:fit_value unknown、{GID}:head_seat_flatness unknown；C：无 threshold_assumed")
T, HX, HY = 3.6, 0.37, 0.21


def _part():
    return box(20.0, 20.0, T, center=(0.0, 0.0, T / 2)).difference(cyl(1.2, 40.0, center=(HX, HY, T / 2)))


def _data(src):
    """src: 'keep'（真数据现状）| None（删掉 src）| 'measured'。同时作用到 fits 桶与 feature_check 阈值。"""
    part = {"id": PART, "inventory_id": "N03_neg_src", "material": "PLA", "print_orientation": "任意", "mirrored_copy": None}
    feat = {"id": FID, "part": PART, "kind": "screw_hole", "check_class": "screw_hole", "count": 1, "spec_verbatim": "1×Ø2.4 通",
            "geom": {"shape": "cylinder", "frame": "export_local", "nominal_d_mm": {"v": 2.4, "src": "measured"},
                     "through": True, "axis": "z", "depth_kind": "cutter_length", "depth_mm": {"v": 40.0, "src": "assumed"},
                     "pos": [HX, HY, T / 2], "hole_positions_mm": [[HX, HY, T / 2]]}}
    fa = {"id": GID, "spec": "M2×6", "qty": 1, "joins": [f"{PART} 板", "servo_neg"], "joint_type": "s288_metal_thread",
          "engagement_mm": {"v": 6.0 - T, "src": "measured"}, "stack_mm": {"v": T, "src": "measured"},
          "status": "ok", "status_verbatim": "ok", "feature_ids": [FID],
          "feature_hole_map": [{"feature_id": FID, "holes": [[HX, HY, T / 2]], "n": 1}],
          "head_locator_map_indices": [0],
          "tool_access": [{"state_ref": "tool:neg", "seats": [{"feature_id": FID, "instance": "N03_neg_src",
                                                                "point_export_local": [HX, HY, T], "outward_export_local": [0, 0, 1],
                                                                "map_index": 0, "hole_index": 0}]}],
          "provenance": ["negatives/n_l5_threshold_src"]}
    d = base_data([part], [feat], fasteners=[fa], relations=[])
    d["tolerances"] = copy.deepcopy(d["tolerances"])
    nodes = [d["tolerances"]["fits"]["servo"]["pad_contact"]["target_range_mm"],
             d["tolerances"]["feature_check_tolerances"]["seat_flatness_spread_mm"]]
    for n in nodes:
        if src == "keep":
            continue
        n.pop("src", None)
        if src is not None:
            n["src"] = src
    return d


def _observe(src, tag):
    import l2_features
    import l5_function
    import l5_screwhead
    l2_features._GEOM_CACHE.clear()
    stl = export_stl(_part(), f"thr_src_{tag}")
    data = _data(src)
    rf = l5_function.run(FakeCtx(data, {PART: stl}))
    rh = l5_screwhead.run(FakeCtx(data, {PART: stl}))
    return dict(fit=findings(rf, subject=BUCKET, check="fit_value"),
                warn_f=[f for f in rf.findings if f.check.startswith("threshold_assumed:") and BUCKET in f.check],
                flat=findings(rh, subject=GID, check="head_seat_flatness"),
                warn_h=[f for f in rh.findings if f.check.startswith("threshold_assumed:") and FCT_KEY in f.check],
                any_warn_f=[f for f in rf.findings if f.check.startswith("threshold_assumed:") and BUCKET in f.check],
                any_warn_h=[f for f in rh.findings if f.check.startswith("threshold_assumed:") and FCT_KEY in f.check])


def _is_unknown(fs):
    return bool(fs) and all(f.state == "FAIL" and f.measured is None for f in fs)


def _fmt(fs):
    return [f"{f.subject}:{f.check}={f.state}/{f.measured}" for f in fs]


def run() -> dict:
    A = _observe("keep", "assumed")
    B = _observe(None, "nosrc")
    C = _observe("measured", "measured")
    okA_fit = bool(A["fit"]) and all(f.state == "PASS" and "src=assumed" in str(f.criterion) for f in A["fit"])
    okA_w, whyA_w, redA_w = assert_red(A["warn_f"], severity="WARN")
    okA_flat, whyA_flat = assert_green(A["flat"])
    okA_wh, whyA_wh, redA_wh = assert_red(A["warn_h"], severity="WARN")
    okB_fit = _is_unknown(B["fit"]) and any("没有来源" in str(f.detail) for f in B["fit"]) and not B["any_warn_f"]
    okB_flat = _is_unknown(B["flat"]) and any("没有来源" in str(f.detail) for f in B["flat"]) and not B["any_warn_h"]
    okC_fit = bool(C["fit"]) and all(f.state == "PASS" and "src=measured" in str(f.criterion) for f in C["fit"]) and not C["any_warn_f"]
    okC_flat = bool(C["flat"]) and all(f.state == "PASS" for f in C["flat"]) and not C["any_warn_h"]
    checks = [(f"A assumed：{BUCKET}:fit_value PASS 且 criterion 含 src=assumed", okA_fit),
              (f"A assumed：threshold_assumed:{BUCKET}.target_range_mm FAIL(WARN) measured 非空", okA_w),
              (f"A assumed：{GID}:head_seat_flatness PASS", okA_flat),
              (f"A assumed：threshold_assumed:{FCT_KEY} FAIL(WARN) measured 非空", okA_wh),
              (f"B 无 src：{BUCKET}:fit_value unknown（阈值没有来源），无 threshold_assumed", okB_fit),
              (f"B 无 src：{GID}:head_seat_flatness unknown（阈值没有来源），无 threshold_assumed", okB_flat),
              (f"C measured：fit_value PASS 且 criterion 含 src=measured，无 threshold_assumed", okC_fit),
              (f"C measured：head_seat_flatness PASS，无 threshold_assumed", okC_flat)]
    failed = [c for c, ok in checks if not ok]
    got = (f"A fit={_fmt(A['fit'])} warn={_fmt(A['warn_f'])} flat={_fmt(A['flat'])} warn_h={_fmt(A['warn_h'])}｜"
           f"B fit={_fmt(B['fit'])} flat={_fmt(B['flat'])}｜C fit={_fmt(C['fit'])} flat={_fmt(C['flat'])} warn={_fmt(C['any_warn_f'] + C['any_warn_h'])}")
    out = result(NAME, EXPECT, not failed, got + (f"｜未成立：{failed}" if failed else ""), redA_w + redA_wh,
                 expect_severity="WARN", detail=" | ".join(x for x in (whyA_w, whyA_flat, whyA_wh) if x))
    out["assertions"] = [{"claim": c, "ok": bool(ok)} for c, ok in checks]
    return out


if __name__ == "__main__":
    import json
    r = run()
    print(json.dumps(r, ensure_ascii=False, indent=1))
    sys.exit(0 if r["passed"] else 1)
