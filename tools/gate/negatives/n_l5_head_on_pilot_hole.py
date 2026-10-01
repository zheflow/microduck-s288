#!/usr/bin/env python3
"""反例（09-13 F-L5-1）—— 接收件上的自攻底孔（pilot_hole）没有螺丝头，头判据不许在它上面量。

坑的形状：l5_screwhead 对 feature_hole_map 里**所有**孔做头判据，含 kind=pilot_hole 的接收孔。
接收件是整块料，"足印环带第一段连续实体"量出来是整个接收件的厚度（记分卡 F24：L05-F12 底孔
量出 stack 31.49 → engagement −19.49），stack_from_seat_crosscheck / engagement_from_measured_seat
假红 —— 红的是"在没有头的地方找头"，不是螺丝。

正确判据：按 `check_class or kind == "pilot_hole"` 跳过接收孔（与 l5_function._group_holes 的
thru/recv 分类一致），只判穿件那一侧；跳过的孔数写进 detail。

  对照  穿件 N03（T=3.6，Ø2.2 通孔，头侧 +z 已在 tool_access 声明）+ 接收件 H01（Ø1.6 底孔盲深 9.2，远端有台阶）
        → stack_from_seat_crosscheck / engagement_from_measured_seat 必须绿（修前假红：接收件厚 12 → 咬入 −4）
  坏样本 穿件的坐面真的悬空（footprint 一侧被切掉整个厚度）→ head_seat_exists 必须红（BLOCK，measured 非空）
"""
from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from _harness import FakeCtx, assert_green, assert_red, base_data, box, cyl, export_stl, findings, result  # noqa: E402

THRU, RECV = "N03", "H01"
F_THRU, F_PILOT = "N03-NEG-THRU", "H01-NEG-PILOT"
GID = "F_NEG_head_on_pilot"
NAME = "接收件底孔不判头：修前 pilot_hole 量出整块料 → 咬入假红；修后只判穿件 → 绿；穿件坐面悬空 → 红"
EXPECT = (f"L5/{GID}:stack_from_seat_crosscheck / engagement_from_measured_seat PASS（只判 {F_THRU}）；"
          f"坏样本 {GID}:head_seat_exists FAIL(BLOCK)")
_GREEN = ("stack_from_seat_crosscheck", "engagement_from_measured_seat", "head_seat_exists", "head_seat_flatness")

T = 3.6
RECV_T = 12.0
PILOT_D, PILOT_DEPTH = 1.6, 9.2
NOTCH_X = 1.5          # 坏样本：x > 1.5 的料整厚切掉 —— 头足印（r 1.1..2.0）一侧悬空
HX, HY = 0.37, 0.21    # 孔心偏离板中心：足印环带的 45° 采样点不许正好压在方板顶面三角形的对角棱上（射线擦边漏检是数值问题，不是判据）


def _thru(bad: bool):
    m = box(20.0, 20.0, T, center=(0.0, 0.0, T / 2)).difference(cyl(1.1, 40.0, center=(HX, HY, T / 2)))
    if bad:
        m = m.difference(box(20.0, 30.0, 30.0, center=(HX + NOTCH_X + 10.0, 0.0, T / 2)))
    return m


def _recv():
    m = box(20.0, 20.0, RECV_T, center=(0.0, 0.0, RECV_T / 2))
    # 盲孔从顶面 z=RECV_T 往下 PILOT_DEPTH；底部台阶 = 远端
    return m.difference(cyl(PILOT_D / 2, PILOT_DEPTH, center=(HX, HY, RECV_T - PILOT_DEPTH / 2)))


def _data():
    parts = [{"id": THRU, "inventory_id": "N03_neg_thru", "material": "PLA", "print_orientation": "任意", "mirrored_copy": None},
             {"id": RECV, "inventory_id": "H01_neg_recv", "material": "PLA", "print_orientation": "任意", "mirrored_copy": None}]
    thru = {"id": F_THRU, "part": THRU, "kind": "screw_hole", "check_class": "screw_hole", "count": 1,
            "spec_verbatim": "1×Ø2.2 通",
            "geom": {"shape": "cylinder", "frame": "export_local", "nominal_d_mm": {"v": 2.2, "src": "measured"},
                     "through": True, "axis": "z", "depth_kind": "cutter_length", "depth_mm": {"v": 40.0, "src": "assumed"},
                     "pos": [HX, HY, T / 2]}}
    pilot = {"id": F_PILOT, "part": RECV, "kind": "pilot_hole", "check_class": "pilot_hole", "count": 1,
             "spec_verbatim": f"1×Ø{PILOT_D} 自攻底孔 深 {PILOT_DEPTH}",
             "geom": {"shape": "cylinder", "frame": "export_local", "nominal_d_mm": {"v": PILOT_D, "src": "assumed"},
                      "through": False, "axis": "-z", "depth_kind": "blind_depth", "depth_mm": {"v": PILOT_DEPTH, "src": "assumed"},
                      "pos": [HX, HY, RECV_T - PILOT_DEPTH / 2]}}
    fa = {"id": GID, "spec": "M2×8", "qty": 1, "joins": [f"{THRU} 板", f"{RECV} Ø1.6 底孔 {PILOT_DEPTH}"],
          "joint_type": "pla_self_tap",
          "engagement_mm": {"v": 8.0 - T, "src": "measured"}, "stack_mm": {"v": T, "src": "measured"},
          "status": "ok", "status_verbatim": "ok", "feature_ids": [F_THRU, F_PILOT],
          "feature_hole_map": [{"feature_id": F_THRU, "instance": 0, "holes": [[HX, HY, T / 2]], "n": 1},
                               {"feature_id": F_PILOT, "instance": 0, "holes": [[HX, HY, RECV_T - PILOT_DEPTH / 2]], "n": 1}],
          "head_locator_map_indices": [0],
          "tool_access": [{"state_ref": "tool:neg",
                           "seats": [{"feature_id": F_THRU, "instance": "N03_neg_thru",
                                      "point_export_local": [HX, HY, T], "outward_export_local": [0, 0, 1],
                                      "map_index": 0, "hole_index": 0}]}],
          "provenance": ["negatives/n_l5_head_on_pilot_hole"]}
    return base_data(parts, [thru, pilot], fasteners=[fa], relations=[])


def _observe(bad: bool):
    import l2_features
    import l5_screwhead
    l2_features._GEOM_CACHE.clear()
    tag = "notch" if bad else "flat"
    st = export_stl(_thru(bad), f"pilot_thru_{tag}")
    sr = export_stl(_recv(), f"pilot_recv_{tag}")
    res = l5_screwhead.run(FakeCtx(_data(), {THRU: st, RECV: sr}))
    return res, findings(res, subject=GID)


def run() -> dict:
    res_g, good = _observe(False)
    res_b, bad = _observe(True)
    ok_green, why_green = assert_green([f for f in good if f.check in _GREEN])
    present = {c for c in _GREEN if any(f.check == c for f in good)}
    ok_red, why_red, red = assert_red([f for f in bad if f.check == "head_seat_exists"], severity="BLOCK")
    detail_mentions_skip = any("接收孔" in str(f.detail) or "pilot" in str(f.detail).lower()
                               for f in good if f.check in ("head_seat_exists", "stack_from_seat_crosscheck"))
    checks = [("对照：四条头判据都发了且 PASS", ok_green and present == set(_GREEN)),
              ("对照：detail 记了跳过的接收孔数", detail_mentions_skip),
              ("坏样本：head_seat_exists FAIL(BLOCK, measured 非空)", ok_red)]
    failed = [c for c, ok in checks if not ok]
    fmt = lambda fs: {f.check: f"{f.state}/{f.measured}" for f in fs if f.check in _GREEN}
    got = f"对照 {fmt(good)}（探孔 {res_g.evidence.get('holes_probed')}）；坏样本 {fmt(bad)}"
    return result(NAME, EXPECT, not failed, got + (f"｜未成立：{failed}" if failed else ""), red,
                  expect_severity="BLOCK", detail=" | ".join(x for x in (why_green, why_red) if x))


if __name__ == "__main__":
    import json
    r = run()
    print(json.dumps(r, ensure_ascii=False, indent=1))
    sys.exit(0 if r["passed"] else 1)
