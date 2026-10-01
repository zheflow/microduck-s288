#!/usr/bin/env python3
"""反例（09-13 F-L5-2 第二半）—— 头判据的起点必须是 tool_access 的**坐面**，不是 feature_hole_map 的刀心。

坑的形状：`features.yaml:hole_positions_mm`（= `fasteners.yaml:feature_hole_map[].holes`）是切刀中点 ——
40 mm 长的刀切 3.6 mm 的板，刀心常离件 15–19 mm，甚至在件外。l5_screwhead 以前拿它当射线起点：
14 组报"孔落在导出包围盒外"（head_seat unknown），F01/F06 从件外的点 `_measure_axis` 抓到别的圆柱
→"声明头侧与实测孔轴不共线"。fasteners.yaml 现在 25 组有探针实测的 `tool_access[].seats[]`
（point_export_local 在件上、outward_export_local 是头侧），头判据必须从它起判；刀心只当轴线锚
（校验坐面点在该孔轴上，横向偏差 > 1e-6 判 unknown，与 tools/gate/tool_access.py 同一规则）。

  A 对照   Ø2.4 通孔 + Ø4.4 沉 1.0 的板，刀心故意放在件外 20 mm，坐面在沉孔底
           → 修前 head_seat unknown（"落在导出包围盒外"）；修后 head_seat_exists / head_seat_flatness PASS，
             l5_function seat_flatness PASS，且沉孔条目（map_index 1，不在 head_locator_map_indices）不算"未声明"
  B 坏样本 同 A，坐面足印里挖一个 0.5 深的台阶 → head_seat_flatness FAIL(BLOCK)，measured 非空（≈0.5）
  C 错位   坐面点横向偏离孔轴 0.3 mm → head_seat / head_side unknown（坐面不在所绑定孔轴上，不猜）
"""
from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from _harness import FakeCtx, assert_green, assert_red, base_data, box, cyl, export_stl, findings, record, result  # noqa: E402

PART = "N03"
F_HOLE, F_CB = "N03-NEG-THRU24", "N03-NEG-CB44"
GID = "F_NEG_seat_origin"
NAME = "头判据起点 = tool_access 坐面（刀心在件外 20 mm）：对照绿、台阶坐面红、坐面偏离孔轴 unknown"
EXPECT = (f"A：L5/{GID}:head_seat_exists / head_seat_flatness / seat_flatness PASS（刀心在件外也能判）；"
          f"B：head_seat_flatness FAIL(BLOCK) measured≈0.5；C：head_seat/head_side unknown")

T = 3.6
CB_D, CB_DEPTH = 4.4, 1.0
HX, HY = 0.37, 0.21            # 孔心偏离板中心：足印采样点不许正好压在方板对角棱上（数值擦边，不是判据）
CUTTER_MID = [HX, HY, T + 20.0]  # 刀心：件外 20 mm（真数据里 40 mm 刀切 3.6 mm 板就是这样）
SEAT_Z = T - CB_DEPTH            # 沉孔底 = 坐面
STEP = (1.2, 1.2, 0.5)           # B：足印内一个 0.5 深的台阶（只压住几个采样点）


def _part(step: bool):
    m = box(20.0, 20.0, T, center=(0.0, 0.0, T / 2))
    m = m.difference(cyl(1.2, 40.0, center=(HX, HY, T / 2)))
    m = m.difference(cyl(CB_D / 2, CB_DEPTH * 2, center=(HX, HY, T)))          # 顶面沉 1.0
    if step:
        m = m.difference(box(*STEP, center=(HX + 1.6, HY, SEAT_Z - STEP[2] / 2)))
    return m


def _data(lateral_mm: float = 0.0):
    part = {"id": PART, "inventory_id": "N03_neg_seat", "material": "PLA", "print_orientation": "任意", "mirrored_copy": None}
    hole = {"id": F_HOLE, "part": PART, "kind": "screw_hole", "check_class": "screw_hole", "count": 1,
            "spec_verbatim": "1×Ø2.4 通（刀心在件外）",
            "geom": {"shape": "cylinder", "frame": "export_local", "nominal_d_mm": {"v": 2.4, "src": "measured"},
                     "through": True, "axis": "z", "depth_kind": "cutter_length", "depth_mm": {"v": 40.0, "src": "assumed"},
                     "pos": CUTTER_MID, "hole_positions_mm": [CUTTER_MID]}}
    cb = {"id": F_CB, "part": PART, "kind": "counterbore", "check_class": "counterbore", "count": 1,
          "spec_verbatim": f"1×Ø{CB_D} 沉 {CB_DEPTH}",
          "geom": {"shape": "cylinder", "frame": "export_local", "nominal_d_mm": {"v": CB_D, "src": "measured"},
                   "through": False, "axis": "z", "depth_kind": "blind_depth", "depth_mm": {"v": CB_DEPTH, "src": "measured"},
                   "pos": [HX, HY, T - CB_DEPTH / 2]}}
    fa = {"id": GID, "spec": "M2×6", "qty": 1, "joins": [f"{PART} 板", "servo_neg"],
          "joint_type": "s288_metal_thread",
          "engagement_mm": {"v": 6.0 - SEAT_Z, "src": "measured"}, "stack_mm": {"v": SEAT_Z, "src": "measured"},
          "status": "ok", "status_verbatim": "ok", "feature_ids": [F_HOLE, F_CB],
          "feature_hole_map": [{"feature_id": F_HOLE, "instance": 0, "holes": [CUTTER_MID], "n": 1},
                               {"feature_id": F_CB, "instance": 0, "holes": [[HX, HY, T - CB_DEPTH / 2]], "n": 1}],
          "head_locator_map_indices": [0],
          "tool_access": [{"state_ref": "tool:neg",
                           "seats": [{"feature_id": F_HOLE, "instance": "N03_neg_seat",
                                      "point_export_local": [HX + lateral_mm, HY, SEAT_Z], "outward_export_local": [0, 0, 1],
                                      "map_index": 0, "hole_index": 0}]}],
          "provenance": ["negatives/n_l5_seat_origin_cutter_mid"]}
    return base_data([part], [hole, cb], fasteners=[fa], relations=[])


def _observe(step: bool, lateral_mm: float = 0.0, tag: str = ""):
    import l2_features
    import l5_function
    import l5_screwhead
    l2_features._GEOM_CACHE.clear()
    stl = export_stl(_part(step), f"seat_origin_{tag}")
    data = _data(lateral_mm)
    rh = l5_screwhead.run(FakeCtx(data, {PART: stl}))
    rf = l5_function.run(FakeCtx(data, {PART: stl}))
    head = [f for f in findings(rh, subject=GID)
            if f.check.split(":")[0] in ("head_seat", "head_side", "head_seat_exists", "head_seat_flatness")]
    seat = [f for f in findings(rf, subject=GID) if f.check == "seat_flatness"]
    return dict(head=head, seat=seat, probed=rh.evidence.get("holes_probed"),
                summary={f.check: f"{f.state}/{f.measured}" for f in head + seat})


def _is_unknown(fs):
    return bool(fs) and all(f.state == "FAIL" and f.measured is None for f in fs)


def run() -> dict:
    A = _observe(False, tag="flat")
    B = _observe(True, tag="step")
    C = _observe(False, lateral_mm=0.3, tag="lateral")
    okA_e, whyA_e = assert_green(A["head"], check="head_seat_exists")
    okA_f, whyA_f = assert_green(A["head"], check="head_seat_flatness")
    okA_s, whyA_s = assert_green(A["seat"], check="seat_flatness")
    okA_u = not any(f.check in ("head_seat", "head_side") for f in A["head"])     # 沉孔条目不算未声明、刀心在件外不算摸不到
    okB, whyB, redB = assert_red(B["head"], severity="BLOCK", check="head_seat_flatness")
    okB_e, whyB_e = assert_green(B["head"], check="head_seat_exists")             # 台阶下面还有料，不是悬空
    okC = _is_unknown([f for f in C["head"] if f.check in ("head_seat", "head_side")]) and \
        not any(f.check in ("head_seat_exists", "head_seat_flatness") for f in C["head"])
    checks = [("A 对照：head_seat_exists PASS", okA_e), ("A 对照：head_seat_flatness PASS", okA_f),
              ("A 对照：l5_function seat_flatness PASS", okA_s),
              ("A 对照：没有 head_seat/head_side unknown（刀心在件外、沉孔条目都不算）", okA_u),
              ("A 对照：探到 1 个孔", A["probed"] == 1),
              ("B 台阶：head_seat_flatness FAIL(BLOCK, measured 非空)", okB),
              ("B 台阶：head_seat_exists 仍 PASS（不是悬空）", okB_e),
              ("C 坐面偏离孔轴 0.3：head_seat/head_side unknown，且不发 PASS", okC)]
    failed = [c for c, ok in checks if not ok]
    got = f"A {A['summary']}｜B {B['summary']}｜C {C['summary']}"
    return result(NAME, EXPECT, not failed, got + (f"｜未成立：{failed}" if failed else ""), redB,
                  expect_severity="BLOCK",
                  detail=" | ".join(x for x in (whyA_e, whyA_f, whyA_s, whyB, whyB_e) if x))


if __name__ == "__main__":
    import json
    r = run()
    print(json.dumps(r, ensure_ascii=False, indent=1))
    sys.exit(0 if r["passed"] else 1)
