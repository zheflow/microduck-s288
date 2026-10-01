#!/usr/bin/env python3
"""反例（09-13）—— 头侧匹配键必须带 instance；沉孔条目不是另一颗螺丝。

坑的形状（主线程 09-13 查出，F02/F03/F10 报"6/12 孔头侧未声明或左右冲突"）：
  · `declared_head_sides` 的 key 是 (feature_id, map_index, hole_index)，没有 instance —— 左件 `hip` 与右件
    `hip_R` 的坐面落到同一个 key 上，方向一样才侥幸不冲突；真要抓的"同一 instance 同一孔给了两个方向"反而
    和"左右两实例"混在一起；
  · 头判据把 feature_hole_map 的**全部**孔当螺丝：F02 的 12 个 coords = L01-F04 6 孔 + L01-F05 沉孔条目 6 孔，
    `head_locator_map_indices: [0]` 明说只有 map 0 是头侧孔全集，沉孔条目本来就没有 seats → 被算成"6 个未声明"。

正确判据：key = (feature_id, instance, map_index, hole_index)；"冲突"只在**同一 instance** 同一孔给了不同 outward 时
成立；头侧判据只覆盖 head_locator_map_indices 里的孔；左右实例共用同一份 STL、export_local 坐标相同 →
几何量一次、记录覆盖了哪些 instance（evidence 不重复计）。

  A 左右两实例同一孔同 outward + 沉孔条目 → 不算冲突、不算未声明：head_seat_exists / head_seat_flatness PASS，
    探孔数 = 1（不是 2），detail 记两个 instance
  B 同一 instance 同一孔两条 outward 相反 → head_side unknown（冲突），不发 PASS
  C 右实例少一个坐面 → head_side unknown（该 instance 的这颗螺丝没有坐面），不猜
"""
from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from _harness import FakeCtx, assert_green, base_data, box, cyl, export_stl, findings, record, result  # noqa: E402

PART = "L01"
F_HOLE, F_CB = "L01-NEG-HORN", "L01-NEG-CB"
GID = "F_NEG_instance_key"
NAME = "头侧键带 instance：L/R 同孔同向不算冲突、沉孔条目不算未声明；同 instance 反向 → unknown；右实例缺坐面 → unknown"
EXPECT = (f"A：L5/{GID}:head_seat_exists / head_seat_flatness PASS 且探孔数 2（左右共用 STL 只量一次）；"
          f"B/C：head_side unknown（measured 为空是设计）")

T = 3.6
CB_D, CB_DEPTH = 4.4, 1.0
HOLES = [(-4.0 + 0.37, 0.21), (4.0 + 0.37, 0.21)]
SEAT_Z = T - CB_DEPTH
L, R = "yaw2roll", "yaw2roll_R"


def _part():
    m = box(24.0, 16.0, T, center=(0.0, 0.0, T / 2))
    for x, y in HOLES:
        m = m.difference(cyl(1.2, 40.0, center=(x, y, T / 2)))
        m = m.difference(cyl(CB_D / 2, CB_DEPTH * 2, center=(x, y, T)))
    return m


def _seat(inst, i, outward=(0, 0, 1)):
    x, y = HOLES[i]
    return {"feature_id": F_HOLE, "instance": inst, "point_export_local": [x, y, SEAT_Z],
            "outward_export_local": list(outward), "map_index": 0, "hole_index": i}


def _data(variant: str):
    part = {"id": PART, "inventory_id": "L01_neg_inst", "material": "PLA", "print_orientation": "任意",
            "mirrored_copy": "mirror_y", "placed_instances": [L, R], "qty": 2}
    hole = {"id": F_HOLE, "part": PART, "kind": "horn_hole", "check_class": "horn_hole", "count": 2,
            "spec_verbatim": "2×Ø2.4 通",
            "geom": {"shape": "cylinder", "frame": "export_local", "nominal_d_mm": {"v": 2.4, "src": "measured"},
                     "through": True, "axis": "z", "depth_kind": "cutter_length", "depth_mm": {"v": 40.0, "src": "assumed"},
                     "pos": [0.0, 0.0, T + 18.0], "hole_positions_mm": [[x, y, T + 18.0] for x, y in HOLES]}}
    cb = {"id": F_CB, "part": PART, "kind": "counterbore", "check_class": "counterbore", "count": 2,
          "spec_verbatim": f"2×Ø{CB_D} 沉 {CB_DEPTH}",
          "geom": {"shape": "cylinder", "frame": "export_local", "nominal_d_mm": {"v": CB_D, "src": "measured"},
                   "through": False, "axis": "z", "depth_kind": "blind_depth", "depth_mm": {"v": CB_DEPTH, "src": "measured"},
                   "pos": [0.0, 0.0, T - CB_DEPTH / 2]}}
    seats_L = [_seat(L, 0), _seat(L, 1)]
    seats_R = [_seat(R, 0), _seat(R, 1)]
    if variant == "conflict":          # 同一 instance 同一孔第二条 outward 相反
        seats_L.append(_seat(L, 0, outward=(0, 0, -1)))
    if variant == "missing_R":         # 右实例少一个坐面
        seats_R = seats_R[:1]
    fa = {"id": GID, "spec": "M2×6", "qty": 4, "joins": [f"{PART} 盘", "servo_neg"],
          "joint_type": "s288_metal_thread",
          "engagement_mm": {"v": 6.0 - SEAT_Z, "src": "measured"}, "stack_mm": {"v": SEAT_Z, "src": "measured"},
          "status": "ok", "status_verbatim": "ok", "feature_ids": [F_HOLE, F_CB],
          "feature_hole_map": [{"feature_id": F_HOLE, "holes": [[x, y, T + 18.0] for x, y in HOLES], "n": 2},
                               {"feature_id": F_CB, "holes": [[x, y, T - CB_DEPTH / 2] for x, y in HOLES], "n": 2}],
          "head_locator_map_indices": [0],
          "tool_access": [{"state_ref": "tool:neg_L", "seats": seats_L},
                          {"state_ref": "tool:neg_R", "seats": seats_R}],
          "provenance": ["negatives/n_l5_seat_instance_key"]}
    return base_data([part], [hole, cb], fasteners=[fa], relations=[])


def _observe(variant: str):
    import l2_features
    import l5_screwhead
    l2_features._GEOM_CACHE.clear()
    stl = export_stl(_part(), f"inst_key_{variant}")
    res = l5_screwhead.run(FakeCtx(_data(variant), {PART: stl}))
    fs = [f for f in findings(res, subject=GID)
          if f.check.split(":")[0] in ("head_seat", "head_side", "head_seat_exists", "head_seat_flatness")]
    return dict(fs=fs, probed=res.evidence.get("holes_probed"),
                summary={f.check: f"{f.state}/{f.measured}" for f in fs})


def _is_unknown(fs):
    return bool(fs) and all(f.state == "FAIL" and f.measured is None for f in fs)


def run() -> dict:
    A = _observe("ok")
    B = _observe("conflict")
    C = _observe("missing_R")
    okA_e, whyA_e = assert_green(A["fs"], check="head_seat_exists")
    okA_f, whyA_f = assert_green(A["fs"], check="head_seat_flatness")
    okA_u = not any(f.check in ("head_seat", "head_side") for f in A["fs"])
    okA_n = A["probed"] == 2
    okA_inst = any(L in str(f.detail) and R in str(f.detail) for f in A["fs"] if f.check == "head_seat_exists")
    sideB = [f for f in B["fs"] if f.check == "head_side"]
    okB = _is_unknown(sideB) and not any(f.check in ("head_seat_exists", "head_seat_flatness") for f in B["fs"]) \
        and any("冲突" in str(f.detail) for f in sideB)
    sideC = [f for f in C["fs"] if f.check == "head_side"]
    okC = _is_unknown(sideC) and not any(f.check in ("head_seat_exists", "head_seat_flatness") for f in C["fs"]) \
        and any(R in str(f.detail) for f in sideC)
    checks = [("A L/R 同向：head_seat_exists PASS", okA_e), ("A L/R 同向：head_seat_flatness PASS", okA_f),
              ("A：没有 head_seat/head_side unknown（沉孔条目不算未声明、左右不算冲突）", okA_u),
              ("A：探孔数 2 = 左右共用 STL 只量一次", okA_n),
              ("A：detail 记录覆盖了 yaw2roll 与 yaw2roll_R 两个 instance", okA_inst),
              ("B 同 instance 反向：head_side unknown 并写明冲突，不发 PASS", okB),
              ("C 右实例缺坐面：head_side unknown 点名 yaw2roll_R，不发 PASS", okC)]
    failed = [c for c, ok in checks if not ok]
    red = [record(f) for f in sideB + sideC if f.state == "FAIL"]
    got = f"A {A['summary']} 探孔 {A['probed']}｜B {B['summary']}｜C {C['summary']}"
    return result(NAME, EXPECT, not failed, got + (f"｜未成立：{failed}" if failed else ""), red,
                  expect_severity="BLOCK",
                  allow_unknown_red="B/C 验的是『同 instance 方向冲突 / 某实例缺坐面 → unknown，不猜』；A 是对照绿",
                  detail=" | ".join(x for x in (whyA_e, whyA_f) if x))


if __name__ == "__main__":
    import json
    r = run()
    print(json.dumps(r, ensure_ascii=False, indent=1))
    sys.exit(0 if r["passed"] else 1)
