#!/usr/bin/env python3
"""反例 FG12 —— 螺丝附近 1.1 mm 是空的，更远处把起子入口封死，刀路判据仍然绿。

内存几何（**不碰 cad/duck_s288 的整鸭件**，形状照 F14 的处境搭：Ø13.8×1.1 是个**沉坑**）：
    P01 承件 plate  x ∈ [−2, 0]，y/z ∈ [−10, 10]；螺丝孔在世界 (0,0,0)，螺丝头朝 +x
    P02 远壁  wall  x ∈ [ 5, 7]，y/z ∈ [−10, 10]  —— 离螺丝头 5 mm，正好把起子入口封死
    tool_envelope: driver=PH0，channel_d_mm=4.0，channel_len_mm=**1.1**（只覆盖沉坑那一段）
    drive_direction: "PH0 从 +x"

修前（l4_assembly.py:719-742, 803-825）：刀路只建 Ø4×**1.1** 的一小段（还允许从"最远表面之外"
起扫）、可见性射线也只射 1.1 mm —— 沉坑里是空的 → `tool_path` 0 mm³、`screw_head_visible` 1/1 → 全绿。

修后：`tool_path_to_outside` 从**真实螺丝头一侧**（孔坐标本身）沿刀轴一路量到**整机包围盒之外**
（这里 7 mm）→ 撞上远壁 → 红；`screw_head_visible` 同样射到包围盒外 → 红；
另外 channel_len_mm=1.1 < 所需 7 mm，`tool_reach_declared` 判 unknown（BLOCK），
明说缺的是这颗螺丝的起子**杆长**与**批头直径**（fasteners.yaml 没有这两个字段，层里不替 data 编）。

好样本（防恒红）：
    good   把远壁挪到 y ∈ [20, 30]（只改这一个盒的 y）→ `tool_path_to_outside` 必须绿；
    good2  再把 available_shaft_len_mm 补到 7.5（≥ 所需 7）→ 整格必须绿（连 tool_reach_declared 一起）。

判据阈值取真实的 tools/gate/data/tolerances.yaml，本文件不写死任何数字判据。
"""
from __future__ import annotations
import sys
from pathlib import Path

_HERE = Path(__file__).resolve()
GATE = _HERE.parents[1]
for _p in (str(GATE), str(GATE.parents[1])):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import core                                                        # noqa: E402

NAME = "螺丝旁 1.1 mm 空、远处封死起子入口，刀路仍绿（FG12）"
EXPECT = "L4/FX:tool_path_to_outside 必须红（从螺丝头一路量到整机包围盒之外）"
_TMP = core.ROOT / "tools/gate/out/_neg_tmp"


def _scene(tag, wall_lo, wall_hi):
    import numpy as np
    import trimesh
    d = _TMP / f"l4_tool_{tag}"
    d.mkdir(parents=True, exist_ok=True)
    for p in d.glob("*.stl"):
        p.unlink()
    for name, (lo, hi) in {"plate": ((-2.0, -10.0, -10.0), (0.0, 10.0, 10.0)),
                           "wall": (wall_lo, wall_hi)}.items():
        lo = np.asarray(lo, float)
        hi = np.asarray(hi, float)
        m = trimesh.creation.box(extents=(hi - lo))
        m.apply_translation((lo + hi) / 2.0)
        m.export(str(d / f"{name}.stl"), file_type="stl")
    return d


def _ctx(placed_dir, channel_len):
    from _harness import FakeCtx                                   # noqa: PLC0415
    data = core.load_data()                                        # 真公差表
    data["parts"] = {"parts": [
        {"id": "P01", "inventory_id": "plate", "build_fn": [], "qty": 1, "placed_instances": ["plate"]},
        {"id": "P02", "inventory_id": "wall", "build_fn": [], "qty": 1, "placed_instances": ["wall"]},
    ]}
    data["features"] = {"features": [{"id": "P01-F01", "part": "P01", "geom": {"nominal_d_mm": {"v": 2.2}, "axis": "x"}}], "frames": {"per_part": {"P01": {
        "export_body": "plate", "stl": "P01_plate.stl",
        "world_to_export_local_R": [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
        "world_to_export_local_t": [0.0, 0.0, 0.0]}}}}
    data["fasteners"] = {"fasteners": [{
        "id": "FX", "spec": "M2×6", "qty": 1,
        "joins": ["反例：P01 → P02"], "joint_type": "s288_metal_thread",
        "tool_envelope": {"driver": "PH0",
                          "bit_d_mm": {"v": 4.0, "src": "assumed"},
                          "available_shaft_len_mm": {"v": float(channel_len), "src": "assumed"},
                          "channel_d_mm": {"v": 4.0, "src": "反例自造"},
                          "channel_len_mm": {"v": float(channel_len), "src": "反例自造"},
                          "verified_by_geometry": False, "note": "反例：沉坑深 1.1"},
        "drive_direction": "PH0 从 +x", "location": "反例沉坑",
        "holes": ["#0"], "status": "ok", "feature_ids": ["P01-F01"],
        "head_locator_map_indices": [0],
        "feature_hole_map": [{"feature_id": "P01-F01", "selector": "反例 1 个孔",
                              "holes": [[0.0, 0.0, 0.0]], "n": 1}],
        "tool_access": [{"state_ref": "tool:test", "seats": [{"feature_id": "P01-F01", "instance": "plate",
                          "point_export_local": [0, 0, 0], "outward_export_local": [1, 0, 0], "map_index": 0, "hole_index": 0}]}],
    }]}
    data["keepouts"] = {"keepouts": []}
    data["waivers"] = {"waivers": []}
    # 本反例只查刀路；装配序放一条 path 为空的占位步（层会判 unknown，不跑扫掠），
    # 因为 assembly_order 整个为空时 run() 会直接返回，刀路那一段就跑不到了
    data["assembly"] = {"assembly_order": [{
        "step": 1, "action": "反例占位：本反例只查刀路", "parts": ["P01", "P02"],
        "already_installed": [], "direction": "+x", "path": {}, "verified": False,
    }], "disassembly_order": [], "tool_states": {"test": {"workspace": "test", "present": ["plate", "wall"]}}}
    return FakeCtx(data, {}, {p.stem: p for p in sorted(placed_dir.glob("*.stl"))})


def _run_layer(placed, channel_len):
    import layers.l4_assembly as L4                                # noqa: PLC0415
    old = L4.PLACED
    try:
        L4.PLACED = placed
        return L4.run(_ctx(placed, channel_len))
    finally:
        L4.PLACED = old


def run() -> dict:
    from _harness import findings, worst_state, summarize, assert_red, assert_green, result   # noqa: PLC0415
    on_axis = ((5.0, -10.0, -10.0), (7.0, 10.0, 10.0))
    off_axis = ((5.0, 20.0, -10.0), (7.0, 30.0, 10.0))             # 只改这一个盒的 y
    cases = {"bad": (on_axis, 1.1), "good": (off_axis, 1.1), "good2": (off_axis, 7.5)}
    st, ev, fs = {}, {}, {}
    for tag, (wall, cl) in cases.items():
        r = _run_layer(_scene(tag, *wall), cl)
        fs[tag] = findings(r, subject="FX", check="tool_path_to_outside")
        fs[tag + "_cell"] = findings(r, subject="FX")
        st[tag] = worst_state(fs[tag])
        st[tag + "_cell"] = worst_state(fs[tag + "_cell"])
        ev[tag] = summarize(fs[tag + "_cell"], n=5)

    ok_red, why_red, red = assert_red(fs["bad"], severity="BLOCK", require_measured=True)
    ok_good, why_good = assert_green(fs["good"])
    ok_good2, why_good2 = assert_green(fs["good2_cell"])
    passed = ok_red and ok_good and ok_good2
    got = (f"坏样本 tool_path_to_outside={'红' if ok_red else why_red}（峰值={red[0]['measured'] if red else None}，整格 {st['bad_cell']}）；"
           f"好样本（远壁挪开）={'绿' if ok_good else why_good}（整格 {st['good_cell']} —— available_shaft_len_mm 仍只声明 1.1 < "
           f"所需 7，tool_reach_declared 照样判 FAIL，这是对的）；"
           f"好样本 2（远壁挪开 + available_shaft_len_mm 补到 7.5）整格={'绿' if ok_good2 else why_good2}")
    if st["bad"] == "ABSENT":
        got = ("坏样本里**根本没有 tool_path_to_outside 这条判据**（刀路只查沉坑里的一小段 = 未修）；"
               f"当时整格是 {st['bad_cell']} —— " + ev["bad"])
    return result(NAME, EXPECT, passed, got=got, red=red, expect_severity="BLOCK",
                  detail=" || ".join(f"{k}: {v}" for k, v in ev.items()))


if __name__ == "__main__":
    import json
    r = run()
    print(json.dumps(r, ensure_ascii=False, indent=1))
    sys.exit(0 if r["passed"] else 1)
