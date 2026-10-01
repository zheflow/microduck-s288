#!/usr/bin/env python3
"""反例 —— 沉头锥面坐面（hr41 落盘 2026-09-25，主设计 Lane C；首例 F34 90° 沉头机牙）：坐面被挡必须红，好锥口必须绿。

经层入口 `tool_access.run_tool_access`（l4_assembly.run 对刀路那一段调用的就是它）。内存几何（不碰整鸭件）：
    P01 承件 plate  x ∈ [−3, 0]，y/z ∈ [−10, 10]；Ø2.4 过孔沿 x 穿透；x=0 面上 90° 锥口 Ø4.2 深 0.9（= jaw.jaw_cs_cuts 同尺寸）；头侧 +x
    坐面声明 seats[].cone = {included_angle_deg 90, top_d_mm 4.2, head_d_mm 3.8}，坐面点 = 孔轴上锥口顶面 (0,0,0)
  good       好锥口 → FX:tool_origin_on_seat PASS（对照；锥面足印 3 圈 × 24 向，料侧有料、空腔侧为空）
  blocked    锥口没切（坐面被件自己的料挡住：锥面空腔侧全是料）→ FX:tool_origin_on_seat FAIL(BLOCK)，measured = 逐颗足印计数
  capped     锥口在，但另一件 cap（x ∈ [0.5, 1.5] 的板）盖在锥口外 → FX:tool_path_to_outside FAIL(BLOCK)，measured = 峰值交集
  flat       同 good 几何但不写 cone（按平面环带判）→ tool_origin_on_seat FAIL：旧平面判据对锥面必然红（hr41 §8 的原话），只记录不进 red
  no_src     cone.top_d_mm 缺 src → FX:tool_reference unknown（FAIL、measured=None），只记录不进 red
同轴异位（F34 两毂同一轴线；tool_access 的"同一实体同一孔轴 = 同一孔"规则按孔深区分，只在特征声明了 depth_mm 时）：
  coax_ok    一件 U 形：x∈[−3,0] 与 x∈[−20,−17] 两片各一个沉头孔、同在 x 轴上、头侧相反，孔深 3 → 轴向相距 17 > 3 → 两颗都判，坐面绿
  coax_dup   同一件，第二个锚点只挪 1.0（≤ 孔深 3）→ tool_reference unknown（仍当同一孔凑两颗）
  coax_nodep 不写 depth_mm → tool_reference unknown（原规则不变）
判据阈值取真实 tools/gate/data/tolerances.yaml；本文件不写数字判据。
"""
from __future__ import annotations
import json
import math
import sys
from pathlib import Path

_HERE = Path(__file__).resolve()
GATE = _HERE.parents[1]
for _p in (str(_HERE.parent), str(GATE), str(GATE.parents[1])):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import core                                                                    # noqa: E402
from core import LayerResult                                                   # noqa: E402
from _harness import FakeCtx, findings, assert_red, assert_green, result, record  # noqa: E402

NAME = "沉头锥面坐面：锥口被料挡 / 锥口外被别件盖住必须红，好锥口绿（hr41 F34 90° 沉头）"
EXPECT = "L4/FX:tool_origin_on_seat（锥口没切）与 FX:tool_path_to_outside（锥口外有盖）FAIL(BLOCK) 带数；好锥口 tool_origin_on_seat PASS"
_TMP = core.ROOT / "tools/gate/out/_neg_tmp"


def _plate(countersunk=True):
    import numpy as np
    import trimesh
    plate = trimesh.creation.box(extents=(3.0, 20.0, 20.0))
    plate.apply_translation((-1.5, 0.0, 0.0))
    hole = trimesh.creation.cylinder(radius=1.2, height=10.0, sections=96)
    hole.apply_transform(trimesh.transformations.rotation_matrix(math.pi / 2, [0, 1, 0]))
    m = plate.difference(hole)
    if countersunk:
        a = np.linspace(0, 2 * math.pi, 96, endpoint=False)
        pts = [(x, d / 2 * math.cos(t), d / 2 * math.sin(t)) for d, x in ((2.4, -0.9), (4.2, 0.0), (4.2, 1.0)) for t in a]
        m = m.difference(trimesh.convex.convex_hull(np.array(pts)))
    return m


def _scene(tag, countersunk=True, cap=False):
    import trimesh
    d = _TMP / f"l4_cone_{tag}"
    d.mkdir(parents=True, exist_ok=True)
    for p in d.glob("*.stl"):
        p.unlink()
    _plate(countersunk).export(str(d / "plate.stl"), file_type="stl")
    if cap:
        c = trimesh.creation.box(extents=(1.0, 20.0, 20.0))
        c.apply_translation((1.0, 0.0, 0.0))
        c.export(str(d / "cap.stl"), file_type="stl")
    return d


def _ctx(placed_dir, cone=True, cone_src=True):
    data = core.load_data()                                        # 真公差表 / 真 frozen（M2 头径）
    parts = [{"id": "P01", "inventory_id": "plate", "build_fn": [], "qty": 1, "placed_instances": ["plate"]}]
    present = ["plate"]
    if (placed_dir / "cap.stl").exists():
        parts.append({"id": "P02", "inventory_id": "cap", "build_fn": [], "qty": 1, "placed_instances": ["cap"]})
        present.append("cap")
    data["parts"] = {"parts": parts}
    data["features"] = {"features": [{"id": "P01-F01", "part": "P01", "kind": "screw_hole",
                                      "geom": {"nominal_d_mm": {"v": 2.4}, "axis": "x"}}],
                        "frames": {"per_part": {"P01": {
                            "export_body": "plate", "stl": "P01_plate.stl",
                            "world_to_export_local_R": [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
                            "world_to_export_local_t": [0.0, 0.0, 0.0]}}}}
    seat = {"feature_id": "P01-F01", "instance": "plate", "map_index": 0, "hole_index": 0,
            "point_export_local": [0.0, 0.0, 0.0], "outward_export_local": [1.0, 0.0, 0.0]}
    if cone:
        seat["cone"] = {"included_angle_deg": {"v": 90.0, "src": "反例：jaw.jaw_cs_cuts 同尺寸"},
                        "top_d_mm": ({"v": 4.2, "src": "反例"} if cone_src else {"v": 4.2}),
                        "head_d_mm": {"v": 3.8, "src": "反例：ISO 7046-1 dk"}}
    data["fasteners"] = {"fasteners": [{
        "id": "FX", "spec": "M2×5 沉头", "qty": 1, "joins": ["反例：沉头锥面"], "joint_type": "machine_screw_hex_nut",
        "tool_envelope": {"driver": "PH0", "bit_d_mm": {"v": 4.0, "src": "assumed"},
                          "available_shaft_len_mm": {"v": 60.0, "src": "反例"}},
        "drive_direction": "PH0 从 +x", "holes": ["#0"], "feature_ids": ["P01-F01"], "head_locator_map_indices": [0],
        "feature_hole_map": [{"feature_id": "P01-F01", "selector": "反例 1 个沉头孔", "holes": [[-1.5, 0.0, 0.0]], "n": 1}],
        "tool_access": [{"state_ref": "tool:test", "seats": [seat]}]}]}
    data["keepouts"] = {"keepouts": []}
    data["waivers"] = {"waivers": []}
    data["assembly"] = {"assembly_order": [{"step": 1, "action": "反例占位：只查刀路/坐面", "parts": [p["id"] for p in parts],
                                            "already_installed": [], "direction": "+x", "path": {}, "verified": False}],
                        "disassembly_order": [], "tool_states": {"test": {"workspace": "test", "present": present}}}
    return FakeCtx(data, {}, {p.stem: p for p in sorted(placed_dir.glob("*.stl"))})


def _observe(tag, countersunk=True, cap=False, cone=True, cone_src=True):
    import layers.l4_assembly as L4                                # noqa: PLC0415
    from tool_access import run_tool_access                        # noqa: PLC0415
    d = _scene(tag, countersunk, cap)
    old = L4.PLACED
    try:
        L4.PLACED = d
        ctx = _ctx(d, cone, cone_src)
        geo = L4._Geo(ctx)
        res = LayerResult(4, "tool")
        run_tool_access(res, ctx, geo, L4._Names(ctx, geo), ctx.data["assembly"], 0.05)
        return res
    finally:
        L4.PLACED = old


def _cs_cut(x_face, sign):
    """x = x_face 面上的 90° 锥口（Ø4.2 → 深 0.9 处 Ø2.4），头侧朝 sign·x。"""
    import numpy as np
    import trimesh
    a = np.linspace(0, 2 * math.pi, 96, endpoint=False)
    pts = [(x_face - sign * dx, d / 2 * math.cos(t), d / 2 * math.sin(t)) for d, dx in ((2.4, 0.9), (4.2, 0.0), (4.2, -1.0)) for t in a]
    return trimesh.convex.convex_hull(np.array(pts))


def _observe_coax(tag, shift2=None, depth=True):
    """U 形件：两片 x∈[−3,0]、x∈[−20,−17] 由 y∈[8,10] 的梁连成一件；两片各一个同轴沉头孔（x 轴），头侧 +x / −x。"""
    import trimesh
    import layers.l4_assembly as L4                                # noqa: PLC0415
    from tool_access import run_tool_access                        # noqa: PLC0415
    d = _TMP / f"l4_cone_{tag}"
    d.mkdir(parents=True, exist_ok=True)
    for p in d.glob("*.stl"):
        p.unlink()
    parts = []
    for lo, hi in (((-3.0, -10.0, -10.0), (0.0, 10.0, 10.0)), ((-20.0, -10.0, -10.0), (-17.0, 10.0, 10.0)),
                   ((-17.5, 8.0, -10.0), (-2.5, 10.0, 10.0))):
        b = trimesh.creation.box(extents=[h - l for l, h in zip(lo, hi)])
        b.apply_translation([(l + h) / 2 for l, h in zip(lo, hi)])
        parts.append(b)
    m = parts[0].union(parts[1]).union(parts[2])
    hole = trimesh.creation.cylinder(radius=1.2, height=30.0, sections=96)
    hole.apply_transform(trimesh.transformations.rotation_matrix(math.pi / 2, [0, 1, 0]))
    m = m.difference(hole).difference(_cs_cut(0.0, 1)).difference(_cs_cut(-20.0, -1))
    m.export(str(d / "plate.stl"), file_type="stl")
    old = L4.PLACED
    try:
        L4.PLACED = d
        ctx = _ctx(d, True, True)
        fx = ctx.data["fasteners"]["fasteners"][0]
        feat = ctx.data["features"]["features"][0]
        if depth:
            feat["geom"]["depth_mm"] = {"v": 3.0, "src": "反例：片厚"}
        a2 = [-18.5 if shift2 is None else -1.5 - shift2, 0.0, 0.0]
        fx["qty"] = 2
        fx["feature_hole_map"][0].update(holes=[[-1.5, 0.0, 0.0], a2], n=2)
        s1 = fx["tool_access"][0]["seats"][0]
        fx["tool_access"][0]["seats"].append({**s1, "hole_index": 1, "point_export_local": [-20.0, 0.0, 0.0] if shift2 is None
                                              else [a2[0] - 1.5, 0.0, 0.0], "outward_export_local": [-1.0, 0.0, 0.0]})
        geo = L4._Geo(ctx)
        res = LayerResult(4, "tool")
        run_tool_access(res, ctx, geo, L4._Names(ctx, geo), ctx.data["assembly"], 0.05)
        return res
    finally:
        L4.PLACED = old


def run() -> dict:
    coax_ok = _observe_coax("coax_ok")
    coax_dup = _observe_coax("coax_dup", shift2=1.0)
    coax_nodep = _observe_coax("coax_nodep", depth=False)
    ok_coax, why_coax = assert_green(findings(coax_ok, subject="FX", check="tool_origin_on_seat"))
    f_cd = findings(coax_dup, subject="FX", check="tool_reference")
    f_cn = findings(coax_nodep, subject="FX", check="tool_reference")
    dup_unknown = bool(f_cd) and f_cd[0].state == "FAIL" and f_cd[0].measured is None
    nodep_unknown = bool(f_cn) and f_cn[0].state == "FAIL" and f_cn[0].measured is None
    good = _observe("good")
    blocked = _observe("blocked", countersunk=False)
    capped = _observe("capped", cap=True)
    flat = _observe("flat", cone=False)
    no_src = _observe("nosrc", cone_src=False)
    ok_good, why_good = assert_green(findings(good, subject="FX", check="tool_origin_on_seat"))
    ok_good_path, why_good_path = assert_green(findings(good, subject="FX", check="tool_path_to_outside"))
    ok_blk, why_blk, red_blk = assert_red(findings(blocked, subject="FX", check="tool_origin_on_seat"), severity="BLOCK")
    ok_cap, why_cap, red_cap = assert_red(findings(capped, subject="FX", check="tool_path_to_outside"), severity="BLOCK")
    f_flat = findings(flat, subject="FX", check="tool_origin_on_seat")
    flat_red = bool(f_flat) and f_flat[0].state == "FAIL"
    f_ns = findings(no_src, subject="FX", check="tool_reference")
    ns_unknown = bool(f_ns) and f_ns[0].state == "FAIL" and f_ns[0].measured is None
    passed = ok_good and ok_good_path and ok_blk and ok_cap and flat_red and ns_unknown and ok_coax and dup_unknown and nodep_unknown
    return result(NAME, EXPECT, passed,
                  got=(f"好锥口 tool_origin_on_seat {'绿' if ok_good else why_good}、刀路 {'绿' if ok_good_path else why_good_path}；"
                       f"锥口被料挡 {'红' if ok_blk else why_blk}(measured={red_blk[0]['measured'] if red_blk else None})；"
                       f"锥口外盖板 {'红' if ok_cap else why_cap}(峰值={red_cap[0]['measured'] if red_cap else None})；"
                       f"不写 cone 按平面判 {'红（旧判据对锥面必然红）' if flat_red else '没红'}；缺 src → tool_reference unknown={ns_unknown}；"
                       f"同轴异位两颗 {'坐面绿' if ok_coax else why_coax}；同轴挪 1.0 ≤ 孔深 → unknown={dup_unknown}；"
                       f"不写孔深 → unknown={nodep_unknown}"),
                  red=red_blk + red_cap, expect_severity="BLOCK",
                  detail=(f"good={[record(f) for f in findings(good, subject='FX')]}；flat={[record(f) for f in f_flat]}；"
                          f"no_src={[str(f.detail)[:160] for f in f_ns]}；coax_ok={[record(f) for f in findings(coax_ok, subject='FX')]}；"
                          f"coax_dup={[str(f.detail)[:120] for f in f_cd]}；coax_nodep={[str(f.detail)[:120] for f in f_cn]}"))


if __name__ == "__main__":
    print(json.dumps(run(), ensure_ascii=False, indent=1))
