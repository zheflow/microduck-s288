#!/usr/bin/env python3
"""L4 声明式弹性接触（elastic_contact_declared）反例的共用场景 —— hr41g 2026-09-24。

不是反例本身（文件名不以 n_ 开头，runner 不收）。四个 n_l4_elastic_*.py 共用这一份内存几何与最小清单，
只改"障碍盒放在哪 / 数据里声明了什么"，判据一律经过层的 `run_declared_motions` / `run(ctx)`。

几何（世界 = export_local，frames.per_part 全部单位阵、零平移）：
    lid   移动件（件 P01）     x∈[0,2]  y∈[−1,1]  z∈[0,1]
    rib   障碍件（件 O01）     按 case 放（见 RIBS）
    base  远处底座（件 B00）   x∈[−6,3] y∈[−3,3]  z∈[−3,−2]（只为让拆卸序的障碍集合非空）
    路径  lid 先 −x 4 mm 再 +z 3 mm（与 B03 "后移 3.5 再上提 30" 同形），步长 0.25

    case ok       rib 顶部 z∈[0.9,1.3]：lid 顶面扫过时交集 0.5×2×0.1 = 0.1 mm³，全部在
                  "lid 弹性特征 P01-F01 bbox（随位移）∩ rib 特征 O01-F01 bbox" 里
    case outside  rib 底部 z∈[−0.3,0.1]：交集同为 0.1 mm³，但在 lid 底面 —— 不在 P01-F01（z 0.5..1.0）里
    case overcap  rib z∈[0.6,1.3]：交集 0.4 mm³，全在声明区内，但 > 过盈 0.12 × 面积上限 1.0 = 0.12
    case clean    rib z∈[1.1,1.3]：不接触（对照：刚体 PASS，不发弹性结果）

数据：parts（P01/O01/B00，material 默认 PETG）、features（P01-F01 kind snap / O01-F01 kind lip，bbox_mm）、
fasteners（G01 joint_type=pla_snap，feature_ids=[P01-F01, O01-F01] —— 现役 F31/F32/F28 的同一登记形式）。
elastic_contact 声明块是 hr41g 提议、主 agent 尚未登记到 features.yaml 的字段（见 hr41_gate_work/记录.md）。
阈值 static_intersection_mm3 取真实 tolerances.yaml。
"""
from __future__ import annotations
import copy
import sys
from contextlib import contextmanager
from pathlib import Path

_HERE = Path(__file__).resolve()
GATE = _HERE.parents[1]
for _p in (str(_HERE.parent), str(GATE), str(GATE.parents[1])):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import core                                                                   # noqa: E402
from _harness import FakeCtx, _TMP                                            # noqa: E402

LID = ((0.0, -1.0, 0.0), (2.0, 1.0, 1.0))
BASE = ((-6.0, -3.0, -3.0), (3.0, 3.0, -2.0))
RIBS = {
    "ok": ((-1.5, -1.0, 0.9), (-1.0, 1.0, 1.3)),
    "outside": ((-1.5, -1.0, -0.3), (-1.0, 1.0, 0.1)),
    "overcap": ((-1.5, -1.0, 0.6), (-1.0, 1.0, 1.3)),
    "clean": ((-1.5, -1.0, 1.1), (-1.0, 1.0, 1.3)),
}
LID_FEATURE_BBOX = ((0.0, -1.0, 0.5), (2.0, 1.0, 1.0))
DELTA = 0.12                       # 声明过盈量（反例自造，不是 B03 的 0.25）
SEGMENTS = [dict(outward_world=[-1, 0, 0], len_mm=4.0), dict(outward_world=[0, 0, 1], len_mm=3.0)]
_IDENT = {"world_to_export_local_R": [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
          "world_to_export_local_t": [0.0, 0.0, 0.0]}


_REAL = None


def _real_data():
    global _REAL
    if _REAL is None:
        _REAL = core.load_data()
    return _REAL


def decl(material="PETG", delta=DELTA, src="assumed", area=None, drop=()):
    """features.yaml:<特征>.elastic_contact 声明块（提议字段）。drop 里列的键不写（造"缺一项"）。"""
    d = {}
    if "material" not in drop:
        d["material"] = material
    if "interference_mm" not in drop:
        node = {"v": delta}
        if src is not None:
            node["src"] = src
        d["interference_mm"] = node
    if area is not None:
        d["contact_area_mm2"] = {"v": area, "src": "assumed"}
    return d


def _write(case):
    import numpy as np
    import trimesh
    d = _TMP / f"l4_elastic_{case}"
    d.mkdir(parents=True, exist_ok=True)
    for p in d.glob("*.stl"):
        p.unlink()
    for name, (lo, hi) in (("lid", LID), ("rib", RIBS[case]), ("base", BASE)):
        lo, hi = np.asarray(lo, float), np.asarray(hi, float)
        m = trimesh.creation.box(extents=(hi - lo))
        m.apply_translation((lo + hi) / 2.0)
        m.export(str(d / f"{name}.stl"), file_type="stl")
    return d


def _feature(fid, part, kind, bbox, ec=None):
    f = {"id": fid, "part": part, "kind": kind, "check_class": kind, "count": 1,
         "purpose": "反例", "datum_frame": "export_local", "contains_world_coords": False,
         "nominal_source": {"v": "反例自造", "src": "assumed"},
         "geom": {"frame": "export_local", "derived_from": "反例", "shape": "box", "axis": "z",
                  "bbox_mm": [list(bbox[0]), list(bbox[1])]},
         "provenance": ["negatives/_l4_elastic_scene.py"]}
    if ec is not None:
        f["elastic_contact"] = ec
    return f


def motion(mid="lid"):
    return dict(id=mid, movers=["lid"], present=["lid", "rib", "base"], workspace="test",
                sense="withdrawal_from_assembled", segments=copy.deepcopy(SEGMENTS), step_mm=0.25)


def build(case, rib_decl=None, lid_decl=None, rib_bbox=None, rib_part_material="PETG", with_group=True):
    """返回 (ctx, placed_dir)。rib_bbox=None → 登记 bbox 与真实 rib 几何一致。"""
    placed = _write(case)
    data = copy.deepcopy(_real_data())                                       # 真公差表（进程内只读一次）
    data["parts"] = {"parts": [
        {"id": "P01", "inventory_id": "lid", "build_fn": [], "qty": 1, "placed_instances": ["lid"], "material": "PETG"},
        {"id": "O01", "inventory_id": "rib", "build_fn": [], "qty": 1, "placed_instances": ["rib"],
         "material": rib_part_material},
        {"id": "B00", "inventory_id": "base", "build_fn": [], "qty": 1, "placed_instances": ["base"], "material": "PETG"},
    ]}
    data["features"] = {"features": [
        _feature("P01-F01", "P01", "snap", LID_FEATURE_BBOX, lid_decl),
        _feature("O01-F01", "O01", "lip", rib_bbox or RIBS[case], rib_decl),
    ], "frames": {"per_part": {"P01": dict(_IDENT), "O01": dict(_IDENT), "B00": dict(_IDENT)}}}
    data["fasteners"] = {"fasteners": ([{
        "id": "G01_snap", "spec": "反例卡扣", "qty": 1, "joins": ["P01", "O01"], "joint_type": "pla_snap",
        "engagement_mm": {"v": None, "src": None}, "feature_ids": ["P01-F01", "O01-F01"], "holes": [],
        "status": "unverified"}] if with_group else []), "known_issues": [], "counts": {}}
    data["keepouts"] = {"keepouts": []}
    data["waivers"] = {"waivers": []}
    data["assembly"] = {
        "subassemblies": {"robot": {"members": []}},
        "assembly_order": [{
            "step": 1, "id": "step01", "seq": 1, "subassembly_of": "robot",
            "action": "反例：lid 后移 4 再上提 3（反向为装入）", "parts": ["P01", "O01", "B00"],
            "required_motion_groups": {"lid": {"workspace": "test", "movers": ["lid"], "present": ["lid", "rib", "base"]}},
            "motions": [motion()]}],
        "disassembly_order": [
            {"seq": 1, "action": "拆 lid：−x 4 再 +z 3", "reverse_of_step": 1, "movers": ["lid"],
             "segments": copy.deepcopy(SEGMENTS), "step_mm": 0.25, "proven": False},
            {"seq": 2, "action": "拆 rib：+z 5", "reverse_of_step": 1, "movers": ["rib"],
             "segments": [dict(outward_world=[0, 0, 1], len_mm=5.0)], "step_mm": 0.25, "proven": False}],
    }
    ctx = FakeCtx(data, {}, {p.stem: p for p in sorted(placed.glob("*.stl"))})
    return ctx, placed


@contextmanager
def placed_patched(L4, placed):
    old = L4.PLACED
    try:
        L4.PLACED = placed
        yield
    finally:
        L4.PLACED = old


def run_motion(case, **kw):
    """经 run_declared_motions 跑单动作步；返回 (res, tol)。"""
    import layers.l4_assembly as L4
    ctx, placed = build(case, **kw)
    with placed_patched(L4, placed):
        geo = L4._Geo(ctx)
        tol = ctx.data["tolerances"]["feature_check_tolerances"]["static_intersection_mm3"]["max"]
        st = ctx.data["assembly"]["assembly_order"][0]
        res = core.LayerResult(4, "反例")
        L4.run_declared_motions(res, geo, st, tol)
    return res, tol


def run_layer(case, **kw):
    """经 L4.run(ctx) 跑整层（含拆卸序）；返回 res。"""
    import layers.l4_assembly as L4
    ctx, placed = build(case, **kw)
    with placed_patched(L4, placed):
        return L4.run(ctx)
