#!/usr/bin/env python3
"""反例 —— 六角螺母压入穴（hr41 落盘 2026-09-25，主设计 Lane C；首例 J02-F04 / J03-F03）：对边做小 0.3 必须红，名义穴必须绿。

背景：kind=cavity、shape=prism_array 的六角穴，nominal_d_mm = 对边 4.2。L2 以前按 Ø4.2 圆孔探（ring_scan 24 向找 r 2.1 整圈），
六角的角上 r 2.42 → 永远 0/N 处到位（可打印件清单草稿 4b：J02-F04「量到 r 2.108–2.317 正是六边形」）。
现在声明 geom.hex_prism {sides 6, across_flats_mm, flat_normals_export_local} 的六角穴按**对边 + 对角**两组射线量：
每站 6 个平面法向（期望半径 = 对边/2）+ 6 个角向（期望 = 对边/(2·cos30°)），对边 = 相对平面半径和、对角 = 相对角半径和，
容差同圆孔（feature_check_tolerances.hole_diameter_mm）；盲深 / 通盲同其它盲穴（step_probe，内圈取内切圆 −0.3、外圈取外接圆 +0.3）。
内存几何（不碰整鸭件）：板 x/z ∈ [−6, 6]、y ∈ [0, 2.5]；六角穴从 y=0 面开口、深 1.65（穴底 y 1.65，皮 0.85）；皮上 Ø2.4 过孔。
  good      对边 4.2 → P01-F01:present PASS、:depth PASS（对照）
  small     穴对边做小 0.3（3.9）→ P01-F01:present FAIL(BLOCK)，measured = "0/1 处到位"
  rotated   声明的平面法向转 30°（登记错）→ present FAIL(BLOCK)（量到的"对边"其实是角）
  no_hex    不写 hex_prism → 回到圆孔探针（本层旧行为，逐字节不变）：present FAIL —— 只记录，证明旧路径没被改
判据阈值取真实 tools/gate/data/tolerances.yaml。
"""
from __future__ import annotations

import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from _harness import FakeCtx, base_data, cyl, export_stl, findings, assert_red, assert_green, result  # noqa: E402

PART = "P01"
FID = "P01-F01"
NAME = "六角螺母穴：对边做小 0.3 / 平面法向登记转 30° 必须红，名义穴绿（hr41 J02-F04/J03-F03）"
EXPECT = f"L2/{PART}:{FID}:present FAIL(BLOCK)（对边 3.9 ≠ 4.2±tol）；名义穴 present/depth PASS"


def _hex_prism_mesh(af, y0, y1, flat_deg):
    import numpy as np
    import trimesh
    R = af / math.sqrt(3.0)
    th0 = math.radians(flat_deg) - math.pi / 6
    pts = [(R * math.cos(th0 + k * math.pi / 3), y, R * math.sin(th0 + k * math.pi / 3)) for y in (y0, y1) for k in range(6)]
    return trimesh.convex.convex_hull(np.array(pts))


def _mesh(af):
    import trimesh
    plate = trimesh.creation.box(extents=(12.0, 2.5, 12.0))
    plate.apply_translation((0.0, 1.25, 0.0))
    m = plate.difference(_hex_prism_mesh(af, -0.5, 1.65, 0.0))       # 穴从 y=0 面开口到 y 1.65
    return m.difference(cyl(1.2, 6.0, center=(0.0, 1.25, 0.0), axis="y"))


def _data(hex_decl=True, flat_deg=0.0):
    part = {"id": PART, "inventory_id": f"{PART}_hexnut", "material": "PETG", "print_orientation": "任意", "mirrored_copy": None}
    geom = {"shape": "prism_array", "frame": "export_local",
            "nominal_d_mm": {"v": 4.2, "src": "反例：对边"},
            "depth_mm": {"v": 1.65, "src": "反例"}, "depth_kind": "blind_depth", "through": False,
            "axis": "y", "pos": [0.0, 0.825, 0.0], "positions_mm": [[0.0, 0.825, 0.0]]}
    if hex_decl:
        a = math.radians(flat_deg)
        geom["hex_prism"] = {"sides": 6, "across_flats_mm": {"v": 4.2, "src": "反例"},
                             "flat_normals_export_local": [[math.cos(a), 0.0, math.sin(a)]]}
    feat = {"id": FID, "part": PART, "kind": "cavity", "check_class": "cavity", "count": 1,
            "spec_verbatim": "六角穴 s 4.2 深 1.65", "geom": geom}
    return base_data([part], [feat])


def _observe(tag, af=4.2, hex_decl=True, flat_deg=0.0):
    import l2_features
    l2_features._GEOM_CACHE.clear()
    stl = export_stl(_mesh(af), f"hexnut_{tag}")
    res = l2_features.run(FakeCtx(_data(hex_decl, flat_deg), {PART: stl}))
    fs = findings(res, subject=PART)
    return fs, {f.check: f"{f.state}/{f.measured}" for f in fs}


def run() -> dict:
    good, good_all = _observe("good")
    small, small_all = _observe("small", af=3.9)
    rot, rot_all = _observe("rotated", flat_deg=30.0)
    old, old_all = _observe("nohex", hex_decl=False)
    ok_g, why_g = assert_green([f for f in good if f.check in (f"{FID}:present", f"{FID}:depth")])
    ok_gd, why_gd = assert_green([f for f in good if f.check == f"{FID}:depth"])
    ok_s, why_s, red_s = assert_red([f for f in small if f.check == f"{FID}:present"], severity="BLOCK")
    ok_r, why_r, red_r = assert_red([f for f in rot if f.check == f"{FID}:present"], severity="BLOCK")
    old_p = [f for f in old if f.check == f"{FID}:present"]
    old_red = bool(old_p) and old_p[0].state == "FAIL"
    passed = ok_g and ok_gd and ok_s and ok_r and old_red
    return result(NAME, EXPECT, passed,
                  got=(f"名义穴 present/depth {'绿' if ok_g and ok_gd else (why_g or why_gd)}；"
                       f"对边做小 0.3 {'红' if ok_s else why_s}(measured={red_s[0]['measured'] if red_s else None})；"
                       f"法向转 30° {'红' if ok_r else why_r}；不写 hex_prism 走旧圆孔探针 present={'红' if old_red else '没红'}"),
                  red=red_s + red_r, expect_severity="BLOCK",
                  detail=f"good={good_all}｜small={small_all}｜rotated={rot_all}｜no_hex={old_all}")


if __name__ == "__main__":
    import json
    print(json.dumps(run(), ensure_ascii=False, indent=1))
