#!/usr/bin/env python3
"""反例 README N2 —— 轴承只查截面 / README N1 —— 空刀。

N2（第 3 层，`l3_static.run`）：
    placed/ 里一只 40×40×6 的 PLA 座（件 L01，通孔 Ø27.4）+ 一颗 6704 轴承实体（Ø27×Ø20×4）。
    座的**外侧端面**留一圈唇：z ∈ [1.0, 2.0]、内径 Ø26.6 —— 只在轴承外圈最外 1 mm 处径向压进 0.2 mm。
    · 完整实体   L3 按 components.yaml:bearing_6704zz 的 envelope 20×27×4 反算环体积/包围盒**认领**这颗实体，
                 再做完整实体 × 座 的布尔 → bearing_6704zz:part_vs_component_solid FAIL(BLOCK)，
                 measured ≈ π/4·(27²−26.6²)·1.0 ≈ 16.8 mm³；同时 L01:component_bearing_6704zz FAIL(BLOCK)
    · 薄片       把轴承换成 Ø27×0.1 的薄片（只有中截面）：中截面碰不到唇（唇在 z ≥ 1.0），
                 而且包围盒/体积对不上 envelope → 层**认领不到** → bearing_6704zz:world_placement unknown
                 （FAIL、measured=None）、_placed:unclaimed_placed_solids FAIL(WARN)。
                 反例要证明的正是：用薄片判就会漏（交集 0），所以层必须拒绝把薄片当轴承 —— 不是绿。
    · 对照       座不留唇 + 完整实体 → part_vs_component_solid PASS（evidence 1 对）
N1（第 2 层，`l2_features.run`）：
    声明一个 Ø2.2 通孔，成品上**没切出来**（刀没落在实体上）→ <fid>:present FAIL(BLOCK)，measured="0/1 处到位"；
    同一根柱子把孔切出来 → present PASS。存在性判据的证据数必须 > 0（射线数）。
判据阈值取真实 tolerances.yaml。
"""
from __future__ import annotations
import json
import sys
from pathlib import Path

_HERE = Path(__file__).resolve()
GATE = _HERE.parents[1]
for _p in (str(_HERE.parent), str(GATE), str(GATE / "layers"), str(GATE.parents[1])):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import core                                                                 # noqa: E402
from _harness import FakeCtx, base_data, box, cyl, export_stl, findings, assert_red, assert_green, result, record  # noqa: E402

PART = "L01"
BEARING = "bearing_6704zz"
NAME = "轴承只查截面（N2）：完整实体压进座唇必须红、薄片必须认领不到（unknown）不许绿；空刀（N1）present 必须红"
EXPECT = (f"L3/{BEARING}:part_vs_component_solid FAIL(BLOCK) ≈16.8 mm³ 且 L3/{PART}:component_{BEARING} FAIL；"
          f"薄片 → {BEARING}:world_placement unknown；对照 PASS；L2/<件>:<孔>:present FAIL(BLOCK) 空刀、PASS 有孔")
_TMP = core.ROOT / "tools/gate/out/_neg_tmp"
BORE_D, LIP_D = 27.4, 26.6


def _seat(lip: bool):
    m = box(40.0, 40.0, 6.0).difference(cyl(BORE_D / 2, 20.0, sections=96))
    if lip:
        ring = cyl(BORE_D / 2 + 0.5, 1.0, center=(0, 0, 1.5), sections=96).difference(
            cyl(LIP_D / 2, 2.0, center=(0, 0, 1.5), sections=96))
        m = m.union(ring)
    return m


def _bearing(full: bool):
    if full:
        return cyl(13.5, 4.0, sections=96).difference(cyl(10.0, 6.0, sections=96))
    return cyl(13.5, 0.1, sections=96)                       # 只有中截面的薄片


def _placed_dir(tag, lip, full):
    d = _TMP / f"l3_bearing_{tag}"
    d.mkdir(parents=True, exist_ok=True)
    for p in d.glob("*.stl"):
        p.unlink()
    seat = d / "seat.stl"
    _seat(lip).export(str(seat), file_type="stl")
    _bearing(full).export(str(d / "bearing_neg.stl"), file_type="stl")
    return d, seat


def _data():
    d = base_data([{"id": PART, "inventory_id": "L01_seat", "material": "PLA", "print_orientation": "任意",
                    "mirrored_copy": None, "qty": 1, "placed_instances": ["seat"]}], [])
    d["keepouts"] = {"keepouts": []}                         # 本反例只查元件完整实体，不跑禁入体
    return d


def _observe_l3(tag, lip, full):
    import l3_static as L3
    placed, seat = _placed_dir(tag, lip, full)
    old = L3.PLACED
    try:
        L3.PLACED = placed
        # 导出件与 placed 件同一个文件：层按（面数, 体积）指纹匹配，不靠文件名
        return L3.run(FakeCtx(_data(), {PART: seat}, {p.stem: p for p in placed.glob("*.stl")}))
    finally:
        L3.PLACED = old


def _observe_l2(cut: bool):
    import l2_features
    l2_features._GEOM_CACHE.clear()
    m = cyl(4.0, 10.0)
    if cut:
        m = m.difference(cyl(1.1, 30.0))
    stl = export_stl(m, "n1_" + ("cut" if cut else "empty"))
    fid = "H03-N1"
    feat = {"id": fid, "part": "H03", "kind": "screw_hole", "check_class": "screw_hole", "count": 1,
            "spec_verbatim": "1×Ø2.2 通",
            "geom": {"shape": "cylinder", "frame": "export_local", "nominal_d_mm": {"v": 2.2, "src": "measured"},
                     "depth_mm": {"v": 10.0, "src": "assumed"}, "depth_kind": "cutter_length", "through": True,
                     "axis": "z", "pos": [0.0, 0.0, 0.0], "axial_span_mm": [[0.0, 0.0, -5.0], [0.0, 0.0, 5.0]]}}
    part = {"id": "H03", "inventory_id": "H03_n1", "material": "PLA", "print_orientation": "任意", "mirrored_copy": None}
    res = l2_features.run(FakeCtx(base_data([part], [feat]), {"H03": stl}))
    return findings(res, subject="H03", check=f"{fid}:present")


def run() -> dict:
    tol = _data()["tolerances"]["feature_check_tolerances"]["static_intersection_mm3"]["max"]
    r_full = _observe_l3("full", lip=True, full=True)
    r_thin = _observe_l3("thin", lip=True, full=False)
    r_good = _observe_l3("good", lip=False, full=True)

    ok_red, why_red, red = assert_red(findings(r_full, subject=BEARING, check="part_vs_component_solid"), severity="BLOCK")
    ok_cell, why_cell, red_cell = assert_red(findings(r_full, subject=PART, check=f"component_{BEARING}"), severity="BLOCK")
    vol = float(red[0]["measured"]) if red else None
    vol_ok = vol is not None and vol > tol and 10.0 < vol < 25.0           # ≈16.8，量到的是唇那一圈
    # 薄片：不许绿。认领不到 → world_placement unknown；part_vs_component_solid 不得出现 PASS
    wp = findings(r_thin, subject=BEARING, check="world_placement")
    pvc_thin = findings(r_thin, subject=BEARING, check="part_vs_component_solid")
    orphan = findings(r_thin, subject="_placed", check="unclaimed_placed_solids")
    thin_ok = (bool(wp) and wp[0].state == "FAIL" and wp[0].measured is None
               and not any(f.state == "PASS" for f in pvc_thin)
               and bool(orphan) and orphan[0].state == "FAIL")
    ok_green, why_green = assert_green(findings(r_good, subject=BEARING, check="part_vs_component_solid"))

    n1_bad = _observe_l2(cut=False)
    n1_good = _observe_l2(cut=True)
    ok_n1, why_n1, red_n1 = assert_red(n1_bad, severity="BLOCK")
    ev_ok = ok_n1 and all(r["evidence_n"] > 0 for r in red_n1)
    ok_n1g, why_n1g = assert_green(n1_good)

    passed = ok_red and ok_cell and vol_ok and thin_ok and ok_green and ok_n1 and ev_ok and ok_n1g
    return result(NAME, EXPECT, passed,
                  got=(f"N2 完整实体 {'红' if ok_red else why_red}(交集={vol} mm³, 阈值 {tol})，件格 {'红' if ok_cell else why_cell}；"
                       f"薄片 {'认领不到→unknown（不绿）' if thin_ok else '未按预期 unknown'}；对照 {'绿' if ok_green else why_green}；"
                       f"N1 空刀 {'红' if ok_n1 else why_n1}({red_n1[0]['measured'] if red_n1 else None}, 射线 {red_n1[0]['evidence_n'] if red_n1 else 0})，"
                       f"有孔 {'绿' if ok_n1g else why_n1g}"),
                  red=red + red_cell + red_n1, expect_severity="BLOCK",
                  detail=(f"薄片：world_placement={[record(f) for f in wp]}；part_vs_component_solid={[record(f) for f in pvc_thin]}；"
                          f"orphan={[record(f) for f in orphan]}；对照={[record(f) for f in findings(r_good, subject=BEARING)]}"))


if __name__ == "__main__":
    print(json.dumps(run(), ensure_ascii=False, indent=1))
