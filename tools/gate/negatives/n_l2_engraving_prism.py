#!/usr/bin/env python3
"""反例（2026-09-19 hr15 件号刻字）—— shape=prism 的挖除特征：① 槽没刻出来必须红；② 声明了刻字的压板保留区不许因刻字变红。

坑的形状：hr15 给 24 件刻 0.4 深件号，H02 / N01 的字落在声明为 plate/back_plate 的区域上，
`solid_retention` / `min_section` 把凹字当"未声明缺料"翻红（复审 B1）。声明挖除只认 box/cylinder，
字形是多边形，用一堆小 box 去凑既不准又要几百条。所以 L2 加 prism（面内多边形 × 深度）：
  · occupancy：每个笔画多边形的代表点在槽深一半处必须没有料 → 板上**没刻**的样本必须红；
  · declared_voids：prism 体（多边形外扩 _KEEP_MARGIN）从保留区扣掉 → 刻了字且声明了的板 solid_retention/min_section 必须绿；
  · 对照：刻了字**没声明** → 保留区红（FG08 老判据仍在）。

几何：10×20×3 板，顶面 z=1.5；刻字 = 两条笔画（矩形 6×1 和 1×4，L 形）深 0.4。
"""
from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from _harness import FakeCtx, base_data, box, export_stl, findings, assert_red, assert_green, result  # noqa: E402

PART = "H02"
FID_PLATE, FID_ENG = "H02-NEG-PLATE", "H02-NEG-ENG"
NAME = "刻字 prism：没刻出来 occupancy 红；声明后压板保留区绿；不声明则保留区红"
EXPECT = (f"L2/{PART}:{FID_ENG}:occupancy 在未刻样本 FAIL；刻了+声明：{FID_PLATE}:solid_retention/min_section PASS；"
          f"刻了+未声明：{FID_PLATE}:solid_retention 或 min_section FAIL")
STROKES = [[(-3.0, 1.5), (3.0, 1.5), (3.0, 2.5), (-3.0, 2.5)],           # 横笔画 6×1
           [(-3.0, -2.5), (-2.0, -2.5), (-2.0, 1.5), (-3.0, 1.5)]]        # 竖笔画 1×4
DEPTH = 0.4


def _mesh(engraved: bool):
    m = box(10.0, 20.0, 3.0)                                # z −1.5..1.5
    if engraved:
        for st in STROKES:
            xs = [p[0] for p in st]; ys = [p[1] for p in st]
            sx, sy = max(xs) - min(xs), max(ys) - min(ys)
            m = m.difference(box(sx, sy, DEPTH + 1.0, center=((max(xs) + min(xs)) / 2, (max(ys) + min(ys)) / 2, 1.5 - DEPTH + (DEPTH + 1.0) / 2)))
    return m


def _data(declare_engraving: bool):
    part = {"id": PART, "inventory_id": f"{PART}_neg_eng", "material": "PLA", "print_orientation": "任意", "mirrored_copy": None}
    plate = {"id": FID_PLATE, "part": PART, "kind": "plate", "check_class": "plate", "count": 1, "spec_verbatim": "压板 10×20×3（反例）",
             "geom": {"shape": "box", "frame": "export_local", "bbox_mm": [[-5.0, -10.0, -1.5], [5.0, 10.0, 1.5]], "pos": [0.0, 0.0, 0.0],
                      "thickness_mm": {"v": 3.0, "src": "assumed"}}}
    eng = {"id": FID_ENG, "part": PART, "kind": "engraving", "check_class": "engraving", "count": 1, "spec_verbatim": "件号刻字 L 形两笔画 深 0.4（反例）",
           "geom": {"shape": "prism", "frame": "export_local", "depth_mm": {"v": DEPTH, "src": "assumed"}, "through": False, "pos": [0.0, 0.0, 1.5],
                    "prism": {"origin": [0.0, 0.0, 1.5], "right": [1.0, 0.0, 0.0], "up": [0.0, 1.0, 0.0], "normal": [0.0, 0.0, 1.0], "depth_mm": DEPTH,
                              "polygons": [{"exterior": [list(p) for p in st], "holes": []} for st in STROKES]}}}
    return base_data([part], [plate, eng] if declare_engraving else [plate])


def _observe(engraved: bool, declared: bool):
    import l2_features
    l2_features._GEOM_CACHE.clear()
    stl = export_stl(_mesh(engraved), f"eng_{'cut' if engraved else 'flat'}")
    res = l2_features.run(FakeCtx(_data(declared), {PART: stl}))
    return findings(res, subject=PART)


def run() -> dict:
    # ① 板上没刻，却声明了刻字 → occupancy 必须红（代表点有料）
    flat = [f for f in _observe(False, True) if f.check == f"{FID_ENG}:occupancy"]
    ok1, why1, red1 = assert_red(flat, severity="BLOCK", require_measured=True)
    # ② 刻了 + 声明 → 压板 solid_retention / min_section 必须绿（刻字被扣掉）
    cut_decl = [f for f in _observe(True, True) if f.check in (f"{FID_PLATE}:solid_retention", f"{FID_PLATE}:min_section")]
    ok2, why2 = assert_green(cut_decl)
    ok2 = ok2 and len(cut_decl) >= 1
    # ③ 刻了 + 不声明 → 老判据照旧红（对照，证明 ② 的绿是扣除带来的，不是判据变松）
    cut_undecl = [f for f in _observe(True, False) if f.check in (f"{FID_PLATE}:solid_retention", f"{FID_PLATE}:min_section")]
    ok3, why3, red3 = assert_red(cut_undecl, severity="BLOCK", require_measured=True)
    return result(NAME, EXPECT, ok1 and ok2 and ok3,
                  got=f"未刻/声明 {'红' if ok1 else why1}({[(r['check'], r['measured']) for r in red1]}) / 刻+声明 {'绿' if ok2 else why2}"
                      f"({[(f.check, f.state, f.measured) for f in cut_decl]}) / 刻+未声明 {'红' if ok3 else why3}({[(r['check'], r['measured']) for r in red3]})",
                  red=red1 + red3, expect_severity="BLOCK",
                  detail=f"prism 两笔画 6×1 + 1×4 深 {DEPTH}；板 10×20×3")


if __name__ == "__main__":
    import json
    print(json.dumps(run(), ensure_ascii=False, indent=1))
