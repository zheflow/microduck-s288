#!/usr/bin/env python3
"""反例 FG07 —— 孔中段缩径（Ø2.2 通孔在 z=1.7 处缩成 Ø2.0）。

坑的形状：`ring_scan` 沿轴扫 21 站，却在这 21 站里挑**最好**的一站去判；
`step_probe` 只 8 个方向、内圈探针半径 r−0.3 = 0.8 从缩径环下面钻过去。
于是"孔存在""孔径合格""真通"三条全绿，而 Ø2.2 的螺丝根本插不过去。

正确判据：整段功能通规 —— 沿孔轴判**最坏截面**，报告最小内径与它的轴向位置。
对照：不加缩径环的同一根柱子必须仍然绿（判据不能写成恒红）。
"""
from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from _harness import FakeCtx, base_data, cyl, export_stl, findings, assert_red, assert_green, result  # noqa: E402

PART = "H03"
FID = "H03-FG07"
NAME = "孔中段缩径 Ø2.2→Ø2.0：整段通规必须红，直孔必须绿"
EXPECT = f"L2/{PART}:{FID}:bore_gauge（最小内径 2.0 < 2.2−0.1）"
_TARGET = ("bore_gauge",)


def _mesh(bad: bool):
    m = cyl(4.0, 10.0).difference(cyl(1.1, 30.0))
    if not bad:
        return m
    ring = cyl(1.1, 0.4, center=(0, 0, 1.7)).difference(cyl(1.0, 2.0, center=(0, 0, 1.7)))
    return m.union(ring)


def _data():
    part = {"id": PART, "inventory_id": f"{PART}_fg07", "material": "PLA",
            "print_orientation": "任意", "mirrored_copy": None}
    feat = {"id": FID, "part": PART, "kind": "screw_hole", "check_class": "screw_hole",
            "count": 1, "spec_verbatim": "1×Ø2.2 通",
            "geom": {"shape": "cylinder", "frame": "export_local",
                     "nominal_d_mm": {"v": 2.2, "src": "measured"},
                     "depth_mm": {"v": 10.0, "src": "assumed"}, "depth_kind": "cutter_length",
                     "through": True, "axis": "z", "pos": [0.0, 0.0, 0.0],
                     "axial_span_mm": [[0.0, 0.0, -5.0], [0.0, 0.0, 5.0]]}}
    return base_data([part], [feat])


def _observe(bad: bool):
    import l2_features
    l2_features._GEOM_CACHE.clear()
    stl = export_stl(_mesh(bad), "fg07_" + ("neck" if bad else "straight"))
    res = l2_features.run(FakeCtx(_data(), {PART: stl}))
    fs = findings(res, subject=PART)
    tgt = [f for f in fs if any(t in f.check for t in _TARGET)]
    return tgt, {f.check: f"{f.state}/{f.measured}" for f in fs}


def run() -> dict:
    bad, bad_all = _observe(True)
    good, good_all = _observe(False)
    ok_red, why_red, red = assert_red(bad, severity="BLOCK", require_measured=True)
    ok_green, why_green = assert_green(good)
    return result(NAME, EXPECT, ok_red and ok_green,
                  got=f"缩径样本 {'红' if ok_red else why_red}(最小内径={bad[0].measured if bad else None}) / 直孔对照 {'绿' if ok_green else why_green}",
                  red=red, expect_severity="BLOCK",
                  detail=f"缩径样本全部判据={bad_all}｜直孔对照={good_all}")


if __name__ == "__main__":
    import json
    print(json.dumps(run(), ensure_ascii=False, indent=1))
