#!/usr/bin/env python3
"""反例 FG14 —— 某孔最浅咬入 1.75 < 下限 1.8，中位数 1.9、最深 1.9 都合格。

坑的形状：`engagement_per_hole` 的 criterion 白纸黑字写着"含 16 向的**最深/最浅两端**"，
代码里 `mins`（料最厚那一向 = 咬入最浅）算出来了，却只塞进 detail 字符串，
**从来没进过判据** —— 判的是 `engs`（中位数）和 `maxs`（最深）。
判据没做到它自己声称的事，于是"咬不住"这一端整条漏掉。

另一半坑：这一组孔是按"件名 + 数量相等"猜出来的（joins 里认出一个件、qty 等于该件孔数），
同件同数的两组会拿到同一批孔 —— 而 fasteners.yaml 早就有 `feature_ids` + `feature_hole_map`
（带 export_local 孔坐标），必须逐孔关联。

正确判据：最浅 / 中位 / 最深三个量各自超差都要红，并说清是哪一端超的。
"""
from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from _harness import FakeCtx, base_data, box, cyl, export_stl, findings, assert_red, assert_green, result  # noqa: E402

PART = "H03"
FID = "H03-FG14"
GID = "FG14_two_holes"
NAME = "某孔最浅咬入 1.75 < 1.8（中位 1.9、最深 1.9 都合格）：最浅端必须红，等厚孔必须绿"
EXPECT = f"L5/{GID}:engagement_per_hole（最浅端 1.75 < 下限 1.8）"
_TARGET = ("engagement_per_hole",)

T = 4.10            # 板厚
BUMP = 0.15         # B 孔坐面一个象限多出来的料 → 该向叠厚 4.25 → 咬入 1.75
HOLES = [(-6.0, 0.0), (6.0, 0.0)]


def _part(bad: bool):
    m = box(30.0, 12.0, T, center=(0.0, 0.0, T / 2))
    if bad:      # B 孔坐面一个象限多出 0.15 料：16 向里只有 4 向变厚，中位数纹丝不动
        m = m.union(box(4.8, 2.9, BUMP, center=(6.0 + 0.6, 1.55, T + BUMP / 2)))
    for x, y in HOLES:
        m = m.difference(cyl(1.1, 30.0, center=(x, y, 0.0)))
    return m


def _data():
    part = {"id": PART, "inventory_id": "H03_fg14", "material": "PLA",
            "print_orientation": "任意", "mirrored_copy": None}
    feat = {"id": FID, "part": PART, "kind": "screw_hole", "check_class": "screw_hole",
            "count": 2, "spec_verbatim": "2×Ø2.2 通",
            "geom": {"shape": "cylinder", "frame": "export_local",
                     "nominal_d_mm": {"v": 2.2, "src": "measured"},
                     "through": True, "axis": "z", "depth_kind": "cutter_length",
                     "depth_mm": {"v": 30.0, "src": "assumed"},
                     "pos": [0.0, 0.0, T / 2],
                     "axial_span_mm": [[0.0, 0.0, -0.5], [0.0, 0.0, T + 0.5]],
                     "hole_positions_mm": [[x, y, T / 2] for x, y in HOLES]}}
    fa = {"id": GID, "spec": "M2×6", "qty": 2,
          "joins": ["H03 背板", "servo_head_yaw"],
          "joint_type": "s288_metal_thread",
          "engagement_mm": {"v": 1.9, "src": "measured"},
          "stack_mm": {"v": 4.1, "src": "measured"},
          "status": "ok", "status_verbatim": "ok",
          "feature_ids": [FID],
          "feature_hole_map": [{"feature_id": FID, "instance": 0,
                                "instance_ref": "反例：两个孔",
                                "holes": [[x, y, T / 2] for x, y in HOLES], "n": 2}],
          "provenance": ["negatives/n_l5_fg14"]}
    return base_data([part], [feat], fasteners=[fa], relations=[])


def _observe(bad: bool):
    import l2_features
    import l5_function
    l2_features._GEOM_CACHE.clear()
    stl = export_stl(_part(bad), "fg14_" + ("bump" if bad else "even"))
    res = l5_function.run(FakeCtx(_data(), {PART: stl}))
    fs = findings(res, subject=GID)
    tgt = [f for f in fs if any(t in f.check for t in _TARGET)]
    seat = findings(res, subject=PART, check="seat_flatness")
    return tgt, {
        "engagement_per_hole": [f"{f.state}/{f.measured}｜{str(f.detail)[:150]}" for f in tgt],
        "件级 seat_flatness": [f"{f.state}/极差{f.measured}" for f in seat]}


def run() -> dict:
    bad, bad_all = _observe(True)
    good, good_all = _observe(False)
    ok_red, why_red, red = assert_red(bad, severity="BLOCK", require_measured=True)
    ok_green, why_green = assert_green(good)
    return result(NAME, EXPECT, ok_red and ok_green,
                  got=f"厚一角样本 {'红' if ok_red else why_red}(最浅咬入={red[0]['measured'] if red else None}) / 等厚对照 {'绿' if ok_green else why_green}",
                  red=red, expect_severity="BLOCK",
                  detail=f"厚一角样本={bad_all}｜等厚对照={good_all}")


if __name__ == "__main__":
    import json
    print(json.dumps(run(), ensure_ascii=False, indent=1))
