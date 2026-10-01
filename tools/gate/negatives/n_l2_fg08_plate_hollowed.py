#!/usr/bin/env python3
"""反例 FG08 —— 声明 10×10×2 的压板，成品只剩中心一颗 0.4 mm 立方。

坑的形状：`occupancy` 的判据是"声明区域内**至少有一个**采样点有料"，
于是"一个点有料 = 整块板还在"（实测 2/28 点有料照样绿）。
压板 / 背板 / 承压面这类件，靠的正是"整块还在"。

正确判据：判**保留比例**与**最小有效截面**，并扣除明确声明的孔槽 ——
声明为实体的区域，凡是没有别的特征声明要挖掉的地方，都必须有料。
对照：完整的 10×10×2 板必须仍然绿。
"""
from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from _harness import FakeCtx, base_data, box, export_stl, findings, assert_red, assert_green, result  # noqa: E402

PART = "H02"
FID = "H02-FG08"
NAME = "压板只剩中心 0.4 立方：保留比例必须红，完整板必须绿"
EXPECT = f"L2/{PART}:{FID}:solid_retention（保留比例 ≈3% / 最小有效截面）"
_TARGET = ("solid_retention", "min_section")


def _mesh(bad: bool):
    return box(0.4, 0.4, 0.4) if bad else box(10.0, 10.0, 2.0)


def _data():
    part = {"id": PART, "inventory_id": f"{PART}_fg08", "material": "PLA",
            "print_orientation": "背板外面朝下", "mirrored_copy": None}
    feat = {"id": FID, "part": PART, "kind": "plate", "check_class": "plate",
            "count": 1, "spec_verbatim": "压板 10×10×2（承压面）",
            "geom": {"shape": "box", "frame": "export_local",
                     "bbox_mm": [[-5.0, -5.0, -1.0], [5.0, 5.0, 1.0]],
                     "pos": [0.0, 0.0, 0.0],
                     "thickness_mm": {"v": 2.0, "src": "assumed"}}}
    return base_data([part], [feat])


def _observe(bad: bool):
    import l2_features
    l2_features._GEOM_CACHE.clear()
    stl = export_stl(_mesh(bad), "fg08_" + ("hollow" if bad else "full"))
    res = l2_features.run(FakeCtx(_data(), {PART: stl}))
    fs = findings(res, subject=PART)
    tgt = [f for f in fs if any(t in f.check for t in _TARGET)]
    return tgt, {f.check: f"{f.state}/{f.measured}" for f in fs}


def run() -> dict:
    bad, bad_all = _observe(True)
    good, good_all = _observe(False)
    # 掏空样本：solid_retention 必须红；min_section 在保留比例已红时可能不发/也红，只要求"至少一条红且都是 BLOCK"
    ok_red, why_red, red = assert_red(bad, severity="BLOCK", require_measured=True)
    ok_green, why_green = assert_green(good)
    return result(NAME, EXPECT, ok_red and ok_green,
                  got=f"掏空样本 {'红' if ok_red else why_red}({[(r['check'], r['measured']) for r in red]}) / 完整板对照 {'绿' if ok_green else why_green}",
                  red=red, expect_severity="BLOCK",
                  detail=f"掏空样本全部判据={bad_all}｜完整板对照={good_all}")


if __name__ == "__main__":
    import json
    print(json.dumps(run(), ensure_ascii=False, indent=1))
