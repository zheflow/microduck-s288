#!/usr/bin/env python3
"""反例 FG09 —— 右件的槽挪了 1 mm，包围盒和圆孔中心都没动：镜像判据不红。

坑的形状：镜像比对只做两件事 —— 比包围盒、比"左件圆柱面 → 右件最近邻圆柱面"。
非圆柱的东西（槽、筋、台、平面）根本没进比对；最近邻是单向的、没有双射，
右件多出或少掉的特征也看不见；轴向（孔轴方向、孔的轴向跨度）一次都没比。

正确判据：双向（左→右 且 右→左）实例对应 + 轴向比较 + 非圆柱表面差异
（镜像后逐点到对面**表面**的最大距离）。
对照：右件是真镜像时必须仍然绿。
"""
from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from _harness import FakeCtx, base_data, box, cyl, export_stl, findings, assert_red, assert_green, result  # noqa: E402

PART = "L03"
NAME = "右件槽挪 1 mm（bbox 与圆孔中心不变）：镜像表面差异必须红，真镜像必须绿"
EXPECT = f"L2/{PART}:mirror:surface（镜像后逐点最近表面距离 1.0 > 0.05）"
_TARGET = ("mirror:surface", "mirror:features")


def _plate(hole_y, slot_x, slot_y):
    """20×20×4 板 + 一个 Ø4 通孔（唯一的圆柱面）+ 一条内部方槽（纯平面，进不了圆柱比对）。"""
    m = box(20, 20, 4).difference(cyl(2.0, 20.0, center=(5.0, hole_y, 0.0)))
    return m.difference(box(4.0, 2.0, 2.0, center=(slot_x, slot_y, 1.0)))


def _left():
    return _plate(+5.0, -5.0, +3.0)


def _right(bad: bool):
    # 真镜像 = 孔到 y=−5、槽到 (−5,−3)；坏样本把槽沿 x 挪 1 mm（包围盒与孔心都不变）
    return _plate(-5.0, -4.0 if bad else -5.0, -3.0)


def _data():
    part = {"id": PART, "inventory_id": "L03_fg09", "material": "PLA",
            "print_orientation": "背板朝下", "mirrored_copy": "L03_R（mirror_y）"}
    feat = {"id": "L03-FG09", "part": PART, "kind": "screw_hole", "check_class": "screw_hole",
            "count": 1, "spec_verbatim": "1×Ø4 通",
            "geom": {"shape": "cylinder", "frame": "export_local",
                     "nominal_d_mm": {"v": 4.0, "src": "measured"},
                     "through": True, "axis": "z", "pos": [5.0, 5.0, 0.0],
                     "depth_kind": "cutter_length", "depth_mm": {"v": 4.0, "src": "assumed"},
                     "axial_span_mm": [[5.0, 5.0, -2.0], [5.0, 5.0, 2.0]]}}
    return base_data([part], [feat])


def _observe(bad: bool):
    import l2_features
    l2_features._GEOM_CACHE.clear()
    tag = "bad" if bad else "true"
    pl = export_stl(_left(), f"fg09_L_{tag}")
    pr = export_stl(_right(bad), f"fg09_R_{tag}")
    # placed 索引的 key 必须是 inventory_id 的子串（l2_features.placed_pair 就是这么认左右件的）
    ctx = FakeCtx(_data(), {PART: pl}, placed_map={"fg09": pl, "fg09_R": pr})
    res = l2_features.run(ctx)
    fs = [f for f in findings(res, subject=PART) if f.check.startswith("mirror")]
    tgt = [f for f in fs if any(t in f.check for t in _TARGET)]
    return tgt, {f.check: f"{f.state}/{f.measured}" for f in fs}


def run() -> dict:
    bad, bad_all = _observe(True)
    good, good_all = _observe(False)
    # 挪槽只动了平面：红必须落在 mirror:surface（圆孔中心没动，mirror:features 允许仍绿）
    ok_red, why_red, red = assert_red(bad, severity="BLOCK", require_measured=True, check="mirror:surface")
    ok_green, why_green = assert_green(good)
    return result(NAME, EXPECT, ok_red and ok_green,
                  got=f"挪槽样本 {'红' if ok_red else why_red}(surface 最大距离={red[0]['measured'] if red else None}) / 真镜像对照 {'绿' if ok_green else why_green}",
                  red=red, expect_severity="BLOCK",
                  detail=f"挪槽样本镜像判据={bad_all}｜真镜像对照={good_all}")


if __name__ == "__main__":
    import json
    print(json.dumps(run(), ensure_ascii=False, indent=1))
