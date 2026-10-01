#!/usr/bin/env python3
"""反例（2026-09-21，hr26 复审 F-11）—— KO01 背插模型里"W 只对载体查"这条放宽的边界必须守住：

  09-21 起 KO01 的 W（背板通窗 x −18.5..−12.8）只对 rows[].carrier 那一件查（通窗是背板的义务），
  板外邻件只须让 A（插头顶/线翘区 x −17.4..−12.8）。放宽之后没有反例守着两侧边界：
    · 邻件闯进 A 必须红 —— 否则"只查载体"会被误写成"邻件什么都不查"；
    · 邻件在 W 但 A 之外（x −18.5..−17.4）不许红 —— 否则放宽等于没放；
    · 同一块料放在载体上必须红 —— W 对载体仍然是义务。
场景（真实运动学 duckstructure.lib.sfw 定舵机落位；件是造的；只留 trunk_base[0] 一行，−y 侧 window，carrier=C01）：
  · nbr_A  ：邻件 X02 2×2×4 方块放在舵机局部 (x −15.0, y −7.75, z −9.3)（x −16..−14，在 A 里）→ X02 keepout_KO01 FAIL(BLOCK) ≈16 mm³。
  · nbr_W  ：邻件 X02 0.8×2×4 方块放在 (x −18.0, …)（x −18.4..−17.6，只在 W 里）→ X02 无红、KO01 keepout_intersection PASS。
  · carr_W ：同一块 W 料换成载体 C01 → C01 keepout_KO01 FAIL(BLOCK) ≈6.4 mm³。
"""
from __future__ import annotations
import copy
import json
import os
import sys
from pathlib import Path

_HERE = Path(__file__).resolve()
GATE = _HERE.parents[1]
for _p in (str(_HERE.parent), str(GATE), str(GATE / "layers"), str(GATE.parents[1])):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import core                                                                 # noqa: E402
from _harness import FakeCtx, base_data, findings, assert_red, assert_green, result, record  # noqa: E402

CARR, NBR = "C01", "X02"
NAME = "KO01 W 只查载体的边界：邻件闯 A 必须红；邻件只在 W（x −18.5..−17.4）不许红；载体在 W 必须红"
EXPECT = ("nbr_A: L3/X02:keepout_KO01 FAIL(BLOCK) ≈16 mm³；nbr_W: X02 无 keepout_KO01 红 且 KO01:keepout_intersection PASS；"
          "carr_W: L3/C01:keepout_KO01 FAIL(BLOCK) ≈6.4 mm³")
_TMP = core.ROOT / "tools/gate/out/_neg_tmp"
SERVO = ("trunk_base", 0)


def _world_box(size, local_center):
    import numpy as np
    import trimesh
    from duckstructure.lib import sfw
    R = np.asarray(sfw(*SERVO), float)
    m = trimesh.creation.box(extents=size)
    m.apply_translation(local_center)
    m.apply_transform(R)
    return m


def _placed_dir(tag, carrier, neighbor):
    d = _TMP / f"p{os.getpid()}" / f"l3_ko01_nbr_{tag}"
    d.mkdir(parents=True, exist_ok=True)
    for p in d.glob("*.stl"):
        p.unlink()
    pc, pn = d / "carrier.stl", d / "neighbor.stl"
    carrier.export(str(pc), file_type="stl"); neighbor.export(str(pn), file_type="stl")
    return d, pc, pn


def _data():
    d = base_data([{"id": CARR, "inventory_id": "C01_carrier", "material": "PLA", "print_orientation": "任意",
                    "mirrored_copy": None, "qty": 1, "placed_instances": ["carrier"]},
                   {"id": NBR, "inventory_id": "X02_neighbor", "material": "PLA", "print_orientation": "任意",
                    "mirrored_copy": None, "qty": 1, "placed_instances": ["neighbor"]}], [])
    ko01 = copy.deepcopy(next(k for k in d["keepouts"]["keepouts"] if k["id"] == "KO01"))
    ko01["availability_table"] = {"rows": [{"servo": f"{SERVO[0]}[{SERVO[1]}]", "mode_pos_y": "none", "mode_neg_y": "window",
                                            "carrier": f"{CARR} 载体背板"}]}
    assert ko01["dims"]["window_x_mm"]["v"] == -18.5, ko01["dims"]["window_x_mm"]
    d["keepouts"] = {"keepouts": [ko01]}
    d["components"] = {"components": []}
    d["frozen"] = {}
    return d


def _observe(tag, carrier, neighbor):
    import l3_static as L3
    placed, pc, pn = _placed_dir(tag, carrier, neighbor)
    old = L3.PLACED
    try:
        L3.PLACED = placed
        return L3.run(FakeCtx(_data(), {CARR: pc, NBR: pn}, {p.stem: p for p in placed.glob("*.stl")}))
    finally:
        L3.PLACED = old


def run() -> dict:
    far = _world_box((4.0, 4.0, 4.0), (-40.0, 0.0, -9.3))              # 什么都不碰的占位件
    in_a = _world_box((2.0, 2.0, 4.0), (-15.0, -7.75, -9.3))           # x −16..−14：A 区（−17.4..−12.8）里
    in_w = _world_box((0.8, 2.0, 4.0), (-18.0, -7.75, -9.3))           # x −18.4..−17.6：只在 W（−18.5..−12.8），A 之外

    r_a = _observe("nbr_A", far, in_a)
    ok_a, why_a, red_a = assert_red(findings(r_a, subject=NBR, check="keepout_KO01"), severity="BLOCK")
    ok_a2, why_a2, red_a2 = assert_red(findings(r_a, subject="KO01", check="keepout_intersection"), severity="BLOCK")
    va = float(red_a[0]["measured"]) if red_a else None
    va_ok = va is not None and 12.0 < va < 20.0                        # 2×2×4 = 16 全在 A 里

    r_w = _observe("nbr_W", far, in_w)
    nbr_red = [f for f in findings(r_w, subject=NBR, check="keepout_KO01") if f.state == "FAIL"]
    ok_w, why_w = assert_green(findings(r_w, subject="KO01", check="keepout_intersection"))
    w_ok = ok_w and not nbr_red

    r_c = _observe("carr_W", in_w, far)
    ok_c, why_c, red_c = assert_red(findings(r_c, subject=CARR, check="keepout_KO01"), severity="BLOCK")
    vc = float(red_c[0]["measured"]) if red_c else None
    vc_ok = vc is not None and 4.0 < vc < 8.0                          # 0.8×2×4 = 6.4 全在 W 里

    passed = ok_a and ok_a2 and va_ok and w_ok and ok_c and vc_ok
    return result(NAME, EXPECT, passed,
                  got=(f"邻件闯 A：件格 {'红' if ok_a else why_a}（{va} mm³），KO01 格 {'红' if ok_a2 else why_a2}；"
                       f"邻件只在 W：KO01 格 {'绿' if ok_w else why_w}，邻件红 {len(nbr_red)} 条；"
                       f"载体在 W：{'红' if ok_c else why_c}（{vc} mm³）"),
                  red=red_a + red_a2 + red_c, expect_severity="BLOCK",
                  detail=f"nbr_W 邻件 findings={[record(f) for f in findings(r_w, subject=NBR)]}")


if __name__ == "__main__":
    print(json.dumps(run(), ensure_ascii=False, indent=1))
