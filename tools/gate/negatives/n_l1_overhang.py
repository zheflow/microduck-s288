#!/usr/bin/env python3
"""反例 L1-B —— 封闭腔里的 70° 悬垂顶：支撑长在拿不出来的地方，必须红。

这一层真会漏掉的样子：
  "悬垂面积 mm²"这种纯统计量谁都会算，而且它**永远不会红** —— 它只是个数。
  真正会把件毁掉的是：悬垂在一个**封闭内腔**里，切片器照样生成支撑，而支撑拆不出来。
  只统计悬垂面积、不追支撑落到哪里的检查器，对这种件全绿。

坏样本：40×40×30 的实心块，内部挖一个**完全封闭**的锥形腔，锥壁离竖直 70°
        （每层水平外探 h·tan70° = 0.55 mm ≫ 判据步距 h·tan45° = 0.2 mm）→ 顶面无支承 →
        支撑柱长在腔里 → support_reachable 红。
对照：同一个块、同一个封闭腔，只把锥壁改成离竖直 30°（每层 0.115 mm < 0.2 mm，自支承）→
      腔里不该有任何支撑 → 必须绿。

两个样本只差一个角度，所以它同时证明了**悬垂角判据真的在起作用**，
而不是"凡有内腔就报红"。判据角度取 printability.yaml:criteria.support_model_overhang_deg（45°），
30/70 正好把它和 README 的 55° 一起夹在中间。
"""
from __future__ import annotations

import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from _harness import FakeCtx, base_data, box, export_stl, findings, assert_red, assert_green, result  # noqa: E402

PART = "H01"
# support_reachable 现在是 WARN（l1_printable.py:1129）。反例记录层的真实严重级，不为凑 BLOCK 改层。
SEVERITY = "WARN"
NAME = "封闭腔 70° 悬垂顶：support_reachable 必须红 [WARN]，30° 自支承腔必须绿"
EXPECT = f"L1/{PART}:support_reachable FAIL [WARN]（支撑落在截面封闭内环里，> 0 mm³）"

BLOCK = (40.0, 40.0, 30.0)
CAVITY_BASE_Z = 8.0
CAVITY_H = 8.0            # 腔高固定，只改锥底半径 → 只改壁角


def _mesh(deg_from_vertical: float):
    """锥形封闭腔：锥尖朝上。锥壁离竖直的角度 = atan(R/H)。"""
    import trimesh
    blk = box(*BLOCK, center=(0.0, 0.0, BLOCK[2] / 2))
    r = CAVITY_H * math.tan(math.radians(deg_from_vertical))
    cone = trimesh.creation.cone(radius=r, height=CAVITY_H, sections=96)
    cone.apply_translation([0.0, 0.0, CAVITY_BASE_Z])
    return blk.difference(cone)


def _data():
    part = {"id": PART, "inventory_id": f"{PART}_neg_overhang", "material": "PLA",
            "print_orientation": "块底面朝下",
            "print_orientation_down_normal": [0, 0, -1],
            "print_orientation_frame": "export_local", "mirrored_copy": None}
    return base_data([part], [])


def _observe(deg: float):
    import l1_printable
    stl = export_stl(_mesh(deg), f"l1_overhang_{int(deg)}deg")
    res = l1_printable.run(FakeCtx(_data(), {PART: stl}))
    fs = findings(res, subject=PART, check="support_reachable")
    oh = findings(res, subject=PART, check="overhang_area")
    return fs, (oh[0].measured if oh else None)


def run() -> dict:
    bad, bad_oh = _observe(70.0)
    good, good_oh = _observe(30.0)
    ok_red, why_red, red = assert_red(bad, severity=SEVERITY, require_measured=True)
    ok_green, why_green = assert_green(good)
    return result(NAME, EXPECT, ok_red and ok_green,
                  got=(f"70° 腔 {'红' if ok_red else why_red}(腔内支撑={bad[0].measured if bad else None} mm³, 悬垂面积={bad_oh} mm²) / "
                       f"30° 腔对照 {'绿' if ok_green else why_green}(腔内支撑={good[0].measured if good else None} mm³, 悬垂面积={good_oh} mm²)"),
                  red=red, expect_severity=SEVERITY,
                  detail=f"坏样本：{(bad[0].detail if bad else '')[:110]}｜对照：{(good[0].detail if good else '')[:110]}")


if __name__ == "__main__":
    import json
    print(json.dumps(run(), ensure_ascii=False, indent=1))
