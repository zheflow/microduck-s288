#!/usr/bin/env python3
"""反例 L1-A —— 厚块上挂一片 0.5 mm 薄筋：放不下 2 圈墙必须红。

这一层真会漏掉的样子（不是随手造的坏样本）：
  最省事的"最薄壁"写法是 `poly.buffer(-r).is_empty` —— 整层被腐蚀光才算薄。
  可是真实零件的薄筋总是**长在厚块上**：厚块那部分怎么腐蚀都还在，整层永远非空，
  于是 0.5 mm 的筋一路绿灯。H02 那条 0.444 mm 窄筋当年就是这么活过检查的
  （parts.yaml:deliberately_added.H02 记着这件事）。
  正确写法是**开运算**（腐蚀再膨胀）后看丢了哪些区域，而不是看整层还剩不剩。

同时它还卡住第二个坑：buffer 的数值容差。0.5 mm 筋对 2×0.45=0.90 mm 判据只差 0.4 mm，
不许被 shapely 的圆角/容差吞掉。

对照：同一个块 + 2.0 mm 筋必须仍然绿 —— 判据不能写成恒红。
"""
from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from _harness import FakeCtx, base_data, box, export_stl, findings, assert_red, assert_green, result  # noqa: E402

PART = "L01"
# min_wall_geometric 现在是 WARN（l1_printable.py:791）：几何腐蚀是切片真值 min_wall_sliced 的旁证。
# 反例只记录层的真实严重级，不为凑 BLOCK 改层；层升回 BLOCK 时本反例必须跟着改（runner 会抓）。
SEVERITY = "WARN"
NAME = "厚块上的 0.5 mm 薄筋：min_wall_geometric 必须红 [WARN]，2.0 mm 筋必须绿"
EXPECT = f"L1/{PART}:min_wall_geometric FAIL [WARN]（局部宽度 0.5 < 2×0.45 = 0.90 mm，且连续 ≥3 层）"

BLOCK = (12.0, 12.0, 10.0)      # 厚块：腐蚀多少都还剩一大块 → 整层永远非空
FIN_L, FIN_H = 20.0, 10.0       # 薄筋：长 20、和块同高，只有厚度在变


def _mesh(fin_t: float):
    import trimesh
    blk = box(*BLOCK, center=(0.0, 0.0, BLOCK[2] / 2))
    fin = box(FIN_L, fin_t, FIN_H, center=(BLOCK[0] / 2 + FIN_L / 2 - 0.5, 0.0, FIN_H / 2))
    return trimesh.util.concatenate([blk, fin]).union(fin)


def _data():
    part = {"id": PART, "inventory_id": f"{PART}_neg_thinwall", "material": "PLA",
            "print_orientation": "平放，薄筋厚度在 XY 截面里",
            # 件自身坐标系里 -z 就是贴床方向 → 朝向变换是单位阵，截面 = XY
            "print_orientation_down_normal": [0, 0, -1],
            "print_orientation_frame": "export_local", "mirrored_copy": None}
    return base_data([part], [])


def _observe(fin_t: float):
    import l1_printable
    stl = export_stl(_mesh(fin_t), f"l1_thinwall_{str(fin_t).replace('.', 'p')}")
    res = l1_printable.run(FakeCtx(_data(), {PART: stl}))
    return findings(res, subject=PART, check="min_wall_geometric")


def run() -> dict:
    bad = _observe(0.5)
    good = _observe(2.0)
    ok_red, why_red, red = assert_red(bad, severity=SEVERITY, require_measured=True)
    ok_green, why_green = assert_green(good)
    return result(NAME, EXPECT, ok_red and ok_green,
                  got=(f"0.5 mm 筋 {'红' if ok_red else why_red}(最薄壁={bad[0].measured if bad else None}) / "
                       f"2.0 mm 筋对照 {'绿' if ok_green else why_green}(最薄壁={good[0].measured if good else None})"),
                  red=red, expect_severity=SEVERITY,
                  detail=f"坏样本：{(bad[0].detail if bad else '')[:110]}｜对照：{(good[0].detail if good else '')[:110]}")


if __name__ == "__main__":
    import json
    print(json.dumps(run(), ensure_ascii=False, indent=1))
