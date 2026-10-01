#!/usr/bin/env python3
"""反例 L1-C —— 支撑挤出线穿过 Ø8 轴承孔：support_in_no_support_zone 必须红（forbid 级），孔挪开必须绿。

这一层真会漏掉的样子：
  "这件要不要支撑"和"支撑落在哪里"是两个问题。只算前者的检查器，对下面这个件全绿 ——
  它当然要支撑，切片器也确实生成了，一切正常；坏就坏在那根支撑柱**正好从 Ø8 轴承孔里穿下去**。
  孔里留一圈支撑疤，轴承压不进。这正是 README 第 2 节 L1 那句"支撑不落在配合面/孔内"要抓的东西。

2026-09-13 起判据按**真 G-code 采样点**判（审计 F-L1-3 落地版，不再用 2D 支撑柱模型夹逼）：
  本反例用 `_harness.landing_stub` 合成一段 `;TYPE:Support material` 挤出线，走 slice_l1.support_landing 的真实
  解析 / 采样 / 逆变换 / 对 l1_printable.no_support_zone_set 数点，塞进 slice_run.parts[件].support_landing。
  · 坏：孔在悬臂正下方，支撑线（z=5，孔内高度）穿过孔心 → forbid 级（kind=bearing_bore）落点 > 0
        → support_in_no_support_zone FAIL(BLOCK)，measured = 落点折算路径 mm
  · 好：孔挪到悬臂盖不到的那一头，**同一条支撑线位置不变** → 孔里没有采样点 → PASS（evidence_n = 禁撑区数 > 0）
两个样本共用完全相同的支撑几何与同一条支撑线，只有孔在动 —— 证明的是"落点判据"，不是"有没有支撑"。
孔的几何从 features.yaml 读（kind=bearing_bore ∈ printability.yaml:no_support_zones.tiers.forbid.kinds）。

网格里孔用 8×8 方孔切（禁撑区仍按声明的 Ø8 圆柱建）：圆孔的 96 边形会让层里逐层 shapely 支撑模型
（现在只作 support_model_zone_crosscheck）膨胀到 400+ s，方孔 1.6 s。
"""
from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from _harness import (FakeCtx, base_data, box, export_stl, findings, assert_red, assert_green,   # noqa: E402
                      result, landing_stub)

PART = "L07"
FID = "L07-NEG-BORE"
NAME = "支撑挤出线穿过 Ø8 轴承孔（forbid 级）：support_in_no_support_zone 必须红，孔挪开必须绿"
EXPECT = f"L1/{PART}:support_in_no_support_zone FAIL(BLOCK)（真 G-code 采样点落进 {FID} 的 Ø8 孔，measured = 路径 mm > 0）；孔挪开 PASS"

PLATE = (50.0, 20.0, 10.0)          # 底板 x −20..30
PLATE_CX = 5.0
PILLAR_X = -17.5                     # 立柱 x −20..−15，z 10..20
CANTI_X0, CANTI_X1 = -20.0, 15.0     # 悬臂 z 20..22，只有 x≈−17.5 一端有立柱托着
BORE_D = 8.0
BAD_X, GOOD_X = 0.0, 25.0            # 坏：悬臂正下方；好：悬臂盖不到的那一头
SUPPORT_LINE = ((-6.0, 0.0, 5.0), (6.0, 0.0, 5.0))   # 支撑挤出线（export_local）：穿过 x=0 处的孔心，z=5 在板内


def _mesh(bore_x: float):
    plate = box(*PLATE, center=(PLATE_CX, 0.0, PLATE[2] / 2))
    pillar = box(5.0, 20.0, 10.0, center=(PILLAR_X, 0.0, 15.0))
    canti = box(CANTI_X1 - CANTI_X0, 20.0, 2.0,
                center=((CANTI_X0 + CANTI_X1) / 2, 0.0, 21.0))
    solid = plate.union(pillar).union(canti)
    return solid.difference(box(BORE_D, BORE_D, 30.0, center=(bore_x, 0.0, 5.0)))   # 方孔，理由见文件头


def _data(bore_x: float):
    part = {"id": PART, "inventory_id": f"{PART}_neg_bore", "material": "PLA",
            "print_orientation": "底板朝下",
            "print_orientation_down_normal": [0, 0, -1],
            "print_orientation_frame": "export_local", "mirrored_copy": None}
    feat = {"id": FID, "part": PART, "kind": "bearing_bore", "check_class": "bearing_bore",
            "count": 1, "purpose": "轴承外圈座（反例）",
            "spec_verbatim": f"1×Ø{BORE_D} 通孔，轴 z",
            "geom": {"shape": "cylinder", "frame": "export_local",
                     "nominal_d_mm": {"v": BORE_D, "src": "measured"},
                     "depth_mm": {"v": PLATE[2], "src": "assumed"},
                     "depth_kind": "cutter_length", "through": True, "axis": "z",
                     "pos": [bore_x, 0.0, PLATE[2] / 2],
                     "axial_span_mm": [[bore_x, 0.0, 0.0], [bore_x, 0.0, PLATE[2]]]}}
    return base_data([part], [feat])


def _observe(bore_x: float, tag: str):
    import l1_printable
    stl = export_stl(_mesh(bore_x), f"l1_bore_{tag}")
    data, landing = landing_stub(_data(bore_x), PART, stl, [SUPPORT_LINE])
    res = l1_printable.run(FakeCtx(data, {PART: stl}))
    return findings(res, subject=PART, check="support_in_no_support_zone"), landing


def run() -> dict:
    bad, l_bad = _observe(BAD_X, "under_cantilever")
    good, l_good = _observe(GOOD_X, "clear")
    ok_red, why_red, red = assert_red(bad, severity="BLOCK", require_measured=True)
    ok_green, why_green = assert_green(good)
    return result(NAME, EXPECT, ok_red and ok_green,
                  got=(f"孔在支撑线下 {'红' if ok_red else why_red}(孔内落点路径={bad[0].measured if bad else None} mm，"
                       f"桩落点 forbid {l_bad['hits_forbid_samples']} 点) / "
                       f"孔挪开对照 {'绿' if ok_green else why_green}(={good[0].measured if good else None} mm，"
                       f"桩落点 forbid {l_good['hits_forbid_samples']} 点)"),
                  red=red, expect_severity="BLOCK",
                  detail=(f"坏样本：{(bad[0].detail if bad else '')[:160]}｜对照：{(good[0].detail if good else '')[:120]}"))


if __name__ == "__main__":
    import json
    r = run()
    print(json.dumps(r, ensure_ascii=False, indent=1))
    sys.exit(0 if r["passed"] else 1)
