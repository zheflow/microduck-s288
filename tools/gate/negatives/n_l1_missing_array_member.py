#!/usr/bin/env python3
"""反例 —— 禁撑孔阵列不能只检查第一个孔：缺任一孔的位置，整条特征必须判 unknown，而不是只查剩下的。

经过 `l1_printable.run(ctx)`（禁撑区那一段调用 _feature_cylinders）：
  一块 50×20×10 平板（没有任何悬垂 → 支撑上界为 0），features.yaml 声明一条 2 孔 Ø2.2 阵列（kind=screw_hole）。
  好    positions_mm 给全 2 个 → 禁撑区建得起来 → support_in_no_support_zone PASS（上界干净）
  坏 A  positions_mm 只给 1 个（count 仍是 2）→ 定位不到 → **unknown**（FAIL(BLOCK)、measured=None）
  坏 B  嵌套 instances 里删掉一组                → 同上
  坏 C  同一组重复两次冒充补齐                    → 同上
坏样本的红是 unknown（算不出来），这正是本反例要验的设计 —— allow_unknown_red。
函数级的逐条断言保留在 detail 里作旁证，红以 run 发出的 finding 为准。
"""
from __future__ import annotations
import json
import sys
from copy import deepcopy
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from _harness import FakeCtx, base_data, box, export_stl, findings, assert_red, assert_green, result, record  # noqa: E402

PART = "H03"
FID = "H03-NEG-ARRAY"
NAME = "禁撑区覆盖 positions_mm 全部孔：缺实例 / 删一组 / 重复冒充补齐 → unknown；完整声明绿"
EXPECT = f"L1/{PART}:support_in_no_support_zone 坏声明 FAIL(BLOCK) 且 measured=None（unknown）；完整声明 PASS"
CHECK = "support_in_no_support_zone"

PLATE = (50.0, 20.0, 10.0)
G = {"shape": "cylinder", "frame": "export_local", "axis": "z",
     "nominal_d_mm": {"v": 2.2, "src": "measured"}, "depth_mm": {"v": 10.0, "src": "assumed"},
     "depth_kind": "cutter_length", "through": True,
     "axial_span_mm": [[0.0, 0.0, 0.0], [0.0, 0.0, 10.0]],
     "count": 2, "positions_mm": [[-10.0, 0.0, 5.0], [10.0, 0.0, 5.0]]}


def _data(geom, count=2):
    part = {"id": PART, "inventory_id": f"{PART}_neg_array", "material": "PLA",
            "print_orientation": "平板朝下", "print_orientation_down_normal": [0, 0, -1],
            "print_orientation_frame": "export_local", "mirrored_copy": None}
    feat = {"id": FID, "part": PART, "kind": "screw_hole", "check_class": "screw_hole",
            "count": count, "purpose": "反例：2 孔阵列", "spec_verbatim": "2×Ø2.2 通", "geom": geom}
    return base_data([part], [feat])


def _observe(geom, tag, count=2):
    """09-13 起主判据按真 G-code 落点记录判：对照样本要绿必须有 support_landing 记录，
    这里用 _harness.landing_stub 塞一条**没有任何支撑线**的记录（平板没有悬垂，切片器也不会撑）；
    坏声明在禁撑区定位那一步就 unknown，与记录无关。"""
    import l1_printable
    from _harness import landing_stub
    stl = export_stl(box(*PLATE, center=(0.0, 0.0, PLATE[2] / 2)), f"l1_array_{tag}")
    data, _ = landing_stub(_data(geom, count), PART, stl, [])
    res = l1_printable.run(FakeCtx(data, {PART: stl}))
    return findings(res, subject=PART, check=CHECK)


def _function_level():
    """旁证：直接调 _feature_cylinders 的逐条计数（原反例），红仍以 run 的 finding 为准。"""
    from layers.l1_printable import _feature_cylinders
    g = {"axial_span_mm": [[0, 0, 0], [0, 0, 3]], "nominal_d_mm": 2.2,
         "count": 2, "positions_mm": [[0, 0, 1.5], [5, 0, 1.5]]}
    other = {**g, "positions_mm": [[0, 5, 1.5], [5, 5, 1.5]]}
    parent = {"count": 2, "count_kind": "instances", "instances": [g, other]}
    holes_alias = deepcopy(g); holes_alias["holes"] = holes_alias.pop("count")
    total = {**parent, "count_kind": "cylinders", "count": 4}
    cases = [({"id": "a", "geom": g}, 2),
             ({"id": "a", "geom": {**g, "positions_mm": g["positions_mm"][:1]}}, 0),
             ({"id": "a", "geom": {**g, "positions_mm": []}}, 0),
             ({"id": "n", "geom": parent}, 4),
             ({"id": "n", "geom": {**parent, "instances": [g, {}]}}, 0),
             ({"id": "n", "geom": {**parent, "instances": [g]}}, 0),
             ({"id": "a", "geom": {**holes_alias, "positions_mm": [[0, 0, 1.5]]}}, 0),
             ({"id": "t", "geom": total}, 4),
             ({"id": "t", "geom": {**total, "instances": [g]}}, 0),
             ({"id": "n", "geom": {**parent, "instances": [g, g]}}, 0)]
    got = [len(_feature_cylinders(f)) for f, _ in cases]
    return got, [w for _, w in cases]


def run() -> dict:
    good = _observe(deepcopy(G), "good")
    other = {**deepcopy(G), "positions_mm": [[-10.0, 5.0, 5.0], [10.0, 5.0, 5.0]]}
    parent = {"shape": "cylinder", "frame": "export_local", "count": 2, "count_kind": "instances",
              "instances": [deepcopy(G), other]}
    bads = {
        "A少一个位置": (deepcopy({**G, "positions_mm": G["positions_mm"][:1]}), 2),
        "B嵌套删一组": ({**parent, "instances": [deepcopy(G)]}, 2),
        "C重复冒充补齐": ({**parent, "instances": [deepcopy(G), deepcopy(G)]}, 2),
    }
    reds, got, ok_all = [], [], True
    for tag, (geom, cnt) in bads.items():
        fs = _observe(geom, tag[0], cnt)
        ok, why, rec = assert_red(fs, severity="BLOCK", require_measured=False)
        # 必须是 unknown（定位不到），不是"量到了支撑落进孔"
        is_unknown = ok and all(r["measured"] is None for r in rec) and any("定位不到" in f.detail for f in fs)
        ok_all &= is_unknown
        reds += rec
        got.append(f"{tag}:{'unknown' if is_unknown else ('红但非 unknown' if ok else why)}")
    ok_green, why_green = assert_green(good)
    got.append(f"完整声明:{'绿' if ok_green else why_green}(上界={good[0].measured if good else None})")
    fn_got, fn_want = _function_level()
    fn_ok = fn_got == fn_want
    return result(NAME, EXPECT, ok_all and ok_green and fn_ok, got="；".join(got) + f"；函数级旁证 {'一致' if fn_ok else '不一致'}",
                  red=reds, expect_severity="BLOCK",
                  allow_unknown_red="坏声明（孔阵位置不全 / 组缺失 / 重复冒充）必须判 unknown：定位不到的禁撑区不能只查剩下的孔，"
                                    "measured=None 是设计（l1_printable.py 禁撑区 unloc 分支）",
                  detail=f"函数级 _feature_cylinders 计数 got={fn_got} want={fn_want}；坏样本记录={reds}")


if __name__ == "__main__":
    print(json.dumps(run(), ensure_ascii=False, indent=1))
