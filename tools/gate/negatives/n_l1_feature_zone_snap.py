#!/usr/bin/env python3
"""反例 hr41g —— 卡扣/舌片这类**凸出实体**特征要能按特征 id 纳入禁撑区，且区 = 特征盒外扩面邻域，不是特征盒本身。

起因（hr39f，Codex）：B03 前舌/凸点/后卡舌/倒钩是 kind=lip / snap，不在 printability.yaml:no_support_zones 的 kind 列表里
→ 正式 L1 对 B03 禁撑区数 = 0 → 判 unknown（正确，没有假绿）。但光把 lip / snap 加进 kind 列表不行：
  · B03 这些特征用 bbox_mm / prism 登记，旧 _feature_boxes 只认 box_export_local → 定位不到 → 仍 unknown；
  · lip / snap 是**实体**：支撑挤出中心线只会贴在面外，落不进实体盒里 —— 用特征盒本身当禁撑区恒 0 点 = 假绿；
  · 全局加 kind 会把 14 条轴承挡边 lip、T01-F14 卡珠（L1 定位不到）一起拖进来，改动它们件的 zones_sha256。
修法（l1_printable.no_support_zone_set）：printability.yaml:no_support_zones.feature_zones.<特征 id> =
  {tier: forbid|removable, face_margin_mm: {v>0, src}, why, src[, post_process]} 按 id 纳入（不改 kind、不波及别件），
  区 = 该特征每个实例的 box_export_local / bbox_mm / prism 外包盒各向外扩 face_margin_mm。

几何：底板 30×20×4（z 0..4）+ 顶面凸点（prism，x −0.5..0.5 / y ±2.5 / z 3.9..4.35，底部埋进底板 0.1）；
特征 B03-NEG-F01 kind=snap（prism），B03-NEG-F02 kind=lip（instances[].bbox_mm，x 10..15 / y ±10 / z 0..4）。
  A 不声明 feature_zones                         → support_in_no_support_zone unknown（= 现役 B03 状态，空清单不是通过）
  B F01 forbid + margin 0.45，支撑线在凸点顶上 0.2 → support_in_no_support_zone FAIL(BLOCK) measured=落点 mm
  C 同 B，支撑线远离两特征                          → support_in_no_support_zone PASS、support_in_removable_zone PASS
  D F02 removable，支撑线在舌片盒外 0.2 以内       → forbid PASS、support_in_removable_zone FAIL(WARN)
  E F01 缺 face_margin_mm / margin=0 / tier 写错   → unknown（problems）
  F feature_zones 指向没有 bbox/prism 的特征        → unknown（定位不到）
  G feature_zones 指向不存在的特征 id               → unknown（problems）
支撑位置由 _harness.landing_stub 合成 G-code 给出，走 slice_l1.support_landing 的真实解析/采样/逆变换。
"""
from __future__ import annotations

import copy
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from _harness import (FakeCtx, base_data, box, export_stl, findings, assert_red, assert_green,   # noqa: E402
                      result, record, landing_stub)

PART = "B03"
F01, F02, F03 = "B03-NEG-F01", "B03-NEG-F02", "B03-NEG-F03"
NAME = "卡扣/舌片实体特征按 id 纳入禁撑区（feature_zones，面邻域外扩）：贴凸点的支撑必须红，不纳入/声明坏 unknown"
EXPECT = (f"L1/{PART}:support_in_no_support_zone FAIL(BLOCK)（支撑线贴 {F01} 凸点顶 0.2 mm）；远离 PASS；"
          f"{F02} removable → support_in_removable_zone FAIL(WARN)；未纳入 / margin 缺或 0 / tier 坏 / 无盒 / 无此特征 → unknown")

LINE_NEAR_BUMP = ((-0.3, 0.0, 4.55), (0.3, 0.0, 4.55))
LINE_FAR = ((-12.0, 0.0, 4.55), (-8.0, 0.0, 4.55))
LINE_NEAR_LIP = ((11.0, 0.0, 4.2), (12.0, 0.0, 4.2))
MARGIN = {"v": 0.45, "src": "assumed", "src_note": "反例：与 hr39f check_b03_support_2026-09-24.py 邻域同值"}


BUMP_UV = [(-0.5, 3.9), (0.5, 3.9), (0.3, 4.35), (-0.3, 4.35)]     # 凸点截面（x, z），底部埋进底板 0.1


def _mesh():
    import numpy as np
    import trimesh
    from shapely.geometry import Polygon
    plate = box(30.0, 20.0, 4.0, center=(0.0, 0.0, 2.0))
    bump = trimesh.creation.extrude_polygon(Polygon(BUMP_UV), 5.0)    # 多边形在 xy、沿 +z 挤 5
    bump.apply_transform(trimesh.transformations.rotation_matrix(np.pi / 2, [1, 0, 0]))   # (x,v,t) → (x,−t,v)
    bump.apply_translation([0.0, 2.5, 0.0])                            # y −2.5..2.5
    return plate.union(bump)


def _features():
    prism = {"origin": [0.0, 2.5, 0.0], "right": [1, 0, 0], "up": [0, 0, 1], "normal": [0, 1, 0], "depth_mm": 5.0,
             "polygons": [{"exterior": [list(p) for p in BUMP_UV], "holes": []}]}
    return [
        {"id": F01, "part": PART, "kind": "snap", "check_class": "snap", "count": 1, "purpose": "凸点（反例）",
         "geom": {"frame": "export_local", "count_kind": "instances", "count": 1,
                  "instances": [{"shape": "prism", "prism": prism, "pos": [0.0, 0.0, 4.17]}]}},
        {"id": F02, "part": PART, "kind": "lip", "check_class": "lip", "count": 1, "purpose": "舌片（反例）",
         "geom": {"frame": "export_local", "count_kind": "instances", "count": 1,
                  "instances": [{"shape": "box", "axis": "z", "pos": [12.5, 0.0, 2.0],
                                 "bbox_mm": [[10.0, -10.0, 0.0], [15.0, 10.0, 4.0]]}]}},
        {"id": F03, "part": PART, "kind": "lip", "check_class": "lip", "count": 1, "purpose": "只有代表点的舌片（反例）",
         "geom": {"frame": "export_local", "shape": "curved_shell", "pos": [0.0, 0.0, 2.0]}},
    ]


_BASE = None


def _data(zones):
    """真判据数据（printability 分级/阈值/采样步长）只读一次，每个场景深拷贝后只换本件清单与 feature_zones。"""
    global _BASE
    part = {"id": PART, "inventory_id": f"{PART}_neg_snapzone", "material": "PETG",
            "print_orientation": "底板朝下", "print_orientation_down_normal": [0, 0, -1],
            "print_orientation_frame": "export_local", "mirrored_copy": None}
    if _BASE is None:
        _BASE = base_data([part], _features())
    d = copy.deepcopy(_BASE)
    if zones is not None:
        d["printability"].setdefault("no_support_zones", {})["feature_zones"] = copy.deepcopy(zones)
    return d


def _observe(zones, line, tag):
    import l1_printable
    stl = export_stl(_mesh(), f"l1_snapzone_{tag}")
    data, landing = landing_stub(_data(zones), PART, stl, [line])
    res = l1_printable.run(FakeCtx(data, {PART: stl}))
    return (findings(res, subject=PART, check="support_in_no_support_zone"),
            findings(res, subject=PART, check="support_in_removable_zone"), landing)


def _z(fid, tier, margin=MARGIN, **kw):
    e = {"tier": tier, "why": "反例", "src": "反例"}
    if margin is not None:
        e["face_margin_mm"] = margin
    e.update(kw)
    return {fid: e}


def _unk(fs, token=None):
    return bool(fs) and fs[0].state == "FAIL" and fs[0].measured is None and (token is None or token in (fs[0].detail or ""))


def run() -> dict:
    both = {**_z(F01, "forbid"), **_z(F02, "removable", post_process="反例：拆撑后打磨舌片面")}
    a, _, _ = _observe(None, LINE_NEAR_BUMP, "none")
    b, b_rem, lb = _observe(both, LINE_NEAR_BUMP, "near_bump")
    c, c_rem, _ = _observe(both, LINE_FAR, "far")
    d, d_rem, _ = _observe(both, LINE_NEAR_LIP, "near_lip")
    e1, _, _ = _observe(_z(F01, "forbid", margin=None), LINE_FAR, "no_margin")
    e2, _, _ = _observe(_z(F01, "forbid", margin={"v": 0.0, "src": "反例"}), LINE_FAR, "zero_margin")
    e3, _, _ = _observe(_z(F01, "forbidden"), LINE_FAR, "bad_tier")
    f, _, _ = _observe(_z(F03, "forbid"), LINE_FAR, "no_box")
    g, _, _ = _observe(_z("B03-NOPE", "forbid"), LINE_FAR, "no_feature")

    ok_b, why_b, red_b = assert_red(b, severity="BLOCK", require_measured=True)
    ok_c, why_c = assert_green(c)
    ok_cr, why_cr = assert_green(c_rem)
    ok_d, why_d = assert_green(d)
    ok_dr, why_dr, red_dr = assert_red(d_rem, severity="WARN", require_measured=True)
    assertions = [
        {"claim": "A 未纳入 → support_in_no_support_zone unknown（空清单不是通过）", "ok": _unk(a)},
        {"claim": "B 凸点顶 0.2 mm 的支撑 → FAIL(BLOCK) 带落点", "ok": ok_b},
        {"claim": "B 桩确实在 forbid 区数到点", "ok": int(lb.get("hits_forbid_samples") or 0) > 0},
        {"claim": "C 远离 → forbid PASS", "ok": ok_c},
        {"claim": "C 远离 → removable PASS", "ok": ok_cr},
        {"claim": "D 舌片外 0.2 mm → forbid PASS", "ok": ok_d},
        {"claim": "D 舌片外 0.2 mm → removable FAIL(WARN) 带落点", "ok": ok_dr},
        {"claim": "D removable detail 带 feature_zones 的 post_process", "ok": bool(d_rem and "拆撑后打磨舌片面" in (d_rem[0].detail or ""))},
        {"claim": "E1 缺 face_margin_mm → unknown", "ok": _unk(e1, "face_margin_mm")},
        {"claim": "E2 face_margin_mm=0 → unknown（0 外扩对实体特征恒 0 点 = 假绿）", "ok": _unk(e2, "face_margin_mm")},
        {"claim": "E3 tier 写错 → unknown", "ok": _unk(e3, "tier")},
        {"claim": "F 无 bbox/prism 的特征 → unknown（定位不到）", "ok": _unk(f, "定位不到")},
        {"claim": "G 不存在的特征 id → unknown", "ok": _unk(g, "B03-NOPE")},
    ]
    return {**result(NAME, EXPECT, all(x["ok"] for x in assertions),
                     got="；".join(f"{'✓' if x['ok'] else '✗'}{x['claim']}" for x in assertions),
                     red=red_b, expect_severity="BLOCK",
                     detail=(f"B={[record(x) for x in b]} {why_b}｜C={why_c}{why_cr}｜D={[record(x) for x in d + d_rem]} {why_d}{why_dr}"
                             f"｜A={[record(x) for x in a]}｜E={[record(x) for x in e1 + e2 + e3]}｜F={[record(x) for x in f]}"
                             f"｜G={[record(x) for x in g]}｜B.detail={(b[0].detail if b else '')[:200]}")),
            "assertions": assertions}


if __name__ == "__main__":
    import json
    r = run()
    print(json.dumps(r, ensure_ascii=False, indent=1))
    sys.exit(0 if r["passed"] else 1)
