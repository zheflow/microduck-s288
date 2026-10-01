#!/usr/bin/env python3
"""反例 L1（09-13 C-1）—— 禁撑区两级 + 记录绑定：removable 级落点只 WARN、主判据不红；声明变了没重切必须 unknown。

一块 50×20×10 平板，声明一个 Ø2.4 螺孔（kind=screw_hole，removable 级）和一个 Ø8 轴承孔（bearing_bore，forbid 级）。
支撑挤出线只放在螺孔里（z=5，孔内），由 _harness.landing_stub 合成 G-code 走 slice_l1.support_landing 真算：
  A  → support_in_removable_zone FAIL(WARN)，measured = 路径 mm，detail 带 printability.yaml 的 post_process 文本（Ø2.4 钻头通孔）；
       support_in_no_support_zone PASS（forbid 级 0 点，有证据）—— **可拆级不许把主判据判红**
  B  同一条记录，建桩之后把该螺孔 per_feature_overrides 改成 forbid（= 禁撑区声明变了）→ zones_sha256 ≠ 现算
       → 两条判据都 unknown（FAIL、measured=None，detail 提到 zones_sha256 / 重跑 slice_l1.py），**不许拿旧记录按新声明判**
  C  改完声明再重建桩（= 重切）→ 覆盖生效：support_in_no_support_zone FAIL(BLOCK)
  D  只有切片记录没有 support_landing → unknown（提示重跑 slice_l1.py）
  E  from_features_yaml_kinds 少一类（旧读法与两级并集不一致）→ unknown（声明问题），旧读法不许悄悄失效
  F  记录的采样步长 ≠ criteria 的 → unknown
红（契约 red）取 A 的 WARN；B/D/E/F 的 unknown 与 C 的 BLOCK 用 assertions 断言。
"""
from __future__ import annotations

import copy
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from _harness import (FakeCtx, base_data, box, export_stl, findings, assert_red, assert_green,   # noqa: E402
                      result, landing_stub)

PART = "H03"
F_SCREW, F_BORE = "H03-NEG-SCREW", "H03-NEG-BORE"
NAME = "禁撑区两级：removable 落点只 WARN 且主判据绿；声明改了没重切 / 无落点记录 / 步长不符 / 旧读法不一致 → unknown"
EXPECT = (f"L1/{PART}:support_in_removable_zone FAIL(WARN)（螺孔里的支撑）+ support_in_no_support_zone PASS；"
          "zones_sha256 不一致 → 两条 unknown；重切后覆盖为 forbid → FAIL(BLOCK)")
CHECK, CHECK_R = "support_in_no_support_zone", "support_in_removable_zone"

PLATE = (50.0, 20.0, 10.0)
SCREW_X, BORE_X = -10.0, 15.0
LINE_IN_SCREW = ((SCREW_X, -0.8, 5.0), (SCREW_X, 0.8, 5.0))


def _feat(fid, kind, x, d):
    return {"id": fid, "part": PART, "kind": kind, "check_class": kind, "count": 1,
            "purpose": f"反例 {kind}", "spec_verbatim": f"Ø{d} 通孔",
            "geom": {"shape": "cylinder", "frame": "export_local", "axis": "z",
                     "nominal_d_mm": {"v": d, "src": "measured"}, "depth_mm": {"v": PLATE[2], "src": "assumed"},
                     "depth_kind": "cutter_length", "through": True, "pos": [x, 0.0, PLATE[2] / 2],
                     "axial_span_mm": [[x, 0.0, 0.0], [x, 0.0, PLATE[2]]]}}


def _data():
    part = {"id": PART, "inventory_id": f"{PART}_neg_tiers", "material": "PLA",
            "print_orientation": "平板朝下", "print_orientation_down_normal": [0, 0, -1],
            "print_orientation_frame": "export_local", "mirrored_copy": None}
    d = base_data([part], [_feat(F_SCREW, "screw_hole", SCREW_X, 2.4), _feat(F_BORE, "bearing_bore", BORE_X, 8.0)])
    d["printability"] = copy.deepcopy(d["printability"])          # 下面要改声明，别动 load_data 的共享字典
    return d


def _override_forbid(data):
    data["printability"]["no_support_zones"].setdefault("per_feature_overrides", {})[F_SCREW] = \
        {"tier": "forbid", "post_process": None, "why": "反例：把螺孔改成 forbid", "src": "本反例"}


def _mesh_stl():
    m = box(*PLATE, center=(0.0, 0.0, PLATE[2] / 2))
    m = m.difference(box(2.4, 2.4, 30.0, center=(SCREW_X, 0.0, 5.0))).difference(box(8.0, 8.0, 30.0, center=(BORE_X, 0.0, 5.0)))
    return export_stl(m, "l1_tiers_plate")


def _run(data, stl):
    import l1_printable
    res = l1_printable.run(FakeCtx(data, {PART: stl}))
    return findings(res, subject=PART, check=CHECK), findings(res, subject=PART, check=CHECK_R)


def _is_unknown(fs, *needles):
    return bool(fs) and fs[0].state == "FAIL" and fs[0].measured is None and any(n in (fs[0].detail or "") for n in needles)


def run() -> dict:
    stl = _mesh_stl()
    # A：螺孔里有支撑 → removable WARN，主判据绿
    dA, lA = landing_stub(_data(), PART, stl, [LINE_IN_SCREW])
    mainA, remA = _run(dA, stl)
    ok_green, why_green = assert_green(mainA)
    ok_warn, why_warn, red = assert_red(remA, severity="WARN", require_measured=True)
    ok_pp = bool(remA) and "Ø2.4 钻头" in (remA[0].detail or "")
    # B：建桩后改声明（override 为 forbid）→ sha 不一致 → unknown
    dB, _ = landing_stub(_data(), PART, stl, [LINE_IN_SCREW], mutate_zone_data=_override_forbid)
    mainB, remB = _run(dB, stl)
    ok_B = _is_unknown(mainB, "zones_sha256", "没重切") and _is_unknown(remB, "zones_sha256", "没重切")
    # C：改完再重建桩（重切）→ 覆盖生效 → BLOCK
    dC = _data(); _override_forbid(dC)
    dC, lC = landing_stub(dC, PART, stl, [LINE_IN_SCREW])
    mainC, _ = _run(dC, stl)
    ok_C, why_C, _ = assert_red(mainC, severity="BLOCK", require_measured=True)
    # D：有切片记录、没 support_landing → unknown
    dD, _ = landing_stub(_data(), PART, stl, [LINE_IN_SCREW])
    dD["printability"]["slice_run"]["parts"][PART].pop("support_landing")
    mainD, _ = _run(dD, stl)
    ok_D = _is_unknown(mainD, "support_landing", "slice_l1.py")
    # E：from_features_yaml_kinds 与两级并集不一致 → unknown
    dE, _ = landing_stub(_data(), PART, stl, [LINE_IN_SCREW])
    kinds = dE["printability"]["no_support_zones"]["from_features_yaml_kinds"]
    dE["printability"]["no_support_zones"]["from_features_yaml_kinds"] = [k for k in kinds if k != "journal"]
    mainE, _ = _run(dE, stl)
    ok_E = _is_unknown(mainE, "from_features_yaml_kinds")
    # F：记录步长 ≠ criteria → unknown
    dF, _ = landing_stub(_data(), PART, stl, [LINE_IN_SCREW])
    dF["printability"]["slice_run"]["parts"][PART]["support_landing"]["sample_step_mm"] = 0.1
    mainF, _ = _run(dF, stl)
    ok_F = _is_unknown(mainF, "步长")

    assertions = [
        {"claim": "A 螺孔支撑 → support_in_no_support_zone PASS（forbid 0 点，有证据）", "ok": ok_green},
        {"claim": "A 螺孔支撑 → support_in_removable_zone FAIL(WARN) measured 非空", "ok": ok_warn},
        {"claim": "A removable 的 detail 带后处理文本『Ø2.4 钻头』", "ok": ok_pp},
        {"claim": "B 声明改了没重切（zones_sha256 不一致）→ 两条判据 unknown", "ok": ok_B},
        {"claim": "C 重切后 per_feature_overrides=forbid 生效 → FAIL(BLOCK)", "ok": ok_C},
        {"claim": "D 无 support_landing 记录 → unknown 并提示重跑 slice_l1.py", "ok": ok_D},
        {"claim": "E from_features_yaml_kinds ≠ 两级并集 → unknown", "ok": ok_E},
        {"claim": "F 记录采样步长 ≠ criteria → unknown", "ok": ok_F},
    ]
    return {**result(NAME, EXPECT, all(a["ok"] for a in assertions),
                     got=(f"A 主{'绿' if ok_green else why_green} 可拆{'WARN' if ok_warn else why_warn}"
                          f"(={remA[0].measured if remA else None} mm，桩 removable {lA['hits_removable_samples']} 点)；"
                          f"B {'unknown' if ok_B else '未 unknown'}；C {'BLOCK' if ok_C else why_C}；"
                          f"D {'unknown' if ok_D else '未 unknown'}；E {'unknown' if ok_E else '未 unknown'}；F {'unknown' if ok_F else '未 unknown'}"),
                     red=red, expect_severity="WARN",
                     detail=(f"A 可拆：{(remA[0].detail if remA else '')[:160]}｜B：{(mainB[0].detail if mainB else '')[:120]}"
                             f"｜C：{(mainC[0].detail if mainC else '')[:100]}")),
            "assertions": assertions}


if __name__ == "__main__":
    import json
    r = run()
    print(json.dumps(r, ensure_ascii=False, indent=1))
    sys.exit(0 if r["passed"] else 1)
