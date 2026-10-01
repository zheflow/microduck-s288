#!/usr/bin/env python3
"""反例 L1（2026-09-25，用户定"按 1"：L01/L03 口袋贴合面接受拆支撑后锉平）—— 配合面壳层的按件降级开关：
  · 默认（没有 per_feature_overrides.<fid>.mating_shells_tier）：支撑落在壳层里仍 support_in_no_support_zone FAIL(BLOCK)（与 09-13 C-4 逐字相同）
  · 声明 mating_shells_tier: removable：同一条支撑线 → 主判据 PASS（有证据）、support_in_removable_zone FAIL(WARN)，
    detail 带 mating_shells_post_process 的文本（后处理进打印清单）
  · 声明写坏（mating_shells_tier: xyz）→ problems → unknown（FAIL、measured=None），不能静默当默认
几何 / 桩与 n_l1_support_in_box_zone 相同（底板 + 立柱 + 悬臂，口袋盒 + 一层 +x 壁壳层，支撑线贴壁）。
"""
from __future__ import annotations

import copy
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from _harness import (FakeCtx, base_data, box, export_stl, findings, assert_red, assert_green,   # noqa: E402
                      result, record, landing_stub)

PART = "L01"
FID = "L01-NEG-CORRIDOR-OVR"
NAME = "配合面壳层按件降级：默认仍红(BLOCK)；mating_shells_tier=removable → 主判据绿 + 可拆 WARN 带后处理文本；写坏 → unknown"
EXPECT = (f"L1/{PART}:support_in_no_support_zone 默认 FAIL(BLOCK)；override removable 时 PASS 且 support_in_removable_zone FAIL(WARN)；"
          "mating_shells_tier=xyz → unknown")
PP_TEXT = "反例专用后处理：拆支撑后平锉刮平壳层疤"

PLATE = (50.0, 20.0, 10.0)
PLATE_CX = 5.0
PILLAR_X = -17.5
CANTI_X0, CANTI_X1 = -20.0, 15.0
BOX_LO, BOX_HI = [-5.0, -8.0, 10.0], [10.0, 8.0, 20.0]
SHELL = {"face": "+x 壁（反例）", "role": "滑配导向壁", "lo": [9.5, -8.0, 10.0], "hi": [10.0, 8.0, 20.0]}
LINE_IN_SHELL = ((9.75, -3.0, 15.0), (9.75, 3.0, 15.0))


def _mesh():
    plate = box(*PLATE, center=(PLATE_CX, 0.0, PLATE[2] / 2))
    pillar = box(5.0, 20.0, 10.0, center=(PILLAR_X, 0.0, 15.0))
    canti = box(CANTI_X1 - CANTI_X0, 20.0, 2.0, center=((CANTI_X0 + CANTI_X1) / 2, 0.0, 21.0))
    return plate.union(pillar).union(canti)


def _data(override):
    part = {"id": PART, "inventory_id": f"{PART}_neg_corridor_ovr", "material": "PLA",
            "print_orientation": "底板朝下", "print_orientation_down_normal": [0, 0, -1],
            "print_orientation_frame": "export_local", "mirrored_copy": None}
    geom = {"shape": "sweep", "frame": "export_local", "axis": "z",
            "box_export_local": {"lo": BOX_LO, "hi": BOX_HI}, "mating_shells_export_local": [SHELL]}
    feat = {"id": FID, "part": PART, "kind": "slide_corridor", "check_class": "slide_corridor",
            "count": 1, "purpose": "舵机装入走廊（反例，壳层降级）", "spec_verbatim": "盒形走廊 + 配合面壳层", "geom": geom}
    data = base_data([part], [feat])
    data["printability"] = copy.deepcopy(data["printability"])
    nsz = data["printability"].setdefault("no_support_zones", {})
    ov = dict(nsz.get("per_feature_overrides") or {})
    ov.pop(FID, None)
    if override is not None:
        ov[FID] = override
    nsz["per_feature_overrides"] = ov
    return data


def _observe(override, tag):
    import l1_printable
    stl = export_stl(_mesh(), f"l1_corridor_ovr_{tag}")
    data, landing = landing_stub(_data(override), PART, stl, [LINE_IN_SHELL])
    res = l1_printable.run(FakeCtx(data, {PART: stl}))
    return (findings(res, subject=PART, check="support_in_no_support_zone"),
            findings(res, subject=PART, check="support_in_removable_zone"), landing)


def run() -> dict:
    base_f, base_rem, l_base = _observe(None, "default")
    ovr = {"tier": "removable", "mating_shells_tier": "removable", "mating_shells_post_process": PP_TEXT,
           "why": "反例：用户 2026-09-25 定口袋贴合面接受后处理", "src": "n_l1_mating_shell_override"}
    ovr_f, ovr_rem, l_ovr = _observe(ovr, "override")
    bad = dict(ovr, mating_shells_tier="xyz")
    bad_f, _, _ = _observe(bad, "bad_value")

    ok_red, why_red, red = assert_red(base_f, severity="BLOCK", require_measured=True)
    ok_base_forbid = l_base["hits_forbid_samples"] > 0 and l_base["hits_removable_samples"] == 0
    ok_green, why_green = assert_green(ovr_f)
    ok_warn, why_warn, _ = assert_red(ovr_rem, severity="WARN", require_measured=True)
    ok_moved = l_ovr["hits_forbid_samples"] == 0 and l_ovr["hits_removable_samples"] == l_base["hits_forbid_samples"]
    ok_pp = bool(ovr_rem) and PP_TEXT in (ovr_rem[0].detail or "")
    bad_d = bad_f[0].detail if bad_f else ""
    ok_unk = bool(bad_f) and bad_f[0].state == "FAIL" and bad_f[0].measured is None and "mating_shells_tier" in bad_d
    assertions = [
        {"claim": "默认（无 override）壳层支撑 → support_in_no_support_zone FAIL(BLOCK)（09-13 行为不变）", "ok": ok_red},
        {"claim": "默认：落点全在 forbid、removable 为 0", "ok": ok_base_forbid},
        {"claim": "mating_shells_tier=removable → 主判据 PASS（有证据）", "ok": ok_green},
        {"claim": "mating_shells_tier=removable → support_in_removable_zone FAIL(WARN) 且 measured 非空", "ok": ok_warn},
        {"claim": "同一条线的落点从 forbid 整体挪到 removable（点数相等）", "ok": ok_moved},
        {"claim": "removable 的 detail 带 mating_shells_post_process 文本", "ok": ok_pp},
        {"claim": "mating_shells_tier 写坏 → unknown（FAIL、measured=None、detail 点名字段）", "ok": ok_unk},
    ]
    return {**result(NAME, EXPECT, all(a["ok"] for a in assertions),
                     got=(f"默认 {'红' if ok_red else why_red}(forbid={l_base['hits_forbid_samples']}) / override 主{'绿' if ok_green else why_green} "
                          f"可拆{'WARN' if ok_warn else why_warn}(removable={l_ovr['hits_removable_samples']}) / 写坏 {'unknown' if ok_unk else '未 unknown'}"),
                     red=red, expect_severity="BLOCK",
                     detail=(f"默认：{(base_f[0].detail if base_f else '')[:120]}｜override 可拆：{(ovr_rem[0].detail if ovr_rem else '')[:160]}"
                             f"｜写坏：{bad_d[:120]}｜{[record(f) for f in bad_f]}")),
            "assertions": assertions}


if __name__ == "__main__":
    import json
    r = run()
    print(json.dumps(r, ensure_ascii=False, indent=1))
    sys.exit(0 if r["passed"] else 1)
