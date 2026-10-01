#!/usr/bin/env python3
"""反例 L1-C'（09-12 第二批，09-13 C-4 改壳层）—— 滑配走廊：支撑落在**配合面壳层**里必须红（forbid），
落在口袋中央只 WARN（removable），坏盒 / 坏壳层必须 unknown。

起因：L01-F13 / L03-F05 是舵机装入走廊（Minkowski 扫掠体），第二批用整个口袋盒当禁撑区 → 口袋里永远有支撑、永远红且没信息量。
09-13 C-4：盒仍声明（geom.box_export_local，按 kind=slide_corridor 归 removable：口袋里的支撑拆掉就行），
配合面另按 geom.mating_shells_export_local 逐面声明 0.5 mm 壳层（一律 forbid）。本反例保证：
  · 坏：支撑挤出线贴着 +x 壁（落在壳层里）→ support_in_no_support_zone FAIL(BLOCK)，measured = 路径 mm；
        同一点**不**再记进 removable（zone_hits 把壳层从口袋盒里扣掉）
  · 中央：支撑线在口袋中央（不在任何壳层）→ support_in_no_support_zone PASS（有证据），
        support_in_removable_zone FAIL(WARN) 且 detail 带后处理文本（printability.yaml:…post_process.slide_corridor）
  · 坏声明：盒 hi ≤ lo → 定位不到 → unknown（FAIL、measured=None，不能静默当没有禁撑区）
  · 坏壳层：壳层 hi ≤ lo → 『写坏』→ unknown
支撑位置由 _harness.landing_stub 合成 G-code 给出，走 slice_l1.support_landing 的真实解析/采样/逆变换。
几何与 n_l1_support_in_bore 共用同一块底板 + 立柱 + 悬臂（没有孔），只有盒/壳层/支撑线在动。
"""
from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from _harness import (FakeCtx, base_data, box, export_stl, findings, assert_red, assert_green,   # noqa: E402
                      result, record, landing_stub)

PART = "L01"
FID = "L01-NEG-CORRIDOR"
NAME = "滑配走廊：支撑落在配合面壳层里必须红(BLOCK)，落在口袋中央只 WARN(removable)，坏盒/坏壳层必须 unknown"
EXPECT = (f"L1/{PART}:support_in_no_support_zone FAIL(BLOCK)（支撑线落在 {FID} 的 +x 壁壳层）；"
          "中央支撑 → 主判据 PASS + support_in_removable_zone FAIL(WARN)；倒置盒 / 倒置壳层 unknown")

PLATE = (50.0, 20.0, 10.0)
PLATE_CX = 5.0
PILLAR_X = -17.5
CANTI_X0, CANTI_X1 = -20.0, 15.0
BOX_LO, BOX_HI = [-5.0, -8.0, 10.0], [10.0, 8.0, 20.0]          # 口袋盒：悬臂下方 x −5..10
SHELL = {"face": "+x 壁（反例）", "role": "滑配导向壁", "lo": [9.5, -8.0, 10.0], "hi": [10.0, 8.0, 20.0]}   # 0.5 厚壳层贴 +x 壁
LINE_IN_SHELL = ((9.75, -3.0, 15.0), (9.75, 3.0, 15.0))         # 贴壁：全在壳层里
LINE_CENTER = ((0.0, 0.0, 15.0), (5.0, 0.0, 15.0))              # 口袋中央：在盒里、不在壳层


def _mesh():
    plate = box(*PLATE, center=(PLATE_CX, 0.0, PLATE[2] / 2))
    pillar = box(5.0, 20.0, 10.0, center=(PILLAR_X, 0.0, 15.0))
    canti = box(CANTI_X1 - CANTI_X0, 20.0, 2.0, center=((CANTI_X0 + CANTI_X1) / 2, 0.0, 21.0))
    return plate.union(pillar).union(canti)


def _data(lo, hi, shells):
    part = {"id": PART, "inventory_id": f"{PART}_neg_corridor", "material": "PLA",
            "print_orientation": "底板朝下", "print_orientation_down_normal": [0, 0, -1],
            "print_orientation_frame": "export_local", "mirrored_copy": None}
    geom = {"shape": "sweep", "frame": "export_local", "axis": "z",
            "box_export_local": {"lo": lo, "hi": hi}}
    if shells is not None:
        geom["mating_shells_export_local"] = shells
    feat = {"id": FID, "part": PART, "kind": "slide_corridor", "check_class": "slide_corridor",
            "count": 1, "purpose": "舵机装入走廊（反例）", "spec_verbatim": "盒形走廊 + 配合面壳层", "geom": geom}
    return base_data([part], [feat])


def _observe(lo, hi, shells, line, tag):
    import l1_printable
    stl = export_stl(_mesh(), f"l1_corridor_{tag}")
    data, landing = landing_stub(_data(lo, hi, shells), PART, stl, [line])
    res = l1_printable.run(FakeCtx(data, {PART: stl}))
    return (findings(res, subject=PART, check="support_in_no_support_zone"),
            findings(res, subject=PART, check="support_in_removable_zone"), landing)


def run() -> dict:
    bad, bad_rem, l_bad = _observe(BOX_LO, BOX_HI, [SHELL], LINE_IN_SHELL, "in_shell")
    mid, mid_rem, l_mid = _observe(BOX_LO, BOX_HI, [SHELL], LINE_CENTER, "center")
    inv, _, _ = _observe(BOX_HI, BOX_LO, [SHELL], LINE_CENTER, "inverted_box")
    bad_shell = dict(SHELL, lo=SHELL["hi"], hi=SHELL["lo"])
    inv_sh, _, _ = _observe(BOX_LO, BOX_HI, [bad_shell], LINE_CENTER, "inverted_shell")

    ok_red, why_red, red = assert_red(bad, severity="BLOCK", require_measured=True)
    # 壳层里的点不许再记进 removable（同一点不同时算两级）
    ok_not_double = bool(bad_rem) and bad_rem[0].state == "PASS" and l_bad["hits_removable_samples"] == 0
    ok_green, why_green = assert_green(mid)
    ok_warn, why_warn, red_warn = assert_red(mid_rem, severity="WARN", require_measured=True)
    ok_pp = bool(mid_rem) and "拆掉" in (mid_rem[0].detail or "")           # 后处理文本进 detail
    unk_d = inv[0].detail if inv else ""
    ok_unk = bool(inv) and inv[0].state == "FAIL" and inv[0].measured is None and ("定位不到" in unk_d)
    unk_sd = inv_sh[0].detail if inv_sh else ""
    ok_unk_sh = bool(inv_sh) and inv_sh[0].state == "FAIL" and inv_sh[0].measured is None and ("写坏" in unk_sd or "定位不到" in unk_sd)
    assertions = [
        {"claim": "壳层里的支撑 → support_in_no_support_zone FAIL(BLOCK)", "ok": ok_red},
        {"claim": "壳层里的点不再记进 removable（support_in_removable_zone PASS，桩 removable 点数 0）", "ok": ok_not_double},
        {"claim": "口袋中央支撑 → 主判据 PASS（有证据）", "ok": ok_green},
        {"claim": "口袋中央支撑 → support_in_removable_zone FAIL(WARN) 且 measured 非空", "ok": ok_warn},
        {"claim": "removable 的 detail 带 slide_corridor 后处理文本", "ok": ok_pp},
        {"claim": "倒置盒 → unknown（定位不到）", "ok": ok_unk},
        {"claim": "倒置壳层 → unknown（写坏）", "ok": ok_unk_sh},
    ]
    return {**result(NAME, EXPECT, all(a["ok"] for a in assertions),
                     got=(f"贴壁 {'红' if ok_red else why_red}(壳层落点={bad[0].measured if bad else None} mm) / "
                          f"中央 主{'绿' if ok_green else why_green} 可拆{'WARN' if ok_warn else why_warn}"
                          f"(={mid_rem[0].measured if mid_rem else None} mm) / 倒置盒 {'unknown' if ok_unk else '未 unknown'} / "
                          f"倒置壳层 {'unknown' if ok_unk_sh else '未 unknown'}"),
                     red=red, expect_severity="BLOCK",
                     detail=(f"坏：{(bad[0].detail if bad else '')[:150]}｜中央可拆：{(mid_rem[0].detail if mid_rem else '')[:150]}"
                             f"｜坏声明：{[record(f) for f in inv]}｜坏壳层：{unk_sd[:80]}")),
            "assertions": assertions}


if __name__ == "__main__":
    import json
    r = run()
    print(json.dumps(r, ensure_ascii=False, indent=1))
    sys.exit(0 if r["passed"] else 1)
