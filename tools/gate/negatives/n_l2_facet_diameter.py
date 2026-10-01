#!/usr/bin/env python3
"""反例（09-13 F-L2-2）—— 64 边刻面的 Ø14.85 孔必须判成 Ø14.85，不是 14.846。

坑的形状：`<特征>:diameter` 用 `dm = 2 * sc["r_med"]` —— 射线从轴心打到刻面**面心**，量到的是
内接圆（r·cos(π/N)）。64 边的 Ø14.85 量成 14.846，对 tolerances 里 `target_range.min == nominal`
的桶（fits.pla.bearing_inner_hub_6702 = [14.85, 14.95]）必红 —— 记分卡 4 条 diameter 红全是这个，
红的是三角化，不是零件。bore_gauge 早已用"三角面棱边采样点到轴距离"规避刻面，diameter 却没跟。

正确判据：棱边法 —— 圆柱刻面的纵向棱和端面弦，两端点精确落在建模圆上，取它们的半径（对分段数免疫）。

  坏样本 1  Ø14.70 的 64 边孔（真缩了 0.15，超出 ring_scan 的 ±0.1 命中窗）→ present 或 diameter 红（BLOCK）
  坏样本 2  Ø14.78 的 64 边孔（缩 0.07，仍在命中窗内、但出了桶的 [14.85,14.95]）→ diameter 必须红（BLOCK，measured ≈ 14.78）
  对照      Ø14.85 的 64 边孔 → diameter 必须绿（以前这里 14.834 < 14.85 假红）
件按 L01（PLA，桶的 print_orientation 点了 L01 的名），特征照 features.yaml:L01-F02 的 geom 结构造
（frame=export_local、axis/pos/axial_span_mm、nominal_d_mm），只是 kind 改成 bearing_bore（孔，不是毂）。
"""
from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from _harness import FakeCtx, assert_green, assert_red, base_data, box, cyl, export_stl, findings, result  # noqa: E402

PART = "L01"
FID = "L01-NEG-HUB"
BUCKET = "fits.pla.bearing_inner_hub_6702"
NAME = "64 边刻面的 Ø14.85 孔：diameter 必须绿（棱边法）；真缩到 Ø14.70 必须红"
EXPECT = f"L2/{PART}:{FID}:diameter 对 {BUCKET} [14.85,14.95]：Ø14.85 PASS / Ø14.78 FAIL(BLOCK) / Ø14.70 present|diameter FAIL(BLOCK)"
D_NOM = 14.85
T = 4.5
SECTIONS = 64


def _mesh(d_real: float):
    m = box(30.0, 30.0, T, center=(0.0, 0.0, T / 2))
    return m.difference(cyl(d_real / 2.0, 3 * T, center=(0.0, 0.0, T / 2), sections=SECTIONS))


def _data():
    part = {"id": PART, "inventory_id": "L01_neg_hub", "material": "PLA",
            "print_orientation": "偏航盘朝下（打印清单）", "mirrored_copy": None}
    feat = {"id": FID, "part": PART, "kind": "bearing_bore", "check_class": "bearing_bore", "count": 1,
            "spec_verbatim": f"Ø{D_NOM} 6702 内圈毂配合（反例：孔）",
            "geom": {"shape": "cylinder", "frame": "export_local", "confidence": "exact", "count": 1,
                     "nominal_d_mm": {"v": D_NOM, "src": "assumed"},
                     "depth_mm": {"v": T, "src": "assumed"}, "depth_kind": "cutter_length",
                     "through": True, "axis": "z", "pos": [0.0, 0.0, T / 2],
                     "axial_span_mm": [[0.0, 0.0, 0.0], [0.0, 0.0, T]]}}
    return base_data([part], [feat])


def _observe(d_real: float, tag: str):
    import l2_features
    l2_features._GEOM_CACHE.clear()
    stl = export_stl(_mesh(d_real), f"l2_facet_{tag}")
    res = l2_features.run(FakeCtx(_data(), {PART: stl}))
    return (findings(res, subject=PART, check=f"{FID}:diameter"),
            findings(res, subject=PART, check=f"{FID}:present"))


def run() -> dict:
    bad1_d, bad1_p = _observe(14.70, "d1470")
    bad2_d, _ = _observe(14.78, "d1478")
    good, _ = _observe(D_NOM, "d1485")
    ok_bad1, why_bad1, red1 = assert_red(bad1_d + bad1_p, severity="BLOCK")
    ok_bad2, why_bad2, red2 = assert_red(bad2_d, severity="BLOCK")
    ok_good, why_good = assert_green(good)
    crit_ok = bool(good) and BUCKET in str(good[0].criterion) and "棱边" in str(good[0].criterion)
    checks = [("Ø14.70 → present/diameter FAIL(BLOCK, measured 非空)", ok_bad1),
              ("Ø14.78 → diameter FAIL(BLOCK, measured 非空)", ok_bad2),
              ("Ø14.85（64 边）→ diameter PASS", ok_good),
              (f"criterion 引用了 {BUCKET} 且写明棱边法", crit_ok)]
    failed = [c for c, ok in checks if not ok]
    fmt = lambda fs: f"{fs[0].state}(measured={fs[0].measured})" if fs else "ABSENT"
    got = (f"Ø14.70 样本 diameter {fmt(bad1_d)} present {fmt(bad1_p)} / Ø14.78 样本 diameter {fmt(bad2_d)} / "
           f"Ø14.85 对照 {fmt(good)}")
    return result(NAME, EXPECT, not failed, got + (f"｜未成立：{failed}" if failed else ""), red1 + red2,
                  expect_severity="BLOCK",
                  detail=" | ".join(x for x in (why_bad1, why_bad2, why_good) if x)
                         or f"对照 criterion：{str(good[0].criterion)[:160]}")


if __name__ == "__main__":
    import json
    r = run()
    print(json.dumps(r, ensure_ascii=False, indent=1))
    sys.exit(0 if r["passed"] else 1)
