#!/usr/bin/env python3
"""反例 —— 双向镜像检查不能用有限的一向掩盖另一向的 NaN，或把空采样当成零距离。

经过 `l2_features.run(ctx)` 的镜像分支（parts.yaml:mirrored_copy → placed_pair → mirror_surface_dev）：
  好    左右件互为真镜像 → mirror:surface PASS（双向距离 ≈ 0）
  坏 A  第二向的最近表面距离返回 NaN（猴子补丁 trimesh.proximity.ProximityQuery）→ mirror_surface_dev 抛
        ValueError → 层判 mirror:surface **unknown**（FAIL(BLOCK)、measured=None）
  坏 B  同上但返回 Inf
  坏 C  parts.yaml:mirror_asymmetry_allowed 的盒把整件都排除 → 某向没有剩余采样点 → unknown
坏样本的红是 unknown（算不出来），这是本反例要验的设计 → allow_unknown_red。
注意：这里补丁的是 ProximityQuery 的 on_surface，第一次调用给 0、第二次给非有限值 ——
如果层只看 max(有限, NaN)=有限 就会假绿，正是本反例要抓的。
"""
from __future__ import annotations
import json
import sys
from pathlib import Path
from unittest.mock import patch

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from _harness import FakeCtx, base_data, box, cyl, export_stl, findings, assert_red, assert_green, result, record  # noqa: E402

PART = "L03"
NAME = "镜像单向 NaN/Inf 及空采样必须 unknown（mirror:surface），真镜像双向距离仍绿"
EXPECT = f"L2/{PART}:mirror:surface 坏样本 FAIL(BLOCK) 且 measured=None（unknown）；真镜像 PASS"
CHECK = "mirror:surface"


def _plate(hole_y):
    return box(20, 20, 4).difference(cyl(2.0, 20.0, center=(5.0, hole_y, 0.0)))


def _data(allow=None):
    part = {"id": PART, "inventory_id": "L03_nfm", "material": "PLA",
            "print_orientation": "背板朝下", "mirrored_copy": "L03_R（mirror_y）"}
    if allow is not None:
        part["mirror_asymmetry_allowed"] = allow
    return base_data([part], [])


def _ctx(allow=None):
    pl = export_stl(_plate(+5.0), "nfm_L")
    pr = export_stl(_plate(-5.0), "nfm_R")
    return FakeCtx(_data(allow), {PART: pl}, placed_map={"nfm": pl, "nfm_R": pr})


def _run(ctx):
    import l2_features
    l2_features._GEOM_CACHE.clear()
    res = l2_features.run(ctx)
    return findings(res, subject=PART, check=CHECK)


def _run_nonfinite(bad_value):
    import trimesh
    calls = []
    real = trimesh.proximity.ProximityQuery

    class _Q(real):                                              # 第一向正常、第二向非有限
        def on_surface(self, pts):
            calls.append(1)
            n = len(pts)
            d = np.zeros(n) if len(calls) == 1 else np.full(n, bad_value)
            return np.asarray(pts), d, np.zeros(n, dtype=int)

    with patch("trimesh.proximity.ProximityQuery", _Q):
        return _run(_ctx())


def run() -> dict:
    good = _run(_ctx())
    ok_green, why_green = assert_green(good)
    reds, got, ok_all = [], [], True
    for tag, fs in (("NaN", _run_nonfinite(float("nan"))), ("Inf", _run_nonfinite(float("inf"))),
                    ("空采样", _run(_ctx(allow=[{"bbox_mm": [[-10, -10, -10], [10, 10, 10]], "why": "negative"}])))):
        ok, why, rec = assert_red(fs, severity="BLOCK", require_measured=False)
        is_unknown = ok and all(r["measured"] is None for r in rec)
        ok_all &= is_unknown
        reds += rec
        got.append(f"{tag}:{'unknown' if is_unknown else ('红但非 unknown' if ok else why)}")
    got.append(f"真镜像:{'绿' if ok_green else why_green}(measured={good[0].measured if good else None})")
    return result(NAME, EXPECT, ok_all and ok_green, got="；".join(got), red=reds, expect_severity="BLOCK",
                  allow_unknown_red="NaN/Inf/空采样时 mirror_surface_dev 抛 ValueError、层判 unknown（measured=None）是设计：不能让 max(有限值, NaN) 隐藏计算失败",
                  detail=f"坏样本记录={reds}；真镜像={[record(f) for f in good]}")


if __name__ == "__main__":
    print(json.dumps(run(), ensure_ascii=False, indent=1))
