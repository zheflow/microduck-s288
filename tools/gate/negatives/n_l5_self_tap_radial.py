#!/usr/bin/env python3
"""反例 —— 底孔大于或等于螺丝主径时，即使长度足够，也不能自攻成牙。

经过 `l5_function.run(ctx)`（不是只调 check_self_tap_radial）：fasteners 清单里放四组 M2 自攻，
只有底孔径不同。红必须来自 run 发出的 finding：
  · Ø2.2（大孔）  → self_tap_radial_overlap FAIL(BLOCK)，measured = (2−2.2)/2 = −0.1
  · Ø2.0（等径）  → FAIL(BLOCK)，measured = 0
  · Ø1.6（小孔）  → PASS（只证明径向必要条件）
  · 缺底孔径      → unknown（FAIL、measured=None）—— 单独记录，不算进 red
判据来自层：径向重叠 (主径−底孔)/2 > 0，本文件不写死阈值。
"""
from __future__ import annotations
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from _harness import FakeCtx, base_data, findings, assert_red, assert_green, result, record  # noqa: E402

NAME = "M2 自攻遇 Ø2.2/Ø2.0 底孔必须红（self_tap_radial_overlap）；Ø1.6 绿；缺数据 unknown"
EXPECT = "L5/<组>:self_tap_radial_overlap FAIL(BLOCK) 且 measured ≤ 0；Ø1.6 组 PASS"
CHECK = "self_tap_radial_overlap"


def _fa(gid, d):
    pilot = {"pilot_hole_d_mm": {"v": d, "src": "assumed"}} if d is not None else {}
    return {"id": gid, "spec": "M2×8", "qty": 1, "joint_type": "pla_self_tap",
            "joins": ["反例：PLA 自攻"], "status": "ok", "pilot": pilot}


def run() -> dict:
    import l5_function
    fas = [_fa("NG_bigpilot", 2.2), _fa("NG_equal", 2.0), _fa("NG_small", 1.6), _fa("NG_nodata", None)]
    res = l5_function.run(FakeCtx(base_data([], [], fasteners=fas, relations=[]), {}))
    ok_big, why_big, red_big = assert_red(findings(res, subject="NG_bigpilot", check=CHECK), severity="BLOCK")
    ok_eq, why_eq, red_eq = assert_red(findings(res, subject="NG_equal", check=CHECK), severity="BLOCK")
    ok_small, why_small = assert_green(findings(res, subject="NG_small", check=CHECK))
    nodata = findings(res, subject="NG_nodata", check=CHECK)
    ok_nodata = bool(nodata) and nodata[0].state == "FAIL" and nodata[0].measured is None
    # 径向重叠必须真的是负/零，不是碰巧红在别的原因
    vals_ok = all(float(x["measured"]) <= 0 for x in red_big + red_eq)
    passed = ok_big and ok_eq and ok_small and ok_nodata and vals_ok
    return result(NAME, EXPECT, passed,
                  got=(f"Ø2.2 {'红' if ok_big else '未红:' + why_big}（{[x['measured'] for x in red_big]}）；"
                       f"Ø2.0 {'红' if ok_eq else '未红:' + why_eq}（{[x['measured'] for x in red_eq]}）；"
                       f"Ø1.6 {'绿' if ok_small else '不绿:' + why_small}；"
                       f"缺数据 {'unknown' if ok_nodata else '未 unknown'}"),
                  red=red_big + red_eq, expect_severity="BLOCK",
                  detail=f"nodata={[record(f) for f in nodata]}")


if __name__ == "__main__":
    print(json.dumps(run(), ensure_ascii=False, indent=1))
