#!/usr/bin/env python3
"""反例（09-13 F-L5-5）—— 自攻底孔盲深先读 `fasteners.yaml:<组>.pilot.pilot_depth_mm`，再退回 joins 自由文本。

坑的形状：`l5_function._blind_depth` 只会从 joins/location 的文字里用正则抠 "Ø1.6 底孔 4.5"；fasteners.yaml
早有结构化的 `pilot.pilot_depth_mm: {v, src}`，层不读 —— 文本改了数不改、数改了文本不改，都没人知道。

正确判据：有 `pilot.pilot_depth_mm` 就用它（provenance 写 pilot.pilot_depth_mm）；取不到再退回 joins 正则；
两者都有且不一致 → unknown，并把两个数写进 detail（不许挑一个用）。

  A 只有 pilot_depth_mm=3.0（joins 无深度文字）  叠 4.0 + 3.0 − 0.3 = 6.7 < M2×8 → screw_length_rule FAIL(BLOCK)，detail 记 pilot.pilot_depth_mm
  B 只有 joins 文字 4.5                          4.0 + 4.5 − 0.3 = 8.2 ≥ 8 → PASS（对照：盲深来源不同，结论随之变）
  C 两者都有且不一致（3.0 vs 4.5）               unknown，detail 同时含 3.0 与 4.5
  D 两者一致（4.5 / 4.5）                        PASS，detail 记 pilot.pilot_depth_mm
"""
from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from _harness import FakeCtx, assert_green, assert_red, base_data, findings, result  # noqa: E402

GID = "F_NEG_pilot_depth"
NAME = "盲深先读 pilot.pilot_depth_mm：3.0 → 顶底红；joins 4.5 → 绿；两者不一致 → unknown"
EXPECT = f"A：L5/{GID}:screw_length_rule FAIL(BLOCK) measured=8.0；B/D PASS；C unknown 且 detail 含两个数"
STACK = 4.0


def _data(pilot, joins_depth):
    joins = ["N03 板", "H01 Ø1.6 底孔" + (f" {joins_depth}" if joins_depth is not None else "（深度另见 pilot）")]
    fa = {"id": GID, "spec": "M2×8", "qty": 1, "joins": joins, "joint_type": "pla_self_tap",
          "engagement_mm": {"v": 8.0 - STACK, "src": "measured"}, "stack_mm": {"v": STACK, "src": "measured"},
          "status": "ok", "status_verbatim": "ok", "feature_ids": [], "feature_hole_map": [],
          "provenance": ["negatives/n_l5_pilot_depth_priority"]}
    if pilot is not None:
        fa["pilot"] = {"pilot_hole_d_mm": {"v": 1.6, "src": "assumed"},
                       "pilot_depth_mm": {"v": pilot, "src": "measured", "src_note": "反例"}}
    part = {"id": "N03", "inventory_id": "N03_neg_pilot", "material": "PLA", "print_orientation": "任意", "mirrored_copy": None}
    return base_data([part], [], fasteners=[fa], relations=[])


def _observe(pilot, joins_depth):
    import l5_function
    res = l5_function.run(FakeCtx(_data(pilot, joins_depth), {}))
    return findings(res, subject=GID, check="screw_length_rule")


def _fmt(fs):
    return [f"{f.state}/{f.measured}｜{str(f.detail)[:120]}" for f in fs]


def run() -> dict:
    A = _observe(3.0, None)
    B = _observe(None, 4.5)
    C = _observe(3.0, 4.5)
    D = _observe(4.5, 4.5)
    okA, whyA, redA = assert_red(A, severity="BLOCK")
    okA_src = any("pilot.pilot_depth_mm" in str(f.detail) for f in A)
    okB, whyB = assert_green(B)
    okC = bool(C) and all(f.state == "FAIL" and f.measured is None for f in C) and \
        any("3.0" in str(f.detail) and "4.5" in str(f.detail) for f in C)
    okD, whyD = assert_green(D)
    okD_src = any("pilot.pilot_depth_mm" in str(f.detail) for f in D)
    checks = [("A pilot 3.0：screw_length_rule FAIL(BLOCK) measured 非空", okA),
              ("A：detail 写明盲深来源 pilot.pilot_depth_mm", okA_src),
              ("B joins 4.5：PASS（盲深来源不同结论随之变）", okB),
              ("C 3.0 vs 4.5 不一致：unknown 且 detail 含两个数", okC),
              ("D 4.5/4.5 一致：PASS", okD), ("D：detail 写明 pilot.pilot_depth_mm", okD_src)]
    failed = [c for c, ok in checks if not ok]
    got = f"A {_fmt(A)}｜B {_fmt(B)}｜C {_fmt(C)}｜D {_fmt(D)}"
    return result(NAME, EXPECT, not failed, got + (f"｜未成立：{failed}" if failed else ""), redA,
                  expect_severity="BLOCK", detail=" | ".join(x for x in (whyA, whyB, whyD) if x))


if __name__ == "__main__":
    import json
    r = run()
    print(json.dumps(r, ensure_ascii=False, indent=1))
    sys.exit(0 if r["passed"] else 1)
