#!/usr/bin/env python3
"""反例（09-13 审计 F-L6-3，collision_free_range 改判）—— 生成的 MJCF 的 <joint range> 是独立产物，会过期。

新口径（两条判据，不再因为"需要收窄"本身判 BLOCK）：
  · `_ranges:collision_free_range`     生成的 sim/duck_s288/robot_walk_s288.xml 每条 <joint range> ⊆ 本轮扫出的无碰撞区间，
                                        否则 FAIL(BLOCK)（训练域里有实物会撞的角度）
  · `_ranges:gen_range_covers_target`  每条 <joint range> ⊇ frozen.yaml:target_range_deg，否则 FAIL(WARN)（训练域被收窄）

场景：替身 placed/ + 真运动树 + 合成 evaluate（lower_leg × ankle_foot 在 θ_left_ankle ≥ 70° 干涉 →
单轴无碰撞区间 left_ankle = [-90, 67.5]）；"生成的 MJCF"用真文件为底、全部关节写成上游值再改 left_ankle：
  坏 D1 left_ankle range = [-90, 90]（⊄ 无碰撞区间）             → collision_free_range FAIL(BLOCK) measured ≥ 1，detail 点名 left_ankle
  坏 D2 left_ankle range = [-30, 30]（⊆ 无碰撞区间，⊉ 目标 [-90,90]）→ collision_free_range PASS，gen_range_covers_target FAIL(WARN)
  对照 D0 目标收窄到 [-90, 60]（带 date+reason），range = [-90, 65] → 两条都 PASS
修前：collision_free_range 只要"需要收窄"就 BLOCK（D0 也红），且没有 gen_range_covers_target → 本反例失败。
"""
from __future__ import annotations
import json
import sys
from copy import deepcopy
from pathlib import Path

_HERE = Path(__file__).resolve()
sys.path.insert(0, str(_HERE.parent))
import _l6_scene                                                                 # noqa: E402,F401  先设 sys.path
import core                                                                      # noqa: E402
from _harness import findings, assert_red, assert_green, result, record, summarize   # noqa: E402
from _l6_scene import dummy_placed, scene_class, run_l6, gen_mjcf_with_ranges, upstream_ranges_deg   # noqa: E402

J = "left_ankle"
A, B = "lower_leg", "ankle_foot"
NAME = "生成 MJCF 的 <joint range> ⊆ 无碰撞区间（否则 BLOCK）且 ⊇ 目标区间（否则 WARN）—— 不再因'需要收窄'判 BLOCK"
EXPECT = ("D1 range ⊄ 无碰撞区间 → _ranges:collision_free_range FAIL(BLOCK)；D2 ⊆ 但 ⊉ 目标 → "
          "collision_free_range PASS + _ranges:gen_range_covers_target FAIL(WARN)；D0 → 两条 PASS")


def _hit(pose, a, b):
    if {a, b} != {A, B}:
        return 0.0
    return 3.0 if float(pose.get(J, 0.0)) >= 70.0 else 0.0


def _narrow_target(base, lo, hi):
    d = deepcopy(base)
    j = next(x for x in d["frozen"]["joint_axes"] if x["name"] == J)
    j["target_range_deg"] = dict(j.get("target_range_deg") or {})
    j["target_range_deg"].update({"v": [lo, hi], "date": "2026-09-13", "reason": "反例 n_l6_gen_range_vs_free 对照样本"})
    return d


def run() -> dict:
    base = core.load_data()
    up = upstream_ranges_deg(base)
    if J not in up:
        return result(NAME, EXPECT, False, got=f"frozen.yaml 里 {J} 没有 target_range_deg，反例建不起来", red=[])
    placed = dummy_placed("gen_range")
    cls = scene_class(hit_fn=_hit)

    x1, n1 = gen_mjcf_with_ranges("D1", {**up, J: (-90.0, 90.0)})
    x2, n2 = gen_mjcf_with_ranges("D2", {**up, J: (-30.0, 30.0)})
    x0, n0 = gen_mjcf_with_ranges("D0", {**up, J: (-90.0, 65.0)})
    r1 = run_l6(base, placed, cls, gen_mjcf=x1)
    r2 = run_l6(base, placed, cls, gen_mjcf=x2)
    r0 = run_l6(_narrow_target(base, -90.0, 60.0), placed, cls, gen_mjcf=x0)

    f1 = findings(r1, subject="_ranges", check="collision_free_range")
    ok1, why1, red1 = assert_red(f1, severity="BLOCK", require_measured=True)
    ok1 = ok1 and any(J in (f.detail or "") for f in f1)
    f2a = findings(r2, subject="_ranges", check="collision_free_range")
    f2b = findings(r2, subject="_ranges", check="gen_range_covers_target")
    ok2a, why2a = assert_green(f2a)
    ok2b, why2b, red2 = assert_red(f2b, severity="WARN", require_measured=True)
    ok2b = ok2b and any(J in (f.detail or "") for f in f2b)
    ok0a, why0a = assert_green(findings(r0, subject="_ranges", check="collision_free_range"))
    ok0b, why0b = assert_green(findings(r0, subject="_ranges", check="gen_range_covers_target"))
    assertions = [
        {"claim": f"D1 生成区间 [-90,90] ⊄ 无碰撞区间 → collision_free_range FAIL(BLOCK) 点名 {J}（{why1 or 'ok'}）", "ok": bool(ok1)},
        {"claim": f"D2 collision_free_range PASS（{why2a or 'ok'}）", "ok": bool(ok2a)},
        {"claim": f"D2 gen_range_covers_target FAIL(WARN) 点名 {J}（{why2b or 'ok'}）", "ok": bool(ok2b)},
        {"claim": f"D0 collision_free_range PASS（{why0a or 'ok'}）", "ok": bool(ok0a)},
        {"claim": f"D0 gen_range_covers_target PASS（{why0b or 'ok'}）", "ok": bool(ok0b)},
        {"claim": f"临时 MJCF 改写到了 <joint range>（D1 {n1} / D2 {n2} / D0 {n0} 条）", "ok": n1 >= 14 and n2 >= 14 and n0 >= 14},
    ]
    passed = all(a["ok"] for a in assertions)
    out = result(NAME, EXPECT, passed,
                 got="；".join(("✔" if a["ok"] else "✘") + a["claim"][:40] for a in assertions)
                     + ("" if passed else " → 修前 collision_free_range 因'需要收窄'恒 BLOCK 且无 gen_range_covers_target（F-L6-3）"),
                 red=red1, expect_severity="BLOCK",
                 detail=f"D1：{summarize(f1)}；D2：{summarize(f2a + f2b)}；D0：{summarize(findings(r0, subject='_ranges'))}")
    out["assertions"] = assertions
    out["red_warn_D2"] = red2
    return out


if __name__ == "__main__":
    print(json.dumps(run(), ensure_ascii=False, indent=1))
