#!/usr/bin/env python3
"""反例（09-13 审计 F-L6-3）—— 碰撞落在目标区间外是工作空间问题（WARN），落在区间内才是件的问题（BLOCK）。

口径（用户拍板）：目标区间**只**从 frozen.yaml:joint_axes[].target_range_deg 读；碰撞姿态在目标区间内 → FAIL(BLOCK)；
全部在目标区间外（但仍在上游扫描域内）→ FAIL(WARN)，detail 写"工作空间问题（控制端限位），不是件的问题"；
两两组合按两轴都在各自目标区间内算"区间内"。

场景：替身 placed/ + 真运动树 + 合成 evaluate：lower_leg × ankle_foot 在 θ_left_ankle ≥ 70° 时干涉 3 mm³
（单轴、组合、包络姿态只要 left_ankle ≥ 70 都撞）。
  坏 A 目标区间人为收窄到 [-60, 60]（带 date + reason）→ 碰撞全在区间外：
        left_ankle:single_axis_sweep / left_knee+left_ankle:pair_combination / _summary:collision_buckets /
        L04|L05:motion_collision 都 FAIL(**WARN**)，detail 含"工作空间"
  坏 B 目标区间 = 上游值（显式写成 range_deg，不依赖真数据的 target 是否已收窄 —— 2026-09-15 起真 frozen.yaml 的
        left_ankle 已收窄到 [-62,62]，再拿真数据当 B 会把 ≥70° 的碰撞判到区间外）→ 碰撞在区间内：同一组判据
        FAIL(**BLOCK**)（assertions 里验，red 只收 WARN 那组）
  对照 无碰撞 → left_ankle:single_axis_sweep PASS
修前层没有目标区间概念，A 也判 BLOCK → 本反例失败。
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
from _l6_scene import dummy_placed, scene_class, run_l6                          # noqa: E402

J1, J2 = "left_knee", "left_ankle"
A, B = "lower_leg", "ankle_foot"
LIMIT = 70.0
NARROW = [-60.0, 60.0]
NAME = "目标区间外的碰撞降 WARN（工作空间问题）、区间内 BLOCK（件的问题）—— 只认 frozen.yaml:target_range_deg"
EXPECT = (f"A 收窄 [{NARROW[0]},{NARROW[1]}]（碰撞在 ≥{LIMIT}°）：single_axis_sweep/pair_combination/"
          f"collision_buckets/motion_collision FAIL(WARN) 且 detail 含'工作空间'；B 上游区间：同组 FAIL(BLOCK)；对照 PASS")


def _hit(pose, a, b):
    if {a, b} != {A, B}:
        return 0.0
    return 3.0 if float(pose.get(J2, 0.0)) >= LIMIT else 0.0


def _narrowed(base):
    d = deepcopy(base)
    j = next(x for x in d["frozen"]["joint_axes"] if x["name"] == J2)
    j["target_range_deg"] = dict(j.get("target_range_deg") or {})
    j["target_range_deg"]["v"] = list(NARROW)
    j["target_range_deg"]["date"] = "2026-09-13"
    j["target_range_deg"]["reason"] = "反例 n_l6_target_range_severity：人为收窄，验区间外碰撞降 WARN"
    return d


def _upstream(base):
    """B：目标区间显式 = 上游值（frozen.yaml:joint_axes[].range_deg 那份上游拷贝），去掉 date/reason。
    不能直接用真数据：真 target_range_deg 一旦人为收窄（2026-09-15 left_ankle → [-62,62]），B 就不再是"上游区间"。"""
    d = deepcopy(base)
    j = next(x for x in d["frozen"]["joint_axes"] if x["name"] == J2)
    j["target_range_deg"] = {"v": [float(v) for v in j["range_deg"]],
                             "src": "反例 n_l6_target_range_severity：B 显式取上游值（= range_deg）"}
    return d


def _pids(res, stems):
    """本反例的两只实体对应的件号（层里 stem→pid 由 parts.yaml 桥接，这里从 motion_collision 的 subject 反查）。"""
    return sorted({f.subject for f in res.findings if f.check == "motion_collision"})


def run() -> dict:
    base = core.load_data()
    placed = dummy_placed("target_sev")
    cls = scene_class(hit_fn=_hit)
    r_a = run_l6(_narrowed(base), placed, cls)
    r_b = run_l6(_upstream(base), placed, cls)
    r_g = run_l6(_upstream(base), placed, scene_class(hit_fn=lambda *a: 0.0))

    def group(res):
        fs = (findings(res, subject=J2, check="single_axis_sweep")
              + findings(res, subject=f"{J1}+{J2}", check="pair_combination")
              + findings(res, subject="_summary", check="collision_buckets"))
        fs += [f for f in res.findings if f.check == "motion_collision"]
        return fs

    fa, fb = group(r_a), group(r_b)
    ok_a, why_a, red_a = assert_red(fa, severity="WARN", require_measured=True)
    ok_a_all = ok_a and all(f.state == "FAIL" and f.severity == "WARN" for f in fa) and len(fa) >= 4
    ok_a_txt = ok_a and all("工作空间" in (f.detail or "") for f in fa)
    ok_b, why_b, red_b = assert_red(fb, severity="BLOCK", require_measured=True)
    ok_b_all = ok_b and all(f.state == "FAIL" and f.severity == "BLOCK" for f in fb) and len(fb) >= 4
    # 口径必须写进 criterion：目标区间只认 frozen.yaml 声明
    crit = " ".join((f.criterion or "") for f in fa)
    ok_crit = "frozen.yaml" in crit and "target_range_deg" in crit
    ok_g, why_g = assert_green(findings(r_g, subject=J2, check="single_axis_sweep"))
    assertions = [
        {"claim": f"A 收窄后 {len(fa)} 条相关判据全部 FAIL(WARN)（{why_a or 'ok'}）", "ok": bool(ok_a_all)},
        {"claim": "A 的 detail 都写明'工作空间问题'", "ok": bool(ok_a_txt)},
        {"claim": f"B 上游区间 {len(fb)} 条相关判据全部 FAIL(BLOCK)（{why_b or 'ok'}）", "ok": bool(ok_b_all)},
        {"claim": "criterion 写明目标区间只认 frozen.yaml:target_range_deg", "ok": bool(ok_crit)},
        {"claim": f"对照 single_axis_sweep({J2}) 绿（{why_g or 'ok'}）", "ok": bool(ok_g)},
    ]
    passed = all(x["ok"] for x in assertions)
    out = result(NAME, EXPECT, passed,
                 got=("A(WARN): " + ("红" if ok_a_all else why_a or "有条目不是 FAIL(WARN)")
                      + "；B(BLOCK): " + ("红" if ok_b_all else why_b or "有条目不是 FAIL(BLOCK)")
                      + f"；对照 {'绿' if ok_g else why_g}"
                      + ("" if ok_a_all else " → 层对上游全程内任何碰撞判 BLOCK，没有目标区间概念（F-L6-3）")),
                 red=red_a, expect_severity="WARN",
                 detail=f"A：{summarize(fa, n=8)}；B：{summarize(fb, n=8)}；motion_collision 件号 {_pids(r_a, (A, B))}")
    out["assertions"] = assertions
    out["red_block_B"] = red_b
    return out


if __name__ == "__main__":
    print(json.dumps(run(), ensure_ascii=False, indent=1))
