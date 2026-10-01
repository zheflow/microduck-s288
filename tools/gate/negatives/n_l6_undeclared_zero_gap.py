#!/usr/bin/env python3
"""反例（09-13 审计 F-L6-2）—— 零位姿贴得近 ≠ 配合面：没在 relations.yaml 里声明 contact 的件对不许豁免最小距离判据。

修前 `_Scene.mating_pairs` 把零位姿间隙 < 0.3 mm 的**任何**件对自动豁免出 min_clearance（记分卡 L6/_zero：37 对），
非配合面的 0.2 mm 零位间隙也被当成"本来就该贴着"。修后只豁免 relations.yaml 里声明为 `contact` 的 `parties` 件对
（读数据，不写死件号）；零位间隙 < 阈值但没声明的件对另发 `_zero:undeclared_zero_gap_pair` FAIL(WARN) 列出来，不阻断。

场景（**真几何、真布尔**，不用合成 evaluate）：替身 placed/ 里把 trunk / yaw2roll 两只换成解析几何 ——
在 yaw2roll 零位姿局部系里，A = 4×4×4 立方，中心 (20,0,0)（离髋偏航轴 20 mm）；B（挂在 trunk 上）= 4×20×4 长条，
中心 (24.2,0,0) → 零位姿径向间隙 0.2 mm（< 阈值 0.3）。A 绕 z 转 θ 时前角的径向坐标 = 22cosθ + 2|sinθ| ≤ √(22²+2²) = 22.09 < 22.2，
**永远不相交**，但 θ = ±2.5…±10° 时间隙缩到 0.11–0.19 mm：这正是"交集 = 0 但装不上"的最小距离判据要抓的东西。
只扫 left_hip_yaw 一条关节（其余 exists_as_mjcf_joint=false，省时间；joint_inventory 会红，不断言）。
  坏   relations.yaml 原样（没有 trunk×yaw2roll 的 contact 声明）→ _clearance:min_clearance FAIL(BLOCK) measured < 0.3，
       且 _zero:undeclared_zero_gap_pair FAIL(WARN) 列出 trunk×yaw2roll
  对照 relations 里加一条 contact（parties placed: yaw2roll / trunk）→ min_clearance PASS（evidence_n>0）、undeclared_zero_gap_pair PASS
修前：零位间隙 0.2 < 0.3 → 自动豁免 → min_clearance 绿 → 本反例失败。
"""
from __future__ import annotations
import json
import sys
from copy import deepcopy
from pathlib import Path

import numpy as np

_HERE = Path(__file__).resolve()
sys.path.insert(0, str(_HERE.parent))
import _l6_scene                                                                 # noqa: E402,F401  先设 sys.path
import core                                                                      # noqa: E402
from _harness import findings, assert_red, assert_green, result, record, summarize   # noqa: E402
from _l6_scene import dummy_placed, scene_class, run_l6, restrict_joints         # noqa: E402

J = "left_hip_yaw"
A, B = "yaw2roll", "trunk"          # placed stem：A 挂在 yaw2roll（随 J 转），B 挂在 trunk_base（不动）
R_A, GAP0 = 20.0, 0.2
NAME = "零位间隙 0.2 mm 但 relations 没声明 contact 的件对：运动中间隙 <0.3 必须红（min_clearance），不许自动豁免"
EXPECT = ("坏：L6/_clearance:min_clearance FAIL(BLOCK) measured<0.3 且 L6/_zero:undeclared_zero_gap_pair FAIL(WARN) 列出 "
          f"{B}×{A}；对照（声明 contact）：两条都 PASS")


def _build_placed(tag):
    import trimesh
    from duckstructure.lib import B as BODIES
    T = np.asarray(BODIES[A]["T_world"], float)                 # yaw2roll 零位姿世界变换（关节轴 = 局部 z）
    d = dummy_placed(tag)
    a = trimesh.creation.box(extents=(4.0, 4.0, 4.0)); a.apply_translation([R_A, 0.0, 0.0])
    b = trimesh.creation.box(extents=(4.0, 20.0, 4.0)); b.apply_translation([R_A + 4.0 + GAP0, 0.0, 0.0])
    for m, stem in ((a, A), (b, B)):
        m.apply_transform(T)
        m.export(str(d / f"{stem}.stl"), file_type="stl")
    return d


def _with_contact(base):
    d = deepcopy(base)
    rels = list((d.get("relations") or {}).get("relations") or [])
    rels.append({"id": "RNEG_zero_gap", "kind": "contact_neg", "between": [f"{A}", f"{B}"],
                 "declared_target": "反例：声明为配合面", "status": "ok",
                 "contact": {"axis": "x", "approach": "+x",
                             "parties": [{"placed": A, "part": "L01", "role": "moving"},
                                         {"placed": B, "part": "T01", "role": "fixed"}]}})
    d["relations"]["relations"] = rels
    return d


def run() -> dict:
    base = restrict_joints(core.load_data(), {J})
    placed = _build_placed("zero_gap")
    cls = scene_class(keep={A, B})
    r_bad = run_l6(base, placed, cls)
    r_ok = run_l6(_with_contact(base), placed, cls)

    fb = findings(r_bad, subject="_clearance", check="min_clearance")
    ok_red, why_red, red = assert_red(fb, severity="BLOCK", require_measured=True)
    ok_val = ok_red and all(0.0 < float(f.measured) < 0.3 for f in fb)
    fu = findings(r_bad, subject="_zero", check="undeclared_zero_gap_pair")
    ok_u, why_u, red_u = assert_red(fu, severity="WARN", require_measured=True)
    ok_u = ok_u and any(A in (f.detail or "") and B in (f.detail or "") for f in fu)
    # 旁证：确实没相交（single_axis_sweep 绿）—— 红必须来自最小距离，不是交集
    ok_nohit, why_nohit = assert_green(findings(r_bad, subject=J, check="single_axis_sweep"))
    ok_g1, why_g1 = assert_green(findings(r_ok, subject="_clearance", check="min_clearance"))
    ok_g2, why_g2 = assert_green(findings(r_ok, subject="_zero", check="undeclared_zero_gap_pair"))
    assertions = [
        {"claim": f"坏样本 min_clearance FAIL(BLOCK) 且 0 < measured < 0.3（{why_red or 'ok'}）", "ok": bool(ok_val)},
        {"claim": f"坏样本 undeclared_zero_gap_pair FAIL(WARN) 列出 {B}×{A}（{why_u or 'ok'}）", "ok": bool(ok_u)},
        {"claim": f"坏样本 single_axis_sweep({J}) 绿 —— 红来自最小距离不是交集（{why_nohit or 'ok'}）", "ok": bool(ok_nohit)},
        {"claim": f"对照（声明 contact）min_clearance PASS（{why_g1 or 'ok'}）", "ok": bool(ok_g1)},
        {"claim": f"对照 undeclared_zero_gap_pair PASS（{why_g2 or 'ok'}）", "ok": bool(ok_g2)},
    ]
    passed = all(a["ok"] for a in assertions)
    out = result(NAME, EXPECT, passed,
                 got="；".join(("✔" if a["ok"] else "✘") + a["claim"][:44] for a in assertions)
                     + ("" if ok_val else " → 零位间隙 <0.3 的件对被自动豁免出 min_clearance（F-L6-2）"),
                 red=red, expect_severity="BLOCK",
                 detail=f"坏：{summarize(fb + fu)}；对照：{summarize(findings(r_ok, subject='_clearance') + findings(r_ok, subject='_zero'))}")
    out["assertions"] = assertions
    out["red_warn"] = red_u
    return out


if __name__ == "__main__":
    print(json.dumps(run(), ensure_ascii=False, indent=1))
