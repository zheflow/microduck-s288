#!/usr/bin/env python3
"""反例（09-13 审计 §2.6，L7 部分）—— L7 里写死的四样东西必须改成从数据取：

  ① 镜像对数 `== 5`（l7:393）→ 由 frozen.yaml:joint_axes 的 left_X/right_X 同名对推：
     A1 声明 6 对（多加一对 left_extra/right_extra，exists_as_mjcf_joint=false，只为凑对数）→ mirror_body_pairs 必须 PASS(6)；
        修前：6 ≠ 5 → 假红。
     A2 声明 5 对但 right_knee 的 mjcf_body 置空 → FAIL(BLOCK, measured=4)，criterion 写"== 声明的对数"。
  ② `home_rad` 缺省当 0（l7:970）→ 缺失 = unknown：A1 里顺手把 left_ankle 的 home_rad 删掉 → `_load:home_pose_declared`
     unknown 并点名 left_ankle（修前：没有这条判据，缺失静默当 0）。
  ③ `"TPU" in material` 推落地件（l7:1301）→ 只认 parts.yaml 的显式字段 `ground_contact: true`：
     B 把 L06 的 ground_contact 删掉 → `_stability:support_polygon` unknown（修前：按 TPU 子串照样找到鞋底，绿）。
  ④ 筋厚/筋长 vs 声明 ±0.1（l7:1675/1701）→ tolerances.yaml:load.min_section_and_rib.rib_declared_vs_measured_tol_mm
     （src: assumed）：B 里把该键删掉 → `H02:H02-R01:rib_thickness` unknown（修前：±0.1 写死，PASS）。
  ⑤ 密度 1.27 / 稳定裕度 3.0 都是 src=assumed 却不产生任何 finding → 对照 C 里必须有
     `_layer:threshold_assumed:density_g_cm3` FAIL(WARN, measured=1.27) 与 `_stability:threshold_assumed:static_stability_margin_mm`
     FAIL(WARN, measured=3.0)，且 com_in_support_home 的 criterion 带 src 标签。
A1/A2 是快跑（parts 清单置空，run 在 _assign 之后返回；两条判据都在那之前发出）；B/C 是整机 run（各 ≈ 9 s）。
对照 C：`_l7_fixture.complete_inventory` + 真 frozen/parts/tolerances → mirror_body_pairs PASS(5)、home_pose_declared PASS、
com_in_support_home PASS、H02-R01:rib_thickness PASS。
用了 negatives/_l7_fixture.py 的夹具（只删不编，见该文件头）。
"""
from __future__ import annotations
import json
import sys
from copy import deepcopy
from pathlib import Path

_HERE = Path(__file__).resolve()
GATE = _HERE.parents[1]
for _p in (str(_HERE.parent), str(GATE), str(GATE / "layers"), str(GATE.parents[1])):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import core                                                                      # noqa: E402
from _harness import findings, assert_red, assert_green, result, record, summarize  # noqa: E402
import _l7_fixture as FX                                                         # noqa: E402

NAME = "L7 泛化：镜像对数从数据推、home_rad 缺失=unknown、落地件只认 ground_contact 字段、筋 ±0.1 进 tolerances、assumed 阈值发 WARN"
EXPECT = ("A1 6 对声明 → mirror_body_pairs PASS(6)，left_ankle 无 home_rad → home_pose_declared unknown；"
          "A2 right_knee 无 body → mirror_body_pairs FAIL(BLOCK, 4)；B 删 ground_contact → support_polygon unknown，"
          "删 rib 容差键 → rib_thickness unknown；C 对照全绿 + threshold_assumed WARN(1.27 / 3.0)")


def _fast(d):
    d = deepcopy(d)
    d["parts"] = {"parts": []}
    return d


def _a1(base):
    d = deepcopy(base)
    ja = d["frozen"]["joint_axes"]
    tmpl = next(j for j in ja if j.get("name") == "left_knee")
    for nm, body in (("left_extra", "neck"), ("right_extra", "neck_pitch")):
        j = deepcopy(tmpl)
        j["name"], j["mjcf_body"], j["exists_as_mjcf_joint"] = nm, body, False
        ja.append(j)
    for j in ja:
        if j.get("name") == "left_ankle":
            j.pop("home_rad", None)
    return d


def _a2(base):
    d = deepcopy(base)
    for j in d["frozen"]["joint_axes"]:
        if j.get("name") == "right_knee":
            j["mjcf_body"] = None
    return d


def _b(ctrl):
    d = deepcopy(ctrl)
    for p in d["parts"]["parts"]:
        if p.get("id") == "L06":
            p.pop("ground_contact", None)
    d["tolerances"]["load"]["min_section_and_rib"].pop("rib_declared_vs_measured_tol_mm", None)
    return d


def run() -> dict:
    base = core.load_data()
    ctrl, drop = FX.complete_inventory(base)
    pd = FX.placed_stand_in("general", drop)

    r_a1 = FX.run_l7(_fast(_a1(base)))
    r_a2 = FX.run_l7(_fast(_a2(base)))
    r_b = FX.run_l7(_b(ctrl), placed_dir=pd)
    r_c = FX.run_l7(ctrl, placed_dir=pd)

    # ① A1：6 对声明必须绿（修前 == 5 写死 → 假红）
    m1 = findings(r_a1, subject="_mjcf", check="mirror_body_pairs")
    ok_m1, why_m1 = assert_green(m1)
    ok_m1 = ok_m1 and all(int(f.measured) == 6 for f in m1)
    # ② A1：home_rad 缺失 → unknown 点名 left_ankle
    h1 = findings(r_a1, subject="_load", check="home_pose_declared")
    ok_h1 = bool(h1) and all(f.state == "FAIL" and f.measured is None and "left_ankle" in (f.detail or "") for f in h1)
    # ① A2：5 声明 4 配到 → FAIL measured=4，criterion 不写死 5
    m2 = findings(r_a2, subject="_mjcf", check="mirror_body_pairs")
    ok_m2, why_m2, red_m2 = assert_red(m2, severity="BLOCK", require_measured=True)
    ok_m2 = ok_m2 and all(int(float(x["measured"])) == 4 for x in red_m2) \
        and all("声明" in (f.criterion or "") for f in m2)
    # ③ B：没有 ground_contact 字段 → support_polygon unknown
    sp = findings(r_b, subject="_stability", check="support_polygon")
    ok_sp = bool(sp) and all(f.state == "FAIL" and f.measured is None and "ground_contact" in (f.detail or "") for f in sp)
    # ④ B：rib 容差键缺 → rib_thickness unknown
    rt = findings(r_b, subject="H02", check="H02-R01:rib_thickness")
    ok_rt = bool(rt) and all(f.state == "FAIL" and f.measured is None for f in rt)
    # ⑤ C：assumed 阈值发 WARN，值就是阈值；com 判据 criterion 带 src
    w_rho = findings(r_c, subject="_layer", check="threshold_assumed:density_g_cm3")
    w_stab = findings(r_c, subject="_stability", check="threshold_assumed:static_stability_margin_mm")
    ok_w = (bool(w_rho) and bool(w_stab)
            and all(f.state == "FAIL" and f.severity == "WARN" and f.measured is not None for f in w_rho + w_stab))
    # 对照 C
    c_m = findings(r_c, subject="_mjcf", check="mirror_body_pairs")
    c_h = findings(r_c, subject="_load", check="home_pose_declared")
    c_com = findings(r_c, subject="_stability", check="com_in_support_home")
    c_rt = findings(r_c, subject="H02", check="H02-R01:rib_thickness")
    ok_c, why_c = assert_green(c_m + c_h + c_com + c_rt)
    ok_c = ok_c and all(int(f.measured) == 5 for f in c_m) and all("src" in (f.criterion or "") for f in c_com)

    assertions = [
        {"claim": f"A1 6 对声明 → mirror_body_pairs PASS(6)（现：{summarize(m1, n=1)} {why_m1}）", "ok": ok_m1},
        {"claim": f"A1 left_ankle 无 home_rad → home_pose_declared unknown 点名（现：{summarize(h1, n=1)}）", "ok": ok_h1},
        {"claim": f"B 删 L06.ground_contact → support_polygon unknown 提到 ground_contact（现：{summarize(sp, n=1)}）", "ok": ok_sp},
        {"claim": f"B 删 rib_declared_vs_measured_tol_mm → H02-R01:rib_thickness unknown（现：{summarize(rt, n=1)}）", "ok": ok_rt},
        {"claim": f"C assumed 阈值发 WARN 且 measured=阈值（现：{summarize(w_rho + w_stab, n=2)}）", "ok": ok_w},
        {"claim": f"C 对照 mirror(5)/home/com(criterion 带 src)/rib_thickness 全绿（{why_c or 'ok'}）", "ok": ok_c},
    ]
    passed = ok_m2 and all(a["ok"] for a in assertions)
    out = result(NAME, EXPECT, passed,
                 got=(f"A2 right_knee 无 body → {summarize(m2, n=1)}{'' if ok_m2 else ' —— measured≠4 或 criterion 写死'}；"
                      + "；".join(("✓" if a["ok"] else "✗") + a["claim"][:60] for a in assertions)),
                 red=(red_m2 if ok_m2 else []), expect_severity="BLOCK",
                 detail=f"A1={[record(f) for f in m1 + h1]}；B={[record(f) for f in sp + rt]}；C={[record(f) for f in w_rho + w_stab + c_com]}")
    out["assertions"] = assertions
    return out


if __name__ == "__main__":
    r = run()
    print(json.dumps(r, ensure_ascii=False, indent=1))
    sys.exit(0 if r["passed"] else 1)
