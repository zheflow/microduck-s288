#!/usr/bin/env python3
"""反例（09-13 审计 F-L6-3，声明有效性）—— 目标区间声明坏了，该关节的碰撞判据必须 unknown，不许照旧判。

口径：目标区间**只**从 frozen.yaml:joint_axes[].target_range_deg 读。缺失 / lo ≥ hi / 非数 / 超出上游扫描域 /
人为收窄（≠ 上游值）却没有 date + reason → 该关节相关判据 unknown（FAIL，measured=None，detail 说缺 target_range_deg）。
"没有目标区间就没法分区间内/外" —— 未知 = 失败（元规则 4），不能退回"全部 BLOCK"或"全部放行"。

场景：替身 placed/ + 真运动树 + 合成 evaluate（lower_leg × ankle_foot 在 θ_left_ankle ≥ 70° 干涉）；
只让 left_knee / left_ankle 两条关节参加扫描（joint_inventory 会因 ≠14 而红，不断言）。
  坏 C1 删掉 left_ankle 的 target_range_deg          → left_ankle:single_axis_sweep 与 left_knee+left_ankle:pair_combination unknown
  坏 C2 v = [60, -60]（无序）                          → 同上
  坏 C3 v = ["x", 60]（非数）                          → 同上
  坏 C4 v = [-60, 60] 但没有 date / reason（人为收窄无依据）→ 同上
  坏 C5 v = [-100, 90]（超出上游扫描域 [-90, 90]）      → 同上（区间外那段根本没扫过）
  对照 真数据 → left_ankle:single_axis_sweep 有 measured（不 unknown）
每条 unknown 的 detail 必须提到 target_range_deg（红在对的原因上）。修前层不读这个字段 → 全部照旧判 → 本反例失败。
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
from _harness import findings, assert_red, result, record, summarize             # noqa: E402
from _l6_scene import dummy_placed, scene_class, run_l6, restrict_joints         # noqa: E402

J1, J2 = "left_knee", "left_ankle"
A, B = "lower_leg", "ankle_foot"
NAME = "目标区间声明缺失/无序/非数/超域/人为收窄无 date+reason → 该关节碰撞判据 unknown（只认 frozen.yaml:target_range_deg）"
EXPECT = f"C1..C5：L6/{J2}:single_axis_sweep 与 L6/{J1}+{J2}:pair_combination FAIL(BLOCK) measured=None，detail 提到 target_range_deg；对照有数"


def _hit(pose, a, b):
    if {a, b} != {A, B}:
        return 0.0
    return 3.0 if float(pose.get(J2, 0.0)) >= 70.0 else 0.0


def _mut(base, fn):
    d = deepcopy(base)
    j = next(x for x in d["frozen"]["joint_axes"] if x["name"] == J2)
    fn(j)
    return d


def _set_v(v, **extra):
    def f(j):
        j["target_range_deg"] = {"v": v, "src": "反例", **extra}
    return f


def run() -> dict:
    base = restrict_joints(core.load_data(), {J1, J2})      # 只扫这两条关节（6 次 run 要压进 5 s）
    placed = dummy_placed("target_decl")
    cls = scene_class(hit_fn=_hit)
    cases = {
        "C1缺字段": _mut(base, lambda j: j.pop("target_range_deg", None)),
        "C2无序": _mut(base, _set_v([60.0, -60.0])),
        "C3非数": _mut(base, _set_v(["x", 60.0])),
        "C4收窄无date/reason": _mut(base, _set_v([-60.0, 60.0])),
        "C5超出扫描域": _mut(base, _set_v([-100.0, 90.0], date="2026-09-13", reason="反例：目标区间比上游扫描域宽")),
    }
    reds, got, ok_all = [], [], True
    for tag, d in cases.items():
        res = run_l6(d, placed, cls)
        fs = findings(res, subject=J2, check="single_axis_sweep") + findings(res, subject=f"{J1}+{J2}", check="pair_combination")
        ok, why, rec = assert_red(fs, severity="BLOCK", require_measured=False)
        unk = ok and len(fs) == 2 and all(f.state == "FAIL" and f.measured is None for f in fs)
        aimed = unk and all("target_range_deg" in (f.detail or "") for f in fs)
        ok_all &= bool(aimed)
        reds += rec
        got.append(f"{tag}:{'unknown' if aimed else ('红但不是 unknown/没指向 target_range_deg' if ok else why)}")
    r_g = run_l6(base, placed, cls)
    fg = findings(r_g, subject=J2, check="single_axis_sweep")
    ok_g = bool(fg) and fg[0].measured is not None
    got.append(f"对照:{'有数' if ok_g else '没有数'}")
    return result(NAME, EXPECT, ok_all and ok_g, got="；".join(got) + ("" if ok_all else " → 层不读 target_range_deg（F-L6-3）"),
                  red=reds, expect_severity="BLOCK",
                  allow_unknown_red="目标区间声明坏了是坏声明：层判 unknown（measured=None）即算抓到",
                  detail=f"对照 {summarize(fg)}；坏样本记录 {[ (r['subject'], r['check'], r['measured']) for r in reds ]}")


if __name__ == "__main__":
    print(json.dumps(run(), ensure_ascii=False, indent=1))
