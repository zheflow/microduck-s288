#!/usr/bin/env python3
"""反例（2026-09-16 能力判据）—— 原版某个任务模式用到的关节角够不到 = 能力受损，必须 BLOCK；
判据只认 frozen.yaml:capability_envelope 声明的 in-scope mode + 包络文件，且左右镜像求并。

场景：替身 placed/ + 真运动树 + 合成 evaluate：lower_leg × ankle_foot 在 θ_left_ankle ≥ 70° 时干涉 3 mm³
（→ 本层量出的 left_ankle 无碰撞区间上端 = 67.5）。包络文件用临时 json 替换（不动真文件）：
  坏 A  in-scope mode "m_bad" 的 left_ankle actual [-30, 80] → 需要 [-33, 83]（margin 3）⊄ [·, 67.5]
        → left_ankle:capability_envelope FAIL(BLOCK)，detail 写明 mode 名与缺口；_capability/box_collisions FAIL(WARN)（09-16 起盒子是上界，阻断改 real_pose_collisions）
  坏 B  只有 **right_ankle** 用到 −80（mode "m_mirror"），left_ankle 本身只到 30；mirror_pairs 声明右 = −左
        → 镜像求并后 left_ankle 也需要到 83 → 同样 FAIL(BLOCK)（能力不分左右）
  坏 C  同 A 但 "m_bad" **不在** modes_in_scope（放进 modes_out_of_scope）→ left_ankle:capability_envelope PASS
        （out-of-scope 的模式不构成能力要求）；对照组 D：包络 [-30, 50] → PASS
  坏 E  frozen.yaml 没有 capability_envelope → _capability/scope_declared unknown（未知=失败），不得绿
修前层里这条判据是 RETIRED（永远不红）→ 本反例失败。
"""
from __future__ import annotations
import json
import os
import sys
from copy import deepcopy
from pathlib import Path

_HERE = Path(__file__).resolve()
sys.path.insert(0, str(_HERE.parent))
import _l6_scene                                                                 # noqa: E402,F401
import core                                                                      # noqa: E402
from _harness import findings, assert_red, assert_green, result, summarize       # noqa: E402
from _l6_scene import dummy_placed, scene_class, run_l6                          # noqa: E402

J1, J2 = "left_knee", "left_ankle"
A, B = "lower_leg", "ankle_foot"
LIMIT = 70.0
NAME = "能力包络够不到 = BLOCK（frozen.yaml:capability_envelope，in-scope mode，左右镜像求并）"
EXPECT = ("A 包络 left_ankle 到 80：left_ankle:capability_envelope FAIL(BLOCK) 且 detail 含 mode 名；box_collisions FAIL(WARN)；"
          "B 只有 right_ankle 到 −80：镜像后同样 BLOCK；C 该 mode 不在 scope：PASS；D 包络到 50：PASS；E 无声明：unknown")

_TMP = Path(os.environ.get("TMPDIR", "/tmp")) / "duck_gate_neg" / f"cap{os.getpid()}"


def _hit(pose, a, b):
    if {a, b} != {A, B}:
        return 0.0
    return 3.0 if float(pose.get(J2, 0.0)) >= LIMIT else 0.0


def _env_json(tag, joints_by_mode):
    """{mode: {joint: (lo, hi)}} → 临时 policy_envelope.json（只写判据读的字段）。"""
    _TMP.mkdir(parents=True, exist_ok=True)
    pe = {m: {j: {"all": {"actual_deg": {"min": lo, "max": hi}}} for j, (lo, hi) in jd.items()}
          for m, jd in joints_by_mode.items()}
    p = _TMP / f"policy_envelope.{tag}.json"
    p.write_text(json.dumps({"policy_envelope": pe}), encoding="utf-8")
    return p


def _data(base, modes_in, modes_out=None, cap=True):
    d = deepcopy(base)
    if not cap:
        d["frozen"].pop("capability_envelope", None)
        return d
    d["frozen"]["capability_envelope"] = {
        "source": "docs/workspace_2026-09-09/policy_envelope.json",   # 被 run 时的 _ENVELOPE 替换，这里只是占位
        "key": "all.actual_deg", "modes_in_scope": list(modes_in),
        "modes_out_of_scope": {m: "反例：不在能力清单" for m in (modes_out or [])},
        "margin_deg": 3.0, "mirror_symmetric": True,
        "mirror_pairs": [["left_knee", "right_knee", -1], ["left_ankle", "right_ankle", -1]],
    }
    return d


def _run(data, env_path, hit=True):
    import l6_motion as L6
    saved = L6._ENVELOPE
    try:
        L6._ENVELOPE = env_path
        return run_l6(data, dummy_placed("cap_env"), scene_class(hit_fn=_hit if hit else (lambda *a: 0.0)))
    finally:
        L6._ENVELOPE = saved


def run() -> dict:
    base = core.load_data()
    ok_j = {J2: (-30.0, 80.0), J1: (-10.0, 30.0)}
    env_a = _env_json("a", {"m_bad": ok_j, "m_fine": {J2: (-20.0, 20.0)}})
    env_b = _env_json("b", {"m_mirror": {"right_ankle": (-80.0, 10.0), J2: (-10.0, 30.0)}})
    env_d = _env_json("d", {"m_fine": {J2: (-30.0, 50.0), J1: (-10.0, 30.0)}})

    r_a = _run(_data(base, ["m_bad", "m_fine"]), env_a)
    r_b = _run(_data(base, ["m_mirror"]), env_b)
    r_c = _run(_data(base, ["m_fine"], modes_out=["m_bad"]), env_a)
    r_d = _run(_data(base, ["m_fine"]), env_d)
    r_e = _run(_data(base, ["m_fine"], cap=False), env_d)

    fa = findings(r_a, subject=J2, check="capability_envelope")
    fa_box = findings(r_a, subject="_capability", check="box_collisions")
    ok_a, why_a, red_a = assert_red(fa, severity="BLOCK", require_measured=True)
    ok_a_txt = ok_a and all("m_bad" in (f.detail or "") for f in fa)
    ok_a_box, why_a_box, _ = assert_red(fa_box, severity="WARN", require_measured=True)     # 09-16：盒子判据降为 WARN（上界）
    fb = findings(r_b, subject=J2, check="capability_envelope")
    ok_b, why_b, _ = assert_red(fb, severity="BLOCK", require_measured=True)
    ok_b_txt = ok_b and all("m_mirror" in (f.detail or "") for f in fb)
    ok_c, why_c = assert_green(findings(r_c, subject=J2, check="capability_envelope"))
    ok_d, why_d = assert_green(findings(r_d, subject=J2, check="capability_envelope"))
    fe = findings(r_e, subject="_capability", check="scope_declared")
    ok_e = bool(fe) and all(f.state not in ("PASS",) and f.measured is None for f in fe)
    crit = " ".join((f.criterion or "") for f in fa)
    ok_crit = "capability_envelope" in crit and "frozen.yaml" in crit
    assertions = [
        {"claim": f"A left_ankle:capability_envelope FAIL(BLOCK)（{why_a or 'ok'}）", "ok": bool(ok_a)},
        {"claim": "A detail 写明够不到的 mode 名（m_bad）", "ok": bool(ok_a_txt)},
        {"claim": f"A _capability/box_collisions FAIL(WARN，上界)（{why_a_box or 'ok'}）", "ok": bool(ok_a_box)},
        {"claim": f"B 只有 right_ankle 用到 −80，镜像后 left_ankle 同样 FAIL(BLOCK)（{why_b or 'ok'}）", "ok": bool(ok_b and ok_b_txt)},
        {"claim": f"C 该 mode 不在 scope → PASS（{why_c or 'ok'}）", "ok": bool(ok_c)},
        {"claim": f"D 包络到 50 → PASS（{why_d or 'ok'}）", "ok": bool(ok_d)},
        {"claim": "E 没有 capability_envelope 声明 → _capability/scope_declared unknown（不绿）", "ok": bool(ok_e)},
        {"claim": "criterion 写明判据只认 frozen.yaml:capability_envelope", "ok": bool(ok_crit)},
    ]
    passed = all(x["ok"] for x in assertions)
    out = result(NAME, EXPECT, passed,
                 got="；".join(("✓" if x["ok"] else "✗") + x["claim"] for x in assertions),
                 red=red_a, expect_severity="BLOCK",
                 detail=f"A：{summarize(fa + fa_box, n=4)}；B：{summarize(fb, n=2)}；E：{summarize(fe, n=2)}")
    out["assertions"] = assertions
    return out


if __name__ == "__main__":
    print(json.dumps(run(), ensure_ascii=False, indent=1))
