#!/usr/bin/env python3
"""反例（09-13 F-L7-3）—— `static_torque` 只判单轴扫描峰值：home 姿态扭矩算了不判、区间反写只扫两端点也能 PASS。

两个坏样本合在**同一次** L7 run 里（subject 不同、互不影响；整机 run ≈ 9 s，守 30 s）：
  ① 区间反写：frozen.yaml:joint_axes[left_knee].range_deg 头尾对调 [90, -90]。
     修前：check_angles(90, -90) 只给两端点 → `joint:left_knee:static_torque` PASS，evidence_n=2（"扫了"两个角）。
     修后：lo ≥ hi → 该关节 unknown（measured=None），detail 指出区间反写；评估点数 == 2 永远不许 PASS。
  ② home 超扭矩、扫描峰值不超：head_yaw 的轴在零位与重力平行，单轴扫描全程扭矩恒为 0（真数据就是 0.0%）。
     夹具把 head_pitch 的 home 取反（−0.3491，与 neck_pitch 的 +0.3491 同向叠成 40° 前倾，真数据两者反向抵消、
     头是水平的），并把头偏航舵机实体 servo__yaw_roll_motion_yaw_roll_motion 沿 +y（= neck_pitch/head_pitch 的轴向，
     所以两个俯仰关节的扫描峰值**一个数都不变**）平移 1.5 m，舵机 mass_g ×4（19.5 → 78 g；neck_pitch 扫描峰值 38%，仍绿）。
     home 姿态下 head_yaw 轴前倾 40°，这 78 g × 1.5 m 的力臂给出 0.729 N·m = 额定 0.6 的 122%。
     修前：`joint:head_yaw:static_torque` PASS（扫描峰值 0），home +0.7293 只出现在 detail 文本里，没有任何 FAIL。
     修后：`joint:head_yaw:static_torque_home` FAIL(BLOCK, measured ≈ 1.2155)，同关节 `static_torque` 仍 PASS。
对照：`_l7_fixture.complete_inventory`（质量清单堵上缺口，否则 F-L7-4 会把整机判据全判 unknown）+ 真 frozen →
     14 条 static_torque 与 14 条 static_torque_home 全 PASS。
用了 negatives/_l7_fixture.py 的夹具（只删不编，见该文件头）。
"""
from __future__ import annotations
import json
import math
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

NAME = "L7：区间反写 → static_torque 必须 unknown（不许两端点 PASS）；home 超扭矩而扫描不超 → static_torque_home 红"
EXPECT = ("L7/joint:left_knee:static_torque FAIL(measured=None，区间反写)；"
          "L7/joint:head_yaw:static_torque_home FAIL(BLOCK, measured≥1) 且同关节 static_torque PASS；"
          "对照 14 关节 static_torque / static_torque_home 全 PASS")
REV_JOINT = "left_knee"
HOME_JOINT = "head_yaw"
SHIFT_STEM = "servo__yaw_roll_motion_yaw_roll_motion"
SHIFT_MM = (0.0, 1500.0, 0.0)
SERVO_X = 4.0


def _bad_data(ctrl):
    d = deepcopy(ctrl)
    for j in d["frozen"]["joint_axes"]:
        if j.get("name") == REV_JOINT:
            lo, hi = j["range_deg"]
            j["range_deg"] = [hi, lo]
        if j.get("name") == "head_pitch":
            j["home_rad"] = -float(j["home_rad"])
    c = next(x for x in d["components"]["components"] if x.get("id") == "servo_s288")
    c["mass_g"]["v"] = float(c["mass_g"]["v"]) * SERVO_X
    return d


def _shift(stem, src, dst):
    if stem != SHIFT_STEM:
        return False
    import trimesh
    m = trimesh.load(str(src), process=False)
    m.apply_translation(list(SHIFT_MM))
    m.export(str(dst), file_type="stl")
    return True


def run() -> dict:
    base = core.load_data()
    ctrl, drop = FX.complete_inventory(base)
    pd_good = FX.placed_stand_in("home_ctrl", drop)
    pd_bad = FX.placed_stand_in("home_bad", drop, mutate=_shift)

    r_bad = FX.run_l7(_bad_data(ctrl), placed_dir=pd_bad)
    r_good = FX.run_l7(ctrl, placed_dir=pd_good)

    # ① 反写关节：必须 unknown，且 detail 指向区间
    knee = findings(r_bad, subject=f"joint:{REV_JOINT}", check="static_torque")
    ok1, why1, red1 = assert_red(knee, severity="BLOCK", require_measured=False)
    aimed1 = ok1 and all(x["measured"] is None for x in red1) and \
        any(("反" in (f.detail or "") or "有序" in (f.detail or "") or "lo" in (f.detail or "")) for f in knee)
    ev_knee = [f.evidence_n for f in knee]

    # ② home 超扭矩：static_torque_home 红、同关节 static_torque 绿、其余关节扫描全绿
    home = findings(r_bad, subject=f"joint:{HOME_JOINT}", check="static_torque_home")
    ok2, why2, red2 = assert_red(home, severity="BLOCK", require_measured=True)
    ratio_ok = ok2 and all(float(x["measured"]) >= 1.0 for x in red2)
    sweep_same = findings(r_bad, subject=f"joint:{HOME_JOINT}", check="static_torque")
    ok2s, why2s = assert_green(sweep_same)
    other_sweeps = [f for f in r_bad.findings if f.check == "static_torque"
                    and f.subject not in (f"joint:{REV_JOINT}",)]
    ok2o, why2o = assert_green(other_sweeps)

    # 对照
    g_sw = [f for f in r_good.findings if f.check == "static_torque"]
    g_hm = [f for f in r_good.findings if f.check == "static_torque_home"]
    okg1, whyg1 = assert_green(g_sw)
    okg2, whyg2 = assert_green(g_hm)
    n_ok = len(g_sw) == 14 and len(g_hm) == 14

    passed = ok1 and aimed1 and ok2 and ratio_ok and ok2s and ok2o and okg1 and okg2 and n_ok
    return result(NAME, EXPECT, passed,
                  got=(f"① {REV_JOINT} 反写 → {summarize(knee, n=1)}（evidence_n={ev_knee}）"
                       f"{'' if aimed1 else ' —— 未按区间反写判 unknown'}；"
                       f"② {HOME_JOINT} home → {summarize(home, n=1) if home else '（没有 static_torque_home 这条判据）'}"
                       f"，同关节扫描 {'绿' if ok2s else why2s}，其余关节扫描 {'全绿' if ok2o else why2o}；"
                       f"对照 static_torque {len(g_sw)} 条 {'绿' if okg1 else whyg1}，"
                       f"static_torque_home {len(g_hm)} 条 {'绿' if okg2 else whyg2}"),
                  red=(red1 if ok1 else []) + (red2 if ok2 else []), expect_severity="BLOCK",
                  allow_unknown_red="① 区间反写是坏声明：层判 unknown（measured=None）才算抓到；② 的 measured 非空",
                  detail=(f"①={[record(f) for f in knee]}；②={[record(f) for f in home]}；"
                          f"同关节扫描={[record(f) for f in sweep_same]}；夹具：servo×{SERVO_X:g}、{SHIFT_STEM} 平移 {SHIFT_MM} mm、"
                          f"head_pitch home 取反；对照={summarize(g_hm, n=2)}"))


if __name__ == "__main__":
    r = run()
    print(json.dumps(r, ensure_ascii=False, indent=1))
    sys.exit(0 if r["passed"] else 1)
