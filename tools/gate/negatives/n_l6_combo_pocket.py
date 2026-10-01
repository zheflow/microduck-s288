#!/usr/bin/env python3
"""反例（09-13 审计 F-L6-1）—— 组合碰撞口袋比粗网格窄：pair_combination 不许假绿。

坏样本：lower_leg × ankle_foot 只在 θ_knee ∈ [20°, 25°] ∧ θ_ankle ∈ [20°, 25°] 有一处 5 mm³ 干涉
（两轴都非零、口袋宽 5° = 修后网格步长：零度对齐的 5° 网格**保证**含 20°/25°；宽度 < 步长的口袋只有对齐时才抓得到，
所以本反例同时断言"detail 记的网格步长 ≤ 口袋宽度" —— 网格一放大就红）。修前两两组合用 max(10°, 量程/12) 粗网格（±90° 关节 = 15°：…0, 15, 30…），
一个采样点都落不进口袋，只在"最差粗格"附近加密 —— 没有最差格就不加密 → pair_combination PASS = 假绿。
修后统一 5° 网格（20° 是网格点）→ FAIL(BLOCK)。

场景：替身 placed/（全清单 1 mm 立方，运动树是真的 duckstructure.lib.B）；布尔换成按姿态判口袋的合成 evaluate
（本反例验的是网格，不是几何）。停在哪一步：不停，run 跑到底；断言只落在 L6/left_knee+left_ankle:pair_combination。
  坏   口袋 → pair_combination FAIL(BLOCK) measured=5.0，且 detail 记的网格步长 ≤ 4°（口袋宽度）
  旁证 同一坏样本的 single_axis_sweep(left_knee / left_ankle) 必须绿 —— 证明这条只有组合扫描能抓
  对照 无口袋 → pair_combination PASS 且 evidence_n > 0
"""
from __future__ import annotations
import json
import re
import sys
from pathlib import Path

_HERE = Path(__file__).resolve()
sys.path.insert(0, str(_HERE.parent))
import _l6_scene                                                                 # noqa: E402,F401  先设 sys.path
import core                                                                      # noqa: E402
from _harness import findings, assert_red, assert_green, result, summarize        # noqa: E402
from _l6_scene import dummy_placed, scene_class, run_l6                          # noqa: E402

J1, J2 = "left_knee", "left_ankle"
A, B = "lower_leg", "ankle_foot"
POCKET = (20.0, 25.0)
NAME = "组合碰撞口袋（两轴都在 [20°,25°] 才干涉，宽 5° ≪ 原粗网格 15°）：pair_combination 必须红"
EXPECT = (f"L6/{J1}+{J2}:pair_combination FAIL(BLOCK) measured=5.0，网格步长 ≤ {POCKET[1] - POCKET[0]}°；"
          f"single_axis_sweep({J1}/{J2}) 绿；无口袋对照 PASS")


def _hit(pose, a, b):
    if {a, b} != {A, B}:
        return 0.0
    t1, t2 = float(pose.get(J1, 0.0)), float(pose.get(J2, 0.0))
    return 5.0 if (POCKET[0] <= t1 <= POCKET[1] and POCKET[0] <= t2 <= POCKET[1]) else 0.0


def run() -> dict:
    base = core.load_data()
    placed = dummy_placed("combo_pocket")
    r_bad = run_l6(base, placed, scene_class(hit_fn=_hit))
    r_good = run_l6(base, placed, scene_class(hit_fn=lambda *a: 0.0))

    fs_bad = findings(r_bad, subject=f"{J1}+{J2}", check="pair_combination")
    ok_red, why_red, red = assert_red(fs_bad, severity="BLOCK", require_measured=True)
    # 网格步长必须写进 detail，且 ≤ 口袋宽度（否则抓到只是碰巧）
    det = " ".join(f.detail for f in fs_bad)
    m = re.search(r"(\d+(?:\.\d+)?)°\s*(?:统一)?网格", det)
    step = float(m.group(1)) if m else None
    ok_step = step is not None and step <= POCKET[1] - POCKET[0]
    ok_single = all(assert_green(findings(r_bad, subject=j, check="single_axis_sweep"))[0] for j in (J1, J2))
    ok_green, why_green = assert_green(findings(r_good, subject=f"{J1}+{J2}", check="pair_combination"))
    passed = ok_red and ok_step and ok_single and ok_green
    return result(NAME, EXPECT, passed,
                  got=(f"口袋样本 {'红' if ok_red else '未红：' + why_red}；detail 记的网格步长 = {step}°"
                       f"{'' if ok_step else '（> 口袋宽度或没写）'}；单轴 {'绿' if ok_single else '不绿'}；"
                       f"对照 {'绿' if ok_green else why_green}"
                       + ("" if ok_red else " → 粗网格漏掉两轴都非零的窄口袋（F-L6-1）")),
                  red=red, expect_severity="BLOCK",
                  detail=f"坏：{summarize(fs_bad)}；对照：{summarize(findings(r_good, subject=f'{J1}+{J2}'))}")


if __name__ == "__main__":
    print(json.dumps(run(), ensure_ascii=False, indent=1))
