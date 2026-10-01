#!/usr/bin/env python3
"""反例 README N4 —— 扫掠区间取反：frozen.yaml:joint_axes 某关节 `range_deg` 写成 [92, -62]（头尾对调）。

`duckstructure.checks.check_angles(lo, hi)` 在 lo > hi 时 arange 为空，**只返回两个端点** ——
如果层拿这个区间去扫，切刀/扫掠就只切两个点，运动交集"立刻 0"，这是 README N4 要抓的假绿。

现在的 l6_motion.run 怎么处理（l6_motion.py:409-424, 583-584）：
    扫描域取 `sc.B[body]["joint"]["range_deg"]`（上游 MJCF），frozen 的 range_deg 只进 `range_diff`，
    而 `_joints:range_matches_upstream` 已 RETIRED（不阻断）。所以 frozen 反写 [92,-62] 时：
    joint_inventory PASS、range_matches_upstream RETIRED、扫掠照旧 —— **层没有任何一条 FAIL**。
    这不是"抓到了"，是"根本没看 frozen 的区间"。按契约，本反例现在**必须失败**，
    直到层在 lo>hi 时判 unknown/FAIL（例如 `_joints:range_declared_ordered` 或把它并进 joint_inventory）。

本反例经过 `l6_motion.run(ctx)`：placed/ 用 n_l6_scene_inventory 的同名替身目录（清单完整、场景建得起来），
frozen 深拷贝后只把 left_knee 的 range_deg 反写。为了把断言限制在"关节清单/区间校验"这一步、不跑整机扫掠，
_Scene 保留真实构造（joint_body / B 都是真的），只把 evaluate 换成抛 ValueError 的桩 —— run 会在
`_zero:boolean` unknown 处停下（那已在关节清单与区间判据之后）。
  坏   left_knee range_deg=[92,-62] → 期望 L6/_joints 或 L6/left_knee 有一条 FAIL(BLOCK) 指出区间反写
  对照 正常 frozen → `_joints:joint_inventory` PASS（14 条都在树里）
"""
from __future__ import annotations
import json
import sys
from copy import deepcopy
from pathlib import Path

_HERE = Path(__file__).resolve()
GATE = _HERE.parents[1]
for _p in (str(_HERE.parent), str(GATE), str(GATE / "layers"), str(GATE.parents[1]), str(GATE.parents[1] / "tools/sim")):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import core                                                                      # noqa: E402
from _harness import FakeCtx, findings, assert_red, assert_green, result, record, summarize  # noqa: E402
from negatives.n_l6_scene_inventory import stems_declared, make_dummy_placed     # noqa: E402

JOINT = "left_knee"
NAME = "扫掠区间取反（frozen range_deg=[92,-62]）：L6 必须 unknown/FAIL，不许照旧绿"
EXPECT = f"L6/_joints 或 L6/{JOINT} 有一条 FAIL(BLOCK) 指出 range_deg 反写；正常区间 _joints:joint_inventory PASS"


def _run(data):
    import l6_motion as L6

    class _NoSweep(L6._Scene):
        """真场景（joint_body / B 都真），只不跑布尔 —— 断言限制在关节清单/区间校验那一步。"""
        def evaluate(self, *a, **k):
            raise ValueError("反例 n_l6_reversed_range：不跑扫掠")

    decl = stems_declared(data)
    placed = make_dummy_placed("reversed", decl)
    old_placed, old_scene = L6.PLACED, L6._Scene
    try:
        L6.PLACED = placed
        L6._Scene = _NoSweep
        return L6.run(FakeCtx(data, {}, {p.stem: p for p in placed.glob("*.stl")}))
    finally:
        L6.PLACED, L6._Scene = old_placed, old_scene


def _reversed_data(base):
    d = deepcopy(base)
    ja = d["frozen"]["joint_axes"]
    j = next(x for x in ja if x.get("name") == JOINT)
    lo, hi = j["range_deg"]
    j["range_deg"] = [hi, lo]                                   # 头尾对调，不取负
    return d, (lo, hi)


def run() -> dict:
    from duckstructure.checks import check_angles
    base = core.load_data()
    bad, (lo, hi) = _reversed_data(base)
    n_fwd, n_rev = len(check_angles(lo, hi)), len(check_angles(hi, lo))

    r_bad = _run(bad)
    r_good = _run(base)
    # 红只认关节清单/区间这两类 subject —— _data:motion_clearance_threshold 那条常红是数据缺口，不算
    fs_bad = findings(r_bad, subject="_joints") + findings(r_bad, subject=JOINT)
    ok_red, why_red, red = assert_red(fs_bad, severity="BLOCK", require_measured=False)
    # 必须真的是针对区间反写的：finding 的 detail/criterion 得提到这个关节或区间
    aimed = ok_red and any(JOINT in (x["subject"] + str(next((f.detail for f in fs_bad if f.check == x["check"]), "")))
                           for x in red)
    ok_green, why_green = assert_green(findings(r_good, subject="_joints", check="joint_inventory"))
    ok_inv_good = bool(findings(r_good, subject="_joints", check="joint_inventory"))
    passed = ok_red and aimed and ok_green
    retired = [record(f) for f in findings(r_bad, subject="_joints", check="range_matches_upstream")]
    return result(NAME, EXPECT, passed,
                  got=(f"check_angles({lo},{hi}) 给 {n_fwd} 个角；反写 check_angles({hi},{lo}) 只给 {n_rev} 个角（=两端点）。"
                       f"反写样本 {'红' if ok_red else '未红：' + why_red}"
                       + ("" if not ok_red or aimed else "（红了但没指向该关节/区间）")
                       + f"；对照 joint_inventory {'绿' if ok_green else why_green}"
                       + ("" if ok_red else "　→ 层现在按上游 MJCF 区间扫、frozen 反写只进已退役的 range_matches_upstream，"
                                             "需要层修复：lo>hi 时 unknown/FAIL")),
                  red=(red if ok_red else []), expect_severity="BLOCK",
                  allow_unknown_red="区间反写是坏声明：层判 unknown（measured=None）即算抓到",
                  detail=(f"反写样本 _joints/{JOINT}：{summarize(fs_bad, n=6)}；range_matches_upstream={retired}；"
                          f"对照有 joint_inventory={ok_inv_good}"))


if __name__ == "__main__":
    print(json.dumps(run(), ensure_ascii=False, indent=1))
