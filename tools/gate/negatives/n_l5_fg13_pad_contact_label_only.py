#!/usr/bin/env python3
"""反例 FG13 —— H02 垫柱退 0.30 mm 什么都没夹住，但清单里 contact=0 / status=ok / src=measured。

这就是 B2 那次事故的复刻：第 3 层"干涉 = 0"完美通过（因为确实不干涉），
第 5 层只看 `status` / `actual_src` 这两个**标签**和 `nominal` 与 `target` 的**纸面比较**，
于是 `L5/<关系>` 和 `L5/fits.servo.pad_contact` 两个格子都是绿的 —— 压板一点夹紧力都没有。

正确判据：实测当前双方的**接触面积**、**有符号配合量**（过盈为正 / 间隙为负）、**夹紧行程**。
`status: ok` 和 `actual_src: measured` 只能决定证据等级，不能替代几何。
"""
from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from _harness import FakeCtx, base_data, box, cyl, export_stl, findings, assert_red, assert_green, result  # noqa: E402

RID = "NG13_pad_contact"
BUCKET = "fits.servo.pad_contact"
NAME = "H02 垫柱退 0.30 夹不住但 status=ok/src=measured：接触几何必须红，贴合必须绿"
EXPECT = f"L5/{RID}:relation_contact（接触面积 0 mm²、有符号配合量 −0.30、夹紧行程 0.30）"
_TARGET = ("relation_contact",)

SERVO_FACE_X = -14.85        # 横滚舵机薄段背面
GAP = 0.30                   # 坏样本：垫柱端面退到 -15.15


def _pad(bad: bool):
    """两根 Ø5 垫柱；坏样本端面停在 -15.15（差 0.30 够不着），好样本正好贴在 -14.85。"""
    face = SERVO_FACE_X - (GAP if bad else 0.0)
    a = cyl(2.5, 10.0, center=(face - 5.0, 4.0, 0.0), axis="x")
    b = cyl(2.5, 10.0, center=(face - 5.0, -4.0, 0.0), axis="x")
    return a.union(b)


def _servo():
    """薄段背面在 x=-14.85 的一块料。"""
    return box(10.0, 20.0, 12.0, center=(SERVO_FACE_X + 5.0, 0.0, 0.0))


def _data():
    parts = [{"id": "H02", "inventory_id": "H02_pad", "material": "PLA",
              "print_orientation": "背板外面朝下", "mirrored_copy": None},
             {"id": "H01", "inventory_id": "H01_servoblock", "material": "PLA",
              "print_orientation": "脚底朝下", "mirrored_copy": None}]
    rel = {"id": RID, "kind": "clamp",
           "between": ["H02 垫柱端面", "横滚舵机薄段背面 -14.85"],
           "declared_target": "接触",
           "target_ref": f"tolerances.yaml:{BUCKET}",
           "actual": "按 T=19.9 名义接触",
           "actual_src": "measured",          # ← 标签说实测
           "status": "ok",                    # ← 标签说通过
           "contact": {"axis": "x", "approach": "+x",
                       "parties": [{"part": "H02", "role": "mover"},
                                   {"part": "H01", "role": "fixed"}],
                       "probe_bbox_mm": [[-16.5, -8.0, -4.0], [-13.5, 8.0, 4.0]]},
           "provenance": ["negatives/n_l5_fg13"]}
    return base_data(parts, [], fasteners=[], relations=[rel])


def _observe(bad: bool):
    import l2_features
    import l5_function
    l2_features._GEOM_CACHE.clear()
    tag = "gap" if bad else "touch"
    pa = export_stl(_pad(bad), f"fg13_pad_{tag}")
    pb = export_stl(_servo(), f"fg13_servo_{tag}")
    ctx = FakeCtx(_data(), {"H02": pa, "H01": pb}, placed_map={"pad": pa, "servoblock": pb})
    res = l5_function.run(ctx)
    rel = findings(res, subject=RID)
    fit = findings(res, subject=BUCKET)
    tgt = [f for f in rel if any(t in f.check for t in _TARGET)]
    return (tgt,
            {"关系格": {f.check: f"{f.state}/{f.measured}" for f in rel},
             f"配合桶 {BUCKET}": {f.check: f"{f.state}/{f.measured}" for f in fit}})


def run() -> dict:
    bad, bad_all = _observe(True)
    good, good_all = _observe(False)
    ok_red, why_red, red = assert_red(bad, severity="BLOCK", require_measured=True)
    ok_green, why_green = assert_green(good)
    return result(NAME, EXPECT, ok_red and ok_green,
                  got=f"退 0.30 样本 {'红' if ok_red else why_red}(measured={red[0]['measured'] if red else None}) / 贴合对照 {'绿' if ok_green else why_green}",
                  red=red, expect_severity="BLOCK",
                  detail=f"退 0.30 样本={bad_all}｜贴合对照={good_all}")


if __name__ == "__main__":
    import json
    print(json.dumps(run(), ensure_ascii=False, indent=1))
