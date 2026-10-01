#!/usr/bin/env python3
"""配合桶的名义凸出量不能代替卡扣咬合证据；每组只能出一条声明判据。"""
from __future__ import annotations

import copy
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from _harness import FakeCtx, base_data, record, result

NAME = "未知卡扣咬合必须保持 unknown，不能借配合桶名义量补齐"
EXPECT = "F31/F32/F28 未知深度均 FAIL(BLOCK, measured=None)；显式声明逐组单条判据且越界必红"


def _fastener(gid, feature_ids, engagement=None):
    return {"id": gid, "joint_type": "pla_snap", "spec": "打印卡扣", "qty": 2,
            "feature_ids": feature_ids, "joins": ["T01", "B01"],
            "engagement_mm": {"v": engagement, "src": "assumed" if engagement is not None else None},
            "stack_mm": {"v": None, "src": None}, "status": "unverified"}


def _observe(fasteners, protrusion=None):
    import l5_function
    data = base_data([], [], fasteners=copy.deepcopy(fasteners), relations=[])
    if protrusion is not None:
        data["tolerances"]["fits"]["snap"]["b01"]["nominal_mm"]["v"]["protrusion"] = protrusion
    res = l5_function.run(FakeCtx(data, {}))
    return [f for f in res.findings if f.check == "engagement_declared"]


def run():
    original = _fastener("F28_snap_B01", ["T01-F14", "B01-F05"])
    foreign = [_fastener("F31_B03_front_detents", ["B03-F04", "T02-F08"]),
               _fastener("F32_B03_rear_barb", ["B03-F06", "B01-F11"]),
               _fastener("F28_snap_B01", ["B03-F04"])]
    reds = [f for fa in foreign for f in _observe([fa])]
    b01 = _observe([original])
    b01_nominal_changed = _observe([original], protrusion=1.2)
    explicit = _observe([_fastener("F31_B03_front_detents", ["B03-F04"], 0.8)])
    bad_depth = _observe([_fastener("F31_B03_front_detents", ["B03-F04"], 1.2)])
    assertions = [
        ("三个未知或错配卡扣各一条 unknown", len(reds) == 3 and all(
            f.state == "FAIL" and f.severity == "BLOCK" and f.measured is None for f in reds)),
        ("原 B01 同样保留未知", len(b01) == 1 and b01[0].state == "FAIL"
         and b01[0].measured is None),
        ("改名义量不能改变未知状态", len(b01_nominal_changed) == 1
         and b01_nominal_changed[0].state == "FAIL" and b01_nominal_changed[0].measured is None),
        ("显式深度越界不能追加空 PASS", len(bad_depth) == 1 and bad_depth[0].state == "FAIL"
         and bad_depth[0].measured == 1.2),
        ("显式深度走正常声明检查", len(explicit) == 1 and explicit[0].state == "PASS"
         and explicit[0].measured == 0.8),
    ]
    failed = [name for name, ok in assertions if not ok]
    out = result(NAME, EXPECT, not failed,
                 json.dumps({"unknown": [record(f) for f in reds],
                             "b01_unknown": [record(f) for f in b01],
                             "explicit": [record(f) for f in explicit],
                             "out_of_range": [record(f) for f in bad_depth]}, ensure_ascii=False),
                 [record(f) for f in reds if f.state == "FAIL"],
                 expect_severity="BLOCK", detail="；".join(failed))
    out["allow_unknown_red"] = "此反例验证缺数据不能借用别的卡扣数值，measured=None 是预期。"
    return out


if __name__ == "__main__":
    outcome = run()
    print(json.dumps(outcome, ensure_ascii=False, indent=2))
    sys.exit(0 if outcome["passed"] else 1)
