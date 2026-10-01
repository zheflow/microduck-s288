#!/usr/bin/env python3
"""反例 审计 §2.6 / L3 MINOR（2026-09-13）—— 禁入体的几何参数与豁免必须来自 keepouts.yaml，层里不写死。

历史：KO15 光轴 `+x`、FOV/出瞳从 name 正则解析（Codex01 FG16）；KO12 的豁免件号 "B01" 写死在 l3_static.py；
KO06/KO07 的关节名、KO11 的 z 区间也写死。修法：全部改读 keepouts.yaml 字段，缺 → unknown。

场景：
  · KO15：出瞳 (0,0,0)、FOV 80×60、挡板放在 x=+10。optical_axis_world=[1,0,0] → camera_fov_unobstructed FAIL(BLOCK)；
          光轴改成 [-1,0,0]（挡板在背后）→ PASS；删掉 optical_axis_world → unknown（不再默认 +x）。
  · KO12：仓 x −83.8..−24.6、BX0=−43.8 拆两段。件 DOOR（不叫 B01）放在抽出走廊段：exempt 声明 {part: DOOR, segment: 抽出走廊}
          → keepout_intersection PASS（超阈值记在"按声明排除"）；不声明 → FAIL(BLOCK)；DOOR 压进仓腔段 → 即使声明也 FAIL。
  · KO07：删 axis_joint → KO07:axis_joint_declared unknown（一条，不重复），且没有 keepout_intersection。
"""
from __future__ import annotations
import copy
import json
import sys
from pathlib import Path

_HERE = Path(__file__).resolve()
GATE = _HERE.parents[1]
for _p in (str(_HERE.parent), str(GATE), str(GATE / "layers"), str(GATE.parents[1])):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import core                                                                 # noqa: E402
from _harness import FakeCtx, base_data, box, findings, assert_red, assert_green, result, record  # noqa: E402

NAME = "禁入体参数来自数据：KO15 光轴/FOV/出瞳、KO12 豁免件、KO07 关节名 —— 写死的默认值全部不许有"
EXPECT = ("KO15 光轴 +x 挡板红 / −x 绿 / 缺字段 unknown；KO12 走廊段 DOOR 声明豁免绿、不声明红、进仓腔声明也红；KO07 缺 axis_joint unknown")
_TMP = core.ROOT / "tools/gate/out/_neg_tmp"


def _ko(kid):
    d = core.load_data()
    return copy.deepcopy(next(k for k in d["keepouts"]["keepouts"] if k["id"] == kid))


def _observe(tag, parts_meshes, keepouts, frozen=None):
    import os
    import l3_static as L3
    d = _TMP / f"p{os.getpid()}" / f"l3_kodata_{tag}"
    d.mkdir(parents=True, exist_ok=True)
    for p in d.glob("*.stl"):
        p.unlink()
    stl, parts = {}, []
    for pid, m in parts_meshes.items():
        p = d / f"{pid.lower()}_body.stl"
        m.export(str(p), file_type="stl")
        stl[pid] = p
        parts.append({"id": pid, "inventory_id": f"{pid}_neg", "material": "PLA", "print_orientation": "任意", "mirrored_copy": None,
                      "qty": 1, "placed_instances": [f"{pid.lower()}_body"]})
    data = base_data(parts, [])
    data["components"] = {"components": []}
    data["keepouts"] = {"keepouts": keepouts}
    data["frozen"] = frozen or {}
    old = L3.PLACED
    try:
        L3.PLACED = d
        return L3.run(FakeCtx(data, stl, {p.stem: p for p in d.glob("*.stl")}))
    finally:
        L3.PLACED = old


def run() -> dict:
    # ── KO15 ──
    ko15 = _ko("KO15")
    ko15.update({"fov_deg": {"v": [80.0, 60.0]}, "apex_world_mm": {"v": [0.0, 0.0, 0.0]}, "optical_axis_world": {"v": [1.0, 0.0, 0.0]}})
    blocker = {"BLK": box(2.0, 40.0, 40.0, center=(10.0, 0.0, 0.0))}
    r15_bad = _observe("ko15_bad", blocker, [ko15])
    ko15_back = copy.deepcopy(ko15)
    ko15_back["optical_axis_world"] = {"v": [-1.0, 0.0, 0.0]}
    r15_good = _observe("ko15_good", blocker, [ko15_back])
    ko15_unk = copy.deepcopy(ko15)
    ko15_unk.pop("optical_axis_world")
    r15_unk = _observe("ko15_unk", blocker, [ko15_unk])
    ok15, why15, red15 = assert_red(findings(r15_bad, subject="KO15", check="camera_fov_unobstructed"), severity="BLOCK")
    ok15g, why15g = assert_green(findings(r15_good, subject="KO15", check="camera_fov_unobstructed"))
    u15 = findings(r15_unk, subject="KO15", check="camera_fov_unobstructed")
    ok15u = bool(u15) and u15[0].state == "FAIL" and u15[0].measured is None

    # ── KO12 ──
    ko12 = _ko("KO12")
    frozen = {"battery_bay_26": {"items": [{"name": "BX0", "value": {"v": -43.8, "src": "assumed"}}]}}
    door_corr = {"DOOR": box(5.0, 10.0, 10.0, center=(-47.5, 0.0, 110.0))}      # 抽出走廊段 x −83.8..−43.8
    door_bay = {"DOOR": box(5.0, 10.0, 10.0, center=(-37.5, 0.0, 110.0))}       # 仓腔段 x −43.8..−24.6
    ko12_ex = copy.deepcopy(ko12)
    ko12_ex["exempt"] = [{"part": "DOOR", "segment": "抽出走廊", "why": "反例：门堵走廊是设计如此"}]
    ko12_no = copy.deepcopy(ko12)
    ko12_no.pop("exempt", None)
    r12_ex = _observe("ko12_ex", door_corr, [ko12_ex], frozen)
    r12_no = _observe("ko12_no", door_corr, [ko12_no], frozen)
    r12_bay = _observe("ko12_bay", door_bay, [ko12_ex], frozen)
    ok12g, why12g = assert_green(findings(r12_ex, subject="KO12", check="keepout_intersection"))
    ex_recorded = bool(findings(r12_ex, subject="KO12", check="keepout_intersection")) and \
        "按声明排除" in str(findings(r12_ex, subject="KO12", check="keepout_intersection")[0].detail)
    ok12, why12, red12 = assert_red(findings(r12_no, subject="KO12", check="keepout_intersection"), severity="BLOCK")
    ok12b, why12b, red12b = assert_red(findings(r12_bay, subject="DOOR", check="keepout_KO12"), severity="BLOCK")

    # ── KO07 ──
    ko07 = _ko("KO07")
    ko07.pop("axis_joint", None)
    r07 = _observe("ko07", blocker, [ko07], {"joint_axes": core.load_data()["frozen"]["joint_axes"]})
    u07 = findings(r07, subject="KO07", check="axis_joint_declared")
    ok07 = (len(u07) == 1 and u07[0].state == "FAIL" and u07[0].measured is None and "axis_joint" in str(u07[0].detail)
            and not findings(r07, subject="KO07", check="keepout_intersection"))

    passed = ok15 and ok15g and ok15u and ok12g and ex_recorded and ok12 and ok12b and ok07
    return result(NAME, EXPECT, passed,
                  got=(f"KO15 +x：{'红' if ok15 else why15}（{red15[0]['measured'] if red15 else None} 条射线被挡）；−x：{'绿' if ok15g else why15g}；"
                       f"缺光轴：{'unknown' if ok15u else '未 unknown'}；KO12 走廊段声明豁免：{'绿' if ok12g else why12g}"
                       f"{'（排除有记录）' if ex_recorded else '（排除没记录！）'}；不声明：{'红' if ok12 else why12}；进仓腔：{'红' if ok12b else why12b}；"
                       f"KO07 缺 axis_joint：{'unknown' if ok07 else '未 unknown'}"),
                  red=red15 + red12 + red12b, expect_severity="BLOCK",
                  detail=f"u15={[record(f) for f in u15]}；u07={[record(f) for f in u07]}；"
                         f"ko12_ex={[record(f) for f in findings(r12_ex, subject='KO12')]}")


if __name__ == "__main__":
    print(json.dumps(run(), ensure_ascii=False, indent=1))
