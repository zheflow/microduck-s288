#!/usr/bin/env python3
"""反例 F-数-1（2026-09-13）—— frozen.yaml 的几何冻结项必须真的对导出件核，不是纸面冻结。

历史：battery_bay_26 / h01_sbc_posts / imu_pose 没有任何层读（审计 F-数-1）。修法：第 3 层 _frozen_checks 按 frozen.yaml 的
measured_on 指到件，射线/截面量出来和冻结值比（tolerances.yaml:frozen_dimension_mm）。

场景（件全是造的，世界坐标；冻结值 = 真 frozen.yaml 的数）：
  · 仓 TB：底板 z 70..73.7、前壁 x −24.6..−21.1、两条导轨 |y| 13.1..15.0 × z 100..146.9。
      bad：导轨内面挪到 13.6 → TB:frozen_battery_bay_26:BAT_Y FAIL(BLOCK)（偏差 0.5 > 0.1），RAIL_Y 仍 PASS。
  · 门 TD：板 x −46.8..−43.8、|y|≤8、z 70..110 → DOOR_T / BX0 / DOOR_YLO PASS。
  · 立柱 HP：4 根 Ø5 柱 @ (y ±29, z ±11.5) 带 Ø1.7 底孔 → count/hole_pitch/post_d/pilot_d PASS；bad：y ±28.5 → hole_pitch FAIL。
  · IMU IM：块上两个 Ø1.7 x 向孔 @ y ±8、z 105.3 → frozen_imu_pose_vs_cad PASS；bad：两孔整体 +1 → FAIL（孔心均值 1 ≠ 0）。
  · measured_on 缺 → _frozen:battery_bay_26_measured_on unknown。
  · hr50（2026-09-28）门板探针退役：frozen.yaml:battery_bay_26.door_plate_retired{probes: [DOOR_T, BX0, DOOR_YLO], date, decided_by}
      → TD 三条 state=RETIRED（criterion 带"已退役"，detail 仍记实测数），TB 仓体探针照旧 PASS；
      坏标记（probes 含仓体探针 BAT_Y）→ _frozen:door_plate_retired unknown，且 TD DOOR_T 照旧判（不退役）；
      hr48 形态的门（板 x −48.8..−45.4 = 外面后移 2.0 + 内面电池槽 1.6）不带标记 → DOOR_T FAIL 3.4 / BX0 FAIL −45.4 /
      DOOR_YLO 标准位 x=−45.3 无料、改在门板外面往内 1.0 处补量 → PASS 8.0；带标记 → 三条 RETIRED 记 3.4 / −45.4 / 8.0。
"""
from __future__ import annotations
import json
import sys
from pathlib import Path

_HERE = Path(__file__).resolve()
GATE = _HERE.parents[1]
for _p in (str(_HERE.parent), str(GATE), str(GATE / "layers"), str(GATE.parents[1])):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import core                                                                 # noqa: E402
from _harness import FakeCtx, base_data, box, cyl, findings, assert_red, assert_green, result, record  # noqa: E402

NAME = "冻结项真核：仓半宽错 0.5 / 立柱孔距错 1 / IMU 孔整体偏 1 必须红；对照绿；measured_on 缺 → unknown；hr50 门板探针退役标记 → RETIRED 记实测、坏标记 unknown、hr48 门无标记照红"
EXPECT = ("TB:frozen_battery_bay_26:BAT_Y FAIL(BLOCK)；HP:frozen_h01_sbc_posts:hole_pitch FAIL(BLOCK)；IM:frozen_imu_pose_vs_cad FAIL(BLOCK)；"
          "对照全 PASS；_frozen:battery_bay_26_measured_on unknown")
_TMP = core.ROOT / "tools/gate/out/_neg_tmp"
BX0, BX1, BZ0, BZ1, BATY, RAILY, RAILZ0, BAYFW = -43.8, -24.6, 73.7, 146.9, 13.1, 15.0, 100.0, 3.5


def _wbox(lo, hi):
    return box(hi[0] - lo[0], hi[1] - lo[1], hi[2] - lo[2], center=tuple((a + b) / 2 for a, b in zip(lo, hi)))


def _bay(baty):
    floor = _wbox((BX0, -RAILY, 70.0), (BX1 + BAYFW, RAILY, BZ0))
    front = _wbox((BX1, -RAILY, 70.0), (BX1 + BAYFW, RAILY, 150.0))
    rails = [_wbox((BX0, sy * baty if sy < 0 else baty, RAILZ0), (BX1 + BAYFW, sy * baty if sy > 0 else -baty, BZ1)) for sy in (1, -1)]
    rails = [_wbox((BX0, baty, RAILZ0), (BX1 + BAYFW, RAILY, BZ1)), _wbox((BX0, -RAILY, RAILZ0), (BX1 + BAYFW, -baty, BZ1))]
    m = floor.union(front)
    for r in rails:
        m = m.union(r)
    return m


def _door(hr48=False):
    if hr48:                                    # 门外面 −48.8、内面电池槽底 −45.4（lib.DOOR_ADD 2.0 / DOOR_SLOT_DEPTH 1.6）
        return _wbox((BX0 - 5.0, -8.0, 70.0), (BX0 - 1.6, 8.0, 110.0))
    return _wbox((BX0 - 3.0, -8.0, 70.0), (BX0, 8.0, 110.0))


def _posts(half_y):
    plate = _wbox((-3.0, -40.0, -25.0), (0.0, 40.0, 25.0))
    m = plate
    for sy in (1, -1):
        for sz in (1, -1):
            post = cyl(2.5, 15.0, center=(7.5, sy * half_y, sz * 11.5), axis="x")
            m = m.union(post)
    for sy in (1, -1):
        for sz in (1, -1):
            m = m.difference(cyl(0.85, 12.0, center=(9.0, sy * half_y, sz * 11.5), axis="x"))
    return m


def _imu(shift_y):
    blk = _wbox((-28.1, -15.0, 98.0), (-18.1, 15.0, 112.0))
    for sy in (1, -1):
        blk = blk.difference(cyl(0.85, 20.0, center=(-23.1, sy * 8.0 + shift_y, 105.3), axis="x"))
    return blk


def _frozen(measured_on=True, retired=None):
    items = [{"name": n, "value": {"v": v, "src": "assumed"}, "meaning": n} for n, v in
             (("BX0", BX0), ("BX1", BX1), ("BZ0", BZ0), ("BZ1", BZ1), ("BAT_Y", BATY), ("RAIL_Y", RAILY), ("RAIL_Z0", RAILZ0),
              ("BAY_FW", BAYFW), ("DOOR_T", 3.0), ("DOOR_YLO", 8.0), ("BAT_CLR", 0.6))]
    fz = {"battery_bay_26": {"items": items},
          "h01_sbc_posts": {"post_count": 4, "post_d_mm": {"v": 5.0}, "pilot_d_mm": {"v": 1.7}, "hole_pitch_mm": {"v": [58.0, 23.0]},
                            "post_axis_world": [1, 0, 0]},
          "imu_pose": {"site_name": "imu", "pos_local_m": [-0.021, 6.64098e-05, -0.0146984],
                       "pos_world_mm_zero_pose": {"v": [-21.0, 0.07, 105.3]}, "our_mounting_pos_mm": {"v": [-23.1, 0.0, 105.3]},
                       "delta_from_site_mm": {"v": [-2.1, -0.07, 0.0]}, "mount_hole_d_mm": {"v": 1.7}}}
    if measured_on:
        fz["battery_bay_26"]["measured_on"] = {"bay": "TB", "door": "TD"}
        fz["h01_sbc_posts"]["measured_on"] = "HP"
        fz["imu_pose"]["measured_on"] = "IM"
    if retired is not None:
        fz["battery_bay_26"]["door_plate_retired"] = retired
    return fz


def _observe(tag, baty=BATY, half_y=29.0, imu_shift=0.0, measured_on=True, retired=None, hr48_door=False):
    import os
    import l3_static as L3
    d = _TMP / f"p{os.getpid()}" / f"l3_frozen_{tag}"
    d.mkdir(parents=True, exist_ok=True)
    for p in d.glob("*.stl"):
        p.unlink()
    meshes = {"TB": _bay(baty), "TD": _door(hr48_door), "HP": _posts(half_y), "IM": _imu(imu_shift)}
    stl = {}
    for pid, m in meshes.items():
        p = d / f"{pid.lower()}_body.stl"
        m.export(str(p), file_type="stl")
        stl[pid] = p
    parts = [{"id": pid, "inventory_id": f"{pid}_neg", "material": "PLA", "print_orientation": "任意", "mirrored_copy": None,
              "qty": 1, "placed_instances": [f"{pid.lower()}_body"]} for pid in meshes]
    data = base_data(parts, [])
    data["components"] = {"components": []}
    data["keepouts"] = {"keepouts": []}
    data["frozen"] = _frozen(measured_on, retired)
    old = L3.PLACED
    try:
        L3.PLACED = d
        return L3.run(FakeCtx(data, stl, {p.stem: p for p in d.glob("*.stl")}))
    finally:
        L3.PLACED = old


def run() -> dict:
    r_good = _observe("good")
    r_bay = _observe("bay", baty=13.6)
    r_post = _observe("post", half_y=28.5)
    r_imu = _observe("imu", imu_shift=1.0)
    r_unk = _observe("unk", measured_on=False)
    FLAG = {"date": "2026-09-28", "decided_by": "反例：用户 09-22 解冻电池仓+门、hr48 有意改门", "probes": ["DOOR_T", "BX0", "DOOR_YLO"],
            "regression_now": "L2 features.yaml:B01-F01 thickness_mm / bbox_mm"}
    r_ret = _observe("ret", retired=FLAG)
    r_badflag = _observe("badflag", retired=dict(FLAG, probes=["DOOR_T", "BAT_Y"]))
    r_48 = _observe("hr48", hr48_door=True)
    r_48ret = _observe("hr48ret", hr48_door=True, retired=FLAG)
    DOORS = ("frozen_battery_bay_26:DOOR_T", "frozen_battery_bay_26:BX0", "frozen_battery_bay_26:DOOR_YLO")

    def one(res, subj, chk):
        fs = findings(res, subject=subj, check=chk)
        return fs[0] if len(fs) == 1 else None

    def st(f):
        return f.state if f is not None else None
    ret_f = [one(r_ret, "TD", c) for c in DOORS]
    ret_ok = all(f is not None and f.state == "RETIRED" and "已退役" in (f.criterion or "") and f.measured is not None for f in ret_f)
    ret_bay_ok = all(st(one(r_ret, "TB", c)) == "PASS" for c in ("frozen_battery_bay_26:BAT_Y", "frozen_battery_bay_26:BX1", "frozen_battery_bay_26:BZ0"))
    bf = one(r_badflag, "_frozen", "door_plate_retired")
    badflag_ok = (bf is not None and bf.state == "FAIL" and bf.measured is None and st(one(r_badflag, "TD", "frozen_battery_bay_26:DOOR_T")) == "PASS")
    f48 = {c.split(":")[1]: one(r_48, "TD", c) for c in DOORS}
    hr48_ok = (st(f48["DOOR_T"]) == "FAIL" and f48["DOOR_T"].severity == "BLOCK" and abs(float(f48["DOOR_T"].measured) - 3.4) < 1e-6
               and st(f48["BX0"]) == "FAIL" and abs(float(f48["BX0"].measured) + 45.4) < 1e-6
               and st(f48["DOOR_YLO"]) == "PASS" and abs(float(f48["DOOR_YLO"].measured) - 8.0) < 1e-6
               and "量不到" in (f48["DOOR_YLO"].detail or ""))
    f48r = {c.split(":")[1]: one(r_48ret, "TD", c) for c in DOORS}
    hr48ret_ok = (all(st(f) == "RETIRED" for f in f48r.values()) and abs(float(f48r["DOOR_T"].measured) - 3.4) < 1e-6
                  and abs(float(f48r["BX0"].measured) + 45.4) < 1e-6 and abs(float(f48r["DOOR_YLO"].measured) - 8.0) < 1e-6)
    red_48 = [record(f48["DOOR_T"]), record(f48["BX0"])] if hr48_ok else []

    ok_bay, why_bay, red_bay = assert_red(findings(r_bay, subject="TB", check="frozen_battery_bay_26:BAT_Y"), severity="BLOCK")
    ok_rail, why_rail = assert_green(findings(r_bay, subject="TB", check="frozen_battery_bay_26:RAIL_Y"))
    ok_post, why_post, red_post = assert_red(findings(r_post, subject="HP", check="frozen_h01_sbc_posts:hole_pitch"), severity="BLOCK")
    ok_imu, why_imu, red_imu = assert_red(findings(r_imu, subject="IM", check="frozen_imu_pose_vs_cad"), severity="BLOCK")
    goods = {}
    for subj, chk in (("TB", "frozen_battery_bay_26:BAT_Y"), ("TB", "frozen_battery_bay_26:BX1"), ("TB", "frozen_battery_bay_26:BAY_FW"),
                      ("TB", "frozen_battery_bay_26:BZ0"), ("TB", "frozen_battery_bay_26:BZ1"), ("TB", "frozen_battery_bay_26:RAIL_Z0"),
                      ("TD", "frozen_battery_bay_26:DOOR_T"), ("TD", "frozen_battery_bay_26:BX0"), ("TD", "frozen_battery_bay_26:DOOR_YLO"),
                      ("HP", "frozen_h01_sbc_posts:count"), ("HP", "frozen_h01_sbc_posts:hole_pitch"), ("HP", "frozen_h01_sbc_posts:post_d"),
                      ("HP", "frozen_h01_sbc_posts:pilot_d"), ("IM", "frozen_imu_pose_vs_cad"), ("_frozen", "frozen_imu_pose_delta_consistent")):
        ok, why = assert_green(findings(r_good, subject=subj, check=chk))
        goods[f"{subj}:{chk}"] = ok or why
    good_ok = all(v is True for v in goods.values())
    unk = findings(r_unk, subject="_frozen", check="battery_bay_26_measured_on")
    unk_ok = bool(unk) and unk[0].state == "FAIL" and unk[0].measured is None and not findings(r_unk, subject="TB", check_prefix="frozen_")

    assertions = [
        {"claim": "门板退役标记：TD DOOR_T/BX0/DOOR_YLO → RETIRED，criterion 带'已退役'，detail 仍有实测数", "ok": bool(ret_ok)},
        {"claim": "门板退役标记不影响仓体探针（TB BAT_Y/BX1/BZ0 仍 PASS）", "ok": bool(ret_bay_ok)},
        {"claim": "坏标记（probes 含 BAT_Y）→ _frozen:door_plate_retired unknown，且不退役 DOOR_T", "ok": bool(badflag_ok)},
        {"claim": "hr48 形态门无标记：DOOR_T FAIL 3.4 / BX0 FAIL −45.4 / DOOR_YLO 补量位置 PASS 8.0", "ok": bool(hr48_ok)},
        {"claim": "hr48 形态门带标记：三条 RETIRED 且记 3.4 / −45.4 / 8.0", "ok": bool(hr48ret_ok)},
    ]
    passed = ok_bay and ok_rail and ok_post and ok_imu and good_ok and unk_ok and all(a["ok"] for a in assertions)
    out = result(NAME, EXPECT, passed,
                  got=(f"仓半宽 13.6：{'红' if ok_bay else why_bay}（{red_bay[0]['measured'] if red_bay else None}），RAIL_Y {'绿' if ok_rail else why_rail}；"
                       f"柱距 57：{'红' if ok_post else why_post}（{red_post[0]['measured'] if red_post else None}）；"
                       f"IMU 孔偏 1：{'红' if ok_imu else why_imu}（{red_imu[0]['measured'] if red_imu else None}）；"
                       f"对照：{'全绿' if good_ok else {k: v for k, v in goods.items() if v is not True}}；measured_on 缺：{'unknown' if unk_ok else '未 unknown'}"),
                  red=red_bay + red_post + red_imu + red_48, expect_severity="BLOCK",
                  detail=f"good={[record(f) for f in r_good.findings if f.check.startswith('frozen') or f.subject == '_frozen'][:20]}；"
                         f"retired={[record(f) for f in ret_f if f]}；hr48={[record(f) for f in f48.values() if f]}")
    out["assertions"] = assertions
    return out


if __name__ == "__main__":
    print(json.dumps(run(), ensure_ascii=False, indent=1))
