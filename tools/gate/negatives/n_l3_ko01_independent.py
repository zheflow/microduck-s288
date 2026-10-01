#!/usr/bin/env python3
"""反例 F-L3-2（2026-09-13，09-20 改成背插模型）—— KO01 插座让位实体必须由 keepouts.yaml 的声明独立建出，不借 CAD 切刀。

历史：l3_static 以前 `from duckstructure.lib import conn_cut` 拿 CAD 开窗用的**同一把刀**当禁入体，
载体 × 自家刀 = 0 是构造保证（切过的地方当然是空的），不是检查。09-20 插座模型换成背插（A 插头顶/线弯区 + B 侧出线槽 / W 背板通窗，
按 availability_table.rows[].mode_* 取），本反例跟着换几何，口径不变。

场景（真实运动学 duckstructure.lib.sfw 定舵机落位；件是造的；只留 trunk_base[0] 一行，−y 侧 mode 可改）：
  · bad_deep ：声明 window_x_mm 从 −18.5 改成 −25.0（更深的通窗），4×4×4 方块放在舵机局部 (x −22, y −7.75, z −9.3)。
                CAD 的 conn_cut(window) 只切到 −18.5 → 拿它当实体永远是 0（绿）；按声明独立建 → keepout_KO01 FAIL(BLOCK) ≈ 64 mm³。
  · good      ：同一块料、声明 window_x −18.5 → 料在通窗外 → keepout_intersection PASS。
  · bad_side  ：mode 改成 pocket，1.6×2×4 方块贴在侧出线槽里（x −14.5, y 11.5..13.5 里取 12.0±1）→ FAIL(BLOCK)；
                同一块料、mode=window（不查 B、W 的 |y| 只到 10.5）→ PASS。模式驱动的差别就是"查的是声明、不是刀"。
  · unknown   ：dims.plug_top_x_mm 删掉 → KO01:keepout_dims_declared unknown，不许绿。
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
from _harness import FakeCtx, base_data, findings, assert_red, assert_green, result, record  # noqa: E402

PART = "CARR"
NAME = "KO01 让位实体独立于 CAD 切刀：声明加深通窗里的料必须红；pocket 侧出线槽里的料必须红而 window 模式不查；dims 缺 → unknown"
EXPECT = ("bad_deep: L3/CARR:keepout_KO01 FAIL(BLOCK) ≈64 mm³ 且 KO01:keepout_intersection FAIL；good: keepout_intersection PASS；"
          "bad_side: pocket 模式 keepout_intersection FAIL(BLOCK)；同料 window 模式 PASS；unknown: KO01:keepout_dims_declared unknown")
_TMP = core.ROOT / "tools/gate/out/_neg_tmp"
SERVO = ("trunk_base", 0)


def _world_box(size, local_center):
    """舵机局部系里的方块 → 世界。"""
    import numpy as np
    import trimesh
    from duckstructure.lib import sfw
    R = np.asarray(sfw(*SERVO), float)
    m = trimesh.creation.box(extents=size)
    m.apply_translation(local_center)
    m.apply_transform(R)
    return m


def _placed_dir(tag, mesh):
    d = _TMP / f"p{__import__('os').getpid()}" / f"l3_ko01_{tag}"
    d.mkdir(parents=True, exist_ok=True)
    for p in d.glob("*.stl"):
        p.unlink()
    path = d / "carrier.stl"
    mesh.export(str(path), file_type="stl")
    return d, path


def _data(window_x=-18.5, mode="window", drop_dims=False):
    d = base_data([{"id": PART, "inventory_id": "CARR_carrier", "material": "PLA", "print_orientation": "任意",
                    "mirrored_copy": None, "qty": 1, "placed_instances": ["carrier"]}], [])
    ko01 = copy.deepcopy(next(k for k in d["keepouts"]["keepouts"] if k["id"] == "KO01"))
    ko01["availability_table"] = {"rows": [{"servo": f"{SERVO[0]}[{SERVO[1]}]", "mode_pos_y": "none", "mode_neg_y": mode}]}
    assert ko01["dims"]["window_x_mm"]["v"] == -18.5, ko01["dims"]["window_x_mm"]
    ko01["dims"]["window_x_mm"] = {"v": window_x, "src": "assumed"}
    if drop_dims:
        ko01["dims"].pop("plug_top_x_mm", None)
    d["keepouts"] = {"keepouts": [ko01]}
    d["components"] = {"components": []}
    d["frozen"] = {}
    return d


def _observe(tag, mesh, **kw):
    import l3_static as L3
    placed, path = _placed_dir(tag, mesh)
    old = L3.PLACED
    try:
        L3.PLACED = placed
        return L3.run(FakeCtx(_data(**kw), {PART: path}, {p.stem: p for p in placed.glob("*.stl")}))
    finally:
        L3.PLACED = old


def run() -> dict:
    tol = _data()["tolerances"]["feature_check_tolerances"]["static_intersection_mm3"]["max"]
    deep = _world_box((4.0, 4.0, 4.0), (-22.0, -7.75, -9.3))        # 舵机局部 x −24..−20：CAD 通窗（到 −18.5）够不到，声明通窗（到 −25）盖得到
    r_bad = _observe("deep_bad", deep, window_x=-25.0)
    r_good = _observe("deep_good", deep, window_x=-18.5)
    ok_bad, why_bad, red_bad = assert_red(findings(r_bad, subject=PART, check="keepout_KO01"), severity="BLOCK")
    ok_bad2, why_bad2, red_bad2 = assert_red(findings(r_bad, subject="KO01", check="keepout_intersection"), severity="BLOCK")
    vol = float(red_bad[0]["measured"]) if red_bad else None
    vol_ok = vol is not None and 50.0 < vol < 70.0                     # 4×4×4 = 64 全在通窗里
    ok_good, why_good = assert_green(findings(r_good, subject="KO01", check="keepout_intersection"))
    xchk = findings(r_bad, subject="KO01", check="keepout_KO01_vs_cad_cutter")
    x_ok = bool(xchk) and xchk[0].severity == "INFO" and xchk[0].measured is not None and float(xchk[0].measured) > 50.0   # 独立−CAD ≈ 通窗多出的 6.5 mm 深

    # 侧出线槽：pocket 模式查 B（|y| 10..13.3），window 模式不查 B 且 W 的 |y| 只到 10.5 → 同一块料一红一绿
    side = _world_box((1.6, 2.0, 4.0), (-14.5, -12.0, -9.3))         # y −13..−11，x −15.3..−13.7
    r_side = _observe("side_pocket", side, mode="pocket")
    r_sidew = _observe("side_window", side, mode="window")
    ok_side, why_side, red_side = assert_red(findings(r_side, subject="KO01", check="keepout_intersection"), severity="BLOCK")
    ok_sidew, why_sidew = assert_green(findings(r_sidew, subject="KO01", check="keepout_intersection"))

    r_unk = _observe("unk", deep, window_x=-25.0, drop_dims=True)
    unk = findings(r_unk, subject="KO01", check="keepout_dims_declared")
    unk_ok = bool(unk) and unk[0].state == "FAIL" and unk[0].measured is None \
        and not any(f.state == "PASS" for f in findings(r_unk, subject="KO01", check="keepout_intersection"))

    passed = ok_bad and ok_bad2 and vol_ok and ok_good and x_ok and ok_side and ok_sidew and unk_ok
    return result(NAME, EXPECT, passed,
                  got=(f"加深通窗里的料：件格 {'红' if ok_bad else why_bad}（{vol} mm³，阈值 {tol}），KO01 格 {'红' if ok_bad2 else why_bad2}；"
                       f"通窗 −18.5 对照 {'绿' if ok_good else why_good}；对照 INFO 独立−CAD={xchk[0].measured if xchk else None}；"
                       f"侧出线槽里的料 pocket {'红' if ok_side else why_side} / window {'绿' if ok_sidew else why_sidew}；"
                       f"dims 缺 → {'unknown' if unk_ok else '未 unknown'}"),
                  red=red_bad + red_bad2 + red_side, expect_severity="BLOCK",
                  detail=f"unknown={[record(f) for f in unk]}；side_window={[record(f) for f in findings(r_sidew, subject='KO01')]}")


if __name__ == "__main__":
    print(json.dumps(run(), ensure_ascii=False, indent=1))
