#!/usr/bin/env python3
"""反例 F-L3-3（2026-09-13）—— 元件实体不能只有与 CAD 同源的 placed 占位体。

历史：L3 "打印件 × 元件完整实体" 查的是 build 导出的 placed 占位体（servo__* / bearing_* / zz_battery）。载体口袋是
servo_env 挖出来的，占位体自己被挖过缺口的话（或者占位体本来就比 datasheet 小），载体 × 占位体 = 0 是构造保证。
修法：components.yaml:envelope_solid 声明环/盒，层按声明独立建实体（落位只借 placed 的 OBB），再查一遍。

场景：
  · placed 舵机占位体 = 34×20×25.7 方块（与真占位体外形一致，claim.by=placed_prefix servo__）**中间挖一个 3×3×3 缺口**；
    载体 CARR = 2×2×2 小筋正好坐在缺口里。
  · bad ：envelope_solid = box 34×20×19.9（datasheet 薄段）→ 独立实体盖住缺口 → part_vs_component_envelope FAIL(BLOCK) = 8 mm³、
          CARR:component_envelope_servo_s288 FAIL；而同源的 part_vs_component_solid 仍是 0（PASS）—— 这就是审计说的假绿。
  · good：占位体不挖缺口、筋挪到外面 → 两条都 PASS，placed_covers_envelope PASS（OBB 34×20×25.7 ⊇ 34×20×19.9）。
  · unknown：不声明 envelope_solid → servo_s288:envelope_solid_declared unknown（FAIL, measured=None），不许绿；
  · ambiguous：声明了 size 但没有 overall_mm，T19.9 与 W20 差 0.1 按 OBB 长度分不出轴 → 同样 unknown（不猜轴：猜错 0.1 就是 1.4 mm³ 的假红）。
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
from _harness import FakeCtx, base_data, box, findings, assert_red, assert_green, result, record  # noqa: E402

PART, COMP = "CARR", "servo_s288"
NAME = "元件独立实体：占位体挖了缺口、载体筋坐在缺口里 → 同源判据 0 但独立实体判据必须红；不声明 envelope_solid → unknown"
EXPECT = (f"bad: L3/{COMP}:part_vs_component_envelope FAIL(BLOCK) 8 mm³ + {PART}:component_envelope_{COMP} FAIL，"
          f"同时 part_vs_component_solid PASS 0；good: 三条 PASS；unknown: envelope_solid_declared unknown")
_TMP = core.ROOT / "tools/gate/out/_neg_tmp"


def _placed_dir(tag, notch: bool, rib_inside: bool):
    import os
    d = _TMP / f"p{os.getpid()}" / f"l3_env_{tag}"
    d.mkdir(parents=True, exist_ok=True)
    for p in d.glob("*.stl"):
        p.unlink()
    servo = box(34.0, 20.0, 25.7)
    if notch:
        servo = servo.difference(box(3.0, 3.0, 3.0))
    servo.export(str(d / "servo__neg_body.stl"), file_type="stl")
    rib = box(2.0, 2.0, 2.0) if rib_inside else box(2.0, 2.0, 2.0, center=(0.0, 0.0, 40.0))
    rp = d / "carrier.stl"
    rib.export(str(rp), file_type="stl")
    return d, rp


def _data(declare: bool):
    d = base_data([{"id": PART, "inventory_id": "CARR_carrier", "material": "PLA", "print_orientation": "任意",
                    "mirrored_copy": None, "qty": 1, "placed_instances": ["carrier"]}], [])
    comp = {"id": COMP, "category": "servo", "qty": 1, "claim": {"by": "placed_prefix", "pattern": "servo__"},
            "envelope_mm": {"v": "L34 × W20 × T19.9(薄段)/22.8(厚段)", "src": "datasheet"}}
    if declare:
        comp["envelope_solid"] = {"shape": "box", "size_mm": [34.0, 20.0, 19.9], "overall_mm": [34.0, 20.0, 25.7], "exact": False, "src": "datasheet 薄段"}
        if declare == "no_overall":
            comp["envelope_solid"].pop("overall_mm")           # T19.9 与 W20 差 0.1：没有 overall 分不出轴 → 必须 unknown
    d["components"] = {"components": [comp]}
    d["keepouts"] = {"keepouts": []}
    d["frozen"] = {}
    return d


def _observe(tag, notch, rib_inside, declare=True):
    import l3_static as L3
    placed, rp = _placed_dir(tag, notch, rib_inside)
    old = L3.PLACED
    try:
        L3.PLACED = placed
        return L3.run(FakeCtx(_data(declare), {PART: rp}, {p.stem: p for p in placed.glob("*.stl")}))
    finally:
        L3.PLACED = old


def run() -> dict:
    r_bad = _observe("bad", notch=True, rib_inside=True)
    r_good = _observe("good", notch=False, rib_inside=False)
    r_unk = _observe("unk", notch=True, rib_inside=True, declare=False)
    r_amb = _observe("amb", notch=True, rib_inside=True, declare="no_overall")

    ok_env, why_env, red_env = assert_red(findings(r_bad, subject=COMP, check="part_vs_component_envelope"), severity="BLOCK")
    ok_cell, why_cell, red_cell = assert_red(findings(r_bad, subject=PART, check=f"component_envelope_{COMP}"), severity="BLOCK")
    vol = float(red_env[0]["measured"]) if red_env else None
    vol_ok = vol is not None and 7.5 < vol < 8.5
    same = findings(r_bad, subject=COMP, check="part_vs_component_solid")
    same_green = bool(same) and same[0].state == "PASS" and float(same[0].measured) == 0.0     # 同源判据看不见 —— 反例要证明的就是这个
    cov_bad = findings(r_bad, subject=COMP, check="placed_covers_envelope")
    cov_ok = bool(cov_bad) and cov_bad[0].state == "PASS"                                     # 缺口不改 OBB，包络仍被盖住

    ok_g1, why_g1 = assert_green(findings(r_good, subject=COMP, check="part_vs_component_envelope"))
    ok_g2, why_g2 = assert_green(findings(r_good, subject=COMP, check="part_vs_component_solid"))
    ok_g3, why_g3 = assert_green(findings(r_good, subject=COMP, check="placed_covers_envelope"))

    unk = findings(r_unk, subject=COMP, check="envelope_solid_declared")
    unk_ok = (bool(unk) and unk[0].state == "FAIL" and unk[0].measured is None
              and not findings(r_unk, subject=COMP, check="part_vs_component_envelope"))

    amb = findings(r_amb, subject=COMP, check="envelope_solid_declared")
    amb_ok = (bool(amb) and amb[0].state == "FAIL" and amb[0].measured is None and "落不到轴" in str(amb[0].detail)
              and not findings(r_amb, subject=COMP, check="part_vs_component_envelope"))
    passed = ok_env and ok_cell and vol_ok and same_green and cov_ok and ok_g1 and ok_g2 and ok_g3 and unk_ok and amb_ok
    return result(NAME, EXPECT, passed,
                  got=(f"bad: 独立实体 {'红' if ok_env else why_env}（{vol} mm³）、件格 {'红' if ok_cell else why_cell}、"
                       f"同源判据 {'0 绿（假绿，被独立实体抓住）' if same_green else '不是 0 绿'}、包络覆盖 {'PASS' if cov_ok else '非 PASS'}；"
                       f"good: {'绿' if ok_g1 else why_g1} / {'绿' if ok_g2 else why_g2} / {'绿' if ok_g3 else why_g3}；"
                       f"unknown: {'unknown' if unk_ok else '未 unknown'}；无 overall（T19.9 vs W20 分不出轴）: {'unknown' if amb_ok else '未 unknown'}"),
                  red=red_env + red_cell, expect_severity="BLOCK",
                  detail=f"same={[record(f) for f in same]}；unk={[record(f) for f in unk]}；cov={[record(f) for f in cov_bad]}")


if __name__ == "__main__":
    print(json.dumps(run(), ensure_ascii=False, indent=1))
