#!/usr/bin/env python3
"""反例 hr41g —— KO22（电池顶插头收纳/后抽通道）这类"包络沿方向后抽"的禁入体要进正式 L3，且缺字段必须 unknown。

来源：hr39f 用独立脚本 check_service_2026-09-24.py 证明"零位、拆 B03+B01 后，电池+插头保守包络沿 −x 连续后抽 60 mm
与 75 个保留实体交集 0"，但 keepouts.yaml:KO22 没接进 L3（L3 对它只发 keepout_solid unknown）。独立报告不能冒充 Gate。
判据（l3_static._ko_bounds_sweep + run() 的禁入体循环）：
  扫掠体 = bounds_world_mm 包络盒沿 sweep_direction_world 平移 0..sweep_len_mm 的凸包；
  障碍 = placed/ 全部实体（打印件 + 元件）− moving_components；exempt[segment=all] 的件（拆卸态不在场）照算、记数、不计峰值；
  交集 > static_intersection_mm3 → keepout_intersection FAIL(BLOCK)；终点包络盒未证明脱离全部障碍 → keepout_sweep_clear_at_end FAIL(BLOCK)；
  声明了 moving_components / sweep_* 但方向、行程、moving 任一缺 → keepout_sweep_declared unknown（不猜）。

内存几何（世界系）：包络盒 x −10..0 / |y|≤2 / z 0..2，方向 −x。
  A 行程 30：BOX 在前方 x 5..8（不在路上）、DOOR 在路上但 exempt、zz_bat 在包络里（moving）→ keepout_intersection PASS、终点脱离 PASS
  B 行程 30：BOX 挪到路上 x −25..−20 → keepout_intersection FAIL(BLOCK) measured=交集体积，且 BOX 件格 keepout_KO22 BLOCK
  B2 行程 30：非打印件的元件 zz_other 在路上 → 同样 FAIL(BLOCK)（障碍含元件，不只打印件）
  C 缺 sweep_direction_world / 缺 sweep_len_mm / 真 KO22 现登记（两项都缺）→ keepout_sweep_declared FAIL unknown，且不发 keepout_intersection
  D 行程 5：TAIL 在更远的路上 x −20..−18 → 交集 0，但终点没越过 TAIL → keepout_sweep_clear_at_end FAIL(BLOCK)
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
from _harness import FakeCtx, box, findings, assert_red, assert_green, result, record, _TMP  # noqa: E402

NAME = "KO22 类后抽通道禁入体接入 L3：路上有料红（含元件）、行程不够红、字段缺 unknown、exempt 只记数"
EXPECT = ("A keepout_intersection/keepout_sweep_clear_at_end PASS；B/B2 keepout_intersection FAIL(BLOCK) 带体积、BOX 件格 keepout_KO22；"
          "C keepout_sweep_declared unknown 且无 keepout_intersection；D keepout_sweep_clear_at_end FAIL(BLOCK)")


def _ko(length=30.0, drop=()):
    k = {"id": "KO22", "name": "反例后抽通道", "owner_parts": "BOX",
         "bounds_world_mm": [[-10.0, -2.0, 0.0], [0.0, 2.0, 2.0]],
         "frame": "world_frozen_battery_bay",
         "sweep_direction_world": {"v": [-1.0, 0.0, 0.0], "src": "反例"},
         "sweep_len_mm": {"v": float(length), "src": "反例"},
         "moving_components": ["zz_bat"],
         "exempt": [{"part": "DOOR", "segment": "all", "why": "反例：拆卸态不在场"}],
         "status": "unchecked"}
    for d in drop:
        k.pop(d, None)
    return k


_REAL = None


def _real():
    """真数据只读一次（公差表等判据来自它）；每个场景深拷贝后只换 parts/features/fasteners/keepouts —— 同 _harness.base_data。"""
    global _REAL
    if _REAL is None:
        _REAL = core.load_data()
    return copy.deepcopy(_REAL)


def _real_ko22():
    # hr41 落盘 2026-09-25（主设计 Lane C）：真 KO22 已补 sweep_direction_world / sweep_len_mm（keepouts.yaml，hr41g 记录 §6）。
    #   c_real 这条验的是"现役登记两项都缺 → unknown 且两项都点名"，所以这里取真 KO22 再去掉这两项 = 2026-09-25 之前的现役登记原样，判据口径不变。
    k = copy.deepcopy(next(k for k in _real()["keepouts"]["keepouts"] if k["id"] == "KO22"))
    for key in ("sweep_direction_world", "sweep_len_mm"):
        k.pop(key, None)
    return k


def _observe(tag, parts_meshes, others, keepouts):
    import os
    import l3_static as L3
    d = _TMP / f"l3_ko22_{tag}_{os.getpid()}"
    d.mkdir(parents=True, exist_ok=True)
    for p in d.glob("*.stl"):
        p.unlink()
    stl, parts = {}, []
    for pid, m in parts_meshes.items():
        p = d / f"{pid.lower()}_body.stl"
        m.export(str(p), file_type="stl")
        stl[pid] = p
        parts.append({"id": pid, "inventory_id": f"{pid}_neg", "material": "PETG", "print_orientation": "任意",
                      "mirrored_copy": None, "qty": 1, "placed_instances": [f"{pid.lower()}_body"]})
    for name, m in others.items():                                  # 非打印件的 placed 实体（元件）
        m.export(str(d / f"{name}.stl"), file_type="stl")
    data = _real()
    data["parts"] = {"parts": parts}
    data["features"] = {"features": [], "frames": {"export_local": "反例：件自己的导出系"}}
    data["fasteners"] = {"fasteners": [], "known_issues": [], "counts": {}}
    data["waivers"] = {"waivers": []}
    data["components"] = {"components": []}
    data["keepouts"] = {"keepouts": keepouts}
    data["frozen"] = {}
    old = L3.PLACED
    try:
        L3.PLACED = d
        return L3.run(FakeCtx(data, stl, {p.stem: p for p in d.glob("*.stl")}))
    finally:
        L3.PLACED = old


def run() -> dict:
    asserts, reds = [], []
    door = box(3.0, 6.0, 4.0, center=(-13.5, 0.0, 1.0))            # 在路上，但 exempt（拆卸态不在场）
    bat = {"zz_bat": box(6.0, 3.0, 1.5, center=(-5.0, 0.0, 1.0))}   # moving：在包络里
    front = box(3.0, 4.0, 2.0, center=(6.5, 0.0, 1.0))              # 前方，不在路上

    ra = _observe("a", {"BOX": front, "DOOR": door}, bat, [_ko(30.0)])
    ia = findings(ra, subject="KO22", check="keepout_intersection")
    ea = findings(ra, subject="KO22", check="keepout_sweep_clear_at_end")
    ok_a, why_a = assert_green(ia)
    ok_ae, why_ae = assert_green(ea)
    asserts.append({"claim": "A keepout_intersection PASS", "ok": ok_a})
    asserts.append({"claim": "A exempt 的 DOOR 照算并记为『按声明排除』", "ok": bool(ia and "按声明排除" in ia[0].detail and "DOOR" in ia[0].detail)})
    asserts.append({"claim": "A keepout_sweep_clear_at_end PASS", "ok": ok_ae})

    rb = _observe("b", {"BOX": box(5.0, 2.0, 1.0, center=(-22.5, 0.0, 1.0)), "DOOR": door}, bat, [_ko(30.0)])
    ib = findings(rb, subject="KO22", check="keepout_intersection")
    pb = findings(rb, subject="BOX", check="keepout_KO22")
    ok_b, why_b, red_b = assert_red(ib, severity="BLOCK", require_measured=True)
    ok_bp, why_bp, red_bp = assert_red(pb, severity="BLOCK", require_measured=True)
    reds += red_b + red_bp
    asserts.append({"claim": "B 路上打印件 → keepout_intersection FAIL(BLOCK) ≈10 mm³", "ok": bool(ok_b and abs(float(ib[0].measured) - 10.0) < 1e-3)})
    asserts.append({"claim": "B BOX 件格 keepout_KO22 FAIL(BLOCK)", "ok": ok_bp})

    rb2 = _observe("b2", {"BOX": front, "DOOR": door},
                   dict(bat, zz_other=box(5.0, 2.0, 1.0, center=(-22.5, 0.0, 1.0))), [_ko(30.0)])
    ib2 = findings(rb2, subject="KO22", check="keepout_intersection")
    ok_b2, why_b2, red_b2 = assert_red(ib2, severity="BLOCK", require_measured=True)
    reds += red_b2
    asserts.append({"claim": "B2 路上的元件 zz_other → keepout_intersection FAIL(BLOCK)", "ok": bool(ok_b2 and "zz_other" in ib2[0].detail)})

    unk_ok = []
    for tag, kos in (("c_dir", [_ko(30.0, drop=("sweep_direction_world",))]),
                     ("c_len", [_ko(30.0, drop=("sweep_len_mm",))]),
                     ("c_real", [_real_ko22()])):
        rc = _observe(tag, {"BOX": front, "DOOR": door}, bat, kos)
        uc = findings(rc, subject="KO22", check="keepout_sweep_declared")
        good = (len(uc) == 1 and uc[0].state == "FAIL" and uc[0].severity == "BLOCK" and uc[0].measured is None
                and not findings(rc, subject="KO22", check="keepout_intersection"))
        if tag == "c_dir":
            good = good and "sweep_direction_world" in uc[0].detail
        if tag == "c_len":
            good = good and "sweep_len_mm" in uc[0].detail
        if tag == "c_real":
            good = good and "sweep_direction_world" in uc[0].detail and "sweep_len_mm" in uc[0].detail
        unk_ok.append((tag, good, [record(f) for f in uc]))
        asserts.append({"claim": f"C {tag} → keepout_sweep_declared unknown、无 keepout_intersection", "ok": good})

    rd = _observe("d", {"BOX": front, "DOOR": door, "TAIL": box(2.0, 4.0, 2.0, center=(-19.0, 0.0, 1.0))}, bat, [_ko(5.0)])
    idd = findings(rd, subject="KO22", check="keepout_intersection")
    ed = findings(rd, subject="KO22", check="keepout_sweep_clear_at_end")
    ok_d, why_d, red_d = assert_red(ed, severity="BLOCK", require_measured=True)
    reds += red_d
    ok_di, why_di = assert_green(idd)
    asserts.append({"claim": "D 行程 5：交集 0（PASS）", "ok": ok_di})
    asserts.append({"claim": "D 行程 5：keepout_sweep_clear_at_end FAIL(BLOCK)，点名 TAIL", "ok": bool(ok_d and "tail_body" in ed[0].detail)})

    passed = all(a["ok"] for a in asserts)
    got = "；".join(f"{'✓' if a['ok'] else '✗'}{a['claim']}" for a in asserts)
    out = result(NAME, EXPECT, passed, got, reds, expect_severity="BLOCK",
                 detail=(f"A={[record(f) for f in ia + ea]} {why_a}{why_ae} | B={[record(f) for f in ib + pb]} {why_b}{why_bp} | "
                         f"B2={[record(f) for f in ib2]} {why_b2} | C={unk_ok} | D={[record(f) for f in idd + ed]} {why_d}{why_di} | "
                         f"C 以外全部 KO22 结果（A）：{[record(f) for f in findings(ra, subject='KO22')]}"))
    out["assertions"] = asserts
    return out


if __name__ == "__main__":
    r = run()
    print(json.dumps(r, ensure_ascii=False, indent=1))
    sys.exit(0 if r["passed"] else 1)
