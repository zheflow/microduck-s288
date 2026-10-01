#!/usr/bin/env python3
"""反例（09-12 第二批）—— 切片解析：Arachne 双 bead 墙不许被当成单圈；真单 bead 必须抓到；层间 ;TYPE: 不重发不许丢层。

I1 调查发现两处漏：
  1. PrusaSlicer 层间类型不变时不重发 ;TYPE:，旧 parse_gcode 每隔几层把有墙的层记成"没墙"（L03 142/74 实为 169/101）。
  2. Arachne 把 2 条 bead 的墙（0.9..1.17 mm）也只标 External perimeter，"整层只有 External" ≠ 单圈。
本反例不依赖切片器：手写 G-code + 内存网格喂 slice_l1.parse_gcode：
  · 方管 外 10×10 内 9×9（壁 0.5），3 层。截面周长 = 40 + 36 = 76。
  · A "单 bead"：每层一条 9.5×9.5 中线环（38 mm）→ 比值 0.5 → layers_single_bead = 3
  · B "双 bead"：每层两条环（外 9.75、内 9.25，共 76 mm）→ 比值 1.0 → layers_single_bead = 0
  · C 同 B 但只有第 1 层写 ;TYPE:（后两层不重发）→ layers_with_perimeter 仍必须是 3（修正 1）
  · D 层判据：l1_printable 对 layers_single_bead=3 必须红、=0 必须绿、缺字段（旧记录）必须 unknown
"""
from __future__ import annotations
import sys
from pathlib import Path

_HERE = Path(__file__).resolve()
GATE = _HERE.parents[1]
for _p in (str(GATE), str(GATE / "layers"), str(GATE / "slicing")):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import core  # noqa: E402

NAME = "切片解析：Arachne 双 bead 不算单圈、真单 bead 必须抓到、;TYPE: 不重发不丢层"
EXPECT = "A 单 bead=3；B 双 bead=0；C 不重发 TYPE 仍 3 层有墙；层判据 红/绿/unknown"


def _gcode(loops_per_layer, retype_every_layer=True, n_layers=3):
    """方管三层。loops_per_layer: [(半边长,...)]，每层各画这些正方形环（中心 (100,100)）。"""
    out = ["M82", "G92 E0"]
    e = 0.0
    for k in range(n_layers):
        z = 0.2 * (k + 1)
        out += [";LAYER_CHANGE", f";Z:{z}", f";HEIGHT:0.2", f"G1 Z{z} F600"]
        if retype_every_layer or k == 0:
            out.append(";TYPE:External perimeter")
        for half in loops_per_layer:
            c = 100.0
            pts = [(c - half, c - half), (c + half, c - half), (c + half, c + half), (c - half, c + half), (c - half, c - half)]
            out.append(f"G1 X{pts[0][0]:.3f} Y{pts[0][1]:.3f} F3000")          # 空移
            for (x, y) in pts[1:]:
                e += 0.1
                out.append(f"G1 X{x:.3f} Y{y:.3f} E{e:.4f}")
    return "\n".join(out) + "\n"


def _tube():
    import trimesh
    outer = trimesh.creation.box(extents=(10.0, 10.0, 0.6)); outer.apply_translation((100.0, 100.0, 0.3))
    inner = trimesh.creation.box(extents=(9.0, 9.0, 2.0)); inner.apply_translation((100.0, 100.0, 0.3))
    return trimesh.boolean.difference([outer, inner])


def run() -> dict:
    import slice_l1
    import l1_printable
    from _harness import FakeCtx, base_data, export_stl, findings, worst_state, assert_red, assert_green, result, record  # noqa: PLC0415
    tube = _tube()
    A = slice_l1.parse_gcode(_gcode([4.75]), mesh=tube)
    B = slice_l1.parse_gcode(_gcode([4.875, 4.625]), mesh=tube)
    C = slice_l1.parse_gcode(_gcode([4.875, 4.625], retype_every_layer=False), mesh=tube)
    ok_parse = (A["layers_single_bead"] == 3 and B["layers_single_bead"] == 0
                and C["layers_with_perimeter"] == 3 and C["layers_single_bead"] == 0
                and A["layers_external_perimeter_only"] == 3 and B["layers_external_perimeter_only"] == 3)
    # 层判据
    part = {"id": "L03", "inventory_id": "L03_neg_bead", "material": "PLA", "print_orientation": "底面朝下",
            "print_orientation_down_normal": [0, 0, -1], "print_orientation_frame": "export_local", "mirrored_copy": None}
    stl = export_stl(tube, "l1_bead_tube")
    states, fs_of = {}, {}
    for tag, rec in (("bad", dict(A)), ("good", dict(B)), ("old", {k: v for k, v in B.items() if k != "layers_single_bead"})):
        data = base_data([part], [])
        srun = data.setdefault("printability", {}).setdefault("slice_run", {})
        srun.setdefault("parts", {})["L03"] = {"source_sha256": core.sha256_file(stl), "returncode": 0, **rec}
        res = l1_printable.run(FakeCtx(data, {"L03": stl}))
        fs = findings(res, subject="L03", check="min_wall_sliced")
        fs_of[tag] = fs
        states[tag] = (worst_state(fs), (fs[0].detail if fs else "")[:80])
    # 坏：单 bead 3 层 → min_wall_sliced FAIL(BLOCK)，measured 非空（量到了 3 层）
    ok_red, why_red, red = assert_red(fs_of["bad"], severity="BLOCK", require_measured=True)
    ok_green, why_green = assert_green(fs_of["good"])
    # 旧记录缺字段 → unknown（FAIL、measured=None），单独记录，不进 red
    old = fs_of["old"]
    ok_old = bool(old) and old[0].state == "FAIL" and old[0].measured is None and "缺 layers_single_bead" in states["old"][1]
    return result(NAME, EXPECT, ok_parse and ok_red and ok_green and ok_old,
                  got=(f"A 单bead={A['layers_single_bead']} ratios={A['single_bead_ratios_ext_only_layers']}；"
                       f"B 双bead={B['layers_single_bead']} ratios={B['single_bead_ratios_ext_only_layers']}；"
                       f"C 不重发TYPE 有墙层={C['layers_with_perimeter']}；"
                       f"层判据：单 bead {'红' if ok_red else why_red}，双 bead {'绿' if ok_green else why_green}，"
                       f"旧记录 {'unknown' if ok_old else '未 unknown'}；解析 {'OK' if ok_parse else '错'}"),
                  red=red, expect_severity="BLOCK",
                  detail=f"层判据 {states}；旧记录={[record(f) for f in old]}")


if __name__ == "__main__":
    import json
    r = run()
    print(json.dumps(r, ensure_ascii=False, indent=1))
    sys.exit(0 if r["passed"] else 1)
