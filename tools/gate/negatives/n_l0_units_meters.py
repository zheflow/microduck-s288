#!/usr/bin/env python3
"""反例 README N7/N8 —— 单位错成米 / Y-up 导出。

经过 `l0_mesh.run(ctx)`，同一块 40×30×20 的解析盒：
  N7 坏   整体缩放 0.001（trimesh 单位歧义的典型后果）→ L0/<件>:units_and_scale FAIL(BLOCK)，
          measured = 最大外形 0.04 mm（判据 ∈ [3, 300] mm，取自层）
  对照    正常尺寸 → units_and_scale PASS
  N8      绕 X 转 −90°（Y-up）：L0 **现在抓不到** —— 包围盒量级不变，逐件基准 vs MJCF（datum_vs_mjcf）
          尚未实现、恒为 unknown。这一例只作"已知抓不到"记录（assertions 里 ok=True 但 claim 写明），
          不假装它红；等 datum_vs_mjcf 实现后把它改成 assert_red。
本文件同时给出 red（N7 的记录）与 assertions（含 N8 的已知缺口）——runner 以 assertions 判定，
N7 的 assert_red 结果也作为一条 assertion 进入，缺一不可。
"""
from __future__ import annotations
import json
import sys
from pathlib import Path

_HERE = Path(__file__).resolve()
for _p in (str(_HERE.parent), str(_HERE.parents[3])):          # negatives/、仓库根（duckstructure.lib）
    if _p not in sys.path:
        sys.path.insert(0, _p)
from _harness import FakeCtx, base_data, box, export_stl, findings, assert_red, assert_green, summarize  # noqa: E402

PART = "T02"
NAME = "STL 缩放 0.001（米当毫米）：units_and_scale 必须红；正常尺寸绿；Y-up 目前是已知抓不到"
EXPECT = f"L0/{PART}:units_and_scale FAIL(BLOCK)，measured ≈ 0.04；对照 PASS；Y-up：datum_vs_mjcf 仍 unknown（已知缺口）"


def _mesh(kind: str):
    import numpy as np
    import trimesh
    m = box(40.0, 30.0, 20.0, center=(0.0, 0.0, 10.0))
    if kind == "meters":
        m.apply_scale(0.001)
    elif kind == "yup":
        m.apply_transform(trimesh.transformations.rotation_matrix(-np.pi / 2, [1, 0, 0]))
    return m


KINDS = {"meters": PART, "normal": PART + "b", "yup": PART + "c"}     # 三个样本作三个件、一次 run


def _data():
    parts = [{"id": pid, "inventory_id": f"{pid}_neg_units_{kind}", "material": "PLA",
              "print_orientation": "任意", "mirrored_copy": None} for kind, pid in KINDS.items()]
    return base_data(parts, [])


def _observe():
    """一次 run 三个件：整机级判据（assembly_preview / 发布副本 / 冻结基线 / kin）只跑一遍，< 5 s。"""
    import l0_mesh
    stl_map = {pid: export_stl(_mesh(kind), f"l0_units_{kind}") for kind, pid in KINDS.items()}
    return l0_mesh.run(FakeCtx(_data(), stl_map))


def run() -> dict:
    res = _observe()
    r_bad = r_good = r_yup = res
    ok_red, why_red, red = assert_red(findings(r_bad, subject=KINDS["meters"], check="units_and_scale"), severity="BLOCK")
    ok_green, why_green = assert_green(findings(r_good, subject=KINDS["normal"], check="units_and_scale"))
    yup_scale = findings(r_yup, subject=KINDS["yup"], check="units_and_scale")
    datum = findings(r_yup, subject="_datum", check="datum_vs_mjcf")
    # Y-up：包围盒判据过（量级没变），datum_vs_mjcf 恒 unknown → 现在抓不到朝向错
    yup_not_caught = (bool(yup_scale) and yup_scale[0].state == "PASS"
                      and bool(datum) and datum[0].state == "FAIL" and datum[0].measured is None)
    assertions = [
        {"claim": "缩放 0.001 → units_and_scale FAIL(BLOCK) 且 measured 非空" + ("" if ok_red else f"（{why_red}）"),
         "ok": ok_red},
        {"claim": "缩放后 measured < 3 mm（判据下限来自层）",
         "ok": ok_red and float(red[0]["measured"]) < 3.0},
        {"claim": "正常尺寸 → units_and_scale PASS" + ("" if ok_green else f"（{why_green}）"), "ok": ok_green},
        {"claim": "【已知抓不到】Y-up（绕 X −90°）：L0 目前不判朝向，units_and_scale 照样 PASS，"
                  "datum_vs_mjcf 仍是 unknown（未实现）—— 这条记录的是缺口，不是红；"
                  "datum_vs_mjcf 实现后必须改成 assert_red",
         "ok": yup_not_caught},
    ]
    passed = all(a["ok"] for a in assertions)
    return {"name": NAME, "passed": passed, "expect": EXPECT,
            "got": (f"米样本 {'红' if ok_red else why_red}（measured={red[0]['measured'] if red else None}）；"
                    f"正常 {'绿' if ok_green else why_green}；"
                    f"Y-up：units_and_scale={yup_scale[0].state if yup_scale else 'ABSENT'}，"
                    f"datum_vs_mjcf={datum[0].state if datum else 'ABSENT'}/measured="
                    f"{datum[0].measured if datum else '-'}（已知抓不到）"),
            "expect_severity": "BLOCK", "red": red, "assertions": assertions,
            "detail": f"米样本：{summarize(findings(r_bad, subject=PART), n=6)}"}


if __name__ == "__main__":
    print(json.dumps(run(), ensure_ascii=False, indent=1))
