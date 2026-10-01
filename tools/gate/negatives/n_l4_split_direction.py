#!/usr/bin/env python3
"""反例 FG10 —— 两组障碍各自独立选方向，合起来无路可走也能全绿。

坏样本（**纯内存解析盒，不碰 cad/duck_s288 的整鸭件**，坐标抄 docs/gate/体系审查_Codex01.md:44）：
    移动件 MV    1 mm 立方   x/y/z ∈ [−0.5, 0.5]
    本步障碍 OBL             x ∈ [ 1.5,  2.5]，y/z ∈ [−1, 1]   （挡住 +x）
    已装障碍 OBI             x ∈ [−2.5, −1.5]，y/z ∈ [−1, 1]   （挡住 −x）
    路程 4 mm / 步长 0.25 mm

修前（l4_assembly.py:455-458）：障碍分成 local（本步 parts）/ installed（already_installed）两组，
**每组各自取较好的一侧符号** —— step_sweep_local 取 −x 得 0 mm³/34 次，step_sweep_installed
取 +x 得 0 mm³/34 次，两条判据都绿；实际 ±x 全堵，一条路都走不了。

修后：判据只有一条 `step_sweep` —— 同一方向、同一符号、对**合并后**的障碍集合求交，
两个符号峰值都是 1 mm³ / 68 次布尔 → 红。

好样本（防止判据写成恒红）：把 OBL 挪到 y ∈ [3, 5]（只改这一个盒的 y），+x 就通了 → 必须仍然绿。

判据阈值取真实的 tools/gate/data/tolerances.yaml，本文件不写死任何数字判据。
"""
from __future__ import annotations
import sys
from pathlib import Path

_HERE = Path(__file__).resolve()
GATE = _HERE.parents[1]
for _p in (str(GATE), str(GATE.parents[1])):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import core                                                        # noqa: E402

NAME = "两组障碍各自选方向：合起来无路可走也全绿（FG10）"
EXPECT = "L4/step01:step_sweep 必须红（合并 local ∪ installed 的完整障碍集合）"
_TMP = core.ROOT / "tools/gate/out/_neg_tmp"


def _write(boxes, tag):
    """把解析盒写成 placed/ 那样的一目录 STL —— 元规则 1：层只认导出的文件。"""
    import numpy as np
    import trimesh
    d = _TMP / f"l4_split_{tag}"
    d.mkdir(parents=True, exist_ok=True)
    for p in d.glob("*.stl"):
        p.unlink()
    for name, (lo, hi) in boxes.items():
        lo = np.asarray(lo, float)
        hi = np.asarray(hi, float)
        m = trimesh.creation.box(extents=(hi - lo))
        m.apply_translation((lo + hi) / 2.0)
        m.export(str(d / f"{name}.stl"), file_type="stl")
    return d


def _ctx(placed_dir):
    from _harness import FakeCtx                                   # noqa: PLC0415
    data = core.load_data()                                        # 真公差表
    data["parts"] = {"parts": [
        {"id": "MV", "inventory_id": "mover", "build_fn": [], "qty": 1, "placed_instances": ["mover"]},
        {"id": "OBL", "inventory_id": "ob_local", "build_fn": [], "qty": 1, "placed_instances": ["ob_local"]},
        {"id": "OBI", "inventory_id": "ob_installed", "build_fn": [], "qty": 1, "placed_instances": ["ob_installed"]},
    ]}
    data["features"] = {"features": [], "frames": {"per_part": {}}}
    data["fasteners"] = {"fasteners": []}
    data["keepouts"] = {"keepouts": []}
    data["waivers"] = {"waivers": []}
    data["assembly"] = {"assembly_order": [{
        "step": 1,
        "action": "反例：把 MV 沿 x 送到位（本步件挡 +x、已装件挡 −x）",
        "parts": ["MV", "OBL"],
        "already_installed": ["OBI"],
        "direction": "+x",
        "path": {"kind": "linear", "len_mm": 4.0, "step_mm": 0.25},
        "verified": True,
        "verified_by": "反例自造",
    }], "disassembly_order": []}
    return FakeCtx(data, {}, {p.stem: p for p in sorted(placed_dir.glob("*.stl"))})


def _run_layer(boxes, tag):
    import layers.l4_assembly as L4                                # noqa: PLC0415
    placed = _write(boxes, tag)
    old = L4.PLACED
    try:
        L4.PLACED = placed                                         # 只换 placed 目录，层代码原样跑
        return L4.run(_ctx(placed)), L4
    finally:
        L4.PLACED = old


def _cell(res, subject):
    from _harness import findings, worst_state, summarize          # noqa: PLC0415
    fs = findings(res, subject=subject)
    return worst_state(fs), summarize(fs, n=8), fs


def run() -> dict:
    from _harness import findings, worst_state, assert_red, assert_green, result   # noqa: PLC0415
    bad = {"mover": ((-0.5, -0.5, -0.5), (0.5, 0.5, 0.5)),
           "ob_local": ((1.5, -1.0, -1.0), (2.5, 1.0, 1.0)),
           "ob_installed": ((-2.5, -1.0, -1.0), (-1.5, 1.0, 1.0))}
    good = dict(bad, ob_local=((1.5, 3.0, -1.0), (2.5, 5.0, 1.0)))         # 只改这一个盒的 y

    r_bad, _ = _run_layer(bad, "bad")
    r_good, _ = _run_layer(good, "good")

    fs_bad = findings(r_bad, subject="step01", check="step_sweep")
    fs_good = findings(r_good, subject="step01", check="step_sweep")
    st_bad = worst_state(fs_bad)
    grp = {f.check: (f.state, f.measured, f.severity)
           for f in findings(r_bad, subject="step01") if f.check.startswith("step_sweep_")}
    cell_bad, sum_bad, _ = _cell(r_bad, "step01")
    cell_good, sum_good, _ = _cell(r_good, "step01")

    ok_red, why_red, red = assert_red(fs_bad, severity="BLOCK", require_measured=True)
    ok_green, why_green = assert_green(fs_good)
    got = (f"坏样本 step_sweep={'红' if ok_red else why_red}（峰值={red[0]['measured'] if red else None}，格子 {cell_bad}）；"
           f"好样本 step_sweep={'绿' if ok_green else why_green}（格子 {cell_good}）；坏样本的分组诊断 {grp}")
    if st_bad == "ABSENT":
        got = ("坏样本里**根本没有 step_sweep 这条判据**（判据仍是分组的 = 未修）；"
               f"分组判据 {grp} —— 两组各自选方向都判绿，这就是 FG10 的假绿")
    return result(NAME, EXPECT, ok_red and ok_green, got=got, red=red, expect_severity="BLOCK",
                  detail=f"坏样本：{sum_bad} || 好样本（OBL 挪到 y∈[3,5]）：{sum_good}")


if __name__ == "__main__":
    import json
    r = run()
    print(json.dumps(r, ensure_ascii=False, indent=1))
    sys.exit(0 if r["passed"] else 1)
