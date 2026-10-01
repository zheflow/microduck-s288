#!/usr/bin/env python3
"""反例 —— 删右件、重复左件、跨件占用同一实体，不能缩小装配检查全集。

经过 `l4_assembly.run(ctx)`：placed/ 里有 part.stl 与 part_R.stl（两只解析盒），parts.yaml 说 P 的 qty=2。
  坏 A  placed_instances=['part']            少一个实例       → L4/_framework:placed_instances FAIL(BLOCK)
  坏 B  placed_instances=['part','part']     重复             → 同上
  坏 C  P 与 Q 都占用 'part'                 跨件重复         → 同上
  好    placed_instances=['part','part_R']   与 qty 一致      → PASS，且 ctx.placed_for 稳定选到左件
measured 是错误清单（非空 list），所以不是 unknown。
"""
from __future__ import annotations
import json
import sys
from pathlib import Path

_HERE = Path(__file__).resolve()
sys.path.insert(0, str(_HERE.parent))
from _harness import FakeCtx, core, findings, assert_red, assert_green, result, summarize  # noqa: E402

NAME = "打印件实例与 BOM 数量一致：缺实例/重复/跨件占用 → placed_instances 红；正确清单绿且选左件"
EXPECT = "L4/_framework:placed_instances FAIL(BLOCK)（坏 A/B/C）；好样本 _framework 全绿"
_TMP = core.ROOT / "tools/gate/out/_neg_tmp"


def _scene():
    import trimesh
    d = _TMP / "l4_placed_instances"
    d.mkdir(parents=True, exist_ok=True)
    for p in d.glob("*.stl"):
        p.unlink()
    for name, x in (("part", -5.0), ("part_R", 5.0)):
        m = trimesh.creation.box(extents=(2.0, 2.0, 2.0))
        m.apply_translation([x, 0.0, 0.0])
        m.export(str(d / f"{name}.stl"), file_type="stl")
    return d


def _ctx(placed, parts):
    data = core.load_data()
    data["parts"] = {"parts": parts}
    data["features"] = {"features": [], "frames": {"per_part": {}}}
    data["fasteners"] = {"fasteners": []}
    data["keepouts"] = {"keepouts": []}
    data["waivers"] = {"waivers": []}
    # 层在 assembly_order 为空时直接返回，走不到 placed_instances 那条；给一条 path 为空的占位步
    data["assembly"] = {"assembly_order": [{
        "step": 1, "action": "反例占位：只验 placed_instances", "parts": [p["id"] for p in parts],
        "already_installed": [], "direction": "+x", "path": {}, "verified": False}],
        "disassembly_order": []}
    return FakeCtx(data, {}, {p.stem: p for p in sorted(placed.glob("*.stl"))})


def _observe(placed, parts):
    import layers.l4_assembly as L4
    old = L4.PLACED
    try:
        L4.PLACED = placed
        return L4.run(_ctx(placed, parts))
    finally:
        L4.PLACED = old


def run() -> dict:
    placed = _scene()
    P = {"id": "P", "inventory_id": "part", "build_fn": [], "qty": 2, "placed_instances": ["part", "part_R"]}
    cases = {
        "A缺实例": [{**P, "placed_instances": ["part"]}],
        "B重复": [{**P, "placed_instances": ["part", "part"]}],
        "C跨件占用": [P, {"id": "Q", "inventory_id": "part", "build_fn": [], "qty": 1, "placed_instances": ["part"]}],
    }
    reds, got = [], []
    ok_all = True
    for tag, parts in cases.items():
        res = _observe(placed, parts)
        ok, why, rec = assert_red(findings(res, subject="_framework", check="placed_instances"), severity="BLOCK")
        ok_all &= ok
        reds += rec
        got.append(f"{tag}:{'红' if ok else '未红(' + why + ')'}")
    res_good = _observe(placed, [P])
    ok_g, why_g = assert_green(findings(res_good, subject="_framework"))
    got.append(f"好样本 _framework:{'绿' if ok_g else '不绿(' + why_g + ')'}")
    return result(NAME, EXPECT, ok_all and ok_g, got="；".join(got), red=reds, expect_severity="BLOCK",
                  detail="好样本 _framework: " + summarize(findings(res_good, subject="_framework"), n=4))


if __name__ == "__main__":
    print(json.dumps(run(), ensure_ascii=False, indent=1))
