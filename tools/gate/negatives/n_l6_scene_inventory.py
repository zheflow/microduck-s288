#!/usr/bin/env python3
"""反例 —— 运动场景不能漏掉新增轴承/打印件：实体缺失、多出、错误连杆、旧快照、空场景都必须拒绝。

经过 `l6_motion.run(ctx)`：placed/ 换成一个**同名替身目录** —— parts.yaml:motion_instances 声明的每个 stem
各放一只 1 mm 立方（几何不重要，本反例只验清单），层读真 cad_geometry_manifest.json 快照与 duckstructure.lib.B。
红必须来自 run 发出的 `L6/_scene:placed_inventory`（FAIL(BLOCK)，measured 是 {missing, extra, ...} 字典，非 None）：
  坏 A  删掉一只轴承实体          → missing
  坏 B  多出一只没人认领的实体    → extra
  坏 C  motion_instances 把某实体挂到不存在的连杆 → unknown_bodies
  坏 D  motion_instances 把某实体挂到别的连杆（与快照不一致）→ snapshot_mismatch
  坏 E  空目录                     → 层在 placed_dir 那一步就 unknown（另记，不进 red）
  好    完整清单                   → placed_inventory PASS
清单判定之后层会去建整机场景（_Scene）；本反例把 _Scene 换成立即抛错的桩，让 run 在 `_layer:scene` unknown 处
停下 —— 只验清单这一步，运动交集另有反例。
"""
from __future__ import annotations
import json
import sys
from copy import deepcopy
from pathlib import Path

_HERE = Path(__file__).resolve()
GATE = _HERE.parents[1]
for _p in (str(_HERE.parent), str(GATE), str(GATE / "layers"), str(GATE.parents[1]), str(GATE.parents[1] / "tools/sim")):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import core                                                                      # noqa: E402
from _harness import FakeCtx, findings, assert_red, assert_green, result, record  # noqa: E402

NAME = "运动场景清单：缺实体 / 多实体 / 错连杆 / 旧快照 都必须红（_scene:placed_inventory），完整清单绿"
EXPECT = "L6/_scene:placed_inventory FAIL(BLOCK)（measured 列出 missing/extra/unknown_bodies/snapshot_mismatch）；完整清单 PASS"
_TMP = core.ROOT / "tools/gate/out/_neg_tmp"


def stems_declared(data):
    """parts.yaml:motion_instances 声明的实体名 → 连杆（层用的就是它）。"""
    return dict((data.get("parts") or {}).get("motion_instances") or {})


def make_dummy_placed(tag, stems, extra=()):
    """每个 stem 一只 1 mm 立方，沿 x 每 10 mm 一只（互不相交）。"""
    import trimesh
    d = _TMP / f"l6_dummy_{tag}"
    d.mkdir(parents=True, exist_ok=True)
    for p in d.glob("*.stl"):
        p.unlink()
    for i, s in enumerate(sorted(list(stems) + list(extra))):
        m = trimesh.creation.box(extents=(1.0, 1.0, 1.0))
        m.apply_translation([10.0 * i, 0.0, 0.0])
        m.export(str(d / f"{s}.stl"), file_type="stl")
    return d


class _NoScene:
    """桩：清单过了就停 —— 本反例不建整机场景（那是运动交集反例的事）。"""
    def __init__(self, *a, **k):
        raise RuntimeError("反例 n_l6_scene_inventory：只验清单，不建场景")


def _run(placed_dir, data):
    import l6_motion as L6
    old_placed, old_scene = L6.PLACED, L6._Scene
    try:
        L6.PLACED = placed_dir
        L6._Scene = _NoScene
        return L6.run(FakeCtx(data, {}, {p.stem: p for p in placed_dir.glob("*.stl")}))
    finally:
        L6.PLACED, L6._Scene = old_placed, old_scene


def run() -> dict:
    base = core.load_data()
    decl = stems_declared(base)
    if not decl:
        return result(NAME, EXPECT, False, got="parts.yaml 没有 motion_instances，反例建不起来", red=[])
    bearing = next((s for s in sorted(decl) if s.startswith("bearing")), sorted(decl)[0])
    some = sorted(decl)[0]
    other_body = next(b for b in sorted(set(decl.values())) if b != decl[some])

    cases = {}
    # A 缺实体
    cases["A缺实体"] = (make_dummy_placed("missing", [s for s in decl if s != bearing]), base)
    # B 多实体
    cases["B多实体"] = (make_dummy_placed("extra", decl, extra=["neg_extra_solid"]), base)
    # C 错连杆（不存在的 body）
    dC = deepcopy(base); dC["parts"]["motion_instances"][some] = "no_such_body"
    cases["C错连杆"] = (make_dummy_placed("badbody", decl), dC)
    # D 与快照不一致（挂到别的合法连杆）
    dD = deepcopy(base); dD["parts"]["motion_instances"][some] = other_body
    cases["D旧快照"] = (make_dummy_placed("snapshot", decl), dD)

    reds, got, ok_all = [], [], True
    for tag, (pd, data) in cases.items():
        res = _run(pd, data)
        fs = findings(res, subject="_scene", check="placed_inventory")
        ok, why, rec = assert_red(fs, severity="BLOCK", require_measured=True)
        ok_all &= ok
        reds += rec
        got.append(f"{tag}:{'红' if ok else why}")
    # E 空目录：层在 placed_dir 那一步 unknown
    empty = _TMP / "l6_dummy_empty"; empty.mkdir(parents=True, exist_ok=True)
    for p in empty.glob("*.stl"):
        p.unlink()
    res_e = _run(empty, base)
    fe = findings(res_e, subject="_layer", check="placed_dir")
    ok_e = bool(fe) and fe[0].state == "FAIL"
    got.append(f"E空目录:{'unknown' if ok_e else '未 unknown'}")
    # 好样本
    res_g = _run(make_dummy_placed("good", decl), base)
    ok_g, why_g = assert_green(findings(res_g, subject="_scene", check="placed_inventory"))
    got.append(f"完整清单:{'绿' if ok_g else why_g}")
    return result(NAME, EXPECT, ok_all and ok_e and ok_g, got="；".join(got), red=reds, expect_severity="BLOCK",
                  detail=f"{len(decl)} 个声明实体；坏样本 measured={[r['measured'] for r in reds]}；空目录={[record(f) for f in fe]}")


if __name__ == "__main__":
    print(json.dumps(run(), ensure_ascii=False, indent=1))
