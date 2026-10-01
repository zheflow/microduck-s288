#!/usr/bin/env python3
"""缺陷 body 对（compare.json defect_pairs）在所有缺陷姿态下的交集，逐**件**转到 export_local 系求并 → 要挖的区域（体积/bbox/STL）。
    ./.venv/bin/python tools/sim/policy_pose_regions.py [--pairs hip_l×hip_l_2,...] [--min-mm3 1.0]
输出 tools/gate/out/policy_steps_2026-09-16/regions/<part>__by_<other>.stl + regions.json
"""
from __future__ import annotations
import argparse, json, sys, math
from pathlib import Path
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
for p in (ROOT / "tools/gate", ROOT / "tools/gate/layers", ROOT, ROOT / "tools/cad", ROOT / "tools/sim"):
    if str(p) not in sys.path:
        sys.path.insert(0, str(p))
import manifold3d as M            # noqa: E402
import trimesh                    # noqa: E402
from core import load_data        # noqa: E402
from gate import Ctx              # noqa: E402
import l6_motion as L6            # noqa: E402

D = ROOT / "tools/gate/out/policy_steps_2026-09-16"
OUT = D / "regions"


def materialize(man):
    raw = man.to_mesh64()
    return M.Manifold(M.Mesh64(vert_properties=np.array(raw.vert_properties, dtype=np.float64, order="C", copy=True),
                               tri_verts=np.array(raw.tri_verts, dtype=np.uint64, order="C", copy=True)))


def m2tri(man):
    raw = man.to_mesh64(); v = np.asarray(raw.vert_properties)[:, :3]; f = np.asarray(raw.tri_verts)
    return trimesh.Trimesh(vertices=v, faces=f, process=False)


def main():
    ap = argparse.ArgumentParser(); ap.add_argument("--pairs", default=""); ap.add_argument("--min-mm3", type=float, default=1.0)
    ap.add_argument("--dir", type=Path, default=D, help="我们的 poses.jsonl / compare.json 所在目录（默认 09-16）；原版仍读 09-16/orig")
    a = ap.parse_args()
    global OUT
    OUT = a.dir / "regions"
    cmp_ = json.loads((a.dir / "compare.json").read_text(encoding="utf-8"))
    want = [x for x in a.pairs.split(",") if x] or list(cmp_["defect_pairs"].keys())
    data = load_data(); ctx = Ctx(data)
    bfp = json.loads((ROOT / "tools/sim/cad_geometry_manifest.json").read_text(encoding="utf-8"))["body_for_part"]
    sc = L6._Scene(ctx, bfp); ctx._mesh.clear()
    idx = {n: i for i, n in enumerate(sc.names)}
    Tw = {b: np.asarray(sc.B[b]["T_world"], float) for b in sc.ORDER}
    ours = {}
    for l in (a.dir / "poses.jsonl").read_text(encoding="utf-8").splitlines():
        if l.strip():
            r = json.loads(l); ours[(r["mode"], r["case"], r["step"])] = r
    orig = {}
    for l in (D / "orig/poses.jsonl").read_text(encoding="utf-8").splitlines():
        if l.strip():
            r = json.loads(l); orig[(r["mode"], r["case"], r["step"])] = r
    obfp = json.loads((ROOT / "tools/gate/out/orig_placed_2026-09-16/body_for_part.json").read_text(encoding="utf-8"))["body_for_part"]
    OUT.mkdir(parents=True, exist_ok=True)
    report = {}
    for pair in want:
        ba, bb = pair.split("×")
        regions = {}      # part -> list of manifold in export_local
        n_pose = 0
        for key, r in ours.items():
            hits = [(x, y, v) for x, y, v in r["hits"] if v >= a.min_mm3 and {bfp.get(x, x), bfp.get(y, y)} == {ba, bb}]
            if not hits:
                continue
            og = orig.get(key)
            if og and any({obfp.get(x, x), obfp.get(y, y)} == {ba, bb} and v >= a.min_mm3 for x, y, v in og["hits"]):
                continue                                                   # 原版也撞 → 不算缺陷
            n_pose += 1
            mats = sc.deltas(r["pose_deg"])
            for x, y, v in hits:
                i, k = idx[x], idx[y]
                mi, mk = sc.solids[i].transform(mats[i, :3]), sc.solids[k].transform(mats[k, :3])
                it = materialize(mi ^ mk)
                for part, ii in ((x, i), (y, k)):
                    posed = mats[ii] @ Tw[sc.bodies[ii]]
                    loc = materialize(it.transform(np.linalg.inv(posed)[:3]))
                    regions.setdefault(part, []).append(loc)
        rep = {"defect_poses": n_pose, "parts": {}}
        for part, lst in regions.items():
            u = M.Manifold.batch_boolean(lst, M.OpType.Add)
            tri = m2tri(u); f = OUT / f"{part}__pair_{ba}_x_{bb}.stl"; tri.export(str(f))
            b = tri.bounds
            rep["parts"][part] = {"union_mm3": round(float(u.volume()), 2), "bbox_local": [np.round(b[0], 1).tolist(), np.round(b[1], 1).tolist()], "stl": str(f.relative_to(ROOT)), "n": len(lst)}
        report[pair] = rep
        print(f"{pair}: {n_pose} 缺陷姿态 → " + "; ".join(f"{p} {d['union_mm3']} mm³ bbox {d['bbox_local']}" for p, d in rep["parts"].items()))
    (OUT / "regions.json").write_text(json.dumps(report, ensure_ascii=False, indent=1), encoding="utf-8")
    return 0


if __name__ == "__main__":
    sys.exit(main())
