"""Recorded design experiments; never modifies the production CAD or placed STL.

Run with the repository Python. Candidate dimensions are study inputs, not
measured part specifications or Gate acceptance thresholds.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import sys
from pathlib import Path

import numpy as np
import trimesh
import manifold3d as M

ROOT = Path(__file__).resolve().parents[2]
for p in (ROOT, ROOT / "tools/gate", ROOT / "tools/gate/layers", ROOT / "tools/cad"):
    sys.path.insert(0, str(p))

import duckstructure as D
from assembly_audit import solid, intersection, path_peak, AXES
from core import load_data
from gate import Ctx
import l6_motion as L6
from duckstructure.s288 import bx, cyl, diff, placed, union


def scene():
    ctx = Ctx(load_data())
    bfp = json.loads((ROOT / "tools/sim/cad_geometry_manifest.json").read_text())["body_for_part"]
    sc = L6._Scene(ctx, bfp)
    ctx._mesh.clear()
    return sc


def hip_fork():
    """Outer fork joining the driven hip to the free rear idler (left).

    Broad screw plate is behind the carrier; only the small nose reaches the
    idler face. Keep the diagonal web below the two upper motor mount screws.
    Dimensions in the current world zero pose, mm; exploratory geometry only.
    """
    disk = cyl(17.0, 3.6, (-11.2, 17.5, 102.5), axis="x")
    # r5.25 + hole radius1.3 + minimum ligament0.8 => diameter >=14.7.
    nose = cyl(15.0, 2.6, (-8.3, 17.5, 102.5), axis="x")
    web = trimesh.util.concatenate([
        bx((3.6, 6.0, 4.0), (-11.2, 21.5, 97.5)),
        bx((3.6, 3.0, 8.0), (-11.2, 35.3, 94.5)),
    ]).convex_hull
    rail = bx((21.0, 3.0, 8.0), (-2.5, 35.3, 94.5))
    holes = placed(D.s288.idler_holes(12, center=(-16, 0, 0)), D.sfw("yaw2roll", 0))
    return diff(union(disk, nose, web, rail), holes)


def poses(kind):
    if kind == "single":
        return [(f"roll:{a:g}", {"left_hip_roll": float(a)})
                for a in sorted(set([-22., 0., 22.] + list(np.arange(-21.25, 22, 2.5))))]
    if kind == "combo":
        return [(f"yaw:{a:g},roll:{b:g}", {"left_hip_yaw": float(a), "left_hip_roll": float(b)})
                for a in np.linspace(-25, 30, 12) for b in [-22, -20, -15, -10, -5, 0, 5, 10, 15, 20, 22]]
    z = np.load(ROOT / "tools/gate/out/policy_steps_2026-09-16/policy_steps.npz")
    names = [str(x) for x in z["joint_names"]]
    src = ROOT / "tools/gate/out/policy_steps_2026-09-17_r5/poses.jsonl"
    out = []
    for line in src.read_text().splitlines():
        r = json.loads(line)
        key = f"{r['mode']}/{r['case']}"
        out.append((f"{key}/{r['step']}", dict(zip(names, z[key + "/qpos_deg"][r["step"]]))))
    return out


def probe(sc, mesh, body, pose_list, exclude=(), replacements=None, gap_exclude=()):
    candidate = solid(mesh)
    rep = {n: solid(m) for n, m in (replacements or {}).items()}
    anchor = sc.bodies.index(body)
    corners = np.c_[trimesh.bounds.corners(mesh.bounds), np.ones(8)]
    nb, checked, hits, worst, gaps = 0, 0, {}, {}, {}
    gap_search = 0.6   # reporting radius, not an acceptance threshold
    for label, pose in pose_list:
        mats = sc.deltas(pose)
        moved = candidate.transform(mats[anchor, :3])
        bounds = (corners @ mats[anchor].T)[:, :3]
        lo, hi = bounds.min(axis=0), bounds.max(axis=0)
        cor = np.einsum("nij,nkj->nki", mats, sc.corners)[..., :3]
        los, his = cor.min(axis=1), cor.max(axis=1)
        for i, n in enumerate(sc.names):
            if sc.bodies[i] == body or n in exclude:
                continue
            checked += 1
            sep = float(np.maximum(lo - his[i], los[i] - hi).max())
            if sep > gap_search:
                continue
            obstacle = rep.get(n, sc.solids[i]).transform(mats[i, :3])
            v = intersection(moved, obstacle) if sep <= 0 else 0.
            nb += int(sep <= 0)
            if n not in gap_exclude:
                gap = float(moved.min_gap(obstacle, gap_search))
                if gap < gaps.get(n, {"mm": gap_search})["mm"]:
                    gaps[n] = {"mm": gap, "pose_id": label}
            if not np.isfinite(v) or v < -1e-6:
                raise ValueError((label, n, v))
            if v > 0.05:
                hits[n] = hits.get(n, 0) + 1
                if n not in worst or v > worst[n]["mm3"]:
                    worst[n] = {"mm3": v, "pose_id": label,
                                "pose_deg": {k: float(vv) for k, vv in pose.items()}}
    return {"poses": len(pose_list), "pair_pose_evaluations": checked,
            "booleans": nb, "hits_gt_0_05": hits, "worst": worst,
            "gap_search_radius_mm": gap_search, "closest_below_search_radius": gaps}


def heads_pair_check(output):
    """Small repeatable check of the saved v5 fork, mirrored, with screw heads."""
    candidate = ROOT / "cad/support_study_2026-09-17/fork_v5/fork_exploratory.stl"
    sources = [candidate, Path(__file__), Path(L6.__file__),
               ROOT / "cad/support_study_2026-09-17/fork_v5/study_source.py",
               ROOT / "tools/gate/out/policy_steps_2026-09-16/policy_steps.npz",
               ROOT / "tools/gate/out/policy_steps_2026-09-17_r5/poses.jsonl",
               ROOT / "upstream/microduck_rl/src/mjlab_microduck/robot/microduck/robot_walk.xml"]
    sources += sorted((ROOT / "duckstructure").glob("*.py"))
    def hashes():
        return {str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest() for p in sources}
    inputs = hashes()
    fork = trimesh.load(candidate, process=True)
    heads = union(*[cyl(4.4, 1.6, (-13.8, 17.5 + D.S['horn_r'] * np.cos(a),
                                  102.5 + D.S['horn_r'] * np.sin(a)), axis="x")
                    for a in np.linspace(0, 2*np.pi, D.S['horn_n'], endpoint=False)])
    left = union(fork, heads)
    right = left.copy(); right.apply_transform(np.diag([1, -1, 1, 1]))
    meshes = [left, right]; solids = [solid(m) for m in meshes]
    corners = np.asarray([np.c_[trimesh.bounds.corners(m.bounds), np.ones(8)] for m in meshes])
    sc = object.__new__(L6._Scene)
    sc.B, sc.ORDER, sc.bodies = D.B, D.ORDER, ["hip_l", "hip_l_2"]
    rows = poses("policy")
    peak, minimum, nb, worst = 0., float('inf'), 0, None
    for label, pose in rows:
        mats = sc.deltas(pose)
        c = np.einsum("nij,nkj->nki", mats, corners)[..., :3]
        lo, hi = c.min(axis=1), c.max(axis=1)
        sep = float(np.maximum(lo[0] - hi[1], lo[1] - hi[0]).max())
        minimum = min(minimum, sep)
        if sep > 0:
            continue
        nb += 1
        v = intersection(solids[0].transform(mats[0, :3]), solids[1].transform(mats[1, :3]))
        if not np.isfinite(v) or v < -1e-6:
            raise ValueError((label, v))
        if v > peak:
            peak, worst = v, label
    assert inputs == hashes(), "inputs changed during pair check"
    result = dict(poses=len(rows), booleans=nb, peak_intersection_mm3=peak, worst=worst,
                  aabb_signed_separation_min_mm=minimum, includes_screw_heads=True,
                  inputs_sha256=inputs,
                  note="Positive AABB separation is a lower bound on mesh separation at sampled poses, not the exact mesh gap.")
    output.write_text(json.dumps(result, ensure_ascii=False, indent=2) + '\n')
    print(json.dumps({k: v for k, v in result.items() if k != 'inputs_sha256'}, ensure_ascii=False))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--poses", choices=["single", "combo", "policy"], default="single")
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--open-bottom", action="store_true", help="U-slot instead of a closed carrier window")
    ap.add_argument("--heads-pair", action="store_true", help="Only check saved v5 bilateral forks and heads; --out is a JSON file")
    args = ap.parse_args()
    if args.heads_pair:
        heads_pair_check(args.out)
        return
    args.out.mkdir(parents=True, exist_ok=True)
    sc = scene()
    fork = hip_fork()
    hip = trimesh.load(ROOT / "cad/duck_s288/placed/hip.stl", process=True)
    carrier = trimesh.load(ROOT / "cad/duck_s288/placed/yaw2roll.stl", process=True)
    window = cyl(16.0, 20.0, (-10, 17.5, 102.5), axis="x")
    if args.open_bottom:
        window = union(window, bx((20., 16., 30.), (-10, 17.5, 87.5)))
    opened_solid = solid(carrier) - solid(window)
    if opened_solid.status() != M.Error.NoError or opened_solid.is_empty():
        raise ValueError(f"invalid opened carrier: {opened_solid.status()}")
    om = opened_solid.to_mesh64()
    opened = trimesh.Trimesh(np.asarray(om.vert_properties)[:, :3],
                            np.asarray(om.tri_verts), process=False)
    pose_list = poses(args.poses)
    fork_r = fork.copy()
    fork_r.apply_transform(np.diag([1, -1, 1, 1]))
    right_carrier = opened.copy()
    right_carrier.apply_transform(np.diag([1, -1, 1, 1]))
    replacements = {"yaw2roll": opened, "yaw2roll_R": right_carrier}
    left_index = sc.bodies.index("hip_l")
    right_index = sc.bodies.index("hip_l_2")
    fs, frs = solid(fork), solid(fork_r)
    # Conservative screw-head envelope, same 4.4 mm clearance diameter used
    # by existing M2 pockets. This is a candidate, not a selected screw/tool.
    heads = union(*[cyl(4.4, 1.6, (-13.8, 17.5 + D.S['horn_r'] * np.cos(a),
                                  102.5 + D.S['horn_r'] * np.sin(a)), axis="x")
                    for a in np.linspace(0, 2*np.pi, D.S['horn_n'], endpoint=False)])
    with_heads = union(fork, heads)
    with_heads_r = with_heads.copy(); with_heads_r.apply_transform(np.diag([1, -1, 1, 1]))
    pair_peak = {"mm3": 0., "pose_id": None}
    for label, pose in pose_list:
        mats = sc.deltas(pose)
        v = intersection(fs.transform(mats[left_index, :3]), frs.transform(mats[right_index, :3]))
        if v > pair_peak["mm3"]:
            pair_peak = {"mm3": v, "pose_id": label}
    # Screening translations only. A failed six-direction scan does not prove
    # no curved/rotated assembly path exists; a success is also not a fastener audit.
    obstacles = {n: solid(replacements[n]) if n in replacements else sc.solids[i]
                 for i, n in enumerate(sc.names) if sc.bodies[i] != "hip_l"}
    assembly = []
    if args.poses == "single":
        for tag, direction in AXES:
            assembly.append({"direction": tag, **path_peak(
                {"hip_with_fork": fs + solid(hip)}, obstacles, direction, step=.5, distance=50.)})
    report = {
        "candidate": "exploratory_outer_hip_idler_fork_not_for_print",
        "tolerance_mm3": 0.05,
        "inputs_sha256": {str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest()
                          for p in sorted((ROOT / "cad/duck_s288/placed").glob("*.stl"))},
        "pose_inputs_sha256": {str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest()
            for p in (ROOT / "tools/gate/out/policy_steps_2026-09-16/policy_steps.npz",
                      ROOT / "tools/gate/out/policy_steps_2026-09-17_r5/poses.jsonl")},
        "script_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        "open_bottom": args.open_bottom,
        "fork_mm3": float(fork.volume),
        "attachment_overlap_mm3": float((solid(fork) ^ solid(hip)).volume()),
        "attachment_bounds_mm": list((solid(fork) ^ solid(hip)).bounding_box()),
        "carrier_components": len(opened_solid.decompose()),
        "fork_plus_hip_components": len((fs + solid(hip)).decompose()),
        "carrier_window_removed_mm3": float(carrier.volume - opened.volume),
        "left_scan": probe(sc, with_heads, "hip_l", pose_list, replacements=replacements,
                           gap_exclude=("servo__yaw2roll_hip_l",)),
        "right_scan": probe(sc, with_heads_r, "hip_l_2", pose_list, replacements=replacements,
                            gap_exclude=("servo__bearing_roll_hip_l_2",)),
        "fork_pair_peak": pair_peak,
        "straight_assembly_screen": assembly,
    }
    (args.out / f"fork_{args.poses}.json").write_text(json.dumps(report, ensure_ascii=False, indent=2))
    fork.export(args.out / "fork_exploratory.stl")
    opened.export(args.out / "carrier_exploratory.stl")
    (args.out / "study_source.py").write_text(Path(__file__).read_text())
    print(json.dumps({k: v for k, v in report.items() if k != "inputs_sha256"}, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
