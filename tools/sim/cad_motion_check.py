"""Exact mesh Booleans at sampled poses for the local replay command gate.

No CAD rebuilding, hardware I/O, or writes to historical audit files. The source
snapshot identifies the geometry; it is not approval of that geometry.
"""
import argparse
import hashlib
import itertools
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np

from motion_guard import GuardRejected, JointLimits, MotionGuard

ROOT = Path(__file__).resolve().parents[2]
ROBOT_XML = Path("upstream/microduck_rl/src/mjlab_microduck/robot/microduck/robot_walk.xml")
PLACED = Path("cad/duck_s288/placed")
DEFAULT_MANIFEST = ROOT / "tools/sim/cad_geometry_manifest.json"
SOURCE_GLOBS = (
    (Path("duckstructure"), "*.py"),
    (Path("duckstructure/data"), "**/*.npz"),
    (Path("duckstructure/data"), "**/*.stl"),
    (Path("duckstructure/data"), "**/*.json"),
    (ROBOT_XML.parent / "assets", "*.stl"),
)


def discovered_sources(root):
    # Include geometry inputs as well as generators. Re-scan during a run so a
    # newly added cutter or source module also invalidates the snapshot.
    return {p.relative_to(root) for directory, pattern in SOURCE_GLOBS
            for p in (root / directory).glob(pattern) if p.is_file()}


SOURCE_FILES = tuple(sorted({
    ROBOT_XML, ROBOT_XML.with_name("scene_walk.xml"),
    ROBOT_XML.parent / "assets/bottom_head_shell.stl",
    Path("upstream/microduck_rl/scripts/infer_policy.py"),
    Path("duckstructure/lib.py"), Path("duckstructure/legs.py"),
    Path("duckstructure/trunk.py"), Path("duckstructure/neck.py"),
    Path("duckstructure/head.py"), Path("duckstructure/checks.py"),
    Path("duckstructure/build.py"), Path("duckstructure/__init__.py"),
    Path("duckstructure/kin.py"),
    Path("duckstructure/s288.py"), Path("tools/cad/mechanical_audit.py"),
    Path("tools/cad/assembly_audit.py"),
    Path("tools/cad/asmcheck.py"),
    Path("tools/cad/ankle_split.py"),
    Path("tools/sim/motion_guard.py"), Path("tools/sim/cad_motion_check.py"),
    Path("tools/sim/render_policy.py"),
    Path("tools/gate/data/parts.yaml"),
    *discovered_sources(ROOT),
}, key=str))


def declared_body_map(data):
    """Ownership is design data; adding a part must not require editing a checker."""
    mapping = data.get("motion_instances")
    if not isinstance(mapping, dict) or not mapping or any(
            not isinstance(n, str) or not isinstance(b, str) or not n or not b
            for n, b in mapping.items()):
        raise GuardRejected("motion_instances_not_declared")
    return dict(mapping)


import yaml
BODY_FOR_PART = declared_body_map(yaml.safe_load(
    (ROOT / "tools/gate/data/parts.yaml").read_text(encoding="utf-8")))
COLLISION_THRESHOLD_MM3 = 0.05
H03_REFERENCE_PAIR = frozenset(("head_bottom_shell", "head_top_shell"))   # hr38：顶壳换成 H05 派生件，合缝面与原版逐面相同


def _signature(path):
    s = path.stat()
    return (s.st_dev, s.st_ino, s.st_size, s.st_mtime_ns, s.st_ctime_ns)


def _inventory(root):
    names = {p.stem for p in (root / PLACED).glob("*.stl")}
    if names != set(BODY_FOR_PART):
        raise GuardRejected("placed_inventory_changed", missing=sorted(set(BODY_FOR_PART) - names),
                            unknown=sorted(names - set(BODY_FOR_PART)))
    return sorted({*SOURCE_FILES, *discovered_sources(root),
                   *(PLACED / (n + ".stl") for n in names)}, key=str)


def capture_manifest(root=ROOT):
    root = Path(root)
    paths = _inventory(root)
    signatures = {str(p): _signature(root / p) for p in paths}
    hashes = {str(p): hashlib.sha256((root / p).read_bytes()).hexdigest() for p in paths}
    if paths != _inventory(root) or any(_signature(root / p) != signatures[str(p)] for p in paths):
        raise GuardRejected("source_changed_during_snapshot")
    return {"version": 1, "scope": "identity snapshot, not geometry approval",
            "body_for_part": BODY_FOR_PART, "source_sha256": hashes}


class SourceSnapshot:
    def __init__(self, root, manifest):
        self.root, self.manifest = Path(root), Path(manifest)
        self.manifest_signature = _signature(self.manifest)
        manifest_bytes = self.manifest.read_bytes()
        value = json.loads(manifest_bytes)
        if value.get("version") != 1 or value.get("body_for_part") != BODY_FOR_PART:
            raise GuardRejected("unsupported_geometry_manifest")
        self.signatures = {str(p): _signature(self.root / p) for p in _inventory(self.root)}
        actual = capture_manifest(self.root)
        if actual["source_sha256"] != value.get("source_sha256"):
            expected = value.get("source_sha256", {})
            changed = [p for p in set(expected) | set(actual["source_sha256"])
                       if expected.get(p) != actual["source_sha256"].get(p)]
            raise GuardRejected("geometry_source_changed", paths=sorted(changed))
        self.identity = hashlib.sha256(manifest_bytes).hexdigest()
        self.hashes = actual["source_sha256"]
        self.assert_unchanged()

    def assert_unchanged(self):
        if {str(p) for p in _inventory(self.root)} != set(self.signatures):
            raise GuardRejected("geometry_inventory_changed_during_run")
        changed = [p for p, sig in self.signatures.items() if _signature(self.root / p) != sig]
        if changed or _signature(self.manifest) != self.manifest_signature:
            raise GuardRejected("geometry_source_changed_during_run", paths=changed)


class CadScene:
    def __init__(self, manifest=DEFAULT_MANIFEST, root=ROOT):
        # Heavy imports and mesh construction happen only when this is explicitly used.
        import manifold3d as manifold
        import trimesh
        self.M, self.trimesh = manifold, trimesh
        self.root = Path(root)
        self.sources = SourceSnapshot(self.root, manifest)
        self.limits = JointLimits.from_mjcf(self.root / ROBOT_XML)
        self._read_kinematics()
        self.names = sorted(BODY_FOR_PART)
        self.bodies = [BODY_FOR_PART[n] for n in self.names]
        if not set(self.bodies) <= set(self.order):
            raise GuardRejected("unknown_body_assignment")
        self.solids, self.corners, self.conversion = [], [], []
        for name in self.names:
            mesh = trimesh.load_mesh(self.root / PLACED / (name + ".stl"), process=True)
            if not np.isfinite(mesh.vertices).all() or mesh.volume <= 0:
                raise GuardRejected("invalid_mesh", part=name)
            solid = manifold.Manifold(manifold.Mesh64(
                vert_properties=np.asarray(mesh.vertices, dtype=np.float64),
                tri_verts=np.asarray(mesh.faces, dtype=np.uint64)))
            if solid.status() != manifold.Error.NoError:
                raise GuardRejected("invalid_solid", part=name, status=str(solid.status()))
            self.solids.append(solid)
            c = trimesh.bounds.corners(mesh.bounds)
            self.corners.append(np.c_[c, np.ones(len(c))])
            self.conversion.append({"part": name, "volume_delta_mm3": float(solid.volume() - mesh.volume)})
        self.corners = np.asarray(self.corners)
        self.moving_pairs, self.static_pairs, self.excluded_pairs = [], [], []
        self.reference_pair_evidence = self._verify_h03_reference_overlap()
        for i, j in itertools.combinations(range(len(self.names)), 2):
            if self.bodies[i] != self.bodies[j]:
                self.moving_pairs.append((i, j))
            elif self.names[i].startswith("orig_") and self.names[j].startswith("orig_"):
                # The original closed-jaw decorative meshes overlap within one rigid
                # body. This matches the CAD audit's explicit original-original scope.
                self.excluded_pairs.append([self.names[i], self.names[j]])
            elif frozenset((self.names[i], self.names[j])) == H03_REFERENCE_PAIR:
                # The sole H03 exemption is checked geometrically against the
                # unchanged original lower shell, not just a volume allow-list.
                continue
            else:
                self.static_pairs.append((i, j))
        self.static_collisions = self._intersections(np.tile(np.eye(4), (len(self.names), 1, 1)), self.static_pairs)
        self.sources.assert_unchanged()

    def _verify_h03_reference_overlap(self):
        tree = ET.parse(self.root / ROBOT_XML).getroot()
        geoms = tree.findall(".//body[@name='jaw_soft']/geom[@mesh='bottom_head_shell']")
        geoms = [g for g in geoms if g.get("class") == "visual"]
        if len(geoms) != 1:
            raise GuardRejected("original_bottom_shell_reference_missing")
        g = geoms[0]
        local = self.trimesh.transformations.quaternion_matrix(
            np.fromstring(g.get("quat", "1 0 0 0"), sep=" "))
        local[:3, 3] = np.fromstring(g.get("pos", "0 0 0"), sep=" ") * 1000
        mesh = self.trimesh.load_mesh(self.root / ROBOT_XML.parent / "assets/bottom_head_shell.stl", process=True)
        mesh.apply_scale(1000)
        mesh.apply_transform(self.body["jaw_soft"]["zero"] @ local)
        # Compare at the same binary-STL coordinate precision as placed meshes;
        # otherwise export roundoff can look like a thin newly added interface.
        mesh.vertices = np.asarray(mesh.vertices, dtype=np.float32).astype(np.float64)
        original = self.M.Manifold(self.M.Mesh64(vert_properties=np.asarray(mesh.vertices, dtype=np.float64),
                                                tri_verts=np.asarray(mesh.faces, dtype=np.uint64)))
        new = self.solids[self.names.index("head_bottom_shell")]
        top = self.solids[self.names.index("head_top_shell")]
        old_overlap, new_overlap = original ^ top, new ^ top
        novel = new_overlap - old_overlap
        if any(s.status() != self.M.Error.NoError for s in (original, old_overlap, new_overlap, novel)):
            raise GuardRejected("original_interface_boolean_failed")
        values = [float(s.volume()) for s in (old_overlap, new_overlap, novel)]
        if not np.isfinite(values).all() or min(values) < -1e-6:
            raise GuardRejected("original_interface_unknown")
        # Only tolerate numerical export error, well below the 0.05 mm³ collision
        # threshold. An intersection outside the original interface is rejected.
        if values[2] > 0.001:
            raise GuardRejected("new_overlap_outside_original_interface",
                                pair=sorted(H03_REFERENCE_PAIR), new_overlap_mm3=values[2])
        return {"pair": sorted(H03_REFERENCE_PAIR), "reason": "intersection contained in original shell interface",
                "original_overlap_mm3": values[0], "current_overlap_mm3": values[1],
                "new_overlap_outside_original_mm3": max(0, values[2]),
                "numerical_export_threshold_mm3": 0.001}

    def _read_kinematics(self):
        self.order, self.body = [], {}
        tree = ET.parse(self.root / ROBOT_XML).getroot()
        def walk(node, parent):
            name = node.get("name")
            if not name or name in self.body:
                raise GuardRejected("invalid_body_tree")
            quat = np.fromstring(node.get("quat", "1 0 0 0"), sep=" ")
            pos = np.fromstring(node.get("pos", "0 0 0"), sep=" ") * 1000
            if (quat.shape != (4,) or pos.shape != (3,) or not np.isfinite(np.r_[quat, pos]).all()
                    or np.linalg.norm(quat) < 1e-12):
                raise GuardRejected("invalid_body_transform", body=name)
            t = self.trimesh.transformations.quaternion_matrix(quat)
            t[:3, 3] = pos
            if len(node.findall("joint")) > 1 or (parent is not None and node.find("freejoint") is not None):
                raise GuardRejected("unsupported_additional_joint", body=name)
            joint = node.find("joint")
            j = None
            if joint is not None:
                axis = np.fromstring(joint.get("axis", "0 0 1"), sep=" ")
                pivot = np.fromstring(joint.get("pos", "0 0 0"), sep=" ") * 1000
                if (joint.get("type", "hinge") != "hinge" or axis.shape != (3,) or pivot.shape != (3,)
                        or not np.isfinite(np.r_[axis, pivot]).all() or np.linalg.norm(axis) < 1e-12):
                    raise GuardRejected("unsupported_joint_transform", body=name)
                j = {"name": joint.get("name"), "axis": axis, "pivot": pivot}
            zero = t if parent is None else self.body[parent]["zero"] @ t
            self.order.append(name)
            self.body[name] = {"parent": parent, "t": t, "zero": zero, "joint": j}
            for child in node.findall("body"):
                walk(child, name)
        bodies = tree.findall("worldbody/body")
        if len(bodies) != 1:
            raise GuardRejected("unsupported_worldbody")
        walk(bodies[0], None)
        found = {b["joint"]["name"] for b in self.body.values() if b["joint"]}
        if found != set(self.limits.names):
            raise GuardRejected("kinematics_joint_mismatch")

    def transforms(self, radians):
        q = dict(zip(self.limits.names, radians))
        posed, delta = {}, {}
        for name in self.order:
            b = self.body[name]
            t = b["t"].copy() if b["parent"] is None else posed[b["parent"]] @ b["t"]
            if b["joint"]:
                j = b["joint"]
                t = t @ self.trimesh.transformations.rotation_matrix(q[j["name"]], j["axis"], j["pivot"])
            posed[name], delta[name] = t, t @ np.linalg.inv(b["zero"])
        return np.asarray([delta[b] for b in self.bodies])

    def _intersections(self, mats, pairs):
        if not pairs:
            return []
        corners = np.einsum("nij,nkj->nki", mats, self.corners)[..., :3]
        lo, hi = corners.min(axis=1), corners.max(axis=1)
        moved, rows = {}, []
        for i, j in pairs:
            if np.any(hi[i] < lo[j]) or np.any(hi[j] < lo[i]):
                continue
            for k in (i, j):
                if k not in moved:
                    moved[k] = self.solids[k].transform(mats[k, :3])
            intersection = moved[i] ^ moved[j]
            value = float(intersection.volume())
            if intersection.status() != self.M.Error.NoError or not np.isfinite(value) or value < -1e-6:
                raise GuardRejected("boolean_failed", a=self.names[i], b=self.names[j])
            if value > COLLISION_THRESHOLD_MM3:
                rows.append({"a": self.names[i], "b": self.names[j], "volume_mm3": value})
        return rows

    def collisions(self, radians):
        return self.static_collisions + self._intersections(self.transforms(radians), self.moving_pairs)

    def guard(self, max_step_deg=0.5):
        return MotionGuard(self.limits, self.collisions, assert_sources=self.sources.assert_unchanged,
                           max_step_deg=max_step_deg)

    def evidence(self):
        return {"manifest_sha256": self.sources.identity, "objects": len(self.names),
                "moving_pairs": len(self.moving_pairs), "static_pairs": len(self.static_pairs),
                "excluded_original_same_body_pairs": self.excluded_pairs,
                "verified_inherited_shell_interface": self.reference_pair_evidence,
                "static_collisions": self.static_collisions,
                "threshold_mm3": COLLISION_THRESHOLD_MM3,
                "solid_conversion": self.conversion,
                "joint_names": self.limits.names,
                "joint_limits_deg": np.degrees(self.limits.radians).tolist()}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)
    snap = sub.add_parser("snapshot", help="bind exact current source identity; does not approve geometry")
    snap.add_argument("--out", type=Path, default=DEFAULT_MANIFEST)
    check = sub.add_parser("trajectory", help="preflight recorded targets; never opens hardware")
    check.add_argument("npz", type=Path)
    check.add_argument("--manifest", type=Path, default=DEFAULT_MANIFEST)
    check.add_argument("--report", type=Path, required=True)
    args = parser.parse_args()
    if args.command == "snapshot":
        args.out.parent.mkdir(parents=True, exist_ok=True)
        args.out.write_text(json.dumps(capture_manifest(), ensure_ascii=False, indent=2) + "\n")
        print("Recorded source identity only; runtime CAD checks are still mandatory:", args.out)
        return 0
    result = {"status": "blocked", "input": str(args.npz.resolve()),
              "scope": "offline, sampled joint-linear target paths; no S288/hardware qualification"}
    try:
        scene = CadScene(args.manifest)
        guard = scene.guard()
        result["geometry"] = scene.evidence()
        with np.load(args.npz, allow_pickle=False) as data:
            if tuple(data["joint_names"].tolist()) != scene.limits.names:
                raise GuardRejected("trajectory_joint_order_mismatch")
            targets, actual = data["target_rad"], data["joint_rad"]
            times = data["time_s"]
            if (targets.ndim != 2 or targets.shape != actual.shape or targets.shape[1] != len(scene.limits.names)
                    or len(times) != len(targets) or not len(times) or not np.isfinite(times).all()
                    or (np.diff(times) <= 0).any()):
                raise GuardRejected("invalid_trajectory_shape_or_time")
            controls = np.array(targets[0], copy=True)
            for index, (q, target) in enumerate(zip(actual, targets)):
                result["index"], result["time_s"] = index, float(times[index])
                guard.apply_target(controls, target, observed=q)
            result.update(status="sampled_preflight_passed", commands=len(targets),
                          geometry_evaluations=guard.geometry_evaluations)
    except GuardRejected as exc:
        result["rejection"] = exc.as_dict()
    except Exception as exc:
        result["rejection"] = {"reason": "preflight_unknown", "error": str(exc)}
    args.report.parent.mkdir(parents=True, exist_ok=True)
    args.report.write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n")
    print(json.dumps({k: v for k, v in result.items() if k != "geometry"}, ensure_ascii=False))
    return 0 if result["status"] == "sampled_preflight_passed" else 2


if __name__ == "__main__":
    raise SystemExit(main())
