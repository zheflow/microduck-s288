"""Pure, inexpensive checks for pose coverage and collision-cache provenance.

This module never loads CAD solids or writes files. Sampling obligations come
from the recorded NPZ, not from either collision result being assessed.
"""
from __future__ import annotations

import hashlib
import importlib.metadata
import json
import math
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
KINEMATIC_MJCF = ROOT / "upstream/microduck_rl/src/mjlab_microduck/robot/microduck/robot_walk.xml"
SAMPLER = ROOT / "tools/sim/policy_pose_collisions.py"


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def joint_names_from_mjcf(path=KINEMATIC_MJCF):
    names = [j.get("name") for j in ET.parse(path).findall(".//worldbody//joint")]
    if not names or any(not n for n in names) or len(names) != len(set(names)):
        raise ValueError("kinematic MJCF has missing/duplicate joint names")
    return names


def _finite(v):
    return isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v)


def expected_poses(steps_path, modes_in_scope, grid_deg, joint_names=None):
    """Reconstruct the complete, ordered deduplication of all in-scope prefall rows.

    ``joint_names`` is only an explicit override for small unit fixtures. Normal
    callers obtain the required joint set from the independent kinematic MJCF.
    The result's ``rows`` maps (mode, case, step) to the *unrounded* joint tuple.
    """
    if not _finite(grid_deg) or grid_deg <= 0:
        raise ValueError("grid must be a finite positive angle")
    scope = list(modes_in_scope)
    if (not scope or any(not isinstance(m, str) or not m for m in scope)
            or len(scope) != len(set(scope))):
        raise ValueError("in-scope modes must be nonempty and unique")
    required = joint_names_from_mjcf() if joint_names is None else list(joint_names)
    if not required or any(not isinstance(n, str) or not n for n in required) or len(required) != len(set(required)):
        raise ValueError("required joint names must be nonempty and unique")
    seen, rows, counts = set(), {}, {m: 0 for m in scope}
    with np.load(steps_path, allow_pickle=False) as z:
        names_array = z["joint_names"]
        if names_array.ndim != 1 or names_array.dtype.kind not in "US":
            raise ValueError("NPZ joint_names must be a string vector")
        names = [str(v) for v in names_array]
        if len(names) != len(set(names)) or set(names) != set(required):
            raise ValueError(f"NPZ joint set differs: missing={sorted(set(required)-set(names))}, "
                             f"extra={sorted(set(names)-set(required))}")
        if len(z.files) != len(set(z.files)):
            raise ValueError("NPZ contains duplicate array names")
        for key in z.files:
            if not key.endswith("/qpos_deg"):
                continue
            tokens = key.split("/")
            if len(tokens) != 3 or not tokens[0] or not tokens[1]:
                raise ValueError(f"invalid trajectory key: {key}")
            mode, case, _ = tokens
            if mode not in counts:
                continue
            q = z[key]
            n_raw = z[key.replace("/qpos_deg", "/prefall_n")]
            if (q.ndim != 2 or q.shape[1] != len(names) or q.dtype.kind not in "iuf"
                    or not np.isfinite(q).all()):
                raise ValueError(f"invalid joint matrix: {key}")
            if n_raw.shape != () or n_raw.dtype.kind not in "iu":
                raise ValueError(f"prefall_n must be an integer scalar: {key}")
            n = int(n_raw)
            if n < 0 or n > len(q):
                raise ValueError(f"prefall_n outside trajectory: {key}: {n}/{len(q)}")
            counts[mode] += n
            for i in range(n):
                with np.errstate(over="ignore", invalid="ignore"):
                    quantized = np.round(q[i] / grid_deg)
                if not np.isfinite(quantized).all():
                    raise ValueError("pose/grid quantization overflow")
                cell = tuple(quantized.tolist())
                if cell in seen:
                    continue
                seen.add(cell)
                rows[(mode, case, i)] = dict(zip(names, (float(v) for v in q[i])))
    if any(n == 0 for n in counts.values()) or not rows:
        raise ValueError(f"missing in-scope prefall samples: {counts}")
    return dict(joint_names=names, rows=rows, raw_count=sum(counts.values()),
                source_mode_counts=counts, grid_deg=float(grid_deg), modes_in_scope=scope)


def validate_pose_records(path, expected, require_complete=True):
    """Validate keys and full joint tuples; accept historic round(..., 2) records.

    When present, ``pose_deg_exact`` must additionally equal the NPZ numbers.
    Missing exact values are accepted only as the documented old file format.
    Returns the records by pose key, useful for safe same-input resumption.
    """
    records, wanted = {}, expected["rows"]
    for line_number, line in enumerate(Path(path).read_text().splitlines(), 1):
        if not line.strip():
            continue
        r = json.loads(line)
        if not isinstance(r, dict):
            raise ValueError(f"pose record is not an object: line {line_number}")
        step = r.get("step")
        if not isinstance(step, int) or isinstance(step, bool) or step < 0:
            raise ValueError(f"invalid pose step: line {line_number}")
        key = (r.get("mode"), r.get("case"), step)
        if not all(isinstance(k, str) for k in key[:2]):
            raise ValueError(f"invalid mode/case: line {line_number}")
        if key in records or key not in wanted:
            raise ValueError(f"duplicate/unexpected pose: {key}")
        pose = r.get("pose_deg")
        exact = wanted[key]
        if not isinstance(pose, dict) or set(pose) != set(exact):
            raise ValueError(f"missing/extra joints at {key}")
        if any(not _finite(pose[j]) or pose[j] != round(exact[j], 2) for j in exact):
            raise ValueError(f"rounded pose does not match NPZ at {key}")
        if "pose_deg_exact" in r:
            full = r["pose_deg_exact"]
            if (not isinstance(full, dict) or set(full) != set(exact)
                    or any(not _finite(full[j]) or full[j] != exact[j] for j in exact)):
                raise ValueError(f"exact pose does not match NPZ at {key}")
        if not isinstance(r.get("hits"), list):
            raise ValueError(f"missing collision records at {key}")
        for hit in r["hits"]:
            if (not isinstance(hit, list) or len(hit) != 3
                    or not all(isinstance(n, str) and n for n in hit[:2])
                    or not _finite(hit[2]) or hit[2] < 0):
                raise ValueError(f"invalid collision record at {key}")
        records[key] = r
    if require_complete and set(records) != set(wanted):
        raise ValueError(f"incomplete pose evidence: got {len(records)}, expected {len(wanted)}, "
                         f"missing {len(set(wanted)-set(records))}")
    return records


def placed_fingerprint(placed):
    files = sorted(Path(placed).glob("*.stl"))
    if not files:
        raise ValueError("placed scene is empty")
    h = hashlib.sha256()
    for path in files:
        h.update(path.name.encode()); h.update(bytes.fromhex(sha(path)))
    return h.hexdigest()


def generation_fingerprint(placed, steps, mapping, scope, grid, tol):
    """Bind geometry, poses, filters, ownership and the actual numerical code.

    Hash the small source/data closure conservatively. Editing unrelated rules
    may require a new run, but cannot silently retain outdated collision math.
    """
    if not _finite(tol) or tol < 0 or not _finite(grid) or grid <= 0:
        raise ValueError("invalid collision tolerance/grid")
    dependencies = {SAMPLER, Path(__file__).resolve(), KINEMATIC_MJCF,
                    ROOT / "tools/gate/layers/l6_motion.py", ROOT / "tools/gate/core.py",
                    ROOT / "tools/gate/gate.py", ROOT / "tools/cad/assembly_audit.py",
                    ROOT / "tools/cad/check_cache.py"}          # hr42：布尔体积可能取自检查原语缓存 → 缓存代码也绑进指纹
    dependencies.update((ROOT / "duckstructure").glob("*.py"))
    dependencies.update((ROOT / "tools/gate/data").glob("*.yaml"))
    versions = {name: importlib.metadata.version(name)
                for name in ("numpy", "trimesh", "manifold3d", "scipy", "shapely")}
    return dict(generation_schema=1, placed_sha256=placed_fingerprint(placed),
                steps_sha256=sha(steps), script_sha256=sha(SAMPLER), mapping_sha256=sha(mapping),
                tol_mm3=float(tol), grid_deg=float(grid), modes_in_scope=sorted(scope),
                dependencies_sha256={str(p.relative_to(ROOT)): sha(p) for p in sorted(dependencies)},
                runtime=dict(python=sys.version, packages=versions))


def validate_resume(out_dir, fingerprint, expected):
    """Reject unproven/mutated cached rows *before* the sampler opens append mode.

    A persisted run.json is required even for a partial run. Legacy directories
    cannot be upgraded by stamping today's fingerprints over their old rows.
    """
    out = Path(out_dir)
    run_path, records_path = out / "run.json", out / "poses.jsonl"
    has_old = any((out / name).exists() for name in
                  ("poses.jsonl", "summary.json", "summary.partial.json", "compare.json"))
    if not run_path.exists():
        if has_old:
            raise ValueError("existing collision output has no generation manifest; use a NEW output directory")
        return {}
    old = json.loads(run_path.read_text())
    if old.get("fingerprint") != fingerprint or old.get("expected_poses") != len(expected["rows"]):
        raise ValueError("collision cache generation inputs changed; use a NEW output directory")
    if not records_path.exists():
        if has_old:
            raise ValueError("collision records missing from existing run; use a NEW output directory")
        return {}
    records = validate_pose_records(records_path, expected, require_complete=False)
    for name in ("summary.json", "summary.partial.json"):
        p = out / name
        if not p.exists():
            continue
        s = json.loads(p.read_text())
        if s.get("fingerprint") != fingerprint:
            raise ValueError("cached summary inputs differ; use a NEW output directory")
        if name == "summary.json" and (s.get("complete") is not True
                or s.get("poses_checked") != len(expected["rows"])
                or set(records) != set(expected["rows"])
                or s.get("poses_sha256") != sha(records_path)):
            raise ValueError("claimed complete summary has incomplete/modified records; use a NEW output directory")
    return records
