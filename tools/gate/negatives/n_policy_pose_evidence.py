"""Independent NPZ obligations and immutable collision-cache generation inputs."""
from __future__ import annotations

import contextlib
import copy
import io
import json
import sys
import tempfile
from pathlib import Path
from unittest.mock import patch

import numpy as np

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "tools/sim"))
import policy_pose_evidence as evidence
import policy_pose_collisions as sampler


def run():
    assertions = []

    def rejected(label, fn):
        try:
            fn()
        except (ValueError, KeyError, TypeError, OSError):
            assertions.append(dict(claim=label, ok=True))
        else:
            assertions.append(dict(claim=label, ok=False))

    with tempfile.TemporaryDirectory(prefix="duck_pose_evidence_") as temp:
        d = Path(temp)
        names = evidence.joint_names_from_mjcf()
        q = np.zeros((2, len(names)))
        q[0, 0] = .123456789
        q[1, 0] = 5.123456789
        steps = d / "steps.npz"
        np.savez(steps, joint_names=np.asarray(names), **{"stand/test/qpos_deg": q, "stand/test/prefall_n": np.asarray(2)})
        expected = evidence.expected_poses(steps, ["stand"], 2.5)
        rows = [dict(mode=k[0], case=k[1], step=k[2], pose_deg={j: round(v, 2) for j, v in pose.items()}, hits=[])
                for k, pose in expected["rows"].items()]
        ours, orig = d / "ours.jsonl", d / "orig.jsonl"

        def write(path, records):
            path.write_text("".join(json.dumps(r) + "\n" for r in records))

        write(ours, rows)
        assertions.append(dict(claim="historic rounded records accepted with all NPZ obligations", ok=len(evidence.validate_pose_records(ours, expected)) == 2))
        for path in (ours, orig):
            write(path, rows[:1])
            rejected(f"both sides drop same pose: {path.name}", lambda path=path: evidence.validate_pose_records(path, expected))
            bad = copy.deepcopy(rows)
            for row in bad:
                del row["pose_deg"][names[-1]]
            write(path, bad)
            rejected(f"both sides drop same joint: {path.name}", lambda path=path: evidence.validate_pose_records(path, expected))
        exact_rows = copy.deepcopy(rows)
        for row, pose in zip(exact_rows, expected["rows"].values()):
            row["pose_deg_exact"] = pose
        write(ours, exact_rows)
        assertions.append(dict(claim="new exact tuples accepted", ok=len(evidence.validate_pose_records(ours, expected)) == 2))
        wrong = copy.deepcopy(exact_rows)
        wrong[0]["pose_deg_exact"][names[0]] += .00001  # rounded value still identical
        write(ours, wrong)
        rejected("exact tuple mismatch below round2 precision", lambda: evidence.validate_pose_records(ours, expected))
        short_npz = d / "missing_joint.npz"
        np.savez(short_npz, joint_names=np.asarray(names[:-1]), **{"stand/test/qpos_deg": q[:, :-1], "stand/test/prefall_n": np.asarray(2)})
        rejected("NPZ itself omits a required kinematic joint", lambda: evidence.expected_poses(short_npz, ["stand"], 2.5))
        rejected("declared mode has no prefall evidence", lambda: evidence.expected_poses(steps, ["stand", "missing_mode"], 2.5))
        for invalid in (0, -1, float("inf"), float("nan"), True):
            rejected(f"invalid deduplication grid {invalid}", lambda invalid=invalid: evidence.expected_poses(steps, ["stand"], invalid))

        placed = d / "placed"; placed.mkdir()
        (placed / "a.stl").write_bytes(b"fixture A; never loaded as geometry")
        (placed / "b.stl").write_bytes(b"fixture B; never loaded as geometry")
        mapping = d / "map.json"
        mapping.write_text(json.dumps({"body_for_part": {"a": "trunk_base", "b": "hip_l"}}))
        out = d / "run"
        calls = []

        class FakeScene:
            names = ["a", "b"]

            def __init__(self, *args):
                calls.append("scene_constructed")

            def moving_pairs(self, active):
                return [(0, 1)]

            def evaluate(self, pose, pairs, tol):
                calls.append("evaluated")
                return [], [], 0

        class FakeCtx:
            def __init__(self, *args):
                self._mesh = {}

        data = {"frozen": {"capability_envelope": {"modes_in_scope": ["stand"]}},
                "tolerances": {"feature_check_tolerances": {"static_intersection_mm3": {"max": .05}}}}

        def sample(limit=0):
            argv = ["policy_pose_collisions", "--steps", str(steps), "--placed", str(placed),
                    "--bfp", str(mapping), "--out", str(out), "--limit", str(limit)]
            with patch.object(sys, "argv", argv), patch.object(sampler, "load_data", return_value=data), \
                 patch.object(sampler, "Ctx", FakeCtx), patch.object(sampler.L6, "_Scene", FakeScene), \
                 patch.object(sampler.L6, "PLACED", placed), contextlib.redirect_stdout(io.StringIO()):
                return sampler.main()

        sample(limit=1)
        partial = json.loads((out / "summary.partial.json").read_text())
        assertions.append(dict(claim="--limit only produces an explicitly partial summary", ok=(
            not (out / "summary.json").exists() and partial["partial"] is True and partial["complete"] is False
            and partial["poses_checked"] == 1 and partial["expected_poses"] == 2)))
        before = {p.name: p.read_bytes() for p in out.iterdir()}
        old_mesh = (placed / "a.stl").read_bytes()
        (placed / "a.stl").write_bytes(b"changed CAD")
        calls.clear()
        rejected("same-directory CAD change cannot reuse or restamp old rows", sample)
        assertions.append(dict(claim="stale cache rejected before geometry and all writes", ok=(
            not calls and before == {p.name: p.read_bytes() for p in out.iterdir()})))
        (placed / "a.stl").write_bytes(old_mesh)
        sample()
        complete = json.loads((out / "summary.json").read_text())
        assertions.append(dict(claim="same-input resume reaches complete NPZ coverage", ok=(
            complete["complete"] is True and complete["partial"] is False and complete["poses_checked"] == 2)))
        records = evidence.validate_pose_records(out / "poses.jsonl", expected)
        assertions.append(dict(claim="sampler retains unrounded angles", ok=all("pose_deg_exact" in r for r in records.values())))
        fp = complete["fingerprint"]
        prior_records = (out / "poses.jsonl").read_bytes()
        changed_records = copy.deepcopy(list(records.values()))
        changed_records[0]["hits"] = [["a", "b", 1.]]
        write(out / "poses.jsonl", changed_records)
        rejected("completed cache cannot restamp edited collision values", lambda: evidence.validate_resume(out, fp, expected))
        (out / "poses.jsonl").write_bytes(prior_records)
        for field, changed in (("mapping_sha256", "different"), ("grid_deg", 5.),
                               ("modes_in_scope", ["different"]), ("tol_mm3", 1.),
                               ("dependencies_sha256", {"l6": "changed"})):
            altered = dict(fp, **{field: changed})
            rejected(f"resume rejects changed {field}", lambda altered=altered: evidence.validate_resume(out, altered, expected))
        legacy = d / "legacy"; legacy.mkdir()
        write(legacy / "poses.jsonl", rows)
        (legacy / "summary.json").write_text(json.dumps({"fingerprint": fp}))
        rejected("legacy rows cannot gain current provenance retroactively", lambda: evidence.validate_resume(legacy, fp, expected))

    return dict(name="policy pose independent obligations and immutable resume provenance", passed=all(a["ok"] for a in assertions),
                expect="common omissions fail; changed inputs never restamp cached results; partial is not complete",
                got="; ".join(f"{a['claim']}={a['ok']}" for a in assertions), assertions=assertions)


if __name__ == "__main__":
    result = run()
    print(json.dumps(result, ensure_ascii=False, indent=2))
    sys.exit(0 if result["passed"] else 1)
