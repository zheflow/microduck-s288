"""Lightweight rejection regressions: no real CAD construction or hardware I/O."""
import json
import os
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import numpy as np

from motion_guard import GuardRejected, JOINT_NAMES, JointLimits, MotionGuard, policy_target
from cad_motion_check import BODY_FOR_PART, PLACED, ROBOT_XML, ROOT, SOURCE_FILES, SourceSnapshot, capture_manifest


def collision():
    return [{"a": "left_foot", "b": "right_foot", "volume_mm3": 100.0}]


class MotionGuardTests(unittest.TestCase):
    def setUp(self):
        self.limits = JointLimits(("a", "b"), np.radians([[-10, 10], [-10, 10]]))

    def test_mjcf_limits_reject_actual_audited_out_of_range_target(self):
        limits = JointLimits.from_mjcf(ROOT / ROBOT_XML)
        with np.load(ROOT / "docs/six_checks_2026-09-08/policy/forward.npz", allow_pickle=False) as data:
            self.assertEqual(tuple(data["joint_names"]), JOINT_NAMES)
            axis = limits.names.index("right_hip_roll")
            q = data["target_rad"][np.argmax(data["target_rad"][:, axis])]
        self.assertGreater(np.degrees(q[axis]), 23)
        with self.assertRaises(GuardRejected) as raised:
            limits.pose(q, "recorded_forward_target")
        self.assertEqual(raised.exception.reason, "joint_range")
        self.assertIn("right_hip_roll", [x["name"] for x in raised.exception.details["joints"]])

    def test_individually_legal_axes_still_reject_combined_collision(self):
        def geometry(q):
            return collision() if (np.degrees(q) > 0.5).all() else []
        guard = MotionGuard(self.limits, geometry)
        guard.check_pose(np.radians([0.75, 0]))
        guard.check_pose(np.radians([0, 0.75]))
        controls = np.zeros(2)
        with self.assertRaises(GuardRejected) as raised:
            guard.apply_target(controls, np.radians([0.75, 0.75]), observed=controls)
        self.assertEqual(raised.exception.reason, "cad_collision")
        np.testing.assert_array_equal(controls, [0, 0])

    def test_legal_endpoints_reject_collision_on_path(self):
        def geometry(q):
            return collision() if np.max(np.abs(np.degrees(q))) < 0.1 else []
        guard = MotionGuard(self.limits, geometry)
        start, end = np.radians([-1, -1]), np.radians([1, 1])
        guard.check_pose(start)
        guard.check_pose(end)
        controls = start.copy()
        with self.assertRaises(GuardRejected) as raised:
            guard.apply_target(controls, end, observed=start)
        self.assertEqual(raised.exception.reason, "cad_collision")
        self.assertEqual(raised.exception.details["fraction"], 0.5)
        np.testing.assert_array_equal(controls, start)

    def test_previous_target_path_is_checked_even_when_actual_already_at_target(self):
        def geometry(q):
            return collision() if abs(np.degrees(q[0])) < 0.1 else []
        guard = MotionGuard(self.limits, geometry)
        controls, end = np.radians([-1, 0]), np.radians([1, 0])
        before = controls.copy()
        with self.assertRaises(GuardRejected) as raised:
            guard.apply_target(controls, end, observed=end)
        self.assertEqual(raised.exception.details["label"], "previous_target_to_target")
        np.testing.assert_array_equal(controls, before)

    def test_actual_to_target_path_is_checked_independent_of_previous_target(self):
        def geometry(q):
            return collision() if abs(np.degrees(q[0])) < 0.1 else []
        guard = MotionGuard(self.limits, geometry)
        controls, actual = np.radians([1, 0]), np.radians([-1, 0])
        before = controls.copy()
        with self.assertRaises(GuardRejected) as raised:
            guard.apply_target(controls, controls.copy(), observed=actual)
        self.assertEqual(raised.exception.details["label"], "observed_to_target")
        np.testing.assert_array_equal(controls, before)

    def test_short_move_also_checks_midpoint(self):
        def geometry(q):
            return collision() if abs(np.degrees(q[0])) < 0.001 else []
        guard = MotionGuard(self.limits, geometry)
        with self.assertRaises(GuardRejected):
            guard.check_transition(np.radians([-0.1, 0]), np.radians([0.1, 0]))

    def test_accepted_target_is_written_unchanged_after_geometry_checks(self):
        seen, controls = [], np.zeros(2)
        def geometry(q):
            self.assertTrue(np.array_equal(controls, [0, 0]))
            seen.append(q)
            return []
        guard = MotionGuard(self.limits, geometry)
        target = np.radians([1.5, -1])
        receipt = guard.apply_target(controls, target, observed=np.zeros(2))
        self.assertEqual(receipt["accepted_commands"], 1)
        self.assertGreater(len(seen), 2)
        np.testing.assert_array_equal(controls, target)

    def test_nonfinite_or_wrong_shape_never_reaches_geometry_or_control(self):
        def geometry(q):
            self.fail("invalid input reached geometry")
        for bad in ([np.nan, 0], [np.inf, 0], [-np.inf, 0], [0], [[0, 0]]):
            with self.subTest(bad=bad):
                controls = np.zeros(2)
                guard = MotionGuard(self.limits, geometry)
                with self.assertRaises(GuardRejected):
                    guard.apply_target(controls, bad, observed=[0, 0])
                np.testing.assert_array_equal(controls, [0, 0])

    def test_integer_destination_cannot_silently_truncate_verified_target(self):
        controls = np.zeros(2, dtype=int)
        with self.assertRaises(GuardRejected):
            MotionGuard(self.limits, lambda q: []).apply_target(controls, [0.01, 0], observed=[0, 0])
        np.testing.assert_array_equal(controls, [0, 0])

    def test_boolean_exception_and_invalid_evidence_fail_closed(self):
        def broken(q):
            raise ValueError("kernel failed")
        for callback in (broken, lambda q: None, lambda q: [{"a": "x", "b": "y", "volume_mm3": -1}],
                         lambda q: [{"a": "x", "b": "y", "volume_mm3": np.nan}]):
            with self.subTest(callback=callback):
                controls = np.zeros(2)
                with self.assertRaises(GuardRejected) as raised:
                    MotionGuard(self.limits, callback).apply_target(controls, [0, 0], observed=[0, 0])
                self.assertEqual(raised.exception.reason, "geometry_unknown")
                np.testing.assert_array_equal(controls, [0, 0])

    def test_source_change_during_validation_prevents_control_write(self):
        state = {"changed": False}
        def sources():
            if state["changed"]:
                raise GuardRejected("geometry_source_changed_during_run")
        def geometry(q):
            state["changed"] = True
            return []
        controls = np.zeros(2)
        with self.assertRaises(GuardRejected):
            MotionGuard(self.limits, geometry, assert_sources=sources).apply_target(controls, [0.01, 0], observed=[0, 0])
        np.testing.assert_array_equal(controls, [0, 0])

    def test_cached_clearance_does_not_override_source_change(self):
        changed = False
        def sources():
            if changed:
                raise GuardRejected("source_changed")
        guard = MotionGuard(self.limits, lambda q: [], assert_sources=sources)
        guard.check_pose([0, 0])
        changed = True
        with self.assertRaises(GuardRejected):
            guard.check_pose([0, 0])

    def test_excessive_path_work_rejected_instead_of_skipped(self):
        guard = MotionGuard(self.limits, lambda q: [], max_samples_per_transition=3)
        with self.assertRaises(GuardRejected) as raised:
            guard.check_transition([0, 0], np.radians([9, 0]))
        self.assertEqual(raised.exception.reason, "transition_budget_exceeded")

    def test_policy_target_does_not_call_unsafe_upstream_writer(self):
        p = SimpleNamespace(use_delay=False, new_cmd_obs=True, default_pose=np.zeros(14), action_scale=1.0)
        np.testing.assert_array_equal(policy_target(p, np.ones(14)), np.ones(14))
        p.use_delay = True
        with self.assertRaises(GuardRejected):
            policy_target(p, np.ones(14))


class SourceSnapshotTests(unittest.TestCase):
    def fixture(self, root):
        paths = [*SOURCE_FILES, *(PLACED / (name + ".stl") for name in BODY_FOR_PART)]
        for rel in paths:
            path = root / rel
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_bytes(str(rel).encode())
        manifest = root / "manifest.json"
        manifest.write_text(json.dumps(capture_manifest(root)))
        return manifest

    def test_changed_geometry_before_loading_rejected(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            manifest = self.fixture(root)
            (root / PLACED / "sole.stl").write_bytes(b"changed")
            with self.assertRaises(GuardRejected) as raised:
                SourceSnapshot(root, manifest)
            self.assertEqual(raised.exception.reason, "geometry_source_changed")

    def test_head_generators_and_cached_geometry_are_bound(self):
        for rel in ("duckstructure/jaw.py", "duckstructure/head_top.py",
                    "duckstructure/electronics.py", "duckstructure/head_scale.py",
                    "duckstructure/data/head_scale_cache.npz"):
            with self.subTest(path=rel), tempfile.TemporaryDirectory() as tmp:
                root = Path(tmp)
                manifest = self.fixture(root)
                (root / rel).write_bytes(b"changed geometry input")
                with self.assertRaises(GuardRejected) as raised:
                    SourceSnapshot(root, manifest)
                self.assertEqual(raised.exception.reason, "geometry_source_changed")

    def test_new_geometry_input_during_run_is_rejected(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            source = SourceSnapshot(root, self.fixture(root))
            (root / "duckstructure/data/new_cut.stl").write_bytes(b"new cut")
            with self.assertRaises(GuardRejected) as raised:
                source.assert_unchanged()
            self.assertEqual(raised.exception.reason, "geometry_inventory_changed_during_run")

    def test_changed_geometry_after_loading_rejected_even_restored_mtime(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            manifest = self.fixture(root)
            source = SourceSnapshot(root, manifest)
            source.assert_unchanged()
            path = root / PLACED / "sole.stl"
            stamp = path.stat()
            path.write_bytes(b"x" * stamp.st_size)
            os.utime(path, ns=(stamp.st_atime_ns, stamp.st_mtime_ns))
            with self.assertRaises(GuardRejected):
                source.assert_unchanged()

    def test_unknown_and_missing_parts_cannot_silently_disappear(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            manifest = self.fixture(root)
            source = SourceSnapshot(root, manifest)
            (root / PLACED / "newpart.stl").write_bytes(b"new")
            with self.assertRaises(GuardRejected):
                source.assert_unchanged()
            (root / PLACED / "newpart.stl").unlink()
            (root / PLACED / "sole.stl").unlink()
            with self.assertRaises(GuardRejected):
                source.assert_unchanged()


class ReplayEntryTests(unittest.TestCase):
    def test_existing_replay_entry_stops_before_unverified_ctrl_or_physics_step(self):
        import render_policy
        limits = JointLimits.from_mjcf(ROOT / ROBOT_XML)
        model = SimpleNamespace(
            opt=SimpleNamespace(), actuator_trnid=np.c_[np.arange(14), np.zeros(14, dtype=int)],
            jnt_range=limits.radians, jnt_qposadr=np.r_[np.arange(7, 21), 0],
            jnt_dofadr=np.r_[np.arange(6, 20), 0])
        data = SimpleNamespace(qpos=np.zeros(21), qvel=np.zeros(20), ctrl=np.zeros(14), time=0)
        def forbidden_step(*args):
            self.fail("rejected target reached a physics step")
        mj = SimpleNamespace(
            MjModel=SimpleNamespace(from_xml_path=lambda path: model), MjData=lambda m: data,
            mjtIntegrator=SimpleNamespace(mjINT_IMPLICITFAST=1), mjtObj=SimpleNamespace(mjOBJ_JOINT=1),
            mj_id2name=lambda m, typ, idx: JOINT_NAMES[idx], mj_name2id=lambda *args: 14,
            mj_forward=lambda *args: None, mj_step=forbidden_step)
        ort = SimpleNamespace(SessionOptions=SimpleNamespace, InferenceSession=lambda *a, **k: None)
        case = self
        class Policy:
            def __init__(self, *args, **kwargs):
                self.default_pose = np.zeros(14)
                self.joint_qpos_indices = np.arange(7, 21)
                self.action_scale, self.use_delay, self.new_cmd_obs = 1, False, True
            def get_observations(self):
                return np.zeros(61)
            def set_vel_cmd(self, *args):
                pass
            def infer(self):
                return np.radians([1, *([0] * 13)])
            def apply_action(self, action):
                case.fail("replay called the unguarded upstream writer")
        class Scene:
            def __init__(self, *args):
                self.limits = limits
            def evidence(self):
                return {"test": "fake geometry, no hardware"}
            def guard(self):
                return MotionGuard(limits, lambda q: collision() if np.degrees(q[0]) > 0.5 else [])
        loader = SimpleNamespace(exec_module=lambda module: setattr(module, "PolicyInference", Policy))
        fake_spec = SimpleNamespace(loader=loader)
        args = SimpleNamespace(manifest=None, onnx=ROOT / "sim/runs/2026-09-06_baseline_xl330/baseline_xl330.onnx",
                               no_video=True, seconds=0.02)
        report = {"accepted_policy_targets": 0, "control_writes": 0}
        with patch.dict("sys.modules", {"mujoco": mj, "onnxruntime": ort}), \
                patch.object(render_policy, "CadScene", Scene), \
                patch.object(render_policy.importlib.util, "spec_from_file_location", return_value=fake_spec), \
                patch.object(render_policy.importlib.util, "module_from_spec", return_value=SimpleNamespace()):
            with self.assertRaises(GuardRejected) as raised:
                render_policy.run(args, report)
        self.assertEqual(raised.exception.reason, "cad_collision")
        np.testing.assert_array_equal(data.ctrl, np.zeros(14))
        self.assertEqual(report["accepted_policy_targets"], 0)
        self.assertEqual(report["control_writes"], 1)  # Only the checked HOME initialization.
        self.assertEqual(report["completed_physics_steps"], 0)


if __name__ == "__main__":
    unittest.main()
