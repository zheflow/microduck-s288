"""Fail-closed, offline command preflight. Angles are radians at this API.

This detects joint-range and sampled combined-pose/transition violations. It is
not a continuous collision proof or an S288 hardware safety controller.
Geometry callbacks must return collision evidence, or raise on an unknown result.
"""
from dataclasses import dataclass
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np


JOINT_NAMES = (
    "left_hip_yaw", "left_hip_roll", "left_hip_pitch", "left_knee", "left_ankle",
    "neck_pitch", "head_pitch", "head_yaw", "head_roll", "right_hip_yaw",
    "right_hip_roll", "right_hip_pitch", "right_knee", "right_ankle",
)


class GuardRejected(RuntimeError):
    def __init__(self, reason, **details):
        self.reason = reason
        self.details = details
        super().__init__(f"CAD command preflight rejected: {reason}: {details}")

    def as_dict(self):
        return {"reason": self.reason, **self.details}


@dataclass(frozen=True)
class JointLimits:
    names: tuple
    radians: np.ndarray

    def __post_init__(self):
        names = tuple(self.names)
        values = np.array(self.radians, dtype=float, copy=True)
        if (not names or len(set(names)) != len(names)
                or values.shape != (len(names), 2) or not np.isfinite(values).all()
                or not np.all(values[:, 0] < values[:, 1])):
            raise GuardRejected("invalid_joint_limits")
        values.setflags(write=False)
        object.__setattr__(self, "names", names)
        object.__setattr__(self, "radians", values)

    @classmethod
    def from_mjcf(cls, path):
        root = ET.parse(Path(path)).getroot()
        compiler = root.find("compiler")
        if compiler is None or compiler.get("angle") != "radian":
            raise GuardRejected("unsupported_mjcf_angle_units")
        names = tuple(x.get("joint") for x in root.findall("actuator/position"))
        if names != JOINT_NAMES:
            raise GuardRejected("joint_order_changed", found=names, expected=JOINT_NAMES)
        joints = {x.get("name"): x for x in root.findall("worldbody//joint")}
        ranges = []
        for name in names:
            j = joints.get(name)
            if j is None or j.get("type", "hinge") != "hinge" or j.get("range") is None:
                raise GuardRejected("unsupported_mjcf_joint", joint=name)
            ranges.append([float(v) for v in j.get("range").split()])
        return cls(names, ranges)

    def pose(self, values, label):
        try:
            pose = np.array(values, dtype=float, copy=True)
        except (ValueError, TypeError) as exc:
            raise GuardRejected("invalid_pose", label=label) from exc
        if pose.shape != (len(self.names),) or not np.isfinite(pose).all():
            raise GuardRejected("nonfinite_or_wrong_shape", label=label, shape=list(pose.shape))
        # Numerical roundoff only; this is not a manufacturing/control allowance.
        bad = (pose < self.radians[:, 0] - 1e-9) | (pose > self.radians[:, 1] + 1e-9)
        if bad.any():
            raise GuardRejected("joint_range", label=label, joints=[
                {"name": self.names[i], "angle_deg": float(np.degrees(pose[i])),
                 "limits_deg": np.degrees(self.radians[i]).tolist()}
                for i in np.flatnonzero(bad)
            ])
        return pose


class MotionGuard:
    """Check every target and interpolated transition before mutating controls.

    Both measured->target and previous-target->target are checked. The finite
    joint-linear samples are deliberately reported as samples, not certification
    of every possible actuator path, tracking error, inertia, or cable routing.
    """

    def __init__(self, limits, collisions, *, assert_sources=None, max_step_deg=0.5,
                 max_samples_per_transition=5000):
        if not math.isfinite(max_step_deg) or not 0 < max_step_deg <= 0.5:
            raise GuardRejected("invalid_sample_step", permitted="0 < step <= 0.5 degrees")
        if not isinstance(max_samples_per_transition, int) or max_samples_per_transition < 3:
            raise GuardRejected("invalid_sample_budget")
        self.limits = limits
        self.collisions = collisions
        self.assert_sources = assert_sources or (lambda: None)
        self.max_step_deg = max_step_deg
        self.max_samples_per_transition = max_samples_per_transition
        self.geometry_evaluations = 0
        self.accepted_commands = 0
        self._cache = {}

    def _source_check(self):
        try:
            self.assert_sources()
        except GuardRejected:
            raise
        except Exception as exc:
            raise GuardRejected("source_check_failed", error=str(exc)) from exc

    def _check(self, pose, label, fraction=None):
        key = pose.tobytes()
        if key not in self._cache:
            try:
                rows = self.collisions(pose.copy())
                if not isinstance(rows, list):
                    raise ValueError("geometry callback must return a list")
                for row in rows:
                    if (not isinstance(row, dict) or not row.get("a") or not row.get("b")
                            or not math.isfinite(float(row["volume_mm3"]))
                            or float(row["volume_mm3"]) < 0):
                        raise ValueError("invalid collision evidence")
            except Exception as exc:
                raise GuardRejected("geometry_unknown", label=label, error=str(exc)) from exc
            self.geometry_evaluations += 1
            if len(self._cache) >= 4096:
                self._cache.clear()
            self._cache[key] = rows
        rows = self._cache[key]
        if rows:
            raise GuardRejected("cad_collision", label=label, fraction=fraction,
                                degrees=dict(zip(self.limits.names, np.degrees(pose).tolist())),
                                collisions=rows)

    def check_pose(self, pose, *, label="pose"):
        self._source_check()
        result = self.limits.pose(pose, label)
        self._check(result, label)
        self._source_check()
        return result

    def _transition(self, start, target, label):
        movement = float(np.degrees(np.max(np.abs(target - start))))
        # Always include a midpoint for a nonzero move, even below the step size.
        subdivisions = 0 if movement == 0 else max(2, math.ceil(movement / self.max_step_deg))
        count = subdivisions + 1
        if count > self.max_samples_per_transition:
            raise GuardRejected("transition_budget_exceeded", label=label, samples=count)
        self._check(start, label, 0.0)
        if subdivisions:
            # Reject an unsafe endpoint first, then inspect the route to it.
            self._check(target, label, 1.0)
            for i in range(1, subdivisions):
                f = i / subdivisions
                self._check(start * (1 - f) + target * f, label, f)
        return count

    def check_transition(self, start, target, *, label="transition"):
        self._source_check()
        q0, q1 = self.limits.pose(start, label + "_start"), self.limits.pose(target, label + "_target")
        count = self._transition(q0, q1, label)
        self._source_check()
        return {"samples": count, "max_step_deg": self.max_step_deg,
                "scope": "sampled_joint_linear_transition"}

    def apply_target(self, controls, target, *, observed, previous_target=None):
        """The sole write occurs after every check succeeds; rejection leaves ctrl unchanged."""
        self._source_check()
        q1 = self.limits.pose(target, "target")
        actual = self.limits.pose(observed, "observed")
        old = self.limits.pose(controls if previous_target is None else previous_target, "previous_target")
        if (not isinstance(controls, np.ndarray) or controls.shape != q1.shape
                or not np.issubdtype(controls.dtype, np.floating) or not controls.flags.writeable):
            raise GuardRejected("invalid_control_destination")
        n_actual = self._transition(actual, q1, "observed_to_target")
        n_target = self._transition(old, q1, "previous_target_to_target")
        self._source_check()
        controls[:] = q1
        self.accepted_commands += 1
        return {"observed_path_samples": n_actual, "previous_target_path_samples": n_target,
                "max_step_deg": self.max_step_deg, "accepted_commands": self.accepted_commands}


def policy_target(policy, action):
    """Mirror the explicitly supported upstream action formula, without writing data.ctrl."""
    if policy.use_delay or not policy.new_cmd_obs:
        raise GuardRejected("unsupported_policy_mode", supported="no delay, new_cmd_obs=True")
    raw = np.asarray(action, dtype=float)
    base = np.asarray(policy.default_pose, dtype=float)
    scale = float(policy.action_scale)
    if (raw.shape != base.shape or raw.shape != (len(JOINT_NAMES),)
            or not np.isfinite(raw).all() or not np.isfinite(base).all() or not math.isfinite(scale)):
        raise GuardRejected("invalid_policy_action")
    return base + raw * scale
