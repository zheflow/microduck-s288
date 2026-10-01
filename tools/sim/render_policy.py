#!/usr/bin/env python3
"""无头回放 ONNX；在写入 MuJoCo ctrl 前执行真实 CAD 联合姿态/过渡预检。

用法：./.venv/bin/python tools/sim/render_policy.py <policy.onnx> <out.mp4>
离线只验回放：添加 --no-video --report <report.json>。
默认精确几何源快照失效、越界、碰撞或计算未知均退出 2，禁止静默跳过。
这是原 XL330 XML 动力学＋离散 CAD 预检，不是 S288 实机控制器。
"""
import argparse
import hashlib
import importlib.util
import json
import math
from pathlib import Path
import sys
import time

import numpy as np

from cad_motion_check import CadScene, DEFAULT_MANIFEST, ROOT
from motion_guard import GuardRejected, policy_target

ROBOT = ROOT / "upstream/microduck_rl/src/mjlab_microduck/robot/microduck"
INFER = ROOT / "upstream/microduck_rl/scripts/infer_policy.py"
SCHEDULE = [(0, 1.5, (0, 0, 0)), (1.5, 6, (0.15, 0, 0)), (6, 9.5, (0.15, 0, 0.8)),
            (9.5, 12, (0, 0.15, 0)), (12, 14.5, (-0.12, 0, 0)), (14.5, math.inf, (0, 0, 0))]


def command_at(t):
    return next(c for lo, hi, c in SCHEDULE if lo <= t < hi)


def run(args, report):
    import mujoco
    import onnxruntime as ort

    scene = CadScene(args.manifest)
    guard = scene.guard()
    report["geometry"] = scene.evidence()
    spec = importlib.util.spec_from_file_location("guarded_upstream_infer", INFER)
    ip = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(ip)
    model = mujoco.MjModel.from_xml_path(str(ROBOT / "scene_walk.xml"))
    model.opt.timestep = 0.005
    model.opt.integrator = mujoco.mjtIntegrator.mjINT_IMPLICITFAST
    model.opt.iterations = 10
    data = mujoco.MjData(model)
    # Limit this offline task's CPU use; restore the imported library immediately.
    options = ort.SessionOptions()
    options.intra_op_num_threads = options.inter_op_num_threads = 1
    factory = ort.InferenceSession
    try:
        ort.InferenceSession = lambda p: factory(str(p), sess_options=options, providers=["CPUExecutionProvider"])
        policy = ip.PolicyInference(model, data, walking_onnx_path=str(args.onnx.resolve()),
                                    new_cmd_obs=True, use_projected_gravity=True)
    finally:
        ort.InferenceSession = factory
    names = tuple(mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, int(j))
                  for j in model.actuator_trnid[:, 0])
    if names != scene.limits.names:
        raise GuardRejected("simulation_joint_order_mismatch", names=names)
    if not np.allclose(model.jnt_range[model.actuator_trnid[:, 0]], scene.limits.radians, atol=1e-12, rtol=0):
        raise GuardRejected("simulation_range_mismatch")
    policy.vel_max_x, policy.vel_min_x = 0.3, -0.3
    policy.vel_max_y, policy.vel_min_y, policy.vel_max_ang = 0.2, -0.2, 1.5
    free_joint = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "trunk_base_freejoint")
    qa, va = int(model.jnt_qposadr[free_joint]), int(model.jnt_dofadr[free_joint])
    qids = np.asarray(policy.joint_qpos_indices)
    data.qpos[qa:qa + 3], data.qpos[qa + 3:qa + 7] = [0, 0, 0.125], [1, 0, 0, 0]
    home = guard.check_pose(policy.default_pose, label="initial_home")
    data.qpos[qids] = home
    # Initial state is placed in HOME without pretending the placement is an
    # executed zero->HOME motion. The HOME target itself was checked above.
    guard.apply_target(data.ctrl, home, observed=home, previous_target=home)
    report["control_writes"] = 1
    mujoco.mj_forward(model, data)
    obs = policy.get_observations()
    if obs.shape != (61,) or not np.isfinite(obs).all():
        raise GuardRejected("invalid_observation", shape=list(obs.shape))
    report.update(onnx_sha256=hashlib.sha256(args.onnx.read_bytes()).hexdigest(),
                  physics_step_s=0.005, command_step_s=0.02, max_joint_linear_step_deg=0.5,
                  integrator="implicitfast", contact_solver_iterations=10,
                  completed_control_steps=0, completed_physics_steps=0)
    renderer = writer = None
    try:
        if not args.no_video:
            import imageio.v2 as imageio
            model.vis.global_.offwidth = max(args.width, 640)
            model.vis.global_.offheight = max(args.height, 480)
            renderer = mujoco.Renderer(model, height=args.height, width=args.width)
            camera = mujoco.MjvCamera()
            camera.type = mujoco.mjtCamera.mjCAMERA_FREE
            camera.distance, camera.azimuth, camera.elevation = 0.75, 140, -18
            args.out.parent.mkdir(parents=True, exist_ok=True)
            writer = imageio.get_writer(args.out, fps=50, codec="libx264", quality=8, macro_block_size=None)
        velocities = []
        for k in range(math.ceil(args.seconds / 0.02)):
            command = command_at(k * 0.02)
            policy.set_vel_cmd(*command)
            report["attempted_control_step"], report["simulation_time_s"] = k, float(data.time)
            action = policy.infer()
            target = policy_target(policy, action)
            # No call to upstream apply_action: that function writes unverified ctrl.
            guard.apply_target(data.ctrl, target, observed=data.qpos[qids])
            report["control_writes"] += 1
            report["accepted_policy_targets"] += 1
            for _ in range(4):
                previous_actual = data.qpos[qids].copy()
                mujoco.mj_step(model, data)
                report["completed_physics_steps"] += 1
                # Actual model dynamics can deviate from the intended interpolation.
                # A detected actual violation also terminates further integration.
                guard.check_transition(previous_actual, data.qpos[qids], label="physics_observation")
            report["completed_control_steps"] += 1
            report["geometry_evaluations"] = guard.geometry_evaluations
            if any(w.number for w in data.warning):
                raise GuardRejected("mujoco_warning", counts=[int(w.number) for w in data.warning])
            if data.qpos[qa + 2] < 0.06:
                raise GuardRejected("simulated_fall", time_s=float(data.time), trunk_z_mm=float(data.qpos[qa + 2] * 1000))
            vb = policy.quat_rotate_inverse(data.qpos[qa + 3:qa + 7].astype(np.float32),
                                            data.qvel[va:va + 3].astype(np.float32))
            velocities.append((float(vb[0]), float(vb[1]), float(data.qvel[va + 5])))
            if renderer is not None:
                mujoco.mj_forward(model, data)
                camera.lookat[:] = data.qpos[qa:qa + 3] + [0, 0, 0.02]
                renderer.update_scene(data, camera=camera)
                writer.append_data(renderer.render().copy())
            if k % 10 == 0:
                print(f"checked t={data.time:.2f}s, commands={k + 1}, CAD poses={guard.geometry_evaluations}", flush=True)
        report.update(status="sampled_preflight_passed", simulation_time_s=float(data.time),
                      mean_velocity_body=np.mean(velocities, axis=0).tolist())
    finally:
        if writer is not None:
            writer.close()
        if renderer is not None:
            renderer.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("onnx", type=Path)
    parser.add_argument("out", type=Path, nargs="?", default=ROOT / "sim/guarded_policy.mp4")
    parser.add_argument("--seconds", type=float, default=16)
    parser.add_argument("--width", type=int, default=800)
    parser.add_argument("--height", type=int, default=500)
    parser.add_argument("--no-video", action="store_true")
    parser.add_argument("--manifest", type=Path, default=DEFAULT_MANIFEST)
    parser.add_argument("--report", type=Path)
    args = parser.parse_args()
    if not math.isfinite(args.seconds) or args.seconds <= 0 or args.width <= 0 or args.height <= 0:
        parser.error("duration and image dimensions must be positive and finite")
    report_path = args.report or args.out.with_suffix(".preflight.json")
    report = {"status": "blocked", "scope": "original XL330 XML dynamics; sampled CAD preflight; no S288 hardware qualification",
              "onnx": str(args.onnx.resolve()), "source_scene": str(ROBOT / "scene_walk.xml"),
              "video": None if args.no_video else str(args.out.resolve()), "video_may_be_partial": True,
              "control_writes": 0, "accepted_policy_targets": 0,
              "completed_control_steps": 0, "completed_physics_steps": 0}
    start = time.monotonic()
    try:
        run(args, report)
    except GuardRejected as exc:
        report["rejection"] = exc.as_dict()
        print(str(exc), file=sys.stderr)
    except Exception as exc:
        report["rejection"] = {"reason": "preflight_unknown", "error": str(exc)}
        print(f"Preflight computation failed; no further control/integration: {exc}", file=sys.stderr)
    report["elapsed_wall_s"] = time.monotonic() - start
    report["video_may_be_partial"] = report["status"] != "sampled_preflight_passed"
    report_path.parent.mkdir(parents=True, exist_ok=True)
    report_path.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n")
    print("preflight:", report["status"], "report:", report_path)
    return 0 if report["status"] == "sampled_preflight_passed" else 2


if __name__ == "__main__":
    raise SystemExit(main())
