#!/usr/bin/env python3
"""测量 ONNX 策略实际用到的关节工作空间包络，并与 CAD 几何安全区对比。

对每个策略、每个指令工况在 MuJoCo 里回放一段，逐控制步记录：
  * 实际关节角  data.qpos[joint_qpos_indices]   —— 碰撞发生在这里
  * 策略目标角  data.ctrl                        —— 软限位加在这里
  * 实际关节角速度 data.qvel[joint_qvel_indices]（MuJoCo 精确值）
    以及目标角速率（相邻控制步 ctrl 差分 / 0.02 s）

不引入 CadScene / motion_guard：本脚本只做包络统计，不依赖 CAD 网格快照。
几何安全区从 cad/duck_s288/audit_summary.json 的扫掠记录里重算（读一次并记 sha256），
读不到就用内嵌的同源表，JSON 里会标明来源。

用法：
    ./.venv/bin/python tools/sim/policy_envelope.py                 # 跑全部
    ./.venv/bin/python tools/sim/policy_envelope.py --policy alpha_walking
    ./.venv/bin/python tools/sim/policy_envelope.py --list

范围声明：原版 XL330 XML 动力学 + 上游 infer_policy 的观测/指令构建。
不是 S288 实机数据，不含实物公差、负载、总线延迟。
"""
from __future__ import annotations

import argparse
import contextlib
import hashlib
import importlib.util
import io
import json
import math
import re
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
ROBOT = ROOT / "upstream/microduck_rl/src/mjlab_microduck/robot/microduck"
INFER = ROOT / "upstream/microduck_rl/scripts/infer_policy.py"
POLICY_DIR = ROOT / "upstream/microduck/policies"
AUDIT = ROOT / "cad/duck_s288/audit_summary.json"
DEFAULT_OUT = ROOT / "docs/workspace_2026-09-09/policy_envelope.json"

# 上游 infer_policy.py 的控制常量（main(): decimation=4, model.opt.timestep=0.005）
PHYSICS_DT = 0.005
DECIMATION = 4
CONTROL_DT = PHYSICS_DT * DECIMATION           # 0.02 s = 50 Hz
INTERFERENCE_MM3 = 0.05                        # 本库的干涉判据（mm³）
FALL_TRUNK_Z_M = 0.06                          # 与 render_policy.py 的跌倒判据一致

# 指令上限的**唯一来源**：upstream/microduck_rl/scripts/infer_policy.py
#   main() 的 "Per-mode velocity command limits matching training ranges" 分支
WALK_VEL = dict(x=(-0.3, 0.3), y=(-0.2, 0.2), ang=(-1.5, 1.5))          # 行走机型
ROLLER_VEL = dict(x=(-0.5, 0.6), y=(0.0, 0.0), ang=(-1.0, 1.0))         # 轮滑机型
#   PolicyInference.__init__ 注释 "Final per-joint training caps:
#   neck/head_pitch ±1.1, head_yaw ±1.4, head_roll ±0.31"（rad）
HEAD_CAP = dict(neck_pitch=1.1, head_pitch=1.1, head_yaw=1.4, head_roll=0.31)
#   infer_policy.py 模块常量 BODY_CMD_MAX_Z / BODY_CMD_MAX_XY / BODY_CMD_MAX_ANGLE
BODY_CAP = dict(z=0.03, xy=0.02, angle=math.radians(30.0))

# 内嵌兜底：与 audit_summary.json 扫掠记录同源（单关节动、其余归零、2.5° 网格、
# 阈值 0.05 mm³）。first_contact = 第一个出现 >0.05 mm³ 的网格点（= 交接表里引用的边界）；
# last_clear = 该点再退一个 2.5° 网格，即最后一个全对无干涉的采样点。
FALLBACK_SAFE = {
    "left_hip_yaw":    dict(mjcf_range=[-25.0, 30.0], first_contact=[-25.0, 30.0], last_clear=[-25.0, 30.0]),
    "left_hip_roll":   dict(mjcf_range=[-22.0, 22.0], first_contact=[-22.0, 22.0], last_clear=[-22.0, 22.0]),
    "left_hip_pitch":  dict(mjcf_range=[-90.0, 90.0], first_contact=[-90.0, 90.0], last_clear=[-90.0, 90.0]),
    "left_knee":       dict(mjcf_range=[-90.0, 90.0], first_contact=[-90.0, 67.5], last_clear=[-90.0, 65.0]),
    "left_ankle":      dict(mjcf_range=[-90.0, 90.0], first_contact=[-40.0, 40.0], last_clear=[-37.5, 37.5]),
    "neck_pitch":      dict(mjcf_range=[-90.0, 60.0], first_contact=[-67.5, 50.0], last_clear=[-65.0, 47.5]),
    "head_pitch":      dict(mjcf_range=[-90.0, 90.0], first_contact=[-67.5, 90.0], last_clear=[-65.0, 90.0]),
    "head_yaw":        dict(mjcf_range=[-170.0, 170.0], first_contact=[-170.0, 170.0], last_clear=[-170.0, 170.0]),
    "head_roll":       dict(mjcf_range=[-25.0, 25.0], first_contact=[-25.0, 25.0], last_clear=[-25.0, 25.0]),
    "right_hip_yaw":   dict(mjcf_range=[-30.0, 25.0], first_contact=[-30.0, 25.0], last_clear=[-30.0, 25.0]),
    "right_hip_roll":  dict(mjcf_range=[-22.0, 22.0], first_contact=[-22.0, 22.0], last_clear=[-22.0, 22.0]),
    "right_hip_pitch": dict(mjcf_range=[-90.0, 90.0], first_contact=[-90.0, 90.0], last_clear=[-90.0, 90.0]),
    "right_knee":      dict(mjcf_range=[-90.0, 90.0], first_contact=[-67.5, 90.0], last_clear=[-65.0, 90.0]),
    "right_ankle":     dict(mjcf_range=[-90.0, 90.0], first_contact=[-40.0, 40.0], last_clear=[-37.5, 37.5]),
}


# --------------------------------------------------------------------------- #
# 几何安全区
# --------------------------------------------------------------------------- #
def load_safe_zone():
    """从 audit_summary.json 的扫掠记录重算安全区；失败则用内嵌表。"""
    try:
        raw = AUDIT.read_bytes()
        data = json.loads(raw)
        step = float(data["step_deg"])
        worst = {}
        for rec in data["sweep"]:
            vol = float(rec["volume_mm3"])
            if vol <= INTERFERENCE_MM3:
                continue
            key = (rec["joint"], round(float(rec["angle_deg"]), 4))
            if vol > worst.get(key, (0.0, None))[0]:
                worst[key] = (vol, f'{rec["a"]}×{rec["b"]}')
        zone = {}
        for jd in data["joints"]:
            name = jd["name"]
            lo, hi = (round(float(v), 6) for v in jd["range_deg"])
            bad = sorted(a for (j, a) in worst if j == name)
            neg = [a for a in bad if a < 0]
            pos = [a for a in bad if a > 0]
            fc_lo = max(neg) if neg else lo
            fc_hi = min(pos) if pos else hi
            zone[name] = {
                "mjcf_range": [lo, hi],
                "first_contact": [fc_lo, fc_hi],
                "last_clear": [fc_lo + step if neg else lo, fc_hi - step if pos else hi],
                "worst_at_first_contact_lo": (
                    {"pair": worst[(name, fc_lo)][1], "volume_mm3": worst[(name, fc_lo)][0]}
                    if neg else None),
                "worst_at_first_contact_hi": (
                    {"pair": worst[(name, fc_hi)][1], "volume_mm3": worst[(name, fc_hi)][0]}
                    if pos else None),
                "colliding_grid_samples": len(bad),
            }
        prov = {"source": str(AUDIT.relative_to(ROOT)),
                "sha256": hashlib.sha256(raw).hexdigest(),
                "mtime_epoch": AUDIT.stat().st_mtime,
                "step_deg": step, "scope": data.get("scope"),
                "sweep_records": len(data["sweep"]),
                "threshold_mm3": INTERFERENCE_MM3}
        return zone, prov
    except Exception as exc:                                     # noqa: BLE001
        return ({k: dict(v) for k, v in FALLBACK_SAFE.items()},
                {"source": "embedded_fallback", "reason": repr(exc)})


# --------------------------------------------------------------------------- #
# 工况定义
# --------------------------------------------------------------------------- #
def twist_cases(vel):
    xlo, xhi = vel["x"]
    ylo, yhi = vel["y"]
    alo, ahi = vel["ang"]
    cases = [
        ("stand_still", (0.0, 0.0, 0.0)),
        ("max_forward", (xhi, 0.0, 0.0)),
        ("max_backward", (xlo, 0.0, 0.0)),
        ("max_turn_left", (0.0, 0.0, ahi)),
        ("max_turn_right", (0.0, 0.0, alo)),
        ("forward_plus_turn", (xhi, 0.0, ahi)),
        ("backward_plus_turn", (xlo, 0.0, alo)),
    ]
    if ylo != 0.0 or yhi != 0.0:
        cases += [("max_strafe_left", (0.0, yhi, 0.0)),
                  ("max_strafe_right", (0.0, ylo, 0.0)),
                  ("forward_plus_strafe", (xhi, yhi, 0.0)),
                  ("all_axes_max", (xhi, yhi, ahi))]
    return [{"name": n, "twist": t} for n, t in cases]


def head_cases(moving_twist=None):
    """头部姿态指令极值 —— neck_pitch / head_pitch 正是安全区最窄的两个关节。

    只对 ONNX metadata 的 command_names 里含 'head_pose' 的策略使用：其余策略
    训练时头部指令槽恒为零，喂非零值是分布外输入（实测会把仿真喂到发散）。
    """
    n, p, y, r = (HEAD_CAP["neck_pitch"], HEAD_CAP["head_pitch"],
                  HEAD_CAP["head_yaw"], HEAD_CAP["head_roll"])
    zero = (0.0, 0.0, 0.0)
    cases = [
        {"name": "head_pitch_down_max", "twist": zero, "head": (-n, -p, 0.0, 0.0)},
        {"name": "head_pitch_up_max", "twist": zero, "head": (n, p, 0.0, 0.0)},
        {"name": "head_yaw_max", "twist": zero, "head": (0.0, 0.0, y, 0.0)},
        {"name": "head_roll_max", "twist": zero, "head": (0.0, 0.0, 0.0, r)},
        {"name": "head_all_max", "twist": zero, "head": (n, p, y, r)},
        {"name": "head_all_min", "twist": zero, "head": (-n, -p, -y, -r)},
    ]
    if moving_twist is not None:
        cases += [
            {"name": "head_all_max_at_max_twist", "twist": moving_twist, "head": (n, p, y, r)},
            {"name": "head_all_min_at_max_twist", "twist": moving_twist, "head": (-n, -p, -y, -r)},
        ]
    return cases


def body_cases():
    z, xy, a = BODY_CAP["z"], BODY_CAP["xy"], BODY_CAP["angle"]
    return [
        {"name": "body_z_up", "body": (0.0, 0.0, z, 0.0, 0.0, 0.0)},
        {"name": "body_z_down", "body": (0.0, 0.0, -z, 0.0, 0.0, 0.0)},
        {"name": "body_pitch_fwd", "body": (0.0, 0.0, 0.0, 0.0, a, 0.0)},
        {"name": "body_pitch_back", "body": (0.0, 0.0, 0.0, 0.0, -a, 0.0)},
        {"name": "body_roll_max", "body": (0.0, 0.0, 0.0, a, 0.0, 0.0)},
        {"name": "body_xy_max", "body": (xy, xy, 0.0, 0.0, 0.0, 0.0)},
        {"name": "body_all_max", "body": (xy, xy, z, a, a, a)},
        {"name": "body_all_min", "body": (-xy, -xy, -z, -a, -a, -a)},
    ]


def repeat_events(kind, arg, first, period, seconds):
    return [{"t": round(float(t), 3), "kind": kind, "arg": arg}
            for t in np.arange(first, seconds - 0.5, period)]


def build_specs(seconds):
    """策略 → (加载槽位, 场景, 工况列表)。槽位/场景映射照抄 infer_policy.py main()。"""
    s = seconds
    return {
        "alpha_walking": dict(
            slot="walking", scene="scene.xml", roller=False,
            cases=twist_cases(WALK_VEL)
            + head_cases(moving_twist=(WALK_VEL["x"][1], WALK_VEL["y"][1], WALK_VEL["ang"][1]))),
        "alpha_stand": dict(
            slot="standing", scene="scene.xml", roller=False,
            cases=[{"name": "hold_stand"}] + body_cases() + head_cases()),
        "alpha_sitstand": dict(
            slot="sitstand", scene="scene.xml", roller=False,
            cases=head_cases() + [{"name": "hold_stand_flag0"},
                   {"name": "sit_down", "events": [{"t": 1.0, "kind": "sit", "arg": None}]},
                   {"name": "sit_then_stand",
                    "events": [{"t": 1.0, "kind": "sit", "arg": None},
                               {"t": round(1.0 + (s - 1.0) / 2, 3), "kind": "sit", "arg": None}]}]),
        "alpha_ground_pick": dict(
            slot="ground_pick", scene="scene.xml", roller=False, companion="alpha_stand",
            cases=[{"name": "ground_pick_cycles",
                    "events": repeat_events("ground_pick", None, 1.0, 4.0, s)}]),
        "ball_kick_left": dict(
            slot="behavior", behavior="kick_left", scene="scene_ball.xml", roller=False,
            companion="alpha_stand",
            cases=[{"name": "kick_left_repeat",
                    "events": repeat_events("behavior", "kick_left", 1.0, 5.0, s)}]),
        "ball_kick_right": dict(
            slot="behavior", behavior="kick_right", scene="scene_ball.xml", roller=False,
            companion="alpha_stand",
            cases=[{"name": "kick_right_repeat",
                    "events": repeat_events("behavior", "kick_right", 1.0, 5.0, s)}]),
        "roulade": dict(
            slot="behavior", behavior="roulade", scene="scene.xml", roller=False,
            companion="alpha_stand",
            cases=[{"name": "roulade_repeat",
                    "events": repeat_events("behavior", "roulade", 1.0, 4.0, s)}]),
        "roller": dict(
            slot="walking", scene="scene_rollers.xml", roller=True,
            cases=twist_cases(ROLLER_VEL)),
        "roller_crouch": dict(
            slot="ground_pick", scene="scene_rollers.xml", roller=True, companion="roller",
            cases=[{"name": "crouch_cycles",
                    "events": repeat_events("ground_pick", None, 1.0, 4.0, s)}]),
    }


# --------------------------------------------------------------------------- #
# 仿真
# --------------------------------------------------------------------------- #
def load_infer_module():
    spec = importlib.util.spec_from_file_location("upstream_infer_policy", INFER)
    module = importlib.util.module_from_spec(spec)
    with contextlib.redirect_stdout(io.StringIO()):
        spec.loader.exec_module(module)
    return module


def torque_limit_nm(current_limit):
    """infer_policy.py main() 的 XL330 电流限幅：forcerange = ±kt*I_max。"""
    if current_limit is None or current_limit <= 0:
        return None, None
    from bam.model import load_model
    kt = float(load_model(motor_name="xl330", model="m6").kt.value)
    return kt * current_limit, kt


def make_policy(ip, model, data, spec, name):
    kwargs = dict(new_cmd_obs=True, use_projected_gravity=True)
    onnx = str((POLICY_DIR / f"{name}.onnx").resolve())
    companion = spec.get("companion")
    comp = str((POLICY_DIR / f"{companion}.onnx").resolve()) if companion else None
    slot = spec["slot"]
    if slot == "walking":
        kwargs["walking_onnx_path"] = onnx
    elif slot == "standing":
        kwargs["standing_onnx_path"] = onnx
    elif slot == "sitstand":
        kwargs["sitstand_onnx_path"] = onnx
    elif slot == "ground_pick":
        kwargs["ground_pick_onnx_path"] = onnx
        # _end_ground_pick() 需要一个 walking 或 standing 会话接管
        kwargs["walking_onnx_path" if companion == "roller" else "standing_onnx_path"] = comp
    elif slot == "behavior":
        kwargs[f'{spec["behavior"]}_onnx_path'] = onnx
        kwargs["standing_onnx_path"] = comp
    else:
        raise ValueError(f"unknown slot {slot}")
    with contextlib.redirect_stdout(io.StringIO()):
        policy = ip.PolicyInference(model, data, **kwargs)
        vel = ROLLER_VEL if spec["roller"] else WALK_VEL
        policy.vel_min_x, policy.vel_max_x = vel["x"]
        policy.vel_min_y, policy.vel_max_y = vel["y"]
        policy.vel_max_ang = vel["ang"][1]
    policy._envelope_initial = (policy.current_policy, policy.ort_session)
    return policy


def reset_policy(policy):
    """把 PolicyInference 恢复到刚构造完的状态（会话不重建，省 ONNX 加载时间）。"""
    policy.current_policy, policy.ort_session = policy._envelope_initial
    policy.input_name = policy.ort_session.get_inputs()[0].name
    policy.output_name = policy.ort_session.get_outputs()[0].name
    policy.last_action[:] = 0.0
    policy.vel_cmd[:] = 0.0
    policy.head_offset[:] = 0.0
    policy.body_cmd[:] = 0.0
    policy.sit_mode = False
    policy.slope_mode = False
    policy.head_mode = False
    policy.body_pose_mode = False
    policy.ground_pick_mode = False
    policy.ground_pick_phase = 0.0
    policy.behavior_mode = None
    policy.behavior_time_left = 0.0
    if policy.action_buffer is not None:
        for buf in policy.action_buffer:
            buf[:] = 0.0
        policy.buffer_index = 0
    with contextlib.redirect_stdout(io.StringIO()):
        policy._update_command()


def run_case(mujoco, model, data, policy, spec, case, seconds, settle_s, qa):
    """回放一个工况，返回逐控制步的原始记录（弧度）。"""
    n_joints = model.nu
    qids = np.asarray(policy.joint_qpos_indices)
    vids = np.asarray(policy.joint_qvel_indices)

    reset_policy(policy)
    mujoco.mj_resetData(model, data)
    data.qpos[qa:qa + 3] = [0.0, 0.0, 0.1385 if spec["roller"] else 0.125]
    data.qpos[qa + 3:qa + 7] = [1.0, 0.0, 0.0, 0.0]
    data.qpos[qids] = policy.default_pose
    data.ctrl[:] = policy.default_pose
    mujoco.mj_forward(model, data)

    twist = case.get("twist")
    head = case.get("head")
    body = case.get("body")
    events = sorted(case.get("events", []), key=lambda e: e["t"])
    applied_cmd = False
    ev_i = 0

    steps = int(round(seconds / CONTROL_DT))
    qpos_log = np.zeros((steps, n_joints))
    ctrl_log = np.zeros((steps, n_joints))
    qvel_log = np.zeros((steps, n_joints))
    trunk_z = np.zeros(steps)
    active = []
    warning = None
    diverged = None
    done = steps

    with contextlib.redirect_stdout(io.StringIO()):
        for k in range(steps):
            t = k * CONTROL_DT
            if not applied_cmd and t >= settle_s:
                if head is not None:
                    policy.head_offset[:] = np.asarray(head, dtype=np.float32)
                if body is not None:
                    policy.body_cmd[:] = np.asarray(body, dtype=np.float32)
                policy.set_vel_cmd(*(twist if twist is not None else (0.0, 0.0, 0.0)))
                applied_cmd = True
            while ev_i < len(events) and t >= events[ev_i]["t"]:
                ev = events[ev_i]
                if ev["kind"] == "sit":
                    policy.toggle_sit()
                elif ev["kind"] == "ground_pick":
                    policy.trigger_ground_pick()
                elif ev["kind"] == "behavior":
                    policy.trigger_behavior(ev["arg"])
                ev_i += 1

            policy.update_ground_pick_phase(CONTROL_DT)
            policy.update_behavior(CONTROL_DT)
            action = policy.infer()
            if not np.isfinite(action).all():
                diverged = {"stage": "policy_action", "control_step": k, "time_s": round(t, 3)}
                done = k
                break
            policy.apply_action(action)

            ctrl_log[k] = data.ctrl[:n_joints]
            active.append(policy.current_policy)
            for _ in range(DECIMATION):
                mujoco.mj_step(model, data)
            qpos_log[k] = data.qpos[qids]
            qvel_log[k] = data.qvel[vids]
            trunk_z[k] = data.qpos[qa + 2]
            if warning is None and any(w.number for w in data.warning):
                warning = {"control_step": k, "time_s": round(t, 3),
                           "counts": [int(w.number) for w in data.warning]}
            if not (np.isfinite(qpos_log[k]).all() and np.isfinite(qvel_log[k]).all()
                    and np.isfinite(trunk_z[k])):
                # 数值发散：该步及之后的数字没有物理意义，整段截断并标记，
                # 绝不让 NaN/爆值进入包络统计。
                diverged = {"stage": "physics_state", "control_step": k, "time_s": round(t, 3)}
                done = k
                break

    return dict(qpos=qpos_log[:done], ctrl=ctrl_log[:done], qvel=qvel_log[:done],
                trunk_z=trunk_z[:done], active=active[:done],
                warning=warning, diverged=diverged, steps_planned=steps)


def stats_block(values):
    if values.size == 0:
        return None
    return {"min": round(float(np.min(values)), 4),
            "max": round(float(np.max(values)), 4),
            "p0.1": round(float(np.percentile(values, 0.1)), 4),
            "p99.9": round(float(np.percentile(values, 99.9)), 4),
            "abs_max": round(float(np.max(np.abs(values))), 4),
            "mean": round(float(np.mean(values)), 4),
            "n": int(values.size)}


def outside_fraction(values, lo, hi):
    n = values.size
    if n == 0:
        return {"below": 0.0, "above": 0.0}
    return {"below": round(float(np.mean(values < lo)), 6),
            "above": round(float(np.mean(values > hi)), 6)}


# --------------------------------------------------------------------------- #
def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--policy", action="append", default=None,
                        help="只跑指定策略（可重复）。默认跑全部 9 个 microduck 策略。")
    parser.add_argument("--seconds", type=float, default=14.0,
                        help="每个工况的仿真时长（含 settle 段），默认 14 s")
    parser.add_argument("--settle", type=float, default=1.0,
                        help="开头丢弃的落地稳定时长（秒），默认 1.0")
    parser.add_argument("--current-limit", type=float, default=1.75,
                        help="XL330 固件电流限幅 [A]，照抄 infer_policy.py 默认 1.75；<=0 关闭")
    parser.add_argument("--out", type=Path, default=DEFAULT_OUT)
    parser.add_argument("--dump-steps", type=Path, default=None,
                        help="（2026-09-16）把每个工况逐控制步的实际关节角（度，settle 之后）存成 npz：键 `<policy>/<case>/qpos_deg`、"
                             "`<policy>/<case>/prefall_n`、`joint_names`。README A2：包络 json 只有各关节 min/max（轴对齐盒），"
                             "Gate L6 box_collisions 是上界；tools/sim/policy_pose_collisions.py 用这份逐步元组按真实姿态判碰撞。")
    parser.add_argument("--list", action="store_true", help="列出策略与工况后退出")
    args = parser.parse_args()

    if args.settle < 0 or args.seconds <= args.settle:
        parser.error("--settle must be >= 0 and --seconds must exceed --settle")

    specs = build_specs(args.seconds)
    if args.list:
        for name, spec in specs.items():
            print(f"{name}  slot={spec['slot']}  scene={spec['scene']}  "
                  f"cases={[c['name'] for c in spec['cases']]}")
        return 0

    wanted = args.policy or list(specs)
    unknown = [p for p in wanted if p not in specs]
    if unknown:
        parser.error(f"unknown policy: {unknown}; known: {list(specs)}")

    import mujoco
    import onnxruntime as ort

    options = ort.SessionOptions()
    options.intra_op_num_threads = options.inter_op_num_threads = 1
    factory = ort.InferenceSession
    ort.InferenceSession = lambda p: factory(str(p), sess_options=options,
                                             providers=["CPUExecutionProvider"])
    ip = load_infer_module()

    safe_zone, safe_prov = load_safe_zone()
    tq, kt = torque_limit_nm(args.current_limit)
    keep_from = int(round(args.settle / CONTROL_DT))
    dump = {} if args.dump_steps else None

    report = {
        "generated": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
        "tool": str(Path(__file__).resolve().relative_to(ROOT)),
        "scope": ("original XL330 XML dynamics + upstream infer_policy observation/command "
                  "construction; joint-workspace envelope only. NOT S288 hardware data: no "
                  "part tolerance, no payload, no bus latency, no backlash."),
        "sim": {"physics_dt_s": PHYSICS_DT, "decimation": DECIMATION,
                "control_dt_s": CONTROL_DT, "control_hz": round(1.0 / CONTROL_DT, 3),
                "seconds_per_case": args.seconds,
                "settle_discarded_s": args.settle,
                "settle_discarded_control_steps": keep_from,
                "settle_rationale": (
                    "For t < settle the trunk is dropping from the upstream initial height "
                    "(125 mm walking / 138.5 mm roller) onto the floor with a zero command; "
                    "those samples measure the drop, not the commanded workspace. The command "
                    "step at t = settle and its full transient are KEPT."),
                "current_limit_A": args.current_limit,
                "xl330_kt": kt, "torque_limit_Nm": tq,
                "torque_limit_source": "infer_policy.py main() --current-limit default 1.75 A"},
        "command_limit_sources": {
            "walk_twist_m_s_rad_s": {"value": WALK_VEL,
                "source": "infer_policy.py main(), 'Per-mode velocity command limits matching "
                          "training ranges', non-roller branch"},
            "roller_twist_m_s_rad_s": {"value": ROLLER_VEL,
                "source": "infer_policy.py main(), same block, roller branch"},
            "head_pose_rad": {"value": HEAD_CAP,
                "source": "infer_policy.py PolicyInference.__init__ comment: 'Final per-joint "
                          "training caps: neck/head_pitch +/-1.1, head_yaw +/-1.4, head_roll +/-0.31'"},
            "body_pose_m_rad": {"value": BODY_CAP,
                "source": "infer_policy.py module constants BODY_CMD_MAX_Z / BODY_CMD_MAX_XY / "
                          "BODY_CMD_MAX_ANGLE"},
        },
        "geometry_safe_zone": safe_zone,
        "geometry_safe_zone_provenance": safe_prov,
        "runs": [], "policy_envelope": {}, "margins": {},
        "violations": [], "not_measured": [],
    }

    started = time.monotonic()
    for pname in wanted:
        spec = specs[pname]
        model = mujoco.MjModel.from_xml_path(str(ROBOT / spec["scene"]))
        model.opt.timestep = PHYSICS_DT
        report["sim"].setdefault("integrator_per_scene", {})[spec["scene"]] = int(model.opt.integrator)
        if tq is not None:
            model.actuator_forcerange[:, 0] = -tq
            model.actuator_forcerange[:, 1] = tq
            model.actuator_forcelimited[:] = 1
        if spec["roller"]:
            for j in range(model.njnt):
                jn = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, j)
                if jn and re.match(r"^passive_", jn):
                    model.dof_frictionloss[int(model.jnt_dofadr[j])] = 0.003
        data = mujoco.MjData(model)
        free_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "trunk_base_freejoint")
        qa = int(model.jnt_qposadr[free_id])
        names = tuple(mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, int(j))
                      for j in model.actuator_trnid[:, 0])
        policy = make_policy(ip, model, data, spec, pname)

        pooled = {"all": {jn: {"actual": [], "target": [], "vel": [], "rate": []} for jn in names},
                  "prefall": {jn: {"actual": [], "target": [], "vel": [], "rate": []} for jn in names}}
        for case in spec["cases"]:
            t0 = time.monotonic()
            log = run_case(mujoco, model, data, policy, spec, case,
                           args.seconds, args.settle, qa)
            n_raw = log["qpos"].shape[0]
            qpos = np.degrees(log["qpos"][keep_from:])
            ctrl = np.degrees(log["ctrl"][keep_from:])
            qvel = np.degrees(log["qvel"][keep_from:])
            # 目标角速率：相邻控制步 ctrl 差分，取落在保留窗口内的那些转移
            rate = np.diff(np.degrees(log["ctrl"]), axis=0)[max(keep_from - 1, 0):] / CONTROL_DT
            act = np.asarray(log["active"][keep_from:])
            tz = log["trunk_z"][keep_from:]
            below = tz < FALL_TRUNK_Z_M
            first_below = int(np.argmax(below)) if below.any() else None
            valid = log["diverged"] is None and qpos.shape[0] > 0
            # prefall 窗口：截到躯干首次低于 60 mm 之前。跌倒之后策略处在分布外，
            # 挥舞出来的角度不代表"策略需要的工作空间"。（坐/翻滚策略本来就该低，
            # 所以两套包络都给，判定时按策略性质选。）
            pf = first_below if first_below is not None else qpos.shape[0]
            if dump is not None and valid:
                dump[f"{pname}/{case['name']}/qpos_deg"] = qpos.astype(np.float32)
                dump[f"{pname}/{case['name']}/prefall_n"] = np.int64(pf)
                dump["joint_names"] = np.array(names)
            run = {
                "policy": pname, "case": case["name"], "scene": spec["scene"],
                "slot": spec["slot"], "companion": spec.get("companion"),
                "twist_cmd": list(case["twist"]) if case.get("twist") is not None else None,
                "head_cmd_rad": list(case["head"]) if case.get("head") is not None else None,
                "body_cmd": list(case["body"]) if case.get("body") is not None else None,
                "events": case.get("events", []),
                "planned_control_steps": log["steps_planned"],
                "simulated_control_steps": n_raw,
                "kept_samples": int(qpos.shape[0]),
                "kept_seconds": round(float(qpos.shape[0] * CONTROL_DT), 3),
                "diverged": log["diverged"],
                "valid_for_envelope": valid,
                "min_trunk_z_mm": round(float(np.min(tz) * 1000.0), 2) if tz.size else None,
                "final_trunk_z_mm": round(float(tz[-1] * 1000.0), 2) if tz.size else None,
                "trunk_below_60mm_fraction": round(float(np.mean(below)), 4) if tz.size else None,
                "first_below_60mm_s": (round(args.settle + first_below * CONTROL_DT, 3)
                                       if first_below is not None else None),
                "prefall_samples": int(pf),
                "active_policy_fractions": {a: round(float(np.mean(act == a)), 4)
                                            for a in sorted(set(act.tolist()))},
                "mujoco_warning": log["warning"],
                "wall_s": round(time.monotonic() - t0, 2),
                "joints": {},
            }
            for i, jn in enumerate(names):
                zone = safe_zone.get(jn)
                entry = {"actual_deg": stats_block(qpos[:, i]),
                         "target_deg": stats_block(ctrl[:, i]),
                         "actual_vel_deg_s": stats_block(qvel[:, i]),
                         "target_rate_deg_s": stats_block(rate[:, i]),
                         "actual_deg_prefall": stats_block(qpos[:pf, i]),
                         "target_deg_prefall": stats_block(ctrl[:pf, i])}
                if zone:
                    lo, hi = zone["first_contact"]
                    entry["time_fraction_outside_first_contact"] = {
                        "actual": outside_fraction(qpos[:, i], lo, hi),
                        "target": outside_fraction(ctrl[:, i], lo, hi),
                        "actual_prefall": outside_fraction(qpos[:pf, i], lo, hi),
                        "target_prefall": outside_fraction(ctrl[:pf, i], lo, hi)}
                run["joints"][jn] = entry
                if not valid:
                    continue
                pooled["all"][jn]["actual"].append(qpos[:, i])
                pooled["all"][jn]["target"].append(ctrl[:, i])
                pooled["all"][jn]["vel"].append(qvel[:, i])
                pooled["all"][jn]["rate"].append(rate[:, i])
                if pf > 0:
                    pooled["prefall"][jn]["actual"].append(qpos[:pf, i])
                    pooled["prefall"][jn]["target"].append(ctrl[:pf, i])
                    pooled["prefall"][jn]["vel"].append(qvel[:pf, i])
                    pooled["prefall"][jn]["rate"].append(rate[:pf, i])
            report["runs"].append(run)
            print(f"[{pname}] {case['name']:26s} kept={run['kept_samples']:4d} "
                  f"minz={run['min_trunk_z_mm']:6.1f}mm "
                  f"below60={run['trunk_below_60mm_fraction']:.2f} "
                  f"diverged={'Y' if log['diverged'] else 'n'} {run['wall_s']:5.2f}s", flush=True)

        runs_ok = [r for r in report["runs"] if r["policy"] == pname and r["valid_for_envelope"]]
        runs_pf = [r for r in runs_ok if r["prefall_samples"] > 0]
        env = {}
        for jn in names:
            block = {}
            for tag, rs, keysuffix in (("all", runs_ok, ""), ("prefall", runs_pf, "_prefall")):
                d = pooled[tag][jn]
                if not d["actual"]:
                    block[tag] = None
                    continue
                block[tag] = {
                    "actual_deg": stats_block(np.concatenate(d["actual"])),
                    "target_deg": stats_block(np.concatenate(d["target"])),
                    "actual_vel_deg_s": stats_block(np.concatenate(d["vel"])),
                    "target_rate_deg_s": stats_block(np.concatenate(d["rate"])),
                    "actual_min_case": min(rs, key=lambda r: r["joints"][jn]["actual_deg" + keysuffix]["min"])["case"],
                    "actual_max_case": max(rs, key=lambda r: r["joints"][jn]["actual_deg" + keysuffix]["max"])["case"],
                    "target_min_case": min(rs, key=lambda r: r["joints"][jn]["target_deg" + keysuffix]["min"])["case"],
                    "target_max_case": max(rs, key=lambda r: r["joints"][jn]["target_deg" + keysuffix]["max"])["case"],
                    "runs_pooled": len(rs),
                }
            env[jn] = block
        report["policy_envelope"][pname] = env

    # ---------------- 裕度与越界 ---------------- #
    for pname, env in report["policy_envelope"].items():
        runs_ok = [r for r in report["runs"] if r["policy"] == pname and r["valid_for_envelope"]]
        margins = {}
        for jn, e in env.items():
            zone = safe_zone.get(jn)
            if zone is None:
                continue
            fc_lo, fc_hi = zone["first_contact"]
            lc_lo, lc_hi = zone["last_clear"]
            m = {"safe_first_contact": [fc_lo, fc_hi], "safe_last_clear": [lc_lo, lc_hi],
                 "mjcf_range": zone.get("mjcf_range")}
            for window in ("all", "prefall"):
                if e.get(window) is None:
                    continue
                suffix = "" if window == "all" else "_prefall"
                m[window] = {}
                for kind, key in (("actual", "actual_deg"), ("target", "target_deg")):
                    lo_v, hi_v = e[window][key]["min"], e[window][key]["max"]
                    m[window][kind] = {
                        "min_deg": lo_v, "max_deg": hi_v,
                        "margin_lo_vs_first_contact": round(lo_v - fc_lo, 4),
                        "margin_hi_vs_first_contact": round(fc_hi - hi_v, 4),
                        "margin_lo_vs_last_clear": round(lo_v - lc_lo, 4),
                        "margin_hi_vs_last_clear": round(lc_hi - hi_v, 4),
                    }
                    for side, bound, sign in (("lo", fc_lo, -1), ("hi", fc_hi, 1)):
                        offenders = []
                        for r in runs_ok:
                            st = r["joints"][jn][key + suffix]
                            if st is None:
                                continue
                            frac = r["joints"][jn]["time_fraction_outside_first_contact"][
                                kind + suffix]
                            excess = (bound - st["min"]) if sign < 0 else (st["max"] - bound)
                            if excess > 1e-9:
                                offenders.append({
                                    "case": r["case"], "exceed_deg": round(excess, 3),
                                    "time_fraction": frac["below" if sign < 0 else "above"]})
                        if not offenders:
                            continue
                        offenders.sort(key=lambda o: -o["exceed_deg"])
                        report["violations"].append({
                            "policy": pname, "joint": jn, "quantity": kind,
                            "window": window, "side": side, "safe_bound_deg": bound,
                            "worst_case": offenders[0]["case"],
                            "worst_exceed_deg": offenders[0]["exceed_deg"],
                            "worst_time_fraction": offenders[0]["time_fraction"],
                            "n_cases_exceeding": len(offenders),
                            "n_cases_total": len(runs_ok),
                            "offenders": offenders})
            margins[jn] = m
        report["margins"][pname] = margins

    report["violations"].sort(key=lambda v: -v["worst_exceed_deg"])
    report["not_measured"] = [
        {"item": "upstream/Open_Duck_Mini/BEST_WALK_ONNX.onnx, BEST_WALK_ONNX_2.onnx",
         "reason": ("Measured input width is [1,101], not [1,61]. These are Open Duck Mini v2 "
                    "AWD/imitation policies for the open_duck_mini_v2 MJCF (a different robot: "
                    "experiments/v2/onnx_AWD_mujoco.py loads mujoco_menagerie/open_duck_mini_v2/"
                    "scene.xml and a 16-value init_pos). The repo ships neither mini_bdx_runtime "
                    "nor that MJCF, and infer_policy.py cannot build a 101-D observation. "
                    "Not run; no numbers estimated.")},
        {"item": "sim/runs/2026-09-06_baseline_xl330/model_3999.pt",
         "reason": "torch is not installed in ./.venv and installing it is out of scope. Not run."},
    ]
    report["elapsed_wall_s"] = round(time.monotonic() - started, 1)

    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(report, ensure_ascii=False, indent=2, default=str) + "\n")
    print(f"\nwrote {args.out}")
    if dump is not None:
        args.dump_steps.parent.mkdir(parents=True, exist_ok=True)
        np.savez_compressed(args.dump_steps, **dump)
        print(f"wrote {args.dump_steps}（{sum(1 for k in dump if k.endswith('/qpos_deg'))} 个工况的逐步关节角）")
    print(f"runs={len(report['runs'])}  violations={len(report['violations'])}  "
          f"wall={report['elapsed_wall_s']}s")
    for v in report["violations"]:
        if v["window"] != "prefall":
            continue
        print(f"  VIOLATION[prefall] {v['policy']}/{v['joint']} {v['quantity']} {v['side']} "
              f"bound={v['safe_bound_deg']}  exceed={v['worst_exceed_deg']}deg  "
              f"case={v['worst_case']}  t_frac={v['worst_time_fraction']}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
