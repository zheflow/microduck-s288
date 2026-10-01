#!/usr/bin/env python3
"""从导出的 placed/ 实体上量每颗螺丝的**真坐面**，生成 fasteners.yaml:tool_access 片段。

为什么要有这个脚本（2026-09-12 Gate 清红第二批）：
  · L4 起子判据（tools/gate/tool_access.py）要求每组螺丝显式声明"头侧孔身份 + 真坐面点 + 头侧方向 +
    拧紧工位"。第一批只写了 F01/F24 两组，其余 27 组 unknown。
  · 坐面不能从 feature_hole_map 的孔坐标（刀具中点）猜，必须在网格上量：沿孔轴、在螺丝头足印环带
    （孔半径..头半径）上找"向内全是料、向外全是空"的那个面。本脚本就做这一件事，量法与
    tool_access.seat_samples 完全一致（同一套 _ring/_ring_radii），所以量出来的点一定过它的探针。
  · 输出只是 YAML 片段 + 逐孔证据；写进 fasteners.yaml 之前人要看一眼候选面列表
    （一个孔轴上可能有多个"料→空"面，脚本按规则选，但规则不是判据）。

选面规则（不是判据，判据在 tool_access.py）：
  1. 沿声明的头侧方向 outward 采样 t ∈ [−T, +T]，找所有 料→空 转换（MV）。
  2. 候选必须：向内 ≥ 0.4 mm 连续有料（头要有东西坐）；向外 ≥ 头高 1.6 mm 连续为空（头要放得下）。
  3. 多个候选取**离孔坐标最近**的一个；其余全部打印出来供人复核。

用法：
    ./.venv/bin/python tools/gate/probe_seats.py [--groups F02,F03] [--out <yaml>]
"""
from __future__ import annotations
import argparse, sys, math
from pathlib import Path
import numpy as np
import yaml

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent.parent
sys.path.insert(0, str(HERE)); sys.path.insert(0, str(HERE / "layers")); sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "tools" / "cad"))

from core import load_data, num                                   # noqa: E402
from gate import Ctx                                              # noqa: E402
from layers.l4_assembly import _Geo, _Names, _world_of_hole       # noqa: E402
from layers.l5_screwhead import _ring, _ring_radii, _declared_axis, _head_geom   # noqa: E402
from tool_access import seat_samples                              # noqa: E402

# ── 每组：头侧孔映射索引 + 头侧方向（世界系，左件/唯一件）+ 拧紧工位 ──────────────
# outward = 起子从哪边来 = 螺丝头露出的那一侧的外法线。右件（mirror_y）由脚本自动取 y 反号。
# state 里 "L"/"R" 分别是左右实例的工位名（assembly.yaml:tool_states 或 motion:<step>:<id>）。
SPEC = {
    "F02_hipyaw_flange":        dict(map=[0], out=(0, 0, -1), state={"L": "tool:hipyaw_flange_L", "R": "tool:hipyaw_flange_R"}),
    "F03_hiproll_back":         dict(map=[0], out=(-1, 0, 0), state={"L": "tool:hiproll_back_L", "R": "tool:hiproll_back_R"}),
    "F04_hiproll_flange":       dict(map=[0], out=(1, 0, 0),  state={"L": "tool:hiproll_flange_L", "R": "tool:hiproll_flange_R"}),
    "F05_hippitch_back":        dict(map=[0], out=(0, 1, 0),  state={"L": "tool:upper_leg_bench_L", "R": "tool:upper_leg_bench_R"}),
    "F06_hippitch_flange":      dict(map=[0], out=(0, -1, 0), state={"L": "tool:hippitch_flange_L", "R": "tool:hippitch_flange_R"}),
    "F07_knee_back":            dict(map=[0], out=(0, 1, 0),  state={"L": "tool:upper_leg_bench_L", "R": "tool:upper_leg_bench_R"}),
    "F08_knee_flange":          dict(map=[0], out=(0, -1, 0), state={"L": "tool:knee_flange_L", "R": "tool:knee_flange_R"}),
    "F09_ankle_back":           dict(map=[0, 1], out=(0, -1, 0), state={"L": "motion:2:left_foot_and_servo", "R": "motion:2:right_foot_and_servo"}),
    "F10_ankle_flange":         dict(map=[0], out=(0, 1, 0),  state={"L": "motion:1:left_foot_to_servo", "R": "motion:1:right_foot_to_servo"}),
    "F11_neckpitch_back_upper": dict(map=[0], out=(0, -1, 0), state={"L": "tool:neck_servo_in_N01"}),
    "F11b_neckpitch_back_lower": dict(map=[0], out=(0, -1, 0), state={"L": "tool:neck_servo_in_N01"}),
    "F12_neckpitch_flange":     dict(map=[0], out=(0, 1, 0),  state={"L": "tool:neck_on_trunk"}),
    "F12b_neckpitch_idler":     dict(map=[0], out=(0, -1, 0), state={"L": "tool:shells_on"}),
    "F13_headpitch_back":       dict(map=[0], out=(0, -1, 0), state={"L": "tool:headpitch_servo_in_N01"}),
    "F14_headpitch_flange":     dict(map=[0], out=(0, 1, 0),  state={"L": "tool:head_on_headpitch"}),
    "F14b_headpitch_idler":     dict(map=[0], out=(0, -1, 0), state={"L": "tool:head_on_headpitch"}),
    "F15_headyaw_back":         dict(map=[0], out=(0, 0, 1),  state={"L": "tool:headyaw_servo_in_N03"}),
    "F16_headyaw_flange":       dict(map=[0], out=(0, 0, -1), state={"L": "tool:N02_on_headyaw"}),
    "F17_headroll_flange":      dict(map=[0], out=(1, 0, 0),  state={"L": "tool:headroll_flange"}),
    "F18_headroll_back_H02":    dict(map=[0], out=(-1, 0, 0), state={"L": "tool:H02_on"}),
    "F19_H02_to_H01":           dict(map=[0], out=(-1, 0, 0), state={"L": "tool:H02_on"}),
    "F20_shell_to_T01":         dict(map=[0, 1], out=(0, 0, 1), state={"L": "tool:shells_on"}),
    "F21_B01_to_T01_front":     dict(map=[0], out=(0, 0, -1), state={"L": "tool:battery_door_on"}),
    "F22_H01_to_H03":           dict(map=[0], out=(0, 0, 1),  state={"L": "tool:head_shells_on"}),
}

T_RANGE, T_STEP = 45.0, 0.1             # 09-13：15 → 45。features 里 horn_hole 的 hole_positions 是**刀心**不是坐面（N02-F07 z 17.35 而脸颊在 z −1..2.4），
CHUNK = 200                             # contains 分块大小（点）
COARSE_STEP, FINE_HALF = 1.0, 2.0       # 坐面可能离锚点 15~20 mm；先粗扫 1.0 步找料段，再在每个 料→空 边界 ±2 细扫 0.1 步（T01 这种大网格才跑得动）
INWARD_MIN, OUTWARD_MIN = 0.4, 1.6      # 头下要有料、头上要有空（M2 头高 1.6，frozen.yaml:P_dict.m2_head.h）


def transitions(mesh, anchor, axis, radii):
    """沿 axis 找 料→空 面。返回 [(t, inward_len, outward_len)]。粗扫定位、细扫定边、二分定面。"""
    pts0 = np.vstack([_ring(anchor, axis, r) for r in radii])
    def _scan(ts):
        allpts = (pts0[None, :, :] + (axis[None, :] * ts[:, None])[:, None, :]).reshape(-1, 3)
        # 分块 contains：trimesh 无 embree 时 ray_triangle 的候选矩阵 ∝ 点数×三角形数，T03/L04 这种网格一次喂 2 万点会吃到 2.4 GB 被系统杀（09-13 两次 exit 137）
        c = np.concatenate([mesh.contains(allpts[i:i + CHUNK]) for i in range(0, len(allpts), CHUNK)]).reshape(len(ts), len(pts0))
        return c.all(axis=1), ~c.any(axis=1)
    tc = np.arange(-T_RANGE, T_RANGE + 1e-9, COARSE_STEP)
    ins_c, _ = _scan(tc)
    # 粗扫里"料段"的两端 ±FINE_HALF 合并成细扫窗口
    win = np.zeros(len(tc), bool)
    for i in range(len(tc)):
        if ins_c[i] and (i + 1 >= len(tc) or not ins_c[i + 1]):
            win[max(0, i - int(FINE_HALF / COARSE_STEP)):min(len(tc), i + int(FINE_HALF / COARSE_STEP) + 2)] = True
    if not win.any():
        return []
    ts = np.unique(np.round(np.concatenate([np.arange(tc[i] - FINE_HALF, tc[i] + FINE_HALF + 1e-9, T_STEP) for i in range(len(tc)) if win[i]]), 3))   # 先 round 再 unique：相邻窗口的同一 t 因浮点误差成两点，会把 料→空 边界判丢（09-13 F12 就是这么漏的）
    inside, void = _scan(ts)
    out = []
    for i in range(len(ts) - 1):
        if inside[i] and void[i + 1] and abs(ts[i + 1] - ts[i] - T_STEP) < 1e-6:
            lo, hi = ts[i], ts[i + 1]
            for _ in range(10):                                  # 二分到 ~5e-5 mm（一次 contains 一轮）
                mid = 0.5 * (lo + hi)
                if mesh.contains(pts0 + axis * mid).all():
                    lo = mid
                else:
                    hi = mid
            t = 0.5 * (lo + hi)
            j = i
            while j >= 0 and inside[j]:
                j -= 1
            k = i + 1
            while k < len(ts) and void[k]:
                k += 1
            out.append((float(t), float(ts[i] - ts[j + 1] + T_STEP), float(ts[k - 1] - ts[i + 1] + T_STEP)))
    return out


def probe_group(gid, fa, spec, ctx, geo, names, features, frames, head_d):
    ops = {}
    evidence = []
    for mi in spec["map"]:
        entry = fa["feature_hole_map"][mi]
        feat = features[entry["feature_id"]]
        part = feat["part"]
        fg = feat.get("geom") or {}
        if fg.get("instances"):
            fg = {**fg, **fg["instances"][entry.get("instance", 0)]}   # 09-13：F20 的 map 没写 instance → 默认 0
        axis_l = _declared_axis(fg.get("axis"))
        hole_d, _ = num(fg.get("nominal_d_mm"))
        stems = names.part_stems(part)
        if not stems:
            raise RuntimeError(f"{gid}: {part} 无 placed 实例")
        R = np.asarray(frames[part]["world_to_export_local_R"], float)
        out_w = np.asarray(spec["out"], float); out_w /= np.linalg.norm(out_w)
        out_l = R @ out_w                                          # 世界 → export_local
        if axis_l is None or abs(float(axis_l @ out_l)) < 1 - 1e-6:
            raise RuntimeError(f"{gid}: {entry['feature_id']} 声明轴 {fg.get('axis')} 与头侧方向 {spec['out']} 不平行（局部 {np.round(out_l,3)}）")
        radii = _ring_radii(hole_d / 2, head_d / 2)
        for hi, anchor_l in enumerate(entry["holes"]):
            anchor_l = np.asarray(anchor_l, float)
            for si, host in enumerate(stems):
                side = "L" if si == 0 else "R"
                state = spec["state"].get(side)
                if state is None:
                    raise RuntimeError(f"{gid}: 缺 {side} 侧工位")
                mesh = geo.mesh(host)
                world = _world_of_hole(frames, part, anchor_l)
                ow = out_w.copy()
                if si > 0:                                         # build.py mirror_y
                    world = world.copy(); world[1] *= -1; ow[1] *= -1
                cands = [c for c in transitions(mesh, world, ow, radii) if c[1] >= INWARD_MIN and c[2] >= OUTWARD_MIN]
                if not cands:
                    evidence.append(dict(gid=gid, host=host, mi=mi, hi=hi, ok=False, why="无满足 内≥0.4/外≥1.6 的料→空面"))
                    continue
                t, inw, outw = min(cands, key=lambda c: abs(c[0]))
                seat_w = world + ow * t
                chk = seat_samples(mesh, seat_w, ow, hole_d, head_d)
                seat_l = anchor_l + out_l * t if si == 0 else anchor_l + out_l * t   # 右件局部坐标与左件相同
                evidence.append(dict(gid=gid, host=host, mi=mi, hi=hi, ok=chk["ok"], t=round(t, 4), inward=inw, outward=outw,
                                     n_cands=len(cands), cands=[round(c[0], 3) for c in cands], seat_world=np.round(seat_w, 4).tolist()))
                if si == 0:
                    ops.setdefault(state, dict(state_ref=state, seats=[], hosts=set()))
                    ops[state]["seats"].append(dict(feature_id=entry["feature_id"], instance=host,
                                                    point_export_local=[float(round(x, 4)) for x in seat_l],
                                                    outward_export_local=[int(round(x)) for x in out_l],
                                                    map_index=mi, hole_index=hi))
                else:
                    ops.setdefault(state, dict(state_ref=state, seats=[], hosts=set()))
                    ops[state]["seats"].append(dict(feature_id=entry["feature_id"], instance=host,
                                                    point_export_local=[float(round(x, 4)) for x in seat_l],
                                                    outward_export_local=[int(round(x)) for x in out_l],
                                                    map_index=mi, hole_index=hi))
    return list(ops.values()), evidence


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--groups", default="")
    ap.add_argument("--out", default=str(ROOT / "tools/gate/out/tool_access_probe.yaml"))
    a = ap.parse_args(argv)
    data = load_data()
    ctx = Ctx(data)
    geo = _Geo(ctx)
    names = _Names(ctx, geo)
    feats = {f["id"]: f for f in data["features"]["features"]}
    frames = data["features"]["frames"]["per_part"]
    fasteners = {f["id"]: f for f in data["fasteners"]["fasteners"]}
    want = [g for g in a.groups.split(",") if g] or list(SPEC)
    head_d = _head_geom(data.get("frozen"), 2.0)[0]
    result, log = {}, []
    for gid in want:
        try:
            ops, ev = probe_group(gid, fasteners[gid], SPEC[gid], ctx, geo, names, feats, frames, head_d)
        except Exception as e:                                     # noqa: BLE001
            print(f"[{gid}] 失败：{e}")
            continue
        bad = [e for e in ev if not e["ok"]]
        print(f"[{gid}] 坐面 {len(ev) - len(bad)}/{len(ev)} 过探针；候选数 {[e.get('n_cands') for e in ev]}")
        for e in ev:
            flag = "OK " if e["ok"] else "BAD"
            print(f"   {flag} {e['host']:18} map{e['mi']} hole{e['hi']} t={e.get('t')} in={e.get('inward')} out={e.get('outward')} cands={e.get('cands')} {e.get('why','')}")
        for op in ops:
            op.pop("hosts", None)
            op["provenance"] = (f"tools/gate/probe_seats.py 2026-09-12：沿头侧方向在头足印环带（Ø{fasteners[gid]['feature_hole_map'][SPEC[gid]['map'][0]]['feature_id']} 孔径..Ø{head_d} 头径，"
                                f"3 圈×24 向）找 料→空 面，取离孔坐标最近者；逐孔候选面数见 tools/gate/out/tool_access_probe.log")
        result[gid] = dict(head_locator_map_indices=list(SPEC[gid]["map"]), tool_access=ops)
        log.extend(ev)
    Path(a.out).write_text(yaml.safe_dump(result, allow_unicode=True, sort_keys=False, width=160))
    Path(a.out).with_suffix(".log").write_text("\n".join(str(e) for e in log))
    print("wrote", a.out)


if __name__ == "__main__":
    main()
