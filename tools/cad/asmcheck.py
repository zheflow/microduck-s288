#!/usr/bin/env python3
"""装配路径检查：把每颗舵机从 home 位置沿 6 个方向直线平移，看它能不能从自己的载体里抽出来。
六个方向都堵时，只能判定尚未证明装配可达；斜插/旋转装配需要另行验证。H01 检查先拆下 H02。
顺便检查 S288 PH2.0 插座（09-20 背插模型：插头顶/线弯区 + 出线区，lib.conn_zone / CONN_MODE，两侧各报）有没有被挡住。

    python tools/cad/asmcheck.py
"""
import math, numpy as np
import sys, os
HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.normpath(os.path.join(HERE, "..", ".."))
sys.path.insert(0, os.path.join(ROOT, "tools", "cad"))
sys.path.insert(0, ROOT)
os.chdir(ROOT)
import duckstructure as D
from duckstructure import s288
from duckstructure.s288 import S

STEPS = np.arange(0.0, 45.01, 0.5)

# hr42（2026-09-24，B 线）：
#   ① 非闭合体（ankle_foot / ankle_foot_R 的 placed 读回不是 volume）的 contains 采样判定走 tools/cad/ray_backend.contains
#      （Embree 找候选面 + trimesh ray_triangle 原公式逐对重算 → 与 pm.contains 同一奇偶规则、同一默认方向；
#      真网格自测逐点相同，见 hr42_work/profile/B/）。DUCK_RAY_BACKEND=triangle 退回 pm.contains；
#      DUCK_RAY_VERIFY=<json> 两后端同点都算、逐点比对写文件（判定仍用默认后端的结果）。
#      采样点本身是 zone.sample() 的随机点（原样未改）→ 这条判据两次跑本来就不是同一批点，对账看"两后端同点同结果"。
#   ② 舵机直线抽出：某方向第一次出现交集 > 0.05 就停（该方向已判"堵"，它的残余值只在"六向全堵"时才打印）；
#      找到第一个通的方向就不再算后面的方向（后面方向的残余值从不打印）。六向全堵时每个方向**全程重算**再打印 → 输出与改前逐字相同。
import ray_backend as _RB
_RB_USED = []


def _contains(pm, pts, site=""):
    _RB.set_site(site)
    _RB_USED.append(1)
    return np.asarray(_RB.contains(pm, np.asarray(pts, dtype=float)))


def _dir_worst(host, env, dv, stop_over=None):
    """舵机沿 dv 平移全程（STEPS）与载体的最大交集 —— 与改前内层循环逐步同算；stop_over 给了就在最大值第一次超过它时提前返回 (值, False)。"""
    worst = 0.0
    for d in STEPS:
        e = env.copy(); e.apply_translation(np.array(dv, float) * d)
        v, _ = D.vol(host, e); worst = max(worst, v)
        if stop_over is not None and worst > stop_over:
            return worst, False
    return worst, True
DIRS = (("+x", (1, 0, 0)), ("-x", (-1, 0, 0)), ("+y", (0, 1, 0)),
        ("-y", (0, -1, 0)), ("+z", (0, 0, 1)), ("-z", (0, 0, -1)))

# 每颗舵机由哪个件"拿着"（载体侧）。舵机装进的就是这个件。
CARRIER = {("trunk_base", 0): "trunk", ("trunk_base", 1): "trunk",
           ("yaw2roll", 0): "yaw2roll", ("upper_leg_left", 0): "upper_leg", ("upper_leg_left", 1): "upper_leg",
           ("leg", 0): "lower_leg", ("neck", 0): "neck", ("neck", 1): "neck",
           ("yaw_roll_motion", 0): "yrm", ("jaw_soft", 1): "head_bracket",
           ("bearing_roll", 0): "yaw2roll_R", ("upper_leg_right", 0): "upper_leg_R",
           ("upper_leg_right", 1): "upper_leg_R", ("leg_2", 0): "lower_leg_R"}

# The jaw servo uses its own relocated frame, not the upstream sfw frame. Its
# carrier path must be tested as well as the original fourteen servos. Socket
# routing has a separate jaw-specific audit in wiring_head.
INSERTION_CARRIER = {**CARRIER, ("jaw_soft", 0): "head_bracket"}


def insertion_servo_mesh(body, index):
    if (body, index) == ("jaw_soft", 0):
        # hr41 装序 P 第 b 步：嘴舵机法兰上**先拧好 J02 转接盘**，两件一起沿 −y 横插进 H01 托架 → 抽出体 = 舵机 + J02
        from duckstructure.jaw import servo_mesh_world, build_jaw_adapter
        return D.union(servo_mesh_world(), build_jaw_adapter())
    return s288.placed(s288.servo_mesh(), D.sfw(body, index))

def build_from_placed():
    """2026-09-18：从 cad/duck_s288/placed/ 读上一次 build 导出的世界坐标件（keep_main 后的同一几何），省掉 11 分钟重建。
    与 build() 的差别只有 head_bracket 里 H02 的并入方式（这里并入 placed 的 head_clamp）。"""
    import trimesh
    from duckstructure.lib import OUT as _OUT
    pd = os.path.join(_OUT, "placed")
    names = ["trunk", "yaw2roll", "upper_leg", "lower_leg", "neck", "yrm", "head_bracket", "yaw2roll_R", "upper_leg_R", "lower_leg_R"]
    parts = {n: trimesh.load(os.path.join(pd, n + ".stl"), process=True) for n in names}
    parts["_head_clamp"] = trimesh.load(os.path.join(pd, "head_clamp.stl"), process=True)
    return parts


def build():
    parts = {}
    parts["trunk"] = D.build_trunk()
    parts["yaw2roll"] = D.build_yaw2roll()
    parts["upper_leg"] = D.build_upper_leg()
    parts["lower_leg"] = D.build_lower_leg()
    parts["neck"] = D.build_neck()
    parts["yrm"] = D.build_yrm()
    parts["head_bracket"] = D.build_head_bracket()
    for n in ("yaw2roll", "upper_leg", "lower_leg"):
        parts[n + "_R"] = D.mirror_y(parts[n])
    return parts

def socket_motion(pool, step_deg=5.0):
    """09-21 线翘 4 mm 后的运动核（零位之外）：每个 window/free 型插座位的线翘区 A 随载体走，逐个**单关节**（载体自己的关节 + 载体直接子件的
    关节，按 frozen.yaml target_range_deg 每 step_deg 一档）摆到位，与**不在载体 body 上**的每个 placed 件求交（同 body 的件相对不动，零位那段已查）。
    这是 L07 后臂 / N02 脸颊 / T03 壳给线翘区让位（legs/neck/trunk 里的 conn_hump_sweep）的独立复核；组合姿态归 Gate L6 / 真实姿态采样（那两处不含线翘区）。"""
    import json, yaml, trimesh
    from duckstructure.lib import B, ORDER, TW
    from trimesh.transformations import rotation_matrix as rot
    bfp = json.load(open(os.path.join(ROOT, "tools/sim/cad_geometry_manifest.json"), encoding="utf-8"))["body_for_part"]
    fz = yaml.safe_load(open(os.path.join(ROOT, "tools/gate/data/frozen.yaml"), encoding="utf-8"))
    rng = {j["name"]: tuple(j["target_range_deg"]["v"]) for j in fz["joint_axes"] if (j.get("target_range_deg") or {}).get("v")}
    def posed(pose):
        out = {}
        for n in ORDER:
            d = B[n]; T = d["T_parent"] if d["parent"] is None else out[d["parent"]] @ d["T_parent"]
            j = d.get("joint")
            if j and abs(pose.get(j["name"], 0.0)) > 1e-12: T = T @ rot(math.radians(pose[j["name"]]), [0, 0, 1], [0, 0, 0])
            out[n] = T
        return {n: out[n] @ np.linalg.inv(B[n]["T_world"]) for n in ORDER}     # 世界零位 → 该姿态的世界
    print("\n[PH2.0 线翘区 A 的单关节运动核：载体关节 + 子件关节各扫 target_range_deg（5°/档），与不同 body 的件求交]")
    bad = 0
    for (body, i), pname in CARRIER.items():
        R = D.sfw(body, i)
        joints = [B[body]["joint"]["name"]] if B[body].get("joint") else []
        joints += [B[c]["joint"]["name"] for c in ORDER if B[c]["parent"] == body and B[c].get("joint")]
        for sy in (1, -1):
            mode = D.CONN_MODE[(body, i)][sy]
            if mode not in ("window", "free"): continue
            zone0 = D.placed(D.conn_zone(sy, "A"), R)
            for jn in joints:
                lo, hi = rng[jn]; worst = {}
                for a in np.arange(lo, hi + 1e-6, step_deg):
                    M = posed({jn: float(a)}); zrel = {}
                    for pn, pm in pool.items():
                        pb = bfp.get(pn)
                        if pb is None or pb == body: continue
                        # 件不动，把线翘区换到该件 body 的零位系里（相对变换），件网格不必逐档搬
                        if pb not in zrel: zrel[pb] = D.placed(zone0, np.linalg.inv(M[pb]) @ M[body])
                        zone = zrel[pb]
                        if np.any(pm.bounds[1] < zone.bounds[0]) or np.any(zone.bounds[1] < pm.bounds[0]): continue
                        if not pm.is_volume:
                            n_in = int(np.asarray(_contains(pm, zone.sample(3000), f"socket_motion:{pn}")).sum()); v = n_in / 3000 * zone.volume
                        else:
                            v, _ = D.vol(pm, zone)
                        if v > 0.05 and v > worst.get(pn, (0, 0))[0]: worst[pn] = (v, float(a))
                for pn, (v, a) in sorted(worst.items(), key=lambda kv: -kv[1][0]):
                    bad += 1
                    print(f"  ⛔ {body}[{i}] {'+y' if sy > 0 else '-y'} [{mode}] 线翘区 vs {pn}：{jn} 扫 [{lo}, {hi}] 最坏 {v:.1f} mm³ @ {a:.0f}°")
    print(f"  {'全程为空 ✓' if bad == 0 else f'{bad} 对相交'}")


if __name__ == "__main__":
    parts = build_from_placed() if "--from-placed" in sys.argv else build()
    print("[舵机能不能从载体里直线抽出]（残余 = 平移过程中与载体的最大交集）")
    for (body, i), pname in INSERTION_CARRIER.items():
        host = parts[pname]
        # 用**真实舵机外形**而不是 servo_env：包络带 0.3 间隙、而且把副轴让位放大到 Ø16（rear_boss_d+2），
        # 拿它当抽出体会把"其实过得去"误判成堵死
        env = insertion_servo_mesh(body, i)
        best, rows = None, []
        for nm, dv in DIRS:                                           # hr42：见文件头 ②（判定与打印不变）
            worst, _full = _dir_worst(host, env, dv, stop_over=0.05)
            if worst <= 0.05:
                best = nm
                break
        if best is None:
            rows = [(nm, _dir_worst(host, env, dv)[0]) for nm, dv in DIRS]
        tag = f"✓ 可沿 {best} 抽出" if best else "⛔ 六个方向全堵死"
        print(f"  {body}[{i}] → {pname:14s} {tag}")
        if not best:
            print("      " + "  ".join(f"{nm}:{w:.0f}" for nm, w in rows))
    print("\n[PH2.0 插座（2026-09-20 背插模型）：插头顶/线弯区 A + 出线区（B 侧出 / W 背板通窗）有没有被任何件挡住]")
    print("  几何 = s288.S.conn_z/plug_*/wire_out/plug_y_in + lib.conn_zone；出线方式按 lib.CONN_MODE（与 keepouts.yaml:KO01.availability_table 同表）。")
    print("  A 必须全空（插头插到底与 23 面齐平，线直立翘起 4.0+0.4 → 局部 x −17.4，09-21）；window 型另查 W（背板通窗到局部 x −18.5，**只查载体自己**）；pocket 型 09-21 作废；")
    print("  free 型两区都该本来就空；none 型（髋偏航外侧口，壳柱压着）不查、该口不用。--from-placed 时查全部 placed 件（含邻件、壳、轴承），否则只查载体。")
    parts["head_bracket"] = D.union(parts["head_bracket"], parts.pop("_head_clamp", None) or D.build_head_clamp())
    if "--from-placed" in sys.argv:
        import glob, trimesh
        from duckstructure.lib import OUT as _OUT
        pool = {}
        for f in sorted(glob.glob(os.path.join(_OUT, "placed", "*.stl"))):
            nm = os.path.basename(f)[:-4]
            if nm.startswith("servo__") or nm.startswith("zz_"): continue
            mm = trimesh.load(f, process=True)
            if not mm.is_volume:
                mm.fill_holes(); mm.fix_normals()
                if not mm.is_volume: print(f"  （{nm} 不是闭合体：改用让位体内 4000 个采样点做 contains 判定）")
            pool[nm] = mm
    else:
        pool = parts
    bad = 0
    for (body, i), pname in CARRIER.items():
        R = D.sfw(body, i)
        for sy in (1, -1):
            mode = D.CONN_MODE[(body, i)][sy]
            kinds = {"pocket": ("A", "B"), "window": ("A", "W"), "free": ("A", "B"), "none": ()}[mode]
            for k in kinds:
                zone = D.placed(D.conn_zone(sy, k), R)
                for pn, pm in pool.items():
                    # W（背板通窗到局部 x −18.5）只对载体自己查 —— 通窗是载体背板的义务；背板外面的邻件（L07/T03/N02 脸颊）只须让 A（线翘区到 −17.4）
                    if k == "W" and pn != pname: continue
                    if not pm.is_volume:
                        if np.any(pm.bounds[1] < zone.bounds[0]) or np.any(zone.bounds[1] < pm.bounds[0]): continue
                        n_in = int(np.asarray(_contains(pm, zone.sample(4000), f"socket:{pn}")).sum())
                        if n_in:
                            bad += 1
                            print(f"  ⛔ {body}[{i}] {'+y' if sy > 0 else '-y'} [{mode}] 区 {k} 被 {pn} 挡（非闭合体，{n_in}/4000 采样点在料里）")
                        continue
                    v, it = D.vol(pm, zone)
                    if v > 0.05:
                        bad += 1
                        print(f"  ⛔ {body}[{i}] {'+y' if sy > 0 else '-y'} [{mode}] 区 {k} 被 {pn} 挡 {v:.1f} mm³ @ {np.round(it.bounds, 1).tolist()}")
    print(f"  {'全部为空 ✓' if bad == 0 else f'{bad} 处被挡'}（{len(pool)} 件参检）")
    if "--from-placed" in sys.argv:
        socket_motion(pool)
    if _RB_USED:                                                  # hr42：非闭合体 contains 用了哪个射线后端（日志里可见；对账白名单这一行）
        print(f"\n[射线后端] 非闭合体 contains {len(_RB_USED)} 次：{_RB.name()}")
