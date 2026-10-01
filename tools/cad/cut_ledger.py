#!/usr/bin/env python
"""削料账本（cut ledger，2026-09-28）：回答「这个件上的每个缺口是谁削的、为什么削」。

做法：用 build 同一套生成函数逐件重建（每件一个新进程，@memo 不串味），运行时给 s288.diff / s288.inter 挂钩：
  · 每一刀（diff 的每个刀体、inter 的包络）在当时的件上削掉的料 x = 件 ∩ 刀；
  · 刀是在哪一行代码生成的（sys.setprofile 在 duckstructure 函数返回网格时打上调用栈）→ 生成函数里那一行 + 上面的注释 = 为什么削；
  · 成品里还缺的那部分 = x − 成品（后面 union 回填的不算）→ 缺口块（体积、世界系包围盒）；
  · 类别（运动扫掠让位 / 舵机本体让位 / 插头线让位 / 真实姿态区域刀 / 他件或壳让位 / 螺丝孔舵盘孔 / 轴承座 / 刻字 / 扎带位 / 其他）；
  · 缺口离本件登记的螺丝孔 / 底孔 / 沉孔（features.yaml）多远（孔壁剩多少）。
只读：不写 STL / yaml，不改源码（__code__ 换钩只在子进程里）。成品体积与 placed/ 对账，对不上会在报告里标出。

用法：
  ./.venv/bin/python -B tools/cad/cut_ledger.py L04 H01 ...        # 逐件子进程，结果写 --out/<件号>.json + <件号>_notches.npz
  ./.venv/bin/python -B tools/cad/cut_ledger.py --all
"""
import os, sys, json, time, types, argparse, subprocess, re, traceback
import numpy as np

ROOT = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", ".."))
DS = os.path.join(ROOT, "duckstructure") + os.sep
DEFAULT_OUT = os.path.join(ROOT, "docs", "design_2026-09-17_bearing_rebuild", "hr49_work", "ledger")
ALL = ["L01", "L02", "L03", "L04", "L05", "L06", "L07", "L10", "L13", "T01", "B01", "T02", "T03", "B03",
       "N01", "N02", "N03", "N04", "N05", "N06", "N07", "N08", "H01", "H02", "H03", "H04", "H05",
       "J01", "J02", "J03", "H06", "H08", "H09", "H10", "E01", "E02", "E03"]
MIN_MM3 = 0.3            # 小于这个体积的缺口块不记（布尔碎渣）
ANCESTOR_FRAC = 0.30     # 被削的网格表面有 ≥30% 落在成品上/里 → 算「这个件的前身」（排除嵌套生成的别的件）

CATS = [  # (类别, 调用栈里出现的函数名关键字)
    ("真实姿态区域刀", ("region_cut",)),
    ("运动扫掠让位", ("sweep_of", "relief_sweep", "n02_swing_cut", "conn_hump_sweep", "_swing", "sweep")),
    ("插头/线让位", ("conn_cut", "conn_zone", "harness", "wire_path", "roll_conn_relief")),
    ("舵机本体让位", ("servo_env", "body_prism", "servo_mesh", "flange_slot")),
    ("轴承座/轴承让位", ("seat", "bearing", "brg", "journal", "idler")),
    ("螺丝孔/舵盘孔/沉孔", ("mount_cut", "mnt_cut", "horn_cut", "horn_holes", "screw", "cbore", "pilot", "hub_screw", "_cs_cuts")),
    ("刻字", ("stamp", "cutter", "text_polys")),
    ("扎带位/线束固定", ("wire_fixings", "tie")),
    ("壳/他件让位", ("minkowski_box", "DILATE6", "shell", "scaled_orig", "_clear_cutter", "clearance")),
]


def _builders():
    from duckstructure.legs import build_yaw2roll, build_hip, build_upper_leg, build_lower_leg, build_ankle_foot, build_ankle_rear_arm
    from duckstructure.trunk import build_trunk, build_battery_door, build_trunk_shell
    from duckstructure.tail import build_battery_lid
    from duckstructure.neck import build_neck, build_neck_pitch, build_yrm, build_head_journal
    from duckstructure.head import (build_head_bracket, build_head_clamp, build_head_bottom_shell, build_face_plate,
                                    build_camera_clamp, build_adapter_tray, build_amp_bracket, build_tof_clamp)
    from duckstructure.head_top import build_head_top_shell
    from duckstructure import jaw as JAW, eye as EYE
    from duckstructure.hip_pitch_bearing_rebuild import build_hip_pitch_sleeve
    from duckstructure.head_bearing_rebuild import build_head_roll_adapter, build_head_yaw_adapter, build_head_bearing_cap, build_head_yaw_cap
    from duckstructure.bearing_rebuild import build_hip_roll_sleeve
    b = {"L01": ("yaw2roll", "yaw2roll", build_yaw2roll), "L02": ("hip_l", "hip", build_hip),
         "L03": ("upper_leg_left", "upper_leg", build_upper_leg), "L04": ("leg", "lower_leg", build_lower_leg),
         "L05": ("ankle_left", "ankle_foot", lambda: build_ankle_foot()[0]), "L06": ("ankle_left", "sole", lambda: build_ankle_foot()[1]),
         "L07": ("ankle_left", "ankle_rear_arm", build_ankle_rear_arm),
         "L10": ("hip_l", "hip_roll_sleeve", build_hip_roll_sleeve), "L13": ("hip_l", "hip_pitch_sleeve", build_hip_pitch_sleeve),
         "T01": ("trunk_base", "trunk", build_trunk), "B01": ("trunk_base", "battery_door", build_battery_door),
         "T02": ("trunk_base", "shell_L", lambda: build_trunk_shell(1, build_trunk())),
         "T03": ("trunk_base", "shell_R", lambda: build_trunk_shell(-1, build_trunk())),
         "B03": ("trunk_base", "battery_lid", build_battery_lid),
         "N01": ("neck", "neck", build_neck), "N02": ("neck_pitch", "neck_pitch", build_neck_pitch), "N03": ("yaw_roll_motion", "yrm", build_yrm),
         "N04": ("yaw_roll_motion", "head_journal", build_head_journal), "N05": ("yaw_roll_motion", "head_roll_adapter", build_head_roll_adapter),
         "N06": ("neck_pitch", "head_yaw_adapter", build_head_yaw_adapter), "N07": ("yaw_roll_motion", "head_bearing_cap", build_head_bearing_cap),
         "N08": ("yaw_roll_motion", "head_yaw_cap", build_head_yaw_cap),
         "H01": ("jaw_soft", "head_bracket", build_head_bracket), "H02": ("jaw_soft", "head_clamp", build_head_clamp),
         "H03": ("jaw_soft", "head_bottom_shell", build_head_bottom_shell), "H04": ("jaw_soft", "face_plate", build_face_plate),
         "H05": ("jaw_soft", "head_top_shell", build_head_top_shell), "J01": ("jaw_soft", "jaw", JAW.build_jaw),
         "J02": ("jaw_soft", "jaw_adapter", JAW.build_jaw_adapter), "J03": ("jaw_soft", "jaw_journal", JAW.build_jaw_journal),
         "H06": ("jaw_soft", "camera_clamp", build_camera_clamp), "H08": ("jaw_soft", "adapter_tray", build_adapter_tray),
         "H09": ("jaw_soft", "amp_bracket", build_amp_bracket), "H10": ("jaw_soft", "tof_clamp", build_tof_clamp)}
    for pid, (fname, nm, fn) in EYE.PARTS.items():
        b[pid] = ("jaw_soft", nm, fn)
    return b


def _src(file, line):
    """返回 (这一行代码, 行尾注释, 语句上方的注释块)。多行语句先找语句起点（上一行以 , ( [ \\ + 结尾就算续行），再取起点上方连续注释（≤12 行）。"""
    try:
        L = open(file, encoding="utf-8").read().split("\n")
    except Exception:
        return "", "", ""
    if not (0 < line <= len(L)):
        return "", "", ""
    def split_c(t):
        i = t.find("#")
        return (t[:i].rstrip(), t[i + 1:].strip()) if i >= 0 else (t.rstrip(), "")
    code, inl = split_c(L[line - 1])
    k = line - 1
    while k > 0:
        prev, _ = split_c(L[k - 1])
        ps = prev.strip()
        if ps and not ps.startswith(("def ", "class ", "@")) and ps.endswith((",", "(", "[", "\\", "+", "*")):
            k -= 1
        else:
            break
    inls = [split_c(L[j])[1] for j in range(k, line) if split_c(L[j])[1]]
    above = []
    i = k - 1
    while i >= 0 and len(above) < 12:
        t = L[i].strip()
        if t.startswith("#"):
            above.append(t.lstrip("#").strip()); i -= 1
        else:
            break
    return code.strip(), " / ".join([x for x in inls + [inl] if x]), " / ".join(reversed(above))


def child(pid, out):
    import trimesh
    t0 = time.time()
    import duckstructure as D
    import duckstructure.s288 as S
    from duckstructure.lib import TW, clean_print_topology
    from duckstructure import wire_fixings as WF
    from duckstructure.stamps import stamp
    from duckstructure.build import CLEAN_AFTER_STAMP
    B = _builders()
    body, pname, fn = B[pid]
    ref = trimesh.load(os.path.join(ROOT, "cad", "duck_s288", "placed", pname + ".stl"), force="mesh")
    ref_bb = ref.bounds

    # ── 调用栈打标（duckstructure 里的函数返回网格 / 网格列表时，记下调用栈）
    def stack_of(frame):
        st = []
        f = frame
        while f is not None and len(st) < 14:
            fn_ = f.f_code.co_filename
            if fn_.startswith(DS):
                st.append((os.path.relpath(fn_, ROOT), f.f_lineno, f.f_code.co_name))
            f = f.f_back
        return st

    KEEP_ARGS = {"region_cut": ("fname", "body", "mirror"), "sweep_of": ("body", "lo", "hi", "n", "grow"), "conn_cut": ("sides", "mode"),
                 "servo_env": ("extra",), "relief_sweep": ("grow", "step")}

    def _info(frame):
        co = frame.f_code; info = dict(fn=co.co_name, file=os.path.relpath(co.co_filename, ROOT))
        keys = KEEP_ARGS.get(co.co_name)
        if keys:
            for k in keys:
                v = frame.f_locals.get(k)
                if isinstance(v, (str, int, float, bool, tuple)): info[k] = v if not isinstance(v, tuple) else list(v)
            if co.co_name == "sweep_of":
                mm = frame.f_locals.get("m")
                t = getattr(mm, "metadata", {}).get("_ledger") if mm is not None else None
                if t: info["swept"] = t[0][2] if t and isinstance(t[0], (list, tuple)) and len(t[0]) > 2 else str(t[:1])
        return info

    def tag(obj, frame):
        objs = [obj] if isinstance(obj, trimesh.Trimesh) else (list(obj) if isinstance(obj, (list, tuple)) and len(obj) < 400 else [])
        outer = frame.f_code.co_name in KEEP_ARGS          # region_cut / sweep_of 等外层函数返回同一个网格时，用外层的名字和参数覆盖
        for o in objs:
            if isinstance(o, trimesh.Trimesh):
                if "_ledger" not in o.metadata:
                    o.metadata["_ledger"] = stack_of(frame.f_back)
                    o.metadata["_ledger_fn"] = _info(frame)
                elif outer:
                    o.metadata["_ledger_fn"] = _info(frame)

    def prof(frame, event, arg):
        if event == "return" and arg is not None and frame.f_code.co_filename.startswith(DS):
            if frame.f_code.co_name in ("diff", "inter") and frame.f_code.co_filename.endswith("s288.py"):
                return
            tag(arg, frame)

    # ── diff / inter 换钩
    ODIFF = types.FunctionType(S.diff.__code__, S.__dict__, "diff_orig", S.diff.__defaults__, S.diff.__closure__)
    OINTER = types.FunctionType(S.inter.__code__, S.__dict__, "inter_orig", S.inter.__defaults__, S.inter.__closure__)
    RECS = []

    def vol(m):
        try:
            return float(m.volume) if m is not None and len(m.faces) else 0.0
        except Exception:
            return 0.0

    ref_pts = trimesh.sample.sample_surface(ref, 600, seed=5)[0]
    ref_vol = float(ref.volume)

    def is_ancestor(a):
        """a 是不是这个件的前身：(A) a 的表面 ≥30% 落在成品上/里；或 (B) 成品表面 ≥60% 落在 a 里/上、且 a 不超过成品 8 倍大
        （B 管『几个件先合成一块再切开』的情况，如 L05+L07 同一个踝件毛坯）。"""
        try:
            if a is None or len(a.faces) == 0: return False
            lo, hi = a.bounds
            if (hi < ref_bb[0] - 1).any() or (lo > ref_bb[1] + 1).any(): return False
            pts = trimesh.sample.sample_surface(a, 500, seed=1)[0]
            ins = ref.contains(pts)
            d = trimesh.proximity.ProximityQuery(ref).signed_distance(pts[~ins]) if (~ins).any() else np.zeros(0)
            if (ins.sum() + int((np.abs(d) < 0.3).sum())) / len(pts) >= ANCESTOR_FRAC:
                return True
            va = float(a.volume)
            if va > 8 * ref_vol: return False
            ins2 = a.contains(ref_pts)
            d2 = trimesh.proximity.ProximityQuery(a).signed_distance(ref_pts[~ins2]) if (~ins2).any() else np.zeros(0)
            return (ins2.sum() + int((np.abs(d2) < 0.3).sum())) / len(ref_pts) >= 0.6
        except Exception:
            return False

    def site(frame):
        st = stack_of(frame)
        return st

    import gc, resource
    DBG = bool(os.environ.get("LEDGER_DEBUG"))

    def _dbg(op, st, a, ms):
        gc.collect()
        if DBG:
            rss = resource.getrusage(resource.RUSAGE_SELF).ru_maxrss / 1e9
            print(f"  [dbg {time.time()-t0:6.0f}s] {op} @ {st[0][0]}:{st[0][1]} a {len(a.faces)} 面, 刀 {[len(m.faces) for m in ms][:6]}, 记录 {len(RECS)}, 峰值 RSS {rss:.2f} GB", flush=True)

    def _tag_result(r, caller):
        # diff / inter 的结果也打上「调用它的那一行」——刀本身是布尔结果时（如 hump = diff(扫掠, 座环柱)）才找得到出处
        try:
            if isinstance(r, trimesh.Trimesh) and "_ledger" not in r.metadata:
                r.metadata["_ledger"] = stack_of(caller); r.metadata["_ledger_fn"] = {"fn": "boolean"}
        except Exception:
            pass

    def hook_diff(a, ms):
        ms2 = [m for m in ms if m is not None]
        r = ODIFF(a, *ms2)
        _tag_result(r, sys._getframe(2))
        if not ms2: return r
        caller = sys._getframe(2)
        st = site(caller)
        va, vr = vol(a), vol(r)
        if va - vr < MIN_MM3 or not is_ancestor(a):
            return r
        rec = dict(op="diff", site=st, removed=va - vr, cutters=[])
        for b in ms2:
            try:
                # 单刀：削掉的料 = 前 − 后（两个都是件大小），不去和巨大的扫掠刀求交（H03 的 N02 扫掠刀是几百个副本的并集，求交会爆内存）
                x = ODIFF(a, r) if len(ms2) == 1 else OINTER(a, b); vx = vol(x)
            except Exception:
                x, vx = None, 0.0
            if vx >= MIN_MM3:
                rec["cutters"].append(dict(tag=b.metadata.get("_ledger", []), fn=b.metadata.get("_ledger_fn", {}), vol=vx, x=x))
        RECS.append(rec)
        _dbg("diff", st, a, ms2)
        return r

    def covers_part(r):
        """inter 的结果还盖住成品表面 ≥60% 才算『修整这个件』；否则是在造一个小特征（如 foot = inter(毛坯, 小圆柱)），不能把毛坯别处的缺口都记到它头上。"""
        try:
            if r is None or len(r.faces) == 0: return False
            ins = r.contains(ref_pts)
            d = trimesh.proximity.ProximityQuery(r).signed_distance(ref_pts[~ins]) if (~ins).any() else np.zeros(0)
            return (ins.sum() + int((np.abs(d) < 0.3).sum())) / len(ref_pts) >= 0.6
        except Exception:
            return False

    def hook_inter(ms):
        ms = list(ms)
        r = OINTER(*ms)
        _tag_result(r, sys._getframe(2))
        if len(ms) < 2: return r
        a = ms[0]; va, vr = vol(a), vol(r)
        if va - vr < MIN_MM3 or not is_ancestor(a) or not covers_part(r):
            return r
        caller = sys._getframe(2)
        rec = dict(op="inter", site=site(caller), removed=va - vr, cutters=[])
        try:
            x = ODIFF(a, *ms[1:]); vx = vol(x)
        except Exception:
            x, vx = None, 0.0
        if vx >= MIN_MM3:
            tags = [m.metadata.get("_ledger", []) for m in ms[1:]]
            rec["cutters"].append(dict(tag=tags[0] if tags else [], fn=ms[1].metadata.get("_ledger_fn", {}), vol=vx, x=x, envelope=True))
        RECS.append(rec)
        _dbg("inter", rec["site"], a, ms[1:])
        return r

    S._LEDGER_DIFF = hook_diff
    S._LEDGER_INTER = hook_inter
    exec("def _w_diff(a, *ms):\n    return _LEDGER_DIFF(a, ms)\n"
         "def _w_inter(*ms):\n    return _LEDGER_INTER(ms)\n", S.__dict__)
    S.diff.__code__ = S.__dict__["_w_diff"].__code__
    S.inter.__code__ = S.__dict__["_w_inter"].__code__

    sys.setprofile(prof)
    try:
        m = fn()
        m = WF.apply(pid, m)
        m = stamp(m, pid)
        if pid in CLEAN_AFTER_STAMP:
            m = clean_print_topology(m)
    finally:
        sys.setprofile(None)
    F = m
    tb = time.time() - t0

    # ── 汇总：每把刀 → 成品里还缺的块（按「代码行 + 这一行调的函数」分组）
    feats = _screw_features(pid, body)
    try:
        regions = json.load(open(os.path.join(ROOT, "duckstructure", "data", "regions_2026-09-17", "regions.json"), encoding="utf-8"))
    except Exception:
        regions = {}
    groups = {}
    for rec in RECS:
        dsite = rec["site"]
        dfile, dline, dfunc = dsite[0] if dsite else ("?", 0, "?")
        for c in rec["cutters"]:
            tg = [tuple(t) for t in (c["tag"] or [])]
            fn = c.get("fn") or {}
            idx = next((i for i, (f, l, fu) in enumerate(tg) if f == dfile and fu == dfunc), None)
            if idx is None:
                idx = next((i for i, (f, l, fu) in enumerate(tg) if not f.endswith("s288.py") and fu not in ("wbox", "cyl", "ybox", "bx", "union", "placed", "hull", "ybox_s", "<listcomp>")), None)
            if idx is None:
                rf, creator = (dfile, dline, dfunc), fn.get("fn", "?")
            else:
                rf = tg[idx]; creator = tg[idx - 1][2] if idx > 0 else fn.get("fn", "?")
            key = f"{rf[0]}:{rf[1]}:{creator}" + (f":{fn.get('fname')}" if fn.get("fname") else "")   # 同一行多把区域刀按文件分开
            try:
                lo_, hi_ = F.bounds[0] - 2.0, F.bounds[1] + 2.0
                clip = trimesh.creation.box(extents=hi_ - lo_, transform=trimesh.transformations.translation_matrix((lo_ + hi_) / 2))
                net = ODIFF(OINTER(c["x"], clip), F)
            except Exception:
                net = None
            vnet = vol(net)
            if vnet < MIN_MM3:
                continue
            g = groups.setdefault(key, dict(file=rf[0], line=rf[1], func=rf[2], creator=creator, diff_site=f"{dfile}:{dline}", ops=set(),
                                            chain=set(), net=0.0, meshes=[], envelope=bool(c.get("envelope")), fn=fn))
            g["ops"].add(rec["op"]); g["net"] += vnet; g["meshes"].append(net)
            g["chain"].update(fu for (_, _, fu) in tg); g["chain"].add(fn.get("fn", ""))
    # 成品的薄面（<0.45 mm = 放不下一条挤出线）：后面按「离哪个缺口 ≤0.6 mm」归给那一刀 → 尖刺 / 没切完的三角片
    ceF = F.triangles_center
    thF = np.empty(len(ceF))
    for k0 in range(0, len(ceF), 3000):          # 分批：H03（9 万面）一次算会爆内存（09-27 film_scan 同坑）
        k1 = min(len(ceF), k0 + 3000)
        thF[k0:k1] = trimesh.proximity.thickness(F, ceF[k0:k1], exterior=False, normals=F.face_normals[k0:k1], method="ray")
    thin_mask = np.nan_to_num(thF, nan=9.0, posinf=9.0) < 0.45
    out_rows, npz = [], {}
    for k, g in sorted(groups.items(), key=lambda kv: -kv[1]["net"]):
        code, inl, cm = _src(os.path.join(ROOT, g["file"]), g["line"])
        chain = sorted(x for x in g["chain"] if x)
        cat = "修形/包络" if g["envelope"] else next((c for c, keys in CATS if any(any(kk.lower() in fu.lower() for kk in keys) for fu in [g["creator"]] + chain)), "其他（看注释）")
        net = trimesh.util.concatenate(g["meshes"]) if len(g["meshes"]) > 1 else g["meshes"][0]
        comps = []
        for q in net.split(only_watertight=False):
            vq = abs(float(q.volume))
            if vq >= MIN_MM3:
                comps.append(dict(vol=round(vq, 2), lo=np.round(q.bounds[0], 2).tolist(), hi=np.round(q.bounds[1], 2).tolist(),
                                  c=np.round(q.vertices.mean(0), 2).tolist()))
        # 尖刺：成品薄面里离这个缺口 ≤0.6 mm 的
        spike_area, spike_min = 0.0, None
        if thin_mask.any():
            idxs = np.where(thin_mask)[0]
            lo, hi = net.bounds[0] - 0.6, net.bounds[1] + 0.6
            cand = idxs[((ceF[idxs] > lo) & (ceF[idxs] < hi)).all(1)]
            if len(cand):
                # 最近点用「缺口表面密采样 + KD 树」代替精确点到面距离：H03 的缺口网格几十万面，精确算法按包围盒找候选三角会爆内存
                from scipy.spatial import cKDTree
                nsamp = int(min(60000, max(3000, net.area * 25)))
                samp = np.vstack([net.vertices, trimesh.sample.sample_surface(net, nsamp, seed=3)[0]])
                dd = cKDTree(samp).query(ceF[cand], k=1)[0]
                hit = cand[dd <= 0.6]
                if len(hit):
                    spike_area = float(F.area_faces[hit].sum()); spike_min = float(np.nanmin(thF[hit]))
        # 螺丝：只对「让位类」刀查（孔 / 舵机腔 / 轴承座 / 刻字本来就贴着螺丝）；只算孔在料里的那几站
        near = []
        if cat in ("真实姿态区域刀", "运动扫掠让位", "插头/线让位", "壳/他件让位", "修形/包络", "其他（看注释）"):
            pts = trimesh.sample.sample_surface(net, 3000, seed=2)[0] if len(net.faces) else np.zeros((0, 3))
            for h in feats:
                if not len(pts): break
                ab = h["b"] - h["a"]; L2 = float(ab @ ab)
                if L2 < 1e-9: continue
                t = np.clip(((pts - h["a"]) @ ab) / L2, 0, 1)
                foot = h["a"] + np.outer(t, ab); dist = np.linalg.norm(pts - foot, axis=1) - h["r"]
                sel = dist < 1.5
                if not sel.any(): continue
                u = ab / np.sqrt(L2); e1 = np.cross(u, [1, 0, 0] if abs(u[0]) < 0.9 else [0, 1, 0]); e1 /= np.linalg.norm(e1); e2 = np.cross(u, e1)
                th_ = np.linspace(0, 2 * np.pi, 24, endpoint=False)
                wall = []
                for p0, dv in zip(foot[sel], dist[sel]):
                    ring = p0 + (h["r"] + 0.35) * (np.outer(np.cos(th_), e1) + np.outer(np.sin(th_), e2))
                    if F.contains(ring).mean() >= 0.5: wall.append(dv)
                if wall:
                    near.append(dict(feature=h["id"], kind=h["kind"], wall_left_mm=round(float(min(wall)), 2), purpose=h["purpose"][:40]))
        prov = {}
        fn = g["fn"] or {}
        if fn.get("fn") == "region_cut" and fn.get("fname"):
            for pair, v in regions.items():
                for pn, pv in (v.get("parts") or {}).items():
                    if str(pv.get("stl", "")).endswith(fn["fname"]):
                        prov = dict(pair=pair, defect_poses=v.get("defect_poses"), union_mm3=pv.get("union_mm3"), mirrored=fn.get("mirror"))
        elif fn.get("fn") == "sweep_of":
            prov = {kk: fn.get(kk) for kk in ("swept", "body", "lo", "hi", "n", "grow") if kk in fn}
        n_ = len(out_rows) + 1
        npz[f"n{n_}_v"] = net.vertices.astype(np.float32); npz[f"n{n_}_f"] = net.faces.astype(np.int32)
        gc.collect()
        if DBG:
            print(f"  [dbg {time.time()-t0:6.0f}s] 汇总 #{n_} {k[:60]} net {len(net.faces)} 面，峰值 RSS {resource.getrusage(resource.RUSAGE_SELF).ru_maxrss/1e9:.2f} GB", flush=True)
        out_rows.append(dict(n=n_, where=f"{g['file']}:{g['line']}", func=g["func"], creator=g["creator"], diff_site=g["diff_site"], category=cat,
                             code=code, inline_comment=inl, comment=cm, net_mm3=round(g["net"], 2), comps=comps,
                             spike_mm2=round(spike_area, 2), spike_min_mm=None if spike_min is None else round(spike_min, 3),
                             near_screw=near, provenance=prov, chain=chain[:14]))
    res = dict(part=pid, body=body, placed=pname, build_s=round(tb, 1), vol_rebuilt=round(vol(F), 2), vol_placed=round(vol(ref), 2),
               vol_match=abs(vol(F) - vol(ref)) < 0.5, n_records=len(RECS), thin_total_mm2=round(float(F.area_faces[thin_mask].sum()), 2),
               notches=out_rows, screw_features=[dict(id=h["id"], kind=h["kind"], purpose=h["purpose"][:40]) for h in feats])
    os.makedirs(out, exist_ok=True)
    json.dump(res, open(os.path.join(out, f"{pid}.json"), "w"), ensure_ascii=False, indent=1)
    np.savez_compressed(os.path.join(out, f"{pid}_notches.npz"), **npz)
    print(f"{pid}: 重建 {tb:.0f}s，成品 {vol(F):.1f} vs placed {vol(ref):.1f}，缺口来源 {len(out_rows)} 条，记录 {len(RECS)}，总耗时 {time.time()-t0:.0f}s", flush=True)


def _seg_dist(P, a, b):
    ab = b - a; L2 = float(ab @ ab)
    if L2 < 1e-12: return np.linalg.norm(P - a, axis=1)
    t = np.clip(((P - a) @ ab) / L2, 0, 1)
    return np.linalg.norm(P - (a + np.outer(t, ab)), axis=1)


def _screw_features(pid, body):
    import yaml
    from duckstructure.lib import TW
    T = TW(body)
    W = lambda p: (T @ np.r_[np.asarray(p, float), 1.0])[:3]
    F = yaml.safe_load(open(os.path.join(ROOT, "tools", "gate", "data", "features.yaml"), encoding="utf-8"))["features"]
    out = []
    for f in F:
        if f.get("part") != pid: continue
        if f.get("kind") not in ("screw_hole", "horn_hole", "pilot_hole", "counterbore", "spot_face", "boss", "nut_pocket", "through_hole"): continue
        g = f.get("geom") or {}
        insts = g.get("instances") or [g]
        for k, ins in enumerate(insts):
            sp = ins.get("axial_span_mm") or g.get("axial_span_mm")
            d = ins.get("nominal_d_mm", g.get("nominal_d_mm")); d = d.get("v") if isinstance(d, dict) else d
            if not sp or not d or not isinstance(sp[0], (list, tuple)) or isinstance(sp[0][0], (list, tuple)): continue
            try:
                out.append(dict(id=f"{f['id']}#{k}", kind=f.get("kind"), purpose=str(f.get("purpose", "")), a=W(sp[0]), b=W(sp[1]), r=float(d) / 2))
            except Exception:
                pass
    return out


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("parts", nargs="*"); ap.add_argument("--all", action="store_true"); ap.add_argument("--out", default=DEFAULT_OUT)
    ap.add_argument("--child", default="")
    a = ap.parse_args()
    if a.child:
        sys.path.insert(0, ROOT); os.chdir(ROOT)
        try:
            child(a.child, a.out)
        except Exception:
            traceback.print_exc(); sys.exit(3)
        sys.exit(0)
    parts = ALL if a.all else a.parts
    os.makedirs(a.out, exist_ok=True)
    for pid in parts:
        t = time.time()
        rc = subprocess.call([sys.executable, "-B", os.path.abspath(__file__), "--child", pid, "--out", a.out], cwd=ROOT,
                             env=dict(os.environ, PYTHONPATH=ROOT))
        print(f"[{pid}] rc={rc} {time.time()-t:.0f}s", flush=True)
