#!/usr/bin/env python3
"""快车道（2026-09-22）：只重切 --only 里的件，其余打印件从缓存读回，然后跑与 duckstructure.build **相同**的 run_checks + verify_mechanics，
写 audit_summary.json / mechanical_audit.json，导出 STL + placed/ 只写重切的件（含镜像件）。跳过 local/ scene_*/ assembly_preview。

    ./.venv/bin/python -B -m duckstructure.build_fast --only H03[,N02,...]        # 件号前缀（文件名前 3 位）

读回优先级：cad/duck_s288/cache/<name>.npz（manifest 里该件导出 STL 的 sha 与现在相同）→ placed/<name>.stl（trimesh 合并顶点后必须是 volume）→ 都不行就重切。
缓存由 build.py（全量）与本脚本（重切的件）在每次 build 末尾写入，所以第一次全量 build 之后快车道才有缓存；没缓存时退化成从 placed/ 读（09-22 实测：
除 L05/L06 外读 placed/ 与内存几何跑出的 audit/placed 逐字节相同，L05 placed 不是 volume 会自动重切）。
快车道只用于迭代；交付/复审前、改 assembly 结构或 Gate 层代码时仍跑全链（hrNN_go.sh）。"""
import os, sys, json, time, hashlib, argparse, re
import numpy as np, trimesh

CACHE_DIRNAME = "cache"

def _cache_dir():
    from duckstructure.lib import OUT
    d = os.path.join(OUT, CACHE_DIRNAME); os.makedirs(d, exist_ok=True); return d

def _sha(path):
    return hashlib.sha256(open(path, "rb").read()).hexdigest() if os.path.exists(path) else None

# ── hr46：件的附属实体（钩）登记表与存读导出住在 duckstructure/build_extras.py（进程内唯一一份；本文件被当 __main__ 跑时也共用）──
from duckstructure.build_extras import (EXTRA_DIRNAME, _EXTRAS, produce_extra, extras_of, mirror_extras,   # noqa: E402,F401
                                        _save_extras, _load_extras, export_extras, mesh_digest)


def write_cache(parts, plan=None):
    """parts = [(body, name, world mesh)]；只存有导出文件的打印件及其 _R 镜像（orig_*/zz_* 每次现生成，便宜）。
    hr42：plan（build.py 的 BuildPlan）给出时，件条目另记 hr42_group / hr42_group_key / hr42_mesh_digest，清单顶层记 __hr42_groups__
    （全量 build 增量用）；快车道（plan=None）重写的件条目不带这些字段 → 下次全量 build 该组必然未命中、重切。"""
    from duckstructure.lib import OUT
    d = _cache_dir(); mpath = os.path.join(d, "manifest.json")
    man = json.load(open(mpath)) if os.path.exists(mpath) else {}
    per, groups = plan.manifest_extra() if plan is not None else ({}, None)
    for body, name, m in parts:
        if name.startswith(("orig_", "zz_")): continue
        base = name[:-2] if (name.endswith("_R") and name[:-2] in FNAME) else name   # shell_R 不是 shell 的镜像（hr33 复审 m-01）
        fname = FNAME.get(base)
        if not fname: continue
        np.savez(os.path.join(d, name + ".npz"), v=np.asarray(m.vertices, dtype=np.float64), f=np.asarray(m.faces, dtype=np.int64))
        ex = _save_extras(d, name, clean=True)          # hr46：附属实体（钩）存进同一条记录，清单记 {标签: 摘要}
        man[name] = dict(body=body, export=fname, export_sha256=_sha(os.path.join(OUT, fname)), hr42_extras=ex, **per.get(name, {}))
    if groups is not None:
        man["__hr42_groups__"] = groups
    json.dump(man, open(mpath, "w"), ensure_ascii=False, indent=1)

# 件名 → 导出文件（与 build.py 的 add(...) 一一对应；改 build.py 时同步）
FNAME = {"yaw2roll": "L01_yaw2roll.stl", "hip": "L02_hip.stl", "upper_leg": "L03_upper_leg.stl", "lower_leg": "L04_lower_leg.stl",
         "ankle_foot": "L05_ankle_foot.stl", "sole": "L06_sole_TPU.stl", "ankle_rear_arm": "L07_ankle_rear_arm.stl",
         "hip_roll_sleeve": "L10_hip_roll_sleeve.stl", "hip_pitch_sleeve": "L13_hip_pitch_sleeve.stl",
         "trunk": "T01_trunk.stl", "battery_door": "B01_battery_door.stl", "shell_L": "T02_shell_L.stl", "shell_R": "T03_shell_R.stl",
         "battery_lid": "B03_battery_lid.stl",          # hr39f
         "neck": "N01_neck.stl", "neck_pitch": "N02_neck_pitch.stl", "yrm": "N03_yaw_roll.stl", "head_journal": "N04_head_journal.stl",
         "head_roll_adapter": "N05_head_roll_adapter.stl", "head_yaw_adapter": "N06_head_yaw_adapter.stl",
         "head_bearing_cap": "N07_head_bearing_cap.stl", "head_yaw_cap": "N08_head_yaw_cap.stl",
         "head_bracket": "H01_head_bracket.stl", "head_clamp": "H02_head_clamp.stl", "head_bottom_shell": "H03_head_bottom_shell.stl",
         "face_plate": "H04_face_plate.stl",
         # hr39c：hr38 起 build.py 已有 H05/J01（快车道当时没跟上，这里补齐）+ 圆眼三件
         "head_top_shell": "H05_head_top_shell.stl", "jaw": "J01_jaw.stl",
         "eye_white": "E01_eye_white.stl", "eye_pupil": "E02_eye_pupil.stl", "eye_highlight": "E03_eye_highlight.stl",
         # hr41：嘴两转接盘 + 摄像头压框
         "jaw_adapter": "J02_jaw_adapter.stl", "jaw_journal": "J03_jaw_journal.stl", "camera_clamp": "H06_camera_clamp.stl",
         # hr43：功放托板（hr43d 退役，条目留痕：不再 add）
         "amp_tray": "H07_amp_tray.stl",
         # hr43d：转接板托板（取代 H07）+ 功放支架（后脑 +y）
         "adapter_tray": "H08_adapter_tray.stl", "amp_bracket": "H09_amp_bracket.stl",
         # hr52：ToF 压条（v6-VL53L5CX 雷达）
         "tof_clamp": "H10_tof_clamp.stl"}

def _load_cached(name, fname):
    """返回 (world mesh, 来源) 或 (None, 原因)。"""
    from duckstructure.lib import OUT
    d = _cache_dir(); mpath = os.path.join(d, "manifest.json"); npz = os.path.join(d, name + ".npz")
    man = json.load(open(mpath)) if os.path.exists(mpath) else {}
    cur = _sha(os.path.join(OUT, fname))
    if name in man and os.path.exists(npz) and man[name].get("export_sha256") == cur and cur is not None:
        err = _load_extras(d, name, man[name].get("hr42_extras") or {})   # hr46：附属实体一起读回；缺 / 摘要不符 → 不用缓存，重切
        if err: return None, err
        z = np.load(npz); return trimesh.Trimesh(vertices=z["v"], faces=z["f"], process=False), "cache"
    pl = os.path.join(OUT, "placed", name + ".stl")
    if os.path.exists(pl):
        m = trimesh.load(pl, force="mesh")            # 默认 merge_vertices；不合并的话每条边都是洞边，is_volume=False
        if m.is_volume: return m, "placed"
        return None, "placed 不是 volume"
    return None, "无缓存无 placed"

def main():
    ap = argparse.ArgumentParser(); ap.add_argument("--only", required=True, help="件号前缀，逗号分隔，如 H03,N02")
    a = ap.parse_args(); ONLY = {s.strip() for s in a.only.split(",") if s.strip()}
    import duckstructure as D
    from duckstructure import s288, K
    from duckstructure.s288 import bx, placed
    from duckstructure.lib import P, B, ORDER, OUT, BX0, BX1, BZ0, BZ1, TW, sfw, keep_main, mirror_y, MIRROR, clean_print_topology, stl_stats, battery_box
    from duckstructure.stamps import stamp
    from duckstructure import wire_fixings as _WF        # hr44
    from duckstructure.build import CLEAN_AFTER_STAMP
    from duckstructure.legs import build_yaw2roll, build_hip, build_upper_leg, build_lower_leg, build_ankle_foot, build_ankle_rear_arm
    from duckstructure.trunk import build_trunk, build_battery_door, build_trunk_shell
    from duckstructure.head import adapter_box, build_head_bracket, build_head_clamp, build_head_bottom_shell, build_face_plate
    from duckstructure.electronics import ELECTRONICS
    from duckstructure.neck import build_neck, build_neck_pitch, build_yrm, build_head_journal
    from duckstructure.hip_pitch_bearing_rebuild import build_hip_pitch_sleeve
    from duckstructure.head_bearing_rebuild import build_head_roll_adapter, build_head_yaw_adapter, build_head_bearing_cap, build_head_yaw_cap
    from duckstructure.checks import EXPORT_STATS, export, run_checks
    from duckstructure.bearing_rebuild import build_hip_roll_sleeve
    t0 = time.time(); parts = []; rebuilt = set(); sources = {}
    def add(body, name, thunk, fname=None):
        if fname and fname[:3] not in ONLY:
            m, src = _load_cached(name, fname)
            if m is not None:
                sources[name] = src
                nt, deg, hole, nonman = stl_stats(os.path.join(OUT, fname)); mm = trimesh.load(os.path.join(OUT, fname), force="mesh")
                EXPORT_STATS.append(dict(file=fname, volume_mm3=float(mm.volume), holes=hole, solids=1, triangles=nt, degenerate=deg, nonmanifold=nonman, cached=src))
                parts.append((body, name, m)); return
            print(f"  [{name}] {src} → 重切", flush=True)
        m = thunk() if callable(thunk) else thunk
        if fname:
            m = _WF.apply(fname[:3], m)                         # hr44：线束固定点（两孔扎带位 / 扎带桥，duckstructure.wire_fixings）
            m = stamp(m, fname[:3])
            if fname[:3] in CLEAN_AFTER_STAMP: m = clean_print_topology(m)
            rebuilt.add(name); sources[name] = "rebuilt"
        parts.append((body, name, m))
        if fname: export(m, fname, body); print(f"  [重切] {fname} {time.time()-t0:.0f} s", flush=True)
    add("yaw2roll", "yaw2roll", build_yaw2roll, "L01_yaw2roll.stl")
    add("hip_l", "hip", build_hip, "L02_hip.stl")
    add("upper_leg_left", "upper_leg", build_upper_leg, "L03_upper_leg.stl")
    add("leg", "lower_leg", build_lower_leg, "L04_lower_leg.stl")
    _af = {}
    def _ankle(which):
        if not _af: _af["af"], _af["sole"] = build_ankle_foot()
        return _af[which]
    add("ankle_left", "ankle_foot", lambda: _ankle("af"), "L05_ankle_foot.stl")
    add("ankle_left", "sole", lambda: _ankle("sole"), "L06_sole_TPU.stl")
    add("ankle_left", "ankle_rear_arm", build_ankle_rear_arm, "L07_ankle_rear_arm.stl")
    add("hip_l", "hip_roll_sleeve", build_hip_roll_sleeve, "L10_hip_roll_sleeve.stl")
    add("hip_l", "hip_pitch_sleeve", build_hip_pitch_sleeve, "L13_hip_pitch_sleeve.stl")
    for body, name, m in list(parts):
        parts.append((MIRROR[body], name + "_R", mirror_y(m)))
        mirror_extras(name, name + "_R", mirror_y)           # hr46：镜像件的附属实体还没做，左件有就报错
        if name in rebuilt: rebuilt.add(name + "_R")
    _tr = {}
    def _trunk():
        if "m" not in _tr: _tr["m"] = build_trunk()
        return _tr["m"]
    add("trunk_base", "trunk", _trunk, "T01_trunk.stl")
    add("trunk_base", "battery_door", build_battery_door, "B01_battery_door.stl")
    parts.append(("jaw_soft", "zz_adapter", adapter_box()))
    parts.append(("trunk_base", "zz_battery", battery_box()))     # hr48：同 build.py，取 lib.battery_box()
    add("trunk_base", "shell_L", lambda: build_trunk_shell(1, _trunk()), "T02_shell_L.stl")
    add("trunk_base", "shell_R", lambda: build_trunk_shell(-1, _trunk()), "T03_shell_R.stl")
    from duckstructure.tail import build_battery_lid                                           # hr39f
    add("trunk_base", "battery_lid", build_battery_lid, "B03_battery_lid.stl")
    add("neck", "neck", build_neck, "N01_neck.stl")
    add("neck_pitch", "neck_pitch", build_neck_pitch, "N02_neck_pitch.stl")
    add("yaw_roll_motion", "yrm", build_yrm, "N03_yaw_roll.stl")
    add("yaw_roll_motion", "head_journal", build_head_journal, "N04_head_journal.stl")
    add("yaw_roll_motion", "head_roll_adapter", build_head_roll_adapter, "N05_head_roll_adapter.stl")
    add("neck_pitch", "head_yaw_adapter", build_head_yaw_adapter, "N06_head_yaw_adapter.stl")
    add("yaw_roll_motion", "head_bearing_cap", build_head_bearing_cap, "N07_head_bearing_cap.stl")
    add("yaw_roll_motion", "head_yaw_cap", build_head_yaw_cap, "N08_head_yaw_cap.stl")
    add("jaw_soft", "head_bracket", build_head_bracket, "H01_head_bracket.stl")
    add("jaw_soft", "head_clamp", build_head_clamp, "H02_head_clamp.stl")
    add("jaw_soft", "head_bottom_shell", build_head_bottom_shell, "H03_head_bottom_shell.stl")
    add("jaw_soft", "face_plate", build_face_plate, "H04_face_plate.stl")
    from duckstructure.head_top import build_head_top_shell
    from duckstructure import jaw as JAW, eye as EYE
    add("jaw_soft", "head_top_shell", build_head_top_shell, "H05_head_top_shell.stl")     # hr38（快车道 hr39c 补）
    add("jaw_soft", "jaw", JAW.build_jaw, "J01_jaw.stl")
    add("jaw_soft", "jaw_adapter", JAW.build_jaw_adapter, "J02_jaw_adapter.stl")        # hr41
    add("jaw_soft", "jaw_journal", JAW.build_jaw_journal, "J03_jaw_journal.stl")        # hr41
    from duckstructure.head import build_camera_clamp
    add("jaw_soft", "camera_clamp", build_camera_clamp, "H06_camera_clamp.stl")         # hr41
    # was_until_2026_09_25_hr43d: from duckstructure.head import build_amp_tray
    # was_until_2026_09_25_hr43d: add("jaw_soft", "amp_tray", build_amp_tray, "H07_amp_tray.stl")                     # hr43
    from duckstructure.head import build_adapter_tray, build_amp_bracket
    add("jaw_soft", "adapter_tray", build_adapter_tray, "H08_adapter_tray.stl")           # hr43d：转接板托板（B′，取代 H07）
    add("jaw_soft", "amp_bracket", build_amp_bracket, "H09_amp_bracket.stl")             # hr43d：功放支架（E′，后脑 +y）
    from duckstructure.head import build_tof_clamp
    add("jaw_soft", "tof_clamp", build_tof_clamp, "H10_tof_clamp.stl")                     # hr52：ToF 压条
    for _pid, (_fname, _nm, _fn) in EYE.PARTS.items():                                     # hr39c 圆眼三件
        add("jaw_soft", _nm, _fn, _fname)
    for body, name, fn in ELECTRONICS: parts.append((body, name, fn()))
    servos = []
    for n in ORDER:
        for i, s in enumerate(B[n]["servos"]):
            if s["drives"] is None: continue
            servos.append((n, f"servo_{n}_{s['drives'].replace(':self','')}", placed(s288.servo_mesh(), sfw(n, i))))
    servos.append(("jaw_soft", "servo_jaw_soft_jaw", JAW.servo_mesh_world()))           # hr38 第 15 颗（快车道 hr39c 补）
    from assembly_audit import bearing_specs
    bearings = []
    for spec in bearing_specs(D):
        raw = spec["shape"].to_mesh64()
        bearings.append((spec["mount"], "bearing_" + spec["name"], trimesh.Trimesh(vertices=np.asarray(raw.vert_properties)[:, :3], faces=np.asarray(raw.tri_verts), process=False)))
    bearings.append(("jaw_soft", "bearing_jaw", JAW.bearing_mesh_world()))              # hr38 嘴惰轮侧 6700ZZ（快车道 hr39c 补）
    allm = parts + servos + bearings
    n_src = {k: sum(1 for v in sources.values() if v == k) for k in ("cache", "placed", "rebuilt")}
    print(f"件就绪 {time.time()-t0:.0f} s：重切 {sorted(rebuilt)}；来源 {n_src}", flush=True)
    xxd = os.path.join(OUT, "xx"); os.makedirs(xxd, exist_ok=True)
    for f in os.listdir(xxd): os.remove(os.path.join(xxd, f))
    checks = run_checks(allm, export_intersections=True); checks["stl"] = EXPORT_STATS
    checks["fast_lane"] = dict(only=sorted(ONLY), rebuilt=sorted(rebuilt), sources=sources, note="快车道：未重切的件从缓存/placed 读回；交付前仍需全量 build")
    with open(os.path.join(OUT, "audit_summary.json"), "w") as f: json.dump(checks, f, ensure_ascii=False, indent=2)
    print(f"run_checks 完 {time.time()-t0:.0f} s", flush=True)
    from mechanical_audit import verify_mechanics
    mechanics = verify_mechanics(D, allm)
    with open(os.path.join(OUT, "mechanical_audit.json"), "w") as f: json.dump(mechanics, f, ensure_ascii=False, indent=2)
    print(f"mechanics 完 {time.time()-t0:.0f} s", flush=True)
    pd = os.path.join(OUT, "placed"); os.makedirs(pd, exist_ok=True)
    for (b, nm, m) in allm:
        if nm in rebuilt:
            keep_main(m, nm).export(os.path.join(pd, (nm if not nm.startswith("servo") else nm.replace("servo_", "servo__")) + ".stl"))
    # hr41：zz_* 电子件占位每次都是现生成的（不进缓存），但以前只有重切件写 placed/ → 改了 electronics.py 后 placed/zz_*.stl 一直是旧的。
    #   这里顺带把 zz_* 全部写回 placed/（便宜，几何只取决于 electronics.py / head.py 常量），让 placed/ 与源码一致。
    for (b, nm, m) in allm:
        if nm.startswith("zz_"):
            keep_main(m, nm).export(os.path.join(pd, nm + ".stl"))
    hooks = export_extras([nm for (b, nm, m) in parts if nm in rebuilt])   # hr46：重切件的附属实体（钩）→ hooks/<件号>_<标签>.stl
    if hooks: print(f"hooks/ 写入 {hooks}", flush=True)
    write_cache([(b, nm, m) for (b, nm, m) in parts if nm in rebuilt])
    print(f"placed/缓存 写入 {sorted(rebuilt)}；总 {time.time()-t0:.0f} s", flush=True)
    try:                                                     # hr44：线束（头内 + 头外）实体导出到 placed 旁 wires/ + 头外粗扫掠摘要（只报告，不改退出码）
        from duckstructure import wiring_body as _WB
        print(_WB.export_all(os.path.join(OUT, "wires")), flush=True)
    except Exception as _e:                                  # noqa: BLE001
        print(f"[线束] 导出 / 检查失败：{_e}", flush=True)
    if not checks["sampled_collision_free"] or not mechanics["passed"] or mechanics["unresolved"]:
        print("[未放行] 整机仍有干涉或未闭环接口；退出码 2。"); sys.exit(2)
    print("[采样检查通过] 快车道结论；交付前跑全量 build + 全链。")

# ═══════════════════ hr42（2026-09-24）：全量 build 的"按内容哈希增量 + 分组并行" ═══════════════════
# 全量 build（python -m duckstructure.build）不再在主进程里逐件现算：
#   · 件按 GROUPS 分组（同组件共享 memo / 中间件，如 L01 内部要算 L02、壳要用 T01 原坯），一组一个**全新**工作进程生成
#     （python -c 引导：先 tools/cad/build_trace.start() 装记录器，再 runpy 跑同一个 duckstructure.build，环境变量 DUCK_BUILD_WORKER=<组>）。
#     工作进程只算本组件（thunk → 刻字 → 规范化，与原来 add() 里完全同一串调用），把"刻字/清理后、keep_main 前"的世界系网格存 npz，
#     并记下本组**真实**依赖：生成期执行过的仓库代码所在文件 + 这些函数体里的 import 目标 + 上述文件的顶层 import 闭包 +
#     这些模块在 import 期模块体里调过的函数所在文件（迭代到不动点）；读过的非 .py 文件（含 import 期读的 MJCF）；列过的目录。
#   · 组键 = sha256(键版本, 组名与件表, Python/平台/几何库版本/射线后端, build.py/build_fast.py/build_trace.py/__init__.py 本身,
#     依赖代码文件逐个 sha, 依赖数据文件逐个 sha, 列过的目录的文件名清单 sha)。
#   · 下次全量 build：按上次记下的依赖**现算**组键；与记录相同且该组每件 npz 的内容摘要对得上 → 直接读回（命中）；否则整组重切。
#     读回的网格与现算逐字节相同（同一数组），之后的导出 / run_checks / verify_mechanics / placed / local / scene 全部照常全量跑。
#   · 宁可多算：依赖表只增不减的来源都进键；任何一个依赖文件变了整组重切；记录器没装上 / 依赖记录缺失 → 未命中。
#   · 开关：--no-build-cache 或 DUCK_BUILD_CACHE=0 → 全部组现切（仍走工作进程，仍记依赖）；--build-jobs N 或 DUCK_BUILD_JOBS（默认 3）。
BUILD_CACHE_SCHEMA = "hr42-build-v1"
GROUPS = {
    "legs": ["yaw2roll", "hip", "upper_leg", "lower_leg", "ankle_foot", "sole", "ankle_rear_arm"],
    "sleeves": ["hip_roll_sleeve", "hip_pitch_sleeve"],
    "trunk": ["trunk", "battery_door", "shell_L", "shell_R"],
    "tail": ["battery_lid"],
    "neck": ["neck", "neck_pitch", "yrm", "head_journal"],
    "headbrg": ["head_roll_adapter", "head_yaw_adapter", "head_bearing_cap", "head_yaw_cap"],
    "head": ["head_bracket", "head_clamp", "head_bottom_shell", "face_plate", "head_top_shell", "jaw",
             "eye_white", "eye_pupil", "eye_highlight",
             "jaw_adapter", "jaw_journal", "camera_clamp", "amp_tray", "tof_clamp"],     # hr41 新件；hr43 H07 amp_tray（J02/J03 与 J01 同出 jaw.py、H06 出 head.py）：件表里没有它们时这几个名字不起作用
}
MISC_GROUP = "misc"          # 表里没有的打印件（以后新加的件）自动归这一组：不会漏生成
_GROUP_OF = {n: g for g, ns in GROUPS.items() for n in ns}
_REPO = os.path.realpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
_ALWAYS = ["duckstructure/build.py", "duckstructure/build_fast.py", "duckstructure/__init__.py", "tools/cad/build_trace.py"]
_NOT_EXPANDED = {"duckstructure/build.py", "duckstructure/build_fast.py"}   # 调度壳：只进键（本身 sha），不展开它们 import 的全部件模块


def group_of(name):
    return _GROUP_OF.get(name, MISC_GROUP)


def _sha_path(rel):
    p = os.path.join(_REPO, rel)
    return hashlib.sha256(open(p, "rb").read()).hexdigest() if os.path.isfile(p) else "MISSING"


def _path_kind(rel):
    """路径现在的存在性（build_trace 探测记录的同一口径）：file / dir / absent / other。"""
    import stat as _st
    try:
        m = os.stat(os.path.join(_REPO, rel)).st_mode
    except OSError:
        return "absent"
    return "dir" if _st.S_ISDIR(m) else ("file" if _st.S_ISREG(m) else "other")


def _listing_sha(rel):
    p = os.path.join(_REPO, rel)
    try:
        names = sorted(os.listdir(p))
    except OSError:
        return "MISSING"
    return hashlib.sha256("\n".join(names).encode()).hexdigest()


def _versions():
    import importlib.metadata as md, platform
    vs = [f"python={sys.version.split()[0]}", f"machine={platform.machine()}", f"system={platform.system()}"]
    for n in ("numpy", "trimesh", "manifold3d", "scipy", "shapely", "rtree", "networkx", "embreex", "mapbox-earcut", "triangle"):
        try:
            vs.append(f"{n}={md.version(n)}")
        except Exception:
            vs.append(f"{n}=-")
    vs.append(f"trimesh.ray.has_embree={bool(trimesh.ray.has_embree)}")
    return ";".join(vs)


# ── 静态部分：import 语句解析（只认仓库内模块：duckstructure/*.py 与 tools/cad/*.py）──
_AST = {}


def _ast(rel):
    if rel not in _AST:
        import ast
        _AST[rel] = ast.parse(open(os.path.join(_REPO, rel), encoding="utf-8").read(), filename=rel)
    return _AST[rel]


def _mod_file(pkg_mod):
    """'lib' → duckstructure/lib.py；不存在返回 None。"""
    rel = f"duckstructure/{pkg_mod}.py"
    return rel if os.path.isfile(os.path.join(_REPO, rel)) else None


def _resolve(node, importer):
    """一条 Import/ImportFrom → 它会读到的仓库模块文件（相对路径）集合。"""
    import ast
    out = set()
    in_pkg = importer.startswith("duckstructure/")
    if isinstance(node, ast.ImportFrom):
        mod = node.module or ""
        if node.level >= 1:
            if not in_pkg:
                return out
            if mod:
                f = _mod_file(mod.split(".")[0])
                if f: out.add(f)
            else:
                for a in node.names:
                    out.add(_mod_file(a.name) or "duckstructure/__init__.py")
        elif mod == "duckstructure":
            for a in node.names:
                out.add(_mod_file(a.name) or "duckstructure/__init__.py")
        elif mod.startswith("duckstructure."):
            f = _mod_file(mod.split(".")[1])
            if f: out.add(f)
        else:
            top = mod.split(".")[0]
            if os.path.isfile(os.path.join(_REPO, "tools/cad", top + ".py")):
                out.add(f"tools/cad/{top}.py")
    else:
        for a in node.names:
            n = a.name
            if n == "duckstructure":
                out.add("duckstructure/__init__.py")
            elif n.startswith("duckstructure."):
                f = _mod_file(n.split(".")[1])
                if f: out.add(f)
                if not a.asname:
                    out.add("duckstructure/__init__.py")          # `import duckstructure.x` 绑定的是包对象
            else:
                top = n.split(".")[0]
                if os.path.isfile(os.path.join(_REPO, "tools/cad", top + ".py")):
                    out.add(f"tools/cad/{top}.py")
    return out


def _imports_in(node, importer, top_level):
    """node 自己这一层作用域里的 import（不进嵌套的 def/lambda；top_level=True 时连模块级的 class 体也算，类体在 import 时执行）。"""
    import ast
    out = set()
    stack = list(ast.iter_child_nodes(node))
    while stack:
        n = stack.pop()
        if isinstance(n, (ast.Import, ast.ImportFrom)):
            out |= _resolve(n, importer)
        if isinstance(n, (ast.FunctionDef, ast.AsyncFunctionDef, ast.Lambda)):
            continue
        if isinstance(n, ast.ClassDef) and not top_level:
            continue
        stack.extend(ast.iter_child_nodes(n))
    return out


_TOP = {}


def _top_imports(rel):
    if rel not in _TOP:
        _TOP[rel] = _imports_in(_ast(rel), rel, True) if rel.endswith(".py") and os.path.isfile(os.path.join(_REPO, rel)) else set()
    return _TOP[rel]


def _code_nodes(rel, line, qualname):
    """代码对象 (co_firstlineno, co_qualname) → AST 节点。装饰过的函数 co_firstlineno 是第一个装饰器的行。对不上 → 返回整个模块（保守）。"""
    import ast
    tree = _ast(rel)
    name = qualname.rsplit(".", 1)[-1]
    if name == "<module>":
        return [tree], True
    hits = []
    for n in ast.walk(tree):
        if isinstance(n, (ast.FunctionDef, ast.AsyncFunctionDef, ast.ClassDef)):
            first = min([d.lineno for d in n.decorator_list] + [n.lineno])
            if first == line and n.name == name:
                hits.append(n)
        elif isinstance(n, ast.Lambda) and name == "<lambda>" and n.lineno == line:
            hits.append(n)
        elif isinstance(n, (ast.GeneratorExp, ast.ListComp, ast.SetComp, ast.DictComp)) and n.lineno == line and name.startswith("<"):
            hits.append(n)
    return (hits, False) if hits else ([tree], True)


def group_deps(rep):
    """build_trace.report() → {code: {依赖代码文件: sha}, data: {文件: sha}, lists: {目录: 清单 sha}, probes: {路径: 首次探测时的存在性},
    notes: [...], unstable: [...]}。"""
    notes = []
    seeds = set()
    for rel, line, q in rep["generation_codes"]:
        if not rel.endswith(".py"):
            continue
        seeds.add(rel)
        if rel in _NOT_EXPANDED:
            continue
        nodes, fallback = _code_nodes(rel, line, q)
        if fallback and q.rsplit(".", 1)[-1] != "<module>":
            notes.append(f"对不上 AST：{rel}:{line}:{q} → 按整模块收 import")
        for n in nodes:
            seeds |= _imports_in(n, rel, n is _ast(rel))
    for m in ("duckstructure/build.py", "duckstructure/__init__.py"):
        for rel, _l, _q in rep["import_attrib"].get(m, []):       # 调度壳/包 init 的模块体在 import 期调过的函数（保守收进来）
            seeds.add(rel)
    seen, todo = set(), list(seeds)
    while todo:
        f = todo.pop()
        if f in seen:
            continue
        seen.add(f)
        if f in _NOT_EXPANDED:
            continue
        nxt = set(_top_imports(f))
        nxt |= {rel for rel, _l, _q in rep["import_attrib"].get(f, [])}
        todo.extend(nxt - seen)
    code_files = sorted(set(seen) | set(_ALWAYS))
    # 稳定性：代码按"开跑时"的 sha（build_trace.start 拍的），收尾再核对一次；数据按打开时的 (mtime, size) 核对 —— 生成期间被改过就不给键（下次必然重切）
    unstable = []
    code = {}
    for rel in code_files:
        s0 = rep.get("py_sha_start", {}).get(rel)
        now = _sha_path(rel)
        if s0 is None:
            s0 = now; notes.append(f"{rel} 不在开跑快照里（按收尾时 sha 记）")
        if now != s0:
            unstable.append(f"代码 {rel} 在生成期间被改过")
        code[rel] = s0
    data = {}
    for rel in sorted(rep["reads"]):
        st0 = (rep.get("reads_stat") or {}).get(rel)
        try:
            st = os.stat(os.path.join(_REPO, rel)); st1 = [st.st_mtime_ns, st.st_size]
        except OSError:
            st1 = None
        if st0 is not None and st1 != list(st0):
            unstable.append(f"数据 {rel} 在生成期间被改过")
        data[rel] = _sha_path(rel)
    lists = {rel: _listing_sha(rel) for rel in sorted(rep["lists"])}
    # 存在性探测（第四任加）：记首次探测时的状态；数据文件已按内容进键的也照记（便宜、口径统一）
    probes = {rel: k for rel, k in sorted((rep.get("probes") or {}).items())}
    return dict(code=code, data=data, lists=lists, probes=probes, notes=notes, unstable=unstable)


def group_spec(g, names, body_of, fname_of, clean_of):
    return [[n, body_of.get(n), fname_of.get(n), bool(clean_of.get(n))] for n in names]


def group_key(g, spec, code_sha, data_sha, list_sha, versions=None, probe_kind=None):
    """组键 = sha256(键版本, 组名, 件表, 环境串, 每个依赖代码文件的 sha, 每个依赖数据文件的 sha, 每个列过的目录的清单 sha,
    每个探测过的路径的存在性)。
    工作进程用"开跑时"的 sha 算（生成期间有改动就不给键）；主进程下次 build 用"现在"的 sha 重算，对不上就重切。"""
    h = hashlib.sha256()
    def up(*xs):
        for x in xs:
            b = x if isinstance(x, bytes) else str(x).encode()
            h.update(len(b).to_bytes(8, "little")); h.update(b)
    up(BUILD_CACHE_SCHEMA, g, json.dumps(spec, ensure_ascii=False), versions or _versions())
    for rel in sorted(code_sha):
        up("code", rel, code_sha[rel])
    for rel in sorted(data_sha):
        up("data", rel, data_sha[rel])
    for rel in sorted(list_sha):
        up("list", rel, list_sha[rel])
    for rel in sorted(probe_kind or {}):
        up("probe", rel, probe_kind[rel])
    return h.hexdigest()


def group_key_now(g, spec, deps, versions=None):
    """按记录下来的依赖清单，用现在磁盘上的内容重算组键。"""
    return group_key(g, spec, {r: _sha_path(r) for r in deps["code"]}, {r: _sha_path(r) for r in deps["data"]},
                     {r: _listing_sha(r) for r in deps["lists"]}, versions,
                     {r: _path_kind(r) for r in (deps.get("probes") or {})})


_WORKER_BOOT = ("import sys, os; sys.path.insert(0, os.path.join(os.getcwd(), 'tools', 'cad')); import build_trace; build_trace.start(); "
                "import runpy; runpy.run_module('duckstructure.build', run_name='__main__', alter_sys=True)")


class BuildPlan:
    """build.py 主进程：prepare()（算组键、起工作进程）→ 每个 add() 取 mesh(name) → finish() 写缓存清单。
    工作进程（DUCK_BUILD_WORKER=<组>）：wants(name) 只对本组件为真 → produce(name, m) 存 npz → end_of_parts() 记依赖并退出。"""

    def __init__(self, argv=None):
        from duckstructure.lib import OUT
        argv = list(sys.argv if argv is None else argv)
        self.out = OUT
        self.worker = os.environ.get("DUCK_BUILD_WORKER") or None
        self.wdir = os.environ.get("DUCK_BUILD_WORKER_OUT") or None
        self.use_cache = not ("--no-build-cache" in argv or os.environ.get("DUCK_BUILD_CACHE", "1") in ("0", "off", "no"))
        jobs = os.environ.get("DUCK_BUILD_JOBS")
        if "--build-jobs" in argv:
            jobs = argv[argv.index("--build-jobs") + 1]
        self.jobs = max(1, int(jobs)) if jobs and str(jobs).isdigit() else 3
        self.meshes, self.status, self.t0 = {}, {}, time.time()
        self.body_of, self.fname_of, self.clean_of = {}, {}, {}
        self.groups_info = {}
        if self.worker:
            import build_trace
            if not build_trace.active():
                raise RuntimeError("工作进程没装依赖记录器（必须经 BuildPlan 的引导命令启动）")
            build_trace.mark_generation()
            self._produced = {}
            self._seen = set()

    # ── 工作进程 ──
    def wants(self, name):
        if self.worker: self._seen.add(name)                     # hr46：工作进程看到的全部件名（end_of_parts 核附属实体登记到了真实件上）
        return (not self.worker) or group_of(name) == self.worker

    def produce(self, name, body, fname, m, clean):
        v, f = np.asarray(m.vertices, dtype=np.float64), np.asarray(m.faces, dtype=np.int64)
        np.savez(os.path.join(self.wdir, name + ".npz"), v=v, f=f)
        self._produced[name] = dict(body=body, fname=fname, clean=bool(clean), digest=mesh_digest(v, f), t=round(time.time() - self.t0, 2),
                                    extras=_save_extras(self.wdir, name, clean=False))   # hr46：附属实体同目录存 npz，记 {标签: 摘要}

    def end_of_parts(self):
        if not self.worker:
            return
        import build_trace
        bad = sorted(n for n in _EXTRAS if n not in self._seen)          # hr46：登记到不存在的件名 = 生成函数写错了件名，不许静默丢
        if bad:
            raise RuntimeError(f"附属实体登记到了不存在的件 {bad}（produce_extra 的件名要与 build.py add() 的件名一致）")
        rep = build_trace.report()
        deps = group_deps(rep)
        vers = _versions()
        spec = [[n, r["body"], r["fname"], r["clean"]] for n, r in self._produced.items()]
        key = None if deps["unstable"] else group_key(self.worker, spec, deps["code"], deps["data"], deps["lists"], vers, deps["probes"])
        json.dump(dict(group=self.worker, produced=self._produced, spec=spec, key=key, deps=deps, trace=rep,
                       seconds=round(time.time() - self.t0, 1), versions=vers),
                  open(os.path.join(self.wdir, "result.json"), "w"), ensure_ascii=False, indent=1)
        sys.stdout.flush(); sys.stderr.flush()
        os._exit(0)

    # ── 主进程 ──
    def prepare(self, part_table):
        """part_table = [(body, name, fname, clean)]（build.py 的 add() 顺序）。算键、判命中、并行起工作进程、等齐。"""
        import subprocess, shutil
        from concurrent.futures import ThreadPoolExecutor
        for body, name, fname, clean in part_table:
            self.body_of[name], self.fname_of[name], self.clean_of[name] = body, fname, clean
        order = []
        for _b, name, _f, _c in part_table:
            g = group_of(name)
            if g not in order:
                order.append(g)
        members = {g: [n for _b, n, _f, _c in part_table if group_of(n) == g] for g in order}
        cdir = _cache_dir(); mpath = os.path.join(cdir, "manifest.json")
        man = json.load(open(mpath)) if os.path.exists(mpath) else {}
        ginfo = man.get("__hr42_groups__", {})
        vers = _versions()
        todo = []
        for g in order:
            spec = group_spec(g, members[g], self.body_of, self.fname_of, self.clean_of)
            rec = ginfo.get(g)
            why = None
            if not self.use_cache:
                why = "--no-build-cache"
            elif not rec or "deps" not in rec:
                why = "无依赖记录（首次或旧缓存）"
            elif rec.get("spec") != spec:
                why = "件表变了"
            elif not rec.get("key"):
                why = "上次生成期间依赖被改过，没给键：" + "; ".join((rec["deps"].get("unstable") or [])[:3])
            else:
                k = group_key_now(g, spec, rec["deps"], vers)
                if k != rec.get("key"):
                    changed = [r for r, s in rec["deps"]["code"].items() if s != _sha_path(r)]
                    changed += [r for r, s in rec["deps"]["data"].items() if s != _sha_path(r)]
                    changed += [r for r, s in rec["deps"]["lists"].items() if s != _listing_sha(r)]
                    changed += [f"{r}（{s}→{_path_kind(r)}）" for r, s in (rec["deps"].get("probes") or {}).items() if s != _path_kind(r)]
                    why = "依赖变了：" + (", ".join(changed[:6]) if changed else "版本/件表")
                else:
                    for n in members[g]:
                        e = man.get(n) or {}
                        npz = os.path.join(cdir, n + ".npz")
                        if not os.path.exists(npz) or e.get("hr42_group_key") != k:
                            why = f"{n} 缓存缺失或不是这把键写的"; break
                        z = np.load(npz)
                        if mesh_digest(z["v"], z["f"]) != e.get("hr42_mesh_digest"):
                            why = f"{n} 缓存内容摘要对不上"; break
                        err = _load_extras(cdir, n, e.get("hr42_extras") or {})   # hr46：附属实体（钩）一起读回、逐个核摘要
                        if err:
                            why = err; break
                        self.meshes[n] = trimesh.Trimesh(vertices=z["v"], faces=z["f"], process=False)
            if why is None:
                self.status[g] = dict(hit=True, key=rec["key"], deps=rec["deps"], spec=spec, seconds=rec.get("seconds"))
                print(f"[增量] 组 {g:8s} 命中（键 {rec['key'][:12]}…，{len(members[g])} 件读回缓存）", flush=True)
            else:
                for n in members[g]:
                    self.meshes.pop(n, None); _EXTRAS.pop(n, None)
                self.status[g] = dict(hit=False, why=why, spec=spec)
                todo.append(g)
                print(f"[增量] 组 {g:8s} 未命中 → 工作进程重切（{why}）", flush=True)
        if not todo:
            return
        wroot = os.path.join(cdir, "hr42_workers"); shutil.rmtree(wroot, ignore_errors=True); os.makedirs(wroot)
        est = {g: float((ginfo.get(g) or {}).get("seconds") or 0.0) for g in todo}
        todo.sort(key=lambda g: -est[g])                       # 最长的组先起（上次的实测秒数）

        def run(g):
            d = os.path.join(wroot, g); os.makedirs(d)
            env = dict(os.environ, DUCK_BUILD_WORKER=g, DUCK_BUILD_WORKER_OUT=d, PYTHONUNBUFFERED="1")
            t = time.time()
            with open(os.path.join(d, "worker.log"), "w") as lf:
                rc = subprocess.call([sys.executable, "-B", "-c", _WORKER_BOOT], cwd=_REPO, env=env, stdout=lf, stderr=subprocess.STDOUT)
            return g, rc, time.time() - t

        print(f"[增量] {len(todo)} 组现切，{min(self.jobs, len(todo))} 个工作进程并行：{todo}", flush=True)
        with ThreadPoolExecutor(max_workers=self.jobs) as ex:
            results = list(ex.map(run, todo))
        for g, rc, sec in results:
            d = os.path.join(wroot, g)
            sys.stdout.write(f"──── 工作进程 {g}（{sec:.1f} s，exit {rc}）日志 ────\n" + open(os.path.join(d, "worker.log")).read())
            if rc != 0 or not os.path.exists(os.path.join(d, "result.json")):
                raise RuntimeError(f"build 工作进程 {g} 失败（exit {rc}），见 {d}/worker.log")
            res = json.load(open(os.path.join(d, "result.json")))
            for n in members[g]:
                z = np.load(os.path.join(d, n + ".npz"))
                if n not in res["produced"] or mesh_digest(z["v"], z["f"]) != res["produced"][n]["digest"]:
                    raise RuntimeError(f"工作进程 {g} 没产出 {n} 或摘要不符")
                err = _load_extras(d, n, res["produced"][n].get("extras") or {})   # hr46：附属实体读回、核摘要
                if err:
                    raise RuntimeError(f"工作进程 {g} 的附属实体读不回：{err}")
                self.meshes[n] = trimesh.Trimesh(vertices=z["v"], faces=z["f"], process=False)
            spec = self.status[g]["spec"]
            if res.get("spec") != spec:
                raise RuntimeError(f"工作进程 {g} 的件表 {res.get('spec')} 与主进程 {spec} 不一致")
            self.status[g].update(key=res.get("key"), deps=res["deps"], seconds=round(sec, 1), notes=res["deps"].get("notes"))
            print(f"[增量] 组 {g} 现切完 {sec:.1f} s；依赖代码 {len(res['deps']['code'])} 个文件、数据 {len(res['deps']['data'])} 个、目录 {len(res['deps']['lists'])} 个"
                  + (f"；备注 {res['deps']['notes'][:3]}" if res["deps"].get("notes") else "")
                  + (f"；⚠ 生成期间依赖被改：{res['deps']['unstable'][:3]}（不记键，下次重切）" if res["deps"].get("unstable") else ""), flush=True)

    def mesh(self, name):
        return self.meshes[name]

    def manifest_extra(self):
        """write_cache 用：件 → (组, 组键, 网格摘要)；组 → 键/依赖/件表/秒数。"""
        per, groups = {}, {}
        for g, st in self.status.items():
            if "deps" not in st:
                continue
            groups[g] = dict(key=st.get("key"), deps=st["deps"], spec=st["spec"], seconds=st.get("seconds"),
                             last=("命中" if st.get("hit") else "现切：" + str(st.get("why"))))
            if not st.get("key"):
                continue
            for n, _b, _f, _c in st["spec"]:
                m = self.meshes.get(n)
                if m is not None:
                    per[n] = dict(hr42_group=g, hr42_group_key=st["key"],
                                  hr42_mesh_digest=mesh_digest(np.asarray(m.vertices), np.asarray(m.faces)))
        return per, groups

    def summary(self):
        hits = [g for g, s in self.status.items() if s.get("hit")]
        miss = [g for g, s in self.status.items() if not s.get("hit")]
        return dict(schema=BUILD_CACHE_SCHEMA, cache=self.use_cache, jobs=self.jobs, hit_groups=hits, rebuilt_groups=miss,
                    rebuilt_parts=[n for g in miss for n, *_ in self.status[g]["spec"]],
                    extras={n: sorted(t) for n, t in sorted(_EXTRAS.items()) if t},        # hr46：本次 build 的附属实体（件 → 标签）
                    why={g: self.status[g].get("why") for g in miss})


if __name__ == "__main__":
    main()
