#!/usr/bin/env python3
"""入口：./.venv/bin/python -m duckstructure.build（在仓库根目录跑）。
等价于原 tools/cad/duck.py 的 __main__：导出 17 个 STL 到 cad/duck_s288/、跑零位/扫掠/装配/机械检查、
写 audit_summary.json 与 mechanical_audit.json；有干涉或未闭环接口时退出码 2。"""
import os, sys, json, time as _time, numpy as np, trimesh
_T0 = _time.time(); _TL = [_T0]
def _tick(label):
    """hr42：阶段计时打印（只打印到日志，不改任何产物/判据）。"""
    now = _time.time(); print(f"[hr42计时] {label} {now - _TL[0]:.2f}s 累计 {now - _T0:.1f}s", flush=True); _TL[0] = now
import duckstructure as D          # 传给 assembly_audit / mechanical_audit 的"duck 模块"（原来是 sys.modules[__name__]）
from duckstructure import s288, K
from duckstructure.s288 import bx, placed
from duckstructure.lib import P, B, ORDER, OUT, BX0, BX1, BZ0, BZ1, TW, sfw, hull, keep_main, wbox, mirror_y, MIRROR, battery_box
from duckstructure.lib import clean_print_topology
from duckstructure.stamps import stamp
from duckstructure import wire_fixings as _WF          # hr44：线束固定点
CLEAN_AFTER_STAMP = {"L01", "L02", "L03", "L04", "L05", "L06", "L07", "L10", "L13", "H03", "H05"}   # builder 里已做规范化的件
from duckstructure.legs import build_yaw2roll, build_hip, build_upper_leg, build_lower_leg, build_ankle_foot, build_ankle_rear_arm
from duckstructure.trunk import build_trunk, build_battery_door, build_trunk_shell
from duckstructure.head import adapter_box, build_face_plate
from duckstructure.head_top import build_head_top_shell                           # hr38：H05 上头壳（原版 top_head_shell 派生件）
from duckstructure import jaw as JAW                                              # hr38：嘴关节（第 15 颗 S288 + J01 + 6700ZZ）
from duckstructure.electronics import ELECTRONICS                                    # hr37：电子件占位实体                                          # hr28：转接板占位改在头里（tail.py 的托架版作废）
from duckstructure.neck import build_neck, build_neck_pitch, build_yrm, build_head_journal
from duckstructure.head import build_head_bracket, build_head_clamp, build_head_bottom_shell
from duckstructure.hip_pitch_bearing_rebuild import build_hip_pitch_sleeve
from duckstructure.head_bearing_rebuild import build_head_roll_adapter, build_head_yaw_adapter, build_head_bearing_cap, build_head_yaw_cap
from duckstructure.checks import EXPORT_STATS, export, vol, run_checks
from duckstructure.bearing_rebuild import build_hip_roll_sleeve

if __name__ == "__main__":
    # hr42（2026-09-24）：件生成改成"先声明、再按内容哈希增量 + 分组并行、最后按原顺序逐件导出"（duckstructure/build_fast.py:BuildPlan）。
    #   声明顺序 / 刻字 / 规范化 / 镜像 / 占位 / 导出与原来逐行对应，产出 STL 与 placed/ 逐字节相同（hr42 对账）；
    #   新加件：add(体, 件名, **生成函数本身（不带括号）**, 文件名)；不在 build_fast.GROUPS 里的件自动归 misc 组。
    #   --no-build-cache（或 DUCK_BUILD_CACHE=0）= 全部现切；--build-jobs N（或 DUCK_BUILD_JOBS，默认 3）= 并行工作进程数。
    #   --no-check-cache（或 DUCK_CHECK_CACHE=0）= run_checks / verify_mechanics 的检查原语缓存不读不写（tools/cad/check_cache.py）。
    #   打印 / 交付 / 复审前：python -m duckstructure.build --no-build-cache --no-check-cache（全量实跑，那份才是放行依据）。
    if "--no-check-cache" in sys.argv:
        os.environ["DUCK_CHECK_CACHE"] = "0"
    from duckstructure.build_fast import BuildPlan, mirror_extras, export_extras   # hr46：附属实体（钩）
    from duckstructure.tail import build_battery_lid          # 下面声明段里要用的模块都在装记录器的"生成期"之前 import 完：
    from duckstructure import eye as EYE                      # 模块体在 import 期执行，只按它们被谁用到进依赖，不算进每个组
    PLAN = BuildPlan()
    parts = []     # (body, name, world mesh)
    _tick("import 完成")
    steps = []
    def add(body, name, thunk, fname=None):
        steps.append(("add", body, name, thunk, fname))
    def extra(body, name, thunk):                     # 只参与检查的占位（主进程现算，便宜；工作进程跳过）
        steps.append(("extra", body, name, thunk, None))
    add("yaw2roll", "yaw2roll", build_yaw2roll, "L01_yaw2roll.stl")
    add("hip_l", "hip", build_hip, "L02_hip.stl")
    add("upper_leg_left", "upper_leg", build_upper_leg, "L03_upper_leg.stl")
    add("leg", "lower_leg", build_lower_leg, "L04_lower_leg.stl")
    _af = {}
    def _ankle(which):                                # 原 af, sole = build_ankle_foot()：一次调用出两件
        if not _af: _af["af"], _af["sole"] = build_ankle_foot()
        return _af[which]
    add("ankle_left", "ankle_foot", lambda: _ankle("af"), "L05_ankle_foot.stl"); add("ankle_left", "sole", lambda: _ankle("sole"), "L06_sole_TPU.stl")
    add("ankle_left", "ankle_rear_arm", build_ankle_rear_arm, "L07_ankle_rear_arm.stl")
    add("hip_l", "hip_roll_sleeve", build_hip_roll_sleeve, "L10_hip_roll_sleeve.stl")
    add("hip_l", "hip_pitch_sleeve", build_hip_pitch_sleeve, "L13_hip_pitch_sleeve.stl")
    steps.append(("mirror", None, None, None, None))  # 原：for body, name, m in list(parts): parts.append((MIRROR[body], name + "_R", mirror_y(m)))
    _tr = {}
    def _trunk():                                     # 原 tr = build_trunk()：T01 与两片壳共用同一个原坯（刻字前）
        if "m" not in _tr: _tr["m"] = build_trunk()
        return _tr["m"]
    add("trunk_base", "trunk", _trunk, "T01_trunk.stl")
    add("trunk_base", "battery_door", build_battery_door, "B01_battery_door.stl")
    # hr28（09-21 用户定）：B02 尾部托架作废 —— 转接板进头里 Radxa 排针口袋下段（head.H03_ADP_*），像原版 HAT 一样随头走；占位挂 jaw_soft
    extra("jaw_soft", "zz_adapter", adapter_box)                                                    # 转接板占位 40×30×9，只参与检查
    # 电池实体也参加检查（BAT_CLR 已经含在仓腔里，这里放的是电池本体尺寸）
    extra("trunk_base", "zz_battery", battery_box)          # hr48：占位体尺寸/站位统一取 lib.battery_box()（20 厚电池、x 在 BX0_BAT..BX1 居中）
    add("trunk_base", "shell_L", lambda: build_trunk_shell(1, _trunk()), "T02_shell_L.stl")
    add("trunk_base", "shell_R", lambda: build_trunk_shell(-1, _trunk()), "T03_shell_R.stl")
    from duckstructure.tail import build_battery_lid                                            # hr39f：方案 (f) 电池顶盖（取消总开关，开盖拔 XT30 断电）
    add("trunk_base", "battery_lid", build_battery_lid, "B03_battery_lid.stl")
    add("neck", "neck", build_neck, "N01_neck.stl")
    add("neck_pitch", "neck_pitch", build_neck_pitch, "N02_neck_pitch.stl")
    add("yaw_roll_motion", "yrm", build_yrm, "N03_yaw_roll.stl")
    add("yaw_roll_motion", "head_journal", build_head_journal, "N04_head_journal.stl")   # 头横滚 B 端轴颈销（09-12 ④）
    add("yaw_roll_motion", "head_roll_adapter", build_head_roll_adapter, "N05_head_roll_adapter.stl")
    add("neck_pitch", "head_yaw_adapter", build_head_yaw_adapter, "N06_head_yaw_adapter.stl")
    add("yaw_roll_motion", "head_bearing_cap", build_head_bearing_cap, "N07_head_bearing_cap.stl")
    add("yaw_roll_motion", "head_yaw_cap", build_head_yaw_cap, "N08_head_yaw_cap.stl")
    add("jaw_soft", "head_bracket", build_head_bracket, "H01_head_bracket.stl")
    add("jaw_soft", "head_clamp", build_head_clamp, "H02_head_clamp.stl")
    add("jaw_soft", "head_bottom_shell", build_head_bottom_shell, "H03_head_bottom_shell.stl")
    add("jaw_soft", "face_plate", build_face_plate, "H04_face_plate.stl")                      # hr37：脸板（替换原版 face_part）
    add("jaw_soft", "head_top_shell", build_head_top_shell, "H05_head_top_shell.stl")          # hr38：上头壳派生件（通风槽/声孔/喇叭卡座/功放卡座），取代 placed/ 里的 orig_top_head_shell
    add("jaw_soft", "jaw", JAW.build_jaw, "J01_jaw.stl")                                       # hr38：活动嘴；hr41：v2（最后从下装，两毂各 3 颗 M2 沉头）
    add("jaw_soft", "jaw_adapter", JAW.build_jaw_adapter, "J02_jaw_adapter.stl")               # hr41：+y 舵机法兰转接盘（一次装好不拆）
    add("jaw_soft", "jaw_journal", JAW.build_jaw_journal, "J03_jaw_journal.stl")               # hr41：−y 可拆轴颈盘（进 6700 内圈）
    from duckstructure.head import build_camera_clamp
    add("jaw_soft", "camera_clamp", build_camera_clamp, "H06_camera_clamp.stl")                # hr41：摄像头内侧压框（PCB 贴 H04 内面）
    # was_until_2026_09_25_hr43d: from duckstructure.head import build_amp_tray
    # was_until_2026_09_25_hr43d: add("jaw_soft", "amp_tray", build_amp_tray, "H07_amp_tray.stl")                            # hr43：功放托板（直排针功放；3×M2×8 拧 H01 旧 Radxa 立柱端面，F35）
    from duckstructure.head import build_adapter_tray, build_amp_bracket
    add("jaw_soft", "adapter_tray", build_adapter_tray, "H08_adapter_tray.stl")                # hr43d：转接板托板（B′，取代 H07；3×M2×8 拧 H01 旧 Radxa 立柱端面，F35；板泡棉胶 + 2 扎带）
    add("jaw_soft", "amp_bracket", build_amp_bracket, "H09_amp_bracket.stl")                  # hr43d：功放支架（E′，后脑 +y；F38 2×M2×6 拧 H03 凸台，功放 F36 2×M2×5）
    from duckstructure.head import build_tof_clamp
    add("jaw_soft", "tof_clamp", build_tof_clamp, "H10_tof_clamp.stl")                          # hr52：ToF 压条（v6-VL53L5CX 雷达；2×M2×6 从背面拧进 H04 两根柱）
    from duckstructure import eye as EYE                                                          # hr39c：圆眼装饰三件（白外圈 / 黑眼珠 / 高光点，每件单色，粘在 H04 前面）
    for _pid, (_fname, _nm, _fn) in EYE.PARTS.items():
        add("jaw_soft", _nm, _fn, _fname)
    # ── 执行：主进程先按组键判命中、未命中的组起工作进程并行现切；然后按上面的声明顺序逐件取回、导出（工作进程只现切本组件，存 npz 后退出）
    if not PLAN.worker:
        PLAN.prepare([(b, n, f, f[:3] in CLEAN_AFTER_STAMP) for k, b, n, t, f in steps if k == "add" and f])
        _tick("生成（分组工作进程 / 缓存读回）")
    for kind, body, name, thunk, fname in steps:
        if kind == "mirror":
            if PLAN.worker: continue
            for b_, n_, m_ in list(parts):
                parts.append((MIRROR[b_], n_ + "_R", mirror_y(m_)))
                mirror_extras(n_, n_ + "_R", mirror_y)          # hr46：镜像件的附属实体还没做，左件有就报错
            _tick("镜像右腿")
            continue
        if kind == "extra":
            if not PLAN.worker: parts.append((body, name, thunk()))
            continue
        if PLAN.worker:
            if fname and PLAN.wants(name):
                # hr15：每个打印件刻件号（stamps.STAMPS 世界系位置；小回转件点码）。刻在 builder 之后、镜像/审计/导出之前，
                # 这样右腿镜像件、placed/ 与所有审计看到的都是刻过字的几何。原来过了 clean_print_topology 的件刻完再过一遍。
                m = thunk() if callable(thunk) else thunk
                m = _WF.apply(fname[:3], m)                   # hr44：线束固定点（两孔扎带位 / 扎带桥，duckstructure.wire_fixings），刻字前
                m = stamp(m, fname[:3])
                if fname[:3] in CLEAN_AFTER_STAMP: m = clean_print_topology(m)
                PLAN.produce(name, body, fname, m, fname[:3] in CLEAN_AFTER_STAMP)
            continue
        m = PLAN.mesh(name) if fname else (thunk() if callable(thunk) else thunk)
        parts.append((body, name, m))
        if fname: export(m, fname, body)
        _tick(f"刻字/导出 {name}")
    PLAN.end_of_parts()                               # 工作进程：记依赖、存结果、在这里退出
    _tick("圆眼")
    for body, name, fn in ELECTRONICS: parts.append((body, name, fn()))                          # hr37：zz_sbc/mic/camera/imu/ubec…；hr39f：开关删除，+ zz_xt30（电池顶对插头+平衡头）
    # hr38：头部三个原版件全部由派生件取代，placed/ 里不再有 orig_*
    #   face_part → H04（hr37）、top_head_shell → H05（hr38）、jaw → J01（hr38）。原版 STL 仍是毛坯与合缝基准（frozen.yaml sha256 不变）。
    _tick("电子件占位")
    # 诊断：横滚舵机背后到底壳内壁的距离
    probe = wbox((-34, -25, 224), (-19.4, 10, 247))
    for p in B["jaw_soft"]["parts"]:
        if p["mesh"] in ("top_head_shell", "bottom_head_shell"):
            m = K.mesh(p["mesh"]); m.apply_transform(TW("jaw_soft") @ p["T"]); v, it = vol(probe, m)
            print(f"  [诊断] {p['mesh']} 在横滚舵机背后区域的材料: {v/1000:.2f} cm³ x∈{np.round(it.bounds[:,0],1) if it is not None else '-'}")
    # 舵机占位
    servos = []
    for n in ORDER:
        for i, s in enumerate(B[n]["servos"]):
            if s["drives"] is None: continue          # 下颚舵机 V1 不装
            R = sfw(n, i)
            servos.append((n, f"servo_{n}_{s['drives'].replace(':self','')}", placed(s288.servo_mesh(), R)))
    # hr38：第 15 颗 = 嘴舵机。MJCF robot_walk.xml 里下颚不是关节 → kin.load() 给它 drives=None（上面的循环跳过），位姿由 jaw.servo_R() 自己定（法兰 +y、顶端 −x）
    servos.append(("jaw_soft", "servo_jaw_soft_jaw", JAW.servo_mesh_world()))
    # 金属轴承必须参加整机多关节检查；不能只量座孔而漏掉完整实体。
    from assembly_audit import bearing_specs
    bearings = []
    for spec in bearing_specs(D):
        raw = spec["shape"].to_mesh64()
        mesh = trimesh.Trimesh(vertices=np.asarray(raw.vert_properties)[:, :3],
                               faces=np.asarray(raw.tri_verts), process=False)
        bearings.append((spec["mount"], "bearing_" + spec["name"], mesh))   # mount = 外圈压在哪个连杆里（bearing_specs 里逐颗写明）
    bearings.append(("jaw_soft", "bearing_jaw", JAW.bearing_mesh_world()))   # hr38：嘴惰轮侧 6700ZZ（外圈在 H01 上半拱 + H03 下半环拼的 Ø15.1 座）
    allm = parts + servos + bearings
    _tick("诊断+舵机/轴承占位")
    xxd = os.path.join(OUT, "xx"); os.makedirs(xxd, exist_ok=True)
    for f in os.listdir(xxd): os.remove(os.path.join(xxd, f))
    checks = run_checks(allm, export_intersections=True)
    _tick("run_checks")
    checks["stl"] = EXPORT_STATS
    with open(os.path.join(OUT, "audit_summary.json"), "w") as f:
        json.dump(checks, f, ensure_ascii=False, indent=2)
    from mechanical_audit import verify_mechanics
    import check_cache as _CC; _CC.set_context("verify_mechanics")       # hr42：未命中日志的"在算谁"（run_checks 里逐对设过，这里换成整段标签）
    mechanics = verify_mechanics(D, allm)
    _tick("verify_mechanics")
    with open(os.path.join(OUT, "mechanical_audit.json"), "w") as f:
        json.dump(mechanics, f, ensure_ascii=False, indent=2)
    # 局部视图：每个打印件 + 它周围的舵机，按世界坐标（看图方便）
    ld = os.path.join(OUT, "local"); os.makedirs(ld, exist_ok=True)
    import shutil
    for f in os.listdir(ld): shutil.rmtree(os.path.join(ld, f))
    for (b, nm, m) in parts:
        if nm.endswith("_R") or nm.startswith("orig_"): continue
        d = os.path.join(ld, nm); os.makedirs(d, exist_ok=True); keep_main(m, nm).export(os.path.join(d, "part.stl"))
        for (sb, snm, sm) in servos:
            v, it = vol(sm, hull(m, m))          # 舵机与该件凸包相交 → 相关舵机
            if v > 50: sm.export(os.path.join(d, snm.replace("servo_", "servo__") + ".stl"))
        if nm == "head_bracket":
            for (sb, snm, sm) in parts:
                if snm.startswith("orig_bottom"): sm.export(os.path.join(d, "orig_bottom.stl"))
    _tick("local/ 局部视图")
    # 两套渲染场景：scene_shell（装上原版躯干壳/头壳 = 完整鸭子）、scene_skeleton（剥掉所有壳 = 骨架）
    SHELLS = ("head_top_shell", "head_bottom_shell", "jaw", "jaw_adapter", "jaw_journal", "face_plate", "shell_L", "shell_R", "battery_lid", "eye_white", "eye_pupil", "eye_highlight")   # hr38：顶壳→H05、嘴→J01；hr39c：+ 圆眼三件；hr39f：+ B03 顶盖（壳的一块）；hr41：+ J02/J03（外露的盘）
    batm = battery_box()
    for scn, with_shell in (("scene_shell", True), ("scene_skeleton", False)):
        sd = os.path.join(OUT, scn); os.makedirs(sd, exist_ok=True)
        for f in os.listdir(sd): os.remove(os.path.join(sd, f))
        for (b, nm, m) in allm:
            if not with_shell and nm in SHELLS: continue
            keep_main(m, nm).export(os.path.join(sd, (nm if not nm.startswith("servo") else nm.replace("servo_", "servo__")) + ".stl"))
        batm.export(os.path.join(sd, "zzbattery.stl"))
    _tick("scene_shell/scene_skeleton")
    sc = trimesh.util.concatenate([m for (_, _, m) in allm]); sc.export(os.path.join(OUT, "assembly_preview.stl"))
    _tick("assembly_preview")
    pd = os.path.join(OUT, "placed"); os.makedirs(pd, exist_ok=True)
    for f in os.listdir(pd): os.remove(os.path.join(pd, f))
    for (b, nm, m) in allm: keep_main(m, nm).export(os.path.join(pd, (nm if not nm.startswith("servo") else nm.replace("servo_", "servo__")) + ".stl"))
    # hr46（2026-09-26）：件的附属实体（线束钩 / 卡槽；件的生成函数里 build_fast.produce_extra 登记）→ hooks/<件号>_<锚点id>.stl。
    #   与主网格同一条缓存记录（命中读回、未命中工作进程产出），增量与无缓存 build 逐字节相同；第 6 层只读这些文件。全量 build 整目录重建。
    _hooks = export_extras([n for k, b, n, t, f in steps if k == "add" and f], PLAN.fname_of, clear_all=True)
    _tick(f"hooks/（{len(_hooks)} 个附属实体）")
    # 2026-09-22 快车道缓存：把每个打印件（含镜像件）**布尔后、keep_main 前**的世界系网格原样存起来，build_fast.py 只重切改动件、其余从这里读回
    # （placed/ 的 STL 经 float32 + 顶点合并后 L05 不再是 volume，布尔会拒；npz 是精确的内存几何）。manifest 记该件导出 STL 的 sha，读回前核对。
    from duckstructure.build_fast import write_cache
    _tick("placed/")
    write_cache(parts, plan=PLAN)                     # hr42：件条目带组键 / 网格摘要，清单顶层带 __hr42_groups__（下次全量 build 增量用）
    import shutil as _sh
    _sh.rmtree(os.path.join(OUT, "cache", "hr42_workers"), ignore_errors=True)
    with open(os.path.join(OUT, "cache", "hr42_build_report.json"), "w") as f:
        json.dump(PLAN.summary(), f, ensure_ascii=False, indent=1)
    print("[增量] " + json.dumps(PLAN.summary(), ensure_ascii=False))
    _tick("快车道缓存")
    print("assembly bbox", np.round(sc.bounds, 1).tolist())
    try:                                                     # hr44：线束（头内 + 头外）实体导出到 placed 旁 wires/ + 头外粗扫掠摘要（只报告，不改退出码）
        from duckstructure import wiring_body as _WB
        print(_WB.export_all(os.path.join(OUT, "wires")), flush=True)
    except Exception as _e:                                  # noqa: BLE001
        print(f"[线束] 导出 / 检查失败：{_e}", flush=True)
    # 生成产物完成后再给失败退出码，避免检查有真碰撞却被脚本当成通过。
    if not checks["sampled_collision_free"] or not mechanics["passed"] or mechanics["unresolved"]:
        print("[未放行] STL 已生成，但整机仍有干涉或未闭环接口；退出码 2。")
        sys.exit(2)
    print("[采样检查通过] 仍需组合姿态、装配实物与打印公差验证。")
