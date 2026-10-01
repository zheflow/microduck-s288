"""导出与检查：STL 导出（闭合/单实体断言 + EXPORT_STATS）、体积相交、零位姿干涉 + 逐关节扫掠（run_checks）、
MJCF 全行程 SWEEP 与旧检查域 LEGACY_LIMITS。装配/机械审计（assembly_audit / mechanical_audit）由 build.py 调用。"""
import os, math, numpy as np, trimesh
from trimesh.transformations import rotation_matrix as rot
from .s288 import placed
from .lib import OUT, B, ORDER, TW, keep_main, stl_stats, to_local

def check_angles(lo, hi, step=2.5):
    """独立检查网格：以零度对齐的 2.5° 网格，另外包含两端；不复用切刀采样。"""
    # MJCF弧度换算会得到-90.000000000006等端点，不能把它和-90重复统计。
    # 仅消除低于1e-9度的浮点噪声，保留非网格端点（如±22度）。
    return sorted({round(float(a), 9) for a in [lo, hi,
                   *np.arange(math.ceil(lo / step) * step, hi + 1e-9, step)]})

# ───────────────────────────── 导出 / 装配 / 检查 ─────────────────────────────
EXPORT_STATS = []
def export(m, name, body=None):
    mm = to_local(m, body) if body else m
    mm = keep_main(mm, name)                  # 布尔留下的零体积碎片（L03 有 40 个、H01 有 4 个）不写进 STL
    path = os.path.join(OUT, name); mm.export(path)
    b = mm.bounds; v = mm.volume / 1000
    # 实体计数门槛 30 mm³（布尔碎片都 <30）；hr39c：整件本身不到 60 mm³ 的小件（E03 眼睛高光 Ø3.4×1.2 = 10.9 mm³）门槛降到整件体积的一半，否则整件被当碎片、报 0 个实体
    # hr41 落盘 2026-09-25（主设计 Lane C）：m1 —— 旧门槛 min(30, 0.5·V) 对小件太松（<60 mm³ 的件最多只有一个分量能过半件体积，nc==1 几乎恒真）。
    #   主设计 2026-09-25 定：thr = min(30.0, 1e−3·V)，nc = #{分量体积 ≥ thr}（有符号体积，空腔壳为负不计）。小件（V<30000）即「第二大分量 < 1e−3·V」；
    #   大件仍按 09-13 起的 30 mm³ 口径，不因 1e−3·V>30 放松（H03/H05 挂 30–48 mm³ 浮块必须仍红）。
    #   34 件 STL 核过全部 nc == 1（hr41_work/landing_0925/runs/t2_m1_components.json）。旧式：thr = min(30.0, 0.5 * V); nc = #{体积 > thr}
    thr = min(30.0, 1e-3 * float(mm.volume))
    comps = mm.split(only_watertight=False); nc = int(sum(c.volume >= thr for c in comps))
    nt, deg, hole, nonman = stl_stats(path)
    assert hole == 0, f"{name}: STL 有 {hole} 条洞边（真·不闭合），不能发去打印"
    if name.startswith("L04_"):
        assert deg == 0 and nonman == 0, f"{name}: 导出后仍有退化面/非流形共边，实体解释不唯一"
    assert nc == 1, f"{name}: 有 {nc} 个独立实体，不是一个打印件"
    EXPORT_STATS.append(dict(file=name, volume_mm3=float(mm.volume), holes=hole, solids=nc,
                             triangles=nt, degenerate=deg, nonmanifold=nonman))
    extra = "" if nc == 1 else "  ⛔散件×%d %s" % (nc, [f"{c.volume/1000:.2f}cm³@{np.round(c.centroid,0).tolist()}" for c in sorted(comps, key=lambda c: c.volume)[:3]])
    if deg or nonman: extra += f"  (退化面{deg} 非流形边{nonman}，需切片复核)"
    print(f"{name:32s} {str(np.round(b[1]-b[0],1)):22s} {v:6.1f}cm³ {v*1.27:4.0f}g 洞边={hole} 实体={nc}{extra}")
def vol(a, b):
    # AABB 只排除必不相交的对；布尔异常必须中止，不能伪装成负体积“通过”。
    if np.any(a.bounds[1] < b.bounds[0]) or np.any(b.bounds[1] < a.bounds[0]):
        return 0.0, None
    # hr42：检查原语缓存（tools/cad/check_cache.py）—— 键 = 两个网格的顶点/面原始字节 + 库版本；值 = 交集体积 + 交集网格数组（原样还原）。
    #   未命中时求交仍是原来那一句 trimesh.boolean.intersection（每次现构造 manifold 对象）：manifold 输出三角形的顺序取决于两个输入
    #   manifold 的构造先后（全局 originalID），跨调用复用 manifold 对象会改变交集网格三角形顺序 → xx/ STL 字节与体积末位都变（第四任实测），
    #   所以不复用。命中时还原的是同一份数组（process=False，数组本来就是 trimesh 处理过的），导出 STL / 体积 / 包围盒逐字节相同。
    #   DUCK_CHECK_CACHE=0 时不读不写，走原路径。
    import check_cache as CC
    k = CC.key("checks.vol", CC.mesh_digest(a), CC.mesh_digest(b)) if CC.enabled() else None
    if k is not None:
        hit, v = CC.get("vol", k)
        if hit:
            return (0.0, None) if v is None else (v[0], trimesh.Trimesh(vertices=v[1], faces=v[2], process=False))
    it = trimesh.boolean.intersection([a, b], engine="manifold")
    res = None if it is None or it.is_empty else (float(it.volume), np.asarray(it.vertices), np.asarray(it.faces))
    if k is not None:
        CC.put("vol", k, res)
    return (0.0, None) if res is None else (res[0], it)
def descendants(body):
    out = [body]
    for c in B[body]["children"]: out += descendants(c["name"])
    return out

# 旧 CAD 的检查域仅用于对比，不是软限位；控制端没有落实这些范围。
# 2026-09-16 行程恢复：踝 / 颈按新切刀域改（踝刀 lib.ANK_SWEEP_LO/HI = 反向 [−92, 62] → 关节 [−62, 92]；颈 trunk.NECK_SWEEP_LO/HI 直接 [−92.5, 62.5]）；
# 膝刀 legs.build_lower_leg 仍是反向 [−62, 92] → 关节 [−92, 62]（这里原写 −90..60，按刀改齐）。
LEGACY_LIMITS = {"left_hip_yaw": (-25, 30), "left_hip_roll": (-22, 22), "left_hip_pitch": (-60, 60),
                 "left_knee": (-92, 62), "left_ankle": (-62, 92), "neck_pitch": (-92.5, 62.5),
                 "head_pitch": (-45, 45), "head_yaw": (-90, 90), "head_roll": (-25, 25)}
LEGACY_LIMITS.update({k.replace("left_", "right_"): (-v[1], -v[0])
                      for k, v in list(LEGACY_LIMITS.items()) if k.startswith("left_")})
SWEEP = {B[n]["joint"]["name"]: tuple(float(v) for v in B[n]["joint"]["range_deg"])
         for n in ORDER if B[n]["joint"]}

def run_checks(allm, limits=None, export_intersections=False):
    """独立逐关节检查，其他关节保持零位；默认覆盖 MJCF 全范围、左右腿及舵机相互干涉。
    输出保留每个碰撞姿态，控制台汇总每个零件对的峰值。此检查不等于多关节组合验证。"""
    limits = SWEEP if limits is None else limits
    report = dict(step_deg=2.5, scope="one joint at a time; all other joints zero",
                  static=[], sweep=[], joints=[], excluded_pairs="original-original; H03/top original seam (reference Boolean checked in mechanics)")
    def cosmetic_pair(a, b):
        return (a.startswith("orig_") and b.startswith("orig_")) or \
               {a, b} == {"head_bottom_shell", "head_top_shell"}      # hr38：顶壳换成 H05 派生件，合缝面仍是原版面（mechanics 里有参考布尔）
    import check_cache as _CC
    def record(rows, a, b, ma, mb, joint=None, angle=None):
        _CC.set_context(f"run_checks:{a}×{b}")        # hr42：只给原语缓存的未命中日志用
        v, it = vol(ma, mb)
        if v <= 0.05: return
        row = dict(a=a, b=b, volume_mm3=v, bounds_mm=it.bounds.tolist())
        if joint is not None:
            lo, hi = LEGACY_LIMITS[joint]
            row.update(joint=joint, angle_deg=float(angle), in_legacy_domain=bool(lo-1e-6 <= angle <= hi+1e-6))
        rows.append(row)
        if export_intersections:
            stem = f"{joint}_{angle:+.4f}_" if joint else "zero_"
            it.export(os.path.join(OUT, "xx", stem+a+"__"+b+".stl"))
    print("\n[零位姿干涉检查，mm³]")
    for i, (_, a, ma) in enumerate(allm):
        for _, b, mb in allm[i+1:]:
            if cosmetic_pair(a, b): continue
            record(report["static"], a, b, ma, mb)
    for r in report["static"]: print(f"  {r['a']} × {r['b']}: {r['volume_mm3']:.3f} mm³")
    print("\n[关节扫掠：MJCF 全行程，2.5°，包括右腿；其他关节=0]")
    for n in ORDER:
        j = B[n]["joint"]
        if not j or j["name"] not in limits: continue
        name = j["name"]; lo, hi = limits[name]; sub = set(descendants(n))
        movers = [(nm, m) for b, nm, m in allm if b in sub]
        statics = [(nm, m) for b, nm, m in allm if b not in sub]
        angles = check_angles(lo, hi); rows = []
        for ang in angles:
            if abs(ang) < 1e-8: continue
            R = rot(math.radians(float(ang)), TW(n)[:3, 2], TW(n)[:3, 3])
            for nm, m in movers:
                mm = placed(m, R)
                for snm, sm in statics:
                    if cosmetic_pair(nm, snm): continue
                    record(rows, nm, snm, mm, sm, name, ang)
        report["joints"].append(dict(name=name, range_deg=[lo, hi], samples=len(angles),
                                     collision_samples=len({r["angle_deg"] for r in rows}),
                                     collision_records=len(rows)))
        report["sweep"].extend(rows)
        print(f"  {name} [{lo:.3f}, {hi:.3f}]°: {len(angles)} 姿态，{len(rows)} 个相交对/姿态", flush=True)
        peaks = {}
        for r in rows:
            key = (r["a"], r["b"])
            if key not in peaks or r["volume_mm3"] > peaks[key]["volume_mm3"]: peaks[key] = r
        for r in sorted(peaks.values(), key=lambda r: -r["volume_mm3"]):
            print(f"    {r['angle_deg']:+.3f}° {r['a']} × {r['b']}: {r['volume_mm3']:.3f} mm³")
    # hr38 嘴关节：MJCF robot_walk.xml 里下颚不是关节（kin 里 jaw_soft servos[0] drives=None），B[] 里没有 jaw body，
    #   所以不走上面的 ORDER 循环，单独扫：J01 + 6 颗法兰螺丝头绕 jaw.JAW_AXIS_P/JAW_AXIS_D 转 JAW_RANGE，其余件全静止。
    from . import jaw as JAW
    # hr41：随嘴转的是 J01 + J02（拧在法兰上）+ J03（轴颈盘）+ 9 颗螺丝头（jaw.JAW_MOVERS / jaw_screw_heads_world）
    jaw_meshes = [(nm, m) for _, nm, m in allm if nm in JAW.JAW_MOVERS]
    if jaw_meshes:
        jaw_meshes.append(("jaw_screws", JAW.jaw_screw_heads_world()))
        statics = [(nm, m) for _, nm, m in allm if nm not in JAW.JAW_MOVERS]
        lo, hi = JAW.JAW_RANGE
        angles = [a for a in np.arange(lo, hi + 1e-9, JAW.JAW_SWEEP_STEP) if abs(a) > 1e-8]
        rows = []
        for ang in angles:
            R = JAW.jaw_R(ang)
            for nm, m in jaw_meshes:
                mm = placed(m, R)
                for snm, sm in statics:
                    if cosmetic_pair(nm, snm): continue
                    _CC.set_context(f"run_checks:{nm}×{snm}")
                    v, it = vol(mm, sm)
                    if v <= 0.05: continue
                    rows.append(dict(a=nm, b=snm, volume_mm3=v, bounds_mm=it.bounds.tolist(), joint="jaw", angle_deg=float(ang), in_legacy_domain=True))
                    if export_intersections: it.export(os.path.join(OUT, "xx", f"jaw_{ang:+.4f}_{nm}__{snm}.stl"))
        report["joints"].append(dict(name="jaw", range_deg=[lo, hi], samples=len(angles),
                                     collision_samples=len({r["angle_deg"] for r in rows}), collision_records=len(rows)))
        report["sweep"].extend(rows)
        print(f"  jaw [{lo:.3f}, {hi:.3f}]°: {len(angles)} 姿态，{len(rows)} 个相交对/姿态", flush=True)
        for r in sorted(rows, key=lambda r: -r["volume_mm3"])[:10]:
            print(f"    {r['angle_deg']:+.3f}° {r['a']} × {r['b']}: {r['volume_mm3']:.3f} mm³")
    report["sampled_collision_free"] = not (report["static"] or report["sweep"])
    print(f"  → 静态 {len(report['static'])} 条；扫掠 {len(report['sweep'])} 个相交对/姿态；"
          f"旧检查域内 {sum(r['in_legacy_domain'] for r in report['sweep'])} 个。")
    return report
