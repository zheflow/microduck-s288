"""已知缺陷的独立实测回归。输入本次构建的世界坐标成品，避免重建/读取过期 STL。
测试角不复用 sweep_of 的采样；射线检查覆盖孔圆周，打空必须报错。
"""
import math
import numpy as np
from trimesh.transformations import rotation_matrix as rot
try:
    from duckstructure.head_bearing_rebuild import HEAD_A, N04_KEY as HBR_N04_KEY   # hr11：头部轴承补丁的设计常量（A 座 Ø27.15、N04 键/槽）
except ImportError:
    import sys as _sys, pathlib as _pl; _sys.path.insert(0, str(_pl.Path(__file__).resolve().parents[2]))
    from duckstructure.head_bearing_rebuild import HEAD_A, N04_KEY as HBR_N04_KEY

# hr42（2026-09-24，B 线）：射线 / contains 由"循环里逐条调"改成"先把一组射线（点）收齐 → 一次批量求交 → 按射线号拆回"，逐条的后处理公式、
#   循环顺序、比较（含并列时谁先）一个不动。批量求交走检查原语缓存 check_cache.ray_location / contains（DUCK_CHECK_CACHE=0 时直接算），
#   它们调 tools/cad/ray_backend.py：射线 = triangle-bulk（trimesh ray_triangle 同一算法，只把逐条 rtree 查询换成批量查询 → 与逐条调
#   m.ray.intersects_location 逐字节相同，拆回后每条射线的行与行序 = 单独调用时的行与行序），contains = Embree 找候选面 + trimesh 原公式判
#   （DUCK_RAY_VERIFY 双算逐点比对，见 hr42_work/profile/B/）。min_gap：_review_r2 的 gap() 里同一网格对象的 manifold 实体只转换一次
#   （MinGap = 候选三角形对精确距离的最小值，与实体何时构造无关）。结果 JSON 与改前逐字节相同（hr42_work/profile/B/mech_*.json）。
import sys as _hr42_sys, pathlib as _hr42_pl
if str(_hr42_pl.Path(__file__).resolve().parent) not in _hr42_sys.path:
    _hr42_sys.path.insert(0, str(_hr42_pl.Path(__file__).resolve().parent))
import check_cache as _CC
import ray_backend as _RB


def _rays(m, origins, directions, multiple_hits=True):
    """一组射线批量求交，按射线拆回：第 i 项 = 第 i 条射线的命中点 (k,3)，= 单独调 m.ray.intersects_location([o],[d],multiple_hits)[0]。"""
    O = np.asarray(origins, dtype=np.float64).reshape(-1, 3)
    Dd = np.asarray(directions, dtype=np.float64).reshape(-1, 3)
    if len(O) == 0:
        return []
    loc, ir, _it = _CC.ray_location(m, O, Dd, multiple_hits=multiple_hits, ns="mech")
    return [x for x, _ in _RB.split_by_ray(len(O), loc, ir)]


def _contains_split(m, groups):
    """几组点一次 contains，按组拆回（contains 逐点独立）。"""
    arrs = [np.asarray(g, dtype=np.float64).reshape(-1, 3) for g in groups]
    r = _CC.contains(m, np.concatenate(arrs), ns="mech") if arrs else np.zeros(0, bool)
    out, k = [], 0
    for a in arrs:
        out.append(r[k:k + len(a)]); k += len(a)
    return out


class _SolidCache:
    """manifold 实体按网格对象缓存（只认同一对象；最多留 n 个，先进先出）。"""
    def __init__(self, n=24):
        self.n, self.d = n, {}
    def __call__(self, m):
        from assembly_audit import solid
        e = self.d.get(id(m))
        if e is not None and e[0] is m:
            return e[1]
        if len(self.d) >= self.n:
            self.d.pop(next(iter(self.d)))
        e = (m, solid(m)); self.d[id(m)] = e
        return e[1]

_GAP_FAST = 1.0


def _gap_fast(sa, sb, lim):
    """hr42（B 线）：= float(sa.min_gap(sb, lim))。先用搜索半径 _GAP_FAST（1 mm）算：结果 g < 1 时就是答案 ——
    MinGap(L) = min(L, 与对方三角形的包围盒外扩 L 后相交的那些三角形对的精确距离)：真最小距离 d* < 1 ≤ L 时，取得 d* 的那对三角形
    两种半径下都在候选里（同一对、同一算式 → 同一个数），半径 L 多出来的候选对包围盒间距 > 1，距离必 > 1 > d* → 两种半径的最小值是同一个数。
    g ≥ 1（1 mm 内没有料）→ 按原 lim 重算（结果就是原来那个）。搜索半径小，manifold 的候选对少 3–15 倍（profile/B/mech_*）。
    DUCK_MECH_GAPFAST=0 关（直接原 lim）；DUCK_MECH_GAP_VERIFY=<json>：每次两种都算，逐位比对，汇总写该文件（返回仍是快路径的值）。"""
    import os
    if lim <= _GAP_FAST or os.environ.get("DUCK_MECH_GAPFAST", "1") in ("0", "off", "no", "false"):
        return float(sa.min_gap(sb, lim))
    g = float(sa.min_gap(sb, _GAP_FAST))
    fast = g < _GAP_FAST
    out = g if fast else float(sa.min_gap(sb, lim))
    vp = os.environ.get("DUCK_MECH_GAP_VERIFY")
    if vp:
        import json
        ref = float(sa.min_gap(sb, lim))
        st = _GAP_VER.setdefault("s", dict(calls=0, fast=0, mismatched=0, max_abs_diff=0.0, rows=[]))
        st["calls"] += 1; st["fast"] += int(fast)
        same = (ref == out) and (math.copysign(1.0, ref) == math.copysign(1.0, out))
        if not same:
            st["mismatched"] += 1; st["rows"].append(dict(lim=lim, fast=out, ref=ref))
        st["max_abs_diff"] = max(st["max_abs_diff"], abs(ref - out))
        with open(vp + ".tmp", "w") as f:
            json.dump(st, f, indent=1)
        os.replace(vp + ".tmp", vp)
    return out


_GAP_VER = {}


def _hip_yaw_bearing(D, ms, check, peak, V, servo_seats):
    """⑤ 髋偏航法兰侧 6702ZZ（原版此处本来就有一颗 22×16×4（舵盘面平面，受力报告 §4 判为推力式夹持），S288 法兰配不了那种夹法，我们改法兰侧径向 6702，见 docs/reports/受力分析与双支撑决策_2026-09-12.md §4）。只写不变量：
      ① T01 座孔 Ø seat_d 在 xf+SLIDE_CLR（xf=法兰面 13.0）以外的三个站位 24 方位整圈有料、直径 ±0.05、壁厚 ≥ (ring_od-seat_d)/2-0.1；
      ② 挡肩 Ø lip_d 只压外圈（open_side 侧滑扇区没料是设计，不算）；
      ③ 6702 本体与 L01 都能沿法兰方向直线装入（与 T01/两颗躯干舵机/对侧 L01·L02/壳/脖子/电池/仓门 交集 ≤0.05）；
      ④ L01 六颗法兰螺丝叠厚 legs.L01_STACK=4.0（M2×L01_SCREW_L=6 咬 2.0，1.8..3.0 内；09-14 前 5.35/M2×8）、头 Ø4×1.6 与起子 Ø4.2×30 包络 0，
         且头与髋横滚舵机本体交集 0 / min_gap ≥0.3（09-14 复审 BLOCKER：旧坐面 18.35 头顶 19.95 压进横滚舵机顶端 19.5 0.45，旧检查没查这一对）；
      ⑤ 座环与 L01 最小间隙 ≥0.3、与髋横滚 6704 ≥0.5；髋件 L02 绕横滚 ±22°（1.25°）不碰座环/轴承。
    反例：拿无座环的旧 T01/L01 跑，①② 必须报 FAIL（09-12 已验 8 条）。"""
    from assembly_audit import solid
    by = D.P['brg_yaw']; xf = D.S['T'] / 2 + D.S['flange_h']
    res = {}
    for tag, idx, l01, hip, brg6704, side in (('L', 0, 'yaw2roll', 'hip', 'bearing_left_hip_roll', 1),
                                                ('R', 1, 'yaw2roll_R', 'hip_R', 'bearing_right_hip_roll', -1)):
        R = D.sfw('trunk_base', idx); og, ex, ey, ez = R[:3, 3], R[:3, 0], R[:3, 1], R[:3, 2]
        def rays(xl, naz=24):
            first, wall = [], []
            od = []
            for k in range(naz):
                a = 2 * math.pi * k / naz; d = ey * math.cos(a) + ez * math.sin(a); o = og + ex * xl
                od.append((o, d))
            hs = _rays(ms['trunk'], [o for o, _ in od], [d for _, d in od], multiple_hits=True)   # hr42：naz 条一次批量
            for (o, d), hit in zip(od, hs):
                t = np.unique(np.round((hit - o) @ d, 4)) if len(hit) else np.array([]); t = t[t >= 0]   # 去掉同一面被射线命中两次的重复
                first.append(float(t[0]) if len(t) else None); wall.append(float(t[1] - t[0]) if len(t) > 1 else None)
            return first, wall
        r = dict(seat={}, lip=None)
        # was_until_2026_09_28_hr50: # hr13 复核：hr12 起 T01 底边的真实姿态区域刀（trunk__pair_trunk_base_x_upper_leg_*，髋横滚 19–26° + 偏航 11–30° 时 L03 俯仰座甩到躯干底边；
        # was_until_2026_09_28_hr50: # 38/78 命中姿态里 12/14 个在 frozen ±22° 内，刀不能撤）把 6702 座环最低端朝腿扇区（L 325°..30°，R 150°..215°）从 x 14.8 起削成 0..0.7 的皮 / 穿透。
        # was_until_2026_09_28_hr50: # hr13：trunk.hip_yaw_seat_notch 把该扇区（±HIP_YAW_SEAT_NOTCH_HALF_DEG）在 x ≥ HIP_YAW_SEAT_NOTCH_X0 干净截掉。新不变量：
        # was_until_2026_09_28_hr50: #   ① x 13.55 / 14.7（截止前 0.1）：24 向全壁 ≥ 1.62、孔径 21.15±0.1；② x 15.0 / 16.85：扇区外全壁，扇区内必须 None（干净截止，不许有皮）。
        # was_until_2026_09_28_hr50: from duckstructure.trunk import HIP_YAW_SEAT_NOTCH_X0, HIP_YAW_SEAT_NOTCH_HALF_DEG
        # was_until_2026_09_28_hr50: cosmin = math.cos(math.radians(HIP_YAW_SEAT_NOTCH_HALF_DEG)) - 1e-9
        # was_until_2026_09_28_hr50: notch = [k for k in range(24) if side * (ey * math.cos(2 * math.pi * k / 24) + ez * math.sin(2 * math.pi * k / 24))[1] >= cosmin]
        # was_until_2026_09_28_hr50: r['notch_azimuths_deg'] = [k * 15 for k in notch]
        # was_until_2026_09_28_hr50: wfull = (by['ring_od'] - by['seat_d']) / 2 - 0.1
        # was_until_2026_09_28_hr50: for xl, in_notch in ((xf + D.SLIDE_CLR + 0.15, 'full'), (HIP_YAW_SEAT_NOTCH_X0 - 0.1, 'full'), (xf + D.RING_T / 2, 'absent'), (xf + D.RING_T - 0.15, 'absent')):
        # was_until_2026_09_28_hr50: f, w = rays(xl); r['seat'][f'{xl:.2f}'] = dict(first=f, wall=w)
        # was_until_2026_09_28_hr50: ok_out = all(f[k] is not None and abs(2 * f[k] - by['seat_d']) <= 0.1 and w[k] is not None and w[k] >= wfull for k in range(24) if k not in notch)
        # was_until_2026_09_28_hr50: ok_in = all(f[k] is not None and abs(2 * f[k] - by['seat_d']) <= 0.1 and w[k] is not None and w[k] >= wfull for k in notch) if in_notch == 'full' \
        # was_until_2026_09_28_hr50: else all(f[k] is None for k in notch)
        # was_until_2026_09_28_hr50: check(f'hip_yaw_{tag} seat bore x{xl:.2f}', ok_out and ok_in)
        # hr50（2026-09-28）：trunk 撤了 hip_yaw_seat_notch（座环恢复 360° 全长）→ 本判据回到 hr10 口径：三个截面 24 向全壁、孔径 seat_d±0.1、壁 ≥ 环厚−0.1。
        for xl in (xf + D.SLIDE_CLR + 0.15, xf + D.RING_T / 2, xf + D.RING_T - 0.15):
            f, w = rays(xl); r['seat'][f'{xl:.2f}'] = dict(first=f, wall=w)
            check(f'hip_yaw_{tag} seat bore x{xl:.2f}',
                  all(v is not None and abs(2 * v - by['seat_d']) <= 0.1 for v in f) and all(v is not None and v >= (by['ring_od'] - by['seat_d']) / 2 - 0.1 for v in w))
        f, w = rays(xf - 0.25)
        keep = [k for k in range(24) if not (side * math.cos(2 * math.pi * k / 24) > 0 and abs(by['lip_d'] / 2 * math.sin(2 * math.pi * k / 24)) <= D.S['flange_d'] / 2 + 0.5)]
        r['lip'] = dict(first=f, azimuths_checked=keep)
        check(f'hip_yaw_{tag} seat lip', all(f[k] is not None and abs(2 * f[k] - by['lip_d']) <= 0.1 for k in keep))
        bname = 'bearing_' + ('left' if tag == 'L' else 'right') + '_hip_yaw'
        obst = [n for n in ms if n not in (l01, bname) and not n.startswith('orig_')
                and (n.startswith('servo_trunk_base') or n in ('trunk', 'zz_battery', 'battery_door', 'shell_L', 'shell_R', 'neck', 'neck_pitch', 'yrm')
                     or n == ('yaw2roll_R' if tag == 'L' else 'yaw2roll') or n == ('hip_R' if tag == 'L' else 'hip'))]
        for mv in (bname, l01):
            worst = (0.0, 0.0, None)
            for d in np.arange(0, 30.01, 0.5):
                m = ms[mv].copy(); m.apply_translation(ex * d)
                for n in obst:
                    v = V(m, ms[n])
                    if v > worst[0]: worst = (v, float(d), n)
            r[f'insert_{mv}'] = worst; check(f'hip_yaw_{tag} {mv} straight insertion', worst[0] <= .05)
        Rp = D.sfw('trunk_base', idx)
        seats = servo_seats(ms[l01], Rp, True); r['L01_stack_mm'] = seats
        st = D.L01_STACK   # 4.0：叠厚是设计常量（legs.py），沉窝深从它反推，不随法兰面 12.85→13.0 变（09-14 前 5.35）
        check(f'hip_yaw_{tag} L01 screw seats', all(v is not None and abs(v - st) < .02 and 1.8 <= D.L01_SCREW_L - v <= 3 for row in seats for v in row))
        heads = []; hvs = []; hgap = []
        roll_servo = ms['servo_yaw2roll_hip_l' if tag == 'L' else 'servo_bearing_roll_hip_l_2']   # 髋横滚舵机（顶端 = 髋偏航局部 x 19.5）
        for a in np.linspace(0, 2 * math.pi, D.S['horn_n'], endpoint=False):
            y, z = D.S['horn_r'] * math.cos(a), D.S['horn_r'] * math.sin(a)
            head = D.placed(D.cyl(4.0, 1.6, (xf + st + 0.8, y, z), axis='x'), Rp)
            heads.append(V(ms[l01], head))
            heads.append(V(ms[l01], D.placed(D.cyl(4.2, 30, (xf + st + 15.0, y, z), axis='x'), Rp)))
            hvs.append(V(head, roll_servo)); hgap.append(float(solid(head).min_gap(solid(roll_servo), 2.)))
        r['L01_head_tool_mm3'] = heads; check(f'hip_yaw_{tag} L01 head/tool envelope', max(heads) <= .05)
        r['L01_head_vs_roll_servo'] = dict(mm3=hvs, min_gap_mm=hgap)   # 09-14：头 vs 横滚舵机本体（旧 5.35 叠厚时每颗 5.646 mm³）
        check(f'hip_yaw_{tag} L01 heads vs roll servo', max(hvs) <= .05 and min(hgap) >= .2999)
        zone = D.inter(ms['trunk'], D.placed(D.cyl(40, 6.0, (xf + 1.75, 0, 0), axis='x'), R))
        r['gap_ring_L01_mm'] = float(solid(zone).min_gap(solid(ms[l01]), 2.)); check(f'hip_yaw_{tag} ring/L01 gap', r['gap_ring_L01_mm'] >= .3 - 1e-3)
        if D.RINGS['left_hip_roll']:   # 09-17：髋横滚 6704 取消后无此件
            r['gap_ring_6704_mm'] = float(solid(zone).min_gap(solid(ms[brg6704]), 2.)); check(f'hip_yaw_{tag} ring/6704 gap', r['gap_ring_6704_mm'] >= .5 - 1e-3)
        body = 'hip_l' if tag == 'L' else 'hip_l_2'
        r['hip_roll_vs_ring'] = peak(ms[hip], zone, body, np.arange(-22, 22.01, 1.25)); r['hip_roll_vs_bearing'] = peak(ms[hip], ms[bname], body, np.arange(-22, 22.01, 1.25))
        check(f'hip_yaw_{tag} hip roll vs ring', r['hip_roll_vs_ring']['volume_mm3'] <= .05 and r['hip_roll_vs_bearing']['volume_mm3'] <= .05)
        res[tag] = r
    return res

def _review_r1(D, ms, check, V):
    """⑥ 2026-09-14 复审（手册尺寸 build #8 之后）8 条 BLOCKER/MAJOR 的不变量，全部对**做出来的**件量，阈值 = 复审判据：
      a. H02 压板 vs 头横滚舵机背面惰轮盘（Ø14×3 精确盘）min_gap ≥0.3（修前 0.20）；
      b. T02/T03 vs 同侧髋偏航舵机背面惰轮盘 交集 0 且 min_gap ≥0.3（修前 T02 0.019 mm³ / 0.0；run_checks 只记 >0.05 mm³ 是已知盲区）；
      c. 头横滚舵机本体 vs 冻结的原版上头壳 min_gap ≥0.3（修前 0.199）；
      d. T01 轭 F12 6 颗头 Ø4×1.6（坐面 = 轭盘外表面）vs T02：零位交集 0、T02 沿 +y 退 0..40 全程 0、头顶以外 Ø4.2×30 起子通道 0（修前 10.7..16.0 mm³/颗）；
         T02 在 Ø26 轭足印内（z ≥ trunk.YOKE_Z0=144.0，其下躯干本来无料）、轭盘外表面外 0.5 之内不许有料（DILATE6 让位在孔里留的"销"，修前 27 mm³ 含 z<144 壳壁 6.4）；
      e. H01 右前脚 (49.1, 9.35) 孔壁到任何削面/外缘的最薄壁 ≥1.2（72 方位 × 6 高度径向射线；修前 1.02）。
    L01 F02 头 vs 髋横滚舵机在 _hip_yaw_bearing 里（同一颗舵机的局部系更顺手）。"""
    from assembly_audit import solid
    S=D.S; xf=S['T']/2+S['flange_h']; res={}
    # a. H02 vs 头横滚惰轮盘
    Rr=D.drv_self('jaw_soft')
    idler=D.placed(D.cyl(S['rear_boss_d'],S['rear_boss_h'],(-S['T']/2-S['rear_boss_h']/2,0,0),axis='x',sections=128),Rr)
    res['H02_vs_head_roll_idler']=dict(mm3=V(ms['head_clamp'],idler),min_gap_mm=float(solid(ms['head_clamp']).min_gap(solid(idler),2.)))
    check('H02 clear of head roll idler',res['H02_vs_head_roll_idler']['mm3']<=1e-6 and res['H02_vs_head_roll_idler']['min_gap_mm']>=.2999)
    # b. 壳 vs 髋偏航惰轮盘
    res['shell_vs_hip_yaw_idler']={}
    for name,idx in (('shell_L',0),('shell_R',1)):
        Rh=D.sfw('trunk_base',idx)
        boss=D.placed(D.cyl(S['rear_boss_d'],S['rear_boss_h'],(-S['T']/2-S['rear_boss_h']/2,0,0),axis='x',sections=128),Rh)
        g=float(solid(ms[name]).min_gap(solid(boss),2.)); v=V(ms[name],boss)
        res['shell_vs_hip_yaw_idler'][name]=dict(mm3=v,min_gap_mm=g)
        check(name+' clear of hip yaw idler',v<=1e-6 and g>=.2999)
    # c. 头横滚舵机 vs 上头壳（hr38：原版 orig_top_head_shell → H05 head_top_shell 派生件；外形/合缝面与原版逐面相同）
    res['head_roll_servo_vs_top_shell_mm']=float(solid(ms['servo_jaw_soft_jaw_soft']).min_gap(solid(ms['head_top_shell']),2.))
    check('head roll servo vs top head shell',res['head_roll_servo_vs_top_shell_mm']>=.2999)
    # d. F12 头 vs T02
    Rn=D.drv_self('neck'); _,_,xe,_=D.driven(ring=D.RINGS['neck_pitch'],d=26.0)
    ang=np.linspace(0,2*math.pi,S['horn_n'],endpoint=False)
    heads=[D.placed(D.cyl(4.0,1.6,(xe+0.8,S['horn_r']*math.cos(a),S['horn_r']*math.sin(a)),axis='x'),Rn) for a in ang]
    tools=[D.placed(D.cyl(4.2,30.,(xe+1.6+15.,S['horn_r']*math.cos(a),S['horn_r']*math.sin(a)),axis='x'),Rn) for a in ang]
    fp=D.inter(D.placed(D.cyl(26.0,0.5,(xe+0.25,0,0),axis='x'),Rn),D.wbox((0,0,D.YOKE_Z0+.05),(60,40,200)))   # 只看轭带 z≥YOKE_Z0：其下躯干本来无料，壳壁不是销
    slide=0.
    for d in np.arange(0,40.+1e-6,.5):
        m=ms['shell_L'].copy(); m.apply_translation((0,d,0)); slide=max(slide,max(V(m,h) for h in heads))
    res['F12_heads_vs_shell_L']=dict(static_mm3=[V(ms['shell_L'],h) for h in heads],tool_mm3=[V(ms['shell_L'],t) for t in tools],
                                     slide_peak_mm3=slide,footprint_pins_mm3=V(ms['shell_L'],fp))
    f=res['F12_heads_vs_shell_L']
    check('F12 heads clear of shell_L',max(f['static_mm3'])<=.05 and max(f['tool_mm3'])<=.05 and f['slide_peak_mm3']<=.05)
    check('shell_L no pins in yoke holes',f['footprint_pins_mm3']<=.05)
    # e. H01 右前脚承压圈最薄壁
    hx,hy=D.HEAD_FEET[3]; worst=(99.,None)
    zbou=[]
    for z in np.linspace(231.6,233.15,6):
        for b in np.linspace(0,2*math.pi,72,endpoint=False):
            u=np.array([math.cos(b),math.sin(b),0.]); o=np.array([hx,hy,z])+u*(1.35+1e-3)
            zbou.append((z,b,o,u))
    hs=_rays(ms['head_bracket'],[q[2] for q in zbou],[q[3] for q in zbou],multiple_hits=True)   # hr42：432 条一次批量
    for (z,b,o,u),hit in zip(zbou,hs):
            if len(hit):
                t=float(np.min((hit-o)@u))
                if t<worst[0]: worst=(t,(round(float(z),2),round(math.degrees(b))))
    res['H01_foot4_min_wall']=dict(mm=worst[0],at=worst[1])
    check('H01 foot (49.1,9.35) rim wall >= 1.2',worst[0]>=1.2)
    return res

def _review_r2(D, ms, check, V):
    """⑦ 2026-09-14 复审 r2（r1 之后）7 条 MAJOR 的不变量，全部对**做出来的**件量，阈值 = 复审判据；角度网格故意与切刀采样错开半格：
      M1 T02 轭区：原版舵盘 lobe 整块开掉（trunk.SHL_YOKE_*）——y 截面 16.55/17.0/17.6/17.85：开口区 (r≤SHL_YOKE_OPEN_R 且 x≥SHL_YOKE_OPEN_X0) 无料；
         开口外圈带 r..r+1.2（θ −80..115°，+x/上/下三面）无料（修前 6 孔间 web 0.65、孔边到 lobe 边 0.86、中心岛 1.00）；开口 −x 直边以左第一段料 ≥1.2 宽（z 144.5..152.4）；
      M2 膝：min_gap(L04, L03) ≥ 0.3，零位 + 膝 −90..60（2.5°，半格偏移）（修前 0.014 全程）；
      M3/M4 踝：min_gap(L07, L04) ≥ 0.3、min_gap(L05, L04) ≥ 0.3，零位 + 踝 ±35（2.5°，半格偏移）（修前 0.0 / 0.0013）；
      M5 头横滚 A 端：H03 下半座 x=8 直径 = seat_d ±0.02（16 向）；min_gap(N03, H03) ≥ 0.25，零位 + head_roll ±25（1°）（修前 Ø21.79 / 0.085）；
      M6 电池：L04 vs 电池在仓内 8 个极限位置 (±BAT_CLR)³，髋偏航 −25..30（1.25°，半格偏移）交集 ≤0.05（修前 3.77 mm³@−25°）且 min_gap ≥ 0.3（09-14 r2 复量补：
         仓腔+DILATE6 版在『后口×外壁×仓底』角点接触 0.000）；名义电池 min_gap ≥ 0.3（全程）；右腿镜像同查；
      M7 髋横滚：L01 挡肩带 (x xf−2..xf, r≤12.5) ↔ 6704 内圈探针 (r bore/2..lip_d/2−0.25, x 13..17) 与 L02 毂 min_gap ≥ 0.5（修前 0.10/0.095）；min_gap(L01, L02) ≥ 0.3 零位 + ±22°（1°）；
         挡肩本身还在：x=xf−0.15、r=seat_d/2−0.3（ring_boss 凸包内腔 Ø26 以外、座孔以内的 0.575 环带中）上 ≥50% 方位有料（6704 外圈端面仍被挡；缺的方位 = 髋件扫掠缺口 + 侧滑口）。"""
    from assembly_audit import solid
    S=D.S; xf=S['T']/2+S['flange_h']; res={}
    _sc=_SolidCache()                                  # hr42：同一网格对象只转一次 manifold（静态件 / M6 同一姿态的 L04 反复用）
    def gap(a,b,lim=5.): return _gap_fast(_sc(a),_sc(b),lim)
    def rotd(m,body,a):
        T=D.TW(body); return D.placed(m,rot(math.radians(float(a)),T[:3,2],T[:3,3]))
    def gap_sweep(mover,body,angles,other):
        best=(9.,None)
        for a in angles:
            g=gap(rotd(ms[mover],body,a),ms[other])
            if g<best[0]: best=(g,float(a))
        return dict(min_gap_mm=best[0],angle_deg=best[1])
    # M1
    Rn=D.drv_self('neck'); cn=D.pt(Rn,0); R=D.SHL_YOKE_OPEN_R; xl=D.SHL_YOKE_OPEN_X0; sh=ms['shell_L']
    m1={}
    for y in (16.55,17.0,17.6,17.85):
        g1=np.array([[cn[0]+r*math.cos(t),y,cn[2]+r*math.sin(t)] for r in np.linspace(.2,R-.05,30) for t in np.linspace(0,2*math.pi,120,endpoint=False)]); g1=g1[g1[:,0]>=xl]
        g2=np.array([[cn[0]+r*math.cos(t),y,cn[2]+r*math.sin(t)] for r in np.linspace(R+.02,R+1.2,12) for t in np.radians(np.arange(-80,115.1,2.5))])
        widths=[]
        zs_=list(np.arange(144.5,152.41,.5))
        cs=_contains_split(sh,[g1,g2]+[np.array([[x,y,z] for x in np.arange(xl-.02,xl-8.,-.02)]) for z in zs_])   # hr42：本 y 站位全部点一次 contains
        for ii in cs[2:]:
            if ii.any():
                s0=int(np.argmax(ii)); e=s0
                while e<len(ii) and ii[e]: e+=1
                widths.append(float((e-s0)*.02))
        m1[str(y)]=dict(core_fill=float(cs[0].mean()),band_fill=float(cs[1].mean()),left_min_width_mm=min(widths) if widths else None)
    res['T02_yoke_opening']=m1
    check('T02 yoke lobe removed (no web/arc band)',all(v['core_fill']<=1e-6 and v['band_fill']<=1e-6 for v in m1.values()))
    check('T02 yoke opening left rim >= 1.2',all(v['left_min_width_mm'] is not None and v['left_min_width_mm']>=1.2 for v in m1.values()))
    # M2
    res['knee_L04_vs_L03']=dict(zero_mm=gap(ms['lower_leg'],ms['upper_leg']),**gap_sweep('lower_leg','leg',np.arange(-88.75,60.,2.5),'upper_leg'))
    check('knee L04 vs L03 ring boss clearance',res['knee_L04_vs_L03']['zero_mm']>=.2999 and res['knee_L04_vs_L03']['min_gap_mm']>=.2999)
    # M3/M4
    for nm,tag in (('ankle_rear_arm','L07'),('ankle_foot','L05')):
        res[f'ankle_{tag}_vs_L04']=dict(zero_mm=gap(ms[nm],ms['lower_leg']),**gap_sweep(nm,'ankle_left',np.arange(-33.75,35.,2.5),'lower_leg'))
        check(f'ankle {tag} vs L04 clearance',res[f'ankle_{tag}_vs_L04']['zero_mm']>=.2999 and res[f'ankle_{tag}_vs_L04']['min_gap_mm']>=.2999)
    # M5
    AXZ=D.YRM_AXZ; vals=[]; od=[]
    for a in np.linspace(0,2*math.pi,16,endpoint=False):
        s=math.sin(a)
        if s>-1e-6: continue
        o=np.array([8.,0.,AXZ]); d=np.array([0.,math.cos(a),s]); od.append((o,d))
    for (o,d),hit in zip(od,_rays(ms['head_bottom_shell'],[o for o,_ in od],[d for _,d in od])):   # hr42：批量
        vals.append(float(np.min((hit-o)@d))*2 if len(hit) else None)
    res['H03_seatA_lower_d']=vals
    check('H03 seat A d',all(v is not None and abs(v-HEAD_A['seat_d'])<.02 for v in vals))   # hr11：A 端 6704 座 Ø27.15（HBR.HEAD_A），不再是 Ø22.18 塑料滑动座
    res['N03_vs_H03']=dict(zero_mm=gap(ms['yrm'],ms['head_bottom_shell']),**gap_sweep('yrm','jaw_soft',np.arange(-25,25.01,1.),'head_bottom_shell'))
    check('N03 vs H03 seat clearance',res['N03_vs_H03']['zero_mm']>=.2499 and res['N03_vs_H03']['min_gap_mm']>=.2499)
    # M6
    bat=ms['zz_battery']; res['L04_vs_battery_in_bay']={}
    for tag,l04,body,angles in (('L','lower_leg','yaw2roll',np.arange(-25.,30.,1.25)+.625),('R','lower_leg_R','bearing_roll',np.arange(-30.,25.,1.25)+.625)):
        worst=(0.,None,None); gmin=(9.,None); gsh=(9.,None,None)   # gsh：8 个仓内极限位置的最小 min_gap（09-14 r2 复量补：DILATE6 版在仓角点接触 0.000）
        for a in angles:
            m=rotd(ms[l04],body,a); g=gap(m,bat)
            if g<gmin[0]: gmin=(g,float(a))
            for sx in (-1,1):
                for sy in (-1,1):
                    for sz in (-1,1):
                        # hr48（2026-09-28）：电池实测 20 厚、B01 门开槽后仓腔 x 20.8：三轴可动量不再都是 BAT_CLR —— 取 lib.BAT_SHIFT=(0.4, 0.6, 0.55)（x 门槽底..前壁、y 导轨、z 仓高）
                        sh=getattr(D,'BAT_SHIFT',(D.BAT_CLR,D.BAT_CLR,D.BAT_CLR))
                        b=bat.copy(); b.apply_translation((sx*sh[0],sy*sh[1],sz*sh[2])); v=V(m,b)
                        if v>worst[0]: worst=(v,float(a),(sx,sy,sz))
                        if v<=0:
                            gs=gap(m,b,1.)
                            if gs<gsh[0]: gsh=(gs,float(a),(sx,sy,sz))
        res['L04_vs_battery_in_bay'][tag]=dict(shifted_peak_mm3=worst[0],at_deg=worst[1],shift=worst[2],nominal_min_gap_mm=gmin[0],nominal_at_deg=gmin[1],
                                              shifted_min_gap_mm=gsh[0],shifted_min_gap_at_deg=gsh[1],shifted_min_gap_shift=gsh[2])
        check(f'L04{tag} vs battery anywhere in bay',worst[0]<=.05 and gmin[0]>=.2999 and gsh[0]>=.2999)
    # M7（09-17：髋横滚 6704 取消后只保留 L01↔L02 间隙这一条）
    Rs=D.sfw('yaw2roll',0); b=D.P['brg']
    if not D.RINGS['left_hip_roll']:
        r7=dict(L01_vs_L02_zero_mm=gap(ms['yaw2roll'],ms['hip']),**{'L01_vs_L02_'+k:v for k,v in gap_sweep('hip','hip_l',np.arange(-22,22.01,1.),'yaw2roll').items()},
                shoulder_band_mm3=0.,shoulder_vs_inner_ring_mm=9.,shoulder_vs_L02_mm=9.,shoulder_present_frac=1.)
        res['hip_roll_L01_shoulder']=r7
        check('L01 vs L02 clearance',r7['L01_vs_L02_zero_mm']>=.2999 and r7['L01_vs_L02_min_gap_mm']>=.2999)
        return res
    # hr08：6703 外圈座是 L01 整体件（挡肩 shoulder_x0..x0）。
    from duckstructure.bearing_rebuild import HR
    seat=ms['yaw2roll']; xs=(HR['shoulder_x0']+HR['x0'])/2
    zone=D.inter(seat,D.placed(D.cyl(HR['seat_od']+1.,HR['x0']-HR['shoulder_x0'],
                 (xs,0,0),axis='x',sections=128),Rs))
    inner=ms['hip_roll_sleeve']
    r7=dict(shoulder_band_mm3=float(zone.volume) if len(zone.faces) else 0.,
            shoulder_vs_inner_ring_mm=gap(zone,inner) if len(zone.faces) else 0.,
            shoulder_vs_L02_mm=gap(zone,ms['hip']) if len(zone.faces) else 0.,
            L01_vs_L02_zero_mm=gap(ms['yaw2roll'],ms['hip']),
            **{'L01_vs_L02_'+k:v for k,v in gap_sweep('hip','hip_l',np.arange(-22,22.01,1.),'yaw2roll').items()})
    rs=(HR['lip_d']+HR['od'])/4
    pts=np.array([D.pt(Rs,xs,rs*math.cos(a),rs*math.sin(a))
                  for a in np.deg2rad(np.arange(2.5,360.,5.))])
    r7['shoulder_present_frac']=float(_contains_split(seat,[pts])[0].mean())
    res['hip_roll_L01_shoulder']=r7
    check('L01 seat shoulder clear of L10 rotating sleeve / L02',
          r7['shoulder_vs_inner_ring_mm']>=.4999 and r7['shoulder_vs_L02_mm']>=.4999)
    check('L01 vs L02 clearance',r7['L01_vs_L02_zero_mm']>=.2999 and r7['L01_vs_L02_min_gap_mm']>=.2999)
    check('L01 rear shoulder retains 300 degree 6703 seat',r7['shoulder_present_frac']>=.80)
    return res

def _review_r3(D, ms, check, V):
    """⑧ 2026-09-14 复审 r3（r2 之后）1 BLOCKER + 4 MAJOR 的不变量，全部对**做出来的**件量，阈值 = 复审判据：
      a. H02 连接臂两颗 M2×10 过孔贯通：沿孔心线及 ±0.6 偏置（孔 r 1.2）的 x 射线，臂内端 H02_ARM_X[1] ±0.6 内无实体、整段臂 H02_ARM_X[0]−0.5..[1]+0.6 内无实体
         （修前刀只到 −12.5、臂端 −12.0，留 0.5 膜）；
      b. N03 A 端起子刀 vs H01/H02 min_gap ≥0.15 → 在 verify_mechanics 的 N03 tool corridors 里（同一组刀）；
      c. L01 偏航端毂 6 个 Ø2.4 过孔外缘到毂外圆（径向射线 孔心→外，x 13.3..16.8 八站位）≥0.9 —— **豁免** <1.2 规则：毂 OD 受 6702 内径 15 定死、Ø2.4 用户定案、
         分度圆 5.25 手册，毂整段就是内圈区无处加厚；韧带非承力（螺丝预紧后过孔壁不受拉，径向力走毂外圆→内圈，受压）。名义 0.975，阈值 0.9 = 0.975 − 0.075 隙配裕量（lib.P brg_yaw 注释）；
      d. T03 颈部区皮层：世界 y 射线 0.5 网格（x 10..36、z 138..162、离颈俯仰轴 r>7.6），凡内壁比本体扫掠面 XR−PLT−NECK_CLR=−17.1 更深（螺丝头扫掠带）且外表面在原版
         −18.40 平面上的点皮层 ≥1.0（−0.05 离散；修前 0.70）。外表面已离开平面的点 = 壳开口沿/底边圆角，削面从圆角里穿出的"羽化"是复审 NOTE（不在本轮），只记不判；N01 6 颗角螺丝头 vs T03 零位 + 颈俯仰 −60..45（1.25°）min_gap ≥0.3（修前 0.8，NECK_HEAD_CLR 0.5 后名义 0.5）；
      e. L07 踝后臂定位榫槽壁 ↔ M2 过孔壁（踝舵机局部 y 射线 x −14.8/−15.5、z 18）≥1.0（修前 0.60，bolt_centers ±3→±3.4），左右件同查。"""
    from assembly_audit import solid
    S=D.S; res={}
    def hits_many(m,od):
        """hr42：一组 (o,d) 批量求交；第 i 项 = 原 hits(m,o_i,d_i)（命中距离四舍五入 4 位去重升序）。"""
        hs=_rays(m,[o for o,_ in od],[d for _,d in od],multiple_hits=True)
        return [sorted(set(np.round((h-o)@d,4))) if len(h) else [] for (o,d),h in zip(od,hs)]
    # a. H02 臂孔贯通
    ax0,ax1=D.H02_ARM_X; worst=0.; band=0.
    od=[(np.array([-30.,y+dy,z+dz]),np.array([1.,0,0])) for (y,z) in D.HEAD_CLAMP_MOUNTS for dy,dz in ((0,0),(.6,0),(-.6,0),(0,.6),(0,-.6))]
    for ts_ in hits_many(ms['head_clamp'],od):                     # hr42：10 条一次批量，顺序同原双循环
            xs=[-30.+t for t in ts_]
            segs=[(xs[i],xs[i+1]) for i in range(0,len(xs)-1,2)]
            band=max(band,sum(max(0.,min(b,ax1+.6)-max(a,ax1-.6)) for a,b in segs))
            worst=max(worst,sum(max(0.,min(b,ax1+.6)-max(a,ax0-.5)) for a,b in segs))
    res['H02_arm_holes']=dict(solid_at_arm_end_mm=band,solid_in_hole_mm=worst)
    check('H02 arm holes through (no membrane at arm end)',band<=1e-3 and worst<=1e-3)
    # c. L01 毂孔圈壁（豁免阈值 0.9）
    res['L01_hub_ligament']={}
    for tag,nm,idx in (('L','yaw2roll',0),('R','yaw2roll_R',1)):
        Rp=D.sfw('trunk_base',idx); og,ex,ey,ez=Rp[:3,3],Rp[:3,0],Rp[:3,1],Rp[:3,2]; ligs=[]
        od=[]
        for xl in np.linspace(13.3,16.8,8):
            for a in np.linspace(0,2*math.pi,S['horn_n'],endpoint=False):
                u=ey*math.cos(a)+ez*math.sin(a); od.append((og+ex*xl+u*S['horn_r'],u))
        for ts in hits_many(ms[nm],od):                            # hr42：8×horn_n 条一次批量
                ligs.append(float(ts[1]-ts[0]) if len(ts)>=2 else None)
        res['L01_hub_ligament'][tag]=dict(min_mm=min(v for v in ligs if v is not None) if any(v is not None for v in ligs) else None,n_missing=sum(v is None for v in ligs),
                                          nominal_mm=D.P['brg_yaw']['hub_d']/2-(S['horn_r']+S['horn_hole_d']/2))
        r=res['L01_hub_ligament'][tag]
        check(f'L01_{tag} hub hole ligament >= 0.9 (waiver, nominal 0.975)',r['n_missing']==0 and r['min_mm']>=.9)
    # d. T03 颈部区皮层 + 头↔壳
    Rn=D.drv_self('neck'); cn=D.pt(Rn,0); sh=ms['shell_R']; xs_body=D.XR-D.PLT-D.NECK_CLR; pts=[]
    xz=[(x,z) for x in np.arange(10.,36.01,.5) for z in np.arange(138.,162.01,.5) if not math.hypot(x-cn[0],z-cn[2])<=7.6]
    for (x,z),ts in zip(xz,hits_many(sh,[(np.array([x,-40.,z]),np.array([0,1.,0])) for x,z in xz])):   # hr42：整张 0.5 网格一次批量
            if len(ts)>=2 and -40.+ts[0]<-16.: pts.append((float(x),float(z),-40.+float(ts[0]),-40.+float(ts[1])))
    pts=[p for p in pts if p[2]>-20.]                              # 只看颈部平面区（外表面 −18.4 附近）；y0<−20 是壳外侧弧面的擦射，不是这层皮
    hb=[(x,z,y0,y1) for x,z,y0,y1 in pts if y1<xs_body-.1]       # 内壁比 −17.1 更深 = 螺丝头扫掠带
    plane=[(x,z,y0,y1) for x,z,y0,y1 in hb if y0<=-18.38]          # 外表面在原版 −18.40 平面上的点（面的主体；r3 第 1 次 build：−18.35 分界时 (22.5,142.5) 已在圆角起点、皮层 0.953）
    rim=[(x,z,y0,y1) for x,z,y0,y1 in hb if y0>-18.38]             # 外表面已离开平面 = 壳开口沿/底边的圆角，削面从圆角里穿出（"羽化"，复审 NOTE，不在本轮 5 条内）
    thp=[y1-y0 for _,_,y0,y1 in plane]; thr=[y1-y0 for _,_,y0,y1 in rim]
    res['T03_neck_skin']=dict(n_head_band=len(hb),plane_n=len(plane),plane_min_mm=min(thp) if thp else None,plane_at=(plane[int(np.argmin(thp))][:2] if thp else None),
                              rim_n=len(rim),rim_min_mm=min(thr) if thr else None,rim_n_below_1=int(sum(1 for v in thr if v<1.)),
                              n_all=len(pts),n_all_below_1p2=int(sum(1 for _,_,y0,y1 in pts if y1-y0<1.2)),n_all_below_0p8=int(sum(1 for _,_,y0,y1 in pts if y1-y0<.8)))
    check('T03 neck-region skin >= 1.0 under screw-head sweep (plane region)',thp and min(thp)>=.95 and len(plane)>=50)
    from duckstructure.neck import neck_screw_heads
    Tn=D.TW('neck'); an,on=Tn[:3,2],Tn[:3,3]; heads=neck_screw_heads(); shs=solid(sh); best=(9.,None,None)
    z0=min(float(solid(h).min_gap(shs,3.)) for h in heads)
    for a1 in np.arange(-60,45.001,1.25):
        R1=rot(math.radians(float(a1)),an,on)
        for k,h in enumerate(heads):
            g=float(solid(D.placed(h,R1)).min_gap(shs,3.))
            if g<best[0]: best=(g,float(a1),k)
    res['N01_heads_vs_T03']=dict(zero_mm=z0,min_gap_mm=best[0],angle_deg=best[1],head=best[2])
    check('N01 screw heads vs T03 sweep clearance',z0>=.2999 and best[0]>=.2999)
    # e. L07 榫槽壁 ↔ 过孔壁
    res['L07_key_slot_to_hole']={}
    for tag,nm,body in (('L','ankle_rear_arm','leg'),('R','ankle_rear_arm_R','leg_2')):
        Ra=D.sfw(body,0); og,ex,ey,ez=Ra[:3,3],Ra[:3,0],Ra[:3,1],Ra[:3,2]; walls=[]
        for ts_ in hits_many(ms[nm],[(og+ex*xl+ez*18.-ey*10.,ey) for xl in (-14.8,-15.5)]):   # hr42：2 条一次批量
            ys=[-10.+t for t in ts_]
            pos=sorted(v for v in ys if v>0); neg=sorted(-v for v in ys if v<0)
            walls.append(float(pos[1]-pos[0]) if len(pos)>=2 else None); walls.append(float(neg[1]-neg[0]) if len(neg)>=2 else None)
        res['L07_key_slot_to_hole'][tag]=dict(walls_mm=walls,min_mm=min(v for v in walls if v is not None) if any(v is not None for v in walls) else None)
        check(f'L07_{tag} key slot to bolt hole wall >= 1.0',all(v is not None for v in walls) and min(walls)>=.999)
    return res

def _review_r4(D, ms, check, V):
    """⑨ 2026-09-15 壳柱螺丝坐面锪平（T02/T03 各 2 颗 F20）的不变量，对**做出来的**壳量：
      头足印 r 1.25/1.6/2.0 × 36 方位从上打 −z 射线，顶面 z 落差（平面度）≤0.02（修前弧面：Ø5 足印落差 0.28 / 0.97）；
      顶面到内表面剩余壁 ≥ lib.SHELL_SEAT_MIN_WALL=1.2（修后实测 1.37..2.18）；平台 z 与 lib.shell_seat_z 目标一致（±0.02）。"""
    res={}
    for side,name in ((1,'shell_L'),(-1,'shell_R')):
        sh=ms[name]
        for (x,y) in D.SHELL_BOSS:
            yy=side*y; zp,_=D.shell_seat_z(side,x,yy); top=[]; wall=[]
            os_=[np.array([x+r*math.cos(a),yy+r*math.sin(a),D.BZR1+1.0]) for r in (1.25,1.6,2.0) for a in np.linspace(0,2*math.pi,36,endpoint=False)]
            for h in _rays(sh,os_,[[0,0,1.0]]*len(os_),multiple_hits=True):   # hr42：108 条一次批量，顺序同原双循环
                    zs=np.sort(h[:,2]) if len(h) else []
                    if len(zs)>=2: top.append(float(zs[-1])); wall.append(float(zs[-1]-zs[0]))
            key=f'{name}({x},{yy})'; flat=(max(top)-min(top)) if top else None
            res[key]=dict(n=len(top),flatness_mm=flat,wall_min_mm=min(wall) if wall else None,plat_z=min(top) if top else None,plat_target=zp)
            check(f'{key} shell screw seat flat <=0.02 & wall >=1.2 & at target',
                  len(top)==108 and flat<=.02 and min(wall)>=D.SHELL_SEAT_MIN_WALL-1e-6 and abs(min(top)-zp)<=.02)
    return res

def verify_mechanics(D, allm):
    ms = {name: m for _, name, m in allm}
    out = {}; failures = []
    _nm = {id(m): name for name, m in ms.items()}             # hr42：未命中日志里写件名（同一对象才认得出；搬过位姿的写 moved）
    def V(a,b):
        _CC.set_context(f"mech:{_nm.get(id(a), 'moved')}×{_nm.get(id(b), 'moved')}")
        return max(0.0,D.vol(a,b)[0])
    def peak(m, other, body, angles):
        T=D.TW(body); best=(0.0,0.0)
        for a in angles:
            v=V(D.placed(m,rot(math.radians(float(a)),T[:3,2],T[:3,3])),other)
            if v>best[0]: best=(v,float(a))
        return dict(volume_mm3=best[0],angle_deg=best[1])
    def check(name, condition):
        if not condition: failures.append(name)
    out['neck_trunk']=peak(ms['neck'],ms['trunk'],'neck',np.arange(-60,45.01,1.25))
    out['neck_hip_servo']=peak(ms['neck'],ms['servo_trunk_base_bearing_roll'],'neck',np.arange(-60,45.01,1.25))
    out['head_roll_1deg']=peak(ms['head_bracket'],ms['yrm'],'jaw_soft',np.arange(-25,25.01,1))
    out['head_roll_cut_midpoints']=peak(ms['head_bracket'],ms['yrm'],'jaw_soft',np.arange(-24.375,25,1.25))
    out['N02_body_clearance_0.25mm']=peak(ms['neck_pitch'],D.placed(D.s288.body_prism(c=.25),D.drv_from('neck_pitch')),'neck_pitch',np.arange(-45,45.01,1.25))
    out['L02_opposite_L01']=peak(ms['hip'],ms['yaw2roll_R'],'yaw2roll',np.arange(-25,30.01,1.25))
    out['L04_battery']=peak(ms['lower_leg'],ms['zz_battery'],'yaw2roll',np.arange(-25,30.01,1.25))
    out['knee_inverse_range']=peak(ms['lower_leg'],ms['upper_leg'],'leg',np.arange(-90,60.01,1.25))
    for k,r in out.items(): check(k,r['volume_mm3']<=.05)
    def radial_diameter(m,x,y,z):
        origin=np.array([x,y,z]); vals=[]
        dirs=[np.array([math.cos(a),math.sin(a),0.]) for a in np.linspace(0,2*math.pi,16,endpoint=False)]
        for direction,hit in zip(dirs,_rays(m,[origin]*len(dirs),dirs)):   # hr42：16 条一次批量
            vals.append(float(np.min((hit-origin)@direction))*2 if len(hit) else None)
        return vals
    out['H01_foot_diameters_mm']={}
    for x,y in [(3.1,-30),(6.6,19),(42.1,-30),(49.1,9.35)]:
        vals=radial_diameter(ms['head_bracket'],x,y,232.3)
        out['H01_foot_diameters_mm'][str((x,y))]=vals
        check('H01 foot '+str((x,y)),all(v is not None and 2.68<v<2.72 for v in vals))
    # 16向承压环：从外侧射向孔所在的板，第一层实体才是螺丝头坐面。
    def seat_x(m,o,ex,ey,ez,face,sign):
        vals=[]
        origins=[o+ey*1.5*math.cos(a)+ez*1.5*math.sin(a)+ex*(face+sign*80.) for a in np.linspace(0,2*math.pi,16,endpoint=False)]
        for hit in _rays(m,origins,[-sign*ex]*len(origins)):          # hr42：16 条一次批量
            ts=np.dot(hit-o,ex) if len(hit) else np.array([])
            ts=ts[(ts-face)*sign>=-1e-5]
            vals.append(float(np.max((ts-face)*sign)) if len(ts) else None)
        return vals
    def servo_seats(m,R,horn):
        og,ex,ey,ez=R[:3,3],R[:3,0],R[:3,1],R[:3,2]
        hr=D.S['horn_r']   # 分度圆半径只认 s288.py 那一个数（09-13 定案 5.25，这里绝不写死）
        coords=[(hr*math.cos(a),hr*math.sin(a)) for a in np.linspace(0,2*math.pi,D.S['horn_n'],endpoint=False)] if horn else [(s*D.S['mnt_dx'],D.S['mnt_z'][0]) for s in (1,-1)]
        face=D.S['T']/2+D.S['flange_h'] if horn else -D.S['T']/2
        return [seat_x(m,og+ey*y+ez*z,ex,ey,ez,face,1 if horn else -1) for y,z in coords]
    out['N02_stack_mm']=servo_seats(ms['neck_pitch'],D.drv_from('neck_pitch'),True)
    out['H02_stack_mm']=servo_seats(ms['head_clamp'],D.drv_self('jaw_soft'),False)
    # 期望叠厚从几何常量推：N02 = 脸颊外表面 17.5 − 共同沉坑 1.1 − 法兰面 13.0 = 3.40（旧 12.85 时 3.55）；H02 = 垫柱 3.2 + 压板 3.0 − 头窝 0.6 = 5.60
    xf_=D.S['T']/2+D.S['flange_h']
    for name,rows,length,expected in [('N02',out['N02_stack_mm'],6,D.NP_CHK_OUT-D.NP_CB-xf_),('H02',out['H02_stack_mm'],8,D.H02_POST_L+D.H02_PLATE_T-D.H02_CB)]:
        check(name+' screw seats',all(v is not None and abs(v-expected)<.02 and 1.8<=length-v<=3 for row in rows for v in row))
    out['H02_arm_stack_mm']=[seat_x(ms['head_clamp'],np.array([0.,y,z]),np.array([1.,0,0]),np.array([0,1.,0]),np.array([0,0,1.]),D.H02_ARM_X[1],-1) for y,z in D.HEAD_CLAMP_MOUNTS]
    check('H02 arm seats',all(v is not None and abs(v-(D.H02_ARM_X[1]-(D.H02_ARM_X[0]+0.05)))<.02 for row in out['H02_arm_stack_mm'] for v in row))   # 6.1：臂 -18.1..-12
    # 螺丝头/起子实心包络，不止看孔中心线。
    Rh=D.drv_from('neck_pitch'); Rr=D.drv_self('jaw_soft')
    out['N02_heads_mm3']=[];out['N03_tools_mm3']=[];out['N03_tools_gap_mm']=[]
    from assembly_audit import solid as _solid_
    for a in np.linspace(0,2*math.pi,D.S['horn_n'],endpoint=False):
        y,z=D.S['horn_r']*math.cos(a),D.S['horn_r']*math.sin(a)
        head=D.placed(D.cyl(4,1.6,(17.2,y,z),axis='x'),Rh)
        out['N02_heads_mm3'].append(V(ms['neck_pitch'],head))
        # 起子从 +x 经 N03 B 端 Ø16.2 通道到 A 盘。此时 N04 销与 B 轴承还没装（装配序 11d/11e 在 11c 之后），所以不查它们。
        # 刀从 A 盘坐面 YRM_A2（世界 x 11.35 = 法兰面 8.1 + 叠厚 3.25）起，换算到横滚舵机局部 x（旧写死 xf+3.25）
        a2=D.YRM_A2-float(D.pt(Rr,0)[0])
        tool=D.placed(D.cyl(4.2,45,(a2+22.5+.01,y,z),axis='x'),Rr)
        out['N03_tools_mm3'].append({n:V(ms[n],tool) for n in ['yrm','head_bracket','head_clamp']})
        # 09-14 复审 R3g MAJOR：原版只判 ∩≤0.05，H01 右前脚承压圈槽 Ø4.3 对 Ø4.2 刀只余 0.048（TC02 缩水后打出来过盈）却通过。
        # 改判 H01/H02 min_gap ≥0.15（槽 H01_FOOT_TOOL_D 4.5 → 0.15）；yrm 不判 gap：刀 −x 端面按定义贴在 A 盘坐面 YRM_A2 外 0.01，gap 恒 0.01。
        out['N03_tools_gap_mm'].append({n:float(_solid_(ms[n]).min_gap(_solid_(tool),3.)) for n in ['head_bracket','head_clamp']})
    check('N02 screw heads',max(out['N02_heads_mm3'])<=.05)
    check('N03 tool corridors',max(v for row in out['N03_tools_mm3'] for v in row.values())<=.05)
    # 阈值 0.15·cos(π/64)−1e-4：刀与槽都是 lib.cyl 的 64 边内接多边形、同相位，平边处名义 0.15 只剩 0.14982（r3 第 1 次 build 实测 0.149817）
    check('N03 tool corridors min_gap (H01/H02) >= 0.15',min(v for row in out['N03_tools_gap_mm'] for v in row.values())>=.15*math.cos(math.pi/64)-1e-4)
    # 头部装配分步 = 拆卸逆序，每一步只允许"后装的件"已经不在（hr11 试算定序，见 head_bearing_rebuild.py 头注 + 记录.md 15）：
    #   H03 -z → N07 盖 -z → N04 键销 -x 退回空偏航舵机腔 → H02 -x → 横滚舵机 -x（N05+A 轴承留在 H01 半座/N03 环里）→ H01 +z 抬离
    #   → 拧掉 F16 后偏航舵机**沿 -y 横着滑出** N03（N03 在 -y 侧无裙墙；-z 被偏航座环挡、+z 被背板挡）→ N06 +z（head_specs stage）
    #   → N05、A 轴承 -x 出 N03 A 环 → B 轴承 +x 脱 N03 台肩 → N03 +z 抬离 N02 轴颈 → 偏航轴承 +z 脱 N02。每条只查当时还在的件。
    SY='servo_yaw_roll_motion_yaw_roll_motion'; SR='servo_jaw_soft_jaw_soft'; BB='bearing_head_roll_B'; BA='bearing_head_roll_A'; BY='bearing_head_yaw'
    N05,N06,N07,N08='head_roll_adapter','head_yaw_adapter','head_bearing_cap','head_yaw_cap'
    # hr41（复审 #1 B1 修法 5）：H03 在装序 P 第 c 步合上，那时已在场的还有 Radxa、嘴舵机、J02（先于 H03 装）→ 宿主补上；
    #   H05/H04/J03/J01/圆眼都在 H03 之后装，不在宿主里（完整分步见 assembly.yaml 步 18 / hr41 报告 §A，由 L4 与 hr41_work/scripts/seq_p.py 逐步核）。
    paths=[('H03_shell','head_bottom_shell',['head_bracket','yrm','head_journal',BB,SR,'head_clamp',N05,N06,N07,N08,BY,BA,SY,'neck_pitch',
                                             'servo_jaw_soft_jaw','jaw_adapter','zz_sbc'],[0,0,-1],45.),
           ('N07_cap',N07,['yrm','head_bracket','head_journal',BB],[0,0,-1],12.),
           ('N04_pin','head_journal',['yrm','head_bracket',BB,SR,BY,N08,N06],[-1,0,0],12.),
           ('H02','head_clamp',['head_bracket'],[-1,0,0],45.),
           ('roll_servo',SR,['yrm',N05,BA,'neck_pitch'],[-1,0,0],45.),
           ('H01_bracket','head_bracket',['yrm',SY,SR,BB,BA,N05,N06,N08,BY,'neck_pitch'],[0,0,1],45.),
           ('yaw_servo_side',SY,['yrm',N06,BY,N08,'neck_pitch','neck',N05,BA,BB],[0,-1,0],40.),
           ('N05_A_adapter',N05,['yrm',BB,BY,N08,'neck_pitch'],[-1,0,0],12.),
           ('bearing_A',BA,['yrm',BB,BY,N08,'neck_pitch'],[-1,0,0],12.),
           ('bearing_B',BB,['yrm',N08,BY,N06,'neck_pitch'],[1,0,0],10.),
           ('N03_empty','yrm',['neck_pitch','neck','servo_neck_neck_pitch'],[0,0,1],30.),
           ('yaw_bearing',BY,['neck_pitch'],[0,0,1],10.)]
    out['assembly_paths_mm3']={}
    for name,mover,hosts,direction,dist in paths:
        vals=[]
        for d in np.arange(0,dist+.01,.5):
            m=ms[mover].copy();m.apply_translation(np.array(direction,float)*d);vals.append(max(V(m,ms[h]) for h in hosts))
        out['assembly_paths_mm3'][name]=max(vals)
        check(name+' insertion',max(vals)<=.05)
    # N04 轴向止挡。+x：上半圈凸缘坐 N03 沉窝（零位交集 0 + 下面的直径检查保证）；-x：头偏航舵机本体（x≤36.0）。
    # 退 0.25 不能碰（名义隙 0.3），退 1.0 必须碰（碰不到 = 舵机根本挡不住销，销会从轴承里退出来）。
    def shifted(name,dx):
        m=ms[name].copy(); m.apply_translation([dx,0.,0.]); return m
    out['N04_axial_stop_mm3']={'retract_0.25':V(shifted('head_journal',-.25),ms[SY]),'retract_1.0':V(shifted('head_journal',-1.),ms[SY])}
    check('N04 -x stop by yaw servo',out['N04_axial_stop_mm3']['retract_0.25']<=.05 and out['N04_axial_stop_mm3']['retract_1.0']>1.)
    # +x 止挡（09-12 复审 MAJOR：以前没有任何检查盯着凸缘是否真坐在沉窝底 —— 沉窝加深 1 mm 或销 +x 端短 1 mm 全套照样 PASS）：
    #   ① N04 与 N03 的最短距 ≤0.05（凸缘 −x 面贴沉窝底）；② 沉窝底的实测 x = YRM_B_WEB + 沉窝深；③ N04 的 x 范围 = (腹板内面, 轴承区 +x 端)
    from assembly_audit import solid as _solid0
    out['N04_plus_x_stop']={'gap_N04_N03_mm':float(_solid0(ms['head_journal']).min_gap(_solid0(ms['yrm']),2.)),
                            'N04_x_bounds':[float(ms['head_journal'].bounds[0,0]),float(ms['head_journal'].bounds[1,0])]}
    cb_floor=[]
    os_=[np.array([D.YRM_B_WEB-.2,8.6*math.cos(a),D.YRM_AXZ+8.6*math.sin(a)]) for a in np.linspace(math.radians(20),math.radians(160),8)]   # 上半圈（凸缘只做上半圈）r=8.6，从腹板内面外 0.2 向 +x 射
    for hit in _rays(ms['yrm'],os_,[[1.,0,0]]*len(os_)):               # hr42：8 条一次批量
        cb_floor.append(float(np.min(hit[:,0])) if len(hit) else None)
    out['N04_plus_x_stop']['cbore_floor_x']=cb_floor
    check('N04 flange seated in counterbore',out['N04_plus_x_stop']['gap_N04_N03_mm']<=.05)
    check('N03 counterbore depth',all(v is not None and abs(v-(D.YRM_B_WEB+D.YRM_B_CB[1]))<.02 for v in cb_floor))
    # hr11：N04 尖端止于 YRM_B_BRG[1]−0.2=44.0，与 N07 盖的轴承面 44.2 留 0.2 夹紧余量（head_specs gaps/N04_tip_vs_N07）
    check('N04 axial extent',abs(out['N04_plus_x_stop']['N04_x_bounds'][0]-D.YRM_B_WEB)<.02 and abs(out['N04_plus_x_stop']['N04_x_bounds'][1]-(D.YRM_B_BRG[1]-0.2))<.02)
    # 头横滚 B 端配合直径：射线从横滚轴心 (y=0,z=AXZ) 在 y-z 面里向外射 16 向（radial_diameter 是给 z 向孔用的，射的是 x-y 面，不能混用）；
    # 半圈座只取存在的那半圈。名义值全从 neck.py / head.py 常量推，不写死。
    AXZ=D.YRM_AXZ
    def x_diameter(m,x,z0=AXZ,half=None,r0=0.0):
        # r0：射线从轴心外 r0 起（hr11：N04 轴承段有 Ø1.7 中心孔穿 N07 中心螺丝，从轴心射会先打到孔壁）
        vals=[]; od=[]
        for a in np.linspace(0,2*math.pi,16,endpoint=False):
            s=math.sin(a)
            if (half=='lower' and s>-1e-6) or (half=='upper' and s<1e-6): continue
            d=np.array([0.,math.cos(a),s]); o=np.array([x,0.,z0])+r0*d
            od.append((o,d))
        for (o,d),hit in zip(od,_rays(m,[o for o,_ in od],[d for _,d in od])):   # hr42：批量
            vals.append((float(np.min((hit-o)@d))+r0)*2 if len(hit) else None)
        return vals
    xb=sum(D.YRM_B_BRG)/2
    out['head_roll_B_fit_mm']={
        'N04_guide_d':x_diameter(ms['head_journal'],D.YRM_B_WEB+1.2),
        'N04_journal_d':x_diameter(ms['head_journal'],xb,r0=1.0),
        'N03_bore_d':x_diameter(ms['yrm'],D.YRM_B_SHLD[0]),
        'N03_cbore_d':[v for v in x_diameter(ms['yrm'],D.YRM_B_WEB+.5) if v is not None],   # 下方几向被 N02 让位刀切空 → None，已滤
        # hr13：N06 出腔槽（neck.n06_exit_band，−y 侧、0.8 深）从腹板内面切进沉窝壁 → 36.8 处槽带内的射线可以是 None 或 >18.2；
        #        沉窝**底层**（槽底 37.1 以外，取 37.2）必须整圈完整 —— N04 凸缘坐面与最后 0.2 的径向壁不受槽影响
        'N03_cbore_d_raw':x_diameter(ms['yrm'],D.YRM_B_WEB+.5),
        'N03_cbore_floor_d':[v for v in x_diameter(ms['yrm'],D.YRM_B_WEB+D.YRM_B_CB[1]-.1) if v is not None],
        'H03_seatB_lower_d':x_diameter(ms['head_bottom_shell'],xb,235.615,'lower'),
        'H01_seatB_upper_d':x_diameter(ms['head_bracket'],xb,235.615,'upper'),
        'H01_lipB_upper_d':x_diameter(ms['head_bracket'],D.YRM_B_BRG[1]+.35,235.615,'upper'),
        'H03_lipB_lower_d':x_diameter(ms['head_bottom_shell'],D.YRM_B_BRG[1]+.35,235.615,'lower')}
    f=out['head_roll_B_fit_mm']
    KEY=HBR_N04_KEY   # hr11：N04 导向段有一条键（radial_top 9.0 → 该向读 Ø18.0），N03 沉窝有对应键槽（slot_top 9.15 → Ø18.3）
    check('N04 guide d',all(v is not None and (abs(v-D.N04_D_GUIDE)<.02 or abs(v-2*KEY['radial_top'])<.02) for v in f['N04_guide_d']))
    check('N04 journal d',all(v is not None and abs(v-D.N04_D_JRN)<.02 for v in f['N04_journal_d']))
    check('N03 bore d',all(v is not None and abs(v-D.YRM_B_BORE)<.02 for v in f['N03_bore_d']))
    band=D.neck.n06_exit_band(); AXZ_=D.YRM_AXZ
    def _in_groove(i,v):
        # 按**名义沉窝壁**（r 9.1）算命中点：槽里的射线要么 None（穿槽出去）要么读大，都不能拿读数反推位置
        a=2*math.pi*i/16; r=D.YRM_B_CB[0]/2; z=AXZ_+r*math.sin(a)
        return math.cos(a)<0 and band[0]-.05<=z<=band[1]+.05             # −y 侧且名义命中点落在槽带 z 内
    cb_out=[v for i,v in enumerate(f['N03_cbore_d_raw']) if v is not None and not _in_groove(i,v)]
    cb_in=[v for i,v in enumerate(f['N03_cbore_d_raw']) if _in_groove(i,v)]
    f['N03_cbore_groove_rays']=cb_in
    check('N03 counterbore d',len(cb_out)>=9 and all(abs(v-D.YRM_B_CB[0])<.02 or abs(v-2*KEY['slot_top'])<.02 for v in cb_out)
          and all(v is None or v>=D.YRM_B_CB[0]-.02 for v in cb_in))
    check('N03 counterbore floor ring d',len(f['N03_cbore_floor_d'])>=9 and all(abs(v-D.YRM_B_CB[0])<.02 or abs(v-2*KEY['slot_top'])<.02 for v in f['N03_cbore_floor_d']))
    check('H03 seat B d',all(v is not None and abs(v-D.P['seat_d'])<.02 for v in f['H03_seatB_lower_d']))
    # hr39c：上半圈**两端**那两根射线从座的开口射出去，读的是座外别的料（>22.5）；H01 那一侧给放大头壳/嘴舵机让位后外面没料了 → None，同样是「射到座外」，不算。
    #   中间 5 根必须 = seat_d（座本身完整），不放宽。
    _ub=f['H01_seatB_upper_d']
    # hr41 落盘 2026-09-25（主设计 Lane C）：m2 —— 旧式对**中间 5 根**也放行 v>22.5（射到座外），座本身缺一块也照样过；现在两端照旧（None / =seat_d / >22.5），
    #   中间 5 根（上半圈 16 向里的 45°..135°）单独要求 |v − seat_d| < 0.02，且必须恰好 5 根（射线数变了要重新审这条）。
    #   旧式：all((v is None and i in (0,len(_ub)-1)) or (v is not None and (abs(v-seat_d)<.02 or v>22.5)) for i,v in enumerate(_ub))
    _ub_end=[_ub[0],_ub[-1]] if _ub else []; _ub_mid=_ub[1:-1]
    check('H01 seat B d',len(_ub_mid)==5 and all(v is None or abs(v-D.P['seat_d'])<.02 or v>22.5 for v in _ub_end)
          and all(v is not None and abs(v-D.P['seat_d'])<.02 for v in _ub_mid))   # 上半圈两端的射线射到座外（>22.5 或 None）不算；中间 5 根必须 = seat_d
    check('H01 lip B d',all(v is not None and abs(v-D.H01_LIP_B[0])<.02 for v in f['H01_lipB_upper_d']))
    check('H03 lip B d',all(v is not None and v>=D.H01_LIP_B[0]-.01 for v in f['H03_lipB_lower_d']))
    # 轴向关系：轴承 -x 面离 N03 台肩 0.1..0.2（贴死会把 Ø21.2 凸台压到外圈上 → 干摩擦），外圈 +x 面贴 H03 唇；N04 离 N02 ≥0.3
    from assembly_audit import solid as _solid
    out['head_roll_B_axial_gap_mm']={'bearing_to_N03':float(_solid(ms[BB]).min_gap(_solid(ms['yrm']),2.)),
                                     'bearing_to_H03':float(_solid(ms[BB]).min_gap(_solid(ms['head_bottom_shell']),2.)),
                                     'N04_to_N02':float(_solid(ms['head_journal']).min_gap(_solid(ms['neck_pitch']),2.))}
    g=out['head_roll_B_axial_gap_mm']
    check('bearing B inner ring on N03 shoulder',g['bearing_to_N03']<=.02)   # hr11：内圈被键销 N04+N07 夹在 N03 Ø17.6 台肩上（旧判据 0.1..0.2 是塑料滑动轴颈时代）
    check('bearing B against H03 lip',g['bearing_to_H03']<=.05)
    check('N04 vs N02 clearance',g['N04_to_N02']>=.2999)
    # 头横滚 ±25°：原来只扫 H01×N03，现在把 N04 和 H03 也扫进去（1° 网格；中点网格由 head_roll_cut_midpoints 那条的同款角度覆盖）
    out['head_roll_N04_1deg']=max(peak(ms[n],ms['head_journal'],'jaw_soft',np.arange(-25,25.01,1))['volume_mm3']
                                  for n in ['head_bracket','head_bottom_shell','head_clamp','head_top_shell',SR])
    out['head_roll_H03_yrm_1deg']=peak(ms['head_bottom_shell'],ms['yrm'],'jaw_soft',np.arange(-25,25.01,1))['volume_mm3']
    check('head_roll N04',out['head_roll_N04_1deg']<=.05); check('head_roll H03 x N03',out['head_roll_H03_yrm_1deg']<=.05)
    out['battery_frozen_region_mm3']=V(ms['trunk'],D.wbox((-200,-200,-200),(-20,200,148)))
    out['B01_receiver_center_segments_mm']={}
    for x,loc in zip(D.DOOR_SCREW_X,_rays(ms['trunk'],[[x,0,65] for x in D.DOOR_SCREW_X],[[0,0,1]]*len(D.DOOR_SCREW_X))):   # hr42：批量
        out['B01_receiver_center_segments_mm'][str(x)]=sorted(float(z) for z in loc[:,2] if 70<z<74) if len(loc) else []
    # T01 倒放：全件最高点着床，不能将内部顶板/甲板说成“贴床”。
    out['T01_inverted_print']={'bed_world_z':float(ms['trunk'].bounds[1,2]),
        'top_plate_height_above_bed_mm':float(ms['trunk'].bounds[1,2]-D.BZR1),
        'deck_height_above_bed_mm':float(ms['trunk'].bounds[1,2]-D.DECK_TOP),
        'battery_floor_height_above_bed_mm':float(ms['trunk'].bounds[1,2]-D.BZ0)}
    # ② 颈俯仰惰轮双支撑（2026-09-12）。四条不变量全部对**做出来的** shell_R / neck 量，不看现场新造的柱：
    #   复核发现"新造一根 idler_hub 再和子树求交"这种检查在 build_trunk_shell 里 union 顺序写错、毂被扫掠体削光时照样通过。
    Rn=D.drv_self('neck'); hd=D.SHR_IDL[2]
    probe=D.placed(D.cyl(hd-1.0,3.0,(D.s288.x_idler_face()-1.5,0,0),axis='x'),Rn)   # 局部 x 惰轮端面-3..惰轮端面（-16..-13）：毂必须完整活到惰轮端面 -13.0（旧写死 -14.5 中心）
    theory=math.pi*((hd-1.0)/2)**2*3-math.pi*(D.S['idler_center_d']/2)**2*3-D.S['horn_n']*math.pi*(D.S['idler_hole_d']/2)**2*3
    out['T03_idler_hub_mm3']=dict(measured=V(ms['shell_R'],probe),theory=float(theory))
    check('T03 idler hub survives (build_trunk_shell union order)',out['T03_idler_hub_mm3']['measured']>=.95*theory)
    zone=D.placed(D.cyl(D.S['rear_boss_d']-.4,1.5,(D.s288.x_idler_face()-.75,0,0),axis='x'),Rn)
    out['N01_into_idler_zone_mm3']=V(ms['neck'],zone)                            # 载体压进惰轮区 = 关节锁死
    check('N01 carrier clear of idler',out['N01_into_idler_zone_mm3']<=1e-6)
    # 皮层 r≤7.3 里除 6 个 Ø2.6 螺丝孔外不许有洞（原版 4 个 XL330 孔外缘 r 7.1 > 毂 7.0，不填就是外观面上的月牙缝）
    x0=D.SHR_IDL[0]; xs=D.XR-D.PLT-D.NECK_CLR
    skin=D.placed(D.cyl(D.SHR_IDL[3],xs-x0,((x0+xs)/2,0,0),axis='x'),Rn)
    pieces=D.diff(skin,ms['shell_R']).split(only_watertight=False)
    out['T03_skin_holes']=dict(n=len(pieces),areas_mm2=sorted(float(c.volume/(xs-x0)) for c in pieces))
    check('T03 skin: only the 6 screw holes',len(pieces)==D.S['horn_n'] and all(abs(a-math.pi*(D.S['idler_hole_d']/2)**2)<.3 for a in out['T03_skin_holes']['areas_mm2']))
    # T03 vs 脖子子树的组合姿态：颈俯仰 -60..45 每 1.25° × 头俯仰 -45/+45（run_checks 只扫单关节）；含 N01 的 6 颗角螺丝头
    Tn,Tp=D.TW('neck'),D.TW('neck_pitch'); heads=D.neck_screw_heads(); worst=0.
    for hp in (-45.,45.):
        n02p=D.placed(ms['neck_pitch'],rot(math.radians(hp),Tp[:3,2],Tp[:3,3]))
        for a in np.arange(-60,45.01,1.25):
            R=rot(math.radians(float(a)),Tn[:3,2],Tn[:3,3])
            for mm in [ms['neck'],ms['servo_neck_neck'],ms['servo_neck_neck_pitch'],n02p,*heads]:
                worst=max(worst,V(D.placed(mm,R),ms['shell_R']))
    out['T03_vs_neck_subtree_mm3']=worst; check('T03 hub vs neck subtree (combined poses)',worst<=.05)
    # 半壳沿 ±y 直线装入：T03 向 -y / T02 向 +y 各退 0..40 mm（远超 SHELL_SLIDE：证明从远处一路直推都不碰），
    # 对 T01 / N01 / 两颗脖子舵机 / N02 / 对侧壳 全程交集 0
    out['shell_slide_mm3']={}
    for side,name in ((-1,'shell_R'),(1,'shell_L')):
        w=0.
        for d in np.arange(0,40.+1e-6,.5):
            m=ms[name].copy(); m.apply_translation((0,side*d,0))
            for o in ['trunk','neck','servo_neck_neck','servo_neck_neck_pitch','neck_pitch','shell_L' if side<0 else 'shell_R']:
                w=max(w,V(m,ms[o]))
        out['shell_slide_mm3'][name]=w; check(name+' straight y install path',w<=.05)
    out['hip_yaw_bearing'] = _hip_yaw_bearing(D, ms, check, peak, V, servo_seats)   # ⑤ 髋偏航 6702（原版是推力式夹持，我们改径向）
    out['review_r1'] = _review_r1(D, ms, check, V)                                   # ⑥ 09-14 复审 8 条的不变量
    out['review_r2'] = _review_r2(D, ms, check, V)                                   # ⑦ 09-14 复审 r2 7 条 MAJOR 的不变量
    out['review_r3'] = _review_r3(D, ms, check, V)                                   # ⑧ 09-14 复审 r3 1 BLOCKER + 4 MAJOR 的不变量
    out['review_r4'] = _review_r4(D, ms, check, V)                                   # ⑨ 09-15 壳柱坐面锪平 4 处
    out['unresolved'] = []
    for x, hits in out['B01_receiver_center_segments_mm'].items():
        if not hits:
            out['unresolved'].append(f'B01 x={x} 后孔沿轴线无仓底接收材料；冻结几何，停用该孔，绑带固定仍待实测')
    from assembly_audit import verify_assembly
    out['assembly_and_fits'] = verify_assembly(D, allm)
    failures.extend(out['assembly_and_fits']['failures'])
    # H03/上壳只允许原版就有的接合交叠；新增重叠不能借白名单藏起来。
    original=D.orig('jaw_soft','bottom_head_shell')
    old_seam=D.inter(original,ms['head_top_shell'])
    new_seam=D.inter(ms['head_bottom_shell'],ms['head_top_shell'])
    # 09-12 起 H03 对上壳让 0.1 mm，交集可能退化成零体积的贴合面片（不是实体）→ 不能再拿去做布尔差，直接记 0。
    new_v=float(new_seam.volume) if (len(new_seam.faces) and new_seam.is_volume) else 0.
    added_v=0. if new_v<1e-6 else max(0.,float(D.diff(new_seam,old_seam).volume))
    out['H03_original_seam']={'reference_mm3':float(old_seam.volume),'current_mm3':new_v,'added_mm3':added_v}
    check('H03 seam introduced collision',out['H03_original_seam']['added_mm3']<.001)
    from assembly_audit import solid
    gap=solid(ms['head_bottom_shell']).min_gap(solid(ms['servo_jaw_soft_jaw_soft']),2.)
    out['H03_motor_gap_mm']=float(gap)
    check('H03 motor nominal clearance',gap>=.2999)
    # 09-14：H03 对 H02 压板/垫柱同样 ±CLR（build_head_bottom_shell 的 clamp_clr）。修前 0.244（板角 vs 内壁）、舵机 +0.2 后 0.069（下垫柱底 vs 地板垫台立壁）
    out['H03_clamp_gap_mm']=float(solid(ms['head_bottom_shell']).min_gap(solid(ms['head_clamp']),2.))
    check('H03 vs H02 nominal clearance',out['H03_clamp_gap_mm']>=.2999)
    out['failures']=failures;out['passed']=not failures
    out['passed_scope']='仅已修接口和错相回归；不包含 unresolved 或整机全行程放行'
    print('\n[已修接口与错相回归] '+('PASS' if not failures else 'FAIL: '+', '.join(failures)),flush=True)
    for name in ['neck_trunk','neck_hip_servo','head_roll_1deg','head_roll_cut_midpoints','N02_body_clearance_0.25mm','L02_opposite_L01','L04_battery','knee_inverse_range']:
        print(f"  {name}: {out[name]['volume_mm3']:.6f} mm³")
    print(f"  T03 惰轮毂 {out['T03_idler_hub_mm3']['measured']:.1f}/{out['T03_idler_hub_mm3']['theory']:.1f} mm³ | N01 进惰轮区 {out['N01_into_idler_zone_mm3']:.4f} | "
          f"皮层洞 {out['T03_skin_holes']['n']} | T03×脖子组合姿态 {out['T03_vs_neck_subtree_mm3']:.4f} | 壳直线装入 {out['shell_slide_mm3']}")
    r6=out['review_r1']
    print(f"  [09-14 复审不变量] H02↔头横滚惰轮 {r6['H02_vs_head_roll_idler']['min_gap_mm']:.3f} | 壳↔髋偏航惰轮 L {r6['shell_vs_hip_yaw_idler']['shell_L']['min_gap_mm']:.3f} R {r6['shell_vs_hip_yaw_idler']['shell_R']['min_gap_mm']:.3f} | "
          f"头横滚舵机↔上头壳 {r6['head_roll_servo_vs_top_shell_mm']:.3f} | F12 头×T02 静态 {max(r6['F12_heads_vs_shell_L']['static_mm3']):.3f} 滑入 {r6['F12_heads_vs_shell_L']['slide_peak_mm3']:.3f} 销 {r6['F12_heads_vs_shell_L']['footprint_pins_mm3']:.3f} | "
          f"H01 脚4 最薄壁 {r6['H01_foot4_min_wall']['mm']:.3f}@{r6['H01_foot4_min_wall']['at']} | L01 头↔横滚舵机 gap L {min(out['hip_yaw_bearing']['L']['L01_head_vs_roll_servo']['min_gap_mm']):.3f} R {min(out['hip_yaw_bearing']['R']['L01_head_vs_roll_servo']['min_gap_mm']):.3f} | H03↔H02 {out['H03_clamp_gap_mm']:.3f}")
    r7=out['review_r2']; m1=r7['T02_yoke_opening']
    print(f"  [09-14 复审 r2 不变量] T02 轭区 开口核心/外圈带 最大有料 {max(v['core_fill'] for v in m1.values()):.3f}/{max(v['band_fill'] for v in m1.values()):.3f} 左缘最窄 {min(v['left_min_width_mm'] or 0 for v in m1.values()):.2f} | "
          f"膝 L04↔L03 {r7['knee_L04_vs_L03']['zero_mm']:.3f}/{r7['knee_L04_vs_L03']['min_gap_mm']:.3f}@{r7['knee_L04_vs_L03']['angle_deg']} | 踝 L07↔L04 {r7['ankle_L07_vs_L04']['zero_mm']:.3f}/{r7['ankle_L07_vs_L04']['min_gap_mm']:.3f}@{r7['ankle_L07_vs_L04']['angle_deg']} "
          f"L05↔L04 {r7['ankle_L05_vs_L04']['zero_mm']:.3f}/{r7['ankle_L05_vs_L04']['min_gap_mm']:.3f}@{r7['ankle_L05_vs_L04']['angle_deg']} | H03 A 座 Ø{np.mean([v for v in r7['H03_seatA_lower_d'] if v]):.3f} N03↔H03 {r7['N03_vs_H03']['zero_mm']:.3f}/{r7['N03_vs_H03']['min_gap_mm']:.3f}@{r7['N03_vs_H03']['angle_deg']} | "
          f"L04↔电池(仓内极限) L {r7['L04_vs_battery_in_bay']['L']['shifted_peak_mm3']:.3f} mm³ 名义 {r7['L04_vs_battery_in_bay']['L']['nominal_min_gap_mm']:.3f}@{r7['L04_vs_battery_in_bay']['L']['nominal_at_deg']} 极限位 gap {r7['L04_vs_battery_in_bay']['L']['shifted_min_gap_mm']:.3f}@{r7['L04_vs_battery_in_bay']['L']['shifted_min_gap_at_deg']}{r7['L04_vs_battery_in_bay']['L']['shifted_min_gap_shift']} R {r7['L04_vs_battery_in_bay']['R']['shifted_peak_mm3']:.3f} 名义 {r7['L04_vs_battery_in_bay']['R']['nominal_min_gap_mm']:.3f} 极限位 gap {r7['L04_vs_battery_in_bay']['R']['shifted_min_gap_mm']:.3f} | "
          f"髋横滚座挡肩↔轴套 {r7['hip_roll_L01_shoulder']['shoulder_vs_inner_ring_mm']:.3f} ↔L02 {r7['hip_roll_L01_shoulder']['shoulder_vs_L02_mm']:.3f} 挡肩存在 {r7['hip_roll_L01_shoulder']['shoulder_present_frac']:.2f} L01↔L02 {r7['hip_roll_L01_shoulder']['L01_vs_L02_zero_mm']:.3f}/{r7['hip_roll_L01_shoulder']['L01_vs_L02_min_gap_mm']:.3f}")
    r8=out['review_r3']
    r9=out['review_r4']
    print('  [09-15 壳柱锪平不变量] '+' | '.join(f"{k} 平面度 {v['flatness_mm']:.3f} 壁 {v['wall_min_mm']:.3f}" for k,v in r9.items()))
    print(f"  [09-14 复审 r3 不变量] H02 臂孔臂端实体 {r8['H02_arm_holes']['solid_at_arm_end_mm']:.3f}/孔内 {r8['H02_arm_holes']['solid_in_hole_mm']:.3f} | "
          f"N03 起子刀↔H01/H02 min_gap {min(v for row in out['N03_tools_gap_mm'] for v in row.values()):.3f} | "
          f"L01 毂孔圈壁 L {r8['L01_hub_ligament']['L']['min_mm']:.3f} R {r8['L01_hub_ligament']['R']['min_mm']:.3f}（豁免 ≥0.9，名义 {r8['L01_hub_ligament']['L']['nominal_mm']:.3f}） | "
          f"T03 颈区皮层(头扫掠带 {r8['T03_neck_skin']['n_head_band']} 点) 平面区 min {r8['T03_neck_skin']['plane_min_mm']:.3f}@{r8['T03_neck_skin']['plane_at']} 开口沿圆角 {r8['T03_neck_skin']['rim_n']} 点 min {r8['T03_neck_skin']['rim_min_mm']} 全区 <1.2 {r8['T03_neck_skin']['n_all_below_1p2']} <0.8 {r8['T03_neck_skin']['n_all_below_0p8']} | "
          f"N01 头↔T03 {r8['N01_heads_vs_T03']['zero_mm']:.3f}/{r8['N01_heads_vs_T03']['min_gap_mm']:.3f}@{r8['N01_heads_vs_T03']['angle_deg']} | "
          f"L07 榫槽↔孔壁 L {r8['L07_key_slot_to_hole']['L']['min_mm']} R {r8['L07_key_slot_to_hole']['R']['min_mm']}")
    for message in out['unresolved']: print('  [未闭环] '+message)
    return out
