"""成品装配回归：完整轴承、实体装入路径和最短间隙。单位mm/mm³。

仅采样验证，不把孔径、零交集或空载体检查当成完整装配通过。
"""
import math
import numpy as np
import manifold3d as M


def solid(mesh):
    shape = M.Manifold(M.Mesh64(
        # Mesh64.to_mesh64() exposes read-only buffers. nanobind requires writable
        # arrays even though this call only reads them; contiguity alone is insufficient.
        vert_properties=np.array(mesh.vertices, dtype=np.float64, order='C', copy=True),
        tri_verts=np.array(mesh.faces, dtype=np.uint64, order='C', copy=True)))
    if shape.status() != M.Error.NoError:
        raise ValueError(f"装配检查实体转换失败: {shape.status()}")
    return shape


def intersection(a, b):
    aa = np.asarray(a.bounding_box()).reshape(2, 3)
    bb = np.asarray(b.bounding_box()).reshape(2, 3)
    if np.any(aa[1] < bb[0]) or np.any(bb[1] < aa[0]):
        return 0.0
    hit = a ^ b
    if hit.status() != M.Error.NoError:
        raise ValueError(f"装配检查布尔失败: {hit.status()}")
    value = float(hit.volume())
    if not math.isfinite(value) or value < -1e-6:
        raise ValueError(f"装配检查体积异常: {value}")
    return max(0.0, value)


def servo_name(D, body, index):
    return 'servo_' + body + '_' + D.B[body]['servos'][index]['drives'].replace(':self', '')


def bearing_specs(D):
    """完整轴承实体（外径×内径×4 厚），舵机局部 x0 = 环的 -x 面。
    mount = 轴承跟着哪个 MJCF 连杆动（外圈压在谁家）：6704 外圈在载体 L01/L03（= body 自己），
    6700 在从动件 L07（= 该舵机驱动的连杆），头横滚 B 端 22×16×4 外圈在 H01/H03（= jaw_soft）、内圈在 N04 销上。
    build.py / qcheck.py 用 mount 决定轴承参加哪个连杆的扫掠，不再从 bore 猜。
    法兰侧轴承（6704/6702）的 -x 面 = 法兰面 x_flange_face()=13.0（旧写死 12.85）；踝 6700 的 -x 面 = lib.ANK_X1b（L05 座终点，-21.4）。"""
    from duckstructure.bearing_rebuild import HR
    from duckstructure.hip_pitch_bearing_rebuild import HP
    from duckstructure.head_bearing_rebuild import HEAD_A, HEAD_YAW
    XF = D.S['T'] / 2 + D.S['flange_h']
    for label, body, index, carrier, driven, od, bore, x0, mount in [
        ('left_hip_roll', 'yaw2roll', 0, 'yaw2roll', 'hip_roll_sleeve', HR['od'], HR['bore'], HR['x0'], 'yaw2roll'),        # hr08 整体座
        ('right_hip_roll', 'bearing_roll', 0, 'yaw2roll_R', 'hip_roll_sleeve_R', HR['od'], HR['bore'], HR['x0'], 'bearing_roll'),
        ('left_hip_pitch', 'upper_leg_left', 1, 'upper_leg', 'hip_pitch_sleeve', HP['od'], HP['bore'], HP['x0'], 'upper_leg_left'),      # hr07 整体座
        ('right_hip_pitch', 'upper_leg_right', 1, 'upper_leg_R', 'hip_pitch_sleeve_R', HP['od'], HP['bore'], HP['x0'], 'upper_leg_right'),
        ('left_knee', 'upper_leg_left', 0, 'upper_leg', 'lower_leg', 27., 20., XF, 'upper_leg_left'),
        ('right_knee', 'upper_leg_right', 0, 'upper_leg_R', 'lower_leg_R', 27., 20., XF, 'upper_leg_right'),
        ('left_ankle', 'leg', 0, 'lower_leg', 'ankle_foot', 15., 10., D.ANK_X1b, 'ankle_left'),
        ('right_ankle', 'leg_2', 0, 'lower_leg_R', 'ankle_foot_R', 15., 10., D.ANK_X1b, 'ankle_right'),
        # ⑤ 髋偏航法兰侧 6702ZZ 15×21×4（09-12；原版此处本来就有一颗 22×16×4 推力式夹持（受力报告第 96 行），S288 配不了那种夹法，我们改法兰侧径向 6702）：外圈座在 T01，内圈毂在 L01，与 6704 同站位 x 13.0..17.0（旧 12.85..16.85）
        ('left_hip_yaw', 'trunk_base', 0, 'trunk', 'yaw2roll', D.P['brg_yaw']['od'], D.P['brg_yaw']['bore'], XF, 'trunk_base'),
        ('right_hip_yaw', 'trunk_base', 1, 'trunk', 'yaw2roll_R', D.P['brg_yaw']['od'], D.P['brg_yaw']['bore'], XF, 'trunk_base'),
        # 头横滚 B 端（09-12 ④）：22×16×4，外圈在 H01 上半座 + H03 下半座，内圈在 N04 销。x0 从 neck.YRM_B_BRG 推：
        # 世界 40.2 − 横滚舵机心 x(−4.7，含 SERVO_SHIFT −1.7；09-14 前 −4.9/−1.5) = 44.9（舵机局部 x 与世界 x 同向）—— 代码里按帧推，不写死
        ('head_roll_A', 'jaw_soft', 1, 'head_bracket', 'head_roll_adapter', HEAD_A['od'], HEAD_A['bore'],
         HEAD_A['x0'] - float(D.pt(D.sfw('jaw_soft',1),0)[0]), 'jaw_soft'),
        ('head_yaw', 'yaw_roll_motion', 0, 'yrm', 'neck_pitch', HEAD_YAW['od'], HEAD_YAW['bore'],
         float(D.pt(D.sfw('yaw_roll_motion',0),0)[2]) - HEAD_YAW['z1'], 'yaw_roll_motion'),
        ('head_roll_B', 'jaw_soft', 1, 'head_bracket', 'head_journal', 22., 16., None, 'jaw_soft'),
    ]:
        if label in ('left_hip_roll', 'right_hip_roll') and not D.RINGS['left_hip_roll']:
            continue                                   # 09-17：髋横滚法兰侧 6704 取消（lib.RINGS 注释）
        if label in ('left_hip_pitch','right_hip_pitch') and not D.RINGS['left_hip_pitch']:
            continue
        if x0 is None:
            x0 = float(D.YRM_B_BRG[0] - D.pt(D.sfw(body, index), 0)[0])
        ring = solid(D.cyl(od, 4, (x0+2, 0, 0), axis='x', sections=256)) - solid(
            D.cyl(bore, 4.1, (x0+2, 0, 0), axis='x', sections=256))
        yield dict(name=label, body=body, carrier=carrier, driven=driven,
                   shape=ring.transform(D.sfw(body, index)[:3]), od=od, bore=bore, x0=x0, mount=mount)


def _cc():
    import check_cache
    return check_cache


def shape_digest(shape):
    """hr42：manifold 实体的内容摘要（它自己导出的 float64 网格字节）—— 检查原语缓存的键。"""
    mm = shape.to_mesh64()
    return _cc().array_digest(np.asarray(mm.vert_properties, dtype=np.float64), np.asarray(mm.tri_verts, dtype=np.int64))


def intersection_keyed(key_parts, a, b):
    """hr42：= intersection(a, b)，结果按 key_parts（两实体摘要 + 位姿字节，由调用方给全）走检查原语缓存。"""
    CC = _cc()
    if not CC.enabled() or key_parts is None:
        return intersection(a, b)
    k = CC.key("assembly_audit.intersection", *key_parts)
    hit, v = CC.get("asm", k)
    if hit:
        return v
    v = intersection(a, b)
    CC.put("asm", k, v)
    return v


def path_peak(movers, obstacles, direction, step=.5, distance=60., dig=None):
    """dig（hr42，可选）：{名字: 实体内容摘要}；给了就逐次求交走检查原语缓存（键 = 移动件摘要 + 平移向量字节 + 障碍件摘要）。"""
    result = dict(volume_mm3=0., at_mm=0., mover=None, obstacle=None)
    for d in np.arange(0, distance+1e-7, step):
        for name, shape in movers.items():
            tv = np.asarray(direction)*d
            moved = shape.translate(tv)
            for other, wall in obstacles.items():
                kp = None if dig is None else (dig[name], np.asarray(tv, dtype=np.float64), dig[other])
                if dig is not None:
                    _cc().set_context(f"assembly:{name}×{other}")
                value = intersection_keyed(kp, moved, wall)
                if value > result['volume_mm3']:
                    result = dict(volume_mm3=value, at_mm=float(d), mover=name, obstacle=other)
    return result


AXES = [(sign+a, value*np.eye(3)[i]) for i, a in enumerate('xyz')
        for sign, value in [('+', 1), ('-', -1)]]


def verify_assembly(D, allm):
    from asmcheck import CARRIER
    meshes = {name: mesh for _, name, mesh in allm}
    shapes = {name: solid(mesh) for name, mesh in meshes.items()}
    CC = _cc()
    dig = {name: CC.mesh_digest(mesh) for name, mesh in meshes.items()} if CC.enabled() else None   # hr42：实体 = solid(源网格)，摘要取源网格
    failures = []
    bearings = list(bearing_specs(D))
    for spec in bearings:
        if dig is not None:
            dig['bearing:' + spec['name']] = shape_digest(spec['shape'])
    bearing_rows = []
    for spec in bearings:
        CC.set_context(f"assembly:bearing_{spec['name']}×*")
        overlaps = [dict(part=name, volume_mm3=intersection_keyed(
                             None if dig is None else (dig['bearing:' + spec['name']], dig[name]), spec['shape'], shape))
                    for name, shape in shapes.items() if name != 'bearing_'+spec['name']]
        blocked = [r for r in overlaps if r['volume_mm3'] > .05]
        if blocked:
            failures.append(spec['name'] + ' bearing envelope')
        bearing_rows.append(dict(name=spec['name'], od_id_t_mm=[spec['od'],spec['bore'],4],
                                 axial_x_mm=[spec['x0'],spec['x0']+4], intersections=overlaps))
        print(f"  [轴承] {spec['name']}: {max(r['volume_mm3'] for r in overlaps):.6f} mm³", flush=True)
    paths = []
    for (body, index), carrier in CARRIER.items():
        mover = servo_name(D, body, index)
        neighbours = [servo_name(D, b, i) for (b, i) in CARRIER if b == body and i != index]
        obstacles = {n: shapes[n] for n in [carrier]+neighbours}
        rows = [dict(direction=tag, **path_peak({mover: shapes[mover]}, obstacles, direction, dig=dig))
                for tag, direction in AXES]
        best = min(rows, key=lambda r:r['volume_mm3'])
        if best['volume_mm3'] > .05:
            failures.append(mover + ' straight insertion')
        paths.append(dict(servo=mover, carrier=carrier, installed=neighbours, paths=rows))
        print(f"  [装入] {mover}: {best['direction']} {best['volume_mm3']:.6f} mm³", flush=True)
    ankle_paths = []
    for side, body, foot, arm, carrier in [('left','leg','ankle_foot','ankle_rear_arm','lower_leg'),('right','leg_2','ankle_foot_R','ankle_rear_arm_R','lower_leg_R')]:
        bearing = next(r['shape'] for r in bearings if r['name'] == side+'_ankle')
        if dig is not None:
            dig['bearing'] = dig['bearing:' + side + '_ankle']
        sn=servo_name(D,body,0); R=D.sfw(body,0); ex=R[:3,0]
        stages=[('main_to_servo',{foot:shapes[foot]},{sn:shapes[sn]},ex),
                ('main_and_servo_to_L04',{foot:shapes[foot],sn:shapes[sn]},{carrier:shapes[carrier]},ex),
                ('bearing_to_detached_arm',{'bearing':bearing},{arm:shapes[arm]},ex),
                ('arm_and_bearing_to_assembly',{arm:shapes[arm],'bearing':bearing},
                 {foot:shapes[foot],sn:shapes[sn],carrier:shapes[carrier]},-ex)]
        rows=[dict(stage=tag,direction=direction.tolist(),**path_peak(movers,obstacles,direction,step=.25,dig=dig))
              for tag,movers,obstacles,direction in stages]
        worst=max(r['volume_mm3'] for r in rows)
        if worst > .05:
            failures.append(side+' ankle subassembly insertion')
        ankle_paths.append(dict(side=side,paths=rows))
        print(f"  [踝子装配] {side}: 四个装配步骤峰值 {worst:.6f} mm³",flush=True)
    gaps = []
    ankle_motion = []
    # 1度切刀与2.5度主检查都不能代替中间角回归。加入真实策略反例角，
    # 连同0.125度网格检查完整踝子树相对小腿/电机；其他关节保持零位。
    angles = sorted(set(np.arange(-35., 35.00001, .125).tolist() +
                        [-33.125269692, -33.372934686, 33.125269692, 33.372934686]))
    for side, body, suffix in [('left', 'ankle_left', ''), ('right', 'ankle_right', '_R')]:
        parent = 'leg' if side == 'left' else 'leg_2'
        mover_names = ['ankle_foot'+suffix, 'ankle_rear_arm'+suffix,
                       'sole'+suffix, 'bearing_'+side+'_ankle']
        obstacle_names = ['lower_leg'+suffix, servo_name(D, parent, 0)]
        T = D.TW(body)
        peaks = {(a,b): dict(a=a,b=b,volume_mm3=0.,angle_deg=0.)
                 for a in mover_names for b in obstacle_names}
        for angle in angles:
            transform = D.rot(math.radians(angle), T[:3,2], T[:3,3])[:3]
            for a in mover_names:
                moved = shapes[a].transform(transform)
                for b in obstacle_names:
                    CC.set_context(f"assembly:{a}×{b}")
                    value = intersection_keyed(None if dig is None else (dig[a], np.asarray(transform, dtype=np.float64), dig[b]),
                                               moved, shapes[b])
                    if value > peaks[a,b]['volume_mm3']:
                        peaks[a,b] = dict(a=a,b=b,volume_mm3=value,angle_deg=angle)
        worst = max(r['volume_mm3'] for r in peaks.values())
        if worst > .05:
            failures.append(side+' ankle intermediate-angle collision')
        ankle_motion.append(dict(side=side,range_deg=[-35,35],step_deg=.125,
                                 samples=len(angles),peaks=list(peaks.values())))
        print(f"  [踝错相回归] {side}: {len(angles)}姿态，峰值 {worst:.6f} mm³",flush=True)
    T = D.TW('neck')
    # A nonzero floor is separate from the volume threshold. Cutter angles are
    # never reused; half-grid samples catch intermediate contact.
    for shell in ['shell_L','shell_R']:
        best = dict(gap_mm=2.,angle_deg=None)
        for angle in np.arange(-60,45.001,1.25):
            matrix=D.rot(math.radians(float(angle)),T[:3,2],T[:3,3])
            moved=shapes['neck'].transform(matrix[:3])
            if dig is not None:                                    # hr42：min_gap 走检查原语缓存（键 = 两源网格摘要 + 位姿字节 + 搜索半径）
                gap=float(CC.cached('asm', CC.key('assembly_audit.min_gap', dig['neck'], np.asarray(matrix[:3], dtype=np.float64), dig[shell], 2.0),
                                    lambda: float(moved.min_gap(shapes[shell],2.))))
            else:
                gap=float(moved.min_gap(shapes[shell],2.))
            if gap<best['gap_mm']:
                best=dict(gap_mm=gap,angle_deg=float(angle))
        if best['gap_mm'] < .3-1e-5:
            failures.append('neck / '+shell+' clearance <0.3mm')
        gaps.append(dict(a='neck',b=shell,required_nominal_gap_mm=.3,**best))
        print(f"  [最短间隙] neck × {shell}: {best['gap_mm']:.6f} mm",flush=True)
    return dict(bearings=bearing_rows,servo_paths=paths,ankle_paths=ankle_paths,ankle_motion=ankle_motion,
                neck_shell_gaps=gaps,failures=failures,passed=not failures,
                scope='Nominal metal envelopes, specified rigid insertion families, sampled neck/shell gap; no physical strength or continuous-motion proof')
