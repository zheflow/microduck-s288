"""L05/L07可装配分体：M2×12连接、承压台阶、定位榫、6700座和TPU让位。
输入世界坐标成品毛坯，返回主脚、可拆后臂、鞋底；不修改输入网格。
"""
import numpy as np
import trimesh
import manifold3d as M
from duckstructure.s288 import cyl

# 09-14 复审 R3a MAJOR：原版 bolt_centers (±3, 18)，过孔 2.2→2.4（09-13）后孔壁 y ±1.8 到定位榫槽壁 ±1.2 只剩 0.60（旧 0.70）。
# 榫槽 ±1.2 / 榫 ±1.0 是配合面不动；孔心外移 0.4 → (±3.4, 18)：槽壁→孔壁 1.0、孔外壁 4.6 到桥边 6.2 仍 1.6、Ø4.4 头窝 1.2..5.6 到 Ø6.4 凸台外缘 6.6 仍 1.0、
# 两头窝之间腹板 1.6→2.4。凸台 Ø6.4 随孔外移到 y 6.6（比桥边 6.2 多凸 0.4，弦高 ±1.55）—— L05/L07 对 L04 的 min_gap 由 mechanical_audit review_r2 M3/M4（≥0.3）盯着。
# 孔位只在这里定义（L05 底孔 Ø1.6 / L07 过孔 Ø2.4 / 头窝 / 凸台 / TPU 起子孔全从 bolt_centers_yz 推），不涉及 L04。
ANKLE_SPLIT = dict(
    bolt_centers_yz=((-3.4, 18.), (3.4, 18.)),
    contact_x=-13.7, main_split_x=-13.5,
    screw_length=12., head_seat_x=-19., screw_tip_x=-7., thread_engagement=6.7,
    pilot_d=1.6, through_d=2.4, counterbore_d=4.4, boss_d=6.4,   # 09-13：过孔 2.2→2.4、头窝 4.2→4.4（与 s288.mnt_hole_d / lib.P.m2_cbore_d 同口径；boss_d 6.4 对 4.4 头窝壁 1.0）
    driver_d=4., driver_length=45., head_d=4., head_h=1.6,
    bridge_top_z=21.2, key_size_yz=(2., 3.), key_depth=2., key_clearance=.2,
    sole_opening_end_x=-18.6, sole_opening_half_y=6.4,
)


def native(mesh):
    result = M.Manifold(M.Mesh64(np.ascontiguousarray(mesh.vertices, dtype=np.float64),
                                np.ascontiguousarray(mesh.faces, dtype=np.uint64)))
    assert result.status() == M.Error.NoError, result.status()
    return result


def mesh(shape):
    result = shape.to_mesh64()
    return trimesh.Trimesh(np.asarray(result.vert_properties)[:, :3],
                           np.asarray(result.tri_verts), process=False)


def split_ankle_foot_native(foot):
    """Input/output manifold solids in the ankle servo frame, dimensions mm."""
    def b(lo, hi): return M.Manifold.cube(np.array(hi)-lo).translate(lo)
    def c(d, h, center): return native(cyl(d, h, center, axis='x', sections=96))
    p = ANKLE_SPLIT
    face = p['contact_x']
    main = foot ^ b((p['main_split_x'], -80, -80), (80, 80, 80))
    arm = foot ^ b((-80, -80, -80), (face, 80, 80))
    centers = p['bolt_centers_yz']
    for y, z in centers:
        main += c(p['boss_d'], 8.9, (-9.25, y, z))
        arm += c(p['boss_d'], 8.8, (-18.1, y, z))
    # Axial clamp faces meet at x=-13.7; the remaining seam keeps 0.2 clearance.
    main += b((face, -6.2, 16.), (-10., 6.2, p['bridge_top_z']))
    arm += b((-18., -6.2, 16.), (face, 6.2, p['bridge_top_z']))
    # 09-14 R3a：孔心 ±3 → ±3.4 后两根 Ø6.4 凸台在桥以外那段（主脚 x −10..−4.8、后臂 −22.5..−18）不再相互重叠（原版重叠 0.4，现相距 0.4），
    # 中间留一道 0.4 宽的 V 谷。补一条 y ±1.6 的腹板把两根凸台连回去（头窝/底孔随后再切，头窝层腹板 1.6→2.4）；全在凸台包络里，不碰 L04。
    main += b((-10., -1.6, 16.), (-9.25 + 8.9 / 2, 1.6, p['bridge_top_z']))
    arm += b((-18.1 - 8.8 / 2, -1.6, 16.), (-18., 1.6, p['bridge_top_z']))
    main += b((-15.7, -1., 16.7), (-13.5, 1., 19.7))
    arm -= b((-15.9, -1.2, 16.5), (-13.6, 1.2, 19.9))
    # Feed the 6700 from +x before the detached arm slides onto the L04 journal.
    arm -= c(15.1, 8., (-17.4, 0, 0))
    arm += c(19.6, .7, (-21.75, 0, 0)) - c(13.5, .9, (-21.75, 0, 0))
    # Positive overlap handles the original STL's float32 rear face at -21.4.
    arm += c(19.6, .2, (-21.4, 0, 0)) - c(15.1, .3, (-21.4, 0, 0))
    for y, z in centers:
        main -= c(p['pilot_d'], 9.2, (-9.25, y, z))
        arm -= c(p['through_d'], 9., (-18.1, y, z))
        arm -= c(p['counterbore_d'], 4., (p['head_seat_x']-2., y, z))
    # Remove sub-float32 slivers, displacement <0.00001 mm; retain both solids.
    return main.simplify(.00001), arm.simplify(.00001)


def split_ankle_foot_world(foot_world, ankle_servo_frame):
    local = foot_world.copy()
    local.apply_transform(np.linalg.inv(ankle_servo_frame))
    output = []
    for part in split_ankle_foot_native(native(local)):
        assert len(part.decompose()) == 1
        result = mesh(part)
        result.apply_transform(ankle_servo_frame)
        output.append(result)
    return tuple(output)


def split_ankle(rawfoot, rawsole, ankle_servo_frame=None):
    """World meshes -> (main foot, detachable rear arm, cleared TPU sole).

    The TPU gets explicit driver access holes: material inside the rear-arm
    counterbores must not survive subtraction as isolated rubber plugs.
    """
    if ankle_servo_frame is None:
        from duckstructure import drv_from
        ankle_servo_frame = drv_from('ankle_left')
    inverse = np.linalg.inv(ankle_servo_frame)
    foot = rawfoot.copy(); foot.apply_transform(inverse)
    sole = rawsole.copy(); sole.apply_transform(inverse)
    main, arm = split_ankle_foot_native(native(foot))
    sole = native(sole) - main - arm
    # Explicit 12.8 mm side-skirt opening exposes both fasteners. A round hole
    # would leave near-zero-thickness lips where it meets the original curved
    # TPU wrap. The opening ends at x=-18.6; the remaining load-bearing floor
    # starts there, below the lowered bridge (z<=21.2).
    p = ANKLE_SPLIT
    sole -= M.Manifold.cube((80.+p['sole_opening_end_x'], 2*p['sole_opening_half_y'], 160.)).translate((-80., -p['sole_opening_half_y'], -80.))
    for y,z in p['bolt_centers_yz']:
        sole -= native(cyl(2.6, 16., (-14., y, z), axis='x', sections=96))
    output = []
    for shape in [main, arm, sole]:
        # 2026-09-16：鞋底开始吃膝刀（Minkowski 膨胀后的扫掠体）后，减法留下 8 片 |体积| < 1e-11 mm³ 的共面碎片；
        # 真·多实体仍然要炸，所以只丢零体积碎片（阈值 1e-3 mm³），剩下的必须恰好一块。
        # was_until_2026_09_28_hr50: comps = [c for c in shape.decompose() if abs(c.volume()) > 1e-3]
        # hr50：试验线翘区扫掠改连续膨胀 + 0.25°/步时，后臂分体切口旁多出一粒 0.0012 mm³ 的布尔碎粒（< 0.11 mm 见方，打不出来）把这里炸了；
        #   那版扫掠已恢复原样，门槛仍放到 0.01 mm³ 兜底：只丢这种碎粒，真·多实体（任何 ≥0.01 mm³ 的第二块）照样炸。
        comps = [c for c in shape.decompose() if abs(c.volume()) > 1e-2]
        assert len(comps) == 1, [c.volume() for c in shape.decompose()]
        part = mesh(comps[0]); part.apply_transform(ankle_servo_frame); output.append(part)
    return tuple(output)
