"""Head bearing rebuild proposal; all dimensions are nominal design inputs.

Four extra printed parts: N05 roll adapter, N06 yaw adapter, N07 B inner-ring
cap, N08 yaw outer-ring cap. No new journal/seat fit is physically qualified.
Neither a 0.15 mm nominal diametral gap nor the 0.30 mm clamp relief is a
measured process allowance. Retain unknown/FAIL until coupons and assembly
preload/drag are measured. Never copy the small-hole offset onto these fits.

A: the six M2x8 screws clamp N03 -> bearing INNER ring -> N05 -> servo horn.
N05's central end stays short of N03, so it cannot bypass that load path.
B: keyed N04 and N07/M2x8 clamp the INNER ring to the N03 inner shoulder.
The B outer ring locates the assembly; A's outer ring has axial float.
Yaw: six M2x8 clamp N02 -> INNER ring -> N06 -> horn. N08/two M2x6 retain
and clamp only the OUTER ring to N03. Screw-ear relief prevents a hard-stop
bypass. Shoulder diameters must be checked against the selected bearing's
actual inner/outer race and shield boundaries; no seal contact is assumed.

Assembly: put A bearing on N05 from +x; place N03/N05/bearing into open H01,
insert roll servo from -x, tighten F17 through the still-open B tool tunnel.
Install B bearing, keyed N04 from the empty yaw-motor cavity, then N07 from
the open underside and tighten its center screw through the +x tool tunnel.
Install H02. Preassemble the yaw bearing and N08 into N03; insert N02's
journal from below and N06 from the empty yaw-motor cavity, then install the
yaw servo (CF3 already plugged) and tighten F16 from below through N02.
H03 closes last. For service remove the yaw servo to an open workstation
before unplugging CF3; in-situ complete unplugging is NOT claimed. The full
plugged-servo insertion path and cable bending still need stage simulation.

CF3 origin audit (superseded 2026-09-20): the side-plug model behind the
old CONN_PULL5.5 corridor was wrong. Photos of the real S288 show both PH2.0
sockets in notches on the BACK (idler) face, pins toward -x; the mated PHR-3
ends flush with the 23 plane (x=-13) and only the wire adds ~2 (S.wire_out).
The yaw-servo back plate is therefore cut as a "window" (lib.CONN_MODE), the
plug is inserted from above the plate after the servo is in, and no side
corridor exists any more; the unplug criterion is "window footprint free to
x=-18.5" (keepouts.yaml:KO01 rows[yaw_roll_motion]).
The nominal runtime protrusion and cable envelope are separate checks.
A trial motor shift -1.7->-1.4 reduced frozen-top clearance0.331330->
0.133045 mm and was rejected; this proposal retains original -1.7.

Mandatory integration before release: parts/features/components/fasteners
(F16,F17,N07,N08), assembly stages and actual driver/head envelopes; L2
journal/bore/shoulder/key witnesses; L3 separate inner/outer race contacts and
positive clamp-relief gaps; L4 assembly/insertion; L5 screw stack, bottoming
and tool reach; L6 every new solid + full bearings, all coupled poses and
fresh fingerprints; keepouts including KO07/KO09/CF3, printability/tolerances,
BN08/BN18 drag/preload/temperature/creep, mass/inertia/MJCF and release lists.
The hardcoded old N04 stop/guide and A plastic-slide assertions must be
replaced with the new invariants, not deleted or waived to obtain green.

Local primitive checks2026-09-17 (not a complete-build release): five new
solids are watertight/single-component;15 new-solid/steel pairs and3 moving
interface pairs have zero overlap to numerical precision. Nominal CF3
protrusion2.0 gives3.600/2.600/4.842 mm gaps to A steel/uncut seat/F17
heads+shafts; adopted protrusion3.0 sensitivity gives2.600/1.600/4.842.
Unknown actual protrusion and cable bend radius are NOT replaced by these
sensitivity inputs. Minimum nominal clamp-bypass gaps: A0.300/yaw0.300/
B0.195 mm. Full N03 final-cut connectivity, sliced sections, all assembly
stages and coupled poses must be rerun after integration.
"""
from . import s288
from .s288 import S, cyl, union, diff, inter, placed
from .lib import sfw, drv_self, pt, wbox, hull, flange_relief, keep_main

HEAD_ROLL_Z = float(pt(drv_self('jaw_soft'), 0)[2])
HEAD_A = dict(x0=6.4, x1=10.4, od=27.0, bore=20.0, journal_d=19.85,
              shoulder_d=21.6, shaft_front=5.6, end_relief=0.30,
              seat_d=27.15, seat_od=31.2, float_x0=5.6, float_x1=11.4,
              seat_x0=5.6, seat_x1=11.4)
HEAD_YAW = dict(z0=220.0, z1=224.0, od=27.0, bore=20.0,
                journal_d=19.85, shoulder_d=21.6, shoulder_z0=219.0,
                seat_d=27.15, seat_od=30.15, lip_id=25.0,
                cap_od=31.15, ear_r=20.0, ear_d=6.0,
                # hr39c（2026-09-23 头等比放大 +10）：两颗 N08→N03 的 M2 耳由 ±y（90°/−90°）改到 120° / −60°（绕偏航轴 (26,0)、从 +x 量），仍对顶 180°。
                #   原 +y 耳 (26, 20) 连同 N08 盖耳绕头横滚扫到 y≈25.6、z≈235.6，正压在新嘴舵机（jaw.py，嘴轴 (31.75, 245.5)）本体后下角与下插座上；
                #   hr39c 重搜（hr39c_work/jaw_scan*.log）：不挪这颗耳，头放大 +10 内嘴舵机 0 个可行摆法。120° 那颗在 (16, 17.3)，离嘴舵机最近处 x 22.3 还有 3.3；
                #   −60° 那颗 (36, −17.3) 离 B 端扫掠刀 x≥39.9 / N08 修边 x≥41 都 ≥2。旧 ±y 值留在 ear_angles_deg_was_until_2026_09_23_hr39c。
                ear_angles_deg=(120.0, -60.0), ear_angles_deg_was_until_2026_09_23_hr39c=(90.0, -90.0))

def ear_xy(theta_deg, r=None):
    """耳心（绕偏航轴 (26, 0)，从 +x 逆时针量 θ）"""
    import math
    r = HEAD_YAW['ear_r'] if r is None else r
    t = math.radians(theta_deg)
    return 26.0 + r * math.cos(t), r * math.sin(t)

def _radial_bar(r0, r1, w, z0, z1, theta_deg):
    """沿偏航轴径向 θ 的条（半径 r0..r1、宽 w、z0..z1）"""
    import math
    from trimesh.transformations import rotation_matrix as _rot
    b = wbox((r0, -w / 2, z0), (r1, w / 2, z1))
    return placed(b, _tr(26.0, 0.0) @ _rot(math.radians(theta_deg), [0, 0, 1], [0, 0, 0]))

def _tr(x, y):
    import numpy as _np
    T = _np.eye(4); T[0, 3] = x; T[1, 3] = y; return T
N04_KEY = dict(x0=37.35, x1=38.9, width=2.0, radial_top=9.0,
               slot_width=2.30, slot_top=9.15)

def _roll_cyl(d, x0, x1):
    return cyl(d, x1-x0, ((x0+x1)/2, 0, HEAD_ROLL_Z), axis='x', sections=128)

def _yaw_cyl(d, z0, z1):
    return cyl(d, z1-z0, (26.0, 0, (z0+z1)/2), sections=128)

def _ring_x(od, bore, x0, x1):
    return diff(_roll_cyl(od,x0,x1),_roll_cyl(bore,x0-.02,x1+.02))

def _ring_z(od, bore, z0, z1):
    return diff(_yaw_cyl(od,z0,z1),_yaw_cyl(bore,z0-.02,z1+.02))

def _horn_holes(R):
    # Generator is local; every instance MUST pass through placed().
    return placed(s288.horn_holes(20.0, (s288.x_flange_face()+4.,0,0)),R)

def roll_A_separation_cut():
    # Remove the former integral PLA disc/journal; N05 now supplies the shaft.
    return _roll_cyl(31.3,5.0,HEAD_A['x1'])

def build_head_roll_adapter():
    a=HEAD_A; R=drv_self('jaw_soft')
    journal=_roll_cyl(a['journal_d'],a['x0']-.02,a['x1']-a['end_relief'])
    shoulder=_roll_cyl(a['shoulder_d'],a['shaft_front'],a['x0'])
    return keep_main(diff(union(journal,shoulder),flange_relief(R),_horn_holes(R)),
                     'head_roll_adapter')

def head_yaw_base(m):
    # N02 under-head face is ZF-5.25; its central plate is 2.50 mm thick.
    zf=float(pt(sfw('yaw_roll_motion',0),s288.x_flange_face())[2])
    base_top=zf-2.75
    shoulder=_yaw_cyl(HEAD_YAW['shoulder_d'],HEAD_YAW['shoulder_z0'],HEAD_YAW['z0'])
    # Keep the original tool passage and holes open through the added shoulder.
    shoulder=diff(shoulder,_yaw_cyl(15.05,218.,221.))
    return union(diff(m,_yaw_cyl(24.,base_top,zf+2.)),shoulder)

def build_head_yaw_adapter():
    h=HEAD_YAW; R=sfw('yaw_roll_motion',0)
    zf=float(pt(R,s288.x_flange_face())[2]); core0=zf-2.75+.30
    core=_yaw_cyl(h['shoulder_d'],core0,zf-1.)
    # Upper chamfer clears N04's lower corner; top diameter stays at 19.85.
    chamfer=hull(_yaw_cyl(h['shoulder_d'],zf-1.,zf-.98),
                 _yaw_cyl(h['journal_d'],zf-.02,zf))
    lip=_ring_z(h['shoulder_d'],20.05,h['z1'],core0+.02)
    return keep_main(diff(union(core,chamfer,lip),_horn_holes(R)), 'head_yaw_adapter')

def yaw_pilot_holes():
    # Also re-applied by build_yrm AFTER union with its original blank.
    return union(*[cyl(1.7,5.4,(*ear_xy(t),224.),sections=128)
                   for t in HEAD_YAW['ear_angles_deg']])

def yaw_carrier_features():
    h=HEAD_YAW
    wall=_ring_z(h['seat_od'],h['seat_d'],220.2,h['z1']+.02)
    upper=_ring_z(h['seat_od'],h['lip_id'],h['z1'],225.0)
    # Initial roots: preserve these real overlaps and widen/blend if strength
    # requires. Their presence does not itself certify minimum section/load.
    roots=[wbox((12.5,-7.,222.),(15.5,7.,232.5)),
           wbox((36.4,8.6,222.),(39.5,11.4,235.0)),
           wbox((36.4,-11.4,222.),(39.5,-8.6,235.0))]
    ears=[]; holes=[]
    for t in h['ear_angles_deg']:          # hr39c：耳挪到 120° / −60°（见 HEAD_YAW 注释），形状照旧（Ø6 耳 + r 14.2..20 宽 6.4 的连接条）
        ex, ey = ear_xy(t)
        ears.append(cyl(h['ear_d'],5.7,(ex,ey,224.45),sections=128)) #221.6..227.3
        ears.append(_radial_bar(14.2, h['ear_r'], 6.4, 221.6, 227.3, t))
        holes.append(cyl(1.7,5.4,(ex,ey,224.),sections=128)) #221.3..226.7
    # Roots must also respect the full bearing volume, not only the annular
    # wall. Bore stops at z1 so the upper OUTER-race shoulder survives.
    seat_clearance=_yaw_cyl(h['seat_d'],219.5,h['z1'])
    # N06 and N02 rotate inside these roots. Match the main N03 clearance;
    # the upper bearing shoulder remains only on the outer-race annulus.
    journal_clearance=_yaw_cyl(h['lip_id'],h['z1'],228.2)
    # The perpendicular roll-B steel reaches z224.6146. Clear it by .30
    # radially/axially; x>=39.9 stays outside the yaw steel (max x39.5), so
    # this does not cut the yaw outer-race contact annulus.
    roll_B_clearance=_roll_cyl(22.6,39.9,44.5)
    # hr09：N06（Ø21.6）从空偏航舵机腔向下装入/向上取出要穿过 z 228.2..232 —— 根 x 12.5..15.5 的内面 r 10.5 挡 0.3（hr08 头部审计 N06 stage 1.7 mm³）
    n06_path=_yaw_cyl(22.4,228.0,232.5)
    return diff(union(wall,upper,*roots,*ears),seat_clearance,journal_clearance,n06_path,
                roll_B_clearance,*holes)

def build_head_yaw_cap():
    h=HEAD_YAW
    # Entire cap and screw heads remain above218.7; N02's old top box
    # reaches218.215. Ears point +/-y, avoiding the new large roll-A bearing.
    plate=_ring_z(h['cap_od'],h['lip_id'],218.7,219.7)
    # Only the outer race is loaded. Main plate stays below the carrier wall.
    land=_ring_z(h['od'],h['lip_id'],219.68,h['z0'])
    ears=[]; holes=[]
    for t in h['ear_angles_deg']:          # hr39c：耳挪到 120° / −60°，形状照旧
        ex, ey = ear_xy(t)
        ears.append(cyl(h['ear_d'],1.1,(ex,ey,220.85),sections=128)) #220.3..221.4
        # Inner arm stays below the bearing seat wall. Raise only the outer
        # segment, outside seat_od/2, to preserve the ear's .20 mm relief.
        ears.append(_radial_bar(14.2, h['ear_r'], 6.4, 218.7, 219.7, t))
        ears.append(_radial_bar(16.0, h['ear_r'], 6.4, 219.68, 221.4, t))
        holes.append(cyl(S['mnt_hole_d'],5.,(ex,ey,220.),sections=128))
        # Flat under-head plane220.3; M2x6 reaches226.3 (4.7 mm in receiver).
        holes.append(cyl(4.4,4.,(ex,ey,218.3),sections=128))
    # hr11：盖盘 Ø31.15 的 +x 边到 x 41.575，正压在 H03 B 端下半座（x≥41.3, r≥11.08 → z≥224.5）正下方，H03 沿 −z 拆卸撞 0.68 mm³；削平到 x≤41.0（该处外圈接触环 Ø27 外仍有 ≥2.5 壁）
    trim=wbox((41.0,-40.,215.),(60.,40.,225.))
    # hr11 试算：−x 边（x 10.43）在 H03 A 座下半环（x 5.6..11.4，环底 z 220.0）正下方 0.3，H03 −z 拆卸擦 0.075 mm³ → 削平到 x≥11.6
    trim2=wbox((0.,-40.,215.),(11.6,40.,225.))
    return keep_main(diff(union(plate,land,*ears),*holes,trim,trim2),'head_yaw_cap')

def yaw_motion_envelope():
    h=HEAD_YAW
    steel=_ring_z(h['od'],h['bore'],h['z0'],h['z1'])
    return union(yaw_carrier_features(),build_head_yaw_cap(),steel,
                 build_head_yaw_adapter())

def journal_key_slot():
    k=N04_KEY
    # Enlarge the existing tool passage, never put a flat across its aperture.
    return wbox((32.,-k['slot_width']/2,HEAD_ROLL_Z+7.75),
                (39.,k['slot_width']/2,HEAD_ROLL_Z+k['slot_top']))

def keyed_journal(m):
    k=N04_KEY
    key=wbox((k['x0'],-k['width']/2,HEAD_ROLL_Z+7.70),
             (k['x1'],k['width']/2,HEAD_ROLL_Z+k['radial_top']))
    # N04 tip 44.0 vs cap bearing face 44.2 leaves a clamp-bypass gap.
    pilot=_roll_cyl(1.7,38.7,44.1)
    return keep_main(diff(union(m,key),pilot),'head_journal')

def build_head_bearing_cap():
    # Outside diameter ends before the H01 foot. Only the smaller center boss
    # and screw continue through the front tool aperture (old min radius 6.79).
    plate=_roll_cyl(17.6,44.2,45.7)
    boss=_roll_cyl(12.0,45.6,47.2)
    return keep_main(diff(union(plate,boss),_roll_cyl(S['mnt_hole_d'],44.1,47.3)),
                     'head_bearing_cap')

def head_seats(m,upper):
    a=HEAD_A
    # A is the non-locating bearing: a full-length through seat, NO thin
    # thrust lips. B carries axial location. Printed seat fit remains unknown.
    A=_ring_x(a['seat_od'],a['seat_d'],a['seat_x0'],a['seat_x1'])
    # B: add the missing -x outer-race stop; retain the existing +x lip.
    # N03 large boss now ends x39.0, before this lip starts at39.3.
    B=union(_ring_x(24.2,20.8,39.3,40.1),_ring_x(24.2,22.18,40.0,41.0))
    half=wbox((-100.,-100.,HEAD_ROLL_Z if upper else 100.),
              (120.,100.,350. if upper else HEAD_ROLL_Z))
    other=wbox((-100.,-100.,100. if upper else HEAD_ROLL_Z),(120.,100.,HEAD_ROLL_Z if upper else 350.))
    # hr11：下半（H03）只加 A 座下半环，且两侧削平到 |y|≤15.05 —— H01 侧板内面 y=15.3 在分型面下 3–4 mm 处，H03 沿 −z 拆卸时 A 环外壁（r 15.6）撞它 0.93 mm³；
    #        B 端下半不加环：外圈后挡只在 H01（head_specs outer_rear_H01_ring_float0p1），而 x 39.3..41 下半正对 N03 偏航座墙（x≤41.08, z≤225）与 N08 盖边。
    ybox=wbox((-100.,-15.05,100.),(120.,15.05,350.))
    upper_add=inter(union(A,B),wbox((-100.,-100.,HEAD_ROLL_Z),(120.,100.,350.)))
    lower_add=inter(inter(A,wbox((-100.,-100.,100.),(120.,100.,HEAD_ROLL_Z))),ybox)
    additions=upper_add if upper else lower_add
    # hr08：对方半座（另一件）在本件里让位 0.2 —— H01 本体在 A 座下半环处本来有料（hr08 头部试build H01×H03 20.5 mm³）。
    from .lib import minkowski_box
    # was_until_2026_09_28_hr50: others=minkowski_box(inter(union(A,B),other),(-.2,)*3,(.2,)*3)   # 让位仍按 A+B 全环的对方半（hr11 试算：只按 A 下半让位时 H01 在 −25° 与 N03 贴到 0）
    # hr50（用户：薄片默认收刀留 ≥0.8）：H01（upper）对 A 下半环的让位按整环 r 15.6 +0.2 切，可 H03 的 A 下半环实际削平到 |y| ≤ 15.05（上面 ybox）——
    #   H01 原版侧板（y ±15.3..16.2）被多切到只剩 0.4（(7.45, 15.96, 234.9) 薄膜 15.4 mm²）。A 部分收到 H03 真实的环（∩ ybox）+0.2，B 仍按全环（hr11 的 N03 −25° 余量靠它）。
    others=minkowski_box(inter(union(inter(A,ybox) if upper else A,B),other),(-.2,)*3,(.2,)*3)
    cutters=[_roll_cyl(a['seat_d'],a['seat_x0']-.02,a['seat_x1']+.02)]
    return union(diff(m,*cutters,others),diff(additions,*cutters))
