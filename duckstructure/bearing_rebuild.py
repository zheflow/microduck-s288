"""髋横滚 6703 副支撑（hr08，2026-09-17）—— L01 整体座，不用可拆座/压盖/耳。

Codex hr01–hr05 的方案（可拆 L09 座 + L08 压盖 + L01 两侧根/外侧耳，舵机沿轴后退 4.2）在 hr07 审计里被证伪：
  · L01 外侧根（r 16.7）和座耳（r 17.3）绕横滚轴 ±24° 扫掠到世界 y 34.6–35.2，切掉 L02 俯仰盘上 F06 三颗螺丝的坐面
    （hr07_plate_diag：roll_module_sweep 168 mm³，坐面 60°/120° 只剩 6/72 点）；横滚轴到该坐面只有 17.0 mm，座壁 12.8 + 耳 ≥4.4 放不下；
  · 上方 T01 偏航座环/偏航舵机、内侧另一条腿、正下方舵机长端 —— 真实姿态扫描（roll_ear_realposes / roll_ear_vs_f06）无耳位可用。
改：座环是 L01 的整体件（与膝/髋俯仰同做法），站位在法兰顶面（13.0）+0.2 之外：挡肩 13.3..13.8、钢圈 13.8..17.8、座壁到 17.9（世界 15.6..19.7；
Codex 的 13.3..17.3 是可拆座才能占的位）；顶部开 60° 让 T01 偏航座环；座环靠一块顶桥和一块外侧桥接到 L01 杯顶板角（两块的 ±24° 扫掠外沿都 <
F06 坐面 y 34.5）；舵机仍沿世界 −z 侧滑装入（SERVO_SHIFT 4.2 保留）。内圈由 L10 轴套（包法兰）与 L02 Ø18.2 压面夹紧（6×M2×10，叠 7.5），
外圈 −x 靠座唇，+x 无压盖（膝同级，bench BN08）。配合值为 CAD 采用值。"""
import math
import numpy as np
import trimesh

from . import s288
from .s288 import cyl, bx, union, diff, placed
from .lib import P, S, sfw, wbox, clean_print_topology

XF = s288.x_flange_face()                              # 13.0
HR = dict(od=23., bore=17., width=4., x0=13.8,
          journal_d=16.85, inner_face_d=18.2, sleeve_x0=10.5, sleeve_shoulder=(12.8, 13.8), sleeve_end=17.5,
          seat_d=23.15, seat_od=25.55, lip_d=22.2, shoulder_x0=13.3, seat_x1=17.9, open_top_deg=60.,
          land_d=18.2, land_x1=18.4, plate_d=20., plate_x0=18.4, plate_x1=21.3, horn_seat=XF + 7.5, screw_stack=7.5, horn_screw_l=10.,
          top_bridge=dict(x0=10.5, x1=13.4, y=9.5, z0=10.3, z1=12.1),          # 接 L01 杯顶板两角；顶 z 12.1 = 世界 114.6 < T01 座环区 114.65
          side_bridge=dict(x0=11.0, x1=13.4, y0=7.0, y1=12.0, z0=8.0, z1=12.0),  # 外侧桥：角点 (12,12) 绕横滚轴 24° → y 15.9+0.4 < 17.0
          side_root=dict(x0=-13.6, x1=13.4, y0=10.3, y1=12.3, z0=-6.0, z1=6.0),   # 外侧根：接背板（舵机后退 4.2 后原杯够不到背板，hr08 试build 背板 175.9 mm³ 分离）；角点 (12.3,6)→ y 13.6+0.4
          back_link=dict(x0=-14.0, x1=-10.5, y0=8.0, y1=12.3, z0=-4.0, z1=6.0),   # 背板（y ±10.3，x −14..−10.3）与外侧根只共面不搭接 → 在舵机薄段背(−10.0)之后加一块搭接
          medial_notch=dict(x0=17.0, y_max=-8.0, z=8.0),                          # 座壁最后 0.8 mm 内侧 ~100° 扇区切掉：roulade 内收极限左右 L01 座壁前端角相碰 1.19 mm³（hr08_diag）
          sleeve_clearance=(18.05, 19.4))                                          # 座前的原杯残料对 L10 轴套(Ø16.85)/轴套肩(Ø18.2)让位 0.6
assert HR['shoulder_x0'] >= XF + 0.2 + 0.05, "座挡肩必须在法兰顶面+0.2 侧滑余量之外"
assert HR['sleeve_end'] <= HR['x0'] + HR['width'] - 0.3 + 1e-9

def top_sector(x0, x1, opening_deg=None):
    a = math.radians((HR['open_top_deg'] if opening_deg is None else opening_deg) / 2)
    points = [(x, y, z) for x in (x0, x1) for y, z in ((0., 0.), (50 * math.sin(a), 50 * math.cos(a)), (-50 * math.sin(a), 50 * math.cos(a)))]
    return trimesh.convex.convex_hull(np.asarray(points))

def _cx(d, x0, x1): return cyl(d, x1 - x0, ((x0 + x1) / 2, 0, 0), axis='x', sections=128)
def _ring(od, bore, x0, x1): return diff(_cx(od, x0, x1), _cx(bore, x0 - .02, x1 + .02))

def hip_roll_seat_local(with_bridges=True):
    """并入 L01 的座毛坯（之后用 hip_roll_seat_cut_local 挖孔/开顶）。"""
    h = HR
    parts = [_cx(h['seat_od'], h['shoulder_x0'], h['seat_x1'])]
    if with_bridges:
        t = h['top_bridge']
        parts.append(bx((t['x1'] - t['x0'], 2 * t['y'], t['z1'] - t['z0']), ((t['x0'] + t['x1']) / 2, 0., (t['z0'] + t['z1']) / 2)))
        for s in (h['side_bridge'], h['side_root'], h['back_link']):
            parts.append(bx((s['x1'] - s['x0'], s['y1'] - s['y0'], s['z1'] - s['z0']), ((s['x0'] + s['x1']) / 2, (s['y0'] + s['y1']) / 2, (s['z0'] + s['z1']) / 2)))
    return union(*parts)

def hip_roll_seat_cut_local():
    h = HR; n = h['medial_notch']; c1, c2 = h['sleeve_clearance']
    return union(_cx(h['seat_d'], h['x0'], h['seat_x1'] + .1),
                 _cx(h['lip_d'], h['shoulder_x0'] - .1, h['x0']),
                 top_sector(h['shoulder_x0'] - .1, h['seat_x1'] + .1),
                 _cx(c1, S['T'] / 2 + .1, h['shoulder_x0'] + .01),                 # 轴套让位（舵机顶面 10.3 → 挡肩）
                 _cx(c2, h['sleeve_shoulder'][0] - .2, h['shoulder_x0'] + .01),   # 轴套肩让位
                 bx((h['seat_x1'] + 1. - n['x0'], 30., 2 * n['z']), ((h['seat_x1'] + 1. + n['x0']) / 2, n['y_max'] - 15., 0.)))   # 内侧前端缺口

def hip_roll_module_local(steel=True):
    """固定在 L01 上的整套（**镗孔后**的座环+桥[+钢圈]）：给 L02 的反向扫掠刀 / 远端让位刀。"""
    m = diff(hip_roll_seat_local(), hip_roll_seat_cut_local())
    if steel: m = union(m, _ring(HR['od'], HR['bore'], HR['x0'], HR['x0'] + HR['width']))
    return m

def hip_roll_sleeve_local():
    """L10：独立法兰轴套。后肩 → 内圈 → L02 压面 是轴向夹紧路径；尖端比压面短 0.3。"""
    h = HR
    tube = _cx(h['journal_d'], h['sleeve_x0'], h['sleeve_end'])
    shoulder = _cx(h['inner_face_d'], *h['sleeve_shoulder'])
    relief = _cx(S['flange_d'] + .6, h['sleeve_x0'] - .1, XF)
    return diff(union(tube, shoulder), relief, s288.horn_holes(20., (16., 0., 0.)))

def hip_roll_driven_local():
    """L02 横滚侧：Ø18.2 压面（只压内圈）+ Ø20 舵盘板。"""
    h = HR
    return union(_cx(h['land_d'], h['x0'] + h['width'] - .01, h['land_x1']), _cx(h['plate_d'], h['plate_x0'], h['plate_x1']))

def hip_roll_old_clearance_local():
    """清 L02 原版毛坯/XL330 填柱：到钢圈端面整柱清，端面..压面段只清 Ø18.3 以外。"""
    h = HR; xe = h['x0'] + h['width']
    return union(_cx(27.5, -1., xe), _ring(27.5, h['land_d'] + .1, xe - .01, h['plate_x0'] + .01))

def build_hip_roll_sleeve():
    return clean_print_topology(placed(hip_roll_sleeve_local(), sfw('yaw2roll', 0)))
