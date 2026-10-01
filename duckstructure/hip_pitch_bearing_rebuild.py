"""髋俯仰 6703 副支撑（hr07，2026-09-17）—— 整体座，不用可拆座/压盖。

Codex 的 hr06 方案（6704 + 可拆座 L12 + 压盖 L11 + 下扇区两耳）在 build 后被证伪：
  · 耳/根在下扇区 r≈22.7，膝 ≥72.5° 踝舵机顶面（世界 z 81..89）撞耳，85° 380 mm³；训练限位 86.2°、原版策略用到 83.1°；
    L03 上的螺丝根被 09-16 的膝弯 60..90° 小腿扫掠刀削成 2 mm 薄片（pitch_ear_study / pitch_ear_realposes 两轮扫描，
    docs/design_2026-09-17_bearing_rebuild/）；
  · 上半圈耳位在真实姿态（横滚 +24 / 偏航 −25 组合）顶进 T01 / 偏航舵机 / 左壳；后扇区（舵机长端）耳根伸进舵机机身；
  · 6704 座壁 r 15.6 本身 ≥85° 就被踝舵机擦到。
改：与膝 L03 座同一做法——外圈座是 L03 的整体凸台（`ring_boss` 思路），舵机仍沿世界 −z 侧滑装入（法兰 x≤13.0 在座挡肩 13.3 之后）；
6703（17×23×4）钢圈 x 14..18（法兰面 13 外 1.0），座壁 r 12.8（膝 87.5° 踝舵机顶 z 89.3 vs 座底 89.7）；
内圈由独立轴套 L13（包法兰，6 颗 F06 M2×10 穿 L02 压盘→轴套→法兰）与 L02 Ø18.2 压面夹紧，轴套尖端比压面短 0.3；
外圈 −x 由座挡肩（Ø22.2 唇）止，+x 无压盖 —— 与膝 6704 同级（靠舵机轴向堆叠/座配合，bench BN08 实测旷量）。
座与 L03 本体的连接：腹板走局部 z ≤ −11（世界 x ≤ −7，踝舵机膝弯带在 z −10..3 之外）到 L03 的后下块。
配合值（seat_d 23.15 / journal 16.85 / lip 22.2）沿用 bearing_rebuild.HR，均为 CAD 采用值，待 PETG 试件。"""
from . import s288
from .s288 import S, cyl, bx, union, diff, placed
from .lib import P, drv_self, clean_print_topology, keep_main

XF = s288.x_flange_face()                          # 13.0
HP = dict(od=23., bore=17., width=4., x0=14.0,
          journal_d=16.85, inner_face_d=18.2, sleeve_x0=10.5, sleeve_shoulder=(13.0, 14.0), sleeve_end=17.7,
          seat_d=23.15, seat_od=25.55, lip_d=22.2, shoulder_x0=13.3, seat_x1=18.1,
          land_d=18.2, land_x1=18.6, plate_d=21.6, plate_x0=18.6, plate_x1=21.5, horn_seat=XF + 7.5, horn_screw_l=10.,
          web=dict(x0=13.3, x1=18.1, y0=-2.0, y1=16.0, z0=-22.0, z1=-11.0))     # 腹板（局部 y 向下=世界 −z，z 向前=世界 +x）
assert HP['shoulder_x0'] >= XF + 0.2 + 0.05, "座挡肩必须在侧滑装入的法兰顶面(13.0)+0.2 之外"
assert HP['sleeve_end'] <= HP['x0'] + HP['width'] - 0.3 + 1e-9, "轴套尖端必须比 L02 压面短 ≥0.3（塑料不许先顶死）"

def _cx(d, x0, x1): return cyl(d, x1 - x0, ((x0 + x1) / 2, 0, 0), axis='x', sections=128)
def _ring(od, bore, x0, x1): return diff(_cx(od, x0, x1), _cx(bore, x0 - .02, x1 + .02))

def pitch_seat_local(with_web=True):
    """L03 上的整体座毛坯（并入本体后再用 pitch_seat_cut_local 挖孔）。"""
    h = HP
    wall = _cx(h['seat_od'], h['x0'] - .02, h['seat_x1'])
    shoulder = _cx(h['seat_od'], h['shoulder_x0'], h['x0'])
    parts = [wall, shoulder]
    if with_web:
        w = h['web']
        parts.append(bx((w['x1'] - w['x0'], w['y1'] - w['y0'], w['z1'] - w['z0']),
                        ((w['x0'] + w['x1']) / 2, (w['y0'] + w['y1']) / 2, (w['z0'] + w['z1']) / 2)))
    return union(*parts)

def pitch_seat_cut_local():
    """座孔 + 唇孔 + 舵机侧滑不碰的挡肩内腔；并集之后切，防毛坯回填。"""
    h = HP
    return union(_cx(h['seat_d'], h['x0'], h['seat_x1'] + .1),          # 座孔从 x0 整起：唇面 = 钢圈后端面（−0.02 会留缝，审计 rear face contact 0）
                 _cx(h['lip_d'], h['shoulder_x0'] - .1, h['x0']))

def pitch_module_local(steel=True):
    """固定在 L03 上的整套（**镗孔后的**座环+腹板[+钢圈]），给 L02 的反向扫掠刀 / 远端让位刀用。
    必须用镗孔后的座：实心毛坯当刀会把座孔里的 L02 压面/L13 一起扫掉（hr07 局部试build 前接触 0 的原因）。"""
    m = diff(pitch_seat_local(), pitch_seat_cut_local())
    if steel: m = union(m, _ring(HP['od'], HP['bore'], HP['x0'], HP['x0'] + HP['width']))
    return m

def pitch_sleeve_local():
    """L13：独立法兰轴套。后肩 → 内圈 → L02 压面 是轴向夹紧路径；尖端比压面短 0.3。"""
    h = HP
    tube = _cx(h['journal_d'], h['sleeve_x0'], h['sleeve_end'])
    shoulder = _cx(h['inner_face_d'], *h['sleeve_shoulder'])
    relief = _cx(S['flange_d'] + .6, h['sleeve_x0'] - .1, XF)
    return diff(union(tube, shoulder), relief, s288.horn_holes(20., (16., 0., 0.)))

def pitch_driven_local():
    """L02 侧：Ø18.2 压面（只压内圈）+ Ø21.6 舵盘板（离外圈端面 0.6）。"""
    h = HP
    return union(_cx(h['land_d'], h['x0'] + h['width'] - .01, h['land_x1']), _cx(h['plate_d'], h['plate_x0'], h['plate_x1']))

def pitch_old_journal_clearance_local():
    """清掉 L02 原版一体轴颈/XL330 填柱：到钢圈端面(18.0)整柱清；18.0..18.6 只清 Ø18.3 以外（留 Ø18.2 压面，外圈对面(Ø19..23)不许有料）。"""
    h = HP; xe = h['x0'] + h['width']
    return union(_cx(27.5, -1., xe), _ring(27.5, h['land_d'] + .1, xe - .01, h['plate_x0'] + .01))

def build_hip_pitch_sleeve():
    return clean_print_topology(placed(pitch_sleeve_local(), drv_self('upper_leg_left')))
