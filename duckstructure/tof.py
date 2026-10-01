"""hr52（2026-09-29 用户定「v6-VL53L5CX 雷达」贴面版 + 板底实心）：头部 ToF 模块的共用几何。
H04 脸板上的 ToF 特征、H10 压条、zz_tof 电子件占位都从这里取常量，免得几处数对不上。

模块：通用 VL53L5CX 板「VL53L5X V2」（都会明武电子 ¥44.5，item 737603917521 选「VL53L5X V2模块」，不要盖片；
  components.yaml:tof_vl53l5cx；商品页截图 docs/design_2026-09-17_bearing_rebuild/tof_预检_2026-09-29/商品页截图_2026-09-29/）。
  商品页尺寸图（卖家手工测，仅参考）：总长 24.7、本体 15.8 × 10、耳宽 5.2 → 孔距 19.5 = 24.7 − 2 × 2.6（推算）。
  **assumed，到货卡尺量完再定**：孔 Ø2.2、板厚 1.6、排针行中心离板中线 3.0（模型；商品 3D 图估 ≈3.8–4.0）、芯片中心离板中线 −2.2（hr52c 按商品正面图；原 −1.5）（芯片 6.4 × 3.0 × 1.5 按 ST 规格）、
  背面元件包络高 1.5（商品图背面两颗 SOT-23 + 阻容）。
  引脚（正面 = 芯片面看、排针在上）：左 → 右 LPn INT SDA SCL GND VIN（商品图丝印）；杜邦壳只插右边 4 个（SDA SCL GND VIN），INT / LPn 空着。
  商家焊 6P 直排针：塑料座在背面（芯片反面），针朝 −x（头里）；芯片面针脚**不剪**（hr52c 用户 09-30 定；原「剪平、焊点 ≤0.6」作废）→ H04 开排针通透长条。
位置（tof_预检_2026-09-29/tilt_scan.py → v6_pick.json）：鸭子左脸（+y）、眼睛右上方，板正面贴 H04 脸板内面；
  板绕 x 斜 27.5°（正面看左高右低；水平摆时杜邦壳撞转接板 XT30 扩展口插头 176 mm³），板心 (y 28.0, z 274.0)。
板系 (x, a, b)：x = 世界 x；a 沿板长（正面看朝右 → 世界 +y 偏下），b 沿板宽（排针那一侧 = +b → 世界 +z 偏 +y）。
  先在板系按世界轴向建（x, a, b 当 x, y, z），再 to_world() 绕 x 转 −27.5° 并平移到板心。
"""
import math
import numpy as np
from trimesh.transformations import rotation_matrix as _rot, translation_matrix as _tr
from .s288 import cyl, union, diff, placed
from .lib import wbox

THETA, YC, ZC = 27.5, 28.0, 274.0            # v6_pick.json
X_IN, X_OUT = 92.17, 93.47                   # = head.H04_PLATE_IN、H04_PLATE_IN + 1.3（脸板内面 / 外面）
PAD = 0.2                                    # 实心平台厚：板正面 91.97 → 芯片顶 91.97 + 1.5 = 93.47 = 脸面齐平
PCB_T = 1.6                                  # assumed
XF = X_IN - PAD                              # 板正面 91.97
XB = XF - PCB_T                              # 板背面 90.37
BODY_A, BODY_B = 7.9, 5.0                    # 本体半长 / 半宽（15.8 × 10，卖家手量）
EAR_D, HOLE_A, HOLE_D = 5.2, 9.75, 2.2       # 耳宽（卖家手量）/ 孔心 a（孔距 19.5 推算）/ 孔径（assumed）
CHIP = (6.4, 3.0, 1.5)                       # 芯片 a × b × 高（ST VL53L5CX 封装 6.4 × 3.0 × 1.5）
CHIP_B = -2.2                                # 芯片中心 b —— hr52c 按商品图量（仍 assumed，到货卡尺定）：正面俯视图（盖片示意，比例按芯片 6.4 × 3.0 标）芯片外沿离非排针长边 ≈1.25–1.3 → 中心离边 2.8 → b −2.2（复审 r2 独立量正面图 −2.18 … −2.28；3D 图读数偏 +b 0.2–0.4、−a 0.3–0.4，透视误差大，窗按两者都装得下放余量）
CHIP_B_was_until_2026_09_30_hr52c = -1.5     # 原 assumed「芯片在排针行和板边之间」：照它开的窗，商品图位置的芯片会顶窗下沿 ≈0.45，板贴不平
PIN_B, PITCH, N_PIN = 3.0, 2.54, 6           # 排针行中心 b（assumed）/ 针距 / 针数 —— hr52c：商品 3D 图（截图 04，按两条长边定标）量得焊盘心离排针侧长边 ≈1.0 → b ≈ 4.0；
#   模型仍按 3.0（线束 ToF 端点按它走；排针行 3.0 … 4.3 背面杜邦壳都不碰件、离 H05 ≥1.37（b 4.3 时 1.367），hr52_work 原型核过），H04 开口按 3.0 … 4.3 放宽；到货量完再定
CHIP_A = 0.0                                 # 芯片中心 a（assumed：沿板长居中；商品正面俯视图芯片在 SDA / SCL 两针正中）—— 复审 r1 m9：原来是写死的隐含假设，补成常量；到货量
PIN_NAMES = ("LPn", "INT", "SDA", "SCL", "GND", "VIN")    # k = 0..5 ↔ a = −6.35 + 2.54 k（正面看左 → 右）
USED = ("SDA", "SCL", "GND", "VIN")          # 插杜邦壳的 4 根 → Radxa 3 / 5（与 IMU 一分二共用）/ 6 / 1
HDR = dict(plastic=2.5, pin_back=6.0, pin_w=0.64, solder_front=1.4)   # 通用 2.54 直排针（短脚 3.0 + 塑座 2.5 + 长脚 6.0 ≈ 总长 11.5）：塑座贴背面、长脚露 6.0 朝头里；
#   hr52c：芯片面针脚**不剪**（用户 09-30：怕剪坏）→ 短脚 3.0 − 板 1.6 = 1.4 冒出芯片面（尖 x 93.37，离脸面 0.1）+ 焊锡包 → H04 开通透长条 PIN_SLOT
HDR_was_until_2026_09_30_hr52c = dict(plastic=2.5, pin_back=6.0, pin_w=0.64, solder_front=0.6)   # 原：芯片面焊点剪到 ≤0.6（窄槽 0.7 深，用户估计商家做不到）
DUP = dict(L=14.0, w=2.54)                   # 杜邦母壳（同 electronics.GPIO_DUP 实量 2.54 × 14）：套到底顶住塑座
BACK_COMP_H = 1.5                            # 背面元件包络高（assumed）

# ── H04 脸板上的 ToF 特征
MARGIN = 0.3                                 # 实心平台 = 板投影外扩 0.3
GROOVE = dict(depth=0.7, b0=2.1, b1=5.6, over_a=1.2)       # hr52c：焊点槽 0.7 深（焊锡包底座 r ≤0.9）按 b 范围开：排针行 b 3.0（原 assumed）… 3.9（商品图估 3.6–3.8 + 余量）
#   → b 2.1 .. 5.6（+b 侧穿出平台边 5.3，不留细边）；两端仍到 ±8.82（穿出平台本体边 ±8.2）
GROOVE_was_until_2026_09_30_hr52c = dict(depth=0.7, half_b=1.3, over_a=1.2)   # 以 PIN_B 3.0 为中心 ±1.3（b 1.7 .. 4.3），底下留 0.8
PIN_SLOT = dict(a_half=7.25, b0=2.4, b1=4.9, r=0.4)       # hr52c 新：排针通透长条（穿脸面，14.5 × 2.5，圆角 0.4）—— 针（半对角 0.45）+ 焊锡包 0.7 以上（r ≤0.6）：
#   针心 b 3.0 … 4.3、沿 a 整排偏 ±0.3 都落在条里；不剪的针尖离脸面 0.1（板薄于 1.6 时会冒出脸面零点几）
PIN_SLOT_was_hr52c_draft = dict(a_half=7.25, b0=2.4, b1=4.5, r=0.4)   # 初稿按排针行 3.0 … 3.9；商品 3D 图量到 ≈4.0 后 b1 放到 4.9
GROOVE_was_hr52_first_build = dict(depth=0.7, half_b=1.3, over_a=0.5)   # 槽端 ±8.12 离平台边 ±8.2 只剩 0.08 的一条边（clean_check thin FAIL，面积 0.56 mm²）→ 槽两端穿出平台边
CHIP_SLOT = dict(clr=0.6, r=0.4)             # hr52d：各边 +0.6 → 7.6 × 4.2（复审 r2 m1：正面图量芯片 (0, −2.2)，3D 图读数偏 +b 0.2–0.4 / −a 0.3–0.4 → 放到两种读数都装得下）；圆角 r 0.4
CHIP_SLOT_was_until_2026_09_30_hr52d = dict(clr=0.4, r=0.4)   # hr52c：7.2 × 3.8；芯片心 b −2.5 … −1.9 都放得下
CHIP_SLOT_was_until_2026_09_30_hr52c = dict(clr=0.25, r=0.4)   # 6.9 × 3.5 @ b −1.5
LOC_PIN = dict(d=1.8, h=1.2)                 # 两耳孔定位销（同 head.H04_CAM_PIN：Ø1.8 × 1.2 < 板厚 1.6）
BOSS = dict(d=5.0, top=88.3, pilot_d=1.7, pilot_bottom=92.9)       # 压条柱：顶 x 88.3；Ø1.7 底孔到 92.9（离脸面 0.57）
BOSS_was_until_2026_09_30_hr52b = dict(d=5.0, top=88.3, pilot_d=1.7, pilot_bottom=92.8)   # 孔底 92.8（离脸面 0.67，同 H04_CLAMP_BOSS）：拧紧后尖 92.5 离孔底正好 0.3 =
#   L5 pilot_bottom_margin 限值（4.5 − 0.3 = 4.2），float32 超 4.6e-6 判红 → 加深 0.1：拧紧后余 0.4（H03 功放柱外皮 0.47 先例）
BOSSES = ((-15.35, 0.0), (10.8, -6.0))       # 两根柱心（板系 a, b）：左柱在左耳外（柱面离耳端 0.5）；右柱在右耳斜下（离本体角 0.57、离耳 0.99 —— 复审 r1 m2：原注 1.06 有误）
BOSSES_was_preview_2026_09_29 = ((-15.35, 0.0), (10.5, -5.6))      # 预览版右柱离本体角只有 0.17（卖家手量尺寸容不下），正式版挪开

# ── H10 压条（同 H06 做法：脚比柱顶缝长 0.2，拧紧时压条微弯、一直压着板）
CLAMP = dict(t=1.8, gap=0.2, w=2.4, disc_d=5.4, foot_d=3.4, hole_d=2.4, bar_a1=12.05, leg_half_a=2.3)
#   t 1.8：M2×6 头下到尖 6 − (1.8 + 0.2) = 咬 4.0（pla_self_tap ≥4.0）；脚 Ø3.4（半径 1.7 < 孔心到本体边 1.85，只压耳朵、不压背面元件）
SCREW = dict(L=6.0, head_d=3.8, head_h=1.3)  # M2×6 盘头自攻（长度 = 头下到尖，同 fasteners.yaml 口径）
CLAMP_FRONT = BOSS["top"] - CLAMP["gap"]     # 88.1：压条前面（离柱顶 0.2）
CLAMP_BACK = CLAMP_FRONT - CLAMP["t"]        # 86.3：压条背面 = 螺丝头坐面


def world_T():
    """板系 → 世界：先绕 x 转 −THETA（a 端朝 +y 下倾），再平移到板心 (0, YC, ZC)。"""
    return _tr([0.0, YC, ZC]) @ _rot(math.radians(-THETA), [1.0, 0.0, 0.0])


def to_world(m):
    return placed(m, world_T())


def ab_world(a, b, x):
    """板系点 → 世界 (x, y, z)。"""
    p = world_T() @ np.array([x, a, b, 1.0])
    return tuple(float(v) for v in p[:3])


def pin_a(k):
    return -(N_PIN - 1) * PITCH / 2 + PITCH * k


def _bx(x0, x1, a0, a1, b0, b1):
    return wbox((x0, a0, b0), (x1, a1, b1))


def _cx(d, x0, x1, a, b, sections=48):
    return cyl(d, x1 - x0, ((x0 + x1) / 2, a, b), axis="x", sections=sections)


def _rrect(x0, x1, ac, bc, wa, wb, r):
    return union(_bx(x0, x1, ac - wa / 2 + r, ac + wa / 2 - r, bc - wb / 2, bc + wb / 2),
                 _bx(x0, x1, ac - wa / 2, ac + wa / 2, bc - wb / 2 + r, bc + wb / 2 - r),
                 *[_cx(2 * r, x0, x1, ac + sa * (wa / 2 - r), bc + sb * (wb / 2 - r), 24) for sa in (-1, 1) for sb in (-1, 1)])


def outline(x0, x1, grow=0.0):
    """板外形（本体 + 两耳）沿 x 拉伸，外扩 grow。"""
    return union(_bx(x0, x1, -BODY_A - grow, BODY_A + grow, -BODY_B - grow, BODY_B + grow),
                 _bx(x0, x1, -HOLE_A, HOLE_A, -EAR_D / 2 - grow, EAR_D / 2 + grow),
                 *[_cx(EAR_D + 2 * grow, x0, x1, s * HOLE_A, 0.0) for s in (-1, 1)])


# ── 电子件占位 zz_tof（板系）：PCB（带孔）+ 芯片 + 背面元件包络 + 排针（塑座 + 6 针，正面针脚不剪冒 1.4）+ 4 个杜邦壳
def module_local():
    pcb = diff(outline(XB, XF), *[_cx(HOLE_D, XB - 1.0, XF + 1.0, s * HOLE_A, 0.0, 32) for s in (-1, 1)])
    ca, cb, ch = CHIP
    chip = _bx(XF, XF + ch, CHIP_A - ca / 2, CHIP_A + ca / 2, CHIP_B - cb / 2, CHIP_B + cb / 2)      # hr52c：+ CHIP_A（= 0.0，几何不变）
    back = _bx(XB - BACK_COMP_H, XB, -BODY_A, BODY_A, -BODY_B, PIN_B - PITCH / 2 - 0.04)      # 背面元件包络：本体、排针塑座以下
    plastic = _bx(XB - HDR["plastic"], XB, pin_a(0) - PITCH / 2, pin_a(N_PIN - 1) + PITCH / 2, PIN_B - PITCH / 2, PIN_B + PITCH / 2)
    w = HDR["pin_w"] / 2
    pins = [_bx(XB - HDR["plastic"] - HDR["pin_back"], XF + HDR["solder_front"], pin_a(k) - w, pin_a(k) + w, PIN_B - w, PIN_B + w) for k in range(N_PIN)]
    x1 = XB - HDR["plastic"]
    dup = [_bx(x1 - DUP["L"], x1, pin_a(k) - DUP["w"] / 2 + 0.02, pin_a(k) + DUP["w"] / 2 - 0.02, PIN_B - DUP["w"] / 2, PIN_B + DUP["w"] / 2)
           for k, nm in enumerate(PIN_NAMES) if nm in USED]
    return union(pcb, chip, back, plastic, *pins, *dup)


def module_world():
    return to_world(module_local())


# ── H04 脸板：加料（实心平台 + 两根压条柱 + 两根定位销）/ 减料（两个底孔 + 焊锡包槽 + 排针通透长条 + 芯片通槽），板系
def h04_add_local():
    plat = outline(XF, X_IN + 0.05, MARGIN)
    bosses = [_cx(BOSS["d"], BOSS["top"], X_IN + 0.05, a, b) for a, b in BOSSES]
    pins = [_cx(LOC_PIN["d"], XF - LOC_PIN["h"], XF + 0.05, s * HOLE_A, 0.0, 32) for s in (-1, 1)]
    return union(plat, *bosses, *pins)


def h04_cut_local():
    pil = [_cx(BOSS["pilot_d"], BOSS["top"] - 0.5, BOSS["pilot_bottom"], a, b, 32) for a, b in BOSSES]
    g = GROOVE
    # was_until_2026_09_30_hr52c: groove = _bx(XF - 0.05, XF + g["depth"], pin_a(0) - PITCH / 2 - g["over_a"], pin_a(N_PIN - 1) + PITCH / 2 + g["over_a"], PIN_B - g["half_b"], PIN_B + g["half_b"])
    groove = _bx(XF - 0.05, XF + g["depth"], pin_a(0) - PITCH / 2 - g["over_a"], pin_a(N_PIN - 1) + PITCH / 2 + g["over_a"], g["b0"], g["b1"])
    p = PIN_SLOT                                                                 # hr52c：排针通透长条（针脚不剪）
    pslot = _rrect(XF - 0.05, X_OUT + 1.0, 0.0, (p["b0"] + p["b1"]) / 2, 2 * p["a_half"], p["b1"] - p["b0"], p["r"])
    s = CHIP_SLOT
    slot = _rrect(X_IN - 1.0, X_OUT + 1.0, CHIP_A, CHIP_B, CHIP[0] + 2 * s["clr"], CHIP[1] + 2 * s["clr"], s["r"])   # hr52c：a 心 0.0 → CHIP_A（= 0.0）
    # was_until_2026_09_30_hr52c: return union(*pil, groove, slot)
    return union(*pil, groove, pslot, slot)


# ── H10 压条（板系）：一根 2.4 宽条从左柱压过左耳、到右耳，再一段腿下到右柱；两只 Ø3.4 脚压耳朵背面；Ø2.4 过孔
def clamp_local():
    c = CLAMP; (la, lb), (ra, rb) = BOSSES; hw = c["w"] / 2
    x0, x1 = CLAMP_BACK, CLAMP_FRONT
    bar = _bx(x0, x1, la, c["bar_a1"], -hw, hw)
    leg = _bx(x0, x1, ra - c["leg_half_a"], ra + c["leg_half_a"], rb, hw)
    discs = [_cx(c["disc_d"], x0, x1, a, b) for a, b in BOSSES]
    feet = [_cx(c["foot_d"], x1 - 0.05, XB, s * HOLE_A, 0.0) for s in (-1, 1)]
    under = [_cx(c["foot_d"] + 0.4, x0, x1, s * HOLE_A, 0.0) for s in (-1, 1)]     # hr52 build #3：脚 Ø3.4 比条宽 2.4 每边多 0.5 → 条这一层垫 Ø3.8 盘托住（切片支撑 0.79 mm³ → 0）
    holes = [_cx(c["hole_d"], x0 - 1.0, x1 + 1.0, a, b, 32) for a, b in BOSSES]
    return diff(union(bar, leg, *discs, *under, *feet), *holes)


def screws_local():
    """2 颗 M2×6（未拧紧位：头下 = 压条背面 86.3，尖 92.3；拧紧后整体 +0.2），仅给预览 / 检查用，不进 placed。"""
    out = []
    for a, b in BOSSES:
        out.append(_cx(SCREW["head_d"], CLAMP_BACK - SCREW["head_h"], CLAMP_BACK, a, b, 32))
        out.append(_cx(2.0, CLAMP_BACK, CLAMP_BACK + SCREW["L"], a, b, 24))
    return union(*out)


def points():
    """登记 / 检查用的世界坐标（板心、芯片槽心、两柱心、两耳孔心、4 个杜邦壳尾中心）。"""
    d = dict(center=ab_world(0.0, 0.0, XF), slot_center_face=ab_world(CHIP_A, CHIP_B, X_OUT),
             bosses=[ab_world(a, b, X_IN) for a, b in BOSSES], holes=[ab_world(s * HOLE_A, 0.0, XF) for s in (-1, 1)],
             dupont_tail={nm: ab_world(pin_a(k), PIN_B, XB - HDR["plastic"] - DUP["L"]) for k, nm in enumerate(PIN_NAMES) if nm in USED},
             bite=round(CLAMP_BACK + SCREW["L"] - BOSS["top"], 3), screw_tip_x=round(CLAMP_BACK + SCREW["L"], 3))
    return d
