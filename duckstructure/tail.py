"""B02 尾部托架：宇树单总线转接板（40×30×9，透明壳）的托盘，2×M2×6 拧在电池仓门 B01 外面。2026-09-20（hr17）。

放哪、为什么（docs/design_2026-09-17_bearing_rebuild/电子件布局_2026-09-20.md §1）：
  · 顶板口袋：颈部让位口是露天的，真实姿态下颈/头在 |y|≥12 处压到 z 152.8，只剩 |y|≤10 × 5 高的一条 → 放不下 40×30 的板；
  · 电池仓两侧腔（壳内、导轨外）：22.5 × 14 × 32，放不下板，放 UBEC/WAGO/一分二正好；
  · 头里：桥上/圆顶下被 N03 横滚扫掠占掉，Radxa 上方只剩 13 高；
  · 尾部（门外）：真实姿态下头最低到 z 137.5（roulade，x −72）、腿在 x<−44 只到 |y|≥26 且 z≤113 → **z 118..134.5、|y|≤22** 这一层是空的。
  全行程网格（neck 60 + head −54）头会低到 z 110 —— 区间外组合，原版同样撞躯干，控制端限位管（frozen target 是单轴并集，不是组合）。

几何（世界系，trunk_base 刚体，随门拆装）：
  · 背板 x −48.3..−46.8（贴门面 BX0−DOOR_T=−46.8）、|y|≤22、z 120..134.5；上延 |y|≤15 到 z 148 挂 2 颗 M2×6（(±6, 144)，门顶条 z 141..148 实心，
    真实姿态头在 x −52..−46 最低 z 152.9/160.2，上延到 148 留 4.9；托架主体 x −52..−81 头最低 137.5，唇顶 134.7 留 2.8，
    离卡珠槽 y ±14 8 mm）—— 螺丝坐面在 Ø6×3.5 凸台外面 x −50.3（凸台含背板 1.5，总叠厚 3.5），M2×6 进门 2.5、尖端离门内面 0.5（电池离门内面 0.6，不进仓腔）
  · 底板 z 122.5..124、x −48.3..−83.9；板平放 z 124..133，40 沿 y、30 沿 x（x −48.6..−78.6，前隙 0.3、尾隙 0.6）；两条 30 mm 短边朝 ±y ——
    PH 座 + XT30(2+2) 在一侧、XT30 入口 + Type-C 在另一侧（09-20 实测口位，两口离角只有 1–3 mm，所以 ±y 端**不能有立墙**）
  · 三面 1.2 高的矮沿（±y 侧、后端 x −79.2..−80.9）：只碰透明壳底边，插头都在壳底 ≥2.6 以上（推断，实物核）
  · 背板顶部 1.5 深的压唇（x −48.3..−49.8, z 133.5..134.7）压住板前沿；板**斜着放进去**（前沿先塞到唇下再落后沿），后沿用一根扎带
    穿底板两条槽（x −81.5..−82.9、后沿外；y ±10..±14）压住——0 焊接 0 胶。打印：底板朝下，压唇 1.5 平悬挑、Ø6 凸台横卧 → 切片支撑 ~0.9 cm³ 落在凸台下与唇下（唇下的支撑柱在口袋里，拆后唇底留疤由 0.5 唇隙吸收）
  · 两侧抱门耳 |y| 15.8..17.5、x −46.8..−44.3、z 123..134（落在门旁 y 16..21 的后缝里，壳后角从 y 21 起；离导轨后端面 0.5）
  · 线：转接板两侧口的线沿 ±y 出托架，向前进门旁后缝（y 16..21 × z 123..148）到电池仓侧腔
"""
import numpy as np
from .s288 import S, cyl, union, diff, placed
from .lib import P, BX0, DOOR_T, DOOR_ADD, keep_main, wbox as _wbox

def wbox(lo, hi):
    """角点任意顺序的方块（本模块里 ±y 对称件常把 y 写反）"""
    return _wbox(tuple(min(a, b) for a, b in zip(lo, hi)), tuple(max(a, b) for a, b in zip(lo, hi)))

ADP = (30.0, 40.0, 9.0)          # 转接板外形 (x, y, z)：components.yaml:bus_adapter.envelope_mm 40×30×9，平放、40 沿 y
ADP_CLR = 0.3
ADP_CLR_REAR = 0.6              # 尾隙（hr18 复审 m1：斜插刚体余量只有 0.26 → 尾隙加 0.3）
XD = BX0 - DOOR_T                # -46.8 门外面
T_BACK, T_FLOOR = 1.5, 1.5
XB = XD - T_BACK                 # -48.3 背板外面 = 板前沿的靠面
Z_FLOOR = (122.5, 124.0)         # 底板
Z_TOP = 134.7                    # 背板/唇顶（唇底 133.5 = 板顶 133 + 0.5）
Y_HALF = 22.0                    # 托架半宽 = 20 + 0.3 隙 + 1.7 沿
RIM_H, RIM_T = 1.2, 1.7
X_END = XB - ADP[0] - ADP_CLR - ADP_CLR_REAR - RIM_T - 0.6 - 1.4 - 1.0   # -83.9：底板后端 = 后沿靠面 −79.2 − 沿 1.7 − 0.6 − 扎带槽 1.4 − 外壁 1.0
LIP = (1.5, 1.2)                 # 压唇：伸出 1.5、厚 1.2，平悬挑（1.5 mm 不需支撑；不能做斜面——斜面会压到板前上角）
SCREW = ((6.0, 144.0), (-6.0, 144.0))   # 2×M2×6 → 门顶条
BOSS_D, BOSS_T = 6.0, 3.5
TIE_SLOT = ((XB - ADP[0] - ADP_CLR - ADP_CLR_REAR - RIM_T - 0.6, XB - ADP[0] - ADP_CLR - ADP_CLR_REAR - RIM_T - 0.6 - 1.4), (10.0, 14.0))   # x −81.5..−82.9（后沿外 0.6）, |y| 10..14
EAR = ((XD, XD + 2.5), (15.8, 17.5), (123.0, 134.0))   # 到 −44.3，离导轨后端面 BX0=−43.8 留 0.5

# ⚠ hr28（2026-09-21）：下面三个（adapter_box / build_tail_tray / door_pilots）作废 —— 用户定转接板进头里（head.adapter_box / H03 口袋），B02 不再 build、
#    B01 不再开底孔。函数留作历史，build.py 不再引用。扎带环（tie_loops）仍在用；开关面板（SWITCH_*）hr39f 删除；hr39f 起本文件另放 B03 电池顶盖。
def adapter_box():
    """【作废 hr28】转接板占位（世界系），供检查/预览：x −48.6..−78.6、|y|≤20、z 124..133"""
    x1 = XB - ADP_CLR; x0 = x1 - ADP[0]
    return wbox((x0, -ADP[1] / 2, Z_FLOOR[1]), (x1, ADP[1] / 2, Z_FLOOR[1] + ADP[2]))

def build_tail_tray():
    z0, z1 = Z_FLOOR
    back = union(wbox((XB, -Y_HALF, 120.0), (XD, Y_HALF, Z_TOP)),
                 wbox((XB, -15.0, Z_TOP - 0.5), (XD, 15.0, 148.0)))                   # 上延到门顶挂螺丝
    floor = wbox((X_END, -Y_HALF, z0), (XB, Y_HALF, z1))
    rims = [wbox((X_END, sy * (Y_HALF - RIM_T), z1 - 0.01), (XB, sy * Y_HALF, z1 + RIM_H)) for sy in (1, -1)]
    x_rim = XB - ADP[0] - ADP_CLR - ADP_CLR_REAR                                       # -79.2 后沿靠面（板尾 −78.6 前留 0.6）
    rims.append(wbox((x_rim - RIM_T, -Y_HALF, z1 - 0.01), (x_rim, Y_HALF, z1 + RIM_H)))
    lip = wbox((XB - LIP[0], -Y_HALF, Z_TOP - LIP[1]), (XB + 0.01, Y_HALF, Z_TOP))
    bosses = [cyl(BOSS_D, BOSS_T, (XD - BOSS_T / 2, y, z), axis="x") for (y, z) in SCREW]
    ears = [wbox((EAR[0][0], sy * EAR[1][0], EAR[2][0]), (EAR[0][1], sy * EAR[1][1], EAR[2][1])) for sy in (1, -1)]
    m = union(back, floor, *rims, lip, *bosses, *ears)
    cuts = [cyl(S["mnt_hole_d"], BOSS_T + 4.0, (XD - BOSS_T / 2 - 1.0, y, z), axis="x") for (y, z) in SCREW]   # Ø2.4 过孔（M2×6 进门 3.0）
    (xa, xb), (ya, yb) = TIE_SLOT
    cuts += [wbox((xb, sy * ya, z0 - 1), (xa, sy * yb, z1 + 1)) for sy in (1, -1)]
    return keep_main(diff(m, *cuts), "tail_tray")

def door_pilots():
    """B01 门上给 B02 的 2 个 Ø1.7 底孔（穿透门厚 3.0；M2×6 自攻）—— 由 trunk.build_battery_door 减掉"""
    return [cyl(1.7, DOOR_T + 2.0, (XD + DOOR_T / 2, y, z), axis="x") for (y, z) in SCREW]

# ── 总开关 KCD1-101：hr39f（2026-09-23）整套删除 ──
#    用户定：取消电源开关，断电 = 开 B03 顶盖拔电池 XT30（docs/design_2026-09-17_bearing_rebuild/hr39_电池拔插代替开关.md §5.1）。
#    原 SW_CUT / SW_PANEL / SW_PANEL_FP / SW_SKIRT_T / SW_UNDER_Z0 / SWITCH_UNDERCUT / SWITCH_PANEL / SW_CHAMFER / SWITCH_CHAMFER /
#    SWITCH_TOPFLAT / SWITCH_CUTOUT 全删，T02 在这一带回到原壳（再被下面的 LID_CUT 开顶盖口）。旧代码见 hr39f_work/src_before/tail.py。


# ── 电池仓导轨外面的扎带环（T01，两侧各 2）──
TIE_X, TIE_Z = (-39.25, -33.25), (126.0, 136.0)   # x 位避开导轨绑带槽 x −29..−27（trunk.battery_bay_cuts）；±2 端面 −41.25/−37.25/−35.25/−31.25 故意离开 L5 R14 探针的 0.5 网格
                                                     # （hr18 复审 M2：射线躺在环端面平面里把 ±y 两环并成一段 → signed +31 假红；根治在 l5 contact_profile 加半步偏移，本轮件侧先躲开）
def tie_loops():
    from .lib import RAIL_Y
    def sbox(a, b, y0, y1, z0, z1, s):
        ya, yb = sorted((s * y0, s * y1)); return wbox((a, ya, z0), (b, yb, z1))
    yl, yb_, yh = RAIL_Y - 0.3, RAIL_Y + 2.2, RAIL_Y + 3.5      # 环：贴导轨外面 −0.3（真·重叠）→ 外 3.5；扎带槽 = yl..yb_（2.2 宽）× 中段 7 高
    out = []
    for s in (1, -1):
        for xc in TIE_X:
            a, b = xc - 2.0, xc + 2.0
            out += [sbox(a, b, yl, yh, TIE_Z[0], TIE_Z[0] + 1.5, s), sbox(a, b, yl, yh, TIE_Z[1] - 1.5, TIE_Z[1], s), sbox(a, b, yb_, yh, TIE_Z[0], TIE_Z[1], s)]
    return out


# ───────────────────────────── B03 电池顶盖（hr39f，2026-09-23）─────────────────────────────
# 方案 (f)（hr39_电池拔插代替开关.md §3）：断电 = 开 B03 拔电池顶上的 XT30 对插头；换电池 = 开 B03 → 拆 B01 → 拉环往后抽电池。
# 盖子外形就是两半壳在开口范围里的那块（从原版 left/right_shell 上切下来，外表面原样），另加：
#   · 4 只盖脚：落在两条导轨顶 BZ1=146.9（|y| 13.35..14.75）→ 盖子高度由导轨定，名义上与原壳面平齐（高差 0）。
#   · 前舌片 ×2（弹性，兼作 −x 棘爪）：从盖子前缘往 +x 伸进两半壳留下的"壳条"（x LID_X[1]..−20.9，颈部开口后沿）下面，
#     壳条下面各加一块承接台阶（lid_strip_pad，属 T02/T03），台阶底有一道 0.3 凸棱；舌尖顶上 0.25 凸点推过凸棱后落在它前面 = 咔一下。
#   · 后卡舌（锁门）：|y|≤5.8 一片 1.3 厚的板，从壳后卷边底往下插进 B01 门外面顶部的平齐凹槽（原 B01-F07 平衡线缺口的位置），
#     舌端倒钩 +x 钩进凹槽里壁的小窝 → 盖子后端不能掀、门顶不能往后退。
#   · 拇指槽：后卷边顶面 0.8 深的随形浅槽（平齐，不凸出），往后推盖子用。
# 装：电池 → B01 → 插 XT30 → B03（放在终位后方 LID_SLIDE 处平放落下，再沿 +x 推 LID_SLIDE 到位，咔）。拆：拇指在槽里往后推 3.5（咔）→ 提起。
#
# **与报告 §3/§5.2 的出入（hr39f 实现时几何核出来的，报告 §9 有写）**：
#   ① LID_X 前沿 −21 → −25：颈部开口后沿就在 x −20.9（|y|<14.5 时 −21 往前没有壳），"前舌片插在两半壳切边下面"只能插在
#     侧切边（|y|>15）下面 —— 那一带上面是壳、后面是卷边（底 151.3），刚体沿任何直线都进不去。退 4 mm 留一条 4 宽的壳条给舌片压。
#   ② 倒钩只管"掀"；盖子 −x 靠前舌片的弹性凸点（刚体进得去的锁都能沿原路退出来，所以必须有一处弹性）。倒钩勾的是门，门又靠盖子锁 ——
#     只靠倒钩会让"门绕底脚转 + 盖子一起后退"成为机构，所以 −x 必须锁在固定件（壳条）上。
LID_X = (-46.6, -25.0)          # 顶盖开口 x（后端伸到壳后缘 −46.1 外；前端见上 ①）
LID_Y, LID_Z0, LID_GAP = 15.0, 148.0, 0.2
LID_SLIDE = 3.5                 # 装配推入行程（+x）
LID_FOOT_Y = (13.35, 14.75)
LID_FOOT_X = ((-38.6, -36.6), (-29.0, -27.0))       # 后脚在 x ≥ BX0+LID_SLIDE+0.5（推入起点仍在导轨上，不落到门顶）
LID_FOOT_TOP = (152.4, 159.4)                        # 后/前脚顶（伸进卷边下层 151.4..152.9 / 顶棚 158.7..160.9 里，build_battery_lid 里射线断言）
# 前舌片 + 壳条承接台阶
LT_Y = (6.5, 11.5)              # |y|：避开 zz_xt30 的平衡头（|y|≤5.7）
LT_X = (-30.0, -22.4)           # 根 −30.0..−28.8 连顶棚，悬臂 −28.8..−22.4（6.4 长）
LT_ROOT_X1, LT_T = -28.8, 1.0
PAD_X, PAD_Y, PAD_Z = (-25.0, -21.4), (6.3, 11.7), 158.7     # 台阶底 158.7（壳条内表面 159.0..159.7）
LT_TOP = PAD_Z - 0.4            # 158.3 舌片顶
RIDGE = (-24.9, -23.7, 0.3)     # 台阶底凸棱：x 基底、下凸高（45° 斜面）→ 棱底 158.4，舌片顶让 0.1
BUMP = (-23.6, -22.6, 0.35)     # 舌尖凸点：x 基底、高（45° 斜面）→ 顶 158.65（离台阶底 0.05），推过凸棱时舌片下弯 0.25
# 后卡舌 + 门上凹槽（门的切口在 trunk.build_battery_door）
# hr48（2026-09-28，电池实测 20 厚 → B01 门外面后移 lib.DOOR_ADD 2.0）：后卡舌 / 根 / 倒钩 / 门凹槽 / 倒钩窝的 x 全部随门后移 DOOR_ADD；was_until_2026_09_28 的值在各行注释里。
#   盖后沿 x<−46.0 本来没有盖料（根部裁皮只到 −46.0），后移后的卡舌上面是空的 → build_battery_lid 里显式加根延长块 RT_EXT（x 根后端..−44.2、|y|≤RT_Y、z 148.2..RT_EXT_ZTOP）。
RT_X, RT_Y, RT_Z0 = (-46.75 - DOOR_ADD, -45.45 - DOOR_ADD), 5.8, 141.7                       # was (-46.75, -45.45)
RT_ROOT = ((-46.75 - DOOR_ADD, -44.2), (148.2, 152.3))          # 门顶以上加厚到卷边底（门顶 148.0，让 0.2）；was ((-46.75, -44.2), …)
RT_EXT_ZTOP = 151.4                                   # hr48：根延长块顶（壳后卷边底 151.3 之下、门顶 148.0 之上）；登记单预检：零位静态件 0 交集（+0.5 包络）、腿链髋偏航全程 ≥0.5
BARB = ((-45.45 - DOOR_ADD, -44.9 - DOOR_ADD), 4.8, (141.7, 142.8))   # was ((-45.45, -44.9), …)
DOOR_RECESS = (-45.4 - DOOR_ADD, 6.0, 141.5)         # 门外面凹槽：x 外面..外面+1.4（hr48 −48.8..−47.4；was −46.8..−45.4）、|y|≤6.0、z 141.5..门顶
DOOR_POCKET = ((-45.4 - DOOR_ADD, -44.8 - DOOR_ADD), 5.0, (141.5, 143.0))   # 倒钩窝（再深 0.6；hr48 窝后到槽底 −45.4 还有 1.4；was (-45.4, -44.8)）
THUMB = ((-44.0, -42.2), 5.0, 0.8)                    # 拇指槽 x、|y|、深

def LID_CUT():
    """两半壳上开顶盖口的刀（世界系）：x 从壳后缘外到 LID_X[1]、|y|≤LID_Y、z ≥ LID_Z0（这一带壳最低 151.3，148 以下本来没有壳料）。"""
    return _wbox((-60.0, -LID_Y, LID_Z0), (LID_X[1], LID_Y, 175.0))

def _moved(m, d):
    c = m.copy(); c.apply_translation(d); return c

def _prism_xz(pts, y0, y1):
    """(x, z) 多边形沿 y 拉伸成 y0..y1 的棱柱"""
    import math as _m, trimesh as _t
    from shapely.geometry import Polygon as _P
    from trimesh.transformations import rotation_matrix as _rot
    m = _t.creation.extrude_polygon(_P(pts), y1 - y0)
    m.apply_transform(_rot(_m.pi / 2, [1, 0, 0])); m.apply_translation((0, y1, 0))
    if m.volume < 0: m.invert()
    return m

def _prism_xy(pts, z0, z1):
    import trimesh as _t
    from shapely.geometry import Polygon as _P
    m = _t.creation.extrude_polygon(_P(pts), z1 - z0); m.apply_translation((0, 0, z0))
    if m.volume < 0: m.invert()
    return m

def lid_strip_pad(side):
    """T02/T03 壳条下的承接台阶 + 凸棱（side=+1 左 / −1 右）。台阶顶伸进壳条 0.9..1.2（壳条外表面 ≥161.3，不穿）。"""
    (x0, x1), (y0, y1) = PAD_X, PAD_Y
    ya, yb = sorted((side * y0, side * y1))
    pad = _wbox((x0, ya, PAD_Z), (x1, yb, 159.9))
    r0, r1, h = RIDGE
    ridge = _prism_xz([(r0, PAD_Z + 0.01), (r1, PAD_Z + 0.01), (r1 - h, PAD_Z - h), (r0 + h, PAD_Z - h)], ya, yb)
    return union(pad, ridge)

def build_battery_lid():
    """B03 电池顶盖（世界系零位 = 装好的位置）。"""
    from .lib import shell, BZ1
    from .s288 import inter
    g = LID_GAP
    box = _wbox((-60.0, -LID_Y + g, LID_Z0), (LID_X[1] - g, LID_Y - g, 175.0))
    L, R = inter(shell(1), box), inter(shell(-1), box)
    # 中缝：原版两半壳在 y≈0 是搭接（外缝 y −0.55..0.05、左壳内翻边伸到 −1.95、右壳唇底与之隔 0.17），盖子是一件 → 把两边的截面各往对面平移 0.5/1.0/.. 填实。
    # 平移的是同一张壳面（y 方向 0.7 内外表面起伏 ≤0.02），不高出原壳（中缝上沿本来就是右壳唇 161.57 最高）。
    sb = _wbox((-60.0, -2.9, LID_Z0), (LID_X[1] - g, 0.4, 175.0))
    fill = union(*[inter(_moved(L, (0, -d, 0)), sb) for d in (0.5, 1.0, 1.5, 2.0, 2.5, 3.0)],
                 *[inter(_moved(R, (0, d, 0)), sb) for d in (0.35, 0.7)])
    body = union(L, R, fill)
    # 盖脚：顶伸进壳料里（射线断言：脚顶以下 0.3 必须还在壳料里，脚顶离该层上表面 ≥ 0.5）
    feet = []
    for (xa, xb), zt in zip(LID_FOOT_X, LID_FOOT_TOP):
        for sy in (1, -1):
            ya, yb = sorted((sy * LID_FOOT_Y[0], sy * LID_FOOT_Y[1]))
            for xx in (xa + 0.05, xb - 0.05):
                for yy in (ya + 0.05, yb - 0.05):
                    hit = body.ray.intersects_location(np.array([[xx, yy, BZ1]]), np.array([[0, 0, 1.0]]))
                    zs = np.sort(hit[0][:, 2])
                    assert len(zs) >= 2 and zs[0] <= zt - 0.3, f"盖脚 ({xx:.1f},{yy:.1f}) 顶 {zt} 没伸进壳料：{zs[:4]}"
                    inside = [(zs[i], zs[i + 1]) for i in range(0, len(zs) - 1, 2) if zs[i] <= zt <= zs[i + 1]]
                    assert inside and inside[0][1] - zt >= 0.5, f"盖脚 ({xx:.1f},{yy:.1f}) 顶 {zt} 离外表面太近：{zs[:4]}"
            feet.append(_wbox((xa, ya, BZ1), (xb, yb, zt)))
    # 前舌片：根连顶棚，悬臂下面/上面都空（上面离顶棚内表面 ≥0.4，下弯 0.25 时下面 |y| 6.5..11.5 没有东西）
    tongues = []
    for sy in (1, -1):
        ya, yb = sorted((sy * LT_Y[0], sy * LT_Y[1]))
        beam = _wbox((LT_X[0], ya, LT_TOP - LT_T), (LT_X[1], yb, LT_TOP))
        root = _wbox((LT_X[0], ya, LT_TOP - LT_T), (LT_ROOT_X1, yb, 159.8))
        b0, b1, h = BUMP
        bump = _prism_xz([(b0, LT_TOP - 0.01), (b1, LT_TOP - 0.01), (b1 - h, LT_TOP + h), (b0 + h, LT_TOP + h)], ya, yb)
        tongues += [beam, root, bump]
    # 后卡舌（锁门）+ 倒钩
    (tx0, tx1), (rx0, rx1), (rz0, rz1) = RT_X, RT_ROOT[0], RT_ROOT[1]
    blade = _wbox((tx0, -RT_Y, RT_Z0), (tx1, RT_Y, rz0 + 0.3))
    root = _wbox((rx0, -RT_Y, rz0), (rx1, RT_Y, rz1))
    # hr39f 09-24：矩形根部在壳后卷边的斜面上冒出约 0.36 mm（竖直射线）；
    # 用现有盖面的连续向下扫掠裁顶，根部不得改变原盖外形。先局部裁皮降低计算量；
    # xy 的 0.0001 仅使 Minkowski 盒有正体积，z 上界仍为 0，不把外表面向上抬。
    from .lib import minkowski_box
    root_skin = inter(body, _wbox((rx0 - 0.01, -RT_Y - 0.01, LID_Z0),
                                  (rx1 + 0.01, RT_Y + 0.01, 175.0)))
    root = inter(root, minkowski_box(root_skin, (-0.0001, -0.0001, -8.0),
                                    (0.0001, 0.0001, 0.0)))
    (bx0, bx1), by, (bz0, bz1) = BARB
    barb = _wbox((tx1 - 0.01, -by, bz0), (bx1, by, bz1))
    ext = _wbox((rx0, -RT_Y, rz0), (-44.2, RT_Y, RT_EXT_ZTOP))                            # hr48：根延长块（不裁皮）——卡舌后移后上面没有盖料，靠它连到 x≥−46.0 的原根
    m = union(body, *feet, *tongues, blade, root, ext, barb)
    # 拇指槽：后卷边顶面随形 0.8 深（外表面往下 0.8 的那层皮）
    (ux0, ux1), uy, ud = THUMB
    skin = inter(diff(m, _moved(m, (0, 0, -ud))), _wbox((ux0, -uy, 155.0), (ux1, uy, 175.0)))
    if skin is not None and not skin.is_empty:
        m = diff(m, skin)
    return keep_main(m, "battery_lid")
