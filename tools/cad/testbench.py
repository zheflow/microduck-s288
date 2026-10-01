#!/usr/bin/env python3
"""S288 单舵机测试台（BAM 标定 + 到货扭矩测试共用）。

对应 upstream/microduck_rl 的 xl330_test_bench：舵机立着装，输出轴水平，法兰面上装一根摆臂，
臂端挂 ~120 g 配重当钟摆，摆幅 ±80°。三个打印件：
  TB01_base   底座 + 立柱 + 舵机卡槽（只夹外壳 20×34×26，不碰任何孔位，官方尺寸）
  TB02_arm    摆臂：舵盘端 Ø14.5×1.5 口袋套住法兰（靠法兰外圆对孔）+ 6×Ø2.6 圆孔（分度圆 r=S["horn_r"]=5.25，09-13 实物定案 Ø10.5）+ 中心 Ø5 通孔 + 口袋底 Ø8×3.0 沉台；臂端 Ø6.5 挂 M6 螺栓+垫片配重
  ⚠ 09-13 s288.S 改成手册值（T 20 / T_lo 23 / 法兰 Ø14×3 / 副轴 Ø14×3）后本文件未重跑：cad/testbench_s288/ 里的 STL 仍是 09-13 白天按 T 19.9/T_lo 22.8 出的
  （测试台已定稿；重跑会把 TB01 内腔加深 0.2、TB02 口袋底/臂杆外移 0.15）。
  TB03_strap  压条：压垫坐舵机顶、两端离墙顶 2mm，2×M2×10 拧进凸台（Ø1.9 深 8 光孔），螺丝拉力=预紧
舵机局部坐标同 s288.py：输出轴 = +x，长边 = z（顶端 z=+9.5，接线端 z=-24.5），宽 20 沿 y。
"""
import math, os, sys
import numpy as np, trimesh
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", ".."))
from duckstructure import s288
from duckstructure.s288 import S, cyl, bx, union, diff

OUT = os.path.join(os.path.dirname(__file__), "..", "..", "cad", "testbench_s288")
os.makedirs(OUT, exist_ok=True)

AXIS_H   = 130.0        # 轴心离桌面高度（摆臂 100 + 配重半径 + 余量）
ARM_LEN  = 96.0         # 轴心到配重中心（原版 96.6）
ARM_W, ARM_T = 12.0, 5.0
BOSS_H = 2.0            # 摆臂贴法兰那面的抬高凸台
CLR, WALL = 0.4, 2.7
L, W, T, ZT = S["L"], S["W"], S["T"], S["top"]
Z_BOT = ZT - L          # -24.5 接线端
BOSS_Y = W / 2 + CLR + WALL + 1.0      # 压条螺丝凸台中心 y=14.1（凸台 y 11.1..17.1，贴墙外侧，不侵入内腔）
STRAP_SPAN = 2 * BOSS_Y               # 压条两孔距 28.2，孔正落在凸台中线
STRAP_LIFT = 2.0                        # 压条底面高出墙顶 2mm：压条只坐在舵机顶上，不落墙，螺丝拉力直接成为预紧力（舵机长 34 是手册值，短 1.9 以内仍压得住）
X_LOW_WALL_G = S["T"] / 2 + CLR + 2.0   # +x 矮墙外表面：本体法兰侧面 +2.4（=12.4，比法兰面 13.0 还低 0.6，摆臂凸台不碰；旧 12.35/12.85）
PAD_BOT = ZT - CLR - 0.2                # 压垫底 8.9（舵机顶实际 9.1）

def build_base():
    # ── 卡槽：内腔 = 舵机 + 间隙；−x 墙 / ±y 墙到 z=+12，+x 墙（法兰侧）只到 z=-14 让开法兰和摆臂
    floor_z = Z_BOT - 2.5
    # 舵机背面是台阶（薄段 T=20 / 厚段 T_lo=23，手册；旧实测 19.9/22.8），法兰面是一整个平面。
    # 内腔按"整段都按下半段厚度"挖（stepped=False）：一定塞得进去。上半段背后空 2.9mm 不影响——
    # 摆臂载荷是绕 x 的力矩 + 竖直力；绕 y 只有 W×30mm≈36 N·mm，靠下部前后筋（z -16..-24.9）足够。
    XBL = s288.x_rear_face_lo()                       # -13.0 下半段背面（旧 -12.85）
    X_OUT_NEG = XBL - CLR - WALL                      # -16.1 卡槽 -x 外表面（旧 -15.95）
    inner = s288.body_prism(c=CLR, stepped=False, z_ext=40)
    outer_hi = bx((X_LOW_WALL_G - X_OUT_NEG, W + 2 * WALL, 12 - floor_z), ((X_LOW_WALL_G + X_OUT_NEG) / 2, 0, (12 + floor_z) / 2))
    # +x 侧只留低墙：把高于 z=-11 且 x>T/2 的部分切掉
    cut_hi = bx((10, W + 2 * WALL + 2, 40), (T / 2 + CLR + 5, 0, -14 + 20))
    cradle = diff(outer_hi, inner, cut_hi)
    RIB_IF = 0.05                            # 筋顶面比舵机外壳往里 0.05（单边过盈，复审后由 0.1 降低），吃掉 0.4 间隙防晃
    rib_t = CLR + RIB_IF + 0.3               # 筋从墙里 0.3 长到过盈面：0.75
    LEAD = CLR + RIB_IF                      # 筋顶端 45° 导入斜面：顶端退到墙面，舵机从上方压入时逐渐被挤
    def rib(size, center, recede):
        """带顶端导入斜面的筋：底部矩形 → 顶部矩形整体沿 recede(dx,dy) 缩进墙里，凸包得 45° 斜面"""
        x, y, z = size; cx, cy, cz = center; z0, z1 = cz - z / 2, cz + z / 2
        pts = []
        for sx in (-1, 1):
            for sy in (-1, 1):
                pts += [(cx + sx * x / 2, cy + sy * y / 2, z0), (cx + sx * x / 2, cy + sy * y / 2, z1 - LEAD),
                        (cx + sx * x / 2 + recede[0], cy + sy * y / 2 + recede[1], z1)]
        return trimesh.convex.convex_hull(np.array(pts))
    ribs = []
    zf = Z_BOT - CLR                         # 槽底 -24.9
    z0, z1 = S["conn_z"]
    WIN_MARGIN = 4.0                         # 接线窗上下各留 4（原 2；轴心/插座位置是图纸反推的，多留余量）
    X_LOW_WALL = X_LOW_WALL_G                # +x 矮墙外侧面
    global WIN_XA, WIN_XB, WIN2_XA, WIN2_XB, XBL_G, ZF_G
    XBL_G = XBL; ZF_G = Z_BOT - CLR
    # 插座位置没量（7a/7b/7c），窗子把舵机整个侧面（x -12.85..+9.95，z 槽底..-1.5）几乎全露出来：
    #   上段（z -20.5..-1.5）x -13.3..+9.1：-x 端在背墙内表面 -13.25 外一点，背墙完整，窗顶 22.4mm 桥接两端有支撑
    #   下段（z 槽底..-20.5）x -10.8..+7.6：两端各留 ~2mm 角柱，角柱内侧放下部 y 向挤压筋（第 6 轮复审：原窗下沿离舵机底 4.4mm，插座若靠底就出不来）
    WIN_XA, WIN_XB = XBL - CLR - 0.05, X_LOW_WALL - 3.25
    WIN2_XA, WIN2_XB = XBL + 2.05, S["T"] / 2 - 2.35
    WIN_Z0, WIN_Z1 = z0 - WIN_MARGIN, z1 + WIN_MARGIN
    rib_lo_top = WIN_Z0 - 0.5                # ±y 墙下部筋止于上段窗下沿 0.5 以下（筋在下段窗两端的角柱内侧）
    for sy in (1, -1):                       # ±y 墙：下部筋在角柱 x=-11.3 / +8.1，槽底..-21；上部筋 x=±8，z 0..8（窗上方，消顶部晃动）
        for xr in (WIN2_XA - 0.5, WIN2_XB + 0.5):
            ribs.append(rib((1.0, rib_t, rib_lo_top - zf), (xr, sy * (W / 2 - RIB_IF + rib_t / 2), (zf + rib_lo_top) / 2), (0, sy * LEAD)))
        for sx in (1, -1):
            ribs.append(rib((1.0, rib_t, 8.0), (sx * 8, sy * (W / 2 - RIB_IF + rib_t / 2), 4.0), (0, sy * LEAD)))
    for sy in (1, -1):                       # ∓x 墙：y=±3（避开 y=±7.35 的安装孔），槽底..-16（+x 矮墙到 -14，够）
        ribs.append(rib((rib_t, 1.0, -16 - zf), (XBL + RIB_IF - rib_t / 2, sy * 3, (zf - 16) / 2), (-LEAD, 0)))
        ribs.append(rib((rib_t, 1.0, -16 - zf), ( (T / 2 - RIB_IF + rib_t / 2), sy * 3, (zf - 16) / 2), ( LEAD, 0)))
    cradle = union(cradle, *ribs)
    # 接线窗：+y 墙，接线端 z 区
    # 压条螺丝凸台：±y 墙外各一条 8×6 通到桌面，Ø1.9 光孔
    z_table = -AXIS_H
    for sy in (1, -1):
        boss = bx((8, 6, 12 - z_table), (-3, sy * BOSS_Y, (12 + z_table) / 2))   # 一直到桌面，兼作加强肋，无悬空
        cradle = union(cradle, boss)
        cradle = diff(cradle, cyl(1.9, 8, (-3, sy * BOSS_Y, 12 - 4), axis="z"))      # Ø1.9 深 8（底 z=4），竖直孔打出来 ≈1.75，M2 机牙自攻成形；凸台壁 2.05：M2×10 穿压条后尖端到 z=6，孔底距窗上倒角 ≥2.5
    for sy in (1, -1):   # 接线窗最后开，切穿墙和凸台（两侧都开：法兰定死了朝向，插座在哪一侧不确定）
        win = bx((WIN_XB - WIN_XA, 12, WIN_Z1 - WIN_Z0), ((WIN_XA + WIN_XB) / 2, sy * (W / 2 + CLR + 6), (WIN_Z0 + WIN_Z1) / 2))
        win2 = bx((WIN2_XB - WIN2_XA, 12, WIN_Z0 + 0.01 - zf), ((WIN2_XA + WIN2_XB) / 2, sy * (W / 2 + CLR + 6), (zf + WIN_Z0 + 0.01) / 2))
        cradle = diff(cradle, win, win2)
        # 窗上方的凸台外伸段（y 13.1..17.1）底面是 4mm 悬挑 → 45° 倒角切掉
        yw = W / 2 + CLR + WALL          # 13.1 墙外侧面
        yo = BOSS_Y + 3.0                # 17.1 凸台外侧面
        pts = np.array([[x, sy * y, z] for x in (-8, 2) for (y, z) in ((yw, WIN_Z1 - 0.01), (yo, WIN_Z1 - 0.01), (yo, WIN_Z1 + (yo - yw)))])
        cradle = diff(cradle, trimesh.convex.convex_hull(pts))
    for sy in (1, -1):   # 接线窗 +x 端的立柱只有 2.4×2.3，向外加厚到 y ±14.5（09-13 起摆臂杯段在 x≥11.35、r 9.25，与本立柱内面 y 10.4 留 1.15，转一圈不变）
        # 内侧从 W/2+CLR=10.4 起（保住 0.4 间隙）；底面在卡槽外缘 y 12.7 以外做 45° 斜切，不悬空
        yi, yo, yc = W / 2 + CLR, 14.5, W / 2 + WALL
        pts = np.array([[x, sy * y, z] for x in (WIN_XB, X_LOW_WALL) for (y, z) in ((yi, floor_z), (yc, floor_z), (yo, floor_z + (yo - yc)), (yo, 12.0), (yi, 12.0))])
        cradle = union(cradle, trimesh.convex.convex_hull(pts))
    # ── 立柱：卡槽底到桌面，空心 3mm 壁；x 方向只到 +13 不挡摆臂
    col_top = floor_z + 0.01
    col_h = col_top - z_table
    X_ARM_IN = T / 2 + S["flange_h"] + BOSS_H        # 摆臂杆内侧面（贴法兰的凸台外）：法兰面 13.0 + 2 = 15.0（旧 14.85）
    x_lo, x_hi = X_OUT_NEG, X_ARM_IN - 6.0 - 3.5   # 立柱 +x 面：M6 杯头 6 高 + 3.5 余量 → 5.35
    col_o = bx((x_hi - x_lo, W + 2 * WALL, col_h), ((x_hi + x_lo) / 2, 0, (col_top + z_table) / 2))
    # 内腔向下开口到桌面（不做封闭空腔，切片直接是壳）
    col = col_o          # 实心（打印时靠切片填充），避免空心顶上 19×25 的桥接
    # ── 底板 + 背鳍/侧鳍
    base = union(bx((160, 30, 4), (-5, 0, z_table + 2.0)), bx((30, 160, 4), (-5, 0, z_table + 2.0)))   # 160×160 十字：配重从 80° 放下时不翻（仍要求夹紧）

    # 鳍从桌面 z_table 起（与底板重叠 4mm），不能从底板顶面起——曾因底板改薄 1mm 导致鳍悬空
    fin_back = bx((28, 3, col_h * 0.75), (X_OUT_NEG - 14, 0, z_table + col_h * 0.75 / 2))   # 背鳍 x -43.7..-15.7，在底板 -85..75 之内
    fins = [fin_back]
    for sy in (1, -1):
        fins.append(bx((3, 18, col_h * 0.5), (X_OUT_NEG + 1.9, sy * (W / 2 + WALL + 9), z_table + col_h * 0.5 / 2)))
    # 卡槽底 +x 侧悬出立柱 7.4mm → 45° 楔形斜撑，从立柱面 x=8 起
    xw = X_LOW_WALL                 # 卡槽 +x 外侧面
    pts = np.array([[x_hi, -(W/2+WALL), floor_z + 0.01], [xw, -(W/2+WALL), floor_z + 0.01], [x_hi, -(W/2+WALL), floor_z - (xw - x_hi)],
                    [x_hi,  (W/2+WALL), floor_z + 0.01], [xw,  (W/2+WALL), floor_z + 0.01], [x_hi,  (W/2+WALL), floor_z - (xw - x_hi)]])
    wedge = trimesh.convex.convex_hull(pts)
    m = union(cradle, col, base, wedge, *fins)
    # 背面副轴让位：Ø20 向上开口的 U 槽（圆心 z=-1.5）。实测副轴端面 -12.8 已与背墙内面 -13.25 不顶，U 槽是双保险：
    # 5a 若比推算大 0.5 也不顶；封闭圆孔会让副轴被孔上方的墙挡住放不进去（第 4 轮复审发现）
    U_D, U_Z = 20.0, -1.5
    m = diff(m, cyl(U_D, 12, (XBL - CLR - 4, 0, U_Z), axis="x"),
                bx((12, U_D, 14.0), (XBL - CLR - 4, 0, U_Z + 7.0)))
    # 十字底板四端各一个 Ø4.5 孔（可选：螺丝/夹具固定到桌板）
    for c in ((-5 + 73, 0), (-5 - 73, 0), (-5, 73), (-5, -73)):
        m = diff(m, cyl(4.5, 8, (c[0], c[1], z_table + 2.0), axis="z"))
    return m

HORN_R_ARM = S["horn_r"]   # 2026-09-13 分度圆定案：用户拿 09-07 版摆臂（r=4.75）往法兰上扣，中心对齐后只对得上 1 孔、其余沿径向外偏 → 手册 Ø10.50 为真。
                           # 09-13 白天测试台先单独写死 5.25；同日晚 s288.py 的 horn_r 已改 5.25（全机同源），这里改回直接读 S 并断言，不再各走各的。
assert abs(HORN_R_ARM - 5.25) < 1e-9, f"S288 分度圆已实物定案 r=5.25，s288.S['horn_r']={HORN_R_ARM}；改了 S 要先确认摆臂"
POCKET_D, POCKET_WRAP = 14.5, 1.5   # 舵盘内嵌口袋：Ø14.5（法兰实测 13.9 / 手册 Ø14；竖直内轮廓按 TC02 缩 0.15/边 → 打出来 ≈14.2，单边 0.15 定位间隙 = 与 Ø2.6 过孔对 M2 的浮动同量级；大孔缩得少也只到 0.2/边，不会套不进）；
                                    # 包住法兰 1.5（法兰凸出 3.0，用户 09-13 要求"包一点就能感觉到套上了"）。口袋底 = 法兰面 13.0（旧 12.85），仍是承压面。
HUB_D_CUP = 18.5        # 口袋段外径：壁 (18.5−14.5)/2 = 2.0；口袋段伸到 x=11.35，比 +x 矮墙切口 10.35 还外 1.0，且 r 9.25 < 卡槽 ±y 墙内面 10.4，转一圈都碰不到墙顶

def build_arm():
    """摆臂舵盘端 = 一个套在法兰上的杯：Ø14.5 口袋深 1.5 把 Ø14 法兰包住（对孔靠法兰外圆定位，不再靠螺丝找孔），
    口袋底贴法兰面；口袋底之外仍是 7 mm 厚（M2×10 + Ø5 垫圈咬法兰 2.5），外侧面与臂杆外侧面齐平（打印时朝下贴床，杯口朝上）。"""
    x_fl = S["T"] / 2 + S["flange_h"]                    # 13.0 法兰面 = 口袋底（旧 12.85）
    x_in = x_fl - POCKET_WRAP                            # 11.35 杯口端面
    x_c = S["T"] / 2 + S["flange_h"] + BOSS_H + ARM_T / 2   # 17.5 臂杆中面（旧 17.35；随法兰面 +0.15，立柱 +x 面 x_hi 同样从它推）
    cup = cyl(HUB_D_CUP, x_c - ARM_T / 2 - x_in, ((x_in + x_c - ARM_T / 2) / 2, 0, 0), axis="x")   # Ø18.5，x 11.35..14.85
    hub = cyl(24.0, ARM_T, (x_c, 0, 0), axis="x")                                                  # Ø24，x 14.85..19.85
    bar = bx((ARM_T, ARM_W, ARM_LEN - 6), (x_c, 0, -(ARM_LEN - 6) / 2 - 10))   # 从毂下缘 z=-10 起，不盖住舵盘孔区
    end = cyl(16.0, ARM_T, (x_c, 0, -ARM_LEN), axis="x")
    m = union(cup, hub, bar, end)
    # 口袋 + 杯口 0.5×45° 导入倒角（法兰一放就滑进去；倒角做成两圆的凸包）
    pocket = cyl(POCKET_D, POCKET_WRAP + 0.02, (x_fl - POCKET_WRAP / 2 - 0.01, 0, 0), axis="x")
    ring = lambda d, x: np.array([[x, d / 2 * math.cos(t), d / 2 * math.sin(t)] for t in np.linspace(0, 2 * math.pi, 96, endpoint=False)])
    chamfer = trimesh.convex.convex_hull(np.vstack([ring(POCKET_D + 1.0, x_in - 0.01), ring(POCKET_D, x_in + 0.5)]))
    # 舵盘孔：分度圆 r=5.25（Ø10.5，09-13 实物定案），6 个 Ø2.6 圆孔（M2 留 ±0.3 位置误差）；中心 Ø5 通孔 + 口袋底 Ø8×3.0 沉台让中心螺钉头
    # （沉台 r=4 > 孔内沿 3.95，口袋底那面 6 个孔和沉台连通，承压面在外侧面，是故意的）
    cuts = [pocket, chamfer,
            cyl(S["horn_center_d"], 30, (x_c, 0, 0), axis="x"),          # Ø5 通孔
            cyl(8.0, 3.0, (x_fl + 1.5, 0, 0), axis="x")]                # Ø8×3.0 沉台：法兰中心螺钉头没量，但 6 个螺孔内沿在 r 3.95，头径几何上不可能 >Ø7.9；高 3.0 盖住任何盘头
    for k in range(S["horn_n"]):
        a = math.radians(k * 360 / S["horn_n"])
        cuts.append(cyl(2.6, 30, (x_c, HORN_R_ARM * math.cos(a), HORN_R_ARM * math.sin(a)), axis="x"))
    cuts.append(cyl(6.5, 20, (x_c, 0, -ARM_LEN), axis="x"))
    return diff(m, *cuts)

def build_strap():
    zb = 12 + STRAP_LIFT
    m = union(bx((8, STRAP_SPAN + 8, 3.0), (-3, 0, zb + 1.5)), bx((8, W - 1.0, zb - PAD_BOT), (-3, 0, (zb + PAD_BOT) / 2)))
    return diff(m, cyl(2.6, 10, (-3, STRAP_SPAN / 2, zb + 1.5), axis="z"),      # Ø2.6 过孔：贴床面象脚后仍 >2.0，压条必须能被螺丝拉下来
                   cyl(2.6, 10, (-3, -STRAP_SPAN / 2, zb + 1.5), axis="z"))

def floating_check(m, name):
    """打印朝向下的悬空检查：每个朝下的面，从其质心往下射线，打不到自身任何面 = 悬空（需支撑）。忽略贴床面。"""
    zmin = m.bounds[0][2]
    down = m.face_normals[:, 2] < -0.75          # 只看比 41° 更平的朝下面（≤45° 斜面可直接打）
    cen = m.triangles_center[down]; nz = m.face_normals[down][:, 2]
    sel = cen[:, 2] > zmin + 0.2
    cen = cen[sel]; areas = m.area_faces[down][sel]
    if len(cen) == 0: print(f"   {name}: 无悬空面"); return 0.0
    origins = cen - np.array([0, 0, 0.05])
    locs, idx, _ = m.ray.intersects_location(origins, np.tile([0, 0, -1.0], (len(origins), 1)), multiple_hits=False)
    gap = np.full(len(origins), np.inf)
    if len(idx): gap[idx] = origins[idx, 2] - locs[:, 2]
    bad = areas[gap > 0.3].sum() / 100
    worst = gap[np.isfinite(gap)].max() if np.isfinite(gap).any() else float("inf")
    print(f"   {name}: 下方有空隙的朝下面积 {bad:.2f} cm²（最大空隙 {worst:.1f} mm，∞=完全悬空 {areas[~np.isfinite(gap)].sum()/100:.2f} cm²）" + ("  ← 检查是否为可接受桥接/孔顶" if bad > 0.05 else "  OK"))
    return bad

def check(m, name):
    parts = m.split(only_watertight=False)
    if len(parts) > 1:
        for q in parts: print("   body:", round(q.volume/1000,2), "cm³ bbox", np.round(q.bounds,1).tolist())
    print(f"{name}: watertight={m.is_watertight} bodies={len(parts)} vol={m.volume/1000:.1f}cm³ "
          f"bbox={np.round(m.extents,1)} ≈{m.volume/1000*1.24:.0f}g PLA")

if __name__ == "__main__":
    base, arm, strap = build_base(), build_arm(), build_strap()
    servo = s288.servo_mesh()
    check(base, "TB01_base"); check(arm, "TB02_arm"); check(strap, "TB03_strap")
    # 干涉：零位 + 摆臂 ±80° 扫掠 vs 底座
    for ang in (0, 40, 80, -40, -80):
        a = arm.copy(); R = trimesh.transformations.rotation_matrix(math.radians(ang), [1, 0, 0]); a.apply_transform(R)
        hit = trimesh.boolean.intersection([a, base], engine="manifold")
        print(f"  arm@{ang:+d}° vs base: {hit.volume/1000:.3f} cm³"); assert hit.volume < 1e-6, ang
        pts, _ = trimesh.sample.sample_surface(a, 20000); _, d, _ = trimesh.proximity.closest_point(base, pts)
        print(f"     最小间隙（表面采样）{d.min():.2f} mm")
    hit = trimesh.boolean.intersection([servo, base], engine="manifold"); print(f"  servo vs base: {hit.volume/1000:.3f} cm³")
    # 放入路径：舵机（含法兰 + 背面副轴）从 z=+30 竖直下落到坐底（含坐底位），每 0.5mm 一步，除挤压筋外不得与底座相交
    # 副轴取 手册 Ø14×3（S）、Ø16×4（最坏）两种，轴心相对槽底再低 0 和 1.5 两种
    for (dd, hh) in ((S["rear_boss_d"], S["rear_boss_h"]), (16.0, 4.0)):
        for zoff in (0.0, -1.5):
            # 外壳不变（坐槽底定位），只有法兰和副轴相对外壳下移 zoff
            sv_ins = union(s288.servo_mesh(with_boss=False),
                           cyl(S["flange_d"], S["flange_h"], (T / 2 + S["flange_h"] / 2, 0, zoff), axis="x"),
                           cyl(dd, hh, (-T / 2 - hh / 2, 0, zoff), axis="x"))
            worst = 0.0
            for dz in list(np.arange(30.0, -CLR - 0.01, -0.5)) + [-CLR]:
                t = sv_ins.copy(); t.apply_translation((0, 0, dz))
                worst = max(worst, trimesh.boolean.intersection([t, base], engine="manifold").volume)
            print(f"  舵机竖直放入路径（副轴 Ø{dd:.1f}×{hh:.1f}，轴心偏 {zoff:+.1f}）：最大相交 {worst:.0f} mm³（筋过盈 ≈5 以内正常，>20 = 放不进去）")
            assert worst < 20, (dd, hh, zoff, worst)
    hit = trimesh.boolean.intersection([servo, arm], engine="manifold");  print(f"  servo vs arm : {hit.volume/1000:.3f} cm³"); assert hit.volume < 1e-6
    sv = servo.copy(); sv.apply_translation((0, 0, -CLR))    # 舵机实际坐在卡槽底 Z_BOT-CLR
    hit = trimesh.boolean.intersection([sv, base], engine="manifold"); print(f"  舵机 vs 卡槽挤压筋 过盈体积 {hit.volume:.0f} mm³（单边过盈 0.05，12 条筋，应 ≈ 5，装入时压扁）")
    wall_top = base.bounds[1][2]; strap_bot = strap.bounds[0][2]; strap_bar_bot = strap.slice_plane([0, STRAP_SPAN / 2, 0], [0, 1, 0]).bounds[0][2]
    print(f"  墙顶 {wall_top:.2f}，压条杆底 {strap_bar_bot:.2f}（高出墙顶 {strap_bar_bot - wall_top:.2f}，应 ={STRAP_LIFT}），压垫底 {strap_bot:.2f}（应 ={PAD_BOT}）")
    assert abs(wall_top - 12) < 1e-6 and abs(strap_bot - PAD_BOT) < 1e-6 and abs(strap_bar_bot - wall_top - STRAP_LIFT) < 1e-6
    hit = trimesh.boolean.intersection([sv, strap], engine="manifold"); print(f"  压条压垫 vs 舵机顶 过盈体积 {hit.volume:.0f} mm³（设计 0.2mm×8×19≈30，>0 才算压住）")
    # 接线窗：舵机整个侧面（x -12.85..+9.95 × z 槽底..-1.5）1mm 网格向外射线，被挡的点只允许落在下段窗两端的角柱（x<-10.8 或 x>7.6，且 z<-20.5）
    blocked = []
    for sy in (1, -1):
        for xx in np.arange(XBL_G + 0.5, T / 2, 1.0):
            for zz in np.arange(ZF_G + 0.5, -1.5, 1.0):
                o = np.array([[xx, sy * (W / 2 - 0.5), zz]]); loc, _, _ = base.ray.intersects_location(o, np.array([[0, sy * 1.0, 0]]), multiple_hits=False)
                free = abs(loc[0][1]) - W / 2 if len(loc) else 99
                if free < 30: blocked.append((sy, xx, zz))
    stray = [b for b in blocked if not (((b[1] < WIN2_XA or b[1] > WIN2_XB) and b[2] < S["conn_z"][0] - 4.0) or b[1] > WIN_XB)]
    print(f"  接线窗：侧面 {len(blocked)} 个网格点被挡，全在角柱/前立柱内 = {not stray}（角柱：x<{WIN2_XA:.1f} 或 x>{WIN2_XB:.1f} 且 z<{S['conn_z'][0]-4:.1f}；前立柱盖住 x>{WIN_XB:.1f} 的 0.85mm；插座离舵机底 ≤4.4 且离前/后边 ≤2 才会碰）")
    assert not stray, stray[:5]
    x_c = S["T"] / 2 + S["flange_h"] + BOSS_H + ARM_T / 2
    head = cyl(10.0, 6.0, (x_c - ARM_T / 2 - 3.0, 0, -ARM_LEN), axis="x")   # DIN912 M6 杯头 Ø10×6 在摆臂 −x 侧（贴臂面）
    stack = cyl(30.0, 20.0 + 5.0, (x_c + ARM_T / 2 + 12.5, 0, -ARM_LEN), axis="x")   # 垫片摞 20 + 螺母 5 在 +x 侧
    for ang in range(0, 360, 10):
        h = union(head.copy(), stack.copy()); h.apply_transform(trimesh.transformations.rotation_matrix(math.radians(ang), [1, 0, 0]))
        vol = trimesh.boolean.intersection([h, base], engine="manifold").volume + trimesh.boolean.intersection([h, strap], engine="manifold").volume
        assert vol < 1e-6, f"配重@{ang}° 与底座/压条相交 {vol:.0f} mm³"
    print("  配重（M6 头 + 垫片摞 + 螺母）整圈 360° 每 10° 扫掠完成（无 !! 即不碰底座、压条）")
    for ang in (0, 80, -80):
        h = head.copy(); h.apply_transform(trimesh.transformations.rotation_matrix(math.radians(ang), [1, 0, 0]))
        hit = trimesh.boolean.intersection([h, base], engine="manifold"); _, dh, _ = trimesh.proximity.closest_point(base, h.vertices)
        print(f"  M6 头@{ang:+d}° vs base: {hit.volume/1000:.3f} cm³, 间隙 {dh.min():.1f} mm")
    # 导出：底座底面放到 z=0；摆臂平放（厚度沿 z）；压条平放
    b = base.copy(); b.apply_translation((0, 0, AXIS_H)); b.export(os.path.join(OUT, "TB01_base.stl"))
    a = arm.copy(); a.apply_transform(trimesh.transformations.rotation_matrix(math.radians(+90), [0, 1, 0])); a.apply_translation((0, 0, -a.bounds[0][2])); a.export(os.path.join(OUT, "TB02_arm.stl"))
    s = strap.copy(); s.apply_transform(trimesh.transformations.rotation_matrix(math.pi, [1, 0, 0])); s.apply_translation((0, 0, -s.bounds[0][2])); s.export(os.path.join(OUT, "TB03_strap.stl"))   # 压垫朝上打
    print("打印朝向悬空检查：")
    for nm, mm in (("TB01_base", b), ("TB02_arm", a), ("TB03_strap", s)): floating_check(mm, nm)
    scene = trimesh.util.concatenate([base, arm, strap, servo]); scene.export(os.path.join(OUT, "assembly_preview.stl"))
    print("exported →", os.path.abspath(OUT))
