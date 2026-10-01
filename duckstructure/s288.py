#!/usr/bin/env python3
"""宇树 Unitree S288 无刷数字舵机的参数与几何原语（鸭子整机 duck.py 与小车 duckcar.py 共用）。

局部坐标（"舵机坐标系"）：输出轴沿 +X，**法兰面朝 +X**；长边沿 Z，顶端 z=+top(9.5)，底端 z=-(L-top)；宽 20 沿 Y。
2026-09-13 起本体厚/台阶/法兰/副轴/分度圆**全部按手册尺寸图 p2**（docs/reference/s288/S288使用手册_含尺寸图_2026-09-10.pdf；
用户拍板："我自己量的不准，所有参数按手册"）。标 [量] 的只剩手册没有的项：插座缺口的 y/z 范围由同一张图像素反推（±0.3mm）；插座**方向**（背插、插到底与 23 面齐平）
2026-09-20 由用户实物照片定。旧卡尺实测值留在各行注释里作历史。
"""
import math, numpy as np, trimesh
from trimesh.creation import box, cylinder
from trimesh.transformations import rotation_matrix as rot

S = dict(
    # ── 本体/法兰/副轴/分度圆：手册尺寸图 p2（2026-09-13 用户拍板：自量不准，全按手册）。旧 2026-09-07 卡尺实测值保留在括号里作历史
    #    （标号见 04_电机到货测试/S288卡尺测量示意图.svg）。
    L=34.0, W=20.0, T=20.0,      # 长 34 × 宽 20 × 轴心段（薄段）厚 T=20：手册尺寸图 p2（2026-09-13 用户拍板：自量不准，全按手册）；旧实测 8c=19.9
    T_lo=23.0,                   # 接线端（厚段）厚 T_lo=23：手册尺寸图 p2（2026-09-13 用户拍板：自量不准，全按手册）；旧实测 8d=22.8。法兰面是一整个平面，台阶/斜坡全在背面
    step_z=(-7.0, -13.5),        # 背面斜坡 (薄段下边 z, 厚段上边 z)：手册尺寸图 p2（2026-09-13 用户拍板：自量不准，全按手册）—— 主线程 400 dpi 像素读数（±0.3）：
                                 # 厚段从接线端起约 10.8（→ z≈-13.7），斜坡约 6.4，薄段从 z≈-7.3 到 +9.5；取 (-7.0, -13.5)：比读数 -13.7 保守 0.2（厚段略长）。旧值 (-7.0, -12.0) 是"没量、按最保守估"
                                 # （干涉检查用；测试台内腔和载体包络各自另有更保守的处理，见 body_prism(stepped=False) / servo_envelope）
    top=9.5,                     # 轴心到顶端 9.5：手册尺寸图 p2（实测同为 9.5）
    flange_d=14.0, flange_h=3.0, # 法兰 Ø14 高 3：手册尺寸图 p2（2026-09-13 用户拍板：自量不准，全按手册）；旧实测 3=13.9 / 4≈2.9（由总厚 25.7 反推）
    horn_n=6, horn_r=5.25,       # 法兰上 6-Ø1.7 ⌄3.0(MAX) M2 自攻孔，分度圆 Ø10.50 → r=5.25：手册尺寸图 p2（2026-09-13 用户拍板：自量不准，全按手册）；旧值 4.75（"实测对孔中心距 9.5"）
    # 2026-09-13 实物定案：用户拿 09-07 版 TB02 摆臂（r=4.75）往法兰上扣，中心对齐后只对得上 1 孔、其余沿径向外偏 → Ø10.5 为真，r=5.25。
    #    证据链（手册逐像素 Ø10.37/Ø10.49/Ø10.34 三次独立测量 + "实测 9.5" 自相矛盾）见 docs/reports/分度圆Ø9.5还是Ø10.5_2026-09-12.md。
    #    横跨法兰与惰轮的所有孔都从这一个数推出来（horn_holes / idler_holes / horn_cbore / idlercheck.seat_radii 都引用它）。
    horn_hole_d=2.4, horn_center_d=5.0,   # 法兰螺孔实测 9≈1.9（M2 攻牙孔）；我们件上的过孔 2.2→2.4（09-13 用户拍板『有则改之』：TC02 孔规实测本机竖直小孔缩 0.15/边，Ø2.2 打出来 1.90 穿不过 M2 → 过孔名义 2.4（打出来 ≈2.1）；原版 XL330 版是 2.2 但人家机器不缩。）；测试台摆臂中心 Ø5 过孔 + Ø8×3.0 沉台让中心螺钉
    rear_boss_d=14.0, rear_boss_h=3.0,    # 背面副轴 Ø14 高 3：手册尺寸图 p2（2026-09-13 用户拍板：自量不准，全按手册）；旧实测 5b=13.8 / 5a≈2.9。
                                          # 副轴端面 x=-T/2-3=-13.0 与厚段背面 T/2-T_lo=-13.0 **齐平**（手册 20+3=23）
    # ── 背面副轴 = **自由转动的惰轮**（2026-09-11 用户拿实物确认：能单独拨动，拿连接件接上去它就跟着转）。
    # 手册尺寸图 p2（docs/reference/s288/S288使用手册_含尺寸图_2026-09-10.pdf，400dpi 放大逐像素量）证明
    # **正反两端是同一个接口**：侧视图两端各一个 Ø14 高 3 的凸台；背视图与正视图都标
    # "6-Ø1.7 ⌄3.0(MAX) 使用M2自攻螺丝" + "Ø10.50"。→ 惰轮孔位 = 法兰孔位，horn_n/horn_r/horn_hole_d/horn_depth 直接通用（两端 r=5.25，09-13 定案）。
    idler_center_d=5.0,                   # 从动件中心让位孔：和法兰同口径（中心是同一种十字螺钉），不要大过分度圆-孔半径
    idler_hole_d=2.6,                     # 惰轮侧 6 颗 M2 的过孔（法兰侧仍是 horn_hole_d=2.2）。故意放大：惰轮侧的毂是**第二个**径向定位，
                                          # 件的位置已经由法兰侧定死，这 6 颗只能"跟着惰轮走"、不能再定位 —— Ø2.6 按 tolerances.yaml 的 −0.30 打印偏移后
                                          # 仍有 ±0.15 浮动（Ø2.2 打出来只有 1.90，M2 都穿不过，见 tolerances.yaml:screened_against_PX02）。
    mnt_dx=8.0, mnt_z=(7.5, -22.5), mnt_hole_d=2.4,   # 背面 4 个角孔（我们的过孔 2.2→2.4，理由同 horn_hole_d）：官方尺寸图 16×30、离两端各 2；Ø1.7 光孔 M2 自攻、深 ≤3.0
                                                      # （卡尺量的 14.7×28.1 是孔内沿到内沿，差一个孔径，以官方为准）
    horn_depth=3.0, mnt_depth=3.0,                     # 法兰 6 孔 / 角孔 螺纹深度 3.0 MAX（官方图）：螺丝进舵机的长度按 3 算
    # ── PH2.0 插座 ×2（2026-09-20 用户拿实物照片推翻 09-11 的"侧插"模型，见 docs/design_2026-09-17_bearing_rebuild/电子件尺寸核实_2026-09-19.md §8）：
    #    座嵌在**背面（惰轮面）接线端两角**各一个向背面、向侧面都开的缺口里，针尖朝 −x → 插头沿 x **从背面**插/拔；
    #    插到底后插头外端与厚段背面 x=−13.0 **齐平**（不外凸）。线从插头顶出来先**直立一段再弯**（原配线硬），2026-09-21 用户拿实物看：
    #    翘起约 4 mm（09-20 口述 1–2 是估的）→ wire_out=4.0 + wire_clr=0.4，让位到 x=−17.4。3 mm 背板兜不住 → pocket 型（背板内面挖槽）作废，
    #    盖住插座的背板一律开通窗、线从背板外面走（lib.CONN_MODE 只剩 window/free/none）。
    #    缺口的 y/z 范围是 09-11 手册尺寸图的像素反推（±0.3）；09-21 用户定：图纸比卡尺准，不再实测。让位刀见 lib.conn_cut；Gate 独立实体见 keepouts.yaml:KO01。
    conn_x=(-13.0, -3.4),        # 缺口 x 区间：厚段背面 −13.0 → 缺口底 −3.4（旧模型 (−10,−3.4) 把薄段背面当成了侧窗上沿）
    conn_z=(-13.9, -4.7),        # 缺口 z 区间（像素反推）；载体两侧壁在这一段照旧不封（pocket 型载体的出线口）
    plug_body=(4.5, 7.8, 6.85),  # JST PHR-3 插头本体 (y 厚, z 宽, x 高=拔插方向)
    plug_clr=0.5,                # 插头四周间隙
    plug_top_x=-13.0,            # 插合态插头外端 x = 厚段背面（实物：不外凸）
    wire_out=4.0,                # 线在插头外端直立翘起的高度（2026-09-21 用户实物目测 ≈4，原配线硬）
    wire_clr=0.4,                # 翘起高度的余量 → 背板/邻件在 x ∈ [−17.4, −13.0] 的插头足印内不许有料（lib.conn_zone "A"）。0.4 而不是 0.5：
                                 #   −17.4 正好是 L07 踝后臂 6700 座环端面（lib.ANK_X0），座环不必削（hr23 削了 0.4 把 L2 L07-F01/F02 座深/座径量歪）
    plug_y_in=5.0,               # 插头足印从机身侧面向内 4.5 + 0.5 → |y| ∈ [W/2−5.0, W/2+plug_clr]
    clr=0.3, wall=2.5,
    plate_t=5.0,                 # 背面安装板厚：前 2.5 让位背凸台，后 2.5 承力
    stub_d=10.0, stub_h=4.5,     # 背板外侧同轴短轴，套 6700ZZ 轴承（10×15×4）
    brg_od=15.0, brg_t=4.0, brg_id=10.0,
    mass_g=19.5,                 # 官方参数表（2026-09-08 客服给的）；J288 是 35 g
)

def cyl(d, h, center=(0, 0, 0), axis="z", sections=64):
    c = cylinder(radius=d / 2, height=h, sections=sections)
    if axis == "x": c.apply_transform(rot(math.pi / 2, [0, 1, 0]))
    if axis == "y": c.apply_transform(rot(math.pi / 2, [1, 0, 0]))
    c.apply_translation(center); return c

def bx(size, center=(0, 0, 0)):
    b = box(extents=size); b.apply_translation(center); return b

def union(*ms):
    ms = [m for m in ms if m is not None]
    return ms[0] if len(ms) == 1 else trimesh.boolean.union(ms, engine="manifold")
def diff(a, *ms):
    ms = [m for m in ms if m is not None]
    return a if not ms else trimesh.boolean.difference([a, *ms], engine="manifold")
def inter(*ms): return trimesh.boolean.intersection(list(ms), engine="manifold")

def zc(): return S["top"] - S["L"] / 2          # 本体 z 中心

def body_prism(c=0.0, stepped=True, z_ext=0.0):
    """本体外壳（不含法兰/副轴）。背面是台阶：上半段（轴心区）厚 T，下半段（接线端）厚 T_lo，中间一段斜坡。
    法兰面是一整个平面，x = +T/2。c=单边间隙；z_ext=顶端向上延伸（挖卡槽时用来开口）。
    stepped=False 或 step_z=None → 整段都按 T_lo 算（保守：卡槽这么挖一定塞得进去，只是上半段背后空 T_lo-T）。
    侧面轮廓有一个凹角，用 shapely 多边形拉伸。"""
    T, T_lo, W, L, top = S["T"], S["T_lo"], S["W"], S["L"], S["top"]
    x_f, x_bh, x_bl = T / 2 + c, -T / 2 - c, T / 2 - T_lo - c
    z_t, z_b = top + c + z_ext, top - L - c
    step = S["step_z"] if stepped else None
    if step is None:
        prof = [(x_f, z_t), (x_f, z_b), (x_bl, z_b), (x_bl, z_t)]
    else:
        s0, s1 = step                      # s0=薄段下边（高），s1=厚段上边（低）
        prof = [(x_f, z_t), (x_f, z_b), (x_bl, z_b), (x_bl, s1 - c), (x_bh, s0 + c), (x_bh, z_t)]
    # 轮廓不是凸的（斜坡→竖直面那个角是凹角），必须拉伸而不能凸包
    from shapely.geometry import Polygon
    y = W / 2 + c
    m = trimesh.creation.extrude_polygon(Polygon(prof), 2 * y)     # 2D (u,v)=(x,z)，沿 +z 拉伸 2y
    m.apply_transform(rot(math.pi / 2, [1, 0, 0]))                  # (u,v,ext) → (x, -ext, v)
    m.apply_translation((0, y, 0))
    return m

def servo_mesh(with_boss=True):
    """舵机占位实体（干涉检查/预览用）——按实测的台阶外形"""
    m = body_prism()
    if with_boss:
        m = union(m, cyl(S["flange_d"], S["flange_h"], (S["T"] / 2 + S["flange_h"] / 2, 0, 0), axis="x"),
                  cyl(S["rear_boss_d"], S["rear_boss_h"], (-S["T"] / 2 - S["rear_boss_h"] / 2, 0, 0), axis="x"))
    return m

def servo_envelope(extra=0.0):
    """本体 + 背面凸台 + 间隙 的包络（从连杆里挖掉）。法兰侧不挖：从动件本来就贴在法兰面上。
    副轴让位：Ø(rear_boss_d+2) 从薄段背面一直到 x_env_boss_face()=-14.0（惰轮端面 -13.0 再外 1.0；c 只放大直径，不改外端面）。"""
    c = S["clr"] + extra
    # 手册 step_z=(-7,-13.5)，但包络仍按更保守的"z>+2 才薄、-1 以下全厚"挖（09-13 起手册有了斜坡位置也不放松：
    # 这样载体背板上排孔的垫柱（z 5..10）不会被包络挖掉，而任何真实斜坡位置都装得进去；载体/从动件对台阶只按这个保守面让）
    saved = S["step_z"]; S["step_z"] = (2.0, -1.0)
    m = body_prism(c=c, stepped=True); S["step_z"] = saved
    m = union(m, cyl(S["rear_boss_d"] + 2.0, S["rear_boss_h"] + 1.0 + c, (-S["T"] / 2 - (S["rear_boss_h"] + 1.0 + c) / 2 + c, 0, 0), axis="x"))
    return m

def horn_holes(thick, center=(0, 0, 0), axis="x"):
    """法兰 6 孔 + 中心让位，沿 axis 打穿 thick"""
    hs = [cyl(S["horn_center_d"], thick + 2, axis=axis)]
    for k in range(S["horn_n"]):
        a = math.radians(k * 360 / S["horn_n"])
        u, v = S["horn_r"] * math.cos(a), S["horn_r"] * math.sin(a)
        off = {"x": (0, u, v), "y": (u, 0, v), "z": (u, v, 0)}[axis]
        hs.append(cyl(S["horn_hole_d"], thick + 2, off, axis=axis))
    m = union(*hs); m.apply_translation(center); return m

def mount_holes(thick, center=(0, 0, 0)):
    """背面 4 个 M2 角孔（舵机坐标系，沿 x 打穿）"""
    hs = [cyl(S["mnt_hole_d"], thick + 2, (0, sx * S["mnt_dx"], z), axis="x") for sx in (1, -1) for z in S["mnt_z"]]
    m = union(*hs); m.apply_translation(center); return m

def idler_holes(thick, center=(0, 0, 0), axis="x"):
    """**惰轮**（背面副轴盘）6 孔（Ø idler_hole_d=2.6）+ 中心 Ø idler_center_d 让位，沿 axis 打穿 thick。舵机局部几何，用前必须 placed()。

    孔位（horn_r / horn_n / horn_hole_d）与法兰**完全共用一个数**，手册两端标注一模一样。
    相位也按 horn_holes 的 k×60°（从局部 +y 起）：没有任何"错开半格"的依据；
    真实相位在装配时靠件上的 6 孔自己找正（6 孔 60° 对称，最多转 60° 就能对上，不影响功能）。
    中心 idler_center_d 让开那颗十字螺钉，和法兰同口径。"""
    hs = [cyl(S["idler_center_d"], thick + 2, axis=axis)]
    for k in range(S["horn_n"]):
        a = math.radians(k * 360 / S["horn_n"])
        u, v = S["horn_r"] * math.cos(a), S["horn_r"] * math.sin(a)
        off = {"x": (0, u, v), "y": (u, 0, v), "z": (u, v, 0)}[axis]
        hs.append(cyl(S["idler_hole_d"], thick + 2, off, axis=axis))
    m = union(*hs); m.apply_translation(center); return m

def x_idler_face(): return -S["T"] / 2 - S["rear_boss_h"]              # -13.0：惰轮端面，从动件贴这里（旧 -12.85）

# ── 关键 x 站位（相对轴心，法兰朝 +x）。括号里是手册尺寸下的值（09-13 起），旧实测值另注
def x_flange_face(): return S["T"] / 2 + S["flange_h"]                 # 13.0：从动件舵盘板贴这里（旧 12.85）
def x_rear_face():   return -S["T"] / 2                                # -10.0：上半段（薄段）背面（旧 -9.95）
def x_rear_face_lo(): return S["T"] / 2 - S["T_lo"]                    # -13.0：下半段（接线端）背面（旧 -12.85）
def x_env_boss_face(): return x_idler_face() - 1.0                     # -14.0：servo_envelope 背面副轴让位的外端面（= 惰轮端面再外 1.0；间隙 c 在公式里抵消，见 servo_envelope）
def x_plate_out():   return x_rear_face() - S["plate_t"]               # -15.0：背板外表面（cradle 用；旧 -14.95）
def x_stub_end():    return x_plate_out() - S["stub_h"]                # -19.5（cradle 用）

def cradle(walls=("+z", "-z", "+y", "-y"), wall_depth=None, stub=True, plate_t=None, wall=None):
    """载体侧：背板（4×M2 拧进舵机背面角孔）+ 同轴短轴 + 可选侧墙。舵机从法兰面方向（+x）放入。
    侧墙 ±y 在插座 z 区留出线口（插头本身背插不外凸，见 conn_* 注释）。目前没有件用它（duck 用 lib.carrier / back_shell）。返回单个实体（舵机坐标系）。"""
    w = wall or S["wall"]; pt = plate_t or S["plate_t"]
    T, W, L, c = S["T"], S["W"], S["L"], S["clr"]
    d = wall_depth if wall_depth is not None else T           # 侧墙从背板向 +x 延伸多深
    xr = x_rear_face()
    plate = bx((pt, W + 2 * w, L + 2 * w), (xr - pt / 2, 0, zc()))
    parts = [plate]
    if stub:
        parts.append(cyl(S["stub_d"], S["stub_h"] + 0.01, (x_plate_out() - S["stub_h"] / 2, 0, 0), axis="x"))
    xw = xr + d / 2
    if "+z" in walls: parts.append(bx((d, W + 2 * w, w), (xw, 0, S["top"] + c + w / 2)))
    if "-z" in walls: parts.append(bx((d, W + 2 * w, w), (xw, 0, -(L - S["top"]) - c - w / 2)))
    for sgn, key in ((1, "+y"), (-1, "-y")):
        if key not in walls: continue
        yw = sgn * (W / 2 + c + w / 2)
        z_hi, z_lo = S["top"] + c + w, -(L - S["top"]) - c - w
        c0, c1 = S["conn_z"]
        parts.append(bx((d, w, z_hi - (c1 + 1.0)), (xw, yw, (z_hi + c1 + 1.0) / 2)))      # 插座区以上
        parts.append(bx((d, w, (c0 - 1.0) - z_lo), (xw, yw, (c0 - 1.0 + z_lo) / 2)))      # 插座区以下
    m = union(*parts)
    m = diff(m, mount_holes(pt + 2, (xr - pt / 2, 0, 0)),
             cyl(S["rear_boss_d"] + 1.5, S["rear_boss_h"] + 0.5 + 0.01, (xr - (S["rear_boss_h"] + 0.5) / 2, 0, 0), axis="x"))
    return m

def horn_plate(t=3.0, d=None):
    """从动件舵盘板：贴在法兰面上，6×M2 拧进法兰攻牙孔。返回 (实体, 要减掉的孔)"""
    d = d or S["flange_d"] + 1.0
    x0 = x_flange_face()
    solid = cyl(d, t, (x0 + t / 2, 0, 0), axis="x")
    holes = horn_holes(t + 2, (x0 + t / 2, 0, 0))
    return solid, holes

def rear_arm(t=None, d=None):
    """从动件背侧支臂：Ø15 轴承座套在载体背板的短轴上。返回 (实体, 要减掉的孔)"""
    t = t or (S["brg_t"] + 1.0); d = d or S["brg_od"] + 6.0
    x1 = x_plate_out() - 0.5                # 支臂内表面离背板 0.5
    solid = cyl(d, t, (x1 - t / 2, 0, 0), axis="x")
    seat = cyl(S["brg_od"] + 0.1, S["brg_t"] + 0.2, (x1 - S["brg_t"] / 2, 0, 0), axis="x")
    thru = cyl(S["brg_id"] + 1.0, t + 2, (x1 - t / 2, 0, 0), axis="x")
    return solid, union(seat, thru)

def frame(center, front, longdir):
    """舵机坐标系 → 目标坐标系的 4×4：x=front（法兰方向），z=longdir（顶端方向）"""
    ax = np.array(front, float); ax /= np.linalg.norm(ax)
    lg = np.array(longdir, float); lg -= ax * (lg @ ax); lg /= np.linalg.norm(lg)
    y = np.cross(lg, ax)
    R = np.eye(4); R[:3, 0] = ax; R[:3, 1] = y; R[:3, 2] = lg; R[:3, 3] = center
    return R

def placed(mesh, R):
    m = mesh.copy(); m.apply_transform(R); return m
