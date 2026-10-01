"""脖子 N01–N03：脖子载体 / 颈俯仰件 / 偏航-横滚件。"""
import math, numpy as np
from . import s288
from . import head_bearing_rebuild as HBR
from .s288 import S, cyl, union, diff, inter, placed
from .lib import (P, XR, PLT, CLR, Wl, conn_hump_sweep, memo, orig, sfw, drv_from, drv_self, pt, servo_env, carrier, mount_cut, mnt_cut2,
                  keep_main, wbox, fill_cyl, driven_patch, flange_relief, horn_cut, horn_cbore, sweep_of, wide_y,
                  conn_cut, conn_zone, idler_hub, idler_cut, idler_keep, idler_bore, x_bshell_out, region_cut, minkowski_box)
from duckstructure.build_fast import produce_extra      # hr46：件的附属实体（钩）接口

# ───────────────────────────── 脖子 ─────────────────────────────
NK_WEB_Z1 = 192.5          # 头俯仰载体背板/腹板的顶：再高就进 N02 原版脸颊的扫掠禁区
NK_WALL_X = ((13.5, 16.5), (35.5, 38.5))   # 兜住头俯仰舵机 20 宽两侧面(x=16/36)的墙。内缘故意压进 0.5 让 servo_env 自己切齐（留 0.3），
                                           # 外缘要和背板(x 16..38.8)真·体积重叠 —— 光靠共面贴着会生成退化面
NK_WALL_Z = (177.3, 184.5)                 # 墙/唇的 z 范围。下限：颈俯仰舵机顶端在 176.9，留 0.4（低了会挡住它沿 +y 轴向拆卸）。
                                           # 上限被两头夹死：S288 PH2.0 插座区从 z=188.5 起（conn_z=-13.9，09-11 图纸值；09-20 定为背插，线从侧面出，这一段仍不能封墙）；
                                           # 再往上 z>188.9 也全被 N02 顶盒的扫掠削光（顶盒扫出 r 10.45..17.7 的环，墙在 r≥11.4）
NK_LIP_Y, NK_LIP_Z1 = (S["T"] / 2 + CLR, s288.x_flange_face() - 0.5), 184.5   # 前唇 y（= 头俯仰舵机局部 x）：舵机前表面 10.0+0.3=10.3 起，到法兰面 13.0 下 0.5=12.5（旧写死 10.25..12.35）；
                                                                            # 顶 184.5 —— N02 顶盒扫掠的内半径 17.7 → z<202.42-17.7 才安全
# 底盖做不了：颈俯仰舵机顶端在 z=176.9、头俯仰舵机底端在 177.92，两颗之间只有 1.0mm。
# 试过在 174.4..177.4 加盖，零位姿被 servo_env 挖空看不出问题，但颈俯仰舵机沿 +y 轴向拆卸时会撞上它 105.9mm³。
NK_IDLER_BORE = 16.0
# was_until_2026_09_28_hr50: NK_BORE_RING = 1.0     # hr50 试过的"孔与窗之间 1.0 实环"，挡插头插入路径，撤回（见 build_neck 的 conn_cut 注释）
# N01 背板上让躯干右壳 T03 的惰轮毂（trunk.SHR_IDL，Ø15.0）穿过去的同轴通孔。09-13 分度圆定案 5.25 → 毂 14→15、本孔 15→16
# （docs/reports/分度圆Ø9.5还是Ø10.5_2026-09-12.md §定案后）。Ø16 是被三头夹出来的（09-12 实测 + 复核的 Ø15 版数字按 r 8.0 重算）：
#   · 上排两根 Ø5 垫柱中心在舵机局部 (±8, +7.5)、离轴 sqrt(8²+7.5²)=10.966 → 柱内缘 8.466；孔半径 8.0 留 0.466（旧 0.966）
#   · 上排角孔 Ø2.4 内缘离轴 9.766 → 孔壁到螺丝孔的韧带：背板中层 1.77 / Ø4.4 沉窝层(-16.3..-15.3) 0.77 / Ø16 让位层与本孔同径
#     —— 同一颗螺丝到背板外缘本来就只有 0.80（板边 y=-10.0、孔边 -9.2），沉窝层 0.77 与之同量级，是新的并列最弱处（不承力：那层只坐螺丝头）
#   · 毂 Ø15.0 → 单边间隙 0.50。毂与孔同心是几何保证（关节轴与舵机轴共线），间隙只需吃打印公差，不需要吃对中误差：
#     惰轮侧 6 颗螺丝用 Ø2.6 过孔"跟着惰轮走"，先紧这 6 颗再紧壳柱，让毂自己找到惰轮的位置（见 trunk.build_trunk_shell）
# 旧实测（Ø15 版）：开孔前后 N01 4.4922 → 4.0864 cm³（−405.8 mm³），仍是单体水密件，STL 洞边/非流形边 0。Ø16 版见 build_2026-09-13_manual_dims.log。

# ── hr45n01（2026-09-26 用户 06:40 定；主设计探索 hr43_work/hr45n01/n01_hook_R_spec.json）：+y 前唇顶面上的"双指卡槽"（非闭合挂钩）
# 线束 HB09_R（束径 5.26）从 +y 外侧按进槽里、被指顶的唇卡住，不用扎带。两根手指从前唇顶面 y 12.5 竖直长出（世界 +y）；
# N01 −y 朝下打印时手指就是竖直墙，指顶唇向槽内伸 0.8、唇下 54° 斜面 → 无悬垂、无支撑（build #4 的扎带侧桥悬空，支撑 15.7→1039.6 mm³，已撤）。
# was_until_2026_09_26_hr45n01: 唇下 45° 斜面 —— 切片实测 45° 每层外伸 0.2 > PrusaSlicer 判悬垂门槛 0.2/tan46° = 0.193 → 槽内两唇下整条长支撑
#   （N01 支撑 15.735 → 34.37 mm³）；斜面起点沿指内面下移 0.3（NK_HOOK under 0.8 → 1.1，54°，每层外伸 0.145）→ 支撑回到 15.735、槽内 0 点。
# 只做 +y（HB09_R）一侧：−y 面是背板 = 打印底面，放不了钩（主设计已否决换朝向：底面接触 715→63 mm²）。
# 槽心 x 26.5 而不是线原站点 x 24.5：颈俯仰后仰时后指（低 x 那根）碰 T02 颈口后沿 —— x 24.5 在 61.0° 首碰（真实姿态 5 个命中，
# neck_pitch 60.9..63.9），x 26.5 首碰 65.5°（目标上限 60、真实姿态最大 63.9 → 0 命中）；x 27.5 首碰 67.5° 但头极限组合多碰。
# 头（jaw_soft/H03）只在 head_pitch 90 + roll 25 + yaw −26..−96 这类极限组合碰指，这些姿态 N01 前唇本来就压进 H03（≤34 mm³），无新增碰撞姿态。
# 沿线长度 = 前唇全长 NK_WALL_Z（7.2）：下限护着颈俯仰舵机 +y 拆卸通道（z ≤176.9），上限护着 N02 顶盒扫掠（z<184.72）。
# 全部尺寸 assumed（束径 5.26 来自 harness HB09_R；槽宽 = 束径 + 0.4；指厚 1.2 / 高 5.5 / 唇 0.8×1.2 / 根部 0.5×45° 倒角），待试打。
# was_until_2026_09_26_hr45n01: NK_HOOK = dict(xc=26.5, od=5.26, clr=0.4, t=1.2, h=5.5, lip=0.8, lip_h=1.2, root=0.5, embed=0.3)
NK_HOOK = dict(xc=26.5, od=5.26, clr=0.4, t=1.2, h=5.5, lip=0.8, lip_h=1.2, under=1.1, root=0.5, embed=0.3)   # embed：指根往前唇里埋 0.3（真·体积重叠，免共面退化面）；
                                                                                                    # under：唇下斜面的竖直落差（lip 0.8 / under 1.1 → 54°；=lip 即 45°）

# ── hr45n01 补充（2026-09-26 用户 07:4x 定"左侧也要钩，打印靠支撑，自己拆"）：−y 背板外面（y −16.3）上的同款双指卡槽，线束 HB09_L（束径 5.96）。
# 按 HB09_L 重算：槽 5.96+0.4 = 6.36、指 1.2、唇 0.8×1.2、唇下 54°；指高 6.0 —— 束心贴槽底 = −16.3 − 2.98 = −19.28，
# 唇尖处束面离束心 sqrt(2.98² − (3.18 − 0.8)²) = 1.79 → 唇带须在 y ≤ −21.07，取 −21.1..−22.3（净距 0.03）。
# x 26.5、z 178.3..185.5（比 R 钩整体上移 1.0）：两侧是头俯仰舵机下排背面螺丝 F13（N01-F07，世界 x 18 / 34、z 179.92，M2 头 Ø4.0×1.6 坐在背板外面，
#   PH0 起子 Ø4.0 从 −y 进）→ 左钩不做指根外侧倒角（root_out 0.0），指外面到 x 34 那颗头 / 起子圆柱 1.12；
#   颈俯仰后仰时后指下端碰 T03 颈口后沿：x 26.5 / z 177.3 起 体素首碰 64.5°（< 65）→ 整体上移 1.0 后 体素 65.5°、网格精确 64.5°，真实姿态最大 63.9° 处净距 0.19
#   （x 往 +x 挪会撞 x 34 螺丝，所以只挪 z；数见 hr43_work/hr45n01/p15_hookL_final.json、p17_hookL_variants.json）。
# 打印（若开）：N01 仍 −y 朝下，整件被左钩抬高 6 mm、背板外面整片悬空靠支撑（用户 07:4x 曾接受：支撑全开、床面起、拆后左脸打磨）。
# ══ 开（用户 2026-09-26 08:1x 拍板"左勾先做吧，我用不上再说"，推翻主设计 08:0x 的"不做"；hr43_work/hr45n01/hr45n01_挂钩报告.md §7）══
#   下面 ①② 是 08:0x 判"线不可行"时的数，用户已接受：① 服务环搭钩照做；② 躯干顶 HB09_L 站外移到躯干轮廓允许的最外，残余算翻身动作残余。
#   ① HB09_L 头侧服务环在 head_pitch 10–40° 这类常见姿态就垂在钩外侧 / 顶上：真实姿态命中 1390 → 4921（新增 3532，其中环 3523），
#      钩下移 1.0 / 缩短到 5.2 仍 ≥4779 —— z 178–200 整条带都是环的路径，开口钩长在活动环里有勾住环、卡住头偏航的风险；
#   ② 颈俯仰 46–54° 钩尖把躯干顶那段线压到 T03 上（最深 3.78 mm，T03 在线下 2.02），要躲得把线外移到 y −25.8，伸出躯干轮廓。
#   根因：背板是颈柱 −y 侧最外面的面，钩尖到 −22.3（右钩 18.0，多伸出 4.3）。几何本身可行（F13 净距 1.12、T03 体素首碰 65.5、真实姿态 0）。
# was_until_2026_09_26_hr45n01_0841: NK_HOOK_L_STUDY = dict(xc=26.5, od=5.96, clr=0.4, t=1.2, h=6.0, lip=0.8, lip_h=1.2, under=1.1, root=0.5, root_out=0.0, embed=0.3)
# was_until_2026_09_26_hr45n01_0841: NK_HOOK_L = False
NK_HOOK_L_STUDY = dict(xc=26.5, od=5.96, clr=0.4, t=1.2, h=6.0, lip=0.8, lip_h=1.2, under=1.1, root=0.5, root_out=0.0, embed=0.3,
                       tip_ch=0.6, tip_end_ch=0.6)   # 全部 assumed；root = 槽侧指根倒角，root_out = 外侧；tip_ch / tip_end_ch = 指尖外棱 / 指尖上端棱 45° 倒角（≥0.5，主设计 08:1x）
NK_HOOK_L = NK_HOOK_L_STUDY                   # 开关：False = 不做左钩（n01_hook_L 返回空）
NK_HOOK_L_Z = (178.3, 185.5)                  # 沿线长度同 R 钩（7.2），整体比 R 钩高 1.0（T03 首碰）

def _n01_hook(H, side, y_face, z0, z1):
    """双指卡槽截面（世界 x 与"离宿主面外向距离 u"），y = y_face + side·u，沿世界 z 拉伸 z0..z1；返回两根手指。
    截面（−x 那根；+x 那根以 xc 镜像）：u ∈ [−embed, h]；指根 45° 倒角 槽侧 root、外侧 root_out（缺省 = root）；指顶唇向槽内伸 lip、高 lip_h，唇下斜面（水平 lip、竖直 under）。"""
    import trimesh
    from shapely.geometry import Polygon
    from shapely.geometry.polygon import orient
    w = H["od"] + H["clr"]; r, ro, e = H["root"], H.get("root_out", H["root"]), H["embed"]
    u1 = H["h"]; us = u1 - H["lip_h"] - H["under"]
    xo, xi = H["xc"] - w / 2 - H["t"], H["xc"] - w / 2
    tc, te = H.get("tip_ch", 0.0), H.get("tip_end_ch", 0.0)          # 指尖外棱（截面角）/ 指尖上端棱（z1 处）倒角
    prof = [(xo - ro, -e), (xi + r, -e), (xi + r, 0.0), (xi, r), (xi, us), (xi + H["lip"], u1 - H["lip_h"]),
            (xi + H["lip"], u1)] + ([(xo + tc, u1), (xo, u1 - tc)] if tc > 0 else [(xo, u1)]) + ([(xo, ro), (xo - ro, 0.0)] if ro > 0 else [])
    cut = None
    if te > 0:                                   # 指尖上端棱：三角棱柱沿 x 贯穿两指（y′ = 离指尖往里的距离，z′ = 离 z1 往下的距离，y′ + z′ ≤ te）
        y_tip, eps = y_face + side * u1, 0.05
        tri = [(y_tip - side * a, z1 - b) for (a, b) in ((-eps, -eps), (te + eps, -eps), (-eps, te + eps))]
        x0, x1 = xo - ro - 1.0, 2 * H["xc"] - xo + ro + 1.0
        cut = trimesh.creation.extrude_polygon(orient(Polygon(tri)), x1 - x0)
        cut.apply_transform(np.array([[0, 0, 1, x0], [1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 0, 1.0]]))   # (X,Y,Z) → (x=Z+x0, y=X, z=Y)，偶置换保手性
    out = []
    for sx in (1, -1):
        pts = [(H["xc"] + sx * (x - H["xc"]), y_face + side * u) for (x, u) in prof]
        m = trimesh.creation.extrude_polygon(orient(Polygon(pts)), z1 - z0)
        m.apply_translation((0.0, 0.0, z0))
        out.append(diff(m, cut) if cut is not None else m)
    return out

def n01_hook_L():
    """N01 −y 背板外面（y = 背板外表面 −16.3）上的双指卡槽（HB09_L），世界系零位；指往 −y 长。NK_HOOK_L = False 时返回 []。"""
    if not NK_HOOK_L: return []                # 关（见 NK_HOOK_L 上方注释）
    return _n01_hook(NK_HOOK_L, -1, pt(sfw("neck", 0), XR)[1] - PLT, *NK_HOOK_L_Z)

def n01_hook_R_was_until_2026_09_26_hr45n01L():
    """N01 +y 前唇上的双指卡槽（世界系、零位），返回两根手指（各是一个沿世界 z 拉伸 NK_WALL_Z 的截面实体）。
    截面在世界 x-y 平面（y = 头俯仰舵机局部 x，N01 打印时的竖直方向）：槽宽 w = od + clr，槽 x [xc−w/2, xc+w/2]；
    指 x [xc−w/2−t, xc−w/2]（+x 那根以 xc 镜像）、y [NK_LIP_Y[1], NK_LIP_Y[1]+h]；指顶唇向槽内伸 lip、高 lip_h，唇下面斜面（水平 lip、竖直 under）；
    指根两侧 root×45° 倒角；截面底边往前唇里下沉 embed。槽口（两唇之间 w − 2·lip）朝 +y 开。"""
    import trimesh
    from shapely.geometry import Polygon
    H = NK_HOOK; w = H["od"] + H["clr"]; r, e = H["root"], H["embed"]
    y0 = NK_LIP_Y[1]; y1 = y0 + H["h"]; ys = y1 - H["lip_h"] - H["under"]        # ys：唇下斜面在指内面上的起点（was_until_2026_09_26_hr45n01: − H["lip"]，45°）
    xo, xi = H["xc"] - w / 2 - H["t"], H["xc"] - w / 2                            # −x 指：外面 / 内面（槽侧）
    prof = [(xo - r, y0 - e), (xi + r, y0 - e), (xi + r, y0), (xi, y0 + r), (xi, ys), (xi + H["lip"], y1 - H["lip_h"]),
            (xi + H["lip"], y1), (xo, y1), (xo, y0 + r), (xo - r, y0)]
    out = []
    for sx in (1, -1):
        pts = [(H["xc"] + sx * (x - H["xc"]), y) for (x, y) in prof]
        m = trimesh.creation.extrude_polygon(Polygon(pts if sx > 0 else pts[::-1]), NK_WALL_Z[1] - NK_WALL_Z[0])
        m.apply_translation((0.0, 0.0, NK_WALL_Z[0])); out.append(m)
    return out

def n01_hook_R():
    """N01 +y 前唇上的双指卡槽（HB09_R），世界系零位；指往 +y 长（截面同 n01_hook_R_was_until_2026_09_26_hr45n01L，改走 _n01_hook）。"""
    return _n01_hook(NK_HOOK, 1, NK_LIP_Y[1], *NK_WALL_Z)

def n01_cbore_ears(R1, xr0):
    """hr50：背板 −x 边（世界 x 16.0）外扩的沉窝圆耳（世界系）。只给沉窝外缘越过板边的那几颗（现在只有上排 −y 侧 (18, 144.92)）：
    圆 Ø(m2_cbore_d + 2·1.0) 同心于螺丝轴、厚 = 背板厚（y xr0−PLT..xr0），只取 x ≤ 16.05（与背板重叠 0.05，不做共面贴合）。"""
    x_edge = 26.0 - 10.0                                   # 背板 −x 边 = carrier rear_y 下限 −10.0（局部 y ≡ 世界 x − 26）
    r_ear = P["m2_cbore_d"] / 2 + 1.0
    out = []
    for sy in (1, -1):
        c = pt(R1, 0.0, sy * S["mnt_dx"], S["mnt_z"][0])          # 上排螺丝轴（沉窝 Ø m2_cbore_d 就在这一排）
        if c[0] - P["m2_cbore_d"] / 2 - 1.0 >= x_edge - 1e-6: continue   # 板边外还够 1.0 的不用补
        disc = cyl(2 * r_ear, PLT, (c[0], xr0 - PLT / 2, c[2]), axis="y", sections=96)
        out.append(inter(disc, wbox((c[0] - r_ear - 1.0, xr0 - PLT, c[2] - r_ear - 1.0), (x_edge + 0.05, xr0, c[2] + r_ear + 1.0))))
    return out

def n01_wall_ledge_cut(R2):
    """hr50：头俯仰舵机包络（servo_envelope，CLR 0.3）的 x-y 足印从包络底往下延 0.45 的一片（世界系）——削掉侧墙内沿在墙底留下的 0.32 厚台阶。
    足印（舵机局部）：x（厚度向）T/2−T_lo−CLR .. T/2+CLR，y ±(W/2+CLR)；z 包络底 −0.45 .. 包络底 +0.05。两颗舵机之间这一带本来就是空的。"""
    zb = S["top"] - S["L"] - CLR
    return placed(s288.bx((S["T_lo"] + 2 * CLR, S["W"] + 2 * CLR, 0.5),
                          ((S["T"] / 2 + CLR + S["T"] / 2 - S["T_lo"] - CLR) / 2, 0.0, zb - 0.2)), R2)

@memo
def build_neck(bore=True):
    """脖子 = 两个背靠背(顶端相对)的舵机载体，背板并成一块。两法兰都朝 +y。

    【N01 不照原版重雕，已验证不划算】原版 neck.stl 是两块 2×20×11 的小侧板（world |y| 11.5..13.5, z 171.9..182.9），
    孔位（舵机局部 ±8, z=-22.5）正好就是 S288 下排背孔 —— 但：
      · 背侧那块沿 -y 平移 1.65（XL330 端面 11.5 → S288 厚段背面 12.85）后 100% 落在我们背板内部，净增材料 0；
        原位 union 只多 13 mm³ 悬空薄舌（卡在两颗舵机 0.4mm 缝里）。
      · 法兰侧那块在 S288 上一颗螺丝都拧不上（那一面只有 z=0 附近的法兰 6 孔），
        要靠打印一体连过去就得加腹板成闭口箱 → 舵机装不进去（+y 轴向 5mm 撞 180mm³，±z 穿管被上排 Ø5 垫柱堵死）。
    脖子的 y 向包络原版和我们都是 29.0，一样宽。

    【但头俯仰载体被 N02 逼着改了】N02 照原版重雕后，它的 -y 脸颊(|y| 14.5..17.5)和顶盒(|y|≤15.71)
    绕头俯仰轴 ±45° 扫掠，把 z>193 那一带全占了。我们原来伸到 z=215.2 的背板必须缩到 192.5，
    **头俯仰舵机上排那 2 个角孔(z=209.92)因此报废，只剩下排 2 颗**。
    这不是相对原版的倒退（原版也只在 z 171.9..182.9 抓这颗舵机），但相对我们上一版是少了 2 颗螺丝，
    两侧墙和前唇留有装配间隙，是过载止挡；正常反力由背板和下排两颗螺丝承担，不能当作无间隙夹持。

    【2026-09-12 惰轮通孔】原版颈俯仰是两端支撑：left_shell 当舵盘、right_shell 内凸台抓 XL330 后端惰轮。
    S288 背面同样是自由转动的惰轮，但我们的 N01 背板（-16.15..-13.15）正挡在 T03 与惰轮之间，
    所以背板上要开一个 Ø NK_IDLER_BORE 的同心通孔让 T03 的毂穿过去。载体须避开旋转毂：
    此处惰轮已与从动件刚接，再与载体刚接才会闭环锁死；材料接近本身不证明锁死。
    bore=False 只给 trunk.neck_swing_env() 用：扫掠体与开孔前逐字一致、少算一次开孔件；开孔与否对 T03 结果无影响
    （复核实测差 <1e-4 mm³：minkowski_box 的 ±0.8 方盒膨胀把孔在扫掠体里缩到 r≈6.5，孔口那圈本来就被毂盖住）。
    @memo 按位置参数做 key，所以调用一定要写 build_neck(False) 而不是 build_neck(bore=False)。"""
    R1, R2 = sfw("neck", 0), sfw("neck", 1)           # (26,0,152.4) 顶端朝下；(26,0,202.4) 顶端朝上
    c1 = placed(carrier(open_side="+z", ring=False, walls=(), front=False, rear_y=(-10.0, 12.8),
                        rear_z=(-(S["L"] - S["top"]) - CLR - Wl, 10.2)), R1)
    z2 = NK_WEB_Z1 - pt(R2, 0)[2]                     # 背板顶换算到舵机局部 z（R2 顶端朝 +z → 局部 z = 世界 z − 202.42）
    c2 = placed(carrier(open_side="+z", ring=False, walls=(), front=False, rear_y=(-12.8, 10.0),
                        rear_z=(-(S["L"] - S["top"]) - 0.6, z2)), R2)
    xr0 = pt(R1, XR)[1]                                # 背板 y（世界）-13.3（旧 -13.15）
    web = wbox((16.0, xr0 - PLT, 142.2), (38.8, xr0, NK_WEB_Z1))
    walls = [wbox((x0, xr0 - PLT, NK_WALL_Z[0]), (x1, NK_LIP_Y[1], NK_WALL_Z[1])) for (x0, x1) in NK_WALL_X]
    # 前唇是留有 CLR 间隙的过载止挡，不计入正常工况的承载力偶。
    lip = wbox((NK_WALL_X[0][0], NK_LIP_Y[0], NK_WALL_Z[0]), (NK_WALL_X[1][1], NK_LIP_Y[1], NK_LIP_Z1))
    # was_until_2026_09_26_hr45n01: m = union(c1, c2, web, lip, *walls)
    # was_until_2026_09_26_hr45n01L: m = union(c1, c2, web, lip, *walls, *n01_hook_R())
    # was_until_2026_09_26_hr46: m = union(c1, c2, web, lip, *walls, *n01_hook_R(), *n01_hook_L())     # hr45n01：+y 前唇 / −y 背板外面两个双指卡槽先并入，再过下面的舵机包络 / 通道 / 扫掠刀
    hr = n01_hook_R(); hl = n01_hook_L()                                   # hr46：两组手指先算出来（几何同上一行，只是拿出来复用）
    # was_until_2026_09_28_hr50: m = union(c1, c2, web, lip, *walls, *hr, *hl)                         # hr45n01：+y 前唇 / −y 背板外面两个双指卡槽先并入，再过下面的舵机包络 / 通道 / 扫掠刀
    # hr50（2026-09-28）：颈俯仰舵机上排 −y 侧那颗螺丝（N01-F06，世界 (18, *, 144.92)）的 Ø m2_cbore_d 沉窝外缘 x 15.8 越过背板 −x 边 16.0 → 沉窝环被板边切开
    #   （clean_check holes 开口环 55°、板边两片 1.3 mm² 刀片）。板边在沉窝一带局部外扩成 Ø(沉窝 + 2×1.0) 的圆耳（只取 x ≤ 16.05 那块，与背板体积重叠 0.05）：
    #   沉窝壁 ≥1.0 闭合。圆耳离颈俯仰轴 ≤14.17 < 背板原角点 14.3 → 颈俯仰扫掠不比原角点更远（T01 缺口 / T02 / T03 颈部让位照旧包得住，build 后复验）。
    m = union(c1, c2, web, lip, *walls, *hr, *hl, *n01_cbore_ears(R1, xr0))  # hr45n01：+y 前唇 / −y 背板外面两个双指卡槽先并入，再过下面的舵机包络 / 通道 / 扫掠刀
    # hr46（2026-09-26）：并入前的手指实体单独登记成附属实体 → cad/duck_s288/hooks/N01_<锚点 id>.stl（L6 "线撞自己固定件" own 桶用；
    #   后面的舵机包络 / 通道 / 扫掠刀理应不碰钩）。标签 = wire_fixings_v1.json anchors 的 id；build_neck 被别的件的扫掠反复调用 → 同几何幂等。
    produce_extra("neck", "FX_HB09R_HOOK_N01", union(*hr))
    if hl: produce_extra("neck", "FX_HB09L_HOOK_N01", union(*hl))
    cuts = [servo_env(R1), servo_env(R2), mount_cut(R1), mnt_cut2(R2, pairs=(1,))]
    # hr50（2026-09-28）：两道侧墙（NK_WALL_X 内缘压进舵机包络 0.5）在墙底 NK_WALL_Z[0]=177.3 到头俯仰舵机包络底 177.62 之间留下 0.32 厚 × 0.8 宽的
    #   内沿台阶（clean_check 薄膜 37.7 mm² ×2，z≈177.46）。加厚要伸进头俯仰舵机（底 177.92）、下移墙底会挡颈俯仰舵机 +y 拆卸（顶 176.9）→ 按规矩整片削穿：
    #   头俯仰舵机包络的 x-y 足印往下延到墙底以下 0.13，墙内面从墙底起就是一个平面（x 15.7 / 36.3）。
    cuts.append(n01_wall_ledge_cut(R2))
    # PH2.0 插座（keepouts.yaml:KO01，09-20 背插模型；09-21 线翘 4 mm → lib.CONN_MODE "window"）：两颗舵机的插座都朝 −y、被本件背板盖住，
    # 3 mm 背板兜不住 4.5 的线翘区 → 背板开通窗、线从背板外面（y<−16.3）走，插头装好舵机后再插。背板外面：
    #   · 颈俯仰颗（R1）：T03 壳内壁在 −17.1（neck_swing_env 削的），只差 0.4 → T03 减掉线翘区绕颈俯仰的扫掠（trunk.build_shell）。
    #     只开 **+y 口**（R1 局部 +y = 世界 +x，口在 x≈33.8）；−y 口（世界 x≈18.3）T03 在颈 +35° 起 7.1 mm³、T01 在 +60° 0.9 mm³ 扫过 → 不用，
    #     颈俯仰当分支根（harness.yaml:HB01，一分二 ×5）。
    #   · 头俯仰颗（R2）：N02 −y 脸颊内面 −14.5 正盖着插座上半截 → 脸颊下半截按线翘区扫掠挖穿（build_neck_pitch wire_clr；hr23 的外移 3.05 撞 H03 作废），两口都开。
    cuts += [conn_cut(R1, sides=(1,), mode="window"), conn_cut(R2, mode="window")]
    # hr50（2026-09-28）试过：+y 窗扣 Ø18 同轴柱、窗与 Ø16 惰轮孔之间留 1.0 实环（孔环闭合）→ 真 PHR-3 本体（4.5×7.8，不加隙）从背板外沿 +x 插进去的
    #   路径被环挡住（穿入 4.5 mm³，插头内角 (y 5.5, z −5.4) 离轴 r 7.7，本体截面跨过 r 8..9；hr50_work/trunk_neck/plug_check.json）→ 撤回。
    #   窗与孔连通是插头插入路径的设计开口（主设计登记 clean_check 白名单）；窗外到板边那条保持闭合（撤区域刀后 2.3，见下方 hr24 刀的 was 注释）。
    # hr24：+y 窗（扣 Ø15.6 毂柱）与 Ø16 惰轮通孔在窗足印里只隔一圈 r 7.8..8.0 的 0.2 薄环 → 一起挖掉，窗与孔干净合并（L2 N01-F10 min_dirs 22）
    ring = diff(cyl(NK_IDLER_BORE + 0.6, 6.0, (-15.8, 0, 0), axis="x"), cyl(NK_IDLER_BORE - 1.0, 8.0, (-15.8, 0, 0), axis="x"))   # r 7.5..8.3 环，x −18.8..−12.8
    win = s288.bx((6.0, S["W"] / 2 + S["plug_clr"] - (S["W"] / 2 - S["plug_y_in"]), S["conn_z"][1] - S["conn_z"][0] + 2 * S["plug_clr"]),
                  (-15.8, (S["W"] / 2 + S["plug_clr"] + S["W"] / 2 - S["plug_y_in"]) / 2, (S["conn_z"][0] + S["conn_z"][1]) / 2))   # +y 窗足印（不扣毂）
    cuts.append(placed(inter(ring, win), R1))
    # hr24：+y 窗外沿（局部 |y| 10.5 = 世界 x 36.5）到板前缘之间只剩 0.2–0.8 的竖条（web 前缘在 z 157..167 被 09-17 jaw 区域刀削到 x≈37，见下面 region_cut）
    # → 窗直接开到板边（局部 y 10.5..13.8），成一个边缘缺口；那一段本来就没有侧墙（cradle 插座区不封）
    # was_until_2026_09_28_hr50: cuts.append(placed(s288.bx((6.0, 3.3, S["conn_z"][1] - S["conn_z"][0] + 2 * S["plug_clr"]),
    # was_until_2026_09_28_hr50:                            (-15.8, S["W"] / 2 + S["plug_clr"] + 1.65 - 0.05, (S["conn_z"][0] + S["conn_z"][1]) / 2)), R1))
    # hr50（2026-09-28 用户定：区域刀全撤、极限帧交给重训约束）：jaw 区域刀撤掉后 web 前缘回到 x 38.8，窗外沿 36.5 到板边留 2.3（≥0.8）
    # → hr24 "窗开到板边" 这刀撤回，Ø16 惰轮孔 + 窗合成的开口四周闭合（店家截图问的"开口环"，clean_check holes N01-F10 断口 21.5°）。
    if bore: cuts.append(idler_bore(R1, NK_IDLER_BORE))
    m = diff(m, *cuts)
    # 09-17 真实姿态区域刀（lib.region_cut）：头俯仰 ~96° 缩头时原版 jaw/H03 压到 web 前缘（7 姿态，134 mm³，x 36.2..38.8 前 2.6 mm 条）；
    # 坐起 head_all_min 时 web 下角碰右髋偏航舵机（1 姿态，1.7 mm³）。只切 N01（jaw 是原版件、舵机不能切）。
    # was_until_2026_09_28_hr50: m = diff(m, region_cut("neck__pair_jaw_soft_x_neck.stl", "neck"), region_cut("neck__pair_neck_x_trunk_base.stl", "neck"))
    # hr50（2026-09-28 用户定"非常极限的姿势靠训练去限制，就不削了"）：两把区域刀都撤。N01#4（jaw/H03 × web 前缘）9 姿态、N01#10（web 下角 × 右髋偏航舵机）
    # 4 姿态全部来自 alpha_stand/body_all_max 与 roulade 极限帧（hr50_work/poses/region_recheck.md），列进重训约束清单；区域刀削出的马赛克切面、
    # 把 F06 上排沉窝 (34,−15.8,144.9) 外侧咬开的缺口、把窗外竖条削到 0.2–0.8 的问题随之消失。
    return keep_main(diff(m, sweep_of(build_neck_pitch(), "neck_pitch", -45, 45, 19, grow=0.4)), "neck")

def neck_screw_heads():
    """N01 背板外面 6 颗角螺丝的头（Ø4×1.6，世界坐标）：颈俯仰舵机 4 颗（上排沉窝到 -15.15、下排坐在板外面 -16.15）
    + 头俯仰舵机下排 2 颗。给 trunk.neck_swing_env 当扫掠体的一部分 —— 复核发现上排那颗头离 T03 内壁只有 0.20，
    T03 的 y 基准改成"毂贴惰轮端面"之后，毂打印短一个层高壳就整体内移、头在颈俯仰转动中蹭壳。"""
    R1, R2 = sfw("neck", 0), sfw("neck", 1)
    hs = []
    for sy in (1, -1):
        hs.append(placed(cyl(P["m2_cbore_d"], P["m2_head_h"], (XR - PLT + 1.0 - P["m2_head_h"] / 2, sy * S["mnt_dx"], S["mnt_z"][0]), axis="x"), R1))   # 上排：沉窝底 -15.3（09-13 沉窝 Ø m2_cbore_d；旧 -15.15）
        hs.append(placed(cyl(P["m2_cbore_d"], P["m2_head_h"], (XR - PLT - P["m2_head_h"] / 2, sy * S["mnt_dx"], S["mnt_z"][1]), axis="x"), R1))         # 下排：板外面 -16.3（旧 -16.15）
        hs.append(placed(cyl(P["m2_cbore_d"], P["m2_head_h"], (XR - PLT - P["m2_head_h"] / 2, sy * S["mnt_dx"], S["mnt_z"][1]), axis="x"), R2))
    return hs

# ── N02 颈俯仰（照原版 neck_pitch.stl 重雕）
NP_AX = (26.0, 202.4215)               # 头俯仰轴 (x,z)
NP_CHK_Y, NP_CHK_OUT = 16.0, 17.5      # 脸颊中面 / 外表面 |y|（板 14.5..17.5，实测 3.0 厚）
NP_XLH = [(20.0, 202.4215), (32.0, 202.4215), (26.0, 196.4215), (26.0, 208.4215)]   # 两侧脸颊上的 XL330 4 孔（r=6）
NP_YAWH = [(20.0, 0.0), (32.0, 0.0), (26.0, 6.0), (26.0, -6.0)]                     # 顶板上的 XL330 偏航 4 孔（r=6）
NP_RIS_D, NP_RIS_ID, NP_RIS_Z0 = 19.85, 15.0, 218.2   # 19.85 mm journal / original 15 mm tool tunnel; fit remains unqualified
# 09-12 第二批（I3）：与 keepouts.yaml 对表的通道刀要比名义大 0.05。lib.cyl 是 64 边**内接**（平边 r=7.5·cos(π/64)=7.4910），
# Gate l3_static._cyl 是 128 边**外接**（顶点 7.5023，故意宁可多报）；名义 Ø15 刀切出来的壁在平边处永远比禁入体小 0.011，
# 整圈留 0.32 mm²/mm 的壳 → KO07 恒红 2.7 mm³。Ø15.05 平边 7.5159 ≥ 7.5023（裕量 0.014 ≈ 1/15 层高，打印上等于没变）。
NP_KO_OVER = 0.05
# 内腔是那 6 颗螺丝唯一的螺丝刀通道：螺丝头 Ø4 @ r=horn_r 5.25 → 头外缘 r7.25，离 Ø15.05 腔壁 0.275（09-13 分度圆定案后；
# 旧 4.75 时 0.775），筒壁仍有 2.5。外径不能再大：N03 的横滚杯扫掠占了 r≥12（现在留 2.0）—— 没有再放大的余地，mechanical_audit N02_heads 盯着。
NP_DISC_T, NP_CB = 5.25, 1.1   # F16: M2x8 stack5.25, engagement2.75; cheek recess unchanged
# 两侧脸颊上 6 颗螺丝头的共同沉坑直径：6×Ø m2_cbore_d 头窝 @ r=horn_r 的外包络 + 0.1。**从 horn_r 推，不要写死**
# （5.25 + Ø4.4 → Ø15.0，脸颊 x 17..35 / z≥193.4 盖得住；旧 4.75 + Ø4.2 → Ø13.8）。写成函数而不是模块常量：
# 模块常量在 import 时就定死了，之后再改 S["horn_r"]（探针猴补丁）它不会跟着变 —— 09-12 就这么踩过一次。
def NP_CB_D(): return 2 * S["horn_r"] + P["m2_cbore_d"] + 0.1     # 09-13：头窝 4.2 → m2_cbore_d
# 惰轮贴合毂外径 Ø15.0（09-13 分度圆定案 5.25 后由 14.0 放大，docs/reports/分度圆Ø9.5还是Ø10.5_2026-09-12.md §定案后）：
# 6 个 Ø2.6 过孔（S["idler_hole_d"]）外缘 r=5.25+1.3=6.55，到毂边 0.95（idler_hub 断言 ≥0.8；Ø14 时只剩 0.45 会响）。
# 09-11 版写"不能做到 Ø13.8（惰轮外径）以上，会压到厚段背平面"—— 09-12 复核实测这是过度保守；09-13 手册 step_z=(-7,-13.5)：
# 厚段背平面 -13.0 只在 z≤-13.5，斜坡在 z=-7.5（毂边）处 x≈-10.2，Ø15 毂碰不到本体；毂只压在 Ø14 的惰轮盘上。
# Ø15 比惰轮盘大 0.5 半径，那 0.5 环对着的是薄段背面（-10.0，3.0 mm 远）。同值 trunk.SHR_IDL[2]。
NP_IDL_HUB_D = 15.0
NP_DOME_Y = (-12.6, 10.65)   # hr50：穹顶正圆柱刀的 y 范围（舵机厚段 −10.3..+10.3 加 grow 0.3；−y 端延到线翘扫掠 −12.5 以外 0.1）

@memo
def build_neck_pitch():
    """颈俯仰件（照原版 neck_pitch.stl 重雕）。原版是跨过头俯仰舵机的 U 形轭：
      · 两侧脸颊 |y| 14.5..17.5（3.0 厚，x 17..35，z 193.42..~216），**两侧都有 XL330 4×Ø2.2@r6 + 中心 Ø6**
        （+y 抓舵盘、-y 抓 XL330 背面惰轮）
      · 顶盒 z 212.87..218.21（|y|≤15.71），下表面是跟着舵机 ±45° 扫掠走的穹顶
      · 顶板 z 218.21..221.115（x 18..34, |y|≤8..9），另一组 XL330 4 孔 + Ø4.4×2.2 沉孔（自下而上拧进偏航舵盘）
    S288 只有三处差异：
      ① 从动面 12.85 → 原版 14.5，补 1.65（driven_patch，Ø18 全程内切原版轮廓）
      ② 惰轮面 -12.85 → 原版 -y 脸颊内表面 -14.5，同样补 1.65（idler_hub）。
         **-y 脸颊照原版抓惰轮** —— 2026-09-11 用户拿实物确认 S288 背面那个盘是自由转动的副轴，
         手册尺寸图 p2 两端标注一模一样（Ø14×3 凸台 + 6-Ø1.7⌄3.0 M2 自攻 + 中心十字螺钉）。
         旧注释写「S288 背面没有惰轮 → -y 脸颊降级为纯罩壳」是**错的**，那一版把头俯仰做成了单端悬臂。
      ③ 偏航舵机沿轴上移 5 → 法兰面从 221.115 抬到 227.765，补一根 Ø20 立筒（Ø15 内腔兼作那 6 颗螺丝唯一的螺丝刀通道）
    实测：原版毛坯与 S288 舵机本体/包络、N03、H01、原版上下头壳、躯干壳的干涉全部是 0 —— 唯一冲突是我们自己的 N01（见 build_neck）。"""
    Rh = drv_from("neck_pitch")                        # 脖子里的头俯仰舵机，法兰朝 +y，轴心 (26,0,202.42)
    Ry = drv_self("yaw_roll_motion")                   # yrm 里的偏航舵机，法兰朝 -z
    O = orig("neck_pitch", "neck_pitch")
    ZF = float(pt(Ry, S["T"] / 2 + S["flange_h"])[2])  # 227.615 = S288 偏航法兰面（旧 227.765）
    fills = []
    for sy in (1, -1):                                 # 两侧脸颊的 XL330 接口孔（4×Ø2.2@r6 + 中心 Ø6），各留 0.3 径向搭接
        fills += [fill_cyl((x, sy * NP_CHK_Y, z), 2.8, "y", 3.0) for (x, z) in NP_XLH]
        fills += [fill_cyl((NP_AX[0], sy * NP_CHK_Y, NP_AX[1]), 6.6, "y", 3.0)]
    fills += [fill_cyl((x, y, 217.6), 5.2, "z", 9.0) for (x, y) in NP_YAWH]     # Ø5.2 一次盖住通孔 + Ø4.4 沉孔
    fills += [fill_cyl((NP_AX[0], 0.0, 217.6), 6.6, "z", 9.0)]
    riser = cyl(NP_RIS_D, ZF - NP_RIS_Z0, (NP_AX[0], 0.0, (NP_RIS_Z0 + ZF) / 2))
    # 09-21 线翘 4 mm（lib.conn_zone "A" 到 y=−17.4）：−y 脸颊内面 −14.5 正盖着头俯仰舵机插座的上半截（z>193.42），3.0 厚的脸颊整个在线翘区里。
    # hr23 试过把脸颊向外加厚 3.05 再在内层扫槽 —— H03 下头壳本来就贴着脸颊外表面 −17.5（hr22 真实姿态 1 个姿态擦 0.05 mm³），外移 3 mm 后
    # 走路姿态 106 个碰、头横滚 −25° 撞 22 mm³ → 作废。hr24：脸颊**不加厚**，下面 wire_clr 直接把线翘区绕头俯仰全程的扫掠从 3.0 厚的脸颊上
    # 整个挖穿（下半截随扫掠带掉，只留毂 Ø15 本身 + 朝顶盒那 ~76° 的扇形连接板，受力路径 毂→扇形板→顶盒；头俯仰悬垂弯矩 25.7 N·mm 对
    # 9 mm 宽 × 3 厚 PETG 板应力 ≈2 MPa）。线翘从开口处直接出来，向 −x 绕到脖子后面进口袋走廊（harness HB01 链 F）。
    m = union(O, *fills, driven_patch(Rh, 18.0), idler_hub(Rh, -NP_CHK_OUT + 3.0, NP_IDL_HUB_D), riser)
    bore = cyl(NP_RIS_ID + NP_KO_OVER, (ZF - NP_DISC_T) - 214.7, (NP_AX[0], 0.0, (214.7 + ZF - NP_DISC_T) / 2))   # z 顶 = 顶盘底 ZF−3.5=224.115（螺丝坐面；跟着法兰面走，旧 224.2646）；hr11：底从 215.0 下延到 214.7——原版 214.8..215.0 有一圈 Ø14.37 台阶，F16 #0/#3 起子外缘 r7.2 擦 0.012 mm³
    # 6×Ø m2_cbore_d @ r=horn_r 的外包络（5.25 → Ø14.9）；Ø12 单坑会挡螺丝头。共同沉坑 NP_CB_D 消除 0.15mm 腹板。两侧脸颊同样处理：
    # 两侧叠厚 = 17.5 − 1.1 − 13.0 = 3.4（旧 3.55）：不沉这 1.1 的话叠厚 4.5 → M2×6 只咬 1.5（太浅），M2×8 则进 3.5 超过 3.0 MAX 顶底。
    recess = union(*[cyl(NP_CB_D(), NP_CB + 0.02, (NP_AX[0], sy * (NP_CHK_OUT - NP_CB / 2 + 0.01), NP_AX[1]), axis="y")
                     for sy in (1, -1)])
    # 让位刀要放过惰轮贴合区：servo_envelope 的背面按最保守的台阶画、还把副轴放大到 Ø15.8，
    # 扫掠后伸到 x=-14.15，不修剪就把 1.65 的贴合毂削掉大半（只剩 0.35，等于没贴上）。
    # was_until_2026_09_28_hr50: sw = diff(sweep_of(servo_env(Rh), "neck_pitch", -45, 45, 61, grow=0.3), idler_keep(Rh, NP_IDL_HUB_D + 0.6))
    # hr50（2026-09-28）：DILATE6 的 6 个轴向副本在穹顶上留 90° 台阶，与 Ø15.05 立筒孔相交处成碎边（clean_check ragged ×2 @ (26, ±6.2, 216.2)）
    #   → 按规矩改 mink 连续膨胀后再转（1.5°/步不变）。
    sw = diff(sweep_of(servo_env(Rh), "neck_pitch", -45, 45, 61, grow=0.3, mink=True), idler_keep(Rh, NP_IDL_HUB_D + 0.6))
    # hr50：穹顶 = 舵机本体顶角（局部 (±(W/2+CLR+0.3), top+CLR+0.3)，离头俯仰轴 r_c = 14.64）绕轴 ±45° 扫出的圆柱面；61 份离散旋转拼出来是多棱面，
    #   与 Ø15.05 立筒孔相交成碎边、在顶盒两端面（x 17.0 / 35.0）下沿留 0.07 的唇（clean_check 薄膜 28.9 / 28.2 mm²）→ 补一把光滑正圆柱刀
    #   （r = r_c + 0.01，256 边，只取轴线以上半圆、舵机厚度向 y −12.6..+10.65）把棱面削成真圆柱面：只多削棱面与圆之间 ≤0.001 的弓形；
    #   −y 端延到 −12.6（线翘扫掠 wire_clr 从 −12.5 起）把两刀之间那条 1.9 宽、下沿楔形的鳍一起切平（clean_check 刀片 3.2 mm² + 尖刺）。
    #   端面处圆柱面与竖直端面成 52° 楔，不再是薄唇。
    r_dome = math.hypot(S["W"] / 2 + CLR + 0.3, S["top"] + CLR + 0.3) + 0.01
    dome = inter(placed(cyl(2 * r_dome, NP_DOME_Y[1] - NP_DOME_Y[0], (0.0, 0.0, 0.0), axis="y", sections=256),
                        np.array([[1, 0, 0, NP_AX[0]], [0, 1, 0, (NP_DOME_Y[0] + NP_DOME_Y[1]) / 2], [0, 0, 1, NP_AX[1]], [0, 0, 0, 1.0]])),
                 wbox((NP_AX[0] - r_dome - 1, NP_DOME_Y[0] - 0.1, NP_AX[1]), (NP_AX[0] + r_dome + 1, NP_DOME_Y[1] + 0.1, NP_AX[1] + r_dome + 1)))
    lip_trim = [dome]
    # 09-20 背插模型：头俯仰舵机两个 PH2.0 插头顶面与厚段背面 y=−13 齐平，线再凸 wire_out=2（到 y −15）；−y 脸颊内表面在 −14.5，
    # 插座 z 区（世界 188..198.2）上半截（>193.42）正对脸颊 → 线只有 1.5。把插头顶/线弯区（lib.conn_zone "A"，x −15..−12.8）绕头俯仰轴
    # ±45° 扫掠（grow 0.3）后从脸颊减掉 = 内表面上一条 0.5+0.3=0.8 深的环形让位槽（脸颊 3.0 → 2.2，只在 r 7.8..10 盘下半圈外沿，hr18 复审实测），惰轮贴合毂 idler_keep 照旧放过（毂在 r≤7.8）。
    # 09-21：线翘区到 −17.4、头俯仰目标区间 [−54.2, 90]（frozen.yaml）→ 在 N02（子件）系里反向扫 [−92, 56.2]（2°/步，各留 2°），grow 0.3。
    # 放过的毂 = Ø NP_IDL_HUB_D 本身（不是 +0.6）：conn_zone 扣 Ø15.6、膨胀 0.3 后扫掠体内孔 ≤ Ø15.0，与毂外圆重合 → 毂完整、毂外一圈不留 0.3 薄环
    # （留了就是 L1 薄壁）。毂壁：Ø2.6 过孔外缘 6.55 → 毂边 7.5，0.95。
    # hr25：膨胀必须 mink=True（连续长方体）。hr24 用 DILATE6：沿 y（= 扫掠轴 = 毂轴）的两个副本不缩孔，A 区端面 −17.4 外到脸颊外表面 −17.5 那 0.1
    # 只有 Ø15.6 的孔 → 毂外留一圈 0.1 厚 × 0.3 宽的唇，斜向另有 ≤0.09 的薄片；L2 N02-F12 直径量到 Ø15.58（声明 Ø15.0）、L1 薄区 73.9 → 80.1 mm²。
    # 预检（scratch n02f12/try_mink.py）：mink 后毂外 r 7.45..7.55 干净，多削 5.3 mm³（全是 ≤0.12 的皮 + 唇），面数 19418 → 12288。
    # hr26：放过的毂柱 Ø15.0 → **Ø14.96**。mink 后扣柱孔在斜向缩到 r 7.38..7.5，扫过的扇区里刀的内壁 = idler_keep 那根柱的 64 边形，
    # 与 idler_hub 的 Ø15.0 64 边形**整面重合** → hr25 导出后 float32 塌成 2 个退化面 + 2 条非流形边（L0 N02 两条 WARN 红，−28° 处）。
    # 柱小 0.04 = 刀比毂面深 0.02：扇区里毂 r 7.48、扇形板那边 7.5，台阶 0.02（毂外圆不是配合面，N02-F13 Ø15.0 ±0.1 内）。
    wire_clr = diff(conn_hump_sweep(Rh, (1, -1), "neck_pitch", -92.0, 56.2, 75, grow=0.3, mink=True), idler_keep(Rh, NP_IDL_HUB_D - 0.04))
    # was_until_2026_09_28_hr50: return keep_main(HBR.head_yaw_base(diff(m, bore, horn_cut(Rh), recess, flange_relief(Rh), horn_cut(Ry),
    # was_until_2026_09_28_hr50:                       idler_cut(Rh), sw, wire_clr)), "neck_pitch")
    return keep_main(HBR.head_yaw_base(diff(m, bore, horn_cut(Rh), recess, flange_relief(Rh), horn_cut(Ry),
                          idler_cut(Rh), sw, wire_clr, *lip_trim)), "neck_pitch")

# ── N03 偏航-横滚（照原版 yaw_roll_motion.stl 重雕）
YRM_AXZ = HBR.HEAD_ROLL_Z                       # head_roll 轴（y=0）
YRM_A0 = 6.4                           # A 座盘 x 起点（原版 Ø16 轴颈坐进底壳 Ø22.18 半座 x 6.2..10.0）
YRM_A_STACK = 5.25                     # F17: 6xM2x8; N03/bearing-inner/N05 clamp; engagement 2.75
YRM_A2 = float(pt(drv_self("jaw_soft"), s288.x_flange_face())[0]) + YRM_A_STACK   # F17 under-head plane = live horn face +5.25 (nominal13.55)
YRM_BORE = (YRM_A2, 14.8)              # A 盘外侧 Ø16 让刀孔（同时把原版 XL330 舵盘 4 孔连根铲掉），从坐面起
# ── B 端（2026-09-12 ④：PLA 滑配盘换回原版的 22×16×4 深沟球轴承）──────────────────────────────
# 原版 N03 B 端是**一体**的 Ø16 轴颈（x 40.2..44.0）+ Ø18 台肩 + Ø12 通孔，轴承坐 x 40..44（MJCF 登记两颗 22×16×4）。
# 我们复刻不了"一体"：S288 A 端 6 颗法兰螺丝的起子只能从 +x 穿 N03 内腔进去（内腔 x 36.3−11.2 = 25.1 长，
# 从 −z 开口塞不进任何整支起子；实测把 B 端做实心后 6 条起子刀路各撞 N03 自己 109~111 mm³），而通道
# Ø ≥ 2×(horn_r+2.1) = 14.7（09-13 定案 5.25；旧 4.75 时 13.7）和 Ø16 轴颈只剩 0.65 壁不可兼得。
# 所以轴颈做成**独立的销 N04**（build_head_journal）：拧完 6 颗后从头偏航舵机腔里沿 +x 插进 B 腹板的 Ø16.2 孔，
# 起子通道顺势放大到 Ø16.2（起子外缘 r 7.35 < 8.1）。N03 这边只留：
#   · Ø16.2 通孔（销的导向孔；销 Ø16.0 → 径向 0.1）
#   · 腹板内面 Ø18.2×1.0 沉窝：坐 N04 的上半圈 Ø18 凸缘 → 销的 +x 止挡；−x 止挡是头偏航舵机本体（x≤36.0，隙 0.3）
#   · Ø21.2 凸台：把原版 r9 薄管周围补厚到 r10.6（≤ H01 座 r11.09−0.49）；+x 面离轴承 −x 面 0.6，绝不擦外圈
#   · Ø17.6 台肩：只顶轴承**内圈**（22×16×4 内圈外径≈18 / 外圈内径≈20，实物到手核对）
# 轴承 x 40.2..44.2 = H03 底壳 B 下半座的槽（原版实测 40.1..44.2，唇 44.2..44.9 r9.3）；H01 上半座刀 x 34..54 无轴向约束。
YRM_B_WEB = float(pt(sfw("yaw_roll_motion", 0), 0.0, -(S["W"] / 2 + CLR))[0])   # 36.3：B 腹板内面 = 头偏航舵机腔 +x 面（舵机 x 16..36 + CLR）
YRM_B_BORE = 16.2                          # 腹板通孔 Ø（= N04_D_GUIDE + 0.2）
YRM_B_CB = (18.2, 1.0)                     # 沉窝 Ø × 深（= N04 凸缘 Ø18 + 0.2，深 = 凸缘厚）
# hr13：N06 压盘出腔路径 = 抬 +z N06_EXIT_LIFT 再沿 −y 出腔（assembly.yaml 步 11 d）；扫掠体各向 +N06_EXIT_CLR 从 N03 减掉（build_yrm）。
# 抬多高都绕不开 B 腹板：N06 Ø21.6 的 +x 边 36.8 > 腹板内面 36.3，而腹板在 z 227..246 整段、|y| ≳ 8.7 处全有料（bore/沉窝口以外），
# 所以出腔必然在腹板内面上开一条 0.8 深的槽（hr13 实测：5/8/10/12 mm 抬升出腔都撞 3.9–6.1 mm³）。取 5 与 n06_travel（腔内直上 227..232.5）一致。
# 槽带 z = [HEAD_YAW.z1 + lift − clr, zf + lift + clr]；mechanical_audit 的沉窝径检查按 n06_exit_band() 放过槽带内的射线、另在槽底以外一层复查整圈。
N06_EXIT_LIFT, N06_EXIT_CLR, N06_EXIT_XSKIN = 5.0, 0.3, 0.1
def n06_exit_band():
    zf = float(pt(sfw("yaw_roll_motion", 0), S["T"] / 2 + S["flange_h"])[2])
    return (HBR.HEAD_YAW["z1"] + N06_EXIT_LIFT - N06_EXIT_CLR, zf + N06_EXIT_LIFT + N06_EXIT_CLR)
YRM_B_BOSS = (YRM_B_WEB, 39.0, 21.2)   # shorter boss clears B outer-ring rear stop
YRM_B_SHLD = (39.0, 40.22, 17.6)   # jclr leaves x40.20 inner-ring shoulder, clamped through keyed N04/N07
YRM_B_BRG = (40.2, 44.2)                   # 22×16×4 轴承 x 区间（assembly_audit.bearing_specs / head.py / N04 都从这里推）
N04_D_GUIDE, N04_D_JRN, N04_FLANGE = 16.0, 15.9, (18.0, 1.0)   # N04 导向段 Ø / 轴承段 Ø（内圈 Ø16 留 0.05 径向，同 6700 的 9.95）/ 半凸缘 Ø×厚
YRM_SKIRT, YRM_SKIRT_Z0, YRM_SKIRT_Y0 = 2.5, 246.0, -23.0   # 裙墙厚 / 起始 z / -y 端
YRM_H01 = (12.4, 10.9)                 # x<12.4 处只准留 r≤10.9（H01 的 seatA/upA/bridge 都在 x≤12）
# hr50（2026-09-28）：h01_clr 在头偏航 6704 座 −x 壁处收回（x0, |y| 上限, z0, z1）：头横滚 ±25° 全程 H03 在 |y|≤4.5 只到 x 11.0、4.5..7 到 11.35（留 0.4）
YRM_SEAT_KEEP = ((11.4, 4.5, 219.95, 225.3), (11.75, 7.0, 219.95, 225.3))
YRM_ROOT_TRIM_X = 14.75                # hr50：A 侧根 x ≥ 此值的残层整片切掉（Ø16 让刀孔端面 14.8 外 0.05）
YRM_N02 = (25.0, 224.0, 228.2)         # N02 头偏航从动盘（同轴 Ø24 @ z224.77..227.77）让位

def build_yrm():
    """N03 head-yaw outer-ring carrier and head-roll inner-ring bridge.
    Original A/B head-roll bearings are retained as a functional reference;
    S288 uses A6704/N05 and B22x16x4/keyed N04/N07 with inner-ring clamps.
    The yaw6704 housing and removable N08 retain its outer ring; N02/N06
    clamp its inner ring. All housing roots must survive the final cutters.
    This nominal rebuild requires assembly, clearance and load validation;
    zero interference alone does not certify bearing retention or strength.
    """
    Ry = sfw("yaw_roll_motion", 0)                     # (26,0,240.6) 法兰朝下（SERVO_SHIFT 已上移 5）
    Rr = drv_self("jaw_soft")                          # 头里的横滚舵机（SERVO_SHIFT −1.7；09-14 前 −1.5 贴原版位置）法兰朝 +x，法兰面 x=8.3（09-13 8.1，旧 7.95）
    O = orig("yaw_roll_motion", "yaw_roll_motion")
    AX = (0.0, YRM_AXZ)
    xf = float(pt(Rr, S["T"] / 2 + S["flange_h"])[0])
    ringA = diff(cyl(P["seat_disc_d"], xf - YRM_A0, ((YRM_A0 + xf) / 2, *AX), axis="x"),
                 cyl(S["flange_d"] + 2 * CLR, xf - YRM_A0 + 0.02, ((YRM_A0 + xf) / 2, *AX), axis="x"))   # 环：让开 Ø14 法兰 → Ø14.6（径向 0.3；09-13 由 +0.3=Ø14.3/径向 0.15 放大，与 lib.flange_relief 同口径）
    discA = cyl(P["seat_disc_d"], YRM_A2 - xf, ((xf + YRM_A2) / 2, *AX), axis="x")                   # 盘：6 颗 M2 的坐面
    # B 端：Ø21.2 凸台 + Ø17.6 内圈台肩（轴颈本身是 N04）
    b0, b1, bd = YRM_B_BOSS; s0, s1, sd = YRM_B_SHLD
    bossB = union(cyl(bd, b1 - b0, ((b0 + b1) / 2, *AX), axis="x"),
                  cyl(sd, s1 - b0, ((b0 + s1) / 2, *AX), axis="x"))
    Ew = wide_y(s288.servo_envelope(), YRM_SKIRT)                 # 帽子沿宽度方向摊开 → 背板 + 两侧裙墙
    Ew2 = Ew.copy(); Ew2.apply_translation((-PLT, 0, 0))
    region = union(wbox((0, YRM_SKIRT_Y0, YRM_SKIRT_Z0), (60, 20, 300.0)),    # 裙墙段（-y 端要让开 H01 翻转时的左纵轨）
                   wbox((0, -40.0, 253.6), (60, 20, 300.0)))                  # 背板段（全宽）
    back = diff(inter(placed(union(Ew, Ew2), Ry), region), servo_env(Ry))
    m = union(O, ringA, discA, bossB, back, HBR.yaw_carrier_features())
    hA = placed(s288.horn_holes(10.0, (S["T"] / 2 + S["flange_h"] + 4.0, 0, 0)), Rr)   # 6×M2 穿 A 盘进法兰（world x 6.1..18.1）
    bore = cyl(16.0, YRM_BORE[1] - YRM_BORE[0], (sum(YRM_BORE) / 2, *AX), axis="x")
    # was_until_2026_09_28_hr50: h01_clr = diff(wbox((-10, -40, 200), (YRM_H01[0], 40, 300)), cyl(2 * YRM_H01[1], 40.0, (YRM_H01[0] - 20, *AX), axis="x"))
    # hr50（2026-09-28）：这把让位把头偏航 6704 座（Ø27.15 孔、外壁 Ø30.15，z 220.2..225）的 −x 壁（x 10.9..12.4）整段削掉，只剩 x 12.40..12.43 的
    #   0.025 皮（clean_check 薄膜 51.4 mm² + holes 开口环 Ø27.2 断口 6.5°，座是保护区）。实测这一带（|y| ≤7，z 219.9..225.3）头横滚 ±25° 全程
    #   只有 H03 伸到 x ≤11.35（|y| ≤4.5 时 ≤11.0），H01 / 头部其它件都不到 → 刀在座壁处收回：|y| ≤4.5 留到 x 11.4、4.5..7 留到 x 11.75（离 H03 ≥0.4），
    #   座 −x 壁回到 ≥1.0（y=0 处 12.425−11.4）。YRM_SEAT_KEEP。
    h01_clr = diff(wbox((-10, -40, 200), (YRM_H01[0], 40, 300)), cyl(2 * YRM_H01[1], 40.0, (YRM_H01[0] - 20, *AX), axis="x"),
                   *[wbox((x0, -y1, z0), (YRM_H01[0] + 0.1, y1, z1)) for (x0, y1, z0, z1) in YRM_SEAT_KEEP])
    n02_clr = cyl(YRM_N02[0], YRM_N02[2] - YRM_N02[1], (26.0, 0.0, (YRM_N02[1] + YRM_N02[2]) / 2))
    rail_clr = wbox((0, -40.0, YRM_SKIRT_Z0), (60, YRM_SKIRT_Y0, 253.6))       # H01 左纵轨翻到 +25° 会升到 y≈-24.3/z≈247.5
    # B 端通孔（N04 导向孔兼 A 端 6 颗螺丝的起子通道，Ø16.2）：从内腔 x 32 一直到台肩外 0.2；腹板内面 Ø18.2×1.0 沉窝坐 N04 凸缘
    x_bore1 = s1 + 0.2
    boreB = cyl(YRM_B_BORE, x_bore1 - 32.0, ((32.0 + x_bore1) / 2, *AX), axis="x")
    cbd, cbt = YRM_B_CB
    cbore = cyl(cbd, cbt + 0.02, (YRM_B_WEB + cbt / 2, *AX), axis="x")
    # 台肩以外（x>40.1）原版那段 Ø16/Ø12 轴颈残料整个铲掉：那里是轴承的位置，只能留 N04。
    # 刀从 s1−0.02 起而不是 s1：和台肩端面共面会在导出 STL 里留 4 个退化面/非流形边（09-12 实测），读回就不是体
    xj = s1 - 0.02
    jclr = cyl(sd + 0.6, 46.0 - xj, ((xj + 46.0) / 2, *AX), axis="x")
    # hr08：H01 的 B 端外圈后挡环（Ø20.8/24.2, x 39.3..40.1）绕横滚轴 ±25° 扫掠时碰到 N03 x 39.1..39.5、r 9.5..13.5 的原版残料 0.92 mm³ —— 环绕自身轴转动是不变量，直接切 Ø20.4..24.6 × x 39.1..40.3
    # was_until_2026_09_28_hr50: hb_clr = diff(cyl(24.6, 1.2, (39.7, *AX), axis="x"), cyl(20.4, 1.4, (39.7, *AX), axis="x"))
    # hr50（2026-09-28）：这道槽把头偏航 6704 上挡圈（r 12.5..15.075）在 x 39.1..40.3 切开，槽外只剩 x 40.3..40.54 的 0.24 外沿（clean_check 薄膜 3.7 mm² ×2
    #   @ (40.4, ±4, 224.5)）。外沿在轴承钢圈接触环（r ≤13.5 → x ≤39.5）之外、收回就撞 H01 后挡环 → 按规矩整片削穿：槽沿 +x 延到 41.3（过挡圈外圆 41.075）。
    hb_clr = diff(cyl(24.6, 41.3 - 39.1, ((39.1 + 41.3) / 2, *AX), axis="x"), cyl(20.4, 41.5 - 38.9, ((38.9 + 41.5) / 2, *AX), axis="x"))
    # hr09：A 端 6 颗 F17 的起子通道（Ø4.4，从坐面 YRM_A2 沿 +x 到 B 端隧道），头部补丁的偏航座根 wbox(12.5..15.5, |y|≤7, z 222..232.5) 挡住下三颗 2.3–5.6 mm³（hr08 头部审计）
    hA_drv = placed(union(*[cyl(4.4, 34.0, (S["T"] / 2 + S["flange_h"] + YRM_A_STACK + 17.05, S["horn_r"] * math.cos(a), S["horn_r"] * math.sin(a)), axis="x")
                             for a in np.linspace(0, 2 * math.pi, S["horn_n"], endpoint=False)]), Rr)
    # hr11：N06（Ø21.6）沿 +z 从空偏航舵机腔取出/装入的行程包络——hr10 头部审计 stage/N06_withdraw_plus_z 在 (36.0..36.29, −4.1..−3.3, 228.2..228.6) 撞主体 0.017 mm³；
    #        与 HBR.yaw_carrier_features 的 n06_path 同 Ø22.4，但只切腔内 x≤36.35（腔壁 36.3+0.05），不碰 B 腹板 36.3..39 与 N04 沉窝
    n06_travel = inter(HBR._yaw_cyl(22.4, 227.0, 232.5), wbox((15.6, -12.0, 226.5), (36.35, 12.0, 233.0)))
    # hr13：N06 只能从空的偏航舵机腔进出，整条路径 = 抬 +z 5 再沿 −y 出腔 40（assembly.yaml 步 11 d，与偏航舵机同一条 −y 通路）。
    #        hr12 复审：n06_travel 只切了腔内直上段；出腔段 Ø21.6 的 ±x 边（x 15.2 / 36.8）擦到 A 侧座根（x 12.5..15.5）与 −y 侧 B 座根
    #        （x 36.4..39.5）4.8–5.1 mm³。这里把 N06 本体沿两段路径做 minkowski 连续扫掠（各向 +0.3，抬升段不向 −z 长：N06 坐在 6704 内圈上，
    #        往下 0.3 会啃外圈台肩）后从本件减掉；只影响 y ≤ 0.3 的半边，+y 侧座根/裙墙不动。B 腹板 36.3..39 在 z 229.3..232.9、y<0.3 处
    #        被削到 x ≥ 37.1（该带正对 Ø16.2 B 通孔，剩下的是孔周环料），N04 沉窝（x 36.3..37.3）在 |y|≤9.1 内本就是空的。
    n06 = HBR.build_head_yaw_adapter(); c, L6 = N06_EXIT_CLR, N06_EXIT_LIFT
    # −x 侧多长 N06_EXIT_XSKIN：hr13 复审发现 A 侧根在 Ø16 让刀孔端面 x 14.8 与槽面 x 14.9 之间留下 0.03–0.34 的皮（~1.7 mm³，打不出来只会成毛刺），
    # 槽面退到 14.8 与孔端面齐平（根 12.5..14.8，仍 2.3 厚）。
    n06_lift = minkowski_box(n06, (-c - N06_EXIT_XSKIN, -c, 0.0), (c, c, L6 + c))
    n06_up = n06.copy(); n06_up.apply_translation((0.0, 0.0, L6))
    n06_exit = minkowski_box(n06_up, (-c - N06_EXIT_XSKIN, -40.0 - c, -c), (c, c, c))
    # 头偏航舵机 PH2.0 插座（keepouts.yaml:KO01，09-20 背插模型，lib.CONN_MODE "window"）：两个插座都在舵机背面（世界 z 253.6 那一面）两角，
    # 盖住它们的是本件背板（z 253.9..256.9 + 座特征到 257.6），背板上方头腔内是空的 → 两个口都在背板上开 5.5×10.2 通窗（刀到局部 −18.5 = z 259.1），
    # 线从背板上方走、插头装好舵机后再插。旧 CF3（+y 侧裙墙切 8.8 宽走廊 / −y 永久不用）作废，裙墙不再开口。
    # 09-14：横滚舵机 SERVO_SHIFT −1.5→−1.7 后法兰面 8.1→8.3，原版 A 端腹板（从原版 XL330 接口面 8.10 起）在 Ø14 内有 0.2 厚一层压进法兰
    # （零位 17.67 mm³，头横滚 ±25° 全程），servo_env 故意不挖法兰侧 → 和 L01/L02/L04/L05/N02 一样加一刀 flange_relief（Ø14.6 × 法兰根下 0.3 到法兰面）
    # hr50：A 侧根（HBR.yaw_carrier_features roots[0]，x 12.5..15.5）被 Ø16 让刀孔（端面 x 14.8）与 N06 出腔扫掠（−x 面 14.8、+y 侧沿 N06 圆弧外退）
    #   夹成 x 14.8..15.5 的 0.1–0.7 残层（clean_check 薄膜 14.0 mm² @ (14.9, 2.5, 230)、3.8 mm²、尖刺 ×3 @ x 14.8）。收回任一刀都撞 N06 出腔或 F17 头 →
    #   按规矩整片削穿：x ≥ 14.75 那层在根的 |y| ≤7.05、z 227.9..232.6 一刀切平（根剩 x 12.5..14.75，Ø16 孔外的部分 2.25 厚）。
    root_trim = wbox((YRM_ROOT_TRIM_X, -7.05, 227.9), (15.7, 7.05, 232.6))
    # hr50：同一个根在 Ø16 让刀孔 x 范围（13.55..14.8）里、孔与根侧面 |y| 7.0 之间 / 孔底与 journal 让位顶 228.2 之间还夹着 0.1–0.4 的楔
    #   （clean_check 刀片 1.7 @ (14.2, −6.9, 231.4)、1.4 ×2 @ (14.4, ±3.4, 228.3)、1.0 @ (14.4, 6.9, 231.4)）→ 这一格（x 13.55..14.8、|y| ≤7.1、z 228.1..232.6）
    #   整片切掉：根在孔的 x 范围外（12.5..13.55）不动。
    root_trim2 = wbox((YRM_BORE[0], -7.1, 228.1), (YRM_BORE[1], 7.1, 232.6))
    # hr50：B 腹板（x 36.3..39）上 Ø16.2 通孔底与 journal 让位顶 228.2 之间只剩 0.1–0.4 的条（刀片 1.1 ×2 @ (37.6, ±3.6, 228.3)）→ |y| ≤4.7 内切平到 229.05
    #   （|y| = 4.7 处孔底 229.0，孔外条 ≥0.8）。
    #   刀 = journal 让位同一根 Ø25 圆柱（128 边）往上延到 229.9 ∩ 与 Ø16.2 通孔同轴的 Ø17.8 圆柱（孔外 0.8）：孔底下 0.8 以内的条整片切掉，
    #   边界全是两根圆柱面，不另起平面（平盒版 r5 在交线处出碎边 n 7–11）。z 上限 229.9 只在孔两侧 |y| 5.7..6.8 的 0.8 环带上起作用。
    bweb_trim = inter(HBR._yaw_cyl(HBR.HEAD_YAW["lip_id"], 228.1, 229.9), HBR._roll_cyl(YRM_B_BORE + 2 * 0.8, 36.2, 40.0))
    # hr50：偏航舵机背板底下副轴让位（servo_env 的 Ø15.8 副轴）外 +y 顶点处留一道 r 7.8..8.0 的 0.2 弧墙（z 254.0..254.6，薄膜 2.1 / 2.0 mm²）→ y ≥7.0 一段切掉
    #   （切后两端 y = 7.0 处弧宽 1.8，不成楔）。上下两块料（z ≥254.7 / ≤253.9）不动。
    back_arc_trim = wbox((18.0, 7.0, 253.92), (34.0, 8.6, 254.68))
    # hr50：−y 侧 B 腹板内层（x 36.3..37.1）在 z 228.6 以上已被 N06 出腔一侧削掉，只剩 z 228.2..228.6 的 0.4 薄沿（y −7.9..−5.5），
    #   上一刀与它相交出碎边（n 6）→ 这条薄沿连同其下 z 227.3..228 的楔形残块一起切掉（y 到根内面 −8.6 为止；x >37.15 的腹板不动）。
    bweb_ledge = wbox((36.2, -8.6, 226.5), (37.15, -4.8, 228.75))
    # was_until_2026_09_28_hr50: return keep_main(diff(m, servo_env(Ry), servo_env(Rr), flange_relief(Rr), hA, HBR.roll_A_separation_cut(), HBR.journal_key_slot(), bore, boreB, cbore, jclr, n06_travel, n06_lift, n06_exit,
    return keep_main(diff(m, servo_env(Ry), servo_env(Rr), flange_relief(Rr), hA, HBR.roll_A_separation_cut(), HBR.journal_key_slot(), bore, boreB, cbore, jclr, n06_travel, n06_lift, n06_exit, root_trim, root_trim2, bweb_trim, back_arc_trim, bweb_ledge,
                          mnt_cut2(Ry, pairs=(0, 1)), h01_clr, n02_clr, rail_clr,
                          conn_cut(Ry, mode="window"),
                          # 上排 2 孔坐面：16 向射线 3.3 / 6.9（裙墙根部有 2 向 3.6 mm 料落在 M2 头足印里），Ø m2_cbore_d 沉到背板外表面 x_bshell_out()=−13.3 铣平
                          # → 叠厚 3.3，M2×6（旧写死 cb_d=6.75 → −13.25，对 12.85 法兰面的背板外表面）
                          mnt_cut2(Ry, pairs=(0,), cb_x0=-20.0, cb_d=x_bshell_out(PLT) - (-20.0)),
                          HBR.yaw_pilot_holes(), hb_clr, hA_drv), "yrm")

def build_head_journal():
    """N04 keyed roll-B journal; N07 and M2x8 clamp the bearing inner ring.
    Flange and key locate N04 in N03. The guide ends0.30 before the bearing;
    the tip ends0.195 before N07, preserving the inner-ring clamp load path.
    Assemble N04 from the empty yaw-servo cavity before fitting that servo.
    Fit, preload, creep and the staged tool/insertion paths remain unverified.
    """
    AX = (0.0, YRM_AXZ)
    x0, xs, x1 = YRM_B_WEB, YRM_B_BRG[0] - 0.3, YRM_B_BRG[1] - 0.2
    fd, ft = N04_FLANGE
    guide = cyl(N04_D_GUIDE, xs - (x0 + 0.3), ((x0 + 0.3 + xs) / 2, *AX), axis="x")
    jrn = cyl(N04_D_JRN, x1 - xs + 0.01, ((xs + x1) / 2, *AX), axis="x")
    flange = inter(cyl(fd, ft, (x0 + ft / 2, *AX), axis="x"), wbox((x0 - 1, -20, YRM_AXZ), (x0 + ft + 1, 20, YRM_AXZ + 20)))
    return HBR.keyed_journal(union(guide, jrn, flange))
