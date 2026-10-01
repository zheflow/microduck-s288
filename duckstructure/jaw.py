"""嘴（下颚）关节：第 15 颗 S288 + J01 活动嘴 + 惰轮侧 6700ZZ。2026-09-23 用户拍板 15 颗全装，A14"只装 14 颗"作废。

━━ hr39c（2026-09-23 头等比放大每侧 +10，duckstructure/head_scale.py）改了什么 ━━
  * 头壳 H03/H05/H04/J01 绕 P0 等比放大 s = 1.2179（壁厚保持）。嘴舵机、6700、H01 不放大。
  * **嘴轴挪到 (31.75, y, 245.5)**，舵机绕嘴轴转 ψ = 5°（顶端朝 −(cos5°, 0, sin5°)，插座端朝 +x 略向上），法兰面 y 47.75。
    依据：hr39c 重搜（docs/design_2026-09-17_bearing_rebuild/hr39c_work/jaw_scan*.py，全平移体素 → jaw_batch 精确复核三轮）：
      - 旧嘴轴（放大后 (26.12, 243.06)）附近插座离那摞件只有 1.5、壳深 1.1 → 不行；
      - Radxa 右下立柱原样 → 全头 0 个可行摆法；N03/N08 +y 耳不挪 → 0 个可行摆法（两条都改了，见 head.py / head_bearing_rebuild.py）；
      - 选定点精确值（191 横滚 × 7 件 + 1691 偏航×横滚对 × N02/N06，全部真实姿态）：本体 4.29 / 插座 2.89、2.41 / 线 2.89、2.33；
        离 Radxa（按官方图）1.72、上 +y 立柱 2.40；本体到放大后外壳 1.91（壁 1.2 + 0.4 够，不鼓包）。
  * 旧值（hr38）：嘴轴 (26.1, 238.6146)、ψ 0（顶端 −x）、法兰面 y 40.0、惰轮座 y −39.5..−35.5 → 见 *_was_until_2026_09_23_hr39c。
  * 两个插座都用（KO01 row 15 两口 free）：嘴改到头链中间，一分二回到 5 根（harness.yaml）。
  * 固定方式照原版思路：H01 顶板 + 端墙从上/两端挡，H03 托台从下托，背面不加板（背面是那摞件扫掠）；嘴子总成（舵机 + J01 + 6700）
    仍从下方 +z 推进 H01（口袋底开、惰轮拱底开），H03 合上时托台 + 下半环同时夹住。
  * J01 = 放大后的原版 jaw：+y 臂在新嘴轴处长 Ø19 毂盘（盖住原版 XL330 孔、给 6 颗 M2 沉窝留料）+ Ø15 垫台（放大后臂内面 y 49.55 到法兰面 47.75 差 1.8）；
    −y 臂去掉原版短轴（放大后在老嘴轴处），在新嘴轴处长 Ø9.95 轴颈 + 毂盘。

━━ hr41（2026-09-24 头部阶段二；复审 #1 B1 BLOCKER 装配死锁，用户定方案 A：J01 最后从下方 +z 装、沉头外露可以）━━
  * J01 v2：去掉 +y Ø15 垫台、−y Ø9.95 轴颈、6 个法兰孔/沉窝；两臂内面在嘴轴一圈削平到 |y| 49.55，
    x 12..44、z 232..257 带内内面再外挪到 |y| 52.05（让开 H03 合缝下沿外鼓 |y| ≤ 51.41），外面加厚 1.0（53.85 → 54.85）；
    两毂各 3 个 M2 沉头孔 r 7.1 @ 350/130/250°（**不对称 = 零位防错**，见 JAW_HUB_SCREW_ANG 注释）。
  * J02（新件，+y 法兰转接盘）：Ø15 颈 y 47.75..49.55（穿壳 Ø16 孔）+ Ø20×2.5 盘 49.55..52.05；3 颗 M2×5 盘头自攻到舵机法兰 @0/120/240°（一次装好不拆）；
    3 个 M2 六角螺母穴（压入，从 −y 侧装），外侧皮 0.85。
  * J03（新件，−y 可拆轴颈盘）：Ø9.95 轴颈 −49.55..−45.05 进 6700 内圈 + Ø20×2.5 盘 −52.05..−49.55，3 个螺母穴从盘内面开口。
  * J01 → J02/J03：各 3 颗 M2×5 沉头机牙 + 压入六角螺母（全机第一处机牙；不自攻、不热熔）。
  * 装序 P（hr39d 阶段一）：Radxa → 嘴舵机+J02 −y 横插 → H03 +z 合 → H05+H04 −z 盖 → J03 +y 插 → J01 从下 +z → 圆眼。
  * H05 保持：hr39c 靠 J01 毂卡在 H05 Ø21 圆让位里（复审 M4 那 3.33 mm³）；A 方案后 +y 必须开 21 宽槽，−y 的 J03 盘上方 H05 实测几乎没料
    （盘上抬 3 mm 只交 0.11 mm³，hr41_work/scripts/probe_lock.py）→ 锁不住。改成**两侧都开 21 槽 + H03 底下 2 颗 M2×8 自攻拧进 H05 凸台**（head_top.H05_KEEP_*）。
  改前常量/函数留在 *_was_until_2026_09_24_hr41。

原版事实（cad/microduck_head 三个 STL 自己量的，世界系；placed 与 cad/microduck_head 之间 dx=+8.1, dz=+235.6146）：
  · 原版嘴轴线 (x 26.100, z 238.615) 沿 y；原版嘴舵机 XL330 法兰朝 +y、+y 臂内面 y 40.0 = 舵盘面；嘴臂在头壳**外面**，
    上壳 / 下壳在枢轴处合成 ≥Ø18 的孔（上壳管上半、下壳管下半，所以嘴子总成能从下面推上去再合 H03）。
  · 张嘴行程：inventory_original.json mouth_mechanism −5°..+30°；−5° 是软嘴（TPU）压缩量，硬件行程 0..30°。
原模型来自 Pollen Robotics Microduck，沿用原资产的 CC BY-SA-NC 许可。
"""
import math, numpy as np, trimesh
from . import s288
from .s288 import S, cyl, union, diff, inter, placed, frame
from .lib import P, CLR, TW, orig, wbox, conn_zone, servo_env, ANK_J, minkowski_box
from trimesh.transformations import rotation_matrix as rot

# ── 轴线 / 行程（世界系）
JAW_AXIS_P = np.array([31.75, 0.0, 245.5])            # hr39c：新嘴轴（见文件头）
JAW_AXIS_P_was_until_2026_09_23_hr39c = (26.1, 0.0, 238.6146)
JAW_AXIS_D = np.array([0.0, 1.0, 0.0])                 # 绕 +y 右手：正角 = 嘴尖往 −z = 张嘴
JAW_RANGE = (0.0, 30.0)                                # 硬件扫掠范围（0 = 闭合）
JAW_RANGE_ORIG = (-5.0, 30.0)                          # 原版 mouth_mechanism；−5 是软嘴压缩量。hr39c：J01 −5..+30 全程对壳 / H01 / 静态件 0 碰（壳按扫掠让位）
JAW_RELIEF_RANGE = (-5.0, 30.0)                        # 壳让位刀用的扫掠范围（= JAW_RANGE_ORIG）
JAW_SWEEP_STEP = 5.0

# ── 嘴舵机（第 15 颗 S288）
JAW_PSI_DEG = 5.0                                      # 绕嘴轴转的角：顶端方向 −(cosψ, 0, sinψ)
JAW_FLANGE_Y = 47.75                                   # 法兰凸台外端面（= J01 +y 垫台贴合面）
JAW_SERVO_FRONT = (0.0, 1.0, 0.0)
JAW_SERVO_LONG = (-math.cos(math.radians(JAW_PSI_DEG)), 0.0, -math.sin(math.radians(JAW_PSI_DEG)))
JAW_SERVO_C = np.array([JAW_AXIS_P[0], JAW_FLANGE_Y - s288.x_flange_face(), JAW_AXIS_P[2]])   # (31.75, 34.75, 245.5)
JAW_SERVO_C_was_until_2026_09_23_hr39c = (26.1, 27.0, 238.6146)
JAW_REAR_Y = float(JAW_SERVO_C[1] + s288.x_rear_face_lo())            # 21.75：厚段背面 = 惰轮端面
JAW_CONN_MODE = {-1: "free", 1: "free"}   # hr39c：两口都用（局部 −y ≈ 世界 +z 上插座 / 局部 +y ≈ 世界 −z 下插座；离那摞件 2.41 / 2.89）
JAW_CONN_MODE_was_until_2026_09_23_hr39c = {-1: "free", 1: "none"}

def servo_R():
    """嘴舵机世界帧（法兰朝 +y，顶端朝 −(cos5°, 0, sin5°)）。本体 x 22.3..57.0，y 21.75..47.75（含法兰凸台），z 234.7..257.6。"""
    return frame(JAW_SERVO_C, JAW_SERVO_FRONT, JAW_SERVO_LONG)

def servo_mesh_world(): return placed(s288.servo_mesh(), servo_R())
def servo_env_world(extra=0.0): return servo_env(servo_R(), extra)
def conn_zone_world(sgn, kind="A"): return placed(conn_zone(sgn, kind), servo_R())
def horn_holes_world(thick=8.0):
    """法兰 6×Ø2.4 @ r 5.25 + 中心 Ø5，沿 y 打穿，中心在法兰面上"""
    return s288.horn_holes(thick, (JAW_AXIS_P[0], JAW_FLANGE_Y, JAW_AXIS_P[2]), axis="y")

# ── J01 两臂在新嘴轴处的面（放大后的原版 jaw 实测：hr39c_work/jaw_arm_probe.py；臂板 4.3 厚 → 49.55..53.9）
J01_ARM_IN_Y = 49.55
J01_ARM_OUT_Y = 53.85

# ── 惰轮侧 6700ZZ（10×15×4）：外圈在 H01 上半拱 + H03 下半环拼的 Ø15.1 座里，内圈套 J01 的 Ø9.95 轴颈
JAW_BRG = dict(od=15.0, bore=10.0, t=4.0, seat_d=15.1,
               y0=-(J01_ARM_IN_Y - 0.5), y1=-(J01_ARM_IN_Y - 4.5),          # −49.05..−45.05：臂内面外 0.5 起、宽 4
               journal_d=ANK_J, journal_y0=-J01_ARM_IN_Y, journal_y1=-(J01_ARM_IN_Y - 4.5),
               arm_face=-J01_ARM_IN_Y)
JAW_BRG_was_until_2026_09_23_hr39c = dict(y0=-39.5, y1=-35.5, journal_y0=-40.0, journal_y1=-35.5, arm_face=-40.0)
JAW_JOURNAL_CLR_D = JAW_BRG["journal_d"] + 1.0     # 轴颈尖端外 Ø11 让位
JAW_SEAT_OD = 18.6                                 # 惰轮座环外径（H01 上半拱 / H03 下半环各一半；壁 (18.6−15.1)/2 = 1.75 ≥1.2）。座环正好落在壳壁那个 y 位置上，
                                                   # 壳在这里开 Ø19.2 孔让它（SHELL_HOLE_D_NEG），J01 毂盘 Ø20 从外面盖住（同原版：座 + 壳孔 + 臂端圆盘盖）

def bearing_mesh_world():
    b = JAW_BRG
    return diff(cyl(b["od"], b["t"], (JAW_AXIS_P[0], (b["y0"] + b["y1"]) / 2, JAW_AXIS_P[2]), axis="y"),
                cyl(b["bore"], b["t"] + 2, (JAW_AXIS_P[0], (b["y0"] + b["y1"]) / 2, JAW_AXIS_P[2]), axis="y"))

def seat_bore_cut():
    """Ø15.1 × (y0−0.02 .. y1+0.02) 座孔刀，对中嘴轴。H01 上半、H03 下半各切一次同一把刀。"""
    b = JAW_BRG
    return cyl(b["seat_d"], abs(b["y1"] - b["y0"]) + 0.04, (JAW_AXIS_P[0], (b["y0"] + b["y1"]) / 2, JAW_AXIS_P[2]), axis="y")

def seat_ring(half):
    """惰轮座环（外 Ø JAW_SEAT_OD、宽 = 轴承宽 y0..y1，Ø15.1 通孔），half = 'upper'（H01）/'lower'（H03）。
    轴向同 hr38：外圈在座里，内圈套轴颈；臂内面（−49.55）到轴承 −y 面（−49.05）0.5 浮动。"""
    b = JAW_BRG; x0, z0 = JAW_AXIS_P[0], JAW_AXIS_P[2]
    ya, yb = b["y0"], b["y1"]
    ring = diff(cyl(JAW_SEAT_OD, yb - ya, (x0, (ya + yb) / 2, z0), axis="y"), seat_bore_cut())
    box = wbox((x0 - 20, ya - 1, z0), (x0 + 20, yb + 1, z0 + 20)) if half == "upper" else wbox((x0 - 20, ya - 1, z0 - 20), (x0 + 20, yb + 1, z0))
    return inter(ring, box)

def journal_clear_cut():
    """轴颈尖端里侧 1.0 的 Ø11 让位（打印过长/装配浮动）"""
    b = JAW_BRG
    return cyl(JAW_JOURNAL_CLR_D, 1.02, (JAW_AXIS_P[0], b["y1"] + 0.5, JAW_AXIS_P[2]), axis="y")

# ── J01 活动嘴（放大后的原版 jaw + 两个新枢轴接口）
J01_NAT_PIVOT = None                                  # 放大后原版嘴轴（在 build_jaw 里由 head_scale 算：(26.12, 243.06)）
J01_HUB_D = 20.0                                       # 两臂新毂盘 Ø（+y：6 颗 M2 沉窝 r 5.25+2.2 = 7.45，外留 2.55；−y：盖住壳上 Ø19.2 座环孔）
J01_PAD_D = 15.0                                       # +y 垫台 Ø（穿头壳 Ø16 孔），y 47.75..臂内面
J01_CBORE_D, J01_CBORE_Y0 = P["m2_cbore_d"], JAW_FLANGE_Y + 3.5      # 51.25：叠厚 3.5，M2×6 咬 2.5（同 hr38）
J01_OLD_FILL_D = 21.0                                  # 老嘴轴处原版 XL330 孔（放大后 4 孔 r 7.3 + 中心孔）整片填掉
J01_STUB_CUT_D = 16.2                                  # 老嘴轴处 −y 原版短轴（Ø11.4 台阶 + Ø9.8 轴颈，放大后 Ø13.9 / Ø11.9）削掉

def _arm_hull():
    """放大后 jaw 的凸包（毂盘/填料的外端面用它截，跟着臂外表面的弧走、不鼓出来）"""
    from . import head_scale as HS
    return trimesh.convex.convex_hull(HS.scaled_orig("jaw").vertices)

# ── hr41 方案 A v2（hr39d_work/scripts/proto2.py 的正式版）
J01_V2_BAND = (12.0, 44.0, 232.0, 257.0)               # 臂内面外挪的区域：x0, x1, z0, z1（proto2.RG；H03 合缝外鼓挡路的料全在这块里，hr39d blocking.json）
J01_V2_IN_Y = 52.05                                     # 带内两臂内面 |y|（外鼓最大 51.41 → 余 0.64）
J01_V2_THICKEN = 1.0                                    # 带内臂外面加厚 1.0（毂厚 2.8）
J01_V2_OUT_Y = J01_ARM_OUT_Y + J01_V2_THICKEN          # 54.85
J01_V2_FLAT_BOX = ((10.0, -J01_ARM_IN_Y, 230.0), (45.0, J01_ARM_IN_Y, 260.0))   # 放大后两臂内面在嘴轴一圈 0.84 的凸起（|y| 48.71..49.55）削平
# 转接盘 ↔ J01 的 3+3 颗 M2 沉头机牙：分度圆 r 7.1。角度故意**不对称**（50/180/310，不是 60/180/300）：
#   J02 的法兰螺丝只用法兰 6 孔里的 3 个（0/120/240），J02 装上法兰有 6 种相位；对称孔位时装错 120° 的 J02 照样能拧上 J01 → 零位差 120° 不会被发现。
#   不对称后 6 种相位里只有 1 种能让 3 颗全对上（其余错 10–60°，r 7.1 处 ≥1.2 mm，Ø2.4 过孔对 M2 只容 0.2）→ 舵机上电停在零位时装错 J02 就拧不上 J01。
#   阶段一方案写的「J02 外面 D 形凸台 + J01 毂 D 形槽」不做：J01 从下方 +z 竖直装入，凸台要穿过 J01 毂下方整段下缘壁（|y| 49.5..53.5 的双层壁），
#   只能在壁里开通槽，壁被劈开 → 改用不对称孔位防错（hr41 报告 §A）。
JAW_HUB_SCREW_R = 7.1                                   # 7.0 时螺母穴内平面 r 4.9 啃 J03 轴颈 Ø9.95；7.1 → 内平面 5.0、外平面 9.2（离盘边 0.8）
JAW_HUB_SCREW_ANG = (350.0, 130.0, 250.0)
# 角度怎么来的（hr41_work/scripts/t_jawmap.py 外表面图 + 盘内干涉）：J02 法兰螺丝用 60/180/300 那组孔，螺母穴离它们的沉窝要 Δ ≥ 47.8°
#   （r 7.1 六角外接 2.43 + 沉窝 Ø4.4 + 壁 0.8）→ 只剩 ≈0/120/240 ±12° 三个窗；J01 臂顶边在嘴轴右上（x > 31）只到 z ≈251.5，锥头 r 2.1 要在臂内 →
#   取 350/130/250（彼此 140/120/100°，不对称）。转 120°/240° 装错时有两颗错 20°（r 7.1 处 2.5 mm），换另一组法兰孔（错 60°）三颗全错。
JAW_HUB_SCREW_ANG_was_2026_09_24_hr41_draft = (50.0, 180.0, 310.0)
JAW_CS = dict(hole_d=2.4, cone_d=4.2, cone_h=0.9, head_dk=3.8, head_k=1.2, L=5.0,
              src="M2×5 十字沉头机牙（ISO 7046-1：dk max 3.8、k max 1.2，datasheet；长度含头）；过孔 2.4 同全机口径；90° 锥 Ø4.2→2.4 深 0.9 → 头顶沉 0.2")
JAW_NUT = dict(s=4.0, m=1.6, pocket_s=4.2, pocket_h=1.65,
               src="M2 六角螺母 ISO 4032：s 4.0 / m 1.6（datasheet）；穴对边 4.2 = 按本机竖直小孔缩 0.15/边打出来 ≈3.9（压入，assumed，TC02 同口径，没试打）")
J02_NECK_D, J02_DISC_D = J01_PAD_D, J01_HUB_D          # Ø15 颈（原垫台）/ Ø20 盘（原毂）
J02_SCREW_ANG = (60.0, 180.0, 300.0)                    # 法兰 6 孔（r 5.25，k×60°）里用 3 个（奇数号）
J02_CB_FLOOR_Y = 50.35                                  # 盘头沉窝底：叠厚 50.35 − 47.75 = 2.6 → M2×5 盘头自攻咬进法兰 2.4（≤ 手册 3.0 MAX）；头 1.6 → 头顶 51.95，低于盘外面 0.1
J02_CENTER = (S["horn_center_d"], 50.55)               # 中心 Ø5 让位（同 J01 旧口径「中心 Ø5 让位」），从法兰面到 y 50.55（盲，留盘 1.5）

def _frustum_y(d0, y0, d1, y1, x0, z0, n=48):
    """沿 y 的圆台（y0 处 Ø d0 → y1 处 Ø d1）"""
    a = np.linspace(0, 2 * math.pi, n, endpoint=False)
    pts = [(x0 + d / 2 * math.cos(t), y, z0 + d / 2 * math.sin(t)) for (d, y) in ((d0, y0), (d1, y1)) for t in a]
    return trimesh.convex.convex_hull(np.array(pts))

def _hex_y(s, y0, y1, x0, z0, flat_normal_deg):
    """沿 y 的六角柱（对边 s），一个平面的外法线在 x–z 平面里朝 flat_normal_deg（从 +x 转向 +z）"""
    R = s / math.sqrt(3.0); th0 = math.radians(flat_normal_deg) - math.pi / 6
    pts = [(x0 + R * math.cos(th0 + k * math.pi / 3), y, z0 + R * math.sin(th0 + k * math.pi / 3)) for y in (y0, y1) for k in range(6)]
    return trimesh.convex.convex_hull(np.array(pts))

def hub_screw_xz():
    x0, z0 = JAW_AXIS_P[0], JAW_AXIS_P[2]
    return [(x0 + JAW_HUB_SCREW_R * math.cos(math.radians(a)), z0 + JAW_HUB_SCREW_R * math.sin(math.radians(a)), a) for a in JAW_HUB_SCREW_ANG]

def jaw_cs_cuts():
    """J01 两毂 3+3 个沉头孔：Ø2.4 穿透 + 90° 锥（外面 Ø4.2 → 深 0.9 处 Ø2.4），外面以外再 Ø4.2 开 2 mm。外面按 J01_V2_OUT_Y + 0.05（毂外端 ≤ 54.9）"""
    out = []; yo = J01_V2_OUT_Y + 0.05; c = JAW_CS
    for s in (1, -1):
        for (x, z, _a) in hub_screw_xz():
            out.append(cyl(c["hole_d"], 6.0, (x, s * (J01_V2_IN_Y + 2.0), z), axis="y"))
            out.append(_frustum_y(c["hole_d"], s * (yo - c["cone_h"]), c["cone_d"], s * yo, x, z))
            out.append(cyl(c["cone_d"], 2.0, (x, s * (yo + 1.0), z), axis="y"))
    return out

def build_jaw():
    """J01 v2（hr41）：见文件头 hr41 段。改前版本 = build_jaw_was_until_2026_09_24_hr41。"""
    from . import head_scale as HS
    O = HS.scaled_orig("jaw")
    nat = HS.S([26.1, 0.0, 238.6146]); xn, zn = float(nat[0]), float(nat[2])
    x0, z0 = JAW_AXIS_P[0], JAW_AXIS_P[2]
    hull = _arm_hull()
    yo, yi = J01_ARM_OUT_Y + 0.6, J01_ARM_IN_Y - 0.6
    stub = cyl(J01_STUB_CUT_D, 6.0, (xn, -(J01_ARM_IN_Y - 3.0 - 0.01), zn), axis="y")
    m = diff(O, stub)
    fills = [inter(cyl(J01_OLD_FILL_D, yo - yi, (xn, s * (yo + yi) / 2, zn), axis="y"), hull) for s in (1, -1)]
    fills = [diff(f, wbox((xn - 20, -J01_ARM_IN_Y, zn - 20), (xn + 20, J01_ARM_IN_Y, zn + 20))) for f in fills]
    hubs = [diff(inter(cyl(J01_HUB_D, yo - yi, (x0, s * (yo + yi) / 2, z0), axis="y"), hull, wbox((x0 - 20, -J01_ARM_OUT_Y - 0.05, z0 - 20), (x0 + 20, J01_ARM_OUT_Y + 0.05, z0 + 20))),
                 wbox((x0 - 20, -J01_ARM_IN_Y, z0 - 20), (x0 + 20, J01_ARM_IN_Y, z0 + 20))) for s in (1, -1)]
    m = union(m, *fills, *hubs)
    m = diff(m, wbox(*J01_V2_FLAT_BOX))                                      # 内侧 0.84 凸起削平（hr39d proto.J01_A flatten）
    bx0, bx1, bz0, bz1 = J01_V2_BAND
    for s in (1, -1):
        ya, yb = sorted((s * J01_V2_IN_Y, s * (J01_ARM_OUT_Y + 0.05)))
        slab = inter(m, wbox((bx0, ya, bz0), (bx1, yb, bz1)))                 # 带内臂外层（52.05..53.9）
        ya2, yb2 = sorted((s * 40.0, s * J01_V2_IN_Y))
        m = diff(m, wbox((bx0, ya2, bz0), (bx1, yb2, bz1)))                  # 内面挪到 52.05
        slab.apply_translation((0.0, s * J01_V2_THICKEN, 0.0))
        m = union(m, slab)                                                     # 外面加厚 1.0
    # 毂外面补平：放大后的臂外面在毂下半圈是斜的（|y| 54.2..54.9，t_jawmap.py），沉头 Ø4.2 锥口会露出 0.3–0.5 →
    #   Ø20 盘在 52.05..54.9 补平，但只在臂自己的 x–z 轮廓内（取带内最内 0.25 层的轮廓沿 y 拉伸），不在臂顶边以上长料
    for s in (1, -1):
        ya, yb = sorted((s * J01_V2_IN_Y, s * (J01_V2_IN_Y + 0.25)))
        layer = inter(m, wbox((bx0, ya, bz0), (bx1, yb, bz1)))
        prism = minkowski_box(layer, (-0.001, min(0.0, s * 3.0), -0.001), (0.001, max(0.0, s * 3.0), 0.001))
        disc = cyl(J01_HUB_D, J01_V2_OUT_Y + 0.05 - J01_V2_IN_Y, (x0, s * (J01_V2_IN_Y + J01_V2_OUT_Y + 0.05) / 2, z0), axis="y", sections=128)
        m = union(m, inter(disc, prism))
    return diff(m, *jaw_cs_cuts())

def build_jaw_adapter():
    """J02：+y 舵机法兰转接盘（hr41 新件）。一次拧到舵机法兰不再拆；J01 +y 毂从外面贴它，3 颗 M2×5 沉头拧进盘里的压入螺母。"""
    x0, z0 = JAW_AXIS_P[0], JAW_AXIS_P[2]; yf, y1, y2 = JAW_FLANGE_Y, J01_ARM_IN_Y, J01_V2_IN_Y
    neck = cyl(J02_NECK_D, y1 - yf + 0.05, (x0, (yf + y1 + 0.05) / 2, z0), axis="y")
    disc = cyl(J02_DISC_D, y2 - y1, (x0, (y1 + y2) / 2, z0), axis="y", sections=128)
    cuts = []
    for a in J02_SCREW_ANG:                                                    # 法兰 3 颗 M2×5 盘头自攻：Ø2.4 过孔 + Ø4.4 沉窝
        x, z = x0 + S["horn_r"] * math.cos(math.radians(a)), z0 + S["horn_r"] * math.sin(math.radians(a))
        cuts.append(cyl(S["horn_hole_d"], 6.0, (x, yf + 2.0, z), axis="y"))
        cuts.append(cyl(P["m2_cbore_d"], y2 + 1.0 - J02_CB_FLOOR_Y, (x, (J02_CB_FLOOR_Y + y2 + 1.0) / 2, z), axis="y"))
    d5, y5 = J02_CENTER
    cuts.append(cyl(d5, y5 - yf + 1.0, (x0, (yf - 1.0 + y5) / 2, z0), axis="y"))
    for (x, z, a) in hub_screw_xz():                                           # 螺母穴：从 −y 侧（法兰面 / 盘内面）开口到穴底 y 51.2（外侧皮 0.85）；M2 过孔穿到盘外面
        cuts.append(_hex_y(JAW_NUT["pocket_s"], yf - 0.5, y1 + JAW_NUT["pocket_h"], x, z, a))
        cuts.append(cyl(JAW_CS["hole_d"], 4.0, (x, y2, z), axis="y"))
    return diff(union(neck, disc), *cuts)

def build_jaw_journal():
    """J03：−y 可拆轴颈盘（hr41 新件）。Ø9.95 轴颈进 6700 内圈；J01 −y 毂从外面贴它，3 颗 M2×5 沉头拧进盘里的压入螺母（穴从盘内面开口）。"""
    x0, z0 = JAW_AXIS_P[0], JAW_AXIS_P[2]; b = JAW_BRG
    y1, y2 = J01_ARM_IN_Y, J01_V2_IN_Y
    journal = cyl(b["journal_d"], abs(b["journal_y1"] - b["journal_y0"]) + 0.05, (x0, (b["journal_y0"] + b["journal_y1"]) / 2 - 0.025, z0), axis="y")
    disc = cyl(J02_DISC_D, y2 - y1, (x0, -(y1 + y2) / 2, z0), axis="y", sections=128)
    cuts = []
    for (x, z, a) in hub_screw_xz():
        cuts.append(_hex_y(JAW_NUT["pocket_s"], -(y1 + JAW_NUT["pocket_h"]), -(y1 - 0.02), x, z, a))
        cuts.append(cyl(JAW_CS["hole_d"], 4.0, (x, -y2, z), axis="y"))
    return diff(union(journal, disc), *cuts)

def build_jaw_was_until_2026_09_24_hr41():
    from . import head_scale as HS
    O = HS.scaled_orig("jaw")
    nat = HS.S([26.1, 0.0, 238.6146]); xn, zn = float(nat[0]), float(nat[2])
    x0, z0 = JAW_AXIS_P[0], JAW_AXIS_P[2]
    hull = _arm_hull()
    yo, yi = J01_ARM_OUT_Y + 0.6, J01_ARM_IN_Y - 0.6          # 盘的 y 范围（外端再被凸包截）
    # 老嘴轴处：±y 都把原版孔/短轴填平（到臂内面），−y 先削短轴
    stub = cyl(J01_STUB_CUT_D, 6.0, (xn, -(J01_ARM_IN_Y - 3.0 - 0.01), zn), axis="y")      # −49.54..−43.54
    m = diff(O, stub)
    fills = [inter(cyl(J01_OLD_FILL_D, yo - yi, (xn, s * (yo + yi) / 2, zn), axis="y"), hull) for s in (1, -1)]
    fills = [diff(f, wbox((xn - 20, -J01_ARM_IN_Y, zn - 20), (xn + 20, J01_ARM_IN_Y, zn + 20))) for f in fills]   # 只填臂板厚度内（|y| ≥ 49.55）
    hubs = [diff(inter(cyl(J01_HUB_D, yo - yi, (x0, s * (yo + yi) / 2, z0), axis="y"), hull, wbox((x0 - 20, -J01_ARM_OUT_Y - 0.05, z0 - 20), (x0 + 20, J01_ARM_OUT_Y + 0.05, z0 + 20))),
                 wbox((x0 - 20, -J01_ARM_IN_Y, z0 - 20), (x0 + 20, J01_ARM_IN_Y, z0 + 20))) for s in (1, -1)]   # 外端面不超出臂外面 53.9（凸包在臂轮廓外会鼓 0.3）
    pad = cyl(J01_PAD_D, J01_ARM_IN_Y + 0.3 - JAW_FLANGE_Y, (x0, (JAW_FLANGE_Y + J01_ARM_IN_Y + 0.3) / 2, z0), axis="y")
    b = JAW_BRG
    journal = cyl(b["journal_d"], abs(b["journal_y1"] - b["journal_y0"]) + 0.3, (x0, (b["journal_y0"] + b["journal_y1"]) / 2 - 0.15, z0), axis="y")
    # −y 臂内面在新嘴轴一圈里并不平（放大后局部到 −48.7，原版 +y/−y 臂内面 XL330 舵盘/短轴那一圈的台）→ 轴承 −y 面 −49.05、H01 座环都会顶到。
    #   在新嘴轴 Ø(座环外径 + 1.0) 里把 −y 臂内面挖平到 −49.55（先挖、再长轴颈）；+y 侧垫台只在 r ≤ 7.5，外圈没有配合件，不挖。
    recess = cyl(JAW_SEAT_OD + 1.0, 8.0, (x0, -(J01_ARM_IN_Y - 4.0), z0), axis="y")             # y −49.55..−41.55
    m = diff(union(m, *fills, *hubs), recess)
    m = union(m, pad, journal)
    cb = [cyl(J01_CBORE_D, 8.0, (x0 + S["horn_r"] * math.cos(a), J01_CBORE_Y0 + 4.0, z0 + S["horn_r"] * math.sin(a)), axis="y")
          for a in np.linspace(0, 2 * math.pi, S["horn_n"], endpoint=False)]
    m = diff(m, horn_holes_world(16.0), *cb)
    return m

def jaw_R(angle_deg):
    return rot(math.radians(float(angle_deg)), JAW_AXIS_D, JAW_AXIS_P)

def jaw_at(m, angle_deg): return placed(m, jaw_R(angle_deg))

def jaw_screw_heads_world():
    """hr41：随嘴转的螺丝头 —— J02 法兰 3 颗 M2×5 盘头（Ø4×1.6，坐在沉窝底 y 50.35）+ J01 两毂 6 颗 M2×5 沉头（按 Ø3.8×1.2 圆柱，头顶沉 0.2）"""
    x0, z0 = JAW_AXIS_P[0], JAW_AXIS_P[2]; c = JAW_CS; yt = J01_V2_OUT_Y + 0.05 - 0.2
    pans = [cyl(P["m2_head_d"], P["m2_head_h"], (x0 + S["horn_r"] * math.cos(math.radians(a)), J02_CB_FLOOR_Y + P["m2_head_h"] / 2, z0 + S["horn_r"] * math.sin(math.radians(a))), axis="y")
            for a in J02_SCREW_ANG]
    cs = [cyl(c["head_dk"] - 0.4, c["head_k"] - 0.3, (x, s * (yt - (c["head_k"] - 0.3) / 2 - 0.15), z), axis="y") for s in (1, -1) for (x, z, _a) in hub_screw_xz()]
    return union(*pans, *cs)

def flange_screw_heads_world_was_until_2026_09_24_hr41():
    """6 颗 M2×6 盘头（Ø4×1.6）坐在沉窝底 y 51.25 上，随嘴转"""
    x0, z0 = JAW_AXIS_P[0], JAW_AXIS_P[2]
    return union(*[cyl(P["m2_head_d"], P["m2_head_h"], (x0 + S["horn_r"] * math.cos(a), J01_CBORE_Y0 + P["m2_head_h"] / 2, z0 + S["horn_r"] * math.sin(a)), axis="y")
                   for a in np.linspace(0, 2 * math.pi, S["horn_n"], endpoint=False)])

def flange_driver_corridors(length=30.0, d=4.2):
    """6 条法兰螺丝起子通道：从沉窝底沿 +y 向外（头壳外面）"""
    x0, z0 = JAW_AXIS_P[0], JAW_AXIS_P[2]
    return [cyl(d, length, (x0 + S["horn_r"] * math.cos(a), J01_CBORE_Y0 + length / 2, z0 + S["horn_r"] * math.sin(a)), axis="y")
            for a in np.linspace(0, 2 * math.pi, S["horn_n"], endpoint=False)]

# ── 壳上的让位（head.py H03 / head_top.py H05 各调一次）
SHELL_HOLE_D = J01_PAD_D + 1.0                         # +y Ø16：法兰凸台 Ø14 / 垫台 Ø15 从这里穿壁（径向 0.5）
SHELL_HOLE_D_NEG = JAW_SEAT_OD + 0.6                   # −y Ø19.2：惰轮座环（H01/H03，静止）穿壁，径向 0.3
SHELL_HUB_RELIEF_D = J01_HUB_D + 1.0                   # Ø21：毂盘外侧（|y| ≥ 臂内面 − 0.5）壳要让开（毂绕自身中心转，足印不变）；hr41 起这里转的是 J02/J03 的 Ø20 盘
HUB_SLOT_W, HUB_SLOT_Z_BOTTOM = SHELL_HUB_RELIEF_D, 225.0   # hr41：H05 两侧把 Ø21 圆让位向下开成 21 宽槽到 z 225（+y：H05 −z 盖下时让 J02 盘；−y：换 TF 卡抬 H05 时让 J03 盘）
def h05_hub_slots(sides=(1, -1)):
    """hr41 H05：|y| ≥ 49.05（= Ø21 圆让位的起点）、x 嘴轴 ± 10.5、z 225..嘴轴 的槽（hr39d proto2.H05_B）"""
    x0, z0 = JAW_AXIS_P[0], JAW_AXIS_P[2]; h = HUB_SLOT_W / 2
    return [wbox((x0 - h, min(s * (J01_ARM_IN_Y - 0.5), s * 60.0), HUB_SLOT_Z_BOTTOM), (x0 + h, max(s * (J01_ARM_IN_Y - 0.5), s * 60.0), z0)) for s in sides]
SLOT_W = J01_PAD_D + 1.0                               # 插入槽宽 16（垫台 / 轴承 Ø15 从下方推上来）
SLOT_Z_BOTTOM = 225.0                                  # 从新孔往下开到这里（嘴子总成从下方 +z 推入的通道；H05 下沿在这之上）
def shell_pivot_cuts(slot=True):
    """两侧：新孔（+y Ø16 / −y Ø19.2，穿壁）+ 毂盘外侧 Ø21 让位 + （slot=True）从孔往下到 SLOT_Z_BOTTOM 的 16 宽插入槽。
    H05 用 slot=True（嘴子总成从下面推上来要过）；H03 用 slot=False（H03 最后合上，它在孔下面那段壁照留，托住垫台/座环）。"""
    x0, z0 = JAW_AXIS_P[0], JAW_AXIS_P[2]; h = SLOT_W / 2
    out = []
    for s, hd in ((1, SHELL_HOLE_D), (-1, SHELL_HOLE_D_NEG)):
        out.append(cyl(hd, 14.0, (x0, s * (J01_ARM_IN_Y - 5.0), z0), axis="y"))
        out.append(cyl(SHELL_HUB_RELIEF_D, 8.0, (x0, s * (J01_ARM_IN_Y - 0.5 + 4.0), z0), axis="y"))
        if slot:
            y_lo, y_hi = sorted((s * (J01_ARM_IN_Y - 12.0), s * (J01_ARM_IN_Y + 2.0)))
            out.append(wbox((x0 - h, y_lo, SLOT_Z_BOTTOM), (x0 + h, y_hi, z0)))
    return out

_RELIEF = {}
def relief_sweep(grow=0.4, step=2.5):
    """J01 −5..+30° 扫掠（连续膨胀 grow），壳/H01 减它 → 张嘴全程 0 碰。老嘴轴处的填料、新毂盘都在里面。（同进程缓存）"""
    key = (grow, step)
    if key in _RELIEF: return _RELIEF[key].copy()
    from .lib import minkowski_box
    J = build_jaw()
    Jg = minkowski_box(J, (-grow,) * 3, (grow,) * 3)
    lo, hi = JAW_RELIEF_RANGE
    _RELIEF[key] = union(*[placed(Jg, jaw_R(a)) for a in np.arange(lo, hi + 1e-9, step)])
    return _RELIEF[key].copy()

# ── H01：嘴舵机托架（顶板 + 两端墙）与惰轮上半拱
H01_CRADLE_CLR = 0.3
def _servo_local_box(x0, x1, y0, y1, z0, z1):
    """舵机局部系盒 → 世界（跟着舵机转 ψ）"""
    return placed(s288.bx((x1 - x0, y1 - y0, z1 - z0), ((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2)), servo_R())

def h01_adds():
    """嘴舵机托架：顶板（压舵机上面 = 局部 −y 面，外加 1.6 厚）+ 顶端端墙（局部 +z 端外 1.6）+ 插座端端墙只做两条竖筋（不挡插座端出线）；
    全部连到前立板（x 47..52）与老口袋端墙区。舵机局部：厚 x −13..+10（背面 → 法兰面），宽 y ±10，长 z −24.5..+9.5。
    顶板只盖 x ∈ [−9, 9]（法兰面 +10 以外是头壳；背面 −13 往外是那摞件扫掠，留空）。"""
    c = H01_CRADLE_CLR; t = 1.6
    top = _servo_local_box(-9.0, 9.0, -10.0 - c - t, -10.0 - c, -24.5 - c - t, 9.5 + c + t)
    end_top = _servo_local_box(-9.0, 9.0, -10.0 - c - t, 10.0 + c - 3.0, 9.5 + c, 9.5 + c + t)        # 顶端端墙（留下半截给 H03 托台）
    ribs = [_servo_local_box(xa, xb, -10.0 - c - t, -2.0, -24.5 - c - t, -24.5 - c) for (xa, xb) in ((-9.0, -6.0), (6.0, 9.0))]   # 插座端两条竖筋
    return [top, end_top, *ribs, h01_seat_arch_adds()]

def h01_seat_arch_adds():
    """惰轮上半拱（座环上半）+ 往 +y 连回 H01 老惰轮拱（原版 motor_support −y 端，(26.1, −39.5..−35.5, 238.6)，外 r≈9.9、顶 248.8）的连接块：
    x ±7（在 H05 插入槽 16 宽以内）× y −45.3..−37（不进轴承的 y 段 −49.05..−45.05）× z 246.8..252.5（盖住老拱顶 248.8）。"""
    x0, z0 = JAW_AXIS_P[0], JAW_AXIS_P[2]
    arch = seat_ring("upper")
    b = JAW_BRG
    web = diff(wbox((x0 - 7.0, b["y1"] - 0.25, 246.8), (x0 + 7.0, -37.0, 252.5)),
               cyl(b["seat_d"] + 0.2, 1.0, (x0, b["y1"] - 0.25 + 0.5, z0), axis="y"),        # 进座环那 0.25 的搭接只在座孔外
               journal_clear_cut())
    return union(arch, web)

def h01_cuts():
    return [servo_env_world(), seat_bore_cut(), journal_clear_cut(), *[conn_zone_world(s, "A") for s in (1, -1)]]

# ── H03：托台（从下托嘴舵机）+ 惰轮下半环
def h03_adds():
    """H03 托台（从下托嘴舵机：局部 +y 面外 0.3 起 2.0 厚，局部 x −9..9、全长）+ 往下到地板的两道立墙 + 惰轮下半环 + 下半环往下连到 H03 侧壁的腹板"""
    c = H01_CRADLE_CLR; t = 2.0
    cradle = _servo_local_box(-9.0, 9.0, 10.0 + c, 10.0 + c + t, -24.5 - c - t, 9.5 + c + t)
    # 立墙：舵机局部 x = −6..−4 / 4..6（世界 y 28.75..30.75 / 36.75..38.75），从托台往下 14（到地板以下，由壳外表面截）
    walls = [_servo_local_box(xa, xb, 10.0 + c + t - 0.5, 10.0 + c + t + 16.0, -24.5 - c - t + 3.0, 9.5 + c + t - 3.0) for (xa, xb) in ((-6.0, -4.0), (4.0, 6.0))]
    b = JAW_BRG; x0, z0 = JAW_AXIS_P[0], JAW_AXIS_P[2]
    web = wbox((x0 - 7.0, b["y0"], 226.0), (x0 + 7.0, b["y1"], z0 - JAW_SEAT_OD / 2 + 1.0))
    ring = seat_ring("lower")
    return [cradle, *walls, ring, diff(web, cyl(JAW_SEAT_OD - 0.05, 40.0, (x0, -45.0, z0), axis="y"))]

def h03_cuts():
    return [servo_env_world(), seat_bore_cut(), journal_clear_cut(), *[conn_zone_world(s, "A") for s in (1, -1)]]

def wire_path_boxes():
    """两个插座出线（hr39c：两口都用，线先沿舵机背面朝 +x（插座端）走，出端面 4 mm 后再分头去 S1 / 头横滚 / 头偏航；路线见 wiring_head.py）"""
    from .lib import conn_zone as _cz
    return [placed(_cz(s, "A"), servo_R()) for s in (1, -1)]

PARTS = {"J01": ("J01_jaw.stl", "jaw", build_jaw),    # 件号 → (STL, placed 名, build)
         "J02": ("J02_jaw_adapter.stl", "jaw_adapter", build_jaw_adapter),     # hr41
         "J03": ("J03_jaw_journal.stl", "jaw_journal", build_jaw_journal)}     # hr41
JAW_MOVERS = ("jaw", "jaw_adapter", "jaw_journal")    # hr41：随嘴关节转的打印件（checks.py 嘴扫掠、Gate 数据用）
