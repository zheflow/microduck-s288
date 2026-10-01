"""腿（左）L01–L07：髋偏航-横滚件 / 髋件 / 大腿 / 小腿 / 踝主脚+鞋底 / 踝后臂。右腿由 build.py 镜像。"""
import math, numpy as np
from .s288 import S, cyl, bx, union, diff, inter, placed
from . import hip_pitch_bearing_rebuild as HPR
from .bearing_rebuild import (HR, hip_roll_seat_local, hip_roll_seat_cut_local, hip_roll_module_local,
                              hip_roll_driven_local, hip_roll_sleeve_local, hip_roll_old_clearance_local)
from .lib import (P, RINGS, BX0, BX1, BZ0, BZ1, BX0_BAT, ANK_J, ANK_COL, ANK_SWEEP_LO, ANK_SWEEP_HI, ANK_SWEEP_N_LEG, ANK_SWEEP_N_SERVO, conn_hump_sweep,
                  KNEE_CUT_HI, KNEE_HEEL_LO, KNEE_HEEL_HI, KNEE_HEEL_N, KNEE_L03_LO, KNEE_L03_HI, KNEE_L03_N, ANK_X0, ANK_X1b, ANK_X1, X_ENV_BOSS, x_bshell_out, XR,
                  memo, orig, sfw, drv_from, drv_self, pt, servo_env, driven, keep_main, servo_slide, bearing_driven_clearance,
                  clean_print_topology, fill_cyl, driven_patch, horn_cbore, flange_relief, horn_cut, silhouette, back_shell,
                  ring_boss, sweep_of, mnt_cut2, mirror_y, conn_cut, back_pad, seat_ring_clearance, seat_ring_zone, wbox, BAT_Y, region_cut, minkowski_box)

# ── 2026-09-17 真实姿态缺陷区域刀（lib.region_cut；数据 duckstructure/data/regions_2026-09-17/，来自 r4 的 policy_pose_compare 10 个缺陷对）──
# 双腿联动/头缩姿态，单关节 sweep 表达不了；每对只切**一侧**：切运动件/远端件，不切原版冻结件与舵机。右件区域镜像到左件，左右同刀。
# L02：L02×L02（roulade 7 姿态，两侧各 5–6 mm³）、L02×原版 jaw（4，14）、L02×T01（1+2，3+2）；L04：L04 踝端外侧 vs L02（6，53）、L04×原版上头壳（1，2）；L07：后臂顶边 vs L02（3，8.5）
# was_until_2026_09_28_hr50: HIP_REGION_CUTS = (("hip__pair_hip_l_x_hip_l_2.stl", "hip_l", False), ("hip_R__pair_hip_l_x_hip_l_2.stl", "hip_l_2", True),
# was_until_2026_09_28_hr50:                    ("hip_R__pair_hip_l_2_x_jaw_soft.stl", "hip_l_2", True), ("hip_R__pair_hip_l_2_x_trunk_base.stl", "hip_l_2", True),
# was_until_2026_09_28_hr50:                    ("hip__pair_hip_l_x_trunk_base.stl", "hip_l", False))
# was_until_2026_09_28_hr50: LOWER_LEG_REGION_CUTS = (("lower_leg_R__pair_hip_l_2_x_leg_2.stl", "leg_2", True), ("lower_leg_R__pair_jaw_soft_x_leg_2.stl", "leg_2", True))
# was_until_2026_09_28_hr50: REAR_ARM_REGION_CUTS = (("ankle_rear_arm_R__pair_ankle_right_x_hip_l_2.stl", "ankle_right", True),)
# hr50（2026-09-28 主设计逐刀决定，依据 hr50_work/poses/region_recheck.md：当前几何复算 + 算法会话 C 中档 ≤2.29 mm³ 且 ≤0.77 mm）——三组区域刀全部撤掉：
#   L02#7 / #12（右 / 左 L02 顶角 × 躯干底边）：轻碰，交集 ≤1.0 / 0.9 mm³、穿深 ≤0.45 / 0.53，在中档内，原版同姿态也撞；
#   L02#9 / #10（两个 L02 翻滚动作髋偏航到头互擦）：穿深 ≤0.53，原版 5/7 也撞，且这两刀咬 L02 沉窝（用户规矩：沉窝不许咬）；
#   L02#8（新头 × 右 L02）：交集最大 504 mm³、刀只削得到 17.5，带刀后 48 姿态照撞 → 刀没用，交关节限位 / 训练；
#   L04#12（新头 × 小腿）：280 mm³ 深 6.2、刀只削 5.8，没用且咬螺丝孔；L04#9（深蹲极限小腿踝端 × L02）：带刀后 4 姿态仍 ≤103 mm³，交限位；
#   L07#7 / L05#11（后臂顶边 × 右 L02）：带刀后 11 姿态照撞，没用。区域刀用 minkowski_box 外扩 0.4，本身就是碎边来源（B 表 L02/L04 碎边×7）。
HIP_REGION_CUTS = ()
LOWER_LEG_REGION_CUTS = ()
REAR_ARM_REGION_CUTS = ()

# ───────────────────────────── 腿（左）——全部世界坐标 ─────────────────────────────
HIP_ROLL_DISC_D = 20.0   # 09-17：髋横滚从动盘 Ø（旧 26，随 6704 一起取消；r 10 → 零位离中线 7.5，与原版 hip_l 的 8.0 同级；试算 0/77 姿态相交，见 lib.RINGS 注释）
L01_SEAT = 19.2      # L01 偏航盘 6 颗 M2 的坐面（髋偏航舵机局部 x）= 原版杯壁外表面 = 横滚舵机顶端(世界 z 112.0)+CLR → 131.5−112.3；只与 top/CLR/舵机位姿有关，不随 T/flange_h 变
# 09-14 复审 BLOCKER：旧坐面 = 法兰面 13.0 + 5.35 = 18.35，头 Ø4×1.6 顶到 19.95 > 杯壁外面 19.2 = 横滚舵机顶端(19.5)−CLR，
# 12 颗头全部压进横滚舵机顶端 0.45（每颗 ∩ 5.646 mm³；旧 12.85 法兰面时已压 0.30）。旧检查只查头 vs L01 自身，没查头 vs 横滚舵机。
# 改：叠厚 5.35 → 4.0（沉窝深 0.85 → 2.2，坐面 17.0 = 毂 13.0..17.5 内、6702 内圈区 13..17 之外），头顶 18.6 ≤ 19.2（离横滚舵机顶 0.9），
# F02 改 M2×6（咬 2.0；长度规则 6 ≤ 4.0+3.0−0.3 ✓）。沉窝 Ø4.4 外缘 r 7.45 只在 x 17.0..17.5 破毂外圆 Ø14.85 0.025（座环让位带内，无功能）。
L01_STACK = 4.0      # 6 颗 M2×6 的叠厚（F02，mechanical_audit hip_yaw L01 screw seats 按它判）：沉窝深 = L01_SEAT − (法兰面 + 4.0) = 2.2（旧 5.35/M2×8/沉 0.85）
L01_SCREW_L = 6.0    # F02 螺丝长（mechanical_audit 咬入 = L01_SCREW_L − 叠厚 = 2.0；旧 8）
ANK_SIL_R = 13.0     # L04 踝端背板的投影轮廓圆半径（silhouette extra_circles，Ø26）
@memo
def build_yaw2roll():
    """髋偏航-横滚件（照原版 yaw2roll.stl 重雕）。原版是个小杯子：一头是偏航从动盘，一头包住横滚舵机的顶半段。
    原版只用了横滚舵机背面**上排**那 2 个孔（实测 Ø4.84 沉孔在世界 (-8.2, 9.5/25.5, 110)，正是 (±8, z=+7.5)），
    下排孔在原版件之外，我们照搬这个 2 螺丝方案。
    偏航端（RINGS["left_hip_yaw"]）：法兰侧 6702 副支撑（原版此处本来就有一颗 22×16×4（舵盘面平面，受力报告 §4 判为推力式夹持），S288 法兰配不了那种夹法，我们改法兰侧径向 6702）（理由见 lib.P["brg_yaw"] / trunk.build_trunk）：
    本件出 Ø14.85 毂（x 13.0..17.5）套 6702 内圈，Ø26 盘退到 17.5..20.5（被横滚舵机包络削到 L01_SEAT=19.2，和以前一样），
    6 颗 M2×6 叠厚 L01_STACK=4.0（19.2−13.0=6.2 沉 2.2；09-14 复审前 5.35/M2×8/沉 0.85，头顶 19.95 穿横滚舵机顶端 19.5——见 L01_STACK 注释）、咬 2.0；
    原版杯顶的 Ø16 盘和两片 XL330 耳朵在座环带（z 114.0..118.7, r 7.4..12.6）被 seat_ring_clearance 切掉。"""
    Rp = drv_from("yaw2roll")                          # 躯干里的髋偏航舵机，法兰朝下
    Rs = sfw("yaw2roll", 0)                            # 髋横滚舵机，法兰朝 +x
    O = orig("yaw2roll", "yaw2roll")
    dr, dh, xe, xi = driven(ring=RINGS["left_hip_yaw"], d=26.0, cbore=True, brg=P["brg_yaw"])
    cs = pt(Rs, 0)
    sil = silhouette(O, 1, pad=0.5, span=(4.0, 32.0), extra_circles=[(cs[0], cs[2], 14.0)])
    add = [placed(dr, Rp), inter(back_shell(Rs, 3.0), sil)]
    if not RINGS["left_hip_yaw"]: add.append(driven_patch(Rp, 26.0))     # 有毂时毂就是贴合面，不再补 1.65 那片
    xf = S["T"] / 2 + S["flange_h"]
    cuts = [horn_cut(Rp), horn_cbore(Rp, L01_SEAT, L01_SEAT - (xf + L01_STACK)),   # 杯壁外表面 19.2（横滚舵机顶端 112.0+CLR，不随 T 变）沉到叠厚 4.0：沉 2.2（09-14；旧 0.85→头穿横滚舵机顶），M2×6 咬 2.0
            servo_env(Rs), servo_env(Rp), flange_relief(Rp),
            mnt_cut2(Rs, pairs=(0,), cb_x0=-14.5, cb_d=x_bshell_out(3.0) - (-14.5))]   # 只用上排 2 孔；沉到背板外表面 -13.3（沉 1.2；旧 1.25 → -13.25）→ 叠厚 3.3，M2×6 咬 2.7
    # hr11：髋偏航舵盘 6 颗 M2 的起子通道（Ø4.2，从叠厚顶 xf+L01_STACK 沿 +x 30；mechanical_audit hip_yaw L01 head/tool envelope 同款刀）——
    #        hr08 整体座并入后 60°/120° 两条通道各被擦进 1.19 mm³（世界 z 111..114.5）
    cuts.append(placed(union(*[cyl(4.2, 30.0, (xf + L01_STACK + 15.0, S["horn_r"] * math.cos(a), S["horn_r"] * math.sin(a)), axis="x")
                               for a in np.linspace(0, 2 * math.pi, S["horn_n"], endpoint=False)]), Rp))
    if RINGS["left_hip_roll"]:
        add.append(placed(hip_roll_seat_local(), Rs))          # hr08：6703 整体座 + 顶桥/外侧桥并入本体
        cuts.append(placed(hip_roll_seat_cut_local(), Rs))     # 座孔/唇孔/开顶在并集之后切
        # 09-20 背插模型（lib.CONN_MODE "free"）：L01 背板到不了插座区，两个口都空着；这把刀只削掉板下角伸进
        # 线弯区的 0.3 mm³ + 侧出线区 2.9 mm³（hr15 探针），两侧都出。旧 hr06 "−y 走廊被挡 64 mm³ / 只开总线侧"是侧插模型的事，作废。
        cuts.append(conn_cut(Rs, sides=(1, -1), mode="pocket"))
    # hr10：L02 上 F06 六颗螺丝头（坐面 horn_seat 起 1.6 高，露出盘面 0.6）一起做 L01 的让位扫掠——hr09 运动审计 yaw2roll×F06_heads 72/278 姿态 6.2 mm³
    f06_heads = placed(union(*[cyl(P["m2_head_d"], P["m2_head_h"], (HPR.HP["horn_seat"] + P["m2_head_h"] / 2, S["horn_r"] * math.cos(a), S["horn_r"] * math.sin(a)), axis="x")
                              for a in np.linspace(0, 2 * math.pi, S["horn_n"], endpoint=False)]), drv_self("upper_leg_left"))
    # was_until_2026_09_28_hr50: cuts.append(sweep_of(union(build_hip(), f06_heads), "hip_l", -24, 24, 9, grow=0.4, mink=True))
    # hr50：6°/步在杯顶外沿留锯齿（clean_check 碎边 (12.9, 29.0, 115.2)）→ 33 步（1.5°/步），范围不变
    cuts.append(sweep_of(union(build_hip(), f06_heads), "hip_l", -24, 24, 33, grow=0.4, mink=True))   # 09-17：无座时也让髋件扫掠（L01 法兰侧本无料，刀为保险）
    # 对侧髋件也必须参与；右髋偏航真实范围 -30..+25，不能照抄左侧。
    cuts.append(sweep_of(mirror_y(build_hip()), "bearing_roll", -30, 25, 45, grow=0.4))
    if RINGS["left_hip_yaw"]:
        cuts.append(seat_ring_clearance(Rp, P["brg_yaw"]))            # 让开 T01 座环(Ø24.6)+0.3 与 6702 本体；毂 Ø14.85 保留
        # 09-12 复审：上面这把环刀（r 12.6 → 世界 x≤18.6）把髋横滚 6704 座的背挡肩顶弧（世界 x 18.6..座孔起点 = 舵机心 6 + xf−0.1）
        # 切成薄片（旧 18.6..18.75 = 0.15，<1.2 规则；09-13 xf 13.0 → 18.6..18.9 = 0.3），这里把那一段挡肩切干净
        # （其余弧段仍挡得住外圈：6704 沿 −x 移 0.2 仍与 L01 交 4.4 mm³）。世界坐标，x1 从 xf 推；右件由 mirror_y 跟随。
        # Restored 6703 has an explicitly open crown; the old 6704 shoulder
        # trimming box must not be reused at its different axial station.
    fills = [fill_cyl((x, y, 113.5), 5.2, "z", 8.0) for (x, y) in ((6.0, 23.5), (12.0, 17.5), (6.0, 11.5), (0.0, 17.5))]  # 原版偏航舵盘 4 孔+沉孔
    # hr50（2026-09-28）：原版杯顶两侧的 Ø2.05 残孔（沿 x，世界 (13.6..17.3, 8.5/26.5, 115.0)），没有螺丝用；被偏航座环让位刀（r 12.6）和 L02 扫掠
    #   咬成孔口不规则（clean_check holes use=none：缺 24–76%、方位漂移 26.5°）→ 按"旧残孔填实"处理，之后各刀照切。
    fills += [fill_cyl((15.15, y, 115.0), 2.2, "x", 4.3) for y in (8.5, 26.5)]     # x 13.0..17.3 = 原孔长（耳前面 17.3 不外凸）
    # hr50：两把清边刀（都不碰螺丝孔/沉窝坐面/座孔）：
    #   ① 横滚舵机厚段背后那条背板尾巴：外轮廓是 silhouette 的 r14 圆，贴着厚段背面 x_l −13.3 越收越薄，z_l −4.2..−2.2 只剩 0.05–0.6（clean_check
    #      薄膜 9.9 / 3.6 mm² + 两个尖刺）。尾巴只是原版外形，不夹舵机（舵机靠上排 2 颗）；局部 z_l −1.0 以下、x_l < −13.3 一刀平切齐（切口处墙厚 0.66），
    #      +y 侧到 y_l 8.0 为止（再往外是 hr08 back_link 搭接块，不动）。
    #   ② 偏航 6702 毂顶 0.5 mm（局部 x 17.0..17.5，6702 内圈端面 17.0 以外）：6 个沉窝 Ø4.4@r5.25 外缘 7.45 刚好越过毂 Ø14.85（7.425）0.025，
    #      在座环让位刀里留下 8 片 0.1 薄皮（clean_check 薄膜/刀片 ×10，z 114.0..114.5）→ 这一段毂收到 r 7.2，沉窝坐面 x 17.0 不动。
    _tail = placed(bx((16.0 - 13.0, 18.5, 3.6), ((-16.0 - 13.0) / 2, (-10.5 + 8.0) / 2, (-4.6 - 1.0) / 2)), Rs)   # x_l 到 −13.0：连包络斜面下端 0.2 的小楔一起切
    #   ③ 偏航座环让位刀（r 12.6，z 114.0..118.7）从杯侧面 y 6.0 / 29.0 穿出，四个穿出口各留一个 24° 楔尖（clean_check 刀片 6.3 / 5.2 / 0.9 mm²，
    #      +y 前耳尖还有碎边）：前耳是原版 XL330 耳朵残料、后墙尖是杯壁残料，都不夹东西；楔尖切到墙厚 ≥0.8 的位置（前耳 x 13.8 处 1.6、后墙 x −0.65 处 0.8），
    #      切口是竖直平面，z 从座环让位刀底面 114.0 起（顶桥 / 外侧桥在 y 8..27、x ≥ 12.3 或 y 24.5..29.5、z ≤ 114.6、x ≥ 12.8，都不碰）。
    _tips = [wbox((10.5, y0, 114.0), (13.8, y1, 118.8)) for (y0, y1) in ((5.0, 7.6), (27.4, 30.0))]
    _tips += [wbox((-0.65, y0, 114.0), (1.6, y1, 118.8)) for (y0, y1) in ((5.0, 7.0), (28.0, 30.0))]
    #   ④ 杯内侧墙底边（世界 x −8.4..−5.1、y 6.8..7.2、z 103.0..103.6）：斜底面朝包络侧面 y 7.2 收成楔尖（clean_check 刀片 3.9 mm²）→ z 103.65 以下、y ≥ 6.8 平切掉楔尖。
    _tips += [wbox((-9.0, 6.8, 101.0), (-4.5, 7.6, 103.65))]
    _hubtop = placed(diff(cyl(15.3, 0.5, (17.25, 0, 0), axis="x", sections=128), cyl(14.4, 0.7, (17.25, 0, 0), axis="x", sections=128)), Rp)
    # was_until_2026_09_28_hr50: return clean_print_topology(keep_main(diff(union(O, *add, *fills), *cuts, servo_slide(Rs)), "yaw2roll"))
    # hr50：servo_slide(Rs) 按算法会话 D 表是冗余刀（撤掉后件完全不变，那块料 99.9% 同时是 servo_env 的缺口）→ 删掉。
    return clean_print_topology(keep_main(diff(union(O, *add, *fills), *cuts, _tail, _hubtop, *_tips), "yaw2roll"))

@memo
def build_hip():
    """髋件（照原版 hip_l.stl 重雕）。不装舵机，是纯双从动盘：一面接髋横滚、一面接髋俯仰。
    原版两面都是 XL330 的 4 孔 r=6 图案，先填掉再打我们的 6 孔（分度圆 2×horn_r）。"""
    Rr = drv_from("hip_l")                             # 髋横滚舵机（在 L01 里），法兰朝 +x
    Rp = drv_self("upper_leg_left")                    # 髋俯仰舵机（在大腿里），法兰朝 -y
    O = orig("hip_l", "hip_l")
    d1, h1, xe1, xi1 = driven(ring=False, d=HIP_ROLL_DISC_D)
    if RINGS["left_hip_roll"]: d1 = hip_roll_driven_local()
    d2, h2, xe2, xi2 = driven(ring=RINGS["left_hip_pitch"], d=19.0)
    if RINGS["left_hip_pitch"]: d2 = HPR.pitch_driven_local()
    add = [placed(d1, Rr), placed(d2, Rp)]
    if not RINGS["left_hip_pitch"]: add.append(driven_patch(Rp,19.0))
    if not RINGS['left_hip_roll']: add.append(driven_patch(Rr,HIP_ROLL_DISC_D))
    fills = [fill_cyl((24.0, y, z), 5.2, "x", 9.0) for (y, z) in ((17.5, 108.5), (17.5, 96.5), (11.5, 102.5), (23.5, 102.5))]  # 原版横滚侧 4 孔
    # 原版俯仰侧 4 孔：填柱 y 34.0..40.3（09-12 第二批 I5）。原 36.3/8.0 = 32.3..40.3 比原版板内面 y=34.0 多凸 1.7（离横滚轴 r 14.8），
    # L02 沿横滚轴 +x 套上/抽离 L01 时撞 L01 6704 座凸台外缘 r 15.5（4.55 mm³@t8.5 / 4.29@t22）；退到板内面后 r 16.5，抽出全程留 1.0。
    # 削凸台不可行：座孔 13.575 + 壁 1.2 = 14.775 对 r 14.8 只剩 0.025。俯仰盘 6 颗坐面在 y 34.65（凸出段本来不参与）。
    fills += [fill_cyl((x, 37.15, z), 5.2, "y", 6.3) for (x, z) in ((10.0, 102.5), (-2.0, 102.5), (4.0, 108.5), (4.0, 96.5))]
    # hr50（2026-09-28）：横滚侧 6 颗 M2 沉窝（L02-F06，保护区）从压板头座面（世界 x 22.3）一直开到毛坯外（x ≈ 29），外段穿过原版 hip_l 的减面毛坯：
    #   相邻沉窝之间只剩十字形薄筋，孔口残缺 15–75%、方位漂移 42–45°（clean_check holes ×5）+ 薄膜 6.4 ×2 + 碎边 ×4。
    #   试过补 Ø17 实心盘（x 21..29）：沉窝干净了，但盘把前内侧撑大，5601 姿态里右 L02×左 L02（roulade 第 57 步）从 0 变 15.9 mm³、
    #   头部件接触也加大 → 改成下面 cuts 里的平切：x ≥ 25.5、y ≤ 23.5 的毛坯整块切掉，沉窝 x 22.3..25.5 段在完整料里（孔壁外 ≥2），孔口是平面。
    # 两面法兰孔的坐面都是原版毛坯的台阶（09-11 16 向射线：横滚面 7.50..10.15、俯仰面 8.15..9.85），螺丝头坐最高点、
    # 1/3 足印悬空 → 偏心预紧。Ø4.2 沉到统一叠厚 7.5（横滚面本来就有 4.5 轴承毂 + 3 板 = 7.5 名义），6+6 颗全用 M2×10 咬 2.5。
    XF = S["T"] / 2 + S["flange_h"]
    blank = union(O, *add, *fills)
    # The original blank did not move with the servo. Extend the tool all
    # the way outside that blank while preserving the under-head plane.
    cbore_end = float(((blank.vertices-Rr[:3,3]) @ Rr[:3,0]).max()) + 1.0
    cuts = [horn_cut(Rr), horn_cut(Rp), flange_relief(Rr), flange_relief(Rp),
            horn_cbore(Rr, cbore_end, cbore_end-(XF+HR['screw_stack'])) if RINGS['left_hip_roll'] else horn_cbore(Rr,XF+10.5,3.0),
            horn_cbore(Rp, XF + 10.5, 3.0),
            sweep_of(servo_env(Rr), "hip_l", -24, 24, 9, grow=0.3),                    # 横滚舵机相对髋件的反向扫掠
            sweep_of(servo_env(Rp), "upper_leg_left", -62, 62, 11, grow=0.3)]          # 髋俯仰舵机相对髋件的反向扫掠
    if RINGS["left_hip_yaw"]:
        # 髋偏航 T01 座环（Ø24.6，z 114.65..119.15）在髋件绕横滚轴 ±22° 的扫掠里：髋件顶板外后角在 +22° 时升到 z 116.3、
        # 离偏航轴 r=10.1（09-12 实测 8.7 mm³@22°、18.7@24°）—— 任何法兰侧径向轴承本体都会被它擦到，与轴承大小无关。
        # 原版这里没有座环所以不用让；我们加了座环就得让：和 L01 让髋件扫掠用同一种刀（反向扫掠 ±24°，grow 0.4），切掉 29.8 mm³
        # （世界 x 0.76..17、y 32.3..38.5、z 108..112，顶板外后角）。±22° 内对座环最小间隙 0.61（0.5° 网格）。
        cuts.append(sweep_of(seat_ring_zone(drv_from("yaw2roll"), P["brg_yaw"]), "hip_l", -24, 24, 25, grow=0.4))
    if RINGS["left_hip_roll"]:
        cuts.append(placed(hip_roll_old_clearance_local(),Rr))
        # hr08：固定在 L01 上的镗孔后整体座（含桥）绕横滚轴反向扫掠；钢圈不进刀（内圈端面必须贴 L02 压面）
        fixed=placed(hip_roll_module_local(steel=False),Rr)
        cuts.append(sweep_of(fixed,"hip_l",-24,24,49,grow=.4,mink=True))
    if RINGS["left_hip_pitch"]:
        cuts.append(placed(HPR.pitch_old_journal_clearance_local(),Rp))
        # hr07：固定在 L03 上的整体座+腹板绕俯仰轴反向扫掠（左目标 [−90,66.9] 各留 2°），钢圈不进刀（内圈端面必须贴 L02 压面）。
        fixed=placed(HPR.pitch_module_local(steel=False),Rp)
        cuts.append(sweep_of(fixed,"upper_leg_left",-92.,69.,162,grow=.4,mink=True))
    cuts += [region_cut(f, b, mirror=mr) for f, b, mr in HIP_REGION_CUTS]          # 09-17 真实姿态区域刀（hr50 起为空）
    # hr50：俯仰侧 +y 端前上角（世界 x 24..27.4、y 36.1..38、z 106.5..107.5）是原版 hip_l 减面网格留下的碎边（clean_check 碎边 26 条短锐边），
    #   一刀平切齐：x ≥ 24、y ≥ 35.5 的部分顶面压到 z 106.4（最多削 1.4；件号 L02 刻在 x 18.8..21.5，不碰；俯仰侧沉窝在 x ≤ 11.5）。
    cuts.append(wbox((24.0, 35.5, 106.4), (31.0, 39.5, 112.0)))
    # hr50：横滚侧下前角（世界 x 25.3..25.5、y 14.7..18.4、z 93.8..94.4）原版减面网格的圆角碎边（clean_check 碎边 33–51 条）→ x ≥ 25.0、z ≤ 94.3 一刀平切
    #   （横滚侧沉窝最低点 z 95.75，离切口 1.45；上面那块 Ø17 补料盘底 z 94.0，被切成 94.3 平底）。
    cuts.append(wbox((25.0, 12.0, 90.0), (32.0, 23.0, 94.3)))
    cuts.append(wbox((25.5, 0.0, 85.0), (40.0, 23.5, 120.0)))      # 横滚侧沉窝口平切（见 fills 处说明）；y 23.5 以上是去俯仰盘的臂，不动
    return clean_print_topology(keep_main(diff(blank, *cuts), "hip"))
def hip_roll_bearing():
    """髋横滚 6703（23×17×4）的实心让位刀；位置与 bearing_specs 一致。
    实物内圈由独立 L10 轴套与 L02 压盘夹持；这里仅供远端踝件让位。"""
    if not RINGS["left_hip_roll"]:
        return None                                    # 09-17：髋横滚不再有法兰侧轴承
    return placed(union(hip_roll_module_local(steel=True),hip_roll_sleeve_local()),sfw('yaw2roll',0))

def hip_pitch_bearing():
    """髋俯仰站整套（座+腹板+钢圈+轴套），给踝跟 / 膝的远端让位刀用。"""
    if not RINGS["left_hip_pitch"]: return None
    return placed(union(HPR.pitch_module_local(steel=True),HPR.pitch_sleeve_local()),
                  drv_self("upper_leg_left"))

@memo
def build_upper_leg():
    """大腿成品 = _build_upper_leg_base() 再让小腿子树（L04 + 踝舵机）绕膝轴扫过关节 KNEE_L03_LO..HI（60..90）。
    2026-09-16 行程恢复：膝 ≥67.5° 起 L04 踝舵机笼壁（距舵机本体 0.3）与踝舵机本体撞大腿；关节 ≤62 那段仍由 build_lower_leg 里的
    反向膝刀削 L04，>62 这段改削 L03（L04 那处笼壁只有 3 mm，再削剩 0.8；L03 侧无特征，见扫掠报告 §2.2）。
    小腿是用 base 版大腿做的刀切出来的，base ⊇ 成品，所以两边不会互相漏。"""
    # was_until_2026_09_28_hr50_r2:     ll = build_lower_leg()
    ll = _build_lower_leg_core()                       # hr50_r2：膝刀仍拿内核件扫（带膝锯齿、料只多不少）→ L03 这把刀与改前逐位相同；导出 L04 另削锯齿（见 build_lower_leg）
    Ra = sfw("leg", 0)                                 # 踝舵机（装在小腿里，随膝一起转）
    mover = union(ll, servo_env(Ra))
    cut = sweep_of(mover, "leg", KNEE_L03_LO, KNEE_L03_HI, KNEE_L03_N, grow=0.4, mink=True)
    # was_until_2026_09_28_hr50_r2:     return clean_print_topology(keep_main(diff(_build_upper_leg_base(), cut), "upper_leg"))   # hr07：整体座并入后导出有退化面/非流形边，规范拓扑
    # hr50_r2（2026-09-28）：膝舵机 PH2.0 窗（sgn −1，世界 x ≤ −21.277、z ≤ 94.9）右上角 clean_check 碎边 9 条 (−21.23, 69.23, 94.68) = 原版 upper_leg_left
    #   一根锥形小销（轴沿 y 过 (−21.0, *, 94.47)、3° 锥，y 60.2 → 69.5 从 Ø0 长到 Ø1.0，原版给 XL330 定位用）被膝窗 / 膝滑槽（x ≤ −21.477）和
    #   髋俯仰滑槽（x ≥ −20.8）夹剩的一根 0.5 宽 × 0.9 高悬空细条（y 66.5 接块、y ≥ 69.5 并进实料）。它不是孔，是残销 → 切掉（不填）：
    #   两块盒刀只占两条滑槽 / 窗边之间那条缝（x 面与膝窗 −21.277、膝滑槽 −21.477、髋俯仰滑槽 −20.8 共面，y 止于实料面 69.5，z 93.8..95.2 上下本来就空），
    #   实算多切 1.22 mm³；离髋俯仰下排 M2 孔（L03-F03#0.1 Ø2.4 @ x −18.5、z 94.5，孔壁 x −19.7）1.1 ≥ 1.0。
    #   只切成品 L03（_build_upper_leg_base 不动 → L04 膝刀、L05/L07 跟刀都不变）；新让位 = 旧 ∪ 盒刀 ⊇ 旧让位。
    xk, xh = pt(sfw("upper_leg_left", 0), 0)[0], pt(drv_self("upper_leg_left"), 0)[0] - 24.8     # 膝舵机心 x −31.777；髋俯仰滑槽 −x 壁 −20.8（局部 z −24.5−0.3）
    pin = union(wbox((xk + 10.5, 66.0, 93.8), (xh, 69.5, 95.2)), wbox((xk + 10.3, 60.0, 93.8), (xh, 67.8, 95.2)))
    return clean_print_topology(keep_main(diff(_build_upper_leg_base(), cut, pin), "upper_leg"))   # hr07：整体座并入后导出有退化面/非流形边，规范拓扑

@memo
def _build_upper_leg_base():
    """大腿（照原版 upper_leg_left + upper_leg_rigidity_plate 重雕）。
    原版是个水滴形薄壳，靠拧 XL330 侧面孔夹住两个舵机；S288 只有背面 4 角孔，
    所以给每个舵机补一层 3mm 等厚背板（back_shell），再用原版投影轮廓裁掉多出来的方角。"""
    Rh = drv_self("upper_leg_left")                    # 髋俯仰舵机 (4,55,102.5)，法兰朝 -y
    Rk = sfw("upper_leg_left", 0)                      # 膝舵机 (-31.8,55,80.5)，法兰朝 -y
    O = orig("upper_leg_left", "upper_leg_left")       # rigidity_plate 在原版里就是单独一片，不并进来
    ch, ck = pt(Rh, 0), pt(Rk, 0)
    sil = silhouette(O, 1, pad=0.5, span=(38.0, 74.0),  # 原版投影轮廓 + 两个舵机 Ø38 足印（背板要盖住 4 个角孔）
                     extra_circles=[(ch[0], ch[2], 19.0), (ck[0], ck[2], 19.0)])
    add = [inter(back_shell(Rh, 3.0), sil), inter(back_shell(Rk, 3.0), sil)]
    # 上排角孔"垫管座"（09-11 探针诊断 + 复审 M1）：膝舵机沿 −z 滑入时厚段扫过薄段背后 x −10.3..−13.3 整层，把上排 3 mm 背板削到只剩 0.19；
    # 髋俯仰舵机侧滑时副轴也啃掉 +y 孔坐面下半圈。薄段背后留不住坐面，也塞不进垫片卡（下面是厚段）。
    # 做法：垫块在背板外表面 -13.3 内 0.3 起（x_in=-13.0，与背板真·体积重叠）到 TUBE_OUT=-15.0（z 0..11 与厚段背板/斜坡搭上），Ø4.4 通孔；
    # 舵机装好后从外面放进 **M2 铜垫管 Ø4×5.0**（舵机薄段背 −10.0 → 管顶 −15.0，与垫块外表面齐平；旧 −9.95/−14.95），
    # **Ø4.5×0.5 平垫**压在管顶 + 垫块上，M2×8：叠厚 5.0 + 0.5 = 5.5，咬 2.5。
    # 剪力走垫管内孔支撑的螺杆，不走 2.3 mm 深的塑料自攻牙（纯隔空座会让牙受弯，复审 M1）。
    TUBE_OUT = -S["T"] / 2 - 5.0                                           # -15.0：垫管顶 = 薄段背面 − 管长 5.0 = 垫块外表面
    add += [inter(back_pad(R, (0.0, 11.0), dy=22.0, x_in=x_bshell_out(3.0) + 0.3, x_out=TUBE_OUT), sil) for R in (Rh, Rk)]
    # hr50（2026-09-28）：等厚背板 back_shell 只宽到舵机包络 |y_l| ≤ 10.3、高到 z_l 9.8，而 Ø4.4 垫管孔（y ±8, z 7.5）边到 |y_l| 10.2 / z_l 9.7、
    #   膝舵机滑入通道边到 |y_l| 10.2 —— 两处都只剩 0.1 的皮：
    #   膝（Rk）：通道两头各 0.1 皮（clean_check 薄膜 47 / 45 mm²，世界 x −21.5 / −42.0）= servo_slide 余量 0.2 与 servo_env 余量 0.3 之差（D 表）。
    #             滑入通道按 D 表不能撤（撤了背板要让 2.7，掰不进去）→ 两处 servo_slide 余量改 0.3（与包络一致），这片皮整片没了（见下面 servo_slide 调用）。
    #   髋俯仰（Rh）：两个垫管孔（L03-F06，保护区）顶上 0.1 皮 + 外侧 0.1 皮（薄膜 25 / 13 / 8 mm²，holes 缺 24–67%）→ 背板这一条补到
    #             z_l 10.7、|y_l| 11.2（孔壁外各 1.0），局部 x −13.3..−10.3、z_l 7.0..10.7；舵机沿局部 +y 侧滑时轮廓只到 z 9.7 / 副轴带 |z| ≤ 7.2，
    #             z 7.0..7.2 那 0.2 仍被滑入通道削掉。+8 那个孔下半圈对着副轴滑过的带（D 表 2.7 mm，掰不进去），那段开口是装入通道固有的，保留。
    add += [placed(bx((3.0, 22.4, 3.7), ((-13.3 - 10.3) / 2, 0.0, (7.0 + 10.7) / 2)), Rh)]
    add += [placed(bx((3.0, 1.6, 2.4), ((-13.3 - 10.3) / 2, sy * 10.4, (4.8 + 7.2) / 2)), Rh) for sy in (1, -1)]   # 垫管孔侧壁往下接到 z_l 4.8
    cuts = [servo_env(Rh), servo_env(Rk), mnt_cut2(Rh), mnt_cut2(Rk)]
    # PH2.0 插座（keepouts.yaml:KO01，09-20 背插模型，lib.CONN_MODE "window"）：插座在舵机背面两角、插头沿 −x 从背面插，
    # 盖住它的是我们自己的等厚背板 + 原版水滴壳外蒙皮（世界 y 70..72）。四个口全部在背板上开 5.5×10.2 通窗（刀到局部 x −18.5 = 穿透蒙皮），
    # 线从大腿外侧面走，插头可以装好舵机后再插 → 髋俯仰/膝各两口可用，串联。旧 CF1 侧窗（膝 −y 挡邻机）作废。右腿镜像跟随。
    cuts += [conn_cut(Rh, mode="window"), conn_cut(Rk, mode="window")]
    if RINGS["left_knee"]:
        rs, rc = ring_boss(O, Rk); add.append(rs); cuts.append(rc)
        # hr50：膝 6704 座的挡肩（局部 x 12.0..12.9、r 12..13.575）外侧是 ring_boss 凸包的锥面，右下方（世界 (−21.3, 42.2, 72.8)）锥面离挡肩面只剩 0.2
        #   （clean_check 薄膜 12.5 mm²）→ 挡肩这一段补成 Ø28.8 整环（外壁 ≥0.8）；舵机法兰 Ø14 在 r 7 以内、L04 在局部 x ≥ 13，都够不到。
        add.append(placed(cyl(28.8, 0.9, (12.45, 0, 0), axis="x"), Rk))
    # was_until_2026_09_28_hr50: cuts += [servo_slide(Rh), servo_slide(Rk)]          # hr07：髋俯仰舵机恢复世界 −z 侧滑装入（整体座在法兰顶面 13.0+0.2 之外）
    # hr50：余量 0.2 → 0.3（= servo_env），两刀边界重合，不再留 0.1 皮（D 表结论 2）。D 表结论 4 的"部分收刀、留 0.5 过盈掰进去"待用户定，这里没做。
    cuts += [servo_slide(Rh, clearance=0.3), servo_slide(Rk, clearance=0.3)]          # hr07：髋俯仰舵机恢复世界 −z 侧滑装入（整体座在法兰顶面 13.0+0.2 之外）
    # hr50：膝通道 −x 侧（世界 x −42.4..−42.1、y 66.6..68.0、z 74..76）原版大腿内侧弧形筋被通道切剩的月牙残条（0.3–0.8，clean_check 刀片 2.3 mm²，
    #   原来和 0.1 皮连在一起没单独报）→ 垫块（y ≥ 68）以下、通道边以外这一小条切掉；外壳壁在 x ≤ −43.5，不碰。
    cuts.append(wbox((-43.3, 65.2, 70.5), (-42.0, 68.0, 81.8)))
    # hr11：躯干里的髋偏航舵机（固定件）在 L03 坐标系里的运动 = 先绕偏航轴 ∓30°、再绕零位横滚轴 ∓26°（与躯干让位刀相反的次序）——
    #        hr10 真实姿态 servo__trunk_base_yaw2roll×upper_leg 0.10 mm³（左髋横滚 25.5° + 偏航 28.5°）
    # hr12：上面那把全组合扫掠刀把 6703 座壁 185°–225° 扇区切穿（restored_bearing_audit 壁 ≥1.1 FAIL）→ 撤掉。这 0.10 mm³ 的擦碰只出现在 1/5601 个姿态
    #        （左髋横滚 25.5° + 偏航 28.5°，超出 ±22° 设计域），位置在座壁外表面（局部 x −12.8..−12.3），任何让位都要把 1.275 的座壁削到 <1.1，与壁厚不变量冲突；
    #        记为已知残余（记录.md 16），躯干侧/钢圈侧的重叠由 trunk.py 的真实姿态区域刀负责。
    pass
    if RINGS["left_hip_pitch"]:
        cuts.append(placed(HPR.pitch_seat_cut_local(), Rh))   # 座孔/唇孔在并集之后切（防毛坯回填）
    # 上排：Ø tube_hole_d 通孔从外面一直打到舵机薄段背面（垫管从外面放进去，也清掉半圈原版斜面碎料），
    # 垫块外表面 TUBE_OUT=−15.0 以外 Ø5.5 让开平垫和螺丝头。下排仍是 3.3 背板，M2×6。
    cuts += [placed(union(*[cyl(P["tube_hole_d"], 40.0, (-20.0, sy * S["mnt_dx"], S["mnt_z"][0]), axis="x") for sy in (1, -1)]
                          + [cyl(5.5, 25.0, (TUBE_OUT - 12.5, sy * S["mnt_dx"], S["mnt_z"][0]), axis="x") for sy in (1, -1)]), R) for R in (Rh, Rk)]
    m = union(O, *add)
    if RINGS["left_hip_pitch"]:
        m = union(m, placed(HPR.pitch_seat_local(), Rh))     # hr07：6703 整体座 + 腹板并入本体
    return keep_main(diff(m, *cuts), "upper_leg")

@memo
# was_until_2026_09_28_hr50_r2: def build_lower_leg():
def _build_lower_leg_core():                           # hr50_r2：原 build_lower_leg 整体改名为内核件（内容一行没动），导出件见下面新的 build_lower_leg
    """小腿（照原版 leg.stl 重雕）。原版是一块雕花平板，靠螺丝拧 XL330 侧面 + 轴心阶梯孔套副轴定位。
    S288 只有背面 4 角孔，而且原版板在 S288 厚段背后只剩 0.65mm，所以补一层 3mm 等厚背板（back_shell）。"""
    Rk = drv_from("leg")          # 膝舵机在大腿里，法兰朝 -y；小腿是从动件
    Ra = sfw("leg", 0)            # 踝舵机装在小腿里，法兰朝 +y
    O = orig("leg", "leg")
    YM, YT = 36.5, 8.4            # 原版板 y 32.5..40.45 的中面与最大厚度（填孔用）
    dk, _, xe, xi = driven(ring=RINGS["left_knee"], d=26.0)      # holes 不用：horn_cut(Rk) 才是摆正了的那把刀
    ca = pt(Ra, 0)                                     # 踝轴心 (-31.78, 50, 38.5)
    sil = silhouette(O, 1, extra_circles=[(ca[0], ca[2], ANK_SIL_R)], span=(28.0, 44.0))   # 原版投影轮廓 + 踝端 Ø26 圆角
    add = [placed(dk, Rk), driven_patch(Rk, 26.0),      # 膝从动盘 + Ø19.85 毂 + 补 1.7 贴 S288 法兰
           inter(back_shell(Ra, 3.0), sil)]             # 踝舵机等厚背板，裁成原版轮廓
    fills = [fill_cyl((x, YM, z), 5.0, "y", YT) for (x, z) in ((-27.5, 76.3), (-36.0, 76.3), (-27.5, 84.75), (-36.0, 84.75))]  # 原版膝舵盘 4 孔+沉孔
    fills += [fill_cyl((-23.8, YM, 61.0), 3.2, "y", YT), fill_cyl((-39.8, YM, 61.0), 3.2, "y", YT)]                            # 原版踝背面 2 孔
    # 原版踝轴心的 XL330 副轴定位阶梯孔（Ø5.5 台阶在舵机局部 x=-14.0、Ø2.7 通到 -9.7）也填掉：它整个落在 S288 背板副轴盘 + Ø9.95 轴颈里，
    # 而且那道台阶面 x=-14.0 与手册尺寸下 servo_envelope 的副轴让位面 X_ENV_BOSS=-14.0 **严格共面**（09-13 实测：不填则 L04 导出后剩 1 条
    # 非流形边 + 1 片 -0 体积碎片，export 的 L04 严格断言不过；旧 -13.85 让位面时差 0.15 没这问题）
    fills += [fill_cyl((ca[0], YM, ca[2]), 6.0, "y", YT)]
    # hr50（2026-09-28）：膝法兰 6 颗 M2 沉窝（L04-F08，保护区）打在原版板膝端（x −37.5..−25.8 只 11.7 宽，比 Ø14.9 的沉窝阵窄）+ 电池仓扫掠斜面上，
    #   孔口只剩月牙碎边（clean_check holes：缺 22–89%、方位漂移 42–48°）。沉窝段（膝舵机局部 x 20.0..22.7 = 世界 y 35.0..32.3，与原版板外面齐平）
    #   补一整块 Ø17 实心盘：沉窝外壁 8.5−7.45 = 1.05，沉窝在完整料里打；电池仓扫掠（关节扫掠让位，照留）只在 −x 侧切出一个平直斜面。
    #   盘顶切平到世界 z 88.25（上排两个沉窝顶 87.25 + 1.0）：上面 z 88.3..91.5 是 L04 件号刻字（stamps.py，刻在 y 34.5 面上），不盖住它。
    fills += [inter(placed(cyl(17.0, 2.7, (21.35, 0, 0), axis="x"), Rk), wbox((-45.0, 30.0, 60.0), (-18.0, 36.0, 88.25)))]
    m = union(O, *add, *fills)
    # 膝法兰孔坐面是原版雕花板的台阶（09-11 射线 7.57..9.85），沉到统一 7.5（轴承毂 4.5 + 板 3 的名义面），6 颗 M2×10 咬 2.5
    # 踝舵机 PH2.0 插座（keepouts.yaml:KO01，09-20 背插模型；09-21 线翘 4 mm → lib.CONN_MODE 改 "window"）：背板外 0.4 就是 L07 后臂，
    # 3 mm 背板兜不住 4.5 的线翘区 → 背板在**前口**（局部 +y = 世界 +x 小腿前缘）开通窗，插头装好舵机后从外面插；线翘区 x −13..−17.4 由
    # build_ankle_parts 里 L07 减掉它绕踝轴的扫掠让开（座环端面 −17.4 与线翘区上沿共面、不削；后臂根 −31° 附近 ≤0.7 mm³）。**后口不用**（世界 −x 后缘：
    # L07 后臂在踝 +90° 时整个扫过那里，37/80 mm³）—— 踝是链尾，一个口够（harness.yaml:HB01）。
    m = diff(m, servo_env(Ra), servo_env(Rk), flange_relief(Rk), horn_cut(Rk), horn_cbore(Rk, S["T"] / 2 + S["flange_h"] + 10.2, 2.7),
             conn_cut(Ra, sides=(1,), mode="window"),
             mnt_cut2(Ra, pairs=(0,)),                                   # 上排：板厚 3.0 → M2×6 咬 2.7
             mnt_cut2(Ra, pairs=(1,), cb_x0=-17.7, cb_d=x_bshell_out(3.0, thick=True) - (-17.7)))   # 下排：原版板多的料沉到厚段背板外表面 XR−3=-16.3（沉 1.4；旧 1.55 → -16.15）→ 叠厚 3.3，M2×6
    # 踝副轴颈 Ø9.95：从背板的副轴凸台盘（x -17.0..-14.0，实心）里长出去到 ANK_X1。起点取 X_ENV_BOSS−1.0=-15.0（盘内 1.0，真·体积重叠）：
    # 旧写死 -13.85 = 旧让位体外端面，09-13 改成 X_ENV_BOSS=-14.0 后轴颈端面与让位体/背板内面只差 2e-15 → 并集留下一圈退化面，
    # clean_print_topology 之后仍剩 1 条非流形边（r≈8 盘沿），export 的 L04 严格断言不过；退进盘里就没有共面
    xj0 = X_ENV_BOSS - 1.0
    m = union(m, placed(cyl(ANK_J, xj0 - ANK_X1, ((xj0 + ANK_X1) / 2, 0, 0), axis="x"), Ra))
    # 小腿运动 -90..+60 时，大腿在小腿坐标中运动 -60..+90；反向区间必须交换并取负。
    # 09-14 复审 MAJOR（M2）：DILATE6 平移副本在 L03 环座『端面 × 外圆』凸角 (x 17.0..17.4 × r 15.5..15.9) 斜向没有覆盖，原版板料填到角上、
    # 膝全程 min_gap 0.014（打印 ±0.1 即摩擦）。改 mink=True 连续膨胀（lib.sweep_of），修后膝 −90..60° 全程 0.400（09-14 r2 预检，L04 少 5.6 mm³）。
    # 2026-09-16 行程恢复：−62 → −KNEE_CUT_HI（关节 +62 → +88.5，见 lib.KNEE_*）；刀用 base 版大腿（成品大腿另被小腿扫过，见 build_upper_leg）。
    # hr50 试过加密到 121 步（1.5°/步）：膝前下方那处碎边反而从 30 条变 50 条、峰值内存 3.65 GB → 不是步长问题，恢复 73 步，碎边列入报告。
    m = diff(m, sweep_of(_build_upper_leg_base(), "leg", -KNEE_CUT_HI, 92, 73, grow=0.4, mink=True))
    # 电池让位（09-14 复审 MAJOR M6）：原来扫的是电池名义实体 + grow 0.4，可仓内单边 BAT_CLR=0.6 > 0.4 —— 电池向腿侧挪 0.6 就在髋偏航 −25° 相交 3.77 mm³。
    # 改扫**仓腔**（= 电池能到的所有位置：x BX0..BX1、|y|≤BAT_Y、z BZ0..BZ1，全部 lib 常量，仓冻结不动）再 grow 0.4；只削小腿。
    # was_until_2026_09_28_hr48: bay = wbox((BX0, -BAT_Y, BZ0), (BX1, BAT_Y, BZ1))
    # hr48（电池实测 20 厚，B01 门内面开槽到 BX0_BAT −45.4）：仓腔后界改 BX0_BAT —— 电池能到的位置多了 x −45.4..−43.8 那 1.6，
    #   不扫的话小腿在髋偏航 −24.4°（右 +24.4°）与推到后/外/下角的电池相交 0.79 mm³（hr48_work/bat_scan2.py）；重扫只多削 24 mm³ 一条棱（l04_extra_cut.png）。
    bay = wbox((BX0_BAT, -BAT_Y, BZ0), (BX1, BAT_Y, BZ1))
    # 09-14 r2 复量：仓腔 + DILATE6 0.4 在仓腔『后口×外壁×仓底』三面角 (BX0, +BAT_Y, BZ0) 斜向覆盖为 0——电池推到那个角（物理可达：坐仓底、靠外壁、顶后门）
    # 在髋偏航 −25° 与 L04 点接触（min_gap 0.000、∩ 0；L04∩(仓腔+0.4 方盒) 3.73 mm³）。改 mink=True 连续膨胀（同膝/踝/髋横滚），仓内任意位置 ≥0.4。
    m = diff(m, sweep_of(bay, "yaw2roll", -30, 25, 45, grow=0.4, mink=True))
    m = diff(m, *[region_cut(f, b, mirror=mr) for f, b, mr in LOWER_LEG_REGION_CUTS])   # 09-17 真实姿态区域刀（踝端外侧 / 上头壳）
    # 注意必须扫掠**真正的大腿件**：早先扫的是 ring_boss 的实心凸台（没挖轴承座），把膝从动盘和 Ø19.85 毂整个削掉了
    # was_until_2026_09_28_hr50: return clean_print_topology(keep_main(diff(m, bearing_driven_clearance(Rk)), "lower_leg"))
    # hr50：再补两把清皮刀（都只削 0.01 级的布尔碎片，不碰任何功能面）：
    #   _hub_skin：Ø26 补片被 bearing_driven_clearance 的 128 边内圆（Ø19.85）裁后，残在毂 64 边形外的 0.004–0.012 薄片（膝轴上下 (−31.8, 40–42, 70.6/90.4)，
    #              clean_check 薄膜 1.6 mm² ×2）—— 用与毂同一个 64 边形当内圆、Ø20.3 当外圆的环刀清掉，毂面本身不动；
    #   _bshell_skin：back_shell 的 (e∪e2)−e 在踝舵机顶端背板外面（局部 x −13.62..−13.3、z_l 6–9，世界 y 36.4–36.7）留下 0.01 厚斜片
    #              （clean_check 薄膜 6.4 mm²；原来被区域刀 #9 顺带削掉一部分）—— 背板外面 −13.3 以外、z_l 2.05..10.5 的一层清掉；上排螺丝头坐面就是 −13.3，不动。
    x0h, xih = S["T"] / 2 + S["flange_h"], S["T"] / 2 + S["flange_h"] + 4.0 + 0.5
    _hub_skin = placed(diff(cyl(20.3, 4.25, (15.125, 0, 0), axis="x", sections=128),          # x 13.0..17.25（毂 12.995..17.505 两头都伸出环刀）
                            cyl(P["brg"]["hub_d"], xih - x0h + 0.01, (x0h + (xih - x0h) / 2, 0, 0), axis="x")), Rk)
    _bshell_skin = placed(bx((13.95 - 13.3, 22.0, 10.5 - 2.05), ((-13.95 - 13.3) / 2, 0.0, (10.5 + 2.05) / 2)), Ra)
    return clean_print_topology(keep_main(diff(m, bearing_driven_clearance(Rk), _hub_skin, _bshell_skin), "lower_leg"))

def _rev_sector(c, r0, r1, th0, th1, y0, y1, n=1440):
    """hr50_r2：绕过 c 点的世界 y 向轴（膝轴 / 踝轴）的环扇形 r0..r1 × θ th0..th1（度，从 −z 往 +x 量）× y0..y1 ——
    给离散扫掠的锯齿换"那条棱连续转过去"的正圆柱包络用（锐边不随步长变，加密没用）。"""
    import trimesh
    ann = diff(cyl(2 * r1, y1 - y0, (c[0], (y0 + y1) / 2, c[2]), axis="y", sections=n),
               cyl(2 * r0, y1 - y0 + 0.2, (c[0], (y0 + y1) / 2, c[2]), axis="y", sections=n))
    R = r1 + 4.0
    P = [(c[0], y, c[2]) for y in (y0 - 0.1, y1 + 0.1)] + [(c[0] + R * math.sin(math.radians(t)), y, c[2] - R * math.cos(math.radians(t)))
                                                          for t in (th0, (th0 + th1) / 2, th1) for y in (y0 - 0.1, y1 + 0.1)]
    return inter(ann, trimesh.convex.convex_hull(np.array(P)))

def _theta_sweep_env(src, body, a0, a1, rr, yy, tt, grow=0.4, extra=0.015, inset=0.05, pre=1.0, seg=1.0, seg2=1.5):
    """hr50_r2：离散扫掠 sweep_of(src, body, a0, a1, n, grow, mink=True) 的"连续版"局部包络（只取 src 在 r rr × y yy × θ_c tt 里那块刀）。
    做法：刀块 G（= src 先 minkowski ±grow，同 sweep_of）映到 (r, y, s = θ°·k) 空间 —— 绕 body 关节轴（世界 y 向）转 a 就是沿 s 平移 a·k ——
    在那里沿 s 做 minkowski（= 从扫掠起点连续推到终点），再映回世界系。r / y 另让 extra（锐边包络比齿根再让 0.015）；
    两端各往里收 inset（0.05 mm）→ 端面藏在原扫掠第一刀 / 最后一刀里面（落在空处，不在老面上刮新皮）。映射前后都细分到 seg / seg2 mm，弦高误差 ≤ 0.004。"""
    import trimesh
    from .lib import TW
    T = TW(body); o = T[:3, 3]
    sgn = -1.0 if T[1, 2] < 0 else 1.0                       # 绕 −y 转 a → θ（从 −z 往 +x 量）加 a
    k = math.pi / 180.0 * 0.5 * (rr[0] + rr[1])               # s = θ°·k，在中径处近似等距
    def sect(r0, r1, th0, th1, y0, y1):
        ann = diff(cyl(2 * r1, y1 - y0, (o[0], (y0 + y1) / 2, o[2]), axis="y", sections=720),
                   cyl(2 * r0, y1 - y0 + 0.2, (o[0], (y0 + y1) / 2, o[2]), axis="y", sections=720))
        R = r1 + 6.0
        P = [(o[0], y, o[2]) for y in (y0 - 0.1, y1 + 0.1)] + [(o[0] + R * math.sin(math.radians(t)), y, o[2] - R * math.cos(math.radians(t)))
                                                              for t in np.linspace(th0, th1, 5) for y in (y0 - 0.1, y1 + 0.1)]
        return inter(ann, trimesh.convex.convex_hull(np.array(P)))
    dth = math.degrees(pre / rr[0])
    Upre = inter(src, sect(rr[0] - pre, rr[1] + pre, tt[0] - dth, tt[1] + dth, yy[0] - pre, yy[1] + pre))   # 先粗裁（外扩 pre > grow·√3）再膨胀，省内存
    G = inter(minkowski_box(Upre, (-grow,) * 3, (grow,) * 3), sect(rr[0], rr[1], tt[0], tt[1], yy[0], yy[1]))
    v, f = trimesh.remesh.subdivide_to_size(np.asarray(G.vertices), np.asarray(G.faces), max_edge=seg)
    A = trimesh.Trimesh(np.column_stack([np.hypot(v[:, 0] - o[0], v[:, 2] - o[2]), v[:, 1],
                                         np.degrees(np.arctan2(v[:, 0] - o[0], -(v[:, 2] - o[2]))) * k]), f, process=False)
    if A.volume < 0: A.invert()
    s0, s1 = sorted((-sgn * a0 * k, -sgn * a1 * k))
    E = minkowski_box(A, (-extra, -extra, s0 + inset), (extra, extra, s1 - inset))
    v2, f2 = trimesh.remesh.subdivide_to_size(np.asarray(E.vertices), np.asarray(E.faces), max_edge=seg2)
    th = np.radians(v2[:, 2] / k)
    out = trimesh.Trimesh(np.column_stack([o[0] + v2[:, 0] * np.sin(th), v2[:, 1], o[2] - v2[:, 0] * np.cos(th)]), f2, process=False)
    if out.volume < 0: out.invert()
    return clean_print_topology(out)

@memo
def build_lower_leg():
    """小腿成品（导出件）= _build_lower_leg_core() − 膝锯齿包络刀。
    hr50_r2（2026-09-28）：膝前下方（世界 x −25.8..−21.8、y 36.5..37.6、z 67.3..69.8）clean_check 碎边 30 条（slab 0.124）= 大腿 base 版绕膝轴
    73 步（2.51°/步）扫掠刀一条直角棱留下的锯齿：齿根 r 14.502、齿尖 r 14.740（膝轴 (−31.777, *, 80.5)；θ 从 −z 往 +x 量 24.2°..43.0°）。
    加密步长没用（hr50 试过 121 步：碎边 30 → 50，锐边角度不随步长变）→ 换成那条棱连续转过去的包络：以膝轴为轴的正圆柱面 r 14.49（比齿根再让 0.012）。
    刀 = 环 r 14.49..16.0 × y 36.5..37.95 × θ 24.2151°（最外一刀直角棱的角点；θ 更小一侧是原版板，不碰）..60°：只削掉齿（实算 0.67 mm³），
    其余部分本来就空。新让位 = 旧扫掠 ∪ 这把包络刀 ⊇ 旧让位。不碰任何孔 / 沉窝（膝法兰沉窝在 y ≤ 35.0，轴承让位环在 r ≤ 14.0）。
    L03 膝刀（build_upper_leg）和 L05/L07 踝刀（build_ankle_parts）仍拿内核件扫 → 那两边的让位与改前逐位相同。"""
    import trimesh
    Rk = drv_from("leg"); ck = pt(Rk, 0)                               # 膝轴（世界 y 向）过 (−31.777, *, 80.5)
    y0, y1, r0, r1, th0, th1 = 36.5, 37.95, 14.49, 16.0, 24.2151, 60.0
    ann = diff(cyl(2 * r1, y1 - y0, (ck[0], (y0 + y1) / 2, ck[2]), axis="y", sections=1440),
               cyl(2 * r0, y1 - y0 + 0.2, (ck[0], (y0 + y1) / 2, ck[2]), axis="y", sections=1440))
    wedge = trimesh.convex.convex_hull(np.array([(ck[0], y, ck[2]) for y in (y0 - 0.1, y1 + 0.1)]
                                                + [(ck[0] + (r1 + 4.0) * math.sin(math.radians(t)), y, ck[2] - (r1 + 4.0) * math.cos(math.radians(t)))
                                                   for t in (th0, (th0 + th1) / 2, th1) for y in (y0 - 0.1, y1 + 0.1)]))
    return clean_print_topology(diff(_build_lower_leg_core(), inter(ann, wedge)))

SOLE_SLIDE_CLR = 0.2   # hr13：鞋底沿 −z 滑装时对 L05/L07 侧面的单边让位（TPU 件；PETG/PLA 内轮廓打印偏移 ~0.15，见 tolerances.yaml）

@memo
def build_ankle_parts():
    """L05主脚、L07可拆后轴承臂和L06鞋底；后臂拆下后两侧分别轴向装入。"""
    Ra = drv_from("ankle_left")                        # 踝舵机在小腿里，法兰朝 +y 外侧
    O = union(orig("ankle_left", "ankle_left"), orig("ankle_left", "foot_left"))
    # 原版踝件在法兰面外是雕出来的曲面，6 个孔的坐面从 3.0 到 6.65 各不相同（螺丝得配 3 种长度）。
    # 加厚从动盘到 6.7 把 6 个坐面拉平，再统一沉 1.2 → 叠厚 5.5，6 颗一样的 M2×8 咬 2.5
    d1, h1, xe, xi = driven(ring=RINGS["left_ankle"], d=26.0, t=6.7)
    add = [placed(d1, Ra), driven_patch(Ra, 26.0)]
    fills = [fill_cyl((-31.76, 66.0, 38.48), 5.6, "y", 7.0), fill_cyl((-31.77, 66.0, 32.5), 5.0, "y", 7.0)]   # 原版 XL330 舵盘/定位孔
    # was_until_2026_09_28_hr50_r2:     ll = build_lower_leg()
    ll = _build_lower_leg_core()                       # hr50_r2：踝刀仍拿内核件扫（与改前逐位相同），导出 L04 的膝锯齿包络刀不影响 L05/L06/L07
    # 小腿相对踝件的反向扫掠 → 踝件让位。09-14 复审 MAJOR（M3/M4）：DILATE6 在 L04 两处凸角斜向没让位——
    #   M3：背板副轴凸台盘 Ø16 (x −17..−14) 的『端面 × 外圆』角，L07 内孔 r 8.0 段 (x −17.4..−17.0) 与盘外圆同径对接，零位 min_gap 0 全圈；
    #   M4：薄段背板 (x −13.3..−10.3) 顶边 z 9.8 与 Ø26 轮廓相交的角 (|y| 8.5, r 13.0)，L05 在分体面 x −13.5..−13.3 留 0.2 厚唇与它同角对接（零位 0.0013、多角为 0）。
    # 改 mink=True 连续膨胀：L05×L04 全程 0.56、L07×L04 0.34..0.40（09-14 r2 预检；L05 −13.7 mm³、L07 −29 mm³，6700 座 Ø15.1 不受影响）。
    clr = sweep_of(ll, "ankle_left", ANK_SWEEP_LO, ANK_SWEEP_HI, ANK_SWEEP_N_LEG, grow=0.4, mink=True)   # 09-16 [−92, 62]（反向；关节 −62..+92）
    # 旧11姿态在±33.1..33.4度留下约0.059mm³碰撞，2.5度主检查也会漏过。
    # 1度让位网格；assembly_audit另用0.125度与真实策略中间角做回归。
    sv = sweep_of(servo_env(Ra), "ankle_left", ANK_SWEEP_LO, ANK_SWEEP_HI, ANK_SWEEP_N_SERVO, grow=0.3)
    # 09-21 踝舵机前口（局部 +y）的 PH2.0 线翘区（lib.conn_zone "A"：x −13..−17.5）随小腿走，L07 后臂内面 −16.7 / 6700 座环 −17.4 绕踝轴
    # 扫过它 → 同 sv 的反向扫掠减掉（1°/步，grow 0.3）。零位只削座环外沿 0.1 深一小片、后臂根部 −31° 附近 ≤0.7 mm³（wire4_probe，记录第 31 条）。
    # hr24：放过 6700 座环（x ≤ ANK_X0=−17.4 内 Ø ANK_COL+0.2）—— 线翘区上沿 −17.4 与座环端面共面（s288.wire_clr 0.4 就是按它定的），grow 0.3 不许再啃座环
    # （hr23 啃了 0.4 → L2 L07-F01/F02 座深 3.8 / 座径 15.58）。座环前面 x −16.7..−17.4 那 0.7 的后臂料照削。
    # hr50 试过把关节负端 −62° 收到 −34°（策略最负 −26.3°）：原来被线翘区削掉的那片（L07 +y 面 z 27.5..29.6、座环前沿 z 36..37.5）露出 clr 6.2°/步的锯齿，
    #   L07 新增碎边 41 + 10 条；它啃的那段 Ø4.6 通道（y ≥ 32.25）现在已经开槽 → 收刀没有收益，恢复原样。
    hump = diff(conn_hump_sweep(Ra, (1,), "ankle_left", ANK_SWEEP_LO, ANK_SWEEP_HI, ANK_SWEEP_N_SERVO, grow=0.3),
                placed(cyl(ANK_COL + 0.2, 8.0, (ANK_X0 - 4.0, 0, 0), axis="x"), Ra))
    # 后臂改为L07可拆件；L05先接法兰，舵机+L05装入L04并拧背孔，最后装L07。
    access = union(*[placed(cyl(4.6, 20.0, (-19.0, sy * S["mnt_dx"], S["mnt_z"][0]), axis="x"), Ra) for sy in (1, -1)])
    collar = placed(cyl(ANK_COL, ANK_X0 - ANK_X1b, ((ANK_X0 + ANK_X1b) / 2, 0, 0), axis="x"), Ra)      # 6700ZZ 外圈座（背面）
    seat = placed(cyl(S["brg_od"] + 0.1, ANK_X0 - ANK_X1b + 0.02, ((ANK_X0 + ANK_X1b) / 2, 0, 0), axis="x"), Ra)
    # 2026-09-16 行程恢复（lib.KNEE_HEEL_*）：膝 ≥67.5° 起脚跟撞髋件 L02 / L01 杯壁 / 髋横滚 6704（原设计没有这把刀；髋侧不能挖）；
    # ≥87.5° 从动盘外面靠边处擦到大腿 L03 侧壁 / 髋俯仰舵机（L03 那面壁只剩 0.94，改削盘面，见 lib 注释 ③）。
    # 把这五件绕**膝轴**在踝件系里反向扫掠（关节 +50..+88.5；髋俯仰置零，组合姿态归 Gate L6 box_collisions），削 L05 跟底边+盘面 / L07 后臂跟部 / L06 鞋跟。
    # 大腿用 base 版（成品 ⊆ base，保守）；build.py 里 L03 已在本函数之前建好，memo 不重算。
    # was_until_2026_09_28_hr50_r2:     heel = sweep_of(union(*[m for m in (build_hip(), build_yaw2roll(), hip_roll_bearing(), hip_pitch_bearing(), _build_upper_leg_base(), servo_env(drv_self("upper_leg_left"))) if m is not None]),
    # was_until_2026_09_28_hr50_r2:                     "leg", KNEE_HEEL_LO, KNEE_HEEL_HI, KNEE_HEEL_N, grow=0.4, mink=True)
    heel_src = union(*[m for m in (build_hip(), build_yaw2roll(), hip_roll_bearing(), hip_pitch_bearing(), _build_upper_leg_base(), servo_env(drv_self("upper_leg_left"))) if m is not None])
    heel = sweep_of(heel_src, "leg", KNEE_HEEL_LO, KNEE_HEEL_HI, KNEE_HEEL_N, grow=0.4, mink=True)   # hr50_r2：刀体单独留一份（heel_src），给下面的连续包络用；这把刀本身一字没改
    m = keep_main(diff(union(O, *add, *fills, collar), horn_cut(Ra), horn_cbore(Ra, xe, 1.2), flange_relief(Ra), seat, access, sv, hump, clr, heel,
                       *[region_cut(f, b, mirror=mr) for f, b, mr in REAR_ARM_REGION_CUTS]), "ankle_foot")   # 09-17 后臂顶边区域刀   # 沉窝从加厚盘外表面 xe=法兰面+6.7=19.7 起（旧写死 19.55），叠厚 5.5
    from ankle_split import split_ankle
    main, arm, sole = split_ankle(m, diff(orig("ankle_left", "sole_left"), m, heel), Ra)      # 鞋底也吃同一把膝刀（原来只减主脚）
    # hr13：鞋底 = 原版鞋底 − 脚，减出来的鞋跟内壁与 L07/L05 侧面**共面**（零间隙）。鞋底沿 −z 装入（assembly.yaml 步 5）时内壁贴着 L07 侧面滑，
    # Gate L4 面片噪声 1.15 mm³（厚 0.014）恒红；L06 的 1010 个退化面/968 条非流形边也出自这些共面。按不变量"鞋底 ∩ (L05∪L07 沿 +z 扫掠 30) = 0"
    # 让位：侧向 SOLE_SLIDE_CLR，**不向 −z 长**（鞋底顶面与脚底面仍贴合承力，4×M2×8 F23 从下拉紧）。TPU 95A 件，0.2 只是让滑装不靠拉伸。
    # hr50（2026-09-28）：alpha_sitstand/sit_down 第 34–35 步（右膝 −77°、右髋俯仰 +31°，不是极限段）右 L07 后臂前端"鸟嘴"（世界 x −14.1..−9.9、
    #   y 30.2..33.4、z 29.3..32.0，左件系）顶进右 L02 前面 14.55 mm³ / 深 2.36（左侧 0.65 / 0.36）。鸟嘴是原版脚外形留下的前伸薄舌（下沿还是扫掠锯齿），
    #   不承力；x ≥ −14.5（= 碰撞最前沿 −14.09 再让 0.4）、z ≥ 28 的部分一刀平切掉（切口是 3 mm 高的竖直平面）；6700 座（x ≤ −22）和起子通道（x ≤ −21.5）不碰。
    arm = diff(arm, wbox((-14.5, 25.0, 26.0), (5.0, 40.0, 40.0)))     # 刀底 z 26（鸟嘴下沿锯齿最低 26.4）：切口是整个截面，不在锯齿上留尖
    # hr50：L05 顶面在分体面 y 36.5..36.7 留一条 0.2 宽、高出顶面 0.1–0.3 的小唇（世界 x −51.7..−40.1，clean_check 薄膜 6.3 mm²）。
    #   顶面（y ≥ 36.9 实测）是斜平面 z = 27.22 + 0.0355·(x + 51)；这条带（y 36.4..36.85）按同一斜面压到顶面下 0.02，唇削平，别处不动。
    import trimesh
    _zp = lambda x: 27.22 + 0.0355 * (x + 51.0) - 0.02
    main = diff(main, trimesh.convex.convex_hull(np.array([[x, y, z] for x in (-52.5, -39.5) for y in (36.4, 36.85) for z in (_zp(x), 29.5)])))
    # hr50：两个 Ø4.6 起子通道（与 L04 上排螺丝同轴，用户认可的通道）在 y ≥ 32.25 那段顶上只剩 0.2–0.3 的皮（L04 背板 0.4 间隙扫出来的顶面，
    #   clean_check 薄膜 2.8 / 2.7 mm²）→ 这段通道顶开成槽（宽同孔径 4.6、y 32.25..33.4），不留皮；螺丝刀照样沿通道进。
    arm = diff(arm, *[wbox((cx - 2.3, 32.25, 31.0), (cx + 2.3, 33.4, 36.0)) for cx in (-31.777 + 8.0, -31.777 - 8.0)])
    # hr50：鞋底外侧沿（世界 x −36.4..−27.0、y 69.9..70.07、z 25.5..26.4）夹在 L05 踝盘外面（y 69.7）+ 滑装让位 0.2 和鞋底外皮之间，只剩 0.1–0.17（薄膜 11.4 mm²）
    #   → 这一段沿口往外补到 y 70.8（沿厚 0.9），滑装让位刀照切；外侧没有别的件（踝盘螺丝在 z ≥ 31，从 +y 拧，够不到 z ≤ 26.4）。
    sole = union(sole, wbox((-37.2, 69.6, 25.3), (-26.3, 70.8, 26.38)))
    sole = diff(sole, minkowski_box(union(main, arm), (-SOLE_SLIDE_CLR, -SOLE_SLIDE_CLR, 0.0), (SOLE_SLIDE_CLR, SOLE_SLIDE_CLR, 30.0)))
    # hr50_r2（2026-09-28）：L07 +y 面（世界 x −33.7..−25.7、y 35.1..36.3、z 27.5..29.6）clean_check 碎边 41 条 = 上面 clr（小腿绕踝轴 26 步、6.16°/步）
    #   一条棱留下的锯齿：齿根 r 10.976、齿尖 r 10.779（踝轴 (−31.777, *, 38.5)；θ 从 −z 往 +x 量 −10.3°..28.4°）。换成那条棱连续转过去的包络：
    #   以踝轴为轴的正圆柱面 r 10.99（齿根再让 0.014）；刀 = 环 r 10.6..10.99 × y 35.1..36.45 × θ −12°..30°（两头 r ≤ 10.99 本来就空，端面落在空处）。
    #   放在鞋底让位之后、只切 L07 → L05/L06 逐位不变；y ≤ 36.45 够不到 L05（分体面 y ≥ 36.5）。新让位 = 旧 clr ∪ 这把刀 ⊇ 旧让位。
    arm = diff(arm, _rev_sector(pt(Ra, 0), 10.6, 10.99, -12.0, 30.0, 35.1, 36.45))
    # hr50_r2（2026-09-28）：脚跟膝刀（heel，髋侧几件绕膝轴 40 步 ≈0.99°/步）的锯齿 —— L06 (1.17, 34.8, 23.0) 29 条、L07 (−1.77, 34.96, 21.46) 47 条 +
    #   (−13.22, 33.91, 20.27) 6 条：刀角在刀体系 θ_c 110–115°、r 66.19（+0.4 = 齿根 66.55–66.71），落到脚上是 θ 17–33° 那段扫掠起点附近。
    #   加密没用（锐边角度不随步长变）→ 换成刀体局部（r 58.5..71.5 × y 25..39.5 × θ_c 98..134，刀在这圈里只在 θ_c 102.5..130.9、y ≤ 38.4，
    #   三个方向的裁边都不切刀）在 (r, y, θ) 空间沿 θ 连续拉满 38.5° 的包络（_theta_sweep_env）：r / y 比齿根再让 0.015，两端各收 0.05 藏进第一 / 最后一刀里。
    #   只切 L06 / L07（放在鞋底让位之后：L05 逐位不变，鞋底让位仍按带齿的 L07 算）；新让位 = 旧 heel ∪ 包络 ⊇ 旧让位。
    heel_env = _theta_sweep_env(heel_src, "leg", KNEE_HEEL_LO, KNEE_HEEL_HI, (58.5, 71.5), (25.0, 39.5), (98.0, 134.0))
    sole = diff(sole, heel_env); arm = diff(arm, heel_env)
    # 三件都过 clean_print_topology（与 L04 同法：float32 焊合 → manifold 规范化）：hr12 导出 L05 1 条非流形边、L07 58 退化面/47 非流形边、
    # L06 1010/968（09-14 起的旧问题，"切片复核"一直挂着）；hr13 试算 L06 让位后 0/0，L05/L07 体积不变。
    # hr51（2026-09-28 20:xx 用户定，v5 打印包）：踝舵机机身两条底棱绕踝轴转出的圆 r 13.79（placed 网格实测）离 L05 弧面只有 0.65
    #   （上面 sv 刀 = 包络 c 0.3 + DILATE6 0.3 → 弧面 r 14.36–14.50），用户："画出来的圆弧不要离下面那么近"。试过包络整体加厚到 c 0.7：机身侧面扫掠
    #   一起外推，L05 左侧 r 17.2 那道 0.75 壁只剩 0.10 → 不用。改成只对棱角圆下刀：以踝轴为轴的正圆柱 r ANK_ARC_R=15.0（缝 1.21），
    #   轴向只覆盖机身段 ±(T/2 + clr 0.3 + DILATE 0.3) = ±10.6（= sv 刀在 r ≤ 14.22 留下的毂面 / 后壁前脸位置，两端各再进 0.02 避免共面留退化面），
    #   毂面 / 后壁 / 侧面扫掠面一概不动；弧面外地板（r 16.29）剩 1.29 ≥ 1.0；左侧 0.75 壁（r 17.2）在刀外。
    #   只减 L05，放在鞋底让位与 heel_env 之后 → L06/L07 逐位不变（同刀碰 L06 0 / L07 0 mm³）。隔离验算（scratch l05_cyl.py）：削 324.7 mm³，
    #   舵机方盒对 L05 全行程最近 0.60 = 法兰面对毂面的轴向缝（未动，与转角无关）。
    ANK_ARC_R = 15.0
    main = diff(main, placed(cyl(2 * ANK_ARC_R, 2 * (S["T"] / 2 + S["clr"] + 0.3 + 0.02), (0, 0, 0), axis="x", sections=1440), Ra))
    return clean_print_topology(main), clean_print_topology(arm), clean_print_topology(keep_main(sole, "sole"))

def build_ankle_foot():
    main, _, sole = build_ankle_parts()
    return main, sole

def build_ankle_rear_arm():
    return build_ankle_parts()[1]
