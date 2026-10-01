"""通用几何原语与参数（原 tools/cad/duck.py 前半段，纯搬运）。
S288 版 Microduck 整鸭参数化 CAD → cad/duck_s288/
关节轴线取自 microduck_rl/robot_walk.xml（kin.py）；保留运动学骨架，S288动力学与可用运动域另验。
所有连杆都在**世界坐标（零位姿）**里建，导出时再转回各自的 MJCF 连杆坐标系；右腿由左腿镜像。
坐标：X 前 Y 左 Z 上，mm。舵机尺寸标 [量] 的见 s288.py。"""
import os, sys, math, json, numpy as np, trimesh
from . import s288, kin as K
from .s288 import S, cyl, bx, union, diff, inter, placed, frame
from trimesh.transformations import rotation_matrix as rot
from collections import Counter

OUT = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "cad", "duck_s288"))
os.makedirs(OUT, exist_ok=True)
B, ORDER = K.load()
Wl, CLR = S["wall"], S["clr"]
XR = s288.x_rear_face_lo() - CLR   # -13.3：载体背板内面（舵机坐标）——贴下半段背面（手册 -13.0；旧 -13.15）。上半段背面在 -T/2=-10.0，靠垫柱
X_ENV_BOSS = s288.x_env_boss_face()  # -14.0：servo_envelope 背面副轴让位的外端面（旧 -13.85）；等厚背板 back_shell(t) 在副轴处的外端 = 这里再 −t
def x_bshell_out(t=3.0, thick=False):
    """等厚背板 back_shell(t) 的外表面 x：薄段处 -(T/2+CLR+t)=-13.3；厚段处 XR−t=-16.3（t=3）。沉孔/坐面从这里推，不写死。"""
    return (XR - t) if thick else -(S["T"] / 2 + CLR + t)
PLT = 3.0          # 舵盘板 / 背板厚
RING_T = 4.0       # 6704ZZ 20×27×4 厚（6702ZZ 15×21×4 同厚）
SLIDE_CLR = 0.4    # 有座环的载体，法兰侧滑槽开到法兰面下这么深（13.4；旧 13.25）。09-12 复审：设计 agent 原给 0.2（= servo_slide 的名义间隙），
                   # 但 T01 倒放打印时座环壁在缺口扇区是跨 15.0 mm（Ø14+1）弦长的**朝下桥面**，桥面下垂/支撑残留 ≥0.2 就把舵机滑入堵死 → 放到 0.4。
                   # 代价：座孔在 ±41° 缺口扇区少 0.2 的座长（4.55→4.35），外圈仍整圈坐住；挡肩（12.5..12.95；旧 12.35..12.80）不受影响。
SEAT_CLR = 0.3     # 从动件对载体座环的径向/轴向让位
# 踝关节副支撑（照原版：轴承在舵机背面）。舵机局部 x：L04 的副轴凸台外端 X_ENV_BOSS−3 = -17.0（旧 -16.85），原版 leg 板到 -17.5
ANK_J, ANK_COL = 9.95, 19.6            # L04 轴颈 Ø9.95（6700ZZ 内圈 Ø10）/ L05 座外径
# 踝让位扫掠：原来只让到 ±37，导致踝的无碰撞连续区间被截在 ±37.5（±40 起 L05/L07 与小腿、踝舵机开始咬）。
# 实测那几处咬合全是 0.03~0.5mm 的薄膜（不是真堆料），09-1x 扫到 ±62 整段吃下来。
# 2026-09-16 行程恢复（用户定：能力与原版一致，转角可不同）：踝目标区间 = frozen.yaml:left_ankle.target_range_deg [−31.2, 90]
# （能力包络推导：alpha_stand 要到 +87 + 3° 边）。这两把刀是"小腿 / 踝舵机在踝件系里的反向扫掠"，角度取负：关节 +90 ↔ 反向 −90，
# 所以扫 [ANK_SWEEP_LO, ANK_SWEEP_HI] = [−92, 62]：− 端到 −90 再留 2°（同膝刀 92 的写法）；+ 端保留原 62（包络只要关节 −31.2，
# 已让出的口袋不填回去）。关节 +90 时舵机本体横躺、最低点在轴下 10，脚顶面原在轴下 8 → 这把刀把 x>10 侧顶面降到 ≤ −11
# （docs/reports/行程恢复_扫掠体积分析_2026-09-16.md §9）。采样密度不变：小腿 ≈6°/步、舵机 ≈1°/步。
ANK_SWEEP_LO, ANK_SWEEP_HI = -92.0, 62.0
ANK_SWEEP_N_LEG = int(round((ANK_SWEEP_HI - ANK_SWEEP_LO) / 6.2)) + 1        # 26（旧 ±62 时 21）
ANK_SWEEP_N_SERVO = int(round(ANK_SWEEP_HI - ANK_SWEEP_LO)) + 1              # 155（旧 125）
# 2026-09-16 行程恢复：膝目标区间 = frozen.yaml:left_knee.target_range_deg [−47.5, 86.2]（能力包络推导：alpha_stand 要到 +83 + 3° 边）。
# L6 量出的无碰撞区间以前卡在 65（docs/reports/行程恢复_扫掠体积分析_2026-09-16.md §2）：膝 ≥67.5° 起
#   ① 脚跟（L07 后臂跟部 / L06 鞋跟 / L05 跟底边）撞髋件 L02、L01 杯壁、髋横滚 6704 —— 原设计没有这把刀；髋侧不能挖（挖后 L02 剩壁 0.2、切到横滚从动盘）
#   ② 小腿 L04 踝舵机笼壁 / 踝舵机本体 撞大腿 L03 —— 原膝刀 (−62, 92) 只到关节 62
#   ③ 膝 87.5° 起 L05 从动盘外面靠边处（r ≈ 10..15，不到法兰螺丝 r 5.25）擦到大腿 L03 侧壁、90° 时擦到髋俯仰舵机 —— L03 那面壁只剩 0.94，
#      改削从动盘外表面（build r2 复核：45 mm³ @ 90°，87.5° 只有 0.4）
# 三把刀都扫到关节 KNEE_CUT_HI = 88.5：目标 86.2 + L6 的 2.5° 网格要求 87.5° 采样点无碰撞（再靠 grow 0.4 兜离散误差）；
# 不扫到 90 —— 86.2..90 是原版任务用不到的角度（L6 只记 WARN），多扫只会多削脚跟。
KNEE_CUT_HI = 88.5
KNEE_HEEL_LO, KNEE_HEEL_HI, KNEE_HEEL_N = -KNEE_CUT_HI, -50.0, 40   # ①③ 髋侧三件 + 大腿(base) + 髋俯仰舵机 绕膝轴在踝件系里**反向**扫掠：关节 +50..+88.5 ↔ −88.5..−50，~1°/步
KNEE_L03_LO, KNEE_L03_HI, KNEE_L03_N = 60.0, 87.0, 30    # hr07：L03 这把刀上限 88.5→87.0（训练限位 86.2+0.8）：6703 整体座底壁 r 12.8（世界 z 89.7）vs 踝舵机顶 87°≈89.1+0.4，88.5° 会削薄座壁        # ② 小腿+踝舵机绕膝轴在大腿系里**正向**扫掠（子件在父件系里，角度 = 关节角）：关节 60..88.5，~1°/步
ANK_X0 = X_ENV_BOSS - 3.0 - 0.4        # -17.4：L05 座 x 起点 = L04 背板(t=3)副轴凸台外端 -17.0 + 扫掠膨胀 0.4（09-13 手册尺寸下正好等于旧值 -17.4：旧 -16.85+0.4 再留 0.15）
ANK_X1b = ANK_X0 - 4.0                 # -21.4：L05 座 x 终点（轴承厚 4.0）
ANK_X1 = ANK_X1b - 0.2                 # -21.6：L04 轴颈伸到这里（比座深 0.2）
# tools/cad/ankle_split.py 的分体站位（contact_x -13.7 / 6700 喂入 -17.4 / 后臂端 -21.4/-21.75）按 -17.4/-21.4 写死，ANK_X0 变了那边要一起改
assert abs(ANK_X0 + 17.4) < 1e-9 and abs(ANK_X1b + 21.4) < 1e-9, (ANK_X0, ANK_X1b)

P = dict(
    brg=dict(od=27.0, bore=20.0, hub_d=19.85, seat_d=27.15, ring_od=33.0, lip_d=24.0),   # 输出端支撑轴承 6704ZZ 20×27×4（髋横滚/膝）：Ø20 毂在内圈，外圈压进载体。ring_od/lip_d 只有 carrier(ring=True) 用，lip_d 按外圈内径≈24.5 假设
    # ⑤ 髋偏航法兰侧径向轴承 —— 原版此处本来就有一颗 22×16×4（舵盘面平面，受力报告 §4 判为推力式夹持），S288 法兰配不了那种夹法，我们改法兰侧径向 6702（docs/reports/受力分析与双支撑决策_2026-09-12.md §4：单腿支撑弯矩 331 N·mm、零副支撑，全机最差；
    #    原版那颗 22×16×4 推力轴承靠 XL330 舵盘凸台当内圈、轴向夹在甲板与舵盘板之间，S288 的 Ø14 塑料法兰复制不了那种夹法）。
    #    为什么不是和髋横滚/膝同规格的 6704：髋横滚 6704 本体（绕横滚轴 z=102.5，x 19.0..23.0；旧 18.85..22.85）顶到 z 116、它的 L01 座 Ø33 顶到 z 119，
    #    偏航 6704 本体 r 13.5 → x≤19.5，两颗轴承本体在零位就相交 4.39 mm³（09-12 探针实测，旧站位）。6702 本体 r 10.5 → x≤17.5，座 Ø24.6 → x≤18.3，
    #    离横滚轴承与横滚座各 0.55（旧站位；09-13 横滚轴承外移 0.15 → 0.70）。毂 Ø14.85 上 6 孔外缘到毂边 0.975（horn_r=5.25、过孔 Ø2.4：7.425−5.25−1.2；旧 Ø2.2 时 1.075、旧 r 4.75 时 1.575）。
    #    09-14 复审 R3g MAJOR **豁免**（<1.2 规则）：毂径受 6702 内径 15 定死、Ø2.4 是 09-13 用户定案（TC02 缩水后 2.2 穿不过 M2）、分度圆 5.25 是手册，三者都不能动；
    #    毂 x 13.0..17.0 整段就是内圈区（17.0..17.5 是 Ø4.4 沉窝层），没有"轴承不套的 x 段"可加厚。该韧带非承力：螺丝预紧后过孔壁不受拉，径向力走毂外圆→内圈（韧带受压）；
    #    打印后过孔缩 0.15/边 → 实际 ≈1.1。mechanical_audit review_r3 按 ≥0.9 盯着（0.975 −0.075 隙配裕量）。
    brg_yaw=dict(od=21.0, bore=15.0, hub_d=14.85, seat_d=21.15, ring_od=24.6, lip_d=18.5),   # 6702ZZ 15×21×4；lip_d 挡肩内径只压外圈端面（外圈内径按 ≥Ø19.0、内圈外径按 ≤Ø17.2 假设，实物到手量）
    m2_head_d=4.0, m2_head_h=1.6,
    m2_cbore_d=4.4,                                     # M2 盘头（Ø3.8~4.0）的沉窝直径。09-13 由 4.2 → 4.4：TC02 实测本机内轮廓缩 0.15/边，4.2 打出来 ≈3.9 坐不进 4.0 的头；原版 XL330 版用的就是 Ø4.4（docs/参考对比_2026-09-12 §5-C）
    tube_hole_d=4.4,                                    # L03 上排 M2 铜垫管 Ø4×5 的过孔（同一个缩水理由，4.2 → 4.4）
    bat=(20.0, 25.0, 72.1),                             # 2026-09-27 到货实测（花牌 竞技短剑 3S 1100，用户卡尺）厚 20.0 宽 25.0 高 72.1、带线 70.8 g；x 厚 y 宽 z 高。
                                                        # was_until_2026_09_27: (18.0, 25.0, 72.0) Infinity RS 商品页标称。仓常量 BX0/BZ1 仍按 18/72 字面量（T01 已打印、冻结），x 差额由 B01 门槽补（DOOR_ADD/DOOR_SLOT_*）
    brd=(65.0, 32.0), brd_h=(58.0, 23.0),               # 香橙派 Zero 3W，孔距 [量]
    foot=(54.0, 40.0), foot_t=3.5, sole_t=3.0,
    seat_d=22.18, seat_t=3.9, seat_disc_d=21.6,         # 原版底壳/支架的 Ø22 半轴承座（实测），我们用 PLA 圆盘当滑动轴颈（径向留 0.3）
)
# ── 电池仓（后开口抽拉式）。位置是量出来的，三条硬约束：
#   ① 上：躯干壳顶内表面在这块 footprint 上最低 151.4（x -44..-36 那道棱），电池顶必须低于它
#   ② 下/侧：腿绕髋偏航扫到 ±25~30° 时会往中线里荡，z<100 处最近能到 |y|=7~13，所以侧导轨只能长在 z≥100
#   ③ 后：躯干壳后缘 x=-46.4，且 |y|<14 这一层壳只有一层顶棚 —— 所以电池能直接往后抽出来
BAT_CLR = 0.6                                       # 单边间隙
# 位置又往前 2、往下 4 挪过一次：电池的放电线只有 5.5cm、17AWG 硅胶线单根 OD≈2.8，
# 原来电池顶到壳顶只剩 1.2mm，线根本出不来。挪完后段净空 5.1 / 前段 12，而且腿的接触起始角没变（还是 ±25°）
BX1 = -24.6                                         # 仓前壁内表面
BX0 = BX1 - (18.0 + 2 * BAT_CLR)                    # -43.8 仓后口
BZ0 = 73.7                                          # 仓底内表面
BZ1 = BZ0 + 72.0 + 2 * BAT_CLR                      # 146.9（壳顶最低 151.4）
BAT_Y = 25.0 / 2 + BAT_CLR                          # 13.1 仓内半宽
RAIL_Y, RAIL_Z0 = 15.0, 100.0                       # 侧导轨外半宽 / 起点（再往下腿会扫到）
DOOR_T, DOOR_Z1, DOOR_YLO = 3.0, 112.0, 8.0         # 仓门厚 / 顶 / 下段收窄到的半宽（避开腿）
# ── hr48（2026-09-27/28）：电池到货实测 20.0×25.0×72.1、70.8 g（设计按 18 厚）。T01 已打印、BX0/BX1/BZ0/BZ1 冻结不动 → 仓腔 x 19.2 靠 B01 门补：
#   门整体外加厚 DOOR_ADD（外面 −46.8 → −48.8，DOOR_T 冻结值不改），门内面在电池投影里开 DOOR_SLOT_DEPTH 深的槽（槽底 BX0_BAT = −45.4 = 电池后端极限，仓腔 x 20.8）；
#   z ≥ DOOR_SLOT_WALL_Z0 的门顶段留 |y|≤DOOR_SLOT_HALF_W 的槽壁夹住电池后端顶部（门下/中段比 13.9 窄、留不下 1.2 壁 → 槽通宽，段厚 3.4）。
#   B03 后卡舌 / 倒钩 / 门凹槽 / 倒钩窝随门外面后移 DOOR_ADD（tail.py）。电池在仓里的单边可动量 BAT_SHIFT：x 0.4（20.8−20）、y 0.6（导轨 BAT_Y）、z 0.55（73.2−72.1），
#   mechanical_audit M6 按它挪极限位（原来三轴都用 BAT_CLR 0.6）。登记单：docs/measured_2026-09-27/实测登记_电池_Radxa_2026-09-27.md。
DOOR_ADD = 2.0
DOOR_SLOT_DEPTH, DOOR_SLOT_HALF_W, DOOR_SLOT_WALL_Z0 = 1.6, 12.7, 120.5
BX0_BAT = BX0 - DOOR_SLOT_DEPTH                     # −45.4 电池后端极限（门槽底）
BAT_SHIFT = (0.4, 0.6, 0.55)
def battery_box():
    """电池占位实体 zz_battery（零位：x 在 BX0_BAT..BX1 居中、y 0、z 在 BZ0..BZ1 居中）。build / build_fast / qcheck 统一从这里取。"""
    return bx(tuple(P["bat"]), ((BX0_BAT + BX1) / 2, 0.0, (BZ0 + BZ1) / 2))
BAY_FW = 3.5     # 仓前壁厚。原来是 3.0（前表面 -21.6），而髋偏航载体前板的后缘在 -21.3 —— 差 0.3mm 谁也没碰到谁，
                 # 于是 z<144.65 那一整段，装着 66g 电池的仓体只靠顶板那一层吊着。加到 3.5 → 前表面 -21.1，真·体积重叠 0.2
BRACE_Z = (112.0, 118.0)                            # 仓门卡扣咬住的横梁
# 躯干顶板（= 髋偏航载体背板）z：BZR0/BZR1 从髋偏航舵机帧 + XR 推（定义在 pt() 之后，见下）；旧写死 144.65/147.65（12.85 法兰面时）
# 2026-09-17 left_hip_roll True → False：原版策略真实姿态（roulade/ground_pick 两髋同时内收 L+30/R−29.5）里，两腿的髋横滚法兰侧 6704 本体互相压入 132.8 mm³
# （tools/gate/out/policy_steps_2026-09-16 poses.jsonl，31 个姿态），钢件切不了。逐尺寸试算（77 个 L−R 偏航 ≥35° 的姿态，+0.4 隙）：
# 轴承 OD 21 @x19..23 仍 7/77 相交、OD 22 9/77、任何 L01 座壁（r+2）28/77 起；只有 r ≤ 10 的 3 mm 盘（L02 直接坐在法兰上）0/77。
# 而 6704 的 Ø20 内圈毂本来就是为了包住 Ø13.9 法兰（bore ≤17 的轴承毂壁 <1.4）。原版这里是一颗 22×16×4 推力垫（OD 22 @18.5，本试算 0.3 mm³，
# 原版自己也只是贴着过）。结论：髋横滚不设法兰侧副支撑，L02 Ø20 盘直接坐法兰，L01 不做座凸台；6704 只剩膝 2 颗。见 docs/reports/髋横滚轴承取消_2026-09-17.md
RINGS = dict(left_hip_yaw=True, left_hip_roll=True, left_hip_pitch=True, left_knee=True, left_ankle=False,   # 髋横滚已恢复为独立6703开顶座，参数见bearing_rebuild.HR；旧6704不再回装
             neck_pitch=False, head_pitch=False, head_yaw=False, head_roll=False)

def hull(*ms): return trimesh.util.concatenate(list(ms)).convex_hull
_MEMO = {}
def memo(f):
    def g(*a):
        k = (f.__name__,) + a
        if k not in _MEMO: _MEMO[k] = f(*a)
        return _MEMO[k]
    g.__name__ = f.__name__; g.__doc__ = f.__doc__; return g

# ── 原版躯干壳（left_shell / right_shell 外形原样复用，只加我们的安装凸台）────────────
SHELL_BOSS = [(-17.0, 20.0), (1.0, 24.0)]   # 每半壳 2 根安装柱的世界 (x, |y|)。扫出来的：左右壳都合格、Ø5.2 圆周全在壳上、壁厚 2.2、圆周落差 ≤1.2、柱高 8.9/10.7、间距 18.4
SHELL_CLR = 0.5        # 拿壳当刀去削 T01 时的外扩量（壳与躯干之间保证 ≥0.5 间隙）
NECK_CLR = 0.8         # 壳给脖子扫掠让位的外扩量
# 09-14 复审 R3a MAJOR：N01 上排 2 颗角螺丝头（Ø4.4×1.6，沉窝底 −15.3 → 头外面 −16.9，比板外面 −16.3 凸 0.6）原版和 N01 本体一样按 NECK_CLR 0.8 扫 →
# 削面 −17.7，T03 颈部区皮层（原壳 −18.40..−16.10）只剩 0.70（≈48 mm²）。我们改成螺丝头单独按 0.5 扫（trunk.neck_swing_env）：削面 −17.4、皮层 1.0；
# 头↔T03 零位 0.8→0.5、毂打印短一个层高（0.2）壳内移后仍 ≥0.3（mechanical_audit review_r3 按扫掠 min_gap ≥0.3 判）。N01 本体/舵机/N02 仍是 0.8。
NECK_HEAD_CLR = 0.5
BOSS_D, BOSS_PILOT = 5.0, 1.6
DILATE6 = lambda g: ((0, 0, 0), (g, 0, 0), (-g, 0, 0), (0, g, 0), (0, -g, 0), (0, 0, g), (0, 0, -g))

def TW(body): return B[body]["T_world"]
def orig(body, name):
    """原版网格放到零位姿世界坐标"""
    T = [q["T"] for q in B[body]["parts"] if q["mesh"] == name][0]
    m = K.mesh(name); m.apply_transform(TW(body) @ T); return m
_SH = {}
def shell(side):
    """side=+1 左壳 / -1 右壳（原版原始外形）"""
    if side not in _SH: _SH[side] = orig("trunk_base", "left_shell" if side > 0 else "right_shell")
    return _SH[side]
def shell_inner_z(side, x, y, r=BOSS_D / 2 + 0.1):
    """壳在 (x,y) 处的内/外表面 z。内表面取 Ø(2r) 圆周 + 圆心上的**最低**值 —— 曲面不平，
    只按圆心取会让凸台顶把壳顶穿（实测圆周内最大落差 1.14mm）。外表面取最低值判壁厚。"""
    sh = shell(side); zin, zout = [], []
    for (dx, dy) in [(0.0, 0.0)] + [(r * math.cos(a), r * math.sin(a)) for a in np.linspace(0, 2 * math.pi, 16, endpoint=False)]:
        hit = sh.ray.intersects_location(ray_origins=np.array([[x + dx, y + dy, BZR1 + 1.0]]), ray_directions=np.array([[0, 0, 1.0]]))
        zs = np.sort(hit[0][:, 2]) if len(hit[0]) else np.array([])
        assert len(zs) >= 2, f"壳{side} 在 ({x+dx:.1f},{y+dy:.1f}) 打不到两层（凸台圆周越界）"
        zin.append(zs[0]); zout.append(zs[1])
    return float(min(zin)), float(min(zout))
SHELL_SEAT_D, SHELL_SEAT_MARGIN, SHELL_SEAT_MIN_WALL = 5.0, 0.1, 1.2   # 壳柱螺丝坐面锪平：Ø5 平台（同 H02-F07），平台 = 头足印(r≤2.0)最低外表面再下 0.1，切后 Ø5 内剩余壁 ≥1.2
def shell_seat_z(side, x, y, r_head=2.0, r_pocket=SHELL_SEAT_D / 2):
    """壳柱螺丝 (x,y) 处的锪平平台 z 与切后最薄壁。原版壳外表面在这里是弧面（09-13 探针：T02 (−17,20) 孔周落差 0.19、(1,24) 0.63；
    09-15 按 Ø5 足印重量：0.28 / 0.97），Ø4 螺丝头坐上去只有一边受力、越拧越翘。平台取头足印 r≤r_head 圆周上**最低**的外表面 z 再减
    SHELL_SEAT_MARGIN（保证整圈被切到、真平），Ø(2·r_pocket) 口袋里每点剩余壁厚 = 平台 z − 该点内表面 z，最薄要 ≥ SHELL_SEAT_MIN_WALL
    （09-15 实测四处 1.29 / 1.84 / 1.30 / 1.87，不用加内垫台）。返回 (z_plat, wall_min)。"""
    sh = shell(side); zin = {}; zout = {}
    for r in (0.0, 0.8, 1.3, 1.6, 2.0, 2.3, r_pocket):
        for a in np.linspace(0, 2 * math.pi, 36, endpoint=False):
            if r == 0.0 and a > 0: break
            hit = sh.ray.intersects_location(ray_origins=np.array([[x + r * math.cos(a), y + r * math.sin(a), BZR1 + 1.0]]), ray_directions=np.array([[0, 0, 1.0]]))
            zs = np.sort(hit[0][:, 2]) if len(hit[0]) else np.array([])
            assert len(zs) >= 2, f"壳{side} 在 ({x+r*math.cos(a):.1f},{y+r*math.sin(a):.1f}) 打不到两层（锪平口袋越界）"
            zin[(r, a)] = zs[0]; zout[(r, a)] = zs[1]
    z_plat = min(v for (r, a), v in zout.items() if r <= r_head) - SHELL_SEAT_MARGIN
    wall_min = min(z_plat - v for v in zin.values())
    return float(z_plat), float(wall_min)
SHELL_SLIDE = 9.0      # 半壳沿 ±y 直线装上去之前要走的距离。毂尖端退出 N01 背板只要 3.3，但复核实测壳向外挪 0.6..6.5 mm 都会顶到 T01（8 mm 后为 0），
                       # 所以让位扫到 9.0：从 9 mm 以外一路直推都不碰（mechanical_audit 验 0..40 mm）
_SHCUT = {}
def shell_cutter(side):
    """壳外扩 SHELL_CLR、且只保留侧壁区(|y|≥19) —— 用来削我们的躯干；颈部那边反过来由壳让位。

    2026-09-12 起还包含**壳沿 ±y 直线装入的整条扫掠**（壳向外平移 0..SHELL_SLIDE，步长 0.5）：
    T03 右壳的惰轮毂要穿过 N01 的让位孔落到惰轮端面，最后 3.3 mm 只能是纯 y 平移；而壳是个往下收口的碗，
    向外挪 0.6 mm 起碗沿就顶到 T01 的甲板边 (-20,-29,122) 和顶板角 (13.5,-26.6,146)，最大 14 mm³
    （复核实测；现状壳同样走不了这条直线，只是以前可以斜着套进去，assembly.yaml 也一直标着"壳合拢路径未审"）。
    两侧一起做：T01 保持左右对称，T02 也按同一条直线路径装。"""
    if side in _SHCUT: return _SHCUT[side]
    zone = wbox((-60, 19.0, 100), (60, 60, 175)) if side > 0 else wbox((-60, -60, 100), (60, -19.0, 175))
    ps = []
    for k in range(int(round(SHELL_SLIDE / 0.5)) + 1):
        for dv in DILATE6(SHELL_CLR):
            c = shell(side).copy(); c.apply_translation(np.array(dv, float) + (0.0, side * 0.5 * k, 0.0))
            it = inter(c, zone)
            if it is not None and not it.is_empty: ps.append(it)
    _SHCUT[side] = union(*ps); return _SHCUT[side]
# 沿舵机自身轴线(法兰方向为正)的挪动，不影响运动学（frozen.yaml:joint_axes_rule：只允许沿轴平移）。
# ("jaw_soft",1) 头横滚：−1.5 → **−1.7**（09-14 复审 MAJOR）：手册尺寸（T_lo 23）下舵机厚段背面×侧面×接线端的底角离冻结的原版上头壳只有 0.199
# （旧 22.8 时 0.298）；上头壳不能削，舵机沿自身轴 +x 挪 0.2（世界 x：心 −4.9 → −4.7，法兰面 8.1 → 8.3，厚段背 −17.9 → −17.7）→ 实测 min_gap 0.331。
# +0.1 只到 0.265（最近点是斜向的，每挪 0.1 只涨 0.066）。跟着帧走的：N03 A 盘 YRM_A2（11.35 → 11.55）、H02 站位、H01/H03 的舵机让位刀、KO01 CF4 插座窗；
# 关节轴线、N03 B 端（YRM_B_*）、H01 四脚、N04 都不动。Gate 数据 frozen.yaml:SERVO_SHIFT / features.yaml:995 / keepouts.yaml:KO01 CF4 要跟。
SERVO_SHIFT = {("yaw_roll_motion", 0): 5.0, ("jaw_soft", 1): -1.7,
               ("yaw2roll", 0): 4.2, ("bearing_roll", 0): 4.2}
def sfw(body, idx):
    s = B[body]["servos"][idx]; R = TW(body) @ frame(s["center"], s["front"], s["long"])
    d = SERVO_SHIFT.get((body, idx), 0.0)
    if d: R = R @ np.array([[1, 0, 0, -d], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1.0]])   # 法兰朝 +x：-d 让法兰面远离从动件
    return R
def drv_from(child):
    """驱动 child 的舵机（装在父连杆里）的世界帧"""
    par = B[child]["parent"]
    for i, s in enumerate(B[par]["servos"]):
        if s["drives"] == child: return sfw(par, i)
    raise KeyError(child)
def drv_self(body):
    """body 自带的、驱动 body 自身的舵机（从动件是它的父连杆）的世界帧"""
    for i, s in enumerate(B[body]["servos"]):
        if s["drives"] == body + ":self": return sfw(body, i)
    raise KeyError(body)
def pt(R, x, y=0.0, z=0.0): return (R @ np.array([x, y, z, 1.0]))[:3]
BZR0 = float(pt(sfw("trunk_base", 0), XR)[2])   # 144.8：躯干顶板底 = 髋偏航载体背板内面（舵机心 131.5 − XR；旧 144.65）
BZR1 = BZR0 + PLT                                # 147.8：顶板顶（旧 147.65）。电池仓 BZ1=146.9 不动（冻结）
def servo_env(R, extra=0.0): return placed(s288.servo_envelope(extra), R)

# ───────────────────────────── 舵机坐标系里的原语（法兰朝 +x） ─────────────────────────────
def carrier(open_side="-y", ring=False, front=True, rear_t=PLT, walls=("+y", "-y", "+z", "-z"),
            rear_y=None, rear_z=None, wall_depth=None, front_y=None, wall_z=None, front_z=None, upper=None, brg=None):
    """载体：背板(4×M2 拧进舵机背面角孔) + 侧墙(去掉 open_side 让舵机滑入) + 前板(法兰过孔, 朝 open_side 开槽) [+ 法兰侧径向轴承外圈座]。
    背板内面贴舵机**下半段（厚段）**背面（XR=-13.3）。上排两个角孔在薄段（背面 -10.0），板和舵机之间差 3.0（T_lo−T），按 upper 处理：
      "step"  ：侧面开口(±y)——背板上半段（z>2.5）加厚到贴薄段（-10.3），副轴沿开口方向在加厚区开滑槽。舵机侧滑时台阶原样通过。
      "card"  ：顶端开口(±z)——舵机厚端先进、要经过上排孔区，背板不能有凸起；装好后从开口塞一片 CARD_T 厚的垫片卡（spacer_card()），
                卡的上沿有 1mm 唇挂在背板顶边的缺口里定位，再穿螺丝。
      "pillar"：无前板（脖子）——舵机从法兰侧放入，背板上直接长 Ø5 垫柱。
    ring=True（目前只有髋偏航 T01 用，brg 默认 P["brg"]，髋偏航传 P["brg_yaw"]）—— 法兰侧副支撑（原版此处本来就有一颗 22×16×4（舵盘面平面，受力报告 §4 判为推力式夹持），S288 法兰配不了那种夹法，我们改法兰侧径向 6702）：
      · 前板仍在 10.3..12.5，离法兰面 0.5（从动件的毂顶面贴法兰面 13.0，转动件对载体要留 0.5）；
      · 座环 Ø brg.ring_od 从前板底 12.5 长到 13.0+RING_T=17.0；座孔 Ø brg.seat_d 在 12.95..17.5（底部通，轴承从法兰侧推入）；
        12.5..12.95 是内径 brg.lip_d 的挡肩：只压外圈端面，不碰会转的内圈和毂；
      · 舵机侧滑槽（Ø14+1 沿 open_side 横穿前板）只开到法兰面下 SLIDE_CLR（13.4），不再像无环时那样开到 17.0 ——
      （以上站位全部从 S 推，括号数是手册尺寸下的值；旧 12.85 法兰面时各少 0.15）
        否则座环被切成 C 形（09-12 实测只剩 ~12/24 圈）。滑槽在挡肩与座孔顶 0.2 处留一个 open_side 方向 ±30° 的缺口，是设计。
    rear_y/rear_z: 背板覆盖范围；front_y: 前板 y 范围；wall_z: 侧墙沿长边(z)的范围；wall_depth: 墙/前板从背面起的深度。"""
    T, Wd, L = S["T"], S["W"], S["L"]
    xr = XR; zc = S["top"] - L / 2
    x_thin = -T / 2 - CLR                              # -10.3 薄段背面 + 间隙
    fy = Wd / 2 + CLR + Wl
    z_hi, z_lo = S["top"] + CLR + Wl, -(L - S["top"]) - CLR - Wl
    ry = rear_y or (-fy, fy); rz = rear_z or (z_lo, z_hi); fyr = front_y or (-fy, fy); wz = wall_z or (z_lo, z_hi); fzr = front_z or (z_lo, z_hi)
    od = {"+y": (0, 1, 0), "-y": (0, -1, 0), "+z": (0, 0, 1), "-z": (0, 0, -1)}[open_side]
    if upper is None:
        upper = "pillar" if not front else ("card" if open_side in ("+z", "-z") else "step")
    parts = []; cuts = []
    if rear_t > 0.1:
        plate = bx((rear_t, ry[1] - ry[0], rz[1] - rz[0]), (xr - rear_t / 2, (ry[0] + ry[1]) / 2, (rz[0] + rz[1]) / 2))
        if upper == "step":
            z_st = S["step_z"][0] + 0.5           # -6.5：台阶起点。垫块要覆盖整个薄段背面，不然舵机前半截悬空 3.3mm
            hi = bx((x_thin - (xr - 0.1), ry[1] - ry[0], rz[1] - z_st), ((x_thin + xr - 0.1) / 2, (ry[0] + ry[1]) / 2, (rz[1] + z_st) / 2))   # x -13.4..-10.3，z -6.5..顶
            plate = union(plate, hi)
            xa, xb = x_thin + 0.5, xr + 0.02           # 从内腔里（-9.8）切到厚段板面外 0.02（-13.28）：只切加厚区，不切厚段板
            rb = cyl(S["rear_boss_d"] + 2.0, xa - xb, ((xa + xb) / 2, 0, 0), axis="x")
            rb2 = rb.copy(); rb2.apply_translation(np.array(od) * 40)
            plate = diff(plate, hull(rb, rb2))
        elif upper == "card":
            ze = rz[1] if od[2] > 0 else rz[0]        # 开口那一端的板边：切 1mm 深缺口给垫片卡的唇
            plate = diff(plate, bx((rear_t + 0.2, CARD_W + 0.2, 1.0 + 0.01), (xr - rear_t / 2, 0, ze - 0.5 * np.sign(od[2]))))
        else:   # pillar
            plate = diff(plate, cyl(S["rear_boss_d"] + 2.0, 1.0 + 0.01, (xr - 0.5, 0, 0), axis="x"))   # Ø16 深 1 浅让位（副轴端 -13.0，板内面 -13.3）
            ph = (x_thin + 0.0) - xr
            for sy in (1, -1):
                # 垫柱只在上排孔（薄段）那儿长；rear_z 被压低时（N01 的头俯仰载体）那排孔根本不在板上，不能凭空长柱子
                if ry[0] < sy * S["mnt_dx"] < ry[1] and rz[0] < S["mnt_z"][0] < rz[1]:
                    plate = union(plate, cyl(5.0, ph, (xr + ph / 2, sy * S["mnt_dx"], S["mnt_z"][0]), axis="x"))
        parts.append(plate)
    xf = T / 2 + S["flange_h"]                          # 13.0 法兰面
    x_front = xf - 0.5                                  # 12.5：前板底面离法兰面/从动件顶面 0.5（有无座环都一样）
    if wall_depth is not None: x_front = xr + wall_depth
    xw = (xr + x_front) / 2; d = x_front - xr
    ws = [w for w in walls if w != open_side]
    if "+z" in ws: parts.append(bx((d, 2 * fy, Wl), (xw, 0, z_hi - Wl / 2)))
    if "-z" in ws: parts.append(bx((d, 2 * fy, Wl), (xw, 0, z_lo + Wl / 2)))
    c0, c1 = S["conn_z"]
    for sgn, key in ((1, "+y"), (-1, "-y")):
        if key not in ws: continue
        yw = sgn * (fy - Wl / 2)
        for (a, b) in ((c1 + 1.0, wz[1]), (wz[0], c0 - 1.0)):        # 插座区上/下两段
            a, b = max(a, wz[0]), min(b, wz[1])
            if b - a > 1.0: parts.append(bx((d, Wl, b - a), (xw, yw, (a + b) / 2)))
    if front:
        ft = x_front - T / 2 - CLR
        parts.append(bx((ft, fyr[1] - fyr[0], fzr[1] - fzr[0]), (T / 2 + CLR + ft / 2, (fyr[0] + fyr[1]) / 2, (fzr[0] + fzr[1]) / 2)))
    b = P["brg"] if brg is None else brg
    if ring:
        parts.append(cyl(b["ring_od"], xf + RING_T - x_front, ((x_front + xf + RING_T) / 2, 0, 0), axis="x"))   # 座环 12.5..17.0
    m = union(*parts)
    x0s = T / 2 - 1.0                                                  # 9.0：法兰根部以上 1
    x1s = xf + SLIDE_CLR if ring else xf + S["flange_h"] + 1.0         # 有座环只到 13.4；无环沿用 17.0
    fh = cyl(S["flange_d"] + 1.0, x1s - x0s, ((x0s + x1s) / 2, 0, 0), axis="x")
    fh2 = fh.copy(); fh2.apply_translation(np.array(od) * 40)
    cuts += [mount_cut_local(), hull(fh, fh2)]
    if ring:
        cuts.append(cyl(b["seat_d"], RING_T + 0.55, (xf - 0.05 + (RING_T + 0.55) / 2, 0, 0), axis="x"))              # 座孔 12.95..17.5，底部通
        cuts.append(cyl(b["lip_d"], xf - x_front + 0.1, (x_front - 0.1 + (xf - x_front + 0.1) / 2, 0, 0), axis="x"))  # 挡肩内径 12.4..13.0
    return diff(m, *cuts)

CARD_W = 20.0                                          # 垫片卡：宽 20（内腔 20.6）
CARD_T = (-S["T"] / 2 - XR) - 0.1                      # 厚 = 背板内面到薄段背面的缝（T_lo−T+CLR=3.3）− 0.1 = 3.2（旧写死 3.1/缝 3.2）。目前没有载体用 "card" 模式
def spacer_card():
    """顶端开口载体用的垫片卡（舵机坐标）：贴在背板内面与舵机薄段背面之间，z 4.6..12.3，上沿 1mm 唇挂进背板顶边缺口。两孔 Ø2.4。"""
    z0, z1 = 4.6, S["top"] + CLR + Wl                  # 12.3 = 背板顶边
    body = bx((CARD_T, CARD_W, z1 - z0), (XR + CARD_T / 2, 0, (z0 + z1) / 2))
    lip = bx((PLT, CARD_W, 1.0), (XR - PLT / 2, 0, z1 - 0.5))
    m = union(body, lip)
    boss = cyl(S["rear_boss_d"] + 2.0, 30, (XR, 0, 0), axis="x")          # 下沿中间让开副轴 Ø14（卡和副轴在同一 x 区间）
    return diff(m, boss, *[cyl(2.4, 30, (XR, sy * S["mnt_dx"], S["mnt_z"][0]), axis="x") for sy in (1, -1)])

def card_cut_local():
    """顶端开口载体的整条滑道：背板内面 → 薄段背面+间隙，y ±10.3，从板底到板顶+1（舵机厚端要从这里滑过，垫片卡最后也塞在这里），
    再加上唇缺口区。后 union 上去的梁/腹板会填进来，build 最后再挖一次"""
    x_thin = -S["T"] / 2 - CLR
    z0, z1 = -(S["L"] - S["top"]) - CLR - Wl, S["top"] + CLR + Wl + 1.0
    chan = bx((x_thin - XR, CARD_W + 0.6, z1 - z0), ((x_thin + XR) / 2, 0, (z1 + z0) / 2))
    notch = bx((PLT + 0.2, CARD_W + 0.2, 2.0), (XR - PLT / 2, 0, z1 - 1.0 - 0.5))          # z 11.3..13.3
    return union(chan, notch)
def card_cut(R): return placed(card_cut_local(), R)

def mount_cut_local():
    """4 个角孔 Ø mnt_hole_d，从背板外面一直打到舵机背面（穿板、垫柱、垫片卡、以及后面 union 上去的任何东西）。
    上排两孔（薄段，堆叠 3+3.0+0.3=6.3）从板外再沉 Ø m2_cbore_d×1.0：M2×8 进舵机 2.7（不沉只有 1.7）"""
    holes = s288.mount_holes(20, (XR - 3.0, 0, 0))
    cb = [cyl(P["m2_cbore_d"], 10.0, (XR - PLT - 5.0 + 1.0, sy * S["mnt_dx"], S["mnt_z"][0]), axis="x") for sy in (1, -1)]   # 到 x=XR−PLT+1=-15.3，沉 1.0
    return union(holes, *cb)
def mount_cut(R): return placed(mount_cut_local(), R)

def driven(ring=False, d=26.0, t=PLT, cbore=False, brg=None):
    """从动件舵盘板（+ 轴承内圈上的毂，毂径 brg["hub_d"]，默认 6704 的 Ø19.85）。返回 (实体, 孔, 板外表面 x, 板内表面 x)。"""
    b = P["brg"] if brg is None else brg
    x0 = S["T"] / 2 + S["flange_h"]
    if ring:
        xi = x0 + RING_T + 0.5
        hub = cyl(b["hub_d"], xi - x0 + 0.01, (x0 + (xi - x0) / 2, 0, 0), axis="x")
        solid = union(hub, cyl(d, t, (xi + t / 2, 0, 0), axis="x"))
    else:
        xi = x0; solid = cyl(d, t, (xi + t / 2, 0, 0), axis="x")
    xe = xi + t
    holes = s288.horn_holes(xe - x0 + 2, ((x0 + xe) / 2, 0, 0))
    if cbore:
        for k in range(S["horn_n"]):
            a = math.radians(k * 360 / S["horn_n"])
            holes = union(holes, cyl(P["m2_cbore_d"], P["m2_head_h"] + 0.01, (xe - P["m2_head_h"] / 2, S["horn_r"] * math.cos(a), S["horn_r"] * math.sin(a)), axis="x"))
    return solid, holes, xe, xi

def stl_stats(path):
    """直接读回二进制 STL，按精确 float32 坐标建边：洞边(只被一个面用的边)=0 才是真闭合。
    trimesh.load() 默认会 merge_vertices，容差会把相邻顶点焊在一起、反而制造出假的"非闭合/散件"，所以这里不用它。"""
    import struct
    from collections import Counter
    with open(path, "rb") as f:
        n = struct.unpack("<I", f.read(84)[80:84])[0]; data = f.read(50 * n)
    tris = np.frombuffer(data, dtype=np.dtype([("n", "<3f4"), ("v", "<3,3f4"), ("a", "<u2")]), count=n)
    _, inv = np.unique(tris["v"].reshape(-1, 3), axis=0, return_inverse=True)
    F = inv.reshape(-1, 3)
    deg = int(((F[:, 0] == F[:, 1]) | (F[:, 1] == F[:, 2]) | (F[:, 0] == F[:, 2])).sum())
    c = Counter(map(tuple, np.sort(np.vstack([F[:, [0, 1]], F[:, [1, 2]], F[:, [2, 0]]]), axis=1)))
    return n, deg, sum(1 for v in c.values() if v == 1), sum(1 for v in c.values() if v > 2)

def keep_main(m, tag="", fill_cavities=False):
    """清理微小正体积碎片，保留内向的空腔壳；删空腔壳等于把空腔填实。
    fill_cavities=True：件收尾时把埋在壁内的封闭空腔填实（hr13 H01：yrm 扫掠刀 + 座环并集在 A 座后方留下 1 个 35 mm³ 的
    封闭空腔，埋在壁内 1.2 mm，切片会留空且壁只剩 1.2/1.7）。只在**确认空腔不在任何运动件/刀路上**的件用（试算见 hr13_local_test.py），
    默认仍保留——静默填实会盖住"本该是通道"的设计错误。"""
    comps = m.split(only_watertight=False)
    if len(comps) <= 1: return m
    big = [c for c in comps if c.volume > 30.0]
    cav = [c for c in comps if c.volume < -1e-3]
    small = [c for c in comps if -1e-3 <= c.volume <= 30.0]
    if small: print(f"    [{tag}] 丢弃碎片 {len(small)} 个，最大 {max(c.volume for c in small)/1000:.3f} cm³")
    if cav and fill_cavities:
        print(f"    [{tag}] 填实内部空腔 {len(cav)} 个，共 {-sum(c.volume for c in cav):.3f} mm³"); cav = []
    elif cav: print(f"    [{tag}] 保留内部空腔 {len(cav)} 个，共 {-sum(c.volume for c in cav):.3f} mm³")
    kept = big + cav
    if not big: raise ValueError(f"{tag}: 没有大于 30 mm³ 的实体")
    return trimesh.util.concatenate(kept) if len(kept) > 1 else kept[0]

def wbox(lo, hi):
    """世界坐标轴对齐方块：lo/hi 各三个坐标"""
    lo, hi = np.array(lo, float), np.array(hi, float)
    return bx(hi - lo, (lo + hi) / 2)

def minkowski_box(m, lo, hi):
    """连续长方体膨胀；保留凹形台阶，不拿整体凸包替代滑入走廊。"""
    from assembly_audit import solid
    import manifold3d as M
    result = solid(m).minkowski_sum(solid(wbox(lo, hi)))
    if result.status() != M.Error.NoError:
        raise ValueError(f"连续让位体失败: {result.status()}")
    mesh = result.to_mesh64()
    return trimesh.Trimesh(vertices=np.asarray(mesh.vert_properties)[:, :3],
                           faces=np.asarray(mesh.tri_verts), process=False)

def clean_print_topology(m):
    """L04的明确实体：精确焊合STL坐标后规范化，消除导出共边歧义。
    旧14条非流形共边使左右读回实体对称差0.2805mm³。先在左件完成
    规范化再镜像；1e-5仅是后续简化公差，不是原歧义面的位移上界。
    """
    from assembly_audit import solid
    vertices, inverse = np.unique(np.asarray(m.vertices, dtype=np.float32).astype(np.float64),
                                  axis=0, return_inverse=True)
    exact = trimesh.Trimesh(vertices=vertices, faces=inverse[np.asarray(m.faces)], process=False)
    result = solid(exact).simplify(1e-5)
    mesh = result.to_mesh64()
    out = trimesh.Trimesh(vertices=np.array(mesh.vert_properties[:, :3], copy=True),
                          faces=np.array(mesh.tri_verts, copy=True), process=False)
    # hr09：simplify 后仍可能留下成对的非流形边（hr08 H03 在 (41.5,0,220.2) 有 2 条，trimesh 判非水密）；
    # 再做一次 float32 焊合→manifold 往返，实测消除且不改体积。
    v2, inv2 = np.unique(np.asarray(out.vertices, dtype=np.float32).astype(np.float64), axis=0, return_inverse=True)
    mesh2 = solid(trimesh.Trimesh(vertices=v2, faces=inv2[np.asarray(out.faces)], process=False)).to_mesh64()
    return trimesh.Trimesh(vertices=np.array(mesh2.vert_properties[:, :3], copy=True),
                           faces=np.array(mesh2.tri_verts, copy=True), process=False)

def servo_slide(R, direction=(0, 0, -1), clearance=0.2):
    """真实舵机外形沿世界 direction 平移60mm，另留0.2mm名义装配余量。"""
    delta = np.asarray(direction, float) * 60.0
    return minkowski_box(placed(s288.servo_mesh(), R),
                         np.minimum(delta, 0) - clearance, np.maximum(delta, 0) + clearance)

def bearing_driven_clearance(R, brg=None):
    """最终并集重新挖出整环：保留毂，轴承外端至盘留0.25mm。默认 6704（Ø27.5/Ø19.85）。"""
    b = P["brg"] if brg is None else brg
    xc = S["T"] / 2 + S["flange_h"] + 2.025         # 环 x 12.8..17.25 = 法兰面−0.2 .. 轴承外端(法兰面+4)+0.25（旧 14.875 中心是按 12.85 法兰面写死的）
    ring = diff(cyl(b["od"] + 0.5, 4.45, (xc, 0, 0), axis="x", sections=128),
                cyl(b["hub_d"], 4.6, (xc, 0, 0), axis="x", sections=128))
    return placed(ring, R)


# ───────────────────────────── 照原版外形重雕（v2）：原版网格当毛坯 ─────────────────────────────
# 原版件是围着 XL330 画的。XL330 沿轴 ±14.5，S288 只有 ±13.0（薄 3.0；旧实测 ±12.85）：
#   · 从动面：原版舵盘板内表面在局部 x=+14.5，S288 法兰面在 +13.0 → 每个从动接口补一片 1.5 厚的盘（driven_patch）
#   · 背板面：原版靠拧 XL330 侧面/背面孔，S288 只有背面 4 角孔(16×30, 深 3.0 MAX) → 背板要补厚到贴住 XR
#   · 原版给 XL330 舵盘留的 4 孔(r=6)要先填掉，再打我们的 6 孔(分度圆 2×horn_r=Ø10.5，09-13 定案)
XLF = 14.5          # XL330 法兰面/背面在舵机局部坐标的 x（原版所有接口面都从这里起）
def fill_cyl(center, dia, axis, t, off=0.0):
    """填掉原版一个孔：沿 axis 的实心圆柱"""
    return cyl(dia, t, tuple(np.array(center) + off * np.array({"x": (1, 0, 0), "y": (0, 1, 0), "z": (0, 0, 1)}[axis])), axis=axis)

def driven_patch(R, d, extra=0.2):
    """补上 S288 法兰面(x=13.0) 到 原版舵盘板内表面(x=14.5) 之间那 1.5mm（旧 12.85 → 1.65）"""
    x0 = S["T"] / 2 + S["flange_h"]
    t = XLF - x0 + extra          # 只往**外**多伸 extra 去和原版件重叠，绝不往舵机那边啃
    return placed(cyl(d, t, (x0 + t / 2, 0, 0), axis="x"), R)

def horn_cbore(R, x_out, depth, d_cb=None, sign=1):
    """从动件外表面 x_out 往里沉 Ø m2_cbore_d × depth 的 6 个头窝（d_cb=None → P["m2_cbore_d"]）：叠厚太大时用来把螺丝头往里坐，
    好让标准长度的 M2 咬进法兰 2~3mm（法兰螺纹深 3.0 MAX，长了会顶底）。
    sign=+1：外表面在**法兰**那一侧，窝从 x_out 往 -x 沉；sign=-1：外表面在**惰轮**那一侧，窝从 x_out 往 +x 沉。"""
    d_cb = P["m2_cbore_d"] if d_cb is None else d_cb
    return placed(union(*[cyl(d_cb, depth + 0.02, (x_out - sign * (depth / 2 - 0.01), S["horn_r"] * math.cos(a), S["horn_r"] * math.sin(a)), axis="x")
                          for a in np.linspace(0, 2 * math.pi, S["horn_n"], endpoint=False)]), R)

# ───────────── 背面惰轮（副轴）侧的双支撑原语 ─────────────
# S288 两端是同一个接口（手册尺寸图 p2，见 s288.py:idler_center_d 的注释）：Ø14×3 凸台 + 6-Ø1.7⌄3.0 @ r=horn_r + 中心一颗十字螺钉。
# （图上那个 Ø10.50 是 6 孔的分度圆、不是中心环 —— 09-12 逐像素量图确认；09-13 实物定案 horn_r=5.25，见 s288.py。）
# 背面那个盘是**自由转动的惰轮**，接上从动件它就跟着转 —— 所以凡是从动件够得到惰轮的关节，都应该两端一起接，
# 让弯矩由「法兰 + 惰轮」两个轴颈分担，而不是全压在法兰这一个塑料轴颈上。
# 惰轮和输出独立转动：载体固定惰轮不必锁输出，但也不能给从动件增加支撑。
# 必须按刚接拓扑区分从动件–惰轮连接、壳体夹紧和材料接近；开窗/叉架可改变可达性。
def idler_hub(R, x_out, d=None):
    """从动件接惰轮的毂（舵机局部）：从从动件自己的贴合面 x_out 一路补到惰轮端面 -13.0，做成面接触。
    x_out 必须比惰轮端面更靠外（更负），否则说明这一侧根本不是从动件、或者件已经压进惰轮里了。"""
    x1 = s288.x_idler_face()
    d = S["rear_boss_d"] + 0.2 if d is None else d
    if x_out > x1 - 0.05:
        raise ValueError(f"idler_hub: 贴合面 {x_out:.2f} 没有比惰轮端面 {x1:.2f} 更外，接不上")
    # 毂壁：6 个 Ø idler_hole_d 过孔的外缘到毂外缘至少 0.8（两条挤出线）。09-13 分度圆定案 5.25 后毂已由 Ø14 放大到 Ø15
    # （neck.NP_IDL_HUB_D / trunk.SHR_IDL：孔外缘 6.55 → 毂边 0.95），N01 让位孔 neck.NK_IDLER_BORE 同步 15→16。
    # 毂放大是安全的：手册 step_z=(-7,-13.5)，厚段背平面 -13.0 只在 z≤-13.5，斜坡在 z=-7.5 处 x≈-10.2，Ø15 毂（z ±7.5）碰不到本体。
    lig = d / 2 - (S["horn_r"] + S["idler_hole_d"] / 2)
    assert lig >= 0.8 - 1e-9, f"idler_hub: 毂 Ø{d} 在 horn_r={S['horn_r']} 下孔外缘到毂边只剩 {lig:.2f} < 0.8"
    return placed(cyl(d, x1 - x_out, ((x_out + x1) / 2, 0, 0), axis="x"), R)

def idler_cut(R, d=40.0, center_x0=None):
    """惰轮 6 孔 + 中心让位，从惰轮端面往**外**(-x) 打穿 d（horn_cut 的背面镜像）。

    center_x0：中心让位孔往外**只开到这个 x**（舵机局部）为止；默认 None = 和 6 孔一起打穿。
    从动件的外表面就是整鸭外观面时必须给（T03 躯干右壳）—— 不给的话中心 Ø5 会在躯干右侧壳上开一个洞，
    而原版右壳那一块是实心的：2026-09-12 对 orig("trunk_base","right_shell") 逐角射线实测，
    颈俯仰轴 r≤7.1 范围内外表面是一整块 y=-18.4000 的平面、内壁 -16.1000（36 个方位 min=max），
    上面只有 4×Ø2.2@r6 的 XL330 惰轮孔，没有中心孔。"""
    x0 = s288.x_idler_face()
    m = s288.idler_holes(d, (x0 - d / 2 + 1.0, 0, 0))
    if center_x0 is not None:                      # 把中心孔在 center_x0 以外的那一段砍掉
        lo = x0 - d - 2.0
        m = diff(m, cyl(S["idler_center_d"] + 0.02, center_x0 - lo, ((lo + center_x0) / 2, 0, 0), axis="x"))
    return placed(m, R)

def idler_bore(R, d, x_out=-25.0):
    """载体背板上让**从动件的毂**穿过去的同轴通孔：从 x_out 一直开到惰轮端面 -13.0。

    载体抱着舵机壳体，须避开已连接从动件的旋转毂；再把该毂与载体刚接才会锁死。
    单有端面材料接近不能证明刚接或锁死，见 tools/cad/idlercheck.py 的筛查边界。
    所以这里只能是纯让位孔。孔径同时受两头夹：> 毂径 + 装配间隙，且 < 上排垫柱内缘离轴的距离。"""
    x1 = s288.x_idler_face()
    return placed(cyl(d, x1 - x_out, ((x_out + x1) / 2, 0, 0), axis="x"), R)

def idler_keep(R, d=None, t=8.0):
    """从让位刀里挖掉的同轴柱：惰轮端面**以外**、Ø d 以内是贴合区（跟着从动件一起转），不需要任何让位。

    为什么非挖不可：servo_envelope 的背面是按最保守的台阶 (2.0,-1.0) 画的，还把副轴凸台放大成 Ø16、
    往外多留 1.0 —— 它一直伸到 x=X_ENV_BOSS=-14.0，再经 sweep_of(grow=0.3~0.4) 就到 -14.3/-14.4，
    正好把 1.5 厚的贴合毂削掉大半。让位体保守是对的（台阶只认保守面），但贴合区不该被它管：
    那一段里跟从动件发生相对运动的只有惰轮本身，而惰轮就是要被抓住的东西。"""
    d = S["rear_boss_d"] if d is None else d
    x0 = s288.x_idler_face()
    return placed(cyl(d, t, (x0 - t / 2, 0, 0), axis="x"), R)

def flange_relief(R, c=CLR):
    """给 S288 法兰（Ø14 凸出 3.0）让位：Ø(flange_d+2c) × (flange_h+c)，从法兰根部下 c 到法兰面。原版件是照 XL330 画的，法兰这一圈会啃到 S288；
    servo_envelope 故意不挖法兰侧（从动盘要贴在法兰面上），所以单独挖这一圈。
    c 默认 CLR=0.3 → Ø14.6（09-13：法兰按手册 Ø14 后，旧 c=0.2 的 Ø14.4 对 Ø14 只剩 0.2 径向，且本机竖直内轮廓打出来缩 0.15/边（tolerances.yaml TC02），
    按"法兰让位孔 ≥ 14+2·clr" 统一放到 Ø14.6；旧 13.9 法兰 + Ø14.4 时是 0.25 径向）。从动件与法兰同转，这一圈只管装得进、不管间隙。"""
    return placed(cyl(S["flange_d"] + 2 * c, S["flange_h"] + c, (S["T"] / 2 + (S["flange_h"] - c) / 2, 0, 0), axis="x"), R)

def horn_cut(R, d=40.0):
    """我们的 6 孔 + 中心 Ø5，从法兰面往外打穿"""
    x0 = S["T"] / 2 + S["flange_h"]
    return placed(s288.horn_holes(d, (x0 + d / 2 - 1.0, 0, 0)), R)

def back_pad(R, dz, dy=13.0, x_in=None, x_out=-16.0):
    """背板加厚块（舵机局部）：从 x_out 补到 x_in（默认 XR），覆盖 z 区间 dz、宽 dy"""
    x_in = XR if x_in is None else x_in
    return placed(bx((x_in - x_out, dy, dz[1] - dz[0]), ((x_in + x_out) / 2, 0, (dz[0] + dz[1]) / 2)), R)

def silhouette(O, axis, extra_circles=(), pad=0.0, span=None):
    """把原版件沿 axis（0=x,1=y,2=z）压扁成 2D 轮廓再拉伸回实体：用来把我们补的等厚背板裁成原版的圆润外形。
    自己做投影而不用 trimesh.path.polygons.projected —— 后者的 2D 基向量是任意的，对不上世界坐标。
    extra_circles: [(u, v, r)] 在投影平面上补的圆。span=(lo,hi) 沿 axis 的拉伸范围。"""
    from shapely.geometry import Polygon, Point
    from shapely.ops import unary_union
    uv = [i for i in (0, 1, 2) if i != axis]
    tri = O.triangles[:, :, uv]
    e1, e2 = tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0]
    ar = np.abs(e1[:, 0] * e2[:, 1] - e1[:, 1] * e2[:, 0]) / 2      # numpy 2 取消了 2D cross
    polys = [Polygon(t) for t, a in zip(tri, ar) if a > 1e-6]
    shape = unary_union(polys).buffer(0.3).buffer(-0.3)               # 闭运算：补掉三角化留下的针孔
    if shape.geom_type == "MultiPolygon": shape = max(shape.geoms, key=lambda g: g.area)
    if shape.interiors:                                                # 丢掉 <4mm² 的碎孔，保留真正的减重孔
        big = [r for r in shape.interiors if Polygon(r).area >= 4.0]
        shape = Polygon(shape.exterior, big)
    if pad: shape = shape.buffer(pad)
    if extra_circles:
        shape = unary_union([shape] + [Point(u, v).buffer(r, resolution=32) for (u, v, r) in extra_circles])
    lo, hi = span if span else (O.bounds[0][axis] - 5, O.bounds[1][axis] + 5)
    m = trimesh.creation.extrude_polygon(shape, hi - lo)          # 局部 (u, v, w=沿 axis)
    M = np.eye(4)
    M[:3, :3] = 0.0
    M[uv[0], 0] = 1.0; M[uv[1], 1] = 1.0; M[axis, 2] = 1.0; M[axis, 3] = lo
    m.apply_transform(M)
    if m.volume < 0: m.invert()                                    # 坐标置换是镜像变换，会把面翻过来
    return m

BSHELL_EPS = 1e-3
def back_shell(R, t=3.0, grow=0.0):
    """贴着 S288 背面台阶轮廓的**等厚**背板（自动含副轴凸台让位）：让位体往 -x 挪 t 再减掉让位体本身。
    好处是薄段/厚段处板厚都是 t，4 个角孔的螺丝长度一样（t + 3.0 咬进舵机），不用垫片卡也不用沉孔。

    09-13：内腔按 grow−BSHELL_EPS（小 0.001）做，调用方 union 之后照旧再减一次 servo_env(grow) —— 最终内腔面只来自那一刀，
    板的内表面不再与让位体严格共面。手册尺寸下让位体副轴面落到 X_ENV_BOSS=-14.0 这种"整数"站位后，L04 里（原版板 + 背板 + 让位刀
    三者共面/共柱面）manifold 输出经 float32 焊合仍剩 1 条非流形边，export 的 L04 严格断言不过；退 0.001 后 0 退化 0 非流形（09-13 实测，
    对 L01/L03/L04 的口袋尺寸无影响：外表面只薄 0.001，沉孔坐面按 x_bshell_out 名义值算差 0.001）。"""
    e = s288.servo_envelope(grow - BSHELL_EPS)
    e2 = e.copy(); e2.apply_translation((-t, 0, 0))
    return placed(diff(union(e, e2), e), R)

def wide_y(m, dy):
    """沿舵机局部 y（本体宽度方向）加宽：union(m, m+dy, m-dy)。
    把 back_shell 那顶"帽子"向两侧延成裙墙用 —— 裙墙内表面照样跟着舵机侧面走。"""
    a = m.copy(); a.apply_translation((0, dy, 0))
    b = m.copy(); b.apply_translation((0, -dy, 0))
    return union(m, a, b)

def flange_slot(R, od, x1=None):
    """法兰侧滑槽：舵机侧滑装配时 Ø14 法兰要从 od 方向横穿过某块板。
    carrier() 内部本来在它自己的前板上开过这条槽，但后 union 进来的板（例如原版腹面甲板）会把槽填回去。
    x1：刀沿 +x 只开到这里（舵机局部）。默认 17.0（= 法兰面+flange_h+1，旧行为）；载体带座环时传 x_flange_face()+SLIDE_CLR，别把座环切成 C 形。"""
    T = S["T"]; x0 = T / 2 - 1.0
    x1 = T / 2 + 2 * S["flange_h"] + 1.0 if x1 is None else x1
    fh = cyl(S["flange_d"] + 1.0, x1 - x0, ((x0 + x1) / 2, 0, 0), axis="x")
    fh2 = fh.copy(); fh2.apply_translation(np.array(od, float) * 40)
    return placed(hull(fh, fh2), R)


def seat_ring_clearance(R, brg=None):
    """从动件给**载体座环 + 轴承**让位：环 Ø(ring_od+2·SEAT_CLR) 减毂 Ø hub_d，x 12.8..17.5（盘面）（旧 12.65..17.35）。
    比 bearing_driven_clearance 大一圈（那把刀只让轴承本体，不让座环壁）。髋偏航 L01 用它，原版杯顶那两片 XL330 耳朵在这一段被切掉。"""
    b = P["brg"] if brg is None else brg
    xf = S["T"] / 2 + S["flange_h"]
    x0, x1 = xf - 0.2, xf + RING_T + 0.5          # 轴向切到 17.5 = Ø26 盘面（旧 17.35）：09-12 复审发现只切到盘面下 0.2 时那层原版杯顶实心料
    #                                                会先于盘面挡住外圈（轴向浮动实测 0.05..0.30 而非 0.05..0.5），切干净让停止面只落在盘上
    ring = diff(cyl(b["ring_od"] + 2 * SEAT_CLR, x1 - x0, ((x0 + x1) / 2, 0, 0), axis="x", sections=128),
                cyl(b["hub_d"], x1 - x0 + 0.2, ((x0 + x1) / 2, 0, 0), axis="x", sections=128))
    return placed(ring, R)

def seat_ring_zone(R, brg=None):
    """载体座环占的整个圆柱（含轴承与挡肩）：Ø ring_od，x 12.5..17.0（旧 12.35..16.85）。给邻件做反向扫掠让位用（L02 绕横滚轴）。"""
    b = P["brg"] if brg is None else brg
    xf = S["T"] / 2 + S["flange_h"]
    return placed(cyl(b["ring_od"], RING_T + 0.5, (xf - 0.5 + (RING_T + 0.5) / 2, 0, 0), axis="x", sections=96), R)
def brg_ring(R, od_extra=4.0, back=1.5):
    """载体侧的 6704ZZ(20×27×4) 外圈座，套在法兰外面。back: 往舵机方向多伸一点，好和原版壳体连上。
    返回 (实体, 要减掉的孔)"""
    xf = S["T"] / 2 + S["flange_h"]
    solid = cyl(P["brg"]["od"] + od_extra, RING_T + back, (xf + (RING_T - back) / 2, 0, 0), axis="x")
    cut = union(cyl(P["brg"]["seat_d"], RING_T + 0.2, (xf + RING_T / 2, 0, 0), axis="x"),
                cyl(S["flange_d"] + 1.0, RING_T + 2, (xf + RING_T / 2, 0, 0), axis="x"))
    return placed(solid, R), placed(cut, R)

def ring_boss(O, R, t=2.5, od_extra=4.0, rim=(10.0, 12.5)):
    """把 6704 外圈座**融进原版壳体的口沿**，而不是外挂一根圆筒。
    做法：取原版壳体在舵机局部 x∈rim 的那一圈料，和法兰外的轴承环取凸包 → 平滑过渡的凸台；
    再挖掉舵机让位体和内腔，只留一层壁。原版件在法兰这一侧本来一点材料都没有
    （实测：膝舵机局部 x>-12.5、半径 13~16 全空），所以这层过渡是必须自己加的。"""
    xf = S["T"] / 2 + S["flange_h"]
    Ol = O.copy(); Ol.apply_transform(np.linalg.inv(R))                     # 原版件 → 舵机局部
    band = inter(Ol, bx((rim[1] - rim[0], 200, 200), ((rim[0] + rim[1]) / 2, 0, 0)))
    if band is None or band.is_empty: raise ValueError("原版壳体在 rim 区间没有料")
    ring = cyl(P["brg"]["od"] + od_extra, RING_T, (xf + RING_T / 2, 0, 0), axis="x")
    boss = hull(band, ring)
    # 掏空：内表面 = 凸包整体沿 -x 缩一层? 用"凸包减去(凸包沿法向内缩)"太贵，直接减舵机让位体 + 一个内腔柱
    boss = diff(boss, cyl(P["brg"]["od"] + od_extra - 2 * t, xf + RING_T - rim[0] + 2, ((rim[0] + xf + RING_T) / 2 - 1, 0, 0), axis="x"))
    boss = union(boss, ring)                                                # 环本身实心，后面再挖座
    # 09-14 复审 MAJOR（M7）：座孔 Ø seat_d 只从 xf−0.1 起，它以内（x<12.9）凸包/原版毛坯的料整环面对着会转的 L02 毂端面(13.0) 与 6704 内圈端面，
    # 实测 L01 在 x 12.5 r 9.5/11 有 9/7 个方位有料、离毂端面/内圈只有 0.095（膝 L03 同位置本来无料）。
    # 改：同 carrier(ring=True) 的挡肩做法——再挖一刀 Ø lip_d（只压外圈端面，r<12 让到 xf−1.0，与前板同样 ≥0.5），x xf−1.0..xf+0.05（0.05 与座孔重叠避免共面）。
    lip_x0 = xf - 1.0
    cut = union(cyl(P["brg"]["seat_d"], RING_T + 0.2, (xf + RING_T / 2, 0, 0), axis="x"),
                cyl(S["flange_d"] + 1.0, RING_T + 4, (xf + RING_T / 2 - 1, 0, 0), axis="x"),
                cyl(P["brg"]["lip_d"], (xf + 0.05) - lip_x0, ((lip_x0 + xf + 0.05) / 2, 0, 0), axis="x"))   # 挡肩内径 12.0..13.05
    return placed(boss, R), placed(cut, R)

def sweep_of(m, body, lo, hi, n=7, grow=0.0, mink=False):
    """把 m 绕 body 的关节轴从 lo 转到 hi，取并集 —— 用来给邻件让位。
    grow：让位余量。默认 DILATE6（6 个轴向平移副本）——在**凸角**处斜向只有 grow/√2、角点更少（09-14 复审 M2/M3/M4：膝环座端面外缘、
    踝副轴凸台盘边、踝背板顶角三处让位为 0）。mink=True：先对 m 做一次 minkowski_box(±grow) 连续膨胀再转（neck_swing_env 同法），
    转任何角度都包住半径 grow 的球 → 各向 ≥grow；副本数 n 而不是 7n，三角形总数反而更少（L04 2 万面 → 膨胀后 7.7 万面，7.9 s）。"""
    T = TW(body); ax, og = T[:3, 2], T[:3, 3]
    if grow and mink:
        m = minkowski_box(m, (-grow,) * 3, (grow,) * 3); grow = 0.0
    ps = []
    for a in np.linspace(lo, hi, n):
        c = placed(m, rot(math.radians(float(a)), ax, og))
        if grow:
            for dv in DILATE6(grow)[1:]:
                c2 = c.copy(); c2.apply_translation(dv); ps.append(c2)
        ps.append(c)
    return union(*ps)

def mnt_cut2(R, pairs=(0, 1), cb_x0=None, cb_d=0.0):
    """角孔 Ø mnt_hole_d 打穿 + 可选 Ø m2_cbore_d 沉孔（从背板外表面 cb_x0 往里 cb_d 深，让 M2×6 正好咬进舵机 3.0）。
    pairs: 0=上排(z=+7.5) 1=下排(z=-22.5)"""
    zs = [S["mnt_z"][i] for i in pairs]
    hs = [cyl(S["mnt_hole_d"], 40.0, (-20.0, sy * S["mnt_dx"], z), axis="x") for sy in (1, -1) for z in zs]
    assert cb_x0 is None or cb_d > 0, f"mnt_cut2: 沉孔深 cb_d={cb_d} 必须 >0（09-13 曾因 x_bshell_out 相减写反得负值、沉孔被静默跳过）"
    if cb_d > 0 and cb_x0 is not None:
        hs += [cyl(P["m2_cbore_d"], cb_d + 0.02, (cb_x0 + cb_d / 2, sy * S["mnt_dx"], z), axis="x") for sy in (1, -1) for z in zs]
    return placed(union(*hs), R)

# ── PH2.0 插座让位（2026-09-20 背插模型：s288.S.conn_z / plug_* / wire_out / wire_clr / plug_y_in）。舵机局部系，一颗舵机两个插座位（±y）。
#    插头插到底与厚段背面 x=plug_top_x=−13 齐平；线从插头顶直立翘起 wire_out=4.0（2026-09-21 用户实物：原配线硬，≈4）+ wire_clr 0.4 → 每个插座位的让位体：
#      A 插头顶/线翘区：x [−17.4, −12.8] × |y| [W/2−plug_y_in, W/2+plug_clr] = [5.0, 10.5] × z conn_z ± plug_clr = [−14.4, −4.2]
#        （往机身里多进 0.2：那里 servo_env 早已挖空，只为不留薄膜）
#    出线方式按载体分（CONN_MODE；keepouts.yaml:KO01.availability_table 与 tools/cad/asmcheck.py 同表）：
#      "window"：背板整个开窗（x 到 CONN_WINDOW_X=−18.5，盖住 3 mm 背板 + 原版蒙皮/座特征），线从背板**外面**走，插头装好舵机后再插。
#                背板外面的邻件在 A 区（含相对运动扫掠）不许有料：L07 后臂 / N02 −y 脸颊 / T03 壳各自减掉 A 的扫掠（见 legs/neck/trunk）。
#      "free"  ：那一侧没有任何件盖住插座（登记用，不出刀）。
#      "none"  ：那一侧不用：被冻结件挡住（髋偏航外侧：壳柱 (1,24) 正压在插座足印上），或线翘区被邻件扫过（踝后口：L07 后臂 +90° 扫过；
#                颈俯仰 −y 口：T03/T01 在 +35..60° 扫过）—— 这几颗只能当链尾 / 分支根（harness.yaml:HB01）。
#      "pocket"（09-20 版：背板内面挖 2 深让位槽 + B 侧出线槽，插头装舵机前先插）**09-21 作废**：线翘 4.4 > 3 mm 背板兜不住。代码路径保留，无载体使用。
#    旧模型（09-11 侧插：5.5×8.8 走廊从机身侧面外伸 8、CONN_PULL 逐颗走廊长）整套作废。
CONN_WINDOW_X = -18.5
CONN_MODE = {("trunk_base", 0): {-1: "window", 1: "none"},   ("trunk_base", 1): {1: "window", -1: "none"},   # 内侧口开窗进顶板口袋；外侧口压在壳柱下
             ("yaw2roll", 0): {1: "free", -1: "free"},       ("bearing_roll", 0): {1: "free", -1: "free"},    # L01 背板到不了插座区（只削 0.3/2.9 mm³ 板角）
             ("upper_leg_left", 0): {1: "window", -1: "window"}, ("upper_leg_left", 1): {1: "window", -1: "window"},
             ("upper_leg_right", 0): {1: "window", -1: "window"}, ("upper_leg_right", 1): {1: "window", -1: "window"},
             ("leg", 0): {1: "window", -1: "none"},          ("leg_2", 0): {-1: "window", 1: "none"},        # 踝 = 链尾只用**前**口（左局部 +y / 右镜像 −y = 世界 +x）；后口 L07 在踝 +90° 扫过（37/80 mm³）
             ("neck", 0): {1: "window", -1: "none"},         ("neck", 1): {1: "window", -1: "window"},       # 颈俯仰只用 +y（世界 +x）口当分支根，−y 口 T03/T01 扫过；头俯仰两口 N02 脸颊外移让位
             ("yaw_roll_motion", 0): {1: "window", -1: "window"},                                            # 背板上方（头腔内）空
             ("jaw_soft", 1): {1: "free", -1: "free"},                                                       # 头横滚：两侧 A/B 全空，H02 在对面
             ("jaw_soft", 0): {-1: "free", 1: "free"}}                                                       # hr39c 嘴舵机（第 15 颗，jaw.JAW_CONN_MODE）：嘴轴挪到 (31.75, 245.5)、ψ5 后两口都 free（离那摞件真实姿态 2.41 / 2.89）→ 嘴在头链中间；hr38 是 {-1: free, 1: none}

CONN_HUB_KEEP_D = S["rear_boss_d"] + 1.6     # 15.6：A/W 区扣掉的同轴柱 —— 从动件贴惰轮的毂（T03 对颈俯仰、N02 对头俯仰，Ø15）就在 x<−13、r≤7.5，
                                             # 毂端面与插头顶面共面于 x=−13 不重叠，线出口在 r≥10.6（插头中面 y 7.75、z −11.3..−7.3）；不扣的话方块内角 r 6.5 会把毂报成挡线
def conn_zone(sgn, kind="A"):
    """舵机局部系的插座让位体（见上表）。kind: A 插头顶/线翘区（到 x=−17.4）；B 侧出线槽（pocket 型遗留）；W 背板开窗（A 足印沿 −x 延到 CONN_WINDOW_X）。A/W 扣掉 Ø CONN_HUB_KEEP_D 同轴柱。"""
    z0, z1 = S["conn_z"][0] - S["plug_clr"], S["conn_z"][1] + S["plug_clr"]
    xt, xo = S["plug_top_x"] + 0.2, S["plug_top_x"] - S["wire_out"] - S["wire_clr"]
    yi, yo = S["W"] / 2 - S["plug_y_in"], S["W"] / 2 + S["plug_clr"]
    if kind == "A": x0, x1, y0, y1 = xo, xt, yi, yo
    elif kind == "B": x0, x1, y0, y1 = xo, xt, S["W"] / 2, S["W"] / 2 + CLR + Wl + 0.5
    elif kind == "W": x0, x1, y0, y1 = CONN_WINDOW_X, xt, yi, yo
    else: raise KeyError(kind)
    m = bx((x1 - x0, y1 - y0, z1 - z0), ((x0 + x1) / 2, sgn * (y0 + y1) / 2, (z0 + z1) / 2))
    if kind in ("A", "W"):
        m = diff(m, cyl(CONN_HUB_KEEP_D, x1 - x0 + 2.0, ((x0 + x1) / 2, 0, 0), axis="x"))
    return m

def conn_cut(R, sides=(1, -1), mode="window"):
    """PH2.0 插座让位刀（世界系）：sides 里每一侧按 mode 出 W（或作废的 A+B）。mode 见 CONN_MODE 注释；"free"/"none" 不出刀（返回 None）。"""
    kinds = {"pocket": ("A", "B"), "window": ("W",), "free": (), "none": ()}[mode]
    ms = [conn_zone(sgn, k) for sgn in sides for k in kinds]
    return placed(union(*ms), R) if ms else None

def conn_hump_sweep(R, sides, body, lo, hi, n, grow=0.3, mink=False):
    """线翘区 A（随 R 那颗舵机的载体走）绕 body 的关节从 lo 转到 hi 的扫掠（世界系）—— 给盖在 window 型背板外面、且相对载体转动的邻件让位
    （L07 后臂绕踝、N02 脸颊绕头俯仰、T03 壳绕颈俯仰）。grow 默认 DILATE6；mink=True 走 sweep_of 的连续长方体膨胀 —— 扫掠轴与 A 区的
    Ø15.6 扣柱同轴时（N02 对头俯仰）必须用它：DILATE6 沿轴向的那两个副本不缩孔，A 区端面外那 0.3 只剩 Ø15.6 的孔 → 毂外留一圈
    0.1 厚 × 0.3 宽的唇 + 斜向 0.09 薄片（hr24 N02-F12 直径量到 Ø15.58、L1 薄区 +6 mm²）；连续膨胀后孔 ≤ Ø15.0，与 idler_keep 的毂外圆重合。"""
    return sweep_of(placed(union(*[conn_zone(sgn, "A") for sgn in sides]), R), body, lo, hi, n, grow=grow, mink=mink)

def ybox(x0, x1, y, z0, z1):
    """按 |y| 给的对称方块"""
    return wbox((min(x0, x1), -abs(y), min(z0, z1)), (max(x0, x1), abs(y), max(z0, z1)))

# ───────────────────────────── 镜像 / 局部坐标 ─────────────────────────────
def to_local(m, body): return placed(m, np.linalg.inv(TW(body)))
def mirror_y(m):
    M = np.eye(4); M[1, 1] = -1; mm = m.copy(); mm.apply_transform(M); return mm

# ── 真实姿态缺陷区域刀（2026-09-17）──
# 原版策略 5601 个真实姿态里"我们撞、原版不撞"的 body 对（tools/sim/policy_pose_compare.py）→ 每件在所有缺陷姿态下的交集并集
# （tools/sim/policy_pose_regions.py，件 export_local 系）。这些是**双腿联动**的姿态（左右髋偏航同时内收等），单关节 sweep_of 表达不了，
# 所以把区域网格本身当刀：复制到 duckstructure/data/regions_2026-09-17/ 作为设计输入（同 orig 网格的地位），转回零位姿世界系再 minkowski 膨胀。
REGION_DIR = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "data", "regions_2026-09-17"))
def region_cut(fname, body, grow=0.4, mirror=False):
    """fname：REGION_DIR 下的 STL（export_local 系，见 checks.to_local：local = inv(TW(body))·world）；body：该件所属 MJCF body。
    mirror=True：这是右件的区域，镜像到左件（左右件同刀，保持对称）。"""
    m = trimesh.load(os.path.join(REGION_DIR, fname), force="mesh")
    m.apply_transform(TW(body))
    if mirror: m = mirror_y(m)
    return minkowski_box(m, (-grow,) * 3, (grow,) * 3)
MIRROR = {"yaw2roll": "bearing_roll", "hip_l": "hip_l_2", "upper_leg_left": "upper_leg_right", "leg": "leg_2", "ankle_left": "ankle_right"}
