"""头 H01–H03：头内支架 / 可拆压板 / 下头壳 S288 适配件（+ H04 脸板）。
hr39c（2026-09-23 用户：头等比放大每侧 +10）：H03/H04 的毛坯换成 head_scale.scaled_orig()（原版绕 P0 放大 s = 1.2179、壁厚保持）；
和不放大的件配合的特征（H01 四脚凸台、A/B 下半座、横滚舵机/H02 让位、那摞件扫掠刀、嘴舵机托台/惰轮下半环）在**真实位置**重做；
挂在壳上的东西（麦克风台、UBEC 座、摄像头立柱/方孔、麦克风浅槽）按 T_* 平移跟着壳走。改前的常量都留在 *_was_until_2026_09_23_hr39c。"""
import math, numpy as np
from . import s288, kin as K
from . import head_scale as HS
from . import head_bearing_rebuild as HBR
from .s288 import S, cyl, union, diff, inter, placed
from .lib import (P, CLR, B, DILATE6, TW, orig, sfw, drv_self, pt, servo_env, keep_main, wbox, minkowski_box, sweep_of, conn_cut, clean_print_topology)
from trimesh.transformations import rotation_matrix as rot
from .neck import build_yrm, build_neck_pitch, YRM_B_BRG, YRM_AXZ, YRM_A0, YRM_A2
from . import tof as TOF          # hr52：左脸 ToF（H04 座 + H10 压条 + zz_tof 占位同源常量）

# ── H01 头内支架（照原版 motor_support.stl 重雕）
HEAD_CLAMP_MOUNTS = ((13.3, 235.0), (13.3, 243.0))  # H02 → H01；孔间薄料0.443增至1.432mm
KO_OVER = 0.05   # 与 keepouts.yaml 对表的通道刀名义外加量（64 边内接刀 vs Gate 128 边外接禁入体），同 neck.NP_KO_OVER
# H03 四个接收孔（F22，09-12 第二批 I4）：原版孔身 Ø2.2 对 M2 自攻径向重叠 −0.10 不能成牙。先 union Ø3.6 填料柱
# （落在原版 Ø6.0..6.6 凸台壁 r 1.1..3.0 之内，伸出壳面 0.0002 mm³=离散噪声），再钻 Ø1.7×7.0 底孔（M2×8：脚 1.7 + 咬入 6.3，
# 8 ≤ 1.7+7.0−0.3）。Ø1.7 而不是 1.6：孔4 +x 壁只有 1.15、孔1..3 壁 2.15，自攻挤压 0.15/边比 0.20 少 25%，且与 H01 香橙派立柱同径。
# 孔4 (49.1,9.35) 下面是壳底外的开放空间不是腔，+x 壁 z<226.5 消失（226.0 仅 0.62）：填料只到 226.5，其下保留原 Ø2.2 段给螺丝尖让位
# （尖端 225.21 落在 Ø2.2 段里，r 1.0 < 1.1）—— 做成 Ø1.7→Ø2.2 台阶通孔，两端开口。
HEAD_FEET = ((3.1, -30.0), (6.6, 19.0), (42.1, -30.0), (49.1, 9.35))   # = H01 四脚（原版 motor_support），H01 代码里仍是字面量
H03_TOP, H03_HOLE_BOT = 231.5146, 221.5146          # 凸台顶面（= H01 脚底 232.3646−0.85）/ 孔1..3 孔底（原版实测）
H03_FILL_D, H03_FILL_Z0_4 = 3.6, 226.5              # 填料柱 Ø / 孔4 填料下端（+x 壁 r_outer 226.25→1.85、226.5→1.99）
# 2026-09-16 用户定"改了更好"：孔4 的 +x 侧被壳外表面截在 r 2.0（z 226.5..229.5 那 3 mm 高的一段，壁只剩 1.15；再往上进壳板就厚了），
# 外面那块空间实测是空的（placed 里只有 H03 自己和 z≥232 的 H01 脚垫）→ 孔4 再加一根 Ø6.0 套环，从填料下端到凸台顶：三面都在原版 Ø6..6.6 凸台里，
# +x 面往外凸 1.0 到空处，壁 = (6.0−1.7)/2 = 2.15，与孔1..3 同。上壳合缝刀（seam）/舵机刀照旧压过它。H01 不动。
H03_COLLAR_D4 = 6.0
H03_PILOT_D, H03_PILOT_DEPTH = 1.7, 7.0             # 底孔 Ø / 深（从凸台顶面）
# ── hr39c 挂在壳上的电子件跟着壳走的平移量（head_scale.D 在各自贴合点的位移，hr39c_work 里算的；x/z 取让配合面对上的那个件）
#    麦克风 / UBEC：x 跟 H04 脸板内面走（PCB +x 边要插进 H04 浅槽 / UBEC 座 +x 端离脸板仍 0.5），y 跟地板走，z 跟地板走（台坐在地板上）
#    摄像头：跟 H04 走（镜头光轴 = 脸板方孔中心；y 取 0 保对称）
T_MIC = np.array([12.07, 6.31, 2.46])
T_UBEC = np.array([12.07, -5.59, 2.46])
T_CAM = np.array([12.07, 0.0, 6.94])
# 通风槽（2026-09-17，主控换 Radxa ZERO 3W 被动散热，用户要求"开个小窗别捂着"）：主板在 H01 立柱尖 x 66，排针朝 −x 的 14 mm 口袋（x 52..66）。
# hr39c：转接板挪到 Radxa 前面（见 adapter_box），口袋下面不再有槽（tub）→ 进风槽回到口袋正下方的地板上（真实位置 x 54..64，穿放大后的地板）。
# was_until_2026_09_28_hr50: H03_VENT_X0, H03_VENT_X1, H03_VENT_W, H03_VENT_YS = 54.0, 64.0, 2.0, (-16.0, -8.0, 0.0, 8.0, 16.0)
# hr50：槽 −x 端 x 54.0 切在放大毛坯一堵斜墙（x 53.8..55.8）上，墙 −x 侧剩 0.1..0.5 的楔（刀片 ×4 @ x 53.9）；收刀会让槽变短、墙还在 → 槽头往 −x 伸到 53.3 把楔切掉
#   （53.3 以外墙在 z 226 以下，槽底 226.0 横切过去墙厚 ≥1.6；槽仍在 Radxa 旧口袋 x 52..66 里）。
H03_VENT_X0, H03_VENT_X1, H03_VENT_W, H03_VENT_YS = 53.3, 64.0, 2.0, (-16.0, -8.0, 0.0, 8.0, 16.0)
H03_VENT_Z = (226.0, 238.0)                                   # 刀 z（放大后口袋下地板 ≈ 232..234）
# ── 转接板（宇树单总线转接板 40×30×9）：hr28 起在 Radxa 排针口袋下段的 H03 槽里（tub）→ **hr39c 挪到 Radxa 前面**（x 72..81）
#    理由：① 新嘴舵机插座端端面 x 55.3..57.0（y ≥ 21.75）正好压在转接板 +y 边 XT30 插头（x 56.3..61.3）上，两种朝向都撞；
#    ② 放大后 Radxa 前面（CSI 面 x 70.6 到摄像头 PCB 84.7）多出 14 mm 深的一块，站得下 9 厚的板；
#    ③ 顺带解开 USB-C2 插不上（hr39_头内线束.md §4.1 选项 C）：口下面只剩 USB 插头井（H03_USB_WELL），插头不再和板打架；
#    ④ 12 V 主干从左前耳沿地板直接到板边 XT30（§4.6 不再翻 Radxa 顶）。
#    竖放在地板上的 U 形槽里（H03-F14 转接板槽，两条 1.2 壁、6 高），顶边由 H05 压筋（head_top.ADP_PRESS）压住。朝向：L 边（口位基准长边）在下、A 边（PH + XT30 扩展）在 **+y**、B 边（Type-C + XT30 主输入）在 −y（hr39c 交接 ⑥：12 V 主干从左前耳沿地板直走到 −y 边 XT30 主输入；PH 在 +y 离嘴舵机两个插座近）。
H03_ADP_X, H03_ADP_Y, H03_ADP_Z = (72.5, 81.5), (-24.0, 16.0), (235.4, 265.4)      # 板占位（zz_adapter，body jaw_soft）：−x 面离 Radxa CSI 面元件包络 70.6 有 1.9（槽壁 71.0..72.2 离 0.4）
H03_ADP_X_was_until_2026_09_23_hr39c, H03_ADP_Y_was_until_2026_09_23_hr39c, H03_ADP_Z_was_until_2026_09_23_hr39c = (54.3, 63.3), 20.0, (226.8, 256.8)
H03_ADP_SLOT = dict(wall=1.2, h=6.0, clr=0.3, rib_y=(-16.0, 8.0), rib_w=1.5)       # 槽：两道 1.2 壁、从板底往上 6；板底两条托筋（x 向）
# hr41（复审 #1 M1 + 用户 09-24 15:50「每个电子件要有明确支撑位」）：转接板顶边 x 向限位做成「叉」——
#   −x 侧：H05 压筋下挂的齿（head_top.ADP_TINE，x 71.0..72.2，隙 0.3；FPC 缝那一侧只能从顶上伸下来）；
#   +x 侧：H03 槽 +x 壁上两根立柱接到 z 256（板底以上 20.6，隙 0.3）。原来 6 高的槽里板顶能倾 6.25°（顶偏 3.3，t·cosθ + h·sinθ = 9.6）→ 立柱后 ≤0.5。
#   立柱 y ±(10.5..13.5)：避开 PH/XT30（y ≥ 16）、Type-C（y ≤ −24）、UBEC 输入线（x ≈86、z ≈237）；x 81.8..83.4 离 H06 压框（x ≥ 84.5）1.1。
H03_ADP_POSTS = dict(x=(81.8, 83.4), ys=((-13.5, -10.5), (10.5, 13.5)), z_top=256.0)
# hr43c（协调员 14:55 ①，用户看实物定）：转接板 40×30 四边全是接口、边上没地方卡 → **不卡边、改双面胶竖粘**：
#   H03-F14 口袋 +x（脸）那一侧的壁 + 两根 +x 立柱（hr41 M1）→ 长成一整块平床 H03-F22：从口袋底 z 232 到板顶 265.4、y 覆盖板背面 40（−24..16），
#   床面 x 82.0 = 板背面 81.5 + 胶带 1.0 − 坑深 0.5；床厚 1.7（坑底剩 1.2）；床面挖 0.5 深浅坑 H03-F23 放 1 mm 泡棉双面胶（坑比板小 2 mm 一圈：y −22..14 × z 237.4..263.4）。
#   板背面按光面假定（用户背面照片待补）。−x 侧壁、两条托筋、两端挡块、H05-F06 顶压筋都保留（压筋当保险）。
#   床背面 x 83.7 离 H06 压框（x 84.5）0.8、离摄像头 FPC 座（x 85.47）1.77、离 UBEC（x ≥ 84.37 且 y ≤ −25.29）1.29（斜向）；UBEC 座（x ≥ 83.07, y ≤ −23.6）同件并上。
H03_ADP_BED = dict(x=(82.0, 83.7), y=(-24.0, 16.0), z0=232.0, z_top=265.4, pit_depth=0.5, pit_y=(-22.0, 14.0), pit_z=(237.4, 263.4), tape_t=1.0,
                   src="板占位 H03_ADP_*（背面 x 81.5）；胶带 1 mm 泡棉双面胶 assumed；坑 0.5 深、比板小 2 mm 一圈（协调员 14:55）")
H03_USB_WELL = dict(cav=((62.3, -14.2, 230.3), (72.5, -3.8, 238.0)), wall=1.2, floor=1.5)   # Radxa USB-C2 口（y −13.47..−4.53，口面 z 234.8 朝 −z）下面的插头井：
                                                                                               # 超薄 FPC 90° 插头 8.9×3.5×4（assumed，同 wiring_head.USB_PLUG_SLIM）占 z 230.8..234.8，线沿 +x 出井到转接板
# hr39c-⑧（agent #4）：USB FPC（Radxa USB-C2 → 转接板 Type-C）插头改朝 **+x** 出，沿井底到 x ≈71.4 竖起、在 Radxa 背面元件包络（x 70.6）与转接板（x 72.5）之间的 1.9 缝里
#   上到 z 243.5 处 45° 折成沿 −y 走（宽向 z 239.5..247.5 = 转接板 Type-C 插头 z 范围），到 y −27 转 +x 插转接板 Type-C（B 边 −y）。
#   槽 −x 壁（x 71.0..72.2、顶 z 241.4）挡在这条缝里 → ① y −13.6..−4.4（FPC 竖起段 y −13..−5 ± 0.6）整段开口；② y −24.5..−13.6 壁顶削到 z 239.0（FPC 横段底 239.5 − 0.5）。
#   槽壁剩：y −24..−13.6 高 3.6（z 235.4..239.0，仍卡住板底边）、y −4.4..16 原高 6；+x 壁 / 托筋 / 挡块 / H05 压筋不动。原计划「朝 −x 出绕 Radxa −y 端外」被 TF 卡包络（y 到 −34.5）+ xt30_12v 堵死。
H03_USB_WALL_CUTS = (((70.8, -13.6, 231.5), (72.4, -4.4, 242.5)), ((70.8, -24.5, 239.0), (72.4, -13.6, 242.5)))
# hr38 以前的 tub 常量（hr39c 作废，留痕）
H03_TUB_IN = ((53.8, 63.8), 21.0, 226.0)
H03_TUB_OUT = ((52.6, 65.3), 22.5, 224.5)
H03_TUB_WALL_TOP = (238.0, 235.5, 235.0)
H03_TUB_RIB = ((-12.0, 12.0), 1.5, 0.8)
# hr37（2026-09-22）：麦克风台 —— H03 地板上、脸板后面，方型 I²S 麦（10×10，electronics.MIC_*）平放、芯片朝 −z 对地板声孔。
#   台 2.0 厚（地板只有 1.5，芯片沉坑 1.4 深不能直接挖地板）；三面 0.8 墙（−x / ±y）2.6 高，−x 墙顶有 0.6 内伸唇压住 PCB 边；+x 边落进 H04 内面 0.5 深浅槽（H04_MIC_SLOT）。
#   hr39c：整组（台 + 麦 + H04 浅槽）平移 T_MIC。
MIC_PAD_was_until_2026_09_23_hr39c = ((69.8, 80.1), (23.2, 34.8), 231.0, 234.2)
MIC_PAD = ((69.8 + T_MIC[0], 80.1 + T_MIC[0]), (23.2 + T_MIC[1], 34.8 + T_MIC[1]), 231.0 + T_MIC[2], 234.2 + T_MIC[2])   # 台 x、y（含墙）、地板顶、台顶
# hr38（2026-09-23 用户看实物）：排针短脚 2.5 在芯片面 —— PCB 贴不平台面。台顶抬到 234.2（台 3.2 厚），两排针位各开一条 2.0 宽 × 2.8 深的让位槽
# （槽底 231.4，不穿地板顶 231.0）；针位 = electronics.mic_boxes 的两排：y = 边内 1.27，x = 中心 ±3.81（3 针 2.54 间距）+ 单边 0.5。
MIC_PIN_SLOT_W, MIC_PIN_SLOT_DEPTH, MIC_PIN_SLOT_XPAD = 2.0, 2.8, 0.5
MIC_PAD_WALL, MIC_PAD_WALL_H, MIC_PAD_LIP = 0.8, 2.6, 0.6  # 墙厚 / 墙高（从台顶）/ −x 墙内伸唇
MIC_SOUND_D, MIC_CHIP_D, MIC_CHIP_DEPTH = 1.5, 4.2, 1.4    # 声孔 Ø（穿地板）/ 芯片沉坑 Ø / 深
# hr43d（加固：hr43_work/转接板安放研究.md §6.2、agent_brief_hr43d 第 5 条）：麦克风台面留 1 mm 泡棉胶位 —— 两排短针槽之间（y 边内 band_y_in）、−x 墙内面到台 +x 端，
#   台面下沉 sink 0.8；贴 1 mm 泡棉双面胶条（≈9.5 × 6.2，中间开 Ø hole_d 孔让芯片 Ø4 穿过、围住 Ø1.5 声孔 = 声学密封垫），压到 0.8 → PCB 被顶在 −x 唇下（唇离 PCB 顶 0.2）= 预压；
#   PCB 位置 MIC_Z0、针槽、沉坑绝对深度都不变。研究写「中间 Ø3 孔」：芯片 Ø4 × 1.2 朝下进沉坑，Ø3 穿不过 → 按芯片 Ø4 + 单边 0.2 = Ø4.4（assumed，到货看芯片外形）。
MIC_FOAM = dict(t=1.0, sink=0.8, band_y_in=1.9, hole_d=4.4,
                src="泡棉胶 1.0（components.yaml:tape_foam_1mm）；下沉 0.8 = 压缩 0.2（assumed）；条宽 = 两排针槽之间（针心离边 1.27 + 针半宽 0.32 + 隙 0.31）")
# hr38（2026-09-23）：UBEC-3A（26×12×6，components.yaml:buck_12v_5v.envelope_mm）从躯干 −y 侧腔挪进头里 —— H03 地板 −y 侧，
#   麦克风台的镜像位。**立放**：6 沿 x、12 沿 y、26 沿 z。hr39c：整组平移 T_UBEC（x 72.3..78.3 → 84.37..90.37）。
#   位置依据（hr38）：体素扫描 26.6×12.6×6.6 在整个头里只有三处站得下，这是唯一一处有打印件地板可以坐的。
#   台/墙尺寸照麦克风台的口径：台 2.0 厚（地板只有 1.5）、墙 0.8 厚 12 高、四面围住，模块从 +z 放下，一根扎带从两条槽穿上来兜住顶面。
UBEC_PAD_was_until_2026_09_23_hr39c = ((71.0, 79.6), (-34.0, -18.0), 231.0, 233.0)
UBEC_PAD = ((71.0 + T_UBEC[0], 79.6 + T_UBEC[0]), (-34.0 + T_UBEC[1], -18.0 + T_UBEC[1]), 231.0 + T_UBEC[2], 233.0 + T_UBEC[2])
UBEC_RAISE = 5.0                                              # hr39c（agent #4）：两头出线 → 模块抬高 5，底端下留线弯 90°（线对 22AWG 硅胶 Ø1.6，弯 R≈3.2）
UBEC_MOD = ((72.3 + T_UBEC[0], 78.3 + T_UBEC[0]), (-31.7 + T_UBEC[1], -19.7 + T_UBEC[1]), (233.0 + T_UBEC[2] + UBEC_RAISE, 259.0 + T_UBEC[2] + UBEC_RAISE))   # 模块本体（= electronics.UBEC_BOX）
UBEC_LEDGE_W = 0.8                                            # 抬高后模块底边坐在 ±x 两条 0.8 宽托条上（托条之间 4.4 空给底端出线）
UBEC_WALL, UBEC_WALL_H = 0.8, 12.0                          # 四面墙厚 / 墙高（从台顶）
UBEC_TIE_SLOT = ((73.5 + T_UBEC[0], 76.5 + T_UBEC[0]), 0.8, ((-33.6 + T_UBEC[1], -32.8 + T_UBEC[1]), (-19.2 + T_UBEC[1], -18.4 + T_UBEC[1])))   # hr39c 留痕（hr41 不再用）
# hr41（复审 #1 M7）：+y 墙出线窗（输入线从模块底端出、走 +y 口）改成 4.4 宽 U 槽一直开到墙顶 → 卖家代焊的 XT30U-F 不用穿窗，线从上面放进槽。
#   原扎带槽在 ±y 墙外竖着过（x 85.57..88.57），正好压在出线窗外侧；U 槽开到墙顶后「挪到窗上方」无墙可挪 → 扎带改成沿 x 绕：
#   模块顶上沿 x 过，±x 墙外 0.1 各开一条 0.8×3 地板槽（y 以模块中心 ±1.5），从 H03 地板底下兜回来。+x 槽离 H04 内面 0.05、带子夹在 +x 墙与 H04 之间（缝 0.95，薄扎带 assumed ≤0.9 厚）。
UBEC_NOTCH_OPEN_TOP = (1,)                 # 哪一侧的出线窗开到墙顶（+1 = +y 墙）
UBEC_TIE_X = dict(w=0.8, gap=0.1, len_y=3.0)
# hr43d（加固：研究 §6.2、agent_brief_hr43d 第 5 条）：UBEC 口袋底贴 1 mm 泡棉 —— 两条托条顶下沉 sink 0.8（模块位置 UBEC_MOD / electronics.UBEC_BOX 不变，
#   托条上各贴 1 mm 泡棉胶压到 0.8，扎带从模块顶上兜住压紧）；+x 扎带缝（+x 墙外槽离 H04 内面 0.05，缝 0.95）不改 → 登记「≤0.9 厚细扎带」（assumed，components / assembly）。
UBEC_FOAM = dict(t=1.0, sink=0.8, src="同 MIC_FOAM；UBEC 热缩外皮上贴泡棉胶（assumed 粘得住）")
# hr37：H04 脸板（替换原版 face_part，外形/厚度沿用：平板插顶壳凹槽，H03 前唇槽锁底边）。摄像头 8M 219 镜座 13.2 方 → 13.6 方孔；
#   4 根 Ø4.5 立柱从板内面到 PCB 前面（长 5.9），Ø1.7 底孔深 5（M2×4 自攻从背面穿 PCB Ø2 孔）。
#   hr39c：毛坯 = 放大后的 face_part；板层 = 原 x 80.05..81.45 那层放大后的位置（前面 81.4 → 93.47，板厚 1.3 保持 → 内面 92.17）。
H04_PLATE_X_was_until_2026_09_23_hr39c = (80.05, 81.45)
H04_PLATE_X = (92.12, 93.52)
H04_PLATE_IN = 92.17                                        # 板内面 x（放大后，实测 head_scale.D(80.1, 0, 240)）
H04_CAM_C, H04_CAM_HOLE, H04_CAM_HOLE_PITCH = (0.0 + T_CAM[1], 251.0 + T_CAM[2]), 13.6, 28.0     # (0, 257.94)
H04_CAM_C_was_until_2026_09_23_hr39c = (0.0, 251.0)
H04_POST_D, H04_POST_X0, H04_PILOT_D, H04_PILOT_DEPTH = 4.5, 74.2 + T_CAM[0], 1.7, 5.0            # 立柱端面 x 86.27（= 摄像头 PCB 前面）
# 原版镜孔 / 右侧小孔 的填料范围：原版口径 (y, z) 按 head_scale.S 等比映射后再外扩 0.6。
# **G7 那条缝**：hr37/38 的镜孔填料上沿 259.5，比原版镜孔真实上沿 ≈260 低，填完后方孔上方留一条 3×0.5 的缝 → 上沿取原版 261.5（映射后 270.9），整片盖住。
def _Sz(z): return float(HS.S([0, 0, z])[2])
def _Sy(y): return float(HS.S([0, y, 0])[1])
H04_FILLS = (((_Sy(-8.5) - 0.6, _Sy(8.5) + 0.6), (_Sz(242.5), _Sz(261.5))), ((_Sy(16.0) - 0.6, _Sy(28.0) + 0.6), (_Sz(244.5), _Sz(254.0))))
H04_FILLS_was_until_2026_09_23_hr39c = (((-8.5, 8.5), (243.0, 259.5)), ((16.0, 28.0), (245.0, 253.5)))
H04_MIC_SLOT = ((H04_PLATE_IN - 0.05, H04_PLATE_IN + 0.55), (23.8 + T_MIC[1], 34.2 + T_MIC[1]), (234.0 + T_MIC[2], 236.0 + T_MIC[2]))   # 麦克风 PCB +x 边的浅槽（跟麦克风走 T_MIC，不跟板走）
# ━━ hr41 摄像头贴板（用户 09-23 23:53 定：去掉 H04 短立柱，PCB 直接贴面板内面 x 92.17，只让镜头伸出、圆眼 E02 罩住；PCB 绝不放到面板外）━━
#   光轴 (y 0, z 257.94) 不变，摄像头整体沿 +x 前移 5.9（= 立柱长）：PCB 90.57..92.17，镜头前端 106.97（electronics.CAM_PCB_X）。
#   H04：r ≤ 11.5 通窗（镜座 13.2 方对角 18.7 < 23，E02 Ø24 盖住）；PCB 投影其余地方挖 0.8 浅坑（留 0.5 皮）给 PCB 正面元件 —— **正面元件高度没量，0.8 是 assumed**；
#   四角 5×5 贴合面（四孔 Ø2 @ ±14 的焊盘圈，常见禁布区，assumed）上各一根 Ø1.8×1.2 短定位销插 PCB 孔；
#   固定 = 新件 H06 内侧压框（build_camera_clamp）：两根竖条压 PCB 四角，两只耳朵各 1 颗 M2×8 自攻拧进 H04 内面 PCB 轮廓外的 Ø5 凸台（螺丝头在头里）。
#   E01 白外圈的 3 个 Ø2 销孔（r 14.2）全落在 PCB 投影里（Ø32 外圈 ⊂ 32×32 方），与 0.5 皮冲突 → 销孔取消，E01 改成套在 E02 圆筒外定心（eye.py）。
H04_CAM_WIN_R = 11.5
H04_CAM_PCB = 32.0
H04_CAM_POCKET = dict(depth=0.8, clr=0.3, corner=5.0, src="浅坑深 0.8 assumed（PCB 正面元件高度未量）；轮廓外扩 0.3；四角 5×5 贴合面 assumed（孔圈禁布区）")
H04_CAM_PIN = dict(d=1.8, h=1.2, pitch=28.0, src="孔 Ø2、孔距 28×28（Radxa 官方机械图，camera_csi）；销 Ø1.8 × 1.2 < PCB 厚 1.6")
H04_CLAMP_BOSS = dict(y=19.2, d=5.0, x_top=87.0, pilot_d=1.7, pilot_bottom_x=92.8)   # 压框凸台：离 PCB 边 16 → 0.7；顶面 87.0；底孔到 92.8（离脸面 93.47 留 0.67）
H06_CLAMP = dict(bar_y=(12.5, 16.0), bar_x=(84.5, 87.2), pad_x=(87.2, 90.57), pad_h=4.5, ear_y=(16.0, 21.7), ear_z=3.0, ear_x=(84.5, 86.8), bottom_h=3.0)
#   H06 压框：竖条离 PCB 背面 3.37（背面元件除 FPC 翻盖座外没量，按 ≤3 assumed）；四角压块前面 = PCB 背面 90.57；耳朵前面 86.8 比凸台顶 87.0 退 0.2 →
#   拧紧时压框弯 0.2 给 PCB 四角预压。耳朵叠厚 2.3 + 隙 0.2 → M2×8 咬入 5.5（≥ pla_self_tap 4.0；螺尖 92.5 离孔底 92.8 余 0.3）。
H04_POST_REMOVED_2026_09_24_hr41 = True   # H04_POST_* 常量只留作 hr39c 留痕，build_face_plate 不再用

def adapter_box():
    """转接板占位（世界系零位，body jaw_soft）：hr43d 起平放在 H08 转接板托板上（electronics.ADP_FLAT / adapter_boxes）；只参与检查/预览，不打印。"""
    # was_until_2026_09_25_hr43d: return wbox((H03_ADP_X[0], H03_ADP_Y[0], H03_ADP_Z[0]), (H03_ADP_X[1], H03_ADP_Y[1], H03_ADP_Z[1]))   # hr39c..hr43c 竖放在 H03 槽 / F22 平床
    from .electronics import adapter_boxes
    return adapter_boxes()
# B 端唇（09-12 ④ 恢复原版）：原版 motor_support 在 x 44.5 起有一圈 r 9.5 的唇顶住 22×16×4 外圈的 +x 面，
# 我们的 Ø22.18×20 座刀（x 34..54）把它切掉了。这里补回上半圈 Ø18.6/Ø23.2 × 0.7，x 44.2..44.9，
# 和 H03 下半座自带的唇（原版实测 x 44.2..44.9、r 9.3）拼成整圈。内径 18.6 > N04 销 Ø15.9（隙 1.35）、> Ø6 拆卸顶杆。
H01_LIP_B = (18.6, 23.2, YRM_B_BRG[1], YRM_B_BRG[1] + 0.7)   # 内 Ø, 外 Ø, x0, x1
# Legacy A-seat cutter roughs out the original blank; HBR.head_seats supplies
# the final6704 pocket and float stops after it. These bounds are independent
# of the former PLA-disc clearance assertion.
H03_SEAT_A_X0 = 6.0
assert HBR.HEAD_A['float_x0'] < HBR.HEAD_A['x0'] < HBR.HEAD_A['x1'] < HBR.HEAD_A['float_x1']
# 合缝让位（09-12 第二批 I6）：原版上壳在 MJCF 里比下壳低 0.2（top_head_shell pos −0.0045 vs bottom −0.0043），上壳两个朝下的
# 合缝面各压进 H03 朝上的面 0.200，整圈 67.9 mm³（orig×orig 同值）。上壳冻结，H03 让：减上壳 DILATE6(0.1)。0.1 = 消掉 0.2 后
# 留 0.1 净空（上壳无紧固件只靠合缝面托，下沉 0.1 与头内件交集仍 0；≥0.2 起撞 face_part 且啃唇，横向副本 δ≤0.15 不啃唇）。
HEAD_SEAM_CLR = 0.1
# H01 右前脚 (49.1, 9.35) 的承压圈内侧给 N03 A 端 6 颗法兰螺丝的起子刀路让位（09-13 分度圆定案 5.25 触发）：
# 起子刀路 Ø4.2×45 @ r=horn_r 从 +x 经 N03 内腔到 A 盘（mechanical_audit N03 tool corridors 同款），横滚舵机局部 y=世界 −z、z=世界 +y →
# 60° 那条落在世界 (y 4.55, z 232.99)，外缘擦到这只脚的承压圈 1.19 mm³（4.75 时 0.013）—— docs/reports/分度圆Ø9.5还是Ø10.5_2026-09-12.md §定案后。
# 09-13 首版拿"绕横滚轴 r ≥ 7.4 的圆柱"整圈削（H01_FOOT_RIM_R=horn_r+2.1+0.05），脚顶 z 233.2 离轴近、削面 y 6.995，孔壁(y 8.0)到削面只剩 1.02（<1.2 规则，09-14 复审 MAJOR）。
# 09-14 改成**只削起子刀路本身**：6 条 Ø H01_FOOT_TOOL_D 沿 x 的刀（同 mechanical_audit 的位置，从 Rr 与 horn_r 推），只有 60° 那条碰脚；
# 首版 Ø(4.2+2·KO_OVER)=Ø4.3：刀心 (4.55, 232.99) 到孔心 (9.35, 232.36) 4.84 − 孔 r 1.35 − 刀 r 2.15 = 孔壁到削面最薄 1.34（实测 1.31）。
# 09-14 复审 R3g MAJOR：Ø4.3 刀（= 4.2 + 2·KO_OVER）对 mechanical_audit 同款 Ø4.2×45 起子刀只余 0.048（TC02 内轮廓缩 0.15/边 → 打出来径向过盈 ≈0.10）。
# 原版这里是 Ø4.3（KO_OVER 名义值，不是装配余量）→ 改 Ø4.5：刀余量 0.15（≥ 0.15 判据，mechanical_audit N03 tool corridors 改判 min_gap）；
# 孔壁到槽面 4.84 − 1.35 − 2.25 = 1.24 仍 ≥1.2（review_r1 e 不变量；r3 build 实测 1.210，64 边刀）。Ø4.4 只余 0.10、Ø4.6 时孔壁 1.19 <1.2，4.5 是唯一同时满足两条的整数档。
H01_FOOT_TOOL_D = 4.5
# hr39c Radxa 右下立柱改支撑（见 build_head_bracket 里 posts 的注释）
H01_POST_LOPY_X0 = 58.1
H01_POST_BAR = ((58.1, 32.8, 237.0), (66.0, 36.0, 265.0))
H01_POST_WEB_LO = ((58.1, 26.5, 237.0), (66.0, 33.0, 242.0))
H01_POST_WEB_HI = ((58.1, 26.5, 260.0), (66.0, 33.0, 265.0))
H01_OTG_RELIEF = ((62.6, -26.87, 234.0), (66.3, -17.0, 243.0))     # hr39c-OTG：−y 下立柱对 USB-C1 口壳（y ≥ −26.67）让位 0.2
# hr47（2026-09-27 用户看打印预览：上两根 Ø5 立柱 (±29, 262.5) 根部只有下 34% 踩在前立板顶 262 上，圆柱上半截悬在板顶外"像没连上"）：
#   前立板顶边在两根立柱处各长一块座块：x 同前立板 47..52、|y| 26..31.5（缺口 |y|≤26 之外、盖住柱外沿 29+2.5）、z 从缺口底 257.6 到柱顶 265 ——
#   立柱根部 x 51..52 整圆埋进座块，柱位 / 端面 x 66 / Ø1.7 底孔全不动（frozen h01_sbc_posts 生成式不变）。改前探针（58cd16ab scratchpad/h01_probe.py）：
#   座块 ±0.4 内无任何 placed 件（上壳/嘴舵机/杜邦壳/Radxa 都不在）、H03 在座块正上方 +z 柱体（拆卸行程）0 mm³ → H03 不变。
#   打印朝向 down_world [0,0,1]（顶朝下）：座块顶 z 265 与横梁顶同一层贴床，首层多两块 5×5.5 的落脚，不加悬垂。
H01_POST_SADDLE = ((47.0, 26.0, 257.6), (52.0, 31.5, 265.0))   # +y 那块；−y 镜像
# hr50（2026-09-28 用户：比一条挤出线还薄的薄片不要）H01 的四处收刀 / 削穿（见 _build_head_bracket 里的用法）：
#   ① N03 走廊盒 x1 = 42.0 把原版毛坯 x 40.7..42.35 那堵墙削到只剩 0.35（y −24.5..−16.1、z 233.2..237.6，薄膜 69 mm²）；这段墙外 N03 在 x ≤ 39、
#      头横滚 ±30° 扫不到 → 收刀：这一块走廊只切到 41.5（墙留 0.85）。
H01_CORRIDOR_KEEP = ((41.5, -24.8, 232.9), (42.1, -15.8, 237.7))
#   ② 前立板顶缺口（z ≥ 257.6）与下面嘴舵机包络（servo_env_world，顶到 257.92、ψ 5° 斜）之间只剩 0.0..0.44（y 21.45..26、薄膜 44.6 mm²）；
#      缺口原本给 Radxa 排针杜邦壳（hr43 起 Radxa 搬后脑）→ 收刀：y 20.5..26 这段缺口底抬到 258.75（离包络顶 ≥0.83），其余缺口不动。
H01_NOTCH_KEEP = ((46.4, 20.5, 257.5), (52.6, 26.1, 258.75))
# hr50_r2 2026-09-28：前缘缺口 +y 端角外（x 52.5..53.75）前立板那条料的侧面在 y 25.75，比缺口侧面 y 26.0 往里 0.25，缺口盒 +x 端 52.5 与它交出
#   0.25 的 L 形台阶（clean_check 碎边 (52.4, 25.9, 259.0) slab 0.116 n 6）→ 这条 y 25.70..26.0 切齐到缺口侧面 26.0（KEEP 顶 258.75 以上）。
#   在 placed 件上实测多切 0.249 mm³、该条碎边消失、无新增；离 Radxa 立柱孔（Ø1.7 @ (62, 29, 262.5) 沿 x）≥1.8。缺口本身不动（新刀 ⊇ 旧刀）。
H01_NOTCH_CORNER_CUT = ((52.45, 25.70, 258.75), (54.2, 26.0, 262.0))
#   ③ 新料避 H03 真实毛坯（±0.4）时，下托板 lowerL 在放大肋（x −3.1..−1.05）与核心左后脚凸台 (3.1, −30) 之间只剩 0.2..0.4 的 C 形薄边（薄膜 16.7 mm²），
#      endR 在放大肋与核心之间剩一片 0.2 的鳍（y 11.7..12.8，薄膜 16.2 mm²）。两边都是 H03 实体，补厚必碰 → 整片削穿（切口在厚 ≥0.8 处，平面）。
H01_SQUEEZE_CUTS = (((-1.3, -33.2, 222.6), (0.3, -28.3, 225.5)), ((-1.3, 11.55, 222.6), (0.3, 13.0, 231.45)))
#   ④ KO09 右前脚起子通道（Ø4.65 @ (49.1, 9.35)）的 −x 边 46.775 比前立板 −x 面 47.0 还靠外，通道两侧留 0..0.8 的楔（刀片 (47.1, 10.6) / (47.4, 8.0)、
#      尖刺 (47.0, 8.4, 243.3)）；起子通道不能收 → 削穿：前立板 −x 面在 |y−9.35| ≤ 2.0 处退到 x 47.9（通道边在 |dy| = 2.0 处 x 47.91，转角 59°）。
# was_until_2026_09_28_hr50_r2: H01_KO09_FACE_CUT = ((46.6, 7.35, 233.1), (47.9, 11.35, 262.2))   # 底 233.1 比 KO09 通道底 233.2 低 0.1，削进右前脚承压圈（mechanical_audit：rim wall 0.035 < 1.2）
H01_KO09_FACE_CUT = ((46.6, 7.35, 233.2), (47.9, 11.35, 262.2))   # 11:50 主设计：刀底抬到 233.2 与通道底齐平；楔本来只在通道旁（z ≥ 233.2），承压圈（z ≤ 233.15）不再碰
#   ⑤ 原版毛坯 −y 侧那道旧 XL330 拱罩（y −31..−28.6、x 13..38）被 N03 上半扫掠刀（头横滚 ±25°、1.25° 一步、DILATE6 0.4）掏空中段，剩拱顶一条
#      0.2..0.75 的带（z 249.25..250.3）+ 两条腿顶的锯齿（z 241..241.5）：刀片 30.1 / 15.8 mm²、尖刺 ×7、碎边 ×3。收刀会碰 N03 → 削穿：
#      y −31.3..−28.3 里 z 241.0 以上整段切掉（拱带去掉、两腿顶切平到 241.0）；两侧斜墙、腿下半段不动。
H01_ARCH_BAND_CUT = ((13.0, -31.3, 241.0), (38.0, -28.3, 250.6))

_H01 = {}
def build_head_bracket():
    """（hr39c：同进程缓存一份，H03 的 H01 让位刀要用）"""
    if "m" not in _H01: _H01["m"] = _build_head_bracket()
    return _H01["m"].copy()


def _build_head_bracket():
    """头内支架（照原版 motor_support.stl 重雕，原壳不动）。

    实测澄清了一件事：**原版 motor_support 根本不夹头横滚 XL330**（与它只相交 0.17 cm³，是擦边）——
    它是**下颚机构支架**（+y 端托下颚 XL330、-y 端 Ø15 轴承拱、顶板托喇叭），
    原版的头横滚舵机是被上下头壳直接抱住的。但它身上有三样东西是 H01 必须有、而我们上一版是手画近似的：
      ① 4 只脚：孔 Ø2.70 @ (3.1,-30.0)(6.6,19.0)(42.1,-30.0)(49.1,9.35)，脚底 z=231.51 正好坐在底壳面上，脚厚 1.70
         （上一版画在 (3,-30)(7,19)(42,-30)(49,9)，每个都差 0.1~0.4）
      ② A/B 两个 Ø22.18 座的**上半圈**（内 r=11.09 外 r=12.70，A 在 x 5.5..9.5、B 在 x 40.5..46.5），正好接上底壳的下半圈
      ③ 一圈贴着头壳内轮廓走的周边梁 —— 原版验证过的，与上下壳干涉恒为 0
    所以拿原版当毛坯，我们只补「舵机笼 + 前立板/香橙派立柱」，把 pads/seatA/seatB/upA/upA2/railL/railR/frontbar 全交还给原版。
    B 座上半圈现在装的是 22×16×4 轴承的外圈（Ø22 在 Ø22.18 里，径向 0.09），不再是 PLA 盘；另补回原版的 B 端唇（H01_LIP_B）。

    舵机笼有 0.3mm 装配间隙，不能传递无回差的反力矩。独立 H02 压板用两颗背角孔固定舵机，
    再用两颗 M2×10 接到本件；拆掉 H02 后保持舵机沿 -x 的抽出路径。香橙派立柱冻结。

    ⚠ 前立板上那个 Ø22.78 孔**不是**轴承/B 盘的滑入口：脚 (49.1, 9.35) 的承压圈离横滚轴心只有 r ≈6.7（x 46..52；09-14 起只给 60° 起子刀路削一条 Ø4.3 的槽，
    不再整圈削到 7.4），任何 Ø>13.4 的东西都进不来。它现在的用途只有一个：拆 N04 时 Ø6 顶杆从 +x 沿轴线进去（实测 0 交集）。"""
    Rr = drv_self("jaw_soft")                       # 头横滚 S288：法兰面 x=+8.3，薄段背 -14.7，厚段背 -17.7（09-14 SERVO_SHIFT −1.7；09-13 8.1/-14.9/-17.9，旧 7.95/-14.85/-17.75）
    O = orig("jaw_soft", "motor_support")           # 毛坯
    y_lo, y_hi = -24.5 - CLR, 9.5 + CLR             # 舵机长边（世界 y）± 间隙
    lower = wbox((-12.0, y_lo - 3.0, 222.8), (5.0, y_hi + 3.0, 225.3))                   # 下托板（舵机底面 225.615）
    lowerL = wbox((-12.0, -33.0, 222.8), (12.0, y_lo, 225.3))                            # 往 -y 延，接原版左脚
    # （上一版还有一块 lowerR 往 +y 延到 y=22，那是为了搭上手画的 railR。railR 交还给原版周边梁之后 lowerR 就悬空了，去掉）
    endL = wbox((-12.0, y_lo - 3.0, 222.8), (5.0, y_lo, 238.0))                          # -y 端挡墙（天花板 y=-28 处 ≈239）
    endR = wbox((-12.0, y_hi, 222.8), (5.0, y_hi + 3.0, 246.0))                          # +y 端挡墙
    bridge = wbox((-14.0, -26.0, 246.4), (12.0, 12.5, 249.5))                            # 上桥（舵机顶面 245.615）
    riseL = wbox((-14.0, -26.0, 236.0), (-11.0, -23.0, 249.5))                           # 桥后端立柱（连到 endL）
    hx, hy = P["brd_h"]
    postbase = wbox((47.0, -31.0, 231.5), (52.0, 31.0, 262.0))                           # 前立板（香橙派；板子没到货，孔距不动）
    # was_until_2026_09_25_hr43c: posts = [cyl(5.0, 66.0 - 51.0, (58.5, sy * hx / 2, 251.0 + sz * hy / 2), axis="x") for sy in (1, -1) for sz in (1, -1) if not (sy > 0 and sz < 0)]
    # hr43c（协调员 14:55 ②）：Radxa 已搬后脑（hr43），四根旧立柱里 H07 只拧 (±29, 262.5) 与 (+29, 239.5)（F35）三根 → 空着的右下 (−29, 239.5) 删；
    #   其余三根位置 / 端面 x 66 / Ø1.7 底孔不动（F35 孔位；B1 靠 H07 托面 + 功放 −y 2.0 + 过孔解决，见 H07_HR43C）。frozen h01_sbc_posts 例外同步。
    posts = [cyl(5.0, 66.0 - 51.0, (58.5, sy * hx / 2, 251.0 + sz * hy / 2), axis="x") for sy in (1, -1) for sz in (1, -1) if sz > 0]
    # hr39c-OTG（agent #4）：−y 下立柱 (58.5, −29, 239.5) Ø5 的 +y 边与 Radxa USB-C1（OTG）口壳（规范宽 8.94 → y ≥ −26.67，x 62.8..66）重 0.17 → 立柱只在口壳旁局部削平（离口壳 0.2）；
    #   Ø1.7 底孔（y −29 ± 0.85）与端面 x 66 的位置都不动，端面只少一条弦（离孔心 2.13 外那 0.37 高的一小条）。OTG 口以后刷机/调试要插线。
    posts = [diff(p, wbox(*H01_OTG_RELIEF)) if abs(p.bounds[0][1] + 31.5) < 0.1 and p.bounds[0][2] < 245.0 else p for p in posts]
    # hr39c：Radxa 右下立柱（+y 下，(58.5, 29, 239.5)）根部 x 51..57 被新嘴舵机插座端占掉（jaw.py：舵机本体到 x 57.03）——
    #   hr39c 重搜：这根立柱原样的话，头放大 +10 内嘴舵机全头 0 个可行摆法（hr39c_work/jaw_scan.log）。H01 立柱冻结，这里**只改支撑方式**：
    #   ① 立柱尖段 x H01_POST_LOPY_X0..66 保留（Ø5、端面 x 66、Ø1.7 底孔、孔位都不变 = Radxa 安装接口不变）；
    #   ② 尖段经连接块（板角安装孔禁布区里，y 26.5..33 × z 237..242）接到板边外的竖梁（y 32.8..36，板边 32.5 外 0.3），竖梁再经上连接块接上 +y 立柱。
    #   竖梁/连接块都在 x ≥ 58.1（离嘴舵机端面 57.03 ≥1.0），y ≥ 26.5（离排针杜邦壳 |y|≤25.4 ≥1.1），z 237..242 在板角 u.FL 座（z≈245..247，官方正面图估）之下。
    posts += [cyl(5.0, 66.0 - H01_POST_LOPY_X0, ((H01_POST_LOPY_X0 + 66.0) / 2, hx / 2, 251.0 - hy / 2), axis="x"),
              wbox(*H01_POST_BAR), wbox(*H01_POST_WEB_LO), wbox(*H01_POST_WEB_HI)]
    # hr11：去掉下托板 lower —— 头部轴承补丁后偏航舵机只能横着（−y）进 N03，而这要求 H01 不在场；横滚舵机的 F17 起子又必须穿过空的偏航舵机腔，
    #        所以 N03+横滚舵机(F17)+N04/B 轴承 → 偏航舵机横进 → **H01 整个从上面套下来**。hr11 试算：H01 沿 +z 对全部件只有 lower 撞横滚舵机 1156 mm³。
    #        托板只是托（笼有 0.3 间隙、不传扭；舵机本体靠 H02 背夹 4 颗背孔螺丝 + F19 固定，头重走 A/B 轴承），去掉后 CF4 插座也不用再啃窗。
    # was_until_2026_09_27_hr47: new = union(lowerL, endL, endR, bridge, riseL, postbase, *posts)
    (sx0, sy0, sz0), (sx1, sy1, sz1) = H01_POST_SADDLE                                    # hr47：两根上立柱的前立板座块（见常量注释）
    saddles = [wbox((sx0, sy0, sz0), (sx1, sy1, sz1)), wbox((sx0, -sy1, sz0), (sx1, -sy0, sz1))]
    new = union(lowerL, endL, endR, bridge, riseL, postbase, *saddles, *posts)
    # hr38 嘴关节（2026-09-23 用户拍板 15 颗全装，duckstructure/jaw.py）：原版 motor_support 毛坯的 XL330 口袋（顶板 z 248.6 / 端墙 x 16.6）
    #   直接装第 15 颗 S288（34×20 同尺寸），照原版由 H01 顶板 + H03 脊上下夹、背面不加板（背面 y<14 全在 N03 横滚扫掠里）。
    #   加：前立板在舵机端面外 x 50.9..53 加厚（让位后只剩 1.1）+ Radxa 右下立柱座块。切：servo_env（顶板 1.7→1.4、端墙、前立板开口）、
    #   惰轮座上半拱 Ø15.0→Ø15.1 对中 238.615（6700ZZ）、轴颈尖端 Ø11 让位、上插座线翘区在 B 座板 x 40.1 角上刮 0.5。见 jaw.py 文件头。
    from . import jaw as JAW
    new = union(new, *JAW.h01_adds())
    # 新料自动避开上下头壳（±0.4）。**原版毛坯不参与这一步**：它 4 只脚本来就贴着壳面 231.51，削 0.4 就悬空了
    shells = []
    for nm in ("top_head_shell", "bottom_head_shell"):            # hr39c：放大后的壳（head_scale）；原来是原版 top/bottom_head_shell
        sh = HS.scaled_orig(nm)
        if nm == "bottom_head_shell":
            # hr39c 修（mechanical_audit「H03_shell insertion」86.7 mm³）：H03 的真实毛坯 = 放大 − H03_CORE_CUT + 原版 ∩ H03_CORE_BOX（颈口核心在真实位置）。
            #   只拿放大毛坯当让位时，新料里 z 222.8..225.3 那块板在左后脚 (3.1, −30) 处让的是**放大后**的凸台（x −3.2..−0.6），
            #   真实凸台 (3.1, −30) 下面反而长出板 → 板在 H03 凸台下面、脚在凸台上面，H03 −z 拆不下来。改成对真实毛坯让位（= hr38 的切法）。
            # was_until_2026_09_28_hr50: sh = union(diff(sh, wbox(*H03_CORE_CUT)), inter(orig("jaw_soft", "bottom_head_shell"), wbox(*H03_CORE_BOX)))
            sh = union(diff(sh, *_h03_blank_cuts()), inter(orig("jaw_soft", "bottom_head_shell"), wbox(*H03_CORE_BOX)))   # hr50：与 H03 毛坯同一套挖法
        shells.append(sh)
        for dv in DILATE6(0.4):
            s2 = sh.copy(); s2.apply_translation(dv); new = diff(new, s2)
    # hr43b（G1，Lane D hr43_登记修正_L1L2.md §4）：上面这步避壳用的是**还没开嘴枢轴孔**的放大原版壳，把 jaw.h01_seat_arch_adds 里 6700 惰轮上半拱
    #   （jaw.seat_ring('upper')：r 7.55..9.3 × y −49.05..−45.05 × z ≥ 245.5）世界前→上 7.5°..105° 那段削掉了：理想拱 897 采样点只 607 有料，缺的 290 点
    #   全在放大原版顶壳 ±0.4 内；可真实 H05/H03 在这里都开了 Ø19.2 孔（jaw.shell_pivot_cuts，径向 0.3），装配后这 290 点谁都没有料。
    #   改（简报二选一取改动最小的「加拱放最后」）：避壳之后把上半拱原样再 union 一次。腹板（y −45.3..−37）避壳后本来就 5004/5004 采样点全在，不动；
    #   后面 cuts 里只有 seat_bore_cut（Ø15.1 = 拱自己的孔）/ journal_clear_cut（Ø11，在孔里）碰得到这里。
    #   改前预演（placed/head_bracket ∪ 上半拱，hr43b dryrun）：897/897 有料，拱 × H05/H03/N04/J03/6700/J01/J02 全 0。
    # was_until_2026_09_25_hr43b: （新增行；改前上半拱只在避壳前随 JAW.h01_adds() 加一次）
    new = union(new, JAW.seat_ring("upper"))
    m = union(O, keep_main(new, "head_bracket_new"))
    cuts = [servo_env(Rr),
            cyl(P["seat_d"], 20.0, (10.0, 0, 235.615), axis="x"),         # A 座 bore Ø22.18
            cyl(P["seat_d"], 20.0, (44.0, 0, 235.615), axis="x"),         # B 座 bore（22×16×4 外圈上半圈）
            cyl(P["seat_d"] + 0.6, 12.0, (50.5, 0, 235.615), axis="x"),   # 前立板开孔：只给 Ø6 拆卸顶杆用（见 docstring ⚠）
            # was_until_2026_09_28_hr50: wbox((12.0, -27.0, 200.0), (42.0, 13.0, 237.6)),              # N03 穿过底壳开口的走廊
            diff(wbox((12.0, -27.0, 200.0), (42.0, 13.0, 237.6)), wbox(*H01_CORRIDOR_KEEP)),   # N03 穿过底壳开口的走廊（hr50 ①：墙边这一块收刀）
            wbox((-15.0, -40.0, 200.0), (-13.0, 40.0, 226.0))]            # 让开 x=-14 的地板筋
    cuts += [cyl(1.7, 12.0, (62.0, sy * hx / 2, 251.0 + sz * hy / 2), axis="x") for sy in (1, -1) for sz in (1, -1)]
    cuts += JAW.h01_cuts()                                                                  # hr38 嘴舵机让位 + 惰轮座（见上）
    # 头横滚舵机 PH2.0 插座（keepouts.yaml:KO01，09-20 背插模型，lib.CONN_MODE "free"）：两个插座在舵机背面（世界 x −17.7 那一面）两角，
    # 世界 (x −19.7..−17.5, y −14.4..−4.2, z 225..231 / 238..248)，插头顶/线弯区和两侧出线区对 H01/H02/H03/上壳全空（hr15 探针），不出刀。
    # 旧 CF4（+y 侧啃 lower 托板 / H03 开可见孔）作废：那是侧插模型的事。
    # 右前脚 Ø4.6 起子通道（KO09）：09-12 第二批（I3）挪到最后一刀 —— 原来在 cuts 里先切、foot（顶 233.2146）后 union 又把
    # 通道底 0.0146 填回去，剩 0.159 mm³；加上 64 边内接 vs Gate 128 边外接的 0.0035 径向差整段壳 0.562 mm³ → KO09 恒红 0.721。
    # 现在：通道刀在 foot/lipB union 之后、且 Ø4.65（平边 2.3222 ≥ Gate 顶点 2.3007）。见函数末尾。
    cuts.append(cyl(2.7, 8.0, (49.1, 9.35, 231.0)))   # 前立板不能填回原版小孔
    # hr29：IMU 线在主控端只能用 2.54 单针杜邦母壳（14 长）套 Radxa 排针 3/4/5/6 脚（SDA/5V/SCL/GND）—— 口袋只有 14 深（x 52..66），壳 14 + 座 2.5 超 2.5；
    #        IDC 座两端各伸 3.5 会撞 Ø5 立柱（立柱边离端针 1.9）。→ 前立板顶边（262）开 7×5 通缺口（x 46.5..52.5，z 257.6 起顶边敞开），杜邦块从上面放下去套针，
    #        壳尾穿出 2.5，线在板后（x<47，射线核到 x 25 全空）拐弯。±y 各开一个（对称，Radxa 装板方向定了只用一侧；pin1 在 +y 端时用 +y 侧）。
    # hr38：−y 侧缺口从 −23.8 放到 −26.0 —— I²S3_M0 的 LRCK/SDI/SDO/GND 在 pin35/38/40/39（y −19.05 / −21.59 / −24.13 / −24.13，
    #   杜邦壳 ±1.27 → 到 −25.40），原 −23.8 兜不住 pin39/40。另加 +y 侧 pin12(I2S3_SCLK_M0, y +11.43) 的单独缺口：
    #   Radxa ZERO 3W 官方 40PIN 表（docs.radxa.com/en/zero/zero3/hardware-design/hardware-interface）里 I2S3 只有 M0 一组齐活
    #   （M1 缺 LRCK：19 SCLK / 21 SDO / 23 MCLK / 24 SDI，表里没有 I2S3_LRCK_M1），所以换针位这条路不通，只能开缺口。
    # hr39c（hr39_头内线束.md §4.3 硬伤③）：Radxa 排针按官方图高 2.7 后，内排针（奇数，z 261.43）pin 1/9/17/25 的杜邦壳（±1.27 → 底 260.16）顶在前立板顶边（262）上
    #   （hr39 实测 pin 1/9/17/25 壳撞 H01 7–12 mm³）→ 三个分段缺口合成一整段：x 46.5..52.5 × |y| ≤ 26 × z ≥ 257.6。|y| 26..31 留给两根 Radxa 立柱。
    H01_FRONT_NOTCH_was_until_2026_09_23_hr39c = (((46.5, 16.8, 257.6), (52.5, 23.8, 270.0)), ((46.5, -26.0, 257.6), (52.5, -16.8, 270.0)), ((46.5, 9.4, 257.6), (52.5, 13.5, 270.0)))
    # was_until_2026_09_28_hr50: cuts += [wbox((46.5, -26.0, 257.6), (52.5, 26.0, 270.0))]
    cuts += [diff(wbox((46.5, -26.0, 257.6), (52.5, 26.0, 270.0)), wbox(*H01_NOTCH_KEEP))]      # hr50 ②：嘴舵机包络上方那段缺口底抬到 258.75
    cuts += [wbox(*b) for b in H01_SQUEEZE_CUTS]                                                 # hr50 ③：夹在 H03 两个实体之间的薄边 / 鳍削穿
    cuts += [wbox(*H01_ARCH_BAND_CUT)]                                                          # hr50 ⑤：N03 扫掠掏空后的旧拱罩残带 / 锯齿腿顶
    # 4 个脚孔用**原版自带的** Ø2.70（我们不再另钻，免得钻到旁边去）
    # N03 的偏航背板顶到 z=257.5（原版 yaw_roll_motion 只到 249.1），横滚 ±25° 时会撞原版环的顶圈 → 反向扫掠让位
    # 头偏航舵机的两个插头 + 背板通窗里的线（09-20 背插模型，lib.CONN_MODE "window"：世界 x 15.5..21 / 31..36.5，y −14.4..−4.2，z 253.6..259.1）
    # 也随 N03 一起转，一并扫掠。旧 CF3 侧走廊（x 10.5..16.5、z 244.6..250.1 啃上桥 +x 边）作废。
    Ry = sfw("yaw_roll_motion", 0)
    yrm_hi = inter(union(build_yrm(), servo_env(Ry), conn_cut(Ry, mode="window")),
                   wbox((-5.0, -30.0, 242.0), (60.0, 30.0, 275.0)))
    # was_until_2026_09_28_hr50: yrm_hi_clearance = sweep_of(yrm_hi, "jaw_soft", -25, 25, 41, grow=0.4)
    # hr50（hr50 公共规矩：运动扫掠的锯齿台阶 → mink 连续膨胀后再转）：DILATE6 的 6 个平移副本 × 41 个角度在 −y 旧拱罩 / 两腿顶留下台阶碎边 + 尖刺；
    #   改成先 minkowski_box(±0.4) 连续膨胀再转（同 ear_clearance / H03 偏航座扫掠），各向 ≥0.4 不变（凸角处多让 ≤0.29）。
    yrm_hi_clearance = sweep_of(yrm_hi, "jaw_soft", -25, 25, 41, grow=0.4, mink=True)
    cuts.append(yrm_hi_clearance)
    # hr39c：N03/N08 两颗耳挪到 120° / −60°（head_bearing_rebuild.HEAD_YAW）后，120° 那颗 (16, 17.3, z 218.7..227.3) 绕头横滚 +20..+24° 扫进
    #   H01 老嘴口袋端墙（原版 motor_support 的 XL330 口袋端墙 x 14..16.6；新嘴舵机在 x ≥ 22.3，有自己的端墙 jaw.h01_adds）37.5 mm³（hr39c_work/stackvol.py）
    #   → 两颗耳（N03 耳 + N08 盖耳）按头横滚 ±30、grow 0.45 让位。
    ear_zones = [inter(union(HBR.yaw_carrier_features(), HBR.build_head_yaw_cap()), wbox(lo, hi))
                 for lo, hi in (((8.0, 10.0, 215.0), (24.0, 25.0, 230.0)), ((28.0, -25.0, 215.0), (44.0, -10.0, 230.0)))]
    ear_clearance = sweep_of(union(*ear_zones), "jaw_soft", -30, 30, 61, grow=0.45, mink=True)
    cuts.append(ear_clearance)
    m = diff(m, *cuts)
    # B 座的前方孔同样切穿这只脚；保留原版承压圈。N03 从下方装入，不经过此脚。
    # 09-14：承压圈只给 A 端 6 条起子刀路本身让位（Ø H01_FOOT_TOOL_D 沿 x，刀长 8 > 脚盘 Ø6.4 的 x 跨度；位置从 Rr/horn_r 推，与 mechanical_audit 同源），
    # 不再整圈削到 r 7.4（那样孔壁到削面只剩 1.02，见文件头注释）
    assert abs(Rr[0, 0] - 1.0) < 1e-9, "头横滚舵机局部 x 应与世界 x 同向"
    tool_cuts = [cyl(H01_FOOT_TOOL_D, 8.0, (49.1, float(c[1]), float(c[2])), axis="x")
                 for a in np.linspace(0, 2 * math.pi, S["horn_n"], endpoint=False)
                 for c in [pt(Rr, 0.0, S["horn_r"] * math.cos(a), S["horn_r"] * math.sin(a))]]
    foot = diff(inter(O, cyl(6.4, 1.7, (49.1, 9.35, 232.3646))), *tool_cuts)
    # B 端唇（上半圈）：座刀之后再 union，否则被 Ø22.18 刀削掉；和新料一样自动避开上下头壳 ±0.4
    li, lo, lx0, lx1 = H01_LIP_B
    lipB = inter(diff(cyl(lo, lx1 - lx0, ((lx0 + lx1) / 2, 0, 235.615), axis="x"), cyl(li, lx1 - lx0 + 0.3, ((lx0 + lx1) / 2, 0, 235.615), axis="x")),
                 wbox((lx0 - 5, -20, 235.615), (lx1 + 5, 20, 260)))
    for sh in shells:
        for dv in DILATE6(0.4):
            s2 = sh.copy(); s2.apply_translation(dv); lipB = diff(lipB, s2)
    m = HBR.head_seats(union(m, foot, lipB, *[cyl(5.0, 6.0, (-9.0, y, z), axis="x") for y, z in HEAD_CLAMP_MOUNTS]), upper=True)
    # Final bearing-seat union must not refill CF3 or the moving N03 envelope.
    m = diff(m, yrm_hi_clearance, ear_clearance)
    # was_until_2026_09_28_hr50: m = diff(m, cyl(4.6 + KO_OVER, 40.0, (49.1, 9.35, 253.2)),          # KO09 通道：最后一刀，之后不许再 union 任何东西回填
    # was_until_2026_09_28_hr50:          *[cyl(1.6, 4.5, (-9.75, y, z), axis="x") for y, z in HEAD_CLAMP_MOUNTS])
    m = diff(m, cyl(4.6 + KO_OVER, 40.0, (49.1, 9.35, 253.2)),          # KO09 通道：最后一刀，之后不许再 union 任何东西回填
             *[cyl(1.6, 4.5, (-9.75, y, z), axis="x") for y, z in HEAD_CLAMP_MOUNTS],
             # was_until_2026_09_28_hr50_r2:          wbox(*H01_KO09_FACE_CUT))                                  # hr50 ④：通道 −x 边外的楔一并削穿
             wbox(*H01_KO09_FACE_CUT),                                  # hr50 ④：通道 −x 边外的楔一并削穿
             wbox(*H01_NOTCH_CORNER_CUT))                               # hr50_r2 2026-09-28：前缘缺口 +y 端角 0.25 台阶切齐（见常量）
    # hr13：hr05 头部补丁起 A 座后方（x 6.8..8.2、y −24.6..−11.3、z 244.3..246.4）有 1 个 35.33 mm³ 的封闭空腔。复审查明它**不是**扫掠刀留下的
    # （yrm_hi 绕 x 轴转、刀最小 x≈10.0，到不了 x 6.8..8.2）：那是原版 motor_support 两块立板（x 5.6..6.8 / 8.2..9.9）之间、底板 z 242.6..244.3
    # 之上、左纵轨 y≤−24.5 之内**原有的槽**，被我们的上桥 bridge（z 246.4 起）盖了顶、又被 head_seats 的 A 环（r 15.6）封住 +y 口。
    # 埋在壁内 1.2 mm，切片会留空、壁只剩 1.2/1.7。填实无害：没有任何件能到那里（到 N03 4.2、A 轴承 2.1、N05 4.8、偏航舵机 7.8 mm；
    # hr13_local_test：填实区与 N03/偏航舵机/N06/N08/偏航轴承/N02 绕横滚轴 ±25° 扫掠全 0），填实不新增任何外表面。
    return keep_main(m, "head_bracket", fill_cavities=True)

# 09-14 复审 MAJOR：垫柱 3.2 时压板内面在舵机局部 −13.2，离背面惰轮端面 −13.0（手册 Ø14×3，与厚段背齐平）只有 0.2（<0.3；旧 2.9 副轴时 0.35）——
# 压板 y 4..11.5 盖住惰轮盘 Ø14 的一段弦，惰轮是自由转动的副轴、压板是载体侧静止件。垫柱 3.2 → 3.4：压板内面 −13.4，离惰轮面 0.4；叠厚 5.8，M2×8 咬 2.2。
H02_POST_L, H02_PLATE_T, H02_CB = 3.4, 3.0, 0.6     # 垫柱长 / 压板厚 / 头窝深 → 上排背孔叠厚 = 3.4+3.0−0.6 = 5.8（F18，M2×8 咬 2.2；旧 3.2/5.6/咬 2.4；mechanical_audit H02 screw seats 按它判）
H02_ARM_X = (-18.15, -12.0)                          # 连接臂 x：外端 min(−18.15, 压板内面−0.05) 压进压板做真·体积重叠，内端贴 H01 的 Ø5 座（x=-12）；
                                                     # 臂上两颗头的坐面统一铣到 H02_ARM_X[0]+0.05=−18.1 → 臂叠厚 6.1（M2×10 进 H01 3.9），不随压板站位变
def build_head_clamp():
    """H02 可拆压板，保持关节轴线及舵机零位不变。
    两根垫柱直接接触名义薄段背面 x=舵机心−T/2=-14.7（09-14 SERVO_SHIFT −1.7；旧 -14.9/-14.85）；压板/垫柱/头窝全部从这个面往 −x 推：M2×8，叠厚 3.4+3.0−0.6=5.8，咬 2.2
    （09-14 垫柱 3.2→3.4：压板内面离背面惰轮端面 0.2→0.4）。
    连接臂在 x=-12 与 H01 接触；两颗 M2×10 穿 6.10，进塑料 3.90（底孔 4.5）。
    先装横滚舵机和A端六颗螺丝，再从 -x 装压板；头壳最后合上。"""
    R = drv_self("jaw_soft")
    holes = [pt(R, -S["T"] / 2, sy * S["mnt_dx"], S["mnt_z"][0]) for sy in (1, -1)]
    xb = float(holes[0][0])                                   # -14.7 舵机薄段背面（世界 x；09-14 前 -14.9）
    xi, xo = xb - H02_POST_L, xb - H02_POST_L - H02_PLATE_T   # 压板内面 -18.1 / 外面 -21.1（09-14：舵机 +0.2、垫柱 +0.2，站位不变；惰轮面 -17.7）
    plate = wbox((xo, 4.0, 225.0), (xi, 11.5, 246.5))
    posts = [cyl(5.0, H02_POST_L + 0.05, (xb - (H02_POST_L + 0.05) / 2, o[1], o[2]), axis="x") for o in holes]   # 垫柱 xb..压板内面，压进板 0.05
    arm = wbox((min(H02_ARM_X[0], xi - 0.05), 9.8, 232.0), (H02_ARM_X[1], 15.8, 246.0))
    cuts = [cyl(S["mnt_hole_d"], 15.0, (-20.0, o[1], o[2]), axis="x") for o in holes]              # 09-13：2.2 → mnt_hole_d(2.4)
    cuts += [cyl(P["m2_cbore_d"], H02_CB + 0.02, (xo - 0.02 + (H02_CB + 0.02) / 2, o[1], o[2]), axis="x") for o in holes]   # 头窝从板外面沉 0.6（09-13：4.2 → m2_cbore_d(4.4)）
    # 09-14 复审 R3a BLOCKER：原版这里是 cyl(mnt_hole_d, 15.0, (-20.0, y, z)) → 刀 x −27.5..−12.5，而臂内端 H02_ARM_X[1] = −12.0，
    # 臂端留 0.5 实心膜、M2×10 装不进 H01 底孔。改成从 H02_ARM_X 推：刀 x 从臂外端再外 3（盖住头窝层）到臂内端再进 1.0（穿透 ≥1 mm），不写死。
    ax_lo, ax_hi = H02_ARM_X[0] - 3.0, H02_ARM_X[1] + 1.0                                     # −21.15 .. −11.0
    cuts += [cyl(S["mnt_hole_d"], ax_hi - ax_lo, ((ax_lo + ax_hi) / 2, y, z), axis="x") for y, z in HEAD_CLAMP_MOUNTS]
    # 后方主板在部分圆周遮住螺丝头；统一铣到 arm 外表面（H02_ARM_X[0]+0.05=-18.1），不能用半圈坐面算叠厚。
    cuts += [cyl(4.6, 10.0, (H02_ARM_X[0] + 0.05 - 5.0, y, z), axis="x") for y, z in HEAD_CLAMP_MOUNTS]
    return keep_main(diff(union(plate, *posts, arm), *cuts), "head_clamp")

# ── hr32（2026-09-22）：H03 给 N02（颈俯仰件，body neck_pitch）让位 ──
#   真实姿态 5601 个里 1502 个 head_bottom_shell×neck_pitch 交叠（走路 1293，峰 55.6 mm³；hr10 起每轮一样，docs/gate/Gate校验_2026-09-22.md §一 1）。
#   两件的相对运动只有 head_yaw × head_roll 两轴（MJCF 链 neck_pitch →(head_yaw) yaw_roll_motion →(head_roll) jaw_soft；颈俯仰/头俯仰两件一起动）。
#   H03 的世界增量 = A_y(θy)·A_r(θr)（父动子跟、绕各自零位轴线，与 tools/gate/layers/l6_motion._Scene.deltas 同法），
#   所以 N02 在 H03 系里 = A_r(−θr)·A_y(−θy)·N02。区间 = frozen.yaml:joint_axes[].target_range_deg（能力包络 + 3°）；网格 5° 与 L6 两两组合同；
#   N02 先 minkowski 连续膨胀 grow（tolerances min_gap 0.3 + 0.1）。逐 θy 把 11 个 θr 副本并起来从 H03 减掉（484 副本一次并集太重）。
HEAD_YAW_TARGET = (-116.4, 95.4)     # frozen.yaml head_yaw target_range_deg（策略实际包络 −94.6..92.3，在内）
HEAD_ROLL_TARGET = (-25.0, 25.0)     # frozen.yaml head_roll target_range_deg（不改；L6 判 BLOCK/WARN 的边界仍是它）
HEAD_ROLL_SWEEP = (-30.0, 30.0)      # hr33：让位刀的横滚扫掠范围按策略实际包络 −28.15..24.04（policy_steps hr32c）外扩到 ±30 —— hr32 真实姿态剩 5 个碰撞全在 −26.8..−28.2°；
                                     #        frozen 目标区间是被上游 MJCF range ±25 夹窄的（能力包络 [−28.2, 26.2]），策略回放早就越过 ±25，件应覆盖策略真会到的地方
N02_SWING_GROW = 0.45                # hr33：0.4 → 0.45，hr32 刀在 (41.7, 8.0, 220.6) 与壳面相切留 1 个非流形顶点（L0 nonmanifold_vertices，复审 m-01）；先例 head.py 偏航座扫掠体同因改 0.45
N02_SWING_STEP = 5.0

def n02_swing_cut(m):
    """从 m（H03，零位世界系）减掉 N02 在 head_yaw 目标区间 × head_roll HEAD_ROLL_SWEEP（±30，hr33）内相对 H03 的扫掠体（见上）。"""
    g = N02_SWING_GROW
    n02 = minkowski_box(build_neck_pitch(), (-g,) * 3, (g,) * 3)
    Ty, Tr = TW("yaw_roll_motion"), TW("jaw_soft")
    ylo, yhi = HEAD_YAW_TARGET; rlo, rhi = HEAD_ROLL_SWEEP     # hr33：横滚按 HEAD_ROLL_SWEEP（±30），偏航仍按目标区间
    ys = [float(a) for a in np.arange(ylo, yhi, N02_SWING_STEP)] + [yhi]
    rs = [float(a) for a in np.linspace(rlo, rhi, int(round((rhi - rlo) / N02_SWING_STEP)) + 1)]
    for ty in ys:
        y = placed(n02, rot(math.radians(-ty), Ty[:3, 2], Ty[:3, 3]))
        m = diff(m, union(*[placed(y, rot(math.radians(-tr), Tr[:3, 2], Tr[:3, 3])) for tr in rs]))
    return m

def mic_pad(m):
    """hr37：H03 地板上的麦克风台（H03-F12）。台 union 到地板顶（重叠 0.2 进地板），三面墙 + −x 唇；芯片沉坑 + Ø1.5 声孔穿地板。"""
    (x0, x1), (y0, y1), zf, zt = MIC_PAD; w, wh, lip = MIC_PAD_WALL, MIC_PAD_WALL_H, MIC_PAD_LIP
    from .electronics import MIC_X, MIC_Y, MIC_T
    pad = wbox((x0, y0, zf - 0.2), (x1, y1, zt))
    walls = [wbox((x0, y0, zt), (x0 + w, y1, zt + wh)),                     # −x 墙
             wbox((x0, y0, zt), (x1, y0 + w, zt + wh)),                     # −y 墙
             wbox((x0, y1 - w, zt), (x1, y1, zt + wh))]                     # +y 墙
    lipb = wbox((x0 + w, y0, zt + MIC_T + 0.2), (x0 + w + lip, y1, zt + wh))   # −x 唇：PCB 顶 (zt+1.6) 上留 0.2，压住 PCB 边 0.6
    cx, cy = (MIC_X[0] + MIC_X[1]) / 2, (MIC_Y[0] + MIC_Y[1]) / 2
    sink = cyl(MIC_CHIP_D, MIC_CHIP_DEPTH + 0.01, (cx, cy, zt - MIC_CHIP_DEPTH / 2 + 0.005))
    hole = cyl(MIC_SOUND_D, (zt - zf) + 4.0, (cx, cy, (zt + zf) / 2 - 1.0))
    # hr38：两排短针让位槽（见 MIC_PIN_SLOT_* 注释）
    hx = 3.81 + MIC_PIN_SLOT_XPAD
    # was_until_2026_09_28_hr50: pin_slots = [wbox((cx - hx, yc - MIC_PIN_SLOT_W / 2, zt - MIC_PIN_SLOT_DEPTH), (cx + hx, yc + MIC_PIN_SLOT_W / 2, zt + 0.5))
    # hr50：槽 +x 端 cx+hx = 91.98 离台 +x 端 x1 = 92.17 只剩 0.19（薄膜 10.7 / 10.6 mm²）；收刀会碰端针（针到 cx+4.13）→ 槽从台端穿出 0.3（台外 z ≥ 233.86 没有料）。
    pin_slots = [wbox((cx - hx, yc - MIC_PIN_SLOT_W / 2, zt - MIC_PIN_SLOT_DEPTH), (max(cx + hx, x1 + 0.3), yc + MIC_PIN_SLOT_W / 2, zt + 0.5))
                 for yc in (MIC_Y[0] + 1.27, MIC_Y[1] - 1.27)]
    # was_until_2026_09_25_hr43d: return diff(union(m, pad, *walls, lipb), sink, hole, *pin_slots)
    fm = MIC_FOAM                                                                             # hr43d：泡棉胶位（见 MIC_FOAM）
    foam = wbox((x0 + w, MIC_Y[0] + fm["band_y_in"], zt - fm["sink"]), (x1 + 0.01, MIC_Y[1] - fm["band_y_in"], zt + 0.5))
    return diff(union(m, pad, *walls, lipb), sink, hole, *pin_slots, foam)


def ubec_pad(m):
    """hr38：H03 地板 −y 侧的 UBEC 座（H03-F13）。台 union 到地板顶（重叠 0.2 进地板），四面墙围住模块，两条扎带槽穿台+地板。"""
    (x0, x1), (y0, y1), zf, zt = UBEC_PAD
    (mx0, mx1), (my0, my1), (mz0, mz1) = UBEC_MOD
    w, wh = UBEC_WALL, UBEC_WALL_H
    pad = wbox((x0, y0, zf - 0.2), (x1, y1, zt))
    walls = [wbox((mx0 - w, my0 - w, zt), (mx0, my1 + w, zt + wh)),        # −x 墙
             wbox((mx1, my0 - w, zt), (mx1 + w, my1 + w, zt + wh)),        # +x 墙
             wbox((mx0 - w, my0 - w, zt), (mx1 + w, my0, zt + wh)),        # −y 墙
             wbox((mx0 - w, my1, zt), (mx1 + w, my1 + w, zt + wh))]        # +y 墙
    tw, tg, tl = UBEC_TIE_X["w"], UBEC_TIE_X["gap"], UBEC_TIE_X["len_y"]; yc = (my0 + my1) / 2
    slots = [wbox((xa, yc - tl / 2, zf - 2.5), (xb, yc + tl / 2, zt + 0.5)) for (xa, xb) in ((mx0 - w - tg - tw, mx0 - w - tg), (mx1 + w + tg, mx1 + w + tg + tw))]
    # hr39c（agent #4）：两头出线 —— 模块抬高 UBEC_RAISE 后底边坐在 ±x 两条托条上（x 各 0.8 宽、y 全长、z 台顶..模块底）；托条之间（x 宽 4.4）是底端线的弯线区，
    #   ±y 两面墙在这一段开口（x 同托条之间、z 台顶..模块底 + 0.5），底端那组线弯 90° 后从 +y（或 −y）口出。只从顶端出线的模块照样坐得住（顶端上方到内顶 ≥12）。
    lw = UBEC_LEDGE_W
    # was_until_2026_09_25_hr43d: ledges = [wbox((mx0, my0, zt - 0.2), (mx0 + lw, my1, mz0)), wbox((mx1 - lw, my0, zt - 0.2), (mx1, my1, mz0))]
    zl = mz0 - UBEC_FOAM["sink"]                                                              # hr43d：托条顶下沉 0.8 放 1 mm 泡棉（见 UBEC_FOAM）
    ledges = [wbox((mx0, my0, zt - 0.2), (mx0 + lw, my1, zl)), wbox((mx1 - lw, my0, zt - 0.2), (mx1, my1, zl))]
    notches = [wbox((mx0 + lw, yy0, zt), (mx1 - lw, yy1, (zt + wh + 0.5) if sgn in UBEC_NOTCH_OPEN_TOP else (mz0 + 0.5)))
               for sgn, (yy0, yy1) in ((-1, (my0 - w - 0.5, my0 + 0.01)), (1, (my1 - 0.01, my1 + w + 0.5)))]
    return diff(union(m, pad, *walls, *ledges), *slots, *notches)


def build_face_plate():
    """H04（hr52）：hr41 版（build_face_plate_was_until_2026_09_29_hr52）+ 左脸 ToF「v6-VL53L5CX 雷达」贴面座（常量 / 几何全在 duckstructure/tof.py）：
    板投影外扩 0.3 的 0.2 实心平台（板正面整片贴平、芯片顶面与脸面 93.47 齐平）+ 两耳 Ø1.8×1.2 定位销 + 两根 Ø5 压条柱（顶 88.3，Ø1.7 底孔到 92.9（hr52b；复审 r1 m3））；
    芯片 7.6×4.2 r0.4 通槽（hr52d；中心 b −2.2）+ 焊锡包 0.7 深槽 + 排针通透长条 14.5×2.5（hr52c：针脚不剪）。H10 压条（build_tof_clamp）压两只耳朵、2 颗 M2×6 拧进这两根柱。"""
    m = build_face_plate_was_until_2026_09_29_hr52()
    m = union(m, TOF.to_world(TOF.h04_add_local()))
    m = diff(m, TOF.to_world(TOF.h04_cut_local()))
    return clean_print_topology(keep_main(m, "face_plate"))


def build_tof_clamp():
    """H10（hr52 新件）：ToF 压条（duckstructure/tof.py CLAMP，同 H06 做法）。1.8 厚、2.4 宽的条从左柱压过左耳到右耳、再一段腿下到右柱；
    两只 Ø3.4 脚压耳朵背面（板背 90.37）；两端 Ø5.4 盘坐柱顶，比柱顶 88.3 退 0.2 → 拧紧时压条微弯、一直压着板；Ø2.4 过孔，2 颗 M2×6 从背面（−x）拧。"""
    return keep_main(TOF.to_world(TOF.clamp_local()), "tof_clamp")


def build_face_plate_was_until_2026_09_29_hr52():
    """H04（hr41）：原版脸板那层平板 + 填孔 + 麦克风浅槽（hr41：下开口竖槽，复审 B1/m 麦克风 PCB 随 H03 竖直上来不再顶槽下沿）；
    摄像头改贴板（见 H04_CAM_* 注释）：r 11.5 通窗 + 0.8 浅坑 + 四角贴合面/定位销 + 两个压框凸台。原版件 CC BY-SA-NC。
    改前 = build_face_plate_was_until_2026_09_24_hr41。hr52 起由 build_face_plate 调用后再加 ToF 座。"""
    face = HS.scaled_orig("face_part"); face.merge_vertices()
    plate = inter(face, wbox((H04_PLATE_X[0], -70, 200), (H04_PLATE_X[1] + 5, 70, 320)))
    x0, x1 = H04_PLATE_IN, H04_PLATE_IN + 1.3
    fills = [wbox((x0, y0, z0), (x1, y1, z1)) for (y0, y1), (z0, z1) in H04_FILLS]
    yc, zc = H04_CAM_C
    win = cyl(2 * H04_CAM_WIN_R, 4.0, (x0 + 0.65, yc, zc), axis="x", sections=128)
    pk = H04_CAM_POCKET; h = H04_CAM_PCB / 2 + pk["clr"]; c = pk["corner"]
    pocket = diff(wbox((x0 - 1.0, yc - h, zc - h), (x0 + pk["depth"], yc + h, zc + h)),
                  *[wbox((x0 - 1.5, yc + sy * h - (c if sy > 0 else 0), zc + sz * h - (c if sz > 0 else 0)),
                         (x0 + pk["depth"] + 0.5, yc + sy * h + (0 if sy > 0 else c), zc + sz * h + (0 if sz > 0 else c))) for sy in (1, -1) for sz in (1, -1)])
    pp = H04_CAM_PIN
    pins = [cyl(pp["d"], pp["h"] + 0.05, (x0 - pp["h"] / 2 + 0.025, yc + sy * pp["pitch"] / 2, zc + sz * pp["pitch"] / 2), axis="x", sections=32)
            for sy in (1, -1) for sz in (1, -1)]
    cb = H04_CLAMP_BOSS
    bosses = [cyl(cb["d"], x0 + 0.05 - cb["x_top"], ((cb["x_top"] + x0 + 0.05) / 2, yc + sy * cb["y"], zc), axis="x") for sy in (1, -1)]
    bpil = [cyl(cb["pilot_d"], cb["pilot_bottom_x"] - cb["x_top"] + 0.5, ((cb["x_top"] - 0.5 + cb["pilot_bottom_x"]) / 2, yc + sy * cb["y"], zc), axis="x") for sy in (1, -1)]
    (sx0, sx1), (sy0, sy1), (sz0, sz1) = H04_MIC_SLOT
    slot = wbox((sx0, sy0, sz0 - 6.0), (sx1, sy1, sz1))                    # hr41：下开口（原 sz0..sz1 封底 → 往下开通到板底以下）
    m = diff(union(plate, *fills), win, pocket)
    m = union(m, *pins, *bosses)
    m = diff(m, *bpil, slot)
    return clean_print_topology(keep_main(m, "face_plate"))


def build_camera_clamp():
    """H06（hr41 新件）：摄像头内侧压框。两根竖条（|y| 12.5..16）从背后压 PCB 四角（四个 4.5 高压块前面 = PCB 背面 90.57），
    两只耳朵各 1 颗 M2×8 自攻拧进 H04 凸台（H04_CLAMP_BOSS）。竖条离 PCB 背面 3.37，避开背面顶边 FPC 翻盖座（|y| ≤ 10.05）。"""
    k, cb = H06_CLAMP, H04_CLAMP_BOSS; yc, zc = H04_CAM_C; h = H04_CAM_PCB / 2
    out = []
    for sy in (1, -1):
        ya, yb = sorted((yc + sy * k["bar_y"][0], yc + sy * k["bar_y"][1]))
        out.append(wbox((k["bar_x"][0], ya, zc - h), (k["bar_x"][1] + 0.01, yb, zc + h)))                       # 竖条
        for sz in (1, -1):
            za, zb = sorted((zc + sz * h, zc + sz * (h - k["pad_h"])))
            out.append(wbox((k["pad_x"][0], ya, za), (k["pad_x"][1], yb, zb)))                                   # 四角压块
        ea, eb = sorted((yc + sy * (k["ear_y"][0] - 0.01), yc + sy * k["ear_y"][1]))
        out.append(wbox((k["ear_x"][0], ea, zc - k["ear_z"]), (k["ear_x"][1], eb, zc + k["ear_z"])))            # 耳朵
    out.append(wbox((k["bar_x"][0], yc - k["bar_y"][1], zc - h), (k["bar_x"][1], yc + k["bar_y"][1], zc - h + k["bottom_h"])))   # 底横梁（顶边有 FPC 翻盖座 + CSI 出线，不做顶梁）
    holes = [cyl(2.4, 6.0, (k["ear_x"][0], yc + sy * cb["y"], zc), axis="x") for sy in (1, -1)]
    return keep_main(diff(union(*out), *holes), "camera_clamp")


def build_face_plate_was_until_2026_09_24_hr41():
    """H04：脸板（替换原版 face_part；hr37）。原版脸板那层平板原样保留（外形 = 顶壳凹槽 + H03 前唇槽的配合面），
    填掉原版镜孔与右侧小孔，开 8M 219 镜座 13.6 方孔，背面 4 根 Ø4.5 立柱（孔距 28×28，Ø1.7 底孔深 5，M2×4 自攻），
    麦克风 PCB +x 边的 0.5 深浅槽。原版脸板背面的立柱/凸台不要 —— 那是原版 M12 摄像头的座。
    hr39c：毛坯 = head_scale 放大后的 face_part（板层 x 92.17..93.47，厚 1.3 保持）；方孔/立柱/浅槽跟摄像头、麦克风走（T_CAM / T_MIC）。
    原模型来自 Pollen Robotics Microduck，沿用原资产的 CC BY-SA-NC 许可。"""
    face = HS.scaled_orig("face_part"); face.merge_vertices()
    plate = inter(face, wbox((H04_PLATE_X[0], -70, 200), (H04_PLATE_X[1] + 5, 70, 320)))
    x0, x1 = H04_PLATE_IN, H04_PLATE_IN + 1.3
    fills = [wbox((x0, y0, z0), (x1, y1, z1)) for (y0, y1), (z0, z1) in H04_FILLS]
    yc, zc = H04_CAM_C; h = H04_CAM_HOLE / 2; p = H04_CAM_HOLE_PITCH / 2
    hole = wbox((x0 - 1, yc - h, zc - h), (x1 + 1, yc + h, zc + h))
    posts = [cyl(H04_POST_D, x0 + 0.05 - H04_POST_X0, (H04_POST_X0 + (x0 + 0.05 - H04_POST_X0) / 2, yc + sy * p, zc + sz * p), axis="x")
             for sy in (1, -1) for sz in (1, -1)]
    pilots = [cyl(H04_PILOT_D, H04_PILOT_DEPTH + 0.5, (H04_POST_X0 - 0.5 + (H04_PILOT_DEPTH + 0.5) / 2, yc + sy * p, zc + sz * p), axis="x")
              for sy in (1, -1) for sz in (1, -1)]
    (sx0, sx1), (sy0, sy1), (sz0, sz1) = H04_MIC_SLOT
    slot = wbox((sx0, sy0, sz0), (sx1, sy1, sz1))
    from . import eye as EYE                                   # hr39c：圆眼装饰白外圈的 3 个定位销孔（Ø2.0 通孔，duckstructure/eye.py）
    m = diff(union(plate, *fills, *posts), hole, *pilots, slot, *EYE.h04_pin_holes())
    return clean_print_topology(keep_main(m, "face_plate"))


# ── hr39c H03：放大毛坯 + 真实位置核心
#   放大毛坯里的内部筋/凸台/座块跟着地板水平拉开了 s 倍（四个 F22 凸台挪了 5–6.5 mm、颈口开口也跟着放大、两个凸台落到开口边上/开口里）。
#   颈口周围这一块（四凸台、A/B 下半座块、框肋、颈口地板）本来就是按不放大的 H01 / 那摞件 / 轴承设计的 → 直接拿**原版**这块放回真实位置：
#     ① 先把放大毛坯在 H03_CORE_CUT（地板顶 221.35 以上、x −0.5..41.5、|y| ≤ 38）里的内部筋挖掉（地板和外壁不在这个盒里）；
#     ② 再并上 原版 ∩ H03_CORE_BOX（地板层取 219.7 以上，不低于放大后的底面 219.6，外表面不冒台阶）。
#   之后照 hr38 的刀序：横滚舵机/H02/座孔/合缝/那摞件扫掠/N02 摆动/麦克风台/UBEC 座/嘴 —— 这些刀本来就是真实位置的。
# hr50（2026-09-28 用户：比一条挤出线还薄的薄片不要，默认收刀、会碰才削穿）：|y| ≤ 38 只切到放大毛坯里那道「原版框肋（核心里 y≈∓30 真实位置有）的放大副本」
#   靠里的一侧，肋在 |y| 38.0..39.3 剩 0.0..1.0 的一层（clean_check：290 / 158 / 46 mm² 三片薄膜、(37.0, 38.0, 229.4) 尖刺）。这道肋按上面 ① 本来就该挖，
#   y 边界放到 ±39.4 把整道肋吃掉（hr50_work/head_films：肋最外 |y| 39.3；x 41.1..41.5 肋尾到 39.6 的 0.24 mm³ 由 x1 面截平）。H01 避壳同步（_h03_blank_cuts）。
# was_until_2026_09_28_hr50: H03_CORE_CUT = ((-0.5, -38.0, 221.35), (41.5, 38.0, 240.0))
H03_CORE_CUT = ((-0.5, -39.4, 221.35), (41.5, 39.4, 240.0))
# hr50：原版 +y 右后脚 (6.6, 19) 凸台的放大副本（孔心放大后 (2.3, 23.2)，在 H03_CORE_CUT 里）在 x −0.5 面外剩一片月牙（x −1.72..−0.5、y 20.2..26.1、z 到 231.7），
#   两头楔尖 = 刀片 (−0.6, 20.8 / 25.5, 226.8)。真实凸台在核心里 → 月牙同 ① 一并挖掉（盒内放大毛坯只有这片月牙，37.9 mm³）。
H03_CORE_CUT_F2 = ((-2.0, 19.9, 221.35), (-0.4, 26.4, 240.0))
# hr50：放大毛坯里原版 B 下半座块两端的放大副本（x 46..50.5、|y| 8.6..16、顶到 238.3）伸进 H01 前立板（x 47..52）的位置：H01 新料避它（±0.4）
#   在前立板底部挖出两个 U 口，H03 这两块又被 H01 让位 0.3 夹成 0.4..0.7 的肋 / 块（H03 薄膜 14.8 mm²、H01 薄膜 3.1 mm²、两件碎边 ×5）。
#   真实 B 座在核心里，这两块是副本 → 前立板底面（凸台顶 231.52）以上、前立板让位范围（x 47..52 ±0.3、|y| ≤ 16.2）里的放大毛坯一并挖掉；
#   H01 前立板不再绕它（U 口没了），H03 这里只剩 231.52 以下（H01 让位刀照旧再切到板底 −0.3）。
H03_FRONT_PLATE_SLAB = ((46.7, -16.2, 231.52), (52.3, 16.2, 239.0))
def _h03_blank_cuts():
    """放大毛坯里要挖掉、换成真实位置核心的部分（H03 毛坯与 H01 避壳共用）"""
    return [wbox(*H03_CORE_CUT), wbox(*H03_CORE_CUT_F2), wbox(*H03_FRONT_PLATE_SLAB)]
H03_CORE_BOX = ((0.0, -33.5, 219.7), (46.0, 33.5, 235.62))
# hr50（任务：H03 旧脚孔）：放大毛坯里原版 F22 脚孔被位移场挪走的残留（不是设计螺丝孔：features.yaml H03-F02/F20/F24/F25 都不在这里；真实脚孔在 HEAD_FEET 由 fills/pilots 重做）——
#   (3.1,−30)→(−1.9..−2.4, −36.5..−38.7)：斜孔 Ø2.5..2.8，+x 侧被 H03_CORE_CUT 的 x −0.5 面切开，孔壁剩 0.05..0.25（薄膜 25.6 mm²）；
#   (42.1,−30)→(45.7..47.4, −36.7..−36.5)：完整盲孔；(49.1,9.35)→(54.0..55.0, 11.4)：完整通孔（+x 孔壁楔尖 = 刀片 (55.3, 11.4, 226.2)）。
#   (6.6,19)→(2.3,23.2) 在 H03_CORE_CUT 里随毛坯挖掉，不用填。填法：原版孔腔（孔心 1.45 以内的全部顶点 = Ø2.2 孔身 + Ø2.8 头窝 + 孔底 / 孔口）在放大网格里的同号顶点
#   （head_scale 位移场不改拓扑）取凸包 = 放大后的孔腔本身，只径向外扩 0.05 压进孔壁；两端面就是孔底 / 孔口所在的面，不冒出凸台。
H03_OLD_FEET_ORIG = ((3.1, -30.0), (42.1, -30.0), (49.1, 9.35))
H03_OLD_FOOT_R, H03_OLD_FOOT_GROW = 1.45, 0.05
_OLD_FILLS = {}
def _h03_old_foot_fills():
    if "f" not in _OLD_FILLS:
        import trimesh
        O = orig("jaw_soft", "bottom_head_shell"); Sm = HS.scaled_orig("bottom_head_shell")
        V0, V1 = np.asarray(O.vertices, float), np.asarray(Sm.vertices, float)
        assert len(V0) == len(V1), "head_scale 放大网格与原版顶点应一一对应"
        out = []
        for (x, y) in H03_OLD_FEET_ORIG:
            sel = (np.hypot(V0[:, 0] - x, V0[:, 1] - y) <= H03_OLD_FOOT_R) & (V0[:, 2] > 218.0) & (V0[:, 2] < 232.0)
            P = V1[sel]
            a, b = HS.move(np.array([x, y, 221.5]), "bottom_head_shell"), HS.move(np.array([x, y, 231.5]), "bottom_head_shell")
            u = (b - a) / np.linalg.norm(b - a)
            d = P - a; t = d @ u; rad = d - np.outer(t, u); rn = np.linalg.norm(rad, axis=1, keepdims=True)
            P2 = P + rad / np.maximum(rn, 1e-9) * H03_OLD_FOOT_GROW
            # 盲孔（原版孔底 221.51 在料里）：孔底那圈顶点再沿轴往下压 0.05 进底板（不然凸包底面和孔底共面，布尔后剩 0.015 mm³ 内部空腔）；
            # 通孔（孔底在壳外表面上，孔 4）不压，免得冒出底面。
            V0s = V0[sel]
            if V0s[:, 2].min() > 221.4:
                P2 = P2 - np.outer((t <= t.min() + 0.05).astype(float) * H03_OLD_FOOT_GROW, u)
            out.append(trimesh.convex.convex_hull(P2))
        _OLD_FILLS["f"] = out
    return [m.copy() for m in _OLD_FILLS["f"]]
def _h03_side_lip_cuts():
    """hr50：两侧嘴口下垂唇的刀 —— 沿唇内面走向（绕 z 转 atan(slope) ≈ 4.0°）的盒，见 H03_JAW_SIDE_LIP"""
    k = H03_JAW_SIDE_LIP; (x0, x1), (v0, v1), (z0, z1) = k["x"], k["v"], k["z"]
    th = math.atan(k["slope"]); L = (x1 - x0) / math.cos(th)
    out = []
    for sg in (1, -1):
        b = wbox((-L / 2, v0, z0), (L / 2, v1, z1))                      # 局部：u 沿唇、v 朝外（+y 侧），原点放在唇内面线 x 中点
        xm = (x0 + x1) / 2; ym = k["y58"] + k["slope"] * (xm - 58.0)
        b.apply_transform(rot(th, (0, 0, 1)))
        b.apply_translation((xm, ym, 0.0))
        if sg < 0: b = mirror_y_local(b)
        out.append(b)
    return out
def mirror_y_local(m):
    m2 = m.copy(); m2.apply_transform(np.diag([1.0, -1.0, 1.0, 1.0])); return m2     # trimesh 对负行列式变换自己会翻面
# hr50（主设计 09-28 决定 1：运动件让位 → 整片削穿成平整的槽；嘴最大张角保持 30°）：后脑两侧 J01 后下角（张嘴 20..30° 时离壁 0.46..0.54）掠过的开口，
#   四周壳壁被 J01 扫掠刀斜着削成 0..0.8 的薄边（薄膜 16.9 / 4.8 mm²、刀片 5.4 / 5.1 mm²）。在壳壁自己的坐标系里（法向 n、u ≈ x、v 沿坡向上；
#   +y 侧实测 n = (−0.034, 0.687, −0.726)、坡角 43.5°，−y 侧取镜像）把开口连同壁厚 < 0.8 的一圈切成 L 形平槽：两个盒的四个侧面都垂直于壁，
#   外沿都落在壁厚 ≥ 0.8 处（hr50_work/head_films/wall_map.py、thin08.py 逐侧量）；深 w −3.0..+2.5 把这里的壁整层切穿。−y 侧薄边只到 v 4.6 → 大盒只到 v 5.0。
H03_JAW_REAR_SLOT = dict(n=(-0.0344, 0.6874, -0.7255), o=(20.549, 43.669, 222.518), w=(-3.0, 2.5),
                         boxes={1: (((13.6, 16.3), (2.6, 9.2)), ((15.0, 23.8), (0.9, 6.9))),        # (u 范围, v 范围)，u 用世界 x
                                -1: (((13.6, 16.3), (2.6, 9.2)), ((15.0, 23.2), (0.9, 5.0)))})
def _h03_jaw_rear_slots():
    k = H03_JAW_REAR_SLOT; n = np.array(k["n"], float); n /= np.linalg.norm(n)
    u = np.array([1.0, 0.0, 0.0]); u -= (u @ n) * n; u /= np.linalg.norm(u)
    v = np.cross(n, u); v = v if v[2] > 0 else -v
    o = np.array(k["o"], float); w0, w1 = k["w"]
    T = np.eye(4); T[:3, 0], T[:3, 1], T[:3, 2], T[:3, 3] = u, v, n, o
    out = []
    for sg, bs in k["boxes"].items():
        for (u0, u1), (v0, v1) in bs:
            b = wbox((u0 - o[0], v0, w0), (u1 - o[0], v1, w1)); b.apply_transform(T)
            out.append(b if sg > 0 else mirror_y_local(b))
    return out
# hr50（主设计 09-28 11:1x，精修工具够不着的两处）：
#   ① B 下半座（H03-F03，Ø22.18 × x 40.1..44.2，挡肩唇 44.2..44.9）座口：偏航座扫掠刀（N03 偏航座墙 / N08 盖边 ±25°、+0.45）把下扇区
#      x 40.1..40.95 的座壁从外面整片掏空，只在 |y| 8.3..9.1、z 228.3..229.0 剩一层 0.15 的皮贴着座孔面（刀片 1.3 mm² ×2）——座外刀削出来的楔，
#      背后已空、不受力；补厚要进 N03 扫掠包络 → 平切掉：座口 x 39.9..40.96 里 r ≤ 11.99、z ≤ 229.05 的皮切掉（z 229.05 以上座壁厚、x ≥ 40.96 全座不动）。
#      座孔刀 seatB / 挡肩 / 座深 x 40.1..44.2 常量全不动。
H03_B_MOUTH_CUT = dict(x=(39.9, 40.96), r=11.99, z1=229.05)
#   ② 惰轮下半环腹板（H03-F16 的 web，z 226.0 起）底下：放大毛坯在 y ≈ −48.4、x 30..37.5、z 224.7..226.0 有一道 0.01..0.37 的裂缝（两块料几乎贴住；
#      薄膜 2.36 mm²，t 0.01），不是哪把刀留下的 → 缝填实（盒只盖缝，两侧都是实料，不改外形、不碰腹板支撑面）。
H03_WEB_CRACK_FILL = ((29.5, -48.5, 224.6), (37.0, -48.1, 226.02))
# hr50_r2 2026-09-28：偏航座扫掠刀（yaw_motion_envelope 绕横滚轴 ±25°、81 份、+0.45）在地板开口 +x 边（B 下半座块 −x 端面）留的拼花面：
#   包络圆柱 +x 极点（41.075+0.45 = 41.525）与包络上 x 41.0 平面（+0.45 = 41.45）两族面的 81 份转动副本交错，x 41.45..41.525 之间 0.075 高的碎台阶
#   （clean_check 碎边 (41.8, 1.0, 220.3) n 4178 里 5803/7457 条短锐边在 x 41.3..41.7、z 219.5..221、|y| ≤ 7）。
#   → 一刀平面 x = 41.535（比两族面都再往 +x 0.01）把拼花面整片切平。只在地板层 z ≤ 222.5、|y| ≤ 10.5：z ≤ 222 时 |y| 10.5 已在开口里（盒端不留台阶），
#   盒顶 222.5 上面端面本来就是 41.525 一整面（台阶 ≤ 0.01）。离 B 座孔底（r 11.09 → z 224.525）2.0，离最近螺丝孔壁（H03-F02#3）7.7；
#   在 build #2 件上实测多切 1.61 mm³（全在 x 41.45..41.535 的碎台阶里）。原扫掠刀不动（新刀 ⊇ 旧刀）。
H03_YAW_FACE_CUT = ((38.0, -10.5, 218.3), (41.535, 10.5, 222.5))
# hr50：A 座（6704，非定位端）−x 端削穿到这里（见 build_head_bottom_shell）
H03_SEAT_A_LIP_X0 = 5.0
# hr50：J01 −5..30° 扫掠刀在嘴口一圈留下的放大毛坯下垂唇（地板下面），整片切平（见 build_head_bottom_shell）：
#   前缘：平盒，顶 231.88 = 前缘地板底面；两侧：唇内面 y = ±(50.89 + 0.0701·(x − 58))（x 58..84 逐 2 mm 实测都在这条线上 ±0.01）、地板底面 232.0..232.3 渐低
#   → 沿唇走向转 4.0° 的盒，只罩唇内面外 0.15 起的一条，顶到 232.35（地板底面外缘最多削 0.3 的一条边，地板厚 ~1.8）。
H03_JAW_LIP_CUTS = (((96.0, -54.0, 228.5), (98.5, 54.0, 231.88)),)
H03_JAW_SIDE_LIP = dict(x=(57.0, 86.5), y58=50.89, slope=0.0701, v=(-0.15, 1.6), z=(228.5, 232.35))
# hr50：两侧地板外沿（J01 臂扫掠刀上面、上壳下沿底下）x 74..85 只剩 0.25 的一片（|y| 53.5..54.7、z 233.65..234.1，薄膜 12.0 / 11.4 mm²）；
#   收刀会进臂的扫掠包络 → 削穿：H03 上侧壁外面（沿 x 在 |y| 53.45..53.85 之间）以外、地板顶（233.93..234.14 起伏）以下这片切掉，
#   侧壁外沿根部最多少 0.35 的一条，侧壁里侧仍连着地板。
H03_SIDE_RIM_CUTS = (((74.0, 53.45, 233.3), (85.5, 55.2, 234.2)), ((74.0, -55.2, 233.3), (85.5, -53.45, 234.2)))
# hr50：N02 摆动刀（头偏航 × 头横滚、5° 一步）在颈口两侧核心框肋 / 核心盒 |y| 33.5 截面上留的楔（刀片 24.4 / 16.3 / 14.8 / 11.9 mm²）：
#   收刀会碰 N02 → 削穿：−y 框肋底（x 19.8..32.2，框肋离地板悬空的那段）切到 224.1（厚 ≥0.9 处）；+y 框肋同理只到 x 26.5（x ≥ 27 框肋落在地板上、厚 2.0）；
#   |y| 31.3..33.6 两条核心盒截面上的立楔切到地板顶以上 220.95（摆动面是斜的，楔脚最里到 |y| 31.7；框肋在 |y| ≤ 30.7）。
H03_N02_WEDGE_CUTS = (((19.8, -31.6, 220.95), (32.2, -29.0, 224.1)), ((19.0, 29.0, 220.95), (26.5, 31.7, 224.1)),
                      ((22.5, -33.6, 220.95), (42.0, -31.3, 224.2)), ((21.0, 31.3, 220.95), (34.0, 33.6, 223.0)))
# hr50：H03_FRONT_PLATE_SLAB 下面那截放大肋（x 48.9..50.4、y −9.4..6、z 226..231.5，0.6..0.7 厚，−x 面是斜的；其薄端 = 薄膜点 (50.0, −1.4..6.2, 227.5..229.3)）一并切掉。
H03_H01_NOTCH_RIB_CUTS = (((48.9, -9.4, 226.0), (50.9, 6.0, 231.52)),)
H01_CLR_H03 = 0.3                 # H03 对 H01 的通用让位（四只脚坐的凸台顶面除外）
H01_UP_SWEEP = 45.0               # hr39c：H03 对 H01 的让位体向 +z 拉长（= mechanical_audit H03_shell −z 拆卸行程 45）


def _h01_clear_cutter(h01):
    """H01 连续膨胀 H01_CLR_H03，但扣掉四只脚下面凸台顶那一层（Ø7.4 × z 231.0..231.52）—— 脚本来就坐在凸台顶 231.5146 上。
    扣的只到凸台顶面，凸台顶以上的放大毛坯（比如 H01 右前脚 (49.1, 9.35) 旁边放大后的斜坡）照样按 0.3 让。"""
    # hr39c 修（mechanical_audit「H03_shell insertion」）：让位体沿 +z 拉长 H01_UP_SWEEP —— H03 在 H01 正上方（拆卸行程内）不许有料，
    #   否则 H03 −z 拆的时候挂住 H01（放大毛坯的地板夹层压在前立板下沿条 (47..49, −16..−12) 上、右前脚旁放大斜坡压在脚顶上）。
    #   脚下凸台顶那层仍扣掉（凸台在脚**下面**，上拉不影响它）。
    c = minkowski_box(h01, (-H01_CLR_H03, -H01_CLR_H03, -H01_CLR_H03), (H01_CLR_H03, H01_CLR_H03, H01_UP_SWEEP))
    return diff(c, *[cyl(7.4, 0.52, (x, y, 231.26)) for (x, y) in HEAD_FEET])


def _diff_sweep_lowmem(m, part, body, lo, hi, n, grow, chunk=9):
    """hr50：m − lib.sweep_of(part, body, lo, hi, n, grow=grow, mink=True)，同一次 minkowski_box 连续膨胀、同一组 np.linspace 角度，几何相同；
    只是不走 trimesh 的 float32 批量并集（H03 偏航座 81 份一次并集峰值 4.7 GB，超 3 GB 内存规矩）：直接用 manifold 双精度，每 chunk 份并一次、
    立刻求值后累加，最后只减一次再转回网格（中间不经 float32 往返：试过 float32 分批并 / 分批减，导出后各留过 1 条非流形边）。"""
    import trimesh, manifold3d as M
    from assembly_audit import solid
    T = TW(body); ax, og = T[:3, 2], T[:3, 3]
    g = minkowski_box(part, (-grow,) * 3, (grow,) * 3)
    angs = np.linspace(lo, hi, n)
    U = None
    for i in range(0, n, chunk):
        c = M.Manifold.batch_boolean([solid(placed(g, rot(math.radians(float(a)), ax, og))) for a in angs[i:i + chunk]], M.OpType.Add)
        c.num_vert()                                                    # manifold 惰性求值：这里强制算掉，控制峰值内存
        U = c if U is None else U + c
        U.num_vert()
    r = solid(m) - U
    if r.status() != M.Error.NoError: raise ValueError(f"偏航座扫掠减法失败: {r.status()}")
    mm = r.to_mesh64()
    return trimesh.Trimesh(vertices=np.asarray(mm.vert_properties)[:, :3], faces=np.asarray(mm.tri_verts), process=False)


# hr50：H03 的 J01 −5..30° 扫掠刀。步长 0.25°、盒膨胀 0.45：原刀步长 2.5° 时两份之间的间隙掉得最快处约 0.17 mm/°（J01 前上沿沿地板前端斜着滑），
#   0.25° 一步两份之间最多再近 ≈0.04 → 全程 ≥0.4（改后实测：±0.4 盒膨胀的 J01 在 −5..30° 每 0.05° 共 701 个角都碰不到 H03）。角度集含原刀全部角度、膨胀更大 → ⊇ 原刀。
H03_JAW_RELIEF = dict(step=0.25, grow=0.45)
_H03JR = {}
def _h03_jaw_relief():
    """H03 用的 J01 扫掠刀（见 H03_JAW_RELIEF）：manifold 双精度直接做 minkowski 和、141 份每 9 份求值一次再累加（约 60 s、峰值 < 1.3 GB）。"""
    if "m" not in _H03JR:
        import trimesh, manifold3d as M
        from assembly_audit import solid
        from . import jaw as JAW
        k = H03_JAW_RELIEF; g = k["grow"]
        G = solid(JAW.build_jaw()).minkowski_sum(solid(wbox((-g,) * 3, (g,) * 3)))
        lo, hi = JAW.JAW_RELIEF_RANGE
        angs = np.round(np.arange(lo, hi + 1e-9, k["step"]), 4)
        U = None
        for i in range(0, len(angs), 9):
            c = M.Manifold.batch_boolean([G.transform(np.ascontiguousarray(JAW.jaw_R(float(a))[:3, :4])) for a in angs[i:i + 9]], M.OpType.Add); c.num_vert()
            U = c if U is None else U + c; U.num_vert()
        mm = U.to_mesh64()
        _H03JR["m"] = trimesh.Trimesh(vertices=np.asarray(mm.vert_properties)[:, :3], faces=np.asarray(mm.tri_verts), process=False)
    return _H03JR["m"].copy()


def _h01_foot_columns():
    """hr50：H01 脚上的 Ø2.7 孔是空的，_h01_clear_cutter 对它不起作用 → 右前脚 (49.1, 9.35) 孔柱里留了一截放大斜坡的管（r 0.85..1.05、z 231.5..233.5，
    薄膜 3.5 mm² + 尖刺 (49.7, 10.0, 233.5)）。四只脚孔柱（Ø2.7 + 2×0.3）从凸台顶豁免层之上（231.52）往上同样让位（螺丝从这里过，H03 不该有料）。
    单独当刀传给 diff（不并进 _h01_clear_cutter 的大网格，省内存）。"""
    z0 = 231.52
    return [cyl(2.7 + 2 * H01_CLR_H03, H01_UP_SWEEP, (x, y, z0 + H01_UP_SWEEP / 2)) for (x, y) in HEAD_FEET]

# hr43b（K1）：头横滚舵机（Gate / asmcheck 叫 jaw_soft[1] = drv_self("jaw_soft")；**不是**嘴舵机）+y 下口（局部 +y = 世界 −z）PH2.0
#   插头顶/线翘区 A + 侧出线区 B（keepouts.yaml:KO01，CONN_MODE free = CAD 不出刀）被 H03 一道横筋挡：放大原版底壳自带的筋 x −23.64..−21.54
#   （厚 2.1 → 顶部 1.75，顶 z 226.05，y −38..+14），筋前面伸进线翘区 x ≥ −22.1（hr41b L3 / hr43 asmcheck：+y/B 15.4728、+y/A 3.7237 mm³）。
#   让位 = lib.conn_zone(+1, A ∪ B) 同一套几何落到舵机上、外扩 0.3；**只有筋厚方向（世界 −x）外扩 H03_KO01_GROW_BACK**：筋背面 x −23.48（z 225.8）..−23.64（z 222），
#   外扩 0.3 时剩壁 1.078 < 1.2（简报前提），0.15 → 1.228，0.1 → 1.278（hr43b dryrun，placed/head_bottom_shell 射线）。
#   刀实际只削筋前面：x −22.2..−21.54 × y −14.7..−3.9 × z 222.0..筋顶 226.05（上面敞开，不新增悬垂）；筋底 z 221..222、地板、下半环、托台都不碰。
H03_KO01_GROW, H03_KO01_GROW_BACK = 0.3, 0.1
def h03_roll_conn_relief():
    """H03 对头横滚舵机 +y 下口 PH2.0 线翘区 A + 侧出线区 B 的让位刀（世界系；见上面注释）"""
    from .lib import conn_zone
    zone = placed(union(conn_zone(1, "A"), conn_zone(1, "B")), drv_self("jaw_soft"))
    g, gb = H03_KO01_GROW, H03_KO01_GROW_BACK
    return minkowski_box(zone, (-gb, -g, -g), (g, g, g))


def build_head_bottom_shell():
    """H03：原下头壳的S288适配衍生件，切内侧舵机让位（±0.3）+ H02 压板让位（±CLR，09-14）+ A/B 下半座都扩到 Ø22.18（A：09-14 M5）；原版文件不改。
    09-20：横滚舵机 PH2.0 可见孔（旧 CF4）取消 —— 插座实为背插、插头不外凸，壳上不再开孔。
    hr39c：毛坯 = head_scale 放大后的 bottom_head_shell + 原版颈口核心（真实位置，见 H03_CORE_*）；转接板槽（tub）取消 → Radxa 前面地板槽 + USB 插头井；
    嘴：托台 + 惰轮下半环（jaw.h03_adds）、新枢轴孔（jaw.shell_pivot_cuts(slot=False)）、J01 −5..30° 扫掠让位（jaw.relief_sweep）。
    原模型来自Pollen Robotics Microduck，沿用原资产的CC BY-SA-NC许可。
    """
    from . import jaw as JAW
    Rr = drv_self("jaw_soft")
    motor = placed(s288.servo_mesh(), Rr)
    cutter = minkowski_box(motor, (-0.3,)*3, (0.3,)*3)
    # 09-14：H02 压板/垫柱也按 ±CLR 让（H03 最后合上，必须让开已装好的 H02）。
    clamp_clr = minkowski_box(build_head_clamp(), (-CLR,)*3, (CLR,)*3)
    # B 端下半座（09-12 ④）：原版下半座实测 r 10.895（Ø21.79），对 Ø22 外圈是 0.105 径向过盈 —— 扩到 Ø22.18 与 H01 上半座同径（径向 0.09）。
    # 只扩槽段 x 40.1..44.2（原版槽 40.1..44.2），不动 44.2..44.9 那圈 r 9.3 的唇：唇顶住外圈 +x 面，是 N03 +x 方向的轴向定位。
    b0, b1 = YRM_B_BRG[0] - 0.1, YRM_B_BRG[1]
    seatB = cyl(P["seat_d"], b1 - b0 + 0.02, ((b0 + b1) / 2, 0, 235.615), axis="x")
    # A 端下半座同样扩到 Ø seat_d（09-14 M5，见 H03_SEAT_A_X0 注释）：x 5.98..YRM_A2+0.1
    a0, a1 = H03_SEAT_A_X0 - 0.02, YRM_A2 + 0.1
    seatA = cyl(P["seat_d"], a1 - a0, ((a0 + a1) / 2, 0, 235.615), axis="x")
    # F22 接收孔重做（I4，见文件头常量注释）：先填后钻；舵机 minkowski / 插座窗 / 座刀永远压过填料
    fills = [cyl(H03_FILL_D, H03_TOP - z0, (x, y, (z0 + H03_TOP) / 2))
             for (x, y), z0 in zip(HEAD_FEET, (H03_HOLE_BOT - 0.5,) * 3 + (H03_FILL_Z0_4,))]
    fills.append(cyl(H03_COLLAR_D4, H03_TOP - H03_FILL_Z0_4, (HEAD_FEET[3][0], HEAD_FEET[3][1], (H03_FILL_Z0_4 + H03_TOP) / 2)))   # 孔4 Ø6 套环（09-16）
    pilots = [cyl(H03_PILOT_D, H03_PILOT_DEPTH + 1.0, (x, y, H03_TOP + 0.5 - H03_PILOT_DEPTH / 2)) for (x, y) in HEAD_FEET]   # 刀 z 224.5146..232.5146
    top = HS.scaled_orig("top_head_shell")                 # hr39c：合缝对放大后的上壳（原来是原版 top_head_shell）
    seam = []
    for dv in DILATE6(HEAD_SEAM_CLR):                      # 合缝让位（I6）：上壳 6 向各平移 0.1 的副本
        c = top.copy(); c.apply_translation(dv); seam.append(c)
    # hr39c：进风槽回到口袋正下方地板（真实 x 54..64），穿放大后的地板
    vents = [wbox((H03_VENT_X0, y - H03_VENT_W / 2, H03_VENT_Z[0]), (H03_VENT_X1, y + H03_VENT_W / 2, H03_VENT_Z[1])) for y in H03_VENT_YS]
    # hr29：N07 头部轴承盖中心 M2×8 的起子通道（Ø4，沿 +x 过 (y 0, z 235.615)）→ 保留 Ø5 让位（hr39c 放大后口袋下地板顶 ≈234，这把刀削掉地板顶一层）
    n07_drv = cyl(5.0, 20.0, (61.0, 0.0, 235.615), axis="x")
    # hr39c 毛坯：放大 + 真实位置核心
    # was_until_2026_09_28_hr50: base = diff(HS.scaled_orig("bottom_head_shell"), wbox(*H03_CORE_CUT))
    base = diff(union(HS.scaled_orig("bottom_head_shell"), *_h03_old_foot_fills()), *_h03_blank_cuts())   # hr50：先填三个旧脚孔，再挖（见常量注释）
    core = inter(orig("jaw_soft", "bottom_head_shell"), wbox(*H03_CORE_BOX))
    # hr39c 转接板槽（H03-F14）+ USB 插头井（H03-F15）
    ax0, ax1 = H03_ADP_X; ay0, ay1 = H03_ADP_Y; az0 = H03_ADP_Z[0]; sl = H03_ADP_SLOT
    # was_until_2026_09_25_hr43c: slot_walls = [wbox((ax0 - sl["clr"] - sl["wall"], ay0, 232.0), (ax0 - sl["clr"], ay1, az0 + sl["h"])),
    # was_until_2026_09_25_hr43c:               wbox((ax1 + sl["clr"], ay0 + 4.0, 232.0), (ax1 + sl["clr"] + sl["wall"], ay1, az0 + sl["h"]))]    # +x 壁 −y 端少 4：让 UBEC 座（x ≥ 83.07, y ≤ −23.6）
    # was_until_2026_09_25_hr43c: ap_ = H03_ADP_POSTS
    # was_until_2026_09_25_hr43c: slot_walls += [wbox((ap_["x"][0], ya_, 232.0), (ap_["x"][1], yb_, ap_["z_top"])) for (ya_, yb_) in ap_["ys"]]              # hr41 M1 +x 立柱
    slot_walls = [wbox((ax0 - sl["clr"] - sl["wall"], ay0, 232.0), (ax0 - sl["clr"], ay1, az0 + sl["h"]))]                     # −x 壁保留
    bd = H03_ADP_BED                                                                                                              # hr43c：+x 壁 + 立柱 → 平床 F22
    slot_walls += [wbox((bd["x"][0], bd["y"][0], bd["z0"]), (bd["x"][1], bd["y"][1], bd["z_top"]))]
    bed_pit = wbox((bd["x"][0] - 0.2, bd["pit_y"][0], bd["pit_z"][0]), (bd["x"][0] + bd["pit_depth"], bd["pit_y"][1], bd["pit_z"][1]))   # F23 胶坑
    slot_ribs = [wbox((ax0 - sl["clr"], y - sl["rib_w"] / 2, 232.0), (ax1 + sl["clr"], y + sl["rib_w"] / 2, az0)) for y in sl["rib_y"]]
    slot_stops = [wbox((ax0 - sl["clr"], yy0, 232.0), (ax1 - 2.0, yy1, az0 + 1.6)) for (yy0, yy1) in ((ay0 - 0.3 - 1.2, ay0 - 0.3), (ay1 + 0.3, ay1 + 0.3 + 1.2))]
    (wc0, wc1), ww, wf = H03_USB_WELL["cav"], H03_USB_WELL["wall"], H03_USB_WELL["floor"]
    well_box = wbox((wc0[0] - ww, wc0[1] - ww, wc0[2] - wf), (wc1[0] + ww, wc1[1] + ww, 234.2))
    well_cav = wbox(wc0, wc1)
    usb_cuts = [wbox(*b) for b in H03_USB_WALL_CUTS]                                                   # hr39c-⑧：槽 −x 壁给 USB FPC 让路（见 H03_USB_WALL_CUTS）
    # was_until_2026_09_25_hr43d: m = diff(union(base, core, *fills, well_box, *slot_walls, *slot_ribs, *slot_stops), cutter, clamp_clr, seatA, seatB, *pilots, *seam, *vents, n07_drv, well_cav, *usb_cuts, bed_pit)
    # hr43d（B′，agent_brief_hr43d 第 3 条）：转接板搬到 H08 托板上 → F14 槽（−x 壁 / 两条托筋 / 两端挡块）、F22 平床、F23 胶坑、F15 USB 插头井（well_box / well_cav）、
    #   −x 壁 USB 切口（H03_USB_WALL_CUTS）全删 —— 上面这些量仍算出来留痕，不再进 union / diff；地板回到放大原版（base）同厚。
    m = diff(union(base, core, *fills), cutter, clamp_clr, seatA, seatB, *pilots, *seam, *vents, n07_drv)
    m = HBR.head_seats(m, upper=False)
    # hr50：A 座 −x 端还剩核心里原版 A 下半座块的一层（x 5.30..5.58，0.28 厚，薄膜 79 + 43 mm²）。HBR 注释：A 是非定位端「full-length through seat, NO thin thrust lips」，
    #   这层恰是一道薄挡边、还压在轴承浮动区 x 5.6 外；收刀留 0.8 会吃掉 −x 浮动量（碰浮动包络）→ 座孔 −x 端削穿到 5.0（x 3.9..5.3 本来就空）。
    m = diff(m, HBR._roll_cyl(HBR.HEAD_A["seat_d"], H03_SEAT_A_LIP_X0, HBR.HEAD_A["seat_x0"] + 0.02))
    # New yaw cartridge is fixed to N03; H03 moves with head_roll.
    # was_until_2026_09_28_hr50: m = diff(m, sweep_of(HBR.yaw_motion_envelope(), "jaw_soft", -25, 25, 81, grow=0.45, mink=True))   # hr11：mink 各向 ≥0.45
    m = _diff_sweep_lowmem(m, HBR.yaw_motion_envelope(), "jaw_soft", -25, 25, 81, grow=0.45)          # hr50：同一把刀，分批减（省内存，见函数注释）
    k = H03_B_MOUTH_CUT                                                                               # hr50 ①：B 座口座外刀留下的皮平切（见常量）
    m = diff(m, inter(cyl(2 * k["r"], k["x"][1] - k["x"][0], ((k["x"][0] + k["x"][1]) / 2, 0.0, 235.615), axis="x"),
                      wbox((k["x"][0] - 0.01, -k["r"] - 0.5, 200.0), (k["x"][1], k["r"] + 0.5, k["z1"]))))
    # was_until_2026_09_28_hr50_r2: （新增行；改前偏航座扫掠刀 +x 端面不另切）
    m = diff(m, wbox(*H03_YAW_FACE_CUT))                                                              # hr50_r2 2026-09-28：扫掠刀 +x 端拼花面切成一个平面（见常量）
    m = n02_swing_cut(m)                                                                              # hr32：H03-F11，N02 绕头偏航×头横滚目标区间扫掠让位
    m = diff(m, *[wbox(*b) for b in H03_N02_WEDGE_CUTS])                                              # hr50：N02 摆动刀留下的楔削穿（见常量）
    m = mic_pad(m)                                                                                    # hr37：麦克风台 + 声孔（H03-F12；hr39c 平移 T_MIC）
    m = ubec_pad(m)                                                                                   # hr38：UBEC-3A 座（H03-F13；hr39c 平移 T_UBEC）
    ab = H03_AMPB                                                                                     # hr43d：H09 功放支架接收凸台（H03-F24）
    m = union(m, *[cyl(ab["d"], ab["z_top"] - ab["z0"], (x, y, (ab["z0"] + ab["z_top"]) / 2), sections=48) for (x, y) in ab["xy"]])
    # hr39c 嘴：托台 + 惰轮下半环（先加再切：嘴舵机/插座区/座孔/J01 扫掠）
    # hr43（主设计 2026-09-25 05:10 实测，hr41 漏掉的 BLOCKER）：**6700 惰轮座下半环被切没了** —— 原刀序是
    #   union(h03_adds) → inter(envelope) → diff(h03_cuts, shell_pivot_cuts(slot=False), relief_sweep)，
    #   shell_pivot_cuts 的 −y 刀 Ø19.2×14（y −51.55..−37.55）比座环 OD 18.6 大、y 段整个盖住座环 → 刚 union 进去的下半环整段被切掉
    #   （placed/head_bottom_shell 在 y −47.05 平面绕嘴轴 r 6.4..9.3 contains 全 0；L2 H03-F17 present 0/24 当时被误判为登记错误）。
    #   这把刀本意是「壳开孔让座环穿壁」，但壳壁在 |y| 52.5..54.7，刀根本够不到壁。
    #   改：先开壳孔 → 再加托台 + 下半环 → 再切 h03_cuts（Ø15.1 座孔 / 轴颈让位 / 舵机让位）+ J01 扫掠。jaw.py 的刀定义不动（H05 用 slot=True，H05 没有环）。
    #   旧三行留痕（hr39c..hr41）：m = union(m, *JAW.h03_adds()); m = inter(m, _h03_envelope());
    #                              m = diff(m, *JAW.h03_cuts(), *JAW.shell_pivot_cuts(slot=False), JAW.relief_sweep())
    m = diff(m, *JAW.shell_pivot_cuts(slot=False))                                                    # hr43：先开壳孔
    # was_until_2026_09_28_hr50: m = union(m, *JAW.h03_adds())
    m = union(m, *JAW.h03_adds(), wbox(*H03_WEB_CRACK_FILL))                                          # hr50 ②：腹板底下的毛坯裂缝填实
    m = inter(m, _h03_envelope())                                                                     # 托台立墙往下长的那截由外壳截掉，不冒出壳外
    # was_until_2026_09_28_hr50: m = diff(m, *JAW.h03_cuts(), JAW.relief_sweep())                                                  # hr43：座孔 / 轴颈让位 / 舵机让位 / J01 扫掠照旧切到环
    # hr50（主设计 09-28：嘴 −5..30° 全程和 H03 间隙 ≥0.4）：原刀（2.5° 一步、盒膨胀 0.4）两份之间 J01 前上沿贴着 H03 地板前端滑过，−3.0° 处只剩 0.104，
    #   侧面 x 44..53、|y| 39 处 0.32..0.35（hr50_work/head_films/jaw_clearance.py）→ H03 改用 _h03_jaw_relief（0.25° 一步、盒膨胀 0.45，⊇ 原刀）。
    m = diff(m, *JAW.h03_cuts(), _h03_jaw_relief())                                                   # hr43：座孔 / 轴颈让位 / 舵机让位 / J01 扫掠照旧切到环
    # hr50：J01 −5..30° 扫掠刀（包络 + 0.4）把放大毛坯嘴口一圈的下垂唇削到只剩一层：前缘 x 96.85..97.1（0.2 厚、z 229.6..231.9，薄膜 498 mm²）、
    #   两侧 x 57..85（y ±51..52.8，0.72 → 0 的楔，薄膜 77 / 66 mm²、尖刺 (84.2, −52.7, 229.7)）。补到 0.8 要吃进扫掠包络 0.2..0.8 → 会碰 J01 → 整片削穿：
    #   地板底面（前缘 231.89，两侧 231.99..232.3）以下这圈唇切平（H03_JAW_LIP_CUTS）。
    # hr50（09-28 决定 1）：+ 后脑两侧 J01 后下角开口的平槽（H03_JAW_REAR_SLOT）
    m = diff(m, *[wbox(*b) for b in H03_JAW_LIP_CUTS], *_h03_side_lip_cuts(), *[wbox(*b) for b in H03_SIDE_RIM_CUTS], *_h03_jaw_rear_slots())
    h01 = build_head_bracket()
    # was_until_2026_09_28_hr50: m = diff(m, _h01_clear_cutter(h01), h01)                                                          # hr39c：对 H01 通用让位 0.3（放大毛坯的筋可能碰 H01，脚坐的凸台顶除外；凸台顶与脚底贴合处再精确减一次 H01）
    m = diff(m, _h01_clear_cutter(h01), h01, *_h01_foot_columns(),                                   # hr39c 对 H01 通用让位 0.3 + hr50 四只脚孔柱
             *[wbox(*b) for b in H03_H01_NOTCH_RIB_CUTS])                                             # hr50：H01 前立板底下那截放大肋
    # hr41：H05 保持螺丝 2 个 Ø2.4 过孔（head_top.H05_KEEP_XY）。OTG 地板口（H03_OTG_OPEN）**作废不开**：用户 09-24 17:15 定 Radxa 搬后脑（hr42），
    #   装好后 USB/OTG/TF 不要求可达（拆顶壳调试）。
    from .head_top import H05_KEEP_XY
    m = diff(m, *[cyl(H03_KEEP_HOLE_D, 6.0, (x, y, 233.0)) for (x, y) in H05_KEEP_XY],
             *[cyl(H03_KEEP_SPOT[0], H03_KEEP_SPOT[1] - 229.0, (x, y, (229.0 + H03_KEEP_SPOT[1]) / 2)) for (x, y) in H05_KEEP_XY])   # 地板底面锪平（盘头坐面）
    # hr43b（K1）：头横滚 +y 下口 KO01 A/B 让位（h03_roll_conn_relief，筋前面削 ≤0.66，剩壁 ≥1.2）。最后一刀，后面不再 union。
    # was_until_2026_09_25_hr43b: （新增行；改前 H03 对这个口不出刀，CONN_MODE free）
    m = diff(m, h03_roll_conn_relief())
    m = diff(m, *[cyl(ab["pilot_d"], ab["pilot_depth"] + 1.0, (x, y, ab["z_top"] + 0.5 - ab["pilot_depth"] / 2), sections=32) for (x, y) in ab["xy"]])   # hr43d：F24 底孔（最后一刀）
    return clean_print_topology(keep_main(m, "head_bottom_shell"))

# hr41 **作废（用户 09-24 17:15：Radxa 搬后脑 hr42，装好后 OTG/TF 不要求可达）——常量留痕，build 不再用**。
# 原方案（复审 #1 M4 / hr39d 阶段一 (ii)）：OTG 口 = Radxa USB-C1（y −26.67..−17.73，口面 z 234.8 朝 −z）正下方的 H03 地板开口，
#   直头插头包络 12.4 × 6.5（assumed）外扩 0.5 → x 60.55..68.05 × y −28.9..−15.5，刀 z 224..234.81（实际只去地板那层 z 232.13..233.96，169.75 mm³，hr39d extras.json）。
#   嘴装好时被 J01 下巴盖住，J01 −5..30° 扫掠对开口 0。维护：拆 J01 6 颗（J03 按住）→ 从这里插直头或朝 +x / −y 的 90° 弯头；朝 −x 的弯头撞 H01/H03/xt30 线（不可）。
H03_OTG_OPEN = ((60.55, -28.9, 224.0), (68.05, -15.5, 234.81))
H03_KEEP_HOLE_D = 2.4
# hr41：盘头坐面锪平 Ø4.6 到 z 232.30 —— 地板底面沿 x 有 ≈0.01/mm 的斜（x 61 → 232.20、67 → 232.14），头足印 Ø4 里起伏 ±0.03，
#   按 L4 tool_origin_on_seat 的平面环带判据（面内外 0.001）不成立（hr41_work/seats41.json：39/72）→ 锪平 0.13，地板剩 1.65，M2×8 咬入 6.05
H03_KEEP_SPOT = (4.6, 232.30)
# hr43d（E′）：H09 功放支架的两个接收凸台（H03-F24）—— 后脑 +y 地板上（地板顶 ≈221.2..221.4、厚 1.7；hr43_work/hr43d/d03_floor_probe.py），Ø5 × 顶 224.5（从 220.8 起，埋进地板），
#   Ø1.7 底孔深 4.3（孔底 220.2，离地板外表面 ≈219.65 留 ≥0.5，不开外观孔）；F38 M2×6 自攻从上穿 H09 脚板 2.0 → 咬 4.0（= pla_self_tap 4.0，尖 220.5）。
#   位置避开 H03 件号刻字（stamps.STAMPS['H03']：地板顶 (−12.24, 27.87)，字 4.5 高沿 y、≈9.4 宽沿 x → x −16.9..−7.5 × y 25.6..30.1）：凸台 y ≥ 30.5；两凸台沿 x 排（Ø5 相距 4.0，并成一条）。
H03_AMPB_was_until_2026_09_25_hr43e = dict(xy=((-22.0, 33.0), (-18.0, 33.0)), d=5.0, z0=220.8, z_top=224.5, pilot_d=1.7, pilot_depth=4.3)
# hr43e（agent_brief_hr43e 第 2 条，L6 H09 目标区间内 33 条）：凸台 / F38 孔心 y 33.0 → 34.7。头俯仰 × 颈俯仰只绕 y 转，躯干 shell_L 在头系里的 y 不变（max 31.76）→
#   H09 整件（含脚板、Ø4 头足印 + 0.2 环带）挪到 y ≥ 32.4 就碰不到它（hr43_work/hr43e/e04_forbid_back.npz：y ≥ 32.5、x ≥ −31 区间内禁入 0）。
#   孔心 34.7：足印环带（Ø4.4）y 32.5..36.9 落在新脚板 32.4..37.2 内；凸台 Ø5 → y 32.2..37.2；孔底 220.2 离地板外皮 ≈219.73（y 34.7，e08_floor_probe）剩 ≈0.47（原 y 33 剩 0.55）。
# hr43e #2（协调员 20:10 ①，F38 pilot_bottom_margin 零余量）：凸台顶 224.5 → 225.0（H09 脚板一起抬 0.5）、底孔 4.3 → 4.8（Ø1.7 不变，孔底仍 220.2、外皮 ≈0.47 不变）→
#   M2×6 穿脚板 2.0 咬 4.0，尖 221.0，离孔底 0.8（L5 判据 ≤ 4.8 − 0.3 = 4.5）。hr43e build #1 值：z_top=224.5, pilot_depth=4.3
H03_AMPB = dict(xy=((-22.0, 34.7), (-18.0, 34.7)), d=5.0, z0=220.8, z_top=225.0, pilot_d=1.7, pilot_depth=4.8)


_ENV = {}
def _h03_envelope():
    """放大后 H03∪H05∪脸板 顶点凸包再往外 0.02（只用来截"往壳外长"的内部加料，壳本身不受影响）"""
    if "e" not in _ENV:
        import trimesh
        V = np.vstack([HS.scaled_orig(n).vertices for n in ("bottom_head_shell", "top_head_shell", "face_part")])
        _ENV["e"] = trimesh.convex.convex_hull(V)
    return _ENV["e"].copy()


# ━━ hr43（2026-09-25）H07 功放托板（新打印件，body jaw_soft）━━
#   用户 2026-09-24 18:20 定：功放搬脸后口袋、托板拧 H01 旧 Radxa 四柱（hr43_设计稿_Radxa后脑A.md 补记 18:20 / 18:50）。功放 = electronics.amp_boxes（直排针版，BX 逐字）。
#   形状 = 竖板 P（贴 H01 柱端面 x 66，2 厚）+ 两条纵梁（|y| 9.4..11.4，沿 x）+ 后端横梁（x 46.5..51.5）+ 两根 Ø4 立柱托功放板底（柱心 = 功放两孔 (49, ±6.3)）。
#   竖板：上横条连两根上立柱 (y ±29, z 262.5)，+y 侧往下一条腿接 +y 下立柱 (29, 239.5)（这根是 hr39c 的短立柱 x 58.1..66，端面/底孔照旧）→ 3 颗 M2×8 自攻（F35）三角形布置。
#   高度预算（z）：H01 前立板顶缺口底 257.6（head.py H01 前缘整段缺口 z ≥ 257.6，|y| ≤ 26）+ 1.0 → 托面底 258.6；功放板底焊点（BX term_solder_bot）261.2 − 0.5 → 托面顶 260.7（厚 2.1）；
#   Ø4 立柱 260.7..262.2（贴功放板底，接触），Ø1.7 通孔（F36 M2×5 从功放板顶拧：板 1.6 → 咬入 3.4，尖在 258.8 不出托面底）。
#   让位：功放杜邦壳/塑座区 x 61.8..64.5 × |y| ≤ 8.9 × z 245..262 留空（纵梁在 |y| ≥ 9.4）；功放板 +x 边 65.55 → 竖板背面 66.0 隙 0.45。
#   装序：台面先把功放拧上托板（F36）、杜邦壳插好 → 整件从 +x 沿 −x 推到 H01 柱端（H03/H04/H05 都还不在，起子从 +x 直进）拧 F35（assembly 步 18a）。
#   打印：竖板前面（世界 +x）朝下平放，纵梁竖着长上去，后端横梁是 18.8 跨的桥（printability H07）。
H07 = dict(plate_x=(66.0, 68.0), bar=((-31.5, 258.6), (31.5, 265.0)), leg=((26.5, 237.0), (31.5, 265.0)),
           screws=((29.0, 262.5), (-29.0, 262.5), (29.0, 239.5)), hole_d=2.4,
           shelf_z=(258.6, 260.7), rail_y=(9.4, 11.4), rail_x=(51.0, 66.05), cross_x=(46.5, 51.5), post_d=4.0, pilot_d=1.7,
           src=dict(shelf_z="H01 缺口底 257.6 + 1.0 / 功放 term_solder_bot 261.2 − 0.5", rail_y="功放塑座/杜邦壳 |y| 8.9 + 0.5",
                    screws="head.py build_head_bracket posts（y ±29、z 251 ± 11.5、端面 x 66、Ø1.7 底孔深 10）", post_xy="electronics.amp_S_holes_world()（孔距 12.6 用户定、离顶边 2.55 assumed）"))
# hr43c（2026-09-25，复审 #2 B1）：F22 右前脚螺丝 (49.1, 9.35) 的起子通道 KO09（Ø4.6，z 233.2..273.2）原被 H07 +y 立柱 / 横梁 / 功放板挡住（声明装序 P′ 下 F22#3 拧不上）。
#   改：托面（两纵梁 + 横梁）、两根 Ø4 立柱、功放整体沿 −y 挪 shift_y（= electronics.AMP_S_C − 旧值 = −2.0；复审要 ≥1.8：立柱心 6.3 → 4.3 ≤ 4.7、功放板 +y 边 8.85 → 6.85 ≤ 7.05）；
#   竖板 P（拧 H01 旧 Radxa 立柱，F35 孔位冻结）不动；在 (49.1, 9.35) 开 Ø5.3 竖直过孔（KO09 Ø4.6 + 单边 0.35，复审「≥ Ø5.3」），穿托面全高；
#   过孔把横梁 +y 端 / +y 纵梁前端截掉一角 → 加一块连接板 web（x 51..53.5 × y 5..纵梁外缘，同托面高，扣过孔）把 +y 纵梁接回横梁（过孔边到 web 最近 ≥ 0）。
H07_HR43C = dict(tool_hole=dict(xy=(49.1, 9.35), d=5.3, z=(250.0, 270.0), src="fasteners F22 孔 4 (49.1,9.35) / keepouts KO09 Ø4.6；复审 #2 B1「≥ Ø5.3」"),
                 web=dict(x=(51.0, 53.5), y0=5.0))


def build_amp_tray():
    """H07（hr43 新件）：功放托板（见 H07 常量注释）。"""
    from .electronics import amp_S_holes_world
    from . import electronics as E
    k = H07; (px0, px1) = k["plate_x"]; (z0, z1) = k["shelf_z"]; (ry0, ry1) = k["rail_y"]
    (by0, bz0), (by1, bz1) = k["bar"]; (ly0, lz0), (ly1, lz1) = k["leg"]
    sy = E.AMP_S_C[1] - E.AMP_S_C_was_until_2026_09_25_hr43c[1]          # hr43c B1：托面 / 立柱随功放 −y 2.0（竖板不动）
    plate = union(wbox((px0, by0, bz0), (px1, by1, bz1)), wbox((px0, ly0, lz0), (px1, ly1, lz1)))
    # was_until_2026_09_25_hr43c: rails = [wbox((k["rail_x"][0], s * ry0 if s > 0 else -ry1, z0), (k["rail_x"][1], s * ry1 if s > 0 else -ry0, z1)) for s in (1, -1)]
    # was_until_2026_09_25_hr43c: cross = wbox((k["cross_x"][0], -ry1, z0), (k["cross_x"][1], ry1, z1))
    rails = [wbox((k["rail_x"][0], (s * ry0 if s > 0 else -ry1) + sy, z0), (k["rail_x"][1], (s * ry1 if s > 0 else -ry0) + sy, z1)) for s in (1, -1)]
    cross = wbox((k["cross_x"][0], -ry1 + sy, z0), (k["cross_x"][1], ry1 + sy, z1))
    w = H07_HR43C["web"]; web = wbox((w["x"][0], w["y0"], z0), (w["x"][1], ry1 + sy, z1))   # hr43c：+y 纵梁 ↔ 横梁连接板（过孔绕开）
    holes_amp = [(x, y) for (x, y, _) in amp_S_holes_world()]                  # 已含功放 −y 2.0
    posts = [cyl(k["post_d"], 262.2 - z1 + 0.05, (x, y, (z1 - 0.05 + 262.2) / 2), sections=48) for (x, y) in holes_amp]
    m = union(plate, *rails, cross, web, *posts)
    cuts = [cyl(k["hole_d"], (px1 - px0) + 2.0, ((px0 + px1) / 2, y, z), axis="x", sections=32) for (y, z) in k["screws"]]
    cuts += [cyl(k["pilot_d"], 262.2 - z0 + 1.0, (x, y, (z0 + 262.2) / 2), sections=32) for (x, y) in holes_amp]
    th = H07_HR43C["tool_hole"]; (tx, ty), (tz0, tz1) = th["xy"], th["z"]
    cuts += [cyl(th["d"], tz1 - tz0, (tx, ty, (tz0 + tz1) / 2), sections=64)]   # hr43c B1：F22 右前脚起子过孔 Ø5.3
    return keep_main(diff(m, *cuts), "amp_tray")


# ━━ hr43d（2026-09-25）H08 转接板托板（新打印件，body jaw_soft；取代 H07 功放托板 —— 功放挪后脑 H09，H07 整件退役）━━
#   依据：hr43_work/转接板安放研究.md §5.3/§5.4（B′）+ agent_brief_hr43d.md 第 1 条；板位 electronics.ADP_FLAT（研究抬 2.0 版脚印整体 +x 5.8 让 F22#3 起子通道 KO09）。
#   竖板照 H07（x 66..68 贴 H01 三根旧立柱端面，F35 3×M2×8 原孔 (±29, 262.5) / (29, 239.5) 复用；+y 下腿照旧）；上横条在板脚印内（|y| ≤ 20.5）截到托面顶，外面照 H07 到 265。
#   托面 = 板脚印 x 51.8..81.8 × y −20..20 × **z 262.0..264.0**（2.0 厚）：研究「托面 261.5..263.5 / 板底 264.5」没扣胶坑 → 板底 264.5 = 托面顶 + 泡棉胶 1.0 − 坑深 0.5 → 托面顶 264.0
#   （托面底比研究高 0.5，下方间隙只增不减）。胶坑 36×26 × 0.5（x 53.8..79.8 × y −18..18）放 1 mm 泡棉胶（满贴 26×36 = 78 % 板面）。
#   两条扎带（≤3 宽、≤1.2 厚）沿 x 绕板 + 托面，y −9.0 / −16.5：板上方 y −6.15..18.15 是 CSI 排线（HB06）走的带，扎带只能放 −y 半边（assumed 位置，口径不影响）；
#     −x 侧托面外伸小耳（x 48.8..51.8）开 1.6 × 3.4 穿孔；+x 侧（外 2.7 就是 H06 压框，外伸耳放不下）托面边开 1.0 深定位槽、扎带贴板 +x 边面下去；竖板在托面下开 3.4 × 1.6 扎带槽。
#   下纵梁两条（y 1..3 / 12.5..14.5，z 260..262，x 51.8..81.8）：避开扎带带；板 −x 边 51.8 离 KO09 轴 (49.1, 9.35) 2.7（Ø4.6 外 0.4）。
#   装：台面先把转接板用 1 mm 泡棉胶贴进胶坑、穿两条扎带捆紧 → 整件从 +x 沿 −x 推到 H01 柱端（H03/H04/H05 都不在，同 H07 的 18a）拧 F35。
#   打印：托面顶（世界 +z）朝下平放 —— 竖板 / 下腿 / 纵梁都竖着长上去；胶坑在贴床面 = 坑底是 26 mm 桥（PETG，胶面不求光，L1 切片核）。
# hr44 第二轮（头部复审 M1，协调员 03:22）：竖板两只耳（|y| 20.5..31.5，F35 上两孔所在）顶 z 265.0 → 264.0 与托面顶齐平：打印朝向 down_world [0,0,1] 下
#   原来只有两耳 44 mm² 贴床、胶面整片悬空 1.0 靠支撑；削平后首层 = 托面顶一圈 + 两耳 ≈325 mm²。F35 上两孔 Ø2.4（顶 263.7）上方料 1.3 → 0.3，这两颗加 GB848 M2 小平垫。
#   was_until_2026_09_26_hr44b: bar=((-31.5, 258.6), (31.5, 265.0)), leg=((26.5, 237.0), (31.5, 265.0))
ADP_TRAY = dict(plate_x=(66.0, 68.0), bar=((-31.5, 258.6), (31.5, 264.0)), leg=((26.5, 237.0), (31.5, 264.0)),
                screws=((29.0, 262.5), (-29.0, 262.5), (29.0, 239.5)), hole_d=2.4, bar_clip_y=20.5,
                shelf_x=(51.8, 81.8), shelf_y=(-20.0, 20.0), shelf_z=(262.0, 264.0),
                pit=dict(x=(53.8, 79.8), y=(-18.0, 18.0), depth=0.5),
                tie_ys=(-9.0, -16.5), tie_w=3.4, tie_t=1.6, tab_x=(48.8, 51.8), hole_x=(49.6, 51.2), notch_depth=1.0,
                rails=((1.0, 3.0), (12.5, 14.5)), rail_z0=260.0,
                # hr43e（agent_brief_hr43e 第 2 条，L6 H08 目标区间内 3 条：adapter_tray×hip 16.2 / ×hip_R 2.6 @ 颈俯仰 −90、头俯仰 +90，×hip 0.13 @ −85 / 90）：
                #   区间内前折到底时两髋（L02）顶到托面前下角（hr43_work/hr43e/e10_hip_map.npz：x ≥ 77 处髋面最高 261.8，x 79..82 × |y| 17..19 最高 262.3；x ≤ 75 全 < 259.5）。
                #   ① y 12.5..14.5 那条纵梁 +x 端 81.8 → 75.5（x ≥ 76 髋面 260.2..261.8 > 梁底 260）；② 托面底在 x ≥ 76.0、|y| ≥ 12.5 挖到 262.8（0.8 深，余 1.2 / 胶坑下 0.7；x 76.0 处髋面 ≤ 260.9），
                #   对髋面 262.3 留 0.5；|y| < 12.5 髋面 ≤ 260.8（x ≤ 82），托面底 262.0 不动。was_until_2026_09_25_hr43e：两条纵梁都 x 51.8..81.8、托面底全平 262.0
                rail_x1=(81.8, 75.5), front_relief=dict(x0=76.0, y_abs0=12.5, z1=262.8),
                src=dict(plate="= H07（head.H07 plate_x/bar/leg/screws/hole_d）", shelf="electronics.ADP_FLAT 脚印；板底 264.5 − 胶 1.0 + 坑 0.5 = 264.0",
                         pit="agent_brief_hr43d 第 1 条（36×26 × 0.5）", ties="研究 §5.3：两条沿 x 绕；y 位置按 CSI 让位定（assumed）"))


def build_adapter_tray():
    """H08（hr43d 新件）：转接板托板（见 ADP_TRAY 注释）。"""
    k = ADP_TRAY; (px0, px1) = k["plate_x"]; (by0, bz0), (by1, bz1) = k["bar"]; (ly0, lz0), (ly1, lz1) = k["leg"]
    (sx0, sx1), (sy0, sy1), (sz0, sz1) = k["shelf_x"], k["shelf_y"], k["shelf_z"]; cy = k["bar_clip_y"]; tw, tt = k["tie_w"], k["tie_t"]
    bars = [wbox((px0, by0, bz0), (px1, -cy, bz1)), wbox((px0, -cy, bz0), (px1, cy, sz1)), wbox((px0, cy, bz0), (px1, by1, bz1))]   # 上横条：板脚印内截到托面顶
    leg = wbox((px0, ly0, lz0), (px1, ly1, lz1))                                                                                   # +y 下腿（照 H07）
    shelf = wbox((sx0, sy0, sz0), (sx1, sy1, sz1))
    tabs = [wbox((k["tab_x"][0], ty - tw / 2 - 1.0, sz0), (sx0 + 0.01, ty + tw / 2 + 1.0, sz1)) for ty in k["tie_ys"]]           # −x 扎带耳
    # was_until_2026_09_25_hr43e: rails = [wbox((sx0, ya, k["rail_z0"]), (sx1, yb, sz0 + 0.01)) for (ya, yb) in k["rails"]]
    rails = [wbox((sx0, ya, k["rail_z0"]), (rx1, yb, sz0 + 0.01)) for (ya, yb), rx1 in zip(k["rails"], k.get("rail_x1") or (sx1,) * len(k["rails"]))]   # 下纵梁（hr43e：+y 那条缩到 75.5）
    m = union(*bars, leg, shelf, *tabs, *rails)
    cuts = [cyl(k["hole_d"], (px1 - px0) + 2.0, ((px0 + px1) / 2, y, z), axis="x", sections=32) for (y, z) in k["screws"]]      # F35 过孔 Ø2.4
    p = k["pit"]; cuts.append(wbox((p["x"][0], p["y"][0], sz1 - p["depth"]), (p["x"][1], p["y"][1], sz1 + 1.0)))               # 胶坑 0.5
    for ty in k["tie_ys"]:
        cuts.append(wbox((k["hole_x"][0], ty - tw / 2, sz0 - 1.0), (k["hole_x"][1], ty + tw / 2, sz1 + 1.0)))                     # −x 耳穿孔 1.6 × 3.4
        cuts.append(wbox((sx1 - k["notch_depth"], ty - tw / 2, sz0 - 1.0), (sx1 + 1.0, ty + tw / 2, sz1 + 1.0)))                 # +x 边定位槽
        cuts.append(wbox((px0 - 0.1, ty - tw / 2, sz0 - tt), (px1 + 0.1, ty + tw / 2, sz0 + 0.01)))                               # 竖板扎带槽（托面下）
    fr = k.get("front_relief")                                                                                                   # hr43e：托面前下角让髋（区间内前折到底）
    if fr:
        cuts += [wbox((fr["x0"], s * fr["y_abs0"] if s > 0 else sy0 - 1.0, sz0 - 1.0), (sx1 + 1.0, sy1 + 1.0 if s > 0 else -fr["y_abs0"], fr["z1"])) for s in (1, -1)]
    return keep_main(diff(m, *cuts), "adapter_tray")


# ━━ hr43d（2026-09-25）H09 功放支架（新打印件，body jaw_soft）：功放 E′ 位（electronics.AMP_S_C/R：后脑 +y、Radxa +y 端正下方，板竖放、元件面朝 +x）——
#   竖板贴功放板背后（x −15.6..−13.6 × y 29.5..38.0 × z 224.5..245.0），两根 Ø4 立柱沿 x 顶到功放 PCB 背面（x −11.6，接触），Ø1.7 通孔：F36 2×M2×5 从元件面 +x 拧（板 1.6 → 咬 3.4，同 hr43 F36 口径）；
#   竖板下接脚板（x −24.0..−13.6 × y 30.0..36.0 × z 224.5..226.5，躲 H03 地板刻字），2 个 Ø2.4 过孔 (−22, 33) / (−18, 33) → F38 2×M2×6 自攻从上拧进 H03 后部地板凸台（H03-F24，head.H03_AMPB）。
#   让位（功放 BX 包络）：立柱 z 227.5..231.5 / 240.1..244.1 离端子焊点盒（x −12.6..−11.6 × z 232..239.6）0.5；竖板离焊点盒 1.0；脚板离杜邦壳（y 20.3..23）≥5、离 H03 +y 壁 ≥1.4
#   （hr43d/d03_floor_probe）；竖板 +y 边 38.0 离 H05 内面 ≈4。
#   装：H03 台面工位 —— H09 放上两凸台拧 F38（起子从上 +z）→ 功放立柱位扣上拧 F36（起子从 +x）→ 端子接喇叭线；整组随 H03 18c 从下 +z 合上（功放 y ≥ 19.25 离 H01/H02 ≥3.45，竖直合上不碰）。
#   打印：脚板底（世界 −z）朝下 —— 竖板竖长上去，两根 Ø4 立柱是 2 mm 短横伸（小悬垂）。
AMP_BRACKET_was_until_2026_09_25_hr43e = dict(plate_x=(-15.6, -13.6), plate_y=(29.5, 38.0), plate_z=(224.5, 245.0), foot_x=(-24.0, -13.6), foot_y=(30.0, 36.0), foot_z=(224.5, 226.5),
                                                                                     post_d=4.0, pilot_d=1.7, screw_hole_d=2.4,
                                                                                     src=dict(pos="electronics.AMP_S_C/R（研究 b14 候选 ①）", post="功放孔 electronics.amp_S_holes_world()（孔距 12.6 用户定、离顶边 2.55 / Ø2.2 assumed）",
                                                                                              pcb_back="AMP_S_C x − 0.8（BX pcb 局部 z −0.8 → 世界 x）", screws="H03_AMPB（F38）"))
# hr43e：竖板 y 29.5 → 32.4 起、脚板 y 30.0..36.0 → 32.4..37.2 + x −24.0 → −24.8（F38 头足印环带 −x 端原来悬空 1/48 向）；脚板 −x 端 1.5 mm（x −24.8..−23.3）
#   只到 y 36.7：那里 H03 +y 壁 / 地板圆角最近（壁 y ≈37.35 @ z 224.5，e08_floor_probe），留 ≥0.65；x ≥ −23.3 段到 37.2（壁 ≥38.26，留 ≥1.06）。
#   立柱（y 35.8 ± 2）/ 竖板 x / z 不变；竖板 −x 面刻字 H09 跟着挪到竖板 y 中心 35.2（stamps.STAMPS["H09"]）。
# hr43e #2（协调员 20:10）：① 竖板底 / 脚板随 H03 凸台顶抬 0.5（plate_z 224.5 → 225.0、foot_z 224.5..226.5 → 225.0..227.0）；
#   ② F36 改 M2×6（用户有 M2 自攻 PA ×6，采购清单:72）→ 咬 6.0 − PCB 1.6 = 4.4，孔要 ≥ 4.7：两根立柱在竖板背面（−x）各加 Ø4 × 1.0 背凸台（x −16.6..−15.6），
#   Ø1.7 通孔 −11.6..−16.6 = 5.0（尖 −16.0，离背面 0.6）。M2×8 备选：咬 6.4 → 孔 ≥ 6.7 → 背凸台要 3.0（x −18.6），没采用。hr43e build #1 值：plate_z=(224.5, 245.0), foot_z=(224.5, 226.5)，无背凸台
# hr44（协调员 22:00 小修 ①，hr43e 全链新红 L4 F38_H09_to_H03::tool_path_to_outside 5.62 mm³）：F38 第 2 孔 (−18, 34.7) 的 Ø4.0 批杆（从脚板顶 z 227.0 竖直往上）
#   在 x −16.6..−16.0 切进两个背凸台（H09-F05，各 2.8 mm³）。挪孔要连带 H03 凸台 H03-F24 + 脚板外沿（hr43e 为 L6 刚挪过 y），挪功放要连带 electronics / 线 / L3 / L6 →
#   只在批杆通道上开让位：竖直圆柱 Ø4.1（批杆 Ø4.0 + 半径 0.05，防刻面余量）同轴孔 2，z 227.05..250（脚板顶之上，不碰脚板 / 竖板 x ≥ −15.6 / 立柱）。
#   代价：背凸台 −x −y 那块被削，F36 Ø1.7 孔在 x < −15.965 的 −y 侧壁开口（咬入段 −11.6..−16.0 里只有末端 0.035 不是整圈，见 features H09-F06 / fasteners F36 note）。
AMP_BRACKET_was_until_2026_09_25_hr44 = "同下，无 f38_relief 键（hr43e #2 值）"
AMP_BRACKET = dict(plate_x=(-15.6, -13.6), plate_y=(32.4, 38.0), plate_z=(225.0, 245.0), foot_x=(-24.8, -13.6), foot_y=(32.4, 37.2), foot_z=(225.0, 227.0),
                   foot_corner=dict(x=(-24.8, -23.3), y_max=36.7), post_back=1.0,
                   f38_relief=dict(hole=1, d=4.1, z=(227.05, 250.0)),
                   post_d=4.0, pilot_d=1.7, screw_hole_d=2.4,
                   src=dict(pos="electronics.AMP_S_C/R（研究 b14 候选 ①）", post="功放孔 electronics.amp_S_holes_world()（孔距 12.6 用户定、离顶边 2.55 / Ø2.2 assumed）",
                            pcb_back="AMP_S_C x − 0.8（BX pcb 局部 z −0.8 → 世界 x）", screws="H03_AMPB（F38）"))


def build_amp_bracket():
    """H09（hr43d 新件）：功放支架（见 AMP_BRACKET 注释）。"""
    from . import electronics as E
    k = AMP_BRACKET; (px0, px1), (py0, py1), (pz0, pz1) = k["plate_x"], k["plate_y"], k["plate_z"]
    (fx0, fx1), (fy0, fy1), (fz0, fz1) = k["foot_x"], k["foot_y"], k["foot_z"]
    xb = E.AMP_S_C[0] - 0.8                                                                  # 功放 PCB 背面 x −11.6（局部 z −0.8 → 世界 +x 轴向）
    holes = [(y, z) for (_, y, z) in E.amp_S_holes_world()]
    plate = wbox((px0, py0, pz0), (px1, py1, pz1))
    # was_until_2026_09_25_hr43e: foot = wbox((fx0, fy0, fz0), (fx1 + 0.01, fy1, fz1))
    fc = k.get("foot_corner")                                                                # hr43e：−x 端那截 y 只到 fc.y_max（躲 H03 +y 壁圆角）
    foot = (union(wbox((fc["x"][1], fy0, fz0), (fx1 + 0.01, fy1, fz1)), wbox((fx0, fy0, fz0), (fc["x"][1] + 0.01, fc["y_max"], fz1)))
            if fc else wbox((fx0, fy0, fz0), (fx1 + 0.01, fy1, fz1)))
    posts = [cyl(k["post_d"], xb - px1 + 0.05, ((px1 - 0.05 + xb) / 2, y, z), axis="x", sections=48) for (y, z) in holes]
    bk = k.get("post_back", 0.0)                                                              # hr43e #2：立柱背凸台（竖板 −x 面再伸 bk）
    posts += [cyl(k["post_d"], bk + 0.05, ((px0 - bk + px0 + 0.05) / 2, y, z), axis="x", sections=48) for (y, z) in holes] if bk > 0 else []
    m = union(plate, foot, *posts)
    # was_until_2026_09_25_hr43e: cuts = [cyl(k["pilot_d"], xb - px0 + 2.0, ((px0 + xb) / 2, y, z), axis="x", sections=32) for (y, z) in holes]
    cuts = [cyl(k["pilot_d"], xb - (px0 - bk) + 2.0, ((px0 - bk + xb) / 2, y, z), axis="x", sections=32) for (y, z) in holes]   # F36 Ø1.7 通孔（hr43e #2：含背凸台 5.0）
    cuts += [cyl(k["screw_hole_d"], (fz1 - fz0) + 2.0, (x, y, (fz0 + fz1) / 2), sections=32) for (x, y) in H03_AMPB["xy"]]  # F38 Ø2.4 过孔
    rl = k.get("f38_relief")                                                                 # hr44：F38 第 2 孔批杆通道让位（只削背凸台，脚板顶以上）
    if rl:
        (rx, ry) = H03_AMPB["xy"][rl["hole"]]
        cuts += [cyl(rl["d"], rl["z"][1] - rl["z"][0], (rx, ry, (rl["z"][0] + rl["z"][1]) / 2), sections=96)]
    return keep_main(diff(m, *cuts), "amp_bracket")
