"""zz_* 电子件占位实体（hr37，2026-09-22）：只参与 Gate 检查 / 预览 / MJCF 质量，不打印。
世界系零位姿。每个尺寸的来源写在旁边（datasheet / 商品页标称 / assumed），位置来源 = 仓库里的定案（components.yaml housed_by、head.py 立柱、trunk.py IMU 孔；hr39f 开关删除，XT30 对插头见 xt30_boxes）。
"实物未知不编数"：UBEC / 麦克风板厚针高 是商品页标称或同类通用款，到货卡尺复核后改这里（components.yaml 同步）。
"""
from .lib import wbox
from .s288 import union, cyl, diff

# ── Radxa ZERO 3W（body jaw_soft）── 板 65×30×1.6（radxa.com Tech Spec），孔距 58×23 = H01 四立柱 (58.5, ±29, 251±11.5)，立柱尖 x 66（head.py posts）
SBC_X0 = 66.0                       # 板背面贴立柱尖
SBC_T = 1.6
SBC_L, SBC_W = 65.0, 30.0           # 长沿 y，宽沿 z
SBC_ZC = 251.0
SBC_HDR = (260.16, 265.24)          # hr39c（hr39_头内线束.md §2.1）：按官方机械图（radxa_zero3w_mech_2026-09-19.png「3.3」= 板长边到排针中线，塑座外廓离边 0.69..5.75）
                                    #   → 塑座 z 260.16..265.24；两排针 z 261.43（奇数，内排）/ 263.97（偶数，外排）
SBC_HDR_was_until_2026_09_23_hr39c = (257.4, 262.5)   # hr37 按「Pi Zero 排 1 离上长边 3.5」估的，整体低 2.7
SBC_HDR_H = 8.7                     # 塑座 2.5 + 针 朝 −x（H01 口袋侧）。09-23 用户卡尺：针尖到板另一面（+x 面，不算背面插座）= 针高 + 板厚 = **10.3** → 板厚按官方 1.6 → 8.7；
                                    #   原 8.5（components.yaml 40 针排针高）。板位置由立柱定（定位尺寸不动）→ 针尖往 −x 多伸 0.2 到 x 57.3。整板最厚 11.7（实测）< 包络 8.7+1.6+3.0 = 13.3（包络取大，背面仍 3.0）
SBC_HDR_H_was_until_2026_09_23_measured = 8.5
SBC_COMP_H = 3.0                    # +x 元件面包络高（assumed：USB-C / micro HDMI 座 ≈3.2 最高，SoC 1.2；到货核）
SBC_COMP_Y = 27.0                   # 元件包络 |y|（assumed：两短边各 5.5 只留背面 CSI 翻盖 / 卡座）
SBC_TF_OUT = 2.0                    # TF 卡凸出 −y 板边（实物 ≈1.5，包络取大按 2；09-23 用户看实物）
SBC_TF_Z = (243.0, 259.0)           # TF 卡槽 + 卡沿短边的占位（assumed：居中 16 宽，没量）


# hr39c（hr39_头内线束.md §2.2）：芯片面（−x）下长边的三个口，zz_sbc 原来没建 —— micro-HDMI / USB-C2（USB3 主机）/ USB-C1（OTG + 5V IN）
#   中心离顶短边 12.5 / 41.5 / 54.7（官方正面图）→ y +20.0 / −9.0 / −22.2；开口朝 −z，外伸板边 1.2（到 z 234.8）；口体高（−x 向）按贴板式 3.2（assumed）→ x 62.8..66
SBC_EDGE_PORTS = {"microHDMI": (20.0, 6.6), "USB_C2": (-9.0, 8.94), "USB_C1": (-22.2, 8.94)}   # y 中心, 宽（micro-HDMI 6.6 datasheet 常见值 assumed；USB-C 8.94 规范）
SBC_PORT_H, SBC_PORT_DEPTH, SBC_PORT_PROTRUDE = 3.2, 7.6, 1.2


def sbc_boxes_was_until_2026_09_25_hr43():
    """hr37..hr41：Radxa 竖放在 H01 前立板四立柱尖（x 66）上的占位。hr43 起不再进 ELECTRONICS（新位见下面 sbc_boxes）。"""
    # 板四角 R3（Pi Zero 脚印）：顶壳内顶在 |y| 32 处只有 z 265.2（x 66..67），直角板角 z 266 顶到 0.8，圆角后角点 z 264.7 留 0.5（hr37 探测）
    r = 3.0; y0, y1, z0, z1 = -SBC_L / 2, SBC_L / 2, SBC_ZC - SBC_W / 2, SBC_ZC + SBC_W / 2
    pcb = union(wbox((SBC_X0, y0 + r, z0), (SBC_X0 + SBC_T, y1 - r, z1)), wbox((SBC_X0, y0, z0 + r), (SBC_X0 + SBC_T, y1, z1 - r)),
                *[cyl(2 * r, SBC_T, (SBC_X0 + SBC_T / 2, yy, zz), axis="x") for yy in (y0 + r, y1 - r) for zz in (z0 + r, z1 - r)])
    hdr = wbox((SBC_X0 - SBC_HDR_H, -25.4, SBC_HDR[0]), (SBC_X0, 25.4, SBC_HDR[1]))   # 塑座 2.5 + 针 6（hr39c：z 按官方图 260.16..265.24）
    comps = wbox((SBC_X0 + SBC_T, -SBC_COMP_Y, SBC_ZC - SBC_W / 2 + 2.0), (SBC_X0 + SBC_T + SBC_COMP_H, SBC_COMP_Y, SBC_ZC + SBC_W / 2 - 2.0))
    ports = [wbox((SBC_X0 - SBC_PORT_H, yc - w / 2, z0 - SBC_PORT_PROTRUDE), (SBC_X0, yc + w / 2, z0 - SBC_PORT_PROTRUDE + SBC_PORT_DEPTH)) for (yc, w) in SBC_EDGE_PORTS.values()]
    # 口壳按「官方图口中心 + USB-C 规范 8.94 壳宽 + 贴板式 3.2 高」推的（assumed）：USB-C1 外缘（y −26.67）会压进 H01 冻结立柱 Ø5 (y −29, z 239.5) 0.17。
    #   立柱是原版 Pi Zero 脚印、装 Radxa 早就验证过的 → 真实口壳必然窄于 8.94 或更靠里；这里把立柱体积从口壳里扣掉（不让假想的 0.17 冒充干涉），到货量口壳宽回填。
    #   hr39c-OTG（agent #4，09-23 主 agent 定）：改成在 H01 −y 下立柱上局部让位（head.H01_OTG_RELIEF，螺丝孔/端面位置不动），口壳恢复规范全宽 8.94 —— OTG 口以后刷机/调试要插线。
    # hr39c-TF（09-23 用户看实物，另一会话转来）：TF 卡槽在背面（+x 面）−y 短端（USB-C1 那头），卡**常驻**；插到底凸出板边 ≈1.5、含卡总长 66.5 → 按「包络取大」留到板边外 2（y −34.5）。
    #   沿短边（z）的位置/宽没量 → assumed 居中 16 宽（z 243..259，卡 11 宽 + 卡座两边余量）；厚按背面包络 3.0（SBC_COMP_H，用户：背面目测凸起 ≈1.5，包络取大仍 3.0）。
    tf = wbox((SBC_X0 + SBC_T, y0 - SBC_TF_OUT, SBC_TF_Z[0]), (SBC_X0 + SBC_T + SBC_COMP_H, -SBC_COMP_Y, SBC_TF_Z[1]))
    return union(pcb, hdr, comps, tf, *ports)


# ━━ hr43（2026-09-25）Radxa 搬后脑顶部 —— 方案 A（用户 2026-09-24 17:25 定，docs/design_2026-09-17_bearing_rebuild/hr43_设计稿_Radxa后脑A.md）━━
#   平放、SoC/排针朝上（+z），吊在 H05 顶壳内面长出的 4 根柱下，M2 自攻 + 小平垫从板底拧进柱（fasteners F37）。
#   几何 = 上面旧板（同一套 SBC_* 常量，竖放在 H01 立柱位）整体刚体变换：世界 = SBC_A_R · (p − SBC_OLD_C) + SBC_A_C
#   （照 hr40_layout_work/rear_extend/design_A_optimize.py：旧板中面中心 C0 (66.8, 0, 251) → 新中心 (−3, −2, 257)；RA 把旧 −x（排针/芯片面）转到 +z）。
#   → 板 x −18..12（30 沿 x）× y −34.5..30.5（65 沿 y）× **z 256.2..257.8**（板顶 = 元件/排针面 257.8）；排针塑座 x 6.16..11.24（中线 x 8.7 = 板边 12 − 3.3，官方图）；
#     USB-C ×2 + micro-HDMI 在 −x 长边（口朝 −x，x −19.2..−11.6）；TF 在底面 −y 端；旧「背面元件包络」3.0（SBC_COMP_H）转到底面。
#   ⚠ 设计稿 §2 表「板 z 255.2..256.8」与同表「中心 (−3,−2,257)」「CSI 座口 z 255.4（在板底）」「SoC 顶 258.5」、脚本的柱长起点 257.8 都矛盾 →
#     按脚本（所有间隙数的出处）取 256.2..257.8；hr43_改件报告「待主设计定」列出。
SBC_OLD_C = (SBC_X0 + SBC_T / 2, 0.0, SBC_ZC)          # (66.8, 0, 251) 旧板中面中心 = design_A_optimize.py 的 C0
SBC_A_C_was_until_2026_09_25_hr43c = (-3.0, -2.0, 257.0)   # design_A_optimize.json A.board_center（hr43）
# hr43c（2026-09-25，用户 13:0x 定「后脑零鼓包 + 杜邦直插四件」；协调员 13:35 要 dy 四档比选，hr43_work/hr43c/c1_dy_sweep.json / c1b_board_r245.json）：
#   板整体平移 (dx, dy, dz) = (−0.5, +1.0, −0.5)。dy ∈ {0.75, 1.0, 1.3, 2.0} 逐针最差（R2 针位、削筋、两端减薄、Rc 2.0）= +1.127 / +1.129 / +1.041 / +0.821，
#   颈扫掠四档都 1.251、H01 1.485；板↔原壳（官方 R2.45 板角）1.093 / 1.284 / 1.513 / 2.030 → 取 +1.0（与 0.75 并列最高、板边更宽；+2.0 差 0.31 > 0.2 不取）。
SBC_A_C = (-3.5, -1.0, 256.5)
HR43C_SBC_SHIFT = tuple(round(a - b, 6) for a, b in zip(SBC_A_C, SBC_A_C_was_until_2026_09_25_hr43c))   # (−0.5, +1.0, −0.5)：随板走的世界系常量都按它平移
SBC_A_R = ((0.0, 0.0, 1.0), (0.0, 1.0, 0.0), (-1.0, 0.0, 0.0))   # design_A_optimize.json A.R（局部 → 世界）
SBC_A_TOP = SBC_A_C[2] + SBC_T / 2                      # 257.8 板顶（SoC/排针面）
SBC_A_HOLES_was_until_2026_09_25_hr43c = ((-14.5, -31.0), (8.5, -31.0), (-14.5, 27.0), (8.5, 27.0))   # 设计稿 §2 板孔世界位（58×23）= 旧 H01 立柱孔 (y ±29, z 251±11.5) 经同一变换
SBC_A_HOLES = tuple((round(x + HR43C_SBC_SHIFT[0], 6), round(y + HR43C_SBC_SHIFT[1], 6)) for (x, y) in SBC_A_HOLES_was_until_2026_09_25_hr43c)   # hr43c 随板：(−15,−30)/(8,−30)/(−15,28)/(8,28)
SBC_HOLE_D = 2.8                                        # measured（电子件尺寸核实_2026-09-19：Ø2.8）；官方 DXF radxa_zero_3w_v1110_top.dxf 孔圆 r 1.4097
SBC_HOLE_KEEPOUT_R = 3.0                                # 官方 DXF（参考/radxa_zero3w/*.dxf，hr43_work/dxf_holes.json）：四孔中心 r 3.0 内正反面都没有焊盘/元件丝印（最近 3.00..3.54）
SBC_A_TOPCOMP_was_until_2026_09_25_hr43c = ((-16.0, -29.0), (10.0, 25.0))
SBC_A_TOPCOMP = tuple((round(x + HR43C_SBC_SHIFT[0], 6), round(y + HR43C_SBC_SHIFT[1], 6)) for (x, y) in SBC_A_TOPCOMP_was_until_2026_09_25_hr43c)   # hr43c 随板 → x −16.5..9.5 × y −28..26。以下为 hr43 原注：顶面（SoC 面）元件包络 x/y：照旧「背面元件包络」口径（短边各留 2、长边各留 5.5，SBC_COMP_Y 27）镜到顶面；高 SBC_COMP_H 3.0（assumed，brief：沿用）
SBC_CORNER_R = 2.45                                     # hr43c：官方 DXF radxa_zero_3w_v1110_top.dxf BOARD_GEOMETRY_OUTLINE 四角凸度 → R 2.541 / 2.466 / 2.480 / 2.457；包络取大 = 最小 R，取 2.45
SBC_CORNER_R_was_until_2026_09_25_hr43c = 3.0          # 「Pi Zero 脚印」R3（sbc_boxes 里写死；旧竖放版 sbc_boxes_was_until_2026_09_25_hr43 仍用 3.0）
SBC_A_CSI_CONN = ((67.6, 26.5, 242.8), (69.1, 32.25, 259.0))   # CSI 22P 翻盖座（旧板系；components.yaml:sbc_radxa_zero3w.board_faces_2026-09-23 官方图反面实量 = wiring_head ref__sbc_csi_conn）→ 新位在板底 +y 端


def sbc_A_T():
    """旧板系 → 世界（hr43 方案 A）4×4"""
    import numpy as np
    R = np.array(SBC_A_R, float); T = np.eye(4); T[:3, :3] = R
    T[:3, 3] = np.array(SBC_A_C, float) - R @ np.array(SBC_OLD_C, float)
    return T


def sbc_A_pt(p):
    """旧板系一点 → 世界（hr43 方案 A）"""
    import numpy as np
    return (sbc_A_T() @ np.r_[np.asarray(p, float), 1.0])[:3]


def sbc_boxes():
    """hr43 方案 A：Radxa 平放后脑顶部（见上面常量注释）。= 旧板各块（同尺寸）经 sbc_A_T 变换 + 顶面元件包络 3.0 + 四孔 Ø2.8 通孔；
    顶/底两面元件包络在四孔 r 3.0 禁布圈里挖掉（官方 DXF）—— 顶面是 H05 柱端（Ø5.4）贴板处，底面是 M2 螺丝头 + 小平垫。
    USB-C / micro-HDMI 口壳仍按规范全宽 8.94（assumed），OTG 口壳离 (−14.5, −31) 孔心 2.33 < 3.0（与 DXF 不符，同 hr39c H01 立柱那 0.17）→ 柱在口壳旁局部削平（head_top.H05_POSTS）。"""
    from .s288 import placed
    import numpy as np
    r = SBC_CORNER_R; y0, y1, z0, z1 = -SBC_L / 2, SBC_L / 2, SBC_ZC - SBC_W / 2, SBC_ZC + SBC_W / 2
    pcb = union(wbox((SBC_X0, y0 + r, z0), (SBC_X0 + SBC_T, y1 - r, z1)), wbox((SBC_X0, y0, z0 + r), (SBC_X0 + SBC_T, y1, z1 - r)),
                *[cyl(2 * r, SBC_T, (SBC_X0 + SBC_T / 2, yy, zz), axis="x") for yy in (y0 + r, y1 - r) for zz in (z0 + r, z1 - r)])
    hdr = wbox((SBC_X0 - SBC_HDR_H, -25.4, SBC_HDR[0]), (SBC_X0, 25.4, SBC_HDR[1]))
    comps_bot = wbox((SBC_X0 + SBC_T, -SBC_COMP_Y, SBC_ZC - SBC_W / 2 + 2.0), (SBC_X0 + SBC_T + SBC_COMP_H, SBC_COMP_Y, SBC_ZC + SBC_W / 2 - 2.0))
    ports = [wbox((SBC_X0 - SBC_PORT_H, yc - w / 2, z0 - SBC_PORT_PROTRUDE), (SBC_X0, yc + w / 2, z0 - SBC_PORT_PROTRUDE + SBC_PORT_DEPTH)) for (yc, w) in SBC_EDGE_PORTS.values()]
    tf = wbox((SBC_X0 + SBC_T, y0 - SBC_TF_OUT, SBC_TF_Z[0]), (SBC_X0 + SBC_T + SBC_COMP_H, -SBC_COMP_Y, SBC_TF_Z[1]))
    csi = wbox(*SBC_A_CSI_CONN)
    T = sbc_A_T()
    (tx0, ty0), (tx1, ty1) = SBC_A_TOPCOMP
    comps_top = wbox((tx0, ty0, SBC_A_TOP), (tx1, ty1, SBC_A_TOP + SBC_COMP_H))
    keep = [cyl(2 * SBC_HOLE_KEEPOUT_R, 30.0, (x, y, SBC_A_TOP)) for (x, y) in SBC_A_HOLES]
    comps = diff(union(placed(comps_bot, T), comps_top), *keep)
    holes = [cyl(SBC_HOLE_D, 4.0, (x, y, SBC_A_TOP - SBC_T / 2), sections=32) for (x, y) in SBC_A_HOLES]
    board = diff(placed(pcb, T), *holes)
    return union(board, comps, *[placed(p, T) for p in (hdr, tf, csi, *ports)])


# ── hr43 40P 直角转接 + 平插杜邦壳（body jaw_soft，**新占位 zz_adapter40**，别和 zz_adapter（宇树总线转接板）混）──
#   实物：嘉钦「GPIO 转接板【双排弯针】」id 993538435240（**未买**）商品页 51×21×12、8.8 g，21 是不是套上后离板面总高**判不出**（物料记录_头部电子件.md）。
#   用户 09-24 17:20 定：排针区上方按 ≥ 21 + 1 = **22** 预留（板面 → 顶壳内面）。brief（hr43 任务书）给的超集：
#     盒 A x −2..26 × y −28..24（40 针 50.8 长，中心 y −2）× z 板面..板面 + 22；盒 B（杜邦壳沿 90° 弯针朝 +x 平插，壳长 14）x 26..40 + 线弯 4 → 44，z 带 262..276（板面 256.8 口径）。
#   hr43 实测两处与超集冲突（hr43_work/explore1.json / explore2.json），按「不编数、只收窄 assumed 值」处理，报告「待主设计定」列出：
#     ① 颈部转动件 N03 在头横滚 −25° 时扫到 x 12..40 的 z 265.4（全 y）→ 板边以外（x > 12）的一切占位必须在 z ≥ 266.5（隙 1.1）；
#        超集原样在 x ≥ 12、z < 266 与扫掠 0 距离 → 盒 A 在板边外那截抬到 266.5 起（板上方 x −2..12 仍从板面起）。
#     ② 喇叭后挡（H05-F03，喇叭座不动）下沿 275.55 → 盒 B 顶取 274.5（隙 1.05）；盒 B 底同 ①。
#   → 实物要求（采购/到货量）：转接本体伸出板边（x 12）以外的部分须在板面上 ≥ 8.7；横针两排 + 杜邦壳须落在板面上 8.7..16.7（z 266.5..274.5）。
ADP40_A_OVER = ((-2.0, -28.0, SBC_A_TOP), (12.0, 24.0, SBC_A_TOP + 22.0))      # 板上方（x ≤ 板边 12）：板面..板面 + 22（用户定 22）
ADP40_A_BEYOND = ((12.0, -28.0, 266.5), (26.0, 24.0, SBC_A_TOP + 22.0))        # 板边外：z 266.5 起（N03 横滚 −25° 扫到 265.4 + 1.1）
ADP40_B = ((26.0, -28.0, 266.5), (44.0, 24.0, 274.5))                           # 杜邦壳 x 26..40（壳长 14）+ 线弯 4；z 266.5..274.5（① ② 夹出来的可行带）
ADP40_B_was_brief = ((26.0, -28.0, 262.0), (44.0, 24.0, 276.0))                 # brief 原值（板面 256.8 口径）：与 N03 扫掠、喇叭后挡相交，见上
# hr43c（2026-09-25）：40P 直角转接 zz_adapter40 作废（用户：后脑不鼓包、直角转接不买）→ ADP40_* / adapter40_boxes 只留痕，不再进 ELECTRONICS；H05 鼓包 F10 同退役。
ADP40_TOP = SBC_A_TOP + 22.0                                                     # 279.8：盒 A 顶（H05 内面要 ≥ 它 + 1.0，head_top.HR43_BULGE）


def adapter40_boxes():
    """hr43：40P 直角转接 + 15 个平插杜邦壳 + 线弯的包络（assumed 超集，见上）。扣掉 zz_sbc（排针/元件包络在盒 A 里面，两个占位不重叠）。"""
    return diff(union(wbox(*ADP40_A_OVER), wbox(*ADP40_A_BEYOND), wbox(*ADP40_B)), sbc_boxes())


# ━━ hr43c（2026-09-25）40P 排针直插杜邦（新占位 **zz_gpio_dupont**，body jaw_soft；取代 zz_adapter40 —— 名字不再以 zz_adapter 开头，免得被 bus_adapter 的前缀认领）━━
#   用户 09-25 13:0x 定：H05 后脑零鼓包、40P 直角转接作废，排针直接插已买的 2.54 杜邦母壳（hr43_work/agent_brief_hr43c.md，依据 无鼓包研究.md §0/§2.5/§2.8）。
#   胶壳 2.54 × 2.8 × 14.0（用户实量；2.8 是哪个方向没说 → 占位取两种朝向的并集「十字」，两排/相邻壳在占位里连成一片）；线 OD 1.4（商品页）；
#   壳底坐在排针塑座顶（板面 + 2.5，官方图 8.7 = 塑座 2.5 + 针 6.2）→ 壳顶 = 板面 + 16.5（273.8）。
#   针位 R2（wiring_head.PINMAP，harness HB04/05/08/10）：14 个壳；一分二（12 / 35 / 17）单壳进两根线 → 两线竖直叠放（上面那根多走 od）；
#   3 号（后排）与 4 号（前排）同 y 两排都插 → 3 号线竖直多走 stack 1.4 再弯，跨过 4 号线（hr43c/c1_dy_sweep.py stack_s 数值求得 = 1.4）。
#   线出壳顶即弯：竖直 s → 中心线弯半径 Rc 2.0（= wiring_head.CABLE["dupont"].min_bend，assumed）朝 +x → 弯完水平。
#   本占位 = 14 个壳 + 17 段弯线管（OD 1.4）；wiring_head 的线从弯完那点（gpio_dup_leg_end）接着往 +x 走（同 hr43 线从盒 B 尾 x 44 接）。扣掉 zz_sbc（排针块 8.7 高那段在 zz_sbc 里）。
GPIO_DUP = dict(w=2.54, w2=2.8, L=14.0, plastic=2.5, od=1.4, Rc=2.0, stack=1.4,
                src="胶壳 2.54×2.8×14.0 用户 09-25 13:0x 实量（2.8 方向未知 → 十字并集）；OD 1.4 商品页；塑座 2.5 官方图；Rc 2.0 = CABLE dupont min_bend（assumed）；stack 1.4 = c1_dy_sweep stack_s")
GPIO_DUP_PINS_was_until_2026_09_29_hr52 = (2, 3, 4, 5, 9, 12, 14, 17, 20, 25, 34, 35, 38, 40)   # R2：6→14、39→20、1→17（与麦 VDD 合用一分二）
GPIO_DUP_SPLIT_was_until_2026_09_29_hr52 = (12, 17, 35)
GPIO_DUP_STACK_OVER_was_until_2026_09_29_hr52 = {3: 4}
# hr52（2026-09-29，v6-VL53L5CX 雷达接线，components.yaml:tof_vl53l5cx）：+ 1 号（ToF VIN 3.3 V，原空）、6 号（ToF GND，原空）；
#   3 / 5 号（I²C3 SDA / SCL）改杜邦「一分二 母对二公」单母头（同 12 / 17 / 35 那根），两腿分给 IMU 与 ToF。
#   新冲突（hr52_work/harness/scripts/h00..h04）：
#   · 2 号（功放 VIN，腿朝 −x）正从 1 号壳顶上方过 → 2 号先竖直多走 1.4 再弯（同 3 跨 4 机制）；
#   · 6 号插上后 5 号（后排朝 +x）要跨 6 号壳 —— 主会话原定 5 号竖直多走 1.4 跨过去（上腿 s 2.8），但 5 号在 CSI 排线（HB06 csi43c_c：y 6.85..18.15、x ≤ 8.5 处 z 277.5）+y 边沿正下方，
#     抬起来两腿都切进排线边（h08：各 0.29 mm³）→ 改：5 号一分二**两腿横排、出壳即弯朝 −45°（+x 偏 −y）、s 0**（线管顶 276.5，离排线底 ≥0.85），不再跨 6 号；
#     弯完两腿同心回正到 +x（ToF 腿 R 2 在内、IMU 腿 R 3.4 在外 → y 16.55 / 15.15 两条线贴着走，离 9 号麦线 ≥2.2），6 号照常朝 +x、s 0；
#   · 3 号一分二**两腿横排**（沿 y ±0.7、同 s 1.5，见 GPIO_DUP_SPLIT_SIDE）：按「上腿 s + od」叠放时上腿 s 2.9，弯顶线管顶 279.4 > H05 内面 ≈279.3（h00：−0.17 穿进顶壳）；
#   · 1 号腿（主会话原定朝 +y）弯完线管到 y 25.83 > H05 +y 吊柱 (8, 28) 削平面 y 25.53（h00：−0.30）→ 改朝 37°（+x 偏 +y，GPIO_DUP_LEG_DIR），
#     再在 wiring_head 里 R 2 回正到 +x，从 2 号竖直段与削平面之间挤过去（h04 网格最优：到 H05 0.094、到 2 号 0.064 —— **只有 ≈0.06..0.1，登记为挤点**）；
#     2 号腿同时偏 −y 10°（朝 (−cos10°, −sin10°)，h04：到 H05 0.335、到 3 号横排 +y 腿 0.139）。
#   ⚠ head_top.hr43_post_solids 用 gpio_dupont_boxes().bounds 的 y 上界 + 1.0 削 +y 吊柱：1 号腿让 y 上界 24.53 → ≈24.9，下次重切 H05 时削平面随之外移 ≈0.35（孔壁 1.62 → ≈1.27 ≥ 1.2）。
GPIO_DUP_PINS = (1, 2, 3, 4, 5, 6, 9, 12, 14, 17, 20, 25, 34, 35, 38, 40)   # hr52：+ 1（ToF VIN）/ 6（ToF GND）
GPIO_DUP_SPLIT = (3, 5, 12, 17, 35)                                         # 一分二单壳两线（hr52：+ 3 / 5）
GPIO_DUP_STACK_OVER = {3: 4, 2: 1}                                          # 后排 3 跨前排 4；hr52：前排 2 跨后排 1（朝 −x）（5 跨 6 作废：抬起来切 CSI 排线边，见上）
GPIO_DUP_SPLIT_SIDE = {3: 0.7, 5: 0.7}                                      # hr52：一分二两腿横排（垂直腿方向各偏 ±0.7，两腿同 s；k0 在右手侧 = 朝 +x 时 −y）
# UBEC 两根输出线（4 = +5V、34 = GND）不是杜邦线：UBEC 自带硅胶线 + 1P 杜邦母壳（wiring_head.CABLE["ubec_out_1p"]：OD 1.6、min_bend 4.0，都 assumed）。
#   hr43c/c3b1_verify_pins.json：34 号 Rc 4.0 余 +1.31 → 按声明 4.0 弯；4 号若 Rc 4.0 则 3 号（后排）得竖直多走 3.5 才跨得过、3 号余 −0.97 →
#   4 号只能 Rc 2.0（< 声明 4.0，硅胶线一次性静弯，**到货确认**；报告列未闭合），3 号跨 4 号（OD 1.4 跨 1.6，中心距 ≥ 1.5）竖直多走 1.5 → 3 号余 +1.03。
GPIO_DUP_LEG_CABLE = {4: dict(od=1.6, Rc=2.0, src="UBEC 输出硅胶线 OD 1.6 assumed；Rc 2.0 < ubec_out_1p 声明 min_bend 4.0（被 3 号跨线卡住，到货确认）"),
                      34: dict(od=1.6, Rc=4.0, src="UBEC 输出硅胶线 OD 1.6 / min_bend 4.0 assumed（wiring_head.CABLE ubec_out_1p）")}
GPIO_DUP_STACK_S_was_until_2026_09_29_hr52 = {3: 1.5}
GPIO_DUP_STACK_S = {3: 1.5, 2: 1.4}                                   # 3 号跨 4 号的竖直多走量（OD 1.4 跨 1.6，c3 数值求中心距 ≥ 1.5；两根都 1.4 时是 1.4）；hr52：2 跨 1 多走 1.4
# hr43d（E′：功放挪后脑 +y，Radxa +y 端正下方、壳尾朝 −x）：功放 5 根（针 2 / 12 下腿 / 25 / 35 下腿 / 40）「出壳即弯」改朝 −x（往后脑，研究 §5.2「这 5 根出壳即弯改朝 −x/+y」），
#   其余 12 根照旧朝 +x。前排 2 / 12 / 40 朝 −x 要跨过后排同 y 位（1 / 11 / 39 都没插壳）；一分二 12 / 35 的上腿（麦，s 1.4）照旧朝 +x，两腿出壳即分开。
#   方向 = 水平单位向量 (dx, dy)（缺省 (1, 0) = 朝 +x）；弯顶（中心线）= 壳顶 273.8 + s + Rc，天花板余量见 hr43d/d06_pin_margin.json。
#   40 号（前排最 −y 端）朝 −x 时弯顶离 H05 内面只 0.68（hr43d/d06_pin_margin.json；天花板往 −y 端 / 往后都压低）→ 改斜 45° 朝 (−x, +y)：弯顶挪到 (8.06, −23.72)，
#   那里内面高 ≈0.8（离 38 号竖直段中心距 1.81 − 两线半径 1.4 = 0.41）。
GPIO_DUP_LEG_DIR_was_until_2026_09_25_hr43d_d06 = {(2, 0): (-1.0, 0.0), (12, 0): (-1.0, 0.0), (25, 0): (-1.0, 0.0), (35, 0): (-1.0, 0.0), (40, 0): (-1.0, 0.0)}
GPIO_DUP_LEG_DIR_was_until_2026_09_29_hr52 = {(2, 0): (-1.0, 0.0), (12, 0): (-1.0, 0.0), (25, 0): (-1.0, 0.0), (35, 0): (-1.0, 0.0), (40, 0): (-0.7071, 0.7071)}
GPIO_DUP_LEG_DIR = {(2, 0): (-0.9848, -0.1736), (12, 0): (-1.0, 0.0), (25, 0): (-1.0, 0.0), (35, 0): (-1.0, 0.0), (40, 0): (-0.7071, 0.7071),
                    (1, 0): (0.7986, 0.6018), (5, 0): (0.7071, -0.7071), (5, 1): (0.7071, -0.7071)}   # hr52：2 号偏 −y 10°、1 号（ToF VIN）朝 37°、5 号两腿朝 −45°（见上方 hr52 注）


def gpio_dup_leg_xy(n, k=0):
    """第 n 针第 k 条腿的出壳点 (x, y)：一般 = 针位；hr52 横排一分二（GPIO_DUP_SPLIT_SIDE）两腿垂直腿方向各偏 ±o（k0 右手侧、k1 左手侧）"""
    x, y = gpio_pin_xy(n); o = GPIO_DUP_SPLIT_SIDE.get(n)
    if not o: return (x, y)
    dx, dy = gpio_dup_leg_dir(n, k); sg = 1.0 if k else -1.0
    return (round(x - sg * o * dy, 4), round(y + sg * o * dx, 4))


def gpio_dup_leg_dir(n, k=0):
    """第 n 针第 k 条腿出壳即弯的水平方向 (dx, dy)（hr43d：功放 5 根朝 −x，其余朝 +x）"""
    return GPIO_DUP_LEG_DIR.get((n, k), (1.0, 0.0))


def gpio_dup_leg_cable(n):
    """(od, Rc)：默认杜邦 OD 1.4 / Rc 2.0；UBEC 两根见 GPIO_DUP_LEG_CABLE"""
    c = GPIO_DUP_LEG_CABLE.get(n); return (c["od"], c["Rc"]) if c else (GPIO_DUP["od"], GPIO_DUP["Rc"])


def gpio_pin_xy(n):
    """Radxa 40P 第 n 针（世界系，hr43c 板位）：旧板系针位（hr39 官方图口径，同 wiring_head.pin_y / HDR_Z）经 sbc_A_T；奇数针后排 x 6.93、偶数针前排 x 9.47"""
    p = sbc_A_pt((SBC_X0, 24.13 - 2.54 * ((n - 1) // 2), 261.43 if n % 2 else 263.97))
    return (round(float(p[0]), 4), round(float(p[1]), 4))


def gpio_dup_top():
    return SBC_A_TOP + GPIO_DUP["plastic"] + GPIO_DUP["L"]                  # 273.8


def gpio_dup_legs():
    """[(针, 腿号, s)]：s = 壳顶之上先竖直多走的量（3 号跨 4 号 + stack；一分二上面那根 + od；hr52 横排一分二两腿同 s）"""
    out = []
    for n in GPIO_DUP_PINS:
        s0 = GPIO_DUP_STACK_S.get(n, GPIO_DUP["stack"]) if (n in GPIO_DUP_STACK_OVER and GPIO_DUP_STACK_OVER[n] in GPIO_DUP_PINS) else 0.0
        out.append((n, 0, s0))
        if n in GPIO_DUP_SPLIT: out.append((n, 1, s0 if n in GPIO_DUP_SPLIT_SIDE else s0 + GPIO_DUP["od"]))
    return out


def gpio_dup_leg_path(n, s, npts=13, k=0):
    """壳顶中心 → 竖直 s → Rc 圆弧朝 gpio_dup_leg_dir(n, k) 弯平（中心线；hr43c 全朝 +x，hr43d 功放 5 根朝 −x）。返回 [(x,y,z)]，末点 = 线的接续点（切向 = 弯的方向）
    hr52：出壳点 = gpio_dup_leg_xy(n, k)（横排一分二两腿各偏 ±0.7，其余 = 针位）"""
    import math
    x, y = gpio_dup_leg_xy(n, k); z0 = gpio_dup_top(); Rc = gpio_dup_leg_cable(n)[1]; dx, dy = gpio_dup_leg_dir(n, k)
    pts = [(x, y, z0)] + ([(x, y, z0 + s)] if s > 1e-9 else [])
    pts += [(x + dx * Rc * (1 - math.cos(t)), y + dy * Rc * (1 - math.cos(t)), z0 + s + Rc * math.sin(t)) for t in [math.pi / 2 * j / (npts - 1) for j in range(1, npts)]]
    return pts


def gpio_dup_leg_end(n, s, k=0):
    x, y = gpio_dup_leg_xy(n, k); Rc = gpio_dup_leg_cable(n)[1]; dx, dy = gpio_dup_leg_dir(n, k); return (x + dx * Rc, y + dy * Rc, gpio_dup_top() + s + Rc)   # hr52：出壳点按 gpio_dup_leg_xy


def _tube(pts, r, k=16):
    """折线管（相邻两点各取一个垂直于切向的圆盘，逐段凸包再并），用于弯线段包络"""
    import numpy as np, trimesh
    P = np.asarray(pts, float); segs = []
    T = np.gradient(P, axis=0); T /= np.linalg.norm(T, axis=1, keepdims=True)
    def disc(c, t):
        a = np.array([0.0, 1.0, 0.0]) if abs(t[1]) < 0.9 else np.array([1.0, 0.0, 0.0])
        u = np.cross(t, a); u /= np.linalg.norm(u); v = np.cross(t, u)
        return [c + r * (np.cos(q) * u + np.sin(q) * v) for q in np.linspace(0, 2 * np.pi, k, endpoint=False)]
    for i in range(len(P) - 1):
        segs.append(trimesh.convex.convex_hull(np.array(disc(P[i], T[i]) + disc(P[i + 1], T[i + 1]))))
    return union(*segs) if len(segs) > 1 else segs[0]


def gpio_dupont_boxes():
    """hr43c：14 个直插杜邦壳（十字并集，壳底 = 塑座顶）+ 17 段出壳即弯的弯线管（OD 1.4，Rc 2.0）；扣掉 zz_sbc。hr52 起 16 壳 / 21 段（+ 1 / 6 号、3 / 5 一分二，复审 r1 m5）。"""
    g = GPIO_DUP; zb = SBC_A_TOP + g["plastic"]; zt = gpio_dup_top(); out = []
    for n in GPIO_DUP_PINS:
        x, y = gpio_pin_xy(n)
        out.append(wbox((x - g["w"] / 2, y - g["w2"] / 2, zb), (x + g["w"] / 2, y + g["w2"] / 2, zt)))
        out.append(wbox((x - g["w2"] / 2, y - g["w"] / 2, zb), (x + g["w2"] / 2, y + g["w"] / 2, zt)))
    for (n, k, s) in gpio_dup_legs():
        # was_until_2026_09_25_hr43d: out.append(_tube(gpio_dup_leg_path(n, s), gpio_dup_leg_cable(n)[0] / 2))
        out.append(_tube(gpio_dup_leg_path(n, s, k=k), gpio_dup_leg_cable(n)[0] / 2))   # hr43d：腿方向按 (n, k)（功放 5 根朝 −x）
    return diff(union(*out), sbc_boxes())


# ── IMU Adafruit 4438（body trunk_base）── 板 25.4×17.78×1.6（Adafruit 板文件），贴 T01 前壁外面 x −21.1，长边沿 y，芯片中心 = 冻结 site (−21, 0, 105.3)，孔 (±10.16, 111.65)
IMU_X0 = -21.1                      # 前壁外面（仓在 −x 侧：内表面 BX1 −24.6、壁 3.5）；板贴在 +x 侧、元件朝 +x（中央通道，起子从 +x 进）
def imu_boxes():
    pcb = wbox((IMU_X0, -12.7, 105.3 - 8.89), (IMU_X0 + 1.6, 12.7, 105.3 + 8.89))
    comps = wbox((IMU_X0 + 1.6, -12.7, 105.3 - 5.0), (IMU_X0 + 1.6 + 3.0, 12.7, 105.3 + 5.0))   # 总高 4.6 = 板 1.6 + 侧插座 2.9 + 元件（datasheet 25.6×17.8×4.6），两个 SH 座在短边中点
    return union(pcb, comps)


# ── 总开关 KCD1-101：hr39f（2026-09-23）删除（取消开关，断电 = 开 B03 拔 XT30；旧 switch_boxes 见 hr39f_work/src_before/electronics.py）──

# ── XT30 对插头 + 平衡头（body trunk_base，hr39f）── 盖上 B03 时的状态：电池母头 + 40 cm 延长线公头对插，沿 y 平放在电池顶前段，平衡头叠在上面。
#    只给 Gate 查"盖子下面够不够高"。尺寸：Amass XT30U 规格书 V1.2 —— 公头长 13.70、母头长 12.40、插入段 6.00 → 对插长 20.1；截面 10.20 × 5.20
#    （datasheet）。平衡头型号没确认，按常见 XH 4P 11.4 × 5.8 × 6.4 **假设**（hr39 §1，到货核）。摆放照 hr39 §3.4（**假设**：花牌电池的线从顶面哪出来没量）。
#    电池顶 z = BZ1 − BAT_CLR = 146.3；电池 x −43.2..−25.2（前段 −36..−25.2 上方到 B03 内表面 158.4..159.6）。
XT30_MATED = (10.2, 20.1, 5.2)       # x × y × z（沿 y 平放）
XT30_X1 = -25.5                      # 前沿离电池前面 0.3
BAL_PLUG = (6.4, 11.4, 5.8)          # x × y × z，assumed（XH 4P）
BAL_X1 = -27.2                       # 叠在对插头上，前沿退到 −27.2（B03 前舌片根在 |y|≥6.5，平衡头 |y|≤5.7）
def xt30_boxes():
    from .lib import battery_box
    z0 = float(battery_box().bounds[1][2])      # hr48：电池顶跟 lib.battery_box() 走（实测 72.1 高居中 → 146.35；was 146.3 写死 = 72 高时 BZ1 − BAT_CLR）；前沿 XT30_X1 −25.5 离新电池前面（−25.0）0.5
    mx, my, mz = XT30_MATED
    mated = wbox((XT30_X1 - mx, -my / 2, z0), (XT30_X1, my / 2, z0 + mz))
    bx_, by_, bz_ = BAL_PLUG
    bal = wbox((BAL_X1 - bx_, -by_ / 2, z0 + mz), (BAL_X1, by_ / 2, z0 + mz + bz_))
    return union(mated, bal)


# ── 麦克风 方型 I²S（敏芯，body jaw_soft）── 板 10×10（商品页标称）×1.6（assumed）；3+3 直排针 11 mm 标准（塑座 2.5 + 露针 6）；芯片顶面声孔。
#    位置：H03 地板 (z 231.1) 上的麦克风台（head.MIC_PAD）之上，平放、芯片朝 −z 对着地板 Ø1.5 声孔；+x 边落在 H04 内面的 0.5 深浅槽里。
_T_MIC = (12.07, 6.31, 2.46)         # hr39c：= head.T_MIC（麦克风跟 H03 地板 / H04 脸板走）
MIC_X = (70.6 + _T_MIC[0], 80.6 + _T_MIC[0])
MIC_Y = (24.0 + _T_MIC[1], 34.0 + _T_MIC[1])
MIC_Z0 = 234.2 + _T_MIC[2]           # 台顶（PCB 底面）；hr38：233.0→234.2 随 head.MIC_PAD（短针槽）；hr39c +2.46
MIC_X_was_until_2026_09_23_hr39c, MIC_Y_was_until_2026_09_23_hr39c, MIC_Z0_was_until_2026_09_23_hr39c = (70.6, 80.6), (24.0, 34.0), 234.2
MIC_PIN_SHORT = 2.5                 # hr38：排针短脚在芯片面（朝 −z）露 2.5（用户 2026-09-23 看实物）
MIC_T = 1.6
MIC_CHIP = (4.0, 1.2)               # 芯片 Ø / 高（朝 −z 进台上的 Ø4.2 沉坑）
MIC_HDR_H = 8.5                     # 塑座 2.5 + 露针 6，朝 +z
MIC_DUPONT_H = 14.0                 # 单排 3P 杜邦母壳 7.62×2.54×14 套在针上（购物车 10 cm 母对母撕 3P）
def mic_boxes():
    x0, x1 = MIC_X; y0, y1 = MIC_Y
    pcb = wbox((x0, y0, MIC_Z0), (x1, y1, MIC_Z0 + MIC_T))
    chip = cyl(MIC_CHIP[0], MIC_CHIP[1], ((x0 + x1) / 2, (y0 + y1) / 2, MIC_Z0 - MIC_CHIP[1] / 2))
    # 两排针在 ±y 边内 1.27（2.54 排距 7.62 之外的两排）：塑座 + 杜邦壳合成一个 7.62×2.54 截面、高 2.5+14 的柱（针 6 全在壳里）
    rows = []
    for yc in (y0 + 1.27, y1 - 1.27):
        rows.append(wbox(((x0 + x1) / 2 - 3.81, yc - 1.27, MIC_Z0 + MIC_T), ((x0 + x1) / 2 + 3.81, yc + 1.27, MIC_Z0 + MIC_T + 2.5 + MIC_DUPONT_H)))
        rows.append(wbox(((x0 + x1) / 2 - 3.81, yc - 0.32, MIC_Z0 - MIC_PIN_SHORT), ((x0 + x1) / 2 + 3.81, yc + 0.32, MIC_Z0)))   # hr38：短针 0.64 方 × 2.5 朝 −z
    return union(pcb, chip, *rows)


# ── 摄像头 Radxa Camera 8M 219（body jaw_soft）── PCB 32×32×1.6、4×Ø2 孔距 28×28、镜座 13.2×13.2 高 14（Radxa 官方机械图，电子件尺寸核实 §2）
#    **09-23 到货卡尺实测（用户，另一会话转来；components.yaml:camera_csi.dims_measured）**：PCB 32×32 与官方图一致；镜头最前端到背面 FPC 翻盖座最高点总厚 **21.5 ±0.2**；
#    背面 FPC 翻盖座 **从 PCB 背面起高 5.1、沿边宽 20.1**（原假设 2 高 × 16 宽）；座离板边的深度没量 → 仍按 5（assumed）；PCB 背面到镜头前端 = 21.5 − 5.1 = **16.4**（官方 15.98，多 0.42）。
#    PCB 厚 1.6 / 镜座 13.2 方 × 14 高 仍按官方图（没单独量）→ 多出来的 0.42 记在镜头凸出镜座那段（lens_h 0.38 → 0.8，assumed 分配）。
#    位置：H04 脸板背面 4 根立柱（head.CAM_POSTS），PCB 前面 = 立柱端面 86.27（不变），镜座穿脸板 13.6 方孔。
CAM_POST_REMOVED_SHIFT = 5.9                 # hr41（用户 09-23 23:53 定）：去 H04 立柱（长 5.9），PCB 直接贴脸板内面 92.17 → 整个模组沿 +x 前移 5.9，光轴 (y 0, z 257.94) 不变
CAM_PCB_X = (72.6 + 12.07 + CAM_POST_REMOVED_SHIFT, 74.2 + 12.07 + CAM_POST_REMOVED_SHIFT)   # 90.57..92.17（PCB 前面 = head.H04_PLATE_IN）
CAM_PCB_X_was_until_2026_09_24_hr41 = (72.6 + 12.07, 74.2 + 12.07)   # hr39c：PCB 前面 = 立柱端面 86.27
CAM_C = (0.0, 251.0 + 6.94)                  # (y, z) 镜头光轴 = H04 方孔中心（hr39c）
CAM_PCB_X_was_until_2026_09_23_hr39c, CAM_C_was_until_2026_09_23_hr39c = (72.6, 74.2), (0.0, 251.0)
CAM_BACK_TO_LENS = 16.4                      # measured（09-23 卡尺：总厚 21.5 ±0.2 − FPC 座高 5.1）；官方 15.98
CAM_FPC_SEAT = dict(h=5.1, w=20.1, d=6.0, src="h/w measured 09-23（高 5.1 从 PCB 背面起、沿边宽 20.1）；座外侧与板边齐平（用户确认）；d 6.0 = 照片按板宽 32 比例粗估 5–6、包络取大（没卡尺）")
CAM_BACK_TO_LENS_was_until_2026_09_23_measured, CAM_FPC_SEAT_was_until_2026_09_23_measured = 15.98, dict(h=2.0, w=16.0, d=5.0)
# hr39c-⑦（agent #4，主 agent 09-23 定）：摄像头模组**倒装 180°**（绕光轴转，图像在软件里翻转）→ 背面 FPC 翻盖座从底边换到**顶边**，排线朝 +z 出、翻过转接板/Radxa 上方。
#   光轴（镜头中心）世界坐标不动（圆眼 E01/E02 以它为中心）；PCB 32×32、4 孔 28×28、镜座 13.2 方在本模型里都关于光轴对称（镜头居中 = 官方图口径，到货未单独核），
#   转 180° 后 PCB / 镜座 / 立柱 / 方孔全不变，只有 FPC 座换边。原来座在底边时排线出不了座（新硬伤 ⑦：座后 1.17、座下 8.0 < 16），见 wiring_head.CSI_CAM。
CAM_FLIPPED = True
CAM_HOLE_D, CAM_HOLE_PITCH = 2.0, 28.0      # hr41：PCB 4 个安装孔 Ø2 @ 28×28（Radxa 官方机械图）建进占位 —— H04 四角定位销（head.H04_CAM_PIN）插在孔里
def camera_boxes():
    x0, x1 = CAM_PCB_X; yc, zc = CAM_C
    pcb = wbox((x0, yc - 16.0, zc - 16.0), (x1, yc + 16.0, zc + 16.0))
    pcb = diff(pcb, *[cyl(CAM_HOLE_D, x1 - x0 + 1.0, ((x0 + x1) / 2, yc + sy * CAM_HOLE_PITCH / 2, zc + sz * CAM_HOLE_PITCH / 2), axis="x", sections=32)
                      for sy in (1, -1) for sz in (1, -1)])
    holder = wbox((x1, yc - 6.6, zc - 6.6), (x1 + 14.0, yc + 6.6, zc + 6.6))
    lens_h = CAM_BACK_TO_LENS - 1.6 - 14.0                                                            # 镜头前端 = PCB 背面 + 16.4（measured）→ 镜座顶之上 0.8
    lens = cyl(8.0, lens_h, (x1 + 14.0 + lens_h / 2, yc, zc), axis="x")
    h, w, d = CAM_FPC_SEAT["h"], CAM_FPC_SEAT["w"], CAM_FPC_SEAT["d"]
    if CAM_FLIPPED:
        fpc = wbox((x0 - h, yc - w / 2, zc + 16.0 - d), (x0, yc + w / 2, zc + 16.0))                       # 倒装：翻盖座在背面**顶边**（高 5.1 宽 20.1 measured，深 5 assumed）
    else:
        fpc = wbox((x0 - h, yc - w / 2, zc - 16.0), (x0, yc + w / 2, zc - 16.0 + d))                       # 正装：背面底边
    return union(pcb, holder, lens, fpc)


# ── UBEC-3A（科亿航模，26 × 12 × 6，商品页规格表；hr38 起 **body jaw_soft** —— 挪进头里）──
#    hr37 的「躯干 −y 侧腔 15×4.5×18 腔包络」**作废**（用户 2026-09-23：「就是用我采购的呀，那个 CAD 现画的东西我根本就买不到」）：
#    26×12×6 在两个侧腔三个朝向都放不进（x 向只有 17、y 向只有 6.5）。
#    新位：H03 地板 −y 侧，麦克风台的镜像位（head.UBEC_MOD / head.ubec_pad，H03-F13）—— **立放**，6 沿 x、12 沿 y、26 沿 z。
#    定位依据见 head.py:UBEC_PAD 上面那段注释（体素扫描：整个头里只有三处站得下，这是唯一一处坐在打印件上的）。
# hr39c（agent #4，09-23 用户：按商品页 UBEC-3A 产品图，**输入裸线与输出舵机插头从两头各出一组**，没到货）→ 模块在座里抬高 UBEC_RAISE，底端下面留线弯 90° 的空间（座改法见 head.ubec_pad）
UBEC_RAISE = 5.0
UBEC_BOX = ((72.3 + 12.07, -31.7 - 5.59, 233.0 + 2.46 + UBEC_RAISE), (78.3 + 12.07, -19.7 - 5.59, 259.0 + 2.46 + UBEC_RAISE))   # hr39c：= head.UBEC_MOD（整组平移 head.T_UBEC）+ 抬高 5
UBEC_BOX_was_until_2026_09_23_hr39c = ((72.3, -31.7, 233.0), (78.3, -19.7, 259.0))
UBEC_BOX_was_until_2026_09_22_hr37 = ((-40.0, -27.2, 132.0), (-25.0, -22.7, 150.0))   # 躯干 −y 侧腔包络 15×4.5×18（Matek 候选，作废）
def ubec_boxes():
    return wbox(*UBEC_BOX)


# ── WAGO 221-412（目录 20.9×8.5×13.1）── hr37 探测：两侧腔都放不下 —— +y 腔 z ≥142.2 是开关端子，剩 14 高 < 20.9 立放；20.9 长 > 腔 x 向可用 17；
#    −y 腔与 BEC 并排 6.5 + 8.5 > 6.5。**本轮不放实体**（购物车 5 只需重议：改走宇树转接板 XT30 拓展口分 12 V + 开关 2.8 插簧，可 0 只；记录 45）。
def wago_boxes():
    return None


# ── 扬声器 Ø28 4Ω 3W 超薄圆喇叭（body jaw_soft；hr38 2026-09-23，**未买**）── Ø28 × 厚 5 含磁钢（淘宝常见规格标称，assumed；带线）。
#    位置：H05 顶壳内面下的卡座（head_top.py F03）：中心 (50, 2.5)（head_top.SPK_C；偏 +y 2.5 是躲 N03 横滚扫掠体的 −y 高侧），振膜朝 +z 对声孔阵列，z 264.5..269.5（肩面 269.5 = 顶壳内面横筋 x 52.2..53.1 的筋底 270.1 @|y|14 − 0.6）。
#    背面两个焊片 + 出线：中心 6×4×1.5 小凸台（assumed），到货量。
SPK_D, SPK_T = 28.0, 5.0
SPK_Z = None                         # hr39c：由 head_top.SPK_Z_SEAT 推（喇叭顶面贴肩面），见 speaker_boxes
SPK_TERM = (6.0, 4.0, 1.5)          # 背面焊片/出线凸台 长(x)×宽(y)×高(−z)，assumed
def speaker_boxes():
    from .head_top import SPK_C, SPK_Z_SEAT
    cx, cy = SPK_C; z1 = SPK_Z_SEAT; z0 = z1 - SPK_T
    body = cyl(SPK_D, z1 - z0, (cx, cy, (z0 + z1) / 2))
    tl, tw, th = SPK_TERM
    term = wbox((cx - tl / 2, cy - tw / 2, z0 - th), (cx + tl / 2, cy + tw / 2, z0 + 0.01))
    return union(body, term)


# ── MAX98357A I²S 功放模块（body jaw_soft；hr38，**未买**）──
#    板 20×18×1.6（淘宝常见模块标称，assumed）；喇叭 2P 2.54 螺丝端子 6.5×7.6×8.5（KF128 通用，assumed）；7P 2.54 信号焊盘。
#    **整鸭不焊接**（用户硬规矩：不动烙铁）→ hr38_顶壳与扬声器.md §3 那个「买不焊排针版 + 焊 7 根线到焊盘」的方案作废。
#    hr38 体素扫描 + 布尔实测（scratchpad/place_study.py、amp_try*.py）：
#      · **已焊直排针**（针座 2.5 + 杜邦母壳 14 = 单侧 16.5，加端子 8.5 → 叠高 26.6）：在整个头里（x −20..80 × |y|≤38 × z 221..277、
#        五种轴对齐朝向、≥1 mm 净空、占用含 N03 组件绕头横滚 ±30° 扫掠）**一个可行位都没有**；连「裸板 + 端子」20.6×18.6×11.6 也 0 个。
#        零位倒是放得下（−y 侧 x 13..18 × y −27..−6 × z 221..240，= 用户体素扫到的那块），但那正是 N03 穿底壳开口的走廊，一转头就撞。
#      · **已焊弯排针（90°）**：壳沿板面走，叠高降到 板 1.6 + 端子 8.5 ≈ 10.1，壳沿 −y 外伸 14 → **可行**。
#        板位沿用报告原位 x −10..10 × y −9..9 × z 252.5..254.1（H01 上桥顶 249.5 隙 3.0），端子挪到 **+y 边**、弯针在 **−y 边**、壳伸到 y −23。
#        实测：对 H01/H03/H05(去掉功放卡座)/H04/H02/嘴舵机/嘴轴承/J01 0..30° 扫掠/zz_sbc/mic/camera/adapter/speaker/两颗横滚轴承/N01
#        与 N03 组件 ±30°（2.5° 步长，10 件）**交集全 0.000**。
T_AMP = (-6.03, 0.0, 11.0)          # hr39c：功放卡座挂在 H05 后脑内面，跟顶壳走（head_scale.D(0, 0, 262) = (−6.03, 0, 11.0)）
AMP_PCB = ((-10.0 + T_AMP[0], 10.0 + T_AMP[0]), (-9.0, 9.0), (252.5 + T_AMP[2], 254.1 + T_AMP[2]))   # 板底 263.5（hr38 原位 252.5：H01 上桥顶 249.5 隙 3.0）
AMP_PCB_was_until_2026_09_23_hr39c = ((-10.0, 10.0), (-9.0, 9.0), (252.5, 254.1))
AMP_TERM = ((-3.8 + T_AMP[0], 3.8 + T_AMP[0]), (3.5, 9.0), 8.5)        # 螺丝端子：x 范围、y 范围（**+y 边**）、高（从板顶）
AMP_HDR_T, AMP_HDR_X = 2.6, 8.9                  # 弯排针座厚（板上方 2.5 高）/ 排针带半宽（7P × 2.54 = 17.78）
AMP_SHELL_LEN, AMP_SHELL_H = 14.0, 3.04          # 杜邦母壳沿 −y 外伸长度 / 壳高（2.54 + 0.5 余量）
AMP_HDR_Y = (-11.6, -9.0)                        # hr39c（hr39_头内线束.md §2.3，协调员 09-23 更正）：90° 弯针塑座贴板边外 y −11.6..−9.0，杜邦壳顶着塑座插到底 → 壳 y −25.6..−11.6
AMP_COMP_H = 1.5                                 # 元件面其余元件包络高
AMP_EDGE_BARE = 1.2                              # ±y 两条长边各留 1.2 mm 无元件边条给 H05 肩压（assumed，到货核）
AMP_PAD_X = (7.4, 10.0)                          # 旧口径：排针焊盘带 x（直排针版）。弯针版焊盘在 −y 边，本常量只作历史留痕
AMP_SOLDER_H = 0.5                               # 板底焊点凸起（通孔焊盘的锡面）
def amp_boxes_was_until_2026_09_25_hr43():
    """hr38..hr41：弯排针版功放挂在 H05 后脑卡座（F04）。hr43 起不再进 ELECTRONICS（直排针版见下面 amp_boxes）。"""
    (x0, x1), (y0, y1), (z0, z1) = AMP_PCB
    xc = (x0 + x1) / 2
    pcb = wbox((x0, y0, z0), (x1, y1, z1))
    (tx0, tx1), (ty0, ty1), th = AMP_TERM
    term = wbox((tx0, ty0, z1), (tx1, ty1, z1 + th))
    hdr = wbox((xc - AMP_HDR_X, AMP_HDR_Y[0], z1), (xc + AMP_HDR_X, AMP_HDR_Y[1], z1 + 2.54))                           # 弯针塑座（hr39c 更正：板边外 y −11.6..−9.0）
    shell = wbox((xc - AMP_HDR_X, AMP_HDR_Y[0] - AMP_SHELL_LEN, z1), (xc + AMP_HDR_X, AMP_HDR_Y[0], z1 + AMP_SHELL_H))   # 杜邦母壳顶着塑座：y −25.6..−11.6（hr38 画成 −23..−9，少伸 2.6）
    comps = wbox((x0, y0 + AMP_EDGE_BARE, z1), (x1, y1 - AMP_EDGE_BARE, z1 + AMP_COMP_H))
    # 板底锡面只在**真有通孔焊点**的两处：−y 边的 7P 排针焊盘带（x ±8.9、离边 1.27）与 +y 边的螺丝端子脚（x ±3.8）
    solder = union(wbox((xc - AMP_HDR_X, y0, z0 - AMP_SOLDER_H), (xc + AMP_HDR_X, y0 + 1.27, z0 + 0.01)),
                   wbox((tx0, ty0, z0 - AMP_SOLDER_H), (tx1, y1, z0 + 0.01)))
    return union(pcb, term, hdr, shell, comps, solder)


# ━━ hr43（2026-09-25）功放 = **直排针版**（信泰微电子 id 588659005983「BGA 封装已焊排针」紫板，**未买**；物料记录_头部电子件.md）━━
#   用户 09-24 18:20 定：功放搬脸后口袋、托板拧 H01 旧四柱（= 新件 H07，head.build_amp_tray）；Radxa 搬走后前腔空了 → 直针包络放得下（设计稿补记 18:50）。
#   包络盒逐字取 hr40_layout_work/rear_extend/amp_straight_scan.py 的 BX / BEND（局部系：z = 板法线、元件面 +z；x = 17.7 边、y = 19.1 边，端子边 +y、排针边 −y）：
#     pcb 17.7×19.1×1.6（商品页尺寸图）、元件 1.5（assumed）、端子 7.3×7.0×8.5（高 assumed）、板底焊点 1.0、排针塑座在板底 2.5、杜邦壳 14 竖挂朝下、线弯 4（变体 B）。
#   位姿 = hr40_layout_work/rear_extend/designA_amp_straight.json：中心 (56, 0, 263)，R 局部 x → 世界 y、局部 y → 世界 −x（端子边朝 −x）→ 板 x 46.45..65.55 × y ±8.85 × z 262.2..263.8；
#     杜邦壳 x 61.8..64.5 × y ±8.9 × z 245.7..259.7，线弯到 241.7。
#   安装孔：2 个在顶边（端子边）两角，孔距 12.6（商品页尺寸图，用户 09-24 定）；离顶边 2.55（→ 世界 x 49，brief「端子边 x≈49」，assumed）；孔径未标 → **Ø2.2 assumed**（图估 2.5–3，取小 = 保守给自攻）。
AMP_S_C_was_until_2026_09_25_hr43c = (56.0, 0.0, 263.0)
# hr43c（复审 #2 B1）：功放 + H07 托面 / 立柱 / 纵梁整体 −y 2.0（复审要 ≥1.8：板 +y 边 8.85 → 6.85 ≤ 7.05、立柱心 6.3 → 4.3 ≤ 4.7），让出 F22 右前脚 (49.1, 9.35) 的起子通道 KO09（Ø4.65）
AMP_S_C_was_until_2026_09_25_hr43d = (56.0, -2.0, 263.0)
AMP_S_R_was_until_2026_09_25_hr43d = ((0.0, -1.0, 0.0), (1.0, 0.0, 0.0), (0.0, 0.0, 1.0))
# hr43d（2026-09-25，E′：hr43_work/转接板安放研究.md §5.2 候选 ①，adapter_layout/b14_amp_verify.json）：功放从脸后口袋（H07 托板，删）挪到**后脑 +y、Radxa +y 端正下方**，
#   板竖放（板法线 = 世界 x）：局部 z（元件面）→ 世界 +x、局部 y（端子边 +y）→ 世界 +y、局部 x → 世界 −z。包络 AABB x −32.1..−1.5 × y 19.25..38.35 × z 226.9..244.7；
#   杜邦壳挂板背朝 −x（y 20.3..23，壳尾线弯到 x −32.1）、端子朝 +x（y 31.35..38.35，接线口朝 +y 离壳 ≈4..6）；两安装孔 → 世界 (−10.8, 35.8, 242.1 / 229.5)。
#   研究复核（b14）：对 H05 3.04 / H03 3.21 / H01 3.45 / H02 3.47、N03 横滚扫掠 16.6、对 hr43c 线 0 根；hr43d/d01_amp_probe.json 复测同值。固定 = 新件 H09 功放支架（head.build_amp_bracket）。
AMP_S_C = (-10.8, 28.8, 235.8)
AMP_S_R = ((0.0, 0.0, 1.0), (0.0, 1.0, 0.0), (-1.0, 0.0, 0.0))
AMP_S_BX = {   # amp_straight_scan.py BX（逐字）
    "pcb": ((-8.85, -9.55, -0.8), (8.85, 9.55, 0.8)),
    "comps": ((-7.65, -8.35, 0.8), (7.65, 8.35, 2.3)),
    "pin_stubs_top": ((-8.9, -8.42, 0.8), (8.9, -5.88, 2.8)),
    "term": ((-3.65, 2.55, 0.8), (3.65, 9.55, 9.3)),
    "term_solder_bot": ((-3.8, 2.55, -1.8), (3.8, 9.55, -0.8)),
    "hdr_plastic": ((-8.9, -8.42, -3.3), (8.9, -5.88, -0.8)),
    "dupont_shells": ((-8.9, -8.5, -17.3), (8.9, -5.8, -3.3)),
}
AMP_S_BEND = {"wire_bend": ((-8.9, -8.5, -21.3), (8.9, -5.8, -17.3))}   # amp_straight_scan.py BEND（变体 B，杜邦壳尾线弯 4）
AMP_S_HOLES_LOCAL = ((-6.3, 7.0), (6.3, 7.0))     # 局部 (x, y)：孔距 12.6（用户定）；y 7.0 = 离顶边 2.55（assumed，brief x≈49）
AMP_S_HOLE_D = 2.2                               # assumed（商品页未标）


def amp_S_T():
    import numpy as np
    T = np.eye(4); T[:3, :3] = np.array(AMP_S_R, float); T[:3, 3] = AMP_S_C
    return T


def amp_S_holes_world():
    import numpy as np
    return [tuple((amp_S_T() @ np.array([x, y, 0.0, 1.0]))[:3]) for (x, y) in AMP_S_HOLES_LOCAL]


def amp_boxes():
    """hr43 直排针版功放（BX 逐字 + BEND）；PCB 上开两个 Ø2.2 安装孔（孔位/孔径 assumed，见常量注释）。"""
    from .s288 import placed
    T = amp_S_T()
    parts = [wbox(lo, hi) for (lo, hi) in list(AMP_S_BX.values()) + list(AMP_S_BEND.values())]
    # was_until_2026_09_25_hr43d: m = placed(union(*parts), T)
    # was_until_2026_09_25_hr43d: holes = [cyl(AMP_S_HOLE_D, 1.6 + 0.2, (x, y, AMP_S_C[2]), sections=32) for (x, y, _) in amp_S_holes_world()]   # 只穿 PCB 那 1.6（元件包络 BX 口径盖在孔上方，螺丝头不建模）
    # was_until_2026_09_25_hr43d: return diff(m, *holes)
    # hr43d：孔改在局部系开（沿局部 z = 板法线，只穿 PCB 那 1.6；元件包络 BX 口径盖在孔上方，螺丝头不建模）再整体放到世界 ——
    #   旧写法按世界 z 轴开孔，只在板法线 = 世界 z（H07 平放）时对；E′ 板竖放（法线 = 世界 x）后必须跟 R 走。
    holes = [cyl(AMP_S_HOLE_D, 1.6 + 0.2, (x, y, 0.0), sections=32) for (x, y) in AMP_S_HOLES_LOCAL]
    return placed(diff(union(*parts), *holes), T)


# ━━ hr43d（2026-09-25）宇树总线转接板平放在新件 H08 转接板托板上（B′：hr43_work/转接板安放研究.md §5.3/§5.4，agent_brief_hr43d.md 第 1 条）━━
#   零件面朝上、板 1.6 + 零件 7.4（components.yaml:bus_adapter.envelope_mm 40×30×9，卡尺 8.7 按 9.0）；40 沿 y、30 沿 x；两条 30 短边朝 ±y，四个口全部侧出 ±y
#   （components.yaml:bus_adapter.ports_2026-09-20，measured：A 边 PH2.0 + XT30(2+2) 扩展口、B 边 XT30 主输入 + Type-C）。A 边朝 +y、B 边朝 −y（同 hr39c/hr43c：
#   12 V 主干从左前耳（−y）上来、PH 去嘴舵机 / 右前耳（+y）都最近）；口位基准长边 L 在 −x（Type-C / XT30 扩展口离 USB 线 / UBEC 近），插头占位见 wiring_head.ADP_PORT。
#   **脚印 x 51.8..81.8**（研究抬 2.0 版 x 46..76 整体 +x 5.8）：研究位盖住 F22 右前脚 (49.1, 9.35) 的起子通道 KO09（Ø4.6，z 233.2..273.2）——
#   板粘在托板上、托板装在 H03 之前（F35 从 +x 拧，H03 在就被麦克风台墙 / UBEC 挡）→ 18c 拧 F22#3 时起子穿不过板；+5.8 = KO09 半径 2.3 + 0.35（同 H07-F06 Ø5.3 口径）+ 49.1 − 46，
#   板 −x 边 51.8 离 KO09 轴 2.7（hr43d/d02_board_probe.json：板 → 喇叭 1.75 / H05 2.05 / H06 2.7 / 摄像头 3.67；插头 → H01 ≥1.1、H05 ≥4.9）。
#   z：板底 264.5（研究抬 2.0 版）；托面 262.0..264.0 带 0.5 胶坑，1 mm 泡棉胶在坑里（坑底 263.5 + 1.0 = 264.5，head.ADP_TRAY）。
ADP_FLAT = dict(x=(51.8, 81.8), y=(-20.0, 20.0), z0=264.5, pcb_t=1.6, comp_h=7.4,
                src="40×30×9 卡尺（components.yaml:bus_adapter.envelope_mm）；板 1.6 + 零件 7.4 朝上（agent_brief_hr43d 第 1 条）；位置 = 研究 B′ 抬 2.0 版 + x 5.8 让 KO09")
ADP_FLAT_was_until_2026_09_25_hr43d = "zz_adapter = head.adapter_box()：竖放 x 72.5..81.5 × y −24..16 × z 235.4..265.4（head.H03_ADP_X/Y/Z，9 沿 x、40 沿 y、30 沿 z）"


def adapter_boxes():
    """hr43d：转接板平放占位 = PCB 1.6 + 零件包络 7.4（同脚印，外形仍是 30×40×9 盒 = components.yaml envelope_solid）。插头占位在 wiring_head（conn__adp_* / conn__usb_plug_adp）。"""
    a = ADP_FLAT; (x0, x1), (y0, y1), z0 = a["x"], a["y"], a["z0"]
    pcb = wbox((x0, y0, z0), (x1, y1, z0 + a["pcb_t"]))
    comps = wbox((x0, y0, z0 + a["pcb_t"] - 0.01), (x1, y1, z0 + a["pcb_t"] + a["comp_h"]))
    return union(pcb, comps)


# ── ToF「v6-VL53L5CX 雷达」（hr52，body jaw_soft）：板 + 芯片 + 背面元件包络 + 直排针 + 4 个杜邦壳；几何全在 duckstructure/tof.py（与 H04 座、H10 压条同源）
def tof_boxes():
    from . import tof as TOF
    return TOF.module_world()


ELECTRONICS = [   # (body, placed 名, builder)
    ("jaw_soft", "zz_sbc", sbc_boxes),                # hr43：方案 A 后脑顶部平放（旧竖放版 = sbc_boxes_was_until_2026_09_25_hr43）
    # was_until_2026_09_25_hr43c: ("jaw_soft", "zz_adapter40", adapter40_boxes),    # hr43：40P 直角转接 + 平插杜邦壳 + 线弯（assumed 超集，收窄见 ADP40_* 注释）
    ("jaw_soft", "zz_gpio_dupont", gpio_dupont_boxes),  # hr43c：排针直插 14 个杜邦壳 + 出壳即弯的弯线管（R2 针位；直角转接作废）；hr52 起 16 壳 / 21 段
    ("jaw_soft", "zz_mic", mic_boxes),
    ("jaw_soft", "zz_camera", camera_boxes),
    ("jaw_soft", "zz_tof", tof_boxes),              # hr52：左脸 ToF（v6-VL53L5CX 雷达）
    ("trunk_base", "zz_imu", imu_boxes),
    ("trunk_base", "zz_xt30", xt30_boxes),          # hr39f：取代 zz_switch（电池顶 XT30 对插头 + 平衡头，盖上 B03 时的状态）
    ("jaw_soft", "zz_ubec", ubec_boxes),        # hr38：从躯干 −y 侧腔挪进头里（H03 地板 −y 侧 UBEC 座 H03-F13）
    ("jaw_soft", "zz_speaker", speaker_boxes),      # hr38：Ø28 喇叭，H05 顶壳卡座
    ("jaw_soft", "zz_amp", amp_boxes),              # hr38：MAX98357A 功放；hr43：直排针版，脸后口袋 H07 托板上（旧弯针版 = amp_boxes_was_until_2026_09_25_hr43）；hr43d：后脑 +y H09 支架上（E′）
]
