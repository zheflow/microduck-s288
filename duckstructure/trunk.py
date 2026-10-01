"""躯干 T01–T03 与电池仓门 B01：电池仓实体/刀、仓门、躯干、脖子扫掠包络、躯干壳。
电池仓位置常量（BAT_CLR/BX0/BX1/BZ0/BZ1/BAT_Y/RAIL_Y/...）在 lib.py（小腿 L04 也要用它们削让位），仓门/底脚/卡珠常量在这里。"""
import math, numpy as np
from trimesh.transformations import rotation_matrix as rot
from . import s288
from .s288 import S, cyl, union, diff, inter, placed
from .lib import (P, XR, PLT, CLR, BAT_CLR, BX0, BX1, BZ0, BZ1, BAT_Y, RAIL_Y, RAIL_Z0, DOOR_T, DOOR_Z1, DOOR_YLO, DOOR_ADD, BX0_BAT, DOOR_SLOT_HALF_W, DOOR_SLOT_WALL_Z0, BAY_FW, BRACE_Z, BZR0, BZR1,
                  RINGS, RING_T, SHELL_BOSS, SHELL_CLR, NECK_CLR, NECK_HEAD_CLR, BOSS_D, BOSS_PILOT, DILATE6, SLIDE_CLR,
                  shell_seat_z, SHELL_SEAT_D, SHELL_SEAT_MIN_WALL,
                  TW, orig, shell, shell_inner_z, shell_cutter, sfw, drv_self, pt, servo_env, carrier, mount_cut, driven,
                  keep_main, wbox, ybox, minkowski_box, silhouette, flange_slot, hull, horn_cut, idler_hub, idler_cut, conn_cut, conn_hump_sweep)
from .neck import build_neck, build_neck_pitch, neck_screw_heads
from duckstructure.build_fast import produce_extra      # hr46：件的附属实体（钩）接口
from .tail import tie_loops, LID_CUT, lid_strip_pad, DOOR_RECESS, DOOR_POCKET, _prism_xz, _prism_xy   # hr39f：开关 SWITCH_* 删除；B03 顶盖

# ───────────────────────────── 电池仓 ─────────────────────────────
FOOT_X = (BX0, -35.8)                   # 仓门底脚：从竖板前面伸进仓底板下面（hr39f：−36.8 → −35.8，多伸 1.0 进底脚隧道）
FOOT_Z = (BZ0 - 6.3, BZ0 - 3.0)         # 底脚厚 3.3，贴在底板下表面
FOOT_Y = 6.5                            # 底板后段窄条半宽（腿偏航扫到 |y|≈7，不能再宽）
DOOR_FOOT_Y = 4.6                       # hr39f：门底脚半宽 6.5 → 4.6，让出底脚隧道两侧墙 |y| 4.8..6.5
DOOR_SCREW_X = (-44.5, -39.0)           # 2× M2×6 从下往上：穿底脚 3.3 + 咬底板 2.7。hr39f：螺丝取消（F21 可选），孔保留；隧道托板在 −39.0 开 Ø4.4 让螺丝头/起子过
# hr39f 门底脚隧道（T01）：底板窄条下面两道侧墙 + 一块托板把底脚上下夹住（各 0.2 隙）。门只能沿 x 抽插，不能绕门顶转
# （盖上 B03 时门顶被后卡舌挡住，门底往外摆 = 底脚尖下沉 ≈0.5/3° → 顶在托板上）。
TUN_X = (BX0, -35.3)
TUN_Y0 = DOOR_FOOT_Y + 0.2              # 4.8 侧墙内面
TUN_Z = (FOOT_Z[0] - 1.8, FOOT_Z[0] - 0.2)   # 托板 65.6..67.2
DOOR_FINGER = (BX0_BAT - 1.2, (7.0, 11.0), 144.5)    # hr39f：门顶内侧指槽（x 到内面、|y|、z 底）：开盖后指甲从电池后面插下去往后拉门。hr48：内面=槽底 −45.4 → 槽到 −46.6（was −45.0）

SNAP_Z, SNAP_Y, SNAP_D = 144.0, 14.05, 2.0     # 导轨后端的卡珠（Ø2 半圆，凸出 1.0）——仓门上口靠它咬住

def battery_bay():
    """后开口电池仓（实体）。侧导轨只从 z=RAIL_Z0 起、底板分三段收窄 —— 都是照腿绕髋偏航
    ±25~30° 的扫掠包络裁的（实测那一带最近能荡到 |y|=7）。"""
    floor = union(ybox(BX0, -33.0, FOOT_Y, BZ0 - 3, BZ0),                        # 后段窄条（后缘与仓口齐平，仓门竖板贴在外面）
                  ybox(-33.0, -29.0, 12.0, BZ0 - 3, BZ0),                        # 过渡
                  ybox(-29.0, BX1 + BAY_FW, RAIL_Y, BZ0 - 3, BZ0))              # 前段全宽
    front = ybox(BX1, BX1 + BAY_FW, RAIL_Y, BZ0 - 3, BZ1)                        # 前壁（顶到电池顶）
    rails = [wbox((BX0, min(sy * BAT_Y, sy * RAIL_Y), RAIL_Z0),
                  (BX1, max(sy * BAT_Y, sy * RAIL_Y), BZ1)) for sy in (1, -1)]
    snaps = [cyl(SNAP_D, 1.9, (BX0, sy * SNAP_Y, SNAP_Z), axis="y") for sy in (1, -1)]   # 卡珠：仓门推上去咔一下
    walls = [ybox_s(TUN_X[0], TUN_X[1], sy * TUN_Y0, sy * FOOT_Y, TUN_Z[0], BZ0 - 3 + 0.3) for sy in (1, -1)]   # hr39f 底脚隧道侧墙（伸进底板 0.3）
    shelf = ybox(TUN_X[0], TUN_X[1], FOOT_Y, TUN_Z[0], TUN_Z[1])                               # hr39f 托板
    # hr38 **否决**：曾想在 BZ1 加两条仓顶压条防电池上窜 —— 打印方向上完全悬空：T01 是 down_world=[0,0,1]（世界 +z 贴床，床面 z=165.4），
    # 而仓上方 z 147..165.4 T01 自己一片空（射线实测 x −42..−27 × y 0..14 全部空到顶），压条离床 16.5 悬空要支撑，支撑落进 73 深的仓里掏不出来。
    # 而且没必要：仓腔 BZ0..BZ1 高 73.2、电池 72，本来就只能上窜 0.6，够不上"晃"。电池靠侧导轨 0.6 间隙 + 仓门 + 重力定位。
    return union(floor, front, *rails, *snaps, *walls, shelf)

def ybox_s(x0, x1, ya, yb, z0, z1):
    """y 给两个带号值（任意顺序）的方块"""
    return wbox((min(x0, x1), min(ya, yb), min(z0, z1)), (max(x0, x1), max(ya, yb), max(z0, z1)))

def battery_bay_cuts():
    """仓腔 + 仓门螺丝底孔 + 绑带槽"""
    cuts = [wbox((BX0 - 40, -BAT_Y, BZ0), (BX1, BAT_Y, BZ1))]                    # 仓腔（往后一直通到底，电池就是从这儿推进去的）
    cuts += [cyl(1.6, 2.7, (x, 0.0, BZ0 - 1.65)) for x in DOOR_SCREW_X]          # 仓门 2 颗 M2 的底孔（Ø1.6，只穿底板不进仓腔）
    cuts += [ybox(-29.0, -27.0, RAIL_Y + 1, z, z + 3.0) for z in (116.0, 128.0)] # 绑带槽（可选：想免工具就用魔术贴，不装仓门）
    cuts.append(ybox(BX1 - 0.5, BX1 + 3.5, 10.0, BZ1 - 9.0, BZ1 + 1))            # 出线口（hr38 用户解冻后放大：14×7 → 20×10）：2×Ø2.8 电池线 + 2 根舵机总线干线 + 平衡线都从这儿过；前壁 |y|≤RAIL_Y=15，两侧各留 5 壁
    cuts.append(ybox(BX1 - 0.5, BX1 + 3.5, 6.0, BZ1 - 22.0, BZ1 - 9.0))          # 往下延一段（hr38：|y| 3.5 → 6）：XT30 母头在槽里能上下滑，抽电池时插头跟着出来，不用靠扎带反作用力硬拽
    cuts += [cyl(2.2, 8.0, (BX1 + 1.5, sy * 9.0, BZ1 - 12.0), axis="x") for sy in (1, -1)]  # 2 个扎带孔：XT30 母头绑在前壁上（hr39f：改绑延长线）
    # hr39f：推电池的导向倒角 1×45°：两条导轨后端内沿（避开卡珠 z 142.5..145.5）+ 底板窄条后端上沿
    for sy in (1, -1):
        tri = [(BX0 - 0.05, sy * (BAT_Y - 0.05)), (BX0 - 0.05, sy * (BAT_Y + 1.05)), (BX0 + 1.05, sy * (BAT_Y - 0.05))]
        cuts += [_prism_xy(tri, z0, z1) for z0, z1 in ((RAIL_Z0 - 0.1, SNAP_Z - 1.5), (SNAP_Z + 1.5, BZ1 + 0.1))]
    cuts.append(_prism_xz([(BX0 - 0.05, BZ0 + 0.05), (BX0 - 0.05, BZ0 - 1.05), (BX0 + 1.05, BZ0 + 0.05)], -FOOT_Y - 0.1, FOOT_Y + 0.1))
    cuts.append(cyl(4.4, 3.0, (DOOR_SCREW_X[1], 0.0, (TUN_Z[0] + TUN_Z[1]) / 2)))            # hr39f：托板上给可选 F21 的螺丝头/起子孔
    return cuts

def build_battery_door():
    """B01 电池仓门：竖板 + 塞进仓底板下面的底脚。装法是纯 +x 推进去：底脚滑进底板下面的隧道（hr39f）、
    上口两道槽套上导轨后端的两颗卡珠。拆的时候反着来（纯 −x）。
    hr39f（方案 (f)）：免工具。M2 不再拧（孔保留，F21 可选）；门外面全平（去掉底部 2.5 凸指扣）；
    门顶外面开平齐凹槽 + 倒钩窝给 B03 后卡舌（原 F07 平衡线缺口的位置）→ 盖上 B03 门就退不出来；门顶内侧两个指槽（开盖后指甲勾住往后拉）。
    竖板按高度分三段收窄（|y| 8 / 12.9 / 15.5）—— 下段是给腿偏航到极限时让位（x=-48 处腿能荡到 |y|=9.3）。"""
    W = [(FOOT_Z[0], 96.0, DOOR_YLO), (101.0, 119.0, BAT_Y - 0.2), (123.0, 148.0, RAIL_Y + 0.5)]
    x_out = BX0 - DOOR_T - DOOR_ADD                                                            # hr48：门外面 −48.8（was BX0 − DOOR_T = −46.8），三段宽不变
    plate = union(*[ybox(x_out, BX0, w, z0, z1) for z0, z1, w in W])
    plate = union(plate, *[hull(ybox(x_out, BX0, W[i][2], W[i][1] - 0.6, W[i][1]),
                                ybox(x_out, BX0, W[i + 1][2], W[i + 1][0], W[i + 1][0] + 0.6)) for i in (0, 1)])
    foot = ybox(FOOT_X[0], FOOT_X[1], DOOR_FOOT_Y, FOOT_Z[0], FOOT_Z[1])
    # hr39f：底部指扣（原 x −49.3..−46.8 凸出 2.5、z 67.4..73.4）删掉 —— 门外面是仰躺着地面，不许有凸起
    m = union(plate, foot)
    cuts = [cyl(S["mnt_hole_d"], 12.0, (x, 0.0, FOOT_Z[0] + 1.0)) for x in DOOR_SCREW_X]              # 2× M2×6 过孔（09-13：2.2 → 2.4 同 s288.mnt_hole_d）
    cuts += [cyl(SNAP_D + 0.4, 4.0, (BX0 - 0.05, sy * SNAP_Y, SNAP_Z), axis="y") for sy in (1, -1)]   # 卡珠槽
    cuts.append(ybox(x_out - 1, BX0 + 1, 9.5, 106.0, 140.0))                              # 减重窗（顺便看得见电池）
    # hr39f：原 hr38 门顶平衡线缺口（|y|≤6、z 143..149 穿透）作废（充电改在体外），改成 B03 后卡舌的平齐凹槽 + 倒钩窝（不穿透）
    rx, ry, rz = DOOR_RECESS
    cuts.append(wbox((x_out - 1, -ry, rz), (rx, ry, 149.0)))                               # 凹槽 x 外面..外面+1.4（hr48：−48.8..−47.4；was −46.8..−45.4）、|y|≤6、z 141.5..顶
    (px0, px1), py, (pz0, pz1) = DOOR_POCKET
    cuts.append(wbox((px0 - 0.1, -py, pz0), (px1, py, pz1)))                                 # 倒钩窝 x −45.4..−44.8（窝后门厚 1.0）
    fx, (fy0, fy1), fz = DOOR_FINGER
    cuts += [ybox_s(fx, BX0 + 1, sy * fy0, sy * fy1, fz, 149.0) for sy in (1, -1)]           # 门顶内侧指槽 1.2 深 × 4 宽 × 3.5
    # hr28：B02 尾部托架作废（转接板进头里），门顶条那两个 Ø1.7 底孔（hr17 door_pilots）不再开 —— 门回到 hr16 形状
    # hr48（电池实测 20 厚，见 lib.DOOR_ADD 注释）：门内面电池槽 —— 槽底 BX0_BAT（−45.4）；z < DOOR_SLOT_WALL_Z0 的下/中段门比 13.9 窄、留不下 ≥1.2 的槽壁 → 通宽切（段厚 5.0→3.4）；
    #   顶段（z ≥ 120.5，半宽 15.5）槽 |y|≤DOOR_SLOT_HALF_W（12.7），两侧留 2.8 壁夹住电池后端顶部；槽顶 BZ1（电池顶极限 146.9），门顶 146.9..148 留 1.1 唇。
    #   卡珠槽（y ±14.05、x 内面）落在槽壁区，照旧切；指槽 DOOR_FINGER 从槽底再深 1.2。
    cuts.append(wbox((BX0_BAT, -16.0, BZ0 - 0.1), (BX0 + 0.5, 16.0, DOOR_SLOT_WALL_Z0)))
    cuts.append(wbox((BX0_BAT, -DOOR_SLOT_HALF_W, DOOR_SLOT_WALL_Z0 - 0.01), (BX0 + 0.5, DOOR_SLOT_HALF_W, BZ1)))
    return keep_main(diff(m, *cuts), "battery_door")

# ───────────────────────────── 躯干 ─────────────────────────────
_RHY = sfw("trunk_base", 0)                                   # 左髋偏航舵机帧（心 z 131.5，法兰朝下）；甲板两个站位从它推，不写死
DECK_TOP = float(pt(_RHY, S["T"] / 2 + CLR)[2])               # 121.2：腹面甲板上表面 = 载体前板上表面 = S288 本体前端面 131.5−T/2=121.5 − CLR 0.3（旧写死 121.25）
DECK_BOT = float(pt(_RHY, s288.x_flange_face() - 0.5)[2])     # 119.0：甲板/载体前板底面 = 法兰面(118.5) 上 0.5 —— 手册尺寸下正好 = 原版甲板底 119.0，不再削
                                                              # （旧 12.85 法兰面时是 119.15，把原版甲板底削掉 0.15 才齐平）。髋偏航座环从这里往下长到 114.5
# 轭带下缘 z：x>15.5、y 13..16.5 的前板碎片从 DECK_BOT 切到这里（原来写死在 build_trunk 的刀里）；mechanical_audit 判 T02"轭孔里的销"只看 z ≥ 这里
# （下面 z<144 那一段 y 16..16.5 的壳料是壳自己的壁，躯干在那里本来就没有料，不是销）
YOKE_Z0 = 144.0
JAW_YOKE_R = 15.5   # hr50_r2 2026-09-28：T01 轭板前上角按 J01 头 +90° 绕颈轴扫掠削成圆角的半径（J01 最内点 15.911，留 0.41；见 build_trunk 末尾注释）

HIP_YAW_SEAT_NOTCH_X0, HIP_YAW_SEAT_NOTCH_HALF_DEG = 14.8, 40.0   # hr13：座环扇区截止的局部 x（法兰面 13.0 + 1.8）/ 朝腿扇区半角（实测缺口 ±35° 内，留 5°）

def hip_yaw_seat_notch(R, side):
    """hr13：髋偏航 6702 座环朝腿扇区（舵机局部 ey 方向 = 世界 ±y，side=+1 左/−1 右）在 x ≥ HIP_YAW_SEAT_NOTCH_X0 的环带切除体（舵机局部系 → placed(R)）。
    只覆盖座环环带（径向 seat_d/2−0.2 .. ring_od/2+0.3），不碰座环之外的躯干底边；x 上限 = 座环末端 + 0.5。"""
    by = P["brg_yaw"]; xf = S["T"] / 2 + S["flange_h"]
    x0, x1 = HIP_YAW_SEAT_NOTCH_X0, xf + RING_T + 0.5
    ring = diff(cyl(by["ring_od"] + 0.6, x1 - x0, ((x0 + x1) / 2, 0, 0), axis="x", sections=128),
                cyl(by["seat_d"] - 0.4, x1 - x0 + 0.02, ((x0 + x1) / 2, 0, 0), axis="x", sections=128))
    # 扇区 = 以局部 +y·side 为中心、半角 HALF 的楔：用两块半空间的交（楔在 y-z 面里，沿 x 贯通）
    a = math.radians(HIP_YAW_SEAT_NOTCH_HALF_DEG); big = 40.0
    wedge = inter(*[placed(wbox((x0 - 1, 0.0, -big), (x1 + 1, big, big)), rot(sgn * (math.pi / 2 - a), [1.0, 0.0, 0.0])) for sgn in (1, -1)])
    if side < 0:
        wedge = placed(wedge, rot(math.pi, [1.0, 0.0, 0.0]))
    return placed(inter(ring, wedge), R)

def seat_shoulder_caps(*Rs):
    """hr50：髋偏航座环挡肩在 x>15.5 月牙段的加厚盖（世界系）。在载体局部系里用与座环同一个 cyl(ring_od)（同 64 边）取 x 12.1..12.55
    （世界 z 119.4..118.95，与环顶 119.0 重叠 0.05），再截 x ≥ 15.45。"""
    by = P["brg_yaw"]; out = []
    for R in Rs:
        disc = placed(cyl(by["ring_od"], 0.45, (12.325, 0, 0), axis="x"), R)
        out.append(inter(disc, wbox((15.45, -40, 100), (40, 40, 140))))
    return out

# hr50：壳刀清理盒（左侧 y>0 写，右侧 y 取负镜像）。(x0, x1), (|y|0, |y|1), (z0, z1)
SHELLCUT_CLEAN = {
    "端墙上口（U 形挂件）": ((-21.4, -18.7), (21.3, 29.8), (140.3, BZR0)),        # 载体端墙 x −21.3..−18.8 被壳挂件右腿削到 140.4..140.7，+x 面留 0.2 皮到 144.8、−x 面 0.01 皮
    "端墙下口（壳底翻边）": ((-21.4, -18.45), (21.8, 30.6), (120.05, 125.2)),     # 壳底翻边穿过端墙 + 甲板后外角：顶面 120.09..120.67 台阶、端墙 +x 皮下垂 0.5
    "连接板后缘（挂件左腿端头）": ((-24.7, -22.8), (21.4, 22.6), (BZR0 - 0.2, BZR1 + 0.2)),   # 连接板 x −24.6 后缘在挂件左腿圆头处留 0.12 皮
    "前端舵机背盖（壳搁板下）": ((13.8, 15.6), (21.3, 29.6), (141.6, 143.3)),     # 舵机薄背 141.8 与壳搁板刀 142.7 之间 0.4 厚残片 + 碎边（前排不用的螺丝位一带）
}

def shellcut_cleanup():
    return [ybox_s(x0, x1, sy * y0, sy * y1, z0, z1) for (x0, x1), (y0, y1), (z0, z1) in SHELLCUT_CLEAN.values() for sy in (1, -1)]

def build_trunk():
    """躯干：两个髋偏航舵机载体（背板在上，法兰朝下）+ 顶板 + 脖子俯仰从动板（yz 面，朝 -y）
    + 3S 电池仓 + **原版腹面甲板 trunk_base.stl**（照原版重雕的那一块）。

    甲板为什么能用、为什么原版在 z 119..122 而我们的背板在 144.65：
      XL330 的壳体 34×20 只占轴向 ±12（两端各一个 Ø16×2.5 凸台补到 ±14.5），原版是把它夹在**输出端壳体前端面**
      (局部 x=+12 → 世界 z=119.5) 上的：甲板 119.0..120.0 贴在那个面下面，4 根 Ø1.8 定位销插进去。
      S288 唯一的载体接口是背面 4 角孔 → 背板内面 XR=-13.15 → 世界 z=144.65，差的就是这 25.65。
      但 S288 在甲板那个平面上只露一圈 Ø14 法兰（法兰面 118.5、本体前端面 121.5；旧 118.65/121.55），
      原版甲板的两个 Ø18.6 孔正好套得下（单边 2.3 间隙），所以甲板照收，
      再用它自己的投影补厚到 DECK_TOP=121.2 与载体前板齐平 → 腹部从"两个开口 U"变成一整块闭口箱（扭转刚度粗估 ×33）。

    髋偏航 6702 座环（RINGS["left_hip_yaw"]，carrier(ring=True, brg=P["brg_yaw"])）—— 原版此处本来就有一颗 22×16×4（舵盘面平面，受力报告 §4 判为推力式夹持），S288 法兰配不了那种夹法，我们改法兰侧径向 6702（2026-09-12 ⑤）：
      受力分析（docs/reports/受力分析与双支撑决策_2026-09-12.md）髋偏航单腿支撑弯矩 331 N·mm、零副支撑，全机最差；
      原版用 22×16×4 推力轴承夹在甲板与舵盘板之间，S288 的 Ø13.9 塑料法兰复制不了那种夹法，所以在法兰侧加径向轴承：
      外圈座在本件（Ø24.6 环，z 114.5..119.0，座孔 Ø21.15 z 114.0..118.55，挡肩 Ø18.5 只压外圈；旧 114.65..119.15 等），内圈毂在 L01。
      规格是 6702 不是 6704：6704 本体与髋横滚 6704 本体相交 4.39 mm³（lib.P["brg_yaw"] 的注释）。"""
    RL, RR = sfw("trunk_base", 0), sfw("trunk_base", 1)     # (6,±17.5,131.5)，法兰朝下，顶端朝前
    # 左：舵机 y=世界 +y；开口朝外侧(+y)；不要前墙（脖子舵机贴着 x=16）
    cL = placed(carrier(open_side="+y", walls=("-z", "-y"), ring=RINGS["left_hip_yaw"], brg=P["brg_yaw"]), RL)
    cR = placed(carrier(open_side="-y", walls=("-z", "+y"), ring=RINGS["left_hip_yaw"], brg=P["brg_yaw"]), RR)
    zr0 = pt(RL, XR)[2]; zr1 = zr0 + PLT                           # 背板 z（世界 144.8..147.8 = BZR0..BZR1）
    assert abs(zr0 - BZR0) < 1e-6 and abs(zr1 - BZR1) < 1e-6, (zr0, zr1)
    top = wbox((-21.3, -30.3, zr0), (15.5, 30.3, zr1))
    # 脖子俯仰：舵机在脖子里，法兰朝 +y；躯干拿舵盘板
    Rn = drv_self("neck")
    dn, _, xe, xi = driven(ring=RINGS["neck_pitch"], d=26.0)
    pn = placed(dn, Rn); cn = pt(Rn, 0)                                      # (26,0,152.4)
    y0, y1 = pt(Rn, xi)[1], pt(Rn, xe)[1]                                    # 13.0..16.0（法兰面→舵盘板外表面；旧 12.85..15.85）
    yoke = wbox((13.0, y0, zr0), (cn[0] + 13, y1, cn[2] + 13))               # 竖板：从顶板边缘上到轴心+13
    cradle = battery_bay()
    link = wbox((BX1, -30.3, zr0), (-20.0, 30.3, zr1))                                          # 仓前壁顶 → 顶板（重叠 1.3）
    # 原版腹面甲板：1.0mm 薄板 z119..120 + 4 根 Ø1.8 定位销(z119..122)。
    # 底面切到 DECK_BOT 与载体前板齐平：手册尺寸下前板底 = 119.0 = 原版甲板底，这一刀不再切掉任何料（旧 12.85 法兰面时前板底 119.15，
    # 曾把甲板底削 0.15 以免腹面出现两个相差 0.15 的朝下平面）。L01 顶面（毂顶 = 法兰面）118.5，间隙 0.50。
    DECK = diff(orig("trunk_base", "trunk_base"), wbox((-60, -40, 0), (60, 40, DECK_BOT)))
    # 用它自己的 z 向投影补厚到 DECK_TOP。注意：silhouette 的 4mm² 内孔阈值**没有**滤掉原版那 4 个 Ø2.3 的 XL330 下排孔
    # （它们 4.15mm²，险过），孔保留下来了，最终被载体前板从上面堵住，腹面只剩 0.15mm 深的盲窝。两个 Ø18.99 舵机孔和中央 12×6 长槽也都在
    deck2 = silhouette(DECK, 2, span=(DECK_BOT, DECK_TOP))
    m = union(cL, cR, top, pn, yoke, cradle, link, DECK, deck2)
    # x>15.5 的三刀原来从 z=100 起切，那只是为了给脖子舵机（本体 z≥142.9、x≥16）让出前脸；甲板底 119.15 以下本来就没有躯干料
    # （09-12 实测：改起点后座环区以外体积差 −0.0001 mm³）。现在 119.15 以下挂着髋偏航座环（前缘到 x=18.3），所以刀从 DECK_BOT 起。
    # 代价：座环 x 15.5..18.3 那片月牙（2.8 宽×4.5 高，两侧共 214 mm³）倒放打印时是悬空面，走自动支撑，落点是座环顶面（非配合面）。
    cuts = [horn_cut(Rn), servo_env(RL), servo_env(RR), servo_env(Rn), mount_cut(RL), mount_cut(RR),   # horn_cut 才是摆正了的刀；原来的 hn 没 placed，是空刀
            wbox((15.5, -40, DECK_BOT), (60, y0, 150)), wbox((15.5, y1 + 0.5, DECK_BOT), (60, 40, 150)),   # x>15.5 只留脖子轭那一条（脖子舵机/背板都在 y<0 那侧）
            wbox((12.0, -17.0, 140), (15.5, 13.5, 150)),                                              # 缺口：脖子舵机/背板俯仰时角部扫过 x 12..15.5（内前角螺丝放弃）
            wbox((15.5, y0, DECK_BOT), (60, y1 + 0.5, YOKE_Z0))]                                      # 轭带下方的前板碎片（YOKE_Z0=144.0）
    cuts += battery_bay_cuts()
    # hr29：IMU 定型 Adafruit 4438 LSM6DSOX（25.4×17.78×1.6，孔距 20.32 Ø2.5，芯片在板中心）：板中心 = 冻结 site (−21, 0, 105.3)，
    #        两孔落在 (±10.16, 111.65)（板长边沿 y、孔在 z 114.19 那条长边下 2.54）；旧 (±8, 105.3) 两孔在板下面、被板盖住 → 删。M2×5 自攻：板 1.6 + 墙 3.5。
    cuts += [cyl(1.7, 10, (BX1 + 1.5, sy * 10.16, 111.65), axis="x") for sy in (1, -1)]      # IMU 2×M2 自攻底孔（Adafruit 4438）
    # hr29：IMU 线（4 根 2.54 杜邦单针壳，每个 2.54×2.54）要从前壁前面穿到甲板上方的中央槽：墙-甲板缝原来只有 2.25×9（x −21.0..−19.0），壳过不去
    #        → 甲板后缘在 |y|≤4.5 处再切 2 mm，缝加宽到 4×9（x −21.1..−17.0，z 119..121.2）；两髋偏航座之间正中，不碰配合面。
    cuts += [wbox((BX1 + 3.5 - 0.1, -4.5, DECK_BOT - 0.05), (-17.0, 4.5, DECK_TOP + 0.05))]
    # 甲板把髋偏航舵机的侧滑通道填上了，重开 Ø15（法兰+1）滑槽（左载体朝 +y 开口、右朝 -y）。
    # 有座环时刀只开到法兰面下 SLIDE_CLR=0.4（13.4）：法兰从座环顶滑过要这一点，再深就把座环切成 C 形
    xs = S["T"] / 2 + S["flange_h"] + SLIDE_CLR if RINGS["left_hip_yaw"] else None
    cuts += [flange_slot(RL, (0, 1, 0), x1=xs), flange_slot(RR, (0, -1, 0), x1=xs)]
    # 髋偏航 PH2.0 插座（keepouts.yaml:KO01，09-20 背插模型，lib.CONN_MODE）：插座在舵机背面（朝上）两角，插头从**顶板上方**沿 −z 插。
    # 内侧口（左 −y / 右 +y，世界 |y| 7..12.5，x −8.4..1.8）在顶板上开 5.5×10.2 通窗 → 线走顶板上方的配电口袋，装好舵机后再插 → "window"。
    # 外侧口（世界 |y| 22.5..28）的足印正压在冻结的壳柱 (1, ±24) 底下，开窗/挖槽都动壳柱根 → 不开，该口不用（"none"）：
    # 髋偏航只有 1 口 → 当链尾（总线树 harness.yaml:HB01 用 4 根一分二把它单独挂出来）。
    cuts += [conn_cut(RL, sides=(-1,), mode="window"), conn_cut(RR, sides=(1,), mode="window")]
    m = diff(m, *cuts)
    # hr11：髋俯仰 6703 整体座（L03，seat_od 25.55）+ 钢圈在髋横滚 23–26°、髋偏航 15–30° 时甩到躯干底边（z 119..121, |y| 28..30.3）——
    #        hr10 真实姿态对比 trunk_base×upper_leg 左 38/右 78 姿态、最深 12.5 mm³（r5 无座时 0）。座环是绕俯仰轴的回转体，俯仰角无关，
    #        只需先绕零位横滚轴 ±26°、再绕偏航轴 ±30° 二维扫掠（mink 0.4）；右侧镜像（右髋偏航 −30..25 ⊂ ±30）。
    # hr12：hr11 用过的"座环+钢圈绕横滚 ±26°×偏航 ±30° 二维扫掠"是角度全组合，切掉 297 mm³（每侧 134）远超真实需要（hr10 真实姿态左 10.7 / 右 26.2 mm³）
    #        → 改用真实姿态区域刀（tools/sim/policy_pose_regions.py --dir policy_steps_2026-09-17_hr10，hr10 几何），左右区域各自再镜像一份保持对称。
    #        hr11 那版见 docs/design_2026-09-17_bearing_rebuild/hr11/duckstructure_src/trunk.py。
    # was_until_2026_09_28_hr50: from .lib import region_cut
    # was_until_2026_09_28_hr50: m = diff(m, *[region_cut(f, "trunk_base", mirror=mr) for f in ("trunk__pair_trunk_base_x_upper_leg_left.stl", "trunk__pair_trunk_base_x_upper_leg_right.stl") for mr in (False, True)])
    # hr50（2026-09-28 用户定"非常极限的姿势靠训练去限制，就不削了"）：T01#13 / #17 四把区域刀撤掉。复核（hr50_work/poses/region_recheck.md）：
    #   现几何不切 41 / 100 姿态，全部来自 alpha_stand/body_all_max 与 roulade 极限帧（最深 1.45，髋横滚 +24° 超出目标 ±22°）→ 列进重训约束清单。
    #   区域刀留下的 (−6.7, ±28.9, 121.1) 两片刀片（4.5 mm²）、底边马赛克切面随之消失。下面 hr13 的座环扇区截平（hip_yaw_seat_notch）本来说不动，10:2x 主设计复核后也撤了（见下）。
    # hr13 复审（2026-09-18）：上面那两把区域刀正好咬在髋偏航 6702 座环**最低端朝腿的扇区**（L 方位 325°..30°、R 150°..215°，方位 0 = 舵机局部 +y = 世界 +y），
    #        从座环轴向 x 14.8（局部，= 法兰面 13.0 + 1.8）起把 1.72 的座壁削成 0..0.7 的皮、有几向穿透 —— 打印出来是毛刺 + 缺座，hr12 记录漏写、判据也没盯住。
    #        区域刀不能撤：38/78 个命中姿态里有 12/14 个横滚 |≤22°|（在 frozen 目标区间内，例如左 roll 19–22° + yaw 13–30°，右侧最深 10 mm³）。
    #        所以把座环在该扇区**干净地截到 x 14.8**（HIP_YAW_SEAT_NOTCH：扇区 ±HIP_YAW_SEAT_NOTCH_HALF_DEG，径向只覆盖座环环带 seat_d/2−0.2 .. ring_od/2+0.3）：
    #        扇区外 280° 座环全长 4.0 全壁，扇区内座长 13.4..14.8 = 1.4 mm 全壁、之后为空（外圈仍被 280° 全长扇区径向约束）。mechanical_audit 'hip_yaw seat bore' 判据按此改。
    # was_until_2026_09_28_hr50: m = diff(m, hip_yaw_seat_notch(RL, 1), hip_yaw_seat_notch(RR, -1))
    # hr50（2026-09-28 主设计，依据 hr50_work/trunk_neck/notch/notch_check.json + notch_grid.json）：区域刀撤了，座环截平的理由没了 → 撤截平、6702 座环恢复 360° 全长。
    #   复核（不截版，每侧回来 60.5 mm³）：零位 / 各单关节目标区间 1° 扫 / 髋偏航×横滚 1° 网格 × 7 档俯仰 × 3 档膝（每侧 52920 姿态）全 0；
    #   5601 真实姿态只 3 个命中，全在极限段（body_all_max ×2 左 L02 0.149 mm³ / 0.166 mm、roulade ×1 右 L02 0.218 / 0.186，横滚都超 ±22°），在 C 中档内。
    #   mechanical_audit 'hip_yaw seat bore' 同步改回 hr10 口径（24 向全壁）。hip_yaw_seat_notch() 函数本体留着作沿革。
    # hr50（2026-09-28）：座环挡肩在 x>15.5 那段月牙上只剩 0.45 厚（x>15.5 的前脸刀从 DECK_BOT=119.0 起削，挡肩 118.55..119.0；clean_check 薄膜 14.0 / 13.6 mm²）
    #   → 月牙上补一片 0.4 的盖（到 119.4），挡肩 0.85。盖只在 x ≥ 15.45（与甲板重叠 0.05），径向不越过座环外圆（同一 cyl 刀面），不进挡肩内孔。
    m = union(m, *seat_shoulder_caps(RL, RR))
    # hr12：L02 俯仰盘上 F06 六颗 M2 头（露出盘面 0.6）在 roulade/极限姿态擦躯干底边（hr09–hr11 运动审计 trunk×F06_heads 左 38/右 179 姿态、4.6 mm³）——
    #        用同一套 5601 姿态算头×躯干交集并集（scratch region_f06_trunk.py，hr10 几何，229 命中，并集 17.5 mm³）当刀，再镜像一份。
    # was_until_2026_09_28_hr50: m = diff(m, region_cut("trunk__pair_F06_heads_x_trunk_base.stl", "trunk_base"), region_cut("trunk__pair_F06_heads_x_trunk_base.stl", "trunk_base", mirror=True))
    # hr50：F06 区域刀是空刀（现几何 5601 姿态不切也 0 碰撞，最近 0.32，poses/region_recheck.md T01-F06），撤。
    # 壳当刀：侧壁区（|y|≥19）我们让 —— 壳一旦挖开就是可见的窟窿。颈部区反过来由壳让位（见 build_trunk_shell）
    m = diff(m, shell_cutter(1), shell_cutter(-1))
    # hr50（2026-09-28）：壳刀是原版减面壳 ⊕ DILATE6 再沿 y 扫的网格刀，在几处留下 0.01–0.26 的皮和马赛克碎边（clean_check：端墙 +x 皮 51/50 mm²、
    #   端墙 −x 面 5.6/5.2、连接板后缘 3.0、端墙下沿皮 5.9、甲板后外角碎边、前端舵机背盖碎边）→ 按规矩"碎边一刀平面切齐"：几把平盒把那几处整片切平
    #   （每把盒 = 壳刀在该处已削区域外扩到最近的平面；不碰螺丝孔 / 壳柱 / 座环；左右对称）。见 SHELLCUT_CLEAN。
    m = diff(m, *shellcut_cleanup())
    # 躯干壳的 4 根安装柱：从顶板顶面长到壳内表面下 0.2，Ø5，顶上 Ø1.6 底孔。
    # 螺丝从**壳外面**往下拧（M2×8：穿壳 2.2 + 进柱 5.8）—— 反过来从顶板底下拧的话只有 23mm 净空，螺丝刀伸不进去
    bosses, pilots = [], []
    for (x, y) in SHELL_BOSS:
        for sy in (1, -1):
            z_in, z_out = shell_inner_z(sy, x, sy * y)
            ztip = z_in - 0.2
            assert ztip - BZR1 > 6.0, f"壳柱 ({x},{sy*y}) 太矮 {ztip - BZR1:.1f}"
            bosses.append(cyl(BOSS_D, ztip - BZR1, (x, sy * y, (BZR1 + ztip) / 2)))
            pilots.append(cyl(BOSS_PILOT, 7.5, (x, sy * y, ztip - 7.5 / 2 + 0.01)))
    m = union(m, *bosses); m = diff(m, *pilots)
    # hr17：电池仓两侧腔（壳内、导轨外，x −43.8..−21.3 × |y| 15..29 × z 120..152）放 UBEC / WAGO / 一分二 —— 导轨外面（|y|=15）长 4 个扎带环
    #        （tail.tie_loops：每侧 x −39 / −33，环外 |y| 18.5，扎带槽 2.2×7；避开导轨绑带槽 x −29..−27）。实物尺寸未量，只给扎带位，不做贴合口袋。
    m = union(m, *tie_loops())
    # 09-17 真实姿态区域刀（lib.region_cut）：头俯仰 ~96° 缩头时原版 jaw 压到轭板前缘（18 姿态，226 mm³，x 30..39 × y 13..16，离颈轴 ≥10.7，不进舵盘孔区）。
    # was_until_2026_09_28_hr50: from .lib import region_cut
    # was_until_2026_09_28_hr50: m = diff(m, region_cut("trunk__pair_jaw_soft_x_trunk_base.stl", "trunk_base"))
    # hr50（用户定区域刀全撤）：T01#7（jaw × 颈轭板前缘）22 姿态全部是 alpha_stand/body_all_max 与 roulade 极限帧（头俯仰顶到 +89..+95°）→ 列进重训约束清单。
    # hr50_r2 2026-09-28：（躯干壳 agent r13）L6 双关节网格上头俯仰 +90°（目标上限）× 颈俯仰 +25..+60 共 8 格 J01×T01 交集 6.7–16.4 mm³、深 ≤1.50
    #   （原版真实动作 ±5.5° 内到过，按"到过的必须不撞"改件），全在轭板前上角 x 35.2..39.0 × z 161.6..165.42 × 整板厚 y 13..16。
    #   头俯仰固定时颈俯仰只让 J01 绕颈轴转 → J01 各点离颈轴半径不变：头 +90° 时扫进轭板的 J01 点最小半径 15.911（颈 13.5..62.5°、外扩 0.3 扫描；
    #   头 +89/+88/+87° 分别 16.78/17.66/18.53，+86° 碰不到），所以刀 = 以颈俯仰轴为轴的正圆柱（256 边）R = JAW_YOKE_R 之外、轴心右上象限内、
    #   只在轭板厚度 y0−0.5..y1+0.5 —— 等于把轭板前上角（角点 r 18.38）削成 R 15.5 的圆角，圆柱与 x=39 / z=165.42 两个板面交在 φ 33° / 57°，
    #   不是尖楔（面夹角 147°）。舵盘 6 孔在 r ≈5.25、盘 Ø26（r 13）都在圆内，不碰。
    jq = wbox((cn[0], y0 - 0.5, cn[2]), (cn[0] + 20.0, y1 + 0.5, cn[2] + 20.0))          # hr50_r2 2026-09-28：颈轴右上象限 × 轭板厚度
    m = diff(m, diff(jq, cyl(2 * JAW_YOKE_R, (y1 - y0) + 3.0, (cn[0], (y0 + y1) / 2, cn[2]), axis="y", sections=256)))
    return keep_main(m, "trunk")

# 2026-09-16 行程恢复（用户定：能力与原版一致）：颈俯仰目标区间 = frozen.yaml:neck_pitch.target_range_deg [−90, 60]（能力包络推导，
# = 上游全程）。这里转的是脖子子树本身（子件在父件系里），角度就是关节角，不取负；两端各多扫一格 2.5°。
# 旧 −60..45 是 09-1x 的工作区，L6 量出的无碰撞区间 [−65, 47.5] 就卡在它上面（docs/reports/行程恢复_扫掠体积分析_2026-09-16.md §4）。
NECK_SWEEP_LO, NECK_SWEEP_HI = -92.5, 62.5
NECK_SWEEP_STEP = 1.25     # hr50：was 2.5（规矩每步 ≤1.5°）

def neck_swing_env():
    """脖子子树在 neck_pitch NECK_SWEEP_LO..NECK_SWEEP_HI（−92.5..62.5）度、head_pitch ±45 度范围内的扫掠包络（只取 z≤175 那段，壳最高才 162），
    外扩 NECK_CLR。用来从躯干壳上挖出脖子的活动空间。"""
    Tn, Tp = TW("neck"), TW("neck_pitch")
    an, on = Tn[:3, 2], Tn[:3, 3]          # neck_pitch 轴 (0,-1,0) @ (26,14.5,152.4)
    ap, op = Tp[:3, 2], Tp[:3, 3]          # head_pitch  轴 (0, 1,0) @ (26,14.5,202.4)
    # 2026-09-16：x 下限 0 → −15。颈 +50..+60° 时 N01 背板尾部摆到 x −9..−1、z 158..161，旧盒把这段扫掠体裁掉 → T03 内皮层留了 0.2 厚的一片
    # （build r1：neck×shell_R 2.47 mm³ @ +55°）。盒只是限制并集范围，放宽不会多挖任何脖子到不了的地方。
    box = wbox((-15, -26, 128), (55, 26, 175))
    # base 用 build_neck(False)（未开惰轮通孔的 N01）：扫掠体与开孔前逐字一致、少算一次开孔件（开孔与否对 T03 无影响，见 neck.build_neck）。
    # 6 颗角螺丝头也要进扫掠体：上排那颗头离 T03 内壁本来只有 0.20（09-12 复核），壳的 y 基准改成"毂贴惰轮"后不能再靠运气。
    # 09-14 复审 R3a MAJOR：原版把 6 颗螺丝头和 N01 本体放在同一个 base 里、一起外扩 NECK_CLR 0.8 → 上排头外面 −16.9+0.8 = −17.7，
    # T03 颈部区皮层（原壳 −18.40）只剩 0.70（≈48 mm²，中位 0.70）。我们改成螺丝头**单独**外扩 NECK_HEAD_CLR 0.5（削面 −17.4、皮层 1.0，
    # 头↔壳扫掠全程 ≥0.5，壳因毂短一层高内移 0.2 后仍 ≥0.3）；N01 本体/两颗舵机/N02 仍按 0.8。两个 Minkowski 体再并起来。
    base = [build_neck(False)] + [placed(s288.servo_mesh(), sfw("neck", i)) for i in (0, 1)] + [build_neck_pitch()]
    heads = neck_screw_heads()
    parts, hparts = [], []
    # was_until_2026_09_28_hr50: for a1 in np.arange(NECK_SWEEP_LO, NECK_SWEEP_HI + 0.001, 2.5):
    # hr50（2026-09-28）：规矩"运动扫掠每步 ≤1.5°"——2.5°/步在 T02 / T03 颈部让位面上留锯齿台阶（clean_check 碎边 n 12 @ T03 (21.1, −10.8, 138.5) 等）→ 1.25°/步。
    for a1 in np.arange(NECK_SWEEP_LO, NECK_SWEEP_HI + 0.001, NECK_SWEEP_STEP):
        R1 = rot(math.radians(a1), an, on)
        for k, mm in enumerate(base):
            mv = mm if k != 3 else placed(mm, rot(math.radians(-45), ap, op))    # 只有 N02（k=3）绕头俯仰转到最低档才可能下探
            c = inter(placed(mv, R1), box)
            if c is not None and not c.is_empty: parts.append(c)
        for h in heads:                                                          # 螺丝头随 N01 走
            c = inter(placed(h, R1), box)
            if c is not None and not c.is_empty: hparts.append(c)
    env = union(*parts); henv = union(*hparts)
    # 六个平移副本并集不是连续膨胀，会留下共面接触。用实心盒做Minkowski和。
    # was_until_2026_09_28_hr50: return union(minkowski_box(env, (-NECK_CLR,)*3, (NECK_CLR,)*3), minkowski_box(henv, (-NECK_HEAD_CLR,)*3, (NECK_HEAD_CLR,)*3))
    # hr50（2026-09-28）：颈俯仰舵机顶端两角（局部 (±W/2, top)，⊕ NECK_CLR 方盒后离颈轴 r = hypot(W/2+0.8, top+0.8) = 14.92）绕颈轴转出的下半圈
    #   是真圆柱面；离散旋转（2.5° 或 1.25°/步）拼成多棱面，与壳底面擦边相交成锯齿台阶（clean_check 碎边 n 12 @ T03 (21.1, −10.8, 138.5)；
    #   1.25° 时换到 T02 镜像位 n 22）→ 补一把光滑正圆柱（256 边，只取颈轴以下半圈、舵机厚度向 y ±(T/2+NECK_CLR)），只多削棱面与圆之间 ≤0.001 的弓形。
    r_arc = math.hypot(S["W"] / 2 + NECK_CLR, S["top"] + NECK_CLR) + 0.005
    ya = S["T"] / 2 + NECK_CLR
    arc = inter(cyl(2 * r_arc, 2 * ya, (on[0], 0.0, on[2]), axis="y", sections=256),
                wbox((on[0] - r_arc - 1, -ya - 0.1, on[2] - r_arc - 1), (on[0] + r_arc + 1, ya + 0.1, on[2])))
    return union(minkowski_box(env, (-NECK_CLR,)*3, (NECK_CLR,)*3), minkowski_box(henv, (-NECK_HEAD_CLR,)*3, (NECK_HEAD_CLR,)*3), arc)

_NSE = []
# 右壳 T03 的颈俯仰惰轮座（舵机局部 x，Rn = drv_self("neck")，局部 x ≡ 世界 y）：
#   x0 = -18.40 原版右壳在颈俯仰轴 r≤7.1 的外表面（一整块平面，36 方位 min=max，复核实测）—— 毂从这里长到惰轮端面 -13.0（旧 -12.85）
#   xw = -16.10 原版右壳内壁（只在 r≤2 是整平面；r 3..8.2 有伸到 -15.10 的辐条和凸缘，全部被 neck_swing_env 削到 XR−PLT−NECK_CLR=-17.1）
#        —— 中心 Ø5 让位孔只开到这里，**不在壳外面开洞**（原版这块是实心的）
#   hd = 15.0  毂径，同 neck.NP_IDL_HUB_D（09-13 分度圆定案 5.25 后由 14.0 放大：Ø2.6 过孔外缘 6.55 到毂边 0.95；Ø14 时只剩 0.45，idler_hub 断言会响）
#   sd = 14.6  壳皮层 (-18.40..-17.1) 里的填孔盘：原版 4 个 XL330 Ø2.2@r6 孔覆盖 r 4.9..7.1，Ø14 毂只到 7.0 时不另填就在外观面上留
#              4 个 0.4~0.7 mm² 的贯穿月牙缝（复核实测）。毂放到 Ø15 后（r 7.5 ≥ 7.3）这片盘被毂完全盖住、几何上冗余，保留是为了
#              mechanical_audit 的"皮层 r≤7.3 只有 6 个螺丝孔"不变量仍按同一半径量；Ø14.6 外缘 r=7.3 仍在原版平面凹坑（r≤7.15）+ 壳料里
SHR_IDL = (-18.40, -16.10, 15.0, 14.6)
# 两侧壳给髋偏航舵机背面惰轮盘的让位刀直径 = rear_boss_d + 2·CLR + 这个：lib.cyl 是 64 边**内接**，Ø14.6 的平边只到 r 7.291、对 Ø14 盘实测 min_gap 0.291 < 0.3
# （09-14 第 1 次 r1 build），+0.2 → Ø14.8 平边 7.391、min_gap ≈0.39。筋径向厚 2.9，削 0.42 无外观影响
HIP_IDLER_RELIEF_EXTRA = 0.2
# 左壳 T02 颈俯仰轭区开口（09-14 复审 MAJOR M1，替换 r1 的 6×Ø4.6 头孔 + 中心销刀）。原版 left_shell 在这里是当 XL330 舵盘用的一块**凸出的圆 lobe**：
# 绕颈俯仰轴 r≈8.0..8.42（+x/上/下三面都是自由边，只有 −x 侧接壳体；上缘 z≈160.7 就是脖子开口的边），壁 y 16.5..17.9（1.4 厚，内侧被 T01 轭 ±0.5 让位削掉）。
# 6 个 Ø4.6 头孔 @ r 5.25 打在这块 lobe 上：孔外缘 r 7.55 → 相邻孔壁 0.65、孔边到 lobe 边 0.86..0.90、中心 Ø5.9 岛只靠 4 条 0.65×1.4 的 web 挂着（全 <1.2）。
# 复审 hint 的 Ø15.1 通口 + 向 +x hull：实测 lobe 上缘/下缘也各剩 0.4..0.9 的弧带（截面 y=17 逐行量），所以把 lobe **整个**开掉：
#   刀 = [圆 Ø2·SHL_YOKE_OPEN_R（= lobe 外径 8.42 + 0.4）∩ 半空间 x ≥ SHL_YOKE_OPEN_X0] ∪ 方块 (x SHL_YOKE_OPEN_X0..轴+R, z ≥ 轴心)，沿 y 从轭盘外 15.5 打穿到壳外。
#   x 左界 = 原版 −x 侧 XL330 孔（r 6、Ø2.2）的 +x 边再进 0.1 = 轴 −6.0+1.1−0.1 = 21.0（原版孔与开口连通、不留 0.02..0.08 的薄片；r2 第 1 次 build 实测）：
#   再往左是壳体/前缘条，圆刀伸过去会把 z 145..150 那条 2..3 mm 宽的前缘条削到 <1（预检）。
#   F12 6 颗头的 Ø(m2_head_d+2·CLR) 通道刀（r1 的做法）保留：180° 那颗头 x 18.45..23.05 跨过左界，只靠开口会压进壳 4.57 mm³（r2 第 1 次 build），
#   头孔与开口连通（孔心 20.75 在左界右侧 0.25），孔左缘到壳体前缘条 ≥1.6。中心销刀不再需要（中心在开口里）。
#   剩下的壳：−x 侧壳体 + 前缘条（z<152.4）+ 上缘（z≥159.5，x<21.0，贴在壳体上）；其余 5 颗头/起子/滑入全部在开口里。
SHL_YOKE_LOBE_R = 8.42          # 原版 lobe 外缘离颈俯仰轴的最大半径（orig left_shell，y=16.5..17.9 截面射线实测；上缘 z 160.7 → 8.3）
SHL_YOKE_OPEN_R = SHL_YOKE_LOBE_R + 0.4
SHL_YOKE_XLH = (6.0, 2.2)       # 原版 4 个 XL330 舵盘孔：r=6、Ø2.2（同 T03 SHR_IDL 注释）
_RNK = drv_self("neck")         # 颈俯仰舵机帧（轭轴 (26,0,152.42)，局部 x ≡ 世界 +y）
SHL_YOKE_OPEN_X0 = float(pt(_RNK, 0)[0]) - SHL_YOKE_XLH[0] + SHL_YOKE_XLH[1] / 2 - 0.1    # 21.0：开口的 −x 直边（mechanical_audit review_r2 同用）
SHL_WEDGE_TRIM_X = 23.3        # hr50：T02 开口圆底边 / 脖子让位之间楔尖的竖直切口 x（此处楔厚 0.9）

# ───────────── hr45（2026-09-26）：T02 / T03 颈根开口挂钩（过颈线束按进去卡住，不用扎带）─────────────
# 用户 06:30 定：脖子中段不加东西（原版也是裸线），只在 T02 / T03 颈根各做一个开口挂钩。主设计简报 hr45hook + 07:2x/07:3x 定案（"外偏上 55°"，
# 选型表与全部数见 docs/design_2026-09-17_bearing_rebuild/hr43_work/hr45_挂钩报告.md，探查 / 真切片脚本 hr43_work/hr45/h01..h11）：
#   位置：过颈线束 HB09_R / HB09_L（duckstructure/data/wiring_body_v1.json）躯干段，从 via_NP 起沿站点折线弧长 s 处（R 11.0 / L 14.0）。
#     via_NP 本身在 F12 / F12b 起子 Ø4.2 圆柱里，起子净距 ≥1.0 要 R s≥10.4、L s≥11.7（h09）；R 的线 s≤11.0 才是直的（再往后 13.8 处拐 31°）；
#     L 从 13 挪到 14 把 −t 端口托内沿悬垂的一撮支撑消掉（h11：2.35 → 0.1 mm³），但钩长里 15.08 处线拐 18° → 钩沿线扫掠（见下）。
#   截面（⟂ 线；x = 世界上方，y = 打印向上 = 世界向内：T02 +y 朝下、T03 −y 朝下打印）：
#     腔 Ø = 束径 + 2·clr（clr 0.3，assumed）；腔顶泪滴 roof 50°（尖朝打印上 → 腔内免支撑；选型时 55°，见 WIRE_HOOK_DIM 注）；壁 1.6（assumed）；钩长 6；
#     开口宽 = 束径 − 1.2（assumed：PETG 弹性，按进去不掉），开口轴从世界上方往世界向外转 55°（颈俯仰 −90..+30 主拉力 = 往上前拉，夹角 ≥45°，h11 实测 R 61.1 / L 61.1）；
#     腹板 web：环的世界向内侧（x −ro..2.0，y 0..6.2）连到壳内沿；再对"钩 ∪ 本片壳截面"做 r = fillet 的闭运算 → 根部圆角 ≥1（简报）。
#   一体：钩窗 [s−3, s+3] 内站点折线点做主方向（PCA）得一条直轴，钩 = 一个带开口的截面多边形沿该轴一次拉伸成实体（主设计 08:40：回原稿一体环）；
#     腹板 x −6.0..2.0 / y 0..7.0 嵌进壳唇与内沿 ≥0.2（不做面贴面零厚度接触）；不再做逐片圆角闭运算（build #8 薄片扫掠 + 逐片圆角 → 腔壁 2 条非流形边、碎片 90 个；
#     分段斜接预切 T03 腔内落撑 29–60 mm³，hr45/h18）。R 线在钩窗内离轴 ≤0.10、L（s 13）≤0.29，都在腔隙 0.3 内。
#   次序：钩先并进原壳（build_trunk_shell 开头），再被脖子扫掠 / T01 让位 / F12 头通道等刀照常削（净距按那些刀的口径）；
#     线槽（腔 + 开口）在壳全部加减料之后（keep_main 前）再清一次，保证线槽干净。
#   登记：wire_fixings_v1.json FX_HB09R_HOOK_T02 / FX_HB09L_HOOK_T03（type hook：wire_fixings.solids 不长几何，几何只在这里）；
#     features.yaml T02-FX_HB09R_HOOK_T02 / T03-FX_HB09L_HOOK_T03（kind clip）；wiring_body_v1.json 站点 fix_FX_HB09R_HOOK_T02 / fix_FX_HB09L_HOOK_T03 = 钩腔心。
WIRE_HOOK_DIM = dict(clr=0.3, wall=1.6, length=6.0, open_less=1.2, open_deg=55.0, roof_deg=50.0, roof_trunc=0.0,
                     web=(-6.0, 2.0, 0.0, 7.0), fillet=0.0, dp_tol=0.15, cut_extra=0.5)
# roof 50°（选型时 55°）：T03 腹板顶会被颈扫掠包络削到 ≈6.4，55° 尖高 5.72 只剩 0.68 顶 → 50° 尖 5.10 留 ≥1.3（探针 h19：Ø6.56 泪滴 50°/55°/60° 切片都 0 支撑，45° 要撑）。
# fillet 0 = 不做根部圆角闭运算（见上）；roof_trunc 0 = 不削尖（h18：平顶被撑）。
# 线路站点折线（世界系，零位；wiring_body_v1.json 2026-09-26 07:2x 版，躯干段，trunk_base 随体）：钩心前后各取到拐点外
WIRE_HOOK = {
    1: dict(id="FX_HB09R_HOOK_T02", route="HB09_R", part="T02", s=11.0, bundle_d=5.26,
            poly=[(18.68, 23.5, 157.32), (17.27, 23.5, 158.73), (15.86, 23.5, 160.14), (14.46, 23.46, 161.54), (13.17, 22.5, 162.5)],
            s0=7.831),        # poly[0] 在 via_NP 起的弧长
    -1: dict(id="FX_HB09L_HOOK_T03", route="HB09_L", part="T03", s=13.0, bundle_d=5.96,       # 选型 13；07:4x 挪 14（端口支撑消）后 08:40 回 13（一体直棱柱，钩窗内线不过 18° 拐点）
             poly=[(17.26, -23.5, 158.74), (15.85, -23.5, 160.15), (14.4, -23.5, 161.5), (12.73, -23.5, 162.27), (11.02, -23.5, 162.98)],
             s0=11.105),
}


def _hook_poly_at(side, sv):
    """站点折线上弧长 sv（从 via_NP 起）处的点；超出两端按首 / 末段外推"""
    H = WIRE_HOOK[side]; P = np.array(H["poly"], float)
    seg = np.linalg.norm(np.diff(P, axis=0), axis=1); cum = np.r_[0.0, np.cumsum(seg)] + H["s0"]
    k = max(0, min(int(np.searchsorted(cum, sv) - 1), len(seg) - 1)); f = (sv - cum[k]) / seg[k]
    return P[k] + f * (P[k + 1] - P[k]), P, cum


def _dp(pts, tol):
    """Douglas–Peucker 折线简化（容差 tol，mm）"""
    if len(pts) <= 2: return list(pts)
    a, b = pts[0], pts[-1]; ab = b - a; L2 = float(ab @ ab)
    d = [float(np.linalg.norm(p - (a + max(0.0, min(1.0, float((p - a) @ ab) / L2)) * ab))) for p in pts[1:-1]]
    i = int(np.argmax(d)) + 1
    if d[i - 1] <= tol: return [a, b]
    return _dp(pts[:i + 1], tol)[:-1] + _dp(pts[i:], tol)


def _hook_segments(side, extra=0.0):
    """钩轴分段 [[a, b], …]（世界系）与钩心 c：窗口 [s − L/2, s + L/2] 内站点折线 DP 简化；
    extra > 0：首段起点 / 末段终点各沿本段方向外伸 extra（线槽刀用，分段与钩体相同）"""
    H, D = WIRE_HOOK[side], WIRE_HOOK_DIM
    lo, hi = H["s"] - D["length"] / 2, H["s"] + D["length"] / 2
    c, P, cum = _hook_poly_at(side, H["s"])
    pts = [_hook_poly_at(side, lo)[0]] + [P[k] for k in range(len(P)) if lo + 1e-6 < cum[k] < hi - 1e-6] + [_hook_poly_at(side, hi)[0]]
    Q = _dp([np.asarray(p, float) for p in pts], D["dp_tol"])
    segs = [[Q[i].copy(), Q[i + 1].copy()] for i in range(len(Q) - 1)]
    if extra > 0:
        t0 = segs[0][1] - segs[0][0]; t0 = t0 / np.linalg.norm(t0); segs[0][0] = segs[0][0] - extra * t0
        t1 = segs[-1][1] - segs[-1][0]; t1 = t1 / np.linalg.norm(t1); segs[-1][1] = segs[-1][1] + extra * t1
    return segs, c


def _hook_line(side):
    """钩轴直线：钩窗 [s − L/2, s + L/2] 内站点折线等距取 61 点做主方向（PCA），过这些点的质心。
    返回 (腔心 c = 折线上 s 处的点在该直线上的投影, 单位切向 t（沿线路前进方向）, 钩窗内折线离轴最大距离)"""
    H, D = WIRE_HOOK[side], WIRE_HOOK_DIM
    c0 = _hook_poly_at(side, H["s"])[0]
    Q = np.array([_hook_poly_at(side, v)[0] for v in np.linspace(H["s"] - D["length"] / 2, H["s"] + D["length"] / 2, 61)])
    m0 = Q.mean(0); t = np.linalg.svd(Q - m0)[2][0]
    if float(t @ (Q[-1] - Q[0])) < 0: t = -t
    R = Q - m0; dev = float(np.max(np.linalg.norm(R - np.outer(R @ t, t), axis=1)))
    return m0 + float((c0 - m0) @ t) * t, t, dev


def _hook_axis(side, step=0.25):
    """（核验脚本用）钩轴按 step 取点：(点 N×3, 相对腔心的沿轴位置 N, 腔心)，范围 = 线槽刀（钩长 ± cut_extra）"""
    c, t, _dev = _hook_line(side); half = WIRE_HOOK_DIM["length"] / 2 + WIRE_HOOK_DIM["cut_extra"]
    sl = np.arange(-half, half + 1e-9, step)
    return c + np.outer(sl, t), sl, c


def _hook_frame(side, t):
    t = t / np.linalg.norm(t)
    up = np.array([0.0, 0.0, 1.0]) - t[2] * t; up /= np.linalg.norm(up)
    pu = np.array([0.0, -float(side), 0.0]); pu = pu - (pu @ t) * t - (pu @ up) * up; pu /= np.linalg.norm(pu)
    return up, pu, np.cross(up, pu)


def _hook_profiles(side, sec=None):
    """钩截面 (料, 线槽)，shapely，x = 世界上方，y = 打印向上（世界向内）。sec = 本片壳截面（给根部圆角闭运算用）"""
    import shapely.geometry as sg, shapely.ops as so
    H, D = WIRE_HOOK[side], WIRE_HOOK_DIM
    rc = H["bundle_d"] / 2 + D["clr"]; ro = rc + D["wall"]; wo = H["bundle_d"] - D["open_less"]
    q = np.linspace(0.0, 2 * math.pi, 145)[:-1]
    def circ(r): return sg.Polygon([(r * math.cos(a), r * math.sin(a)) for a in q])
    al = math.radians(D["roof_deg"])
    cav = so.unary_union([circ(rc), sg.Polygon([(rc * math.sin(al), rc * math.cos(al)), (0.0, rc / math.cos(al)),
                                                (-rc * math.sin(al), rc * math.cos(al))])])
    if D.get("roof_trunc"):                                                 # 泪滴尖削平（平顶搭桥）
        cav = cav.intersection(sg.box(-2 * rc, -2 * rc, 2 * rc, rc / math.cos(al) - D["roof_trunc"]))
    g = math.radians(D["open_deg"]); a = np.array([math.cos(-g), math.sin(-g)]); n = np.array([-a[1], a[0]])
    slot = sg.Polygon([tuple(-a - wo / 2 * n), tuple((ro + 3) * a - wo / 2 * n), tuple((ro + 3) * a + wo / 2 * n), tuple(-a + wo / 2 * n)])
    air = so.unary_union([cav, slot])
    x0, x1, y0, y1 = D["web"]
    outer = so.unary_union([circ(ro), sg.box(-ro if x0 is None else x0, y0, x1, y1)])
    if sec is not None and not sec.is_empty:
        f = D["fillet"]; near = sg.Point(0, 0).buffer(ro + 3.5, 64)
        closed = so.unary_union([outer, sec]).buffer(f, 32).buffer(-f, 32)
        fill = closed.intersection(near).difference(sec).intersection(outer.buffer(f + 0.5, 32))   # 只留钩根附近的圆角料（壳自身别处的凹角不动）
        outer = so.unary_union([outer, fill])
    solid = outer.difference(air).buffer(0)
    return solid, air


def _sec2d(m, o, t, up, pu):
    """网格 m 在过 o、法向 t 的平面上的截面 → shapely（x = up 分量, y = pu 分量；奇偶填充），只留 ±16 窗口"""
    import shapely.geometry as sg
    try: s = m.section(plane_origin=o, plane_normal=t)
    except Exception: s = None
    if s is None: return sg.Polygon()
    out = sg.Polygon()
    polys = []
    for e in s.discrete:
        E = np.asarray(e); qq = np.c_[(E - o) @ up, (E - o) @ pu]
        if len(qq) >= 3:
            pg = sg.Polygon(qq)
            polys.append(pg if pg.is_valid else pg.buffer(0))
    for pg in sorted(polys, key=lambda g_: -g_.area): out = out.symmetric_difference(pg)
    return out.buffer(0).intersection(sg.box(-16, -16, 16, 16))


def wire_hook(side, sh=None):
    """side 侧壳的颈根挂钩：(加料网格, 线槽刀网格)（世界系）。一个带开口的截面多边形沿钩轴直线一次拉伸（线槽刀两头各多 cut_extra）。
    sh 只在 fillet > 0 时用（钩心截面做根部圆角闭运算）；现 fillet = 0 → 不用。"""
    import trimesh
    if side not in WIRE_HOOK: return None, None
    D = WIRE_HOOK_DIM
    c, t, _dev = _hook_line(side)
    up, pu, ez = _hook_frame(side, t)
    sec = _sec2d(sh, c, t, up, pu) if (sh is not None and D.get("fillet")) else None
    solid, air = _hook_profiles(side, sec)
    T = np.eye(4); T[:3, 0], T[:3, 1], T[:3, 2], T[:3, 3] = up, pu, ez, c
    out = []
    for prof, Lz in ((solid, D["length"]), (air, D["length"] + 2 * D["cut_extra"])):
        geoms = [prof] if prof.geom_type == "Polygon" else [g_ for g_ in prof.geoms if g_.area > 1e-3]
        ms = []
        for g_ in geoms:
            pm = trimesh.creation.extrude_polygon(g_.simplify(0.005), Lz); pm.apply_translation((0, 0, -Lz / 2)); pm.apply_transform(T); ms.append(pm)
        out.append(union(*ms) if len(ms) > 1 else ms[0])
    return out[0], out[1]


def _hook_extra(add, cut):
    """hr46（2026-09-26）：颈根钩的附属实体（L6 "线撞自己固定件" own 桶用）= 钩的实际料 = 加料 − 线槽刀。
    任务书 hr46 A2：diff 出非流形 / 非水密就退回加料。判法 = 按导出口径（写成 STL、float32、读回合并顶点）必须是水密体 —— 钩文件就是这样被读的。
    实测 09-26：T02 diff 在 float32 下 4 个顶点重合 → 7 条非流形边 → 退回加料（353.806 vs diff 353.788 mm³）；T03 diff 水密（343.256 mm³）。"""
    import io
    import trimesh
    d = diff(add, cut)
    r = trimesh.load(io.BytesIO(d.export(file_type="stl")), file_type="stl", process=True)
    return d if (r.is_watertight and r.is_volume) else add


# hr50：壳上两处原版孔重挖（孔心 / 轴向为原版壳网格上孔壁面拟圆，左壳 side=+1、右壳 −1）：
#   "后侧孔"：壳后上角侧壁一个 Ø≈4.9 的原版孔（轴 y，拟圆 R 2.43）→ 左壳 Ø6.0 重切（y 只在壁厚范围内）。
#   右壳同位孔（拟圆 R 2.33，孔沿碎三角到 r ≈3.0（−x 侧、上侧）/ 3.5）：同心 Ø6.0 在 r 2.9 处留两片 4.5 / 4.1 mm² 刀片（r6），同心 Ø7.2 把孔下方
#   z 151..152.7 那条横板削成 62.6 mm² 薄膜（r7）→ 改偏心：孔心上移 0.3、Ø6.6（底仍在 152.08，与左壳 Ø6.0 同高；−x / 上侧盖到 r 3.3）。
#   （"前通道口"——T01 前排不用螺丝上方竖管在壳顶面的开口——试过 Ø6.0 重切 z 149.5..157.5：碎边仍在 r 3.0..3.4 外圈、还多出 13.6 mm² 刀片（r6），撤回，见报告。）
SHELL_REBORE = {1: [("y", (-38.01, 154.91), (26.5, 33.0), 6.0)],
                -1: [("y", (-37.97, 155.08 + 0.3), (-33.0, -26.5), 6.6)]}

def shell_rebores(side):
    out = []
    for ax, (a, b), (t0, t1), d in SHELL_REBORE[side]:
        c = (a, (t0 + t1) / 2, b) if ax == "y" else (a, b, (t0 + t1) / 2)
        out.append(cyl(d, t1 - t0, c, axis=ax, sections=128))
    return out

# hr50_r2 2026-09-28：（躯干壳 agent r13）原版减面壳"前通道口"（T01-F04 前排不用螺丝上方那根竖管 Ø5.05 在壳前上斜面的出口）口沿是减面碎三角，
#   原版同位同数（orig left_shell 碎边 n 449 / 253、right_shell n 763），我们件上 clean_check 碎边 T02 n 459 + 175、T03 n 706（hr50_work/trunk_neck/clean_check_after.json）。
#   口沿碎边都落在一张斜面 ±0.4 内（r12 网格口沿短锐边最小二乘：T02 z = 154.194 − 0.6848(x−13.517) − 0.3709(y−25.538)，法向倾 37.9°；T03 同形镜像值见下），
#   是原版约 1 mm 45° 倒角被减面揉碎 → 按规矩"碎孔重挖成圆孔 / 碎边一刀平面切齐"：沿该斜面法向重锪一个 90° 锥口（正回转体 128 边），
#   地板 = 口沿平面下 MOUTH_CS[1]、地板处半径 MOUTH_CS[0]，锥面 45° 外扩到面上 MOUTH_CS[2]。竖直圆柱口试过（r13/eA）：竖壁与 38° 斜面夹 52° 楔 → 刀片 12.7 mm²，改锥口。
#   不碰颈根 C 钩：刀减去钩加料体（_hk_add）本身，并让开钩内侧面（T02 y 23.46 / T03 y −23.50）以内、x < MOUTH_XH 那块（r13/eB 不让时钩根边上出碎边 n 8 / n 21 + 碎屑）。
SHELL_MOUTH = {1: (13.517, 25.538, 154.194, -0.6848, -0.3709, 23.46),     # hr50_r2 2026-09-28：口轴 (x, y)、口沿拟合平面 z0 + gx(x−ax) + gy(y−ay)、钩内侧面 |y|
               -1: (13.541, -26.028, 154.258, -0.6880, 0.3560, 23.50)}
MOUTH_CS = (3.6, 0.85, 1.0)     # hr50_r2 2026-09-28：锥口：地板处半径、地板在口沿平面下的深度、刀顶在口沿平面上的高度（碎边在平面 −0.70..+0.44 内，锥面 45°）
MOUTH_XH = 14.3                 # hr50_r2 2026-09-28：钩内侧（|y| < 钩内侧面）只在 x ≥ 14.3 切（钩内侧面 x 10.65..13.60，r13/eD 扫 14.3 / 15.0：15.0 留碎边 n 21）
SHR_SWEEP_END_BOX = ((-11.0, -17.45, 158.0), (-9.5, -15.9, 162.0))   # hr50_r2 2026-09-28：T03 颈/线翘扫掠末端内沿整高切齐（见 build_trunk_shell side<0 注释）
T02_YOKE_CORNER_BOX = ((12.0, 15.0, 150.0), (12.5, 16.5, 166.0))       # hr50_r2 2026-09-28：T02 轭板让位角延到 x 12.0，与钩腹板端面齐（见 build_trunk_shell side>0 注释）

def shell_mouth_cut(side, hook_add=None):
    # hr50_r2 2026-09-28：前通道口 90° 锥口刀（见上方 SHELL_MOUTH 注释）；返回世界系刀体，已减掉钩加料体与钩内侧让开块
    import trimesh
    from .s288 import bx
    ax_, ay_, z0, gx, gy, yh = SHELL_MOUTH[side]
    R, d, t = MOUTH_CS
    n = np.array([-gx, -gy, 1.0]); n /= np.linalg.norm(n)
    cone = trimesh.creation.revolve(np.array([[0.0, -d], [R, -d], [R + d + t, t], [0.0, t]]), sections=128)
    T = trimesh.geometry.align_vectors([0, 0, 1], n); T[:3, 3] = (ax_, ay_, z0); cone.apply_transform(T)
    keep_off = bx((MOUTH_XH - (ax_ - 30.0), 30.0, 60.0), (((ax_ - 30.0) + MOUTH_XH) / 2, side * (yh - 15.0), z0))   # 钩内侧、x < MOUTH_XH
    c = diff(cone, keep_off)
    return diff(c, hook_add) if hook_add is not None else c

def build_trunk_shell(side, trunk=None):
    """躯干壳（原版 left/right_shell 外形原样）：
      - 挖掉脖子在整个俯仰范围内的活动空间（含 N01 的 6 颗角螺丝头）
      - 挖掉我们的脖子轭/颈俯仰舵盘板占的位置
      - **右壳恢复原版的颈俯仰惰轮座**（2026-09-12）：原版 right_shell 的内凸台就是抓 XL330 后端惰轮的
        （2026-09-11 用户拿实物确认 S288 背面那个盘是自由转动的副轴；手册尺寸图 p2 两端接口逐字相同）。
        旧注释"S288 没有对应特征，对我们没用"是错的，那一版把颈俯仰做成了单端悬臂。
        做法：在**减完扫掠体和 T01 让位之后**再 union 一根 Ø15.0 的同心毂（从原版外表面 -18.40 伸到惰轮端面 -13.0；09-13 由 Ø14 放大）
        + 一片 Ø14.6 的皮层填孔盘，再打 6 个 Ø2.6 + 中心 Ø5（只到原版内壁）。顺序不能颠倒：扫掠体在轴心一带一直盖到 -17.1，
        先 union 会被整根削光（复核实测：错序后探针体积 0）。mechanical_audit 里有不变量盯着这个顺序。
        配套：N01 背板开 Ø16 通孔（neck.NK_IDLER_BORE）；T01 让出壳沿 y 直线装入的扫掠（lib.shell_cutter / SHELL_SLIDE）。
        螺丝：6×M2×8（壳壁 2.30 + 毂 3.10 = 5.40，进惰轮 2.6；长度规则 8 ≤ 5.4+3.0−0.3=8.1 ✓；旧 12.85 惰轮面时 5.55/2.45；咬入可接受性按塑料自攻桶判 unknown）。
        装配顺序（写进 assembly.yaml）：脖子总成装到 T01 → T02 → T03 沿 +y 直线推上，毂穿 N01 孔落到惰轮 →
        **先手紧惰轮 6 颗（让毂找到惰轮的位置）再紧壳柱 2 颗**；首装只上 3 颗（0/120/240），留 3 个处女孔给下次拆装；
        推壳之前先在一个孔里旋进一颗 M2 让尖端露出毂端面 ~1 mm 当相位销（推上后惰轮被毂盖死、看不见也拨不动）；
        上电前手转颈俯仰全行程确认无额外阻力（过约束验收项）。
      - **两侧壳各给同侧髋偏航舵机的背面惰轮盘让位**（09-14 复审 MAJOR）：原版左壳在髋偏航轴 r 6.97 处有一条内筋（世界 (10.7,22.65,143.05)，
        径向厚 2.9），手册副轴 Ø14×3 后与惰轮盘实体相交 0.019 mm³（旧 Ø13.8 时余 0.068；右壳同位 r 7.339 ✓）。惰轮自由转动、壳是载体侧静止件，
        按 idlercheck 判据①属拖磨风险。减一刀 Ø(rear_boss_d+2·CLR)=Ø14.6 × (薄段背 −10.0 → 惰轮面 −13.0−CLR=−13.3)，只削筋内缘 0.33，无外观影响。
        build 的 run_checks 只记 >0.05 mm³，抓不到这种量级 —— mechanical_audit 另加 min_gap ≥0.3 的不变量。
      - **左壳在 T01 轭区开口**（09-14 复审 BLOCKER→r1 6×Ø4.6 头孔→r2 MAJOR M1 改整块开掉，见 SHL_YOKE_* 注释）：+y 端 T01 的轭当颈俯仰舵盘，F12 在壳装上之前拧好、
        头 Ø4×1.6 露在轭盘外表面 y 16.0..17.6；原版左壳在轭足印里是它自己当舵盘的那块圆 lobe（壁 y 16.5..17.9），6 颗头 vs 壳 10.7..16.0 mm³/颗，壳合不上。
        r1 打 6 个 Ø4.6 孔后 lobe 只剩 0.65 的 web 和 0.86 的弧带（<1.2），r2 把 lobe 整个开掉（刀 = 圆 ∩ x≥21.1 ∪ 上方方块），
        F12 头/起子/滑入路径与 T01 轭孔里的"销"（DILATE6 让位在孔内腔留下的 Ø1.4/Ø4 料）都随之消失。
      - 加 2 个 Ø5 安装凸台，落到顶板顶面 z=147.8"""
    sh = shell(side)
    _hk_add, _hk_cut = wire_hook(side, sh)                                  # hr45：颈根开口挂钩（见 WIRE_HOOK 注释），先并进原壳，下面的让位刀照常削它
    if _hk_add is not None:                                                 # hr46：钩单独登记成附属实体 → hooks/T02|T03_<锚点 id>.stl（见 _hook_extra）
        produce_extra("shell_L" if side > 0 else "shell_R", WIRE_HOOK[side]["id"], _hook_extra(_hk_add, _hk_cut))
    if _hk_add is not None: sh = union(sh, _hk_add)
    if not _NSE: _NSE.append(neck_swing_env())
    sh = diff(sh, _NSE[0])
    Rh = sfw("trunk_base", 0 if side > 0 else 1)                           # 同侧髋偏航舵机：背面惰轮盘 Ø14×3 在局部 x −13.0..−10.0（世界 z 141.5..144.5）
    sh = diff(sh, placed(cyl(S["rear_boss_d"] + 2 * CLR + HIP_IDLER_RELIEF_EXTRA, S["rear_boss_h"] + CLR,
                             (-S["T"] / 2 - (S["rear_boss_h"] + CLR) / 2, 0, 0), axis="x"), Rh))
    if trunk is not None:                       # 只在颈部区(x≥11)让位；侧壁区反过来是躯干让壳（shell_cutter）
        zone = wbox((11.0, -26, 138), (60, 26, 175))
        for dv in DILATE6(SHELL_CLR):
            c = trunk.copy(); c.apply_translation(dv)
            it = inter(c, zone)
            if it is not None and not it.is_empty: sh = diff(sh, it)
    if side > 0:                                # +y 端：T01 的轭当颈俯仰舵盘 → 壳上原版的 lobe 整块开掉（09-14 r2 M1，见 SHL_YOKE_* 注释）
        Rn = drv_self("neck"); cn = pt(Rn, 0)                                    # 轭轴 (26, 0, 152.42)，局部 x ≡ 世界 +y
        assert abs(Rn[1, 0] - 1.0) < 1e-9, "颈俯仰舵机局部 x 应与世界 +y 同向"
        _, _, xe, _ = driven(ring=RINGS["neck_pitch"], d=26.0)                  # 16.0：轭盘外表面 = F12 头的坐面
        y0, y1 = cn[1] + xe - 0.5, cn[1] + xe + 12.0                             # 沿 y 从轭盘外 15.5 打穿壳壁（17.9）到 28（那一带 y>18 无壳料，z=159..165 截面实测）
        r_open, x_left = SHL_YOKE_OPEN_R, SHL_YOKE_OPEN_X0
        assert r_open >= S["horn_r"] + P["m2_head_d"] / 2 + CLR, "开口必须盖住 6 颗头"
        circ = inter(cyl(2 * r_open, y1 - y0, (cn[0], (y0 + y1) / 2, cn[2]), axis="y", sections=128),
                     wbox((x_left, y0 - 1, cn[2] - 40), (cn[0] + r_open + 1, y1 + 1, cn[2] + 40)))
        top = wbox((x_left, y0, cn[2]), (cn[0] + r_open, y1, cn[2] + 30.0))
        heads = placed(union(*[cyl(P["m2_head_d"] + 2 * CLR, y1 - y0, (xe - 0.5 + (y1 - y0) / 2, S["horn_r"] * math.cos(a), S["horn_r"] * math.sin(a)), axis="x")
                               for a in np.linspace(0, 2 * math.pi, S["horn_n"], endpoint=False)]), Rn)   # 6 颗头通道（180° 那颗跨过左界）
        sh = diff(sh, union(circ, top, heads))
        # hr50（2026-09-28）：开口圆（r 8.82）底边与脖子扫掠让位下沿在 x 21..24.4、y 15.9..17.8 夹出一个楔，尖端 z≈143.65 处 <0.45
        #   （clean_check 刀片 2.98 mm² @ (24.4, 16.7, 143.6)）→ 楔在 x 23.3 处厚 0.9，竖直一刀把尖切掉。
        sh = diff(sh, wbox((SHL_WEDGE_TRIM_X, 15.4, 142.5), (25.5, 18.2, 144.6)))
    if trunk is not None and side > 0:
        # hr50：T01 轭板（x 13..39、y 13..16）的 −x/+y 竖棱在 DILATE6 六向平移让位里缺一条 0.5×0.5 的斜角棱（阶梯状），与壳面相交出碎边
        #   （clean_check 碎边 n 8 @ (12.7, 16.4, 159.6)）→ 把这条棱补切成方角（与六向让位一致的 0.5 间隙，只多切 0.5×0.5 的角）。
        sh = diff(sh, wbox((13.0 - SHELL_CLR, 16.0, BZR0 - SHELL_CLR), (13.0, 16.0 + SHELL_CLR, 152.42 + 13 + SHELL_CLR)))
        # hr50_r2 2026-09-28：颈根钩腹板端面在 y 16.436，轭板让位面在 y 16.5（x≥12.5 已削到 16.5）→ x=12.5 处 0.064 高的台阶出碎边 n 7 @ (12.37, 16.49, 159.49)；
        #   让位角往 −x 延到 x 12.0（主设计 12:3x 批准：只削钩腹板端面 0.064 厚 / 0.128 mm³，钩指/钩腔/朝向不动；r13/xC 试切 T02 三项 PASS）。
        sh = diff(sh, wbox(*T02_YOKE_CORNER_BOX))
    if side < 0:                                # 惰轮在 -y 那一端
        Rn = drv_self("neck")
        x0, xw, hd, sd = SHR_IDL
        xs = XR - PLT - NECK_CLR                                                          # -17.1：扫掠体削出的壳内壁（旧 -16.95）
        skin = placed(cyl(sd, xs - x0, ((x0 + xs) / 2, 0, 0), axis="x"), Rn)              # 皮层填孔盘 -18.40..-17.1
        sh = diff(union(sh, idler_hub(Rn, x0, hd), skin), idler_cut(Rn, center_x0=xw))
        # 09-21 线翘 4 mm：N01 背板在颈俯仰 +y 口 / 头俯仰两口开通窗后，线翘区（lib.conn_zone "A"，舵机局部 x −13..−17.5）伸到壳内壁 −17.1 里 0.3；
        # 把这三块区绕颈俯仰轴 NECK_SWEEP_LO..HI 扫掠（同 neck_swing_env 的范围）从壳上减掉 → 内壁那一带再浅挖 0.3。
        # 颈俯仰 −y 口不开（T03 在 +35° 起 7.1 mm³ / T01 在 +60° 0.9 mm³ 扫过），不在这里让。
        # hr24：grow 0（1°/步，r≤18 处弦高 <0.001）：壳外表面在这一带是 −18.40 的平面，线翘区上沿 −17.4 → 壁 1.0（≥0.9 两圈）；hr23 grow 0.3 挖到 −17.7/−17.8
        #        壁只剩 0.6–0.7（射线中位 0.90、44% <0.8）。线翘 4.0 的名义顶 −17.0 离壳 0.4，这 0.4 全是 wire_clr。
        n_sw = int(round(NECK_SWEEP_HI - NECK_SWEEP_LO)) + 1
        sh = diff(sh, conn_hump_sweep(sfw("neck", 0), (1,), "neck", NECK_SWEEP_LO, NECK_SWEEP_HI, n_sw, grow=0.0),
                      conn_hump_sweep(sfw("neck", 1), (1, -1), "neck", NECK_SWEEP_LO, NECK_SWEEP_HI, n_sw, grow=0.0))
        # hr50_r2 2026-09-28：（躯干壳 agent r13）颈扫掠（壳内壁 −17.1）与线翘扫掠（−17.4）在颈 +62.5° 末端收在 x −11.0..−9.6 的壳顶板内沿，
        #   留下 0.07 厚的唇（z 159.05..159.12）+ −17.1/−17.4 两级台阶（clean_check 碎边 n 8 @ (−10.84, −16.74, 159.44)，T02 同位没有：线翘扫掠只在 T03）
        #   → 一把世界方盒把这段内沿整高切到 −17.45（比两把扫掠都深，只多削不少削），唇和台阶一起没了；离壳柱 (−17, −20) 孔 ≥4 mm。
        sh = diff(sh, wbox(*SHR_SWEEP_END_BOX))
    # hr39f：总开关 collar/孔（hr17–hr38 的 SWITCH_*）整套删除，T02 这块回到原壳（壳顶 152 斜到 159）。
    # hr39f：开 B03 电池顶盖口（tail.LID_CUT：x 壳后缘外..−25.0、|y|≤15、z≥148），壳条（x −25.0..−20.9）下面加承接前舌片的台阶 + 凸棱。
    sh = union(diff(sh, LID_CUT()), lid_strip_pad(side))
    for (x, y) in SHELL_BOSS:                      # 2 个 Ø2.4 过孔，对着 T01 的安装柱，M2×8 从外面拧下去
        yy = side * y
        z_in, z_out = shell_inner_z(side, x, yy)
        assert z_out - z_in > 1.8, f"壳孔 ({x},{yy}) 处壁太薄 {z_out - z_in:.1f}"
        sh = diff(sh, cyl(2.4, (z_out - z_in) + 6.0, (x, yy, (z_in + z_out) / 2)))
        # 09-15 锪平（原版这里是弧面，螺丝头坐不平——09-13 探针 T02 (−17,20) 落差 0.19、(1,24) 0.63；Ø5 足印实测 0.28/0.97）：
        # Ø SHELL_SEAT_D 口袋从平台 z 往上切穿外表面，平台 = 头足印最低外表面 − 0.1（lib.shell_seat_z），切后口袋内最薄壁 ≥1.2（实测 1.29..1.87，不加垫台）。
        # M2×8 因此在 (1,±24) 多进柱 ~0.7（穿壳 1.4..1.5 + 进柱 ≤6.6 < 底孔 7.5）。
        z_plat, wall_min = shell_seat_z(side, x, yy)
        assert wall_min >= SHELL_SEAT_MIN_WALL - 1e-6, f"壳柱 ({x},{yy}) 锪平后最薄壁 {wall_min:.2f} < {SHELL_SEAT_MIN_WALL}"
        sh = diff(sh, cyl(SHELL_SEAT_D, 6.0, (x, yy, z_plat + 3.0)))
    # hr50（2026-09-28）：原版壳网格是 onshape-to-robot 减面过的（20970 面），后上角原版孔的孔沿是碎三角 / 锯齿（clean_check 碎边 n 438–460 + 尖刺，
    #   原版同位同数，继承来的）→ 按规矩"碎孔重挖成干净圆孔"：同轴正圆柱（128 边）把孔沿重切一遍，见 SHELL_REBORE。
    sh = diff(sh, *shell_rebores(side))
    sh = diff(sh, shell_mouth_cut(side, _hk_add))                          # hr50_r2 2026-09-28：前通道口口沿碎边重锪 90° 锥口（见 SHELL_MOUTH 注释），钩加料体本身不切
    if _hk_cut is not None: sh = diff(sh, _hk_cut)                          # hr45：线槽（腔 + 开口）在全部加减料之后再清一次
    return keep_main(sh, "trunk_shell")
