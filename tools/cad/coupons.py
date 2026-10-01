#!/usr/bin/env python3
"""S288 公差试件（跟测试台一起打，验证打印机的孔径/配合公差，给整鸭载体设计定数）。
全部平放打印、无支撑。舵机坐标同 s288.py。
  TC01_bearing  6704ZZ 20×27×4 轴承座试件：4 个外圈孔 Ø27.0/27.15/27.3/27.45（4 厚，切角那头是 27.0）
  TC01b_hubs    3 个内圈毂 Ø19.7/19.85/20.0（高 5，切角那头是 19.7）
  TC02_holes    M2 孔规：竖直孔 Ø1.6…2.6 步 0.1（3 厚，切角那头是 1.6）
  TC02b_hholes  横向孔规：站着打，孔轴水平，Ø2.0/2.2/2.4/2.6（横孔打出来比竖孔更小）
  TC03_pocket   舵机下半段卡槽环 ×2：内腔 22.8×20 + 单边 0.2 / 0.4（高 6 闭合环，套在接线端试松紧，先拔线）
  TC04_horn     舵盘板 Ø(flange_d+1)×3：6×Ø horn_hole_d at r=S["horn_r"] + 中心 Ø5（整鸭从动件用的就是这个，直接拧到法兰上试）；另一片孔 Ø2.4
                （已打印的一批是 r4.75/Ø2.2 时出的；09-13 起 S 为 r5.25/Ø2.4/法兰 Ø14，重跑本文件才会变）
  TC05_mount    背面安装板 24×36×3（整鸭载体背板的原型）：贴下半段厚背面，上排两孔下面各一个 Ø5×2.8 垫柱够到薄背面，
                Ø14.5 孔让开副轴，4 孔在官方角孔位 16×30 (±8, +7.5 / −22.5)；一片 Ø2.4 一片 Ø3.0——2.4 全能拧上=孔位对；只有 3.0 能拧=差 0.3~0.5
"""
import os, sys, math
import numpy as np, trimesh
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", ".."))
from duckstructure.s288 import S, cyl, bx, union, diff

OUT = os.path.join(os.path.dirname(__file__), "..", "..", "cad", "coupons_s288"); os.makedirs(OUT, exist_ok=True)

def bearing():
    bores = (27.0, 27.15, 27.3, 27.45)
    pitch = 33.0; w = pitch * len(bores); t = 4.0
    m = bx((w, 34, t), (w / 2, 0, t / 2))
    m = diff(m, *[cyl(d, t + 2, (pitch * (i + 0.5), 0, t / 2)) for i, d in enumerate(bores)])
    return diff(m, bx((4, 4, 10), (0, -17, 0)))              # 切角：这头是 27.0，往右递增

def hubs():
    ds = (19.7, 19.85, 20.0)
    base = bx((26 * len(ds), 26, 2.0), (13 * len(ds), 0, 1.0))
    pegs = [cyl(d, 5.0, (26 * (i + 0.5), 0, 2.0 + 2.5)) for i, d in enumerate(ds)]
    return diff(union(base, *pegs), bx((4, 4, 10), (0, -13, 0)))   # 切角：这头是 19.7

def holes():
    ds = [round(1.6 + 0.1 * i, 1) for i in range(11)]
    pitch = 6.0; w = pitch * len(ds) + 4; t = 3.0
    m = bx((w, 10, t), (w / 2, 0, t / 2))
    m = diff(m, *[cyl(d, t + 2, (2 + pitch * (i + 0.5), 0, t / 2)) for i, d in enumerate(ds)])
    m = diff(m, bx((3, 3, 10), (0, -5, 0)))                 # 切角那头是 1.6
    return m

def hholes():
    ds = (2.0, 2.2, 2.4, 2.6)
    pitch = 7.0; w = pitch * len(ds) + 2; h = 8.0; t = 6.0   # 站着打：长 w 沿 x，高 h 沿 z，厚 t 沿 y；孔沿 y 打穿
    m = bx((w, t, h), (w / 2, 0, h / 2))
    m = diff(m, *[cyl(d, t + 2, (1 + pitch * (i + 0.5), 0, h / 2), axis="y") for i, d in enumerate(ds)])
    m = diff(m, bx((4, 8, 4), (0, 0, h)))                    # 顶角切一刀（2×2）：这头是 2.0
    return m

def pocket(clr, wall=2.5, h=6.0):                          # 高 6：插座离底可能只有 8mm，环不能顶到插座/线
    ix, iy = S["T_lo"] + 2 * clr, S["W"] + 2 * clr
    m = bx((ix + 2 * wall, iy + 2 * wall, h), (0, 0, h / 2))
    m = diff(m, bx((ix, iy, h + 2), (0, 0, h / 2)))
    n = int(round(clr / 0.2))                                # 刻痕数 = 间隙/0.2：1 道=0.2，2 道=0.4
    for i in range(n): m = diff(m, bx((1.0, 3, 3), (-6 + 4 * i, iy / 2 + wall, h)))
    return m

def horn(hole_d):
    t = 3.0; d = S["flange_d"] + 1.0
    m = cyl(d, t, (0, 0, t / 2))
    cuts = [cyl(5.0, t + 2, (0, 0, t / 2))]
    for k in range(S["horn_n"]):
        a = math.radians(k * 360 / S["horn_n"])
        cuts.append(cyl(hole_d, t + 2, (S["horn_r"] * math.cos(a), S["horn_r"] * math.sin(a), t / 2)))
    m = diff(m, *cuts)
    if hole_d > 2.3:                                          # 缺口标记：有缺口 = Ø2.4；放在两孔之间的 30° 方向，离孔 ≥1.4
        m = diff(m, bx((2, 2, 4), (d / 2 * math.cos(math.radians(30)), d / 2 * math.sin(math.radians(30)), t)))
    return m

def mount(hole_d):
    """舵机坐标系里建，最后翻成平放（背板外表面朝下贴床，垫柱朝上）"""
    t = 3.0; W2 = 24.0
    x_in = S["T"] / 2 - S["T_lo"]                      # -13.0 下半段背面（手册；旧实测 -12.85）
    x_thin = -S["T"] / 2                               # -10.0 上半段背面（旧 -9.95）
    plate = bx((t, W2, S["L"] + 2.0), (x_in - t / 2, 0, S["top"] - S["L"] / 2))   # 两端各多 1：角孔离端只有 2，Ø5 垫柱要有地方站
    z_hi, z_lo = S["mnt_z"]
    posts = [cyl(5.0, (x_thin - 0.1) - x_in, (x_in + ((x_thin - 0.1) - x_in) / 2, sy * S["mnt_dx"], z_hi), axis="x") for sy in (1, -1)]
    plate = diff(plate, cyl(14.5, t + 2, (x_in - t / 2, 0, 0), axis="x"))          # 副轴让位 Ø14.5（副轴 13.8；Ø16 会伸到垫柱底下形成悬空）
    m = union(plate, *posts)
    cuts = [cyl(hole_d, 20, (x_in - 2, sy * S["mnt_dx"], z), axis="x") for sy in (1, -1) for z in (z_hi, z_lo)]
    m = diff(m, *cuts)
    if hole_d > 2.7: m = diff(m, bx((t + 2, 3, 3), (x_in - t / 2, 0, S["top"])))     # 顶边缺口（切穿板厚）= Ø3.0 那片
    # 翻成打印姿态：-x（外表面）朝下
    m.apply_transform(trimesh.transformations.rotation_matrix(math.radians(-90), [0, 1, 0]))
    m.apply_translation((0, 0, -m.bounds[0][2]))
    return m

def check(m, name, allow_down=0.0):
    parts = m.split(only_watertight=False)
    print(f"{name}: watertight={m.is_watertight} bodies={len(parts)} vol={m.volume/1000:.1f}cm³ bbox={np.round(m.extents,1)} zmin={m.bounds[0][2]:.2f}")
    assert m.is_watertight and len(parts) == 1 and abs(m.bounds[0][2]) < 1e-6
    down = m.face_normals[:, 2] < -0.75; cen = m.triangles_center[down]; sel = cen[:, 2] > 0.2
    assert m.area_faces[down][sel].sum() < 1.0 + allow_down, f"{name} 有朝下悬空面"

if __name__ == "__main__":
    parts = {"TC01_bearing": bearing(), "TC01b_hubs": hubs(), "TC02_holes": holes(), "TC02b_hholes": hholes(),
             "TC03_pocket_c02": pocket(0.2), "TC03_pocket_c04": pocket(0.4),
             "TC04_horn_22": horn(2.2), "TC04_horn_24": horn(2.4),
             "TC05_mount_24": mount(2.4), "TC05_mount_30": mount(3.0)}
    tot = 0
    for n, m in parts.items():
        check(m, n, allow_down=60 if n == "TC02b_hholes" else 0)   # 横孔顶是 Ø≤2.6 的小桥，正常
        tot += m.volume; m.export(os.path.join(OUT, n + ".stl"))
    # 复核关键孔径：用截面轮廓量
    b = parts["TC01_bearing"]
    sec = b.section(plane_origin=(0, 0, 2), plane_normal=(0, 0, 1)).to_2D()[0]
    ws = sorted(g.bounds[2] - g.bounds[0] for pl in sec.polygons_full for g in pl.interiors)
    assert len(ws) == 4 and all(abs(w - d) < 0.05 for w, d in zip(ws, (27.0, 27.15, 27.3, 27.45))), ws
    h = parts["TC01b_hubs"]
    sec = h.section(plane_origin=(0, 0, 4), plane_normal=(0, 0, 1)).to_2D()[0]
    ws = sorted(pl.bounds[2] - pl.bounds[0] for pl in sec.polygons_full)
    assert len(ws) == 3 and all(abs(w - d) < 0.05 for w, d in zip(ws, (19.7, 19.85, 20.0))), ws
    # 背板 vs 实测台阶舵机：翻回舵机坐标系后不得相交；垫柱顶离薄背面 0.1；4 孔穿过 s288.mount_holes
    from duckstructure import s288
    mp = mount(2.4); mp.apply_translation((0, 0, -mp.bounds[0][2]))
    raw = mp.copy(); raw.apply_transform(trimesh.transformations.rotation_matrix(math.radians(90), [0, 1, 0]))
    raw.apply_translation((S["T"] / 2 - S["T_lo"] - 3.0 - raw.bounds[0][0] - 0.05, 0, 0))   # 外移 0.05 避免共面布尔碎片
    # 斜坡位置没量（step_z=None 时舵机模型整段按厚的算，垫柱必然"相交"）。副轴 Ø13.8 坐在薄背面上 → 薄面至少覆盖 z ±6.9，
    # 垫柱在 z 4.2..9.2 一定落在薄面上；这里按最坏"斜坡从 -6.9 开始、-12 结束"建舵机做检查
    saved = S["step_z"]; S["step_z"] = (-6.9, -12.0)
    servo = s288.servo_mesh(); S["step_z"] = saved
    hit = trimesh.boolean.intersection([raw, servo], engine="manifold").volume
    assert hit < 1e-6, f"背板与舵机相交 {hit:.1f} mm³"
    top_x = raw.bounds[1][0]; assert abs(top_x - (-S["T"] / 2 - 0.15)) < 1e-3, top_x
    pins = s288.mount_holes(30, (S["T"] / 2 - S["T_lo"] - 3, 0, 0))
    assert trimesh.boolean.intersection([raw, pins], engine="manifold").volume < 1e-6, "螺丝孔位不对"
    print("背板复核 OK：不碰舵机、垫柱顶离薄背面 0.1、Ø2.2 销全部穿过 4 孔")
    print(f"轴承孔径复核 OK；合计 {tot/1000:.1f} cm³ ≈ {tot/1000*1.24:.0f} g PLA →", os.path.abspath(OUT))
