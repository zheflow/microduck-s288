"""件号标记（hr15，2026-09-19 用户要求"每个组件来个浅字编号"）。
每个打印件在一个外表面刻 0.3–0.4 mm 深的凹字（凹字任何打印朝向都不需要支撑；0.4 = 两层）；
放不下 3 个字的小回转件改"点码"：Ø1.2×0.3 深凹点，点数 = 件号末位（L13 三点、N05 五点、N06 六点、N07 七点、N08 八点；
L10 端面太窄且末位是 0，按规则不点——两只套筒里有三点的是 L13）。
位置由 docs/design_2026-09-17_bearing_rebuild/hr15_pick_stamp_faces.py / hr15_pick_dots.py 在 hr14 placed/ 上自动选出
（规则：平面共面组，文字矩形连 2.0 mm 边距（小字 1.0/0.6）落在面多边形内、下方壁厚 ≥1.5（小字 1.2/1.0）、
外侧 0.3/0.8 mm 不在别的实体里（个别小件放宽到配合面：装前可见）、外侧 3..30 mm 空着的比例最高）；这里只放结果。
坐标是世界系零位姿（与 build.py 里 add() 收到的网格一致）。右腿件由切片软件沿 Y 镜像，字随之镜像：**反字 = 右件**。
字号 h = 大写字高；DejaVu Sans Bold 由 matplotlib 自带，任何机器上字形一致。"""
import numpy as np, trimesh
from shapely.geometry import Polygon
from shapely import affinity
from .s288 import diff, union

STAMPS = {
    'L01': dict(kind='text', text='L01', h=4.5, depth=0.4, point=[1.472, 29.8, 102.506], normal=[-0.0, 1.0, 0.0], up=[0.0, 0.0, 1.0]),  # 杯外侧 +y 面 327 mm² 壁 2.1 露出 0.53
    'L02': dict(kind='text', text='L02', h=2.6, depth=0.4, point=[20.151, 38.0, 102.499], normal=[0.0, 1.0, -0.0], up=[1.0, -0.0, 0.0]),  # +y 端面 85 mm² 壁 4.3 露出 1.00
    'L03': dict(kind='text', text='L03', h=4.5, depth=0.4, point=[-31.777, 71.299, 95.66], normal=[0.0, 1.0, 0.0], up=[0.0, 0.0, 1.0]),  # 外侧 +y 面 395 mm² 壁 3.0 露出 1.00
    'L04': dict(kind='text', text='L04', h=3.2, depth=0.4, point=[-31.776, 34.5, 89.918], normal=[0.0, -1.0, 0.0], up=[0.0, 0.0, 1.0]),  # 内侧 −y 面 300 mm² 壁 3.0 露出 0.83
    'L05': dict(kind='text', text='L05', h=4.5, depth=0.4, point=[-18.065, 50.284, 21.533], normal=[1.0, 0.0, 0.0], up=[0.0, 0.0, 1.0]),  # 前面 +x 280 mm² 壁 1.8 露出 1.00
    'L06': dict(kind='text', text='L06', h=4.5, depth=0.4, point=[-24.978, 50.619, 14.7], normal=[0.0, 0.08, -0.997], up=[1.0, -0.0, 0.0]),  # 鞋底底面 907 mm² 壁 2.0 露出 1.00（TPU）
    'L07': dict(kind='text', text='L07', h=2.6, depth=0.4, point=[-20.677, 31.4, 21.119], normal=[0.0, -1.0, -0.0], up=[0.0, 0.0, 1.0]),  # 外侧 −y 面（被 TPU 鞋底包住，装前可见）207 mm² 壁 2.6；原选的 114 mm² 面前方有台阶，刀具会多切
    'L10': dict(kind='none'),  # 套筒端面太窄；末位 0 不点
    'L13': dict(kind='dots', n=3, d=1.2, depth=0.3, normal=[0.0, -1.0, -0.0], centers=[[5.65, 37.3, 109.303], [7.5, 37.3, 108.562], [9.067, 37.3, 107.33]]),  # −y 端面 壁 4.8 露出 0.50 配合面
    'T01': dict(kind='text', text='T01', h=4.5, depth=0.4, point=[-4.292, -0.118, 147.8], normal=[-0.0, -0.0, 1.0], up=[1.0, 0.0, 0.0]),  # 顶面 1923 mm² 壁 3.0 露出 1.00
    'B01': dict(kind='text', text='B01', h=4.5, depth=0.4, point=[-48.8, 0.0, 85.4], normal=[-1.0, 0.0, 0.0], up=[0.0, 1.0, 0.0]),  # 门外面 1178 mm² 壁 3.1 露出 1.00；hr48 门外面 −46.8 → −48.8（lib.DOOR_ADD），同一位置、壁 3.4
    # hr39f B03 电池顶盖：外表面全是原壳外观曲面；两侧切面（y=±14.8，打印贴床面）是卷边截面，最宽处 <6 mm，放不下 2.6 字高 + 2.0 边距 → 照 L10 规则不刻
    'B03': dict(kind='none'),
    'B02': dict(kind='text', text='B02', h=2.6, depth=0.3, point=[-48.3, 0.0, 138.0], normal=[-1.0, 0.0, 0.0], up=[0.0, 0.0, 1.0]),  # hr17 托架背板上延段外面（两凸台之间）壁 1.5 露出 1.2
    'T02': dict(kind='text', text='T02', h=4.5, depth=0.4, point=[-12.943, 31.311, 139.967], normal=[0.0, 0.999, -0.035], up=[0.0, 0.035, 0.999]),  # 壳外面 1228 mm² 壁 2.2 露出 1.00
    'T03': dict(kind='text', text='T03', h=4.5, depth=0.4, point=[-12.941, -31.811, 140.043], normal=[-0.0, -0.999, -0.035], up=[-0.0, -0.035, 0.999]),  # 壳外面 1228 mm² 壁 2.2 露出 1.00
    'N01': dict(kind='text', text='N01', h=4.5, depth=0.4, point=[26.744, -16.3, 175.622], normal=[-0.0, -1.0, 0.0], up=[1.0, 0.0, 0.0]),  # −y 侧面 850 mm² 壁 3.0 露出 1.00
    'N02': dict(kind='text', text='N02', h=2.6, depth=0.3, point=[17.0, -0.1, 216.021], normal=[-1.0, 0.0, 0.0], up=[0.0, 0.0, 1.0]),  # −x 面 240 mm² 壁 1.5 露出 1.00
    'N03': dict(kind='text', text='N03', h=4.5, depth=0.4, point=[26.0, -16.399, 256.915], normal=[0.0, -0.0, 1.0], up=[1.0, 0.0, 0.0]),  # 顶面 480 mm² 壁 3.1 露出 0.95
    'N04': dict(kind='text', text='N04', h=2.6, depth=0.4, point=[44.005, -0.349, 232.265], normal=[1.0, 0.0, 0.0], up=[0.0, 0.0, 1.0]),  # 轴颈销 +x 端面 196 mm² 壁 7.5 露出 0.84 配合面（N07 盖住）
    'N05': dict(kind='dots', n=5, d=1.2, depth=0.3, normal=[1.0, 0.0, 0.0], centers=[[10.1, 3.812, 242.042], [10.1, 1.976, 242.82], [10.1, 0.0, 243.085], [10.1, -1.976, 242.82], [10.1, -3.812, 242.042]]),  # +x 端面 壁 4.6 露出 0.33 配合面
    'N06': dict(kind='dots', n=6, d=1.2, depth=0.3, normal=[-0.0, -0.0, 1.0], centers=[[27.588, -8.641, 227.615], [29.091, -7.323, 227.615], [30.515, -5.919, 227.615], [31.853, -4.432, 227.615], [33.101, -2.87, 227.615], [34.255, -1.237, 227.615]]),  # 顶面 壁 2.5 露出 0.33
    'N07': dict(kind='dots', n=7, d=1.2, depth=0.3, normal=[1.0, 0.0, 0.0], centers=[[47.2, 1.527, 233.703], [47.2, 2.429, 235.426], [47.2, 1.783, 237.259], [47.2, 0.0, 238.035], [47.2, -1.783, 237.259], [47.2, -2.429, 235.426], [47.2, -1.527, 233.703]]),  # 盖外面 壁 3.1 露出 1.00
    'N08': dict(kind='dots', n=8, d=1.2, depth=0.3, normal=[0.0, 0.0, -1.0], centers=[[37.725, -6.691, 218.7], [38.585, -4.886, 218.7], [39.168, -2.975, 218.7], [39.463, -0.999, 218.7], [39.463, 0.999, 218.7], [39.168, 2.975, 218.7], [38.585, 4.886, 218.7], [37.725, 6.691, 218.7]]),  # 唇环底面 壁 1.1（盖只有 1.0 厚，剩 0.7）露出 1.00
    'H01': dict(kind='text', text='H01', h=4.5, depth=0.4, point=[-3.476, -7.311, 249.5], normal=[0.0, 0.0, 1.0], up=[1.0, 0.0, 0.0]),  # 顶面 822 mm² 壁 3.2 露出 0.99
    'H02': dict(kind='text', text='H02', h=3.2, depth=0.4, point=[-21.1, 7.752, 235.621], normal=[-1.0, 0.0, 0.0], up=[0.0, 1.0, 0.0]),  # −x 面 129 mm² 壁 3.0 露出 0.99
    'H03': dict(kind='text', text='H03', h=4.5, depth=0.4, point=[-12.237, 27.866, 221.217], normal=[0.009, -0.009, 1.0], up=[0.0, 1.0, 0.009]),  # 底壳顶缘 506 mm² 壁 1.8 露出 1.00；hr39c：跟放大后的地板走（head_scale.move，原 [-5.391, 22.873, 220.987]）
    'H04': dict(kind='text', text='H04', h=3.2, depth=0.4, point=[92.17, -34.104, 244.75], normal=[-1.0, 0.0, 0.0], up=[0.0, 0.0, 1.0]),  # hr39c：跟放大后的脸板内面走（原 [80.1, -28.0, 240.0]）  # hr37 脸板内面（−x，头里）y −32..−24 × z 238..242：避开摄像头 PCB |y|≤16 与麦克风 y 24..34；板 1.3 ≥ 小字 1.2
    # hr38 两个新件暂不刻（照 L10 的"放不下就不刻"规则，记在 hr38_待办交接.md §4）：
    #   H05 上头壳：外表面全是外观曲面（原版形状不改），内面被喇叭/功放卡座 + 两条原版横筋占满，没有 ≥2.6 字高 + 边距 2.0 的平面共面组；
    #   J01 活动嘴：整件都是外观面（嘴），且 +y 臂已被法兰沉窝占满、−y 臂是 Ø9.95 轴颈。
    #   要刻的话得跑 hr15c_pick_stamps.py 在 hr38 placed 上重选面，本轮没做。
    'H05': dict(kind='none'),
    'J01': dict(kind='none'),
    # hr39c 圆眼装饰三件（duckstructure/eye.py）：整件都是外观面，不刻
    'E01': dict(kind='none'),
    'E02': dict(kind='none'),
    'E03': dict(kind='none'),
    # hr41 新件（点码，点数 = 件号末位；位置按 duckstructure/jaw.py / head.py 常量手算，都是装前可见、装后贴着别的件的内侧面）：
    #   J02：盘内面环带（y 49.55，朝 −y，r 7.5..10 在颈外）r 8.75 @ 52°/68°，离螺母穴（350/130/250°）≥ 60°；
    #   J03：盘内面（y −49.55，朝 +y）r 8.0 @ 44/60/76°，离螺母穴 ≥ 54°、离轴颈 r 4.975 ≥ 2.4；
    #   H06：+y 竖条背面（x 84.5，朝 −x）y 14.25、z 247.94..257.94 每 2.0 一点。
    'J02': dict(kind='dots', n=2, d=1.2, depth=0.3, normal=[0.0, -1.0, 0.0], centers=[[37.137, 49.55, 252.395], [35.028, 49.55, 253.613]]),
    'J03': dict(kind='dots', n=3, d=1.2, depth=0.3, normal=[0.0, 1.0, 0.0], centers=[[37.505, -49.55, 251.057], [35.75, -49.55, 252.428], [33.685, -49.55, 253.262]]),
    'H06': dict(kind='dots', n=6, d=1.2, depth=0.3, normal=[-1.0, 0.0, 0.0], centers=[[84.5, 14.25, 247.94], [84.5, 14.25, 249.94], [84.5, 14.25, 251.94], [84.5, 14.25, 253.94], [84.5, 14.25, 255.94], [84.5, 14.25, 257.94]]),
    # hr43 H07 功放托板：竖板 +y 腿前面（x 68，朝 +x，y 26.5..31.5 × z 237..258.6）小字 2.6 竖排（上 = +y，读向 −z），字心 z 248 离下螺丝孔（239.5 ± 1.2）与上横条（258.6）都 ≥ 5；
    #   板厚 2.0 → 剩 1.7；这面打印时贴床（printability H07 down_world [1,0,0]），字在首层，同 hr15「一半字朝床」已接受口径
    'H07': dict(kind='text', text='H07', h=2.6, depth=0.3, point=[68.0, 29.0, 248.0], normal=[1.0, 0.0, 0.0], up=[0.0, 1.0, 0.0]),
    # hr43d H08 转接板托板：竖板 +y 下腿前面（同 H07 位置 —— 竖板 / 下腿几何照 H07 不变），打印时这面竖着（托面顶朝下），字在侧壁
    'H08': dict(kind='text', text='H08', h=2.6, depth=0.3, point=[68.0, 29.0, 248.0], normal=[1.0, 0.0, 0.0], up=[0.0, 1.0, 0.0]),
    # hr43d H09 功放支架：竖板 −x 面（x −15.6，y 29.5..38 × z 224.5..245）中部，字心 (y 33.75, z 235) 离两个 F36 孔（z 229.5 / 242.1）≥5；板厚 2.0 → 剩 1.7
    # was_until_2026_09_25_hr43e: 'H09': dict(kind='text', text='H09', h=2.6, depth=0.3, point=[-15.6, 33.75, 235.0], normal=[-1.0, 0.0, 0.0], up=[0.0, 0.0, 1.0]),
    # hr43e：H09 竖板 y 29.5..38 → 32.4..38（5.6 宽，躲 L6 区间内 shell_L，head.AMP_BRACKET）；字宽 7.12（features H09-S01 多边形 u ±3.56）放不下横排 →
    #   改竖排：上 = +y、阅读方向 = 上 × 法线 = +z；字心 (y 35.2 = 竖板 y 中心, z 236.2) → 字 z 232.64..239.76 × y 33.9..36.5，离 F36 孔边（z 230.35 / 241.25）2.3 / 1.5
    'H09': dict(kind='text', text='H09', h=2.6, depth=0.3, point=[-15.6, 35.2, 236.2], normal=[-1.0, 0.0, 0.0], up=[0.0, 1.0, 0.0]),
    # hr52 H10 ToF 压条：条宽 2.4 放不下 2.6 字高 + 边距，件号末位是 0 → 照 L10 规则不点（全机只有这一件是斜着的细条，装时认得出）
    'H10': dict(kind='none'),
}
OVER = 1.0          # 刀具向外多伸 1 mm，保证切穿表面
FONT = dict(family="DejaVu Sans", weight="bold")

def text_polys(text, h):
    """字串 → 居中的 shapely 多边形列表（含孔，偶奇填充），大写字高 = h。"""
    from matplotlib.textpath import TextPath
    from matplotlib.font_manager import FontProperties
    tp = TextPath((0, 0), text, size=1.0, prop=FontProperties(**FONT))
    g = None
    for p in tp.to_polygons():
        if len(p) < 3: continue
        q = Polygon(p)
        if not q.is_valid or q.area < 1e-9: continue
        g = q if g is None else g.symmetric_difference(q)
    b = g.bounds; s = h / (b[3] - b[1])
    g = affinity.scale(g, s, s, origin=(0, 0)); b = g.bounds
    g = affinity.translate(g, -(b[0] + b[2]) / 2, -(b[1] + b[3]) / 2)
    return list(g.geoms) if g.geom_type == "MultiPolygon" else [g]

def _frame(normal, up):
    n = np.asarray(normal, float); n = n / np.linalg.norm(n)
    u = np.asarray(up, float); u = u - n * (u @ n); u = u / np.linalg.norm(u)
    r = np.cross(u, n)                       # 从外面看的阅读方向：右 = 上 × 法线
    T = np.eye(4); T[:3, 0], T[:3, 1], T[:3, 2] = r, u, n
    return T, n

def cutter(spec):
    if spec["kind"] == "none": return None, 0.0
    if spec["kind"] == "text":
        T, n = _frame(spec["normal"], spec["up"]); T[:3, 3] = spec["point"]
        ms, area = [], 0.0
        for poly in text_polys(spec["text"], spec["h"]):
            m = trimesh.creation.extrude_polygon(poly, spec["depth"] + OVER)
            m.apply_translation((0, 0, -spec["depth"])); m.apply_transform(T); ms.append(m); area += poly.area
        return union(*ms), area * spec["depth"]
    if spec["kind"] == "dots":
        n = np.asarray(spec["normal"], float); n = n / np.linalg.norm(n)
        ms = []
        for c in spec["centers"]:
            m = trimesh.creation.cylinder(radius=spec["d"] / 2, height=spec["depth"] + OVER, sections=24)
            m.apply_transform(trimesh.geometry.align_vectors([0, 0, 1], n))
            m.apply_translation(np.asarray(c, float) + n * ((OVER - spec["depth"]) / 2)); ms.append(m)
        return union(*ms), spec["n"] * np.pi * (spec["d"] / 2) ** 2 * spec["depth"]
    raise ValueError(spec["kind"])

def stamp(m, pid):
    """在世界系网格 m 上刻件号；位置不在表面上（几何漂移）或去除体积不对就报错，不静默跳过。"""
    spec = STAMPS.get(pid)
    if spec is None: raise KeyError(f"{pid}: stamps.STAMPS 没有这个件")
    if spec["kind"] == "none": return m
    pts = np.array([spec["point"]] if spec["kind"] == "text" else spec["centers"], float)
    _, dist, _ = trimesh.proximity.closest_point(m, pts)
    assert dist.max() < 0.05, f"{pid}: 刻字位置离表面 {dist.max():.3f} mm，件几何变了，重跑 hr15_pick_stamp_faces.py"
    c, expect = cutter(spec)
    out = diff(m, c); removed = float(m.volume - out.volume)
    assert 0.6 * expect < removed < 1.3 * expect, f"{pid}: 刻字去除 {removed:.2f} mm³，预期 {expect:.2f}（面不平或刀具伸进了别处）"
    label = spec["text"] if spec["kind"] == "text" else f"{spec['n']}点"
    print(f"    [{pid}] 刻 {label} 深 {spec['depth']} 去除 {removed:.2f} mm³")
    return out
