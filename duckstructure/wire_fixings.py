"""线束固定点几何（hr44，2026-09-25）—— 协调员 22:58 / 23:30 口径：整鸭只用两种固定点，**不做卡扣线夹 / 独立小件 / 带盖线槽 / 螺丝压板**：
  A 两孔扎带位：现有板 / 壁（厚 1.5..5、板背后净空 ≥1.5）上开两个 Ø2.6 通孔，孔心距 6（两孔连线 ⟂ 束），扎带穿两孔把束勒在板面；不加料。
  B 扎带桥（没法开孔的实心处 / 背后贴件处）：外表面加 8（沿束）× 3（横）× 2.5（高）小桥，桥底横穿 3.2（沿束）× 1.5（高）隧道给扎带过；
    打印时隧道顶跨 3.2 ≤ 4 免支撑。扎带 = 尼龙 2.5 宽 × 1.1 厚 × 100 长（assumed，components zip_tie_2p5）。
锚点数据 = duckstructure/data/wire_fixings_v1.json（hr43_work/hr44/p40_anchors.py 由 wiring_body 线路站点就近件面求出：面点 / 外法向 / 束方向 / 壁厚 / 背后净空 → A 或 B）。
右腿件（L01_R / L02_R / L04_R）由左件镜像得到 → 只在左件上加（数据里右侧锚点与左侧镜像对称，差 ≤0.9 mm，不另建）。
接入：build.py / build_fast.py 在 stamp() 之前调 apply(件号, 世界系网格)。"""
import os, json
import numpy as np

DATA = os.path.join(os.path.dirname(os.path.abspath(__file__)), "data", "wire_fixings_v1.json")
FIX = dict(hole_d=2.6, pitch=6.0, bridge_len=8.0, bridge_w=3.0, bridge_h=2.5, tunnel_len=3.2, tunnel_h=1.5, embed=0.8,
           src="协调员 22:58：A Ø2.6×2 孔距 6；B 8×3×2.5 桥、隧道 3.2×1.5；embed 0.8 = 桥根埋进件面（曲面贴合，assumed）")
MIRRORED = {"L01_R", "L02_R", "L04_R", "L03_R"}


def anchors(pid=None):
    if not os.path.exists(DATA): return []
    A = json.load(open(DATA, encoding="utf-8"))["anchors"]
    return [a for a in A if (pid is None or a["part"] == pid) and a["part"] not in MIRRORED]


def _frame(a):
    n = np.asarray(a["normal"], float); n /= np.linalg.norm(n)
    t = np.asarray(a["t_dir"], float); t = t - np.dot(t, n) * n; t /= np.linalg.norm(t)
    u = np.cross(n, t)
    return n, t, u


def _obox(c, x, y, size):
    from .wiring_head import obox
    return obox(c, x, y, size)


def solids(a):
    """锚点 a 的 (加料实体列表, 去料实体列表)，世界系"""
    import trimesh
    n, t, u = _frame(a); c = np.asarray(a["point"], float)
    if a["type"] == "A":
        L = (a.get("wall") or 3.0) + 2.0
        cuts = []
        for s in (-0.5, 0.5):
            cyl = trimesh.creation.cylinder(radius=FIX["hole_d"] / 2, height=L + 2.0, sections=32)
            z = np.array([0, 0, 1.0]); ax = np.cross(z, n); sn = np.linalg.norm(ax)
            if sn > 1e-9:
                cyl.apply_transform(trimesh.transformations.rotation_matrix(float(np.arctan2(sn, np.dot(z, n))), ax / sn))
            elif np.dot(z, n) < 0:
                cyl.apply_transform(trimesh.transformations.rotation_matrix(np.pi, [1.0, 0, 0]))
            cyl.apply_translation(c + s * FIX["pitch"] * u - n * (L / 2 - 1.0))
            cuts.append(cyl)
        return [], cuts
    if a["type"] == "hook":
        # 非闭合挂钩（hr45，2026-09-26）：几何直接画在件自己的 build 里（neck.py / trunk 壳），锚点只登记位置，不在这里长任何东西。
        # 算法会话 2026-09-26 指出：以前非 A 一律走下面的 B 桥分支，type=hook 会被静默做成扎带桥 → 改成显式分支。
        return [], []
    if a["type"] != "B":   # was_until_2026_09_26_hr45: 没有这一句，非 A 全按 B 桥处理
        raise ValueError(f"wire_fixings 锚点 {a.get('id')} 的 type={a['type']!r} 未知（只认 A / B / hook）")
    h, e = FIX["bridge_h"], FIX["embed"]
    add = [_obox(c + n * (h - e) / 2, t, u, (FIX["bridge_len"], FIX["bridge_w"], h + e))]
    cut = [_obox(c + n * (FIX["tunnel_h"] / 2), t, u, (FIX["tunnel_len"], FIX["bridge_w"] + 2.0, FIX["tunnel_h"]))]
    return add, cut


def apply(pid, m):
    """件号 pid（fname[:3]）的世界系网格 m → 加上本件全部锚点几何"""
    A = anchors(pid)
    if not A: return m
    from .s288 import union, diff
    adds, cuts = [], []
    for a in A:
        ad, ct = solids(a); adds += ad; cuts += ct
    if adds: m = union(m, *adds)
    if cuts: m = diff(m, *cuts)
    return m
