#!/usr/bin/env python3
"""第 1 层 可打印 —— 打印机吐得出来吗。

判据（README 第 2 节 L1）：最薄壁 ≥2 圈 · 最小孔 ≥2×喷嘴 · 悬垂 >55° 的面在支撑可拆区 ·
支撑不落在配合面/孔内 · TPU 单独规则。全部阈值来自 data/printability.yaml，朝向来自
data/parts.yaml:print_orientation_down_normal，孔与禁撑区来自 data/features.yaml。
**本文件里不许出现任何零件专用的坐标或数字。**

两类证据，报告里必须分得清：
  A. 真几何计算（本文件现算，每次跑都重算）
     · 逐层截面 → shapely 形态学开运算 → "放不下 N 圈墙"的层与区域面积、最薄壁 mm
       （**层平面内**的宽度，不是 3D 最小肉厚 —— "放得下 N 圈墙"本来就是逐层 2D 的性质；
        最窄方向沿打印 Z 的肉本层量不到也不判，见 min_wall_geometric 的 criterion）
     · 三角面法向 → 悬垂面积 / 占比 / 最大连续悬垂片
     · 逐层 2D 支撑柱模型 → 支撑接触区、支撑穿过/落进禁撑柱体的体积（只作与 G-code 的交叉核对）、被封闭腔困住的支撑
     · 首层贴床截面积
     · 孔径：从 features.yaml 逐条读名义直径（不从网格找孔 —— 换件只改 data）
  B. 真切片（PrusaSlicer 2.9.6，离线跑，指标固化在 printability.yaml:slice_run）
     · 层数、支撑挤出体积、桥接挤出体积、只有外圈墙没有内圈墙的层数、切片器告警
     · **支撑落点**（09-13 F-L1-3 落地版）：slice_l1.py 把 G-code 里 Support material / interface 挤出段按
       criteria.support_landing_sample_step_mm 采样、逆变换回 export_local、对 no_support_zone_set() 建出的禁撑区
       逐区数点，记进 slice_run.parts.<件>.support_landing（含 zones_sha256）。层按它判：
         support_in_no_support_zone  forbid 级（bearing_bore/journal/spot_face + 滑配走廊配合面壳层）落点 > 允许值 → FAIL(BLOCK)
         support_in_removable_zone   removable 级（螺孔/舵盘孔/沉孔/起子通道/口袋非配合面）落点 → FAIL(WARN) + 后处理文本
         support_model_zone_crosscheck  2D 模型上界=0 却有真落点 → FAIL(WARN)，其余 INFO
       没有记录 / source_sha256 或 zones_sha256 对不上 → unknown（重跑 slice_l1.py）
     · 每次跑核对 slice_run.source_sha256 == 当前 STL 的 sha256，对不上判红（不许用过期切片下结论）

保守方向（重要，决定哪些绿是可信的）：
  · 支撑模型用切片器实参 45°（不是 README 的 55°）—— 多算支撑不少算
  · 桥接跨距没有实测 → **不做任何桥接豁免**，所有无支承区都当成要撑
  两条都是"多算支撑"，所以"支撑没落进禁撑区"这个结论是可信的上界结论；
  反过来"落进去了"则可能被桥接豁免掉，需要桥接实测才能细化 —— criterion 里写明了。
"""
from __future__ import annotations

import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from core import (LayerResult, PASS, FAIL, BLOCK, WARN, INFO, num, ROOT,        # noqa: E402
                  sha256_file)

LAYER = 1
NAME = "可打印"

AX = {"x": (1., 0., 0.), "-x": (-1., 0., 0.), "y": (0., 1., 0.), "-y": (0., -1., 0.),
      "z": (0., 0., 1.), "-z": (0., 0., -1.)}


# ── 小工具 ─────────────────────────────────────────────────────────────────
def _v(field, default=None):
    """{v:…, src:…} → 数值（拿不到给 default）。core.num 只返回值，这里还要 unknown_reason。"""
    val, _ = num(field, default)
    return val


def _why(field) -> str:
    if isinstance(field, dict):
        return str(field.get("unknown_reason") or field.get("src_note") or "")
    return ""


def _fmt(x, n=3):
    return None if x is None else round(float(x), n)


# ── 朝向与切片 ─────────────────────────────────────────────────────────────
def _orient(mesh, down):
    """把件按 down（件自身坐标系里贴床的方向）转成 -z 朝下，并把 z_min 落到 0。
    返回 (转好的网格, 3×3 旋转)。旋转必须是带符号的置换阵 —— 否则禁撑柱体的
    轴会变成斜的，2D 截面就不再是圆/矩形，下面的解析截面会失真。"""
    import numpy as np, trimesh
    d = np.asarray(down, dtype=float)
    n = float(np.linalg.norm(d))
    if not np.isfinite(n) or n < 1e-9:
        return None, None
    M = trimesh.geometry.align_vectors(d / n, [0, 0, -1])
    R = M[:3, :3]
    if not np.all(np.isclose(np.abs(R), np.round(np.abs(R)), atol=1e-6)):
        return None, None
    m = mesh.copy()
    m.apply_transform(M)
    m.apply_translation([0, 0, -float(m.bounds[0, 2])])
    return m, R


def _sections(mesh, h):
    """逐层截面 → shapely 多边形（2D 坐标就是件的 x,y —— mesh_multiplane 对
    plane_normal=+z 返回的 to_3D 是单位阵加 z 平移，已核对）。"""
    import numpy as np, trimesh
    from shapely.ops import unary_union
    top = float(mesh.bounds[1, 2])
    zs = np.arange(h / 2.0, top, h)
    if len(zs) == 0:
        return [], []
    secs, _, _ = trimesh.intersections.mesh_multiplane(
        mesh, np.zeros(3), np.array([0., 0., 1.]), zs)
    polys = []
    for seg in secs:
        if seg is None or len(seg) == 0:
            polys.append(None)
            continue
        try:
            pf = list(trimesh.load_path(seg).polygons_full)
        except Exception:
            pf = []
        polys.append(unary_union(pf) if pf else None)
    return list(zs), polys


# ── 判据 1：最薄壁 ─────────────────────────────────────────────────────────
def _thin_regions(poly, r, min_area):
    """开运算（先腐蚀 r 再膨胀 r）掉的区域 = 局部宽度 < 2r 的地方。
    返回 (过滤后的区域数, 过滤后面积, 原始面积)。min_area 用来滤掉凸直角的圆角化
    （每个直角必掉 r²(1−π/4)，那不是薄壁）。"""
    if poly is None or poly.is_empty:
        return 0, 0.0, 0.0
    try:
        op = poly.buffer(-r).buffer(r)
    except Exception:
        return 0, 0.0, 0.0
    if op.is_empty:
        return 1, float(poly.area), float(poly.area)
    lost = poly.difference(op)
    if lost.is_empty:
        return 0, 0.0, 0.0
    comps = list(getattr(lost, "geoms", [lost]))
    big = [c.area for c in comps if c.area >= min_area]
    return len(big), float(sum(big)), float(lost.area)


def _runs(polys, r, min_area):
    """返回 (命中层数, 最长连续命中层数, 最薄那层的 (下标, 面积))。"""
    flags, worst = [], None
    for i, p in enumerate(polys):
        n, a, _ = _thin_regions(p, r, min_area)
        flags.append(bool(n))
        if n and (worst is None or a > worst[1]):
            worst = (i, a)
    run = best = 0
    for f in flags:
        run = run + 1 if f else 0
        best = max(best, run)
    return sum(flags), best, worst


def _min_wall(polys, min_area, min_layers, r_ref, lo=0.02, hi=3.0, iters=11):
    """最薄壁 = 2·r*，r* = 最小的 r，使得**连续 min_layers 层**都出现"足够大"的开运算损失区。
    返回 (最薄壁 mm 或 None, 最后的 r, floored)。hi 都没触发 → (None, hi, False)，读作"比 2·hi 还厚"；
    lo 就已经触发 → (None, lo, True)，读作"比 2·lo 还窄，二分给不出更细的数"。

    **面积门槛按 r² 缩放**（min_area · min(1, r/r_ref)²，r_ref = 判红用的那个半径）：
      · 门槛的物理含义是"凸直角圆角化的噪声底"，而单角损失 = r²(1−π/4) 本身就 ∝ r²
        （printability.yaml:criteria.thin_region_min_area_mm2 的 src_note 自己写了这条）。
        用**不随 r 缩放的固定门槛**去往下二分，等于越往细处走门槛相对噪声越离谱地严：
        夹点越窄、开运算削掉的面积越小，于是先掉到门槛以下、再也进不了统计 ——
        结果 measured 报出来的"最薄壁"**比真实肉厚大**（元规则 9：measured 不许陈述没验证过的事）。
        实测：L01 用固定门槛报 0.759 mm，按 r² 缩放报 0.159 mm，手工量沉窝之间的肉确实是
        0.55（相邻沉窝）/0.15（沉窝与中心过孔）mm —— 固定门槛高报了 5 倍。
      · 缩放**只往下不往上**（min(1, r/r_ref)²）：r ≥ r_ref 时门槛就是 yaml 原值。
        往上也缩会把门槛涨成 r² 而薄区面积在 r 超过半宽后就饱和了，hit(r) 会不再单调，
        二分直接失效（厚壁件会被误判成"比 2·hi 还厚"）。
      · 于是 r = r_ref 时缩放因子恰为 1，**判红那一档的门槛和 yaml 里写的完全一样**，
        且最薄壁 ≥ 2·r_ref 的件（本来就不判红的那些）报出来的数和以前逐位相同：
        本函数只影响 measured，不影响任何 PASS/FAIL（判红走的是 _runs(polys, r_crit, min_area)）。
    """
    def area_at(r):
        if not r_ref or r_ref <= 0:
            return min_area
        return min_area * min(1.0, r / r_ref) ** 2

    def hit(r):
        _, best, _ = _runs(polys, r, area_at(r))
        return best >= min_layers
    if not hit(hi):
        return None, hi, False
    if hit(lo):
        return None, lo, True
    a, b = lo, hi
    for _ in range(iters):
        mid = 0.5 * (a + b)
        if hit(mid):
            b = mid
        else:
            a = mid
    return 2.0 * b, b, False


# ── 判据 3/4：悬垂 + 2D 支撑柱模型 ─────────────────────────────────────────
def _overhang(mesh, thr_deg):
    """三角面法向 → 悬垂面。悬垂角定义见 printability.yaml:criteria。
    返回 (总面积, 总表面积, 最大连续悬垂片面积, 悬垂面数, 总面数)。"""
    import numpy as np
    n = mesh.face_normals
    a = mesh.area_faces
    down = np.clip(-n[:, 2], 0.0, 1.0)
    ang = np.degrees(np.arcsin(down))
    mask = ang > thr_deg
    tot = float(a[mask].sum())
    biggest = 0.0
    if mask.any():
        try:
            from scipy.sparse import coo_matrix
            from scipy.sparse.csgraph import connected_components
            adj = mesh.face_adjacency
            keep = mask[adj[:, 0]] & mask[adj[:, 1]]
            e = adj[keep]
            nf = len(a)
            g = coo_matrix((np.ones(len(e)), (e[:, 0], e[:, 1])), shape=(nf, nf))
            _, lab = connected_components(g, directed=False)
            idx = np.where(mask)[0]
            sums = {}
            for i in idx:
                sums[lab[i]] = sums.get(lab[i], 0.0) + float(a[i])
            biggest = max(sums.values()) if sums else 0.0
        except Exception:
            biggest = float(a[mask].max()) if mask.any() else 0.0
    return tot, float(a.sum()), biggest, int(mask.sum()), int(len(a))


def _filled(poly):
    """把截面的内环填掉 —— 得到"下一层的实体从四面把这块地方围住了没有"的判断底面。"""
    from shapely.geometry import Polygon
    from shapely.ops import unary_union
    gs = [Polygon(g.exterior) for g in getattr(poly, "geoms", [poly])
          if getattr(g, "exterior", None) is not None]
    return unary_union(gs) if gs else None


# ── 支撑柱第二段：自上而下扫（2026-09-25 hr43 L1 事故后单独成函数）────────────────────────────
# 事故：hr43 全链（H01 翻面后第一次过 L1）上界模型逐层扫时 S 的顶点数每层 ×1.12（211 层里 198→73 层从 112 点涨到 70 万点），
# 预取进程吃到 63 GB、串行又吃 36–51 GB / 57 min。涨的是共线冗余节点：竖直墙在相邻层的截面是同一条线、只差浮点尾数，
# S 的边界贴着墙逐层做 S − P，近重合线段每次互相打断出新节点（间距 1e-10…1e-6 mm），下一层再打断 —— 链有多长就滚多少轮。
# 证据与对账：docs/design_2026-09-17_bearing_rebuild/hr42_work/l1_incident/报告.md。
_SWEEP_SLACK_VERTS = 20000          # 守卫余量（顶点数）：只影响"什么时候换求值顺序"，不影响任何结果


def _nverts(g):
    import shapely
    return 0 if (g is None or g.is_empty) else int(shapely.get_num_coordinates(g))


def _support_sweep(polys, unsupported):
    """_support_model 的第二段。定义（逐层写法，下面的循环与 2026-09-25 之前逐字相同）：
        S = ∅；对 i = n−1 … 1：S = S ∪ U_i；contact[i−1] = S ∩ P_{i−1}；S = cross[i−1] = S − P_{i−1}；bed = 最后的 S
    （U_i = unsupported[i]，P_{i−1} = polys[i−1]；P_{i−1} 为 None/空时 S 不减、contact 空；集合为空记 None）。
    返回 (cross, contact, bed)。

    守卫：真几何里 S 的每个顶点都来自某层截面 / 某层无支承区的顶点或它们的交点，S 的顶点数不会超过全部输入的顶点总数；
    超过了（+ 余量 _SWEEP_SLACK_VERTS）只能是浮点节点在逐层滚雪球 —— 这时丢掉逐层结果，改用 _support_sweep_tree
    从头算**同一组集合**（每层 S 只经 O(log² n) 层重叠计算得到）。没触发守卫的件走的就是原来那段代码，结果逐位不变。
    不做任何简化 / 精度截断 / 多边形数限制。"""
    import shapely
    from shapely.ops import unary_union
    n = len(polys)
    cap = sum(_nverts(p) for p in polys) + sum(_nverts(u) for u in unsupported) + _SWEEP_SLACK_VERTS
    cross = [None] * n
    contact = [None] * n
    S = None
    for i in range(n - 1, 0, -1):
        if unsupported[i] is not None:
            S = unsupported[i] if S is None else unary_union([S, unsupported[i]])
        if S is None or S.is_empty:
            continue
        below = polys[i - 1]
        if below is not None and not below.is_empty:
            c = S.intersection(below)
            if not c.is_empty:
                contact[i - 1] = c
            S = S.difference(below)
        cross[i - 1] = S if (S is not None and not S.is_empty) else None
        if shapely.get_num_coordinates(S) > cap:
            cross = contact = S = c = below = None       # 先放掉逐层扫攒下的膨胀几何，再换求值顺序
            return _support_sweep_tree(polys, unsupported)
    return cross, contact, S


def _support_sweep_tree(polys, unsupported):
    """与 _support_sweep 的逐层定义是同一组集合，只是求值顺序不同（守卫触发时才用）。
    把每一步写成 f_t(X) = (X ∪ U_i) − P_{i−1} = (X − B_t) ∪ A_t（B_t = P_{i−1}，A_t = U_i − P_{i−1}；步 t ↔ 层 i = n−1−t），
    两步合成仍是同一形状（集合恒等式，不是近似）：先 (B1, A1) 后 (B2, A2) = (B1 ∪ B2, (A1 − B2) ∪ A2)。
    compose 按二分树合成区间；scan 二分扫出每一步之后的 S（右半的入口 = 左半的合成作用在入口上，不再接着左半的逐层结果往下滚）。
    每层 S 由原始截面经 O(log² n) 层重叠计算得到；contact[i−1] = (上一步的 S ∪ U_i) ∩ P_{i−1}，与逐层写法同一个集合。"""
    from shapely.ops import unary_union
    n = len(polys)
    cross = [None] * n
    contact = [None] * n
    m = n - 1
    if m <= 0:
        return cross, contact, None

    def nz(g):
        return g is not None and not g.is_empty

    def union(a, b):                            # None/空当"没有"；两边都有才做 unary_union
        if not nz(a):
            return b if nz(b) else None
        if not nz(b):
            return a
        return unary_union([a, b])

    def minus(a, b):
        if not nz(a):
            return None
        if not nz(b):
            return a
        r = a.difference(b)
        return None if r.is_empty else r

    B = [polys[n - 2 - t] if nz(polys[n - 2 - t]) else None for t in range(m)]     # P_{i−1}
    U = [unsupported[n - 1 - t] for t in range(m)]                                 # U_i
    memo = {}

    def compose(lo, hi):                        # 第 lo..hi−1 步依次作用后的 (B, A)：X ↦ (X − B) ∪ A
        k = (lo, hi)
        if k not in memo:
            if hi - lo == 1:
                memo[k] = (B[lo], minus(U[lo], B[lo]))
            else:
                mid = (lo + hi) // 2
                b1, a1 = compose(lo, mid)
                b2, a2 = compose(mid, hi)
                memo[k] = (union(b1, b2), union(minus(a1, b2), a2))
        return memo[k]

    X = [None] * m                              # X[t] = 第 t 步之后的 S（= cross[n−2−t]）

    def scan(lo, hi, x_in):                     # x_in = 第 lo 步之前的 S
        if hi - lo == 1:
            T = union(x_in, U[lo])
            if T is None:
                return
            b = B[lo]
            if b is None:
                X[lo] = T
                return
            c = T.intersection(b)
            if not c.is_empty:
                contact[n - 2 - lo] = c
            X[lo] = minus(T, b)
            return
        mid = (lo + hi) // 2
        scan(lo, mid, x_in)
        b, a = compose(lo, mid)
        scan(mid, hi, union(minus(x_in, b), a))

    scan(0, m, None)
    for t in range(m):
        cross[n - 2 - t] = X[t]
    return cross, contact, X[m - 1]


def _support_model(polys, h, overhang_deg, min_w, bridge_exempt, anchor_sides=2):
    """逐层 2D 竖直支撑柱模型。

    自支承步距 step = h·tan(θ)：某层多边形落在下一层多边形外扩 step 之外的部分，就是
    "这一层新出现、下面没东西托着"的无支承区（等价于 3D 里 >θ 的悬垂，在层空间里算）。
    bridge_exempt：
      False → **什么都不豁免**，所有无支承区都要撑 ⇒ 支撑范围的**上界**
      True  → 三种情形当作不需要支撑（每一条都往"少撑"的方向放宽）⇒ 支撑范围的**下界**：
              ① 这块无支承区比一条挤出线 min_w 还窄 —— 由外圈墙自己挑着（overhang perimeter）
              ② 它被下一层实体四面围住（口袋顶，落在 _filled(下一层) 里）→ 能桥
              ③ 它至少踩在 anchor_sides 个互不相连的下一层实体上（两侧有锚点）→ 能桥
              ①放在下界不放在上界：连续斜面每层只探出零点几毫米，到底算不算"要撑"取决于
              切片器的实现细节，把它放进上界会让上界失去"一定不会漏"的意义，放进下界才对。
    真实支撑一定夹在这两者之间，所以：下界都干净 = 可能干净；下界就脏 = 一定脏。

    自上而下扫：无支承区并进柱子集合 S；每下一层，S 里压到实体上的部分是**支撑接触区**
    （支撑疤留在那儿），剩下的继续往下走，最后落到床上。这一段在 _support_sweep 里算
    （2026-09-25 hr43 L1 事故后加了顶点数守卫与分治求值兜底，集合定义一字未改，见该函数）。
    """
    from shapely.ops import unary_union
    step = h * math.tan(math.radians(overhang_deg))
    n = len(polys)
    unsupported = [None] * n
    bridged_area = 0.0
    for i in range(n):
        cur = polys[i]
        if cur is None or cur.is_empty:
            continue
        below = polys[i - 1] if i > 0 else None
        if below is None or below.is_empty:
            unsupported[i] = cur if i > 0 else None      # 第 0 层由床托着
            continue
        try:
            u = cur.difference(below.buffer(step))
        except Exception:
            u = None
        if u is None or u.is_empty:
            continue
        if bridge_exempt and min_w and min_w > 0:
            u = u.buffer(-min_w / 2.0).buffer(min_w / 2.0)
            if u.is_empty:
                continue
        if bridge_exempt:
            F = _filled(below)
            eps = step + (min_w or 0.0) / 2.0
            keep = []
            for c in getattr(u, "geoms", [u]):
                if c.is_empty:
                    continue
                # ① 被下一层实体四面围住（口袋顶）→ 一定能桥
                if F is not None and c.difference(F.buffer(1e-6)).area <= 1e-6 * max(c.area, 1.0):
                    bridged_area += c.area
                    continue
                # ② 两侧（≥ anchor_sides 个互不相连的锚点）都踩在下一层实体上 → 假设跨距无限时能桥
                touch = c.buffer(eps).intersection(below)
                pieces = [g for g in getattr(touch, "geoms", [touch])
                          if not g.is_empty and g.area > (min_w or 0.1) ** 2]
                if len(pieces) >= anchor_sides:
                    bridged_area += c.area
                    continue
                keep.append(c)
            u = unary_union(keep) if keep else None
            if u is None or u.is_empty:
                continue
        unsupported[i] = u

    cross, contact, bed = _support_sweep(polys, unsupported)
    occupied = sum(c.area for c in cross if c is not None) * h
    contact_area = sum(c.area for c in contact if c is not None)
    return {"cross": cross, "contact": contact, "step_mm": step,
            "occupied_mm3": float(occupied), "contact_mm2": float(contact_area),
            "bridged_mm2": float(bridged_area),
            "bed_mm2": float(bed.area) if (bed is not None and not bed.is_empty) else 0.0,
            "unsupported_layers": int(sum(1 for u in unsupported if u is not None)),
            "unsupported_mm2": float(sum(u.area for u in unsupported if u is not None))}


# ── hr42（2026-09-24）：本层几个重计算函数走检查原语缓存（tools/cad/check_cache.py）────────────────────────
# 键 = 本文件（l1_printable.py）整份 sha256 + 函数名 + 全部实参的内容（网格顶点/面字节、逐层截面多边形的 WKB、数值参数）+ 库版本；
# 值 = 该函数的返回值原样（截面多边形 shapely 2 可 pickle）。本文件任何一处改动 → 键全变 → 全部现算（宁可多算）。
# 判据（阈值比较、PASS/FAIL、报数文字）仍在 run() 里每次现判。DUCK_CHECK_CACHE=0 时直接调原函数。
import hashlib as _hl
_L1_SHA = _hl.sha256(Path(__file__).read_bytes()).hexdigest()
_POLYS_DIG = {}                      # id(polys) → (polys, 摘要)：同一个逐层截面列表只算一次 WKB 摘要（持有引用，id 不会被复用）


def _hr42_cc():
    p = str(ROOT / "tools" / "cad")
    if p not in sys.path:
        sys.path.insert(0, p)
    import check_cache
    return check_cache


def _polys_digest(polys):
    e = _POLYS_DIG.get(id(polys))
    if e is not None and e[0] is polys:
        return e[1]
    h = _hl.blake2b(digest_size=16)
    for q in polys:
        b = b"<None>" if q is None else q.wkb
        h.update(len(b).to_bytes(8, "little")); h.update(b)
    d = h.digest()
    if len(_POLYS_DIG) > 64:
        _POLYS_DIG.clear()
    _POLYS_DIG[id(polys)] = (polys, d)
    return d


_L1_PREF = {}                        # hr42：本轮并行"影子跑"算好的结果（键同下）→ (值, 出处)；与持久缓存无关，--no-check-cache 时也用
_L1_REC = [None]                     # 影子跑工作进程里记下本进程算过的 (键 → (值, 出处))；出处 = (是否取自持久缓存, 来源 run 标签)


def _hr42_cached(fn, keyparts):
    import copy, functools, inspect
    sig = inspect.signature(fn)

    @functools.wraps(fn)
    def g(*a, **k):
        CC = _hr42_cc()
        use_cc = CC.enabled()
        if not (use_cc or _L1_PREF or _L1_REC[0] is not None):
            return fn(*a, **k)
        b = sig.bind(*a, **k); b.apply_defaults()
        key = CC.key("l1." + fn.__name__, _L1_SHA, *keyparts(CC, b.arguments))
        if key in _L1_PREF:
            v, prov = _L1_PREF[key]
            CC.note("l1", prov[0], prov[1], key)       # 第四任：按影子跑那边的出处记（取自持久缓存 / 本轮现算），逐格归属才不会把缓存记成现算
            return copy.deepcopy(v)
        prov = (False, None)
        if use_cc:
            hit, v, tag = CC.lookup("l1", key)
            if hit:
                if _L1_REC[0] is not None:                  # 出处原样带回主进程（tag 是之前的 run 号 = 缓存；是本轮 run 号 = 本轮复用）
                    _L1_REC[0][key] = (copy.deepcopy(v), (True, tag))
                return copy.deepcopy(v)
        v = fn(*a, **k)
        if fn.__name__ == "_sections":            # 截面列表要能被后面几个函数按 WKB 认出来：缓存的是这一份
            _polys_digest(v[1])
        if use_cc:
            CC.put("l1", key, copy.deepcopy(v))
        if _L1_REC[0] is not None:
            _L1_REC[0][key] = (copy.deepcopy(v), prov)
        return v
    return g


def _l1_shadow(pids):
    """hr42：进程池工作进程 —— 只对 pids 这几件把整层 run() 照跑一遍（同一 ctx 构造、同一实参），记下几个重函数的结果交回主进程。
    主进程随后照常全量 run()，这些调用在 _L1_PREF 里命中；判据、顺序、证据都由主进程那一遍产生。"""
    import os
    os.environ["L1_SHADOW"] = "1"
    from core import load_data
    from gate import Ctx
    _L1_REC[0] = {}
    try:
        run(Ctx(load_data(), pids))
    except Exception:                                             # noqa: BLE001  影子跑出错只丢速度：主进程照常现算
        pass
    rec, _L1_REC[0] = _L1_REC[0], None
    import check_cache as _CC
    _CC.worker_flush()
    return rec


def _l1_prefetch(ctx):
    """按 L1_JOBS（默认 4）个进程并行影子跑；L1_JOBS=1、已在影子进程里、或只有 1 件时不做。件按 STL 大小从大到小轮流分给各进程。"""
    import os
    if os.environ.get("L1_SHADOW") == "1":
        return
    jobs = int(os.environ.get("L1_JOBS", "4")) if os.environ.get("L1_JOBS", "4").isdigit() else 4
    pids = [pid for pid in ctx.parts if not (ctx.only and pid not in ctx.only) and ctx.stl(pid) is not None]
    if jobs <= 1 or len(pids) <= 1:
        return
    pids.sort(key=lambda q: -ctx.stl(q).stat().st_size)
    groups = [pids[i::jobs] for i in range(jobs) if pids[i::jobs]]
    import multiprocessing as mp
    from concurrent.futures import ProcessPoolExecutor
    # 第四任：影子跑进程的命中统计不并进本层 —— 主进程用到每个预取值时按它的出处 note() 一次（统计 + 逐格归属），
    #   并进来会重复计数；影子跑算了、主进程没用上的值不算本层检查用过的原语。
    with ProcessPoolExecutor(max_workers=len(groups), mp_context=mp.get_context("spawn")) as ex:
        for rec in ex.map(_l1_shadow, groups):
            _L1_PREF.update(rec)


def _hr42_wrap_all():
    global _sections, _min_wall, _runs, _support_model, _overhang, _thin_regions
    num_ = lambda x: None if x is None else float(x)
    _sections = _hr42_cached(_sections, lambda CC, A: (CC.mesh_digest(A["mesh"]), num_(A["h"])))
    _overhang = _hr42_cached(_overhang, lambda CC, A: (CC.mesh_digest(A["mesh"]), num_(A["thr_deg"])))
    _min_wall = _hr42_cached(_min_wall, lambda CC, A: (_polys_digest(A["polys"]), num_(A["min_area"]), num_(A["min_layers"]),
                                                       num_(A["r_ref"]), num_(A["lo"]), num_(A["hi"]), int(A["iters"])))
    _runs = _hr42_cached(_runs, lambda CC, A: (_polys_digest(A["polys"]), num_(A["r"]), num_(A["min_area"])))
    _support_model = _hr42_cached(_support_model, lambda CC, A: (_polys_digest(A["polys"]), num_(A["h"]), num_(A["overhang_deg"]),
                                                                 num_(A["min_w"]), bool(A["bridge_exempt"]), int(A["anchor_sides"])))
    _thin_regions = _hr42_cached(_thin_regions, lambda CC, A: (b"<None>" if A["poly"] is None else A["poly"].wkb,
                                                               num_(A["r"]), num_(A["min_area"])))


def _interior_rings(poly):
    """截面里的封闭空腔（多边形的内环）。支撑落在这里面 = 拆不出来。"""
    from shapely.geometry import Polygon
    from shapely.ops import unary_union
    out = []
    for g in getattr(poly, "geoms", [poly]):
        for r in getattr(g, "interiors", []):
            out.append(Polygon(r))
    return unary_union(out) if out else None


# ── 禁撑区：features.yaml → 柱体 → 逐层 2D 区域 ────────────────────────────
def _feature_cylinders(feat):
    """一条特征 → [(中心, 轴向单位向量, 长度, 直径, 标签)]，全在 export_local。
    只认能定位的：axial_span_mm（给出位置+轴向+长度）+ nominal_d_mm；
    多孔阵列再用 hole_positions_mm 平移。定位不到就返回 []（调用方判 unknown）。"""
    import numpy as np
    g = feat.get("geom") or {}
    out = []
    def count_of(node, default=None):
        vals = [node[k] for k in ("count", "holes") if k in node]
        if not vals:
            return default
        if any(isinstance(v, bool) or not isinstance(v, (int, float))
               or not np.isfinite(v) or v <= 0 or int(v) != v for v in vals):
            return None
        return int(vals[0]) if len(set(vals)) == 1 else None

    if "instances" in g:
        insts = g["instances"]
        expected = count_of(g)
        kind = g.get("count_kind")
        if not isinstance(insts, list) or not insts or expected is None or kind not in ("instances", "cylinders"):
            return []
        if kind == "instances" and len(insts) != expected:
            return []
        base = {k: v for k, v in g.items() if k not in ("instances", "count", "holes", "count_kind")}
        for k, inst in enumerate(insts):
            if not isinstance(inst, dict):
                return []
            merged = dict(base)
            merged.update(inst)
            members = _feature_cylinders({"id": feat["id"], "geom": merged})
            if not members:
                return []                 # 一组缺定位时整条不完整，不能只查其余组
            for c in members:
                # 实例标签 = <特征>#<实例>[@<孔序>]：一个实例含多孔时保留成员自己的 @i 后缀，
                # 否则同一实例的几个孔标签全同 → 按标签落点的字典会互相覆盖（09-13 T01-F04#1 被三计）
                sub = str(c[4])[len(str(feat["id"])):]
                out.append((c[0], c[1], c[2], c[3], f"{feat['id']}#{k}{sub}"))
        keys = []
        for center, axis, length, dia, _ in out:
            axis = np.asarray(axis)
            axis = axis if axis[np.flatnonzero(np.abs(axis) > 1e-9)[0]] > 0 else -axis
            keys.append(tuple(np.round(np.r_[center, axis, length, dia], 9)))
        if len(set(keys)) != len(keys):
            return []                     # 重复整组不等于补齐另一个孔
        return out if kind == "instances" or len(out) == expected else []
    span = g.get("axial_span_mm")
    d = _v(g.get("nominal_d_mm"))
    if span is None or d is None or not np.isfinite(d) or d <= 0:
        return []
    try:
        a = np.asarray(span[0], dtype=float)
        b = np.asarray(span[1], dtype=float)
    except Exception:
        return []
    if a.shape != (3,) or b.shape != (3,) or not np.isfinite([a, b]).all():
        return []
    vec = b - a
    L = float(np.linalg.norm(vec))
    if L < 1e-9:
        ax = AX.get(str(g.get("axis")))
        if ax is None:
            return []
        vec = np.asarray(ax, dtype=float)
        L = float(_v(g.get("depth_mm")) or 0.0)
        if L < 1e-9:
            return []
    u = vec / float(np.linalg.norm(vec))
    mid = 0.5 * (a + b)
    centers = g.get("hole_positions_mm", g.get("positions_mm"))
    if centers is not None:
        expected = count_of(g, feat.get("count"))
        if expected is None or len(centers) != expected:
            return []                     # 声明四孔而只有一个坐标，不能只查这一个
        centers = np.asarray(centers, dtype=float)
        if centers.shape != (expected, 3) or not np.isfinite(centers).all() or len(np.unique(centers, axis=0)) != expected:
            return []
        return [(c, u, L, float(d), f"{feat['id']}@{i}") for i, c in enumerate(centers)]
    if count_of(g, feat.get("count", 1)) != 1:
        return []                         # 多孔阵列缺位置，单个 span 不代表所有实例
    return [(mid, u, L, float(d), feat["id"])]


def _parse_box(b):
    """{lo:[x,y,z], hi:[x,y,z]} → (lo, hi)；非数 / 形状不对 / hi ≤ lo → None。"""
    import numpy as np
    if not isinstance(b, dict):
        return None
    try:
        lo = np.asarray(b.get("lo"), dtype=float)
        hi = np.asarray(b.get("hi"), dtype=float)
    except Exception:
        return None
    if lo.shape != (3,) or hi.shape != (3,) or not np.isfinite([lo, hi]).all() or not (hi > lo).all():
        return None
    return lo, hi


def _feature_boxes(feat, key="box_export_local"):
    """非柱形禁撑区（滑配走廊等）：features.yaml 用 export_local 的轴对齐盒 geom.box_export_local
    {lo:[x,y,z], hi:[x,y,z]} 声明（多实例放在 instances[k].box_export_local）。盒是走廊扫掠体的外包
    （AABB），只会多算禁撑区，不会少算 —— 所以"没落进去"的结论仍然可信。返回 [(lo, hi, 标签)]。

    key 可换成 "mating_shells_export_local"（09-13 C-4，滑配走廊的**配合面壳层**）：该字段是一个列表
    [{lo, hi, face, role, …}, …]，每条一个 0.5 mm 厚的盒，标签 = <特征/实例标签>/shell<j>。壳层在
    features.yaml 里是逐面**声明**的（推导写在 mating_shells_derivation），本函数只读不算。
    壳层字段缺失 = 该实例没声明配合面（features.yaml 里写"未定"的那些面），不算格式错误；
    给了却解析不出来（非数 / hi ≤ lo）→ 整条返回 []（调用方判 unknown），与 box_export_local 同一口径。"""
    import numpy as np
    g = feat.get("geom") or {}
    shells = key != "box_export_local"

    def one(node, tag):
        b = node.get(key)
        if shells:
            if b is None:
                return []                 # 没声明配合面 ≠ 坏声明
            if not isinstance(b, list):
                return None
            out = []
            for j, s in enumerate(b):
                r = _parse_box(s) if isinstance(s, dict) else None
                if r is None:
                    return None
                out.append((r[0], r[1], f"{tag}/shell{j}"))
            return out
        r = _parse_box(b)
        return None if r is None else [(r[0], r[1], tag)]

    if "instances" in g:
        insts = g["instances"]
        if not isinstance(insts, list) or not insts:
            return []
        out = []
        for k, inst in enumerate(insts):
            if not isinstance(inst, dict):
                return []
            r = one(inst, f"{feat['id']}#{k}")
            if r is None:
                return []                 # 一个实例缺盒 / 壳层写坏，整条不完整
            out += r
        keys = {tuple(np.round(np.r_[lo, hi], 9)) for lo, hi, _ in out}
        return out if len(keys) == len(out) else []
    r = one(g, feat["id"])
    return r or []


def _prism_aabb(pz):
    """features.yaml 的 prism（L2 约定：点 = origin + u·right + v·up − normal·s，s∈[0, depth]）→ export_local 外包盒 (lo, hi)。"""
    import numpy as np
    if not isinstance(pz, dict):
        return None
    try:
        o, r, up, n = (np.asarray(pz.get(k), dtype=float) for k in ("origin", "right", "up", "normal"))
        d = pz.get("depth_mm")
        d = float(d.get("v") if isinstance(d, dict) else d)
    except (TypeError, ValueError, AttributeError):
        return None
    if any(a.shape != (3,) or not np.isfinite(a).all() for a in (o, r, up, n)) or not (d > 0) \
            or min(float(np.linalg.norm(a)) for a in (r, up, n)) <= 0:
        return None
    uv = []
    for item in pz.get("polygons") or []:
        ext = item.get("exterior") if isinstance(item, dict) else item
        for q in (ext or []):
            try:
                uv.append((float(q[0]), float(q[1])))
            except (TypeError, ValueError, IndexError):
                return None
    if len(uv) < 3:
        return None
    uv = np.asarray(uv)
    r, up, n = r / np.linalg.norm(r), up / np.linalg.norm(up), n / np.linalg.norm(n)
    pts = np.array([o + a * r + b * up - n * t for a in (uv[:, 0].min(), uv[:, 0].max())
                    for b in (uv[:, 1].min(), uv[:, 1].max()) for t in (0.0, d)])
    return pts.min(axis=0), pts.max(axis=0)


def _feature_zone_boxes(feat):
    """feature_zones 纳入的特征 → [(lo, hi, 标签)]（export_local）。每个实例按 box_export_local {lo,hi} → bbox_mm [[lo],[hi]]
    → prism 外包盒的顺序取第一个能解析的；任一实例三者都没有 / 写坏 → 整条返回 []（调用方判"定位不到"）。
    bbox_mm 与 prism 是 features.yaml 给 L2 用的实体特征外包（B03 卡扣就是这样登记的），这里只借它的位置，外扩在调用方。"""
    import numpy as np
    g = feat.get("geom") or {}
    base = {k: v for k, v in g.items() if k not in ("instances",)}

    def one(node):
        r = _parse_box(node.get("box_export_local")) if node.get("box_export_local") is not None else None
        if r is None and node.get("bbox_mm") is not None:
            bb = node.get("bbox_mm")
            r = _parse_box({"lo": bb[0], "hi": bb[1]}) if isinstance(bb, (list, tuple)) and len(bb) == 2 else None
        if r is None and node.get("prism") is not None:
            r = _prism_aabb(node.get("prism"))
            if r is not None and not (r[1] > r[0]).all():
                r = None
        return r

    insts = g.get("instances")
    if isinstance(insts, list) and insts:
        out = []
        for k, inst in enumerate(insts):
            if not isinstance(inst, dict):
                return []
            own = any(inst.get(x) is not None for x in ("box_export_local", "bbox_mm", "prism"))
            node = inst if own else dict(base, **inst)
            r = one(node)
            if r is None:
                return []
            out.append((np.asarray(r[0], float), np.asarray(r[1], float), f"{feat['id']}#{k}"))
        return out
    r = one(g)
    return [(np.asarray(r[0], float), np.asarray(r[1], float), feat["id"])] if r is not None else []


# ── 禁撑区集合（层与 slicing/slice_l1.py 共用；zones_sha256 由此而来）───────────
def no_support_zone_set(P: dict, feats_all: list, pid: str) -> dict:
    """printability.yaml:no_support_zones（两级）+ features.yaml → 本件的禁撑区几何清单（export_local）。

    返回 {zones: [zone…], unlocated: [(fid, kind)…], problems: [str…], sha256, margin, tiers}
      zone = {tag, feature, kind, tier: "forbid"|"removable", shape: "cyl"|"box", post_process, role,
              cyl: center/axis/length/dia ｜ box: lo/hi}
    分级规则（数据在 printability.yaml，这里只读）：
      · tier = per_feature_overrides[fid].tier，否则按 kind 落在 tiers.forbid.kinds / tiers.removable.kinds；
      · slide_corridor：geom.box_export_local（整个口袋+走廊）按 kind 的级（removable），
        geom.mating_shells_export_local（配合面 0.5 mm 壳层）默认 forbid；**per_feature_overrides[fid].mating_shells_tier: removable
        可按件降级**（2026-09-25 用户定：L01-F13 / L03-F05 口袋贴合面接受拆支撑后锉平，见 printability.yaml 该条 why/src），
        降级后落点记 removable、post_process 取 per_feature_overrides[fid].mating_shells_post_process；
      · from_features_yaml_kinds 必须等于两级 kinds 的并集（旧读法不许悄悄失效）→ 不等记 problems。
      · hr41g（2026-09-24）feature_zones.<特征 id> = {tier, face_margin_mm: {v>0, src}, why, src[, post_process]}：
        按 id 纳入 kind 不在列表里的**实体**特征（卡扣凸点/舌片/倒钩），不改 kind、不波及别件同 kind 特征；
        区 = 每个实例的 box_export_local / bbox_mm / prism 外包盒各向外扩 face_margin_mm（支撑中心线只会贴在实体面外，
        不外扩的实体盒恒 0 点 = 假绿，所以 margin 必须 > 0）。条目坏 / 特征不存在 / 同时按 kind 纳入 → problems；
        定位不到 → unlocated。只在本件确有 feature_zones 区时才进 sha（其余件的 zones_sha256 不变）。
    sha256 = 全部 zone 几何（四舍五入 6 位）+ 分级配置 + 轴向外扩 margin 的规范 JSON。
    slice_l1.py 用同一函数建区、把 sha 记进 support_landing.zones_sha256；层核对 sha 相同才采信记录。
    **本函数不做任何判定**：unlocated / problems 非空时由调用方判 unknown。"""
    import hashlib, json
    import numpy as np
    cfg = P.get("no_support_zones") or {}
    tiers = cfg.get("tiers") or {}
    forbid_kinds = set((tiers.get("forbid") or {}).get("kinds") or [])
    rem_cfg = tiers.get("removable") or {}
    rem_kinds = set(rem_cfg.get("kinds") or [])
    post = rem_cfg.get("post_process") or {}
    overrides = cfg.get("per_feature_overrides") or {}
    declared = set(cfg.get("from_features_yaml_kinds") or [])
    margin = _v((P.get("criteria") or {}).get("forbidden_zone_axial_margin_mm"))
    problems = []
    if not forbid_kinds or not rem_kinds:
        problems.append("printability.yaml:no_support_zones.tiers 缺 forbid.kinds / removable.kinds（两级分级）")
    if forbid_kinds & rem_kinds:
        problems.append(f"同一 kind 同时在 forbid 与 removable：{sorted(forbid_kinds & rem_kinds)}")
    if declared != (forbid_kinds | rem_kinds):
        problems.append("from_features_yaml_kinds ≠ tiers.forbid.kinds ∪ tiers.removable.kinds："
                        f"差集 {sorted(declared ^ (forbid_kinds | rem_kinds))}")
    for fid, ov in overrides.items():
        if not isinstance(ov, dict) or ov.get("tier") not in ("forbid", "removable"):
            problems.append(f"per_feature_overrides.{fid}.tier 必须是 forbid|removable")
        elif ov.get("mating_shells_tier") is not None and ov.get("mating_shells_tier") not in ("forbid", "removable"):
            problems.append(f"per_feature_overrides.{fid}.mating_shells_tier 必须是 forbid|removable")   # 2026-09-25 配合面壳层按件降级
    if margin is None:
        problems.append("criteria.forbidden_zone_axial_margin_mm 缺失")
    fzones = cfg.get("feature_zones") or {}
    if not isinstance(fzones, dict):
        problems.append("no_support_zones.feature_zones 必须是 {特征 id: {tier, face_margin_mm, why, src}}")
        fzones = {}
    feat_ids = {f.get("id") for f in feats_all}
    fz_ok = {}
    for fid, e in fzones.items():
        bad = []
        if fid not in feat_ids:
            bad.append(f"feature_zones.{fid} 在 features.yaml 里找不到这个特征")
        if not isinstance(e, dict):
            bad.append(f"feature_zones.{fid} 必须是 dict")
        else:
            if e.get("tier") not in ("forbid", "removable"):
                bad.append(f"feature_zones.{fid}.tier={e.get('tier')!r} 必须是 forbid|removable")
            fm = e.get("face_margin_mm")
            fmv = _v(fm)
            if not (isinstance(fmv, (int, float)) and not isinstance(fmv, bool) and math.isfinite(fmv) and fmv > 0):
                bad.append(f"feature_zones.{fid}.face_margin_mm={fm!r} 必须是 > 0 的数（实体特征不外扩 = 恒 0 点假绿）")
            elif not (isinstance(fm, dict) and fm.get("src")):
                bad.append(f"feature_zones.{fid}.face_margin_mm 缺 src（阈值要有来源）")
        if bad:
            # 只报本件的（别件的坏条目在别件的记录里报；不存在的特征 id 每件都报 —— 它属于谁说不清）
            owner = next((f.get("part") for f in feats_all if f.get("id") == fid), None)
            if owner in (None, pid):
                problems += bad
            continue
        fz_ok[fid] = e

    zones, unloc = [], []
    for f in feats_all:
        kind = f.get("kind")
        fid = f.get("id")
        if f.get("part") != pid:
            continue
        if fid in fz_ok:
            if kind in declared:
                problems.append(f"{fid} 既按 kind={kind} 纳入又在 feature_zones 里 —— 二选一（级别说不清）")
                continue
            e = fz_ok[fid]
            fm = float(_v(e["face_margin_mm"]))
            tier = e["tier"]
            bs = _feature_zone_boxes(f)
            if not bs:
                unloc.append((fid, kind))
                continue
            for lo, hi, tag in bs:
                zones.append(dict(feature=fid, kind=kind, tier=tier,
                                  post_process=((e.get("post_process") or post.get(kind)) if tier == "removable" else None),
                                  tag=tag, shape="box", role="feature_zone",
                                  lo=[float(x) - fm for x in lo], hi=[float(x) + fm for x in hi]))
            continue
        if kind not in declared:
            continue
        ov = overrides.get(fid) or {}
        tier = ov.get("tier") or ("forbid" if kind in forbid_kinds else "removable")
        pp = ov.get("post_process") or post.get(kind)
        base = dict(feature=fid, kind=kind, tier=tier, post_process=(pp if tier == "removable" else None))
        cs = _feature_cylinders(f)
        if cs:
            for c, u, L, d, tag in cs:
                u = np.asarray(u, dtype=float)
                zones.append(dict(base, tag=tag, shape="cyl", role="feature",
                                  center=[float(x) for x in c], axis=[float(x) for x in u],
                                  length=float(L), dia=float(d)))
            continue
        bs = _feature_boxes(f)
        if bs:
            for lo, hi, tag in bs:
                zones.append(dict(base, tag=tag, shape="box", role="feature",
                                  lo=[float(x) for x in lo], hi=[float(x) for x in hi]))
            raw_shells = (f.get("geom") or {}).get("mating_shells_export_local")
            has_shells = raw_shells is not None or any(
                isinstance(i, dict) and i.get("mating_shells_export_local") is not None
                for i in ((f.get("geom") or {}).get("instances") or []))
            shells = _feature_boxes(f, key="mating_shells_export_local")
            if has_shells and not shells:
                problems.append(f"{fid}.mating_shells_export_local 写坏（非 [{{lo,hi}}…] / hi ≤ lo / 重复）")
            # 2026-09-25（用户定"按 1"）：壳层默认 forbid；per_feature_overrides[fid].mating_shells_tier=removable 时按件降级，
            #   post_process 用 mating_shells_post_process（缺则退到 kind 的通用文本）。默认路径与 09-13 C-4 逐字相同。
            sh_tier = ov.get("mating_shells_tier") if ov.get("mating_shells_tier") in ("forbid", "removable") else "forbid"   # 写坏 → 上面已记 problems（unknown），这里退回默认不让后面 KeyError
            sh_pp = (ov.get("mating_shells_post_process") or post.get(kind)) if sh_tier == "removable" else None
            for lo, hi, tag in shells:
                zones.append(dict(feature=fid, kind=kind, tier=sh_tier, post_process=sh_pp,
                                  tag=tag, shape="box", role="mating_shell",
                                  lo=[float(x) for x in lo], hi=[float(x) for x in hi]))
            continue
        unloc.append((fid, kind))

    tags = [z["tag"] for z in zones]
    if len(set(tags)) != len(tags):
        problems.append("禁撑区标签重复（同一标签的落点会互相覆盖）：" + ",".join(sorted({t for t in tags if tags.count(t) > 1}))[:120])

    def _r(x):
        return [round(float(v), 6) for v in x]
    body = {"margin": margin,
            "tiers": {"forbid": sorted(forbid_kinds), "removable": sorted(rem_kinds),
                      "post_process": {k: str(v) for k, v in sorted(post.items())},
                      # 只带**本件特征**的 override：别件的 override 改了不该让本件的落点记录作废（09-13）
                      "overrides": {k: {"tier": (v or {}).get("tier"), "post_process": str((v or {}).get("post_process")),
                                        "mating_shells_tier": (v or {}).get("mating_shells_tier"),
                                        "mating_shells_post_process": str((v or {}).get("mating_shells_post_process"))}
                                    for k, v in sorted(overrides.items())
                                    if isinstance(v, dict) and k in {z["feature"] for z in zones}}},
            "zones": sorted(
                ({"tag": z["tag"], "tier": z["tier"], "shape": z["shape"], "role": z["role"],
                  **({"center": _r(z["center"]), "axis": _r(z["axis"]), "length": round(z["length"], 6), "dia": round(z["dia"], 6)}
                     if z["shape"] == "cyl" else {"lo": _r(z["lo"]), "hi": _r(z["hi"])})}
                 for z in zones), key=lambda d: d["tag"])}
    used_fz = sorted({z["feature"] for z in zones if z.get("role") == "feature_zone"})
    if used_fz:                                   # 只在本件确有 feature_zones 区时进 sha：别件的 zones_sha256 不因本字段存在而作废
        body["feature_zones"] = {k: {"tier": fz_ok[k].get("tier"), "face_margin_mm": _v(fz_ok[k].get("face_margin_mm")),
                                     "post_process": str(fz_ok[k].get("post_process"))} for k in used_fz}
    sha = hashlib.sha256(json.dumps(body, sort_keys=True, ensure_ascii=False).encode("utf-8")).hexdigest()
    return {"zones": zones, "unlocated": unloc, "problems": problems, "sha256": sha, "margin": margin,
            "tiers": {"forbid": sorted(forbid_kinds), "removable": sorted(rem_kinds)}}


def zone_hits(zones: list, pts, margin: float) -> dict:
    """export_local 采样点 × 禁撑区 → {tag: 落进去的点数}。柱体沿轴两端各外扩 margin（盲孔底那一层），
    盒按声明原样。滑配走廊的口袋盒（role=feature）**扣掉**落在它自己配合面壳层（tag 前缀相同的 role=mating_shell）
    里的点 —— 『口袋其它体积归 removable』，同一点不同时记 forbid 与 removable。纯几何，无判定。"""
    import numpy as np
    P = np.asarray(pts, dtype=float).reshape(-1, 3)
    if len(P) == 0:
        return {z["tag"]: 0 for z in zones}
    masks = {}
    for z in zones:
        if z["shape"] == "cyl":
            c = np.asarray(z["center"], dtype=float); u = np.asarray(z["axis"], dtype=float)
            u = u / max(float(np.linalg.norm(u)), 1e-12)
            v = P - c
            a = v @ u
            rad = np.linalg.norm(v - np.outer(a, u), axis=1)
            m = (np.abs(a) <= z["length"] / 2.0 + float(margin or 0.0)) & (rad <= z["dia"] / 2.0)
        else:
            lo = np.asarray(z["lo"], dtype=float); hi = np.asarray(z["hi"], dtype=float)
            m = np.all((P >= lo) & (P <= hi), axis=1)
        masks[z["tag"]] = m
    out = {}
    for z in zones:
        m = masks[z["tag"]]
        if z["shape"] == "box" and z.get("role") == "feature":
            for s in zones:
                if s.get("role") == "mating_shell" and s["tag"].startswith(z["tag"] + "/shell"):
                    m = m & ~masks[s["tag"]]
        out[z["tag"]] = int(m.sum())
    return out


def _box_section(lo, hi, z, margin):
    """轴对齐盒在高度 z 的截面（打印坐标系，盒经置换阵旋转后仍是轴对齐盒）。"""
    from shapely.geometry import box as _box
    if not (lo[2] - margin <= z <= hi[2] + margin):
        return None
    return _box(min(lo[0], hi[0]), min(lo[1], hi[1]), max(lo[0], hi[0]), max(lo[1], hi[1]))


def _cyl_section(center, axis, length, dia, z, margin):
    """柱体在高度 z 处的 2D 截面（打印坐标系）。轴要么竖直要么水平（旋转是置换阵保证的）。"""
    from shapely.geometry import Point, Polygon
    import numpy as np
    r = dia / 2.0
    az = float(axis[2])
    half = length / 2.0
    if abs(abs(az) - 1.0) < 1e-6:                       # 竖直孔 → 圆
        lo = center[2] - half - margin
        hi = center[2] + half + margin
        if not (lo <= z <= hi):
            return None
        return Point(center[0], center[1]).buffer(r, quad_segs=32)
    if abs(az) < 1e-6:                                  # 水平孔 → 矩形（宽随 z 变）
        dz = abs(z - float(center[2]))
        if dz > r + margin:
            return None
        w = math.sqrt(max(r * r - min(dz, r) ** 2, 0.0))
        if w <= 0 and dz > r:
            return None
        w = max(w, 1e-6)
        u = np.array([float(axis[0]), float(axis[1])])
        u = u / max(float(np.linalg.norm(u)), 1e-12)
        p = np.array([-u[1], u[0]])
        c = np.array([float(center[0]), float(center[1])])
        e = half + margin
        pts = [c + u * e + p * w, c - u * e + p * w, c - u * e - p * w, c + u * e - p * w]
        return Polygon([tuple(x) for x in pts])
    return "SKEW"


# ── 层 ─────────────────────────────────────────────────────────────────────
CAL_RECORD_FILE = "tools/gate/data/l1_calibration.yaml"
CAL_UNKNOWN_CRITERION = "必须有数（未知=失败）"     # core.LayerResult.unknown 写的 criterion 原文；标定统计按它识别 unknown


def calibration_fingerprint(P: dict) -> str:
    """标定记录的有效性指纹：判据参数（process/criteria/materials）+ 标定集声明 + 本层代码。任一变 → 记录过期。"""
    import hashlib, json
    body = {k: P.get(k) for k in ("process", "criteria", "materials", "calibration_set")}
    h = hashlib.sha256(json.dumps(body, sort_keys=True, ensure_ascii=False, default=str).encode("utf-8"))
    h.update(Path(__file__).read_bytes())
    return h.hexdigest()


def _calibration(res, ctx, P):
    """`_calibration` 格：用 calibrate_l1.py 跑出的记录（data/l1_calibration.yaml）判本层 BLOCK 判据有没有误伤已证明能打的件。
    只写不变量：记录必须存在、指纹必须对得上、每件网格 sha 必须对得上、BLOCK 判据在可评件上的失败比例 ≤ max。"""
    import hashlib
    cs = P.get("calibration_set") or {}
    prov = "printability.yaml:calibration_set + " + CAL_RECORD_FILE
    if not cs or not cs.get("parts"):
        res.unknown("_calibration", "proven_parts", "printability.yaml 没有 calibration_set 声明（标定集是谁、朝向、失败比例上限）", provenance=prov)
        return
    max_frac = _v(cs.get("max_block_fail_fraction"))
    if max_frac is None:
        res.unknown("_calibration", "proven_parts", "calibration_set.max_block_fail_fraction 没有数", provenance=prov)
        return
    rec = (ctx.data.get("l1_calibration") or {}).get("l1_calibration") or {}
    res.inputs.append(CAL_RECORD_FILE)
    if not rec:
        res.unknown("_calibration", "proven_parts", "没有标定记录：先跑 ./.venv/bin/python tools/gate/calibrate_l1.py", provenance=prov)
        return
    fp = calibration_fingerprint(P)
    if rec.get("fingerprint") != fp:
        res.add(subject="_calibration", check="proven_parts", state=FAIL, severity=BLOCK,
                measured=f"记录指纹 {str(rec.get('fingerprint'))[:12]}… ≠ 当前 {fp[:12]}…", evidence_n=0,
                criterion="标定记录的指纹（判据参数 + 标定集声明 + l1_printable.py）必须等于当前值；判据一改就得重标定",
                detail="重跑 tools/gate/calibrate_l1.py", provenance=prov)
        return
    bad = []
    for cp in cs["parts"]:
        cid = cp.get("id"); mp = ROOT / str(cp.get("mesh") or "")
        r = (rec.get("parts") or {}).get(cid) or {}
        if not mp.exists():
            bad.append(f"{cid}: 网格缺失 {cp.get('mesh')}")
            continue
        sha = hashlib.sha256(mp.read_bytes()).hexdigest()
        if r.get("mesh_sha256") != sha:
            bad.append(f"{cid}: 网格 sha 与记录不符")
        res.inputs.append(str(cp.get("mesh")))
    if bad:
        res.add(subject="_calibration", check="proven_parts", state=FAIL, severity=BLOCK,
                measured="; ".join(bad)[:300], evidence_n=len(cs["parts"]),
                criterion="标定集每个网格必须存在且 sha256 与记录一致", detail="重跑 tools/gate/calibrate_l1.py", provenance=prov)
        return
    scope = cs.get("checks_in_scope")
    if not isinstance(scope, list) or not scope:
        res.unknown("_calibration", "proven_parts", "calibration_set.checks_in_scope 没声明（哪些判据是量几何的、要被标定）", provenance=prov)
        return
    checks = rec.get("checks") or {}
    missing_scope = [k for k in scope if k not in checks]
    over, warn_over, never, rows = [], [], [], []
    n_warn_eval = 0
    if missing_scope:
        rows.append("记录里没有这些范围内判据的统计：" + ",".join(missing_scope))
    for name, c in sorted(checks.items()):
        if name not in scope:
            continue
        n_eval = int(c.get("n_eval") or 0); n_fail = int(c.get("n_fail") or 0); sev = c.get("severity")
        frac = (n_fail / n_eval) if n_eval else None
        rows.append(f"{name}[{sev}] {n_fail}/{n_eval}")
        if sev == BLOCK and n_eval == 0:
            never.append(name)
        if sev == BLOCK and n_eval > 0 and frac is not None and frac > max_frac:
            over.append(f"{name} {n_fail}/{n_eval}={frac:.2f} > {max_frac}")
        if sev == WARN:
            n_warn_eval += n_eval
            if n_eval > 0 and frac is not None and frac > max_frac:
                warn_over.append(f"{name} {n_fail}/{n_eval}={frac:.2f} > {max_frac}")
    # F-L1-2（2026-09-13，用户口径：只报不判）：范围内 WARN 判据超限另发一条 WARN，不阻断、不进 proven_parts。
    res.add(subject="_calibration", check="warn_checks_over_limit",
            state=FAIL if warn_over else PASS, severity=WARN,
            measured=("; ".join(warn_over)) if warn_over else f"范围内 WARN 判据无一超过 {max_frac}",
            evidence_n=n_warn_eval,
            criterion=(f"checks_in_scope 里 severity=WARN 的 L1 判据在标定集可评件上的失败比例 ≤ {max_frac}；"
                       "超过只报 WARN（该判据红了好件，阈值可疑），不阻断、不计入 proven_parts。范围外判据只在 detail 里列出。"),
            detail=f"记录 {rec.get('date')}；逐判据：" + "；".join(rows), provenance=prov)
    # F-L1-1（2026-09-13）：范围内任何 BLOCK 判据 n_eval == 0 = 在标定集上一次都没评过 → 标定不成立（unknown）。
    # 以前这里对 n_eval=0 什么都不说，"没评过"被当成"没误伤"。记录缺失（missing_scope）连严重级都不知道，同样 unknown。
    if never or missing_scope:
        res.unknown("_calibration", "proven_parts",
                    f"范围内 BLOCK 判据 {never + missing_scope} 在标定集上一次都没评（n_eval=0 / 记录缺失），标定不成立"
                    f" —— 『没评过』不是『没误伤』。"
                    + (f" 已评判据里另有误伤：{'; '.join(over)}。" if over else "")
                    + f" 逐判据：{'；'.join(rows)}。重跑 tools/gate/calibrate_l1.py，或让这些判据在标定件上真的评到数",
                    provenance=prov)
        return
    res.add(subject="_calibration", check="proven_parts",
            state=FAIL if over else PASS, severity=BLOCK,
            measured=("误伤：" + "; ".join(over)) if over else f"BLOCK 判据无一超过 {max_frac}",
            evidence_n=len(cs["parts"]),
            criterion=(f"对 {len(cs['parts'])} 件已实打实打出来的原版件（printability.yaml:calibration_set），"
                       f"checks_in_scope 里**每一条** severity=BLOCK 的 L1 判据都必须在可评件上评过（n_eval ≥ 1，"
                       f"一次都没评 = unknown = 标定不成立），且失败比例 ≤ {max_frac}；unknown（含 PASS 而 evidence_n=0）不计入分母。"
                       "超过 = 该判据不是可打印性边界（它红了好件），不能再当 BLOCK 用。WARN 判据超限见同格 warn_checks_over_limit（只报不判）。"),
            detail=f"记录 {rec.get('date')}；逐判据：" + "；".join(rows),
            provenance=prov)


_hr42_wrap_all()


def run(ctx) -> LayerResult:
    """hr42：外包一层 —— 先并行影子跑把重函数结果预取进 _L1_PREF，再照原样全量 run（判据/顺序/证据不变），最后清空预取表。"""
    try:
        _l1_prefetch(ctx)
    except Exception as e:                                        # noqa: BLE001  预取失败只丢速度
        print(f"[L1 预取] 失败，改串行：{e}", flush=True)
    try:
        return _run_impl(ctx)
    finally:
        if __import__("os").environ.get("L1_SHADOW") != "1":
            _L1_PREF.clear()


def _run_impl(ctx) -> LayerResult:
    import numpy as np
    from shapely.ops import unary_union

    res = LayerResult(LAYER, NAME)
    P = ctx.data.get("printability") or {}
    if not P:
        res.unknown("_layer", "printability_data",
                    "data/printability.yaml 读不到 —— 第 1 层的全部阈值都在那里",
                    provenance="tools/gate/data/printability.yaml")
        return res

    proc = P.get("process") or {}
    crit = P.get("criteria") or {}
    mats = P.get("materials") or {}
    srun = (P.get("slice_run") or {})
    sparts = srun.get("parts") or {}

    nozzle = _v(proc.get("nozzle_d_mm"))
    h = _v(proc.get("layer_h_mm"))
    lw = _v(proc.get("wall_line_width_mm"))
    n_peri = _v(crit.get("min_wall_perimeters"))
    hole_mult = _v(crit.get("min_hole_nozzle_multiple"))
    oh_deg = _v(crit.get("overhang_angle_from_vertical_deg"))
    sup_deg = _v(crit.get("support_model_overhang_deg"))
    min_area = _v(crit.get("thin_region_min_area_mm2"))
    min_layers = int(_v(crit.get("thin_region_min_layers")) or 0)
    margin = _v(crit.get("forbidden_zone_axial_margin_mm"))
    sup_max = _v(crit.get("support_in_forbidden_max_mm3"))
    min_ow = _v(crit.get("support_min_overhang_width_mm"))
    sup_sp = _v(proc.get("support_material_spacing_mm"))
    sup_lw = _v(proc.get("support_line_width_mm"))
    sup_fill = (sup_lw / sup_sp) if (sup_lw and sup_sp) else 0.0
    crit_bridge = crit.get("max_bridge_span_mm")
    anchor_sides = int(_v(crit.get("min_bridge_anchor_sides")) or 2)
    first_min = _v(crit.get("first_layer_min_area_mm2"))
    bridge_span = _v(crit.get("max_bridge_span_mm"))
    hole_subs = ((crit.get("min_hole_check") or {}).get("feature_kind_substrings")
                 or ["hole", "bore"])

    missing = [k for k, v in (("nozzle_d_mm", nozzle), ("layer_h_mm", h),
                              ("wall_line_width_mm", lw), ("min_wall_perimeters", n_peri),
                              ("min_hole_nozzle_multiple", hole_mult),
                              ("overhang_angle_from_vertical_deg", oh_deg),
                              ("support_model_overhang_deg", sup_deg),
                              ("thin_region_min_area_mm2", min_area),
                              ("thin_region_min_layers", min_layers or None),
                              ("support_min_overhang_width_mm", min_ow),
                              ("support_material_spacing_mm", sup_sp),
                              ("forbidden_zone_axial_margin_mm", margin)) if v is None]
    if missing:
        res.unknown("_layer", "thresholds_present",
                    f"printability.yaml 缺这些判据参数：{missing}",
                    provenance="tools/gate/data/printability.yaml:process/criteria")
        return res

    # 全层级的两条"未知=失败"：没有阈值就是没有阈值，不许拿一个数凑绿
    res.unknown("_layer", "first_layer_adhesion_threshold",
                "首层贴床面积没有判据阈值：" + (_why(crit.get("first_layer_min_area_mm2"))
                                              or "printability.yaml:criteria.first_layer_min_area_mm2 = null"),
                severity=WARN,
                provenance="printability.yaml:criteria.first_layer_min_area_mm2")
    res.unknown("_layer", "bridge_span_threshold",
                "桥接跨距没有实测：" + (_why(crit.get("max_bridge_span_mm")) or "")
                + " → 本层对所有件一律不做桥接豁免（保守：多算支撑）",
                severity=WARN,
                provenance="printability.yaml:criteria.max_bridge_span_mm")
    res.inputs.append("V2_S288版发布产物/01_整鸭打印件/打印清单与参数.md")

    feats_all = (ctx.data.get("features") or {}).get("features") or []
    parts_recs = {p["id"]: p for p in ((ctx.data.get("parts") or {}).get("parts") or [])}

    wall_req = n_peri * lw
    wall_req_nozzle = n_peri * nozzle
    hole_req = hole_mult * nozzle

    checked = []
    for pid in ctx.parts:
        if ctx.only and pid not in ctx.only:
            continue
        rec = parts_recs.get(pid) or {}
        stl = ctx.stl(pid)
        if stl is None:
            res.unknown(pid, "stl_exists", f"cad/duck_s288/ 里找不到 {pid} 的导出 STL",
                        provenance="parts.yaml:parts[].id")
            continue
        res.inputs.append(str(stl.relative_to(ROOT)))

        # ── 材料专项阈值（TPU 的全是 null → 下面逐条判 unknown）
        mat = rec.get("material")
        mspec = mats.get(mat) or {}
        m_peri = _v(mspec.get("min_wall_perimeters"))
        m_oh = _v(mspec.get("overhang_angle_from_vertical_deg"))
        m_rem = mspec.get("supports_removable")
        m_rem_v = m_rem.get("v") if isinstance(m_rem, dict) else m_rem
        if not mspec:
            res.unknown(pid, "material_rules",
                        f"printability.yaml:materials 里没有材料 '{mat}' 的规则",
                        provenance="parts.yaml:parts[].material")

        # ── 判据 0：打印朝向必须是机器可读的，且和真切片用的那一个一致
        down = rec.get("print_orientation_down_normal")
        srec = sparts.get(pid) or {}
        if srec.get("slice_cache_key"):                   # hr42（第四任）：本件切片记录是沿用上次的（slice_l1 按内容复用）→ 归到本格的 cached_from
            _hr42_cc().note("slice", bool(srec.get("slice_cached_from")), srec.get("slice_cached_from"),
                            str(srec["slice_cache_key"]).encode())
        if not (isinstance(down, (list, tuple)) and len(down) == 3):
            res.unknown(pid, "print_orientation",
                        "parts.yaml 没有机器可读的 print_orientation_down_normal（自由文本不能当判据）",
                        provenance="parts.yaml:parts[].print_orientation_down_normal")
            continue
        d_slice = srec.get("down_normal_export_local")
        same = (d_slice is not None
                and len(d_slice) == 3
                and all(abs(float(a) - float(b)) < 1e-6 for a, b in zip(down, d_slice)))
        res.add(subject=pid, check="print_orientation",
                state=PASS if same else FAIL, severity=BLOCK,
                measured=list(down), evidence_n=3,
                criterion="parts.yaml 声明的贴床法向（件自身坐标系）必须等于 slice_run 里真正切过的那一个",
                detail=(f"自由文本『{rec.get('print_orientation')}』；"
                        f"世界写法 {rec.get('print_orientation_down_normal_world')}；"
                        f"切片用的 {d_slice}"),
                provenance="parts.yaml:print_orientation_down_normal / printability.yaml:slice_run")

        # ── 判据 0b：切片新鲜度。对不上就是拿过期 gcode 下结论
        cur_sha = sha256_file(stl)
        rec_sha = srec.get("source_sha256")
        fresh = bool(rec_sha) and rec_sha == cur_sha
        rc = srec.get("returncode")
        warns = srec.get("warnings") or []
        res.add(subject=pid, check="slice_freshness",
                state=PASS if (fresh and rc == 0 and not warns) else FAIL, severity=BLOCK,
                measured="fresh" if fresh else "STALE",
                evidence_n=2,
                criterion="printability.yaml:slice_run 记录的 source_sha256 必须等于当前导出 STL 的 sha256，"
                          "且切片 returncode=0、无告警（元规则 1/6：验的和印的必须是同一份）",
                detail=(f"当前 {cur_sha[:12] if cur_sha else '?'} / 记录 "
                        f"{(rec_sha or '(无记录)')[:12]}；rc={rc}；告警 {warns[:2]}"
                        + ("" if fresh else "　→ 本件的切片派生判据（墙/支撑/桥接）不可用")),
                provenance=f"printability.yaml:slice_run.parts.{pid}.source_sha256")

        # ── 摆到打印床上
        _hr42_cc().set_context(f"l1:{pid}")               # hr42：只给原语缓存的未命中日志用
        try:
            mesh0 = ctx.solid(stl)
        except Exception as e:
            res.unknown(pid, "mesh_load", f"读 STL 失败：{e}")
            continue
        oriented, R = _orient(mesh0, down)
        if oriented is None:
            res.unknown(pid, "orientation_transform",
                        f"贴床法向 {down} 转不出置换旋转（斜朝向），本层的解析截面不适用",
                        provenance="l1_printable.py:_orient")
            continue
        zs, polys = _sections(oriented, h)
        n_layers = len(zs)
        if n_layers == 0 or all(p is None for p in polys):
            res.unknown(pid, "sections", "逐层截面一层都取不到（网格或朝向有问题）")
            continue
        solid_polys = [p for p in polys if p is not None]

        # ── 判据 1a：逐层最薄壁（真几何）
        # 圈数必须用**本件材料**的 materials.<mat>.min_wall_perimeters。
        # criteria.min_wall_perimeters 只是没有材料专项时的全局读法：材料给了 null（TPU 95A）时
        # 不许拿它顶上去 —— 那等于把 PLA 的结论安到 TPU 头上（元规则 3：未知=失败）。
        # 材料给了值就一路用材料值（下面的 r_crit / bad_layers / best_run / 最薄壁全跟着它走），
        # 不许『算了材料专属圈数、却仍拿全局圈数去量』。
        peri_use = m_peri if m_peri is not None else n_peri
        req = peri_use * lw
        r_crit = req / 2.0                 # 宽 W 的墙腐蚀 W/2 才消失 → 要 N 圈就腐蚀 N·w/2
        bad_layers, thin_area, raw_area, worst = 0, 0.0, 0.0, None
        for z, p in zip(zs, polys):
            k, a, raw = _thin_regions(p, r_crit, min_area)
            raw_area += raw
            if k:
                bad_layers += 1
                thin_area += a
                if worst is None or a > worst[1]:
                    worst = (float(z), a)
        # 判红：门槛半径 r_crit + yaml 原样的固定面积门槛（这一条是判据，一个字没动）
        _, best_run, _ = _runs(polys, r_crit, min_area)
        # 报数：往细处二分时面积门槛按 r² 缩放（理由见 _min_wall 的 docstring），
        #       不然 measured 会把真实肉厚**高报**。这一步不参与 PASS/FAIL。
        mw, _r, floored = _min_wall(polys, min_area, min_layers, r_crit)
        if mw is not None:
            meas = mw_txt = _fmt(mw)
        elif floored:
            # 比二分下限还窄，不许写成一个像样的数。这种件（多半是 CSG 相切留下的刀刃）
            # 光报 "<0.04" 会把信息全丢掉，所以补一个**固定噪声底口径**（r_ref=0 = 老口径）的数
            # 一起报：两个数分别标好口径，读的人自己看要哪一个。
            mw2, _r2, floored2 = _min_wall(polys, min_area, min_layers, 0)
            flat = (f"{mw2:.3f}" if mw2 is not None
                    else (f"<{2 * _r2:.2f}" if floored2 else f">{2 * _r2:.2f}"))
            mw_txt = f"<{2 * _r:.2f}"
            meas = f"{mw_txt} ｜ 固定噪声底口径 {flat}"
        else:
            meas = mw_txt = f">{2 * _r:.2f}"
        howto = (f"measured = 最薄壁（**层平面内**，逐层 2D 截面上量的，不是 3D 最小肉厚）"
                 f"{mw_txt} mm：二分找最小的 r，使得连续 ≥ {min_layers} 层都还有"
                 f"面积 ≥ {min_area}×min(1, r/{r_crit:.3f})² mm² 的开运算损失区。"
                 f"面积门槛往细处按 r² 缩小、往粗处封顶在 yaml 原值，是因为它本来就是『凸直角圆角化』的噪声底、"
                 f"而单角损失 ∝ r²（往上也缩会让 hit(r) 不单调、二分失效）；"
                 f"r = {r_crit:.3f}（判红那一档）时缩放因子恰为 1，与 "
                 f"printability.yaml:criteria.thin_region_min_area_mm2 完全一致。"
                 f"复现：./.venv/bin/python + trimesh 读同一个 STL → l1_printable._orient 按 "
                 f"parts.yaml:print_orientation_down_normal 摆正 → _sections(m, {h}) 取截面 → "
                 f"对某层做 p.difference(p.buffer(-r).buffer(r))，把 r 从 {r_crit:.3f} 往下扫，"
                 f"看削掉的区域在哪一档消失，消失前最后一档的 2r 就是那圈肉的宽度。"
                 f"报成 <x 的件是二分下限就已经触发 —— 那多半是 CSG 相切留下的刀刃，"
                 f"不是墙；这种情况另附『固定噪声底口径』的数（面积门槛不随 r 缩放，"
                 f"= 本文件改缩放之前的老口径），两个数各自标了口径，别混着读。")
        if m_peri is None:
            res.unknown(pid, "min_wall_geometric",
                        f"材料 '{mat}' 没有最小壁厚判据：" + _why(mspec.get("min_wall_perimeters"))
                        + f"　| 只能报几何量到的数、不能判合格：最薄壁"
                          f"（层平面内、逐层 2D 量的，不是 3D 最小肉厚）{meas} mm；"
                          f"按**全局读法** criteria.min_wall_perimeters={n_peri} × 线宽 {lw} = "
                          f"{wall_req:.2f} mm 量，会有 {bad_layers}/{n_layers} 层放不下"
                          f"（最长连续 {best_run} 层）—— 这个数只作参考，不是本件材料的判据。",
                        provenance=f"printability.yaml:materials.{mat}.min_wall_perimeters")
        else:
            ok = best_run < min_layers
            # 2026-09-12/13 标定：同一判据判 16 件已实打实打出来的原版件 11 红（printability.yaml:criteria.min_wall_perimeters.calibration_2026-09-12）
            # → 这个 2D 指标不是可打印性的边界，降 WARN；墙数的 BLOCK 判据由下面的 min_wall_sliced（真 G-code）承担。
            res.add(subject=pid, check="min_wall_geometric",
                    state=PASS if ok else FAIL, severity=WARN,
                    measured=meas,
                    evidence_n=n_layers,
                    criterion=f"逐层截面开运算，圈数取**本件材料**的 materials.{mat}.min_wall_perimeters="
                              f"{m_peri}：不许出现『局部宽度 < {m_peri}×{lw}={req:.2f} mm、"
                              f"面积 ≥ {min_area} mm²、且连续 ≥ {min_layers} 层』的区域"
                              f"（2×喷嘴的宽松读法是 {wall_req_nozzle:.2f} mm；连续层数的理由见 "
                              f"printability.yaml:criteria.thin_region_min_layers）。"
                              f"『局部宽度』只量**层平面内**的宽度（开运算做在每层的 2D 截面上），"
                              f"因为『放不放得下 N 圈墙』本来就是逐层的 2D 性质。"
                              f"**不做的事**：最窄方向沿打印 Z 的肉（比如轴线摆成水平的沉窝之间那圈肉），"
                              f"在每一层的截面里都比它真实的 3D 厚度宽，本判据量不到、也不判 —— "
                              f"那是强度问题不是挤不挤得出来的问题，本层不覆盖它，别把这里的绿读成"
                              f"『3D 最小肉厚合格』。"
                              f"判红只看这一档，不看 measured 的大小。",
                    detail=(f"{bad_layers}/{n_layers} 层放不下 {m_peri} 圈，最长连续 {best_run} 层；"
                            f"薄区面积合计 {thin_area:.3f} mm²"
                            f"（未滤凸角圆角化的原始值 {raw_area:.3f} mm²）"
                            + (f"；面积最大的一层 z={worst[0]:.2f} mm 丢 {worst[1]:.3f} mm²" if worst else "")
                            + f"；层高 {h}，共 {n_layers} 层。" + howto),
                    provenance=f"printability.yaml:materials.{mat}.min_wall_perimeters + "
                               f"process.wall_line_width_mm")

        # ── 判据 1b：真切片给的墙数（只有外圈没有内圈的层 = 那层的墙只有 1 圈）
        eo = srec.get("layers_external_perimeter_only")
        sb = srec.get("layers_single_bead")
        lwp = srec.get("layers_with_perimeter")
        if not fresh or eo is None:
            res.unknown(pid, "min_wall_sliced",
                        "没有对应当前 STL 的切片结果，取不到真实墙数",
                        provenance=f"printability.yaml:slice_run.parts.{pid}")
        elif sb is None:
            # 09-12 第二批：旧记录只有 layers_external_perimeter_only。Arachne 把 2 条 bead 的墙也只标
            # External perimeter，所以那个数不能判"单圈"；没有 layers_single_bead 就是没量过 → unknown。
            res.unknown(pid, "min_wall_sliced",
                        f"切片记录缺 layers_single_bead（只有 ext_only={eo}，Arachne 下它不等于单圈墙层数）；"
                        "用 09-12 之后的 tools/gate/slicing/slice_l1.py 重切片",
                        provenance=f"printability.yaml:slice_run.parts.{pid}.layers_single_bead")
        else:
            unm = int(srec.get("single_bead_unmeasured_layers") or 0)
            res.add(subject=pid, check="min_wall_sliced",
                    state=PASS if (sb == 0 and unm == 0) else FAIL, severity=BLOCK,
                    measured=sb, evidence_n=int(eo or 0),
                    criterion="真切片（PrusaSlicer 2.9.6，Arachne）：不许有『整层只有一条 bead 的墙』的层。"
                              "只标 External perimeter 的层里，外圈（External + Overhang perimeter）走线总长 / "
                              "该层网格截面周长 < 0.75 判为单 bead（两面各一条 bead ≈ 1.0，整墙一条中线 ≈ 0.5）。"
                              "量不了周长的层计为未测（未测 = 失败）。同层薄厚混杂仍由 min_wall_geometric 兜底。",
                    detail=f"{sb} 个单 bead 层 / {eo} 个只标 External 的层 / {lwp} 个有墙层；未测 {unm}；"
                           f"比值最小 {min(srec.get('single_bead_ratios_ext_only_layers') or [float('nan')])}；"
                           f"本件共 {srec.get('layers')} 层（切片器计数）",
                    provenance=f"printability.yaml:slice_run.parts.{pid}.layers_single_bead")

        # ── 判据 2：最小孔径（从 features.yaml 读，不从网格找孔）
        holes, no_dia = [], []
        for f in feats_all:
            if f.get("part") != pid:
                continue
            k = str(f.get("kind") or "")
            if not any(s in k for s in hole_subs):
                continue
            g = f.get("geom") or {}
            got = False
            for inst in (g.get("instances") or [None]):
                gg = dict(g) if inst is None else {**{x: y for x, y in g.items() if x != "instances"}, **inst}
                d = _v(gg.get("nominal_d_mm"))
                cnt = int(gg.get("count") or 1) if inst is None else 1
                if d is not None:
                    holes.append((f["id"], k, float(d), cnt))
                    got = True
            if not got:
                no_dia.append((f["id"], k, _why(g.get("nominal_d_mm"))))
        if not holes and not no_dia:
            res.add(subject=pid, check="min_hole_dia", state=PASS, severity=BLOCK,
                    measured=None, evidence_n=0,
                    criterion=f"features.yaml 里 kind 含 {hole_subs} 的特征，名义直径 ≥ "
                              f"{hole_mult}×{nozzle}={hole_req:.2f} mm",
                    detail="本件没有声明任何孔类特征", provenance="features.yaml:features[].kind")
        elif no_dia:
            res.unknown(pid, "min_hole_dia",
                        f"{len(no_dia)} 条孔类特征没有名义直径，量不了："
                        + "; ".join(f"{i}({k}) {w[:40]}" for i, k, w in no_dia[:3])
                        + (f"　| 其余 {len(holes)} 条最小 "
                           f"{min(d for _, _, d, _ in holes):.2f} mm" if holes else ""),
                        provenance="features.yaml:features[].geom.nominal_d_mm")
        else:
            bad = [(i, k, d) for i, k, d, _ in holes if d < hole_req]
            dmin = min(d for _, _, d, _ in holes)
            res.add(subject=pid, check="min_hole_dia",
                    state=PASS if not bad else FAIL, severity=BLOCK,
                    measured=_fmt(dmin, 4), evidence_n=sum(c for _, _, _, c in holes),
                    criterion=f"features.yaml 里 kind 含 {hole_subs} 的特征，名义直径 ≥ "
                              f"{hole_mult}×{nozzle}={hole_req:.2f} mm",
                    detail=(f"{len(holes)} 条孔类特征（合计 {sum(c for _,_,_,c in holes)} 个孔），"
                            f"最小 Ø{dmin:.3f}"
                            + (f"；不合格 {[(i, round(d,3)) for i, _, d in bad][:4]}" if bad else "")),
                    provenance="features.yaml:features[].geom.nominal_d_mm")

        # ── 判据 3：悬垂（三角面法向；采样与建模角度无关）
        deg = m_oh if m_oh is not None else oh_deg
        oh_a, tot_a, big_a, nf_oh, nf = _overhang(oriented, deg)
        if m_oh is None:
            res.unknown(pid, "overhang_area",
                        f"材料 '{mat}' 没有悬垂角判据：" + _why(mspec.get("overhang_angle_from_vertical_deg"))
                        + f"　| 按 PLA 的 {oh_deg}° 量：悬垂 {oh_a:.2f} mm²（{100*oh_a/max(tot_a,1e-9):.2f}%），"
                          f"最大连续片 {big_a:.2f} mm²",
                        provenance=f"printability.yaml:materials.{mat}.overhang_angle_from_vertical_deg")
        else:
            res.add(subject=pid, check="overhang_area", state=PASS, severity=INFO,
                    measured=_fmt(oh_a, 2), evidence_n=nf,
                    criterion=f"测量项（不设阈值）：悬垂角 > {deg}° 的面积。"
                              f"悬垂角 = asin(−n_z)，竖直墙 0°、水平天花板 90°。"
                              f"能不能撑得住由 support_reachable / support_in_no_support_zone 判。",
                    detail=(f"悬垂 {oh_a:.3f} mm² / 表面 {tot_a:.3f} mm² = "
                            f"{100*oh_a/max(tot_a,1e-9):.2f}%；最大连续悬垂片 {big_a:.3f} mm²；"
                            f"悬垂面片 {nf_oh}/{nf}"),
                    provenance="printability.yaml:criteria.overhang_angle_from_vertical_deg")

        # ── 判据 4：2D 支撑柱模型（上下界）+ 禁撑区
        # 桥接跨距未知（printability.yaml 已写明），所以不猜，改成夹逼：
        #   hi = 完全不豁免桥接 → 支撑范围上界
        #   lo = 假设桥接跨距无限（凡被下一层四面围住的无支承区都当能桥过去）→ 下界
        # 真实支撑一定在两者之间。
        sm_hi = _support_model(polys, h, sup_deg, min_ow, False)
        sm_lo = _support_model(polys, h, sup_deg, min_ow, True, anchor_sides)
        sm = sm_hi

        # 自检：夹逼区间必须把真切片框住。
        # 关键是**先换算单位再比**：模型算的是支撑柱的**占据体积**，切片器报的是支撑的**挤出体积**，
        # 支撑按 support_material_spacing_mm 稀疏填充，两者差一个填充率 fill。
        # 对同一块支撑区域恒有：占据 × fill ≤ 挤出 ≤ 占据。
        # 从前这里写的是 `lo_占据 ≤ 挤出 ≤ hi_占据`，那是**拿两个不同口径的体积硬比**，
        # 左半边对 9 个件根本不成立（例如 T01 lo_占据 35935 > 挤出 16082、H02 154.9 > 91.4），
        # 而代码其实只做了两条符号比较、从没验证过那个区间 —— measured 在替代码撒谎（元规则 8/9）。
        slicer_sup = srec.get("support_volume_mm3")
        brackets_ok = False          # F-L1-3：下界模型是否被切片器证实；没证实的下界不能单独判红
        if not fresh or slicer_sup is None:
            res.unknown(pid, "support_model_brackets_slicer",
                        "没有对应当前 STL 的切片结果，无法核对支撑模型",
                        provenance=f"printability.yaml:slice_run.parts.{pid}.support_volume_mm3")
        elif not sup_fill or sup_fill <= 0.0:
            res.unknown(pid, "support_model_brackets_slicer",
                        "支撑稀疏填充率算不出来（process.support_line_width_mm / "
                        "support_material_spacing_mm 至少缺一个），模型的『占据体积』就换不成"
                        "切片器的『挤出体积』，两者无从比较 —— 未知=失败，不许只靠符号比较凑绿",
                        provenance="printability.yaml:process.support_line_width_mm / support_material_spacing_mm")
        else:
            real = float(slicer_sup)
            hi_occ = float(sm_hi["occupied_mm3"])
            lo_occ = float(sm_lo["occupied_mm3"])
            lo_ext = lo_occ * sup_fill      # 下界区域按稀疏填充率至少要挤出这么多
            hi_ext = hi_occ                 # 挤出量不可能超过占据量 → 上界区域的挤出上限
            ok_sign_hi = (real <= 0.0) or (hi_occ > 0.0)
            ok_sign_lo = (lo_occ <= 0.0) or (real > 0.0)
            ok_num_hi = real <= hi_ext      # 破了 = sm_hi 根本不是上界
            ok_num_lo = real >= lo_ext      # 破了 = 切片器撑得比下界还少
            bad = [n for n, v in (("符号:真>0⇒hi>0", ok_sign_hi), ("符号:lo>0⇒真>0", ok_sign_lo),
                                  ("数值:真≤hi占据", ok_num_hi),
                                  ("数值:真≥lo占据×fill", ok_num_lo)) if not v]
            brackets_ok = not bad
            res.add(subject=pid, check="support_model_brackets_slicer",
                    state=PASS if not bad else FAIL, severity=BLOCK,
                    measured=(f"挤出真值 {real:.1f} ｜ 换算后区间 [{lo_ext:.1f}, {hi_ext:.1f}] mm³"
                              + ("" if not bad else f" ｜ 破了：{bad}")),
                    evidence_n=n_layers,
                    criterion="模型给的是支撑范围的上下界，必须把真切片框住。**先把口径换成挤出体积再比**："
                              f"支撑是稀疏填充，填充率 fill = support_line_width_mm/support_material_spacing_mm = "
                              f"{sup_lw}/{sup_sp} = {sup_fill:.3f}，对同一块支撑区域恒有 占据×fill ≤ 挤出 ≤ 占据。"
                              "四条同时成立才算过："
                              "①切片器要撑 ⇒ 上界也必须要撑；②下界要撑 ⇒ 切片器必须真的撑了；"
                              "③挤出真值 ≤ **上界占据**（这是『上界之所以是上界』的必要条件 —— 它一破，"
                              "support_in_no_support_zone 那条『上界都干净 → 一定不会落进去』的绿就不能信）；"
                              "④挤出真值 ≥ **下界占据 × fill**（下界区域至少得被撑到，且不比稀疏填充更稀）。"
                              "不做的事：不拿占据体积直接和挤出体积比大小（口径不同），"
                              "也不判两者的比值 —— 支撑接触层是加密的，比值不是常数。",
                    detail=(f"上界占据 {hi_occ:.1f} mm³（接触 {sm_hi['contact_mm2']:.2f} mm²，"
                            f"贴床 {sm_hi['bed_mm2']:.2f} mm²，无支承层 {sm_hi['unsupported_layers']}/{n_layers}）；"
                            f"下界占据 {lo_occ:.1f} mm³（判为可桥接 {sm_lo['bridged_mm2']:.1f} mm²）；"
                            f"切片器挤出 {real:.1f} mm³，桥接挤出 {srec.get('bridge_infill_volume_mm3')} mm³。"
                            f"换算后要求 {lo_ext:.1f} ≤ {real:.1f} ≤ {hi_ext:.1f}。"
                            "注意别照着 lo_占据 ≤ 挤出 ≤ hi_占据 读 —— 那是口径错配："
                            + (f"本件 lo_占据 {lo_occ:.1f} > 挤出真值 {real:.1f}，"
                               f"硬按那个读法比就会造出一条没意义的红（换算后 lo_挤出 = "
                               f"{lo_occ:.1f}×{sup_fill:.3f} = {lo_ext:.1f} 才是能比的那个数）。"
                               if lo_occ > real else
                               f"本件 lo_占据 {lo_occ:.1f} ≤ 挤出真值 {real:.1f}，"
                               f"这一件恰好不冲突，但那是巧合、不是那个读法成立 —— "
                               f"能比的仍然是换算后的 lo_挤出 {lo_ext:.1f}。")
                            + "（这两句是按本件当次算出的 lo_占据 / 挤出真值现写的，不是写死的清单。）"
                            "复现：printability.yaml:slice_run.parts.<件号>.support_volume_mm3 是切片器挤出量；"
                            "lo/hi 由本文件 _support_model(polys, h, support_model_overhang_deg, "
                            "support_min_overhang_width_mm, bridge_exempt=True/False) 现算，"
                            "occupied_mm3 = Σ(柱子截面积)×层高。"),
                    provenance="l1_printable.py:_support_model / printability.yaml:slice_run + "
                               "process.support_line_width_mm / support_material_spacing_mm")

        # 禁撑区：printability.yaml:no_support_zones（forbid / removable 两级）× features.yaml → 本件 zone 清单
        # （no_support_zone_set 与 slicing/slice_l1.py 共用，zones_sha256 由它算；层不再自建柱体清单）
        ZS = no_support_zone_set(P, feats_all, pid)
        zones, unloc = ZS["zones"], ZS["unlocated"]
        cyls = [z for z in zones if z["shape"] == "cyl"]
        boxes = [z for z in zones if z["shape"] == "box"]
        step = _v(crit.get("support_landing_sample_step_mm"))
        forbid_max = _v(crit.get("support_landing_forbid_max_samples"))
        skew = 0
        prepped = []
        zone_by_layer = [None] * n_layers
        if cyls or boxes:
            Rm = np.asarray(R, dtype=float)
            # _orient 已把 z_min 移到 0：同样的平移要加到柱体上
            tz = -float((np.asarray(mesh0.vertices) @ Rm.T)[:, 2].min())
            for z_ in cyls:
                c2 = Rm @ np.asarray(z_["center"], dtype=float)
                c2 = np.array([c2[0], c2[1], c2[2] + tz])
                u2 = Rm @ np.asarray(z_["axis"], dtype=float)
                prepped.append((c2, u2, z_["length"], z_["dia"], z_["tag"]))
            pboxes = []
            for z_ in boxes:
                lo, hi = z_["lo"], z_["hi"]
                corners = np.array([[x, y, zc] for x in (lo[0], hi[0]) for y in (lo[1], hi[1]) for zc in (lo[2], hi[2])])
                cc = corners @ Rm.T
                cc[:, 2] += tz
                pboxes.append((cc.min(axis=0), cc.max(axis=0), z_["tag"]))
            for i, z in enumerate(zs):
                shapes = []
                for c2, u2, L, d, tag in prepped:
                    s = _cyl_section(c2, u2, L, d, float(z), margin)
                    if s is None:
                        continue
                    if s == "SKEW":
                        skew += 1
                        continue
                    shapes.append(s)
                for lo2, hi2, tag in pboxes:
                    s = _box_section(lo2, hi2, float(z), margin)
                    if s is not None:
                        shapes.append(s)
                if shapes:
                    zone_by_layer[i] = unary_union(shapes)

        prov_zone = ("features.yaml + printability.yaml:no_support_zones.tiers | "
                     f"printability.yaml:slice_run.parts.{pid}.support_landing")
        if ZS["problems"] or unloc or skew:
            why_z = ("禁撑区不完整：" + f"{len(unloc)} 条禁撑特征定位不到"
                     + (f"（{[x[0] for x in unloc][:4]}）" if unloc else "")
                     + (f"，{skew} 次斜轴柱体跳过" if skew else "")
                     + (f"；声明问题：{ZS['problems']}" if ZS["problems"] else "")
                     + f"；已建成 {len(cyls)} 个柱体 + {len(boxes)} 个盒。"
                       "圆柱需 geom.axial_span_mm、nominal_d_mm 和全部实例位置；"
                       "非圆柱扫掠（滑配走廊）需 geom.box_export_local {lo,hi}（扫掠体的 export_local 外包盒），不能伪填一个孔径代替；"
                       "配合面壳层 geom.mating_shells_export_local 给了就必须是 [{lo,hi,…}] 且 hi > lo；"
                       "no_support_zones.feature_zones 纳入的实体特征每个实例需 box_export_local / bbox_mm / prism 之一。")
            res.unknown(pid, "support_in_no_support_zone", why_z,
                        provenance="printability.yaml:no_support_zones.geometry_requirements")
            res.unknown(pid, "support_in_removable_zone", why_z, severity=WARN,
                        provenance="printability.yaml:no_support_zones.geometry_requirements")
        elif not zones:
            why_z = ("features.yaml 没给本件声明任何禁撑特征（孔/座/轴颈/滑配走廊），"
                     "printability.yaml:no_support_zones.feature_zones 也没有按特征 id 纳入本件的卡扣/舌片，"
                     "所以『支撑没落进配合面/孔内』这句话无从验证 —— 空清单不是通过")
            res.unknown(pid, "support_in_no_support_zone", why_z,
                        provenance="printability.yaml:no_support_zones.from_features_yaml_kinds")
            res.unknown(pid, "support_in_removable_zone", why_z, severity=WARN,
                        provenance="printability.yaml:no_support_zones.from_features_yaml_kinds")
        else:
            # ① 2D 支撑柱模型对禁撑区的命中（只作交叉核对，不再是判据本体）
            def _hit(model):
                vol, area, lay, who = 0.0, 0.0, 0, {}
                for i in range(n_layers):
                    Z = zone_by_layer[i]
                    if Z is None:
                        continue
                    pieces = [x for x in (model["cross"][i], model["contact"][i]) if x is not None]
                    if not pieces:
                        continue
                    S = unary_union(pieces)
                    inter = S.intersection(Z)
                    if inter.is_empty or inter.area <= 0:
                        continue
                    lay += 1
                    area += inter.area
                    vol += inter.area * h
                    for c2, u2, L, d, tag in prepped:
                        s = _cyl_section(c2, u2, L, d, float(zs[i]), margin)
                        if s is None or s == "SKEW":
                            continue
                        x = S.intersection(s)
                        if not x.is_empty and x.area > 0:
                            who[tag.split("#")[0].split("@")[0]] = \
                                who.get(tag.split("#")[0].split("@")[0], 0.0) + x.area * h
                return vol, area, lay, who
            hi_vol, hi_area, hi_lay, hi_who = _hit(sm_hi)
            lo_vol, lo_area, lo_lay, lo_who = _hit(sm_lo)
            lim = sup_max or 0.0
            n_forbid = sum(1 for z_ in zones if z_["tier"] == "forbid")
            n_rem = len(zones) - n_forbid
            det_model = (f"{len(cyls)} 个禁撑柱体 + {len(boxes)} 个禁撑盒（forbid {n_forbid} / removable {n_rem}）× {n_layers} 层；"
                         f"模型上界命中 {hi_lay} 层 / {hi_area:.4f} mm² / {hi_vol:.4f} mm³；"
                         f"下界命中 {lo_lay} 层 / {lo_area:.4f} mm² / {lo_vol:.4f} mm³")

            # ② 真 G-code 支撑落点（判据本体，09-13 审计 F-L1-3 落地版）
            landing = srec.get("support_landing") if fresh else None
            l_sha = (landing or {}).get("zones_sha256")
            l_step = (landing or {}).get("sample_step_mm")
            crit_main = (
                f"判据来源 = **真 G-code**（printability.yaml:slice_run，{srun.get('slicer', '?')[:18]}…）："
                "解析 ;TYPE:Support material / Support material interface 的挤出段（G1 带 E 且 XY 有位移），"
                f"沿每段每 {step} mm 取一点（含端点，z 取该层 ;Z: 即层顶），按 slice_run 记录的 align_vectors 矩阵与平移"
                "逆变换回 export_local，逐个禁撑区数落进去的点。"
                f"**forbid 级**（kind ∈ {ZS['tiers']['forbid']} + 滑配走廊配合面壳层 mating_shells_export_local；"
                f"柱体两端外扩 {margin} mm）落点数必须 ≤ {forbid_max}；measured = 落点折算路径 mm（点数×{step}，逐区求和，"
                "区重叠处按区各计）。记录必须同时对应当前 STL（source_sha256）与当前禁撑区声明（zones_sha256），"
                "否则 unknown（重跑 tools/gate/slicing/slice_l1.py）。removable 级落点另见 support_in_removable_zone（WARN）。")
            if step is None or forbid_max is None:
                why_z = ("printability.yaml:criteria 缺 support_landing_sample_step_mm / support_landing_forbid_max_samples"
                         "（采样步长与 forbid 级允许点数），G-code 落点无从判。" + det_model)
                res.unknown(pid, "support_in_no_support_zone", why_z, provenance="printability.yaml:criteria")
                res.unknown(pid, "support_in_removable_zone", why_z, severity=WARN, provenance="printability.yaml:criteria")
            elif not landing:
                why_z = ("没有对应当前 STL 的 G-code 支撑落点记录（slice_run.parts.<件>.support_landing 缺失或切片记录过期）"
                         "—— 2D 模型只作交叉核对、不再单独判红绿。要解开：./.venv/bin/python tools/gate/slicing/slice_l1.py。" + det_model)
                res.unknown(pid, "support_in_no_support_zone", why_z, provenance=prov_zone)
                res.unknown(pid, "support_in_removable_zone", why_z, severity=WARN, provenance=prov_zone)
            elif l_sha != ZS["sha256"]:
                why_z = (f"禁撑区声明变了没重切：记录 zones_sha256 {str(l_sha)[:12]}… ≠ 现算 {ZS['sha256'][:12]}…"
                         "（features.yaml 禁撑特征 / printability.yaml 分级 / 轴向外扩改过）。要解开：重跑 slice_l1.py。" + det_model)
                res.unknown(pid, "support_in_no_support_zone", why_z, provenance=prov_zone)
                res.unknown(pid, "support_in_removable_zone", why_z, severity=WARN, provenance=prov_zone)
            elif l_step is None or abs(float(l_step) - float(step)) > 1e-9:
                why_z = f"记录的采样步长 {l_step} ≠ criteria 的 {step}，落点数不可比。要解开：重跑 slice_l1.py。" + det_model
                res.unknown(pid, "support_in_no_support_zone", why_z, provenance=prov_zone)
                res.unknown(pid, "support_in_removable_zone", why_z, severity=WARN, provenance=prov_zone)
            else:
                per = landing.get("per_zone") or {}
                hits = {"forbid": [], "removable": []}
                for z_ in zones:
                    r_ = per.get(z_["tag"]) or {}
                    n_ = int(r_.get("samples") or 0)
                    if n_ > 0:
                        hits[z_["tier"]].append((z_, n_, float(r_.get("path_mm") if r_.get("path_mm") is not None else n_ * step)))
                f_n = sum(n_ for _, n_, _ in hits["forbid"]); f_mm = sum(mm for _, _, mm in hits["forbid"])
                r_n = sum(n_ for _, n_, _ in hits["removable"]); r_mm = sum(mm for _, _, mm in hits["removable"])
                tot = int(landing.get("samples_total") or 0)
                head = (f"支撑采样点 {tot}（支撑路径 {landing.get('support_path_mm')} mm，切片实参 "
                        f"{srec.get('slicer_argv_extra') or '默认'}）；{len(zones)} 个禁撑区（forbid {n_forbid} / removable {n_rem}）；")
                top_f = sorted(hits["forbid"], key=lambda t: -t[1])[:6]
                res.add(subject=pid, check="support_in_no_support_zone",
                        state=FAIL if f_n > forbid_max else PASS, severity=BLOCK,
                        measured=_fmt(f_mm, 3), evidence_n=len(zones),
                        criterion=crit_main,
                        detail=head + f"forbid 级落点 {f_n} 点 / {f_mm:.1f} mm"
                               + ("：" + "，".join(f"{z_['tag']}[{z_['kind']}{'·配合面壳层' if z_['role']=='mating_shell' else ''}] "
                                                 f"{n_} 点/{mm:.1f} mm" for z_, n_, mm in top_f) if top_f else "（无）")
                               + "；" + det_model,
                        provenance=prov_zone)
                top_r = sorted(hits["removable"], key=lambda t: -t[1])[:8]
                res.add(subject=pid, check="support_in_removable_zone",
                        state=FAIL if r_n > 0 else PASS, severity=WARN,
                        measured=_fmt(r_mm, 3), evidence_n=len(zones),
                        criterion=("同 support_in_no_support_zone 的 G-code 采样；落进 **removable 级**"
                                   f"（kind ∈ {ZS['tiers']['removable']}，含 slide_corridor 口袋盒的非配合面体积；"
                                   "per_feature_overrides 可改级）的采样点 > 0 → WARN：不阻断，但 detail 里逐特征的"
                                   "后处理文本（printability.yaml:no_support_zones.tiers.removable.post_process / overrides）"
                                   "必须进打印清单后处理栏。measured = 落点折算路径 mm。"),
                        detail=head + f"removable 级落点 {r_n} 点 / {r_mm:.1f} mm"
                               + ("；后处理：" + "；".join(
                                   f"{z_['tag']}[{z_['kind']}] {n_} 点/{mm:.1f} mm → {z_.get('post_process') or '（post_process 未写）'}"
                                   for z_, n_, mm in top_r) if top_r else "（无）"),
                        provenance=prov_zone)
                # ③ 模型 vs G-code 交叉核对：上界=0 却有真落点 → 模型漏了（WARN）；其余只报数（INFO）
                missed = (hi_vol <= lim) and (f_n + r_n > 0)
                res.add(subject=pid, check="support_model_zone_crosscheck",
                        state=FAIL if missed else PASS, severity=WARN if missed else INFO,
                        measured=f"模型上界 {hi_vol:.4f} mm³ / G-code {f_n + r_n} 点", evidence_n=n_layers,
                        criterion=(f"2D 支撑柱模型（上界不豁免桥接 / 下界桥接无限）对禁撑区的命中与真 G-code 落点交叉核对："
                                   f"上界 ≤ {lim} mm³（判『一定干净』）而 G-code 有任何落点 → 模型漏算，FAIL(WARN)；"
                                   "其余组合只报数（INFO）。本判据不替代 support_in_no_support_zone。"),
                        detail=det_model + f"；G-code forbid {f_n} 点 / removable {r_n} 点"
                               + (f"；模型下界落在 " + ", ".join(f"{k} {v:.3f} mm³" for k, v in sorted(lo_who.items(), key=lambda kv: -kv[1])[:4])
                                  if lo_who else ""),
                        provenance="l1_printable.py:_support_model + " + prov_zone)

        # ── 判据 4b：支撑可拆区（README『悬垂 >55° 的面在支撑可拆区』）
        if m_rem_v is None:
            res.unknown(pid, "support_reachable",
                        f"材料 '{mat}' 的支撑可拆性没有实测："
                        + _why(mspec.get("supports_removable")),
                        provenance=f"printability.yaml:materials.{mat}.supports_removable")
        else:
            enc_vol, enc_layers = 0.0, 0
            for i in range(n_layers):
                S = sm["cross"][i]
                p = polys[i]
                if S is None or p is None:
                    continue
                ring = _interior_rings(p)
                if ring is None:
                    continue
                x = S.intersection(ring)
                if not x.is_empty and x.area > 0:
                    enc_layers += 1
                    enc_vol += x.area * h
            res.add(subject=pid, check="support_reachable",
                    state=PASS if enc_vol <= 0.0 else FAIL, severity=WARN,
                    measured=_fmt(enc_vol, 4), evidence_n=n_layers,
                    criterion="支撑不许落在截面的封闭内环里（那种支撑拆不出来）。"
                              "用的是**上界**支撑模型（sm_hi，不做任何桥接豁免 → 多算支撑），"
                              "所以它也是往多里报的一侧。"
                              "本判据是**逐层**的必要条件：某几层被内环围住、别的层通到外面的柱子仍可能拆得掉，"
                              "所以它会多报不会少报 → 定为 WARN，红了要人看一眼，不直接卡。",
                    detail=f"被封闭内环围住的支撑 {enc_vol:.4f} mm³，涉及 {enc_layers}/{n_layers} 层",
                    provenance="l1_printable.py:_interior_rings / README 第 2 节 L1")

        # ── 判据 5：首层贴床面积（只报数，阈值在 _layer 那条 unknown 里）
        first = polys[0]
        fa = float(first.area) if first is not None else 0.0
        res.add(subject=pid, check="first_layer_area",
                state=PASS if fa > 0 else FAIL, severity=INFO,
                measured=_fmt(fa, 3), evidence_n=1,
                criterion=("测量项：z_min 那一层的截面积。"
                           + (f"阈值 {first_min} mm²" if first_min is not None
                              else "阈值 = null（printability.yaml 里已写明缺什么才能定阈值），"
                                   "所以这里只报数、不判合格")),
                detail=(f"首层 z={zs[0]:.2f} mm，贴床截面 {fa:.3f} mm²；"
                        f"另有 {_v(proc.get('brim_width_mm'))} mm 裙边未计入；"
                        f"件总高 {oriented.bounds[1,2]:.3f} mm / {n_layers} 层"),
                provenance="printability.yaml:criteria.first_layer_min_area_mm2")

        checked.append(pid)
        res.covered.add(pid)

    # 件覆盖率（core.py:COVERAGE_ITEMS 目前只登记 L2/L4，这里自建一格）
    want = [p for p in ctx.parts if not ctx.only or p in ctx.only]
    miss = sorted(set(want) - set(checked))
    res.add(subject="_coverage", check="件覆盖率",
            state=PASS if not miss else FAIL, severity=BLOCK,
            measured=f"{len(checked)}/{len(want)}", evidence_n=len(checked),
            criterion="parts.yaml 声明的每个打印件都必须走完本层的全部判据",
            detail="全覆盖" if not miss else f"没走完：{miss}",
            provenance="tools/gate/README.md 第 2 节 L1")

    if not P.get("_calibration_inner_run"):
        _calibration(res, ctx, P)

    res.evidence = {"parts_checked": len(checked),
                    # 这是**全局读法**（criteria.min_wall_perimeters × wall_line_width_mm）的数，
                    # 只是层级摘要。逐件判红实际用的是 materials.<该件材料>.min_wall_perimeters；
                    # 材料给 null 的件（TPU 95A）判 unknown，不套这个数。别读成"每件都按这个判过"。
                    "wall_criterion_mm_global_fallback": round(wall_req, 4),
                    "hole_criterion_mm": round(hole_req, 4),
                    "support_model_deg": sup_deg,
                    "sliced_with": srun.get("slicer")}
    return res
