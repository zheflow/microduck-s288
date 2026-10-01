#!/usr/bin/env python
"""clean_check_holes —— 螺丝孔 / 沉窝侧壁被切开一部分（部分开口）+ 孔沿 / 圆环被径向切穿（开口环）的纯几何检测。

给 tools/cad/clean_check.py 调：
    from clean_check_holes import check_holes
    res = check_holes(mesh, contains=None, params=None, screw_features=None)
单独跑：
    ./.venv/bin/python -B tools/cad/clean_check_holes.py a.stl b.stl --out x.json [--features hole_features_world.json] [--params '{"k": v}']

输入
  mesh            trimesh.Trimesh，件在自己文件坐标系里的网格（没合并顶点的会先复制一份合并）。
  contains        contains(points (N,3)) -> bool (N,)；不传用 tools/cad/ray_backend.py 的 RB.contains（Embree）。
                  本模块不 import manifold3d、不调 trimesh 布尔（embreex 与 manifold3d 两份 TBB 同进程会崩）。
  params          覆盖 DEFAULTS 里的阈值（返回的 params 是实际用的全部阈值）。
  screw_features  可选。{"own": [...本件 features...], "other": [...别的件 features...]}；也接受一个列表（params["placed"] 给了就按
                  placed 字段分本件 / 别件，否则全当本件）。每条 {id, kind, purpose, a, b（轴线两端点，与 mesh 同一坐标系）, d, part?, placed?}。
                  不传 → 每个孔 use = {"match": "n/a"}（原版件）。

方法
  1 圆柱识别（不依赖世界轴）：取相邻面夹角 ang_min..ang_max 的棱边（CAD 网格的圆柱母线棱，实测棱边方向与 n1×n2 偏差 <0.1°），
    每条边 → 轴向 = 棱边方向、轴心 = 棱边端点与两个对顶点沿轴投影后的外接圆心、半径；两面法线都指向轴心 = 孔（凸台不要）。
    KD 树按（a·aᵀ 6 维【与轴正负号无关】、轴线相对件中心的垂足、log r）单链聚类成种子 → 同轴同径的种子合并 →
    收孔壁面（法线 ⟂ 轴、指向轴、三顶点到轴距离 = r）→ 用收到的面重拟合（轴 = Σ面积·nnᵀ 最小特征向量、分箱拟圆修正斜轴、半径对 t 线性 = 锥度）
    → 几乎同轴的碎片合并（被位移场拉歪的残孔常被拆成两半）→ 沿轴按空档分段 → 方位覆盖 cov（孔壁面方位区间并集 / 360°）。
    保留 Ø d_min..d_ring_max、长 ≥ len_min、cov ≥ cov_min 的；Ø ≤ d_hole_max 的进"孔"表（kind：≤ d_screw_max = screw，否则 cbore）。
  2 逐层扫描：沿轴每 step 取一层（两端各外延 margin），每层 n_ring 个方位；径向取 ring_offs（孔壁外 0.05 / 0.3 / 0.55 / 0.8 / 1.0），
    孔心 / 半径先用逐层局部拟圆（孔壁面与截平面求交拟圆）；拟合残差 > warp_rms 的歪孔改用截面真实孔洞轮廓往外偏（shapely buffer）。
  3 部分开口（只看 Ø ≤ d_hole_max）：f = r+ring_off 圈上不在实体里的比例；f_lo ≤ f ≤ f_hi 为部分开口层，f > f_hi 为孔口 / 通腔。
    连续部分开口层成一段，按下面顺序定模式（partial_runs 的说明）：mid / exit_short / mouth_irregular / exit_slanted / side_open。
    报（flag）：mid、mouth_irregular（原版 32 件 256 孔里 0 处）；side_open 默认不报（flag_side_open=True 才报：原版 25 段设计性侧开口与 L04#12 同形）。
  4 开口环（Ø d_min..d_ring_max 的孔 / 环形座都看）：某层某方位从孔壁（r+0.05）到 r+1.0 全不在实体里 = 该方位被径向切穿；
    整圈孔壁都没料的层是孔口，不算。相邻层方位重叠的缺口连成一个断口，按 where（mouth / through / mid）与宽度分类（classify_break）。
    报：through 且断口宽度中位数 ≤ ring_narrow_deg（窄缝贯通，环没闭合）。通到哪里（leads_to）：断口中层作垂直孔轴截面，
    孔心空区不在任何多边形外轮廓内 → "板边/外轮廓"；落在比孔大很多的内环里 → "另一个孔或腔"；否则 "连通外部"。
  5 用途（use）：按 features 同轴匹配（match_use 的说明）→ own / channel / none / n/a。

返回
  {"status": "PASS"/"FAIL",   # 有 flag 孔或 flag 开口环 → FAIL
   "params": {...},
   "holes": [ {id, kind, d_mm, axis, center, extent_mm（相对 center 沿轴）, ends, cov, taper, fit_rms, warped,
               n_layers, step_mm, n_partial_layers, max_partial_frac, partial_len_mm, partial_where（mouth/mid）, partial_pattern,
               partial_bbox（开口段缺失点包围盒，文件坐标）, flag, reason, use, runs[...] } ],
   "findings": [flag 的孔],      # 孔 id 与 open_rings 的 hole_id 同一编号（按直径排序）；Ø>d_hole_max 的只出现在 open_rings，所以 holes 里 id 可能跳号
   "open_rings": [ {hole_id, kind（screw/cbore/bore）, d_mm, axis, center, ..., where, pattern, width_deg（最宽层）, width_deg_median,
                    width_mm_at_r05, az_deg, len_mm, n_layers, t, bbox, widths, before, after, leads_to, via, flag, reason, use} ],
   "open_ring_findings": [flag 的开口环],
   "stats": {n_holes, n_flagged, n_flagged_by_use{own,channel,none,n/a}, n_open_rings, n_open_rings_flagged,
             n_open_rings_flagged_by_use, run_patterns, seconds, seconds_detect, faces, ...}}
"""
import os
import sys
import time
import json

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.normpath(os.path.join(_HERE, "..", ".."))

DEFAULTS = dict(
    ang_min_deg=1.0, ang_max_deg=50.0,      # 圆柱棱边：相邻面法线夹角窗口
    r_min=0.55, r_max=15.5,                  # 候选半径（Ø1.1..Ø31）
    dir_tol_deg=2.0, ctr_tol=0.25, logr_tol=0.06,   # 种子聚类容差
    min_edges=4,
    face_dir_tol_deg=1.5, face_r_tol=0.03, face_r_tol_rel=0.01,   # 收面：法线 ⟂ 轴、三顶点到轴距离 = r
    cov_min=0.40, gap_tol=0.3, len_min=0.3,
    d_min=1.2, d_screw_max=2.9, d_hole_max=6.5, d_ring_max=30.0,
    step=0.2, ring_off=0.3, n_ring=72, margin=0.5, max_layers=200,
    f_lo=0.10, f_hi=0.90,
    short_layers=3, az_dev_max=25.0, mono_tol=0.05, exit_edge_f=0.70, flag_side_open=False,
    open_rings=True, ring_d_min=1.2, ring_offs=(0.05, 0.3, 0.55, 0.8, 1.0), az_arc=0.4, break_min_pts=2,
    merge_area_ratio=1.3, ring_leads_all=False, leads_max_mm=60.0, leads_step_mm=0.5,
    local_fit=True, local_win=0.35, warp_rms=0.005,
    ring_narrow_deg=60.0, ring_flag_mid=False, contains_chunk=25000,
    use_ang_deg=5.0, use_dist_mm=0.5, use_dd_mm=0.3, use_dd_rel=0.10, channel_max_gap=40.0,
)


def _perp_basis(a):
    ref = np.eye(3)[int(np.argmin(np.abs(a)))]
    u = np.cross(a, ref); u /= np.linalg.norm(u)
    w = np.cross(a, u)
    return u, w


# ───────────────────────────── 圆柱识别 ─────────────────────────────
def edge_cylinders(mesh, P):
    """每条圆柱棱边 → (轴向 a, 轴心 c, 半径 r, 孔/凸台)。只取相邻面夹角在窗口内的边。
    轴向 = 棱边方向（= n1×n2，CAD 网格圆柱棱边与母线平行，实测偏差 <0.1°）；
    轴心 = 棱边端点 + 两个对顶点沿轴投影后三点的外接圆心；孔 = 两面法线都指向轴心。"""
    ang = mesh.face_adjacency_angles
    amin, amax = np.radians(P["ang_min_deg"]), np.radians(P["ang_max_deg"])
    sel = np.flatnonzero((ang >= amin) & (ang <= amax))
    adj = mesh.face_adjacency[sel]
    ev = mesh.face_adjacency_edges[sel]
    un = mesh.face_adjacency_unshared[sel]
    ok = (un >= 0).all(1)
    sel, adj, ev, un = sel[ok], adj[ok], ev[ok], un[ok]
    V = mesh.vertices
    p0, p1 = V[ev[:, 0]], V[ev[:, 1]]
    a = p1 - p0
    L = np.linalg.norm(a, axis=1)
    good = L > 1e-6
    a = a / np.where(good, L, 1.0)[:, None]
    b = V[un[:, 0]] - p0
    c = V[un[:, 1]] - p0
    b -= (b * a).sum(1)[:, None] * a
    c -= (c * a).sum(1)[:, None] * a
    bxc = np.cross(b, c)
    den = 2.0 * (bxc * bxc).sum(1)
    good &= den > 1e-14
    bb = (b * b).sum(1)[:, None]; cc2 = (c * c).sum(1)[:, None]
    cen = np.cross(bb * c - cc2 * b, bxc) / np.where(good, den, 1.0)[:, None]
    r = np.linalg.norm(cen, axis=1)
    n1 = mesh.face_normals[adj[:, 0]]; n2 = mesh.face_normals[adj[:, 1]]
    s1 = (n1 * cen).sum(1); s2 = (n2 * cen).sum(1)
    hole = (s1 > 0) & (s2 > 0)
    boss = (s1 < 0) & (s2 < 0)
    keep = good & (hole | boss) & (r >= P["r_min"]) & (r <= P["r_max"])
    return dict(a=a[keep], c=(p0 + cen)[keep], r=r[keep], hole=hole[keep],
                p0=p0[keep], p1=p1[keep], L=L[keep], ang=ang[sel][keep])


def seed_clusters(E, origin, P, want_hole=True):
    """KD 树按（轴向 a·aᵀ 6 维【与正负号无关】、轴线相对件中心的垂足、log r）单链聚类。"""
    from scipy.spatial import cKDTree
    from scipy.sparse import coo_matrix
    from scipy.sparse.csgraph import connected_components
    idx = np.flatnonzero(E["hole"] == want_hole)
    if len(idx) == 0:
        return []
    a = E["a"][idx]; c = E["c"][idx] - origin; r = E["r"][idx]
    emb_a = np.column_stack([a[:, 0] ** 2, a[:, 1] ** 2, a[:, 2] ** 2,
                             np.sqrt(2) * a[:, 0] * a[:, 1], np.sqrt(2) * a[:, 0] * a[:, 2], np.sqrt(2) * a[:, 1] * a[:, 2]])
    cp = c - (c * a).sum(1)[:, None] * a
    s_a = 1.0 / (np.sqrt(2) * np.sin(np.radians(P["dir_tol_deg"])))
    X = np.column_stack([emb_a * s_a, cp / P["ctr_tol"], np.log(r)[:, None] / P["logr_tol"]])
    pairs = cKDTree(X).query_pairs(1.0, output_type="ndarray")
    n = len(idx)
    G = coo_matrix((np.ones(len(pairs)), (pairs[:, 0], pairs[:, 1])), shape=(n, n)) if len(pairs) else coo_matrix((n, n))
    ncomp, lab = connected_components(G, directed=False)
    cnt = np.bincount(lab, minlength=ncomp)
    order = np.argsort(lab, kind="stable")
    bnd = np.searchsorted(lab[order], np.arange(ncomp + 1))
    return [idx[order[bnd[k]:bnd[k + 1]]] for k in range(ncomp) if cnt[k] >= P["min_edges"]]


def _circle_fit(x, y):
    A = np.column_stack([x, y, np.ones_like(x)])
    sol = np.linalg.lstsq(A, -(x * x + y * y), rcond=None)[0]
    cx, cy = -sol[0] / 2, -sol[1] / 2
    return cx, cy, float(np.sqrt(max(cx * cx + cy * cy - sol[2], 1e-12)))


def fit_seed(E, mem, P):
    """种子聚类 → 轴向（加权主方向）+ 截面圆（代数拟合，剔除离群边）；pt = 种子段中点处的轴心。"""
    a = E["a"][mem]; w = E["L"][mem]
    M = (a[:, :, None] * a[:, None, :] * w[:, None, None]).sum(0)
    ax = np.linalg.eigh(M)[1][:, -1]
    mid = 0.5 * (E["p0"][mem] + E["p1"][mem])
    u, v = _perp_basis(ax)
    x_all, y_all = mid @ u, mid @ v
    keep = np.ones(len(mem), bool)
    for _ in range(4):
        cx, cy, rr = _circle_fit(x_all[keep], y_all[keep])
        res = np.abs(np.hypot(x_all - cx, y_all - cy) - rr)
        nk = res <= max(0.03, 0.02 * rr)
        if nk.sum() < P["min_edges"] or (nk == keep).all():
            break
        keep = nk
    t = np.concatenate([E["p0"][mem][keep] @ ax, E["p1"][mem][keep] @ ax])
    tm = 0.5 * (t.min() + t.max())
    pt = cx * u + cy * v + tm * ax
    return dict(axis=ax, pt=pt, r=rr, t0=float(t.min() - tm), t1=float(t.max() - tm), n=int(keep.sum()))


def _same_line(f, g, P, r):
    if abs(f["axis"] @ g["axis"]) < np.cos(np.radians(P["dir_tol_deg"])):
        return False
    d1 = g["pt"] - f["pt"]; d1 = np.linalg.norm(d1 - (d1 @ f["axis"]) * f["axis"])
    d2 = f["pt"] - g["pt"]; d2 = np.linalg.norm(d2 - (d2 @ g["axis"]) * g["axis"])
    return max(d1, d2) <= 0.1 + 0.02 * r


def merge_seeds(fits, P):
    """同轴线 + 同半径的种子并成一组（切口边上的离群估计会把一个圆柱拆成几组种子）。"""
    fits = sorted(fits, key=lambda f: -f["n"])
    used = [False] * len(fits)
    out = []
    for i, f in enumerate(fits):
        if used[i]:
            continue
        grp = [f]; used[i] = True
        for j in range(i + 1, len(fits)):
            if used[j]:
                continue
            g = fits[j]
            if abs(f["r"] - g["r"]) > 0.05 + 0.02 * f["r"] or not _same_line(f, g, P, f["r"]):
                continue
            grp.append(g); used[j] = True
        out.append(grp)
    return out


def collect_faces(mesh, tree, ax, pt, r, k, t0, t1, P, rtol=None, dir_deg=None):
    """圆柱 / 小锥度圆锥面上的三角面：法线 ⟂ 轴（容差 dir_deg）、法线指向轴（孔）、三个顶点到轴距离 = r + k·t（容差 rtol）。
    t 相对 pt 沿轴；沿轴迭代扩展查询范围（孔可能比种子段长）。"""
    tri = mesh.triangles
    fn = mesh.face_normals
    sin_tol = np.sin(np.radians(P["face_dir_tol_deg"] if dir_deg is None else dir_deg))
    if rtol is None:
        rtol = P["face_r_tol"] + P["face_r_tol_rel"] * r
    lo, hi = t0 - 1.0, t1 + 1.0
    fids = np.array([], dtype=np.int64)
    for _ in range(8):
        mid = pt + ax * (0.5 * (lo + hi))
        rad = np.hypot(0.5 * (hi - lo), r + abs(k) * (hi - lo) + 1.0)
        cand = np.asarray(tree.query_ball_point(mid, rad), dtype=np.int64)
        if len(cand) == 0:
            break
        n = fn[cand]
        okn = (np.abs(n @ ax) < sin_tol) & ((n * n).sum(1) > 0.5)
        cand = cand[okn]
        T = tri[cand] - pt                       # (k,3,3)
        tt = T @ ax                              # (k,3)
        perp = T - tt[..., None] * ax
        dist = np.linalg.norm(perp, axis=2)
        okr = (np.abs(dist - (r + k * tt)) <= rtol).all(1)
        pm = perp.mean(1)
        pmn = np.linalg.norm(pm, axis=1)
        okd = (fn[cand] * (-pm)).sum(1) > 0.9 * pmn            # 法线指向轴（与径向夹角 < 26°）= 孔
        sel = okr & okd
        fids = cand[sel]
        if len(fids) == 0:
            break
        tmin, tmax = float(tt[sel].min()), float(tt[sel].max())
        grow = False
        if tmin < lo + 0.5:
            lo = tmin - 3.0; grow = True
        if tmax > hi - 0.5:
            hi = tmax + 3.0; grow = True
        if not grow:
            break
    return fids


def refit_cone(mesh, fids, ax0):
    """用收到的面重拟合轴与截面：轴向初值 = Σ面积·n·nᵀ 的最小特征向量；再沿轴 1 mm 分箱逐箱拟圆，
    圆心连线修正轴（斜孔 / 被位移场拉斜的残孔），半径对 t 线性拟合（锥度）。返回 (ax, pt(段中点), r(段中点), k=dr/dt, 残差 RMS)。"""
    n = mesh.face_normals[fids]; A = mesh.area_faces[fids]
    M = (n[:, :, None] * n[:, None, :] * A[:, None, None]).sum(0)
    w, V = np.linalg.eigh(M)
    ax = V[:, 0]
    if w[1] < 1e-6 * max(w[2], 1e-12) or abs(ax @ ax0) < np.cos(np.radians(12.0)):
        ax = ax0                                        # 弧太短定不了轴 → 沿用种子轴
    if ax @ ax0 < 0:
        ax = -ax
    P3 = mesh.vertices[np.unique(mesh.faces[fids])]
    k = 0.0
    for _ in range(2):
        u, v = _perp_basis(ax)
        x, y, t = P3 @ u, P3 @ v, P3 @ ax
        tlo, thi = t.min(), t.max()
        nb = int(max(1, np.floor((thi - tlo) / 1.0)))
        cs, rs, ts = [], [], []
        if nb >= 2:
            edges = np.linspace(tlo, thi + 1e-9, nb + 1)
            for b in range(nb):
                m = (t >= edges[b]) & (t < edges[b + 1])
                if m.sum() < 5:
                    continue
                cx, cy, rr = _circle_fit(x[m], y[m])
                ang = np.arctan2(y[m] - cy, x[m] - cx)
                span = 2 * np.pi - np.max(np.diff(np.sort(np.r_[ang, ang.min() + 2 * np.pi])))
                if span < np.radians(120):
                    continue
                cs.append((cx, cy)); rs.append(rr); ts.append(0.5 * (edges[b] + edges[b + 1]))
        if len(ts) >= 2 and (max(ts) - min(ts)) >= 2.0:
            ts = np.asarray(ts); cs = np.asarray(cs); rs = np.asarray(rs)
            ex = np.polyfit(ts, cs[:, 0], 1)[0]; ey = np.polyfit(ts, cs[:, 1], 1)[0]
            k = float(np.polyfit(ts, rs, 1)[0])
            nax = ax + ex * u + ey * v
            nax /= np.linalg.norm(nax)
            if abs(nax @ ax) > np.cos(np.radians(15.0)):
                ax = nax
                continue
        break
    u, v = _perp_basis(ax)
    x, y, t = P3 @ u, P3 @ v, P3 @ ax
    tm = 0.5 * (t.min() + t.max())
    # 锥：先按 k 把各点半径折算到 tm，再整体拟圆
    if abs(k) > 1e-4:
        cx, cy, _ = _circle_fit(x, y)
        for _ in range(3):
            rho = np.hypot(x - cx, y - cy)
            A_ = np.column_stack([np.ones_like(t), t - tm])
            r_mid, k = np.linalg.lstsq(A_, rho, rcond=None)[0]
            sc = r_mid / np.maximum(r_mid + k * (t - tm), 1e-6)
            cx, cy, _ = _circle_fit(cx + (x - cx) * sc, cy + (y - cy) * sc)
        r = float(r_mid)
    else:
        cx, cy, r = _circle_fit(x, y)
        k = 0.0
    rho = np.hypot(x - cx, y - cy)
    res = float(np.sqrt(np.mean((rho - (r + k * (t - tm))) ** 2)))
    return ax, cx * u + cy * v + tm * ax, r, float(k), res


def face_intervals(mesh, fids, ax, pt, u, v):
    T = mesh.triangles[fids] - pt
    tt = T @ ax
    phi = np.degrees(np.arctan2(T @ v, T @ u)) % 360.0     # (k,3)
    ps = np.sort(phi, axis=1)
    gaps = np.column_stack([ps[:, 1] - ps[:, 0], ps[:, 2] - ps[:, 1], ps[:, 0] + 360 - ps[:, 2]])
    g = np.argmax(gaps, axis=1)
    start = np.where(g == 0, ps[:, 1], np.where(g == 1, ps[:, 2], ps[:, 0]))
    width = 360.0 - gaps[np.arange(len(g)), g]
    return tt.min(1), tt.max(1), start, width


def coverage(start, width, nb=720):
    bins = np.zeros(nb, bool)
    s = np.floor(start / 360.0 * nb).astype(int)
    e = np.ceil((start + width) / 360.0 * nb).astype(int)
    for a, b in zip(s, e):
        if b - a >= nb:
            bins[:] = True
            break
        bins[np.arange(a, b) % nb] = True
    return float(bins.mean())


def _fit_group(mesh, tree, grp, P):
    f = grp[0]
    ax, pt, r = f["axis"], f["pt"], f["r"]
    ts = []
    for g in grp:
        s = 1.0 if g["axis"] @ ax > 0 else -1.0
        base = (g["pt"] - pt) @ ax
        ts += [base + s * g["t0"], base + s * g["t1"]]
    t0, t1 = min(ts), max(ts)
    fids = collect_faces(mesh, tree, ax, pt, r, 0.0, t0, t1, P)
    if len(fids) < 3:
        return None
    k, res = 0.0, 0.0
    for it in range(2):
        ax2, pt2, r2, k2, res2 = refit_cone(mesh, fids, ax)
        if abs(r2 - r) > 0.15 + 0.1 * r:
            break
        tt = (mesh.triangles[fids] - pt2) @ ax2
        rt = P["face_r_tol"] + P["face_r_tol_rel"] * r2 + (2.5 * res2 if res2 > 0.01 else 0.0)
        fids2 = collect_faces(mesh, tree, ax2, pt2, r2, k2, float(tt.min()), float(tt.max()), P,
                              rtol=min(rt, 0.12 + 0.03 * r2), dir_deg=P["face_dir_tol_deg"] + (3.0 if res2 > 0.01 else 0.0))
        if len(fids2) < 0.8 * len(fids):
            break
        grew = len(fids2) > len(fids)
        ax, pt, r, k, res, fids = ax2, pt2, r2, k2, res2, fids2
        if not grew:
            break
    return ax, pt, r, k, res, fids


def merge_coaxial(mesh, tree, raw, P):
    """几乎同轴、半径接近、沿轴重叠的几个候选（被位移场拉歪的残孔常被拆成两半各拟一个圆柱）→ 合并面重拟合；覆盖变好才采纳。"""
    if len(raw) < 2:
        return raw
    raw = sorted(raw, key=lambda x: -len(x[5]))
    used = [False] * len(raw)
    out = []
    for i in range(len(raw)):
        if used[i]:
            continue
        ax, pt, r, k, res, fids, ns = raw[i]
        grp = [i]
        for j in range(i + 1, len(raw)):
            if used[j]:
                continue
            ax2, pt2, r2, k2, res2, fids2, ns2 = raw[j]
            if abs(ax @ ax2) < np.cos(np.radians(6.0)) or abs(r - r2) > 0.25 + 0.05 * r:
                continue
            d1 = pt2 - pt; d1 = np.linalg.norm(d1 - (d1 @ ax) * ax)
            d2 = pt - pt2; d2 = np.linalg.norm(d2 - (d2 @ ax2) * ax2)
            if max(d1, d2) > 0.35 + 0.05 * r:
                continue
            ta = (mesh.triangles[fids] - pt) @ ax; tb = (mesh.triangles[fids2] - pt) @ ax
            if tb.min() > ta.max() + 0.3 or tb.max() < ta.min() - 0.3:
                continue
            grp.append(j)
        if len(grp) == 1:
            used[i] = True
            out.append(raw[i])
            continue
        allf = np.unique(np.concatenate([raw[j][5] for j in grp]))
        ax3, pt3, r3, k3, res3 = refit_cone(mesh, allf, ax)
        tt = (mesh.triangles[allf] - pt3) @ ax3
        rt = min(P["face_r_tol"] + P["face_r_tol_rel"] * r3 + 2.5 * res3, 0.12 + 0.03 * r3)
        f3 = collect_faces(mesh, tree, ax3, pt3, r3, k3, float(tt.min()), float(tt.max()), P, rtol=rt,
                           dir_deg=P["face_dir_tol_deg"] + 3.0)
        u, v = _perp_basis(ax3)
        def cov_of(fs, a_, p_):
            uu, vv = _perp_basis(a_)
            _, _, st_, wd_ = face_intervals(mesh, fs, a_, p_, uu, vv)
            return coverage(st_, wd_)
        best = max(cov_of(raw[j][5], raw[j][0], raw[j][1]) for j in grp)
        if len(f3) >= 3 and cov_of(f3, ax3, pt3) >= best + 0.05:
            for j in grp:
                used[j] = True
            out.append((ax3, pt3, r3, k3, res3, f3, sum(raw[j][6] for j in grp)))
        else:
            for j in grp:
                used[j] = True                     # 不合并也只留面最多的那个（同一个孔的碎片）
            out.append(raw[i])
    return out


def detect_cylinders(mesh, P):
    """→ 孔圆柱列表：axis（单位）、pt（段中点处轴心）、r（段中点半径）、k（锥度 dr/dt）、t0/t1（相对 pt 沿轴）、
    cov（整段方位覆盖 0..1）、faces。"""
    from scipy.spatial import cKDTree
    E = edge_cylinders(mesh, P)
    origin = mesh.bounds.mean(0)
    seeds = seed_clusters(E, origin, P, True)
    fits = [fit_seed(E, mem, P) for mem in seeds]
    groups = merge_seeds([f for f in fits if f["n"] >= P["min_edges"]], P)
    tree = cKDTree(mesh.triangles_center)
    raw = []
    for grp in groups:
        g = _fit_group(mesh, tree, grp, P)
        if g is not None:
            raw.append(g + (sum(x["n"] for x in grp),))
    raw = merge_coaxial(mesh, tree, raw, P)
    # 去重：两个候选共用一半以上的面 → 留面多的
    raw.sort(key=lambda x: -len(x[5]))
    owner = set()
    cyls = []
    for ax, pt, r, k, res, fids, nseed in raw:
        fl = fids.tolist()
        if sum(1 for f in fl if f in owner) > 0.5 * len(fl):
            continue
        owner.update(fl)
        u, v = _perp_basis(ax)
        ta, tb, st, wd = face_intervals(mesh, fids, ax, pt, u, v)
        o = np.argsort(ta)
        segs, cs, ce, cur = [], ta[o[0]], tb[o[0]], [o[0]]
        for q in o[1:]:
            if ta[q] > ce + P["gap_tol"]:
                segs.append((cs, ce, cur)); cs, ce, cur = ta[q], tb[q], [q]
            else:
                ce = max(ce, tb[q]); cur.append(q)
        segs.append((cs, ce, cur))
        for (s0, s1, ks) in segs:
            ks = np.asarray(ks)
            cm = 0.5 * (s0 + s1)
            cyls.append(dict(axis=ax, pt=pt + cm * ax, r=float(r + k * cm), k=float(k), res=float(res),
                             t0=float(s0 - cm), t1=float(s1 - cm),
                             cov=coverage(st[ks], wd[ks]), nfaces=len(ks), faces=fids[ks], u=u, v=v, nseed=nseed,
                             f_t0=ta[ks] - cm, f_t1=tb[ks] - cm, f_st=st[ks], f_wd=wd[ks]))
    return _dedup_final(cyls)


def _dedup_final(cyls):
    """同一个孔留下的两个近似圆柱（歪孔两种拟合各收了一部分面）：几乎同轴（≤6°）、轴线距离 ≤0.35+0.05r、半径差 ≤0.25+0.05r、
    沿轴重叠 ≥ 短的一半 → 留覆盖高的（同覆盖留面多的）。"""
    order = sorted(range(len(cyls)), key=lambda i: (-cyls[i]["cov"], -cyls[i]["nfaces"]))
    keep = []
    for i in order:
        c = cyls[i]
        dup = False
        for j in keep:
            k = cyls[j]
            if abs(c["axis"] @ k["axis"]) < np.cos(np.radians(6.0)) or abs(c["r"] - k["r"]) > 0.25 + 0.05 * k["r"]:
                continue
            d1 = c["pt"] - k["pt"]; d1 = np.linalg.norm(d1 - (d1 @ k["axis"]) * k["axis"])
            if d1 > 0.35 + 0.05 * k["r"]:
                continue
            s = np.sign(c["axis"] @ k["axis"])
            base = (c["pt"] - k["pt"]) @ k["axis"]
            a0, a1 = sorted([base + s * c["t0"], base + s * c["t1"]])
            ov = min(a1, k["t1"]) - max(a0, k["t0"])
            if ov >= 0.5 * min(a1 - a0, k["t1"] - k["t0"]):
                dup = True
                break
        if not dup:
            keep.append(i)
    return [cyls[i] for i in sorted(keep)]


# ───────────────────────────── 逐层扫描 ─────────────────────────────
def _default_contains(mesh):
    if _HERE not in sys.path:
        sys.path.insert(0, _HERE)
    import ray_backend as RB
    return lambda pts: np.asarray(RB.contains(mesh, pts), dtype=bool)


CONTAINS_CHUNK = [25000]        # 每批 contains 点数（RB.contains 的内存峰值约与批大小成正比；check_holes 按 params 设）


def chunked(contains, pts, chunk=None):
    chunk = chunk or CONTAINS_CHUNK[0]
    out = np.empty(len(pts), dtype=bool)
    for k in range(0, len(pts), chunk):
        out[k:k + chunk] = np.asarray(contains(pts[k:k + chunk]), dtype=bool)
    return out


_TH_PHASE = 0.37          # 方位采样相位（度），避开与网格顶点正对
_T_PHASE = 0.0137         # 层位相位（mm），避开与端面正好共面


def _layers(c, P):
    L = c["t1"] - c["t0"] + 2 * P["margin"]
    step = max(P["step"], L / P["max_layers"])
    return np.arange(c["t0"] - P["margin"], c["t1"] + P["margin"] + 1e-9, step) + _T_PHASE, step


def _n_az(c, P):
    if 2 * c["r"] <= P["d_hole_max"]:
        return P["n_ring"]
    return int(np.clip(np.ceil(2 * np.pi * (c["r"] + 0.5) / P["az_arc"]), P["n_ring"], 360))


def local_fits(mesh, tree, c, t, P):
    """逐层局部拟圆：取孔壁附近的面（到轴距离在 r(t)±local_win、法线 ⟂ 轴 ±10°、指向轴），与每层截平面求交得截线端点，
    逐层拟圆 → 该层孔心偏移 (cx, cy)（在 u,v 基下，相对轴线）与半径 ρ。被位移场拉歪 / 锥 / 略椭的孔（H03 残孔）靠这个把
    "r+0.3 圈"放在真实孔壁外 0.3 处；规整孔上结果与整体拟合相同。拟不出来（点少、弧 <150°、残差大、偏离整体模型太多）的层
    用相邻有效层插值，全都没有就用整体模型。返回 (K,3) 的 [cx, cy, ρ]。"""
    K = len(t)
    base = np.column_stack([np.zeros(K), np.zeros(K), c["r"] + c.get("k", 0.0) * t])
    if not P["local_fit"]:
        return base
    ax, pt, u, v = c["axis"], c["pt"], c["u"], c["v"]
    win = P["local_win"]
    fids = collect_faces(mesh, tree, ax, pt, c["r"], c.get("k", 0.0), float(t[0]), float(t[-1]), P, rtol=win, dir_deg=10.0)
    if len(fids) < 3:
        return base
    T = mesh.triangles[fids] - pt                                   # (F,3,3)
    S = T @ ax; X = T @ u; Y = T @ v                                 # (F,3)
    out = base.copy()
    ok = np.zeros(K, bool)
    for kk in range(K):
        d = S - t[kk]
        xs, ys = [], []
        for i, j in ((0, 1), (1, 2), (2, 0)):
            cr = d[:, i] * d[:, j] < 0
            if cr.any():
                w = d[cr, i] / (d[cr, i] - d[cr, j])
                xs.append(X[cr, i] + w * (X[cr, j] - X[cr, i])); ys.append(Y[cr, i] + w * (Y[cr, j] - Y[cr, i]))
        if not xs:
            continue
        x = np.concatenate(xs); y = np.concatenate(ys)
        if len(x) < 6:
            continue
        cx, cy, rr = _circle_fit(x, y)
        ang = np.sort(np.arctan2(y - cy, x - cx))
        span = 2 * np.pi - np.max(np.diff(np.r_[ang, ang[0] + 2 * np.pi]))
        res = np.abs(np.hypot(x - cx, y - cy) - rr)
        if span < np.radians(150) or np.median(res) > 0.05:
            continue
        if abs(rr - base[kk, 2]) > win or np.hypot(cx, cy) > win:
            continue
        out[kk] = (cx, cy, rr); ok[kk] = True
    if ok.any() and not ok.all():
        idx = np.flatnonzero(ok)
        for col in range(3):
            out[~ok, col] = np.interp(np.flatnonzero(~ok), idx, out[idx, col])
    return out


def section_rings(mesh, tree, c, t, loc, n, offs, P):
    """拟合残差大的孔（被位移场拉歪 / 锥 / 椭的残孔）：逐层用垂直孔轴的截面找真实孔洞轮廓（包住轴心、面积 0.4–2.5 倍 πr² 的闭环），
    把轮廓往外偏 off（shapely buffer）后按弧长等分取 n 个点 → 代替圆环。某层找不到闭环（孔在这层开口 / 连到别处）就用圆环。
    返回 (K, n, len(offs), 3) 的点，与 _grid 同形。"""
    from shapely.geometry import Point
    G = _grid(c, t, offs, n, loc)
    ax, u, v = c["axis"], c["u"], c["v"]
    L = (t[-1] - t[0]) / 2 + 1.0
    cen0 = c["pt"] + ax * (0.5 * (t[0] + t[-1]))
    cand = np.asarray(tree.query_ball_point(cen0, float(np.hypot(L, c["r"] + 2.5))), dtype=np.int64)
    if len(cand) == 0:
        return G
    sub = mesh.submesh([cand], append=True)
    rr = c["r"] + c.get("k", 0.0) * t
    p0 = Point(0.0, 0.0)
    for kk in range(len(t)):
        o = c["pt"] + ax * t[kk] + loc[kk, 0] * u + loc[kk, 1] * v
        try:
            sec = sub.section(plane_origin=o, plane_normal=ax)
            if sec is None:
                continue
            T = np.eye(4)
            T[0, :3], T[1, :3], T[2, :3] = u, v, ax
            T[:3, 3] = -T[:3, :3] @ o
            P2, _ = sec.to_2D(to_2D=T)
            polys = [pg for pg in P2.polygons_closed if pg is not None]
        except Exception:
            continue
        area0 = np.pi * rr[kk] ** 2
        hole = None
        for pg in polys:
            if pg.contains(p0) and 0.4 * area0 <= pg.area <= 2.5 * area0 and (hole is None or pg.area < hole.area):
                hole = pg
        if hole is None:
            continue
        for mi, off in enumerate(offs):
            ring = hole.buffer(float(off), join_style=1).exterior
            if ring.length <= 0:
                continue
            ds = (np.arange(n) + 0.5) / n * ring.length
            xy = np.array([ring.interpolate(float(dd)).coords[0] for dd in ds])
            # 按方位角排序对齐到 _grid 的方位顺序（方位 j = j·360/n + 相位）
            ang = np.degrees(np.arctan2(xy[:, 1], xy[:, 0])) % 360.0
            want = (np.arange(n) * (360.0 / n) + _TH_PHASE) % 360.0
            idx = np.argmin(np.abs(((ang[None, :] - want[:, None]) + 180.0) % 360.0 - 180.0), axis=1)
            xy = xy[idx]
            G[kk, :, mi, :] = o + xy[:, 0:1] * u + xy[:, 1:2] * v
    return G


def _grid(c, t, offs, n, loc=None):
    """(K, n, m, 3)：第 k 层、方位 j、径向第 i 个偏移的点；半径 = 该层孔壁半径 + off；loc = local_fits 的 (K,3)。"""
    th = np.radians(np.arange(n) * (360.0 / n) + _TH_PHASE)
    dirs = np.cos(th)[:, None] * c["u"] + np.sin(th)[:, None] * c["v"]          # (n,3)
    if loc is None:
        loc = np.column_stack([np.zeros(len(t)), np.zeros(len(t)), c["r"] + c.get("k", 0.0) * t])
    R = loc[:, 2:3] + np.asarray(offs)[None, :]                                    # (K,m)
    cen = c["pt"] + t[:, None] * c["axis"] + loc[:, 0:1] * c["u"] + loc[:, 1:2] * c["v"]   # (K,3)
    return cen[:, None, None, :] + R[:, None, :, None] * dirs[None, :, None, :]


def _runs(mask, circular=False):
    """布尔序列 → [(起, 止(含))]；circular=True 时首尾相接的段合并（止可能 < 起，表示跨 0）。"""
    out, i, n = [], 0, len(mask)
    while i < n:
        if mask[i]:
            j = i
            while j + 1 < n and mask[j + 1]:
                j += 1
            out.append((i, j)); i = j + 1
        else:
            i += 1
    if circular and len(out) >= 2 and out[0][0] == 0 and out[-1][1] == n - 1:
        out = [(out[-1][0], out[0][1])] + out[1:-1]
    return out


def _arcs(mask):
    """一圈布尔（True = 缺）→ [(起索引, 宽度点数)]，跨 0 的段已合并。"""
    n = len(mask)
    if mask.all():
        return [(0, n)]
    return [(a, (b - a) % n + 1) for a, b in _runs(mask, circular=True)]


def _sector(miss, n):
    """一层缺失 → (最大缺口中心方位°, 最大缺口宽°, 缺口段数)。"""
    if not miss.any():
        return None, 0.0, 0
    if miss.all():
        return None, 360.0, 1
    arcs = _arcs(miss)
    a, w = max(arcs, key=lambda x: x[1])
    step = 360.0 / n
    return float(((a + (w - 1) / 2.0) * step + _TH_PHASE) % 360.0), float(w * step), len(arcs)


def _circ_dev(deg):
    if len(deg) < 2:
        return 0.0
    a = np.radians(deg)
    mu = np.arctan2(np.sin(a).mean(), np.cos(a).mean())
    return float(np.degrees(np.max(np.abs((a - mu + np.pi) % (2 * np.pi) - np.pi))))


# ───────────────────────────── 部分开口（螺丝孔 / 沉窝） ─────────────────────────────
def partial_runs(f, secs, step, P):
    """逐层 f（r+0.3 圈缺失比例）→ 部分开口段（0.10 ≤ f ≤ 0.90 的连续层）及其模式。
    模式（按顺序判）：
      mid            上下相邻层都是完整孔壁（f<0.10）→ 孔壁中段被切（原版 0 处）                       → 报
      exit_short     挨着孔口 / 腔，≤ short_layers 层（≤0.6 mm）→ 孔口倒角 / 端面略斜的过渡           → 不报
      mouth_irregular 挨着孔口，段内（去掉首末两层）≥2 层出现 ≥2 段缺口，或缺口方位漂移 > az_dev_max
                     → 不是一个平直的面切出来的（多把刀 / 扫掠 / 区域刀）                               → 报
      exit_slanted   挨着孔口，单段缺口、方位稳定、f 朝孔口单调升到 ≥ exit_edge_f → 孔斜穿出斜面 / 曲面   → 不报
      side_open      挨着孔口，单段缺口、方位稳定，但平台 / 突变（孔一侧沿整段开口：C 形孔、台阶边的沉窝…）
                     → 与原版设计性侧开口同形，默认不报（flag_side_open=True 时报）"""
    FLO, FHI = P["f_lo"], P["f_hi"]
    K = len(f)
    cls = np.where(f < FLO, "F", np.where(f > FHI, "O", "P"))
    out = []
    for (i, j) in _runs(cls == "P"):
        prev = cls[i - 1] if i > 0 else "E"
        nxt = cls[j + 1] if j + 1 < K else "E"
        n = j - i + 1
        seg = f[i:j + 1]
        where = "mid" if (prev == "F" and nxt == "F") else "mouth"
        inner = list(range(i + 1, j)) if n >= 3 else list(range(i, j + 1))
        multi = sum(1 for q in inner if secs[q][2] >= 2)
        az = [secs[q][0] for q in inner if f[q] >= 0.15 and secs[q][0] is not None]
        azdev = _circ_dev(az)
        if prev in "OE" and nxt not in "OE":
            prof = seg[::-1]
        elif nxt in "OE" and prev not in "OE":
            prof = seg
        else:
            prof = None                                  # 两头都开（C 形贯通）
        mono_drop = float(-np.clip(np.diff(prof), None, 0).sum()) if prof is not None and n > 1 else 0.0
        if where == "mid":
            pat = "mid"
        elif n <= P["short_layers"]:
            pat = "exit_short"
        elif multi >= 2 or azdev > P["az_dev_max"]:
            pat = "mouth_irregular"
        elif prof is not None and mono_drop <= P["mono_tol"] and prof[-1] >= P["exit_edge_f"]:
            pat = "exit_slanted"
        else:
            pat = "side_open"
        out.append(dict(i=i, j=j, n=n, len_mm=round(n * step, 2), where=where, prev=prev, next=nxt, pattern=pat,
                        fmax=float(seg.max()), fmin=float(seg.min()), multi_arc_layers=multi, az_dev_deg=round(azdev, 1),
                        mono_drop=round(mono_drop, 3), edge_f=None if prof is None else float(prof[-1])))
    return out


FLAG_PATTERNS = ("mid", "mouth_irregular")
PAT_CN = {"mid": "孔壁中段被切（上下都是完整孔壁）", "mouth_irregular": "孔口段缺口不规则（多段缺口 / 方位漂移）",
          "exit_short": "孔口短过渡（倒角 / 端面略斜，≤0.6 mm）", "exit_slanted": "孔斜穿出斜面 / 曲面（f 朝孔口单调升到 ≥0.7）",
          "side_open": "孔口段单侧平直开口（C 形孔 / 台阶边沉窝，与原版设计性开口同形）"}


# ───────────────────────────── 开口环 ─────────────────────────────
def ring_breaks(cut, mouth, t, step, n, c, pts_cut, P):
    """cut[k, j] = 第 k 层方位 j 从孔壁（r+0.05）到 r+1.0 全不在实体里；mouth[k] = 这一层孔壁整圈都没料（孔口 / 腔）。
    → 断口组：相邻层方位区间重叠的缺口连成一组。"""
    K = len(t)
    nodes = []                                   # (k, 起, 宽)
    for k in range(K):
        if mouth[k] or not cut[k].any():
            continue
        for a, w in _arcs(cut[k]):
            if w >= P["break_min_pts"]:
                nodes.append((k, a, w))
    if not nodes:
        return []
    parent = list(range(len(nodes)))

    def find(x):
        while parent[x] != x:
            parent[x] = parent[parent[x]]; x = parent[x]
        return x

    def overlap(a1, w1, a2, w2):
        s1 = set(((a1 + np.arange(w1)) % n).tolist())
        return any(((a2 + q) % n) in s1 for q in range(w2))

    byk = {}
    for idx, (k, a, w) in enumerate(nodes):
        byk.setdefault(k, []).append(idx)
    for idx, (k, a, w) in enumerate(nodes):
        for jdx in byk.get(k + 1, []):
            k2, a2, w2 = nodes[jdx]
            if overlap(a, w, a2, w2):
                parent[find(idx)] = find(jdx)
    groups = {}
    for idx in range(len(nodes)):
        groups.setdefault(find(idx), []).append(idx)
    out = []
    for g in groups.values():
        ks = sorted(set(nodes[q][0] for q in g))
        k0, k1 = ks[0], ks[-1]
        before = "E" if k0 == 0 else ("O" if mouth[k0 - 1] else "W")      # W = 有孔壁、没断
        after = "E" if k1 == K - 1 else ("O" if mouth[k1 + 1] else "W")
        widths = [max(nodes[q][2] for q in g if nodes[q][0] == k) * 360.0 / n for k in ks]
        # 最宽那层的缺口中心方位
        kw = ks[int(np.argmax(widths))]
        a, w = max(((nodes[q][1], nodes[q][2]) for q in g if nodes[q][0] == kw), key=lambda x: x[1])
        az = ((a + (w - 1) / 2.0) * 360.0 / n + _TH_PHASE) % 360.0
        # 断口点包围盒（孔壁外到 r+1.0 的点）
        sel = np.zeros_like(cut, dtype=bool)
        for q in g:
            k, a_, w_ = nodes[q]
            sel[k, (a_ + np.arange(w_)) % n] = True
        P3 = pts_cut[sel].reshape(-1, 3)
        where = "through" if (before in "OE" and after in "OE") else ("mid" if (before == "W" and after == "W") else "mouth")
        out.append(dict(k0=k0, k1=k1, n_layers=len(ks), len_mm=round(len(ks) * step, 2), before=before, after=after, where=where,
                        width_deg_max=round(max(widths), 1), width_deg_med=round(float(np.median(widths)), 1),
                        width_mm=round(np.radians(max(widths)) * (c["r"] + 0.5), 2), az_deg=round(float(az), 1), kw=kw,
                        widths=[round(x, 1) for x in widths],
                        bbox=[np.round(P3.min(0), 2).tolist(), np.round(P3.max(0), 2).tolist()]))
    return out


def leads_to(mesh, contains, c, t_mid, az_deg, P):
    """断口通到哪里：在断口中层作垂直孔轴的截面。孔心所在的空区 ——
    不在任何截面多边形外轮廓内 → 连着件外（"板边/外轮廓"）；落在某个多边形的内环里且内环面积明显大于孔 → "另一个孔或腔"。
    截面失败时退到射线：沿断口中线往外逐点采样，一路没碰到料 → "板边/外轮廓"，碰到料 → "连通外部"（说不清）。
    另给 via：沿断口中线径向直走是否先碰到料（碰到 = 先进窗 / 腔再出去）。"""
    o = c["pt"] + c["axis"] * t_mid
    th = np.radians(az_deg)
    d = np.cos(th) * c["u"] + np.sin(th) * c["v"]
    r_t = c["r"] + c.get("k", 0.0) * t_mid
    # 径向直走
    ext = min(float(np.linalg.norm(mesh.bounds[1] - mesh.bounds[0])), P["leads_max_mm"])
    ss = np.arange(r_t + 1.0, r_t + 1.0 + ext, P["leads_step_mm"])
    inside = chunked(contains, o + ss[:, None] * d[None, :])
    hit = bool(inside.any())
    hit_at = float(ss[np.argmax(inside)] - r_t) if hit else None
    via = "径向直通件外" if not hit else f"径向 {hit_at:.1f} mm 处碰到料（先进窗 / 腔）"
    try:
        from shapely.geometry import Point, Polygon
        sec = mesh.section(plane_origin=o, plane_normal=c["axis"])
        if sec is None:
            raise ValueError("no section")
        T = np.eye(4)
        T[0, :3], T[1, :3], T[2, :3] = c["u"], c["v"], c["axis"]
        T[:3, 3] = -T[:3, :3] @ o
        P2, _ = sec.to_2D(to_2D=T)
        pt0 = Point(0.0, 0.0)
        best = None
        for poly in P2.polygons_full:
            for ring in poly.interiors:
                rp = Polygon(ring)
                if rp.contains(pt0) and (best is None or rp.area < best.area):
                    best = rp
        if best is None:
            inside_any = any(Polygon(poly.exterior).contains(pt0) for poly in P2.polygons_full)
            if not inside_any:
                return "板边/外轮廓", via
            return "连通外部", via
        ratio = best.area / (np.pi * r_t * r_t)
        if ratio > P["merge_area_ratio"]:
            return "另一个孔或腔", via + f"（截面上孔与别的空腔连成一个内环，面积 = 孔的 {ratio:.1f} 倍）"
        return "连通外部", via + "（截面内环接近孔本身：断口只在这一层附近）"
    except Exception:
        return ("板边/外轮廓" if not hit else "连通外部"), via


# ───────────────────────────── 用途匹配（features） ─────────────────────────────
OWN_KINDS = ("screw_hole", "pilot_hole", "horn_hole", "counterbore", "spot_face", "bore", "through_hole")
OTHER_KINDS = ("screw_hole", "pilot_hole", "horn_hole", "counterbore", "spot_face", "tool_channel", "through_hole")


def _norm_features(screw_features, P):
    if screw_features is None:
        return None, None
    if isinstance(screw_features, dict):
        own = list(screw_features.get("own") or [])
        other = list(screw_features.get("other") or [])
    else:
        own = list(screw_features); other = []
        me = P.get("placed")
        if me:
            other = [f for f in own if f.get("placed") not in (None, me)]
            own = [f for f in own if f.get("placed") in (None, me)]
    return own, other


def _feat_geom(f):
    a = np.asarray(f["a"], float); b = np.asarray(f["b"], float)
    L = float(np.linalg.norm(b - a))
    return a, b, (b - a) / L if L > 1e-9 else None, L


def match_use(h, own, other, P):
    """孔 → 用途。own：本件 features；other：别的件 features（都是和网格同一坐标系的 a/b 轴线端点 + d）。
    判同轴：轴向夹角 ≤ use_ang_deg、轴线距离 ≤ use_dist_mm（孔中点到特征轴线、特征中点到孔轴线取大）、
    本件的还要沿轴区间重叠（容差 0.3）且直径差 ≤ max(use_dd_mm, use_dd_rel·d特征)；
    别的件的只要求本孔直径 ≥ 特征直径 − use_dd_mm（起子 / 螺丝头通道比螺丝孔大）、沿轴间隔 ≤ channel_max_gap。"""
    if own is None:
        return {"match": "n/a"}
    ax, c, d = np.asarray(h["axis"]), np.asarray(h["center"]), h["d_mm"]
    t0, t1 = h["extent_mm"]
    cosang = np.cos(np.radians(P["use_ang_deg"]))

    def coax(f):
        a, b, fa, L = _feat_geom(f)
        if fa is None:
            return None
        if abs(fa @ ax) < cosang:
            return None
        m = 0.5 * (a + b)
        d1 = c - a; d1 = float(np.linalg.norm(d1 - (d1 @ fa) * fa))
        d2 = m - c; d2 = float(np.linalg.norm(d2 - (d2 @ ax) * ax))
        dist = max(d1, d2)
        if dist > P["use_dist_mm"]:
            return None
        ta, tb = sorted([(a - c) @ ax, (b - c) @ ax])
        gap = max(0.0, ta - t1, t0 - tb)
        ang = float(np.degrees(np.arccos(min(1.0, abs(fa @ ax)))))
        return dict(dist=round(dist, 3), gap=round(float(gap), 2), ang=round(ang, 2), fd=float(f["d"]))

    best = None
    for f in own:
        if f.get("kind") not in OWN_KINDS:
            continue
        g = coax(f)
        if g is None or g["gap"] > 0.3:
            continue
        dd = abs(d - g["fd"])
        if dd > max(P["use_dd_mm"], P["use_dd_rel"] * g["fd"]):
            continue
        if best is None or dd < best[0]:
            best = (dd, f, g)
    if best:
        dd, f, g = best
        return {"match": "own", "feature": f.get("id"), "kind": f.get("kind"), "purpose": f.get("purpose"),
                "d_feature": g["fd"], "dd_mm": round(dd, 3), "dist_mm": g["dist"], "ang_deg": g["ang"]}
    # 本件螺丝孔 / 底孔 / 舵盘孔同轴、比它大：该孔的沉窝 / 头座（features 里常没单列沉窝，如 L02/L04 舵盘孔的 Ø4.4 沉窝）
    best = None
    for f in own:
        if f.get("kind") not in ("screw_hole", "pilot_hole", "horn_hole", "through_hole"):
            continue
        g = coax(f)
        if g is None or g["gap"] > 0.3 or d <= g["fd"] + P["use_dd_mm"] or d > P["d_hole_max"]:
            continue
        if best is None or g["dist"] < best[1]["dist"]:
            best = (f, g)
    if best:
        f, g = best
        return {"match": "own", "feature": f.get("id"), "kind": f.get("kind"), "purpose": f.get("purpose"),
                "inferred": f"与本件 {f.get('kind')} Ø{g['fd']} 同轴、比它大 → 推断为它的沉窝 / 头座（features 未单列）",
                "d_feature": g["fd"], "dist_mm": g["dist"], "ang_deg": g["ang"]}
    for f in own:
        if f.get("kind") != "tool_channel":
            continue
        g = coax(f)
        if g is None or g["gap"] > 0.3:
            continue
        if abs(d - g["fd"]) > max(0.5, 0.15 * g["fd"]):
            continue
        return {"match": "channel", "via": "本件 tool_channel", "feature": f.get("id"), "kind": f.get("kind"),
                "purpose": f.get("purpose"), "d_feature": g["fd"], "dist_mm": g["dist"], "ang_deg": g["ang"]}
    best = None
    for f in other or []:
        if f.get("kind") not in OTHER_KINDS:
            continue
        g = coax(f)
        if g is None or g["gap"] > P["channel_max_gap"]:
            continue
        if d < g["fd"] - P["use_dd_mm"]:
            continue
        key = (0 if f.get("kind") == "tool_channel" else 1, g["gap"], g["dist"])
        if best is None or key < best[0]:
            best = (key, f, g)
    if best:
        _, f, g = best
        return {"match": "channel", "via": "与别的件的孔同轴", "feature": f.get("id"), "part": f.get("part"),
                "placed": f.get("placed"), "kind": f.get("kind"), "purpose": f.get("purpose"), "d_feature": g["fd"],
                "dist_mm": g["dist"], "ang_deg": g["ang"], "gap_mm": g["gap"]}
    return {"match": "none"}


# ───────────────────────────── 主入口 ─────────────────────────────
def _prep(mesh):
    import trimesh
    if not isinstance(mesh, trimesh.Trimesh):
        raise TypeError("mesh 必须是 trimesh.Trimesh")
    if len(mesh.faces) and len(mesh.vertices) > 2.5 * len(mesh.faces):      # 没合并顶点（每面自带 3 个点）
        m = mesh.copy()
        m.merge_vertices()
        return m
    return mesh


def _r3(x, n=3):
    return [round(float(v), n) for v in x]


def check_holes(mesh, contains=None, params=None, screw_features=None):
    """见模块说明。"""
    t_start = time.time()
    P = dict(DEFAULTS)
    P.update(params or {})
    CONTAINS_CHUNK[0] = int(P["contains_chunk"])
    m = _prep(mesh)
    if contains is None:
        contains = _default_contains(mesh)
    own, other = _norm_features(screw_features, P)
    cyls = detect_cylinders(m, P)
    t_det = time.time() - t_start
    cand = [c for c in cyls if c["t1"] - c["t0"] >= P["len_min"] and c["cov"] >= P["cov_min"]
            and P["d_min"] <= 2 * c["r"] <= P["d_ring_max"]]
    cand.sort(key=lambda c: (2 * c["r"], tuple(np.round(c["pt"], 2))))
    OFFS = np.asarray(P["ring_offs"], float)                         # 孔壁外 → r+1.0；第 1 个是孔壁（0.05）、第 2 个是 r+0.3
    i03 = int(np.argmin(np.abs(OFFS - P["ring_off"])))
    # 第一轮：孔壁环 + r+0.3 环
    from scipy.spatial import cKDTree
    tree = cKDTree(m.triangles_center) if cand else None
    grids, meta = [], []
    for c in cand:
        t, step = _layers(c, P)
        n = _n_az(c, P)
        loc = local_fits(m, tree, c, t, P)
        if c.get("res", 0.0) > P["warp_rms"]:
            G = section_rings(m, tree, c, t, loc, n, OFFS, P)            # 歪孔：沿真实孔洞轮廓外偏
        else:
            G = _grid(c, t, OFFS, n, loc)                               # (K,n,m,3)
        grids.append(G); meta.append((t, step, n, loc))
    first = np.concatenate([G[:, :, [0, i03], :].reshape(-1, 3) for G in grids]) if grids else np.zeros((0, 3))
    ins1 = chunked(contains, first)
    k = 0
    INS = []
    for G in grids:
        K, n, mm, _ = G.shape
        a = np.zeros((K, n, mm), dtype=bool)
        cnt = K * n * 2
        a[:, :, [0, i03]] = ins1[k:k + cnt].reshape(K, n, 2); k += cnt
        INS.append(a)
    # 第二轮：孔壁和 r+0.3 都没料的方位，再查外面几圈
    rest_idx = [q for q in range(len(OFFS)) if q not in (0, i03)]
    need, where_ = [], []
    for ci, (G, a) in enumerate(zip(grids, INS)):
        sel = ~a[:, :, 0] & ~a[:, :, i03]
        if sel.any() and rest_idx:
            need.append(G[sel][:, rest_idx, :].reshape(-1, 3)); where_.append((ci, sel))
    if need:
        ins2 = chunked(contains, np.concatenate(need))
        k = 0
        for ci, sel in where_:
            nsel = int(sel.sum()); cnt = nsel * len(rest_idx)
            sub = ins2[k:k + cnt].reshape(nsel, len(rest_idx)); k += cnt
            blk = INS[ci][sel]
            blk[:, rest_idx] = sub
            INS[ci][sel] = blk
    holes, rings = [], []
    hid = 0
    for c, G, a, (t, step, n, loc) in zip(cand, grids, INS, meta):
        d = 2 * c["r"]
        ax = c["axis"]
        ends = [c["pt"] + ax * c["t0"], c["pt"] + ax * c["t1"]]
        base = dict(d_mm=round(d, 3), axis=_r3(ax, 4), center=_r3(c["pt"]), extent_mm=[round(c["t0"], 3), round(c["t1"], 3)],
                    ends=[_r3(ends[0]), _r3(ends[1])], cov=round(c["cov"], 3), taper=round(c.get("k", 0.0), 4),
                    fit_rms=round(c.get("res", 0.0), 4), warped=bool(c.get("res", 0.0) > P["warp_rms"]))
        use = match_use(dict(base, axis=ax, center=c["pt"]), own, other, P)
        miss03 = ~a[:, :, i03]
        f = miss03.mean(1)
        hid += 1
        if d <= P["d_hole_max"]:
            secs = [_sector(miss03[q], n) for q in range(len(t))]
            rr = partial_runs(f, secs, step, P)
            flagged = [x for x in rr if x["pattern"] in FLAG_PATTERNS or (P["flag_side_open"] and x["pattern"] == "side_open")]
            worst = max(flagged, key=lambda x: (x["fmax"], x["n"])) if flagged else (max(rr, key=lambda x: (x["n"] * x["fmax"])) if rr else None)
            bbox = None
            if worst:
                pts = G[worst["i"]:worst["j"] + 1, :, i03, :][miss03[worst["i"]:worst["j"] + 1]]
                if len(pts):
                    bbox = [_r3(pts.min(0)), _r3(pts.max(0))]
            npart = int(((f >= P["f_lo"]) & (f <= P["f_hi"])).sum())
            flag = bool(flagged)
            if worst is None:
                reason = "没有部分开口层（0.10≤f≤0.90）"
            else:
                tr = [round(float(t[worst["i"]]), 2), round(float(t[worst["j"]]), 2)]
                reason = (f"{PAT_CN[worst['pattern']]}：r+{P['ring_off']} 圈缺 {worst['fmin']*100:.0f}–{worst['fmax']*100:.0f}%，"
                          f"连续 {worst['n']} 层 {worst['len_mm']} mm（沿轴 {tr[0]}..{tr[1]}），"
                          f"多段缺口层 {worst['multi_arc_layers']}，方位漂移 {worst['az_dev_deg']}°")
                if len(rr) > 1:
                    reason += f"；另有 {len(rr) - 1} 段：" + "、".join(f"{x['pattern']}({x['n']}层,{x['fmax']*100:.0f}%)" for x in rr if x is not worst)
            holes.append(dict(id=hid, kind="screw" if d <= P["d_screw_max"] else "cbore", **base,
                              n_layers=len(t), step_mm=round(step, 3), n_partial_layers=npart,
                              max_partial_frac=round(worst["fmax"], 3) if worst else 0.0,
                              partial_len_mm=worst["len_mm"] if worst else 0.0,
                              partial_where=worst["where"] if worst else None,
                              partial_pattern=worst["pattern"] if worst else None,
                              partial_bbox=bbox, flag=flag, reason=reason, use=use,
                              runs=[dict(pattern=x["pattern"], where=x["where"], n=x["n"], len_mm=x["len_mm"],
                                         t=[round(float(t[x["i"]]), 2), round(float(t[x["j"]]), 2)],
                                         fmin=round(x["fmin"], 3), fmax=round(x["fmax"], 3), multi_arc_layers=x["multi_arc_layers"],
                                         az_dev_deg=x["az_dev_deg"], edge_f=None if x["edge_f"] is None else round(x["edge_f"], 3))
                                    for x in rr]))
        # 开口环
        if P["open_rings"] and d >= P["ring_d_min"]:
            cut = ~a.any(axis=2)                                                  # 孔壁到 r+1.0 全没料
            mouth = ~a[:, :, 0].any(axis=1)                                       # 孔壁整圈没料 = 孔口 / 腔
            brs = ring_breaks(cut, mouth, t, step, n, c, G, P)
            for b in brs:
                pat, flag, why = classify_break(b, P)
                lt, via = (None, None)
                if flag or P["ring_leads_all"]:
                    lt, via = leads_to(m, contains, c, float(t[b["kw"]]), b["az_deg"], P)
                rings.append(dict(hole_id=hid, kind=("screw" if d <= P["d_screw_max"] else ("cbore" if d <= P["d_hole_max"] else "bore")),
                                  **base, where=b["where"], pattern=pat, width_deg=b["width_deg_max"], width_deg_median=b["width_deg_med"],
                                  width_mm_at_r05=b["width_mm"], az_deg=b["az_deg"], len_mm=b["len_mm"], n_layers=b["n_layers"],
                                  t=[round(float(t[b["k0"]]), 2), round(float(t[b["k1"]]), 2)], bbox=b["bbox"],
                                  widths=b["widths"], before=b["before"], after=b["after"],
                                  leads_to=lt, via=via, flag=flag, reason=why, use=use))
    findings = [h for h in holes if h["flag"]]
    ring_find = [r for r in rings if r["flag"]]

    def _cnt(lst):
        return {"own": sum(1 for h in lst if h["use"].get("match") == "own"),
                "channel": sum(1 for h in lst if h["use"].get("match") == "channel"),
                "none": sum(1 for h in lst if h["use"].get("match") == "none"),
                "n/a": sum(1 for h in lst if h["use"].get("match") == "n/a")}
    pats = {}
    for h in holes:
        for x in h["runs"]:
            pats[x["pattern"]] = pats.get(x["pattern"], 0) + 1
    status = "FAIL" if (findings or ring_find) else "PASS"
    return {"status": status,
            "params": {k: (list(v) if isinstance(v, tuple) else v) for k, v in P.items()},
            "holes": holes, "findings": findings, "open_rings": rings, "open_ring_findings": ring_find,
            "stats": {"n_holes": len(holes), "n_flagged": len(findings), "n_flagged_by_use": _cnt(findings),
                      "n_cylinders_checked_for_rings": sum(1 for c in cand if 2 * c["r"] >= P["ring_d_min"]),
                      "n_open_rings": len(rings), "n_open_rings_flagged": len(ring_find), "n_open_rings_flagged_by_use": _cnt(ring_find),
                      "run_patterns": pats, "seconds": round(time.time() - t_start, 2), "seconds_detect": round(t_det, 2),
                      "faces": int(len(m.faces))}}


RING_CN = {"mouth_exit": "孔口段断口（一头挨着孔口 / 腔，孔斜出或口部开口）",
           "through_narrow": "窄缝贯通：孔沿整段被一条窄缝切穿，环没闭合",
           "through_wide": "宽口贯通：C 形 / 半孔 / 开口座（原版同形，设计性开口）",
           "mid_slit": "孔壁中段有一段被径向切穿（上下都是完整环）"}


def classify_break(b, P):
    """开口环断口 → (模式, 报不报, 理由)。按原版标定（见 holes_calibration.md）：
      where = mouth（一头挨着孔口 / 腔 / 扫描端）                      → mouth_exit，不报（原版 56 处全是这类孔口斜出 / 口部开口）
      where = through（从孔口到孔口整段都断）且断口宽度中位数 ≤ ring_narrow_deg → through_narrow，报（N01 惰轮孔 21.5°）
      where = through 且更宽                                               → through_wide，不报（原版 23 处：半孔 180°、C 形座、开口孔，最窄 75°）
      where = mid（上下都有完整环）                                         → mid_slit，默认不报（原版 m12 镜座有一处 7–18° 中段缝，与我们的同形；ring_flag_mid=True 时报）"""
    w = b["width_deg_med"]
    if b["where"] == "mouth":
        pat, flag = "mouth_exit", False
    elif b["where"] == "through":
        if w <= P["ring_narrow_deg"]:
            pat, flag = "through_narrow", True
        else:
            pat, flag = "through_wide", False
    else:
        pat, flag = "mid_slit", bool(P["ring_flag_mid"])
    why = (f"{RING_CN[pat]}：断口宽 {b['width_deg_med']}°（最宽 {b['width_deg_max']}°，r+0.5 处弧长 {b['width_mm']} mm），"
           f"沿轴 {b['len_mm']} mm（{b['n_layers']} 层；前后 {b['before']}/{b['after']}，O=孔口 E=扫描端 W=完整环）")
    return pat, flag, why


def _load_features(path):
    d = json.load(open(path, encoding="utf-8"))
    return d["features"] if isinstance(d, dict) else d


def main(argv=None):
    import argparse
    import trimesh
    ap = argparse.ArgumentParser(description="螺丝孔 / 沉窝部分开口 + 开口环检测（纯几何）")
    ap.add_argument("stl", nargs="+")
    ap.add_argument("--out", required=True)
    ap.add_argument("--features", default=None, help="hole_features_world.json（按文件名找件：placed = 文件名去 .stl）")
    ap.add_argument("--params", default=None, help="JSON 串，覆盖 DEFAULTS")
    a = ap.parse_args(argv)
    params = json.loads(a.params) if a.params else {}
    feats = _load_features(a.features) if a.features else None
    out, tot = {}, dict(files=0, fail=0, holes=0, flagged=0, open_rings=0, open_rings_flagged=0)
    for f in a.stl:
        placed = os.path.splitext(os.path.basename(f))[0]
        sf = None
        if feats is not None and any(x.get("placed") == placed for x in feats):
            sf = {"own": [x for x in feats if x.get("placed") == placed], "other": [x for x in feats if x.get("placed") != placed]}
        m = trimesh.load(f, force="mesh")
        R = check_holes(m, params=dict(params, placed=placed), screw_features=sf)
        st = R["stats"]
        out[f] = R
        tot["files"] += 1; tot["fail"] += R["status"] == "FAIL"; tot["holes"] += st["n_holes"]; tot["flagged"] += st["n_flagged"]
        tot["open_rings"] += st["n_open_rings"]; tot["open_rings_flagged"] += st["n_open_rings_flagged"]
        print(f"{R['status']} {f}: 孔 {st['n_holes']}，部分开口报 {st['n_flagged']} {st['n_flagged_by_use']}；"
              f"开口环 {st['n_open_rings']}，报 {st['n_open_rings_flagged']}；{st['seconds']} s", flush=True)
        for h in R["findings"]:
            print(f"   孔#{h['id']} {h['kind']} Ø{h['d_mm']} @{h['center']} use={h['use'].get('match')}：{h['reason']}")
        for r in R["open_ring_findings"]:
            print(f"   环#{r['hole_id']} {r['kind']} Ø{r['d_mm']} @{r['center']} → {r['leads_to']}：{r['reason']}")
    json.dump(dict(summary=tot, files=out), open(a.out, "w", encoding="utf-8"), ensure_ascii=False, indent=1)
    print("写出", a.out, tot)
    return 0


if __name__ == "__main__":
    sys.exit(main())
