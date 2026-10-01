#!/usr/bin/env python3
"""第 2 层 特征存在 —— 设计里说的孔/台/槽，在**导出的 STL** 里真的存在吗。

这一层是"空刀"的克星：`driven()` / `horn_holes()` 返回舵机局部坐标，不 `placed()` 就直接 diff，
等于在世界原点切了个寂寞 —— 6 个法兰孔一个都没打出来，而当时所有检查都是绿的。
所以本层**不看建模代码、不看内存网格**，只看 `cad/duck_s288/*.stl` 读回来的三角面。

判据来源（一律 tolerances.yaml，层里不写死数字）：
  feature_check_tolerances.hole_diameter_mm       孔径 ±
  feature_check_tolerances.hole_depth_mm          盲深/沉孔台阶深 ±
  feature_check_tolerances.mirror_error_mm        镜像件特征镜像误差
  feature_check_tolerances.horn_concentricity_mm  舵盘 6 孔同心 / 孔阵半径
  feature_check_tolerances.horn_clock_deg         舵盘 6 孔分度
  fits.<桶>.target_range_mm                       有对应配合桶的孔径按桶判
  fits.<件的材料>.<feature_type=wall_thickness>   孔周壁厚（按件的材料选桶，不是一律 PLA）

坐标系（2026-09-09 起不再是 unknown 的来源）：
  features.yaml 每条特征都有 `geom:` 段，`pos` / `axial_span_mm` / `hole_positions_mm` /
  `bbox_mm` 全部已改写到 **export_local**，也就是导出 STL 自己的坐标系
  （`frames.export_local`：p_export_local = inv(TW(body))·p_world）。
  所以本层**直接拿 geom 的坐标当射线起点、拿 geom.axis 当射线轴**，不再从
  spec_verbatim 的自由文本猜位置 —— spec_verbatim 退化为追溯信息，只出现在 detail 里。
  只有 `geom.confidence == "unresolved"`（2 条 not_modeled）仍判 unknown：那是数据本身没有。

不用 trimesh 的 split() / is_watertight（README 坑 11）：process=False 时顶点没合并，
每个三角面各算一块；process=True 又按容差焊点造假闭合。这里沿用 l0_mesh 的精确 float32 去重。
"""
from __future__ import annotations

import math
import re
import sys
from itertools import combinations
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from core import LayerResult, PASS, FAIL, BLOCK, WARN, INFO, num, ROOT   # noqa: E402

LAYER = 2
NAME = "特征存在"

# ── 采样 / 数值常量 ────────────────────────────────────────────────────────
# 这些不是判据（判据全在 tolerances.yaml），是"怎么把三角面认成圆柱"的数值稳定量与采样密度。
# 元规则 3：采样角度全部均匀分度，与 duck.py 建模用的 sections 无关。
_SEG_ANGLE_DEG = 35.0   # 光滑面片聚类的二面角上限：> 圆柱 32 段的 11.25°，< 孔口/倒角的 90°
_FIT_RES_MAX = 0.02     # 圆拟合残差上限 mm：STL 顶点精确落在名义圆上，超了就不是圆柱面
_MIN_COVER_DEG = 40.0   # 角覆盖下限：滤掉有机曲面上偶然拟合成圆的碎片（实测噪声都 < 30°）
_MIN_FACES = 4
_RAY_DIRS = 16          # 每个孔的周向射线条数 —— 这是证据数，不是判据
_PATTERN_RES = 0.05     # 6 孔阵 RANSAC 的径向内点容差（识别用，判据另取 horn_concentricity_mm）
_EPS = 1e-7

# ── geom 段探针的采样密度（同样不是判据）───────────────────────────────────
_SCAN_DIRS = 24         # 每站的周向射线条数，与 features.yaml:frames.verification 用的 24 一致
_SCAN_STATIONS = 21     # 沿轴扫描的站数（声明的 axial_span 与件包围盒的交集内均分）
_PROBE_WINDOW = 2.0     # 径向搜索窗 mm：只在名义半径 ± 该窗内找壁，窗外算"没量到"
_STEP_OFFSET = 0.3      # 量台阶深度时，内/外两圈探针相对名义半径的偏移 mm
_STEP_DIRS = 8          # 量台阶深度/通盲的平行轴射线条数
_STEP_MARGIN = 3.0      # 台阶探针沿轴向外让开声明区间的余量 mm（射线尽量短，少踩脏网格）
_THROUGH_LOOK = 1.0     # 判"通"时往声明区间外多看这么长：料是不是紧接着又回来了
_OCC_GRID = 3           # box 类特征的占据采样网格边长（3×3×3 = 27 点）
_OCC_INSET = 0.08       # 占据采样点相对 bbox 的内缩比例，避开面上的数值歧义
_OCC_GRID_CAP = 60      # 单条特征最多打多少个 bbox 网格旁证点（射线预算）
_KEEP_STEP = 1.0        # 保留区判据的网格步长 mm（不是判据，是采样密度）
_KEEP_CAP = 512         # 单条实体特征最多取多少个保留区采样点
_KEEP_MARGIN = 0.25     # 扣除已声明孔槽时往外让开的余量 mm（面上的点不算数）
_SEAT_RINGS = 3         # 螺丝头足印环带上取几圈半径（内=孔壁，外=头沿）
_GAUGE_END_SKIP = 1     # 通规的棱边扫描在孔身两端各让开几个站距（避开端面倒角/台阶的边）
_GAUGE_EDGE_SAMPLES = 9  # 每条三角面棱边取几个采样点（棱边是直线，采样只影响最小值的定位精度）
_GAUGE_END_FRAC = 0.15   # 短孔（只有两三站）时，两端改按孔身长度的比例让开，别把整段让没了
_PARITY_DIRS = np.array([[0.7371, 0.4567, 0.4982], [-0.5123, 0.7719, -0.3761],
                         [0.3319, -0.4111, 0.8487]])   # 三个非轴对齐方向做奇偶判内外，多数表决


# ── STL 读回 + 基础几何 ────────────────────────────────────────────────────
_GEOM_CACHE: dict[str, "PartGeom"] = {}


def read_stl_arrays(p: Path):
    """二进制 STL → (顶点表, 面表)。精确 float32 去重（和 l0_mesh._bodies_and_extents 同一套）。"""
    import struct
    with open(p, "rb") as f:
        n = struct.unpack("<I", f.read(84)[80:84])[0]
        data = f.read(50 * n)
    tris = np.frombuffer(data, dtype=np.dtype([("n", "<3f4"), ("v", "<3,3f4"), ("a", "<u2")]), count=n)
    V = tris["v"].reshape(-1, 3)
    uniq, inv = np.unique(V, axis=0, return_inverse=True)
    return uniq.astype(np.float64), inv.reshape(-1, 3).astype(np.int64)


def _face_geom(V, F):
    a, b, c = V[F[:, 0]], V[F[:, 1]], V[F[:, 2]]
    n = np.cross(b - a, c - a)
    L = np.linalg.norm(n, axis=1)
    ok = L > 1e-12
    nn = np.zeros_like(n)
    nn[ok] = n[ok] / L[ok, None]
    return nn, L / 2.0, (a + b + c) / 3.0


def _smooth_patches(F, N):
    """相邻且二面角 < _SEG_ANGLE_DEG 的面并成一片。孔口是 90° 硬边，孔壁自然独立成片。"""
    E: dict[tuple, list] = {}
    for fi in range(len(F)):
        a, b, c = F[fi]
        for u, v in ((a, b), (b, c), (c, a)):
            E.setdefault((u, v) if u < v else (v, u), []).append(fi)
    parent = np.arange(len(F))

    def find(x):
        while parent[x] != x:
            parent[x] = parent[parent[x]]
            x = parent[x]
        return x

    ct = math.cos(math.radians(_SEG_ANGLE_DEG))
    for fl in E.values():
        if len(fl) != 2:
            continue
        i, j = fl
        if float(np.dot(N[i], N[j])) >= ct:
            ri, rj = find(i), find(j)
            if ri != rj:
                parent[rj] = ri
    return np.array([find(i) for i in range(len(F))])


def _fit_circle(P2):
    x, y = P2[:, 0], P2[:, 1]
    A = np.column_stack([x, y, np.ones(len(x))])
    sol, *_ = np.linalg.lstsq(A, x * x + y * y, rcond=None)
    cx, cy = sol[0] / 2, sol[1] / 2
    r = math.sqrt(max(sol[2] + cx * cx + cy * cy, 0.0))
    return float(cx), float(cy), float(r), float(np.abs(np.hypot(x - cx, y - cy) - r).max())


def _basis(ax):
    tmp = np.array([1.0, 0, 0]) if abs(ax[0]) < 0.9 else np.array([0, 1.0, 0])
    e1 = np.cross(ax, tmp)
    e1 /= np.linalg.norm(e1)
    return e1, np.cross(ax, e1)


def detect_cylinders(V, F):
    """在导出的三角面上找圆柱面：光滑面片 → 法向共面定轴 → 顶点拟合圆 → 凹/凸判孔/台。

    每条 dict: axis(单位) / p0(轴上离原点最近的点) / mid(面片轴向中点) / r / t0,t1(沿轴范围)
              hole(True=孔，法向指向轴心) / cover(角覆盖 deg) / nf / res
    半圈座（H01 A/B 座 ~102°）也要收，所以不要求 360°。
    仍然保留：舵盘 6 孔阵与镜像比对是**与声明坐标无关**的不变量，用它们互为旁证。
    """
    N, _A, C = _face_geom(V, F)
    roots = _smooth_patches(F, N)
    order = np.argsort(roots, kind="stable")
    groups = np.split(order, np.searchsorted(roots[order], np.unique(roots)[1:]))
    out = []
    sin6 = math.sin(math.radians(6.0))
    for fl in groups:
        if len(fl) < _MIN_FACES:
            continue
        Nl = N[fl]
        _u, _s, vt = np.linalg.svd(Nl, full_matrices=False)
        ax = vt[-1]
        if float(np.abs(Nl @ ax).max()) > sin6:          # 法向没全部垂直于轴 → 不是圆柱
            continue
        P = V[np.unique(F[fl])]
        e1, e2 = _basis(ax)
        P2 = np.column_stack([P @ e1, P @ e2])
        cx, cy, rad, res = _fit_circle(P2)
        if rad < 0.3 or rad > 80.0 or res > _FIT_RES_MAX:
            continue
        ang = np.sort(np.mod(np.degrees(np.arctan2(P2[:, 1] - cy, P2[:, 0] - cx)), 360.0))
        if len(ang) < 3:
            continue
        cover = 360.0 - float(np.diff(np.concatenate([ang, [ang[0] + 360.0]])).max())
        if cover < _MIN_COVER_DEG:
            continue
        c2 = np.column_stack([C[fl] @ e1, C[fl] @ e2])
        n2 = np.column_stack([Nl @ e1, Nl @ e2])
        d = np.column_stack([cx - c2[:, 0], cy - c2[:, 1]])
        dn = np.linalg.norm(d, axis=1)
        m = dn > 1e-9
        if not m.any():
            continue
        frac_in = float((np.sum(d[m] * n2[m], axis=1) / dn[m] > 0).mean())
        if 0.15 < frac_in < 0.85:                        # 一半朝内一半朝外 → 不是单一圆柱面
            continue
        p0 = cx * e1 + cy * e2
        t = P @ ax
        out.append(dict(axis=ax, p0=p0, r=rad, t0=float(t.min()), t1=float(t.max()),
                        mid=p0 + ax * float((t.min() + t.max()) / 2), hole=bool(frac_in > 0.5),
                        cover=cover, nf=int(len(fl)), res=res, n_patch=1))
    return _merge_coaxial(out)


def _merge_coaxial(cyls, dr=0.03, dp=0.05):
    """同轴同径、被肋/相交几何打断的面片合成一条（L07 的 6700 座就被打断成 319° + 45°）。"""
    used = [False] * len(cyls)
    out = []
    for i, a in enumerate(cyls):
        if used[i]:
            continue
        grp = [a]
        used[i] = True
        for j in range(i + 1, len(cyls)):
            b = cyls[j]
            if used[j] or a["hole"] != b["hole"] or abs(a["r"] - b["r"]) > dr:
                continue
            if abs(abs(float(np.dot(a["axis"], b["axis"]))) - 1.0) > 1e-3:
                continue
            v = b["p0"] - a["p0"]
            if np.linalg.norm(v - float(np.dot(v, a["axis"])) * a["axis"]) > dp:
                continue
            grp.append(b)
            used[j] = True
        ax = a["axis"]
        ts = []
        for g in grp:
            s = 1.0 if float(np.dot(g["axis"], ax)) > 0 else -1.0
            ts += [s * g["t0"], s * g["t1"]]
        t0, t1 = min(ts), max(ts)
        out.append(dict(axis=ax, p0=a["p0"], r=float(np.mean([g["r"] for g in grp])),
                        t0=t0, t1=t1, mid=a["p0"] + ax * (t0 + t1) / 2,
                        hole=a["hole"], cover=min(360.0, sum(g["cover"] for g in grp)),
                        nf=sum(g["nf"] for g in grp), res=max(g["res"] for g in grp),
                        n_patch=len(grp)))
    return out


# ── 射线 ───────────────────────────────────────────────────────────────────
def ray_hits(V, F, o, d):
    """Möller–Trumbore，一条射线打全部三角面。返回 (排序后的 t, 该面法向·方向)。"""
    a = V[F[:, 0]]
    e1 = V[F[:, 1]] - a
    e2 = V[F[:, 2]] - a
    p = np.cross(d, e2)
    det = np.einsum("ij,ij->i", e1, p)
    ok = np.abs(det) > 1e-12
    inv = np.zeros_like(det)
    inv[ok] = 1.0 / det[ok]
    tv = o - a
    u = np.einsum("ij,ij->i", tv, p) * inv
    q = np.cross(tv, e1)
    v = (q @ d) * inv
    t = np.einsum("ij,ij->i", e2, q) * inv
    m = ok & (u >= -1e-9) & (v >= -1e-9) & (u + v <= 1 + 1e-9) & (t > _EPS)
    tt = t[m]
    nd = np.cross(e1, e2)[m] @ d
    i = np.argsort(tt)
    return tt[i], nd[i]


def solid_spans(t, nd):
    """沿射线的实体区间（进入 = 面法向与射线反向）。返回 (总长, [(进, 出)…])。"""
    depth, start, total, segs = 0, None, 0.0, []
    for ti, ni in zip(t, nd):
        if ni < 0:
            if depth == 0:
                start = float(ti)
            depth += 1
        else:
            depth -= 1
            if depth <= 0 and start is not None:
                total += float(ti) - start
                segs.append((start, float(ti)))
                start = None
            depth = max(depth, 0)
    return total, segs


def dedup_hits(t, nd, eps=1e-6):
    """同一个 t 上的同向重复命中只留一个。

    两种来源：布尔缝留下的重合面（L07 的 6700 座）、射线正好压在两三角面的公共边上
    （方块面的对角线就是）。不去重，`solid_spans` 的进出计数不归零，整段料会被吞掉。
    """
    tt, nn, pt, ps = [], [], None, None
    for x, n in zip(t, nd):
        x, sg = float(x), (n < 0)
        if pt is not None and abs(x - pt) < eps and sg == ps:
            continue
        tt.append(x)
        nn.append(-1.0 if sg else 1.0)
        pt, ps = x, sg
    return np.array(tt), np.array(nn)


def solid_spans_dedup(t, nd):
    return solid_spans(*dedup_hits(t, nd))


def hole_open_ends(V, F, cyl, back=None):
    """沿孔轴打一条射线：轴线上落在孔身范围内还有料 → 盲孔。返回 (是否真通, 盲深 or None)。"""
    ax, mid = cyl["axis"], cyl["mid"]
    span = cyl["t1"] - cyl["t0"]
    back = 50.0 if back is None else float(back)
    o = mid - ax * (span / 2 + back)
    t, nd = ray_hits(V, F, o, ax)
    lo, hi = back, back + span
    inside = [(float(x), float(n)) for x, n in zip(t, nd) if lo - 0.05 < x < hi + 0.05]
    enter = next((x for x, n in inside if n < 0), None)
    if enter is None:
        return True, None
    return False, max(0.0, enter - lo)


def hole_wall_min(V, F, cyl):
    """孔周最小壁厚：孔身中点、垂直孔轴向外打 _RAY_DIRS 条射线，
    取「从孔壁到下一个自由面」这一段料的最小值。某向直接通到外面就不计入。"""
    ax, mid, r = cyl["axis"], cyl["mid"], cyl["r"]
    e1, e2 = _basis(ax)
    best, n_ok = None, 0
    for k in range(_RAY_DIRS):
        a = 2 * math.pi * k / _RAY_DIRS
        d = math.cos(a) * e1 + math.sin(a) * e2
        t, nd = ray_hits(V, F, mid, d)
        _tot, segs = solid_spans(t, nd)
        seg = next((s for s in segs if s[1] > r - 0.05), None)
        if seg is None:
            continue
        w = seg[1] - max(seg[0], r)
        n_ok += 1
        if best is None or w < best:
            best = w
    return best, n_ok


def seat_profile(V, F, cyl, r_in, r_out, back=80.0, n_ring=_SEAT_RINGS, n_ang=_RAY_DIRS):
    """一颗螺丝头足印下的**完整环带**剖面：内半径 r_in（孔壁）到外半径 r_out（头沿）。

    FG15 的教训：以前这里只量"厚度极差"，而**等厚斜面**的厚度极差恰好是 0 ——
    坐面歪 15° 也照样绿。所以本函数同时给三个量，判据方各取所需：

      thickness  沿孔轴、与孔身重叠的那一段连续料的长度（螺丝真要穿过去的叠厚）
      face_t     该向**头侧承压面**在孔轴上的坐标 —— 平面度就是它的极差，
                 等厚斜面的 thickness 恒定而 face_t 摆动 2·r·tanθ
      solid_frac 环带里真有承压料的采样比例 → 有效承压面积 = 比例 × 环带面积

    只算与孔身重叠的连续料，远处另一堵墙不算（否则壳类件会量出几十毫米的假极差）。
    两端都量，返回**较差**的那一端（不知道头朝哪边时保守取）。
    """
    ax, mid = np.asarray(cyl["axis"], float), np.asarray(cyl["mid"], float)
    e1, e2 = _basis(ax)
    span = cyl["t1"] - cyl["t0"]
    lo, hi = back - span / 2, back + span / 2
    # 采样半径取**等面积**子环带的中线：既不会正好压在孔壁（r_in）或头沿（r_out）上
    # 打出擦边射线，solid_frac × 环面积 也才是无偏的有效承压面积。
    a, b = float(r_in), float(r_out)
    if n_ring <= 1 or b <= a:
        radii = [a]
    else:
        n_ring = int(n_ring)
        radii = [math.sqrt(a * a + (k + 0.5) / n_ring * (b * b - a * a)) for k in range(n_ring)]
    th, face_up, face_dn, n_rays = [], [], [], 0
    for r in radii:
        for k in range(n_ang):
            a = 2 * math.pi * k / n_ang
            u = math.cos(a) * e1 + math.sin(a) * e2
            t, nd = ray_hits(V, F, mid + r * u - ax * back, ax)
            n_rays += 1
            _tot, segs = solid_spans_dedup(t, nd)
            keep = [(s0, s1) for s0, s1 in segs if s1 > lo - 0.05 and s0 < hi + 0.05]
            th.append(sum(s1 - s0 for s0, s1 in keep))
            # 承压面 = 这段料朝两端的两张自由面在孔轴上的位置（相对孔身中点）
            face_up.append(min(s0 for s0, _s1 in keep) - back if keep else np.nan)
            face_dn.append(max(s1 for _s0, s1 in keep) - back if keep else np.nan)
    arr = np.array(th, float)
    n_void = int((arr <= 1e-6).sum())
    solid = arr[arr > 1e-6]
    fu, fd = np.array(face_up, float), np.array(face_dn, float)
    flat = [float(np.ptp(x[~np.isnan(x)])) if np.isfinite(x).sum() >= 2 else 0.0 for x in (fu, fd)]
    return dict(
        min=float(solid.min()) if len(solid) else 0.0,
        max=float(solid.max()) if len(solid) else 0.0,
        spread=float(np.ptp(solid)) if len(solid) else 0.0,
        med=float(np.median(solid)) if len(solid) else 0.0,
        n=len(th), void=n_void, n_rays=n_rays,
        flatness=float(max(flat)),                 # 两端取较差的一端（保守）
        flatness_ends=[round(x, 5) for x in flat],
        solid_frac=float(len(solid) / len(th)) if th else 0.0,
        ring_area_mm2=float(math.pi * (float(r_out) ** 2 - float(r_in) ** 2)),
        r_in=float(r_in), r_out=float(r_out), n_ring=len(radii), n_ang=int(n_ang))


def seat_spread(V, F, cyl, r_ring, back=80.0):
    """旧接口（只有厚度那几个量）。保留给还没换到 seat_profile 的调用方。"""
    p = seat_profile(V, F, cyl, r_ring, r_ring, back=back, n_ring=1)
    return p["min"], p["max"], p["spread"], p["med"], p["n"], p["void"]


class PartGeom:
    """一个导出 STL 的几何视图，带缓存。L2/L5 共用。"""

    def __init__(self, path: Path):
        self.path = path
        self.V, self.F = read_stl_arrays(path)
        self.cyls = detect_cylinders(self.V, self.F)
        self.reach = float(np.linalg.norm(self.V.max(axis=0) - self.V.min(axis=0))) + 10.0
        self.holes = [c for c in self.cyls if c["hole"]]
        self.bosses = [c for c in self.cyls if not c["hole"]]
        self._horn: dict[float, list] = {}
        # 面的外接球（质心 + 半径）：给 geom 探针做局部面片预筛，
        # 否则每条射线都要扫全部三角面（L06 有 6 万多个面）。
        a, b, c = self.V[self.F[:, 0]], self.V[self.F[:, 1]], self.V[self.F[:, 2]]
        self.fc = (a + b + c) / 3.0
        self.fr = np.maximum.reduce([np.linalg.norm(a - self.fc, axis=1),
                                     np.linalg.norm(b - self.fc, axis=1),
                                     np.linalg.norm(c - self.fc, axis=1)])

    @property
    def bbox(self):
        return self.V.min(axis=0), self.V.max(axis=0)

    def faces_near_axis(self, p, ax, R):
        """离「过 p 沿 ax 的直线」不超过 R 的面 —— 探针只可能打到这些。"""
        d = self.fc - p
        perp = np.linalg.norm(d - np.outer(d @ ax, ax), axis=1)
        return self.F[perp <= R + self.fr]

    def horn_patterns(self, hole_d, dtol):
        """找「6 个同径孔均布在一个圆上、轴向平行、同一张面」的阵列 —— S288 舵盘 6 孔。
        纯阵列自身的不变量，不依赖任何坐标系。RANSAC：三点定圆 → 收内点 → 恰好 6 个才算。"""
        key = round(float(hole_d), 3)
        if key in self._horn:
            return self._horn[key]
        cand = [c for c in self.holes if abs(2 * c["r"] - hole_d) <= dtol and c["cover"] > 180.0]
        pats = []
        # 按轴向 + 轴向位置分簇
        clusters: list[list] = []
        for c in cand:
            for cl in clusters:
                a = cl[0]
                if abs(abs(float(np.dot(a["axis"], c["axis"]))) - 1.0) > 1e-3:
                    continue
                if abs(float(np.dot(c["mid"] - a["mid"], a["axis"]))) > 1.5:
                    continue
                cl.append(c)
                break
            else:
                clusters.append([c])
        for cl in clusters:
            if len(cl) < 6:
                continue
            ax = cl[0]["axis"]
            e1, e2 = _basis(ax)
            pts = [np.array([c["mid"] @ e1, c["mid"] @ e2]) for c in cl]
            left = list(range(len(cl)))
            for tri in combinations(range(min(len(cl), 14)), 3):
                if not set(tri) <= set(left):
                    continue
                P3 = np.array([pts[i] for i in tri])
                cx, cy, rad, _res = _fit_circle(P3)
                if rad < 1.0 or rad > 40.0:
                    continue
                inl = [i for i in left
                       if abs(math.hypot(pts[i][0] - cx, pts[i][1] - cy) - rad) <= _PATTERN_RES]
                if len(inl) != 6:
                    continue
                Pc = np.array([pts[i] for i in inl])
                cx, cy, rad, rres = _fit_circle(Pc)
                ang = np.sort(np.mod(np.degrees(np.arctan2(Pc[:, 1] - cy, Pc[:, 0] - cx)), 360.0))
                gaps = np.diff(np.concatenate([ang, [ang[0] + 360.0]]))
                pats.append(dict(holes=[cl[i] for i in inl], axis=ax, center2=(cx, cy),
                                 r_pattern=rad, radial_res=rres,
                                 gaps=[float(x) for x in gaps], angles=[float(x) for x in ang]))
                left = [i for i in left if i not in inl]
                if len(left) < 6:
                    break
        self._horn[key] = pats
        return pats


def geom(path: Path) -> PartGeom:
    k = str(path)
    if k not in _GEOM_CACHE:
        _GEOM_CACHE[k] = PartGeom(path)
    return _GEOM_CACHE[k]


# ── spec_verbatim 解析（geom 段接上之后只剩追溯用途，保留给别的层引用）──────
_RX_D = re.compile(r"(?:(\d+)\s*[×x]\s*)?Ø\s*(\d+(?:\.\d+)?)")
_RX_DEPTH = re.compile(r"(?:深|沉)\s*(\d+(?:\.\d+)?)")


def spec_diameters(spec: str):
    """[(数量, 直径)…]，按出现顺序。没写 n× 的记 1。"""
    return [(int(m.group(1)) if m.group(1) else 1, float(m.group(2))) for m in _RX_D.finditer(spec)]


def spec_is_through(spec: str) -> bool:
    return ("打穿" in spec) or ("通孔" in spec) or ("穿" in spec) or bool(re.search(r"通(?![道过])", spec))


def spec_depth(spec: str):
    m = _RX_DEPTH.search(spec)
    return float(m.group(1)) if m else None


# ── features.yaml:geom 段 → 探针 ───────────────────────────────────────────
AXIS_VEC = {"x": (1.0, 0.0, 0.0), "+x": (1.0, 0.0, 0.0), "-x": (-1.0, 0.0, 0.0),
            "y": (0.0, 1.0, 0.0), "+y": (0.0, 1.0, 0.0), "-y": (0.0, -1.0, 0.0),
            "z": (0.0, 0.0, 1.0), "+z": (0.0, 0.0, 1.0), "-z": (0.0, 0.0, -1.0)}


def axis_of(a):
    """geom.axis（'x' / '-z' / [x,y,z]）→ 单位向量；给不出返回 None。"""
    v = None
    if isinstance(a, str):
        v = AXIS_VEC.get(a.strip())
    elif isinstance(a, (list, tuple)) and len(a) == 3:
        try:
            v = tuple(float(x) for x in a)
        except (TypeError, ValueError):
            v = None
    if v is None:
        return None
    w = np.array(v, float)
    n = float(np.linalg.norm(w))
    return w / n if n > 1e-9 else None


def pt3(p):
    if isinstance(p, (list, tuple)) and len(p) == 3:
        try:
            return np.array([float(x) for x in p])
        except (TypeError, ValueError):
            return None
    return None


def gv(node):
    """{v:…, src:…} → 值；不是数就 None（core.num 的薄封装，只要值）。"""
    v, _s = num(node)
    return v


# 特征是"挖出来的"还是"堆上去的"。这是分类不是判据 —— 决定射线从哪边打、
# 声明区域里应该有料还是没料。两边都没有的类（split_face）保持 None，判 unknown。
VOID_CLASSES = {"screw_hole", "horn_hole", "pilot_hole", "counterbore", "bearing_bore",
                "tool_channel", "bore", "spot_face", "snap_groove", "flange_relief",
                "servo_envelope_cut", "sweep_cut", "clearance_cut", "shell_clearance_cut",
                "slide_corridor", "cut", "window", "cavity", "slot", "cable_channel",
                "engraving"}      # 2026-09-19 hr15：件号刻字/点码（shape=prism，见 _prism）
SOLID_CLASSES = {"journal", "boss", "snap", "driven_disc", "patch", "fill", "plate",
                 "back_plate", "carrier_back_plate", "carrier_wall", "carrier_front_plate",
                 "deck", "battery_bay_floor", "battery_bay_wall", "battery_rail", "foot",
                 "grip", "web", "wall", "lip", "cage", "arm", "bridge", "rail"}
# 兼容旧命名（别的层可能 import）
HOLE_CLASSES = {"screw_hole", "horn_hole", "pilot_hole", "counterbore", "bearing_bore",
                "tool_channel", "bore", "spot_face", "snap_groove"}
# 要往里插东西（螺丝、起子、轴、轴承）的孔，才谈得上"整段通规"。
# 让位槽 / 扫掠让位 / 空腔不是通规对象：它们的功能是"别挡着"，不是"能穿过去"。
GAUGE_CLASSES = {"screw_hole", "horn_hole", "pilot_hole", "counterbore", "bearing_bore",
                 "tool_channel", "bore", "spot_face"}
SHAFT_CLASSES = {"journal", "boss", "snap", "driven_disc", "patch"}
CYL_CLASSES = HOLE_CLASSES | SHAFT_CLASSES
BBOX_KEYS = ("bbox_mm", "zone_bbox_mm", "region_bbox_mm", "swept_box_bbox_mm",
             "segments_bbox_mm", "bridge_bbox_mm", "yoke_bbox_mm", "swept_box_bbox_mm",
             # 滑配走廊等扫掠体的 export_local 外包盒，{lo:[x,y,z], hi:[x,y,z]} 形式（L1 _feature_boxes 同款读法）
             "box_export_local")


def polarity(kind, gd):
    if kind in VOID_CLASSES:
        return "void"
    if kind in SOLID_CLASSES:
        return "solid"
    df = str(gd.get("derived_from") or "")      # ankle_split.py 那两条 key 只能靠源码行区分
    if "-=" in df:
        return "void"
    if "+=" in df:
        return "solid"
    return None


_INHERIT = ("shape", "nominal_d_mm", "inner_d_mm", "depth_mm", "depth_kind", "through",
            "axis", "pos", "axial_span_mm", "count", "pitch_r_mm", "half_only", "min_dirs", "role",
            "hole_positions_mm", "positions_mm", "thickness_mm", "effective_count", "prism") + BBOX_KEYS


def probe_units(gd):
    """geom 段 → 可独立量的探针单元列表。

    instances[] 逐条继承顶层字段后覆盖；hole_positions_mm / positions_mm 再展开成一孔一条。
    每条带 _span_t = (沿轴起, 沿轴止)（相对该条的 pos，标量），由 axial_span_mm 投影得到。
    """
    base = {k: gd.get(k) for k in _INHERIT}
    insts = gd.get("instances")
    raw = []
    if isinstance(insts, list) and insts:
        for it in insts:
            if not isinstance(it, dict):
                continue
            m = dict(base)
            m.update({k: v for k, v in it.items() if v is not None})
            raw.append(m)
    else:
        raw.append(dict(base))
    out, seen = [], set()
    for m in raw:
        ax = axis_of(m.get("axis"))
        ref = pt3(m.get("pos"))
        span = m.get("axial_span_mm")
        st = None
        if ax is not None and ref is not None and isinstance(span, (list, tuple)) and len(span) == 2:
            a, b = pt3(span[0]), pt3(span[1])
            if a is not None and b is not None:
                t0, t1 = float((a - ref) @ ax), float((b - ref) @ ax)
                st = (min(t0, t1), max(t0, t1))
        if st is None:
            # 没写 axial_span_mm 的条目：pos 是轴向中点（有 span 的条目全都是这个约定，
            # 例如 L01-F04 pos z=15.35 正是 −5.65..36.35 的中点），用 depth_mm 补出区间。
            dep = gv(m.get("depth_mm"))
            if dep is None:
                dep = gv(m.get("thickness_mm"))
            if isinstance(dep, (int, float)) and dep > 0:
                st = (-float(dep) / 2.0, float(dep) / 2.0)
                m["_span_from_depth"] = True
        m["_span_t"] = st
        m["_axis"] = ax
        pts = m.get("hole_positions_mm") or m.get("positions_mm")
        cand = []
        if isinstance(pts, list) and pts and isinstance(pts[0], (list, tuple)):
            for p in pts:
                q = dict(m)
                q["_pos"] = pt3(p)
                cand.append(q)
        else:
            q = dict(m)
            q["_pos"] = ref
            cand.append(q)
        for q in cand:
            if q["_pos"] is None and isinstance(q.get("prism"), dict):
                q["_pos"] = pt3(q["prism"].get("origin"))
            if q["_pos"] is None:
                bb = next((q.get(k) for k in BBOX_KEYS if q.get(k)), None)
                lohi = _bbox(bb)
                if lohi is not None:
                    q["_pos"] = (lohi[0] + lohi[1]) / 2.0
            key = (None if q["_pos"] is None else tuple(np.round(q["_pos"], 4)),
                   None if q["_axis"] is None else tuple(np.round(q["_axis"], 4)),
                   gv(q.get("nominal_d_mm")))
            if key in seen and key[0] is not None:
                continue
            seen.add(key)
            out.append(q)
    return out


def _bbox(bb):
    """[[lo],[hi]] 或 {lo:[…], hi:[…]}（geom.box_export_local，与 l1_printable._feature_boxes 同一读法）→ (lo, hi)。"""
    if isinstance(bb, dict):
        bb = (bb.get("lo"), bb.get("hi"))
    if not (isinstance(bb, (list, tuple)) and len(bb) == 2):
        return None
    lo, hi = pt3(bb[0]), pt3(bb[1])
    if lo is None or hi is None:
        return None
    return np.minimum(lo, hi), np.maximum(lo, hi)


def _scan_stations(g, pos, ax, span_t):
    """把声明的轴向区间与件包围盒在该轴上的投影取交，均分成 _SCAN_STATIONS 站。"""
    proj = g.V @ ax
    p0 = float(pos @ ax)
    lo_p, hi_p = float(proj.min()) - p0, float(proj.max()) - p0
    lo, hi = span_t if span_t else (-_PROBE_WINDOW, _PROBE_WINDOW)
    lo, hi = max(lo, lo_p), min(hi, hi_p)
    if hi <= lo:
        return np.array([max(min(0.0, hi_p), lo_p)]), False
    ins = 0.03 * (hi - lo)
    return np.linspace(lo + ins, hi - ins, _SCAN_STATIONS), True


def ring_scan(g, pos, ax, d_nom, pol, span_t, dtol):
    """沿声明轴扫描：每站 _SCAN_DIRS 向，看名义半径处有没有一整圈面。

    每一向从轴心往外打一条射线，收下窗内**全部**穿越半径，取离名义半径最近的那个。
    不取"最近命中"是因为同心特征会互相遮挡（Ø4.2 沉孔套在 Ø2.2 过孔外、Ø14.3 让位环
    套在 Ø5 中心孔外），取最近只会永远量到最里面那圈。判据没变：名义半径处必须有整圈面。
    挖出来的（孔/环）与堆上去的（盘/柱）用同一条射线：轴心在料里时首个穿越就是外表面。
    返回 dict(n_rays, matches, n_dirs, radii, t_best, r_med, in_part)。
    """
    r_nom = float(d_nom) / 2.0
    R = r_nom + _PROBE_WINDOW
    Fl = g.faces_near_axis(pos, ax, R)
    ts, in_part = _scan_stations(g, pos, ax, span_t)
    res = dict(n_rays=0, matches=0, n_dirs=_SCAN_DIRS, radii=[], t_best=None,
               r_med=None, in_part=in_part, n_faces=int(len(Fl)), ts=[float(x) for x in ts],
               stations=[])
    if len(Fl) == 0:
        return res
    e1, e2 = _basis(ax)
    dirs = [math.cos(2 * math.pi * k / _SCAN_DIRS) * e1 + math.sin(2 * math.pi * k / _SCAN_DIRS) * e2
            for k in range(_SCAN_DIRS)]
    best = None
    for t in ts:
        c = pos + ax * float(t)
        radii, first = [], []
        for d in dirs:
            tt, _nd = ray_hits(g.V, Fl, c, d)
            cand = [float(x) for x in tt if x <= R]
            radii.append(min(cand, key=lambda x: abs(2 * x - float(d_nom))) if cand else None)
            first.append(min(cand) if cand else None)          # 离轴最近的那堵壁 = 该向的内接半径
        res["n_rays"] += len(dirs)
        m = sum(1 for r in radii if r is not None and abs(2 * r - float(d_nom)) <= dtol)
        dev = max((abs(2 * r - float(d_nom)) for r in radii if r is not None), default=1e9)
        # 每一站都记下来 —— FG07 的教训：只留"最好的一站"，中段缩径就永远看不见
        res["stations"].append(dict(t=float(t), matches=m, dev=float(dev),
                                    r_first_min=(min(x for x in first if x is not None)
                                                 if any(x is not None for x in first) else None)))
        if best is None or (m, -dev) > (best[0], -best[1]):
            best = (m, dev, radii, float(t))
    if best is None:
        return res
    m, _dev, radii, t = best
    fin = [r for r in radii if r is not None]
    res.update(matches=m, radii=radii, t_best=t,
               r_med=(float(np.median(fin)) if fin else None))
    return res


# ── hr41 落盘 2026-09-25（主设计 Lane C）：六角螺母穴（kind=cavity、shape=prism_array、geom.hex_prism；首例 J02-F04 / J03-F03）──────────
#   以前按 nominal_d_mm（= 对边 4.2）当圆孔探：ring_scan 24 向找 r 2.1 的整圈，六角的角在 r 2.42 → 永远 0/N 处到位（假红）。
#   现在：对边 af、一个平面的外法线 n（export_local，⊥ 轴，逐位置声明）。每站 12 向 = 6 个平面法向（期望半径 af/2）+ 6 个角向（法向转 30°，
#   期望 af/(2·cos30°)）；每向同 ring_scan 取窗内离期望半径最近的穿越。对边 = 相对两平面半径之和（3 对）、对角 = 相对两角半径之和（3 对）。
#   站选法同 ring_scan（命中向最多、偏差最小）；逐向容差 = 直径当量 ±hole_diameter_mm（与圆孔同一个桶）。只有写了 geom.hex_prism 才走这里。
_HEX_COS30 = math.cos(math.pi / 6)


def hex_scan(g, pos, ax, af, flat_n, span_t, dtol):
    r_f = float(af) / 2.0
    r_c = r_f / _HEX_COS30
    R = r_c + _PROBE_WINDOW
    Fl = g.faces_near_axis(pos, ax, R)
    ts, in_part = _scan_stations(g, pos, ax, span_t)
    res = dict(n_rays=0, matches=0, n_dirs=12, t_best=None, af=None, ac=None, radii_f=[], radii_c=[],
               in_part=in_part, n_faces=int(len(Fl)), ts=[float(x) for x in ts], stations=[], r_f=r_f, r_c=r_c)
    if len(Fl) == 0:
        return res
    a = np.asarray(ax, float) / np.linalg.norm(ax)
    n = np.asarray(flat_n, float)
    n = n - a * float(n @ a)
    n = n / np.linalg.norm(n)
    b = np.cross(a, n)
    dirs_f = [math.cos(k * math.pi / 3) * n + math.sin(k * math.pi / 3) * b for k in range(6)]
    dirs_c = [math.cos(k * math.pi / 3 + math.pi / 6) * n + math.sin(k * math.pi / 3 + math.pi / 6) * b for k in range(6)]
    best = None
    for t in ts:
        c = pos + a * float(t)
        rf, rc = [], []
        for dset, rexp, out in ((dirs_f, r_f, rf), (dirs_c, r_c, rc)):
            for d in dset:
                tt, _nd = ray_hits(g.V, Fl, c, d)
                cand = [float(x) for x in tt if x <= R]
                out.append(min(cand, key=lambda x: abs(x - rexp)) if cand else None)
        res["n_rays"] += 12
        m = (sum(1 for r in rf if r is not None and abs(2 * r - 2 * r_f) <= dtol)
             + sum(1 for r in rc if r is not None and abs(2 * r - 2 * r_c) <= dtol))
        dev = max([abs(2 * r - 2 * r_f) for r in rf if r is not None] + [abs(2 * r - 2 * r_c) for r in rc if r is not None],
                  default=1e9)
        afs = [rf[k] + rf[k + 3] for k in range(3) if rf[k] is not None and rf[k + 3] is not None]
        acs = [rc[k] + rc[k + 3] for k in range(3) if rc[k] is not None and rc[k + 3] is not None]
        res["stations"].append(dict(t=float(t), matches=m, dev=float(dev), af=afs, ac=acs))
        if best is None or (m, -dev) > (best[0], -best[1]):
            best = (m, dev, rf, rc, afs, acs, float(t))
    if best is None:
        return res
    m, _dev, rf, rc, afs, acs, t = best
    res.update(matches=m, t_best=t, radii_f=rf, radii_c=rc, af=afs, ac=acs)
    return res


def _hex_cavity(res, g, pid, fid, gd, units, hexspec, dtol, dep_lo, dep_hi, spec, tag, prov):
    """六角穴的 present（每处最好一站 12 向全到位）+ 通盲 + 盲深（与圆孔盲穴同口径：step_probe，内圈 = 内切圆 −_STEP_OFFSET、外圈 = 外接圆 +_STEP_OFFSET）。
    声明写坏（非六角 / 对边缺 src / 与 nominal_d_mm 不一致 / 法向条数或方向不对）→ present unknown，不回退圆孔探针、不猜。返回本条射线数。"""
    hp = f"features.yaml:{fid}.geom.hex_prism"
    probs = []
    if not isinstance(hexspec, dict):
        hexspec = {}
        probs.append("geom.hex_prism 必须是 {sides, across_flats_mm, flat_normals_export_local}")
    if hexspec and hexspec.get("sides") != 6:
        probs.append(f"sides={hexspec.get('sides')!r}（本判据只认 6）")
    afn = hexspec.get("across_flats_mm")
    af = gv(afn)
    if not (isinstance(afn, dict) and afn.get("src")) or not isinstance(af, (int, float)) or isinstance(af, bool) \
            or not math.isfinite(float(af)) or float(af) <= 0:
        probs.append(f"across_flats_mm={afn!r} 缺值/非正/缺 src")
        af = None
    dn = gv(gd.get("nominal_d_mm"))
    if af is not None and dn is not None and abs(float(dn) - float(af)) > 1e-9:
        probs.append(f"across_flats_mm {af} ≠ nominal_d_mm {dn}（名义就是对边，两处必须一致）")
    cu = [u for u in units if u.get("_pos") is not None and u.get("_axis") is not None]
    fns = hexspec.get("flat_normals_export_local")
    normals = []
    if not cu:
        probs.append("没有可探的位置（pos/positions_mm + axis）")
    elif not isinstance(fns, list) or len(fns) != len(cu):
        probs.append(f"flat_normals_export_local 条数 {len(fns) if isinstance(fns, list) else None} ≠ 位置数 {len(cu)}")
    else:
        for u, v in zip(cu, fns):
            try:
                n = np.asarray(v, float)
                a = np.asarray(u["_axis"], float) / np.linalg.norm(u["_axis"])
                ok = n.shape == (3,) and bool(np.isfinite(n).all()) and float(np.linalg.norm(n)) > 0 \
                    and abs(float(n @ a)) / float(np.linalg.norm(n)) <= 1e-6
            except (TypeError, ValueError):
                ok = False
            if not ok:
                probs.append(f"平面法向 {v!r} 不是 ⊥ 轴的有限非零三维向量")
            normals.append(v)
    if probs:
        res.unknown(pid, f"{fid}:present", "六角穴声明写坏：" + "；".join(probs) + " —— 不猜对边/朝向，也不回退圆孔探针",
                    provenance=hp)
        return 0
    af = float(af)
    scans = []
    for u, fn in zip(cu, normals):
        sc = hex_scan(g, u["_pos"], u["_axis"], af, fn, u.get("_span_t"), dtol)
        sc["_u"] = u
        scans.append(sc)
    n_rays = sum(s["n_rays"] for s in scans)
    ok = [s for s in scans if s["matches"] == 12]
    worst = min(scans, key=lambda s: s["matches"])
    af_all = [round(x, 4) for s in scans for x in (s["af"] or [])]
    ac_all = [round(x, 4) for s in scans for x in (s["ac"] or [])]
    ac_nom = af / _HEX_COS30
    res.add(subject=pid, check=f"{fid}:present", state=PASS if len(ok) == len(scans) and n_rays else FAIL, severity=BLOCK,
            measured=f"{len(ok)}/{len(scans)} 处到位",
            criterion=(f"六角穴（geom.hex_prism）{len(scans)} 个位置，每处沿轴 {_SCAN_STATIONS} 站 × 12 向（6 平面法向 + 6 角向）"
                       f"都要量到对边 {af} / 对角 {ac_nom:.4f}（= 对边/cos30°），逐向直径当量 ±{dtol}（feature_check_tolerances.hole_diameter_mm）"),
            evidence_n=n_rays,
            detail=(f"最好一站实测对边 {af_all}、对角 {ac_all}；最差一处 {worst['matches']}/12 向命中，平面向半径 "
                    f"{[None if r is None else round(r, 4) for r in worst['radii_f']]}、角向半径 "
                    f"{[None if r is None else round(r, 4) for r in worst['radii_c']]}（窗内三角面 {worst['n_faces']} 个）｜spec：{spec}{tag}"),
            provenance=hp + " | " + prov)
    # 通 / 盲、盲深：与圆孔盲穴同一判据，只把内/外圈探针换成内切圆 −_STEP_OFFSET、外接圆 +_STEP_OFFSET
    r_f, r_c = af / 2.0, af / 2.0 / _HEX_COS30
    ri, ro = max(r_f - _STEP_OFFSET, r_f * 0.4), r_c + _STEP_OFFSET
    rows = []
    for u in cu:
        sp = step_probe(g, u["_pos"], u["_axis"], af, u.get("_span_t"), ri=ri, ro=ro)
        if sp:
            n_rays += sp["n_rays"]
        rows.append((u, sp))
    ev = sum(sp["n_rays"] for _u, sp in rows if sp)
    live = [(u, sp) for u, sp in rows if sp and sp["n_dirs"] > 0]
    dead = [(u, sp) for u, sp in rows if sp and sp["n_dirs"] == 0]
    if not rows or all(sp is None for _u, sp in rows):
        res.unknown(pid, f"{fid}:through", "声明位置附近取不到三角面，通盲量不出来", provenance=prov)
        return n_rays
    if dead and not live:
        res.add(subject=pid, check=f"{fid}:through", state=FAIL, severity=BLOCK, measured=f"0/{len(rows)} 处有料板",
                criterion=f"声明的轴向区间里，六角外接圆之外（r={ro:.2f}）必须有料", evidence_n=ev,
                detail=f"{len(dead)} 处 × {_STEP_DIRS} 向平行轴射线，声明区间内一向都没碰到料板｜spec：{spec}", provenance=prov)
        return n_rays
    bad, tot_d, thr_d, decl = [], 0, 0, []
    for u, sp in live:
        tot_d += sp["n_dirs"]
        thr_d += sp["n_through"]
        t_decl = u.get("through")
        decl.append(t_decl)
        if t_decl is None:
            continue
        if t_decl and sp["n_through"] != sp["n_dirs"]:
            bad.append(u)
        if (not t_decl) and sp["n_through"] > 0:
            bad.append(u)
    if all(x is None for x in decl):
        res.unknown(pid, f"{fid}:through", "geom.through 是 null，通盲没有声明值可比", provenance=prov, severity=WARN)
    else:
        res.add(subject=pid, check=f"{fid}:through", state=PASS if not bad else FAIL, severity=BLOCK,
                measured=f"{thr_d}/{tot_d} 向真通",
                criterion=(f"geom.through={sorted({str(x) for x in decl})} → {len(live)} 处 × {_STEP_DIRS} 向（内圈 r {ri:.2f} = 内切圆 −{_STEP_OFFSET}、"
                           f"外圈 r {ro:.2f} = 外接圆 +{_STEP_OFFSET}）：声明通的每一向都不剩料，声明不通的每一向都还剩料"),
                evidence_n=ev, detail=f"掏空量 {[round(x, 3) for _u, sp in live for x in sp['voids']][:8]}｜spec：{spec}",
                provenance=prov)
    blind = [(u, sp) for u, sp in live if u.get("depth_kind") == "blind_depth" and gv(u.get("depth_mm")) is not None
             and u.get("through") is not True]
    if blind:
        dv = float(gv(blind[0][0].get("depth_mm")))
        lo2, hi2 = dv + float(dep_lo), dv + float(dep_hi)
        meds = [float(np.median(sp["voids"])) for _u, sp in blind if sp["voids"]]
        nok = sum(1 for x in meds if lo2 <= x <= hi2)
        res.add(subject=pid, check=f"{fid}:depth", state=PASS if meds and nok == len(meds) else FAIL, severity=BLOCK,
                measured=(round(max(meds, key=lambda x: abs(x - dv)), 4) if meds else f"0/{len(blind)} 处量到台阶"),
                criterion=f"盲深 {dv} + [{dep_lo}, {dep_hi}]（feature_check_tolerances.hole_depth_mm），{len(blind)} 处全部合格",
                evidence_n=sum(sp["n_dirs"] for _u, sp in blind), detail=f"逐处实测掏空量 {[round(x, 4) for x in meds]}",
                provenance="tolerances.yaml:feature_check_tolerances.hole_depth_mm | " + prov)
    return n_rays


# 导出 STL 顶点是 float32：Ø30 以内一个坐标的表示噪声 < 1e-6 mm，半径→直径再乘 2 后 < 4e-6。
# 这不是判据放宽（阈值一个没动），是不让 14.85 存成 14.8499994 之后被 `min == nominal` 的桶判红。
_NUM_EPS = 1e-5
_FACET_N_MIN = 8            # 棱边法的半径带上界按"至少 8 边"算：真半径 ≤ 射线内接半径 / cos(π/8)


def facet_free_diameter(g, pos, ax, d_nom, sc, need):
    """刻面免疫的直径（棱边法）—— F-L2-2。

    射线从轴心打到刻面**面心**，量到的是内接圆 r·cos(π/N)：64 边的 Ø14.85 量成 14.834，
    对 `target_range.min == nominal` 的桶必红，红的是三角化不是零件。
    圆柱面被切成三角形之后，**纵向棱与端面弦的两个端点都精确落在建模圆上**，所以取
    "两端点离轴等距"的棱边（|Δr| < 1e-3，排除从孔壁扇到外平面的斜棱），在孔身轴向范围内、
    半径落在 [r_ray, r_ray/cos(π/8)]（真半径不可能比射线内接半径小，也不可能比 8 边形的外接半径大）
    的那些取中位数 —— 与分段数无关，与 bore_gauge 的棱边采样同源。
    返回 (直径, 棱边数)；取不到棱边给 (None, 0)，调用方退回射线值并在 detail 里说明。
    """
    r_ray = sc.get("r_med")
    if r_ray is None:
        return None, 0
    sts = sc.get("stations") or []
    ok_t = [s["t"] for s in sts if s["matches"] >= need]
    ts = sc.get("ts") or []
    step = (abs(ts[-1] - ts[0]) / max(1, len(ts) - 1)) if len(ts) >= 2 else 0.5
    if ok_t:
        t_lo, t_hi = min(ok_t) - step / 2, max(ok_t) + step / 2
    elif ts:
        t_lo, t_hi = min(ts), max(ts)
    else:
        return None, 0
    pos = np.asarray(pos, float); ax = np.asarray(ax, float)
    Fl = g.faces_near_axis(pos, ax, float(d_nom) / 2.0 + _PROBE_WINDOW)
    if not len(Fl):
        return None, 0
    P = g.V[Fl]
    A = np.concatenate([P[:, 0], P[:, 1], P[:, 2]]) - pos
    B = np.concatenate([P[:, 1], P[:, 2], P[:, 0]]) - pos
    tA, tB = A @ ax, B @ ax
    rA = np.linalg.norm(A - np.outer(tA, ax), axis=1)
    rB = np.linalg.norm(B - np.outer(tB, ax), axis=1)
    tm = (tA + tB) / 2.0
    band_hi = float(r_ray) / math.cos(math.pi / _FACET_N_MIN)
    keep = ((np.abs(rA - rB) < 1e-3) & (tm >= t_lo) & (tm <= t_hi)
            & (rA >= float(r_ray) - 1e-3) & (rA <= band_hi))
    if not keep.any():
        return None, 0
    r = float(np.median((rA[keep] + rB[keep]) / 2.0))
    return 2.0 * r, int(keep.sum())


def bore_gauge(g, pos, ax, d_nom, sc, need):
    """整段功能通规：Ø d_gauge 的销子能不能从头走到尾。

    FG07 的坑：`ring_scan` 在 21 站里挑**最好**的一站判"孔存在/孔径合格"，
    `step_probe` 的内圈探针又比名义半径小 0.3、从缩径环下面钻过去 ——
    于是 Ø2.2 的孔中段缩到 Ø2.0 三条判据全绿，螺丝插不进去。

    这里判**最坏截面**，两条互补的量：
      d_min_vertex  孔身范围内、离孔轴最近的**三角面棱边采样点**的 2×距离。
                    圆柱面被切成三角形之后，纵向棱边精确落在名义圆上，所以这个量对
                    分段数（刻面化）免疫 —— Ø27 的孔切成 32 段，射线量到的内接圆是
                    26.87（面心内凹），棱边仍是 27.00。只取顶点不行：一根光滑圆柱的
                    顶点全在两端，中段一个都没有。
      n_bad_station 孔身两端之间，整圈没量到名义直径的站数。这条抓的是
                    "某一截面根本不是这个孔"（被料堵住 / 换了直径），与刻面无关。
    孔身范围 = 第一站到最后一站**整圈命中**的区间；区间外是刀具超长的空走，不算。
    返回 None 表示这条孔量不出孔身范围（判不了，调用方按未知处理）。
    """
    sts = sc.get("stations") or []
    ok = [i for i, s in enumerate(sts) if s["matches"] >= need]
    if len(ok) < 2:
        return None
    i0, i1 = ok[0], ok[-1]
    t0, t1 = sts[i0]["t"], sts[i1]["t"]
    bad = [sts[i] for i in range(i0, i1 + 1) if sts[i]["matches"] < need]
    # 顶点扫描：孔身范围内缩掉两端各 _GAUGE_END_SKIP 个站距，避开端面/台阶边缘的顶点
    step = (t1 - t0) / max(1, (i1 - i0))
    skip = min(_GAUGE_END_SKIP * step, _GAUGE_END_FRAC * (t1 - t0))
    a0, a1 = t0 + skip, t1 - skip
    d_vertex, t_vertex, n_pts = None, None, 0
    if a1 > a0:
        Fl = g.faces_near_axis(np.asarray(pos, float), ax, float(d_nom) / 2.0 + _PROBE_WINDOW)
        if len(Fl):
            P = g.V[Fl]                                  # (n,3,3)
            segs = [(P[:, 0], P[:, 1]), (P[:, 1], P[:, 2]), (P[:, 2], P[:, 0])]
            Q = np.concatenate([A + (B - A) * s for A, B in segs
                                for s in np.linspace(0.0, 1.0, _GAUGE_EDGE_SAMPLES)])
            rel = Q - np.asarray(pos, float)
            tv = rel @ ax
            m = (tv >= a0) & (tv <= a1)
            if m.any():
                perp = np.linalg.norm(rel[m] - np.outer(tv[m], ax), axis=1)
                win = perp <= float(d_nom) / 2.0 + _PROBE_WINDOW
                n_pts = int(win.sum())
                if win.any():
                    j = int(np.argmin(perp[win]))
                    d_vertex = float(2.0 * perp[win][j])
                    t_vertex = float(tv[m][win][j])
    st_min = [s["r_first_min"] for s in sts[i0:i1 + 1] if s["r_first_min"] is not None]
    return dict(t0=t0, t1=t1, n_station=i1 - i0 + 1, n_bad=len(bad),
                bad_t=[round(s["t"], 3) for s in bad[:6]],
                d_min_vertex=d_vertex, t_at_min=t_vertex, n_edge_pts=n_pts,
                d_min_ray=(2.0 * min(st_min) if st_min else None),
                n_rays=(i1 - i0 + 1) * sc.get("n_dirs", _SCAN_DIRS) + n_pts)


def _spans_at(g, Fl, pos, ax, u, rad, w0, w1):
    """沿轴、在半径 rad 的平行线上取料段（区间用相对 w0 的偏移表示）。"""
    o = pos + ax * w0 + u * rad
    t, nd = ray_hits(g.V, Fl, o, ax)
    _tot, segs = solid_spans(t, nd)
    L = w1 - w0
    return [(max(s0, 0.0), min(s1, L)) for s0, s1 in segs if s1 > 0 and s0 < L]


def step_probe(g, pos, ax, d_nom, span_t, ri=None, ro=None):
    """量这把刀在**声明的轴向区间**里到底掏掉了多少料 —— 方向无关，不假设下刀方向。
    ri / ro（hr41 落盘 2026-09-25，主设计 Lane C）：内/外圈探针半径显式给定时用它（六角穴：内切圆 −0.3 / 外接圆 +0.3）；
    不给 = 原式（圆孔 r∓_STEP_OFFSET），现有调用逐字节不变。

    同一向打两条平行轴的射线：一条在名义半径**之外**（ro，还该是料），一条在**之内**（ri，
    该被掏空）。外圈那条给出这一处的料板；料板与声明区间的交集就是判据区间 region，
    region 里内圈还剩多少料 = keep，掏掉多少 = void。keep≈0 就是这一向真通。

    这样既不依赖 axial_span_mm 的书写顺序（L01-F05 沉孔的刀往 −z 走，span 却按坐标升序写），
    也能量沉孔（Ø4.2 套在 Ø2.2 过孔外，内圈正好落在两径之间）。
    外圈在声明区间里一点料都没有的向单独计 n_noslab —— 那说明这把刀落在了别人已经掏空的
    地方（L01-F09 的 Ø4.2 全在原版 Ø4.84 沉孔里），是结论不是异常。
    """
    r = float(d_nom) / 2.0
    if ri is None or ro is None:
        ri, ro = max(r - _STEP_OFFSET, r * 0.4), r + _STEP_OFFSET
    Fl = g.faces_near_axis(pos, ax, ro + _PROBE_WINDOW)
    if len(Fl) == 0:
        return None
    proj = g.V @ ax
    p0 = float(pos @ ax)
    lo_p, hi_p = float(proj.min()) - p0, float(proj.max()) - p0
    s0, s1 = (span_t if span_t else (-0.5, 0.5))
    # 起点必须在件外：沿轴退到包围盒之外，这样"第一个面就是进料"永远成立，
    # 不用赌网格闭合（奇偶法在预筛过的面子集上是错的：子集根本不闭合）。
    w0, w1 = lo_p - 1.0, hi_p + 1.0
    s0, s1 = s0 - w0, s1 - w0
    L = w1 - w0
    e1, e2 = _basis(ax)
    voids, n_through, n_dirs, n_noslab = [], 0, 0, 0

    def spans(rad, u):
        """料段。布尔缝会在同一个 t 上留下两张同向的重合面（L07 的 6700 座就有），
        不去重的话 solid_spans 的进出计数会一直不归零，整段料被吞掉 → 先按 (t, 朝向) 去重。"""
        t, nd = ray_hits(g.V, Fl, pos + ax * w0 + u * rad, ax)
        tt, nn, pt, ps = [], [], None, None
        for x, n in zip(t, nd):
            x, sg = float(x), (n < 0)
            if pt is not None and abs(x - pt) < 1e-6 and sg == ps:
                continue
            tt.append(x)
            nn.append(-1.0 if sg else 1.0)
            pt, ps = x, sg
        _tot, segs = solid_spans(np.array(tt), np.array(nn))
        return [(max(x, 0.0), min(y, L)) for x, y in segs if y > 0 and x < L]

    for k in range(_STEP_DIRS):
        aa = 2 * math.pi * k / _STEP_DIRS
        u = math.cos(aa) * e1 + math.sin(aa) * e2
        outer = spans(ro, u)
        best = max(outer, key=lambda sg: max(0.0, min(sg[1], s1) - max(sg[0], s0)),
                   default=None)
        ov = 0.0 if best is None else max(0.0, min(best[1], s1) - max(best[0], s0))
        if ov <= 1e-6:
            n_noslab += 1
            continue
        n_dirs += 1
        inner = spans(ri, u)
        r0, r1 = max(best[0], s0), min(best[1], s1)
        keep = sum(max(0.0, min(y, r1) - max(x, r0)) for x, y in inner)
        voids.append((r1 - r0) - keep)
        # 真通 = 声明区间里一点料都不剩，而且**紧接着区间外 _THROUGH_LOOK 之内**料也没回来
        # （料马上回来 = 刀没穿透，H01-F12 的 Ø1.6 底孔 4.5 打进 6 长的柱子、
        #   T01-F16 仓底 2.7 的刀在 3.0 厚的板上留 0.3 都是这种）。
        q1 = min(best[1], s1 + _THROUGH_LOOK)
        if sum(max(0.0, min(y, q1) - max(x, r0)) for x, y in inner) <= 1e-6:
            n_through += 1
    return dict(n_dirs=n_dirs, n_noslab=n_noslab, n_through=n_through, voids=voids,
                n_rays=2 * _STEP_DIRS)


def occupancy(g, pts):
    """点在不在料里：三个非轴对齐方向各打一条射线数穿越次数，多数表决（射线数 = 3×点数）。"""
    out = []
    for p in pts:
        votes = 0
        for d in _PARITY_DIRS:
            d = d / np.linalg.norm(d)
            t, _nd = ray_hits(g.V, g.F, np.asarray(p, float), d)
            votes += int(len(t) % 2 == 1)
        out.append(votes >= 2)
    return out


# ── 2026-09-19 hr15：刻字/点码 = 平面上的多边形凹槽（shape: prism）──────────────────────────
# geom.prism = {origin, right, up, normal（export_local，单位向量）, depth_mm, polygons: [{exterior: [[u,v],…], holes: [[[u,v],…],…]},…]}
# 多边形是面内坐标（相对 origin，沿 right/up）。判据：① 每个笔画多边形的代表点在槽深一半处必须没有料（occupancy）；
# ② 保留区/截面判据把 prism 体（多边形外扩 _KEEP_MARGIN × 深度 −depth−margin..+margin）当声明挖除扣掉（declared_voids）。
def _prism(u):
    pz = u.get("prism") if isinstance(u, dict) else None
    if not isinstance(pz, dict):
        return None
    try:
        o = pt3(pz.get("origin")); r = pt3(pz.get("right")); up = pt3(pz.get("up")); n = pt3(pz.get("normal"))
        d = float(gv(pz.get("depth_mm")) if isinstance(pz.get("depth_mm"), dict) else pz.get("depth_mm"))
    except (TypeError, ValueError):
        return None
    if o is None or r is None or up is None or n is None or not (d > 0):
        return None
    from shapely.geometry import Polygon
    polys = []
    for item in pz.get("polygons") or []:
        if isinstance(item, dict):
            ext, holes = item.get("exterior"), item.get("holes") or []
        else:
            ext, holes = item, []
        if not isinstance(ext, list) or len(ext) < 3:
            continue
        pg = Polygon([(float(a), float(b)) for a, b in ext], [[(float(a), float(b)) for a, b in h] for h in holes if len(h) >= 3])
        if pg.is_valid and pg.area > 1e-6:
            polys.append(pg)
    if not polys:
        return None
    r = r / np.linalg.norm(r); up = up / np.linalg.norm(up); n = n / np.linalg.norm(n)
    return dict(o=o, r=r, up=up, n=n, d=d, polys=polys)


def _prism_points(pz):
    """笔画代表点（每个多边形 1 个，必在笔画里）+ 沿笔画骨架的补充采样（诊断/证据），都放在槽深一半处。"""
    refs, grid = [], []
    for pg in pz["polys"]:
        q = pg.representative_point(); refs.append((q.x, q.y))
        core = pg.buffer(-min(0.2, 0.3 * (pg.area / max(pg.length, 1e-9))))      # 往里缩一点，采样点离笔画边远一些
        if core.is_empty:
            core = pg
        from shapely.geometry import Point
        for g in getattr(core, "geoms", [core]):
            b = g.bounds
            for x in np.arange(b[0] + 0.25, b[2], 0.5):
                for y in np.arange(b[1] + 0.25, b[3], 0.5):
                    if g.contains(Point(x, y)):
                        grid.append((x, y))
    if not grid:
        grid = list(refs)                                                          # 笔画细到网格采不到：代表点兼作证据点
    def to3(pts):
        if not pts:
            return np.zeros((0, 3))
        uv = np.asarray(pts, float)
        return pz["o"] + np.outer(uv[:, 0], pz["r"]) + np.outer(uv[:, 1], pz["up"]) - pz["n"] * (pz["d"] / 2.0)
    return to3(refs), to3(grid)


def occupancy_points(u):
    """判据点 = geom 声明的参考点（pos 与 bbox 中心），外加 box 形的 bbox 网格做旁证。

    只判参考点、不判整片 bbox：布尔是**有先后的**（先堆一块板再在上面开窗、
    先挖仓腔再往里加导轨），整片 bbox 全有料/全没料在成品上本来就不成立，
    拿它当判据会造出一堆假红。网格只写进 detail 当诊断量。
    """
    pz = _prism(u)
    if pz is not None:
        return _prism_points(pz)
    bb = _bbox(next((u.get(k) for k in BBOX_KEYS if u.get(k)), None))
    pos = u.get("_pos")
    refs = []
    if pos is not None:
        refs.append(np.asarray(pos, float))
    if bb is not None:
        c = (bb[0] + bb[1]) / 2.0
        if not refs or float(np.linalg.norm(c - refs[0])) > 1e-6:
            refs.append(c)
    grid = np.zeros((0, 3))
    if bb is not None and u.get("shape") == "box":
        lo, hi = bb
        d = hi - lo
        n = _OCC_GRID
        axes = [np.linspace(lo[i] + _OCC_INSET * d[i], hi[i] - _OCC_INSET * d[i], n)
                for i in range(3)]
        grid = np.stack(np.meshgrid(*axes, indexing="ij"), -1).reshape(-1, 3)
    return (np.array(refs) if refs else np.zeros((0, 3))), grid


# ── 保留区（FG08：一个点有料 ≠ 整块板还在）────────────────────────────────
def declared_voids(feats, pid):
    """同一个件上**明确声明要挖掉**的区域：给保留区判据做扣除。

    两种形状都收：box 类的 bbox（任一 BBOX_KEYS）、cylinder 类的 (轴, 位置, 名义直径, 轴向区间)。
    返回一个谓词 `inside(P) -> bool 数组`。声明为挖除的地方没有料是应该的，不算缺料。
    """
    boxes, cyls, prisms = [], [], []
    for f in feats:
        if export_part(f) != pid:
            continue
        gd = f.get("geom") or {}
        if gd.get("confidence") == "unresolved":
            continue
        if polarity(f.get("check_class") or f.get("kind"), gd) != "void":
            continue
        for u in probe_units(gd):
            pz = _prism(u)
            if pz is not None:
                prisms.append((pz, [pg.buffer(_KEEP_MARGIN) for pg in pz["polys"]]))
                continue
            bb = _bbox(next((u.get(k) for k in BBOX_KEYS if u.get(k)), None))
            if bb is not None:
                boxes.append((bb[0] - _KEEP_MARGIN, bb[1] + _KEEP_MARGIN))
            d = gv(u.get("nominal_d_mm"))
            if d and u.get("_pos") is not None and u.get("_axis") is not None:
                st = u.get("_span_t") or (-1e6, 1e6)
                cyls.append((np.asarray(u["_pos"], float), np.asarray(u["_axis"], float),
                             float(d) / 2.0 + _KEEP_MARGIN, st[0] - _KEEP_MARGIN, st[1] + _KEEP_MARGIN))

    def inside(P):
        P = np.atleast_2d(np.asarray(P, float))
        m = np.zeros(len(P), bool)
        for lo, hi in boxes:
            m |= np.all((P >= lo) & (P <= hi), axis=1)
        for c, a, r, s0, s1 in cyls:
            rel = P - c
            t = rel @ a
            perp = np.linalg.norm(rel - np.outer(t, a), axis=1)
            m |= (perp <= r) & (t >= s0) & (t <= s1)
        if prisms:
            from shapely.geometry import Point
            for pz, grown in prisms:
                rel = P - pz["o"]; w = rel @ pz["n"]
                sel = np.where((w >= -pz["d"] - _KEEP_MARGIN) & (w <= _KEEP_MARGIN) & ~m)[0]
                for i in sel:
                    q = Point(float(rel[i] @ pz["r"]), float(rel[i] @ pz["up"]))
                    if any(g.contains(q) for g in grown):
                        m[i] = True
        return m

    return inside, len(boxes) + len(prisms), len(cyls)


def region_scan(g, lo, hi, axis_i, mask_fn):
    """声明区域**逐列**一条射线：一次同时拿到保留比例和最小有效截面厚度。

    列沿 bbox 的**最短轴**（板/墙的厚度方向）。一条射线给出这一列的全部料段：
      · 列上每个采样点在不在料里 → 保留比例（"整块还在吗"，FG08）
      · 料段与声明区间的重叠长度 → 该列的有效截面厚度（"承压面还有多厚"）
    只在"没有被别的特征声明挖掉"的采样点上判（mask_fn 给 True 的跳过），
    这样一块按设计开了孔的板不会因为孔被判成缺料。

    每列一条射线（不是每点三条奇偶射线）—— 同样的判据，射线数少一个量级。
    """
    ax = np.zeros(3)
    ax[axis_i] = 1.0
    idx = [i for i in range(3) if i != axis_i]
    d = np.asarray(hi, float) - np.asarray(lo, float)
    n = [max(2, min(9, int(np.ceil(d[i] / _KEEP_STEP)) + 1)) for i in idx]
    nz = max(2, min(9, int(np.ceil(d[axis_i] / _KEEP_STEP)) + 1))
    us = np.linspace(lo[idx[0]] + _OCC_INSET * d[idx[0]], hi[idx[0]] - _OCC_INSET * d[idx[0]], n[0])
    vs = np.linspace(lo[idx[1]] + _OCC_INSET * d[idx[1]], hi[idx[1]] - _OCC_INSET * d[idx[1]], n[1])
    s0, s1 = float(lo[axis_i]) + _OCC_INSET * d[axis_i], float(hi[axis_i]) - _OCC_INSET * d[axis_i]
    zs = np.linspace(s0, s1, nz)
    back = float(np.linalg.norm(g.V.max(axis=0) - g.V.min(axis=0))) + 5.0
    thin, n_rays, n_skip, n_tot, n_keep = [], 0, 0, 0, 0
    for u in us:
        for v in vs:
            P = np.zeros((nz, 3))
            P[:, idx[0]], P[:, idx[1]], P[:, axis_i] = u, v, zs
            keepmask = ~np.asarray(mask_fn(P), bool)
            n_skip += int((~keepmask).sum())
            if not keepmask.any():
                continue
            o = P[0] - ax * back
            t, nd = ray_hits(g.V, g.F, o, ax)
            n_rays += 1
            _tot, segs = solid_spans_dedup(t, nd)
            off = back - zs[0]                       # 射线参数 t = 该轴坐标 + off
            for z, k in zip(zs, keepmask):
                if not k:
                    continue
                n_tot += 1
                tz = z + off
                if any(a - 1e-9 <= tz <= b + 1e-9 for a, b in segs):
                    n_keep += 1
            lo_w, hi_w = float(lo[axis_i]) + off, float(hi[axis_i]) + off
            th = sum(min(b, hi_w) - max(a, lo_w) for a, b in segs if b > lo_w and a < hi_w)
            # 整列都被声明的孔槽盖住时不计厚度（那本来就该是空的）
            if keepmask.all():
                thin.append(float(max(0.0, th)))
    return dict(thin=thin, n_rays=n_rays, n_skip=n_skip, n_tot=n_tot, n_keep=n_keep,
                axis=axis_i, n_col=len(us) * len(vs))


# ── 镜像（FG09：只比包围盒和圆孔中心，槽/筋/轴向全没比）──────────────────
def cyl_pairs(gl, gr, M, dr=0.02):
    """左右两件圆柱特征的**双向**对应：位置、半径、轴向、轴向跨度都要对上。

    返回 (逐对误差, 左件没配上的, 右件没配上的, 最大轴向夹角 deg, 最大跨度差)。
    以前只做左→右单向最近邻，右件多出来的特征根本看不见。
    """
    def key(c):
        return dict(r=c["r"], mid=np.asarray(c["mid"], float), hole=c["hole"],
                    axis=np.asarray(c["axis"], float), span=abs(c["t1"] - c["t0"]))

    L = [key(c) for c in gl.cyls]
    Rm = []
    for c in gr.cyls:
        k = key(c)
        k["mid"] = k["mid"] * M
        k["axis"] = k["axis"] * M
        Rm.append(k)
    used_r, errs, ang_max, span_max, unl = set(), [], 0.0, 0.0, 0
    for a in L:
        cand = [(float(np.linalg.norm(b["mid"] - a["mid"])), j) for j, b in enumerate(Rm)
                if b["hole"] == a["hole"] and abs(b["r"] - a["r"]) <= dr and j not in used_r]
        if not cand:
            unl += 1
            continue
        e, j = min(cand)
        used_r.add(j)
        errs.append(e)
        b = Rm[j]
        cosang = abs(float(np.dot(a["axis"], b["axis"])))
        ang_max = max(ang_max, math.degrees(math.acos(min(1.0, cosang))))
        span_max = max(span_max, abs(a["span"] - b["span"]))
    return errs, unl, len(Rm) - len(used_r), ang_max, span_max


def mirror_surface_dev(ctx, pl, pr, M, allow=None, n_pts=1200, seed=20260909):
    """镜像后**非圆柱表面**的差异：双向逐点到对面表面的最近距离取最大。

    槽、筋、台、平面这些没有圆柱面的东西，只有这条能看见（FG09）。
    采样用固定种子的面积加权点，与建模分段无关（元规则 3）。
    `allow` 是 parts.yaml 里**显式声明**允许不对称的包围盒列表
    （`parts[].mirror_asymmetry_allowed: [{bbox_mm: [[..],[..]], why: "..."}]`）——
    没有声明就一处都不许不对称；不许在层里默认放过任何东西。
    """
    import trimesh
    ml, mr = ctx.solid(pl), ctx.solid(pr)
    mrm = mr.copy()
    mrm.apply_transform(np.diag([M[0], M[1], M[2], 1.0]))
    boxes = []
    for a_ in (allow or []):
        bb = _bbox((a_ or {}).get("bbox_mm")) if isinstance(a_, dict) else None
        if bb is not None:
            boxes.append(bb)
    rng = np.random.default_rng(seed)
    out = {"n_allowed_boxes": len(boxes), "n_excluded": 0}
    for tag, (a, b) in (("l2r", (ml, mrm)), ("r2l", (mrm, ml))):
        pts, _fi = trimesh.sample.sample_surface(a, n_pts, seed=int(rng.integers(1 << 30)))
        if boxes:
            keep = np.ones(len(pts), bool)
            for lo, hi in boxes:
                keep &= ~np.all((pts >= lo) & (pts <= hi), axis=1)
            out["n_excluded"] += int((~keep).sum())
            pts = pts[keep]
        if not len(pts):
            raise ValueError(f"镜像 {tag} 方向没有剩余采样点，不能用另一方向的证据补齐")
        # 只要**无符号**的点到面距离：signed_distance 还要额外打射线判内外，这里用不上
        _cp, d, _tid = trimesh.proximity.ProximityQuery(b).on_surface(pts)
        if not np.isfinite(d).all():
            raise ValueError(f"镜像 {tag} 距离含 NaN/Inf；不能让 max(有限值, NaN) 隐藏计算失败")
        out[tag] = float(np.abs(np.asarray(d, float)).max())
        out[tag + "_n"] = int(len(pts))
    return out


# ── tolerances 取值 ────────────────────────────────────────────────────────
def tol_pair(node, key="target_range_mm"):
    """{min:…, max:…} → (min, max)；取不到给 (None, None)。"""
    if not isinstance(node, dict):
        return None, None
    rng = node.get(key) if key in node else node
    if not isinstance(rng, dict):
        return None, None
    return rng.get("min"), rng.get("max")


def walk_fits(fits, prefix="fits"):
    """展开 fits 树 → {点号路径: 桶}。桶 = 带 target_range_mm / target_range_deg 的 dict。"""
    out = {}
    if not isinstance(fits, dict):
        return out
    for k, v in fits.items():
        if not isinstance(v, dict):
            continue
        p = f"{prefix}.{k}"
        if "target_range_mm" in v or "target_range_deg" in v:
            out[p] = v
        else:
            out.update(walk_fits(v, p))
    return out


def mat_key(s):
    """'TPU 95A' → 'tpu'、'PLA' → 'pla'。fits 树的第一层就是按材料分的，这里只做大小写/后缀归一。"""
    return str(s or "").strip().split()[0].lower() if str(s or "").strip() else None


def bucket_material_ok(path, b, part):
    """桶是不是这个件能用的：材料必须对上，打印方向要么写'任意'、要么点了这个件的名。

    FG 审查指出的硬编码：以前只按"名义直径相等"认桶，PLA 的桶会被套到 TPU 件上，
    打印方向（层向决定强度和孔形）也完全不看。README 第 1 节：公差按
    **材料 × 打印方向 × 特征类型**分桶，这里就按这三样认。
    """
    pm = mat_key(part.get("material"))
    bm = mat_key(b.get("material")) or (path.split(".")[1] if path.count(".") >= 2 else None)
    if pm and bm and pm != bm:
        return False, f"桶是 {bm}，件是 {pm}"
    ori = str(b.get("print_orientation") or "").strip()
    if ori and ori != "任意":
        pid = part.get("id") or ""
        if pid and pid not in ori and str(part.get("print_orientation") or "") not in ori:
            return False, f"桶的打印方向『{ori[:24]}』没有涵盖 {pid}（{part.get('print_orientation')}）"
    return True, ""


def bucket_for_feature(buckets, d, part, kind=None):
    """按 **材料 × 打印方向 × 名义直径** 认桶（数据驱动，层里不写死映射表）。

    只有桶的 target_range 与 nominal 是**同一个量**时才用它的区间：
    fits.pla.plain_slide_head_roll 的 nominal_d_mm 是 Ø22.18 而 target 是单边间隙 0.2~0.4，
    量纲对不上（桶里已用 *_quantity 标明），这种退回 feature_check_tolerances.hole_diameter_mm。
    返回 (桶路径, 桶, 区间 or None, 排除理由)。
    """
    skipped = []
    for path, b in buckets.items():
        v = gv(b.get("nominal_d_mm"))
        if v is None:
            v = gv(b.get("nominal_mm"))
        if not isinstance(v, (int, float)) or abs(float(v) - d) > 1e-6:
            continue
        ok, why = bucket_material_ok(path, b, part or {})
        if not ok:
            skipped.append(f"{path}（{why}）")
            continue
        q_nom = b.get("nominal_mm_quantity") or b.get("nominal_d_mm_quantity")
        q_tgt = b.get("target_range_mm_quantity")
        lo, hi = tol_pair(b)
        if q_nom and q_tgt and q_nom != q_tgt:
            return path, b, None, ""
        if isinstance(lo, (int, float)) and isinstance(hi, (int, float)) and lo - 1.0 <= d <= hi + 1.0:
            return path, b, (float(lo), float(hi)), ""
        return path, b, None, ""
    return None, None, None, "；".join(skipped)


def wall_bucket(buckets, part):
    """该件材料自己的壁厚桶（feature_type=wall_thickness 且材料对得上）。

    以前孔周壁厚一律取 fits.pla.min_wall = 2.5，TPU 的 L06 也拿 PLA 的数判 —— 硬编码。
    现在按材料从数据里找；找不到就让调用方判 unknown（元规则 4）。
    """
    for path, b in sorted(buckets.items()):
        if str(b.get("feature_type") or "") != "wall_thickness":
            continue
        ok, _why = bucket_material_ok(path, b, part or {})
        if not ok:
            continue
        lo, _hi = tol_pair(b)
        if isinstance(lo, (int, float)):
            return path, b, float(lo)
    return None, None, None


# ── 件 ↔ placed 名 ─────────────────────────────────────────────────────────
def placed_pair(ctx, part):
    """镜像件的 placed 左右两份。placed 名不是件号（L06_sole_TPU → sole / sole_R），
    用「最长的、有 _R 兄弟的、且是 inventory_id 子串的」stem 去认。"""
    idx = getattr(ctx, "_placed_index", {}) or {}
    ident = f"{part.get('inventory_id') or ''}|{part['id']}"
    best = None
    for stem in idx:
        if stem.endswith("_R") or f"{stem}_R" not in idx:
            continue
        if stem in ident and (best is None or len(stem) > len(best)):
            best = stem
    if best is None:
        return None, None
    return ctx.placed(best), ctx.placed(best + "_R")


def export_part(f):
    """特征在**导出件**上归谁。geom.actual_part_in_export 是数据自己登记的订正
    （L05-F07 那 2 个 Ø4.6 起子通道实际在 L07 上），以它为准。"""
    g = f.get("geom") or {}
    return g.get("actual_part_in_export") or f.get("part")


# ── 层主体 ─────────────────────────────────────────────────────────────────
def run(ctx) -> LayerResult:                                             # noqa: C901
    res = LayerResult(LAYER, NAME)
    T = ctx.data.get("tolerances") or {}
    fct = T.get("feature_check_tolerances") or {}
    buckets = walk_fits(T.get("fits") or {})

    d_lo, d_hi = tol_pair(fct.get("hole_diameter_mm"), key="_")
    dep_lo, dep_hi = tol_pair(fct.get("hole_depth_mm"), key="_")
    _mir_lo, mir_hi = tol_pair(fct.get("mirror_error_mm"), key="_")
    _con_lo, con_hi = tol_pair(fct.get("horn_concentricity_mm"), key="_")
    _clk_lo, clk_hi = tol_pair(fct.get("horn_clock_deg"), key="_")

    if None in (d_lo, d_hi, dep_lo, dep_hi, mir_hi, con_hi, clk_hi):
        res.unknown("_tolerances", "feature_check_tolerances",
                    "tolerances.yaml:feature_check_tolerances 缺 hole_diameter_mm / hole_depth_mm / "
                    "mirror_error_mm / horn_concentricity_mm / horn_clock_deg 之一，本层没有判据可用",
                    provenance="tolerances.yaml:feature_check_tolerances")
        return res
    dtol = max(abs(float(d_lo)), abs(float(d_hi)))

    fdata = ctx.data.get("features") or {}
    feats = fdata.get("features") or []
    frames = fdata.get("frames") or {}
    parts = {p["id"]: p for p in ((ctx.data.get("parts") or {}).get("parts") or [])}
    by_part: dict[str, list] = {}
    for f in feats:
        by_part.setdefault(export_part(f), []).append(f)

    # geom 段的坐标是不是真的写在 export_local 里 —— 这是本层全部定位判据的前提。
    # 元规则 4：frame 缺失（None）也是"不知道这些坐标在哪个系里"，一律判红，不许当合格。
    bad_frame = sorted({f["id"] for f in feats
                        if (f.get("geom") or {}).get("frame") != "export_local"})
    res.add(subject="_features", check="geom_frame",
            state=PASS if not bad_frame else FAIL, severity=BLOCK,
            measured=f"{len(feats) - len(bad_frame)}/{len(feats)} 条 frame=export_local",
            criterion="features.yaml 每条 geom.frame 必须显式写成 export_local（= 导出 STL 自己的"
                      "坐标系，frames.export_local）；缺 frame = 坐标系未知 = 失败（元规则 4），"
                      "不能拿来当射线起点",
            evidence_n=len(feats),
            detail=("" if not bad_frame else f"例外：{bad_frame[:6]}")
                   + f"｜frames.per_part 给了 {len(frames.get('per_part') or {})} 个件的 "
                     f"world_to_export_local，servo_frames {len(frames.get('servo_frames') or {})} 个",
            provenance="features.yaml:frames | features.yaml:features[].geom.frame")

    n_feat_checked = n_rays = 0

    for pid in ctx.parts:
        if ctx.only and pid not in ctx.only:
            continue
        part = parts.get(pid, {})
        mine = by_part.get(pid, [])
        stl = ctx.stl(pid)
        if stl is None:
            res.unknown(pid, "stl_exists",
                        f"cad/duck_s288/ 里找不到 {pid} 的导出 STL，{len(mine)} 条特征一条都验不了",
                        provenance="parts.yaml:parts[].id")
            continue
        res.inputs.append(str(stl.relative_to(ROOT)))
        try:
            g = geom(stl)
        except Exception as e:
            res.unknown(pid, "stl_readable", f"读/分析 STL 失败：{e}")
            continue

        for f in mine:
            fid = f["id"]
            # 元规则 4 的覆盖侧：core.py:COVERAGE_ITEMS 用 res.covered 算特征覆盖率 —— 不登记 = 覆盖不明 = 红。
            # 2026-09-13（F-L2-1）：以前一进循环就 covered.add，`if cyl_u:` / `if box_u and not cyl_u:` 两支都空时
            # 一条判据都没发也算"已覆盖"（12 条 sweep/corridor 类特征把 L2/_coverage 171/171 撑成假绿）。
            # 现在：只有本条特征**真量到东西**（至少一条带 measured 的判据）才登记；只发了 unknown 的不算覆盖。
            n0 = len(res.findings)
            try:
                kind = f.get("check_class") or f.get("kind")
                spec = str(f.get("spec_verbatim") or "")[:44]
                gd = f.get("geom") or {}
                prov = f"features.yaml:{fid}.geom"
                moved = (gd.get("actual_part_in_export") and f.get("part") != pid)
                tag = f"（数据订正：本条登记在 {f.get('part')} 名下，导出件上实际属于 {pid}）" if moved else ""

                if not gd:
                    res.unknown(pid, f"{fid}:geom", "features.yaml 这条没有 geom 段，无坐标可用",
                                provenance=f"features.yaml:{fid}")
                    continue
                if gd.get("confidence") == "unresolved":
                    res.unknown(pid, f"{fid}:geom",
                                f"geom.confidence=unresolved（{gd.get('unknown_class')}）："
                                f"{str(gd.get('unknown_reason'))[:120]}",
                                provenance=prov)
                    continue

                pol = polarity(kind, gd)
                units = probe_units(gd)
                cyl_u = [u for u in units
                         if u.get("_pos") is not None and u.get("_axis") is not None
                         and gv(u.get("nominal_d_mm")) is not None]
                cyl_ids = {id(u) for u in cyl_u}
                box_u = [u for u in units if id(u) not in cyl_ids and u.get("_pos") is not None]
                n_feat_checked += 1

                if pol is None:
                    res.unknown(pid, f"{fid}:polarity",
                                f"check_class='{kind}' 既不在挖除类也不在增料类里，derived_from "
                                f"('{str(gd.get('derived_from'))[:40]}') 也看不出加减 —— "
                                f"不知道声明区域里该有料还是该没料，判不了",
                                provenance=prov)
                    continue

                # ── A′. hr41 落盘 2026-09-25（主设计 Lane C）：六角螺母穴 —— 只有 kind=cavity + shape=prism_array + 写了 geom.hex_prism 才走；
                #    其它形状 / 没写 hex_prism 的一律走下面原来的圆孔探针（逐字节不变）
                if kind == "cavity" and gd.get("shape") == "prism_array" and gd.get("hex_prism") is not None:
                    n_rays += _hex_cavity(res, g, pid, fid, gd, units, gd.get("hex_prism"), dtol, dep_lo, dep_hi, spec, tag, prov)
                    continue

                # ── A. 有名义直径 + 位置 + 轴：沿轴扫描找整圈壁 ──────────────
                if cyl_u:
                    d_nom = float(gv(cyl_u[0].get("nominal_d_mm")))
                    scans = []
                    for u in cyl_u:
                        dn = gv(u.get("nominal_d_mm")) or d_nom
                        sc = ring_scan(g, u["_pos"], u["_axis"], float(dn), pol, u.get("_span_t"), dtol)
                        n_rays += sc["n_rays"]
                        sc["_u"], sc["_d"] = u, float(dn)
                        scans.append(sc)
                    csrcs = []
                    half = bool(cyl_u[0].get("half_only"))
                    need = _SCAN_DIRS // 2 if half else _SCAN_DIRS
                    # hr13（2026-09-18）：geom.min_dirs —— 带削平/缺口的环（N08 盖盘两侧削平 x≤41.0 / x≥11.6）显式声明至少几向能量到整圈，
                    # 不拿 half_only（12/24）冒充；声明值必须 > 12 且 ≤ 24，否则忽略。
                    if cyl_u[0].get("min_dirs") is not None:
                        try:
                            md = int(gv(cyl_u[0].get("min_dirs")) if isinstance(cyl_u[0].get("min_dirs"), dict) else cyl_u[0].get("min_dirs"))
                            if _SCAN_DIRS // 2 < md <= _SCAN_DIRS:
                                need = md
                        except (TypeError, ValueError):
                            pass
                    ok = [s for s in scans if s["matches"] >= need]
                    ev = sum(s["n_rays"] for s in scans)
                    worst = min(scans, key=lambda s: s["matches"])
                    res.add(subject=pid, check=f"{fid}:present",
                            state=PASS if len(ok) == len(scans) and ev else FAIL,
                            severity=BLOCK if pol == "void" else WARN,
                            measured=f"{len(ok)}/{len(scans)} 处到位",
                            criterion=f"geom 声明的 {len(scans)} 个位置，每处沿轴 {_SCAN_STATIONS} 站 × "
                                      f"{_SCAN_DIRS} 向射线都要量到 Ø{d_nom} 的整圈"
                                      + (f"（half_only：只要求 {need}/{_SCAN_DIRS} 向）" if half else (f"（min_dirs：削平/缺口环只要求 {need}/{_SCAN_DIRS} 向）" if need != _SCAN_DIRS else ""))
                                      + f"，容差 ±{dtol}（feature_check_tolerances.hole_diameter_mm）",
                            evidence_n=ev,
                            detail=f"最差一处 {worst['matches']}/{_SCAN_DIRS} 向命中，实测半径 "
                                   f"{[None if r is None else round(r, 4) for r in worst['radii'][:8]]}"
                                   f"（窗内三角面 {worst['n_faces']} 个，声明轴向区间"
                                   f"{'与件包围盒有交集' if worst['in_part'] else '完全落在件外'}）"
                                   f"｜spec：{spec}{tag}"
                                   + ("｜凸特征可能被周围料吞掉而不留自由圆柱面，记 WARN"
                                      if pol == "solid" else ""),
                            provenance=prov)

                    have = [s for s in scans if s["matches"] >= need]
                    if have:
                        judged, crits, skips = [], [], []
                        for sc in have:
                            bpath, _b, brange, bskip = bucket_for_feature(buckets, sc["_d"], part, kind)
                            if bskip and bskip not in skips:
                                skips.append(bskip)
                            if brange:
                                lo, hi = brange
                                c = f"Ø{sc['_d']} ∈ [{lo}, {hi}]（{bpath}.target_range_mm）"
                                cs = f"tolerances.yaml:{bpath}"
                            else:
                                lo, hi = sc["_d"] + float(d_lo), sc["_d"] + float(d_hi)
                                c = (f"Ø{sc['_d']} ∈ [{round(lo, 4)}, {round(hi, 4)}]"
                                     f"（geom.nominal_d_mm ± feature_check_tolerances.hole_diameter_mm）")
                                cs = "tolerances.yaml:feature_check_tolerances.hole_diameter_mm"
                                if bpath:
                                    c += f"；{bpath} 的 target_range 与 nominal 量纲不同，不能当孔径判据"
                            # F-L2-2：直径用棱边法（刻面免疫），不用射线内接径 2·r_med
                            dm, n_edge = facet_free_diameter(g, sc["_u"]["_pos"], sc["_u"]["_axis"], sc["_d"], sc, need)
                            if dm is None:
                                dm, n_edge = 2 * sc["r_med"], 0
                            judged.append((sc, dm, (lo - _NUM_EPS) <= dm <= (hi + _NUM_EPS), n_edge))
                            if c not in crits:
                                crits.append(c)
                                if cs not in csrcs:
                                    csrcs.append(cs)
                        wd, dmw, _okw, _ne = max(judged, key=lambda x: abs(x[1] - x[0]["_d"]))
                        nbad = sum(1 for _s, _d, ok, _n in judged if not ok)
                        n_fallback = sum(1 for _s, _d, _ok, n in judged if n == 0)
                        res.add(subject=pid, check=f"{fid}:diameter",
                                state=PASS if nbad == 0 else FAIL, severity=BLOCK,
                                measured=round(dmw, 4),
                                criterion=("棱边法（刻面免疫：取两端点离轴等距的三角面棱边 = 圆柱刻面的纵向棱/端面弦，"
                                           "端点精确落在建模圆上，对分段数免疫；不用射线内接径）：" + "；".join(crits))[:300],
                                evidence_n=sum(s["n_rays"] for s in have) + sum(n for _s, _d, _ok, n in judged),
                                detail=f"{len(have)}/{len(scans)} 处量到半径，{nbad} 处超差；"
                                       f"最偏一处 Ø{dmw:.4f}（声明 Ø{wd['_d']}，"
                                       f"src={(wd['_u'].get('nominal_d_mm') or {}).get('src')}，"
                                       f"棱边 {_ne} 条，射线内接径 {2 * wd['r_med']:.4f} 只作旁证）"
                                       f"@ 轴向 {wd['t_best']:.3f}"
                                       + (f"｜{n_fallback} 处取不到等距棱边，退回射线内接径（偏小，可能假红）" if n_fallback else "")
                                       + (f"｜排除的同径桶：{'；'.join(skips)[:120]}" if skips else ""),
                                provenance=" | ".join(csrcs) + " | " + prov)

                    # ── 整段功能通规（FG07）：判**最坏截面**，不是最好的那一站 ────
                    if pol == "void" and kind in GAUGE_CLASSES:
                        if half:
                            res.add(subject=pid, check=f"{fid}:bore_gauge", state=PASS, severity=INFO,
                                    measured="n/a（half_only）",
                                    criterion="半开的座（geom.half_only）没有闭合的孔身，通规不适用；"
                                              "它的配合由 relations 的滑配间隙判",
                                    evidence_n=1, provenance=prov)
                        else:
                            gz = [(s, bore_gauge(g, s["_u"]["_pos"], s["_u"]["_axis"], s["_d"], s, need))
                                  for s in scans]
                            live = [(s, z) for s, z in gz if z]
                            if not live:
                                res.unknown(pid, f"{fid}:bore_gauge",
                                            f"{len(scans)} 处都量不出孔身范围（整圈命中的站不足 2 站），"
                                            f"通规判不了 —— 孔到底通不通见 {fid}:present",
                                            provenance=prov)
                            else:
                                rows = []
                                for sc_, z in live:
                                    d_lim = sc_["_d"] + float(d_lo)
                                    dv = z["d_min_vertex"]
                                    rows.append((sc_, z, d_lim,
                                                 (dv is not None and dv < d_lim - 1e-9) or z["n_bad"] > 0))
                                bad = [r for r in rows if r[3]]
                                blind = [r for r in rows if r[1]["d_min_vertex"] is None]
                                w = min(rows, key=lambda r: (r[1]["d_min_vertex"]
                                                             if r[1]["d_min_vertex"] is not None else 1e9))
                                if blind and not bad:
                                    # 量不出全段最小内径就不能说"通规过了"（元规则 2：没有量就不是通过）
                                    res.unknown(pid, f"{fid}:bore_gauge",
                                                f"{len(blind)}/{len(rows)} 处的孔身只有 "
                                                f"{blind[0][1]['n_station']} 站、或窗内取不到三角面棱边，"
                                                f"量不出全段最小内径；只知道这些处 "
                                                f"{sum(r[1]['n_bad'] for r in rows)} 站没量到整圈",
                                                provenance=prov)
                                else:
                                    res.add(subject=pid, check=f"{fid}:bore_gauge",
                                            state=PASS if not bad else FAIL, severity=BLOCK,
                                            measured=(round(w[1]["d_min_vertex"], 4)
                                                      if w[1]["d_min_vertex"] is not None else None),
                                            criterion=f"整段通规：孔身全长上最小内径 ≥ Ø{w[0]['_d']} + "
                                                      f"{d_lo}（feature_check_tolerances."
                                                      f"hole_diameter_mm.min），且孔身两端之间**每一站**"
                                                      f"都要量到整圈 —— 判最坏截面，不是最好截面",
                                            evidence_n=sum(z["n_rays"] + 1 for _s, z in live),
                                            detail=f"{len(bad)}/{len(rows)} 处不过规；最紧一处最小内径 "
                                                   f"Ø{w[1]['d_min_vertex']} @ 轴向 {w[1]['t_at_min']}"
                                                   f"（孔身 {round(w[1]['t0'],3)}..{round(w[1]['t1'],3)}，"
                                                   f"{w[1]['n_station']} 站里 {w[1]['n_bad']} 站没整圈"
                                                   f"{('，位置 ' + str(w[1]['bad_t'])) if w[1]['n_bad'] else ''}；"
                                                   f"射线量到的内接径 "
                                                   f"{None if w[1]['d_min_ray'] is None else round(w[1]['d_min_ray'], 4)}"
                                                   f"，该量受分段数影响、只作旁证）｜spec：{spec}",
                                            provenance="tolerances.yaml:feature_check_tolerances."
                                                       "hole_diameter_mm | " + prov)

                    # 通 / 盲：只对挖出来的特征有意义，逐个实例按**它自己**的
                    # through / depth_kind / depth_mm 判，不拿第一条覆盖全条。
                    if pol == "void":
                        rows = []
                        for u in cyl_u:
                            sp = step_probe(g, u["_pos"], u["_axis"],
                                            float(gv(u.get("nominal_d_mm")) or d_nom), u.get("_span_t"))
                            if sp:
                                n_rays += sp["n_rays"]
                            rows.append((u, sp))
                        ev = sum(sp["n_rays"] for _u, sp in rows if sp)
                        live = [(u, sp) for u, sp in rows if sp and sp["n_dirs"] > 0]
                        dead = [(u, sp) for u, sp in rows if sp and sp["n_dirs"] == 0]
                        if not rows or all(sp is None for _u, sp in rows):
                            res.unknown(pid, f"{fid}:through",
                                        f"声明位置附近取不到三角面，通盲量不出来", provenance=prov)
                        elif dead and not live:
                            res.add(subject=pid, check=f"{fid}:through", state=FAIL, severity=BLOCK,
                                    measured=f"0/{len(rows)} 处有料板",
                                    criterion=f"声明的轴向区间里，名义半径之外（r={d_nom / 2 + _STEP_OFFSET:.2f}）"
                                              f"必须有料 —— 没有料就说明这把刀落在别人已经掏空的地方，"
                                              f"不产生任何孔壁/台阶",
                                    evidence_n=ev,
                                    detail=f"{len(dead)} 处 × {_STEP_DIRS} 向平行轴射线，"
                                           f"声明区间内一向都没碰到料板｜spec：{spec}",
                                    provenance=prov)
                        else:
                            cb = kind in ("counterbore", "spot_face")
                            bad, tot_d, thr_d, decl = [], 0, 0, []
                            for u, sp in live:
                                tot_d += sp["n_dirs"]
                                thr_d += sp["n_through"]
                                t_decl = u.get("through")
                                decl.append(t_decl)
                                if t_decl is None:
                                    continue
                                if t_decl and sp["n_through"] != sp["n_dirs"]:
                                    bad.append(u)
                                if (not t_decl) and (not cb) and sp["n_through"] > 0:
                                    bad.append(u)
                            if all(x is None for x in decl):
                                res.unknown(pid, f"{fid}:through",
                                            f"geom.through 是 null（{str(gd.get('note'))[:60]}），"
                                            f"通盲没有声明值可比", provenance=prov, severity=WARN)
                            else:
                                res.add(subject=pid, check=f"{fid}:through",
                                        state=PASS if not bad else FAIL, severity=BLOCK,
                                        measured=f"{thr_d}/{tot_d} 向真通",
                                        criterion=f"geom.through={sorted({str(x) for x in decl})} → "
                                                  f"{len(live)} 处 × {_STEP_DIRS} 向：声明通的必须每一向"
                                                  f"都不剩料，声明不通的必须每一向都还剩料"
                                                  + ("（沉孔/坐面类只判台阶深，不判通盲）" if cb else ""),
                                        evidence_n=ev,
                                        detail=f"{len(dead)} 处外圈在声明区间内没有料板；掏空量 "
                                               f"{[round(x, 3) for _u, sp in live for x in sp['voids']][:8]}"
                                               f"｜spec：{spec}",
                                        provenance=prov)
                            # 盲深：只有 depth_kind=blind_depth 才拿去比 —— cutter_length 是刀长不是深度
                            blind = [(u, sp) for u, sp in live
                                     if u.get("depth_kind") == "blind_depth"
                                     and gv(u.get("depth_mm")) is not None
                                     and u.get("through") is not True]
                            if blind:
                                dv = float(gv(blind[0][0].get("depth_mm")))
                                lo2, hi2 = dv + float(dep_lo), dv + float(dep_hi)
                                meds = [float(np.median(sp["voids"])) for _u, sp in blind if sp["voids"]]
                                nok = sum(1 for x in meds if lo2 <= x <= hi2)
                                res.add(subject=pid, check=f"{fid}:depth",
                                        state=PASS if meds and nok == len(meds) else FAIL, severity=BLOCK,
                                        measured=(round(max(meds, key=lambda x: abs(x - dv)), 4)
                                                  if meds else f"0/{len(blind)} 处量到台阶"),
                                        criterion=f"盲深 {dv} + [{dep_lo}, {dep_hi}]"
                                                  f"（feature_check_tolerances.hole_depth_mm），"
                                                  f"{len(blind)} 处全部合格",
                                        evidence_n=sum(sp["n_dirs"] for _u, sp in blind),
                                        detail=f"逐处实测掏空量 {[round(x, 4) for x in meds]}",
                                        provenance="tolerances.yaml:feature_check_tolerances."
                                                   "hole_depth_mm | " + prov)
                            else:
                                dk0 = cyl_u[0].get("depth_kind")
                                dv0 = gv(cyl_u[0].get("depth_mm"))
                                if dv0 is not None and dk0 and dk0 != "blind_depth":
                                    res.add(subject=pid, check=f"{fid}:depth_kind", state=PASS,
                                            severity=INFO, measured=f"{dk0}={dv0}",
                                            criterion=f"geom.depth_kind={dk0} 不是盲深（刀长/实体厚度），"
                                                      f"不能拿去和实测台阶深比 —— 该条只判通盲",
                                            evidence_n=1, provenance=prov)

                    # 孔周壁厚 —— 阈值按**件的材料**取，不再一律用 PLA 的 2.5
                    if pol == "void":
                        wpath, wb, wall_lo = wall_bucket(buckets, part)
                        if wall_lo is None:
                            res.unknown(pid, f"{fid}:wall",
                                        f"tolerances.yaml 的 fits 里找不到材料 "
                                        f"'{part.get('material')}' 的 feature_type=wall_thickness 桶 —— "
                                        f"这个件没有可用的壁厚下限（以前一律套 fits.pla.min_wall=2.5，"
                                        f"TPU 件也用 PLA 的数，那是层里写死的映射）",
                                        provenance="tolerances.yaml:fits | parts.yaml:material",
                                        severity=WARN)
                        else:
                            ws = []
                            for s in scans[:8]:
                                u = s["_u"]
                                # 沿孔身取几站各量一圈，取**最薄**的那一站 —— 壁厚是沿孔变化的，
                                # 只在一个站上量会把最薄处漏掉。
                                tl = s.get("ts") or [s["t_best"] or 0.0]
                                picks = sorted({tl[0], tl[len(tl) // 2], tl[-1], s["t_best"] or tl[0]})
                                for tp in picks:
                                    mid = u["_pos"] + u["_axis"] * float(tp)
                                    w, n = hole_wall_min(g.V, g.F,
                                                         dict(axis=u["_axis"], mid=mid, r=s["_d"] / 2.0))
                                    n_rays += _RAY_DIRS
                                    ws.append((w, n))
                            vals = [w for w, _n in ws if w is not None]
                            if not vals:
                                res.unknown(pid, f"{fid}:wall",
                                            f"{_RAY_DIRS} 向径向射线一条也没量到从孔壁到自由面的料段"
                                            f"（孔整段开在空腔里）", provenance=prov, severity=WARN)
                            else:
                                wmin = min(vals)
                                res.add(subject=pid, check=f"{fid}:wall",
                                        state=PASS if wmin >= float(wall_lo) else FAIL, severity=WARN,
                                        measured=round(wmin, 4),
                                        criterion=f"孔周最小壁厚 ≥ {wall_lo}"
                                                  f"（{wpath}.target_range_mm.min，材料 "
                                                  f"{wb.get('material')} / 方向 {wb.get('print_orientation')}）",
                                        evidence_n=sum(n for _w, n in ws),
                                        detail=f"阈值按件的材料 {part.get('material')} 从数据里选桶（src="
                                               f"{(wb.get('nominal_mm') or {}).get('src')} / status="
                                               f"{wb.get('status')}），它本是「结构墙最小厚度」，拿来当"
                                               f"孔周壁厚是目前该材料唯一取得到的阈值 → 记 WARN，实测值即证据",
                                        provenance=f"tolerances.yaml:{wpath} | parts.yaml:material | " + prov)

                # ── C. 两支都空：geom 里既没有可探的柱体（pos+axis+nominal_d）也没有 bbox/pos ──
                #    以前这里静默什么都不发（却已记 covered）。未知 = 失败（元规则 4）。
                if not cyl_u and not box_u:
                    res.unknown(pid, f"{fid}:geometry",
                                f"geom 既无可探针的柱体（pos+axis+nominal_d_mm）也无 bbox/pos"
                                f"（shape={gd.get('shape')}，units={len(units)}），本层没有任何判据能落在它上面；"
                                f"扫掠体请补 box_export_local {{lo,hi}}（export_local 外包盒）",
                                provenance=prov)

                # ── B. 没有直径的：按声明区域判"该处有料/没料" ────────────────
                if box_u and not cyl_u:
                    refs, grids = [], []
                    for u in box_u:
                        Rp, Gp = occupancy_points(u)
                        if len(Rp):
                            refs.append(Rp)
                        if len(Gp):
                            grids.append(Gp)
                    if not refs and not grids:
                        res.unknown(pid, f"{fid}:occupancy",
                                    f"geom 段既没有 nominal_d_mm 也没有 bbox/pos，量不到位置"
                                    f"（shape={gd.get('shape')}）", provenance=prov)
                    else:
                        want_in = (pol == "solid")
                        P = np.vstack(refs) if refs else np.zeros((0, 3))
                        r_ok = sum(1 for b_ in occupancy(g, P) if bool(b_) == want_in) if len(P) else 0
                        n_rays += 3 * len(P)
                        G = np.vstack(grids) if grids else np.zeros((0, 3))
                        if len(G) > _OCC_GRID_CAP:
                            G = G[np.linspace(0, len(G) - 1, _OCC_GRID_CAP).astype(int)]
                        g_ok = sum(1 for b_ in occupancy(g, G) if bool(b_) == want_in) if len(G) else 0
                        n_rays += 3 * len(G)
                        tot, sat = len(P) + len(G), r_ok + g_ok
                        # 判据分两条：① 声明区域里一个符合的点都没有 = 该特征在导出件上根本不存在（空刀）；
                        # ② 挖除类的参考点上必须真的没有料 —— 后续布尔不会专挑刀心把料补回来。
                        # 不判"整片 bbox 全符合"：布尔有先后（先堆板再开窗、先挖仓腔再加导轨），
                        # 那种判据只会造假红。占比写进 measured 当诊断量。
                        bad = (sat == 0) or (not want_in and len(P) and r_ok < len(P))
                        res.add(subject=pid, check=f"{fid}:occupancy",
                                state=FAIL if bad else PASS,
                                severity=BLOCK if len(G) else WARN,
                                measured=f"{sat}/{tot} 点符合（参考点 {r_ok}/{len(P)}）",
                                criterion=f"{'增料' if want_in else '挖除'}类特征（{kind}）："
                                          f"声明区域内至少要有一个采样点"
                                          f"{'有料' if want_in else '没有料'}（一个都没有 = 空刀）"
                                          + ("；且 pos / bbox 中心这些参考点必须没有料" if not want_in else "")
                                          + ("；**这条只判'在不在'**，整块还在不在由 "
                                             f"{fid}:solid_retention 判" if want_in else ""),
                                evidence_n=3 * tot,
                                detail=f"shape={gd.get('shape')}，参考点 {len(P)} 个 + bbox 网格 {len(G)} 个，"
                                       f"每点 3 条奇偶射线"
                                       + ("；该形状只有包络没有解析形状，判据只能落在参考点上，记 WARN"
                                          if not len(G) else "")
                                       + f"｜spec：{spec}{tag}",
                                provenance=prov)

                        # ── 保留区（FG08）：声明为实体的区域，扣掉别的特征声明要挖的地方之后，
                        #    **剩下的每一格都必须有料**。一个点有料 ≠ 整块压板还在。
                        if want_in:
                            void_in, nb, nc = declared_voids(feats, pid)
                            tot_k = keep = 0
                            thin, n_ray_sec, n_skip_sec, sec_axis = [], 0, 0, None
                            decl_th = None
                            unscanned_regions = []
                            for u in box_u:
                                bb = _bbox(next((u.get(k) for k in BBOX_KEYS if u.get(k)), None))
                                if bb is None:
                                    unscanned_regions.append(u.get("shape") or "unknown")
                                    continue
                                sec_axis = ai = int(np.argmin(bb[1] - bb[0]))
                                rs = region_scan(g, bb[0], bb[1], ai, void_in)
                                n_rays += rs["n_rays"]
                                n_ray_sec += rs["n_rays"]
                                n_skip_sec += rs["n_skip"]
                                tot_k += rs["n_tot"]
                                keep += rs["n_keep"]
                                thin += rs["thin"]
                                if decl_th is None:
                                    decl_th = gv(u.get("thickness_mm"))
                            if unscanned_regions:
                                res.unknown(pid, f"{fid}:solid_retention_coverage",
                                            f"{len(box_u)} 个实体探针单元中有 {len(unscanned_regions)} 个"
                                            f"没有可扫描的保留区 bbox（shape={unscanned_regions}）；"
                                            f"参考点有料不能证明该区域完整保留，其他单元的"
                                            f"solid_retention 结果不能代替这部分未知",
                                            provenance=prov, severity=WARN)
                            if tot_k == 0:
                                res.unknown(pid, f"{fid}:solid_retention",
                                            f"声明为实体的区域全部落在别的特征声明的孔槽里"
                                            f"（扣除 {n_skip_sec} 格，box {nb} 个 / cyl {nc} 个），"
                                            f"没有可判的保留区",
                                            provenance=prov, severity=WARN)
                            else:
                                frac = keep / tot_k
                                res.add(subject=pid, check=f"{fid}:solid_retention",
                                        state=PASS if keep == tot_k else FAIL, severity=BLOCK,
                                        measured=round(frac, 4),
                                        criterion=f"有可扫描 bbox 的声明实体区域，扣掉本件其它特征声明要挖掉的孔槽"
                                                  f"（{nb} 个 bbox + {nc} 个圆柱，让开 {_KEEP_MARGIN}）之后，"
                                                  f"**剩下的每一格都必须有料**（保留比例 = 1.0）；"
                                                  f"'至少一个点有料' 不是判据",
                                        evidence_n=tot_k,
                                        detail=f"{keep}/{tot_k} 格有料（扣除 {n_skip_sec} 格已声明的孔槽），"
                                               f"沿最短轴 {'xyz'[sec_axis]} 逐列 {n_ray_sec} 条射线"
                                               f"｜缺料格 = 没有任何特征声明要挖、成品却是空的"
                                               + (f"｜另有 {len(unscanned_regions)} 个单元未扫描，见 "
                                                  f"{fid}:solid_retention_coverage" if unscanned_regions else "")
                                               + f"｜spec：{spec}{tag}",
                                        provenance=prov + " | features.yaml:同件的挖除类特征")
                                if thin:
                                    tmin = float(min(thin))
                                    if decl_th is None:
                                        res.unknown(pid, f"{fid}:min_section",
                                                    f"实测最小有效截面厚度 {round(tmin,4)}（沿 bbox 最短轴 "
                                                    f"{'xyz'[sec_axis]}，{len(thin)} 格），但 geom 没写 "
                                                    f"thickness_mm，没有声明值可比",
                                                    provenance=prov, severity=WARN)
                                    else:
                                        lim = float(decl_th) + float(dep_lo)
                                        res.add(subject=pid, check=f"{fid}:min_section",
                                                state=PASS if tmin >= lim else FAIL, severity=BLOCK,
                                                measured=round(tmin, 4),
                                                criterion=f"最小有效截面厚度 ≥ 声明厚度 {decl_th} + {dep_lo}"
                                                          f"（feature_check_tolerances.hole_depth_mm.min）"
                                                          f"，沿 bbox 最短轴 {'xyz'[sec_axis]} 逐格量",
                                                evidence_n=len(thin),
                                                detail=f"{len(thin)} 格（已跳过 {n_skip_sec} 格声明的孔槽），"
                                                       f"最薄 {round(tmin,4)} / 中位 "
                                                       f"{round(float(np.median(thin)),4)}｜承压件靠的是"
                                                       f"这个厚度，不是'有没有一个点'",
                                                provenance="tolerances.yaml:feature_check_tolerances."
                                                           "hole_depth_mm | " + prov)
            finally:
                if any(f.measured is not None for f in res.findings[n0:]):
                    res.covered.add(fid)

        # ── 舵盘 6 孔阵（件级，坐标系无关的不变量，与 geom 定位互为旁证）──
        horn_feats = [f for f in mine if (f.get("check_class") == "horn_hole")]
        if horn_feats:
            d_horn, want, decl_r = None, 0, None
            for f in horn_feats:
                gd = f.get("geom") or {}
                if d_horn is None:
                    d_horn = gv(gd.get("nominal_d_mm"))
                if decl_r is None:
                    decl_r = gv(gd.get("pitch_r_mm"))
                srcs = [i for i in (gd.get("instances") or []) if i.get("hole_positions_mm")] \
                    or ([gd] if gd.get("hole_positions_mm") else [])
                n_holes = sum(len(u.get("hole_positions_mm") or []) for u in srcs)
                want += max(1, n_holes // 6)
            pats = g.horn_patterns(float(d_horn), dtol) if d_horn else []
            res.add(subject=pid, check="horn:pattern_count",
                    state=PASS if (want and len(pats) >= want) else FAIL, severity=WARN,
                    measured=len(pats),
                    criterion=f"应能找到 {want} 组「6 个 Ø{d_horn} 孔均布在一个圆上」的阵列（S288 舵盘）",
                    evidence_n=sum(len(p["holes"]) for p in pats),
                    detail=f"features.yaml 该件声明 {len(horn_feats)} 条 horn_hole；这条是**坐标系无关的旁证**（不看 geom 坐标，直接在三角面里找 6 孔圆阵），同轴同面的两片法兰会被聚成一簇而少计 → 记 WARN；每个孔到底在不在声明位置由 <特征>:present 判 BLOCK",
                    provenance="features.yaml:horn_hole.geom | s288.py 舵盘 6 孔 60° 分度")
            for i, p in enumerate(pats):
                worst = max(abs(x - 60.0) for x in p["gaps"])
                res.add(subject=pid, check=f"horn{i}:index_60deg",
                        state=PASS if worst <= abs(float(clk_hi)) else FAIL, severity=BLOCK,
                        measured=round(worst, 4),
                        criterion=f"6 孔相邻夹角 60° ± {clk_hi}（feature_check_tolerances.horn_clock_deg）",
                        evidence_n=len(p["gaps"]), detail=f"实测夹角 {[round(x, 3) for x in p['gaps']]}",
                        provenance="tolerances.yaml:feature_check_tolerances.horn_clock_deg")
                res.add(subject=pid, check=f"horn{i}:concentricity",
                        state=PASS if p["radial_res"] <= float(con_hi) else FAIL, severity=BLOCK,
                        measured=round(p["radial_res"], 5),
                        criterion=f"6 孔中心到最佳拟合圆的径向偏差 ≤ {con_hi}"
                                  f"（feature_check_tolerances.horn_concentricity_mm）",
                        evidence_n=6, detail=f"阵列半径实测 r={p['r_pattern']:.4f}",
                        provenance="tolerances.yaml:feature_check_tolerances.horn_concentricity_mm")
                if decl_r is not None:
                    res.add(subject=pid, check=f"horn{i}:pitch_radius",
                            state=PASS if abs(p["r_pattern"] - float(decl_r)) <= float(con_hi) else FAIL,
                            severity=BLOCK, measured=round(p["r_pattern"], 5),
                            criterion=f"孔阵分度圆半径 = geom.pitch_r_mm {decl_r} ± {con_hi}"
                                      f"（feature_check_tolerances.horn_concentricity_mm）",
                            evidence_n=6, detail="半径错了整片法兰都拧不上",
                            provenance="features.yaml:horn_hole.geom.pitch_r_mm | "
                                       "tolerances.yaml:feature_check_tolerances.horn_concentricity_mm")
                else:
                    res.unknown(pid, f"horn{i}:pitch_radius",
                                "该件的 horn_hole 条目 geom 里没有 pitch_r_mm，孔阵半径没有声明值可比",
                                provenance="features.yaml:horn_hole.geom", severity=WARN)

        # ── 镜像件 ──────────────────────────────────────────────────────
        if part.get("mirrored_copy"):
            pl, pr = placed_pair(ctx, part)
            if pl is None or pr is None:
                res.unknown(pid, "mirror",
                            f"parts.yaml 声明 mirrored_copy='{str(part.get('mirrored_copy'))[:24]}'，"
                            f"但 cad/duck_s288/placed/ 里找不到成对的左右件，镜像误差无从量",
                            provenance="parts.yaml:mirrored_copy")
            else:
                try:
                    gl, gr = geom(pl), geom(pr)
                    res.inputs += [str(pl.relative_to(ROOT)), str(pr.relative_to(ROOT))]
                    M = np.array([1.0, -1.0, 1.0])            # parts.yaml 声明 mirror_y
                    lo_l, hi_l = gl.bbox
                    lo_r, hi_r = gr.bbox
                    mlo = np.minimum(lo_r * M, hi_r * M)
                    mhi = np.maximum(lo_r * M, hi_r * M)
                    bb = float(max(np.abs(lo_l - mlo).max(), np.abs(hi_l - mhi).max()))
                    res.add(subject=pid, check="mirror:bbox",
                            state=PASS if bb <= float(mir_hi) else FAIL, severity=BLOCK,
                            measured=round(bb, 5),
                            criterion=f"左件与右件镜像后包围盒逐轴误差 ≤ {mir_hi}"
                                      f"（feature_check_tolerances.mirror_error_mm）",
                            evidence_n=6, detail=f"{pl.name} vs mirror_y({pr.name})",
                            provenance="tolerances.yaml:feature_check_tolerances.mirror_error_mm")
                    # ① 圆柱特征：**双向**一一对应 + 轴向 + 轴向跨度（FG09）
                    errs, un_l, un_r, ang_max, span_max = cyl_pairs(gl, gr, M)
                    if not errs:
                        res.unknown(pid, "mirror:features",
                                    f"左件 {len(gl.cyls)} 个圆柱面镜像后在右件里一个都配不上，给不出误差",
                                    provenance="parts.yaml:mirrored_copy")
                    else:
                        w = max(errs)
                        ok_all = (w <= float(mir_hi) and un_l == 0 and un_r == 0
                                  and span_max <= float(mir_hi))
                        res.add(subject=pid, check="mirror:features",
                                state=PASS if ok_all else FAIL,
                                severity=BLOCK, measured=round(w, 5),
                                criterion=f"每个圆柱特征镜像后位置误差 ≤ {mir_hi}，**双向**一一对上"
                                          f"（左→右 与 右→左 都不许有落单的），轴向平行、"
                                          f"轴向跨度差 ≤ {mir_hi}"
                                          f"（feature_check_tolerances.mirror_error_mm）",
                                evidence_n=len(errs) + len(gl.cyls) + len(gr.cyls),
                                detail=f"配上 {len(errs)} 对；左件落单 {un_l} 个、右件落单 {un_r} 个"
                                       f"（左 {len(gl.cyls)} / 右 {len(gr.cyls)} 个圆柱面）；"
                                       f"最大轴向夹角 {round(ang_max,4)}°、最大轴向跨度差 "
                                       f"{round(span_max,4)}",
                                provenance="tolerances.yaml:feature_check_tolerances.mirror_error_mm")

                    # ② 非圆柱表面：槽、筋、台、平面 —— 只有这条能看见（FG09 的真正缺口）
                    try:
                        allow = part.get("mirror_asymmetry_allowed") or []
                        dv = mirror_surface_dev(ctx, pl, pr, M, allow=allow)
                        wsurf = max(dv["l2r"], dv["r2l"])
                        res.add(subject=pid, check="mirror:surface",
                                state=PASS if wsurf <= float(mir_hi) else FAIL, severity=BLOCK,
                                measured=round(wsurf, 5),
                                criterion=f"左件与镜像后的右件，**双向**逐点到对面表面的距离最大值 "
                                          f"≤ {mir_hi}（feature_check_tolerances.mirror_error_mm）；"
                                          f"包围盒和圆孔中心相同、只挪了槽/筋的右件必须在这条上红。"
                                          f"允许不对称的地方必须在 parts.yaml:parts[]."
                                          f"mirror_asymmetry_allowed 里显式声明（本件声明了 "
                                          f"{len(allow)} 处）",
                                evidence_n=dv["l2r_n"] + dv["r2l_n"],
                                detail=f"左→右 {round(dv['l2r'],5)}（{dv['l2r_n']} 点）/ "
                                       f"右→左 {round(dv['r2l'],5)}（{dv['r2l_n']} 点）；"
                                       f"面积加权采样、固定种子，与建模分段无关（元规则 3）"
                                       + (f"；按声明排除了 {dv['n_excluded']} 个采样点"
                                          if dv["n_allowed_boxes"] else ""),
                                provenance="tolerances.yaml:feature_check_tolerances.mirror_error_mm | "
                                           "parts.yaml:mirrored_copy")
                    except Exception as e:
                        res.unknown(pid, "mirror:surface",
                                    f"非圆柱表面镜像比对算不出来：{e}",
                                    provenance="parts.yaml:mirrored_copy")
                except Exception as e:
                    res.unknown(pid, "mirror", f"镜像比对失败：{e}")

    orphan = sorted({export_part(f) for f in feats} - set(ctx.parts))
    if orphan:
        res.unknown("_features", "part_ref",
                    f"features.yaml 里这些 part 在 parts.yaml 里没有：{orphan}",
                    provenance="features.yaml:features[].part")

    res.evidence = {"features_declared": len(feats),
                    "features_with_geometry_check": n_feat_checked,
                    "rays_cast": n_rays,
                    "parts_checked": len([p for p in ctx.parts if not ctx.only or p in ctx.only])}
    return res
