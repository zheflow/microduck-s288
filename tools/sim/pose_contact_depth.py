#!/usr/bin/env python3
"""两件摆好后的穿插深度（hr50，2026-09-28）：给 L6 真实姿态碰撞的"轻碰"判据用（原版 p25：≤2.29 mm³ 且 ≤0.77 mm）。

算法与 docs/design_2026-09-17_bearing_rebuild/hr50_work/algo/C_work/c2_depth.py 相同（C 用它标定原版 62 对真接触的峰值深度，
自检：两立方体面叠 0.30 → 0.300、45° 棱扎进 0.50 → 0.500）：
  I = A∩B（manifold 布尔）。I 的表面 ⊂ ∂A ∪ ∂B：落在 ∂B 上的点（B 的表面、在 A 里）到 ∂A 的距离 = B 扎进 A 的深度；
  落在 ∂A 上的点到 ∂B 的距离 = A 扎进 B 的深度；两者取大。I 表面取全部顶点 + 按面积均匀采样（间距 ≈ SPACING），
  每点到 ∂A、∂B 的精确点-三角形距离（trimesh.proximity.closest_point）；dA ≤ dB 的点算在 ∂A 上。
  距离只对 I 包围盒外扩 CROP 内的三角形算；结果 ≤ CROP 时与全网格精确相等，> CROP 记下界并标 crop_limited。
  薄壁件被整道穿透时深度会卡在半壁厚附近（这时要和体积一起看）。
本模块只用 trimesh + 调用方给的 manifold 实体，不 import embree。
"""
from __future__ import annotations
import numpy as np
import trimesh

SPACING = 0.12          # mm，I 表面随机采样的目标间距
N_MIN, N_MAX = 3000, 40000
CROP = 15.0             # mm
CHUNK = 800             # closest_point 分块，控内存


def _crop_mesh(tris, lo, hi):
    tlo, thi = tris.min(axis=1), tris.max(axis=1)
    keep = np.all(thi >= lo, axis=1) & np.all(tlo <= hi, axis=1)
    sub = tris[keep]
    if len(sub) == 0:
        return None
    return trimesh.Trimesh(vertices=sub.reshape(-1, 3), faces=np.arange(len(sub) * 3).reshape(-1, 3), process=False)


def _dist(tris, pts):
    lo, hi = pts.min(axis=0) - CROP, pts.max(axis=0) + CROP
    sub = _crop_mesh(tris, lo, hi)
    if sub is None:
        return np.full(len(pts), CROP), True
    d = np.empty(len(pts))
    for s in range(0, len(pts), CHUNK):
        _, d[s:s + CHUNK], _ = trimesh.proximity.closest_point(sub, pts[s:s + CHUNK])
    return d, bool((d > CROP).any())


def contact_depth(tris_a, solid_a, M_a, tris_b, solid_b, M_b, seed=20260928):
    """tris_* = 零位网格三角形 (n,3,3)；solid_* = 同一网格的 manifold 实体；M_* = 4×4 世界变换（相对零位）。
    返回 dict(vol_mm3, depth_mm, depth_A_in_B_mm, depth_B_in_A_mm, deepest_point_world, n_samples, crop_limited)。"""
    Ia = solid_a.transform(np.asarray(M_a)[:3]); Ib = solid_b.transform(np.asarray(M_b)[:3])
    it = Ia ^ Ib
    vol = float(it.volume())
    out = dict(vol_mm3=vol)
    if vol <= 1e-9:
        out.update(depth_mm=0.0, depth_A_in_B_mm=0.0, depth_B_in_A_mm=0.0, n_samples=0, crop_limited=False)
        return out
    raw = it.to_mesh64()
    V = np.asarray(raw.vert_properties, dtype=np.float64)[:, :3]
    F = np.asarray(raw.tri_verts, dtype=np.int64)
    It = trimesh.Trimesh(vertices=V, faces=F, process=False)
    n = int(np.clip(float(It.area) / SPACING ** 2, N_MIN, N_MAX))
    samp, _ = trimesh.sample.sample_surface(It, n, seed=int(seed))
    P = np.vstack([V, samp])
    ia, ib = np.linalg.inv(M_a), np.linalg.inv(M_b)
    Pa = P @ ia[:3, :3].T + ia[:3, 3]
    Pb = P @ ib[:3, :3].T + ib[:3, 3]
    dA, ca = _dist(np.asarray(tris_a), Pa)
    dB, cb = _dist(np.asarray(tris_b), Pb)
    onA = dA <= dB
    a_in_b = float(dB[onA].max()) if onA.any() else 0.0
    b_in_a = float(dA[~onA].max()) if (~onA).any() else 0.0
    k = int(np.argmax(np.maximum(np.where(onA, dB, 0.0), np.where(~onA, dA, 0.0))))
    out.update(depth_mm=max(a_in_b, b_in_a), depth_A_in_B_mm=a_in_b, depth_B_in_A_mm=b_in_a,
               deepest_point_world=[round(float(x), 3) for x in P[k]], n_samples=int(len(P)), crop_limited=bool(ca or cb))
    return out
