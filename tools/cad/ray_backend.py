"""hr42（2026-09-24，B 线）：射线查询统一入口 —— 与 trimesh 纯 Python 射线（trimesh.ray.ray_triangle，基线用的那个）**同形同序同值**。

    import ray_backend as RB
    loc, idx_ray, idx_tri = RB.intersects_location(mesh, origins, directions, multiple_hits=True)
    idx_tri, idx_ray      = RB.intersects_id(mesh, origins, directions, multiple_hits=True)       # = mesh.ray.intersects_id(...)
    hit                   = RB.intersects_any(mesh, origins, directions)
    inside                = RB.contains(mesh, points)                                         # = mesh.contains(points)
    RB.name()             # "embree-4.4.0+exact" / "triangle"

后端（环境变量 DUCK_RAY_BACKEND，默认 auto）：
  auto（默认）
      contains → embree+exact；intersects_location / intersects_id / intersects_any → bulk（见下）。
      理由（B_tools/rb_test.py 实测，真网格）：embree+exact 在 contains（trimesh 默认的一般方向射线）上与 triangle 逐点相同；
      但 CAD 件上"整数坐标 + 轴向"的射线常常**正好躺在面内 / 擦着棱**，Embree 在 float32 下直接漏掉这类擦边命中
      （head_bracket 0.5 mm 网格 +z 射线：566 个命中漏 4 个），邻域补查也补不回（没有相邻面被命中）→ 射线类一律走 bulk（按构造逐字节同 triangle）。
  embree
      Embree 只负责**找候选三角面**（多次命中迭代 + 每个命中点邻域盒内的全部三角面 + 起点后撤 1e-5 mm 找"起点身后 1e-6 内"的面），
      命中与否、命中位置、去重、输出顺序全部用 trimesh ray_triangle 的**原公式逐对照算**：
        ① rtree 候选资格（三角面 AABB ∩ trimesh.ray_bounds 给的射线盒，闭区间，与 libspatialindex 同判）
        ② intersections.planes_lines（原始起点 + 原始方向，不单位化 —— 与 ray_triangle_id 一致）
        ③ points_to_barycentric 的 [-tol.zero, 1+tol.zero] 判内、distance > -1e-6 判前向
        ④ multiple_hits=False：每条射线 argmin 距离；return_locations：grouping.unique_rows 去重排序；
           并列 / 重复时"谁先"按该射线在 rtree 里的候选顺序定（只对有并列的射线查 rtree，intersection_v 批量）
      Embree 没漏掉真命中面时结果与 triangle 逐字节相同；擦边射线会漏（见 auto 的理由），所以只给 contains 用。
  bulk
      trimesh ray_triangle 同一算法，只把逐条射线的 rtree 查询换成 intersection_v 批量查询（候选集合与顺序与逐条查询相同）
      + 上面同一套逐对原公式 → 结果按构造逐字节同 triangle（rb_test 全部用例逐字节相同）。省的是逐条 Python 开销，所以调用点要**批量**传射线。
  triangle
      原样调 mesh.ray.* / mesh.contains（trimesh 纯 Python，基线）。
空结果的形状也照 trimesh：射线盒里一个候选面都没有（或全部与射线平行）时 locations 是 (0,)，否则 (0, 3)。

DUCK_RAY_VERIFY=<json 路径>：每次调用两个后端（当前后端 + triangle）都算，逐条比对（命中数、index_ray、index_tri、位置差、contains 逐点），
  汇总追加写进该文件（原子改名；多进程各写 <路径>.<pid>.json）；**返回的仍是当前后端的结果**（对账跑的就是生产路径）。
  调用方可用 set_site("件名/调用点") 标注，写进对账行。

注意：本模块 import 时先 import trimesh（此时 hr42_embree 不在 sys.path → trimesh 全局仍用纯 Python 射线，mesh.ray / mesh.contains 行为不变），
      之后才把 hr42_embree 追加到 sys.path 末尾、显式 import trimesh.ray.ray_pyembree。其它代码的射线行为一律不受影响。
"""
from __future__ import annotations

import json
import os
import sys
import time

import numpy as np
import trimesh                                   # 必须先于 embree 路径 import（见模块说明）
import trimesh.ray.ray_triangle as _RT
from trimesh import grouping as _grp
from trimesh import intersections as _isect
from trimesh import triangles as _tri
from trimesh import util as _util
from trimesh.constants import tol as _tol

_ROOT = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", ".."))
_EMB_DIR = os.path.join(_ROOT, ".venv", "hr42_embree")
_S = {"mod": None, "tried": False, "ver": None, "err": None, "site": "", "verify": [], "vpath": None}
_BACK_EPS = 1e-5            # Embree 起点后撤（mm）：找 trimesh 判"前向"的 distance ∈ (-1e-6, 0] 的面
_MAX_HITS = 1000


# ── 后端选择 ────────────────────────────────────────────────────────────────
# 第五任（主 agent 2026-09-25 00:3x）：name() / backend() 不再 import Embree 模块。
#   原来 name() → backend("contains") → _embree_mod() 会 import trimesh.ray.ray_pyembree（带进 embreex 的 libembree4 + libtbb.12），
#   而 check_cache.versions() 调 name() → 每个 gate 层进程都载入两份 TBB（manifold3d 内嵌 oneTBB + embreex 的 libtbb.12），哪怕这一层一条 Embree 射线都不打；
#   (e) 第一次 Gate 崩的进程里正好两份都在（dev/e_gate_crash/）。现在：Embree 是否"可用"只看 hr42_embree 目录在不在 + importlib.metadata 读版本
#   （不 import 模块），name() 串与原来逐字节相同（缓存键不变）；Embree 模块只在第一次真用它找候选（_fast_id 里 b == "embree"）时才 import，
#   import 失败 → 退到同算法的批量版（结果与 triangle 相同）并在 stderr 报一次。
def _embree_probe():
    """不 import 模块地判断 Embree 可不可用：目录在 → 把目录加进 sys.path（与原来相同）+ 读 embreex 版本（读不到记 "?"，与原来相同）。"""
    if not _S.get("probed"):
        _S["probed"] = True
        if os.path.isdir(_EMB_DIR):
            if _EMB_DIR not in sys.path:
                sys.path.append(_EMB_DIR)
            try:
                import importlib.metadata as md
                _S["ver"] = md.version("embreex")
            except Exception:                                        # noqa: BLE001
                _S["ver"] = "?"
        else:
            _S["err"] = f"{_EMB_DIR} 不存在"
    return _S["err"] is None


def _embree_mod():
    """真要用 Embree 时才调：第一次 import trimesh.ray.ray_pyembree；失败返回 None（并记 _S["err"]，之后 backend() 都退 bulk）。"""
    if not _S["tried"]:
        _S["tried"] = True
        if _embree_probe():
            try:
                import importlib
                _S["mod"] = importlib.import_module("trimesh.ray.ray_pyembree")
            except Exception as e:                                   # noqa: BLE001
                _S["mod"] = None
                _S["err"] = repr(e)
                print(f"[ray_backend] Embree 模块 import 失败，contains 退到同算法批量版（结果不变）：{e!r}", file=sys.stderr, flush=True)
    return _S["mod"]


def backend(op: str = "rays") -> str:
    """op = "rays"（intersects_*）或 "contains"。返回要用的后端：embree / bulk / triangle（embree 只看可用性，不 import 模块）。"""
    b = os.environ.get("DUCK_RAY_BACKEND", "auto").strip().lower() or "auto"
    if b not in ("auto", "embree", "triangle", "bulk"):
        raise ValueError(f"DUCK_RAY_BACKEND={b!r} 不认识（auto / embree / bulk / triangle）")
    if b == "auto":
        b = "embree" if op == "contains" else "bulk"
    if b == "embree" and not _embree_probe():
        return "bulk"                                                # embree 不可用 → 退到同算法的批量版（结果与 triangle 相同）
    return b


def name() -> str:
    """本进程射线后端的说明串（写进日志 / evidence）。"""
    def one(op):
        b = backend(op)
        return f"embree-{_S['ver']}+exact" if b == "embree" else ("triangle-bulk" if b == "bulk" else "triangle")
    s = f"rays={one('rays')};contains={one('contains')}"
    if _S["err"] and os.environ.get("DUCK_RAY_BACKEND", "auto") in ("auto", "embree"):
        s += f"（embree 不可用：{_S['err']}）"
    return s


def set_site(site: str):
    """对账行里写"谁在调"（件名 / 调用点），只影响 DUCK_RAY_VERIFY 的记录，不影响结果。"""
    _S["site"] = str(site)


# ── trimesh ray_triangle 的逐对原公式 ───────────────────────────────────────
def _as_rays(origins, directions):
    o = np.asanyarray(origins, dtype=np.float64)
    d = np.asanyarray(directions, dtype=np.float64)
    if o.ndim == 1:
        o = o.reshape((1, -1))
    if d.ndim == 1:
        d = d.reshape((1, -1))
    return o, d


def _tree(mesh):
    return mesh.triangles_tree


def _ray_boxes(mesh, o, d):
    """与 ray_triangle.ray_triangle_candidates 同一个射线盒（ray_bounds 逐行独立 → 子集算与整批算逐字节相同）。"""
    return _RT.ray_bounds(ray_origins=o, ray_directions=d, bounds=_tree(mesh).bounds)


def _tri_bounds(mesh):
    c = mesh._cache
    tb = c["hr42_tri_bounds"]
    if tb is None:
        t = np.asanyarray(mesh.triangles, dtype=np.float64)
        tb = (t.min(axis=1), t.max(axis=1))
        c["hr42_tri_bounds"] = tb
    return tb


def _exact(mesh, o, d, cand_ray, cand_tri, boxes):
    """对候选对 (ray, tri) 逐对照做 ray_triangle_id（multiple_hits=True 那条路）的全部判定。返回按输入顺序保留的
    (index_tri, index_ray, location, distance)。boxes = 每条射线的 rtree 查询盒（(m, 6)）。"""
    cand_ray = np.asarray(cand_ray, dtype=np.int64)
    cand_tri = np.asarray(cand_tri, dtype=np.int64)
    # trimesh 的早退：一个候选都没有 / 全部与射线平行 → locations 是 (0,) 的 float 数组（不是 (0, 3)）
    empty = (np.array([], dtype=np.int64), np.array([], dtype=np.int64), np.array([], dtype=np.float64), np.array([], dtype=np.float64))
    if len(cand_ray) == 0:
        return empty
    # ① rtree 候选资格（libspatialindex Region::intersectsRegion：闭区间）
    tmin, tmax = _tri_bounds(mesh)
    bx = boxes[cand_ray]
    ok = np.all(tmin[cand_tri] <= bx[:, 3:], axis=1) & np.all(tmax[cand_tri] >= bx[:, :3], axis=1)
    cand_ray, cand_tri = cand_ray[ok], cand_tri[ok]
    if len(cand_ray) == 0:
        return empty
    triangles = mesh.triangles
    triangle_candidates = triangles[cand_tri]
    line_origins = o[cand_ray]
    line_directions = d[cand_ray]
    plane_origins = triangle_candidates[:, 0, :]
    plane_normals = mesh.face_normals[cand_tri]
    location, valid = _isect.planes_lines(plane_origins=plane_origins, plane_normals=plane_normals,
                                          line_origins=line_origins, line_directions=line_directions)
    if len(triangle_candidates) == 0 or not valid.any():
        return empty
    barycentric = _tri.points_to_barycentric(triangle_candidates[valid], location)
    hit = np.logical_and((barycentric > -_tol.zero).all(axis=1), (barycentric < (1 + _tol.zero)).all(axis=1))
    index_tri = cand_tri[valid][hit]
    index_ray = cand_ray[valid][hit]
    location = location[hit]
    vector = location - o[index_ray]
    distance = _util.diagonal_dot(vector, d[index_ray])
    forward = distance > -1e-6
    return index_tri[forward], index_ray[forward], location[forward], distance[forward]


def _rtree_rank(mesh, o, d, rays, boxes):
    """这些射线各自在 rtree 里的候选顺序：{ray: {tri: 名次}}（intersection_v 批量；与逐条 tree.intersection 同序，已在真网格上核过）。"""
    rays = np.unique(np.asarray(rays, dtype=np.int64))
    if len(rays) == 0:
        return {}
    b = boxes[rays]
    ids, counts = _tree(mesh).intersection_v(np.ascontiguousarray(b[:, :3]), np.ascontiguousarray(b[:, 3:]))
    out, k = {}, 0
    for r, n in zip(rays.tolist(), counts.astype(np.int64).tolist()):
        seg = ids[k:k + n]
        k += n
        out[r] = {int(t): i for i, t in enumerate(seg.tolist())}
    return out


def _order_within_ray(mesh, o, d, index_tri, index_ray, boxes):
    """把命中行排成 trimesh 的候选顺序：射线号升序，同一射线内按 rtree 候选名次。"""
    if len(index_ray) == 0:
        return np.arange(0)
    multi = np.flatnonzero(np.bincount(index_ray) > 1)
    rank = _rtree_rank(mesh, o, d, multi, boxes) if len(multi) else {}
    key2 = np.array([rank[r][t] if r in rank else 0 for r, t in zip(index_ray.tolist(), index_tri.tolist())], dtype=np.int64)
    return np.lexsort((key2, index_ray))


def _finish(mesh, o, d, index_tri, index_ray, location, distance, boxes, multiple_hits, return_locations, need_order=True):
    """ray_triangle_id 之后的收尾（first-hit 选取）+ RayMeshIntersector.intersects_id 的 unique_rows —— 与 trimesh 同序。"""
    if need_order:
        order = _order_within_ray(mesh, o, d, index_tri, index_ray, boxes)
        index_tri, index_ray, location, distance = index_tri[order], index_ray[order], location[order], distance[order]
    if not multiple_hits and len(index_ray):
        first = np.array([g[distance[g].argmin()] for g in _grp.group(index_ray)])
        index_tri, index_ray, location = index_tri[first], index_ray[first], location[first]
    if return_locations:
        if len(index_tri) == 0:
            return index_tri, index_ray, location
        unique = _grp.unique_rows(np.column_stack((location, index_ray)))[0]
        return index_tri[unique], index_ray[unique], location[unique]
    return index_tri, index_ray


# ── Embree 找候选 ───────────────────────────────────────────────────────────
def _emb_intersector(mesh):
    c = mesh._cache
    ri = c["hr42_embree_ri"]
    if ri is None:
        ri = _embree_mod().RayMeshIntersector(mesh)
        c["hr42_embree_ri"] = ri
    return ri


def _embree_candidates(mesh, o, d):
    """Embree 多次命中迭代（起点后撤 _BACK_EPS）得到的 (ray, tri) + 每个命中点邻域盒内全部三角面（rtree 批量查询）。"""
    ri = _emb_intersector(mesh)
    du = _util.unitize(d)
    o2 = o - du * _BACK_EPS
    tri_e, ray_e, loc_e = ri.intersects_id(o2, d, multiple_hits=True, max_hits=_MAX_HITS, return_locations=True)
    if len(ray_e) == 0:
        return np.array([], dtype=np.int64), np.array([], dtype=np.int64)
    base = max(1e-8, float(mesh.scale) * 1e-6)
    # Embree 每次命中后前进 base 偏移（卡住时翻倍）→ 其间被跨过的面都在命中点 ±几倍 base 内；邻边/顶点上的并列面也在这里
    h = 8.0 * base + 1e-6
    ids, counts = _tree(mesh).intersection_v(np.ascontiguousarray(loc_e - h), np.ascontiguousarray(loc_e + h))
    rr = np.repeat(ray_e, counts.astype(np.int64))
    cand_ray = np.concatenate([ray_e, rr])
    cand_tri = np.concatenate([tri_e, ids.astype(np.int64)])
    pair = np.unique(np.column_stack((cand_ray, cand_tri)), axis=0)
    return pair[:, 0], pair[:, 1]


def _bulk_candidates(mesh, o, d, boxes):
    ids, counts = _tree(mesh).intersection_v(np.ascontiguousarray(boxes[:, :3]), np.ascontiguousarray(boxes[:, 3:]))
    return np.repeat(np.arange(len(o), dtype=np.int64), counts.astype(np.int64)), ids.astype(np.int64)


def _fast_id(mesh, o, d, multiple_hits, return_locations, need_order=True, op="rays"):
    b = backend(op)
    if b == "embree" and _embree_mod() is None:                     # 第五任：第一次真用 Embree 才 import；import 失败 → 退 bulk（结果同）
        b = "bulk"
    boxes = _ray_boxes(mesh, o, d)
    if b == "embree":
        cr, ct = _embree_candidates(mesh, o, d)
    else:
        cr, ct = _bulk_candidates(mesh, o, d, boxes)
    it, ir, loc, dist = _exact(mesh, o, d, cr, ct, boxes)
    if b == "embree" and len(ir) == 0 and return_locations and need_order:
        # 空结果的形状要照 trimesh：用全部 rtree 候选再判一次"有没有不平行的候选面"（只在整批都没命中时做）
        cr2, ct2 = _bulk_candidates(mesh, o, d, boxes)
        it, ir, loc, dist = _exact(mesh, o, d, cr2, ct2, boxes)
    return _finish(mesh, o, d, it, ir, loc, dist, boxes, multiple_hits, return_locations, need_order=need_order)


# ── 对账 ─────────────────────────────────────────────────────────────────────
def _verify_on():
    return bool(os.environ.get("DUCK_RAY_VERIFY"))


def _vwrite(row):
    p = os.environ.get("DUCK_RAY_VERIFY")
    if not p:
        return
    _S["verify"].append(row)
    rows = _S["verify"]
    summ = dict(backend=name(), pid=os.getpid(), calls=len(rows),
                rays=int(sum(r.get("n", 0) for r in rows)),
                mismatched_calls=int(sum(1 for r in rows if not r["same"])),
                by_kind={},
                rows=rows[-2000:])
    for r in rows:
        k = summ["by_kind"].setdefault(r["kind"], dict(calls=0, n=0, mismatched=0, max_loc_diff=0.0))
        k["calls"] += 1
        k["n"] += int(r.get("n", 0))
        k["mismatched"] += int(not r["same"])
        k["max_loc_diff"] = max(k["max_loc_diff"], float(r.get("max_loc_diff", 0.0) or 0.0))
    path = p if not os.environ.get("DUCK_RAY_VERIFY_PER_PID") else f"{p}.{os.getpid()}.json"
    tmp = f"{path}.{os.getpid()}.tmp"
    with open(tmp, "w", encoding="utf-8") as f:
        json.dump(summ, f, ensure_ascii=False, indent=1)
    os.replace(tmp, path)


def _cmp_loc(kind, a, b, n, t_fast, t_ref):
    la, ra, ta = a
    lb, rb, tb = b
    same_n = len(ra) == len(rb)
    row = dict(kind=kind, site=_S["site"], n=int(n), hits=int(len(ra)), hits_ref=int(len(rb)),
               t_fast=round(t_fast, 6), t_ref=round(t_ref, 6))
    if same_n and len(ra):
        row["index_ray_equal"] = bool(np.array_equal(ra, rb))
        row["index_tri_equal"] = bool(np.array_equal(ta, tb))
        row["max_loc_diff"] = float(np.abs(np.asarray(la) - np.asarray(lb)).max())
        row["bitwise_loc"] = bool(np.array_equal(la, lb))
    else:
        row["index_ray_equal"] = same_n
        row["index_tri_equal"] = same_n
        row["max_loc_diff"] = 0.0 if same_n else None
        row["bitwise_loc"] = same_n
    row["same"] = bool(same_n and row["index_ray_equal"] and row["index_tri_equal"]
                       and (row["max_loc_diff"] is not None and row["max_loc_diff"] <= 1e-9))
    if not row["same"]:
        # 定位用：每条射线命中数
        ca = np.bincount(np.asarray(ra, dtype=np.int64), minlength=int(n)) if len(ra) else np.zeros(int(n), int)
        cb = np.bincount(np.asarray(rb, dtype=np.int64), minlength=int(n)) if len(rb) else np.zeros(int(n), int)
        bad = np.flatnonzero(ca != cb)
        row["rays_count_diff"] = [[int(i), int(ca[i]), int(cb[i])] for i in bad[:20]]
    return row


# ── 公开接口 ────────────────────────────────────────────────────────────────
def intersects_location(mesh, origins, directions, multiple_hits=True):
    """= mesh.ray.intersects_location(origins, directions, multiple_hits=multiple_hits)（trimesh ray_triangle 同形同序同值）。"""
    o, d = _as_rays(origins, directions)
    b = backend("rays")
    if b == "triangle":
        return mesh.ray.intersects_location(o, d, multiple_hits=multiple_hits)
    t0 = time.perf_counter()
    it, ir, loc = _fast_id(mesh, o, d, multiple_hits, True)
    res = (loc, ir, it)
    if _verify_on():
        t1 = time.perf_counter()
        ref = mesh.ray.intersects_location(o, d, multiple_hits=multiple_hits)
        t2 = time.perf_counter()
        _vwrite(_cmp_loc("intersects_location" + ("" if multiple_hits else "(first)"), res, ref, len(o), t1 - t0, t2 - t1))
    return res


def intersects_id(mesh, origins, directions, multiple_hits=True, return_locations=False):
    """= mesh.ray.intersects_id(...)（trimesh ray_triangle：return_locations=False 时不去重、按候选顺序）。"""
    o, d = _as_rays(origins, directions)
    b = backend("rays")
    if b == "triangle":
        return mesh.ray.intersects_id(o, d, multiple_hits=multiple_hits, return_locations=return_locations)
    t0 = time.perf_counter()
    res = _fast_id(mesh, o, d, multiple_hits, return_locations)
    if _verify_on():
        t1 = time.perf_counter()
        ref = mesh.ray.intersects_id(o, d, multiple_hits=multiple_hits, return_locations=return_locations)
        t2 = time.perf_counter()
        if return_locations:
            row = _cmp_loc("intersects_id+loc", (res[2], res[1], res[0]), (ref[2], ref[1], ref[0]), len(o), t1 - t0, t2 - t1)
        else:
            z = np.zeros((len(res[1]), 3)); zr = np.zeros((len(ref[1]), 3))
            row = _cmp_loc("intersects_id", (z, res[1], res[0]), (zr, ref[1], ref[0]), len(o), t1 - t0, t2 - t1)
        _vwrite(row)
    return res


def intersects_any(mesh, origins, directions):
    """= mesh.ray.intersects_any(origins, directions)。"""
    o, d = _as_rays(origins, directions)
    b = backend("rays")
    if b == "triangle":
        return mesh.ray.intersects_any(o, d)
    t0 = time.perf_counter()
    _it, ir = _fast_id(mesh, o, d, True, False, need_order=False)
    out = np.zeros(len(o), dtype=bool)
    hit_idx = np.unique(ir)
    if len(hit_idx) > 0:
        out[hit_idx] = True
    if _verify_on():
        t1 = time.perf_counter()
        ref = np.asarray(mesh.ray.intersects_any(o, d))
        t2 = time.perf_counter()
        dis = int((out != ref).sum())
        _vwrite(dict(kind="intersects_any", site=_S["site"], n=int(len(o)), hits=int(out.sum()), hits_ref=int(ref.sum()),
                     disagree=dis, same=dis == 0, t_fast=round(t1 - t0, 6), t_ref=round(t2 - t1, 6)))
    return out


class _ExactIntersector:
    """给 trimesh.ray.ray_util.contains_points 用的"求交器"：intersects_location 走本模块（计数只看 index_ray → 不必排 rtree 名次）。"""

    def __init__(self, mesh):
        self.mesh = mesh

    def intersects_location(self, ray_origins, ray_directions, multiple_hits=True, **_kw):
        o, d = _as_rays(ray_origins, ray_directions)
        it, ir, loc = _fast_id(self.mesh, o, d, multiple_hits, True, need_order=False, op="contains")
        return loc, ir, it


def contains(mesh, points):
    """= mesh.contains(points)（trimesh ray_util.contains_points，同一奇偶规则、同一默认方向；
    "两向不一致"的点 trimesh 自己用随机方向重查 —— 那一步基线本身就不确定，原样保留）。"""
    p = np.asanyarray(points, dtype=np.float64)
    b = backend("contains")
    if b == "triangle":
        return np.asarray(mesh.contains(p))
    from trimesh.ray.ray_util import contains_points
    t0 = time.perf_counter()
    out = np.asarray(contains_points(_ExactIntersector(mesh), p))
    if _verify_on():
        t1 = time.perf_counter()
        ref = np.asarray(mesh.contains(p))
        t2 = time.perf_counter()
        dis = int((out != ref).sum())
        _vwrite(dict(kind="contains", site=_S["site"], n=int(len(p)), hits=int(out.sum()), hits_ref=int(ref.sum()),
                     disagree=dis, same=dis == 0, t_fast=round(t1 - t0, 6), t_ref=round(t2 - t1, 6)))
    return out


def split_by_ray(n_rays, locations, index_ray, index_tri=None):
    """批量求交结果按射线拆回去：返回长度 n_rays 的列表，第 i 项 = 第 i 条射线的 (locations, index_tri)，
    行序 = 批量结果里该射线的行序（trimesh 对每条射线的行序与单独调用时相同：unique_rows 的键里射线号是最后一列、同一射线内按位置字节排）。"""
    index_ray = np.asarray(index_ray, dtype=np.int64)
    order = np.argsort(index_ray, kind="stable")
    bounds = np.searchsorted(index_ray[order], np.arange(n_rays + 1))
    out = []
    for i in range(n_rays):
        sel = order[bounds[i]:bounds[i + 1]]
        out.append((np.asarray(locations)[sel], None if index_tri is None else np.asarray(index_tri)[sel]))
    return out
