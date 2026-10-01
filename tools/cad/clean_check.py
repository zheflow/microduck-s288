#!/usr/bin/env python
"""hr50（2026-09-28）打印件"干净度"检查：用户不想在打印件上看到的几类问题，阈值以原版 Microduck 打印件标定。

检查类别（--checks，默认全部）
  thin   薄片：比一条挤出线（T_LINE = 0.45 mm）还薄的料。逐点向内射线量厚度，薄点聚簇后分三种：
           薄膜 membrane —— 两面法向成对（对面那面与本面近似平行反向），薄区从厚料往里伸 ≥ MEM_DEPTH：打出来是软片或洞；
           刀片 blade    —— 楔形（不成对），薄带往里伸 > BLADE_DEPTH_STRICT（尖角 < 45°）：尖头缺一截、边缘碎；
           楔尖 tip      —— 薄带更窄的尖角 / 棱：打印自然圆掉，不报（簇仍全部写在 clusters 里）。
         两套口径（--profile，每个簇同时写 strict_flag / orig_flag）：
           strict = 纯物理：薄膜 面积 ≥ 0.5 且往里伸 ≥ 0.25；楔形 面积 ≥ 0.5 且往里伸 > 0.45；
           orig（默认）= 以原版为准：原版打印件（完整网格 + head_scale 放大后的头壳；onshape-to-robot 减面的 4 个网格不参与）本来就有的同级问题不报 ——
                  薄膜在包络（面积 ≤ 13.4、往里伸 ≤ 1.172、中位厚 ≥ 0.27）里不报；楔形往里伸 ≤ 0.581（原版最大）不报。
  frag   断裂 / 尖刺 / 碎片：
           debris —— 与主体不连通的小碎体；
           spike  —— 针状尖刺：顶点周围 SPIKE_R 球内的高斯曲率积分（角亏）≥ SPIKE_DEG（立方体角 = 90°，针尖 → 360°），且尖端到厚料 > SPIKE_LEN；
           ragged —— 碎边：锐边（≥30°）按分叉点断成折线，短折线（0.05..1 mm）在 1.5 mm 内扎堆 ≥ RAG_N 条（区域刀的"马赛克"切面、采样扫掠的锯齿台阶、
                     减面网格孔沿的碎三角）。整体起伏 ≤ 0.1 mm 的记 micro_seam、都在一个 ±0.5 薄平板里且方向只有面内 / 沿法向的记 planar_detail（刻字），
                     这两种只进 info 不报。原版完整网格上 ragged / spike 都是 0 处。
  holes  螺丝孔 / 沉窝侧壁部分开口 + 孔沿开口环：tools/cad/clean_check_holes.py（孔检测子 agent 写，判据与标定见 hr50_work/algo/holes_calibration.md）：
           不依赖世界轴找圆柱孔（Ø1.2–30）；部分开口 = r+0.3 圈缺 10–90%，按形态分 mid / 口部不规则（报）与 倒角长度孔口 / 斜出口 / 口部单侧平开（不报，
           原版同形 26 处）；开口环 = 孔壁到 r+1.0 径向切穿，整段贯通且断口中位宽 ≤ 60° 才报（N01 Ø16 惰轮孔 21.5°）。原版 34 个文件 0 处。
           每个孔带 use（own 本件螺丝孔 / channel 起子或螺丝头通道 / none 没有螺丝用它 / n/a 原版与打印包）。

只读：不写 STL、不改数据。Embree（.venv/hr42_embree）只在本进程里用；本进程不 import manifold3d（两份 TBB 同进程会崩，见 ray_backend.py 说明）。
坐标 = 输入文件自己的坐标系（我们的件用 cad/duck_s288/placed/*.stl = 世界系，与缺口账本同系；原版用 cad/microduck_orig/placed/*.stl）。

用法
  ./.venv/bin/python -B tools/cad/clean_check.py --set ours [--parts L04,H03] --out X.json [--jobs 3]
  ./.venv/bin/python -B tools/cad/clean_check.py --set orig --out Y.json
  ./.venv/bin/python -B tools/cad/clean_check.py --set pack:<打印包目录> --out Z.json
  ./.venv/bin/python -B tools/cad/clean_check.py a.stl b.stl --out W.json
"""
import os, sys, json, time, glob, argparse, subprocess, resource, re, hashlib

import numpy as np

ROOT = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", ".."))
HERE = os.path.dirname(os.path.abspath(__file__))
ALGO = os.path.join(ROOT, "docs", "design_2026-09-17_bearing_rebuild", "hr50_work", "algo")
# features.yaml 孔类特征转世界系、阵列逐孔展开（孔检测子 agent 09-28 重导：258 条，一孔一轴；未展开的与旧文件差 < 6e-14 mm）
FEATURES_JSON = os.path.join(ALGO, "holes_work", "hole_features_world_expanded.json")

# 我们的 37 种打印件（hr52 + H10）→ placed 名（= cut_ledger 的 builders；左件，右件是镜像）
PLACED = {"B01": "battery_door", "B03": "battery_lid", "E01": "eye_white", "E02": "eye_pupil", "E03": "eye_highlight",
          "H01": "head_bracket", "H02": "head_clamp", "H03": "head_bottom_shell", "H04": "face_plate", "H05": "head_top_shell",
          "H06": "camera_clamp", "H08": "adapter_tray", "H09": "amp_bracket", "H10": "tof_clamp", "J01": "jaw", "J02": "jaw_adapter", "J03": "jaw_journal",
          "L01": "yaw2roll", "L02": "hip", "L03": "upper_leg", "L04": "lower_leg", "L05": "ankle_foot", "L06": "sole",
          "L07": "ankle_rear_arm", "L10": "hip_roll_sleeve", "L13": "hip_pitch_sleeve", "N01": "neck", "N02": "neck_pitch",
          "N03": "yrm", "N04": "head_journal", "N05": "head_roll_adapter", "N06": "head_yaw_adapter", "N07": "head_bearing_cap",
          "N08": "head_yaw_cap", "T01": "trunk", "T02": "shell_L", "T03": "shell_R"}
ORIG_SKIP = ("servo__", "pcb__", "elec_", "np_f970", "lens__", "speaker__")      # 原版里不是打印件的

# ── 参数（标定后的值；每次输出都把实际用的参数写进 JSON）────────────────────────────────
P = dict(
    T_LINE=0.45,        # 一条挤出线宽（0.4 喷嘴常用线宽），薄于此 = 放不下一条线
    S=0.15,             # 表面采样间距 mm
    EPS=1e-3,           # 射线起点往里退 mm（避开本面）
    CLUSTER_R=0.5,      # 薄点聚簇半径 mm
    PAIR_COS=-0.94,     # 对面法向与本面法向夹角 ≥ 160°（cos ≤ −0.94）= 两面成对（薄膜 / 极尖刀片）
    PAIR_FRAC=0.5,      # 簇内成对面积占比 ≥ 此值 = 薄膜
    MEM_DEPTH=0.25,     # 薄膜：薄区往里伸 ≥ 0.25 mm（= 薄膜宽 ≥ ~0.5 mm，约一条线宽）才报
    BLADE_DEPTH_STRICT=0.45,   # 楔形：薄带往里伸 > 0.45 mm（尖角 < 45°）= 刀片（纯物理口径）
    BLADE_DEPTH_ORIG=0.581,    # 以原版为准：原版（完整网格 + 放大头壳）楔尖往里伸的最大值 0.581（scaled jaw @ (23.45, 49.32, 226.24)），超过才报
    MIN_AREA=0.5,       # 薄簇面积（单面计）≥ 0.5 mm² 才报
    DEBRIS_MM3=1.0,     # 小碎体：体积 < 1 mm³（或 < 主体 0.5%）
    DEBRIS_FRAC=0.005,
    SPIKE_R=0.3,        # 尖刺：角亏在 0.3 mm 球内求和
    SPIKE_DEG=200.0,    # ≥ 200°（针尖 / 三面都尖的角；立方体角 90°、30° 楔棱柱角 150°）
    SPIKE_LEN=0.15,     # 且尖端到最近"厚料"（厚度 ≥ 一条线宽）> 0.15 mm 才报：原版最大 0.12（top_head_shell (79.1, 40.51, 250.4)），采样间距 0.15 是分辨率下限
    SHARP_DEG=30.0,     # 锐边：两面夹角（法向差）≥ 30°
    TURN_DEG=180.0,     # 锐边折线只在分叉点（锐边度数 ≠ 2）断开，不按转角断：干净的相贯线三角化后是锯齿折线，按转角断会被切成一堆短线误报（09-28 标定）
    RAG_MIN=0.05,       # 比 0.05 mm 还短的锐边折线不算（网格细碎、看不见也打不出）
    RAG_SHORT=1.0,      # 短锐边折线 < 1.0 mm
    RAG_R=1.5,          # 扎堆半径
    RAG_N=6,            # 1.5 mm 内 ≥ 6 条短锐边折线 = 碎边。09-28 标定：RAG_N = 12/8/6/5 时原版完整网格一处都不报，我们报 59/69/82/90；
                        #   逐个看图：6 比 8 多出来的主要是按角度采样的扫掠锯齿台阶（真的），5 比 6 多出来的是干净的小台阶小缺口（误报）→ 取 6
    PLANAR_SLAB=0.5,    # 扎堆的短锐边都在一个 ±0.5 mm 的平板里、且方向只有"面内 / 沿法向"两种 = 平面细节（刻字、花纹），只记 info
    PLANAR_FRAC=0.85,
    MICRO_SLAB=0.1,     # 扎堆的短锐边整体起伏 ≤ 0.1 mm（共面的布尔接缝 / 微台阶、平面上的相贯线）= 看不见也打不出，只记 info
    PROFILE="orig",     # orig = 以原版为准（原版打印件本来就有的同级薄膜不报）；strict = 纯物理口径
    # 以原版为准的薄膜包络（2026-09-28 标定：原版完整网格 + head_scale 放大后的原版头壳上所有薄膜的上限；简化网格不参与）：
    #   面积 ≤ 13.4 mm²（原版 ankle 左右各一片 13.1 / 13.4）、往里伸 ≤ 1.17 mm（放大后的 face_part 0.3 mm 隔膜 1.15 / 1.17；未放大 0.97）、
    #   中位厚 ≥ 0.27 mm（原版最薄的中位厚 0.279）。三条都在包络里才不报。
    ENV_AREA=13.4,
    ENV_DEPTH=1.172,
    ENV_TMED=0.27,
)


def rss_gb():
    r = resource.getrusage(resource.RUSAGE_SELF).ru_maxrss
    return r / 1e9 if sys.platform == "darwin" else r / 1e6


# ── 网格 / 射线 ─────────────────────────────────────────────────────────────────────────
def load_mesh(path):
    import trimesh
    m = trimesh.load(path, force="mesh", process=True)
    info = dict(faces=int(len(m.faces)), vertices=int(len(m.vertices)), watertight=bool(m.is_watertight),
                winding_consistent=bool(m.is_winding_consistent))
    # 朝向：不水密的文件 m.volume 不可靠，用三角形有向体积和的符号判断整体是不是里外翻了（inside-out）。翻了就翻回来再查，并记成文件问题。
    import trimesh.triangles as _tt
    sv = float(_tt.mass_properties(m.triangles, skip_inertia=True)["volume"])
    issues = []
    if sv < 0:
        m.invert(); info["inverted_fixed"] = True; sv = -sv
        issues.append("整件法向朝里（inside-out），已翻回再查；切片软件多半会自动修，但文件本身是反的")
    if not info["watertight"]:
        nm = int(len(trimesh.grouping.group_rows(m.edges_sorted, require_count=1)))
        issues.append(f"不水密（开边 {nm}；其余是非流形 / 退化面）")
    info["volume_mm3"] = round(sv, 2)
    info["file_issues"] = issues
    info["area_mm2"] = round(float(m.area), 2)
    info["bounds"] = np.round(m.bounds, 3).tolist()
    # 原版仿真网格经 onshape-to-robot 简化（microduck_rl config_mjcf_*.json: simplify_stls=true, max_stl_size=1.0 MB）：
    # 超过 1 MiB 的件被减面到正好顶格（20970 面 × 50 B + 84 = 1 048 584 B），减面会在孔沿 / 细节处留下尖刺和碎三角。
    try:
        nb = os.path.getsize(path)
        info["stl_bytes"] = int(nb)
        info["simplified_suspect"] = bool(1_040_000 <= nb <= 1_048_700)
    except OSError:
        pass
    return m, info


def intersector(m):
    emb = os.path.join(ROOT, ".venv", "hr42_embree")
    if emb not in sys.path:
        sys.path.append(emb)
    from trimesh.ray.ray_pyembree import RayMeshIntersector
    return RayMeshIntersector(m)


def first_hit(inter, O, D, chunk=400_000):
    """每条射线的第一个命中：返回 (距离, 命中面)；没命中 = (inf, −1)。"""
    t = np.full(len(O), np.inf); h = np.full(len(O), -1, np.int64)
    for k0 in range(0, len(O), chunk):
        o, d = O[k0:k0 + chunk], D[k0:k0 + chunk]
        it, ir, loc = inter.intersects_id(o, d, multiple_hits=False, return_locations=True)
        if len(ir):
            t[k0 + ir] = np.linalg.norm(loc - o[ir], axis=1); h[k0 + ir] = it
    return t, h


def sample_surface(m, s):
    """确定性表面采样：每面 max(面积/s², 最长边/s) 个点（R2 低差异序列折进三角形），单点面取形心。返回 点、面号、每点代表面积。"""
    A = m.area_faces
    tri = m.triangles
    L = np.linalg.norm(tri[:, [1, 2, 0]] - tri, axis=2).max(1)
    n = np.maximum(np.ceil(A / (s * s)), np.ceil(L / s)).astype(np.int64)
    n = np.maximum(n, 1)
    fid = np.repeat(np.arange(len(A)), n)
    start = np.cumsum(n) - n
    k = np.arange(int(n.sum())) - np.repeat(start, n)
    g = 1.32471795724474602596
    u = (0.5 + (k + 1) / g) % 1.0
    v = (0.5 + (k + 1) / (g * g)) % 1.0
    flip = (u + v) > 1.0
    u[flip] = 1.0 - u[flip]; v[flip] = 1.0 - v[flip]
    one = n[fid] == 1
    u[one] = 1.0 / 3; v[one] = 1.0 / 3
    t0 = tri[fid, 0]
    Pt = t0 + u[:, None] * (tri[fid, 1] - t0) + v[:, None] * (tri[fid, 2] - t0)
    w = (A / n)[fid]
    return Pt, fid, w


def _components(n, pairs):
    from scipy.sparse import coo_matrix
    from scipy.sparse.csgraph import connected_components
    if n == 0:
        return 0, np.zeros(0, np.int64)
    if len(pairs) == 0:
        return n, np.arange(n)
    g = coo_matrix((np.ones(len(pairs)), (pairs[:, 0], pairs[:, 1])), shape=(n, n))
    return connected_components(g, directed=False)


# ── thin：薄片 ─────────────────────────────────────────────────────────────────────────
def check_thin(m, inter, ctx):
    from scipy.spatial import cKDTree
    s = P["S"]
    X, fid, w = sample_surface(m, s)
    N = m.face_normals[fid]
    t, h = first_hit(inter, X - N * P["EPS"], -N)
    t = t + P["EPS"]
    thin = t < P["T_LINE"]
    ctx["samples"] = dict(X=X, N=N, t=t, fid=fid, w=w, thin=thin)       # 给 frag 用（尖刺长度）
    out = dict(n_samples=int(len(X)), thin_area_mm2=round(float(w[thin].sum()), 2), clusters=[], findings=[])
    if not thin.any():
        out["status"] = "PASS"
        return out
    ti = np.nonzero(thin)[0]
    cos = np.full(len(ti), 1.0)
    ok = h[ti] >= 0
    cos[ok] = np.einsum("ij,ij->i", N[ti[ok]], m.face_normals[h[ti[ok]]])
    paired = cos <= P["PAIR_COS"]
    # 薄区往里伸多深：到最近"同侧"厚点（法向同半球，避免跨过窄缝量到对面墙）的距离
    thick = np.nonzero(~thin)[0]
    depth = np.full(len(ti), np.inf)
    if len(thick):
        tree = cKDTree(X[thick])
        k = min(32, len(thick))
        dd, ii = tree.query(X[ti], k=k)
        dd = dd.reshape(len(ti), k); ii = ii.reshape(len(ti), k)
        same = np.einsum("ijk,ik->ij", N[thick][ii], N[ti]) > 0.0
        first = np.where(same.any(1), same.argmax(1), k - 1)
        depth = dd[np.arange(len(ti)), first]
    # 聚簇
    tt = cKDTree(X[ti])
    pairs = tt.query_pairs(P["CLUSTER_R"], output_type="ndarray")
    nc, lab = _components(len(ti), pairs)
    rows = []
    for c in range(nc):
        sel = np.nonzero(lab == c)[0]
        wi = w[ti[sel]]; a = float(wi.sum())
        if a < 0.05:
            continue
        pf = float(wi[paired[sel]].sum() / a)
        dep = float(np.percentile(depth[sel], 98))
        Xi = X[ti[sel]]
        tmed = float(np.median(t[ti[sel]]))
        big = a >= P["MIN_AREA"]
        if pf >= P["PAIR_FRAC"]:
            kind = "membrane"
            strict_flag = big and dep >= P["MEM_DEPTH"]
            in_env = a <= P["ENV_AREA"] and dep <= P["ENV_DEPTH"] and tmed >= P["ENV_TMED"]
            orig_flag = strict_flag and not in_env
        else:
            kind = "blade" if dep > P["BLADE_DEPTH_STRICT"] else "tip"
            strict_flag = big and dep > P["BLADE_DEPTH_STRICT"]
            in_env = dep <= P["BLADE_DEPTH_ORIG"]
            orig_flag = big and dep > P["BLADE_DEPTH_ORIG"]
        flag = orig_flag if P["PROFILE"] == "orig" else strict_flag
        rows.append(dict(kind=kind, flag=bool(flag), strict_flag=bool(strict_flag), orig_flag=bool(orig_flag), within_orig_envelope=bool(in_env),
                         area_mm2=round(a, 3), t_min=round(float(t[ti[sel]].min()), 3),
                         t_med=round(tmed, 3), paired_frac=round(pf, 2), depth_mm=round(dep, 3),
                         c=np.round((Xi * wi[:, None]).sum(0) / a, 2).tolist(), lo=np.round(Xi.min(0), 2).tolist(),
                         hi=np.round(Xi.max(0), 2).tolist(), n=int(len(sel)),
                         pts=np.round(Xi[np.linspace(0, len(Xi) - 1, min(24, len(Xi))).astype(int)], 2).tolist()))
    rows.sort(key=lambda r: (-r["flag"], -r["area_mm2"]))
    out["clusters"] = rows[:400]
    out["n_clusters"] = len(rows)
    out["findings"] = [r for r in rows if r["flag"]]
    out["n_flag_strict"] = sum(1 for r in rows if r["strict_flag"]); out["n_flag_orig"] = sum(1 for r in rows if r["orig_flag"])
    out["by_kind"] = {k: dict(n=sum(1 for r in rows if r["kind"] == k), area=round(sum(r["area_mm2"] for r in rows if r["kind"] == k), 2))
                      for k in ("membrane", "blade", "tip")}
    out["status"] = "FAIL" if out["findings"] else "PASS"
    return out


# ── frag：碎体 / 尖刺 / 碎边 ────────────────────────────────────────────────────────────
def check_frag(m, inter, ctx):
    from scipy.spatial import cKDTree
    out = dict(debris=[], spikes=[], ragged=[], findings=[])
    # 1) 碎体
    nc, lab = _components(len(m.faces), m.face_adjacency)
    if nc > 1:
        comps = []
        for c in range(nc):
            f = np.nonzero(lab == c)[0]
            sub = m.submesh([f], append=True)
            vol = abs(float(sub.volume)) if sub.is_watertight else None
            comps.append(dict(faces=int(len(f)), volume_mm3=None if vol is None else round(vol, 3), area_mm2=round(float(sub.area), 3),
                              lo=np.round(sub.bounds[0], 2).tolist(), hi=np.round(sub.bounds[1], 2).tolist()))
        comps.sort(key=lambda r: -(r["volume_mm3"] or 0))
        main = comps[0]["volume_mm3"] or 0
        out["degenerate"] = []
        for r in comps[1:]:
            v = r["volume_mm3"]
            if r["area_mm2"] < 0.01 or (v is not None and v < 1e-3 and r["faces"] <= 4):
                # 零面积 / 零体积的几个退化三角（导出文件瑕疵，非水密的来源之一）：不是能打出来的料，记 info（文件体检见 print_file_check.py）
                r["kind"] = "degenerate"; out["degenerate"].append(r)
            elif v is None or v < P["DEBRIS_MM3"] or v < P["DEBRIS_FRAC"] * main:
                r["kind"] = "debris"; out["debris"].append(r)
        out["n_bodies"] = nc
    else:
        out["n_bodies"] = 1
    # 2) 尖刺：角亏球内求和
    V = m.vertices
    asum = np.bincount(m.faces.ravel(), weights=m.face_angles.ravel(), minlength=len(V))
    used = np.bincount(m.faces.ravel(), minlength=len(V)) > 0
    defect = np.where(used, 2 * np.pi - asum, 0.0)
    cand = np.nonzero(defect > np.radians(45))[0]
    if len(cand):
        tv = cKDTree(V)
        nb = tv.query_ball_point(V[cand], P["SPIKE_R"])
        tot = np.array([defect[j].sum() for j in nb])
        hit = cand[np.degrees(tot) >= P["SPIKE_DEG"]]
        if len(hit):
            tot_hit = np.degrees(tot[np.degrees(tot) >= P["SPIKE_DEG"]])
            th = cKDTree(V[hit]); pr = th.query_pairs(0.6, output_type="ndarray")
            n2, lab2 = _components(len(hit), pr)
            S = ctx.get("samples")
            for c in range(n2):
                sel = np.nonzero(lab2 == c)[0]
                j = sel[np.argmax(tot_hit[sel])]
                p = V[hit[j]]
                length = None
                if S is not None and (~S["thin"]).any():
                    # 尖刺长度：尖端到最近"厚"表面点（厚度 ≥ 一条线宽）的距离
                    if "thick_tree" not in ctx:
                        ctx["thick_tree"] = cKDTree(S["X"][~S["thin"]])
                    length = float(ctx["thick_tree"].query(p)[0])
                row = dict(kind="spike", deg=round(float(tot_hit[j]), 1), p=np.round(p, 2).tolist(),
                           n_vertices=int(len(sel)), length_mm=None if length is None else round(length, 3))
                row["flag"] = length is None or length > P["SPIKE_LEN"]
                out["spikes"].append(row)
    # 3) 碎边：短锐边折线扎堆
    fa = m.face_adjacency; ang = np.degrees(m.face_adjacency_angles); E = m.face_adjacency_edges
    sh = ang >= P["SHARP_DEG"]
    Es = E[sh]
    if len(Es):
        deg = np.bincount(Es.ravel(), minlength=len(V))
        # 顶点处转角：只对度 = 2 的顶点算
        vec = V[Es[:, 1]] - V[Es[:, 0]]
        ln = np.linalg.norm(vec, axis=1); ln[ln == 0] = 1e-12
        u = vec / ln[:, None]
        brk = deg != 2
        two = np.nonzero(deg == 2)[0]
        if len(two):
            # 每个度 2 顶点的两条边
            inc = np.concatenate([np.column_stack([Es[:, 0], np.arange(len(Es))]), np.column_stack([Es[:, 1], np.arange(len(Es))])])
            inc = inc[np.argsort(inc[:, 0], kind="stable")]
            first = np.searchsorted(inc[:, 0], two)
            e1, e2 = inc[first, 1], inc[first + 1, 1]
            # 方向统一成"离开该顶点"
            d1 = np.where((Es[e1, 0] == two)[:, None], u[e1], -u[e1])
            d2 = np.where((Es[e2, 0] == two)[:, None], u[e2], -u[e2])
            turn = 180.0 - np.degrees(np.arccos(np.clip(np.einsum("ij,ij->i", d1, d2), -1, 1)))
            brk[two[turn > P["TURN_DEG"]]] = True
        # 边图：两条锐边共享一个"非断点"顶点 → 同一折线
        inc = np.concatenate([np.column_stack([Es[:, 0], np.arange(len(Es))]), np.column_stack([Es[:, 1], np.arange(len(Es))])])
        inc = inc[~brk[inc[:, 0]]]
        inc = inc[np.argsort(inc[:, 0], kind="stable")]
        same = inc[1:, 0] == inc[:-1, 0]
        pairs = np.column_stack([inc[:-1, 1][same], inc[1:, 1][same]])
        npl, pl = _components(len(Es), pairs)
        plen = np.bincount(pl, weights=ln, minlength=npl)
        mid = (V[Es[:, 0]] + V[Es[:, 1]]) / 2
        pc = np.zeros((npl, 3)); np.add.at(pc, pl, mid * ln[:, None]); pc /= np.maximum(plen, 1e-12)[:, None]
        short = np.nonzero((plen < P["RAG_SHORT"]) & (plen >= P["RAG_MIN"]))[0]
        out["n_sharp_polylines"] = int(npl); out["n_short_polylines"] = int(len(short))
        if len(short) >= P["RAG_N"]:
            ts = cKDTree(pc[short])
            cnt = np.array([len(x) for x in ts.query_ball_point(pc[short], P["RAG_R"])])
            dense = short[cnt >= P["RAG_N"]]
            if len(dense):
                td = cKDTree(pc[dense]); pr = td.query_pairs(P["RAG_R"], output_type="ndarray")
                n3, lab3 = _components(len(dense), pr)
                for c in range(n3):
                    sel = dense[lab3 == c]
                    q = pc[sel]
                    # 这一堆里（含扎堆邻域）一共多少条短折线
                    allq = set()
                    for x in ts.query_ball_point(q, P["RAG_R"]):
                        allq.update(x)
                    ids = short[sorted(allq)]
                    Q = pc[ids]
                    # 平面细节判定（刻字 / 花纹）：这些折线的锐边是否都在一个薄平板里，方向只有面内或沿法向
                    em = np.isin(pl, ids)
                    mids, ul, lw = mid[em], u[em], ln[em]
                    cc = (mids * lw[:, None]).sum(0) / lw.sum()
                    _, sv, vt = np.linalg.svd((mids - cc) * np.sqrt(lw)[:, None], full_matrices=False)
                    nrm = vt[2]
                    slab = float(np.percentile(np.abs((mids - cc) @ nrm), 95))
                    dn = np.abs(ul @ nrm)
                    frac = float(lw[(dn < 0.26) | (dn > 0.97)].sum() / lw.sum())
                    planar = slab <= P["PLANAR_SLAB"] and frac >= P["PLANAR_FRAC"]
                    micro = slab <= P["MICRO_SLAB"]
                    kind_r = "micro_seam" if micro else ("planar_detail" if planar else "ragged")
                    out["ragged"].append(dict(kind=kind_r, flag=kind_r == "ragged", n_short=int(len(ids)),
                                              short_len_mm=round(float(plen[ids].sum()), 2), slab_mm=round(slab, 3), axis_frac=round(frac, 2),
                                              c=np.round(Q.mean(0), 2).tolist(), lo=np.round(Q.min(0), 2).tolist(), hi=np.round(Q.max(0), 2).tolist(),
                                              pts=np.round(Q[np.linspace(0, len(Q) - 1, min(24, len(Q))).astype(int)], 2).tolist()))
    for r in out["debris"]:
        r["flag"] = True
    out["findings"] = [r for r in out["debris"] + out["spikes"] + out["ragged"] if r.get("flag")]
    out["info"] = [r for r in out["spikes"] + out["ragged"] if not r.get("flag")] + out.get("degenerate", [])
    out["status"] = "FAIL" if out["findings"] else "PASS"
    return out


# ── holes：螺丝孔 / 沉窝侧壁开口 + 孔沿开口环（子模块）───────────────────────────────────
def check_holes_wrap(m, inter, ctx):
    try:
        sys.path.insert(0, HERE)
        import clean_check_holes as CH
    except Exception as e:                                             # noqa: BLE001
        return dict(status="NOT_RUN", reason=f"clean_check_holes 不可用：{e!r}")
    return CH.check_holes(m, params=None, screw_features=ctx.get("features"))


CHECKS = {"thin": check_thin, "frag": check_frag, "holes": check_holes_wrap}


EXEMPT_JSON = os.path.join(HERE, "clean_check_exempt.json")


def apply_exemptions(res, features):
    """显式豁免表（clean_check_exempt.json）：只对世界系的我们的件（有 features）生效。命中的发现从 findings 挪到 exempted，不改阈值；
    该件登记了却没命中的条目记 stale_exemptions（几何变了要重新确认）。"""
    h = res["checks"].get("holes")
    if not features or not h or h.get("status") not in ("PASS", "FAIL") or not os.path.exists(EXEMPT_JSON):
        return
    pn = features["own"][0].get("placed")
    pid = next((k for k, v in PLACED.items() if v == pn), None)
    E = json.load(open(EXEMPT_JSON, encoding="utf-8"))
    tc, td = E["tol_center_mm"], E["tol_d_mm"]
    mine = [e for e in E["entries"] if e["part"] == pid]
    used = set()
    for kind, key in (("hole", "findings"), ("ring", "open_ring_findings")):
        keep = []
        for f in h.get(key) or []:
            e = next((e for e in mine if e["kind"] == kind and abs(f["d_mm"] - e["d_mm"]) <= td
                      and float(np.linalg.norm(np.asarray(f["center"]) - e["center"])) <= tc), None)
            if e is None:
                keep.append(f)
            else:
                used.add(e["id"]); h.setdefault("exempted", []).append(dict(f, exempt_id=e["id"], exempt_why=e["why"], exempt_kind=kind))
        h[key] = keep
    stale = [e["id"] for e in mine if e["id"] not in used]
    if stale:
        h["stale_exemptions"] = stale
    h["exempt_table"] = dict(version=E["version"], sha256=hashlib.sha256(open(EXEMPT_JSON, "rb").read()).hexdigest()[:16])
    st = h.setdefault("stats", {})
    st["n_exempted"] = len(h.get("exempted") or [])                    # stats.n_flagged / n_flagged_by_use 是豁免前的数
    st["n_flagged_after_exempt"] = len(h.get("findings") or [])
    st["n_ring_flagged_after_exempt"] = len(h.get("open_ring_findings") or [])
    h["status"] = "FAIL" if (h.get("findings") or h.get("open_ring_findings")) else "PASS"


def run_file(path, label, checks, features=None):
    t0 = time.time()
    m, info = load_mesh(path)
    inter = intersector(m)
    ctx = dict(features=features)
    res = dict(label=label, file=os.path.relpath(path, ROOT), mesh=info, checks={})
    for c in checks:
        t1 = time.time()
        try:
            r = CHECKS[c](m, inter, ctx)
        except Exception as e:                                         # noqa: BLE001
            import traceback
            r = dict(status="ERROR", error=repr(e), trace=traceback.format_exc()[-1500:])
        r["seconds"] = round(time.time() - t1, 1)
        res["checks"][c] = r
    apply_exemptions(res, features)
    st = [r.get("status") for r in res["checks"].values()]
    res["status"] = "ERROR" if "ERROR" in st else ("FAIL" if "FAIL" in st else "PASS")
    res["seconds"] = round(time.time() - t0, 1)
    res["peak_rss_gb"] = round(rss_gb(), 2)
    return res


# ── 文件集合 ─────────────────────────────────────────────────────────────────────────
def file_set(spec, parts=None):
    out = []
    if spec == "ours":
        for pid, nm in PLACED.items():
            if parts and pid not in parts: continue
            out.append((os.path.join(ROOT, "cad", "duck_s288", "placed", nm + ".stl"), pid, nm))
    elif spec == "orig":
        for f in sorted(glob.glob(os.path.join(ROOT, "cad", "microduck_orig", "placed", "*.stl"))):
            b = os.path.basename(f)
            if b.startswith(ORIG_SKIP): continue
            lab = b[:-4]
            if parts and lab not in parts: continue
            out.append((f, lab, None))
    elif spec.startswith("pack:"):
        d = spec[5:]
        for f in sorted(glob.glob(os.path.join(d, "**", "*.stl"), recursive=True)):
            lab = os.path.basename(f)[:-4]
            if parts and not any(lab.startswith(p) for p in parts): continue
            out.append((f, lab, None))
    return out


def features_for(placed_name):
    """我们的件（placed 世界系）才有 features：{"own": 本件, "other": 别的件（认起子 / 螺丝头通道）}。原版、打印包（打印坐标系）→ None（use = n/a）。"""
    if not placed_name or not os.path.exists(FEATURES_JSON):
        return None
    F = json.load(open(FEATURES_JSON, encoding="utf-8"))["features"]
    own = [f for f in F if f.get("placed") == placed_name]
    return {"own": own, "other": [f for f in F if f.get("placed") != placed_name]} if own else None


def guess_placed(path):
    """直接给文件时认 placed（孔的用途要对 features）：文件名去 .stl 等于某个 placed 名，或以件号开头（L02_xxx.stl → hip）。
    还要确认文件在世界坐标：包围盒与 cad/duck_s288/placed/<placed>.stl 的包围盒各角差 ≤ 2 mm（打印包是打印坐标系，对不上 → 不对 features）。"""
    stem = os.path.basename(path)[:-4]
    pn = stem if stem in PLACED.values() else None
    if pn is None:
        m = re.match(r"^([A-Z]\d{2})(?:_|$)", stem)
        pn = PLACED.get(m.group(1)) if m else None
    if pn is None:
        return None
    ref = os.path.join(ROOT, "cad", "duck_s288", "placed", pn + ".stl")
    try:
        import trimesh
        b1 = trimesh.load(path, force="mesh").bounds; b2 = trimesh.load(ref, force="mesh").bounds
    except Exception:                                                  # noqa: BLE001
        return None
    return pn if float(np.abs(b1 - b2).max()) <= 2.0 else None


def mem_free_gb():
    try:
        o = subprocess.run(["vm_stat"], capture_output=True, text=True).stdout
        ps = int(re.search(r"page size of (\d+)", o).group(1))
        get = lambda k: int(re.search(rf"{k}:\s+(\d+)", o).group(1))
        return (get("Pages free") + get("Pages inactive") + get("Pages speculative")) * ps / 1e9
    except Exception:                                                  # noqa: BLE001
        return 99.0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("files", nargs="*")
    ap.add_argument("--set", default="")
    ap.add_argument("--parts", default="")
    ap.add_argument("--checks", default="thin,frag,holes")
    ap.add_argument("--out", default="")
    ap.add_argument("--jobs", type=int, default=2)
    ap.add_argument("--profile", default="orig", choices=["orig", "strict"])
    ap.add_argument("--param", action="append", default=[], help="覆盖参数 K=V（标定实验用），可重复")
    ap.add_argument("--child", default="")          # 内部：单文件子进程
    ap.add_argument("--label", default="")
    ap.add_argument("--placed", default="")
    ap.add_argument("--tmp", default="")
    a = ap.parse_args()
    checks = [c for c in a.checks.split(",") if c]
    P["PROFILE"] = a.profile
    for kv in a.param:
        k, v = kv.split("=", 1)
        P[k] = type(P[k])(v) if k in P else float(v)
    if a.child:
        feats = features_for(a.placed) if "holes" in checks else None
        r = run_file(a.child, a.label, checks, feats)
        r["placed"] = a.placed or None
        json.dump(r, open(a.tmp, "w"), ensure_ascii=False)
        return
    parts = [p for p in a.parts.split(",") if p]
    items = []
    if a.set:
        items += file_set(a.set, parts)
    for f in a.files:
        items.append((os.path.abspath(f), os.path.basename(f)[:-4], guess_placed(f)))
    if not items:
        ap.error("没有要查的文件（--set ours|orig|pack:<dir> 或给文件）")
    out = a.out or os.path.join(ALGO, f"clean_check_{a.set.replace(':', '_').replace('/', '_') or 'files'}.json")
    tmpd = out + ".parts"; os.makedirs(tmpd, exist_ok=True)
    # 大件先跑、单独跑（面数 > 30 万独占）；小件并行 --jobs
    def nfaces(p):
        try:
            return os.path.getsize(p) // 50
        except OSError:
            return 0
    items.sort(key=lambda it: -nfaces(it[0]))
    running, results, t0 = [], {}, time.time()
    def reap(block=False):
        for pr in list(running):
            p, it, tmp, ts = pr
            if p.poll() is None and not block: continue
            p.wait()
            running.remove(pr)
            try:
                results[it[1]] = json.load(open(tmp, encoding="utf-8"))
            except Exception as e:                                     # noqa: BLE001
                results[it[1]] = dict(label=it[1], file=os.path.relpath(it[0], ROOT), status="ERROR", error=f"子进程失败 rc={p.returncode}: {e!r}")
            r = results[it[1]]
            fs = {k: (v.get("status"), len(v.get("findings") or []) + len(v.get("open_ring_findings") or []))
                  for k, v in (r.get("checks") or {}).items()}                  # holes：孔 + 开口环都算
            print(f"[{time.time()-t0:6.0f}s] {it[1]:40s} {r.get('status')} {fs} {r.get('seconds')}s rss {r.get('peak_rss_gb')} GB", flush=True)
    for it in items:
        heavy = nfaces(it[0]) > 300_000
        while running and (heavy or len(running) >= a.jobs or any(nfaces(x[1][0]) > 300_000 for x in running) or mem_free_gb() < 4.0):
            reap(); time.sleep(0.5)
        tmp = os.path.join(tmpd, it[1] + ".json")
        cmd = [sys.executable, "-B", os.path.abspath(__file__), "--child", it[0], "--label", it[1], "--placed", it[2] or "",
               "--tmp", tmp, "--checks", ",".join(checks), "--profile", a.profile] + sum((["--param", kv] for kv in a.param), [])
        running.append((subprocess.Popen(["nice", "-n", "10"] + cmd, cwd=ROOT), it, tmp, time.time()))
    while running:
        reap(); time.sleep(0.5)
    res = dict(tool="tools/cad/clean_check.py", generated=time.strftime("%Y-%m-%d %H:%M:%S"), set=a.set or "files", checks=checks,
               params=P, files=[results[it[1]] for it in items])
    try:
        sys.path.insert(0, HERE)
        import clean_check_holes as CH
        res["holes_params"] = getattr(CH, "P", None) or getattr(CH, "PARAMS", None)
    except Exception:                                                  # noqa: BLE001
        pass
    json.dump(res, open(out, "w"), ensure_ascii=False, indent=1)
    print("wrote", out)


if __name__ == "__main__":
    main()
