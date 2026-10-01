#!/usr/bin/env python
"""打印文件体检（2026-09-28，起因：hr48 L04_R 补打件法向整体反了，build / Gate 都没查到——Gate L0 只看 cad/duck_s288 源件，
镜像件是出包脚本最后才生成的）。直接读 STL 原始三角面，逐文件查：

  反法向     有向体积 < 0（整件内外颠倒；切片软件里像缺面 / 锯齿）
  法向不一致  同一条边被两个面按同一方向走（局部翻面）
  开边 / 非流形边 / 退化面 / 碎体（<1 mm³ 的独立小块）
  存的法向与顶点顺序相反的面数（多数切片软件会重算，但说明出包时翻过面）
  版本 / 手性  打印包文件 ↔ 当前 cad/duck_s288/placed 的同名件：面心一一对应做最小二乘，残差 ≈0 = 同一版几何；
             变换行列式 −1 = 手性错（R 件没镜像或镜像了两次）；面数不同 = 不是同一版几何（旧包 / 已改件）
  ×2 件     一个文件打两件（L10 / L13）时，查它的镜像是否与自己全等（不全等 = 左右件不能用同一个文件）

只读。用法：
  ./.venv/bin/python -B tools/cad/print_file_check.py                 # 默认：打印包/*/stl + 所有补打 zip + cad/duck_s288/*.stl
  ./.venv/bin/python -B tools/cad/print_file_check.py --selftest      # 反例：必须全部报出、正例不许报
"""
import os, sys, glob, json, struct, zipfile, io, argparse, re, tempfile
import numpy as np

ROOT = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", ".."))
PACK = os.path.join(ROOT, "docs", "design_2026-09-17_bearing_rebuild", "打印包")


def read_stl(data):
    """→ (三角形 n×3×3 float64, 存的法向 n×3 或 None)。二进制判据：84 + 50n == 文件长。"""
    if len(data) >= 84:
        n = struct.unpack("<I", data[80:84])[0]
        if 84 + 50 * n == len(data):
            rec = np.frombuffer(data, dtype=np.dtype([("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")]), count=n, offset=84)
            return rec["v"].astype(np.float64), rec["n"].astype(np.float64)
    import trimesh
    m = trimesh.load(io.BytesIO(data), file_type="stl", force="mesh", process=False)
    return np.asarray(m.triangles, float), None


def audit_tris(tri, nrm):
    V = tri.reshape(-1, 3)
    uv, inv = np.unique(V, axis=0, return_inverse=True)
    F = inv.reshape(-1, 3)
    e1, e2 = tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0]
    cr = np.cross(e1, e2); area = 0.5 * np.linalg.norm(cr, axis=1)
    collapsed = (F[:, 0] == F[:, 1]) | (F[:, 1] == F[:, 2]) | (F[:, 0] == F[:, 2])      # 顶点重合 → 真坏面（拓扑断开）
    zero_area = (~collapsed) & (area < 1e-10)                                                # 三点共线的零面积细长面：拓扑完好、切片无影响，只计数
    deg = collapsed
    Fg = F[~collapsed]
    nv = len(uv)
    E = np.vstack([Fg[:, [0, 1]], Fg[:, [1, 2]], Fg[:, [2, 0]]]).astype(np.int64)
    und = np.sort(E, axis=1); ukey = und[:, 0] * nv + und[:, 1]
    _, ucnt = np.unique(ukey, return_counts=True)
    dkey = E[:, 0] * nv + E[:, 1]
    _, dcnt = np.unique(dkey, return_counts=True)
    vol = float(np.einsum("ij,ij->i", tri[:, 0], np.cross(tri[:, 1], tri[:, 2])).sum() / 6.0)
    flipped_stored = 0
    if nrm is not None:
        ok = (np.linalg.norm(nrm, axis=1) > 0.5) & (area > 1e-4)          # 近零面积面的法向方向数值上不稳，不算
        flipped_stored = int(((np.einsum("ij,ij->i", cr, nrm) < 0) & ok).sum())
    # 碎体：按共享顶点连通
    from scipy.sparse import coo_matrix
    from scipy.sparse.csgraph import connected_components
    g = coo_matrix((np.ones(len(E)), (E[:, 0], E[:, 1])), shape=(nv, nv))
    ncomp, lab = connected_components(g, directed=False)
    flab = lab[Fg[:, 0]]
    tv = np.einsum("ij,ij->i", tri[~collapsed][:, 0], np.cross(tri[~collapsed][:, 1], tri[~collapsed][:, 2])) / 6.0
    comp_vol = np.bincount(flab, weights=tv, minlength=ncomp)
    used = np.unique(flab)
    bodies = [float(comp_vol[c]) for c in used]
    # 近重合顶点（<1e-4，却没被合并）：float32 截断或出包变换造成，常是非流形边的来源
    from scipy.spatial import cKDTree
    close_pairs = len(cKDTree(uv).query_pairs(1e-4)) if nv < 400000 else -1
    return dict(faces=int(len(tri)), degenerate=int(deg.sum()), zero_area=int(zero_area.sum()), volume=round(vol, 2), open_edges=int((ucnt == 1).sum()),
                nonmanifold_edges=int((ucnt > 2).sum()), winding_conflicts=int((dcnt > 1).sum()), stored_normal_flipped=flipped_stored,
                bodies=len(bodies), tiny_bodies=int(sum(1 for b in bodies if abs(b) < 1.0)), near_dup_vertices=close_pairs,
                area=round(float(area.sum()), 2))


def fit_affine(src, dst):
    A = np.c_[src, np.ones(len(src))]
    M, *_ = np.linalg.lstsq(A, dst, rcond=None)
    R = M[:3].T
    return float(np.abs(A @ M - dst).max()), float(np.linalg.det(R)), float(np.abs(R @ R.T - np.eye(3)).max())


def _fname_map():
    sys.path.insert(0, ROOT)
    from duckstructure.build_fast import FNAME
    return {v[:3]: k for k, v in FNAME.items()}


def placed_for(fname, pmap):
    pid = os.path.basename(fname)[:3]
    nm = pmap.get(pid)
    if nm is None: return None
    side_r = re.search(r"_R(_|\.)", os.path.basename(fname)) is not None
    p = os.path.join(ROOT, "cad", "duck_s288", "placed", nm + ("_R" if (side_r and not nm.endswith("_R")) else "") + ".stl")   # T03 的 placed 名已是 shell_R（复审 r1 MINOR：原来拼成 shell_R_R 漏查）
    return p if os.path.exists(p) else None


def mirror_self_congruent(m, tol=0.1):
    """m 的镜像能否用正交旋转 + 平移与 m 重合（左右件能否共用一个文件）。
    主轴对齐（8 种符号里保向的 4 种；主惯量简并时再绕简并轴每 15° 起一个初值）→ 点到面 ICP 8 轮 → 取 99 分位距离最小的。"""
    import trimesh
    mir = m.copy(); mir.apply_transform(np.diag([1, -1, 1, 1.0]))
    if mir.volume < 0: mir.invert()
    q0 = trimesh.sample.sample_surface(mir, 1500, seed=3)[0]
    def frame(mesh):
        # 精确质心 + 惯量主轴（不用随机采样：采样质心有 ~0.1 mm 噪声，回转体会被误判不全等）
        c = np.asarray(mesh.center_mass, float); w, v = np.linalg.eigh(np.asarray(mesh.moment_inertia, float))
        return c, v, w
    c0, V0, w0 = frame(m); c1, V1, _ = frame(mir)
    inits = []
    for sx in (1, -1):
        for sy in (1, -1):
            for sz in (1, -1):
                R = V0 @ np.diag([sx, sy, sz]) @ V1.T
                if np.linalg.det(R) > 0: inits.append(R)
    degen = [(i, j) for i, j in ((0, 1), (1, 2)) if abs(w0[i] - w0[j]) / max(abs(w0[j]), 1e-9) < 0.03]
    if degen:
        k = ({0, 1, 2} - set(degen[0])).pop(); ax = V0[:, k]
        base = list(inits)
        for R in base:
            for ang in np.radians(np.arange(15, 360, 15)):
                K = np.array([[0, -ax[2], ax[1]], [ax[2], 0, -ax[0]], [-ax[1], ax[0], 0]])
                Rz = np.eye(3) + np.sin(ang) * K + (1 - np.cos(ang)) * (K @ K)
                inits.append(Rz @ R)
    best = 9e9
    if degen:
        # 回转体（套筒 / 圆盘）：绕轴的转角 ICP 感觉不到（孔太小），改成绕轴 1° 粗扫 + 0.05° 细扫，直接取最小 p99
        k = ({0, 1, 2} - set(degen[0])).pop(); ax = V0[:, k]
        K = np.array([[0, -ax[2], ax[1]], [ax[2], 0, -ax[0]], [-ax[1], ax[0], 0]])
        rot = lambda ang: np.eye(3) + np.sin(ang) * K + (1 - np.cos(ang)) * (K @ K)
        qs = q0[::3]
        def score(R, ang, pts):
            q = (pts - c1) @ (rot(ang) @ R).T + c0
            return float(np.percentile(trimesh.proximity.closest_point(m, q)[1], 99))
        for R in [R for R in inits[:8]]:
            angs = np.radians(np.arange(0, 360, 1.0)); sc = [score(R, a_, qs) for a_ in angs]
            a0 = angs[int(np.argmin(sc))]
            fine = a0 + np.radians(np.arange(-1.0, 1.0001, 0.05)); sc2 = [score(R, a_, q0) for a_ in fine]
            best = min(best, float(min(sc2)))
            if best < tol * 0.5: break
        return best < tol, round(best, 3)
    for R in inits:
        q = (q0 - c1) @ R.T + c0
        for _ in range(15):
            cp, d, _ = trimesh.proximity.closest_point(m, q)
            mu_q, mu_c = q.mean(0), cp.mean(0); H = (q - mu_q).T @ (cp - mu_c); U, S_, Vt = np.linalg.svd(H)
            D = np.diag([1, 1, np.sign(np.linalg.det(Vt.T @ U.T))]); Rk = Vt.T @ D @ U.T
            q = (q - mu_q) @ Rk.T + mu_c
        d = trimesh.proximity.closest_point(m, q)[1]
        best = min(best, float(np.percentile(d, 99)))
        if best < tol * 0.5: break
    return best < tol, round(best, 3)


def check_file(path, data, pmap):
    tri, nrm = read_stl(data)
    r = audit_tris(tri, nrm)
    r["file"] = path
    issues = []
    if r["volume"] < 0: issues.append("反法向（体积为负）")
    if r["winding_conflicts"]: issues.append(f"法向不一致 {r['winding_conflicts']} 条边")
    if r["open_edges"]: issues.append(f"开边 {r['open_edges']}")
    if r["nonmanifold_edges"]: issues.append(f"非流形边 {r['nonmanifold_edges']}")
    if r["degenerate"]: issues.append(f"顶点重合的坏面 {r['degenerate']}")
    if r["tiny_bodies"]: issues.append(f"碎体 {r['tiny_bodies']}")
    if r["bodies"] > 1 and not r["tiny_bodies"]: issues.append(f"多实体 {r['bodies']}")
    if r["stored_normal_flipped"]: issues.append(f"存的法向反 {r['stored_normal_flipped']} 面")
    pp = placed_for(path, pmap) if pmap else None
    if pp:
        import trimesh
        ptri, _ = read_stl(open(pp, "rb").read())
        if len(ptri) == len(tri):
            res, det, orth = fit_affine(tri.mean(1), ptri.mean(1))
            r["vs_placed"] = dict(placed=os.path.relpath(pp, ROOT), resid=round(res, 5), det=round(det, 4), orth=round(orth, 5))
            if res > 0.01 or orth > 1e-3: issues.append(f"与当前设计几何不同（面心拟合残差 {res:.3f} mm）")
            elif det < 0: issues.append("手性错（与当前设计互为镜像：R 件没镜像 / 镜像了两次）")
        else:
            pv = abs(audit_tris(ptri, None)["volume"])
            r["vs_placed"] = dict(placed=os.path.relpath(pp, ROOT), faces_file=len(tri), faces_placed=len(ptri), vol_file=abs(r["volume"]), vol_placed=round(pv, 2))
            issues.append(f"与当前设计不是同一版（面数 {len(tri)} vs {len(ptri)}，体积 {abs(r['volume']):.1f} vs {pv:.1f}）")
    if re.search(r"_x2\.stl$", path):
        import trimesh
        m = trimesh.Trimesh(*_merge(tri))
        ok, d = mirror_self_congruent(m)
        r["x2_mirror_congruent"] = dict(ok=ok, p99_mm=d)
        if not ok: issues.append(f"一个文件打两件，但它的镜像与自己不全等（p99 偏差 {d} mm）→ 左右件不能共用")
    r["issues"] = issues
    return r


def _merge(tri):
    V = tri.reshape(-1, 3); uv, inv = np.unique(V, axis=0, return_inverse=True)
    return uv, inv.reshape(-1, 3)


def targets():
    out = []
    for d in sorted(glob.glob(os.path.join(PACK, "*", "stl"))):
        for f in sorted(glob.glob(os.path.join(d, "*.stl"))):
            out.append((os.path.relpath(f, ROOT), open(f, "rb").read()))
    for z in sorted(glob.glob(os.path.join(PACK, "*.zip")) + glob.glob(os.path.join(PACK, "*", "*.zip"))):
        try:
            with zipfile.ZipFile(z) as zz:
                for n in zz.namelist():
                    if n.lower().endswith(".stl"):
                        out.append((os.path.relpath(z, ROOT) + "::" + n, zz.read(n)))
        except Exception as e:
            out.append((os.path.relpath(z, ROOT), b""))
    for f in sorted(glob.glob(os.path.join(ROOT, "cad", "duck_s288", "*.stl"))):
        if "assembly_preview" in f: continue
        out.append((os.path.relpath(f, ROOT), open(f, "rb").read()))
    return out


def selftest():
    import trimesh
    bad = 0
    def run(name, mesh, expect_issue, path="selftest.stl", pmap=None, placed_mesh=None):
        nonlocal bad
        data = trimesh.exchange.stl.export_stl(mesh)
        r = check_file(path, data, None)
        if placed_mesh is not None:   # 手性 / 版本：直接比
            res, det, orth = fit_affine(np.asarray(mesh.triangles).mean(1), np.asarray(placed_mesh.triangles).mean(1))
            if det < 0: r["issues"].append("手性错")
        got = bool(r["issues"])
        ok = got == expect_issue
        bad += (not ok)
        print(f"  {'✅' if ok else '❌'} {name}: {'报出 ' + '；'.join(r['issues']) if got else '无问题'}（期望{'报出' if expect_issue else '无问题'}）")
    box = trimesh.creation.box((20, 10, 5))
    run("正常盒", box, False)
    run("正常圆柱 128 边", trimesh.creation.cylinder(radius=6, height=10, sections=128), False)
    inv = box.copy(); inv.invert(); run("整件反法向", inv, True)
    half = box.copy(); f = half.faces.copy(); f[:4] = f[:4, ::-1]; half = trimesh.Trimesh(half.vertices, f, process=False); run("局部翻面", half, True)
    op = trimesh.Trimesh(box.vertices, box.faces[2:], process=False); run("缺两面（开边）", op, True)
    b2 = trimesh.creation.box((10, 10, 10)); b3 = b2.copy(); b3.apply_translation((10, 10, 0))
    run("两盒共一条棱（非流形边）", trimesh.util.concatenate([b2, b3]), True)
    ch = trimesh.boolean.difference([trimesh.creation.box((30, 20, 6)), trimesh.creation.cylinder(radius=3, height=10, transform=trimesh.transformations.translation_matrix((8, 4, 0)))], engine="manifold")
    mir = ch.copy(); mir.apply_transform(np.diag([1, -1, 1, 1.0]))
    if mir.volume < 0: mir.invert()
    run("手性：R 件＝镜像（对）", mir, False, placed_mesh=mir)
    run("手性：R 件没镜像（错）", ch, True, placed_mesh=mir)
    # 当年 hr48 的 bug：镜像后多翻一次
    bug = ch.copy(); bug.apply_transform(np.diag([1, -1, 1, 1.0])); bug.invert(); run("hr48 旧 bug：镜像+多翻一次", bug, True)
    ok_sym, d = mirror_self_congruent(trimesh.creation.cylinder(radius=6, height=4, sections=64))
    print(f"  {'✅' if ok_sym else '❌'} ×2 对称件（圆盘）判全等：{ok_sym}（p99 {d}）"); bad += (not ok_sym)
    ok_ch, d = mirror_self_congruent(ch)
    print(f"  {'✅' if ok_ch else '❌'} ×2 件（偏心孔板，关于 y 对称面其实对称）判全等：{ok_ch}（p99 {d}）"); bad += (not ok_ch)
    chir = trimesh.boolean.union([trimesh.creation.box((20, 4, 4)), trimesh.creation.box((4, 12, 4), transform=trimesh.transformations.translation_matrix((8, 4, 0))),
                                  trimesh.creation.box((4, 4, 10), transform=trimesh.transformations.translation_matrix((8, 8, 3)))], engine="manifold")
    ok_c, d = mirror_self_congruent(chir)
    print(f"  {'✅' if not ok_c else '❌'} ×2 件（三向 L 形，真手性）判不全等：{not ok_c}（p99 {d}）"); bad += ok_c
    print("自测：" + ("全部通过" if bad == 0 else f"{bad} 项不对"))
    return bad


if __name__ == "__main__":
    ap = argparse.ArgumentParser(); ap.add_argument("--selftest", action="store_true"); ap.add_argument("--out", default="")
    a = ap.parse_args()
    if a.selftest:
        sys.exit(1 if selftest() else 0)
    pmap = _fname_map()
    rows = []
    import hashlib
    seen = {}
    for path, data in targets():
        h = hashlib.sha256(data).hexdigest() if data else None
        if h and h in seen:
            r = dict(seen[h]); r["file"] = path; r["same_as"] = seen[h]["file"]; rows.append(r); continue
        if not data:
            rows.append(dict(file=path, issues=["读不出（zip 坏）"])); continue
        try:
            r = check_file(path, data, pmap); rows.append(r)
            if h: seen[h] = r
        except Exception as e:
            rows.append(dict(file=path, issues=[f"检查失败 {e!r}"]))
    bad = [r for r in rows if r.get("issues")]
    for r in rows:
        print(("⚠ " if r.get("issues") else "  ") + f"{r['file']}: " + ("；".join(r["issues"]) if r.get("issues") else "OK"))
    print(f"\n共 {len(rows)} 个文件，有问题 {len(bad)} 个")
    if a.out:
        json.dump(rows, open(a.out, "w"), ensure_ascii=False, indent=1, default=str)
