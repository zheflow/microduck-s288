"""hr39c 头壳等比放大（2026-09-23 用户：「你按照等比放大+10来吧，开始设计」）。

口径（用户原话 / 主 agent 定）：
  * 头宽硬上限每侧 +10（「肉眼能接受的极限」）→ s = (91.8 + 20) / 91.8 = 1.21786，绕 P0 = (26, 0, 218.2)（头底中心；头底 / 脖子接口不动）。
  * **等比**（保持原版头部弧度，不插直段、不鼓包）；**壁厚保持原厚**（不跟着 ×s），全部件 PETG、头壳 2 圈外墙 + 30% 填充。
  * 放大对象 = 原版四个头壳网格（bottom_head_shell → H03、top_head_shell → H05、face_part → H04、jaw → J01）；
    内部件（那摞件、H01/H02、舵机、轴承、Radxa）一概不放大。凡是和不放大的件配合的特征由各 builder 在**真实位置**重做（head.py / head_top.py / jaw.py）。

做法（位移场，拓扑不变 → 放大后仍是 watertight 单实体）：
  1. 外表皮：对每个三角面从面心沿法向 + 6 个偏 30° 的方向打 7 根射线，≥4 根逃出（不碰任何头壳）= 外表皮面。
     H03/H05/H04 三件只对这三件互相测（这样 H03 鸟嘴下面对着 J01 的那面算外表皮）；J01 对四件测（J01 上面贴着 H03 的那面算内面）。
  2. 外表皮上的点 F 映射到 P0 + s(F − P0)（= 原版等比放大，外形逐点精确）。
  3. 其余顶点（内表面、内部筋）：F(v) = v 到外表皮的「加权最近点」（KD 树，距离 d ≤ d_min + 1.2 的采样点按 exp(−((d−d_min)/0.6)²) 加权平均，
     让最近点在两块外表皮之间切换时位移连续），位移 D(v) = (s − 1)(F(v) − P0)。
     → 薄壁：内表面点与它的外表皮垂足同位移，壁厚逐点保持；内部筋/凸台跟它脚下那块壁一起平移（水平方向跟着拉开 s 倍，高度不变）。
  4. 挂在壳上的东西（电子件、卡座、脸板立柱）用 move(p) = p + D(p) 跟着壳走（刚体平移，不放大）。

缓存：结果写 duckstructure/data/head_scale_cache.npz（含原版 STL 的 sha256 + s + P0 + 版本号，不一致就重算，约 4 分钟）。
原模型来自 Pollen Robotics Microduck，沿用原资产的 CC BY-SA-NC 许可。"""
import os, math, json, hashlib, time
import numpy as np, trimesh

P0 = np.array([26.0, 0.0, 218.2])
HEAD_W0 = 91.8                     # 原版头宽（bottom/top shell y ±45.88 → 91.8 口径，hr39b 同）
WIDEN_PER_SIDE = 10.0              # 用户 2026-09-23 定：每侧 +10（硬上限）
SCALE = (HEAD_W0 + 2.0 * WIDEN_PER_SIDE) / HEAD_W0      # 1.217865
PARTS = ("bottom_head_shell", "top_head_shell", "face_part", "jaw")
SHELL_GROUP = ("bottom_head_shell", "top_head_shell", "face_part")
RAY_TILT_DEG, RAY_N, RAY_ESCAPE_MIN = 30.0, 6, 4
FOOT_BAND, FOOT_SIGMA, SAMPLE_PITCH = 1.2, 0.6, 0.3
VERSION = "hr39c-2"         # hr39c-2：最近外表皮只在**本件**的外表皮里找（hr39c-1 四件共用一棵树 → J01 臂内面离壳外表面 0.3，被壳带着走，臂变厚 0.6）
_HERE = os.path.dirname(os.path.abspath(__file__))
CACHE = os.path.join(_HERE, "data", "head_scale_cache.npz")


def S(p):
    """外表皮点的等比映射（绕 P0 放大 SCALE）。"""
    return P0 + SCALE * (np.asarray(p, float) - P0)


def _orig_meshes():
    from .lib import orig
    return {n: orig("jaw_soft", n) for n in PARTS}


def _sha_orig():
    from . import kin as K
    h = hashlib.sha256()
    for n in PARTS:
        h.update(open(os.path.join(K.ASSETS, n + ".stl"), "rb").read())
    h.update(f"{SCALE:.9f} {P0.tolist()} {VERSION} {RAY_TILT_DEG} {RAY_N} {RAY_ESCAPE_MIN} {FOOT_BAND} {FOOT_SIGMA} {SAMPLE_PITCH}".encode())
    return h.hexdigest()


def _ray_dirs(n):
    """每个面：法向 + RAY_N 根绕法向偏 RAY_TILT_DEG 的方向。返回 (F, 1+RAY_N, 3)。"""
    a = np.where(np.abs(n[:, 2:3]) < 0.9, np.array([[0, 0, 1.0]]), np.array([[1.0, 0, 0]]))
    u = np.cross(n, a); u /= np.linalg.norm(u, axis=1, keepdims=True)
    w = np.cross(n, u)
    t = math.radians(RAY_TILT_DEG)
    out = [n]
    for k in range(RAY_N):
        ph = 2 * math.pi * k / RAY_N
        d = math.cos(t) * n + math.sin(t) * (math.cos(ph) * u + math.sin(ph) * w)
        out.append(d / np.linalg.norm(d, axis=1, keepdims=True))
    return np.stack(out, 1)


def exterior_masks(M, log=print):
    """每件一个 bool[面数]：True = 外表皮面。"""
    shell_u = trimesh.util.concatenate([M[n] for n in SHELL_GROUP])
    all_u = trimesh.util.concatenate([M[n] for n in PARTS])
    out = {}
    for n in PARTS:
        m = M[n]; tgt = shell_u if n in SHELL_GROUP else all_u
        c = m.triangles_center; nn = m.face_normals
        D = _ray_dirs(nn)
        esc = np.zeros(len(c), int)
        t0 = time.time()
        for k in range(D.shape[1]):
            d = D[:, k, :]
            hit = np.zeros(len(c), bool)
            for s0 in range(0, len(c), 20000):
                hit[s0:s0 + 20000] = tgt.ray.intersects_any(c[s0:s0 + 20000] + 0.01 * d[s0:s0 + 20000], d[s0:s0 + 20000])
            esc += ~hit
        out[n] = esc >= RAY_ESCAPE_MIN
        log(f"    [head_scale] {n}: 外表皮面 {int(out[n].sum())}/{len(c)}（{time.time()-t0:.0f}s）")
    return out


def _samples(M, ext):
    """每件自己的外表皮采样点（dict）"""
    out = {}
    for n in PARTS:
        m = M[n]; sub = m.submesh([np.nonzero(ext[n])[0]], append=True)
        k = int(sub.area / SAMPLE_PITCH ** 2) + 100
        p, _ = trimesh.sample.sample_surface_even(sub, k, seed=7)
        out[n] = np.vstack([np.asarray(p), np.asarray(m.vertices[np.unique(m.faces[ext[n]])])])
    return out


def foot_points(Q, tree, samples, chunk=4000):
    """Q (n,3) → 加权最近外表皮点 F（见文件头第 3 条）；另返回 d_min。"""
    F = np.empty_like(Q); dmin = np.empty(len(Q))
    d0, _ = tree.query(Q, k=1)
    for s0 in range(0, len(Q), chunk):
        q = Q[s0:s0 + chunk]; dm = d0[s0:s0 + chunk]
        idx = tree.query_ball_point(q, dm + FOOT_BAND)
        for i, (row, di) in enumerate(zip(idx, dm)):
            P = samples[row]; d = np.linalg.norm(P - q[i], axis=1)
            w = np.exp(-(((d - di) / FOOT_SIGMA) ** 2))
            F[s0 + i] = (w[:, None] * P).sum(0) / w.sum()
        dmin[s0:s0 + chunk] = dm
    return F, dmin


_STATE = {}


EXT_CACHE = os.path.join(_HERE, "data", "head_scale_ext.npz")


def _ext_cached(M, log=print):
    """外表皮分类很慢（~8 分钟射线），单独缓存（只依赖原版 STL + 射线参数）"""
    from . import kin as K
    h = hashlib.sha256()
    for n in PARTS: h.update(open(os.path.join(K.ASSETS, n + ".stl"), "rb").read())
    h.update(f"{RAY_TILT_DEG} {RAY_N} {RAY_ESCAPE_MIN} {SHELL_GROUP}".encode()); key = h.hexdigest()
    if os.path.exists(EXT_CACHE):
        Z = np.load(EXT_CACHE)
        if str(Z["key"]) == key: return {n: Z[n] for n in PARTS}
    ext = exterior_masks(M, log)
    np.savez_compressed(EXT_CACHE, key=key, **ext)
    return ext


def _compute(log=print):
    t0 = time.time()
    M = _orig_meshes()
    ext = _ext_cached(M, log)
    samples = _samples(M, ext)
    from scipy.spatial import cKDTree
    out = {}
    for n in PARTS:
        tree = cKDTree(samples[n])
        m = M[n]; V = np.asarray(m.vertices, float)
        on_ext = np.zeros(len(V), bool); on_ext[np.unique(m.faces[ext[n]])] = True
        F = V.copy(); dmin = np.zeros(len(V))
        if (~on_ext).any():
            F[~on_ext], dmin[~on_ext] = foot_points(V[~on_ext], tree, samples[n])
        Vn = V + (SCALE - 1.0) * (F - P0)
        mm = trimesh.Trimesh(vertices=Vn, faces=m.faces, process=False)
        flip = int((np.einsum("ij,ij->i", mm.face_normals, m.face_normals) < 0.0).sum())
        out[n] = dict(v=Vn, f=np.asarray(m.faces), ext=ext[n], dmin=dmin, flip=flip)
        log(f"    [head_scale] {n}: 体积 {m.volume/1000:.2f} → {mm.volume/1000:.2f} cm³（×{mm.volume/m.volume:.3f}；s²={SCALE**2:.3f} s³={SCALE**3:.3f}），翻面 {flip}，watertight {mm.is_watertight}")
    np.savez_compressed(CACHE, sha=_sha_orig(), **{f"samples__{n}": samples[n].astype(np.float32) for n in PARTS},
                        **{f"{n}__{k}": v for n, d in out.items() for k, v in d.items()})
    log(f"    [head_scale] 缓存写入 {CACHE}（{time.time()-t0:.0f}s）")
    return out, samples


def _load(log=print):
    if "parts" in _STATE: return _STATE
    if os.path.exists(CACHE):
        Z = np.load(CACHE)
        if str(Z["sha"]) == _sha_orig():
            parts = {n: {k: Z[f"{n}__{k}"] for k in ("v", "f", "ext", "dmin", "flip")} for n in PARTS}
            _STATE.update(parts=parts, samples={n: Z[f"samples__{n}"].astype(np.float64) for n in PARTS}); return _STATE
        log("    [head_scale] 缓存与原版 STL / 参数不一致 → 重算")
    parts, samples = _compute(log)
    _STATE.update(parts=parts, samples=samples)
    return _STATE


def scaled_orig(name):
    """原版头壳网格等比放大（壁厚保持）后的世界系零位网格（新对象，可随便改）。name ∈ PARTS。"""
    st = _load()
    d = st["parts"][name]
    return trimesh.Trimesh(vertices=np.array(d["v"], float), faces=np.array(d["f"]), process=False)


def exterior_mask(name):
    return np.array(_load()["parts"][name]["ext"], bool)


_TREE = {}


def D(p, part):
    """点 p（(3,) 或 (n,3)，原版世界系）处、挂在件 part 上的东西的位移：(s−1)(F(p) − P0)，F = part 自己外表皮上的加权最近点。
    挂在壳上的东西用 move(p, part) 跟壳走（刚体平移，不放大）。"""
    st = _load()
    if part not in _TREE:
        from scipy.spatial import cKDTree
        _TREE[part] = cKDTree(st["samples"][part])
    q = np.atleast_2d(np.asarray(p, float))
    F, _ = foot_points(q, _TREE[part], st["samples"][part])
    out = (SCALE - 1.0) * (F - P0)
    return out[0] if np.ndim(p) == 1 else out


def move(p, part):
    return np.asarray(p, float) + D(p, part)


def summary():
    """报告用：每件原版 / 放大后体积、翻面数。"""
    from .lib import orig
    st = _load(); out = {}
    for n in PARTS:
        d = st["parts"][n]
        m0 = orig("jaw_soft", n); m1 = trimesh.Trimesh(vertices=d["v"], faces=d["f"], process=False)
        out[n] = dict(vol0_mm3=round(float(m0.volume), 1), vol1_mm3=round(float(m1.volume), 1), ratio=round(float(m1.volume / m0.volume), 4),
                      flipped_faces=int(d["flip"]), exterior_faces=int(np.sum(d["ext"])), faces=int(len(d["f"])))
    return out


if __name__ == "__main__":
    import sys
    if "--force" in sys.argv and os.path.exists(CACHE): os.remove(CACHE)
    print(json.dumps(summary(), ensure_ascii=False, indent=1))
