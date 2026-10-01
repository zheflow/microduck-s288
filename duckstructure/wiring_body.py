"""过颈 / 躯干 / 腿线束实体（hr44，2026-09-25）—— 接口同 wiring_head：build() / export() / 检查。

用户（hr44 简报）：「线这一轮就建模完 —— 过颈、躯干、腿的线全部像头内那样建成实体并检查」「相近的线扎成束走一起」「电子件与线都要固定牢」。
范围 = 头外的线：过颈两支（HB09-R：IMU 4 + PH 延长；HB09-L：12 V 对线 + 头链 7→6 跳 + S2→颈俯仰 5 跳）、躯干（电池顶 XT30 → 口袋、IMU 段 1、
口袋 S2 → 侧腔 S3/S4/S5）、两条腿链（每腿：S4/S5 → 髋偏航单挂 + 髋横滚 → 髋俯仰 → 膝 → 踝，舵机原配 PH2.0 3P 扁线 200）。头内线在 wiring_head。

模型（所有坐标 = 世界系零位，与 wiring_head / placed 同系，mm）：
  一条线路 = 站点序列 [(body, 点, 标签)]。同一 body 的连续站点是**跟件走的刚性段**（线夹 / 扎带把线按在件上）；相邻两站 body 不同 = **跨关节的柔性段**：
    · hermite：两端按固定段切向出线，三次 Hermite，切向长度 m 二分求解使曲线长 = 线长 L（L = 零位长 + 余长）；拉不到（最短也 > L）= 张紧（不通过）。
    · wrap：绕偏航轴的螺旋（头偏航 ±170°）：两端化到轴的柱坐标，转角随 head_yaw 连续展开，半径外鼓 b·sin(πt) 二分求解使长 = L；b<0 解不出 = 张紧。
  余长 = 各柔性段在全区间里「最短可行长」的最大值 − 零位长（模型算，assumed 余量 +3 mm）。
检查（sweep_check）：姿态 = l6_motion 同一套采样（单轴 2.5° 独立网格 + 父子相邻关节两两 5° 网格，扫描域 = 上游 MJCF 区间，区内/区外按 frozen.yaml 目标区间分），
  每个姿态把线（束外包管）表面抽样点反变换到各 body 零位系，查 body 占用体素（0.5 mm，placed 全件，射线奇偶填充）→ 命中 = 线∩件；另查张紧、弯半径。
数值来源：socket 位 = lib.sfw @ KO01 插座局部 (−13, ±7.75, −9.3)；线材截面 = wiring_head.CABLE（measured / assumed 照抄）；束径 = 最密堆外接圆 + 缠绕管壁（assumed）。
用法：
    from duckstructure import wiring_body as WB
    meshes, meta = WB.build()            # wire__ / tie__ / conn__ 实体（世界系零位），meta 每根线长度 / 截面 / 弯半径 / 买的长度
    WB.export(outdir)
    res = WB.sweep_check()               # 全区间扫掠（l6 同采样）；WB.zero_check() 零位线∩件（manifold 精确布尔）
"""
import os, math, json, hashlib
import numpy as np
import trimesh
from . import wiring_head as WH

# ════════════════════════════════════════════════════════════════════════════════════════════
# 0. 运动学（与 tools/gate/layers/l6_motion._Scene.deltas 同口径：父连杆动、子连杆跟着动；关节轴 = 该 body 零位世界系 z 轴、过原点）
# ════════════════════════════════════════════════════════════════════════════════════════════
def _tree():
    from .lib import B, ORDER, TW
    return B, ORDER, TW


def joint_body():
    B, ORDER, _ = _tree()
    return {B[n]["joint"]["name"]: n for n in ORDER if B[n]["joint"]}


def body_T(pose_deg):
    """pose {关节名: 角度°} → {body: 4×4 世界位姿增量（相对零位）}"""
    from trimesh.transformations import rotation_matrix as rot
    B, ORDER, TW = _tree()
    out = {}
    for n in ORDER:
        par = B[n]["parent"]
        Tp = out[par] if par is not None else np.eye(4)
        j = B[n]["joint"]
        a = pose_deg.get(j["name"], 0.0) if j else 0.0
        if j and abs(a) > 1e-12:
            T = TW(n); out[n] = Tp @ rot(math.radians(float(a)), T[:3, 2], T[:3, 3])
        else:
            out[n] = Tp
    return out


def path_joints(b0, b1):
    """两个 body 之间运动树路径上的关节名（跨哪些关节）"""
    B, ORDER, _ = _tree()
    def up(n):
        out = [n]
        while B[n]["parent"] is not None:
            n = B[n]["parent"]; out.append(n)
        return out
    a, b = up(b0), up(b1)
    common = next(x for x in a if x in b)
    js = [B[n]["joint"]["name"] for n in a[:a.index(common)] + b[:b.index(common)] if B[n]["joint"]]
    return js


# ════════════════════════════════════════════════════════════════════════════════════════════
# 1. 线材（截面同 wiring_head.CABLE；束 = 最密堆外接圆 + 缠绕管壁）
# ════════════════════════════════════════════════════════════════════════════════════════════
PACK = {1: 1.0, 2: 2.0, 3: 2.155, 4: 2.414, 5: 2.701, 6: 3.0, 7: 3.0, 8: 3.305}
WRAP = dict(t=0.35, src="assumed：螺旋缠绕管（PE，内径按束径选 Ø4 / Ø6）壁厚 0.35；不缠时按扎带 2.5 mm 宽 × 1.0 厚每 30–40 mm 一道")
CABLE_B = {
    "servo_flat": dict(WH.CABLE["ph_servo_flat"]),          # 舵机原配 PH2.0 3P 扁线 4.5 × 1.5（measured）
    "ph_ext_3c": dict(kind="flat", w=3.3, t=1.1, min_bend=2.5, src="assumed：PH2.0 3P 公母延长线（cable_ph20_ext，商品页未给截面）同 wiring_head.ph_leg_3c"),
    "dupont": dict(WH.CABLE["dupont"]),
    "xt30_18awg_pair": dict(WH.CABLE["xt30_18awg_pair"]),
    "qt_dupont_4": dict(kind="round", od=1.0, min_bend=2.0, src="assumed：QT→杜邦公 150（cable_qt_dupont_m）28 AWG OD ≈1.0（components dims_assumed）"),
}


def cable_equiv_d(c):
    """外接圆直径（束堆算用）：圆线 = od；并线 = n·od 跑道外接；扁线 = 对角线"""
    c = CABLE_B[c] if isinstance(c, str) else c
    if c["kind"] == "round": return c["od"]
    if c["kind"] == "pair": return c["n"] * c["od"]
    return math.hypot(c["w"], c["t"])


def cable_area(c):
    c = CABLE_B[c] if isinstance(c, str) else c
    if c["kind"] == "round": return math.pi * c["od"] ** 2 / 4
    if c["kind"] == "pair": return c["n"] * math.pi * c["od"] ** 2 / 4
    return c["w"] * c["t"]


FILL = dict(v=0.6, src="assumed：混合截面线束（圆线 + 扁线）的填充率 0.6（线束工程常用 0.5–0.7）")


def bundle_od(members, wrap=True):
    """成员线 → 束外径（mm）：全是同径圆线 → 等径圆最密堆外接圆；混合 → 面积 / 填充率 FILL 折成圆；再加缠绕管壁 2×WRAP.t（assumed）"""
    cs = [CABLE_B[m] for m in members]
    if len(cs) == 1:
        d = cable_equiv_d(cs[0])
    elif all(c["kind"] == "round" for c in cs) and len({c["od"] for c in cs}) == 1 and len(cs) in PACK:
        d = PACK[len(cs)] * cs[0]["od"]
    else:
        d = max(math.sqrt(sum(cable_area(c) for c in cs) / FILL["v"] * 4 / math.pi), max(cable_equiv_d(c) for c in cs))
    return round(d + (2 * WRAP["t"] if wrap else 0.0), 2)


# ════════════════════════════════════════════════════════════════════════════════════════════
# 2. 柔性段模型
# ════════════════════════════════════════════════════════════════════════════════════════════
def _hermite(pA, tA, pB, tB, m, n=40):
    t = np.linspace(0, 1, n)[:, None]
    h00, h10, h01, h11 = 2 * t ** 3 - 3 * t ** 2 + 1, t ** 3 - 2 * t ** 2 + t, -2 * t ** 3 + 3 * t ** 2, t ** 3 - t ** 2
    return h00 * pA + h10 * m * tA + h01 * pB + h11 * m * tB


def _plen(P): return float(np.linalg.norm(np.diff(P, axis=0), axis=1).sum())


def flex_hermite(pA, tA, pB, tB, L):
    """两端切向固定的 Hermite，长度 = L（二分 m）。返回 (点列, 状态)。状态 ok / tension（最短也 > L）"""
    ch = float(np.linalg.norm(pB - pA))
    lo, hi = 0.05 * max(ch, 1.0), 6.0 * max(L, ch, 1.0)
    Pl = _hermite(pA, tA, pB, tB, lo)
    if _plen(Pl) > L + 1e-6:
        return Pl, "tension"
    for _ in range(40):
        mid = 0.5 * (lo + hi)
        if _plen(_hermite(pA, tA, pB, tB, mid)) < L: lo = mid
        else: hi = mid
    return _hermite(pA, tA, pB, tB, 0.5 * (lo + hi)), "ok"


def flex_wrap(pA, pB, axis_o, axis_d, dtheta, L, n=48, hpow=None, bulge=None):
    """绕轴螺旋：A → B，转角 dtheta（弧度，已按关节角连续展开），半径外鼓 b·sin(πt)，长 = L（二分 b ≥ 0）。
    高度剖面：默认 smoothstep；hpow = p 时 h = hA + (hB − hA)(1 − (1 − t)^p)（出 A 端即快速降到 B 的高度 —— 过颈侧环躲头横滚下压用，hr44）
    bulge（hr44 build #5，2026-09-26 加，旧行为不变）：None / "radial" = 原来的径向外鼓 b·sin(πt)；"sag" = 余量沿轴负向下垂 h −= s·sin(πt)、半径线性不外鼓
    （过颈研究 hr44neck v1：外鼓会把环甩到 |y| 53–56，下垂把外缘压在 |y| ≤ 25.7；研究稿里用 hpow −1 作记号，并回后改成 bulge="sag"）"""
    d = axis_d / np.linalg.norm(axis_d)
    e1 = pA - axis_o - np.dot(pA - axis_o, d) * d
    rA = float(np.linalg.norm(e1)); e1 = e1 / max(rA, 1e-9); e2 = np.cross(d, e1)
    wB = pB - axis_o - np.dot(pB - axis_o, d) * d
    rB = float(np.linalg.norm(wB))
    hA, hB = float(np.dot(pA - axis_o, d)), float(np.dot(pB - axis_o, d))
    t = np.linspace(0, 1, n)
    def curve(b):
        if bulge == "sag":   # hr44 build #5：下垂式余量（半径不外鼓，高度往轴负向垂 b·sin πt）
            r = rA + (rB - rA) * t
            th = dtheta * t
            h = hA + (hB - hA) * ((3 * t ** 2 - 2 * t ** 3) if not hpow else (1 - (1 - t) ** hpow)) - b * np.sin(np.pi * t)
            return axis_o + np.outer(r * np.cos(th), e1) + np.outer(r * np.sin(th), e2) + np.outer(h, d)
        r = rA + (rB - rA) * t + b * np.sin(np.pi * t)
        th = dtheta * t
        h = hA + (hB - hA) * ((3 * t ** 2 - 2 * t ** 3) if not hpow else (1 - (1 - t) ** hpow))
        return axis_o + np.outer(r * np.cos(th), e1) + np.outer(r * np.sin(th), e2) + np.outer(h, d)
    P0 = curve(0.0)
    if _plen(P0) > L + 1e-6:
        return P0, "tension"
    lo, hi = 0.0, 4.0 * L
    for _ in range(40):
        mid = 0.5 * (lo + hi)
        if _plen(curve(mid)) < L: lo = mid
        else: hi = mid
    return curve(0.5 * (lo + hi)), "ok"


# ════════════════════════════════════════════════════════════════════════════════════════════
# 3. 线路求值
# ════════════════════════════════════════════════════════════════════════════════════════════
def _runs(route):
    runs = []
    for (b, p, tag) in route["stations"]:
        if runs and runs[-1][0] == b: runs[-1][1].append((np.asarray(p, float), tag))
        else: runs.append([b, [(np.asarray(p, float), tag)]])
    return runs


def _xf(T, p): return (T @ np.r_[p, 1.0])[:3]
def _xv(T, v): return T[:3, :3] @ v


def centerline(route, pose=None, T=None, step=0.5, want_status=False):
    """线路在位姿 pose（{关节: 角度}；None = 零位）下的中心线（0.5 mm 重采样）。T = body_T(pose)（可预先算好传入）。want_status：另回每个柔性段的状态与长度"""
    if pose and T is None: T = body_T(pose)
    pose = pose or {}
    runs = _runs(route)
    pieces, status = [], []
    I = np.eye(4)
    for k, (b, pts) in enumerate(runs):
        Tb = I if T is None else T[b]
        P = np.array([_xf(Tb, p) for p, _ in pts])
        if k > 0:
            bA, ptsA = runs[k - 1]
            TA = I if T is None else T[bA]
            PA = np.array([_xf(TA, p) for p, _ in ptsA])
            pA, pB = PA[-1], P[0]
            tA = PA[-1] - PA[-2] if len(PA) > 1 else pB - pA
            tB = P[1] - P[0] if len(P) > 1 else pB - pA
            tA = tA / max(np.linalg.norm(tA), 1e-9); tB = tB / max(np.linalg.norm(tB), 1e-9)
            fx = route.get("flex", {}).get(k - 1, {})
            L = fx.get("L")
            if L is None:
                L = zero_flex_len(route, k - 1)
            if fx.get("model") == "wrap":
                ax_b, jn = fx["axis_body"], fx["joint"]
                from .lib import TW
                Ta = I if T is None else T[_tree()[0][ax_b]["parent"]]
                o = _xf(Ta, TW(ax_b)[:3, 3]); dvec = _xv(Ta, TW(ax_b)[:3, 2])
                ang = float(pose.get(jn, 0.0))
                # was_until_2026_09_26_hr44: seg, st = flex_wrap(pA, pB, o, dvec, fx["dtheta0"] + math.radians(ang) * fx.get("sign", 1.0), L, hpow=fx.get("hpow"))
                seg, st = flex_wrap(pA, pB, o, dvec, fx["dtheta0"] + math.radians(ang) * fx.get("sign", 1.0), L, hpow=fx.get("hpow"), bulge=fx.get("bulge"))
            else:
                seg, st = flex_hermite(pA, tA, pB, tB, L)
            pieces.append(seg[1:-1]); status.append(dict(k=k - 1, state=st, L=round(L, 2), len_min=round(_plen(seg), 2)))
        pieces.append(P)
    C = np.vstack(pieces)
    C = _resample(C, step)
    return (C, status) if want_status else C


def _resample(P, step):
    d = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(P, axis=0), axis=1))]
    if d[-1] < 1e-9: return P[:1]
    s = np.linspace(0, d[-1], max(2, int(math.ceil(d[-1] / step)) + 1))
    return np.stack([np.interp(s, d, P[:, k]) for k in range(3)], 1)


def zero_flex_len(route, k):
    """柔性段 k 的零位长度 = 零位两端点直连 Hermite（m = 弦长）长；route['flex'][k]['L'] 未给时用它 + slack"""
    runs = _runs(route)
    (bA, ptsA), (bB, ptsB) = runs[k], runs[k + 1]
    pA, pB = ptsA[-1][0], ptsB[0][0]
    fx = route.get("flex", {}).get(k, {})
    return float(np.linalg.norm(pB - pA)) + fx.get("slack", 0.0)


# ════════════════════════════════════════════════════════════════════════════════════════════
# 4. 姿态采样（= l6_motion：单轴 2.5° 独立网格 + 父子相邻关节两两 5° 网格；扫描域 = 上游 MJCF 区间；区内 / 区外按 frozen 目标区间）
# ════════════════════════════════════════════════════════════════════════════════════════════
STEP_DEG, COMBO_STEP_DEG = 2.5, 5.0
FIX_SKIP_MM = 6.0          # hr44：扎带固定点两侧这段弧长与所挂件的接触 = 预期接触（桥长 8 / 2 + 2）


def _grid(lo, hi, step):
    return sorted({round(float(a), 9) for a in [lo, hi, *np.arange(math.ceil(lo / step) * step, hi + 1e-9, step)]})


def target_ranges():
    import yaml
    p = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "tools", "gate", "data", "frozen.yaml")
    out = {}
    for j in (yaml.safe_load(open(p, encoding="utf-8")) or {}).get("joint_axes") or []:
        t = j.get("target_range_deg")
        v = t.get("v") if isinstance(t, dict) else t
        if isinstance(v, (list, tuple)) and len(v) == 2: out[j["name"]] = (float(v[0]), float(v[1]))
    return out


def poses_for(joints, coarse=False):
    """joints 的 l6 同款采样：[(pose, 来源)]。coarse=True（迭代用）：单轴 10°、两两 15°，端点 / 角点都含"""
    B, ORDER, _ = _tree(); jb = joint_body()
    rng = {j: tuple(float(x) for x in B[jb[j]]["joint"]["range_deg"]) for j in joints}
    out = [({}, "zero")]
    for j in joints:
        for a in _grid(*rng[j], 10.0 if coarse else STEP_DEG):
            if abs(a) > 1e-8: out.append(({j: a}, f"single:{j}"))
    pairs = []
    for j in joints:
        par = B[jb[j]]["parent"]
        while par is not None:
            pj = B[par].get("joint")
            if pj:
                if pj["name"] in joints: pairs.append((pj["name"], j))
                break
            par = B[par]["parent"]
    for j1, j2 in pairs:
        for a in _grid(*rng[j1], 15.0 if coarse else COMBO_STEP_DEG):
            for b in _grid(*rng[j2], 15.0 if coarse else COMBO_STEP_DEG):
                if abs(a) < 1e-8 and abs(b) < 1e-8: continue
                out.append(({j1: a, j2: b}, f"combo:{j1}+{j2}"))
    return out, rng, pairs


def in_target(pose, tgt):
    return all((j in tgt) and tgt[j][0] - 1e-6 <= a <= tgt[j][1] + 1e-6 for j, a in pose.items())


# ════════════════════════════════════════════════════════════════════════════════════════════
# 5. 占用体素（placed 全件按 body 标签，0.5 mm，竖直射线奇偶）—— 扫掠检查用
# ════════════════════════════════════════════════════════════════════════════════════════════
VOX = dict(P=0.5, lo=(-95.0, -90.0, 0.0), hi=(70.0, 90.0, 240.0))
_VOXC = {}


def _placed_dir():
    return os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "cad", "duck_s288", "placed"))


def body_for_part():
    root = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
    bf = json.load(open(os.path.join(root, "tools", "sim", "cad_geometry_manifest.json")))["body_for_part"]
    bf.update({"jaw_adapter": "jaw_soft", "jaw_journal": "jaw_soft", "camera_clamp": "jaw_soft", "adapter_tray": "jaw_soft",
               "amp_bracket": "jaw_soft", "zz_gpio_dupont": "jaw_soft", "tof_clamp": "jaw_soft", "zz_tof": "jaw_soft"})   # hr52：+ H10 / zz_tof
    return bf


def occupancy(placed=None, cache_dir=None):
    """(lab uint8 [nx,ny,nz]：0 空 / body 序号 + 1, 网格)；按 placed/*.stl 内容哈希缓存到 cad/duck_s288/cache/wires_body_vox.npz"""
    placed = placed or _placed_dir()
    B, ORDER, _ = _tree()
    bf = body_for_part()
    names = sorted(n for n in bf if os.path.exists(os.path.join(placed, n + ".stl")))
    h = hashlib.sha256()
    for n in names:
        h.update(n.encode()); h.update(open(os.path.join(placed, n + ".stl"), "rb").read())
    key = h.hexdigest()[:16]
    if key in _VOXC: return _VOXC[key]
    cache_dir = cache_dir or os.path.join(os.path.dirname(placed), "cache")
    os.makedirs(cache_dir, exist_ok=True)
    cp = os.path.join(cache_dir, "wires_body_vox.npz")
    if os.path.exists(cp):
        D = np.load(cp, allow_pickle=True)
        if str(D["key"]) == key:
            _VOXC[key] = (D["lab"], dict(P=float(D["P"]), lo=tuple(D["lo"]), shape=D["lab"].shape, bodies=list(D["bodies"])))
            return _VOXC[key]
    P = VOX["P"]; lo = np.array(VOX["lo"]); hi = np.array(VOX["hi"])
    xs = np.arange(lo[0] + P / 2, hi[0], P); ys = np.arange(lo[1] + P / 2, hi[1], P); zs = np.arange(lo[2] + P / 2, hi[2], P)
    NY, NZ = len(ys), len(zs)
    lab = np.zeros((len(xs), NY, NZ), np.uint8)
    XX, YY = np.meshgrid(xs, ys, indexing="ij"); COLS = np.stack([XX.ravel(), YY.ravel()], 1)
    for n in names:
        m = trimesh.load_mesh(os.path.join(placed, n + ".stl"), process=True)
        b = m.bounds; val = ORDER.index(bf[n]) + 1
        if b[1][2] < lo[2] or b[0][2] > hi[2]: continue
        sel = np.where((COLS[:, 0] >= b[0][0] - P) & (COLS[:, 0] <= b[1][0] + P) & (COLS[:, 1] >= b[0][1] - P) & (COLS[:, 1] <= b[1][1] + P))[0]
        for s0 in range(0, len(sel), 20000):
            ss = sel[s0:s0 + 20000]
            o = np.stack([COLS[ss, 0] + 0.0131, COLS[ss, 1] + 0.0077, np.full(len(ss), min(lo[2], b[0][2]) - 5.0)], 1)
            loc, ridx, _ = m.ray.intersects_location(o, np.tile([0.0, 0.0, 1.0], (len(ss), 1)), multiple_hits=True)
            if not len(loc): continue
            order = np.lexsort((loc[:, 2], ridx)); loc, ridx = loc[order], ridx[order]
            starts = np.r_[0, np.flatnonzero(np.diff(ridx)) + 1]; ends = np.r_[starts[1:], len(ridx)]
            for s, e in zip(starts, ends):
                zz = loc[s:e, 2]; zz = zz[np.r_[True, np.diff(zz) > 1e-6]]
                if len(zz) % 2: zz = zz[:-1]
                i, j = divmod(int(ss[ridx[s]]), NY)
                for a, bb in zip(zz[0::2], zz[1::2]):
                    k0 = int(np.ceil((a - lo[2]) / P - 0.5)); k1 = int(np.floor((bb - lo[2]) / P - 0.5))
                    if k1 < 0 or k0 >= NZ or k1 < k0: continue
                    lab[i, j, max(k0, 0):min(k1 + 1, NZ)] = val
    np.savez_compressed(cp, lab=lab, P=P, lo=lo, bodies=np.array(ORDER), key=key)
    _VOXC[key] = (lab, dict(P=P, lo=tuple(lo), shape=lab.shape, bodies=list(ORDER)))
    return _VOXC[key]


def tube_samples(C, r, k=10):
    """中心线 C 半径 r 的管表面抽样点（每 0.5 mm 一圈 k 点）+ 中心线本身"""
    T = np.gradient(C, axis=0); T /= np.maximum(np.linalg.norm(T, axis=1, keepdims=True), 1e-9)
    a = np.where(np.abs(T[:, 2:3]) < 0.9, np.array([[0, 0, 1.0]]), np.array([[1.0, 0, 0]]))
    N = np.cross(T, a); N /= np.maximum(np.linalg.norm(N, axis=1, keepdims=True), 1e-9); Bn = np.cross(T, N)
    ang = np.linspace(0, 2 * np.pi, k, endpoint=False)
    ring = [C + r * (np.cos(t) * N + np.sin(t) * Bn) for t in ang]
    return np.vstack([C] + ring)


def hits(Pw, T, lab, grid, skip_ends=None):
    """世界点 Pw（姿态 T 下）→ 每个 body 反变换到零位系查占用。返回 {body: 命中点数}"""
    B, ORDER, _ = _tree()
    P = grid["P"]; lo = np.array(grid["lo"]); sh = np.array(grid["shape"])
    out = {}
    H = np.c_[Pw, np.ones(len(Pw))]
    for bi, b in enumerate(ORDER):
        Q = (np.linalg.inv(T[b]) @ H.T).T[:, :3] if T is not None else Pw
        ii = np.floor((Q - lo) / P).astype(int)
        ok = np.all((ii >= 0) & (ii < sh), axis=1)
        if not ok.any(): continue
        v = lab[ii[ok, 0], ii[ok, 1], ii[ok, 2]]
        n = int((v == bi + 1).sum())
        if n: out[b] = n
    return out


# ── hr46（2026-09-26）：线撞"自己的固定件"（own）/ 撞别的（other）两桶证据 ─────────────────────────────────────────────
#   只给 L6 自己那批命中点贴标签（不多采一个点、不改任何计数，判据仍按合计口径）；膜测试对象只是本线路登记的 hook 型固定件实体
#   = cad/duck_s288/hooks/<件号>_<锚点 id>.stl（件的生成函数经 build_fast.produce_extra 产出，与件同一条缓存记录）。
FIXINGS = os.path.join(os.path.dirname(os.path.abspath(__file__)), "data", "wire_fixings_v1.json")
HOOKS_REL = "cad/duck_s288/hooks"
OWN_SD_MM = 0.25          # assumed：半个体素（VOX P 0.5 的一半）—— 命中点在钩实体里或离钩面 ≤ 0.25 算 own
_OWN_FIX = {}
_FIX_ANCHORS = {}


def hits_mask(Pw, T, lab, grid, only=None):
    """hr46：同 hits()，但返回 {body: (命中掩码 bool[len(Pw)], 该 body 零位系点 Q)}，只含命中点数 > 0 的 body，顺序同 hits()（ORDER）；
    掩码.sum() 与 hits() 的点数逐 body 相等（同一个式子）。only = body 名集合时只查这些 body（其余 body 的数不影响这些 body 的数）。"""
    B, ORDER, _ = _tree()
    P = grid["P"]; lo = np.array(grid["lo"]); sh = np.array(grid["shape"])
    out = {}
    H = np.c_[Pw, np.ones(len(Pw))]
    for bi, b in enumerate(ORDER):
        if only is not None and b not in only: continue
        Q = (np.linalg.inv(T[b]) @ H.T).T[:, :3] if T is not None else Pw
        ii = np.floor((Q - lo) / P).astype(int)
        ok = np.all((ii >= 0) & (ii < sh), axis=1)
        if not ok.any(): continue
        m = np.zeros(len(Pw), dtype=bool)
        m[ok] = lab[ii[ok, 0], ii[ok, 1], ii[ok, 2]] == bi + 1
        if m.any(): out[b] = (m, Q)
    return out


def _fsig(p):
    try:
        s = os.stat(p); return (True, s.st_mtime_ns, s.st_size)
    except OSError:
        return (False, 0, 0)


def own_fixing_solids(route_id):
    """hr46：线路 route_id 自己的 hook 型固定件（wire_fixings_v1.json anchors 里 route == route_id 且 type == "hook"）。
    文件 = cad/duck_s288/hooks/<part>_<id>.stl —— 由锚点的 part + id **拼名，不 glob**（写错标签的文件不会被吃进来）。
    返回 (solids=[(宿主 body, trimesh)], files=[相对路径], missing=[相对路径])。宿主 body = body_for_part()[件名]，件名由件号反查
    build_fast.FNAME（= build.py add() 的件名：N01 → neck → neck；T02 → shell_L → trunk_base；T03 → shell_R → trunk_base）。
    钩网格按 STL 读、合并顶点（process=True）。进程内缓存，键 = 锚点文件与各钩文件的（存在, mtime, size）—— 文件被删 / 改名 / 改写即重读。"""
    root = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
    fs = _fsig(FIXINGS)
    if _FIX_ANCHORS.get("sig") != fs:
        _FIX_ANCHORS.update(sig=fs, v=[a for a in (json.load(open(FIXINGS, encoding="utf-8")).get("anchors") or [])
                                        if isinstance(a, dict) and a.get("type") == "hook"])
    anchors = [a for a in _FIX_ANCHORS["v"] if a.get("route") == route_id]
    rels = [f"{HOOKS_REL}/{a['part']}_{a['id']}.stl" for a in anchors]
    key = (route_id, fs, tuple((rel, _fsig(os.path.join(root, rel))) for rel in rels))
    if key in _OWN_FIX: return _OWN_FIX[key]
    from duckstructure.build_fast import FNAME                       # 件名 → 导出文件名（件号 = 前 3 位）
    name_of = {fn[:3]: nm for nm, fn in FNAME.items()}
    bf = body_for_part()
    solids, files, missing = [], [], []
    for a, rel in zip(anchors, rels):
        p = os.path.join(root, rel)
        if not os.path.exists(p):
            missing.append(rel); continue
        nm = name_of.get(a["part"])
        if nm is None or nm not in bf:
            raise ValueError(f"hook 锚点 {a['id']} 的件号 {a['part']} 认不出宿主 body（build_fast.FNAME / cad_geometry_manifest body_for_part）")
        solids.append((bf[nm], trimesh.load_mesh(p, process=True))); files.append(rel)
    _OWN_FIX[key] = (solids, files, missing)
    return _OWN_FIX[key]


# ════════════════════════════════════════════════════════════════════════════════════════════
# 6. 线路数据（v1：hr43_work/hr44 规划脚本产出 → duckstructure/data/wiring_body_v1.json）+ 生成
# ════════════════════════════════════════════════════════════════════════════════════════════
DATA = os.path.join(os.path.dirname(os.path.abspath(__file__)), "data", "wiring_body_v1.json")


def routes():
    D = json.load(open(DATA, encoding="utf-8"))
    out = []
    for r in D["routes"]:
        r = dict(r)
        r["stations"] = [(b, tuple(p), t) for b, p, t in r["stations"]]
        r["flex"] = {int(k): v for k, v in (r.get("flex") or {}).items()}
        if r.get("od") is None:
            r["od"] = bundle_od(r["members"], wrap=len(r["members"]) > 1) if r.get("kind") == "bundle" else cable_equiv_d(r.get("cable") or r["members"][0])
        out.append(r)
    return out, D.get("junctions", {})


def _profile(r):
    if len(r["members"]) == 1:
        c = CABLE_B[r["members"][0]]
        if c["kind"] == "flat": return WH.profile_rect(c["w"], c["t"])
        if c["kind"] == "pair": return WH.profile_stadium(c["od"], c["n"])
        return WH.profile_circle(c["od"], 12)
    return WH.profile_circle(r["od"], 16)


def _sockets():
    """全部舵机插座（插头顶面中心、出线方向、局部系）—— lib.sfw @ KO01 (−13, ±7.75, −9.3)"""
    from . import lib as Lb
    B, ORDER, _ = _tree()
    out = []
    for n in ORDER:
        for i, s in enumerate(B[n]["servos"]):
            R = Lb.sfw(n, i)
            for sg in (-1, 1):
                out.append(dict(body=n, idx=i, side=sg, p=np.array(Lb.pt(R, -13.0, sg * 7.75, -9.3)), R=R))
    return out


def build(only=None):
    """返回 (meshes, meta)：wire__<线路>（世界系零位）、splitter__<节点>、conn__plug_<body>_<i>_<±>（PHR-3 插头本体 6.85×7.8×4.5，插在舵机里）。"""
    rs, junc = routes()
    meshes, meta = {}, {}
    socks = _sockets()
    used = set()
    for r in rs:
        if only and r["id"] not in only: continue
        C, st = centerline(r, step=0.5, want_status=True)
        T, S, U = WH.frames(C)
        m = WH.sweep(C, S, U, _profile(r))
        meshes["wire__" + r["id"]] = m
        R0, at, _ = WH.min_bend_radius(C)
        mb = min(CABLE_B[x]["min_bend"] for x in r["members"])
        meta[r["id"]] = dict(harness=r.get("harness"), members=r["members"], n_members=len(r["members"]), od_mm=r["od"], length_mm=round(_plen(C), 1),
                             bought_mm=r.get("bought"), bought_src=r.get("bought_src"), min_bend_R=round(float(R0), 2),
                             min_bend_at=None if at is None else [round(float(v), 1) for v in at], min_bend_limit=mb,
                             flex=[dict(k=s["k"], state=s["state"], L=s["L"]) for s in st], crosses=r.get("crosses"), status=r.get("status"),
                             v1_check=r.get("v1_check"), note=r.get("note", ""), volume_mm3=round(float(m.volume), 1),
                             bodies=sorted({b for b, _, _ in r["stations"]}))
        for (b, p, tag) in r["stations"]:
            if tag != "plug_top": continue
            k = min(range(len(socks)), key=lambda j: np.linalg.norm(socks[j]["p"] - np.asarray(p)))
            sk = socks[k]
            if np.linalg.norm(sk["p"] - np.asarray(p)) > 0.5 or (sk["body"], sk["idx"], sk["side"]) in used: continue
            used.add((sk["body"], sk["idx"], sk["side"]))
            Rm = sk["R"]; x, z = Rm[:3, 0], Rm[:3, 2]
            c = sk["p"] + x * WH.PLUG_BODY[0] / 2
            meshes[f"conn__plug_{sk['body']}_{sk['idx']}_{'p' if sk['side'] > 0 else 'n'}"] = WH.obox(c, x, z, WH.PLUG_BODY)
    for k, j in junc.items():
        ax = np.asarray(j["axis"], float); up = np.array([0, 0, 1.0]) if abs(ax[2]) < 0.9 else np.array([1.0, 0, 0])
        meshes["splitter__" + k] = WH.obox(np.asarray(j["c"], float), ax, up, j["size"])
        meta["splitter__" + k] = dict(note=j.get("note"), size=j["size"], center=j["c"])
    return meshes, meta


def export(outdir):
    os.makedirs(outdir, exist_ok=True)
    meshes, meta = build()
    for n, m in meshes.items():
        m.export(os.path.join(outdir, f"{n}.stl"))
    with open(os.path.join(outdir, "wires_body_meta.json"), "w") as f:
        json.dump(meta, f, ensure_ascii=False, indent=1)
    return meshes, meta


# ════════════════════════════════════════════════════════════════════════════════════════════
# 7. 检查
# ════════════════════════════════════════════════════════════════════════════════════════════
def sweep_check(only=None, end_skip_mm=4.0, every=1, placed=None, coarse=False):
    """l6 同采样扫掠：每条线路 → 跨的关节的单轴 2.5° + 父子相邻两两 5°；每个姿态 = 线外包管（半径 od/2）表面抽样点 × 各 body 占用体素。
    两端 end_skip_mm 内（插头 / 进头出口 / 口袋对插点，线本来就贴着自己插的件）不计。返回 {线路: 结果}。"""
    rs, _ = routes()
    lab, grid = occupancy(placed)
    tgt = target_ranges()
    out = {}
    for r in rs:
        if only and r["id"] not in only: continue
        js = list(r.get("crosses") or [])
        poses = poses_for(js, coarse=coarse)[0] if js else [({}, "zero")]
        rows, tens = [], 0
        tens_in = 0
        worstR = (1e9, None)
        for (pose, src) in poses[::every]:
            T = body_T(pose)
            C, st = centerline(r, pose=pose, T=T, step=1.0, want_status=True)
            bad = [s for s in st if s["state"] != "ok"]
            it = in_target(pose, tgt) if pose else True
            if bad:
                tens += 1; tens_in += int(it)
            L = np.r_[0, np.cumsum(np.linalg.norm(np.diff(C, axis=0), axis=1))]
            keep = (L >= end_skip_mm) & (L <= L[-1] - end_skip_mm)
            # hr44：扎带固定点（站点标签 fix_*）处线就是按在桥 / 板上的 → 该站点 ±FIX_SKIP_MM 弧长内的抽样点与**该站点所挂 body** 的接触算预期接触（同插头），
            #   其余 body 照查。弧长位置 = 该站点世界坐标在本姿态中心线上的最近点。
            fx_masks = []
            for (b_, p_, t_) in r["stations"]:
                if str(t_).startswith("fix_"):
                    pw = _xf(T[b_], np.asarray(p_, float)); kk = int(np.argmin(np.linalg.norm(C - pw, axis=1)))
                    fx_masks.append((b_, (L >= L[kk] - FIX_SKIP_MM) & (L <= L[kk] + FIX_SKIP_MM)))
            if keep.sum() < 2: continue
            if fx_masks:
                own = {}
                for b_, msk in fx_masks: own[b_] = own.get(b_, np.zeros_like(keep)) | msk
                h = hits(tube_samples(C[keep], r["od"] / 2, k=10), T, lab, grid)
                for b_, msk in own.items():
                    if b_ in h:
                        k2 = keep & ~msk
                        hb = hits(tube_samples(C[k2], r["od"] / 2, k=10), T, lab, grid).get(b_, 0) if k2.sum() >= 2 else 0
                        if hb: h[b_] = hb
                        else: h.pop(b_)
            else:
                Pw = tube_samples(C[keep], r["od"] / 2, k=10)
                h = hits(Pw, T, lab, grid)
            if h: rows.append(dict(pose=pose, src=src, in_target=it, hits=h))
            if len(C) > 8:
                R, at, _ = WH.min_bend_radius(C)
                if R < worstR[0]: worstR = (float(R), pose)
        out[r["id"]] = dict(n_poses=len(poses[::every]), n_hit=len(rows), n_hit_in_target=sum(1 for x in rows if x["in_target"]),
                            tension=tens, tension_in_target=tens_in, worst_bend=[round(worstR[0], 2), worstR[1]], crosses=js,
                            rows_in_target=[x for x in rows if x["in_target"]][:12])
    return out


def zero_check(placed=None, only=None):
    """零位：线 / 一分二 / 插头 × placed 件（manifold 精确布尔，同 hr43d d_check_wires 口径）。插头 × 它插的舵机 = 预期接触。"""
    import manifold3d as M
    placed = placed or _placed_dir()
    bf = body_for_part()
    def sol(m):
        m = m.copy(); m.merge_vertices()
        s = M.Manifold(M.Mesh64(vert_properties=np.array(m.vertices, dtype=np.float64, order="C"), tri_verts=np.array(m.faces, dtype=np.uint64, order="C")))
        return s if s.status() == M.Error.NoError else None
    meshes, meta = build(only)
    parts = {}
    for n in bf:
        p = os.path.join(placed, n + ".stl")
        if os.path.exists(p):
            s = sol(trimesh.load_mesh(p, process=True))
            if s is not None: parts[n] = s
    res = {}
    for n, m in meshes.items():
        s = sol(m)
        if s is None: res[n] = "not_manifold"; continue
        A = np.asarray(s.bounding_box()).reshape(2, 3)
        h = {}
        for pn, ps in parts.items():
            Bb = np.asarray(ps.bounding_box()).reshape(2, 3)
            if np.any(A[1] < Bb[0]) or np.any(Bb[1] < A[0]): continue
            v = (s ^ ps).volume()
            if v > 0.01: h[pn] = round(v, 3)
        if h: res[n] = h
    return res


def sweep_summary_was_until_2026_09_26_hr44b(coarse=False):
    """【留痕，hr44 第二轮前的契约；现行 sweep_summary 在文件末第 7 节】API（hr44reg / L6 harness_sweep 用，协调员 23:15 契约）：
    {线路 id: {"n_poses", "hit_in_target", "tension_in_target", "min_bend_R", "min_bend_limit", "od_mm", "length_mm", "status"}}。
    coarse=True：单轴 10° / 两两 15°（含端点角点，≤60 s）；False：l6 同全集（单轴 2.5° / 两两 5°）。另附 "hit_all" / "tension_all" / "worst_bend_R_swept"。"""
    s = sweep_check(coarse=coarse)
    _, meta = build()
    out = {}
    for k, v in s.items():
        m = meta.get(k, {})
        out[k] = dict(n_poses=v["n_poses"], hit_in_target=v["n_hit_in_target"], tension_in_target=v["tension_in_target"],
                      min_bend_R=m.get("min_bend_R"), min_bend_limit=m.get("min_bend_limit"), od_mm=m.get("od_mm"), length_mm=m.get("length_mm"),
                      status=m.get("status"), hit_all=v["n_hit"], tension_all=v["tension"], worst_bend_R_swept=v["worst_bend"][0], crosses=v["crosses"])
    return out


def export_all(outdir, coarse=True):
    """build / build_fast 末尾调（hr44）：头内（wiring_head）+ 头外（本模块）线束实体导出到 outdir（= cad/duck_s288/wires/，placed 旁），
    写 wires_meta.json（head / body 两段）+ wires_check.json（头外线路 sweep_summary，coarse 默认 = 粗采样 ≈10 s）。返回一行摘要。"""
    import time as _t
    from . import wiring_head as _WH
    t0 = _t.time()
    os.makedirs(outdir, exist_ok=True)
    for f in os.listdir(outdir):
        if f.endswith(".stl") or f.endswith(".json"): os.remove(os.path.join(outdir, f))
    mh, meta_h = _WH.build(); mb, meta_b = build()
    for n, m in list(mh.items()) + list(mb.items()):
        m.export(os.path.join(outdir, n + ".stl"))
    def _clean(o):
        if isinstance(o, dict): return {k: _clean(v) for k, v in o.items()}
        if isinstance(o, (list, tuple)): return [_clean(v) for v in o]
        if isinstance(o, np.ndarray): return o.tolist()
        if isinstance(o, np.floating): return float(o)
        if isinstance(o, np.integer): return int(o)
        return o
    json.dump(dict(head=_clean(meta_h), body=_clean(meta_b)), open(os.path.join(outdir, "wires_meta.json"), "w"), ensure_ascii=False, indent=1)
    s = sweep_summary(coarse=coarse)
    json.dump(dict(coarse=coarse, summary=_clean(s)), open(os.path.join(outdir, "wires_check.json"), "w"), ensure_ascii=False, indent=1)
    # was_until_2026_09_26_hr44b: bad = {k: (v["hit_in_target"], v["tension_in_target"]) for k, v in s.items() if v["hit_in_target"] or v["tension_in_target"]}
    bad = {k: (v["hit_in_target_limb"], v["hit_real_poses"], v["tension_in_target_limb"], v["tension_real_poses"]) for k, v in s.items()
           if v["hit_in_target_limb"] or v["hit_real_poses"] or v["tension_in_target_limb"] or v["tension_real_poses"]}
    return f"[线束] 导出 {len(mh) + len(mb)} 实体 → {outdir}；头外 {len(s)} 条线路{'粗' if coarse else '全'}采样：（肢体区内命中, 真实姿态命中, 肢体区内张紧, 真实姿态张紧）非零 {bad if bad else '无'}（{_t.time() - t0:.0f} s）"


# ════════════════════════════════════════════════════════════════════════════════════════════
# 7. hr44 第二轮（复审 #3 M2，2026-09-26）：姿态集扩到线路所在肢体链的全部关节 + 真实姿态集
#    旧 sweep_check 只扫线路自己跨的关节（r["crosses"]）→ 别的关节带动的件（脚 / 小腿 / 大腿）压线漏判（复审 B-3：bus_L_3to4 真实姿态 >1 mm 116 个、
#    bus_R_1to2 最深 7.1，而记分卡 hit_in_target 都是 0）。现在：
#      limb 姿态集 = 该肢体链全部关节的单轴 2.5° + 父子相邻两两 5°（coarse：10° / 15°），腿线 = 该腿 5 关节，躯干线 = 两髋偏航，过颈 = 颈 4 关节；
#      real 姿态集 = 真实姿态 5601（frozen.yaml capability_envelope.modes_in_scope 的 prefall 段、2.5° 网格去重，与 L6 / policy_pose_collisions 同源；不降采样）。
#    sweep_summary 新键：hit_in_target_limb / tension_in_target_limb / n_poses_limb、hit_real_poses / tension_real_poses / n_real_poses；
#    旧键 hit_in_target / tension_in_target 保留作留痕（= 只扫跨越关节），L6 不读旧键（Gate 守护 02:32 硬要求 1）。
# ════════════════════════════════════════════════════════════════════════════════════════════
LIMB_JOINTS = {
    "left_leg": ("left_hip_yaw", "left_hip_roll", "left_hip_pitch", "left_knee", "left_ankle"),
    "right_leg": ("right_hip_yaw", "right_hip_roll", "right_hip_pitch", "right_knee", "right_ankle"),
    "torso": ("left_hip_yaw", "right_hip_yaw"),
    "neck": ("neck_pitch", "head_pitch", "head_yaw", "head_roll"),
}
_LIMB_OF_BODY = {}


def _limb_body_map():
    if _LIMB_OF_BODY: return _LIMB_OF_BODY
    B, ORDER, _ = _tree(); jb = joint_body()
    def desc(n):
        out = {n}
        for c in (B[n].get("children") or []): out |= desc(c["name"])
        return out
    for limb, js in LIMB_JOINTS.items():
        if limb == "torso": continue
        for b in desc(jb[js[0]]): _LIMB_OF_BODY[b] = limb
    return _LIMB_OF_BODY


def limb_of(route):
    """线路所在肢体链：站点 body（除 trunk_base）落在哪条腿 / 颈子树 + 跨的关节属于哪条；都没有 → torso（躯干线，姿态集 = 两髋偏航）"""
    lb = _limb_body_map()
    limbs = {lb.get(b) for b, _, _ in route["stations"] if b != "trunk_base"} - {None}
    cr = set(route.get("crosses") or [])
    limbs |= {l for l, js in LIMB_JOINTS.items() if l != "torso" and cr & set(js)}
    return "+".join(sorted(limbs)) if limbs else "torso"


def limb_joints(route):
    js = []
    for l in limb_of(route).split("+"):
        js += [j for j in LIMB_JOINTS[l] if j not in js]
    return js


_REAL = {}


def real_poses():
    """真实姿态集 [{关节: 度}]（policy_steps_2026-09-16 的 in-scope prefall 段，2.5° 网格去重，= L6 / tools/sim/policy_pose_collisions 同源），进程内缓存"""
    if "v" in _REAL: return _REAL["v"]
    import sys as _sys, yaml as _yaml
    root = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
    p = os.path.join(root, "tools", "sim")
    if p not in _sys.path: _sys.path.insert(0, p)
    import policy_pose_evidence as _EV
    fz = _yaml.safe_load(open(os.path.join(root, "tools", "gate", "data", "frozen.yaml"), encoding="utf-8"))
    scope = list(fz["capability_envelope"]["modes_in_scope"])
    ex = _EV.expected_poses(os.path.join(root, "tools", "gate", "out", "policy_steps_2026-09-16", "policy_steps.npz"), scope, 2.5)
    _REAL["v"] = [{str(k): float(x) for k, x in v.items()} for v in ex["rows"].values()]   # rows：(mode, case, step) → {关节: 度}（未取整）
    _REAL["keys"] = list(ex["rows"].keys())
    return _REAL["v"]


def _eval_pose_was_until_2026_09_26_hr46(r, T, pose, lab, grid, end_skip_mm):
    """【留痕，hr46 前；现行 _eval_pose 在下面】一条线路在一个姿态：(命中 {body: 点数} 或 {}, 是否张紧, 最小弯 R)"""
    C, st = centerline(r, pose=pose, T=T, step=1.0, want_status=True)
    bad = any(s["state"] != "ok" for s in st)
    L = np.r_[0, np.cumsum(np.linalg.norm(np.diff(C, axis=0), axis=1))]
    keep = (L >= end_skip_mm) & (L <= L[-1] - end_skip_mm)
    if keep.sum() < 2: return {}, bad, 1e9
    own = {}
    for (b_, p_, t_) in r["stations"]:
        if str(t_).startswith("fix_"):
            pw = _xf(T[b_], np.asarray(p_, float)); kk = int(np.argmin(np.linalg.norm(C - pw, axis=1)))
            own[b_] = own.get(b_, np.zeros_like(keep)) | ((L >= L[kk] - FIX_SKIP_MM) & (L <= L[kk] + FIX_SKIP_MM))
    h = hits(tube_samples(C[keep], r["od"] / 2, k=10), T, lab, grid)
    for b_, msk in own.items():
        if b_ in h:
            k2 = keep & ~msk
            hb = hits(tube_samples(C[k2], r["od"] / 2, k=10), T, lab, grid).get(b_, 0) if k2.sum() >= 2 else 0
            if hb: h[b_] = hb
            else: h.pop(b_)
    R = WH.min_bend_radius(C)[0] if len(C) > 8 else 1e9
    return h, bad, float(R)


def _eval_pose(r, T, pose, lab, grid, end_skip_mm, classify=True, own_solids=None):
    """一条线路在一个姿态：(命中 {body: 点数} 或 {}, 是否张紧, 最小弯 R, own_any, other_any)。
    hr46：h / 张紧 / 弯 R 与 hr46 前（_eval_pose_was_until_2026_09_26_hr46）同一批点、同一口径 —— 计数 = hits_mask 掩码.sum()（与 hits() 同式）；
      fix 站点 ±FIX_SKIP_MM 复算只查该站点所挂 body（原来算全部 body 再 .get(该 body)，该 body 的数与别的 body 无关 → 数相同）。
      多返回 (own_any, other_any)：对**最终计入 h 的那批点**（fix 屏蔽之后）贴标签 —— 点命中的 body == 本线路某个 own 钩的宿主 body，
      且该点（已在宿主零位系 = inv(T[宿主]) @ p）到钩实体的 trimesh.proximity.signed_distance ≥ −OWN_SD_MM → own；其余 → other。
      钩实体 AABB 外扩 OWN_SD_MM 以外的点离钩面必 > OWN_SD_MM（signed_distance < −OWN_SD_MM）→ 先滤掉，只对剩下的点算（结果与全算相同）。
      classify=False → 不贴标签（返回 False, bool(h)，调用方不用这两个值）；own_solids = own_fixing_solids(r["id"])[0]（None → 现取）。"""
    C, st = centerline(r, pose=pose, T=T, step=1.0, want_status=True)
    bad = any(s["state"] != "ok" for s in st)
    L = np.r_[0, np.cumsum(np.linalg.norm(np.diff(C, axis=0), axis=1))]
    keep = (L >= end_skip_mm) & (L <= L[-1] - end_skip_mm)
    if keep.sum() < 2: return {}, bad, 1e9, False, False
    own = {}
    for (b_, p_, t_) in r["stations"]:
        if str(t_).startswith("fix_"):
            pw = _xf(T[b_], np.asarray(p_, float)); kk = int(np.argmin(np.linalg.norm(C - pw, axis=1)))
            own[b_] = own.get(b_, np.zeros_like(keep)) | ((L >= L[kk] - FIX_SKIP_MM) & (L <= L[kk] + FIX_SKIP_MM))
    fin = hits_mask(tube_samples(C[keep], r["od"] / 2, k=10), T, lab, grid)    # body → (掩码, 零位系点) = 最终计入 h 的点
    h = {b: int(m.sum()) for b, (m, _q) in fin.items()}                         # = hits(同一批点)
    for b_, msk in own.items():
        if b_ in h:
            k2 = keep & ~msk
            f2 = hits_mask(tube_samples(C[k2], r["od"] / 2, k=10), T, lab, grid, only={b_}) if k2.sum() >= 2 else {}
            hb = int(f2[b_][0].sum()) if b_ in f2 else 0
            if hb: h[b_] = hb; fin[b_] = f2[b_]
            else: h.pop(b_); fin.pop(b_)
    R = WH.min_bend_radius(C)[0] if len(C) > 8 else 1e9
    if not h: return h, bad, float(R), False, False
    if not classify: return h, bad, float(R), False, True
    sol = own_fixing_solids(r["id"])[0] if own_solids is None else own_solids
    own_any = other_any = False
    for b, (m, Q) in fin.items():
        ms_b = [ms for (hb_, ms) in sol if hb_ == b]
        if not ms_b:
            other_any = True; continue
        q = Q[m]; is_own = np.zeros(len(q), dtype=bool)
        for ms in ms_b:
            lo_, hi_ = ms.bounds[0] - OWN_SD_MM - 1e-6, ms.bounds[1] + OWN_SD_MM + 1e-6
            cand = np.flatnonzero(~is_own & np.all((q >= lo_) & (q <= hi_), axis=1))
            if len(cand):
                is_own[cand[trimesh.proximity.signed_distance(ms, q[cand]) >= -OWN_SD_MM]] = True
        own_any = own_any or bool(is_own.any()); other_any = other_any or bool(not is_own.all())
    return h, bad, float(R), own_any, other_any


def sweep_limb_real(only=None, end_skip_mm=4.0, placed=None, coarse=False):
    """复审 M2：每条线路按『所在肢体链全部关节』姿态集（区内判）+ 真实姿态集各跑一遍。
    返回 {线路: dict(limb, limb_joints, n_poses_limb, hit_in_target_limb, tension_in_target_limb, hit_limb_all, rows_limb[:12],
                     n_real_poses, hit_real_poses, tension_real_poses, rows_real[:12], worst_bend_limb, worst_bend_real)}。真实姿态不降采样。
    hr46：另有两桶证据 hit_in_target_limb_own / _other / _both、hit_real_poses_own / _other / _both（按姿态计：该姿态最终命中点里有 own / 有 other / 都有）、
    own_fixing_files / own_fixing_missing（own_fixing_solids 拼名找的钩文件）、identity_ok（两套 own + other − both == 合计）。合计键与 hr46 前逐字同口径。"""
    rs, _ = routes()
    rs = [r for r in rs if not only or r["id"] in only]
    lab, grid = occupancy(placed)
    tgt = target_ranges()
    out = {r["id"]: dict(limb=limb_of(r), limb_joints=limb_joints(r), n_poses_limb=0, hit_in_target_limb=0, tension_in_target_limb=0, hit_limb_all=0,
                         rows_limb=[], n_real_poses=0, hit_real_poses=0, tension_real_poses=0, rows_real=[], worst_bend_limb=[1e9, None],
                         worst_bend_real=[1e9, None]) for r in rs}
    own_sol = {}
    for r in rs:                                             # hr46：两桶证据（own = 撞本线路自己登记的 hook 型固定件实体 / other = 其余）；钩网格每条线路只读一次
        sol, files, missing = own_fixing_solids(r["id"]); own_sol[r["id"]] = sol
        out[r["id"]].update(hit_in_target_limb_own=0, hit_in_target_limb_other=0, hit_in_target_limb_both=0,
                            hit_real_poses_own=0, hit_real_poses_other=0, hit_real_poses_both=0,
                            own_fixing_files=list(files), own_fixing_missing=list(missing))
    # limb：按姿态集分组，同组线路共用 body_T
    groups = {}
    for r in rs: groups.setdefault(tuple(limb_joints(r)), []).append(r)
    for js, grp in groups.items():
        poses = poses_for(list(js), coarse=coarse)[0]
        for (pose, src) in poses:
            T = body_T(pose)
            it = in_target(pose, tgt) if pose else True
            for r in grp:
                o = out[r["id"]]; o["n_poses_limb"] += 1
                # was_until_2026_09_26_hr46: h, bad, R = _eval_pose(r, T, pose, lab, grid, end_skip_mm)
                h, bad, R, ow, ot = _eval_pose(r, T, pose, lab, grid, end_skip_mm, classify=bool(it), own_solids=own_sol[r["id"]])
                if h:
                    o["hit_limb_all"] += 1
                    if it:
                        o["hit_in_target_limb"] += 1
                        o["hit_in_target_limb_own"] += int(ow); o["hit_in_target_limb_other"] += int(ot); o["hit_in_target_limb_both"] += int(ow and ot)   # hr46
                        if len(o["rows_limb"]) < 12: o["rows_limb"].append(dict(pose=pose, src=src, hits=h))
                if bad and it: o["tension_in_target_limb"] += 1
                if R < o["worst_bend_limb"][0]: o["worst_bend_limb"] = [round(R, 2), pose]
    # real：姿态外层、线路内层（每个姿态只算一次 body_T）
    for pose in real_poses():
        T = body_T(pose)
        for r in rs:
            o = out[r["id"]]; o["n_real_poses"] += 1
            # was_until_2026_09_26_hr46: h, bad, R = _eval_pose(r, T, pose, lab, grid, end_skip_mm)
            h, bad, R, ow, ot = _eval_pose(r, T, pose, lab, grid, end_skip_mm, own_solids=own_sol[r["id"]])
            if h:
                o["hit_real_poses"] += 1
                o["hit_real_poses_own"] += int(ow); o["hit_real_poses_other"] += int(ot); o["hit_real_poses_both"] += int(ow and ot)   # hr46
                if len(o["rows_real"]) < 12: o["rows_real"].append(dict(pose={k: round(v, 2) for k, v in pose.items()}, hits=h))
            if bad: o["tension_real_poses"] += 1
            if R < o["worst_bend_real"][0]: o["worst_bend_real"] = [round(R, 2), {k: round(v, 2) for k, v in pose.items()}]
    for o in out.values():                                   # hr46：恒等式 own + other − both == 合计（肢体区内 / 真实姿态两套）
        o["identity_ok"] = bool(o["hit_in_target_limb_own"] + o["hit_in_target_limb_other"] - o["hit_in_target_limb_both"] == o["hit_in_target_limb"]
                                and o["hit_real_poses_own"] + o["hit_real_poses_other"] - o["hit_real_poses_both"] == o["hit_real_poses"])
    return out


def sweep_summary(coarse=False):
    """API（L6 harness_sweep 用）。hr44 第二轮（复审 #3 M2，2026-09-26）契约：
    {线路 id: {"n_poses"（= n_poses_limb）, "hit_in_target_limb", "tension_in_target_limb", "n_poses_limb", "limb", "limb_joints",
               "hit_real_poses", "tension_real_poses", "n_real_poses", "min_bend_R", "min_bend_limit", "od_mm", "length_mm", "status",
               旧键留痕 "hit_in_target" / "tension_in_target"（= 只扫跨越关节）, "hit_all", "tension_all", "worst_bend_R_swept", "crosses"}}。
    L6 判 PASS ⇔ hit_in_target_limb == 0 且 hit_real_poses == 0 且 tension_in_target_limb == 0 且 tension_real_poses == 0 且 min_bend_R ≥ min_bend_limit；
    新键缺任何一个 → NOT_RUN（不许回退旧键）。coarse=True：肢体姿态集单轴 10° / 两两 15°；真实姿态永远全 5601。"""
    s = sweep_check(coarse=coarse)
    x = sweep_limb_real(coarse=coarse)
    _, meta = build()
    out = {}
    for k, v in s.items():
        m = meta.get(k, {}); w = x.get(k, {})
        out[k] = dict(n_poses=w.get("n_poses_limb"), hit_in_target_limb=w.get("hit_in_target_limb"), tension_in_target_limb=w.get("tension_in_target_limb"),
                      n_poses_limb=w.get("n_poses_limb"), limb=w.get("limb"), limb_joints=w.get("limb_joints"), hit_limb_all=w.get("hit_limb_all"),
                      hit_real_poses=w.get("hit_real_poses"), tension_real_poses=w.get("tension_real_poses"), n_real_poses=w.get("n_real_poses"),
                      real_examples=[dict(pose=q["pose"], hits=q["hits"]) for q in (w.get("rows_real") or [])[:3]],
                      limb_examples=[dict(pose=q["pose"], hits=q["hits"]) for q in (w.get("rows_limb") or [])[:3]],
                      min_bend_R=m.get("min_bend_R"), min_bend_limit=m.get("min_bend_limit"), od_mm=m.get("od_mm"), length_mm=m.get("length_mm"),
                      status=m.get("status"), hit_in_target=v["n_hit_in_target"], tension_in_target=v["tension_in_target"], n_poses_crosses=v["n_poses"],
                      hit_all=v["n_hit"], tension_all=v["tension"],
                      worst_bend_R_swept=min(v["worst_bend"][0], (w.get("worst_bend_limb") or [1e9])[0], (w.get("worst_bend_real") or [1e9])[0]),
                      crosses=v["crosses"],
                      # hr46：两桶证据原样透传（上面旧键一个不动；L6 判据仍按合计口径）
                      hit_in_target_limb_own=w.get("hit_in_target_limb_own"), hit_in_target_limb_other=w.get("hit_in_target_limb_other"),
                      hit_in_target_limb_both=w.get("hit_in_target_limb_both"), hit_real_poses_own=w.get("hit_real_poses_own"),
                      hit_real_poses_other=w.get("hit_real_poses_other"), hit_real_poses_both=w.get("hit_real_poses_both"),
                      own_fixing_files=w.get("own_fixing_files"), own_fixing_missing=w.get("own_fixing_missing"), identity_ok=w.get("identity_ok"))
    return out
