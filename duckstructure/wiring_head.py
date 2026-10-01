"""头内线束实体（hr39，2026-09-23）—— 独立模块，**本轮不进 build / Gate**。

用户原话：「建模时最好把这些线也建进去，这样我们能大致知道线该从哪儿走，比如线太粗的话也能提前发现」「线的体积也要算进去」。
范围 = jaw_soft 刚体内（头里）的线：CSI 排线、USB-C 线、UBEC 输入/输出、麦克风/功放杜邦线（含 pin12/pin35 两根一分二）、喇叭线、
总线 PH（转接板 → S1 → 头横滚 8 号；S1 → 延长线出头）、过颈束 HB09 的头内段。**不做**嘴舵机的线和 S6（嘴部在另一个 agent 手里重排）。

做法：每根线 = 航点 → 向心 Catmull-Rom（α=0.5）→ 每 0.5 mm 重采样 → 沿曲线扫截面（圆线扫圆、扁线扫矩形、两芯并线扫跑道形、
FPC 扫 0.3 厚矩形并按「宽向」航点控制朝向）→ 闭合实体；插头 = 有向盒；多余线长 = 线自己在「盘线区」里绕的那一段（体积随线一起算）。
所有坐标 = 世界系零位姿（与 duckstructure/electronics.py、cad/duck_s288/placed 同系），单位 mm。
每个尺寸都标来源：datasheet / measured / drawing（官方机械图实量）/ assumed（没数据，按常见件估，到货要量）。「实物未知不编数」：
assumed 的值在报告里单列。

用法：
    from duckstructure import wiring_head as WH
    meshes, meta = WH.build()     # meshes {名字: trimesh}（wire__ 线 / conn__ 插头 / stow__ 盘线区 / ref__ 按官方图补的实体），世界系；meta 每根线的长度/截面/弯折/买的长度
    WH.export(outdir)             # 每个实体一个 STL（世界系）+ wires_meta.json（长度/截面/航点/买的长度）
检查脚本：docs/design_2026-09-17_bearing_rebuild/hr39_wires/check_wires.py；报告：docs/design_2026-09-17_bearing_rebuild/hr39_头内线束.md
"""
import os, math, json
import numpy as np
import trimesh

# ════════════════════════════════════════════════════════════════════════════════════════════
# 0. 几何引擎
# ════════════════════════════════════════════════════════════════════════════════════════════
def _u(v):
    v = np.asarray(v, float); n = np.linalg.norm(v)
    return v / n if n > 1e-12 else v


def obox(center, xax, yax, size):
    """有向盒：中心、局部 x/y 轴（世界向量，自动正交化）、尺寸 (sx, sy, sz)。"""
    x = _u(xax); y = _u(np.asarray(yax, float) - np.dot(yax, x) * x); z = np.cross(x, y)
    T = np.eye(4); T[:3, 0], T[:3, 1], T[:3, 2], T[:3, 3] = x, y, z, center
    return trimesh.creation.box(extents=size, transform=T)


def abox(lo, hi):
    """轴对齐盒（世界系）。"""
    lo, hi = np.asarray(lo, float), np.asarray(hi, float)
    return trimesh.creation.box(extents=hi - lo, transform=trimesh.transformations.translation_matrix((lo + hi) / 2))


def catmull_rom(P, step=0.5, alpha=0.5):
    """向心 Catmull-Rom 过全部航点（两端外推一个虚点），按弧长约 step 重采样。返回 (n,3) 以及每个航点在输出里的下标。"""
    P = np.asarray(P, float)
    if len(P) == 2:
        L = np.linalg.norm(P[1] - P[0]); n = max(2, int(math.ceil(L / step)) + 1)
        return np.linspace(P[0], P[1], n), [0, n - 1]
    Q = np.vstack([2 * P[0] - P[1], P, 2 * P[-1] - P[-2]])
    out, marks = [P[0]], [0]
    for i in range(1, len(Q) - 2):
        p0, p1, p2, p3 = Q[i - 1], Q[i], Q[i + 1], Q[i + 2]
        t0 = 0.0
        t1 = t0 + max(np.linalg.norm(p1 - p0), 1e-6) ** alpha
        t2 = t1 + max(np.linalg.norm(p2 - p1), 1e-6) ** alpha
        t3 = t2 + max(np.linalg.norm(p3 - p2), 1e-6) ** alpha
        seg = np.linalg.norm(p2 - p1)
        n = max(2, int(math.ceil(seg * 1.3 / step)))
        for t in np.linspace(t1, t2, n + 1)[1:]:
            A1 = (t1 - t) / (t1 - t0) * p0 + (t - t0) / (t1 - t0) * p1
            A2 = (t2 - t) / (t2 - t1) * p1 + (t - t1) / (t2 - t1) * p2
            A3 = (t3 - t) / (t3 - t2) * p2 + (t - t2) / (t3 - t2) * p3
            B1 = (t2 - t) / (t2 - t0) * A1 + (t - t0) / (t2 - t0) * A2
            B2 = (t3 - t) / (t3 - t1) * A2 + (t - t1) / (t3 - t1) * A3
            out.append((t2 - t) / (t2 - t1) * B1 + (t - t1) / (t2 - t1) * B2)
        marks.append(len(out) - 1)
    return _resample(np.array(out), step, marks)


def _resample(pts, step, marks):
    d = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(pts, axis=0), axis=1))]
    L = d[-1]; n = max(2, int(math.ceil(L / step)) + 1)
    s = np.linspace(0, L, n)
    out = np.stack([np.interp(s, d, pts[:, k]) for k in range(3)], 1)
    newmarks = [int(np.argmin(abs(s - d[m]))) for m in marks]
    return out, newmarks


def _tangents(pts):
    T = np.gradient(pts, axis=0)
    return np.array([_u(t) for t in T])


def frames(pts, side_hints=None, marks=None):
    """每个采样点的 (T, S, U)：T 切向；S「宽向」（扁线的宽边方向）；U = T×S（厚向）。
    side_hints：每个航点一个宽向提示（None = 该点不约束）；在有提示的航点之间按弧长线性插值再投影到法平面；
    全为 None 时用旋转最小标架（双反射法）。"""
    T = _tangents(pts)
    n = len(pts)
    S = np.zeros_like(pts)
    if side_hints is None or all(h is None for h in side_hints):
        a = np.array([0, 0, 1.0]) if abs(T[0][2]) < 0.9 else np.array([1.0, 0, 0])
        S[0] = _u(a - np.dot(a, T[0]) * T[0])
        for i in range(n - 1):   # double reflection (Wang et al. 2008)
            v1 = pts[i + 1] - pts[i]; c1 = np.dot(v1, v1)
            if c1 < 1e-12: S[i + 1] = S[i]; continue
            rL = S[i] - (2 / c1) * np.dot(v1, S[i]) * v1; tL = T[i] - (2 / c1) * np.dot(v1, T[i]) * v1
            v2 = T[i + 1] - tL; c2 = np.dot(v2, v2)
            S[i + 1] = rL - (2 / c2) * np.dot(v2, rL) * v2 if c2 > 1e-12 else rL
            S[i + 1] = _u(S[i + 1] - np.dot(S[i + 1], T[i + 1]) * T[i + 1])
    else:
        d = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(pts, axis=0), axis=1))]
        known = [(d[marks[k]], np.asarray(h, float)) for k, h in enumerate(side_hints) if h is not None]
        for i in range(n):
            if d[i] <= known[0][0]: h = known[0][1]
            elif d[i] >= known[-1][0]: h = known[-1][1]
            else:
                for (da, ha), (db, hb) in zip(known[:-1], known[1:]):
                    if da <= d[i] <= db:
                        w = (d[i] - da) / max(db - da, 1e-9)
                        if np.dot(ha, hb) < 0: hb = -hb
                        h = (1 - w) * ha + w * hb; break
            s = h - np.dot(h, T[i]) * T[i]
            if np.linalg.norm(s) < 1e-6:   # 提示与切向平行：沿用上一个
                s = S[i - 1] if i else np.cross(T[i], [0, 0, 1.0])
            S[i] = _u(s)
            if i and np.dot(S[i], S[i - 1]) < 0: S[i] = -S[i]
    U = np.cross(T, S)
    return T, S, U


def profile_circle(d, k=12):
    a = np.linspace(0, 2 * np.pi, k, endpoint=False)
    return np.stack([0.5 * d * np.cos(a), 0.5 * d * np.sin(a)], 1)


def profile_rect(w, t):
    return np.array([[-w / 2, -t / 2], [w / 2, -t / 2], [w / 2, t / 2], [-w / 2, t / 2]])


def profile_stadium(d, n, k=8):
    """n 根外径 d 的圆线并排（宽向 S）的外包（跑道形/多圆凸包）。"""
    if n == 1: return profile_circle(d, 2 * k)
    half = (n - 1) * d / 2
    a1 = np.linspace(-np.pi / 2, np.pi / 2, k + 1); a2 = np.linspace(np.pi / 2, 3 * np.pi / 2, k + 1)
    right = np.stack([half + 0.5 * d * np.cos(a1), 0.5 * d * np.sin(a1)], 1)
    left = np.stack([-half + 0.5 * d * np.cos(a2), 0.5 * d * np.sin(a2)], 1)
    return np.vstack([right, left])


def sweep(pts, S, U, prof):
    """沿采样点扫一个 2D 截面（prof: (k,2)，第一列沿 S、第二列沿 U；或 callable(i)->(k,2) 变截面）→ 闭合三角网格（两端平盖）。"""
    n = len(pts)
    rings = []
    for i in range(n):
        pr = prof(i) if callable(prof) else prof
        rings.append(pts[i] + np.outer(pr[:, 0], S[i]) + np.outer(pr[:, 1], U[i]))
    k = len(rings[0])
    V = np.vstack(rings + [pts[0][None], pts[-1][None]])
    F = []
    for i in range(n - 1):
        a, b = i * k, (i + 1) * k
        for j in range(k):
            j2 = (j + 1) % k
            F += [[a + j, b + j2, b + j], [a + j, a + j2, b + j2]]   # 截面逆时针（S,U），S×U=T → 侧面这样绕才朝外
    c0, c1 = n * k, n * k + 1
    for j in range(k):
        j2 = (j + 1) % k
        F += [[c0, j2, j], [c1, (n - 1) * k + j, (n - 1) * k + j2]]
    m = trimesh.Trimesh(vertices=V, faces=np.array(F), process=True)
    if m.volume < 0: m.invert()
    return m


def polyline_len(p):
    return float(np.linalg.norm(np.diff(p, axis=0), axis=1).sum())


def min_bend_radius(p, S=None, window=2.0):
    """离散曲率半径（弦长 window 的三点圆）；返回 (全局最小 R, 最小 R 处点)；给 S 时另返回「面内弯」（扁线宽向上的曲率分量）的最小 R。"""
    d = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(p, axis=0), axis=1))]
    Rmin, at, Rin = np.inf, None, np.inf
    j = 0
    for i in range(len(p)):
        a = np.searchsorted(d, d[i] - window); b = np.searchsorted(d, d[i] + window)
        if a >= i or b >= len(p) or b <= i: continue
        A, B, C = p[a], p[i], p[b]
        ab, bc, ca = np.linalg.norm(B - A), np.linalg.norm(C - B), np.linalg.norm(A - C)
        area2 = np.linalg.norm(np.cross(B - A, C - A))
        if area2 < 1e-9: continue
        R = ab * bc * ca / (2 * area2)
        if R < Rmin: Rmin, at = R, B
        if S is not None:
            k_vec = (A + C - 2 * B) / max((ab * bc), 1e-9)          # 近似曲率向量方向
            kin = abs(np.dot(_u(k_vec), S[i])) / max(R, 1e-9)
            if kin > 1e-9: Rin = min(Rin, 1.0 / kin)
    return Rmin, at, Rin


# ════════════════════════════════════════════════════════════════════════════════════════════
# 1. 线材截面（{v, src}）
# ════════════════════════════════════════════════════════════════════════════════════════════
CABLE = {
    # 舵机原配线：扁排线三芯 4.5 × 1.5（用户卡尺 2026-09-22，harness.yaml:HB01.cross_section_mm，measured）
    "ph_servo_flat": dict(kind="flat", w=4.5, t=1.5, min_bend=3.0, src="measured（HB01.cross_section_mm）；min_bend assumed（HB01.min_bend_r_mm 未量，软线按 2×t）"),
    # 一分二 S1 两条腿 / PH 延长线：商品页没给截面 —— 按同类 26AWG 三芯并排（每芯 Ø1.1）→ 3.3 × 1.1（assumed）
    "ph_leg_3c": dict(kind="flat", w=3.3, t=1.1, min_bend=2.5, src="assumed（26AWG 三芯并排 Ø1.1；到货量）"),
    # 杜邦线（母对母彩排撕开用）：26–28AWG，外径 ≈1.2–1.4（components.yaml:cable_dupont_ff dims_assumed OD≈1.2）→ 取 1.3（assumed）
    "dupont_was_until_2026_09_25_hr43c": dict(kind="round", od=1.3, min_bend=2.0, src="assumed（components.yaml:cable_dupont_ff OD≈1.2，取 1.3 偏保守）"),
    "dupont": dict(kind="round", od=1.4, min_bend=2.0, src="hr43c：OD 1.4 = 商品页外径（用户 09-25 给，hr43c 简报）；min_bend 2.0 assumed（中心线，出壳即弯 Rc 口径）"),
    # 喇叭自带两根细线：≈28AWG Ø1.0（assumed）
    "spk_wire": dict(kind="round", od=1.0, min_bend=2.0, src="assumed（商品页未给）"),
    # UBEC 输入/输出硅胶线：UBEC-3A 商品页没给线规 —— 按 22AWG 硅胶 Ø1.6（assumed）
    "ubec_in_pair": dict(kind="pair", od=1.6, n=2, min_bend=4.0, src="assumed（22AWG 硅胶 Ø1.6，两芯并行）"),
    "ubec_out_1p": dict(kind="round", od=1.6, min_bend=4.0, src="assumed（同上，正负各一根单线 + 1P 杜邦母壳）"),
    # XT30 12 V 主干：18AWG 硅胶 Ø2.0（HB03 od 未量；18AWG 硅胶常见 1.9–2.3）→ 两芯并行跑道形
    "xt30_18awg_pair": dict(kind="pair", od=2.0, n=2, min_bend=6.0, src="assumed（18AWG 硅胶 Ø2.0；min_bend 3×od）"),
    # USB：见 USB_OPTION —— 圆线 Ø3.5（常见细 USB-C 2.0 线，assumed）或 FPC 扁线 8.0 × 0.3（assumed）
    "usb_round": dict(kind="round", od=3.5, min_bend=10.0, src="assumed（细 USB-C 线 Ø3.0–4.0；弯折半径 3×od）"),
    "usb_fpc": dict(kind="flat", w=8.0, t=0.3, min_bend=1.0, src="assumed（Type-C FPC 软排线常见 8–10 宽 0.3 厚）"),
    # 摄像头排线：22P 0.5 mm → 15P 1.0 mm 同向 150 mm（随摄像头附，components.yaml:camera_csi）。宽：15P 端 16.0、22P 端 12.0（按脚距推，assumed）；厚 0.3（assumed）
    # hr39c（09-23 用户：装机用金色 22→15 **同面**排线，白色 15→15 不用；线长没量）→ 15P 端按 1.0 间距标准宽 (15+1)×1.0 = 16.0、22P 端按 0.5 间距标准宽 (22+1)×0.5 = 11.5（都 assumed）
    "fpc_csi": dict(kind="flat", w=11.3, t=0.3, min_bend=1.0, w_head=16.0, head_len=5.0,
                    src="hr41：线身宽 11.3 measured（用户 09-23 卡尺）；摄像头端 16 宽只露出座口 ≈5 后台阶收窄（用户估，assumed）；厚 0.3 assumed；22P 端按线身 11.3（标准 11.5，差 0.2 不另建）"),
    "fpc_csi_was_until_2026_09_24_hr41": dict(kind="flat", w=16.0, w_end=11.5, t=0.3, min_bend=1.0, src="15P 1.0 间距标准宽 16.0 / 22P 0.5 间距标准宽 11.5（assumed，标准 FFC 宽）；厚 0.3 assumed"),
}


def cable_profile(cable, w_override=None):
    c = CABLE[cable]
    if c["kind"] == "round": return profile_circle(c["od"], 12)
    if c["kind"] == "pair": return profile_stadium(c["od"], c["n"])
    return profile_rect(w_override or c["w"], c["t"])


def cable_area(cable):
    c = CABLE[cable]
    if c["kind"] == "round": return math.pi * c["od"] ** 2 / 4
    if c["kind"] == "pair": return c["n"] * math.pi * c["od"] ** 2 / 4
    return c["w"] * c["t"]


# ════════════════════════════════════════════════════════════════════════════════════════════
# 2. 端点与插头（世界系零位）
# ════════════════════════════════════════════════════════════════════════════════════════════
# ── 2.1 Radxa ZERO 3W 40 针 —— **按官方机械图**（docs/design_2026-09-17_bearing_rebuild/radxa_zero3w_mech_2026-09-19.png，
#    正面图「3.3」= 板长边到排针中线；两排针孔实量离边 2.02 / 4.59；排针塑座外廓离边 0.69..5.75；「32.4」= 顶短边到排针中点）。
#    板 z 236..266（electronics.SBC_ZC 251 ± 15）→ 外排（偶数针）z 263.97、内排（奇数针）z 261.43、塑座 z 260.16..265.24。
#    **与 electronics.SBC_HDR = (257.4, 262.5) 不一致（低 2.7）** —— 那是 hr37 按「Pi Zero 排 1 离边 3.5」估的，报告 §0 列为待改。
#    pin 1 在 +y 端（图上 ▶ 标在 CSI 那一端，与 hr38 H01 缺口口径一致），奇数针在内排（Pi 规矩）。
SBC_TOP = 266.0
HDR_Z = {1: SBC_TOP - 4.57, 0: SBC_TOP - 2.03}            # 奇数针内排 / 偶数针外排（drawing）
HDR_PLASTIC = ((63.5, 66.0), (-25.4, 25.4), (SBC_TOP - 5.84, SBC_TOP - 0.76))   # 2.54 高塑座贴芯片面（x 66）朝 −x
HDR_PIN_TIP_X = 57.3                                        # 09-23 实测：针尖到板 +x 面 10.3 → 针尖 x 57.3（electronics.SBC_HDR_H 8.7；原 57.5）
DUP = dict(L=14.0, W=2.54, src="task 数据：杜邦母壳 2.54 × 2.54 × 14")
DUP_X = (HDR_PLASTIC[0][0] - DUP["L"], HDR_PLASTIC[0][0])   # 49.5..63.5：壳顶着塑座插到底


def pin_y(n): return 24.13 - 2.54 * ((n - 1) // 2)
def pin_z(n): return HDR_Z[n % 2]
def pin_exit(n): return np.array([DUP_X[0], pin_y(n), pin_z(n)])      # 线从杜邦壳尾出来的点，朝 −x


def dupont_on_pin(n):
    return obox(((DUP_X[0] + DUP_X[1]) / 2, pin_y(n), pin_z(n)), (1, 0, 0), (0, 1, 0), (DUP["L"], DUP["W"], DUP["W"]))


# 针位分配（hr39 提案 —— 原口径里 pin4/pin6 被 IMU(HB05) 与 UBEC(HB04) 同时占用，冲突；见报告 §3）
PINMAP_was_until_2026_09_25_hr43c = {
    1: ("IMU V+ (3.3 V)", "HB05"), 3: ("IMU SDA", "HB05"), 5: ("IMU SCL", "HB05"), 6: ("IMU GND", "HB05"),
    4: ("UBEC +5 V", "HB04"), 34: ("UBEC GND", "HB04"),
    2: ("功放 VIN 5 V", "HB10"), 25: ("功放 GND", "HB10"), 40: ("功放 DIN ← I²S3_SDO_M0", "HB10"),
    17: ("麦克风 VDD 3.3 V", "HB08"), 39: ("麦克风 GND", "HB08"), 9: ("麦克风 L/R → GND", "HB08"), 38: ("麦克风 SD → I²S3_SDI_M0", "HB08"),
    12: ("I²S3_SCLK_M0 一分二 A（麦 SCK + 功放 BCLK）", "HB08/HB10"), 35: ("I²S3_LRCK_M0 一分二 B（麦 WS + 功放 LRC）", "HB08/HB10"),
}
# hr43c（2026-09-25，用户 13:0x 定直插四件之「改针位 R2」，无鼓包研究.md §2.5）：两端「同 y 两排都插」拆开 —— IMU GND 6→14、麦 GND 39→20（R1，零新件），
#   IMU 3.3 V 1→17 与麦 VDD 合用（R2，多一根「杜邦一分二 母对二公」，件同 12 / 35 那根）。信号针（3/5 I²C3_M0、12/35/38/40 I²S3_M0）不动。
PINMAP = {
    3: ("IMU SDA", "HB05"), 5: ("IMU SCL", "HB05"), 14: ("IMU GND（hr43c：原 6）", "HB05"),
    4: ("UBEC +5 V", "HB04"), 34: ("UBEC GND", "HB04"),
    2: ("功放 VIN 5 V", "HB10"), 25: ("功放 GND", "HB10"), 40: ("功放 DIN ← I²S3_SDO_M0", "HB10"),
    17: ("3.3 V 一分二 C（麦 VDD + IMU V+；hr43c：IMU 原 1）", "HB08/HB05"), 20: ("麦克风 GND（hr43c：原 39）", "HB08"), 9: ("麦克风 L/R → GND", "HB08"),
    38: ("麦克风 SD → I²S3_SDI_M0", "HB08"),
    12: ("I²S3_SCLK_M0 一分二 A（麦 SCK + 功放 BCLK）", "HB08/HB10"), 35: ("I²S3_LRCK_M0 一分二 B（麦 WS + 功放 LRC）", "HB08/HB10"),
}

# ── 2.2 Radxa 板芯片面下长边的口（官方机械图正面：micro-HDMI / USB3.0 / USB2-OTG+5V IN 中心离顶短边 12.5 / 41.5 / 54.7，外伸板边 ≈1.2）
#    芯片面朝 −x → 这三个口开口朝 −z、在 z 234.8；口体高（−x 方向）按常见贴板 USB-C 3.2（assumed，到货量）。
SBC_EDGE_PORTS = {"microHDMI": 32.5 - 12.5, "USB_C2_usb3": 32.5 - 41.5, "USB_C1_otg": 32.5 - 54.7}
USBC_RCPT = dict(w=8.94, h=3.2, depth=7.6, protrude=1.2, src="w datasheet(USB-C 规范)；h 3.2 assumed(贴板式)；depth/protrude drawing 实量")


def sbc_edge_port_box(name, w=None):
    yc = SBC_EDGE_PORTS[name]; w = w or USBC_RCPT["w"]
    z0 = 236.0 - USBC_RCPT["protrude"]
    return abox((66.0 - USBC_RCPT["h"], yc - w / 2, z0), (66.0, yc + w / 2, z0 + USBC_RCPT["depth"]))


# ── 2.3 Radxa CSI 座（components.yaml:sbc_radxa_zero3w.board_faces_2026-09-23，官方图反面实量）：+x 面，x 67.6..69.1 × y 26.5..32.25 × z 242.8..259.0，
#    排线从 +y 端（y 32.25）沿 −y 插入，插入深 4（assumed）；槽中面 x 68.35
CSI_RADXA = dict(x=68.35, y_face=32.25, z=(245.0, 257.0), insert=4.0)
# ── 2.4 摄像头 FPC 座（electronics.camera_boxes：x 70.6..72.6 × y −8..8 × z 235..240，背面底边，排线朝 −z 出）
CSI_CAM_was_until_2026_09_23_flip = dict(x=71.6 + 12.07, z_face=235.0 + 6.94, y=(-8.0, 8.0), insert=3.0)   # 正装：座在背面底边、排线朝 −z 出（新硬伤 ⑦）
# hr39c-⑦（agent #4）：摄像头**倒装 180°**（electronics.CAM_FLIPPED）→ 翻盖座在背面顶边：x 79.57..84.67（高 5.1 measured）× y ±10.05（宽 20.1 measured）× z 268.94..273.94（深 5 assumed），
#   排线从座顶 z 273.94 朝 **+z** 出；槽口在座高中间（x 84.67 − 2.55 = 82.12，assumed：实物槽口离 PCB 背面多高没量）。
CSI_CAM_was_until_2026_09_24_hr41 = dict(x=84.67 - 5.1 / 2, z_face=235.0 + 6.94 + 32.0, y=(-8.0, 8.0), insert=3.0, out=+1)
# hr41：摄像头去立柱贴板、整体前移 5.9（electronics.CAM_PCB_X 90.57..92.17）→ 翻盖座 x 85.47..90.57，槽口中面 88.02；排线仍从座顶 z 273.94 朝 +z 出
CSI_CAM = dict(x=90.57 - 5.1 / 2, z_face=235.0 + 6.94 + 32.0, y=(-8.0, 8.0), insert=3.0, out=+1)
# hr41 HB06 最短走法（无波浪）：座顶竖直 6（硬补强）→ R 2.5 弯朝 −x → 转接板上方 z 282.44 横过（两道 H05 压筋 |y| 11..13 之间）→ R 2.5 弯朝 −z → 缝里 x 71.76 下到折叠三角顶
#   → 45° 折（折痕 y + z = 251）成沿 +y → 缝里 x 71.44 到 y = CSI_HAIRPIN_Y → 绕 z 轴 R 1.545 发夹弯（Radxa 端，基线就有的那个 U）→ x 68.35 沿 −y 插进 Radxa CSI 座（插入 4）。
#   CSI_HAIRPIN_Y = 34.25 是最短（座口外直段 2.0）；发夹弯两腿每加长 Δ 收 2Δ 余长（hr41 报告 §B-M5 的「单 U 最大可收长度」就是这个 Δ 的上限）。
CSI_BEND_R = 2.5
CSI_HAIRPIN_Y = 34.25
CSI_HAIRPIN_R = 1.545

# ── 2.5 宇树转接板（zz_adapter：x 54.3..63.3 × |y|≤20 × z 226.8..256.8，竖放）四个口 —— 口位 photo_scaled ±1.5（components.yaml:bus_adapter.port_positions_2026-09-22）
#    **朝向（hr39 选）**：口位基准长边 L = 下长边（z 226.8），A 边（PH + XT30 扩展口）在 −y，B 边（Type-C + XT30 主输入）在 +y。
#    四种朝向都查过（报告 §2）：L 在上时两只 XT30 插头撞 H03 槽 ±y 挡边 + Radxa 下立柱（z 237..242，冻结）；L 在下时 XT30 全清、
#    Type-C 与 PH 撞槽挡边（顶 235）—— 挡边开豁口即可，立柱不动 → 选 L 在下。A 在 −y 让 UBEC 输入线最短（UBEC 在 −y）。
#    口轴线离 PCB 面多高没量（port_axis_height_mm = null）→ 取板厚中线 x 58.8（assumed，±4）。
# hr39c：转接板挪到 Radxa 前面（head.H03_ADP_X/Y/Z = x 72.5..81.5 × y −24..16 × z 235.4..265.4），L 边仍在下；**A 边（PH + XT30 扩展）改在 +y、B 边（Type-C + XT30 主输入）在 −y**
#   （hr39c 交接 ⑥：12 V 主干从左前耳沿地板直走到 −y 边 XT30 主输入；PH 在 +y 离嘴舵机插座近）。口轴线仍取板厚中线（x 77.0，assumed ±4）。
ADP_was_until_2026_09_23_hr39c = dict(x=(54.3, 63.3), y=20.0, z=(226.8, 256.8), xc=58.8)
ADP_was_until_2026_09_25_hr43d = dict(x=(72.5, 81.5), y=(-24.0, 16.0), z=(235.4, 265.4), xc=77.0)
# hr43d（B′，agent_brief_hr43d 第 1 条）：转接板**平放**在 H08 托板上（electronics.ADP_FLAT：x 51.8..81.8 × y −20..20，PCB 264.5..266.1 + 零件 7.4 朝上）。
#   口位基准长边 L 在 −x（x 51.8）；A 边（PH + XT30 扩展）朝 +y、B 边（Type-C + XT30 主输入）朝 −y（同 hr39c/hr43c）；口中心 x = 51.8 + 离 L 边距离（photo_scaled ±1.5）。
#   插头截面元组照旧 (a, b)：竖放时 a = 沿 z（板面内）、b = 沿 x（板法线）；平放后 a = 沿 x（板面内）、b = 沿 z（板法线）。口轴离板面多高没量（port_axis_height_mm = null）
#   → 插头底贴 PCB 顶 266.1、中心 z = 266.1 + b/2（assumed，真口轴更低时插头会更靠近 H01 上立柱顶 265 / H08 竖板上横条顶 265）。
ADP = dict(x=(51.8, 81.8), y=(-20.0, 20.0), z_pcb_top=266.1, flat=True, xc=None)
ADP_PORT_was_until_2026_09_25_hr43d = "同下表数值；竖放口径（L = 下长边 z 235.4，截面 (z 向, x 向)）"
ADP_PORT = {   # name: (边 ±y, 离 L 边距离, 插头截面 (竖放：z 向, x 向 / hr43d 平放：x 向, z 向), 插头伸出板边长度, src)
    "PH":      (+1, 6.0, (7.8, 4.5), 6.85, "PHR-3 壳 7.8×4.5×6.85（JST 目录，HB01.plug_envelope_mm）"),
    "XT30ext": (+1, 19.9, (9.5, 5.0), 13.0, "XT30U-F 插合面 8.9×3.8（商品页标称）+ 热缩 → 9.5×5.0；伸出 13（assumed，含焊杯热缩）"),
    "TypeC":   (-1, 8.1, (8.9, 3.5), 5.0, "按 FPC 式超薄弯头插头 8.9×3.5×5（assumed；常规弯头 12×6.5×8.5 放不下，见报告）"),
    "XT30in":  (-1, 22.6, (9.5, 5.0), 13.0, "同 XT30ext"),
}


def adp_port(name):
    if ADP.get("flat"):                                  # hr43d：平放（见 ADP 注释）
        side, d, (wx, wz), ln, _ = ADP_PORT[name]
        xc = ADP["x"][0] + d; zc = ADP["z_pcb_top"] + wz / 2; y0 = ADP["y"][1] if side > 0 else ADP["y"][0]
        return dict(center=np.array([xc, y0, zc]), out=np.array([0, side, 0.0]), wz=wz, wx=wx, ln=ln, exit=np.array([xc, y0 + side * ln, zc]))
    side, d, (wz, wx), ln, _ = ADP_PORT[name]
    zc = ADP["z"][0] + d
    y0 = ADP["y"][1] if side > 0 else ADP["y"][0]      # hr39c：板不再关于 y=0 对称
    return dict(center=np.array([ADP["xc"], y0, zc]), out=np.array([0, side, 0.0]), wz=wz, wx=wx, ln=ln,
                exit=np.array([ADP["xc"], y0 + side * ln, zc]))


def adp_plug_box(name):
    p = adp_port(name)
    c = p["center"] + p["out"] * p["ln"] / 2
    return obox(c, (0, 1, 0), (1, 0, 0), (p["ln"], p["wx"], p["wz"]))


# ── 2.6 UBEC-3A（zz_ubec：x 72.3..78.3 × y −31.7..−19.7 × z 233.0..259.0，立放）。两端出线：输入（XT30 母头 10 cm）/ 输出（1P×2 10 cm）
#    **哪头出输入、哪头出输出、是不是两头出线 —— 商品页没说（assumed：常见 UBEC 两端各出一组）**。立放后底端贴 H03-F13 台面（z 233.0），底端没有出线空间。
UBEC_was_until_2026_09_23_hr39c = dict(lo=(72.3, -31.7, 233.0), hi=(78.3, -19.7, 259.0))
UBEC_was_until_2026_09_23_raise = dict(lo=(84.37, -37.29, 235.46), hi=(90.37, -25.29, 261.46))
UBEC = dict(lo=(84.37, -37.29, 240.46), hi=(90.37, -25.29, 266.46))     # hr39c：= electronics.UBEC_BOX（平移 T_UBEC + agent #4 两头出线抬高 5）
# ── 2.7 麦克风（electronics.mic_boxes）：两排 3 针 y 25.27 / 32.73，x 73.06 / 75.6 / 78.14；杜邦壳顶 z 252.3，线朝 +z 出
MIC_PIN_X = (73.06 + 12.07, 75.6 + 12.07, 78.14 + 12.07); MIC_ROW_Y = (25.27 + 6.31, 32.73 + 6.31); MIC_DUP_TOP = 252.3 + 2.46   # hr39c：+ head.T_MIC (12.07, 6.31, 2.46)
# ── 2.8 功放 MAX98357A（**按协调员 09-23 更正**：90° 弯针塑座贴板边外 y −11.6..−9.0、针再往 −y 伸 ≈6；杜邦壳顶着塑座前面插 → 壳 y −25.6..−11.6；
#    弯针在元件面（板上方 z 254.1 起），针轴 z = 254.1 + 1.27 = 255.37；7 针 x −7.62..7.62）。electronics.amp_boxes 里壳画在 y −23..−9，少伸 2.6 —— 报告里写 zz_amp 改法。
# hr39c：功放卡座挂在 H05 后脑内面、跟顶壳走 → 整组 + electronics.T_AMP (−6.03, 0, 11.0)
_TA = (-6.03, 0.0, 11.0)
AMP = dict(board=((-10.0 + _TA[0], 10.0 + _TA[0]), (-9.0, 9.0), (252.5 + _TA[2], 254.1 + _TA[2])), hdr_y=(-11.6, -9.0), pin_z=254.1 + _TA[2] + 1.27, dup_y=(-25.6, -11.6),
           pins_x=[v + _TA[0] for v in (-7.62, -5.08, -2.54, 0.0, 2.54, 5.08, 7.62)], pins=["LRC", "BCLK", "DIN", "GAIN", "SD", "GND", "VIN"],
           term=((-3.8 + _TA[0], 3.8 + _TA[0]), (3.5, 9.0), (254.1 + _TA[2], 262.6 + _TA[2])), term_wire_z=256.6 + _TA[2], term_pitch=3.5)
# pins 顺序照 Adafruit MAX98357A 丝印（LRC BCLK DIN GAIN SD GND Vin）；淘宝通用板**顺序与哪头在 x− 未核**（assumed）—— 只影响哪根线在束里的位置，不影响束外包。
# ── 2.9 喇叭（electronics.speaker_boxes：中心 (50, 2.5)，z 264.5..269.5，背面焊片凸台 x 47..53 × y 0.5..4.5 × z 263.0..264.5）
SPK = dict(c=(55.18, 3.06), z0=276.75, bump=((52.18, 58.18), (1.06, 5.06), (275.25, 276.75)))   # hr39c：+ head_top.T_SPK，喇叭底 276.75（electronics.speaker_boxes 实算）
# ── 2.10 头横滚 8 号（servo__jaw_soft_jaw_soft）下插座（入口，+y 局部）：插头顶 x −17.7（与厚段背面齐平）、中面 (y −9.3, z 227.865)，
#    PHR-3 体沿 x 插进 6.85；线从插头顶沿 −x 直立翘 4（KO01 wire_out=4.0，用户实物）再弯。harness.yaml:HB01 branches F_head
ROLL_IN = dict(top_x=-17.7, y=-9.3, z=227.865)
# ── 2.11 出头口：头腔底开口（H03 底板在 z ≈219.5..221 那一圈开口，N03 从这里穿下去）里避开 横滚/偏航/俯仰 单关节扫掠的两块「前耳」：
#    左前 x 36..46 × y −28..−14、右前 x 36..46 × y 14..28（route_search exitzone；hr39 cache/slice_z_220_221_222_223.png）
EXIT_L = np.array([41.0, -20.0, 218.5]); EXIT_R = np.array([41.0, 20.0, 218.5])     # hr39c：H03 颈口用原版核心（H03_CORE_*），出头口位置不变
# ── 2.11b 嘴舵机 14 号（hr39c：ψ5、嘴轴 (31.75, 245.5)、法兰面 y 47.75；jaw.servo_R）两个插座：背插，插头顶面 = 厚段背面 y 21.75（jaw.JAW_REAR_Y），
#    线从插头顶沿舵机局部 −x = **世界 −y** 直立翘 4.4（到 y 17.35，KO01 wire_out 4.0 + 0.4）再弯（交接：「嘴两根线先朝 +x」）。中面由 s288.S 的 conn_z / plug_y_in 经 servo_R 实算：
#    lo = 局部 +y 口（世界下口），hi = 局部 −y 口（世界上口）。两口都 free（lib.CONN_MODE[("jaw_soft", 0)]）。
JAW_PLUG = {"lo": np.array([41.69, 21.75, 238.59]), "hi": np.array([40.34, 21.75, 254.03])}
JAW_WIRE_RISE = 4.4
PLUG_BODY = (6.85, 7.8, 4.5)                       # PHR-3：拔插向 × 针排向 × 厚（s288.S plug_body 的 x/z/y）
JAW_PIN_ROW = (0.996, 0.0, 0.087)                  # 嘴舵机插头针排向（世界）= servo_R 的局部 z（取反，盒子对称）

# ── 2.12 连接器实体（线的一部分：插头/杜邦壳/转接块；ref__ 开头的是「被插的那一侧」按官方图补的实体，只用来查间隙，不是线）
USB_PLUG_SLIM = dict(w=8.9, t=3.5, h=4.0, src="assumed：Type-C FPC 式超薄直角公头（壳 8.3×2.5 + 注塑 ≤8.9×3.5，口面以下 4.0）；常规弯头 12×6.5×8 头部放不下")
S1_JUNCTION = dict(size=(14.0, 8.0, 5.0), src="assumed：PH2.0 3P 线端针座（一分二「母=针头」）+ 插上的 PHR-3 壳，长 14 × 宽 8 × 厚 5")


def connector_solids_was_until_2026_09_25_hr43():
    """hr39c..hr41 版（Radxa 竖放在 H01 立柱位时的针座 / 杜邦壳 / 口 / 功放弯针）。hr43 起用文件末尾的 connector_solids()。"""
    out = {}
    (x0, x1), (y0, y1), (z0, z1) = HDR_PLASTIC
    out["ref__sbc_header_drawing"] = abox((x0, y0, z0), (x1, y1, z1))
    for n in sorted(PINMAP_was_until_2026_09_25_hr43c):
        out[f"conn__dup_pin{n:02d}"] = dupont_on_pin(n)
    for nm in SBC_EDGE_PORTS:
        out[f"ref__sbc_{nm}"] = sbc_edge_port_box(nm)
    out["ref__sbc_csi_conn"] = abox((67.6, 26.5, 242.8), (69.1, 32.25, 259.0))
    # USB 插头（Radxa 侧，插在 USB-C2 口里、口面 z 234.8 以下）
    yc = SBC_EDGE_PORTS["USB_C2_usb3"]; xc = 66.0 - USBC_RCPT["h"] / 2; z0p = 236.0 - USBC_RCPT["protrude"]
    out["conn__usb_plug_radxa"] = abox((xc - USB_PLUG_SLIM["t"] / 2, yc - USB_PLUG_SLIM["w"] / 2, z0p - USB_PLUG_SLIM["h"]),
                                      (xc + USB_PLUG_SLIM["t"] / 2, yc + USB_PLUG_SLIM["w"] / 2, z0p))
    for nm in ADP_PORT:
        out[f"conn__adp_{nm}"] = adp_plug_box(nm)
    # 功放：按实物更正的弯针塑座 + 5 个用到的杜邦母壳（GAIN/SD 不接）
    (bx0, bx1), _, (bz0, bz1) = AMP["board"]
    out["ref__amp_rightangle_header"] = abox((-8.89 + _TA[0], AMP["hdr_y"][0], bz1), (8.89 + _TA[0], AMP["hdr_y"][1], bz1 + 2.54))
    for x, nm in zip(AMP["pins_x"], AMP["pins"]):
        if nm in ("GAIN", "SD"): continue
        out[f"conn__amp_dup_{nm}"] = abox((x - 1.27, AMP["dup_y"][0], AMP["pin_z"] - 1.27), (x + 1.27, AMP["dup_y"][1], AMP["pin_z"] + 1.27))
    # hr39c-2（agent #3）：三个用到的舵机插座上插着的 PHR-3 插头本体（背插，插头顶面 = 厚段背面；s288.S plug_body：拔插向 6.85 × 针排向 7.8 × 厚 4.5）。
    #   舵机 STL 在插座处是空的（缺口），不补这块线会从插头里穿过去（hr39c 第一遍 bus_jaw 就从嘴上插头里穿了）。针排向：嘴舵机 = 舵机局部 z = 世界 ≈x（servo_R 实算 (−0.996, 0, −0.087)）；
    #   8 号 = 世界 y（同 hr39 sides=Yv 口径）。与它插的舵机「预期接触」（check_wires EXPECTED）。
    for k, p in JAW_PLUG.items():
        out[f"conn__jaw_plug_{k}"] = obox(p + np.array([0.0, PLUG_BODY[0] / 2, 0.0]), (0, 1, 0), JAW_PIN_ROW, PLUG_BODY)
    out["conn__roll8_plug_lo"] = obox((ROLL_IN["top_x"] + PLUG_BODY[0] / 2, ROLL_IN["y"], ROLL_IN["z"]), (1, 0, 0), (0, 1, 0), PLUG_BODY)
    for k, (c, ax) in enumerate(S1_JUNCTIONS_AT):
        out[f"conn__s1_junction{k + 1}"] = obox(c, ax, (0, 0, 1) if abs(ax[2]) < 0.9 else (0, 1, 0), S1_JUNCTION["size"])   # 竖放时 8 宽沿 y、5 厚沿 x
    return out


S1_JUNCTIONS_AT = []    # 由 _define() 填：[(中心, 轴向)]


def _tri_prism_x(tri_yz, x0, x1):
    """沿 x 拉伸的三角柱（FPC 45° 折叠处的双层三角区；tri_yz = 三个 (y, z) 顶点）"""
    V = [(x, y, z) for x in (x0, x1) for (y, z) in tri_yz]
    F = [(0, 2, 1), (3, 4, 5), (0, 1, 4), (0, 4, 3), (1, 2, 5), (1, 5, 4), (2, 0, 3), (2, 3, 5)]
    m = trimesh.Trimesh(vertices=V, faces=F, process=True)
    if m.volume < 0: m.invert()
    return m
STOWS = {}              # 由 _define() 填：{名字: dict(lo, hi, wires, excess_mm, note)}


def stow_solids():
    out = {}
    for k, v in STOWS.items():
        out[f"stow__{k}"] = trimesh.util.concatenate([abox(lo, hi) for lo, hi in v["boxes"]])
    return out


# ════════════════════════════════════════════════════════════════════════════════════════════
# 3. 线路 —— 见 WIRES（航点由 hr39_wires/route_search.py 找走廊后人工定稿；每条线的 note 里写为什么这么走）
# ════════════════════════════════════════════════════════════════════════════════════════════
WIRES = []    # 由 _define() 填（本文件末尾）


def _w(**kw):
    WIRES.append(kw)


# ════════════════════════════════════════════════════════════════════════════════════════════
# 4. 生成
# ════════════════════════════════════════════════════════════════════════════════════════════
def build_wire(w, step=0.5):
    pts, marks = catmull_rom(w["pts"], step)
    T, S, U = frames(pts, w.get("sides"), marks)
    c = CABLE[w["cable"]]
    if c["kind"] == "flat" and "w_head" in c and w.get("wide_head"):
        # hr41：头段台阶加宽（CSI 摄像头端 16 宽露出 ≈5，然后突变到线身 11.3）
        d = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(pts, axis=0), axis=1))]
        def prof(i):
            return profile_rect(c["w_head"] if d[i] <= c["head_len"] else c["w"], c["t"])
        m = sweep(pts, S, U, prof)
    elif c["kind"] == "flat" and "w_end" in c and w.get("taper_at") is not None:
        # FPC 宽度渐变：从 taper_at（弧长，mm）起在 taper_len 内由 w 线性变到 w_end
        d = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(pts, axis=0), axis=1))]
        a, L = w["taper_at"], w.get("taper_len", 10.0)
        def prof(i):
            ww = c["w"] + (c["w_end"] - c["w"]) * np.clip((d[i] - a) / L, 0, 1)
            return profile_rect(ww, c["t"])
        m = sweep(pts, S, U, prof)
    else:
        m = sweep(pts, S, U, cable_profile(w["cable"]))
    return m, pts, S


def build(only=None):
    """返回 (meshes, meta)。meshes: {名字: trimesh}；meta: {线名: {...}}。"""
    _define()
    meshes, meta = {}, {}
    for w in WIRES:
        if only and w["id"] not in only: continue
        m, pts, S = build_wire(w)
        L = polyline_len(pts)
        c = CABLE[w["cable"]]
        R, at, Rin = min_bend_radius(pts, S if c["kind"] == "flat" else None)
        meshes["wire__" + w["id"]] = m
        meta[w["id"]] = dict(harness=w.get("harness"), cable=w["cable"], cable_src=c["src"], length_mm=round(L, 1),
                             bought_mm=w.get("bought"), bought_src=w.get("bought_src"), fixed_mm=w.get("fixed_mm", 0.0),
                             min_bend_R=round(float(R), 2), min_bend_at=None if at is None else [round(float(v), 1) for v in at],
                             min_bend_limit=c["min_bend"], inplane_R=None if not np.isfinite(Rin) else round(float(Rin), 1),
                             volume_mm3=round(float(m.volume), 1), area_mm2=round(cable_area(w["cable"]), 2),
                             ends=w.get("ends"), anchors=w.get("anchors", []), group=w.get("group"), note=w.get("note", ""),
                             creases=w.get("creases"), crease_at=w.get("crease_at"),
                             waypoints=[[round(float(v), 2) for v in p] for p in w["pts"]])
        for k, pb in enumerate(w.get("plugs", [])):
            meshes[f"plug__{w['id']}__{k}"] = pb
    for name, m in connector_solids().items():
        meshes[name] = m
    for name, m in stow_solids().items():
        meshes[name] = m
    for name, m in tie_solids().items():                     # hr44：束上的扎带环（束本身的，不是件上的扎带座）
        meshes[name] = m
    return meshes, meta


def export(outdir):
    os.makedirs(outdir, exist_ok=True)
    meshes, meta = build()
    for n, m in meshes.items():
        m.export(os.path.join(outdir, f"{n}.stl"))
    with open(os.path.join(outdir, "wires_meta.json"), "w") as f:
        json.dump(meta, f, ensure_ascii=False, indent=1)
    return meshes, meta


# ════════════════════════════════════════════════════════════════════════════════════════════
# 5. 线路定义（航点 + 元数据）
# ════════════════════════════════════════════════════════════════════════════════════════════
# ── 航点 = design_routes.py / reroute.py 体素最短可行路 → relax_routes.py + relax2.py 沿距离场梯度推离（件：目标线面净空 2.0；线与线：只消除互穿）→ 每 2.5 mm 取点。
#    mode：hard = 体素搜索全程净空 ≥ r；hard_thin = 要把 r 降到 0.6r 才有路；soft = 硬约束无路。snap = 端点离最近可行体素（≥0.9 = 接头处就挤）。relax_min_clear = 推完后到件的最小线面净空（体素估，±0.3）。
_ROUTES = {
    "bus_ext": dict(mode="hard", r=1.0, snap=(0.38, 0.87), L_search=68.4, relax_min_clear=0.69,
              pts=[(76.40, 22.85, 241.40), (75.97, 25.27, 241.70), (74.53, 26.85, 240.45), (73.01, 28.27, 239.10), (71.42, 29.52, 237.72), (69.75, 30.47, 236.30), (67.80, 30.87, 235.11), (65.60, 30.43, 235.05), (63.98, 28.91, 235.34), (62.54, 26.93, 235.41), (61.05, 24.88, 235.41), (59.29, 23.32, 235.35), (57.31, 22.10, 235.19), (55.29, 21.05, 234.47), (53.99, 20.25, 232.58), (52.50, 19.78, 230.68), (50.52, 19.62, 230.48), (48.52, 19.61, 230.47), (46.03, 19.75, 230.42), (43.56, 20.00, 230.32), (41.59, 20.09, 229.94), (40.02, 20.19, 228.92), (39.78, 20.30, 226.86), (39.61, 20.40, 224.61), (39.81, 20.60, 222.23), (40.02, 20.82, 220.04), (40.04, 21.18, 217.60), (41.00, 22.00, 215.50)]),
    # hr39c-⑦（agent #4）：bus_jaw 前段手改 —— 原路线从 PH 口斜穿 Radxa +y 端（转接块中心 (71.9, 33.8, 248.2)）正好占着 CSI 座口外（排线必须从 y > 32.5 板外沿 −y 插进座）→
    #   改成出 PH 口沿 +y 直走（转接块在这段）到 y ≈42 再贴侧壁往 −x 爬到 (66.25, 38.48, 253.8) 接回原路线；原前 10 个航点见 cache/wiring_head.before_agent4.py。
    "bus_jaw": dict(mode="hard", r=1.0, snap=(0.38, 0.36), L_search=102.4, relax_min_clear=1.7,
              pts=[(77.60, 22.85, 241.40), (77.75, 25.30, 241.60), (77.80, 27.80, 241.90), (77.80, 31.00, 242.10), (77.80, 35.00, 242.30), (77.80, 39.00, 242.50), (77.70, 42.00, 242.80), (76.90, 43.40, 244.10), (74.50, 43.60, 246.00), (71.50, 42.90, 248.00), (68.90, 41.30, 250.20), (67.30, 39.70, 252.20), (66.25, 38.48, 253.80), (64.49, 38.60, 255.56), (62.72, 38.62, 257.31), (60.94, 38.57, 259.07), (59.17, 38.39, 260.81), (57.51, 38.13, 261.85), (55.10, 37.62, 262.05), (52.75, 36.87, 262.07), (50.50, 35.86, 262.03), (48.35, 34.62, 261.92), (46.67, 33.50, 261.79), (45.06, 32.23, 261.63), (44.52, 29.85, 261.39), (44.30, 27.86, 261.14), (43.04, 26.39, 260.80), (41.02, 25.09, 260.19), (39.26, 23.70, 259.29), (37.88, 22.17, 258.11), (36.94, 20.60, 256.57), (36.36, 19.90, 254.41), (36.25, 19.10, 252.40), (36.65, 17.63, 250.56), (37.03, 17.77, 248.43), (38.15, 18.90, 246.56), (39.59, 19.08, 244.56), (40.69, 19.23, 242.38), (41.59, 19.44, 240.10), (41.73, 21.26, 238.69), (41.69, 21.75, 238.59)]),
    "bus_jaw_roll": dict(mode="hard", r=1.0, snap=(1.92, 1.85), L_search=108.7, relax_min_clear=1.98,
              pts=[(40.34, 21.75, 254.03), (40.55, 18.77, 254.13), (39.32, 16.48, 255.09), (37.31, 15.07, 256.18), (34.66, 14.61, 257.09), (31.75, 14.60, 257.64), (28.78, 14.70, 257.91), (25.80, 14.84, 258.03), (22.81, 14.98, 258.07), (19.82, 15.12, 258.07), (16.83, 15.24, 258.05), (13.84, 15.35, 258.00), (10.85, 15.44, 257.94), (7.86, 15.49, 257.87), (4.87, 15.50, 257.79), (1.88, 15.42, 257.71), (-1.09, 15.19, 257.63), (-3.99, 14.62, 257.57), (-6.68, 13.49, 257.44), (-9.05, 11.82, 257.04), (-11.12, 9.90, 256.18), (-13.01, 8.00, 254.91), (-14.80, 6.20, 253.34), (-16.47, 4.51, 251.55), (-17.97, 2.91, 249.53), (-19.15, 1.44, 247.24), (-19.92, 1.07, 244.40), (-20.31, 0.71, 241.48), (-20.50, -0.01, 238.64), (-20.86, -1.35, 236.10), (-21.67, -3.12, 233.93), (-22.74, -5.05, 232.02), (-23.13, -7.04, 230.33), (-21.97, -8.73, 229.09), (-19.63, -9.50, 228.05), (-17.70, -9.30, 227.87)]),
    "imu_pin1": dict(mode="hard", r=0.8, snap=(0.50, 0.38), L_search=72.6, relax_min_clear=1.38,
              pts=[(38.80, 17.03, 215.42), (38.46, 17.62, 217.81), (38.17, 18.17, 220.21), (37.94, 18.64, 222.63), (37.76, 19.00, 225.08), (37.58, 19.23, 227.54), (37.36, 19.32, 230.01), (37.06, 19.30, 232.47), (36.70, 19.20, 234.92), (36.28, 19.07, 237.36), (35.84, 18.95, 239.81), (35.40, 18.87, 242.24), (34.96, 18.80, 244.67), (34.51, 18.58, 247.09), (34.04, 17.90, 249.37), (33.65, 17.76, 251.78), (34.20, 17.98, 254.10), (35.11, 18.37, 256.23), (36.46, 18.92, 258.00), (38.22, 19.62, 259.31), (40.24, 20.50, 260.19), (42.36, 21.55, 260.75), (44.50, 22.73, 261.12), (46.62, 23.96, 261.42), (49.03, 24.30, 261.46), (49.50, 24.13, 261.43)]),
    "imu_pin3": dict(mode="hard", r=0.8, snap=(0.64, 0.11), L_search=91.6, relax_min_clear=1.97,
              pts=[(40.54, 17.55, 215.43), (39.05, 18.90, 216.18), (38.26, 19.82, 218.06), (37.85, 20.69, 220.24), (37.40, 21.40, 222.51), (36.78, 21.98, 224.78), (35.85, 22.42, 226.95), (34.58, 22.62, 228.97), (33.05, 22.51, 230.83), (31.40, 22.12, 232.59), (29.70, 21.54, 234.29), (28.00, 20.93, 235.98), (26.32, 20.41, 237.72), (24.70, 20.05, 239.53), (23.20, 19.80, 241.46), (21.89, 19.52, 243.50), (20.78, 19.06, 245.59), (19.92, 18.29, 247.69), (19.43, 17.91, 249.95), (19.57, 18.00, 252.21), (20.41, 18.10, 254.32), (21.78, 18.25, 256.22), (23.48, 18.41, 257.96), (25.34, 18.35, 259.59), (27.31, 18.32, 261.07), (29.40, 18.32, 262.37), (31.62, 18.34, 263.41), (33.94, 18.37, 264.14), (36.33, 18.46, 264.52), (38.72, 18.68, 264.50), (41.05, 19.11, 264.11), (43.26, 19.76, 263.40), (45.38, 20.59, 262.49), (47.54, 21.41, 261.62), (49.50, 21.59, 261.43)]),
    "imu_pin5": dict(mode="hard", r=0.8, snap=(0.54, 0.46), L_search=103.4, relax_min_clear=1.43,
              pts=[(42.30, 17.50, 215.50), (41.72, 15.96, 217.38), (41.31, 15.93, 219.77), (41.13, 16.08, 222.21), (41.34, 15.98, 224.65), (41.71, 15.81, 227.08), (41.72, 15.96, 229.46), (40.89, 16.40, 231.49), (39.10, 16.68, 232.84), (36.79, 16.66, 233.61), (34.35, 16.54, 234.07), (31.87, 16.43, 234.38), (29.39, 16.39, 234.65), (26.94, 16.43, 235.06), (24.60, 16.56, 235.82), (22.48, 16.82, 237.01), (20.77, 17.27, 238.66), (19.71, 18.01, 240.67), (19.26, 18.97, 242.85), (18.92, 19.64, 245.01), (18.20, 19.19, 246.93), (17.43, 18.11, 248.44), (16.71, 17.88, 250.43), (16.34, 17.67, 252.73), (17.01, 17.79, 254.87), (18.45, 18.24, 256.78), (19.87, 18.42, 258.19), (21.39, 18.02, 259.54), (22.95, 17.08, 260.42), (24.59, 15.98, 260.84), (26.78, 14.90, 261.28), (29.12, 14.18, 261.55), (31.57, 13.80, 261.67), (34.05, 13.72, 261.68), (36.53, 13.92, 261.64), (38.95, 14.46, 261.59), (41.23, 15.40, 261.55), (43.37, 16.66, 261.52), (45.41, 18.09, 261.50), (47.55, 19.36, 261.48), (49.50, 19.05, 261.43)]),
    "imu_pin6": dict(mode="hard", r=0.8, snap=(0.54, 1.19), L_search=115.1, relax_min_clear=1.6,
              pts=[(44.14, 17.56, 215.44), (43.37, 19.57, 216.54), (42.41, 21.45, 217.67), (41.18, 23.18, 218.85), (39.70, 23.51, 220.17), (38.49, 23.68, 221.80), (37.01, 24.04, 223.77), (35.41, 24.54, 225.56), (33.14, 24.86, 226.43), (30.80, 25.18, 227.11), (28.41, 25.49, 227.66), (26.06, 25.85, 228.24), (23.85, 26.36, 229.03), (21.94, 27.13, 230.12), (20.35, 28.23, 231.41), (19.58, 29.66, 233.12), (19.07, 31.32, 234.81), (18.66, 33.02, 236.51), (18.26, 34.54, 238.35), (17.83, 35.71, 240.40), (17.42, 36.47, 242.64), (17.09, 36.87, 245.03), (16.88, 36.99, 247.47), (16.86, 36.86, 249.92), (17.10, 36.52, 252.31), (17.62, 35.99, 254.61), (18.38, 35.31, 256.81), (20.32, 34.53, 258.01), (22.46, 33.69, 258.85), (24.62, 32.73, 259.45), (26.75, 31.58, 259.88), (28.73, 30.21, 260.30), (30.55, 28.66, 260.83), (32.25, 27.05, 261.56), (33.95, 25.54, 262.47), (35.77, 24.24, 263.40), (37.79, 23.18, 264.18), (39.97, 22.27, 264.66), (42.21, 21.36, 264.82), (44.41, 20.36, 264.75), (46.53, 19.29, 264.16), (49.00, 19.10, 263.97), (49.50, 19.05, 263.97)]),
    # hr39c（agent #4）：Radxa −y 端外那段（航点 19..25）整体再往 −y 让 0.5..1.9 —— 躲新加的 TF 卡包络（y 到 −34.5，x 67.6..70.6，z 243..259）
    "xt30_12v": dict(mode="hard", r=1.2, snap=(0.50, 0.87), L_search=72.8, relax_min_clear=1.28,
              pts=[(42.50, -23.50, 215.50), (41.30, -21.86, 216.95), (40.85, -20.58, 218.66), (40.89, -19.91, 220.91), (41.86, -20.55, 222.53), (43.12, -21.84, 224.05), (44.53, -23.44, 225.06), (46.12, -25.14, 225.88), (47.69, -26.94, 226.61), (49.13, -28.80, 227.36), (50.36, -30.61, 228.36), (51.16, -32.45, 229.78), (51.58, -33.15, 231.76), (52.01, -33.44, 233.79), (52.93, -33.89, 235.91), (54.50, -34.26, 237.63), (56.38, -34.43, 239.22), (58.27, -34.45, 240.84), (60.14, -34.46, 242.49), (62.01, -35.12, 244.13), (63.87, -36.07, 245.75), (65.73, -37.10, 247.35), (67.57, -37.77, 248.96), (69.31, -38.16, 250.57), (70.90, -38.50, 252.16), (72.41, -39.08, 253.73), (74.03, -39.60, 255.29), (75.76, -39.61, 256.84), (76.99, -37.99, 257.99), (77.00, -37.00, 258.00)]),
    # hr39c（agent #4）：UBEC 两头出线（商品页产品图）→ 模块抬高 5，**输入走底端**（模型选这个朝向装；输出走顶端）：底面中心朝 −z 出 → R≈3 弯朝 +y → 从座 +y 墙口出 →
    #   贴地板（z ≈237.3，摄像头 PCB 底 241.94 之下）沿 +y 到 y ≈24 → 竖起（麦克风台 y 29.5 之前）→ 翻到 XT30 扩展口插头尾（A 边 +y，y 29）。原顶端出线版见 cache/wiring_head.before_agent4.py。
    "ubec_in": dict(mode="manual", r=1.0, snap=(0.0, 0.0), L_search=0.0, relax_min_clear=None,
              pts=[(87.37, -31.29, 240.44), (87.37, -31.29, 239.2), (87.37, -30.55, 238.0), (87.37, -29.1, 237.35), (87.30, -26.5, 237.3), (87.10, -20.0, 237.3), (86.80, -10.0, 237.3), (86.60, 0.0, 237.3), (86.50, 10.0, 237.3), (86.40, 18.0, 237.4), (86.10, 23.0, 238.2), (85.40, 25.4, 240.8), (84.60, 26.6, 244.5), (83.80, 27.4, 248.5), (82.70, 28.5, 251.6), (81.30, 30.2, 254.2), (80.20, 32.3, 255.3), (78.90, 33.3, 255.3), (77.60, 32.4, 255.3), (77.05, 30.6, 255.3), (77.00, 29.0, 255.3)]),
    "ubec_out_5v": dict(mode="hard", r=0.9, snap=(0.56, 0.48), L_search=80.0, relax_min_clear=1.35,
              pts=[(87.37, -31.79, 266.46), (87.30, -31.70, 268.4), (86.40, -29.90, 269.4), (84.98, -28.04, 269.4), (83.30, -26.69, 269.0), (81.51, -25.52, 268.6), (79.78, -24.02, 268.16), (78.02, -22.33, 268.54), (76.23, -20.61, 268.67), (74.48, -18.84, 268.67), (72.82, -16.99, 268.61), (71.41, -14.98, 268.56), (70.53, -12.72, 268.53), (70.14, -10.29, 268.53), (69.78, -7.85, 268.56), (69.03, -5.52, 268.63), (67.83, -3.37, 268.72), (66.31, -1.41, 268.78), (64.62, 0.42, 268.82), (62.84, 2.17, 268.84), (61.02, 3.87, 268.88), (59.17, 5.54, 268.91), (57.31, 7.20, 268.92), (55.46, 8.87, 268.87), (53.64, 10.57, 268.71), (51.89, 12.31, 268.42), (50.23, 14.08, 267.92), (48.73, 15.94, 267.25), (47.64, 17.97, 266.51), (47.30, 20.22, 265.63), (47.27, 21.26, 263.97), (49.50, 21.59, 263.97)]),
    "ubec_out_gnd": dict(mode="hard", r=0.9, snap=(0.49, 0.47), L_search=59.0, relax_min_clear=1.6,
              pts=[(87.37, -34.39, 266.46), (87.00, -34.49, 268.6), (85.60, -35.20, 269.3), (83.87, -35.55, 268.7), (80.44, -35.32, 267.0), (77.49, -34.74, 265.88), (74.92, -34.78, 265.80), (72.00, -34.89, 265.54), (69.03, -34.93, 265.37), (66.10, -34.75, 265.48), (63.34, -33.96, 265.89), (60.89, -32.62, 266.68), (58.71, -30.84, 267.58), (56.61, -28.77, 267.95), (54.49, -26.69, 268.12), (52.38, -24.59, 268.11), (50.30, -22.48, 267.91), (48.31, -20.36, 267.37), (46.94, -18.45, 265.58), (47.05, -16.59, 263.71), (49.50, -16.51, 263.97)]),
    "mic_pin38": dict(mode="hard", r=0.8, snap=(0.46, 0.48), L_search=87.8, relax_min_clear=1.98,
              pts=[(85.13, 31.58, 254.76), (85.28, 30.16, 257.14), (85.20, 27.21, 257.27), (84.60, 24.65, 257.85), (83.32, 22.82, 259.39), (81.59, 21.79, 261.45), (79.39, 21.05, 263.28), (77.25, 20.02, 265.03), (75.30, 18.53, 266.66), (73.31, 16.77, 267.95), (71.19, 14.76, 268.40), (69.64, 12.23, 268.59), (68.05, 9.74, 268.83), (66.28, 7.48, 269.47), (64.43, 5.45, 270.55), (62.50, 3.49, 271.57), (60.47, 1.51, 271.86), (58.45, -0.40, 271.17), (56.57, -2.18, 269.78), (54.87, -3.98, 268.15), (53.32, -6.04, 266.74), (51.79, -8.36, 265.78), (50.08, -10.64, 265.02), (48.19, -12.72, 264.06), (46.64, -14.97, 262.91), (46.09, -17.73, 262.70), (46.38, -20.48, 263.48), (48.51, -21.61, 263.98), (49.50, -21.59, 263.97)]),
    "mic_pin17": dict(mode="hard", r=0.8, snap=(0.32, 0.32), L_search=66.9, relax_min_clear=1.97,
              pts=[(87.67, 31.58, 254.76), (87.07, 31.86, 257.65), (84.89, 32.42, 259.53), (82.68, 32.30, 261.33), (80.52, 31.19, 262.76), (78.42, 29.38, 263.62), (76.34, 27.32, 264.16), (74.32, 25.25, 264.83), (72.36, 23.26, 265.86), (70.45, 21.34, 267.11), (68.55, 19.38, 268.32), (66.43, 17.32, 268.77), (64.32, 15.22, 268.87), (62.16, 13.17, 268.67), (59.93, 11.25, 268.23), (57.57, 9.51, 267.68), (55.21, 8.08, 266.65), (52.76, 7.26, 265.33), (49.99, 6.65, 264.61), (47.43, 5.58, 263.72), (47.24, 3.68, 261.60), (49.50, 3.81, 261.43)]),
    "mic_pin39": dict(mode="hard", r=0.8, snap=(0.40, 0.38), L_search=95.9, relax_min_clear=1.96,
              pts=[(90.21, 31.58, 254.76), (89.81, 31.56, 257.71), (87.88, 31.06, 259.95), (86.00, 30.24, 262.03), (84.12, 28.95, 263.88), (82.31, 27.17, 265.35), (80.55, 25.08, 266.49), (78.80, 22.97, 267.63), (77.06, 21.04, 269.05), (75.27, 19.22, 270.56), (73.39, 17.29, 271.74), (71.49, 15.14, 272.33), (70.35, 12.50, 272.50), (70.24, 9.53, 272.52), (69.69, 6.66, 272.52), (68.45, 4.02, 272.54), (66.68, 1.66, 272.58), (64.65, -0.52, 272.63), (62.49, -2.58, 272.68), (60.27, -4.58, 272.70), (58.02, -6.54, 272.69), (55.74, -8.47, 272.63), (53.46, -10.39, 272.48), (51.19, -12.30, 272.19), (48.99, -14.21, 271.62), (46.96, -16.09, 270.62), (45.31, -17.95, 269.11), (44.38, -19.82, 267.17), (44.43, -21.60, 265.00), (45.42, -23.02, 262.89), (47.57, -24.35, 261.43), (49.50, -24.13, 261.43)]),
    "mic_pin9": dict(mode="hard", r=0.8, snap=(0.65, 0.48), L_search=62.8, relax_min_clear=1.96,
              pts=[(85.13, 39.04, 254.76), (84.61, 38.75, 257.48), (81.89, 37.88, 258.24), (79.44, 36.71, 259.32), (77.36, 35.22, 260.78), (75.56, 33.53, 262.41), (74.01, 31.52, 263.96), (72.41, 29.27, 265.45), (70.24, 28.04, 267.27), (68.18, 26.18, 268.64), (65.18, 25.26, 268.77), (62.74, 23.79, 268.81), (60.62, 21.76, 268.82), (58.53, 19.67, 268.58), (56.38, 17.78, 267.91), (54.06, 16.32, 266.89), (51.45, 15.49, 265.84), (48.64, 15.00, 264.95), (46.92, 14.04, 262.85), (48.52, 13.81, 261.45), (49.50, 13.97, 261.43)]),
    "mic_pin35": dict(mode="hard", r=0.8, snap=(0.56, 1.03), L_search=96.8, relax_min_clear=1.99,
              pts=[(87.67, 39.04, 254.76), (87.82, 38.54, 257.70), (87.20, 37.44, 260.40), (86.16, 36.20, 262.79), (84.68, 34.63, 264.75), (82.88, 32.84, 266.25), (80.91, 30.89, 267.27), (78.85, 28.84, 267.87), (76.77, 26.76, 268.33), (74.72, 24.73, 269.01), (72.76, 22.77, 270.07), (70.85, 20.84, 271.28), (68.92, 18.82, 272.28), (66.93, 16.71, 272.94), (64.92, 14.55, 273.33), (62.89, 12.37, 273.48), (60.87, 10.19, 273.38), (58.85, 8.04, 273.00), (56.85, 5.98, 272.26), (54.89, 4.07, 271.13), (53.00, 2.35, 269.63), (51.20, 0.74, 267.88), (49.53, -0.91, 266.06), (48.08, -2.84, 264.40), (47.00, -5.25, 263.24), (46.32, -7.95, 262.30), (45.72, -10.55, 261.00), (45.17, -13.12, 259.83), (45.10, -15.64, 259.49), (45.83, -17.59, 260.26), (48.03, -19.18, 261.65), (49.50, -19.05, 261.43)]),
    "mic_pin12": dict(mode="hard", r=0.8, snap=(0.60, 0.48), L_search=74.5, relax_min_clear=1.96,
              pts=[(90.21, 39.04, 254.76), (89.41, 39.51, 257.55), (86.72, 39.63, 258.71), (84.27, 39.01, 260.15), (82.24, 37.97, 261.97), (80.48, 36.47, 263.82), (78.76, 34.76, 265.53), (76.91, 33.01, 266.95), (75.02, 30.98, 267.94), (72.87, 29.09, 269.33), (70.48, 27.96, 270.73), (68.18, 26.26, 271.74), (65.35, 25.42, 272.46), (62.86, 24.13, 272.74), (60.69, 22.18, 272.87), (58.65, 20.00, 272.91), (56.64, 17.79, 272.86), (54.60, 15.62, 272.86), (52.49, 13.61, 272.55), (50.37, 12.00, 271.55), (48.54, 11.06, 269.68), (47.35, 10.88, 267.17), (46.88, 11.22, 264.42), (49.50, 11.43, 263.97)]),
    "amp_LRC_pin35": dict(mode="hard", r=0.8, snap=(0.22, 1.03), L_search=72.4, relax_min_clear=2.0,
              pts=[(-13.65, -25.60, 266.37), (-12.72, -28.01, 265.32), (-10.08, -27.88, 263.75), (-7.34, -27.72, 263.18), (-4.39, -27.74, 262.91), (-1.40, -27.87, 262.84), (1.59, -28.02, 262.88), (4.58, -28.16, 262.98), (7.57, -28.30, 263.11), (10.56, -28.43, 263.27), (13.55, -28.54, 263.43), (16.54, -28.63, 263.60), (19.53, -28.70, 263.76), (22.52, -28.73, 263.90), (25.51, -28.72, 264.01), (28.50, -28.65, 264.08), (31.49, -28.47, 264.05), (34.43, -28.09, 263.85), (37.23, -27.37, 263.32), (39.70, -26.15, 262.37), (41.80, -24.45, 261.24), (43.66, -22.44, 260.55), (45.40, -20.46, 260.75), (48.02, -19.06, 261.44), (49.50, -19.05, 261.43)]),
    "amp_BCLK_pin12": dict(mode="hard", r=0.8, snap=(0.42, 0.48), L_search=83.7, relax_min_clear=2.0,
              pts=[(-11.11, -25.60, 266.37), (-9.77, -27.49, 266.62), (-7.66, -28.38, 266.78), (-5.25, -28.71, 267.06), (-2.80, -28.95, 267.39), (-0.32, -29.15, 267.56), (1.68, -29.26, 267.57), (3.68, -29.13, 267.52), (5.51, -28.29, 267.51), (7.72, -27.15, 267.46), (9.79, -25.78, 267.39), (11.68, -24.18, 267.32), (13.45, -22.42, 267.28), (15.12, -20.57, 267.28), (16.77, -18.70, 267.39), (18.42, -16.86, 267.64), (20.08, -15.05, 268.06), (21.76, -13.29, 268.59), (23.47, -11.55, 269.12), (25.20, -9.81, 269.53), (26.95, -8.05, 269.72), (28.70, -6.30, 269.61), (30.42, -4.58, 269.16), (32.07, -2.93, 268.34), (33.65, -1.34, 267.25), (35.19, 0.23, 266.09), (36.76, 1.86, 265.07), (38.37, 3.59, 264.31), (40.04, 5.37, 263.85), (41.75, 7.16, 263.60), (43.49, 8.91, 263.46), (45.19, 10.49, 263.31), (47.05, 11.56, 263.57), (49.50, 11.43, 263.97)]),
    "amp_DIN_pin40": dict(mode="hard", r=0.8, snap=(0.88, 1.17), L_search=70.8, relax_min_clear=1.98,
              pts=[(-8.57, -25.60, 266.37), (-8.68, -27.42, 265.24), (-7.97, -29.41, 264.58), (-6.28, -30.76, 263.95), (-4.09, -31.67, 263.38), (-1.72, -32.27, 263.01), (0.73, -32.66, 262.81), (3.20, -32.92, 262.71), (5.69, -33.11, 262.68), (8.18, -33.26, 262.68), (10.67, -33.37, 262.70), (13.16, -33.46, 262.74), (15.65, -33.52, 262.79), (18.14, -33.54, 262.85), (20.64, -33.54, 262.92), (23.13, -33.50, 263.00), (25.62, -33.42, 263.07), (28.11, -33.29, 263.15), (30.59, -33.10, 263.23), (33.06, -32.81, 263.31), (35.51, -32.37, 263.38), (37.91, -31.72, 263.44), (40.18, -30.76, 263.54), (42.26, -29.44, 263.68), (44.08, -27.79, 263.86), (45.71, -25.92, 264.02), (47.54, -24.34, 264.06), (49.50, -24.13, 263.97)]),
    "amp_GND_pin25": dict(mode="hard", r=0.8, snap=(0.98, 0.94), L_search=69.4, relax_min_clear=1.98,
              pts=[(-0.95, -25.60, 266.37), (0.34, -28.10, 266.06), (3.29, -28.07, 265.60), (5.56, -26.30, 265.13), (7.46, -24.23, 264.44), (9.86, -22.99, 263.71), (12.68, -22.59, 263.22), (15.64, -22.62, 262.98), (18.61, -22.77, 262.84), (21.60, -22.93, 262.77), (24.58, -23.05, 262.77), (27.56, -23.08, 262.89), (30.51, -22.89, 263.14), (33.34, -22.27, 263.62), (35.90, -21.10, 264.37), (38.13, -19.44, 265.33), (40.14, -17.39, 265.81), (41.59, -15.05, 265.03), (42.06, -12.66, 263.44), (43.02, -10.49, 261.85), (44.62, -8.40, 261.10), (46.60, -6.43, 261.40), (49.50, -6.35, 261.43)]),
    "amp_VIN_pin2": dict(mode="hard", r=0.8, snap=(0.88, 0.60), L_search=85.4, relax_min_clear=1.96,
              pts=[(1.59, -25.60, 266.37), (2.73, -27.69, 267.60), (4.54, -26.55, 269.56), (4.98, -23.68, 269.73), (5.43, -20.76, 269.67), (6.40, -17.99, 269.60), (7.88, -15.45, 269.61), (9.69, -13.10, 269.67), (11.67, -10.87, 269.76), (13.72, -8.71, 269.86), (15.81, -6.59, 269.96), (17.93, -4.50, 270.06), (20.08, -2.43, 270.16), (22.24, -0.37, 270.25), (24.41, 1.67, 270.33), (26.59, 3.70, 270.40), (28.78, 5.72, 270.46), (30.98, 7.74, 270.51), (33.17, 9.76, 270.53), (35.37, 11.77, 270.54), (37.57, 13.79, 270.51), (39.75, 15.82, 270.43), (41.89, 17.88, 270.26), (43.93, 19.96, 269.82), (45.60, 21.96, 268.79), (46.54, 23.52, 266.85), (47.07, 24.41, 264.23), (49.50, 24.13, 263.97)]),
    "spk_pair": dict(mode="hard", r=0.9, snap=(0.69, 0.69), L_search=75.2, relax_min_clear=1.98,
              pts=[(55.18, 3.06, 275.25), (53.99, 2.21, 272.86), (51.04, 2.15, 272.67), (48.13, 2.37, 273.09), (45.32, 3.13, 273.36), (42.68, 4.43, 273.48), (40.07, 5.79, 273.52), (37.26, 6.60, 273.66), (34.32, 6.85, 273.80), (31.35, 6.78, 273.96), (28.37, 6.57, 274.11), (25.40, 6.33, 274.22), (22.42, 6.12, 274.26), (19.44, 6.03, 274.20), (16.50, 6.18, 273.93), (13.72, 6.78, 273.33), (11.29, 7.95, 272.27), (9.30, 9.58, 270.87), (7.48, 11.39, 269.39), (5.33, 12.94, 268.19), (2.59, 13.62, 267.52), (-0.30, 13.75, 267.33), (-3.17, 13.22, 267.38), (-5.88, 12.13, 267.47), (-6.11, 9.49, 267.58), (-6.03, 9.00, 267.60)]),
}


# 买的长度（mm）与来源 —— 报告「需要 vs 买的」按这张表
BOUGHT = {
    "servo_cable": (200.0, "measured（HB01.segment_len_mm 用户卡尺 200）"),
    "ph_short_100": (100.0, "hr39c 推荐：PH2.0 3P 壳-壳（双头同向）10 cm，淘宝现成件（转接块 → 嘴下口路上 ≈76，60 不够）"),
    "ph_short_60": (60.0, "hr39 推荐：PH2.0 3P 壳-壳短线 60 mm（原配 200 余 ~190 mm，后脑盘不下，见报告；原配线留作备用）"),
    "splitter_leg_15": (150.0, "hr39 推荐：一分二腿 15 cm（20 cm 余太多，后脑/嘴舵机顶盘不下）"),
    "s1_leg": (100.0, "assumed（components.yaml:bus_splitter『腿长 ≥100 商品页』，到货量）"),
    "dupont_10cm": (100.0, "购物车『10 cm 母对母』（mic 那批，HB08 / electronics.MIC_DUPONT）"),
    "dupont_40cm": (400.0, "HB10『拟用已有 40P 杜邦母母 40 cm 撕 5 根』；IMU 段 2 同款 400（HB05）"),
    "dupont_15cm": (150.0, "hr39 推荐（现口径 HB10『40 cm 撕』余 300+ mm/根 × 3 根 ≈3.2 cm³ 盘不下，见报告）"),
    "splitter_leg": (200.0, "assumed（『杜邦线 一分二 母对二母』商品页未给腿长，常见 20 cm）"),
    "ubec_lead": (100.0, "卖家定 10 cm（components.yaml:buck_12v_5v.connectors_2026-09-22）"),
    "xt30_30cm": (300.0, "购物车 XT30U-F 30 cm（HB03，整根从躯干开关到头里，头内只是一段）"),
    "spk_lead": (100.0, "assumed（components.yaml:speaker cable『≈100 assumed』）"),
    "csi_fpc": (150.0, "随摄像头附 22→15 pin 同向 150（components.yaml:camera_csi）"),
    "usb_c": (150.0, "components.yaml:sbc_radxa_zero3w.connectors『弯头公对公 15 cm』"),
}


def _dense(pts, step=0.5):
    d, _ = catmull_rom(pts, step)
    return d


def _split_at(pts, s_cut, gap=0.0):
    """沿 Catmull-Rom 曲线在弧长 s_cut 处断开（中间留 gap 给转接块），返回 (前段点, 后段点, 断点, 断点切向)"""
    d = _dense(pts)
    L = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(d, axis=0), axis=1))]
    i0 = int(np.searchsorted(L, s_cut)); i1 = int(np.searchsorted(L, s_cut + gap))
    i0 = max(2, min(i0, len(d) - 3)); i1 = max(i0 + 1, min(i1, len(d) - 2))
    a = d[:i0 + 1][::4].tolist() + [d[i0].tolist()]
    b = [d[i1].tolist()] + d[i1:][::4][1:].tolist() + [d[-1].tolist()]
    c = 0.5 * (d[i0] + d[i1]); t = _u(d[min(i1 + 1, len(d) - 1)] - d[max(i0 - 1, 0)])
    return a, b, c, t


S1_SPLIT_S = 5.0        # hr39c-⑦（agent #4）：bus_jaw 前段改道（让出 Radxa +y 端 CSI 座口，见 _ROUTES["bus_jaw"] 注）→ 转接块沿 +y 直段放在转接板 PH 口外 y ≈28..42（x 75.3..80.3、z 238.3..246.3）
S1_SPLIT_S_was_until_2026_09_23_csi = 7.5        # hr39c：S1 转接块沿 bus_jaw 路线的弧长位置（hr39c_work/wires/pick_junction.py：整条线只有 s 8..10 放得下 14×8×5 且对件/扫掠/别的线体素 0 重叠；9.0 精确布尔碰 H01 竖梁角 0.2 mm³、8.0 还有 0.001 → 7.5）


# ━━ hr41（2026-09-24，复审 #1 M2）：8 根弯折半径低于自己声明下限的线重走（+ 为让路改的 mic_pin39 / amp_BCLK 两根），航点由
#    docs/design_2026-09-17_bearing_rebuild/hr41_work/wires/wires_fix.py（端部显式圆弧）+ smooth_opt.py（带碰撞约束平滑，xt30）生成；_define() 末尾整根替换。
#    结果（min R / 下限）：ubec_out_5v 4.02/4.0、ubec_out_gnd 4.06/4.0、mic_pin17 2.24/2.0、mic_pin9 2.21/2.0、mic_pin12_splitleg 2.28/2.0、amp_VIN_pin2 2.19/2.0 —— 满足；
#    xt30_12v 4.83/6.0（原 2.65，插头端改 R 6.6 螺旋弧，耳口 + 竖爬段受 H03 颈口核心/H01 左前脚/侧壁限制）、ubec_in 2.62/4.0（原 1.58，底端 R 4.1 已满足，
#    插头端掉头受麦克风排针柱 x ≥ 83.86 与 S1 盘线区 y ≥ 35.5 夹住）—— 这两根**仍低于下限**，下限本身是 assumed（3×/2.5×od），登记为「下限 assumed、待实测」，不改小下限。
_HR41_ROUTES = {
    "ubec_out_5v": [(87.400, -31.800, 266.480), (87.400, -31.800, 267.460), (87.302, -31.678, 268.651), (87.014, -31.320, 269.760), (86.556, -30.750, 270.713), (85.959, -30.007, 271.444), (85.264, -29.143, 271.903), (84.518, -28.215, 272.060), (81.600, -25.400, 271.900), (78.800, -22.600, 271.200), (76.000, -20.200, 270.200), (72.800, -17.200, 269.200), (70.400, -15.200, 268.800), (69.700, -12.700, 268.500), (69.500, -10.300, 268.500), (69.200, -7.800, 268.600), (69.030, -5.520, 268.630), (67.830, -3.370, 268.720), (66.310, -1.410, 268.780), (64.620, 0.420, 268.820), (62.840, 2.170, 268.840), (61.020, 3.870, 268.880), (60.000, 9.000, 270.200), (60.000, 12.600, 271.900), (60.000, 16.590, 272.770), (59.619, 18.503, 272.770), (58.536, 20.126, 272.770), (56.913, 21.209, 272.770), (55.000, 21.590, 272.770), (52.500, 21.590, 272.770), (50.500, 21.590, 272.770), (47.316, 21.590, 272.435), (45.889, 21.590, 271.481), (44.935, 21.590, 270.054), (44.600, 21.590, 268.370), (44.935, 21.590, 266.686), (45.889, 21.590, 265.259), (47.316, 21.590, 264.305), (49.000, 21.590, 263.970), (49.500, 21.590, 263.970)],
    "ubec_out_gnd": [(87.400, -34.400, 266.480), (87.400, -34.400, 267.460), (87.244, -34.416, 268.651), (86.787, -34.462, 269.760), (86.060, -34.535, 270.713), (85.112, -34.631, 271.444), (84.008, -34.743, 271.903), (82.823, -34.862, 272.060), (79.000, -35.000, 271.600), (75.000, -34.900, 270.600), (71.000, -34.800, 269.800), (67.000, -34.400, 269.400), (63.300, -33.600, 269.500), (60.200, -31.500, 270.000), (57.900, -28.600, 270.600), (56.000, -25.200, 271.800), (56.000, -21.510, 272.770), (55.619, -19.597, 272.770), (54.536, -17.974, 272.770), (52.913, -16.891, 272.770), (51.000, -16.510, 272.770), (50.000, -16.510, 272.770), (47.316, -16.510, 272.435), (45.889, -16.510, 271.481), (44.935, -16.510, 270.054), (44.600, -16.510, 268.370), (44.935, -16.510, 266.686), (45.889, -16.510, 265.259), (47.316, -16.510, 264.305), (49.000, -16.510, 263.970), (49.500, -16.510, 263.970)],
    "mic_pin17": [(87.670, 31.580, 254.760), (87.070, 31.860, 257.650), (84.890, 32.420, 259.530), (82.680, 32.300, 261.330), (80.520, 31.190, 262.760), (78.420, 29.380, 263.620), (76.340, 27.320, 264.160), (74.320, 25.250, 264.830), (72.360, 23.260, 265.860), (70.450, 21.340, 267.110), (68.550, 19.380, 268.320), (66.430, 17.320, 268.770), (64.320, 15.220, 268.870), (62.160, 13.170, 268.670), (59.930, 11.250, 268.230), (57.570, 9.510, 267.680), (55.400, 7.000, 266.900), (53.400, 4.600, 266.100), (51.300, 3.810, 265.850), (49.000, 3.810, 265.930), (48.139, 3.810, 265.759), (47.409, 3.810, 265.271), (46.921, 3.810, 264.541), (46.750, 3.810, 263.680), (46.921, 3.810, 262.819), (47.409, 3.810, 262.089), (48.139, 3.810, 261.601), (49.000, 3.810, 261.430), (49.500, 3.810, 261.430)],
    "mic_pin9": [(85.130, 39.040, 254.760), (84.610, 38.750, 257.480), (81.890, 37.880, 258.240), (79.440, 36.710, 259.320), (77.360, 35.220, 260.780), (75.560, 33.530, 262.410), (74.010, 31.520, 263.960), (72.410, 29.270, 265.450), (70.240, 28.040, 267.270), (68.180, 26.180, 268.640), (65.180, 25.260, 268.770), (62.740, 23.790, 268.810), (60.620, 21.760, 268.820), (58.530, 19.670, 268.580), (56.380, 17.780, 267.910), (54.300, 16.000, 266.900), (52.200, 14.400, 266.100), (50.800, 13.970, 265.870), (49.000, 13.970, 265.930), (48.139, 13.970, 265.759), (47.409, 13.970, 265.271), (46.921, 13.970, 264.541), (46.750, 13.970, 263.680), (46.921, 13.970, 262.819), (47.409, 13.970, 262.089), (48.139, 13.970, 261.601), (49.000, 13.970, 261.430), (49.500, 13.970, 261.430)],
    "mic_pin12_splitleg": [(90.210, 39.040, 254.760), (89.410, 39.510, 257.550), (86.720, 39.630, 258.710), (84.270, 39.010, 260.150), (82.240, 37.970, 261.970), (80.480, 36.470, 263.820), (78.760, 34.760, 265.530), (76.910, 33.010, 266.950), (75.020, 30.980, 267.940), (72.870, 29.090, 269.330), (70.480, 27.960, 270.730), (68.180, 26.260, 271.740), (65.400, 25.400, 271.000), (62.900, 24.100, 270.500), (60.700, 22.200, 270.400), (58.600, 20.000, 270.400), (56.600, 17.800, 270.300), (54.400, 15.000, 269.900), (52.600, 12.600, 269.200), (51.000, 11.500, 268.600), (49.000, 11.430, 268.570), (48.120, 11.430, 268.395), (47.374, 11.430, 267.896), (46.875, 11.430, 267.150), (46.700, 11.430, 266.270), (46.875, 11.430, 265.390), (47.374, 11.430, 264.644), (48.120, 11.430, 264.145), (49.000, 11.430, 263.970), (49.500, 11.430, 263.970)],
    "amp_VIN_pin2": [(1.590, -25.600, 266.370), (1.590, -26.100, 266.370), (1.590, -27.691, 267.029), (1.590, -28.350, 268.620), (1.590, -27.691, 270.211), (1.590, -26.100, 270.870), (2.600, -23.800, 270.400), (4.300, -21.500, 270.100), (6.400, -17.990, 269.600), (7.880, -15.450, 269.610), (9.690, -13.100, 269.670), (11.670, -10.870, 269.760), (13.720, -8.710, 269.860), (15.810, -6.590, 269.960), (17.930, -4.500, 270.060), (20.080, -2.430, 270.160), (22.240, -0.370, 270.250), (24.410, 1.670, 270.330), (26.590, 3.700, 270.400), (28.780, 5.720, 270.460), (30.980, 7.740, 270.510), (33.170, 9.760, 270.530), (35.370, 11.770, 270.540), (37.570, 13.790, 270.510), (39.750, 15.820, 270.430), (41.000, 18.600, 270.200), (41.900, 21.600, 269.400), (42.900, 23.600, 268.100), (44.300, 24.130, 266.600), (45.900, 24.130, 265.000), (47.400, 24.130, 264.100), (48.500, 24.130, 263.970), (49.500, 24.130, 263.970)],
    "mic_pin39": [(90.210, 31.580, 254.760), (89.810, 31.560, 257.710), (87.880, 31.060, 259.950), (86.000, 30.240, 262.030), (84.120, 28.950, 263.880), (82.310, 27.170, 265.350), (80.550, 25.080, 266.490), (78.800, 22.970, 267.630), (77.060, 21.040, 269.050), (75.270, 19.220, 270.560), (73.390, 17.290, 271.740), (71.490, 15.140, 272.330), (69.800, 12.500, 272.500), (70.240, 9.530, 272.520), (69.690, 6.660, 272.520), (68.450, 4.020, 272.540), (66.680, 1.660, 272.580), (64.650, -0.520, 272.630), (62.490, -2.580, 272.680), (60.270, -4.580, 272.700), (58.020, -6.540, 272.690), (55.700, -8.500, 273.000), (53.400, -10.600, 273.800), (51.000, -12.900, 274.300), (48.500, -15.200, 274.500), (46.000, -17.400, 274.400), (43.900, -19.600, 273.400), (42.800, -21.600, 271.000), (42.600, -23.200, 267.800), (43.000, -24.000, 264.700), (44.400, -24.130, 262.400), (46.400, -24.130, 261.500), (48.500, -24.130, 261.430), (49.500, -24.130, 261.430)],
    "amp_BCLK_pin12_splitleg": [(-11.110, -25.600, 266.370), (-9.770, -27.490, 266.620), (-7.660, -28.380, 266.780), (-5.250, -28.710, 267.060), (-2.800, -29.600, 267.400), (-0.300, -30.200, 267.600), (1.700, -30.300, 267.600), (3.700, -30.000, 267.500), (5.500, -29.000, 267.500), (7.720, -27.150, 267.460), (9.790, -25.780, 267.390), (11.680, -24.180, 267.320), (13.450, -22.420, 267.280), (15.120, -20.570, 267.280), (16.770, -18.700, 267.390), (18.420, -16.860, 267.640), (20.080, -15.050, 268.060), (21.760, -13.290, 268.590), (23.470, -11.550, 269.120), (25.200, -9.810, 269.530), (26.950, -8.050, 269.720), (28.700, -6.300, 269.610), (30.420, -4.580, 269.160), (32.070, -2.930, 268.340), (33.650, -1.340, 267.250), (35.190, 0.230, 266.090), (36.760, 1.860, 265.070), (38.370, 3.590, 264.310), (40.040, 5.370, 263.850), (41.750, 7.160, 263.600), (43.490, 8.910, 263.460), (45.190, 10.490, 263.310), (47.050, 11.560, 263.570), (49.500, 11.430, 263.970)],
    "xt30_12v": [(39.900, -19.200, 214.000), (40.460, -19.760, 216.560), (41.002, -20.302, 218.992), (41.400, -20.700, 221.400), (42.480, -21.784, 223.220), (43.922, -23.262, 224.678), (45.500, -24.900, 226.000), (47.400, -26.800, 226.900), (51.580, -33.150, 231.760), (52.325, -33.789, 233.790), (53.322, -34.810, 235.818), (54.892, -37.124, 237.825), (56.600, -40.600, 240.800), (58.300, -43.200, 245.000), (60.500, -44.300, 249.000), (63.600, -44.500, 251.400), (67.000, -44.500, 252.000), (70.400, -44.500, 252.000), (72.440, -44.177, 252.573), (74.279, -43.240, 254.073), (75.740, -41.779, 255.927), (76.677, -39.940, 257.427), (77.000, -37.900, 258.000), (77.000, -37.500, 258.000), (77.000, -37.000, 258.000)],
    "ubec_in": [(87.370, -31.290, 240.460), (87.370, -31.150, 239.399), (87.370, -30.741, 238.410), (87.370, -30.089, 237.561), (87.370, -29.240, 236.909), (87.370, -28.251, 236.500), (87.370, -27.190, 236.360), (87.370, -22.000, 236.360), (87.200, -14.000, 236.660), (86.900, -6.000, 236.900), (86.600, 2.000, 237.200), (86.500, 10.000, 237.300), (86.400, 18.000, 237.400), (86.100, 23.000, 238.200), (85.400, 25.400, 240.800), (84.600, 26.600, 244.500), (83.500, 26.400, 247.800), (82.300, 26.300, 251.500), (82.300, 26.440, 252.561), (82.300, 26.849, 253.550), (82.300, 27.501, 254.399), (82.300, 28.350, 255.051), (82.300, 29.339, 255.460), (82.300, 30.400, 255.600), (82.098, 31.414, 255.600), (81.524, 32.274, 255.600), (80.664, 32.848, 255.600), (79.650, 33.050, 255.600), (78.636, 32.848, 255.600), (77.776, 32.274, 255.600), (77.202, 31.414, 255.600), (77.000, 30.400, 255.600), (77.000, 29.600, 255.350), (77.000, 29.000, 255.300)],
}

def csi_route(hairpin_y=None, bend_r=None):
    """hr41 HB06 走法（世界系零位）：返回 (csiA 航点, csiB 航点, 折叠三角 plug)。hairpin_y = 发夹弯 +y 腿末端 y（≥ 34.25）。"""
    hy = CSI_HAIRPIN_Y if hairpin_y is None else float(hairpin_y); R = CSI_BEND_R if bend_r is None else float(bend_r)
    XA, XB = 71.76, 71.44
    x0c, zc = CSI_CAM["x"], CSI_CAM["z_face"]; hw = CABLE["fpc_csi"]["w"] / 2
    z1 = zc + 6.0; zh = z1 + R; xa2 = XA + R
    arc1 = [(x0c - R + R * math.cos(math.radians(t)), 0.0, z1 + R * math.sin(math.radians(t))) for t in (0.0, 30.0, 60.0, 90.0)]
    arc2 = [(xa2 + R * math.cos(math.radians(t)), 0.0, z1 + R * math.sin(math.radians(t))) for t in (90.0, 120.0, 150.0, 180.0)]
    mid = [(x, 0.0, zh) for x in np.linspace(x0c - R, xa2, 5)[1:-1]]
    ztop = 251.0 + hw
    down = [(XA, 0.0, z) for z in (z1 - 4.0, z1 - 10.0, z1 - 16.0) if z > ztop + 1.0] + [(XA, 0.0, ztop)]
    csiA = [(x0c, 0.0, zc + 0.02), (x0c, 0.0, zc + 3.0)] + arc1 + mid + arc2 + down
    rr = (XB - CSI_RADXA["x"]) / 2; cx = (XB + CSI_RADXA["x"]) / 2
    ys = [hw] + [v for v in (12.0, 20.0, 28.0, 32.5) if v < hy - 0.5] + [hy]
    hp = [(cx + rr * math.cos(math.radians(t)), hy + rr * math.sin(math.radians(t)), 251.0) for t in (0.0, 45.0, 90.0, 135.0, 180.0)]
    back = [(CSI_RADXA["x"], v, 251.0) for v in np.linspace(hy, CSI_RADXA["y_face"], max(2, int((hy - CSI_RADXA["y_face"]) / 4) + 2))[1:]]
    csiB = [(XB, v, 251.0) for v in ys] + hp[1:] + back + [(CSI_RADXA["x"], CSI_RADXA["y_face"] - CSI_RADXA["insert"], 251.0)]
    fold = _tri_prism_x(((-hw, ztop), (hw, ztop), (hw, 251.0 - hw)), XB - 0.15, XA + 0.15)
    return csiA, csiB, fold


def csi_length(hairpin_y=None, bend_r=None):
    """HB06 按 csi_route 走的总长（mm）= csiA + csiB（Catmull-Rom 0.5 重采样后的折线长）+ 折叠中心线（线宽）+ 摄像头端插入 3"""
    a, b, _ = csi_route(hairpin_y, bend_r)
    return polyline_len(_dense(a)) + polyline_len(_dense(b)) + CABLE["fpc_csi"]["w"] + CSI_CAM["insert"]


def _define():
    WIRES.clear(); S1_JUNCTIONS_AT.clear(); STOWS.clear()
    R = _ROUTES
    Yv, Zv, Xv = np.array([0, 1.0, 0]), np.array([0, 0, 1.0]), np.array([1.0, 0, 0])
    Jv = np.array(JAW_PIN_ROW)
    # ── 总线 HB01（hr39c 链）：转接板 PH ← S1 公壳；S1 腿① → 转接块 → PH 短线 → 14 号嘴下口；14 号嘴上口 → 嘴原配扁线 200 → 8 号头横滚下口；
    #    S1 腿② → 出头口 R → 头外 PH 延长线（→ 7 → 6）。转接块在 Radxa +y 端外（x ≈71、y ≈35、z ≈249，斜放，见 S1_SPLIT_S）。
    a, b, JC, JT = _split_at(R["bus_jaw"]["pts"], S1_SPLIT_S, S1_JUNCTION["size"][0])
    jy = Zv if abs(JT[2]) < 0.9 else Yv                     # 转接块 8 宽的方向（与 connector_solids 的 obox 口径一致）
    S1_JUNCTIONS_AT.append((JC, JT))
    _w(id="bus_s1_leg1", harness="HB01", cable="ph_leg_3c", pts=a, sides=[Zv] + [None] * (len(a) - 2) + [jy],
       bought=BOUGHT["s1_leg"][0], bought_src=BOUGHT["s1_leg"][1], route=R["bus_jaw"], group="bus", stow="stow_bus_plus_y",
       ends=("转接板 PH 座（A 边 +y，z 241.4）← S1 公壳", "S1 母针头① ← PH 短线壳（转接块 conn__s1_junction1）"),
       note="腿① 头内只走 ≈9（转接块紧挨转接板 +y 角外）；腿长 ≥100（商品页，assumed）余 ≈90 盘在 Radxa +y 端外")
    nJ = min(len(b) - 2, 13)                                  # 最后 ≈25 mm（沿嘴舵机背面下到下插座那段）宽向锁成针排向 ≈x：厚向朝 y，贴着 H01 横梁（y 19.3..20.3、z 248.6..250.3）前面过
    _w(id="bus_jaw_ph", harness="HB01", cable="ph_leg_3c", pts=b, sides=[jy] + [None] * (len(b) - 1 - nJ) + [Jv] * nJ,
       bought=BOUGHT["ph_short_100"][0], bought_src=BOUGHT["ph_short_100"][1], route=R["bus_jaw"], group="bus", stow="stow_jaw_top",
       ends=("转接块", "14 号嘴舵机下插座（插头顶 y 21.75，线沿 −y 翘 4.4 后弯）"),
       note="从 Radxa +y 端外翻过嘴舵机顶（z ≈264）再沿嘴舵机背面（y ≈17..20）下到下插座")
    _w(id="bus_jaw_roll", harness="HB01", cable="ph_servo_flat", pts=R["bus_jaw_roll"]["pts"],
       sides=[Jv, Jv] + [None] * (len(R["bus_jaw_roll"]["pts"]) - 4) + [Yv, Yv],
       bought=BOUGHT["servo_cable"][0], bought_src=BOUGHT["servo_cable"][1], route=R["bus_jaw_roll"], group="bus", stow="stow_jaw_top",
       ends=("14 号嘴舵机上插座（插头顶 y 21.75，线沿 −y 翘 4.4）", "8 号头横滚下插座（插头顶 x −17.7，线翘 4 后弯）"),
       note="嘴原配扁线 200：沿嘴舵机背面上沿（z ≈256）往后 → 过头横滚扫掠外 → 8 号背面下插座；余 ≈95 Z 字折在嘴舵机顶上")
    ne = len(R["bus_ext"]["pts"])                             # 出 PH 插头后 ≈5 mm 内由针排向 z 扭平（宽向水平）：要从 Radxa +y 下角与 H03 地板之间（高 ≈2..3）钻过去；尾段手改（agent #3）：翻过 H03 右前耳那道肋顶（z ≈230）后在 x ≈40 竖直下耳（肋在 x ≥41.5；原来从肋底角与地板边之间 ≈1.1 的斜缝钻，扁线过不去）
    _w(id="bus_s1_leg2_ext", harness="HB01/HB09", cable="ph_leg_3c", pts=R["bus_ext"]["pts"], sides=[Zv, None, None] + [Yv] * (ne - 3),
       bought=BOUGHT["s1_leg"][0], bought_src=BOUGHT["s1_leg"][1], use_full=True, route=R["bus_ext"], group="hb09",
       ends=("转接板 PH 座 ← S1 公壳", "出头口 R（右前耳）→ 头外接 PH 延长线（接头在头外）"),
       note="过颈束 HB09 的一员：Radxa +y 端下沿 → 嘴舵机下插座下方 → 右前耳出头；头内 ≈68 < 腿长 100")
    # ── IMU（HB05 段 2 的头内部分）4 根 → pin 1/3/5/6
    for n in (1, 3, 5, 6):
        r = R[f"imu_pin{n}"]
        _w(id=f"imu_pin{n}", harness="HB05/HB09", cable="dupont", pts=r["pts"], bought=BOUGHT["dupont_40cm"][0], bought_src=BOUGHT["dupont_40cm"][1],
           use_full=True, route=r, group="hb09", ends=("出头口 R（右前耳）", f"Radxa pin {n}（{PINMAP_was_until_2026_09_25_hr43c[n][0]}）"),
           note="400 线整根从躯干口袋对插点到这里；头内只算这一段，余量跟头外那段一起算（HB05 slack 70..85）")
    # ── 12 V 主干（HB03 头内段）→ XT30 主输入（hr39c：B 边 −y）
    r = R["xt30_12v"]
    _w(id="xt30_12v", harness="HB03/HB09", cable="xt30_18awg_pair", pts=r["pts"], bought=BOUGHT["xt30_30cm"][0], bought_src=BOUGHT["xt30_30cm"][1],
       use_full=True, route=r, group="hb09", ends=("出头口 L（左前耳）", "转接板 XT30 主输入（B 边 −y，z 258.0）← XT30U-F"),
       note="hr39c：B 边挪到 −y 后从左前耳沿 −y 侧直上到主输入，头内 ≈73（hr39 绕 Radxa 顶 ≈190）")
    # ── UBEC（HB04）：假设两组线都从顶端出（用户未回）
    r = R["ubec_in"]
    _w(id="ubec_in", harness="HB04", cable="ubec_in_pair", pts=r["pts"], bought=BOUGHT["ubec_lead"][0], bought_src=BOUGHT["ubec_lead"][1],
       route=r, group="ubec", stow="stow_ubec_in", ends=("UBEC 底端（两头出线，模型按输入朝下装）", "XT30U-F 母头 → 转接板 XT30(2+2) 扩展口（A 边 +y，z 255.3）"),
       note="hr39c（agent #4）：底端出 → 座 +y 墙口 → 贴地板在摄像头 PCB 下沿 +y → y ≈24 竖起 → XT30 扩展口插头尾；若到货只从顶端出，按 before_agent4 旧顶端走法")
    for key, n in (("ubec_out_5v", 4), ("ubec_out_gnd", 34)):
        r = R[key]
        _w(id=key, harness="HB04", cable="ubec_out_1p", pts=r["pts"], bought=BOUGHT["ubec_lead"][0], bought_src=BOUGHT["ubec_lead"][1],
           route=r, group="ubec", stow="stow_ubec_top", ends=("UBEC 顶端（两头出线：输出朝上装）", f"1P 杜邦母 → Radxa pin {n}（{PINMAP_was_until_2026_09_25_hr43c[n][0]}）"))
    # ── 麦克风（HB08）：4 根普通 10 cm + 两根一分二的腿
    for n in (38, 17, 39, 9):
        r = R[f"mic_pin{n}"]
        _w(id=f"mic_pin{n}", harness="HB08", cable="dupont", pts=r["pts"], bought=BOUGHT["dupont_10cm"][0], bought_src=BOUGHT["dupont_10cm"][1],
           route=r, group="mic", stow="stow_mic_front", ends=("麦克风排针（杜邦壳顶 z 254.76）", f"Radxa pin {n}（{PINMAP_was_until_2026_09_25_hr43c[n][0]}）"))
    for n in (12, 35):
        r = R[f"mic_pin{n}"]
        _w(id=f"mic_pin{n}_splitleg", harness="HB08", cable="dupont", pts=r["pts"], bought=BOUGHT["splitter_leg_15"][0], bought_plan=BOUGHT["splitter_leg"][0], bought_src=BOUGHT["splitter_leg_15"][1],
           route=r, group="mic", stow="stow_mic_front", ends=("麦克风排针", f"一分二（单母头插 Radxa pin {n}）"))
    # ── 功放（HB10）：DIN/GND/VIN 三根 + 两根一分二的腿
    amp_assign = {"LRC": 35, "BCLK": 12, "DIN": 40, "GND": 25, "VIN": 2}
    for nm, n in amp_assign.items():
        r = R[f"amp_{nm}_pin{n}"]
        leg = nm in ("LRC", "BCLK")
        bk = "splitter_leg_15" if leg else ("dupont_15cm" if nm == "VIN" else "dupont_10cm")
        _w(id=f"amp_{nm}_pin{n}" + ("_splitleg" if leg else ""), harness="HB10", cable="dupont", pts=r["pts"], bought=BOUGHT[bk][0], bought_src=BOUGHT[bk][1],
           route=r, group="amp", stow="stow_rear", bought_plan=200.0 if leg else 400.0, ends=(f"功放 {nm} 弯针（杜邦壳尾 y −25.6）", f"Radxa pin {n}（{PINMAP_was_until_2026_09_25_hr43c[n][0]}）"))
    # ── 喇叭（HB11）两根 → 功放 +y 螺丝端子（H05 +y 导轨已开窗）
    r = R["spk_pair"]
    _w(id="spk_pair", harness="HB11", cable="spk_pair2", pts=r["pts"], bought=BOUGHT["spk_lead"][0], bought_src=BOUGHT["spk_lead"][1],
       route=r, group="spk", stow="stow_rear", ends=("喇叭背面焊片（凸台底 z 275.25）", "功放 +y 螺丝端子（线从 +y 进）"))
    # ── CSI 排线（HB06，金色 22→15 同面线）：hr39c-⑦（agent #4）摄像头倒装 → 排线从座顶朝 +z 出，翻过转接板上方（z ≈277.6，两道 H05 压筋 |y| 11..13 之间），
    #    在 Radxa 背面元件包络（x 70.6）与转接板（x 72.5）之间的 1.9 缝里竖直下到 z 259，45° 折（折痕 y + z = 251，双层三角 = plug）成沿 +y 走（宽向 z 243..259），
    #    到 y 34.25 绕 z 轴 U 弯（R 1.55）回到 x 68.35 沿 −y 插进 Radxa CSI 座（座口 y 32.25 外直段 2.0，插入 4；22P 端最后 ≈14 收到 11.5 宽）。
    #    同面线 + 一次 45° 折 + 顺弯/U 弯：两端触点面分别朝各自的板（摄像头座触点朝 +x = 摄像头 PCB、Radxa 座触点朝 −x = Radxa 板）—— 按两端都是「下接触」翻盖座算（assumed，到货核）；
    #    若有一端是上接触，把 45° 折改成反向折（折痕对调、占位相同）也不行（折叠次数的奇偶不变）→ 需要多一道折，届时重核。
    XA, XB = 71.76, 71.44                                   # 折前（下行段）/ 折后（+y 段）两层的中面：缝 70.6..72.5 里各占 0.3，层间 0.02
    x0c, zc = CSI_CAM["x"], CSI_CAM["z_face"]
    csiA, csiB, csi_fold = csi_route(CSI_HAIRPIN_Y)
    hw = CABLE["fpc_csi"]["w"] / 2
    _w(id="csi_fpc_a", harness="HB06", cable="fpc_csi", pts=csiA, sides=[Yv] * len(csiA), group="csi", wide_head=True,
       bought=150.0, bought_src="线身丝印「22P 0.5mm to 15P 1.0mm. L150mm」（用户 09-23：长度按丝印 150，先前「实测 142」量错作废）。长度核算 = csi_fpc_a + csi_fpc_b + fixed_mm",
       fixed_mm=CABLE["fpc_csi"]["w"] + CSI_CAM["insert"],
       creases=1, crease_at=f"缝里 x 71.3..71.9、y ±{hw:.2f}、z {251 - hw:.2f}..{251 + hw:.2f} 的 45° 折（折痕 y + z = 251）；其余全是软弯",
       ends=("摄像头 FPC 翻盖座（倒装后在背面顶边，排线朝 +z 出，z 273.94；hr41 摄像头前移 5.9 → 座口中面 x 88.02）", f"45° 折（缝里 z {251 + hw:.2f}..{251 - hw:.2f}，折痕 y + z = 251）"),
       note="hr41 最短走法：座顶竖直 6（补强）→ R2.5 → 转接板上方 z 282.44 横过 → R2.5 → 缝里 x 71.76 下行。**不收余长**（150 线在现布局只许 1 个 U 时收不下，见 harness HB06 / hr41 报告）。fixed_mm = 折叠处中心线（= 线宽 11.3）+ 摄像头端插入 3",
       plugs=[csi_fold])
    _w(id="csi_fpc_b", harness="HB06", cable="fpc_csi", pts=csiB, sides=[Zv] * len(csiB), bought=0.0, bought_src="与 csi_fpc_a 同一根", group="csi", creases=0,
       ends=("45° 折", "Radxa CSI 22P 座（+x 面，+y 端，从板外沿 −y 插入 4）"),
       note=f"宽向 z {251 - hw:.2f}..{251 + hw:.2f}：缝里 x 71.44 沿 +y（离元件包络 0.69、离转接板 0.91）→ y {CSI_HAIRPIN_Y} 绕 z 发夹弯 R{CSI_HAIRPIN_R}（Radxa 端，基线就有）→ x 68.35 沿 −y 进座")
    # ── USB（Radxa USB-C2 ↔ 转接板 Type-C，无 HB 号）：hr39c-⑧（agent #4）两头「超薄 90° 弯头 FPC USB-C」（USB_PLUG_SLIM，assumed），FPC 8 × 0.3（assumed）。
    #    Radxa 端插头线朝 **+x** 出（主 agent 原定 −x 绕 Radxa −y 端外：被 TF 卡包络 y ≤ −34.5 与 xt30_12v 占住，改走缝）：井底 z 231.3 沿 +x → R1.25 竖起（x 71.4，缝里）
    #    → 上到 z 239.5 处 45° 折（折痕 y + z = 234.5，双层三角 = plug）成沿 −y 走（宽向 z 239.5..247.5 = 转接板 Type-C 插头 z 范围）→ y −27 绕 z R1.5 弯朝 +x → 插头 −x 面（y −28.5）。
    #    H03 槽 −x 壁在 FPC 经过处削口（head.H03_USB_WALL_CUTS）。CSI 的折叠三角在 z ≥ 251 − y，USB 的在 z ≤ 234.5 − y，同一条缝里上下错开（y −8..−5 处 z 差 ≥ 8.5）。
    XU, XV = 71.40, 71.70
    yu = SBC_EDGE_PORTS["USB_C2_usb3"]; tc = adp_port("TypeC"); zu = tc["center"][2]
    usbA = [(66.15 + 0.02, yu, 231.3), (68.5, yu, 231.3), (XU - 1.25, yu, 231.3), (XU - 0.366, yu, 231.666), (XU, yu, 232.55), (XU, yu, 235.5), (XU, yu, zu - 4.0)]
    _w(id="usb_fpc_a", harness="(无 HB 号：sbc↔bus_adapter USB)", cable="usb_fpc", pts=usbA, sides=[Yv] * len(usbA), bought=None, group="usb",
       ends=("Radxa USB-C2 插头（井里，线朝 +x 出，z 231.3）", "45° 折（缝里 z 239.5..247.5，折痕 y + z = 234.5）"),
       note="平躺在 USB 井底（井 x 62.3..72.5）沿 +x → R1.25 竖起在缝里 x 71.4（槽 −x 壁 y −13.6..−4.4 已开口）",
       plugs=[_tri_prism_x(((yu - 4.0, zu - 4.0), (yu - 4.0, zu + 4.0), (yu + 4.0, zu - 4.0)), XU - 0.15, XV + 0.15)])
    y1 = tc["center"][1] - tc["ln"] + 0.5                   # 线从插头 −x 面靠外端 0.5 处出（y −28.5）
    usbB = [(XV, yu - 4.0, zu), (XV, -16.0, zu), (XV, -22.0, zu), (XV, y1 + 1.5, zu), (XV + 0.44, y1 + 0.44, zu), (XV + 1.5, y1, zu), (74.2, y1, zu),
            (tc["center"][0] - tc["wx"] / 2 - 0.02, y1, zu)]
    _w(id="usb_fpc_b", harness="(无 HB 号：sbc↔bus_adapter USB)", cable="usb_fpc", pts=usbB, sides=[Zv] * len(usbB), bought=None, group="usb",
       ends=("45° 折", "转接板 Type-C 插头（B 边 −y，线从插头 −x 面出，z 243.5）"),
       note="缝里 x 71.7 沿 −y（宽向 z 239.5..247.5，槽 −x 壁在这段削到 z 239.0）→ y −27 绕 z R1.5 朝 +x → 插头 −x 面 x 75.25")
    for w in WIRES:                                          # hr41：M2 重走线整根替换（见 _HR41_ROUTES 注释）
        if w["id"] in _HR41_ROUTES:
            w["pts"] = [tuple(p) for p in _HR41_ROUTES[w["id"]]]; w["hr41_rerouted"] = True
    STOWS.update({k: v for k, v in _STOW_BOXES.items() if '_was_until_' not in k})
    _apply_hr43()                                              # hr43：Radxa 后脑重走线（文件末尾）
    _apply_hr43c()                                             # hr43c：排针直插杜邦 + R2 针位 + 功放 −y 2.0 + 板挪 → 重走线（文件末尾）
    _apply_hr43d()                                             # hr43d：转接板平放 H08 + 功放后脑 H09 → 重走线 / S1 转接块挪位 / CSI 局部抬（文件末尾）
    _apply_hr44()                                              # hr44：排针杜邦 15 根 + UBEC 输出 2 根按去向扎成 4 束（IMU / 麦 / 功放 / UBEC 输出）重走（文件末尾）
    _apply_hr52()                                              # hr52：ToF（v6-VL53L5CX 雷达）4 根 + 排针 1/3/5/6 腿变化连带 3 根改起点 + 麦束起头让开 ToF（文件末尾）

# 盘线区外包盒（hr39c：hr39c_work/wires/fit_stows.py；原 hr39_wires/fit_stows.py：完全落在空处的轴对齐盒，体积 ≥ 余长×截面÷装填率；装填率扁线 0.8 / 圆线束 0.5）
_STOW_BOXES = {
    "stow_bus_plus_y": dict(boxes=[((72.5, 33.5, 248.6), (82.0, 42.0, 252.9)), ((74.5, 35.5, 253.6), (79.0, 41.0, 255.9)), ((71.5, 36.5, 245.6), (74.0, 39.0, 247.9)), ((75.5, 42.5, 248.6), (78.0, 45.0, 250.9))], fit_mm3=432, need_mm3=429, note="转接板 PH 口外、转接块旁（hr39c-⑦ agent #4 重排）：S1 腿① 余 ≈95（腿长 100 assumed）"),
    "stow_jaw_top": dict(boxes=[((33.5, 28.6, 259.6), (42.0, 36.0, 272.9)), ((31.6, 25.6, 264.6), (44.0, 27.9, 267.9)), ((36.5, 36.5, 263.6), (39.0, 39.0, 268.9)), ((42.5, 30.6, 264.6), (45.0, 33.0, 266.9)), ((36.5, 25.6, 261.6), (39.0, 27.9, 263.9))], fit_mm3=993, need_mm3=988, note="嘴舵机顶上：嘴原配扁线（嘴上口 → 8 号）余 ≈95 Z 字折 + PH 10 cm 短线余"),
    "stow_ubec_top": dict(boxes=[((84.0, -37.5, 273.7), (90.0, -24.0, 276.9))], fit_mm3=259, need_mm3=163, note="hr41：UBEC 顶上（两根输出线 R 4.6 弯出后的顶 272.9 之上 0.8，内顶 y −38 处 277.7）：输出两根（5V / GND 1P）余长。原 hr39c 盒被新弯出段穿过（5V 3.6、GND 1.1 mm³）"),
    "stow_ubec_top_was_until_2026_09_24_hr41": dict(boxes=[((84.5, -38.5, 271.6), (90.0, -23.1, 273.9)), ((85.5, -35.5, 274.6), (88.0, -27.1, 276.9)), ((83.5, -33.5, 268.6), (86.0, -31.1, 270.9)), ((88.5, -30.4, 268.6), (91.0, -28.1, 270.9)), ((81.5, -32.5, 271.6), (84.0, -30.1, 273.9))], fit_mm3=289, need_mm3=279, note="旧"),
    "stow_mic_front": dict(boxes=[((60.5, 31.6, 266.6), (74.0, 36.0, 275.9)), ((63.5, 28.6, 267.6), (69.0, 30.9, 273.9)), ((68.5, 29.6, 263.6), (71.0, 34.0, 265.9)), ((69.5, 28.6, 272.6), (72.0, 30.9, 274.9)), ((66.5, 36.5, 268.6), (69.0, 39.0, 270.9))], fit_mm3=690, need_mm3=678, note="麦克风与 Radxa 排针之间的上方：麦克风 4 根 + 两根一分二腿的余长"),
    "stow_rear": dict(boxes=[((-21.4, -17.4, 258.6), (-17.1, -2.0, 268.9)), ((-17.4, -15.4, 269.6), (-7.0, -11.1, 271.9)), ((-16.4, -14.4, 261.6), (-12.1, -11.1, 263.9)), ((-19.4, -10.4, 269.6), (-17.1, -8.1, 271.9)), ((-16.4, -7.5, 267.6), (-14.1, -5.0, 269.9))], fit_mm3=878, need_mm3=866, note="后脑：功放 5 根（含两根一分二腿）+ 喇叭线的余长"),
    "stow_ubec_in": dict(boxes=[((84.8, -20.0, 237.9), (90.2, -12.0, 241.6))], fit_mm3=160, need_mm3=97, note="hr41：摄像头 PCB/H06 压框底梁（241.94）之下、UBEC 输入线（顶 237.16）之上：输入线余长 ≈12。原盒 x 到 91 被前移 5.9 的摄像头 PCB 占（6.9 mm³）"),
    "stow_ubec_in_was_until_2026_09_24_hr41": dict(boxes=[((88.5, -10.4, 238.6), (91.0, -5.0, 244.9)), ((85.5, -18.4, 239.6), (90.0, -16.1, 241.9)), ((86.5, -19.4, 242.6), (89.0, -17.1, 244.9)), ((86.5, -21.4, 239.6), (89.0, -19.1, 241.9))], fit_mm3=136, need_mm3=124, note="旧"),
}


# ━━━━━━━━━━━━━━━━━━━━ hr43（2026-09-25）Radxa 搬后脑（方案 A）→ 头内线重走 ━━━━━━━━━━━━━━━━━━━━
#   Radxa 平放后脑顶部（electronics.sbc_boxes，板顶 257.8），排针上插 40P 直角转接、15 个杜邦壳沿弯针朝 +x 平插（electronics.zz_adapter40 盒 B：壳 x 26..40 + 线弯 40..44）。
#   → 所有接 Radxa 排针的杜邦线（IMU 4 / UBEC 输出 2 / 功放 5 / 麦克风 6，其中 pin 12、35 是一分二两腿共壳 = 15 个壳 17 根腿）一端改到盒 B 线弯区外缘 x 44：
#     针位 y = 旧 pin_y(n) − 2（新板中心 y −2）；两排横针高度 **assumed**：奇数针（旧内排）在上排 z 271.77、偶数针在下排 269.23（盒 B z 266.5..274.5 中线 ±1.27；
#     直角转接内排针要跨过外排 → 内排在上，到货量）。线从 x 44 起沿 +x 出。
#   功放改直排针版（electronics.amp_boxes BX）：杜邦壳竖挂在板底 x 61.8..64.5、针在 y = 局部 x（LRC −7.62 … VIN +7.62，Adafruit 丝印顺序，assumed），
#     壳尾线弯区底 z 241.7 → 线从那里沿 −z 出（BX 的 4 mm 线弯区算在 zz_amp 里）。喇叭线改到功放端子块 −x 面（x 46.45，线口 z 266.3 = 板顶 + 2.5，同 hr39 口径）。
#   航点 = hr43_work/scripts/route43_run.py（头腔体素 0.5 mm → 1 mm Dijkstra，净空 ≥ r + 0.45，逐根把已走的线当占用）+ 约束平滑；_define() 末尾整根替换（同 hr41 _HR41_ROUTES 做法）。
#   CSI（HB06）与 USB-C2（sbc↔转接板）按新位置另走（见 _HR43_CSI / _HR43_USB）。
HR43_PIN_EXIT_X = 44.0
HR43_ROW_Z = {1: 271.77, 0: 269.23}
def hr43_pin_y(n): return 22.13 - 2.54 * ((n - 1) // 2)
def hr43_pin_exit(n): return np.array([HR43_PIN_EXIT_X, hr43_pin_y(n), HR43_ROW_Z[n % 2]])
HR43_AMP_PIN_Y = {"LRC": -7.62, "BCLK": -5.08, "DIN": -2.54, "GAIN": 0.0, "SD": 2.54, "GND": 5.08, "VIN": 7.62}
HR43_AMP_EXIT = dict(x=63.15, z=241.7)
HR43_AMP_TERM = dict(x_face=46.45, z_wire=266.3, pitch=3.5)
HR43_USB_PLUG_RADXA = dict(lo=(-27.2, -17.0, 256.15), hi=(-19.2, -5.0, 262.65), exit=(-23.2, -17.0, 259.4), out=(0.0, -1.0, 0.0),
                           src="assumed：常见 USB-C 90° 弯头公头包络 12 × 6.5、口面外 8；口 = sbc_boxes USB-C2（x −19.2 口面，y −11.0，z 259.4 口中）；线从 −y 面出（brief：沿 −y 侧走）")
# hr43d：转接板平放 → Type-C 90° 弯头（同 assumed 包络 12 × 6.5、口面外 8）插 B 边 −y（口中心 x 51.8 + 8.1 = 59.9），12 沿 x、6.5 沿 z（底贴板面 266.1）、8 沿 −y；线从 −x 面靠外端出
HR43D_USB_PLUG_ADP = dict(lo=(53.9, -28.0, 266.1), hi=(65.9, -20.0, 272.6), exit=(53.9, -26.0, 269.35), out=(-1.0, 0.0, 0.0),
                          src="assumed：同 HR43_USB_PLUG_ADP 90° 弯头 12 × 6.5 × 口面外 8；转接板平放（hr43d），口 = B 边 −y、离 L 边 8.1（photo_scaled）")
HR43_USB_PLUG_ADP = dict(lo=(73.75, -32.0, 237.5), hi=(80.25, -24.0, 249.5), exit=(73.75, -28.0, 243.5), out=(-1.0, 0.0, 0.0),
                         src="assumed：同上 90° 弯头；插转接板 B 边 Type-C（y −24 口面，z 243.5，x 77），口面外 8、宽 12 沿 z、厚 6.5 沿 x；线从 −x 面出")
_HR43_ROUTES = {   # hr43_work/routes43_final.json（route43_run.py → fix_bends.py，1 mm 均匀航点）
    "ubec_out_5v": [(87.4, -31.8, 266.48), (87.4, -31.8, 267.48), (87.32, -31.8, 268.48), (87.14, -31.8, 269.47), (86.68, -31.8, 270.37), (86.05, -31.8, 271.14), (85.28, -31.8, 271.78), (84.38, -31.8, 272.22), (83.39, -31.8, 272.42), (82.39, -31.8, 272.5), (81.39, -31.8, 272.5), (80.38, -31.8, 272.5), (79.38, -31.8, 272.5), (78.37, -31.8, 272.5), (77.37, -31.8, 272.5), (76.36, -31.8, 272.5), (75.36, -31.8, 272.5), (74.35, -31.8, 272.5), (73.36, -31.7, 272.5), (72.37, -31.51, 272.5), (71.47, -31.06, 272.5), (70.68, -30.45, 272.5), (70.05, -29.68, 272.5), (69.5, -28.84, 272.5), (69.18, -27.89, 272.5), (69.03, -26.89, 272.5), (69.0, -25.89, 272.5), (69.0, -24.88, 272.5), (69.0, -23.88, 272.5), (69.0, -22.87, 272.5), (69.0, -21.87, 272.5), (69.0, -20.87, 272.5), (69.0, -19.86, 272.5), (69.0, -18.86, 272.5), (69.0, -17.85, 272.5), (69.0, -16.85, 272.5), (69.0, -15.84, 272.5), (69.0, -14.84, 272.5), (69.0, -13.83, 272.5), (69.0, -12.83, 272.5), (69.0, -11.82, 272.5), (69.0, -10.82, 272.5), (69.0, -9.81, 272.5), (69.0, -8.81, 272.5), (69.0, -7.8, 272.5), (69.0, -6.8, 272.5), (69.0, -5.79, 272.5), (69.0, -4.79, 272.5), (69.0, -3.78, 272.5), (69.0, -2.78, 272.5), (69.0, -1.77, 272.5), (69.0, -0.77, 272.5), (69.0, 0.24, 272.5), (69.0, 1.24, 272.5), (69.0, 2.25, 272.5), (69.0, 3.25, 272.5), (69.0, 4.26, 272.5), (69.0, 5.26, 272.5), (69.0, 6.26, 272.5), (69.0, 7.27, 272.5), (69.0, 8.27, 272.5), (69.0, 9.28, 272.5), (69.0, 10.28, 272.5), (69.0, 11.29, 272.5), (68.95, 12.29, 272.5), (68.78, 13.28, 272.5), (68.41, 14.21, 272.5), (67.88, 15.06, 272.5), (67.18, 15.79, 272.5), (66.35, 16.35, 272.5), (65.43, 16.76, 272.5), (64.44, 16.93, 272.5), (63.44, 17.0, 272.46), (62.44, 17.0, 272.38), (61.44, 17.0, 272.31), (60.44, 17.0, 272.23), (59.45, 17.08, 272.09), (58.47, 17.23, 271.89), (57.5, 17.38, 271.7), (56.52, 17.52, 271.5), (55.57, 17.73, 271.27), (54.65, 18.01, 270.99), (53.72, 18.28, 270.72), (52.8, 18.56, 270.44), (51.87, 18.83, 270.17), (50.91, 19.05, 269.95), (49.96, 19.28, 269.72), (49.0, 19.5, 269.5), (48.01, 19.54, 269.37), (47.01, 19.59, 269.23), (46.01, 19.59, 269.23), (45.0, 19.59, 269.23), (44.0, 19.59, 269.23)],
    "ubec_out_gnd": [(87.4, -34.4, 266.48), (87.4, -34.4, 267.48), (87.31, -34.4, 268.48), (87.11, -34.4, 269.46), (86.64, -34.4, 270.34), (86.08, -34.4, 271.18), (85.25, -34.4, 271.74), (84.37, -34.4, 272.21), (83.38, -34.4, 272.4), (82.39, -34.44, 272.5), (81.39, -34.47, 272.48), (80.42, -34.26, 272.31), (79.46, -34.04, 272.13), (78.49, -33.83, 271.96), (77.53, -33.61, 271.79), (76.56, -33.4, 271.62), (75.6, -33.18, 271.45), (74.63, -32.96, 271.27), (73.67, -32.74, 271.1), (72.71, -32.51, 270.94), (71.75, -32.24, 270.78), (70.82, -31.9, 270.65), (69.97, -31.37, 270.57), (69.13, -30.83, 270.5), (68.29, -30.28, 270.44), (67.45, -29.74, 270.37), (66.61, -29.18, 270.31), (65.78, -28.63, 270.24), (64.95, -28.07, 270.18), (64.13, -27.49, 270.13), (63.31, -26.91, 270.07), (62.5, -26.33, 270.02), (61.68, -25.74, 269.96), (60.88, -25.14, 269.92), (60.08, -24.54, 269.87), (59.28, -23.93, 269.82), (58.49, -23.31, 269.78), (57.7, -22.69, 269.74), (56.92, -22.07, 269.7), (56.14, -21.44, 269.66), (55.35, -20.82, 269.63), (54.57, -20.19, 269.59), (53.79, -19.56, 269.56), (52.95, -19.01, 269.51), (52.01, -18.67, 269.43), (51.02, -18.55, 269.34), (50.02, -18.51, 269.25), (49.02, -18.51, 269.23), (48.01, -18.51, 269.23), (47.01, -18.51, 269.23), (46.01, -18.51, 269.23), (45.0, -18.51, 269.23), (44.0, -18.51, 269.23)],
    "amp_LRC_pin35_splitleg": [(44.0, -21.05, 271.77), (45.0, -21.05, 271.77), (46.01, -21.05, 271.77), (47.01, -21.05, 271.77), (48.01, -21.05, 271.77), (49.01, -21.08, 271.75), (49.6, -21.36, 271.11), (49.85, -21.05, 270.19), (50.09, -20.73, 269.27), (50.33, -20.41, 268.35), (50.57, -20.09, 267.43), (50.81, -19.76, 266.51), (51.04, -19.44, 265.59), (51.27, -19.12, 264.67), (51.5, -18.79, 263.75), (51.73, -18.46, 262.83), (51.94, -18.13, 261.91), (52.16, -17.79, 260.99), (52.36, -17.45, 260.07), (52.56, -17.1, 259.15), (52.76, -16.75, 258.23), (52.96, -16.4, 257.32), (53.15, -16.03, 256.4), (53.34, -15.66, 255.49), (53.52, -15.29, 254.57), (53.7, -14.91, 253.66), (53.88, -14.52, 252.75), (54.05, -14.13, 251.85), (54.23, -13.73, 250.95), (54.4, -13.32, 250.05), (54.56, -12.9, 249.15), (54.73, -12.48, 248.25), (54.89, -12.05, 247.36), (55.06, -11.62, 246.47), (55.22, -11.18, 245.59), (55.38, -10.73, 244.7), (55.54, -10.28, 243.82), (55.7, -9.83, 242.94), (55.86, -9.37, 242.06), (56.02, -8.91, 241.18), (56.19, -8.45, 240.31), (56.47, -8.02, 239.45), (57.1, -7.72, 238.74), (57.98, -7.59, 238.29), (58.98, -7.62, 238.2), (59.98, -7.62, 238.2), (60.98, -7.62, 238.29), (61.94, -7.62, 238.58), (62.65, -7.62, 239.29), (63.02, -7.62, 240.21), (63.15, -7.62, 241.2), (63.15, -7.62, 241.7)],
    "amp_BCLK_pin12_splitleg": [(44.0, 9.43, 269.23), (45.01, 9.43, 269.23), (46.01, 9.43, 269.23), (47.02, 9.43, 269.23), (48.03, 9.43, 269.23), (49.03, 9.44, 269.26), (49.76, 9.69, 268.92), (50.15, 9.98, 268.04), (50.55, 10.27, 267.16), (50.94, 10.56, 266.28), (51.34, 10.85, 265.4), (51.73, 11.14, 264.52), (52.13, 11.43, 263.64), (52.52, 11.72, 262.76), (52.92, 12.01, 261.88), (53.31, 12.3, 261.0), (53.69, 12.54, 260.1), (53.92, 12.42, 259.14), (54.1, 12.15, 258.18), (54.2, 11.58, 257.36), (54.28, 10.93, 256.6), (54.36, 10.27, 255.84), (54.44, 9.61, 255.09), (54.52, 8.95, 254.33), (54.6, 8.29, 253.58), (54.68, 7.63, 252.82), (54.76, 6.96, 252.07), (54.85, 6.3, 251.32), (54.93, 5.63, 250.57), (55.02, 4.96, 249.83), (55.11, 4.29, 249.08), (55.2, 3.62, 248.34), (55.29, 2.95, 247.59), (55.38, 2.27, 246.85), (55.48, 1.59, 246.11), (55.57, 0.92, 245.38), (55.66, 0.24, 244.64), (55.76, -0.45, 243.91), (55.86, -1.13, 243.17), (55.96, -1.81, 242.44), (56.06, -2.49, 241.71), (56.16, -3.18, 240.98), (56.29, -3.86, 240.24), (56.55, -4.49, 239.51), (57.13, -4.95, 238.84), (57.96, -5.18, 238.32), (58.95, -5.08, 238.2), (59.95, -5.08, 238.2), (60.95, -5.08, 238.29), (61.91, -5.08, 238.59), (62.64, -5.08, 239.28), (63.02, -5.08, 240.21), (63.15, -5.08, 241.2), (63.15, -5.08, 241.7)],
    "amp_DIN_pin40": [(44.0, -26.13, 269.23), (45.01, -26.13, 269.23), (46.02, -26.13, 269.23), (47.03, -26.13, 269.23), (48.03, -26.13, 269.23), (49.01, -26.02, 269.05), (49.75, -25.61, 268.52), (50.23, -25.04, 267.85), (50.61, -24.43, 267.14), (50.94, -23.8, 266.43), (51.27, -23.18, 265.71), (51.6, -22.55, 264.99), (51.93, -21.92, 264.27), (52.26, -21.29, 263.56), (52.58, -20.66, 262.84), (52.9, -20.03, 262.12), (53.22, -19.4, 261.4), (53.54, -18.77, 260.68), (53.86, -18.14, 259.96), (54.13, -17.51, 259.22), (54.35, -16.88, 258.46), (54.55, -16.25, 257.7), (54.76, -15.62, 256.94), (54.96, -15.0, 256.18), (55.16, -14.37, 255.41), (55.36, -13.74, 254.65), (55.55, -13.12, 253.88), (55.74, -12.49, 253.11), (55.93, -11.87, 252.34), (56.11, -11.24, 251.57), (56.29, -10.62, 250.8), (56.47, -10.0, 250.02), (56.64, -9.37, 249.25), (56.81, -8.75, 248.47), (56.97, -8.14, 247.69), (57.13, -7.52, 246.91), (57.3, -6.9, 246.13), (57.46, -6.28, 245.35), (57.61, -5.66, 244.57), (57.77, -5.05, 243.78), (57.93, -4.44, 243.0), (58.05, -3.84, 242.2), (57.98, -3.36, 241.32), (57.94, -3.02, 240.38), (58.31, -2.8, 239.48), (59.02, -2.65, 238.8), (59.95, -2.57, 238.42), (60.95, -2.54, 238.3), (61.91, -2.54, 238.58), (62.64, -2.54, 239.28), (63.02, -2.54, 240.2), (63.15, -2.54, 241.2), (63.15, -2.54, 241.7)],
    "amp_GND_pin25": [(44.0, -8.35, 271.77), (45.0, -8.35, 271.77), (46.01, -8.35, 271.77), (47.01, -8.35, 271.77), (48.01, -8.35, 271.77), (49.0, -8.42, 271.64), (49.74, -8.68, 271.03), (50.27, -8.98, 270.24), (50.78, -9.29, 269.43), (51.28, -9.6, 268.62), (51.78, -9.91, 267.8), (52.28, -10.22, 266.99), (52.78, -10.53, 266.18), (53.28, -10.84, 265.37), (53.78, -11.15, 264.55), (54.28, -11.46, 263.74), (54.78, -11.77, 262.93), (55.29, -12.08, 262.12), (55.79, -12.38, 261.31), (56.28, -12.55, 260.45), (56.74, -12.54, 259.56), (57.06, -12.25, 258.67), (57.14, -11.64, 257.88), (57.2, -11.0, 257.1), (57.25, -10.36, 256.33), (57.29, -9.72, 255.56), (57.33, -9.08, 254.79), (57.37, -8.43, 254.02), (57.41, -7.79, 253.26), (57.43, -7.14, 252.49), (57.45, -6.49, 251.73), (57.46, -5.83, 250.97), (57.46, -5.18, 250.21), (57.45, -4.52, 249.46), (57.43, -3.85, 248.71), (57.4, -3.19, 247.96), (57.37, -2.52, 247.21), (57.33, -1.85, 246.46), (57.28, -1.18, 245.72), (57.23, -0.51, 244.98), (57.18, 0.17, 244.23), (57.12, 0.84, 243.49), (57.04, 1.52, 242.76), (56.95, 2.2, 242.03), (56.86, 2.88, 241.3), (56.81, 3.56, 240.56), (57.2, 4.1, 239.81), (57.59, 4.64, 239.06), (57.98, 5.18, 238.31), (58.97, 5.08, 238.2), (59.98, 5.08, 238.2), (60.97, 5.08, 238.29), (61.93, 5.08, 238.59), (62.65, 5.08, 239.29), (63.02, 5.08, 240.21), (63.15, 5.08, 241.2), (63.15, 5.08, 241.7)],
    "amp_VIN_pin2": [(44.0, 22.13, 269.23), (45.01, 22.13, 269.23), (46.02, 22.13, 269.23), (47.03, 22.13, 269.23), (48.04, 22.13, 269.23), (48.98, 22.13, 268.96), (49.67, 21.98, 268.25), (50.1, 21.71, 267.38), (50.42, 21.39, 266.48), (50.69, 21.06, 265.56), (50.96, 20.72, 264.65), (51.23, 20.38, 263.74), (51.5, 20.04, 262.83), (51.77, 19.69, 261.92), (52.03, 19.35, 261.01), (52.3, 19.01, 260.1), (52.57, 18.66, 259.19), (52.83, 18.31, 258.28), (53.07, 17.94, 257.37), (53.26, 17.52, 256.48), (53.46, 17.09, 255.58), (53.64, 16.65, 254.69), (53.83, 16.21, 253.8), (54.01, 15.77, 252.92), (54.19, 15.32, 252.03), (54.36, 14.86, 251.15), (54.53, 14.4, 250.27), (54.69, 13.93, 249.39), (54.85, 13.45, 248.51), (55.01, 12.97, 247.64), (55.17, 12.48, 246.78), (55.32, 11.98, 245.91), (55.46, 11.48, 245.05), (55.6, 10.97, 244.18), (55.74, 10.46, 243.33), (55.88, 9.94, 242.47), (56.01, 9.42, 241.62), (56.15, 8.9, 240.77), (56.33, 8.38, 239.92), (56.72, 7.92, 239.11), (57.49, 7.66, 238.53), (58.43, 7.61, 238.22), (59.44, 7.62, 238.2), (60.45, 7.62, 238.2), (61.44, 7.62, 238.41), (62.29, 7.62, 238.92), (62.89, 7.62, 239.72), (63.15, 7.62, 240.69), (63.15, 7.62, 241.7)],
    "imu_pin1": [(38.8, 17.03, 215.42), (38.66, 17.27, 216.39), (38.53, 17.51, 217.35), (38.4, 17.74, 218.32), (38.28, 17.96, 219.29), (38.17, 18.18, 220.26), (38.07, 18.37, 221.25), (37.98, 18.56, 222.23), (37.9, 18.73, 223.21), (37.82, 18.87, 224.2), (37.75, 19.01, 225.2), (37.68, 19.1, 226.19), (37.61, 19.2, 227.19), (37.52, 19.25, 228.19), (37.43, 19.29, 229.19), (37.34, 19.32, 230.19), (37.22, 19.31, 231.18), (37.09, 19.3, 232.18), (36.96, 19.27, 233.17), (36.81, 19.23, 234.17), (36.66, 19.19, 235.16), (36.49, 19.13, 236.14), (36.32, 19.08, 237.13), (36.14, 19.03, 238.12), (35.97, 18.98, 239.11), (35.79, 18.94, 240.09), (35.61, 18.91, 241.08), (35.41, 18.87, 242.06), (35.07, 18.83, 243.01), (34.44, 18.8, 243.77), (33.52, 18.87, 244.15), (32.53, 19.01, 244.24), (31.55, 19.21, 244.19), (30.58, 19.45, 244.09), (29.61, 19.68, 243.99), (28.64, 19.92, 243.89), (27.67, 20.15, 243.79), (26.7, 20.39, 243.69), (25.73, 20.62, 243.58), (24.76, 20.88, 243.49), (23.87, 21.31, 243.51), (23.1, 21.94, 243.65), (22.33, 22.57, 243.8), (21.57, 23.21, 243.95), (20.81, 23.84, 244.1), (20.05, 24.48, 244.25), (19.46, 25.25, 244.46), (19.21, 26.18, 244.75), (19.03, 27.12, 245.06), (18.85, 28.05, 245.37), (18.67, 28.99, 245.67), (18.48, 29.93, 245.98), (18.29, 30.87, 246.28), (18.11, 31.81, 246.59), (17.92, 32.74, 246.91), (17.72, 33.63, 247.33), (17.52, 34.43, 247.89), (17.37, 34.86, 248.76), (17.29, 34.97, 249.75), (17.38, 34.8, 250.73), (17.54, 34.54, 251.68), (17.69, 34.27, 252.64), (17.85, 34.0, 253.59), (18.01, 33.73, 254.55), (18.16, 33.47, 255.5), (18.34, 33.19, 256.45), (18.9, 32.76, 257.12), (19.67, 32.25, 257.5), (20.47, 31.75, 257.85), (21.28, 31.25, 258.18), (22.09, 30.76, 258.52), (22.9, 30.28, 258.85), (23.72, 29.8, 259.18), (24.55, 29.34, 259.5), (25.39, 28.88, 259.81), (26.23, 28.44, 260.11), (27.09, 28.0, 260.42), (27.95, 27.59, 260.71), (28.82, 27.18, 261.0), (29.71, 26.78, 261.27), (30.59, 26.4, 261.55), (31.49, 26.02, 261.81), (32.38, 25.66, 262.07), (33.28, 25.29, 262.33), (34.19, 24.94, 262.58), (35.09, 24.58, 262.83), (36.0, 24.24, 263.09), (36.94, 23.98, 263.31), (37.92, 23.89, 263.48), (38.91, 23.81, 263.65), (39.9, 23.72, 263.82), (40.88, 23.64, 263.99), (41.87, 23.56, 264.16), (42.85, 23.48, 264.33), (43.84, 23.4, 264.51), (44.82, 23.32, 264.69), (45.81, 23.24, 264.87), (46.79, 23.15, 265.06), (47.77, 23.07, 265.24), (48.75, 22.99, 265.43), (49.74, 22.9, 265.63), (50.72, 22.82, 265.82), (51.69, 22.71, 266.06), (52.59, 22.55, 266.47), (53.24, 22.31, 267.18), (53.5, 22.14, 268.1), (53.5, 22.13, 269.11), (53.33, 22.13, 270.09), (52.82, 22.13, 270.95), (52.0, 22.13, 271.54), (51.03, 22.13, 271.76), (50.02, 22.13, 271.77), (49.02, 22.13, 271.77), (48.02, 22.13, 271.77), (47.01, 22.13, 271.77), (46.01, 22.13, 271.77), (45.0, 22.13, 271.77), (44.0, 22.13, 271.77)],
    "imu_pin3": [(40.54, 17.55, 215.43), (39.85, 18.18, 215.78), (39.15, 18.81, 216.14), (38.74, 19.26, 216.92), (38.39, 19.67, 217.76), (38.15, 20.06, 218.65), (37.98, 20.42, 219.57), (37.8, 20.77, 220.49), (37.61, 21.06, 221.43), (37.42, 21.35, 222.37), (37.18, 21.6, 223.3), (36.93, 21.84, 224.24), (36.61, 22.06, 225.16), (36.23, 22.24, 226.07), (35.82, 22.41, 226.96), (35.3, 22.51, 227.82), (34.77, 22.58, 228.66), (34.17, 22.59, 229.46), (33.54, 22.54, 230.23), (32.89, 22.46, 230.99), (32.22, 22.31, 231.72), (31.54, 22.15, 232.43), (30.86, 21.93, 233.13), (30.17, 21.7, 233.82), (29.48, 21.46, 234.5), (28.8, 21.22, 235.19), (28.11, 20.97, 235.87), (27.43, 20.75, 236.57), (26.75, 20.54, 237.28), (26.08, 20.36, 237.99), (25.42, 20.21, 238.73), (24.76, 20.07, 239.48), (24.14, 19.96, 240.25), (23.53, 19.85, 241.04), (22.96, 19.75, 241.85), (22.37, 19.72, 242.66), (21.69, 19.89, 243.36), (20.95, 20.34, 243.84), (20.25, 21.05, 243.92), (19.7, 21.85, 243.7), (19.3, 22.67, 243.31), (19.07, 23.56, 242.91), (18.93, 24.52, 242.67), (18.8, 25.5, 242.6), (18.64, 26.48, 242.72), (18.48, 27.45, 242.9), (18.32, 28.42, 243.09), (18.16, 29.39, 243.28), (18.0, 30.36, 243.46), (17.84, 31.33, 243.65), (17.68, 32.3, 243.84), (17.49, 33.25, 244.09), (17.23, 34.12, 244.5), (16.89, 34.86, 245.07), (16.49, 35.3, 245.87), (16.09, 35.5, 246.76), (15.78, 35.53, 247.71), (15.52, 35.44, 248.67), (15.31, 35.29, 249.64), (15.23, 35.02, 250.59), (15.43, 34.53, 251.44), (15.77, 33.96, 252.19), (16.12, 33.39, 252.93), (16.5, 32.8, 253.64), (16.96, 32.21, 254.31), (17.45, 31.62, 254.94), (17.95, 31.02, 255.57), (18.45, 30.42, 256.2), (18.96, 29.83, 256.82), (19.55, 29.26, 257.38), (20.34, 28.77, 257.75), (21.21, 28.35, 258.02), (22.08, 27.94, 258.28), (22.96, 27.53, 258.54), (23.84, 27.12, 258.79), (24.72, 26.72, 259.04), (25.61, 26.33, 259.28), (26.5, 25.95, 259.53), (27.39, 25.57, 259.77), (28.29, 25.21, 260.01), (29.19, 24.85, 260.25), (30.1, 24.51, 260.49), (31.02, 24.17, 260.72), (31.93, 23.85, 260.95), (32.86, 23.53, 261.18), (33.78, 23.23, 261.41), (34.71, 22.94, 261.64), (35.64, 22.66, 261.86), (36.58, 22.39, 262.08), (37.52, 22.13, 262.31), (38.47, 21.88, 262.53), (39.41, 21.64, 262.75), (40.36, 21.42, 262.97), (41.31, 21.19, 263.19), (42.26, 20.98, 263.41), (43.21, 20.77, 263.64), (44.17, 20.57, 263.86), (45.12, 20.37, 264.08), (46.08, 20.18, 264.3), (47.04, 19.99, 264.52), (47.99, 19.81, 264.74), (48.95, 19.63, 264.97), (49.91, 19.51, 265.23), (50.85, 19.49, 265.56), (51.79, 19.49, 265.91), (52.67, 19.51, 266.38), (53.29, 19.55, 267.16), (53.5, 19.59, 268.12), (53.49, 19.59, 269.12), (53.31, 19.59, 270.11), (52.79, 19.59, 270.96), (51.98, 19.59, 271.53), (51.0, 19.59, 271.76), (50.0, 19.59, 271.77), (49.0, 19.59, 271.77), (48.0, 19.59, 271.77), (47.0, 19.59, 271.77), (46.0, 19.59, 271.77), (45.0, 19.59, 271.77), (44.0, 19.59, 271.77)],
    "imu_pin5": [(42.3, 17.5, 215.5), (42.07, 16.88, 216.25), (41.83, 16.26, 217.01), (41.63, 15.95, 217.89), (41.46, 15.94, 218.88), (41.31, 15.94, 219.86), (41.23, 16.0, 220.86), (41.16, 16.06, 221.86), (41.19, 16.05, 222.86), (41.27, 16.01, 223.86), (41.37, 15.96, 224.85), (41.52, 15.9, 225.84), (41.67, 15.83, 226.83), (41.71, 15.86, 227.83), (41.72, 15.92, 228.83), (41.57, 16.04, 229.81), (41.2, 16.23, 230.72), (40.73, 16.42, 231.58), (39.95, 16.55, 232.2), (39.14, 16.66, 232.77), (38.21, 16.67, 233.14), (37.26, 16.66, 233.45), (36.29, 16.63, 233.7), (35.31, 16.59, 233.89), (34.32, 16.54, 234.07), (33.33, 16.49, 234.2), (32.33, 16.45, 234.32), (31.34, 16.42, 234.44), (30.34, 16.41, 234.55), (29.35, 16.39, 234.66), (28.36, 16.41, 234.82), (27.37, 16.42, 234.99), (26.4, 16.46, 235.24), (25.45, 16.51, 235.55), (24.51, 16.58, 235.89), (23.63, 16.68, 236.36), (22.77, 16.79, 236.86), (22.0, 16.95, 237.48), (21.29, 17.14, 238.16), (20.66, 17.37, 238.91), (20.2, 17.67, 239.74), (19.82, 18.01, 240.6), (19.52, 18.5, 241.42), (19.19, 19.19, 242.07), (18.88, 20.1, 242.33), (18.66, 21.06, 242.18), (18.53, 21.98, 241.82), (18.49, 22.9, 241.41), (18.47, 23.85, 241.1), (18.39, 24.84, 240.95), (18.27, 25.83, 240.9), (18.16, 26.82, 240.87), (18.05, 27.82, 240.83), (17.93, 28.81, 240.79), (17.82, 29.81, 240.76), (17.71, 30.8, 240.73), (17.59, 31.8, 240.7), (17.45, 32.79, 240.71), (17.22, 33.75, 240.86), (16.74, 34.49, 241.31), (16.11, 34.9, 241.97), (15.45, 35.03, 242.72), (14.83, 34.98, 243.5), (14.39, 34.59, 244.3), (14.1, 34.03, 245.08), (13.99, 33.4, 245.85), (13.97, 32.75, 246.61), (13.97, 32.09, 247.36), (13.96, 31.43, 248.12), (13.95, 30.78, 248.88), (13.97, 30.12, 249.63), (14.18, 29.46, 250.35), (14.6, 28.83, 251.01), (15.11, 28.23, 251.63), (15.7, 27.66, 252.2), (16.32, 27.11, 252.77), (16.94, 26.55, 253.33), (17.56, 26.0, 253.89), (18.17, 25.45, 254.46), (18.8, 24.9, 255.01), (19.51, 24.41, 255.52), (20.26, 23.95, 255.99), (21.03, 23.5, 256.45), (21.8, 23.05, 256.91), (22.59, 22.62, 257.36), (23.39, 22.2, 257.8), (24.2, 21.79, 258.22), (25.03, 21.4, 258.63), (25.86, 21.02, 259.02), (26.72, 20.67, 259.4), (27.59, 20.33, 259.77), (28.48, 20.02, 260.12), (29.37, 19.72, 260.46), (30.28, 19.45, 260.77), (31.2, 19.19, 261.08), (32.13, 18.96, 261.37), (33.07, 18.74, 261.65), (34.02, 18.54, 261.91), (34.97, 18.37, 262.16), (35.93, 18.21, 262.41), (36.89, 18.07, 262.64), (37.86, 17.94, 262.87), (38.83, 17.83, 263.09), (39.8, 17.73, 263.31), (40.78, 17.65, 263.52), (41.76, 17.58, 263.72), (42.74, 17.53, 263.93), (43.72, 17.49, 264.13), (44.7, 17.45, 264.33), (45.68, 17.43, 264.52), (46.67, 17.41, 264.72), (47.65, 17.4, 264.92), (48.63, 17.4, 265.12), (49.61, 17.41, 265.31), (50.6, 17.42, 265.51), (51.58, 17.43, 265.71), (52.56, 17.45, 265.91), (53.44, 17.36, 266.38), (54.32, 17.28, 266.85), (55.21, 17.2, 267.32), (55.2, 17.05, 268.3), (55.18, 17.05, 269.3), (54.95, 17.05, 270.28), (54.37, 17.05, 271.09), (53.5, 17.05, 271.6), (52.52, 17.05, 271.76), (51.52, 17.05, 271.77), (50.51, 17.05, 271.77), (49.51, 17.05, 271.77), (48.51, 17.05, 271.77), (47.51, 17.05, 271.77), (46.51, 17.05, 271.77), (45.5, 17.05, 271.77), (44.5, 17.05, 271.77), (44.0, 17.05, 271.77)],
    "imu_pin6": [(44.14, 17.56, 215.44), (43.82, 18.39, 215.9), (43.5, 19.23, 216.35), (43.13, 20.04, 216.82), (42.73, 20.83, 217.29), (42.3, 21.6, 217.78), (41.79, 22.32, 218.26), (41.28, 23.01, 218.76), (40.56, 23.31, 219.4), (39.83, 23.47, 220.07), (39.2, 23.58, 220.84), (38.61, 23.66, 221.65), (38.01, 23.8, 222.44), (37.41, 23.94, 223.23), (36.79, 24.11, 224.01), (36.14, 24.31, 224.74), (35.45, 24.51, 225.44), (34.56, 24.66, 225.89), (33.63, 24.79, 226.24), (32.69, 24.92, 226.56), (31.73, 25.05, 226.84), (30.77, 25.18, 227.1), (29.81, 25.31, 227.34), (28.84, 25.44, 227.56), (27.87, 25.57, 227.8), (26.91, 25.72, 228.04), (25.95, 25.88, 228.29), (25.02, 26.09, 228.61), (24.1, 26.31, 228.95), (23.24, 26.6, 229.37), (22.42, 26.94, 229.85), (21.65, 27.34, 230.36), (20.97, 27.81, 230.92), (20.35, 28.32, 231.52), (19.98, 28.92, 232.24), (19.67, 29.54, 232.96), (19.41, 30.2, 233.66), (19.2, 30.89, 234.37), (19.01, 31.58, 235.07), (18.84, 32.28, 235.77), (18.67, 32.96, 236.48), (18.5, 33.62, 237.23), (18.33, 34.24, 238.0), (18.16, 34.8, 238.81), (17.98, 35.29, 239.67), (17.8, 35.74, 240.55), (17.63, 36.08, 241.48), (17.46, 36.38, 242.42), (17.32, 36.6, 243.39), (17.21, 36.78, 244.37), (17.27, 37.14, 245.3), (17.39, 37.38, 246.25), (17.25, 37.24, 247.23), (17.12, 37.09, 248.22), (16.98, 36.94, 249.2), (16.84, 36.79, 250.18), (16.7, 36.62, 251.16), (16.6, 36.29, 252.1), (16.53, 35.83, 252.99), (16.5, 35.3, 253.85), (16.49, 34.74, 254.68), (16.49, 34.18, 255.51), (16.53, 33.59, 256.31), (16.82, 32.89, 256.96), (17.29, 32.13, 257.43), (17.77, 31.39, 257.89), (18.24, 30.63, 258.36), (18.74, 29.88, 258.8), (19.26, 29.13, 259.22), (19.86, 28.41, 259.57), (20.54, 27.72, 259.82), (21.25, 27.04, 260.05), (21.96, 26.36, 260.25), (22.68, 25.7, 260.46), (23.4, 25.03, 260.67), (24.12, 24.36, 260.87), (24.84, 23.69, 261.07), (25.57, 23.03, 261.28), (26.29, 22.36, 261.48), (27.02, 21.7, 261.68), (27.75, 21.04, 261.88), (28.47, 20.37, 262.07), (29.21, 19.71, 262.26), (29.94, 19.06, 262.45), (30.72, 18.45, 262.6), (31.58, 17.93, 262.66), (32.46, 17.45, 262.71), (33.35, 16.99, 262.74), (34.27, 16.59, 262.75), (35.25, 16.38, 262.76), (36.24, 16.26, 262.77), (37.24, 16.14, 262.77), (38.24, 16.03, 262.78), (39.24, 15.98, 262.8), (40.24, 15.97, 262.83), (41.25, 15.97, 262.85), (42.25, 15.97, 262.88), (43.25, 16.0, 262.91), (44.25, 16.08, 262.96), (45.25, 16.19, 263.01), (46.24, 16.32, 263.07), (47.23, 16.49, 263.13), (48.21, 16.69, 263.23), (49.18, 16.92, 263.37), (50.12, 17.12, 263.6), (50.9, 17.22, 264.16), (51.09, 17.13, 265.08), (51.0, 17.05, 266.07), (50.91, 17.05, 267.06), (50.57, 17.05, 267.99), (49.91, 17.05, 268.71), (49.01, 17.05, 269.11), (48.02, 17.05, 269.22), (47.01, 17.05, 269.23), (46.01, 17.05, 269.23), (45.0, 17.05, 269.23), (44.0, 17.05, 269.23)],
    "mic_pin38": [(85.13, 31.58, 254.76), (85.13, 31.58, 255.76), (85.06, 31.23, 256.62), (84.65, 30.54, 257.21), (84.11, 29.87, 257.72), (83.55, 29.21, 258.23), (82.99, 28.55, 258.72), (82.43, 27.88, 259.22), (81.86, 27.22, 259.71), (81.3, 26.55, 260.21), (80.73, 25.89, 260.7), (80.16, 25.21, 261.17), (79.58, 24.52, 261.6), (79.01, 23.82, 262.02), (78.43, 23.12, 262.44), (77.85, 22.42, 262.86), (77.26, 21.71, 263.27), (76.68, 21.01, 263.68), (76.1, 20.3, 264.09), (75.52, 19.59, 264.49), (74.94, 18.88, 264.9), (74.35, 18.17, 265.29), (73.77, 17.46, 265.69), (73.19, 16.75, 266.08), (72.61, 16.03, 266.47), (72.02, 15.3, 266.83), (71.44, 14.57, 267.18), (70.85, 13.84, 267.53), (70.27, 13.1, 267.88), (69.82, 12.24, 268.08), (69.54, 11.28, 268.09), (69.26, 10.32, 268.1), (68.97, 9.36, 268.12), (68.68, 8.4, 268.13), (68.38, 7.45, 268.14), (68.07, 6.49, 268.15), (67.75, 5.54, 268.15), (67.43, 4.6, 268.15), (67.1, 3.65, 268.15), (66.75, 2.71, 268.15), (66.39, 1.78, 268.15), (66.02, 0.85, 268.14), (65.64, -0.08, 268.13), (65.23, -1.0, 268.12), (64.82, -1.91, 268.1), (64.39, -2.81, 268.09), (63.95, -3.71, 268.07), (63.5, -4.61, 268.05), (63.03, -5.49, 268.03), (62.56, -6.38, 268.0), (62.07, -7.25, 267.98), (61.57, -8.12, 267.95), (61.05, -8.97, 267.92), (60.53, -9.83, 267.89), (60.0, -10.68, 267.85), (59.46, -11.52, 267.82), (58.91, -12.36, 267.78), (58.36, -13.19, 267.74), (57.8, -14.02, 267.71), (57.23, -14.85, 267.67), (56.66, -15.67, 267.63), (56.09, -16.49, 267.59), (55.51, -17.31, 267.55), (54.93, -18.12, 267.52), (54.35, -18.94, 267.48), (53.69, -19.66, 267.65), (52.99, -20.3, 267.96), (52.29, -20.95, 268.27), (51.58, -21.59, 268.58), (50.88, -22.23, 268.89), (50.18, -22.88, 269.2), (49.45, -23.5, 269.46), (48.51, -23.59, 269.23), (47.51, -23.59, 269.23), (46.5, -23.59, 269.23), (45.5, -23.59, 269.23), (44.5, -23.59, 269.23), (44.0, -23.59, 269.23)],
    "mic_pin17": [(87.67, 31.58, 254.76), (87.67, 31.58, 255.77), (87.4, 31.36, 256.64), (86.94, 30.72, 257.28), (86.49, 30.09, 257.91), (85.89, 29.51, 258.48), (85.26, 28.95, 259.03), (84.64, 28.39, 259.59), (84.02, 27.83, 260.15), (83.39, 27.26, 260.7), (82.77, 26.7, 261.26), (82.14, 26.14, 261.81), (81.52, 25.57, 262.37), (80.89, 25.01, 262.92), (80.17, 24.5, 263.4), (79.42, 24.01, 263.86), (78.67, 23.52, 264.32), (77.91, 23.03, 264.77), (77.11, 22.56, 265.14), (76.24, 22.11, 265.39), (75.37, 21.67, 265.64), (74.5, 21.22, 265.88), (73.64, 20.77, 266.13), (72.77, 20.31, 266.37), (71.9, 19.85, 266.61), (71.04, 19.39, 266.85), (70.18, 18.91, 267.08), (69.33, 18.43, 267.31), (68.48, 17.95, 267.54), (67.63, 17.45, 267.77), (66.79, 16.95, 267.99), (65.95, 16.44, 268.22), (65.12, 15.92, 268.44), (64.29, 15.39, 268.67), (63.47, 14.85, 268.9), (62.65, 14.3, 269.13), (61.85, 13.75, 269.36), (61.05, 13.18, 269.6), (60.26, 12.6, 269.83), (59.47, 12.02, 270.07), (58.7, 11.42, 270.31), (57.93, 10.81, 270.56), (57.17, 10.2, 270.81), (56.42, 9.59, 271.07), (55.67, 8.97, 271.33), (54.93, 8.33, 271.59), (54.2, 7.7, 271.86), (53.47, 7.05, 272.13), (52.75, 6.4, 272.41), (52.04, 5.75, 272.68), (51.32, 5.09, 272.96), (50.61, 4.44, 273.24), (49.91, 3.77, 273.52), (49.21, 3.11, 273.81), (48.46, 2.49, 274.07), (47.58, 2.03, 274.17), (46.64, 1.84, 273.88), (45.82, 1.81, 273.31), (45.22, 1.81, 272.5), (44.5, 1.81, 271.83), (44.0, 1.81, 271.77)],
    "mic_pin39": [(90.21, 31.58, 254.76), (90.21, 31.58, 255.76), (90.3, 31.51, 256.75), (90.24, 31.06, 257.62), (89.92, 30.32, 258.2), (89.54, 29.53, 258.69), (89.16, 28.74, 259.17), (88.77, 27.95, 259.65), (88.39, 27.15, 260.12), (88.01, 26.35, 260.59), (87.64, 25.54, 261.05), (87.27, 24.73, 261.51), (86.9, 23.92, 261.96), (86.53, 23.1, 262.41), (86.16, 22.28, 262.85), (85.8, 21.46, 263.29), (85.44, 20.63, 263.72), (85.08, 19.8, 264.14), (84.72, 18.96, 264.57), (84.37, 18.12, 264.98), (84.02, 17.27, 265.38), (83.67, 16.42, 265.79), (83.33, 15.58, 266.19), (82.98, 14.73, 266.59), (82.63, 13.87, 266.98), (82.29, 13.02, 267.38), (81.95, 12.16, 267.77), (81.61, 11.31, 268.16), (81.05, 10.5, 268.33), (80.42, 9.73, 268.39), (79.79, 8.95, 268.45), (79.16, 8.18, 268.51), (78.53, 7.4, 268.58), (77.89, 6.63, 268.65), (77.26, 5.86, 268.72), (76.62, 5.09, 268.79), (75.98, 4.32, 268.86), (75.35, 3.55, 268.94), (74.7, 2.79, 269.02), (74.06, 2.02, 269.11), (73.42, 1.26, 269.19), (72.77, 0.5, 269.28), (72.13, -0.26, 269.37), (71.48, -1.02, 269.47), (70.83, -1.78, 269.57), (70.18, -2.53, 269.67), (69.53, -3.29, 269.77), (68.88, -4.04, 269.87), (68.23, -4.79, 269.98), (67.58, -5.55, 270.08), (66.92, -6.3, 270.19), (66.27, -7.05, 270.29), (65.62, -7.8, 270.39), (64.96, -8.55, 270.5), (64.31, -9.31, 270.59), (63.66, -10.06, 270.69), (63.0, -10.81, 270.78), (62.35, -11.57, 270.87), (61.7, -12.32, 270.95), (61.04, -13.08, 271.03), (60.39, -13.83, 271.1), (59.73, -14.58, 271.17), (59.08, -15.34, 271.23), (58.42, -16.09, 271.3), (57.77, -16.85, 271.34), (57.11, -17.61, 271.39), (56.46, -18.36, 271.44), (55.81, -19.12, 271.49), (55.15, -19.88, 271.51), (54.5, -20.64, 271.52), (53.85, -21.41, 271.52), (53.2, -22.17, 271.52), (52.55, -22.93, 271.52), (51.9, -23.69, 271.52), (51.25, -24.45, 271.51), (50.6, -25.21, 271.51), (49.92, -25.95, 271.52), (49.0, -26.18, 271.7), (48.01, -26.13, 271.77), (47.0, -26.13, 271.77), (46.0, -26.13, 271.77), (45.0, -26.13, 271.77), (44.0, -26.13, 271.77)],
    "mic_pin9": [(85.13, 39.04, 254.76), (85.13, 39.04, 255.77), (84.82, 38.87, 256.64), (84.14, 38.43, 257.23), (83.42, 37.89, 257.7), (82.7, 37.33, 258.13), (81.98, 36.77, 258.54), (81.26, 36.2, 258.96), (80.53, 35.62, 259.37), (79.81, 35.05, 259.77), (79.09, 34.47, 260.18), (78.37, 33.89, 260.58), (77.65, 33.3, 260.97), (76.93, 32.71, 261.37), (76.21, 32.11, 261.75), (75.5, 31.52, 262.14), (74.78, 30.92, 262.52), (74.06, 30.32, 262.89), (73.35, 29.71, 263.27), (72.63, 29.1, 263.64), (71.92, 28.49, 264.0), (71.21, 27.88, 264.37), (70.49, 27.26, 264.73), (69.78, 26.65, 265.1), (69.06, 26.03, 265.44), (68.31, 25.4, 265.7), (67.56, 24.77, 265.93), (66.81, 24.14, 266.16), (66.05, 23.51, 266.38), (65.29, 22.89, 266.61), (64.53, 22.26, 266.83), (63.77, 21.64, 267.06), (63.01, 21.01, 267.28), (62.25, 20.39, 267.51), (61.49, 19.77, 267.73), (60.73, 19.15, 267.96), (59.96, 18.53, 268.18), (59.2, 17.91, 268.41), (58.43, 17.29, 268.63), (57.67, 16.67, 268.86), (56.89, 16.06, 269.09), (56.08, 15.54, 269.35), (55.25, 15.03, 269.63), (54.42, 14.52, 269.89), (53.59, 14.01, 270.16), (52.76, 13.5, 270.43), (51.93, 12.99, 270.7), (51.11, 12.48, 270.97), (50.28, 11.98, 271.25), (49.4, 11.66, 271.56), (48.54, 11.97, 271.77), (47.53, 11.97, 271.77), (46.52, 11.97, 271.77), (45.51, 11.97, 271.77), (44.5, 11.97, 271.77), (44.0, 11.97, 271.77)],
    "mic_pin35_splitleg": [(87.67, 39.04, 254.76), (87.67, 39.04, 255.76), (87.35, 38.93, 256.64), (86.82, 38.36, 257.26), (86.34, 37.64, 257.79), (85.87, 36.91, 258.3), (85.41, 36.18, 258.8), (84.94, 35.45, 259.3), (84.47, 34.72, 259.8), (83.98, 33.98, 260.29), (83.5, 33.25, 260.77), (83.01, 32.51, 261.25), (82.52, 31.78, 261.73), (82.03, 31.04, 262.2), (81.52, 30.31, 262.66), (81.01, 29.57, 263.12), (80.5, 28.84, 263.57), (79.98, 28.1, 264.02), (79.46, 27.36, 264.46), (78.94, 26.63, 264.9), (78.41, 25.89, 265.33), (77.88, 25.16, 265.77), (77.35, 24.42, 266.2), (76.82, 23.68, 266.62), (76.29, 22.93, 267.02), (75.72, 22.14, 267.27), (75.15, 21.34, 267.49), (74.58, 20.55, 267.71), (74.02, 19.75, 267.94), (73.45, 18.95, 268.16), (72.88, 18.15, 268.38), (72.31, 17.36, 268.6), (71.74, 16.56, 268.82), (71.17, 15.76, 269.05), (70.6, 14.96, 269.27), (70.05, 14.15, 269.47), (69.56, 13.29, 269.62), (69.1, 12.4, 269.74), (68.64, 11.52, 269.86), (68.18, 10.64, 269.98), (67.72, 9.75, 270.09), (67.26, 8.87, 270.21), (66.8, 7.98, 270.33), (66.33, 7.1, 270.43), (65.87, 6.21, 270.54), (65.4, 5.33, 270.64), (64.94, 4.45, 270.74), (64.47, 3.56, 270.83), (64.0, 2.68, 270.92), (63.53, 1.8, 271.01), (63.05, 0.92, 271.1), (62.56, 0.04, 271.17), (62.08, -0.83, 271.24), (61.59, -1.71, 271.31), (61.09, -2.58, 271.37), (60.59, -3.45, 271.42), (60.09, -4.31, 271.47), (59.58, -5.18, 271.52), (59.06, -6.04, 271.55), (58.54, -6.89, 271.59), (58.01, -7.75, 271.62), (57.47, -8.6, 271.64), (56.94, -9.45, 271.65), (56.4, -10.29, 271.66), (55.85, -11.14, 271.67), (55.3, -11.98, 271.67), (54.75, -12.81, 271.67), (54.19, -13.65, 271.66), (53.63, -14.48, 271.65), (53.07, -15.31, 271.63), (52.5, -16.14, 271.61), (51.94, -16.97, 271.59), (51.37, -17.8, 271.57), (50.8, -18.63, 271.55), (50.23, -19.45, 271.53), (49.65, -20.28, 271.51), (49.02, -21.01, 271.75), (48.02, -21.05, 271.77), (47.01, -21.05, 271.77), (46.01, -21.05, 271.77), (45.0, -21.05, 271.77), (44.0, -21.05, 271.77)],
    "mic_pin12_splitleg": [(90.21, 39.04, 254.76), (90.21, 39.04, 255.76), (90.24, 39.15, 256.75), (89.94, 39.07, 257.68), (89.32, 38.66, 258.34), (88.63, 38.19, 258.89), (87.92, 37.69, 259.41), (87.22, 37.2, 259.94), (86.52, 36.71, 260.46), (85.82, 36.22, 260.98), (85.11, 35.68, 261.44), (84.39, 35.13, 261.87), (83.66, 34.54, 262.22), (82.91, 33.94, 262.52), (82.17, 33.33, 262.81), (81.43, 32.72, 263.1), (80.68, 32.11, 263.37), (79.93, 31.5, 263.64), (79.18, 30.88, 263.9), (78.43, 30.27, 264.14), (77.67, 29.65, 264.39), (76.91, 29.03, 264.61), (76.15, 28.42, 264.83), (75.39, 27.8, 265.03), (74.62, 27.18, 265.22), (73.86, 26.56, 265.39), (73.08, 25.94, 265.56), (72.31, 25.32, 265.72), (71.53, 24.7, 265.86), (70.75, 24.08, 266.01), (69.97, 23.47, 266.13), (69.18, 22.86, 266.25), (68.4, 22.25, 266.37), (67.61, 21.64, 266.48), (66.82, 21.03, 266.59), (66.02, 20.43, 266.7), (65.22, 19.83, 266.81), (64.43, 19.23, 266.91), (63.63, 18.64, 267.02), (62.82, 18.05, 267.13), (62.01, 17.46, 267.24), (61.2, 16.88, 267.36), (60.39, 16.3, 267.47), (59.58, 15.73, 267.6), (58.76, 15.16, 267.73), (57.94, 14.61, 267.87), (57.11, 14.05, 268.01), (56.28, 13.51, 268.15), (55.44, 12.98, 268.31), (54.6, 12.46, 268.46), (53.75, 11.95, 268.63), (52.9, 11.44, 268.8), (52.05, 10.95, 268.97), (51.19, 10.46, 269.15), (50.33, 9.97, 269.33), (49.46, 9.5, 269.47), (48.52, 9.43, 269.23), (47.51, 9.43, 269.23), (46.51, 9.43, 269.23), (45.51, 9.43, 269.23), (44.5, 9.43, 269.23), (44.0, 9.43, 269.23)],
    "bus_jaw_roll": [(40.34, 21.75, 254.03), (40.41, 20.75, 254.06), (40.42, 19.76, 254.14), (40.21, 18.8, 254.35), (39.77, 17.96, 254.68), (39.14, 17.28, 255.04), (38.33, 16.82, 255.38), (37.39, 16.59, 255.63), (36.41, 16.5, 255.8), (35.41, 16.53, 255.88), (34.44, 16.74, 255.81), (33.5, 17.03, 255.67), (32.55, 17.33, 255.53), (31.6, 17.62, 255.4), (30.66, 17.92, 255.26), (29.71, 18.22, 255.12), (28.77, 18.52, 254.99), (27.83, 18.83, 254.86), (26.88, 19.14, 254.73), (25.94, 19.46, 254.6), (25.0, 19.77, 254.47), (24.06, 20.09, 254.35), (23.12, 20.41, 254.22), (22.19, 20.74, 254.1), (21.25, 21.07, 253.98), (20.31, 21.4, 253.85), (19.38, 21.73, 253.73), (18.44, 22.06, 253.6), (17.51, 22.4, 253.49), (16.58, 22.75, 253.38), (15.65, 23.1, 253.26), (14.72, 23.44, 253.11), (13.85, 23.67, 252.68), (13.05, 23.83, 252.11), (12.24, 23.98, 251.54), (11.38, 24.08, 251.04), (10.43, 24.02, 250.73), (9.47, 23.91, 250.51), (8.48, 23.74, 250.41), (7.5, 23.57, 250.36), (6.52, 23.38, 250.3), (5.54, 23.19, 250.24), (4.56, 22.99, 250.19), (3.58, 22.78, 250.13), (2.61, 22.56, 250.07), (1.64, 22.34, 250.02), (0.66, 22.1, 249.96), (-0.3, 21.85, 249.91), (-1.27, 21.58, 249.86), (-2.22, 21.3, 249.81), (-3.18, 21.01, 249.76), (-4.13, 20.7, 249.71), (-5.07, 20.37, 249.66), (-6.01, 20.04, 249.61), (-6.95, 19.68, 249.56), (-7.87, 19.3, 249.51), (-8.79, 18.91, 249.46), (-9.71, 18.51, 249.41), (-10.61, 18.09, 249.36), (-11.51, 17.65, 249.3), (-12.41, 17.2, 249.25), (-13.29, 16.74, 249.2), (-14.17, 16.27, 249.15), (-15.05, 15.79, 249.09), (-15.92, 15.29, 249.04), (-16.78, 14.79, 248.99), (-17.64, 14.28, 248.93), (-18.49, 13.77, 248.88), (-19.35, 13.25, 248.82), (-20.2, 12.72, 248.77), (-21.05, 12.2, 248.71), (-21.85, 11.62, 248.56), (-22.45, 10.93, 248.18), (-22.86, 10.2, 247.63), (-23.15, 9.47, 247.02), (-23.36, 8.74, 246.36), (-23.51, 8.01, 245.69), (-23.52, 7.29, 245.0), (-23.52, 6.57, 244.3), (-23.52, 5.85, 243.61), (-23.51, 5.13, 242.91), (-23.48, 4.42, 242.21), (-23.41, 3.7, 241.52), (-23.35, 2.99, 240.82), (-23.28, 2.28, 240.12), (-23.21, 1.56, 239.42), (-23.14, 0.85, 238.72), (-23.07, 0.14, 238.02), (-23.0, -0.58, 237.33), (-22.93, -1.29, 236.63), (-22.86, -2.0, 235.93), (-22.79, -2.71, 235.23), (-22.72, -3.43, 234.53), (-22.64, -4.14, 233.83), (-22.59, -4.81, 233.09), (-22.69, -5.29, 232.22), (-22.87, -5.87, 231.43), (-23.0, -6.58, 230.74), (-22.83, -7.34, 230.12), (-22.41, -8.07, 229.58), (-21.84, -8.7, 229.07), (-20.99, -9.05, 228.66), (-20.12, -9.34, 228.27), (-19.19, -9.45, 228.01), (-18.2, -9.35, 227.92), (-17.7, -9.3, 227.87)],
    "usb_c": [(-23.2, -17.0, 259.4), (-23.2, -18.0, 259.4), (-23.04, -18.91, 259.16), (-22.73, -19.73, 258.67), (-22.41, -20.54, 258.19), (-22.1, -21.36, 257.7), (-21.75, -22.13, 257.18), (-21.34, -22.84, 256.6), (-20.91, -23.5, 255.99), (-20.44, -24.11, 255.35), (-19.95, -24.69, 254.69), (-19.47, -25.26, 254.03), (-18.93, -25.82, 253.4), (-18.35, -26.36, 252.79), (-17.77, -26.91, 252.19), (-17.18, -27.45, 251.59), (-16.38, -27.87, 251.17), (-15.44, -28.08, 250.99), (-14.44, -28.17, 250.96), (-13.44, -28.26, 250.93), (-12.45, -28.35, 250.89), (-11.45, -28.43, 250.88), (-10.46, -28.52, 250.86), (-9.46, -28.61, 250.85), (-8.46, -28.7, 250.83), (-7.47, -28.8, 250.83), (-6.47, -28.9, 250.82), (-5.48, -29.01, 250.81), (-4.48, -29.12, 250.8), (-3.49, -29.23, 250.8), (-2.5, -29.35, 250.8), (-1.51, -29.48, 250.8), (-0.51, -29.61, 250.81), (0.48, -29.74, 250.81), (1.47, -29.88, 250.82), (2.46, -30.03, 250.83), (3.44, -30.18, 250.84), (4.43, -30.34, 250.86), (5.42, -30.51, 250.88), (6.4, -30.68, 250.9), (7.39, -30.85, 250.92), (8.37, -31.02, 250.94), (9.36, -31.2, 250.97), (10.34, -31.38, 250.99), (11.32, -31.57, 251.02), (12.3, -31.76, 251.05), (13.29, -31.95, 251.08), (14.27, -32.14, 251.12), (15.25, -32.33, 251.15), (16.23, -32.53, 251.18), (17.21, -32.72, 251.22), (18.19, -32.91, 251.25), (19.18, -33.02, 251.27), (20.18, -33.06, 251.29), (21.18, -33.08, 251.3), (22.18, -33.11, 251.3), (23.18, -33.13, 251.31), (24.18, -33.15, 251.31), (25.18, -33.17, 251.32), (26.18, -33.19, 251.33), (27.18, -33.21, 251.33), (28.18, -33.22, 251.33), (29.18, -33.24, 251.33), (30.18, -33.26, 251.32), (31.18, -33.27, 251.32), (32.18, -33.28, 251.31), (33.18, -33.29, 251.3), (34.18, -33.29, 251.28), (35.18, -33.3, 251.26), (36.18, -33.3, 251.24), (37.18, -33.3, 251.21), (38.18, -33.3, 251.18), (39.18, -33.29, 251.15), (40.18, -33.28, 251.12), (41.18, -33.27, 251.08), (42.18, -33.26, 251.03), (43.18, -33.24, 250.98), (44.18, -33.22, 250.93), (45.18, -33.21, 250.87), (46.17, -33.19, 250.81), (47.17, -33.16, 250.75), (48.17, -33.14, 250.69), (49.17, -33.11, 250.62), (50.16, -33.07, 250.54), (51.16, -33.04, 250.46), (52.16, -33.0, 250.38), (53.09, -32.82, 250.09), (54.03, -32.64, 249.8), (54.97, -32.45, 249.5), (55.88, -32.23, 249.16), (56.79, -32.0, 248.79), (57.68, -31.76, 248.42), (58.58, -31.52, 248.05), (59.47, -31.27, 247.67), (60.36, -31.02, 247.28), (61.24, -30.77, 246.89), (62.12, -30.5, 246.49), (63.0, -30.24, 246.1), (63.88, -29.97, 245.69), (64.75, -29.7, 245.29), (65.62, -29.42, 244.88), (66.49, -29.13, 244.49), (67.4, -28.84, 244.17), (68.33, -28.57, 243.93), (69.28, -28.35, 243.74), (70.26, -28.18, 243.61), (71.25, -28.06, 243.54), (72.25, -28.0, 243.5), (73.25, -28.0, 243.5), (73.75, -28.0, 243.5)],
    "spk_pair": [(55.18, 3.06, 275.25), (55.18, 3.07, 274.25), (55.35, 3.3, 273.29), (55.36, 3.62, 272.35), (54.89, 4.12, 271.62), (54.3, 4.59, 270.98), (53.51, 5.0, 270.51), (52.71, 5.39, 270.05), (51.91, 5.79, 269.6), (51.12, 6.18, 269.14), (50.32, 6.58, 268.68), (49.52, 6.97, 268.22), (48.72, 7.36, 267.76), (47.91, 7.7, 267.28), (47.09, 7.82, 266.73), (46.37, 7.48, 266.13), (45.84, 6.83, 265.58), (45.44, 6.06, 265.09), (45.09, 5.25, 264.61), (44.81, 4.41, 264.15), (44.6, 3.49, 263.82), (44.47, 2.51, 263.76), (44.48, 1.56, 264.05), (44.75, 0.85, 264.7), (45.17, 0.34, 265.45), (45.7, 0.0, 266.23), (46.2, 0.0, 266.3)],
}


def connector_solids():
    """hr43：Radxa 端的针座/杜邦壳在 zz_adapter40 里（不再单独出 conn__dup_pin*）；功放直排针的壳在 zz_amp 里（不再出 conn__amp_*）；
    Radxa 口 / CSI 座按新位置（electronics.sbc_A_T 变换）出 ref__；USB 两端 90° 弯头公头出 conn__usb_plug_*（assumed 包络）。其余同 hr41。
    旧版 = connector_solids_was_until_2026_09_25_hr43。"""
    from . import electronics as E
    T = E.sbc_A_T(); out = {}
    for nm in SBC_EDGE_PORTS:
        m = sbc_edge_port_box(nm); m.apply_transform(T); out[f"ref__sbc_{nm}"] = m
    m = abox(*E.SBC_A_CSI_CONN); m.apply_transform(T); out["ref__sbc_csi_conn"] = m
    # was_until_2026_09_25_hr43c: out["conn__usb_plug_radxa"] = abox(HR43_USB_PLUG_RADXA["lo"], HR43_USB_PLUG_RADXA["hi"])
    out["conn__usb_plug_radxa"] = abox(HR43C_USB_PLUG_RADXA["lo"], HR43C_USB_PLUG_RADXA["hi"])   # hr43c：随板平移
    # was_until_2026_09_25_hr43d: out["conn__usb_plug_adp"] = abox(HR43_USB_PLUG_ADP["lo"], HR43_USB_PLUG_ADP["hi"])
    out["conn__usb_plug_adp"] = abox(HR43D_USB_PLUG_ADP["lo"], HR43D_USB_PLUG_ADP["hi"])        # hr43d：转接板平放后的弯头位
    for nm in ADP_PORT:
        if nm == "TypeC": continue          # hr43：Type-C 上插的是 conn__usb_plug_adp（90° 弯头），FPC 超薄头作废
        out[f"conn__adp_{nm}"] = adp_plug_box(nm)
    for k, p in JAW_PLUG.items():
        out[f"conn__jaw_plug_{k}"] = obox(p + np.array([0.0, PLUG_BODY[0] / 2, 0.0]), (0, 1, 0), JAW_PIN_ROW, PLUG_BODY)
    out["conn__roll8_plug_lo"] = obox((ROLL_IN["top_x"] + PLUG_BODY[0] / 2, ROLL_IN["y"], ROLL_IN["z"]), (1, 0, 0), (0, 1, 0), PLUG_BODY)
    for k, (c, ax) in enumerate(S1_JUNCTIONS_AT):
        out[f"conn__s1_junction{k + 1}"] = obox(c, ax, (0, 0, 1) if abs(ax[2]) < 0.9 else (0, 1, 0), S1_JUNCTION["size"])
    return out


_HR43_DROP = ("csi_fpc_a", "csi_fpc_b", "usb_fpc_a", "usb_fpc_b")       # 旧位 CSI / USB FPC 走线随 Radxa 搬走作废（新走法见 _HR43_CSI / _HR43_USB）
_HR43_STOW_DROP = ("stow_rear", "stow_mic_front", "stow_ubec_top")        # 旧余长盒：stow_rear 压在新 Radxa 上；麦克风 / UBEC 输出线重走后位置作废
_HR43_CSI_was_until_2026_09_25_hr43c = None                               # hr43：本轮未建
_HR43_CSI = None                                                          # hr43c：文件末尾赋 _hr43c_csi（排线 3 折候选路径）
_HR43_USB = None


def _apply_hr43():
    """_define() 末尾调：整根替换 _HR43_ROUTES 里的线（航点 / 两端说明 / 余长盒），删作废线与盘线区，加 hr43 CSI / USB。"""
    keep = []
    for w in WIRES:
        if w["id"] in _HR43_DROP: continue
        if w["id"] in _HR43_ROUTES:
            w["pts"] = [tuple(p) for p in _HR43_ROUTES[w["id"]]]; w["hr43_rerouted"] = True
            w.pop("sides", None); w["stow"] = None
            e0, e1 = w.get("ends") or ("", "")
            if w["id"].startswith("amp_"):
                w["ends"] = (f"功放直排针杜邦壳（板底 x 61.8..64.5，线弯区底 z {HR43_AMP_EXIT['z']}，hr43）", "Radxa 40P 直角转接横针上的杜邦壳（盒 B 尾 x 44，hr43）")
            elif w["id"].startswith(("mic_", "imu_", "ubec_out")):
                w["ends"] = (e0, "Radxa 40P 直角转接横针上的杜邦壳（盒 B 尾 x 44，hr43）")
            elif w["id"] == "bus_jaw_roll":
                w["sides"] = [np.array(JAW_PIN_ROW), np.array(JAW_PIN_ROW)] + [None] * (len(w["pts"]) - 4) + [np.array([0, 1.0, 0]), np.array([0, 1.0, 0])]
                w["stow"] = "stow_jaw_top"
            elif w["id"] == "spk_pair":
                w["ends"] = ("喇叭背面焊片（凸台底 z 275.25）", f"功放端子块 −x 面（x {HR43_AMP_TERM['x_face']}，线口 z {HR43_AMP_TERM['z_wire']}，hr43）")
            w["note"] = (w.get("note", "") + "｜hr43：按 Radxa 后脑 + 功放前口袋重走（hr43_work/routes43.json）").lstrip("｜")
        keep.append(w)
    WIRES[:] = keep
    if "usb_c" in _HR43_ROUTES:          # hr43：USB-C2 ↔ 转接板 Type-C（15 cm 90° 弯头公对公，圆线 Ø3.5 assumed；两头插头 conn__usb_plug_*）
        WIRES.append(dict(id="usb_c", harness="HB01（usb_link）", cable="usb_round", pts=[tuple(p) for p in _HR43_ROUTES["usb_c"]], bought=150.0,
                          bought_src="harness.yaml:HB01.usb_link_2026-09-24_hr41.purchase_spec『USB-C 公对公 15 cm』（assumed）", group="usb", hr43_rerouted=True,
                          ends=("Radxa USB-C2 90° 弯头（口朝 −x，线从 −y 面出，x −23.2 y −17 z 259.4）", "转接板 Type-C 90° 弯头（B 边 −y，线从 −x 面进，x 73.75 y −28 z 243.5）"),
                          note="hr43：沿 −y 侧从后脑到前口袋（route43_run.py）；圆线 Ø3.5 在头里弯不到 CABLE 声明的 R10（实走最小 R 见 check），要换细软线或 FPC —— 报告『待主设计定』"))
    for k in _HR43_STOW_DROP: STOWS.pop(k, None)
    if _HR43_CSI is not None: _HR43_CSI()
    if _HR43_USB is not None: _HR43_USB()


# ━━━━━━━━━━━━━━━━━━━━ hr43c（2026-09-25）后脑零鼓包：排针直插杜邦 + R2 针位 + 功放 −y 2.0（复审 #2 B1）+ 板挪 → 头内线重走 ━━━━━━━━━━━━━━━━━━━━
#   Radxa 端：14 个杜邦壳直插排针（electronics.gpio_dupont_boxes，壳顶 = 板面 + 16.5）；线出壳顶即弯（中心线 Rc：杜邦 2.0 / UBEC +5V 2.0 / UBEC GND 4.0）朝 +x，
#   弯段在 zz_gpio_dupont 占位里；本文件的线从「弯完接续点」electronics.gpio_dup_leg_end(针, s) 起（hr43 是从盒 B 尾 x 44 起），先 +x 直走到 x 15 再路由（hr43c/c5_route.py）。
#   一分二（12 / 35 / 17）单壳两线叠放：下腿 s 0、上腿 s 1.4；12/35 下腿 → 功放、上腿 → 麦；17 下腿 → 麦 VDD、上腿 → IMU V+。3 号跨 4 号多走 1.5。
#   改名（针号跟着改）：imu_pin6 → imu_pin14、mic_pin39 → mic_pin20、imu_pin1 → imu_pin17_splitleg、mic_pin17 → mic_pin17_splitleg。
#   功放 −y 2.0（electronics.AMP_S_C）→ 功放端 5 根、喇叭线进端子那头都 −y 2.0；板 (−0.5, +1.0, −0.5) → USB-C Radxa 端 90° 弯头随板。
HR43C_RENAME = {"imu_pin6": "imu_pin14", "mic_pin39": "mic_pin20", "imu_pin1": "imu_pin17_splitleg", "mic_pin17": "mic_pin17_splitleg"}
HR43C_LEG = {   # 线名 → (Radxa 针, 腿号)（腿号 = electronics.gpio_dup_legs 的 k）
    "amp_VIN_pin2": (2, 0), "amp_BCLK_pin12_splitleg": (12, 0), "amp_LRC_pin35_splitleg": (35, 0), "amp_GND_pin25": (25, 0), "amp_DIN_pin40": (40, 0),
    "mic_pin12_splitleg": (12, 1), "mic_pin35_splitleg": (35, 1), "mic_pin17_splitleg": (17, 0), "mic_pin38": (38, 0), "mic_pin9": (9, 0), "mic_pin20": (20, 0),
    "imu_pin3": (3, 0), "imu_pin5": (5, 0), "imu_pin14": (14, 0), "imu_pin17_splitleg": (17, 1),
    "ubec_out_5v": (4, 0), "ubec_out_gnd": (34, 0)}
HR43C_USB_PLUG_RADXA = dict(lo=(-27.7, -16.0, 255.65), hi=(-19.7, -4.0, 262.15), exit=(-23.7, -16.0, 258.9), out=(0.0, -1.0, 0.0),
                            src="hr43 HR43_USB_PLUG_RADXA（assumed 90° 弯头包络 12 × 6.5、口面外 8）随板平移 (−0.5, +1.0, −0.5)")
_HR43C_ROUTES = {   # hr43_work/hr43c/routes43c_fb.json（hr43c/c5_route.py → c6_fix_bends.py，1 mm 均匀航点；Radxa 端首/末点 = electronics.gpio_dup_leg_end）
    "ubec_out_5v": [(87.4, -31.8, 266.48), (87.4, -31.8, 267.48), (87.31, -31.8, 268.48), (87.11, -31.8, 269.46), (86.64, -31.8, 270.34), (86.08, -31.8, 271.17), (85.25, -31.8, 271.74), (84.37, -31.8, 272.21), (83.39, -31.8, 272.4), (82.4, -31.69, 272.5), (81.42, -31.47, 272.5), (80.49, -31.11, 272.5), (79.55, -30.74, 272.5), (78.62, -30.37, 272.5), (77.69, -30.0, 272.5), (76.76, -29.62, 272.5), (75.84, -29.23, 272.51), (74.91, -28.84, 272.51), (74.0, -28.43, 272.51), (73.08, -28.02, 272.51), (72.18, -27.58, 272.51), (71.28, -27.14, 272.52), (70.38, -26.69, 272.52), (69.5, -26.22, 272.53), (68.61, -25.75, 272.53), (67.74, -25.25, 272.54), (66.87, -24.75, 272.55), (66.01, -24.23, 272.56), (65.16, -23.7, 272.56), (64.31, -23.16, 272.57), (63.48, -22.6, 272.58), (62.65, -22.05, 272.59), (61.83, -21.46, 272.61), (61.02, -20.88, 272.62), (60.21, -20.28, 272.64), (59.42, -19.67, 272.66), (58.62, -19.05, 272.68), (57.84, -18.43, 272.7), (57.05, -17.8, 272.72), (56.28, -17.16, 272.74), (55.52, -16.52, 272.77), (54.75, -15.86, 272.8), (54.0, -15.21, 272.84), (53.24, -14.55, 272.87), (52.49, -13.88, 272.91), (51.75, -13.21, 272.94), (51.01, -12.53, 272.98), (50.27, -11.85, 273.02), (49.53, -11.18, 273.06), (48.8, -10.49, 273.11), (48.07, -9.8, 273.15), (47.35, -9.11, 273.2), (46.62, -8.42, 273.26), (45.9, -7.72, 273.31), (45.18, -7.03, 273.37), (44.45, -6.34, 273.42), (43.74, -5.64, 273.48), (43.02, -4.94, 273.55), (42.3, -4.24, 273.61), (41.58, -3.54, 273.67), (40.87, -2.84, 273.73), (40.15, -2.14, 273.79), (39.44, -1.44, 273.86), (38.72, -0.75, 273.92), (38.0, -0.05, 273.99), (37.28, 0.65, 274.06), (36.56, 1.35, 274.13), (35.85, 2.04, 274.19), (35.13, 2.74, 274.26), (34.41, 3.43, 274.32), (33.68, 4.13, 274.39), (32.96, 4.82, 274.45), (32.23, 5.51, 274.51), (31.5, 6.19, 274.57), (30.77, 6.88, 274.63), (30.04, 7.56, 274.69), (29.31, 8.25, 274.75), (28.58, 8.93, 274.8), (27.84, 9.61, 274.85), (27.1, 10.28, 274.91), (26.36, 10.96, 274.96), (25.62, 11.63, 275.0), (24.87, 12.3, 275.05), (24.13, 12.97, 275.09), (23.38, 13.64, 275.13), (22.63, 14.3, 275.17), (21.87, 14.96, 275.21), (21.12, 15.62, 275.25), (20.36, 16.28, 275.28), (19.61, 16.94, 275.32), (18.85, 17.6, 275.35), (18.09, 18.26, 275.39), (17.33, 18.91, 275.42), (16.58, 19.57, 275.45), (15.81, 20.22, 275.5), (14.98, 20.59, 275.8), (13.98, 20.59, 275.8), (12.97, 20.59, 275.8), (11.97, 20.59, 275.8), (11.47, 20.59, 275.8)],
    "ubec_out_gnd": [(87.4, -34.4, 266.48), (87.4, -34.4, 267.48), (87.31, -34.4, 268.48), (87.11, -34.4, 269.46), (86.65, -34.4, 270.33), (86.09, -34.4, 271.16), (85.26, -34.4, 271.73), (84.4, -34.4, 272.2), (83.41, -34.4, 272.4), (82.42, -34.43, 272.5), (81.42, -34.5, 272.5), (80.42, -34.47, 272.5), (79.42, -34.45, 272.5), (78.41, -34.42, 272.5), (77.41, -34.4, 272.5), (76.41, -34.38, 272.5), (75.41, -34.35, 272.5), (74.4, -34.33, 272.5), (73.4, -34.3, 272.5), (72.4, -34.26, 272.5), (71.4, -34.23, 272.5), (70.4, -34.19, 272.5), (69.39, -34.16, 272.5), (68.39, -34.11, 272.5), (67.39, -34.07, 272.51), (66.39, -34.02, 272.51), (65.39, -33.97, 272.51), (64.39, -33.91, 272.51), (63.39, -33.85, 272.51), (62.39, -33.79, 272.52), (61.39, -33.72, 272.52), (60.39, -33.64, 272.53), (59.39, -33.56, 272.54), (58.39, -33.46, 272.55), (57.39, -33.37, 272.56), (56.39, -33.25, 272.57), (55.4, -33.14, 272.58), (54.4, -33.01, 272.6), (53.41, -32.86, 272.62), (52.42, -32.72, 272.64), (51.43, -32.55, 272.67), (50.45, -32.37, 272.7), (49.46, -32.18, 272.73), (48.48, -31.98, 272.77), (47.5, -31.76, 272.81), (46.53, -31.52, 272.86), (45.56, -31.28, 272.92), (44.6, -31.0, 272.98), (43.63, -30.73, 273.04), (42.68, -30.43, 273.11), (41.73, -30.13, 273.19), (40.78, -29.8, 273.27), (39.85, -29.46, 273.37), (38.91, -29.11, 273.46), (37.99, -28.74, 273.57), (37.06, -28.37, 273.68), (36.15, -27.97, 273.8), (35.23, -27.58, 273.92), (34.33, -27.16, 274.05), (33.43, -26.75, 274.18), (32.53, -26.32, 274.32), (31.64, -25.88, 274.47), (30.75, -25.45, 274.62), (29.87, -25.0, 274.77), (28.98, -24.55, 274.93), (28.1, -24.1, 275.08), (27.22, -23.65, 275.24), (26.34, -23.19, 275.41), (25.47, -22.74, 275.57), (24.59, -22.28, 275.74), (23.72, -21.82, 275.91), (22.84, -21.36, 276.08), (21.97, -20.9, 276.25), (21.09, -20.44, 276.42), (20.22, -19.98, 276.58), (19.34, -19.52, 276.75), (18.47, -19.06, 276.92), (17.59, -18.6, 277.09), (16.72, -18.14, 277.26), (15.84, -17.68, 277.43), (14.97, -17.51, 277.8), (13.97, -17.51, 277.8), (13.47, -17.51, 277.8)],
    "amp_LRC_pin35_splitleg": [(8.93, -20.05, 275.8), (9.93, -20.05, 275.8), (10.93, -20.05, 275.8), (11.93, -20.05, 275.8), (12.93, -20.05, 275.8), (13.93, -20.05, 275.8), (14.93, -20.05, 275.8), (15.75, -20.45, 275.41), (16.68, -20.27, 275.08), (17.6, -20.08, 274.75), (18.53, -19.9, 274.42), (19.46, -19.72, 274.09), (20.39, -19.54, 273.77), (21.32, -19.36, 273.44), (22.25, -19.19, 273.11), (23.18, -19.02, 272.79), (24.11, -18.86, 272.46), (25.04, -18.7, 272.13), (25.97, -18.54, 271.81), (26.91, -18.39, 271.48), (27.84, -18.24, 271.14), (28.77, -18.1, 270.8), (29.7, -17.97, 270.46), (30.63, -17.84, 270.12), (31.56, -17.71, 269.76), (32.48, -17.59, 269.4), (33.4, -17.48, 269.03), (34.32, -17.37, 268.65), (35.24, -17.27, 268.25), (36.15, -17.17, 267.86), (37.05, -17.07, 267.44), (37.95, -16.98, 267.01), (38.84, -16.89, 266.56), (39.73, -16.8, 266.11), (40.61, -16.72, 265.64), (41.48, -16.64, 265.15), (42.35, -16.56, 264.66), (43.2, -16.49, 264.14), (44.05, -16.41, 263.62), (44.89, -16.35, 263.08), (45.73, -16.28, 262.53), (46.55, -16.21, 261.97), (47.37, -16.14, 261.4), (48.18, -16.08, 260.82), (48.99, -16.02, 260.24), (49.8, -15.95, 259.65), (50.6, -15.89, 259.06), (51.41, -15.83, 258.46), (52.2, -15.76, 257.86), (52.95, -15.67, 257.2), (53.2, -15.41, 256.28), (53.39, -15.13, 255.34), (53.57, -14.84, 254.4), (53.75, -14.55, 253.46), (53.94, -14.26, 252.52), (54.12, -13.97, 251.58), (54.3, -13.67, 250.64), (54.48, -13.37, 249.7), (54.65, -13.07, 248.76), (54.83, -12.76, 247.83), (55.0, -12.44, 246.9), (55.17, -12.12, 245.96), (55.34, -11.81, 245.03), (55.51, -11.48, 244.1), (55.68, -11.16, 243.16), (55.84, -10.83, 242.23), (56.0, -10.5, 241.3), (56.18, -10.17, 240.38), (56.48, -9.87, 239.48), (57.12, -9.66, 238.75), (58.0, -9.59, 238.29), (59.0, -9.62, 238.2), (60.0, -9.62, 238.2), (60.99, -9.62, 238.29), (61.95, -9.62, 238.59), (62.66, -9.62, 239.29), (63.02, -9.62, 240.22), (63.15, -9.62, 241.2), (63.15, -9.62, 241.7)],
    "amp_BCLK_pin12_splitleg": [(11.47, 10.43, 275.8), (12.48, 10.43, 275.8), (13.48, 10.43, 275.8), (14.49, 10.43, 275.8), (15.42, 10.49, 275.54), (16.25, 10.5, 274.97), (17.08, 10.5, 274.4), (17.9, 10.5, 273.82), (18.72, 10.5, 273.24), (19.55, 10.5, 272.66), (20.36, 10.5, 272.07), (21.17, 10.5, 271.48), (21.98, 10.5, 270.88), (22.79, 10.5, 270.28), (23.59, 10.5, 269.67), (24.39, 10.49, 269.06), (25.19, 10.49, 268.45), (25.98, 10.49, 267.82), (26.76, 10.48, 267.19), (27.54, 10.47, 266.56), (28.31, 10.46, 265.91), (29.08, 10.44, 265.26), (29.84, 10.43, 264.6), (30.6, 10.41, 263.94), (31.34, 10.39, 263.27), (32.09, 10.37, 262.59), (32.83, 10.34, 261.91), (33.56, 10.31, 261.22), (34.29, 10.28, 260.53), (35.02, 10.24, 259.83), (35.74, 10.21, 259.13), (36.45, 10.17, 258.42), (37.16, 10.13, 257.71), (37.88, 10.09, 257.0), (38.59, 10.04, 256.29), (39.29, 9.99, 255.57), (39.95, 9.91, 254.82), (40.62, 9.83, 254.07), (41.28, 9.74, 253.32), (41.94, 9.66, 252.56), (42.6, 9.58, 251.81), (43.26, 9.49, 251.06), (43.92, 9.4, 250.3), (44.58, 9.31, 249.54), (45.23, 9.22, 248.78), (45.89, 9.13, 248.03), (46.54, 9.04, 247.27), (47.17, 8.91, 246.49), (47.76, 8.75, 245.7), (48.29, 8.36, 244.97), (48.68, 7.62, 244.41), (49.07, 6.83, 243.93), (49.49, 5.99, 243.57), (49.93, 5.15, 243.22), (50.36, 4.31, 242.88), (50.79, 3.46, 242.54), (51.22, 2.62, 242.2), (51.66, 1.78, 241.87), (52.1, 0.93, 241.55), (52.54, 0.09, 241.23), (52.98, -0.76, 240.91), (53.42, -1.61, 240.6), (53.86, -2.46, 240.29), (54.3, -3.31, 239.98), (54.75, -4.16, 239.68), (55.19, -5.01, 239.37), (55.64, -5.86, 239.07), (56.18, -6.65, 238.77), (57.0, -7.13, 238.5), (57.98, -7.2, 238.29), (58.97, -7.08, 238.2), (59.97, -7.08, 238.2), (60.97, -7.08, 238.29), (61.93, -7.08, 238.57), (62.65, -7.08, 239.28), (63.02, -7.08, 240.21), (63.15, -7.08, 241.2), (63.15, -7.08, 241.7)],
    "amp_DIN_pin40": [(11.47, -25.13, 275.8), (12.47, -25.13, 275.8), (13.48, -25.13, 275.8), (14.48, -25.13, 275.8), (15.37, -25.39, 275.58), (16.31, -25.38, 275.26), (17.27, -25.24, 274.98), (18.22, -25.09, 274.7), (19.18, -24.95, 274.42), (20.13, -24.81, 274.15), (21.09, -24.67, 273.87), (22.04, -24.52, 273.59), (23.0, -24.38, 273.3), (23.95, -24.25, 273.02), (24.9, -24.1, 272.73), (25.85, -23.96, 272.45), (26.8, -23.82, 272.15), (27.76, -23.69, 271.86), (28.7, -23.55, 271.55), (29.65, -23.42, 271.25), (30.6, -23.28, 270.93), (31.53, -23.13, 270.61), (32.47, -22.99, 270.27), (33.4, -22.84, 269.93), (34.33, -22.7, 269.58), (35.26, -22.55, 269.21), (36.18, -22.39, 268.84), (37.09, -22.23, 268.44), (37.99, -22.06, 268.04), (38.89, -21.89, 267.62), (39.77, -21.71, 267.18), (40.65, -21.53, 266.73), (41.52, -21.33, 266.26), (42.38, -21.13, 265.78), (43.22, -20.91, 265.28), (44.05, -20.69, 264.76), (44.87, -20.45, 264.23), (45.69, -20.21, 263.69), (46.48, -19.96, 263.13), (47.27, -19.71, 262.56), (48.05, -19.44, 261.98), (48.82, -19.17, 261.39), (49.58, -18.9, 260.8), (50.33, -18.61, 260.2), (51.08, -18.33, 259.59), (51.83, -18.05, 258.98), (52.56, -17.76, 258.36), (53.27, -17.44, 257.73), (53.95, -17.09, 257.07), (54.45, -16.62, 256.34), (54.89, -16.11, 255.59), (55.18, -15.55, 254.82), (55.4, -14.96, 254.03), (55.63, -14.38, 253.24), (55.85, -13.8, 252.45), (56.03, -13.23, 251.65), (56.14, -12.67, 250.82), (56.24, -12.11, 249.99), (56.34, -11.56, 249.16), (56.44, -11.0, 248.33), (56.53, -10.45, 247.5), (56.63, -9.89, 246.66), (56.72, -9.34, 245.83), (56.82, -8.78, 245.0), (56.91, -8.23, 244.16), (57.0, -7.68, 243.33), (57.09, -7.13, 242.49), (57.18, -6.58, 241.66), (57.26, -6.03, 240.82), (57.41, -5.49, 239.98), (57.77, -5.01, 239.19), (58.51, -4.7, 238.6), (59.47, -4.58, 238.32), (60.47, -4.54, 238.24), (61.46, -4.54, 238.41), (62.3, -4.54, 238.93), (62.89, -4.54, 239.72), (63.15, -4.54, 240.7), (63.15, -4.54, 241.7)],
    "amp_GND_pin25": [(8.93, -7.35, 275.8), (9.94, -7.35, 275.8), (10.94, -7.35, 275.8), (11.95, -7.35, 275.8), (12.95, -7.35, 275.8), (13.96, -7.35, 275.8), (14.96, -7.36, 275.79), (15.83, -7.33, 275.34), (16.64, -6.91, 274.93), (17.46, -6.49, 274.53), (18.28, -6.07, 274.13), (19.1, -5.65, 273.72), (19.92, -5.22, 273.33), (20.74, -4.8, 272.93), (21.57, -4.38, 272.54), (22.39, -3.97, 272.15), (23.22, -3.55, 271.76), (24.05, -3.13, 271.38), (24.88, -2.71, 271.0), (25.71, -2.29, 270.62), (26.55, -1.87, 270.25), (27.38, -1.45, 269.88), (28.22, -1.03, 269.51), (29.06, -0.61, 269.15), (29.9, -0.19, 268.79), (30.73, 0.23, 268.42), (31.57, 0.65, 268.07), (32.41, 1.08, 267.71), (33.25, 1.5, 267.35), (34.09, 1.93, 267.0), (34.93, 2.35, 266.64), (35.76, 2.78, 266.28), (36.6, 3.2, 265.92), (37.44, 3.63, 265.56), (38.27, 4.05, 265.19), (39.1, 4.48, 264.83), (39.93, 4.91, 264.46), (40.76, 5.33, 264.08), (41.59, 5.76, 263.71), (42.42, 6.18, 263.32), (43.25, 6.6, 262.94), (44.08, 7.03, 262.56), (44.9, 7.45, 262.17), (45.72, 7.87, 261.77), (46.55, 8.3, 261.38), (47.37, 8.72, 260.98), (48.18, 9.14, 260.57), (49.0, 9.56, 260.17), (49.82, 9.97, 259.76), (50.64, 10.39, 259.35), (51.46, 10.81, 258.94), (52.23, 11.17, 258.45), (52.65, 11.16, 257.55), (52.97, 11.05, 256.61), (53.18, 10.76, 255.67), (53.37, 10.41, 254.75), (53.55, 10.06, 253.82), (53.74, 9.71, 252.9), (53.92, 9.35, 251.98), (54.1, 8.99, 251.06), (54.28, 8.62, 250.14), (54.46, 8.25, 249.23), (54.65, 7.87, 248.31), (54.82, 7.48, 247.4), (54.99, 7.1, 246.49), (55.17, 6.7, 245.58), (55.34, 6.3, 244.67), (55.51, 5.9, 243.77), (55.68, 5.5, 242.87), (55.85, 5.09, 241.96), (56.02, 4.68, 241.06), (56.19, 4.26, 240.16), (56.45, 3.85, 239.29), (57.08, 3.47, 238.62), (57.98, 3.21, 238.29), (58.97, 3.08, 238.2), (59.98, 3.08, 238.2), (60.97, 3.08, 238.29), (61.94, 3.08, 238.58), (62.65, 3.08, 239.28), (63.02, 3.08, 240.21), (63.15, 3.08, 241.2), (63.15, 3.08, 241.7)],
    "amp_VIN_pin2": [(11.47, 23.13, 275.8), (12.47, 23.13, 275.8), (13.47, 23.13, 275.8), (14.47, 23.13, 275.8), (15.36, 23.38, 275.59), (16.3, 23.35, 275.27), (17.24, 23.16, 275.01), (18.19, 22.98, 274.74), (19.14, 22.8, 274.47), (20.09, 22.62, 274.2), (21.03, 22.45, 273.93), (21.98, 22.27, 273.66), (22.92, 22.09, 273.39), (23.87, 21.92, 273.11), (24.81, 21.75, 272.83), (25.76, 21.58, 272.55), (26.7, 21.41, 272.26), (27.65, 21.25, 271.97), (28.59, 21.09, 271.67), (29.52, 20.93, 271.36), (30.46, 20.77, 271.04), (31.39, 20.61, 270.71), (32.32, 20.45, 270.38), (33.25, 20.3, 270.03), (34.17, 20.14, 269.68), (35.08, 19.98, 269.31), (35.99, 19.82, 268.92), (36.9, 19.66, 268.52), (37.79, 19.5, 268.11), (38.68, 19.33, 267.68), (39.56, 19.16, 267.24), (40.43, 19.0, 266.78), (41.3, 18.82, 266.3), (42.15, 18.64, 265.81), (42.99, 18.46, 265.3), (43.83, 18.27, 264.79), (44.65, 18.08, 264.25), (45.47, 17.89, 263.71), (46.27, 17.69, 263.15), (47.07, 17.49, 262.58), (47.86, 17.29, 262.0), (48.65, 17.09, 261.42), (49.43, 16.88, 260.82), (50.2, 16.67, 260.22), (50.97, 16.46, 259.62), (51.74, 16.25, 259.02), (52.39, 16.02, 258.3), (52.73, 15.74, 257.41), (52.99, 15.43, 256.49), (53.19, 15.05, 255.59), (53.37, 14.64, 254.69), (53.55, 14.23, 253.8), (53.72, 13.82, 252.9), (53.9, 13.4, 252.01), (54.07, 12.98, 251.12), (54.24, 12.54, 250.24), (54.41, 12.11, 249.35), (54.57, 11.66, 248.47), (54.74, 11.22, 247.59), (54.9, 10.76, 246.72), (55.06, 10.29, 245.85), (55.21, 9.82, 244.98), (55.37, 9.34, 244.11), (55.52, 8.86, 243.25), (55.67, 8.37, 242.39), (55.81, 7.88, 241.53), (55.96, 7.38, 240.68), (56.15, 6.88, 239.83), (56.54, 6.38, 239.06), (57.19, 5.95, 238.44), (58.02, 5.59, 238.02), (59.0, 5.62, 238.2), (60.0, 5.62, 238.2), (60.99, 5.62, 238.29), (61.94, 5.62, 238.59), (62.66, 5.62, 239.29), (63.02, 5.62, 240.22), (63.15, 5.62, 241.2), (63.15, 5.62, 241.7)],
    "imu_pin3": [(40.54, 17.55, 215.43), (39.85, 18.18, 215.78), (39.15, 18.81, 216.13), (38.74, 19.26, 216.92), (38.39, 19.67, 217.76), (38.15, 20.06, 218.65), (37.98, 20.42, 219.57), (37.8, 20.77, 220.49), (37.61, 21.06, 221.43), (37.43, 21.35, 222.37), (37.18, 21.6, 223.3), (36.93, 21.84, 224.24), (36.61, 22.06, 225.17), (36.23, 22.24, 226.07), (35.83, 22.42, 226.97), (35.3, 22.51, 227.82), (34.77, 22.59, 228.66), (34.17, 22.59, 229.46), (33.54, 22.55, 230.23), (32.89, 22.47, 230.99), (32.22, 22.31, 231.72), (31.54, 22.15, 232.44), (30.86, 21.93, 233.13), (30.17, 21.7, 233.82), (29.48, 21.46, 234.51), (28.79, 21.21, 235.19), (28.11, 20.97, 235.88), (27.43, 20.75, 236.58), (26.75, 20.54, 237.28), (26.07, 20.36, 238.0), (25.41, 20.21, 238.73), (24.76, 20.07, 239.48), (24.14, 19.96, 240.25), (23.53, 19.85, 241.04), (22.95, 19.75, 241.85), (22.41, 19.63, 242.69), (21.86, 19.69, 243.5), (21.27, 19.92, 244.28), (20.69, 20.15, 245.05), (20.25, 20.74, 245.49), (20.05, 21.72, 245.49), (19.85, 22.7, 245.48), (19.65, 23.68, 245.47), (19.46, 24.66, 245.47), (19.26, 25.64, 245.47), (19.07, 26.61, 245.61), (18.88, 27.57, 245.83), (18.7, 28.53, 246.07), (18.52, 29.48, 246.3), (18.34, 30.44, 246.53), (18.15, 31.39, 246.77), (17.97, 32.35, 247.0), (17.79, 33.3, 247.24), (17.58, 34.17, 247.67), (17.38, 34.67, 248.51), (17.21, 34.93, 249.45), (17.13, 34.72, 250.42), (17.09, 34.4, 251.37), (17.09, 33.96, 252.27), (17.09, 33.5, 253.16), (17.09, 33.03, 254.04), (17.09, 32.57, 254.93), (17.1, 32.1, 255.81), (17.11, 31.63, 256.69), (17.12, 31.15, 257.57), (17.14, 30.67, 258.45), (17.16, 30.18, 259.33), (17.18, 29.7, 260.2), (17.21, 29.2, 261.07), (17.24, 28.71, 261.94), (17.27, 28.21, 262.81), (17.31, 27.7, 263.67), (17.34, 27.2, 264.53), (17.39, 26.69, 265.39), (17.43, 26.18, 266.25), (17.48, 25.66, 267.11), (17.54, 25.15, 267.96), (17.59, 24.63, 268.82), (17.64, 24.11, 269.67), (17.7, 23.59, 270.52), (17.75, 23.07, 271.38), (17.81, 22.55, 272.23), (17.87, 22.03, 273.08), (17.94, 21.51, 273.93), (17.99, 20.99, 274.79), (17.69, 20.71, 275.68), (17.19, 20.57, 276.54), (16.33, 20.53, 277.05), (15.41, 20.52, 277.44), (14.43, 20.59, 277.3), (13.43, 20.59, 277.3), (12.43, 20.59, 277.3), (11.43, 20.59, 277.3), (10.43, 20.59, 277.3), (9.43, 20.59, 277.3), (8.93, 20.59, 277.3)],
    "imu_pin5": [(42.3, 17.5, 215.5), (42.07, 16.88, 216.25), (41.83, 16.26, 217.01), (41.63, 15.95, 217.89), (41.46, 15.94, 218.88), (41.3, 15.94, 219.86), (41.23, 16.0, 220.86), (41.16, 16.06, 221.86), (41.19, 16.05, 222.86), (41.27, 16.01, 223.86), (41.37, 15.96, 224.85), (41.52, 15.9, 225.84), (41.67, 15.83, 226.83), (41.71, 15.86, 227.83), (41.72, 15.92, 228.83), (41.57, 16.04, 229.81), (41.2, 16.23, 230.72), (40.73, 16.42, 231.58), (39.95, 16.55, 232.2), (39.14, 16.66, 232.77), (38.2, 16.67, 233.14), (37.25, 16.66, 233.45), (36.28, 16.63, 233.7), (35.3, 16.59, 233.89), (34.31, 16.54, 234.07), (33.32, 16.49, 234.2), (32.33, 16.45, 234.32), (31.33, 16.42, 234.44), (30.34, 16.41, 234.55), (29.34, 16.39, 234.67), (28.35, 16.41, 234.82), (27.36, 16.42, 234.99), (26.39, 16.46, 235.24), (25.44, 16.51, 235.55), (24.5, 16.58, 235.89), (23.63, 16.68, 236.37), (22.77, 16.79, 236.87), (21.99, 16.95, 237.48), (21.29, 17.14, 238.17), (20.66, 17.37, 238.91), (20.2, 17.67, 239.75), (19.79, 17.99, 240.6), (19.54, 18.37, 241.5), (19.35, 18.74, 242.41), (19.09, 19.14, 243.29), (18.76, 19.79, 243.96), (18.52, 20.72, 244.22), (18.39, 21.71, 244.24), (18.33, 22.71, 244.18), (18.29, 23.71, 244.11), (18.25, 24.71, 244.04), (18.2, 25.71, 244.0), (18.14, 26.71, 243.99), (18.07, 27.71, 244.0), (17.99, 28.71, 244.05), (17.91, 29.71, 244.1), (17.83, 30.7, 244.15), (17.75, 31.7, 244.2), (17.62, 32.69, 244.3), (17.5, 33.68, 244.4), (17.12, 34.46, 244.85), (16.57, 34.91, 245.56), (16.08, 35.05, 246.42), (15.65, 35.03, 247.33), (15.22, 35.02, 248.23), (14.81, 34.97, 249.14), (14.46, 34.8, 250.07), (14.22, 34.49, 250.98), (14.2, 33.94, 251.82), (14.23, 33.35, 252.63), (14.25, 32.76, 253.44), (14.28, 32.17, 254.25), (14.31, 31.58, 255.06), (14.34, 30.99, 255.87), (14.37, 30.41, 256.68), (14.4, 29.82, 257.5), (14.44, 29.24, 258.31), (14.47, 28.66, 259.13), (14.51, 28.08, 259.94), (14.55, 27.5, 260.76), (14.58, 26.92, 261.58), (14.63, 26.35, 262.4), (14.67, 25.77, 263.22), (14.72, 25.2, 264.04), (14.77, 24.64, 264.87), (14.82, 24.07, 265.7), (14.87, 23.51, 266.52), (14.92, 22.95, 267.35), (14.97, 22.39, 268.18), (15.03, 21.83, 269.01), (15.09, 21.27, 269.84), (15.15, 20.71, 270.67), (15.21, 20.15, 271.5), (15.27, 19.6, 272.33), (15.29, 19.07, 273.18), (15.16, 18.59, 274.05), (14.7, 18.25, 274.87), (13.89, 18.09, 275.43), (12.93, 18.05, 275.7), (11.94, 18.05, 275.8), (10.93, 18.05, 275.8), (9.93, 18.05, 275.8), (8.93, 18.05, 275.8)],
    "imu_pin14": [(44.14, 17.56, 215.44), (43.82, 18.39, 215.9), (43.5, 19.22, 216.35), (43.14, 20.03, 216.82), (42.73, 20.81, 217.29), (42.31, 21.59, 217.76), (41.81, 22.3, 218.25), (41.3, 23.01, 218.74), (40.59, 23.31, 219.37), (39.86, 23.47, 220.03), (39.23, 23.58, 220.8), (38.64, 23.66, 221.6), (38.04, 23.79, 222.4), (37.45, 23.93, 223.19), (36.84, 24.1, 223.96), (36.18, 24.3, 224.69), (35.52, 24.5, 225.41), (34.64, 24.65, 225.86), (33.71, 24.78, 226.21), (32.78, 24.91, 226.53), (31.82, 25.04, 226.81), (30.87, 25.17, 227.09), (29.9, 25.3, 227.32), (28.94, 25.42, 227.54), (27.97, 25.56, 227.77), (27.01, 25.7, 228.01), (26.06, 25.86, 228.25), (25.13, 26.06, 228.57), (24.21, 26.28, 228.91), (23.35, 26.56, 229.32), (22.53, 26.89, 229.79), (21.75, 27.27, 230.28), (21.06, 27.74, 230.84), (20.4, 28.23, 231.41), (20.03, 28.83, 232.12), (19.71, 29.44, 232.85), (19.45, 30.09, 233.56), (19.24, 30.78, 234.26), (19.04, 31.46, 234.95), (18.87, 32.16, 235.65), (18.7, 32.85, 236.35), (18.53, 33.5, 237.1), (18.37, 34.13, 237.86), (18.2, 34.71, 238.66), (18.02, 35.2, 239.51), (17.84, 35.67, 240.37), (17.67, 36.01, 241.3), (17.5, 36.32, 242.23), (17.34, 36.56, 243.19), (17.09, 36.76, 244.14), (16.57, 36.83, 244.99), (15.78, 36.6, 245.54), (14.95, 36.14, 245.85), (14.23, 35.51, 246.14), (13.92, 34.77, 246.72), (13.9, 34.05, 247.41), (13.9, 33.33, 248.1), (13.92, 32.62, 248.8), (13.93, 31.9, 249.5), (13.95, 31.18, 250.2), (13.96, 30.47, 250.9), (13.98, 29.75, 251.6), (14.0, 29.04, 252.29), (14.03, 28.32, 253.0), (14.05, 27.61, 253.7), (14.07, 26.9, 254.41), (14.1, 26.2, 255.11), (14.13, 25.49, 255.82), (14.16, 24.79, 256.53), (14.19, 24.09, 257.24), (14.23, 23.39, 257.96), (14.26, 22.69, 258.68), (14.3, 21.99, 259.39), (14.34, 21.3, 260.11), (14.39, 20.61, 260.84), (14.43, 19.92, 261.56), (14.48, 19.24, 262.29), (14.52, 18.56, 263.02), (14.57, 17.88, 263.75), (14.62, 17.2, 264.49), (14.67, 16.53, 265.23), (14.73, 15.86, 265.97), (14.78, 15.19, 266.71), (14.84, 14.52, 267.46), (14.89, 13.86, 268.2), (14.95, 13.2, 268.95), (15.02, 12.54, 269.7), (15.08, 11.88, 270.45), (15.14, 11.22, 271.2), (15.21, 10.57, 271.95), (15.27, 9.91, 272.71), (15.29, 9.28, 273.48), (14.85, 8.82, 274.25), (14.41, 8.35, 275.02), (13.97, 7.89, 275.8), (12.97, 7.89, 275.8), (11.97, 7.89, 275.8), (11.47, 7.89, 275.8)],
    "imu_pin17_splitleg": [(38.8, 17.03, 215.42), (38.66, 17.27, 216.39), (38.53, 17.51, 217.35), (38.4, 17.74, 218.32), (38.28, 17.96, 219.29), (38.17, 18.18, 220.26), (38.07, 18.37, 221.24), (37.98, 18.56, 222.23), (37.9, 18.73, 223.21), (37.82, 18.87, 224.2), (37.75, 19.01, 225.19), (37.68, 19.1, 226.19), (37.61, 19.2, 227.19), (37.52, 19.25, 228.19), (37.43, 19.29, 229.19), (37.34, 19.32, 230.18), (37.22, 19.31, 231.18), (37.09, 19.3, 232.18), (36.96, 19.27, 233.17), (36.81, 19.23, 234.16), (36.66, 19.19, 235.15), (36.49, 19.13, 236.14), (36.32, 19.08, 237.13), (36.14, 19.03, 238.12), (35.97, 18.98, 239.1), (35.77, 18.94, 240.09), (35.36, 18.88, 241.0), (34.69, 18.82, 241.73), (33.78, 18.76, 242.12), (32.79, 18.72, 242.09), (31.85, 18.71, 241.75), (31.05, 18.74, 241.16), (30.32, 18.79, 240.46), (29.6, 18.84, 239.77), (28.88, 18.89, 239.07), (28.16, 18.94, 238.37), (27.43, 19.02, 237.69), (26.69, 19.15, 237.02), (25.94, 19.38, 236.4), (25.22, 19.91, 235.99), (24.56, 20.63, 235.76), (23.91, 21.37, 235.55), (23.27, 22.11, 235.35), (22.62, 22.85, 235.14), (21.97, 23.59, 234.93), (21.33, 24.33, 234.73), (20.69, 25.08, 234.53), (20.07, 25.84, 234.3), (19.54, 26.65, 234.03), (19.06, 27.48, 233.74), (18.58, 28.31, 233.44), (18.12, 29.16, 233.19), (17.81, 30.11, 233.15), (17.63, 31.08, 233.32), (17.41, 32.0, 233.65), (17.15, 32.85, 234.12), (16.88, 33.67, 234.63), (16.58, 34.46, 235.17), (16.13, 34.95, 235.89), (15.57, 35.03, 236.72), (14.99, 35.04, 237.54), (14.43, 35.02, 238.37), (13.98, 34.75, 239.21), (13.68, 34.24, 240.02), (13.51, 33.64, 240.81), (13.46, 33.0, 241.58), (13.44, 32.35, 242.34), (13.42, 31.69, 243.1), (13.4, 31.03, 243.86), (13.39, 30.37, 244.62), (13.38, 29.72, 245.37), (13.38, 29.06, 246.13), (13.38, 28.4, 246.89), (13.39, 27.74, 247.65), (13.4, 27.08, 248.4), (13.42, 26.42, 249.16), (13.44, 25.76, 249.92), (13.47, 25.1, 250.67), (13.51, 24.44, 251.42), (13.55, 23.78, 252.18), (13.59, 23.11, 252.93), (13.64, 22.45, 253.68), (13.69, 21.79, 254.43), (13.75, 21.13, 255.19), (13.81, 20.47, 255.95), (13.88, 19.82, 256.7), (13.95, 19.16, 257.45), (14.03, 18.5, 258.21), (14.1, 17.84, 258.97), (14.18, 17.19, 259.72), (14.26, 16.54, 260.48), (14.34, 15.89, 261.24), (14.43, 15.24, 262.0), (14.52, 14.6, 262.77), (14.61, 13.95, 263.53), (14.71, 13.31, 264.3), (14.8, 12.67, 265.07), (14.89, 12.04, 265.84), (14.99, 11.4, 266.61), (15.09, 10.77, 267.39), (15.18, 10.14, 268.16), (15.28, 9.52, 268.94), (15.38, 8.89, 269.72), (15.48, 8.27, 270.5), (15.58, 7.64, 271.28), (15.68, 7.02, 272.06), (15.78, 6.4, 272.84), (15.88, 5.78, 273.62), (15.98, 5.16, 274.41), (16.07, 4.54, 275.19), (16.09, 3.9, 275.97), (15.83, 3.25, 276.68), (15.4, 2.65, 277.34), (14.45, 2.81, 277.2), (13.45, 2.81, 277.2), (12.44, 2.81, 277.2), (11.44, 2.81, 277.2), (10.44, 2.81, 277.2), (9.43, 2.81, 277.2), (8.93, 2.81, 277.2)],
    "mic_pin38": [(85.13, 31.58, 254.76), (85.13, 31.58, 255.76), (84.95, 31.41, 256.7), (84.24, 31.16, 257.36), (83.45, 30.94, 257.94), (82.65, 30.72, 258.51), (81.85, 30.5, 259.08), (81.05, 30.28, 259.64), (80.25, 30.06, 260.2), (79.44, 29.81, 260.74), (78.63, 29.56, 261.28), (77.8, 29.22, 261.72), (76.95, 28.8, 262.08), (76.11, 28.38, 262.43), (75.27, 27.96, 262.78), (74.43, 27.54, 263.13), (73.59, 27.11, 263.48), (72.74, 26.68, 263.82), (71.9, 26.25, 264.16), (71.06, 25.81, 264.49), (70.22, 25.38, 264.82), (69.38, 24.94, 265.15), (68.54, 24.5, 265.48), (67.7, 24.06, 265.81), (66.87, 23.55, 266.06), (66.07, 22.96, 266.23), (65.27, 22.37, 266.38), (64.48, 21.78, 266.54), (63.68, 21.18, 266.69), (62.89, 20.58, 266.83), (62.1, 19.98, 266.97), (61.31, 19.37, 267.1), (60.53, 18.75, 267.23), (59.75, 18.13, 267.35), (58.98, 17.5, 267.46), (58.2, 16.86, 267.57), (57.44, 16.22, 267.67), (56.67, 15.58, 267.77), (55.91, 14.93, 267.86), (55.15, 14.27, 267.94), (54.4, 13.61, 268.02), (53.65, 12.94, 268.09), (52.91, 12.27, 268.16), (52.16, 11.6, 268.22), (51.42, 10.93, 268.29), (50.68, 10.25, 268.35), (49.95, 9.56, 268.4), (49.22, 8.88, 268.46), (48.49, 8.19, 268.51), (47.76, 7.5, 268.56), (47.03, 6.8, 268.61), (46.31, 6.11, 268.65), (45.58, 5.42, 268.7), (44.86, 4.72, 268.74), (44.14, 4.02, 268.79), (43.42, 3.32, 268.84), (42.71, 2.61, 268.89), (41.99, 1.91, 268.94), (41.27, 1.21, 268.98), (40.56, 0.51, 269.03), (39.84, -0.2, 269.08), (39.13, -0.9, 269.13), (38.41, -1.6, 269.19), (37.69, -2.31, 269.24), (36.98, -3.01, 269.3), (36.27, -3.72, 269.37), (35.55, -4.42, 269.44), (34.84, -5.12, 269.5), (34.12, -5.83, 269.57), (33.41, -6.53, 269.65), (32.69, -7.23, 269.74), (31.98, -7.93, 269.82), (31.26, -8.63, 269.92), (30.55, -9.33, 270.01), (29.83, -10.03, 270.12), (29.12, -10.72, 270.23), (28.4, -11.42, 270.34), (27.69, -12.12, 270.46), (26.97, -12.81, 270.59), (26.26, -13.5, 270.72), (25.54, -14.2, 270.85), (24.82, -14.89, 270.99), (24.11, -15.58, 271.13), (23.39, -16.27, 271.28), (22.68, -16.96, 271.43), (21.96, -17.65, 271.59), (21.25, -18.34, 271.74), (20.53, -19.02, 271.9), (19.82, -19.71, 272.06), (19.1, -20.39, 272.25), (18.38, -21.05, 272.47), (17.73, -21.39, 273.15), (17.07, -21.72, 273.84), (16.42, -22.04, 274.53), (15.77, -22.37, 275.22), (14.98, -22.59, 275.79), (13.98, -22.59, 275.8), (12.98, -22.59, 275.8), (11.97, -22.59, 275.8), (11.47, -22.59, 275.8)],
    "mic_pin17_splitleg": [(87.67, 31.58, 254.76), (87.67, 31.58, 255.77), (87.37, 31.39, 256.61), (86.71, 30.85, 257.15), (86.06, 30.31, 257.69), (85.31, 29.84, 258.16), (84.54, 29.38, 258.62), (83.78, 28.91, 259.09), (83.02, 28.45, 259.55), (82.25, 27.99, 260.01), (81.47, 27.54, 260.46), (80.69, 27.09, 260.91), (79.9, 26.65, 261.35), (79.12, 26.21, 261.79), (78.32, 25.78, 262.22), (77.51, 25.35, 262.65), (76.7, 24.94, 263.06), (75.88, 24.53, 263.47), (75.05, 24.12, 263.88), (74.21, 23.73, 264.27), (73.37, 23.34, 264.66), (72.52, 22.96, 265.03), (71.66, 22.59, 265.4), (70.79, 22.23, 265.76), (69.92, 21.88, 266.11), (69.04, 21.53, 266.46), (68.16, 21.19, 266.8), (67.27, 20.86, 267.13), (66.38, 20.53, 267.46), (65.48, 20.21, 267.78), (64.59, 19.89, 268.09), (63.69, 19.57, 268.41), (62.79, 19.25, 268.73), (61.88, 18.95, 269.02), (60.9, 18.76, 269.17), (59.93, 18.57, 269.32), (58.95, 18.38, 269.47), (57.98, 18.19, 269.62), (57.0, 18.0, 269.76), (56.02, 17.8, 269.9), (55.04, 17.61, 270.03), (54.07, 17.42, 270.16), (53.09, 17.23, 270.28), (52.11, 17.03, 270.39), (51.13, 16.83, 270.51), (50.15, 16.62, 270.61), (49.17, 16.41, 270.71), (48.2, 16.2, 270.81), (47.22, 15.98, 270.9), (46.25, 15.75, 270.99), (45.27, 15.51, 271.07), (44.3, 15.27, 271.16), (43.33, 15.01, 271.23), (42.37, 14.75, 271.31), (41.4, 14.47, 271.39), (40.44, 14.18, 271.46), (39.49, 13.89, 271.54), (38.53, 13.57, 271.62), (37.58, 13.26, 271.71), (36.64, 12.91, 271.8), (35.7, 12.57, 271.88), (34.77, 12.21, 271.98), (33.84, 11.83, 272.07), (32.92, 11.45, 272.17), (32.0, 11.06, 272.28), (31.08, 10.66, 272.39), (30.18, 10.24, 272.51), (29.27, 9.82, 272.63), (28.38, 9.38, 272.77), (27.49, 8.94, 272.91), (26.6, 8.49, 273.05), (25.71, 8.04, 273.2), (24.83, 7.58, 273.35), (23.96, 7.11, 273.51), (23.08, 6.64, 273.67), (22.21, 6.17, 273.84), (21.34, 5.69, 274.01), (20.48, 5.21, 274.18), (19.61, 4.74, 274.35), (18.75, 4.25, 274.53), (17.88, 3.77, 274.71), (17.03, 3.3, 274.94), (16.18, 2.86, 275.25), (15.35, 2.59, 275.59), (14.46, 2.81, 275.8), (13.45, 2.81, 275.8), (12.45, 2.81, 275.8), (11.44, 2.81, 275.8), (10.44, 2.81, 275.8), (9.43, 2.81, 275.8), (8.93, 2.81, 275.8)],
    "mic_pin20": [(90.21, 31.58, 254.76), (90.21, 31.58, 255.76), (90.26, 31.51, 256.76), (89.98, 31.19, 257.65), (89.33, 30.71, 258.25), (88.61, 30.23, 258.75), (87.88, 29.74, 259.23), (87.14, 29.25, 259.71), (86.4, 28.76, 260.19), (85.66, 28.28, 260.66), (84.91, 27.8, 261.13), (84.16, 27.32, 261.59), (83.4, 26.85, 262.05), (82.63, 26.38, 262.49), (81.86, 25.91, 262.94), (81.07, 25.45, 263.36), (80.28, 25.0, 263.79), (79.48, 24.55, 264.2), (78.68, 24.1, 264.61), (77.87, 23.66, 265.0), (77.05, 23.23, 265.39), (76.22, 22.8, 265.76), (75.38, 22.38, 266.12), (74.54, 21.96, 266.47), (73.68, 21.55, 266.8), (72.82, 21.14, 267.13), (71.95, 20.75, 267.43), (71.08, 20.35, 267.73), (70.19, 19.97, 268.01), (69.3, 19.59, 268.28), (68.4, 19.22, 268.53), (67.5, 18.85, 268.76), (66.59, 18.48, 268.99), (65.68, 18.13, 269.2), (64.76, 17.77, 269.41), (63.83, 17.42, 269.59), (62.91, 17.08, 269.77), (61.97, 16.74, 269.93), (61.04, 16.4, 270.08), (60.1, 16.07, 270.22), (59.16, 15.75, 270.35), (58.22, 15.42, 270.47), (57.27, 15.1, 270.58), (56.33, 14.78, 270.69), (55.38, 14.46, 270.78), (54.43, 14.14, 270.87), (53.48, 13.83, 270.95), (52.53, 13.51, 271.02), (51.58, 13.2, 271.09), (50.62, 12.89, 271.16), (49.67, 12.58, 271.23), (48.72, 12.27, 271.29), (47.76, 11.95, 271.35), (46.81, 11.64, 271.41), (45.86, 11.32, 271.46), (44.91, 11.0, 271.52), (43.96, 10.68, 271.58), (43.01, 10.36, 271.64), (42.06, 10.03, 271.7), (41.11, 9.71, 271.77), (40.16, 9.38, 271.83), (39.22, 9.06, 271.9), (38.27, 8.73, 271.97), (37.33, 8.39, 272.05), (36.38, 8.06, 272.13), (35.44, 7.73, 272.21), (34.49, 7.39, 272.29), (33.55, 7.06, 272.38), (32.61, 6.73, 272.46), (31.67, 6.39, 272.58), (30.73, 6.05, 272.72), (29.8, 5.7, 272.87), (28.87, 5.36, 273.02), (27.94, 5.01, 273.17), (27.0, 4.68, 273.32), (26.07, 4.34, 273.48), (25.14, 4.0, 273.64), (24.21, 3.66, 273.81), (23.28, 3.32, 273.97), (22.35, 2.98, 274.15), (21.42, 2.64, 274.33), (20.49, 2.31, 274.51), (19.56, 1.97, 274.69), (18.63, 1.63, 274.87), (17.7, 1.29, 275.06), (16.78, 0.96, 275.24), (15.85, 0.62, 275.43), (14.99, 0.27, 275.8), (13.98, 0.27, 275.8), (12.98, 0.27, 275.8), (11.97, 0.27, 275.8), (11.47, 0.27, 275.8)],
    "mic_pin9": [(85.13, 39.04, 254.76), (85.13, 39.04, 255.76), (84.81, 38.98, 256.64), (84.08, 38.67, 257.25), (83.31, 38.25, 257.73), (82.53, 37.82, 258.18), (81.74, 37.37, 258.63), (80.96, 36.93, 259.06), (80.17, 36.49, 259.5), (79.37, 36.06, 259.93), (78.58, 35.63, 260.36), (77.78, 35.2, 260.79), (76.98, 34.77, 261.22), (76.18, 34.35, 261.64), (75.37, 33.92, 262.06), (74.57, 33.5, 262.48), (73.76, 33.08, 262.9), (72.94, 32.67, 263.31), (72.13, 32.26, 263.72), (71.31, 31.85, 264.13), (70.49, 31.44, 264.54), (69.67, 31.03, 264.95), (68.85, 30.62, 265.35), (68.03, 30.21, 265.76), (67.14, 29.86, 266.06), (66.18, 29.61, 266.18), (65.22, 29.35, 266.31), (64.26, 29.1, 266.43), (63.3, 28.84, 266.56), (62.34, 28.57, 266.68), (61.38, 28.31, 266.81), (60.43, 28.03, 266.94), (59.47, 27.75, 267.06), (58.52, 27.47, 267.19), (57.57, 27.18, 267.32), (56.62, 26.88, 267.45), (55.67, 26.58, 267.58), (54.73, 26.27, 267.71), (53.79, 25.95, 267.84), (52.85, 25.63, 267.98), (51.91, 25.29, 268.11), (50.98, 24.96, 268.24), (50.04, 24.62, 268.38), (49.11, 24.27, 268.52), (48.19, 23.92, 268.66), (47.26, 23.56, 268.8), (46.34, 23.2, 268.95), (45.42, 22.83, 269.1), (44.5, 22.46, 269.24), (43.58, 22.09, 269.39), (42.66, 21.72, 269.54), (41.74, 21.34, 269.69), (40.83, 20.96, 269.84), (39.92, 20.57, 270.0), (39.0, 20.19, 270.15), (38.09, 19.8, 270.3), (37.18, 19.42, 270.45), (36.26, 19.04, 270.61), (35.32, 18.74, 270.78), (34.38, 18.46, 270.95), (33.43, 18.18, 271.13), (32.48, 17.9, 271.31), (31.54, 17.62, 271.49), (30.59, 17.34, 271.67), (29.65, 17.05, 271.85), (28.7, 16.77, 272.04), (27.76, 16.49, 272.22), (26.82, 16.21, 272.41), (25.87, 15.93, 272.6), (24.93, 15.65, 272.79), (23.99, 15.37, 272.98), (23.04, 15.09, 273.17), (22.1, 14.8, 273.36), (21.17, 14.51, 273.59), (20.27, 14.19, 273.89), (19.37, 13.87, 274.2), (18.47, 13.55, 274.5), (17.57, 13.23, 274.8), (16.67, 12.91, 275.1), (15.77, 12.6, 275.41), (14.94, 12.96, 275.8), (13.94, 12.97, 275.8), (12.94, 12.97, 275.8), (11.94, 12.97, 275.8), (10.93, 12.97, 275.8), (9.93, 12.97, 275.8), (8.93, 12.97, 275.8)],
    "mic_pin35_splitleg": [(87.67, 39.04, 254.76), (87.67, 39.04, 255.76), (87.32, 39.0, 256.62), (86.67, 38.55, 257.24), (86.08, 37.95, 257.78), (85.5, 37.33, 258.31), (84.91, 36.7, 258.83), (84.33, 36.07, 259.35), (83.74, 35.44, 259.86), (83.15, 34.81, 260.37), (82.55, 34.17, 260.87), (81.96, 33.54, 261.38), (81.35, 32.91, 261.87), (80.75, 32.27, 262.35), (80.14, 31.64, 262.83), (79.52, 31.0, 263.31), (78.9, 30.36, 263.78), (78.28, 29.72, 264.23), (77.65, 29.08, 264.68), (77.02, 28.45, 265.13), (76.38, 27.81, 265.57), (75.74, 27.17, 266.0), (75.09, 26.53, 266.42), (74.44, 25.88, 266.84), (73.79, 25.24, 267.25), (73.13, 24.6, 267.65), (72.47, 23.95, 268.05), (71.81, 23.31, 268.44), (71.15, 22.67, 268.83), (70.48, 22.02, 269.22), (69.81, 21.38, 269.6), (69.14, 20.74, 269.98), (68.47, 20.1, 270.36), (67.75, 19.46, 270.65), (67.0, 18.83, 270.87), (66.22, 18.22, 271.04), (65.42, 17.63, 271.15), (64.62, 17.04, 271.26), (63.82, 16.44, 271.37), (63.01, 15.85, 271.47), (62.21, 15.25, 271.55), (61.41, 14.65, 271.61), (60.61, 14.05, 271.66), (59.81, 13.44, 271.71), (59.01, 12.84, 271.76), (58.21, 12.24, 271.81), (57.41, 11.63, 271.86), (56.61, 11.02, 271.9), (55.82, 10.41, 271.93), (55.02, 9.8, 271.96), (54.23, 9.18, 271.99), (53.44, 8.56, 272.02), (52.65, 7.94, 272.04), (51.87, 7.32, 272.06), (51.09, 6.69, 272.07), (50.31, 6.05, 272.08), (49.53, 5.42, 272.08), (48.76, 4.78, 272.08), (47.98, 4.14, 272.08), (47.21, 3.49, 272.07), (46.45, 2.85, 272.07), (45.68, 2.2, 272.06), (44.91, 1.55, 272.05), (44.15, 0.9, 272.03), (43.39, 0.25, 272.02), (42.63, -0.4, 272.01), (41.86, -1.06, 271.99), (41.11, -1.72, 271.99), (40.4, -2.41, 272.14), (39.7, -3.11, 272.33), (39.0, -3.81, 272.52), (38.31, -4.5, 272.71), (37.61, -5.2, 272.9), (36.91, -5.9, 273.1), (36.22, -6.59, 273.29), (35.52, -7.29, 273.49), (34.83, -7.98, 273.69), (34.13, -8.68, 273.9), (33.43, -9.37, 274.1), (32.73, -10.05, 274.31), (32.04, -10.74, 274.53), (31.34, -11.43, 274.75), (30.64, -12.12, 274.97), (29.94, -12.8, 275.19), (29.24, -13.49, 275.42), (28.55, -14.17, 275.65), (27.84, -14.85, 275.88), (27.14, -15.53, 276.11), (26.44, -16.21, 276.35), (25.74, -16.88, 276.59), (25.04, -17.56, 276.83), (24.33, -18.23, 277.07), (23.63, -18.91, 277.31), (22.93, -19.59, 277.55), (22.23, -20.26, 277.8), (21.53, -20.94, 278.04), (20.6, -21.28, 277.96), (19.64, -21.54, 277.8), (18.66, -21.34, 277.72), (17.69, -21.08, 277.65), (16.73, -20.82, 277.59), (15.76, -20.57, 277.52), (14.95, -20.06, 277.21), (13.95, -20.05, 277.2), (12.94, -20.05, 277.2), (11.94, -20.05, 277.2), (10.94, -20.05, 277.2), (9.93, -20.05, 277.2), (8.93, -20.05, 277.2)],
    "mic_pin12_splitleg": [(90.21, 39.04, 254.76), (90.21, 39.04, 255.77), (90.23, 39.17, 256.75), (89.87, 39.19, 257.66), (89.15, 38.93, 258.31), (88.35, 38.61, 258.83), (87.55, 38.28, 259.33), (86.73, 37.95, 259.82), (85.92, 37.62, 260.3), (85.09, 37.3, 260.79), (84.26, 36.97, 261.25), (83.43, 36.65, 261.71), (82.59, 36.34, 262.16), (81.74, 36.02, 262.6), (80.88, 35.72, 263.02), (80.01, 35.42, 263.44), (79.14, 35.12, 263.84), (78.27, 34.82, 264.23), (77.38, 34.53, 264.61), (76.49, 34.24, 264.97), (75.59, 33.96, 265.33), (74.68, 33.69, 265.67), (73.77, 33.42, 266.0), (72.86, 33.15, 266.31), (71.94, 32.89, 266.62), (71.01, 32.63, 266.92), (70.09, 32.37, 267.21), (69.16, 32.11, 267.5), (68.22, 31.85, 267.77), (67.29, 31.59, 268.04), (66.36, 31.34, 268.3), (65.42, 31.08, 268.57), (64.48, 30.82, 268.83), (63.55, 30.56, 269.08), (62.61, 30.29, 269.34), (61.68, 30.02, 269.6), (60.75, 29.75, 269.86), (59.82, 29.47, 270.12), (58.89, 29.19, 270.38), (57.96, 28.9, 270.65), (57.04, 28.61, 270.91), (56.11, 28.31, 271.18), (55.2, 28.01, 271.45), (54.28, 27.69, 271.72), (53.36, 27.38, 272.0), (52.45, 27.06, 272.27), (51.54, 26.74, 272.55), (50.63, 26.41, 272.83), (49.72, 26.08, 273.11), (48.82, 25.75, 273.38), (47.91, 25.41, 273.65), (47.0, 25.08, 273.92), (46.09, 24.73, 274.19), (45.18, 24.39, 274.45), (44.27, 24.04, 274.7), (43.36, 23.69, 274.95), (42.45, 23.34, 275.19), (41.54, 22.99, 275.42), (40.63, 22.63, 275.64), (39.71, 22.27, 275.86), (38.8, 21.91, 276.05), (37.88, 21.54, 276.24), (36.96, 21.17, 276.42), (36.04, 20.8, 276.58), (35.12, 20.42, 276.73), (34.21, 20.03, 276.87), (33.29, 19.64, 277.0), (32.37, 19.24, 277.11), (31.46, 18.84, 277.22), (30.54, 18.43, 277.3), (29.63, 18.01, 277.38), (28.72, 17.59, 277.44), (27.82, 17.15, 277.5), (26.92, 16.71, 277.54), (26.02, 16.26, 277.57), (25.12, 15.8, 277.6), (24.23, 15.34, 277.62), (23.34, 14.87, 277.63), (22.46, 14.39, 277.63), (21.57, 13.91, 277.63), (20.69, 13.43, 277.62), (19.81, 12.94, 277.61), (18.94, 12.45, 277.59), (18.06, 11.95, 277.57), (17.18, 11.46, 277.54), (16.31, 10.96, 277.52), (15.42, 10.5, 277.44), (14.49, 10.43, 277.2), (13.48, 10.43, 277.2), (12.48, 10.43, 277.2), (11.47, 10.43, 277.2)],
    "bus_jaw_roll": [(40.34, 21.75, 254.03), (40.4, 20.75, 254.07), (40.35, 19.76, 254.2), (40.1, 18.84, 254.51), (39.69, 18.06, 254.99), (39.16, 17.48, 255.61), (38.54, 17.15, 256.32), (37.87, 17.07, 257.05), (37.18, 17.17, 257.77), (36.46, 17.43, 258.42), (35.73, 17.88, 258.93), (34.99, 18.42, 259.33), (34.24, 19.06, 259.51), (33.5, 19.74, 259.56), (32.76, 20.41, 259.6), (32.02, 21.09, 259.63), (31.28, 21.77, 259.67), (30.54, 22.45, 259.71), (29.8, 23.12, 259.74), (29.06, 23.8, 259.78), (28.32, 24.48, 259.81), (27.58, 25.15, 259.84), (26.84, 25.83, 259.87), (26.09, 26.5, 259.89), (25.35, 27.17, 259.92), (24.61, 27.85, 259.94), (23.86, 28.52, 259.97), (23.12, 29.19, 259.99), (22.37, 29.87, 260.01), (21.63, 30.54, 260.03), (20.88, 31.21, 260.05), (20.14, 31.88, 260.07), (19.39, 32.55, 260.08), (18.56, 33.11, 260.01), (17.65, 33.44, 259.79), (16.72, 33.65, 259.48), (15.84, 33.69, 259.0), (14.98, 33.66, 258.48), (14.12, 33.65, 257.96), (13.27, 33.63, 257.44), (12.41, 33.61, 256.92), (11.55, 33.58, 256.39), (10.7, 33.56, 255.87), (9.84, 33.54, 255.35), (9.01, 33.45, 254.79), (8.25, 33.21, 254.18), (7.5, 32.92, 253.59), (6.7, 32.58, 253.09), (5.88, 32.22, 252.65), (5.05, 31.85, 252.2), (4.23, 31.49, 251.77), (3.37, 31.07, 251.46), (2.5, 30.6, 251.3), (1.62, 30.12, 251.17), (0.75, 29.65, 251.03), (-0.12, 29.17, 250.9), (-1.0, 28.7, 250.77), (-1.87, 28.22, 250.64), (-2.75, 27.75, 250.5), (-3.62, 27.28, 250.37), (-4.49, 26.8, 250.24), (-5.37, 26.32, 250.1), (-6.21, 25.8, 249.99), (-6.98, 25.16, 249.92), (-7.74, 24.5, 249.86), (-8.49, 23.84, 249.8), (-9.25, 23.18, 249.74), (-10.0, 22.52, 249.68), (-10.75, 21.86, 249.61), (-11.5, 21.2, 249.55), (-12.25, 20.53, 249.49), (-13.0, 19.87, 249.43), (-13.75, 19.2, 249.36), (-14.49, 18.53, 249.3), (-15.23, 17.85, 249.24), (-15.97, 17.18, 249.17), (-16.72, 16.51, 249.11), (-17.46, 15.83, 249.04), (-18.19, 15.15, 248.98), (-18.93, 14.48, 248.91), (-19.66, 13.8, 248.84), (-20.4, 13.12, 248.78), (-21.14, 12.44, 248.71), (-21.86, 11.75, 248.6), (-22.41, 11.01, 248.23), (-22.81, 10.28, 247.68), (-23.13, 9.54, 247.07), (-23.33, 8.81, 246.42), (-23.48, 8.08, 245.75), (-23.51, 7.36, 245.05), (-23.51, 6.63, 244.36), (-23.51, 5.91, 243.66), (-23.51, 5.19, 242.96), (-23.49, 4.47, 242.26), (-23.42, 3.76, 241.56), (-23.35, 3.04, 240.86), (-23.29, 2.32, 240.16), (-23.22, 1.61, 239.46), (-23.15, 0.89, 238.76), (-23.08, 0.18, 238.06), (-23.01, -0.54, 237.36), (-22.93, -1.25, 236.66), (-22.86, -1.97, 235.96), (-22.79, -2.68, 235.26), (-22.72, -3.4, 234.56), (-22.65, -4.11, 233.86), (-22.6, -4.79, 233.12), (-22.69, -5.27, 232.25), (-22.87, -5.85, 231.45), (-23.0, -6.56, 230.76), (-22.84, -7.33, 230.14), (-22.42, -8.05, 229.59), (-21.85, -8.7, 229.08), (-21.0, -9.04, 228.66), (-20.13, -9.34, 228.27), (-19.19, -9.45, 228.01), (-18.2, -9.35, 227.92), (-17.7, -9.3, 227.87)],
    "usb_c": [(-23.7, -16.0, 258.9), (-23.7, -17.0, 258.9), (-23.52, -17.9, 258.64), (-23.16, -18.68, 258.13), (-22.8, -19.47, 257.62), (-22.45, -20.25, 257.11), (-22.06, -21.0, 256.57), (-21.63, -21.68, 255.97), (-21.17, -22.31, 255.34), (-20.69, -22.91, 254.7), (-20.21, -23.49, 254.05), (-19.72, -24.08, 253.39), (-19.2, -24.66, 252.76), (-18.68, -25.24, 252.13), (-18.17, -25.81, 251.49), (-17.51, -26.34, 250.98), (-16.72, -26.81, 250.59), (-15.83, -27.2, 250.31), (-14.95, -27.58, 250.04), (-14.07, -27.97, 249.77), (-13.07, -28.1, 249.75), (-12.08, -28.23, 249.73), (-11.09, -28.36, 249.71), (-10.09, -28.47, 249.7), (-9.09, -28.57, 249.72), (-8.1, -28.68, 249.74), (-7.1, -28.78, 249.76), (-6.11, -28.89, 249.78), (-5.11, -29.01, 249.8), (-4.11, -29.12, 249.83), (-3.12, -29.25, 249.86), (-2.13, -29.37, 249.89), (-1.13, -29.51, 249.92), (-0.14, -29.64, 249.96), (0.85, -29.79, 250.0), (1.84, -29.94, 250.04), (2.83, -30.09, 250.09), (3.82, -30.25, 250.14), (4.8, -30.41, 250.19), (5.79, -30.58, 250.25), (6.78, -30.75, 250.31), (7.76, -30.93, 250.37), (8.74, -31.11, 250.44), (9.73, -31.29, 250.5), (10.71, -31.48, 250.57), (11.69, -31.68, 250.64), (12.67, -31.87, 250.72), (13.65, -32.07, 250.8), (14.63, -32.27, 250.88), (15.61, -32.47, 250.95), (16.59, -32.67, 251.03), (17.57, -32.86, 251.11), (18.56, -33.0, 251.16), (19.56, -33.05, 251.18), (20.56, -33.07, 251.2), (21.56, -33.1, 251.21), (22.56, -33.12, 251.23), (23.56, -33.15, 251.24), (24.57, -33.17, 251.25), (25.57, -33.19, 251.26), (26.57, -33.2, 251.27), (27.57, -33.22, 251.27), (28.57, -33.24, 251.28), (29.58, -33.25, 251.28), (30.58, -33.27, 251.28), (31.58, -33.28, 251.27), (32.58, -33.29, 251.26), (33.59, -33.29, 251.25), (34.59, -33.3, 251.24), (35.59, -33.3, 251.22), (36.59, -33.3, 251.2), (37.59, -33.3, 251.18), (38.59, -33.3, 251.15), (39.6, -33.29, 251.11), (40.6, -33.28, 251.08), (41.6, -33.26, 251.04), (42.6, -33.25, 250.99), (43.6, -33.23, 250.95), (44.6, -33.21, 250.89), (45.6, -33.19, 250.84), (46.6, -33.17, 250.78), (47.6, -33.15, 250.71), (48.6, -33.12, 250.65), (49.6, -33.09, 250.58), (50.6, -33.06, 250.5), (51.6, -33.02, 250.42), (52.57, -32.91, 250.24), (53.51, -32.72, 249.94), (54.44, -32.54, 249.64), (55.37, -32.33, 249.32), (56.28, -32.11, 248.96), (57.18, -31.87, 248.59), (58.08, -31.63, 248.23), (58.98, -31.38, 247.85), (59.87, -31.14, 247.47), (60.76, -30.89, 247.09), (61.65, -30.63, 246.69), (62.53, -30.37, 246.3), (63.41, -30.1, 245.9), (64.29, -29.83, 245.49), (65.16, -29.56, 245.08), (66.04, -29.28, 244.69), (66.94, -28.99, 244.35), (67.86, -28.72, 244.08), (68.82, -28.5, 243.89), (69.79, -28.3, 243.74), (70.76, -28.1, 243.58), (71.75, -28.0, 243.5), (72.75, -28.0, 243.5), (73.75, -28.0, 243.5)],
    "spk_pair": [(55.18, 3.06, 275.25), (55.19, 3.07, 274.23), (55.33, 3.31, 273.25), (55.11, 3.62, 272.33), (54.44, 3.95, 271.63), (53.7, 4.27, 271.01), (52.96, 4.59, 270.39), (52.22, 4.91, 269.77), (51.47, 5.23, 269.15), (50.73, 5.54, 268.53), (49.98, 5.86, 267.91), (49.21, 6.15, 267.31), (48.39, 6.36, 266.74), (47.52, 6.42, 266.22), (46.68, 6.12, 265.76), (46.06, 5.44, 265.33), (45.65, 4.59, 264.96), (45.28, 3.71, 264.6), (44.96, 2.82, 264.21), (44.7, 1.92, 263.82), (44.51, 0.94, 263.63), (44.42, -0.06, 263.75), (44.55, -0.9, 264.28), (44.89, -1.49, 265.03), (45.4, -1.87, 265.83), (46.2, -2.0, 266.3)],
}


def _apply_hr43c():
    """_define() 末尾（_apply_hr43 之后）调：改名 + 整根替换 _HR43C_ROUTES 里的线，Radxa 端 ends 按新针号 / 直插杜邦重写。"""
    if not _HR43C_ROUTES: return
    from . import electronics as E
    for w in WIRES:
        if w["id"] in HR43C_RENAME:
            w["id_was_until_2026_09_25_hr43c"] = w["id"]; w["id"] = HR43C_RENAME[w["id"]]
        if w["id"] not in _HR43C_ROUTES: continue
        w["pts"] = [tuple(p) for p in _HR43C_ROUTES[w["id"]]]; w["hr43c_rerouted"] = True
        if w["id"] != "bus_jaw_roll": w.pop("sides", None)
        else: w["sides"] = [np.array(JAW_PIN_ROW), np.array(JAW_PIN_ROW)] + [None] * (len(w["pts"]) - 4) + [np.array([0, 1.0, 0]), np.array([0, 1.0, 0])]
        if w["id"] in HR43C_LEG:
            n, k = HR43C_LEG[w["id"]]; s = [ss for (nn, kk, ss) in E.gpio_dup_legs() if nn == n and kk == k][0]
            od, Rc = E.gpio_dup_leg_cable(n); pe = E.gpio_dup_leg_end(n, s)
            rad = f"Radxa pin {n}（{PINMAP[n][0]}）直插杜邦壳顶出线即弯 Rc {Rc:g}（弯段在 zz_gpio_dupont）→ 接续点 ({pe[0]:.2f}, {pe[1]:.2f}, {pe[2]:.2f})，hr43c"
            e0, e1 = w.get("ends") or ("", "")
            w["ends"] = (rad, e1) if w["id"].startswith("amp_") else (e0, rad)
            if w["id"].startswith("amp_"):
                w["ends"] = (rad, f"功放直排针杜邦壳（板底 x 61.8..64.5，线弯区底 z {HR43_AMP_EXIT['z']}，y 随功放 −2.0，hr43c）")
            if w["id"] in ("mic_pin17_splitleg", "imu_pin17_splitleg"):
                w["bought_plan"] = BOUGHT["splitter_leg"][0]
                w["note"] = (w.get("note", "") + "｜hr43c：3.3 V 两家合用 17 号 —— 一分二（母对二公，件同 12 / 35 那根）单壳插 17，两腿接麦 VDD / IMU V+").lstrip("｜")
        elif w["id"] == "spk_pair":
            w["ends"] = ("喇叭背面焊片（凸台底 z 275.25）", f"功放端子块 −x 面（x {HR43_AMP_TERM['x_face']}，线口 z {HR43_AMP_TERM['z_wire']}，y 随功放 −2.0，hr43c）")
        elif w["id"] == "usb_c":
            w["ends"] = (f"Radxa USB-C2 90° 弯头（随板挪，线从 −y 面出，x {HR43C_USB_PLUG_RADXA['exit'][0]} y {HR43C_USB_PLUG_RADXA['exit'][1]} z {HR43C_USB_PLUG_RADXA['exit'][2]}）",
                          "转接板 Type-C 90° 弯头（B 边 −y，线从 −x 面进，x 73.75 y −28 z 243.5）")
        w["note"] = (w.get("note", "") + "｜hr43c：排针直插杜邦 / 功放 −y 2.0 / 板挪后重走（hr43c/routes43c_fb.json）").lstrip("｜")


# ── hr43c（复审 #2 B2）CSI 200 排线候选路径（HB06）：hr43c/c11_csi_ribbon.py 在 hr43c 体素（件 + 扫掠 + 现有线全占）上做排线状态格搜索
#   （1 mm 格；截面沿宽向两侧各 ≥ 5.65 + 0.3 可走；允许软弯 / 45° 折 / 扭转）找到的路（3 折 0 扭），c14 用平带盒复核对件 / 线 0 交、最近 H05 0.14。
#   走法：摄像头座顶 (88.02, ·, 273.94) 竖出 → R≈1 弯朝 −x、宽沿 y，z 275.5 从两道 H05-F06 压筋（|y| 11..13）中间过转接板 / H06 上方 → 缓降到 274.5 钻喇叭前挡 / 喇叭底下（顶离前挡下沿 ≈0.9）
#   → 折 1 (62.5, −0.5) 改朝 +y、宽沿 x（喇叭底与功放端子顶 272.3 之间）→ 折 2 (62.5, 12.5) 改朝 −x、宽沿 y → x 36 前缓升到 279.5 从杜邦线上方过排针区
#   → x 8.5 以后降到 277.5、x 1.5 竖直下到 261.5（排针后、SoC 元件包络 260.3 上方）→ 折 3 (−3.6, 12.5) 改朝 +y、宽沿 x，在板面上方 z 261.5 从两根 y 28 吊柱中间（x −12..5）过
#   → 绕板 +y 边（y 31.5）U 弯（R≈1.2..1.5）下到板底 → 沿 −y 插 Radxa CSI 座（板底 +y 端，槽中面 z 254.95，插入 4）。
#   ⚠ 候选（模型级）：① 折数 3 为奇数 → 两端触点朝向要到货按座子「上/下接触」核（不对就再加 1 折）；② 板边 U 弯 R≈1.2..1.5 ≥ CABLE min_bend 1.0，但 < 复审建议 2.5（assumed）；
#   ③ 线长 ≈ 各段 + 3 折 × 线宽 11.3 + 摄像头端插入 3 ≈ 185..190 → 200 mm 线余 ≈10..15（150 不够）；④ 离 H05 顶板最近 0.14。
HR43C_CSI_FOLDS = ((62.5, -0.5, 274.5, "z"), (62.5, 12.5, 274.5, "z"), (-3.6, 12.5, 261.5, "z"))   # 折痕中心 + 平带厚向
_HR43C_CSI_PTS = {
    "csi43c_a": [(88.02, -0.5, 273.96), (88.02, -0.5, 274.6), (87.3, -0.5, 275.3), (86.5, -0.5, 275.5), (80.0, -0.5, 275.5), (73.5, -0.5, 275.5),
                 (70.5, -0.5, 274.6), (67.0, -0.5, 274.5), (62.5, -0.5, 274.5)],
    "csi43c_b": [(62.5, -0.5, 274.5), (62.5, 5.0, 274.5), (62.5, 12.5, 274.5)],
    "csi43c_c": [(62.5, 12.5, 274.5), (50.0, 12.5, 274.5), (37.0, 12.5, 274.5), (34.5, 12.5, 275.5), (33.0, 12.5, 278.0), (31.0, 12.5, 279.5), (22.0, 12.5, 279.5),
                 (12.0, 12.5, 279.5), (10.0, 12.5, 278.5), (8.5, 12.5, 277.5), (3.0, 12.5, 277.5), (1.7, 12.5, 276.8), (1.5, 12.5, 275.5), (1.5, 12.5, 268.0),
                 (1.5, 12.5, 263.0), (1.2, 12.5, 262.0), (0.5, 12.5, 261.6), (-3.6, 12.5, 261.5)],
    "csi43c_d": [(-3.6, 12.5, 261.5), (-3.6, 20.0, 261.5), (-3.6, 30.5, 261.5), (-3.6, 32.3, 261.0), (-3.6, 33.3, 259.8), (-3.6, 33.5, 258.0), (-3.6, 33.3, 256.2),
                 (-3.6, 32.5, 255.2), (-3.6, 31.25, 254.95), (-3.6, 27.25, 254.95)],
}


def _hr43c_csi():
    """hr43c：CSI 排线 4 段（3 折）加进 WIRES（HB06）；折处双层平方块当 plug"""
    Yv, Xv = np.array([0, 1.0, 0]), np.array([1.0, 0, 0])
    hw = CABLE["fpc_csi"]["w"] / 2; t = CABLE["fpc_csi"]["t"]
    folds = [abox((x - hw, y - hw, z - t), (x + hw, y + hw, z + t)) for (x, y, z, _) in HR43C_CSI_FOLDS]
    side = {"csi43c_a": Yv, "csi43c_b": Xv, "csi43c_c": Yv, "csi43c_d": Xv}
    ends = {"csi43c_a": ("摄像头 FPC 翻盖座（倒装，背面顶边，座顶 z 273.94，线朝 +z 出）", "折 1 (62.5, −0.5, 274.5)"),
            "csi43c_b": ("折 1", "折 2 (62.5, 12.5, 274.5)"), "csi43c_c": ("折 2", "折 3 (−3.6, 12.5, 261.5)"),
            "csi43c_d": ("折 3", "Radxa CSI 22P 座（板底 +y 端，hr43c 板位 x −11.7..4.5 × y 25.5..31.25，沿 −y 插入 4，槽中面 z 254.95）")}
    for k, pts in _HR43C_CSI_PTS.items():
        w = dict(id=k, harness="HB06", cable="fpc_csi", pts=list(pts), sides=[side[k]] * len(pts), group="csi", creases=0, ends=ends[k], hr43c_rerouted=True,
                 bought=(200.0 if k == "csi43c_a" else 0.0), bought_src=("CSI 200 mm 22→15 同面线（用户未答买不买；150 不够，见 _HR43_CSI 注）" if k == "csi43c_a" else "与 csi43c_a 同一根"),
                 note="hr43c B2 候选路径（hr43c/c11_csi_ribbon.json / c14_csi_verify.json）；见 HR43C_CSI_FOLDS 上方注释")
        if k == "csi43c_a":
            w.update(wide_head=True, fixed_mm=3 * CABLE["fpc_csi"]["w"] + 3.0, creases=3, crease_at="3 处 45° 折：" + "；".join(f"({x}, {y}, {z})" for (x, y, z, _) in HR43C_CSI_FOLDS),
                     plugs=folds)
        WIRES.append(w)


_HR43_CSI_was_until_2026_09_25_hr43d = _hr43c_csi
_HR43_CSI = _hr43c_csi


# ━━━━━━━━━━━━━━━━━━━━ hr43d（2026-09-25）转接板平放 H08 托板（B′）+ 功放挪后脑 H09（E′）→ 头内线重走 ━━━━━━━━━━━━━━━━━━━━
#   依据 hr43_work/转接板安放研究.md §5.2–§5.4、agent_brief_hr43d.md 第 4 条；航点 = hr43_work/hr43d/d_route.py（体素 0.5 → 1 mm Dijkstra，净空 ≥ r + 0.6，逐根占用）→ d_fix_bends。
#   · 转接板 4 口：见 ADP / ADP_PORT（平放，口朝 ±y 侧出）；Type-C 弯头 HR43D_USB_PLUG_ADP；S1 转接块挪到 HR43D_S1_JUNCTION。
#   · 功放（electronics.AMP_S_C/R，后脑 +y）：5 根杜邦线 Radxa 端改「出壳即弯朝 −x」（electronics.GPIO_DUP_LEG_DIR），功放端从壳尾线弯区（x −32.1..−28.1、y 20.3..23）出；
#     针序（局部 x → 世界 z = 235.8 − 局部 x）：LRC 243.42 / BCLK 240.88 / DIN 238.34 / GND 230.72 / VIN 228.18（Adafruit 丝印顺序，assumed 同 hr43）。
#   · 喇叭线：喇叭背面焊片 → 功放端子（端子块 +y 面接线口朝 +y，y 38.35；线长自带 ≈100 **未实量，assumed**）。
#   · CSI（HB06）：hr43c 候选路径在板上方那段（x 62..88）整体抬 0.35（274.5 → 274.85），折 1 / 折 2 挪到 x 64.5（让开喇叭背面焊片凸台 x ≤ 58.18）→ 离板 ≥1.05。
HR43D_AMP_PIN_Z = {nm: round(float(235.8 - v), 2) for nm, v in {"LRC": -7.62, "BCLK": -5.08, "DIN": -2.54, "GND": 5.08, "VIN": 7.62}.items()}   # 世界 z（局部 x 取反 + 235.8）
HR43D_AMP_TAIL = dict(x=-28.1, y=21.65, bend_x=-32.1, src="electronics.AMP_S_BX dupont_shells 局部 z −17.3 → 世界 x −28.1；壳 y 带 20.3..23（中 21.65）；线弯区到 −32.1")
HR43D_AMP_TERM = dict(x=(-10.0, -1.5), y_face=38.35, z=(232.15, 239.45), src="electronics.AMP_S_BX term → 世界；KF128 接线口朝局部 +y = 世界 +y（assumed，到货看端子朝向）")
HR43D_S1_JUNCTION = None          # hr43d：S1 转接块留 hr43c 原位（(77.7, 35.2, 242.4) 沿 y，d_route v4 起；v1..v3 挪块试过找不到出路）
_HR43D_ROUTES = {   # hr43_work/hr43d/routes43d_fb.json（hr43d/d_route.py → d_fix_bends.py，1 mm 均匀航点；Radxa 端首/末点 = electronics.gpio_dup_leg_end）
    "amp_LRC_pin35_splitleg": [(4.93, -20.05, 275.8), (3.93, -20.06, 275.8), (3.2, -20.15, 275.36), (2.51, -19.5, 275.03), (1.83, -18.82, 274.72), (1.16, -18.15, 274.39), (0.49, -17.47, 274.07), (-0.18, -16.8, 273.75), (-0.84, -16.12, 273.42), (-1.51, -15.45, 273.08), (-2.17, -14.78, 272.74), (-2.83, -14.11, 272.39), (-3.49, -13.44, 272.04), (-4.15, -12.77, 271.68), (-4.8, -12.1, 271.32), (-5.45, -11.43, 270.94), (-6.1, -10.77, 270.56), (-6.74, -10.1, 270.17), (-7.38, -9.44, 269.77), (-8.02, -8.78, 269.36), (-8.65, -8.12, 268.94), (-9.28, -7.47, 268.52), (-9.9, -6.81, 268.08), (-10.52, -6.16, 267.64), (-11.14, -5.5, 267.18), (-11.75, -4.86, 266.72), (-12.35, -4.21, 266.25), (-12.95, -3.56, 265.77), (-13.55, -2.92, 265.29), (-14.14, -2.28, 264.79), (-14.73, -1.64, 264.29), (-15.32, -1.0, 263.78), (-15.89, -0.36, 263.26), (-16.47, 0.28, 262.74), (-17.04, 0.91, 262.21), (-17.6, 1.54, 261.67), (-18.16, 2.17, 261.13), (-18.72, 2.8, 260.58), (-19.27, 3.43, 260.03), (-19.82, 4.06, 259.47), (-20.37, 4.69, 258.91), (-20.91, 5.32, 258.35), (-21.45, 5.95, 257.78), (-21.98, 6.58, 257.21), (-22.51, 7.21, 256.64), (-23.04, 7.84, 256.06), (-23.56, 8.47, 255.48), (-24.09, 9.1, 254.9), (-24.6, 9.73, 254.31), (-25.12, 10.36, 253.73), (-25.63, 11.0, 253.14), (-26.14, 11.63, 252.55), (-26.64, 12.27, 251.96), (-27.15, 12.91, 251.37), (-27.65, 13.54, 250.78), (-28.15, 14.18, 250.19), (-28.64, 14.82, 249.6), (-29.14, 15.46, 249.0), (-29.63, 16.1, 248.41), (-30.12, 16.75, 247.81), (-30.61, 17.39, 247.22), (-31.1, 18.03, 246.62), (-31.59, 18.68, 246.03), (-32.01, 19.35, 245.42), (-32.36, 20.06, 244.8), (-32.72, 20.76, 244.17), (-33.07, 21.46, 243.55), (-32.12, 21.65, 243.42)],
    "amp_GND_pin25": [(4.93, -7.35, 275.8), (3.92, -7.35, 275.79), (3.12, -7.28, 275.3), (2.32, -6.83, 274.89), (1.52, -6.38, 274.47), (0.72, -5.94, 274.05), (-0.07, -5.49, 273.63), (-0.87, -5.04, 273.21), (-1.67, -4.58, 272.8), (-2.46, -4.13, 272.37), (-3.25, -3.68, 271.95), (-4.04, -3.22, 271.53), (-4.83, -2.77, 271.1), (-5.62, -2.31, 270.67), (-6.4, -1.86, 270.23), (-7.18, -1.4, 269.8), (-7.96, -0.94, 269.35), (-8.73, -0.48, 268.91), (-9.5, -0.02, 268.45), (-10.27, 0.44, 267.99), (-11.03, 0.91, 267.53), (-11.79, 1.38, 267.06), (-12.54, 1.84, 266.58), (-13.28, 2.31, 266.09), (-14.02, 2.78, 265.6), (-14.75, 3.25, 265.09), (-15.48, 3.71, 264.57), (-16.2, 4.18, 264.05), (-16.92, 4.64, 263.51), (-17.62, 5.1, 262.97), (-18.32, 5.56, 262.41), (-19.02, 6.02, 261.84), (-19.7, 6.48, 261.26), (-20.39, 6.93, 260.68), (-21.05, 7.38, 260.08), (-21.72, 7.83, 259.47), (-22.38, 8.27, 258.86), (-23.04, 8.71, 258.24), (-23.7, 9.15, 257.62), (-24.35, 9.59, 256.99), (-24.99, 10.03, 256.35), (-25.62, 10.45, 255.69), (-26.24, 10.87, 255.02), (-26.87, 11.29, 254.35), (-27.49, 11.7, 253.68), (-28.1, 12.11, 252.99), (-28.7, 12.51, 252.29), (-29.23, 12.86, 251.51), (-29.69, 13.15, 250.67), (-30.13, 13.43, 249.81), (-30.54, 13.69, 248.93), (-30.94, 13.96, 248.04), (-31.34, 14.22, 247.16), (-31.74, 14.49, 246.28), (-32.15, 14.75, 245.39), (-32.55, 15.01, 244.51), (-32.95, 15.28, 243.63), (-33.35, 15.54, 242.74), (-33.54, 15.89, 241.82), (-33.61, 16.27, 240.89), (-33.63, 16.69, 239.98), (-33.62, 17.15, 239.08), (-33.61, 17.6, 238.18), (-33.59, 18.05, 237.29), (-33.58, 18.51, 236.39), (-33.57, 18.96, 235.49), (-33.56, 19.42, 234.59), (-33.55, 19.87, 233.7), (-33.44, 20.35, 232.82), (-33.3, 20.83, 231.94), (-33.17, 21.31, 231.07), (-32.62, 21.65, 230.71), (-32.12, 21.65, 230.72)],
    "amp_VIN_pin2": [(7.47, 23.13, 275.8), (6.47, 23.13, 275.8), (5.65, 22.6, 275.55), (4.75, 22.52, 275.13), (3.86, 22.55, 274.69), (2.96, 22.57, 274.24), (2.07, 22.6, 273.78), (1.18, 22.63, 273.33), (0.29, 22.66, 272.86), (-0.6, 22.69, 272.39), (-1.47, 22.73, 271.91), (-2.35, 22.76, 271.43), (-3.22, 22.8, 270.93), (-4.09, 22.85, 270.43), (-4.95, 22.9, 269.92), (-5.8, 22.95, 269.4), (-6.65, 23.01, 268.87), (-7.48, 23.07, 268.32), (-8.32, 23.14, 267.77), (-9.14, 23.2, 267.21), (-9.97, 23.27, 266.64), (-10.78, 23.35, 266.05), (-11.59, 23.43, 265.47), (-12.39, 23.51, 264.87), (-13.18, 23.59, 264.27), (-13.98, 23.67, 263.66), (-14.77, 23.76, 263.05), (-15.55, 23.85, 262.44), (-16.34, 23.94, 261.83), (-17.09, 24.03, 261.18), (-17.68, 24.18, 260.37), (-18.25, 24.33, 259.57), (-18.83, 24.47, 258.76), (-19.4, 24.61, 257.95), (-19.83, 24.67, 257.05), (-20.22, 24.7, 256.12), (-20.6, 24.74, 255.2), (-20.99, 24.76, 254.28), (-21.39, 24.79, 253.36), (-21.79, 24.82, 252.44), (-22.19, 24.84, 251.52), (-22.6, 24.85, 250.61), (-23.01, 24.87, 249.69), (-23.43, 24.87, 248.78), (-23.85, 24.87, 247.87), (-24.27, 24.86, 246.97), (-24.7, 24.86, 246.06), (-25.14, 24.84, 245.16), (-25.58, 24.82, 244.26), (-26.03, 24.8, 243.37), (-26.48, 24.76, 242.47), (-26.94, 24.72, 241.58), (-27.4, 24.68, 240.69), (-27.87, 24.63, 239.81), (-28.34, 24.57, 238.93), (-28.81, 24.52, 238.04), (-29.28, 24.46, 237.16), (-29.76, 24.39, 236.29), (-30.24, 24.32, 235.41), (-30.73, 24.25, 234.54), (-31.21, 24.18, 233.66), (-31.7, 24.11, 232.79), (-32.18, 24.02, 231.91), (-32.59, 23.71, 231.06), (-32.82, 23.16, 230.27), (-32.93, 22.52, 229.51), (-33.03, 21.89, 228.74), (-32.62, 21.65, 228.19), (-32.12, 21.65, 228.18)],
    "spk_pair": [(52.18, 3.06, 276.0), (51.18, 3.06, 276.0), (50.44, 3.25, 275.34), (49.71, 3.45, 274.69), (49.01, 3.94, 274.18), (48.34, 4.55, 273.75), (47.67, 5.16, 273.31), (47.0, 5.76, 272.88), (46.32, 6.37, 272.46), (45.64, 7.02, 272.12), (44.95, 7.67, 271.78), (44.26, 8.32, 271.45), (43.57, 8.97, 271.12), (42.88, 9.62, 270.8), (42.19, 10.27, 270.49), (41.49, 10.93, 270.18), (40.8, 11.59, 269.89), (40.11, 12.25, 269.59), (39.41, 12.92, 269.31), (38.71, 13.58, 269.04), (38.01, 14.25, 268.78), (37.31, 14.93, 268.53), (36.61, 15.6, 268.28), (35.91, 16.28, 268.05), (35.21, 16.96, 267.81), (34.51, 17.64, 267.6), (33.81, 18.33, 267.38), (33.11, 19.01, 267.18), (32.41, 19.7, 266.98), (31.71, 20.39, 266.78), (31.01, 21.09, 266.59), (30.31, 21.78, 266.4), (29.61, 22.47, 266.22), (28.91, 23.17, 266.04), (28.21, 23.86, 265.87), (27.5, 24.56, 265.7), (26.8, 25.26, 265.53), (26.11, 25.95, 265.35), (25.41, 26.65, 265.18), (24.71, 27.35, 265.01), (24.01, 28.05, 264.84), (23.31, 28.75, 264.67), (22.61, 29.45, 264.5), (21.91, 30.15, 264.33), (21.21, 30.85, 264.17), (20.51, 31.55, 264.0), (19.81, 32.25, 263.83), (19.12, 32.94, 263.66), (18.45, 33.62, 263.35), (17.83, 34.26, 262.89), (17.22, 34.9, 262.41), (16.6, 35.53, 261.94), (15.99, 36.13, 261.42), (15.48, 36.49, 260.64), (14.98, 36.82, 259.83), (14.48, 37.15, 259.03), (13.98, 37.47, 258.22), (13.45, 37.71, 257.41), (12.91, 37.93, 256.59), (12.31, 38.1, 255.8), (11.68, 38.24, 255.03), (11.06, 38.38, 254.26), (10.43, 38.52, 253.49), (9.8, 38.66, 252.73), (9.17, 38.79, 251.96), (8.53, 38.93, 251.19), (7.89, 39.06, 250.43), (7.25, 39.2, 249.67), (6.61, 39.33, 248.91), (5.96, 39.46, 248.16), (5.32, 39.59, 247.4), (4.66, 39.71, 246.65), (4.01, 39.84, 245.9), (3.35, 39.96, 245.16), (2.69, 40.08, 244.41), (2.02, 40.2, 243.67), (1.36, 40.32, 242.93), (0.69, 40.44, 242.19), (0.02, 40.56, 241.46), (-0.65, 40.68, 240.72), (-1.33, 40.79, 239.99), (-2.0, 40.91, 239.26), (-2.68, 41.02, 238.53), (-3.39, 41.12, 237.83), (-4.14, 41.2, 237.16), (-4.88, 41.28, 236.5), (-5.63, 41.36, 235.83), (-5.75, 40.38, 235.8), (-5.75, 39.37, 235.8), (-5.75, 38.37, 235.8)],
    "usb_c": [(-23.7, -16.0, 258.9), (-23.7, -17.0, 258.9), (-23.7, -18.0, 258.9), (-23.69, -19.01, 258.9), (-23.01, -19.12, 259.62), (-22.32, -19.23, 260.34), (-21.64, -19.34, 261.07), (-20.95, -19.45, 261.79), (-20.04, -19.37, 262.2), (-19.1, -19.26, 262.5), (-18.12, -19.14, 262.69), (-17.14, -19.02, 262.88), (-16.17, -18.89, 263.08), (-15.2, -18.77, 263.28), (-14.22, -18.63, 263.47), (-13.25, -18.49, 263.67), (-12.28, -18.35, 263.87), (-11.31, -18.2, 264.08), (-10.34, -18.05, 264.28), (-9.37, -17.89, 264.49), (-8.41, -17.73, 264.7), (-7.44, -17.57, 264.9), (-6.47, -17.4, 265.11), (-5.51, -17.23, 265.32), (-4.55, -17.06, 265.54), (-3.58, -16.89, 265.76), (-2.64, -16.65, 266.0), (-1.7, -16.41, 266.25), (-0.76, -16.16, 266.5), (0.17, -15.91, 266.75), (1.11, -15.66, 267.0), (2.05, -15.42, 267.25), (2.99, -15.17, 267.5), (3.92, -14.92, 267.75), (4.87, -14.66, 267.97), (5.83, -14.42, 268.13), (6.8, -14.21, 268.25), (7.79, -14.08, 268.32), (8.79, -14.02, 268.34), (9.79, -14.0, 268.35), (10.79, -14.0, 268.36), (11.8, -14.01, 268.37), (12.8, -14.06, 268.38), (13.8, -14.12, 268.39), (14.79, -14.19, 268.4), (15.79, -14.27, 268.41), (16.79, -14.35, 268.42), (17.79, -14.44, 268.43), (18.79, -14.53, 268.44), (19.78, -14.65, 268.45), (20.78, -14.77, 268.46), (21.77, -14.9, 268.48), (22.76, -15.05, 268.49), (23.75, -15.22, 268.5), (24.73, -15.4, 268.51), (25.72, -15.6, 268.53), (26.69, -15.82, 268.54), (27.67, -16.05, 268.56), (28.63, -16.31, 268.58), (29.59, -16.6, 268.6), (30.55, -16.89, 268.63), (31.5, -17.22, 268.65), (32.44, -17.55, 268.68), (33.37, -17.92, 268.7), (34.3, -18.29, 268.73), (35.22, -18.7, 268.76), (36.13, -19.11, 268.8), (37.03, -19.55, 268.84), (37.93, -20.0, 268.87), (38.81, -20.47, 268.92), (39.69, -20.95, 268.96), (40.56, -21.44, 269.01), (41.42, -21.95, 269.06), (42.28, -22.47, 269.11), (43.13, -23.0, 269.16), (43.97, -23.53, 269.21), (44.82, -24.07, 269.26), (45.67, -24.59, 269.31), (46.59, -25.0, 269.32), (47.5, -25.41, 269.34), (48.41, -25.82, 269.35), (49.37, -26.0, 269.35), (50.37, -26.0, 269.35), (51.38, -26.0, 269.35), (52.38, -26.0, 269.35), (53.38, -26.0, 269.35), (53.88, -26.0, 269.35)],
    "xt30_12v": [(39.9, -19.2, 214.0), (40.11, -19.41, 214.96), (40.32, -19.62, 215.91), (40.53, -19.83, 216.87), (40.74, -20.04, 217.82), (40.95, -20.25, 218.78), (41.13, -20.43, 219.75), (41.29, -20.59, 220.72), (41.55, -20.85, 221.64), (42.0, -21.3, 222.41), (42.46, -21.77, 223.17), (43.02, -22.34, 223.77), (43.59, -22.93, 224.35), (43.95, -23.73, 224.77), (44.44, -24.51, 225.16), (45.03, -25.19, 225.58), (45.63, -25.87, 226.01), (46.23, -26.54, 226.44), (46.84, -27.21, 226.87), (47.44, -27.89, 227.3), (48.04, -28.56, 227.73), (48.64, -29.23, 228.16), (49.25, -29.91, 228.59), (49.82, -30.55, 229.09), (50.4, -31.2, 229.6), (50.98, -31.84, 230.1), (51.46, -32.38, 230.79), (51.73, -32.6, 231.73), (52.01, -32.81, 232.67), (52.28, -33.02, 233.61), (52.58, -33.1, 234.56), (52.9, -33.14, 235.51), (53.23, -33.18, 236.45), (53.56, -33.22, 237.39), (53.9, -33.26, 238.33), (54.25, -33.3, 239.27), (54.62, -33.34, 240.2), (54.99, -33.39, 241.13), (55.38, -33.43, 242.05), (55.78, -33.48, 242.97), (56.19, -33.53, 243.88), (56.62, -33.58, 244.78), (57.06, -33.63, 245.68), (57.52, -33.69, 246.56), (58.0, -33.75, 247.44), (58.48, -33.81, 248.31), (58.99, -33.87, 249.18), (59.5, -33.94, 250.03), (60.03, -34.02, 250.88), (60.57, -34.09, 251.72), (61.13, -34.17, 252.55), (61.7, -34.25, 253.37), (62.27, -34.34, 254.18), (62.86, -34.43, 254.98), (63.46, -34.52, 255.78), (64.07, -34.62, 256.57), (64.69, -34.72, 257.35), (65.31, -34.83, 258.12), (65.95, -34.93, 258.89), (66.58, -35.04, 259.65), (67.23, -35.15, 260.41), (67.87, -35.27, 261.17), (68.53, -35.39, 261.92), (69.19, -35.51, 262.66), (69.85, -35.63, 263.4), (70.51, -35.75, 264.14), (71.18, -35.85, 264.88), (71.84, -35.85, 265.62), (72.48, -35.64, 266.37), (73.12, -35.44, 267.11), (73.76, -35.23, 267.85), (74.4, -35.02, 268.59), (74.4, -34.02, 268.6), (74.4, -33.02, 268.6)],
    "bus_s1_leg1": [(56.2, 26.85, 268.35), (56.2, 27.85, 268.35), (56.21, 28.85, 268.36), (56.63, 29.75, 268.46), (57.08, 30.64, 268.35), (57.55, 31.5, 268.2), (58.15, 32.22, 267.85), (58.76, 32.93, 267.5), (59.37, 33.64, 267.14), (59.98, 34.35, 266.78), (60.59, 35.06, 266.42), (61.21, 35.76, 266.06), (61.85, 36.39, 265.64), (62.52, 36.97, 265.16), (63.23, 37.13, 264.49), (63.93, 37.11, 263.77), (64.63, 37.09, 263.06), (65.33, 37.07, 262.35), (66.03, 37.04, 261.63), (66.72, 37.0, 260.91), (67.31, 36.65, 260.19), (67.76, 36.07, 259.52), (68.2, 35.48, 258.84), (68.65, 34.89, 258.17), (69.1, 34.3, 257.49), (69.54, 33.71, 256.82), (69.99, 33.13, 256.14), (70.44, 32.55, 255.46), (70.89, 31.97, 254.78), (71.34, 31.39, 254.1), (71.79, 30.82, 253.42), (72.24, 30.25, 252.73), (72.69, 29.68, 252.04), (73.14, 29.12, 251.35), (73.6, 28.56, 250.65), (74.05, 28.0, 249.96), (74.5, 27.44, 249.26), (74.96, 26.89, 248.56), (75.41, 26.34, 247.86), (75.87, 25.79, 247.16), (76.32, 25.25, 246.45), (76.78, 24.71, 245.74), (77.19, 24.21, 244.98), (77.33, 24.1, 244.0), (77.47, 23.99, 243.02), (77.6, 22.85, 241.4), (77.73, 24.83, 241.56), (77.79, 26.81, 241.79), (77.8, 28.3, 241.94)],
    "ubec_in": [(87.37, -31.29, 240.46), (87.37, -31.16, 239.47), (87.37, -30.79, 238.54), (87.37, -30.21, 237.72), (87.37, -29.45, 237.08), (87.37, -28.56, 236.63), (87.37, -27.58, 236.41), (87.37, -26.59, 236.36), (87.37, -25.59, 236.36), (87.37, -24.59, 236.36), (87.37, -23.59, 236.36), (87.36, -23.0, 236.9), (87.24, -22.94, 237.89), (87.12, -22.87, 238.88), (87.0, -22.81, 239.87), (86.78, -22.55, 240.81), (86.49, -22.12, 241.66), (86.13, -21.56, 242.41), (85.75, -20.93, 243.08), (85.33, -20.24, 243.67), (84.91, -19.54, 244.25), (84.49, -18.85, 244.83), (84.07, -18.14, 245.41), (83.69, -17.38, 245.93), (83.31, -16.62, 246.45), (83.03, -15.76, 246.83), (82.92, -14.77, 246.95), (82.81, -13.78, 247.06), (82.7, -12.79, 247.18), (82.6, -11.81, 247.3), (82.49, -10.82, 247.41), (82.38, -9.83, 247.53), (82.27, -8.84, 247.65), (82.16, -7.86, 247.77), (82.04, -6.87, 247.89), (81.92, -5.89, 248.02), (81.8, -4.9, 248.15), (81.67, -3.92, 248.29), (81.55, -2.94, 248.42), (81.42, -1.96, 248.56), (81.28, -0.98, 248.71), (81.14, 0.0, 248.86), (80.99, 0.98, 249.02), (80.84, 1.95, 249.19), (80.67, 2.92, 249.37), (80.5, 3.89, 249.56), (80.32, 4.85, 249.76), (80.13, 5.81, 249.97), (79.94, 6.76, 250.2), (79.73, 7.72, 250.44), (79.52, 8.66, 250.69), (79.3, 9.6, 250.95), (79.07, 10.53, 251.23), (78.84, 11.46, 251.53), (78.6, 12.38, 251.83), (78.35, 13.29, 252.16), (78.1, 14.2, 252.49), (77.84, 15.1, 252.85), (77.57, 15.99, 253.22), (77.31, 16.87, 253.6), (77.05, 17.75, 254.0), (76.78, 18.62, 254.42), (76.51, 19.49, 254.85), (76.25, 20.34, 255.29), (75.98, 21.19, 255.74), (75.71, 22.04, 256.21), (75.46, 22.88, 256.69), (75.2, 23.71, 257.18), (74.95, 24.54, 257.68), (74.7, 25.36, 258.19), (74.45, 26.18, 258.71), (74.21, 26.99, 259.23), (73.97, 27.8, 259.77), (73.74, 28.61, 260.31), (73.5, 29.41, 260.86), (73.28, 30.22, 261.42), (73.05, 31.02, 261.97), (72.84, 31.81, 262.53), (72.62, 32.61, 263.1), (72.42, 33.33, 263.75), (72.23, 33.98, 264.49), (72.04, 34.63, 265.23), (71.9, 35.0, 266.09), (71.82, 35.01, 267.09), (71.74, 35.02, 268.09), (71.7, 34.52, 268.6), (71.7, 33.52, 268.6), (71.7, 33.02, 268.6)],
    "ubec_out_5v": [(87.4, -31.8, 266.48), (87.4, -31.8, 267.48), (87.31, -31.8, 268.48), (87.11, -31.8, 269.46), (86.64, -31.8, 270.35), (86.07, -31.8, 271.17), (85.24, -31.8, 271.74), (84.36, -31.8, 272.21), (83.38, -31.8, 272.41), (82.39, -31.69, 272.5), (81.42, -31.45, 272.52), (80.55, -30.98, 272.7), (79.69, -30.5, 272.88), (78.82, -30.02, 273.06), (77.95, -29.55, 273.24), (77.09, -29.08, 273.41), (76.22, -28.6, 273.58), (75.35, -28.13, 273.75), (74.47, -27.66, 273.9), (73.6, -27.19, 274.05), (72.73, -26.71, 274.19), (71.85, -26.24, 274.33), (70.98, -25.77, 274.45), (70.1, -25.29, 274.57), (69.23, -24.81, 274.68), (68.35, -24.32, 274.78), (67.48, -23.84, 274.87), (66.61, -23.35, 274.94), (65.74, -22.85, 275.01), (64.87, -22.35, 275.06), (64.0, -21.85, 275.11), (63.14, -21.33, 275.14), (62.28, -20.82, 275.17), (61.42, -20.29, 275.18), (60.57, -19.76, 275.19), (59.72, -19.23, 275.18), (58.88, -18.69, 275.17), (58.03, -18.14, 275.15), (57.2, -17.59, 275.12), (56.36, -17.03, 275.08), (55.53, -16.47, 275.04), (54.7, -15.9, 274.99), (53.88, -15.34, 274.94), (53.06, -14.76, 274.89), (52.23, -14.19, 274.84), (51.41, -13.62, 274.78), (50.59, -13.04, 274.72), (49.8, -12.43, 274.63), (49.02, -11.81, 274.53), (48.29, -11.12, 274.46), (47.62, -10.38, 274.41), (46.95, -9.63, 274.36), (46.27, -8.89, 274.31), (45.6, -8.15, 274.26), (44.93, -7.4, 274.22), (44.27, -6.65, 274.17), (43.61, -5.89, 274.13), (42.95, -5.14, 274.09), (42.29, -4.38, 274.05), (41.64, -3.62, 274.02), (40.99, -2.86, 273.99), (40.34, -2.09, 273.96), (39.7, -1.32, 273.93), (39.06, -0.55, 273.91), (38.42, 0.23, 273.89), (37.78, 1.0, 273.87), (37.15, 1.78, 273.87), (36.52, 2.56, 273.86), (35.89, 3.35, 273.86), (35.27, 4.13, 273.87), (34.64, 4.92, 273.87), (34.02, 5.7, 273.88), (33.39, 6.49, 273.9), (32.77, 7.28, 273.92), (32.15, 8.06, 273.94), (31.52, 8.85, 273.98), (30.9, 9.63, 274.01), (30.27, 10.41, 274.05), (29.64, 11.19, 274.1), (29.01, 11.98, 274.15), (28.37, 12.75, 274.2), (27.73, 13.52, 274.26), (27.09, 14.29, 274.32), (26.45, 15.06, 274.39), (25.8, 15.82, 274.46), (25.15, 16.58, 274.53), (24.5, 17.34, 274.6), (23.84, 18.1, 274.68), (23.18, 18.85, 274.76), (22.53, 19.61, 274.84), (21.87, 20.36, 274.92), (21.2, 21.11, 275.0), (20.42, 21.72, 275.11), (19.63, 22.33, 275.21), (18.84, 22.94, 275.31), (17.84, 23.02, 275.4), (16.92, 22.76, 275.45), (16.1, 22.18, 275.48), (15.47, 21.44, 275.53), (14.98, 20.61, 275.79), (13.98, 20.59, 275.8), (12.98, 20.59, 275.8), (11.97, 20.59, 275.8), (11.47, 20.59, 275.8)],
    "ubec_out_gnd": [(87.4, -34.4, 266.48), (87.4, -34.4, 267.49), (87.3, -34.4, 268.48), (87.11, -34.4, 269.47), (86.64, -34.4, 270.34), (86.08, -34.4, 271.17), (85.25, -34.4, 271.74), (84.38, -34.4, 272.21), (83.39, -34.4, 272.4), (82.4, -34.44, 272.5), (81.4, -34.49, 272.51), (80.4, -34.43, 272.62), (79.4, -34.37, 272.73), (78.4, -34.31, 272.83), (77.41, -34.24, 272.93), (76.41, -34.17, 273.03), (75.41, -34.1, 273.13), (74.41, -34.03, 273.23), (73.41, -33.96, 273.32), (72.42, -33.89, 273.41), (71.42, -33.81, 273.49), (70.42, -33.74, 273.58), (69.42, -33.66, 273.66), (68.42, -33.57, 273.73), (67.42, -33.48, 273.79), (66.42, -33.39, 273.86), (65.43, -33.3, 273.92), (64.43, -33.2, 273.98), (63.43, -33.09, 274.04), (62.43, -32.99, 274.09), (61.43, -32.88, 274.13), (60.43, -32.78, 274.17), (59.44, -32.66, 274.21), (58.44, -32.55, 274.25), (57.44, -32.43, 274.28), (56.44, -32.3, 274.31), (55.45, -32.18, 274.34), (54.45, -32.04, 274.37), (53.46, -31.9, 274.4), (52.46, -31.75, 274.42), (51.47, -31.61, 274.45), (50.48, -31.44, 274.47), (49.49, -31.28, 274.49), (48.5, -31.1, 274.52), (47.51, -30.91, 274.55), (46.52, -30.72, 274.58), (45.54, -30.52, 274.61), (44.56, -30.31, 274.63), (43.58, -30.08, 274.67), (42.6, -29.84, 274.71), (41.63, -29.59, 274.74), (40.66, -29.32, 274.78), (39.7, -29.05, 274.82), (38.74, -28.75, 274.87), (37.78, -28.46, 274.92), (36.83, -28.13, 274.98), (35.88, -27.81, 275.04), (34.94, -27.46, 275.11), (34.0, -27.1, 275.18), (33.07, -26.73, 275.25), (32.15, -26.34, 275.33), (31.23, -25.96, 275.42), (30.31, -25.54, 275.5), (29.4, -25.13, 275.59), (28.5, -24.69, 275.69), (27.6, -24.25, 275.79), (26.71, -23.8, 275.89), (25.82, -23.34, 276.01), (24.94, -22.88, 276.12), (24.06, -22.4, 276.23), (23.18, -21.93, 276.35), (22.31, -21.44, 276.48), (21.44, -20.96, 276.6), (20.58, -20.46, 276.73), (19.71, -19.96, 276.86), (18.85, -19.46, 276.99), (17.99, -18.96, 277.12), (17.13, -18.46, 277.25), (16.27, -17.95, 277.38), (15.41, -17.5, 277.55), (14.48, -17.51, 277.8), (13.47, -17.51, 277.8)],
    "mic_pin38": [(85.13, 31.58, 254.76), (85.13, 31.58, 255.76), (85.13, 31.58, 256.76), (84.87, 31.36, 257.63), (84.05, 30.82, 257.4), (83.24, 30.28, 257.18), (82.42, 29.74, 256.95), (81.57, 29.25, 256.79), (80.71, 28.75, 256.63), (79.85, 28.26, 256.48), (78.99, 27.76, 256.32), (78.14, 27.27, 256.17), (77.28, 26.77, 256.01), (76.42, 26.28, 255.86), (75.56, 25.79, 255.71), (74.7, 25.29, 255.56), (73.85, 24.76, 255.51), (73.03, 24.19, 255.58), (72.2, 23.63, 255.66), (71.38, 23.07, 255.75), (70.55, 22.51, 255.83), (69.73, 21.94, 255.93), (68.91, 21.38, 256.03), (68.09, 20.82, 256.15), (67.27, 20.25, 256.27), (66.45, 19.69, 256.4), (65.63, 19.12, 256.54), (64.82, 18.56, 256.69), (64.01, 17.99, 256.85), (63.2, 17.42, 257.02), (62.4, 16.85, 257.21), (61.59, 16.29, 257.41), (60.79, 15.72, 257.62), (60.0, 15.15, 257.84), (59.2, 14.59, 258.06), (58.41, 14.02, 258.3), (57.62, 13.46, 258.53), (56.82, 12.89, 258.77), (56.04, 12.33, 259.03), (55.28, 11.77, 259.37), (54.52, 11.21, 259.71), (53.77, 10.64, 260.06), (53.01, 10.08, 260.4), (52.26, 9.51, 260.75), (51.51, 8.95, 261.11), (50.77, 8.38, 261.46), (50.02, 7.82, 261.82), (49.28, 7.25, 262.18), (48.53, 6.68, 262.54), (47.79, 6.12, 262.9), (47.05, 5.55, 263.26), (46.31, 4.97, 263.61), (45.57, 4.4, 263.97), (44.83, 3.81, 264.32), (44.09, 3.23, 264.67), (43.36, 2.65, 265.02), (42.62, 2.06, 265.36), (41.89, 1.47, 265.7), (41.15, 0.87, 266.03), (40.42, 0.27, 266.36), (39.68, -0.33, 266.68), (38.95, -0.93, 267.0), (38.22, -1.55, 267.3), (37.49, -2.16, 267.61), (36.76, -2.78, 267.91), (36.02, -3.39, 268.2), (35.3, -4.02, 268.49), (34.57, -4.65, 268.77), (33.84, -5.28, 269.05), (33.11, -5.91, 269.32), (32.39, -6.55, 269.59), (31.67, -7.19, 269.86), (30.95, -7.84, 270.12), (30.23, -8.49, 270.37), (29.51, -9.14, 270.63), (28.79, -9.79, 270.88), (28.08, -10.45, 271.13), (27.37, -11.11, 271.38), (26.66, -11.78, 271.63), (25.95, -12.44, 271.87), (25.24, -13.11, 272.12), (24.54, -13.78, 272.36), (23.83, -14.45, 272.61), (23.13, -15.12, 272.85), (22.43, -15.79, 273.1), (21.72, -16.46, 273.34), (21.02, -17.14, 273.58), (20.32, -17.81, 273.83), (19.62, -18.48, 274.07), (18.92, -19.16, 274.31), (18.23, -19.84, 274.55), (17.53, -20.52, 274.79), (16.83, -21.2, 275.03), (16.14, -21.88, 275.28), (15.41, -22.51, 275.56), (14.48, -22.59, 275.8), (13.47, -22.59, 275.8), (12.47, -22.59, 275.8), (11.47, -22.59, 275.8)],
    "mic_pin17_splitleg": [(87.67, 31.58, 254.76), (87.67, 31.58, 255.76), (87.67, 31.58, 256.76), (87.56, 31.47, 257.74), (86.86, 30.75, 257.81), (86.17, 30.04, 257.88), (85.45, 29.35, 257.97), (84.7, 28.68, 258.02), (83.96, 28.02, 258.06), (83.21, 27.36, 258.1), (82.46, 26.69, 258.14), (81.71, 26.03, 258.18), (80.96, 25.37, 258.22), (80.21, 24.71, 258.26), (79.46, 24.05, 258.3), (78.71, 23.39, 258.34), (77.96, 22.73, 258.38), (77.21, 22.07, 258.42), (76.46, 21.41, 258.46), (75.67, 20.79, 258.44), (74.82, 20.27, 258.35), (73.98, 19.74, 258.25), (73.13, 19.21, 258.16), (72.29, 18.69, 258.06), (71.44, 18.16, 257.97), (70.6, 17.63, 257.88), (69.75, 17.1, 257.79), (68.91, 16.58, 257.7), (68.06, 16.05, 257.62), (67.21, 15.52, 257.53), (66.36, 15.01, 257.5), (65.49, 14.51, 257.55), (64.62, 14.02, 257.59), (63.75, 13.53, 257.65), (62.87, 13.05, 257.7), (62.0, 12.57, 257.77), (61.12, 12.1, 257.84), (60.24, 11.63, 257.93), (59.35, 11.18, 258.03), (58.46, 10.73, 258.14), (57.57, 10.29, 258.25), (56.67, 9.87, 258.39), (55.77, 9.46, 258.53), (54.87, 9.06, 258.68), (53.96, 8.67, 258.84), (53.05, 8.3, 259.02), (52.13, 7.94, 259.21), (51.21, 7.6, 259.41), (50.29, 7.28, 259.62), (49.36, 6.97, 259.85), (48.44, 6.69, 260.09), (47.51, 6.41, 260.33), (46.57, 6.15, 260.59), (45.64, 5.91, 260.85), (44.71, 5.68, 261.14), (43.78, 5.47, 261.43), (42.85, 5.27, 261.74), (41.92, 5.08, 262.06), (40.99, 4.9, 262.38), (40.06, 4.74, 262.73), (39.14, 4.59, 263.08), (38.22, 4.46, 263.45), (37.3, 4.33, 263.82), (36.39, 4.21, 264.22), (35.47, 4.1, 264.62), (34.57, 4.0, 265.03), (33.66, 3.9, 265.44), (32.76, 3.81, 265.87), (31.87, 3.73, 266.31), (30.97, 3.64, 266.75), (30.09, 3.57, 267.21), (29.2, 3.49, 267.66), (28.32, 3.42, 268.14), (27.44, 3.35, 268.61), (26.57, 3.29, 269.09), (25.69, 3.23, 269.57), (24.82, 3.16, 270.06), (23.95, 3.1, 270.55), (23.09, 3.03, 271.05), (22.22, 2.97, 271.55), (21.36, 2.91, 272.05), (20.5, 2.85, 272.56), (19.64, 2.79, 273.06), (18.77, 2.73, 273.57), (17.91, 2.67, 274.07), (17.05, 2.61, 274.58), (16.19, 2.55, 275.09), (15.34, 2.6, 275.6), (14.43, 2.81, 275.8), (13.43, 2.81, 275.8), (12.43, 2.81, 275.8), (11.43, 2.81, 275.8), (10.43, 2.81, 275.8), (9.43, 2.81, 275.8), (8.93, 2.81, 275.8)],
    "mic_pin20": [(90.21, 31.58, 254.76), (90.21, 31.58, 255.76), (90.21, 31.58, 256.76), (89.93, 31.26, 257.53), (89.23, 30.66, 257.14), (88.53, 30.06, 256.75), (87.75, 29.52, 256.43), (86.9, 29.05, 256.2), (86.03, 28.61, 255.99), (85.15, 28.17, 255.79), (84.27, 27.73, 255.59), (83.39, 27.3, 255.41), (82.5, 26.87, 255.23), (81.61, 26.45, 255.06), (80.72, 26.03, 254.91), (79.82, 25.62, 254.77), (78.91, 25.22, 254.63), (78.0, 24.82, 254.51), (77.09, 24.43, 254.39), (76.18, 24.03, 254.29), (75.26, 23.64, 254.19), (74.34, 23.26, 254.09), (73.42, 22.89, 254.01), (72.48, 22.53, 253.97), (71.55, 22.18, 253.93), (70.61, 21.83, 253.9), (69.67, 21.49, 253.91), (68.72, 21.2, 254.02), (67.77, 20.93, 254.17), (66.82, 20.65, 254.31), (65.87, 20.38, 254.48), (64.94, 20.1, 254.73), (64.03, 19.83, 255.04), (63.12, 19.55, 255.35), (62.21, 19.28, 255.67), (61.3, 19.01, 255.99), (60.4, 18.73, 256.32), (59.5, 18.45, 256.66), (58.61, 18.16, 257.0), (57.72, 17.86, 257.35), (56.84, 17.57, 257.72), (55.96, 17.26, 258.08), (55.08, 16.95, 258.46), (54.21, 16.64, 258.83), (53.34, 16.32, 259.22), (52.48, 15.99, 259.6), (51.62, 15.65, 259.99), (50.77, 15.31, 260.39), (49.92, 14.97, 260.79), (49.08, 14.61, 261.2), (48.24, 14.25, 261.6), (47.4, 13.89, 262.01), (46.56, 13.52, 262.41), (45.73, 13.15, 262.82), (44.9, 12.77, 263.23), (44.07, 12.39, 263.64), (43.24, 12.0, 264.05), (42.42, 11.6, 264.45), (41.59, 11.21, 264.85), (40.77, 10.81, 265.26), (39.94, 10.41, 265.66), (39.12, 10.0, 266.06), (38.3, 9.6, 266.45), (37.47, 9.19, 266.85), (36.65, 8.78, 267.24), (35.82, 8.37, 267.62), (35.0, 7.96, 268.01), (34.17, 7.55, 268.4), (33.34, 7.14, 268.78), (32.51, 6.73, 269.17), (31.68, 6.32, 269.55), (30.85, 5.91, 269.92), (30.02, 5.5, 270.3), (29.18, 5.1, 270.67), (28.35, 4.69, 271.04), (27.51, 4.28, 271.41), (26.68, 3.88, 271.78), (25.84, 3.47, 272.15), (25.0, 3.06, 272.52), (24.17, 2.66, 272.88), (23.33, 2.26, 273.25), (22.45, 1.92, 273.58), (21.54, 1.61, 273.88), (20.62, 1.34, 274.17), (19.69, 1.1, 274.44), (18.74, 0.92, 274.69), (17.78, 0.8, 274.93), (16.82, 0.67, 275.17), (15.85, 0.54, 275.41), (14.97, 0.28, 275.79), (13.97, 0.27, 275.8), (12.97, 0.27, 275.8), (11.97, 0.27, 275.8), (11.47, 0.27, 275.8)],
    "mic_pin9": [(85.13, 39.04, 254.76), (85.13, 39.04, 255.76), (85.13, 39.04, 256.76), (85.1, 39.07, 257.76), (84.17, 39.09, 258.13), (83.24, 39.12, 258.5), (82.31, 39.14, 258.88), (81.4, 39.03, 259.27), (80.49, 38.92, 259.67), (79.58, 38.8, 260.07), (78.67, 38.68, 260.48), (77.77, 38.55, 260.88), (76.86, 38.41, 261.29), (75.96, 38.27, 261.7), (75.06, 38.12, 262.11), (74.16, 37.96, 262.53), (73.26, 37.8, 262.94), (72.37, 37.62, 263.36), (71.48, 37.45, 263.78), (70.59, 37.26, 264.2), (69.7, 37.07, 264.62), (68.82, 36.86, 265.05), (67.93, 36.66, 265.47), (67.05, 36.45, 265.89), (66.17, 36.23, 266.31), (65.29, 36.0, 266.74), (64.41, 35.77, 267.16), (63.53, 35.54, 267.58), (62.66, 35.31, 268.01), (61.78, 35.07, 268.43), (60.9, 34.84, 268.85), (60.03, 34.6, 269.27), (59.12, 34.28, 269.56), (58.21, 33.89, 269.68), (57.3, 33.5, 269.81), (56.38, 33.11, 269.92), (55.47, 32.72, 270.04), (54.55, 32.33, 270.15), (53.64, 31.93, 270.26), (52.72, 31.54, 270.37), (51.81, 31.15, 270.48), (50.89, 30.75, 270.58), (49.98, 30.36, 270.67), (49.06, 29.97, 270.76), (48.15, 29.57, 270.85), (47.23, 29.18, 270.94), (46.31, 28.78, 271.02), (45.4, 28.39, 271.11), (44.48, 27.99, 271.19), (43.56, 27.59, 271.27), (42.65, 27.2, 271.34), (41.73, 26.81, 271.42), (40.81, 26.42, 271.49), (39.88, 26.04, 271.54), (38.95, 25.67, 271.6), (38.03, 25.29, 271.65), (37.1, 24.92, 271.71), (36.18, 24.53, 271.77), (35.26, 24.15, 271.82), (34.34, 23.76, 271.89), (33.42, 23.36, 271.96), (32.5, 22.96, 272.03), (31.59, 22.56, 272.1), (30.68, 22.15, 272.18), (29.77, 21.74, 272.26), (28.86, 21.32, 272.34), (27.96, 20.9, 272.42), (27.05, 20.48, 272.51), (26.15, 20.05, 272.6), (25.25, 19.62, 272.69), (24.35, 19.19, 272.78), (23.46, 18.75, 272.87), (22.56, 18.31, 272.97), (21.78, 17.74, 273.19), (21.04, 17.12, 273.46), (20.3, 16.5, 273.74), (19.56, 15.89, 274.01), (18.82, 15.27, 274.28), (18.08, 14.65, 274.55), (17.34, 14.03, 274.82), (16.6, 13.42, 275.1), (15.84, 12.83, 275.38), (14.93, 12.9, 275.74), (13.94, 12.97, 275.8), (12.94, 12.97, 275.8), (11.93, 12.97, 275.8), (10.93, 12.97, 275.8), (9.93, 12.97, 275.8), (8.93, 12.97, 275.8)],
    "mic_pin35_splitleg": [(87.67, 39.04, 254.76), (87.67, 39.04, 255.76), (87.67, 39.04, 256.76), (87.63, 39.08, 257.76), (86.85, 39.25, 258.22), (86.18, 39.01, 258.93), (85.48, 38.83, 259.61), (84.74, 38.68, 260.28), (84.01, 38.53, 260.94), (83.27, 38.38, 261.61), (82.54, 38.24, 262.27), (81.8, 38.09, 262.93), (81.06, 37.95, 263.59), (80.32, 37.81, 264.25), (79.58, 37.67, 264.91), (78.83, 37.54, 265.56), (78.09, 37.41, 266.21), (77.34, 37.28, 266.87), (76.58, 37.15, 267.51), (75.83, 37.02, 268.16), (75.08, 36.9, 268.81), (74.33, 36.77, 269.46), (73.58, 36.65, 270.11), (72.81, 36.52, 270.74), (71.95, 36.26, 271.15), (71.03, 35.9, 271.32), (70.12, 35.53, 271.48), (69.2, 35.16, 271.66), (68.29, 34.78, 271.83), (67.39, 34.39, 272.0), (66.48, 34.0, 272.17), (65.59, 33.58, 272.34), (64.7, 33.16, 272.52), (63.81, 32.73, 272.7), (62.93, 32.29, 272.88), (62.06, 31.82, 273.06), (61.2, 31.35, 273.24), (60.35, 30.85, 273.43), (59.51, 30.35, 273.61), (58.67, 29.83, 273.81), (57.85, 29.29, 274.0), (57.04, 28.74, 274.2), (56.24, 28.18, 274.39), (55.44, 27.6, 274.59), (54.66, 27.01, 274.79), (53.88, 26.41, 274.99), (53.12, 25.8, 275.2), (52.36, 25.17, 275.4), (51.62, 24.54, 275.61), (50.87, 23.9, 275.82), (50.14, 23.26, 276.03), (49.41, 22.6, 276.23), (48.68, 21.94, 276.44), (47.97, 21.28, 276.65), (47.25, 20.62, 276.86), (46.53, 19.95, 277.07), (45.81, 19.28, 277.28), (45.16, 18.53, 277.41), (44.57, 17.73, 277.47), (43.97, 16.93, 277.54), (43.37, 16.13, 277.6), (42.77, 15.33, 277.66), (42.17, 14.54, 277.71), (41.56, 13.74, 277.77), (40.95, 12.95, 277.82), (40.35, 12.15, 277.87), (39.74, 11.36, 277.91), (39.12, 10.58, 277.95), (38.5, 9.79, 277.98), (37.88, 9.01, 278.01), (37.25, 8.23, 278.04), (36.62, 7.45, 278.06), (35.99, 6.67, 278.08), (35.36, 5.9, 278.09), (34.72, 5.13, 278.1), (34.08, 4.36, 278.1), (33.44, 3.59, 278.09), (32.79, 2.82, 278.07), (32.14, 2.06, 278.06), (31.49, 1.3, 278.04), (30.83, 0.55, 278.01), (30.18, -0.21, 277.97), (29.52, -0.96, 277.93), (28.86, -1.71, 277.88), (28.2, -2.46, 277.82), (27.54, -3.21, 277.76), (26.88, -3.96, 277.7), (26.22, -4.71, 277.62), (25.56, -5.46, 277.54), (24.9, -6.2, 277.46), (24.24, -6.95, 277.36), (23.58, -7.7, 277.26), (22.92, -8.44, 277.15), (22.26, -9.19, 277.04), (21.6, -9.93, 276.93), (20.94, -10.68, 276.81), (20.29, -11.42, 276.69), (19.63, -12.17, 276.56), (18.98, -12.92, 276.43), (18.33, -13.67, 276.29), (17.67, -14.41, 276.15), (17.02, -15.16, 276.01), (16.38, -15.91, 275.86), (15.73, -16.66, 275.72), (15.17, -17.48, 275.69), (14.97, -18.38, 276.04), (14.62, -19.17, 276.54), (13.87, -19.7, 276.92), (12.93, -19.95, 277.11), (11.93, -20.04, 277.19), (10.93, -20.05, 277.2), (9.93, -20.05, 277.2), (8.93, -20.05, 277.2)],
    "mic_pin12_splitleg": [(90.21, 39.04, 254.76), (90.21, 39.04, 255.76), (90.21, 39.04, 256.76), (90.21, 39.05, 257.76), (89.65, 39.2, 258.36), (88.96, 39.0, 259.05), (88.27, 38.78, 259.75), (87.58, 38.55, 260.44), (86.89, 38.33, 261.13), (86.2, 38.1, 261.82), (85.5, 37.88, 262.5), (84.8, 37.66, 263.19), (84.11, 37.44, 263.87), (83.4, 37.22, 264.55), (82.7, 37.0, 265.23), (81.99, 36.79, 265.9), (81.28, 36.57, 266.58), (80.57, 36.36, 267.25), (79.86, 36.15, 267.93), (79.15, 35.94, 268.6), (78.44, 35.72, 269.27), (77.72, 35.51, 269.94), (77.01, 35.29, 270.61), (76.3, 35.08, 271.28), (75.47, 34.75, 271.72), (74.62, 34.37, 272.09), (73.77, 34.0, 272.46), (72.92, 33.62, 272.83), (72.05, 33.21, 273.06), (71.16, 32.75, 273.11), (70.26, 32.3, 273.16), (69.37, 31.85, 273.21), (68.48, 31.39, 273.25), (67.58, 30.94, 273.29), (66.69, 30.5, 273.32), (65.79, 30.06, 273.35), (64.89, 29.62, 273.37), (63.98, 29.2, 273.38), (63.07, 28.78, 273.39), (62.16, 28.36, 273.39), (61.25, 27.95, 273.38), (60.33, 27.55, 273.37), (59.41, 27.16, 273.35), (58.49, 26.77, 273.33), (57.56, 26.39, 273.29), (56.63, 26.02, 273.25), (55.7, 25.66, 273.2), (54.76, 25.3, 273.15), (53.82, 24.95, 273.1), (52.88, 24.61, 273.04), (51.94, 24.27, 272.98), (51.0, 23.95, 272.92), (50.05, 23.62, 272.85), (49.1, 23.31, 272.79), (48.16, 22.99, 272.73), (47.21, 22.68, 272.66), (46.26, 22.36, 272.6), (45.31, 22.05, 272.54), (44.35, 21.74, 272.48), (43.4, 21.44, 272.43), (42.45, 21.13, 272.38), (41.5, 20.82, 272.34), (40.55, 20.51, 272.31), (39.6, 20.2, 272.27), (38.65, 19.88, 272.25), (37.7, 19.56, 272.23), (36.75, 19.24, 272.22), (35.8, 18.91, 272.21), (34.86, 18.58, 272.21), (33.91, 18.25, 272.22), (32.97, 17.91, 272.23), (32.03, 17.57, 272.24), (31.09, 17.22, 272.26), (30.15, 16.88, 272.29), (29.21, 16.53, 272.32), (28.27, 16.19, 272.36), (27.33, 15.84, 272.4), (26.39, 15.49, 272.43), (25.46, 15.13, 272.49), (24.63, 14.7, 272.85), (23.8, 14.27, 273.22), (22.97, 13.85, 273.59), (22.15, 13.42, 273.96), (21.32, 13.0, 274.34), (20.49, 12.57, 274.71), (19.67, 12.14, 275.08), (18.84, 11.71, 275.45), (18.02, 11.28, 275.82), (17.19, 10.86, 276.19), (16.34, 10.5, 276.57), (15.43, 10.45, 277.0), (14.48, 10.43, 277.2), (13.47, 10.43, 277.2), (12.47, 10.43, 277.2), (11.47, 10.43, 277.2)],
    "bus_s1_leg2_ext": [(59.4, 26.85, 268.35), (59.4, 27.85, 268.35), (59.41, 28.85, 268.35), (60.4, 29.02, 268.29), (61.38, 29.19, 268.22), (62.3, 28.91, 267.95), (62.94, 28.26, 267.54), (63.38, 27.48, 267.1), (63.73, 26.65, 266.67), (64.04, 25.8, 266.23), (64.11, 25.32, 265.39), (64.05, 25.12, 264.41), (63.98, 24.97, 263.42), (63.92, 24.9, 262.43), (63.86, 24.84, 261.43), (63.79, 24.78, 260.43), (63.73, 24.72, 259.44), (63.66, 24.66, 258.44), (63.6, 24.61, 257.44), (63.53, 24.56, 256.45), (63.46, 24.51, 255.45), (63.38, 24.46, 254.45), (63.31, 24.43, 253.46), (63.23, 24.4, 252.46), (63.15, 24.37, 251.46), (63.07, 24.35, 250.46), (62.98, 24.33, 249.47), (62.89, 24.32, 248.47), (62.8, 24.31, 247.47), (62.7, 24.31, 246.48), (62.6, 24.31, 245.48), (62.5, 24.32, 244.49), (62.39, 24.33, 243.49), (62.29, 24.34, 242.5), (62.18, 24.36, 241.5), (62.07, 24.37, 240.51), (61.96, 24.4, 239.51), (61.83, 24.41, 238.52), (61.67, 24.41, 237.53), (61.52, 24.42, 236.55), (61.36, 24.42, 235.56), (60.48, 23.96, 235.43), (59.29, 23.32, 235.35), (57.31, 22.1, 235.19), (55.29, 21.05, 234.47), (53.99, 20.25, 232.58), (52.5, 19.78, 230.68), (50.52, 19.62, 230.48), (48.52, 19.61, 230.47), (46.03, 19.75, 230.42), (43.56, 20.0, 230.32), (41.59, 20.09, 229.94), (40.02, 20.19, 228.92), (39.78, 20.3, 226.86), (39.61, 20.4, 224.61), (39.81, 20.6, 222.23), (40.02, 20.82, 220.04), (40.04, 21.18, 217.6), (41.0, 22.0, 215.5)],
    "amp_BCLK_pin12_splitleg": [(7.47, 10.43, 275.8), (6.47, 10.43, 275.8), (5.72, 10.14, 275.38), (5.22, 9.45, 274.85), (4.78, 8.72, 274.32), (4.41, 7.96, 273.79), (4.07, 7.17, 273.26), (3.74, 6.39, 272.74), (3.38, 5.62, 272.21), (2.68, 5.05, 271.79), (1.86, 4.61, 271.41), (0.96, 4.64, 271.03), (0.05, 4.86, 270.67), (-0.86, 5.08, 270.3), (-1.76, 5.31, 269.92), (-2.65, 5.55, 269.54), (-3.54, 5.79, 269.15), (-4.42, 6.05, 268.74), (-5.29, 6.32, 268.33), (-6.16, 6.6, 267.91), (-7.01, 6.9, 267.47), (-7.85, 7.2, 267.02), (-8.68, 7.53, 266.56), (-9.5, 7.86, 266.08), (-10.29, 8.22, 265.59), (-11.08, 8.58, 265.09), (-11.86, 8.95, 264.58), (-12.63, 9.33, 264.06), (-13.38, 9.73, 263.53), (-14.13, 10.13, 262.99), (-14.87, 10.53, 262.45), (-15.6, 10.95, 261.91), (-16.33, 11.36, 261.36), (-17.05, 11.78, 260.8), (-17.72, 12.23, 260.21), (-18.3, 12.73, 259.56), (-18.87, 13.23, 258.91), (-19.44, 13.74, 258.26), (-20.01, 14.25, 257.61), (-20.57, 14.75, 256.95), (-21.14, 15.26, 256.3), (-21.7, 15.77, 255.64), (-22.26, 16.27, 254.98), (-22.81, 16.78, 254.32), (-23.37, 17.29, 253.66), (-23.92, 17.8, 253.0), (-24.47, 18.31, 252.33), (-25.02, 18.81, 251.66), (-25.57, 19.32, 250.99), (-26.11, 19.82, 250.32), (-26.66, 20.32, 249.64), (-27.2, 20.83, 248.97), (-27.74, 21.33, 248.29), (-28.28, 21.83, 247.61), (-28.82, 22.34, 246.94), (-29.37, 22.84, 246.26), (-29.91, 23.33, 245.58), (-30.49, 23.71, 244.86), (-31.09, 24.01, 244.11), (-31.69, 24.0, 243.32), (-32.05, 23.43, 242.58), (-32.42, 22.86, 241.84), (-32.78, 22.29, 241.1), (-32.62, 21.67, 240.87), (-32.12, 21.65, 240.88)],
    "amp_DIN_pin40": [(8.06, -23.72, 275.8), (7.35, -23.01, 275.8), (6.53, -22.52, 275.51), (5.69, -22.16, 275.08), (4.86, -21.81, 274.64), (4.03, -21.46, 274.2), (3.2, -21.1, 273.77), (2.45, -20.59, 273.36), (1.72, -20.02, 272.96), (0.99, -19.46, 272.56), (0.27, -18.89, 272.15), (-0.45, -18.32, 271.75), (-1.17, -17.75, 271.35), (-1.89, -17.18, 270.94), (-2.6, -16.59, 270.54), (-3.31, -16.01, 270.13), (-4.01, -15.43, 269.72), (-4.71, -14.84, 269.31), (-5.4, -14.24, 268.89), (-6.09, -13.64, 268.48), (-6.77, -13.04, 268.06), (-7.45, -12.43, 267.63), (-8.13, -11.83, 267.21), (-8.79, -11.21, 266.78), (-9.46, -10.59, 266.35), (-10.12, -9.97, 265.92), (-10.77, -9.35, 265.49), (-11.42, -8.72, 265.05), (-12.07, -8.09, 264.62), (-12.72, -7.46, 264.18), (-13.36, -6.83, 263.74), (-14.0, -6.19, 263.29), (-14.63, -5.56, 262.85), (-15.27, -4.92, 262.4), (-15.9, -4.29, 261.95), (-16.54, -3.65, 261.51), (-17.16, -3.01, 261.06), (-17.67, -2.36, 260.49), (-18.15, -1.71, 259.89), (-18.63, -1.06, 259.3), (-19.12, -0.41, 258.71), (-19.6, 0.24, 258.12), (-20.07, 0.89, 257.52), (-20.55, 1.54, 256.92), (-21.02, 2.2, 256.32), (-21.49, 2.85, 255.72), (-21.96, 3.5, 255.12), (-22.42, 4.15, 254.52), (-22.88, 4.81, 253.91), (-23.34, 5.47, 253.31), (-23.8, 6.13, 252.7), (-24.25, 6.78, 252.1), (-24.7, 7.44, 251.49), (-25.15, 8.1, 250.88), (-25.6, 8.76, 250.27), (-26.04, 9.42, 249.65), (-26.48, 10.08, 249.04), (-26.92, 10.74, 248.43), (-27.36, 11.4, 247.81), (-27.8, 12.06, 247.2), (-28.23, 12.72, 246.58), (-28.66, 13.39, 245.97), (-29.09, 14.06, 245.36), (-29.53, 14.72, 244.74), (-29.96, 15.38, 244.13), (-30.4, 16.05, 243.51), (-30.83, 16.71, 242.9), (-31.26, 17.38, 242.29), (-31.69, 18.04, 241.67), (-32.13, 18.71, 241.06), (-32.45, 19.4, 240.42), (-32.67, 20.12, 239.76), (-32.89, 20.84, 239.1), (-33.11, 21.56, 238.43), (-32.12, 21.65, 238.34)],
}
HR43D_CSI_FOLDS = ((64.5, -0.5, 274.85, "z"), (64.5, 12.5, 274.85, "z"), (-3.6, 12.5, 261.5, "z"))
_HR43D_CSI_PTS = {
    "csi43c_a": [(88.02, -0.5, 273.96), (88.02, -0.5, 274.6), (87.3, -0.5, 275.3), (86.5, -0.5, 275.5), (80.0, -0.5, 275.5), (73.5, -0.5, 275.5),
                 (70.5, -0.5, 274.95), (67.0, -0.5, 274.85), (64.5, -0.5, 274.85)],
    "csi43c_b": [(64.5, -0.5, 274.85), (64.5, 5.0, 274.85), (64.5, 12.5, 274.85)],
    "csi43c_c": [(64.5, 12.5, 274.85), (50.0, 12.5, 274.85), (37.0, 12.5, 274.85)] + _HR43C_CSI_PTS["csi43c_c"][3:],
    "csi43c_d": list(_HR43C_CSI_PTS["csi43c_d"]),
}


def _hr43d_csi():
    """hr43d：CSI 排线 4 段（3 折）= hr43c 候选路径，板上方那段抬 0.35、折 1/2 挪到 x 64.5（见 _HR43D_CSI_PTS）"""
    Yv, Xv = np.array([0, 1.0, 0]), np.array([1.0, 0, 0])
    hw = CABLE["fpc_csi"]["w"] / 2; t = CABLE["fpc_csi"]["t"]
    folds = [abox((x - hw, y - hw, z - t), (x + hw, y + hw, z + t)) for (x, y, z, _) in HR43D_CSI_FOLDS]
    side = {"csi43c_a": Yv, "csi43c_b": Xv, "csi43c_c": Yv, "csi43c_d": Xv}
    f1, f2, f3 = HR43D_CSI_FOLDS
    ends = {"csi43c_a": ("摄像头 FPC 翻盖座（倒装，背面顶边，座顶 z 273.94，线朝 +z 出）", f"折 1 ({f1[0]}, {f1[1]}, {f1[2]})"),
            "csi43c_b": ("折 1", f"折 2 ({f2[0]}, {f2[1]}, {f2[2]})"), "csi43c_c": ("折 2", f"折 3 ({f3[0]}, {f3[1]}, {f3[2]})"),
            "csi43c_d": ("折 3", "Radxa CSI 22P 座（板底 +y 端，hr43c 板位 x −11.7..4.5 × y 25.5..31.25，沿 −y 插入 4，槽中面 z 254.95）")}
    for k, pts in _HR43D_CSI_PTS.items():
        w = dict(id=k, harness="HB06", cable="fpc_csi", pts=list(pts), sides=[side[k]] * len(pts), group="csi", creases=0, ends=ends[k], hr43c_rerouted=True, hr43d_rerouted=True,
                 bought=(200.0 if k == "csi43c_a" else 0.0), bought_src=("CSI 200 mm 22→15 同面线（用户未答买不买；150 不够，见 _HR43_CSI 注）" if k == "csi43c_a" else "与 csi43c_a 同一根"),
                 note="hr43c B2 候选路径；hr43d：板上方段抬 0.35、折 1/2 挪到 x 64.5（转接板平放后离板 ≥1.05，见 _HR43D_CSI_PTS 上方注释）")
        if k == "csi43c_a":
            w.update(wide_head=True, fixed_mm=3 * CABLE["fpc_csi"]["w"] + 3.0, creases=3, crease_at="3 处 45° 折：" + "；".join(f"({x}, {y}, {z})" for (x, y, z, _) in HR43D_CSI_FOLDS),
                     plugs=folds)
        WIRES.append(w)


def _apply_hr43d():
    """_define() 末尾（_apply_hr43c 之后）调：S1 转接块挪位、整根替换 _HR43D_ROUTES 里的线（两端说明按新位置重写）。CSI 由 _HR43_CSI（= _hr43d_csi）在 _apply_hr43 里加。"""
    if HR43D_S1_JUNCTION is not None:
        S1_JUNCTIONS_AT[:] = [(np.array(HR43D_S1_JUNCTION[0], float), np.array(HR43D_S1_JUNCTION[1], float))]
    if not _HR43D_ROUTES: return
    from . import electronics as E
    for w in WIRES:
        if w["id"] not in _HR43D_ROUTES: continue
        w["pts"] = [tuple(p) for p in _HR43D_ROUTES[w["id"]]]; w["hr43d_rerouted"] = True
        w.pop("sides", None)
        n_ = len(w["pts"])                                    # 扁线宽向：尾段沿用旧线口径（腿② 下右前耳那 8 点宽向 y；嘴舵机 PH 短线最后 13 点宽向 = 针排向），其余自动
        if w["id"] == "bus_s1_leg2_ext": w["sides"] = [np.array([1.0, 0, 0])] * 5 + [None] * (n_ - 22) + [np.array([0, 1.0, 0])] * 17   # PH 口处宽向 = 针排向 x；尾段 hr43c 旧线宽向 y
        elif w["id"] == "bus_s1_leg1": w["sides"] = [np.array([1.0, 0, 0])] * n_                                                    # 宽向 x（PH 针排向），竖下时厚向朝 H01 +y 立柱吊梁
        elif w["id"] == "bus_jaw_ph": w["sides"] = [None] * (n_ - 13) + [np.array(JAW_PIN_ROW)] * 13
        e0, e1 = w.get("ends") or ("", "")
        if w["id"].startswith("amp_"):
            nm = w["id"].split("_")[1]; n, k = HR43C_LEG[w["id"]]; s = [ss for (nn, kk, ss) in E.gpio_dup_legs() if nn == n and kk == k][0]
            pe = E.gpio_dup_leg_end(n, s, k)
            w["ends"] = (f"Radxa pin {n}（{PINMAP[n][0]}）直插杜邦壳顶出线即弯朝 −x（hr43d，Rc 2.0，弯段在 zz_gpio_dupont）→ 接续点 ({pe[0]:.2f}, {pe[1]:.2f}, {pe[2]:.2f})",
                         f"功放 {nm} 直排针杜邦壳尾（后脑 +y，壳尾 x {HR43D_AMP_TAIL['x']}、y {HR43D_AMP_TAIL['y']}、z {HR43D_AMP_PIN_Z[nm]}，hr43d）")
        elif w["id"] == "spk_pair":
            w["ends"] = ("喇叭背面焊片（凸台底 z 275.25）", f"功放端子块 +y 接线口（后脑 +y，y {HR43D_AMP_TERM['y_face']}，hr43d；喇叭自带线 ≈100 assumed）")
        elif w["id"] == "usb_c":
            w["ends"] = (e0, f"转接板 Type-C 90° 弯头（B 边 −y，平放，线从 −x 面进，x {HR43D_USB_PLUG_ADP['exit'][0]} y {HR43D_USB_PLUG_ADP['exit'][1]} z {HR43D_USB_PLUG_ADP['exit'][2]}，hr43d）")
        elif w["id"] == "xt30_12v":
            p = adp_port("XT30in")["exit"]; w["ends"] = (e0, f"转接板 XT30 主输入（B 边 −y，平放，插头尾 ({p[0]:.1f}, {p[1]:.1f}, {p[2]:.2f})，hr43d）← XT30U-F")
        elif w["id"] == "ubec_in":
            p = adp_port("XT30ext")["exit"]; w["ends"] = (e0, f"XT30U-F 母头 → 转接板 XT30(2+2) 扩展口（A 边 +y，平放，插头尾 ({p[0]:.1f}, {p[1]:.1f}, {p[2]:.2f})，hr43d）")
        elif w["id"] in ("bus_s1_leg1", "bus_s1_leg2_ext"):
            p = adp_port("PH")["exit"]; w["ends"] = (f"转接板 PH 座（A 边 +y，平放，插头尾 ({p[0]:.1f}, {p[1]:.1f}, {p[2]:.2f})，hr43d）← S1 公壳", e1)
        w["note"] = (w.get("note", "") + "｜hr43d：转接板平放 H08 / 功放后脑 H09 后重走（hr43_work/hr43d/d_route.py）").lstrip("｜")


_HR43_CSI = _hr43d_csi            # hr43d（旧值 _hr43c_csi 见上）

CABLE["spk_pair2"] = dict(kind="pair", od=1.0, n=2, min_bend=2.0, src="assumed（喇叭自带两根 ≈28AWG Ø1.0 并行）")


# ━━━━━━━━━━━━━━━━━━━━ hr44（2026-09-25）头内线束规整：按去向扎成 4 束 ━━━━━━━━━━━━━━━━━━━━
#   用户（hr44 简报）：「相近的线扎成束走一起，要规整；现在头里的线看着乱」；22:55 口径：**头内不加扎带座 / 线夹几何**，扎成束后用胶带 / 泡棉胶贴壳内面（H05 / H03），
#   只在 assembly 写贴哪几处。
#   4 束（成员 = 排针上直插的杜邦 15 根 + UBEC 输出 2 根，其余线 hr43d 原样）：IMU 4（→ 出头口 R 接过颈束 HB09-R）/ 麦克风 6（→ 脸后麦克风排针）/
#   功放 5（→ 后脑 +y 功放直排针）/ UBEC 输出 2（UBEC 顶 → Radxa pin 4 / 34）。
#   做法（hr43_work/hr44/p09_trunks_relax.py → p13_lanes.py → p14_eval.py → p24_head_data.py）：干线 = 头腔体素（现 placed 重建）最短路 + 弹性带平滑（净空 ≥ 束半径 + 0.6、
#   弯半径 ≥ 6）；束截面 = 等径圆最密堆（4 根 2.414d / 5 根 2.701d / 6 根 3d）；干线中段各线 = 旋转最小标架上的平行车道（不互穿），两端尾线体素路由 + 修弯；
#   扎带位 = 车道段上均布 2–3 道（tie__，扎带 2.5 宽 × 1.0 厚 assumed，环内径 = 束径）。数据：duckstructure/data/wiring_head_hr44.json（旧航点 = _HR43D_ROUTES / _HR43C_ROUTES）。
HR44_DATA = os.path.join(os.path.dirname(os.path.abspath(__file__)), "data", "wiring_head_hr44.json")
HR44_TIE = dict(w=2.5, t=1.0, src="assumed：尼龙扎带 2.5 × 100（采购清单:75 已有）宽 2.5、厚 ≈1.0；环内径 = 束径")
HR44_BUNDLE_OF = {}          # 线名 → 束名（_apply_hr44 填）


def _hr44_data():
    return json.load(open(HR44_DATA, encoding="utf-8")) if os.path.exists(HR44_DATA) else None


def _apply_hr44():
    """_define() 末尾（_apply_hr43d 之后）调：17 根线整根换 hr44 束内航点；group 改成束名；两端说明不变。"""
    D = _hr44_data()
    if not D: return
    for nm, b in D["bundles"].items():
        for w in b["members"]: HR44_BUNDLE_OF[w] = nm
    for w in WIRES:
        if w["id"] not in D["routes"]: continue
        w["pts_was_until_2026_09_25_hr44"] = "见 _HR43D_ROUTES / _HR43C_ROUTES（hr43d 终版）"
        w["pts"] = [tuple(p) for p in D["routes"][w["id"]]]; w["hr44_rerouted"] = True
        w.pop("sides", None)
        nm = HR44_BUNDLE_OF[w["id"]]; b = D["bundles"][nm]
        w["group"] = "hr44_" + nm; w["bundle"] = nm
        w["note"] = (w.get("note", "") + f"｜hr44：扎成「{nm}」束（{len(b['members'])} 根，束径 {b['bundle_d']}，干线 {b['trunk_len']}，扎带 {len(b['ties'])} 道）").lstrip("｜")


def tie_solids_was_until_2026_09_29_hr52():
    """hr44 束上的扎带环（世界系零位）：tie__<束>_<k>（hr52 前版：只读 hr44 数据）"""
    D = _hr44_data(); out = {}
    if not D: return out
    for nm, b in D["bundles"].items():
        for k, t in enumerate(b["ties"]):
            ri = b["bundle_d"] / 2 + 0.05
            m = trimesh.creation.annulus(r_min=ri, r_max=ri + HR44_TIE["t"], height=HR44_TIE["w"], sections=24)
            z = np.array([0, 0, 1.0]); d = _u(t["t"])
            ax = np.cross(z, d); s = np.linalg.norm(ax)
            if s > 1e-9:
                m.apply_transform(trimesh.transformations.rotation_matrix(math.atan2(s, float(np.dot(z, d))), ax / s))
            elif np.dot(z, d) < 0:
                m.apply_transform(trimesh.transformations.rotation_matrix(math.pi, [1.0, 0, 0]))
            m.apply_translation(t["c"])
            out[f"tie__{nm}_{k + 1}"] = m
    return out


def tie_solids():
    """束上的扎带环（世界系零位）：tie__<束>_<k>。hr44 四束 + hr52 覆盖（wiring_head_hr52.json：tie_override 挪位、bundles 新束 tof）"""
    D = _hr44_data(); D52 = _hr52_data(); out = {}
    bundles = {nm: dict(b) for nm, b in (D or {}).get("bundles", {}).items()}
    if D52:
        for nm, ov in (D52.get("tie_override") or {}).items():
            ties = list(bundles[nm]["ties"])
            for k, t in ov.items(): ties[int(k)] = t
            bundles[nm]["ties"] = ties
        bundles.update(D52.get("bundles") or {})
    for nm, b in bundles.items():
        for k, t in enumerate(b["ties"]):
            ri = b["bundle_d"] / 2 + 0.05
            m = trimesh.creation.annulus(r_min=ri, r_max=ri + HR44_TIE["t"], height=HR44_TIE["w"], sections=24)
            z = np.array([0, 0, 1.0]); d = _u(t["t"])
            ax = np.cross(z, d); s = np.linalg.norm(ax)
            if s > 1e-9:
                m.apply_transform(trimesh.transformations.rotation_matrix(math.atan2(s, float(np.dot(z, d))), ax / s))
            elif np.dot(z, d) < 0:
                m.apply_transform(trimesh.transformations.rotation_matrix(math.pi, [1.0, 0, 0]))
            m.apply_translation(t["c"])
            out[f"tie__{nm}_{k + 1}"] = m
    return out



# ━━━━━━━━━━━━━━━━━━━━ hr52（2026-09-29）v6-VL53L5CX 雷达（ToF）4 根杜邦（HB12）+ 排针腿变化 + 麦克风束起头让开 ToF ━━━━━━━━━━━━━━━━━━━━
#   接线（components.yaml:tof_vl53l5cx）：VIN → Radxa 1（3.3 V，原空）、GND → 6（原空）、SDA → 3、SCL → 5（3 / 5 原 IMU 单线 → 杜邦「一分二 母对二公」单母头插排针，
#   两腿分 IMU / ToF，同 17 号那根）。ToF 端 = 模块背面直排针上的 4 个杜邦母壳（tof.points()["dupont_tail"]，壳尾 x 73.87，线沿 −x 出、最后 2 mm 沿 +x 直进壳）。
#   排针端（electronics.GPIO_DUP_* hr52 注）：1 号腿朝 37°（+x 偏 +y）后 R 2 回正（挤在 2 号竖直段与 H05 +y 吊柱削平面之间，到 H05 ≈0.1）、2 号抬 1.4 偏 −y 10°、
#   3 号一分二横排（IMU 在 y 19.89、ToF 在 21.29，s 1.5）、5 号一分二横排朝 −45°（s 0，躲 CSI 排线）、6 号朝 +x。
#   走法（hr52_work/harness/scripts：h14_trunk → h15_tof）：排针区 0.2 mm 细网格最短路 + 弹性带（净空 r + 0.45 起找，逐根占用；GND 经 (15.2, 19.7, 278.85) 从两根 SDA 上方过、
#   SCL 经 (19, 16.9, 278) 走 CSI 排线下 / IMU 两根下潜段之上、从 UBEC 输出线上方跨过）→ x 33 起 2×2 束（头腔体素最短路 + 弹性带，束心 EDT ≥ 2.3）→ x 61.6 起四根同步 Hermite 扇出到壳尾。
#   连带改的旧线：imu_pin3（出壳点 y −0.7）、imu_pin5（腿改 −45°）、amp_VIN_pin2（腿抬 1.4、偏 −y 10°、出腿即 R 2 下潜）只改排针端那一小段（h10）；
#   麦克风 6 根（h18）：束起头（车道 x > 66）整体平移 (0, −2.0, −0.2)（x 66→72.5 smoothstep 过渡）——原车道 x 74..80 在 ToF SDA/SCL 壳底下（pin12 −0.63，v6_check 的 zz_tof 2.41 就是这段），
#   6 根车道在同一截面截断后 6 根尾线从麦排针重走（占用含 ToF 4 根）；麦束第 1 道扎带从干线 s 11.9 挪到 s 19.5（原位的环套住 ToF SDA 进壳段）。
HR52_DATA = os.path.join(os.path.dirname(os.path.abspath(__file__)), "data", "wiring_head_hr52.json")
PINMAP_was_until_2026_09_29_hr52 = dict(PINMAP)
PINMAP = {**PINMAP, 1: ("ToF VIN 3.3 V（hr52，原空）", "HB12"), 6: ("ToF GND（hr52，原空）", "HB12"),
          3: ("I²C3 SDA 一分二 D（IMU SDA + ToF SDA，hr52）", "HB05/HB12"), 5: ("I²C3 SCL 一分二 E（IMU SCL + ToF SCL，hr52）", "HB05/HB12")}
HR43C_LEG.update({"tof_SDA_pin3_splitleg": (3, 1), "tof_SCL_pin5_splitleg": (5, 1), "tof_GND_pin6": (6, 0), "tof_VIN_pin1": (1, 0)})   # hr52
BOUGHT["tof_dupont_10cm"] = (100.0, "hr52 assumed：ToF 4 根杜邦 母对母 10 cm（模型线长 ≈66..69 + 余量 20 → ≥ 89 → 选常见 10 cm，商品规格未定）；"
                                    "SDA / SCL 那两根母头插在一分二的公头上（一分二腿长未知，见 bought_plan），VIN / GND 两根整根直通")
HR52_BUNDLE_OF = {}          # 线名 → 束名（_apply_hr52 填：ToF 4 根 → "tof"）
HR52_WHY = {"imu_pin3": "3 号改一分二横排：IMU 腿出壳点 y 20.59 → 19.89（排针端 ≈7 mm 航点 y −0.7 渐变接回原线）",
            "imu_pin5": "5 号改一分二横排、腿朝 −45°（躲 CSI 排线 + 不跨 6 号壳）：排针端改成同心 R 3.4 回正 + 下潜，x ≥ 15.1 接回原线",
            "amp_VIN_pin2": "2 号腿抬 1.4、偏 −y 10°（跨 1 号 ToF VIN 出壳）：出腿即 R 2 下潜（躲 H05 内面台阶），x ≤ 0.95 接回原线",
            "mic_pin38": "麦束起头让 ToF：车道 x > 66 平移 (0, −2.0, −0.2)，尾线重走", "mic_pin17_splitleg": "麦束起头让 ToF：车道 x > 66 平移 (0, −2.0, −0.2)，尾线重走",
            "mic_pin20": "麦束起头让 ToF：车道 x > 66 平移 (0, −2.0, −0.2)，尾线重走", "mic_pin9": "麦束起头让 ToF：车道 x > 66 平移 (0, −2.0, −0.2)，尾线重走",
            "mic_pin12_splitleg": "麦束起头让 ToF：车道 x > 66 平移 (0, −2.0, −0.2)，尾线重走（原车道 x 74..80 穿 zz_tof 0.63 / 尾线穿 H10 压条）",
            "mic_pin35_splitleg": "麦束起头让 ToF：车道 x > 66 平移 (0, −2.0, −0.2)，尾线重走"}
HR52_TOF_NAME = {"tof_SDA_pin3_splitleg": "SDA", "tof_SCL_pin5_splitleg": "SCL", "tof_GND_pin6": "GND", "tof_VIN_pin1": "VIN"}


def _hr52_data():
    return json.load(open(HR52_DATA, encoding="utf-8")) if os.path.exists(HR52_DATA) else None


def _apply_hr52():
    """_define() 末尾（_apply_hr44 之后）调：3 根改排针端 + 麦 6 根整根换 hr52 航点；新加 ToF 4 根（HB12）；ToF 束登记到 HR52_BUNDLE_OF。"""
    D = _hr52_data()
    if not D: return
    from . import electronics as E
    for w in WIRES:
        if w["id"] not in D["routes"]: continue
        w["pts_was_until_2026_09_29_hr52"] = "见 duckstructure/data/wiring_head_hr44.json（hr44 束内航点）"
        w["pts"] = [tuple(p) for p in D["routes"][w["id"]]]; w["hr52_rerouted"] = True
        w.pop("sides", None)
        w["note"] = (w.get("note", "") + "｜hr52：" + HR52_WHY.get(w["id"], "重走")).lstrip("｜")
        if w["id"] in ("imu_pin3", "imu_pin5"):
            n, k = HR43C_LEG[w["id"]]; s = [ss for (nn, kk, ss) in E.gpio_dup_legs() if nn == n and kk == k][0]; pe = E.gpio_dup_leg_end(n, s, k)
            w["bought_plan"] = BOUGHT["splitter_leg"][0]
            w["ends"] = (w["ends"][0], f"Radxa pin {n}（{PINMAP[n][0]}）一分二单母头直插、本腿（IMU）出壳即弯（弯段在 zz_gpio_dupont）→ 接续点 ({pe[0]:.2f}, {pe[1]:.2f}, {pe[2]:.2f})，hr52")
        elif w["id"] == "amp_VIN_pin2":
            s = [ss for (nn, kk, ss) in E.gpio_dup_legs() if nn == 2 and kk == 0][0]; pe = E.gpio_dup_leg_end(2, s, 0)
            w["ends"] = (f"Radxa pin 2（{PINMAP[2][0]}）直插杜邦壳顶先竖直 1.4 再弯朝 (−cos10°, −sin10°)（hr52，跨 1 号 ToF VIN 出壳；弯段在 zz_gpio_dupont）→ 接续点 ({pe[0]:.2f}, {pe[1]:.2f}, {pe[2]:.2f})", w["ends"][1])
    tails = {}
    try:
        from . import tof as TOF
        tails = TOF.points()["dupont_tail"]
    except Exception:                                            # noqa: BLE001  tof 模块缺 → 两端说明不带坐标
        pass
    for wid, spec in D.get("tof_wires", {}).items():
        n, k = spec["pin"], spec["leg"]; s = [ss for (nn, kk, ss) in E.gpio_dup_legs() if nn == n and kk == k][0]; pe = E.gpio_dup_leg_end(n, s, k)
        nm = HR52_TOF_NAME[wid]; t = tails.get(nm)
        split = n in E.GPIO_DUP_SPLIT
        WIRES.append(dict(
            id=wid, harness="HB12", cable="dupont", pts=[tuple(p) for p in spec["pts"]], group="hr52_tof", bundle="tof", hr52_new=True,
            bought=BOUGHT["tof_dupont_10cm"][0], bought_src=BOUGHT["tof_dupont_10cm"][1], bought_plan=(BOUGHT["splitter_leg"][0] if split else None),
            ends=(f"ToF 模块（v6-VL53L5CX 雷达）{nm} 针上的杜邦母壳（壳尾" + (f" ({t[0]:.2f}, {t[1]:.2f}, {t[2]:.2f})" if t else "") + "，线沿 −x 出，最后 2 mm 沿 +x 直进壳）",
                  f"Radxa pin {n}（{PINMAP[n][0]}）" + ("一分二单母头直插、本腿（ToF）" if split else "直插杜邦壳顶") + f"出线即弯（弯段在 zz_gpio_dupont）→ 接续点 ({pe[0]:.2f}, {pe[1]:.2f}, {pe[2]:.2f})，hr52"),
            note=("hr52 新线（HB12）：ToF → 排针；x 33..61.6 与另 3 根扎成 tof 束（2×2），排针端细网格路由、ToF 端同步扇出"
                  + ("；物理上 = 一分二公头腿 + 这根母对母 10 cm，中途公母对插接头与余长盘线未建模" if split else ""))))
    for nm, b in (D.get("bundles") or {}).items():
        for m in b["members"]: HR52_BUNDLE_OF[m] = nm


_define()
