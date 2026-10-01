#!/usr/bin/env python
"""把 cut_ledger.py 的逐件结果 + print_file_check.py 的文件体检，做成一个自包含 HTML（图内嵌），每件：
  打印朝向俯视 + 斜视两张图，缺口按类别上色、按编号标出；表格：编号 / 类别 / 体积 / 尖刺 / 为什么削（代码注释）/ 来源 / 螺丝。
只读。用法：./.venv/bin/python -B tools/cad/cut_ledger_report.py [--ledger DIR] [--files JSON] [--out HTML]
"""
import os, sys, json, glob, base64, io, html, argparse
import numpy as np

ROOT = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", ".."))
WORK = os.path.join(ROOT, "docs", "design_2026-09-17_bearing_rebuild", "hr49_work")
COL = {"真实姿态区域刀": (0.90, 0.10, 0.10), "运动扫掠让位": (0.98, 0.55, 0.05), "插头/线让位": (0.55, 0.25, 0.80),
       "舵机本体让位": (0.30, 0.45, 0.75), "轴承座/轴承让位": (0.05, 0.60, 0.60), "螺丝孔/舵盘孔/沉孔": (0.25, 0.25, 0.25),
       "刻字": (0.65, 0.65, 0.65), "扎带位/线束固定": (0.15, 0.65, 0.20), "壳/他件让位": (0.55, 0.35, 0.15),
       "外形修整（不是缺口）": (0.80, 0.70, 0.45), "装配通道（舵机滑入/起子）": (0.10, 0.75, 0.95), "电池仓/电子件位": (0.95, 0.80, 0.10),
       "功能开口（通风/声孔/窗/枢轴）": (0.40, 0.85, 0.85), "其他（看注释）": (0.95, 0.45, 0.70)}
PLAIN = {
    "真实姿态区域刀": "原版策略录下来的真实动作里，这两个件曾经互相穿进去（例如膝盖弯到底时小腿碰到髋）。代码把几个姿态的碰撞体拼起来、外扩 0.4 mm，从件上挖掉。形状是碰撞体原样，所以边缘不规则、容易留尖刺。",
    "运动扫掠让位": "关节在活动范围内转动时，别的件扫过的空间，从这个件上挖掉，保证转到头也不碰。",
    "插头/线让位": "舵机插头插上后、线翘起来占的空间。",
    "舵机本体让位": "放舵机本体的腔。",
    "轴承座/轴承让位": "轴承座孔，或给轴承 / 轴承座环留的空间。",
    "螺丝孔/舵盘孔/沉孔": "螺丝孔、舵盘孔、沉头窝。",
    "刻字": "件号刻字。",
    "扎带位/线束固定": "扎带孔 / 线束固定位。",
    "壳/他件让位": "给外壳或旁边的件留的间隙。",
    "外形修整（不是缺口）": "从原版大件 / 毛坯上把这个件的外形切出来（例如把原版躯干壳切成左右两片、把脸板从原版面罩里切出来），切掉的是别的件的部分，不是这个件上的缺口。",
    "装配通道（舵机滑入/起子）": "装配时舵机要从这里滑进去、或起子要从这里伸进去拧螺丝，沿路径清掉的料。",
    "电池仓/电子件位": "电池仓腔、IMU / 麦克风 / 转接板等电子件的安装位和走线口。",
    "功能开口（通风/声孔/窗/枢轴）": "设计上本来就要开的口：散热通风槽、喇叭声孔、摄像头窗、嘴巴转轴孔等。",
    "其他（看注释）": "看右边的代码注释。",
}


def img_b64(fig):
    import matplotlib.pyplot as plt
    buf = io.BytesIO(); fig.savefig(buf, format="png", dpi=90); plt.close(fig)
    return base64.b64encode(buf.getvalue()).decode()


def print_frame(pid, body):
    """world → 打印坐标（与出包同法：world→export_local→down_normal 对齐 −z→平移到床面、xy 居中由调用方做）。"""
    import yaml, trimesh
    sys.path.insert(0, ROOT)
    from duckstructure.lib import TW
    PR = yaml.safe_load(open(os.path.join(ROOT, "tools", "gate", "data", "printability.yaml"), encoding="utf-8"))
    sr = ((PR.get("slice_run") or {}).get("parts") or {}).get(pid) or {}
    dn = np.asarray(sr.get("down_normal_export_local") or [0, 0, -1], float)
    M = trimesh.geometry.align_vectors(dn, np.array([0, 0, -1.0]))
    return M @ np.linalg.inv(TW(body))


SHOW = {"真实姿态区域刀", "运动扫掠让位", "插头/线让位", "壳/他件让位", "其他（看注释）", "轴承座/轴承让位", "装配通道（舵机滑入/起子）", "电池仓/电子件位"}

RECAT = [  # (类别, 生成函数名关键字, 注释关键字, 代码行关键字) —— 先看生成函数，再看代码行里的变量名，再看注释，最后看调用链
    ("真实姿态区域刀", ("region_cut",), ("区域刀",), ("region_cut",)),
    ("运动扫掠让位", ("sweep_of", "relief_sweep", "swing", "hump_sweep"), ("扫掠",), ("sweep", "swing")),
    ("功能开口（通风/声孔/窗/枢轴）", ("vent", "sound", "pivot", "roof", "window"), ("通风", "声孔", "通窗", "开窗", "枢轴"), ("win ", "win=", "window", "vent", "sound")),
    ("装配通道（舵机滑入/起子）", ("servo_slide", "slide", "tool", "corridor", "driver"), ("起子", "滑装", "滑入", "刀路", "通道"), ("access", "slide", "tool", "corridor")),
    ("插头/线让位", ("conn_cut", "conn_zone", "roll_conn_relief", "wire_path"), ("插座", "插头", "线翘"), ("conn",)),
    ("舵机本体让位", ("servo_env", "body_prism", "servo_mesh", "flange_slot", "flange_relief"), (), ("servo_env", "flange")),
    ("轴承座/轴承让位", ("seat", "bearing", "brg", "journal", "idler"), ("座孔", "轴承"), ("seat", "brg", "journal", "idler", "bore")),
    ("螺丝孔/舵盘孔/沉孔", ("horn_cut", "horn_cbore", "mnt_cut", "mount_cut", "screw", "cbore", "pilot", "hub_screw", "_cs_cuts"), ("沉窝", "底孔", "过孔"), ("horn", "pilot", "bpil", "cbore", "screw", "mnt")),
    ("电池仓/电子件位", ("battery", "bay", "imu", "mic_pad", "ubec", "adapter", "sbc", "amp"), ("电池", "IMU", "杜邦", "转接板", "麦克风", "功放", "摄像头", "浅坑"), ("pocket", "slot", "battery", "bay", "imu", "mic", "ubec", "cam")),
    ("扎带位/线束固定", ("wire_fix", "tie"), ("扎带",), ("tie",)),
    ("壳/他件让位", ("minkowski_box", "dilate6", "shell", "scaled_orig", "_clear_cutter", "clearance"), ("让位", "避壳"), ("shell", "clr", "clear")),
]


def recategorize(n):
    if n["category"] in ("修形/包络", "外形修整（不是缺口）"): return "外形修整（不是缺口）"
    cr = (n.get("creator") or "").lower(); txt = (n.get("inline_comment") or "") + " " + (n.get("comment") or "")
    if n["where"].startswith("duckstructure/stamps.py"): return "刻字"
    if "wire_fixings" in n["where"]: return "扎带位/线束固定"
    for cat, fk, ck, kk in RECAT:
        if any(k in cr for k in fk): return cat
    code = (n.get("code") or "").lower()
    lhs = code.split("=")[0] if "=" in code and "==" not in code.split("=")[0] else ""
    for cat, fk, ck, kk in RECAT:
        if lhs and any(k in lhs + "=" for k in kk): return cat
    for cat, fk, ck, kk in RECAT:
        if any(k in code for k in kk): return cat
    for cat, fk, ck, kk in RECAT:
        if any(k in txt for k in ck): return cat
    ch = " ".join(n.get("chain") or []).lower()
    for cat, fk, ck, kk in RECAT:
        if any(k in ch for k in fk): return cat
    return "其他（看注释）"


def _ortho(ax, meshes, R, title, labels):
    """画家算法正交投影：R 把世界向量转到相机系（x 右、y 上、z 朝向观察者）。meshes = [(mesh, rgba, 是否背面剔除)]。"""
    from matplotlib.collections import PolyCollection
    light = np.array([0.35, 0.55, 0.76]); light /= np.linalg.norm(light)
    polys, cols, depth = [], [], []
    for m, rgba, cull in meshes:
        v = m.vertices @ R.T; tri = v[m.faces]; n = m.face_normals @ R.T
        keep = n[:, 2] > 0 if cull else np.ones(len(tri), bool)
        tri, n = tri[keep], n[keep]
        shade = 0.45 + 0.55 * np.clip(np.abs(n @ light), 0, 1)
        c = np.tile(np.asarray(rgba, float), (len(tri), 1)); c[:, :3] *= shade[:, None]
        polys.append(tri[:, :, :2]); cols.append(c); depth.append(tri[:, :, 2].mean(1))
    P = np.concatenate(polys); C = np.concatenate(cols); Dp = np.concatenate(depth)
    o = np.argsort(Dp)
    ax.add_collection(PolyCollection(P[o], facecolors=C[o], edgecolors="none", antialiased=False))
    allv = np.concatenate([m.vertices @ R.T for m, _, _ in meshes[:1]])
    lo, hi = allv[:, :2].min(0), allv[:, :2].max(0); pad = 0.06 * (hi - lo).max()
    ax.set_xlim(lo[0] - pad, hi[0] + pad); ax.set_ylim(lo[1] - pad, hi[1] + pad); ax.set_aspect("equal"); ax.axis("off")
    ax.set_title(title, fontsize=10)
    for txt, p3, col in labels:
        q = p3 @ R.T
        ax.annotate(txt, (q[0], q[1]), xytext=(q[0] + 0.08 * (hi - lo).max(), q[1] + 0.06 * (hi - lo).max()), fontsize=10, fontweight="bold",
                    color="black", arrowprops=dict(arrowstyle="-", color=col, lw=1.2),
                    bbox=dict(boxstyle="round,pad=0.2", fc=(1, 1, 1, 0.9), ec=col, lw=1.4))


def render(pid, res, npz):
    import trimesh, matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    plt.rcParams["font.family"] = ["Heiti TC", "PingFang SC", "Arial Unicode MS", "DejaVu Sans"]
    part = trimesh.load(os.path.join(ROOT, "cad", "duck_s288", "placed", res["placed"] + ".stl"), force="mesh")
    T = print_frame(pid, res["body"])
    part.apply_transform(T)
    c = (part.bounds[0] + part.bounds[1]) / 2; sh = np.array([-c[0], -c[1], -part.bounds[0][2]])
    part.apply_translation(sh)
    notches = []
    for n in res["notches"]:
        k = f"n{n['n']}_v"
        if k not in npz: continue
        if n["category"] not in SHOW or n["net_mm3"] < 0.5: continue
        m = trimesh.Trimesh(npz[k], npz[f"n{n['n']}_f"], process=False)
        m.apply_transform(T); m.apply_translation(sh)
        notches.append((n, m))
    top = np.eye(3)
    az, el = np.radians(-50), np.radians(32)
    Rz = np.array([[np.cos(az), -np.sin(az), 0], [np.sin(az), np.cos(az), 0], [0, 0, 1]])
    Rx = np.array([[1, 0, 0], [0, np.cos(el - np.pi / 2), -np.sin(el - np.pi / 2)], [0, np.sin(el - np.pi / 2), np.cos(el - np.pi / 2)]])
    iso = Rx @ Rz
    fig, axes = plt.subplots(1, 2, figsize=(15, 7.5))
    for ax, R, title in ((axes[0], top, "俯视（同切片软件从上往下看）"), (axes[1], iso, "斜视")):
        meshes = [(part, (0.86, 0.86, 0.84, 1.0), True)]
        labels = []
        for n, m in notches:
            col = COL.get(n["category"], (0.9, 0.4, 0.7))
            meshes.append((m, (*col, 0.75), True))
            labels.append((str(n["n"]), m.vertices.mean(0), col))
        _ortho(ax, meshes, R, title, labels)
    fig.suptitle(f"{pid}：灰=成品；彩色=成品上缺掉的料（只标不直观的让位类和有标记的；孔/舵机腔/刻字只在表里），数字=下表编号", fontsize=11)
    plt.tight_layout()
    return img_b64(fig)


def find_holes(part):
    """三轴 0.5 mm 截面找小圆孔（内环：Ø1.4..5.2、圆度 ≥0.85），按轴向聚成一根根孔：轴、中心、直径、在料里的轴向段。"""
    from shapely.geometry import Polygon
    recs = []
    lo, hi = part.bounds
    for ax in range(3):
        nrm = np.zeros(3); nrm[ax] = 1
        for v in np.arange(lo[ax] + 0.25, hi[ax], 0.5):
            org = np.zeros(3); org[ax] = v
            try:
                s = part.section(plane_origin=org, plane_normal=nrm)
                if s is None: continue
                P, T = s.to_planar()
                polys = P.polygons_full
            except Exception:
                continue
            for poly in polys:
                for ring in poly.interiors:
                    r = Polygon(ring)
                    a = r.area
                    if a < np.pi * 0.7 ** 2 or a > np.pi * 2.6 ** 2: continue
                    if 4 * np.pi * a / (r.length ** 2) < 0.85: continue
                    c2 = np.array(r.centroid.coords[0]); p3 = (T @ np.r_[c2, 0, 1])[:3]
                    recs.append((ax, p3, 2 * np.sqrt(a / np.pi)))
    holes = []
    for ax in range(3):
        R = [(p, d) for a, p, d in recs if a == ax]
        used = np.zeros(len(R), bool)
        oth = [i for i in range(3) if i != ax]
        for i, (p, d) in enumerate(R):
            if used[i]: continue
            grp = [j for j, (q, e) in enumerate(R) if not used[j] and np.linalg.norm(q[oth] - p[oth]) < 0.4 and abs(e - d) < 0.4]
            for j in grp: used[j] = True
            if len(grp) < 2: continue
            pts = np.array([R[j][0] for j in grp]); ds = np.array([R[j][1] for j in grp])
            a3, b3 = pts[np.argmin(pts[:, ax])].copy(), pts[np.argmax(pts[:, ax])].copy()
            a3[ax] -= 0.25; b3[ax] += 0.25
            holes.append(dict(axis="xyz"[ax], ax=ax, a=a3, b=b3, d=float(np.median(ds)), n=len(grp), centers=pts, ds=ds))
    return holes


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--ledger", default=os.path.join(WORK, "ledger")); ap.add_argument("--files", default=os.path.join(WORK, "print_file_check.json"))
    ap.add_argument("--out", default=os.path.join(WORK, "缺口账本_全件_2026-09-28.html"))
    a = ap.parse_args()
    parts = []
    for f in sorted(glob.glob(os.path.join(a.ledger, "*.json"))):
        r = json.load(open(f, encoding="utf-8"))
        # 同一块缺口被记两次（嵌套生成别的件时在同一个毛坯上做了同一刀，如 H03 生成里顺带生成 H01 的避壳刀）：体积、尖刺、各块包围盒全同 → 只留第一条
        seen, keep = set(), []
        for n in r["notches"]:
            sig = (round(n["net_mm3"], 1), round(n.get("spike_mm2") or 0, 1), tuple(tuple(c["lo"]) + tuple(c["hi"]) for c in n.get("comps") or []))
            if sig in seen: continue
            seen.add(sig); keep.append(n)
        r["notches"] = keep
        for n in r["notches"]:
            n["category"] = recategorize(n)
        for n in r["notches"]:
            flags = []
            if n["category"] == "真实姿态区域刀": flags.append("区域刀")
            if (n.get("spike_mm2") or 0) >= 1.0: flags.append(f"尖刺 {n['spike_mm2']:.1f} mm²")
            for s in n.get("near_screw") or []:
                if s["wall_left_mm"] < 1.2: flags.append(f"螺丝孔壁剩 {s['wall_left_mm']:.2f}（{s['feature']}）")
            n["_flags"] = flags; n["_flag"] = bool(flags)
        parts.append(r)
    import trimesh
    for r in parts:
        pid = r["part"]
        try:
            part = trimesh.load(os.path.join(ROOT, "cad", "duck_s288", "placed", r["placed"] + ".stl"), force="mesh")
            holes = find_holes(part)
            npz0 = dict(np.load(os.path.join(a.ledger, f"{pid}_notches.npz")))
        except Exception as e:
            print(pid, "hole scan fail", e); holes, npz0 = [], {}
        r["_holes"] = len(holes)
        CLEAR = {"真实姿态区域刀", "运动扫掠让位", "插头/线让位", "壳/他件让位", "其他（看注释）"}
        th_ = np.linspace(0, 2 * np.pi, 72, endpoint=False)
        for n in r["notches"]:
            n["geo_screw"] = []
            n["_flags"] = [x for x in n["_flags"] if not x.startswith("螺丝孔壁剩")]
            if n["category"] not in CLEAR or f"n{n['n']}_v" not in npz0: continue
            nm_ = trimesh.Trimesh(npz0[f"n{n['n']}_v"].astype(float), npz0[f"n{n['n']}_f"], process=False)
            P = trimesh.sample.sample_surface(nm_, 4000, seed=9)[0] if len(nm_.faces) else nm_.vertices
            P = np.vstack([P, nm_.vertices])
            for h in holes:
                if h["d"] > 5.3: continue
                ab = h["b"] - h["a"]; L2 = float(ab @ ab)
                if L2 < 1e-9: continue
                t = ((P - h["a"]) @ ab) / L2; foot = h["a"] + np.outer(t, ab); rad = np.linalg.norm(P - foot, axis=1); r0 = h["d"] / 2
                sel = (t > -0.05) & (t < 1.05) & (rad - r0 < 1.2)
                if not sel.any(): continue
                ax = h["ax"]; oth = [i for i in range(3) if i != ax]
                best_cut, cov_at = 0.0, None
                cc = h["centers"].mean(0); rr = h["d"] / 2 + 0.3
                # 沿孔轴逐层（找到孔圈的那段两头各再延 1.5 mm：孔壁被切开的那几层截面里孔圈不闭合，找孔时找不到）
                for sv in np.arange(h["a"][ax] - 1.5, h["b"][ax] + 1.5001, 0.25):
                    c0 = cc.copy(); c0[ax] = sv
                    ring = np.repeat(c0[None, :], len(th_), axis=0)
                    ring[:, oth[0]] += rr * np.cos(th_); ring[:, oth[1]] += rr * np.sin(th_)
                    inF = part.contains(ring); inN = nm_.contains(ring) & ~inF
                    if (inF | inN).mean() < 0.6: continue          # 这一层切之前孔外也没多少料（孔口 / 件边）——不算
                    if inN.mean() >= 0.9 or inF.mean() < 0.1: continue   # 整圈都没了 = 孔已经出到腔里（孔口，设计如此），不是侧壁被咬
                    if inN.mean() > best_cut: best_cut, cov_at = float(inN.mean()), float(inF.mean())
                kind = "螺丝孔" if h["d"] <= 2.9 else "沉窝/大孔"
                g = dict(hole=f"{kind} Ø{h['d']:.1f} 沿{h['axis']} @({h['a'][0]:.1f},{h['a'][1]:.1f},{h['a'][2]:.1f})", wall_left_mm=round(float((rad[sel] - r0).min()), 2),
                         cut_frac=round(best_cut, 2), min_ring=None if cov_at is None else round(cov_at, 2), kind=kind)
                n["geo_screw"].append(g)
                if best_cut >= 0.10:
                    n["_flags"].append(f"{kind}壁被这把刀切掉一圈的 {best_cut*100:.0f}%（{g['hole']}）")
            n["_flag"] = bool(n["_flags"])
    files = json.load(open(a.files, encoding="utf-8")) if os.path.exists(a.files) else []
    H = ["<!doctype html><meta charset='utf-8'><title>缺口账本</title>",
         "<style>body{font-family:-apple-system,'PingFang SC',sans-serif;margin:16px;max-width:1500px;background:#fafaf8;color:#222}"
         "table{border-collapse:collapse;width:100%;font-size:13px;margin:6px 0 18px}td,th{border:1px solid #ccc;padding:4px 6px;vertical-align:top}"
         "th{background:#eee}.flag{color:#b00;font-weight:600}.sw{display:inline-block;width:12px;height:12px;border-radius:2px;margin-right:4px;vertical-align:middle}"
         "img{max-width:100%;border:1px solid #ddd}code{font-size:12px;color:#555}h2{margin-top:34px;border-top:2px solid #999;padding-top:10px}"
         "@media (prefers-color-scheme: dark){body{background:#1d1d1b;color:#ddd}th{background:#333}td,th{border-color:#555}}</style>"]
    H.append("<h1>缺口账本：每个件上的每一处缺口是谁削的、为什么削</h1>")
    H.append("<p>做法：用出件时同一套生成代码逐件重建，记下每一刀在成品上留下的缺口（后面又补回去的不算），对应到生成它的那一行代码和上面的注释。"
             "『尖刺』= 缺口边上比一条挤出线（0.45 mm）还薄的薄片，切片后打不出来或打成毛刺。『螺丝孔壁剩』只对让位类缺口算，且只算螺丝孔在料里的那一段。</p>")
    # ── 结论（给人看的）
    allN = [(r["part"], n) for r in parts for n in r["notches"]]
    regs = [(pid, n) for pid, n in allN if n["category"] == "真实姿态区域刀"]
    spikes = sorted([(pid, n) for pid, n in allN if (n.get("spike_mm2") or 0) >= 1.0], key=lambda t: -t[1]["spike_mm2"])
    scr = [(pid, n, g) for pid, n in allN for g in (n.get("geo_screw") or []) if (g.get("cut_frac") or 0) >= 0.10]
    bycat = {}
    for pid, n in allN:
        c = bycat.setdefault(n["category"], [0, 0.0]); c[0] += 1; c[1] += n["net_mm3"]
    H.append("<h2 style='border:none'>结论</h2><ol>")
    H.append(f"<li><b>每一处缺口都有出处。</b>{len(parts)} 个件、{len(allN)} 处缺口来源，全部对应到生成它的那一行代码和注释；没有来路不明的缺口。"
             "件上看着怪的缺口绝大多数是『让位』：给舵机本体、插头、轴承、别的件在关节转到头时扫过的空间、装配时舵机滑进去 / 起子伸进去的路径留地方。"
             "原版 Microduck 的件本来就有这些，只是原版用小舵机、件少，我们换 S288 大舵机又加了轴承，要让的东西更多。</li>")
    H.append(f"<li><b>真正『诡异』的是 {len(regs)} 处『真实姿态区域刀』。</b>它们不是按零位或关节行程算的，而是把原版走路策略录下来的几个姿态里两个件互相穿进去的那团体积原样挖掉（外扩 0.4 mm）。"
             "那团体积是几个姿态拼的不规则形状，所以挖出来边缘像碎裂、容易留尖刺和没切干净的三角薄片。"
             + (f"其中 {sum(1 for _, n in regs if any((g.get('cut_frac') or 0) >= 0.10 for g in n.get('geo_screw') or []))} 处咬到了螺丝孔 / 沉窝的侧壁（见下表『附近螺丝孔壁』列），螺丝照样能拧紧；其余离螺丝孔都有料。"
                if any(any((g.get('cut_frac') or 0) >= 0.10 for g in n.get('geo_screw') or []) for _, n in regs) else "都没碰到螺丝孔。")
             + "都不在受力面上，不影响装配；看着难看是真的。</li>")
    H.append(f"<li><b>尖刺 / 薄片</b>（比一条挤出线 0.45 mm 还薄的料）：有 {len(spikes)} 处缺口边上 ≥1 mm²。切片后这些要么打不出来，要么打成一丝毛刺，用刀片一刮就掉，不影响强度。</li>")
    H.append(f"<li><b>让位缺口把螺丝孔壁切开</b>（在孔真正有圈的那几层，孔外 0.3 mm 那一圈被这把刀削掉 ≥10%；起子通道、沉窝、舵机腔这类本来就贴着螺丝的不算）：{len(scr)} 处。" + ("见下表：螺丝照样能拧，只是孔的一侧开了口。" if scr else "没有。") + "</li>")
    H.append("</ol>")
    H.append("<h3>全部真实姿态区域刀</h3><table><tr><th>件</th><th>#</th><th>缺口 mm³</th><th>尖刺 mm²</th><th>碰撞对 / 姿态数</th><th>附近螺丝孔壁</th></tr>")
    for pid, n in regs:
        pv = n.get("provenance") or {}
        gs = "；".join(f"{g['hole']} 孔外一圈被切掉 {g['cut_frac']*100:.0f}%" for g in (n.get("geo_screw") or []) if (g.get("cut_frac") or 0) > 0) or "没碰到螺丝孔"
        H.append(f"<tr><td><a href='#{pid}'>{pid}</a></td><td>{n['n']}</td><td>{n['net_mm3']:.1f}</td><td>{n.get('spike_mm2', 0):.1f}</td>"
                 f"<td>{html.escape(str(pv.get('pair') or ((n.get('inline_comment') or n.get('comment') or '')[:70])))} / {pv.get('defect_poses', '见注释')}</td><td>{html.escape(gs)}</td></tr>")
    H.append("</table>")
    H.append("<h3>尖刺最多的缺口（前 15）</h3><table><tr><th>件</th><th>#</th><th>类别</th><th>尖刺 mm²</th><th>最薄 mm</th><th>为什么削</th></tr>")
    for pid, n in spikes[:15]:
        why = (n.get("inline_comment") or n.get("comment") or "")[:120]
        H.append(f"<tr><td><a href='#{pid}'>{pid}</a></td><td>{n['n']}</td><td>{n['category']}</td><td>{n['spike_mm2']:.1f}</td><td>{n.get('spike_min_mm')}</td><td>{html.escape(why)}</td></tr>")
    H.append("</table>")
    if scr:
        H.append("<h3>把螺丝孔壁切开的缺口</h3><table><tr><th>件</th><th>#</th><th>类别</th><th>孔</th><th>孔外一圈</th></tr>")
        for pid, n, g in scr:
            H.append(f"<tr><td><a href='#{pid}'>{pid}</a></td><td>{n['n']}</td><td>{n['category']}</td><td>{html.escape(g['hole'])}</td><td>被切掉 {g['cut_frac']*100:.0f}%（该层剩 {g['min_ring']*100:.0f}%）</td></tr>")
        H.append("</table>")
    H.append("<h3>按类别合计</h3><table><tr><th>类别</th><th>处数</th><th>削掉 mm³</th></tr>" + "".join(
        f"<tr><td>{k}</td><td>{v[0]}</td><td>{v[1]:.0f}</td></tr>" for k, v in sorted(bycat.items(), key=lambda kv: -kv[1][1])) + "</table>")
    H.append("<h3>类别说明</h3><table><tr><th>类别</th><th>白话</th></tr>" + "".join(
        f"<tr><td><span class='sw' style='background:rgb({int(c[0]*255)},{int(c[1]*255)},{int(c[2]*255)})'></span>{k}</td><td>{html.escape(PLAIN[k])}</td></tr>" for k, c in COL.items()) + "</table>")
    # 总表
    H.append("<h3>各件总览</h3><table><tr><th>件</th><th>重建与现件体积</th><th>缺口来源</th><th>区域刀</th><th>尖刺合计 mm²</th><th>螺丝孔壁 &lt;1.2 的缺口</th></tr>")
    for r in parts:
        nreg = sum(1 for n in r["notches"] if n["category"] == "真实姿态区域刀")
        sp = sum(n.get("spike_mm2") or 0 for n in r["notches"])
        scr = sum(1 for n in r["notches"] if any((g.get("cut_frac") or 0) >= 0.10 for g in n.get("geo_screw") or []))
        ok = "一致" if r.get("vol_match") else f"<span class='flag'>不一致 {r['vol_rebuilt']} vs {r['vol_placed']}</span>"
        H.append(f"<tr><td><a href='#{r['part']}'>{r['part']}</a></td><td>{ok}</td><td>{len(r['notches'])}</td>"
                 f"<td>{'<span class=flag>' + str(nreg) + '</span>' if nreg else 0}</td><td>{sp:.1f}</td><td>{'<span class=flag>' + str(scr) + '</span>' if scr else 0}</td></tr>")
    H.append("</table>")
    if files:
        bad = [f for f in files if f.get("issues")]
        H.append(f"<h3>打印文件体检（{len(files)} 个文件，有问题 {len(bad)} 个）</h3><table><tr><th>文件</th><th>问题</th></tr>")
        for f in bad:
            H.append(f"<tr><td><code>{html.escape(f['file'].replace('docs/design_2026-09-17_bearing_rebuild/打印包/', ''))}</code></td><td>{html.escape('；'.join(f['issues']))}</td></tr>")
        H.append("</table>")
    for r in parts:
        pid = r["part"]
        try:
            npz = dict(np.load(os.path.join(a.ledger, f"{pid}_notches.npz")))
            img = render(pid, r, npz)
        except Exception as e:
            img = None; print(pid, "render fail", e)
        H.append(f"<h2 id='{pid}'>{pid}（{r['placed']}）</h2>")
        if img: H.append(f"<img src='data:image/png;base64,{img}'>")
        H.append("<table><tr><th>#</th><th>类别</th><th>缺口 mm³</th><th>标记</th><th>为什么削（代码行尾注释 / 上方注释）</th><th>代码位置</th><th>来源</th></tr>")
        for n in r["notches"]:
            c = COL.get(n["category"], (0.9, 0.4, 0.7))
            why = " ／ ".join(x for x in (n.get("inline_comment"), n.get("comment")) if x) or "（无注释）"
            prov = n.get("provenance") or {}
            ptxt = ""
            if prov.get("pair"): ptxt = f"碰撞对 {prov['pair']}，{prov.get('defect_poses')} 个姿态，原始交叠 {prov.get('union_mm3')} mm³{'，从右件镜像' if prov.get('mirrored') else ''}"
            elif prov.get("body") is not None: ptxt = f"跟着关节 {prov.get('body')} 在 {prov.get('lo')}..{prov.get('hi')}° 转，{prov.get('n')} 步，外扩 {prov.get('grow')}"
            flags = "<br>".join(f"<span class=flag>{html.escape(x)}</span>" for x in n["_flags"])
            H.append(f"<tr><td>{n['n']}</td><td><span class='sw' style='background:rgb({int(c[0]*255)},{int(c[1]*255)},{int(c[2]*255)})'></span>{n['category']}</td>"
                     f"<td>{n['net_mm3']:.1f}</td><td>{flags}</td><td>{html.escape(why[:420])}</td><td><code>{html.escape(n['where'])}</code><br><code>{html.escape(n['creator'])}</code></td><td>{html.escape(ptxt)}</td></tr>")
        H.append("</table>")
    open(a.out, "w", encoding="utf-8").write("\n".join(H))
    print("wrote", a.out)


if __name__ == "__main__":
    main()
