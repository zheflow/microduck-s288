#!/usr/bin/env python
"""hr50（2026-09-28）局部精修 finish_pass：只在 clean_check 报出来的问题附近的小盒子里动刀，把剩下的零散问题修掉。

修什么（输入 = 世界系水密 STL + 它的 clean_check JSON，orig 口径；只处理 flag 为真的条目）
  thin/membrane 薄膜、thin/blade 刀片 —— 开运算（R_OPEN=0.25）：比 0.5 薄的料整片削穿 / 削钝，切口是 r0.25–0.3 圆角，不留新皮
  frag/spike 尖刺、frag/ragged 碎边     —— 开运算削掉细尖 + 闭运算（R_CLOSE=0.15）填平 0.3 以下的缝 / 台阶 + 再开（R_OPEN2=0.15）倒掉闭运算留下的小凸棱
  frag/debris 小碎体                     —— 直接删（不连通、< 1 mm³）
不修：holes 类（螺丝孔 / 沉窝侧壁开口、开口环）—— 要在源码里改刀，列进报告交人工。

方法
  1 保护体（螺丝孔 / 底孔 / 舵盘孔 / 中心孔 / 沉窝 / 铣平面 / 轴承座 / 挡肩 / 轴颈 / 从动盘 / 凸台 / 键）：
      a) 登记：tools/gate/data/features.yaml 本件，阵列逐孔展开，export_local → 世界 = duckstructure.lib.TW(body)（与 clean_check 用的
         algo/holes_work/hole_features_world_expanded.json 同法）；孔的轴向范围用几何实测的孔壁段（登记的是刀长，常穿过整件）；
         凸出件（从动盘 / 轴颈 / 凸台）只护侧面一圈 + 两个端面，网格上找不到（覆盖率 < 0.2，登记过时）的不护、写进 notes；
      b) 几何：clean_check_holes.detect_cylinders（与 clean_check holes 同一个圆柱识别）+ match_use 认用途；
         channel（别件螺丝同轴让位）和"起子通道"按用户规矩 4 不护，其余（own / none / n/a）都护；
      c) --protect extra.json 手工追加（{a,b,d} 圆柱 / {lo,hi} 盒）。
    保护体 = 孔 / 凸台 外扩 PROTECT_PAD=1.0（孔壁外 ≥1.0 完整）、两端各伸 1.0；Ø > 6.5 的大孔只护孔壁一圈 [r−1, r+1]。
  2 判定：finding 采样点离保护体 < FIND_PAD(0.3) → 拒修（问题就在保护区上，交人工）。
  3 盒：finding 包围盒外扩 BOX_MARGIN=1.5（每个面在 0.8..2.5 里挑"干净"位置：少穿锐棱、不贴锐棱、过渡带里没有三棱角点、
      两个盒面上的台阶点不在 1.5 mm 内配对 —— clean_check 把 1.5 mm 内 ≥6 条短锐棱且不共面的算碎边）；
      盒碰到保护体 / 拒修 finding 的禁区 → 把某个面往里收（最少收到离 finding 0.8），收不开 → 拒修。盒里不留任何保护体
      （09-28 试过"盒 − 保护体"挖空：圆柱面上的接缝实测出尖刺 / 碎边 / 碎体，放弃）→ 保护区内逐三角形不变。重叠的盒合并后重摆。
      --strict-protect = 任务书原文：外扩 1.5 的盒碰到保护区就拒修，不收盒。
  4 盒内形态学（网格 H=0.05，网格比盒大出形态学作用距离 ≈0.9 mm，盒内结果与无限大网格一致）：
      d = 原件精确有符号距离（窄带逐点到三角形精确距离；符号 = 沿 x 扫描线奇偶，网格加无理偏移避开退化）；
      开 = 内切球并集 O = ∪{B(c, min(−d, CAP)) : −d(c) ≥ r(c)}，d_O(x) = min_c(|x−c| − ρ(c))（O 外是精确距离，闭运算前不用重算）；
      闭 = 对补集做同样的事；再开同理。搜索半径按"下一步要用到的精确范围"定（morph_radii）。
      开运算半径在盒面附近 RAMP=0.5 内降到 RFLOOR×R_OPEN=0.15（不降到 0：比 3H 更细的圆角 MT 画成锯齿 → 碎边）；
      场差 ≤ 0.003 处贴回原 SDF（平面上补丁与原件逐点重合，接缝零台阶）。
      等值面 = 自写 marching tetrahedra（Kuhn 6 四面体，全网格一致 → 水密流形；最外层强制为外 → 补丁自带封口），
      再做 4 轮切向 Laplace 松弛 + 牛顿投影回三线性等值面（MT 三角形形状差，相邻面法向跳 30° 以上会被当成锐棱 / 碎边）。
  5 合并：补丁先 simplify(1e-4)（只动补丁）；新件 = (原件 − 盒) ∪ (补丁 ∩ 盒)，manifold3d 布尔（Mesh64，与 assembly_audit.solid 同样转换）；
      全部盒做完后按 float32 焊合顶点、删退化面、重建 manifold（lib.clean_print_topology 的焊合法；不做整件 simplify ——
      它会重剖盒外三角形，clean_check 的碎边统计随之变，09-28 N02 实测盒外冒出新"碎边"）。盒外 / 保护区内逐三角形不变。
  6 盒级回退：布尔失败 / 修完不是单体（分出 ≥1 mm³ 的块）/ 累计体积变化 > 0.5% / 盒内没改动 → 该盒原样保留、finding 记 failed。
  7 自检（CLI 默认开，最多 PASSES=3 轮）：新件用 clean_check 单文件子进程模式（--child … --placed，thin,frag,holes，orig）重跑；
      对账：原 finding 消失 / 残留、盒内 / 盒外新增、孔类新增；盒内新增的（多是盒面接缝的平面台阶和别的面凑成一堆）下一轮再修，
      下一轮的盒把碰到的上一轮盒整个包进去（旧接缝进核心被抹平）；
      与原件布尔差：削 / 补体积、盒外差、保护体内差（应为 0）。

接口
  CLI：./.venv/bin/python -B tools/cad/finish_pass.py in.stl --check in_clean.json --out out.stl [--report r.json] [--protect extra.json]
       [--label L01] [--placed yaw2roll] [--strict-protect] [--unprotect geo#19,...] [--no-selfcheck] [--png cmp.png] [--param K=V ...]
       --check 可以是 clean_check 整份输出（files[]，按 --label / 文件名挑）或 --child 单文件输出。要用孔检测对上用途，--check 最好用
       `clean_check.py --child <stl> --label <件号> --placed <placed名> --tmp X.json --checks thin,frag,holes --profile orig` 跑出来的。
       输出：out.stl、out_clean.json（最后一轮自检）、report JSON、对比图 PNG。
  批处理（完整 chain 用）：./.venv/bin/python -B tools/cad/finish_pass.py --apply-cad L01,T02,... [--cad-root DIR] [--dry-run]
       每件：读 <root>/placed/<placed>.stl（世界系）→ clean_check 子进程 → 精修 + 自检 → 验收（无新增 thin/frag/孔类、保护区 / 盒外表面采样最大偏差 ≤1e-4 mm、
       水密单体、至少修掉一条）→ 写回 placed/<placed>.stl、placed/<placed>_R.stl（有右件时 = mirror_y，与 build 同法；右件没有单独导出文件）
       和 <root>/<件号>_*.stl（= inv(TW(body))·世界，与 checks.export 同法），原文件先备份到 <root>/_finish_work/backup/；
       验收不过、导出系对不上（导出 ≠ inv(TW)·placed）、右件 ≠ mirror_y(左件) → 该件原样不动。汇总：<root>/_finish_work/apply_summary.json。
       注意：写回后 cad/duck_s288/cache 清单里的导出 sha 对不上，build_fast 下次会当过期重切 —— 精修要放在 build 之后跑。
  Python：finish(mesh, findings, protect, params=None) -> (new_mesh, report)
       mesh = trimesh.Trimesh（世界系水密）；findings = load_findings(pick_entry(json, label))；protect = build_protect(mesh, placed, extra)[0]（或 None）。
       finish 本身不跑自检（本进程有 manifold3d，不能同进程开 Embree）；自检用 run_clean_check()/compare()/diff_audit()。

限制（什么时候不修 / 修不了）
  - holes 类 finding 一律不修，报告里列出。
  - finding 本身在保护区里 / 贴着保护区（< 0.3），或盒收到离 finding 0.8 还躲不开保护体 → 拒修（常见：沉窝口的薄膜、螺丝孔边的刀片）。
  - 保护体是按"孔 / 凸台 外扩 1.0"画的：未登记、几何也认不出来的配合面护不到；认出来但用途 none 的孔（可能是残孔）也当保护体 → 宁可拒修。
  - 盒体素 > MAX_VOXELS（4.5e6 ≈ 560 mm³ 的网格，盒本身约 6.4 mm 见方或等体积的长条）→ 拒修：大片问题是刀本身的事，回源码改。
  - 只会"削薄料 + 填小缝 + 倒圆"：不会补厚（薄墙要 ≥0.8 得回源码收刀），不会重挖圆孔；盒内所有锐棱都倒 r≈0.25–0.3（凸）/ 0.15–0.2（凹）。
  - 盒面上留"圆角 vs 原件锐棱"的平面小台阶（≤0.1，clean_check 记 micro_seam / planar_detail）；两个盒面的台阶凑在一起偶尔被报碎边 → 下一轮修。
  - 输出三角形数增加（每盒约 1–3 万面）。
"""
import os, sys, json, time, argparse, resource, subprocess, hashlib

import numpy as np

ROOT = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", ".."))
HERE = os.path.dirname(os.path.abspath(__file__))

PARAMS = dict(
    H=0.05,             # SDF 网格间距 mm
    R_OPEN=0.25,        # 开运算半径：去掉薄于 0.5 的料
    R_CLOSE=0.15,       # 闭运算半径：填掉 0.3 以下的缝 / 台阶
    R_OPEN2=0.15,       # 闭运算之后再开一次的半径（0 = 不做）：倒掉闭运算留下的小凸棱 / 锥点
    CFLOOR=1.0,         # 闭运算 / 再开半径在盒面上的比例（1 = 不衰减）
    CAP_EXTRA=0.05,     # 内切球半径上限 = R + CAP_EXTRA（球比 R 大一点等值面更平；搜索半径见 morph_radii）
    RAMP=0.5,           # 开运算半径从 D 边界往里 RAMP 内由 RFLOOR·R_OPEN 升到 R_OPEN
    RFLOOR=0.6,         # 盒面上的开运算半径 = RFLOOR × R_OPEN = 0.15：补丁里没有比 3H 更细的圆角 / 楔尖（MT 画不出来，降到 0 时过渡带出碎边）；
                        #   盒面上圆角与原件锐棱之间留一个落在盒面平面里的小台阶（clean_check 记 micro_seam / planar_detail，不报）
    DOMAIN="box",       # 修改区：box = 盒 − 保护体；blob = 改动体素外扩 DOMAIN_PAD ∩ 盒 − 保护体（试验，接缝退化三角多，不用）
    DOMAIN_PAD=0.15,
    SNAP_TOL=0.006,     # 补丁场与原 SDF 差 ≤ 0.003 处完全贴回原件、0.003..0.006 过渡（平面上内切球并集的波纹 ≤ 0.003 → 接缝零台阶）
    BOX_MARGIN=1.5,     # finding 包围盒外扩（目标）
    BOX_MARGIN_MIN=0.8, # 躲保护体时盒面最多收到离 finding 0.8
    BOX_MARGIN_MAX=2.5, # 挑"干净"盒面位置的范围上限
    ZONE_GAP=0.4,       # 盒面离保护体至少 0.4：盒面接缝万一出碎边，下一轮还能修（离保护体 < FIND_PAD 的会被拒）
    BOX_JOIN=0.0,       # 盒间距 < 此值就合并（0 = 只合并重叠的）
    FIND_PAD=0.3,       # finding 点外扩多少算"碰到保护区"
    PROTECT_PAD=1.0,    # 保护体 = 孔 / 凸台外扩 1.0（孔壁外 ≥1.0 完整 —— 用户规矩）
    PROTECT_AX_PAD=1.0, # 保护体沿轴两端各伸
    BORE_SOLID_D=6.5,   # Ø ≤ 6.5 的孔整根圆柱都护（螺丝孔 / 沉窝），更大的只护孔壁一圈
    MAX_VOXELS=4.5e6,   # 单盒网格上限（含形态学外扩；09-28 实测峰值约 0.6 KB/体素：5.0e6 体素 → 3.0 GB，超单进程 3 GB 规矩）
    PATCH_SIMPLIFY=1e-4,  # 补丁（松弛后的 MT 等值面）先 manifold.simplify 的容差 mm：平面上的碎三角合并，省内存；只动补丁、不碰盒外
                          #   （试过 2e-3：过渡带里的小圆角被压成折面，锐棱变多；1e-4 在 r0.15 圆角上面片夹角 < 5°）
    SEAM_SIMPLIFY=0.0,    # 每盒合并后整件 manifold.simplify 的容差 mm（0 = 不做；接缝零面积碎三角交给最后的 float32 焊合）。
                          #   整件 simplify 会重剖盒外的共面三角形 —— 形状不变，但 clean_check 的碎边统计变（N02 盒外冒出新"碎边"）
    WELD_F32=True,        # 输出前按 float32 焊合顶点并重建 manifold（lib.clean_print_topology 同法）
    WELD_SIMPLIFY=0.0,    # 焊合后整件 simplify 容差（0 = 不做：simplify 会重剖盒外三角形，改变 clean_check 的碎边统计）
    RELAX_ITERS=4,        # 补丁顶点切向松弛 + 投影回等值面的轮数（MT 三角形形状差，相邻法向跳 30° 以上 → 碎边误报）
    RELAX_LAMBDA=0.5,
    STRICT_PROTECT=False, # True = 按任务书原文：盒碰到保护区就整盒拒修
    VOL_TOL=0.005,      # 体积变化上限（相对）
    PASSES=1,           # CLI 自检轮数上限：第 2 轮起只修上一轮在盒里新冒出来的 finding（09-28 实测第 2、3 轮从没让 finding 变少，缺省 1）
    ROLLBACK_ATTEMPTS=3,  # 回退后从原件重来的次数上限（含第一次）
    SEAM_RAGGED_OK=True,  # 09-28 13:2x 主设计定两级：盒里消掉了硬类（薄膜 / 刀片 / 尖刺）→ 盒缝即使被判 ragged 也写回（status applied_with_seams，
                          #   逐条报坐标 + slab）；只修碎边的盒 → 盒缝不许新增 ragged（出了就回退）。新增薄膜 / 刀片 / 尖刺 / 孔类一律不写回
    DEBRIS_MM3=1.0, DEBRIS_FRAC=0.005,
    JITTER=(1.234567e-4, 2.345678e-4, 3.456789e-4),   # 网格无理偏移：避开与 CAD 圆整坐标面 / 棱重合（扫描线奇偶退化）
)


def rss_gb():
    r = resource.getrusage(resource.RUSAGE_SELF).ru_maxrss
    return r / 1e9 if sys.platform == "darwin" else r / 1e6


def sha16(path):
    try:
        return hashlib.sha256(open(path, "rb").read()).hexdigest()[:16]
    except OSError:
        return None


# ═════════════════════════════ 1. SDF：窄带精确距离 + 扫描线奇偶符号 ═════════════════════════════
def _closest_dist(P, A, B, C):
    """点到三角形精确距离（Ericson 7 区域，向量化）。P,A,B,C: (n,3)。"""
    ab = B - A; ac = C - A; ap = P - A
    d1 = np.einsum("ij,ij->i", ab, ap); d2 = np.einsum("ij,ij->i", ac, ap)
    bp = P - B; d3 = np.einsum("ij,ij->i", ab, bp); d4 = np.einsum("ij,ij->i", ac, bp)
    cp = P - C; d5 = np.einsum("ij,ij->i", ab, cp); d6 = np.einsum("ij,ij->i", ac, cp)
    va = d3 * d6 - d5 * d4; vb = d5 * d2 - d1 * d6; vc = d1 * d4 - d3 * d2
    with np.errstate(divide="ignore", invalid="ignore"):
        den = va + vb + vc
        v = np.where(den != 0, vb / den, 0.0); w = np.where(den != 0, vc / den, 0.0)
        Q = A + ab * v[:, None] + ac * w[:, None]                                     # 7 面内
        e1 = d4 - d3; e2 = d5 - d6
        m = (va <= 0) & (e1 >= 0) & (e2 >= 0)                                        # 6 BC 边
        t = np.where(e1 + e2 != 0, e1 / (e1 + e2), 0.0); Q[m] = (B + (C - B) * t[:, None])[m]
        m = (vb <= 0) & (d2 >= 0) & (d6 <= 0)                                        # 5 AC 边
        t = np.where(d2 - d6 != 0, d2 / (d2 - d6), 0.0); Q[m] = (A + ac * t[:, None])[m]
        m = (d6 >= 0) & (d5 <= d6); Q[m] = C[m]                                      # 4 C 点
        m = (vc <= 0) & (d1 >= 0) & (d3 <= 0)                                        # 3 AB 边
        t = np.where(d1 - d3 != 0, d1 / (d1 - d3), 0.0); Q[m] = (A + ab * t[:, None])[m]
        m = (d3 >= 0) & (d4 <= d3); Q[m] = B[m]                                      # 2 B 点
        m = (d1 <= 0) & (d2 <= 0); Q[m] = A[m]                                       # 1 A 点
    return np.linalg.norm(P - Q, axis=1)


def _enum(counts):
    """counts (n,) → (owner, local)：第 i 个元素重复 counts[i] 次，local = 0..counts[i]−1。"""
    counts = np.asarray(counts, np.int64)
    tot = int(counts.sum())
    owner = np.repeat(np.arange(len(counts)), counts)
    local = np.arange(tot, dtype=np.int64) - np.repeat(np.cumsum(counts) - counts, counts)
    return owner, local


def band_udf(tris, g0, h, shape, band, chunk=250_000):
    """网格点到三角形集合的精确无符号距离，只算 ≤ band 的（其余 = +inf）。tris: (T,3,3)。
    枚举：每个三角形按法向主轴 a 取投影面上的列（投影三角形外扩 band 内的列），每列只取平面 ±band/|n_a| 的一段 → 候选 ≈ 面积 × 2band / H³。"""
    shape = np.asarray(shape)
    U = np.full(int(np.prod(shape)), np.inf)
    if len(tris) == 0:
        return U.reshape(shape)
    stride = np.array([shape[1] * shape[2], shape[2], 1])
    top = g0 + (shape - 1) * h
    nrm = np.cross(tris[:, 1] - tris[:, 0], tris[:, 2] - tris[:, 0])
    nl = np.linalg.norm(nrm, axis=1)
    ok = nl > 1e-14
    tris, nrm = tris[ok], nrm[ok] / nl[ok, None]
    dom = np.argmax(np.abs(nrm), axis=1)
    for ax in range(3):
        bb, cc = [i for i in range(3) if i != ax]
        T = tris[dom == ax]; N = nrm[dom == ax]
        if not len(T):
            continue
        plo = T[:, :, [bb, cc]].min(1) - band; phi = T[:, :, [bb, cc]].max(1) + band
        j0 = np.clip(np.ceil((plo[:, 0] - g0[bb]) / h), 0, shape[bb] - 1).astype(np.int64)
        j1 = np.clip(np.floor((phi[:, 0] - g0[bb]) / h), -1, shape[bb] - 1).astype(np.int64)
        k0 = np.clip(np.ceil((plo[:, 1] - g0[cc]) / h), 0, shape[cc] - 1).astype(np.int64)
        k1 = np.clip(np.floor((phi[:, 1] - g0[cc]) / h), -1, shape[cc] - 1).astype(np.int64)
        alo = T[:, :, ax].min(1) - band; ahi = T[:, :, ax].max(1) + band
        live = (j1 >= j0) & (k1 >= k0) & (ahi >= g0[ax]) & (alo <= top[ax])
        idx = np.nonzero(live)[0]
        nj = (j1 - j0 + 1)[idx]; nk = (k1 - k0 + 1)[idx]
        ncol = nj * nk
        # 分块：按列数累计
        cs = np.cumsum(ncol); start = 0
        while start < len(idx):
            stop = start + max(1, int(np.searchsorted(cs - (cs[start - 1] if start else 0), chunk, side="right")))
            sub = idx[start:stop]
            o, q = _enum(ncol[start:stop])
            t = sub[o]
            jj = j0[t] + q // nk[start:stop][o]; kk = k0[t] + q % nk[start:stop][o]
            yb = g0[bb] + jj * h; zc = g0[cc] + kk * h
            # 列剪枝：投影面内到投影三角形的距离 ≤ band
            P2 = np.zeros((len(t), 3)); P2[:, bb] = yb; P2[:, cc] = zc
            A2 = T[t, 0].copy(); B2 = T[t, 1].copy(); C2 = T[t, 2].copy()
            A2[:, ax] = 0; B2[:, ax] = 0; C2[:, ax] = 0
            keep = _closest_dist(P2, A2, B2, C2) <= band
            t, jj, kk, yb, zc = t[keep], jj[keep], kk[keep], yb[keep], zc[keep]
            # 平面在该列的 a 坐标 ± band/|n_a|
            na = N[t, ax]
            a0 = (np.einsum("ij,ij->i", N[t], T[t, 0]) - N[t, bb] * yb - N[t, cc] * zc) / na
            half = band / np.abs(na)
            i0 = np.clip(np.ceil((a0 - half - g0[ax]) / h), 0, shape[ax] - 1).astype(np.int64)
            i1 = np.clip(np.floor((a0 + half - g0[ax]) / h), -1, shape[ax] - 1).astype(np.int64)
            ni = np.maximum(i1 - i0 + 1, 0)
            o2, q2 = _enum(ni)
            for s0 in range(0, len(o2), chunk):
                oo, qq = o2[s0:s0 + chunk], q2[s0:s0 + chunk]
                I = np.empty((len(oo), 3), np.int64)
                I[:, ax] = i0[oo] + qq; I[:, bb] = jj[oo]; I[:, cc] = kk[oo]
                tt = t[oo]
                d = _closest_dist(g0 + I * h, T[tt, 0], T[tt, 1], T[tt, 2])
                k = d <= band
                np.minimum.at(U, (I[k] * stride).sum(1), d[k])
            start = stop
    return U.reshape(shape)


def parity_inside(tris, g0, h, shape):
    """沿 +x 的扫描线奇偶判内外。tris: 整件三角形 (T,3,3)。返回 bool (shape)。
    某条扫描线穿过的次数为奇数（碰到棱 / 顶点的退化）→ 该线在 yz 上挪 1e-5 重算。"""
    shape = np.asarray(shape)
    ny, nz = int(shape[1]), int(shape[2])
    xs = g0[0] + np.arange(shape[0]) * h

    def crossings(yoff, zoff, cols=None):
        Y = tris[:, :, 1] - (g0[1] + yoff); Z = tris[:, :, 2] - (g0[2] + zoff)
        j0 = np.clip(np.ceil(Y.min(1) / h), 0, ny - 1).astype(np.int64); j1 = np.clip(np.floor(Y.max(1) / h), -1, ny - 1).astype(np.int64)
        k0 = np.clip(np.ceil(Z.min(1) / h), 0, nz - 1).astype(np.int64); k1 = np.clip(np.floor(Z.max(1) / h), -1, nz - 1).astype(np.int64)
        nj = np.maximum(j1 - j0 + 1, 0); nk = np.maximum(k1 - k0 + 1, 0)
        ok = (nj > 0) & (nk > 0) & (Y.max(1) >= 0) & (Z.max(1) >= 0)
        idx = np.nonzero(ok)[0]
        cnt = nj[idx] * nk[idx]
        tid = np.repeat(idx, cnt)
        q = np.arange(int(cnt.sum())) - np.repeat(np.cumsum(cnt) - cnt, cnt)
        jj = j0[tid] + q // nk[tid]; kk = k0[tid] + q % nk[tid]
        if cols is not None:
            keep = cols[jj, kk]
            tid, jj, kk = tid[keep], jj[keep], kk[keep]
        py = jj * h; pz = kk * h
        y0, y1, y2 = Y[tid, 0], Y[tid, 1], Y[tid, 2]; z0, z1, z2 = Z[tid, 0], Z[tid, 1], Z[tid, 2]
        det = (y1 - y0) * (z2 - z0) - (y2 - y0) * (z1 - z0)
        with np.errstate(divide="ignore", invalid="ignore"):
            l1 = ((py - y0) * (z2 - z0) - (y2 - y0) * (pz - z0)) / det
            l2 = ((y1 - y0) * (pz - z0) - (py - y0) * (z1 - z0)) / det
        l0 = 1 - l1 - l2
        inside = (det != 0) & (l0 >= 0) & (l1 >= 0) & (l2 >= 0)
        tid, jj, kk, l0, l1, l2 = tid[inside], jj[inside], kk[inside], l0[inside], l1[inside], l2[inside]
        x = l0 * tris[tid, 0, 0] + l1 * tris[tid, 1, 0] + l2 * tris[tid, 2, 0]
        return jj * nz + kk, x

    col, x = crossings(0.0, 0.0)
    ncol = np.bincount(col, minlength=ny * nz)
    odd = (ncol % 2 == 1).reshape(ny, nz)
    n_odd = int(odd.sum())
    if n_odd:
        colb, xb = crossings(1.1e-5, 1.3e-5, cols=odd)
        keep = ~odd.ravel()[col]
        col = np.concatenate([col[keep], colb]); x = np.concatenate([x[keep], xb])
    xmin = min(xs[0], x.min() if len(x) else xs[0]) - 0.5
    xmax = max(xs[-1], x.max() if len(x) else xs[-1]) + 0.5
    sc = 0.5 / (xmax - xmin)                                                       # 列内 x → [0, 0.5)
    key = np.sort(col.astype(np.float64) + (x - xmin) * sc)
    gcol = np.arange(ny * nz, dtype=np.float64)
    first = np.searchsorted(key, gcol)                                              # 每列第一个交点
    ins = np.empty((shape[0], ny * nz), bool)
    for i, xv in enumerate(xs):                                                     # 逐层：该列 x < xv 的交点数
        ins[i] = ((np.searchsorted(key, gcol + (xv - xmin) * sc) - first) % 2) == 1
    ins = ins.reshape(shape[0], ny, nz)
    return ins, n_odd


def grid_sdf(mesh, lo, hi, h, band, jitter=(0, 0, 0), pad=None):
    """盒 [lo,hi] 上的网格（每边外加 pad，缺省 2 格）有符号距离：|d| ≤ band 精确，外面截成 ±band。负 = 在件里。"""
    pad = 2 * h if pad is None else pad
    g0 = np.asarray(lo, float) - pad + np.asarray(jitter)
    shape = np.floor((np.asarray(hi) + pad - g0) / h).astype(int) + 1
    T = mesh.triangles
    lo_b, hi_b = g0 - band - h, g0 + (shape - 1) * h + band + h
    near = np.all((T.max(1) >= lo_b) & (T.min(1) <= hi_b), axis=1)
    U = band_udf(T[near], g0, h, shape, band)
    ins, n_odd = parity_inside(T, g0, h, shape)
    D = np.minimum(U, band)
    D = np.where(ins, -D, D)
    return g0, shape, D, dict(n_odd_columns=n_odd, n_band=int(np.isfinite(U).sum()))


# ═════════════════════════════ 2. 形态学：内切球并集（开）+ 补集内切球并集（闭） ═════════════════════════════
def _ball_offsets(rv):
    k = int(np.ceil(rv))
    g = np.arange(-k, k + 1)
    O = np.stack(np.meshgrid(g, g, g, indexing="ij"), -1).reshape(-1, 3)
    L = np.linalg.norm(O, axis=1)
    keep = L <= rv + 1e-9
    o = np.argsort(L[keep], kind="stable")
    return O[keep][o], L[keep][o]


def ball_union_field(rho, h, R, floor):
    """F(x) = min(floor(x), min_{|o|≤R} (|o| − ρ(x+o)))，ρ = −inf 处没有球。
    在球并集外、真值 ≤ R − max ρ 时 = 到并集的精确距离；更远处 R 外的球可能更近，结果取 floor（调用方给的真值下界，要 ≥ R − max ρ 才不丢精度）。"""
    rho = rho.astype(np.float32)
    O, L = _ball_offsets(R / h)
    k = int(np.ceil(R / h))
    pad = np.pad(rho, k, constant_values=-np.inf)
    out = np.array(np.broadcast_to(np.asarray(floor, np.float32), rho.shape), np.float32)
    nx, ny, nz = rho.shape
    tmp = np.empty_like(out)
    for (dx, dy, dz), l in zip(O, (L * h).astype(np.float32)):
        sl = pad[k + dx:k + dx + nx, k + dy:k + dy + ny, k + dz:k + dz + nz]
        np.subtract(l, sl, out=tmp)
        np.minimum(out, tmp, out=out)
    return out


def morph_radii(P):
    """开 / 闭 / 再开的球半径上限与搜索半径。
    ball_union_field 的结果在"真值 ≤ R − 球半径上限"时精确，更远处给的是下界 floor：
      闭运算要用 d_O 在 O 外 ≤ CAPC 的精确值 → R1 − CAPO ≥ CAPC；
      再开（R_OPEN2 > 0）要用闭集内部 ≤ CAP3 的精确深度 → R2 − CAPC ≥ CAP3；最后一步只需零等值面 ±2H 精确。"""
    h, ro, rc = P["H"], P["R_OPEN"], P["R_CLOSE"]
    capo, capc = ro + P["CAP_EXTRA"], rc + P["CAP_EXTRA"]
    cap3 = (P.get("R_OPEN2", 0.0) + P["CAP_EXTRA"]) if P.get("R_OPEN2", 0.0) > 0 else 0.0
    R1 = max(capo + 2 * h, capo + capc)
    R2 = capc + max(2 * h, cap3)
    R3 = cap3 + 2 * h
    return capo, capc, R1, R2, cap3, R3


def morph_field(d, w, h, P):
    """d：原件 SDF（负 = 里）；w：0..1（D 边界 0、往里 RAMP 处 1）。返回补丁场（负 = 里）。
    开运算半径 = R_OPEN·(RFLOOR + (1−RFLOOR)·w)：到 D 边界也不降到 0 —— 半径 < 2H 的圆角 marching tetrahedra 只能画成锯齿（09-28 实测：
    过渡带里一圈"碎边"）；D 边界上圆角与原件锐棱之间留一个落在边界面里的小台阶（平面内，clean_check 记 micro_seam）。闭运算半径恒定 R_CLOSE。
    最后"贴回"：|F−d| < SNAP_TOL 处取 d（平面上补丁与原件逐点重合，接缝不留 0.002 的台阶）。"""
    ro, rc = P["R_OPEN"], P["R_CLOSE"]
    capo, capc, R1, R2, cap3, R3 = morph_radii(P)
    rfield = ro * (P["RFLOOR"] + (1.0 - P["RFLOOR"]) * w)
    # 开：内切球（半径 ≥ r(c) 的才算），球半径取 min(−d, capo)；O ⊆ 原件 → 下界 max(d, R1−capo)
    rho = np.where(-d >= rfield, np.minimum(-d, capo), -np.inf)
    del rfield
    dO = ball_union_field(rho, h, R1, np.maximum(d, R1 - capo))
    del rho
    # 闭：补集的开（dO 在 O 外精确到 capc）
    wc = P.get("CFLOOR", 1.0) + (1.0 - P.get("CFLOOR", 1.0)) * w           # 闭 / 再开半径的盒面衰减（CFLOOR=1 = 不衰减）
    rho2 = np.where(dO >= rc * wc, np.minimum(dO, capc), -np.inf)
    del dO
    F = -ball_union_field(rho2, h, R2, R2 - capc)
    del rho2
    if cap3 > 0:
        # 再开一次（半径 R_OPEN2）：闭运算在补集球之间留下的小凸棱 / 锥点再倒圆（09-28 L01 实测：三棱交汇的角上 MT 出一簇锐棱 → 碎边）。
        # F 在闭集内部精确到 R2−capc ≥ cap3，所以内切球半径可取到 cap3
        r3 = P["R_OPEN2"] * wc
        rho3 = np.where(-F >= r3, np.minimum(-F, cap3), -np.inf)
        F = ball_union_field(rho3, h, R3, np.maximum(F, R3 - cap3))
        del rho3
    t = P["SNAP_TOL"]
    if t > 0:                                                            # |F−d| ≤ t/2 完全贴回，t/2..t 过渡
        s_ = smoothstep((t - np.abs(F - d)) / (0.5 * t))
        F = F + (d - F) * s_
    return F


def smoothstep(s):
    s = np.clip(s, 0.0, 1.0)
    return s * s * (3 - 2 * s)


# ═════════════════════════════ 3. Marching tetrahedra（Kuhn 剖分，全网格一致 → 水密） ═════════════════════════════
_DIRS = np.array([(1, 0, 0), (0, 1, 0), (0, 0, 1), (1, 1, 0), (1, 0, 1), (0, 1, 1), (1, 1, 1)])
_TET_E = [(0, 1), (0, 2), (0, 3), (1, 2), (1, 3), (2, 3)]


def _build_tables():
    types = []
    for a, b, c in [(0, 1, 2), (0, 2, 1), (1, 0, 2), (1, 2, 0), (2, 0, 1), (2, 1, 0)]:
        ea, eb = np.eye(3, dtype=int)[a], np.eye(3, dtype=int)[b]
        V = np.array([(0, 0, 0), ea, ea + eb, (1, 1, 1)])
        edges = []
        for i, j in _TET_E:
            dv = V[j] - V[i]
            edges.append((i, int(np.nonzero((_DIRS == dv).all(1))[0][0])))       # (起点顶点号, 方向号)
        tab = {}
        for code in range(16):
            ins = [(code >> v) & 1 for v in range(4)]
            I = [v for v in range(4) if ins[v]]; Oo = [v for v in range(4) if not ins[v]]
            if len(I) in (0, 4):
                tab[code] = []; continue
            eid = lambda p, q: _TET_E.index((min(p, q), max(p, q)))
            if len(I) == 1 or len(Oo) == 1:
                s = I[0] if len(I) == 1 else Oo[0]
                tris = [[eid(s, t) for t in range(4) if t != s]]
            else:
                p, q = I; r, s = Oo
                tris = [[eid(p, r), eid(p, s), eid(q, s)], [eid(p, r), eid(q, s), eid(q, r)]]
            # 定向：用棱中点几何算法向，要朝向"外"（外顶点形心 − 内顶点形心）
            mid = np.array([(V[i] + V[j]) / 2 for i, j in _TET_E], float)
            out_dir = V[Oo].mean(0) - V[I].mean(0)
            fixed = []
            for t in tris:
                nrm = np.cross(mid[t[1]] - mid[t[0]], mid[t[2]] - mid[t[0]])
                fixed.append(t if nrm @ out_dir > 0 else [t[0], t[2], t[1]])
            tab[code] = fixed
        types.append((V, edges, tab))
    return types


_MT = _build_tables()


def marching_tets(F, g0, h):
    """F：网格场（负 = 里）。返回 (verts, faces)，外法向。F 最外一层应为正（补丁自带封口）。"""
    F = np.where(np.abs(F) < 1e-7, np.where(F < 0, -1e-7, 1e-7), F).astype(np.float64)
    nx, ny, nz = F.shape
    ins = F < 0
    c = [ins[i:nx - 1 + i, j:ny - 1 + j, k:nz - 1 + k] for i in (0, 1) for j in (0, 1) for k in (0, 1)]
    anyin = np.logical_or.reduce(c); allin = np.logical_and.reduce(c)
    ci = np.argwhere(anyin & ~allin)                                                # 活跃立方体左下角
    stride = np.array([ny * nz, nz, 1])
    eid_all, tri_all = [], []
    for V, edges, tab in _MT:
        vid = ci[:, None, :] + V[None, :, :]                                        # (n,4,3)
        f = F[vid[..., 0], vid[..., 1], vid[..., 2]]                                # (n,4)
        code = ((f < 0) * np.array([1, 2, 4, 8])).sum(1)
        # 每条四面体棱的全局 id = 起点线性号 × 7 + 方向
        gid = np.stack([(vid[:, s] * stride).sum(1) * 7 + dr for s, dr in edges], 1)   # (n,6)
        for cd in range(1, 15):
            sel = np.nonzero(code == cd)[0]
            if not len(sel):
                continue
            for t in tab[cd]:
                tri_all.append(gid[sel][:, t])
    if not tri_all:
        return np.zeros((0, 3)), np.zeros((0, 3), np.int64)
    T = np.concatenate(tri_all)
    uid, inv = np.unique(T.ravel(), return_inverse=True)
    faces = inv.reshape(-1, 3)
    # 顶点位置：沿全局棱线性插值
    lin, dr = uid // 7, uid % 7
    a = np.stack(np.unravel_index(lin, F.shape), 1)
    b = a + _DIRS[dr]
    fa = F[a[:, 0], a[:, 1], a[:, 2]]; fb = F[b[:, 0], b[:, 1], b[:, 2]]
    t = fa / (fa - fb)
    verts = g0 + (a + (b - a) * t[:, None]) * h
    return verts, faces


def _trilinear(F, g0, h, X):
    """网格场 F 在点 X 处的三线性插值与梯度。"""
    n = np.asarray(F.shape)
    u = (X - g0) / h
    i = np.clip(np.floor(u).astype(np.int64), 0, n - 2)
    t = np.clip(u - i, 0.0, 1.0)
    c = {}
    for a in (0, 1):
        for b in (0, 1):
            for e in (0, 1):
                c[a, b, e] = F[i[:, 0] + a, i[:, 1] + b, i[:, 2] + e]
    tx, ty, tz = t[:, 0], t[:, 1], t[:, 2]
    c00 = c[0, 0, 0] * (1 - tx) + c[1, 0, 0] * tx; c01 = c[0, 0, 1] * (1 - tx) + c[1, 0, 1] * tx
    c10 = c[0, 1, 0] * (1 - tx) + c[1, 1, 0] * tx; c11 = c[0, 1, 1] * (1 - tx) + c[1, 1, 1] * tx
    c0 = c00 * (1 - ty) + c10 * ty; c1 = c01 * (1 - ty) + c11 * ty
    val = c0 * (1 - tz) + c1 * tz
    gz = (c1 - c0) / h
    gy = ((c10 - c00) * (1 - tz) + (c11 - c01) * tz) / h
    d00 = c[1, 0, 0] - c[0, 0, 0]; d10 = c[1, 1, 0] - c[0, 1, 0]; d01 = c[1, 0, 1] - c[0, 0, 1]; d11 = c[1, 1, 1] - c[0, 1, 1]
    gx = ((d00 * (1 - ty) + d10 * ty) * (1 - tz) + (d01 * (1 - ty) + d11 * ty) * tz) / h
    return val, np.stack([gx, gy, gz], 1)


def relax_patch(V, faces, F, g0, h, iters, lam, pin):
    """切向 Laplace 松弛 + 牛顿投影回三线性等值面：改善 MT 三角形形状，不改拓扑。平面（F 线性）上的点仍严格在平面上。
    pin：不动的顶点（网格外层封口附近）。有三角形翻面就返回 None（调用方用未松弛的）。"""
    V = V.copy()
    E = np.concatenate([faces[:, [0, 1]], faces[:, [1, 2]], faces[:, [2, 0]]])
    E = np.concatenate([E, E[:, ::-1]])
    cnt = np.bincount(E[:, 0], minlength=len(V)).astype(float)
    mov = ~pin
    n0 = None
    for _ in range(iters):
        nb = np.zeros_like(V)
        for k in range(3):
            nb[:, k] = np.bincount(E[:, 0], weights=V[E[:, 1], k], minlength=len(V))
        L = nb / np.maximum(cnt, 1)[:, None] - V
        _, g = _trilinear(F, g0, h, V)
        gn = g / np.maximum(np.linalg.norm(g, axis=1), 1e-12)[:, None]
        Lt = L - np.einsum("ij,ij->i", L, gn)[:, None] * gn
        V[mov] += lam * Lt[mov]
        for _ in range(2):
            val, g = _trilinear(F, g0, h, V)
            g2 = np.maximum(np.einsum("ij,ij->i", g, g), 1e-12)
            step = (val / g2)[:, None] * g
            step = np.clip(step, -0.5 * h, 0.5 * h)
            V[mov] -= step[mov]
    # 翻面检查：面法向要和场梯度同向
    T = V[faces]
    fn = np.cross(T[:, 1] - T[:, 0], T[:, 2] - T[:, 0])
    _, g = _trilinear(F, g0, h, T.mean(1))
    area = np.linalg.norm(fn, axis=1)
    bad = (np.einsum("ij,ij->i", fn, g) <= 0) & (area > 1e-10)
    if bad.any():
        return None, int(bad.sum())
    return V, 0


# ═════════════════════════════ 4. 网格 ↔ manifold ═════════════════════════════
def to_manifold(verts, faces):
    import manifold3d as M
    m = M.Manifold(M.Mesh64(vert_properties=np.array(verts, dtype=np.float64, order="C", copy=True),
                            tri_verts=np.array(faces, dtype=np.uint64, order="C", copy=True)))
    if m.status() != M.Error.NoError:
        raise ValueError(f"manifold 转换失败: {m.status()}")
    return m


def from_manifold(m):
    import trimesh
    g = m.to_mesh64()
    return trimesh.Trimesh(vertices=np.array(g.vert_properties[:, :3], copy=True), faces=np.array(g.tri_verts, copy=True), process=False)


# ═════════════════════════════ 5. findings（clean_check JSON） ═════════════════════════════
# 件号 → (运动体 body, placed 名)；= tools/cad/cut_ledger._builders / hr49_work/ledger/*.json（左件；右件 = 镜像，查左件）
BODY = {"B01": ("trunk_base", "battery_door"), "B03": ("trunk_base", "battery_lid"), "E01": ("jaw_soft", "eye_white"),
        "E02": ("jaw_soft", "eye_pupil"), "E03": ("jaw_soft", "eye_highlight"), "H01": ("jaw_soft", "head_bracket"),
        "H02": ("jaw_soft", "head_clamp"), "H03": ("jaw_soft", "head_bottom_shell"), "H04": ("jaw_soft", "face_plate"),
        "H05": ("jaw_soft", "head_top_shell"), "H06": ("jaw_soft", "camera_clamp"), "H08": ("jaw_soft", "adapter_tray"), "H10": ("jaw_soft", "tof_clamp"),
        "H09": ("jaw_soft", "amp_bracket"), "J01": ("jaw_soft", "jaw"), "J02": ("jaw_soft", "jaw_adapter"), "J03": ("jaw_soft", "jaw_journal"),
        "L01": ("yaw2roll", "yaw2roll"), "L02": ("hip_l", "hip"), "L03": ("upper_leg_left", "upper_leg"), "L04": ("leg", "lower_leg"),
        "L05": ("ankle_left", "ankle_foot"), "L06": ("ankle_left", "sole"), "L07": ("ankle_left", "ankle_rear_arm"),
        "L10": ("hip_l", "hip_roll_sleeve"), "L13": ("hip_l", "hip_pitch_sleeve"), "N01": ("neck", "neck"), "N02": ("neck_pitch", "neck_pitch"),
        "N03": ("yaw_roll_motion", "yrm"), "N04": ("yaw_roll_motion", "head_journal"), "N05": ("yaw_roll_motion", "head_roll_adapter"),
        "N06": ("neck_pitch", "head_yaw_adapter"), "N07": ("yaw_roll_motion", "head_bearing_cap"), "N08": ("yaw_roll_motion", "head_yaw_cap"),
        "T01": ("trunk_base", "trunk"), "T02": ("trunk_base", "shell_L"), "T03": ("trunk_base", "shell_R")}
PLACED2PID = {v[1]: k for k, v in BODY.items()}


def pick_entry(check, label=None, stl=None):
    """clean_check 输出 → 单件条目（{checks:{thin,frag,holes}} 那一层）。"""
    if "checks" in check and "files" not in check:
        return check
    files = check.get("files") or []
    for f in files:
        if label and f.get("label") == label:
            return f
    if stl:
        b = os.path.basename(stl)[:-4]
        for f in files:
            if os.path.basename(f.get("file", ""))[:-4] == b or f.get("placed") == b or f.get("label") == b:
                return f
    if len(files) == 1:
        return files[0]
    raise ValueError(f"clean_check JSON 里找不到这个件（label={label}, stl={stl}）；可选 {[f.get('label') for f in files]}")


def load_findings(entry):
    """单件条目 → finding 列表（只要 flag 为真的）。每条：id, cat(thin/frag/holes), kind, lo, hi, pts, 以及原字段摘要。"""
    out = []
    ch = entry.get("checks", {})
    th = ch.get("thin") or {}
    for i, r in enumerate(th.get("findings") or []):
        if not r.get("flag", True):
            continue
        pts = np.asarray(r.get("pts") or [r["c"]], float)
        out.append(dict(id=f"thin#{i}", cat="thin", kind=r["kind"], lo=list(r["lo"]), hi=list(r["hi"]), c=list(r["c"]), pts=pts.tolist(),
                        area_mm2=r.get("area_mm2"), depth_mm=r.get("depth_mm"), t_med=r.get("t_med")))
    fr = ch.get("frag") or {}
    ns = nr = nd = 0
    for r in fr.get("findings") or []:
        if not r.get("flag", True):
            continue
        k = r.get("kind")
        if k == "spike":
            p = np.asarray(r["p"], float)
            out.append(dict(id=f"spike#{ns}", cat="frag", kind="spike", lo=(p - 0.3).tolist(), hi=(p + 0.3).tolist(), c=p.tolist(),
                            pts=[p.tolist()], deg=r.get("deg"), length_mm=r.get("length_mm"))); ns += 1
        elif k == "ragged":
            pts = np.asarray(r.get("pts") or [r["c"]], float)
            out.append(dict(id=f"ragged#{nr}", cat="frag", kind="ragged", lo=list(r["lo"]), hi=list(r["hi"]), c=list(r["c"]), pts=pts.tolist(),
                            n_short=r.get("n_short"), slab_mm=r.get("slab_mm"))); nr += 1
        elif k == "debris":
            lo, hi = np.asarray(r["lo"], float), np.asarray(r["hi"], float)
            out.append(dict(id=f"debris#{nd}", cat="frag", kind="debris", lo=lo.tolist(), hi=hi.tolist(), c=((lo + hi) / 2).tolist(),
                            pts=[((lo + hi) / 2).tolist()], volume_mm3=r.get("volume_mm3"))); nd += 1
    ho = ch.get("holes") or {}
    for r in ho.get("findings") or []:
        c = np.asarray(r.get("center", [0, 0, 0]), float)
        out.append(dict(id=f"hole#{r.get('id')}", cat="holes", kind="hole_" + str(r.get("partial_pattern") or r.get("kind")), lo=c.tolist(), hi=c.tolist(),
                        c=c.tolist(), pts=[c.tolist()], d_mm=r.get("d_mm"), reason=r.get("reason"), use=(r.get("use") or {}).get("match")))
    for j, r in enumerate(ho.get("open_ring_findings") or []):
        bb = np.asarray(r.get("bbox") or [r.get("center")] * 2, float)
        out.append(dict(id=f"ring#{r.get('hole_id')}.{j}", cat="holes", kind="open_ring", lo=bb[0].tolist(), hi=bb[-1].tolist(),
                        c=bb.mean(0).tolist(), pts=[bb.mean(0).tolist()], d_mm=r.get("d_mm"), reason=r.get("reason"), use=(r.get("use") or {}).get("match")))
    return out


# ═════════════════════════════ 6. 保护区 ═════════════════════════════
PROTECT_KINDS = ("screw_hole", "horn_hole", "pilot_hole", "counterbore", "spot_face", "nut_pocket", "through_hole", "bore",
                 "bearing_bore", "journal", "driven_disc", "lip", "boss", "key")
POSITIVE_KINDS = ("journal", "driven_disc", "boss", "key")          # 凸出的料（整根圆柱护）；其余是孔（小的整根护、大的护孔壁一圈）


def _is_channel(f):
    pur = str(f.get("purpose", ""))
    return f.get("kind") == "tool_channel" or "起子通道" in pur


def _val(x):
    return x.get("v") if isinstance(x, dict) else x


def declared_features(pid):
    """features.yaml 本件保护类特征 → 世界系 [{id, kind, purpose, a, b, d, d_in?, lo?, hi?}]（阵列逐孔展开 + 舵盘中心孔）。"""
    import yaml
    if pid not in BODY:
        return []
    if ROOT not in sys.path:
        sys.path.insert(0, ROOT)
    from duckstructure.lib import TW
    T = TW(BODY[pid][0])
    W = lambda p: (T @ np.r_[np.asarray(p, float), 1.0])[:3]
    F = yaml.safe_load(open(os.path.join(ROOT, "tools", "gate", "data", "features.yaml"), encoding="utf-8"))["features"]
    out = []
    for f in F:
        if f.get("part") != pid or f.get("kind") not in PROTECT_KINDS or _is_channel(f):
            continue
        g = f.get("geom") or {}
        insts = g.get("instances") or [g]
        for k, ins in enumerate(insts):
            rid = f"{f['id']}#{k}"
            base = dict(kind=f.get("kind"), purpose=str(f.get("purpose", ""))[:60])
            bbx = ins.get("bbox_mm") or g.get("bbox_mm")
            if bbx and (ins.get("shape") or g.get("shape")) == "box":
                C = np.array([W([bbx[i][0], bbx[j][1], bbx[k2][2]]) for i in (0, 1) for j in (0, 1) for k2 in (0, 1)])
                out.append(dict(id=rid, lo=C.min(0).tolist(), hi=C.max(0).tolist(), **base))
                continue
            sp = ins.get("axial_span_mm") or g.get("axial_span_mm")
            d = _val(ins.get("nominal_d_mm", g.get("nominal_d_mm")))
            if not sp or not d or not isinstance(sp[0], (list, tuple)) or isinstance(sp[0][0], (list, tuple)):
                continue
            a0, b0 = np.asarray(sp[0], float), np.asarray(sp[1], float)
            din = _val(ins.get("inner_d_mm", g.get("inner_d_mm")))
            hp = ins.get("hole_positions_mm") or (g.get("hole_positions_mm") if ins is g or not ins.get("axial_span_mm") else None)
            if hp:
                ax = (b0 - a0) / np.linalg.norm(b0 - a0)
                for q, hpos in enumerate(hp):
                    hpos = np.asarray(hpos, float)
                    off = (hpos - a0) - ((hpos - a0) @ ax) * ax
                    out.append(dict(id=f"{rid}.{q}", a=W(a0 + off).tolist(), b=W(b0 + off).tolist(), d=float(d), **base))
                cb = _val(g.get("center_bore_d_mm"))
                if cb:
                    out.append(dict(id=f"{rid}.center", a=W(a0).tolist(), b=W(b0).tolist(), d=float(cb), kind="bore_center",
                                    purpose="舵盘中心孔"))
            else:
                e = dict(id=rid, a=W(a0).tolist(), b=W(b0).tolist(), d=float(d), **base)
                if din:
                    e["d_in"] = float(din)
                out.append(e)
    return out


def geometric_holes(mesh, placed):
    """clean_check_holes 的圆柱识别（Ø1.2..31 的孔）+ 用途匹配（与 clean_check 同一份 features）。本进程不用 Embree。"""
    if HERE not in sys.path:
        sys.path.insert(0, HERE)
    import clean_check_holes as CH
    import clean_check as CC
    P = dict(CH.DEFAULTS)
    m = CH._prep(mesh)
    cyls = CH.detect_cylinders(m, P)
    feats = CC.features_for(placed) if placed else None
    own = other = None
    if isinstance(feats, dict):
        own, other = list(feats.get("own") or []), list(feats.get("other") or [])
    elif feats is not None:
        own = [f for f in feats if f.get("placed") == placed]; other = [f for f in feats if f.get("placed") != placed]
    out = []
    for c in cyls:
        if c["t1"] - c["t0"] < P["len_min"] or c["cov"] < P["cov_min"] or not (P["d_min"] <= 2 * c["r"] <= P["d_ring_max"]):
            continue
        h = dict(axis=np.asarray(c["axis"]), center=np.asarray(c["pt"]), d_mm=2 * c["r"], extent_mm=[c["t0"], c["t1"]])
        use = CH.match_use(h, own, other, P) if own is not None else {"match": "n/a"}
        ax = np.asarray(c["axis"], float); pt = np.asarray(c["pt"], float)
        out.append(dict(id=f"geo#{len(out)}", kind="geo_hole", a=(pt + c["t0"] * ax).tolist(), b=(pt + c["t1"] * ax).tolist(),
                        d=float(2 * c["r"]), cov=round(float(c["cov"]), 2), use=use.get("match"), feature=use.get("feature"),
                        purpose=str(use.get("purpose") or "")[:60]))
    return out


def build_protect(mesh, placed=None, extra=None, P=None):
    """保护体列表（世界系）。每个：{id, src, type:'cyl'|'box', a,b,r_out,r_in | lo,hi, why}。"""
    P = dict(PARAMS, **(P or {}))
    pad, apad = P["PROTECT_PAD"], P["PROTECT_AX_PAD"]
    zones, notes = [], []
    pid = PLACED2PID.get(placed)

    def surf_cov(a, b, r, n_ang=48, n_t=6):
        """登记的圆柱面 / 端面在网格上找得到多少：取样点到网格表面 ≤ 0.05 的比例（侧面，两个端面）。"""
        import trimesh
        u = (b - a) / np.linalg.norm(b - a)
        e1 = np.cross(u, [1.0, 0, 0] if abs(u[0]) < 0.9 else [0, 1.0, 0]); e1 /= np.linalg.norm(e1); e2 = np.cross(u, e1)
        th = np.linspace(0, 2 * np.pi, n_ang, endpoint=False)
        ring = np.cos(th)[:, None] * e1 + np.sin(th)[:, None] * e2
        side = np.concatenate([a + (b - a) * t + r * ring for t in np.linspace(0.1, 0.9, n_t)])
        rr = np.concatenate([f * r * ring for f in (0.35, 0.65, 0.9)])
        ends = [a + rr, b + rr]
        cov = []
        for Q in [side] + ends:
            _, dist, _ = trimesh.proximity.closest_point(mesh, Q)
            cov.append(float((dist <= 0.05).mean()))
        return cov

    def add_cyl(src, f, positive):
        a, b = np.asarray(f["a"], float), np.asarray(f["b"], float)
        L = np.linalg.norm(b - a)
        if L < 1e-9:
            return
        u = (b - a) / L
        r = f["d"] / 2
        why = f"{f.get('kind')} Ø{f['d']:.2f} {f.get('purpose', '')}".strip()
        if positive:
            # 凸出的配合件（从动盘 / 轴颈 / 凸台）：只护它的配合面 —— 侧面一圈 [r−1, r+1] + 两个端面 ±1 的盘；网格上找不到的（登记过时）不护
            cs, ca, cb = surf_cov(a, b, r)
            if max(cs, ca, cb) < 0.2:
                notes.append(f"{f['id']} {why}：侧面 / 端面在网格上覆盖率 {cs:.2f}/{ca:.2f}/{cb:.2f} < 0.2（登记与当前几何对不上）→ 不护")
                return
            if cs >= 0.2:
                zones.append(dict(id=f["id"] + ".side", src=src, type="cyl", a=(a - apad * u).tolist(), b=(b + apad * u).tolist(),
                                  r_out=r + pad, r_in=max(0.0, r - pad), why=why + f"（侧面，覆盖 {cs:.2f}）"))
            for nm, p0, cv in (("end_a", a, ca), ("end_b", b, cb)):
                if cv >= 0.2:
                    zones.append(dict(id=f["id"] + "." + nm, src=src, type="cyl", a=(p0 - apad * u).tolist(), b=(p0 + apad * u).tolist(),
                                      r_out=r + pad, r_in=0.0, why=why + f"（端面，覆盖 {cv:.2f}）"))
            return
        solid = f["d"] <= P["BORE_SOLID_D"]
        r_in = 0.0 if solid else max(0.0, (f.get("d_in", f["d"]) / 2) - pad)
        zones.append(dict(id=f["id"], src=src, type="cyl", a=(a - apad * u).tolist(), b=(b + apad * u).tolist(), r_out=r + pad, r_in=r_in,
                          why=why))

    # 几何找到的孔（实际存在的孔壁段）；起子通道 / 别件同轴让位 = channel，按用户规矩 4 不护
    geo = []
    try:
        for g in geometric_holes(mesh, placed):
            if g["use"] == "channel" or "起子通道" in g.get("purpose", ""):
                notes.append(f"{g['id']} Ø{g['d']:.2f} use={g['use']}（{g.get('feature')} {g.get('purpose', '')[:20]}）→ 起子 / 让位通道，按用户规矩不护")
                continue
            geo.append(g)
    except Exception as e:                                              # noqa: BLE001
        notes.append(f"几何找孔失败：{e!r}")

    def matched(f):
        """登记的孔 ↔ 几何孔：同轴（≤5°、轴距 ≤0.5）、同径（差 ≤ max(0.3, 5%)）、沿轴与登记段重叠。"""
        a, b = np.asarray(f["a"], float), np.asarray(f["b"], float); u = (b - a) / np.linalg.norm(b - a)
        hit = []
        for g in geo:
            ga, gb = np.asarray(g["a"], float), np.asarray(g["b"], float); gu = (gb - ga) / np.linalg.norm(gb - ga)
            if abs(gu @ u) < np.cos(np.radians(5)) or abs(g["d"] - f["d"]) > max(0.3, 0.05 * f["d"]):
                continue
            m = (ga + gb) / 2 - a
            if np.linalg.norm(m - (m @ u) * u) > 0.5:
                continue
            L = np.linalg.norm(b - a); ta, tb = sorted([(ga - a) @ u, (gb - a) @ u])
            if tb < -0.3 or ta > L + 0.3:                                # 沿轴要和登记的段重叠
                continue
            hit.append(g)
        return hit

    used_geo = set()
    if pid:
        try:
            for f in declared_features(pid):
                if "lo" in f:
                    zones.append(dict(id=f["id"], src="features.yaml", type="box", lo=(np.asarray(f["lo"]) - pad).tolist(),
                                      hi=(np.asarray(f["hi"]) + pad).tolist(), why=f"{f['kind']} {f['purpose']}"))
                    continue
                if f["kind"] in POSITIVE_KINDS:
                    add_cyl("features.yaml", f, True)
                    continue
                hit = matched(f)
                if hit:                                                  # 用几何实测的孔壁段（登记的是刀长，常常穿过整件）
                    for g in hit:
                        used_geo.add(g["id"])
                        add_cyl("features.yaml+geometry", dict(g, id=f"{f['id']}~{g['id']}", kind=f["kind"], purpose=f["purpose"]), False)
                else:
                    add_cyl("features.yaml", f, False)
        except Exception as e:                                          # noqa: BLE001
            notes.append(f"features.yaml 读取失败：{e!r}")
    else:
        notes.append(f"placed={placed!r} 不认识 → 没有登记特征，只用几何找孔")
    for g in geo:
        if g["id"] not in used_geo:
            add_cyl("geometry", dict(g, kind=f"geo/{g['use']}", purpose=(g.get("feature") or "") + " " + g.get("purpose", "")), False)
    for i, e in enumerate(extra or []):
        if "lo" in e:
            zones.append(dict(id=e.get("id", f"extra#{i}"), src="extra", type="box", lo=list(e["lo"]), hi=list(e["hi"]), why=e.get("why", "手工")))
        else:
            ee = dict(e, id=e.get("id", f"extra#{i}"), kind="extra", purpose=e.get("why", "手工"))
            a0, b0 = np.asarray(ee["a"], float), np.asarray(ee["b"], float); u0 = (b0 - a0) / np.linalg.norm(b0 - a0)
            zones.append(dict(id=ee["id"], src="extra", type="cyl", a=(a0 - apad * u0).tolist(), b=(b0 + apad * u0).tolist(),
                              r_out=ee["d"] / 2 + pad, r_in=0.0, why=ee["purpose"]))
    return zones, notes


def zone_sdf(z, X):
    """点到保护体的有符号距离（负 = 在里面）。X (n,3)。"""
    X = np.asarray(X, float)
    if z["type"] == "box":
        lo, hi = np.asarray(z["lo"]), np.asarray(z["hi"])
        q = np.maximum(lo - X, X - hi)
        return np.linalg.norm(np.maximum(q, 0), axis=1) + np.minimum(q.max(1), 0)
    a, b = np.asarray(z["a"]), np.asarray(z["b"])
    L = np.linalg.norm(b - a); u = (b - a) / L
    t = (X - a) @ u
    rho = np.linalg.norm((X - a) - np.outer(t, u), axis=1)
    d_ax = np.maximum(-t, t - L)
    d_rad = np.maximum(rho - z["r_out"], z["r_in"] - rho)
    return np.hypot(np.maximum(d_rad, 0), np.maximum(d_ax, 0)) + np.minimum(np.maximum(d_rad, d_ax), 0)


def zone_aabb(z):
    if z["type"] == "box":
        return np.asarray(z["lo"]), np.asarray(z["hi"])
    a, b = np.asarray(z["a"]), np.asarray(z["b"])
    return np.minimum(a, b) - z["r_out"], np.maximum(a, b) + z["r_out"]


def zone_manifold(z):
    import manifold3d as M
    if z["type"] == "box":
        lo, hi = np.asarray(z["lo"]), np.asarray(z["hi"])
        return M.Manifold.cube((hi - lo).tolist()).translate(lo.tolist())
    a, b = np.asarray(z["a"]), np.asarray(z["b"])
    L = float(np.linalg.norm(b - a)); u = (b - a) / L
    seg = 96
    c = M.Manifold.cylinder(L, z["r_out"], z["r_out"], seg)
    if z["r_in"] > 1e-6:
        c = c - M.Manifold.cylinder(L + 2, z["r_in"], z["r_in"], seg).translate([0, 0, -1])
    # z 轴 → u：Rodrigues
    zc = np.array([0.0, 0.0, 1.0]); v = np.cross(zc, u); s = np.linalg.norm(v); cth = zc @ u
    if s < 1e-12:
        R = np.eye(3) if cth > 0 else np.diag([1.0, -1.0, -1.0])
    else:
        K = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
        R = np.eye(3) + K + K @ K * ((1 - cth) / s ** 2)
    Mx = np.hstack([R, a.reshape(3, 1)])
    return c.transform(Mx.tolist())


# ═════════════════════════════ 7. 盒 + 精修主函数 ═════════════════════════════
def _merge_boxes(items, join):
    """items: [(lo, hi, [finding id])] → 重叠（间距 < join）的合并，直到不再变。"""
    items = [(np.asarray(lo, float), np.asarray(hi, float), list(ids)) for lo, hi, ids in items]
    changed = True
    while changed:
        changed = False
        out = []
        for lo, hi, ids in items:
            for k, (lo2, hi2, ids2) in enumerate(out):
                if np.all(lo <= hi2 + join) and np.all(lo2 <= hi + join):
                    out[k] = (np.minimum(lo, lo2), np.maximum(hi, hi2), ids2 + ids)
                    changed = True
                    break
            else:
                out.append((lo, hi, ids))
        items = out
    return items


def _nearest_zone(X, zones, pad=5.0):
    X = np.atleast_2d(np.asarray(X, float))
    best = (np.inf, None)
    for z in zones:
        zl, zh = zone_aabb(z)
        if np.any(X.min(0) > zh + pad) or np.any(X.max(0) < zl - pad):
            continue
        d = float(zone_sdf(z, X).min())
        if d < best[0]:
            best = (d, z)
    return best


def _box_zones(lo, hi, zones, step=0.25):
    """与盒真正相交的保护体（AABB 预筛 + 盒内 0.25 网格点上 sdf<0）。"""
    hit = []
    g = [np.arange(lo[a], hi[a] + 1e-9, step) for a in range(3)]
    X = np.stack(np.meshgrid(*g, indexing="ij"), -1).reshape(-1, 3)
    for z in zones:
        zl, zh = zone_aabb(z)
        if np.any(lo > zh) or np.any(hi < zl):
            continue
        if (zone_sdf(z, X) < step).any():
            hit.append(z)
    return hit


def _zone_hits_box(z, lo, hi, step=0.1, tol=0.02):
    """保护体与盒是否相交（盒 ∩ 保护体包围盒内按 0.1 采样，sdf < tol 算碰到）。"""
    zl, zh = zone_aabb(z)
    a, b = np.maximum(lo, zl - tol), np.minimum(hi, zh + tol)
    if np.any(a > b):
        return False
    g = [np.linspace(a[i], b[i], max(2, int(np.ceil((b[i] - a[i]) / step)) + 1)) for i in range(3)]
    X = np.stack(np.meshgrid(*g, indexing="ij"), -1).reshape(-1, 3)
    for k in range(0, len(X), 400_000):
        if (zone_sdf(z, X[k:k + 400_000]) < tol).any():
            return True
    return False


def _sharp_mid(mesh, deg=30.0):
    """原件锐棱（相邻面夹角 ≥ deg）两端点 (n,2,3)。"""
    ang = np.degrees(mesh.face_adjacency_angles)
    E = mesh.face_adjacency_edges[ang >= deg]
    return mesh.vertices[E]


def _face_score(seg, a, p, lo, hi, inward, ramp=0.5):
    """盒面 {x_a = p}（侧向范围 lo..hi，inward = 盒内方向 ±1）处的"脏"程度：
    穿过它的锐棱数 + 2 × 贴着它（两端都在 ±0.3 内）的锐棱数 + 3 × 过渡带（盒内 RAMP+0.1）里 ≥3 条锐棱交汇的角点数。"""
    if seg is None or not len(seg):
        return 0.0
    oth = [i for i in range(3) if i != a]
    mid = seg.mean(1)
    lat = np.all((mid[:, oth] >= lo[oth] - 0.3) & (mid[:, oth] <= hi[oth] + 0.3), axis=1)
    sg = seg[lat]
    s0, s1 = sg[:, 0, a] - p, sg[:, 1, a] - p
    cross = np.sum((s0 * s1) < 0)
    near = np.sum((np.abs(s0) < 0.3) & (np.abs(s1) < 0.3))
    corners = 0
    if len(sg):
        P_ = np.round(sg.reshape(-1, 3), 4)
        u, cnt = np.unique(P_, axis=0, return_counts=True)
        c3 = u[cnt >= 3]
        if len(c3):
            dd = (c3[:, a] - p) * inward
            corners = int(np.sum((dd >= -0.05) & (dd <= ramp + 0.1)))
    return float(cross + 2 * near + 3 * corners)


def _face_cross_pts(seg, a, p, lo, hi):
    """锐棱与盒面 {x_a = p}（侧向限在盒内 ±0.3）的交点。"""
    if seg is None or not len(seg):
        return np.zeros((0, 3))
    oth = [i for i in range(3) if i != a]
    s0, s1 = seg[:, 0, a] - p, seg[:, 1, a] - p
    k = (s0 * s1) < 0
    if not k.any():
        return np.zeros((0, 3))
    t = s0[k] / (s0[k] - s1[k])
    q = seg[k, 0] + (seg[k, 1] - seg[k, 0]) * t[:, None]
    lat = np.all((q[:, oth] >= lo[oth] - 0.3) & (q[:, oth] <= hi[oth] + 0.3), axis=1)
    return q[lat]


def _pair_penalty(Q, others, r=1.5):
    """这个面的台阶点与别的面的台阶点在 r 内的对数：两个盒面上的平面台阶凑在 1.5 mm 内，clean_check 会当成一堆不共面的碎边。"""
    if not len(Q) or not len(others):
        return 0
    d = np.linalg.norm(Q[:, None, :] - others[None, :, :], axis=2)
    return int((d < r).sum())


def fit_box(core_lo, core_hi, zones, sharp_mid, P):
    """finding 核心包围盒 → 修改盒：
      1) 每个面在核心外 [BOX_MARGIN_MIN, BOX_MARGIN_MAX] 里挑位置：离原件锐棱最远（面 ±0.3 内的锐棱中点最少，同分取最靠近 BOX_MARGIN 的）——
         盒面上的接缝台阶不和原有的短锐棱凑成一堆；
      2) 碰到保护体 / 拒修禁区就把某个面往里收（收得最少的那个面），收到离核心 < BOX_MARGIN_MIN 还碰 → 返回 (None, 原因)。
    盒里不留任何保护体 → 接缝全在盒的 6 个平面上（曲面接缝实测会出尖刺 / 碎边）。"""
    core_lo, core_hi = np.asarray(core_lo, float), np.asarray(core_hi, float)
    mmin, mtar, mmax = P["BOX_MARGIN_MIN"], P["BOX_MARGIN"], P["BOX_MARGIN_MAX"]
    lo, hi = core_lo - mtar, core_hi + mtar
    if P.get("STRICT_PROTECT"):
        hit = [z for z in zones if _zone_hits_box(z, lo, hi)]
        if hit:
            return None, "--strict-protect：外扩 1.5 的盒碰到保护区 " + ", ".join(z["id"] for z in hit[:4])
    if sharp_mid is not None and len(sharp_mid):
        for _ in range(3):                                              # 各面互相影响（侧向范围、台阶点配对），坐标轮换摆三轮
            for a in range(3):
                for side in (0, 1):
                    cands = np.arange(mmin, mmax + 1e-9, 0.1)
                    pos = core_lo[a] - cands if side == 0 else core_hi[a] + cands
                    others = [ _face_cross_pts(sharp_mid, b_, (lo[b_] if s_ == 0 else hi[b_]), lo, hi)
                               for b_ in range(3) for s_ in (0, 1) if b_ != a ]
                    others = np.concatenate(others) if others else np.zeros((0, 3))
                    score = np.array([_face_score(sharp_mid, a, p_, lo, hi, 1 if side == 0 else -1, P["RAMP"])
                                      + 3.0 * _pair_penalty(_face_cross_pts(sharp_mid, a, p_, lo, hi), others) for p_ in pos]) \
                        + 0.05 * np.abs(cands - mtar)
                    k = int(np.argmin(score))
                    if side == 0:
                        lo[a] = pos[k]
                    else:
                        hi[a] = pos[k]
    gap = P.get("ZONE_GAP", 0.0)
    for _ in range(30):
        hit = [z for z in zones if _zone_hits_box(z, lo, hi, tol=0.02 + gap)]
        if not hit:
            return (lo, hi), None
        z = hit[0]
        best = None
        for a in range(3):
            for side in (0, 1):
                lim = (core_lo[a] - mmin) if side == 0 else (core_hi[a] + mmin)
                cur_p = lo[a] if side == 0 else hi[a]
                if (side == 0 and cur_p >= lim) or (side == 1 and cur_p <= lim):
                    continue
                def ok(pv):
                    l2, h2 = lo.copy(), hi.copy()
                    if side == 0:
                        l2[a] = pv
                    else:
                        h2[a] = pv
                    return not _zone_hits_box(z, l2, h2, tol=0.02 + gap)
                if not ok(lim):
                    continue
                aa, bb = cur_p, lim                                          # aa 碰，bb 不碰
                for _ in range(12):
                    mid = 0.5 * (aa + bb)
                    if ok(mid):
                        bb = mid
                    else:
                        aa = mid
                mv = abs(bb - cur_p)
                if best is None or mv < best[0]:
                    best = (mv, a, side, bb)
        if best is None:
            return None, f"盒收到离 finding {mmin} mm 也躲不开保护体 {z['id']}（{z['why'][:40]}）"
        _, a, side, pv = best
        if side == 0:
            lo[a] = pv
        else:
            hi[a] = pv
    return None, "躲保护体迭代没收敛"


def _grid_weight(g0, shape, h, lo, hi, zones, ramp):
    """w：到修改区 D = 盒 − 保护体 边界的距离 / RAMP 的 smoothstep（盒外 / 保护体内 = 0）。"""
    X = [g0[a] + np.arange(shape[a]) * h for a in range(3)]
    db = np.full(tuple(shape), np.inf, np.float32)
    for a in range(3):
        da = np.minimum(X[a] - lo[a], hi[a] - X[a]).astype(np.float32)
        sh = [1, 1, 1]; sh[a] = -1
        db = np.minimum(db, da.reshape(sh))
    if zones:
        G = np.stack(np.meshgrid(*X, indexing="ij"), -1).reshape(-1, 3)
        for z in zones:
            db = np.minimum(db, zone_sdf(z, G).reshape(tuple(shape)).astype(np.float32))
        del G
    return smoothstep(db / ramp)


def weld_f32(m, tol):
    """顶点按 float32（STL 精度）焊合 → 去掉退化面 → 重建 manifold。不做这步，布尔接缝上相距 < float32 分辨率的顶点在 STL 里会并成一点，
    读回来是非流形边 / 一堆零面积"碎体"。tol > 0 时再 simplify(tol)（lib.clean_print_topology 同法）——
    注意 simplify 是整件的，会把盒外的共面三角形也重新剖分（形状不变，但 clean_check 的碎边统计依赖剖分，09-28 N02 实测盒外冒出 431 条的"新碎边"），
    所以缺省 tol=0，只有焊合后重建失败才退回 simplify(1e-6)。失败返回 None。"""
    t = from_manifold(m)
    v, inv = np.unique(np.asarray(t.vertices, dtype=np.float32).astype(np.float64), axis=0, return_inverse=True)
    f = np.asarray(inv).reshape(-1)[np.asarray(t.faces)]
    f = f[(f[:, 0] != f[:, 1]) & (f[:, 1] != f[:, 2]) & (f[:, 0] != f[:, 2])]
    try:
        m2 = to_manifold(v, f)
    except ValueError:
        return None
    return m2.simplify(tol) if tol > 0 else m2


def _small_components(m, keep_min):
    """manifold → (主体, [丢掉的小碎体体积])；小于 keep_min 的连通块丢掉。"""
    parts = m.decompose()
    if len(parts) <= 1:
        return m, []
    parts = sorted(parts, key=lambda p: -p.volume())
    main = parts[0]
    dropped, keep = [], [main]
    for p in parts[1:]:
        if p.volume() < keep_min:
            dropped.append(round(float(p.volume()), 4))
        else:
            keep.append(p)
    import manifold3d as M
    out = keep[0]
    for p in keep[1:]:
        out = out + p
    return out, dropped


def finish(mesh, findings, protect=None, params=None, log=print, keep_out_extra=None, box_prefix="", prev_boxes=None):
    """局部精修。mesh：trimesh（世界系，水密）；findings：load_findings() 的列表；protect：build_protect() 的保护体列表（None = 不设保护）。
    返回 (新 trimesh, report dict)。任何一盒失败都回退到该盒之前的状态、继续下一盒。"""
    import manifold3d as M
    P = dict(PARAMS, **(params or {}))
    h = P["H"]
    capo, capc, R1, R2, cap3, R3 = morph_radii(P)
    band = capo + 2 * h
    zones = list(protect or [])
    t_all = time.time()
    rep = dict(params={k: (list(v) if isinstance(v, tuple) else v) for k, v in P.items()}, radii=dict(capo=capo, capc=capc, R1=R1, R2=R2, cap3=cap3, R3=R3, band=band),
               findings=[], boxes=[], notes=[])
    cur = to_manifold(mesh.vertices, mesh.faces)
    V0 = cur.volume()
    rep["volume_before"] = round(V0, 3)
    # 1) 分类
    todo = []
    for f in findings:
        row = dict(id=f["id"], cat=f["cat"], kind=f["kind"], c=[round(v, 2) for v in f["c"]])
        if f["cat"] == "holes":
            row.update(status="not_handled", reason="孔类（侧壁开口 / 开口环）要回源码改刀，精修不动")
        elif f["kind"] == "debris":
            row.update(status="debris")
        else:
            d, z = _nearest_zone(f["pts"] + [f["c"]], zones)
            row["protect_dist_mm"] = None if z is None else round(d, 3)
            if z is not None and d < P["FIND_PAD"]:
                row.update(status="refused", reason=f"finding 本身在保护区里 / 贴着保护区（{z['id']}：{z['why'][:50]}；距离 {d:.2f} mm < {P['FIND_PAD']}）",
                           zone=z["id"])
            else:
                row.update(status="todo")
                todo.append(f)
        rep["findings"].append(row)
    rows = {r["id"]: r for r in rep["findings"]}
    # 2) 小碎体（debris finding 或任何不连通小块）
    if any(r["status"] == "debris" for r in rep["findings"]) or len(cur.decompose()) > 1:
        cur2, dropped = _small_components(cur, P["DEBRIS_MM3"])
        if dropped:
            rep["notes"].append(f"删掉小碎体 {len(dropped)} 块，体积 {dropped} mm³")
            cur = cur2
        for r in rep["findings"]:
            if r["status"] == "debris":
                r["status"] = "fixed" if dropped else "failed"
    # 3) 拒修的 finding 周围设禁区（不让别的盒把它修一半）
    keep_out = []
    for f in findings:
        if rows[f["id"]]["status"] == "refused":
            X = np.asarray(f["pts"] + [f["c"]], float)
            keep_out.append(dict(id="keepout:" + f["id"], src="refused", type="box", lo=(X.min(0) - 0.5).tolist(), hi=(X.max(0) + 0.5).tolist(),
                                 why="拒修 finding 的禁区"))
    # 4) 盒：先按每条 finding 摆盒（躲保护体 / 禁区），重叠的合并后再摆一次
    keep_out = keep_out + list(keep_out_extra or [])
    avoid = zones + keep_out
    sharp = _sharp_mid(mesh)
    fmap = {f["id"]: f for f in todo}
    groups = []
    h_ = P["H"]
    capo_, capc_, R1_, R2_, cap3_, R3_ = morph_radii(P)
    vox = lambda lo_, hi_: float(np.prod(np.floor((hi_ - lo_ + 2 * (R1_ + R2_ + R3_ + h_)) / h_) + 1))
    for f in todo:
        bx, why = fit_box(f["lo"], f["hi"], avoid, sharp, P)
        if bx is None:
            rows[f["id"]].update(status="refused", reason=why)
            continue
        # 第 2 轮起：新 finding 多半是上一轮盒面接缝的台阶，新盒要把碰到的旧盒整个包进去（旧接缝进核心被抹平，新盒面落在更外面），
        # 否则新盒面又切在旧盒修过的圆角上、留下新台阶
        for pb in prev_boxes or []:
            plo, phi = np.asarray(pb["lo"], float), np.asarray(pb["hi"], float)
            if np.all(bx[0] <= phi) and np.all(plo <= bx[1]):
                clo = np.minimum(np.asarray(f["lo"], float), plo); chi = np.maximum(np.asarray(f["hi"], float), phi)
                P_small = dict(P, BOX_MARGIN=P["BOX_MARGIN_MIN"] + 0.2, BOX_MARGIN_MAX=P["BOX_MARGIN"])
                bx2, why2 = fit_box(clo, chi, avoid, sharp, P_small)
                if bx2 is not None and vox(bx2[0], bx2[1]) <= P["MAX_VOXELS"]:
                    bx = bx2
                    rows[f["id"]]["grown_from"] = pb["id"]
        groups.append((bx[0], bx[1], [f["id"]]))
    changed = True
    while changed:
        changed = False
        out_g = []
        for g in groups:
            for k, g2 in enumerate(out_g):
                if np.all(g[0] <= g2[1] + P["BOX_JOIN"]) and np.all(g2[0] <= g[1] + P["BOX_JOIN"]):
                    ids = g2[2] + g[2]
                    clo = np.min([fmap[i]["lo"] for i in ids], axis=0); chi = np.max([fmap[i]["hi"] for i in ids], axis=0)
                    bx, why = fit_box(clo, chi, avoid, sharp, P)
                    if bx is None:
                        out_g[k] = (np.minimum(g[0], g2[0]), np.maximum(g[1], g2[1]), ids, why)
                    else:
                        out_g[k] = (bx[0], bx[1], ids)
                    changed = True
                    break
            else:
                out_g.append(g)
        groups = []
        for g in out_g:
            if len(g) == 4:                                               # 合并后摆不下 → 整组拒修
                for i in g[2]:
                    rows[i].update(status="refused", reason="几条 finding 的盒重叠、合并后" + g[3])
            else:
                groups.append(g)
    # 合并后太大的组：拆回单条各自一盒（按顺序修，后一盒看到前一盒的结果；接缝出事由自检 + 回退兜底）
    final_g = []
    for g in groups:
        if len(g[2]) > 1 and vox(g[0], g[1]) > P["MAX_VOXELS"]:
            for i in g[2]:
                bx, why = fit_box(fmap[i]["lo"], fmap[i]["hi"], avoid, sharp, P)
                if bx is None:
                    rows[i].update(status="refused", reason=why)
                else:
                    final_g.append((bx[0], bx[1], [i]))
                    rows[i]["split_from_group"] = True
        else:
            final_g.append(g)
    groups = final_g
    for bi, (lo, hi, ids) in enumerate(groups):
        t0 = time.time()
        b = dict(id=f"{box_prefix}B{bi}", lo=np.round(lo, 3).tolist(), hi=np.round(hi, 3).tolist(), findings=ids)
        rep["boxes"].append(b)
        bz = []                                                           # 盒已经躲开了所有保护体
        b["protect_in_box"] = []
        nvox = int(np.prod(np.floor((hi - lo + 2 * (R1 + R2 + R3 + h)) / h) + 1))
        if nvox > P["MAX_VOXELS"] and len(ids) == 1:
            # 单条 finding 的盒超限：外扩收到 BOX_MARGIN_MIN 再试一次（L06 / L07 的碎边盒只超 20–25%）
            f0 = fmap[ids[0]]
            bx2, _ = fit_box(f0["lo"], f0["hi"], avoid, sharp, dict(P, BOX_MARGIN=P["BOX_MARGIN_MIN"], BOX_MARGIN_MAX=P["BOX_MARGIN_MIN"] + 0.3))
            if bx2 is not None:
                n2 = int(np.prod(np.floor((bx2[1] - bx2[0] + 2 * (R1 + R2 + R3 + h)) / h) + 1))
                if n2 <= P["MAX_VOXELS"]:
                    lo, hi, nvox = bx2[0], bx2[1], n2
                    b.update(lo=np.round(lo, 3).tolist(), hi=np.round(hi, 3).tolist(), shrunk_margin=True)
        b["voxels"] = nvox
        if nvox > P["MAX_VOXELS"]:
            b.update(status="refused", reason=f"盒太大（{nvox:.2e} 体素 > {P['MAX_VOXELS']:.0e}）：大片问题回源码改刀")
            for i in ids:
                rows[i].update(status="refused", reason=b["reason"])
            continue
        try:
            tm = from_manifold(cur)
            gpad = R1 + R2 + R3 + h                                            # 网格比盒大出形态学的作用距离：盒内结果与"无限大网格"一致
            g0, shape, d, info = grid_sdf(tm, lo, hi, h, band, P["JITTER"], pad=gpad)
            b["odd_columns"] = info["n_odd_columns"]
            w = _grid_weight(g0, shape, h, lo, hi, bz, P["RAMP"])
            F = morph_field(d, w, h, P)
            del w
            # 真正改动的体素：里外变了，或者表面附近场差 > SNAP_TOL（圆角 / 填角）
            chg = ((F < 0) != (d < 0)) | ((np.abs(F - d) > P["SNAP_TOL"]) & (np.abs(d) < 2 * h))
            b["changed_voxels"] = int(chg.sum())
            if not chg.any():
                raise ValueError("形态学在盒内没有改动任何东西（finding 可能是检测阈值边缘）")
            if P["DOMAIN"] == "blob":
                # 修改区 D = 改动体素外扩 DOMAIN_PAD ∩ 盒 − 保护体（试验选项：09-28 实测接缝退化三角更多，默认不用）
                from scipy.ndimage import distance_transform_edt
                gdom = (distance_transform_edt(~chg) * h - P["DOMAIN_PAD"]).astype(np.float32)
                Xg = [g0[a_] + np.arange(shape[a_]) * h for a_ in range(3)]
                for a_ in range(3):
                    da = np.maximum(lo[a_] - Xg[a_], Xg[a_] - hi[a_]).astype(np.float32)
                    sh_ = [1, 1, 1]; sh_[a_] = -1
                    gdom = np.maximum(gdom, da.reshape(sh_))
                if bz:
                    G = np.stack(np.meshgrid(*Xg, indexing="ij"), -1).reshape(-1, 3)
                    for z in bz:
                        gdom = np.maximum(gdom, -zone_sdf(z, G).reshape(tuple(shape)).astype(np.float32))
                    del G
                gdom[0] = gdom[-1] = 1.0; gdom[:, 0] = gdom[:, -1] = 1.0; gdom[:, :, 0] = gdom[:, :, -1] = 1.0
                DV, DF = marching_tets(gdom, g0, h)
                del gdom
                D = to_manifold(DV, DF)
            else:
                D = M.Manifold.cube((hi - lo).tolist()).translate(lo.tolist())
                for z in bz:
                    D = D - zone_manifold(z)
            del chg
            b["domain_mm3"] = round(D.volume(), 3)
            del d
            F[0] = F[-1] = band; F[:, 0] = F[:, -1] = band; F[:, :, 0] = F[:, :, -1] = band
            V, Fc = marching_tets(F, g0, h)
            if P["RELAX_ITERS"] > 0 and len(V):
                inner_lo, inner_hi = lo - 0.25 * h, hi + 0.25 * h
                pin = np.any((V < inner_lo) | (V > inner_hi), axis=1)
                V2, nbad = relax_patch(V, Fc, F, g0, h, P["RELAX_ITERS"], P["RELAX_LAMBDA"], pin)
                b["relax"] = "ok" if V2 is not None else f"翻面 {nbad} 个，用未松弛的"
                if V2 is not None:
                    V = V2
            del F
            patch = to_manifold(V, Fc)
            if P["PATCH_SIMPLIFY"] > 0:
                patch = patch.simplify(P["PATCH_SIMPLIFY"])
            new = (cur - D) + (patch ^ D)
            if new.status() != M.Error.NoError:
                raise ValueError(f"布尔失败 {new.status()}")
            if P["SEAM_SIMPLIFY"] > 0:
                new = new.simplify(P["SEAM_SIMPLIFY"])
            new, dropped = _small_components(new, P["DEBRIS_MM3"])
            if len(new.decompose()) > 1:
                raise ValueError("修完不是单体（有 ≥1 mm³ 的块被分出去）")
            dv = new.volume() - cur.volume()
            if abs(new.volume() - V0) > P["VOL_TOL"] * V0:
                raise ValueError(f"累计体积变化 {new.volume() - V0:+.2f} mm³ 超过 {P['VOL_TOL']:.1%}")
            cur = new
            b.update(status="fixed", dV_mm3=round(dv, 4), patch_tris=int(patch.num_tri()), dropped_debris_mm3=dropped)
            for i in ids:
                rows[i].update(status="fixed_pending_check", box=b["id"])
        except Exception as e:                                          # noqa: BLE001
            b.update(status="failed", reason=f"{type(e).__name__}: {e}")
            for i in ids:
                rows[i].update(status="failed", reason=b["reason"], box=b["id"])
        import gc
        gc.collect()
        b["seconds"] = round(time.time() - t0, 1)
        b["peak_rss_gb"] = round(rss_gb(), 2)
        log(f"  盒 {b['id']} {b['status']:8s} 体素 {b.get('voxels', 0):>9,} 用时 {b['seconds']:5.1f}s  体积差 {b.get('dV_mm3', 0):+.3f} mm³  峰值 {b['peak_rss_gb']} GB  "
            f"{b.get('reason', '')[:80]}  finding {ids}")
    if P["WELD_F32"]:
        w32 = weld_f32(cur, P["WELD_SIMPLIFY"])
        if w32 is None:
            w32 = weld_f32(cur.simplify(1e-6), 0.0)
            rep["notes"].append("float32 焊合后重建失败，退回整件 simplify(1e-6) 再焊（盒外三角剖分可能变了）")
        if w32 is not None and abs(w32.volume() - cur.volume()) < 1e-3 and len(w32.decompose()) == len(cur.decompose()):
            cur = w32
            rep["weld_f32"] = "ok"
        else:
            rep["weld_f32"] = "失败，保留未焊合的（STL 读回可能有零面积碎三角）"
    out = from_manifold(cur)
    rep["volume_after"] = round(cur.volume(), 3)
    rep["dV_mm3"] = round(cur.volume() - V0, 3)
    rep["dV_rel"] = round((cur.volume() - V0) / V0, 6)
    rep["n_bodies"] = len(cur.decompose())
    rep["watertight"] = bool(out.is_watertight)
    rep["faces_before"], rep["faces_after"] = int(len(mesh.faces)), int(len(out.faces))
    rep["seconds"] = round(time.time() - t_all, 1)
    rep["peak_rss_gb"] = round(rss_gb(), 2)
    rep["keep_out"] = keep_out
    return out, rep


# ═════════════════════════════ 8. 自检（clean_check 子进程 + 对账） ═════════════════════════════
HARD_KINDS = ("membrane", "blade", "spike", "debris")            # 用户硬线：尖刺 / 断裂 / 比挤出线薄的薄片；碎边是"尽量修"


def is_hard(f):
    return f["cat"] == "thin" or f["kind"] in HARD_KINDS


def run_clean_check(stl, label, placed, out_json, timeout=900):
    """clean_check 单文件子进程模式（与 algo 口径一致：thin,frag,holes，orig）。本进程有 manifold3d，不能同进程开 Embree。"""
    cmd = [sys.executable, "-B", os.path.join(HERE, "clean_check.py"), "--child", os.path.abspath(stl), "--label", label or "part",
           "--placed", placed or "", "--tmp", os.path.abspath(out_json), "--checks", "thin,frag,holes", "--profile", "orig"]
    t0 = time.time()
    r = subprocess.run(cmd, cwd=ROOT, capture_output=True, text=True, timeout=timeout)
    if r.returncode != 0 or not os.path.exists(out_json):
        raise RuntimeError(f"clean_check 失败 rc={r.returncode}: {r.stderr[-800:]}")
    res = json.load(open(out_json, encoding="utf-8"))
    res["_wall_s"] = round(time.time() - t0, 1)
    return res


def _box_overlap(alo, ahi, blo, bhi, pad=0.0):
    return bool(np.all(np.asarray(alo) <= np.asarray(bhi) + pad) and np.all(np.asarray(blo) <= np.asarray(ahi) + pad))


def compare(before, after, rep):
    """before / after：load_findings() 列表。按类别 + 位置（包围盒外扩 0.5 相交）对账 → 消失 / 残留 / 新增（盒内 / 盒外）。"""
    grp = lambda f: "thin" if f["cat"] == "thin" else f["kind"]
    used = set()
    res = dict(gone=[], remaining=[], new_in_box=[], new_outside=[])
    for f in before:
        hit = [g for g in after if grp(g) == grp(f) and _box_overlap(f["lo"], f["hi"], g["lo"], g["hi"], 0.5)]
        if hit:
            res["remaining"].append(dict(id=f["id"], kind=f["kind"], c=f["c"], now=[g["id"] for g in hit]))
            used.update(g["id"] for g in hit)
        else:
            res["gone"].append(f["id"])
    boxes = [b for b in rep.get("boxes", []) if b.get("status") == "fixed"]
    for g in after:
        if g["id"] in used:
            continue
        inb = [b["id"] for b in boxes if _box_overlap(g["lo"], g["hi"], b["lo"], b["hi"], 0.0)]
        row = dict(id=g["id"], cat=g["cat"], kind=g["kind"], c=[round(v, 2) for v in g["c"]], boxes=inb,
                   detail={k: g.get(k) for k in ("area_mm2", "depth_mm", "t_med", "length_mm", "n_short", "slab_mm", "d_mm", "reason") if g.get(k) is not None})
        (res["new_in_box"] if inb else res["new_outside"]).append(row)
    return res


def diff_audit(old_mesh, new_mesh, rep, zones):
    """新旧件布尔差：总削 / 补体积、落在修改盒外的差（应为 0）、落在保护体里的差（应为 0）。"""
    import manifold3d as M
    A = to_manifold(old_mesh.vertices, old_mesh.faces); B = to_manifold(new_mesh.vertices, new_mesh.faces)
    rem = A - B; add = B - A
    out = dict(removed_mm3=round(rem.volume(), 4), added_mm3=round(add.volume(), 4))
    sym = rem + add
    boxes = [b for b in rep.get("boxes", []) if b.get("status") == "fixed"]
    U = None
    for b in boxes:
        lo, hi = np.asarray(b["lo"]), np.asarray(b["hi"])
        c = M.Manifold.cube((hi - lo).tolist()).translate(lo.tolist())
        U = c if U is None else U + c
    out["diff_outside_boxes_mm3"] = round((sym - U).volume() if U is not None else sym.volume(), 6)
    worst = (0.0, None)
    for z in zones:
        v = abs((sym ^ zone_manifold(z)).volume())
        if v > worst[0]:
            worst = (v, z["id"])
    out["diff_in_protect_mm3"] = round(worst[0], 6)
    out["diff_in_protect_worst_zone"] = worst[1]
    out["diff_outside_boxes_mm3"] = abs(out["diff_outside_boxes_mm3"])
    # 布尔体积在大片共面处会有数值噪声（H03 实测盒外 −0.064 mm³），再用表面采样量一次：盒外 / 保护体内的新旧表面最大偏差
    import trimesh
    lo_hi = [(np.asarray(b["lo"]) - 0.02, np.asarray(b["hi"]) + 0.02) for b in boxes]
    def outside(X):
        k = np.ones(len(X), bool)
        for lo, hi in lo_hi:
            k &= ~np.all((X >= lo) & (X <= hi), axis=1)
        return k
    dev_out, dev_prot = 0.0, 0.0
    for src, dst in ((new_mesh, old_mesh), (old_mesh, new_mesh)):
        X, _ = trimesh.sample.sample_surface_even(src, 20000, seed=1) if hasattr(trimesh.sample, "sample_surface_even") else trimesh.sample.sample_surface(src, 20000)
        X = np.asarray(X)
        ko = outside(X)
        if ko.any():
            _, dd, _ = trimesh.proximity.closest_point(dst, X[ko])
            dev_out = max(dev_out, float(dd.max()))
        kp = np.zeros(len(X), bool)
        for z in zones:
            kp |= zone_sdf(z, X) < 0
        if kp.any():
            _, dd, _ = trimesh.proximity.closest_point(dst, X[kp])
            dev_prot = max(dev_prot, float(dd.max()))
    out["max_dev_outside_boxes_mm"] = round(dev_out, 6)
    out["max_dev_in_protect_mm"] = round(dev_prot, 6)
    if not sym.is_empty():
        bb = np.asarray(sym.bounding_box()).reshape(2, 3)
        out["diff_bbox"] = np.round(bb, 2).tolist()
    return out


# ═════════════════════════════ 9. 对比图 ═════════════════════════════
def _render(ax, mesh_m, lo, hi, axis, sgn, title, boxes=(), zones=(), pts=None):
    """把件裁到 [lo,hi] 里，沿 axis（sgn=+1 从 +axis 往 −axis 看）正交投影，Lambert 着色 + 画家算法。"""
    import manifold3d as M
    from matplotlib.collections import PolyCollection
    from matplotlib.patches import Rectangle
    clip = mesh_m ^ M.Manifold.cube((hi - lo).tolist()).translate(lo.tolist())
    uv = [i for i in range(3) if i != axis]
    if sgn < 0:
        uv = uv[::-1]
    if not clip.is_empty():
        t = from_manifold(clip)
        T = t.triangles; N = t.face_normals
        vd = np.zeros(3); vd[axis] = sgn
        keep = (N @ vd) > 1e-6                                          # 背面剔除
        T, N = T[keep], N[keep]
        o = np.argsort(T[:, :, axis].mean(1) * sgn)
        Lg = np.array([0.35, 0.45, 0.82]); Lg /= np.linalg.norm(Lg)
        lam = 0.35 + 0.45 * np.abs(N @ Lg) + 0.2 * (N @ vd)
        col = np.clip(np.stack([0.55 * lam + 0.15, 0.62 * lam + 0.14, 0.72 * lam + 0.16], 1), 0, 1)
        ax.add_collection(PolyCollection(T[o][:, :, uv], facecolors=col[o], edgecolors="none", antialiased=False))
    for b in boxes:
        blo, bhi = np.asarray(b["lo"]), np.asarray(b["hi"])
        colr = {"fixed": "#1a9850", "refused": "#d73027", "not_handled": "#d73027", "failed": "#fc8d59"}.get(b.get("status"), "#4575b4")
        ax.add_patch(Rectangle((blo[uv[0]], blo[uv[1]]), bhi[uv[0]] - blo[uv[0]], bhi[uv[1]] - blo[uv[1]], fill=False, ec=colr, lw=1.2, ls="--"))
        ax.text(blo[uv[0]], bhi[uv[1]], b["id"], color=colr, fontsize=7, va="bottom")
    if pts is not None and len(pts):
        P_ = np.asarray(pts)
        ax.plot(P_[:, uv[0]], P_[:, uv[1]], "x", color="#c51b7d", ms=3, mew=0.8)
    ax.set_xlim(lo[uv[0]], hi[uv[0]]); ax.set_ylim(lo[uv[1]], hi[uv[1]])
    ax.set_aspect("equal"); ax.tick_params(labelsize=6)
    ax.set_xlabel("xyz"[uv[0]] + " mm", fontsize=6); ax.set_ylabel("xyz"[uv[1]] + " mm", fontsize=6)
    ax.set_title(title, fontsize=7)


def plot_compare(old_mesh, new_mesh, rep, findings, out_png, label="", max_rows=9):
    """每行一个盒（修了 / 失败）或一组拒修的 finding：修前 / 修后 × 两个视向（盒最薄的轴 + 次薄的轴）。
    框：绿 = 修了，橙 = 失败，红 = 拒修 / 不处理；× = finding 采样点。行数上限 max_rows，其余只列在报告里。"""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    plt.rcParams["font.sans-serif"] = ["Hiragino Sans GB", "Heiti SC", "Arial Unicode MS", "DejaVu Sans"]
    plt.rcParams["axes.unicode_minus"] = False
    A = to_manifold(old_mesh.vertices, old_mesh.faces); B = to_manifold(new_mesh.vertices, new_mesh.faces)
    fmap = {f["id"]: f for f in findings}
    rows = [dict(b, kind="box") for b in rep["boxes"] if b.get("status") in ("fixed", "failed")]
    # 拒修 / 不处理的 finding：相距 < 2 mm 的并成一组
    grp = []
    for r in rep["findings"]:
        if r["status"] not in ("refused", "not_handled") or r["id"] not in fmap:
            continue
        f = fmap[r["id"]]
        lo, hi = np.asarray(f["lo"], float), np.asarray(f["hi"], float)
        for g in grp:
            if np.all(lo <= g["hi"] + 2.0) and np.all(g["lo"] <= hi + 2.0):
                g["lo"] = np.minimum(g["lo"], lo); g["hi"] = np.maximum(g["hi"], hi); g["findings"].append(r["id"])
                g["reason"] = g["reason"] if len(g["reason"]) > 60 else (g["reason"] + "；" + (r.get("reason") or ""))
                break
        else:
            grp.append(dict(id="拒修" if r["status"] == "refused" else "不处理", lo=lo, hi=hi, findings=[r["id"]], status=r["status"],
                            reason=r.get("reason") or ""))
    for g in grp:
        g["lo"] = (g["lo"] - 1.2).tolist(); g["hi"] = (g["hi"] + 1.2).tolist()
    rows += grp
    if not rows:
        return None
    rows = rows[:max_rows]
    n = len(rows)
    fig, axs = plt.subplots(n, 4, figsize=(13, 3.2 * n + 0.8), squeeze=False)
    for i, b in enumerate(rows):
        lo, hi = np.asarray(b["lo"], float) - 0.6, np.asarray(b["hi"], float) + 0.6
        order = np.argsort(hi - lo)
        pl = [np.asarray(fmap[k]["pts"]) for k in b["findings"] if k in fmap]
        pts = np.concatenate(pl) if pl else None
        st = {"fixed": "已修", "failed": "失败", "refused": "拒修", "not_handled": "不处理"}.get(b.get("status"), b.get("status"))
        for j, axis in enumerate(order[:2]):
            for k, (mm, tag) in enumerate(((A, "修前"), (B, "修后"))):
                _render(axs[i][2 * j + k], mm, lo, hi, int(axis), 1, f"{label} {b['id']} {tag}  沿 -{'xyz'[axis]} 看  [{st}]",
                        boxes=[b], pts=pts if k == 0 else None)
        why = (b.get("reason") or "")[:120]
        axs[i][0].text(0.0, -0.3, f"finding: {', '.join(b['findings'])[:80]}  {why}", transform=axs[i][0].transAxes, fontsize=6, va="top")
    fig.suptitle(f"{label} finish_pass 修前 / 修后（绿框 = 已修，橙 = 失败，红 = 拒修 / 不处理；× = finding 采样点）", fontsize=9)
    fig.tight_layout()
    fig.savefig(out_png, dpi=100)
    plt.close(fig)
    return out_png


# ═════════════════════════════ 10. CLI ═════════════════════════════
def main(argv=None):
    import trimesh
    ap = argparse.ArgumentParser(description="局部精修：只在 clean_check 报出的问题附近的小盒子里削薄料、抹尖刺碎边")
    ap.add_argument("stl")
    ap.add_argument("--check", required=True, help="clean_check JSON（整份 files[] 或 --child 单文件输出）")
    ap.add_argument("--out", required=True)
    ap.add_argument("--report", default="")
    ap.add_argument("--protect", default="", help="额外保护体 JSON：[{a,b,d} | {lo,hi}]（世界系）")
    ap.add_argument("--label", default="", help="件号（L01 …）；缺省按 placed 名反查")
    ap.add_argument("--placed", default="", help="placed 名（yaw2roll …）；缺省按件号查")
    ap.add_argument("--strict-protect", action="store_true", help="按任务书原文：盒碰到保护区就整盒拒修")
    ap.add_argument("--unprotect", default="", help="逗号分隔的保护体 id（或前缀，如 geo#19、L01-F05），主设计认定不需要护的（例如确认是让位的 none 孔）")
    ap.add_argument("--no-selfcheck", action="store_true")
    ap.add_argument("--png", default="", help="对比图（缺省 = out 同名 .png）")
    ap.add_argument("--param", action="append", default=[], help="覆盖参数 K=V")
    a = ap.parse_args(argv)
    t0 = time.time()
    P = {}
    for kv in a.param:
        k, v = kv.split("=", 1)
        P[k] = type(PARAMS[k])(v) if k in PARAMS and not isinstance(PARAMS[k], bool) else (v.lower() in ("1", "true", "yes") if isinstance(PARAMS.get(k), bool) else float(v))
    if a.strict_protect:
        P["STRICT_PROTECT"] = True
    placed = a.placed or (BODY[a.label][1] if a.label in BODY else "")
    label = a.label or PLACED2PID.get(placed, "") or os.path.basename(a.stl)[:-4]
    if not placed:
        b = os.path.basename(a.stl)[:-4]
        placed = b if b in PLACED2PID else ""
    check = json.load(open(a.check, encoding="utf-8"))
    entry = pick_entry(check, label, a.stl)
    findings = load_findings(entry)
    mesh = trimesh.load(a.stl, force="mesh", process=True)
    input_note = None
    if not mesh.is_watertight:
        # trimesh 合并顶点后判不水密，常见是两片在一条棱上相切（边被 4 个面共用，09-28 腿部新 L05）。manifold3d 能读成单体、体积一致就用它的网格
        ok = False
        try:
            man0 = to_manifold(mesh.vertices, mesh.faces)
            m2 = from_manifold(man0)
            if m2.is_watertight and len(man0.decompose()) == 1 and abs(man0.volume() - abs(float(mesh.volume))) <= 1e-3 * abs(float(mesh.volume)):
                input_note = "输入 trimesh 判不水密（非流形棱），manifold3d 读成单体、体积一致 → 用 manifold 网格继续"
                print("  " + input_note, flush=True)
                mesh, ok = m2, True
        except ValueError:
            pass
        if not ok:
            print("输入不水密、manifold3d 也读不成单体 —— 精修要求水密单体输入，拒绝。", file=sys.stderr)
            sys.exit(2)
    if mesh.volume < 0:
        mesh.invert()
    extra = json.load(open(a.protect, encoding="utf-8")) if a.protect else None
    print(f"[{label} / {placed}] {len(findings)} 条 finding（thin {sum(f['cat'] == 'thin' for f in findings)}，frag {sum(f['cat'] == 'frag' for f in findings)}，"
          f"holes {sum(f['cat'] == 'holes' for f in findings)}）", flush=True)
    t1 = time.time()
    zones, notes = build_protect(mesh, placed, extra, P)
    if a.unprotect:
        keys = [k.strip() for k in a.unprotect.split(",") if k.strip()]
        drop = [z for z in zones if any(z["id"] == k or z["id"].startswith(k) for k in keys)]
        zones = [z for z in zones if z not in drop]
        notes.append(f"--unprotect 去掉 {len(drop)} 个保护体：{[z['id'] for z in drop]}")
    t_prot = time.time() - t1
    print(f"  保护体 {len(zones)} 个（{t_prot:.1f}s）", flush=True)
    for n_ in notes:
        print("   ·", n_[:160])
    # ── 多轮：第 1 轮修 clean_check 报的；之后每轮只修上一轮在盒里新冒出来的（接缝台阶等），最多 PASSES 轮 ──
    cj = a.out[:-4] + "_clean.json"
    orig_fix = [f for f in findings if f["cat"] != "holes"]
    os.makedirs(os.path.dirname(os.path.abspath(a.out)), exist_ok=True)
    npass = 1 if a.no_selfcheck else int(P.get("PASSES", PARAMS["PASSES"]))

    def run_rounds(work0):
        passes_, cur_, work, keep_extra, after_, best = [], mesh, work0, [], None, None
        for ip in range(1, npass + 1):
            print(f"  ── 第 {ip} 轮：{len(work)} 条", flush=True)
            new_, rp_i = finish(cur_, work, zones, P, keep_out_extra=keep_extra, box_prefix=f"P{ip}.",
                                prev_boxes=[b_ for r_ in passes_ for b_ in r_["boxes"] if b_.get("status") == "fixed"])
            passes_.append(rp_i)
            new_.export(a.out)
            cur_ = new_
            if a.no_selfcheck:
                break
            try:
                after_ = run_clean_check(a.out, label, placed, cj)
            except Exception as e:                                      # noqa: BLE001
                print("  自检失败：", e); after_ = None
                break
            aft_ = load_findings(after_)
            cmp_ = compare(orig_fix, [f for f in aft_ if f["cat"] != "holes"], dict(boxes=[b_ for r_ in passes_ for b_ in r_["boxes"]]))
            newids = {x["id"] for x in cmp_["new_in_box"] if not (P.get("SEAM_RAGGED_OK", PARAMS["SEAM_RAGGED_OK"]) and x["kind"] == "ragged")}
            nhard = sum(1 for f in aft_ if f["cat"] != "holes" and is_hard(f))
            nflag = (1000 * nhard if P.get("SEAM_RAGGED_OK", PARAMS["SEAM_RAGGED_OK"]) else 0) + sum(1 for f in aft_ if f["cat"] != "holes") + 100 * len(cmp_["new_outside"])
            print(f"     自检：原 finding 消失 {len(cmp_['gone'])}、残留 {len(cmp_['remaining'])}；盒内新增 {len(newids)}，盒外新增 {len(cmp_['new_outside'])}"
                  f"（thin+frag 共 {sum(1 for f in aft_ if f['cat'] != 'holes')} 条）", flush=True)
            if best is None or nflag < best[0]:                        # 记住最好的一轮（后面的轮次不保证单调变好）
                best = (nflag, ip, cur_, after_, cmp_)
            if not newids or ip == npass:
                break
            keep_extra += rp_i.get("keep_out", [])
            work = [f for f in aft_ if f["id"] in newids]
        if best is None:
            return passes_, cur_, after_, None
        if best[1] != len(passes_):                                     # 最后一轮不是最好的 → 退回最好的那一轮
            print(f"  取第 {best[1]} 轮的结果（之后的轮次 finding 没更少）", flush=True)
            best[2].export(a.out)
            json.dump(best[3], open(cj, "w", encoding="utf-8"), ensure_ascii=False)
        return passes_[:best[1]], best[2], best[3], best[4]

    # ── 不许变坏：修完还有新增 finding（盒面接缝碎边），或者某盒的目标一条都没消掉 → 该盒整盒回退（finding 记拒修），从原件重来，最多 2 次 ──
    rolled = {}
    natt = int(P.get("ROLLBACK_ATTEMPTS", PARAMS["ROLLBACK_ATTEMPTS"]))
    for attempt in range(natt):
        work0 = [f for f in findings if f["id"] not in rolled]
        passes, cur, after, cmpf = run_rounds(work0)
        if after is None or cmpf is None:
            break
        allnew = cmpf["new_in_box"] + cmpf["new_outside"]
        p1 = [b_ for b_ in passes[0]["boxes"] if b_.get("status") == "fixed"]
        remain = {x["id"] for x in cmpf["remaining"]}
        useless = [b_ for b_ in p1 if b_["findings"] and all(i in remain for i in b_["findings"])]   # 修了但目标一条没消掉
        near_of = lambda x: [b_ for b_ in p1 if np.all(np.asarray(x["c"], float) >= np.asarray(b_["lo"]) - 1.5)
                             and np.all(np.asarray(x["c"], float) <= np.asarray(b_["hi"]) + 1.5)]
        if P.get("SEAM_RAGGED_OK", PARAMS["SEAM_RAGGED_OK"]):
            bad = [x for x in allnew if x["kind"] != "ragged"]                 # 硬线：新增薄膜 / 刀片 / 尖刺 / 碎体
            # 两级规则：消掉硬类的盒允许盒缝新增 ragged；只修碎边的盒只要盒缝新增 ragged 就回退
            fmap0 = {f["id"]: f for f in findings}
            for b_ in p1:
                if b_ in useless or any(is_hard(fmap0[i]) for i in b_["findings"] if i in fmap0):
                    continue
                new_n = sum(1 for x in allnew if x["kind"] == "ragged" and b_ in near_of(x))
                if new_n > 0:                                               # 只修碎边的盒：盒缝不许新增 ragged
                    useless.append(b_)
        else:
            bad = allnew
        if (not bad and not useless) or attempt == natt - 1:
            break
        culprits = set()
        for b_ in useless:
            for fid in b_["findings"]:
                if fid not in rolled:
                    rolled[fid] = f"修了没消掉（或只修碎边、盒缝新增的碎边不少于消掉的），整盒回退、不动这块（第 {attempt + 1} 次）"
            culprits.update(b_["findings"])
        for x in bad:
            c = np.asarray(x["c"], float)
            near = near_of(x)
            if not near and p1:
                near = [min(p1, key=lambda b_: float(np.linalg.norm(c - (np.asarray(b_["lo"]) + np.asarray(b_["hi"])) / 2)))]
            for b_ in near:
                culprits.update(b_["findings"])
        newc = culprits - set(rolled)
        if not culprits:
            break
        for fid in newc:
            rolled[fid] = (f"修了会在盒附近新增薄膜 / 刀片 / 尖刺（{len(bad)} 条），整盒回退（第 {attempt + 1} 次）" if P.get("SEAM_RAGGED_OK", PARAMS["SEAM_RAGGED_OK"])
                           else f"修了会在盒面接缝出新碎边 / 尖刺（{len(bad)} 条，最多 {npass} 轮修不掉），整盒回退（第 {attempt + 1} 次）")
        print(f"  ── 回退 {sorted(culprits)}，从原件重来", flush=True)
    # ── 汇总（以原件、原 finding 为准）──
    frows = [r_ for r_ in passes[0]["findings"]]
    have = {r_["id"] for r_ in frows}
    for f in findings:
        if f["id"] in rolled:
            if f["id"] in have:
                for r_ in frows:
                    if r_["id"] == f["id"]:
                        r_.update(status="refused", reason=rolled[f["id"]])
            else:
                frows.append(dict(id=f["id"], cat=f["cat"], kind=f["kind"], c=[round(v, 2) for v in f["c"]], status="refused", reason=rolled[f["id"]]))
    rep = dict(passes=passes, findings=frows, boxes=[b_ for r_ in passes for b_ in r_["boxes"]], rolled_back=rolled,
               params=passes[0]["params"], radii=passes[0]["radii"], keep_out=passes[0].get("keep_out", []))
    rep.update(tool="tools/cad/finish_pass.py", tool_sha16=sha16(os.path.abspath(__file__)), input=os.path.relpath(os.path.abspath(a.stl), ROOT),
               input_sha16=sha16(a.stl), input_note=input_note, check=os.path.relpath(os.path.abspath(a.check), ROOT), label=label, placed=placed,
               protect_zones=zones, protect_notes=notes, protect_seconds=round(t_prot, 1),
               clean_check_sha16=sha16(os.path.join(HERE, "clean_check.py")), clean_check_holes_sha16=sha16(os.path.join(HERE, "clean_check_holes.py")))
    V0 = float(mesh.volume); V1 = float(cur.volume)
    rep.update(volume_before=round(V0, 3), volume_after=round(V1, 3), dV_mm3=round(V1 - V0, 3), dV_rel=round((V1 - V0) / V0, 6),
               n_bodies=len(cur.split(only_watertight=False)), watertight=bool(cur.is_watertight),
               faces_before=int(len(mesh.faces)), faces_after=int(len(cur.faces)), seconds=round(sum(r_["seconds"] for r_ in passes), 1),
               output=os.path.relpath(os.path.abspath(a.out), ROOT))
    new = cur
    print(f"  写出 {a.out}：体积 {rep['volume_before']:.2f} → {rep['volume_after']:.2f}（{rep['dV_mm3']:+.3f} mm³，{rep['dV_rel']:+.4%}），"
          f"单体 {rep['n_bodies'] == 1}，水密 {rep['watertight']}，面 {rep['faces_before']} → {rep['faces_after']}，精修 {rep['seconds']}s（{len(passes)} 轮）", flush=True)
    rep["diff"] = diff_audit(mesh, new, rep, zones)
    print(f"  布尔差：削 {rep['diff']['removed_mm3']} 补 {rep['diff']['added_mm3']} mm³；盒外 {rep['diff']['diff_outside_boxes_mm3']}，"
          f"保护体内 {rep['diff']['diff_in_protect_mm3']} mm³；表面采样最大偏差：盒外 {rep['diff'].get('max_dev_outside_boxes_mm')} mm，"
          f"保护体内 {rep['diff'].get('max_dev_in_protect_mm')} mm", flush=True)
    if after is not None:
        aft = load_findings(after)
        rep["selfcheck"] = dict(clean_json=os.path.relpath(os.path.abspath(cj), ROOT), status=after.get("status"), seconds=after.get("_wall_s"),
                                before_counts={c: sum(f["cat"] == c for f in findings) for c in ("thin", "frag", "holes")},
                                after_counts={c: sum(f["cat"] == c for f in aft) for c in ("thin", "frag", "holes")},
                                **compare(orig_fix, [f for f in aft if f["cat"] != "holes"], rep))
        hb = [f for f in findings if f["cat"] == "holes"]; ha = [f for f in aft if f["cat"] == "holes"]
        rep["selfcheck"]["holes_new"] = [g["id"] for g in ha if not any(_box_overlap(g["lo"], g["hi"], f["lo"], f["hi"], 0.5) for f in hb)]
        newall = rep["selfcheck"]["new_in_box"] + rep["selfcheck"]["new_outside"]
        rep["selfcheck"]["new_hard"] = [x for x in newall if x["kind"] != "ragged"]
        rep["selfcheck"]["seam_ragged_added"] = [dict(id=x["id"], c=x["c"], boxes=x.get("boxes"), n_short=x["detail"].get("n_short"),
                                                      slab_mm=x["detail"].get("slab_mm")) for x in newall if x["kind"] == "ragged"]
        gone = set(rep["selfcheck"]["gone"])
        for r in rep["findings"]:
            if r["status"] in ("fixed_pending_check", "failed") and r.get("box"):
                r["status"] = "fixed" if r["id"] in gone else ("remaining" if r["status"] == "fixed_pending_check" else "failed")
        sc = rep["selfcheck"]
        print(f"  自检 clean_check：{sc['status']}；thin/frag/holes {sc['before_counts']} → {sc['after_counts']}；消失 {len(sc['gone'])}，残留 {len(sc['remaining'])}，"
              f"盒内新增 {len(sc['new_in_box'])}，盒外新增 {len(sc['new_outside'])}，孔类新增 {len(sc['holes_new'])}；"
              f"其中新增硬类 {len(sc['new_hard'])}、盒缝碎边 {len(sc['seam_ragged_added'])}", flush=True)
        for x in sc["seam_ragged_added"]:
            print(f"     盒缝新增碎边 {x['id']} @ {x['c']}（盒 {x['boxes']}，短棱 {x['n_short']}，起伏 {x['slab_mm']} mm）", flush=True)
    png = a.png or a.out[:-4] + ".png"
    try:
        plot_compare(mesh, new, rep, findings, png, label)
        rep["png"] = os.path.relpath(os.path.abspath(png), ROOT)
    except Exception as e:                                              # noqa: BLE001
        rep["png_error"] = repr(e)
    rep["wall_seconds"] = round(time.time() - t0, 1)
    rep["peak_rss_gb"] = round(rss_gb(), 2)
    rp = a.report or a.out[:-4] + "_report.json"
    json.dump(rep, open(rp, "w", encoding="utf-8"), ensure_ascii=False, indent=1, default=lambda o: o.tolist() if hasattr(o, "tolist") else str(o))
    print(f"  报告 {rp}（总 {rep['wall_seconds']}s，峰值 RSS {rep['peak_rss_gb']} GB）")
    st = [r["status"] for r in rep["findings"]]
    print("  finding 结果：" + "，".join(f"{k} {st.count(k)}" for k in sorted(set(st))))
    return rep


# ═════════════════════════════ 11. 完整 chain 批处理入口：--apply-cad ═════════════════════════════
def _mirror_y(m):
    """与 duckstructure/lib.mirror_y 相同（y → −y）；build 的右件 = mirror_y(左件世界网格)，只写 placed/<名>_R.stl，没有单独的导出文件。"""
    M_ = np.eye(4); M_[1, 1] = -1
    mm = m.copy(); mm.apply_transform(M_)
    return mm


def _f32_clean(m):
    """STL 是 float32：先把顶点按 float32 焊合、删退化面、重建 manifold（不行就先 simplify(1e-5) 收掉亚微米短边再焊）→ 写出再读回拓扑不变。
    与 duckstructure/lib.clean_print_topology 同法（L04 导出一直这么做）。返回 trimesh；两条路都失败抛 ValueError。"""
    import trimesh

    def weld(vv, ff):
        v, inv = np.unique(np.asarray(vv, dtype=np.float32).astype(np.float64), axis=0, return_inverse=True)
        f = np.asarray(inv).reshape(-1)[np.asarray(ff)]
        f = f[(f[:, 0] != f[:, 1]) & (f[:, 1] != f[:, 2]) & (f[:, 0] != f[:, 2])]
        return to_manifold(v, f)
    try:
        man = weld(m.vertices, m.faces)
    except ValueError:
        man0 = to_manifold(m.vertices, m.faces).simplify(1e-5)
        g0 = man0.to_mesh64()
        man = weld(np.asarray(g0.vert_properties)[:, :3], np.asarray(g0.tri_verts))
    nb0 = len(m.split(only_watertight=False))
    if len(man.decompose()) != nb0:
        raise ValueError(f"float32 焊合后体数变了 {nb0} → {len(man.decompose())}")
    out = from_manifold(man)
    if abs(float(out.volume) - float(m.volume)) > 1e-6 * abs(float(m.volume)) + 1e-3:
        raise ValueError(f"float32 焊合后体积变了 {float(m.volume):.4f} → {float(out.volume):.4f}")
    return out


def _same_shape(a, b, tol=1e-3):
    va, vb = abs(float(a.volume)), abs(float(b.volume))
    return abs(va - vb) <= tol * max(va, 1.0) and float(np.abs(np.asarray(a.bounds) - np.asarray(b.bounds)).max()) <= 1e-3


def apply_cad_main(argv):
    """--apply-cad L01,T02,... [--cad-root DIR]：对每件读 <root>/placed/<placed>.stl（世界系）→ clean_check 子进程拿 findings → 精修 + 自检 →
    通过验收才写回 placed/<placed>.stl、placed/<placed>_R.stl（有右件时 = mirror_y，与 build 同法）和导出系 <root>/<件号>_*.stl（= inv(TW(body))·世界，
    与 duckstructure/checks.export 同法）。验收不过 / 导出系对不上 / 输入不水密 → 该件原样不动，记进 <root>/_finish_work/apply_summary.json。"""
    import trimesh, glob as _glob, shutil
    ap = argparse.ArgumentParser(description="finish_pass 批处理：精修 cad 下的件并写回 placed / 导出文件")
    ap.add_argument("--apply-cad", required=True, help="件号，逗号分隔（左件；右件随左件镜像）")
    ap.add_argument("--cad-root", default=os.path.join(ROOT, "cad", "duck_s288"))
    ap.add_argument("--work", default="", help="中间文件目录（缺省 <cad-root>/_finish_work）")
    ap.add_argument("--unprotect", default="")
    ap.add_argument("--strict-protect", action="store_true")
    ap.add_argument("--param", action="append", default=[])
    ap.add_argument("--dry-run", action="store_true", help="只精修 + 验收，不写回")
    a = ap.parse_args(argv)
    root = os.path.abspath(a.cad_root)
    work = os.path.abspath(a.work or os.path.join(root, "_finish_work"))
    os.makedirs(work, exist_ok=True)
    if ROOT not in sys.path:
        sys.path.insert(0, ROOT)
    from duckstructure.lib import TW
    real = os.path.abspath(os.path.join(ROOT, "cad", "duck_s288"))
    print(f"cad-root = {root}{'（真 cad！）' if root == real else ''}；work = {work}", flush=True)
    summary = dict(tool="tools/cad/finish_pass.py --apply-cad", tool_sha16=sha16(os.path.abspath(__file__)), cad_root=root,
                   clean_check_sha16=sha16(os.path.join(HERE, "clean_check.py")), started=time.strftime("%Y-%m-%d %H:%M:%S"), parts=[])
    pass_args = sum((["--param", kv] for kv in a.param), [])
    P_ = dict(PARAMS)
    for kv in a.param:
        k_, v_ = kv.split("=", 1)
        if k_ in P_ and isinstance(P_[k_], bool):
            P_[k_] = v_.lower() in ("1", "true", "yes")
    if a.unprotect:
        pass_args += ["--unprotect", a.unprotect]
    if a.strict_protect:
        pass_args += ["--strict-protect"]
    for pid in [x.strip() for x in a.apply_cad.split(",") if x.strip()]:
        t0 = time.time()
        rec = dict(pid=pid)
        summary["parts"].append(rec)
        try:
            if pid not in BODY:
                raise ValueError(f"不认识的件号 {pid}")
            body, placed = BODY[pid]
            pl = os.path.join(root, "placed", placed + ".stl")
            plR = os.path.join(root, "placed", placed + "_R.stl")
            ex = sorted(_glob.glob(os.path.join(root, f"{pid}_*.stl")))
            rec.update(body=body, placed=placed, placed_file=pl, right_file=plR if os.path.exists(plR) else None, export_file=ex[0] if len(ex) == 1 else ex)
            if not os.path.exists(pl) or len(ex) != 1:
                raise ValueError(f"找不到 placed 文件或导出文件不唯一：{pl} / {ex}")
            W = trimesh.load(pl, force="mesh", process=True)
            T = np.asarray(TW(body), float)
            loc = W.copy(); loc.apply_transform(np.linalg.inv(T))
            E = trimesh.load(ex[0], force="mesh", process=True)
            if not _same_shape(loc, E):
                raise ValueError(f"导出文件 ≠ inv(TW({body}))·placed（体积 {abs(loc.volume):.2f} vs {abs(E.volume):.2f}，包围盒差 "
                                 f"{float(np.abs(np.asarray(loc.bounds) - np.asarray(E.bounds)).max()):.4f}）—— 不是同一次 build 的，不动")
            if rec["right_file"]:
                R0 = trimesh.load(plR, force="mesh", process=True)
                if not _same_shape(_mirror_y(W), R0):
                    raise ValueError("右件 placed ≠ mirror_y(左件 placed)，镜像口径对不上，不动")
            bj = os.path.join(work, f"{pid}_before_clean.json")
            before = run_clean_check(pl, pid, placed, bj)
            bf = load_findings(before)
            nfix = sum(1 for f in bf if f["cat"] in ("thin", "frag"))
            rec["before_counts"] = {c: sum(f["cat"] == c for f in bf) for c in ("thin", "frag", "holes")}
            if nfix == 0:
                rec.update(status="clean", note="thin / frag 没有 finding，不动")
                continue
            out = os.path.join(work, f"{pid}_out.stl")
            rep = main([pl, "--check", bj, "--label", pid, "--placed", placed, "--out", out, "--report", os.path.join(work, f"{pid}_report.json"),
                        "--png", os.path.join(work, f"{pid}_compare.png")] + pass_args)
            sc = rep.get("selfcheck") or {}
            d = rep.get("diff") or {}
            st = [r_["status"] for r_ in rep["findings"]]
            rec.update(after_counts=sc.get("after_counts"), fixed=st.count("fixed"), refused=st.count("refused"), not_handled=st.count("not_handled"),
                       remaining=st.count("remaining"), new=len(sc.get("new_in_box", [])) + len(sc.get("new_outside", [])), holes_new=len(sc.get("holes_new", [])),
                       new_hard=len(sc.get("new_hard", [])), seam_ragged_added=sc.get("seam_ragged_added", []),
                       dV_mm3=rep.get("dV_mm3"), diff_in_protect_mm3=d.get("diff_in_protect_mm3"), diff_outside_boxes_mm3=d.get("diff_outside_boxes_mm3"),
                       max_dev_in_protect_mm=d.get("max_dev_in_protect_mm"), max_dev_outside_boxes_mm=d.get("max_dev_outside_boxes_mm"),
                       report=os.path.join(work, f"{pid}_report.json"))
            why = []
            if "error" in sc or not sc:
                why.append("自检没跑成")
            if rec["new_hard"] or rec["holes_new"] or (rec["new"] and not P_.get("SEAM_RAGGED_OK")):
                why.append(f"自检新增 {rec['new_hard']} 条薄膜 / 刀片 / 尖刺 / 碎体、{rec['holes_new']} 条孔类（新增共 {rec['new']}）")
            # 有没有改善：硬类（thin / spike / debris）变少，或硬类不变、碎边变少
            aft_f = load_findings(json.load(open(os.path.join(work, f"{pid}_out_clean.json"), encoding="utf-8")))
            hb_, ha_ = sum(1 for f in bf if f["cat"] != "holes" and is_hard(f)), sum(1 for f in aft_f if f["cat"] != "holes" and is_hard(f))
            rb_, ra_ = sum(1 for f in bf if f["kind"] == "ragged"), sum(1 for f in aft_f if f["kind"] == "ragged")
            rec.update(hard_before=hb_, hard_after=ha_, ragged_before=rb_, ragged_after=ra_)
            if not (ha_ < hb_ or (ha_ == hb_ and ra_ < rb_)):
                why.append(f"没改善（硬类 {hb_}→{ha_}，碎边 {rb_}→{ra_}）")
            if (d.get("max_dev_in_protect_mm") or 0) > 1e-4 or (d.get("diff_in_protect_mm3") or 0) > 1e-3:
                why.append(f"保护区有差（表面最大偏差 {d.get('max_dev_in_protect_mm')} mm，布尔差 {d.get('diff_in_protect_mm3')} mm³，{d.get('diff_in_protect_worst_zone')}）")
            if (d.get("max_dev_outside_boxes_mm") or 0) > 1e-4:
                why.append(f"盒外有差（表面最大偏差 {d.get('max_dev_outside_boxes_mm')} mm）")
            if not rep.get("watertight") or rep.get("n_bodies") != 1:
                why.append("输出不是水密单体")
            if abs(rep.get("dV_rel") or 0) > PARAMS["VOL_TOL"]:
                why.append(f"体积变化 {rep.get('dV_rel'):+.3%} 超限")
            if rec["fixed"] == 0:
                why.append("一条都没修掉（全拒修 / 回退）")
            if why:
                rec.update(status="untouched", reason="；".join(why))
                continue
            if a.dry_run:
                rec.update(status="dry_run_ok")
                continue
            NW = trimesh.load(out, force="mesh", process=True)
            # ① 三个输出先在内存里做好：都按 float32 焊合 + manifold 重建（STL 是 float32，焊合后往返拓扑不变）。
            #   09-28 build#2：导出件 = inv(TW)·世界 之后直接写 STL，变换后相距 < float32 分辨率的顶点在读回时并成一点 → H03 导出不水密。
            W_new = _f32_clean(NW)
            L_new = W_new.copy(); L_new.apply_transform(np.linalg.inv(T)); L_new = _f32_clean(L_new)
            # hr50_r3 2026-09-28：placed 与导出件必须是同一份拓扑 —— Gate L3/L7 用 (面数, |体积|) 指纹把导出件配到 placed 实体；build#3 的 H03 导出
            #   在局部系焊合时掉了 2 个退化面（336104 ≠ placed 336106）→ L3/H03、L7/H03 placed_instance 红、L7 反例 4 个失败。
            #   所以世界件 = TW·焊合后的导出件，右件 = mirror_y(世界件)（r5：不再焊合，保面序）；三者面数不同就拒绝写回（下面读回时再核一次）。
            W_new = L_new.copy(); W_new.apply_transform(T)
            outs = [(pl, W_new, "placed"), (ex[0], L_new, "export")]
            if rec["right_file"]:
                R_new = _mirror_y(W_new)      # r5（09-28 17:5x）：不再 _f32_clean(R_new)——manifold 重建会把三角面重排序，而 print_file_check
                if R_new.volume < 0:          #   按"面心一一对应"把打印件（= 导出件镜像，trimesh 保序）对 placed_R，面序一变残差 32 mm（L05_R 实测，几何其实相同）。
                    R_new.invert()            #   W_new 已经 f32 焊合，y→−y 在 float32 里精确，镜像后拓扑/面数不变，不需要再焊；面数检查照旧。
                if len(R_new.faces) != len(L_new.faces):
                    raise ValueError(f"右件焊合后面数 {len(R_new.faces)} ≠ 导出件 {len(L_new.faces)}，Gate 指纹会对不上，不动")
                outs.append((plR, R_new, "placed_R"))
            dV_world = float(W_new.volume) - float(abs(W.volume))
            rec["dV_world_mm3"] = round(dV_world, 4)
            # ② 写临时文件（同目录、不带 .stl 后缀，别的工具 glob 不到）→ 读回核对；任何一项不过 → 删临时文件，原件一个字节都不动
            tmps = []
            try:
                for path, m_, kind in outs:
                    tmp = os.path.join(os.path.dirname(path), "." + os.path.basename(path) + ".finish_tmp")
                    m_.export(tmp, file_type="stl")
                    tmps.append((tmp, path, kind))
                    g = trimesh.load(tmp, file_type="stl", force="mesh", process=True)
                    o = trimesh.load(path, force="mesh", process=True)
                    nb_g, nb_o = len(g.split(only_watertight=False)), len(o.split(only_watertight=False))
                    chk = dict(kind=kind, watertight=bool(g.is_watertight), bodies=nb_g, orig_watertight=bool(o.is_watertight), orig_bodies=nb_o,
                               volume=round(float(g.volume), 4), orig_volume=round(float(o.volume), 4), faces=int(len(g.faces)))
                    rec.setdefault("readback", {})[os.path.basename(path)] = chk
                    if kind in ("placed", "placed_R"):
                        # 水密单体；原件本来就不水密的（L05 build 件有一条 4 面共边的相切棱，trimesh 判不水密、manifold 读成单体）→ 不比原件差：
                        # 体数相同、原件不水密。两种情况都要求 体积差 = 精修体积差 ±(1e-6·V + 1e-3)
                        if not (g.is_watertight and nb_g == 1):
                            if o.is_watertight or nb_g != nb_o:
                                raise ValueError(f"{kind} 读回不是水密单体（原件是）/ 体数变了：{os.path.basename(path)} {chk}")
                        _tol = 1e-6 * abs(float(o.volume)) + 1e-3   # r4（09-28 17:3x）：放置变换/焊接的浮点舍入在 4 万 mm³ 件上有 0.003 量级（J01/H03 实测），绝对 1e-3 太死；相对 1e-6 仍能抓住任何真实几何差（最小一条精修 ≥0.01 mm³）
                        if abs((float(g.volume) - float(o.volume)) - dV_world) > _tol:
                            raise ValueError(f"{kind} 体积差 {float(g.volume) - float(o.volume):+.4f} ≠ 精修体积差 {dV_world:+.4f} ±{_tol:.4f}：{os.path.basename(path)}")
                    else:                                                  # 导出件：不比原件差
                        # 读回是水密单体 = 最好的情况，直接算过（原件不水密时 trimesh 按面连通会把它拆成 2 块，L07 实测，这时"体数相同"反而拒掉了变好）；
                        # 否则：体数与原件相同、原件水密则须水密；两种情况都要求 体积差 = 精修体积差 ±(1e-6·V + 1e-3)
                        good = g.is_watertight and nb_g == 1
                        if not good:
                            if nb_g != nb_o:
                                raise ValueError(f"导出件读回体数 {nb_g} ≠ 原件 {nb_o}：{os.path.basename(path)}")
                            if o.is_watertight and not g.is_watertight:
                                raise ValueError(f"导出件原件水密、写出的读回不水密：{os.path.basename(path)}")
                        _tol = 1e-6 * abs(float(o.volume)) + 1e-3   # r4（09-28 17:3x）：放置变换/焊接的浮点舍入在 4 万 mm³ 件上有 0.003 量级（J01/H03 实测），绝对 1e-3 太死；相对 1e-6 仍能抓住任何真实几何差（最小一条精修 ≥0.01 mm³）
                        if abs((float(g.volume) - float(o.volume)) - dV_world) > _tol:
                            raise ValueError(f"导出件体积差 {float(g.volume) - float(o.volume):+.4f} ≠ 精修体积差 {dV_world:+.4f} ±{_tol:.4f}：{os.path.basename(path)}")
                # hr50_r3：三份文件读回面数必须一致（Gate L3/L7 指纹 = 面数 + |体积|），否则一律不写
                nf = {k: v["faces"] for k, v in rec["readback"].items()}
                if len(set(nf.values())) != 1:
                    raise ValueError(f"写出读回面数不一致（Gate 指纹会对不上）：{nf}")
            except Exception:
                for tmp, _, _ in tmps:
                    if os.path.exists(tmp):
                        os.remove(tmp)
                rec["rolled_back"] = False
                raise
            # ③ 备份原件 → 逐个原子替换（os.replace）；替换中途任何异常 → 已替换的全部从备份复原，记 rolled_back=true
            bak = os.path.join(work, "backup"); os.makedirs(bak, exist_ok=True)
            bak_sha = {}
            for _, path, _ in tmps:
                shutil.copy2(path, os.path.join(bak, os.path.basename(path)))
                bak_sha[path] = sha16(os.path.join(bak, os.path.basename(path)))
                if bak_sha[path] != sha16(path):
                    raise RuntimeError(f"备份核对不一致：{path}")
            replaced = []
            try:
                for tmp, path, kind in tmps:
                    want = sha16(tmp)
                    os.replace(tmp, path)
                    replaced.append(path)
                    if sha16(path) != want:
                        raise RuntimeError(f"替换后 sha 不对：{path}")
                    if os.environ.get("FINISH_APPLY_INJECT") == pid:          # 测试钩子：第一件替换后故意失败，验回滚
                        raise RuntimeError(f"FINISH_APPLY_INJECT={pid}：故意在替换 {os.path.basename(path)} 之后失败（回滚测试）")
            except Exception as e:
                restored = []
                for path in replaced:
                    shutil.copy2(os.path.join(bak, os.path.basename(path)), path)
                    restored.append(dict(file=os.path.basename(path), sha_ok=sha16(path) == bak_sha[path]))
                for tmp, _, _ in tmps:
                    if os.path.exists(tmp):
                        os.remove(tmp)
                rec.update(rolled_back=True, restored=restored)
                raise RuntimeError(f"写回中途失败，已从备份复原 {len(restored)} 个文件（sha 全对 = {all(r_['sha_ok'] for r_ in restored)}）：{e}")
            wrote = [path for _, path, _ in tmps]
            seams = rec.get("seam_ragged_added") or []
            rec.update(status="applied_with_seams" if seams else "applied", rolled_back=False, wrote=wrote, backup_dir=bak,
                       sha16={os.path.basename(f_): sha16(f_) for f_ in wrote})
            for x in seams:
                print(f"[apply] {pid}: 盒缝碎边 @ {x.get('c')} slab {x.get('slab_mm')} mm（盒 {x.get('boxes')}）", flush=True)
        except SystemExit as e:                                         # main() 对不水密输入 sys.exit(2)
            rec.update(status="untouched", reason=f"输入不合格（exit {e.code}）")
        except Exception as e:                                          # noqa: BLE001
            rec.update(status="untouched", reason=f"{type(e).__name__}: {e}")
        finally:
            rec["seconds"] = round(time.time() - t0, 1)
            rec.setdefault("rolled_back", False)
            print(f"[apply] {pid}: {rec.get('status')}  rolled_back={rec.get('rolled_back')}  {rec.get('reason', '')[:140]}", flush=True)
            json.dump(summary, open(os.path.join(work, "apply_summary.json"), "w", encoding="utf-8"), ensure_ascii=False, indent=1,
                      default=lambda o: o.tolist() if hasattr(o, "tolist") else str(o))
    summary["seam_ragged_added"] = [dict(pid=r_["pid"], c=x.get("c"), slab_mm=x.get("slab_mm"), n_short=x.get("n_short"), box=x.get("boxes"))
                                    for r_ in summary["parts"] if r_.get("status") == "applied_with_seams" for x in (r_.get("seam_ragged_added") or [])]
    summary["finished"] = time.strftime("%Y-%m-%d %H:%M:%S")
    json.dump(summary, open(os.path.join(work, "apply_summary.json"), "w", encoding="utf-8"), ensure_ascii=False, indent=1,
              default=lambda o: o.tolist() if hasattr(o, "tolist") else str(o))
    print("汇总：" + "，".join(f"{r_['pid']} {r_.get('status')}" for r_ in summary["parts"]) + f"  → {os.path.join(work, 'apply_summary.json')}")
    return summary


if __name__ == "__main__":
    if any(x == "--apply-cad" or x.startswith("--apply-cad=") for x in sys.argv[1:]):
        apply_cad_main(sys.argv[1:])
    else:
        main()
