#!/usr/bin/env python3
"""第 0 层 网格与坐标 —— 这文件是不是一个合法实体、在对的坐标系里。

判据（tolerances.yaml:feature_check_tolerances）：
  洞边=0 · 独立实体=1 · 退化面=0 · 非流形边=0 · 尺寸量级像 mm 不像 m · 基准落位 vs MJCF
不使用 trimesh 的 is_watertight（默认 merge_vertices 会制造假的非闭合，见 README 坑 11）。
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from core import (LayerResult, PASS, FAIL, BLOCK, WARN, INFO, num, ROOT,   # noqa: E402
                  sha256_file)

LAYER = 0
NAME = "网格与坐标"


def _stats(p: Path):
    from duckstructure.lib import stl_stats
    return stl_stats(str(p))          # (三角面数, 退化面, 洞边, 非流形边)


def _nonmanifold_vertices(p: Path):
    """非流形顶点（夹点 / bow-tie）：一个顶点的相邻面扇不连通 = 两片曲面只在一点相接。
    hole_edges / nonmanifold_edges 都看不见它（hr32 H03 让位刀在 (41.7, 8.1, 220.6) 留了 1 个，Euler 数变奇数，复审 m-01）。
    顶点按 stl_stats 同款精确 float32 去重；每个顶点上把共边的相邻面并查集，块数 >1 即夹点。返回 (个数, 前 3 个坐标)。"""
    import struct
    import numpy as np
    from collections import defaultdict
    with open(p, "rb") as f:
        n = struct.unpack("<I", f.read(84)[80:84])[0]
        data = f.read(50 * n)
    tris = np.frombuffer(data, dtype=np.dtype([("n", "<3f4"), ("v", "<3,3f4"), ("a", "<u2")]), count=n)
    V = tris["v"].reshape(-1, 3)
    uniq, inv = np.unique(V, axis=0, return_inverse=True)
    F = inv.reshape(-1, 3)
    faces_of = defaultdict(list)                  # 顶点 → 相邻面
    for fi, (a, b, c) in enumerate(F):
        faces_of[a].append(fi); faces_of[b].append(fi); faces_of[c].append(fi)
    bad = []
    for v, fs in faces_of.items():
        if len(fs) < 4:
            continue                               # 3 个面以内不可能分成两扇
        # 该顶点上的边（v, u）→ 用到它的面；共边的面连通
        by_edge = defaultdict(list)
        for fi in fs:
            for u in F[fi]:
                if u != v: by_edge[u].append(fi)
        parent = {fi: fi for fi in fs}
        def find(x):
            while parent[x] != x:
                parent[x] = parent[parent[x]]; x = parent[x]
            return x
        for lst in by_edge.values():
            r0 = find(lst[0])
            for fi in lst[1:]:
                r = find(fi)
                if r != r0: parent[r] = r0
        if len({find(fi) for fi in fs}) > 1:
            bad.append([round(float(x), 2) for x in uniq[v]])
    return len(bad), bad[:3]

def _bodies_and_extents(p: Path):
    """连通块数 + 包围盒。**不能用 trimesh.split()** —— process=False 时顶点没合并，
    每个三角面各算一块（B01 会报 922 块）；process=True 又会按容差焊点造出假闭合。
    这里沿用 stl_stats 的精确 float32 去重，再在面-顶点图上并查集。"""
    import struct
    import numpy as np
    with open(p, "rb") as f:
        n = struct.unpack("<I", f.read(84)[80:84])[0]
        data = f.read(50 * n)
    tris = np.frombuffer(data, dtype=np.dtype([("n", "<3f4"), ("v", "<3,3f4"), ("a", "<u2")]), count=n)
    V = tris["v"].reshape(-1, 3)
    uniq, inv = np.unique(V, axis=0, return_inverse=True)
    F = inv.reshape(-1, 3)
    parent = np.arange(len(uniq))

    def find(x):
        while parent[x] != x:
            parent[x] = parent[parent[x]]; x = parent[x]
        return x

    for a, b, c in F:
        ra, rb, rc = find(a), find(b), find(c)
        if ra != rb: parent[rb] = ra
        if ra != rc: parent[rc] = ra
    root_of_face = np.array([find(a) for a, _, _ in F])
    # 逐连通块算有符号体积：正 = 实体，负 = 内部空腔壳（keep_main 故意保留，删了等于把空腔填实）
    tv = tris["v"].astype(np.float64)
    sv = np.einsum("ij,ij->i", tv[:, 0], np.cross(tv[:, 1], tv[:, 2])) / 6.0
    vols = {}
    for r in np.unique(root_of_face):
        vols[int(r)] = float(sv[root_of_face == r].sum())
    ext = uniq.max(axis=0) - uniq.min(axis=0)
    return vols, ext


def run(ctx) -> LayerResult:
    res = LayerResult(LAYER, NAME)
    tol = (ctx.data.get("tolerances") or {}).get("feature_check_tolerances") or {}
    # 量级判据：整鸭最大件不超过 ~300 mm、最小件不小于 ~5 mm。写成范围而不是单值。
    lo_mm, hi_mm = 3.0, 300.0

    for pid in ctx.parts:
        if ctx.only and pid not in ctx.only:
            continue
        stl = ctx.stl(pid)
        if stl is None:
            res.unknown(pid, "stl_exists", f"cad/duck_s288/ 里找不到 {pid} 的导出 STL",
                        provenance="parts.yaml:parts[].id")
            continue
        res.inputs.append(str(stl.relative_to(ROOT)))
        try:
            ntri, deg, holes, nonman = _stats(stl)
        except Exception as e:
            res.unknown(pid, "stl_readable", f"读 STL 失败：{e}", provenance="core:元规则4")
            continue

        res.add(subject=pid, check="hole_edges", state=PASS if holes == 0 else FAIL,
                severity=BLOCK, measured=holes, criterion="洞边 = 0（只被一个面用的边）",
                evidence_n=ntri, detail=f"{ntri} 个三角面",
                provenance="README 第 2 节 L0 / 坑 11")
        res.add(subject=pid, check="degenerate_faces", state=PASS if deg == 0 else FAIL,
                severity=WARN, measured=deg, criterion="退化面 = 0",
                evidence_n=ntri, provenance="README 第 2 节 L0")
        res.add(subject=pid, check="nonmanifold_edges", state=PASS if nonman == 0 else FAIL,
                severity=WARN, measured=nonman, criterion="非流形边 = 0（被 >2 个面用的边）",
                evidence_n=ntri, provenance="README 第 2 节 L0")
        # 2026-09-22 hr32 复审 m-01：非流形顶点是上面三条的盲区（边都合法、Euler 数却是奇数）
        try:
            nmv, where = _nonmanifold_vertices(stl)
            res.add(subject=pid, check="nonmanifold_vertices", state=PASS if nmv == 0 else FAIL,
                    severity=WARN, measured=nmv, criterion="非流形顶点 = 0（顶点的相邻面扇必须连通；夹点会被切片器当零厚度扔掉）",
                    evidence_n=ntri, detail=(f"夹点 {where}（export_local）" if nmv else "全部顶点面扇连通"),
                    provenance="hr32_review.md m-01 / README 第 2 节 L0")
        except Exception as e:
            res.unknown(pid, "nonmanifold_vertices", f"顶点面扇统计失败：{e}")

        try:
            vols, ext = _bodies_and_extents(stl)
        except Exception as e:
            res.unknown(pid, "bodies", f"连通块统计失败：{e}")
            continue
        solids = [v for v in vols.values() if v > 1e-6]
        cavities = [v for v in vols.values() if v < -1e-6]
        slivers = [v for v in vols.values() if abs(v) <= 1e-6]
        res.add(subject=pid, check="single_solid",
                state=PASS if len(solids) == 1 and not slivers else FAIL, severity=BLOCK,
                measured=f"实体{len(solids)} 空腔{len(cavities)} 零体积{len(slivers)}",
                criterion="正体积连通块 = 1；零体积碎片 = 0（内部空腔壳合法，keep_main 故意保留）",
                evidence_n=len(vols),
                detail=("空腔共 %.3f mm³" % -sum(cavities)) if cavities else "",
                provenance="README 第 2 节 L0 / 坑 6")
        mx = float(max(ext))
        ok_scale = lo_mm <= mx <= hi_mm
        res.add(subject=pid, check="units_and_scale", state=PASS if ok_scale else FAIL,
                severity=BLOCK, measured=round(mx, 4),
                criterion=f"最大外形尺寸 ∈ [{lo_mm}, {hi_mm}] mm（抓米/毫米、未 apply transform）",
                evidence_n=3, detail=f"包围盒 {[round(float(v),3) for v in ext]}",
                provenance="README 第 2 节 L0")

    # 基准落位：整机装配包围盒 vs MJCF 骨架，抓整体坐标系错位
    dat_tol, _ = num(tol.get("datum_vs_mjcf_mm"), 0.05)
    ap = ctx.placed("__assembly__") or (ROOT / "cad/duck_s288/assembly_preview.stl")
    if ap.exists():
        try:
            import numpy as np
            mj = ctx.mjcf()
            sites = {}
            for b in mj[0].values() if isinstance(mj, tuple) else []:
                sites.update(b.get("sites") or {})
            _, ext_a = _bodies_and_extents(ap)
            hi, lo = ext_a, np.zeros(3)
            plausible = bool(np.all(ext_a > 50) and np.all(ext_a < 400))
            res.add(subject="_assembly", check="assembly_bbox_scale",
                    state=PASS if plausible else FAIL, severity=BLOCK,
                    measured=[round(float(v), 2) for v in (hi - lo)],
                    criterion="整机包围盒每轴 ∈ (50, 400) mm",
                    evidence_n=1, detail=f"{len(sites)} 个 MJCF site 可用于后续基准比对",
                    provenance="README 第 2 节 L0")
        except Exception as e:
            res.unknown("_assembly", "assembly_bbox_scale", f"整机包围盒检查失败：{e}")
    else:
        res.unknown("_assembly", "assembly_bbox_scale", "assembly_preview.stl 不存在")

    # ── 场景 ⑯：被验证的和被打印的必须是同一套（发布副本 / 冻结基线）
    # Gate 原来只记录"当前文件指纹"，从不核对导出件↔发布副本、也不核对冻结基线的期望值 ——
    # 于是换掉发布 STL 或改动原版件都不会红。审查 FG05。
    rel_dir = ROOT / "V2_S288版发布产物/01_整鸭打印件/S288专用件"
    diff, same, missing = [], 0, []
    for pid in ctx.parts:
        stl = ctx.stl(pid)
        if stl is None:
            continue
        cand = rel_dir / stl.name
        res.inputs.append(str(stl.relative_to(ROOT)))
        if not cand.exists():
            missing.append(stl.name); continue
        res.inputs.append(str(cand.relative_to(ROOT)))
        if sha256_file(stl) == sha256_file(cand):
            same += 1
        else:
            diff.append(pid)
    # 2026-09-21 hr27（复审 F-01 延伸）：发布副本里**多出来**的 STL 也算红 —— 09-17 退役的 L08/L09/L11/L12 留在 cad/duck_s288/，
    # 被同步脚本的通配符抄进 S288专用件/ 躺了 4 天；按目录打印就会多打 4 个早已不在 parts.yaml 里的件。
    part_names = {ctx.stl(p).name for p in ctx.parts if ctx.stl(p) is not None}
    extra = sorted(f.name for f in rel_dir.glob("*.stl") if f.name not in part_names) if rel_dir.exists() else []
    res.add(subject="_release", check="export_vs_release_sha256",
            state=PASS if not diff and not missing and not extra else FAIL, severity=BLOCK,
            measured=len(diff) + len(missing) + len(extra),
            criterion="cad/duck_s288/ 的导出件与 01_整鸭打印件/S288专用件/ 的发布副本必须逐字节相同，且发布副本里不得有 parts.yaml 之外的 STL",
            evidence_n=same + len(diff) + len(missing) + len(extra),
            detail=(f"{same} 件一致" + (f"；不一致 {diff}" if diff else "")
                    + (f"；发布副本缺失 {missing}" if missing else "")
                    + (f"；发布副本多出（不在 parts.yaml）{extra}" if extra else "")
                    + ("　→ 送去打印的不是被检查的那一份" if diff or missing or extra else "")),
            provenance="README 元规则 6（hash 绑定）/ 场景 ⑯")

    # ── 场景 ⑯ 之三（2026-09-21 hr26 复审 F-01）：发布包根目录不能再躺着一套"看起来一样"的 STL。
    # 上面那条只盯 S288专用件/，09-19 起 01_整鸭打印件/ 顶层留着 24 个 hr15 同名旧件两天没人看见 ——
    # 拿错目录就是"送去打印的不是被检查的那一份"。判据：01_整鸭打印件/ 下除 S288专用件/ 与 原版件_原封不动/
    # 之外，任何子目录里出现与 S288专用件 同名的 STL，都必须逐字节相同；根目录本身不得有任何 STL
    # （历史留档只能放进名字带"旧版"或"勿用"的子目录 —— 那种目录里的同名旧件不查；其它子目录里同名不同 sha 算红）。
    rel_root = rel_dir.parent
    stray, stale, seen_root = [], [], 0
    rel_names = {f.name for f in rel_dir.glob("*.stl")} if rel_dir.exists() else set()
    if rel_root.exists():
        for f in sorted(rel_root.glob("*.stl")):
            stray.append(f.name); seen_root += 1
        for d in sorted(p for p in rel_root.iterdir() if p.is_dir() and p.name not in (rel_dir.name, "原版件_原封不动")
                        and "旧版" not in p.name and "勿用" not in p.name):
            for f in sorted(d.glob("*.stl")):
                if f.name in rel_names and sha256_file(f) != sha256_file(rel_dir / f.name):
                    stale.append(f"{d.name}/{f.name}")
    res.add(subject="_release", check="release_root_no_duplicate_stl",
            state=PASS if not stray and not stale else FAIL, severity=BLOCK,
            measured=len(stray) + len(stale),
            criterion="01_整鸭打印件/ 根目录无 STL；其它子目录（原版件_原封不动/ 与名字带 旧版/勿用 的留档目录除外）里与 S288专用件/ 同名的 STL 必须逐字节相同",
            evidence_n=seen_root + len(rel_names),
            detail=(("根目录 STL：" + ", ".join(stray[:6]) + ("…" if len(stray) > 6 else "")) if stray else "根目录无 STL")
                   + (f"；同名不同 sha：{stale[:6]}" if stale else "")
                   + ("　→ 两套同名件并存，拿错目录就打错件" if stray or stale else ""),
            provenance="hr26_review.md F-01 / 场景 ⑯")

    # ── 场景 ⑯ 之二：源码副本。06_CAD源码/duckstructure 是发布包里的**第二份**代码，
    # 手工 cp 维护、没有任何检查看着它。它和 duckstructure/ 一旦分叉，发布包里的
    # "复现方法"就复现不出发布包里的 STL —— 而上面那条 sha256 只比 STL，看不见这件事。
    src_live = ROOT / "duckstructure"
    src_copy = ROOT / "V2_S288版发布产物/06_CAD源码/duckstructure"
    if not src_copy.exists():
        res.unknown("_release", "source_copy_sha256",
                    "06_CAD源码/duckstructure 不存在 —— 发布包没带可复现的源码",
                    severity=BLOCK, provenance="README 第 5 节 DoD『同步 06_CAD源码/』")
    else:
        def _py(d):
            return {f.relative_to(d).as_posix(): f
                    for f in sorted(d.rglob("*.py"))
                    if "__pycache__" not in f.parts}
        live, copy = _py(src_live), _py(src_copy)
        s_bad = sorted(k for k in live.keys() & copy.keys()
                       if sha256_file(live[k]) != sha256_file(copy[k]))
        s_only_live = sorted(live.keys() - copy.keys())
        s_only_copy = sorted(copy.keys() - live.keys())
        for k in live: res.inputs.append(f"duckstructure/{k}")
        n_bad = len(s_bad) + len(s_only_live) + len(s_only_copy)
        res.add(subject="_release", check="source_copy_sha256",
                state=PASS if n_bad == 0 and live else FAIL, severity=BLOCK,
                measured=n_bad,
                criterion="duckstructure/ 与 06_CAD源码/duckstructure/ 的 *.py 必须逐字节相同",
                evidence_n=len(live | copy),
                detail=(f"{len(live) - len(s_bad) - len(s_only_live)}/{len(live)} 一致"
                        + (f"；内容不同 {s_bad}" if s_bad else "")
                        + (f"；副本缺 {s_only_live}" if s_only_live else "")
                        + (f"；副本多出 {s_only_copy}" if s_only_copy else "")
                        + ("　→ 发布包里的源码跑不出发布包里的 STL" if n_bad else "")),
                provenance="README 第 5 节 DoD『同步 06_CAD源码/』/ 场景 ⑯")

    frozen = (ctx.data.get("frozen") or {})
    for key, what in (("original_stl_sha256", "原版件（CC BY-SA-NC，一字不改）"),
                      ("kinematics_sha256", "运动学来源")):
        rows = frozen.get(key) or []
        bad, okn, gone = [], 0, []
        for r in rows:
            f = ROOT / r["file"]
            res.inputs.append(r["file"])
            if not f.exists():
                gone.append(r["file"]); continue
            if sha256_file(f) == r.get("sha256"):
                okn += 1
            else:
                bad.append(r["file"])
        res.add(subject="_frozen", check=f"{key}_matches_baseline",
                state=PASS if not bad and not gone and rows else FAIL, severity=BLOCK,
                measured=len(bad) + len(gone),
                criterion=f"{what}：当前 sha256 必须等于 frozen.yaml 记录的基线",
                evidence_n=len(rows),
                detail=(f"{okn}/{len(rows)} 与基线一致"
                        + (f"；已改动 {bad}" if bad else "") + (f"；文件不存在 {gone}" if gone else "")
                        if rows else "frozen.yaml 里没有这组基线"),
                provenance=f"frozen.yaml:{key}")

    # 基准特征逐件比对 MJCF 还没实现 —— 元规则 4：不许静默跳过
    res.unknown("_datum", "datum_vs_mjcf",
                f"逐件基准特征（舵盘轴心）vs MJCF ±{dat_tol} mm 的比对尚未实现",
                provenance="tolerances.yaml:feature_check_tolerances.datum_vs_mjcf_mm")

    res.evidence = {"parts_checked": len([p for p in ctx.parts if not ctx.only or p in ctx.only]),
                    "criteria_per_part": 5}
    return res
