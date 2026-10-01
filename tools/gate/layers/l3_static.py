#!/usr/bin/env python3
"""第 3 层 静态装配 —— 零位姿下"各就各位吗"。

判据（tolerances.yaml:feature_check_tolerances.static_intersection_mm3）：任意两实体交集 ≤ max。
三类交集：
  A 打印件 × 打印件         —— cad/duck_s288/placed/ 里的世界坐标件
  B 打印件 × 元件**完整实体** —— 不是截面、不是孔径。4 颗 6704 各 ~402 mm³ 的历史事故就是
                              "截面孔径对 ≠ 整颗装得进去"，只查孔径永远看不见 → 这里查整颗环。
  C 打印件 × 禁入体          —— keepouts.yaml 21 条
    （hr41g 2026-09-24：写了 bounds_world_mm 的结构化禁入体由 _ko_bounds_sweep 建；声明 moving_components /
     sweep_* 的是"包络沿方向后抽"的扫掠型（首例 KO22），障碍 = placed/ 全部实体（打印件+元件）− 随行件，
     另判终点脱离 keepout_sweep_clear_at_end；字段缺 → keepout_sweep_declared unknown。反例 n_l3_ko22_sweep）

坑：
  · **不能用 process=False 载 STL 再做布尔**。manifold3d 对未合点的 STL 直接判 Error.NotManifold；
    trimesh 的 manifold engine 更早一步在 is_volume 上就拒绝。这里统一 process=True 合点后再进布尔。
    合点只影响布尔可行性，不参与第 0 层的拓扑判据（那边照旧用精确 float32 去重，见 l0_mesh）。
  · 不用 trimesh.load(...).split() 数连通块，不用 is_watertight（README 坑 11）。
  · 布尔炸了 → 抛异常 → 判红（照 duckstructure/checks.py:vol 与 assembly_audit.intersection 的做法），
    绝不把异常吞成"体积 0 = 通过"。
  · keepouts.yaml 的 frame 字段说明坐标系：servo_local / part_local / world_frozen_*。
    把局部坐标当世界坐标就是"空刀"的来历 —— 凡是需要局部帧的，一律走 duckstructure.lib.sfw / MJCF 关节帧。
"""
from __future__ import annotations
import json
import math
import re
import struct
import sys
from pathlib import Path

_HERE = Path(__file__).resolve()
sys.path.insert(0, str(_HERE.parents[1]))                       # tools/gate → core
for _p in (str(_HERE.parents[3]), str(_HERE.parents[3] / "tools" / "cad")):
    if _p not in sys.path:
        sys.path.insert(0, _p)                                  # 仓库根（duckstructure）+ tools/cad（assembly_audit）

from core import (LayerResult, PASS, FAIL, NOT_RUN, RETIRED, BLOCK, WARN, INFO,      # noqa: E402
                  ROOT, PLACED, STL_DIR)

LAYER = 3
NAME = "静态装配"

_TOL_KEY = "tolerances.yaml:feature_check_tolerances.static_intersection_mm3"
_MECH = STL_DIR / "mechanical_audit.json"


# ── STL 指纹（不建 trimesh 对象，只读文件） ────────────────────────────────
def _stl_sig(p: Path):
    """(三角面数, 有符号体积 mm³)。用来把导出件和 placed 件对上号 —— 体积和面数都是刚体不变量。"""
    import numpy as np
    with open(p, "rb") as f:
        n = struct.unpack("<I", f.read(84)[80:84])[0]
        data = f.read(50 * n)
    tris = np.frombuffer(data, dtype=np.dtype([("n", "<3f4"), ("v", "<3,3f4"), ("a", "<u2")]), count=n)
    v = tris["v"].astype(np.float64)
    vol = float(np.einsum("ij,ij->i", v[:, 0], np.cross(v[:, 1], v[:, 2])).sum() / 6.0)
    return n, vol


# ── 几何底座：合点加载 + manifold3d 布尔 ──────────────────────────────────
class _Geo:
    def __init__(self):
        self._mesh, self._solid = {}, {}
        self.booleans = 0
        self.aabb_skips = 0

    def mesh(self, path):
        k = str(path)
        if k not in self._mesh:
            import trimesh
            self._mesh[k] = trimesh.load(k, process=True)   # 必须合点，理由见模块 docstring
        return self._mesh[k]

    def solid(self, path):
        k = str(path)
        if k not in self._solid:
            from assembly_audit import solid
            self._solid[k] = solid(self.mesh(path))
        return self._solid[k]

    @staticmethod
    def wrap(mesh):
        from assembly_audit import solid
        return solid(mesh)

    def inter(self, a, b):
        """交集体积。AABB 粗筛在 assembly_audit.intersection 里；布尔失败抛 ValueError（不吞）。"""
        import numpy as np
        from assembly_audit import intersection
        aa = np.asarray(a.bounding_box()).reshape(2, 3)
        bb = np.asarray(b.bounding_box()).reshape(2, 3)
        if np.any(aa[1] < bb[0]) or np.any(bb[1] < aa[0]):
            self.aabb_skips += 1
            return 0.0
        self.booleans += 1
        return intersection(a, b)


def _box(size, center):
    import numpy as np
    import trimesh
    m = trimesh.creation.box(extents=np.asarray(size, float))
    m.apply_translation(np.asarray(center, float))
    return m


def _facet_noise_mm3(d, h, n_solids=1, sections=128):
    """外接 `sections` 边形比真圆多出的体积上界：环带面积 π((r/cos(π/n))² − r²) × 长度 × 个数。
    这就是禁入柱体的"刻面噪声"：多算的方向 = 偏保守（只会多报侵入，不会漏报）。写进 criterion 与 keepouts.yaml:facet_noise_mm3，
    **不**按它放宽阈值（F-L3-1 口径：外接 128 边形不改；阈值不动）。"""
    r = d / 2.0
    ro = r / math.cos(math.pi / sections)
    return math.pi * (ro * ro - r * r) * float(h) * int(n_solids)


def _cyl(d, h, center, axis="z", sections=128):
    """禁入体圆柱。**用外接多边形**：trimesh 的 cylinder 顶点落在圆上（内接），体积系统性偏小
    —— Ø15×9.3 少算 0.66 mm³、Ø4.6×40 少算 0.27 mm³，都是 0.05 阈值的 5~13 倍，
    方向恰好是漏报（真侵入 <0.7 mm³ 会被算成 0）。外接后近似误差改成偏保守（宁可多报）。"""
    import math as _m
    import numpy as np
    import trimesh
    from trimesh.transformations import rotation_matrix as rot
    r = (d / 2.0) / _m.cos(_m.pi / sections)                    # 外接：顶点在圆外，边中点切于圆
    c = trimesh.creation.cylinder(radius=r, height=h, sections=sections)
    if axis == "x":
        c.apply_transform(rot(_m.pi / 2, [0, 1, 0]))
    if axis == "y":
        c.apply_transform(rot(_m.pi / 2, [1, 0, 0]))
    c.apply_translation(np.asarray(center, float))
    return c


def _rel(p):
    """仓库相对路径，用于 res.inputs 的过期绑定。路径不在仓库里时退回绝对路径 —— 记账不能把层弄崩。"""
    try:
        return str(Path(p).relative_to(ROOT))
    except ValueError:
        return str(p)


def _placed(mesh, R):
    m = mesh.copy()
    m.apply_transform(R)
    return m


# ── data 解析工具 ──────────────────────────────────────────────────────────
_NUM = r"[0-9]+(?:\.[0-9]+)?"


def _triple(text):
    """从 '20×27×4' / '72×25×18' / '19×15×5' 这种写法里取三元组；取不到返回 None。"""
    if not isinstance(text, str):
        return None
    m = re.search(rf"({_NUM})\s*[×xX*]\s*({_NUM})\s*[×xX*]\s*({_NUM})", text)
    return tuple(float(g) for g in m.groups()) if m else None


def _env_of(comp):
    e = comp.get("envelope_mm")
    if not isinstance(e, dict):                                  # 写成裸字符串/None 时不能 AttributeError
        return (e if isinstance(e, str) else None,
                None if isinstance(e, str) else "envelope_mm 不是 {v:…} 结构", None)
    return (e.get("v"), e.get("unknown_reason"), e.get("src"))


def _tol_max(ctx):
    t = ((ctx.data.get("tolerances") or {}).get("feature_check_tolerances") or {}) \
        .get("static_intersection_mm3") or {}
    return t.get("max")


# ── 打印件：件号 ↔ placed 件 ───────────────────────────────────────────────
def _index_placed():
    return {p.stem: p for p in sorted(PLACED.glob("*.stl"))} if PLACED.exists() else {}


def _match_parts(ctx, placed_idx, res):
    """pid -> [placed stem]。用（三角面数, |体积|）指纹匹配导出件与 placed 件。
    不靠文件名 —— parts.yaml 的 inventory_id 与 placed 名并不一一对应（L06_sole_TPU→sole、N03_yaw_roll→yrm）。"""
    sig = {}
    for stem, p in placed_idx.items():
        try:
            n, v = _stl_sig(p)
        except Exception:
            continue
        sig[stem] = (n, abs(v))
    out, used = {}, set()
    for pid in ctx.parts:
        stl = ctx.stl(pid)
        if stl is None:
            res.unknown(pid, "stl_exists", f"cad/duck_s288/ 里找不到 {pid} 的导出 STL",
                        provenance="parts.yaml:parts[].id")
            continue
        n, v = _stl_sig(stl)
        # 相对容差 1e-5：同一个网格写成局部坐标和世界坐标两份 STL，float32 舍入位置不同，
        # 有符号体积会差到 ~1e-7 相对量（实测 L07 1.5e-7）。面数相同 + 体积相对差 <1e-5 已足够唯一。
        hits = [s for s, (sn, sv) in sig.items()
                if sn == n and abs(sv - abs(v)) <= 1e-5 * max(abs(v), 1.0)]
        if not hits:
            res.unknown(pid, "placed_instance",
                        f"placed/ 里找不到与导出件指纹一致的世界坐标件（{n} 面 / {abs(v):.6f} mm³）—— "
                        "第 3 层无法把它放到位",
                        provenance="cad/duck_s288/placed/")
            continue
        for h in hits:
            out.setdefault(pid, []).append(h)
            used.add(h)
    return out, used


# ── 元件：按 components.yaml:claim 认领 placed 实体（2026-09-13 起不按 category 分支，与 L7 同一份声明）────
def _claim_components(comps, placed_idx, part_stems, geo):
    """component id -> (stems, note)。规则只读 components.yaml:components[].claim.by：
       mjcf_servo_geom（placed/servo__<body>_<drives>，名单来自 MJCF body.servos）| envelope_ring（envelope_mm 三元组反算环体积+包围盒）
       | envelope_box（三元组反算盒体积+包围盒）| placed_prefix（claim.pattern 前缀）。没有 claim / 不认识 → 不认领（上层 world_placement unknown）。"""
    import numpy as np
    free = [s for s in placed_idx if s not in part_stems]
    claimed, notes = {}, {}

    def by_envelope(tri, ring, skip_prefixes):
        if ring:
            t, bore, od = sorted(tri)
            want_v = math.pi / 4.0 * (od * od - bore * bore) * t
            want_e = tuple(sorted((t, od, od)))
        else:
            want_v = tri[0] * tri[1] * tri[2]
            want_e = tuple(sorted(tri))
        hits = []
        for s_ in sorted(free):
            if s_.startswith(tuple(skip_prefixes)):
                continue
            m = geo.mesh(placed_idx[s_])
            e = tuple(sorted(np.round(m.bounds[1] - m.bounds[0], 6)))
            if all(abs(a - b) <= 0.05 for a, b in zip(e, want_e)) and abs(m.volume - want_v) / want_v < 0.01:
                hits.append(s_)
        return hits, want_v, want_e

    # 先算出各类 claim 用到的名单，前缀类先认领，包络类再在剩下的里找（顺序与 L7 一致：名单/前缀确定性最高）
    servo_names = set()
    try:
        from duckstructure.lib import B, ORDER
        for n in ORDER:
            for sv in B[n]["servos"]:
                if sv.get("drives") is not None:
                    servo_names.add(f"servo__{n}_{str(sv['drives']).split(':')[0]}")   # 'body:self' → 驱动自己的舵机
    except Exception:                                          # noqa: BLE001
        servo_names = None
    prefix_claims = [c for c in comps if isinstance(c.get("claim"), dict) and c["claim"].get("by") in ("mjcf_servo_geom", "placed_prefix")]
    env_claims = [c for c in comps if isinstance(c.get("claim"), dict) and c["claim"].get("by") in ("envelope_ring", "envelope_box")]
    for c in comps:
        cid = c.get("id")
        cl = c.get("claim")
        if not isinstance(cl, dict) or cl.get("by") not in ("mjcf_servo_geom", "placed_prefix", "envelope_ring", "envelope_box"):
            notes[cid] = f"components.yaml 没有可用的 claim.by（={cl.get('by') if isinstance(cl, dict) else None!r}）—— 不认领"
    for c in prefix_claims + env_claims:
        cid, cl = c.get("id"), c["claim"]
        env, _, _ = _env_of(c)
        tri = _triple(env)
        if cl["by"] == "mjcf_servo_geom":
            if servo_names is None:
                notes[cid] = "claim.by=mjcf_servo_geom 但 MJCF 名单取不到（duckstructure 导入失败）—— 不认领"
                hits = []
            else:
                hits = sorted(s_ for s_ in free if s_ in servo_names)
                notes[cid] = f"claim.by=mjcf_servo_geom：placed 名 ∈ MJCF body.servos 推出的 servo__<body>_<drives>（{len(servo_names)} 个名字）"
        elif cl["by"] == "placed_prefix":
            pat = str(cl.get("pattern") or "")
            hits = sorted(s_ for s_ in free if pat and s_.startswith(pat))
            notes[cid] = f"claim.by=placed_prefix：placed 名前缀 {pat!r}"
        elif tri:
            ring = cl["by"] == "envelope_ring"
            taken = {s_ for ss in claimed.values() for s_ in ss}
            hits, want_v, want_e = by_envelope(tri, ring, [])
            hits = [h for h in hits if h not in taken]
            notes[cid] = (f"claim.by={cl['by']}：按 envelope {env} 反算{'环' if ring else '盒'}体积 {want_v:.2f} mm³ / 包围盒 {want_e} 认领")
        else:
            hits = []
            notes[cid] = f"claim.by={cl['by']} 但 envelope_mm 解析不出三元组：{str(env)[:60]}"
        if hits:
            claimed[cid] = hits
            free = [s_ for s_ in free if s_ not in hits]
    return claimed, notes, free


# ── 排除项（必须显式记录） ────────────────────────────────────────────────
def _seam_added(ctx, geo, placed_idx, s, o, pid):
    """新增接缝 = (我们的件 ∩ 原版顶壳) − (原版毛坯 ∩ 原版顶壳)，取体积。
    毛坯网格名从 parts.yaml:parts[pid].derived_from_original 里取（"原版 bottom_head_shell（…）"）。
    拿不到就返回 (None, 原因) —— 上层判 unknown，不退回标量相减。"""
    pr = next((x for x in ((ctx.data.get("parts") or {}).get("parts") or []) if x.get("id") == pid), {})
    txt = pr.get("derived_from_original") or ""
    m = re.search(r"([A-Za-z_][A-Za-z0-9_]*)", txt.replace("原版", " "))
    if not m:
        return None, f"parts.yaml:parts[{pid}].derived_from_original={txt!r} 里认不出毛坯网格名"
    name = m.group(1)
    try:
        from duckstructure.lib import B, TW
        from duckstructure import kin
        hit = [(n, q["T"]) for n in B for q in (B[n].get("parts") or []) if q.get("mesh") == name]
        if not hit:
            return None, f"MJCF 里没有名为 {name} 的原版网格"
        body, T = hit[0]
        raw = kin.mesh(name)
        raw.apply_transform(TW(body) @ T)
        import trimesh
        raw = trimesh.load(trimesh.util.wrap_as_stream(trimesh.exchange.stl.export_stl(raw)),
                           file_type="stl", process=True)
        top = geo.solid(placed_idx[o])
        new_seam = geo.wrap(geo.mesh(placed_idx[s])) ^ top
        old_seam = geo.wrap(raw) ^ top
        return max(0.0, float((new_seam - old_seam).volume())), name
    except Exception as e:                                      # noqa: BLE001
        return None, f"建不出原版毛坯 {name} 的世界实体：{e}"


def _exempted(exempt, pid, tag):
    """豁免必须**按分段**匹配，不能只按件号。KO12 的豁免理由是"门堵住抽出走廊是设计如此"，
    要是只按件号匹配，同一个门压进**电池仓腔**（会压电池）也会被一起赦免。"""
    e = exempt.get(pid)
    if not e:
        return False
    seg = e[0]
    return seg is None or seg in tag


def _seam_decl(ctx, pid2stems, orig_stems):
    """从 parts.yaml 里找"与某个原版件的接缝是原版自带、按声明排除"这条声明，别把件名写死在层里。
    找法：某个件的 mates_with 里同时提到「接缝」和一个 placed 里存在的原版件名。
    基线值取 mechanical_audit.json 里那条记录的 reference_mm3（原版 × 原版 算出来的，不是我们这版的当前值）；
    基线所在的 key 也从 parts.yaml 的 provenance 里反查，不硬编码。
    返回 (pid, 本件 stem, 原版 stem, 基线 mm³, 声明原文, 基线出处) 或 None。"""
    for p in ((ctx.data.get("parts") or {}).get("parts") or []):
        pid = p.get("id")
        if pid not in pid2stems:
            continue
        for txt in (p.get("mates_with") or []):
            if not isinstance(txt, str) or "接缝" not in txt:
                continue
            for o in sorted(orig_stems):
                if o[len("orig_"):] not in txt:
                    continue
                key, base = None, None
                prov = list(p.get("provenance") or [])
                for blk in ((ctx.data.get("parts") or {}).get("deliberately_added") or []):
                    if isinstance(blk, dict) and blk.get("id") == pid:
                        prov += list(blk.get("provenance") or []) + [str(blk.get("evidence") or "")]
                for s in prov:
                    m = re.search(r"mechanical_audit\.json:([A-Za-z0-9_]*seam[A-Za-z0-9_]*)", str(s))
                    if m:
                        key = m.group(1)
                        break
                if key:
                    try:
                        base = float(json.loads(_MECH.read_text(encoding="utf-8"))[key]["reference_mm3"])
                    except Exception:                            # noqa: BLE001
                        base = None
                return pid, pid2stems[pid][0], o, base, txt, f"mechanical_audit.json:{key}.reference_mm3"
    return None


# ── 禁入体 ────────────────────────────────────────────────────────────────
# 五条属于"路径 / 扫掠"，README 第 2 节把它们分给第 4 层（装得进去）和第 6 层（运动），
# 第 3 层零位姿静态交集对它们没有意义 —— 记 NOT_RUN 并写清归属层，不静默跳过、也不假装通过。
# 2026-09-13 起不再写死：keepouts.yaml:KOxx.checked_in_layer（4/6）+ covered_by_subjects 声明"归哪层、由哪些格子查"，
# core.obligations 按它派义务、core.build_scorecard 把那些格子汇总成 L{层}/KOxx；本层只登记 NOT_RUN INFO 留痕。
def _v(x):
    """{v: …, src: …} 或裸值 → 值。"""
    return x.get("v") if isinstance(x, dict) else x


def _deferred(kos):
    return {k["id"]: (k.get("checked_in_layer"), k.get("covered_by_note") or "")
            for k in kos if isinstance(k, dict) and k.get("id") and k.get("checked_in_layer") not in (None, 3)}


_SIGN = r"-?[0-9]+(?:\.[0-9]+)?"


def _diam(s):
    m = re.search(rf"[Ø⌀]\s*({_NUM})", s or "")
    return float(m.group(1)) if m else None


def _span(s, axis):
    """'z 215..224.3' / 'x -83.8..-24.6' → (lo, hi)。"""
    m = re.search(rf"{axis}\s*({_SIGN})\s*\.\.\s*({_SIGN})", s or "")
    return (float(m.group(1)), float(m.group(2))) if m else None


def _at(s):
    """'@(49.1,9.35)' / '@(89,0,264)' → tuple。"""
    m = re.search(r"@\s*\(([^)]*)\)", s or "")
    if not m:
        return None
    vals = re.findall(_SIGN, m.group(1))
    return tuple(float(v) for v in vals) if vals else None


# ── 结构化包络 / 扫掠禁入体（hr41g 2026-09-24；首例 KO22 电池顶插头收纳/后抽通道）──────────────────
#   keepouts.yaml:<KO>.bounds_world_mm = 世界系包络盒 [[x,y,z],[x,y,z]]（不再从 geom_verbatim 正则解析）。
#   · 声明了 moving_components 或任一 sweep_* 字段 → 扫掠型："随行件（moving_components）带着这个包络沿
#     sweep_direction_world 平移 0..sweep_len_mm"。扫掠体 = 包络盒起止两位 16 个角点的凸包（盒沿直线的 Minkowski 和，精确）。
#     障碍 = placed/ 全部实体（打印件 + 元件，元件挡路同样抽不出来）− 随行件；exempt[segment=all] 的件按
#     "拆卸态不在场"处理：照算、记数、不计峰值（与 KO12 exempt 同一记账口径）。另判终点脱离（keepout_sweep_clear_at_end）。
#     方向 / 行程 / 随行件任一缺或坏 → keepout_sweep_declared unknown，不猜、不代跑（"字段齐则算，缺则 unknown"）。
#   · 只有 bounds_world_mm、没有任何扫掠声明 → 静态包络盒，和其它 KO 同口径只查打印件。
_SWEEP_KEYS = ("sweep_direction_world", "sweep_len_mm")


def _ko_bounds_sweep(k, geo):
    import numpy as np
    import trimesh
    kid = k["id"]
    prov = f"keepouts.yaml:{kid}.bounds_world_mm/sweep_direction_world/sweep_len_mm/moving_components/exempt"
    frame = k.get("frame")
    used = frame if isinstance(frame, str) and frame.startswith("world") else "world"   # bounds_world_mm 按名字就是世界系

    def bad(why):
        return dict(solids=[], targets=None, used_frame=used, note="", prov=prov,
                    extra_unknown=("keepout_sweep_declared", why))

    bb = _v(k.get("bounds_world_mm"))
    try:
        lo, hi = np.asarray(bb[0], float), np.asarray(bb[1], float)
        ok = (lo.shape == (3,) and hi.shape == (3,) and bool(np.isfinite(lo).all()) and bool(np.isfinite(hi).all())
              and bool(np.all(hi > lo)))
    except Exception:                                           # noqa: BLE001
        ok = False
    if not ok:
        return bad(f"bounds_world_mm={bb!r} 不是 [[x,y,z],[x,y,z]] 且 lo<hi 的包络盒")
    exempt = {}
    for e in (k.get("exempt") or []):
        if isinstance(e, dict) and e.get("part") and e.get("segment") == "all":
            exempt[str(e["part"])] = (None, str(e.get("why") or f"（keepouts.yaml:{kid}.exempt 未写理由）"))
    moving = k.get("moving_components")
    if not (moving or any(key in k for key in _SWEEP_KEYS)):
        return dict(solids=[("box", geo.wrap(_box(hi - lo, (lo + hi) / 2.0)))], targets=None, exempt=exempt,
                    used_frame=used, note=f"静态包络盒 {lo.tolist()}..{hi.tolist()}（keepouts.yaml:{kid}.bounds_world_mm）",
                    prov=prov)
    missing = []
    dv = _v(k.get("sweep_direction_world"))
    d = None
    if "sweep_direction_world" not in k:
        missing.append("缺 sweep_direction_world（扫掠方向，世界系三维向量）")
    else:
        try:
            d = np.asarray(dv, float)
            if d.shape != (3,) or not np.isfinite(d).all() or float(np.linalg.norm(d)) <= 0:
                d = None
        except Exception:                                       # noqa: BLE001
            d = None
        if d is None:
            missing.append(f"sweep_direction_world={dv!r} 不是有限非零三维向量")
    L = _v(k.get("sweep_len_mm"))
    if "sweep_len_mm" not in k:
        missing.append("缺 sweep_len_mm（行程 mm）")
    elif not (isinstance(L, (int, float)) and not isinstance(L, bool) and math.isfinite(L) and L > 0):
        missing.append(f"sweep_len_mm={L!r} 不是有限正数")
    if not (isinstance(moving, list) and moving and all(isinstance(m, str) and m for m in moving)):
        missing.append("缺 moving_components（随包络移动、不当障碍的实体名列表）")
    if missing:
        return bad("扫掠型禁入体（声明了 moving_components 或 sweep_*）字段不齐：" + "；".join(missing)
                   + " —— 不猜方向/行程，不代跑；独立脚本的通过不能冒充这里通过")
    d = d / np.linalg.norm(d)
    c0 = np.array([[x, y, z] for x in (lo[0], hi[0]) for y in (lo[1], hi[1]) for z in (lo[2], hi[2])])
    hull = trimesh.convex.convex_hull(np.vstack([c0, c0 + d * float(L)]))
    dtxt = "[" + ", ".join(f"{x:g}" for x in d) + "]"
    return dict(solids=[(f"sweep {dtxt}×{float(L):g} mm", geo.wrap(hull))], targets=None, exempt=exempt,
                used_frame=used, obstacles="all_placed", moving=list(moving),
                sweep=dict(d=d, L=float(L), end=(lo + d * float(L), hi + d * float(L))),
                note=(f"包络盒 {lo.tolist()}..{hi.tolist()} 沿 {dtxt} 扫 0..{float(L):g} mm（凸包）；障碍 = placed/ 全部实体 − "
                      f"随行件 {list(moving)}；拆卸态不在场（exempt segment=all）：{list(exempt) or '无'}"),
                prov=prov)


def _sep_along(end_box, ob_box, d):
    """终点包络盒之后再沿 d 走不会碰到 ob（AABB 证明，与 l4_assembly._proven_clear_after 同口径）：
    (a) 沿 d 已整体越过；(b) ⊥d 的 u/w/两条 45° 对角任一方向投影不重叠。AABB 分离即真分离，只用来证明。"""
    import numpy as np

    def span(b, v):
        c, e = (b[0] + b[1]) / 2.0, (b[1] - b[0]) / 2.0
        return float(c @ v) - float(np.abs(v) @ e), float(c @ v) + float(np.abs(v) @ e)

    if span(end_box, d)[0] >= span(ob_box, d)[1]:
        return True
    ref = np.array([1.0, 0.0, 0.0]) if abs(float(d[0])) < 0.9 else np.array([0.0, 1.0, 0.0])
    u = np.cross(d, ref)
    u = u / np.linalg.norm(u)
    w = np.cross(d, u)
    r = math.sqrt(0.5)
    for v in (u, w, (u + w) * r, (u - w) * r):
        a, b = span(end_box, v), span(ob_box, v)
        if a[1] < b[0] or b[1] < a[0]:
            return True
    return False


def _ko_geoms(ctx, geo, res, world_bounds=None):
    """能从 keepouts.yaml 的 geom_verbatim + frame **唯一确定**的禁入体实体。
    尺寸一律从 geom_verbatim 现场解析，坐标系一律按 frame 取（servo_local→sfw、
    world_frozen_joint→MJCF 关节帧、world_frozen_battery_bay→冻结世界系）—— 层里不抄坐标。
    返回 ko_id -> dict(solids=[(标签, manifold)], targets=None|set(pid), exempt={pid:理由},
                       ambiguous=bool, note, prov)。
    确定不了的一律不返回 → 上层判 unknown（元规则 4），绝不拿占位方块假装查过。"""
    out = {}
    from duckstructure.lib import B, ORDER, sfw
    from duckstructure.s288 import S
    kos = {k["id"]: k for k in ((ctx.data.get("keepouts") or {}).get("keepouts") or [])}

    def gv(kid):
        return (kos.get(kid) or {}).get("geom_verbatim") or ""

    # ── KO01 PH2.0 插头顶/线弯区 + 出线区（frame=servo_local；2026-09-20 背插模型，旧"侧窗面片 + 拔插走廊"作废）
    #    实体由 keepouts.yaml:KO01.dims 独立建（审计 F-L3-2 口径不变：不 import CAD 的 conn_cut 当实体，只作对照 INFO）：
    #      A 插头顶/线翘区 = x [plug_top_x − wire_out − wire_clr, plug_top_x] × |y| [W/2 − plug_y_in, W/2 + plug_clr] × z conn_z ± plug_clr（09-21 wire_out 4.0 + wire_clr 0.5）
    #      B 侧出线槽     = 同 x/z × |y| side_exit_y
    #      W 背板通窗     = x [window_x, plug_top_x] × A 的 y/z；A/W 扣掉 Ø hub_keep_d 同轴柱（从动件贴惰轮的毂，与插头顶面共面不重叠）
    #    每颗每侧按 availability_table.rows[].mode_pos_y / mode_neg_y：pocket → A+B，window → A+W，free → A+B，none → 不查。
    #    W/2 取 s288.S["W"]（舵机宽，手册值）；舵机局部→世界只借 duckstructure.lib.sfw（舵机落位，不是切刀）。查全部打印件（targets=None）。
    ko01 = kos.get("KO01") or {}
    modes = {}; carriers = {}; solid_targets = {}
    for r in ((ko01.get("availability_table") or {}).get("rows") or []):
        if r.get("servo"):
            modes[r["servo"]] = {1: r.get("mode_pos_y"), -1: r.get("mode_neg_y")}
            carriers[r["servo"]] = r.get("carrier")
    dims = ko01.get("dims") or {}
    ptx, wo, czr = _v(dims.get("plug_top_x_mm")), _v(dims.get("wire_out_mm")), _v(dims.get("conn_z_mm"))
    pclr, pyin, sey, wx = _v(dims.get("plug_clr_mm")), _v(dims.get("plug_y_in_mm")), _v(dims.get("side_exit_y_mm")), _v(dims.get("window_x_mm"))
    hkd = _v(dims.get("hub_keep_d_mm")); wclr = _v(dims.get("wire_clr_mm"))
    _num = lambda v: isinstance(v, (int, float)) and not isinstance(v, bool)          # noqa: E731
    _pair = lambda v: isinstance(v, (list, tuple)) and len(v) == 2 and all(_num(t) for t in v)   # noqa: E731
    ok_dims = (_num(ptx) and _num(wo) and wo > 0 and _num(wclr) and wclr >= 0 and _pair(czr) and _num(pclr) and _num(pyin) and _pair(sey)
               and _num(wx) and wx < ptx and _num(hkd) and hkd > 0)
    valid_modes = {"pocket", "window", "free", "none"}
    bad_modes = [f"{k}:{v}" for k, d_ in modes.items() for v in d_.values() if v not in valid_modes]
    if not modes:
        out["KO01"] = dict(extra_unknown=("bus_side_declared", "availability_table.rows[].mode_pos_y/mode_neg_y 一条都没有 —— 不知道每颗哪一侧按什么方式查"),
                           solids=[], targets=None, used_frame="servo_local", note="", prov="keepouts.yaml:KO01")
    elif not ok_dims or bad_modes:
        out["KO01"] = dict(extra_unknown=("keepout_dims_declared",
                                          f"KO01 的 dims plug_top_x_mm={ptx} wire_out_mm={wo} wire_clr_mm={wclr} conn_z_mm={czr} plug_clr_mm={pclr} plug_y_in_mm={pyin} "
                                          f"side_exit_y_mm={sey} window_x_mm={wx} hub_keep_d_mm={hkd}（要求全数值、wire_out>0、window_x<plug_top_x、hub_keep_d>0）；mode 非法 {bad_modes or '无'}"
                                          " —— 建不出独立实体，不借 CAD 切刀"),
                           solids=[], targets=None, used_frame="servo_local", note="", prov="keepouts.yaml:KO01.dims/availability_table")
    else:
        W2 = float(S["W"]) / 2.0
        z0, z1 = float(czr[0]) - float(pclr), float(czr[1]) + float(pclr)
        xa0, xa1 = float(ptx) - float(wo) - float(wclr), float(ptx)
        ya0, ya1 = W2 - float(pyin), W2 + float(pclr)
        yb0, yb1 = float(sey[0]), float(sey[1])
        def _zone(sy, kind):
            if kind == "A": x0, x1, y0, y1 = xa0, xa1, ya0, ya1
            elif kind == "B": x0, x1, y0, y1 = xa0, xa1, yb0, yb1
            else: x0, x1, y0, y1 = float(wx), xa1, ya0, ya1
            m_ = _box((x1 - x0, y1 - y0, z1 - z0), ((x0 + x1) / 2, sy * (y0 + y1) / 2, (z0 + z1) / 2))
            if kind in ("A", "W"):
                # 扣掉从动件贴惰轮的毂柱（dims.hub_keep_d_mm，外接 128 边形 → 扣得略多 = 方向偏保守）
                m_ = m_.difference(_cyl(float(hkd), x1 - x0 + 2.0, ((x0 + x1) / 2, 0.0, 0.0), axis="x"))
            return m_
        kinds_of = {"pocket": ("A", "B"), "window": ("A", "W"), "free": ("A", "B"), "none": ()}
        plugs, cad = [], {}
        n_none = 0
        try:
            from duckstructure.lib import conn_cut
        except Exception:                                      # noqa: BLE001
            conn_cut = None
        for n in ORDER:
            for i_, sv in enumerate(B[n]["servos"]):
                if sv["drives"] is None:
                    continue
                key = f"{n}[{i_}]"
                md = modes.get(key)
                if md is None:
                    continue
                R = sfw(n, i_)
                for sy in (1, -1):
                    mode = md.get(sy)
                    if mode == "none" or mode is None:
                        n_none += 1
                        continue
                    for kind in kinds_of[mode]:
                        tag = f"{key}{'+y' if sy > 0 else '-y'}/{kind}"
                        plugs.append((tag, geo.wrap(_placed(_zone(sy, kind), R))))
                        if kind == "W":
                            # W（背板通窗到 window_x）只对载体件查：通窗是载体背板的义务，板外邻件只须让 A（09-21）。载体 = rows[].carrier 里第一个件号
                            cp = re.findall(r"[A-Z]\d\d", str(carriers.get(key) or ""))
                            solid_targets[tag] = {cp[0]} if cp else None
                    if conn_cut is not None:
                        try:
                            cc = conn_cut(R, sides=(sy,), mode=mode)
                            cad[f"{key}{'+y' if sy > 0 else '-y'}"] = geo.wrap(cc) if cc is not None else "free/none 模式无刀"
                        except Exception as e:                  # noqa: BLE001
                            cad[f"{key}{'+y' if sy > 0 else '-y'}"] = f"CAD 切刀建不出：{e}"
        out["KO01"] = dict(solids=plugs, targets=None, solid_targets=solid_targets, used_frame="servo_local", cad_cutters=cad, cad_cutters_by_side=True,
                           note=f"{len(plugs)} 个实体（{len(modes)} 颗 × 2 侧，none 不查 {n_none} 处）：A 插头顶/线弯区 x {xa0}..{xa1} × |y| {ya0}..{ya1} × z {z0}..{z1}；"
                                f"pocket 加 B 侧出线槽 |y| {yb0}..{yb1}；window 加 W 通窗 x {wx}..{xa1}（keepouts.yaml:KO01.dims，舵机局部系经 duckstructure.lib.sfw 落世界，含 SERVO_SHIFT）。"
                                "实体独立于 CAD 切刀；区空 ⇒ 插头插得到底、线出得来",
                           prov="keepouts.yaml:KO01.dims.plug_top_x_mm/wire_out_mm/wire_clr_mm/conn_z_mm/plug_clr_mm/plug_y_in_mm/side_exit_y_mm/window_x_mm/hub_keep_d_mm"
                                " + availability_table.rows[].mode_pos_y|mode_neg_y + s288.S.W + duckstructure.lib.sfw（落位）")

    # ── KO06 N02 头俯仰 6 螺丝头（frame=part_local = 驱动 neck_pitch 的舵机帧）
    #    'Ø4×1.6 @x17.2'；6 个位置 = 法兰 6 孔分度圆（s288.S.horn_n/horn_r）
    g = gv("KO06")
    d = _diam(g)
    hx = re.search(rf"@\s*x\s*({_NUM})", g)
    hh = re.search(rf"[Ø⌀]\s*{_NUM}\s*[×xX*]\s*({_NUM})", g)
    jn6 = _v((kos.get("KO06") or {}).get("servo_frame_of_joint"))
    if d and hx and hh and not jn6:
        out["KO06"] = dict(solids=[], targets=None, used_frame="servo_local", note="", prov="keepouts.yaml:KO06.servo_frame_of_joint",
                           extra_unknown=("servo_frame_declared", "keepouts.yaml:KO06.servo_frame_of_joint 缺失 —— 不知道 frame=servo_local 指哪颗舵机（以前写死 neck_pitch）"))
    elif d and hx and hh:
        try:
            from duckstructure.lib import drv_from
            R = drv_from(str(jn6))
            heads = []
            for k in range(S["horn_n"]):
                a = 2 * math.pi * k / S["horn_n"]
                y, z = S["horn_r"] * math.cos(a), S["horn_r"] * math.sin(a)
                heads.append((f"horn{k}", geo.wrap(
                    _placed(_cyl(d, float(hh.group(1)), (float(hx.group(1)), y, z), axis="x"), R))))
            out["KO06"] = dict(solids=heads, targets=None, used_frame="servo_local",
                               facet_noise_mm3=_facet_noise_mm3(d, float(hh.group(1)), len(heads)),
                               note=f"{S['horn_n']} 个 Ø{d}×{hh.group(1)} 螺丝头盘 @ 舵机局部 "
                                    f"x={hx.group(1)}、r={S['horn_r']}（舵机 = 驱动 {jn6} 的那颗，keepouts.yaml:KO06.servo_frame_of_joint）",
                               prov="keepouts.yaml:KO06.geom_verbatim/servo_frame_of_joint + s288.py:S.horn_r/horn_n "
                                    f"+ duckstructure.lib.drv_from({jn6!r})")
        except Exception as e:                                # noqa: BLE001
            res.unknown("KO06", "keepout_solid", f"建不出 N02 螺丝头实体：{e}",
                        provenance="keepouts.yaml:KO06")

    # ── KO07 N02 立筒 Ø15 内腔（唯一起子通道）：'Ø15 z 215..224.3'，轴 = head_yaw 关节轴
    g = gv("KO07")
    d, zs = _diam(g), _span(g, "z")
    ja = {j["name"]: j for j in ((ctx.data.get("frozen") or {}).get("joint_axes") or [])}
    aj7 = _v((kos.get("KO07") or {}).get("axis_joint"))
    hy = (ja.get(str(aj7)) or {}).get("mjcf_body") if aj7 else None
    if d and zs and not aj7:
        out["KO07"] = dict(solids=[], targets=None, used_frame="world_frozen_joint", note="", prov="keepouts.yaml:KO07.axis_joint",
                           extra_unknown=("axis_joint_declared", "keepouts.yaml:KO07.axis_joint 缺失 —— 不知道立筒轴取哪个关节（以前写死 head_yaw）"))
    elif d and zs and aj7 and hy not in B:
        out["KO07"] = dict(solids=[], targets=None, used_frame="world_frozen_joint", note="", prov="keepouts.yaml:KO07.axis_joint",
                           extra_unknown=("axis_joint_declared", f"KO07.axis_joint={aj7!r} 在 frozen.yaml:joint_axes 里没有 mjcf_body 或 MJCF 里没有该 body"))
    elif d and zs and hy in B:
        T = B[hy]["T_world"]
        ax, org = T[:3, 2], T[:3, 3]
        if abs(abs(ax[2]) - 1.0) < 1e-6:                       # 竖直轴，按 z 区间给筒才成立
            m = _cyl(d, zs[1] - zs[0], (org[0], org[1], (zs[0] + zs[1]) / 2), axis="z")
            out["KO07"] = dict(solids=[("tube", geo.wrap(m))], targets=None,
                               used_frame="world_frozen_joint", facet_noise_mm3=_facet_noise_mm3(d, zs[1] - zs[0]),
                               note=f"Ø{d} 圆柱 z {zs[0]}..{zs[1]}，轴心取 {aj7} 关节轴 "
                                    f"({org[0]:.3f}, {org[1]:.3f})（keepouts.yaml:KO07.axis_joint → frozen.yaml:joint_axes → MJCF，不是抄坐标）",
                               prov=f"keepouts.yaml:KO07.geom_verbatim/axis_joint + frozen.yaml:joint_axes.{aj7}")

    # ── KO09 H01 右前脚 Ø4.6 通道：'Ø4.6 z 233.2..273.2 @(49.1,9.35)' 世界坐标齐全
    g = gv("KO09")
    d, zs, xy = _diam(g), _span(g, "z"), _at(g)
    if d and zs and xy and len(xy) == 2:
        m = _cyl(d, zs[1] - zs[0], (xy[0], xy[1], (zs[0] + zs[1]) / 2), axis="z")
        out["KO09"] = dict(solids=[("channel", geo.wrap(m))], targets=None, used_frame="world",
                           facet_noise_mm3=_facet_noise_mm3(d, zs[1] - zs[0]),
                           note=f"Ø{d} 圆柱 z {zs[0]}..{zs[1]} @({xy[0]}, {xy[1]})；"
                                "只查 17 个打印件 —— 原版顶壳挡不挡这条通道属装配序，归第 4 层",
                           prov="keepouts.yaml:KO09.geom_verbatim")

    # ── KO11 躯干壳侧壁区：geom_verbatim 只写了代码符号 shell_cutter，不是几何声明。
    #    按字面把 |y|≥19 整片当禁入体是**错的**（真正的切刀是"壳外扩 SHELL_CLR 后再与该片相交"，
    #    T01 在壳覆盖不到的地方本来就该有料）。量一次只为把误差摆出来，判 ambiguous。
    lim = None
    mm = re.search(rf"\|y\|\s*[≥>=]+\s*({_NUM})", (kos.get("KO11") or {}).get("name") or "")
    dil = re.search(rf"外扩\s*({_NUM})", (kos.get("KO11") or {}).get("name") or "")
    zz = _v((kos.get("KO11") or {}).get("probe_zone_z_mm"))
    if mm and isinstance(zz, (list, tuple)) and len(zz) == 2 and all(isinstance(v, (int, float)) for v in zz):
        lim = float(mm.group(1))
        z0, z1 = float(zz[0]), float(zz[1])                     # keepouts.yaml:KO11.probe_zone_z_mm（以前写死 100..175）
        wx = float(world_bounds[1][0] - world_bounds[0][0]) + 20.0 if world_bounds is not None else 300.0
        wy = float(max(abs(world_bounds[0][1]), abs(world_bounds[1][1]))) + 10.0 if world_bounds is not None else 100.0
        xc = float(world_bounds[0][0] + world_bounds[1][0]) / 2 if world_bounds is not None else 0.0
        walls, opts = [], [(lim, "字面值")]
        if dil:
            opts.append((lim - float(dil.group(1)), f"'外扩 {dil.group(1)}' 往内扩的解释"))
        for v, tag in opts:
            for sy in (1, -1):
                c = _box((wx, wy, z1 - z0), (xc, sy * (v + wy / 2), (z0 + z1) / 2))
                walls.append((f"|y|≥{v} {tag} {'+y' if sy > 0 else '-y'}", geo.wrap(c)))
        out["KO11"] = dict(solids=walls, targets=None, ambiguous=True, used_frame="world",
                           note=f"按字面 {[o[0] for o in opts]} 两种解释各 ±y 两块，"
                                f"z 限 {z0}..{z1}（keepouts.yaml:KO11.probe_zone_z_mm）；盒的 x/y 外沿取全部 placed 件包围盒外扩",
                           prov="keepouts.yaml:KO11.name/probe_zone_z_mm")
    elif mm:
        out["KO11"] = dict(solids=[], targets=None, used_frame="world", note="", prov="keepouts.yaml:KO11.probe_zone_z_mm",
                           extra_unknown=("probe_zone_declared", "keepouts.yaml:KO11.probe_zone_z_mm 缺失或不是 [z0, z1]（以前写死 100..175）"))

    # ── KO12 电池仓腔 + 后抽出路径（frame=world_frozen_battery_bay）：
    #    'x -83.8..-24.6 |y|≤13.1 z 73.7..146.9'。按 frozen.yaml:battery_bay_26.BX0 拆两段：
    #    仓腔谁都不许进；抽出走廊被 B01 门挡住是设计如此。
    g = gv("KO12")
    xs, zs = _span(g, "x"), _span(g, "z")
    ay = re.search(rf"\|y\|\s*[≤<=]+\s*({_NUM})", g)
    # battery_bay_26 的 26 项里有 4 项 value 是字符串（P 字典引用 + 3 个函数体），必须先挡掉
    bay = {i["name"]: i["value"].get("v")
           for i in (((ctx.data.get("frozen") or {}).get("battery_bay_26") or {}).get("items") or [])
           if isinstance(i, dict) and "name" in i and isinstance(i.get("value"), dict)}
    bx0 = bay.get("BX0")
    if xs and zs and ay:
        w = 2 * float(ay.group(1))
        if isinstance(bx0, (int, float)) and xs[0] < bx0 < xs[1]:
            segs = [(xs[0], bx0, "抽出走廊"), (bx0, xs[1], "仓腔")]
            note = f"按 frozen.yaml:battery_bay_26.BX0={bx0} 拆成仓腔与抽出走廊两段"
            # 豁免只认 keepouts.yaml:KO12.exempt[] 的声明（part + segment + why），层里不写件号（以前写死 "B01"）
            exempt = {}
            for e in ((kos.get("KO12") or {}).get("exempt") or []):
                if isinstance(e, dict) and e.get("part") and e.get("segment") in [t for _, _, t in segs]:
                    exempt[str(e["part"])] = (str(e["segment"]), str(e.get("why") or "（keepouts.yaml:KO12.exempt 未写理由）"))
        else:
            segs = [(xs[0], xs[1], "仓腔+走廊合一")]
            note = "frozen.yaml:battery_bay_26 里取不到 BX0，两段没拆开"
            exempt = {}                                       # 没拆段就不豁免任何东西
        out["KO12"] = dict(solids=[(f"{t} x {a}..{b}",
                                    geo.wrap(_box((b - a, w, zs[1] - zs[0]),
                                                  ((a + b) / 2, 0.0, (zs[0] + zs[1]) / 2))))
                                   for a, b, t in segs],
                           targets=None, exempt=exempt, used_frame="world_frozen_battery_bay",
                           note=note + f"；|y|≤{ay.group(1)} / z {zs[0]}..{zs[1]}；豁免声明 {list(exempt) or '无'}（keepouts.yaml:KO12.exempt）",
                           prov="keepouts.yaml:KO12.geom_verbatim/exempt + frozen.yaml:battery_bay_26.BX0")

    # KO21 6704/6700 完整实体：直接用 placed 里的整颗环（判据落在各轴承的 part_vs_component_solid）

    # 结构化包络 / 扫掠禁入体（bounds_world_mm；hr41g，见 _ko_bounds_sweep）。已被上面专用代码建过的 KO 不覆盖。
    for kid, k in sorted(kos.items()):
        if kid not in out and isinstance(k, dict) and "bounds_world_mm" in k:
            out[kid] = _ko_bounds_sweep(k, geo)
    return out


def _ko_camera_rays(ctx, geo, placed_idx, res):
    """KO15 镜头视锥：FOV / 出瞳 / 光轴全部读 keepouts.yaml:KO15 的 fov_deg / apex_world_mm / optical_axis_world
    （2026-09-13 起；以前从 name 正则解析、光轴 +x 写死 —— Codex01 FG16）。缺任一字段 → unknown。1° 网格。
    只问"有没有被挡"，不问视距 —— 所以不必编造一个锥体长度（data 里也没有）。
    采样网格独立于建模角度（元规则 3）：固定 1° 整数网格，不复用任何切刀角。"""
    import numpy as np
    import trimesh
    ko = next((k for k in ((ctx.data.get("keepouts") or {}).get("keepouts") or [])
               if k.get("id") == "KO15"), None)
    fov, apex, oax = _v((ko or {}).get("fov_deg")), _v((ko or {}).get("apex_world_mm")), _v((ko or {}).get("optical_axis_world"))

    def _vec(x, n):
        return isinstance(x, (list, tuple)) and len(x) == n and all(isinstance(v, (int, float)) for v in x)
    if not (_vec(fov, 2) and _vec(apex, 3) and _vec(oax, 3) and float(np.linalg.norm(oax)) > 0):
        res.unknown("KO15", "camera_fov_unobstructed",
                    f"keepouts.yaml:KO15 缺机器可读字段：fov_deg={fov!r} apex_world_mm={apex!r} optical_axis_world={oax!r}"
                    "（不再从 name 解析、不再写死 +x）", provenance="keepouts.yaml:KO15.fov_deg/apex_world_mm/optical_axis_world")
        return
    fh, fv = float(fov[0]) / 2.0, float(fov[1]) / 2.0
    apex = np.asarray(apex, float)
    ax = np.asarray(oax, float) / np.linalg.norm(oax)
    upref = np.array([0.0, 0.0, 1.0]) if abs(ax[2]) < 0.9 else np.array([1.0, 0.0, 0.0])
    side = np.cross(upref, ax)
    side /= np.linalg.norm(side)
    up = np.cross(ax, side)
    dirs = []
    for h in np.arange(-fh, fh + 1e-9, 1.0):
        for v in np.arange(-fv, fv + 1e-9, 1.0):
            ch, sh = math.cos(math.radians(h)), math.sin(math.radians(h))
            cv, sv = math.cos(math.radians(v)), math.sin(math.radians(v))
            dirs.append(cv * ch * ax + cv * sh * side + sv * up)
    dirs = np.asarray(dirs)
    declared = re.search(rf"({_NUM})\s*射线", (ko or {}).get("geom_verbatim") or "")
    names = sorted(placed_idx)
    big = trimesh.util.concatenate([geo.mesh(placed_idx[n]) for n in names])
    blocked = np.asarray(big.ray.intersects_any(np.repeat(apex[None], len(dirs), axis=0), dirs))
    nb = int(blocked.sum())
    culprits = []
    if nb:
        bd = dirs[blocked]
        for n in names:
            hit = np.asarray(geo.mesh(placed_idx[n]).ray.intersects_any(
                np.repeat(apex[None], len(bd), axis=0), bd))
            if hit.any():
                culprits.append(f"{n}×{int(hit.sum())}")
    same = bool(declared) and abs(float(declared.group(1)) - len(dirs)) < 0.5
    asrc = (ko or {}).get("apex_world_mm")
    asrc = asrc.get("src", "") if isinstance(asrc, dict) else ""
    res.add(subject="KO15", check="camera_fov_unobstructed", state=PASS if nb == 0 else FAIL,
            severity=BLOCK, measured=nb,
            criterion=f"{fov[0]}°×{fov[1]}° 视锥内被打断的射线数 = 0"
                      f"（FOV / 出瞳 / 光轴 {[float(v) for v in oax]} 全部来自 keepouts.yaml:KO15 的字段，不从 name 解析、不写死）",
            evidence_n=len(dirs),
            detail=f"出瞳 {apex.tolist()}、光轴 {ax.round(6).tolist()}、1° 网格 {len(dirs)} 条"
                   + (f"（与 geom_verbatim 声明的 {declared.group(1)} 条一致）" if same
                      else f"（geom_verbatim 声明 {declared.group(1) if declared else '?'} 条，对不上）")
                   + "；" + (f"被挡：{', '.join(culprits)}" if culprits else "全部通过")
                   + (f" | 出瞳出处：{asrc}" if asrc else ""),
            provenance="keepouts.yaml:KO15.fov_deg/apex_world_mm/optical_axis_world + components.yaml:discrepancies.D-COMP-03")



# ── 元件独立实体（F-L3-3，2026-09-13）────────────────────────────────────
def _tol_of(ctx, key):
    t = ((ctx.data.get("tolerances") or {}).get("feature_check_tolerances") or {}).get(key) or {}
    mx = t.get("max")
    src = t.get("src")
    return (float(mx) if isinstance(mx, (int, float)) else None), (f"src={src}" if src else "src 缺")


def _envelope_solid(comp, placed_mesh, geo):
    """components.yaml:components[].envelope_solid → 世界系**独立**实体（manifold）。
    声明：{shape: ring, bore_mm, od_mm, t_mm} | {shape: box, size_mm: [a,b,c], exact: bool} | {shape: none, why}。
    落位只借 placed 实体的 OBB（中心 + 轴向）—— 尺寸一个不抄 CAD/placed。
    返回 (solid|None, note, declared_none: bool, geom: dict)。"""
    import numpy as np
    import trimesh
    from trimesh.transformations import rotation_matrix as rot
    es = comp.get("envelope_solid")
    if not isinstance(es, dict):
        return None, "components.yaml:components[].envelope_solid 未声明（shape: ring|box|none）", False, {}
    shape = es.get("shape")
    if shape == "none":
        return None, f"声明 none：{es.get('why') or '（未写理由）'}", True, {}
    T, ext = trimesh.bounds.oriented_bounds(placed_mesh)
    Tinv = np.linalg.inv(T)
    order = [int(a) for a in np.argsort(ext)]
    if shape == "ring":
        bore, od, t = (_v(es.get(k)) for k in ("bore_mm", "od_mm", "t_mm"))
        if not (all(isinstance(x, (int, float)) for x in (bore, od, t)) and 0 < bore < od and t > 0):
            return None, f"ring 声明不完整/不合法：{es}", False, {}
        ax = order[0]                                           # OBB 最薄的轴 = 环轴
        outer = trimesh.creation.cylinder(radius=od / 2.0, height=float(t), sections=128)
        inner = trimesh.creation.cylinder(radius=bore / 2.0, height=float(t) + 2.0, sections=128)
        R = np.eye(4) if ax == 2 else (rot(math.pi / 2, [0, 1, 0]) if ax == 0 else rot(math.pi / 2, [1, 0, 0]))
        for m in (outer, inner):
            m.apply_transform(Tinv @ R)
        return (geo.wrap(outer) - geo.wrap(inner),
                f"环 Ø{od}/Ø{bore}×{t}（envelope_solid），轴 = placed 实体 OBB 最薄轴", False,
                {"shape": "ring", "bore": float(bore), "od": float(od), "t": float(t), "axis": ax, "ext": [float(e) for e in ext], "T": T})
    if shape == "box":
        size = _v(es.get("size_mm"))
        if not (isinstance(size, (list, tuple)) and len(size) == 3 and all(isinstance(x, (int, float)) and x > 0 for x in size)):
            return None, f"box 声明不完整/不合法：{es}", False, {}
        # 盒的三边落到 OBB 哪根轴：按 overall_mm（该件沿三轴的**总**外形，datasheet；缺省 = size_mm）与 OBB 三轴长度配对。
        # 两个 overall 值差 < 0.5 mm 时按大小排序分不出轴 → ambiguous（不猜；例：舵机 T19.9 与 W20 差 0.1，
        # 必须靠 overall 的 25.7（薄段+厚段差+法兰）vs 20 分辨）。
        overall = _v(es.get("overall_mm")) or size
        if not (isinstance(overall, (list, tuple)) and len(overall) == 3 and all(isinstance(x, (int, float)) and x > 0 for x in overall)):
            return None, f"box.overall_mm 不合法：{overall}", False, {}
        pairs = sorted(zip((float(x) for x in overall), (float(x) for x in size)))   # 按 overall 排序，size 跟着走
        ov_sorted = [a for a, _ in pairs]
        if any(abs(ov_sorted[k + 1] - ov_sorted[k]) < 0.5 and abs(pairs[k + 1][1] - pairs[k][1]) > 1e-9 for k in range(2)):
            return None, (f"box 三边落不到轴上：overall_mm {list(overall)} 里有两个值差 < 0.5 而对应 size_mm 不同，"
                          "按 OBB 长度排序分不出哪根轴是哪边（给 overall_mm 拉开差距，例：舵机 T 轴总长 = 厚段 + 法兰）"), False, {}
        dims = [0.0, 0.0, 0.0]
        for k, axi in enumerate(order):
            dims[axi] = pairs[k][1]
        b = trimesh.creation.box(extents=dims)
        b.apply_transform(Tinv)
        return (geo.wrap(b), f"盒 {list(size)}（envelope_solid；轴按 overall_mm {list(overall)} 与 OBB 三轴长度配对）、同中心", False,
                {"shape": "box", "size_sorted": [b_ for _, b_ in pairs], "overall_sorted": ov_sorted,
                 "exact": bool(es.get("exact")), "ext": [float(e) for e in ext]})
    return None, f"envelope_solid.shape={shape!r} 不认识（ring|box|none）", False, {}


def _placed_covers_envelope(placed_mesh, geom, tol):
    """顶点法（对分段数免疫，F-L2-2 口径）：placed 实体的 OBB 三轴长度 / 环的顶点内半径 vs 声明尺寸。
    返回 (ok, measured: dict, text)。ring 三个数都要在 ±tol；box `exact` 时三边 ±tol，否则三边 ≥ 声明 − tol（⊇）。"""
    import numpy as np
    ext = sorted(geom["ext"])
    if geom["shape"] == "ring":
        T, ax = geom["T"], geom["axis"]
        v = np.asarray(placed_mesh.vertices, float)
        v = (T[:3, :3] @ v.T).T + T[:3, 3]                       # 到 OBB 系
        idx = [k for k in range(3) if k != ax]
        r = np.sqrt(v[:, idx[0]] ** 2 + v[:, idx[1]] ** 2)
        m = {"od_vertex": float(2 * r.max()), "bore_vertex": float(2 * r.min()), "t_extent": float(ext[0])}
        dev = {"od": m["od_vertex"] - geom["od"], "bore": m["bore_vertex"] - geom["bore"], "t": m["t_extent"] - geom["t"]}
        ok = all(abs(x) <= tol for x in dev.values())
        return ok, {**m, "dev": {k: round(x, 6) for k, x in dev.items()}}, \
            f"顶点外径 {m['od_vertex']:.4f} / 顶点内径 {m['bore_vertex']:.4f} / 厚 {m['t_extent']:.4f} vs 声明 {geom['od']}/{geom['bore']}/{geom['t']}"
    srt, ov = geom["size_sorted"], geom["overall_sorted"]
    dev_ov = [ext[k] - ov[k] for k in range(3)]                  # 总外形必须对上（exact 与否都要）
    dev = [ext[k] - srt[k] for k in range(3)]
    ok = all(abs(x) <= tol for x in dev_ov) and (all(abs(x) <= tol for x in dev) if geom["exact"] else all(x >= -tol for x in dev))
    return ok, {"extents_sorted": [round(e, 6) for e in ext], "dev_overall": [round(x, 6) for x in dev_ov], "dev": [round(x, 6) for x in dev]}, \
        f"OBB 三边 {[round(e, 4) for e in ext]} vs 总外形 {ov}（±{tol}）/ 独立实体边 {srt}（{'exact' if geom['exact'] else '⊇'}）"


# ── 冻结项实测（F-数-1，2026-09-13）：frozen.yaml 的几何冻结项对导出件射线/截面核 ────
def _ray_dists(mesh, origin, direction):
    import numpy as np
    loc, _, _ = mesh.ray.intersects_location([origin], [direction], multiple_hits=True)
    if len(loc) == 0:
        return []
    return sorted(float(x) for x in np.linalg.norm(loc - np.asarray(origin, float), axis=1))


def _frozen_checks(ctx, geo, placed_idx, pid2stems, res):
    """frozen.yaml 里能对导出件核的冻结项：battery_bay_26（T01/B01 射线）、h01_sbc_posts（H01 截面）、imu_pose（T01 射线 + MJCF）。
    哪个件由 frozen.yaml 的 measured_on 字段说；没写 → unknown。阈值 tolerances.yaml:feature_check_tolerances.frozen_dimension_mm。
    不能核的项（函数体 / P 字典 / SERVO_SHIFT / 死常量 / 设计间隙）逐条登记 INFO，写明由 build 的 verify_frozen 锁定。"""
    import numpy as np
    fz = ctx.data.get("frozen") or {}
    tol, tsrc = _tol_of(ctx, "frozen_dimension_mm")
    only = ctx.only
    if tol is None:
        res.unknown("_frozen", "threshold", "tolerances.yaml:feature_check_tolerances.frozen_dimension_mm.max 取不到 —— 冻结项无判据",
                    provenance="tolerances.yaml:feature_check_tolerances.frozen_dimension_mm")
        return
    crit = f"|实测 − 冻结值| ≤ {tol} mm（frozen_dimension_mm，{tsrc}）"
    PROV = "frozen.yaml + cad/duck_s288/placed/*.stl（射线/截面）"

    def mesh_of(pid):
        st = (pid2stems.get(pid) or [None])[0]
        return geo.mesh(placed_idx[st]) if st else None

    # hr50（2026-09-28，用户 09-22 解冻电池仓+门、hr48 有意改门：外面 −46.8→−48.8、内面开电池槽到 −45.4）：门板探针退役。
    #   frozen.yaml:battery_bay_26.door_plate_retired{date, decided_by, probes[], regression_now}：列入 probes 的门板侧探针照量、照记数字，
    #   state=RETIRED（基准失去权威、只留记录；不阻断、不计 INCOMPLETE），criterion 写现行回归判据（L2 features B01-F01）。
    #   只允许门板侧探针名；别的名字 / 缺 date、decided_by → 声明无效 → unknown，且不退役任何探针（不许拿它静默仓体探针）。
    DOOR_PROBES = ("DOOR_T", "BX0", "DOOR_YLO", "FOOT_X", "FOOT_Y", "FOOT_Z")
    retired, dpr_note, dpr_reg = set(), "", ""
    dpr = (fz.get("battery_bay_26") or {}).get("door_plate_retired")
    if dpr is not None:
        probes = dpr.get("probes") if isinstance(dpr, dict) else None
        plist = probes if isinstance(probes, list) else []
        bad = [q for q in plist if not isinstance(q, str) or q not in DOOR_PROBES]
        if not isinstance(dpr, dict) or not plist or bad or not dpr.get("date") or not dpr.get("decided_by"):
            res.unknown("_frozen", "door_plate_retired",
                        f"frozen.yaml:battery_bay_26.door_plate_retired 无效（probes={probes!r}，只允许门板侧 {list(DOOR_PROBES)}；"
                        f"date / decided_by 必填）—— 不退役任何探针", provenance="frozen.yaml:battery_bay_26.door_plate_retired")
        else:
            retired = set(plist)
            dpr_note = f"已退役（frozen.yaml:battery_bay_26.door_plate_retired，{dpr.get('date')}，{dpr.get('decided_by')}）"
            dpr_reg = str(dpr.get("regression_now") or "（未写 regression_now）")

    def judge(pid, name, measured, declared, extra=""):
        if name.startswith("battery_bay_26:") and name.split(":", 1)[1] in retired:
            if measured is None:
                mtxt, devs = None, []
                dtxt = f"射线/截面量不到（{extra}）；原冻结值 {declared}"
            elif isinstance(declared, (list, tuple)):
                devs = [float(m) - float(d) for m, d in zip(measured, declared)]
                mtxt = [round(float(m), 4) for m in measured]
                dtxt = f"实测 {mtxt}，原冻结值 {declared}，偏差 {[round(x, 4) for x in devs]}" + (f"；{extra}" if extra else "")
            else:
                devs = [float(measured) - float(declared)]
                mtxt = round(float(measured), 4)
                dtxt = f"实测 {mtxt}，原冻结值 {declared}，偏差 {[round(x, 4) for x in devs]}" + (f"；{extra}" if extra else "")
            res.add(subject=pid, check=f"frozen_{name}", state=RETIRED, severity=INFO, measured=mtxt,
                    criterion=f"{dpr_note}：原判据『{crit}』不再阻断，只记录实测；现行回归判据：{dpr_reg}",
                    evidence_n=max(1, len(devs)), detail=dtxt, provenance=PROV + " / frozen.yaml:battery_bay_26.door_plate_retired")
            return
        if measured is None:
            res.unknown(pid, f"frozen_{name}", f"射线/截面量不到（{extra}）—— 冻结值 {declared} 无法核", provenance=PROV)
            return
        if isinstance(declared, (list, tuple)):
            devs = [float(m) - float(d) for m, d in zip(measured, declared)]
            ok = len(measured) == len(declared) and all(abs(x) <= tol for x in devs)
            mtxt = [round(float(m), 4) for m in measured]
        else:
            devs = [float(measured) - float(declared)]
            ok = abs(devs[0]) <= tol
            mtxt = round(float(measured), 4)
        res.add(subject=pid, check=f"frozen_{name}", state=PASS if ok else FAIL, severity=BLOCK, measured=mtxt,
                criterion=crit, evidence_n=max(1, len(devs)),
                detail=f"冻结值 {declared}，偏差 {[round(x, 4) for x in devs]}" + (f"；{extra}" if extra else ""), provenance=PROV)

    # ── battery_bay_26 ──
    bay = fz.get("battery_bay_26") or {}
    items = {i["name"]: i for i in (bay.get("items") or []) if isinstance(i, dict) and "name" in i}
    val = {n: _v(i.get("value")) for n, i in items.items()}
    mo = bay.get("measured_on") if isinstance(bay.get("measured_on"), dict) else {}
    pt, pd = mo.get("bay"), mo.get("door")
    if items and not (pt and pd):
        res.unknown("_frozen", "battery_bay_26_measured_on",
                    "frozen.yaml:battery_bay_26.measured_on{bay, door} 缺失 —— 不知道对哪两个件核 26 项", provenance="frozen.yaml:battery_bay_26.measured_on")
    elif items:
        mt, md = mesh_of(pt), mesh_of(pd)
        num = lambda k: val.get(k) if isinstance(val.get(k), (int, float)) else None
        BX0, BX1, BZ0, BZ1, BATY, RAILY = (num(k) for k in ("BX0", "BX1", "BZ0", "BZ1", "BAT_Y", "RAIL_Y"))
        geo_ok = all(x is not None for x in (BX0, BX1, BZ0, BZ1, BATY, RAILY))
        if mt is None or md is None or not geo_ok:
            res.unknown(pt or "_frozen", "frozen_battery_bay_26",
                        f"件 {pt}/{pd} 的 placed 实体或 BX0/BX1/BZ0/BZ1/BAT_Y/RAIL_Y 数值缺失（探针原点无从定）", provenance=PROV)
        else:
            c = [(BX0 + BX1) / 2, 0.0, (BZ0 + BZ1) / 2]          # 仓腔中心（冻结值定探针原点，量的是壁）
            if only is None or pt in only:
                px = _ray_dists(mt, c, (1, 0, 0))
                judge(pt, "battery_bay_26:BX1", c[0] + px[0] if px else None, BX1, "+x 射线首命中 = 仓前界")
                judge(pt, "battery_bay_26:BAY_FW", (px[1] - px[0]) if len(px) >= 2 else None, num("BAY_FW"), "+x 射线第 1、2 命中之差 = 前壁厚")
                pz = _ray_dists(mt, c, (0, 0, -1))
                judge(pt, "battery_bay_26:BZ0", c[2] - pz[0] if pz else None, BZ0, "−z 射线首命中 = 仓底")
                ys = [_ray_dists(mt, c, (0, sy, 0)) for sy in (1, -1)]
                judge(pt, "battery_bay_26:BAT_Y", [y[0] for y in ys] if all(y for y in ys) else None, [BATY, BATY], "±y 射线首命中 = 仓半宽")
                judge(pt, "battery_bay_26:RAIL_Y", [y[1] for y in ys] if all(len(y) >= 2 for y in ys) else None, [RAILY, RAILY], "±y 射线第 2 命中 = 导轨外沿")
                ymid = (BATY + RAILY) / 2
                zs = [_ray_dists(mt, (c[0], sy * ymid, BZ0 + 5.0), (0, 0, 1)) for sy in (1, -1)]
                judge(pt, "battery_bay_26:RAIL_Z0", [BZ0 + 5.0 + z[0] for z in zs] if all(z for z in zs) else None,
                      [num("RAIL_Z0")] * 2, f"导轨带 |y|={ymid} 处 +z 射线首命中 = 导轨起始")
                judge(pt, "battery_bay_26:BZ1", [BZ0 + 5.0 + z[1] for z in zs] if all(len(z) >= 2 for z in zs) else None,
                      [BZ1] * 2, f"导轨带 |y|={ymid} 处 +z 射线第 2 命中 = 仓顶")
                sb = val.get("SHELL_BOSS")
                if isinstance(sb, (list, tuple)) and all(isinstance(q, (list, tuple)) and len(q) == 2 for q in sb):
                    spans, miss = [], []
                    for (bx, by) in sb:
                        for sy in (1, -1):
                            h = [BZ1 - 10.0 + d for d in _ray_dists(mt, (float(bx), sy * float(by), BZ1 - 10.0), (0, 0, 1))]
                            h = [z for z in h if z > BZ1 - 10.0 + 1e-6]
                            if len(h) >= 2:
                                spans.append(round(h[-1] - h[-2], 3))
                            else:
                                miss.append(f"({bx},{sy * by})")
                    res.add(subject=pt, check="frozen_battery_bay_26:SHELL_BOSS", state=PASS if not miss else FAIL, severity=BLOCK,
                            measured=len(spans), criterion=f"4 根壳柱在冻结 (x,±y) 处 +z 射线（自仓顶下 10 起）命中有料 = {2 * len(sb)} 处",
                            evidence_n=2 * len(sb),
                            detail=f"命中柱段高 {spans}" + (f"；未命中：{miss}" if miss else "") + "（柱径未冻结，只核位置有料）", provenance=PROV)
            if only is None or pd in only:
                fx, fzr, fy = val.get("FOOT_X"), val.get("FOOT_Z"), num("FOOT_Y")
                if isinstance(fzr, (list, tuple)) and len(fzr) == 2 and isinstance(fx, (list, tuple)) and len(fx) == 2:
                    zmid, xmid = (float(fzr[0]) + float(fzr[1])) / 2, (float(fx[0]) + float(fx[1])) / 2
                    # 底脚：在底脚 x 中点沿 −z 打，进/出 = FOOT_Z；在底脚 z 中点沿 +x 打，最后一次出料 = FOOT_X[1]
                    #（FOOT_X[0] = 仓后界 BX0，底脚与门板在那里连成一体，射线分不开，不单独核）
                    zz = _ray_dists(md, (xmid, 0.0, float(fzr[1]) + 10.0), (0, 0, -1))
                    judge(pd, "battery_bay_26:FOOT_Z", [float(fzr[1]) + 10.0 - zz[1], float(fzr[1]) + 10.0 - zz[0]] if len(zz) >= 2 else None,
                          list(fzr), f"x={xmid} y=0 处 −z 射线首个命中对 = 底脚顶/底")
                    xx = _ray_dists(md, (float(fx[0]) - 20.0, 0.0, zmid), (1, 0, 0))
                    judge(pd, "battery_bay_26:FOOT_X", float(fx[0]) - 20.0 + xx[-1] if xx else None, float(fx[1]),
                          f"z={zmid} y=0 处 +x 射线最后一次出料 = 底脚前端（FOOT_X[0]={fx[0]} 与门板/仓后界连成一体，不单独核）")
                    if fy is not None:
                        yy = _ray_dists(md, (xmid, -30.0, zmid), (0, 1, 0))
                        judge(pd, "battery_bay_26:FOOT_Y", ((yy[-1] - yy[0]) / 2) if len(yy) >= 2 else None, fy, f"x={xmid} z={zmid} 处 +y 射线首末命中之差/2")
                elif isinstance(fzr, (list, tuple)):
                    judge(pd, "battery_bay_26:FOOT_Z", None, list(fzr), "FOOT_X 缺失，定不出底脚探针位置")
                dt = num("DOOR_T")
                dyl = num("DOOR_YLO")
                zlow = BZ0 + 5.0
                # 门板厚与仓后界：沿 +x 穿门板取首个命中对（扫几条 y/z，避开中央窗口）
                pair = None
                for yq in (0.0, 6.0, -6.0, 12.0, -12.0):
                    for zq in (zlow, (BZ0 + BZ1) / 2, BZ1 - 5.0):
                        h = _ray_dists(md, (BX0 - 20.0, yq, zq), (1, 0, 0))
                        if len(h) >= 2:
                            pair = (BX0 - 20.0 + h[0], BX0 - 20.0 + h[1], yq, zq)
                            break
                    if pair:
                        break
                judge(pd, "battery_bay_26:DOOR_T", (pair[1] - pair[0]) if pair else None, dt, f"+x 射线穿门板首个命中对之差 @y={pair[2] if pair else '-'} z={pair[3] if pair else '-'}")
                judge(pd, "battery_bay_26:BX0", pair[1] if pair else None, BX0, "门板内面 x = 仓后界")
                if dyl is not None:
                    xq = BX0 - (dt or 3.0) / 2
                    yy = _ray_dists(md, (xq, -30.0, zlow), (0, 1, 0))
                    note = f"门下段 z={zlow} x={xq:.2f} 处 +y 射线首末命中之差/2"
                    if len(yy) < 2 and pair:
                        # hr50：标准位 x = BX0 − DOOR_T/2 落在 hr48 门内电池槽里（无料），改到实测门板外面往内 1.0 处再量一次（只补量，不改判据）
                        xq = pair[0] + min(1.0, (pair[1] - pair[0]) / 2)
                        yy = _ray_dists(md, (xq, -30.0, zlow), (0, 1, 0))
                        note = f"标准位 x={BX0 - (dt or 3.0) / 2:.2f} 量不到（无料），改在实测门板外面 {pair[0]:.2f} 往内 1.0 处 x={xq:.2f} 量：z={zlow} +y 射线首末命中之差/2"
                    judge(pd, "battery_bay_26:DOOR_YLO", ((yy[-1] - yy[0]) / 2) if len(yy) >= 2 else None, dyl, note)
                sz_, sy_, sd_ = num("SNAP_Z"), num("SNAP_Y"), num("SNAP_D")
                if None not in (sz_, sy_, sd_) and any(abs(sy_ + sgn * sd_ / 2 - w) <= tol for sgn in (1, -1) for w in (BATY, RAILY)):
                    res.unknown(pt, "frozen_battery_bay_26:SNAP",
                                f"卡珠 Ø{sd_} @|y|={sy_} 的两侧面 {sy_ - sd_ / 2}/{sy_ + sd_ / 2} 与导轨面 {BATY}/{RAILY} 差 ≤ {tol}，"
                                "射线分不出卡珠面和导轨面 —— 冻结值无法用射线核（要用截面/曲率）", provenance=PROV)
                elif None not in (sz_, sy_, sd_):
                    found = None
                    for m_, who in ((md, pd), (mt, pt)):
                        for xq in np.arange(BX0 - 8.0, BX1 + 0.01, 0.5):
                            for sgn in (1, -1):
                                h = _ray_dists(m_, (float(xq), sgn * (sy_ + 10.0), sz_), (0, -sgn, 0))
                                yhit = [sy_ + 10.0 - d for d in h]
                                cand = [y for y in yhit if abs(y - (sy_ + sd_ / 2)) <= tol or abs(y - (sy_ - sd_ / 2)) <= tol]
                                if cand:
                                    found = (who, float(xq), sgn, cand[0])
                                    break
                            if found:
                                break
                        if found:
                            break
                    if found:
                        res.add(subject=found[0], check="frozen_battery_bay_26:SNAP", state=PASS, severity=BLOCK, measured=round(found[3], 4),
                                criterion=f"z={sz_} 处沿 ∓y 射线在 |y| = SNAP_Y ± SNAP_D/2 = {sy_} ± {sd_ / 2}（{crit}）处有命中",
                                evidence_n=1, detail=f"命中于 {found[0]} x={found[1]} 侧 {'+' if found[2] > 0 else '-'}y", provenance=PROV)
                    else:
                        res.unknown(pd, "frozen_battery_bay_26:SNAP",
                                    f"z={sz_}、x∈[{BX0 - 8.0},{BX1}] 步 0.5、±y 射线在 {pd}/{pt} 上都没找到 |y|≈{sy_}±{sd_ / 2} 的卡珠面 —— 冻结值无法核", provenance=PROV)
            # 不能对导出件核的项：逐条登记（不假装核过，也不算 unknown —— 它们不是导出件几何）
            non_geo = [n for n in items if n not in {"BX0", "BX1", "BZ0", "BZ1", "BAT_Y", "RAIL_Y", "RAIL_Z0", "BAY_FW", "SHELL_BOSS",
                                                    "FOOT_X", "FOOT_Z", "FOOT_Y", "DOOR_T", "DOOR_YLO", "SNAP_Z", "SNAP_Y", "SNAP_D"}]
            res.add(subject="_frozen", check="battery_bay_26_non_geometric", state=PASS, severity=INFO, measured=len(non_geo),
                    criterion="记录项：不是导出件几何的冻结项（设计间隙 / 死常量 / 停用常量 / SERVO_SHIFT / P 字典 / 函数体），锁定在 build 的 verify_frozen（AST/常量），Gate 不重复核",
                    evidence_n=len(non_geo), detail="、".join(f"{n}（{(items[n].get('meaning') or '')[:24]}）" for n in non_geo),
                    provenance="frozen.yaml:battery_bay_26.items / duckstructure verify_frozen")

    # ── h01_sbc_posts ──
    hp = fz.get("h01_sbc_posts") or {}
    ph = hp.get("measured_on")
    if hp and not ph:
        res.unknown("_frozen", "h01_sbc_posts_measured_on", "frozen.yaml:h01_sbc_posts.measured_on 缺失", provenance="frozen.yaml:h01_sbc_posts.measured_on")
    elif hp and (only is None or ph in only):
        mh = mesh_of(ph)
        n_post, dpost, dpil, pitch = hp.get("post_count"), _v(hp.get("post_d_mm")), _v(hp.get("pilot_d_mm")), _v(hp.get("hole_pitch_mm"))
        if mh is None or not isinstance(n_post, int) or not isinstance(dpost, (int, float)) or not (isinstance(pitch, (list, tuple)) and len(pitch) == 2):
            res.unknown(ph, "frozen_h01_sbc_posts", f"placed 实体或 post_count/post_d_mm/hole_pitch_mm 缺失（{n_post},{dpost},{pitch}）", provenance=PROV)
        else:
            ax = hp.get("post_axis_world") or [1, 0, 0]
            ax = np.asarray(_v(ax), float)
            ax /= np.linalg.norm(ax)
            vb = np.asarray(mh.vertices, float)
            proj = vb @ ax
            x_cut = float(proj.max()) - 3.0                       # 立柱是件上沿柱轴最外的特征，在外沿内 3 mm 切截面
            sec = mh.section(plane_origin=(ax * x_cut).tolist(), plane_normal=ax.tolist())
            loops = []
            if sec is not None:
                for pts in sec.discrete:
                    pts = np.asarray(pts, float)
                    # 去掉轴向分量，取平面内两轴的范围
                    inpl = pts - np.outer(pts @ ax, ax)
                    ext = inpl.max(axis=0) - inpl.min(axis=0)
                    ext = sorted(float(e) for e in ext if e > 1e-9)
                    loops.append((inpl.mean(axis=0), ext))
            outer = [(c, e) for c, e in loops if e and abs(e[-1] - float(dpost)) <= tol and abs(e[0] - float(dpost)) <= tol]
            pil = [(c, e) for c, e in loops if e and isinstance(dpil, (int, float)) and abs(e[-1] - float(dpil)) <= tol]
            res.add(subject=ph, check="frozen_h01_sbc_posts:count", state=PASS if len(outer) == n_post else FAIL, severity=BLOCK,
                    measured=len(outer), criterion=f"沿柱轴外沿内 3 mm 的截面上直径 = post_d_mm {dpost}±{tol} 的闭环数 == post_count {n_post}",
                    evidence_n=len(loops), detail=f"截面闭环 {len(loops)} 个，Ø{dpost} 的 {len(outer)} 个，Ø{dpil} 底孔 {len(pil)} 个", provenance=PROV)
            if len(outer) == n_post and n_post >= 2:
                cs = np.asarray([c for c, _ in outer])
                # 两个面内轴：取与柱轴正交的两个世界轴
                idx = [k for k in range(3) if abs(ax[k]) < 0.5]
                p1 = float(cs[:, idx[0]].max() - cs[:, idx[0]].min())
                p2 = float(cs[:, idx[1]].max() - cs[:, idx[1]].min())
                judge(ph, "h01_sbc_posts:hole_pitch", sorted([p1, p2], reverse=True), sorted([float(pitch[0]), float(pitch[1])], reverse=True),
                      f"{n_post} 根柱心在两面内轴上的极差")
                judge(ph, "h01_sbc_posts:post_d", [e[-1] for _, e in outer], [float(dpost)] * n_post, "各柱截面外径")
                if isinstance(dpil, (int, float)):
                    judge(ph, "h01_sbc_posts:pilot_d", [e[-1] for _, e in pil] if len(pil) == n_post else None, [float(dpil)] * n_post,
                          f"底孔截面 {len(pil)}/{n_post} 个")

    # ── imu_pose ──
    im = fz.get("imu_pose") or {}
    pi_ = im.get("measured_on")
    if im and not pi_:
        res.unknown("_frozen", "imu_pose_measured_on", "frozen.yaml:imu_pose.measured_on 缺失", provenance="frozen.yaml:imu_pose.measured_on")
    elif im:
        mp = _v(im.get("our_mounting_pos_mm"))
        site = _v(im.get("pos_world_mm_zero_pose"))
        loc_m = im.get("pos_local_m")
        delta = _v(im.get("delta_from_site_mm"))
        hd = _v(im.get("mount_hole_d_mm"))
        # (1) 冻结 site 位置 vs 生成 MJCF 的 site（独立产物，会过期）
        try:
            import xml.etree.ElementTree as ET
            xml = ROOT / "sim/duck_s288/robot_walk_s288.xml"
            root = ET.parse(xml).getroot()
            st = next((e for e in root.iter("site") if e.get("name") == im.get("site_name")), None)
            if st is None:
                res.unknown("_frozen", "frozen_imu_pose_vs_mjcf", f"{xml.name} 里没有 site {im.get('site_name')!r}", provenance="frozen.yaml:imu_pose / sim/duck_s288/robot_walk_s288.xml")
            elif isinstance(loc_m, (list, tuple)) and len(loc_m) == 3:
                got = [float(x) * 1000.0 for x in st.get("pos", "").split()]
                want = [float(x) * 1000.0 for x in loc_m]
                res.inputs.append(_rel(xml))
                judge("_frozen", "imu_pose_vs_mjcf", got, want, f"生成 MJCF {xml.name} 的 <site {im.get('site_name')}> pos×1000 vs frozen pos_local_m×1000（body 系）")
            else:
                res.unknown("_frozen", "frozen_imu_pose_vs_mjcf", "frozen.yaml:imu_pose.pos_local_m 不是三元组", provenance="frozen.yaml:imu_pose")
        except Exception as e:                                  # noqa: BLE001
            res.unknown("_frozen", "frozen_imu_pose_vs_mjcf", f"读生成 MJCF 失败：{e}", provenance="sim/duck_s288/robot_walk_s288.xml")
        # (2) delta 算术一致
        if all(isinstance(x, (list, tuple)) and len(x) == 3 for x in (mp, site, delta)):
            judge("_frozen", "imu_pose_delta_consistent", [float(mp[k]) - float(site[k]) for k in range(3)], [float(d) for d in delta],
                  "our_mounting_pos_mm − pos_world_mm_zero_pose vs 记录的 delta_from_site_mm")
        # (3) 安装孔真在 CAD 导出件上：以 our_mounting_pos 为中心沿 ±y 扫径向射线扇，找 Ø=mount_hole_d 的孔
        mtr = mesh_of(pi_)
        if only is None or pi_ in only:
            if mtr is None or not (isinstance(mp, (list, tuple)) and len(mp) == 3) or not isinstance(hd, (int, float)):
                res.unknown(pi_ or "_frozen", "frozen_imu_pose_vs_cad",
                            f"件 {pi_} 的 placed 实体 / our_mounting_pos_mm / mount_hole_d_mm 缺失（{mp}, {hd}）", provenance=PROV)
            else:
                r0 = float(hd) / 2.0
                hits_y = []
                for yq in np.arange(-15.0, 15.0 + 1e-9, 0.25):
                    o = (float(mp[0]), float(yq), float(mp[2]))
                    rs = []
                    for a in np.linspace(0, 2 * math.pi, 16, endpoint=False):
                        d = _ray_dists(mtr, o, (0.0, math.cos(a), math.sin(a)))
                        rs.append(d[0] if d else None)
                    if all(r is not None and abs(r - r0) <= tol for r in rs):
                        hits_y.append(float(yq))
                runs = []
                for yq in hits_y:
                    if runs and abs(yq - runs[-1][-1]) <= 0.26:
                        runs[-1].append(yq)
                    else:
                        runs.append([yq])
                centers = [float(np.mean(r)) for r in runs]
                res.add(subject=pi_, check="frozen_imu_pose_vs_cad", state=PASS if len(centers) == 2 and abs(float(np.mean(centers)) - float(mp[1])) <= tol else FAIL,
                        severity=BLOCK, measured=[round(c, 4) for c in centers],
                        criterion=f"在 x={mp[0]}、z={mp[2]} 的 ±y 扫描线上恰有 2 个 Ø{hd}±{tol} 的孔（16 向径向射线全命中 r={r0}±{tol}），其中心 y 的均值 = our_mounting_pos y {mp[1]} ± {tol}",
                        evidence_n=len(hits_y) * 16 if hits_y else int(30 / 0.25) * 16,
                        detail=f"命中孔心 y {[round(c, 3) for c in centers]}，均值 {round(float(np.mean(centers)), 4) if centers else None}；"
                               f"扫描 y∈[-15,15] 步 0.25，命中站 {len(hits_y)}", provenance=PROV + " / duckstructure/trunk.py IMU 2×M2")

# ── 主流程 ────────────────────────────────────────────────────────────────
# ── 元件固定方式（hr38 新增；用户 2026-09-22 指出这一层以前只有自由文本，机器判不了）──────────
#   以前 components.yaml 只有 mount: 自由文本，"未建模" / "未设计（H04 未出件）" / None 都能蒙混过关，
#   Gate 里没有任何一条判据能说"这个元件是悬空的"。本组把它变成三条可复现判据。
_FIX_KINDS = {"screws", "pocket", "clip", "zip_tie", "press_fit", "rail", "seat", "captive"}


def _fix_tol(ctx, key, dflt):
    t = (ctx.data.get("tolerances") or {}).get("feature_check_tolerances") or {}
    v = t.get(key)
    v = v.get("v") if isinstance(v, dict) else v
    return (float(v), f"tolerances.yaml:feature_check_tolerances.{key}") if isinstance(v, (int, float)) else (dflt, f"内置缺省 {dflt}")


_FIX_SAMPLE_SEED = 0      # hr41 落盘 2026-09-25（Lane C）：fixation_contact / fixation_bearing_dirs 表面采样的固定种子（trimesh Trimesh.sample(…, seed=)）


def _fixation_checks(ctx, geo, comps, claimed, placed_idx, res):
    """每个认领到 placed 实体的元件：① 声明得结构化且解析得到 ② 几何上真挨着承载件 ③ 不是六向皆空。"""
    import numpy as np
    import trimesh
    tol, tsrc = _fix_tol(ctx, "fixation_contact_mm", 1.0)
    reach, rsrc = _fix_tol(ctx, "fixation_reach_mm", 8.0)
    dat = ctx.data
    fids = {f.get("id") for f in (dat.get("fasteners") or {}).get("fasteners") or []}
    feids = {f.get("id") for f in (dat.get("features") or {}).get("features") or []}
    pdata = (dat.get("parts") or {})
    pids = {p.get("id") for p in (pdata.get("parts") or [])} | {p.get("id") for p in (pdata.get("deliberately_added") or [])}
    known = fids | feids | pids
    prov = "components.yaml:components[].fixation"

    stem_of_pid = {}
    for pid in pids:
        for st, path in placed_idx.items():
            if st == pid or st.startswith(pid + "_"):
                stem_of_pid.setdefault(pid, []).append(st)
    # placed 的名字不是件 id，靠 parts.yaml 的 placed_instances 建映射
    for p in (pdata.get("parts") or []) + (pdata.get("deliberately_added") or []):
        inst = p.get("placed_instances")
        if isinstance(inst, list):
            stem_of_pid[p.get("id")] = [n for n in inst if n in placed_idx]

    for c in comps:
        cid = c.get("id")
        stems = claimed.get(cid, [])
        if not stems:
            continue                                     # 没落位的，world_placement 已经红过了
        fx = c.get("fixation")
        if not isinstance(fx, dict):
            res.unknown(cid, "fixation_declared",
                        f"components.yaml 没有结构化 fixation 块（现在只有自由文本 mount: {str(c.get('mount'))[:60]!r}）"
                        f" —— 机器判不了『这个元件靠什么固定住、是不是悬空的』",
                        provenance=prov)
            continue
        kind, by, bp = fx.get("kind"), fx.get("by"), fx.get("bearing_parts")
        bad = []
        if kind not in _FIX_KINDS:
            bad.append(f"kind={kind!r} 不在枚举 {sorted(_FIX_KINDS)}")
        if not isinstance(by, list) or not by:
            bad.append("by 必须是非空列表（螺丝组 id / 特征 id / 件 id）")
        else:
            miss = [b for b in by if b not in known]
            if miss:
                bad.append(f"by 里这些 id 在 fasteners/features/parts 里都查不到：{miss}")
        if not isinstance(bp, list) or not bp:
            bad.append("bearing_parts 必须是非空列表（真正托住它的打印件 id）")
        else:
            miss = [b for b in bp if b not in pids]
            if miss:
                bad.append(f"bearing_parts 里不是打印件 id：{miss}")
        if bad:
            res.add(subject=cid, check="fixation_declared", state=FAIL, severity=BLOCK, measured=len(bad),
                    criterion="fixation: {kind ∈ 枚举, by=[可解析 id], bearing_parts=[打印件 id], dof_note}",
                    evidence_n=len(bad), detail="；".join(bad), provenance=prov)
            continue
        res.add(subject=cid, check="fixation_declared", state=PASS, severity=BLOCK, measured=kind,
                criterion="fixation: {kind ∈ 枚举, by=[可解析 id], bearing_parts=[打印件 id], dof_note}",
                evidence_n=len(by) + len(bp),
                detail=f"kind={kind}；by={by}；bearing_parts={bp}；{str(fx.get('dof_note') or '')[:90]}",
                provenance=prov)

        # ② 真的挨着吗
        try:
            cm = geo.mesh(placed_idx[stems[0]])
            # hr41 落盘 2026-09-25（主设计 Lane C）：采样固定种子 seed=0（hr42_检测提速.md §6 建议①）—— 原 cm.sample(300) 每次取 OS 熵，
            #   同一几何两次跑 30 条里 15 条 measured/detail 不同（hr41 run1/run2）；采样数、判据、阈值都不变，只是可复现
            pts = cm.sample(300, seed=_FIX_SAMPLE_SEED) if cm.area > 0 else np.asarray(cm.vertices)
            best, who = None, None
            for b in bp:
                for st in stem_of_pid.get(b, []):
                    d = float(np.min(trimesh.proximity.ProximityQuery(geo.mesh(placed_idx[st])).on_surface(pts)[1]))
                    if best is None or d < best:
                        best, who = d, f"{b}({st})"
            if best is None:
                res.unknown(cid, "fixation_contact", f"bearing_parts={bp} 在 placed/ 里找不到对应实体", provenance=prov)
            else:
                res.add(subject=cid, check="fixation_contact", state=PASS if best <= tol else FAIL, severity=BLOCK,
                        measured=round(best, 3),
                        criterion=f"元件实体表面到承载件表面的最近距离 ≤ {tol} mm（{tsrc}）—— 声明了固定却离着老远 = 假声明/悬空"
                                  f"（元件表面采样 Trimesh.sample 300 点，seed={_FIX_SAMPLE_SEED} 固定，同几何两次同结果）",
                        evidence_n=len(pts),
                        detail=f"最近的是 {who}，距离 {best:.3f} mm；采样 {len(pts)} 点（seed={_FIX_SAMPLE_SEED}）",
                        provenance=prov + " / cad/duck_s288/placed/")
            # ③ 六向有几向被承载件挡住（= 挡住了几个平动自由度）
            #    **hr38 修正**：原来只从「元件包围盒中心」一点打射线 —— 对"板子大、承载件只托一条边/几根柱"的元件是瞎的：
            #    zz_camera 的中心正好落在四根立柱之间的空档，六向全从 H04-F01 的 13.6 方镜孔和柱间隙穿出去；
            #    zz_adapter 的 −z 射线正好从 H03-F09 那 5 条通风槽漏出去。两者 fixation_contact 都是 **0.000**（真的贴着），却被这条判成 0 向。
            #    改成**从元件表面采样点**打（复用 ② 的采样，降到 64 点），任一点命中即算该方向被挡；起点已在表面，所以距离阈值就是 reach，不再加半外形。
            samp = pts if len(pts) <= 64 else pts[np.linspace(0, len(pts) - 1, 64).astype(int)]
            dirs = [(1, 0, 0), (-1, 0, 0), (0, 1, 0), (0, -1, 0), (0, 0, 1), (0, 0, -1)]
            hit, near = [], {}
            for b in bp:
                for st in stem_of_pid.get(b, []):
                    bm = geo.mesh(placed_idx[st])
                    for i, dv in enumerate(dirs):
                        d3 = np.tile(np.asarray(dv, dtype=float), (len(samp), 1))
                        loc, idx_r, _ = bm.ray.intersects_location(samp, d3, multiple_hits=False)
                        if not len(loc):
                            continue
                        dmin = float(np.min(np.linalg.norm(loc - samp[idx_r], axis=1)))
                        near[i] = min(near.get(i, 1e9), dmin)
                        if dmin <= reach:
                            hit.append(i)
            nd = len(set(hit))
            res.add(subject=cid, check="fixation_bearing_dirs", state=PASS if nd >= 1 else FAIL, severity=WARN,
                    measured=nd,
                    criterion=f"从元件**表面采样点**沿 ±x/±y/±z 打射线，至少 1 个方向在 {reach} mm 内碰到承载件（{rsrc}）；0 向 = 纯悬空。"
                              f"数字本身是信息量（挡住几个平动自由度）——螺丝固定的板通常只有坐面那 1 向"
                              f"（采样点 = fixation_contact 的 300 点等距取 64，seed={_FIX_SAMPLE_SEED} 固定）",
                    evidence_n=6 * len(samp),
                    detail=(f"命中方向 {sorted(set(hit))}（0=+x 1=−x 2=+y 3=−y 4=+z 5=−z），承载件 {bp}；采样 {len(samp)} 点；"
                            + ("各向最近命中 " + "、".join(f"{k}:{v:.2f}" for k, v in sorted(near.items()))
                               if near else "六向射线全部打空（一个方向都没碰到承载件）")),
                    provenance=prov + " / cad/duck_s288/placed/")
        except Exception as e:                            # noqa: BLE001
            res.unknown(cid, "fixation_contact", f"几何校核异常：{e}", provenance=prov)


def run(ctx) -> LayerResult:
    res = LayerResult(LAYER, NAME)
    tol = _tol_max(ctx)
    if tol is None:
        res.unknown("_layer", "threshold", f"{_TOL_KEY}.max 取不到，第 3 层没有判据",
                    provenance=_TOL_KEY)
        return res
    crit = f"交集 ≤ {tol} mm³（{_TOL_KEY}）"

    placed_idx = _index_placed()
    if not placed_idx:
        res.unknown("_layer", "placed_dir", "cad/duck_s288/placed/ 不存在或为空 —— 拿不到世界坐标件",
                    provenance="tools/gate/layers/__init__.py: ctx.placed")
        return res
    geo = _Geo()

    pid2stems, part_stems = _match_parts(ctx, placed_idx, res)
    stem2pid = {s: pid for pid, ss in pid2stems.items() for s in ss}
    only = ctx.only
    for pid in sorted(pid2stems):
        for s in pid2stems[pid]:
            res.inputs.append(_rel(placed_idx[s]))

    comps = (ctx.data.get("components") or {}).get("components") or []
    claimed, cnotes, orphan = _claim_components(comps, placed_idx, part_stems, geo)
    for cid, ss in claimed.items():
        for s in ss:
            res.inputs.append(_rel(placed_idx[s]))

    # ── A 打印件 × 打印件 ──────────────────────────────────────────────
    orig_stems = set(claimed.get("original_prints", []))
    if _MECH.exists():
        res.inputs.append(_rel(_MECH))       # 接缝基线出自这个 json，是一条 BLOCK 判据的输入
    seam = _seam_decl(ctx, pid2stems, orig_stems)
    seam_pair = (seam[0], seam[1], seam[2]) if seam else None

    part_items = sorted((s, stem2pid[s]) for s in part_stems if s in stem2pid)
    if only:
        # 「元件完整实体 × 打印件」和「禁入体 × 打印件」都是**整机量**：少查一个件，
        # 峰值就可能从红变绿（实测 --parts L01,H02 时 KO01 会从 FAIL 214.83 变成 PASS 0）。
        # 所以这两类照全部 24 个实例算，只有逐件的 part_vs_part 受 --parts 影响。
        res.add(subject="_layer", check="scope_vs_parts_filter", state=PASS, severity=WARN,
                measured=sorted(only),
                criterion="--parts 只裁剪逐件判据（part_vs_part），元件完整实体与禁入体一律按全部件算",
                evidence_n=len(only),
                detail="禁入体和元件是整机量，按子集算出来的峰值会把红变成绿，本层不裁剪它们",
                provenance="tools/gate/layers/__init__.py: ctx.only")
    worst_pp = {pid: (0.0, "") for pid in pid2stems}
    over_pp = {pid: [] for pid in pid2stems}
    npairs = {pid: 0 for pid in pid2stems}
    boom = []
    for i, (sa, pa) in enumerate(part_items):
        for sb, pb in part_items[i + 1:]:
            if only and pa not in only and pb not in only:
                continue
            try:
                v = geo.inter(geo.solid(placed_idx[sa]), geo.solid(placed_idx[sb]))
            except Exception as e:                              # noqa: BLE001
                boom.append(f"{sa}×{sb}: {e}")
                continue
            npairs[pa] += 1
            npairs[pb] += 1
            for p in (pa, pb):
                if v > worst_pp[p][0]:
                    worst_pp[p] = (v, f"{sa}×{sb}")
                if v > tol:
                    over_pp[p].append((v, f"{sa}×{sb}"))
    for pid in sorted(pid2stems):
        if only and pid not in only:
            continue
        v, who = worst_pp[pid]
        over = sorted(over_pp[pid], reverse=True)
        res.add(subject=pid, check="part_vs_part", state=PASS if v <= tol else FAIL, severity=BLOCK,
                measured=round(v, 6), criterion=crit, evidence_n=npairs[pid],
                detail=(("超阈值 %d 对：" % len(over)) + "；".join(f"{w} {x:.4f} mm³" for x, w in over[:8])
                        if over else (f"最大交集 {v:.6f} 来自 {who}" if v > 0 else "所有件对交集为 0"))
                       + f"；{npairs[pid]} 个件对（placed 世界坐标，含左右镜像实例）",
                provenance="README 第 2 节 L3 / " + _TOL_KEY)

    # ── A' 显式排除项 —— 排除必须留痕 ──────────────────────────────────
    if len(orig_stems) >= 2:
        # 排除归排除，数还是要量出来摆在这里 —— 不许"悄悄不算"
        pairs, oo = [], sorted(orig_stems)
        for i, a in enumerate(oo):
            for b in oo[i + 1:]:
                try:
                    v = geo.inter(geo.solid(placed_idx[a]), geo.solid(placed_idx[b]))
                    pairs.append(f"{a}×{b} {v:.4f} mm³")
                except Exception as e:                          # noqa: BLE001
                    pairs.append(f"{a}×{b} 布尔失败({e})")
        res.add(subject="original_prints", check="excluded_pair_orig_x_orig", state=NOT_RUN, severity=INFO,
                measured=len(pairs), criterion="原版件之间的接合区不由本项目负责，按声明排除（数照量）",
                evidence_n=len(pairs),
                detail="显式排除：" + "、".join(pairs)
                       + " | 依据 components.yaml:original_prints（原版件 CC BY-SA-NC，原封不动）"
                         "+ frozen.yaml:original_stl_sha256（6 个原版 STL 冻结）"
                         "+ duckstructure/checks.py:run_checks.cosmetic_pair",
                provenance="components.yaml:components[original_prints] / frozen.yaml:original_stl_sha256")
    if seam:
        pid, s, o, seam_ref, decl, refsrc = seam
        if seam_ref is None:
            res.unknown(pid, "original_seam_added",
                        f"{refsrc} 读不到 —— 没有**原版 × 原版**算出来的基线，就无法把'原版自带接缝'"
                        f"与'我们新增的干涉'分开；这一对不做任何排除，按普通件对判（声明原文：{decl}）",
                        provenance="parts.yaml:parts[].mates_with")
        else:
            try:
                cur = geo.inter(geo.solid(placed_idx[s]), geo.solid(placed_idx[o]))
                add, how = _seam_added(ctx, geo, placed_idx, s, o, pid)
                if add is None:
                    # 标量相减是**错的判据**：削掉 X mm³ 原版接缝、同时别处新增 X mm³ 真干涉，
                    # cur - ref 正好抵成 0 → 假绿。tools/cad/mechanical_audit.py:103 用的是布尔差。
                    res.unknown(pid, "original_seam_added",
                                f"拿不到原版件世界网格（{how}），只能标量相减 {cur:.6f}−{seam_ref:.6f}"
                                f"={cur - seam_ref:+.6f} mm³ —— 那个算式会被'一边削一边加'抵消掉，"
                                f"不能当判据（正确做法见 tools/cad/mechanical_audit.py:99-106 的布尔差）",
                                provenance=f"parts.yaml:parts[{pid}].mates_with / {refsrc}")
                else:
                    res.add(subject=pid, check="original_seam_added", state=PASS if add <= tol else FAIL,
                            severity=BLOCK, measured=round(add, 6),
                            criterion=f"新增交集（**布尔差**，不是标量相减）≤ {tol} mm³", evidence_n=2,
                            detail=f"显式排除：{s} × {o} 的原版自带接缝 {seam_ref:.6f} mm³（{refsrc}）；"
                                   f"本次 {s}×{o} 交集 {cur:.6f} mm³；"
                                   f"新增 = ({s}∩{o}) − ({how}∩{o}) 的体积 = {add:.6f} mm³。"
                                   f"声明来自 parts.yaml:parts[{pid}].mates_with『{decl}』",
                            provenance=f"parts.yaml:parts[{pid}].mates_with / {refsrc} / "
                                       f"tools/cad/mechanical_audit.py:99-106")
            except Exception as e:                              # noqa: BLE001
                boom.append(f"{s}×{o}: {e}")

    # ── B 打印件 × 元件完整实体 ────────────────────────────────────────
    for c in comps:
        cid = c.get("id")
        cat = (c.get("category") or "").lower()
        env, ureason, _src = _env_of(c)
        stems = claimed.get(cid, [])
        qty = c.get("qty")

        # B1 envelope 本身能不能机器读
        tri = _triple(env)
        if env is None or ureason:
            res.unknown(cid, "envelope_declared",
                        f"envelope_mm.v={env!r}，unknown_reason={ureason!r} —— "
                        "第 3 层不拿占位方块假装查过（元规则 4）",
                        provenance="components.yaml:components[].envelope_mm")
        elif tri is None:
            res.unknown(cid, "envelope_machine_readable",
                        f"envelope_mm 是自由文本，机器解析不出实体尺寸：{str(env)[:70]}…",
                        provenance="components.yaml:components[].envelope_mm", severity=WARN)
        else:
            res.add(subject=cid, check="envelope_machine_readable", state=PASS, severity=INFO,
                    measured=list(tri), criterion="envelope_mm 能解析成 a×b×c 三元组", evidence_n=1,
                    detail=f"来自 {str(env)[:60]}", provenance="components.yaml:components[].envelope_mm")

        # B2 有没有世界落位
        if not stems:
            res.unknown(cid, "world_placement",
                        f"components.yaml 不写元件世界坐标，placed/ 里也没有按 claim 认领到的实体 —— "
                        f"第 3 层无法把它放到位（category={cat or '-'}, qty={qty}；{cnotes.get(cid, '')}）",
                        provenance="components.yaml 文件头：『不写元件的世界坐标』")
            continue
        res.add(subject=cid, check="world_placement", state=PASS, severity=INFO,
                measured=len(stems), criterion="能在 placed/ 里认领到完整实体", evidence_n=len(stems),
                detail=f"{cnotes.get(cid,'')}；实体：{', '.join(stems)}",
                provenance="cad/duck_s288/placed/")
        if isinstance(qty, int) and qty != len(stems):
            res.add(subject=cid, check="instance_count", state=FAIL, severity=BLOCK,
                    measured=len(stems), criterion=f"落位实体数 = components.yaml qty = {qty}",
                    evidence_n=len(stems),
                    detail=f"认领到 {len(stems)} 个：{', '.join(stems)}",
                    provenance="components.yaml:components[].qty")

        # B3 完整实体 × 每个打印件 —— 这一条就是 4 颗 6704 的可复现判据
        #    **实体与 CAD 同源**：placed/ 里的 servo__* / bearing_* / zz_battery 是 build 导出的占位体，载体对自家舵机为 0
        #    是 servo_env 挖过的构造保证（审计 F-L3-3）。所以下面 B4 再用 components.yaml:envelope_solid 独立建一份实体复查。
        crit_same = crit + "（元件实体 = build 导出的 placed 占位体，与 CAD 同源；独立复查见 part_vs_component_envelope）"
        worst, nb, hits = 0.0, 0, []
        pairs_pid = {}
        for cs in stems:
            for ps, pid in part_items:                       # 整机量：不按 --parts 裁剪，理由见 _layer/scope
                if seam_pair and cid == "original_prints" and ps == seam_pair[1] and cs == seam_pair[2]:
                    continue                                    # 已在 original_seam_added 里单独按基线判
                try:
                    v = geo.inter(geo.solid(placed_idx[cs]), geo.solid(placed_idx[ps]))
                except Exception as e:                          # noqa: BLE001
                    boom.append(f"{cs}×{ps}: {e}")
                    continue
                nb += 1
                pairs_pid[pid] = pairs_pid.get(pid, 0) + 1
                worst = max(worst, v)
                if v > tol:
                    hits.append((v, f"{cs} × {ps}({pid})"))
        hits.sort(reverse=True)
        # 违规同样要落到件的格子：不然本层的招牌案例（4 颗 6704）真复现时，L01/L02 那两格还是绿的
        bypid = {}
        for v, w in hits:
            bypid.setdefault(w.rsplit("(", 1)[-1].rstrip(")"), []).append((v, w))
        for pid, rows in sorted(bypid.items()):
            rows.sort(reverse=True)
            res.add(subject=pid, check=f"component_{cid}", state=FAIL, severity=BLOCK,
                    measured=round(rows[0][0], 6), criterion=crit_same, evidence_n=pairs_pid.get(pid, len(rows)),
                    detail=f"元件 {cid} 的**完整实体**压进本件：{len(rows)} 处超阈值（查过 {pairs_pid.get(pid, 0)} 对），"
                           + "；".join(f"{w} {v:.4f} mm³" for v, w in rows[:5]),
                    provenance="README 第 2 节 L3 / keepouts.yaml:KO21 / " + _TOL_KEY)
        res.add(subject=cid, check="part_vs_component_solid", state=PASS if worst <= tol else FAIL,
                severity=BLOCK, measured=round(worst, 6), criterion=crit_same, evidence_n=nb,
                detail=(("超阈值 %d 对：" % len(hits)) + "；".join(f"{w} {v:.4f} mm³" for v, w in hits[:8])
                        if hits else f"全部 ≤ {tol}（峰值 {worst:.6f}）")
                       + f"；{nb} 个「完整实体 × 打印件」对。**查的是整颗实体不是截面** —— "
                         "4 颗 6704 各 ~402 mm³ 的历史事故就是只查孔径截面漏掉的",
                provenance="README 第 2 节 L3 / keepouts.yaml:KO21 / " + _TOL_KEY)

        # B4 独立实体（F-L3-3）：components.yaml:envelope_solid 建的环/盒，落位只借 placed 的 OBB；
        #    (a) placed ⊇ 声明包络（顶点法，对分段数免疫）；(b) 独立实体 × 每个打印件。
        etol, etsrc = _tol_of(ctx, "envelope_vs_placed_mm")
        worst_e, nb_e, hits_e, notes_e, pairs_e, cover = 0.0, 0, [], [], {}, []
        declared_none = False
        for cs in stems:
            env, note, declared_none, eg = _envelope_solid(c, geo.mesh(placed_idx[cs]), geo)
            if declared_none:
                break
            if env is None:
                notes_e.append(f"{cs}: {note}")
                continue
            if etol is None:
                cover.append((cs, None, "tolerances.yaml:feature_check_tolerances.envelope_vs_placed_mm.max 取不到"))
            else:
                ok_c, m_c, txt_c = _placed_covers_envelope(geo.mesh(placed_idx[cs]), eg, etol)
                cover.append((cs, ok_c, txt_c))
            for ps, pid in part_items:
                try:
                    v = geo.inter(env, geo.solid(placed_idx[ps]))
                except Exception as e:                          # noqa: BLE001
                    boom.append(f"env:{cs}×{ps}: {e}")
                    continue
                nb_e += 1
                pairs_e[pid] = pairs_e.get(pid, 0) + 1
                worst_e = max(worst_e, v)
                if v > tol:
                    hits_e.append((v, f"env({cs}) × {ps}({pid})"))
        if declared_none:
            res.add(subject=cid, check="envelope_solid_declared", state=PASS, severity=INFO, measured="none", evidence_n=1,
                    criterion="components.yaml:envelope_solid.shape=none 时不建独立实体（要写 why）", detail=note,
                    provenance="components.yaml:components[].envelope_solid")
        elif notes_e and not cover:
            res.unknown(cid, "envelope_solid_declared",
                        "独立实体建不出来：" + "；".join(notes_e[:3]) + " —— 元件实体只剩与 CAD 同源的 placed 占位体，独立复查没跑",
                        provenance="components.yaml:components[].envelope_solid")
        else:
            bad = [x for x in cover if x[1] is False]
            unk = [x for x in cover if x[1] is None]
            if unk:
                res.unknown(cid, "placed_covers_envelope", unk[0][2], provenance="tolerances.yaml:feature_check_tolerances.envelope_vs_placed_mm")
            else:
                res.add(subject=cid, check="placed_covers_envelope", state=PASS if not bad else FAIL, severity=BLOCK,
                        measured=len(bad), criterion=f"每个 placed 实体的尺寸（顶点法：OBB 三边 / 环的顶点内外径）对上 envelope_solid 声明，偏差 ≤ {etol} mm"
                                                     f"（envelope_vs_placed_mm，{etsrc}）；box 非 exact 时三边 ≥ 声明 − {etol}（⊇）",
                        evidence_n=len(cover), detail="；".join(f"{cs}: {'OK' if ok else 'NG'} {txt}" for cs, ok, txt in cover[:4]),
                        provenance="components.yaml:components[].envelope_solid + cad/duck_s288/placed/")
            hits_e.sort(reverse=True)
            bypid_e = {}
            for v, w in hits_e:
                bypid_e.setdefault(w.rsplit("(", 1)[-1].rstrip(")"), []).append((v, w))
            for pid, rows in sorted(bypid_e.items()):
                rows.sort(reverse=True)
                res.add(subject=pid, check=f"component_envelope_{cid}", state=FAIL, severity=BLOCK,
                        measured=round(rows[0][0], 6), criterion=crit + "（独立实体：components.yaml:envelope_solid，不与 CAD 同源）",
                        evidence_n=pairs_e.get(pid, len(rows)),
                        detail=f"元件 {cid} 按声明独立建的实体压进本件：{len(rows)} 处超阈值（查过 {pairs_e.get(pid, 0)} 对），"
                               + "；".join(f"{w} {v:.4f} mm³" for v, w in rows[:5]),
                        provenance="components.yaml:components[].envelope_solid / " + _TOL_KEY)
            if nb_e:
                res.add(subject=cid, check="part_vs_component_envelope", state=PASS if worst_e <= tol else FAIL, severity=BLOCK,
                        measured=round(worst_e, 6), criterion=crit + "（独立实体：components.yaml:envelope_solid 建的环/盒，落位借 placed 的 OBB，尺寸不抄 CAD）",
                        evidence_n=nb_e,
                        detail=(("超阈值 %d 对：" % len(hits_e)) + "；".join(f"{w} {v:.4f} mm³" for v, w in hits_e[:8])
                                if hits_e else f"全部 ≤ {tol}（峰值 {worst_e:.6f}）") + f"；{nb_e} 个「独立实体 × 打印件」对；{note}",
                        provenance="components.yaml:components[].envelope_solid / " + _TOL_KEY)

    # hr38：元件固定方式判据（以前只有 mount: 自由文本，机器判不了悬空）
    _fixation_checks(ctx, geo, comps, claimed, placed_idx, res)

    # 元件里的 PH2.0 插头实体走 KO01；带头螺丝没有坐标源
    res.unknown("fasteners", "screw_head_solids",
                "fasteners.yaml 30 组 157 颗只有 location/holes 的自然语言描述，没有任何孔的数值坐标，"
                "features.yaml 的 spec_verbatim 也被明确禁止当判据 —— 带头螺丝的完整实体建不出来，"
                "第 3 层查不了『螺丝头会不会顶到别的件』",
                provenance="fasteners.yaml:fasteners[].location / features.yaml 文件头")

    if orphan:
        res.add(subject="_placed", check="unclaimed_placed_solids", state=FAIL, severity=WARN,
                measured=len(orphan), criterion="placed/ 里每个实体都要能对上 parts.yaml 或 components.yaml",
                evidence_n=len(orphan),
                detail="认不出归属的 placed 实体：" + ", ".join(sorted(orphan)),
                provenance="cad/duck_s288/placed/")

    # ── C 打印件 × 禁入体 ──────────────────────────────────────────────
    kos = (ctx.data.get("keepouts") or {}).get("keepouts") or []
    try:
        import numpy as _np
        _bs = _np.asarray([geo.mesh(placed_idx[s_]).bounds for s_ in placed_idx], float)
        world_bounds = (_bs[:, 0, :].min(axis=0), _bs[:, 1, :].max(axis=0)) if len(_bs) else None
    except Exception:                                           # noqa: BLE001
        world_bounds = None
    try:
        built = _ko_geoms(ctx, geo, res, world_bounds=world_bounds)
    except Exception as e:                                      # noqa: BLE001
        built = {}
        res.unknown("_keepouts", "build", f"禁入体实体构造整体失败：{e}", provenance="keepouts.yaml")
    _ko_camera_rays(ctx, geo, placed_idx, res)

    deferred = _deferred(kos)
    for ko in kos:
        kid = ko.get("id")
        if kid == "KO15":
            continue                                            # 已由射线检查处理
        status = ko.get("status")
        if kid == "KO21":
            # 「完整实体」这条的判据已经落在各轴承的 part_vs_component_solid 上，这里只登记覆盖情况。
            # 轴承按 components.yaml 的 category 找，不写死型号名（换轴承只改 data）。
            bcat = [c.get("id") for c in comps if (c.get("category") or "").lower() == "bearing"]
            hits = [c for c in bcat if c in claimed]
            n = sum(len(claimed[c]) for c in hits)
            if not n:
                res.unknown(kid, "keepout_full_solid",
                            f"components.yaml 里 category=bearing 的元件 {bcat} 一个实体都没认领到 —— "
                            "『轴承装得进、不被削』这条没跑",
                            provenance="keepouts.yaml:KO21 / components.yaml")
                continue
            res.add(subject=kid, check="keepout_full_solid", state=PASS, severity=INFO, measured=n,
                    criterion="6704/6700 的完整环实体参加静态交集（判据体现在各轴承的 part_vs_component_solid）",
                    evidence_n=n,
                    detail=f"完整环实体 {n} 颗：{', '.join(sum((claimed[c] for c in hits), []))}",
                    provenance="keepouts.yaml:KO21")
            continue
        if kid in deferred:
            layer, why = deferred[kid]
            res.add(subject=kid, check="scope", state=NOT_RUN, severity=INFO, measured=None,
                    criterion=f"归第 {layer} 层（keepouts.yaml:{kid}.checked_in_layer；义务由 core.obligations 派给该层，"
                              f"covered_by_subjects 的格子由 core.build_scorecard 汇总成 L{layer}/{kid}）", evidence_n=0,
                    detail=f"{why}；零位姿静态交集对它没有意义，第 3 层不代跑也不代过。"
                           f"现状 status={status}，checked_by={ko.get('checked_by')}",
                    provenance=f"keepouts.yaml:{kid}.checked_in_layer / README 第 2 节 L{layer}")
            continue
        g = built.get(kid)
        if not g:
            res.unknown(kid, "keepout_solid",
                        f"geom_verbatim={ko.get('geom_verbatim')!r} / frame={ko.get('frame')!r} "
                        f"确定不出唯一实体（缺中心/缺起点/缺一个轴的区间/写着无模型）—— "
                        f"不臆造几何。status={status}, checked_by={ko.get('checked_by')}",
                        provenance=f"keepouts.yaml:{kid}")
            continue
        # owner_parts 只说这条禁入体"归谁管"，**不是**"只有这几个件需要查"——
        # 禁入体是谁都不许进的区域，缩到 owner 上会漏掉别的件闯进来（实测：按 owner 缩范围后
        # KO12 就再也看不到电池门那 3034 mm³ 了）。所以一律查全部打印件，owner 只写进 detail。
        owner = str(ko.get("owner_parts") or "")
        owned = sorted({t for t in re.split(r"[/、,，\s]+", owner) if t in pid2stems})
        targets = None
        g["targets"] = None
        # 层实际用的坐标系必须和 data 声明的 frame 对得上，对不上要留痕 ——
        # "把局部坐标当世界坐标"正是历史上"空刀"的来历，不能无声吞掉。
        declared, used = ko.get("frame"), g.get("used_frame")
        if used and declared and declared != used:
            res.add(subject=kid, check="frame_declared_vs_used", state=FAIL, severity=WARN,
                    measured=[declared, used],
                    criterion="keepouts.yaml 的 frame 必须和层实际用的坐标系一致",
                    evidence_n=len(g["solids"]),
                    detail=f"data 声明 frame={declared!r}，但 geom_verbatim 里的数值只有按 {used!r} 解释"
                           f"才落在本件上（已数值核对）。现在结果是对的，但哪天有人'修好' data 让坐标"
                           f"真变成 {declared!r}，本层照旧按 {used!r} 用 = 一个上百 mm 的空刀 + 绿格。"
                           f"要么改 data 的 frame，要么把层改成按 frame 分派",
                    provenance=f"keepouts.yaml:{kid}.frame")
        if g.get("extra_unknown"):
            res.unknown(kid, g["extra_unknown"][0], g["extra_unknown"][1],
                        provenance=f"keepouts.yaml:{kid}.geom_verbatim")
            if not g.get("solids"):
                continue                                        # 建不出实体就没有交集可判，不许留一条 0 对的 PASS
        exempt = g.get("exempt") or {}
        # F-L3-1（2026-09-13）：禁入柱体是**外接 128 边形**（比真圆略大，方向偏保守）。刻面多出来的体积上界
        # 写进 criterion 与一条 INFO，让读记分卡的人知道 0.0x mm³ 的红有多少是刻面；**阈值不动、多边形不动**
        #（不许改成和 s288.cyl 同源的 64 边内接 —— 那是同源验证，元规则 3）。
        crit_ko = crit
        fn = g.get("facet_noise_mm3")
        if fn is not None:
            crit_ko = (crit + f"；禁入柱体按外接 128 边形建，刻面噪声上限 ≈ {fn:.4f} mm³"
                              f"（环带 π((r/cos(π/128))²−r²)×长度，只会多报不会漏报，不据此放宽阈值）")
            decl = ko.get("facet_noise_mm3")
            dv = decl.get("v") if isinstance(decl, dict) else decl
            res.add(subject=kid, check="keepout_facet_noise", state=PASS, severity=INFO, measured=round(fn, 6),
                    criterion="记录值：外接 128 边形禁入柱体比真圆多出的体积上界（mm³）；与 keepouts.yaml:facet_noise_mm3 对照",
                    evidence_n=len(g["solids"]),
                    detail=(f"keepouts.yaml 记 {dv}（差 {abs(float(dv) - fn):.6f}）" if isinstance(dv, (int, float))
                            else "keepouts.yaml 没写 facet_noise_mm3（人读的字段，不影响判据）")
                           + f"；阈值 {tol} mm³ 不因它放宽，超阈值的红要看 measured 与它的量级",
                    provenance=f"keepouts.yaml:{kid}.facet_noise_mm3 / l3_static._facet_noise_mm3")
        if g.get("cad_cutters"):
            # F-L3-2 对照：独立实体 vs CAD 切刀（conn_cut）的对称差体积，只报不判 —— 差异 = 数据声明与 CAD 切刀口径不同
            #（例：切刀往机身里多进 0.5、窗口薄板 6.6×9.2 大于走廊截面 5.5×8.8），不是侵入。
            diffs, broken = [], []
            if g.get("cad_cutters_by_side"):
                # KO01（09-20）：实体按 侧/区 拆（tag "servo[i]±y/A"），CAD 切刀按侧合体 → 同侧实体先并起来再对照
                by_side = {}
                for tag, sol in g["solids"]:
                    by_side.setdefault(tag.rsplit("/", 1)[0], []).append(sol)
                pairs = []
                for side, sols in by_side.items():
                    u = sols[0]
                    for s_ in sols[1:]:
                        u = u + s_
                    pairs.append((side, u))
            else:
                pairs = list(g["solids"])
            for tag, sol in pairs:
                cc = g["cad_cutters"].get(tag)
                if cc is None or isinstance(cc, str):
                    broken.append(f"{tag}: {cc or '无'}")
                    continue
                try:
                    diffs.append((tag, float((sol - cc).volume()), float((cc - sol).volume())))
                except Exception as e:                          # noqa: BLE001
                    broken.append(f"{tag}: {e}")
            res.add(subject=kid, check=f"keepout_{kid}_vs_cad_cutter", state=PASS, severity=INFO,
                    measured=round(max((a for _, a, _ in diffs), default=0.0), 6),
                    criterion="记录值：独立实体 − CAD 切刀（conn_cut）的体积（mm³，逐处最大）与反向差；只报不判，判据在 keepout_intersection（独立实体 × 打印件）",
                    evidence_n=len(diffs),
                    detail="；".join(f"{t} 独立−CAD {a:.3f} / CAD−独立 {b:.3f}" for t, a, b in diffs[:6])
                           + (f"；CAD 切刀建不出 {len(broken)} 处：{'; '.join(broken[:3])}" if broken else ""),
                    provenance=g["prov"] + " / duckstructure.lib.conn_cut（对照）")
        if g.get("window_faces"):
            # (a) 窗口面片：采样点落在打印件实体内 = 面上有料。口袋壁隔着间隙盖住窗口边缘不算（插座不外凸）。
            import numpy as np
            inside_by = {}
            n_pts = 0
            for tag, pw in g["window_faces"]:
                n_pts += len(pw)
                for ps, pid in part_items:
                    m_ = geo.mesh(placed_idx[ps])
                    lo, hi = m_.bounds
                    sel = np.all((pw >= lo - 1e-6) & (pw <= hi + 1e-6), axis=1)
                    if not sel.any():
                        continue
                    try:
                        inside = np.asarray(m_.contains(pw[sel]))
                    except Exception as e:                      # noqa: BLE001
                        boom.append(f"{kid}/window {tag}×{ps}: {e}")
                        continue
                    if inside.any():
                        inside_by.setdefault(pid, []).append((int(inside.sum()), f"{tag} × {ps}"))
            tot = sum(n for rows in inside_by.values() for n, _ in rows)
            for pid, rows in sorted(inside_by.items()):
                rows.sort(reverse=True)
                res.add(subject=pid, check=f"keepout_{kid}_window_face", state=FAIL, severity=BLOCK,
                        measured=sum(n for n, _ in rows), criterion="插座窗口面片上的采样点落在打印件实体内的数量 = 0（面上不许有任何材料）",
                        evidence_n=n_pts, detail="；".join(f"{w} {n} 点" for n, w in rows[:5]), provenance=g["prov"])
            res.add(subject=kid, check=f"keepout_{kid}_window_face", state=PASS if tot == 0 else FAIL, severity=BLOCK, measured=tot,
                    criterion="插座窗口面片（geom_verbatim (a)，dims.conn_x_mm × conn_z_mm @ y=±y0）上按 window_face_grid_mm 取、离面 window_face_probe_offset_mm 的采样点，"
                              "落在任一打印件实体内（trimesh contains）的数量 = 0；面前留有 ≥ offset 间隙的口袋壁不算面上有料（插座不外凸）",
                    evidence_n=n_pts * len(part_items),
                    detail=f"{len(g['window_faces'])} 个窗口面 × {n_pts // max(1, len(g['window_faces']))} 点/面 × {len(part_items)} 件"
                           + ("；有料：" + "；".join(f"{w} {n}" for rows in inside_by.values() for n, w in rows[:3]) if tot else "；全部为空"),
                    provenance=g["prov"])
        hits, hits_ex = [], []
        by_pid = {}
        pairs_pid = {}
        nb, worst = 0, 0.0
        items = part_items
        if g.get("obstacles") == "all_placed":
            # 扫掠型（hr41g）：障碍 = placed/ 全部实体（打印件 + 元件）− 随行件；随行件名先落到 placed 实体（实体名或件号）
            skip, bad_mv = set(), []
            for mname in (g.get("moving") or []):
                if mname in placed_idx:
                    skip.add(mname)
                elif mname in pid2stems:
                    skip |= set(pid2stems[mname])
                else:
                    bad_mv.append(mname)
            if bad_mv:
                res.unknown(kid, "keepout_sweep_declared",
                            f"moving_components 里 {bad_mv} 在 placed/ 里既不是实体名也不是件号 —— 随行件认不出，"
                            f"扫掠会把它当障碍或漏掉它，不代跑", provenance=g["prov"])
                continue
            items = [(s_, stem2pid.get(s_, s_)) for s_ in sorted(placed_idx) if s_ not in skip]
        for tag, sol in g["solids"]:
            st = (g.get("solid_targets") or {}).get(tag)      # 逐实体的目标件（KO01 的 W 只查载体）；None = 全部
            for ps, pid in items:                            # 整机量：不按 --parts 裁剪
                if targets and pid not in targets:
                    continue
                if st is not None and pid not in st:
                    continue
                try:
                    v = geo.inter(sol, geo.solid(placed_idx[ps]))
                except Exception as e:                          # noqa: BLE001
                    boom.append(f"{kid}/{tag}×{ps}: {e}")
                    continue
                nb += 1
                pairs_pid[pid] = pairs_pid.get(pid, 0) + 1
                ex = _exempted(exempt, pid, tag)
                if not ex:
                    worst = max(worst, v)                        # 峰值要含未超阈值的对，否则通过时恒报 0
                if v <= tol:
                    continue
                if ex:
                    hits_ex.append((v, f"{tag} × {ps}({pid})"))
                else:
                    hits.append((v, f"{tag} × {ps}({pid})"))
                    by_pid.setdefault(pid, []).append((v, f"{tag} × {ps}"))
        hits.sort(reverse=True)
        hits_ex.sort(reverse=True)
        # 违规也要落到**件**的格子上，否则 17 件 × 8 层那张表会全绿，人一眼看过去以为没事
        if not g.get("ambiguous"):
            for pid, rows in sorted(by_pid.items()):
                if pid not in pid2stems:
                    continue                                 # 元件（扫掠型障碍）没有件格，记在禁入体格的 detail 里
                rows.sort(reverse=True)
                res.add(subject=pid, check=f"keepout_{kid}", state=FAIL, severity=BLOCK,
                        measured=round(rows[0][0], 6), criterion=crit_ko, evidence_n=pairs_pid.get(pid, len(rows)),
                        detail=f"{kid}（{ko.get('name')}）：{len(rows)} 处超阈值（查过 {pairs_pid.get(pid, 0)} 对），" +
                               "；".join(f"{w} {v:.4f} mm³" for v, w in rows[:5]),
                        provenance=g["prov"])
        lst = "；".join(f"{w} {v:.4f} mm³" for v, w in hits[:8]) or "无"
        exl = ("；**按声明排除**（仍记数）：" + "；".join(f"{w} {v:.4f} mm³" for v, w in hits_ex[:4])
               + "；理由 " + " / ".join(f"{k}@{r[0]}: {r[1]}" for k, r in exempt.items())) if hits_ex else \
              ("；声明的排除项 " + " / ".join(f"{k}@{r[0]}: {r[1]}" for k, r in exempt.items()) + " 本次没有触发"
               if exempt else "")
        if g.get("ambiguous"):
            res.add(subject=kid, check="keepout_geometry_ambiguous", state=FAIL, severity=BLOCK,
                    measured=round(worst, 6),
                    criterion="geom_verbatim 必须是可复现的几何声明，不能是代码符号",
                    evidence_n=nb,
                    detail=f"geom_verbatim={ko.get('geom_verbatim')!r} 是 duckstructure/lib.py 里的切刀对象名，"
                           f"不是 data 里的几何。按字面建实体量到 {worst:.4f} mm³（{lst}），"
                           f"但那**不是**侵入 —— |y|≥19 那整片里本来就合法地住着躯干壳 T02/T03 和大腿件，"
                           f"真正的切刀是『壳外扩 SHELL_CLR 后再与该片相交』那一小块。"
                           f"等价的可判据是 T01 × T02/T03 零位交集，本层 part_vs_part 已覆盖（实测 0）。"
                           f"data 缺字段 → 判红（元规则 4）。{g['note']}",
                    provenance=g["prov"])
            continue
        res.add(subject=kid, check="keepout_intersection", state=PASS if worst <= tol else FAIL,
                severity=BLOCK, measured=round(worst, 6), criterion=crit_ko, evidence_n=nb,
                detail=f"{g['note']}；{nb} 个「禁入体 × {'打印件' if items is part_items else 'placed 实体'}」对"
                       + (f"；超阈值 {len(hits)} 对：{lst}" if hits
                          else f"；无一超阈值，最大交集 {worst:.6f} mm³")
                       + ((f"；data 声明 owner_parts={owner!r}（解析到 {owned}），"
                           f"但本条对**全部 24 个打印件实例**都查了" if owner else "")
                          if items is part_items else
                          f"；障碍 = placed/ 全部 {len(items)} 个实体（打印件+元件，已扣随行件 {g.get('moving')}）")
                       + exl,
                provenance=g["prov"])
        if g.get("sweep"):
            # 终点脱离：行程不够、终点还在别的实体影子里 = 没证明抽得出来（FG11 同理；exempt 的件拆卸态不在场，不参与）
            import numpy as np
            sw = g["sweep"]
            unproven = []
            for ps, pid in items:
                if _exempted(exempt, pid, ""):
                    continue
                b = geo.mesh(placed_idx[ps]).bounds
                if not _sep_along(sw["end"], (np.asarray(b[0], float), np.asarray(b[1], float)), sw["d"]):
                    unproven.append(ps)
            res.add(subject=kid, check="keepout_sweep_clear_at_end", state=PASS if not unproven else FAIL,
                    severity=BLOCK, measured=len(unproven),
                    criterion="扫掠终点的包络盒必须已证明脱离全部障碍（沿方向整体越过，或 ⊥方向投影分离；AABB 只用来证明，"
                              "与 l4_assembly._proven_clear_after 同口径）—— 行程不够等于没证明抽得出来",
                    evidence_n=len(items),
                    detail=(f"终点包络 {np.round(sw['end'][0], 4).tolist()}..{np.round(sw['end'][1], 4).tolist()}；"
                            + (f"未证明脱离 {len(unproven)} 个：{unproven[:8]}" if unproven
                               else f"对 {len(items)} 个障碍（扣 exempt）全部证明脱离")),
                    provenance=g["prov"])

    # ── D 冻结项实测（F-数-1）──────────────────────────────────────────
    try:
        _frozen_checks(ctx, geo, placed_idx, pid2stems, res)
    except Exception as e:                                      # noqa: BLE001
        res.unknown("_frozen", "frozen_checks", f"冻结项核对整体失败：{e}", provenance="frozen.yaml")

    if boom:
        res.unknown("_boolean", "boolean_failures",
                    f"{len(boom)} 对布尔运算失败（不当作 0 通过）：" + " | ".join(boom[:5]),
                    provenance="duckstructure/checks.py:vol 的做法：布尔炸了必须中止")
        # 失败的那一对既没进证据也没进峰值，如果只记在 _boolean 上，涉事的件格照样是绿的
        for line in boom:
            who = line.split(":")[0]
            for token in re.split(r"[×/]", who):
                pid = stem2pid.get(token.strip())
                if pid:
                    res.unknown(pid, "boolean_failed",
                                f"涉及本件的布尔运算失败，这一对没有结果：{line[:160]}",
                                provenance="duckstructure/checks.py:vol")

    res.evidence = {"placed_solids": len(placed_idx),
                    "printed_instances": len(part_items),
                    "component_instances": sum(len(v) for v in claimed.values()),
                    "booleans_run": geo.booleans,
                    "aabb_prefiltered": geo.aabb_skips,
                    "keepouts_total": len(kos),
                    "keepouts_implemented": len([k for k, g in built.items() if not g.get("ambiguous")]) + 2,
                    "keepouts_deferred": len(deferred)}
    return res
