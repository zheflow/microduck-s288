#!/usr/bin/env python3
"""第 5 层 功能达成 —— 装上之后它真的干活吗。

这一层是 H02 事故的克星：压板四周留 0.30 mm 均匀间隙，第 3 层"干涉 = 0"完美通过，
但压板什么都没夹住。所以本层只问功能量：咬入够不够、螺丝会不会顶底、坐面平不平、
过盈/间隙/同心落没落在公差表的桶里、舵盘相位对不对。

判据来源（一律 tolerances.yaml / relations.yaml，层里不写死数字）：
  engagement_by_joint_type.<joint_type>.range_mm     咬入范围**按连接类型**取，不用通用数
  engagement_by_joint_type.s288_metal_thread.hard_cap_mm   螺孔盲深上限
  screw_length_rule.formula                          螺丝总长 ≤ 叠厚 + 盲深 − 余量（余量从公式里解析）
  screw_length_rule.per_hole_not_per_group           逐孔不逐组
  feature_check_tolerances.seat_flatness_spread_mm   逐颗螺丝足印环带的叠厚极差**与承压面平面度**
  feature_check_tolerances.horn_clock_deg / horn_concentricity_mm
  fits.<桶>.target_range_mm                          配合数值
  relations.yaml:relations[].target_ref              每条功能关系自己指向的桶

两条历史事故是本层的验收标准：
  N01 = F11b_neckpitch_back_lower —— audit 按组取 max 叠厚 5.2 选了 M2×8，把下排 3.3 掩盖掉，
        M2×8 进舵机 4.7 > 螺孔深 3.0 → 顶底 1.7。本层逐孔算，必须判红。
  N03 = F15_headyaw_back —— audit 用 8 向中位数 3.30，把不平的半圈平均掉了，
        实际 16 向 3.3..6.9，螺丝头坐不平。本层量整条足印环带的极差，必须判红。

2026-09-09 复审（docs/gate/体系审查_Codex01.md）修掉的三条假绿：
  FG13 只看 status / actual_src 标签和 nominal vs target 的纸面比较 → 现在实测
       **接触面积 / 有符号配合量 / 夹紧行程**（relations[].contact），标签不参与判定。
  FG14 criterion 写着"含最深/最浅两端"，代码却只判中位数和最深端，最浅端（拧不住那一端）
       算出来只进 detail；孔的归属还按"件名 + 数量"猜 → 现在按 feature_hole_map 逐孔关联，
       最浅/中位/最深三个量各自都判。
  FG15 坐面只量厚度极差，等厚斜面恒过 → 现在同时判头侧承压面平面度。
2026-09-13 审计 F-L5-2 / F-L5-3：
  坐面改按 fasteners.yaml 的穿件孔逐组判（seat_flatness = 头侧承压面平面度 + 每向有料，seat_stack_spread = 叠厚极差），
       头侧只从 tool_access[].seats[].outward_export_local 取，取不到 → unknown，不取较差端；
  配合桶 fit_measured_geometry 只在 relation_contact PASS 时才算"有实测撑着"，量了但没过 → FAIL。
2026-09-13 第二批（F-L5-2 起点 / F-L5-4 阈值来源 / F-L5-5 盲深）：
  坐面判据只覆盖 head_locator_map_indices 指向的穿件孔（沉孔/铣面条目是同一颗螺丝，不是另一颗）；头侧按
       tool_access[].seats[] 逐坐面取，键带 instance（左右实例同 outward 不是冲突；同一 instance 反向才是）；
       feature_hole_map 的孔点是**刀心**，只当轴线锚（坐面点横向偏离孔轴 > 1e-6 → unknown，与 tool_access.py 同规则）；
  阈值（fits.*.target_range_* / feature_check_tolerances.*）读取时把 src 写进 criterion：没有 src → 该判据 unknown
       （"阈值没有来源"）；src=assumed → 照判 + 另发 threshold_assumed:<键> FAIL(WARN)，measured=阈值本身；
  自攻盲深先读 fasteners.yaml:<组>.pilot.pilot_depth_mm，再退回 joins/location 正则；两者都有且不一致 → unknown。
2026-09-25（gate_guard 第 6 项，校验会话 18:55 事故）：
  joins 必须是字符串列表。含半角逗号的流式列表项没加引号 → YAML 拆成几项、数字段变 int/float → 以前 _blind_depth 的
  " ".join 抛 TypeError、整个本模块没交回任何判据（hr43d L5 少 49 格）。现在只给该组发一条 joins_format FAIL(BLOCK)
  （measured 如 "joins[1]=int 23"），该组盲深不从 joins/location 文字取（写了 pilot.pilot_depth_mm 仍照判），其他组照常评；
  数据正常时不发这条（记分卡逐格不变）。
"""
from __future__ import annotations

import re
import sys
from pathlib import Path

import numpy as np

_HERE = Path(__file__).resolve()
sys.path.insert(0, str(_HERE.parents[1]))
sys.path.insert(0, str(_HERE.parent))
from core import LayerResult, PASS, FAIL, BLOCK, WARN, INFO, num, ROOT   # noqa: E402

try:                                    # gate.py 以 layers.l5_screwhead 导入
    from layers.l5_screwhead import (declared_head_sides, seat_table, hole_geom,     # noqa: E402,F401
                                     threshold_meta, src_label, AssumedThresholds, LATERAL_TOL)
except ImportError:
    from l5_screwhead import (declared_head_sides, seat_table, hole_geom,            # noqa: E402,F401
                              threshold_meta, src_label, AssumedThresholds, LATERAL_TOL)
try:                                    # gate.py 以 layers.l2_features 导入，复用它的几何缓存
    from layers.l2_features import (geom, seat_profile, walk_fits,               # noqa: E402
                                    tol_pair, ray_hits, solid_spans_dedup, gv,
                                    _RAY_DIRS, _SEAT_RINGS)
except ImportError:                     # 单独 import 本模块时的退路
    from l2_features import (geom, seat_profile, walk_fits,                      # noqa: E402
                             tol_pair, ray_hits, solid_spans_dedup, gv,
                             _RAY_DIRS, _SEAT_RINGS)

LAYER = 5
NAME = "功能达成"

# 解析用正则（不是判据，是把清单里的文字变成数）
_RX_SCREW_LEN = re.compile(r"M2\s*[×x]\s*(\d+(?:\.\d+)?)")
_RX_PILOT_DEPTH = re.compile(r"Ø\s*\d+(?:\.\d+)?\s*(?:底孔|深)\s*(\d+(?:\.\d+)?)")
_RX_MARGIN = re.compile(r"[−–—-]\s*(\d+(?:\.\d+)?)\s*$")
_RX_R = re.compile(r"\br\s*(\d+(?:\.\d+)?)")


def _dig(root, dotted):
    """按点号路径取值，取不到给 None。"""
    cur = root
    for k in dotted.split("."):
        if not isinstance(cur, dict) or k not in cur:
            return None
        cur = cur[k]
    return cur


def _resolve_ref(data, ref):
    """'tolerances.yaml:fits.pla.key_slot' → (文件, 路径, 节点)。节点为 None 表示悬空引用。"""
    if not isinstance(ref, str) or ":" not in ref:
        return None, None, None
    fn, path = ref.split(":", 1)
    stem = fn[:-5] if fn.endswith(".yaml") else fn
    return stem, path, _dig(data.get(stem) or {}, path)


# ── 咬入 / 螺丝长度 ────────────────────────────────────────────────────────
def _engage_rule(T, jt):
    node = ((T.get("engagement_by_joint_type") or {}).get(jt)) or {}
    rng = node.get("range_mm") or {}
    cap, cap_src = num(node.get("hard_cap_mm"))
    return node, rng.get("min"), rng.get("max"), cap, cap_src


# 导出 STL 的顶点是 float32，射线量出来的长度带 ~1e-6 mm 的**表示噪声**。
# 这不是判据放宽（判据一个都没动），是不让绿红由 float32 的最后一位决定：
# 叠厚量到 2.9999998 时"咬入 3.0000002 > 上限 3.0"这种红没有任何物理含义。
_NUM_EPS = 1e-6


def _in_range(v, lo, hi):
    if v is None:
        return None
    if lo is not None and v < lo - _NUM_EPS:
        return False
    if hi is not None and v > hi + _NUM_EPS:
        return False
    return True


def _screw_length(spec):
    """从 spec 取名义长度。F11b 的 spec 是"打印清单写 M2×8（应为 M2×6）"——
    取**第一个**，也就是清单上真会被采购/拧上去的那个长度。"""
    m = _RX_SCREW_LEN.search(spec or "")
    return float(m.group(1)) if m else None


def check_self_tap_radial(res, fa):
    """自攻的必要条件：主径必须大于底孔径。不能拿轴向伸入代替径向成牙。"""
    family = (fa.get("screw_type") or {}).get("family")
    family = family.get("v") if isinstance(family, dict) else family
    if "self_tap" not in str(fa.get("joint_type", "")) and family != "self_tapping":
        return
    gid = fa["id"]
    m = re.search(r"M(\d+(?:\.\d+)?)(?=[×x*\s]|$)", str(fa.get("spec", "")))
    pd, src = num((fa.get("pilot") or {}).get("pilot_hole_d_mm"))
    if m is None or pd is None or not np.isfinite(pd) or pd <= 0:
        res.unknown(gid, "self_tap_radial_overlap", "缺主径或本组底孔径，无法判断自攻径向成牙的必要条件",
                    provenance=f"fasteners.yaml:{gid}.spec/pilot.pilot_hole_d_mm")
        return
    major = float(m.group(1))
    overlap = (major - pd) / 2
    res.add(subject=gid, check="self_tap_radial_overlap", state=PASS if overlap > 0 else FAIL,
            severity=BLOCK, measured=overlap, evidence_n=2,
            criterion="自攻径向重叠 (螺丝主径−底孔径)/2 > 0 mm；只判必要条件，不证明咬入强度或打印后尺寸",
            detail=f"主径 {major} mm，底孔 {pd} mm（{src}）；等径/大孔不能成牙。仍须逐孔几何、轴向咬入及试件验证。",
            provenance=f"fasteners.yaml:{gid}.spec/pilot.pilot_hole_d_mm")


def _joins_format_problem(fa):
    """fasteners.yaml:<组>.joins 必须是**字符串列表**（每项一句关系文字）。返回 None = 合格（没写 joins 不算格式错：
    文字源为空，盲深照旧只看 pilot / location）；否则返回 measured 文本，如 "joins[1]=int 23"（多项用 "; " 连）。"""
    j = fa.get("joins")
    if j is None:
        return None
    if not isinstance(j, list):
        return f"joins={type(j).__name__} {str(j)[:40]!r}"
    bad = [f"joins[{i}]={type(x).__name__} {x!r}"[:80] for i, x in enumerate(j) if not isinstance(x, str)]
    return "; ".join(bad) if bad else None


def _report_joins_format(res, fa, problem):
    """joins 格式错 → 该组一条 joins_format FAIL(BLOCK)。只在出错时发（正常数据不多出任何判据）。"""
    gid = fa.get("id", "?")
    j = fa.get("joins")
    glued = ", ".join(str(x) for x in j) if isinstance(j, list) else str(j)
    res.add(subject=gid, check="joins_format", state=FAIL, severity=BLOCK, measured=problem,
            criterion="fasteners.yaml:<组>.joins 必须是字符串列表（每项一句关系文字）；YAML 流式列表里含半角逗号的项"
                      "必须整项加引号，否则会被拆成几项、数字段变成 int/float",
            evidence_n=len(j) if isinstance(j, list) else 1,
            detail=(f"fasteners.yaml:{gid}.joins 有非字符串项：{problem} —— 多半是流式列表 [...] 里某项含半角逗号没加引号、"
                    f"被 YAML 拆开了，把那一项整项加引号（'…'）；按逗号拼回是 {glued[:120]!r}。本组盲深不从 joins/location "
                    f"文字取（写了 pilot.pilot_depth_mm 仍照判，没写 → screw_length_rule unknown），其他组照常评"),
            provenance=f"fasteners.yaml:{gid}.joins")


def _blind_depth(fa, cap, joins_ok=True):
    """盲深 → (值, 来源说明)。金属螺纹/舵机壳用桶的 hard_cap_mm；自攻（F-L5-5，2026-09-13）：
    先读 `fasteners.yaml:<组>.pilot.pilot_depth_mm`（{v,src} 结构），取不到再退回 joins/location 文字里的
    "Ø1.6 底孔 4.5 / Ø1.7 深 10"；两者都有且不一致 → (None, 写明两个数)，不许挑一个用。
    joins_ok=False（joins 格式错，已发 joins_format，2026-09-25）：文字源不可信 → 只认 pilot.pilot_depth_mm，没有就拿不到数。"""
    if cap is not None:
        return float(cap), "engagement_by_joint_type.<jt>.hard_cap_mm"
    gid = fa.get("id", "?")
    pd, pd_src = num((fa.get("pilot") or {}).get("pilot_depth_mm"))
    if pd is not None and not (np.isfinite(pd) and pd > 0):
        pd = None
    if not joins_ok:
        if pd is not None:
            return float(pd), (f"fasteners.yaml:{gid}.pilot.pilot_depth_mm（src={pd_src}；joins 格式错，"
                               f"没与 joins/location 文字核对）")
        return None, (f"fasteners.yaml:{gid}.joins 格式错（见 joins_format），不从 joins/location 文字取底孔深，"
                      f"pilot.pilot_depth_mm 也没给 —— 盲深拿不到数")
    txt = " ".join([*(fa.get("joins") or []), str(fa.get("location") or "")])
    vals = [float(x) for x in _RX_PILOT_DEPTH.findall(txt)]
    txt_v = vals[0] if len(set(vals)) == 1 else None
    if pd is not None and txt_v is not None and abs(float(pd) - txt_v) > _NUM_EPS:
        return None, (f"不一致：fasteners.yaml:{gid}.pilot.pilot_depth_mm = {pd}（src={pd_src}） vs "
                      f"joins/location 文字里的底孔深 {txt_v} —— 两个数对不上，不挑一个用")
    if pd is not None:
        return float(pd), f"fasteners.yaml:{gid}.pilot.pilot_depth_mm（src={pd_src}）"
    if txt_v is not None:
        return txt_v, "fasteners.yaml:joins（底孔深度写在关系文字里；pilot.pilot_depth_mm 未给）"
    return None, None


# ── 坐面 / 逐孔叠厚 ────────────────────────────────────────────────────────
# 2026-09-13（F-L5-2）：删掉了 _screw_holes / _measure_seats —— 它们把件上所有 Ø(m_through±0.1) 的孔都当螺丝孔
# 扫一遍、且坐面取"较差的一端"。现在坐面按 fasteners.yaml 的穿件孔逐组判，头侧按 tool_access 声明。


# ── 孔级关联（FG14：以前按"件名 + 数量"猜，现在按 feature_hole_map 逐孔对）──
_MATCH_R_MM = 0.6      # 声明孔心到实测孔轴的**径向**关联半径（这是关联容差，不是判据）


def _match_hole(g, pt, d_nom, dtol, axis=None):
    """声明的孔坐标（export_local）→ 导出件上真实存在的那个孔（圆柱面）。

    只按**径向**关联：孔轴必须从声明的点旁边穿过去。轴向不设限 —— features.yaml 的
    hole_positions_mm 记的是**刀具中点**，刀比料长得多（Ø2.2 的刀走 42 mm 穿 5 mm 的法兰），
    点常常落在料外面。同一条轴上有多个孔时取轴向最近的那个，轴向偏移写进 detail。
    对不上就返回 None，调用方判 unknown —— 元规则 4：关联不上不是"跳过"，是失败。
    """
    p = np.asarray(pt, float)
    best = None
    for c in g.holes:
        if abs(2 * c["r"] - d_nom) > dtol:
            continue
        ax = np.asarray(c["axis"], float)
        if axis is not None and abs(float(np.dot(ax, np.asarray(axis, float)))) < 0.9:
            continue
        rel = p - np.asarray(c["mid"], float)
        t = float(rel @ ax)
        d = float(np.linalg.norm(rel - t * ax))
        if d > _MATCH_R_MM:
            continue
        gap = max(0.0, abs(t) - (c["t1"] - c["t0"]) / 2.0)   # 点到孔身区间的轴向距离
        key = (round(d, 3), gap)
        if best is None or key < (round(best[0], 3), best[2]):
            best = (d, c, gap)
    return best


def _group_holes(fa, feats):
    """一组螺丝的 feature_hole_map → [(feature_id, 件, 特征, 孔坐标)]，按**接收/穿过**分类。

    kind=pilot_hole 的是接收孔（螺纹咬在里面），其余（screw_hole/counterbore/tool_channel/
    horn_hole）是螺丝穿过去的孔 —— 叠厚要在穿过去那一侧量。
    """
    thru, recv, miss = [], [], []
    for mi, e in enumerate(fa.get("feature_hole_map") or []):
        fid = e.get("feature_id")
        fe = feats.get(fid)
        if fe is None:
            miss.append(f"{fid}（features.yaml 里没有这条）")
            continue
        kind = fe.get("check_class") or fe.get("kind")
        bucket = recv if kind == "pilot_hole" else thru
        fe_i = {**fe, "geom": hole_geom(fe, e)}      # geom.instances[feature_hole_map[].instance] 覆盖顶层（axis/pos/nominal_d）
        for hi, h in enumerate(e.get("holes") or []):
            if isinstance(h, (list, tuple)) and len(h) == 3:
                bucket.append((fid, fe.get("part"), fe_i, [float(x) for x in h], (mi, hi)))
            else:
                miss.append(f"{fid} 的孔坐标不是三元组：{str(h)[:30]}")
    return thru, recv, miss


# ── 成对配合的实测几何（FG13：状态标签不能替代几何）─────────────────────────
_CONTACT_STEP = 0.5       # 接触面网格步长 mm（采样密度，不是判据）
_CONTACT_MAX = 40         # 每个方向最多多少格
_AXIS_VEC = {"x": (1.0, 0, 0), "y": (0, 1.0, 0), "z": (0, 0, 1.0)}


def _axis_vec(a):
    if not isinstance(a, str):
        return None, 1.0
    t = a.strip().lstrip("+")
    sgn = -1.0 if t.startswith("-") else 1.0
    v = _AXIS_VEC.get(t.lstrip("-"))
    return (np.array(v, float) if v else None), sgn


def _resolve_party(ctx, p):
    """关系里的一方 → 世界坐标（placed/）的导出件。给不出就说清为什么。"""
    if not isinstance(p, dict):
        return None, f"parties 里这一项不是 dict：{str(p)[:40]}"
    if p.get("placed"):
        q = ctx.placed(p["placed"])
        return (q, "") if q else (None, f"placed/{p['placed']}.stl 不存在")
    if p.get("part"):
        q = ctx.placed_for(p["part"])
        return (q, "") if q else (None, f"件 {p['part']} 在 placed/ 里找不到落位件")
    return None, f"这一方既没有 part 也没有 placed：{sorted(p)}"


def contact_profile(ga, gb, ax, sgn, lo, hi):
    """动件 A 沿 sgn·ax 压向定件 B：逐格量**有符号配合量**、**接触面积**、**夹紧行程**。

    每格沿 ax 打一条射线，取 A 朝 B 那一面与 B 朝 A 那一面在轴上的坐标：
        signed = (A 朝向面) − (B 朝向面)，沿压紧方向为正
        > 0 过盈（真的压上了，有夹紧力）  = 0 刚好贴  < 0 间隙（什么都没夹住）
    夹紧行程 = 动件还要走多远才碰到定件 = max(0, −max(signed))。
    这就是 H02 那次事故的正面判据：四周 0.30 均匀间隙时干涉 = 0，而这里是 −0.30、接触面积 0。
    """
    u = np.asarray(ax, float) * float(sgn)
    idx = [i for i in range(3) if abs(u[i]) < 0.9]
    d = np.asarray(hi, float) - np.asarray(lo, float)
    n = [max(2, min(_CONTACT_MAX, int(np.ceil(d[i] / _CONTACT_STEP)) + 1)) for i in idx]
    us = np.linspace(lo[idx[0]], hi[idx[0]], n[0])
    vs = np.linspace(lo[idx[1]], hi[idx[1]], n[1])
    cell = ((hi[idx[0]] - lo[idx[0]]) / max(1, n[0] - 1)) * \
           ((hi[idx[1]] - lo[idx[1]]) / max(1, n[1] - 1))
    ai = int(np.argmax(np.abs(u)))
    w0, w1 = float(min(lo[ai], hi[ai])), float(max(lo[ai], hi[ai]))
    back = 200.0
    signed, n_rays, n_cell, n_pair = [], 0, 0, 0
    for a_ in us:
        for b_ in vs:
            p = np.zeros(3)
            p[idx[0]], p[idx[1]], p[ai] = a_, b_, (w0 + w1) / 2
            o = p - u * back
            n_cell += 1
            # 射线参数 t ↔ 轴坐标：点 = o + u·t → 轴坐标 = p[ai] + u[ai]·(t − back)
            pai, uai = float(p[ai]), float(u[ai])
            segs2 = []
            for gx in (ga, gb):
                t, nd = ray_hits(gx.V, gx.F, o, u)
                n_rays += 1
                _tot, segs = solid_spans_dedup(t, nd)
                cc = [tuple(sorted((pai + uai * (x - back), pai + uai * (y - back))))
                      for x, y in segs]
                segs2.append([s for s in cc if s[1] > w0 - 1e-9 and s[0] < w1 + 1e-9])
            aa, bb = segs2
            if not aa or not bb:
                continue
            # 沿压紧方向 fwd：A 的"最前面"、B 的"最靠后"
            fwd = 1.0 if uai >= 0 else -1.0
            a_face = max(s[1] if fwd > 0 else s[0] for s in aa) if fwd > 0 else \
                min(s[0] for s in aa)
            b_face = min(s[0] for s in bb) if fwd > 0 else max(s[1] for s in bb)
            n_pair += 1
            signed.append((a_face - b_face) * fwd)
    if not signed:
        return None
    arr = np.array(signed, float)
    return dict(n_cell=n_cell, n_pair=n_pair, n_rays=n_rays, cell_mm2=float(cell),
                signed_min=float(arr.min()), signed_max=float(arr.max()),
                signed_med=float(np.median(arr)),
                # 同层 _in_range 已用 _NUM_EPS 吸收 float32 表示噪声，这里也必须用：
                # 否则两个面本来共面（signed = -9.5e-7，即 1 ulp）会被算成"不接触"→ 面积 0
                # → kind=clamp 的"必须有 >0 接触面积"假红。R17 就栽在这。
                contact_cells=int((arr >= -_NUM_EPS).sum()),
                contact_area_mm2=float((arr >= -_NUM_EPS).sum() * cell),
                clamp_travel_mm=float(max(0.0, -arr.max())),
                projected_area_mm2=float(n_pair * cell))


# ── 层主体 ─────────────────────────────────────────────────────────────────
def run(ctx) -> LayerResult:                                              # noqa: C901
    res = LayerResult(LAYER, NAME)
    D = ctx.data
    T = D.get("tolerances") or {}
    fct = T.get("feature_check_tolerances") or {}
    buckets = walk_fits(T.get("fits") or {})
    slr = T.get("screw_length_rule") or {}

    _seat_lo, seat_hi, seat_src = threshold_meta(fct.get("seat_flatness_spread_mm"))
    _ck_lo, clk_hi, clk_src = threshold_meta(fct.get("horn_clock_deg"))
    _cc_lo, con_hi, con_src = threshold_meta(fct.get("horn_concentricity_mm"))
    SEAT_KEY, CLK_KEY, CON_KEY = ("feature_check_tolerances.seat_flatness_spread_mm",
                                  "feature_check_tolerances.horn_clock_deg",
                                  "feature_check_tolerances.horn_concentricity_mm")
    assumed = AssumedThresholds()          # F-L5-4：被引用过的 src=assumed 阈值，run 末尾发 threshold_assumed WARN
    d_lo, d_hi = tol_pair(fct.get("hole_diameter_mm"), key="_")
    dtol = max(abs(float(d_lo or 0.1)), abs(float(d_hi or 0.1)))

    m_through, _s1 = num((buckets.get("fits.pla.m2_through_hole") or {}).get("nominal_mm"))
    m_cbore, _s2 = num((buckets.get("fits.pla.m2_counterbore") or {}).get("nominal_mm"))
    mm = _RX_MARGIN.search(str(slr.get("formula") or ""))
    margin = float(mm.group(1)) if mm else None

    n_rays = 0

    # ══ 1. 螺丝：逐组取自己的 joint_type 桶，逐孔算，不按组取 max ═══════════
    fasteners = (D.get("fasteners") or {}).get("fasteners") or []
    if not fasteners:
        res.unknown("_fasteners", "list", "fasteners.yaml 里没有 fasteners 列表")
    if margin is None:
        res.unknown("_fasteners", "screw_length_rule",
                    f"tolerances.yaml:screw_length_rule.formula 里解析不出安全余量"
                    f"（'{slr.get('formula')}'），螺丝总长规则无判据",
                    provenance="tolerances.yaml:screw_length_rule.formula")
    res.add(subject="_fasteners", check="per_hole_not_per_group",
            state=PASS if slr.get("per_hole_not_per_group") else FAIL, severity=BLOCK,
            measured=bool(slr.get("per_hole_not_per_group")),
            criterion="tolerances.yaml:screw_length_rule.per_hole_not_per_group 必须为真，"
                      "本层按每组自己的叠厚判，能拿到几何时再按每个孔的 16 向叠厚判",
            evidence_n=len(fasteners),
            detail=str(slr.get("why") or "")[:110],
            provenance="tolerances.yaml:screw_length_rule")

    for fa in fasteners:
        gid = fa["id"]
        # 2026-09-25：joins 被 YAML 拆碎（非字符串项）→ 只红这一组（joins_format BLOCK）、该组盲深不从文字取；正常数据不发这条
        joins_bad = _joins_format_problem(fa)
        if joins_bad:
            _report_joins_format(res, fa, joins_bad)
        check_self_tap_radial(res, fa)
        jt = fa.get("joint_type")
        node, emin, emax, cap, cap_src = _engage_rule(T, jt)
        prov = f"fasteners.yaml:{gid} | tolerances.yaml:engagement_by_joint_type.{jt}"
        if not node:
            res.unknown(gid, "joint_type",
                        f"joint_type='{jt}' 在 tolerances.yaml:engagement_by_joint_type 里没有对应桶",
                        provenance=prov)
            continue

        L = _screw_length(str(fa.get("spec") or ""))
        stack, stack_src = num(fa.get("stack_mm"))
        eng, eng_src = num(fa.get("engagement_mm"))
        qty = fa.get("qty") or 0
        bd, bd_src = _blind_depth(fa, cap, joins_ok=not joins_bad)

        # 1a) 连接类型本身没有咬入概念的两类，按 rule 判它自己的要求
        if emin is None and emax is None:
            if jt == "none_zip_tie":
                res.add(subject=gid, check="engagement", state=PASS, severity=INFO, measured="n/a",
                        criterion=f"{jt}: {node.get('rule')}（range_mm 两端都是 null = 本来就没有这个概念）",
                        evidence_n=1, detail=f"{qty} 根扎带", provenance=prov)
            elif jt == "none_no_receiver":
                ok = (fa.get("status") == "disabled") and qty == 0
                res.add(subject=gid, check="engagement", state=PASS if ok else FAIL, severity=BLOCK,
                        measured=f"status={fa.get('status')} qty={qty}",
                        criterion=f"{jt}: {node.get('rule')}",
                        evidence_n=2, provenance=prov)
            else:
                res.unknown(gid, "engagement",
                            f"{jt} 的 range_mm 两端都是 null（{node.get('unknown_reason') or '清单没给'}），"
                            f"该组 {qty} 颗没有咬入判据", provenance=prov)
        else:
            # 1b) 清单声明的咬入
            # 未知咬合必须保留未知；配合桶的名义凸出量不是本组的咬合证据。
            ok = _in_range(eng, emin, emax)
            if ok is None:
                res.unknown(gid, "engagement_declared",
                            f"fasteners.yaml:{gid}.engagement_mm.v 是 null"
                            f"（{(fa.get('engagement_mm') or {}).get('unknown_reason') or '清单没给'}）"
                            f"——{qty} 颗螺丝的咬入无从判", provenance=prov)
            else:
                res.add(subject=gid, check="engagement_declared", state=PASS if ok else FAIL,
                        severity=BLOCK, measured=eng,
                        criterion=f"{jt} 咬入 ∈ [{emin}, {emax}]（{node.get('rule')}）"[:150],
                        evidence_n=qty, detail=f"src={eng_src}", provenance=prov)

            # 1c) 由「长度 − 叠厚」推出的咬入（清单值和几何值互相咬合的第二道）
            if L is None or stack is None:
                res.unknown(gid, "engagement_derived",
                            f"算不出咬入：螺丝长度={L}（spec='{str(fa.get('spec'))[:26]}'）、"
                            f"叠厚={stack}（stack_mm.src={stack_src}）", provenance=prov)
            else:
                e2 = L - float(stack)
                ok2 = _in_range(e2, emin, emax)
                res.add(subject=gid, check="engagement_derived", state=PASS if ok2 else FAIL,
                        severity=BLOCK, measured=round(e2, 4),
                        criterion=f"长度 {L} − 叠厚 {stack} 的咬入 ∈ [{emin}, {emax}]（{jt}）",
                        evidence_n=qty,
                        detail=(str(fa.get("engagement_alternatives") or "")
                                or f"清单声明 {eng}"), provenance=prov)

        # 1d) 螺丝总长规则 —— 逐孔（本组有几种叠厚就算几次，不取 max）
        is_screw = "M2" in str(fa.get("spec") or "")
        if not is_screw:
            res.add(subject=gid, check="screw_length_rule", state=PASS, severity=INFO,
                    measured="n/a", criterion=f"本组不是螺丝（spec='{str(fa.get('spec'))[:22]}'，"
                                              f"joint_type={jt}），螺丝总长规则不适用",
                    evidence_n=qty, provenance=prov)
        elif margin is not None:
            if L is None or stack is None or bd is None:
                why_bd = ("金属螺纹/舵机壳用 hard_cap_mm" if cap is not None else
                          (bd_src if bd_src else "自攻要先读 pilot.pilot_depth_mm、再从 joins 里的底孔深度取，都没取到"))
                res.unknown(gid, "screw_length_rule",
                            f"{slr.get('formula')}：长度={L} 叠厚={stack} 盲深={bd}（{why_bd}）"
                            f" —— 三个数缺一就判不了",
                            provenance="tolerances.yaml:screw_length_rule | " + prov)
            else:
                lim = float(stack) + float(bd) - margin
                res.add(subject=gid, check="screw_length_rule", state=PASS if L <= lim else FAIL,
                        severity=BLOCK, measured=L,
                        criterion=f"螺丝总长 ≤ 叠厚 {stack} + 盲深 {bd} − {margin} = {round(lim,4)}"
                                  f"（screw_length_rule.formula，逐孔）",
                        evidence_n=qty,
                        detail=f"盲深来源：{bd_src}"
                               + (f"；超出 {round(L - lim, 4)} mm（顶底）" if L > lim else ""),
                        provenance="tolerances.yaml:screw_length_rule | " + prov)

        # 1e) 清单自己声明的状态 / 已知问题
        issue = fa.get("issue") or {}
        st = fa.get("status")
        if issue:
            res.add(subject=gid, check="declared_issue", state=FAIL,
                    severity=issue.get("severity") or BLOCK, measured=issue.get("status"),
                    criterion="fasteners.yaml 里带 issue 的组一律不放行，直到 issue 关掉",
                    evidence_n=qty,
                    detail=f"{str(issue.get('why'))[:70]} | 被掩盖方式：{str(issue.get('masked_by'))[:60]}",
                    provenance=f"fasteners.yaml:{gid}.issue")
        elif st in ("ok", "disabled"):
            res.add(subject=gid, check="declared_status", state=PASS, severity=INFO, measured=st,
                    criterion="fasteners.yaml:status ∈ {ok, disabled}", evidence_n=qty,
                    provenance=f"fasteners.yaml:{gid}.status")
        else:
            res.add(subject=gid, check="declared_status", state=FAIL, severity=WARN, measured=st,
                    criterion="fasteners.yaml:status 必须是 ok/disabled；unverified = 没查过（元规则 4）",
                    evidence_n=qty, detail=str(fa.get("status_verbatim") or "")[:70],
                    provenance=f"fasteners.yaml:{gid}.status")

    known = set((D.get("fasteners") or {}).get("known_issues") or [])
    have = {f["id"] for f in fasteners if f.get("issue")}
    res.add(subject="_fasteners", check="known_issues_covered",
            state=PASS if known and known <= have else FAIL, severity=BLOCK,
            measured=sorted(have), criterion=f"fasteners.yaml:known_issues {sorted(known)} 必须逐条落到带 issue 的组上",
            evidence_n=len(known), provenance="fasteners.yaml:known_issues")

    # ══ 2. 坐面平整度 / 逐孔叠厚（几何，抓 F15 / FG14 / FG15 / F-L5-2）═══════════
    #   2026-09-13（F-L5-2）：坐面**按 fasteners.yaml 的穿件孔逐组判**（不再按直径扫全件），头侧承压面
    #   只看 tool_access[].seats[].outward_export_local 声明的那一端（不取较差端）；取不到 → unknown。
    #   FG14 的两半：① 归属按 feature_hole_map 的孔坐标逐孔关联到导出件上真实存在的那个孔，关联不上就 unknown；
    #   ② 咬入的最浅端 / 中位 / 最深端三个量各自都判。
    feats_by_id = {f["id"]: f for f in ((D.get("features") or {}).get("features") or [])}
    if seat_hi is None:
        res.unknown("_seat", "seat_flatness",
                    "tolerances.yaml:feature_check_tolerances.seat_flatness_spread_mm 取不到，坐面无判据",
                    provenance="tolerances.yaml:feature_check_tolerances")
    elif m_through is None or m_cbore is None:
        res.unknown("_seat", "seat_flatness",
                    "fits.pla.m2_through_hole / m2_counterbore 的 nominal 取不到，"
                    "定不出该在哪个半径上打坐面射线", provenance="tolerances.yaml:fits.pla")
    else:
        # 采样范围 = 从过孔壁到沉孔沿的**整条环带**（= 螺丝头真正压住的那圈料），
        # 不是一条半径上的一圈。这是采样位置不是判据；判据是 seat_flatness_spread_mm。
        r_in, r_out = float(m_through) / 2, float(m_cbore) / 2
        for fa in fasteners:
            gid, jt = fa["id"], fa.get("joint_type")
            node, emin, emax, cap, _cs = _engage_rule(T, jt)
            if emin is None and emax is None:
                continue
            L = _screw_length(str(fa.get("spec") or ""))
            prov = (f"fasteners.yaml:{gid}.feature_hole_map | "
                    f"tolerances.yaml:engagement_by_joint_type.{jt}")
            prov_side = f"fasteners.yaml:{gid}.tool_access[].seats[].outward_export_local"
            thru, recv, miss = _group_holes(fa, feats_by_id)
            # 头侧孔全集 = head_locator_map_indices 指向的 map 项：同一颗螺丝的沉孔/铣面条目不是另一颗螺丝，
            # 叠厚/坐面都在杆孔上量。locators 缺失时退回全部穿件孔（坐面那条会因头侧未声明判 unknown）。
            hmap = fa.get("feature_hole_map") or []
            locs = fa.get("head_locator_map_indices")
            loc_ok = (isinstance(locs, list) and bool(locs) and len(set(locs)) == len(locs)
                      and all(type(i) is int and 0 <= i < len(hmap) for i in locs))
            if loc_ok:
                thru = [t for t in thru if t[4][0] in locs]
            if L is None:
                res.unknown(gid, "engagement_per_hole",
                            f"spec='{str(fa.get('spec'))[:30]}' 里解析不出螺丝长度，"
                            f"逐孔咬入 = 长度 − 该孔叠厚，算不了", provenance=prov)
                continue
            if not thru:
                res.unknown(gid, "engagement_per_hole",
                            f"feature_hole_map 里没有**穿过去**那一侧的孔坐标"
                            f"（接收孔 {len(recv)} 个、坏条目 {miss[:2]}）——"
                            f"叠厚要在螺丝穿过的件上量，没有孔就归不了位",
                            provenance=prov)
                continue
            table = seat_table(fa)
            group_inst = table["instances"]
            has_ta = bool(fa.get("tool_access"))
            rows, unmatched, undeclared = [], [], []
            skipped_only = 0
            for fid, ppid, fe, pt, (mi, hi) in thru:
                if ctx.only and ppid not in ctx.only:
                    skipped_only += 1                 # 子集跑（--parts）范围外的件：不判也不报
                    continue
                stl = ctx.stl(ppid) if ppid else None
                if stl is None:
                    unmatched.append(f"{fid}@{ppid}: 取不到导出 STL")
                    continue
                try:
                    gg = geom(stl)
                except Exception as e:
                    unmatched.append(f"{fid}@{ppid}: STL 读不了（{e}）")
                    continue
                rp = str(stl.relative_to(ROOT))
                if rp not in res.inputs:
                    res.inputs.append(rp)
                dn = gv((fe.get("geom") or {}).get("nominal_d_mm"))
                if dn is None:
                    unmatched.append(f"{fid}: geom 没有 nominal_d_mm")
                    continue
                fax = _axis_vec((fe.get("geom") or {}).get("axis"))[0]
                hit = _match_hole(gg, pt, float(dn), dtol, axis=fax)
                if hit is None:
                    unmatched.append(f"{fid}@{ppid} {[round(x,2) for x in pt]}: "
                                     f"导出件上离该轴 {_MATCH_R_MM}mm 内没有 Ø{dn} 的孔")
                    continue
                dist, c, gap = hit
                s = seat_profile(gg.V, gg.F, c, r_in, r_out, gg.reach)
                n_rays += s["n_rays"]
                # 头侧承压面平面度：只取 tool_access 坐面声明的那一端。seat_profile 的 flatness_ends[0] 是射线入口面
                # （孔轴 −c.axis 那一侧）、[1] 是出口面（+c.axis 侧）。键带 instance：左右实例（同一份 STL）同 outward
                # 只算一次；同一 instance 反向 = 冲突；缺某个 instance 的坐面 = 那颗螺丝未声明。
                st = table["holes"].get((fid, mi, hi))
                flat_head = None
                tag = f"{fid}#{mi}.{hi}"
                if not has_ta or st is None:
                    undeclared.append(tag + (f"@{sorted(str(x) for x in group_inst)}" if group_inst else ""))
                elif group_inst - st["instances"]:
                    undeclared.append(f"{tag}@缺 {sorted(str(x) for x in (group_inst - st['instances']))}")
                elif st["conflicts"]:
                    undeclared.append(f"{tag}(同一 instance {st['conflicts']} 头侧方向冲突)")
                else:
                    ends, bad_row = set(), None
                    for row in st["rows"]:
                        side, spt = row["outward"], row["point"]
                        delta = spt - np.asarray(pt, float)
                        lat = float(np.linalg.norm(delta - side * float(delta @ side)))
                        if lat > LATERAL_TOL:
                            bad_row = f"{tag}(坐面点横向偏离孔轴 {lat:.2e} > {LATERAL_TOL}，孔/坐面数据错位)"
                            break
                        dot = float(side @ np.asarray(c["axis"], float))
                        if abs(dot) < 0.9:
                            bad_row = f"{tag}(声明头侧与孔轴不共线)"
                            break
                        ends.add(1 if dot > 0 else 0)
                    if bad_row:
                        undeclared.append(bad_row)
                    elif len(ends) != 1:
                        undeclared.append(f"{tag}(不同 instance 的坐面头侧相反)")
                    else:
                        flat_head = float(s["flatness_ends"][ends.pop()])
                rows.append((fid, ppid, pt, dist, s, gap, flat_head))
            if not rows:
                if skipped_only == len(thru):
                    continue                          # 全在 --parts 范围外，属于子集跑未覆盖，不是"关联不上"
                res.unknown(gid, "engagement_per_hole",
                            f"{len(thru)} 个声明孔一个都没关联到导出件上的实际孔："
                            f"{unmatched[:3]}", provenance=prov)
                continue

            # ── 2a) 坐面：头侧承压面平面度（声明端）+ 每向有料；叠厚极差另判 seat_stack_spread ──
            judged = [r for r in rows if r[6] is not None]
            if undeclared or not judged:
                res.unknown(gid, "seat_flatness",
                            f"{len(undeclared)}/{len(rows)} 个头侧孔取不到坐面/头侧（{prov_side} 里没有 "
                            f"feature_id+instance+map_index+hole_index 对得上的坐面，或冲突/错位）—— 不取较差端，坐面不判：{undeclared[:3]}",
                            provenance=prov_side)
            elif seat_src is None:
                res.unknown(gid, "seat_flatness",
                            f"阈值没有来源：tolerances.yaml:{SEAT_KEY} 没有 src（{seat_hi}）—— F-L5-4 不判",
                            provenance=f"tolerances.yaml:{SEAT_KEY}")
            else:
                if seat_src == "assumed":
                    assumed.note(SEAT_KEY, seat_hi, f"{gid}:seat_flatness")
                bad = [r for r in judged if r[6] > float(seat_hi) + _NUM_EPS or r[4]["void"] > 0]
                w = max(judged, key=lambda r: r[6])
                n_void = sum(1 for r in judged if r[4]["void"] > 0)
                res.add(subject=gid, check="seat_flatness", state=PASS if not bad else FAIL,
                        severity=BLOCK, measured=round(w[6], 4),
                        criterion=f"本组每个头侧孔（head_locator_map_indices）的**头侧承压面平面度** ≤ {seat_hi}"
                                  f"（{SEAT_KEY}，{src_label(seat_src)}；足印环带 Ø{m_through}→Ø{m_cbore}，"
                                  f"{_SEAT_RINGS} 圈 × {_RAY_DIRS} 向），且每一向都有料；头侧只按 {prov_side} "
                                  f"（键含 instance）声明的方向取，不取较差端（取不到 → unknown）；孔由 feature_hole_map 的刀心逐个关联到孔轴，"
                                  f"坐面点必须在该孔轴上",
                        evidence_n=sum(r[4]["n"] for r in judged),
                        detail=f"{len(judged)} 个穿件孔，{len(bad)} 个不合格（其中 {n_void} 个有射线打在空处 = "
                               f"螺丝头足印悬在料外）；最差 {w[0]}@{w[1]} {[round(x,2) for x in w[2]]} 头侧平面度 "
                               f"{w[6]:.4f}（两端 {w[4]['flatness_ends']}，只判声明端）｜等厚斜面的叠厚极差是 0，"
                               f"只有平面度看得见",
                        provenance="tolerances.yaml:feature_check_tolerances.seat_flatness_spread_mm | " + prov_side)
                for i, r in enumerate(sorted(bad, key=lambda r: -r[6])[:6]):
                    sp = r[4]
                    res.add(subject=gid, check=f"seat_flatness:hole{i}", state=FAIL, severity=BLOCK,
                            measured=round(r[6], 4),
                            criterion=f"该孔自己的足印环带：头侧承压面平面度 ≤ {seat_hi}（{src_label(seat_src)}），且每一向都要有料",
                            evidence_n=sp["n"],
                            detail=f"{r[0]}@{r[1]} {[round(x,2) for x in r[2]]}：头侧平面度 {r[6]:.3f}"
                                   f"（两端 {sp['flatness_ends']}）；{sp['void']}/{sp['n']} 向打在空处，有效承压面积 "
                                   f"{sp['solid_frac'] * sp['ring_area_mm2']:.2f}/{sp['ring_area_mm2']:.2f} mm²",
                            provenance="tolerances.yaml:feature_check_tolerances.seat_flatness_spread_mm")
            if seat_src is None:
                res.unknown(gid, "seat_stack_spread",
                            f"阈值没有来源：tolerances.yaml:{SEAT_KEY} 没有 src（{seat_hi}）—— F-L5-4 不判",
                            provenance=f"tolerances.yaml:{SEAT_KEY}")
            else:
                if seat_src == "assumed":
                    assumed.note(SEAT_KEY, seat_hi, f"{gid}:seat_stack_spread")
                bad_sp = [r for r in rows if r[4]["spread"] > float(seat_hi) + _NUM_EPS]
                ws = max(rows, key=lambda r: r[4]["spread"])
                res.add(subject=gid, check="seat_stack_spread", state=PASS if not bad_sp else FAIL,
                        severity=BLOCK, measured=round(ws[4]["spread"], 4),
                        criterion=f"本组每个穿件孔足印环带的**叠厚极差** ≤ {seat_hi}"
                                  f"（{SEAT_KEY}，{src_label(seat_src)}，applies_to=同一颗螺丝坐面各向叠厚极差，抓 F15 3.3..6.9）；"
                                  f"与头侧无关，两端都算",
                        evidence_n=sum(r[4]["n"] for r in rows),
                        detail=f"{len(rows)} 个穿件孔，{len(bad_sp)} 个超差；最差 {ws[0]}@{ws[1]} 叠厚 "
                               f"{ws[4]['min']:.3f}..{ws[4]['max']:.3f}（中位数 {ws[4]['med']:.3f}）",
                        provenance=f"tolerances.yaml:{SEAT_KEY}")

            # ── 2b) 逐孔咬入：最浅 / 中位 / 最深三个量各自判（FG14）──
            shallow = [L - r[4]["max"] for r in rows]   # 料最厚那一向 = 咬入最浅（拧不住）
            med = [L - r[4]["med"] for r in rows]
            deep = [L - r[4]["min"] for r in rows]      # 料最薄那一向 = 咬入最深（顶底）
            b_sh = [i for i, e in enumerate(shallow) if _in_range(e, emin, emax) is False]
            b_md = [i for i, e in enumerate(med) if _in_range(e, emin, emax) is False]
            b_dp = [i for i, e in enumerate(deep) if _in_range(e, emin, emax) is False]
            worst_i = min(range(len(rows)), key=lambda i: shallow[i])
            res.add(subject=gid, check="engagement_per_hole",
                    state=PASS if not (b_sh or b_md or b_dp or unmatched) else FAIL,
                    severity=BLOCK, measured=round(shallow[worst_i], 4),
                    criterion=f"{len(rows)} 个孔**各自**的咬入（长度 {L} − 该孔叠厚）∈ "
                              f"[{emin}, {emax}]：**最浅端（料最厚向）、中位、最深端（料最薄向）"
                              f"三个量各自都要在范围内**；孔由 feature_hole_map 的坐标逐个关联",
                    evidence_n=sum(r[4]["n"] for r in rows),
                    detail=f"最浅 {[round(x,3) for x in shallow]} / 中位 {[round(x,3) for x in med]}"
                           f" / 最深 {[round(x,3) for x in deep]}；超差：最浅端 {len(b_sh)} 个"
                           f"（孔 {[rows[i][0] for i in b_sh][:4]}）、中位 {len(b_md)} 个、"
                           f"最深端 {len(b_dp)} 个｜最差一孔 {rows[worst_i][0]}@{rows[worst_i][1]} "
                           f"{[round(x,2) for x in rows[worst_i][2]]}（关联径向残差 "
                           f"{rows[worst_i][3]:.3f} mm、轴向偏移 {rows[worst_i][5]:.2f} mm —— "
                           f"声明坐标是刀具中点，落在料外是正常的）"
                           + (f"｜{len(unmatched)} 个孔关联不上：{unmatched[:2]}" if unmatched else ""),
                    provenance=prov)
            # 接收孔的盲底余量：螺丝尖伸进去多少 vs 底孔自己的盲深
            if margin is not None:
                for fid, ppid, fe, pt, _idx in recv:
                    dep = gv((fe.get("geom") or {}).get("depth_mm"))
                    if dep is None or (fe.get("geom") or {}).get("depth_kind") != "blind_depth":
                        continue
                    lim = float(dep) - margin
                    tip = max(deep)
                    res.add(subject=gid, check=f"pilot_bottom_margin:{fid}",
                            state=PASS if tip <= lim else FAIL, severity=BLOCK,
                            measured=round(tip, 4),
                            criterion=f"螺丝伸进接收孔的最深量 ≤ 底孔盲深 {dep} − {margin} = "
                                      f"{round(lim,4)}（screw_length_rule.formula 的尖端余量，"
                                      f"底孔盲深取 features.yaml:{fid}.geom.depth_mm）",
                            evidence_n=len(rows),
                            detail=f"最深端用的是所有孔里料最薄的那一向（{len(rows)} 个孔）",
                            provenance=f"features.yaml:{fid}.geom.depth_mm | " + prov)
                    break

    # ══ 3. 舵盘时钟 ═══════════════════════════════════════════════════════
    feats = (D.get("features") or {}).get("features") or []
    horn_by_part: dict[str, list] = {}
    for f in feats:
        if (f.get("check_class") or f.get("kind")) == "horn_hole":
            horn_by_part.setdefault(f["part"], []).append(f)
    clock_bucket = buckets.get("fits.horn.clock_phase") or {}
    for pid, fl in sorted(horn_by_part.items()):
        if ctx.only and pid not in ctx.only:
            continue
        stl = ctx.stl(pid)
        if stl is None:
            res.unknown(pid, "horn_clock", f"找不到 {pid} 的导出 STL")
            continue
        pats = geom(stl).horn_patterns(float(m_through) if m_through else 2.2, dtol) if m_through else []
        if not pats:
            res.unknown(pid, "horn_clock",
                        f"features.yaml 声明 {len(fl)} 条 horn_hole，但导出 STL 里找不到 6 孔圆阵，"
                        f"零位相位无从量（空刀的典型症状）",
                        provenance="features.yaml:horn_hole")
            continue
        # 声明的相位：features.yaml / relations.yaml 里都没有"这一片舵盘的零位相位 = X°"，
        # tolerances 的 fits.horn.clock_phase.nominal_deg=0 也没说 0° 是相对哪条基准方向量的。
        decl_r = None
        for f in fl:
            m = _RX_R.search(str((f.get("nominal_source") or {}).get("v") or "")) or \
                _RX_R.search(str(f.get("spec_verbatim") or ""))
            if m:
                decl_r = float(m.group(1))
        nom_deg, nom_src = num(clock_bucket.get("nominal_deg"))
        for i, p in enumerate(pats):
            # 与坐标系无关的可测量：相邻孔夹角必须是 60°（错一个孔 = 某一档变 0/120）
            worst = max(abs(x - 60.0) for x in p["gaps"])
            if clk_hi is None:
                res.unknown(pid, f"horn{i}:clock_index", f"tolerances.yaml:{CLK_KEY}.max 取不到（实测最大偏 {worst:.4f}°）",
                            provenance=f"tolerances.yaml:{CLK_KEY}")
            elif clk_src is None:
                res.unknown(pid, f"horn{i}:clock_index",
                            f"阈值没有来源：tolerances.yaml:{CLK_KEY} 没有 src（±{clk_hi}°；实测最大偏 {worst:.4f}°）—— F-L5-4 不判",
                            provenance=f"tolerances.yaml:{CLK_KEY}")
            else:
                if clk_src == "assumed":
                    assumed.note(CLK_KEY, clk_hi, f"{pid}:horn{i}:clock_index")
                res.add(subject=pid, check=f"horn{i}:clock_index",
                        state=PASS if worst <= abs(float(clk_hi)) else FAIL,
                        severity=BLOCK, measured=round(worst, 4),
                        criterion=f"6 孔分度 60° ± {clk_hi}（{CLK_KEY}，{src_label(clk_src)}）；"
                                  f"装错一个孔会让某一档变成 0° 或 120°",
                        evidence_n=len(p["gaps"]), detail=f"实测夹角 {[round(x,3) for x in p['gaps']]}",
                        provenance=f"tolerances.yaml:{CLK_KEY}")
            if decl_r is not None and con_hi is None:
                res.unknown(pid, f"horn{i}:pattern_radius", f"tolerances.yaml:{CON_KEY}.max 取不到（实测 r {p['r_pattern']:.5f}）",
                            provenance=f"tolerances.yaml:{CON_KEY}")
            elif decl_r is not None and con_src is None:
                res.unknown(pid, f"horn{i}:pattern_radius",
                            f"阈值没有来源：tolerances.yaml:{CON_KEY} 没有 src（±{con_hi}；实测 r {p['r_pattern']:.5f} vs 声明 {decl_r}）—— F-L5-4 不判",
                            provenance=f"tolerances.yaml:{CON_KEY}")
            elif decl_r is not None:
                if con_src == "assumed":
                    assumed.note(CON_KEY, con_hi, f"{pid}:horn{i}:pattern_radius")
                res.add(subject=pid, check=f"horn{i}:pattern_radius",
                        state=PASS if abs(p["r_pattern"] - decl_r) <= float(con_hi) else FAIL,
                        severity=BLOCK, measured=round(p["r_pattern"], 5),
                        criterion=f"孔阵半径 = 声明 r{decl_r} ± {con_hi}"
                                  f"（{CON_KEY}，{src_label(con_src)}）",
                        evidence_n=6, detail="半径错了整片法兰都拧不上",
                        provenance="features.yaml:%s | tolerances.yaml:%s" % (fl[0]["id"], CON_KEY))
            else:
                res.unknown(pid, f"horn{i}:pattern_radius",
                            "features.yaml 这几条 horn_hole 没给孔阵半径 r，无法判",
                            provenance="features.yaml:horn_hole", severity=WARN)
            # 绝对相位：没有声明基准，判不了
            e1 = p["axis"]
            res.unknown(pid, f"horn{i}:clock_phase_vs_declared",
                        f"零位相位没有可比的声明值：features.yaml / relations.yaml 里都没有"
                        f"「这片舵盘零位相位 = X°、相对哪条基准方向」；tolerances.yaml:fits.horn."
                        f"clock_phase.nominal_deg = {nom_deg}（src={nom_src}，status="
                        f"{clock_bucket.get('status')}）也没给基准方向。"
                        f"实测（以孔阵轴 {np.round(e1, 3).tolist()} 为法向、任取基准）"
                        f"相邻孔角 {[round(x,2) for x in p['angles']]}，留给实物法兰核对",
                        provenance="relations.yaml:R06 | tolerances.yaml:fits.horn.clock_phase")

    # ══ 4. 功能关系 22 条 + C1/C2/C3 ═══════════════════════════════════════
    R = D.get("relations") or {}
    rels = list(R.get("relations") or []) + list(R.get("gate_entries_from_comparison_C") or [])
    if not rels:
        res.unknown("_relations", "list", "relations.yaml 里没有 relations")
    n_open = 0
    core_ids = {r["id"] for r in (R.get("relations") or [])}
    measured_buckets: dict[str, str] = {}      # 实测几何**通过**的配合桶 → 是哪条关系量的
    failed_buckets: dict[str, str] = {}        # 量了但没过的桶 → 关系 id（F-L5-3：不能再算"有实测撑着"）
    for r in rels:
        rid = r["id"]
        ref = r.get("target_ref")
        stem, path, node = _resolve_ref(D, ref)
        prov = f"relations.yaml:{rid}"
        # ── FG13：先量几何，再谈状态。status/actual_src 只决定证据等级，不能替代几何 ──
        cg = r.get("contact") or {}
        parties = cg.get("parties") or []
        ax, sgn = _axis_vec(cg.get("approach") or cg.get("axis"))
        bb = cg.get("probe_bbox_mm")
        lo_hi = None
        if isinstance(bb, (list, tuple)) and len(bb) == 2:
            try:
                lo_hi = (np.array([float(x) for x in bb[0]]),
                         np.array([float(x) for x in bb[1]]))
            except (TypeError, ValueError):
                lo_hi = None
        tlo, thi, tsrc = threshold_meta(node if isinstance(node, dict) else {}, "target_range_mm")
        tkey = f"{path}.target_range_mm"
        if not (len(parties) == 2 and ax is not None and lo_hi is not None):
            res.unknown(rid, "relation_contact",
                        f"这条关系没有可量的成对几何：relations.yaml:{rid} 缺 contact 段"
                        f"（需要 axis/approach + 两个 parties（part 或 placed）+ probe_bbox_mm）。"
                        f"between={str(r.get('between'))[:48]} 是自由文本，机器读不出是哪两个实体的"
                        f"哪两个面。**没有几何就没有结论** —— status='{r.get('status')}' 与 "
                        f"actual_src='{r.get('actual_src')}' 只是标签，不能当通过依据（元规则 4）",
                        provenance=prov + " | tolerances.yaml:" + str(path))
        else:
            pa, wa = _resolve_party(ctx, parties[0])
            pb, wb = _resolve_party(ctx, parties[1])
            if pa is None or pb is None:
                res.unknown(rid, "relation_contact",
                            f"contact.parties 解析不到落位件：{wa or ''} {wb or ''}",
                            provenance=prov)
            elif tlo is None and thi is None:
                res.unknown(rid, "relation_contact",
                            f"target_ref='{ref}' 的 target_range 两端都是 null，量出来也没有判据",
                            provenance=prov + " | tolerances.yaml:" + str(path))
            elif tsrc is None:
                res.unknown(rid, "relation_contact",
                            f"阈值没有来源：tolerances.yaml:{tkey} 没有 src（区间 [{tlo}, {thi}] 不知道是实测/手册还是拍脑袋）"
                            f"—— F-L5-4 不判；几何未量（量了也没有可信区间可比）",
                            provenance=prov + " | tolerances.yaml:" + tkey)
            else:
                try:
                    ga, gb = geom(pa), geom(pb)
                    for q in (pa, pb):
                        rp = str(q.relative_to(ROOT))
                        if rp not in res.inputs:
                            res.inputs.append(rp)
                    cp = contact_profile(ga, gb, ax, sgn, lo_hi[0], lo_hi[1])
                except Exception as e:
                    cp, ga = None, None
                    res.unknown(rid, "relation_contact", f"接触几何量不出来：{e}", provenance=prov)
                if cp is None:
                    if ga is not None:
                        res.unknown(rid, "relation_contact",
                                    f"探测窗 {bb} 里没有一格同时打到两方的料 —— 两件在这个窗里"
                                    f"根本不相对，接触量无从谈起", provenance=prov)
                else:
                    n_rays += cp["n_rays"]
                    need_touch = str(r.get("kind") or "") in (
                        "clamp", "press_fit", "bearing_seat", "driven_disc", "snap")
                    # 口径必须显式声明再比。contact_profile 量的是**有符号配合量（过盈为正、
                    # 间隙为负）**，而 tolerances.yaml 里一部分桶的 target_range_mm 写的是
                    # **间隙的绝对值（正数）** —— 直接比会让几何完全达标的条目判红。
                    # 没声明口径 = 判不了（元规则 4），不许猜。
                    trng = (node if isinstance(node, dict) else {}).get("target_range_mm") or {}
                    conv = trng.get("sign_convention")
                    if conv == "clearance_magnitude":
                        # 间隙绝对值 c = -signed；取负后两端会互换
                        cmp_lo, cmp_hi = -cp["signed_max"], -cp["signed_min"]
                        conv_note = "（本桶口径=间隙绝对值，已把实测有符号量取负后比较）"
                    elif conv == "signed_interference_positive":
                        cmp_lo, cmp_hi = cp["signed_min"], cp["signed_max"]
                        conv_note = "（本桶口径=有符号量，过盈为正）"
                    else:
                        res.unknown(rid, "relation_contact",
                                    f"target_ref='{ref}' 的 target_range_mm 没有声明 "
                                    f"sign_convention（meaning='{trng.get('meaning')}'）。"
                                    f"实测有符号配合量 {cp['signed_min']:.4f}..{cp['signed_max']:.4f}、"
                                    f"接触面积 {cp['contact_area_mm2']:.2f} mm² 已经量到了，"
                                    f"但不知道区间 [{tlo}, {thi}] 是间隙绝对值还是有符号量 —— "
                                    f"两种口径符号相反，猜错会把达标判成不达标",
                                    provenance=prov + " | tolerances.yaml:" + str(path))
                        continue
                    ok_fit = (_in_range(cmp_lo, tlo, thi) is not False and
                              _in_range(cmp_hi, tlo, thi) is not False)
                    ok_touch = (cp["contact_area_mm2"] > 0) if need_touch else True
                    if tsrc == "assumed":
                        assumed.note(tkey, [tlo, thi], f"{rid}:relation_contact", subject=str(path))
                    res.add(subject=rid, check="relation_contact",
                            state=PASS if (ok_fit and ok_touch) else FAIL, severity=BLOCK,
                            measured=round(cp["signed_med"], 4),
                            criterion=f"实测**有符号配合量**（过盈为正 / 间隙为负）两端都要落在 "
                                      f"[{tlo}, {thi}]（{ref}，{src_label(tsrc)}）{conv_note}"
                                      + (f"；{r.get('kind')} 类还必须有 > 0 的**接触面积**"
                                         f"（0 = 什么都没夹住）" if need_touch else ""),
                            evidence_n=cp["n_pair"],
                            detail=f"有符号配合量 {cp['signed_min']:.4f}..{cp['signed_max']:.4f}"
                                   f"（中位 {cp['signed_med']:.4f}）；接触面积 "
                                   f"{cp['contact_area_mm2']:.2f} mm²（投影重叠 "
                                   f"{cp['projected_area_mm2']:.2f} mm²，{cp['n_pair']}/"
                                   f"{cp['n_cell']} 格两方都有料）；夹紧行程 "
                                   f"{cp['clamp_travel_mm']:.4f} mm（动件还要走这么远才碰到）"
                                   f"｜status={r.get('status')} / actual_src={r.get('actual_src')} "
                                   f"这两个标签**没有参与判定**",
                            provenance=prov + " | tolerances.yaml:" + str(path))
                    # F-L5-3（2026-09-13）：只有 relation_contact 真过了才算"该桶有实测几何撑着"；
                    # 量了但红的记到 failed_buckets，下面桶级 fit_measured_geometry 判 FAIL 而不是 PASS。
                    if path:
                        if ok_fit and ok_touch:
                            measured_buckets[str(path)] = rid
                        else:
                            failed_buckets.setdefault(str(path), rid)
        if node is None:
            res.unknown(rid, "target_ref",
                        f"target_ref='{ref}' 在 {stem}.yaml 里解析不到（路径 '{path}' 不存在）"
                        f" —— 判据取不到，这条关系没法判",
                        provenance=prov)
        st = r.get("status")
        sev = r.get("severity") or BLOCK
        if st == "open":
            n_open += 1 if rid in core_ids else 0
            res.add(subject=rid, check="relation_status", state=FAIL, severity=sev,
                    measured="open", criterion="relations.yaml 注释：status=open 表示没有满足的证据，"
                                               "第 5/7 层判 BLOCK",
                    evidence_n=1,
                    detail=f"{str(r.get('declared_target'))[:44]} | 现状：{str(r.get('actual'))[:64]}",
                    provenance=prov + " | " + str(ref))
        else:
            res.add(subject=rid, check="relation_status", state=PASS if st == "ok" else FAIL,
                    severity=sev if st != "ok" else INFO, measured=st,
                    criterion="relations.yaml:status 必须是 ok",
                    evidence_n=1, detail=str(r.get("actual"))[:70], provenance=prov)
        src = r.get("actual_src")
        if src in (None, "assumed"):
            res.add(subject=rid, check="relation_evidence", state=FAIL, severity=WARN,
                    measured=str(src),
                    criterion="actual_src 必须是 measured/datasheet（assumed/null = 现状值本身是猜的）",
                    evidence_n=1, detail=str(r.get("note") or "")[:70], provenance=prov)
        else:
            res.add(subject=rid, check="relation_evidence", state=PASS, severity=INFO, measured=src,
                    criterion="actual_src ∈ {measured, datasheet}", evidence_n=1, provenance=prov)
    declared_open = ((R.get("counts") or {}).get("open"))
    res.add(subject="_relations", check="open_count",
            state=PASS if declared_open == n_open else FAIL, severity=WARN, measured=n_open,
            criterion=f"relations 里 status=open 的条数应与 relations.yaml:counts.open（{declared_open}）一致"
                      f"（C1/C2/C3 不计入 counts.open）",
            evidence_n=len(rels), provenance="relations.yaml:counts.open")

    # ══ 5. 配合数值 + 占位值 ═══════════════════════════════════════════════
    ph = T.get("placeholder_summary") or {}
    declared_ph = set(ph.get("fits_on_placeholder") or [])
    found_ph = set()
    for bpath, b in sorted(buckets.items()):
        prov = f"tolerances.yaml:{bpath}"
        is_deg = "target_range_deg" in b
        nom_field = b.get("nominal_deg") if is_deg else b.get("nominal_mm")
        v, src = num(nom_field)
        rng_key = "target_range_deg" if is_deg else "target_range_mm"
        lo, hi, tsrc = threshold_meta(b, rng_key)
        unit = "°" if is_deg else "mm"
        if src in (None, "unknown", "assumed") or v is None:
            found_ph.add(bpath)
        if isinstance(v, dict):
            res.unknown(bpath, "fit_value",
                        f"nominal 是复合结构 {sorted(v)}，而 target_range 是单一量"
                        f"（'{b.get('target_range_mm', {}).get('meaning')}'），"
                        f"tolerances.yaml 没说用哪一项去比 —— 判不了",
                        provenance=prov)
        elif v is None:
            res.unknown(bpath, "fit_value",
                        f"nominal 是 null（status={b.get('status')}，"
                        f"{nom_field.get('unknown_reason') if isinstance(nom_field, dict) else ''}）"
                        f"，目标区间 [{lo}, {hi}]{unit} 无值可比",
                        provenance=prov)
        elif lo is None and hi is None:
            res.unknown(bpath, "fit_value",
                        f"target_range 两端都是 null（status={b.get('status')}），没有判据",
                        provenance=prov)
        elif tsrc is None:
            res.unknown(bpath, "fit_value",
                        f"阈值没有来源：tolerances.yaml:{bpath}.{rng_key} 没有 src（区间 [{lo}, {hi}]{unit} 不知道是实测/手册"
                        f"还是拍脑袋；nominal={v}）—— F-L5-4 不判",
                        provenance=prov + f".{rng_key}")
        elif not is_deg and isinstance(hi, (int, float)) and float(v) > float(hi) * 3 + 1.0:
            res.unknown(bpath, "fit_value",
                        f"nominal={v}{unit} 与 target_range [{lo}, {hi}]{unit} 不是同一个量"
                        f"（'{(b.get('target_range_mm') or {}).get('meaning')}'）—— "
                        f"tolerances.yaml 这个桶的 nominal 是尺寸、target 是间隙，机械地比毫无意义。"
                        f"conflicts 里 '{'seat_d' if 'plain_slide' in bpath else '?'}' 说 Gate 必须按保守值判，"
                        f"但保守值只写在散文里，没有机器可读字段",
                        provenance=prov + " | tolerances.yaml:conflicts")
        else:
            ok = _in_range(float(v), lo, hi)
            if tsrc == "assumed":
                assumed.note(f"{bpath}.{rng_key}", [lo, hi], f"{bpath}:fit_value", subject=bpath)
            # FG13：这条只证明**清单自己前后一致**（声明的名义值落在自己声明的区间里），
            # 它不是"成品这个配合达成了"的证据 —— 所以通过时只记 INFO，
            # 真正的放行依据是下面那条 fit_measured_geometry（实测几何）。
            res.add(subject=bpath, check="fit_value", state=PASS if ok else FAIL,
                    severity=BLOCK if not ok else INFO,
                    measured=v, criterion=(f"（清单自洽）nominal ∈ [{lo}, {hi}]{unit}（{src_label(tsrc)}；"
                                           f"{(b.get('target_range_mm') or b.get('target_range_deg') or {}).get('meaning')}）")[:170],
                    evidence_n=1,
                    detail=f"src={src}, status={b.get('status')}｜这是纸面比较，不是成品证据",
                    provenance=prov)
        # 每个配合桶都要有**实测几何**撑着（元规则 4）。名义值比区间、status=ok、
        # actual_src=measured 都只是标签/纸面数，不能让格子变绿 —— 这正是 H02 那次
        # 「0.30 均匀间隙、干涉=0、什么都没夹住」能全绿的原因。
        if bpath in measured_buckets:
            res.add(subject=bpath, check="fit_measured_geometry", state=PASS, severity=BLOCK,
                    measured=measured_buckets[bpath],
                    criterion="该配合桶必须有一条 relations 用**实测几何**（接触面积 / 有符号配合量 / "
                              "夹紧行程）验证过，**且那条 relation_contact 为 PASS**（量了但没过不算撑着）",
                    evidence_n=1, detail=f"由 {measured_buckets[bpath]} 的 relation_contact 量出且通过",
                    provenance=prov + f" | relations.yaml:{measured_buckets[bpath]}")
        elif bpath in failed_buckets:
            rid_f = failed_buckets[bpath]
            res.add(subject=bpath, check="fit_measured_geometry", state=FAIL, severity=BLOCK,
                    measured=rid_f,
                    criterion="该配合桶必须有一条 relations 用**实测几何**（接触面积 / 有符号配合量 / "
                              "夹紧行程）验证过，**且那条 relation_contact 为 PASS**（量了但没过不算撑着）",
                    evidence_n=1,
                    detail=f"{rid_f} 量了但没过：它的 relation_contact 是 FAIL —— 实测几何说这个配合没达成，"
                           f"桶不能因为『有人量过』就绿（F-L5-3）",
                    provenance=prov + f" | relations.yaml:{rid_f}")
        else:
            res.unknown(bpath, "fit_measured_geometry",
                        f"没有任何 relations 条目用实测几何验证过这个桶：指向它的关系要么没有 "
                        f"contact 段（parties + axis + probe_bbox_mm），要么根本没有关系指过来。"
                        f"nominal={v}{unit} 与 target [{lo}, {hi}] 的比较是**纸面比较**，"
                        f"status='{b.get('status')}' 是标签 —— 都不能当成品配合的证据",
                        provenance=prov + " | relations.yaml:relations[].contact")
        if src == "assumed" or (v is not None and src in (None, "unknown")):
            res.add(subject=bpath, check="fit_on_placeholder", state=FAIL, severity=WARN,
                    measured=f"src={src}, coupon={b.get('coupon')}",
                    criterion="配合数值必须来自实测/手册；src=assumed 表示这条配合靠占位值撑着",
                    evidence_n=1,
                    detail=str((nom_field or {}).get("src_note") or "")[:80] if isinstance(nom_field, dict) else "",
                    provenance=prov + " | tolerances.yaml:placeholder_summary")
    res.add(subject="_fits", check="placeholder_summary_consistent",
            state=PASS if declared_ph == found_ph else FAIL, severity=WARN,
            measured=len(found_ph),
            criterion=f"实际靠占位值的桶应与 placeholder_summary.fits_on_placeholder"
                      f"（{ph.get('fits_on_placeholder_count')} 个）一致",
            evidence_n=len(buckets),
            detail=f"清单有本层没认出的：{sorted(declared_ph - found_ph)[:4]}；"
                   f"本层认出清单没写的：{sorted(found_ph - declared_ph)[:4]}",
            provenance="tolerances.yaml:placeholder_summary")
    cp, ct = ph.get("coupons_printed"), ph.get("coupons_total")
    res.add(subject="_fits", check="coupons_printed", state=PASS if cp == ct else FAIL,
            severity=BLOCK, measured=cp,
            criterion=f"公差试件应全部打完（{ct} 件）；一件没打 = 所有过盈/间隙都是名义值",
            evidence_n=ct or 0, detail=str(ph.get("note") or "")[:90],
            provenance="tolerances.yaml:placeholder_summary")

    assumed.emit(res, NAME)
    res.evidence = {"fastener_groups": len(fasteners),
                    "screws_total": sum(f.get("qty") or 0 for f in fasteners),
                    "relations": len(rels), "fit_buckets": len(buckets),
                    "seat_rays": n_rays, "assumed_thresholds_used": len(assumed.used)}
    return res
