#!/usr/bin/env python3
"""第 5 层追加判据 G-A：螺丝头。

起因（2026-09-09，用户打出测试台实物后提出）：现有 L5 只查**咬入**和**总长**，
从来没查过**头**。测试台摆臂 6 颗 M2 的头凸在毂外面、Ø5 垫圈够不够坐、会不会碰到东西
—— 一次都没算过；它没出事是因为那个方向恰好没东西，不是因为查过。

本模块回答每颗螺丝四个问题（**逐孔**，不跨孔取统计量，不限前几个孔）：
  A1 头坐在哪         承压面在不在、有效承压面积够不够（足印环带 N_RING 圈 × N_ANG 向全要有料）
  A2 坐面平不平       **每一个孔**自己的头足印内承压面轴向极差 —— 等厚斜面的料厚极差是 0，
                     只有这条看得见（FG15）。头侧**只**按 fasteners.yaml:tool_access[].seats[].
                     outward_export_local 取（F-L5-2），取不到判 unknown，不取较差端
  A3 沉孔配不配套     沉孔够宽够深吗（深 < 头高 = 头仍凸出，却已经削掉了叠厚）
  A3b 叠厚对不对账    从**实测的坐面**沿孔轴重新量一遍叠厚，和 fasteners.yaml 记的对账
                     —— 不重复扣沉孔：射线本来就不计空腔，记录的 stack_mm 已含沉孔
                     —— **逐孔比**，不取全组中位数（8 个平的会把 1 个斜的压下去）
  A4 头凸出多少、碰不碰   头实体 vs 其它已装件

判据全部从 data 取，一个数字都不在层里写死；取不到就判 unknown（元规则 4）：
  头几何   frozen.yaml:P_dict.m<N>_head（d/h）—— N 由该组 spec 里的 M<N> 决定，不写死 m2
  过孔径   features.yaml:<孔>.geom.nominal_d_mm，沉孔/铣面类退回 fits.*.m<N>_through_hole
  沉孔     tolerances.yaml:fits.*.m<N>_counterbore
  坐面     tolerances.yaml:feature_check_tolerances.seat_flatness_spread_mm
  叠厚对账 STACK_TOL_REFS 按序找（专属字段 → 轴向尺寸通用容差）
  咬入     tolerances.yaml:engagement_by_joint_type[<joint_type>]

坐标：`features.yaml` 的特征**全部**声明 `frame: export_local`，本模块直接在 `cad/duck_s288/<件>.stl`
上做（元规则 1：查导出文件，不查内存网格）。每个孔按它自己特征的 part 取件 —— 一组螺丝可以跨件（穿件 + 接收件）。

起点（2026-09-13，F-L5-2 第二半）：`feature_hole_map[].holes` = `features.yaml:hole_positions_mm` 是**刀心**（切刀中点），
40 mm 的刀切 3.6 mm 的板时它离件 15–19 mm、甚至在件外 —— 它**不是坐面**，不当射线起点。头判据按
`fasteners.yaml:<组>.tool_access[].seats[]` 逐坐面判：起点 = `point_export_local`、头侧 = `outward_export_local`；
刀心只当**轴线锚**：坐面点到该孔轴的横向偏差 > LATERAL_TOL（1e-6，与 tools/gate/tool_access.py 同一规则）判 unknown。
匹配键带 `instance`：左件/右件（mirror_y 导出，export_local 坐标相同、同一份 STL）同一孔同 outward 不是冲突，
几何量一次、记录覆盖了哪些 instance；"冲突"只在**同一 instance** 同一孔给了不同 outward 时成立。
头侧判据只覆盖 `head_locator_map_indices` 里的孔 —— 同一颗螺丝的 counterbore/spot_face 条目不是另一颗螺丝，不计入"未声明"。
取不到坐面的头侧孔 → 该组 head_seat / head_side unknown，不猜、不取较差端；没有 tool_access 的组保持 unknown。

孔轴：优先取特征声明轴（`geom.axis`，有 `geom.instances` 时按 `feature_hole_map[].instance` 取那一实例）；
实测轴仍量（起点在孔腔里：坐面点沿 −outward 进 0.5×孔径，不再从刀心打），声明与实测不符单独报
`hole_axis_declared_vs_measured`，不静默采信任何一方。声明轴缺失时用实测轴。

阈值来源（2026-09-13，F-L5-4）：层读阈值时把 src 写进 criterion（"阈值 src=measured/datasheet/assumed/缺"）；
阈值节点**没有 src** → 该判据 unknown（"阈值没有来源"）；src=assumed → 判据照跑，另发一条
`threshold_assumed:<键>` FAIL(WARN)（measured=阈值本身），让"对着采用阈值的 PASS"在记分卡上看得见。
"""
import math
import re
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from core import LayerResult, PASS, FAIL, BLOCK, WARN, INFO, num, ROOT  # noqa: E402

LAYER = 5
NAME = "螺丝头 G-A"

# 这些连接类型没有"螺丝头"，头判据整条不适用（扎带、卡珠、无接收材料）
NOT_A_SCREW = {"none_zip_tie", "pla_snap", "none_no_receiver"}
# 导出 STL 顶点是 float32，量出来的长度带 ~1e-6 mm 表示噪声。不是放宽判据，
# 是不让绿红由 float32 最后一位决定（见 l5_function._NUM_EPS 同一注释）。
NUM_EPS = 1e-6
N_ANG = 24          # 头足印圆周采样方向数（采样密度，不是判据）
N_RING = 3          # 头足印环带上取几圈半径（同上）
RAY_BACK = 60.0     # 射线起点退到件外多远（同上）

# 坐面点到所绑定孔轴的横向偏差容许量（数值口径，与 tools/gate/tool_access.py 同一规则；不是配合公差）
LATERAL_TOL = 1e-6

# 叠厚对账的容差没有专属字段，按优先级从数据里找同量纲（轴向尺寸）的判据；
# 一个都取不到就判 unknown（元规则 4），层里不写死数字。
STACK_TOL_REFS = ("feature_check_tolerances.stack_crosscheck_mm",
                  "feature_check_tolerances.hole_depth_mm")


def _dig(root, dotted):
    cur = root
    for k in dotted.split("."):
        if not isinstance(cur, dict) or k not in cur:
            return None
        cur = cur[k]
    return cur


# ── 阈值来源（F-L5-4）────────────────────────────────────────────────────────
def threshold_meta(node, key=None):
    """阈值节点 → (min, max, src)。key 给了就取 node[key]（如 target_range_mm），否则 node 自己就是 {min,max,src}。
    src 为 None = 节点没有 src（调用方判 unknown：阈值没有来源）。"""
    if not isinstance(node, dict):
        return None, None, None
    rng = node.get(key) if key else node
    if not isinstance(rng, dict):
        return None, None, None
    src = rng.get("src")
    return rng.get("min"), rng.get("max"), (str(src) if src not in (None, "") else None)


def src_label(src):
    return f"阈值 src={src or '缺'}"


class AssumedThresholds:
    """收集本轮被引用过的 src=assumed 阈值，run 末尾每 (subject, key) 发一条 threshold_assumed FAIL(WARN)。"""

    def __init__(self):
        self.used = {}

    def note(self, key, value, consumer, subject="_thresholds"):
        rec = self.used.setdefault((subject, key), [value, []])
        rec[1].append(consumer)

    def emit(self, res, layer_tag):
        for (subject, key), (value, consumers) in sorted(self.used.items(), key=lambda kv: (kv[0][0], kv[0][1])):
            res.add(subject=subject, check=f"threshold_assumed:{key}", state=FAIL, severity=WARN,
                    measured=value,
                    criterion="F-L5-4：阈值 src=assumed（区间由 tolerances.yaml 提出、无试件/手册来源）—— 引用它的判据照判，"
                              "但本条 WARN 让『对着采用阈值的 PASS』在记分卡上看得见；阈值补上 measured/datasheet 来源后本条消失",
                    evidence_n=len(consumers),
                    detail=f"tolerances.yaml:{key} = {value}；{layer_tag} 里 {len(consumers)} 条判据引用它：{consumers[:6]}",
                    provenance=f"tolerances.yaml:{key}")


def _stack_tol(tol):
    """(容差, 出处, src)；取不到给 (None, 已找过的路径, None)。"""
    for ref in STACK_TOL_REFS:
        node = _dig(tol, ref)
        if isinstance(node, dict):
            lo, hi, src = threshold_meta(node)
            vals = [abs(float(x)) for x in (lo, hi) if isinstance(x, (int, float))]
            if vals:
                return max(vals), f"tolerances.yaml:{ref}", src
    return None, " / ".join(f"tolerances.yaml:{r}" for r in STACK_TOL_REFS), None


def _head_geom(frozen, dnom):
    """按螺丝规格从 frozen.yaml:P_dict 取头几何 —— 以前写死读 m2_head，M3 也会拿 M2 的头。"""
    key = f"m{int(dnom)}_head" if float(dnom).is_integer() else f"m{dnom}_head"
    node = ((frozen or {}).get("P_dict") or {}).get(key) or {}
    d, ds = num(node.get("d"))
    h, hs = num(node.get("h"))
    return d, h, ds, hs, f"frozen.yaml:P_dict.{key}"


_NOT_THE_SHANK_HOLE = {"counterbore", "spot_face", "tool_channel"}


def _hole_d(tol, feat, dnom):
    """螺丝**杆**穿过去的那个孔的名义直径 —— 头足印环带的内半径就是它。

    沉孔 / 铣面 / 起子通道的 geom.nominal_d_mm 是**沉孔**直径（Ø4.2），比头径还大，
    拿它当内半径会让整条环带落到沉孔壁上打擦边射线（量出 1.0 = 沉孔深这种假极差）。
    这几类一律退回公差表里同规格的过孔桶。
    """
    kind = (feat or {}).get("check_class") or (feat or {}).get("kind")
    if kind not in _NOT_THE_SHANK_HOLE:
        v, _s = num(((feat or {}).get("geom") or {}).get("nominal_d_mm"))
        if v is not None:
            return float(v), f"features.yaml:{(feat or {}).get('id')}.geom.nominal_d_mm"
    key = f"m{int(dnom)}_through_hole" if float(dnom).is_integer() else f"m{dnom}_through_hole"
    for mat, sub in ((tol.get("fits") or {}).items()):
        if isinstance(sub, dict) and key in sub:
            v, _s = num((sub[key] or {}).get("nominal_mm"))
            if v is not None:
                return float(v), f"tolerances.yaml:fits.{mat}.{key}.nominal_mm"
    return None, None


def _ring_radii(r_in, r_out, n=N_RING):
    """等面积子环带的中线半径：不压在孔壁/头沿上打擦边射线，面积统计也无偏。"""
    a, b = float(r_in), float(r_out)
    if b <= a:
        return [a]
    return [math.sqrt(a * a + (k + 0.5) / n * (b * b - a * a)) for k in range(n)]


def hole_geom(fe, entry):
    """features.yaml 特征的 geom；有 `geom.instances` 时按 `feature_hole_map[].instance` 取那一实例覆盖顶层
    （axis / pos / nominal_d_mm 等按实例）。取不到实例就只给顶层。"""
    fg = dict(((fe or {}).get("geom") or {}))
    insts = fg.get("instances")
    inst = (entry or {}).get("instance")
    if isinstance(insts, list) and type(inst) is int and 0 <= inst < len(insts) and isinstance(insts[inst], dict):
        fg.update(insts[inst])
    elif isinstance(insts, dict) and inst in insts and isinstance(insts[inst], dict):
        fg.update(insts[inst])
    return fg


def _unit(v):
    try:
        a = np.asarray(v, float)
    except (TypeError, ValueError):
        return None
    if a.shape != (3,) or not np.isfinite(a).all() or np.linalg.norm(a) <= 0:
        return None
    return a / np.linalg.norm(a)


def declared_head_sides(fa):
    """fasteners.yaml:<组>.tool_access[].seats[] → {(feature_id, instance, map_index, hole_index): 头侧单位向量(export_local)}。

    F-L5-2（2026-09-13）：头侧**只**从这里取（seat.outward_export_local，与 tools/gate/tool_access.py 读同一字段）。
    键带 instance：左件 `hip` 与右件 `hip_R` 是两颗螺丝，各自一条；**同一 instance** 同一孔出现两条方向不一致
    → 记 None（= 冲突）。方向一致的重复条目只算一条。取不到 → 调用方判 unknown，不猜。"""
    out = {}
    for op in (fa.get("tool_access") or []):
        if not isinstance(op, dict):
            continue
        for seat in (op.get("seats") or []):
            if not isinstance(seat, dict):
                continue
            key = (seat.get("feature_id"), seat.get("instance"), seat.get("map_index"), seat.get("hole_index"))
            a = _unit(seat.get("outward_export_local"))
            if a is None:
                continue
            if key not in out:
                out[key] = a
            elif out[key] is not None and float(out[key] @ a) < 0.999:
                out[key] = None
    return out


def seat_table(fa):
    """tool_access[].seats[] 按孔身份归并：
        {"holes": {(feature_id, map_index, hole_index): {"rows": [{"point", "outward", "instances"}],
                                                         "instances": set, "conflicts": [instance…]}},
         "instances": set(全组出现过的 instance)}
    同一孔身份、坐面点与 outward 都相同的左右实例合成一行（同一份 STL、同一坐标 → 几何只量一次，instances 记谁被覆盖）；
    坐面点不同则各一行。conflicts = declared_head_sides 判为冲突的 instance。"""
    sides = declared_head_sides(fa)
    holes, instances = {}, set()
    for op in (fa.get("tool_access") or []):
        if not isinstance(op, dict):
            continue
        for seat in (op.get("seats") or []):
            if not isinstance(seat, dict):
                continue
            fid, inst, mi, hi = seat.get("feature_id"), seat.get("instance"), seat.get("map_index"), seat.get("hole_index")
            pt = None
            try:
                pt = np.asarray(seat.get("point_export_local"), float)
            except (TypeError, ValueError):
                pt = None
            ax = _unit(seat.get("outward_export_local"))
            if pt is None or pt.shape != (3,) or not np.isfinite(pt).all() or ax is None:
                continue
            instances.add(inst)
            rec = holes.setdefault((fid, mi, hi), {"rows": [], "instances": set(), "conflicts": []})
            rec["instances"].add(inst)
            if sides.get((fid, inst, mi, hi)) is None:
                if inst not in rec["conflicts"]:
                    rec["conflicts"].append(inst)
                continue
            for row in rec["rows"]:
                if np.linalg.norm(row["point"] - pt) <= LATERAL_TOL and float(row["outward"] @ ax) >= 0.999:
                    if inst not in row["instances"]:
                        row["instances"].append(inst)
                    break
            else:
                rec["rows"].append({"point": pt, "outward": ax, "instances": [inst]})
    return {"holes": holes, "instances": instances}


def _hits_along(mesh, origins, direction):
    """一批平行射线（multiple_hits）：每条射线 → [(t, entering), …] 按 t 升序。entering = 面法向逆着射线（进料）。"""
    dirs = np.tile(np.asarray(direction, float), (len(origins), 1))
    loc, idr, tri = mesh.ray.intersects_location(origins, dirs, multiple_hits=True)
    per = [[] for _ in range(len(origins))]
    if len(loc):
        t = (loc - origins[idr]) @ np.asarray(direction, float)
        entering = (mesh.face_normals[tri] @ np.asarray(direction, float)) < 0
        for i in range(len(idr)):
            per[idr[i]].append((float(t[i]), bool(entering[i])))
    return [sorted(x) for x in per]


def _seat_end(mesh, pt, ax, sgn, radii, zone, n=N_ANG):
    """从头侧看螺丝头足印环带。起点 = **坐面点** pt（tool_access 声明），沿 sgn·ax 退 RAY_BACK 再向内打；
    每条射线取**头高区**（坐面外侧 ≤ zone = 头高）内的第一张进料面在孔轴上的位置（相对坐面点，头侧为正）——
    更外侧的无关结构（杯壁、另一堵墙）不是坐面，不算；头高区内没有进料面 = 该采样点头下悬空（miss）。
    radii[0] 那一圈（最靠孔壁）顺带量**该孔自己的叠厚**：从那张进料面到紧接着的出料面 = 第一段连续实体
    （沉孔是空的，射线直接穿过去，第一段实体自然从沉孔底开始；头高区外的无关结构不算）。
    返回 miss/tot/spread(极差)/med(中位位置)/n_hit/run_med(叠厚中位)/run_n/run_tot。"""
    out = sgn * np.asarray(ax, float)
    org = np.vstack([_ring(pt + out * RAY_BACK, ax, r, n) for r in radii])
    depth, miss, runs = [], 0, []
    for i, hits in enumerate(_hits_along(mesh, org, -out)):
        ent = [(t, RAY_BACK - t) for t, e in hits if e and RAY_BACK - t <= zone + NUM_EPS]
        if not ent:
            miss += 1
            continue
        t0, d0 = ent[0]                       # 沿射线最先碰到的、落在头高区内的进料面
        depth.append(d0)
        if i < n:                             # radii[0] 的那一圈 → 叠厚
            exits = [t for t, e in hits if (not e) and t > t0 + NUM_EPS]
            if exits:
                runs.append(float(exits[0] - t0))
    res = dict(miss=miss, tot=len(org), run_med=(float(np.median(runs)) if runs else None),
               run_n=len(runs), run_tot=n)
    if not depth:
        res.update(spread=0.0, med=None, n_hit=0)
        return res
    d = np.asarray(depth, float)
    res.update(spread=float(np.ptp(d)), med=float(np.median(d)), n_hit=len(d))
    return res


def _spec_len(spec):
    """'M2×6' / 'M2x6' / 'M2×8 自攻' → (2.0, 6.0)。解析不出返回 (None, None)。"""
    if not isinstance(spec, str):
        return None, None
    m = re.search(r"M(\d+(?:\.\d+)?)\s*[×x*]\s*(\d+(?:\.\d+)?)", spec)
    return (float(m.group(1)), float(m.group(2))) if m else (None, None)


AXES = {"x": np.array([1.0, 0, 0]), "y": np.array([0, 1.0, 0]), "z": np.array([0, 0, 1.0])}


def _declared_axis(a):
    """'z' / '-z' / None → 单位向量或 None。符号只影响朝向，不影响轴。"""
    if not isinstance(a, str):
        return None
    t = a.strip().lstrip("+")
    sgn = -1.0 if t.startswith("-") else 1.0
    t = t.lstrip("-")
    return sgn * AXES[t] if t in AXES else None


def _measure_axis(mesh, pt, reach=40.0):
    """孔轴自测：从**孔腔里的点**沿 ±x/±y/±z 各打一条射线，取“两个方向上最近命中距离的较小者”
    最大的那根轴 —— 孔轴方向上射线在空腔里走得最远，垂直方向立刻撞孔壁。
    起点必须在孔腔里（调用方给坐面点沿 −outward 进 0.5×孔径）—— 从件外的刀心打会抓到别的圆柱（F01/F06 09-13）。
    返回 (轴向量, 该轴的自由行程, 次优轴的自由行程)。"""
    scores = []
    for name, a in AXES.items():
        free = []
        for sgn in (1.0, -1.0):
            loc, idr, _ = mesh.ray.intersects_location(
                np.array([pt]), np.array([sgn * a]), multiple_hits=False)
            free.append(float(np.linalg.norm(loc[0] - pt)) if len(loc) else reach)
        scores.append((min(free), name, a))
    scores.sort(reverse=True)
    return scores[0][2], scores[0][0], scores[1][0]


def _first_hit(mesh, origins, direction):
    """一批平行射线，各自最近命中点沿 direction 的标量位置；未命中为 nan。"""
    dirs = np.tile(direction, (len(origins), 1))
    loc, idr, _ = mesh.ray.intersects_location(origins, dirs, multiple_hits=False)
    out = np.full(len(origins), np.nan)
    if len(loc):
        t = (loc - origins[idr]) @ direction
        for i, ri in enumerate(idr):
            if np.isnan(out[ri]) or t[i] < out[ri]:
                out[ri] = t[i]
    return out


def _ring(center, axis, r, n=N_ANG):
    """垂直 axis 的圆环采样点。"""
    a = np.array(axis, float)
    a = a / np.linalg.norm(a)
    ref = np.array([1.0, 0, 0]) if abs(a[0]) < 0.9 else np.array([0, 1.0, 0])
    u = np.cross(a, ref); u /= np.linalg.norm(u)
    v = np.cross(a, u)
    th = np.linspace(0, 2 * math.pi, n, endpoint=False)
    return np.array(center) + r * (np.outer(np.cos(th), u) + np.outer(np.sin(th), v))


def _locators(F):
    """head_locator_map_indices → 合法的下标列表或 None（缺/重复/越界/非整数）。"""
    hmap = F.get("feature_hole_map") or []
    locs = F.get("head_locator_map_indices")
    if (not isinstance(locs, list) or not locs or len(set(locs)) != len(locs)
            or any(type(i) is not int or i < 0 or i >= len(hmap) for i in locs)):
        return None
    return locs


def run(ctx) -> LayerResult:                                              # noqa: C901
    res = LayerResult(LAYER, NAME)
    data = ctx.data
    frozen = data.get("frozen") or {}
    tol = (data.get("tolerances") or {})
    fct = tol.get("feature_check_tolerances") or {}
    _flat_lo, flat_max, flat_src = threshold_meta(fct.get("seat_flatness_spread_mm"))
    FLAT_KEY = "feature_check_tolerances.seat_flatness_spread_mm"
    eng_by = tol.get("engagement_by_joint_type") or {}
    feats = {f["id"]: f for f in ((data.get("features") or {}).get("features") or [])}
    fasteners = (data.get("fasteners") or {}).get("fasteners") or []
    stack_tol, stack_tol_ref, stack_src = _stack_tol(tol)
    assumed = AssumedThresholds()

    rays_total = holes_done = 0
    instances_covered = 0

    for F in fasteners:
        fid = F.get("id", "?")
        if ctx.only and not any(p in str(F.get("joins", "")) + str(F.get("location", "")) for p in ctx.only):
            continue
        if F.get("joint_type") in NOT_A_SCREW:
            res.add(subject=fid, check="head_criteria_applicable", state=PASS, severity=INFO,
                    measured=F.get("joint_type"), criterion="非螺丝连接，G-A 头判据不适用",
                    evidence_n=1, provenance="G-A · fasteners.yaml:joint_type")
            continue
        dnom, L = _spec_len(F.get("spec"))
        if dnom is None:
            res.unknown(fid, "head_geometry",
                        f"spec='{str(F.get('spec'))[:30]}' 里解析不出螺丝规格，"
                        f"不知道该用哪一号的头几何（frozen.yaml:P_dict.m<N>_head）",
                        provenance="fasteners.yaml:spec | frozen.yaml:P_dict")
            continue
        # 头几何按**这颗螺丝自己的规格**从数据取，不再写死 m2_head
        head_d, head_h, hd_src, hh_src, head_ref = _head_geom(frozen, dnom)
        if head_d is None or head_h is None:
            res.unknown(fid, "head_geometry",
                        f"{head_ref} 没有可用的 d/h —— M{dnom:g} 的头径/头高没有数据，"
                        f"承压面、沉孔配套、凸出量三条都判不了",
                        provenance=head_ref)
            continue
        head_r = head_d / 2.0
        res.add(subject=fid, check="head_geometry_source", state=PASS, severity=INFO,
                measured=f"Ø{head_d}×{head_h}",
                criterion=f"头几何按 spec 的 M{dnom:g} 从 {head_ref} 取，不在层里写死",
                evidence_n=1, detail=f"来源等级 d={hd_src} h={hh_src}", provenance=head_ref)

        hmap = F.get("feature_hole_map") or []
        n_coords = sum(len(e.get("holes") or []) for e in hmap if isinstance(e, dict))
        prov_map = "fasteners.yaml:feature_hole_map"
        prov_side = f"fasteners.yaml:{fid}.tool_access[].seats[]"
        if not n_coords:
            res.unknown(fid, "head_seat", "没有孔坐标（feature_hole_map 缺 holes），头的位置无从谈起",
                        provenance=prov_map)
            continue

        # 沉孔：同件同位置有没有 counterbore 特征
        cbf = next((feats[i] for i in (F.get("feature_ids") or [])
                    if i in feats and feats[i].get("kind") == "counterbore"), None)
        cb_d, _ = num(((cbf or {}).get("geom") or {}).get("nominal_d_mm"))
        cb_dep, _ = num(((cbf or {}).get("geom") or {}).get("depth_mm"))
        cbb, cb_key = None, None
        for _mat, sub in ((tol.get("fits") or {}).items()):
            key = f"m{int(dnom)}_counterbore" if float(dnom).is_integer() else f"m{dnom}_counterbore"
            if isinstance(sub, dict) and key in sub:
                cbb = sub[key] or {}
                cb_key = f"fits.{_mat}.{key}.target_range_mm"
                break
        cb_lo, _cb_hi, cb_src = threshold_meta(cbb, "target_range_mm")

        stack_rec, stack_src_rec = num(F.get("stack_mm"))
        rng = (eng_by.get(F.get("joint_type")) or {}).get("range_mm") or {}
        emin, emax = rng.get("min"), rng.get("max")

        # ── 头侧孔全集 = head_locator_map_indices 指向的 map 项（沉孔/铣面条目是同一颗螺丝，不是另一颗）──
        locs = _locators(F)
        if locs is None:
            res.unknown(fid, "head_side",
                        f"fasteners.yaml:{fid}.head_locator_map_indices 缺/重复/越界 —— 不知道 feature_hole_map "
                        f"的 {len(hmap)} 项里哪些是头侧孔全集（沉孔/铣面条目不是另一颗螺丝），头判据不判",
                        provenance=f"fasteners.yaml:{fid}.head_locator_map_indices")
            if not F.get("tool_access"):
                res.unknown(fid, "head_seat",
                            f"没有 tool_access 坐面：{n_coords} 个 feature_hole_map 孔点是刀心不是坐面，头的位置无从谈起",
                            provenance=prov_side)
            continue
        heads, recv_skipped, skipped_only = [], [], 0
        for mi in locs:
            e = hmap[mi]
            feat_id = e.get("feature_id")
            fe = feats.get(feat_id) or {}
            part = fe.get("part")
            for hi, h in enumerate(e.get("holes") or []):
                # F-L5-1（2026-09-13）：接收孔（自攻底孔 / 舵机壳螺孔）上没有螺丝头 —— 与 l5_function._group_holes
                # 的 thru/recv 分类一致，kind=pilot_hole 一律跳过。
                if (fe.get("check_class") or fe.get("kind")) == "pilot_hole":
                    recv_skipped.append(f"{feat_id}@{part}#{mi}.{hi}")
                    continue
                if ctx.only and part not in ctx.only:
                    skipped_only += 1
                    continue
                try:
                    anchor = np.asarray(h, float)
                    if anchor.shape != (3,) or not np.isfinite(anchor).all():
                        anchor = None
                except (TypeError, ValueError):
                    anchor = None
                heads.append((feat_id, mi, hi, anchor, fe, part, e))
        if not heads:
            if skipped_only:
                continue
            if recv_skipped:
                res.unknown(fid, "head_seat",
                            f"head_locator_map_indices 指向的 {len(recv_skipped)} 个孔全是接收孔（pilot_hole：{recv_skipped[:3]}），"
                            f"没有穿件那一侧的孔坐标，头判据没地方落",
                            provenance=prov_map + " | features.yaml:kind")
            else:
                res.unknown(fid, "head_seat", "head_locator_map_indices 指向的 map 项没有 holes", provenance=prov_map)
            continue
        if not F.get("tool_access"):
            res.unknown(fid, "head_side",
                        f"{len(heads)} 个头侧孔没有 tool_access[].seats[]（坐面点/头侧方向未声明）—— 不取较差端，头判据不判",
                        provenance=prov_side + ".outward_export_local")
            res.unknown(fid, "head_seat",
                        f"没有 tool_access 坐面：feature_hole_map 的孔点是刀心（切刀中点，常在件外）不是坐面，不当起点",
                        provenance=prov_side + ".point_export_local")
            continue

        table = seat_table(F)
        group_inst = table["instances"]
        undeclared, conflicts, misaligned, off_axis, oob, seat_off = [], [], [], [], [], []
        holes = []
        axis_mismatch, n_axis = 0, 0
        for feat_id, mi, hi, anchor, fe, part, entry in heads:
            tag = f"{feat_id}#{mi}.{hi}"
            st = table["holes"].get((feat_id, mi, hi))
            missing = sorted(str(x) for x in (group_inst - (st["instances"] if st else set())))
            if st is None:
                undeclared.append(f"{tag}@{sorted(str(x) for x in group_inst)}")
                continue
            if missing:
                undeclared.append(f"{tag}@{missing}")
            if st["conflicts"]:
                conflicts.append(f"{tag}@{st['conflicts']}：同一 instance 同一孔给了不一致的 outward_export_local（冲突）")
            if missing or st["conflicts"]:
                continue
            if anchor is None:
                oob.append(f"{tag}: feature_hole_map 孔坐标不是有限三元组，轴线锚缺失")
                continue
            fg = hole_geom(fe, entry)
            fe_g = {**fe, "geom": fg}
            stl = ctx.stl(part) if part else None
            if stl is None:
                oob.append(f"{tag}@{part}: 取不到导出 STL")
                continue
            mesh = ctx.solid(stl)
            rel = str(Path(stl).relative_to(ROOT))
            if rel not in res.inputs:
                res.inputs.append(rel)
            hd, hd_ref = _hole_d(tol, fe_g, dnom)
            if hd is None:
                oob.append(f"{tag}: 取不到过孔名义直径，定不出头足印内半径")
                continue
            if hd / 2.0 >= head_r:
                oob.append(f"{tag}: 过孔 Ø{hd} ≥ 头径 Ø{head_d} —— 头根本坐不住（要么该配垫圈，要么孔径记错了）")
                continue
            ax_decl = _declared_axis(fg.get("axis"))
            radii = _ring_radii(hd / 2.0, head_r)
            for row in st["rows"]:
                pt, side = row["point"], row["outward"]
                delta = pt - anchor
                lat = float(np.linalg.norm(delta - side * float(delta @ side)))
                if lat > LATERAL_TOL:
                    off_axis.append(f"{tag}@{row['instances']}: 坐面点 {np.round(pt, 4).tolist()} 到所绑定孔轴（锚 "
                                    f"{np.round(anchor, 4).tolist()}）横向偏 {lat:.3e} > {LATERAL_TOL}")
                    continue
                lo_b, hi_b = mesh.bounds
                if np.any(pt < lo_b - 2) or np.any(pt > hi_b + 2):
                    oob.append(f"{tag}: 坐面点 {np.round(pt, 2).tolist()} 落在 {part} 导出包围盒外 "
                               f"({np.round(lo_b, 2).tolist()}..{np.round(hi_b, 2).tolist()})")
                    continue
                # 实测轴：起点在孔腔里（坐面点沿 −outward 进 0.5×孔径），不从刀心打
                ax_meas, _f1, _f2 = _measure_axis(mesh, pt - side * (0.5 * hd))
                rays_total += 6
                n_axis += 1
                if ax_decl is not None:
                    if abs(float(ax_meas @ ax_decl)) < 0.9:
                        axis_mismatch += 1
                    ax = ax_decl
                    if abs(float(side @ ax)) < 1 - 1e-6:
                        misaligned.append(f"{tag}: 声明头侧 {np.round(side, 3).tolist()} 与特征声明轴 "
                                          f"{np.round(ax, 3).tolist()}（features.yaml:{feat_id}.geom.axis）不共线")
                        continue
                else:
                    ax = ax_meas
                    if abs(float(side @ ax)) < 0.9:
                        misaligned.append(f"{tag}: 声明头侧 {np.round(side, 3).tolist()} 与实测孔轴 "
                                          f"{np.round(ax, 3).tolist()} 不共线（特征未声明 axis）")
                        continue
                sgn = 1.0 if float(side @ ax) > 0 else -1.0
                # 坐面点必须在料面上：足印环带在坐面头侧 1e-3 mm 处不能**全部**是料（与 tool_access.seat_samples 的
                # ±0.001 探针同口径；孔轴上的点本身在孔腔里，只能看环带）。全在料里 = 声明的坐面点在料里、真承压面在更
                # 外侧 → unknown，不许把"头高区内没有进料面"当成头悬空报出来；部分在料里是坐面不平，交给下面的极差判据
                ring_pts = np.vstack([_ring(pt + sgn * ax * 1e-3, ax, r) for r in radii])
                try:
                    inside = mesh.contains(ring_pts)
                except Exception as e:                                  # 非流形网格 contains 会炸
                    oob.append(f"{tag}: 坐面点是否在料里判不出（mesh.contains: {e}）")
                    continue
                if bool(inside.all()):
                    seat_off.append(f"{tag}@{row['instances']}: 坐面点 {np.round(pt, 4).tolist()} 头侧 1e-3 mm 处足印环带 "
                                    f"{len(ring_pts)}/{len(ring_pts)} 点全是料 —— 声明的坐面点在料里，不是料→空的承压面")
                    continue
                seat = _seat_end(mesh, pt, ax, sgn, radii, zone=head_h)
                rays_total += seat["tot"]
                if flat_max is not None and seat["med"] is not None and abs(seat["med"]) > float(flat_max) + NUM_EPS:
                    seat_off.append(f"{tag}: 坐面点到实测承压面（头高区内第一张进料面）中位偏 {seat['med']:.4f} mm "
                                    f"> {flat_max}（{FLAT_KEY}）—— 声明的坐面点不在料面上")
                    continue
                rr, nhit, ntot = seat["run_med"], seat["run_n"], seat["run_tot"]
                holes.append(dict(k=f"{mi}.{hi}", fid=feat_id, part=part, pt=pt, ax=ax, hd=hd, hd_ref=hd_ref,
                                  radii=radii, worst=seat, side=sgn, inst=list(row["instances"]),
                                  stack=(rr if (rr is not None and nhit >= ntot // 2) else None),
                                  area=float(math.pi * (head_r ** 2 - (hd / 2.0) ** 2))))
                holes_done += 1
                instances_covered += len(row["instances"])

        n_heads = len(heads)
        if undeclared:
            res.unknown(fid, "head_side",
                        f"{len(undeclared)}/{n_heads} 个头侧孔没有坐面：{prov_side} 里没有 feature_id+instance+map_index+hole_index "
                        f"对得上的条目（@ 后是缺坐面的 instance）—— 不取较差端，头判据不判：{undeclared[:3]}",
                        provenance=prov_side + ".outward_export_local")
        if conflicts:
            res.unknown(fid, "head_side",
                        f"{len(conflicts)} 个头侧孔在**同一 instance** 上给了不一致的头侧方向（冲突，不猜）：{conflicts[:2]}",
                        provenance=prov_side + ".outward_export_local")
        if misaligned:
            res.unknown(fid, "head_side",
                        f"{len(misaligned)} 个坐面声明的头侧方向与孔轴不共线，头判据不判：{misaligned[:2]}",
                        provenance=prov_side + ".outward_export_local | features.yaml:geom.axis")
        if off_axis:
            res.unknown(fid, "head_seat",
                        f"{len(off_axis)} 个坐面不在所绑定孔轴上（孔/坐面数据横向错位，与 tool_access.py 同一规则）：{off_axis[:2]}",
                        provenance=prov_side + ".point_export_local | " + prov_map)
        if seat_off:
            res.unknown(fid, "head_seat",
                        f"{len(seat_off)} 个坐面点不在实测承压面上：{seat_off[:2]}",
                        provenance=prov_side + ".point_export_local")
        if oob:
            res.unknown(fid, "head_seat", f"{len(oob)}/{n_heads} 个头侧孔摸不到：{oob[:3]}",
                        provenance=prov_map + " | " + prov_side)
        if undeclared or conflicts or misaligned or off_axis or seat_off or oob:
            # 任一头侧孔取不到坐面 → 该组 head_seat 判 unknown，不用部分孔的结果冒充全组（不猜）
            if not holes:
                pass
            elif not (off_axis or seat_off or oob):
                res.unknown(fid, "head_seat",
                            f"{len(holes)}/{n_heads} 个头侧孔有坐面、其余没有 —— 全组坐面不齐不判，"
                            f"不用一部分孔的结果冒充整组", provenance=prov_side)
            continue
        if not holes:
            continue
        skip_note = (f"｜跳过 {len(recv_skipped)} 个接收孔（pilot_hole，无头）：{recv_skipped[:3]}"
                     if recv_skipped else "｜跳过 0 个接收孔")
        inst_note = f"｜坐面覆盖 instance {sorted({str(i) for h in holes for i in h['inst']})}（左右实例同一份 STL，几何各量一次）"
        if axis_mismatch:
            res.add(subject=fid, check="hole_axis_declared_vs_measured", state=FAIL, severity=WARN,
                    measured=f"{axis_mismatch}/{n_axis} 个孔不符",
                    criterion="geom.axis 声明的轴要和从导出件测出来的孔轴一致（实测起点在孔腔里：坐面点沿 −outward 进 0.5×孔径）",
                    evidence_n=n_axis * 6,
                    detail="判据用的是声明轴（坐面 outward 与它共线）；声明值与实测不符说明 features.yaml 的 axis 或坐面数据有一处记错了",
                    provenance="G-A/A0 · features.yaml:geom.axis | " + prov_side)

        # ── A1 承压面在不在 + 有效承压面积（逐孔，全组每一个孔，不取统计量）──
        bad_seat = [h for h in holes if h["worst"]["miss"] > 0]
        wa = max(holes, key=lambda h: h["worst"]["miss"])
        res.add(subject=fid, check="head_seat_exists",
                state=PASS if not bad_seat else FAIL, severity=BLOCK,
                measured=f"{len(bad_seat)}/{len(holes)} 个孔有落空",
                criterion=f"每个**头侧**孔（head_locator_map_indices 指向、kind≠pilot_hole）的螺丝头足印环带（过孔壁 → Ø{head_d} 头沿，{N_RING} 圈 × "
                          f"{N_ANG} 向，从 tool_access 坐面点沿 outward 看，只认头高 {head_h} 以内的第一张进料面）"
                          f"**全部**要有承压料 —— 有效承压面积必须等于整圈环面积",
                evidence_n=sum(h["worst"]["tot"] for h in holes),
                detail=f"最差一孔 {wa['fid']}#{wa['k']}@{wa['part']}：{wa['worst']['miss']}/"
                       f"{wa['worst']['tot']} 个采样点落空 → 有效承压面积 "
                       f"{wa['area'] * (1 - wa['worst']['miss'] / max(1, wa['worst']['tot'])):.2f}"
                       f"/{wa['area']:.2f} mm²｜落空 = 螺丝头有一部分悬在空处" + skip_note + inst_note,
                provenance=f"G-A/A1 · {head_ref} | " + prov_side)
        for h in bad_seat[:6]:
            frac = 1 - h["worst"]["miss"] / max(1, h["worst"]["tot"])
            res.add(subject=fid, check=f"head_bearing_area:hole{h['k']}", state=FAIL, severity=BLOCK,
                    measured=round(h["area"] * frac, 3),
                    criterion=f"该孔的有效承压面积 = 整圈环面积 {h['area']:.2f} mm²"
                              f"（Ø{h['hd']} → Ø{head_d}）",
                    evidence_n=h["worst"]["tot"],
                    detail=f"{h['fid']}@{h['part']} 坐面 {np.round(h['pt'],2).tolist()} instance {h['inst']}："
                           f"{h['worst']['miss']}/{h['worst']['tot']} 点悬空",
                    provenance=f"G-A/A1 · {head_ref} | {h['hd_ref']}")

        # ── A2 承压面平不平（逐孔判，**不跨孔取统计量**）──────────────────
        if flat_max is None:
            res.unknown(fid, "head_seat_flatness",
                        f"tolerances.yaml:{FLAT_KEY}.max 取不到",
                        provenance=f"tolerances.yaml:{FLAT_KEY}")
        elif flat_src is None:
            res.unknown(fid, "head_seat_flatness",
                        f"阈值没有来源：tolerances.yaml:{FLAT_KEY} 没有 src（{flat_max} 不知道是实测/手册还是拍脑袋）—— F-L5-4 不判",
                        provenance=f"tolerances.yaml:{FLAT_KEY}")
        else:
            if flat_src == "assumed":
                assumed.note(FLAT_KEY, flat_max, f"{fid}:head_seat_flatness")
            bad_flat = [h for h in holes if h["worst"]["spread"] > float(flat_max) + NUM_EPS]
            wf = max(holes, key=lambda h: h["worst"]["spread"])
            res.add(subject=fid, check="head_seat_flatness",
                    state=PASS if not bad_flat else FAIL,
                    severity=BLOCK, measured=round(wf["worst"]["spread"], 4),
                    criterion=f"**每一个**头侧孔的头足印内承压面轴向极差 ≤ {flat_max} mm"
                              f"（{FLAT_KEY}，{src_label(flat_src)}）；"
                              f"起点 = fasteners.yaml:tool_access[].seats[].point_export_local，头侧 = outward_export_local"
                              f"（不取较差端，取不到坐面的孔判 unknown）；measured 是最差的那一个孔，不是全组的中位数",
                    evidence_n=sum(h["worst"]["n_hit"] for h in holes),
                    detail=f"{len(bad_flat)}/{len(holes)} 个孔不平；最差 {wf['fid']}#{wf['k']}@"
                           f"{wf['part']} 坐面 {np.round(wf['pt'],2).tolist()} = "
                           f"{wf['worst']['spread']:.4f}｜等厚斜面坐面的**料厚极差是 0**，"
                           f"只有这条（承压面轴向位置的极差）看得见" + inst_note,
                    provenance=f"G-A/A2 · tolerances.yaml:{FLAT_KEY} | " + prov_side)
            for h in bad_flat[:6]:
                res.add(subject=fid, check=f"head_seat_flatness:hole{h['k']}", state=FAIL,
                        severity=BLOCK, measured=round(h["worst"]["spread"], 4),
                        criterion=f"该孔自己的头足印内承压面轴向极差 ≤ {flat_max} mm（{src_label(flat_src)}）",
                        evidence_n=h["worst"]["n_hit"],
                        detail=f"{h['fid']}@{h['part']} 坐面 {np.round(h['pt'],2).tolist()} instance {h['inst']}；"
                               f"极差大 = 头只压住一边，预紧偏心",
                        provenance=f"G-A/A2 · tolerances.yaml:{FLAT_KEY}")

        # ── A3 沉孔配套：宽度、深度、以及"沉孔吃掉叠厚 → 咬入变深"的耦合 ──
        prot = None
        if cbf is not None:
            if cb_d is None or cb_dep is None:
                res.unknown(fid, "counterbore_geometry",
                            f"{cbf['id']} 是沉孔但 geom 缺直径或深度", provenance="features.yaml")
            elif cbb is not None and cb_lo is not None and cb_src is None:
                res.unknown(fid, "counterbore_diameter",
                            f"阈值没有来源：tolerances.yaml:{cb_key} 没有 src（下限 {cb_lo}）—— F-L5-4 不判",
                            provenance=f"tolerances.yaml:{cb_key}")
                prot = max(0.0, head_h - cb_dep)
            else:
                if cbb is not None and cb_lo is not None and cb_src == "assumed":
                    assumed.note(cb_key, cb_lo, f"{fid}:counterbore_diameter")
                need_d = max([x for x in (head_d, cb_lo) if isinstance(x, (int, float))])
                res.add(subject=fid, check="counterbore_diameter",
                        state=PASS if cb_d >= need_d else FAIL, severity=BLOCK,
                        measured=cb_d,
                        criterion=f"沉孔直径 ≥ 头径 {head_d}（{head_ref}）"
                                  + (f"，且 ≥ 公差表下限 {cb_lo}（{cb_key}，{src_label(cb_src)}）" if cb_lo is not None else
                                     "；公差表里没有该规格的沉孔桶，只按头径判"),
                        evidence_n=1, detail=f"沉孔 {cbf['id']}",
                        provenance="G-A/A3 · " + head_ref)
                res.add(subject=fid, check="counterbore_depth",
                        state=PASS if cb_dep >= head_h else FAIL, severity=WARN,
                        measured=cb_dep, criterion=f"沉孔深 ≥ 头高 {head_h}（{head_ref}）"
                                                   f"，否则头仍凸出，等于没沉",
                        evidence_n=1, provenance="G-A/A3 · " + head_ref)
                prot = max(0.0, head_h - cb_dep)
        else:
            prot = head_h
            res.add(subject=fid, check="counterbore_declared", state=PASS, severity=INFO,
                    measured="无沉孔", criterion="无沉孔时头全高凸出，凸出量要进装配刀路与运动包络",
                    evidence_n=1, detail=f"头凸出 {head_h} mm", provenance="G-A/A3 · " + head_ref)

        # ── A3b 叠厚对账 + 咬入：**逐孔**，不跨孔取中位数（FG15 的另一半）──
        meas = [h for h in holes if h["stack"] is not None]
        if not meas:
            res.unknown(fid, "stack_from_seat_crosscheck",
                        f"{len(holes)} 个孔一个都没量出连续料段，叠厚无从对账",
                        provenance="G-A/A3b")
        elif stack_rec is None:
            res.unknown(fid, "stack_from_seat_crosscheck",
                        f"fasteners.yaml 没记 stack_mm，无从对账（实测逐孔 "
                        f"{[round(h['stack'],3) for h in meas][:8]}）",
                        provenance="fasteners.yaml:stack_mm")
        elif stack_tol is None:
            res.unknown(fid, "stack_from_seat_crosscheck",
                        f"叠厚对账的容差取不到（找过 {stack_tol_ref}）—— 判据没有数据来源，"
                        f"不许在层里写死。实测逐孔 {[round(h['stack'],3) for h in meas][:8]}，"
                        f"清单记 {stack_rec}",
                        provenance=stack_tol_ref)
        elif stack_src is None:
            res.unknown(fid, "stack_from_seat_crosscheck",
                        f"阈值没有来源：{stack_tol_ref} 没有 src（±{stack_tol}）—— F-L5-4 不判。"
                        f"实测逐孔 {[round(h['stack'],3) for h in meas][:8]}，清单记 {stack_rec}",
                        provenance=stack_tol_ref)
        else:
            if stack_src == "assumed":
                assumed.note(stack_tol_ref.split(":", 1)[-1], stack_tol, f"{fid}:stack_from_seat_crosscheck")
            devs = [(h, abs(h["stack"] - float(stack_rec))) for h in meas]
            bad = [(h, d) for h, d in devs if d > stack_tol + NUM_EPS]
            wh, wd = max(devs, key=lambda x: x[1])
            res.add(subject=fid, check="stack_from_seat_crosscheck",
                    state=PASS if not bad else FAIL, severity=BLOCK,
                    measured=round(wh["stack"], 3),
                    criterion=f"**每一个头侧孔**（接收孔 pilot_hole 无头，不量）从 tool_access 坐面沿孔轴量到的叠厚（足印内圈 {N_ANG} 向，"
                              f"头高区内第一张进料面起的第一段连续实体，取中位数），与 fasteners.yaml 记的 "
                              f"{stack_rec}（{stack_src_rec}）之差 ≤ {stack_tol}"
                              f"（{stack_tol_ref}，{src_label(stack_src)}）；不跨孔取中位数",
                    evidence_n=len(meas) * N_ANG,
                    detail=f"{len(bad)}/{len(meas)} 个孔对不上；最差 {wh['fid']}#{wh['k']}@"
                           f"{wh['part']} 实测 {wh['stack']:.3f} 差 {wd:.3f}｜逐孔 "
                           f"{[round(h['stack'],3) for h in meas][:10]}。对不上说明记录的叠厚"
                           f"不是从真正的承压面量的 —— 沉孔一改、坐面一挪，咬入就变了而没人知道" + skip_note,
                    provenance="G-A/A3b · fasteners.yaml:stack_mm | " + stack_tol_ref)
            if L is not None and (emin is not None or emax is not None):
                engs = [(h, L - h["stack"]) for h in meas]
                bad_e = [(h, e) for h, e in engs
                         if (emin is not None and e < emin - NUM_EPS)
                         or (emax is not None and e > emax + NUM_EPS)]
                we, wev = min(engs, key=lambda x: x[1])
                res.add(subject=fid, check="engagement_from_measured_seat",
                        state=PASS if not bad_e else FAIL, severity=BLOCK,
                        measured=round(wev, 3),
                        criterion=f"**每一个孔**用它自己的实测叠厚算的咬入（{L} − 该孔叠厚）"
                                  f"要落在 [{emin}, {emax}]",
                        evidence_n=len(meas) * N_ANG,
                        detail=f"{len(bad_e)}/{len(engs)} 个孔超差；逐孔 "
                               f"{[round(e,3) for _h, e in engs][:10]}｜低于下限 = 拧不住，"
                               f"超上限 = 顶死：手感很紧，但头没压住件，一点夹紧力都没有",
                        provenance="G-A/A3b · tolerances.yaml:engagement_by_joint_type")

        if prot is not None:
            res.add(subject=fid, check="head_protrusion", state=PASS, severity=INFO,
                    measured=round(prot, 3), criterion="记录值，供 L4 刀路与 L6 运动包络使用",
                    evidence_n=1, provenance="G-A/A5")

        # A4 头实体 vs 其它已装件 —— 需要头侧方向；本模块只判到"有没有数据支撑"
        res.unknown(fid, "head_vs_neighbours",
                    "头实体与相邻件（含会动的）的干涉尚未实现：需要 fasteners.yaml 给出每颗螺丝的"
                    "头侧方向（drive_direction 现在是自由文本），以及 L6 的运动姿态",
                    provenance="G-A/A4", severity=WARN)

    assumed.emit(res, NAME)
    res.evidence = {"fastener_groups": len(fasteners), "holes_probed": holes_done,
                    "seat_instances_covered": instances_covered,
                    "rays": rays_total, "angles_per_hole": N_ANG, "rings_per_hole": N_RING}
    return res
