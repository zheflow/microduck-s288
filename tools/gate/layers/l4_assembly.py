#!/usr/bin/env python3
"""第 4 层 装得进去 —— "每个东西有路进到它的位置吗"。

判据（README 第 2 节 L4）：
  A 按 assembly.yaml 19 步逐步扫掠：移动件沿声明方向平移 len_mm，与已装件交集 ≤ 阈值
  B disassembly_order 8 条同样要通（拆不出来 = 装错了也救不回来）
  C 每颗螺丝必须声明 tool_access 真坐面与头侧方向，按装配工位在场全集检查。
    用 tool_envelope.bit_d_mm 建逐颗批杆，不用 channel_d_mm / channel_len_mm 代替工具。
  D 螺丝头沿刀轴可见（射线，独立于布尔的第二路证据）；实际可用杆长另查 available_shaft_len_mm。

历史教训（本层存在的理由）：踝子总成的装配顺序写在文档里，**从来没人模拟过**；一模拟发现
反向抽出峰值 372.6 mm³、六个方向全堵、74 条路径全失败，最后不得不拆出 L07 可拆后臂。
所以本层的每一步都必须真扫掠，`verified_by: 无` / `verified: false` 一律 res.unknown（元规则 4）。

坑：
  · 布尔必须用 process=True 的网格（ctx.solid）；manifold3d 对未合点的 STL 直接判 NotManifold。
  · 布尔失败抛异常 → 判红，绝不吞成"体积 0 = 通过"（照抄 duckstructure/checks.py:vol 的做法）。
  · **扫掠方向的正负号**：assembly.yaml 的 direction 字段在不同步里含义不一致 ——
    step 1 的 "+ex" 是**抽出**方向（+ex 峰值 ~0、−ex 峰值 2465），
    step 6 的 "世界 -z→+z" 是**装入**方向（−z 峰值 0、+z 峰值 2468）。
    数据没有声明哪个是哪个 → _axis_from_text 把符号**解析出来并交出去**（不再丢），
    判据两个符号都试（placed/ 是最终位姿，沿 +s 抽出 ⇔ 沿 −s 装入，都是"存在直线路径"的证据），
    声明的符号是否就是能走的那个，由 step_direction_declared 单独判。
  · 尚未迁移为 motions 的历史步骤继续诊断，本层用 `parts[0] = 移动件` 这条约定
    （与 12 步的 action 原文逐条核对过，见 _MOVER_RULE），其余 parts + already_installed 当不动件。

三条已修的假绿（docs/gate/体系审查_Codex01.md，反例在 tools/gate/negatives/n_l4_*.py）：
  · FG10 分组各自选方向 —— 曾把障碍分成 local（本步 parts）/ installed（already_installed）两组、
    **每组各自取较好的一侧符号**：移动件被本步件挡 +x、被已装件挡 −x 时两条判据都 0 mm³ 判绿，
    实际无路。现在判据只有一条 `step_sweep`：同一方向、同一符号、对**合并后的完整障碍集合**求交；
    两组的数字保留成 `step_sweep_local` / `step_sweep_installed` 诊断（severity=INFO，不是放行判据）。
  · FG11 零长度路径 / 拆卸失败仍假定已拆 —— 现在 len_mm ≤ 0、step_mm ≤ 0 直接 BLOCK；
    每条路径末端必须**证明**完全脱离（_proven_clear_after：沿 d 越过 或 ⊥d 影子分离）；
    拆卸序只有**本条判定成功**才把该件迁出障碍集合，失败/未知则后续条目全部挂 `disassembly_prereq`。
  · FG12：从经过足印验证的真坐面连续检查到拧紧工位外；移到远墙外的起点不能通过坐面验证。
    需求杆长与可用杆长分开。仍未声明真坐面的螺丝保持 unknown，不沿旧孔中点出虚假几何结论。

声明式弹性接触（hr41g 2026-09-24，反例 negatives/n_l4_elastic_*.py）：
  · 刚体扫掠里 PETG 卡扣越棱的设计过盈必然留下交集。**不豁免**：只有每个超阈交集都落在 pla_snap 组两侧特征
    bbox 之交（外扩 ELASTIC_ZONE_PAD_MM）里、≤ 过盈量×面积上限、且声明了 PETG + 过盈量、末端仍证明脱离时，
    该路径才由 BLOCK 降为 WARN（elastic_contact_declared："声明弹性接触，保持力/寿命待实测 BN19/BN20"），永不判 PASS；
    区外 / 超上限 / 声明不全 → 原 BLOCK 保留。只接入显式 motions / segments 路径（旧式 linear 步未接入）。
    拆卸序里按声明弹性拆出的件迁出障碍集合，但后续每条挂 disassembly_prereq_elastic（WARN），不静默。
"""
from __future__ import annotations
import math
import re
import sys
from pathlib import Path

_HERE = Path(__file__).resolve()
sys.path.insert(0, str(_HERE.parents[1]))                       # tools/gate → core
for _p in (str(_HERE.parents[3]), str(_HERE.parents[3] / "tools" / "cad")):
    if _p not in sys.path:
        sys.path.insert(0, _p)                                  # 仓库根（duckstructure）+ tools/cad（assembly_audit）

from core import (LayerResult, PASS, FAIL, NOT_RUN, BLOCK, WARN, INFO,      # noqa: E402
                  ROOT, PLACED, placed_instance_map, num)

LAYER = 4
NAME = "装得进去"

_TOL_KEY = "tolerances.yaml:feature_check_tolerances.static_intersection_mm3"
_MOVER_RULE = ("assembly.yaml 没有 movers 字段。本层取 parts[0] 为移动件，其余 parts + "
               "already_installed 为不动件 —— 这条约定与 12 步的 action 原文逐条对得上"
               "（step1 'L05 从法兰侧套到踝舵机'→L05；step3 '6700 …喂入拆下的 L07 座'→6700；"
               "step6 '髋横滚舵机自下装入 L01'→舵机；step11 'N03 先从下方装入 H01 半座'→N03 …）")

_NUM = r"[0-9]+(?:\.[0-9]+)?"
_AXV = {"x": (1.0, 0.0, 0.0), "y": (0.0, 1.0, 0.0), "z": (0.0, 0.0, 1.0)}


# ── 几何底座 ──────────────────────────────────────────────────────────────
class _Geo:
    """placed/ 世界坐标件 → manifold 实体。布尔走 assembly_audit（AABB 粗筛 + 失败抛异常）。"""

    def __init__(self, ctx):
        self.ctx = ctx
        self._solid, self._mesh, self._bnd = {}, {}, {}
        self._world = None
        self.booleans = 0
        self.index = {p.stem: p for p in sorted(PLACED.glob("*.stl"))} if PLACED.exists() else {}

    def mesh(self, stem):
        if stem not in self._mesh:
            self._mesh[stem] = self.ctx.solid(self.index[stem])   # process=True，布尔前提
        return self._mesh[stem]

    def bounds(self, stem):
        """该件的 AABB（(lo, hi) 两个 3 向量）。AABB 是网格的外包 —— 在 AABB 上能证明分离的，
        网格一定也分离；反过来不成立，所以只用它做**证明**，不用它下"相交"的结论。"""
        import numpy as np
        if stem not in self._bnd:
            b = self.mesh(stem).bounds
            self._bnd[stem] = (np.asarray(b[0], float), np.asarray(b[1], float))
        return self._bnd[stem]

    def world_bounds(self):
        """整机包围盒 —— 刀路"通到机外"的终点由它给出，不写死数字。"""
        import numpy as np
        if self._world is None:
            lo = np.array([float("inf")] * 3)
            hi = np.array([float("-inf")] * 3)
            for stem in self.index:
                b0, b1 = self.bounds(stem)
                lo = np.minimum(lo, b0)
                hi = np.maximum(hi, b1)
            self._world = (lo, hi)
        return self._world

    def solid(self, stem):
        if stem not in self._solid:
            from assembly_audit import solid
            self._solid[stem] = solid(self.mesh(stem))
        return self._solid[stem]

    @staticmethod
    def wrap(mesh):
        from assembly_audit import solid
        return solid(mesh)

    def inter(self, a, b):
        from assembly_audit import intersection
        self.booleans += 1
        return intersection(a, b)                                 # 布尔炸了抛 ValueError，不吞

    def digest(self, stem):
        """hr42：该件源网格（placed/ 读入、process=True）的内容摘要 —— 检查原语缓存的键。"""
        if not hasattr(self, "_dig"):
            self._dig = {}
        if stem not in self._dig:
            import check_cache as _CC
            self._dig[stem] = _CC.mesh_digest(self.mesh(stem))
        return self._dig[stem]

    def inter_swept(self, mover, off, tvec, ob, moved):
        """hr42：= self.inter(moved, self.solid(ob))，moved = solid(mover) 先平移 off 再平移 tvec（None = 没平移）。
        走检查原语缓存：键 = 两件源网格摘要 + 两段平移向量的原始字节 + 库版本；命中时照样计一次布尔（booleans_run 口径不变）。"""
        import check_cache as _CC
        if not _CC.enabled():
            return self.inter(moved, self.solid(ob))
        import numpy as np
        _CC.set_context(f"l4:{mover}×{ob}")
        k = _CC.key("l4.inter_swept", self.digest(mover),
                    b"" if off is None else np.asarray(off, float), b"" if tvec is None else np.asarray(tvec, float),
                    self.digest(ob))
        hit, v = _CC.get("l4", k)
        if hit:
            self.booleans += 1
            return v
        v = self.inter(moved, self.solid(ob))
        _CC.put("l4", k, v)
        return v


def _cyl(d, h, base, axis, sections=96):
    """从 base 沿 axis 长 h 的圆柱。**外接多边形**（顶点在圆外）：trimesh 的内接柱体积系统性偏小，
    方向恰好是漏报（真侵入被算成 0）—— 见 l3_static._cyl 的实测数字。"""
    import numpy as np
    import trimesh
    from trimesh.transformations import rotation_matrix as rot
    r = (d / 2.0) / math.cos(math.pi / sections)
    c = trimesh.creation.cylinder(radius=r, height=h, sections=sections)   # 轴 = +z，中心在原点
    a = np.asarray(axis, float)
    a = a / np.linalg.norm(a)
    z = np.array([0.0, 0.0, 1.0])
    v = np.cross(z, a)
    if np.linalg.norm(v) > 1e-12:
        c.apply_transform(rot(math.acos(float(np.clip(z @ a, -1, 1))), v))
    elif z @ a < 0:
        c.apply_transform(rot(math.pi, [1.0, 0.0, 0.0]))
    c.apply_translation(np.asarray(base, float) + a * (h / 2.0))
    return c


def _rel(p):
    try:
        return str(Path(p).relative_to(ROOT))
    except ValueError:
        return str(p)


# ── 名字 → placed 实体 ────────────────────────────────────────────────────
class _Names:
    """assembly.yaml 的 parts / already_installed 里写的是混合口径（件号 L05、元件 servo_ankle、
    轴承 6700、'6704×2'、'电池'、'腿子总成'…）。这里把能落地的解析成 placed 实体名，
    落不了地的返回 None 并由调用方记账 —— 不猜。"""

    def __init__(self, ctx, geo):
        self.ctx, self.geo = ctx, geo
        self.unresolved = {}
        self._cache = {}
        # 舵机：placed 名 = servo__<载体 body>_<drives>，由 MJCF 解析出来，不写死
        self.servo_by_joint = {}
        try:
            from duckstructure.lib import B, ORDER
            for n in ORDER:
                for s in B[n]["servos"]:
                    d = s.get("drives")
                    if not d:
                        continue
                    child = d.replace(":self", "")
                    j = (B.get(child) or {}).get("joint")
                    stem = f"servo__{n}_{child}"
                    if j and stem in geo.index:
                        self.servo_by_joint[j["name"]] = stem
        except Exception:                                            # noqa: BLE001
            pass
        # 轴承：按外径分型号（6700 = Ø15、6704 = Ø27），不写死件名
        self.bearing_by_od = {}
        for stem in geo.index:
            if not stem.startswith("bearing"):
                continue
            b = geo.mesh(stem).bounds
            od = round(float(max(b[1] - b[0])), 1)
            self.bearing_by_od.setdefault(od, []).append(stem)

    def part_stems(self, pid):
        """件号 → 已校验 BOM 数量及全局唯一性的 placed 实例（含左右）。"""
        rec = next((p for p in ((self.ctx.data.get("parts") or {}).get("parts") or [])
                    if p.get("id") == pid), None)
        if rec and "placed_instances" in rec:
            mapping, errors = placed_instance_map(self.ctx.data["parts"]["parts"], self.geo.index)
            return mapping.get(pid, []) if not errors else []
        cands = []
        p = self.ctx.placed_for(pid)
        if p is not None:
            cands.append(p.stem)
        if rec:
            inv = rec.get("inventory_id") or ""
            cands += [inv, inv.split("_", 1)[-1] if "_" in inv else inv]
            cands.append(re.sub(r"_(TPU|PLA)$", "", inv.split("_", 1)[-1] if "_" in inv else inv))
            for fn in (rec.get("build_fn") or []):
                cands.append(fn.replace("build_", ""))
        cands.append(pid)
        base = None
        for c in cands:
            if not c:
                continue
            if c in self.geo.index:
                base = c[:-2] if (c.endswith("_R") and c[:-2] in self.geo.index) else c
                break
        if base is None:
            return []
        return [base] + ([base + "_R"] if (base + "_R") in self.geo.index else [])

    def _bearing_od(self, tok):
        """'6704' / '6704×2' / '22×16×4' / '6700ZZ' → 外径（mm），按 components.yaml 轴承条目解析；认不出 → None。"""
        import re as _re
        if not hasattr(self, "_bearing_models"):
            self._bearing_models = {}
            for c in ((self.ctx.data.get("components") or {}).get("components") or []):
                if (c.get("category") or "").lower() != "bearing":
                    continue
                txt = " ".join(str(x) for x in (c.get("model"), ((c.get("envelope_mm") or {}).get("v")
                                                             if isinstance(c.get("envelope_mm"), dict) else None)) if x)
                m3 = _re.search(r"(\d+(?:\.\d+)?)\s*[×xX*]\s*(\d+(?:\.\d+)?)\s*[×xX*]\s*(\d+(?:\.\d+)?)", txt)
                if not m3:
                    continue
                tri = sorted(float(g) for g in m3.groups())
                od = round(tri[2], 1)                      # 三元组里最大的是外径（厚最小、内径居中）
                keys = [f"{tri[0]:g}×{tri[1]:g}×{tri[2]:g}"]
                # 型号代码只认 model 开头的那个（"6704ZZ 20×27×4"）；正文里提到的别的型号（D-COMP-01 那句
                # "原版把 16×22×4 错标成 6704ZZ"）不是这颗轴承的代码，不能拿来登记
                mm = _re.match(r"\s*(6\d{3})[A-Za-z]*\b", str(c.get("model") or ""))
                if mm:
                    keys.append(mm.group(1))
                for k in keys:
                    if k in self._bearing_models and self._bearing_models[k] != od:
                        self._bearing_models[k] = None        # 同一代码两种外径 → 说不清，不猜
                    else:
                        self._bearing_models.setdefault(k, od)
        t = str(tok).strip()
        m3 = _re.fullmatch(r"(\d+(?:\.\d+)?)\s*[×xX*]\s*(\d+(?:\.\d+)?)\s*[×xX*]\s*(\d+(?:\.\d+)?)", t)
        if m3:
            tri = sorted(float(g) for g in m3.groups())
            return self._bearing_models.get(f"{tri[0]:g}×{tri[1]:g}×{tri[2]:g}")
        m = _re.fullmatch(r"(6\d{3})[A-Za-z]*(?:\s*[×xX]\s*\d+)?", t)
        return self._bearing_models.get(m.group(1)) if m else None

    def _lookup(self, tok):
        import re as _re
        if self._bearing_od(tok) is not None and _re.search(r"\d\s*[×xX*]\s*\d+(?:\.\d+)?\s*[×xX*]", str(tok)):
            t = str(tok).strip()                       # 三元组 '22×16×4' 不能把末尾 ×4 当数量剥掉
        else:
            t = _re.sub(r"[×xX]\s*\d+\s*$", "", str(tok)).strip()
        if t in self.geo.index:
            return [t], "显式 placed 实例"
        # 1 打印件号
        if any(x.get("id") == t for x in ((self.ctx.data.get("parts") or {}).get("parts") or [])):
            out = self.part_stems(t)
            if out:
                return out, "parts.yaml 件号"
        # 2 舵机：servo_<关节后缀>
        if t.startswith("servo_"):
            suf = t[len("servo_"):]
            hits = [s for j, s in sorted(self.servo_by_joint.items())
                    if j == suf or j == "left_" + suf or j == "right_" + suf]
            if hits:
                return hits, "frozen.yaml:joint_axes → MJCF 舵机"
        # 3 轴承：型号名（6700/6702/6704…）或 "内径×外径×厚" 三元组 → 外径 → placed 里同外径的完整环。
        #   外径从 components.yaml:components[category=bearing].model / envelope_mm 解析（2026-09-13 起不再写死 15/27）。
        od = self._bearing_od(t)
        if od is not None:
            hits = sorted(self.bearing_by_od.get(od, []))
            if hits:
                return hits, f"placed 里外径 Ø{od} 的完整轴承环（components.yaml 型号→外径）"
        # 4 电池 / 原版件
        if t in ("电池",) and "zz_battery" in self.geo.index:
            return ["zz_battery"], "placed:zz_battery（电池本体尺寸）"
        if ("orig_" + t) in self.geo.index:
            return ["orig_" + t], "placed:orig_*（原版件）"
        return None, None

    def resolve(self, tok):
        if tok in self._cache:
            return self._cache[tok]
        hits, how = self._lookup(tok)
        if hits is None:
            self.unresolved[str(tok)] = self.unresolved.get(str(tok), 0) + 1
        self._cache[tok] = (hits or [], how)
        return self._cache[tok]


# ── 方向解析 ──────────────────────────────────────────────────────────────
def _axis_from_text(txt, servo_x=None):
    """把 direction / path.axis 这类自然语言解析成**带符号的世界轴**。
    返回 (轴向量, 符号, 说明) 或 (None, None, 原因)。

    FG11：这里以前只交出轴、把符号扔了，调用方连"data 声明的是哪一侧"都不知道。
    符号 = +1 / −1（原文只出现一个符号）；None = 原文自己就没定（'±y 侧滑'、'世界 -z→+z'
    这种两个符号都写了的，或者 'ex' 这种没写符号的）。判据用符号的方式见 judge_step_path。"""
    import numpy as np
    s = str(txt or "")
    if "ex" in s:
        if servo_x is None:
            return None, None, "方向写作 ex（舵机法兰轴），但本步的 parts/already_installed 里认不出唯一的舵机"
        m = re.search(r"([+-])\s*ex", s)
        sg = {"+": 1.0, "-": -1.0}.get(m.group(1)) if m else None
        return (np.asarray(servo_x, float), sg,
                f"ex = 该步舵机帧的 +x（duckstructure.lib.sfw，不是抄坐标）；原文符号 "
                + ("未写" if sg is None else ("+" if sg > 0 else "−")))
    hit = re.findall(r"([+-±])\s*([xyz])", s)
    if hit:
        axes = {a for _, a in hit}
        if len(axes) == 1:
            a = axes.pop()
            signs = {h[0] for h in hit}
            sg = (1.0 if signs == {"+"} else (-1.0 if signs == {"-"} else None))
            return (np.asarray(_AXV[a], float), sg,
                    f"世界 {a} 轴（原文 {s!r}，符号 "
                    + ("+" if sg == 1.0 else ("−" if sg == -1.0 else f"未定：原文写了 {sorted(signs)}")) + "）")
        return None, None, f"一条 direction 里出现了多个轴 {sorted(axes)}：{s!r}"
    if "轴向" in s:
        return None, None, "direction='轴向' —— 没说是哪个轴，压装轴向要由被压件的座轴给出，data 没给"
    return None, None, f"direction/axis 解析不出世界轴：{s!r}"


def _servo_x_for(step, names):
    """步骤上下文里唯一的舵机 → 它的舵机帧 +x（ex）。
    左右两颗是同一个关节的镜像（left_ankle / right_ankle），它们的 ex 互为 mirror_y；
    本层解析出来的 ex 是世界轴对齐的，mirror_y 对轴对齐向量等价于取负，而两个符号都会扫，
    所以取左侧那颗即可。真有两个**不同关节**的舵机就返回 None（不猜）。"""
    import numpy as np
    from duckstructure.lib import B, ORDER, sfw
    toks = [t for t in (list(step.get("parts") or []) + list(step.get("already_installed") or []))
            if str(t).startswith("servo_")]
    stems = set()
    for t in toks:
        stems.update(names.resolve(t)[0])
    if not stems:
        return None
    joints = {j for j, s in names.servo_by_joint.items() if s in stems}
    if len({re.sub(r"^(left|right)_", "", j) for j in joints}) != 1:
        return None
    stem = sorted(stems)[0]
    for n in ORDER:
        for i, s in enumerate(B[n]["servos"]):
            d = (s.get("drives") or "").replace(":self", "")
            if d and f"servo__{n}_{d}" == stem:
                return np.asarray(sfw(n, i)[:3, 0], float)
    return None


# ── 扫掠 ──────────────────────────────────────────────────────────────────
def _sweep1(geo, mover, obstacles, direction, length, step):
    """**单个**移动件沿 direction 平移 0..length，与 obstacles 的最大交集。
    返回 (峰值, 发生在几 mm, 障碍件, 布尔次数, 逐障碍峰值 {stem: (峰值, 几 mm)})。
    逐障碍那份是给"分组诊断"用的 —— 分组不再重扫一遍（省一半布尔），也不再当判据。
    布尔异常向上抛（不吞）。"""
    import numpy as np
    d = np.asarray(direction, float)
    d = d / np.linalg.norm(d)
    best = (0.0, 0.0, None)
    n = 0
    per = {ob: (0.0, 0.0) for ob in obstacles if ob != mover}
    sm = geo.solid(mover)
    for t in sorted(set(np.arange(0.0, length, step).tolist() + [float(length)])):
        moved = sm.translate(d * float(t)) if t else sm
        for ob in obstacles:
            if ob == mover:
                continue
            v = geo.inter_swept(mover, None, (d * float(t)) if t else None, ob, moved)      # hr42：原语缓存；= geo.inter(moved, geo.solid(ob))
            n += 1
            if v > per[ob][0]:
                per[ob] = (v, float(t))
            if v > best[0]:
                best = (v, float(t), ob)
    return best + (n, per)


def _sweep_step(geo, movers, obstacles, d, length, step):
    """左右两个实例的可行方向互为镜像（左件 +y 进、右件 −y 进），所以符号必须**逐件**取，
    不能先对所有移动件取 max 再对符号取 min —— 那会强迫两侧走同一个符号，凭空造出红格。
    返回 (峰值, 明细行, 布尔次数)；峰值 = 各移动件「两个符号里较好的那个」的最大值。

    ⚠ **这不是判据，本层也不再调用它**。留着是因为它就是 FG10 的旧口径本身：
    docs/gate/体系审查_Codex01.md:44 的复现是"调用实际 _sweep_step"，删掉那条复现就没法跑了。
    层里的分组数字改从 judge_step_path 的逐障碍记录里拆（见 run() 里的 _grp，不再重扫一遍）。
    判据见 judge_step_path：同一方向 + 同一符号 + 完整障碍集合。"""
    rows, n, peak = [], 0, 0.0
    for mv in movers:
        p = _sweep1(geo, mv, obstacles, d, length, step)
        m = _sweep1(geo, mv, obstacles, [-x for x in d], length, step)
        n += p[3] + m[3]
        win, sense = (p, "+") if p[0] <= m[0] else (m, "−")
        rows.append(dict(mover=mv, plus=p, minus=m, best=win[0], sense=sense,
                         at=win[1], blocker=win[2]))
        peak = max(peak, win[0])
    return peak, rows, n


# ── 路径合法性：起点、终点、长度 ──────────────────────────────────────────
def _disjoint(a, b):
    return a[1] < b[0] or b[1] < a[0]


def _perp_axes(d):
    """⊥d 的 4 条方向（u、w 和两条 45° 对角）。在其中任何一条上两者投影不重叠，
    就证明了"沿 d 平移永远碰不到"（分离轴限制在 ⊥d 的那一族里）。"""
    import numpy as np
    d = np.asarray(d, float)
    d = d / np.linalg.norm(d)
    ref = np.array([1.0, 0.0, 0.0]) if abs(float(d[0])) < 0.9 else np.array([0.0, 1.0, 0.0])
    u = np.cross(d, ref)
    u = u / np.linalg.norm(u)
    w = np.cross(d, u)
    r = math.sqrt(0.5)
    return [u, w, (u + w) * r, (u - w) * r]


def _span(box, v):
    """AABB 在方向 v 上的支撑区间。AABB 是网格的外包 —— 在 AABB 上分离即真分离，
    所以只拿它做**证明**，不拿它下"相交"的结论。"""
    import numpy as np
    lo, hi = box
    c = (lo + hi) / 2.0
    e = (hi - lo) / 2.0
    return float(c @ v) - float(np.abs(v) @ e), float(c @ v) + float(np.abs(v) @ e)


def _proven_clear_after(geo, mover, obstacles, d, t_end):
    """**证明**移动件走到 t_end 之后再沿 d 走下去不会碰到任何障碍（= 真的抽出来了 / 真的是从
    自由空间进来的）。两条证明各自充分：
      (a) 沿 d 已整体越过：mover_min·d ≥ ob_max·d —— 之后只会更远；
      (b) 某条 ⊥d 方向上投影不重叠 —— 沿 d 平移不改变该投影。
    两条都给不出 → 这条路径**没有证明移动件出得来**（FG11：终点必须完全脱离）。
    返回 (是否全部证明, 没证明的障碍, 检查次数)。"""
    import numpy as np
    d = np.asarray(d, float)
    d = d / np.linalg.norm(d)
    lo, hi = geo.bounds(mover)
    mb = (lo + d * float(t_end), hi + d * float(t_end))
    perp = _perp_axes(d)
    unproven, n = [], 0
    for ob in obstacles:
        if ob == mover:
            continue
        n += 1
        ob_b = geo.bounds(ob)
        if _span(mb, d)[0] >= _span(ob_b, d)[1]:
            continue
        if any(_disjoint(_span(mb, v), _span(ob_b, v)) for v in perp):
            continue
        unproven.append(ob)
    return (not unproven), unproven, n


def judge_step_path(geo, movers, obstacles, axis, length, step, tol):
    """**一条路径 = 一个方向 + 一个符号 + 一个完整的障碍集合。**（FG10 + FG11 的判据本体）

    合格（逐移动件各判各的 —— 左右镜像件的可行符号本来就相反）= 存在一个符号 s 使得
      ① 0..length 全程与**全部**障碍的交集 ≤ tol，且
      ② 走到 length 时已经**证明**完全脱离（_proven_clear_after）。
    length ≤ 0、step ≤ 0、障碍集合为空 → 一律不合格：0 mm³ 是"没查"，不是"通过"（元规则 2/4）。

    符号两个都试的理由：placed/ 是最终位姿，从最终位姿沿 +s 抽出 ⇔ 沿 −s 装入，两者都是
    "存在一条直线路径"的证据。assembly.yaml 声明的符号是不是能走的那个，由调用方用返回的
    per-sense 数字单独判（step_direction_declared），**不在这里偷偷放行**。"""
    import numpy as np
    a = np.asarray(axis, float)
    a = a / np.linalg.norm(a)
    out = dict(ok=False, peak=0.0, evidence_n=0, nsamp=0, rows=[], reason=None,
               n_obstacles=len(obstacles), length=float(length), step=float(step))
    if not (float(length) > 0.0):
        out["reason"] = (f"path.len_mm={length} —— 长度 ≤ 0 的路径什么都没证明（交集必然 0，"
                         f"移动件根本没动过）")
        return out
    if not (float(step) > 0.0):
        out["reason"] = f"path.step_mm={step} —— 步长 ≤ 0，采样不出任何中间位姿"
        return out
    if not movers:
        out["reason"] = "没有可解析的移动件 —— 没有移动件就没有路径可判（元规则 2）"
        return out
    obstacles = sorted(set(obstacles) - set(movers))
    if not obstacles:
        out["reason"] = "合并后的障碍集合为空 —— 扫出来必然是 0，那是没查不是通过（元规则 2）"
        return out
    out["nsamp"] = len(np.unique(np.r_[np.arange(0.0, float(length), float(step)), float(length)]))
    ok_all = True
    for mv in movers:
        senses = {}
        for s in (1.0, -1.0):
            pk, at, blk, n, per = _sweep1(geo, mv, obstacles, a * s, float(length), float(step))
            proven, unproven, nc = _proven_clear_after(geo, mv, obstacles, a * s, float(length))
            senses[s] = dict(peak=pk, at=at, blocker=blk, n=n, per=per,
                             proven=proven, unproven=unproven, nc=nc,
                             ok=(pk <= tol and proven))
            out["evidence_n"] += n + nc
        good = [s for s in (1.0, -1.0) if senses[s]["ok"]]
        pick = good[0] if good else min((1.0, -1.0), key=lambda s: (senses[s]["peak"], not senses[s]["proven"]))
        w = senses[pick]
        row = dict(mover=mv, sense=("+" if pick > 0 else "−"), ok=bool(good),
                   peak=w["peak"], at=w["at"], blocker=w["blocker"],
                   proven=w["proven"], unproven=w["unproven"],
                   plus=senses[1.0], minus=senses[-1.0],
                   good_senses=["+" if s > 0 else "−" for s in good])
        out["rows"].append(row)
        out["peak"] = max(out["peak"], w["peak"])
        ok_all = ok_all and bool(good)
    out["ok"] = ok_all
    if not ok_all and out["reason"] is None:
        bad = [r for r in out["rows"] if not r["ok"]]
        out["reason"] = "；".join(
            f"{r['mover']}：两个符号都不成立"
            + (f"（较好的一侧 {r['sense']} 峰值 {r['peak']:.6f} mm³ > {tol}）" if r["peak"] > tol else "")
            + (f"（走完 {length} mm 仍未证明脱离 {r['unproven'][:3]}"
               + ("…" if len(r["unproven"]) > 3 else "") + "）" if r["unproven"] else "")
            for r in bad)
    return out


# ── 螺丝孔坐标 export_local → 世界 ────────────────────────────────────────
def _segments_of(motion):
    """一条声明动作 = 一段或多段**首尾相接的直线**（segments），每段自己的世界方向与长度。
    单段写法 outward_world/len_mm 仍然有效（= 一段）。多段是给"先退回内腔、再从腔口出来"这种
    刚性件用的（09-12 ④ 的 N04 轴颈销：从头偏航舵机腔沿 +x 插入，直线抽回腔内 12 mm 后必须
    改沿 −z 出腔口才算真的脱离）。段与段之间不做旋转 —— 需要旋转的动作本判据仍然判不了。"""
    import numpy as np
    segs = motion.get("segments")
    if segs is None:
        segs = [dict(outward_world=motion.get("outward_world"), len_mm=motion.get("len_mm"))]
    if not isinstance(segs, list) or not segs:
        raise ValueError("segments 必须是非空列表")
    out = []
    for k, sg in enumerate(segs):
        axis = np.asarray((sg or {}).get("outward_world"), float)
        length = float((sg or {}).get("len_mm") if (sg or {}).get("len_mm") is not None else "nan")
        if axis.shape != (3,) or not np.isfinite(axis).all() or np.linalg.norm(axis) <= 0:
            raise ValueError(f"第 {k + 1} 段方向须为有限非零三维向量")
        if not np.isfinite(length) or length <= 0:
            raise ValueError(f"第 {k + 1} 段长度须为有限正数")
        out.append((axis / np.linalg.norm(axis), length))
    return out


def _sweep1_from(geo, mover, obstacles, offset, direction, length, step, over=None, sink=None):
    """同 _sweep1，但移动件先平移 offset（前几段走过的位移）再沿 direction 扫。
    over/sink（hr41g）：交集 > over 的每个采样记进 sink（障碍, 位移向量, 本段 t, 体积），
    给声明式弹性接触分类用；不改峰值、不改判据。"""
    import numpy as np
    d = np.asarray(direction, float)
    d = d / np.linalg.norm(d)
    off = np.asarray(offset, float)
    best = (0.0, 0.0, None)
    n = 0
    per = {ob: (0.0, 0.0) for ob in obstacles if ob != mover}
    sm = geo.solid(mover)
    base = sm.translate(off) if np.any(off) else sm
    for t in sorted(set(np.arange(0.0, length, step).tolist() + [float(length)])):
        moved = base.translate(d * float(t)) if t else base
        for ob in obstacles:
            if ob == mover:
                continue
            v = geo.inter_swept(mover, off if np.any(off) else None, (d * float(t)) if t else None, ob, moved)   # hr42：原语缓存
            n += 1
            if sink is not None and over is not None and v > over:
                sink.append((ob, off + d * float(t), float(t), v))
            if v > per[ob][0]:
                per[ob] = (v, float(t))
            if v > best[0]:
                best = (v, float(t), ob)
    return best + (n, per)


def _proven_clear_at(geo, mover, obstacles, offset, d):
    """同 _proven_clear_after，但终点位移是任意向量 offset，"再走下去"的方向是最后一段的 d。"""
    import numpy as np
    d = np.asarray(d, float)
    d = d / np.linalg.norm(d)
    off = np.asarray(offset, float)
    lo, hi = geo.bounds(mover)
    mb = (lo + off, hi + off)
    perp = _perp_axes(d)
    unproven, n = [], 0
    for ob in obstacles:
        if ob == mover:
            continue
        n += 1
        ob_b = geo.bounds(ob)
        if _span(mb, d)[0] >= _span(ob_b, d)[1]:
            continue
        if any(_disjoint(_span(mb, v), _span(ob_b, v)) for v in perp):
            continue
        unproven.append(ob)
    return (not unproven), unproven, n


# ── 声明式弹性接触（hr41g 2026-09-24，反例 negatives/n_l4_elastic_*.py）────────────────────────
#   刚体扫掠里，PETG 卡扣越棱时的设计过盈必然留下交集（B03 顶盖开盖：两前舌凸点各 0.65 mm³ @dx=−1.25）。
#   这**不是豁免**：只有当**每一个**超阈交集都能被数据里的弹性卡扣声明完整解释时，才把该路径从 BLOCK 降为
#   WARN，而且永远不判 PASS（保持力/寿命只有实测能证，bench.yaml BN19/BN20）。解释条件（缺一条 = 原 BLOCK 保留）：
#     ① 声明组：fasteners.yaml 里 joint_type ∈ ELASTIC_JOINT_TYPES 的连接组，其 feature_ids 同时含"移动件所属件"
#        与"障碍件所属件"的特征（现役 F31/F32 的登记形式；件号由 parts.yaml:placed_instances 反查）；
#     ② 声明区：同一组里"移动件特征 bbox（随当前位移平移）∩ 障碍件特征 bbox"，两边各外扩 ELASTIC_ZONE_PAD_MM；
#        交集减去声明区后剩下的体积 ≤ static_intersection_mm3（区外 = 普通刚体判据，阈值同一个，不另设）；
#     ③ 声明量：该区两侧特征**至少一侧**有 features.yaml:<特征>.elastic_contact，其中
#        material ∈ ELASTIC_MATERIALS 且与 parts.yaml 该件 material 一致、interference_mm = {v>0, src 非空}；
#        两侧都声明且过盈量不同 → 说不清（unknown）；
#     ④ 上限：落在每个区里的交集体积 ≤ δ × A。A = 声明的 elastic_contact.contact_area_mm2；缺则取该区"核心盒"
#        （两特征 bbox 之交，不外扩）三个面积里最大的那个 —— 过盈沿轴向时接触投影面积的上界，只会偏宽不会偏严；
#     ⑤ 路径末端仍须证明完全脱离（_proven_clear_at）：弹性不能替代"真的出来了"。
#   只接入显式 motions / segments 路径（judge_fixed_path）；旧式 linear 步（judge_step_path）未接入。
ELASTIC_JOINT_TYPES = ("pla_snap",)        # fasteners.yaml 现有卡扣连接类型键（l5_screwhead.NOT_A_SCREW 同名），不新造
ELASTIC_MATERIALS = ("PETG",)              # 本项目只把 PETG 卡扣当可挠件；PLA 脆、TPU 是另一种机理，都不走这条
# 声明区外扩（mm）。实测 B03×两壳在 dx=−1.25 的交集 bbox 与名义特征 bbox 只差 ≤1e-5 mm（float32 STL 在 ~160 mm
# 处的量化）；0.01 是它的 1000 倍，又远小于过盈 0.25 与 0.4 线宽。外扩只放大"包含"判定，不放大 ④ 的上限（核心盒不外扩）。
ELASTIC_ZONE_PAD_MM = 0.01
ELASTIC_TEXT = "声明弹性接触，保持力/寿命待实测 BN19/BN20"
_ELASTIC_EPS_MM3 = 1e-9                    # 某区里落了多少交集才算"该区参与了解释"（布尔噪声底，不是判据阈值）


def _elastic_str(x):
    if isinstance(x, dict):
        x = x.get("v")
    return x.strip() if isinstance(x, str) and x.strip() else None


def _box_lohi(bb):
    """[[lo],[hi]] 或 {lo:[…], hi:[…]} → (lo, hi)；读法同 l2_features._bbox。"""
    import numpy as np
    if isinstance(bb, dict):
        bb = (bb.get("lo"), bb.get("hi"))
    if not (isinstance(bb, (list, tuple)) and len(bb) == 2):
        return None
    try:
        lo, hi = np.asarray(bb[0], float), np.asarray(bb[1], float)
    except (TypeError, ValueError):
        return None
    if lo.shape != (3,) or hi.shape != (3,) or not (np.isfinite(lo).all() and np.isfinite(hi).all()):
        return None
    return np.minimum(lo, hi), np.maximum(lo, hi)


def _prism_corners(pz):
    """prism（L2 约定：点 = origin + u·right + v·up − normal·s，s∈[0, depth]）→ 8 个角点（面内取多边形外包矩形）。"""
    import numpy as np
    try:
        o, r, up, n = (np.asarray(pz.get(k), float) for k in ("origin", "right", "up", "normal"))
        d = pz.get("depth_mm")
        d = float(d.get("v") if isinstance(d, dict) else d)
    except (TypeError, ValueError, AttributeError):
        return None
    if any(a.shape != (3,) or not np.isfinite(a).all() or np.linalg.norm(a) <= 0 for a in (r, up, n)) \
            or o.shape != (3,) or not np.isfinite(o).all() or not (d > 0):
        return None
    uv = []
    for item in pz.get("polygons") or []:
        ext = item.get("exterior") if isinstance(item, dict) else item
        for q in (ext or []):
            try:
                uv.append((float(q[0]), float(q[1])))
            except (TypeError, ValueError, IndexError):
                return None
    if len(uv) < 3:
        return None
    uv = np.asarray(uv)
    r, up, n = r / np.linalg.norm(r), up / np.linalg.norm(up), n / np.linalg.norm(n)
    return np.array([o + a * r + b * up - n * s
                     for a in (uv[:, 0].min(), uv[:, 0].max())
                     for b in (uv[:, 1].min(), uv[:, 1].max()) for s in (0.0, d)])


def feature_world_boxes(feat, frames):
    """特征 geom → 世界 AABB 列表（每个实例一个）+ 说明。
    认 BBOX_KEYS（与 L2 同一组键）与 prism；其它形状（curved_shell、仅代表点、box_clipped…）没有可用的盒 →
    不进声明区（只会更严）。export_local → 世界按 features.yaml:frames.per_part.<件>.world_to_export_local_R/t 逆变换，
    旋转时取 8 角点外包（只会更大；仅用于"包含"判定，上限另按核心盒算）。"""
    import numpy as np
    from layers.l2_features import BBOX_KEYS
    part = feat.get("part")
    f = frames.get(part) if isinstance(frames, dict) else None
    try:
        R = np.asarray(f["world_to_export_local_R"], float)
        t = np.asarray(f["world_to_export_local_t"], float)
        assert R.shape == (3, 3) and t.shape == (3,)
    except Exception:                                              # noqa: BLE001
        return [], f"{feat.get('id')}: features.yaml:frames.per_part.{part} 缺/坏，登记坐标落不到世界"
    gd = feat.get("geom") or {}
    geo_keys = ("prism",) + tuple(BBOX_KEYS)
    base = {k: v for k, v in gd.items() if k != "instances"}
    insts = gd.get("instances")
    raw = []
    if isinstance(insts, list) and insts:
        for it in insts:
            if not isinstance(it, dict):
                continue
            own = any(it.get(k) for k in geo_keys)
            m = {k: v for k, v in base.items() if not (own and k in geo_keys)}
            m.update({k: v for k, v in it.items() if v is not None})
            raw.append(m)
    else:
        raw.append(base)
    boxes, skipped = [], 0
    for m in raw:
        pts = None
        if isinstance(m.get("prism"), dict):
            pts = _prism_corners(m["prism"])
        else:
            lohi = _box_lohi(next((m.get(k) for k in BBOX_KEYS if m.get(k)), None))
            if lohi is not None:
                lo, hi = lohi
                pts = np.array([[x, y, z] for x in (lo[0], hi[0]) for y in (lo[1], hi[1]) for z in (lo[2], hi[2])])
        if pts is None:
            skipped += 1
            continue
        w = (R.T @ (pts - t).T).T
        boxes.append((w.min(0), w.max(0)))
    why = f"{feat.get('id')}: {len(boxes)} 个实例盒" + (f"，{skipped} 个实例没有 bbox/prism（不进声明区）" if skipped else "")
    return boxes, why


def elastic_decl(feat, parts_by_id):
    """features.yaml:<特征>.elastic_contact → dict(ok, delta, src, material, area, problems)。字段缺/坏逐条点名。"""
    fid, part = feat.get("id"), feat.get("part")
    ec = feat.get("elastic_contact")
    if not isinstance(ec, dict):
        return dict(ok=False, problems=[f"{fid} 缺 elastic_contact（至少一侧特征需声明 material 与 interference_mm）"])
    probs = []
    mat = _elastic_str(ec.get("material"))
    if mat is None:
        probs.append(f"{fid} 缺 elastic_contact.material")
    elif mat not in ELASTIC_MATERIALS:
        probs.append(f"{fid} elastic_contact.material={mat!r} 不在 {list(ELASTIC_MATERIALS)}")
    pmat = _elastic_str((parts_by_id.get(part) or {}).get("material"))
    if mat is not None and pmat is not None and pmat != mat:
        probs.append(f"{fid} elastic_contact.material={mat!r} 与 parts.yaml:{part}.material={pmat!r} 不一致")
    node = ec.get("interference_mm")
    delta, _src = num(node)
    if node is None:
        probs.append(f"{fid} 缺 elastic_contact.interference_mm")
    elif delta is None or not math.isfinite(float(delta)) or float(delta) <= 0:
        probs.append(f"{fid} elastic_contact.interference_mm.v={(node.get('v') if isinstance(node, dict) else node)!r} 不是有限正数")
    elif not (isinstance(node, dict) and node.get("src")):
        probs.append(f"{fid} elastic_contact.interference_mm 缺 src（阈值要有来源，README 4b）")
    area = None
    anode = ec.get("contact_area_mm2")
    if anode is not None:
        a, _ = num(anode)
        if a is None or not math.isfinite(float(a)) or float(a) <= 0:
            probs.append(f"{fid} elastic_contact.contact_area_mm2 不是有限正数")
        else:
            area = float(a)
    return dict(ok=not probs, delta=(float(delta) if not probs else None),
                src=(node.get("src") if isinstance(node, dict) else None), material=mat, area=area, problems=probs)


class _ElasticIndex:
    """数据侧索引（每个 _Geo 建一次）：实体 → 件号、特征、帧、卡扣组。"""

    def __init__(self, geo):
        data = geo.ctx.data
        parts = (data.get("parts") or {}).get("parts") or []
        self.part_of = {}
        for p in parts:
            for s in (p.get("placed_instances") or []):
                self.part_of[s] = p.get("id") if s not in self.part_of else None      # 一个实体两个件号 = 说不清
        self.parts_by_id = {p.get("id"): p for p in parts if isinstance(p, dict)}
        feats = (data.get("features") or {}).get("features") or []
        self.feat = {f.get("id"): f for f in feats if isinstance(f, dict)}
        self.frames = ((data.get("features") or {}).get("frames") or {}).get("per_part") or {}
        self.groups = [g for g in ((data.get("fasteners") or {}).get("fasteners") or [])
                       if isinstance(g, dict) and g.get("joint_type") in ELASTIC_JOINT_TYPES]
        self._boxes, self._decl = {}, {}

    def boxes(self, fid):
        if fid not in self._boxes:
            self._boxes[fid] = feature_world_boxes(self.feat[fid], self.frames)
        return self._boxes[fid]

    def decl(self, fid):
        if fid not in self._decl:
            self._decl[fid] = elastic_decl(self.feat[fid], self.parts_by_id)
        return self._decl[fid]

    def pairs(self, pm, po):
        """[(组 id, 移动件侧特征 id, 障碍件侧特征 id)]，同一组内配对；feature_ids 指向不存在的特征 → 进 missing。"""
        out, missing = [], []
        for g in self.groups:
            ids = [i for i in (g.get("feature_ids") or []) if isinstance(i, str)]
            missing += [f"{g.get('id')}.feature_ids→{i}" for i in ids if i not in self.feat]
            fa = [i for i in ids if i in self.feat and self.feat[i].get("part") == pm]
            fb = [i for i in ids if i in self.feat and self.feat[i].get("part") == po]
            out += [(g.get("id"), a, b) for a in fa for b in fb]
        return out, missing


def classify_elastic(geo, contacts, tol):
    """超阈交集采样 → 逐个判"是否被声明弹性接触完整解释"。contacts: [(移动件, 障碍, 位移, 路程, 体积)]。
    返回 dict(applicable, ok, rows)。status ∈ ok / outside / over_cap / undeclared / no_group。布尔异常向上抛（不吞）。"""
    import numpy as np
    import manifold3d as Mf
    idx = getattr(geo, "_elastic_idx", None)
    if idx is None:
        idx = geo._elastic_idx = _ElasticIndex(geo)
    pad = float(ELASTIC_ZONE_PAD_MM)

    def cube(lo, hi):
        return Mf.Manifold.cube([float(x) for x in (hi - lo)]).translate([float(x) for x in lo])

    rows = []
    for mv, ob, off, at, v in contacts:
        off = np.asarray(off, float)
        pm, po = idx.part_of.get(mv), idx.part_of.get(ob)
        prs, missing = idx.pairs(pm, po) if (pm and po) else ([], [])
        base = dict(mover=mv, obstacle=ob, at=round(float(at), 6), v=float(v), offset=[float(x) for x in off])
        if not prs:
            rows.append(dict(base, status="no_group", out=float(v), problems=[],
                             txt=f"{mv}({pm})×{ob}({po}) @{at:.2f} mm：{v:.6g} mm³，这对件不在任何 "
                                 f"{list(ELASTIC_JOINT_TYPES)} 组的 feature_ids 两侧（= 声明区外）"))
            continue
        zones, box_notes = [], []
        for gid, fa, fb in prs:
            ba, na = idx.boxes(fa)
            bb, nb = idx.boxes(fb)
            box_notes += [na, nb]
            for alo, ahi in ba:
                alo2, ahi2 = alo + off, ahi + off
                for blo, bhi in bb:
                    plo, phi = np.maximum(alo2 - pad, blo - pad), np.minimum(ahi2 + pad, bhi + pad)
                    if np.any(phi <= plo):
                        continue
                    clo, chi = np.maximum(alo2, blo), np.minimum(ahi2, bhi)
                    ext = np.clip(chi - clo, 0.0, None)
                    zones.append(dict(gid=gid, fa=fa, fb=fb, plo=plo, phi=phi,
                                      core_face=float(max(ext[0] * ext[1], ext[1] * ext[2], ext[0] * ext[2]))))
        hit = geo.solid(mv).translate(off) ^ geo.solid(ob)
        if hit.status() != Mf.Error.NoError:
            raise ValueError(f"弹性分类布尔失败: {hit.status()}")
        geo.booleans += 1
        if zones:
            Z = cube(zones[0]["plo"], zones[0]["phi"])
            for z in zones[1:]:
                Z = Z + cube(z["plo"], z["phi"])
            rest = hit - Z
            if rest.status() != Mf.Error.NoError:
                raise ValueError(f"弹性分类布尔失败: {rest.status()}")
            out_v = max(0.0, float(rest.volume()))
        else:
            out_v = max(0.0, float(hit.volume()))
        head = f"{mv}×{ob} @{at:.2f} mm：{v:.6g} mm³"
        if out_v > tol:
            rows.append(dict(base, status="outside", out=out_v, problems=[],
                             txt=head + f"，声明区外 {out_v:.6g} mm³ > {tol}（区 = 同组两侧特征 bbox 之交外扩 "
                                        f"{pad} mm，{len(zones)} 个；{'；'.join(sorted(set(box_notes)))}）"
                                 + (f"；登记缺失 {missing}" if missing else "")))
            continue
        probs, over, parts_txt = [], [], []
        for z in zones:
            vk = max(0.0, float((hit ^ cube(z["plo"], z["phi"])).volume()))
            geo.booleans += 1
            if vk <= _ELASTIC_EPS_MM3:
                continue
            da, db = idx.decl(z["fa"]), idx.decl(z["fb"])
            if da["ok"] and db["ok"] and abs(da["delta"] - db["delta"]) > 1e-9:
                probs.append(f"{z['fa']} 与 {z['fb']} 都声明过盈量但不同（{da['delta']} vs {db['delta']}）")
                continue
            d = da if da["ok"] else (db if db["ok"] else None)
            if d is None:
                probs += da["problems"] + db["problems"]
                continue
            area = d["area"] if d["area"] is not None else z["core_face"]
            cap = d["delta"] * area
            s = (f"{z['gid']}:{z['fa']}∩{z['fb']} 区内 {vk:.6g} mm³ / 上限 {cap:.6g}（过盈 {d['delta']:g} mm[src={d['src']}] × "
                 f"A {area:.4g} mm²，A={'声明 contact_area_mm2' if d['area'] is not None else '核心盒最大面面积'}）")
            if vk > cap:
                over.append(vk)
                s = s.replace("区内", "区内超上限", 1)
            parts_txt.append(s)
        if probs:
            rows.append(dict(base, status="undeclared", out=out_v, problems=sorted(set(probs)),
                             txt=head + f"，在声明区内（区外 {out_v:.3g}），但声明不全"))
        elif over:
            rows.append(dict(base, status="over_cap", out=out_v, over_v=max(over), problems=[],
                             txt=head + f"，超上限：" + "；".join(parts_txt)))
        elif not parts_txt:
            rows.append(dict(base, status="outside", out=max(out_v, float(v)), problems=[],
                             txt=head + "，没有任何声明区真正含到这块交集"))
        else:
            rows.append(dict(base, status="ok", out=out_v, problems=[],
                             txt=head + f"，区外 {out_v:.3g} ≤ {tol}；" + "；".join(parts_txt)))
    applicable = any(r["status"] != "no_group" for r in rows)
    return dict(applicable=applicable, ok=bool(rows) and all(r["status"] == "ok" for r in rows), rows=rows)


def emit_elastic(res, subject, check, el, tol, prov=""):
    """把 classify_elastic 的结论发成一条 finding：全部解释得通 → FAIL(WARN)（不判 PASS）；否则 FAIL(BLOCK)。
    measured：WARN = 最大单采样交集；BLOCK = 最大区外体积 → 最大超上限区内体积 → 声明不全时 None（unknown）。"""
    from collections import Counter
    rows = el["rows"]
    outs = [r["out"] for r in rows if r["status"] in ("outside", "no_group")]
    overs = [r["over_v"] for r in rows if r["status"] == "over_cap"]
    if el["ok"]:
        measured = round(max(r["v"] for r in rows), 8)
    elif outs:
        measured = round(max(outs), 8)
    elif overs:
        measured = round(max(overs), 8)
    else:
        measured = None
    shown = sorted(rows, key=lambda r: (r["status"] == "ok", -r["v"]))[:6]
    probs = sorted({p for r in rows for p in r.get("problems", [])})
    cnt = dict(Counter(r["status"] for r in rows))
    detail = ((ELASTIC_TEXT + "（刚体交集超阈，但每个超阈采样都落在声明弹性区内且 ≤ 过盈量×面积上限；不判 PASS）")
              if el["ok"] else "声明弹性接触**不成立** → 刚体 BLOCK 保留")
    detail += f"；超阈采样 {len(rows)} 个 {cnt}；" + "；".join(r["txt"] for r in shown) + ("…" if len(rows) > 6 else "")
    if probs:
        detail += "；声明缺陷（features.yaml:<特征>.elastic_contact）：" + "；".join(probs)
    if el.get("unproven"):
        detail += f"；末端未证明完全脱离：{el['unproven']}（弹性不能替代脱离证明）"
    res.add(subject=subject, check=check, state=FAIL, severity=WARN if el["ok"] else BLOCK, measured=measured,
            criterion=(f"超阈（> {tol} mm³）交集须全部落在同一 {list(ELASTIC_JOINT_TYPES)} 组内『移动件特征 bbox（随位移）∩ "
                       f"障碍件特征 bbox』外扩 {ELASTIC_ZONE_PAD_MM} mm 的声明区里（区外 ≤ {tol} mm³），每区体积 ≤ 过盈量 × "
                       f"接触面积上限（声明 contact_area_mm2，缺则核心盒最大面面积），且至少一侧特征声明 "
                       f"elastic_contact.material ∈ {list(ELASTIC_MATERIALS)}（与 parts.yaml 一致）与 interference_mm（带 src），"
                       f"末端证明脱离 → WARN（{ELASTIC_TEXT}，不判 PASS）；否则 BLOCK"),
            evidence_n=len(rows), detail=detail,
            provenance=(prov + " | fasteners.yaml:<pla_snap 组>.feature_ids / features.yaml:<特征>.geom + elastic_contact"
                        " / tolerances.yaml:feature_check_tolerances.static_intersection_mm3").strip(" |"))


def judge_fixed_path(geo, motion, tol):
    """Rigid subassembly, one declared outward polyline, complete declared workbench state.

    All movers share one translation. No per-component choice of direction and no
    omission of a present obstacle. Other workbench subassemblies are not magically
    treated as co-located in their final world poses.
    """
    import numpy as np
    movers, present = motion.get("movers"), motion.get("present")
    if not isinstance(movers, list) or not movers or not isinstance(present, list) or not present:
        raise ValueError("需显式 movers 与 present（此工位当前在场的全部实体）")
    if len(set(movers)) != len(movers) or len(set(present)) != len(present):
        raise ValueError("movers/present 有重复实体")
    if not set(movers) <= set(present) or not set(present) <= set(geo.index):
        raise ValueError("移动件不在 present 内，或有实体未导出：" + str(sorted(set(present) - set(geo.index))))
    if motion.get("sense") != "withdrawal_from_assembled" or not motion.get("workspace"):
        raise ValueError("需声明工位和 withdrawal_from_assembled：t=0 为最终配合位，反向为装入")
    segs = _segments_of(motion)
    step = float(motion["step_mm"]) if motion.get("step_mm") is not None else float("nan")
    if not np.isfinite(step) or step <= 0:
        raise ValueError("路径步长须为有限正数")
    obs = sorted(set(present) - set(movers))
    if not obs:
        raise ValueError("没有障碍物，零交集没有证据")
    rows, evidence = [], 0
    contacts = []                        # hr41g：超阈交集采样 (移动件, 障碍, 位移向量, 路程, 体积)
    for mv in movers:
        off = np.zeros(3)
        pk, at, blocker, per = 0.0, 0.0, None, {ob: (0.0, 0.0) for ob in obs}
        walked = 0.0
        for axis, length in segs:
            sink = []
            p1, a1, b1, n1, per1 = _sweep1_from(geo, mv, obs, off, axis, length, step, over=tol, sink=sink)
            contacts += [(mv, ob3, o3, walked + t3, v3) for ob3, o3, t3, v3 in sink]
            evidence += n1
            for ob, (v, t) in per1.items():
                if v > per[ob][0]:
                    per[ob] = (v, walked + t)
            if p1 > pk:
                pk, at, blocker = p1, walked + a1, b1
            off = off + axis * length
            walked += length
        proven, unproven, nc = _proven_clear_at(geo, mv, obs, off, segs[-1][0])
        evidence += nc
        rows.append(dict(mover=mv, peak=pk, at=at, blocker=blocker,
                         proven=proven, unproven=unproven, per=per))
    ok = all(r["peak"] <= tol and r["proven"] for r in rows)
    elastic = None
    if not ok and contacts:
        elastic = classify_elastic(geo, contacts, tol)          # 布尔异常向上抛 → 调用方 unknown
        unproven = [r["mover"] for r in rows if not r["proven"]]
        if unproven:
            elastic["ok"] = False
            elastic["unproven"] = unproven
    return dict(ok=ok,
                peak=max(r["peak"] for r in rows), rows=rows, evidence_n=evidence,
                movers=movers, obstacles=obs, elastic=elastic,
                direction=[a.tolist() for a, _ in segs] if len(segs) > 1 else segs[0][0].tolist(),
                segments=[dict(direction=a.tolist(), len_mm=L) for a, L in segs])


def motion_coverage(st):
    """动作义务先声明；删除路径或漏掉刚性组成员不能使整步通过。"""
    required, motions = st.get("required_motion_groups"), st.get("motions")
    if not isinstance(required, dict) or not required or not isinstance(motions, list) or not motions:
        raise ValueError("缺 required_motion_groups 或 motions，不能证明本步动作全集已覆盖")
    ids = [m.get("id") for m in motions]
    if len(ids) != len(set(ids)) or set(ids) != set(required):
        raise ValueError(f"动作覆盖不全/重复：需要 {list(required)}，实际 {ids}")
    for m in motions:
        obligation = required[m["id"]]
        if not isinstance(obligation, dict) or not obligation.get("workspace"):
            raise ValueError(f"{m['id']} 缺工位义务")
        if m.get("workspace") != obligation["workspace"]:
            raise ValueError(f"{m['id']} 工位与义务不符")
        for field in ("movers", "present"):
            expected, actual = obligation.get(field), m.get(field)
            if not isinstance(expected, list) or not expected or len(set(expected)) != len(expected):
                raise ValueError(f"{m['id']} 的 {field} 义务为空/重复")
            if not isinstance(actual, list) or sorted(actual) != sorted(expected):
                raise ValueError(f"{m['id']} 的 {field} 实例不全：需要 {expected}，实际 {actual}")
    return len(ids)


# ── F-L4-1（2026-09-13 审计第二批）：工位在场全集不能只靠自报 ─────────────────────────
#   present 以前完全由写 assembly.yaml 的人给，少写一个障碍没人发现（"条件绿"）。现在按 data 推导每个动作的
#   **最小**在场集合并要求 present ⊇ 它：
#       derived(M) = movers(M) ∪ resolve(step.parts) ∪ { movers(M') : M' 早于 M（按 seq、再按 motions 顺序），
#                                                       subassembly_of(M') ∈ closure(subassembly_of(M)) }
#   再按动作所在的**侧**过滤（左侧动作只要求左件 + 中央件；右件在另一张台面上是设计如此）。
#   子总成闭包来自 assembly.yaml:subassemblies（只声明一次），归属来自每步 subassembly_of（motions 可逐动作覆盖）。
#   反向（present 里有按 seq 应当**还没装**的实体）只 WARN：多算障碍只会让扫掠更严，但说明声明的顺序自相矛盾。
def _side_of(stem, names):
    """placed 实体在哪一侧：'L' / 'R' / 'C'（中央/单件）。规则全部来自 data 与 build 的命名约定：
    打印件 = parts.yaml:placed_instances 的次序（首项左件）；舵机 = frozen.yaml 关节名的 left_/right_ 前缀；
    轴承 = build 导出名里的 left/right（bearing_left_*/bearing_right_*）；其余（trunk/neck/head/电池/原版件）= 中央。"""
    if not hasattr(names, "_side_cache"):
        names._side_cache = {}
        for p in ((names.ctx.data.get("parts") or {}).get("parts") or []):
            stems = names.part_stems(p.get("id"))
            if len(stems) >= 2:
                names._side_cache[stems[0]] = "L"
                for s2 in stems[1:]:
                    names._side_cache[s2] = "R"
            elif stems:
                names._side_cache[stems[0]] = "C"
        for j, s2 in names.servo_by_joint.items():
            names._side_cache[s2] = "L" if j.startswith("left_") else ("R" if j.startswith("right_") else "C")
    if stem in names._side_cache:
        return names._side_cache[stem]
    if stem.startswith("bearing"):
        if "left" in stem:
            return "L"
        if "right" in stem:
            return "R"
        return "C"
    return "R" if stem.endswith("_R") else "C"


def _subassembly_closure(asm, name):
    """closure(X) = X 及 subassemblies[X].members 递归。X 没声明 → None（调用方判 unknown）。"""
    subs = asm.get("subassemblies") or {}
    if name not in subs:
        return None
    out, todo = set(), [name]
    while todo:
        x = todo.pop()
        if x in out:
            continue
        out.add(x)
        node = subs.get(x)
        if not isinstance(node, dict):
            return None
        for m in (node.get("members") or []):
            if m not in subs:
                return None
            todo.append(m)
    return out


def ordered_motions(asm):
    """[(位置, step, motion, subassembly)]，位置 = (seq, motions 里的下标)。seq 缺 → 退回 step 编号。"""
    out = []
    for st in (asm.get("assembly_order") or []):
        seq = st.get("seq", st.get("step"))
        for k, m in enumerate(st.get("motions") or []):
            out.append(((float(seq), k), st, m, m.get("subassembly_of") or st.get("subassembly_of")))
    out.sort(key=lambda r: r[0])
    return out


def intro_table(asm, names):
    """每个 placed 实体**第一次进入装配**的位置与子总成：
       · 作为某动作的 mover 出现 → (该动作位置, 该动作的 subassembly_of)；
       · 从未移动过、只在某步 parts 里出现（台面底件 L03/T01/H01…）→ ((该步 seq, -1), 该步的 subassembly_of)。
    位置越小越早。既不是 mover 也不在任何 parts 里的实体（orig_face_part…）不在表里 = assembly.yaml 没有任何一步装它。"""
    key = id(asm)
    cache = getattr(names, "_intro_cache", None)
    if cache and cache[0] == key:
        return cache[1]
    mover_first = {}                 # 实体 → (最早作为 mover 的位置, 子总成)
    for pos, _st, m, sub in ordered_motions(asm):
        for s2 in (m.get("movers") or []):
            mover_first.setdefault(s2, (pos, sub))
    parts_first = {}                 # 实体 → (最早出现在 parts 的步位置 (seq,-1), 子总成)
    for st in sorted((asm.get("assembly_order") or []), key=lambda x: float(x.get("seq", x.get("step")))):
        pos = (float(st.get("seq", st.get("step"))), -1)
        for t in (st.get("parts") or []):
            for s2 in names.resolve(t)[0]:
                parts_first.setdefault(s2, (pos, st.get("subassembly_of")))
    intro = {}
    for s2 in set(mover_first) | set(parts_first):
        mv, pt = mover_first.get(s2), parts_first.get(s2)
        # 同一步里既在 parts 又是某动作的 mover → 以动作为准（它说了这一步里什么时候进来）；
        # 更早的步里就作为台面底件出现过 → 那一步才是它进入装配的时刻（L03 在步 7 是台面，步 8 才整条腿一起动）
        if mv is None:
            intro[s2] = pt
        elif pt is None or pt[0][0] >= mv[0][0]:
            intro[s2] = mv
        else:
            intro[s2] = pt
    names._intro_cache = (key, intro)
    return intro


def derive_present(asm, names, st, motion, upto=None, closure_of=None, side=None):
    """返回 (required_stems, side, note) 或抛 ValueError（声明不全 → 调用方 unknown）。
    required = movers ∪ { s ∈ intro_table : intro_pos(s) < upto 且 intro_sub(s) ∈ closure(子总成) }，再按侧过滤。
    upto：(seq, idx) 位置，严格早于它的算已装；默认本动作自己的位置（本步 parts 里的台面底件位置是 (seq,-1)，自然算早）。
    closure_of：子总成名；默认本动作的 subassembly_of。side：显式给侧（tool_states 声明的），不给则由 movers 推。"""
    mot_id = motion.get("id")
    movers = list(motion.get("movers") or [])
    if not movers:
        raise ValueError("动作没有 movers")
    if side is None:
        sides = {_side_of(s, names) for s in movers}
        sides.discard("C")
        if len(sides) > 1:
            raise ValueError(f"movers 同时含左右两侧实体 {sorted(sides)}，一个动作只能在一张台面上")
        side = sides.pop() if sides else "C"
    elif side not in ("L", "R", "C"):
        raise ValueError(f"side={side!r} 只能是 L/R/C")
    sub = closure_of or motion.get("subassembly_of") or st.get("subassembly_of")
    if not sub:
        raise ValueError(f"步 {st.get('step')} / 动作 {mot_id} 没有 subassembly_of（assembly.yaml 每步必须声明一次）")
    clo = _subassembly_closure(asm, sub)
    if clo is None:
        raise ValueError(f"subassembly_of={sub!r} 在 assembly.yaml:subassemblies 里找不到或 members 不闭合")
    if upto is None:
        me = [r for r in ordered_motions(asm) if r[1] is st and r[2] is motion]
        if not me:
            raise ValueError("动作不在 assembly_order 里")
        upto = me[0][0]
    intro = intro_table(asm, names)
    required = set(movers)
    n_early = 0
    for s2, (pos, isub) in intro.items():
        if pos < upto and isub in clo:
            required.add(s2); n_early += 1
    keep = {s2 for s2 in required if _side_of(s2, names) in (side, "C")}
    unresolved = [t for t in (st.get("parts") or []) if not names.resolve(t)[0]]
    note = (f"侧={side}；闭包 {sub} → {sorted(clo)}；早于本位置、闭包内已装实体 {n_early} 个"
            + (f"；本步 parts 里认不出的词 {unresolved}（不进推导集合）" if unresolved else ""))
    return keep, side, note


def installed_upto(asm, names, upto):
    """严格早于 upto 进入装配的所有实体（不分子总成）：present 里出现别的实体 = 声明顺序自相矛盾。"""
    return {s2 for s2, (pos, _sub) in intro_table(asm, names).items() if pos < upto}


def judge_present(res, asm, names, subject, check_tag, st, motion, present, upto=None, closure_of=None, prov="",
                  side=None):
    """两条判据：present ⊇ derived（BLOCK）；present ⊆ 已装 ∪ 本步 parts（WARN，顺序矛盾）。"""
    try:
        req, side, note = derive_present(asm, names, st, motion, upto=upto, closure_of=closure_of, side=side)
    except ValueError as e:
        res.unknown(subject, f"present_covers_derived:{check_tag}", f"推导不出最小在场集合：{e}", provenance=prov)
        return False
    pres = set(present or [])
    missing = sorted(req - pres)
    res.add(subject=subject, check=f"present_covers_derived:{check_tag}",
            state=PASS if not missing else FAIL, severity=BLOCK, measured=len(missing),
            criterion="present ⊇ movers ∪ {按 seq/动作顺序早于本位置进入装配、且所属子总成 ∈ closure(本动作 subassembly_of) 的实体}"
                      "（实体进入装配 = 首次作为 mover，或首次出现在某步 parts 里的台面底件；按动作所在侧过滤：左侧动作只要求左件+中央件）；"
                      "差集非空 = 少写了障碍，扫掠结论是条件绿",
            evidence_n=len(req),
            detail=f"推导集合 {len(req)} 个；{note}" + (f"；**present 缺 {missing}**" if missing else "；present 全覆盖"),
            provenance=prov + " | assembly.yaml:subassemblies / assembly_order[].seq,subassembly_of")
    pos = upto
    if pos is None:
        me = [r for r in ordered_motions(asm) if r[1] is st and r[2] is motion]
        pos = me[0][0] if me else None
    if pos is not None:
        allowed = installed_upto(asm, names, pos) | set(motion.get("movers") or [])
        intro = intro_table(asm, names)
        early = sorted(pres - allowed)
        # 同一步里稍后动作才装的实体（"另一颗已装也通过"这种故意多算的障碍）只记 INFO；
        # 更晚的步才装的、或 assembly.yaml 没有任何一步装的 → WARN（顺序声明自相矛盾 / 漏了一步）
        same_step = [x for x in early if x in intro and intro[x][0][0] == pos[0]]
        later = [x for x in early if x in intro and intro[x][0][0] > pos[0]]
        never = [x for x in early if x not in intro]
        bad = later + never
        res.add(subject=subject, check=f"present_only_installed:{check_tag}",
                state=PASS if not bad else FAIL, severity=WARN, measured=len(bad),
                criterion="present ⊆ {按 seq 早于本步进入装配的实体} ∪ 本步实体；"
                          "多出来的实体按声明顺序此时还没装 —— 多算障碍只会让扫掠更严，但顺序声明自相矛盾（或某件从没被任何一步装上）要人看；"
                          "同一步里稍后动作才装的只记入 detail",
                evidence_n=len(pres),
                detail=((f"更晚的步才装的：{later}；" if later else "")
                        + (f"assembly.yaml 任何一步的 parts/movers 里都没出现（没有任何一步装它）：{never}；" if never else "")
                        + (f"同一步稍后动作才装（故意多算的障碍，不判）：{same_step}" if same_step else "")
                        or "present 里没有按顺序还没装的实体"),
                provenance=prov + " | assembly.yaml:assembly_order[].seq")
    return not missing


def run_declared_motions(res, geo, st, tol):
    """Machine-readable steps coexist with unresolved historical prose, without auto-passing it."""
    sid = f"step{st['step']:02d}"
    outcomes = []
    try:
        n = motion_coverage(st)
    except (ValueError, TypeError) as e:
        res.unknown(sid, "motion_coverage", str(e), provenance=f"assembly.yaml:{sid}.required_motion_groups")
        return {"ok": False, "motions": []}
    res.add(subject=sid, check="motion_coverage", state=PASS, severity=BLOCK,
            measured=n, evidence_n=n, criterion="动作ID全集、工位、移动组与在场全集均与独立义务声明相符",
            provenance=f"assembly.yaml:{sid}.required_motion_groups")
    asm = getattr(res, "_asm", None) or {}
    names = getattr(res, "_names", None)
    for k, motion in enumerate(st["motions"]):
        if names is not None:
            judge_present(res, asm, names, sid, motion.get("id", k), st, motion, motion.get("present"),
                          prov=f"assembly.yaml:{sid}.motions[{k}].present")
        mid = motion.get("id", k)
        try:
            row = judge_fixed_path(geo, motion, tol)
            outcomes.append(row)
            el = row.get("elastic")
            el_ok = (not row["ok"]) and bool(el and el["ok"])
            res.add(subject=sid, check=f"motion:{mid}",
                    state=PASS if row["ok"] else FAIL, severity=WARN if el_ok else BLOCK,
                    measured=row["peak"], evidence_n=row["evidence_n"],
                    criterion=(f"同一刚性子总成、声明方向、全部在场障碍，交集 ≤ {tol} mm³；末端完全脱离"
                               "（超阈交集若全部为声明弹性接触 → WARN，见 elastic_contact_declared；永不因此判 PASS）"),
                    detail=((f"刚体交集超阈，但每个超阈采样都落在声明弹性区内 → 降为 WARN，见 elastic_contact_declared:{mid}；"
                             if el_ok else "")
                            + str({"workspace": motion["workspace"],
                                   **{kk: vv for kk, vv in row.items() if kk != "elastic"}})),
                    provenance=f"assembly.yaml:{sid}.motions[{k}]")
            if el and el["applicable"]:
                emit_elastic(res, sid, f"elastic_contact_declared:{mid}", el, tol,
                             prov=f"assembly.yaml:{sid}.motions[{k}]")
        except Exception as e:
            outcomes.append({"ok": False, "peak": None, "evidence_n": 0})
            res.unknown(sid, f"motion:{mid}", str(e),
                        provenance=f"assembly.yaml:{sid}.motions[{k}]")
    ok = bool(outcomes) and all(r["ok"] for r in outcomes)
    el_only = (bool(outcomes) and not ok
               and all(r["ok"] or bool((r.get("elastic") or {}).get("ok")) for r in outcomes))
    res.add(subject=sid, check="step_sweep", state=PASS if ok else FAIL, severity=WARN if el_only else BLOCK,
            measured=[r["peak"] for r in outcomes], evidence_n=sum(r["evidence_n"] for r in outcomes),
            criterion=("本步全部必需子动作已覆盖且可达；这不代替整机拆卸序与螺丝刀路检查"
                       "（未通过的动作若全部只差声明弹性接触 → WARN，不判 PASS）"),
            detail=(f"未通过的动作全部是声明弹性接触（{ELASTIC_TEXT}）" if el_only else ""),
            provenance=f"assembly.yaml:{sid}.motions")
    return {"ok": ok, "elastic_ok": el_only, "motions": outcomes}


def _world_of_hole(frames, part, p):
    import numpy as np
    f = frames.get(part)
    if not f:
        return None
    R = np.asarray(f["world_to_export_local_R"], float)
    t = np.asarray(f["world_to_export_local_t"], float)
    return R.T @ (np.asarray(p, float) - t)                       # p_export = R·p_world + t


def _drive_axis(txt):
    """drive_direction 的自然语言 → **从螺丝头指向机外**的世界方向（带符号）。

    FG12：刀路必须从真实螺丝头一侧出发、一路连到机外，所以这里要的是"起子从哪边来"，
    不是"螺丝往哪边拧"。'从' 后面那个方位就是起子来的那一侧 = 向外的方向：
        'PH0 从 -y 内侧'          → −y
        'PH0（从顶板上方 z+ 向下）' → +z（'从…上方'；'向下' 是拧入方向，不是起子方位）
        'PH0 从 L01 杯底 (-z) 向上' → −z
        'PH0 长杆经 Ø15 内腔从下向上' → −z
    返回 (向外的单位向量, 说明) 或 (None, 原因)。原文同时给出字母轴和上/下且互相矛盾时
    返回 None —— 不猜（元规则 4）。"""
    import numpy as np
    s = str(txt or "")
    lit = None
    toks = {(m.group(2), m.group(1)) for m in re.finditer(r"([+-])\s*([xyz])", s)}
    if len(toks) > 1:
        return None, (f"drive_direction 里出现了不止一个带符号的轴 {sorted(toks)}：{s!r} —— 不猜")
    if toks:
        ax, sg = toks.pop()
        lit = np.asarray(_AXV[ax], float) * (1.0 if sg == "+" else -1.0)
    ud = None
    if re.search(r"从[^，；。]{0,6}(上方|上面|顶)|从上|向下", s):        # 起子在上 / 往下拧 → 头朝 +z
        ud = np.asarray(_AXV["z"], float)
    elif re.search(r"从下|杯底|底部|下方|向上", s):                     # 起子在下 / 往上拧 → 头朝 −z
        ud = -np.asarray(_AXV["z"], float)
    if lit is not None and ud is not None and float(lit @ ud) < 0:
        return None, (f"drive_direction 里字母轴与上/下说法矛盾：{s!r} —— "
                      f"字母轴说头朝 {'+' if float(lit[2] or lit[0] or lit[1]) > 0 else '−'}、"
                      f"上/下说法说反过来；起子从哪一侧来说不清，不猜")
    v = lit if lit is not None else ud
    if v is None:
        return None, f"drive_direction 解析不出『起子从哪一侧来』的世界方向：{s!r}"
    tag = {(1.0, 0.0, 0.0): "+x", (-1.0, 0.0, 0.0): "−x", (0.0, 1.0, 0.0): "+y",
           (0.0, -1.0, 0.0): "−y", (0.0, 0.0, 1.0): "+z", (0.0, 0.0, -1.0): "−z"}[tuple(float(x) for x in v)]
    return v, f"起子从 {tag} 一侧进入（螺丝头朝 {tag}；原文 {s!r}）"


def _reach_to_outside(world_bounds, point, axis):
    """从 point 沿 axis 走到**整机包围盒之外**要多长。判据的"外部"由 placed/ 的整机包围盒给出，
    层里不写死数字。"""
    import numpy as np
    lo, hi = world_bounds
    p = np.asarray(point, float)
    a = np.asarray(axis, float)
    a = a / np.linalg.norm(a)
    ts = [float((hi[i] - p[i]) / a[i]) if a[i] > 0 else float((lo[i] - p[i]) / a[i])
          for i in range(3) if abs(float(a[i])) > 1e-12]
    return max(0.0, min(ts))


def judge_tool_channel(geo, seat, ax_out, ch_d, length, tol, stems=None):
    """从螺丝头一侧 seat 沿 ax_out 建 Ø ch_d × length 的刀路圆柱，与整机每个实体求交。
    返回 {peak, culprits, evidence_n, ok}。布尔炸了向上抛（不吞成 0）。"""
    chan = geo.wrap(_cyl(float(ch_d), float(length), seat, ax_out))
    peak, cul, n = 0.0, [], 0
    for stem in sorted(stems if stems is not None else geo.index):
        v = geo.inter(chan, geo.solid(stem))
        n += 1
        peak = max(peak, v)
        if v > tol:
            cul.append((v, stem))
    return dict(peak=peak, culprits=sorted(cul, reverse=True), evidence_n=n, ok=peak <= tol)


def run(ctx) -> LayerResult:                                       # noqa: C901
    import numpy as np
    res = LayerResult(LAYER, NAME)
    tolrec = ((ctx.data.get("tolerances") or {}).get("feature_check_tolerances") or {}) \
        .get("static_intersection_mm3") or {}
    tol = tolrec.get("max")
    if tol is None:
        res.unknown("_layer", "threshold", f"{_TOL_KEY}.max 取不到，第 4 层没有判据", provenance=_TOL_KEY)
        return res
    crit = f"扫掠过程中交集 ≤ {tol} mm³（{_TOL_KEY}）"

    if not PLACED.exists() or not any(PLACED.glob("*.stl")):
        res.unknown("_layer", "placed_dir", "cad/duck_s288/placed/ 不存在或为空 —— 拿不到世界坐标件",
                    provenance="tools/gate/layers/__init__.py: ctx.placed")
        return res
    geo = _Geo(ctx)
    names = _Names(ctx, geo)
    for stem, p in geo.index.items():
        res.inputs.append(_rel(p))

    asm = ctx.data.get("assembly") or {}
    steps = asm.get("assembly_order") or []
    if not steps:
        res.unknown("_layer", "assembly_order", "assembly.yaml:assembly_order 为空", provenance="assembly.yaml")
        return res

    # BOM 数量独立于映射列表；删除右件/重复左件不能减少检查义务。
    _, inventory_errors = placed_instance_map((ctx.data.get("parts") or {}).get("parts") or [], geo.index)
    res.add(subject="_framework", check="placed_instances", state=FAIL if inventory_errors else PASS,
            severity=BLOCK, measured=inventory_errors or len(ctx.parts), evidence_n=len(ctx.parts),
            criterion="每件实例数等于 BOM qty；实例唯一、文件存在、跨件不重复",
            provenance="parts.yaml:parts[].placed_instances/qty")
    if inventory_errors:
        return res

    # 两个 API 的首实例应一致；顺序来自 data，不依赖 glob 顺序。
    bad_pf = []
    for p in ((ctx.data.get("parts") or {}).get("parts") or []):
        pid = p.get("id")
        got = ctx.placed_for(pid)
        mine = names.part_stems(pid)
        if got is None:
            bad_pf.append(f"{pid}→None（本层解析到 {mine or '也解析不出'}）")
        elif mine and got.stem != mine[0]:
            bad_pf.append(f"{pid}→{got.stem}（左件其实是 {mine[0]}）")
    if bad_pf:
        res.add(subject="_framework", check="ctx_placed_for", state=FAIL, severity=WARN,
                measured=len(bad_pf),
                criterion="ctx.placed_for(pid) 应稳定返回该件的**左件** placed 实体",
                evidence_n=len(ctx.parts),
                detail="；".join(bad_pf),
                provenance="tools/gate/gate.py:Ctx.placed_for")

    # placed 实体名 → 件号（违规归因用；part_stems 每次都要扫一遍清单，这里算一次）
    pid_of_stem = {}
    for _p in ((ctx.data.get("parts") or {}).get("parts") or []):
        for _s in names.part_stems(_p.get("id")):
            pid_of_stem[_s] = _p.get("id")

    # ── 数据缺陷：移动件与方向符号都没有字段 ─────────────────────────────
    unstructured = [s.get("step") for s in steps if not s.get("motions")]
    res.add(subject="_data", check="mover_set_declared", state=FAIL if unstructured else PASS, severity=WARN, measured=unstructured,
            criterion="assembly.yaml 每步应声明移动件/不动件（movers 字段）",
            evidence_n=len(steps),
            detail=f"仍未结构化的步骤 {unstructured}。motions 必须声明每个刚性移动组和工位在场全集；未迁移步骤仍按旧规则诊断。",
            provenance="assembly.yaml:assembly_order[].parts")
    res.add(subject="_data", check="direction_sign_convention", state=FAIL if unstructured else PASS, severity=BLOCK, measured=unstructured,
            criterion="direction 字段必须声明是装入方向还是抽出方向",
            evidence_n=len(steps),
            detail="实测两个反例：step1 'ex（舵机法兰方向）' 里 +ex 峰值≈0 而 −ex 峰值 2465 mm³（+ex 是抽出）；"
                   "step6 '世界 -z→+z' 里 −z 峰值 0 而 +z 峰值 2468 mm³（写的是装入、能走的是 −z）。"
                   "同一个字段两种含义 → 本层只能两个符号都扫、取较小的判，"
                   "**这等于把判据放松了一半**，必须由 data 补一个 sense 字段来收紧。"
                   "本层已把符号解析出来并逐步判 step_direction_declared（声明的那一侧是否真能走）",
            provenance="assembly.yaml:assembly_order[].direction")
    res.add(subject="_data", check="path_start_state_declared", state=FAIL if unstructured else PASS, severity=WARN, measured=unstructured,
            criterion="assembly.yaml 每步应声明路径起点状态（原位起步 / 自由空间起步）与起点坐标",
            evidence_n=len(steps),
            detail="placed/ 里所有件都在**最终位姿**，所以本层只能把 t=0 当『原位起步』"
                   "（移动件已经坐在配合位上），扫掠是从装配位往外走 = 装入路径的反演。"
                   "因此『起点在障碍体外』这条无法对 t=0 判，改判**终点**：走完 len_mm 必须"
                   "已证明完全脱离所有障碍（沿方向越过 或 ⊥方向影子分离），否则这条路径"
                   "没有证明件是从自由空间进来的 —— 见 judge_step_path / _proven_clear_after。"
                   "data 若要更严，应补 path.start_pose 与 path.sense 两个字段",
            provenance="assembly.yaml:assembly_order[].path")

    # ── A 19 步扫掠 ───────────────────────────────────────────────────────
    res._asm, res._names = asm, names          # run_declared_motions 里的 F-L4-1 推导要用
    step_result = {}
    for st in steps:
        n = st.get("step")
        sid = f"step{n:02d}" if isinstance(n, int) else f"step_{n}"
        if st.get("motions"):
            step_result[n] = run_declared_motions(res, geo, st, tol)
            res.covered.add(str(st.get("id", n)))
            continue
        parts = [str(x) for x in (st.get("parts") or [])]
        already = [str(x) for x in (st.get("already_installed") or [])]
        path = st.get("path") or {}
        kind, L, dstep = path.get("kind"), path.get("len_mm"), path.get("step_mm")
        verified = bool(st.get("verified"))
        vby = st.get("verified_by")
        base = (f"action={st.get('action')!r}；verified={verified}（verified_by={vby!r}）；"
                f"kind={kind!r} len={L} step={dstep}")

        if not parts:
            res.unknown(sid, "step_sweep", f"这一步 parts 为空，没有可扫掠的件。{base}",
                        provenance=f"assembly.yaml:assembly_order[{n}]")
            continue
        mov_stems, how = names.resolve(parts[0])
        local_stems, inst_stems = [], []
        for t in parts[1:]:
            local_stems += names.resolve(t)[0]
        for t in already:
            inst_stems += names.resolve(t)[0]
        local_stems = sorted(set(local_stems) - set(mov_stems))
        inst_stems = sorted(set(inst_stems) - set(mov_stems) - set(local_stems))
        obs_stems = sorted(set(local_stems) | set(inst_stems))

        if kind not in ("linear",) or L is None or dstep is None:
            why = {None: "path.kind 是 null（这一步根本没有路径模型）",
                   "axial_press": "path.kind=axial_press（压装），data 没给压入轴，也没给行程/步长",
                   "multi": "path.kind=multi，axis 字段是三条子运动的自然语言，"
                            "assembly.yaml 没把子运动拆成结构化条目、也没声明各自的移动件"}.get(
                kind, f"path.kind={kind!r}")
            miss = [k for k, v in (("len_mm", L), ("step_mm", dstep)) if v is None]
            res.unknown(sid, "step_sweep",
                        f"{why}" + (f"；缺 {', '.join(miss)}" if miss else "") +
                        f" —— 无法扫掠，本步不代跑也不代过。{base}",
                        provenance=f"assembly.yaml:assembly_order[{n}].path")
            continue
        if not mov_stems:
            res.unknown(sid, "step_sweep",
                        f"移动件 parts[0]={parts[0]!r} 在 placed/ 里认不出实体（{_MOVER_RULE}）。{base}",
                        provenance=f"assembly.yaml:assembly_order[{n}].parts")
            continue
        if not obs_stems:
            res.unknown(sid, "step_sweep",
                        f"这一步没有任何可解析的不动件（parts[1:]={parts[1:]}，"
                        f"already_installed={already}）—— 扫掠出来必然是 0，"
                        f"那是没查，不是通过（元规则 2）。{base}",
                        provenance=f"assembly.yaml:assembly_order[{n}].already_installed")
            continue

        d, dsign, dnote = _axis_from_text(st.get("direction") or path.get("axis"),
                                          servo_x=_servo_x_for(st, names))
        if d is None:
            res.unknown(sid, "step_sweep", f"{dnote}。{base}",
                        provenance=f"assembly.yaml:assembly_order[{n}].direction")
            continue
        # 左右镜像件的方向 = mirror_y(d)。本层解析出来的方向全是世界轴对齐的，
        # mirror_y 对轴对齐向量等价于 ±d —— 两个符号都扫，镜像已经被覆盖。
        #
        # FG10：**判据只有一条，对合并后的完整障碍集合求交**。以前把 local（本步 parts 里的
        # 配合件）与 installed（already_installed 全集）分成两组、每组各自挑较好的一侧符号，
        # 于是"被本步件挡 +x、被已装件挡 −x"两条判据同时 0 mm³ 判绿而实际无路可走。
        # 分组的数字仍然出（下面的 step_sweep_local / _installed，severity=INFO 诊断），
        # 它们从**同一次**扫掠的逐障碍记录里拆出来，不再单独重扫。
        try:
            v = judge_step_path(geo, mov_stems, obs_stems, d, float(L), float(dstep), tol)
        except Exception as e:                                     # noqa: BLE001
            res.unknown(sid, "step_sweep",
                        f"扫掠布尔失败（不当作 0 通过）：{e}。{base}",
                        provenance="duckstructure/checks.py:vol 的做法：布尔炸了必须中止")
            continue
        rows = v["rows"]

        def _grp(stems_):
            """从同一次合并扫掠里拆出某一组障碍的旧口径数字（逐件取较好符号）—— 只是诊断。"""
            pk, lines = 0.0, []
            for r in rows:
                a = max([r["plus"]["per"].get(x, (0.0, 0.0))[0] for x in stems_] or [0.0])
                b = max([r["minus"]["per"].get(x, (0.0, 0.0))[0] for x in stems_] or [0.0])
                pk = max(pk, min(a, b))
                lines.append(f"{r['mover']} +{a:.4g}/−{b:.4g}→较好的一侧 {min(a, b):.6f}")
            return pk, "；".join(lines) or "无"

        pk_l, txt_l = _grp(local_stems)
        pk_i, txt_i = _grp(inst_stems)
        n_l = v["nsamp"] * len(mov_stems) * len(local_stems) * 2
        n_i = v["nsamp"] * len(mov_stems) * len(inst_stems) * 2
        step_result[n] = dict(peak=v["peak"], ok=v["ok"], peak_local=pk_l, peak_inst=pk_i,
                              movers=mov_stems, local=local_stems, inst=inst_stems,
                              d=d, sign=dsign, L=L, s=dstep, dnote=dnote)

        declared = path.get("peak_mm3")
        head = (f"移动件 {mov_stems}（{how}）沿 {dnote} 平移 0..{L} mm / 步长 {dstep}，"
                f"对**合并后**的 {len(obs_stems)} 个不动件（本步配合件 {len(local_stems)} + "
                f"已装件 {len(inst_stems)}）同一符号整条路径求交。")
        det = head + "；".join(
            f"{r['mover']}：+{r['plus']['peak']:.6f}/−{r['minus']['peak']:.6f} mm³，"
            f"末端脱离 +{'是' if r['plus']['proven'] else '否'}/−{'是' if r['minus']['proven'] else '否'}"
            + (f"，可走的符号 {r['good_senses']}" if r["good_senses"] else "，**两个符号都不成立**")
            + (f"（较好的一侧 {r['sense']} @{r['at']} mm 撞 {r['blocker']}）" if r["peak"] > tol else "")
            for r in rows)
        if v["reason"]:
            det += f"；{v['reason']}"
        if declared is not None:
            det += f"；assembly.yaml 声明 peak_mm3={declared}"
            if abs(float(declared) - v["peak"]) > max(1e-3, 0.05 * max(float(declared), v["peak"])):
                det += "（**与本层实测对不上**）"
        res.add(subject=sid, check="step_sweep",
                state=PASS if v["ok"] else FAIL, severity=BLOCK,
                measured=round(v["peak"], 6),
                criterion=(f"同一方向 + 同一符号 + 完整障碍集合：全程交集 ≤ {tol} mm³ 且走完 "
                           f"{L} mm 已证明完全脱离（{_TOL_KEY}）"),
                evidence_n=v["evidence_n"],
                detail=det + f"。{base}",
                provenance=f"assembly.yaml:assembly_order[{n}] / " + _TOL_KEY)
        # 分组诊断（**不是**放行判据 —— FG10 的假绿就是把这两条当判据来的）
        if local_stems:
            res.add(subject=sid, check="step_sweep_local", state=PASS if pk_l <= tol else FAIL,
                    severity=INFO, measured=round(pk_l, 6),
                    criterion="诊断量：只看本步 parts 里的配合件、逐件取较好符号（旧口径，非判据）",
                    evidence_n=n_l,
                    detail=f"本步配合件 {local_stems}：{txt_l}", provenance=f"assembly.yaml:assembly_order[{n}]")
        else:
            res.unknown(sid, "step_sweep_local",
                        f"parts[1:] 为空（parts={parts}）—— 本步没有可解析的配合件；"
                        f"判据（step_sweep）用的是合并集合，这条只是缺声明的记账。{base}",
                        provenance=f"assembly.yaml:assembly_order[{n}].parts", severity=WARN)
        if inst_stems:
            res.add(subject=sid, check="step_sweep_installed", state=PASS if pk_i <= tol else FAIL,
                    severity=INFO, measured=round(pk_i, 6),
                    criterion="诊断量：只看 already_installed、逐件取较好符号（旧口径，非判据）",
                    evidence_n=n_i,
                    detail=f"already_installed 里另外 {len(inst_stems)} 个已装件：{txt_i}"
                           "。**注意**：placed/ 里所有件都在最终位姿，本诊断不区分"
                           "『已装在主体上』与『还是独立子总成』—— assembly.yaml 没有子总成字段，"
                           "合并判据红了要先看是不是这个原因",
                    provenance=f"assembly.yaml:assembly_order[{n}].already_installed")
        else:
            res.unknown(sid, "step_sweep_installed",
                        f"already_installed 为空 —— 第 1 步之前确实没有已装件；"
                        f"判据（step_sweep）用的是合并集合，这条只是缺声明的记账。{base}",
                        provenance=f"assembly.yaml:assembly_order[{n}].already_installed", severity=WARN)
        # FG11：符号解析出来了就要用 —— data 声明的那一侧是不是真能走，单独判
        if dsign is not None:
            tag = "+" if dsign > 0 else "−"
            bad_sign = [r["mover"] for r in rows if tag not in r["good_senses"]]
            res.add(subject=sid, check="step_direction_declared",
                    state=PASS if not bad_sign else FAIL, severity=WARN,
                    measured=tag, criterion=f"assembly.yaml 声明的符号（{tag}）本身就得是能走的那一侧",
                    evidence_n=len(rows) * 2,
                    detail=(f"{dnote}；" + "；".join(
                        f"{r['mover']} 可走 {r['good_senses'] or '无'}" for r in rows)
                        + ("" if not bad_sign else
                           f" —— **声明的 {tag} 向走不通**（能走的是另一侧）。几何上"
                           f"『沿 +s 抽出 ⇔ 沿 −s 装入』，所以 step_sweep 仍可能是绿的，"
                           f"但按 assembly.yaml 的字面照做会装不进去 —— 见 _data/direction_sign_convention")),
                    provenance=f"assembly.yaml:assembly_order[{n}].direction")
        if not verified:
            res.add(subject=sid, check="step_verified_declared", state=FAIL, severity=WARN,
                    measured=verified, criterion="verified=false 的步必须由本层真跑出数（元规则 4）",
                    evidence_n=v["evidence_n"],
                    detail=f"data 说 verified_by={vby!r}；本层已真扫掠，合并峰值 {v['peak']:.6f} mm³"
                           f"（配合件 {pk_l:.6f} / 已装件 {pk_i:.6f}）—— data 该回填",
                    provenance=f"assembly.yaml:assembly_order[{n}].verified")
        # 失败时补一份 6 方向诊断 —— 这正是历史上"六个方向全堵"那条证据的形态
        if not v["ok"]:
            rows6 = []
            for tag6, vec in (("+x", (1., 0, 0)), ("-x", (-1., 0, 0)), ("+y", (0, 1., 0)),
                              ("-y", (0, -1., 0)), ("+z", (0, 0, 1.)), ("-z", (0, 0, -1.))):
                try:
                    pk6, out6 = 0.0, True
                    for mv in mov_stems:
                        pk6 = max(pk6, _sweep1(geo, mv, obs_stems, vec, float(L), float(dstep))[0])
                        out6 = out6 and _proven_clear_after(geo, mv, obs_stems, vec, float(L))[0]
                    rows6.append((tag6, pk6, out6))
                except Exception:                                  # noqa: BLE001
                    rows6.append((tag6, float("nan"), False))
            ok6 = [t for t, x, o in rows6 if x <= tol and o]
            res.add(subject=sid, check="six_direction_reachability",
                    state=PASS if ok6 else FAIL, severity=BLOCK,
                    measured=min(x for _, x, _ in rows6),
                    criterion="声明方向不成立时，至少要有一个世界轴方向能把移动件整条送到位并完全脱离",
                    evidence_n=6 * len(mov_stems),
                    detail=" ".join(f"{t}:{x:.4g}{'' if o else '(未脱离)'}" for t, x, o in rows6)
                           + ("（**六个方向全堵** —— 只能判定尚未证明可达，斜插/旋转装配另验，"
                              "见 asmcheck.py:3）" if not ok6 else f"（可沿 {', '.join(ok6)} 走）")
                           + f"；移动件 {mov_stems} vs 全部 {len(obs_stems)} 个不动件",
                    provenance=f"assembly.yaml:assembly_order[{n}] / tools/cad/asmcheck.py")
        # 违规落到件的格子上，否则 17 件 × 8 层那张表会全绿。
        # 归因用**选中那条路径**（同一符号、完整障碍集合）的逐障碍记录，不再分组。
        p2 = re.sub(r"[×xX]\s*\d+\s*$", "", parts[0]).strip()
        if p2 not in pid_of_stem.values():
            p2 = None
        blockers = {}                    # 件号 → [最差体积, 说明]（左右两个实例合成一条）
        for r in rows:
            if r["ok"]:
                continue
            w = r["plus"] if r["sense"] == "+" else r["minus"]
            if p2:
                res.add(subject=p2, check=f"assembly_{sid}", state=FAIL, severity=BLOCK,
                        measured=round(r["peak"], 6), criterion=crit, evidence_n=v["evidence_n"],
                        detail=f"第 {n} 步（{st.get('action')}）本件（{r['mover']}）作为移动件"
                               f"两个符号都不成立：较好的一侧 {r['sense']} 峰值 {r['peak']:.4f} mm³ "
                               f"@ {r['at']} mm"
                               + (f"（撞 {r['blocker']}）" if r["blocker"] else "")
                               + (f"；走完 {L} mm 仍未证明脱离 {r['unproven'][:3]}" if r["unproven"] else ""),
                        provenance=f"assembly.yaml:assembly_order[{n}]")
            for ob, (vv, tt) in sorted(w["per"].items(), key=lambda kv: -kv[1][0]):
                if vv <= tol:
                    break
                ob_pid = pid_of_stem.get(ob)
                if ob_pid and ob_pid != p2:
                    e = blockers.setdefault(ob_pid, [0.0, []])
                    if vv > e[0]:
                        e[0] = vv
                    if len(e[1]) < 4:
                        e[1].append(f"{r['mover']} 走 {r['sense']} 向 {vv:.4f} mm³ @ {tt} mm"
                                    f"（{'本步配合件' if ob in local_stems else '已装件'}）")
        for ob_pid, (mx, why2) in sorted(blockers.items()):
            res.add(subject=ob_pid, check=f"assembly_blocks_{sid}", state=FAIL,
                    severity=BLOCK, measured=round(mx, 6), criterion=crit,
                    evidence_n=v["evidence_n"],
                    detail=f"第 {n} 步（{st.get('action')}）本件挡住移动件的直线路径："
                           + "；".join(why2),
                    provenance=f"assembly.yaml:assembly_order[{n}]")

    # ── B 拆卸序 ─────────────────────────────────────────────────────────
    # 拆和装在几何上是同一条扫掠（从装配位往外扫 = 装入路径的反演），真正的区别在**不动件集合**：
    # 装的时候只有 already_installed 在场，拆的时候整机都在（减去这条拆序里已经先拆掉的）。
    # 所以这一段单独按"整机 − 已拆走的"做，不复用 A 段的结果。
    step_by_id = {s.get("step"): s for s in steps}
    removed = set()
    unproven_prior: list[str] = []          # FG11：前面哪几条没证明拆得下来
    elastic_prior: list[str] = []           # hr41g：前面哪几条只靠声明弹性接触拆出（WARN）

    def _prereq(sid_, act_):
        """FG11：前一条没拆成功，后面就不能按『它已经拆走了』继续判。
        这里不静默继续 —— 每条后续都挂一条 BLOCK，说明前置未成立。
        hr41g：前一条只靠声明弹性接触拆出（WARN）时，该件已迁出障碍集合，后续每条挂一条
        disassembly_prereq_elastic（WARN），写明本条结论以卡扣实物能按声明挠曲为前提。"""
        if elastic_prior:
            res.add(subject=sid_, check="disassembly_prereq_elastic", state=FAIL, severity=WARN,
                    measured=len(elastic_prior),
                    criterion="前置条目只在声明弹性接触下成立（WARN）时，本条『那些件已经不在了』以卡扣实物能按声明挠曲为前提",
                    evidence_n=len(elastic_prior),
                    detail=(f"'{act_}' 的前置 {elastic_prior} 只靠声明弹性接触拆出（{ELASTIC_TEXT}）；"
                            "它们要拆的件已按声明迁出障碍集合，本条几何结论随之是条件性的"),
                    provenance="assembly.yaml:disassembly_order / elastic_contact_declared")
        if not unproven_prior:
            return
        res.add(subject=sid_, check="disassembly_prereq", state=FAIL, severity=BLOCK,
                measured=len(unproven_prior),
                criterion="拆卸序是有状态的：只有前面每一条都判定成功，本条才谈得上『那些件已经不在了』",
                evidence_n=len(unproven_prior),
                detail=f"'{act_}' 的前置未成立：{unproven_prior} 这几条没有判定成功，"
                       f"它们要拆的件**仍留在障碍集合里**（本层不做『假定已拆』）。"
                       f"本条的几何结果只在『前置真能拆掉』的前提下才有意义",
                provenance="assembly.yaml:disassembly_order")

    for de in (asm.get("disassembly_order") or []):
        seq = de.get("seq")
        sid = f"disasm{seq:02d}" if isinstance(seq, int) else f"disasm_{seq}"
        rev = de.get("reverse_of_step")
        proven = de.get("proven")
        act = str(de.get("action") or "")
        st = step_by_id.get(rev)
        _prereq(sid, act)
        if rev is None or st is None:
            res.unknown(sid, "disassembly_sweep",
                        f"reverse_of_step={rev!r}（'{act}'）—— 找不到对应的装配步，"
                        f"拿不到行程/步长/移动件。data 自己写着 proven={proven}"
                        f"（{de.get('unknown_reason')}）",
                        provenance=f"assembly.yaml:disassembly_order[{seq}]")
            unproven_prior.append(sid)
            continue
        # hr13（2026-09-18）：拆序条目可以**显式**声明 movers + segments（与 motions 同格式：多段首尾相接的直线，各自世界方向与长度）
        # + step_mm，优先于下面的原文解析。理由：整机拆卸里内部件（H03/N07/N04/偏航舵机…）沿单段直线退出后**仍在整机包围盒里**，
        # AABB 证明不了"已脱离"（_proven_clear_at 只认"沿方向已越过"或"垂直方向包围盒分离"），单段永远红；多段（先退回、再从缺口出机）才证得了，
        # 也才是真实的拆法。障碍集合不变：整机 − 已判定拆走；proven 判据不变。
        if isinstance(de.get("movers"), list) and isinstance(de.get("segments"), list):
            mov_stems = []
            for t in de["movers"]:
                mov_stems += names.resolve(str(t))[0]
            dstep = de.get("step_mm") if de.get("step_mm") is not None else (st.get("path") or {}).get("step_mm")
            obs = sorted(set(geo.index) - set(mov_stems) - removed)
            try:
                if not mov_stems:
                    raise ValueError(f"movers {de['movers']} 在 placed 里认不出实体")
                v = judge_fixed_path(geo, dict(movers=mov_stems, present=sorted(set(obs) | set(mov_stems)),
                                               sense="withdrawal_from_assembled", workspace="whole_robot_minus_removed",
                                               segments=de["segments"], step_mm=dstep), tol)
            except Exception as e:                                     # noqa: BLE001
                res.unknown(sid, "disassembly_sweep", f"显式 movers/segments 判定失败（不当作 0 通过）：{e}",
                            provenance=f"assembly.yaml:disassembly_order[{seq}]")
                unproven_prior.append(sid)
                continue
            ok = bool(v["ok"])
            el = v.get("elastic")
            el_ok = (not ok) and bool(el and el["ok"])
            segtxt = " → ".join(f"{s['direction']}×{s['len_mm']} mm" for s in v["segments"])
            res.add(subject=sid, check="disassembly_sweep", state=PASS if ok else FAIL,
                    severity=WARN if el_ok else BLOCK,
                    measured=round(v["peak"], 6),
                    criterion=(f"声明的多段直线路径 + 完整障碍集合：全程交集 ≤ {tol} mm³ 且走完后已证明完全脱离（{_TOL_KEY}）"
                               "；超阈交集若全部为声明弹性接触 → WARN（见 elastic_contact_declared），不判 PASS"),
                    evidence_n=v["evidence_n"],
                    detail=f"'{act}'（反第 {rev} 步）：移动件 {mov_stems} 走 {segtxt} / 步长 {dstep}，不动件 = 整机 {len(geo.index)} 个实体减去"
                           f"**已判定拆走**的 {len(removed)} 个 = {len(obs)} 个。"
                           + "；".join(f"{r2['mover']} 峰值 {r2['peak']:.6f} mm³ @ {r2['at']} mm"
                                       + (f" 撞 {r2['blocker']}" if r2['peak'] > tol else "")
                                       + f"，末端脱离 {'是' if r2['proven'] else '否（未证明脱离 ' + str(r2['unproven'][:4]) + '）'}"
                                       for r2 in v["rows"])
                           + f"。data 声明 proven={proven}"
                           + ("；刚体交集超阈，但每个超阈采样都落在声明弹性区内 → WARN，按声明可挠拆出"
                              "（见 elastic_contact_declared）" if el_ok else "")
                           + ("；**拆不出来** —— 拆不出来等于装错了也救不回来" if not ok and not el_ok else ""),
                    provenance=f"assembly.yaml:disassembly_order[{seq}] / " + _TOL_KEY)
            if el and el["applicable"]:
                emit_elastic(res, sid, "elastic_contact_declared", el, tol,
                             prov=f"assembly.yaml:disassembly_order[{seq}]")
            if ok:
                removed |= set(mov_stems)
            elif el_ok:
                removed |= set(mov_stems)        # 按声明弹性可拆：迁出，但后续条目挂 disassembly_prereq_elastic（WARN）
                elastic_prior.append(sid)
            else:
                unproven_prior.append(sid)
                blockers3 = {}
                for r2 in v["rows"]:
                    for ob, (vv, tt) in sorted(r2["per"].items(), key=lambda kv: -kv[1][0]):
                        if vv <= tol:
                            break
                        ob_pid = pid_of_stem.get(ob)
                        if ob_pid:
                            e = blockers3.setdefault(ob_pid, [0.0, []])
                            e[0] = max(e[0], vv)
                            if len(e[1]) < 4:
                                e[1].append(f"{r2['mover']} {vv:.4f} mm³ @ {tt} mm")
                for ob_pid, (mx, why2) in sorted(blockers3.items()):
                    res.add(subject=ob_pid, check=f"disassembly_blocks_{sid}", state=FAIL, severity=BLOCK,
                            measured=round(mx, 6), criterion=crit, evidence_n=v["evidence_n"],
                            detail=f"拆序第 {seq} 条（{act}）被本件挡住：" + "；".join(why2),
                            provenance=f"assembly.yaml:disassembly_order[{seq}]")
            continue
        sparts = [str(x) for x in (st.get("parts") or [])]
        # 移动件：action 原文里点到名的件优先；点不到就退回该步的 parts[0]
        picked = [t for t in sparts if t.startswith("servo_")] if "舵机" in act else []
        if len(picked) != 1:
            picked = [t for t in sparts
                      if re.sub(r"[×xX]\s*\d+\s*$", "", t).strip() in act]
        if len(picked) != 1:
            picked = sparts[:1]
        mov_stems = names.resolve(picked[0])[0] if picked else []
        path = st.get("path") or {}
        L, dstep = path.get("len_mm"), path.get("step_mm")
        # 方向：拆序自己的 action 原文优先（它写得比装配步的 direction 具体）
        d, _dsg, dnote = _axis_from_text(act, servo_x=_servo_x_for(st, names))
        if d is None:
            d, _dsg, dnote = _axis_from_text(st.get("direction") or path.get("axis"),
                                             servo_x=_servo_x_for(st, names))
            dnote = dnote + "（退回第 %s 步的 direction，拆序原文里没有方向）" % rev
        if not mov_stems or d is None or L is None or dstep is None:
            miss = []
            if not mov_stems:
                miss.append(f"移动件（action 里点到 {picked}，placed 里认不出实体）")
            if d is None:
                miss.append(f"方向（{dnote}）")
            if L is None or dstep is None:
                miss.append(f"行程/步长（第 {rev} 步 path.len_mm={L} step_mm={dstep}）")
            res.unknown(sid, "disassembly_sweep",
                        f"'{act}' 缺：{'；'.join(miss)}。data 声明 proven={proven}",
                        provenance=f"assembly.yaml:disassembly_order[{seq}]")
            unproven_prior.append(sid)
            continue
        # FG11：**判定成功之前不许把该件迁出障碍集合**。以前这里先 removed.add(mover) 再扫，
        # 于是 A 拆不出来，后续仍按『A 已拆走』继续判，得到虚假的绿。
        obs = sorted(set(geo.index) - set(mov_stems) - removed)
        try:
            v = judge_step_path(geo, mov_stems, obs, d, float(L), float(dstep), tol)
        except Exception as e:                                     # noqa: BLE001
            res.unknown(sid, "disassembly_sweep", f"扫掠布尔失败（不当作 0 通过）：{e}",
                        provenance="duckstructure/checks.py:vol")
            unproven_prior.append(sid)
            continue
        ok = bool(v["ok"])
        res.add(subject=sid, check="disassembly_sweep", state=PASS if ok else FAIL, severity=BLOCK,
                measured=round(v["peak"], 6),
                criterion=(f"同一方向 + 同一符号 + 完整障碍集合：全程交集 ≤ {tol} mm³ 且走完 {L} mm "
                           f"已证明完全脱离（{_TOL_KEY}）"),
                evidence_n=v["evidence_n"],
                detail=f"'{act}'（反第 {rev} 步）：移动件 {mov_stems} 沿 {dnote} 平移 0..{L} mm / "
                       f"步长 {dstep}，不动件 = 整机 {len(geo.index)} 个实体减去**已判定拆走**的 "
                       f"{len(removed)} 个 = {len(obs)} 个。"
                       + "；".join(f"{r2['mover']} +{r2['plus']['peak']:.6f}/−{r2['minus']['peak']:.6f} mm³，"
                                   f"末端脱离 +{'是' if r2['plus']['proven'] else '否'}/"
                                   f"−{'是' if r2['minus']['proven'] else '否'}"
                                   + (f"（较好的一侧 {r2['sense']} @{r2['at']} mm 撞 {r2['blocker']}）"
                                      if r2["peak"] > tol else "")
                                   for r2 in v["rows"])
                       + (f"；{v['reason']}" if v["reason"] else "")
                       + f"。data 声明 proven={proven}"
                       + ("；**拆不出来** —— 拆不出来等于装错了也救不回来"
                          "（踝子总成当年就是死在反向抽出 372.6 mm³ 上）" if not ok else ""),
                provenance=f"assembly.yaml:disassembly_order[{seq}] / " + _TOL_KEY)
        if ok:
            removed |= set(mov_stems)        # 只有**本条判定成功**才允许迁移状态
        else:
            unproven_prior.append(sid)
        blockers2 = {}                   # 件号 → [最差体积, 说明]（同一件只出一条）
        for r2 in v["rows"]:
            if r2["ok"]:
                continue
            w2 = r2["plus"] if r2["sense"] == "+" else r2["minus"]
            for ob, (vv, tt) in sorted(w2["per"].items(), key=lambda kv: -kv[1][0]):
                if vv <= tol:
                    break
                ob_pid = pid_of_stem.get(ob)
                if ob_pid:
                    e = blockers2.setdefault(ob_pid, [0.0, []])
                    if vv > e[0]:
                        e[0] = vv
                    if len(e[1]) < 4:
                        e[1].append(f"{r2['mover']} 走 {r2['sense']} 向 {vv:.4f} mm³ @ {tt} mm")
        for ob_pid, (mx, why2) in sorted(blockers2.items()):
            res.add(subject=ob_pid, check=f"disassembly_blocks_{sid}", state=FAIL, severity=BLOCK,
                    measured=round(mx, 6), criterion=crit, evidence_n=v["evidence_n"],
                    detail=f"拆序第 {seq} 条（{act}）被本件挡住：" + "；".join(why2),
                    provenance=f"assembly.yaml:disassembly_order[{seq}]")

    # ── B' 拧紧工位（tool_states）的在场全集同样要 ⊇ 推导集合（F-L4-1）────────
    steps_by_no = {s.get("step"): s for s in steps}
    for tid, tstate in sorted((asm.get("tool_states") or {}).items()):
        prov = f"assembly.yaml:tool_states.{tid}"
        st = steps_by_no.get((tstate or {}).get("after_step"))
        if st is None:
            res.unknown(f"tool:{tid}", "present_covers_derived:state",
                        f"tool_states.{tid} 缺 after_step（或指向不存在的步）—— 不知道拧紧时装到哪一步，推不出在场集合",
                        provenance=prov)
            continue
        allm = ordered_motions(asm)
        mine = [r for r in allm if r[1] is st]
        if not mine:
            res.unknown(f"tool:{tid}", "present_covers_derived:state",
                        f"第 {st.get('step')} 步没有 motions，推不出该步装了什么", provenance=prov)
            continue
        am, side = tstate.get("after_motion"), tstate.get("side")
        if side not in ("L", "R", "C"):
            res.unknown(f"tool:{tid}", "present_covers_derived:state",
                        f"tool_states.{tid} 缺 side（L/R/C）—— 不知道这是哪张台面上的工位", provenance=prov)
            continue
        if am:
            pick = [r for r in mine if r[2].get("id") == am]
            if not pick:
                res.unknown(f"tool:{tid}", "present_covers_derived:state",
                            f"after_motion={am!r} 不是第 {st.get('step')} 步的动作", provenance=prov)
                continue
        else:
            # 没指定动作 → 取该步里**本侧**最后一个动作（右侧工位不能拿左侧动作当"已装到哪"）
            pick = [r for r in mine if all(_side_of(m, names) in (side, "C") for m in (r[2].get("movers") or []))] or mine
        pos, _st, mot, _sub = pick[-1]
        # 工位"在动作之后"：把该动作自己也算进已装 → upto 取它的下一个位置
        upto = (pos[0], pos[1] + 0.5)
        pseudo = {"id": f"tool:{tid}", "movers": list(mot.get("movers") or []),
                  "subassembly_of": tstate.get("subassembly_of") or mot.get("subassembly_of") or st.get("subassembly_of")}
        judge_present(res, asm, names, f"tool:{tid}", "state", st, pseudo, tstate.get("present"),
                      upto=upto, closure_of=pseudo["subassembly_of"], prov=prov, side=side)

    # ── C/D 真坐面 + 实际工具包络 + 拧紧时工位 ───────────────────
    from tool_access import run_tool_access
    res.inputs.append("tools/gate/tool_access.py")
    res.inputs.append("tools/gate/layers/l5_screwhead.py")
    run_tool_access(res, ctx, geo, names, asm, tol)
    n_channels = sum(f.check == "tool_path_to_outside" for f in res.findings)

    res.add(subject="_crosscheck", check="tool_channel_vs_keepouts", state=NOT_RUN, severity=INFO,
            measured=n_channels, evidence_n=n_channels,
            criterion="禁入通道与对应工具的坐面、轴线、装配状态应关联；二者直径不同，交集体积不必相等",
            detail="keepout 与 tool_access 的显式关联尚未实现。此前把孔中点当坐面、拿内腔当批杆得到的体积不可用于几何修复。",
            provenance="keepouts.yaml / fasteners.yaml:tool_access")

    if names.unresolved:
        res.add(subject="_names", check="unresolved_tokens", state=FAIL, severity=WARN,
                measured=len(names.unresolved),
                criterion="assembly.yaml 里写的每个件/元件都要能落到 placed/ 的实体上",
                evidence_n=sum(names.unresolved.values()),
                detail="认不出的词（出现次数）：" + "、".join(
                    f"{k}×{v}" for k, v in sorted(names.unresolved.items(), key=lambda x: -x[1])[:12])
                       + " —— 其中 '腿子总成' 这类是子总成名（它的组成件已经逐个列在同一份清单里，"
                         "不额外丢证据）；SBC/IMU/buck/switch 是没有 CAD 位置的元件（assembly.yaml"
                         "第 19 步自己写着 not_modeled）",
                provenance="assembly.yaml:assembly_order[].parts / already_installed")

    res.evidence = {"steps_total": len(steps),
                    "steps_swept": len(step_result),
                    "disassembly_total": len(asm.get("disassembly_order") or []),
                     "fastener_groups": len((ctx.data.get("fasteners") or {}).get("fasteners") or []),
                    "fastener_groups_with_envelope": n_channels,
                    "placed_solids": len(geo.index),
                    "booleans_run": geo.booleans}
    return res
