#!/usr/bin/env python3
# ── E′ 停用（2026-09-26，hr46 E 段）──────────────────────────────────────────────────────────────────────────
#   用户原话（09-26 晚）：『注释掉吧，我已经开始打印了，后面我会再找你，有问题那个时候再修吧。』
#   做法：_run_impl 里 E′ 段对 _harness_sweep(ctx, res) 的调用整段注释掉（_harness_sweep / _hs_* 函数本体原样保留、不删），
#         换成一行占位 `hs_done, hs_ev = set(), {"disabled": …}`（带「E′ 停用占位」注释）→ 本层不再出 L6/<线路 id>:harness_sweep 线路格，
#         各束仍走下面原有的『没有折线』unknown 格（HB09_cross_neck 也回到这一格）。反例 n_l6_harness_sweep.py 同时退役
#         （改名 tools/gate/negatives/retired_n_l6_harness_sweep.py 并整文件注释，README 末尾记了一行）。
#   恢复方法：① 删掉带「E′ 停用占位」注释的那一行；② 把它上面紧挨着的 4 行注释里的 3 行原代码取消注释（去掉行首一层 "# "），
#            即恢复 `hs_done, hs_ev = _harness_sweep(ctx, res)` 与其上两行说明注释；③ 反例改回 n_l6_harness_sweep.py 并取消整文件注释。
#   停用前原文备份：docs/design_2026-09-17_bearing_rebuild/hr43_work/hr46/before/E_2026-09-26_user_disable/tools/gate/layers/l6_motion.py
"""第 6 层 运动 —— "动起来撞不撞、卡不卡"。

判据（README 第 2 节 L6）：
  A 单轴全行程：frozen.yaml:joint_axes 里 exists_as_mjcf_joint 的 14 条，
    用 duckstructure.checks.check_angles() 的 **2.5° 独立网格 + 强制含端点**
    （元规则 3：采样网格独立于建模用的切刀角度，否则是自我验证）
  B **相邻关节两两组合**（膝×踝、髋×膝、颈×头、头俯仰×头横滚…）—— 现有检查只做单轴，组合从没查过
  C **能力包络**（2026-09-16 起）：frozen.yaml:capability_envelope 声明的 in-scope 任务模式下，原版策略实际用到的
    关节区间（docs/workspace_2026-09-09/policy_envelope.json，左右镜像求并）。用户定：能力必须与原版一致、转角可以不同 →
    joint_axes[].target_range_deg 由它推导（tools/gate/derive_target_ranges.py），本层另判 _capability/*：
    每个 in-scope mode 的包络 ± margin ⊆ 本层量出的无碰撞区间（BLOCK），包络盒内的组合碰撞 = 0（BLOCK）。
    09-09 那条『策略包络 = 上一版策略、已过期』的 RETIRED 记录随之作废：包络现在是**能力要求**，不是放行依据的历史参考
  D 不只判交集 = 0，还要判**最小距离 ≥ 公差表值**（打印公差一来，间隙 0 但不相交的地方实物就卡死）
  E 线束：harness.yaml 9 束沿 route_polyline 扫管，跨关节的走全行程
  E′ 线束实体模型（hr44reg 2026-09-25）：duckstructure.wiring_body.sweep_summary(coarse=False) 逐线路 → L6/<线路 id>:harness_sweep
    （PASS ⇔ 区内线∩件 0 且区内张紧 0 且零位最小弯 R ≥ 限；调不通 / 超时 / 替身场景 → NOT_RUN(BLOCK)；不走 _Scene、不用 check_cache）

结构性缺陷（frozen.yaml:joint_axes_rule.cutter_vs_mjcf_warning）：
  让位切刀按 LEGACY 角度范围切，而检查按 MJCF 全程跑 → 349 条碰撞。
  所以本层把碰撞分三档报：**策略包络内 / 切刀区间内 / 超出切刀区间**。
  落在切刀区间内的是真缺陷（切刀本该让开却没让开）；超出切刀区间的是工作空间问题
  （MJCF 允许的角度 CAD 从来没打算支持）—— 两者混在一个数字里就没人知道该修哪个。

2026-09-09 权威方向倒转（docs/重训路线_2026-09-09.md）：
  过去 MJCF 是真理、CAD 必须符合它；现在 CAD + 实测是真理，MJCF 由 CAD 生成
  （tools/sim/make_mjcf.py → sim/duck_s288/robot_walk_s288.xml）。所以：
  · 上游 robot_walk.xml 的 <joint range> 只当**扫描候选域**（它最宽），不当放行依据 ——
    "range 必须等于上游"那条判据降级为 _joints/range_matches_upstream（RETIRED，2026-09-13 前误写成 STALE）。
  · 也**不能**把基准改指向新生成的 robot_walk_s288.xml：那份 XML 的 range 正是本层导出的
    collision_free_range，指过去就成了同义反复（更隐蔽的假绿）。降级 + 说清楚，不换基准。
  · 训练模型的判据一律对着**我们生成的那份**跑（_simmodel/*）；上游那份只留一条 RETIRED 记录。

坑：
  · 布尔必须用 process=True 的网格（ctx.solid）；布尔失败抛异常判红，不吞成"体积 0 = 通过"。
  · 不用 trimesh.load(...).split() 数连通块，不用 is_watertight（README 坑 11）。
  · **反向扫掠的区间要交换并取负**：小腿动 −90..+60 时，大腿在小腿坐标里是 −60..+90。
    本层不做手工反演 —— 一律按 MJCF 树正向摆姿态（父连杆动、子连杆跟着动），
    从根本上避免这个符号错（negatives/N4 就是这个坑）。

2026-09-13 审计 F-L6-1/2/3（每条各有反例，见 negatives/n_l6_*.py）：
  · F-L6-3 **目标区间**：碰撞分"目标区间内 = BLOCK（件的问题）／全部在区间外但仍在上游扫描域内 = WARN
    （工作空间问题：控制端限位，不是件的问题）"。目标区间**只认** frozen.yaml:joint_axes[].target_range_deg
    的声明（缺失 / lo≥hi / 非数 / 超出扫描域 / 人为收窄却没有 date+reason → 该关节相关判据 unknown）。
    两两组合按两轴都在各自目标区间内算"区间内"。`collision_free_range` 改判"生成的 MJCF 的 <joint range> ⊆
    本轮扫出的无碰撞区间"（否则 BLOCK），另加 `gen_range_covers_target`（⊇ 目标区间，否则 WARN —— 训练域被收窄）；
    **不再**因为"需要收窄"本身判 BLOCK。这不是同义反复：那份 XML 是独立产物，会过期。
  · F-L6-1 两两组合统一 **5° 网格**（原来 max(10°, 量程/12) 粗网格只在最差粗格附近加密，两轴都非零、宽度 < 网格的
    碰撞口袋看不见）。代价：组合扫描从 44 s 涨到 ≈350 s（2026-09-13 在真场景逐对实测），不为省时间放大网格。
  · F-L6-2 最小距离判据只豁免 relations.yaml 里声明为 `contact` 的 `parties` 件对；零位间隙 < 阈值但没声明的件对
    另发 `_zero:undeclared_zero_gap_pair` FAIL(WARN) 列出，不阻断、不豁免。
"""
from __future__ import annotations
import itertools
import json
import math
import re
import sys
from pathlib import Path

_HERE = Path(__file__).resolve()
sys.path.insert(0, str(_HERE.parents[1]))                       # tools/gate → core
for _p in (str(_HERE.parents[3]), str(_HERE.parents[3] / "tools" / "cad"),
           str(_HERE.parents[3] / "tools" / "sim")):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from core import (LayerResult, PASS, FAIL, NOT_RUN, STALE, RETIRED, BLOCK, WARN, INFO,   # noqa: E402
                  ROOT, PLACED)

LAYER = 6
NAME = "运动"

_TOL_KEY = "tolerances.yaml:feature_check_tolerances.static_intersection_mm3"
_GAP_KEY = "tolerances.yaml:fits.deck.deck_to_L01_gap.target_range_mm.min"
_MANIFEST = ROOT / "tools/sim/cad_geometry_manifest.json"
_ENVELOPE = ROOT / "docs/workspace_2026-09-09/policy_envelope.json"
# 真实姿态碰撞检查（2026-09-16，README A2 的修法）：tools/sim/policy_envelope.py --dump-steps 记原版策略逐控制步关节元组，
# tools/sim/policy_pose_collisions.py 按真实姿态在 placed/ 上做布尔，summary.json 带 placed/steps/脚本三重指纹。
# 本层只读这份离线结果并核指纹（同 L1 的 slice_run 做法）：新鲜且 0 命中 → PASS；有命中 → BLOCK；缺/过期 → unknown。
_REAL_POSES = ROOT / "tools/gate/out/policy_steps_2026-09-29_hr52/summary.json"  # hr38：嘴关节 J01 + H05 上头壳 + 喇叭/功放/UBEC 落位（改件，全链，新目录）  # hr37：H04 脸板 + 电子件占位实体（改件，全链）  # hr34：件同 hr33；assembly.yaml 改装序（F20）→ yaml 进指纹 → 重采  # hr32c：hr32 复审收尾改了 features/printability.yaml + 本文件（点名依赖）→ 重采作快车道基线；hr32b = Gate 终稿 ✅131 ❌383
_REAL_POSES_SCRIPT = ROOT / "tools/sim/policy_pose_collisions.py"
# hr50（2026-09-28）：真实姿态碰撞的判定口径（极限段 → 重训约束；其余姿态原版 p25 轻碰容差）。见文件头注释。
_REAL_POSE_POLICY = ROOT / "tools/gate/data/real_pose_policy.yaml"
_REAL_POSE_DEPTH_MODULE = ROOT / "tools/sim/pose_contact_depth.py"
_REAL_POSES_COMPARE_SCRIPT = ROOT / "tools/sim/policy_pose_compare.py"
# 重训后训练用的 MJCF 由 CAD 生成（tools/sim/make_mjcf.py），本层要查的是**这一份**
_GEN_MJCF = ROOT / "sim/duck_s288/robot_walk_s288.xml"
_GEN_TRAIN_MJCF = ROOT / "sim/duck_s288/robot_walk_s288_train.xml"   # 09-17：训练接口（脚底 geom 名 / 传感器 site）在训练变体里，09-16 起与全碰撞版分开
_GEN_ALLCOL_MJCF = ROOT / "sim/duck_s288/robot_walk_s288_allcol.xml"  # 其余六任务使用的正式变体也必须同步 BAM
# 09-17 摩擦同步：BAM m1 有两份落点（发布目录 / mjlab 包内），MJCF 的 chosen_actuator 是 make_mjcf 从发布目录那份烤进去的；
# 训练时 FrictionDRBamActuator 读包内那份做 DR 中心值 —— 三处必须是同一组数，否则"关节层摩擦"和"执行器层摩擦"各说各的。
_BAM_RELEASE = ROOT / "V2_S288版发布产物/04_电机到货测试/params/s288/m1.json"
_BAM_PKG = ROOT / "sim/duck_s288/mjlab_duck_s288/src/mjlab_duck_s288/params/s288_m1.json"
_BAM_FIELDS = (("friction_base", "joint", "frictionloss"), ("friction_viscous", "joint", "damping"),
               ("armature", "joint", "armature"), ("max_torque", "position", "forcerange"))
_BAM_REL_TOL = 2e-3                 # make_mjcf 写 6 位有效数字；相对差超过 0.2% 就不是同一组数
# 训练环境按名字抓 geom/site/body 的地方 —— 名字与正则从这里读，判据里不写死（元规则 7）
_ENV_CFG = ROOT / ("upstream/microduck_rl/src/mjlab_microduck/tasks/"
                   "microduck_velocity_env_cfg.py")

_STEP_DEG = 2.5                     # 单轴独立网格（checks.check_angles 的默认）
_COMBO_STEP_DEG = 5.0               # 两两组合统一网格（F-L6-1：不再 max(10°, 量程/12) 粗扫）
_GAP_SEARCH = 2.0                   # min_gap 的搜索半径（超过就返回这个值，不是真距离）
_TARGET_KEY = "frozen.yaml:joint_axes[].target_range_deg"
_CAP_KEY = "frozen.yaml:capability_envelope"
_TGT_TOL = 0.1                      # 目标区间与上游值的比较容差（上游弧度→度只保留 1 位小数）
_RANGES_OUT = ROOT / "tools/gate/out/mjcf_ranges.json"      # 单轴无碰撞区间导出（反例改指到临时文件）

# frozen.yaml 的切刀区间只写在一句自然语言里，标签 → 关节名的对应表只能写在层里。
# 数值一律从 data 解析，层里不写死角度。
_CUTTER_LABELS = [("髋俯仰", ["left_hip_pitch", "right_hip_pitch"]),
                  ("膝", ["left_knee", "right_knee"]),
                  ("踝", ["left_ankle", "right_ankle"]),
                  ("头俯仰", ["head_pitch"]),
                  ("颈摆", ["neck_pitch"])]


def _rel(p):
    try:
        return str(Path(p).relative_to(ROOT))
    except ValueError:
        return str(p)


# ── 场景：placed 世界坐标件 + MJCF 运动树 ────────────────────────────────
def check_scene_inventory(res, stems, mapping, snapshot_mapping, bodies):
    """An absent, extra or unowned solid must stop the scene, never vanish from it."""
    missing = sorted(set(stems) - set(mapping))
    extra = sorted(set(mapping) - set(stems))
    unknown = {n: b for n, b in mapping.items() if b not in bodies}
    changed = sorted(n for n in set(mapping) | set(snapshot_mapping)
                     if mapping.get(n) != snapshot_mapping.get(n))
    ok = bool(stems) and not (missing or extra or unknown or changed)
    res.add(subject="_scene", check="placed_inventory", state=PASS if ok else FAIL,
            severity=BLOCK, measured={"objects": len(stems), "missing": missing,
                                     "extra": extra, "unknown_bodies": unknown,
                                     "snapshot_mismatch": changed},
            criterion="每个导出实体恰有一个有效 MJCF 连杆归属，且与内容快照一致；空清单失败",
            evidence_n=len(stems),
            detail="只验证检查覆盖与归属声明；运动是否碰撞由逐姿态布尔另判。",
            provenance="parts.yaml:motion_instances / cad_geometry_manifest.json")
    return ok


class _Scene:
    def __init__(self, ctx, body_for_part):
        import numpy as np
        from assembly_audit import solid
        from duckstructure.lib import B, ORDER
        self.B, self.ORDER = B, ORDER
        self.index = {p.stem: p for p in sorted(PLACED.glob("*.stl"))}
        self.names = sorted(n for n in self.index if n in body_for_part)
        self.bodies = [body_for_part[n] for n in self.names]
        self.solids, corners = [], []
        # hr42：每个实体的附属数据按"实体对象"登记（id → (实体, …)），不按下标 —— 反例的 _NegScene 会在构造后按 keep 重排
        #   names/solids/corners，下标会错位，对象不会。_src[id] = (实体, 源网格内容摘要, 凸包顶点(零位世界系), 其 AABB lo, hi)
        self._src = {}
        import trimesh
        import check_cache as _CC
        for n in self.names:
            m = ctx.solid(self.index[n])                     # process=True，布尔前提
            s = solid(m)
            self.solids.append(s)
            try:
                hv = np.asarray(m.convex_hull.vertices, dtype=np.float64)
            except Exception:                                # noqa: BLE001  凸包算不出（退化件）→ 用全部顶点（包围关系照样成立）
                hv = np.asarray(m.vertices, dtype=np.float64)
            self._src[id(s)] = (s, _CC.mesh_digest(m), hv, hv.min(axis=0), hv.max(axis=0))
            c = trimesh.bounds.corners(m.bounds)
            corners.append(np.c_[c, np.ones(len(c))])
        self.corners = np.asarray(corners)
        self.booleans = 0
        self.gaps = 0
        # hr42 计数（evidence 用）：布尔/min_gap 被"凸包分离证明"跳过的次数、被本进程内完全相同输入的记忆命中的次数
        self.obb_bool = 0
        self.obb_gap = 0
        self.memo_bool = 0
        self.memo_gap = 0
        self.memo = None                                     # L6 层打开（{}）；采样 / audit_motion 的姿态两两不同，不开
        # 关节 → (body, 世界轴, 世界原点)。MJCF 里每条 joint 都是 axis="0 0 1" 且无 pos，
        # 所以轴 = 该 body 零位姿世界系的 z 轴、支点 = 该 body 原点（与 checks.run_checks 一致）。
        self.joint_body = {}
        for n in ORDER:
            j = B[n].get("joint")
            if j:
                self.joint_body[j["name"]] = n
        self.desc = {n: set(self._desc(n)) for n in ORDER}

    def _desc(self, n):
        out = [n]
        for c in self.B[n]["children"]:
            out += self._desc(c["name"])
        return out

    def deltas(self, pose_deg):
        """pose(关节→角度) → 每个 placed 实体的世界变换增量（相对零位姿）。
        按 MJCF 树正向合成，父连杆转了子连杆跟着转 —— 不做任何手工的"反向区间"换算。"""
        import numpy as np
        from trimesh.transformations import rotation_matrix as rot
        posed = {}
        for n in self.ORDER:
            d = self.B[n]
            T = d["T_parent"] if d["parent"] is None else posed[d["parent"]] @ d["T_parent"]
            j = d.get("joint")
            if j and abs(pose_deg.get(j["name"], 0.0)) > 1e-12:
                T = T @ rot(math.radians(float(pose_deg[j["name"]])), [0, 0, 1], [0, 0, 0])
            posed[n] = T
        inv = {n: posed[n] @ np.linalg.inv(self.B[n]["T_world"]) for n in self.ORDER}
        return np.asarray([inv[b] for b in self.bodies])

    def moving_pairs(self, active):
        """受这几条关节影响的实体对：两件之间的相对位姿会变，才有必要查。
        判据 = 至少有一条 active 关节把这两件分在树的两侧（异或）。"""
        sets = [self.desc[self.joint_body[j]] for j in active if j in self.joint_body]
        out = []
        for i, k in itertools.combinations(range(len(self.names)), 2):
            a, b = self.bodies[i], self.bodies[k]
            if a == b:
                continue                                     # 同一刚体 → 静态，归第 3 层
            if any((a in s) != (b in s) for s in sets):
                out.append((i, k))
        return out

    # ── hr42（2026-09-24，B 线）：实体附属数据 ────────────────────────────────
    def _aux(self, x):
        """实体 x 的 (源网格摘要, 凸包顶点, lo, hi)。调用方事后追加的实体（audit_motion 的螺丝头）没有源网格：
        摘要用 manifold 自己导出的 float64 网格算，凸包用 manifold.hull() 的顶点（凸包顶点 ⊂ 原顶点，包围关系精确）。"""
        s = self.solids[x]
        r = self._src.get(id(s))
        if r is None or r[0] is not s:
            import numpy as np
            import check_cache as _CC
            mm = s.to_mesh64()
            dig = _CC.array_digest(np.asarray(mm.vert_properties, dtype=np.float64), np.asarray(mm.tri_verts, dtype=np.int64))
            try:
                hv = np.asarray(s.hull().to_mesh64().vert_properties, dtype=np.float64)[:, :3]
            except Exception:                                # noqa: BLE001
                hv = np.asarray(mm.vert_properties, dtype=np.float64)[:, :3]
            r = (s, dig, hv, hv.min(axis=0), hv.max(axis=0))
            self._src[id(s)] = r
        return r[1:]

    def _digest(self, x):
        """hr42：实体 x 的内容摘要（检查原语缓存的键；tools/cad/check_cache.py）。"""
        return self._aux(x)[0]

    def _sep(self, i, k, mats, inv, wbox):
        """两实体之间距离的一个**下界**（mm；≤ 0 = 这一法分不开）：凸包顶点在 ① 世界系 ② i 的零位系 ③ k 的零位系下
        各做一次轴向区间分离，取最大。刚体变换保距、向单位轴投影 1-Lipschitz → 任一系里轴向分离 s ⇒ 两网格距离 ≥ s。"""
        _d, Hi, loi, hii = self._aux(i)
        _d, Hk, lok, hik = self._aux(k)
        for x, H in ((i, Hi), (k, Hk)):
            if x not in wbox:
                P = H @ mats[x, :3, :3].T + mats[x, :3, 3]
                wbox[x] = (P.min(axis=0), P.max(axis=0))
        (wli, whi), (wlk, whk) = wbox[i], wbox[k]
        s = float(max((wli - whk).max(), (wlk - whi).max()))
        if s > _OBB_EPS:
            return s
        R = inv[i] @ mats[k]                                 # k 的零位坐标 → i 的零位坐标
        P = Hk @ R[:3, :3].T + R[:3, 3]
        s = max(s, float((P.min(axis=0) - hii).max()), float((loi - P.max(axis=0)).max()))
        if s > _OBB_EPS:
            return s
        R = inv[k] @ mats[i]
        P = Hi @ R[:3, :3].T + R[:3, 3]
        return max(s, float((P.min(axis=0) - hik).max()), float((lok - P.max(axis=0)).max()))

    # ── hr42（2026-09-24）：并行预取 —— 层代码在进循环前把"马上要问的姿态"整批交给进程池算好，evaluate 查表命中就直接返回
    #    同一份 (hits, near, 布尔次数, min_gap 次数, 各 hr42 计数)，并照原样累加计数器；键 = 姿态 + 件对清单内容 + tol/gap_tol/mating 内容。
    #    查不到（没预取、或预取的姿态与实际问的不一样）就照原来现算 —— 预取只影响快慢，不影响结果与顺序。
    #    只对**原版 _Scene 类、没被打桩**的场景预取（反例的子类 / 打桩的 evaluate 看不到工作进程，那里一律现算）。
    def _pairs_digest(self, pairs):
        import hashlib
        c = getattr(self, "_pd_cache", None)
        if c is None:
            c = self._pd_cache = {}
        e = c.get(id(pairs))
        if e is not None and e[0] is pairs:
            return e[1]
        d = hashlib.blake2b(repr([tuple(x) for x in pairs]).encode(), digest_size=16).digest()
        if len(c) > 4096:
            c.clear()
        c[id(pairs)] = (pairs, d)
        return d

    def _pkey(self, pose_deg, pairs, tol, gap_tol, mating):
        return (tuple(sorted((str(j), float(a)) for j, a in pose_deg.items())), self._pairs_digest(pairs), float(tol),
                None if gap_tol is None else float(gap_tol),
                None if not mating else self._pairs_digest(sorted(mating)))

    def evaluate(self, pose_deg, pairs, tol, gap_tol=None, mating=frozenset()):
        pref = getattr(self, "_pref", None)
        if pref:
            r = pref.get(self._pkey(pose_deg, pairs, tol, gap_tol, mating))
            if r is not None:
                hits, near, nb, ng, cnt = r
                self.booleans += nb
                self.gaps += ng
                self.obb_bool += cnt[0]; self.obb_gap += cnt[1]; self.memo_bool += cnt[2]; self.memo_gap += cnt[3]
                return list(hits), list(near), nb
        return self._evaluate(pose_deg, pairs, tol, gap_tol, mating)

    def prefetch(self, reqs, bfp):
        """reqs = [(pose, pairs, tol, gap_tol, mating)]。按 L6_JOBS（默认 4）个进程并行算好放进预取表；L6_JOBS=1 不预取。"""
        import os
        jobs = int(os.environ.get("L6_JOBS", "4")) if os.environ.get("L6_JOBS", "4").isdigit() else 4
        if jobs <= 1 or not reqs:
            return 0
        if type(self) is not _SCENE_BASE or _SCENE_BASE._evaluate is not _ORIG["_evaluate"] \
                or _SCENE_BASE.deltas is not _ORIG["deltas"] or "deltas" in vars(self) or "_evaluate" in vars(self) \
                or PLACED != _ORIG["PLACED"]:
            return 0                                         # 子类 / 打桩 / 换了 placed 目录：工作进程看不到，不预取
        if getattr(self, "_pref", None) is None:
            self._pref = {}
        groups, seen = {}, set()
        for pose, pairs, tol, gap_tol, mating in reqs:
            k = self._pkey(pose, pairs, tol, gap_tol, mating)
            if k in self._pref or k in seen:
                continue
            seen.add(k)
            gk = (k[1], k[2], k[3], k[4])
            g = groups.setdefault(gk, dict(pairs=list(pairs), tol=tol, gap_tol=gap_tol, mating=frozenset(mating or ()), items=[]))
            g["items"].append((k, dict(pose)))
        n = sum(len(g["items"]) for g in groups.values())
        if not n:
            return 0
        size = max(1, -(-n // (jobs * 6)))
        chunks = []
        for g in groups.values():
            it = g["items"]
            for i in range(0, len(it), size):
                chunks.append((g["pairs"], g["tol"], g["gap_tol"], g["mating"], it[i:i + size]))
        ex = _pref_pool(jobs, bfp)
        for res in ex.map(_pref_eval, chunks):
            for k, hits, near, nb, ng, cnt in res:
                self._pref[k] = (hits, near, nb, ng, cnt)
        return n

    def _evaluate(self, pose_deg, pairs, tol, gap_tol=None, mating=frozenset()):
        """一个姿态下的交集与最小距离。布尔失败抛 ValueError（不吞成 0）。
        `mating` 是 relations.yaml 里**声明为 contact** 的件对（配合面），它们的间隙归第 3/5 层，本层不重复报
        （F-L6-2：不再按"零位姿贴得近"自动豁免 —— 那个集合由 zero_gap_pairs 给，只用来列 undeclared_zero_gap_pair）。
        gap_tol=None 时不算最小距离（两两组合扫描只判交集）。

        hr42（B 线，结果与改前逐字节相同，只省计算）：
          · AABB 粗筛照旧；过了粗筛的对先用凸包做"分离证明"（_sep：三个坐标系里的轴向区间分离取最大，是两网格距离的下界）：
            分离 > 1e-6 mm ⇒ 两网格隔着正距离 ⇒ manifold 求交必为空、体积恰为 0.0 → 不做布尔（计 obb_bool，不计入 booleans）；
            还要算 min_gap 的对：分离 ≥ gap_tol ⇒ min_gap ≥ gap_tol ⇒ 进不了 near → 不做 min_gap（计 obb_gap，不计入 gaps）。
          · self.memo（L6 层打开）：本进程里 (i, k, 两个位姿矩阵的原始字节) 完全相同的布尔 / min_gap 只算一次
            （组合扫描里"两件都在子关节上游"的对，每个子关节角都是同一对矩阵）；命中照样计入 booleans / gaps（逻辑次数不变），另计 memo_*。
          · 检查原语缓存（A 线 check_cache，DUCK_CHECK_CACHE=0 关）照旧。
          DUCK_L6_OBB=0 关分离证明（对账用）。"""
        import numpy as np
        import manifold3d as M
        mats = self.deltas(pose_deg)
        cor = np.einsum("nij,nkj->nki", mats, self.corners)[..., :3]
        lo, hi = cor.min(axis=1), cor.max(axis=1)
        moved, hits, near, nb = {}, [], [], 0
        import check_cache as _CC                            # hr42：检查原语缓存（键 = 两实体源网格摘要 + 两个位姿矩阵字节 + 库版本）
        use_cc = _CC.enabled()
        obb = _obb_on()
        memo = self.memo
        inv, wbox = None, {}

        def _mv(x):
            if x not in moved:
                moved[x] = self.solids[x].transform(mats[x, :3])
            return moved[x]
        for i, k in pairs:
            sep = np.maximum(lo[i] - hi[k], lo[k] - hi[i]).max()
            want_gap = gap_tol is not None and (i, k) not in mating
            if sep > (gap_tol if want_gap else 0.0):
                continue                                     # AABB 粗筛
            s_obb = None
            if obb:
                if inv is None:
                    inv = _rigid_inv(mats)
                s_obb = self._sep(i, k, mats, inv, wbox)
            mk = None
            if s_obb is not None and s_obb > _OBB_EPS:
                v = 0.0                                      # 凸包分离证明：交集必空（见 docstring）
                self.obb_bool += 1
            else:
                nb += 1
                if memo is not None:
                    mk = (i, k, mats[i, :3].tobytes(), mats[k, :3].tobytes())
                v = memo.get(mk) if mk is not None else None
                if v is not None:
                    self.memo_bool += 1
                else:
                    if use_cc:
                        _CC.set_context(f"l6:{self.names[i]}×{self.names[k]}")
                    ck = _CC.key("l6.inter", self._digest(i), mats[i, :3], self._digest(k), mats[k, :3]) if use_cc else None
                    hit, v = _CC.get("l6", ck) if use_cc else (False, None)
                    if not hit:
                        it = _mv(i) ^ _mv(k)
                        v = float(it.volume())
                        if it.status() != M.Error.NoError or not math.isfinite(v) or v < -1e-6:
                            raise ValueError(f"布尔失败 {self.names[i]}×{self.names[k]}: {it.status()} v={v}")
                        if use_cc:
                            _CC.put("l6", ck, v)
                    if mk is not None:
                        memo[mk] = v
            if v > tol:
                hits.append((self.names[i], self.names[k], v))
            elif want_gap and sep <= gap_tol:
                if s_obb is not None and s_obb >= gap_tol + _GAP_EPS:
                    self.obb_gap += 1                        # 距离下界 ≥ 阈值：min_gap ≥ 阈值，进不了 near
                    continue
                gm = (mk if mk is not None else (i, k, mats[i, :3].tobytes(), mats[k, :3].tobytes())) + ("gap",) \
                    if memo is not None else None
                g = memo.get(gm) if gm is not None else None
                if g is not None:
                    self.memo_gap += 1
                else:
                    gk = _CC.key("l6.min_gap", self._digest(i), mats[i, :3], self._digest(k), mats[k, :3], float(_GAP_SEARCH)) if use_cc else None
                    ghit, g = _CC.get("l6", gk) if use_cc else (False, None)
                    if not ghit:
                        g = float(_mv(i).min_gap(_mv(k), _GAP_SEARCH))
                        if use_cc:
                            _CC.put("l6", gk, g)
                    if gm is not None:
                        memo[gm] = g
                self.gaps += 1
                if 0.0 <= g < gap_tol:
                    near.append((self.names[i], self.names[k], g))
        self.booleans += nb
        return hits, near, nb

    def hr42_counters(self):
        """hr42：写进 evidence 的实现层计数（结果无关）。"""
        return {"booleans_obb_prefiltered": self.obb_bool, "min_gap_obb_prefiltered": self.obb_gap,
                "booleans_memo_hits": self.memo_bool, "min_gap_memo_hits": self.memo_gap}

    def zero_gap_pairs(self, pairs, gap_tol):
        """零位姿就已经贴得近（间隙 < 阈值）的件对 —— 只是**几何事实**，不是豁免依据（F-L6-2）。
        舵机坐进笼子、轴承压进座这些本来就该贴着的面必须在 relations.yaml 里声明为 contact 才豁免；
        这里量出来但没声明的对，由 run() 发 _zero:undeclared_zero_gap_pair 列出来。"""
        import numpy as np
        mats = self.deltas({})
        cor = np.einsum("nij,nkj->nki", mats, self.corners)[..., :3]
        lo, hi = cor.min(axis=1), cor.max(axis=1)
        out = set()
        for i, k in pairs:
            if np.maximum(lo[i] - hi[k], lo[k] - hi[i]).max() > gap_tol:
                continue
            g = float(self.solids[i].min_gap(self.solids[k], _GAP_SEARCH))
            self.gaps += 1
            if g < gap_tol:
                out.add((i, k))
        return out


# hr42：分离证明的余量与开关
_OBB_EPS = 1e-6          # 只有分离 > 1e-6 mm 才跳过布尔（远大于位姿矩阵 / 顶点变换的浮点误差 ~1e-13 mm，远小于任何判据阈值）
_GAP_EPS = 1e-9


def _obb_on():
    import os
    return os.environ.get("DUCK_L6_OBB", "1") not in ("0", "off", "no", "false")


def _memo_on():
    import os
    return os.environ.get("DUCK_L6_MEMO", "1") not in ("0", "off", "no", "false")


def _rigid_inv(mats):
    """一批刚体 4×4 的逆：[Rᵀ, −Rᵀt]（deltas 是旋转 × 平移的乘积）。"""
    import numpy as np
    inv = np.zeros_like(mats)
    rt = np.transpose(mats[:, :3, :3], (0, 2, 1))
    inv[:, :3, :3] = rt
    inv[:, :3, 3] = -np.einsum("nij,nj->ni", rt, mats[:, :3, 3])
    inv[:, 3, 3] = 1.0
    return inv


_SCENE_BASE = _Scene
_ORIG = {"_evaluate": _Scene._evaluate, "deltas": _Scene.deltas, "PLACED": PLACED}


# ── hr42：L6 并行预取的进程池（spawn；每个工作进程自己按同一 placed/ 与 body_for_part 建场景）──────────────
_PREF = {"pool": None, "key": None, "stats_dir": None}
_PREF_W = {}


def _pref_init(placed_dir, bfp):
    global PLACED
    PLACED = Path(placed_dir)
    from core import load_data
    from gate import Ctx
    ctx = Ctx(load_data())
    sc = _Scene(ctx, bfp)
    ctx._mesh.clear()
    if _memo_on():
        sc.memo = {}
    _PREF_W["sc"] = sc


def _pref_eval(chunk):
    pairs, tol, gap_tol, mating, items = chunk
    sc = _PREF_W["sc"]
    out = []
    for k, pose in items:
        g0, c0 = sc.gaps, (sc.obb_bool, sc.obb_gap, sc.memo_bool, sc.memo_gap)
        hits, near, nb = sc._evaluate(pose, pairs, tol, gap_tol, mating)
        c1 = (sc.obb_bool, sc.obb_gap, sc.memo_bool, sc.memo_gap)
        out.append((k, hits, near, nb, sc.gaps - g0, tuple(b - a for a, b in zip(c0, c1))))
    import check_cache as _CC
    _CC.worker_flush()
    return out


def _pref_pool(jobs, bfp):
    import json as _json, multiprocessing as mp, os, tempfile
    from concurrent.futures import ProcessPoolExecutor
    key = (jobs, str(PLACED), _json.dumps(bfp, sort_keys=True))
    if _PREF["pool"] is not None and _PREF["key"] == key:
        return _PREF["pool"]
    _pref_close()
    _PREF["stats_dir"] = tempfile.mkdtemp(prefix="hr42_l6stats_")
    old = os.environ.get("DUCK_CHECK_CACHE_STATS_DIR")
    os.environ["DUCK_CHECK_CACHE_STATS_DIR"] = _PREF["stats_dir"]        # 工作进程继承：原语缓存统计写这里
    try:
        _PREF["pool"] = ProcessPoolExecutor(max_workers=jobs, mp_context=mp.get_context("spawn"),
                                            initializer=_pref_init, initargs=(str(PLACED), bfp))
    finally:
        if old is None:
            os.environ.pop("DUCK_CHECK_CACHE_STATS_DIR", None)
        else:
            os.environ["DUCK_CHECK_CACHE_STATS_DIR"] = old
    _PREF["key"] = key
    return _PREF["pool"]


def _pref_close():
    """关池，并把工作进程的原语缓存统计并进本进程（层的 evidence.check_cache 才完整）。"""
    ex = _PREF.get("pool")
    if ex is not None:
        ex.shutdown(wait=True)
    _PREF["pool"] = None; _PREF["key"] = None
    d = _PREF.get("stats_dir")
    if d:
        import check_cache as _CC
        _CC.absorb_stats(_CC.collect_stats(d))
        _PREF["stats_dir"] = None


# ── data 解析 ─────────────────────────────────────────────────────────────
def _cutter_ranges(ctx, legacy):
    """frozen.yaml:joint_axes_rule.cutter_vs_mjcf_warning 那句话 → 每个关节的切刀区间。
    数值全部从 data 解析；标签→关节名的对应表在 _CUTTER_LABELS（层里只能有这个）。

    **符号陷阱**：那句话里的数字有的是关节角区间（颈摆 -60..45），有的是**反向扫掠**的参数
    （膝 -62..92 —— 小腿动 −90..+60 时大腿在小腿坐标里是 −60..+90，区间交换并取负）。
    所以每条都取 (lo,hi) 与 (-hi,-lo) 两个候选，用 duckstructure/checks.py:LEGACY_LIMITS
    做裁判取近的那个，并把选择过程写进证据 —— 不靠人记住哪条要反。"""
    txt = str(((ctx.data.get("frozen") or {}).get("joint_axes_rule") or {})
              .get("cutter_vs_mjcf_warning") or "")
    out, parsed = {}, []
    for label, joints in _CUTTER_LABELS:
        m = re.search(re.escape(label) + r"[^/]*?(±\s*([0-9.]+)|(-?[0-9.]+)\s*\.\.\s*(-?[0-9.]+))", txt)
        if not m:
            continue
        if m.group(2):
            lo, hi = -float(m.group(2)), float(m.group(2))
        else:
            lo, hi = float(m.group(3)), float(m.group(4))
        for j in joints:
            cands = [(lo, hi), (-hi, -lo)]
            if j in legacy:
                lg = tuple(float(x) for x in legacy[j])
                pick = min(cands, key=lambda c: max(abs(c[0] - lg[0]), abs(c[1] - lg[1])))
                tag = "原样" if pick == cands[0] else "**交换并取负**（反向扫掠）"
                parsed.append(f"{label}({j}) {lo}..{hi} → {pick[0]}..{pick[1]} {tag}"
                              f"，LEGACY {lg} 差 "
                              f"{max(abs(pick[0] - lg[0]), abs(pick[1] - lg[1])):.3g}°")
            else:
                pick = cands[0]
                parsed.append(f"{label}({j}) {lo}..{hi} 原样（LEGACY_LIMITS 里没有这条，无从裁判）")
            out[j] = pick
    return out, txt, parsed


def _strip_py_comments(txt):
    """去掉 Python 行注释（跳过字符串里的 #）。上游配置里 pattern= 和 entity= 中间夹着行尾注释，
    不去掉的话『按 kwargs 顺序写的正则』会漏掉 entity，把该查的要求整条丢掉。"""
    out = []
    for line in txt.splitlines():
        q, i = None, 0
        while i < len(line):
            c = line[i]
            if q:
                if c == "\\":
                    i += 1
                elif c == q:
                    q = None
            elif c in "\"'":
                q = c
            elif c == "#":
                line = line[:i]
                break
            i += 1
        out.append(line)
    return "\n".join(out)


def _balanced(txt, start):
    """txt[start] 必须是 '('，返回配对右括号之间的内容（找不到返回 None）。"""
    depth = 0
    for i in range(start, len(txt)):
        if txt[i] == "(":
            depth += 1
        elif txt[i] == ")":
            depth -= 1
            if depth == 0:
                return txt[start + 1:i]
    return None


def _train_interface(txt):
    """上游训练环境配置源码 → 它按**名字**抓的东西。
    返回 {'geom': [(出处, 正则)], 'site': [...], 'body': [...]}。
    这里只解析、不判断；判断在 run() 里对着生成的 MJCF 做。
    **一个名字都不许写死在本层**（元规则 7）—— 解析不出来就报 unknown。"""
    src = _strip_py_comments(txt)
    req = {"geom": [], "site": [], "body": []}
    for mo in re.finditer(r"ContactMatch\s*\(", src):
        blk = _balanced(src, mo.end() - 1)
        if blk is None:
            continue
        kw = dict(re.findall(r'(\w+)\s*=\s*r?"([^"]*)"', blk))
        if kw.get("entity") != "robot":
            continue                    # secondary=terrain 之类不是机器人模型该提供的名字
        mode, pat = kw.get("mode"), kw.get("pattern")
        if not pat:
            continue
        if mode == "geom":
            req["geom"].append((f"ContactMatch(mode={mode}).pattern", pat))
        elif mode in ("body", "subtree"):
            req["body"].append((f"ContactMatch(mode={mode}).pattern", pat))
    for var, kind in (("foot_frictions_geom_names", "geom"), ("site_names", "site")):
        mo = re.search(re.escape(var) + r"\s*=\s*[\(\[]([^)\]]*)[)\]]", src)
        if mo:
            for nm in re.findall(r'"([^"]+)"', mo.group(1)):
                req[kind].append((var, "^" + re.escape(nm) + "$"))
    # 同一条要求会出现两次（一个传感器的 primary + secondary 都指 trunk_base）—— 去重，
    # 免得记分卡上同一句话打印两遍
    for k, v in req.items():
        req[k] = list(dict.fromkeys(v))
    return req


def _policy_box(env, key="actual_deg"):
    """policy_envelope.json → 每个关节策略实际用到的角度区间（各 mode 求并）。
    key='actual_deg' 是**实际走到**的角度；'target_deg' 是发出去的指令（含被限位夹掉的超程）。"""
    box, per_mode = {}, {}
    for mode, joints in (env.get("policy_envelope") or {}).items():
        for j, w in (joints or {}).items():
            t = ((w or {}).get("all") or {}).get(key) or {}
            lo, hi = t.get("min"), t.get("max")
            if lo is None or hi is None:
                continue
            per_mode.setdefault(mode, {})[j] = (float(lo), float(hi))
            b = box.get(j)
            box[j] = (min(float(lo), b[0]), max(float(hi), b[1])) if b else (float(lo), float(hi))
    return box, per_mode


def _target_range(j, mj):
    """frozen.yaml:joint_axes[] 一条 → ((lo, hi), None) 或 (None, 为什么不能用)。mj = 上游扫描域 [lo, hi]（度）。
    目标区间**只认**这个字段（F-L6-3）：缺 / 不是 [lo, hi] / 非数 / lo ≥ hi / 超出扫描域 / 人为收窄（≠ 上游值）却没有
    date + reason → 不能用 → 该关节相关判据 unknown。"""
    t = j.get("target_range_deg")
    if not isinstance(t, dict) or "v" not in t:
        return None, f"缺 target_range_deg（{_TARGET_KEY}）"
    v = t.get("v")
    if not (isinstance(v, (list, tuple)) and len(v) == 2):
        return None, f"target_range_deg.v={v!r} 不是 [lo, hi]（{_TARGET_KEY}）"
    try:
        lo, hi = float(v[0]), float(v[1])
    except (TypeError, ValueError):
        return None, f"target_range_deg.v={v!r} 不是数（{_TARGET_KEY}）"
    if isinstance(v[0], bool) or isinstance(v[1], bool) or not (math.isfinite(lo) and math.isfinite(hi)) or not lo < hi:
        return None, f"target_range_deg.v={v!r} 不是有序的 [lo, hi]（lo ≥ hi 或非有限数；{_TARGET_KEY}）"
    if lo < mj[0] - _TGT_TOL or hi > mj[1] + _TGT_TOL:
        return None, (f"target_range_deg.v={v!r} 超出上游扫描域 [{mj[0]:.3f}, {mj[1]:.3f}] —— "
                      f"超出的那段根本没扫过，分不出区间内/外（{_TARGET_KEY}）")
    narrowed = abs(lo - mj[0]) > _TGT_TOL or abs(hi - mj[1]) > _TGT_TOL
    if narrowed and not (str(t.get("date") or "").strip() and str(t.get("reason") or "").strip()):
        return None, (f"target_range_deg.v={v!r} ≠ 上游值 [{mj[0]:.1f}, {mj[1]:.1f}]（人为收窄）但没有 date + reason"
                      f"（{_TARGET_KEY}）")
    return (lo, hi), None


def _mjcf_joint_ranges_deg(path):
    """一份 MJCF 的每条 <joint name range> → {name: (lo_deg, hi_deg)}。解析不了就抛，调用方判 unknown。"""
    import xml.etree.ElementTree as ET
    root = ET.parse(str(path)).getroot()
    out = {}
    for j in root.iter("joint"):
        nm, rg = j.get("name"), j.get("range")
        if nm and rg:
            lo, hi = (float(x) for x in rg.split())
            out[nm] = (math.degrees(lo), math.degrees(hi))
    return out


def _contact_pairs(relations, name_idx, bodies, stem2pid):
    """relations.yaml 里声明为 contact 的 parties 件对 → 场景里的 (i, k) 索引对（F-L6-2：豁免只认声明）。
    parties 一方给 `placed`（实体名）就认那一只；只给 `part`（件号）就认该件号的全部实例（parts.yaml 桥接）。
    返回 (索引对集合, 解析到的声明数, 解析不到的 [(关系 id, 原因)…])。"""
    def stems(p):
        if not isinstance(p, dict):
            return [], f"parties 里这一项不是 dict：{str(p)[:40]}"
        if p.get("placed"):
            return ([p["placed"]] if p["placed"] in name_idx else []), (
                "" if p["placed"] in name_idx else f"placed/{p['placed']} 不在运动场景里")
        if p.get("part"):
            got = [s for s, pid in stem2pid.items() if pid == p["part"] and s in name_idx]
            return got, ("" if got else f"件 {p['part']} 在运动场景里认不出实体")
        return [], f"这一方既没有 part 也没有 placed：{sorted(p)}"

    out, n_ok, bad = set(), 0, []
    for rel in relations or []:
        c = rel.get("contact") if isinstance(rel, dict) else None
        if not isinstance(c, dict):
            continue
        parties = c.get("parties")
        if not (isinstance(parties, list) and len(parties) >= 2):
            bad.append((rel.get("id"), "contact.parties 不是 ≥2 项的列表"))
            continue
        sa, wa = stems(parties[0])
        sb, wb = stems(parties[1])
        if not sa or not sb:
            bad.append((rel.get("id"), (wa or "") + (" " if wa and wb else "") + (wb or "")))
            continue
        n_ok += 1
        for a in sa:
            for b in sb:
                i, k = name_idx[a], name_idx[b]
                if i != k and bodies[i] != bodies[k]:
                    out.add((min(i, k), max(i, k)))
    return out, n_ok, bad



def _placed_fingerprint():
    import hashlib
    h = hashlib.sha256()
    for f in sorted(PLACED.glob("*.stl")):
        h.update(f.name.encode()); h.update(hashlib.sha256(f.read_bytes()).digest())
    return h.hexdigest()


def _load_real_pose_policy():
    """tools/gate/data/real_pose_policy.yaml → dict(segs={(mode, case)}, v_max, d_max, orig_i={件对}, why={(mode, case): 说明})。坏了就抛。"""
    import yaml
    p = yaml.safe_load(_REAL_POSE_POLICY.read_text(encoding="utf-8"))
    segs, why = set(), {}
    for r in p.get("retrain_segments") or []:
        k = (str(r["mode"]), str(r["case"]))
        segs.add(k); why[k] = str(r.get("why") or "")
    lc = p["light_contact"]
    v, d = float(lc["v_max_mm3"]), float(lc["d_max_mm"])
    if not (math.isfinite(v) and math.isfinite(d) and v >= 0 and d >= 0):
        raise ValueError(f"light_contact 阈值无效：{lc}")
    pc = p.get("pair_combination") or {}
    combo = None
    if pc:
        r = float(pc["reach_deg"])
        if not (math.isfinite(r) and r >= _COMBO_STEP_DEG / 2):     # 小于网格半步 → 有真实样本落不进任何网格点的邻域（覆盖漏洞）
            raise ValueError(f"pair_combination.reach_deg 无效（须 ≥ 组合网格半步 {_COMBO_STEP_DEG / 2}°）：{pc}")
        combo = dict(reach=r, excl=bool(pc.get("exclude_retrain_segments", True)))
    appr = []
    for i, e in enumerate(p.get("approved_retrain_constraints") or []):
        try:
            j = [str(x) for x in e["joints"]]; pr = str(e["pair"]).split("×")
            q1 = [float(x) for x in e["q1"]]; q2 = [float(x) for x in e["q2"]]
            cap = float(e["approved_up_to_mm3"])
            if (len(j) != 2 or len(pr) != 2 or len(q1) != 2 or len(q2) != 2 or q1[0] > q1[1] or q2[0] > q2[1]
                    or not (math.isfinite(cap) and cap >= 0) or not e.get("date") or not e.get("decided_by") or not e.get("why")):
                raise ValueError("字段缺失 / 区间无序 / 上限无效（要 joints[2]、pair a×b、q1[2]、q2[2]、approved_up_to_mm3、date、decided_by、why）")
        except (KeyError, TypeError, ValueError, AttributeError) as ex:
            raise ValueError(f"approved_retrain_constraints[{i}]（{(e.get('id') if isinstance(e, dict) else e)!r}）无效：{ex}")
        appr.append(dict(id=str(e.get("id") or f"ARC#{i}"), joints=j, pair=frozenset(pr), q1=q1, q2=q2, cap=cap,
                         date=str(e["date"]), by=str(e["decided_by"]), why=str(e["why"]), evidence=str(e.get("evidence") or "")))
    ids = [a["id"] for a in appr]
    if len(set(ids)) != len(ids):
        raise ValueError(f"approved_retrain_constraints 的 id 重复：{ids}")
    # hr50 v6（2026-09-28 用户定 C）：策略包络角点姿态记录按 pair_combination 同一口径分类；缺段 → None（层判 unknown），写错 → 口径文件无效
    pep, pe = None, p.get("policy_extreme_poses")
    if pe is not None:
        if not isinstance(pe, dict) or pe.get("classify") != "pair_combination" or not combo:
            raise ValueError(f"policy_extreme_poses 段无效（只认 classify: pair_combination，且要有 pair_combination 段）：{pe!r}")
        pep = dict(classify="pair_combination", reached=str(pe.get("reached") or ""), decided=str(pe.get("decided") or ""))
    return dict(segs=segs, v_max=v, d_max=d, orig_i=set(p.get("orig_modeling_pairs") or []), why=why, combo=combo, approved=appr, pep=pep)


class _DepthScene:
    """真实姿态接触深度用：按需载入 placed 件（process=True 网格 + manifold 实体），位姿用 _Scene.deltas（同一套 MJCF 树）。"""
    def __init__(self, body_for_part):
        from duckstructure.lib import B, ORDER
        self.B, self.ORDER, self.bfp, self.cache = B, ORDER, body_for_part, {}

    def part(self, name):
        if name not in self.cache:
            import numpy as np
            import trimesh
            from assembly_audit import solid
            m = trimesh.load(str(PLACED / f"{name}.stl"), process=True)
            self.cache[name] = (np.asarray(m.triangles), solid(m))
        return self.cache[name]

    def depth(self, a, b, pose_deg):
        import types
        import pose_contact_depth as PCD
        ta, sa = self.part(a); tb, sb = self.part(b)
        ns = types.SimpleNamespace(B=self.B, ORDER=self.ORDER, bodies=[self.bfp[a], self.bfp[b]])
        M = _Scene.deltas(ns, pose_deg)
        return PCD.contact_depth(ta, sa, M[0], tb, sb, M[1])


def _light_contact_depth(scene, solid_pairs, pose_deg):
    """一个姿态上一个 body 对的穿插深度（mm）= 该 body 对各件对深度的最大值；逐件对明细一起返回。算不出就抛（调用方按超标处理）。"""
    rows, dmax = [], 0.0
    for sp in sorted(solid_pairs):
        a, b = sp.split("×")
        r = scene.depth(a, b, pose_deg)
        rows.append(dict(pair=sp, vol_mm3=round(r["vol_mm3"], 4), depth_mm=round(r["depth_mm"], 4), crop_limited=r.get("crop_limited")))
        dmax = max(dmax, float(r["depth_mm"]))
    return dmax, rows


def _key_joints(pose_deg, ranges, n=4, near_deg=3.0):
    """最能说明这一帧的关节：离 MJCF 限位 ≤ near_deg 的（标"到头"）优先，其余按 |角度| 大到小，共 n 个。"""
    items = []
    for j, a in pose_deg.items():
        lo, hi = ranges.get(j, (None, None))
        at = lo is not None and (a - lo <= near_deg or hi - a <= near_deg)
        items.append((0 if at else 1, -abs(a), j, a, at))
    items.sort()
    return [f"{j} {a:+.0f}°" + ("（到头）" if at else "") for _, _, j, a, at in items[:n]]


def _classify_real_poses(per_pose, pol, ours_bfp):
    """hr50：逐姿态、逐 body 对分三类。per_pose = policy_pose_compare.load(...) 的结果。
    返回 (retrain, light_c, viol, depth_scene)：极限段 → retrain；其余 ≤ p25（体积且深度）→ light_c；其余 → viol（深度算不出也进 viol）。"""
    retrain, light_c, viol = {}, {}, {}
    depth_scene = None
    for key in sorted(per_pose):
        po, pose = per_pose[key]
        seg = (key[0], key[1])
        for k, d in po.items():
            if seg in pol["segs"]:
                s_ = retrain.setdefault(k, dict(poses=0, max_mm3=0.0, segs={}, parts=set(), max_key=None, max_pose=None, solid_pairs=None))
                s_["poses"] += 1; s_["segs"][f"{seg[0]}/{seg[1]}"] = s_["segs"].get(f"{seg[0]}/{seg[1]}", 0) + 1
                s_["parts"] |= d["parts"]
                if d["v"] > s_["max_mm3"]:
                    s_.update(max_mm3=d["v"], max_key=key, max_pose=pose, solid_pairs=sorted(d["solid_pairs"]))
                continue
            why = None
            if d["v"] <= pol["v_max"]:
                try:
                    if depth_scene is None:
                        depth_scene = _DepthScene(ours_bfp)
                    dep, rows = _light_contact_depth(depth_scene, d["solid_pairs"], pose)
                except Exception as e:                                       # noqa: BLE001
                    dep, rows, why = None, [], f"深度算不出（{e!r}）→ 按超标处理"
                if dep is not None and dep <= pol["d_max"]:
                    s_ = light_c.setdefault(k, dict(poses=0, max_mm3=0.0, max_depth_mm=0.0, segs={}, example=None))
                    s_["poses"] += 1; s_["segs"][f"{seg[0]}/{seg[1]}"] = s_["segs"].get(f"{seg[0]}/{seg[1]}", 0) + 1
                    s_["max_mm3"] = max(s_["max_mm3"], d["v"])
                    if dep >= s_["max_depth_mm"]:
                        s_.update(max_depth_mm=dep, example=dict(mode=key[0], case=key[1], step=key[2], rows=rows))
                    continue
                if why is None:
                    why = f"交集 {d['v']:.3f} ≤ {pol['v_max']} 但深度 {dep:.3f} mm > {pol['d_max']}"
            else:
                why = f"交集 {d['v']:.3f} mm³ > {pol['v_max']}"
            s_ = viol.setdefault(k, dict(poses=0, max_mm3=0.0, segs={}, parts=set(), example=None, why=None))
            s_["poses"] += 1; s_["segs"][f"{seg[0]}/{seg[1]}"] = s_["segs"].get(f"{seg[0]}/{seg[1]}", 0) + 1
            s_["parts"] |= d["parts"]
            if d["v"] >= s_["max_mm3"]:
                s_.update(max_mm3=d["v"], example=dict(mode=key[0], case=key[1], step=key[2]), why=why)
    return retrain, light_c, viol, depth_scene


def _combo_envelope(cap, pol):
    """hr50 v4：pair_combination 的"真实动作包络"样本。cap = frozen.yaml:capability_envelope。
    样本 = pose_steps_source（sha 必须与声明一致）里 modes_in_scope 各段前 prefall_n 步，去掉 retrain_segments 三段极限段；
    mirror_symmetric 时按 mirror_pairs（左 ≡ sign × 右，与 derive_target_ranges._mirror 同一约定）把左右镜像样本并进来。
    返回 (joint_names, S[N,14] 度, 说明 dict)。来源缺失 / sha 不对 / mode 缺 / 镜像声明坏 → 抛 ValueError（调用方判 unknown）。"""
    import hashlib
    import numpy as np
    if not pol.get("combo"):
        raise ValueError(f"{_rel(_REAL_POSE_POLICY)} 没有 pair_combination 段")
    src = (cap or {}).get("pose_steps_source") or {}
    path = ROOT / str(src.get("path") or "")
    if not src.get("path") or not path.is_file():
        raise ValueError(f"capability_envelope.pose_steps_source.path 不存在：{src.get('path')}")
    sha = hashlib.sha256(path.read_bytes()).hexdigest()
    if sha != str(src.get("sha256")):
        raise ValueError(f"{_rel(path)} sha256 {sha[:16]} 与声明 {str(src.get('sha256'))[:16]} 不一致")
    modes = list((cap or {}).get("modes_in_scope") or [])
    if not modes:
        raise ValueError("capability_envelope.modes_in_scope 为空")
    d = np.load(str(path), allow_pickle=False)
    names = [str(x) for x in d["joint_names"]]
    rows, used, skipped = [], [], []
    for k in d.files:
        if not k.endswith("/qpos_deg"):
            continue
        mode, case = k.split("/")[:2]
        if mode not in modes:
            continue
        if pol["combo"]["excl"] and (mode, case) in pol["segs"]:
            skipped.append(f"{mode}/{case}"); continue
        n = int(d[f"{mode}/{case}/prefall_n"])
        rows.append(np.asarray(d[k][:n], dtype=np.float64)); used.append(f"{mode}/{case}")
    missing = [m for m in modes if not any(u.startswith(m + "/") for u in used + skipped)]   # 只有极限段的 mode（roulade）= 有数据、按口径去掉
    if missing:
        raise ValueError(f"modes_in_scope 这些 mode 在 {_rel(path)} 里没有样本：{missing}")
    S = np.vstack(rows)
    n_raw, mir = len(S), []
    if cap.get("mirror_symmetric"):
        pairs = cap.get("mirror_pairs") or []
        if not pairs:
            raise ValueError("capability_envelope.mirror_symmetric = true 但没有 mirror_pairs")
        M = S.copy()
        for left, right, sign in pairs:
            if left not in names or right not in names:
                raise ValueError(f"mirror_pairs 的关节不在 {_rel(path)} 里：{left} / {right}")
            il, ir, sg = names.index(left), names.index(right), float(sign)
            M[:, il] = sg * S[:, ir]
            M[:, ir] = sg * S[:, il]
            mir.append(f"{left}↔{right}（sign {sg:+.0f}）")
        S = np.vstack([S, M])
    return names, S, dict(path=_rel(path), sha=sha[:16], n=len(S), n_raw=n_raw, cases=len(used), skipped=skipped, mirror=mir)


def _combo_reached(env, j1, a, j2, b, reach):
    """(a, b) 是否"到过"：有样本在两轴上都与之相差 ≤ reach（切比雪夫邻域）。关节不在记录里 → 抛 ValueError（不猜）。"""
    import numpy as np
    names, S, _ = env
    if j1 not in names or j2 not in names:
        raise ValueError(f"真实动作记录里没有关节 {[j for j in (j1, j2) if j not in names]}")
    i1, i2 = names.index(j1), names.index(j2)
    return bool(np.any((np.abs(S[:, i1] - a) <= reach + 1e-9) & (np.abs(S[:, i2] - b) <= reach + 1e-9)))


def _combo_depth_mm(scene, pa, pb, pose_deg):
    """一个件对在一个姿态上的穿插深度（mm）。scene.depth → pose_contact_depth.contact_depth 的 dict，这里只取 depth_mm；
    取不到 / 不是数 → 抛（调用方按"深度算不出 = 超标"处理，不得变绿）。"""
    r = scene.depth(pa, pb, pose_deg)
    d = float(r["depth_mm"])
    if not math.isfinite(d) or d < 0:
        raise ValueError(f"depth_mm 无效：{r.get('depth_mm')!r}")
    return d


def _grid_components(pts, step):
    """网格点按 8 邻接（两轴各差 ≤ step）分连通块 → [[下标…], …]。不把两块隔开的角落并成一个大盒子。"""
    import numpy as np
    P = np.asarray(pts, dtype=np.float64).reshape(-1, 2)
    par = list(range(len(P)))

    def f(x):
        while par[x] != x:
            par[x] = par[par[x]]
            x = par[x]
        return x
    for i in range(len(P)):
        near = np.nonzero((np.abs(P[:, 0] - P[i, 0]) <= step + 1e-6) & (np.abs(P[:, 1] - P[i, 1]) <= step + 1e-6))[0]
        for k in near:
            ri, rk = f(i), f(int(k))
            if ri != rk:
                par[rk] = ri
    comp = {}
    for i in range(len(P)):
        comp.setdefault(f(i), []).append(i)
    return list(comp.values())


_MEASURED_DECIMALS = 4      # 组合碰撞逐条记录的 vol_mm3 记到 4 位小数，记分卡 measured 取的是这些数的最大值；上限"原样登记 chain 的数"，
                            # 比较也按同一精度取整（原始浮点 54.225006 记成 54.225，对上限 54.225 不算超）


def _approved_match(approved, j1, a, j2, b, x):
    """(a, b) 上的件对 x 是否落在某条用户批准的重训约束盒里 → (entry, over_cap)。盒内但体积超上限 → (entry, True)；不在任何盒 → (None, False)。
    体积按 _MEASURED_DECIMALS 位取整后与上限比（上限 = chain 记分卡 measured 原样登记，measured 来自 4 位小数的逐条记录）。"""
    over = None
    v6 = round(float(x[2]), _MEASURED_DECIMALS)
    for e in approved or []:
        if e["pair"] != frozenset((x[0], x[1])) or set(e["joints"]) != {j1, j2}:
            continue
        qa, qb = (e["q1"], e["q2"]) if e["joints"][0] == j1 else (e["q2"], e["q1"])
        if qa[0] - 1e-9 <= a <= qa[1] + 1e-9 and qb[0] - 1e-9 <= b <= qb[1] + 1e-9:
            if v6 <= e["cap"] + 1e-9:
                return e, False
            over = e
    return over, over is not None


def _pose_reached(env, pose, reach):
    """hr50 v6：一个多轴姿态是否"到过"：存在样本在姿态涉及的**全部**轴上都与之相差 ≤ reach（切比雪夫）；姿态里没写的关节按 0。
    比两轴判定严得多 —— 角点姿态（各关节同时到端点）几乎不可能被逐步记录盖到，这正是用户 09-28 选 C 的依据。"""
    import numpy as np
    names, S, _ = env
    q = np.array([float(pose.get(j, 0.0)) for j in names], dtype=np.float64)
    return bool(np.any(np.all(np.abs(np.asarray(S, dtype=np.float64) - q) <= reach + 1e-9, axis=1)))


def _approved_match_pose(approved, pose, x):
    """多轴姿态版 _approved_match：条目的两条关节在姿态里的角度落在盒内、件对相同 → (entry, over_cap)；优先返回不超上限的命中。"""
    best_over = None
    for e in approved or []:
        j1, j2 = e["joints"]
        ent, over = _approved_match([e], j1, float(pose.get(j1, 0.0)), j2, float(pose.get(j2, 0.0)), x)
        if ent is not None and not over:
            return ent, False
        if over:
            best_over = ent
    return best_over, best_over is not None


def _classify_pose_rows(rows, env, pol, depth_of, in_target, approved=None):
    """hr50 v6：策略包络角点姿态的碰撞记录逐条分类（与 _classify_combo_rows 同序、同口径）。rows = [(mode, tag, 全轴姿态 dict, (partA, partB, v))]。
      workspace：有一轴在目标区间外 → WARN；retrain：区间内、全轴 ±reach 内原版真实动作从没到过 → 重训约束（带深度、姿态去零）；
      light：到过、交集 ≤ v_max 且深度 ≤ d_max；approved：到过、超轻碰、落在批准盒内且 ≤ 上限；viol：其余（含深度算不出、盒内超上限）；unknown：目标区间分不出。
    返回 (cls[与 rows 对齐], retrain[…], light[…], viol[…], workspace[…], approved[…])，retrain / viol / workspace 按体积降序。"""
    reach = float(pol["combo"]["reach"])
    cls, ret, light, viol, ws, appr = [], [], [], [], [], []
    for mode, tag, pose, x in rows:
        pair = f"{x[0]}×{x[1]}"
        v6 = round(float(x[2]), _MEASURED_DECIMALS)
        base = dict(mode=str(mode), tag=str(tag), pair=pair, pose={k: round(float(q), 2) for k, q in pose.items() if abs(float(q)) > 1e-12}, vol_mm3=v6)
        tgt = in_target(pose)
        if tgt is None:
            cls.append("unknown"); continue
        if tgt is False:
            cls.append("workspace"); ws.append(base); continue
        if not _pose_reached(env, pose, reach):
            try:
                dep = round(float(depth_of(x[0], x[1], pose)), 4)
            except Exception as e:                                           # noqa: BLE001
                dep = f"算不出（{e!r}）"
            cls.append("retrain"); ret.append(dict(base, depth_mm=dep)); continue
        why = None
        if x[2] <= pol["v_max"]:
            try:
                dep = float(depth_of(x[0], x[1], pose))
            except Exception as e:                                           # noqa: BLE001
                dep, why = None, f"深度算不出（{e!r}）→ 按超标处理"
            if dep is not None and dep <= pol["d_max"]:
                cls.append("light"); light.append(dict(base, depth_mm=round(dep, 4))); continue
            if why is None:
                why = f"交集 {x[2]:.3f} ≤ {pol['v_max']} 但深度 {dep:.3f} mm > {pol['d_max']}"
        else:
            why = f"交集 {x[2]:.3f} mm³ > {pol['v_max']}"
        e, over = _approved_match_pose(approved, pose, x)
        if e is not None and not over:
            cls.append("approved"); appr.append(dict(base, id=e["id"], cap=e["cap"], why=why)); continue
        if over:
            why += f"；在用户批准盒 {e['id']} 内但 {v6} 超过批准上限 {e['cap']} mm³（几何变了，批准不算数）"
        cls.append("viol"); viol.append(dict(base, why=why))
    for lst in (ret, viol, ws):
        lst.sort(key=lambda r: -r["vol_mm3"])
    return cls, ret, light, viol, ws, appr


def _row_block_flag(r):
    """一条碰撞记录算不算"件的问题"：组合记录按 combo_class、角点姿态记录按 pose_class（hr50 v4 / v6：viol → 是；light / retrain / workspace /
    approved → 否；unknown → 分不出）；其余记录（单轴）照旧按目标区间（F-L6-3）。"""
    c = r.get("combo_class")
    if c is None:
        c = r.get("pose_class")
    if c is None:
        return r["in_target"]
    if c == "unknown":
        return None
    return c == "viol"


def _merge_retrain_blocks(items, env=None):
    """重训约束按关节对合并成训练用区间：同一关节对下，外扩盒（q1_pad × q2_pad）两轴都相交的块并成一个区，区 = 各块外扩盒的并集外包；
    列出区里的件对（各自最大交集）、格数；env 给了就数并集盒里有几个原版真实样本（>0 = 盒子会误伤真实动作，要按格点限）。
    返回 [{joints, q1, q2, pairs:[{pair, max_mm3}], n_blocks, n_cells, real_samples_in_box, max_mm3}]（按最大交集降序）。"""
    import numpy as np
    out, by = [], {}
    for it in items:
        by.setdefault(it["joints"], []).append(it)
    for joints, blocks in by.items():
        n = len(blocks)
        par = list(range(n))

        def f(x):
            while par[x] != x:
                par[x] = par[par[x]]; x = par[x]
            return x

        def box(b):
            return (b["q1_pad"][1], b["q1_pad"][2], b["q2_pad"][1], b["q2_pad"][2])
        for i in range(n):
            a1, b1, c1, d1 = box(blocks[i])
            for k in range(i + 1, n):
                a2, b2, c2, d2 = box(blocks[k])
                if a1 <= b2 + 1e-9 and a2 <= b1 + 1e-9 and c1 <= d2 + 1e-9 and c2 <= d1 + 1e-9:
                    ri, rk = f(i), f(k)
                    if ri != rk:
                        par[rk] = ri
        comps = {}
        for i in range(n):
            comps.setdefault(f(i), []).append(blocks[i])
        j1, j2 = joints.split("+")
        for bl in comps.values():
            q1 = [min(b["q1_pad"][1] for b in bl), max(b["q1_pad"][2] for b in bl)]
            q2 = [min(b["q2_pad"][1] for b in bl), max(b["q2_pad"][2] for b in bl)]
            pairs = {}
            for b in bl:
                pairs[b["pair"]] = max(pairs.get(b["pair"], 0.0), float(b["max_mm3"]))
            n_in = None
            if env is not None:
                names, S = env[0], env[1]
                if j1 in names and j2 in names:
                    i1, i2 = names.index(j1), names.index(j2)
                    n_in = int(np.count_nonzero((S[:, i1] >= q1[0]) & (S[:, i1] <= q1[1]) & (S[:, i2] >= q2[0]) & (S[:, i2] <= q2[1])))
            out.append(dict(joints=joints, q1=[j1, round(q1[0], 2), round(q1[1], 2)], q2=[j2, round(q2[0], 2), round(q2[1], 2)],
                            pairs=[dict(pair=k, max_mm3=round(v, 4)) for k, v in sorted(pairs.items(), key=lambda kv: -kv[1])],
                            n_blocks=len(bl), n_cells=sum(int(b["poses"]) for b in bl), real_samples_in_box=n_in,
                            max_mm3=round(max(pairs.values()), 4)))
    out.sort(key=lambda r: -r["max_mm3"])
    return out


def _classify_combo_rows(rows, j1, j2, env, pol, depth_of, in_target, r1, r2, step, approved=None):
    """pair_combination 的碰撞记录逐条分类（hr50 v4，2026-09-28 用户定）。rows = [(a, b, (partA, partB, v))]。
      workspace：有一轴在 frozen.yaml 目标区间外（F-L6-3：工作空间问题，控制端限位）→ WARN；
      retrain  ：目标区间内、但 (a, b) ±reach 内原版真实动作从没到过 → 重训约束（不判 FAIL）；
      light    ：到过、交集 ≤ v_max 且深度 ≤ d_max → 轻碰（不判 FAIL）；
      approved ：到过、超出轻碰，但落在用户批准的重训约束盒内且体积 ≤ 批准上限（hr50 v4，approved_retrain_constraints）→ 不 BLOCK；
      viol     ：到过、超出轻碰或深度算不出（含盒内但超上限）→ BLOCK；unknown：目标区间分不出。
    depth_of(partA, partB, pose) → mm（抛 = 算不出）。r1 / r2 = 两轴扫描域（给重训区间外扩一格时夹边）。
    返回 (cls[与 rows 对齐], retrain[连通块…], light[…], viol[…], workspace[…], approved[…])。"""
    import numpy as np
    reach = float(pol["combo"]["reach"])
    cls, light, viol, ws, ret_rows, appr = [], [], [], [], {}, []
    for idx, (a, b, x) in enumerate(rows):
        pair = f"{x[0]}×{x[1]}"
        pose = {j1: float(a), j2: float(b)}
        tgt = in_target(pose)
        if tgt is None:
            cls.append("unknown"); continue
        if tgt is False:
            cls.append("workspace"); ws.append(dict(pair=pair, pose=pose, vol_mm3=round(x[2], _MEASURED_DECIMALS))); continue
        if not _combo_reached(env, j1, a, j2, b, reach):
            cls.append("retrain"); ret_rows.setdefault(pair, []).append(idx); continue
        why = None
        if x[2] <= pol["v_max"]:
            try:
                dep = float(depth_of(x[0], x[1], pose))
            except Exception as e:                                           # noqa: BLE001
                dep, why = None, f"深度算不出（{e!r}）→ 按超标处理"
            if dep is not None and dep <= pol["d_max"]:
                cls.append("light"); light.append(dict(pair=pair, pose=pose, vol_mm3=round(x[2], _MEASURED_DECIMALS), depth_mm=round(dep, 4)))
                continue
            if why is None:
                why = f"交集 {x[2]:.3f} ≤ {pol['v_max']} 但深度 {dep:.3f} mm > {pol['d_max']}"
        else:
            why = f"交集 {x[2]:.3f} mm³ > {pol['v_max']}"
        e, over = _approved_match(approved, j1, a, j2, b, x)
        if e is not None and not over:
            cls.append("approved"); appr.append(dict(pair=pair, pose=pose, vol_mm3=round(x[2], _MEASURED_DECIMALS), id=e["id"], cap=e["cap"], why=why))
            continue
        if over:
            why += f"；在用户批准盒 {e['id']} 内但 {round(float(x[2]), _MEASURED_DECIMALS)} 超过批准上限 {e['cap']} mm³（几何变了，批准不算数）"
        cls.append("viol"); viol.append(dict(pair=pair, pose=pose, vol_mm3=round(x[2], _MEASURED_DECIMALS), why=why))
    names, S = env[0], env[1]
    i1, i2 = names.index(j1), names.index(j2)
    retrain = []
    for pair, idxs in sorted(ret_rows.items()):
        pts = [(float(rows[i][0]), float(rows[i][1])) for i in idxs]
        for comp in _grid_components(pts, step):
            cells = sorted({(round(pts[c][0], 2), round(pts[c][1], 2)) for c in comp})
            k = max(comp, key=lambda c: rows[idxs[c]][2][2])
            vmax, pmax = float(rows[idxs[k]][2][2]), {j1: pts[k][0], j2: pts[k][1]}
            q1 = [min(c[0] for c in cells), max(c[0] for c in cells)]
            q2 = [min(c[1] for c in cells), max(c[1] for c in cells)]
            # 撞点只在网格上量到；真实边界在相邻无碰撞格点之间 → 给训练的约束盒各向外扩一格，夹到扫描域
            q1p = [round(max(q1[0] - step, r1[0]), 2), round(min(q1[1] + step, r1[1]), 2)]     # 夹到扫描域（MJCF 弧度→度有 1e-14 噪声，取整）
            q2p = [round(max(q2[0] - step, r2[0]), 2), round(min(q2[1] + step, r2[1]), 2)]
            n_in = int(np.count_nonzero((S[:, i1] >= q1p[0]) & (S[:, i1] <= q1p[1]) & (S[:, i2] >= q2p[0]) & (S[:, i2] <= q2p[1])))
            pa, pb = pair.split("×")
            try:
                dep = round(float(depth_of(pa, pb, pmax)), 4)
            except Exception as e:                                           # noqa: BLE001
                dep = f"算不出：{e!r}"
            retrain.append(dict(pair=pair, poses=len(comp), max_mm3=round(vmax, 4), max_pose=pmax, depth_mm=dep,
                                q1=[j1, *q1], q2=[j2, *q2], q1_pad=[j1, *q1p], q2_pad=[j2, *q2p],
                                real_samples_in_pad_box=n_in, cells=[list(c) for c in cells]))
    return cls, retrain, light, viol, ws, appr


def _real_pose_collisions(res, cap, tol=None):
    """真实姿态上的实体交叠必须闭环；原版同 body 对碰撞不是豁免。

    旧版仅查 defect_pairs 会放行共有但变严重、或实际接触件不同的碰撞。
    对比产物须完整、新鲜，且复算原始逐姿态记录，不能靠删除汇总字段变绿。
    本项仍是采样筛查，不证明全分辨率、连续过渡或受载运动合格。
    """
    import hashlib
    import policy_pose_compare as compare
    import policy_pose_evidence as evidence
    tol = compare.numerical_tolerance() if tol is None else tol
    cmp_path = _REAL_POSES.parent / "compare.json"
    res.inputs += [_rel(p) for p in (_REAL_POSES, cmp_path, _REAL_POSES.parent / "poses.jsonl",
                                     _REAL_POSES_SCRIPT, _REAL_POSES_COMPARE_SCRIPT,
                                     Path(evidence.__file__), compare.OURS_MAP, compare.ORIG_MAP,
                                     compare.TOLERANCES, ROOT / "tools/gate/data/frozen.yaml")]
    # The examined result cannot choose its own easier question set.
    source = cap.get("pose_steps_source") or {}
    source_path = ROOT / str(source.get("path", ""))
    res.inputs.append(_rel(source_path))
    if not source_path.is_file() or source.get("sha256") != evidence.sha(source_path):
        res.unknown("_capability", "real_pose_collisions", "批准的逐步姿态源缺失/指纹不一致：frozen.capability_envelope.pose_steps_source")
        return
    prov = (_rel(_REAL_POSES) + " / " + _rel(cmp_path) + " / " + _rel(_REAL_POSES_SCRIPT) + " / " + _rel(_REAL_POSES_COMPARE_SCRIPT)
            + " / tools/sim/policy_envelope.py --dump-steps / tools/sim/orig_scene_export.py")
    crit = (f"in-scope 采样真实姿态在当前 placed/ 上交集 > {tol} mm³ 的可动实体对 = 0；"
            "原版共有碰撞不自动豁免（接触用途必须另有实物/几何证据）；"
            "summary/compare/逐姿态元组/映射/阈值指纹一致，姿态清单完整且两方逐关节角相同")
    if not _REAL_POSES.exists():
        res.unknown("_capability", "real_pose_collisions", f"没有 {_rel(_REAL_POSES)}：先跑 policy_envelope.py --dump-steps + policy_pose_collisions.py",
                    provenance=prov); return
    try:
        s = json.loads(_REAL_POSES.read_text(encoding="utf-8"))
        fp = s["fingerprint"]
    except Exception as e:
        res.unknown("_capability", "real_pose_collisions", f"summary.json 读不了/没有 fingerprint：{e}", provenance=prov); return
    stale = []
    if fp.get("placed_sha256") != _placed_fingerprint():
        stale.append("placed/ 场景变了")
    steps = ROOT / s.get("steps_file", "")
    res.inputs.append(_rel(steps))
    if steps.resolve() != source_path.resolve():
        stale.append("summary 选择的逐步姿态源不在批准声明中")
    if not steps.exists() or fp.get("steps_sha256") != hashlib.sha256(steps.read_bytes()).hexdigest():
        stale.append("逐步元组文件变了/不在")
    if fp.get("script_sha256") != hashlib.sha256(_REAL_POSES_SCRIPT.read_bytes()).hexdigest():
        stale.append("检查脚本变了")
    if fp.get("tol_mm3") != tol:
        stale.append("采样交集阈值不是当前 Gate 阈值")
    want = sorted(cap.get("modes_in_scope") or [])
    if sorted(s.get("modes_in_scope") or []) != want:
        stale.append(f"in-scope 模式不一致：summary {s.get('modes_in_scope')} vs frozen {want}")
    if stale:
        res.unknown("_capability", "real_pose_collisions", "离线结果过期，不认：" + "；".join(stale) + " → 重跑 policy_pose_collisions.py",
                    provenance=prov); return
    try:
        grid = s["grid_deg"]
        if (isinstance(grid, bool) or not isinstance(grid, (int, float))
                or not math.isfinite(grid) or not 0 < grid <= _STEP_DEG):
            raise ValueError(f"真实姿态去重网格必须为0..{_STEP_DEG}°，不许放粗来减少采样义务")
        expected = evidence.expected_poses(steps, want, s["grid_deg"])
        current_fp = evidence.generation_fingerprint(PLACED, steps, compare.OURS_MAP,
                                                     want, s["grid_deg"], tol)
        res.inputs += list(current_fp["dependencies_sha256"])
        res.inputs += [_rel(p) for p in sorted(PLACED.glob("*.stl"))]
        if fp != current_fp or s.get("complete") is not True or s.get("partial") is not False:
            # 2026-09-22 hr32 复审 m-03：点名是哪个依赖变了（hr29/hr32 都是采样后 merge_slice_run 改了 printability.yaml，没人看出来）
            od, cd = (fp.get("dependencies_sha256") or {}), current_fp["dependencies_sha256"]
            changed = sorted(k for k in set(od) | set(cd) if od.get(k) != cd.get(k))
            other = sorted(k for k in set(fp) | set(current_fp) if k != "dependencies_sha256" and fp.get(k) != current_fp.get(k))
            why = ("采样后变了的依赖：" + "、".join(changed[:6]) + ("…" if len(changed) > 6 else "")) if changed else ""
            if other: why += ("；" if why else "") + "指纹字段不同：" + "、".join(other)
            if s.get("complete") is not True or s.get("partial") is not False: why += ("；" if why else "") + "本次仅限量采样"
            raise ValueError("采样生成指纹不完整/过期，或本次仅限量采样；使用新目录完整重跑" + (f"（{why}）" if why else ""))
        if s.get("expected_poses") != len(expected["rows"]):
            raise ValueError("采样义务数与 NPZ 不一致")
        if s.get("poses_sha256") != evidence.sha(_REAL_POSES.parent / "poses.jsonl"):
            raise ValueError("生成时与当前逐姿态记录指纹不一致")
    except (ValueError, KeyError, TypeError, OSError) as e:
        res.unknown("_capability", "real_pose_collisions", f"采样生成证据无效：{e}", provenance=prov)
        return
    n_hit = int(s.get("poses_with_hits", 0)); n = int(s.get("poses_checked", 0))
    if not cmp_path.exists():
        res.unknown("_capability", "real_pose_collisions", f"没有 {_rel(cmp_path)}：先跑 policy_pose_compare.py --ours {_rel(_REAL_POSES.parent)}",
                    provenance=prov); return
    try:
        c = json.loads(cmp_path.read_text(encoding="utf-8")); cfp = c["fingerprint"]
    except Exception as e:
        res.unknown("_capability", "real_pose_collisions", f"compare.json 读不了/没有 fingerprint：{e}", provenance=prov); return
    stale = []
    ours_poses = _REAL_POSES.parent / "poses.jsonl"
    if not ours_poses.exists() or cfp.get("ours_poses_sha256") != hashlib.sha256(ours_poses.read_bytes()).hexdigest():
        stale.append("compare 用的 poses.jsonl 不是当前这份")
    orig_poses = ROOT / str(cfp.get("orig_dir", "")) / "poses.jsonl"
    res.inputs.append(_rel(orig_poses))
    if not orig_poses.exists() or cfp.get("orig_poses_sha256") != hashlib.sha256(orig_poses.read_bytes()).hexdigest():
        stale.append("原版 poses.jsonl 变了/不在")
    if cfp.get("script_sha256") != hashlib.sha256(_REAL_POSES_COMPARE_SCRIPT.read_bytes()).hexdigest():
        stale.append("对比脚本变了")
    for field, path in (("ours_summary_sha256", _REAL_POSES),
                        ("ours_map_sha256", compare.OURS_MAP),
                        ("orig_map_sha256", compare.ORIG_MAP),
                        ("tolerances_sha256", compare.TOLERANCES)):
        if cfp.get(field) != hashlib.sha256(path.read_bytes()).hexdigest():
            stale.append(f"{field} 不一致/缺失")
    if stale:
        res.unknown("_capability", "real_pose_collisions", "compare.json 过期，不认：" + "；".join(stale) + " → 重跑 policy_pose_compare.py",
                    provenance=prov); return
    try:
        evidence.validate_pose_records(ours_poses, expected)
        evidence.validate_pose_records(orig_poses, expected)
        fresh = compare.analyze(ours_poses, orig_poses,
                                json.loads(compare.OURS_MAP.read_text())["body_for_part"],
                                json.loads(compare.ORIG_MAP.read_text())["body_for_part"], tol)
        if any(c.get(k) != v for k, v in fresh.items()):
            raise ValueError("compare 字段缺失/与逐姿态复算不一致")
        if fresh["poses_compared"] != n or n <= 0:
            raise ValueError("summary 姿态数与实际记录不一致/没有证据")
    except (ValueError, KeyError, TypeError, OSError) as e:
        res.unknown("_capability", "real_pose_collisions", f"逐姿态证据无效：{e}", provenance=prov)
        return
    # hr50（2026-09-28）：逐姿态分三类 —— 极限段 → 重训约束（不判 FAIL）；其余姿态 ≤ 原版 p25 的轻碰（体积 + 深度）→ 不判 FAIL；其余 → FAIL。
    res.inputs += [_rel(_REAL_POSE_POLICY), _rel(_REAL_POSE_DEPTH_MODULE)]
    try:
        pol = _load_real_pose_policy()
    except Exception as e:                                                   # noqa: BLE001
        res.unknown("_capability", "real_pose_collisions", f"判定口径文件读不了：{_rel(_REAL_POSE_POLICY)}：{e!r}", provenance=prov)
        return
    ours_bfp = json.loads(compare.OURS_MAP.read_text())["body_for_part"]
    per_pose = compare.load(ours_poses, ours_bfp, tol)                       # 与 analyze 同一份逐姿态记录、同一阈值（上面已复算核对过）
    retrain, light_c, viol, depth_scene = _classify_real_poses(per_pose, pol, ours_bfp)
    n_viol = sum(v["poses"] for v in viol.values())
    crit2 = (f"真实姿态碰撞（hr50 口径，{_rel(_REAL_POSE_POLICY)}）：极限段 {sorted('/'.join(x) for x in pol['segs'])} 的碰撞进重训约束、不判 FAIL；"
             f"其余姿态交集 ≤ {pol['v_max']} mm³ 且穿插深度 ≤ {pol['d_max']} mm 为轻碰（原版自身真接触峰值 p25），超出即 FAIL；"
             "深度算不出按超标；原版共有碰撞不豁免。" + crit)
    top = sorted(viol.items(), key=lambda kv: -kv[1]["max_mm3"])[:8]
    res.add(subject="_capability", check="real_pose_collisions", state=FAIL if viol else PASS, severity=BLOCK,
            measured=n_viol, criterion=crit2, evidence_n=n,
            detail=(f"{n} 个采样姿态、{fresh['poses_with_collisions']} 个有实体交叠（> {tol} mm³）。"
                    f"超标：{len(viol)} 个 body 对 / {n_viol} 个姿态次；轻碰 {len(light_c)} 对；重训约束 {len(retrain)} 对（见 real_pose_retrain_constraints）。"
                    + ("超标明细：" + "；".join(f"{k}: {v['max_mm3']:.3f} mm³，{v['poses']} 姿态，{v['segs']}，{v['why']}，件 {sorted(v['parts'])}" for k, v in top) + "。" if viol else "")
                    + ("轻碰：" + "；".join(f"{k}: ≤{v['max_mm3']:.3f} mm³ / 深 {v['max_depth_mm']:.3f} mm，{v['poses']} 姿态 {v['segs']}" for k, v in sorted(light_c.items())) + "。" if light_c else "")
                    + "零碰撞 / 轻碰也不证明连续运动、打印公差或负载合格。"),
            provenance=prov)
    # 重训约束清单：每对取交集最大的那一帧算深度 + 关键关节（顶到 MJCF 限位的标"到头"）
    try:
        ranges = _mjcf_joint_ranges_deg(_GEN_MJCF)
    except Exception:                                                        # noqa: BLE001
        ranges = {}
    items = []
    for k, v in sorted(retrain.items(), key=lambda kv: -kv[1]["max_mm3"]):
        dep, drows = None, []
        try:
            if depth_scene is None:
                depth_scene = _DepthScene(ours_bfp)
            dep, drows = _light_contact_depth(depth_scene, v["solid_pairs"], v["max_pose"])
        except Exception as e:                                               # noqa: BLE001
            drows = [dict(error=repr(e))]
        mk = v["max_key"]
        items.append(dict(pair=k, poses=v["poses"], segments=v["segs"], max_mm3=round(v["max_mm3"], 3),
                          depth_mm_at_max=None if dep is None else round(dep, 3), depth_rows=drows, parts=sorted(v["parts"]),
                          max_pose=dict(mode=mk[0], case=mk[1], step=mk[2]), key_joints=_key_joints(v["max_pose"], ranges)))
    res.add(subject="_capability", check="real_pose_retrain_constraints", state=FAIL if items else PASS, severity=INFO,
            measured=dict(n_pairs=len(items), items=items), evidence_n=sum(v["poses"] for v in retrain.values()),
            criterion=("用户 2026-09-28：极限姿势靠重训限制、不削。这些 (mode, case) 段里的碰撞不阻断，逐对列出交给训练"
                       "（开自碰撞惩罚或收限位）：" + "、".join(f"{a}/{b}（{pol['why'].get((a, b), '')}）" for a, b in sorted(pol["segs"]))),
            detail=("；".join(f"{it['pair']}：{it['poses']} 姿态 {it['segments']}，最大交集 {it['max_mm3']} mm³，"
                              f"深 {it['depth_mm_at_max'] if it['depth_mm_at_max'] is not None else '算不出'} mm，"
                              f"最大那帧 {it['max_pose']['mode']}/{it['max_pose']['case']}#{it['max_pose']['step']}：{'、'.join(it['key_joints'])}"
                              for it in items) if items else "极限段内没有碰撞"),
            provenance=prov + " / " + _rel(_REAL_POSE_POLICY))

def _mjcf_chosen_actuator(path):
    """<default class="chosen_actuator"> 里 joint/position 的属性；没这个 class → None。
    正则而不是 ElementTree：make_mjcf 的头注释里有 "--"，严格 XML 解析器不收，MuJoCo 收。"""
    txt = re.sub(r"<!--.*?-->", "", path.read_text(encoding="utf-8"), flags=re.S)   # 头注释里也写着 chosen_actuator，先剥掉
    m = re.search(r'<default class="chosen_actuator">(.*?)</default>', txt, re.S)
    if not m:
        return None
    out = {}
    for tag in ("joint", "position"):
        e = re.search(r"<" + tag + r"\b([^>]*)/?>", m.group(1))
        if e:
            out[tag] = dict(re.findall(r'([\w:-]+)="([^"]*)"', e.group(1)))
    return out


def _bam_params_sync(res):
    """发布/包内 BAM 与 base/train/allcol 三种正式 MJCF 变体的参数一致。"""
    import hashlib
    sub, chk = "_simmodel", "bam_params_sync"
    models = (_GEN_MJCF, _GEN_TRAIN_MJCF, _GEN_ALLCOL_MJCF)
    res.inputs += [_rel(p) for p in (_BAM_RELEASE, _BAM_PKG, *models)]
    prov = " / ".join(_rel(p) for p in (_BAM_RELEASE, _BAM_PKG, *models))

    def valid_parameter(value, key):
        # JSON booleans/strings are not measurements; NaN/Inf and negative
        # losses/inertia are invalid even when all copies contain the same value.
        if not isinstance(value, (int, float)) or isinstance(value, bool):
            return False
        try:
            return math.isfinite(value) and (value > 0 if key == "max_torque" else value >= 0)
        except (OverflowError, TypeError):
            return False

    def finish(bad, lines):
        res.add(subject=sub, check=chk, state=FAIL if bad else PASS, severity=BLOCK, measured=bad,
                criterion=f"BAM friction_base/friction_viscous/armature 必须是有限非负数，max_torque 必须有限且>0；"
                          f"发布/包内 json 与 base/train/allcol 的 chosen_actuator 数值一致（相对差 ≤{_BAM_REL_TOL:g}）；"
                          "frictionloss/damping/armature 各恰一个非负标量；forcerange 恰两个有限、有序端点，"
                          "分别为 -max_torque 和 +max_torque，不能仅比较绝对值最大值",
                evidence_n=len(lines), detail="；".join(lines) + ("" if bad else "。两份参数与三种正式模型一致"), provenance=prov)

    js = {}
    for tag, p in (("release", _BAM_RELEASE), ("pkg", _BAM_PKG)):
        if not p.exists():
            res.unknown(sub, chk, f"{_rel(p)} 不存在 —— 训练执行器的摩擦中心值没有来源，核对不了", provenance=prov)
            return
        try:
            js[tag] = json.loads(p.read_text(encoding="utf-8"))
        except Exception as e:                                       # noqa: BLE001
            res.unknown(sub, chk, f"{_rel(p)} 解析不了：{e}", provenance=prov)
            return
        if not isinstance(js[tag], dict):
            res.unknown(sub, chk, f"{_rel(p)} 顶层必须是参数对象，实际是 {type(js[tag]).__name__}", provenance=prov)
            return
    bad, lines = 0, []
    invalid = False
    same_sha = hashlib.sha256(_BAM_RELEASE.read_bytes()).hexdigest() == hashlib.sha256(_BAM_PKG.read_bytes()).hexdigest()
    for key, _t, _a in _BAM_FIELDS:
        a, b = js["release"].get(key), js["pkg"].get(key)
        if a is None or b is None:
            res.unknown(sub, chk, f"m1.json 缺字段 {key}（发布 {a} / 包内 {b}）", provenance=prov)
            return
        if not valid_parameter(a, key) or not valid_parameter(b, key):
            bad += 1; invalid = True
            lines.append(f"✘ json {key}: 发布 {a!r} / 包内 {b!r}，必须是有限"
                         + ("正数" if key == "max_torque" else "非负数"))
            continue
        ok = abs(a - b) <= _BAM_REL_TOL * max(abs(a), 1e-9)
        bad += 0 if ok else 1
        lines.append(f"{'✔' if ok else '✘'} json {key}: 发布 {a:.6g} / 包内 {b:.6g}")
    if invalid:
        finish(bad, lines)
        return
    if not same_sha and bad == 0:
        lines.append("（两份 json 字节不同但四个用到的字段一致 —— 允许，其余字段是拟合统计）")
    for mp in models:
        if not mp.exists():
            res.unknown(sub, chk, f"{_rel(mp)} 不存在", provenance=prov)
            return
        try:
            ca = _mjcf_chosen_actuator(mp)
        except (OSError, ValueError) as e:
            res.unknown(sub, chk, f"{_rel(mp)} 读取不了：{e}", provenance=prov)
            return
        if not ca:
            res.unknown(sub, chk, f"{_rel(mp)} 里没有 class=chosen_actuator 的 default —— 执行器参数没写进去", provenance=prov)
            return
        for key, tag, attr in _BAM_FIELDS:
            raw = (ca.get(tag) or {}).get(attr)
            if raw is None:
                bad += 1
                lines.append(f"✘ {_rel(mp)} chosen_actuator/{tag}@{attr} 缺失")
                continue
            want = js["release"][key]
            try:
                vals = [float(x) for x in raw.split()]
            except (ValueError, OverflowError):
                vals = []
            eps = _BAM_REL_TOL * max(abs(want), 1e-9)
            if attr == "forcerange":
                ok = (len(vals) == 2 and all(math.isfinite(v) for v in vals)
                      and vals[0] < 0 < vals[1]
                      and abs(vals[0] + want) <= eps and abs(vals[1] - want) <= eps)
                expected = f"[-{want:.6g}, +{want:.6g}]"
            else:
                ok = (len(vals) == 1 and math.isfinite(vals[0]) and vals[0] >= 0
                      and abs(vals[0] - want) <= eps)
                expected = f"{want:.6g}（单个有限非负数）"
            bad += 0 if ok else 1
            lines.append(f"{'✔' if ok else '✘'} {mp.name} {tag}@{attr}={raw!r}，期望 {expected}")
    finish(bad, lines)


# ── E′ 线束实体模型扫掠（hr44reg 2026-09-25；协调员 23:15 契约 + 00:50 五条硬要求）──────────────────────
# was_until_2026_09_26_hr44b: _HS_CRIT = ("hit_in_target == 0 且 tension_in_target == 0 且 min_bend_R ≥ min_bend_limit" …)
_HS_CRIT = ("hit_in_target_limb == 0 且 hit_real_poses == 0 且 tension_in_target_limb == 0 且 tension_real_poses == 0 且 min_bend_R ≥ min_bend_limit"
            "（duckstructure.wiring_body.sweep_summary 契约 hr44 第二轮：姿态集 = 线路所在肢体链全部关节（单轴 2.5° + 父子两两 5°，区内按 frozen.yaml）"
            " + 真实姿态 5601；新键缺任一 → NOT_RUN，不回退旧 hit_in_target）")
_HS_PROV = "duckstructure.wiring_body.sweep_summary / harness.yaml:bundles[].route_model / README 第 2 节 L6 E"
_HS_TIMEOUT_DEFAULT = 600.0           # s；DUCK_HARNESS_TIMEOUT_S 覆盖（≤0 = 不设超时）。超时 = NOT_RUN(BLOCK)。实测 coarse=False 76.5 s / coarse=True 12.5 s（16 条线路，本机 2026-09-25）→ ≈8 倍余量


def _hs_num(x):
    return isinstance(x, (int, float)) and not isinstance(x, bool) and math.isfinite(x)


def _hs_call(fn, coarse, timeout_s):
    """sweep_summary(coarse=…)，主线程里用 SIGALRM 限时（层进程 = 进程池 worker 的主线程）；非主线程无法限时 → 照跑，evidence 记下。"""
    import signal
    import threading
    if not (timeout_s and timeout_s > 0 and hasattr(signal, "SIGALRM")
            and threading.current_thread() is threading.main_thread()):
        return fn(coarse=coarse), False

    def _alarm(signum, frame):
        raise TimeoutError(f"sweep_summary(coarse={coarse}) 超过 {timeout_s:g} s（DUCK_HARNESS_TIMEOUT_S）")
    old = signal.signal(signal.SIGALRM, _alarm)
    signal.setitimer(signal.ITIMER_REAL, float(timeout_s))
    try:
        return fn(coarse=coarse), True
    finally:
        signal.setitimer(signal.ITIMER_REAL, 0.0)
        signal.signal(signal.SIGALRM, old)


# ── hr46（2026-09-26）：两桶证据（own = 撞本线路自己登记的 hook 型固定件实体 / other = 其余）—— 只作证据，判据 _HS_CRIT 不变（仍按合计口径）──
_HS_FIXINGS = ROOT / "duckstructure/data/wire_fixings_v1.json"     # own 桶的 hook 锚点：本层自己按 part + id 拼钩文件名
_HS_OWN_KEYS = ("hit_in_target_limb_own", "hit_in_target_limb_other", "hit_in_target_limb_both",
                "hit_real_poses_own", "hit_real_poses_other", "hit_real_poses_both")


def _hs_expected_hooks():
    """hr46：({线路 id: 期望钩文件（_rel 路径，排序）}, 错误原因 | None)。本层自己按 wire_fixings_v1.json anchors（type hook）的 part + id 拼
    PLACED 旁 hooks/<part>_<id>.stl —— 不 glob 目录（写错标签的文件不能被吃进来），也不只信 sweep_summary 报的 own_fixing_files。"""
    import json
    hd = Path(PLACED).parent / "hooks"
    try:
        js = json.loads(_HS_FIXINGS.read_text(encoding="utf-8"))
        exp = {}
        for a in js.get("anchors") or []:
            if isinstance(a, dict) and a.get("type") == "hook":
                exp.setdefault(str(a.get("route")), []).append(_rel(hd / f"{a.get('part')}_{a.get('id')}.stl"))
        return {k: sorted(x) for k, x in exp.items()}, None
    except Exception as e:                                                                   # noqa: BLE001
        return None, f"{type(e).__name__}: {e}"


def _hs_own_check(sid, v, exp_hooks, exp_err):
    """hr46：(原因标签, 说明) 或 None（= 通过）。任一条 → 该线路格 NOT_RUN(BLOCK)（不许当零命中、不许照旧判 PASS / FAIL 混过去）：
    ① hook 锚点读不了；② 登记了 hook 锚点但钩文件不在（sweep_summary 报 missing，或本层自己 is_file 不过）；
    ③ sweep_summary 报的钩文件表与本层按 anchors 拼的对不上；④ 恒等式 own + other − both == 合计（肢体区 / 真实姿态两套，本层自己重算）
    不成立，或 sweep_summary 的 identity_ok 不是 True。"""
    if exp_hooks is None:
        return "HookAnchors", f"{_rel(_HS_FIXINGS)} 的 hook 锚点读不了（{exp_err}），own 钩文件无从核对"
    want = exp_hooks.get(sid, [])
    got_f = sorted(str(x) for x in v.get("own_fixing_files") or [])
    got_m = sorted(str(x) for x in v.get("own_fixing_missing") or [])
    absent = sorted(set(got_m) | {p for p in want if not (ROOT / p).is_file()})
    if absent:
        return "HookMissing", f"登记了 hook 锚点但 hooks/ 里没有 {absent}（不许当零命中）"
    if got_f != want:
        return "HookMismatch", f"sweep_summary 报的 own 钩文件 {got_f} 与本层按 anchors（part + id）拼的 {want} 对不上"
    a, b, c, n = (v.get(k) for k in ("hit_in_target_limb_own", "hit_in_target_limb_other", "hit_in_target_limb_both", "hit_in_target_limb"))
    d, e, f, m = (v.get(k) for k in ("hit_real_poses_own", "hit_real_poses_other", "hit_real_poses_both", "hit_real_poses"))
    idl, idr = a + b - c == n, d + e - f == m
    if not (idl and idr) or v.get("identity_ok") is not True:
        return "IdentityBroken", (f"恒等式不成立：own + other − both == 合计 —— 肢体区 {a}+{b}−{c}={a + b - c} vs {n}（{'✓' if idl else '✘'}）、"
                                  f"真实姿态 {d}+{e}−{f}={d + e - f} vs {m}（{'✓' if idr else '✘'}）；sweep_summary identity_ok={v.get('identity_ok')!r}")
    return None


def _hs_own_note(v, want):
    """hr46：detail 末尾追加的两桶证据（不参与判）。"""
    return (f"｜两桶（证据，不参与判；own = 撞本线路自己登记的 hook 钩实体（wiring_body.OWN_SD_MM 内）/ other = 其余）："
            f"own/other/both 肢体区 {v.get('hit_in_target_limb_own')}/{v.get('hit_in_target_limb_other')}/{v.get('hit_in_target_limb_both')}、"
            f"真实 {v.get('hit_real_poses_own')}/{v.get('hit_real_poses_other')}/{v.get('hit_real_poses_both')}；"
            f"钩文件 {want if want else '无（本线路没登记 hook 锚点）'}")


def _harness_sweep(ctx, res):
    """线束按 duckstructure.wiring_body 实体模型逐线路扫掠 → 每条线路一格 L6/<线路 id>，判据 harness_sweep。

    · 调 sweep_summary(coarse=False)（l6 同采样：单轴 2.5° + 父子相邻两两 5°）；DUCK_HARNESS_COARSE=1 → coarse=True（快车道：单轴 10° / 两两 15°）。
    · 每条线路：PASS ⇔ hit_in_target == 0 且 tension_in_target == 0 且 min_bend_R ≥ min_bend_limit；否则 FAIL(BLOCK)。
      measured = {hit_in_target, tension_in_target, min_bend_R} 三个数，detail = 模型 status + 限值 / 采样数 / 全域数；evidence_n = n_poses。
      数缺（不是有限数）→ FAIL(BLOCK)（未知 = 失败），detail 点名缺哪个。
    · NOT_RUN(BLOCK)（绝不 INFO，免得"没跑"读成放行）：import 失败 / 没有 sweep_summary / 抛异常 / 超时 / 返回的不是 dict /
      本层 placed/ 与 wiring_body 扫的 placed/ 不是同一目录（反例替身场景：结果不属于本轮场景，不调用）/
      harness.yaml:route_model 登记了但 sweep_summary 没返回的线路。detail 与 evidence 写原因 + 异常类型。
    · 不经过 _Scene（不走凸包分离跳过证明，不进 n_skipped 恒等式）、不用 check_cache：每轮现算。
    返回 (本轮判过且 route_model.covers_whole_bundle 的束 id 集合, evidence dict)。"""
    import importlib
    import os
    import time
    import core as _core
    bundles = (ctx.data.get("harness") or {}).get("bundles") or []
    declared = {}                                           # 线路 id → 登记它的束 id
    for hb in bundles:
        rm = hb.get("route_model")
        if isinstance(rm, dict):
            for rid in rm.get("routes") or []:
                declared.setdefault(str(rid), hb.get("id"))
    coarse = os.environ.get("DUCK_HARNESS_COARSE", "").strip() not in ("", "0", "false", "False")
    exp_hooks, exp_err = _hs_expected_hooks()              # hr46：本层自己拼的 own 钩文件表（见 _hs_expected_hooks）
    if _HS_FIXINGS.exists():
        res.inputs.append(_rel(_HS_FIXINGS))               # hr46：hook 锚点变了 → 本层格子过期
    try:
        timeout_s = float(os.environ.get("DUCK_HARNESS_TIMEOUT_S", _HS_TIMEOUT_DEFAULT))
    except ValueError:
        timeout_s = _HS_TIMEOUT_DEFAULT
    mode = ("coarse=True（DUCK_HARNESS_COARSE=1 快车道：单轴 10° / 两两 15°）" if coarse
            else "coarse=False（l6 同采样：单轴 2.5° / 两两 5°）")
    ev = {"source": "duckstructure.wiring_body.sweep_summary", "mode": mode, "timeout_s": timeout_s,
          "declared_routes": len(declared), "check_cache": "不用（每轮现算）", "scene_skip_path": "不经过 _Scene"}
    summ, why, exc = None, None, None
    t0 = time.perf_counter()
    try:
        WB = importlib.import_module("duckstructure.wiring_body")
        fn = getattr(WB, "sweep_summary", None)
        if not callable(fn):
            raise AttributeError("duckstructure.wiring_body 没有可调用的 sweep_summary")
        for _dp in (getattr(WB, "DATA", None),):
            if _dp and Path(_dp).exists():
                res.inputs.append(_rel(Path(_dp)))           # 线路数据变了 → 本层格子过期
        _pd = getattr(WB, "_placed_dir", None)
        wb_placed = Path(_pd()) if callable(_pd) else Path(_core.PLACED)
        if wb_placed.resolve() != Path(PLACED).resolve():
            why, exc = (f"本层 placed/ = {_rel(PLACED)}，wiring_body 扫的是 {_rel(wb_placed)} —— 不是同一个场景"
                        f"（反例替身），扫掠结果不属于本轮，不调用"), "PlacedMismatch"
        else:
            summ, ev["timeout_enforced"] = _hs_call(fn, coarse, timeout_s)
            if not isinstance(summ, dict):
                why, exc, summ = f"sweep_summary 返回 {type(summ).__name__}，不是 {{线路 id: {{…}}}}", "TypeError", None
    except Exception as e:                                                                   # noqa: BLE001
        summ, why, exc = None, f"{type(e).__name__}: {e}", type(e).__name__
    ev["seconds"] = round(time.perf_counter() - t0, 2)
    if summ is None:
        ev.update(not_run=why, exception_type=exc)
        for rid in (sorted(declared) or ["_harness"]):
            res.add(subject=rid, check="harness_sweep", state=NOT_RUN, severity=BLOCK, measured=None,
                    criterion=_HS_CRIT, evidence_n=0,
                    detail=(f"没判（{exc}）：{why}；{mode}"
                            + (f"；harness.yaml:{declared[rid]}.route_model 登记的线路" if rid in declared else "")),
                    provenance=_HS_PROV)
        return set(), ev
    judged = set()
    for rid in sorted(summ, key=str):
        v, sid = summ[rid], str(rid)
        owner = declared.get(sid)
        reg = (f"登记在 harness.yaml:{owner}.route_model" if owner
               else "harness.yaml 没有任何束的 route_model 登记这条线路")
        if not isinstance(v, dict):
            res.add(subject=sid, check="harness_sweep", state=NOT_RUN, severity=BLOCK, measured=None,
                    criterion=_HS_CRIT, evidence_n=0,
                    detail=f"没判（TypeError）：sweep_summary[{sid!r}] 是 {type(v).__name__} 不是 dict；{reg}", provenance=_HS_PROV)
            continue
        # hr44 第二轮（复审 #3 M2 + Gate 守护 02:32）：只读新键；四个新键缺任一 → NOT_RUN(BLOCK)，不许拿旧 hit_in_target（只扫跨越关节）顶替。
        #   was_until_2026_09_26_hr44b: hit, ten = v.get("hit_in_target"), v.get("tension_in_target")
        new_keys = ("hit_in_target_limb", "hit_real_poses", "tension_in_target_limb", "tension_real_poses")
        # was_until_2026_09_26_hr46: lack = [k for k in new_keys if not _hs_num(v.get(k))]
        lack = [k for k in new_keys + _HS_OWN_KEYS if not _hs_num(v.get(k))]                        # hr46：两桶 6 键并进来
        lack += [k for k in ("own_fixing_files", "own_fixing_missing") if not isinstance(v.get(k), list)]
        lack += [k for k in ("identity_ok",) if not isinstance(v.get(k), bool)]
        if lack:
            res.add(subject=sid, check="harness_sweep", state=NOT_RUN, severity=BLOCK, measured=None,
                    criterion=_HS_CRIT, evidence_n=0,
                    detail=f"没判（MissingKey）：sweep_summary[{sid!r}] 缺新键 {lack}（肢体全关节 / 真实姿态两套数都要有；不回退旧 hit_in_target）；{reg}",
                    provenance=_HS_PROV)
            continue
        # hr46：own 钩文件与恒等式本层自己核（见 _hs_own_check）；不过 → NOT_RUN(BLOCK)
        bad_own = _hs_own_check(sid, v, exp_hooks, exp_err)
        if bad_own:
            res.add(subject=sid, check="harness_sweep", state=NOT_RUN, severity=BLOCK, measured=None,
                    criterion=_HS_CRIT, evidence_n=0, detail=f"没判（{bad_own[0]}）：{bad_own[1]}；{reg}", provenance=_HS_PROV)
            continue
        own_files = (exp_hooks or {}).get(sid, [])
        res.inputs.extend(p for p in own_files if p not in res.inputs)     # hr46：本层自己拼的钩文件表进 hash 绑定（钩一改 → 记分卡 STALE）
        hit, hit_r, ten, ten_r = (v.get(k) for k in new_keys)
        R, lim, n = v.get("min_bend_R"), v.get("min_bend_limit"), v.get("n_poses")
        miss = [k for k, x in (("min_bend_R", R), ("min_bend_limit", lim), ("n_poses", n)) if not _hs_num(x)]
        if not miss and n <= 0:
            miss.append("n_poses（= 0，没有采样姿态）")
        measured = {"hit_in_target_limb": hit, "hit_real_poses": hit_r, "tension_in_target_limb": ten, "tension_real_poses": ten_r, "min_bend_R": R}
        measured.update({k: v.get(k) for k in _HS_OWN_KEYS})              # hr46：两桶 6 键追加在原 5 键之后（证据，不参与判）
        ok = not miss and hit == 0 and hit_r == 0 and ten == 0 and ten_r == 0 and R >= lim
        why_bad = []
        if miss:
            why_bad.append(f"数缺（未知 = 失败）：{miss}")
        if hit > 0:
            why_bad.append(f"肢体全关节目标区间内 {hit} 个姿态线∩件")
        if hit_r > 0:
            why_bad.append(f"真实姿态 {hit_r} / {v.get('n_real_poses')} 个线∩件")
        if ten > 0:
            why_bad.append(f"肢体全关节目标区间内 {ten} 个姿态张紧（线长不够）")
        if ten_r > 0:
            why_bad.append(f"真实姿态 {ten_r} 个张紧（线长不够）")
        if _hs_num(R) and _hs_num(lim) and R < lim:
            why_bad.append(f"零位最小弯 R {R} < 限 {lim}")
        detail = ("；".join(why_bad) + "｜" if why_bad else "") + (
            f"模型 status：{v.get('status')}｜min_bend_limit={lim}；n_poses={n}；全扫描域 hit_all={v.get('hit_all')} / "
            f"tension_all={v.get('tension_all')}；扫掠中最小弯 R={v.get('worst_bend_R_swept')}；od={v.get('od_mm')}；"
            f"长 {v.get('length_mm')}；跨 {v.get('crosses')}；肢体 {v.get('limb')} {v.get('limb_joints')}（{v.get('n_poses_limb')} 姿态）；"
            f"真实姿态 {v.get('n_real_poses')}；旧口径（只扫跨越关节）hit_in_target={v.get('hit_in_target')}（不参与判）；{mode}；{reg}") + _hs_own_note(v, own_files)
        res.add(subject=sid, check="harness_sweep", state=PASS if ok else FAIL, severity=BLOCK, measured=measured,
                criterion=_HS_CRIT, evidence_n=int(n) if _hs_num(n) and n > 0 else 0, detail=detail, provenance=_HS_PROV)
        judged.add(sid)
    for rid in sorted(set(declared) - {str(k) for k in summ}):
        res.add(subject=rid, check="harness_sweep", state=NOT_RUN, severity=BLOCK, measured=None,
                criterion=_HS_CRIT, evidence_n=0,
                detail=(f"没判（MissingRoute）：harness.yaml:{declared[rid]}.route_model 登记了这条线路，"
                        f"但 sweep_summary 没返回它（线路改名 / 删了？）；{mode}"),
                provenance=_HS_PROV)
    done = set()
    for hb in bundles:
        rm = hb.get("route_model")
        if isinstance(rm, dict) and rm.get("covers_whole_bundle") is True:
            rs = [str(r) for r in (rm.get("routes") or [])]
            if rs and all(r in judged for r in rs):
                done.add(hb.get("id"))
    ev.update(routes=len(summ), judged=len(judged), bundles_covered=sorted(done),
              failed=sorted(f.subject for f in res.findings if f.check == "harness_sweep" and f.state == FAIL))
    return done, ev


def run(ctx) -> LayerResult:
    """hr42：外包一层，保证并行预取的进程池在层结束时关掉（不改 _run_impl 的任何判据）。"""
    try:
        return _run_impl(ctx)
    finally:
        _pref_close()


def _run_impl(ctx) -> LayerResult:                                 # noqa: C901
    import numpy as np
    res = LayerResult(LAYER, NAME)
    tolrec = ((ctx.data.get("tolerances") or {}).get("feature_check_tolerances") or {}) \
        .get("static_intersection_mm3") or {}
    tol = tolrec.get("max")
    if tol is None:
        res.unknown("_layer", "threshold", f"{_TOL_KEY}.max 取不到，第 6 层没有判据", provenance=_TOL_KEY)
        return res
    gap_rec = (((ctx.data.get("tolerances") or {}).get("fits") or {}).get("deck") or {}) \
        .get("deck_to_L01_gap") or {}
    gap_tol = ((gap_rec.get("target_range_mm") or {}).get("min"))
    crit = (f"运动全过程交集 ≤ {tol} mm³（{_TOL_KEY}）。碰撞姿态落在目标区间内 → FAIL(BLOCK)（件的问题）；"
            f"碰撞全部落在目标区间外（仍在上游扫描域内）→ FAIL(WARN)（工作空间问题：控制端限位，不是件的问题）。"
            f"目标区间**只认** {_TARGET_KEY} 的声明（缺失 / lo≥hi / 非数 / 超出扫描域 / 人为收窄无 date+reason → "
            f"该关节相关判据 unknown）；两两组合按两轴都在各自目标区间内算区间内")

    # tolerances.yaml 没有"运动最小间隙"这一桶 —— 只有甲板×L01 那一条 min_gap
    res.add(subject="_data", check="motion_clearance_threshold", state=FAIL, severity=BLOCK,
            measured=gap_tol,
            criterion="tolerances.yaml 应有一条**通用**的运动最小间隙阈值",
            evidence_n=1,
            detail=f"整张公差表里只有 fits.deck.deck_to_L01_gap 一条 feature_type=min_gap，"
                   f"它的 meaning 明写『第 6 层运动最小距离』、min={gap_tol}（src="
                   f"{(gap_rec.get('nominal_mm') or {}).get('src')}，status={gap_rec.get('status')}），"
                   f"但那是**甲板对 L01 这一对**的值。本层拿它当全局阈值用，"
                   f"因为没有别的可用 —— 这是数据缺口，不是判据"
            if gap_tol is not None else
            "tolerances.yaml 里连一条 min_gap 都没有，最小距离判据无从取阈值",
            provenance=_GAP_KEY)
    if gap_tol is None:
        gap_tol = None

    # ── 场景 ─────────────────────────────────────────────────────────────
    if not PLACED.exists() or not any(PLACED.glob("*.stl")):
        res.unknown("_layer", "placed_dir", "cad/duck_s288/placed/ 不存在或为空", provenance="placed/")
        return res
    try:
        man = json.loads(_MANIFEST.read_text(encoding="utf-8"))
        bfp = man.get("body_for_part") or {}
    except Exception as e:                                         # noqa: BLE001
        res.unknown("_layer", "body_for_part",
                    f"读不到 {_rel(_MANIFEST)}（placed 实体 → MJCF 连杆的对应表）：{e} —— "
                    "没有这张表就不知道哪个件跟着哪条关节动",
                    provenance="tools/sim/cad_geometry_manifest.json:body_for_part")
        return res
    res.inputs.append(_rel(_MANIFEST))
    try:
        from cad_motion_check import declared_body_map
        from duckstructure.lib import B
        declared = declared_body_map(ctx.data.get("parts") or {})
    except Exception as e:                                         # noqa: BLE001
        res.unknown("_scene", "placed_inventory", f"运动归属清单无效：{e}",
                    provenance="parts.yaml:motion_instances")
        return res
    res.inputs.append("tools/sim/cad_motion_check.py")
    stems = {p.stem for p in PLACED.glob("*.stl")}
    if not check_scene_inventory(res, stems, declared, bfp, B):
        return res
    for p in sorted(PLACED.glob("*.stl")):
        res.inputs.append(_rel(p))
    try:
        sc = _Scene(ctx, bfp)
    except Exception as e:                                         # noqa: BLE001
        res.unknown("_layer", "scene", f"场景建不起来（布尔实体转换/运动树）：{e}",
                    provenance="tools/cad/assembly_audit.py:solid / duckstructure/kin.py")
        return res
    if _memo_on() and hasattr(sc, "memo"):
        sc.memo = {}                                               # hr42：本层的组合扫描有大量完全相同的位姿对（见 _Scene._evaluate）

    # ── 关节清单 ─────────────────────────────────────────────────────────
    ja = [j for j in ((ctx.data.get("frozen") or {}).get("joint_axes") or [])]
    joints, notmjcf, notree, range_diff = [], [], [], []
    tgt, tgt_bad = {}, {}                  # 目标区间（F-L6-3）：只认 frozen.yaml 声明；坏声明 → 该关节相关判据 unknown
    for j in ja:
        nm = j.get("name")
        if not j.get("exists_as_mjcf_joint"):
            notmjcf.append(nm)
            continue
        body = sc.joint_body.get(nm)
        if body is None:
            notree.append(f"{nm}: frozen 说它是 MJCF joint，但运动树里找不到")
            continue
        rng = [float(x) for x in (j.get("range_deg") or [])]
        mj = [float(x) for x in sc.B[body]["joint"]["range_deg"]]
        # README 反例 N4「扫掠区间反」：区间必须是有序的 [lo, hi]。反写（头尾对调）时 check_angles 只会给
        # 两个端点，整段行程一个姿态都没扫 —— 那是"没查"，不是"没撞"（元规则 2/4）。声明与扫描域都判。
        bad_order = [f"frozen.yaml:joint_axes[{nm}].range_deg={rng}" for _ in [0]
                     if len(rng) != 2 or not (rng[0] < rng[1])] + \
                    [f"上游 <joint range>={mj}" for _ in [0] if len(mj) != 2 or not (mj[0] < mj[1])]
        if bad_order:
            res.unknown("_joints", f"range_declared_ordered:{nm}",
                        f"{nm} 的角度区间不是有序的 [lo, hi]：{'；'.join(bad_order)} —— 区间反了扫不出任何中间姿态，"
                        f"本关节不参加扫描（未知=失败）",
                        provenance="frozen.yaml:joint_axes[].range_deg / duckstructure/checks.py:check_angles")
            continue
        if max(abs(rng[0] - mj[0]), abs(rng[1] - mj[1])) > 1e-6:
            range_diff.append(f"{nm}: frozen {rng} vs 上游 {[round(x, 6) for x in mj]}")
        joints.append((nm, body, mj))          # 扫描域取上游区间（最宽的候选域），见下一条
        t, why = _target_range(j, mj)
        if t is None:
            tgt_bad[nm] = why
        else:
            tgt[nm] = t
    # ①「14 条关节在册、每条都在运动树里找得到」—— 这是结构完整性，与限位的权威归属无关，
    #    所以仍然是 BLOCK。
    res.add(subject="_joints", check="joint_inventory",
            state=PASS if (len(joints) == 14 and not notree) else FAIL, severity=BLOCK,
            measured=len(joints),
            criterion="frozen.yaml:joint_axes 里 exists_as_mjcf_joint 的关节数 = 14，"
                      "且每条都能在运动树（duckstructure/lib.py:B）里找到对应 body —— "
                      "**本条不判范围**，范围见 _joints/range_matches_upstream",
            evidence_n=len(ja),
            detail=f"{len(joints)} 条参加检查；exists_as_mjcf_joint=false 的：{notmjcf}"
                   + ("；**树里找不到**：" + "；".join(notree) if notree else "；每条都在树里"),
            provenance="frozen.yaml:joint_axes / duckstructure/lib.py:B")
    # ②「range_deg 必须逐条等于上游 robot_walk.xml」—— 这条判据的权威没了。
    #    2026-09-09 拍板重训（docs/重训路线_2026-09-09.md）：不再迁就原版的限位/质量/执行器上限，
    #    MJCF 由 CAD 生成（tools/sim/make_mjcf.py）。上游区间从"必须符合的真理"降级成
    #    "扫描用的机械候选域"。原来这条判 PASS，等于拿一个已作废的要求当放行依据。
    #    **不能改成"等于新生成的 robot_walk_s288.xml"** —— 那份 XML 的 joint range 正是本层
    #    自己导出的 collision_free_range，指过去就成了同义反复（更隐蔽的假绿）。
    #    所以只降级 + 说清楚，不换基准。
    # 2026-09-13（F-核-5）：退役 ≠ 过期。STALE 在 core 里是"输入变了没重跑"，会让 verdict 永远
    # INCOMPLETE；基准失去权威的判据用 RETIRED（不阻断、不计 INCOMPLETE、单独计数）。
    res.add(subject="_joints", check="range_matches_upstream", state=RETIRED, severity=WARN,
            measured=len(range_diff),
            criterion="【已退役（基准失去权威），保留为记录】frozen.yaml:joint_axes.range_deg 与上游 "
                      "robot_walk.xml 的 <joint range> 逐条一致（差 ≤1e-6°）",
            evidence_n=len(joints),
            detail=("与上游全部一致" if not range_diff else
                    f"{len(range_diff)} 条对不上：" + "；".join(range_diff))
                   + "。**但这个『一致』不再是通过的理由**：重训后 CAD + 实测是真理，"
                     "上游限位不是。本层仍按上游区间扫描，只因为它是最宽的机械候选域 —— "
                     "结论看 _ranges/collision_free_range（本层自己量出来的无碰撞区间），"
                     "那份结果与『必须等于上游』直接矛盾（导出在 tools/gate/out/mjcf_ranges.json）。"
                     "**连带作废**：frozen.yaml:joint_axes_rule.cutter_vs_mjcf_warning 里"
                     "『第 6 层必须按 MJCF 范围跑、不许按切刀范围跑』那句同样没有权威来源了，"
                     "需要跟着改 —— frozen.yaml 不归本层改，本层只在这里记一笔",
            provenance="docs/重训路线_2026-09-09.md / frozen.yaml:joint_axes.range_deg / "
                       "upstream/microduck_rl/.../robot_walk.xml")
    if not joints:
        res.unknown("_layer", "joints", "一条可用关节都没有", provenance="frozen.yaml:joint_axes")
        return res

    # ── 切刀区间（分档用） ───────────────────────────────────────────────
    from duckstructure.checks import LEGACY_LIMITS, check_angles
    cut, cut_txt, parsed = _cutter_ranges(ctx, LEGACY_LIMITS)
    nodecl = [nm for nm, _, _ in joints if nm not in cut]
    disagree = [f"{nm}: frozen 解析 {cut[nm]} vs checks.LEGACY_LIMITS {LEGACY_LIMITS[nm]}"
                for nm in cut if nm in LEGACY_LIMITS
                and max(abs(cut[nm][0] - LEGACY_LIMITS[nm][0]), abs(cut[nm][1] - LEGACY_LIMITS[nm][1])) > 1e-6]
    for nm in nodecl:                                              # 没声明的退回 LEGACY
        if nm in LEGACY_LIMITS:
            cut[nm] = tuple(float(x) for x in LEGACY_LIMITS[nm])
    res.add(subject="_cutter", check="cutter_domain_declared", state=FAIL, severity=WARN,
            measured=len(parsed),
            criterion="切刀区间应是机器可读字段，且与 duckstructure/checks.py:LEGACY_LIMITS 一致",
            evidence_n=len(joints),
            detail=f"frozen.yaml:joint_axes_rule.cutter_vs_mjcf_warning 只是一句话：{cut_txt!r}；"
                   f"本层解析出 {parsed}；未声明因而退回 LEGACY_LIMITS 的关节：{nodecl}"
                   + ("；**两个来源对不上**：" + "；".join(disagree) if disagree else "")
                   + "；另外 duckstructure/lib.py:ANK_SWEEP 现在是 62（工作区），"
                     "而 parts.yaml:source 写着现有 STL 是 ±37 那一版 —— 三个数三个地方",
            provenance="frozen.yaml:joint_axes_rule.cutter_vs_mjcf_warning / "
                       "duckstructure/checks.py:LEGACY_LIMITS")

    # ── 能力包络（frozen.yaml:capability_envelope，2026-09-16）────────────────
    # box = in-scope mode 的实际角并集（左右镜像求并，不含 margin）；per_mode 逐 mode；
    # clipped = box ± margin 夹到上游扫描域 = 『能力盒』，碰撞记录的 policy 档按它分；need_by_mode 给 _capability/* 用。
    box, per_mode, cap_mirrored, cap_missing, cap_margin = {}, {}, {}, [], 0.0
    cap = ((ctx.data.get("frozen") or {}).get("capability_envelope") or {})
    cap_err = None
    if not cap or not cap.get("modes_in_scope") or cap.get("margin_deg") is None:
        cap_err = "frozen.yaml:capability_envelope 缺失或没有 modes_in_scope / margin_deg"
    elif not _ENVELOPE.exists():
        cap_err = f"{_rel(_ENVELOPE)} 不存在 —— 拿不到原版策略实际用到的关节范围"
    else:
        res.inputs.append(_rel(_ENVELOPE))
        try:
            import derive_target_ranges as _dtr                      # tools/gate/ 在 sys.path（core 同目录）
            env = json.loads(_ENVELOPE.read_text(encoding="utf-8"))
            box, per_mode, cap_mirrored, cap_missing = _dtr.capability_box(cap, env)
            cap_margin = float(cap["margin_deg"])
            if cap_missing:
                cap_err = f"capability_envelope.modes_in_scope 里这些 mode 在包络文件里没有：{cap_missing}"
        except Exception as e:                                     # noqa: BLE001
            cap_err = f"读不了能力包络：{e}"
    if cap_err:
        res.unknown("_capability", "scope_declared", cap_err + " —— 能力判据（_capability/*）与『能力盒内』档位都判不了",
                    provenance=_CAP_KEY + " / " + _rel(_ENVELOPE))
        box, per_mode = {}, {}
    mjrange = {nm: (lo, hi) for nm, _, (lo, hi) in joints}
    clipped = {j: (max(v[0] - cap_margin, mjrange[j][0]), min(v[1] + cap_margin, mjrange[j][1]))
               for j, v in box.items() if j in mjrange}
    need_by_mode = {}                                               # joint → {mode: (lo, hi)}（含 margin，夹上游域）
    for m, jd in per_mode.items():
        for j, (lo, hi) in jd.items():
            if j in mjrange:
                need_by_mode.setdefault(j, {})[m] = (max(lo - cap_margin, mjrange[j][0]),
                                                     min(hi + cap_margin, mjrange[j][1]))
    if box:
        res.add(subject="_capability", check="scope_declared", state=PASS, severity=INFO,
                measured=len(per_mode),
                criterion=f"{_CAP_KEY}：modes_in_scope 都在包络文件里；能力盒 = 各 mode 实际角并集"
                          f"{'（左右镜像求并）' if cap.get('mirror_symmetric') else ''} ± margin_deg，夹到上游扫描域",
                evidence_n=sum(len(jd) for jd in per_mode.values()),
                detail=f"{len(per_mode)} 个 mode：{', '.join(sorted(per_mode))}；margin {cap_margin:g}°；"
                       f"out_of_scope：{', '.join(sorted((cap.get('modes_out_of_scope') or {}).keys())) or '无'}；能力盒："
                       + "、".join(f"{j} [{v[0]:.1f},{v[1]:.1f}]" for j, v in sorted(clipped.items())),
                provenance=_CAP_KEY + " / " + _rel(_ENVELOPE) + " / tools/gate/derive_target_ranges.py")

    def in_target(pose):
        """姿态是否在目标区间内（F-L6-3）：所有非零关节都在各自 target_range_deg 内 → True；
        有任一非零关节没有可用的目标区间 → None（分不出来 = unknown）；否则 False。"""
        act = {j: a for j, a in pose.items() if abs(a) > 1e-12}
        if any(j not in tgt for j in act):
            return None
        return all(tgt[j][0] - 1e-6 <= a <= tgt[j][1] + 1e-6 for j, a in act.items())

    def bucket(pose):
        """一条碰撞记录的档位：策略包络内 / 切刀区间内 / 超出切刀区间 / 目标区间内。"""
        act = {j: a for j, a in pose.items() if abs(a) > 1e-12}
        in_cut = all(cut.get(j, (-1e9, 1e9))[0] - 1e-6 <= a <= cut.get(j, (-1e9, 1e9))[1] + 1e-6
                     for j, a in act.items())
        in_pol = bool(clipped) and all(
            clipped.get(j, (0.0, 0.0))[0] - 1e-6 <= a <= clipped.get(j, (0.0, 0.0))[1] + 1e-6
            for j, a in act.items())
        return in_pol, in_cut, in_target(pose)

    stats = {"policy": [0, 0.0, None], "in_cutter": [0, 0.0, None], "out_cutter": [0, 0.0, None],
             "in_target": [0, 0.0, None], "out_target": [0, 0.0, None], "target_unknown": [0, 0.0, None]}
    all_rows = []

    pose_counts = {"policy": 0, "policy_combo": 0}

    def record(pose, hits, src):
        """记一个姿态的碰撞；返回本次追加进 all_rows 的记录（与 hits 同序，组合扫描用它回填 combo_class）。"""
        in_pol, in_cut, in_tgt = bucket(pose)
        added = []
        if in_pol:
            pose_counts["policy"] += 1
            if src.startswith("combo"):
                pose_counts["policy_combo"] += 1
        for a, b, v in hits:
            all_rows.append(dict(a=a, b=b, v=v, pose={k: round(x, 4) for k, x in pose.items()
                                                      if abs(x) > 1e-12}, src=src,
                                 policy=in_pol, in_cutter=in_cut, in_target=in_tgt))
            added.append(all_rows[-1])
            for key, on in (("policy", in_pol), ("in_cutter", in_cut), ("out_cutter", not in_cut),
                            ("in_target", in_tgt is True), ("out_target", in_tgt is False),
                            ("target_unknown", in_tgt is None)):
                if not on:
                    continue
                s = stats[key]
                s[0] += 1
                if v > s[1]:
                    s[1], s[2] = v, f"{a}×{b} @ {pose}"
        return added

    def block_flag(r):
        """一条碰撞记录算不算"件的问题"（给 sev_of）：组合扫描的记录按 hr50 v4 分类（viol → 是；light / retrain / workspace → 否；
        unknown → 分不出）；其余记录（单轴、策略包络姿态）照旧按目标区间（F-L6-3）。"""
        return _row_block_flag(r)

    def sev_of(rows_in_target):
        """一组碰撞/近距记录的严重级（F-L6-3）：有一条在目标区间内 → BLOCK；全部在区间外 → WARN；
        有分不出来的 → None（调用方判 unknown）。"""
        flags = list(rows_in_target)
        if any(f is None for f in flags):
            return None
        return BLOCK if any(flags) else WARN

    def ws_note(n_out):
        return (f"；其中 {n_out} 条落在目标区间外 —— **工作空间问题（控制端限位），不是件的问题**"
                if n_out else "")

    # ── 零位姿基线（应由第 3 层保证为 0；这里只取证，不代第 3 层判） ────
    # 件号 ↔ placed 实体（motion_collision 落格子、relations 的 part 方解析都要用；原来放在最后，F-L6-2 提前）
    stem2pid = {}
    for p in ((ctx.data.get("parts") or {}).get("parts") or []):
        pid = p.get("id")
        # ctx.placed_for 对 L06 返回 None、对 L03/L07 返回 *_R（框架 bug，第 4 层有专门的判据记着），
        # 这里自己补一遍候选名，免得整件在运动红格里消失
        cands = []
        got = ctx.placed_for(pid)
        if got is not None:
            cands.append(got.stem)
        inv = p.get("inventory_id") or ""
        short = inv.split("_", 1)[-1] if "_" in inv else inv
        cands += [inv, short, re.sub(r"_(TPU|PLA)$", "", short), pid]
        for fn in (p.get("build_fn") or []):
            cands.append(fn.replace("build_", ""))
        base = next((c[:-2] if (c.endswith("_R") and c[:-2] in sc.index) else c
                     for c in cands if c and c in sc.index), None)
        if base is None:
            res.add(subject=pid, check="motion_scope", state=FAIL, severity=WARN, measured=None,
                    criterion="每个打印件都要能对上一个 placed 实体，否则它的运动碰撞落不到本件格子上",
                    evidence_n=len(sc.names),
                    detail=f"件号 {pid} 在 placed/ 里认不出实体（试过 {cands}）",
                    provenance="tools/gate/gate.py:Ctx.placed_for")
            continue
        for s in (base, base + "_R"):
            if s in sc.index:
                stem2pid[s] = pid

    zero = {nm: 0.0 for nm, _, _ in joints}
    all_pairs = sc.moving_pairs([nm for nm, _, _ in joints])
    try:
        z_hits, _zn, _zk = sc.evaluate(zero, all_pairs, tol)
        zero_gap = sc.zero_gap_pairs(all_pairs, gap_tol) if gap_tol is not None else set()
    except ValueError as e:
        res.unknown("_zero", "boolean", f"零位姿布尔失败：{e}", provenance="duckstructure/checks.py:vol")
        return res
    # F-L6-2：最小距离判据的豁免**只认** relations.yaml 声明为 contact 的 parties 件对（读数据，不写死件号）
    name_idx = {n: i for i, n in enumerate(sc.names)}
    rels = ((ctx.data.get("relations") or {}).get("relations") or [])
    mating, n_decl, decl_bad = _contact_pairs(rels, name_idx, sc.bodies, stem2pid)
    mating = {p for p in mating if p in set(all_pairs)}
    pair_txt = lambda ps: "、".join(f"{sc.names[i]}×{sc.names[k]}" for i, k in sorted(ps))   # noqa: E731
    res.add(subject="_zero", check="declared_contact_pairs",
            state=PASS if not decl_bad else FAIL, severity=WARN, measured=len(mating),
            criterion="relations.yaml 里每条 contact 声明的 parties 都要能解析成运动场景里的实体对 —— "
                      "解析不到的声明豁免不了任何对（保守：按未声明判）",
            evidence_n=n_decl + len(decl_bad),
            detail=f"{n_decl} 条 contact 声明解析成 {len(mating)} 个运动件对（豁免出最小距离判据）：{pair_txt(mating)}"
                   + (f"；**解析不到** {len(decl_bad)} 条：" + "；".join(f"{rid}: {why}" for rid, why in decl_bad)
                      if decl_bad else ""),
            provenance="relations.yaml:*.contact.parties / parts.yaml:placed_instances")
    undeclared = sorted(zero_gap - mating)
    res.add(subject="_zero", check="undeclared_zero_gap_pair",
            state=FAIL if undeclared else PASS, severity=WARN, measured=len(undeclared),
            criterion=f"零位姿间隙 < {gap_tol} mm 但 relations.yaml 没声明为 contact 的件对数 = 0 —— "
                      "这些对**不豁免**、照常参加最小距离判据；列出来是让人去声明或去改几何，不阻断",
            evidence_n=len(all_pairs),
            detail=(f"{len(zero_gap)} 对零位姿贴得近，其中 {len(undeclared)} 对没有 contact 声明：{pair_txt(undeclared)}"
                    if undeclared else f"{len(zero_gap)} 对零位姿贴得近，全部有 contact 声明"),
            provenance="relations.yaml:*.contact.parties / tools/cad/assembly_audit.py:min_gap")
    res.add(subject="_zero", check="zero_pose_baseline", state=NOT_RUN, severity=INFO,
            measured=len(z_hits),
            criterion="零位姿交集归第 3 层判；这里只做运动记录的基线（扣掉它才知道哪些是动出来的）",
            evidence_n=len(all_pairs),
            detail=(f"零位姿 {len(z_hits)} 对超阈值：" + "；".join(f"{a}×{b} {v:.3f}" for a, b, v in z_hits[:5])
                    if z_hits else "零位姿无一对超阈值")
                   + f"；声明为 contact 的配合面 {len(mating)} 对，最小距离判据把它们排除（那是第 3/5 层的判据）："
                   + pair_txt(list(mating)[:8])
                   + f"；零位姿贴得近（间隙 < {gap_tol} mm）但没声明的 {len(undeclared)} 对**不排除**，见 undeclared_zero_gap_pair",
            provenance="README 第 2 节 L3/L6")

    # ── A 单轴全行程 ─────────────────────────────────────────────────────
    # hr42：先把 A（单轴）与 B（两两组合的统一网格）要问的姿态整批并行预取（下面循环里的 evaluate 查表命中；结果/顺序不变）
    _reqs = []
    for nm, body, (lo, hi) in joints:
        if nm in tgt_bad:
            continue
        _pp = sc.moving_pairs([nm])
        _reqs += [({nm: float(a)}, _pp, tol, gap_tol, mating) for a in check_angles(lo, hi, _STEP_DEG) if abs(a) > 1e-8]
    _chain = []
    for nm, body, _ in joints:
        par = sc.B[body]["parent"]
        while par is not None:
            pj = sc.B[par].get("joint")
            if pj:
                _chain.append((pj["name"], nm))
                break
            par = sc.B[par]["parent"]
    for j1, j2 in _chain:
        if j1 not in mjrange or j1 in tgt_bad or j2 in tgt_bad or j2 not in mjrange:
            continue
        _pp = sc.moving_pairs([j1, j2])
        _g1 = check_angles(mjrange[j1][0], mjrange[j1][1], _COMBO_STEP_DEG)
        _g2 = check_angles(mjrange[j2][0], mjrange[j2][1], _COMBO_STEP_DEG)
        _reqs += [({j1: float(a), j2: float(b)}, _pp, tol, None, frozenset())
                  for a in _g1 for b in _g2 if not (abs(a) < 1e-8 and abs(b) < 1e-8)]
    try:
        sc.prefetch(_reqs, bfp)
    except Exception as _e:                                          # noqa: BLE001  预取失败只丢速度：下面照常现算
        print(f"[L6 预取] 失败，改串行：{_e}", flush=True)
    near_rows = []
    swept = set()                       # 真正扫过的关节（目标区间可用）；tgt_bad 的关节不扫、判 unknown
    for nm, body, (lo, hi) in joints:
        if nm in tgt_bad:
            res.unknown(nm, "single_axis_sweep",
                        f"{nm}：{tgt_bad[nm]} —— 没有可用的目标区间就分不出碰撞在区间内/外（BLOCK/WARN），"
                        f"本关节不扫（未知=失败）；相关的 pair_combination / collision_free_range 同判 unknown",
                        provenance=_TARGET_KEY)
            continue
        angles = [a for a in check_angles(lo, hi, _STEP_DEG) if abs(a) > 1e-8]
        pairs = sc.moving_pairs([nm])
        rows, near, nb = [], [], 0
        try:
            for a in angles:
                h, n2, k = sc.evaluate({nm: float(a)}, pairs, tol, gap_tol, mating)
                nb += k
                record({nm: float(a)}, h, f"single:{nm}")
                rows += [(a, x) for x in h]
                near += [(a, x) for x in n2]
        except ValueError as e:
            res.unknown(nm, "single_axis_sweep", f"布尔失败（不当作 0 通过）：{e}",
                        provenance="duckstructure/checks.py:vol")
            continue
        swept.add(nm)
        near_rows += [(nm, a, x, in_target({nm: float(a)})) for a, x in near]
        incut = [(a, x) for a, x in rows
                 if cut.get(nm, (-1e9, 1e9))[0] - 1e-6 <= a <= cut.get(nm, (-1e9, 1e9))[1] + 1e-6]
        inpol = [(a, x) for a, x in rows
                 if clipped.get(nm) and clipped[nm][0] - 1e-6 <= a <= clipped[nm][1] + 1e-6]
        intgt = [(a, x) for a, x in rows if in_target({nm: float(a)})]
        peak = max((x[2] for _, x in rows), default=0.0)
        worst = max(rows, key=lambda r: r[1][2]) if rows else None
        res.add(subject=nm, check="single_axis_sweep", state=PASS if not rows else FAIL,
                severity=(BLOCK if (not rows or intgt) else WARN), measured=round(peak, 6), criterion=crit,
                evidence_n=len(angles) * len(pairs),
                detail=f"上游 MJCF 区间 [{lo:.3f}, {hi:.3f}]°（**扫描候选域，不是权威限位**），"
                       f"目标区间 [{tgt[nm][0]:.1f}, {tgt[nm][1]:.1f}]°（{_TARGET_KEY}），{len(angles)} 个角度"
                       f"（check_angles 的 {_STEP_DEG}° 独立网格 + 强制含端点，"
                       f"**不复用建模用的切刀角**），{len(pairs)} 个受影响件对，{nb} 次布尔。"
                       + (f"{len(rows)} 条碰撞记录：目标区间内 {len(intgt)} 条／外 {len(rows) - len(intgt)} 条"
                          f"{ws_note(len(rows) - len(intgt))}；切刀区间 {cut.get(nm)} 内 {len(incut)} 条／"
                          f"外 {len(rows) - len(incut)} 条；策略包络 "
                          f"{tuple(round(x, 1) for x in clipped[nm]) if clipped.get(nm) else '—'} 内 "
                          f"{len(inpol)} 条。峰值 {peak:.3f} mm³ @ {worst[0]:+.3f}° "
                          f"{worst[1][0]}×{worst[1][1]}" if rows else "全程无一对超阈值")
                       + (f"；另有 {len(near)} 条『不相交但间隙 < {gap_tol} mm』" if near else ""),
                provenance="frozen.yaml:joint_axes / duckstructure/checks.py:check_angles / " + _TOL_KEY)

    # ── B 相邻关节两两组合 ───────────────────────────────────────────────
    chain = []
    for nm, body, _ in joints:
        par = sc.B[body]["parent"]
        while par is not None:
            pj = sc.B[par].get("joint")
            if pj:
                chain.append((pj["name"], nm))
                break
            par = sc.B[par]["parent"]
    combo_rows = 0
    combo_pol, combo_env, combo_err, combo_scene = None, None, None, {}
    res.inputs += [_rel(_REAL_POSE_POLICY), _rel(_REAL_POSE_DEPTH_MODULE)]
    try:
        combo_pol = _load_real_pose_policy()
        combo_env = _combo_envelope(cap, combo_pol)
        res.inputs.append(combo_env[2]["path"])
    except Exception as e:                                                   # noqa: BLE001
        combo_err = f"组合碰撞的真实动作包络 / 口径读不了（{_rel(_REAL_POSE_POLICY)}）：{e!r}"

    def combo_depth(pa, pb, pose):
        if "s" not in combo_scene:
            combo_scene["s"] = _DepthScene(bfp)
        return _combo_depth_mm(combo_scene["s"], pa, pb, pose)      # 只取 depth_mm（contact_depth 返回的是 dict）
    combo_retrain_all, combo_approved_all = [], []
    crit_combo = (crit + f"。两两组合统一 {_COMBO_STEP_DEG}° 网格（零度对齐、强制含端点；F-L6-1：不再 max(10°, 量程/12) "
                         f"粗扫 —— 两轴都非零、宽度 < 网格的碰撞口袋粗网格看不见），峰值粗格附近再加密到 {_STEP_DEG}°")
    for j1, j2 in chain:
        if j1 not in mjrange:
            res.unknown(f"{j1}+{j2}", "pair_combination",
                        f"父关节 {j1} 不在本层关节清单里（frozen.yaml:joint_axes 没把它列为 MJCF joint 或区间无序），组合扫不了",
                        provenance="frozen.yaml:joint_axes")
            continue
        bad = [f"{j}：{tgt_bad[j]}" for j in (j1, j2) if j in tgt_bad]
        if bad:
            res.unknown(f"{j1}+{j2}", "pair_combination",
                        "；".join(bad) + " —— 没有可用的目标区间就分不出组合碰撞在区间内/外，本组合不扫（未知=失败）",
                        provenance=_TARGET_KEY)
            continue
        r1 = mjrange[j1]
        r2 = mjrange[j2]
        g1 = check_angles(r1[0], r1[1], _COMBO_STEP_DEG)
        g2 = check_angles(r2[0], r2[1], _COMBO_STEP_DEG)
        pairs = sc.moving_pairs([j1, j2])
        rows, row_recs, nb, n_refine = [], [], 0, 0          # row_recs[i] = rows[i] 在 all_rows 里的那条记录（回填 combo_class）
        try:
            for a in g1:
                for b in g2:
                    if abs(a) < 1e-8 and abs(b) < 1e-8:
                        continue
                    pose = {j1: float(a), j2: float(b)}
                    h, _n, k = sc.evaluate(pose, pairs, tol)
                    nb += k
                    row_recs += record(pose, h, f"combo:{j1}+{j2}")
                    rows += [(a, b, x) for x in h]
            # 只在最差的那个格附近加密到 2.5°，把峰值找准（检出靠上面的统一网格，不靠这里）
            if rows:
                wa, wb, _x = max(rows, key=lambda r: r[2][2])
                fa = [x for x in np.arange(wa - _COMBO_STEP_DEG, wa + _COMBO_STEP_DEG + 1e-9, _STEP_DEG)
                      if r1[0] - 1e-6 <= x <= r1[1] + 1e-6]
                fb = [x for x in np.arange(wb - _COMBO_STEP_DEG, wb + _COMBO_STEP_DEG + 1e-9, _STEP_DEG)
                      if r2[0] - 1e-6 <= x <= r2[1] + 1e-6]
                for a in fa:
                    for b in fb:
                        pose = {j1: float(a), j2: float(b)}
                        h, _n, k = sc.evaluate(pose, pairs, tol)
                        nb += k
                        n_refine += 1
                        row_recs += record(pose, h, f"combo_refine:{j1}+{j2}")
                        rows += [(float(a), float(b), x) for x in h]
        except ValueError as e:
            for rr in row_recs:
                rr["combo_class"] = "unknown"                    # 本组合判不了 → 已记下的记录在件格子 / 汇总里也按分不出处理
            res.unknown(f"{j1}+{j2}", "pair_combination", f"布尔失败（不当作 0 通过）：{e}",
                        provenance="duckstructure/checks.py:vol")
            continue
        combo_rows += len(rows)
        # 只有其中一条关节单独动时也撞的记录不算"组合才出现的"
        solo = {(x[0], x[1]) for a, b, x in rows if abs(a) < 1e-8 or abs(b) < 1e-8}
        onlyboth = [(a, b, x) for a, b, x in rows if abs(a) > 1e-8 and abs(b) > 1e-8
                    and (x[0], x[1]) not in solo]
        intgt = [(a, b, x) for a, b, x in rows if in_target({j1: float(a), j2: float(b)})]
        peak = max((x[2] for _, _, x in rows), default=0.0)
        w = max(rows, key=lambda r: r[2][2]) if rows else None
        # hr50 v4（2026-09-28 用户定，tools/gate/data/real_pose_policy.yaml:pair_combination）：逐条分类 ——
        #   目标区间外 → 工作空间 WARN（F-L6-3）；区间内原版真实动作没到过 → 重训约束（INFO）；到过 → 轻碰（C 中档）或 BLOCK
        cp = combo_pol or {}
        crit_c2 = (f"hr50 v4（{_rel(_REAL_POSE_POLICY)}:pair_combination，2026-09-28 用户定）：碰撞姿态有一轴在目标区间外 → "
                   f"工作空间问题 WARN（F-L6-3，目标区间只认 {_TARGET_KEY}）；目标区间内、网格姿态 ±{(cp.get('combo') or {}).get('reach', '?')}° "
                   f"内原版真实动作（capability_envelope.pose_steps_source，左右镜像求并，去掉极限段）从没到过 → 重训约束"
                   f"（_capability/pair_combination_retrain_constraints，不判 FAIL）；到过 → 交集 ≤ {cp.get('v_max', '?')} mm³ 且"
                   f"深度 ≤ {cp.get('d_max', '?')} mm 为轻碰，否则 FAIL(BLOCK)（深度算不出按超标）；超轻碰但落在 approved_retrain_constraints"
                   f"（hr50 v4，用户逐条批准的重训联动限位：件对 × 耦合盒 × 体积上限）盒内且 ≤ 上限 → 不 FAIL、逐条列出，超上限照 BLOCK。" + crit_combo)
        if rows and combo_err:
            for rr in row_recs:
                rr["combo_class"] = "unknown"
            res.unknown(f"{j1}+{j2}", "pair_combination", combo_err + f" —— 有 {len(rows)} 条碰撞记录分不了类（未知=失败）",
                        provenance=_rel(_REAL_POSE_POLICY))
            continue
        try:
            cls, c_ret, c_light, c_viol, c_ws, c_appr = (
                _classify_combo_rows(rows, j1, j2, combo_env, combo_pol, combo_depth, in_target, r1, r2, _COMBO_STEP_DEG,
                                     (combo_pol or {}).get("approved"))
                if rows else ([], [], [], [], [], []))
            if len(cls) != len(row_recs):
                raise ValueError(f"分类条数 {len(cls)} ≠ 碰撞记录条数 {len(row_recs)}")
        except Exception as e:                                               # noqa: BLE001
            for rr in row_recs:
                rr["combo_class"] = "unknown"
            res.unknown(f"{j1}+{j2}", "pair_combination", f"组合碰撞分类失败（不当作通过）：{e!r}",
                        provenance=_rel(_REAL_POSE_POLICY))
            continue
        for rr, c in zip(row_recs, cls):
            rr["combo_class"] = c
        combo_retrain_all += [dict(joints=f"{j1}+{j2}", **t) for t in c_ret]
        combo_approved_all += [dict(joints=f"{j1}+{j2}", **r) for r in c_appr]
        n_ret = sum(t["poses"] for t in c_ret)
        n_unk = sum(1 for c in cls if c == "unknown")
        vw = max(c_viol, key=lambda r: r["vol_mm3"]) if c_viol else None
        ww = max(c_ws, key=lambda r: r["vol_mm3"]) if c_ws else None
        if n_unk:
            res.unknown(f"{j1}+{j2}", "pair_combination", f"{n_unk} 条碰撞记录分不出在不在目标区间内（{_TARGET_KEY}）—— 分不了类",
                        provenance=_TARGET_KEY)
            continue
        res.add(subject=f"{j1}+{j2}", check="pair_combination", state=FAIL if (c_viol or c_ws) else PASS,
                severity=(BLOCK if (c_viol or not c_ws) else WARN),
                measured=round(max((r["vol_mm3"] for r in (c_viol or c_ws)), default=0.0), 6), criterion=crit_c2,
                evidence_n=(len(g1) * len(g2)) * len(pairs),
                detail=f"{j1} × {j2} 两两组合：{_COMBO_STEP_DEG}° 统一网格 {len(g1)}×{len(g2)} 个姿态"
                       f"（+ 峰值粗格附近 {_STEP_DEG}° 加密 {n_refine} 个姿态），{len(pairs)} 个受影响件对，{nb} 次布尔；"
                       f"目标区间 {j1} [{tgt[j1][0]:.1f}, {tgt[j1][1]:.1f}]° × {j2} [{tgt[j2][0]:.1f}, {tgt[j2][1]:.1f}]°。"
                       + (f"{len(rows)} 条碰撞记录（峰值 {peak:.3f} mm³ @ {j1}{w[0]:+.2f}° {j2}{w[1]:+.2f}° {w[2][0]}×{w[2][1]}；"
                          f"两轴都非零才出现的 {len(onlyboth)} 条）分类：**真实动作到过且超轻碰 {len(c_viol)} 条（BLOCK）**"
                          + (f"，最大 {vw['vol_mm3']} mm³ {vw['pair']} @ {vw['pose']}：{vw['why']}" if vw else "")
                          + f"；目标区间外 {len(c_ws)} 条{ws_note(len(c_ws))}"
                          + (f"，最大 {ww['vol_mm3']} mm³ {ww['pair']} @ {ww['pose']}" if ww else "")
                          + f"；真实动作没到过 → 重训约束 {n_ret} 条（{len(c_ret)} 块，逐块见 _capability/pair_combination_retrain_constraints）"
                          + f"；到过且轻碰 {len(c_light)} 条"
                          + ("：" + "；".join(f"{r['pair']} {r['vol_mm3']} mm³ / {r['depth_mm']} mm @ {r['pose']}" for r in c_light[:6])
                             + (f"（共 {len(c_light)} 条，只列前 6）" if len(c_light) > 6 else "") if c_light else "")
                          + (f"；**用户批准交重训 {len(c_appr)} 条**（不 FAIL）：" + "；".join(
                              f"{r['id']} {r['pair']} {r['vol_mm3']} mm³ ≤ 上限 {r['cap']} @ {r['pose']}" for r in c_appr[:6])
                             + (f"（共 {len(c_appr)} 条，只列前 6）" if len(c_appr) > 6 else "") if c_appr else "")
                          if rows else "全部姿态无一对超阈值"),
                provenance="README 第 2 节 L6『相邻关节两两组合』/ " + _TOL_KEY + " / " + _rel(_REAL_POSE_POLICY))

    if combo_env is not None:
        items = sorted(combo_retrain_all, key=lambda r: -r["max_mm3"])
        ev = combo_env[2]
        appr_pol = (combo_pol or {}).get("approved") or []
        used = sorted({r["id"] for r in combo_approved_all})
        unused = [e["id"] for e in appr_pol if e["id"] not in used]
        merged = _merge_retrain_blocks(items, combo_env)
        res.add(subject="_capability", check="pair_combination_retrain_constraints", state=PASS, severity=INFO,
                measured=dict(n_pairs=len({(r["joints"], r["pair"]) for r in items}), n_blocks=len(items), n_regions=len(merged),
                              per_joint_pair=merged, items=items,
                              approved=dict(n_rows=len(combo_approved_all), used=used, unused=unused,
                                            entries=[dict(id=e["id"], joints=e["joints"], pair="×".join(sorted(e["pair"])), q1=e["q1"], q2=e["q2"],
                                                          approved_up_to_mm3=e["cap"], date=e["date"], decided_by=e["by"], why=e["why"],
                                                          evidence=e["evidence"]) for e in appr_pol],
                                            rows=combo_approved_all)),
                criterion="相邻两关节组合里，目标区间内但原版真实动作从没到过的角落上的碰撞：不削，写成『q1、q2 不能同时进这个区间』交重训限制"
                          f"（{_rel(_REAL_POSE_POLICY)}:pair_combination；2026-09-28 用户定）。区间 = 撞点网格按连通块各自的范围、各向外扩一格"
                          f"（{_COMBO_STEP_DEG}°，真实边界在相邻无碰撞格点之间），夹到扫描域；盒子里若有原版真实样本，盒子会误伤真实动作 → 要按格点限",
                evidence_n=ev["n"],
                detail=f"包络 = {ev['path']}（sha {ev['sha']}）{ev['cases']} 段、{ev['n_raw']} 个 prefall 样本"
                       f"{'，左右镜像后 ' + str(ev['n']) + ' 个（' + '、'.join(ev['mirror']) + '）' if ev['mirror'] else '（不镜像）'}；"
                       f"去掉极限段 {ev['skipped']}。**训练用（按关节对合并，{len(merged)} 个区）**："
                       + ("；".join(f"{m['joints']}：{m['q1'][0]}∈[{m['q1'][1]:g},{m['q1'][2]:g}]° 且 {m['q2'][0]}∈[{m['q2'][1]:g},{m['q2'][2]:g}]° 不能同时"
                                    f"（{m['n_blocks']} 块 / {m['n_cells']} 格，{len(m['pairs'])} 个件对，最大 {m['max_mm3']} mm³ {m['pairs'][0]['pair']}"
                                    + (f"；⚠ 盒内有 {m['real_samples_in_box']} 个原版真实样本 → 按格点限" if m['real_samples_in_box'] else "") + "）"
                                    for m in merged) or "无")
                       + f"。逐件对 {len(items)} 块："
                       + ("；".join(f"{r['pair']}（{r['joints']}）：{r['q1_pad'][0]}∈[{r['q1_pad'][1]:g}, {r['q1_pad'][2]:g}]° 且 "
                                    f"{r['q2_pad'][0]}∈[{r['q2_pad'][1]:g}, {r['q2_pad'][2]:g}]° 不能同时（撞点网格 "
                                    f"[{r['q1'][1]:g}, {r['q1'][2]:g}]×[{r['q2'][1]:g}, {r['q2'][2]:g}]，{r['poses']} 格），"
                                    f"最大 {r['max_mm3']} mm³ / 深 {r['depth_mm']} mm"
                                    + (f"；⚠ 盒内有 {r['real_samples_in_pad_box']} 个原版真实样本 → 按格点限" if r["real_samples_in_pad_box"] else "")
                                    for r in items) or "无")
                       + f"。用户批准的重训联动限位（approved_retrain_constraints，{len(appr_pol)} 条）："
                       + ("；".join(f"{e['id']} {'+'.join(e['joints'])} {'×'.join(sorted(e['pair']))} "
                                    f"{e['joints'][0]}∈[{e['q1'][0]:g},{e['q1'][1]:g}]° 且 {e['joints'][1]}∈[{e['q2'][0]:g},{e['q2'][1]:g}]° "
                                    f"≤ {e['cap']} mm³（{e['date']}，{e['by']}）" for e in appr_pol) or "无")
                       + f"；本次命中 {len(combo_approved_all)} 条（{', '.join(used) or '无'}）"
                       + (f"；**登记了但没命中：{unused}**（盒内没有一条『到过且超轻碰且 ≤ 上限』的记录：几何已不撞、或体积超了上限（那些记录在 pair_combination 里按 BLOCK 报）、或登记的件对 / 盒子 / 关节写错 —— 要核）" if unused else ""),
                provenance=_rel(_REAL_POSE_POLICY) + " / frozen.yaml:capability_envelope.pose_steps_source / mirror_pairs")

    # ── C 策略包络：各 mode 的极端姿态（全 14 轴一起摆） ────────────────
    if clipped:
        allpairs = all_pairs
        pol_rows, pol_recs, nb = [], [], 0          # pol_rows[i] = (mode, tag, 全 14 轴姿态, (a, b, v))；pol_recs[i] = all_rows 里对应记录（回填 pose_class）
        try:
            for mode, jr in sorted(per_mode.items()):
                mid = {j: (max(v[0], mjrange[j][0]) + min(v[1], mjrange[j][1])) / 2
                       for j, v in jr.items() if j in mjrange}
                poses = [("all_lo", {j: max(v[0], mjrange[j][0]) for j, v in jr.items() if j in mjrange}),
                         ("all_hi", {j: min(v[1], mjrange[j][1]) for j, v in jr.items() if j in mjrange})]
                for j, v in sorted(jr.items()):
                    if j not in mjrange:
                        continue
                    for tag, a in (("lo", max(v[0], mjrange[j][0])), ("hi", min(v[1], mjrange[j][1]))):
                        p = dict(mid)
                        p[j] = a
                        poses.append((f"{j}_{tag}", p))
                for tag, p in poses:
                    h, _n, k = sc.evaluate(p, allpairs, tol)
                    nb += k
                    pol_recs += record(p, h, f"policy:{mode}:{tag}")
                    pol_rows += [(mode, tag, p, x) for x in h]
        except ValueError as e:
            for rr in pol_recs:
                rr["pose_class"] = "unknown"
            res.unknown("_policy", "policy_extreme_poses", f"布尔失败（不当作 0 通过）：{e}",
                        provenance="duckstructure/checks.py:vol")
        else:
            # hr50 v6（2026-09-28 用户定 C，real_pose_policy.yaml:policy_extreme_poses）：角点记录按 pair_combination 同一口径逐条分类；
            #   "到过" = 全部 14 轴同时 ±reach 内有原版样本；没到过 → 重训约束 INFO；只有"到过且超轻碰"算件的问题
            cp = combo_pol or {}
            pep = cp.get("pep")
            crit_pep = (f"hr50 v6（{_rel(_REAL_POSE_POLICY)}:policy_extreme_poses，2026-09-28 用户定）：角点姿态记录逐条分类 —— 有一轴在目标区间外 → "
                        f"工作空间 WARN（F-L6-3）；区间内、全部 14 轴同时 ±{(cp.get('combo') or {}).get('reach', '?')}° 内原版真实动作"
                        f"（同 pair_combination 的样本：pose_steps_source、去极限段、左右镜像）从没到过 → 重训约束（_policy/policy_extreme_poses_retrain_constraints，"
                        f"不判 FAIL）；到过 → 交集 ≤ {cp.get('v_max', '?')} mm³ 且深度 ≤ {cp.get('d_max', '?')} mm 为轻碰，落在 approved_retrain_constraints 盒内"
                        f"（按条目两条关节在姿态里的角度）且 ≤ 上限 → 用户批准，否则 FAIL(BLOCK)（深度算不出按超标）。" + crit)
            head_txt = (f"{len(per_mode)} 个 mode，每个取 all_lo / all_hi / 每条关节各自到端点（其余关节取该 mode 区间中点）= 30 个 14 轴同时摆的姿态，"
                        f"{len(allpairs)} 个件对，{nb} 次布尔。**这是包络盒的角点，不是可达集**（策略未必真同时走到这些角）；角点盒来源 {_rel(_ENVELOPE)}"
                        f"（09-09 逐 mode 统计）与『到过』样本 pose_steps_source（09-16 逐步记录）不是同一批数据，角点盒比逐步记录宽。")
            if pol_rows and combo_err:
                for rr in pol_recs:
                    rr["pose_class"] = "unknown"
                res.unknown("_policy", "policy_extreme_poses", combo_err + f" —— 有 {len(pol_rows)} 条角点碰撞记录分不了类（未知=失败）。" + head_txt,
                            provenance=_rel(_REAL_POSE_POLICY))
            elif pol_rows and not pep:
                for rr in pol_recs:
                    rr["pose_class"] = "unknown"
                res.unknown("_policy", "policy_extreme_poses", f"{_rel(_REAL_POSE_POLICY)} 没有 policy_extreme_poses 段（hr50 v6）—— 有 {len(pol_rows)} 条角点碰撞记录"
                            f"分不了类（未知=失败）。" + head_txt, provenance=_rel(_REAL_POSE_POLICY))
            else:
                try:
                    cls, p_ret, p_light, p_viol, p_ws, p_appr = (
                        _classify_pose_rows(pol_rows, combo_env, combo_pol, combo_depth, in_target, cp.get("approved"))
                        if pol_rows else ([], [], [], [], [], []))
                    if len(cls) != len(pol_recs):
                        raise ValueError(f"分类条数 {len(cls)} ≠ 碰撞记录条数 {len(pol_recs)}")
                except Exception as e:                                       # noqa: BLE001
                    for rr in pol_recs:
                        rr["pose_class"] = "unknown"
                    res.unknown("_policy", "policy_extreme_poses", f"角点碰撞分类失败（不当作通过）：{e!r}。" + head_txt,
                                provenance=_rel(_REAL_POSE_POLICY))
                else:
                    for rr, c in zip(pol_recs, cls):
                        rr["pose_class"] = c
                    peak_all = max((x[2] for _, _, _, x in pol_rows), default=0.0)
                    peak_v = max((r["vol_mm3"] for r in p_viol), default=0.0)
                    peak_w = max((r["vol_mm3"] for r in p_ws), default=0.0)

                    def _fmt(r):
                        return (f"{r['mode']}/{r['tag']} {r['pair']} {r['vol_mm3']} mm³"
                                + (f" / 深 {r['depth_mm']} mm" if r.get("depth_mm") is not None else "")
                                + (f"（{r['why']}）" if r.get("why") else ""))
                    res.add(subject="_policy", check="policy_extreme_poses",
                            state=PASS if not (p_viol or p_ws) else FAIL,
                            severity=BLOCK if (p_viol or not p_ws) else WARN,
                            measured=round(peak_v if p_viol else (peak_w if p_ws else 0.0), 6), criterion=crit_pep, evidence_n=nb,
                            detail=head_txt
                                   + (f"{len(pol_rows)} 条碰撞记录（峰值 {peak_all:.3f} mm³），按 hr50 v6：到过且超轻碰 {len(p_viol)}、用户批准交重训 {len(p_appr)}、"
                                      f"到过且轻碰 {len(p_light)}、没到过（重训约束）{len(p_ret)}、目标区间外 {len(p_ws)}、分不出 {cls.count('unknown')}（只有第一类算件的问题）。"
                                      + ("**到过且超轻碰**：" + "；".join(_fmt(r) for r in p_viol) + "。" if p_viol else "")
                                      + ("用户批准：" + "；".join(f"{_fmt(r)} ≤ {r['cap']}（{r['id']}）" for r in p_appr) + "。" if p_appr else "")
                                      + ("轻碰：" + "；".join(_fmt(r) for r in p_light) + "。" if p_light else "")
                                      + (f"没到过 → 重训约束 {len(p_ret)} 条（明细在 _policy/policy_extreme_poses_retrain_constraints；最大 {_fmt(p_ret[0])}）。" if p_ret else "")
                                      + (f"目标区间外 {len(p_ws)} 条（工作空间问题，控制端限位，峰值 {peak_w:.3f} mm³）。" if p_ws else "")
                                      if pol_rows else "无一对超阈值"),
                            provenance=_rel(_ENVELOPE) + " / " + _rel(_REAL_POSE_POLICY) + " / " + _TOL_KEY)
                    if pol_rows:
                        res.add(subject="_policy", check="policy_extreme_poses_retrain_constraints", state=PASS, severity=INFO,
                                measured=dict(n=len(p_ret), items=p_ret),
                                criterion="角点姿态里目标区间内、但原版真实动作（全部 14 轴同时 ±reach 内）从没到过的碰撞：不削，交重训限制"
                                          "（角点 = 该 mode 各关节同时到包络端点；训练侧要保证策略不同时走到这些角）",
                                evidence_n=len(pol_rows),
                                detail=("；".join(f"{r['mode']}/{r['tag']}：{r['pair']} {r['vol_mm3']} mm³ / 深 {r['depth_mm']} mm @ {r['pose']}" for r in p_ret) or "无"),
                                provenance=_rel(_REAL_POSE_POLICY) + " / " + _rel(_ENVELOPE))

    # ── D 最小距离 ───────────────────────────────────────────────────────
    if gap_tol is not None:
        worst_gap = min((x[2] for _, _, x, _t in near_rows), default=None)
        byp = {}
        for nm, a, x, _t in near_rows:
            byp.setdefault((x[0], x[1]), []).append((x[2], nm, a))
        n_in = sum(1 for _n, _a, _x, t in near_rows if t)
        res.add(subject="_clearance", check="min_clearance",
                state=PASS if not near_rows else FAIL,
                severity=(BLOCK if (not near_rows or n_in) else WARN),
                measured=None if worst_gap is None else round(worst_gap, 6),
                criterion=f"运动过程中不相交的件对最小距离 ≥ {gap_tol} mm（{_GAP_KEY}）—— "
                          f"交集 = 0 不等于装得上：打印公差一来，间隙 0 的地方实物就卡死。"
                          f"豁免**只认** relations.yaml 声明为 contact 的 parties 件对（零位姿贴得近不是豁免依据，"
                          f"见 _zero/undeclared_zero_gap_pair）；近距姿态在目标区间内 → BLOCK，全部在区间外 → WARN"
                          f"（目标区间只认 {_TARGET_KEY}）",
                evidence_n=sc.gaps,
                detail=f"单轴全行程里共做了 {sc.gaps} 次 min_gap（搜索半径 {_GAP_SEARCH} mm，"
                       f"只对 AABB 距离 ≤ {gap_tol} 的对做；豁免 {len(mating)} 对声明 contact）。"
                       + (f"{len(near_rows)} 处『不相交但间隙 < {gap_tol}』（目标区间内 {n_in} 条"
                          f"{ws_note(len(near_rows) - n_in)}），涉及 {len(byp)} 个件对："
                          + "；".join(f"{a}×{b} 最小 {min(r[0] for r in rs):.4f} mm"
                                      f"（{min(rs)[1]} {min(rs)[2]:+.2f}°）"
                                      for (a, b), rs in sorted(byp.items(),
                                                               key=lambda kv: min(r[0] for r in kv[1]))[:8])
                          if near_rows else "没有一对落在阈值以内"),
                provenance=_GAP_KEY + " / tools/cad/assembly_audit.py:min_gap / relations.yaml:*.contact.parties")

    # ── 三档汇总 + 落到件的格子 ─────────────────────────────────────────
    bucket_txt = (f"共 {len(all_rows)} 条碰撞记录（一条 = 一个姿态下的一个件对）："
                  + "；".join(f"{k} {v[0]} 条，峰值 {v[1]:.3f} mm³（{v[2]}）"
                              for k, v in (("目标区间内", stats["in_target"]),
                                           ("目标区间外", stats["out_target"]),
                                           ("目标区间分不出", stats["target_unknown"]),
                                           ("策略包络内", stats["policy"]),
                                           ("切刀区间内", stats["in_cutter"]),
                                           ("超出切刀区间", stats["out_cutter"])))
                  + ws_note(stats["out_target"][0])
                  + f"；其中组合姿态贡献 {combo_rows} 条。共 {sc.booleans} 次布尔。"
                    f"切刀区间来源见 _cutter/cutter_domain_declared")
    n_cc = {c: sum(1 for r in all_rows if r.get("combo_class") == c) for c in ("viol", "approved", "light", "retrain", "workspace", "unknown")}
    n_pc = {c: sum(1 for r in all_rows if r.get("pose_class") == c) for c in ("viol", "approved", "light", "retrain", "workspace", "unknown")}
    bucket_txt += (f"组合姿态记录按 hr50 v4 分类：到过且超轻碰 {n_cc['viol']}、用户批准交重训 {n_cc['approved']}、到过且轻碰 {n_cc['light']}、"
                   f"没到过（重训约束）{n_cc['retrain']}、目标区间外 {n_cc['workspace']}、分不出 {n_cc['unknown']}（只有第一类算件的问题）。"
                   f"角点姿态记录按 hr50 v6 分类：到过且超轻碰 {n_pc['viol']}、用户批准交重训 {n_pc['approved']}、到过且轻碰 {n_pc['light']}、"
                   f"没到过（重训约束）{n_pc['retrain']}、目标区间外 {n_pc['workspace']}、分不出 {n_pc['unknown']}（同上，只有第一类算件的问题）。")
    sev_sum = sev_of(block_flag(r) for r in all_rows)
    if sev_sum is None:
        res.unknown("_summary", "collision_buckets",
                    f"{stats['target_unknown'][0]} 条碰撞记录的姿态含没有可用目标区间的关节（{sorted(tgt_bad)}），"
                    f"{n_cc['unknown']} 条组合记录 / {n_pc['unknown']} 条角点记录分不了类 —— 分不出件的问题还是工作空间问题 —— " + bucket_txt, provenance=_TARGET_KEY)
    else:
        res.add(subject="_summary", check="collision_buckets",
                state=PASS if not all_rows else FAIL, severity=sev_sum,
                measured=len(all_rows),
                criterion="碰撞记录必须按『目标区间内 / 目标区间外』（F-L6-3，决定 BLOCK/WARN：区间内有一条即 BLOCK，"
                          f"全部在区间外为 WARN；目标区间只认 {_TARGET_KEY}；组合姿态记录按 hr50 v4、角点姿态记录按 hr50 v6 分类，只有『真实动作到过且超轻碰』算区间内的件问题）"
                          "以及『策略包络内 / 切刀区间内 / 超出切刀区间』"
                          "分开计数 —— 混在一个数字里就分不清哪些是件的问题、哪些是工作空间问题（控制端限位）",
                evidence_n=sc.booleans,
                detail=bucket_txt,
                provenance=_TARGET_KEY + " / frozen.yaml:joint_axes_rule.cutter_vs_mjcf_warning / " + _TOL_KEY)

    # ── 2026-09-09 重训之后：这一层从"判"改成"产" ──────────────────────
    # 以前 MJCF 的 range 是冻结输入，CAD 在这个区间里撞了也只能红着 —— 因为改区间
    # 等于策略作废。既然已决定重训（docs/重训路线_2026-09-09.md），关节区间就成了
    # **我们自己定的东西**。这里从单轴扫描直接导出「每个关节最大的、含 home 的、
    # 已验证无碰撞的角度区间」，写成一份可以直接喂进新 MJCF 的文件。
    frozen_axes = ((ctx.data.get("frozen") or {}).get("joint_axes") or [])
    home_deg = {j.get("name"): math.degrees(float(j.get("home_rad") or 0.0))
                for j in frozen_axes if j.get("name")}
    bad_at: dict[str, set] = {}
    for r in all_rows:
        src = str(r.get("src") or "")
        if not src.startswith("single:"):
            continue
        j = src.split(":", 1)[1]
        bad_at.setdefault(j, set()).add(float((r.get("pose") or {}).get(j, 0.0)))

    ranges_out, shrunk, home_bad, unscanned = {}, [], [], []
    for nm, _u, (lo, hi) in joints:
        if nm not in swept:
            unscanned.append(nm); continue
        bad = sorted(bad_at.get(nm, ()))
        h = home_deg.get(nm, 0.0)
        if any(abs(a - h) <= _STEP_DEG / 2 for a in bad):
            home_bad.append(nm); ranges_out[nm] = None; continue
        below = [a for a in bad if a < h]
        above = [a for a in bad if a > h]
        # 退到「最后一格确实扫干净的角度」：撞点再退一整格 _STEP_DEG。
        # 不取撞点与前一格的中点 —— 那一段根本没扫过，不许当成已验证。
        new_lo = max(lo, max(below) + _STEP_DEG) if below else lo
        new_hi = min(hi, min(above) - _STEP_DEG) if above else hi
        ranges_out[nm] = [round(new_lo, 4), round(new_hi, 4)]
        if new_lo > lo + 1e-9 or new_hi < hi - 1e-9:
            shrunk.append(f"{nm} [{lo:.1f},{hi:.1f}]→[{new_lo:.1f},{new_hi:.1f}]")

    out_p = _RANGES_OUT
    try:
        out_p.parent.mkdir(parents=True, exist_ok=True)
        out_p.write_text(json.dumps(
            {"note": "第 6 层单轴扫描导出的无碰撞区间，单位度。喂给新 MJCF 的 <joint range> 前"
                     "必须先看 _ranges.combo_collision_inside_free_range —— 单轴干净不等于组合干净。",
             "step_deg": _STEP_DEG, "source": "tools/gate/layers/l6_motion.py",
             "mjcf_range_deg": {nm: [lo, hi] for nm, _u, (lo, hi) in joints},
             "target_range_deg": {nm: list(v) for nm, v in tgt.items()},
             "home_deg": home_deg,
             "collision_free_range_deg": ranges_out,
             "unscanned_no_target_range": unscanned}, ensure_ascii=False, indent=1),
            encoding="utf-8")
        wrote = str(out_p.relative_to(ROOT)) if str(out_p).startswith(str(ROOT)) else str(out_p)
    except Exception as e:                                          # noqa: BLE001
        wrote = f"(写不出来：{e})"

    # ── F-L6-3：collision_free_range 改判"生成的 MJCF ⊆ 无碰撞区间"（BLOCK），另判"⊇ 目标区间"（WARN） ──
    #    生成的 sim/duck_s288/robot_walk_s288.xml 是独立产物（tools/sim/make_mjcf.py 读上一次导出的 mjcf_ranges.json），
    #    几何一改它就过期 —— 所以这不是同义反复。"需要收窄"本身不再是 BLOCK 的理由（只写进 detail）。
    shrink_txt = (f"{len(joints)} 个关节（本轮扫了 {len(swept)} 个）；本轮无碰撞区间导出 → {wrote}。"
                  + (f"**home 姿态本身就碰**的关节 {home_bad} —— 收窄限位救不了，这些必须改几何；" if home_bad else "")
                  + (f"相对上游扫描域需要收窄 {len(shrunk)} 个：" + "；".join(shrunk)
                     if shrunk else "没有一个关节需要收窄（单轴全行程干净）")
                  + (f"；未扫（目标区间不可用）：{unscanned}" if unscanned else ""))
    gen_rng, gen_err = {}, None
    if not _GEN_MJCF.exists():
        gen_err = f"{_rel(_GEN_MJCF)} 不存在"
    else:
        try:
            gen_rng = _mjcf_joint_ranges_deg(_GEN_MJCF)
        except Exception as e:                                      # noqa: BLE001
            gen_err = f"{_rel(_GEN_MJCF)} 解析不了：{e}"
    crit_free = (f"生成的 {_rel(_GEN_MJCF)} 每条 <joint range> ⊆ 本轮单轴扫描导出的无碰撞区间（含 home、撞点退一格 "
                 f"{_STEP_DEG}°）—— 训练域里不许有实物会撞的角度。那份 XML 是独立产物会过期，所以每轮重判；"
                 f"**不再**因为『相对上游需要收窄』本身判红（收窄只记在 detail）")
    crit_cover = (f"生成的 {_rel(_GEN_MJCF)} 每条 <joint range> ⊇ {_TARGET_KEY} —— 否则训练域被收窄到目标工作空间之内"
                  f"（策略学不到目标区间的边角；WARN，可豁免）。目标区间只认 frozen.yaml 声明")
    if gen_err:
        res.unknown("_ranges", "collision_free_range", f"{gen_err} —— 核对不了训练域是否落在无碰撞区间内。{shrink_txt}",
                    provenance=_rel(_GEN_MJCF) + " / tools/sim/make_mjcf.py")
        res.unknown("_ranges", "gen_range_covers_target", f"{gen_err} —— 核对不了训练域是否覆盖目标区间",
                    provenance=_rel(_GEN_MJCF) + " / " + _TARGET_KEY, severity=WARN)
    elif unscanned:
        res.unknown("_ranges", "collision_free_range",
                    f"{len(unscanned)} 个关节没有可用目标区间、本轮没扫（{unscanned}），它们的无碰撞区间未知，"
                    f"生成的 MJCF 的 range 无从核对。{shrink_txt}", provenance=_TARGET_KEY)
        res.unknown("_ranges", "gen_range_covers_target",
                    f"{len(unscanned)} 个关节没有可用目标区间（{unscanned}）：" + "；".join(f"{j}：{tgt_bad[j]}" for j in unscanned),
                    provenance=_TARGET_KEY, severity=WARN)
    else:
        viol, cover_bad, gen_missing, lines = [], [], [], []
        for nm, _u, (lo, hi) in joints:
            g = gen_rng.get(nm)
            if g is None:
                gen_missing.append(nm); continue
            fr = ranges_out.get(nm)
            if fr is None:
                viol.append(f"{nm}: home 本身就碰，没有无碰撞区间可言（生成 [{g[0]:.1f},{g[1]:.1f}]）")
            elif g[0] < fr[0] - 1e-3 or g[1] > fr[1] + 1e-3:
                viol.append(f"{nm}: 生成 [{g[0]:.1f},{g[1]:.1f}] ⊄ 无碰撞 [{fr[0]:.1f},{fr[1]:.1f}]")
            t = tgt[nm]
            if g[0] > t[0] + 1e-3 or g[1] < t[1] - 1e-3:
                cover_bad.append(f"{nm}: 生成 [{g[0]:.1f},{g[1]:.1f}] ⊉ 目标 [{t[0]:.1f},{t[1]:.1f}]")
            lines.append(f"{nm} 生成 [{g[0]:.1f},{g[1]:.1f}] / 无碰撞 "
                         f"{'—' if fr is None else f'[{fr[0]:.1f},{fr[1]:.1f}]'} / 目标 [{t[0]:.1f},{t[1]:.1f}]")
        if gen_missing:
            res.unknown("_ranges", "collision_free_range",
                        f"生成的 MJCF 里没有这些关节的 <joint range>：{gen_missing}。{shrink_txt}",
                        provenance=_rel(_GEN_MJCF))
            res.unknown("_ranges", "gen_range_covers_target",
                        f"生成的 MJCF 里没有这些关节的 <joint range>：{gen_missing}",
                        provenance=_rel(_GEN_MJCF), severity=WARN)
        else:
            res.add(subject="_ranges", check="collision_free_range",
                    state=FAIL if viol else PASS, severity=BLOCK, measured=len(viol),
                    criterion=crit_free, evidence_n=len(joints),
                    detail=(f"{len(viol)} 条 <joint range> 伸进了会撞的角度：" + "；".join(viol) + "。"
                            if viol else "生成的 MJCF 每条 range 都在本轮无碰撞区间内。")
                           + shrink_txt + "。逐条：" + "；".join(lines),
                    provenance=_rel(_GEN_MJCF) + " / tools/sim/make_mjcf.py / docs/重训路线_2026-09-09.md")
            res.add(subject="_ranges", check="gen_range_covers_target",
                    state=FAIL if cover_bad else PASS, severity=WARN, measured=len(cover_bad),
                    criterion=crit_cover, evidence_n=len(joints),
                    detail=(f"{len(cover_bad)} 条训练域比目标区间窄（训练域被收窄）：" + "；".join(cover_bad)
                            if cover_bad else "生成的 MJCF 每条 range 都覆盖目标区间"),
                    provenance=_rel(_GEN_MJCF) + " / " + _TARGET_KEY)

    # 单轴干净 ≠ 组合干净。收窄单轴区间修不掉的，就是这些。
    inside = [r for r in all_rows
              if str(r.get("src") or "").startswith("combo")
              and all((ranges_out.get(j) is not None
                       and ranges_out[j][0] - 1e-9 <= a <= ranges_out[j][1] + 1e-9)
                      for j, a in (r.get("pose") or {}).items())]
    ins_pairs = sorted({f"{r['a']}×{r['b']}" for r in inside})
    n_ins_tgt = sum(1 for r in inside if r["in_target"])
    sev_ins = sev_of(r["in_target"] for r in inside)
    res.add(subject="_ranges", check="combo_collision_inside_free_range",
            state=FAIL if inside else PASS, severity=WARN,
            measured=len(inside),
            criterion="落在单轴无碰撞区间**之内**的组合姿态碰撞数 = 0 —— 这些是收窄单轴限位修不掉的。"
                      f"（2026-09-16 起一律 WARN：目标区间是每关节各自的盒子，两轴同时到角落原版未必做过 —— README A2；"
                      f"阻断判据改为 _capability/real_pose_collisions 按原版真实姿态判）"
                      f"两轴都在目标区间内 → 原 BLOCK（只能改几何）；全部在目标区间外 → WARN（工作空间问题，控制端限位）；"
                      f"目标区间只认 {_TARGET_KEY}",
            evidence_n=sum(1 for r in all_rows if str(r.get("src") or "").startswith("combo")),
            detail=(f"{len(inside)} 条组合碰撞记录落在单轴无碰撞区间内（目标区间内 {n_ins_tgt} 条"
                    f"{ws_note(len(inside) - n_ins_tgt)}），涉及 {len(ins_pairs)} 个件对："
                    + "、".join(ins_pairs[:8]) + ("…" if len(ins_pairs) > 8 else "")
                    + f"；峰值 {max((r['v'] for r in inside), default=0):.3f} mm³"
                    if inside else "单轴无碰撞区间内没有组合碰撞 —— 单轴收窄就够了"),
            provenance="docs/重训路线_2026-09-09.md / " + _TARGET_KEY)

    # ── 能力判据（2026-09-16，frozen.yaml:capability_envelope）─────────────────
    # ① 每个关节：每个 in-scope mode 的包络 ± margin ⊆ 本层量出的无碰撞区间（单轴）→ 否则该 mode 的能力受损 = BLOCK。
    #    这不是拿目标区间自证：目标区间由包络推导，无碰撞区间由件扫出来，两者独立；件不够转就红。
    # ② 能力盒内（clipped）的组合姿态碰撞 = 0 —— 单轴够转、组合起来撞也是能力受损。
    if box:
        crit_cap = (f"能力包络（{_CAP_KEY}）：每个 in-scope mode 的实际角 ± margin_deg"
                    f"{'（左右镜像求并）' if cap.get('mirror_symmetric') else ''} ⊆ 本层单轴扫描的无碰撞区间"
                    f"（撞点退一格 {_STEP_DEG}°）。用户 2026-09-16 定：能力与原版一致、转角可不同 —— 包络是能力要求，不是历史参考")
        for nm in sorted(need_by_mode):
            needs = need_by_mode[nm]
            free = ranges_out.get(nm)
            if nm not in swept or free is None:
                why = ("没扫（目标区间不可用）" if nm not in swept else "home 姿态就撞，无碰撞区间不存在")
                res.unknown(nm, "capability_envelope", f"{nm}：{why} —— 判不了 {len(needs)} 个 mode 的包络是否可达",
                            provenance=_CAP_KEY + " / " + _rel(_ENVELOPE))
                continue
            bad = {m: (lo, hi) for m, (lo, hi) in needs.items()
                   if lo < free[0] - 1e-6 or hi > free[1] + 1e-6}
            worst = max((max(free[0] - lo, 0.0, hi - free[1]) for lo, hi in bad.values()), default=0.0)
            res.add(subject=nm, check="capability_envelope",
                    state=FAIL if bad else PASS, severity=BLOCK, measured=round(worst, 3),
                    criterion=crit_cap, evidence_n=len(needs),
                    detail=(f"{nm} 无碰撞区间 [{free[0]:.1f}, {free[1]:.1f}]；{len(bad)}/{len(needs)} 个 mode 的包络够不到："
                            + "；".join(f"{m} 需要 [{lo:.1f},{hi:.1f}]" for m, (lo, hi) in sorted(bad.items()))
                            + f"；最大缺口 {worst:.1f}°"
                            if bad else
                            f"{nm} 无碰撞区间 [{free[0]:.1f}, {free[1]:.1f}] 覆盖 {len(needs)} 个 mode 的包络："
                            + "；".join(f"{m} [{lo:.1f},{hi:.1f}]" for m, (lo, hi) in sorted(needs.items()))),
                    provenance=_CAP_KEY + " / " + _rel(_ENVELOPE) + " / tools/gate/derive_target_ranges.py")
        pol_rows = [r for r in all_rows if r.get("policy")]
        pol_pairs = sorted({f"{r['a']}×{r['b']}" for r in pol_rows})
        pol_combo = [r for r in pol_rows if str(r.get("src") or "").startswith("combo")]
        res.add(subject="_capability", check="box_collisions",
                state=FAIL if pol_rows else PASS, severity=WARN, measured=len(pol_rows),
                criterion=f"能力盒（{_CAP_KEY} 各 mode 实际角并集 ± margin，夹到上游扫描域）内的碰撞记录 = 0，"
                          f"单轴与组合都算。**上界**（README A2：盒是轴对齐包围盒，角落原版未必到过）→ WARN；"
                          f"阻断判据是 _capability/real_pose_collisions（原版真实姿态）",
                evidence_n=pose_counts["policy"],
                detail=(f"{len(pol_rows)} 条碰撞记录落在能力盒内（其中组合姿态 {len(pol_combo)} 条），涉及 {len(pol_pairs)} 个件对："
                        + "、".join(pol_pairs[:10]) + ("…" if len(pol_pairs) > 10 else "")
                        + f"；峰值 {max((r['v'] for r in pol_rows), default=0):.3f} mm³（{stats['policy'][2]}）"
                        if pol_rows else
                        f"能力盒内扫过 {pose_counts['policy']} 个姿态（组合 {pose_counts['policy_combo']}），无碰撞"),
                provenance=_CAP_KEY + " / " + _rel(_ENVELOPE))
        _real_pose_collisions(res, cap, tol)

    # ── 训练用的模型：接口对不对、看不看得见这些碰撞 ──────────────────────
    # 上游 microduck_velocity_env_cfg.py:319 的原话：
    #   "Self-collision penalty: discourages legs from crashing into the trunk battery holder
    #    (... on leg, leg_2, battery_holder). **With proper joint-range limits the policy
    #    can't actually reach the body**, but a positive signal here keeps it well clear."
    # 原版靠「限位保证够不到 + 惩罚项保险」两条腿。我们的几何跟原版不一样（为适配 S288 重画的法兰凸台/
    # 摆臂/惰轮凸台/安装耳，H01/H03 重雕、N02/N03 新做；S288 与 XL330 外形档相同），原版那套限位对我们不成立，所以必须自己查 ——
    # 而且要查**我们自己生成的那份**（sim/duck_s288/robot_walk_s288.xml），不是上游那份。
    import xml.etree.ElementTree as _ET
    from duckstructure import kin as _kin
    _UP_MJCF = Path(_kin.MD) / "robot_walk.xml"
    body_of = dict(zip(sc.names, sc.bodies))

    def _reachable(pose):
        """这个姿态在本层导出的收窄限位之内吗？保守：区间没导出 / home 本身就碰 → 算可达。"""
        for j, a in (pose or {}).items():
            if j not in ranges_out:
                return True
            r = ranges_out[j]
            if r is None:
                return True
            if not (r[0] - 1e-9 <= float(a) <= r[1] + 1e-9):
                return False
        return True

    # ① 训练环境按**名字**抓的东西，生成的 MJCF 里有没有 ────────────────
    #    上游 ContactSensorCfg / foot_frictions_geom_names / site_names 全是硬编码名字，
    #    名字对不上 → 传感器抓不到 → air_time / foot_slip / 摩擦随机化全部失效，
    #    模型照样能编译、训练照样能跑，只是奖励是错的。这比"没有消费者"严重一档。
    #    名字与正则一律从上游配置源码解析（元规则 7），本层不写死。
    if _GEN_MJCF.exists():
        res.inputs.append(_rel(_GEN_MJCF))       # 元规则 6：结论绑到这份 MJCF 的哈希上
    req, req_err = None, None
    if _ENV_CFG.exists():
        res.inputs.append(_rel(_ENV_CFG))
        try:
            req = _train_interface(_ENV_CFG.read_text(encoding="utf-8"))
        except Exception as e:                                      # noqa: BLE001
            req_err = f"解析抛异常：{e}"
    else:
        req_err = "文件不存在"

    try:
        import mujoco as _mj
    except Exception as e:                                          # noqa: BLE001
        _mj = None
        res.unknown("_simmodel", "train_model_compiles",
                    f"import mujoco 失败：{e} —— 生成的 MJCF 能不能编译、"
                    f"接触过滤怎么走，全查不了（未知=失败）",
                    provenance="mujoco / " + _rel(_GEN_MJCF))

    _bam_params_sync(res)                                          # 09-17 摩擦三处同步
    models = {}
    if _mj is not None:
        for tag, p in (("gen", _GEN_TRAIN_MJCF if _GEN_TRAIN_MJCF.exists() else _GEN_MJCF), ("up", _UP_MJCF)):   # 09-17：训练接口查训练变体
            if not p.exists():
                res.unknown("_simmodel", f"train_model_compiles[{tag}]",
                            f"{_rel(p)} 不存在", provenance=_rel(p))
                continue
            try:
                m = _mj.MjModel.from_xml_path(str(p))
            except Exception as e:                                  # noqa: BLE001
                res.unknown("_simmodel", f"train_model_compiles[{tag}]",
                            f"{_rel(p)} MuJoCo 编译不过：{e} —— 编译不过就根本训不了",
                            provenance=_rel(p))
                continue
            models[tag] = (m, {
                k: [(_mj.mj_id2name(m, o, i) or "") for i in range(n)]
                for k, o, n in (("geom", _mj.mjtObj.mjOBJ_GEOM, m.ngeom),
                                ("site", _mj.mjtObj.mjOBJ_SITE, m.nsite),
                                ("body", _mj.mjtObj.mjOBJ_BODY, m.nbody))})
        if "gen" in models:
            res.add(subject="_simmodel", check="train_model_compiles", state=PASS, severity=BLOCK,
                    measured=[models["gen"][0].nbody, models["gen"][0].ngeom],
                    criterion=f"{_rel(_GEN_MJCF)} 必须能被 MuJoCo 编译（编译不过 = 训不了）",
                    evidence_n=1,
                    detail=f"MuJoCo {getattr(_mj, '__version__', '?')} 编译通过："
                           f"{models['gen'][0].nbody} 个 body、{models['gen'][0].ngeom} 个 geom。"
                           f"**编译得过不等于训得对** —— 接口名字见下面几条",
                    provenance=_rel(_GEN_MJCF) + " / tools/sim/make_mjcf.py")

    if not req or not any(req.values()):
        res.unknown("_simmodel", "train_interface_names",
                    f"从 {_rel(_ENV_CFG)} 解析不出训练环境按名字抓的 geom/site/body"
                    f"（{req_err or '一条都没解析到'}）—— 名字不许写死在判据里（元规则 7），"
                    f"解析不出就是未知 = 失败",
                    provenance=_rel(_ENV_CFG))
    elif "gen" not in models:
        res.unknown("_simmodel", "train_interface_names",
                    f"解析出了 {sum(len(v) for v in req.values())} 条接口要求，"
                    f"但 {_rel(_GEN_MJCF)} 没读进来，核对不了",
                    provenance=_rel(_GEN_MJCF))
    else:
        gn = models["gen"][1]
        un = models["up"][1] if "up" in models else None
        lines, nbad = {}, {}
        for kind in ("geom", "site", "body"):
            lines[kind], nbad[kind] = [], 0
            for src_key, pat in req[kind]:
                try:
                    rx = re.compile(pat)
                except re.error as e:                               # noqa: BLE001
                    lines[kind].append(f"✘ {src_key} /{pat}/ 正则编译不过（{e}）")
                    nbad[kind] += 1
                    continue
                ours = sum(1 for n in gn[kind] if n and rx.fullmatch(n))
                theirs = None if un is None else sum(1 for n in un[kind] if n and rx.fullmatch(n))
                need = 1 if theirs is None else max(1, theirs)
                ok = ours >= need
                nbad[kind] += 0 if ok else 1
                lines[kind].append(f"{'✔' if ok else '✘'} {src_key} /{pat}/ → 生成的 MJCF 里 "
                                   f"{ours} 个，上游参考模型 "
                                   f"{'?（没读到）' if theirs is None else theirs} 个（要求 ≥{need}）")
        for check, kinds, what in (
                ("train_contact_geom_names", ("geom",),
                 "接触传感器 / 摩擦随机化按名字抓的 **geom**"),
                ("train_sensor_frames", ("site", "body"),
                 "传感器坐标系与自碰传感器按名字抓的 **site / body**")):
            got = [x for k in kinds for x in lines[k]]
            bad = sum(nbad[k] for k in kinds)
            if not got:
                res.unknown("_simmodel", check,
                            f"上游配置里没解析出任何{what}要求 —— 解析不到就是未知 = 失败",
                            provenance=_rel(_ENV_CFG))
                continue
            res.add(subject="_simmodel", check=check,
                    state=FAIL if bad else PASS, severity=BLOCK, measured=bad,
                    criterion=f"{_rel(_ENV_CFG)} 里{what}名字/正则，在 {_rel(_GEN_MJCF)} 里"
                              f"匹配到的个数必须 ≥ 它在上游参考模型 robot_walk.xml 里匹配到的个数"
                              f"（且 ≥1）—— 名字对不上，传感器抓不到，模型照样编译、训练照样跑，"
                              f"只是奖励是错的",
                    evidence_n=len(got),
                    detail="；".join(got)
                           + ("。**这不是『没有消费者』，是『喂进去也训不对』**：这些名字上游是"
                              "硬编码的（ContactSensorCfg.pattern / foot_frictions_geom_names），"
                              "抓不到就是：feet_ground_contact 这个 track_air_time=True 的接触"
                              "传感器拿不到落地时刻（cfg.rewards['air_time'] 靠它，"
                              "microduck_velocity_env_cfg.py:220-231 / 330-336），"
                              "脚底摩擦随机化（同文件 foot_frictions_geom_names）也抓空。"
                              "生成器 tools/sim/make_mjcf.py 要么给对应的 geom 补上游的名字，"
                              "要么上游那几处配置跟着改 —— 两边都不改就是训一份奖励错的策略"
                              if bad else "。全部对得上"),
                    provenance=_rel(_ENV_CFG) + " / " + _rel(_GEN_MJCF))

        # ② 谁能碰地 —— 上游只有两只脚，我们是逐件凸包全上 ────────────
        foot_rx = []
        for _k, _p in req["geom"]:
            try:
                foot_rx.append(re.compile(_p))
            except re.error:
                pass                                # 编译不过的已在上一条判据里记红
        part = {}
        for tag, (m, nm) in sorted(models.items()):
            ground, unseen = set(), set()
            for gi in range(m.ngeom):
                if not (int(m.geom_contype[gi]) & 1 or int(m.geom_conaffinity[gi]) & 1):
                    continue                    # 与默认 contype/conaffinity=1 的地面咬不上
                b = nm["body"][int(m.geom_bodyid[gi])]
                ground.add(b)
                g = nm["geom"][gi]
                if not any(rx.fullmatch(g) for rx in foot_rx if g):
                    unseen.add(b)
            part[tag] = (ground, unseen)
        if "gen" in part:
            g_ground, g_unseen = part["gen"]
            u_ground, u_unseen = part.get("up", (set(), set()))
            res.add(subject="_simmodel", check="ground_contact_partition",
                    state=FAIL if g_unseen else PASS, severity=WARN, measured=len(g_unseen),
                    criterion="能与地面接触（contype/conaffinity 与默认 1/1 的地面咬得上）的 body，"
                              "都应该被上游那几条 geom 名字/正则（ContactMatch(mode=geom).pattern + "
                              "foot_frictions_geom_names）抓到 —— 抓不到的 body 碰地时，"
                              "feet_ground_contact 的 track_air_time 仍认为脚在空中，"
                              "air_time / foot_slip 奖励按错的落地时刻算",
                    evidence_n=models["gen"][0].ngeom,
                    detail=f"生成的 MJCF：{len(g_ground)} 个 body 有能碰地的 geom"
                           f"（{sorted(g_ground)}），其中 {len(g_unseen)} 个不被脚底正则覆盖；"
                           f"上游参考模型：{len(u_ground)} 个（{sorted(u_ground)}），"
                           f"不被覆盖 {len(u_unseen)} 个。"
                           + ("我们把每个件的凸包都设成 class=\"collision\"（contype/conaffinity "
                              "走默认 1/1），于是**整只鸭子都能碰地**：膝盖着地、下巴杵地都不会"
                              "计入落地时刻。要么给非脚底的碰撞几何换成只自碰的类"
                              "（上游 additional.xml 的 self_collision_only：contype=2 conaffinity=2，"
                              "咬不上 1/1 的地面），要么接受这个改动并同步改上游的奖励配置。"
                              "**降级为 WARN**：重训之后『整鸭碰地』有可能是我们故意要的"
                              "（更接近实物），本层不替这个决定拍板，只保证它不是无意中发生的"
                              if g_unseen else "全部落地面都被脚底正则覆盖"),
                    provenance=_rel(_ENV_CFG) + " / " + _rel(_GEN_MJCF)
                               + " / upstream/.../additional.xml:self_collision_only")

    # ③ 本层量到的碰撞，训练模型看得见吗 ──────────────────────────────
    #    这一条只依赖生成的模型本身，不依赖上游配置解析 —— 所以放在 req 分支之外，
    #    免得"上游配置解析不出来"顺带把这条 BLOCK 判据从记分卡上抹掉（元规则 3/10）。
    if "gen" not in models:
        res.unknown("_simmodel", "collision_geom_coverage",
                    f"{_rel(_GEN_MJCF)} 读不进来 —— 本层量到的 {len(all_rows)} 条碰撞在训练模型里"
                    f"看不看得见，无从判定（未知=失败）",
                    provenance=_rel(_GEN_MJCF))
    else:
        m, nm = models["gen"]
        by_body = {}
        for gi in range(m.ngeom):
            by_body.setdefault(int(m.geom_bodyid[gi]), []).append(
                (int(m.geom_contype[gi]), int(m.geom_conaffinity[gi])))
        bid = {n: i for i, n in enumerate(nm["body"]) if n}
        filt_parent = not bool(int(m.opt.disableflags) & int(_mj.mjtDisableBit.mjDSBL_FILTERPARENT))
        excl = {tuple(sorted((int(s) >> 16, int(s) & 0xFFFF)))
                for s in list(m.exclude_signature)[:int(m.nexclude)]}

        def _blind_why(ba, bb):
            """MuJoCo 在这两个 body 之间**永远**产生不了接触的原因；None = 看得见。
            过滤规则按 MuJoCo 的接触过滤：同体 / <exclude> / filterparent 的父子体 /
            contype & conaffinity 咬不上。"""
            ia, ib = bid.get(ba), bid.get(bb)
            if ia is None or ib is None:
                return f"body {ba or '?'}/{bb or '?'} 不在生成的 MJCF 里"
            if ia == ib:
                return None                                  # 同一刚体，本层根本不查
            if tuple(sorted((ia, ib))) in excl:
                return "被 <contact><exclude> 显式排除"
            if filt_parent and (int(m.body_parentid[ia]) == ib or int(m.body_parentid[ib]) == ia):
                return "父子体 —— MuJoCo 默认 filterparent 打开，父子之间的接触被丢掉"
            for ct, ca in by_body.get(ia, ()):
                for ct2, ca2 in by_body.get(ib, ()):
                    if (ct & ca2) or (ct2 & ca):
                        return None
            return "两边没有一对 geom 的 contype/conaffinity 咬得上（或者根本没有碰撞几何）"

        blind, reach_blind, why_n, worst = 0, 0, {}, {}
        by_src = {}                     # src 前缀 → [该类记录数, 其中碰不到的, 其中碰不到且可达的]
        for r in all_rows:
            kind = str(r.get("src") or "?").split(":", 1)[0]
            b = by_src.setdefault(kind, [0, 0, 0])
            b[0] += 1
            why = _blind_why(body_of.get(r["a"]), body_of.get(r["b"]))
            if why is None:
                continue
            blind += 1
            b[1] += 1
            why_n[why] = why_n.get(why, 0) + 1
            if _reachable(r.get("pose")):
                reach_blind += 1
                b[2] += 1
                k = f"{r['a']}×{r['b']}"
                if r["v"] > worst.get(k, (0.0, ""))[0]:
                    worst[k] = (r["v"], why)
        top = sorted(worst.items(), key=lambda kv: -kv[1][0])[:6]
        src_txt = "；".join(f"{k} {v[0]} 条（碰不到 {v[1]}、其中可达 {v[2]}）"
                            for k, v in sorted(by_src.items()))
        res.add(subject="_simmodel", check="collision_geom_coverage",
                state=FAIL if reach_blind else PASS, severity=BLOCK, measured=reach_blind,
                criterion="本层量到的每条碰撞，在**我们自己生成的**训练模型里要么看得见"
                          "（两端 body 有咬得上的碰撞几何、且没被 MuJoCo 的接触过滤丢掉），"
                          "要么被 _ranges/collision_free_range 的收窄限位挡在可达域之外 —— "
                          "两条都不满足 = 实机会撞、仿真检测不到、自碰惩罚惩罚不到、策略学不会躲",
                evidence_n=len(all_rows),
                detail=f"对着 {_rel(_GEN_MJCF)} 判（**不是上游那份**，见 "
                       f"_simmodel/upstream_collision_geom_coverage）："
                       f"{len(all_rows)} 条碰撞记录里 {blind} 条两端在仿真里碰不到，"
                       f"其中 {reach_blind} 条**还落在收窄限位之内**（收窄救不了）。"
                       f"原因分布：" + "；".join(f"{k} {v} 条" for k, v in
                                                sorted(why_n.items(), key=lambda kv: -kv[1]))
                       + f"。按来源拆：{src_txt}。**读数时注意**：single:* 那一档的"
                         f"『可达』恒为 0 不是判据的功劳 —— 收窄区间就是从单轴撞点退一格算出来的，"
                         f"单轴撞点按定义落在区间外。这条判据真正查到东西的地方是 combo:* / "
                         f"policy:* 那几档（组合姿态与包络姿态并没有参与收窄区间的推导）"
                       + (f"。最狠的几对：" + "；".join(f"{k} {v[0]:.1f} mm³（{v[1]}）"
                                                       for k, v in top)
                          + "。三条路：给这些 body 对加 <contact><pair>（显式加回来）、"
                            "把 filterparent 关掉、或者改几何。"
                            "**注意**：这里红的绝大多数是相邻连杆（膝×踝、颈×头），"
                            "MuJoCo 默认就把父子体的接触丢掉 —— 上游靠『限位保证够不到』"
                            "绕开这件事，我们的限位是自己定的，绕不开"
                          if reach_blind else
                          "。收窄限位之内的碰撞，训练模型全都看得见"),
                provenance=_rel(_GEN_MJCF) + " / upstream/.../microduck_velocity_env_cfg.py:319-327 "
                                             "/ docs/重训路线_2026-09-09.md")

    # ④ 旧判据量的是上游那份 —— 降级成记录 ──────────────────────────────
    #    记分卡上"15 个 body 只有 2 个有碰撞几何、8016 条碰撞记录仿真看不见"说的是
    #    upstream/.../robot_walk.xml。重训后我们训的不是那份模型，这条不再是放行依据。
    try:
        _root = _ET.parse(str(_UP_MJCF)).getroot()
        col_bodies = set()
        for _b in _root.iter("body"):
            nmb = _b.get("name")
            for _g in _b.findall("geom"):
                # 有效 contype 来自 class 的 <default>，geom 上没有这个属性 ——
                # 所以不能写 (get("contype") or "1") != "0"，那会把每个 visual geom 都算成会碰。
                _cls, _ct = _g.get("class"), _g.get("contype")
                if _cls == "collision" or (_ct is not None and _ct != "0"):
                    col_bodies.add(nmb)
                    break
        n_bodies = sum(1 for _ in _root.iter("body"))
    except Exception as e:                                          # noqa: BLE001
        res.unknown("_simmodel", "upstream_collision_geom_coverage",
                    f"读不了上游 {_rel(_UP_MJCF)} 的 geom：{e}", provenance=_rel(_UP_MJCF),
                    severity=INFO)
    else:
        up_blind = sum(1 for r in all_rows
                       if not (body_of.get(r["a"]) in col_bodies
                               and body_of.get(r["b"]) in col_bodies))
        need = {}
        for r in all_rows:
            for bd in (body_of.get(r["a"]), body_of.get(r["b"])):
                if bd and bd not in col_bodies:
                    need[bd] = max(need.get(bd, 0.0), r["v"])
        res.add(subject="_simmodel", check="upstream_collision_geom_coverage",
                state=RETIRED, severity=INFO, measured=up_blind,
                criterion="【已退役（基准失去权威），保留为记录】上游 robot_walk.xml 里，本层量到会碰的件对"
                          "两端 body 都要有碰撞几何",
                evidence_n=len(all_rows),
                detail=f"上游 {_rel(_UP_MJCF)} 共 {n_bodies} 个 body，只有 {len(col_bodies)} 个"
                       f"有碰撞几何（{sorted(col_bodies)}），按它算 {up_blind}/{len(all_rows)} 条"
                       f"碰撞记录看不见，缺碰撞几何的 body（按最大干涉排序）："
                       + "、".join(f"{k}({v:.0f} mm³)" for k, v in
                                   sorted(need.items(), key=lambda kv: -kv[1]))
                       + "。**但我们训的不是这份模型**：2026-09-09 决定重训，训练用的 MJCF 由 "
                         "tools/sim/make_mjcf.py 从我们的 CAD 生成。这条数字只说明"
                         "『上游的碰撞模型有多稀』，不再是放行依据 —— 真正的判据是 "
                         "_simmodel/collision_geom_coverage（对着生成的那份判）",
                provenance=_rel(_UP_MJCF) + " / docs/重训路线_2026-09-09.md")

    # stem2pid 在零位姿基线之前就建好了（F-L6-2 需要它解析 relations 的 part 方）
    bypid = {}
    for r in all_rows:
        for s in (r["a"], r["b"]):
            pid = stem2pid.get(s)
            if pid:
                bypid.setdefault(pid, []).append(r)
    for pid, rows in sorted(bypid.items()):
        rows.sort(key=lambda r: -r["v"])
        npol = sum(1 for r in rows if r["policy"])
        ncut = sum(1 for r in rows if r["in_cutter"])
        ntgt = sum(1 for r in rows if r["in_target"])
        sev = sev_of(block_flag(r) for r in rows)
        pcc = {c: sum(1 for r in rows if r.get("combo_class") == c) for c in ("viol", "approved", "light", "retrain", "workspace", "unknown")}
        ppc = {c: sum(1 for r in rows if r.get("pose_class") == c) for c in ("viol", "approved", "light", "retrain", "workspace", "unknown")}
        txt = (f"{len(rows)} 条碰撞记录（目标区间内 {ntgt}、外 {len(rows) - ntgt}{ws_note(len(rows) - ntgt)}；"
               f"策略包络内 {npol}、切刀区间内 {ncut}"
               + (f"；其中组合姿态 {sum(pcc.values())} 条按 hr50 v4：到过且超轻碰 {pcc['viol']}、用户批准交重训 {pcc['approved']}、轻碰 {pcc['light']}、"
                  f"没到过（重训约束，不算件的问题）{pcc['retrain']}、目标区间外 {pcc['workspace']}、分不出 {pcc['unknown']}"
                  if sum(pcc.values()) else "")
               + (f"；其中角点姿态 {sum(ppc.values())} 条按 hr50 v6：到过且超轻碰 {ppc['viol']}、用户批准交重训 {ppc['approved']}、轻碰 {ppc['light']}、"
                  f"没到过（重训约束，不算件的问题）{ppc['retrain']}、目标区间外 {ppc['workspace']}、分不出 {ppc['unknown']}"
                  if sum(ppc.values()) else "")
               + "）；"
               f"最严重：{rows[0]['a']}×{rows[0]['b']} {rows[0]['v']:.3f} mm³ @ "
               f"{rows[0]['pose']}（{rows[0]['src']}）")
        if sev is None:
            res.unknown(pid, "motion_collision",
                        f"有碰撞记录的姿态含没有可用目标区间的关节（{sorted(tgt_bad)}）或组合 / 角点记录分不了类（{pcc['unknown']} / {ppc['unknown']} 条），"
                        "分不出件的问题还是工作空间问题 —— " + txt,
                        provenance=_TARGET_KEY)
            continue
        res.add(subject=pid, check="motion_collision", state=FAIL, severity=sev,
                measured=round(rows[0]["v"], 6),
                criterion=crit + f"；组合姿态记录按 hr50 v4 分类（{_rel(_REAL_POSE_POLICY)}:pair_combination）、角点姿态记录按 hr50 v6 分类（同文件 policy_extreme_poses），只有『真实动作到过且超轻碰』算件的问题",
                evidence_n=len(rows),
                detail=txt, provenance="README 第 2 节 L6 / " + _TARGET_KEY + " / " + _TOL_KEY)

    # ── E 线束 ───────────────────────────────────────────────────────────
    # 【2026-09-26 用户停用，见文件头「E′ 停用」】下面三行是原 E′ 段，整段注释掉（代码不删）：
    # # E′（hr44reg 2026-09-25）：先按 duckstructure.wiring_body 实体模型逐线路判（L6/<线路 id>:harness_sweep）；
    # #    route_model.covers_whole_bundle 且本轮每条线路都判了的束，不再发下面『没有折线』unknown（格子由线路格承担）。
    # hs_done, hs_ev = _harness_sweep(ctx, res)
    hs_done, hs_ev = set(), {"disabled": "2026-09-26 用户停用 E′ 线束实体模型扫掠（_harness_sweep 未调用；恢复方法见 l6_motion.py 文件头「E′ 停用」）"}   # E′ 停用占位
    for hb in ((ctx.data.get("harness") or {}).get("bundles") or []):
        hid = hb.get("id")
        if hid in hs_done:
            continue
        route = hb.get("route_polyline")
        od = hb.get("od_mm") or {}
        odv = od.get("v") if isinstance(od, dict) else od
        status = hb.get("status")
        cj = hb.get("crosses_joints") or []
        machine = isinstance(route, (list, tuple)) and len(route) >= 2
        rm = hb.get("route_model")
        rm_note = (f"；route_model 已登记 {len(rm.get('routes') or [])} 条线路按 wiring_body 模型逐条判"
                   f"（L6/<线路 id>:harness_sweep），本格仍红 = 其余段无模型：{str(rm.get('uncovered') or '')[:90]}"
                   if isinstance(rm, dict) else "")
        res.unknown(hid, "harness_sweep",
                    (f"status={status}；route_polyline={'自然语言：' + str(route)[:60] if route else 'null'}"
                     f"（不是折线坐标，扫不了管）；od_mm={odv}"
                     f"（{od.get('unknown_reason') if isinstance(od, dict) else ''}）；"
                     f"跨 {len(cj)} 条关节 {cj[:4]}{'…' if len(cj) > 4 else ''}。"
                     f"没有折线就没有管，没有管就没有『沿路径扫过全行程』这条判据 —— "
                     f"**本层不拿直线段假装线束**（元规则 4）"
                     if not machine else f"route_polyline 有 {len(route)} 个点但本层还没实现管扫掠") + rm_note,
                    provenance=f"harness.yaml:{hid}.route_polyline / keepouts.yaml:KO19")

    # ── 归本层的禁入体（第 3 层显式让出来的三条） ──────────────────────
    kos = {k.get("id"): k for k in ((ctx.data.get("keepouts") or {}).get("keepouts") or [])}
    for kid, how in (("KO10", "本层的 neck_pitch / head_pitch 单轴 + 两者组合已覆盖；"
                              "assembly_audit 只对 neck×shell 做过 1.25° 采样、且只查这一对"),
                     ("KO13", "本层的 *_hip_yaw 单轴 + hip_yaw×hip_roll 组合已覆盖；"
                              "电池实体 zz_battery 在 placed 里参加了检查")):
        k = kos.get(kid)
        if not k:
            continue
        # owner_parts 说这条禁入体归谁管 → 解析成 placed 实体，从本层的碰撞记录里筛出相关的
        owners = set()
        for tokk in re.split(r"[/、,，\s]+", str(k.get("owner_parts") or "")):
            for s, pid in stem2pid.items():
                if pid == tokk:
                    owners.add(s)
        rows = [r for r in all_rows if r["a"] in owners or r["b"] in owners]
        if not owners:
            res.unknown(kid, "keepout_motion", f"owner_parts={k.get('owner_parts')!r} 解析不出 placed 实体",
                        provenance=f"keepouts.yaml:{kid}")
            continue
        peak = max((r["v"] for r in rows), default=0.0)
        w = max(rows, key=lambda r: r["v"]) if rows else None
        sev_k = sev_of(r["in_target"] for r in rows)
        n_k_tgt = sum(1 for r in rows if r["in_target"])
        k_txt = (f"{k.get('name')}（{k.get('geom_verbatim')}，owner_parts="
                 f"{k.get('owner_parts')!r} → {sorted(owners)}）：{how}。"
                 + (f"{len(rows)} 条碰撞记录涉及这些件（目标区间内 {n_k_tgt}、外 {len(rows) - n_k_tgt}"
                    f"{ws_note(len(rows) - n_k_tgt)}；策略包络内 "
                    f"{sum(1 for r in rows if r['policy'])}、切刀区间内 "
                    f"{sum(1 for r in rows if r['in_cutter'])}），峰值 {peak:.3f} mm³ "
                    f"{w['a']}×{w['b']} @ {w['pose']}" if rows else "没有一条碰撞记录涉及这些件"))
        if sev_k is None:
            res.unknown(kid, "keepout_motion",
                        f"涉及的碰撞记录含没有可用目标区间的关节（{sorted(tgt_bad)}），分不出区间内/外 —— " + k_txt,
                        provenance=f"keepouts.yaml:{kid} / " + _TARGET_KEY)
            continue
        res.add(subject=kid, check="keepout_motion", state=PASS if not rows else FAIL,
                severity=(BLOCK if not rows else sev_k),
                measured=round(peak, 6),
                criterion=crit + "；本条只按 owner_parts 过滤碰撞记录（**一侧是 owner 件就算**），"
                                 "不按 geom_verbatim 里的角度区间过滤 —— 所以数字里也包含"
                                 "别的关节把别的件撞到 owner 件上的记录，看峰值那一行的件名和姿态",
                evidence_n=sc.booleans,
                detail=k_txt,
                provenance=f"keepouts.yaml:{kid} / README 第 2 节 L6 / " + _TARGET_KEY)
    if "KO19" in kos:
        res.unknown("KO19", "keepout_harness_twist",
                    f"{kos['KO19'].get('name')}：geom_verbatim='{kos['KO19'].get('geom_verbatim')}' —— "
                    f"头偏航 ±170° 的线束扭转没有任何模型，harness.yaml 里 HB01/HB04/HB05/HB09 "
                    f"四束跨这条关节的线也都是 unmodeled",
                    provenance="keepouts.yaml:KO19 / harness.yaml")

    res.evidence = {"placed_solids": len(sc.names),
                    "joints": len(joints),
                    "adjacent_pairs": len(chain),
                    "collision_records": len(all_rows),
                    "records_in_policy_box": stats["policy"][0],
                    "records_in_cutter_domain": stats["in_cutter"][0],
                    "records_outside_cutter_domain": stats["out_cutter"][0],
                    "records_in_target_range": stats["in_target"][0],
                    "records_outside_target_range": stats["out_target"][0],
                    "records_target_unknown": stats["target_unknown"][0],
                    "joints_without_target_range": sorted(tgt_bad),
                    "combo_grid_step_deg": _COMBO_STEP_DEG,
                    "declared_contact_pairs": len(mating),
                    "undeclared_zero_gap_pairs": len(undeclared),
                    "near_miss_rows": len(near_rows),
                    "booleans_run": sc.booleans,
                    "min_gap_calls": sc.gaps}
    res.evidence["harness_sweep"] = hs_ev                            # hr44reg：E′ 线束模型扫掠（来源 / 采样档 / 耗时 / 没跑原因与异常类型）
    if hasattr(sc, "hr42_counters"):                               # hr42：实现层计数（凸包分离证明 / 同输入记忆），结果无关
        res.evidence.update(sc.hr42_counters())
    return res
