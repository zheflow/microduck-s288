#!/usr/bin/env python3
"""第 7 层 质量与力 —— 撑得住、站得稳吗。

README 第 2 节 L7：17 件体积×密度 + 元件实重 → 每 body 质量重心 · **与 MJCF `<inertial>` 逐 body 比漂移** ·
每关节静扭矩 < S288 额定 × 系数 · 零位/关键姿态重心投影在支撑多边形内 · 承力最小截面 / 薄筋长厚比 / 层向 vs 受力。

这一层项目里从来没有过。S288 比 XL330 重、我们还加了屏，仿真和实物会分家 —— 逐 body 比漂移
是 sim2real 最便宜也最常被跳过的一步。

件 → MJCF body 的归属不靠人工表：导出件是 to_local(world, body)、placed 件是同一网格的世界坐标，
所以 **TW(body) @ 局部质心 == 世界质心** 只对唯一一个 body 成立（实测 15 个 body 里残差 0.000）。
左右镜像件按 frozen.yaml:joint_axes 的 left_X/right_X 配对换到对侧 body。

判据来源与缺口（元规则 4：拿不到数就是红）：
  · 密度         tolerances.yaml:load.density_g_cm3（1.27, assumed）—— TPU 没有单独密度
  · 元件质量     components.yaml:components[].mass_g —— 19 个里只有 2 个有数
  · 质量/质心阈值 tolerances.yaml:load.mass_drift_vs_mjcf（域随机化范围推出来的）——
                 **2026-09-10 起这 15 条 drift_criteria 是 RETIRED（退役）**：2026-09-09 决定重训，
                 MJCF 不再是真理（docs/重训路线_2026-09-09.md），阈值也来自旧策略；
                 而且两侧密度口径不同（我们 assumed vs 上游 <inertial> 反解，数字一律见
#                 tolerances.yaml:load.density_g_cm3.vs_upstream_mjcf_implied_density，本文件不复述）。
                 详见 _mjcf.mjcf_baseline_authority；**不许把基准改指向 robot_walk_s288.xml**
                 （那份是同一套算法从同一批网格生成的，指过去等于同义反复）。
  · 惯量阈值     tolerances.yaml:load.inertia_drift_vs_mjcf 两个键都是 null（拒填，见那里的
                 refused_2026-09-09：训练的 ±5% 是各向同性缩放，约束不了分布）→ res.unknown
  · 扭矩系数     tolerances.yaml:load.servo_output_bearing.static_torque_margin = null → res.unknown
  · 额定扭矩     tolerances.yaml:load.servo_rated_torque_Nm.stall_torque_Nm = 0.6（名义**堵转**）——
                 **2026-09-13（F-L7-5）起这是唯一读取的键**（该桶自述 authoritative: true）。镜像键
                 load.servo_output_bearing.rated_stall_torque_Nm 已按它自己的 duplication_note 删除；若有人写回
                 且与权威值不等，_torque_source 判 torque_source_consistent FAIL(BLOCK)。不再退回扫 docs/ 的散文。
                 continuous_torque_Nm 仍是 null → 一切『够不够力』的结论都只能是必要条件。
  · 力矩上限     **2026-09-10 新增** _forcerange_vs_servo：MJCF <actuator forcerange> vs 上面那个额定。
                 此前整套 Gate 一条都没查过（`grep -rn forcerange tools/gate/` 为空），
                 而 sim/duck_s288/robot_walk_s288.xml 的 14 个 actuator 仍是上游 XL330 的 ±0.96 N·m。
  · 截面/薄筋    tolerances.yaml:load.min_section_and_rib 三个字段全 null → res.unknown

2026-09-09 修的三处"层量的不是它该量的东西"（FG20 + 两条 refused_2026-09-09）：
  1. `_mjcf_inertial` 此前只读 <inertial mass= pos=>，**惯量张量一个字节都没读过** —— 质量对上、
     质心对上而料从中间挪到两端，惯量差一倍也一声不吭。现在 fullinertia / diaginertia+quat 都读，
     CAD 侧按平行轴定理合成全张量，两边**各自对角化**比主惯量三元组 + 主轴夹角（坐标系无关；
     只比迹不够，迹相同而分布不同正是要抓的）。
  2. `_sections` 此前量"沿世界 z 每 1 mm 切一刀、整件最小的那层水平截面"——那不是承力截面。
     现在沿 relations.yaml:load_paths 声明的方向、只在 from→to 那一段、在 export_local 里切。
     **2026-09-10 更正：这只修好了一半。** trimesh 的 section 是无限平面切整件，返回值只由
     (法向 d, 平面偏移 o·d) 决定 —— 换了法向是真的进步，但量到的仍然是**整件截面**，
     与路径的横向位置无关（把整条路径平移到件外 10 m，11/11 条空刀仍为 0、逐刀面积逐位相同）。
     所以 load_path_endpoints 已从 BLOCK 降为 INFO（load_path_continuous 保留 BLOCK：
#     它量的『整件截面 > 0』是逻辑上站得住的必要条件），
     并各补一条 unknown（load_path_local_section / load_path_endpoint_on_material）说明真判据没实现。
  3. 薄筋此前**根本没有量筋的代码**（只有一句 `if rib is None: unknown`）。现在 `_ribs` 读
     features.yaml:kind=rib 的探针，真的把长/厚量出来并与声明值比。

2026-09-13 审计第二批（F-L7-3/4/5/6 + §2.6 泛化；每条都有反例，见 negatives/n_l7_*.py）：
  · F-L7-3  `static_torque` 只判过单轴扫描峰值：home 姿态扭矩算了不判 → 新增 `static_torque_home`（同阈值、
            14 关节同时置 home）；range_deg lo ≥ hi 或评估点数 < 3 → 该关节 unknown（check_angles 反写只给
            两端点，"扫了 2 个角"不是扫）。
  · F-L7-4  整机汇总判据（static_torque / static_torque_home / com_in_support_* / head_mass_budget）在**质量清单
            有缺口**时一律 unknown：缺口 = qty>0 无质量、无 claim 规则、认领数 ≠ qty、定不出 body、有质量认领不到
            实体、qty 不是整数、打印件归属失败。detail 列缺口清单并只给"仅含已知质量的下界值"文本，不拿下界当绿。
  · F-L7-5  额定扭矩只读权威键（见上）。
  · F-L7-6  元件认领不再按 category 分支：components.yaml 每条元件自己声明 `claim`（见 _CLAIM_BY 注释），
            没有 claim → component_body unknown；认领到的实例数 ≠ qty → component_qty FAIL(BLOCK)。
  · §2.6    镜像对数由 frozen.yaml 的 left_/right_ 声明推（不再 == 5）；home_rad 缺失 = unknown（不再当 0）；
            落地件只认 parts.yaml:ground_contact（不再 "TPU" in material）；筋厚/长 vs 声明的容差进
            tolerances.yaml:load.min_section_and_rib.rib_declared_vs_measured_tol_mm；密度 / 稳定裕度 / 筋容差
            的 src 写进 criterion，src=assumed 各发一条 threshold_assumed:<键> FAIL(WARN, measured=阈值)。
"""
from __future__ import annotations
import math
import re
import struct
import sys
from pathlib import Path

_HERE = Path(__file__).resolve()
sys.path.insert(0, str(_HERE.parents[1]))                       # tools/gate → core
_REPO = _HERE.parents[3]
if str(_REPO) not in sys.path:
    sys.path.insert(0, str(_REPO))                              # 仓库根（duckstructure）

from core import (LayerResult, PASS, FAIL, STALE, RETIRED, BLOCK, WARN, INFO,        # noqa: E402
                  ROOT, PLACED, num)

LAYER = 7
NAME = "质量与力"

_NUM = r"[0-9]+(?:\.[0-9]+)?"
_G = 9.80665                      # 标准重力加速度（物理常数，不是判据阈值）
_CONTACT_BAND_MM = 0.5            # 支撑多边形的接触带厚度：采样参数，不是判据
_SECTION_STEP_MM = 1.0            # 最小截面扫描步距：采样参数，不是判据
_SECTION_PROBE_MM = 10000.0       # 自检探针：把整条承力路径沿法向平移这么远再切一遍，
                                  # 用来证明"这刀量的是不是这条路径"。采样参数，不是判据
_SECTION_PROBE_REL = 1e-6         # 探针的相对判变阈值。平移 10 m 之后平面方程的量级大了 4 个数量级，
                                  # 交线求解会带进 ~1e-13 的相对浮点噪声；不设这个阈值，噪声会被
                                  # 当成"截面变了"，把一条本该红的自检判绿。1e-6 远高于噪声、
                                  # 远低于任何真实几何变化。数值容差，不是判据阈值
_BUDGET_CAP_G = 100000.0          # 头部质量预算二分的上界哨兵：超过它就认为该关节不受重力限制


# ── 小工具 ────────────────────────────────────────────────────────────────
def _mesh(p):
    import trimesh
    return trimesh.load(str(p), process=True)


def _stl_sig(p: Path):
    """(三角面数, 有符号体积 mm³)，直接读文件。体积是刚体不变量 → 用来把导出件和 placed 件对上号。"""
    import numpy as np
    with open(p, "rb") as f:
        n = struct.unpack("<I", f.read(84)[80:84])[0]
        data = f.read(50 * n)
    tris = np.frombuffer(data, dtype=np.dtype([("n", "<3f4"), ("v", "<3,3f4"), ("a", "<u2")]), count=n)
    v = tris["v"].astype(np.float64)
    return n, float(np.einsum("ij,ij->i", v[:, 0], np.cross(v[:, 1], v[:, 2])).sum() / 6.0)


def _rel(p):
    """仓库相对路径，用于 res.inputs 的过期绑定。路径不在仓库里时退回绝对路径 —— 记账不能把层弄崩。"""
    try:
        return str(Path(p).relative_to(ROOT))
    except ValueError:
        return str(p)


def _Rz(t):
    import numpy as np
    c, s = math.cos(t), math.sin(t)
    M = np.eye(4)
    M[0, 0] = c; M[0, 1] = -s; M[1, 0] = s; M[1, 1] = c
    return M


_RATED_KEY = "tolerances.yaml:load.servo_rated_torque_Nm.stall_torque_Nm"
_DEPRECATED_KEY = "tolerances.yaml:load.servo_output_bearing.rated_stall_torque_Nm"


def _rated_torque_Nm(ctx):
    """S288 额定（名义堵转）扭矩 —— **只读**权威键 tolerances.yaml:load.servo_rated_torque_Nm.stall_torque_Nm。

    2026-09-13（F-L7-5）：该桶自述 authoritative: true；镜像键 load.servo_output_bearing.rated_stall_torque_Nm
    的 duplication_note 写明"唯一权威定义在 load.servo_rated_torque_Nm……要把层改成读那里、把本键删掉"。
    以前先读镜像键、找不到再去 docs/ 的散文里 grep『堵转 0.6 N·m』—— 那条退回路径连同它的两个已知弱点
    （命中的原句是在禁止拿峰值除以 0.6；docs/ 不在 core.SOURCE_GLOBS 里、不受 hash 绑定）一起删掉：
    阈值不在 data 里就是 unknown，不去别处找。返回 (值或 None, src)。"""
    load = (ctx.data.get("tolerances") or {}).get("load") or {}
    srt = load.get("servo_rated_torque_Nm")
    v, src = num(srt.get("stall_torque_Nm") if isinstance(srt, dict) else None)
    return (float(v) if isinstance(v, (int, float)) else None), src


def _torque_source(ctx, res):
    """F-L7-5：额定扭矩来源唯一。发两条判据，都只读 tolerances.yaml，与质量清单无关，所以放在 run 最前面：
      · `_load:rated_stall_torque`  权威键读到的数（INFO）；读不到 → unknown；
      · `_load:torque_source_consistent`  废弃镜像键若仍存在且 ≠ 权威 → FAIL(BLOCK, measured=差值)；
        相等 → PASS(INFO) 并提示应删；不存在 → PASS(INFO)。"""
    rated, src = _rated_torque_Nm(ctx)
    load = (ctx.data.get("tolerances") or {}).get("load") or {}
    sob = load.get("servo_output_bearing") or {}
    if rated is None:
        res.unknown("_load", "rated_stall_torque",
                    f"{_RATED_KEY} 取不到数字 —— 静扭矩 / forcerange 都没有比较基准。"
                    "**不再退回 docs/ 散文 grep**（2026-09-13 F-L7-5：阈值不在 data 里就是 unknown）",
                    provenance=_RATED_KEY)
    else:
        res.add(subject="_load", check="rated_stall_torque", state=PASS, severity=INFO, measured=rated,
                criterion=f"S288 名义堵转扭矩只从 {_RATED_KEY} 读（yaml 自述 authoritative: true；src={src!r}）",
                evidence_n=1,
                detail=f"{rated} N·m（src={src!r}）。这是堵转不是可持续输出（该桶 not_sustainable），"
                       "下面所有『占额定的比例』都只是必要条件",
                provenance=_RATED_KEY)
    if isinstance(sob, dict) and "rated_stall_torque_Nm" in sob:
        dv, dsrc = num(sob.get("rated_stall_torque_Nm"))
        if dv is None or rated is None:
            res.unknown("_load", "torque_source_consistent",
                        f"废弃镜像键 {_DEPRECATED_KEY} 仍存在，但它（v={dv!r}）或权威键（{rated!r}）取不到数字，"
                        "两处一不一致都说不清；该键按 yaml 自述应删",
                        provenance=_DEPRECATED_KEY + " / " + _RATED_KEY)
        else:
            diff = float(dv) - float(rated)
            same = abs(diff) <= 1e-12
            res.add(subject="_load", check="torque_source_consistent",
                    state=PASS if same else FAIL, severity=INFO if same else BLOCK, measured=round(diff, 6),
                    criterion=f"废弃镜像键 {_DEPRECATED_KEY} 若存在，必须等于权威 {_RATED_KEY}（差 == 0）；"
                              "该键按它自己的 duplication_note 应删，层不读它",
                    evidence_n=1,
                    detail=(f"废弃键 {dv}（src={dsrc!r}）vs 权威 {rated}（src={src!r}），差 {diff:+.6f} N·m。"
                            + ("两处相等 —— 但 duplication_note 写明本键应删、层已不读它，请删掉以免再次分叉"
                               if same else
                               "**两处不一致**：层只认权威键；废弃键必须删掉或改成一致，否则读它的人拿到的是另一个门槛")),
                    provenance=_DEPRECATED_KEY + " / " + _RATED_KEY)
    else:
        res.add(subject="_load", check="torque_source_consistent", state=PASS, severity=INFO, measured=0,
                criterion=f"废弃镜像键 {_DEPRECATED_KEY} 不存在（已按 yaml 自述删除）→ 额定只有一个来源",
                evidence_n=1, detail=f"额定扭矩唯一来源 {_RATED_KEY} = {rated}", provenance=_RATED_KEY)
    return rated, src


# ── MJCF actuator forcerange vs 实物舵机额定 ──────────────────────────────
# 2026-09-10 新增。此前 `grep -rn forcerange tools/gate/` 是空的 —— 整套 Gate 没有任何一条判据
# 看过"策略被允许用多大力矩"这件事，而这正是 sim2real 会直接摔件的一项。
_SIM_MJCF = ROOT / "sim" / "duck_s288" / "robot_walk_s288.xml"


def _forcerange_by_class(root):
    """MJCF <default> 树 → {class 名: (lo, hi) N·m}。子 default 继承父的 <position forcerange>。
    只认 <position>（本模型的 actuator 全是 position）。"""
    out = {}

    def walk(el, inherited):
        cur = inherited
        pos = el.find("position")
        if pos is not None and pos.get("forcerange"):
            try:
                v = [float(x) for x in pos.get("forcerange").split()]
                cur = (v[0], v[1]) if len(v) == 2 else inherited
            except ValueError:
                cur = inherited
        cls = el.get("class")
        if cls:
            out[cls] = cur
        for c in el.findall("default"):
            walk(c, cur)

    for d in root.findall("default"):
        walk(d, None)
    return out


def _actuators_forcerange(root):
    """[(actuator 名, joint 名, class 名, (lo,hi) 或 None)]，class 上的缺省用 default 树解析。"""
    fr = _forcerange_by_class(root)
    out = []
    act = root.find("actuator")
    if act is None:
        return out
    for a in act:
        cls = a.get("class")
        own = a.get("forcerange")
        rng = None
        if own:
            try:
                v = [float(x) for x in own.split()]
                rng = (v[0], v[1]) if len(v) == 2 else None
            except ValueError:
                rng = None
        if rng is None:
            rng = fr.get(cls)
        out.append((a.get("name"), a.get("joint"), cls, rng))
    return out


def _forcerange_vs_servo(ctx, res):
    """**策略被允许用多大力矩 vs 实物舵机能出多大力矩。**

    额定一律从 tolerances.yaml:load.servo_rated_torque_Nm 读（元规则 7：层里不写死 0.6）。
    该桶里有两条明令，本判据都遵守：
      · continuous_torque_Nm.do_not_derive_from —— 不许由电流×扭矩常数反推持续扭矩；
      · servo_output_bearing.static_torque_margin.forbidden_derivations —— FD1/FD2/FD3。
    所以这里**只判一个必要条件**（forcerange 上限 ≤ 名义堵转），并同时留一条红说明真判据
    （对着**可持续**扭矩比）判不了；手册三个数自相矛盾这件事也单独留一条红，不假装知道答案。"""
    import xml.etree.ElementTree as ET
    load = (ctx.data.get("tolerances") or {}).get("load") or {}
    srt = load.get("servo_rated_torque_Nm") or {}
    stall, stall_src = num(srt.get("stall_torque_Nm"))
    cont, _cont_src = num(srt.get("continuous_torque_Nm"))
    dnd = (srt.get("continuous_torque_Nm") or {}).get("do_not_derive_from") \
        if isinstance(srt.get("continuous_torque_Nm"), dict) else None
    fds = ((load.get("servo_output_bearing") or {}).get("static_torque_margin") or {}).get(
        "forbidden_derivations") if isinstance(
        (load.get("servo_output_bearing") or {}).get("static_torque_margin"), dict) else None
    fd1 = next((f.get("what") for f in (fds or []) if isinstance(f, dict) and f.get("id") == "FD1"), None)

    if stall is None:
        res.unknown("_actuator", "servo_rated_torque",
                    "tolerances.yaml:load.servo_rated_torque_Nm.stall_torque_Nm 取不到数字 —— "
                    "MJCF 的 <actuator forcerange> 没有可比的基准",
                    provenance="tolerances.yaml:load.servo_rated_torque_Nm.stall_torque_Nm")

    files = []
    if _SIM_MJCF.exists():
        files.append(("sim", _SIM_MJCF))
    else:
        res.unknown("_actuator", "sim_mjcf_present",
                    f"{_rel(_SIM_MJCF)} 不存在 —— 重训后策略要用的 MJCF 是它（由 "
                    "tools/sim/make_mjcf.py 生成），拿不到就查不了 forcerange",
                    provenance="tools/sim/make_mjcf.py")
    try:
        from duckstructure import kin
        up = Path(kin.MD) / "robot_walk.xml"
        if up.exists():
            files.append(("upstream", up))
    except Exception:                                           # noqa: BLE001
        pass

    parsed = {}
    for tag, f in files:
        res.inputs.append(_rel(f))
        try:
            parsed[tag] = _actuators_forcerange(ET.parse(f).getroot())
        except Exception as e:                                  # noqa: BLE001
            res.unknown("_actuator", f"{tag}_forcerange_parse",
                        f"{_rel(f)} 的 <actuator>/<default> 解析失败：{e}", provenance=_rel(f))

    # 训练用的是**生成的那份**，所以只认 sim。
    # 2026-09-10 复审打穿的假绿：这里原本写的是
    #     acts = parsed.get("sim") or parsed.get("upstream") or []
    # 那个 `or` 在 sim 解析出 0 个 actuator 时会**静默退回读上游 robot_walk.xml**，
    # 而 src_file 仍然指向 sim —— 于是 15 条 finding 的 detail 把上游的数报成 sim 的数。
    # 这正是元规则 10 说的那类失败：检查了、通过了、查的不是那个东西。
    if "sim" not in parsed:
        return
    which, acts = "sim", parsed["sim"]
    src_file = _rel(dict(files)["sim"])
    if not acts:
        res.unknown("_actuator", "forcerange_inventory",
                    f"{src_file} 里一个 <actuator> 都没解析到 —— 策略被允许输出多大力矩，未知。"
                    "**不许拿上游 robot_walk.xml 的执行器顶上**：训练用的是这份生成产物",
                    provenance=src_file + " / tools/sim/make_mjcf.py")
        return

    # 两份 MJCF 的 forcerange 一不一样：如果一样，说明生成器**原样继承了上游 XL330 的执行器**
    same = None
    if "sim" in parsed and "upstream" in parsed:
        same = ({(n, r) for n, _j, _c, r in parsed["sim"]}
                == {(n, r) for n, _j, _c, r in parsed["upstream"]})
    clsset = sorted({c for _n, _j, c, _r in acts if c})
    rngset = sorted({r for _n, _j, _c, r in acts if r})
    # 2026-09-10 复审打穿的假绿之二：state 原本是 `PASS if acts else FAIL`，
    # 而 acts 只是 actuator 列表，跟「forcerange 有没有解析出来」无关 ——
    # 把 MJCF 里每个 forcerange="..." 属性删光，这条照样 PASS/BLOCK measured=14，
    # detail 还打印「forcerange 取值 []」。criterion 说的是"每个"，代码一个都没查。
    n_rng = sum(1 for _n, _j, _c, r in acts if r is not None)
    res.add(subject="_actuator", check="forcerange_inventory",
            state=PASS if n_rng == len(acts) else FAIL,
            severity=BLOCK, measured=f"{n_rng}/{len(acts)}",
            criterion="必须能从 MJCF 里解析出每个 <actuator> 的 forcerange（含 <default class> 继承）",
            evidence_n=len(acts),
            detail=f"{src_file}：{len(acts)} 个 actuator，其中 {n_rng} 个解析出 forcerange，"
                   f"class {clsset}，forcerange 取值 {rngset} N·m"
                   + ("" if n_rng == len(acts) else
                      f"；**{len(acts) - n_rng} 个 actuator 解析不出 forcerange** —— "
                      "这些关节的力矩上限未知，逐关节判据会各自走 unknown")
                   + ("" if same is None else
                      ("；**与上游 robot_walk.xml 逐个 actuator 完全相同** —— "
                       "生成器只换了 <inertial>/<geom>，执行器一个字节没改，还是 XL330 那一套"
                       if same else "；与上游 robot_walk.xml 不同")),
            provenance=src_file + " / tools/sim/make_mjcf.py")

    if stall is None:
        return
    for nm, jt, cls, rng in acts:
        subj = f"joint:{jt or nm}"
        if rng is None:
            res.unknown(subj, "forcerange_vs_servo_rating",
                        f"actuator {nm}（class={cls}）在 {src_file} 里解析不出 forcerange —— "
                        "策略的力矩上限未知，不能当通过",
                        provenance=src_file)
            continue
        peak = max(abs(rng[0]), abs(rng[1]))
        ok = peak <= stall + 1e-12
        res.add(subject=subj, check="forcerange_vs_servo_rating",
                state=PASS if ok else FAIL, severity=BLOCK, measured=round(peak, 4),
                criterion=f"MJCF <actuator> 的 |forcerange| ≤ 舵机**名义堵转** {stall} N·m"
                          f"（tolerances.yaml:load.servo_rated_torque_Nm.stall_torque_Nm，src={stall_src!r}）"
                          "—— 这只是**必要条件**：堵转不是可持续输出",
                evidence_n=1,
                detail=f"{src_file}：actuator {nm}（joint {jt}，class {cls}）forcerange "
                       f"[{rng[0]}, {rng[1]}] N·m → 上限 {peak} N·m，"
                       f"舵机名义堵转 {stall} N·m，比值 {peak/stall:.2f}×"
                       + ("。策略被允许输出的力矩超过实物舵机的名义堵转 —— "
                          "仿真里学出来的动作，实物做不出来（S288 是塑料齿塑料壳）"
                          if not ok else "。满足必要条件"),
                provenance=src_file + " / tolerances.yaml:load.servo_rated_torque_Nm.stall_torque_Nm")

    if cont is None:
        res.unknown("_actuator", "forcerange_vs_continuous_torque",
                    "**真判据判不了**：forcerange 该比的是舵机**可持续**输出扭矩，"
                    "而 tolerances.yaml:load.servo_rated_torque_Nm.continuous_torque_Nm = null"
                    "（unknown_class=not_measured）。上面每个关节那条只是"
                    f"『≤ 名义堵转 {stall} N·m』这个必要条件，过了也不等于实物撑得住。"
                    + (f"该键还明令：{dnd}" if dnd else "")
                    + " 要解开见 bench.yaml:BN13（实测堵转）/ BN03（温升 → 可持续水平）",
                    provenance="tolerances.yaml:load.servo_rated_torque_Nm.continuous_torque_Nm / "
                               "bench.yaml:BN13 / bench.yaml:BN03")
    if fd1:
        res.unknown("_actuator", "servo_datasheet_self_contradiction",
                    "**手册自相矛盾，本层不选边**：同一张参数表同时给出名义堵转 "
                    f"{stall} N·m 和一组电流/扭矩常数，而 tolerances.yaml 已经把由后者反推这件事"
                    f"列为禁止推导（FD1：{fd1}）。两个口径对不上就意味着"
                    "『实物输出端到底能出多大力矩』本仓库没有唯一答案 —— "
                    "上面各关节的必要条件用的是**较大**的那个（堵转），"
                    "也就是**最宽松**的读法；真实上限只会更低，所以那些 PASS 不能当放行。"
                    "唯一出路是实测（bench.yaml:BN13）",
                    provenance="tolerances.yaml:load.servo_output_bearing.static_torque_margin."
                               "forbidden_derivations.FD1 / "
                               "tolerances.yaml:load.servo_rated_torque_Nm.stall_torque_Nm")


def _margin(poly, p):
    """点 p 到凸多边形 poly 各边的最小有符号距离（正 = 在内部，负 = 在外面多远）。
    朝向由**多边形自己的环绕方向**（有符号面积）定，不能拿点的位置去猜 ——
    点离得远时'多数边距离为负'那种启发式会翻号，把'在外面'算成'在里面'。"""
    import numpy as np
    n = len(poly)
    if n < 3:
        return None
    area2 = sum(float(poly[i][0] * poly[(i + 1) % n][1] - poly[(i + 1) % n][0] * poly[i][1])
                for i in range(n))
    if abs(area2) < 1e-12:
        return None
    ccw = area2 > 0
    d = []
    for i in range(n):
        a, b = poly[i], poly[(i + 1) % n]
        e = b - a
        L = float(np.linalg.norm(e))
        if L < 1e-12:
            continue
        inward = np.array([-e[1], e[0]]) / L if ccw else np.array([e[1], -e[0]]) / L
        d.append(float(np.dot(p - a, inward)))
    return min(d) if d else None


# ── 件 → MJCF body ────────────────────────────────────────────────────────
def _assign(ctx, res):
    """返回 (pid -> [(placed stem, body)], stem->Path, B, order)。
    匹配规则见模块 docstring：TW(body) @ 局部质心 == 世界质心。"""
    import numpy as np
    B, order = ctx.mjcf()
    placed = {p.stem: p for p in sorted(PLACED.glob("*.stl"))} if PLACED.exists() else {}
    if not placed:
        res.unknown("_layer", "placed_dir", "cad/duck_s288/placed/ 不存在或为空 —— 拿不到世界坐标")
        return {}, placed, B, order, {}

    ja = ((ctx.data.get("frozen") or {}).get("joint_axes") or [])
    jb = {j["name"]: j.get("mjcf_body") for j in ja if isinstance(j, dict) and j.get("name")}
    lefts = sorted(n for n in jb if n.startswith("left_"))
    rights = sorted(n for n in jb if n.startswith("right_"))
    mirror, unmatched = {}, []
    for n in lefts:
        b, r = jb.get(n), jb.get("right_" + n[5:])
        if b and r and b in B and r in B:
            mirror[b] = r
        else:
            unmatched.append(f"{n}↔right_{n[5:]}: body {b!r}/{r!r}"
                             + ("" if (b in B and r in B) else "（不在 MJCF 里 / 缺）"))
    for n in rights:
        if "left_" + n[6:] not in jb:
            unmatched.append(f"{n} 没有同名 left_{n[6:]}")
    # 镜像 body 配对错了，整条腿的质量会挂到对侧 —— 扭矩和重心跟着全错，所以判 BLOCK 不判 WARN。
    # 2026-09-13（审计 §2.6）：以前写死 `== 5`。对数由 frozen.yaml 自己的 left_/right_ 声明推出
    # （left_ 数 == right_ 数 == 配到的对数，且每对的两个 body 都在 MJCF 里），加一对腿或少一个关节只改数据。
    ok = bool(lefts) and len(mirror) == len(lefts) == len(rights) and not unmatched
    res.add(subject="_mjcf", check="mirror_body_pairs", state=PASS if ok else FAIL,
            severity=BLOCK, measured=len(mirror),
            criterion="frozen.yaml:joint_axes 里每个 left_X 都有同名 right_X、双方 mjcf_body 都在 MJCF 里："
                      f"配到的对数 == 数据声明的对数（left_ {len(lefts)} 个 == right_ {len(rights)} 个）",
            evidence_n=len(jb),
            detail=f"配到 {sorted(mirror.items())}" + (f"；配不上：{unmatched}" if unmatched else ""),
            provenance="frozen.yaml:joint_axes[].name / joint_axes[].mjcf_body")

    sig = {}
    for stem, p in placed.items():
        # 元规则 6/7（审计 F-核-2）：本层的世界质心、舵机/电池/轴承实体、orig_* 全部读自 placed/，
        # 以前只声明 18 个导出件 —— placed 改了格子不会 STALE。
        res.inputs.append(_rel(p))
        try:
            sig[stem] = _stl_sig(p)
        except Exception as e:                                  # noqa: BLE001
            res.unknown("_layer", f"placed_sig:{stem}", f"placed 实体读不出指纹（{e}）—— 它承载的质量会从整机量里消失",
                        provenance=_rel(p))
            continue

    out = {}
    for pid in ctx.parts:
        stl = ctx.stl(pid)
        if stl is None:
            res.unknown(pid, "stl_exists", f"cad/duck_s288/ 里找不到 {pid} 的导出 STL",
                        provenance="parts.yaml:parts[].id")
            continue
        n, v = _stl_sig(stl)
        cands = [s for s, (sn, sv) in sig.items()
                 if sn == n and abs(abs(sv) - abs(v)) <= 1e-5 * max(abs(v), 1.0)]
        if not cands:
            res.unknown(pid, "placed_instance", f"placed/ 里没有与导出件指纹一致的世界坐标件（{n} 面）",
                        provenance="cad/duck_s288/placed/")
            continue
        # 用**体积质心**而不是顶点均值：process=True 合点后两份 STL 的顶点集不一定一一对应
        # （L06 鞋底 63376 面，顶点均值会差 0.2 mm），体积质心是严格的刚体不变量（实测残差 < 4e-6）。
        cl = np.asarray(_mesh(stl).center_mass, float)
        rows = []
        for s in cands:
            cw = np.asarray(_mesh(placed[s]).center_mass, float)
            ds = sorted((float(np.linalg.norm(B[b]["T_world"][:3, :3] @ cl + B[b]["T_world"][:3, 3] - cw)), b)
                        for b in order)
            rows.append((s, ds[0][1], ds[0][0], ds[1][0]))       # (stem, 最近 body, 残差, 次近残差)
        base_stem, base_body, resid, second = min(rows, key=lambda r: r[2])
        if resid > 1e-3:
            res.unknown(pid, "body_assignment",
                        f"没有 MJCF body 能把导出件的体积质心映射到 placed 件的世界质心（最小残差 "
                        f"{resid:.4f} mm > 1e-3，最好的是 {base_stem}→{base_body}）",
                        provenance="frozen.yaml:joint_axes / duckstructure.kin")
            continue
        # 镜像实例必须**验**，不能只按 frozen 的 left/right 表断言。
        # 右件 = mirror_y(左件世界网格)，所以 mirror_y(TW(左body) @ 局部质心) 必须落在右件的世界质心上，
        # 且那个右 body 就是 frozen 表给的那个。frozen 的 left/right 配错时，这里必须红。
        pairs, notes, ev = [(base_stem, base_body)], [f"{base_stem}→{base_body} 残差 {resid:.3e}"], len(order)
        bad = []
        for s, b, _d, _s2 in rows:
            if s == base_stem:
                continue
            mb = mirror.get(base_body)
            if mb is None:
                bad.append(f"{s}: frozen.yaml 里 {base_body} 没有对侧 body")
                continue
            T0, T1 = B[base_body]["T_world"], B[mb]["T_world"]
            w_left = T0[:3, :3] @ cl + T0[:3, 3]
            w_mirror = np.array([w_left[0], -w_left[1], w_left[2]])     # mirror_y（duckstructure/lib.py:446）
            cw = np.asarray(_mesh(placed[s]).center_mass, float)
            r_world = float(np.linalg.norm(w_mirror - cw))
            # 光有"几何是左件的镜像"还不够 —— 还要证明 frozen 表给的这个 mb **就是左 body 的镜像 body**，
            # 否则把 right_ankle 写成 leg_2 这种错配零位姿下看不出来（世界残差照样是 0）。
            # 判据：body 原点与关节轴（本体系 +z）都必须是左 body 的 y 镜像。
            o0, o1 = T0[:3, 3], T1[:3, 3]
            a0, a1 = T0[:3, 2], T1[:3, 2]
            r_org = float(np.linalg.norm(np.array([o0[0], -o0[1], o0[2]]) - o1))
            r_ax = float(np.linalg.norm(np.array([a0[0], -a0[1], a0[2]]) - a1))
            ev += 3
            if r_world > 1e-3 or r_org > 1e-3 or r_ax > 1e-6:
                bad.append(f"{s}→{mb}: 世界残差 {r_world:.4f} mm / body 原点镜像残差 {r_org:.4f} mm / "
                           f"关节轴镜像残差 {r_ax:.6f}")
                continue
            pairs.append((s, mb))
            notes.append(f"{s}→{mb} 镜像残差 {max(r_world, r_org, r_ax):.3e}")
        if bad:
            res.unknown(pid, "body_assignment",
                        "镜像实例的 body 归属验不过（frozen.yaml 的 left/right 配对与几何对不上）："
                        + "；".join(bad) + f"。已验过的：{'; '.join(notes)}",
                        provenance="frozen.yaml:joint_axes[].mjcf_body / duckstructure/lib.py:MIRROR")
            continue
        out[pid] = pairs
        if ctx.only and pid not in ctx.only:
            continue          # 未选中的件不发 finding，否则它的格子只剩这条 INFO PASS → 假绿
        res.add(subject=pid, check="body_assignment", state=PASS, severity=INFO,
                measured=[f"{s}→{b}" for s, b in pairs],
                criterion="TW(body) @ 局部体积质心 == placed 世界体积质心（残差 < 1e-3 mm，且唯一）；"
                          "镜像实例另按 mirror_y 双向验残差",
                evidence_n=ev,
                detail="；".join(notes) + f"；次近的 body 差 {second:.2f} mm（唯一）",
                provenance="frozen.yaml:joint_axes[].mjcf_body / duckstructure.kin")
    return out, placed, B, order, mirror


# ── 元件认领规则（components.yaml:components[].claim）──────────────────────
# 2026-09-13（F-L7-6）：以前按 category 写死（servo→MJCF geom、battery/bearing→envelope 正则、
# cid=="original_prints"→orig_ 前缀），board/imu/camera/other 无规则、有质量也进不去，qty 从不校验 ——
# 换一颗舵机、加一个摄像头就得改代码。现在每条元件自己声明怎么在 placed/ 里认领实体，层只按规则做，
# **不再看 category**。规则内容就是把原来四个分支逐条搬进 yaml，没有发明新的匹配：
#   claim.by:         mjcf_servo_geom  —— placed/servo__<body>_<drives>，来自 MJCF body.servos（原 servo 分支）
#                     envelope_box     —— envelope_mm.v 里的 a×b×c 反算体积/包围盒去 placed 里认（原 battery 分支）
#                     envelope_ring    —— envelope_mm.v 里的 内径×外径×厚 反算环体积/包围盒（原 bearing 分支）
#                     placed_prefix    —— placed 名前缀 claim.pattern（原 original_prints 分支）
#   claim.body_from:  housed_by（缺省：housed_by 各件的实例 body，按最近 body 原点分配；跨互不镜像的多个 body → 不猜）
#                     mjcf_servo_host（mjcf_servo_geom 隐含：舵机所在 body）
#                     mjcf_part_mesh（原版件：哪个 body 的 visual geom 引用了这个网格名）
#   claim.mass_estimate_from_geometry: true —— mass_g 取不出数时按 实体体积 × 密度 推一个 INFO 值（原 orig_est 分支；
#                     推算值不冒充声明值，该元件仍算质量缺口）
# 没有 claim 的元件 → component_body:<id> unknown（不是静默漏掉）；认领到的实例数 ≠ qty → component_qty FAIL(BLOCK)。
_CLAIM_BY = ("mjcf_servo_geom", "envelope_box", "envelope_ring", "placed_prefix")


# ── 质量清单 ──────────────────────────────────────────────────────────────
def _mass_items(ctx, res, assign, placed, B, order, mirror):
    """[(名字, body, 质量 g, 世界质心 3, 类别, 来源)]，外加缺质量的清单、密度、逐项惯量张量、**缺口清单**。

    第 4 个返回值 inert_of[(名字, body)] = (关于该项自身质心、**世界轴**下的 3×3 惯量张量 g·mm²,
    质点模型下的同一张量（恒为 0）, 模型标签)。打印件按实体×密度；元件按 placed 里的**包络实体**
    配 mass_g 反算的均匀密度 —— 包络里的真实质量分布（PCB/电机转子）没有数据，所以同时留下质点模型
    的对照量，让"这条差异有多大"可见，而不是把包络假设当成事实。

    第 5 个返回值 gaps = [(id, 判据名, 一句话)]：质量清单的缺口（F-L7-4）—— qty>0 无质量 / 无 claim 规则 /
    认领数 ≠ qty / 定不出 body / 有质量认领不到实体 / qty 不是整数 / 密度无来源。
    整机汇总判据（static_torque / static_torque_home / com_in_support_* / head_mass_budget）看到 gaps 非空一律 unknown。"""
    import numpy as np
    load = (ctx.data.get("tolerances") or {}).get("load") or {}
    rho_node = load.get("density_g_cm3")
    rho_g_cm3, rho_src = num(rho_node)
    items, missing, inert_of, gaps = [], [], {}, []
    if rho_g_cm3 is None:
        res.unknown("_layer", "density", "tolerances.yaml:load.density_g_cm3 取不到，17 件质量算不了",
                    provenance="tolerances.yaml:load.density_g_cm3")
        gaps.append(("_layer", "density", "密度取不到"))
        return items, missing, None, inert_of, gaps
    if rho_src in (None, "unknown", ""):
        # 阈值/常数没有来源就不能用（与 L5 threshold_src 同规则）
        res.unknown("_layer", "density", f"tolerances.yaml:load.density_g_cm3 = {rho_g_cm3} 没有 src —— 常数没有来源，不能拿它算质量",
                    provenance="tolerances.yaml:load.density_g_cm3.src")
        gaps.append(("_layer", "density", "密度没有来源"))
        return items, missing, None, inert_of, gaps
    rho_note = (rho_node.get("src_note") if isinstance(rho_node, dict) else None) or ""
    if rho_src == "assumed":
        # 2026-09-13（审计 §2.6）：1.27 是 assumed 却从不产生任何 finding。照算，但留一条 WARN，值就是这个常数。
        res.add(subject="_layer", check="threshold_assumed:density_g_cm3", state=FAIL, severity=WARN,
                measured=rho_g_cm3,
                criterion="tolerances.yaml:load.density_g_cm3 的 src 应是 measured / datasheet；src=assumed 照算但不算有来源",
                evidence_n=1,
                detail=f"{rho_g_cm3} g/cm³，src=assumed（{rho_note}）。17 件质量、每 body 惯量、静扭矩、重心都乘着它；"
                       "实心体积 × 这个数是上界（见 load.density_by_material.infill_upper_bound_note），"
                       "称重（bench.yaml:BN15）之前它就是个假设",
                provenance="tolerances.yaml:load.density_g_cm3")
    rho = rho_g_cm3 / 1000.0                                     # g/mm³

    def _own_I(mesh, mass_g, tag):
        """网格关于自身质心、世界轴下的惯量张量 g·mm²（trimesh 的 moment_inertia 是关于质心、
        density=1 的体积惯量，×实际密度即得质量惯量；placed 网格本身就在世界系，无需再转）。"""
        v = float(mesh.volume)
        if v <= 0 or mass_g is None or mass_g <= 0:
            return None, tag
        return np.asarray(mesh.moment_inertia, float) * (mass_g / v), tag

    parts_by_id = {p["id"]: p for p in ((ctx.data.get("parts") or {}).get("parts") or [])}
    mats = set()
    for pid, pairs in assign.items():
        stl = ctx.stl(pid)
        loc = _mesh(stl)
        m_g = float(loc.volume) * rho
        mat = (parts_by_id.get(pid) or {}).get("material") or "?"
        mats.add(mat)
        for s, b in pairs:
            w = _mesh(placed[s])
            items.append((s, b, m_g, np.asarray(w.center_mass, float), "print", f"{mat} × {rho_g_cm3} g/cm³"))
            inert_of[(s, b)] = _own_I(w, m_g, "solid_mesh")
        if ctx.only and pid not in ctx.only:
            continue
        res.add(subject=pid, check="part_mass", state=PASS, severity=INFO, measured=round(m_g, 4),
                criterion=f"导出 STL 体积 × tolerances.yaml:load.density_g_cm3 = {rho_g_cm3}（src={rho_src!r}）",
                evidence_n=len(pairs),
                detail=f"{loc.volume:.3f} mm³ × {rho_g_cm3} g/cm³ = {m_g:.4f} g/件 × {len(pairs)} 件；"
                       f"材料 {mat}；文件 {stl.name}",
                provenance="tolerances.yaml:load.density_g_cm3 / parts.yaml:parts[].material")
    if any("TPU" in m for m in mats):
        res.unknown("_layer", "density_per_material",
                    f"tolerances.yaml:load 只有一个 density_g_cm3={rho_g_cm3}，"
                    f"但 parts.yaml 里同时有 {sorted(mats)} —— TPU 95A 用 PLA 的密度算，L06 的质量是错的",
                    provenance="tolerances.yaml:load.density_g_cm3 / parts.yaml:parts[].material",
                    severity=WARN)

    # 元件
    comps = (ctx.data.get("components") or {}).get("components") or []
    servo_hosts = {}
    for n in order:
        for i, s in enumerate(B[n]["servos"]):
            if s["drives"] is None:
                continue
            servo_hosts["servo__" + n + "_" + s["drives"].replace(":self", "")] = n
    part_stems = {s for pairs in assign.values() for s, _ in pairs}
    # 原版件归哪个 body：查 MJCF 里哪个 body 的 visual geom 引用了这个网格名，不写死 jaw_soft
    orig_body = {}
    for n in order:
        for p in B[n].get("parts") or []:
            orig_body["orig_" + str(p.get("mesh"))] = n
    r_to_l = {r: l for l, r in (mirror or {}).items()}

    def cand_bodies(comp):
        """housed_by 各件**全部实例**的 body（左右都要），以及去掉镜像后的"基"body 集合。
        以前只取每件第一个实例的 body → 右腿轴承会挂到左 body 上。"""
        bodies, bases = [], set()
        for pid in (comp.get("housed_by") or []):
            for _s, b in (assign.get(pid) or []):
                if b not in bodies:
                    bodies.append(b)
                bases.add(r_to_l.get(b, b))
        return bodies, bases

    def body_for(comp, com):
        """唯一的宿主连杆（可以是它的左右镜像对）；每个实体按**最近的 body 原点**落到左或右。
        housed_by 跨了多个互不镜像的连杆就返回 None（**不猜**）。
        轴承就是这种情况：外圈在载体、内圈在从动件，横跨两个连杆，data 里没说哪一颗装在哪 —— 只能判 unknown。"""
        bodies, bases = cand_bodies(comp)
        if len(bases) != 1:
            return None, bodies
        return min(bodies, key=lambda b: float(np.linalg.norm(B[b]["T_world"][:3, 3] - com))), bodies

    def env_triple(comp):
        e = comp.get("envelope_mm")
        v = e.get("v") if isinstance(e, dict) else None          # envelope_mm 写成裸字符串时不能崩
        m = re.search(rf"({_NUM})\s*[×xX*]\s*({_NUM})\s*[×xX*]\s*({_NUM})", v or "")
        return tuple(sorted(float(g) for g in m.groups())) if m else None

    def match_by_shape(want_vol, want_ext):
        """按 envelope 反算出的体积/包围盒去 placed 里认领实体（与第 3 层同一套规则）。"""
        got = []
        for s in sorted(placed):
            if s in part_stems or s.startswith(("servo__", "orig_")):
                continue
            w = _mesh(placed[s])
            e = tuple(sorted(np.round(w.bounds[1] - w.bounds[0], 6)))
            if all(abs(a - b) <= 0.05 for a, b in zip(e, want_ext)) \
                    and abs(w.volume - want_vol) / want_vol < 0.01:
                got.append(s)
        return got

    def claim_stems(comp, rule):
        """按 claim 规则认领 → ([(stem, body 或 None)], 怎么认的, 规则错误)。body 为 None 的由 body_for 定。"""
        by = rule.get("by")
        if by == "mjcf_servo_geom":
            return ([(s, b) for s, b in servo_hosts.items() if s in placed],
                    "MJCF body.servos → placed/servo__<body>_<drives>（body = 舵机所在 body）", None)
        if by == "placed_prefix":
            pat = rule.get("pattern")
            if not isinstance(pat, str) or not pat:
                return None, "", "claim.by=placed_prefix 但没有 claim.pattern"
            ss = [s for s in sorted(placed) if s.startswith(pat) and s not in part_stems]
            if rule.get("body_from") == "mjcf_part_mesh":
                nb = [s for s in ss if s not in orig_body]
                if nb:
                    return None, "", (f"placed 前缀 '{pat}' 认领到 {nb}，但 MJCF 里没有任何 body 的 visual geom "
                                      "引用这些网格名（claim.body_from=mjcf_part_mesh 定不出 body）")
                return [(s, orig_body[s]) for s in ss], f"placed 前缀 '{pat}'（body = MJCF visual geom 引用该网格的 body）", None
            return [(s, None) for s in ss], f"placed 前缀 '{pat}'（body 由 housed_by 定）", None
        if by in ("envelope_box", "envelope_ring"):
            tri = env_triple(comp)
            if tri is None:
                return None, "", f"claim.by={by} 但 envelope_mm.v 里解析不出 a×b×c 三个数：{(comp.get('envelope_mm') or {}).get('v')!r}"
            if by == "envelope_box":
                vol, ext = tri[0] * tri[1] * tri[2], tri
                how = f"envelope_mm {tri} 当长方体：体积 {vol:.1f} mm³ + 包围盒 ±0.05"
            else:
                # **不能按 'bearing' 前缀一把抓** —— 那样 6704 和 6700 会各自认领全部 6 颗，
                # 质量互相串到对方的关节上。按 envelope 的内径×外径×厚反算环体积区分（同第 3 层）。
                t, bore, od = tri
                vol, ext = math.pi / 4.0 * (od * od - bore * bore) * t, tuple(sorted((t, od, od)))
                how = f"envelope_mm {tri} 当环（内径×外径×厚）：环体积 {vol:.1f} mm³ + 包围盒 {ext} ±0.05"
            return [(s, None) for s in match_by_shape(vol, ext)], how + "（body 由 housed_by 定）", None
        return None, "", f"claim.by={by!r} 不认识（只支持 {list(_CLAIM_BY)}）"

    claimed, not_installed = [], []
    for c in comps:
        cid = c.get("id")
        qty = c.get("qty")
        mgf = c.get("mass_g")
        qty_i = qty if isinstance(qty, int) and not isinstance(qty, bool) else None
        if qty_i is None:
            res.unknown(cid, "component_qty",
                        f"components.yaml qty={qty!r} 不是整数 —— 装几个都说不清，这份元件（mass_g={mgf.get('v') if isinstance(mgf, dict) else mgf!r}）"
                        "进不了整机合计",
                        provenance="components.yaml:components[].qty")
            gaps.append((cid, "component_qty", f"qty={qty!r} 不是整数"))
            continue
        if qty_i <= 0:
            not_installed.append(cid)                            # qty=0 = 不装：对整机质量的贡献恒为 0，不要求质量
            continue
        mg, msrc = num(mgf if isinstance(mgf, dict) else None)
        if not isinstance(mg, (int, float)):
            why = (mgf.get("unknown_reason") if isinstance(mgf, dict) else None) \
                  or f"mass_g={mgf!r} 里取不到数字"
            missing.append((cid, qty_i, why))
            gaps.append((cid, "component_mass", f"qty={qty_i} 无可用 mass_g"))
            mg = None
        rule = c.get("claim")
        if not isinstance(rule, dict) or not rule.get("by"):
            # 没有规则 → 不猜、不静默：这份元件（不管有没有质量）定不出 body，整机量因此不全
            res.unknown(cid, "component_body",
                        f"components.yaml 没有给这条元件 claim 规则（qty={qty_i}，mass_g={mg}）—— "
                        "2026-09-13 起层不再按 category 猜它在 placed/ 里长什么样；认领不到就定不出 body，"
                        "这份质量进不了整机合计、每 body 漂移、静扭矩和重心投影。"
                        f"要清这条红：给它加 claim（by ∈ {list(_CLAIM_BY)}）并让 placed/ 里有它的实体",
                        provenance="components.yaml:components[].claim")
            gaps.append((cid, "component_body", "没有 claim 规则"))
            continue
        stems, how, err = claim_stems(c, rule)
        if err:
            res.unknown(cid, "component_claim_rule", f"claim 规则用不了：{err}",
                        provenance="components.yaml:components[].claim")
            gaps.append((cid, "component_claim_rule", err))
            continue
        n = len(stems)
        # qty 必须校验：认领到的实例数就是"装了几个"，与清单不符就是清单或 placed 有一边错了
        res.add(subject=cid, check="component_qty", state=PASS if n == qty_i else FAIL,
                severity=BLOCK if n != qty_i else INFO, measured=n,
                criterion=f"按 claim 规则（by={rule.get('by')}）在 placed/ 里认领到的实例数 == components.yaml qty（{qty_i}）",
                evidence_n=len(placed),
                detail=f"{how}；认领到 {n} 个：{[s for s, _ in stems][:8]}"
                       + ("" if n == qty_i else f" —— **≠ qty {qty_i}**：清单与 placed/ 有一边错了，整机量按哪个都不对"),
                provenance="components.yaml:components[].qty / components[].claim / cad/duck_s288/placed/")
        if n != qty_i:
            gaps.append((cid, "component_qty", f"认领到 {n} ≠ qty {qty_i}"))
        if not stems:
            # 有质量却认领不到实体 —— 必须红。否则"把 mass_g 填上"反而会让 missing 里的红消失，
            # 而这几十克从头到尾没进过质量/扭矩/重心（补数据让 Gate 变空，元规则 4 的反向违反）。
            if mg is not None:
                res.unknown(cid, "world_placement_for_mass",
                            f"components.yaml 给了 mass_g={mg} g（qty={qty_i}），但 placed/ 里按 claim 规则认领不到它的实体 —— "
                            "这份质量进不了整机合计、每 body 漂移、静扭矩和重心投影",
                            provenance="components.yaml:components[].mass_g / cad/duck_s288/placed/")
                gaps.append((cid, "world_placement_for_mass", f"mass_g={mg} 但认领到 0 个实体"))
            continue
        rows = []
        for s, b in stems:
            w = _mesh(placed[s])
            com = np.asarray(w.center_mass, float)
            cands = None
            if b is None:
                b, cands = body_for(c, com)
            rows.append((s, b, w, com, cands))
        nob = [(s, cands) for s, b, _w, _c, cands in rows if b is None]
        if nob:
            # body 定不下来 = 这份质量挂不到任何连杆上，只能红，不能静默跳过
            res.unknown(cid, "component_body",
                        f"认领到 {len(nob)} 个实体（{', '.join(s for s, _ in nob[:6])}）但定不出它们属于哪个 MJCF body："
                        f"housed_by={c.get('housed_by')} → 落在 {nob[0][1]} 这些 body 上（去掉左右镜像后仍不止一个连杆）。"
                        "跨连杆的件（轴承外圈在载体、内圈在从动件）data 里没写哪一颗装在哪，本层**不猜** —— "
                        "这份质量因此进不了逐 body 漂移与静扭矩",
                        provenance="components.yaml:components[].housed_by")
            gaps.append((cid, "component_body", f"{len(nob)} 个实体定不出 body"))
            rows = [r for r in rows if r[1] is not None]
            if not rows:
                continue
        claimed.append(cid)
        for s, b, w, com, _cands in rows:
            if mg is None:
                # 原版件的 mass_g 是 'PLA≈49.8 / TPU≈27.1' 这种字符串，取不出数；
                # 但它们是真的打印件，能按几何×密度自己算 —— 算出来只作 INFO，不冒充声明值（仍是缺口）。
                if rule.get("mass_estimate_from_geometry") is True:
                    est = float(w.volume) * rho
                    items.append((s, b, est, com, "orig_est", f"几何 × {rho_g_cm3} g/cm³（推算，非声明）"))
                    inert_of[(s, b)] = _own_I(w, est, "solid_mesh")
                continue
            items.append((s, b, float(mg), com, str(c.get("category") or "component"), f"components.yaml mass_g（{msrc}）"))
            inert_of[(s, b)] = _own_I(w, float(mg), "envelope_uniform")
    res.add(subject="_layer", check="component_claims", state=PASS, severity=INFO, measured=len(claimed),
            criterion="按 components.yaml:components[].claim 认领并挂到 body 的元件数（簿记，不判通过）",
            evidence_n=len(comps),
            detail=f"认领并挂到 body：{claimed}；qty=0 不装、不要求质量：{not_installed}；"
                   f"清单缺口 {len(gaps)} 项：{[f'{i}({k})' for i, k, _ in gaps]}",
            provenance="components.yaml:components[].claim / [].qty")
    return items, missing, rho_g_cm3, inert_of, gaps


# ── FK ────────────────────────────────────────────────────────────────────
def _fk(B, order, jb_of_body, q):
    """q: joint 名 -> rad。返回 body -> 4x4 世界位姿。零位时与 kin 的 T_world 完全一致（层内已断言）。"""
    import numpy as np
    T = {}

    def walk(b, Tp):
        Tb = Tp @ B[b]["T_parent"]
        jn = jb_of_body.get(b)
        if jn is not None:
            Tb = Tb @ _Rz(q.get(jn, 0.0))
        T[b] = Tb
        for c in B[b]["children"]:
            walk(c["name"], Tb)

    walk(order[0], np.eye(4))
    return T


def _descend(B, b):
    out = [b]
    for c in B[b]["children"]:
        out += _descend(B, c["name"])
    return out


# ── 主流程 ────────────────────────────────────────────────────────────────
def run(ctx) -> LayerResult:
    import numpy as np
    res = LayerResult(LAYER, NAME)

    if ctx.only:
        # 整机量（质量合计、每 body 漂移、各关节静扭矩、重心投影）少一个件就全错。
        # 所以 --parts 只筛"逐件"那几条判据，整机量照样按 17 件全算 —— 这件事必须写在记分卡上，
        # 不能让人以为跑了子集就得到了同样的整机结论。
        res.add(subject="_layer", check="scope_vs_parts_filter", state=PASS, severity=WARN,
                measured=sorted(ctx.only),
                criterion="--parts 只影响逐件判据（part_mass / min_section），整机判据一律按全部 17 件算",
                evidence_n=len(ctx.only),
                detail="质量合计 / 每 body 漂移 / 各关节静扭矩 / 重心投影都是整机量，"
                       "少算一个件结果就是错的，所以本层不按 --parts 裁剪它们",
                provenance="tools/gate/layers/__init__.py: ctx.only")

    # ── 只读 data 的判据放在最前面：与质量清单无关，_assign 提前 return 也必须跑到 ────────
    rated, rated_src = _torque_source(ctx, res)                 # F-L7-5：额定扭矩唯一来源
    _forcerange_vs_servo(ctx, res)                              # MJCF actuator forcerange vs 舵机额定
    ja = [j for j in ((ctx.data.get("frozen") or {}).get("joint_axes") or [])
          if isinstance(j, dict) and j.get("exists_as_mjcf_joint")]
    home = _home_pose(res, ja)                                  # §2.6：home_rad 缺失 = unknown，不当 0

    assign, placed, B, order, mirror = _assign(ctx, res)
    if not assign:
        return res
    for pid in assign:
        res.inputs.append(_rel(ctx.stl(pid)))
    # F-L7-4：没能归属到 body 的打印件也是整机质量的缺口（它们的 unknown 已在 _assign 里逐件发出）
    gaps = [(pid, "body_assignment", "导出件没有归属到 MJCF body（见同件的 stl_exists / placed_instance / body_assignment）")
            for pid in ctx.parts if pid not in assign]

    # 逐件的 L7 判据（承力最小截面 / 薄筋长厚比 / 层向 vs 受力）三个阈值在 tolerances.yaml 里全是 null，
    # 所以**没有任何一个件的 L7 格子能判通过**。这条必须在这里就发出来（而不是等到最后的 _sections）：
    # 中途任何一次提前 return 都不能留下"只有 body_assignment 通过 → 格子变绿"的假绿。
    _msr = ((ctx.data.get("tolerances") or {}).get("load") or {}).get("min_section_and_rib") or {}
    _lack = [k for k in ("min_load_section_mm2", "max_rib_slenderness") if num(_msr.get(k))[0] is None]
    for pid in sorted(assign):
        if ctx.only and pid not in ctx.only:
            continue
        if not _lack:
            continue                                             # 阈值齐了就不发这条，真判据在 _sections 里
        res.unknown(pid, "load_criteria",
                    "本件的 L7 逐件强度判据（承力最小截面 / 薄筋长厚比 / 层向 vs 受力）在 "
                    f"tolerances.yaml:load.min_section_and_rib 里还缺 {_lack} —— "
                    "2026-09-09 起**量的东西已经修好了**（沿 relations.yaml:load_paths 声明的承力路径切法平面、"
                    "按 features.yaml:kind=rib 的探针量筋），同格的 load_section / rib_thickness 都是"
                    "真实测出来的数；缺的只剩那两个材料门槛（PLA/TPU 的许用应力，本仓库没有试件数据）",
                    provenance="tolerances.yaml:load.min_section_and_rib / relations.yaml:load_paths")

    items, missing, _rho, inert_of, comp_gaps = _mass_items(ctx, res, assign, placed, B, order, mirror)
    gaps += comp_gaps
    if not items:
        # 质量清单空了，后面的整机量一条都没跑 —— 不能让 _mjcf 只剩一条 mirror_body_pairs 的绿
        res.unknown("_mjcf", "mjcf_comparison",
                    "质量清单为空（见同层 _layer 的红），逐 body 漂移、静扭矩、重心投影全部没跑",
                    provenance="README 第 2 节 L7")
        return res

    # 缺质量的元件：逐条红（元规则 4）
    for cid, qty, why in missing:
        res.unknown(cid, "component_mass",
                    f"components.yaml mass_g 没有可用数值（qty={qty}）：{why}",
                    provenance="components.yaml:components[].mass_g")

    jb_of_body = {j["mjcf_body"]: j["name"] for j in ja if j.get("mjcf_body") in B}

    # 零位 FK 必须与 kin 的 T_world 逐 body 相等，否则后面所有姿态都不可信
    T0 = _fk(B, order, jb_of_body, {})
    resid = max(float(np.abs(T0[b] - B[b]["T_world"]).max()) for b in order)
    res.add(subject="_fk", check="zero_pose_consistency", state=PASS if resid < 1e-9 else FAIL,
            severity=BLOCK, measured=resid,
            criterion="本层 FK 在零位与 duckstructure.kin 的 T_world 逐 body 相等（< 1e-9）",
            evidence_n=len(order), detail=f"{len(order)} 个 body，最大元素差 {resid:.3e}",
            provenance="duckstructure.kin.load / frozen.yaml:joint_axes")

    # 件的局部坐标（在自己 body 系里），后面所有姿态都从这里算
    loc_items = []
    loc_I = {}                     # (名字, body) -> (惯量张量 g·mm²（body 轴、关于该项自身质心）, 模型标签)
    for nm, b, mg, com, kind, src in items:
        if b is None:
            continue
        Ti = np.linalg.inv(B[b]["T_world"])
        loc_items.append((nm, b, mg, Ti[:3, :3] @ com + Ti[:3, 3], kind, src))
        Iw, tag = inert_of.get((nm, b), (None, None))
        Rb = B[b]["T_world"][:3, :3]
        loc_I[(nm, b)] = (None if Iw is None else Rb.T @ Iw @ Rb, tag)
    total_g = sum(i[2] for i in loc_items)

    # ── 逐 body 质量与质心 vs MJCF <inertial> ─────────────────────────────
    inert = _mjcf_inertial(res)
    drift_tol = ((ctx.data.get("tolerances") or {}).get("load") or {}).get("mass_drift_vs_mjcf") or {}
    drift_m, _ = num(drift_tol.get("mass_g") if isinstance(drift_tol, dict) else None)
    drift_c, _ = num(drift_tol.get("com_mm") if isinstance(drift_tol, dict) else None)
    if drift_m is None or drift_c is None:
        res.unknown("_mjcf", "mass_drift_threshold",
                    "tolerances.yaml:load.mass_drift_vs_mjcf 里取不到 {mass_g, com_mm} 这对上限 —— "
                    "README 第 7 层要求逐 body 比漂移，但没给判据。数照量，只能报 INFO。"
                    "把这个桶填上（例如 mass_g: {v: 5}, com_mm: {v: 2}）本层立刻改成真判据",
                    provenance="README 第 2 节 L7 / tolerances.yaml:load.mass_drift_vs_mjcf")
    itol = ((ctx.data.get("tolerances") or {}).get("load") or {}).get("inertia_drift_vs_mjcf") or {}
    inert_rel, _ = num(itol.get("principal_moment_rel") if isinstance(itol, dict) else None)
    inert_deg, _ = num(itol.get("principal_axis_deg") if isinstance(itol, dict) else None)
    if inert_rel is None or inert_deg is None:
        res.unknown("_mjcf", "inertia_drift_threshold",
                    "tolerances.yaml:load.inertia_drift_vs_mjcf 里取不到 "
                    "{principal_moment_rel, principal_axis_deg} 这对上限 —— 惯量张量两边都算出来了"
                    "（CAD 侧对角化取主惯量，MJCF 侧读 fullinertia/diaginertia），但没有可判的门槛。"
                    "**不许随手填一个 ±10%**：唯一候选出处 MASS_INERTIA_RANDOMIZATION_RANGE 是 "
                    "dr.pseudo_inertia 的 e^(2α) **各向同性缩放**，主轴朝向与主惯量之比一个都不变，"
                    "所以它约束不了『分布』这件事。要什么才能解开见该键的 what_would_unblock",
                    provenance="README 第 2 节 L7 / tolerances.yaml:load.inertia_drift_vs_mjcf")
    if inert:
        # ── 2026-09-10：这一整块比较的**权威方向已经倒转** ────────────────────────
        # 过去 MJCF 是真理、CAD 必须符合它；2026-09-09 拍板重训（docs/重训路线_2026-09-09.md）之后
        # 是 CAD + 实测是真理、MJCF 由 CAD 生成（tools/sim/make_mjcf.py → sim/duck_s288/robot_walk_s288.xml）。
        # 所以"逐 body 与**上游** <inertial> 比漂移"这件事失去了放行权。下面这一条把理由讲一次，
        # 每个 body 的 drift_criteria 只留指针 —— 那 15 条随之从 BLOCK-FAIL 降为 RETIRED。
        _rho_node = ((ctx.data.get("tolerances") or {}).get("load") or {}).get("density_g_cm3") or {}
        _rho_v, _rho_src = num(_rho_node)
        # 上游密度反解的数字**一律从 tolerances.yaml 现读**，不在这里手抄第二遍。
        # 2026-09-10 复审抓到：手抄那一版有五处错（漏了 ankle_left/right 两个 body、
        # trunk_base 与 jaw_soft 的差值写反了符号、"10/15"实为 8/15）。
        # 数据表是对的，错的是转录 —— 所以改成拼数据，让它没有再错一次的机会（元规则 8/9）。
        _up = _rho_node.get("vs_upstream_mjcf_implied_density") or {}
        _up_rho = (num(_up.get("implied_plastic_density_g_cm3"))[0]
                   if isinstance(_up.get("implied_plastic_density_g_cm3"), dict) else None)
        _up_tog = ((_up.get("implied_plastic_density_g_cm3") or {}).get("solved_together_with") or {}
                   if isinstance(_up.get("implied_plastic_density_g_cm3"), dict) else {})
        _up_xl, _up_brg = _up_tog.get("xl330_g"), _up_tog.get("bearing_22x16x4_g")
        _up_res = ((_up.get("implied_plastic_density_g_cm3") or {}).get("residual_max_g")
                   if isinstance(_up.get("implied_plastic_density_g_cm3"), dict) else None)
        _up_bad = ((_up.get("bodies_that_do_not_fit") or {}).get("rows") or [])
        _up_bad_txt = "、".join(f"{r.get('body')} 差 +{r.get('unexplained_g')} g" for r in _up_bad[:4]) \
                      + ("…" if len(_up_bad) > 4 else "")
        _up_gap = _up.get("caliber_gap_vs_our_1_27")
        _up_gap_txt = (_up_gap.get("ratio_note") if isinstance(_up_gap, dict) else None) or \
                      (f"{(_rho_v / _up_rho - 1) * 100:.1f}%" if (_rho_v and _up_rho) else "未知")
        # 2026-09-13（F-核-5）：基准失去权威 = RETIRED，不是 STALE（STALE 在 core 里是"输入变了没重跑"）。
        res.add(subject="_mjcf", check="mjcf_baseline_authority", state=RETIRED, severity=WARN,
                measured=len(inert),
                criterion="【已退役（基准失去权威），保留为记录】逐 body 质量/质心漂移的**基准**是否还有权威 —— 基准 = 上游 "
                          "upstream/microduck_rl/.../robot_walk.xml 的 <inertial>",
                evidence_n=len(inert),
                detail="**本条已退役，下面 15 条 drift_criteria 一并退役（RETIRED）**，理由两条，互相独立：\n"
                       "① **权威方向倒转**。2026-09-09 决定重新训练、不再迁就原版 Microduck 的质量分布"
                       "（docs/重训路线_2026-09-09.md）。阈值本身也来自旧策略："
                       "tolerances.yaml:load.mass_drift_vs_mjcf.mass_g = 0.286 g 的 src_note 写着出处是上游 "
                       "microduck_velocity_env_cfg.py:73 的 MASS_INERTIA_RANDOMIZATION_RANGE —— "
                       "那是**旧策略的域随机化范围**，新策略还没训，它约束不了任何东西。\n"
                       "② **密度口径根本不同**。本层用 tolerances.yaml:load.density_g_cm3 = "
                       f"{_rho_v} g/cm³（src={_rho_src!r}，**本层实读**）；"
                       f"而上游 <inertial> 隐含的塑料密度是 **{_up_rho} g/cm³**，两者差 **{_up_gap_txt}**，"
                       "比 0.286 g 这个阈值大一个量级 —— 也就是说 15/15 全红里有一个系统性偏置，"
                       "红得不是地方。\n"
                       "  上游密度的来历（**本层代码没有算它**，是 2026-09-10 本轮离线复算的结果，"
                       "方法可复现）：取上游 robot_walk.xml 里 7 个 body（yaw2roll / bearing_roll / hip_l / "
                       "leg / leg_2 / neck / neck_pitch，其中 bearing_roll、leg_2 是前两者的镜像，"
                       "**不同几何只有 5 个**），每个 body 的 <inertial mass> = ρ×(该 body 全部塑料 visual "
                       "网格体积之和) + n_xl330×m_xl330 + n_bearing×m_bearing，三个未知数最小二乘 → "
                       f"ρ={_up_rho} g/cm³、m_xl330={_up_xl} g、m_bearing(22×16×4)={_up_brg} g，"
                       f"7 条残差最大 {_up_res} g。另外 {len(_up_bad)} 个 body 用同一组解拟不上"
                       f"（{_up_bad_txt}；**差值为正 = 上游比按密度算出来的重**，"
                       "说明上游那几个 <inertial> 上挂着 mesh 之外的质量），本条只用能拟合的 7 个。\n"
                       "**不要把基准改指向 sim/duck_s288/robot_walk_s288.xml 来'修好'这件事**："
                       "那份 MJCF 的 <inertial> 是 tools/sim/make_mjcf.py 用**和本层同一套算法、"
                       "同一批 CAD 网格、同一个 1.27 g/cm³** 算出来的，指过去会让漂移恒等于零"
                       "（大部分 body 的质量差落在 1e-4 g 量级；**本层没有算这个数**，"
                       "2026-09-10 离线实测是 8/15），那是比现在更隐蔽的假绿 —— "
                       "同义反复不是通过。正确出路是实测（称重）或重训后的新基准。",
                provenance="docs/重训路线_2026-09-09.md / tolerances.yaml:load.mass_drift_vs_mjcf.mass_g "
                           "/ tolerances.yaml:load.density_g_cm3 / "
                           "upstream/microduck_rl/.../robot_walk.xml:<inertial>")
        mj_total = sum(v["mass_kg"] for v in inert.values()) * 1000.0
        for b in order:
            sel = [i for i in loc_items if i[1] == b]
            ms = sum(i[2] for i in sel)
            mj = inert.get(b) or {}
            mm, cm = mj.get("mass_kg"), mj.get("com_mm")
            if mm is None:
                res.unknown(f"body:{b}", "mjcf_inertial", f"MJCF 里 body {b} 没有 <inertial>")
                continue
            if ms <= 0:
                # 一个有惯量的 body 在 CAD 侧一件一元件都没有 —— 归属规则漏了东西，不能拿 (0,0,0) 当质心报数
                res.unknown(f"body:{b}", "body_mass_vs_mjcf",
                            f"CAD 侧这个 body 上一件都没有（MJCF 给了 {mm*1000:.3f} g）—— "
                            "件/元件的 body 归属漏了，质心无从谈起",
                            provenance="robot_walk.xml:<inertial> / frozen.yaml:joint_axes")
                continue
            com = sum(i[2] * np.asarray(i[3]) for i in sel) / ms
            mm_g = mm * 1000.0
            d_m = ms - mm_g
            d_c = float(np.linalg.norm(com - cm))
            res.add(subject=f"body:{b}", check="body_mass_vs_mjcf", state=PASS, severity=INFO,
                    measured=round(d_m, 3),
                    criterion="CAD 质量 − **上游** MJCF <inertial mass>（纯数值，不判通过；"
                              "这个基准已过期，见 L07/_mjcf.mjcf_baseline_authority）",
                    evidence_n=len(sel),
                    detail=f"CAD {ms:.3f} g vs MJCF {mm_g:.3f} g → 漂移 {d_m:+.3f} g "
                           f"({(d_m/mm_g*100 if mm_g else float('nan')):+.1f}%)；"
                           f"计入 {len(sel)} 项：{', '.join(i[0] for i in sel)}",
                    provenance="robot_walk.xml:<inertial mass> / components.yaml:mass_g")
            if drift_m is None or drift_c is None:
                # 没有阈值就没有"通过"：这条让每个 body 的格子红着，而不是靠两条 INFO 数值变绿
                res.unknown(f"body:{b}", "drift_criteria",
                            f"这个 body 的质量漂移 {d_m:+.3f} g / 质心漂移 {d_c:.3f} mm 已经量出来了，"
                            "但 tolerances.yaml:load.mass_drift_vs_mjcf 里缺 "
                            f"{'mass_g' if drift_m is None else ''}"
                            f"{'/' if drift_m is None and drift_c is None else ''}"
                            f"{'com_mm' if drift_c is None else ''} —— 判不了",
                            provenance="README 第 2 节 L7 / tolerances.yaml:load.mass_drift_vs_mjcf")
            else:
                # 2026-09-10：**降级为 RETIRED**（09-13 前误写成 STALE）。数照量（下面 measured 是真的），但这个比较的基准
                # （上游 <inertial>）和阈值（旧策略的域随机化范围）都已失去权威，理由见
                # 同层 L07/_mjcf.mjcf_baseline_authority。原来这里是 PASS/FAIL + BLOCK。
                ok = abs(d_m) <= drift_m and d_c <= drift_c
                res.add(subject=f"body:{b}", check="drift_criteria", state=RETIRED, severity=WARN,
                        measured=[round(d_m, 3), round(d_c, 3)],
                        criterion=f"【已退役（基准失去权威），保留为记录】|质量漂移| ≤ {drift_m} g 且 质心漂移 ≤ {drift_c} mm，"
                                  "基准是**上游** robot_walk.xml 的 <inertial>",
                        evidence_n=len(sel),
                        detail=f"质量 {d_m:+.3f} g / 质心 {d_c:.3f} mm；"
                               f"按旧阈值这条{'满足' if ok else '**不满足**'}。"
                               "**但这个判断已经不作数**：基准与阈值都属于旧策略，且两侧密度口径"
                               "密度口径不同（比例见 tolerances.yaml:load.density_g_cm3.vs_upstream_mjcf_implied_density）—— 完整理由见 L07/_mjcf.mjcf_baseline_authority。"
                               "本条不再给放行，也不再以 BLOCK-FAIL 的身份指认这个 body 有问题",
                        provenance="tolerances.yaml:load.mass_drift_vs_mjcf / "
                                   "docs/重训路线_2026-09-09.md / L07/_mjcf.mjcf_baseline_authority")
            res.add(subject=f"body:{b}", check="body_com_vs_mjcf", state=PASS, severity=INFO,
                    measured=round(d_c, 3),
                    criterion="CAD 质心 − **上游** MJCF <inertial pos>，body 局部系（纯数值，不判通过；"
                              "这个基准已过期，见 L07/_mjcf.mjcf_baseline_authority）",
                    evidence_n=len(sel),
                    detail=f"CAD {np.round(com,3).tolist()} vs MJCF {np.round(cm,3).tolist()} mm → 距离 {d_c:.3f} mm",
                    provenance="robot_walk.xml:<inertial pos>")
            _inertia_body(res, b, sel, com, loc_I, mj, inert_rel, inert_deg)
        res.add(subject="_mjcf", check="total_mass_vs_mjcf", state=PASS, severity=INFO,
                measured=round(total_g - mj_total, 3),
                criterion="整机质量 − **上游** MJCF 15 个 body 之和（纯数值，不判通过；"
                          "这个基准已过期，见同格 mjcf_baseline_authority）",
                evidence_n=len(loc_items),
                detail=f"CAD 合计 {total_g:.3f} g（{len(loc_items)} 项）vs MJCF {mj_total:.3f} g "
                       f"→ {total_g - mj_total:+.3f} g（{(total_g-mj_total)/mj_total*100:+.1f}%）。"
                       f"**CAD 侧质量清单还有 {len(gaps)} 项缺口**（{_gap_text(gaps)}），补上只会更重",
                provenance="robot_walk.xml:<inertial>")

    # ── 各关节静扭矩 ──────────────────────────────────────────────────────
    load = (ctx.data.get("tolerances") or {}).get("load") or {}
    sob = load.get("servo_output_bearing") or {}
    margin_v, _ = num(sob.get("static_torque_margin"))
    if margin_v is None:
        res.unknown("_load", "static_torque_margin",
                    "tolerances.yaml:load.servo_output_bearing.static_torque_margin = null —— "
                    "README 第 7 层的『< 额定 × 系数』判不了：**系数**没有数。"
                    f"（额定本身是有的：{_RATED_KEY}，同层 _actuator.forcerange_vs_servo_rating 读的就是它。）"
                    f"下面各关节只按**必要条件 系数=1** 判（额定 {rated} N·m，src={rated_src!r}）；"
                    "0.6 N·m 是客服参数表的名义**堵转**值，不是可持续输出，实际系数只会更小",
                    provenance="tolerances.yaml:load.servo_output_bearing.static_torque_margin")
    if not sob.get("rated_radial_N") or num(sob.get("rated_radial_N"))[0] is None:
        res.unknown("_load", "output_bearing_rating",
                    "tolerances.yaml:load.servo_output_bearing 的 rated_radial_N / rated_axial_N 都是 null —— "
                    "relations.yaml:C2_missing_bearing_load 要求对少掉 7 颗轴承的关节单独算悬臂载荷并与额定比，"
                    "没有额定值就比不了",
                    provenance="relations.yaml:C2_missing_bearing_load / tolerances.yaml:load.servo_output_bearing")

    from duckstructure.checks import check_angles          # 独立采样网格（元规则 3）

    def torque(jname, body, q):
        T = _fk(B, order, jb_of_body, q)
        Tb = T[body]
        n = Tb[:3, 2] / np.linalg.norm(Tb[:3, 2])
        p = Tb[:3, 3] / 1000.0
        sub = set(_descend(B, body))
        tau = np.zeros(3)
        dist = 0.0
        for nm, b, mg, cl, kind, src in loc_items:
            if b not in sub:
                continue
            r = (T[b][:3, :3] @ cl + T[b][:3, 3]) / 1000.0
            tau += np.cross(r - p, np.array([0.0, 0.0, -mg / 1000.0 * _G]))
            dist += mg
        return float(n @ tau), dist

    # F-L7-4：质量清单有缺口 → 下面每个关节都是 unknown，数只当"仅含已知质量的下界"写进 detail
    gap_txt = _gap_text(gaps)
    _m = margin_v if margin_v is not None else 1.0
    crit_sweep = (f"|静扭矩| < 额定 {rated} N·m × static_torque_margin {margin_v}"
                  if margin_v is not None else
                  f"|静扭矩| < 额定 {rated} N·m × static_torque_margin；margin 是 null，"
                  f"这里只按**必要条件 margin=1** 判（见 L07/_load）") + \
        f"；额定来自 {_RATED_KEY}（src={rated_src!r}）；区间必须是有序的 [lo, hi] 且评估点数 ≥ 3" \
        "（duckstructure/checks.py:check_angles：2.5° 零度对齐网格 + 两端，lo<hi 且宽度 ≥ 2.5° 时必然 ≥ 3；" \
        "== 2 就是区间退化成两端点，不算扫）；质量清单不全 → unknown，不给下界当绿"
    crit_home = (f"home 姿态（frozen.yaml:joint_axes[].home_rad，全部关节同时置 home）下 |静扭矩| < 额定 {rated} N·m × "
                 f"static_torque_margin {margin_v if margin_v is not None else 'null（按必要条件 margin=1）'}"
                 f"；额定来自 {_RATED_KEY}（src={rated_src!r}）；质量清单不全 → unknown，不给下界当绿")
    worst_ratio, worst_j = 0.0, ""
    for j in ja:
        jn, body = j["name"], j["mjcf_body"]
        if body not in B:
            res.unknown(f"joint:{jn}", "static_torque", f"MJCF 里没有 body {body}")
            res.unknown(f"joint:{jn}", "static_torque_home", f"MJCF 里没有 body {body}")
            continue
        rng = j.get("range_deg")
        try:
            lo, hi = (float(x) for x in rng)
        except (TypeError, ValueError):
            lo = hi = None
        # F-L7-3：lo ≥ hi 一律 unknown，不许让 check_angles 退化成两端点（反写时它只返回 [hi, lo]）
        angles = check_angles(lo, hi) if (lo is not None and hi is not None and lo < hi) else []
        t0, dist = torque(jn, body, {})
        th = torque(jn, body, dict(home))[0] if home is not None else None
        best = (0.0, 0.0)
        for a in angles:
            t, _ = torque(jn, body, {jn: math.radians(float(a))})
            if abs(t) > abs(best[0]):
                best = (t, float(a))
        ratio = abs(best[0]) / rated if (rated and angles) else None
        ratio_h = abs(th) / rated if (rated and th is not None) else None
        if ratio is not None and ratio > worst_ratio:
            worst_ratio, worst_j = ratio, jn
        if rated is None:
            res.unknown(f"joint:{jn}", "static_torque",
                        f"额定扭矩取不到（{_RATED_KEY}）；实测零位 {t0:+.4f} / "
                        f"全程峰值 {best[0]:+.4f} N·m @ {best[1]}°（{len(angles)} 角）", provenance=_RATED_KEY)
            res.unknown(f"joint:{jn}", "static_torque_home",
                        f"额定扭矩取不到（{_RATED_KEY}）；home {th if th is None else f'{th:+.4f}'} N·m",
                        provenance=_RATED_KEY)
            continue
        # 系数未定 → 这个关节判不了。必要条件过了不等于放行，格子必须红着。
        # 系数一旦填进 tolerances.yaml，这条 unknown 自动消失，下面的判据自动收紧到 rated × margin。
        if margin_v is None:
            res.unknown(f"joint:{jn}", "torque_criteria",
                        f"实测峰值 {best[0]:+.4f} N·m = 名义堵转 {rated} N·m 的 "
                        f"{(ratio * 100) if ratio is not None else float('nan'):.1f}%，"
                        "但 tolerances.yaml:load.servo_output_bearing.static_torque_margin 是 null —— "
                        "只能证明满足 margin=1 的必要条件，不能判通过",
                        provenance="tolerances.yaml:load.servo_output_bearing.static_torque_margin")
        sweep_txt = (f"本关节全程（{lo}..{hi}°，{len(angles)} 个 2.5° 独立网格角，其余关节归零）峰值 "
                     f"{best[0]:+.4f} N·m @ {best[1]}° = 额定的 {(ratio or 0) * 100:.1f}%；"
                     f"零位 {t0:+.4f} N·m；该关节以下总质量 {dist:.1f} g")
        if len(angles) < 3:
            res.unknown(f"joint:{jn}", "static_torque",
                        f"frozen.yaml:joint_axes[{jn}].range_deg={rng!r} 不是有序的 [lo, hi]（要求 lo < hi），"
                        f"或退化到评估点数只有 {len(angles)}（< 3）—— 区间反写时 check_angles 只给两端点，"
                        "『扫了 2 个角』不是扫；本关节静扭矩不评（未知=失败）",
                        provenance="frozen.yaml:joint_axes[].range_deg / duckstructure/checks.py:check_angles")
        elif gaps:
            res.unknown(f"joint:{jn}", "static_torque",
                        f"质量清单不全 → unknown，不给下界当绿。仅含已知质量的**下界**：{sweep_txt}。{gap_txt}",
                        provenance="components.yaml:components[] / " + _RATED_KEY)
        else:
            res.add(subject=f"joint:{jn}", check="static_torque",
                    state=PASS if abs(best[0]) < rated * _m else FAIL, severity=BLOCK,
                    measured=round(ratio, 4), criterion=crit_sweep, evidence_n=len(angles),
                    detail=sweep_txt, provenance=f"frozen.yaml:joint_axes.range_deg / {_RATED_KEY} / "
                                                 "tolerances.yaml:load.servo_output_bearing")
        # F-L7-3：home 姿态扭矩单独判（以前算了只进 detail）。它与扫描区间无关（全部关节同时置 home）。
        if home is None:
            res.unknown(f"joint:{jn}", "static_torque_home",
                        "frozen.yaml:joint_axes 的 home_rad 声明不全（见 L07/_load.home_pose_declared）—— "
                        "home 姿态不存在，扭矩无从算起；不拿 0 顶替",
                        provenance="frozen.yaml:joint_axes[].home_rad")
        elif gaps:
            res.unknown(f"joint:{jn}", "static_torque_home",
                        f"质量清单不全 → unknown，不给下界当绿。仅含已知质量的**下界**：home 姿态 {th:+.4f} N·m "
                        f"= 额定的 {ratio_h * 100:.1f}%。{gap_txt}",
                        provenance="components.yaml:components[] / frozen.yaml:joint_axes[].home_rad")
        else:
            res.add(subject=f"joint:{jn}", check="static_torque_home",
                    state=PASS if abs(th) < rated * _m else FAIL, severity=BLOCK,
                    measured=round(ratio_h, 4), criterion=crit_home, evidence_n=len(home),
                    detail=f"home 姿态（{len(home)} 个关节按 frozen.yaml:home_rad 同时置位）{th:+.4f} N·m "
                           f"= 额定的 {ratio_h * 100:.1f}%；同关节单轴扫描峰值 {best[0]:+.4f} N·m —— "
                           "两者可以差很多：扫描时其余关节归零，home 时整机姿态不同（力臂不同）",
                    provenance=f"frozen.yaml:joint_axes[].home_rad / {_RATED_KEY}")
    if rated:
        res.add(subject="_load", check="worst_joint_torque_ratio", state=PASS, severity=INFO,
                measured=round(worst_ratio, 4),
                criterion="所有关节单轴扫描静扭矩占额定的最大比例（供选型/加负载时看；不判通过）", evidence_n=len(ja),
                detail=f"最吃力的是 {worst_j}，占额定 {rated} N·m 的 {worst_ratio*100:.1f}%"
                       + (f"。**质量清单不全，这是下界**：{gap_txt}" if gaps else ""),
                provenance="本层 static_torque 汇总")

    # ── 静态稳定 ─────────────────────────────────────────────────────────
    _stability(ctx, res, B, order, jb_of_body, loc_items, placed, assign, home, gaps)

    # ── 头部质量上限 ─────────────────────────────────────────────────────
    if rated:
        _head_budget(res, B, order, jb_of_body, loc_items, ja, rated, gaps)

    # ── 承力最小截面 / 薄筋 ───────────────────────────────────────────────
    _sections(ctx, res, assign, placed)

    res.evidence = {"parts_massed": len(assign),
                    "mass_items": len(loc_items),
                    "total_mass_g": round(total_g, 3),
                    "components_without_mass": len(missing),
                    "mass_inventory_gaps": len(gaps),
                    "joints": len(ja),
                    "rated_stall_torque_Nm": rated}
    return res


def _gap_text(gaps):
    """缺口清单 → 一句话（进 detail）。"""
    if not gaps:
        return "质量清单无缺口"
    return (f"质量清单缺口 {len(gaps)} 项：" +
            "；".join(f"{i}({k}：{w})" for i, k, w in gaps[:12]) + ("…" if len(gaps) > 12 else ""))


def _home_pose(res, ja):
    """frozen.yaml:joint_axes[].home_rad → {关节: rad}。**任何一个关节缺 home_rad（或不是数）→ None**，
    并发 `_load:home_pose_declared` unknown（2026-09-13 审计 §2.6：以前 `or 0.0` 把缺失当零位，
    而 home 是实机上电保持的姿态，缺一个关节的声明整个姿态就不存在）。全齐 → INFO PASS。"""
    home, bad = {}, []
    for j in ja:
        v = j.get("home_rad")
        if isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(float(v)):
            home[j["name"]] = float(v)
        else:
            bad.append(f"{j.get('name')}: home_rad={v!r}")
    if bad or not ja:
        res.unknown("_load", "home_pose_declared",
                    f"frozen.yaml:joint_axes 里 {len(bad)}/{len(ja)} 个关节没有可用的 home_rad（{bad[:6]}）—— "
                    "home 姿态不完整，static_torque_home / com_in_support_home 都判不了；**不拿 0 顶替**",
                    provenance="frozen.yaml:joint_axes[].home_rad")
        return None
    res.add(subject="_load", check="home_pose_declared", state=PASS, severity=INFO, measured=len(home),
            criterion="frozen.yaml:joint_axes 里每个 exists_as_mjcf_joint 的关节都声明了数值 home_rad（缺一个 = home 姿态不存在）",
            evidence_n=len(ja), detail=f"{len(home)} 个关节：{ {k: round(v, 4) for k, v in home.items()} }",
            provenance="frozen.yaml:joint_axes[].home_rad")
    return home


def _quat_R(q):
    """MuJoCo 四元数 (w x y z) → 3×3 旋转矩阵。"""
    import numpy as np
    w, x, y, z = (float(v) for v in q)
    n = math.sqrt(w * w + x * x + y * y + z * z)
    if n < 1e-12:
        return None
    w, x, y, z = w / n, x / n, y / n, z / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def _mjcf_inertial(res):
    """robot_walk.xml 的 <inertial mass= pos= diaginertia|fullinertia= quat=> —— duckstructure.kin
    不解析这些属性，这里自己读同一个文件。

    2026-09-09 起**必须连惯量张量一起读**（此前只读 mass/pos）：质量对上、质心对上、而料从中间挪到
    两端，惯量可以差一倍，逐 body 漂移却一声不吭。MuJoCo 两种写法都合法，都要支持：
      · fullinertia="Ixx Iyy Izz Ixy Ixz Iyz" —— 在 **body 系轴**下、关于 <inertial pos>（质心）的全张量；
      · diaginertia="I1 I2 I3" + quat（缺省 = 单位四元数）—— 在**惯量主轴系**下的三个主惯量，
        主轴系相对 body 系的姿态由 quat 给；本函数统一转成 body 系全张量 R·diag·Rᵀ。
    两个都没有的 body：MuJoCo 会自己从 <geom> 算 —— 那是另一套几何，**本层不猜**，标 unknown 判红。

    返回 {body: {"mass_kg", "com_mm", "I_gmm2"(3×3 或 None), "I_src", "I_reason"}}。
    单位换算：MJCF 是 kg·m²，本层内部统一 g·mm² → ×1e9（1000 g/kg × 1e6 mm²/m²）。

    顺手核一件事：本层把关节轴当成 body 系的 +z（和 duckstructure/checks.py:run_checks 一样用 TW[:3,2]），
    这个假设只有在 MJCF 每个 <joint axis> 都是 (0 0 1) 时才成立 —— 不成立就必须判红，不能默默算错扭矩。"""
    import numpy as np
    import xml.etree.ElementTree as ET
    try:
        from duckstructure import kin
        xml = Path(kin.MD) / "robot_walk.xml"
        root = ET.parse(xml).getroot()
    except Exception as e:                                      # noqa: BLE001
        res.unknown("_mjcf", "inertial_parse", f"读不到 robot_walk.xml 的 <inertial>：{e}",
                    provenance="upstream/microduck_rl/.../robot_walk.xml")
        return {}
    out = {}
    bad = []
    n_j = 0
    KGM2_TO_GMM2 = 1.0e9

    def tensor(i):
        """<inertial> 元素 → (body 系全张量 g·mm², 出处标签, 说不了话的原因)。"""
        fi = i.get("fullinertia")
        di = i.get("diaginertia")
        if fi is not None:
            v = [float(x) for x in fi.split()]
            if len(v) != 6:
                return None, None, f"fullinertia 应当是 6 个数，实际 {len(v)} 个：{fi!r}"
            ixx, iyy, izz, ixy, ixz, iyz = v
            M = np.array([[ixx, ixy, ixz], [ixy, iyy, iyz], [ixz, iyz, izz]]) * KGM2_TO_GMM2
            return M, "fullinertia", None
        if di is not None:
            v = [float(x) for x in di.split()]
            if len(v) != 3:
                return None, None, f"diaginertia 应当是 3 个数，实际 {len(v)} 个：{di!r}"
            q = (i.get("quat") or "1 0 0 0").split()
            if len(q) != 4:
                return None, None, f"quat 应当是 4 个数（w x y z），实际 {len(q)} 个：{i.get('quat')!r}"
            R = _quat_R(q)
            if R is None:
                return None, None, f"quat 模长为 0，转不出主轴姿态：{i.get('quat')!r}"
            D = np.diag([float(x) for x in v]) * KGM2_TO_GMM2
            return R @ D @ R.T, "diaginertia+quat", None
        return None, None, ("<inertial> 只给了 mass/pos，没有 diaginertia 也没有 fullinertia —— "
                            "MuJoCo 会改用 <geom> 自己算惯量，那是另一套几何，本层不猜")

    def walk(b):
        nonlocal n_j
        i = b.find("inertial")
        if i is not None:
            I, isrc, why = tensor(i)
            out[b.get("name")] = {
                "mass_kg": float(i.get("mass")),
                "com_mm": np.array([float(x) for x in i.get("pos").split()]) * 1000.0,
                "I_gmm2": I, "I_src": isrc, "I_reason": why,
            }
        for j in b.findall("joint"):
            n_j += 1
            ax = [float(x) for x in (j.get("axis") or "0 0 1").split()]
            if j.get("pos") is not None or ax != [0.0, 0.0, 1.0]:
                bad.append(f"{j.get('name')}: axis={j.get('axis')} pos={j.get('pos')}")
        for c in b.findall("body"):
            walk(c)

    walk(root.find("worldbody").find("body"))
    res.add(subject="_mjcf", check="joint_axis_assumption", state=PASS if not bad else FAIL,
            severity=BLOCK, measured=len(bad),
            criterion="MJCF 每个 <joint> 的 axis = (0 0 1) 且 pos 缺省（= body 原点）—— "
                      "本层的关节轴取 TW(body)[:3,2]、作用点取 TW(body)[:3,3]，靠的就是这个",
            evidence_n=n_j,
            detail=f"{n_j} 个 <joint>" + (f"，不满足的：{'; '.join(bad)}" if bad else "，全部满足"),
            provenance="upstream/microduck_rl/.../robot_walk.xml / duckstructure/checks.py:run_checks")
    srcs = sorted({str(v["I_src"]) for v in out.values()})
    n_I = sum(1 for v in out.values() if v["I_gmm2"] is not None)
    no_I = [b for b, v in out.items() if v["I_gmm2"] is None]
    # 2026-09-13（F-L7-2）：以前 `PASS if n_I`——只要任何一个 body 有张量整条就绿，criterion 却写"每个"。
    # 现在 n_I == len(out)，且 len(out) > 0（一个 body 都没解析到也不是通过）。
    res.add(subject="_mjcf", check="inertia_tensor_parsed",
            state=PASS if (out and n_I == len(out)) else FAIL,
            severity=BLOCK, measured=f"{n_I}/{len(out)}",
            criterion="MJCF **每个** <inertial> 必须给出惯量张量（fullinertia 或 diaginertia[+quat]），"
                      "n_I == body 数才算过 —— 只有 mass/pos 的 body 由 MuJoCo 从 <geom> 反算，本层不猜",
            evidence_n=len(out),
            detail=f"{len(out)} 个 body 的 <inertial>，惯量写法：{srcs}；"
                   f"单位换算 kg·m² × 1e9 = g·mm²"
                   + (f"；缺张量的 body（{len(no_I)}）：{no_I[:6]}" if no_I else ""),
            provenance="upstream/microduck_rl/.../robot_walk.xml:<inertial>")
    return out


def _principal(I):
    """对称张量 → (升序主惯量 3, 对应主轴列向量 3×3)。对角化就是"换到主轴系"，
    所以主惯量三元组是**坐标系无关**的量 —— MJCF 的 diaginertia 给的正是它。"""
    import numpy as np
    w, V = np.linalg.eigh((np.asarray(I, float) + np.asarray(I, float).T) / 2.0)
    o = np.argsort(w)
    return w[o], V[:, o]


def _axis_angles_deg(V1, V2):
    """两组主轴之间的逐轴夹角（度）。特征向量的正负号是任意的 → 取 |cos| 再取 arccos。"""
    import numpy as np
    return [math.degrees(math.acos(min(1.0, max(0.0, abs(float(np.dot(V1[:, k], V2[:, k])))))))
            for k in range(3)]


def _inertia_body(res, b, sel, com, loc_I, mj, tol_rel, tol_deg):
    """逐 body 惯量张量 vs MJCF <inertial>。

    为什么不能只比迹：迹相同而分布不同（把料从中间挪到两端）正是要抓的东西。所以两边都
    **对角化**成三个主惯量（坐标系无关），排序后逐个比；主轴朝向单独比（两组特征向量的最大夹角）。
    两边都取**关于各自质心**的中心惯量 —— 质心位置的差异已经由 body_com_vs_mjcf 单独判过，
    不能再混进分布这条里。多体合并用平行轴定理搬到公共质心，不是直接相加。"""
    import numpy as np
    lack = [i[0] for i in sel if loc_I.get((i[0], i[1]), (None, None))[0] is None]
    if lack:
        res.unknown(f"body:{b}", "cad_inertia",
                    f"这个 body 上有 {len(lack)} 项算不出惯量张量（体积或质量取不到）："
                    + ", ".join(lack[:6]) + " —— 缺一项，整个 body 的惯量就是错的，不报数",
                    provenance="cad/duck_s288/placed/ / components.yaml:mass_g")
        return
    I_cad = np.zeros((3, 3))
    I_pt = np.zeros((3, 3))       # 同一套，但把**元件**退化成质点（打印件仍按实体）
    tags, n_env = set(), 0
    for nm, bb, mg, cl, _k, _s in sel:
        Ii, tag = loc_I[(nm, bb)]
        tags.add(str(tag))
        d = np.asarray(cl, float) - np.asarray(com, float)
        shift = mg * (float(d @ d) * np.eye(3) - np.outer(d, d))   # 平行轴定理
        I_cad += Ii + shift
        if tag == "envelope_uniform":
            n_env += 1
            I_pt += shift
        else:
            I_pt += Ii + shift
    I_mj = mj.get("I_gmm2")
    if I_mj is None:
        res.unknown(f"body:{b}", "inertia_vs_mjcf",
                    f"MJCF 侧拿不到惯量张量：{mj.get('I_reason')}。CAD 侧已经算出主惯量 "
                    f"{np.round(_principal(I_cad)[0], 1).tolist()} g·mm²，但没有可比的对象",
                    provenance="robot_walk.xml:<inertial> / core.py 元规则 4")
        return
    wc, Vc = _principal(I_cad)
    wm, Vm = _principal(I_mj)
    wp, _Vp = _principal(I_pt)
    rel = [float((wc[k] - wm[k]) / wm[k]) if wm[k] > 0 else float("nan") for k in range(3)]
    ang = _axis_angles_deg(Vc, Vm)
    max_rel = max(abs(r) for r in rel)
    max_ang = max(ang)
    # 主惯量接近简并时主轴本身就不唯一 —— 夹角大不代表分布错，必须把简并度一起报出来
    gap = min((wm[1] - wm[0]) / wm[2], (wm[2] - wm[1]) / wm[2]) if wm[2] > 0 else float("nan")
    # 元件按 placed 包络实体+均匀密度建模；包络里的真实分布（S288 转子、电池卷芯、PCB 元器件）
    # 没有数据 → 把**元件**退化成质点（打印件仍按实体）作对照，两个模型的主惯量差就是本判据
    # 自身的建模不确定度。这个 body 上没有元件时它恒为 0，detail 里会写明。
    model_spread = max(abs(float((wp[k] - wc[k]) / wc[k])) if wc[k] > 0 else float("nan")
                       for k in range(3))
    res.add(subject=f"body:{b}", check="principal_inertia_vs_mjcf", state=PASS, severity=INFO,
            measured=round(max_rel, 4),
            criterion="CAD 主惯量三元组（对角化，升序）vs MJCF diaginertia/fullinertia 的主惯量，逐个比相对差"
                      "（阈值缺，见 L07/_mjcf.inertia_drift_threshold）",
            evidence_n=len(sel) * 3,
            detail=f"CAD {np.round(wc, 1).tolist()} vs MJCF {np.round(wm, 1).tolist()} g·mm²"
                   f"（MJCF 写法 {mj.get('I_src')}）→ 相对差 "
                   f"{[round(r*100, 1) for r in rel]} %，最大 {max_rel*100:.1f}%；"
                   f"迹 CAD {np.trace(I_cad):.1f} vs MJCF {np.trace(I_mj):.1f} g·mm²；"
                   f"计入 {len(sel)} 项（其中元件 {n_env} 项），惯量模型 {sorted(tags)}；"
                   + (f"把这 {n_env} 个元件退化成**质点**（包络内分布未知的敏感性）主惯量会变 "
                      f"{model_spread*100:.1f}% —— 这是本条判据自身的建模不确定度"
                      if n_env else "这个 body 上没有元件，惯量全部来自打印件实体，无包络建模不确定度"),
            provenance="robot_walk.xml:<inertial fullinertia|diaginertia> / "
                       "trimesh.moment_inertia × tolerances.yaml:load.density_g_cm3")
    res.add(subject=f"body:{b}", check="principal_axis_vs_mjcf", state=PASS, severity=INFO,
            measured=round(max_ang, 3),
            criterion="CAD 主轴与 MJCF 主轴的逐轴夹角（两侧都按主惯量升序配对；特征向量符号任意 → 取 |cos|），"
                      "取最大（阈值缺，见 L07/_mjcf.inertia_drift_threshold）",
            evidence_n=3,
            detail=f"三轴夹角 {[round(a, 3) for a in ang]}°，最大 {max_ang:.3f}°；"
                   f"MJCF 主惯量的相对间隔 min(I2−I1, I3−I2)/I3 = {gap*100:.1f}%"
                   + ("（**接近简并：这个平面内的主轴本身不唯一，而且按大小配对很可能配错，"
                      "这个角度不可解释**）" if gap < 0.05 else
                      "（间隔够大，按大小配对是可靠的）")
                   + "。本条只用来看『分布的朝向变了没有』，**判定要连着 principal_inertia_vs_mjcf 一起看**："
                     "主惯量都对上、只有朝向转了，才是真的把料挪了位置",
            provenance="robot_walk.xml:<inertial> / numpy.linalg.eigh")
    if tol_rel is None or tol_deg is None:
        res.unknown(f"body:{b}", "inertia_criteria",
                    f"这个 body 的主惯量最大相对漂移 {max_rel*100:.1f}%、主轴最大夹角 {max_ang:.3f}° "
                    "已经量出来了，但 tolerances.yaml:load.inertia_drift_vs_mjcf 里缺 "
                    f"{'principal_moment_rel' if tol_rel is None else ''}"
                    f"{'/' if tol_rel is None and tol_deg is None else ''}"
                    f"{'principal_axis_deg' if tol_deg is None else ''} —— 判不了。"
                    "填一个凭空的 ±10% 会让 15 个格子一起变绿，那正是本体系要抓的事",
                    provenance="tolerances.yaml:load.inertia_drift_vs_mjcf")
    else:
        ok = max_rel <= float(tol_rel) and max_ang <= float(tol_deg)
        res.add(subject=f"body:{b}", check="inertia_criteria", state=PASS if ok else FAIL,
                severity=BLOCK, measured=[round(max_rel, 4), round(max_ang, 3)],
                criterion=f"主惯量逐个相对漂移 ≤ {tol_rel} 且 主轴最大夹角 ≤ {tol_deg}°",
                evidence_n=len(sel) * 3 + 3,
                detail=f"主惯量相对差 {[round(r*100, 1) for r in rel]} %；主轴夹角 "
                       f"{[round(a, 3) for a in ang]}°",
                provenance="tolerances.yaml:load.inertia_drift_vs_mjcf")


def _stability(ctx, res, B, order, jb_of_body, loc_items, placed, assign, home, gaps):
    """整机重心投影 vs 双脚 / 单脚支撑多边形。支撑面 = parts.yaml 里声明 `ground_contact: true` 的件最低点附近的接触带。

    2026-09-13（审计 §2.6）：以前拿 `"TPU" in material` 猜落地件 —— 换个硬底鞋或给别的件用 TPU 就错。
    现在只认 parts.yaml 的显式字段；没有任何件声明 → unknown。
    F-L7-4：质量清单有缺口时 com_in_support_* 一律 unknown（重心是整机量，缺头部件时算出来的是下界）。"""
    import numpy as np
    try:
        from scipy.spatial import ConvexHull
    except Exception as e:                                      # noqa: BLE001
        res.unknown("_stability", "convex_hull", f"算不了支撑多边形：{e}")
        return
    parts_by_id = {p["id"]: p for p in ((ctx.data.get("parts") or {}).get("parts") or [])}
    decl = sorted(pid for pid, p in parts_by_id.items() if p.get("ground_contact") is True)
    if not decl:
        res.unknown("_stability", "support_polygon",
                    "parts.yaml 里没有任何件声明 `ground_contact: true` —— 落地件（支撑面）只认这个显式字段，"
                    "层不再拿 'TPU' in material 去猜（2026-09-13 审计 §2.6）；没有落地件就没有支撑多边形",
                    provenance="parts.yaml:parts[].ground_contact")
        return
    soles = []
    for pid in decl:
        for s, b in (assign.get(pid) or []):
            Ti = np.linalg.inv(B[b]["T_world"])
            v = np.asarray(_mesh(placed[s]).vertices, float)
            soles.append((s, b, (Ti[:3, :3] @ v.T).T + Ti[:3, 3]))
    not_assigned = [pid for pid in decl if pid not in assign]
    if len(soles) < 2 or not_assigned:
        res.unknown("_stability", "support_polygon",
                    f"parts.yaml 声明 ground_contact 的件 {decl} 在本层件→body 归属里只找到 {len(soles)} 个实例"
                    + (f"（{not_assigned} 没有归属，见同件的 body_assignment）" if not_assigned else "")
                    + " —— 凑不出双脚支撑多边形",
                    provenance="parts.yaml:parts[].ground_contact")
        return
    stab_node = ((ctx.data.get("tolerances") or {}).get("load") or {}).get("static_stability_margin_mm")
    stab_min, stab_src = num(stab_node)
    stab_key = "tolerances.yaml:load.static_stability_margin_mm"
    if stab_min is None:
        res.unknown("_stability", "margin_threshold",
                    f"{stab_key} 取不到 —— "
                    "下面只按几何充要条件（余量 > 0 = 在多边形内）判，余量该留多少没人定过。"
                    "填上这个数本层立刻改用它当门槛",
                    provenance="README 第 2 节 L7 / " + stab_key)
        need, need_txt = 0.0, "0 mm（没有余量阈值，见同格 margin_threshold）"
    elif stab_src in (None, "unknown", ""):
        res.unknown("_stability", "margin_threshold",
                    f"{stab_key} = {stab_min} 没有 src —— 阈值没有来源，不能拿它判", provenance=stab_key + ".src")
        need, need_txt = None, ""
    else:
        need, need_txt = float(stab_min), f"{stab_min} mm（{stab_key}，src={stab_src!r}）"
        if stab_src == "assumed":
            # 2026-09-13（审计 §2.6）：3.0 是 assumed 却从不产生任何 finding。照判，留一条 WARN，值就是阈值。
            res.add(subject="_stability", check="threshold_assumed:static_stability_margin_mm", state=FAIL,
                    severity=WARN, measured=float(stab_min),
                    criterion=f"{stab_key} 的 src 应是 measured / datasheet；src=assumed 照判但不算有来源",
                    evidence_n=1,
                    detail=f"{stab_min} mm，src=assumed（{(stab_node.get('src_note') if isinstance(stab_node, dict) else '') or ''}）；"
                           "推翻条件见该键 what_would_overturn_it",
                    provenance=stab_key)
    gap_txt = _gap_text(gaps)

    def analyse(tag, q, sev, why=""):
        if q is None:
            res.unknown("_stability", f"com_in_support_{tag}",
                        f"{tag} 姿态不存在：frozen.yaml:joint_axes 的 home_rad 声明不全（见 L07/_load.home_pose_declared），"
                        "不拿 0 顶替", provenance="frozen.yaml:joint_axes[].home_rad", severity=sev)
            return
        T = _fk(B, order, jb_of_body, q)
        tot = sum(i[2] for i in loc_items)
        com = sum(i[2] * (T[i[1]][:3, :3] @ np.asarray(i[3]) + T[i[1]][:3, 3]) for i in loc_items) / tot
        pts = {s: (T[b][:3, :3] @ v.T).T + T[b][:3, 3] for s, b, v in soles}
        allw = np.vstack(list(pts.values()))
        zmin = float(allw[:, 2].min())
        band = allw[allw[:, 2] < zmin + _CONTACT_BAND_MM][:, :2]
        try:
            hull = band[ConvexHull(band).vertices]
        except Exception as e:                                  # noqa: BLE001  接触带退化成共线/点
            res.unknown("_stability", f"com_in_support_{tag}",
                        f"{tag} 姿态下 {len(band)} 个接触点凑不出凸包（{e}）—— 支撑多边形不存在",
                        provenance="README 第 2 节 L7", severity=sev)
            return
        m2 = _margin(hull, com[:2])
        feet_apart = max(abs(float(p[:, 2].min()) - zmin) for p in pts.values())
        geo = (f"{tag} 姿态：重心 {np.round(com,2).tolist()} mm，总质量 {tot:.1f} g；"
               f"接触带 z < {zmin:.2f}+{_CONTACT_BAND_MM} 取到 {len(band)} 个点、凸包 {len(hull)} 顶点，"
               f"包络 x[{hull[:,0].min():.1f},{hull[:,0].max():.1f}] "
               f"y[{hull[:,1].min():.1f},{hull[:,1].max():.1f}]；两脚最低点高差 {feet_apart:.2f} mm；"
               f"余量 {'算不出' if m2 is None else f'{m2:.3f} mm'}")
        if need is None:
            res.unknown("_stability", f"com_in_support_{tag}",
                        f"余量阈值没有来源（见同格 margin_threshold），不判。{geo}", provenance=stab_key, severity=sev)
        elif gaps:
            # F-L7-4：整机重心是整机量，清单不全时算出来的只是下界 —— 不给下界当绿
            res.unknown("_stability", f"com_in_support_{tag}",
                        f"质量清单不全 → unknown，不给下界当绿。仅含已知质量的**下界**：{geo}（阈值 {need_txt}）。"
                        f"{gap_txt}。" + why,
                        provenance="components.yaml:components[] / frozen.yaml:joint_axes.home_rad", severity=sev)
        else:
            res.add(subject="_stability", check=f"com_in_support_{tag}",
                    state=PASS if (m2 is not None and m2 > need) else FAIL, severity=sev,
                    measured=None if m2 is None else round(m2, 3),
                    criterion=f"整机重心水平投影落在双脚接触点凸包内，且余量 > {need_txt}；"
                              "落地件 = parts.yaml:ground_contact=true 的件；质量清单不全 → unknown（不给下界当绿）",
                    evidence_n=len(band),
                    detail=geo + "。" + why,
                    provenance="README 第 2 节 L7 / parts.yaml:parts[].ground_contact / "
                               "frozen.yaml:joint_axes.home_rad / " + stab_key)
        for s, b, _v in soles:
            w = pts[s]
            zz = float(w[:, 2].min())
            bb = w[w[:, 2] < zz + _CONTACT_BAND_MM][:, :2]
            if len(bb) < 3:
                continue
            h1 = bb[ConvexHull(bb).vertices]
            m1 = _margin(h1, com[:2])
            res.add(subject="_stability", check=f"com_in_support_{tag}_single_{s}",
                    state=PASS, severity=INFO, measured=None if m1 is None else round(m1, 3),
                    criterion="单脚支撑余量（没有判据，只报数" + ("；质量清单不全，是下界" if gaps else "") + "）",
                    evidence_n=len(bb),
                    detail=(f"{tag} 姿态下只站 {s}：余量 {m1:.3f} mm" if m1 is not None
                            else f"{tag} 姿态下只站 {s}：接触带退化成共线/点，算不出余量")
                           + ("（负值 = 重心在这只脚的支撑面之外，要靠横移/横滚把重心搬过去）"
                              if m1 is not None and m1 < 0 else ""),
                    provenance="README 第 2 节 L7")

    # 零位姿是建模基准而不是站姿（腿伸直、脚面朝向与站立时不同），支撑多边形在那儿只有几何意义 —— 判 WARN；
    # frozen.yaml:joint_axes.home_rad 那个姿态才是实机要站住的，判 BLOCK。
    analyse("zero", {}, WARN,
            "零位姿是建模基准不是站姿（14 关节全 0、腿伸直），支撑多边形在这里只有几何意义 → 判 WARN；"
            "真正要站住的是下面的 home 姿态（frozen.yaml:joint_axes.home_rad）")
    analyse("home", None if home is None else dict(home), BLOCK,
            "home = frozen.yaml:joint_axes.home_rad，实机上电保持的就是这个姿态 → 判 BLOCK")


def _head_budget(res, B, order, jb_of_body, loc_items, ja, rated, gaps):
    """由颈部/头部关节的额定扭矩倒推：还能往头里加多少克。
    附加质量放在头（最远端 body）的现有质心处，随关节一起转。
    F-L7-4：质量清单有缺口时这是**上界**（漏掉的头部件会先吃掉预算）→ unknown，数只进 detail。"""
    import numpy as np
    # 头部末端 body = frozen.yaml:joint_axes 里 head_*/neck_* 那几个关节的 body 中最深的一个
    # （它是其余几个的后代）。不靠人工表，也不拿"最后一个叶子 body"瞎猜（那会摸到脚上去）。
    cands = [j["mjcf_body"] for j in ja
             if re.match(r"^(head|neck)_", j.get("name") or "") and j.get("mjcf_body") in B]
    leaf = None
    for b in cands:
        if all(b in set(_descend(B, o)) for o in cands):
            leaf = b
    if leaf is None or not any(i[1] == leaf for i in loc_items):
        res.unknown("_load", "head_mass_budget",
                    f"找不到头部末端 body（head_*/neck_* 关节的 body 候选 {cands}）",
                    provenance="frozen.yaml:joint_axes")
        return
    sel = [i for i in loc_items if i[1] == leaf]
    hm = sum(i[2] for i in sel)
    hcl = sum(i[2] * np.asarray(i[3]) for i in sel) / hm

    def tau(jn, body, ang_deg, extra_g):
        q = {jn: math.radians(ang_deg)}
        T = _fk(B, order, jb_of_body, q)
        Tb = T[body]
        n = Tb[:3, 2] / np.linalg.norm(Tb[:3, 2])
        p = Tb[:3, 3] / 1000.0
        sub = set(_descend(B, body))
        t = np.zeros(3)
        for nm, b, mg, cl, kind, src in loc_items:
            if b not in sub:
                continue
            r = (T[b][:3, :3] @ cl + T[b][:3, 3]) / 1000.0
            t += np.cross(r - p, np.array([0.0, 0.0, -mg / 1000.0 * _G]))
        if extra_g and leaf in sub:
            r = (T[leaf][:3, :3] @ hcl + T[leaf][:3, 3]) / 1000.0
            t += np.cross(r - p, np.array([0.0, 0.0, -extra_g / 1000.0 * _G]))
        return abs(float(n @ t))

    from duckstructure.checks import check_angles
    rows = []
    bad_rng = []
    for j in ja:
        jn, body = j["name"], j["mjcf_body"]
        if body not in B or leaf not in set(_descend(B, body)):
            continue                                            # 不承担头部载荷的关节跳过
        try:
            lo, hi = (float(x) for x in j["range_deg"])
        except (TypeError, ValueError, KeyError):
            lo = hi = None
        angles = check_angles(lo, hi) if (lo is not None and hi is not None and lo < hi) else []
        if len(angles) < 3:                                     # 区间反写/退化：同 static_torque，不拿两端点当扫过
            bad_rng.append(f"{jn}: range_deg={j.get('range_deg')!r}")
            continue

        def worst(extra):
            return max(tau(jn, body, a, extra) for a in angles)

        cur = worst(0.0)
        if cur >= rated:
            rows.append((jn, 0.0, cur))
            continue
        cap = _BUDGET_CAP_G
        if worst(cap) < rated:
            rows.append((jn, float("inf"), cur))                 # 轴与重力平行 → 静扭矩恒为 0，不限重
            continue
        lo_m, hi_m = 0.0, 1.0
        while worst(hi_m) < rated:
            hi_m *= 2.0
        for _ in range(50):
            mid = (lo_m + hi_m) / 2.0
            if worst(mid) < rated:
                lo_m = mid
            else:
                hi_m = mid
        rows.append((jn, lo_m, cur))
    if bad_rng:
        res.unknown("_load", "head_mass_budget",
                    f"承担头部载荷的关节里有区间声明无效的：{bad_rng} —— 少扫一个关节预算就不成立",
                    provenance="frozen.yaml:joint_axes[].range_deg")
        return
    if not rows:
        res.unknown("_load", "head_mass_budget", "没有找到承担头部载荷的关节")
        return
    rows.sort(key=lambda r: r[1])
    jn, budget, cur = rows[0]
    if not math.isfinite(budget):
        res.unknown("_load", "head_mass_budget",
                    "所有承载关节的轴都与重力平行，静扭矩恒为 0 —— 这个预算算不出来（数据不对）")
        return
    if gaps:
        res.unknown("_load", "head_mass_budget",
                    f"质量清单不全 → unknown，不给上界当数。仅含已知质量的**上界**：{budget:.1f} g"
                    f"（卡脖子 {jn}，空载全程峰值 {cur:.4f} N·m，按 margin=1）；漏掉的头部件会先吃掉这个预算。{_gap_text(gaps)}",
                    provenance="components.yaml:components[] / frozen.yaml:joint_axes.range_deg")
        return
    res.add(subject="_load", check="head_mass_budget", state=PASS, severity=INFO,
            measured=round(budget, 1),
            criterion=f"在头部现有质心处还能加多少克，使**所有**承载关节全程静扭矩 < 额定 {rated} N·m × 系数；"
                      f"系数是 null，这里按 1 算（上界，见 L07/_load.static_torque_margin）",
            evidence_n=len(rows),
            detail=f"卡脖子的是 {jn}（该关节空载全程峰值 {cur:.4f} N·m）；预算 {budget:.1f} g。"
                   f"附加质量假设放在头部 body «{leaf}» 现有质心 {np.round(hcl,1).tolist()} mm（本体系）、"
                   f"当前头部 {hm:.1f} g。各关节预算：" +
                   "；".join(f"{n}=" + ("不限（轴与重力平行）" if not math.isfinite(b) else f"{b:.0f} g")
                             for n, b, _ in rows) +
                   "。**0.6 N·m 是名义堵转不是可持续输出**，真按连续扭矩选系数，这个预算要按比例砍；"
                   "摄像头/麦克风/屏/基准板还都没选型，这个数就是它们的总预算",
            provenance="docs S288 名义堵转 / frozen.yaml:joint_axes.range_deg / README 第 2 节 L7")


def _unit(v, name):
    """列表 → 单位向量。不是 3 个数 / 模长为 0 → (None, 原因)。"""
    import numpy as np
    if not (isinstance(v, (list, tuple)) and len(v) == 3):
        return None, f"{name} 不是 3 个数：{v!r}"
    try:
        a = np.array([float(x) for x in v], float)
    except (TypeError, ValueError):
        return None, f"{name} 里有非数值：{v!r}"
    n = float(np.linalg.norm(a))
    if n < 1e-9:
        return None, f"{name} 模长为 0"
    return a / n, (None if abs(n - 1.0) <= 1e-3 else
                   f"{name} 模长 {n:.6f} ≠ 1（已归一化后使用）")


def _pt3(d, key, name):
    import numpy as np
    v = (d or {}).get(key)
    if not (isinstance(v, (list, tuple)) and len(v) == 3):
        return None, f"{name} 不是 3 个数：{v!r}"
    try:
        return np.array([float(x) for x in v], float), None
    except (TypeError, ValueError):
        return None, f"{name} 里有非数值：{v!r}"


def _first_solid_span(mesh, o, d):
    """从点 o 沿单位向量 d 打一条射线，返回**第一段实体**的长度（第 1、2 个正向交点之差）。
    交点少于 2 个 → None。o 在孔里、也在实体外都适用：量的是"沿这个方向第一块料有多厚"。"""
    import numpy as np
    loc, _i, _t = mesh.ray.intersects_location([np.asarray(o, float)], [np.asarray(d, float)])
    if len(loc) < 2:
        return None
    t = np.sort((loc - np.asarray(o, float)) @ np.asarray(d, float))
    t = t[t > 1e-9]
    if len(t) < 2:
        return None
    t = t[np.r_[True, np.diff(t) > 1e-7]]
    return float(t[1] - t[0]) if len(t) >= 2 else None


def _ribs(ctx, res, assign):
    """承力薄筋：读 features.yaml 里 kind == 'rib' 的条目，在**导出 STL（export_local）**上现场量
    长/厚/长厚比，与声明值比（±0.1），再拿 tolerances 的上限判。

    这一段以前**根本不存在** —— l7_mass.py:_sections 里只有一句 `if rib is None: res.unknown(...)`，
    阈值一旦填上那条 unknown 就消失、而没有任何检查接上去（见
    tolerances.yaml:load.min_section_and_rib.max_rib_slenderness.refused_2026-09-09）。
    量法本身不含任何工程阈值：厚度是"沿 thickness_axis 的第一段实体"，长度是"厚度还等于最小值的
    那一段连续区间"，两者都只用到存在性与数值相等。"""
    import numpy as np
    msr = ((ctx.data.get("tolerances") or {}).get("load") or {}).get("min_section_and_rib") or {}
    rib_max, rib_src = num(msr.get("max_rib_slenderness"))
    # 2026-09-13（审计 §2.6）：实测 vs 声明的 ±0.1 以前写死在层里 → 进 tolerances.yaml，带 src。
    tol_key = "tolerances.yaml:load.min_section_and_rib.rib_declared_vs_measured_tol_mm"
    tol_node = msr.get("rib_declared_vs_measured_tol_mm")
    rib_tol, rib_tol_src = num(tol_node)
    if rib_tol is None:
        res.unknown("_section", "rib_declared_tol",
                    f"{tol_key} 取不到 —— 实测筋厚/筋长与 features.yaml 声明值之间允许差多少没有数，"
                    "下面每条 rib_thickness / rib_length 都判不了", provenance=tol_key)
        rib_tol = None
    elif rib_tol_src in (None, "unknown", ""):
        res.unknown("_section", "rib_declared_tol", f"{tol_key} = {rib_tol} 没有 src —— 阈值没有来源，不能拿它判",
                    provenance=tol_key + ".src")
        rib_tol = None
    elif rib_tol_src == "assumed":
        res.add(subject="_section", check="threshold_assumed:rib_declared_vs_measured_tol_mm", state=FAIL,
                severity=WARN, measured=float(rib_tol),
                criterion=f"{tol_key} 的 src 应是 measured / datasheet；src=assumed 照判但不算有来源",
                evidence_n=1,
                detail=f"±{rib_tol} mm，src=assumed（{(tol_node.get('src_note') if isinstance(tol_node, dict) else '') or ''}）",
                provenance=tol_key)
    tol_txt = f"± {rib_tol} mm（{tol_key}，src={rib_tol_src!r}）"
    feats = (ctx.data.get("features") or {}).get("features") or []
    ribs = [f for f in feats if isinstance(f, dict) and f.get("kind") == "rib"]
    parts_with = sorted({f.get("part") for f in ribs})
    # 清单本身就是缺口：这两条来自一句散文，17 个件没有做过系统的薄壁提取
    res.unknown("_section", "rib_inventory",
                f"features.yaml 里只有 {len(ribs)} 条 kind=rib 的承力薄筋（{', '.join(f['id'] for f in ribs)}，"
                f"落在 {parts_with}），出处是打印清单里的**一句散文**"
                "（『H02 沉孔窄筋射线厚度 1.434mm…新增 L07 后鞋底最薄约 1.763mm』）。"
                f"其余 {len(assign) - len(parts_with)} 个件从来没有做过系统的薄壁/薄筋提取 —— "
                "所以『没有更多筋』这件事没有证据。要清这条红：对 17 件跑一次几何薄壁扫描"
                "（例如逐件最大内接球半径场），把所有低于打印壁厚阈值的连通区都建成 kind=rib 条目。",
                provenance="features.yaml:features[kind=rib] / "
                           "V2_S288版发布产物/01_整鸭打印件/打印清单与参数.md:61")
    for f in ribs:
        fid, pid = f.get("id"), f.get("part")
        if pid not in assign:
            res.unknown("_section", f"{fid}:part",
                        f"features.yaml 的 {fid} 挂在件 {pid} 上，但本层的件→body 归属里没有它",
                        provenance=f"features.yaml:{fid}.part")
            continue
        if ctx.only and pid not in ctx.only:
            continue
        gp = ((f.get("geom") or {}).get("rib_probe")) or {}
        prov = f"features.yaml:{fid}.geom.rib_probe"
        if gp.get("frame") != "export_local":
            res.unknown(pid, f"{fid}:rib_probe",
                        f"rib_probe.frame='{gp.get('frame')}'，本层只在 export_local（导出 STL 自己的坐标系）"
                        "里量 —— 换算规则在 features.yaml:frames，不在层里",
                        provenance=prov)
            continue
        stl = ctx.stl(pid)
        if stl is None:
            res.unknown(pid, f"{fid}:rib_probe", f"找不到 {pid} 的导出 STL", provenance=prov)
            continue
        mesh = _mesh(stl)
        tax, tnote = _unit(gp.get("thickness_axis"), "thickness_axis")
        if tax is None:
            res.unknown(pid, f"{fid}:rib_probe", f"厚度方向取不出来：{tnote}", provenance=prov)
            continue
        mode = gp.get("mode")
        d_th, _ = num(f.get("thickness_mm"))
        d_len, _ = num(f.get("length_mm"))
        th = ln = None
        ev = 0
        how = ""
        if mode == "run_scan":
            o0, e = _pt3(gp, "origin_mm", "origin_mm")
            rax, rnote = _unit(gp.get("run_axis"), "run_axis")
            span = gp.get("run_span_mm")
            step, _ = num(gp.get("grid_step_mm"))
            etol, _ = num(gp.get("thickness_equal_tol_mm"))
            bad = ([e] if e else []) + ([rnote] if rax is None else []) + \
                  ([] if isinstance(span, (list, tuple)) and len(span) == 2 else ["run_span_mm 不是 [a, b]"]) + \
                  ([] if step and step > 0 else ["grid_step_mm 取不到"]) + \
                  ([] if etol is not None and etol >= 0 else ["thickness_equal_tol_mm 取不到"])
            if bad:
                res.unknown(pid, f"{fid}:rib_probe", "run_scan 探针字段不全：" + "；".join(bad),
                            provenance=prov)
                continue
            ts = np.arange(float(span[0]), float(span[1]) + 1e-9, float(step))
            hit = {}
            for t in ts:
                s = _first_solid_span(mesh, o0 + rax * float(t), tax)
                if s is not None:
                    hit[float(t)] = s
            ev = len(ts)
            if not hit:
                res.unknown(pid, f"{fid}:rib_probe",
                            f"沿 run_axis 在声明的 {list(span)} 区间里 {len(ts)} 个采样点一条料都没打到 —— "
                            "探针位置或方向不对（空刀），不是『筋很薄』",
                            provenance=prov)
                continue
            th = min(hit.values())
            keys = sorted(k for k, v in hit.items() if v <= th + float(etol))
            runs, cur = [], [keys[0]]
            for a, b in zip(keys, keys[1:]):
                if b - a <= float(step) * 1.5:
                    cur.append(b)
                else:
                    runs.append(cur)
                    cur = [b]
            runs.append(cur)
            sel = [r for r in runs if r[0] - 1e-9 <= 0.0 <= r[-1] + 1e-9] or [max(runs, key=len)]
            ln = float(sel[0][-1] - sel[0][0])
            how = (f"run_scan：origin {np.round(o0,4).tolist()}、厚度轴 {np.round(tax,6).tolist()}、"
                   f"走向轴 {np.round(rax,6).tolist()}、区间 {list(span)}、步距 {step} mm；"
                   f"{len(ts)} 个采样点里 {len(hit)} 个打到料；最薄 {th:.6f} mm，"
                   f"厚度仍等于最小值（差 ≤ {etol}）的连续段跨度 {ln:.4f} mm")
        elif mode == "region_scan":
            bb = gp.get("region_bbox_mm")
            step, _ = num(gp.get("grid_step_mm"))
            if not (isinstance(bb, (list, tuple)) and len(bb) == 2 and step and step > 0):
                res.unknown(pid, f"{fid}:rib_probe",
                            "region_scan 探针要 region_bbox_mm=[[x0,y0,z0],[x1,y1,z1]] 与 grid_step_mm",
                            provenance=prov)
                continue
            p0 = np.array([float(x) for x in bb[0]], float)
            p1 = np.array([float(x) for x in bb[1]], float)
            ax = int(np.argmax(np.abs(tax)))                    # 射线方向所在的坐标轴
            oth = [k for k in range(3) if k != ax]
            grids = []
            for k in oth:
                g = np.arange(p0[k] + float(step) / 2.0, p1[k], float(step))
                grids.append(np.r_[p0[k] + 1e-3, g, p1[k] - 1e-3])
            pts = []
            for a in grids[0]:
                for b in grids[1]:
                    p = np.zeros(3)
                    p[oth[0]], p[oth[1]] = a, b
                    p[ax] = (p0[ax] if tax[ax] > 0 else p1[ax]) - np.sign(tax[ax]) * 5.0
                    pts.append(p)
            pts = np.array(pts)
            loc, idx, _t = mesh.ray.intersects_location(pts, np.tile(tax, (len(pts), 1)))
            ev = len(pts)
            best, where, span = None, None, None
            for i in np.unique(idx):
                t = np.sort((loc[idx == i] - pts[i]) @ tax)
                t = t[np.r_[True, np.diff(t) > 1e-5]]
                if len(t) % 2:
                    continue
                tot = float(np.sum(t[1::2] - t[::2]))
                if best is None or tot < best:
                    q = pts[i].copy()
                    q[ax] = float("nan")                        # 射线起点在域外，这一维没有意义
                    best, where = tot, q
                    span = (pts[i] + tax * float(t[0]), pts[i] + tax * float(t[-1]))
            if best is None:
                res.unknown(pid, f"{fid}:rib_probe",
                            f"region_scan 的 {len(pts)} 条射线里没有一条打出成对交点（空刀）",
                            provenance=prov)
                continue
            th = best
            how = (f"region_scan：域 {bb}、厚度轴 {np.round(tax,6).tolist()}、步距 {step} mm、"
                   f"{len(pts)} 条射线；最小实体总厚 {th:.7f} mm，出现在域内 "
                   f"{[('—' if math.isnan(c) else round(float(c), 4)) for c in where]}"
                   f"（沿厚度轴那一维记 —，那是射线起点不是件上的点）；"
                   f"该处实体段 {np.round(span[0],4).tolist()} → {np.round(span[1],4).tolist()}")
        else:
            res.unknown(pid, f"{fid}:rib_probe",
                        f"rib_probe.mode='{mode}' 不认识（只支持 run_scan / region_scan）",
                        provenance=prov)
            continue
        # ── 实测 vs 声明（容差来自 tolerances.yaml）：data 和几何脱节时必须看得见
        if d_th is None:
            res.unknown(pid, f"{fid}:rib_thickness",
                        f"实测厚度 {th:.6f} mm（{how}），但 features.yaml 的 thickness_mm 没有可用数值 —— "
                        "对不上就没有『对不对得上』这件事",
                        provenance=f"features.yaml:{fid}.thickness_mm")
        elif rib_tol is None:
            res.unknown(pid, f"{fid}:rib_thickness",
                        f"实测厚度 {th:.6f} mm vs 声明 {d_th} mm，差 {th - float(d_th):+.6f} mm —— 但 {tol_key} "
                        "缺或没有来源（见 L07/_section.rib_declared_tol），判不了",
                        provenance=f"{prov} / {tol_key}")
        else:
            res.add(subject=pid, check=f"{fid}:rib_thickness",
                    state=PASS if abs(th - float(d_th)) <= float(rib_tol) else FAIL, severity=BLOCK,
                    measured=round(th, 6),
                    criterion=f"实测筋厚 = features.yaml:{fid}.thickness_mm ({d_th}) {tol_txt}",
                    evidence_n=ev,
                    detail=f"实测 {th:.6f} mm vs 声明 {d_th} mm，差 {th - float(d_th):+.6f} mm；{how}",
                    provenance=f"{prov} / features.yaml:{fid}.thickness_mm / {tol_key}")
        if ln is None:
            why = ((f.get("length_mm") or {}).get("unknown_reason")
                   if isinstance(f.get("length_mm"), dict) else None)
            res.unknown(pid, f"{fid}:rib_length",
                        f"这条筋量不出『长』：探针 mode={mode} 只给厚度。"
                        + (f"features.yaml 自己的理由：{why}" if why else
                           "features.yaml 里也没写为什么量不出来"),
                        provenance=prov)
            res.unknown(pid, f"{fid}:rib_slenderness",
                        f"长厚比 = 长 ÷ 厚，厚已实测 {th:.6f} mm 但长是 unknown —— 比值不存在。"
                        "（就算长有了，tolerances.yaml:load.min_section_and_rib.max_rib_slenderness "
                        f"现在是 {rib_max!r}，仍然没有可判的上限）",
                        provenance=prov + " / tolerances.yaml:load.min_section_and_rib.max_rib_slenderness")
            continue
        if d_len is None:
            res.unknown(pid, f"{fid}:rib_length",
                        f"实测筋长 {ln:.4f} mm，但 features.yaml 的 length_mm 没有可用数值",
                        provenance=f"features.yaml:{fid}.length_mm")
        elif rib_tol is None:
            res.unknown(pid, f"{fid}:rib_length",
                        f"实测筋长 {ln:.4f} mm vs 声明 {d_len} mm，差 {ln - float(d_len):+.4f} mm —— 但 {tol_key} "
                        "缺或没有来源（见 L07/_section.rib_declared_tol），判不了",
                        provenance=f"{prov} / {tol_key}")
        else:
            res.add(subject=pid, check=f"{fid}:rib_length",
                    state=PASS if abs(ln - float(d_len)) <= float(rib_tol) else FAIL, severity=BLOCK,
                    measured=round(ln, 4),
                    criterion=f"实测筋长 = features.yaml:{fid}.length_mm ({d_len}) {tol_txt}",
                    evidence_n=ev,
                    detail=f"实测 {ln:.4f} mm vs 声明 {d_len} mm，差 {ln - float(d_len):+.4f} mm",
                    provenance=f"{prov} / features.yaml:{fid}.length_mm / {tol_key}")
        sl = ln / th if th > 0 else None
        if sl is None:
            res.unknown(pid, f"{fid}:rib_slenderness", "厚度为 0，长厚比算不出来", provenance=prov)
        elif rib_max is None:
            res.unknown(pid, f"{fid}:rib_slenderness",
                        f"实测长厚比 {sl:.4f}（长 {ln:.4f} ÷ 厚 {th:.6f}），但 "
                        "tolerances.yaml:load.min_section_and_rib.max_rib_slenderness = null —— "
                        "PLA/TPU 的层间许用应力本仓库没有任何试件数据（TC01–TC05 都不测强度），"
                        "上限定不出来。**判据现在是真的在算了，缺的只是那个数**",
                        provenance=prov + " / tolerances.yaml:load.min_section_and_rib.max_rib_slenderness")
        else:
            res.add(subject=pid, check=f"{fid}:rib_slenderness",
                    state=PASS if sl <= float(rib_max) else FAIL, severity=BLOCK,
                    measured=round(sl, 4),
                    criterion=f"长厚比 ≤ {rib_max}（tolerances.yaml:load.min_section_and_rib."
                              f"max_rib_slenderness，{rib_src}）",
                    evidence_n=ev, detail=f"长 {ln:.4f} ÷ 厚 {th:.6f} = {sl:.4f}",
                    provenance=prov + " / tolerances.yaml:load.min_section_and_rib.max_rib_slenderness")


def _sections(ctx, res, assign, placed):
    """承力最小截面：沿 relations.yaml:load_paths 声明的方向、只在 from→to 那一段里切。

    2026-09-09 重写。旧版量的是"沿**世界 z** 每 1 mm 切一刀、**整件**最小的那一层水平截面积"——
    那不是承力截面：H03 量到的 7.611 mm² 是下头壳最上沿的一圈装饰唇边（一克载荷都不过），
    L06 鞋底的 256.897 mm² 反倒是整件最大的实心块；而且只沿重力方向切，L04 踝副轴颈那种层间剪切
    根本不在这个方向上；还只切了镜像对里的一个实例、且只在零位姿。
    见 tolerances.yaml:load.min_section_and_rib.min_load_section_mm2.refused_2026-09-09。

    现在：① 在**导出 STL（export_local = 该件所属连杆的坐标系）**上切 —— 与姿态、与左右镜像无关；
          ② 法平面沿声明的 direction；③ 只在 from→to 的投影区间里切；
          ④ 没有声明承力路径的件 → unknown 判红，detail 里贴 relations.yaml 自己写的理由。
    两种红要分得开：「没有承力路径声明」和「有路径、量出来了、但没有许用应力 σ_allow」。"""
    import numpy as np
    load = (ctx.data.get("tolerances") or {}).get("load") or {}
    msr = load.get("min_section_and_rib") or {}
    a_min, a_src = num(msr.get("min_load_section_mm2"))
    rel = ctx.data.get("relations") or {}
    paths = [p for p in (rel.get("load_paths") or []) if isinstance(p, dict)]
    no_path = {x.get("part"): x for x in (rel.get("parts_without_declared_load_path") or [])
               if isinstance(x, dict)}
    by_part = {}
    for p in paths:
        by_part.setdefault(p.get("part"), []).append(p)

    if a_min is None:
        res.unknown("_section", "min_load_section",
                    "tolerances.yaml:load.min_section_and_rib.min_load_section_mm2 = null。"
                    "**缺口有两个，不是一个**（2026-09-09 只说了数据侧那个，那句话是错的）：\n"
                    "① 数据侧：承力截面的门槛是 F/σ_allow，而 PLA/TPU 打印件的层间许用应力本仓库"
                    "没有任何试件数据（tolerances.yaml:deliberately_added 里 TC01–TC05 五个试件"
                    "都不测强度，coupons_printed: 0）。\n"
                    "② **层侧也还没修好**：2026-09-09 的改动只是把切平面的法向从『世界 z』换成了 "
                    f"relations.yaml:load_paths 声明的方向（{len(paths)} 条），"
                    "但用的仍然是 trimesh 的**无限平面**切整件 —— 量到的是整件截面，不是承力路径附近"
                    "的局部截面。把整条路径横向平移 10 m，空刀数与逐刀面积一位不差"
                    "（2026-09-10 实测；逐条见各件的 <lid>:load_path_local_section）。"
                    "所以 σ_allow 就算填上，也只会给出一个高估的『截面』",
                    provenance="tolerances.yaml:load.min_section_and_rib.min_load_section_mm2 / "
                               "relations.yaml:load_paths")
    res.unknown("_section", "layer_vs_load",
                "layer_vs_load_rule 只有一句话（『L04 踝副轴颈是从背板悬臂长出的光轴，层间剪切最危险 —— 无判据』），"
                "parts.yaml 的 print_orientation 是自由文本、没有打印坐标系变换 —— "
                "受力方向现在有了（relations.yaml:load_paths[].direction，export_local），"
                "但**层向**还没有：要算夹角，得先把 print_orientation 变成 export_local → 打印坐标系的变换矩阵。"
                "两个向量缺一个，夹角就算不出来",
                provenance="tolerances.yaml:load.min_section_and_rib.layer_vs_load_rule / "
                           "parts.yaml:parts[].print_orientation / relations.yaml:load_paths[].direction")

    for pid in sorted(assign):
        if ctx.only and pid not in ctx.only:
            continue
        mine = by_part.get(pid) or []
        if not mine:
            nd = no_path.get(pid)
            res.unknown(pid, "load_path_declared",
                        f"这个件没有声明承力路径（relations.yaml:load_paths 里没有 part={pid}）—— "
                        "所以本层量不了它的承力截面。"
                        + (f"relations.yaml:parts_without_declared_load_path 给的理由："
                           f"{nd.get('reason')}｜要什么才能写出来：{nd.get('what_would_unblock')}"
                           if nd else
                           "而且 relations.yaml:parts_without_declared_load_path 里也没写为什么写不出来 —— "
                           "连『为什么没有』都没有记录，这是数据缺口不是通过"),
                        provenance="relations.yaml:load_paths / "
                                   "relations.yaml:parts_without_declared_load_path")
            continue
        stl = ctx.stl(pid)
        if stl is None:
            res.unknown(pid, "load_section", f"找不到 {pid} 的导出 STL", provenance="parts.yaml:parts[].id")
            continue
        m = _mesh(stl)
        for p in mine:
            lid = p.get("id") or "?"
            prov = f"relations.yaml:load_paths.{lid}"
            if p.get("frame") != "export_local":
                res.unknown(pid, f"{lid}:load_section",
                            f"frame='{p.get('frame')}'，本层只在 export_local 里切（换算规则在 "
                            "features.yaml:frames，不写进层）", provenance=prov)
                continue
            d, dnote = _unit(p.get("direction"), "direction")
            p0, e0 = _pt3(p.get("from"), "pos_mm", "from.pos_mm")
            p1, e1 = _pt3(p.get("to"), "pos_mm", "to.pos_mm")
            bad = [x for x in (dnote if d is None else None, e0, e1) if x]
            if d is None or p0 is None or p1 is None:
                res.unknown(pid, f"{lid}:load_section", "承力路径声明不全：" + "；".join(bad),
                            provenance=prov)
                continue
            v = p1 - p0
            s1 = float(v @ d)
            if s1 <= _SECTION_STEP_MM:
                res.unknown(pid, f"{lid}:load_section",
                            f"from→to 在 direction 上的投影只有 {s1:.4f} mm（≤ 采样步距 "
                            f"{_SECTION_STEP_MM}）—— 方向写反了或者两点太近，切不出一段路径",
                            provenance=prov)
                continue
            L = float(np.linalg.norm(v))
            ang = math.degrees(math.acos(min(1.0, max(-1.0, s1 / L)))) if L > 0 else 0.0
            blo, bhi = m.bounds
            # 2026-09-10：**这一条只测得了轴对齐包围盒（AABB），不是"端点落在件上"。**
            # 降为 INFO，并在下面补一条红，说明真判据没实现。
            # 容差 1e-3 mm：落地面 / 贴合面这类端点**本来就在表面上**，不是内点
            ins = [bool(np.all(q >= blo - 1e-3) and np.all(q <= bhi + 1e-3)) for q in (p0, p1)]
            bb_vol = float(np.prod(bhi - blo))
            air = (1.0 - float(m.volume) / bb_vol) * 100.0 if bb_vol > 0 else float("nan")
            try:
                inmat = [bool(x) for x in m.contains(np.array([p0, p1]))]
                cerr = None
            except Exception as e:                              # noqa: BLE001
                inmat, cerr = None, str(e)
            res.add(subject=pid, check=f"{lid}:load_path_endpoints",
                    state=PASS if all(ins) else FAIL, severity=INFO, measured=sum(ins),
                    criterion="声明的 from/to 两个端点都落在该件导出 STL 的**轴对齐包围盒**内"
                              "（容差 1e-3 mm）—— 只是包围盒，**不等于落在件上**",
                    evidence_n=2,
                    detail=f"from {np.round(p0,3).tolist()} {'在盒内' if ins[0] else '**在盒外**'}、"
                           f"to {np.round(p1,3).tolist()} {'在盒内' if ins[1] else '**在盒外**'}；"
                           f"导出件包围盒 {np.round(blo,3).tolist()}..{np.round(bhi,3).tolist()}，"
                           f"盒里有 {air:.1f}% 是空气（体积 {m.volume:.1f} / 盒 {bb_vol:.1f} mm³）—— "
                           "所以『在盒内』这件事本身几乎不排除什么。**本条 2026-09-10 从 BLOCK 降为 INFO**，"
                           "真判据见同件 " + f"{lid}:load_path_endpoint_on_material",
                    provenance=prov + " / cad/duck_s288/" + Path(stl).name)
            _c_txt = ("trimesh.contains 判端点在实体内的个数 = "
                      f"{sum(inmat)}/2（from {'在料里' if inmat[0] else '不在料里'}、"
                      f"to {'在料里' if inmat[1] else '不在料里'}）" if inmat is not None
                      else f"trimesh.contains 跑不了：{cerr}")
            res.unknown(pid, f"{lid}:load_path_endpoint_on_material",
                        "**真判据没有实现**：要判的是『from/to 落在该件的料里，或者落在一个合法的"
                        "承力位置上（贴合面、法兰孔心、轴承座心这类端点合理地就不在实体内部）』，"
                        "本层现在只测得了轴对齐包围盒（同件 " + f"{lid}:load_path_endpoints" + "，"
                        f"该件包围盒 {air:.1f}% 是空气）。"
                        f"本轮顺手实测：{_c_txt} —— "
                        "**但不能直接把判据换成 contains**：那样上面这类合法端点会变成一批假红。"
                        "要解开需要两样东西里的一样：① relations.yaml:load_paths[].from/to 里声明端点"
                        "所依附的 features.yaml 特征 id（孔心/贴合面），层去查『端点是否落在该特征上』；"
                        "② 或者声明一个允许的贴面容差（端点到最近表面的距离上限），层去查距离。"
                        "两样都没有 → 按元规则 3，这里是红，不是通过",
                        provenance=prov + " / features.yaml / cad/duck_s288/" + Path(stl).name)
            areas, empty = [], []
            for s in np.arange(_SECTION_STEP_MM / 2.0, s1, _SECTION_STEP_MM):
                o = p0 + d * float(s)
                a = 0.0
                try:
                    sec = m.section(plane_origin=o.tolist(), plane_normal=d.tolist())
                    if sec is not None:
                        p2 = sec.to_2D()[0] if hasattr(sec, "to_2D") else sec.to_planar()[0]
                        a = abs(float(p2.area))
                except Exception as e:                          # noqa: BLE001
                    res.unknown(pid, f"{lid}:load_section",
                                f"在 s={float(s):.3f} mm 处切片失败：{e}", provenance=prov)
                    areas = None
                    break
                areas.append((a, float(s)))
                if a <= 0.0:
                    empty.append(float(s))
            if areas is None:
                continue
            if not areas:
                res.unknown(pid, f"{lid}:load_section", "路径区间里一刀都没切", provenance=prov)
                continue
            best = min(areas)
            # 2026-09-10：**这一条测的不是"承力路径全程有料"。** trimesh 的 section 是**无限平面**
            # 切整个件，返回值只取决于法向 d 和 o·d —— 也就是说它只对端点**沿 direction 的投影位置**
            # 敏感（路径起点落在件外那种错还是能抓到，relations.yaml:LP-L04-01/LP-N01-01 的 src_note
            # 记的就是这么发现的），对路径的**横向位置完全不敏感**。本轮实测见 detail。
            # 所以从 BLOCK 降为 INFO，并在下面补一条红说明真判据没实现。
            # 2026-09-10 复审纠正：这条 2026-09-10 早些时候被降成 INFO，**降过头了**。
            # 它量的「整件在该法平面上的截面积 > 0」是一条逻辑上完全站得住的**必要条件**：
            # 整件在那个平面上一点料都没有 ⟹ 承力路径上必然没料。空刀 > 0 是硬失败。
            # 它抓不到的是「路径横向位置不对」，那个交给同件的 load_path_local_section。
            # 所以 severity 改回 BLOCK，只保留改好的 criterion 文字（说清它是必要条件）。
            res.add(subject=pid, check=f"{lid}:load_path_continuous",
                    state=PASS if not empty else FAIL, severity=BLOCK, measured=len(empty),
                    criterion=f"沿 {lid} 的 direction、在 from→to 之间每 {_SECTION_STEP_MM} mm 取一张"
                              "**无限法平面**切整个件，每一刀的**整件截面积** > 0。"
                              "**这不是『承力路径上有料』**：section 的结果只由 (法向 d, 平面偏移 o·d) "
                              "决定，与路径的横向位置无关",
                    evidence_n=len(areas),
                    detail=f"{len(areas)} 刀 × {_SECTION_STEP_MM} mm，沿 {np.round(d,6).tolist()} 的法平面，"
                           f"s ∈ [0, {s1:.4f}]（from {np.round(p0,3).tolist()} → to "
                           f"{np.round(p1,3).tolist()}，直线距离 {L:.4f} mm，direction 与 to−from 夹角 "
                           f"{ang:.2f}°）；空刀 {len(empty)} 处"
                           + (f"：s={[round(x,2) for x in empty[:6]]}" if empty else "")
                           + (f"；{dnote}" if dnote else "")
                           + f"。**它与路径横向位置无关**：同件 {lid}:load_path_local_section 是"
                             "本层每轮现做的自检（把整条路径平移到件外再切一遍，数有几刀变了），"
                             "结果就在那一格。"
                             "**空刀 > 0 是硬失败**：整件在那一刀的法平面上都没料，路径上更没有 —— "
                             "这是必要条件，仍判 BLOCK。它抓到过真 bug（端点沿 direction 落在件外，"
                             "relations.yaml:LP-L04-01/LP-N01-01 的 src_note 记的就是这么发现的）。"
                             "它抓不到的是『路径横向位置压根不在料上』—— 那个交给 load_path_local_section",
                    provenance=prov + " / cad/duck_s288/" + Path(stl).name)
            # ── 自检：这刀到底量没量"这条路径"？ ──────────────────────────────
            # 把整条路径沿一个垂直于 direction 的方向平移 _SECTION_PROBE_MM（远到件外），
            # 再用同一段代码切一遍。真正的"沿路径局部截面"必然大变；而无限平面切整件
            # 只由 (法向 d, 偏移 o·d) 决定 → 一刀都不会变。变了几刀是**实测**出来的。
            _pp = np.cross(d, np.array([1.0, 0.0, 0.0]))
            if float(np.linalg.norm(_pp)) < 1e-6:
                _pp = np.cross(d, np.array([0.0, 1.0, 0.0]))
            _pp = _pp / float(np.linalg.norm(_pp))
            _shift = _pp * _SECTION_PROBE_MM
            n_moved, probe_ok, areas2 = 0, True, []
            for a, sv in areas:
                try:
                    _sec = m.section(plane_origin=(p0 + _shift + d * sv).tolist(),
                                     plane_normal=d.tolist())
                    a2 = 0.0 if _sec is None else abs(float(
                        (_sec.to_2D()[0] if hasattr(_sec, "to_2D") else _sec.to_planar()[0]).area))
                except Exception:                               # noqa: BLE001
                    probe_ok = False
                    break
                areas2.append(a2)
                if abs(a2 - a) > _SECTION_PROBE_REL * max(abs(a), abs(a2), 1.0):
                    n_moved += 1
            if not probe_ok:
                res.unknown(pid, f"{lid}:load_path_local_section",
                            "自检探针（把整条路径平移到件外再切一遍）跑失败 —— "
                            "拿不到『这刀量的是不是这条路径』的证据",
                            provenance=prov)
            else:
                # 判据锚在**本层真正报出去的那个数**（最小截面）上，而不是"随便哪一刀变了"：
                # 后者会被 10 m 平移带进来的浮点噪声骗过去（实测 5/11 条路径全刀"变了"，
                # 相对变化只有 ~1e-13）。所以两个都算，但只有最小截面变了才算过。
                min2 = min(areas2) if areas2 else None
                d_min = abs(min2 - best[0]) if min2 is not None else 0.0
                moved_min = (min2 is not None
                             and d_min > _SECTION_PROBE_REL * max(abs(best[0]), abs(min2), 1.0))
                res.add(subject=pid, check=f"{lid}:load_path_local_section",
                        state=PASS if moved_min else FAIL, severity=BLOCK,
                        measured=round(d_min, 9),
                        criterion="本层报出去的**最小截面**必须对承力路径的横向位置敏感 —— "
                                  f"把 {lid} 整条平移 {_SECTION_PROBE_MM:.0f} mm 到件外再切一遍，"
                                  f"最小截面的变化必须 > {_SECTION_PROBE_REL:g} × 量级"
                                  "（这个相对阈值只为挡住平移带来的浮点噪声）。"
                                  "纹丝不动 = 量的不是这条路径的截面",
                        evidence_n=len(areas),
                        detail=f"横移 {_SECTION_PROBE_MM:.0f} mm 后最小截面 "
                               f"{best[0]:.6f} → {(min2 if min2 is not None else float('nan')):.6f} mm²"
                               f"（变化 {d_min:.3e} mm²）；{len(areas)} 刀里有 {n_moved} 刀"
                               f"的截面积变化超过 {_SECTION_PROBE_REL:g} 相对阈值"
                               + ("。**最小截面纹丝不动 = 这条路径的位置对结果毫无影响**：本层用的 "
                                  "trimesh.Trimesh.section 是**无限平面**切整件，返回值只由 "
                                  "(法向 d, 平面偏移 o·d) 决定。所以同件的 "
                                  f"{lid}:load_path_continuous / {lid}:load_section 给的是"
                                  "**整件截面**，不是承力路径附近的局部截面。要真的量局部截面，"
                                  "至少需要 ① 把每刀的截面多边形按连通分量拆开、只保留包含路径点"
                                  "（或离它最近）的那一片，并声明『多远算这条路径的料』"
                                  "（一个半径/包络，现在 relations.yaml 里没有）；"
                                  "② 或者在 relations.yaml:load_paths 里声明该路径横截面所属的 "
                                  "features.yaml 特征。在那之前本层没有任何一条判据看过"
                                  "承力路径附近的料" if not moved_min else
                                  "。最小截面对路径位置有反应 —— 这条自检过了"),
                        provenance=prov + " / tools/gate/layers/l7_mass.py:_sections"
                                          "（trimesh.Trimesh.section 是无限平面）")
            res.add(subject=pid, check=f"{lid}:load_section", state=PASS, severity=INFO,
                    measured=round(best[0], 4),
                    criterion=f"**整件**在 {lid}（{p.get('load_case')}）的一族法平面上的最小截面积 mm²"
                              "（不是承力路径附近的局部截面 —— 见同件 "
                              f"{lid}:load_path_local_section）"
                              + (f"；参考门槛 ≥ {a_min}" if a_min is not None else
                                 "（阈值缺 σ_allow，见 L07/_section.min_load_section）"),
                    evidence_n=len(areas),
                    detail=f"最小 {best[0]:.4f} mm² @ s={best[1]:.2f} mm；"
                           f"沿路径 {min(a for a, _ in areas):.4f}..{max(a for a, _ in areas):.4f} mm²；"
                           f"起点『{(p.get('from') or {}).get('what')}』→ 终点『{(p.get('to') or {}).get('what')}』；"
                           f"工况 {p.get('load_case')}。**在 export_local 里切**，所以左右镜像件与"
                           f"零位/home 姿态下这个数是同一个（旧版沿世界 z 切，换姿态就变）。"
                           "**但这个数是整件在该平面上全部截面之和**：它包含离承力路径很远的料"
                           "（法兰边、外壳唇边都算进去），也不会因为路径挪到别处而变 —— "
                           f"证据见同件 {lid}:load_path_local_section（本层每轮现做的横移自检）。"
                           "拿它当『承力截面』会高估",
                    provenance=prov + " / tolerances.yaml:load.min_section_and_rib.min_load_section_mm2")
            if a_min is None:
                res.unknown(pid, f"{lid}:load_section_criteria",
                            f"**整件**在 {lid} 那一族法平面上的最小截面已经量出来了"
                            f"（{best[0]:.4f} mm² @ s={best[1]:.2f} mm，{len(areas)} 刀），"
                            "但 tolerances.yaml:load.min_section_and_rib."
                            "min_load_section_mm2 = null —— 门槛是 F/σ_allow，σ_allow 没有试件数据。"
                            "**这条红与『没有承力路径声明』那条红不是一回事**：路径有了、"
                            "缺的是材料门槛。"
                            "**注意还缺第二样**：量到的那个数是**整件**在法平面上的截面，不是路径附近的"
                            f"局部截面（证据见同件 {lid}:load_path_local_section 的横移自检）—— "
                            "即使 σ_allow 填上，"
                            "也得先把局部截面量出来，这条才会变成真判据",
                            provenance="tolerances.yaml:load.min_section_and_rib.min_load_section_mm2 / "
                                       "tolerances.yaml:deliberately_added（TC01–TC05，coupons_printed: 0）")
    _ribs(ctx, res, assign)
