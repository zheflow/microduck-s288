#!/usr/bin/env python3
"""从我们自己的 CAD 生成训练用的 MJCF —— `sim/duck_s288/robot_walk_s288.xml`。

    ./.venv/bin/python tools/sim/make_mjcf.py

2026-09-09 定了重训（docs/重训路线_2026-09-09.md）之后，权威方向倒过来了：
以前 MJCF 是冻结输入、CAD 必须符合它；现在 **CAD + 实测是真理，MJCF 由它生成**。

**不改 upstream/**。上游那份 `robot_walk.xml` 一个字节都不动，只当骨架读 ——
这样它仍然可以跟上游 diff，也仍然是 frozen.yaml:kinematics_sha256 的基线。

这一版负责生成的：
  1. `<joint range>`  ← tools/gate/out/mjcf_ranges.json 的 **target_range_deg**（frozen.yaml 控制端目标区间，
                        09-15 按第 6 层无碰撞区间各留 3° 收窄；退回 collision_free_range_deg 仅当没有 target）
  2. `<inertial>`     ← 我们的 placed/ 实体 × 密度 + components.yaml 的元件实重（servo 20.33 实称等，读 yaml 不硬编）
                        + 未被 placed 实体认领的已知点质量（实体与点质量不能重复计算）
  3. `<geom>`         ← 两份输出：
        robot_walk_s288.xml        全部 placed 件逐件凸包（Gate 第 6/7 层读它；自碰覆盖）
        robot_walk_s288_train.xml  脚底碰撞（sole/sole_R 凸包，命名 left/right_foot_collision）+ 小腿/躯干
                                   自碰体（lower_leg/lower_leg_R/trunk/zz_battery 凸包 → class self_collision_only，
                                   contype=conaffinity=2 只互碰不碰地）—— 与上游 robot_walk.xml 同构（上游是 leg×2 +
                                   power_support 三个自碰体）。mjlab 速度任务用它：脚底碰撞 + trunk 子树自碰传感器；
                                   全件凸包会在零位贴合处（hip×servo）虚报自碰把训练压死。
                                   09-16 v1 训练（无自碰体）策略把两腿交叉穿过去 → 补上
        robot_walk_s288_allcol.xml 起立/坐站/翻身/捡物/踢球用（对应上游 robot_allcollisions.xml）：脚底 + 躯干/壳/电池门 +
                                   髋件 + 小腿 + 三片头壳 的凸包都是 class collision（碰地也互碰，同上游 11 个 collision 网格：
                                   np_f970/hip_l×2/leg×2/sole×2/top_head_shell/jaw/bottom_head_shell），电池仍 self_collision_only。
                                   09-16 加：要训 VelStand/StandUp/SitStand/Roulade/GroundPick/BallKick 六个能力
  4. `<default class="chosen_actuator">` ← S288 BAM m1（09-16 台架辨识，04_电机到货测试/params/s288/README.md）：
        forcerange ±0.706（电机侧，减 0.066 摩擦 = 输出 0.64，吊瓶实测 0.66）、frictionloss 0.066、damping 0.0108、
        armature 5.5e-4；position kp = 0.941×S288_KP_CMD、kv = 0.886×S288_KD_CMD（固件标度实测）。
        mjlab 训练时 BAM 执行器会把这些覆盖成逐步计算的摩擦预算，这里的值给纯 MuJoCo 回放/Gate 用。

**故意没有生成的**，都在输出文件头部列成 TODO，不许当成已经对了：
  · `<site>` 位置 —— 改传感器安装点会改观测，不在本脚本的范围内
  · 未称重的电子件（屏/摄像头/麦/降压板/总线板/IMU/线/开关）—— components.yaml 里 mass_g 为空，不编；
    训练侧用躯干/头质量随机化上限覆盖（见 sim/duck_s288/mjlab_duck_s288/）
"""
from __future__ import annotations

import json
import math
import re
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import trimesh
import yaml

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
sys.path.insert(0, str(ROOT))

from duckstructure import kin                                      # noqa: E402

PLACED = ROOT / "cad/duck_s288/placed"
MANIFEST = HERE / "cad_geometry_manifest.json"
RANGES = ROOT / "tools/gate/out/mjcf_ranges.json"
OUT_DIR = ROOT / "sim/duck_s288"
ASSETS = OUT_DIR / "assets"

# 回放变体配色（取自上游 robot_walk.xml 的 material rgba：壳白、脚/嘴橙、面板深灰、舵机深灰、轴承浅蓝灰）
PLAY_PALETTE = {
    "white": "0.901961 0.901961 0.901961 1",
    "shell": "0.8 0.8 0.8 1",
    "light": "0.866667 0.866667 0.866667 1",
    "leg": "0.811765 0.858824 0.898039 1",
    "teal": "0.537255 0.854902 0.827451 1",
    "orange": "0.980392 0.713725 0.00392157 1",
    "dark": "0.262745 0.282353 0.301961 1",
    "panel": "0.34902 0.376471 0.4 1",
    "servo": "0.286275 0.286275 0.286275 1",
    "bearing": "0.713725 0.760784 0.8 1",
    "mid": "0.647059 0.647059 0.647059 1",
    "sky": "0.768627 0.886275 0.952941 1",
}


def play_material_for(stem: str) -> str:
    s = stem.lower()
    if s.startswith("servo__"):
        return "servo"
    if s.startswith("bearing_"):
        return "bearing"
    if s in ("ankle_foot", "ankle_foot_R".lower(), "jaw", "head_bottom_shell"):        # hr38：orig_jaw -> J01 jaw
        return "orange"
    if s.startswith("sole"):
        return "teal"
    if s in ("orig_face_part", "trunk", "neck_pitch", "zz_battery"):
        return "panel"
    if s.startswith("head_bracket") or s.startswith("head_clamp") or s.startswith("head_journal") or s.startswith("ankle_rear_arm"):
        return "dark"
    if s.startswith("lower_leg"):
        return "leg"
    if s.startswith("upper_leg"):
        return "light"
    if s.startswith("yaw2roll"):
        return "mid"
    if s.startswith("yrm"):
        return "sky"
    if s.startswith("hip") or s == "battery_door" or s == "head_top_shell":            # hr38：orig_top_head_shell -> H05
        return "white"
    if s.startswith("shell") or s == "neck":
        return "shell"
    return "light"

# 密度：仓库里只有一个值（tolerances.yaml:density_g_cm3 = 1.27, src: assumed，打印清单口径）。
# TPU 95A 实际约 1.21，但本仓库没有第二个值，硬编一个网上的数会变成凭空阈值 —— 统一用 1.27 并声明。
DENSITY_G_CM3 = 1.27

# 外购件实重：从 components.yaml:mass_g 读（09-16 起不再硬编；09-15 舵机实称 20.33 替代假设 19.5）。
COMPONENTS_YAML = ROOT / "tools/gate/data/components.yaml"


class MissingComponentMass(ValueError):
    pass


def _component_mass_g(cid: str) -> float:
    """components.yaml 里 id=cid 的 mass_g（标量或 {v: ...}）；没有就抛错，不编数。"""
    doc = yaml.safe_load(COMPONENTS_YAML.read_text(encoding="utf-8"))

    def find(o):
        if isinstance(o, dict):
            if o.get("id") == cid:
                return o
            for v in o.values():
                r = find(v)
                if r is not None:
                    return r
        elif isinstance(o, list):
            for v in o:
                r = find(v)
                if r is not None:
                    return r
        return None
    c = find(doc.get("components", []))
    if c is None:
        raise KeyError(f"components.yaml 没有 id={cid}")
    m = c.get("mass_g")
    if isinstance(m, dict):
        m = m.get("v")
    if m is None:
        raise MissingComponentMass(f"components.yaml:{cid}.mass_g 为空 —— 不编数")
    if isinstance(m, bool) or not math.isfinite(float(m)) or float(m) <= 0:
        raise ValueError(f"components.yaml:{cid}.mass_g 必须是有限正数")
    return float(m)


SERVO_G = _component_mass_g("servo_s288")
BRG6704_G = _component_mass_g("bearing_6704zz")
BRG6700_G = _component_mass_g("bearing_6700zz")
BRG6703_G = _component_mass_g("bearing_6703zz")   # 09-17 hr12：髋横滚/髋俯仰恢复为 6703ZZ ×4（components.yaml 目录净重 4.2 g）
BATTERY_G = _component_mass_g("battery_3s")

# 已知质量 + 位置可由 CAD/数据推出的电子件 → 点质量（世界 mm, 质量 g, 所属 body, 出处）
#   Radxa ZERO 3W（2026-09-17 换板；14 g 为占位，官网无重量）：head.py 前立板 4 根 Ø5 立柱 x 58.5..66（长 15）、中心 (y 0, z 251)，板贴柱尖 → 板中面 x≈66.8
#   XT30 1.7 g：components.yaml mount "2×Ø2.2 扎带孔 @(-23.1,±9,134.9)" → 中点
POINT_MASSES = [
    ("sbc_radxa_zero3w", (66.8, 0.0, 251.0), _component_mass_g("sbc_radxa_zero3w"), "jaw_soft",
     "duckstructure/head.py posts x 58.5+15 / brd_h 中心 (0,251)"),
    ("connector_xt30", (-23.1, 0.0, 134.9), _component_mass_g("connector_xt30"), "trunk_base",
     "components.yaml:connector_xt30.mount 扎带孔中点"),
]

# Collision envelopes are not printed parts. Use the corresponding purchased
# mass wherever available; retained volume proxies must remain explicit.
COMPONENT_FOR_PLACED = {
    "zz_battery": "battery_3s", "zz_sbc": "sbc_radxa_zero3w",
    "zz_imu": "imu_icm42688", "zz_camera": "camera_csi",
    "zz_mic": "mic_inmp441", "zz_ubec": "buck_12v_5v",
    "zz_adapter": "bus_adapter", "zz_amp": "amp_max98357a",
    "zz_speaker": "speaker", "zz_xt30": "connector_xt30",
}


def component_for_part(stem: str) -> str | None:
    if stem.startswith("servo__"):
        return "servo_s288"
    if stem.startswith("bearing_"):
        if "ankle" in stem or stem == "bearing_jaw":
            return "bearing_6700zz"
        if "hip_roll" in stem or "hip_pitch" in stem:
            return "bearing_6703zz"
        if "hip_yaw" in stem:
            return "bearing_6702zz"
        if stem == "bearing_head_roll_B":
            return "bearing_22x16x4"
        return "bearing_6704zz"
    return COMPONENT_FOR_PLACED.get(stem)


def point_masses_for_parts(stems):
    represented = {component_for_part(s) for s in stems}
    return [p for p in POINT_MASSES if p[0] not in represented]

# S288 指令刚度/阻尼（输出端 N·m/rad, N·m·s/rad）。上游 XL330 "200 kp" 等效 0.55 N·m/rad（很软）；
# S288 齿轮箱摩擦 0.066 N·m 是 XL330 的 13 倍，同样软会有 6° 死区，取 2.0（台架辨识覆盖 2~12）→ 死区 ≈1.9°、
# 0.66 N·m 上限在 19° 误差处饱和。训练侧 mjlab 用同一常量（mjlab_duck_s288 里 kp_fw=S288_KP_CMD）。
S288_KP_CMD, S288_KD_CMD = 2.0, 0.05
S288_KP_RATIO, S288_KD_RATIO = 0.941, 0.886            # 固件标度实测（04_电机到货测试/s288_bam.py）
S288_BAM_M1 = ROOT / "V2_S288版发布产物/04_电机到货测试/params/s288/m1.json"


def part_mass_g(stem: str, mesh: trimesh.Trimesh) -> tuple[float, str]:
    """→ (质量 g, 来源)。外购件用实重，打印件和原版件用 体积×密度。"""
    cid = component_for_part(stem)
    if cid is not None:
        try:
            return _component_mass_g(cid), f"components.yaml:{cid}.mass_g"
        except MissingComponentMass:
            # Historical proxies are retained only for unweighed components.
            # They are listed in build_report, never called measured masses.
            pass
    if stem.startswith("zz_") and cid is None:
        raise ValueError(f"外购件 {stem} 缺元件质量映射，不能按打印塑料计算")
    if stem.startswith("bearing_"):
        return BRG6704_G, f"unmeasured_proxy:{cid}; historical 6704 mass proxy (not measured)"
    if cid is not None:
        return mesh.volume / 1000.0 * DENSITY_G_CM3, f"unmeasured_proxy:{cid}; envelope volume×{DENSITY_G_CM3} (not measured)"
    return mesh.volume / 1000.0 * DENSITY_G_CM3, f"体积×{DENSITY_G_CM3} g/cm³(assumed)"


def inertia_about(mesh: trimesh.Trimesh, mass_g: float, about_m: np.ndarray) -> np.ndarray:
    """网格（世界系、mm）关于 about_m（米）的惯量张量，单位 kg·m²。
    trimesh 的 moment_inertia 是关于**自身质心**、密度=1 的；这里换算成真实质量再平行轴。"""
    m_kg = mass_g / 1000.0
    # 密度=1 时的张量正比于体积；除掉它再乘真实质量 → 与网格是否闭合的体积估计一致
    v = float(mesh.volume)
    if v <= 0:
        raise ValueError("体积 ≤ 0")
    I_c = np.asarray(mesh.moment_inertia, float) / v * m_kg    # mm² · kg
    I_c *= 1e-6                                                # mm² → m²
    c_m = np.asarray(mesh.center_mass, float) / 1000.0
    d = c_m - about_m
    return I_c + m_kg * (np.dot(d, d) * np.eye(3) - np.outer(d, d))


def _halves(mesh: trimesh.Trimesh):
    """沿本块自己最长的轴对半切。→ [块] 或 None（切不动）。"""
    ext = mesh.bounds[1] - mesh.bounds[0]
    ax = int(np.argmax(ext))
    n = np.zeros(3); n[ax] = 1.0
    o = np.zeros(3); o[ax] = float(mesh.bounds[0][ax] + ext[ax] / 2.0)
    out = []
    for sign in (1.0, -1.0):
        try:
            sl = mesh.slice_plane(o, sign * n, cap=True)
        except Exception:                                           # noqa: BLE001
            return None
        if sl is None or sl.is_empty or len(sl.faces) < 4 or float(sl.volume) <= 1e-9:
            continue
        out.append(sl)
    return out or None


def convex_pieces(mesh: trimesh.Trimesh, max_ratio: float = 1.5,
                  max_pieces: int = 16) -> tuple[list, float, int]:
    """→ ([凸块], 体积膨胀比, 片数)。

    MuJoCo 的 mesh geom **一律按凸包参与碰撞**，所以凹件（trunk 的电池仓、
    头壳的穹顶、upper_leg 的舵机笼）会被填实，在真实网格根本不碰的姿态上虚报接触。
    本机没有 coacd / vhacdx（trimesh.convex_decomposition 要它们），这里自己做：
    **按膨胀比递归二分** —— 哪块还太胖就沿它自己的最长轴再切一刀。
    切的方向由每块自己的形状决定，不预设、不对某个件硬编码。
    """
    v = float(mesh.volume)
    if v <= 0:
        return [mesh.convex_hull], float("inf"), 1

    def ratio_of(parts):
        return sum(float(x.convex_hull.volume) for x in parts) / v

    parts = [mesh]
    while len(parts) < max_pieces and ratio_of(parts) > max_ratio:
        # 每轮只切"最亏"的那一块（凸包比它自己实体多出来最多的）
        worst_i, worst_gain = None, 0.0
        for i, x in enumerate(parts):
            gain = float(x.convex_hull.volume) - float(x.volume)
            if gain > worst_gain:
                worst_i, worst_gain = i, gain
        if worst_i is None:
            break
        hs = _halves(parts[worst_i])
        if not hs or len(hs) < 2:
            break
        parts = parts[:worst_i] + hs + parts[worst_i + 1:]
    hulls = [x.convex_hull for x in parts]
    return hulls, ratio_of(parts), len(hulls)


# ── hr41 落盘 2026-09-25（主设计 Lane C）：嘴关节链（hr41_头部阶段二.md §9 / §11 第 5 条；hr39c_复审_1.md m14 口径）────────────
#   链 = 嘴舵机法兰 → J02 转接盘（F28 3×M2×5 自攻，装一次不拆）→ J01 活动嘴（F34 3×M2×5 沉头 + 螺母）(+ J03 轴颈盘，F34 另 3 颗，装在 6700ZZ 内圈)。
#   这几样随嘴转（= duckstructure/jaw.py:JAW_MOVERS + 惰轮侧 6700ZZ），从头刚体 jaw_soft 里分出来成子 body "jaw"：原点 = 嘴轴点 jaw.JAW_AXIS_P，
#   姿态同 jaw_soft。默认**不加关节**（子 body 焊在 jaw_soft 上：整头合成刚体的质量/质心/惯量与合在一起逐项等价，策略 action 仍 14 维）；
#   环境变量 DUCK_MJCF_JAW_JOINT=1 才加 hinge "jaw"（轴 jaw.JAW_AXIS_D，区间 jaw.JAW_RANGE_ORIG）+ 同款 S288 位置执行器 —— 加了就是 15 维、要重训（待主设计定）。
#   质量口径（m14；hr41 原文只有"m14：MJCF 质量口径"一句 → 按主设计 2026-09-25 口径）：J01/J02/J03 = placed STL 体积 × DENSITY_G_CM3 1.27（实心，assumed）；
#   6700ZZ = components.yaml:bearing_6700zz.mass_g 1.9 g（datasheet）整颗记在嘴上（外圈其实随头，偏保守）；嘴上 9 颗螺丝 + 6 颗螺母未称、未计。
JAW_CHAIN = (("jaw_adapter", "J02"), ("jaw", "J01"), ("jaw_journal", "J03"), ("bearing_jaw", "6700ZZ"))
JAW_BODY, JAW_PARENT = "jaw", "jaw_soft"


def _jaw_joint_enabled() -> bool:
    import os
    return os.environ.get("DUCK_MJCF_JAW_JOINT", "0") == "1"


def split_jaw_chain(per_body: dict, bodies: dict, notes: list) -> dict | None:
    """把 JAW_CHAIN 的件从 per_body[JAW_PARENT] 挪到 per_body[JAW_BODY]，并给 bodies[JAW_BODY] 定零位姿（原点 = 嘴轴点、姿态同父）。
    缺件照实记 notes（不补、不猜）；一件都不在就不建子 body。→ 报告 dict 或 None。"""
    from duckstructure import jaw as JAW
    parent = per_body.get(JAW_PARENT) or []
    want = dict(JAW_CHAIN)
    moved = [it for it in parent if it[0] in want]
    missing = [s for s, _pid in JAW_CHAIN if s not in {it[0] for it in moved}]
    if missing:
        notes.append(f"嘴关节链缺件（placed/ 或 body_for_part→{JAW_PARENT} 里没有）：{missing}")
    if not moved:
        return None
    per_body[JAW_PARENT] = [it for it in parent if it[0] not in want]
    per_body[JAW_BODY] = moved
    Tp = np.asarray(bodies[JAW_PARENT]["T_world"], float)
    Tj = Tp.copy()
    Tj[:3, 3] = np.asarray(JAW.JAW_AXIS_P, float)                   # 世界 mm；姿态同 jaw_soft
    bodies[JAW_BODY] = {"T_world": Tj}
    return {"body": JAW_BODY, "parent": JAW_PARENT, "joint": _jaw_joint_enabled(),
            "chain": "嘴舵机法兰 → J02 → J01 (+J03, 6700ZZ)",
            "axis_point_world_mm": [float(x) for x in JAW.JAW_AXIS_P], "axis_dir_world": [float(x) for x in JAW.JAW_AXIS_D],
            "range_deg": [float(x) for x in JAW.JAW_RANGE_ORIG],
            "parts": [{"part": s, "id": want[s], "mass_g": round(g, 4), "src": src} for s, _m, g, src in moved],
            "mass_g": round(sum(g for _s, _m, g, _src in moved), 4), "missing": missing,
            "mass_basis": "m14 口径（主设计 2026-09-25）：J01/J02/J03 = placed STL 体积 × 1.27 g/cm³（实心，assumed）+ 6700ZZ 1.9 g"
                          "（components.yaml:bearing_6700zz.mass_g，datasheet，整颗记在嘴上）；螺丝/螺母未计",
            "src": "duckstructure/jaw.py:JAW_AXIS_P / JAW_AXIS_D / JAW_RANGE_ORIG / JAW_MOVERS；hr41_头部阶段二.md §9、§11 第 5 条；hr39c_复审_1.md m14"}


def add_jaw_body(root, body_el: dict, bodies: dict) -> ET.Element:
    """在 jaw_soft 下挂 <body name=jaw>（pos = 嘴轴点在父系的坐标，quat 单位）；DUCK_MJCF_JAW_JOINT=1 时再加 hinge + 执行器。"""
    from duckstructure import jaw as JAW
    Tp = np.asarray(bodies[JAW_PARENT]["T_world"], float)
    Tj = np.asarray(bodies[JAW_BODY]["T_world"], float)
    rel = np.linalg.inv(Tp) @ Tj
    rel[:3, 3][np.abs(rel[:3, 3]) < 1e-9] = 0.0                     # 求逆的浮点噪声（y 7e-18）写成 0
    el = ET.SubElement(body_el[JAW_PARENT], "body", {"name": JAW_BODY, "pos": " ".join(f"{x / 1000.0:.9g}" for x in rel[:3, 3]),
                                                      "quat": "1 0 0 0"})
    if _jaw_joint_enabled():
        ax = Tj[:3, :3].T @ np.asarray(JAW.JAW_AXIS_D, float)
        lo, hi = (math.radians(float(x)) for x in JAW.JAW_RANGE_ORIG)
        ET.SubElement(el, "joint", {"axis": " ".join(f"{x:.9g}" for x in ax), "name": JAW_BODY, "type": "hinge",
                                    "range": f"{lo:.10g} {hi:.10g}", "class": "chosen_actuator"})
        act = root.find("actuator")
        ET.SubElement(act, "position", {"class": "chosen_actuator", "name": JAW_BODY, "joint": JAW_BODY})
    return el


def main() -> int:
    bodies, order = kin.load()
    body_for_part = json.loads(MANIFEST.read_text(encoding="utf-8"))["body_for_part"]
    ranges_deg = {}
    if RANGES.exists():
        _rj = json.loads(RANGES.read_text(encoding="utf-8"))
        ranges_deg = _rj.get("target_range_deg") or _rj.get("collision_free_range_deg") or {}
        ranges_key = "target_range_deg" if _rj.get("target_range_deg") else "collision_free_range_deg"
    else:
        ranges_key = "(无)"
        print(f"[警告] {RANGES} 不存在 —— 关节区间保持上游原值，先跑一次 gate.py --layers 6")

    ASSETS.mkdir(parents=True, exist_ok=True)
    for old in ASSETS.glob("*.stl"):
        old.unlink()

    # ── 逐件：世界 mm → body 局部 m，取凸包，导出 ──────────────────────
    per_body: dict[str, list] = {}
    notes: list[str] = []
    vis_meshes: list[tuple[str, str, object]] = []                # (body, stem, 局部 m 网格)
    for stl in sorted(PLACED.glob("*.stl")):
        stem = stl.stem
        body = body_for_part.get(stem)
        if body is None:
            notes.append(f"placed/{stem}.stl 不在 body_for_part 里 —— 没有进任何 body")
            continue
        m = trimesh.load(str(stl), process=True)
        g, src = part_mass_g(stem, m)
        per_body.setdefault(body, []).append((stem, m, g, src))
    # hr41 落盘 2026-09-25（主设计 Lane C）：嘴关节链 J02/J01/J03/6700 从 jaw_soft 分到子 body "jaw"（见 JAW_CHAIN 注）
    jaw_report = split_jaw_chain(per_body, bodies, notes)

    # ── 质量清单里缺的（placed/ 里没有实体的元件）──────────────────────
    placed_stems = {s for items in per_body.values() for s, *_ in items}
    point_masses = point_masses_for_parts(placed_stems)
    mass_proxies = [{"part": s, "mass_g": g, "src": src}
                    for items in per_body.values() for s, _m, g, src in items
                    if src.startswith("unmeasured_proxy:")]
    represented = {component_for_part(s) for s in placed_stems}
    represented.update(c for c, *_ in point_masses)
    proxy_components = {component_for_part(p["part"]) for p in mass_proxies}
    partial_masses = []
    if "zz_xt30" in placed_stems:
        partial_masses.append({"part": "zz_xt30", "component": "connector_xt30",
                               "scope": "已有插头质量只计一次；合并包络中的平衡头和40cm延长线质量尚缺"})
        proxy_components.add("connector_xt30")
    components = yaml.safe_load(COMPONENTS_YAML.read_text(encoding="utf-8"))["components"]
    missing = sorted(c["id"] for c in components if c.get("qty", 0) != 0
                     and c["id"] not in ("filament", "original_prints")
                     and (c["id"] not in represented or c["id"] in proxy_components))

    root = ET.parse(str(Path(kin.MD) / "robot_walk.xml")).getroot()
    body_el = {b.get("name"): b for b in root.iter("body")}
    if jaw_report is not None:                                     # hr41 落盘 2026-09-25（Lane C）：先挂子 body，下面逐 body 循环照常写 inertial/geom
        body_el[JAW_BODY] = add_jaw_body(root, body_el, bodies)

    asset = root.find("asset")
    for child in list(asset):
        asset.remove(child)

    totals, hull_report = {}, {}
    for name, items in per_body.items():
        el = body_el.get(name)
        if el is None:
            notes.append(f"body {name} 在 robot_walk.xml 里找不到")
            continue
        Tw = np.asarray(bodies[name]["T_world"], float)            # 零位姿，mm
        Tinv = np.linalg.inv(Tw)
        origin_m = Tw[:3, 3] / 1000.0

        # 惯量：先在**世界系**累加（各件都在世界系里），最后整体转到 body 局部系
        pts = [(cid, np.asarray(xyz, float), g) for cid, xyz, g, b, _src in point_masses if b == name]
        M = sum(g for _s, _m, g, _src in items) + sum(g for _c, _x, g in pts)
        com_w = (sum(np.asarray(m.center_mass, float) * g for _s, m, g, _src in items)
                 + sum(x * g for _c, x, g in pts)) / M
        I_w = np.zeros((3, 3))
        for _s, m, g, _src in items:
            I_w += inertia_about(m, g, com_w / 1000.0)
        for _c, x, g in pts:                                       # 点质量：只有平行轴项
            d = (x - com_w) / 1000.0
            I_w += g / 1000.0 * (np.dot(d, d) * np.eye(3) - np.outer(d, d))
        R = Tw[:3, :3]
        I_b = R.T @ I_w @ R                                        # 世界 → body 局部（同一点）
        com_b = (Tinv @ np.append(com_w, 1.0))[:3] / 1000.0
        totals[name] = (M, com_b, I_b)

        for old in el.findall("inertial"):
            el.remove(old)
        ine = ET.Element("inertial")
        ine.set("pos", " ".join(f"{x:.9g}" for x in com_b))
        ine.set("mass", f"{M / 1000.0:.9g}")
        ine.set("fullinertia", " ".join(f"{x:.9g}" for x in
                                        (I_b[0, 0], I_b[1, 1], I_b[2, 2],
                                         I_b[0, 1], I_b[0, 2], I_b[1, 2])))
        el.insert(0, ine)

        # geom 全换成我们的件
        for old in el.findall("geom"):
            el.remove(old)
        for stem, m, _g, _src in items:
            loc = m.copy()
            loc.apply_transform(Tinv)                              # → body 局部，mm
            loc.apply_scale(0.001)                                 # mm → m
            vis_meshes.append((name, stem, loc.copy()))            # 回放变体用整件网格当视觉
            if stem in ("sole", "sole_R"):                       # 脚底：单凸包（训练变体要用它当 foot_collision）
                pieces, ratio, k = [loc.convex_hull], float(loc.convex_hull.volume / max(loc.volume, 1e-9)), 1
            else:
                pieces, ratio, k = convex_pieces(loc)
            hull_report[stem] = {"pieces": k, "volume_ratio": round(ratio, 4)}
            for i, h in enumerate(pieces):
                gn = f"col_{stem}" + (f"_{i}" if len(pieces) > 1 else "")
                h.export(str(ASSETS / f"{gn}.stl"))
                ET.SubElement(asset, "mesh", {"name": gn, "file": f"{gn}.stl"})
                ET.SubElement(el, "geom", {
                    "type": "mesh", "name": gn, "class": "collision", "mesh": gn})

    # ── 关节区间 ────────────────────────────────────────────────────────
    changed = []
    for j in root.iter("joint"):
        nm = j.get("name")
        rg = ranges_deg.get(nm)
        if not rg:
            continue
        lo_r, hi_r = math.radians(rg[0]), math.radians(rg[1])
        old = [float(x) for x in (j.get("range") or "0 0").split()]
        j.set("range", f"{lo_r:.10g} {hi_r:.10g}")
        # 数值比，不比字符串 —— 上游写的是 -1.5707963267948966，我们写 -1.570796327，
        # 字符串永远不等，会把 14 条全报成"收窄了"
        if abs(old[0] - lo_r) > 1e-9 or abs(old[1] - hi_r) > 1e-9:
            changed.append(f"{nm} {math.degrees(old[0]):.1f}..{math.degrees(old[1]):.1f}"
                           f"→{rg[0]:.1f}..{rg[1]:.1f}°")

    comp = root.find("compiler")
    if comp is None:
        comp = ET.Element("compiler"); root.insert(0, comp)
    comp.set("meshdir", "assets")
    comp.set("angle", "radian")

    # ── 执行器默认值 ← S288 BAM m1（09-16）────────────────────────────
    bam = json.loads(S288_BAM_M1.read_text(encoding="utf-8"))
    act_vals = dict(forcerange=bam["max_torque"], frictionloss=bam["friction_base"],
                    damping=bam["friction_viscous"], armature=bam["armature"],
                    kp=S288_KP_RATIO * S288_KP_CMD, kv=S288_KD_RATIO * S288_KD_CMD)
    n_act_def = 0
    for d in root.iter("default"):
        if d.get("class") != "chosen_actuator":
            continue
        for j in d.findall("joint"):
            j.set("damping", f"{act_vals['damping']:.6g}"); j.set("frictionloss", f"{act_vals['frictionloss']:.6g}")
            j.set("armature", f"{act_vals['armature']:.6g}")
        for pz in d.findall("position"):
            pz.set("kp", f"{act_vals['kp']:.6g}"); pz.set("kv", f"{act_vals['kv']:.6g}")
            pz.set("forcerange", f"-{act_vals['forcerange']:.6g} {act_vals['forcerange']:.6g}")
        n_act_def += 1
    if n_act_def == 0:
        notes.append("骨架里没有 class=chosen_actuator 的 default —— 执行器参数没写进去")

    def _header(variant: str) -> ET.Comment:
        geom_line = (f"<geom class=\"collision\">（{sum(len(v) for v in per_body.values())} 件逐件凸包；Gate 第 6/7 层读本文件）"
                     if variant == "all" else
                     "同 robot_walk_s288_train.xml 的碰撞/自碰几何（物理逐字节相同）+ 整件网格视觉（vis_*.stl，原版涂装配色）"
                     "+ 相机眼睛；只供 play/录像，Gate 不读"
                     if variant == "play" else
                     "<geom class=\"collision\">（脚底 + 躯干/壳/电池门 + 髋件 + 小腿 + 三片头壳 的凸包：碰地也互碰，"
                     "对应上游 robot_allcollisions.xml 的 11 个 collision 网格）+ 电池 self_collision_only，其余件仅 visual；"
                     "供 VelStand/StandUp/SitStand/Roulade/GroundPick/BallKick"
                     if variant == "allcol" else
                     "<geom class=\"collision\">（脚底 left/right_foot_collision = sole/sole_R 凸包）+ "
                     "<geom class=\"self_collision_only\">（小腿 lower_leg/lower_leg_R + 躯干 trunk/zz_battery 凸包，只互碰；"
                     "对应上游 leg×2 + power_support），其余件仅 visual；与上游 robot_walk.xml 同构，供 mjlab 速度任务）")
        return ET.Comment(
            "\n  本文件由 tools/sim/make_mjcf.py 生成，不要手改 —— 改了下次重跑就没了。\n"
            "  权威方向：CAD + 实测 → MJCF（docs/重训路线_2026-09-09.md）。\n"
            f"  已生成：<inertial>（{len(totals)} 个 body，来自 placed/ × 密度 {DENSITY_G_CM3} g/cm³ + components.yaml 元件实重"
            f" + 点质量 {', '.join(c for c, *_ in point_masses) or '无'}；未称重代理项 {len(mass_proxies)}）\n"
            f"          {geom_line}\n"
            f"          <joint range>（{len(changed)} 条 ← mjcf_ranges.json:{ranges_key}：{'; '.join(changed) or '无'}）\n"
            f"          <default class=\"chosen_actuator\">（S288 BAM m1 {S288_BAM_M1.relative_to(ROOT)}：forcerange ±{act_vals['forcerange']:.3f}"
            f" frictionloss {act_vals['frictionloss']:.4f} damping {act_vals['damping']:.4f} armature {act_vals['armature']:.2e}；"
            f" kp {act_vals['kp']:.3f} = {S288_KP_RATIO}×{S288_KP_CMD}, kv {act_vals['kv']:.4f} = {S288_KD_RATIO}×{S288_KD_CMD}；"
            " mjlab 训练时 BAM 执行器逐步覆盖）\n"
            + (f"          <body name=\"{JAW_BODY}\">（嘴关节链 {jaw_report['chain']}，{jaw_report['mass_g']:.3f} g，"
               + ("hinge + 执行器 = 第 15 个自由度" if jaw_report["joint"] else "无关节 = 焊在 jaw_soft 上，action 仍 14 维")
               + f"；口径 {jaw_report['mass_basis']}）\n" if jaw_report is not None else "")
            + "  **故意没生成、不许当成已经对了**：\n"
            "    · <site> 位置沿用上游 —— 改传感器安装点会改观测，不在本脚本范围。\n"
            f"    · 质量未闭环（未计或沿用显式历史代理，不能当实重）：{', '.join(missing)}\n"
            "    · 碰撞几何是凸包；凹件会比真实网格胖 —— 保真度由 Gate 第 6 层核对，别默认它对。\n")

    OUT_DIR.mkdir(parents=True, exist_ok=True)
    out = OUT_DIR / "robot_walk_s288.xml"
    root.insert(0, _header("all"))
    ET.indent(root, space="  ")
    out.write_text('<?xml version="1.0" encoding="utf-8"?>\n'
                   + ET.tostring(root, encoding="unicode"), encoding="utf-8")

    # ── 训练变体：只保留脚底碰撞（与上游 robot_walk.xml 同构）──────────
    import copy as _copy
    troot = _copy.deepcopy(root)
    for c in [c for c in troot if isinstance(c.tag, type(ET.Comment)) and c.tag is ET.Comment]:
        troot.remove(c)
    foot_geom = {"col_sole": "left_foot_collision", "col_sole_R": "right_foot_collision"}
    # 自碰体（对应上游 robot_walk.xml 的 leg×2 + power_support 三个 self_collision_only 网格）：
    # 小腿两件 + 躯干框架 + 电池。contype=conaffinity=2：只和同类互碰，不碰地（地面 contype 1）。
    self_col_stems = {"lower_leg", "lower_leg_R", "trunk", "zz_battery"}
    n_removed = 0
    n_self = 0
    for b in troot.iter("body"):
        for gm in list(b.findall("geom")):
            if gm.get("class") != "collision":
                continue
            nm = gm.get("name", "")
            if nm in foot_geom:
                gm.set("name", foot_geom[nm])
            elif re.sub(r"_\d+$", "", nm[4:]) in self_col_stems:
                gm.set("class", "self_collision_only"); n_self += 1
            else:
                gm.set("class", "visual"); n_removed += 1          # 留作纯视觉（contype/conaffinity 0），play 时能看见
    kept = sorted(gm.get("name") for gm in troot.iter("geom") if gm.get("class") == "collision")
    if kept != ["left_foot_collision", "right_foot_collision"]:
        notes.append(f"训练变体脚底碰撞几何不对：{kept}")
    self_bodies = sorted({b.get("name") for b in troot.iter("body")
                          for gm in b.findall("geom") if gm.get("class") == "self_collision_only"})
    if self_bodies != ["leg", "leg_2", "trunk_base"]:
        notes.append(f"训练变体自碰体所在 body 不对：{self_bodies}（期望 leg/leg_2/trunk_base）")
    troot.insert(0, _header("train"))
    ET.indent(troot, space="  ")
    out_train = OUT_DIR / "robot_walk_s288_train.xml"
    out_train.write_text('<?xml version="1.0" encoding="utf-8"?>\n'
                         + ET.tostring(troot, encoding="unicode"), encoding="utf-8")

    # ── 全碰撞变体（起立/坐站/翻身/捡物/踢球）：对应上游 robot_allcollisions.xml 的 11 个 collision 网格 ──
    #    地面接触件 = 躯干框架 + 两半壳 + 电池门（上游 np_f970 一件）、髋件×2、小腿×2、脚底×2、三片头壳（上壳/下巴/下壳）。
    #    class collision（contype=conaffinity=1）：碰地、也互碰（起立时头/壳撑地、腿顶躯干）；父子 body 之间 MuJoCo 默认不碰。
    #    电池仍 self_collision_only；其余件（大腿、偏航杯、颈、头支架、舵机、轴承）纯视觉 —— 与上游取舍一致。
    aroot = _copy.deepcopy(root)
    for c in [c for c in aroot if c.tag is ET.Comment]:
        aroot.remove(c)
    ground_col_stems = {"trunk", "shell_L", "shell_R", "battery_door", "hip", "hip_R", "lower_leg", "lower_leg_R",
                        "head_top_shell", "jaw", "head_bottom_shell"}      # hr38
    a_kept, a_vis, a_self = [], 0, 0
    for b in aroot.iter("body"):
        for gm in list(b.findall("geom")):
            if gm.get("class") != "collision":
                continue
            nm = gm.get("name", "")
            stem = re.sub(r"_\d+$", "", nm[4:])
            if nm in foot_geom:
                gm.set("name", foot_geom[nm]); a_kept.append(foot_geom[nm])
            elif stem in ground_col_stems:
                a_kept.append(nm)
            elif stem == "zz_battery":
                gm.set("class", "self_collision_only"); a_self += 1
            else:
                gm.set("class", "visual"); a_vis += 1
    a_bodies = sorted({b.get("name") for b in aroot.iter("body") for gm in b.findall("geom") if gm.get("class") == "collision"})
    # hr41 落盘 2026-09-25（Lane C）：J01（stem jaw）的碰撞凸包现在挂在子 body "jaw" 上
    if a_bodies != sorted(["ankle_left", "ankle_right", "hip_l", "hip_l_2", "jaw_soft", "leg", "leg_2", "trunk_base"]
                          + ([JAW_BODY] if jaw_report is not None else [])):
        notes.append(f"全碰撞变体地面接触 body 不对：{a_bodies}")
    aroot.insert(0, _header("allcol"))
    ET.indent(aroot, space="  ")
    out_allcol = OUT_DIR / "robot_walk_s288_allcol.xml"
    out_allcol.write_text('<?xml version="1.0" encoding="utf-8"?>\n'
                          + ET.tostring(aroot, encoding="unicode"), encoding="utf-8")

    # ── 回放变体：训练变体 + 整件网格视觉（按原版涂装配色）+ 相机"眼睛"。物理与训练变体完全相同
    #    （body/joint/inertial/碰撞几何逐字节同），只多 class=visual 的网格 —— 录像/演示用，Gate 不读。
    proot = _copy.deepcopy(troot)
    for c in [c for c in proot if c.tag is ET.Comment]:
        proot.remove(c)
    n_hull_vis = 0
    for b in proot.iter("body"):
        for gm in list(b.findall("geom")):
            if gm.get("class") == "visual":
                b.remove(gm); n_hull_vis += 1
    passet = proot.find("asset")
    for mat, rgba in PLAY_PALETTE.items():
        ET.SubElement(passet, "material", {"name": f"mat_{mat}", "rgba": rgba})
    bodies_by_name = {b.get("name"): b for b in proot.iter("body")}
    for old_vis in ASSETS.glob("vis_*.stl"):
        old_vis.unlink()
    for body_name, stem, loc in vis_meshes:
        loc.export(str(ASSETS / f"vis_{stem}.stl"))
        ET.SubElement(passet, "mesh", {"name": f"vis_{stem}", "file": f"vis_{stem}.stl"})
        ET.SubElement(bodies_by_name[body_name], "geom", {
            "type": "mesh", "name": f"vis_{stem}", "class": "visual", "mesh": f"vis_{stem}",
            "material": f"mat_{play_material_for(stem)}"})
    # 眼睛 = 头部相机镜头环（原版是黄圈 + 深色镜头），放在 head_camera site 处、朝向同 site
    eye_site = next((st for st in proot.iter("site") if st.get("name") == "head_camera"), None)
    if eye_site is not None:
        eye_body = next(b for b in proot.iter("body") if eye_site in list(b))
        # 镜头盘的法线要朝机器人前方（世界 +x）。jaw_soft 的 body 轴在 HOME 是 x→世界 z、z→世界 −x
        # （home_pose_render.py 打印过），所以圆柱轴（局部 z）绕 y 转 180° 变成 body −z = 世界 +x；
        # 位置沿 body −z 再挪 2 mm 让盘凸出面板。
        px, py, pz = (float(v) for v in eye_site.get("pos").split())
        pos = f"{px:.6g} {py:.6g} {pz - 0.002:.6g}"
        ET.SubElement(eye_body, "geom", {"type": "cylinder", "name": "vis_eye_ring", "class": "visual",
                                         "size": "0.012 0.0012", "pos": pos, "quat": "0 0 1 0", "material": "mat_orange"})
        ET.SubElement(eye_body, "geom", {"type": "cylinder", "name": "vis_eye_lens", "class": "visual",
                                         "size": "0.0065 0.0018", "pos": pos, "quat": "0 0 1 0", "material": "mat_dark"})
    else:
        notes.append("回放变体：没找到 head_camera site，没画眼睛")
    proot.insert(0, _header("play"))
    ET.indent(proot, space="  ")
    out_play = OUT_DIR / "robot_walk_s288_play.xml"
    out_play.write_text('<?xml version="1.0" encoding="utf-8"?>\n'
                        + ET.tostring(proot, encoding="unicode"), encoding="utf-8")

    (OUT_DIR / "build_report.json").write_text(json.dumps({
        "source_skeleton": "upstream/microduck_rl/.../robot_walk.xml（未修改）",
        "density_g_cm3": DENSITY_G_CM3,
        "bodies": {k: {"mass_g": round(v[0], 4),
                       "com_local_m": [round(float(x), 8) for x in v[1]],
                       "fullinertia_kgm2": [round(float(x), 12) for x in
                                            (v[2][0, 0], v[2][1, 1], v[2][2, 2],
                                             v[2][0, 1], v[2][0, 2], v[2][1, 2])]}
                   for k, v in sorted(totals.items())},
        "total_mass_g": round(sum(v[0] for v in totals.values()), 4),
        "collision_hulls": hull_report,
        "ranges_changed": changed,
        "ranges_key": ranges_key,
        "point_masses": [{"id": c, "world_mm": list(x), "mass_g": g, "body": b, "src": src} for c, x, g, b, src in point_masses],
        "mass_proxies": mass_proxies,
        "partial_masses": partial_masses,
        "mass_inventory_complete": not missing,
        "actuator_defaults": {k: float(v) for k, v in act_vals.items()},
        "actuator_src": str(S288_BAM_M1.relative_to(ROOT)),
        "train_variant": {"file": "robot_walk_s288_train.xml", "collision_geoms": kept,
                          "self_collision_only_hulls": n_self, "self_collision_bodies": self_bodies,
                          "demoted_to_visual": n_removed},
        "allcol_variant": {"file": "robot_walk_s288_allcol.xml", "collision_geoms": len(a_kept),
                           "collision_bodies": a_bodies, "ground_col_stems": sorted(ground_col_stems),
                           "self_collision_only_hulls": a_self, "demoted_to_visual": a_vis},
        "missing_mass_components": missing,
        "jaw_chain": jaw_report,                                    # hr41 落盘 2026-09-25（Lane C）：嘴关节链与 m14 质量口径
        "notes": notes,
    }, ensure_ascii=False, indent=1), encoding="utf-8")

    print(f"→ {out.relative_to(ROOT)}")
    print(f"   {len(totals)} 个 body，整机 {sum(v[0] for v in totals.values()):.1f} g，"
          f"碰撞几何 {sum(len(v) for v in per_body.values())} 件")
    print(f"   关节区间收窄 {len(changed)} 条（{ranges_key}）：{'; '.join(changed) or '无'}")
    print(f"   执行器默认 ← {S288_BAM_M1.relative_to(ROOT)}：{ {k: round(v, 5) for k, v in act_vals.items()} }")
    print(f"→ {out_train.relative_to(ROOT)}（训练变体：碰撞几何 {kept}，自碰体 {n_self} 个凸包 @ {self_bodies}，其余 {n_removed} 个凸包改为纯视觉）")
    print(f"→ {out_allcol.relative_to(ROOT)}（全碰撞变体：{len(a_kept)} 个碰撞凸包 @ {a_bodies}，电池自碰 {a_self}，其余 {a_vis} 个纯视觉）")
    for n in notes:
        print(f"   [注意] {n}")

    # ── 编译验证 ────────────────────────────────────────────────────────
    try:
        import mujoco
        for f in (out, out_train, out_allcol):
            mdl = mujoco.MjModel.from_xml_path(str(f))
            print(f"   MuJoCo 编译通过 {f.name}：{mdl.nbody} body / {mdl.njnt} joint / "
                  f"{mdl.ngeom} geom / {mdl.nu} actuator，模型总质量 "
                  f"{sum(mdl.body_mass):.4f} kg")
    except Exception as e:                                          # noqa: BLE001
        print(f"   **MuJoCo 编译失败**：{e}")
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
