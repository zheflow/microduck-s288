#!/usr/bin/env python3
"""反例 FG15 —— 第 9 颗螺丝的坐面是一块**等厚斜面**；坐面只按声明的头侧判（F-L5-2）。

坑的形状（两层叠加，2026-09-09）：
  · `l2_features.seat_spread` 量的是**厚度极差**，不是头侧平面度 —— 等厚斜面沿孔轴的料厚
    处处相同，极差 = 0，`seat_flatness` 完美通过；
  · `l5_screwhead` 又只查每组**前 8 个孔**（`coords[:8]`），第 9 颗从头到尾没被摸过；
    而且叠厚对账 `stack_from_seat_crosscheck` 还**跨孔取中位数**，
    就算查到了，8 个平的也会把 1 个斜的压下去。

第三层坑（2026-09-13 审计 F-L5-2）：头侧不知道朝哪边，于是 l2 `seat_profile` "返回较差的那一端"、
l5_screwhead `worst = max(ends, …)` —— 远端（螺丝尖那一侧）有台阶的孔被判成"坐面不平"，红的是对面。
且 l5_function `_screw_holes` 把件上所有 Ø(m_through±0.1) 的孔都当螺丝孔扫一遍，跟螺丝组无关。

正确判据：逐颗真实螺丝头足印判**头侧承压面平面度 + 有效承压面积 + 该孔自己的叠厚**，
不跨孔取统计量，不限前 8 个；**头侧只从 fasteners.yaml:tool_access[].seats[].outward_export_local 取**
（按 feature_id + map_index + hole_index 对孔），取不到头侧 → unknown，不取较差端；
坐面按 fasteners 的穿件孔逐组判，不按直径扫全件。

  A 坏样本   第 9 颗坐在等厚斜面上、头侧 +z 已声明 → head_seat_flatness / seat_flatness 红（BLOCK）
  B 对照     9 个平坐面、头侧已声明 → 绿
  C 远端台阶 第 9 颗头侧（+z）平、远端（−z）足印下有 0.8 深的小坑、头侧已声明 → 头侧平面度必须绿
             （修前"取较差端"→ 红）；叠厚极差那条（seat_stack_spread）照旧红 —— 料厚真的变了，那是另一条判据
  D 未声明   同 C 几何但没有 tool_access → head_seat_flatness / seat_flatness 必须 unknown（measured 为空），不许猜一端
  E 坐面点在料里（2026-09-13 加）第 9 颗的 point_export_local 写成 [x, y, T]（本反例 09-13 前的写法：那时层不读坐面点），
             它落在斜面板的料里 → head_seat 必须 unknown（坐面声明与几何不符），不许报成"头悬空"或判绿

2026-09-13 起头判据的**起点**是 tool_access 坐面点（feature_hole_map 的孔点是刀心，只当轴线锚），逐孔判据名改为
head_seat_flatness:hole<map_index>.<hole_index>。
"""
from __future__ import annotations

import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from _harness import FakeCtx, assert_green, assert_red, base_data, box, cyl, export_stl, findings, record, result  # noqa: E402

PART = "N03"
FID = "N03-FG15"
GID = "FG15_nine_holes"
NAME = "第 9 颗螺丝坐在等厚斜面上：逐孔头侧承压面必须红，9 个平坐面必须绿；远端台阶按声明头侧判绿；头侧未声明 → unknown"
BAD = 8                                   # 第 9 颗（0 起数）
EXPECT = (f"L5/{GID}:head_seat_flatness:hole0.{BAD} 与 L5/{GID}:seat_flatness（等厚斜面：厚度极差 0、头侧平面度 ≈1.0）红；"
          f"远端台阶 + 声明头侧 → 绿；未声明头侧 → unknown；坐面点在料里 → head_seat unknown")

T = 3.6
TILT_DEG = 15.0
FLAT = [(-28.0 + 4.0 * i, 0.0) for i in range(8)]     # 8 个平坐面孔
POCKET = (1.2, 1.2, 0.8)                              # C/D：远端（底面）足印下的小坑（x 偏 +1.6，只压住几个采样点）


def _tilt_center(bad: bool = True):
    a = math.radians(TILT_DEG) if bad else 0.0
    return 14.0 + 9.0 * math.cos(a), 0.0, T / 2 + 9.0 * math.sin(a)


def _part(bad: bool, far_step: bool = False):
    import trimesh
    base = box(56.0, 12.0, T, center=(-12.0, 0.0, T / 2))
    slab = box(18.0, 12.0, T, center=(9.0, 0.0, 0.0))
    if bad:      # 等厚斜面：沿 z 的料厚处处 T/cos15°，厚度极差 = 0，但头侧面歪 15°
        slab.apply_transform(trimesh.transformations.rotation_matrix(
            -math.radians(TILT_DEG), [0, 1, 0]))
    slab.apply_translation([14.0, 0.0, T / 2])
    m = base.union(slab)
    for x, y in FLAT:
        m = m.difference(cyl(1.1, 40.0, center=(x, y, 0.0)))
    cx, cy, _cz = _tilt_center(bad)
    m = m.difference(cyl(1.1, 40.0, center=(cx, cy, 0.0)))
    if far_step:
        m = m.difference(box(*POCKET, center=(cx + 1.6, cy, POCKET[2] / 2)))
    return m


def _holes(bad: bool = True):
    cx, cy, cz = _tilt_center(bad)
    return [[x, y, T / 2] for x, y in FLAT] + [[cx, cy, cz]]


def _seat_points(bad: bool, in_material: bool = False):
    """真坐面点（料→空面在孔轴上的位置）：平坐面在 z=T；斜面第 9 颗在 cz + (T/2)/cos θ（板的上表面）。
    in_material=True 复现 09-13 前的写法 [x, y, T] —— 对斜面那颗它在料里。"""
    cx, cy, cz = _tilt_center(bad)
    pts = [[x, y, T] for x, y in FLAT]
    if bad and not in_material:
        pts.append([cx, cy, cz + (T / 2) / math.cos(math.radians(TILT_DEG))])
    else:
        pts.append([cx, cy, T])
    return pts


def _data(bad: bool = True, declare_head: bool = True, seat_in_material: bool = False):
    part = {"id": PART, "inventory_id": "N03_fg15", "material": "PLA",
            "print_orientation": "背板朝下", "mirrored_copy": None}
    feat = {"id": FID, "part": PART, "kind": "screw_hole", "check_class": "screw_hole",
            "count": 9, "spec_verbatim": "9×Ø2.2 通（第 9 颗坐在等厚斜面上）",
            "geom": {"shape": "cylinder", "frame": "export_local",
                     "nominal_d_mm": {"v": 2.2, "src": "measured"},
                     "through": True, "axis": "z", "depth_kind": "cutter_length",
                     "depth_mm": {"v": 40.0, "src": "assumed"},
                     "pos": [0.0, 0.0, T / 2],
                     "hole_positions_mm": _holes(bad)}}
    holes = _holes(bad)
    fa = {"id": GID, "spec": "M2×6", "qty": 9,
          "joins": ["N03 背板", "servo_head_yaw"],
          "joint_type": "s288_metal_thread",
          "engagement_mm": {"v": 2.4, "src": "measured"},
          "stack_mm": {"v": 3.6, "src": "measured"},
          "status": "ok", "status_verbatim": "ok",
          "feature_ids": [FID],
          "feature_hole_map": [{"feature_id": FID, "instance": 0,
                                "instance_ref": "反例：9 个孔，第 9 个坐面是等厚斜面",
                                "holes": holes, "n": 9}],
          "head_locator_map_indices": [0],
          "provenance": ["negatives/n_l5_fg15"]}
    if declare_head:
        pts = _seat_points(bad, seat_in_material)
        fa["tool_access"] = [{"state_ref": "tool:neg",
                              "seats": [{"feature_id": FID, "instance": "N03_fg15",
                                         "point_export_local": pts[i], "outward_export_local": [0, 0, 1],
                                         "map_index": 0, "hole_index": i} for i, h in enumerate(holes)]}]
    return base_data([part], [feat], fasteners=[fa], relations=[])


def _observe(bad: bool, far_step: bool = False, declare_head: bool = True, tag: str = "", seat_in_material: bool = False,
             with_function: bool = True):
    import l2_features
    import l5_function
    import l5_screwhead
    l2_features._GEOM_CACHE.clear()
    stl = export_stl(_part(bad, far_step), f"fg15_{tag}")
    data = _data(bad, declare_head, seat_in_material)
    rh = l5_screwhead.run(FakeCtx(data, {PART: stl}))
    rf = l5_function.run(FakeCtx(data, {PART: stl})) if with_function else rh.__class__(5, "skip")
    head = [f for f in findings(rh, subject=GID)
            if f.check.split(":")[0] in ("head_seat_flatness", "head_side", "head_seat", "head_seat_exists")]
    seat = [f for f in rf.findings if f.check.startswith("seat_flatness") and f.subject in (GID, PART)]
    spread = [f for f in rf.findings if f.check == "seat_stack_spread" and f.subject in (GID, PART)]
    return dict(head=head, seat=seat, spread=spread, probed=rh.evidence.get("holes_probed"),
                summary={"探到的孔数": rh.evidence.get("holes_probed"), "声明孔数": 9,
                         "螺丝头逐孔承压面": {f.check: f"{f.state}/{f.measured}" for f in head},
                         "坐面": {f.check: f"{f.state}/{f.measured}" for f in seat},
                         "叠厚极差": {f.check: f"{f.state}/{f.measured}" for f in spread}})


def _is_unknown(fs):
    return bool(fs) and all(f.state == "FAIL" and f.measured is None for f in fs)


def run() -> dict:
    A = _observe(True, tag="sloped")
    B = _observe(False, tag="flat")
    C = _observe(False, far_step=True, tag="farstep")
    D = _observe(False, far_step=True, declare_head=False, tag="farstep_undeclared")
    E = _observe(True, tag="sloped_seat_in_material", seat_in_material=True, with_function=False)   # E 只看 l5_screwhead，省 1 s
    okA_h, whyA_h, redA_h = assert_red(A["head"], severity="BLOCK", check=f"head_seat_flatness:hole0.{BAD}")
    okA_s, whyA_s, redA_s = assert_red(A["seat"], severity="BLOCK", check="seat_flatness")
    okB_h, whyB_h = assert_green(B["head"], check="head_seat_flatness")
    okB_s, whyB_s = assert_green(B["seat"], check="seat_flatness")
    okC_h, whyC_h = assert_green(C["head"], check="head_seat_flatness")
    okC_s, whyC_s = assert_green(C["seat"], check="seat_flatness")
    okD_h = _is_unknown([f for f in D["head"] if f.check.startswith("head_seat_flatness") or f.check.startswith("head_side")])
    okD_s = _is_unknown([f for f in D["seat"] if f.check == "seat_flatness"])
    okE = _is_unknown([f for f in E["head"] if f.check == "head_seat"]) and \
        not any(f.check in ("head_seat_exists", "head_seat_flatness") for f in E["head"]) and \
        any("料里" in str(f.detail) for f in E["head"] if f.check == "head_seat")
    checks = [(f"A 斜坐面：head_seat_flatness:hole0.{BAD} FAIL(BLOCK)", okA_h),
              ("A 斜坐面：seat_flatness FAIL(BLOCK)", okA_s),
              ("B 全平：head_seat_flatness PASS", okB_h), ("B 全平：seat_flatness PASS", okB_s),
              ("C 远端台阶 + 声明头侧：head_seat_flatness PASS（不取较差端）", okC_h),
              ("C 远端台阶 + 声明头侧：seat_flatness PASS（头侧平面度按声明端）", okC_s),
              ("D 未声明头侧：head_seat_flatness/head_side 全部 unknown", okD_h),
              ("D 未声明头侧：seat_flatness unknown", okD_s),
              ("E 坐面点在料里：head_seat unknown（点名『料里』），不发 head_seat_exists/flatness", okE),
              ("A 探到 9 个孔", A["probed"] == 9)]
    failed = [c for c, ok in checks if not ok]
    red = redA_h + redA_s + [record(f) for f in D["seat"] if f.check == "seat_flatness" and f.state == "FAIL"]
    got = (f"A 斜坐面 head={ {k: v for k, v in A['summary']['螺丝头逐孔承压面'].items() if 'hole' in k or k == 'head_seat_flatness'} } "
           f"seat={A['summary']['坐面']}；B 全平 seat={B['summary']['坐面']}；"
           f"C 远端台阶 head={ {k: v for k, v in C['summary']['螺丝头逐孔承压面'].items() if k == 'head_seat_flatness'} } "
           f"seat={C['summary']['坐面']} 叠厚极差={C['summary']['叠厚极差']}；"
           f"D 未声明 head={ {k: v for k, v in D['summary']['螺丝头逐孔承压面'].items()} } seat={D['summary']['坐面']}；"
           f"E 坐面点在料里 head={ {k: v for k, v in E['summary']['螺丝头逐孔承压面'].items()} }")
    return result(NAME, EXPECT, not failed, got + (f"｜未成立：{failed}" if failed else ""), red,
                  expect_severity="BLOCK",
                  allow_unknown_red="D 验的是『头侧方向未声明 → unknown，不取较差端』这条设计路径；A 的 measured 非空",
                  detail=" | ".join(x for x in (whyA_h, whyA_s, whyB_h, whyB_s, whyC_h, whyC_s) if x))


if __name__ == "__main__":
    import json
    r = run()
    print(json.dumps(r, ensure_ascii=False, indent=1))
    sys.exit(0 if r["passed"] else 1)
