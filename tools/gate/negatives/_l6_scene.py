#!/usr/bin/env python3
"""L6 反例共用脚手架（不是反例本身：文件名不以 n_ 开头，runner 不收）。

L6 是整机层：真 placed/ 有 50 只实体，单轴 + 两两组合要跑几万次布尔，反例跑不动。这里提供三样东西，
让反例在 < 5 s 内**经过 `l6_motion.run(ctx)`** 并把断言落在 run 发出的 finding 上：

  · `dummy_placed(tag)`  —— parts.yaml:motion_instances 声明的每个 stem 一只 1 mm 立方（清单完整、场景建得起来），
                            调用方可以把其中几只换成自己造的解析几何；
  · `scene_class(keep=, hit_fn=)` —— `_Scene` 子类：`keep` 只保留这几只实体参与运动判据（其余留在清单里但不进配对），
                            `hit_fn(pose, stem_a, stem_b) -> 干涉体积` 给了就用合成 evaluate 代替布尔
                            （只验网格 / 分档 / 区间逻辑时用；验真几何间隙时不给 hit_fn）；
  · `run_l6(data, placed_dir, scene_cls, gen_mjcf=)` —— 猴子补丁 PLACED / _Scene / _GEN_MJCF / _RANGES_OUT 跑一遍层，
                            **跑完还原**，并把层导出的 mjcf_ranges.json 引到临时文件（旧版层没有 _RANGES_OUT 这个名字，
                            会把合成结果写进真 tools/gate/out/mjcf_ranges.json —— 这里备份/还原兜底）。
  · `restrict_joints(data, keep)` —— frozen 深拷贝，只让这几条关节 exists_as_mjcf_joint（其余关节不扫，省时间；
                            joint_inventory 会因 ≠14 而红，调用方不要对它断言）。
  · `gen_mjcf_with_ranges(tag, ranges_deg)` —— 以真 sim/duck_s288/robot_walk_s288.xml 为底，改写指定关节的
                            <joint range>（度→弧度）写成临时 XML，给 collision_free_range 那两条判据当"生成的 MJCF"。
"""
from __future__ import annotations
import math
import os
import re
import sys
from copy import deepcopy
from pathlib import Path

_HERE = Path(__file__).resolve()
GATE = _HERE.parents[1]
for _p in (str(_HERE.parent), str(GATE), str(GATE / "layers"), str(GATE.parents[1]),
           str(GATE.parents[1] / "tools/sim"), str(GATE.parents[1] / "tools/cad")):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import core                                                     # noqa: E402
from _harness import FakeCtx                                    # noqa: E402
from n_l6_scene_inventory import stems_declared, make_dummy_placed   # noqa: E402

_TMP = core.ROOT / "tools/gate/out/_neg_tmp"
_MINE = _TMP / f"p{os.getpid()}"
_REAL_RANGES = core.ROOT / "tools/gate/out/mjcf_ranges.json"
_REAL_GEN = core.ROOT / "sim/duck_s288/robot_walk_s288.xml"


def dummy_placed(tag, data=None):
    data = data or core.load_data()
    return make_dummy_placed(tag, stems_declared(data))


def scene_class(keep=None, hit_fn=None):
    import l6_motion as L6

    class _NegScene(L6._Scene):
        def __init__(self, ctx, bfp):
            super().__init__(ctx, bfp)
            if keep is not None:
                idx = [i for i, n in enumerate(self.names) if n in keep]
                self.names = [self.names[i] for i in idx]
                self.bodies = [self.bodies[i] for i in idx]
                self.solids = [self.solids[i] for i in idx]
                self.corners = self.corners[idx]

        if hit_fn is not None:
            def evaluate(self, pose_deg, pairs, tol, gap_tol=None, mating=frozenset()):
                hits = []
                for i, k in pairs:
                    v = float(hit_fn(pose_deg, self.names[i], self.names[k]) or 0.0)
                    if v > tol:
                        hits.append((self.names[i], self.names[k], v))
                self.booleans += len(pairs)
                return hits, [], len(pairs)

    return _NegScene


def run_l6(data, placed_dir, scene_cls=None, gen_mjcf=None):
    import l6_motion as L6
    names = ("PLACED", "_Scene", "_GEN_MJCF", "_RANGES_OUT")
    saved = {k: getattr(L6, k) for k in names if hasattr(L6, k)}
    before = _REAL_RANGES.read_bytes() if _REAL_RANGES.exists() else None
    _MINE.mkdir(parents=True, exist_ok=True)
    try:
        L6.PLACED = Path(placed_dir)
        if scene_cls is not None:
            L6._Scene = scene_cls
        if gen_mjcf is not None:
            L6._GEN_MJCF = Path(gen_mjcf)
        if hasattr(L6, "_RANGES_OUT"):
            L6._RANGES_OUT = _MINE / "mjcf_ranges.neg.json"
        return L6.run(FakeCtx(data, {}, {p.stem: p for p in Path(placed_dir).glob("*.stl")}))
    finally:
        for k, v in saved.items():
            setattr(L6, k, v)
        if before is not None:
            if not _REAL_RANGES.exists() or _REAL_RANGES.read_bytes() != before:
                _REAL_RANGES.write_bytes(before)
        elif _REAL_RANGES.exists():
            _REAL_RANGES.unlink()


def restrict_joints(data, keep):
    d = deepcopy(data)
    for j in d["frozen"]["joint_axes"]:
        if j.get("name") not in keep:
            j["exists_as_mjcf_joint"] = False
    return d


def joint_axis(name):
    j = next(x for x in core.load_data()["frozen"]["joint_axes"] if x["name"] == name)
    return j


def gen_mjcf_with_ranges(tag, ranges_deg: dict):
    """以真生成的 MJCF 为底改写 <joint range>。ranges_deg = {joint: (lo_deg, hi_deg)}。"""
    txt = _REAL_GEN.read_text(encoding="utf-8")
    n = 0
    for nm, (lo, hi) in ranges_deg.items():
        pat = re.compile(r'(<joint\b[^>]*\bname="' + re.escape(nm) + r'"[^>]*\brange=")([^"]*)(")')
        txt, k = pat.subn(lambda m: f"{m.group(1)}{math.radians(lo):.10g} {math.radians(hi):.10g}{m.group(3)}", txt)
        n += k
    _MINE.mkdir(parents=True, exist_ok=True)
    p = _MINE / f"robot_walk_s288.{tag}.xml"
    p.write_text(txt, encoding="utf-8")
    return p, n


def upstream_ranges_deg(data=None):
    """frozen.yaml:joint_axes[].target_range_deg.v（现在全部 = 上游值）→ {joint: (lo, hi)}。"""
    data = data or core.load_data()
    out = {}
    for j in data["frozen"]["joint_axes"]:
        t = (j.get("target_range_deg") or {}).get("v")
        if j.get("exists_as_mjcf_joint") and isinstance(t, list) and len(t) == 2:
            out[j["name"]] = (float(t[0]), float(t[1]))
    return out
