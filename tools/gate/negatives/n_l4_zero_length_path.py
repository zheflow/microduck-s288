#!/usr/bin/env python3
"""反例 FG11 —— 路径长度填 0（以及行程不足以抽出、拆卸失败却继续假定已拆）也能全绿。

一套内存几何跑三种路径长（**不碰 cad/duck_s288 的整鸭件**）：
    移动件 MV     1 mm 立方  x/y/z ∈ [−0.5, 0.5]
    本步障碍 SL1  套筒       x ∈ [ 3, 10]，y/z ∈ [−3, 3]，中心有 Ø2.4 的通孔（移动件穿得过去）
    已装障碍 SL2  套筒       x ∈ [−10, −3]，同样带通孔
    方向 +x，步长 0.25

  A 坏样本 len_mm = 0    修前：0 mm³ + **4 次证据**（1 个采样 × 2 个障碍 × 2 个符号）→ 判绿。
                          修后：长度 ≤ 0 直接 BLOCK —— 移动件根本没动过，交集必然 0，那是没查。
  B 坏样本 len_mm = 1    修前：0 mm³ → 判绿。修后：走完 1 mm 移动件还在两个套筒的"影子"里，
                          既没沿方向越过、⊥方向也没分离 → **没有证明抽得出来** → BLOCK。
  C 好样本 len_mm = 12   穿过通孔走到 x=12.5，已越过 SL1 的 x=10 → 判据必须仍然绿（防恒红）。

拆卸序（FG11 的第三条）：disasm01 拆 MV、disasm02 拆 SL1。
    修前：先把 MV 从障碍集合里划掉再扫 —— MV 拆不出来，disasm02 仍按"MV 已拆走"判绿。
    修后：只有 disasm01 **判定成功**才允许迁移状态；失败时 disasm02 挂 `disassembly_prereq`（BLOCK）。

判据阈值取真实的 tools/gate/data/tolerances.yaml，本文件不写死任何数字判据。
"""
from __future__ import annotations
import sys
from pathlib import Path

_HERE = Path(__file__).resolve()
GATE = _HERE.parents[1]
for _p in (str(GATE), str(GATE.parents[1])):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import core                                                        # noqa: E402

NAME = "路径长度 0 / 行程不足以抽出 / 拆卸失败仍假定已拆（FG11）"
EXPECT = ("L4/step01:step_sweep 在 len_mm=0 和 len_mm=1 时必须红、len_mm=12 时必须绿；"
          "disasm01 失败时 L4/disasm02:disassembly_prereq 必须红")
_TMP = core.ROOT / "tools/gate/out/_neg_tmp"


def _sleeve(x0, x1):
    """带通孔的套筒：移动件从孔里穿得过去（交集恒 0），但它的"影子"罩住移动件 ——
    走得不够远就没法证明抽出来了。"""
    import numpy as np
    import trimesh
    b = trimesh.creation.box(extents=(x1 - x0, 6.0, 6.0))
    b.apply_translation(np.array([(x0 + x1) / 2.0, 0.0, 0.0]))
    bore = trimesh.creation.cylinder(radius=1.2, height=(x1 - x0) * 3, sections=48)
    bore.apply_transform(trimesh.transformations.rotation_matrix(np.pi / 2, [0, 1, 0]))
    bore.apply_translation(np.array([(x0 + x1) / 2.0, 0.0, 0.0]))
    return trimesh.boolean.difference([b, bore])


def _scene():
    import numpy as np
    import trimesh
    d = _TMP / "l4_zero_len"
    d.mkdir(parents=True, exist_ok=True)
    for p in d.glob("*.stl"):
        p.unlink()
    mv = trimesh.creation.box(extents=(1.0, 1.0, 1.0))
    mv.export(str(d / "mover.stl"), file_type="stl")
    _sleeve(3.0, 10.0).export(str(d / "sleeve1.stl"), file_type="stl")
    _sleeve(-10.0, -3.0).export(str(d / "sleeve2.stl"), file_type="stl")
    _ = np
    return d


def _ctx(placed_dir, length):
    from _harness import FakeCtx                                   # noqa: PLC0415
    data = core.load_data()                                        # 真公差表
    data["parts"] = {"parts": [
        {"id": "MV", "inventory_id": "mover", "build_fn": [], "qty": 1, "placed_instances": ["mover"]},
        {"id": "SL1", "inventory_id": "sleeve1", "build_fn": [], "qty": 1, "placed_instances": ["sleeve1"]},
        {"id": "SL2", "inventory_id": "sleeve2", "build_fn": [], "qty": 1, "placed_instances": ["sleeve2"]},
    ]}
    data["features"] = {"features": [], "frames": {"per_part": {}}}
    data["fasteners"] = {"fasteners": []}
    data["keepouts"] = {"keepouts": []}
    data["waivers"] = {"waivers": []}
    data["assembly"] = {
        "assembly_order": [{
            "step": 1,
            "action": "反例：MV 沿 +x 穿过两个套筒的通孔",
            "parts": ["MV", "SL1"],
            "already_installed": ["SL2"],
            "direction": "+x",
            "path": {"kind": "linear", "len_mm": float(length), "step_mm": 0.25},
            "verified": True,
            "verified_by": "反例自造",
        }],
        "disassembly_order": [
            {"seq": 1, "action": "拆 MV（沿 +x 抽出）", "reverse_of_step": 1, "proven": False},
            {"seq": 2, "action": "拆 SL1（沿 +x 抽出）", "reverse_of_step": 1, "proven": False},
        ],
    }
    return FakeCtx(data, {}, {p.stem: p for p in sorted(placed_dir.glob("*.stl"))})


def _run_layer(placed, length):
    import layers.l4_assembly as L4                                # noqa: PLC0415
    old = L4.PLACED
    try:
        L4.PLACED = placed
        return L4.run(_ctx(placed, length))
    finally:
        L4.PLACED = old


def run() -> dict:
    from _harness import findings, worst_state, summarize, assert_red, assert_green, result   # noqa: PLC0415
    placed = _scene()
    out, ev, fs_of = {}, {}, {}
    for tag, length in (("len0", 0.0), ("len1", 1.0), ("len12", 12.0)):
        r = _run_layer(placed, length)
        fs = findings(r, subject="step01", check="step_sweep")
        fs_of[tag] = fs
        out[tag] = worst_state(fs)
        # 连整格一起记：修好之前 step_sweep 这条判据根本不存在，格子里只有分组诊断的假绿，
        # 这份摘要就是"修前绿"的证据（len0 那格应当是 0 mm³ / 一共 4 次证据）
        ev[tag] = summarize(fs, n=2) + " ｜整格：" + summarize(findings(r, subject="step01"), n=6)
        if tag == "len1":
            pre = findings(r, subject="disasm02", check="disassembly_prereq")
            fs_of["prereq"] = pre
            out["prereq"] = worst_state(pre)
            ev["prereq"] = summarize(pre, n=1)
            ev["disasm01"] = summarize(findings(r, subject="disasm01", check="disassembly_sweep"), n=1)
        if tag == "len12":
            out["prereq_ok"] = worst_state(findings(r, subject="disasm02", check="disassembly_prereq"))
            ev["disasm02_ok"] = summarize(findings(r, subject="disasm02"), n=3)

    # len=0：长度 ≤ 0 直接 BLOCK。层可能以 unknown（measured=None）或带数的 FAIL 发出 —— 两种都算红，
    # 但只把带 measured 的记录放进 red；len=1 与 disassembly_prereq 必须带数。
    ok0, why0, red0 = assert_red(fs_of["len0"], severity="BLOCK", require_measured=False)
    ok1, why1, red1 = assert_red(fs_of["len1"], severity="BLOCK", require_measured=True)
    okp, whyp, redp = assert_red(fs_of["prereq"], severity="BLOCK", require_measured=True)
    ok12, why12 = assert_green(fs_of["len12"])
    red = [x for x in red0 if x["measured"] is not None] + red1 + redp
    passed = ok0 and ok1 and okp and ok12 and out["prereq_ok"] == "ABSENT" and bool(red)
    return result(
        NAME, EXPECT, passed,
        got=(f"len=0 → {'红' if ok0 else why0}（{[x['measured'] for x in red0]}）；len=1 → {'红' if ok1 else why1}；"
             f"len=12（好样本）→ {'绿' if ok12 else why12}；"
             f"disasm01 失败时 disasm02:disassembly_prereq → {'红' if okp else whyp}；"
             f"disasm01 成功时 → {out['prereq_ok']}（应当没有这条）"),
        red=red, expect_severity="BLOCK",
        detail=(f"len0: {ev['len0']} || len1: {ev['len1']} || len12: {ev['len12']} || "
                f"disasm01(len1): {ev['disasm01']} || prereq: {ev['prereq']}"))


if __name__ == "__main__":
    import json
    r = run()
    print(json.dumps(r, ensure_ascii=False, indent=1))
    sys.exit(0 if r["passed"] else 1)
