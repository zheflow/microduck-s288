#!/usr/bin/env python3
"""反例（09-12 清红第二批）—— 多段直线路径：只走第一段还在腔里、第二段太短、第二段穿墙，都必须红。

为什么要有多段：④ 的 N04 轴颈销从头偏航舵机腔沿 +x 插入，抽回腔内只有 12 mm 直线，之后必须改向
（−z）从腔口出来。单段直线永远证明不了它"真的抽出来了"，所以 judge_fixed_path 允许 segments。
这条反例保证：多段不是一张"随便走走就绿"的通行证 —— 终点脱离证明与逐段扫掠都还在。

**经过层的 `run_declared_motions`**（run() 对每个带 motions 的装配步调用的就是它）：每条路径作为一条
声明动作进 step 的 motions，红落在 `L4/step01:motion:<id>`（FAIL(BLOCK)，measured=峰值交集 mm³）。

内存几何（不碰整鸭件）：
    L 形通道块 BLK：x ∈ [−8, 10]，y/z ∈ [−3, 3]；水平孔 Ø2.4 沿 x 从 x=−6.7 到 +10（右端开口）；
    竖直孔 Ø2.4 沿 z 在 x=−5.5，从 z=0 打穿底面（z=−3）—— 肘弯朝下开口。
    移动件 MV：1 mm 立方，原位 x/y/z ∈ [−0.5, 0.5]（在水平孔里）。

  A 单段 −x 5.5           走到肘弯里（x=−5.5），沿 −x 前面是实心墙、⊥x 方向仍被块罩着 → 未证明脱离 → 红
  B 两段 −x 5.5 / −z 10   出竖直孔到 z=−9.5..−10.5，已越过块底 z=−3 → 绿（防恒红）
  C 两段 −x 5.5 / −z 2    第二段太短（还在竖直孔里）→ 未证明脱离 → 红
  D 两段 −x 3.0 / −z 10   第一段没走到肘弯，第二段直接切进块料 → 交集 > 阈值 → 红
  E 段声明坏（长度 0 / 方向 NaN / 空列表）→ 层判 unknown（motion:E* FAIL、measured=None），单独记录

阈值取真实 tolerances.yaml，本文件不写死判据数字。
"""
from __future__ import annotations
import sys
from pathlib import Path

_HERE = Path(__file__).resolve()
GATE = _HERE.parents[1]
for _p in (str(_HERE.parent), str(GATE), str(GATE / "layers"), str(GATE.parents[1])):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import core                                                        # noqa: E402

NAME = "多段路径：只走一段/第二段太短/第二段穿墙 必须红（FG11 的多段形式，经 run_declared_motions）"
EXPECT = "L4/step01:motion:A/C/D FAIL(BLOCK) 带峰值；motion:B PASS；E 坏声明 unknown"
_TMP = core.ROOT / "tools/gate/out/_neg_tmp"


def _scene():
    import numpy as np
    import trimesh
    from trimesh.transformations import rotation_matrix as rot
    d = _TMP / "l4_segments"
    d.mkdir(parents=True, exist_ok=True)
    for p in d.glob("*.stl"):
        p.unlink()
    blk = trimesh.creation.box(extents=(18.0, 6.0, 6.0))
    blk.apply_translation(np.array([1.0, 0.0, 0.0]))                      # x ∈ [−8, 10]
    hbore = trimesh.creation.cylinder(radius=1.2, height=20.0, sections=48)  # 沿 z → 转到 x
    hbore.apply_transform(rot(np.pi / 2, [0, 1, 0]))
    hbore.apply_translation(np.array([3.3, 0.0, 0.0]))                    # x ∈ [−6.7, 13.3]
    vbore = trimesh.creation.cylinder(radius=1.2, height=8.0, sections=48)
    vbore.apply_translation(np.array([-5.5, 0.0, -4.0]))                  # z ∈ [−8, 0]，穿底
    blk = trimesh.boolean.difference([blk, hbore, vbore])
    blk.export(str(d / "blk.stl"), file_type="stl")
    trimesh.creation.box(extents=(1.0, 1.0, 1.0)).export(str(d / "mover.stl"), file_type="stl")
    return d


def _motion(mid, **kw):
    return dict(id=mid, movers=["mover"], present=["mover", "blk"], workspace="test",
                sense="withdrawal_from_assembled", step_mm=0.25, **kw)


def run() -> dict:
    from _harness import FakeCtx, findings, assert_red, assert_green, result, record   # noqa: PLC0415
    import layers.l4_assembly as L4                                # noqa: PLC0415
    placed = _scene()
    old = L4.PLACED
    try:
        L4.PLACED = placed
        ctx = FakeCtx(core.load_data(), {}, {p.stem: p for p in placed.glob("*.stl")})
        geo = L4._Geo(ctx)
        tol = ctx.data["tolerances"]["feature_check_tolerances"]["static_intersection_mm3"]["max"]
        motions = [
            _motion("A", outward_world=[-1, 0, 0], len_mm=5.5),
            _motion("B", segments=[dict(outward_world=[-1, 0, 0], len_mm=5.5), dict(outward_world=[0, 0, -1], len_mm=10.0)]),
            _motion("C", segments=[dict(outward_world=[-1, 0, 0], len_mm=5.5), dict(outward_world=[0, 0, -1], len_mm=2.0)]),
            _motion("D", segments=[dict(outward_world=[-1, 0, 0], len_mm=3.0), dict(outward_world=[0, 0, -1], len_mm=10.0)]),
            _motion("E_empty", segments=[]),
            _motion("E_zero", segments=[dict(outward_world=[-1, 0, 0], len_mm=0.0)]),
            _motion("E_nan", segments=[dict(outward_world=[float("nan"), 0, 0], len_mm=5.0)]),
            _motion("E_nolen", segments=[dict(outward_world=[-1, 0, 0])]),
        ]
        st = {"step": 1,
              "required_motion_groups": {m["id"]: {k: m[k] for k in ("movers", "present", "workspace")} for m in motions},
              "motions": motions}
        res = core.LayerResult(4, "反例")
        L4.run_declared_motions(res, geo, st, tol)
    finally:
        L4.PLACED = old

    def fs(mid):
        return findings(res, subject="step01", check=f"motion:{mid}")

    reds, got, ok_all = [], [], True
    for mid in ("A", "C", "D"):
        ok, why, rec = assert_red(fs(mid), severity="BLOCK", require_measured=True)
        ok_all &= ok
        reds += rec
        got.append(f"{mid}:{'红' if ok else why}(峰值={rec[0]['measured'] if rec else None})")
    # 语义再核一遍：A/C 是"未证明脱离"（峰值 ≤ 阈值），D 是"穿墙"（峰值 > 阈值）
    pk = {mid: float(fs(mid)[0].measured) for mid in ("A", "C", "D") if fs(mid) and fs(mid)[0].measured is not None}
    sem_ok = (pk.get("A", 1e9) <= tol and pk.get("C", 1e9) <= tol and pk.get("D", 0.0) > tol)
    ok_b, why_b = assert_green(fs("B"))
    got.append(f"B:{'绿' if ok_b else why_b}")
    unk = [fs(m) for m in ("E_empty", "E_zero", "E_nan", "E_nolen")]
    ok_unk = all(u and u[0].state == "FAIL" and u[0].measured is None for u in unk)
    got.append(f"E 坏声明 {'全部 unknown' if ok_unk else '有的没 unknown'}")
    step = findings(res, subject="step01", check="step_sweep")
    return result(NAME, EXPECT, ok_all and sem_ok and ok_b and ok_unk, got="；".join(got) + f"；语义（A/C 未脱离、D 穿墙）{'成立' if sem_ok else '不成立'}",
                  red=reds, expect_severity="BLOCK",
                  detail=f"峰值 {pk}（阈值 {tol}）；step_sweep={[record(f) for f in step]}；E={[record(u[0]) for u in unk if u]}")


if __name__ == "__main__":
    import json
    r = run()
    print(json.dumps(r, ensure_ascii=False, indent=1))
    sys.exit(0 if r["passed"] else 1)
