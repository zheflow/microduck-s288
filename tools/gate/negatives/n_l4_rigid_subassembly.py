#!/usr/bin/env python3
"""反例 —— 两件已经连接的子总成，不许各挑一个相反的抽出方向。

经过层的 `run_declared_motions`（run() 对每个带 motions 的装配步调用的就是它）：
    a、b 两个 1 mm 立方是一个刚性子总成；front 挡住 +x、back 挡住 −x。
  坏 plus   整组沿 +x 抽 4 mm → 撞 front  → L4/step01:motion:plus  FAIL(BLOCK)，measured=峰值交集
  坏 minus  整组沿 −x 抽 4 mm → 撞 back   → motion:minus FAIL(BLOCK)
  好 good   把 back 从在场清单里拿掉（只剩 front）后沿 −x 抽 → motion:good PASS
  坏声明    len_mm=0 / 方向 NaN / present 里有未导出实体 → 层判 unknown（FAIL、measured=None），单独记录
  覆盖义务  动作清单缺一条 / 重复 / 移动组少件 / 在场少件 / 工位不符 → step:motion_coverage unknown（FAIL），
            证明"删掉一条路径不能让整步通过"。
"""
from __future__ import annotations
import json
import sys
from pathlib import Path

_HERE = Path(__file__).resolve()
GATE = _HERE.parents[1]
for _p in (str(_HERE.parent), str(GATE), str(GATE.parents[1])):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from _harness import FakeCtx, core, findings, assert_red, assert_green, result, record   # noqa: E402
from negatives.n_l4_split_direction import _write                                        # noqa: E402
import layers.l4_assembly as L4                                                          # noqa: E402

NAME = "刚性子总成统一方向：正反都红（motion:*）、去掉反向障碍后反向绿；坏声明/漏路径 unknown"
EXPECT = "L4/step01:motion:plus / motion:minus FAIL(BLOCK) 带峰值；motion:good PASS；坏声明与覆盖缺口 unknown"


def _m(mid, **kw):
    base = dict(movers=["a", "b"], present=["a", "b", "front", "back"], workspace="test",
                sense="withdrawal_from_assembled", len_mm=4, step_mm=.25)
    base.update(kw)
    return dict(id=mid, **base)


def _step(motions, required=None):
    req = required if required is not None else \
        {m["id"]: {k: m[k] for k in ("movers", "present", "workspace")} for m in motions}
    return {"step": 1, "required_motion_groups": req, "motions": motions}


def run():
    boxes = {"a": ((-.5, -.5, -.5), (.5, .5, .5)),
             "b": ((-.5, 3.5, -.5), (.5, 4.5, .5)),
             "front": ((1.5, -1, -1), (2.5, 1, 1)),
             "back": ((-2.5, 3, -1), (-1.5, 5, 1))}
    directory = _write(boxes, "rigid")
    old = L4.PLACED
    try:
        L4.PLACED = directory
        ctx = FakeCtx(core.load_data(), {}, {p.stem: p for p in directory.glob('*.stl')})
        geo = L4._Geo(ctx)
        tol = ctx.data['tolerances']['feature_check_tolerances']['static_intersection_mm3']['max']
        motions = [_m("plus", outward_world=[1, 0, 0]),
                   _m("minus", outward_world=[-1, 0, 0]),
                   _m("good", outward_world=[-1, 0, 0], present=["a", "b", "front"]),
                   _m("bad_len0", outward_world=[1, 0, 0], len_mm=0),
                   _m("bad_nan", outward_world=[float("nan"), 0, 0]),
                   _m("bad_missing", outward_world=[1, 0, 0], present=["a", "b", "front", "back", "missing"])]
        res = core.LayerResult(4, "反例")
        L4.run_declared_motions(res, geo, _step(motions), tol)

        # 覆盖义务：五种声明缺陷 → motion_coverage unknown
        left = _m("left", movers=["a"], present=["a", "front"], workspace="left", outward_world=[-1, 0, 0])
        right = _m("right", movers=["b"], present=["b", "back"], workspace="right", outward_world=[1, 0, 0])
        req = {m["id"]: {k: m[k] for k in ("movers", "present", "workspace")} for m in (left, right)}
        cov = []
        for mm in ([left], [left, left], [left, {**right, "movers": ["a"]}],
                   [left, {**right, "present": ["b"]}], [left, {**right, "workspace": "left"}]):
            r2 = core.LayerResult(4, "反例")
            L4.run_declared_motions(r2, geo, _step(mm, req), tol)
            c = findings(r2, subject="step01", check="motion_coverage")
            cov.append(bool(c) and c[0].state == "FAIL" and c[0].measured is None)
        r3 = core.LayerResult(4, "反例")
        L4.run_declared_motions(r3, geo, _step([left, right], req), tol)
        cov_ok_full = findings(r3, subject="step01", check="motion_coverage")
        complete = bool(cov_ok_full) and cov_ok_full[0].state == "PASS" and cov_ok_full[0].measured == 2
    finally:
        L4.PLACED = old

    def fs(mid):
        return findings(res, subject="step01", check=f"motion:{mid}")

    ok_p, why_p, red_p = assert_red(fs("plus"), severity="BLOCK", require_measured=True)
    ok_m, why_m, red_m = assert_red(fs("minus"), severity="BLOCK", require_measured=True)
    ok_g, why_g = assert_green(fs("good"))
    unk = [fs(m) for m in ("bad_len0", "bad_nan", "bad_missing")]
    ok_unk = all(u and u[0].state == "FAIL" and u[0].measured is None for u in unk)
    passed = ok_p and ok_m and ok_g and ok_unk and all(cov) and complete
    return result(NAME, EXPECT, passed,
                  got=(f"plus {'红' if ok_p else why_p}(峰值={red_p[0]['measured'] if red_p else None})；"
                       f"minus {'红' if ok_m else why_m}(峰值={red_m[0]['measured'] if red_m else None})；"
                       f"good {'绿' if ok_g else why_g}；坏声明 {'全 unknown' if ok_unk else '未全 unknown'}；"
                       f"覆盖缺口 {cov}；完整清单 motion_coverage={'PASS/2' if complete else '不对'}"),
                  red=red_p + red_m, expect_severity="BLOCK",
                  detail=f"坏声明={[record(u[0]) for u in unk if u]}")


if __name__ == "__main__":
    print(json.dumps(run(), ensure_ascii=False, indent=1))
