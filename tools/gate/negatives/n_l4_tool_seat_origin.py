#!/usr/bin/env python3
"""反例 —— 把刀路起点移到挡墙之外、把内腔当批头、把需求杆长当可用杆长，都不能放行。

复用 n_l4_tool_far_seal 的场景（plate + 5 mm 外的远壁），经层的 `tool_access.run_tool_access`
（l4_assembly.run 对刀路那一段调用的就是它）。红必须来自它发出的 finding：
  wall       原样：远壁封死起子入口 → FX:tool_path_to_outside FAIL(BLOCK)，measured=峰值交集
  shifted    坐面点挪到 x=8（挡墙之外）→ FX:tool_origin_on_seat FAIL(BLOCK)，measured=非空（起点不在件表面）
  duplicate  qty=2 但两个 seat 完全相同 → tool_reference unknown（FAIL、measured=None）
  double_surface / repeated_axis / null_anchor / nan_anchor → tool_reference unknown
  wrong_length 把 channel_len 冒充 shaft_len → tool_reach_declared FAIL/unknown
  good_length  available_shaft_len_mm=8 ≥ 所需 → tool_reach_declared PASS（对照）
坏声明那几条是 unknown（measured=None）—— 单独记录，不进 red；red 只放 wall / shifted 这两条量到数的。
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

from core import LayerResult                                                   # noqa: E402
from _harness import findings, assert_red, assert_green, result, record        # noqa: E402
from negatives.n_l4_tool_far_seal import _scene, _ctx                          # noqa: E402
from layers.l4_assembly import _Geo, _Names                                    # noqa: E402
import layers.l4_assembly as L4                                                # noqa: E402
from tool_access import run_tool_access                                        # noqa: E402

NAME = "真实坐面与工具规格：远墙红 / 悬空起点红 / 重复坐面 unknown / 需求长度冒充工具规格红"
EXPECT = "L4/FX:tool_path_to_outside 与 FX:tool_origin_on_seat FAIL(BLOCK) 带数；tool_reference 坏声明 unknown；对照 tool_reach_declared PASS"


def run():
    directory = _scene('origin', (5., -10., -10.), (7., 10., 10.))
    old = L4.PLACED
    try:
        L4.PLACED = directory

        def observe(change):
            ctx = _ctx(directory, 8)
            fa = ctx.data['fasteners']['fasteners'][0]
            change(fa)
            geo = _Geo(ctx)
            res = LayerResult(4, 'tool')
            run_tool_access(res, ctx, geo, _Names(ctx, geo), ctx.data['assembly'], .05)
            return res

        wall = observe(lambda f: None)
        shifted = observe(lambda f: f['tool_access'][0]['seats'][0].update(point_export_local=[8, 0, 0]))
        duplicate = observe(lambda f: (f.update(qty=2), f['tool_access'][0]['seats'].append(dict(f['tool_access'][0]['seats'][0]))))

        def double_surface(f):
            f['qty'] = 2
            f['feature_hole_map'][0].update(holes=[[0, 0, 0], [0, 5, 0]], n=2)
            f['tool_access'][0]['seats'].append({**f['tool_access'][0]['seats'][0],
                                                 'point_export_local': [-2, 0, 0], 'outward_export_local': [-1, 0, 0]})
        duplicate_identity = observe(double_surface)

        def duplicate_map(f):
            f['qty'] = 2
            f['feature_hole_map'].append(dict(f['feature_hole_map'][0]))
            f['head_locator_map_indices'] = [0, 1]
            f['tool_access'][0]['seats'].append({**f['tool_access'][0]['seats'][0], 'map_index': 1,
                                                 'point_export_local': [-2, 0, 0], 'outward_export_local': [-1, 0, 0]})
        repeated_axis = observe(duplicate_map)
        null_anchor = observe(lambda f: f['feature_hole_map'][0].update(holes=[None]))
        nan_anchor = observe(lambda f: f['feature_hole_map'][0].update(holes=[[0, float('nan'), 0]]))
        wrong_length = observe(lambda f: (f['tool_envelope'].pop('available_shaft_len_mm'),
                                         f['tool_envelope'].update(shaft_len_mm={'v': 100}, channel_len_mm={'v': 100})))
    finally:
        L4.PLACED = old

    ok_wall, why_wall, red_wall = assert_red(findings(wall, subject="FX", check="tool_path_to_outside"), severity="BLOCK")
    ok_shift, why_shift, red_shift = assert_red(findings(shifted, subject="FX", check="tool_origin_on_seat"), severity="BLOCK")
    ok_len, why_len, red_len = assert_red(findings(wrong_length, subject="FX", check="tool_reach_declared"),
                                          severity="BLOCK", require_measured=False)
    ok_good, why_good = assert_green(findings(wall, subject="FX", check="tool_reach_declared"))
    unk = {}
    for tag, r in (("duplicate", duplicate), ("double_surface", duplicate_identity), ("repeated_axis", repeated_axis),
                   ("null_anchor", null_anchor), ("nan_anchor", nan_anchor)):
        f = findings(r, subject="FX", check="tool_reference")
        unk[tag] = bool(f) and f[0].state == "FAIL" and f[0].measured is None
    passed = ok_wall and ok_shift and ok_len and ok_good and all(unk.values())
    red = red_wall + red_shift + [x for x in red_len if x["measured"] is not None]
    return result(NAME, EXPECT, passed,
                  got=(f"远墙 {'红' if ok_wall else why_wall}(峰值={red_wall[0]['measured'] if red_wall else None})；"
                       f"悬空起点 {'红' if ok_shift else why_shift}(measured={red_shift[0]['measured'] if red_shift else None})；"
                       f"需求长度冒充 {'红' if ok_len else why_len}；重复/坏锚点 tool_reference unknown={unk}；"
                       f"对照 tool_reach_declared {'绿' if ok_good else why_good}"),
                  red=red, expect_severity="BLOCK",
                  detail=f"wall={[record(f) for f in findings(wall, subject='FX')]}；wrong_length={red_len}")


if __name__ == "__main__":
    print(json.dumps(run(), ensure_ascii=False, indent=1))
