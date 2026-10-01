#!/usr/bin/env python3
"""反例（09-13 F-L7-6）—— 元件认领按 category 写死、qty 从不校验。

修前：servo→MJCF geom、battery/bearing→envelope 正则、original_prints→orig_ 前缀，全在 l7_mass._mass_items 的
if/elif 里；board/imu/camera/other 无规则，有质量也进不去；qty 写多少都没人看。
修后：认领规则由 components.yaml 每条元件自己的 `claim:` 声明（by: mjcf_servo_geom | envelope_box |
envelope_ring | placed_prefix），层按规则认领、不再看 category；没有 claim 的元件 → `component_body:<id>` unknown；
认领到的实例数 ≠ qty → `component_qty` FAIL(BLOCK, measured=实例数)。

坏样本（同一次 run，两个 subject 互不相干）：
  ① battery_3s.qty 1 → 2（placed 只有一个 zz_battery）→ `battery_3s:component_qty` FAIL(BLOCK, measured=1)；
  ② 删掉 bearing_6704zz 的 `claim` → `bearing_6704zz:component_body` unknown（有质量 5.9 g × 4，不许静默消失），
     且因为这是清单缺口，`com_in_support_home` 跟着 unknown（F-L7-4）。
对照：完整清单 → battery_3s / bearing_6704zz 的 component_qty PASS（measured == 清单 qty；09-17 髋横滚 6704 取消后 1 / 2），没有 component_body 红。
用了 negatives/_l7_fixture.py 的夹具（只删不编，见该文件头）。
"""
from __future__ import annotations
import json
import sys
from copy import deepcopy
from pathlib import Path

_HERE = Path(__file__).resolve()
GATE = _HERE.parents[1]
for _p in (str(_HERE.parent), str(GATE), str(GATE / "layers"), str(GATE.parents[1])):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import core                                                                      # noqa: E402
from _harness import findings, assert_red, assert_green, result, record, summarize  # noqa: E402
import _l7_fixture as FX                                                         # noqa: E402

NAME = "L7：qty 写 2 实际认领到 1 → component_qty 红；有质量但无 claim 规则 → component_body unknown 而不是消失"
EXPECT = ("L7/battery_3s:component_qty FAIL(BLOCK, measured=1)；L7/bearing_6704zz:component_body FAIL(measured=None) "
          "且 com_in_support_home unknown；对照 component_qty PASS(measured == 清单 qty，09-17 起 1/2)、无 component_body 红")
QTY_ID, CLAIM_ID = "battery_3s", "bearing_6704zz"


def run() -> dict:
    base = core.load_data()
    ctrl, drop = FX.complete_inventory(base)
    pd = FX.placed_stand_in("claim", drop)
    bad = deepcopy(ctrl)
    cq = next(x for x in bad["components"]["components"] if x.get("id") == QTY_ID)
    cq["qty"] = 2
    cc = next(x for x in bad["components"]["components"] if x.get("id") == CLAIM_ID)
    had_claim = cc.pop("claim", None) is not None

    r_bad = FX.run_l7(bad, placed_dir=pd)
    r_good = FX.run_l7(ctrl, placed_dir=pd)

    qty = findings(r_bad, subject=QTY_ID, check="component_qty")
    ok1, why1, red1 = assert_red(qty, severity="BLOCK", require_measured=True)
    n_ok = ok1 and all(int(float(x["measured"])) == 1 for x in red1)
    body = findings(r_bad, subject=CLAIM_ID, check="component_body")
    unk2 = bool(body) and all(f.state == "FAIL" and f.measured is None for f in body) \
        and any("claim" in (f.detail or "") for f in body)
    com = findings(r_bad, subject="_stability", check="com_in_support_home")
    com_unk = bool(com) and all(f.state == "FAIL" and f.measured is None for f in com) \
        and any(CLAIM_ID in (f.detail or "") for f in com)

    g_q1 = findings(r_good, subject=QTY_ID, check="component_qty")
    g_q2 = findings(r_good, subject=CLAIM_ID, check="component_qty")
    okg1, whyg1 = assert_green(g_q1)
    okg2, whyg2 = assert_green(g_q2)
    g_vals = ([f.measured for f in g_q1], [f.measured for f in g_q2])
    g_body_red = [f for f in r_good.findings if f.check == "component_body" and f.state == "FAIL"]
    # 09-17：对照组的期望 measured = 清单 qty（bearing_6704zz 09-17 起 4 → 2：髋横滚法兰侧取消，只剩膝），不再写死 4
    g_qty_expect = ([next(x for x in ctrl["components"]["components"] if x.get("id") == QTY_ID)["qty"]],
                    [next(x for x in ctrl["components"]["components"] if x.get("id") == CLAIM_ID)["qty"]])
    g_qty_vals_ok = okg1 and okg2 and g_vals == g_qty_expect

    assertions = [
        {"claim": f"components.yaml 里 {CLAIM_ID} 本来带 claim 规则（夹具删掉它）", "ok": had_claim},
        {"claim": f"{CLAIM_ID} 无 claim 规则 → component_body unknown 且 detail 提到 claim（现：{summarize(body, n=1)}）", "ok": unk2},
        {"claim": f"该缺口让 com_in_support_home unknown 并点名 {CLAIM_ID}（现：{summarize(com, n=1)}）", "ok": com_unk},
        {"claim": f"对照 component_qty PASS 且 measured == qty：{QTY_ID}={g_vals[0]}，{CLAIM_ID}={g_vals[1]}", "ok": g_qty_vals_ok},
        {"claim": f"对照没有 component_body 红（现有 {[f.subject for f in g_body_red]}）", "ok": not g_body_red},
    ]
    passed = ok1 and n_ok and all(a["ok"] for a in assertions)
    out = result(NAME, EXPECT, passed,
                 got=(f"① {QTY_ID} qty=2 → component_qty {summarize(qty, n=1) if qty else '（没有这条判据）'}；"
                      f"② {CLAIM_ID} 无 claim → component_body {summarize(body, n=1) if body else '（没有这条判据 = 静默消失）'}；"
                      f"对照 {QTY_ID}={g_vals[0]} {CLAIM_ID}={g_vals[1]}，component_body 红 {len(g_body_red)}"),
                 red=(red1 if ok1 else []), expect_severity="BLOCK",
                 detail=f"坏={[record(f) for f in qty + body + com]}")
    out["assertions"] = assertions
    return out


if __name__ == "__main__":
    r = run()
    print(json.dumps(r, ensure_ascii=False, indent=1))
    sys.exit(0 if r["passed"] else 1)
