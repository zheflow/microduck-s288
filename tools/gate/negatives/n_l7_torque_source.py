#!/usr/bin/env python3
"""反例（09-13 F-L7-5）—— 额定扭矩读两个键，两处不一致无人报警。

tolerances.yaml 自己写着：`load.servo_rated_torque_Nm` 是**唯一权威**，`load.servo_output_bearing.rated_stall_torque_Nm`
是它的镜像，"两处不一致时以 load.servo_rated_torque_Nm 为准，并且要把层改成读那里、把本键删掉"
（该键的 duplication_note）。修前 `_rated_torque_Nm` 只认镜像键（找不到再去 docs/ 的散文里 grep 0.6）。
修后：只读权威键 `servo_rated_torque_Nm.stall_torque_Nm`；废弃键若仍存在且与权威值不等 → `_load:torque_source_consistent`
FAIL(BLOCK, measured=差值)；相等 → INFO 提示应删；不存在 → INFO PASS。数据侧已按 yaml 自述把废弃键删掉。

坏样本：把废弃键写回 v=0.5（权威 0.6）→ 差 −0.1 → 红；同时 `_load:rated_stall_torque` 报的数必须是权威值 0.6。
对照 A：真数据（废弃键已删）→ torque_source_consistent PASS(INFO)。
对照 B：废弃键写回 0.6（相等）→ PASS(INFO)，detail 提示应删。
三次都是"快跑"：parts.yaml 清单置空 → `_assign` 认不到件、run 在整机量之前返回，但扭矩来源判据在那之前就已发出
（README 反例包允许对整机层这样做，断言仍落在 run 发出的 finding 上）。
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

NAME = "L7：废弃键 rated_stall_torque_Nm 与权威 servo_rated_torque_Nm 不一致 → torque_source_consistent 红；一致/已删 → 绿"
EXPECT = ("L7/_load:torque_source_consistent FAIL(BLOCK, measured=-0.1)，rated_stall_torque measured=权威值；"
          "真数据（键已删）PASS；写回相等值 PASS")
DEP_KEY = "rated_stall_torque_Nm"


def _fast(data):
    d = deepcopy(data)
    d["parts"] = {"parts": []}
    return d


def _with_dep(data, v):
    d = deepcopy(data)
    sob = d["tolerances"]["load"].setdefault("servo_output_bearing", {})
    sob[DEP_KEY] = {"v": v, "src": "datasheet", "src_note": "反例写回的镜像键"}
    return d


def run() -> dict:
    base = core.load_data()
    auth, _ = core.num(base["tolerances"]["load"]["servo_rated_torque_Nm"]["stall_torque_Nm"])
    dep_present_in_real = DEP_KEY in (base["tolerances"]["load"].get("servo_output_bearing") or {})

    r_bad = FX.run_l7(_fast(_with_dep(base, 0.5)))
    r_a = FX.run_l7(_fast(base))
    r_b = FX.run_l7(_fast(_with_dep(base, auth)))

    bad = findings(r_bad, subject="_load", check="torque_source_consistent")
    ok1, why1, red1 = assert_red(bad, severity="BLOCK", require_measured=True)
    diff_ok = ok1 and all(abs(abs(float(x["measured"])) - abs(0.5 - float(auth))) < 1e-9 for x in red1)
    rated = findings(r_bad, subject="_load", check="rated_stall_torque")
    rated_ok = bool(rated) and all(f.state == "PASS" and abs(float(f.measured) - float(auth)) < 1e-12 for f in rated)

    a = findings(r_a, subject="_load", check="torque_source_consistent")
    okA, whyA = assert_green(a)
    b = findings(r_b, subject="_load", check="torque_source_consistent")
    okB, whyB = assert_green(b)
    b_says_delete = okB and any("删" in (f.detail or "") for f in b)

    assertions = [
        {"claim": f"真 tolerances.yaml 里废弃键 {DEP_KEY} 已删（yaml 自述要删）", "ok": not dep_present_in_real},
        {"claim": f"坏样本下层读到的额定是权威值 {auth}，不是废弃键的 0.5（现：{summarize(rated, n=1)}）", "ok": rated_ok},
        {"claim": f"对照 A（真数据）torque_source_consistent PASS：{whyA or 'ok'}", "ok": okA},
        {"claim": f"对照 B（写回相等值）PASS 且 detail 提示应删：{whyB or summarize(b, n=1)}", "ok": b_says_delete},
    ]
    passed = ok1 and diff_ok and all(x["ok"] for x in assertions)
    out = result(NAME, EXPECT, passed,
                 got=(f"废弃键 0.5 vs 权威 {auth} → {summarize(bad, n=1) if bad else '（没有 torque_source_consistent 这条判据）'}；"
                      f"rated_stall_torque={summarize(rated, n=1)}；对照 A {'绿' if okA else whyA}；对照 B {'绿' if okB else whyB}"),
                 red=(red1 if ok1 else []), expect_severity="BLOCK",
                 detail=f"坏={[record(f) for f in bad]}；A={[record(f) for f in a]}；B={[record(f) for f in b]}")
    out["assertions"] = assertions
    return out


if __name__ == "__main__":
    r = run()
    print(json.dumps(r, ensure_ascii=False, indent=1))
    sys.exit(0 if r["passed"] else 1)
