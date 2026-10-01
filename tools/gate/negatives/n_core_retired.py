"""坏样本：判据"退役"（基准失去权威）被写成 STALE。

历史教训（审计 2026-09-13 F-核-5）：l6_motion.py / l7_mass.py 把 `state=STALE` 硬编码当"判据退役"标签，
而 core 里 STALE = "输入变了没重跑"，会让 verdict 永远 INCOMPLETE、退出码永远 ≥ 2，
且 tally[STALE] 不再能回答"哪些格子过期了"。
修法：新增状态 RETIRED —— 不阻断、不计入 INCOMPLETE、单独计数；STALE 仍然让整机 INCOMPLETE。
"""
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from core import LayerResult, build_scorecard, PASS, FAIL, STALE, WARN, BLOCK   # noqa: E402

NAME = "退役判据用 RETIRED：不阻断、不算 INCOMPLETE；STALE 仍然 INCOMPLETE"
EXPECT = "RETIRED 格 → verdict CLEAR 且不在 blocking_cells；换成 STALE → verdict INCOMPLETE"


def run() -> dict:
    from core import RETIRED
    r = LayerResult(6, "运动")
    r.add(subject="_joints", check="range_matches_upstream", state=RETIRED, severity=WARN,
          measured=0, criterion="【已失去权威，保留为记录】", evidence_n=14)
    r.add(subject="left_knee", check="single_axis_sweep", state=PASS, severity=BLOCK,
          measured=0.0, criterion="交集 ≤ 0.05", evidence_n=100)
    sc = build_scorecard([r], {}, [], 0.0)
    a_ok = (sc["verdict"] == "CLEAR" and "L6/_joints" not in sc["blocking_cells"]
            and sc["tally"].get(RETIRED) == 1 and sc["cells"]["L6/_joints"]["state"] == RETIRED)

    s = LayerResult(6, "运动")
    s.add(subject="_joints", check="range_matches_upstream", state=STALE, severity=WARN,
          measured=0, criterion="t", evidence_n=14)
    s.add(subject="left_knee", check="single_axis_sweep", state=PASS, severity=BLOCK,
          measured=0.0, criterion="t", evidence_n=100)
    sc2 = build_scorecard([s], {}, [], 0.0)
    b_ok = sc2["verdict"] == "INCOMPLETE"

    assertions = [{"claim": "RETIRED 不阻断且 verdict=CLEAR", "ok": bool(a_ok)},
                  {"claim": "STALE 仍 INCOMPLETE", "ok": bool(b_ok)}]
    return {"name": NAME, "passed": all(x["ok"] for x in assertions), "expect": EXPECT,
            "got": f"RETIRED: verdict={sc['verdict']} tally={sc['tally']}；STALE: verdict={sc2['verdict']}",
            "assertions": assertions, "detail": "core.py:RETIRED / build_scorecard verdict"}


if __name__ == "__main__":
    import json
    print(json.dumps(run(), ensure_ascii=False, indent=1))
