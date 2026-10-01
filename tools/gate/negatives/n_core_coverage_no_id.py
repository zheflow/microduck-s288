"""坏样本：覆盖率义务的数据条目没有 id（assembly.yaml 的 19 步就是这样）。

历史教训（审计 2026-09-13 F-核-1）：core.build_scorecard 的 COVERAGE_ITEMS 循环里
`want = {x["id"] ...}; if not want: continue` —— 清单没 id 时覆盖率义务静默消失，
`L4/_coverage` 格从未存在过。元规则 4 的框架级落实自己违反了元规则 4。
"""
import copy
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from core import LayerResult, build_scorecard, load_data, PASS, FAIL   # noqa: E402

NAME = "覆盖率义务的清单条目缺 id：必须出一格红，不能静默跳过"
EXPECT = "L4/_coverage FAIL(BLOCK)，detail 说明缺 id；补上 id 且层自报 covered → PASS"


def run() -> dict:
    data = load_data()
    steps = (data.get("assembly") or {}).get("assembly_order") or []
    bad = copy.deepcopy(data)
    for s in bad["assembly"]["assembly_order"]:
        s.pop("id", None)
    r = LayerResult(4, "装得进去")
    r.add(subject="step01", check="step_sweep", state=PASS, measured=0.0, criterion="t", evidence_n=5)
    sc = build_scorecard([r], {}, [], 0.0, data=bad)
    c = sc["cells"].get("L4/_coverage")
    k = (c or {}).get("checks", [{}])[0]
    bad_red = bool(c) and c["state"] == FAIL and k.get("severity") == "BLOCK" and "id" in str(k.get("detail"))

    good = copy.deepcopy(data)
    for i, s in enumerate(good["assembly"]["assembly_order"]):
        s["id"] = f"neg_step{i:02d}"
    g = LayerResult(4, "装得进去")
    g.add(subject="step01", check="step_sweep", state=PASS, measured=0.0, criterion="t", evidence_n=5)
    g.covered = {s["id"] for s in good["assembly"]["assembly_order"]}
    sc2 = build_scorecard([g], {}, [], 0.0, data=good)
    c2 = sc2["cells"].get("L4/_coverage")
    good_green = bool(c2) and c2["state"] == PASS

    return {"name": NAME, "passed": bad_red and good_green, "expect": EXPECT,
            "got": f"缺 id：格={'有' if c else '无'} state={(c or {}).get('state')} detail={str(k.get('detail'))[:60]}；"
                   f"有 id 且全覆盖：state={(c2 or {}).get('state')}",
            "red": ([{"check": k.get("check"), "state": k.get("state"), "severity": k.get("severity"),
                      "measured": k.get("measured")}] if c else []),
            "expect_severity": "BLOCK",
            "allow_unknown_red": "覆盖率算不出来本身就是 unknown 类红（measured 为空是设计）",
            "detail": f"assembly.yaml 当前 {len(steps)} 步；core.py:COVERAGE_ITEMS"}


if __name__ == "__main__":
    import json
    print(json.dumps(run(), ensure_ascii=False, indent=1))
