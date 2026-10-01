"""坏样本：一个层什么都不做（发不出任何 finding）。

历史教训：记分卡原来只为"已发出的 finding"建格子，于是没生成的检查义务从矩阵上消失
（是 `·` 不是 ❌）。极端情况把所有层删光会判 CLEAR。审查 FG01。
内核级反例：assertions 形式。
"""
import json
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from core import LayerResult, build_scorecard, load_data, PASS, FAIL   # noqa: E402

NAME = "空层 / 未实现的层必须变红，不能从记分卡上消失"
EXPECT = "空层 → 出现 obligation_unmet 红格且非 CLEAR；补齐义务 → 该层义务格不红"


def run() -> dict:
    data = load_data()
    empty = LayerResult(2, "特征存在")                      # 什么都不发
    sc_bad = build_scorecard([empty], {}, [], 0.0, data=data)
    unmet = [cid for cid, c in sc_bad["cells"].items()
             if c["layer"] == 2 and any(k["check"] == "obligation_unmet" and k["state"] == FAIL
                                        for k in c["checks"])]
    n_parts = len(data["parts"]["parts"])

    # 好样本：同一个层发出覆盖全部义务的判据 → 该层的义务格不应再红
    good = LayerResult(2, "特征存在")
    good.covered = {x["id"] for x in data["features"]["features"]}
    for p in data["parts"]["parts"]:
        good.add(subject=p["id"], check="stub", state=PASS, measured=1,
                 criterion="反例用桩", evidence_n=1)
    sc_good = build_scorecard([good], {}, [], 0.0, data=data)
    l2_unmet = [c for cid, c in sc_good["cells"].items()
                if c["layer"] == 2 and any(k["check"] in ("obligation_unmet", "特征覆盖率")
                                           and k["state"] == FAIL for k in c["checks"])]

    assertions = [
        {"claim": "空层 → blocking_cells 非空且 verdict 非 CLEAR",
         "ok": bool(sc_bad["blocking_cells"]) and sc_bad["verdict"] != "CLEAR"},
        {"claim": f"空层 → 该层每个件（{n_parts} 件）+ _layer 都有 obligation_unmet 红格",
         "ok": len(unmet) >= n_parts + 1 and "L2/_layer" in unmet},
        {"claim": "空层 → L2/_coverage 特征覆盖率红（层没自报 covered）",
         "ok": sc_bad["cells"].get("L2/_coverage", {}).get("state") == FAIL},
        {"claim": "补齐义务后 L2 没有 obligation_unmet / 特征覆盖率红格", "ok": not l2_unmet},
    ]
    return {"name": NAME, "passed": all(x["ok"] for x in assertions),
            "expect": EXPECT,
            "got": f"空层 blocking={len(sc_bad['blocking_cells'])} verdict={sc_bad['verdict']} "
                   f"unmet={len(unmet)}；好样本 L2 未满足义务 {len(l2_unmet)} 格",
            "assertions": assertions, "detail": "core.py:obligations + COVERAGE_ITEMS"}


if __name__ == "__main__":
    print(json.dumps(run(), ensure_ascii=False, indent=1))
