"""坏样本：格子里有个"通过的 BLOCK 判据"，同时有个"失败的 WARN 判据"。

旧逻辑判"格子 FAIL 且含任何 BLOCK 项"就算阻断，没看那条 BLOCK 是否真失败 →
阻断清单说不清到底哪条 BLOCK 判据没过。
内核级反例：assertions 形式。
"""
import json
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from core import LayerResult, build_scorecard, PASS, FAIL, BLOCK, WARN   # noqa: E402

NAME = "阻断清单只应包含真正失败的 BLOCK 判据"
EXPECT = "L7/B 在阻断清单里，L7/W（只有 WARN 失败）不在；L7/W 格子仍是 FAIL 但 verdict 只因 B 阻断"


def run() -> dict:
    r = LayerResult(7, "质量与力")
    r.add(subject="W", check="warn_fail", state=FAIL, severity=WARN,
          measured=1, criterion="t", evidence_n=1)
    r.add(subject="W", check="block_pass", state=PASS, severity=BLOCK,
          measured=0, criterion="t", evidence_n=1)
    r.add(subject="B", check="block_fail", state=FAIL, severity=BLOCK,
          measured=1, criterion="t", evidence_n=1)
    sc = build_scorecard([r], {}, [], 0.0)
    blocks = set(sc["blocking_cells"])

    # 对照：只剩 W 那格（WARN 失败 + BLOCK 通过）时，整机不得 BLOCKED
    r2 = LayerResult(7, "质量与力")
    r2.add(subject="W", check="warn_fail", state=FAIL, severity=WARN, measured=1, criterion="t", evidence_n=1)
    r2.add(subject="W", check="block_pass", state=PASS, severity=BLOCK, measured=0, criterion="t", evidence_n=1)
    sc2 = build_scorecard([r2], {}, [], 0.0)

    assertions = [
        {"claim": "真失败的 BLOCK 判据所在格 L7/B 在阻断清单里", "ok": "L7/B" in blocks},
        {"claim": "只有 WARN 失败、BLOCK 通过的格 L7/W 不在阻断清单里", "ok": "L7/W" not in blocks},
        {"claim": "L7/W 格子状态仍是 FAIL（WARN 失败不能被同格 BLOCK 通过托绿）",
         "ok": sc["cells"]["L7/W"]["state"] == FAIL},
        {"claim": "只有 WARN 失败时 verdict 不是 BLOCKED", "ok": sc2["verdict"] != "BLOCKED" and not sc2["blocking_cells"]},
    ]
    return {"name": NAME, "passed": all(x["ok"] for x in assertions),
            "expect": EXPECT,
            "got": f"阻断清单={sorted(blocks)}；W 格={sc['cells']['L7/W']['state']}；只剩 W 时 verdict={sc2['verdict']}",
            "assertions": assertions, "detail": "core.py:build_scorecard blocks"}


if __name__ == "__main__":
    print(json.dumps(run(), ensure_ascii=False, indent=1))
