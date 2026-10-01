"""坏样本：判据说"通过"但一个东西都没查（evidence_n=0）。

元规则 2：每个 0 都要带证据数量。"0 碰撞 / 0 对"是没跑，不是通过。
内核级反例：用 assertions 形式（runner 契约），每条断言单独可读。
"""
import json
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from core import LayerResult, build_scorecard, PASS, NOT_RUN   # noqa: E402

NAME = "evidence_n=0 的 PASS 必须降级，不能算通过"
EXPECT = "无证据的 PASS → NOT_RUN；有证据的 PASS 保持 PASS；verdict 因 NOT_RUN 变 INCOMPLETE"


def run() -> dict:
    r = LayerResult(3, "静态装配")
    r.add(subject="A", check="zero_evidence", state=PASS, measured=0.0,
          criterion="交集 ≤ 0.05", evidence_n=0)
    r.add(subject="B", check="real", state=PASS, measured=0.0,
          criterion="交集 ≤ 0.05", evidence_n=7)
    sc = build_scorecard([r], {}, [], 0.0)
    a = sc["cells"]["L3/A"]["state"]
    b = sc["cells"]["L3/B"]["state"]
    a_check = sc["cells"]["L3/A"]["checks"][0]
    assertions = [
        {"claim": "evidence_n=0 的 PASS 格子 → NOT_RUN", "ok": a == NOT_RUN},
        {"claim": "那条判据本身也被改成 NOT_RUN 且 detail 说明原因",
         "ok": a_check["state"] == NOT_RUN and "evidence_n=0" in str(a_check.get("detail", ""))},
        {"claim": "evidence_n=7 的 PASS 保持 PASS", "ok": b == PASS},
        {"claim": "有 NOT_RUN 格子时 verdict 不是 CLEAR", "ok": sc["verdict"] != "CLEAR"},
    ]
    return {"name": NAME, "passed": all(x["ok"] for x in assertions),
            "expect": EXPECT,
            "got": f"无证据={a} 有证据={b} verdict={sc['verdict']}",
            "assertions": assertions, "detail": "core.py:build_scorecard 元规则 2"}


if __name__ == "__main__":
    print(json.dumps(run(), ensure_ascii=False, indent=1))
