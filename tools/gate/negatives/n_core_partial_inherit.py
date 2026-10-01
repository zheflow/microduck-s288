"""坏样本：`--parts` 子集跑（只查 Y），上一版里 X 的格子。

历史教训（审计 2026-09-13 F-核-6 / Codex01 FG02）：同层未查的件没有继承逻辑，
被义务循环整片写成 obligation_unmet 红；于是"改一个件只重跑受影响子集"（元规则 7）不成立，
而子集跑的结果又覆盖 scorecard.prev.json。
规则：子集跑时，本轮跑过的层里**不在范围内**的格子按 inputs_hash 继承（变了 → STALE）；
全层跑（没有 --parts）时不继承，义务缺口仍然红。
"""
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from core import LayerResult, build_scorecard, sha256_file, ROOT, PASS, FAIL, STALE   # noqa: E402

NAME = "子集跑：范围外的同层格子按 hash 继承；全层跑不继承"
EXPECT = "only={Y}：L5/X 沿用 PASS；输入 hash 变了 → STALE；only=None：L5/X 不继承（义务红）"

_F = "tools/gate/core.py"


def _prev(h):
    return {"cells": {"L5/X": {"layer": 5, "subject": "X", "state": PASS, "evidence_n": 3, "notes": [],
                               "checks": [{"check": "c", "state": PASS, "severity": "BLOCK",
                                           "measured": 1, "criterion": "t", "evidence_n": 3}],
                               "inputs_hash": {_F: h}, "state_computed": PASS}}}


def _run(only, h):
    r = LayerResult(5, "功能达成")
    r.add(subject="Y", check="c", state=PASS, measured=1, criterion="t", evidence_n=3)
    return build_scorecard([r], {}, [], 0.0, prev=_prev(h), layers_run=[5], only=only)


def run() -> dict:
    real = sha256_file(ROOT / _F)
    a = _run({"Y"}, real)
    b = _run({"Y"}, "0" * 64)
    c = _run(None, real)
    a_ok = a["cells"].get("L5/X", {}).get("state") == PASS
    b_ok = b["cells"].get("L5/X", {}).get("state") == STALE
    cx = c["cells"].get("L5/X")
    c_ok = (cx is None) or (cx["state"] == FAIL and any(k.get("check") == "obligation_unmet" for k in cx["checks"]))
    assertions = [{"claim": "子集跑，输入未变 → 继承 PASS", "ok": bool(a_ok)},
                  {"claim": "子集跑，输入已变 → STALE", "ok": bool(b_ok)},
                  {"claim": "全层跑 → 不继承", "ok": bool(c_ok)}]
    return {"name": NAME, "passed": all(x["ok"] for x in assertions), "expect": EXPECT,
            "got": f"未变={a['cells'].get('L5/X', {}).get('state')} 已变={b['cells'].get('L5/X', {}).get('state')} "
                   f"全层={None if cx is None else cx['state']}",
            "assertions": assertions, "detail": "core.py:build_scorecard(only=…)"}


if __name__ == "__main__":
    import json
    print(json.dumps(run(), ensure_ascii=False, indent=1))
