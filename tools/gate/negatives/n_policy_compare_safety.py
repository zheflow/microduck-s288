"""Physical collision acceptance must not inherit exemptions from upstream."""
from __future__ import annotations
import copy
import json
import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "tools/sim"))
import policy_pose_compare as compare


def run():
    row = {"mode": "stand", "case": "test", "step": 1,
           "pose_deg": {"joint": 2.0}, "hits": [["a", "b", 22.73]]}
    original = dict(row, hits=[["a", "b", 3.0]])
    mapping = {"a": "bodyA", "b": "bodyB"}
    assertions = []
    with tempfile.TemporaryDirectory(prefix="duck_compare_negative_") as d:
        p, q = Path(d) / "ours.jsonl", Path(d) / "orig.jsonl"
        def check(ours, orig):
            p.write_text("\n".join(json.dumps(r) for r in ours))
            q.write_text("\n".join(json.dumps(r) for r in orig))
            return compare.analyze(p, q, mapping, mapping, 0.05)
        r = check([row], [original])
        assertions.append(("shared collision remains physical defect", r["poses_with_collisions"] == 1))
        assertions.append(("worsening measured per same pose", abs(r["worsened_pairs"]["bodyA×bodyB"]["max_increase_mm3"] - 19.73) < 1e-8))
        equal = check([original], [original])
        assertions.append(("equal upstream collision is not exempt", equal["poses_with_collisions"] == 1))
        small = check([dict(row, hits=[["a", "b", .2]])], [dict(row, hits=[])])
        assertions.append(("0.2 mm3 cannot round away", small["poses_with_defect"] == 1))
        clear = check([dict(row, hits=[])], [dict(row, hits=[])])
        assertions.append(("clear independent control", clear["poses_with_collisions"] == 0))
        mismatched = copy.deepcopy(original); mismatched["pose_deg"]["joint"] += .1
        cases = [("missing pose", [row], []), ("duplicate pose", [row, row], [original]),
                 ("different pose tuple", [row], [mismatched]),
                 ("nonfinite volume", [dict(row, hits=[["a", "b", float("nan")]])], [original]),
                 ("unmapped solid", [dict(row, hits=[["x", "b", 1.]])], [original]),
                 ("no evidence", [], [])]
        for name, ours, orig in cases:
            try:
                check(ours, orig)
            except ValueError:
                assertions.append((name, True))
            else:
                assertions.append((name, False))
    return {"name": "policy_compare_safety", "passed": all(ok for _, ok in assertions),
            "expect": "shared/worsened collision remains physical defect; invalid evidence raises",
            "got": "; ".join(f"{name}={ok}" for name, ok in assertions),
            "assertions": [{"claim": n, "ok": ok} for n, ok in assertions]}


if __name__ == "__main__":
    result = run()
    print(json.dumps(result, ensure_ascii=False, indent=2))
    sys.exit(0 if result["passed"] else 1)
