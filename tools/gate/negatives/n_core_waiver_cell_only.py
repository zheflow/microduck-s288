"""坏样本：豁免 id 只写到格子（"L5/X"），没写判据名。

历史教训（审计 2026-09-13 F-核-4）：core.apply_waivers 接受 `L5/X`、`X.5` 这种不带 check 名的 id，
会把该格**全部** WARN 判据一起豁免。豁免必须精确到判据：`L5/X:check` 或 `X.5.check`。
另：waivers.yaml 文档 schema 写的 `bound_input_hashes` 键必须是仓库相对路径 → sha256，
按旧文档（duck_py_sha256 …）写的豁免永远匹配不上 manifest。
"""
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from core import LayerResult, apply_waivers, sha256_file, ROOT, FAIL, WAIVED, WARN   # noqa: E402

NAME = "豁免 id 必须带判据名；只写格子 id 的豁免被拒绝"
EXPECT = "'L5/X' → 两条 WARN 都仍 FAIL 且有拒绝说明；'L5/X:c1' → 只有 c1 WAIVED"

_F = "tools/gate/data/parts.yaml"


def _two():
    r = LayerResult(5, "功能达成")
    a = r.add(subject="X", check="c1", state=FAIL, severity=WARN, measured=1, criterion="t", evidence_n=1)
    b = r.add(subject="X", check="c2", state=FAIL, severity=WARN, measured=1, criterion="t", evidence_n=1)
    return a, b


def run() -> dict:
    manifest = {_F: sha256_file(ROOT / _F)}
    bound = {_F: manifest[_F]}
    a, b = _two()
    notes = apply_waivers([a, b], {"L5/X": {"reason": "r", "issued_by": "u", "date": "d",
                                            "bound_input_hashes": bound}}, manifest)
    cell_only_rejected = a.state == FAIL and b.state == FAIL and any("判据名" in n or "check" in n for n in notes)
    a2, b2 = _two()
    apply_waivers([a2, b2], {"L5/X:c1": {"reason": "r", "issued_by": "u", "date": "d",
                                         "bound_input_hashes": bound}}, manifest)
    precise_ok = a2.state == WAIVED and b2.state == FAIL
    assertions = [{"claim": "格子级 id 被拒绝", "ok": bool(cell_only_rejected)},
                  {"claim": "带判据名的 id 只豁免那一条", "ok": bool(precise_ok)}]
    return {"name": NAME, "passed": all(x["ok"] for x in assertions), "expect": EXPECT,
            "got": f"格子级：c1={a.state} c2={b.state} notes={notes[:1]}；精确：c1={a2.state} c2={b2.state}",
            "assertions": assertions, "detail": "core.py:apply_waivers"}


if __name__ == "__main__":
    import json
    print(json.dumps(run(), ensure_ascii=False, indent=1))
