"""坏样本：豁免没有绑定输入 hash，或绑定的输入已经变了。

历史教训：waivers.yaml 的 schema 写 bound_input_hashes（复数），core 只读单数 →
按文档填的豁免拿到空 dict，等于无条件生效。审查 FG04。
"""
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from core import LayerResult, apply_waivers, FAIL, WAIVED, WARN   # noqa: E402

NAME = "无绑定 / 绑定已失效的豁免必须被拒绝"


def _f():
    r = LayerResult(5, "功能达成")
    return r.add(subject="X", check="c", state=FAIL, severity=WARN,
                 measured=1, criterion="t", evidence_n=1)


def run() -> dict:
    manifest = {"tools/gate/data/parts.yaml": "AAA"}
    unbound = _f()
    apply_waivers([unbound], {"L5/X:c": {"reason": "r", "issued_by": "u", "date": "d"}}, manifest)
    stale = _f()
    apply_waivers([stale], {"L5/X:c": {"reason": "r", "bound_input_hashes":
                                     {"tools/gate/data/parts.yaml": "OLD"}}}, manifest)
    good = _f()
    apply_waivers([good], {"L5/X:c": {"reason": "r", "bound_input_hashes":
                                    {"tools/gate/data/parts.yaml": "AAA"}}}, manifest)
    ok = unbound.state == FAIL and stale.state == FAIL and good.state == WAIVED
    assertions = [{"claim": "无绑定 → 拒绝", "ok": unbound.state == FAIL},
                  {"claim": "绑定已变 → 失效", "ok": stale.state == FAIL},
                  {"claim": "绑定一致（仓库相对路径 → sha256）→ WAIVED", "ok": good.state == WAIVED}]
    return {"name": NAME, "passed": ok,
            "expect": "无绑定→FAIL；绑定已变→FAIL；绑定一致→WAIVED（豁免 id 必须带判据名，见 n_core_waiver_cell_only）",
            "got": f"无绑定={unbound.state} 失效={stale.state} 有效={good.state}",
            "assertions": assertions, "detail": "core.py:apply_waivers"}


if __name__ == "__main__":
    import json
    print(json.dumps(run(), ensure_ascii=False, indent=1))
