#!/usr/bin/env python3
"""反例（09-13 F-核-5 层侧）—— 退役判据不许再写成 `state=STALE`。

core 里 STALE 的语义是"输入变了没重跑"（继承逻辑可恢复、verdict 记 INCOMPLETE）；
l6_motion / l7_mass 把"基准失去权威、保留为记录"的五条判据硬编码成 STALE，
结果整机 verdict 永远 INCOMPLETE、tally[STALE] 说不清哪些格子真过期。
core 已加 RETIRED（n_core_retired.py 验内核侧），本反例验**层侧**：

  · core.RETIRED 存在；
  · 四处 res.add(...) —— l6 range_matches_upstream / upstream_collision_geom_coverage，
    l7 mjcf_baseline_authority / drift_criteria —— 的 state 是 RETIRED，不是 STALE；
    （原第五处 l6 policy_box 于 2026-09-16 被能力判据 _capability/* 取代、整条删除 —— 包络成了能力要求，
    不再是"失去权威的历史参考"；见 n_l6_capability_envelope.py）
  · 两个层文件里 `state=STALE` 一处都不剩（用 ast 找 res.add 调用的 state 关键字，不靠正则猜）；
  · 五条的 criterion 文本带"已退役"前缀（人读记分卡时不会误以为是"输入变了"）。

用 assertions 契约（内核级 / 静态反例），不造几何。
"""
from __future__ import annotations

import ast
import sys
from pathlib import Path

HERE = Path(__file__).resolve()
GATE = HERE.parents[1]
sys.path.insert(0, str(GATE))

NAME = "l6/l7 退役判据用 RETIRED 不用 STALE（静态检查四处 res.add）"
EXPECT = "四处 state=RETIRED、criterion 带【已退役】、层里再无 state=STALE；core.RETIRED 存在"

TARGETS = {
    "layers/l6_motion.py": ["range_matches_upstream", "upstream_collision_geom_coverage"],
    "layers/l7_mass.py": ["mjcf_baseline_authority", "drift_criteria"],
}


def _kw(call: ast.Call, name: str):
    for k in call.keywords:
        if k.arg == name:
            return k.value
    return None


def _const_str(node) -> str | None:
    if isinstance(node, ast.Constant) and isinstance(node.value, str):
        return node.value
    if isinstance(node, ast.JoinedStr):
        return "".join(v.value for v in node.values if isinstance(v, ast.Constant) and isinstance(v.value, str))
    if isinstance(node, ast.BinOp) and isinstance(node.op, ast.Add):
        a, b = _const_str(node.left), _const_str(node.right)
        return (a or "") + (b or "") if (a is not None or b is not None) else None
    return None


def _scan(rel: str):
    """返回 ({check: (state_name, criterion_text)}, [所有 state=STALE 的行号])。"""
    src = (GATE / rel).read_text(encoding="utf-8")
    tree = ast.parse(src)
    found, stale_lines = {}, []
    for node in ast.walk(tree):
        if not isinstance(node, ast.Call):
            continue
        fn = node.func
        if not (isinstance(fn, ast.Attribute) and fn.attr == "add"):
            continue
        st = _kw(node, "state")
        st_name = st.id if isinstance(st, ast.Name) else None
        if st_name == "STALE":
            stale_lines.append(node.lineno)
        chk = _const_str(_kw(node, "check")) if _kw(node, "check") is not None else None
        if chk:
            found[chk] = (st_name, _const_str(_kw(node, "criterion")) or "", node.lineno)
    return found, stale_lines


def run() -> dict:
    assertions = []
    try:
        import core
        has_retired = isinstance(getattr(core, "RETIRED", None), str) and "RETIRED" in core.STATES
    except Exception as e:  # noqa: BLE001
        has_retired = False
        assertions.append({"claim": f"import core 失败：{e}", "ok": False})
    assertions.append({"claim": "core.RETIRED 存在且在 core.STATES 里", "ok": bool(has_retired)})
    got = []
    for rel, checks in TARGETS.items():
        found, stale_lines = _scan(rel)
        assertions.append({"claim": f"{rel} 里 res.add(state=STALE) 一处都不剩（现有行 {stale_lines}）",
                           "ok": not stale_lines})
        for c in checks:
            st, crit, ln = found.get(c, (None, "", None))
            assertions.append({"claim": f"{rel}:{ln} {c} state=RETIRED（现为 {st}）", "ok": st == "RETIRED"})
            assertions.append({"claim": f"{rel}:{ln} {c} criterion 以【已退役 开头",
                               "ok": crit.startswith("【已退役")})
            got.append(f"{c}={st}")
    return {"name": NAME, "passed": all(a["ok"] for a in assertions), "expect": EXPECT,
            "got": "；".join(got), "assertions": assertions,
            "detail": "静态 ast 扫 res.add 的 state / check / criterion 关键字"}


if __name__ == "__main__":
    import json
    r = run()
    print(json.dumps(r, ensure_ascii=False, indent=1))
    sys.exit(0 if r["passed"] else 1)
