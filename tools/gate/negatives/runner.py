#!/usr/bin/env python3
"""反例回归包（元规则 8）。

    ./.venv/bin/python tools/gate/negatives/runner.py            # 全包（≈7 min，gate.py 会按 key 缓存）
    ./.venv/bin/python tools/gate/negatives/n_xxx.py             # 单个反例

**一个从来没红过的检查器等于没有。** 每个 `n_*.py` 造一个明确的坏样本，
证明"指定判据会在指定严重级变红、且真的量到了东西"；同时必须证明"好样本仍然绿"。

模块契约（2026-09-13 起，由本 runner 校验；缺一项 = 失败，元规则 4）：
    NAME = "一句话说这个坏样本是什么"
    def run() -> dict:
        {"name": str, "passed": bool, "expect": str, "got": str, "detail": str,
         "expect_severity": "BLOCK" | "WARN",     # 坏样本应该红在哪一级
         "red": [{"check","state","severity","measured","evidence_n"}, ...]}   # _harness.assert_red 给
        内核级反例可以用 "assertions": [{"claim": str, "ok": bool}, ...] 代替 red。
    校验：
      · red 非空；每条 state == FAIL、severity == expect_severity；
      · 每条 measured 非 None —— unknown（算不出来）不算抓到缺陷；除非模块给出
        allow_unknown_red="<理由>"（专门验"坏声明必须 unknown"的反例）；
      · 或 assertions 非空且全 ok。
      · 空壳（passed=True 但既无 red 也无 assertions）算失败。
    以前只看 state：min_wall_geometric / support_reachable 被降成 WARN 之后反例照过，
    "翻回 BLOCK 也没人抓"发生过（审计 2026-09-13 F-反-1）。

Gate 不得在反例包未全过的情况下判 CLEAR。
"""
from __future__ import annotations
import importlib.util, json, sys, time, traceback
from pathlib import Path

HERE = Path(__file__).resolve().parent
for _p in (str(HERE), str(HERE.parent), str(HERE.parents[2])):   # negatives/、tools/gate、仓库根
    if _p not in sys.path:
        sys.path.insert(0, _p)

SEVERITIES = ("BLOCK", "WARN", "INFO")


def _load(p: Path):
    spec = importlib.util.spec_from_file_location(f"neg_{p.stem}", p)
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m


def check_contract(r: dict) -> str | None:
    """返回违反契约的原因；None = 合规。"""
    if not isinstance(r, dict) or "passed" not in r:
        return "模块没有返回 passed 字段 → 结果不明 = 失败（元规则 4）"
    for k in ("name", "expect", "got"):
        if not r.get(k):
            return f"缺 {k} 字段"
    red, asserts = r.get("red"), r.get("assertions")
    if asserts:
        bad = [a for a in asserts if not isinstance(a, dict) or "claim" not in a or "ok" not in a]
        if bad:
            return "assertions 里有条目缺 claim/ok"
        failed = [a["claim"] for a in asserts if not a["ok"]]
        if failed:
            return "断言未成立：" + "；".join(str(x) for x in failed[:3])
        if not red:
            return None                     # 纯内核级反例
        # 两者都给（"红 + 已知缺口"混合文件）→ red 也要按下面的规则校验，不因为有 assertions 就放过
    if not red:
        return "空壳：既没有 red（坏样本上红了的判据）也没有 assertions —— 什么都没证明"
    sev = r.get("expect_severity")
    if sev not in SEVERITIES:
        return f"expect_severity 必须是 {SEVERITIES}，现在是 {sev!r}"
    for x in red:
        if not isinstance(x, dict) or x.get("state") != "FAIL":
            return f"red 里有一条不是 FAIL：{x}"
        if x.get("severity") != sev:
            return (f"红了但严重级是 {x.get('severity')}，期望 {sev}（判据被降级/升级了，反例必须跟着改）：{x.get('check')}")
        if x.get("measured") is None and not r.get("allow_unknown_red"):
            return f"red 里 {x.get('check')} 的 measured 为空 = unknown，不算抓到缺陷（要验 unknown 路径请给 allow_unknown_red）"
    return None


def run_all(verbose: bool = True) -> dict:
    rows = []
    for p in sorted(HERE.glob("n_*.py")):
        t0 = time.perf_counter()
        try:
            r = _load(p).run()
            if not isinstance(r, dict):
                r = {"name": p.stem, "passed": False, "expect": "?", "got": "run() 没返回 dict"}
            why = check_contract(r)
            if why:
                r = dict(r); r["passed"] = False
                r["got"] = f"[契约] {why} ｜ 模块自报：{r.get('got', '')}"
        except Exception:
            r = {"name": p.stem, "passed": False, "expect": "-",
                 "got": "反例本身抛异常", "detail": traceback.format_exc(limit=4)}
        r["file"] = p.name
        r["seconds"] = round(time.perf_counter() - t0, 2)
        rows.append(r)
        if verbose:
            print(f"  {'✅' if r['passed'] else '❌'} {r['file']:36s} {str(r.get('name'))[:70]}  ({r['seconds']} s)")
            if not r["passed"]:
                print(f"       期望: {r.get('expect','')}\n       实际: {str(r.get('got',''))[:400]}")
    ok = all(r["passed"] for r in rows)
    return {"all_passed": ok and bool(rows), "n": len(rows), "rows": rows,
            "note": "" if rows else "negatives/ 里一个反例都没有 —— 检查器从未证明过自己会红"}


if __name__ == "__main__":
    print("── 反例回归包")
    t0 = time.perf_counter()
    res = run_all()
    print(f"\n{res['n']} 个反例，{'全部通过' if res['all_passed'] else '有失败'}，{time.perf_counter() - t0:.1f} s"
          + (f"　{res['note']}" if res["note"] else ""))
    sys.exit(0 if res["all_passed"] else 1)
