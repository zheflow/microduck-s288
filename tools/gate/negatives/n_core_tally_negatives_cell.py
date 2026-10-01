#!/usr/bin/env python3
"""内核反例（hr50 run2 记账 bug，2026-09-28）：L0/_negatives 这一格是 build_scorecard 之后才补进记分卡的，补格必须同步
tally / blocking_cells / verdict。事故：run2 85 个反例全过，全过分支只写格子、没给 tally[PASS] +1 → 记分卡 ✅169 而 628 格实际 170
（失败分支给 tally[FAIL] +1，所以 run1 有反例失败时没暴露）。子集跑还可能从 prev 继承上一轮的 L0/_negatives，覆盖时要先把旧状态从计数里拿掉。
造法同 n_core_layer_error（假层 + 最小数据 + 反例包桩 + 真 gate.main，--out 反例临时目录）：
  P 全量、反例包全过 → L0/_negatives PASS；sum(tally) == 格数；tally 逐状态 == 按格子数出来的；不在 blocking_cells；CLEAR、rc 0；
  F 全量、反例包 1 个失败 → 格 FAIL；sum(tally) == 格数；tally[FAIL] == FAIL 格数；在 blocking_cells；BLOCKED、rc 1；
  S 紧接着在 F 的 out 目录子集跑（--layers 3，反例包全过）→ 从 prev 继承的 FAIL 格被本轮 PASS 覆盖：sum(tally) == 格数、
    tally[FAIL] == FAIL 格数（旧 FAIL 不能留在计数里）、不在 blocking_cells。
旧 gate.py：P 的 tally[PASS] 比格数少 1（sum(tally) ≠ 格数）→ 本反例红。"""
from __future__ import annotations

import contextlib
import io
import json
import os
import shutil
import sys
from collections import Counter
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))   # tools/gate
sys.path.insert(0, str(Path(__file__).resolve().parent))      # negatives/

NAME = "L0/_negatives 补格同步 tally / blocking_cells / verdict：全过、失败、子集继承三种情形 sum(tally) == 格数"
EXPECT = ("P 全过：格 PASS、tally 逐状态 == 格子实数、CLEAR rc 0；F 失败：格 FAIL、tally[FAIL] 对、在 blocking、BLOCKED rc 1；"
          "S 子集继承旧 FAIL 格再覆盖成 PASS：计数不残留旧状态、不在 blocking")


def _neg_stub(all_ok):
    rows = [{"file": "n_stub_a.py", "passed": True}, {"file": "n_stub_b.py", "passed": bool(all_ok)}]
    return lambda use_cache: {"all_passed": bool(all_ok), "n": 2, "rows": rows, "note": "反例包桩（n_core_tally_negatives_cell）",
                              "cached": False, "seconds": 0.0}


def _counts(sc):
    return dict(Counter(c["state"] for c in (sc.get("cells") or {}).values()))


def run() -> dict:
    import gate
    import core
    from _harness import _TMP, result
    import n_core_layer_error as me
    root = _TMP / "tally_negatives_cell"
    shutil.rmtree(root, ignore_errors=True)
    saved = {"discover": gate.discover, "load_data": gate.load_data, "task": gate._run_layer_task,
             "neg": gate._run_negatives_task, "git": core.git_head}
    installed, res = set(), {}

    def scenario(tag, out, names, argv, all_ok):
        def fake_discover(want=None):
            mods = me._install([nm for nm in names if not want or int(nm[1]) in want])
            installed.update(f"layers.{nm}" for nm in names)
            return mods, {}
        gate.discover = fake_discover
        gate._run_negatives_task = _neg_stub(all_ok)
        buf, rc, err = io.StringIO(), None, None
        try:
            with contextlib.redirect_stdout(buf):
                rc = gate.main(argv + ["--out", str(out), "--no-check-cache"])
        except (Exception, SystemExit) as e:          # noqa: BLE001
            err = f"{type(e).__name__}: {e}"
        stem = "scorecard.partial" if "--layers" in argv else "scorecard"
        try:
            sc = json.loads((out / f"{stem}.json").read_text(encoding="utf-8"))
        except OSError:
            sc = {"cells": {}, "blocking_cells": [], "verdict": None, "tally": {}}
        res[tag] = dict(rc=rc, err=err, sc=sc)
        return res[tag]

    try:
        gate.load_data = me._mini_data
        core.git_head = lambda: "neg-stub"
        P = scenario("P", root / "P", me.ALL_OK, ["--jobs", "1"], True)
        F = scenario("F", root / "F", me.ALL_OK, ["--jobs", "1"], False)
        S = scenario("S", root / "F", ["l3_negfake_ok"], ["--jobs", "1", "--layers", "3"], True)
    finally:
        gate.discover, gate.load_data, gate._run_layer_task = saved["discover"], saved["load_data"], saved["task"]
        gate._run_negatives_task, core.git_head = saved["neg"], saved["git"]
        for k in installed:
            sys.modules.pop(k, None)

    def cell(r):
        return (r["sc"].get("cells") or {}).get("L0/_negatives") or {}

    def tally_ok(r):
        t = r["sc"].get("tally") or {}
        return bool(t) and sum(t.values()) == len(r["sc"].get("cells") or {}) and all(t.get(s, 0) == n for s, n in _counts(r["sc"]).items())

    a = [{"claim": "P 全过：L0/_negatives PASS；sum(tally) == 格数且逐状态与格子相符；不在 blocking_cells；CLEAR、rc 0",
          "ok": (P["err"] is None and cell(P).get("state") == "PASS" and tally_ok(P)
                 and "L0/_negatives" not in (P["sc"].get("blocking_cells") or []) and P["sc"].get("verdict") == "CLEAR" and P["rc"] == 0)},
         {"claim": "F 一个反例失败：格 FAIL；sum(tally) == 格数且 tally[FAIL] == FAIL 格数；在 blocking_cells；BLOCKED、rc 1",
          "ok": (F["err"] is None and cell(F).get("state") == "FAIL" and tally_ok(F)
                 and "L0/_negatives" in (F["sc"].get("blocking_cells") or []) and F["sc"].get("verdict") == "BLOCKED" and F["rc"] == 1)},
         {"claim": "S 子集跑继承 F 的 FAIL 格再被本轮 PASS 覆盖：格 PASS；sum(tally) == 格数且逐状态相符（旧 FAIL 不残留）；不在 blocking_cells",
          "ok": (S["err"] is None and cell(S).get("state") == "PASS" and tally_ok(S)
                 and "L0/_negatives" not in (S["sc"].get("blocking_cells") or []))}]
    got = "；".join(f"{k}: rc={v['rc']} err={v['err']} verdict={v['sc'].get('verdict')} tally={v['sc'].get('tally')} cells={_counts(v['sc'])} "
                   f"neg={cell(v).get('state')} blocking={'L0/_negatives' in (v['sc'].get('blocking_cells') or [])}" for k, v in res.items())
    out = result(NAME, EXPECT, all(x["ok"] for x in a), got, [], detail="gate.py: L0/_negatives 补格 → _set_negatives_cell 同步 tally / blocking_cells / verdict")
    out["assertions"] = a
    return out


if __name__ == "__main__":
    print(json.dumps(run(), ensure_ascii=False, indent=1))
