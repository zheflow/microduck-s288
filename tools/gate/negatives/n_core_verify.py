"""坏样本：记分卡生成后，它绑定的某个输入文件被改了。

历史教训（审计 2026-09-13 F-核-3）：`source_manifest` 只写进记分卡，没有任何工具把它和磁盘比。
"hash 对不上的记分卡作废"（元规则 6）靠人眼。修法：gate.py --verify。
同时验反例缓存 key：反例/层/内核/数据任一文件变了 key 必须变。
"""
import json
import sys
import tempfile
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import core  # noqa: E402

NAME = "gate.py --verify 必须抓到记分卡绑定输入已变；反例缓存 key 随文件内容变"
EXPECT = "输入文件改动 → verify 报 changed 且 ok=False；未改 → ok=True；缓存 key 改文件后不同"


def run() -> dict:
    from gate import verify_scorecard, negatives_cache_key
    rel_dir = core.ROOT / "tools/gate/out/_neg_tmp"
    rel_dir.mkdir(parents=True, exist_ok=True)
    f = rel_dir / "verify_probe.txt"
    f.write_text("v1", encoding="utf-8")
    rel = str(f.relative_to(core.ROOT))
    sc = {"git": core.git_head(), "source_manifest": {rel: core.sha256_file(f)},
          "cells": {"L0/A": {"layer": 0, "subject": "A", "state": "PASS", "checks": [],
                             "inputs_hash": {rel: core.sha256_file(f)}}}}
    same = verify_scorecard(sc)
    f.write_text("v2", encoding="utf-8")
    diff = verify_scorecard(sc)
    a_ok = same["ok"] is True and not same["changed"]
    b_ok = diff["ok"] is False and rel in diff["changed"] and "L0/A" in diff["stale_cells"]

    k1 = negatives_cache_key(extra=[f])
    f.write_text("v3", encoding="utf-8")
    k2 = negatives_cache_key(extra=[f])
    c_ok = k1 != k2 and len(k1) == 64
    try:
        f.unlink()
    except OSError:
        pass
    assertions = [{"claim": "未改 → ok", "ok": bool(a_ok)}, {"claim": "改了 → changed + stale_cells", "ok": bool(b_ok)},
                  {"claim": "缓存 key 随内容变", "ok": bool(c_ok)}]
    return {"name": NAME, "passed": all(x["ok"] for x in assertions), "expect": EXPECT,
            "got": f"未改 ok={same['ok']}；改后 ok={diff['ok']} changed={diff['changed'][:1]} stale={diff['stale_cells'][:1]}；key 变={k1 != k2}",
            "assertions": assertions, "detail": "gate.py:verify_scorecard / negatives_cache_key"}


if __name__ == "__main__":
    print(json.dumps(run(), ensure_ascii=False, indent=1))
