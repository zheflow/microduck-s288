#!/usr/bin/env python3
"""内核反例（09-13 E-16）—— 反例缓存 key 必须静态全盖 + 运行时清单兜底。

以前 NEG_KEY_GLOBS 只有 negatives/layers/core/gate/tool_access/slice_l1 + data/*.yaml：反例经 FakeCtx.mjcf() 走
duckstructure/kin.py、L6/L7 反例走 duckstructure/lib.py、切片桩读 slicing/*.yaml、标定反例读 upstream 网格 ——
这些一变缓存照旧命中。现在验：
  ① 静态 glob 覆盖 tools/gate/**/*.py、duckstructure/*.py、tools/cad/*.py、data/*.yaml、slicing/*.yaml，
     清单里真有 duckstructure/lib.py、kin.py、slicing/slice_run.yaml、标定集网格路径（不存在也进 key）；
  ② 改一个 duckstructure/*.py 的内容 key 变 —— 在**临时副本**上：把 duckstructure/lib.py 复制到反例临时目录，
     经 extra 进 key；改副本一个字节 key 必须变（不碰仓库源码）；
  ③ 版本字符串进 key：versions 不同 key 不同；默认 key == 传当前版本串算的 key；
  ④ 运行时清单：runtime_manifest() 含 tools/gate/gate.py / core.py、不含 .venv；清单里某文件 sha 变 → runtime_manifest_valid False；
  ⑤ 命中判定端到端：把 NEG_CACHE 指到临时文件、runner.run_all 换成假的 → 静态 key 相同且清单未变 → cached=True；
     改清单里的文件 → 不命中、重跑、新缓存带 runtime_manifest。
全部 assertions（不经过层）。
"""
from __future__ import annotations

import json
import shutil
import sys
import time
import types
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parent))
import core  # noqa: E402
from _harness import _TMP  # noqa: E402

NAME = "反例缓存 key：静态全盖（duckstructure/tools/cad/slicing yaml/标定网格）+ 版本号 + 运行时清单兜底"
EXPECT = "改临时副本的 duckstructure/*.py 内容 → key 变；版本串进 key；运行时文件 sha 变 → 不命中"


def run() -> dict:
    import gate
    probe_dir = _TMP / "cache_key_probe"
    probe_dir.mkdir(parents=True, exist_ok=True)
    assertions = []

    # ① 静态 glob 覆盖
    globs = set(gate.NEG_KEY_GLOBS)
    files = gate.negatives_key_files()
    rels = set()
    for p in files:
        try:
            rels.add(str(p.resolve().relative_to(core.ROOT)))
        except ValueError:
            rels.add(str(p))
    ok1 = ({"tools/gate/**/*.py", "duckstructure/*.py", "tools/cad/*.py", "tools/gate/data/*.yaml", "tools/gate/slicing/*.yaml"} <= globs
           and {"duckstructure/lib.py", "duckstructure/kin.py", "tools/gate/slicing/slice_run.yaml",
                "tools/gate/data/l1_calibration.yaml", "tools/gate/negatives/_harness.py"} <= rels
           and any(r.startswith("upstream/") for r in rels))
    assertions.append({"claim": "静态 glob 覆盖 tools/gate/**、duckstructure、tools/cad、data/slicing yaml、标定网格", "ok": bool(ok1)})

    # ② 改一个 duckstructure/*.py 的内容（临时副本）→ key 变
    src = core.ROOT / "duckstructure" / "lib.py"
    cp = probe_dir / "lib_copy.py"
    shutil.copyfile(src, cp)
    k1 = gate.negatives_cache_key(extra=[cp])
    cp.write_text(cp.read_text(encoding="utf-8") + "\n# neg probe\n", encoding="utf-8")
    k2 = gate.negatives_cache_key(extra=[cp])
    k1b = gate.negatives_cache_key(extra=[cp])
    assertions.append({"claim": "duckstructure/lib.py 副本改一字节 → key 变（同内容两次算相同）", "ok": bool(k1 != k2 and k2 == k1b and len(k1) == 64)})

    # ③ 版本串进 key
    kv1 = gate.negatives_cache_key(versions="python=3.0;trimesh=1")
    kv2 = gate.negatives_cache_key(versions="python=3.0;trimesh=2")
    kdef = gate.negatives_cache_key()
    kcur = gate.negatives_cache_key(versions=gate._versions_string())
    assertions.append({"claim": "版本字符串进 key（不同版本不同 key；默认 = 当前版本串）",
                       "ok": bool(kv1 != kv2 and kdef == kcur and "trimesh=" in gate._versions_string())})

    # ④ 运行时清单
    man = gate.runtime_manifest()
    ok4a = "tools/gate/gate.py" in man and "tools/gate/core.py" in man and not any(".venv" in k for k in man)
    rt_file = probe_dir / "rt_probe.py"
    rt_file.write_text("X = 1\n", encoding="utf-8")
    rt_rel = str(rt_file.relative_to(core.ROOT))
    small = {rt_rel: core.sha256_file(rt_file)}
    ok_same, _ = gate.runtime_manifest_valid(small)
    rt_file.write_text("X = 2\n", encoding="utf-8")
    ok_diff, changed = gate.runtime_manifest_valid(small)
    assertions.append({"claim": "runtime_manifest 含 gate/core、不含 .venv；清单文件 sha 变 → valid=False 并点名",
                       "ok": bool(ok4a and ok_same and not ok_diff and changed == [rt_rel])})

    # ⑤ 命中判定端到端（假 runner + 临时 NEG_CACHE）
    saved_cache = gate.NEG_CACHE
    saved_mods = {k: sys.modules.get(k) for k in ("negatives", "negatives.runner")}
    fake_runner = types.ModuleType("negatives.runner")
    calls = {"n": 0}

    def fake_run_all(verbose=True):
        calls["n"] += 1
        return {"all_passed": True, "n": 0, "rows": [], "note": "fake runner（反例 n_core_neg_cache_key）"}
    fake_runner.run_all = fake_run_all
    fake_pkg = types.ModuleType("negatives"); fake_pkg.__path__ = []; fake_pkg.runner = fake_runner
    ok5 = False
    import contextlib, io
    try:
        _quiet = contextlib.redirect_stdout(io.StringIO()); _quiet.__enter__()   # run_negatives 会 print 一行"重跑"，别污染 JSON 输出
        gate.NEG_CACHE = probe_dir / "negatives.cache.json"
        sys.modules["negatives"] = fake_pkg; sys.modules["negatives.runner"] = fake_runner
        rt_file.write_text("X = 3\n", encoding="utf-8")
        key_now = gate.negatives_cache_key()
        gate.NEG_CACHE.write_text(json.dumps({"key": key_now, "when": "2026-09-13 00:00:00",
                                              "runtime_manifest": {rt_rel: core.sha256_file(rt_file)},
                                              "result": {"all_passed": True, "n": 0, "note": "旧结果"}}), encoding="utf-8")
        hit = gate.run_negatives(use_cache=True)
        hit_ok = hit.get("cached") is True and calls["n"] == 0 and hit.get("cached_when") == "2026-09-13 00:00:00"
        rt_file.write_text("X = 4\n", encoding="utf-8")            # 静态 key 不变（探针不在 glob 里），运行时清单变
        miss = gate.run_negatives(use_cache=True)
        new_cache = json.loads(gate.NEG_CACHE.read_text(encoding="utf-8"))
        miss_ok = (miss.get("cached") is False and calls["n"] == 1
                   and "tools/gate/gate.py" in (new_cache.get("runtime_manifest") or {})
                   and new_cache.get("key") == key_now and "versions" in new_cache)
        # 旧格式缓存（没有 runtime_manifest）不许命中
        gate.NEG_CACHE.write_text(json.dumps({"key": key_now, "when": "x", "result": {"all_passed": True, "n": 0}}), encoding="utf-8")
        old_fmt = gate.run_negatives(use_cache=True)
        old_ok = old_fmt.get("cached") is False and calls["n"] == 2
        ok5 = hit_ok and miss_ok and old_ok
    finally:
        _quiet.__exit__(None, None, None)
        gate.NEG_CACHE = saved_cache
        for k, v in saved_mods.items():
            if v is None:
                sys.modules.pop(k, None)
            else:
                sys.modules[k] = v
    assertions.append({"claim": "命中 = 静态 key 相同且运行时清单未变；清单文件变 → 重跑并写新清单；无清单的旧缓存不命中", "ok": bool(ok5)})

    for f in (cp, rt_file):
        try:
            f.unlink()
        except OSError:
            pass
    return {"name": NAME, "passed": all(a["ok"] for a in assertions), "expect": EXPECT,
            "got": "；".join(f"{a['claim'][:14]}={'ok' if a['ok'] else 'FAIL'}" for a in assertions),
            "assertions": assertions,
            "detail": f"静态 key 文件 {len(files)} 个；运行时清单 {len(man)} 个模块文件；版本串 {gate._versions_string()}"}


if __name__ == "__main__":
    r = run()
    print(json.dumps(r, ensure_ascii=False, indent=1))
    sys.exit(0 if r["passed"] else 1)
