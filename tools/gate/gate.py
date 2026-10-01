#!/usr/bin/env python3
"""Gate 入口。

    ./.venv/bin/python tools/gate/gate.py                  # 跑所有已实现的层
    ./.venv/bin/python tools/gate/gate.py --layers 0,2,5   # 只跑某几层（子集跑）
    ./.venv/bin/python tools/gate/gate.py --parts L01,H02  # 只查某几个件（子集跑）
    ./.venv/bin/python tools/gate/gate.py --out /tmp/sc    # 换输出前缀
    ./.venv/bin/python tools/gate/gate.py --verify         # 核对 out/scorecard.json 绑定的输入是否还在磁盘上原样（元规则 6）

产出：全量跑 → tools/gate/out/scorecard.{json,md} + impact.md，并更新 scorecard.prev.json；
      子集跑（--layers / --parts）→ scorecard.partial.{json,md}，**不写 prev**，范围外的格子按 hash 从 prev 继承。
退出码：0 = CLEAR；1 = 有 BLOCK；2 = 无 BLOCK 但有 NOT_RUN/STALE（不完整，不算通过）；
        --verify：0 = 记分卡仍有效；3 = 绑定输入已变 / git 不同。
反例包：每次先跑（元规则 8）。结果缓存到 out/negatives.cache.json（E-16，2026-09-13）：
        静态 key = sha256(NEG_KEY_GLOBS 全部文件：tools/gate/**/*.py、duckstructure/*.py、tools/cad/*.py、
        tools/gate/data/*.yaml、tools/gate/slicing/*.yaml、标定集网格 + Python/trimesh/manifold3d/numpy/shapely 版本号)；
        运行时清单 = 反例跑完后 sys.modules 里落在仓库内的每个模块文件的 sha256。
        命中 = 静态 key 相同 **且** 清单里每个文件现在的 sha256 仍相同；命中时 L0/_negatives 的 checks[0] 标 cached: true + 命中时间。
层守门（gate_guard，2026-09-25）：层模块崩溃（抛异常 / 层进程被杀、进程池断 / 返回不可解析 / import 失败）→ 记分卡加 L<n>/_layer_error
        （FAIL，BLOCK），终端摘要顶部与记分卡 md 标题下各一行 ⚠ 报错，结论 BLOCKED、退出码 1；全量跑时层模块 0 条判据（空跑）同样加格，
        子集跑（--layers/--parts）只打印提示行。没有触发时不加格、不加行（正常运行记分卡逐格不变）。反例 negatives/n_core_layer_error.py。
"""
from __future__ import annotations
import argparse, hashlib, importlib, json, sys, time, traceback
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parents[1]))       # 仓库根，为了 import duckstructure

from core import (ROOT, STL_DIR, PLACED, LayerResult, LAYER_NAMES, load_data, load_waivers,   # noqa: E402
                  apply_waivers, source_manifest, build_scorecard, render_markdown, sha256_file,
                  git_head, PASS, FAIL, STALE, NOT_RUN, WAIVED, RETIRED, placed_instance_map)


class Ctx:
    def __init__(self, data, only=None):
        self.data = data
        self.only = set(only) if only else None
        self.parts = [p["id"] for p in (data.get("parts", {}).get("parts") or [])]
        self._mesh, self._mjcf, self._placed_alias = {}, None, {}
        self._stl_index = {p.stem: p for p in STL_DIR.glob("*.stl")}
        self._placed_index = {p.stem: p for p in PLACED.glob("*.stl")} if PLACED.exists() else {}

    def stl(self, pid):
        for stem, p in self._stl_index.items():
            if stem.startswith(pid + "_") or stem == pid:
                return p
        return None

    def placed(self, name):
        return self._placed_index.get(name)

    def placed_for(self, pid):
        """件号 → placed/ 里的世界坐标件。placed 名跟件号对不上
        （L06_sole_TPU→sole、N03_yaw_roll→yrm），所以用 parts.yaml 的
        inventory_id / build_fn 搭桥，全失败才返回 None（调用方判 unknown）。"""
        if pid in self._placed_alias:
            return self._placed_index.get(self._placed_alias[pid])
        rec = next((p for p in (self.data.get("parts", {}).get("parts") or []) if p["id"] == pid), None)
        if rec and "placed_instances" in rec:
            mapping, errors = placed_instance_map(self.data["parts"]["parts"], self._placed_index)
            if errors:
                return None
            declared = mapping[pid]
            self._placed_alias[pid] = declared[0]
            return self._placed_index[declared[0]]
        cands = []
        if rec:
            inv = (rec.get("inventory_id") or "")
            cands += [inv, inv.split("_", 1)[-1] if "_" in inv else inv]
            for fn in (rec.get("build_fn") or []):
                cands.append(fn.replace("build_", ""))
        cands.append(pid)
        for cand in cands:
            if cand in self._placed_index:
                self._placed_alias[pid] = cand
                return self._placed_index[cand]
        return None

    def mesh(self, path, solid=False):
        """solid=False：process=False，顶点不合并 —— 精确网格统计用（L0）。
        solid=True：process=True，**布尔层必须用这个** —— process=False 载进来的网格
        is_volume 为假，trimesh/manifold3d 会直接拒（Not all meshes are volumes）。"""
        k = (str(path), solid)
        if k not in self._mesh:
            import trimesh
            self._mesh[k] = trimesh.load(str(path), process=bool(solid))
        return self._mesh[k]

    def solid(self, path):
        return self.mesh(path, solid=True)

    def mjcf(self):
        if self._mjcf is None:
            from duckstructure import kin
            self._mjcf = kin.load()
        return self._mjcf


def discover(want=None):
    """按需加载。一层可以挂多个模块（判据可以分文件写，互不影响）；
    某个模块 import 失败只让那一层变红，不炸掉整轮。"""
    mods, broken = {}, {}
    for p in sorted((HERE / "layers").glob("l[0-7]_*.py")):
        n = int(p.stem[1])
        if want and n not in want:
            continue
        try:
            m = importlib.import_module(f"layers.{p.stem}")
            mods.setdefault(getattr(m, "LAYER", n), []).append(m)
        except Exception:
            broken.setdefault(n, []).append(f"{p.name}:\n" + traceback.format_exc(limit=4))
    return mods, broken


# ── 反例缓存（元规则 8 + 元规则 10；E-16 静态全盖 + 运行时清单）─────────────
NEG_CACHE = HERE / "out" / "negatives.cache.json"
# 静态 key 覆盖的文件。以前只有 negatives/layers/core/gate/tool_access/slice_l1 + data/*.yaml：
# 反例经 FakeCtx.mjcf() 走 duckstructure/kin.py、L6/L7 反例走 duckstructure/lib.py、切片桩走 slicing/*.yaml、
# 标定反例读 upstream 网格 —— 这些一变，缓存照旧命中（审计 E-16）。现在按 glob 全盖，再加运行时清单兜底。
NEG_KEY_GLOBS = ["tools/gate/**/*.py", "duckstructure/*.py", "tools/cad/*.py",
                 "tools/gate/data/*.yaml", "tools/gate/slicing/*.yaml"]
NEG_KEY_PACKAGES = ("trimesh", "manifold3d", "numpy", "shapely")


def _calibration_mesh_paths(data=None) -> list[str]:
    """标定反例（n_l1_calibration_record / calibrate_l1.py）读的 upstream 网格：printability.yaml:calibration_set.parts[].mesh。
    文件不存在照样进 key（hash 记 "?"），这样"网格出现/消失"也会让 key 变。"""
    try:
        if data is None:
            import yaml
            data = {"printability": yaml.safe_load((HERE / "data" / "printability.yaml").read_text(encoding="utf-8")) or {}}
        cs = (data.get("printability") or {}).get("calibration_set") or {}
        return sorted({str(p.get("mesh")) for p in (cs.get("parts") or []) if isinstance(p, dict) and p.get("mesh")})
    except Exception:
        return []


def _versions_string() -> str:
    import importlib.metadata as md
    vs = [f"python={sys.version.split()[0]}"]
    for name in NEG_KEY_PACKAGES:
        try:
            vs.append(f"{name}={md.version(name)}")
        except Exception:
            vs.append(f"{name}=?")
    return ";".join(vs)


def negatives_key_files(extra=None, data=None) -> list[Path]:
    """静态 key 覆盖的文件清单（有序、去重）。extra 允许反例塞探针文件；标定网格路径以字符串形式加入（可不存在）。"""
    files: list[Path] = []
    seen = set()
    for g in NEG_KEY_GLOBS:
        for p in sorted(ROOT.glob(g)):
            if p.is_file() and "/out/" not in str(p) and str(p) not in seen:
                seen.add(str(p)); files.append(p)
    for rel in _calibration_mesh_paths(data):
        p = ROOT / rel
        if str(p) not in seen:
            seen.add(str(p)); files.append(p)
    for p in (extra or []):
        p = Path(p)
        if str(p) not in seen:
            seen.add(str(p)); files.append(p)
    return files


def negatives_cache_key(extra=None, data=None, versions: str | None = None) -> str:
    """反例结论只在"反例 + 检查器 + 判据数据 + 上游几何库 + 依赖版本"一个字节都没变时才可沿用。
    versions 默认取当前解释器/包版本；反例可传字串验"版本进 key"。"""
    h = hashlib.sha256()
    for p in negatives_key_files(extra, data):
        try:
            rel = str(p.resolve().relative_to(ROOT))
        except ValueError:
            rel = str(p)
        h.update(rel.encode("utf-8")); h.update(b"\0")
        sha = sha256_file(p)
        h.update(sha.encode("ascii") if sha else b"?"); h.update(b"\n")
    h.update(b"versions:" + (versions if versions is not None else _versions_string()).encode("utf-8"))
    return h.hexdigest()


def runtime_manifest() -> dict:
    """反例跑完后：sys.modules 里文件落在仓库内的模块 → {仓库相对路径: sha256}。
    这是"静态 glob 漏了谁"的兜底：真被 import 过的文件一个不落。"""
    out = {}
    for m in list(sys.modules.values()):
        f = getattr(m, "__file__", None)
        if not f:
            continue
        try:
            p = Path(f).resolve()
            rel = str(p.relative_to(ROOT))
        except (ValueError, OSError):
            continue
        if "/.venv/" in str(p) or "/out/" in rel:
            continue
        out[rel] = sha256_file(p)
    return dict(sorted(out.items()))


def runtime_manifest_valid(manifest: dict) -> tuple[bool, list[str]]:
    """缓存里的运行时清单现在还原样吗？返回 (ok, 变了的路径)。"""
    changed = []
    for rel, sha in (manifest or {}).items():
        if sha256_file(ROOT / rel) != sha:
            changed.append(rel)
    return not changed, changed


def run_negatives(use_cache: bool = True, jobs: int = 1) -> dict:
    """jobs（hr42）：>1 = 反例按文件分 jobs 个进程并行（只有 gate 顶层 _run_negatives_task 这样调）；默认 1 = 原来的单进程 run_all()。
    默认必须是 1：反例 n_core_neg_cache_key 会把 negatives.runner 换成假模块再调本函数，验证"未命中 → 调 run_all 重跑"，
    分进程路径不走 run_all（而且会在反例进程里再起一整套反例 → 递归）。"""
    key = negatives_cache_key()
    if use_cache and NEG_CACHE.exists():
        try:
            c = json.loads(NEG_CACHE.read_text(encoding="utf-8"))
            rt_ok, rt_changed = runtime_manifest_valid(c.get("runtime_manifest") or {})
            if c.get("key") == key and isinstance(c.get("result"), dict) and rt_ok and c.get("runtime_manifest"):
                r = dict(c["result"]); r["cached"] = True; r["cache_key"] = key; r["cached_when"] = c.get("when")
                r["note"] = (r.get("note") or "") + f"（缓存命中：{c.get('when')}，key={key[:12]}…，运行时清单 {len(c['runtime_manifest'])} 文件未变）"
                return r
            if c.get("key") == key and not rt_ok:
                print(f"   反例缓存静态 key 相同但运行时文件变了 {rt_changed[:3]} → 重跑", flush=True)
        except Exception:
            pass
    sys.path.insert(0, str(HERE / "negatives"))
    from negatives.runner import run_all          # noqa: PLC0415
    t0 = time.time()
    split = max(1, int(jobs or 1))
    if split > 1:
        r, manifest = _run_negatives_split(split)   # hr42：按文件分组多进程跑，行按文件名排序合并（与串行同序同内容）
    else:
        r = run_all()
        manifest = None
    r["cached"] = False; r["cache_key"] = key; r["seconds"] = round(time.time() - t0, 1)
    manifest = runtime_manifest() if manifest is None else manifest
    r["runtime_manifest_n"] = len(manifest)
    try:
        NEG_CACHE.parent.mkdir(parents=True, exist_ok=True)
        NEG_CACHE.write_text(json.dumps({"key": key, "when": time.strftime("%Y-%m-%d %H:%M:%S"),
                                         "versions": _versions_string(),
                                         "static_files_n": len(negatives_key_files()),
                                         "runtime_manifest": manifest,
                                         "result": {k: v for k, v in r.items() if k != "rows"} | {"failed": [
                                             x["file"] for x in r.get("rows", []) if not x.get("passed")]}},
                                        ensure_ascii=False, indent=1), encoding="utf-8")
    except OSError:
        pass
    return r


# ── hr42（2026-09-24）：反例包按文件分组、多进程并行 ─────────────────────────────
# 每个反例本来就是独立模块（runner.run_all 逐个 exec_module + run()），临时件按进程分目录（_harness：_neg_tmp/p<pid>），
# gate 原来就让反例包与各层并行跑 —— 反例之间同样可以分进程跑。每组在全新的 spawn 进程里按文件名顺序跑，
# 行合并后按文件名排序（= runner.run_all 的 sorted(glob) 顺序），契约校验用 runner.check_contract 原函数；
# 运行时清单（sys.modules 里的仓库文件）取各组并集。NEG_JOBS=1 = 原来的单进程串行。
def _neg_split():
    import os
    v = os.environ.get("NEG_JOBS", "4")
    return max(1, int(v)) if v.isdigit() else 4


def _neg_chunk(files):
    import os
    os.environ["DUCK_CHECK_CACHE"] = "0"; os.environ["L6_JOBS"] = "1"; os.environ["L1_JOBS"] = "1"; os.environ["NEG_JOBS"] = "1"
    sys.path.insert(0, str(HERE / "negatives"))
    from negatives import runner as R           # noqa: PLC0415
    rows = []
    for name in files:
        p = R.HERE / name
        t0 = time.perf_counter()
        try:
            r = R._load(p).run()
            if not isinstance(r, dict):
                r = {"name": p.stem, "passed": False, "expect": "?", "got": "run() 没返回 dict"}
            why = R.check_contract(r)
            if why:
                r = dict(r); r["passed"] = False
                r["got"] = f"[契约] {why} ｜ 模块自报：{r.get('got', '')}"
        except Exception:
            r = {"name": p.stem, "passed": False, "expect": "-",
                 "got": "反例本身抛异常", "detail": traceback.format_exc(limit=4)}
        r["file"] = p.name
        r["seconds"] = round(time.perf_counter() - t0, 2)
        rows.append(_picklable_row(r))
    return rows, runtime_manifest()


def _picklable_row(r):
    import pickle
    try:
        pickle.dumps(r); return r
    except Exception:
        return {k: (v if _picklable(v) else repr(v)) for k, v in r.items()}


def _run_negatives_split(k):
    import multiprocessing as mp
    from concurrent.futures import ProcessPoolExecutor
    files = sorted(p.name for p in (HERE / "negatives").glob("n_*.py"))
    groups = [files[i::k] for i in range(k) if files[i::k]]
    rows, manifest = [], {}
    with ProcessPoolExecutor(max_workers=len(groups), mp_context=mp.get_context("spawn")) as ex:
        for rr, man in ex.map(_neg_chunk, groups):
            rows += rr; manifest.update(man)
    rows.sort(key=lambda r: r["file"])
    for r in rows:
        print(f"  {'✅' if r['passed'] else '❌'} {r['file']:36s} {str(r.get('name'))[:70]}  ({r['seconds']} s)")
        if not r["passed"]:
            print(f"       期望: {r.get('expect','')}\n       实际: {str(r.get('got',''))[:400]}")
    ok = all(r["passed"] for r in rows)
    return ({"all_passed": ok and bool(rows), "n": len(rows), "rows": rows,
             "note": "" if rows else "negatives/ 里一个反例都没有 —— 检查器从未证明过自己会红"}, dict(sorted(manifest.items())))


# ── 记分卡校验（元规则 6：hash 对不上的记分卡作废）────────────────────────
def verify_scorecard(sc: dict) -> dict:
    """逐格核对 inputs_hash（以及 source_manifest）与磁盘；比对 git HEAD。
    返回 {ok, changed:[路径], stale_cells:[格子], missing:[路径], git_now, git_then, dirty}。"""
    changed, missing, stale = set(), set(), []
    files = dict(sc.get("source_manifest") or {})
    for cid, c in (sc.get("cells") or {}).items():
        ih = c.get("inputs_hash") or {}
        bad = False
        for p, h in ih.items():
            now = sha256_file(ROOT / p)
            if now is None:
                missing.add(p); bad = True
            elif now != h:
                changed.add(p); bad = True
        if bad:
            stale.append(cid)
    for p, h in files.items():
        now = sha256_file(ROOT / p)
        if now is None:
            missing.add(p)
        elif h is not None and now != h:
            changed.add(p)
    git_then, git_now = str(sc.get("git") or ""), git_head()
    same_git = git_then.split("+")[0] == git_now.split("+")[0]
    return {"ok": not changed and not missing and same_git,
            "changed": sorted(changed), "missing": sorted(missing), "stale_cells": sorted(stale),
            "git_then": git_then, "git_now": git_now,
            "dirty": git_then.endswith("+dirty") or git_now.endswith("+dirty")}


def _verify_cli(path: Path) -> int:
    if not path.exists():
        print(f"记分卡不存在：{path}"); return 3
    sc = json.loads(path.read_text(encoding="utf-8"))
    v = verify_scorecard(sc)
    print(f"记分卡 {path.name}：verdict={sc.get('verdict')} git={v['git_then']} → 当前 {v['git_now']}"
          + ("（子集跑）" if sc.get("partial") else ""))
    if v["ok"]:
        print(f"✅ 绑定的 {len(sc.get('source_manifest') or {})} 个输入与磁盘一致，git 同一 HEAD"
              + ("；但工作区 dirty，记分卡与提交无法一一对应" if v["dirty"] else ""))
        return 0
    print(f"❌ 记分卡作废：{len(v['changed'])} 个输入已变、{len(v['missing'])} 个缺失、{len(v['stale_cells'])} 个格子过期"
          + ("" if v["git_then"].split("+")[0] == v["git_now"].split("+")[0] else "；git HEAD 不同"))
    for p in v["changed"][:20]:
        print(f"   变了：{p}")
    for p in v["missing"][:10]:
        print(f"   缺失：{p}")
    print(f"   过期格子（前 12）：{v['stale_cells'][:12]}")
    return 3


# ── 按层并行（2026-09-18）：层与层之间不读彼此结果，只共享输入文件；每层在独立进程里各自 load_data + Ctx，
#    结果（LayerResult）pickle 回主进程后按原来的顺序合并。--jobs 1 = 旧的串行路径（同一个函数，逐个跑）。
#    反例包在自己的进程里串行跑（它在进程内改模块全局、写 _neg_tmp，不能拆开并行），与层并行。
def _check_cache():
    """hr42：检查原语缓存（tools/cad/check_cache.py；键 = 全部数值输入字节 + 库版本/后端，判据每次现判）。"""
    p = str(ROOT / "tools" / "cad")
    if p not in sys.path:
        sys.path.insert(0, p)
    import check_cache
    return check_cache


def _annotate_cells_check_cache(sc, outs, results, CC):
    """hr42（第四任）：把层进程送回的逐格归属写进记分卡格子。只加字段，不改 state / checks / inputs_hash。
    · 归属到原语的格：check_cache = {status, hits, misses, same_run, cached_from, key}；
      hits = 取自之前 run 的原语数，misses + same_run = 本轮现算（same_run = 本轮别处算过、复用）。
      status：cached（全部取自之前的 run）/ mixed（部分）/ fresh（全部本轮现算）。cached / mixed 另写
      cached_from（run 号 → 次数）、cache_key，并在 notes 末尾加一句（和"本次实算"区分开）。
    · 本层有 finding、本层用过缓存、但本格没归属到任何原语的格（层把原语集中算在前面 / 进程池里算）：status=unattributed，
      本层有取自之前 run 的命中时照写本层的 cached_from —— 不能证明本格全是本次现算，就不说它是。
    · 内核生成的格（义务 / 覆盖率 / 禁入体汇总 / 继承）不是检查计算，不标。
    返回各 status 的格数。"""
    per_cell = {}
    for o in outs:
        for cid, s in (o.get("cc_cells") or {}).items():
            if cid in per_cell:
                a = per_cell[cid]
                keys = sorted(k for k in (a.get("key"), s.get("key")) if k)
                h = hashlib.blake2b("".join(keys).encode(), digest_size=8).hexdigest() if keys else None
                cf = dict(a["cached_from"])
                for t, n in s["cached_from"].items():
                    cf[t] = cf.get(t, 0) + n
                per_cell[cid] = {"hits": a["hits"] + s["hits"], "misses": a["misses"] + s["misses"],
                                 "same_run": a.get("same_run", 0) + s.get("same_run", 0),
                                 "cached_from": dict(sorted(cf.items())), "key": h}
            else:
                per_cell[cid] = s
    layer_ns = {}
    for r in results:
        agg = {}
        for k, v in (r.evidence or {}).items():
            if k == "check_cache" or k.endswith(".check_cache"):
                agg = CC.merge_stats(agg, (v or {}).get("namespaces"))
        layer_ns[r.layer] = agg
    finding_cells = {f.cell for r in results for f in r.findings}
    count = {}
    for cid, c in sc["cells"].items():
        s = per_cell.get(cid)
        if s is None and cid not in finding_cells:
            continue                                       # 内核生成的格：不标
        lay = c.get("layer")
        ns = layer_ns.get(lay) or {}
        lay_hits = sum(v["hits"] for v in ns.values()); lay_miss = sum(v["misses"] for v in ns.values())
        if s is None:
            if not (lay_hits or lay_miss):
                continue                                   # 本层根本没用检查原语缓存：不标
            lay_from = {}
            for v in ns.values():
                for t, n in v["from_runs"].items():
                    lay_from[t] = lay_from.get(t, 0) + n
            c["check_cache"] = {"status": "unattributed", "layer_hits": lay_hits, "layer_misses": lay_miss,
                                "layer_cached_from": dict(sorted(lay_from.items()))}
            if lay_hits:
                c["cached_from"] = dict(sorted(lay_from.items()))
                c["notes"] = list(c.get("notes") or []) + [
                    f"检查原语缓存：本格没单独归属到几何原语；本层有 {lay_hits} 个原语取自缓存（cached_from {dict(sorted(lay_from.items()))}），"
                    f"不能证明本格全部本次现算"]
            st = "unattributed"
        else:
            fresh_n = s["misses"] + s.get("same_run", 0)
            st = "cached" if not fresh_n else ("fresh" if not s["hits"] else "mixed")
            c["check_cache"] = dict(status=st, **s)
            if st in ("cached", "mixed"):
                c["cached_from"] = s["cached_from"]
                c["cache_key"] = s["key"]
                c["notes"] = list(c.get("notes") or []) + [
                    (f"几何原语全部取自缓存（{s['hits']} 个）" if st == "cached" else
                     f"几何原语部分取自缓存（取自之前 run {s['hits']} / 本轮现算 {fresh_n}）")
                    + f"：cached_from={s['cached_from']} 键 {s['key']}；判据本次现判"]
        count[st] = count.get(st, 0) + 1
    return dict(sorted(count.items()))


def _run_layer_task(task):
    n, modname, only = task
    t0 = time.time()
    try:
        data = load_data()
        ctx = Ctx(data, only)
        m = importlib.import_module(f"layers.{modname}")
        name = getattr(m, "NAME", LAYER_NAMES.get(n, "?"))
        CC = _check_cache(); CC.reset_stats(); CC.set_context(f"L{n}:{modname}")   # 未命中日志的"在算谁"：层内没细分的就记层名
        # hr42（第四任）：逐格归属 —— 每条 finding 生成时，把"自上一条 finding 以来"查过的检查原语（命中 / 现算、来自哪次 run、键）
        #   归到这条 finding 的格子（L<层>/<对象>）。只包 LayerResult.add 这一个入口（unknown() 也走它），不改 core.py、不改层代码；
        #   判据、finding 内容与顺序不受影响。缓存关着（--no-check-cache）时不包、不归属。
        cc_cells, cc_tail, att = {}, None, CC.enabled()
        if att:
            CC.attrib_begin()
            _orig_add = LayerResult.add

            def _add(self, **kw):
                f = _orig_add(self, **kw)
                b = CC.attrib_take()
                if b and (b["hits"] or b["misses"] or b["same_run"]):
                    cc_cells[f.cell] = CC.merge_bucket(cc_cells.get(f.cell), b)
                return f
            LayerResult.add = _add
        try:
            r = m.run(ctx)
        finally:
            if att:
                LayerResult.add = _orig_add
                cc_tail = CC.attrib_take(); CC.attrib_end()
            CC.flush()                              # 层进程由进程池 os._exit 收掉，atexit 不跑 → 这里就写分片
        # 本层用了多少缓存的几何原语、来自哪次 run（cached_from）→ 记进本层 evidence，和本次现算的区分开
        r.evidence = dict(r.evidence or {}); r.evidence["check_cache"] = CC.evidence()
        if att:
            r.evidence["check_cache"]["unattributed_tail"] = CC.bucket_summary(cc_tail)   # 最后一条 finding 之后才查的原语
        # 层把工作对象挂在结果上（L4 的 res._asm/res._names 含 manifold3d 实体，pickle 不了）；
        # 只把 LayerResult 的正式字段送回主进程，私有属性丢掉（主进程只读 findings/inputs/evidence/error/covered）。
        import dataclasses, pickle
        for k in [k for k in vars(r) if k not in {f.name for f in dataclasses.fields(r)}]:
            delattr(r, k)
        try:
            pickle.dumps(r)
        except Exception as e:                     # 仍有不可序列化的证据值 → 转成字符串，判据本身不丢
            for f in r.findings:
                for fld in ("measured", "detail", "criterion"):
                    try:
                        pickle.dumps(getattr(f, fld))
                    except Exception:
                        setattr(f, fld, repr(getattr(f, fld)))
            r.evidence = {k: (v if _picklable(v) else repr(v)) for k, v in (r.evidence or {}).items()}
            r.error = (r.error or "") + f"\n[gate --jobs] 结果里有不可序列化对象已转 repr：{e}"
        return dict(n=n, mod=modname, name=name, result=r, error=None, seconds=round(time.time() - t0, 1),
                    cc_cells={cid: CC.bucket_summary(b) for cid, b in cc_cells.items()})
    except Exception:
        return dict(n=n, mod=modname, name=modname, result=None, error=traceback.format_exc(limit=6),
                    seconds=round(time.time() - t0, 1))


# hr42：层模块的相对耗时（hr42b 基线 gate.log 实测秒数），只用来排提交顺序；判据无关，缺了按 0。
_LAYER_COST = {"l6_motion": 908.0, "l1_printable": 495.0, "l4_assembly": 86.2, "l2_features": 78.3, "l3_static": 14.9,
               "l5_screwhead": 13.5, "l7_mass": 11.4, "l0_mesh": 9.3, "l5_function": 8.6}


def _picklable(v):
    import pickle
    try:
        pickle.dumps(v); return True
    except Exception:
        return False


def _run_negatives_task(use_cache):
    # hr42：反例包一律不用检查原语缓存、也不开 L6 并行预取 —— 反例会打桩/改几何来证明检查器会红，
    #       缓存或另起的进程（看不到本进程里的打桩）都不能替它们挡掉被测路径
    import os
    saved = {k: os.environ.get(k) for k in ("DUCK_CHECK_CACHE", "L6_JOBS", "L1_JOBS")}
    os.environ["DUCK_CHECK_CACHE"] = "0"
    os.environ["L6_JOBS"] = "1"
    os.environ["L1_JOBS"] = "1"
    try:
        return run_negatives(use_cache=use_cache, jobs=_neg_split())
    finally:
        for k, v in saved.items():
            if v is None:
                os.environ.pop(k, None)
            else:
                os.environ[k] = v


def _default_jobs():
    import os
    v = os.environ.get("GATE_JOBS")
    if v and v.isdigit():
        return max(1, int(v))
    return max(1, min(4, (os.cpu_count() or 2) // 3))     # 内存优先：每个层进程要载全部网格，4 个并发在 18 GB 机器上是上限


# ── gate_guard（2026-09-25）：层崩溃 / 层空跑守门 ─────────────────────────────────
# 事故（校验会话 09-25 18:55）：fasteners.yaml 一条 joins 含半角逗号没加引号，YAML 把它拆成 [str, int] → l5_function 里
#   " ".join 抛 TypeError，整个模块没交回任何判据；其它层照跑，终端只一行"0 条判据，0 条红 … [层异常]"，记分卡里只多一个
#   淹在几百个红格里的 L5/_layer —— 很容易被当成"L5 全过"。守门（只在触发时加格 / 加行；正常运行不进任何新分支）：
#   a. 层崩溃 = 层模块没交回结果：抛异常（_run_layer_task 已兜住）/ 层进程被杀、进程池断（fu.result() 抛 BrokenProcessPool）/
#      返回不可解析 / import 失败（discover 的 broken）→ L<n>/_layer_error（FAIL，BLOCK，check=layer_crashed），任何跑法都加；
#   b. 层空跑 = 层模块跑完但 0 条判据：全量跑（没有 --layers/--parts）→ 同样加 L<n>/_layer_error（check=layer_zero_findings）；
#      子集跑只打印提示行、不加格（子集里某层没有对象是正常的）。按**模块**判：L5 有两个模块，只看层合计会被另一个模块盖住。
#   终端摘要（结论行之前）与记分卡 md（标题下）顶部各一行 ⚠；结论按 BLOCK（blocking_cells + verdict），退出码 1。
GUARD_SUBJECT = "_layer_error"
GUARD_PROV = "tools/gate/gate.py:_layer_guard（gate_guard 2026-09-25）"
_GUARD_HOW = {"exception": "层模块抛异常", "import": "层模块 import 失败",
              "pool_broken": "层进程被杀 / 进程池断（段错误、内存超限、看门狗 kill -9 都是这样；断池时还没收回的模块全部算上）",
              "no_result": "层进程没交回结果", "unparseable": "层进程返回不可解析"}
_POOL_BROKEN = False          # 本轮进程池断过 → __main__ 用 os._exit 收尾（见 _exit_after_main）


def _exc_head(text) -> str:
    """traceback 文本 → 最后一条顶格的异常行（"TypeError: …"），跳过 Traceback / During handling / direct cause 提示行；
    没有顶格行就取最后一个非空行。"""
    lines = [ln.rstrip() for ln in str(text or "").splitlines() if ln.strip()]
    for ln in reversed(lines):
        if ln[:1].strip() and not ln.startswith(("Traceback (most recent call last)", "During handling of the above exception",
                                                  "The above exception was the direct cause")):
            return ln.strip()
    return lines[-1].strip() if lines else "（没有错误文本）"


def _crashed_out(tk, e):
    """拿不回层结果（层进程被杀 → BrokenProcessPool、结果反序列化失败、层里 sys.exit 漏出来 …）→ 按层崩溃记，不整轮炸掉。"""
    global _POOL_BROKEN
    from concurrent.futures.process import BrokenProcessPool
    how = "pool_broken" if isinstance(e, BrokenProcessPool) else "no_result"
    if how == "pool_broken":
        _POOL_BROKEN = True
    return dict(n=tk[0], mod=tk[1], name=tk[1], result=None, seconds="?", guard_how=how,
                guard_head=f"{type(e).__name__}: {e}",
                error="".join(traceback.format_exception(type(e), e, e.__traceback__, limit=6)))


def _checked_out(tk, o):
    """层结果形状检查：必须是 _run_layer_task 交回的 dict —— result 有 findings 列表（每条有 state / cell），或 result=None 且带 error。
    合格 → 原样返回同一个对象（正常路径什么都不改）；不合格 → 按"返回不可解析"崩溃记。"""
    try:
        r = o.get("result") if isinstance(o, dict) else None
        ok = (isinstance(o, dict) and o.get("n") == tk[0] and o.get("mod") == tk[1]
              and ((r is None and bool(o.get("error")))
                   or (r is not None and isinstance(getattr(r, "findings", None), list)
                       and all(hasattr(f, "state") and hasattr(f, "cell") for f in r.findings))))
    except Exception:
        ok = False
    if ok:
        return o
    try:
        shown = repr(o)[:300]
    except Exception as e:
        shown = f"（repr 也失败：{type(e).__name__}）"
    return dict(n=tk[0], mod=tk[1], name=tk[1], result=None, seconds="?", guard_how="unparseable",
                guard_head=f"返回不可解析：{type(o).__name__}",
                error=f"层进程返回不可解析（不是带 findings 的 LayerResult / 带 error 的 dict）：{type(o).__name__} {shown}")


def _neg_crashed(e):
    """反例包进程崩溃 / 被杀 → 反例包结论 = 失败（L0/_negatives 按原逻辑判红），横幅另写一行。"""
    global _POOL_BROKEN
    from concurrent.futures.process import BrokenProcessPool
    if isinstance(e, BrokenProcessPool):
        _POOL_BROKEN = True
    head = f"{type(e).__name__}: {e}"
    return {"all_passed": False, "n": 0, "rows": [], "cached": False, "seconds": "?", "guard_head": head,
            "note": f"反例包进程崩溃：{head}", "failed": [f"<反例包进程崩溃：{head[:200]}>"]}


def _guard_live(o, partial):
    """层结果一收回就多打一行（只在崩溃 / 空跑时；正常结果不打印）。"""
    r = o.get("result")
    if r is None:
        print(f"   ⚠ L{o['n']} {o['mod']} 崩溃：{o.get('guard_head') or _exc_head(o.get('error'))}"
              f" —— 这 0 条判据不是通过，判据全部未评", flush=True)
    elif not r.findings:
        print(f"   ⚠ L{o['n']} {o['mod']} 空跑：0 条判据"
              + ("（子集跑：范围内可能确实没有对象）" if partial else " —— 疑似崩溃或数据为空，不是通过"), flush=True)


def _set_negatives_cell(sc, cell):
    """写入 / 覆盖 L0/_negatives 格并同步 tally / blocking_cells / verdict（hr50 run2 2026-09-28 记账 bug：这格是 build_scorecard
    之后才补的，全过分支只写格子没给 tally[PASS] +1，记分卡写 ✅169 而 628 格实际 170；失败分支有 +1 所以 run1 没暴露）。
    子集跑可能从 prev 继承过同名格：先把旧状态从计数里拿掉再记本轮的（与 _layer_guard 同法）；verdict 按与 build_scorecard 同一公式重算。"""
    from core import NOT_RUN, STALE
    cid = "L0/_negatives"
    old = sc["cells"].get(cid)
    if old is not None and old.get("state") is not None:
        sc["tally"][old["state"]] = sc["tally"].get(old["state"], 0) - 1
    sc["cells"][cid] = cell
    sc["tally"][cell["state"]] = sc["tally"].get(cell["state"], 0) + 1
    blocks = set(sc.get("blocking_cells") or [])
    (blocks.add if cell["state"] == FAIL else blocks.discard)(cid)
    sc["blocking_cells"] = sorted(blocks)
    sc["verdict"] = "BLOCKED" if blocks else ("INCOMPLETE" if (sc["tally"].get(NOT_RUN) or sc["tally"].get(STALE)) else "CLEAR")


def _layer_guard(outs, broken, results, partial, sc):
    """崩溃（任何跑法）/ 空跑（只全量跑）→ L<n>/_layer_error 写进 sc（同步 tally / blocking_cells / verdict），
    返回 (报错行, 提示行)。没有触发 → ([], [])，sc 一个字节都不动。"""
    recs = []
    for n in sorted(broken):
        for txt in broken[n]:
            recs.append(dict(layer=n, mod=str(txt).split(":", 1)[0].removesuffix(".py"), kind="crash", how="import",
                             text=str(txt), head=None))
    for o in outs:
        r = o.get("result")
        if r is None:
            recs.append(dict(layer=o["n"], mod=o["mod"], kind="crash", how=o.get("guard_how", "exception"),
                             text=str(o.get("error") or ""), head=o.get("guard_head")))
        elif not r.findings:
            recs.append(dict(layer=o["n"], mod=o["mod"], kind="empty", how="zero_findings", text=str(r.error or ""), head=None))
    if not recs:
        return [], []
    tot = {}
    for r in results:
        tot[r.layer] = tot.get(r.layer, 0) + len(r.findings)
    errors, notes, cells, summary = [], [], {}, []
    for x in recs:
        n, mod = x["layer"], x["mod"]
        lname, cid, others = LAYER_NAMES.get(n, "?"), f"L{n}/{GUARD_SUBJECT}", tot.get(n, 0)
        if x["kind"] == "empty" and partial:
            notes.append(f"⚠ 提示：第 {n} 层 {lname}（{mod}）本轮 0 条判据 —— 子集跑，范围内可能确实没有对象，不加格"
                         f"（全量跑出现就是 BLOCK）")
            continue
        if x["kind"] == "crash":
            head = x["head"] or _exc_head(x["text"])
            tail = "\n".join([ln.rstrip() for ln in x["text"].splitlines() if ln.strip()][-5:])
            chk = {"check": "layer_crashed", "state": FAIL, "severity": "BLOCK", "measured": head.split(":", 1)[0].strip()[:80],
                   "criterion": "层模块必须跑完并交回可解析的结果；崩溃 / 被杀 / 返回不可解析 / import 失败 = 本层判据全部未评，不是通过",
                   "evidence_n": 0,
                   "detail": (f"{_GUARD_HOW.get(x['how'], x['how'])}（layers.{mod}）：{head}\ntraceback 末 5 行：\n{tail}\n本层判据全部未评"
                              + ("" if not others else f"（指 {mod} 这一模块的判据；本层其余模块照常发出 {others} 条）")),
                   "provenance": GUARD_PROV}
            errors.append(f"⚠ 第 {n} 层 {lname} 崩溃（{mod}）：{head} —— "
                          + ("本层 0 条判据不是通过" if not others else f"{mod} 0 条判据不是通过（本层其余模块 {others} 条照常）")
                          + f"；格 {cid} BLOCK")
        else:
            chk = {"check": "layer_zero_findings", "state": FAIL, "severity": "BLOCK", "measured": 0,
                   "criterion": "全量跑时每个层模块至少发出 1 条判据；0 条 = 疑似崩溃或数据为空，不是通过",
                   "evidence_n": 0,
                   "detail": (f"layers.{mod} 跑完但 0 条判据：疑似崩溃或数据为空（全量跑）"
                              + (f"；层自报 error：{_exc_head(x['text'])}" if x["text"].strip() else "")),
                   "provenance": GUARD_PROV}
            errors.append(f"⚠ 第 {n} 层 {lname} 空跑（{mod}）：0 条判据 —— 疑似崩溃或数据为空，不是通过；格 {cid} BLOCK")
        c = cells.get(cid)
        if c is None:
            c = cells[cid] = {"layer": n, "subject": GUARD_SUBJECT, "state": FAIL, "evidence_n": 0, "checks": [],
                              "inputs_hash": dict((sc["cells"].get(f"L{n}/_layer") or {}).get("inputs_hash") or {}),
                              "state_computed": FAIL}
        c["checks"].append(chk)
        summary.append({"layer": n, "module": mod, "kind": x["kind"], "how": x["how"], "check": chk["check"],
                        "head": str(chk["detail"]).split("\n", 1)[0][:300]})
    for cid, c in cells.items():
        old = sc["cells"].get(cid)                   # 子集跑可能从 prev 继承过同名格：先把它从计数里拿掉再换成本轮的
        if old is not None:
            sc["tally"][old["state"]] = sc["tally"].get(old["state"], 0) - 1
        sc["cells"][cid] = c
        sc["tally"][FAIL] = sc["tally"].get(FAIL, 0) + 1
    if cells:
        sc["blocking_cells"] = sorted(set(sc["blocking_cells"]) | set(cells))
        sc["verdict"] = "BLOCKED"
        sc["layer_errors"] = summary
    return errors, notes


def main(argv=None):
    global _POOL_BROKEN
    _POOL_BROKEN = False
    ap = argparse.ArgumentParser()
    ap.add_argument("--layers", default="", help="逗号分隔，默认全部已实现的层（给了就是子集跑）")
    ap.add_argument("--parts", default="", help="逗号分隔件号，默认全部（给了就是子集跑）")
    ap.add_argument("--out", default=str(HERE / "out"))
    ap.add_argument("--skip-negatives", action="store_true",
                    help="跳过反例回归（只在调试单层时用；跳过后结论不得当作放行依据）")
    ap.add_argument("--no-neg-cache", action="store_true", help="强制重跑反例包，不用缓存")
    ap.add_argument("--jobs", type=int, default=None,
                    help="并行进程数：层按进程并行、反例包在另一个进程里串行同时跑（默认 GATE_JOBS 或 min(4, cpu/3)；1 = 串行）")
    ap.add_argument("--verify", nargs="?", const="", metavar="SCORECARD",
                    help="不跑层，只核对记分卡绑定的输入 hash（默认 out/scorecard.json）")
    ap.add_argument("--no-check-cache", action="store_true",
                    help="hr42：不读不写检查原语缓存，全部几何原语现算（交付/复审前的全链必须这样跑）")
    a = ap.parse_args(argv)
    if a.no_check_cache:
        import os
        os.environ["DUCK_CHECK_CACHE"] = "0"          # 进程池 spawn 的层进程继承环境
    _check_cache().run_tag()                          # hr42：先定 run 号（写进环境变量），之后起的层进程 / 反例进程继承同一个号

    if a.verify is not None:
        return _verify_cli(Path(a.verify) if a.verify else Path(a.out) / "scorecard.json")

    t0 = time.time()
    data = load_data()
    jobs = a.jobs if a.jobs else _default_jobs()

    only = [x for x in a.parts.split(",") if x]
    want = [int(x) for x in a.layers.split(",") if x.strip()] or None
    partial = bool(only) or bool(want)
    mods, broken = discover(want)
    want = want or sorted(set(mods) | set(broken)) or list(range(8))
    tasks = [(n, m.__name__.split(".")[-1], only) for n in want for m in (mods.get(n) or [])]

    # ── 元规则 8：反例必须跑（一个从来没红过的检查器等于没有）。串行模式先跑反例再跑层（旧行为）；
    #    并行模式反例在独立进程里与各层同时跑，结论仍写进同一张记分卡。
    neg = {"all_passed": None, "n": 0, "rows": [], "note": "--skip-negatives", "cached": False}
    outs = []
    if jobs <= 1 or len(tasks) <= 1:
        if not a.skip_negatives:
            print("── 反例回归包（元规则 8：先证明自己会红）", flush=True)
            try:
                neg = _run_negatives_task(not a.no_neg_cache)
            except (Exception, SystemExit) as e:           # gate_guard：反例包自己崩了 → 结论 = 失败，不整轮炸掉
                neg = _neg_crashed(e)
            print(f"   {neg['n']} 个反例，{'全过' if neg['all_passed'] else '有失败 → 本轮结论不可信'}"
                  + ("（缓存）" if neg.get("cached") else f"（{neg.get('seconds', '?')} s）") + "\n", flush=True)
        for tk in tasks:
            print(f"── L{tk[0]} {tk[1]} …", flush=True)
            try:
                o = _run_layer_task(tk)
            except (Exception, SystemExit) as e:           # gate_guard：漏出 _run_layer_task 的（层里 sys.exit 之类）→ 按崩溃记
                o = _crashed_out(tk, e)
            outs.append(_checked_out(tk, o))
            o = outs[-1]; nb = sum(1 for f in (o["result"].findings if o["result"] else []) if f.state == FAIL)
            print(f"   {o['name']}：{len(o['result'].findings) if o['result'] else 0} 条判据，{nb} 条红，{o['seconds']} s"
                  + (" [层异常]" if o["error"] else ""), flush=True)
            _guard_live(o, partial)
    else:
        import multiprocessing as mp
        from concurrent.futures import ProcessPoolExecutor
        print(f"── 并行：{len(tasks)} 个层模块 + 反例包，{jobs} 进程（--jobs / GATE_JOBS）", flush=True)
        with ProcessPoolExecutor(max_workers=jobs, mp_context=mp.get_context("spawn")) as ex:
            # hr42：长的先提交（L6/L1 各自还会开自己的并行预取），反例包放在最长的两层之后；结果仍按原 tasks 顺序收、合并
            order = sorted(range(len(tasks)), key=lambda i: -_LAYER_COST.get(tasks[i][1], 0.0))
            futs_by = {}
            fneg = None
            for rank, i in enumerate(order):
                if rank == 2 and not a.skip_negatives:
                    fneg = ex.submit(_run_negatives_task, not a.no_neg_cache)
                futs_by[i] = ex.submit(_run_layer_task, tasks[i])
            if fneg is None and not a.skip_negatives:
                fneg = ex.submit(_run_negatives_task, not a.no_neg_cache)
            futs = [futs_by[i] for i in range(len(tasks))]
            for tk, fu in zip(tasks, futs):
                try:
                    o = fu.result()
                except (Exception, SystemExit) as e:       # gate_guard：层进程被杀 / 进程池断 / 结果回不来 → 按崩溃记，不整轮炸掉
                    o = _crashed_out(tk, e)
                o = _checked_out(tk, o); outs.append(o)
                nb = sum(1 for f in (o["result"].findings if o["result"] else []) if f.state == FAIL)
                print(f"── L{o['n']} {o['name']}：{len(o['result'].findings) if o['result'] else 0} 条判据，{nb} 条红，{o['seconds']} s"
                      + (" [层异常]" if o["error"] else ""), flush=True)
                _guard_live(o, partial)
            if fneg is not None:
                try:
                    neg = fneg.result()
                except (Exception, SystemExit) as e:       # gate_guard：反例包进程被杀 / 崩了 → 结论 = 失败
                    neg = _neg_crashed(e)
                print(f"── 反例回归包：{neg['n']} 个反例，{'全过' if neg['all_passed'] else '有失败 → 本轮结论不可信'}"
                      + ("（缓存）" if neg.get("cached") else f"（{neg.get('seconds', '?')} s）"), flush=True)
                for row in neg.get("rows", []):
                    if not row.get("passed"):
                        print(f"   ❌ {row.get('file','?')}  期望: {str(row.get('expect',''))[:160]}\n      实际: {str(row.get('got',''))[:300]}", flush=True)

    results = []
    by_layer = {}
    for o in outs:
        by_layer.setdefault(o["n"], []).append(o)
    for n in want:
        if n in broken:
            r = LayerResult(n, LAYER_NAMES.get(n, "?")); r.error = "\n".join(broken[n])
            results.append(r); print(f"── L{n} import 失败", flush=True)
        olist = by_layer.get(n) or []
        if not olist:
            if n not in broken:
                r = LayerResult(n, LAYER_NAMES.get(n, "?"))
                r.unknown("_layer", "implemented", f"第 {n} 层（{LAYER_NAMES.get(n)}）还没实现",
                          provenance="tools/gate/README.md 第 2 节")
                results.append(r)
            continue
        merged = LayerResult(n, LAYER_NAMES.get(n, "?"))
        for o in olist:
            r = o["result"]
            if r is not None:
                merged.findings += r.findings
                merged.inputs += r.inputs
                merged.covered |= (r.covered or set())   # 覆盖率集合也要并，否则 _coverage 恒为 0
                for k, v in (r.evidence or {}).items():
                    merged.evidence[f"{o['name']}.{k}"] = v
                if r.error:
                    merged.error = (merged.error or "") + f"\n[layers.{o['mod']}] {r.error}"
            if o["error"]:
                merged.error = (merged.error or "") + f"\n[layers.{o['mod']}]\n" + o["error"]
        bad = sum(1 for f in merged.findings if f.state == FAIL)
        print(f"   L{n} 累计 {len(merged.findings)} 条判据，{bad} 条红" + (" [层异常]" if merged.error else ""), flush=True)
        results.append(merged)

    out = Path(a.out); out.mkdir(parents=True, exist_ok=True)
    prev_p = out / "scorecard.prev.json"
    prev = json.loads(prev_p.read_text(encoding="utf-8")) if prev_p.exists() else None

    manifest = source_manifest(sorted({p for r in results for p in r.inputs}))
    notes = apply_waivers([f for r in results for f in r.findings], load_waivers(data), manifest)
    sc = build_scorecard(results, manifest, notes, t0, prev=prev, layers_run=want, data=data,
                         only=set(only) if only else None)

    # 反例格**无论过不过都写**（审计 F-核-7：以前只在失败时建格且没有 inputs_hash，之后的子集跑会把
    # 旧红按"输入未变"继承，反例修好也留红）。inputs = 缓存 key 覆盖的那批文件。
    neg_inputs = {}
    for p in negatives_key_files(data=data):
        try:
            neg_inputs[str(p.resolve().relative_to(ROOT))] = sha256_file(p)
        except ValueError:
            neg_inputs[str(p)] = sha256_file(p)
    failed = [r["file"] for r in neg.get("rows", []) if not r.get("passed")] or list(neg.get("failed") or [])
    if neg["all_passed"] is True:
        chk = {"check": "negatives_regression", "state": PASS, "severity": "BLOCK",
               "measured": neg["n"], "criterion": "反例回归包必须全部通过（元规则 8）",
               "evidence_n": neg["n"],
               "detail": ("缓存命中：" if neg.get("cached") else "本轮实跑：") + f"{neg['n']} 个反例全过"
                         + (f"，{neg.get('seconds')} s" if not neg.get("cached") else ""),
               "provenance": "tools/gate/negatives/runner.py"}
        if neg.get("cached"):
            chk["cached"] = True
            chk["cached_when"] = neg.get("cached_when")
        _set_negatives_cell(sc, {
            "layer": 0, "subject": "_negatives", "state": PASS, "evidence_n": neg["n"],
            "checks": [chk],
            "inputs_hash": neg_inputs, "state_computed": PASS})
    else:
        why = ("反例回归被跳过（--skip-negatives）—— 未证明检查器会红"
               if neg["all_passed"] is None else f"反例回归有 {len(failed)} 个失败：{failed}")
        _set_negatives_cell(sc, {
            "layer": 0, "subject": "_negatives", "state": "FAIL", "evidence_n": neg["n"],
            "checks": [{"check": "negatives_regression", "state": "FAIL", "severity": "BLOCK",
                        "measured": neg["n"], "criterion": "反例回归包必须全部通过（元规则 8）",
                        "evidence_n": neg["n"], "detail": why,
                        "provenance": "tools/gate/negatives/runner.py"}],
            "inputs_hash": neg_inputs, "state_computed": "FAIL"})
    sc["negatives"] = {k: v for k, v in neg.items() if k != "rows"} | {"failed": failed}
    # gate_guard：层崩溃 / 层空跑 → L<n>/_layer_error + 顶部报错行（没触发 = 两个空列表，sc 不动）
    guard_err, guard_note = _layer_guard(outs, broken, results, partial, sc)
    if neg.get("guard_head"):
        guard_err.insert(0, f"⚠ 反例包进程崩溃：{neg['guard_head']} —— 反例回归不是全过；格 L0/_negatives BLOCK")
    if _POOL_BROKEN:
        guard_err.append("⚠ 进程池断过（层进程被杀 / 段错误）：反例包与 L1·L6 预取池的孙进程可能成了孤儿"
                         "（PPID 1、命令行含 spawn_main），跑完用 ps 查一下手工清")
    # hr42：全卡汇总检查原语缓存（各层 evidence 里有逐层明细）；关着缓存时写 enabled=false
    CC = _check_cache()
    cell_status = _annotate_cells_check_cache(sc, outs, results, CC) if CC.enabled() else {}
    agg = {}
    for r in results:
        for k, v in (r.evidence or {}).items():             # 合并后的层 evidence 键是 "<层模块名>.check_cache"
            if k == "check_cache" or k.endswith(".check_cache"):
                agg = CC.merge_stats(agg, (v or {}).get("namespaces"))
    sc["check_cache"] = dict(enabled=CC.enabled(), run=CC.run_tag() if CC.enabled() else None, namespaces=agg,
                             hits=sum(v["hits"] for v in agg.values()), misses=sum(v["misses"] for v in agg.values()),
                             same_run=sum(v.get("same_run", 0) for v in agg.values()),
                             from_runs=CC.merge_stats({}, {ns: dict(v, hits=0, misses=0) for ns, v in agg.items()}) and
                                       {t: sum(v["from_runs"].get(t, 0) for v in agg.values()) for t in sorted({t for v in agg.values() for t in v["from_runs"]})},
                             cells_by_status=cell_status,
                             note="命中 = 该几何原语（布尔体积/min_gap/射线/contains/L1 截面等）的全部数值输入与上次某 run 逐字节相同、直接取当时结果；"
                                  "判据/阈值/豁免/覆盖率每次现判。逐格：cells.<格>.check_cache（status=cached 全部取自缓存 / mixed 部分 / fresh 全部现算 / "
                                  "unattributed 本格没单独归属到原语），取自缓存的格另有 cached_from（run 标签 → 次数）+ cache_key（本格用到的全部原语键的哈希）。"
                                  "--no-check-cache 全量实跑。")

    # 子集跑 → partial 文件，不动 prev / impact（那两个只属于全量跑，元规则 7）
    stem = "scorecard.partial" if partial else "scorecard"
    (out / f"{stem}.json").write_text(json.dumps(sc, ensure_ascii=False, indent=1), encoding="utf-8")
    md = render_markdown(sc, data)
    cc = sc.get("check_cache") or {}
    if cc:                                           # hr42：记分卡第二行就写清几何原语有多少取自缓存、来自哪次 run
        line = (f"> 检查原语缓存：{'开' if cc.get('enabled') else '关（--no-check-cache，全量实跑）'}；命中 {cc.get('hits', 0)} / 现算 {cc.get('misses', 0)}"
                + (f"；命中的来自 run {cc.get('from_runs')}" if cc.get("hits") else "")
                + (f"；格子：{cc.get('cells_by_status')}" if cc.get("cells_by_status") else "")
                + "（逐层明细见 evidence.*.check_cache，逐格见 cells.*.check_cache / cached_from）\n")
        head, _, rest = md.partition("\n")
        md = head + "\n\n" + line + rest
    if guard_err or guard_note:                      # gate_guard：记分卡 md 标题下第一段就是报错（正常运行不加）
        head, _, rest = md.partition("\n")
        md = (head + "\n\n" + "\n\n".join([f"> **{ln.replace('<', '&lt;')}**" for ln in guard_err]
                                            + [f"> {ln.replace('<', '&lt;')}" for ln in guard_note]) + "\n" + rest)
    (out / f"{stem}.md").write_text(md, encoding="utf-8")

    if not partial:
        if prev:
            old = prev
            changed = [c for c in sc["cells"]
                       if old.get("cells", {}).get(c, {}).get("state") != sc["cells"][c]["state"]]
            lines = ["# 变更影响", "", f"上次 `{old.get('git','?')}` → 本次 `{sc['git']}`", ""]
            lines += [f"- `{c}`: {old.get('cells',{}).get(c,{}).get('state','(新)')} → {sc['cells'][c]['state']}"
                      for c in sorted(changed)] or ["(没有格子状态变化)"]
            (out / "impact.md").write_text("\n".join(lines) + "\n", encoding="utf-8")
        prev_p.write_text(json.dumps(sc, ensure_ascii=False, indent=1), encoding="utf-8")

    t = sc["tally"]
    if guard_err:                                    # gate_guard：终端摘要顶部醒目报错（正常运行不打印）
        print("\n" + "!" * 88)
        for ln in guard_err:
            print(ln)
        print("!" * 88, flush=True)
    for ln in guard_note:
        print(ln)
    print(f"\n{sc['verdict']}{'（子集跑）' if partial else ''}  ✅{t[PASS]} ❌{t[FAIL]} 🕓{t[STALE]} ⬜{t[NOT_RUN]} 🟡{t[WAIVED]} 🪦{t.get(RETIRED, 0)}")
    print(f"记分卡 → {out / (stem + '.md')}")
    return 1 if sc["blocking_cells"] else (2 if t[NOT_RUN] or t[STALE] else 0)


def _exit_after_main(rc):
    """gate_guard：本轮进程池断过（层进程被杀 / 段错误）时，反例包与 L1/L6 预取池的孙进程会成孤儿，一直握着 multiprocessing
    资源追踪器的管道 → 解释器收尾时 ResourceTracker.__del__ 的 waitpid 永远等不到（09-25 00:10 e_gate：主线程挂在
    Py_FinalizeEx → os_waitpid，见 hr42_work/dev/e_gate_crash/e_gate_hang_sample.txt；那次的孤儿 spawn_main(tracker_fd=…) 还活着）。
    这时记分卡和报错都已写出 → 冲掉缓冲直接 os._exit(rc)，退出码照给。没断过池 = 原来的 sys.exit(rc)。"""
    if _POOL_BROKEN:
        import os
        for f in (sys.stdout, sys.stderr):
            try:
                f.flush()
            except Exception:
                pass
        os._exit(rc if isinstance(rc, int) else 1)
    sys.exit(rc)


if __name__ == "__main__":
    _exit_after_main(main())
