"""hr42（2026-09-24）：检查原语缓存 —— 按输入内容哈希，只缓存纯几何计算，不缓存判据。

原语 = 一次布尔求交（体积 + 交集网格）/ 一次 min_gap / 一次射线查询 / 一次 contains / L1 的几个重函数。
键 = blake2b-128( 键版本 ‖ 后端与库版本 ‖ 原语种类 ‖ 全部数值输入的原始字节（网格顶点/面数组、变换矩阵、射线起点/方向、点、参数） )。
值 = 该原语的精确结果（float 或 numpy 数组，原样 pickle 存取，逐字节还原）。
所以：没动的件 → 它参与的原语输入字节不变 → 命中；动了的件 → 它参与的每个原语输入字节都变 → 必然未命中、现算。
阈值、豁免、结论、覆盖率每次都由检查代码现判；缓存只替掉"同样的输入再算一遍"的几何计算，不缩小任何依赖集。

开关：环境变量 DUCK_CHECK_CACHE=0（gate.py / audit_motion.py / policy_pose_collisions.py / slice_l1.py 的 --no-check-cache 会设它；
      build 用 DUCK_CHECK_CACHE=0 或 go_chain.sh nocache）→ 不读不写，全量实跑（交付 / 复审前的全链按项目规矩必须这样跑）。
      DUCK_CHECK_CACHE_TAG=<标签>：本次 run 的人读标签；条目记的来源是 run 号 = DUCK_CHECK_CACHE_RUN_ID（没有就在第一个 import 本模块的进程里
      生成 "<标签>@<年月日-时分秒>-<pid>" 并写进环境变量，之后起的子进程继承 → 同一 run 的各进程同一个号；go_chain.sh 给整条链一个号）。
      命中时报 cached_from = 条目的 run 号；**同一 run 号的条目 = 本轮自己算过又复用（same_run），按"本轮现算"计，不算缓存命中**。
存储：<仓库根>/tools/gate/out/check_cache/<命名空间>.pkl（主文件）+ <命名空间>.<pid>.<序号>.shard.pkl（各进程增量，原子改名写入）；
      进程首次用到某命名空间时读主文件 + 全部分片；主进程退出前 compact() 合并（fcntl 锁）。
统计：stats() → {命名空间: {hits, misses, from_runs{标签: 次数}}}，由调用方写进 evidence（gate 层 / motion.json / summary.json 旁的 check_cache.json）。
逐格归属（第四任加）：attrib_begin() 之后每次查缓存都记进"当前归属桶"（命中数 / 现算数 / 来自哪些 run / 用到的键），
      attrib_take() 取走当前桶并开新桶 —— gate.py 在每条 finding 生成时取一次，归到该 finding 的格子（L<层>/<对象>）；
      值来自别处（进程池预取）时用 note() 按原出处补记，保证"取自缓存"的原语不会被记成本次现算。
射线后端（第四任加，与 B 线约定）：tools/cad/ray_backend.py 能 import 就用它的 intersects_location / contains，
      否则照旧直调 trimesh；后端名（ray_backend.name() 或 trimesh 射线类名）进键与 versions()。
"""
from __future__ import annotations
import atexit, fcntl, hashlib, os, pickle, sys, time
from pathlib import Path

import numpy as np

SCHEMA = "hr42-prim-v1"
ROOT = Path(__file__).resolve().parents[2]
DIR = ROOT / "tools" / "gate" / "out" / "check_cache"

_ns = {}            # 命名空间 → dict(key → (value, tag))
_new = {}           # 命名空间 → dict(key → (value, tag))：本进程新算的，flush 写分片
_stats = {}         # 命名空间 → {"hits", "misses", "from_runs"}
_shard_seq = [0]
_versions = [None]


def enabled() -> bool:
    return os.environ.get("DUCK_CHECK_CACHE", "1") not in ("0", "off", "no", "false")


_TAG = [None]


def run_tag() -> str:
    """本次 run 号（条目的来源标记；见文件头）。"""
    if _TAG[0] is None:
        rid = os.environ.get("DUCK_CHECK_CACHE_RUN_ID")
        if not rid:
            rid = f"{os.environ.get('DUCK_CHECK_CACHE_TAG') or 'run'}@{time.strftime('%Y%m%d-%H%M%S')}-{os.getpid()}"
            os.environ["DUCK_CHECK_CACHE_RUN_ID"] = rid          # 之后 spawn / subprocess 起的子进程继承 → 同一个 run 号
        _TAG[0] = rid
    return _TAG[0]


def is_prev(tag) -> bool:
    """命中的条目是不是来自之前的 run（True = 真·缓存命中；False = 本轮自己算过的）。"""
    return tag is not None and tag != run_tag()


# ── 射线后端（B 线的 tools/cad/ray_backend.py；没有就 trimesh 原路径）────────────────────
_RB = [None, False]      # [模块, 已尝试]


def ray_backend():
    """能 import tools/cad/ray_backend 就返回该模块（暴露 intersects_location / contains / name），否则 None。"""
    if not _RB[1]:
        _RB[1] = True
        p = str(Path(__file__).resolve().parent)
        if p not in sys.path:
            sys.path.insert(0, p)
        try:
            import ray_backend as rb           # noqa: PLC0415
            _RB[0] = rb
        except ImportError:
            _RB[0] = None
    return _RB[0]


def backend_name(mesh=None) -> str:
    rb = ray_backend()
    if rb is not None:
        return "ray_backend:" + str(rb.name())
    if mesh is not None:
        return "trimesh:" + type(mesh.ray).__module__ + "." + type(mesh.ray).__name__
    return "trimesh"


def versions() -> str:
    """进键的环境串：Python、平台、几何库版本、射线后端（embree 在不在 / 选没选、ray_backend 名）。"""
    if _versions[0] is None:
        import importlib.metadata as md, platform
        # 第五任（B 线 §7-3）：先 import ray_backend 并调 name() —— 它会把 .venv/hr42_embree 加进 sys.path；
        #   不先调的话 md.version("embreex") 取决于本进程之前有没有人 import 过 ray_backend，同一环境不同进程的版本串会不一样
        #   （只会多未命中、不会错，但"全命中"跑会被它打散）。
        rb = ray_backend()
        rb_name = rb.name() if rb is not None else "-"
        vs = [f"python={sys.version.split()[0]}", f"machine={platform.machine()}", f"system={platform.system()}"]
        for n in ("numpy", "trimesh", "manifold3d", "scipy", "shapely", "rtree", "embreex"):
            try:
                vs.append(f"{n}={md.version(n)}")
            except Exception:
                vs.append(f"{n}=-")
        try:
            import trimesh
            vs.append(f"trimesh.ray.has_embree={bool(trimesh.ray.has_embree)}")
        except Exception:
            pass
        vs.append(f"ray_backend={rb_name}")
        _versions[0] = ";".join(vs)
    return _versions[0]


def _h(*parts) -> bytes:
    h = hashlib.blake2b(digest_size=16)
    h.update(SCHEMA.encode()); h.update(b"\0"); h.update(versions().encode()); h.update(b"\0")
    for p in parts:
        if isinstance(p, bytes):
            b = p
        elif isinstance(p, str):
            b = p.encode()
        elif isinstance(p, np.ndarray):
            a = np.ascontiguousarray(p)
            b = str(a.dtype).encode() + str(a.shape).encode() + a.tobytes()
        elif isinstance(p, (int, float, bool)) or p is None:
            b = repr(p).encode()
        elif isinstance(p, (tuple, list)):
            b = _h(*p)
        else:
            raise TypeError(f"check_cache 键不认这种输入：{type(p)}")
        h.update(len(b).to_bytes(8, "little")); h.update(b)
    return h.digest()


def key(kind: str, *parts) -> bytes:
    return _h(kind, *parts)


def mesh_digest(mesh) -> bytes:
    """trimesh 网格内容摘要（float64 顶点 + int64 面的原始字节）。存在 trimesh 自己的 _cache 里：顶点/面一改，trimesh 会清掉它。"""
    c = getattr(mesh, "_cache", None)
    d = c["hr42_digest"] if c is not None else None
    if d is None:
        d = _h("mesh", np.asarray(mesh.vertices, dtype=np.float64), np.asarray(mesh.faces, dtype=np.int64))
        if c is not None:
            c["hr42_digest"] = d
    return d


def array_digest(*arrays) -> bytes:
    return _h("arrays", *[np.asarray(a) for a in arrays])


def _file(ns):
    return DIR / f"{ns}.pkl"


def _load(ns):
    if ns in _ns:
        return _ns[ns]
    d = {}
    if enabled() and DIR.exists():
        for p in [_file(ns)] + sorted(DIR.glob(f"{ns}.*.shard.pkl")):
            try:
                with open(p, "rb") as f:
                    d.update(pickle.load(f))
            except Exception:
                pass                     # 半截 / 坏分片：当没有（只会多算，不会错）
    _ns[ns] = d
    _new.setdefault(ns, {})
    _stats.setdefault(ns, {"hits": 0, "misses": 0, "same_run": 0, "from_runs": {}})
    return d


_CTX = [""]


def set_context(ctx: str):
    """调用方标一下"现在在算谁"（件对 / 件号 / 函数），只用于未命中日志（DUCK_CHECK_CACHE_MISSLOG），不进键、不影响结果。"""
    _CTX[0] = ctx


def _log_miss(ns):
    p = os.environ.get("DUCK_CHECK_CACHE_MISSLOG")
    if not p:
        return
    try:
        with open(p, "a") as f:
            f.write(f"{os.getpid()}\t{ns}\t{_CTX[0]}\n")
    except OSError:
        pass


# ── 逐格归属桶 ───────────────────────────────────────────────────────────────
_ATT = [None]       # 当前归属桶（None = 没开）


def _new_bucket():
    return {"hits": 0, "misses": 0, "same_run": 0, "from_runs": {}, "keys": []}


def attrib_begin():
    """开始逐格归属（gate 层进程在跑层之前调）。"""
    _ATT[0] = _new_bucket()


def attrib_take() -> dict | None:
    """取走当前桶（自上次 take 以来的全部查缓存记录）并开一个新桶；没开归属时返回 None。"""
    b = _ATT[0]
    if b is None:
        return None
    _ATT[0] = _new_bucket()
    return b


def attrib_end():
    _ATT[0] = None


def _attrib(kind: str, tag, k):
    """kind = "hit"（取自之前的 run）/ "same_run"（本轮别处算过、复用）/ "miss"（现算）。"""
    b = _ATT[0]
    if b is None:
        return
    if kind == "hit":
        b["hits"] += 1
        t = str(tag)
        b["from_runs"][t] = b["from_runs"].get(t, 0) + 1
    elif kind == "same_run":
        b["same_run"] += 1
    else:
        b["misses"] += 1
    b["keys"].append(k if isinstance(k, bytes) else str(k).encode())


def bucket_summary(b: dict | None) -> dict:
    """归属桶 → 写进格子的摘要：命中（之前 run 的）/ 现算 / 本轮复用 数、cached_from（run 号 → 次数）、
    键哈希（本格用到的全部原语键排序后的 blake2b）。"""
    if not b:
        return {"hits": 0, "misses": 0, "same_run": 0, "cached_from": {}, "key": None}
    h = hashlib.blake2b(digest_size=8)
    for k in sorted(b["keys"]):
        h.update(k)
    return {"hits": b["hits"], "misses": b["misses"], "same_run": b.get("same_run", 0),
            "cached_from": dict(sorted(b["from_runs"].items())), "key": h.hexdigest() if b["keys"] else None}


def merge_bucket(a: dict | None, b: dict | None) -> dict:
    out = _new_bucket()
    for x in (a, b):
        if not x:
            continue
        out["hits"] += x["hits"]; out["misses"] += x["misses"]; out["same_run"] += x.get("same_run", 0); out["keys"] += list(x["keys"])
        for t, n in x["from_runs"].items():
            out["from_runs"][t] = out["from_runs"].get(t, 0) + n
    return out


def note(ns, hit: bool, tag, k):
    """值不是本进程查缓存拿到的（进程池预取、影子跑、沿用的切片记录），按它原本的出处补记：统计 + 归属。
    hit=True 且 tag 是之前的 run 号 → 取自缓存；hit=True 但 tag 是本轮 run 号 → 本轮复用；hit=False → 本轮现算。"""
    if not enabled():
        return
    _load(ns)
    st = _stats[ns]
    if hit and is_prev(tag):
        st["hits"] += 1
        st["from_runs"][str(tag)] = st["from_runs"].get(str(tag), 0) + 1
        _attrib("hit", tag, k)
    elif hit:
        st["same_run"] += 1
        _attrib("same_run", tag, k)
    else:
        st["misses"] += 1
        _attrib("miss", None, k)


def lookup(ns, k):
    """查到返回 (True, value, 来源 run 号)；没有返回 (False, None, None)。关了缓存恒未命中且不计数、不归属。
    来源 run 号 == 本轮 run 号（is_prev 为假）= 本轮别处刚算过的，统计记 same_run、归属按本轮现算。"""
    if not enabled():
        return False, None, None
    d = _load(ns)
    st = _stats[ns]
    if k in d:
        v, tag = d[k]
        if is_prev(tag):
            st["hits"] += 1
            st["from_runs"][tag] = st["from_runs"].get(tag, 0) + 1
            _attrib("hit", tag, k)
        else:
            st["same_run"] += 1
            _attrib("same_run", tag, k)
        return True, v, tag
    st["misses"] += 1
    _attrib("miss", None, k)
    _log_miss(ns)
    return False, None, None


def get(ns, k):
    """命中返回 (True, value)；未命中返回 (False, None)。关了缓存恒未命中且不计数。"""
    hit, v, _tag = lookup(ns, k)
    return hit, v


def put(ns, k, value):
    if not enabled():
        return
    d = _load(ns)
    tag = run_tag()
    d[k] = (value, tag)
    _new[ns][k] = (value, tag)


def cached(ns, k, compute):
    hit, v = get(ns, k)
    if hit:
        return v
    v = compute()
    put(ns, k, v)
    return v


def flush():
    """把本进程新算的条目写成分片（原子改名）。进程池工作进程在每段任务末尾调它（它们退出走 os._exit，atexit 不跑）。"""
    if not enabled():
        return
    for ns, new in _new.items():
        if not new:
            continue
        DIR.mkdir(parents=True, exist_ok=True)
        _shard_seq[0] += 1
        p = DIR / f"{ns}.{os.getpid()}.{_shard_seq[0]}.shard.pkl"
        tmp = p.with_suffix(".tmp")
        with open(tmp, "wb") as f:
            pickle.dump(new, f, protocol=pickle.HIGHEST_PROTOCOL)
        os.replace(tmp, p)
        _new[ns] = {}


def write_stats():
    """进程池工作进程：环境变量 DUCK_CHECK_CACHE_STATS_DIR 给了就把本进程累计统计写成 stats.<pid>.json（覆盖写），主进程事后 collect_stats() 汇总。"""
    d = os.environ.get("DUCK_CHECK_CACHE_STATS_DIR")
    if not d or not enabled():
        return
    import json
    os.makedirs(d, exist_ok=True)
    tmp = os.path.join(d, f".stats.{os.getpid()}.tmp")
    with open(tmp, "w") as f:
        json.dump(stats(), f)
    os.replace(tmp, os.path.join(d, f"stats.{os.getpid()}.json"))


def collect_stats(d) -> dict:
    import json
    out = {}
    for p in sorted(Path(d).glob("stats.*.json")):
        try:
            out = merge_stats(out, json.loads(p.read_text()))
        except Exception:
            pass
    return out


def worker_flush():
    """进程池工作进程每段任务末尾调：写分片 + 写统计（它们退出走 os._exit，atexit 不跑）。"""
    flush(); write_stats()


def compact(namespaces=None):
    """主进程收尾：主文件 + 全部分片合并成新主文件（加锁），删掉已合并的分片。"""
    if not enabled() or not DIR.exists():
        return
    flush()
    lock = DIR / ".lock"
    with open(lock, "a+") as lf:
        fcntl.flock(lf, fcntl.LOCK_EX)
        names = namespaces or sorted({p.name.split(".")[0] for p in DIR.glob("*.pkl")})
        for ns in names:
            shards = sorted(DIR.glob(f"{ns}.*.shard.pkl"))
            if not shards:
                continue
            d = {}
            for p in [_file(ns)] + shards:
                try:
                    with open(p, "rb") as f:
                        d.update(pickle.load(f))
                except Exception:
                    pass
            tmp = _file(ns).with_suffix(".tmp")
            with open(tmp, "wb") as f:
                pickle.dump(d, f, protocol=pickle.HIGHEST_PROTOCOL)
            os.replace(tmp, _file(ns))
            for p in shards:
                try:
                    p.unlink()
                except OSError:
                    pass
        fcntl.flock(lf, fcntl.LOCK_UN)


def absorb_stats(other: dict):
    """把子进程（进程池）汇总来的统计并进本进程的统计（evidence() 才完整）。"""
    for ns, v in (other or {}).items():
        st = _stats.setdefault(ns, {"hits": 0, "misses": 0, "same_run": 0, "from_runs": {}})
        st["hits"] += v.get("hits", 0); st["misses"] += v.get("misses", 0); st["same_run"] += v.get("same_run", 0)
        for t, n in (v.get("from_runs") or {}).items():
            st["from_runs"][t] = st["from_runs"].get(t, 0) + n


def reset_stats():
    """一个进程里连跑几个任务（gate 的层进程）时，每个任务开头清零统计（已读入的缓存不丢）。"""
    for st in _stats.values():
        st["hits"] = 0; st["misses"] = 0; st["same_run"] = 0; st["from_runs"] = {}


def stats():
    return {ns: dict(hits=s["hits"], misses=s["misses"], same_run=s.get("same_run", 0), from_runs=dict(s["from_runs"]))
            for ns, s in _stats.items()}


def merge_stats(a: dict, b: dict) -> dict:
    out = {k: dict(hits=v.get("hits", 0), misses=v.get("misses", 0), same_run=v.get("same_run", 0), from_runs=dict(v.get("from_runs") or {}))
           for k, v in (a or {}).items()}
    for ns, s in (b or {}).items():
        r = out.setdefault(ns, dict(hits=0, misses=0, same_run=0, from_runs={}))
        r["hits"] += s.get("hits", 0); r["misses"] += s.get("misses", 0); r["same_run"] += s.get("same_run", 0)
        for t, n in (s.get("from_runs") or {}).items():
            r["from_runs"][t] = r["from_runs"].get(t, 0) + n
    return out


def evidence(extra_stats=None) -> dict:
    """写进 evidence 的一段：开没开、存在哪、键版本、本进程（+ 子进程汇总）命中/未命中、命中条目来自哪些 run 标签。"""
    st = merge_stats(stats(), extra_stats)
    return dict(enabled=enabled(), store=str(DIR.relative_to(ROOT)) if DIR.is_relative_to(ROOT) else str(DIR),
                schema=SCHEMA, versions=versions(), run_tag=run_tag() if enabled() else None,
                namespaces=st,
                note="只缓存纯几何原语（布尔体积/交集网格、min_gap、射线、contains、L1 截面/壁厚/支撑），键 = 全部数值输入字节 + 库版本/后端；判据每次现判"
                     if enabled() else "DUCK_CHECK_CACHE=0：全量实跑，不读不写缓存")


# ── 常用原语 ────────────────────────────────────────────────────────────────
def ray_location(mesh, origins, directions, multiple_hits=True, ns="rays"):
    """= mesh.ray.intersects_location(origins, directions, multiple_hits=...)（有 ray_backend 就走它，同形返回），
    结果按 (mesh 内容, 射线字节, 后端) 缓存。"""
    o = np.asarray(origins, dtype=np.float64); d = np.asarray(directions, dtype=np.float64)
    k = key("ray_location", mesh_digest(mesh), o, d, bool(multiple_hits), backend_name(mesh))
    hit, v = get(ns, k)
    if hit:
        return tuple(np.array(x, copy=True) for x in v)
    rb = ray_backend()
    if rb is not None:
        r = rb.intersects_location(mesh, o, d, multiple_hits=multiple_hits)
    else:
        r = mesh.ray.intersects_location(o, d, multiple_hits=multiple_hits)
    v = tuple(np.asarray(x) for x in r)
    put(ns, k, v)
    return tuple(np.array(x, copy=True) for x in v)


def contains(mesh, points, ns="contains"):
    """= mesh.contains(points)（有 ray_backend 就走它），按 (mesh 内容, 点字节, 后端) 缓存。"""
    p = np.asarray(points, dtype=np.float64)
    k = key("contains", mesh_digest(mesh), p, backend_name(mesh))
    hit, v = get(ns, k)
    if hit:
        return np.array(v, copy=True)
    rb = ray_backend()
    r = np.asarray(rb.contains(mesh, p) if rb is not None else mesh.contains(p))
    put(ns, k, r)
    return np.array(r, copy=True)


if __name__ != "__main__":
    run_tag()                    # import 时就定下 run 号并写进环境变量：之后起的子进程（进程池 / 影子跑）继承同一个号
    atexit.register(lambda: (flush(), compact()) if enabled() else None)
