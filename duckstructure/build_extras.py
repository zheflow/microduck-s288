"""hr46（2026-09-26）：件的附属实体（线束钩 / 卡槽）的登记表与存读导出。
单独成模块的原因：登记表是进程内状态，必须全进程只有一份。build_fast.py 会被 `python -m duckstructure.build_fast` 当 __main__ 跑，
这时 neck.py 里 `from duckstructure.build_fast import produce_extra` 拿到的是第二份模块对象、另一张表（hr46 子 agent 18:16 发现）。
本模块只被 import、永不当 __main__，两份 build_fast 都从这里拿同一张表。对外仍可 `from duckstructure.build_fast import produce_extra`（re-export）。"""
import os, re, hashlib
import numpy as np, trimesh


def mesh_digest(v, f):
    h = hashlib.blake2b(digest_size=16)
    for a in (np.ascontiguousarray(v, dtype=np.float64), np.ascontiguousarray(f, dtype=np.int64)):
        h.update(str(a.shape).encode()); h.update(a.tobytes())
    return h.hexdigest()


# ── hr46（2026-09-26）：件的附属实体 ──────────────────────────────────────────────────────────────────────────
#   钩 / 卡槽这类"和件同一次生成、但要单独成文件"的小实体。件的生成函数画完钩后调 produce_extra(件名, 标签, 网格)，
#   标签 = wire_fixings 锚点 id（一钩一文件）。走向与主网格完全相同：
#     工作进程 produce() 存 <件>.__extra__<标签>.npz（result.json 记 {标签: 摘要}）→ 主进程读回、核摘要；
#     命中时从 cache/ 读回（manifest 件条目 hr42_extras = {标签: 摘要}，缺文件 / 摘要不符 = 整组未命中）；
#     导出阶段 export_extras() 写 OUT/hooks/<件号>_<标签>.stl（件号 = 导出文件名前 3 位）。
#   同一 npz → 同一导出：增量 build 与无缓存 build 的 hooks/*.stl 逐字节相同。Gate 第 6 层只读这些文件做"线撞自己固定件"的 (a) 区。
#   镜像件（*_R）上的附属实体还没做：左件若登记了，镜像步直接报错（mirror_extras），不许静默丢。
EXTRA_DIRNAME = "hooks"
_EXTRAS = {}                                  # 件名 → {标签: trimesh}；本进程内登记（工作进程 / 主进程 / 快车道各自一份）
_EXTRA_TAG_RE = re.compile(r"[A-Za-z0-9_.-]{1,80}")


def produce_extra(name, tag, mesh):
    """登记件 name 的附属实体 tag（坐标系 = 该件生成函数返回网格的坐标系，即世界系）。同件同标签登记两次 = bug，直接报错。"""
    if not isinstance(name, str) or not name:
        raise ValueError(f"produce_extra：件名非法 {name!r}")
    if not isinstance(tag, str) or not _EXTRA_TAG_RE.fullmatch(tag):
        raise ValueError(f"produce_extra：标签 {tag!r} 只许 [A-Za-z0-9_.-]，1–80 字")
    v = np.asarray(mesh.vertices, dtype=np.float64); f = np.asarray(mesh.faces, dtype=np.int64)
    if len(v) == 0 or len(f) == 0:
        raise ValueError(f"produce_extra：{name}/{tag} 是空网格")
    d = _EXTRAS.setdefault(name, {})
    if tag in d:                                   # 生成函数会被别的件反复调用（如 build_neck 被腿/躯干/头的扫掠调）：同几何 = 幂等
        if mesh_digest(v, f) == mesh_digest(np.asarray(d[tag].vertices), np.asarray(d[tag].faces)):
            return
        raise ValueError(f"produce_extra：附属实体 {name}/{tag} 登记了两次且几何不同")
    d[tag] = trimesh.Trimesh(vertices=v.copy(), faces=f.copy(), process=False)


def extras_of(name):
    return dict(_EXTRAS.get(name, {}))


def mirror_extras(name, name_R, mirror_fn):
    """build 的镜像步（左件 → 右件）。镜像件的附属实体导出还没做：左件登记了就报错。做的时候：右钩 = mirror_fn(左钩)，文件名另定（<件号>_<标签> 会与左件撞名）。"""
    ex = _EXTRAS.get(name)
    if ex:
        raise NotImplementedError(f"件 {name} 登记了附属实体 {sorted(ex)}，但镜像件 {name_R} 的附属实体导出还没做（build_fast.mirror_extras）")


def _extra_npz(d, name, tag):
    return os.path.join(d, f"{name}.__extra__{tag}.npz")


def _save_extras(d, name, clean):
    """把 name 的附属实体存进目录 d，返回 {标签: 摘要}。clean=True 先删该件旧的 __extra__ 文件（去掉的钩不能留旧文件）；
    工作进程必须 clean=False：目录是新建的，而且 os.listdir 会被依赖记录器记成目录依赖、把组键搞成永远未命中。"""
    if clean:
        for fn in os.listdir(d):
            if fn.startswith(f"{name}.__extra__") and fn.endswith(".npz"):
                os.remove(os.path.join(d, fn))
    out = {}
    for tag, m in sorted(_EXTRAS.get(name, {}).items()):
        v, f = np.asarray(m.vertices, dtype=np.float64), np.asarray(m.faces, dtype=np.int64)
        np.savez(_extra_npz(d, name, tag), v=v, f=f)
        out[tag] = mesh_digest(v, f)
    return out


def _load_extras(d, name, expect):
    """从目录 d 读回 name 的附属实体（expect = {标签: 摘要}），逐个核摘要。成功返回 None 并写进登记表；缺文件 / 摘要不符返回原因。"""
    got = {}
    for tag, dig in sorted((expect or {}).items()):
        p = _extra_npz(d, name, tag)
        if not os.path.exists(p):
            return f"{name} 附属实体 {tag} 缓存缺失"
        z = np.load(p)
        if mesh_digest(z["v"], z["f"]) != dig:
            return f"{name} 附属实体 {tag} 缓存摘要对不上"
        got[tag] = trimesh.Trimesh(vertices=z["v"], faces=z["f"], process=False)
    _EXTRAS[name] = got
    return None


def _pid_of(name, fname_of=None):
    from duckstructure.build_fast import FNAME                # 纯数据表，延迟取，避免循环 import
    fn = (fname_of or {}).get(name) or FNAME.get(name)
    return fn[:3] if fn else None


def export_extras(names, fname_of=None, clear_all=False):
    """names 里各件的附属实体 → OUT/hooks/<件号>_<标签>.stl，返回写出的文件名（排序）。
    clear_all=True（全量 build）先清空整个目录；否则只删 names 里各件号的旧文件（快车道只动重切件）。有附属实体却认不出件号 → 报错。"""
    from duckstructure.lib import OUT
    hd = os.path.join(OUT, EXTRA_DIRNAME); os.makedirs(hd, exist_ok=True)
    if clear_all:
        for fn in os.listdir(hd):
            if fn.endswith(".stl"):
                os.remove(os.path.join(hd, fn))
    written = []
    for name in names:
        ex = _EXTRAS.get(name, {})
        pid = _pid_of(name, fname_of)
        if pid is None:
            if ex:
                raise RuntimeError(f"件 {name} 有附属实体 {sorted(ex)} 但认不出件号（件表 / FNAME 都没有）")
            continue
        if not clear_all:
            for fn in os.listdir(hd):
                if fn.startswith(pid + "_") and fn.endswith(".stl"):
                    os.remove(os.path.join(hd, fn))
        for tag, m in sorted(ex.items()):
            p = os.path.join(hd, f"{pid}_{tag}.stl"); m.export(p); written.append(os.path.basename(p))
    return sorted(written)


