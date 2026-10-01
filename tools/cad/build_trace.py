"""hr42（2026-09-24）：build 分组工作进程的依赖记录器 —— 必须在 import duckstructure **之前** start()。

记下面几样（只记仓库内、.venv 与 __pycache__ 以外的文件）：
  · 生成期执行过的代码对象（sys.monitoring PY_START；每个代码位置第一次执行就记下并 DISABLE，开销可忽略）；
  · import 期执行过的**函数**，按"当时正在执行哪个仓库模块的模块体"归属（PY_START/PY_RETURN 维护模块体栈）——
    模块体在 import 时调的函数决定该模块的模块级状态，谁依赖这个模块，谁就连带依赖这些函数所在的文件；
  · **读**过的非 .py 文件（审计事件 open；.py 是 import 系统读源码，归代码依赖）；import 期与生成期都记。
    只记"读到了开跑前就在盘上的内容"的打开：只写 / 截断 / 新建的打开记成本进程的产出（writes），不进依赖；
    本进程自己写过的文件再读回来也不算依赖（第四任修：原来把工作进程自己存的 npz 记成了读依赖，写完又被删 → 组键永远对不上）；
  · 列过的目录（审计事件 os.listdir / os.scandir / glob.glob）；
  · 探测过存在性的路径（os.stat / os.lstat：os.path.exists / isfile / isdir、Path.exists 都走它）——
    "有缓存文件就读、没有就现算"这类分支由存在性决定，首次探测时的状态（文件 / 目录 / 不存在）也进键。
duckstructure/build_fast.py 在工作进程末尾调 report()，再把"执行过的函数体里的 import 语句 + 模块顶层 import 闭包 + import 期归属"
算成模块依赖（见 build_fast.group_deps）。本文件不 import 任何仓库模块。
"""
import os, sys

ROOT = os.path.realpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", ".."))
_PFX = ROOT + os.sep
_SELF = os.path.realpath(os.path.abspath(__file__))
_TOOL = None
_phase = ["import"]
_gen = set()          # 生成期执行过的仓库代码对象 (绝对路径, co_firstlineno, co_qualname)
_modstack = []        # import 期正在执行的仓库模块体（绝对路径）
_imp_attr = {}        # import 期：模块体文件 → {在它的模块体执行期间执行过的函数 (路径, 行, qualname)}
_imp_mods = set()     # import 期执行过模块体的仓库文件
_reads = {}           # 非 .py 文件绝对路径 → 首次（读）打开的阶段
_reads_stat = {}      # 非 .py 文件绝对路径 → 首次打开时的 (mtime_ns, size)：收尾时核对"生成期间没被改过"
_writes = {}          # 本进程写 / 新建 / 截断 / 删过的仓库路径 → 首次的阶段（产出，不进依赖）
_probes = {}          # 探测过存在性的仓库路径 → 首次探测时的 "file" / "dir" / "absent" / "other"
_py_sha0 = {}         # start() 那一刻仓库 .py 源码的 sha256（相对路径 → sha）：组键用"开跑时的源码"，收尾时再核对一遍
_pyreads = set()      # 被打开的 .py（import 系统读源码也会走这里，仅作记录）
_lists = {}           # 目录绝对路径 → 阶段
_orig_stat = os.stat  # 未包装的 os.stat / os.lstat（钩子自己用，不记探测）
_orig_lstat = os.lstat
import threading as _threading
_tl = _threading.local()   # 每线程的"正在钩子里"标志：钩子内部自己调 stat/realpath 不记、不递归（线程各自一份，别的线程照记）


def _busy():
    return getattr(_tl, "busy", False)


def _inrepo(p):
    return p.startswith(_PFX) and (os.sep + ".venv" + os.sep) not in p and (os.sep + "__pycache__" + os.sep) not in p


def _real(p):
    try:
        return os.path.realpath(os.fsdecode(p))
    except Exception:
        return None


def _kind(p):
    import stat as _st
    try:
        m = _orig_stat(p).st_mode
    except OSError:
        return "absent"
    return "dir" if _st.S_ISDIR(m) else ("file" if _st.S_ISREG(m) else "other")


def _on_start(code, offset):
    fn = code.co_filename
    if not fn or fn == _SELF or not _inrepo(fn):
        return sys.monitoring.DISABLE
    if _phase[0] == "generation":
        _gen.add((fn, code.co_firstlineno, code.co_qualname))
        return sys.monitoring.DISABLE
    if code.co_name == "<module>":
        _modstack.append(fn); _imp_mods.add(fn)
    elif _modstack:
        _imp_attr.setdefault(_modstack[-1], set()).add((fn, code.co_firstlineno, code.co_qualname))
    return None                              # import 期不 DISABLE：同一函数可能在不同模块体里被调，要逐个归属


def _on_return(code, offset, retval):
    if _phase[0] == "generation":
        return sys.monitoring.DISABLE
    fn = code.co_filename
    if code.co_name == "<module>" and _modstack and _modstack[-1] == fn:
        _modstack.pop()
    return None


def _open_mode(args):
    """审计事件 open 的 (path, mode, flags) → (读到旧内容?, 写?)。builtins.open 给 mode 串 + flags；os.open 给 None + flags。"""
    mode = args[1] if len(args) > 1 else None
    flags = args[2] if len(args) > 2 else None
    if isinstance(flags, int) and flags >= 0:
        acc = flags & os.O_ACCMODE
        reads = acc in (os.O_RDONLY, os.O_RDWR) and not (flags & os.O_TRUNC)
        writes = acc in (os.O_WRONLY, os.O_RDWR) or bool(flags & (os.O_CREAT | os.O_TRUNC | os.O_APPEND))
        return reads, writes
    m = mode if isinstance(mode, str) else "r"
    reads = ("r" in m or "+" in m) and "w" not in m
    writes = any(c in m for c in "wax+")
    return reads, writes


def _audit(ev, args):
    if _busy():
        return
    _tl.busy = True
    try:
        _audit2(ev, args)
    finally:
        _tl.busy = False


def _audit2(ev, args):
    if ev == "open":
        p = args[0] if args else None
        if not isinstance(p, (str, bytes, os.PathLike)):
            return                               # 按 fd 打开（os.fdopen 等）：不是路径
        p = _real(p)
        if p is None or not _inrepo(p):
            return
        if p.endswith(".py"):
            _pyreads.add(p)
            return
        reads, writes = _open_mode(args)
        existed = _kind(p) != "absent"           # 钩子在真正打开之前触发：此刻不存在 = 这次是新建
        if reads and existed and p not in _writes and p not in _reads:
            _reads[p] = _phase[0]
            try:
                st = _orig_stat(p); _reads_stat[p] = (st.st_mtime_ns, st.st_size)
            except OSError:
                _reads_stat[p] = None
        if writes and p not in _writes:
            _writes[p] = _phase[0]
    elif ev in ("os.listdir", "os.scandir", "glob.glob", "glob.glob/2"):
        p = args[0] if args else None
        if p is None:
            p = "."
        if isinstance(p, (str, bytes, os.PathLike)):
            p = _real(p)
            if p is not None and _inrepo(p) and p not in _lists:
                _lists[p] = _phase[0]
    elif ev in ("os.mkdir", "os.remove", "os.rmdir", "os.rename", "os.replace", "os.truncate", "shutil.rmtree",
                "shutil.move", "shutil.copyfile", "os.symlink", "os.link"):
        # 本进程改动盘面的路径：记成产出（rename/replace/copy/link 的目标是第二个参数；删 / 截断 / 建目录是第一个）
        cand = []
        if ev in ("os.rename", "os.replace", "shutil.move", "shutil.copyfile", "os.symlink", "os.link"):
            cand = list(args[:2])
        elif args:
            cand = [args[0]]
        for q in cand:
            if isinstance(q, (str, bytes, os.PathLike)):
                q = _real(q)
                if q is not None and _inrepo(q) and q not in _writes:
                    _writes[q] = _phase[0]


def _wrap_stat(orig):
    def stat(path, *a, **k):
        if not _busy() and isinstance(path, (str, bytes, os.PathLike)):
            _tl.busy = True
            try:
                q = _real(path)
                if q is not None and _inrepo(q) and not q.endswith(".py") and q not in _probes:
                    _probes[q] = _kind(q)
            finally:
                _tl.busy = False
        return orig(path, *a, **k)
    stat.__name__ = orig.__name__; stat.__doc__ = orig.__doc__
    return stat


def _snapshot_py():
    import hashlib
    for sub in ("duckstructure", os.path.join("tools", "cad")):
        d = os.path.join(ROOT, sub)
        for n in sorted(os.listdir(d)):
            if n.endswith(".py"):
                q = os.path.join(d, n)
                with open(q, "rb") as f:
                    _py_sha0[os.path.relpath(q, ROOT)] = hashlib.sha256(f.read()).hexdigest()


def start():
    global _TOOL
    _snapshot_py()                                   # 先拍源码（在装审计钩子之前，免得把自己读源码记成数据读）
    sys.addaudithook(_audit)
    os.stat = _wrap_stat(_orig_stat)                 # 只包 os 模块属性：import 系统用的是 posix.stat，不受影响
    os.lstat = _wrap_stat(_orig_lstat)
    mon = sys.monitoring
    for tid in (3, 4, 1, 2, 0):                      # 3、4 是 CPython 留给第三方工具的空号；cProfile 占 2
        if mon.get_tool(tid) is None:
            _TOOL = tid
            break
    if _TOOL is None:
        raise RuntimeError("sys.monitoring 没有空闲 tool id")
    mon.use_tool_id(_TOOL, "hr42-build-trace")
    mon.register_callback(_TOOL, mon.events.PY_START, _on_start)
    mon.register_callback(_TOOL, mon.events.PY_RETURN, _on_return)
    mon.set_events(_TOOL, mon.events.PY_START | mon.events.PY_RETURN)


def mark_generation():
    """import 结束、开始生成：之后执行的代码对象记到生成期。被 DISABLE 的代码位置要 restart_events 才会再报。"""
    _phase[0] = "generation"
    if _TOOL is not None:
        sys.monitoring.set_events(_TOOL, sys.monitoring.events.PY_START)
        sys.monitoring.restart_events()


def active():
    return _TOOL is not None


def report():
    rel = lambda f: os.path.relpath(f, ROOT)
    gen, attr = list(_gen), {k: list(v) for k, v in list(_imp_attr.items())}   # 先拍快照：下面的推导式本身也会触发 PY_START
    reads, lists, pyr, mods = dict(_reads), dict(_lists), set(_pyreads), set(_imp_mods)
    rstat, writes, probes = dict(_reads_stat), dict(_writes), dict(_probes)
    # 读依赖：钩子里已经只记"开跑前就在盘上、本进程还没写过"的读（先读后写的仍算：读到的是开跑前的内容）。
    # 探测：记首次探测时的状态，不剔本进程后来自己建的路径 —— "没有就现算并写出"的缓存文件，下次存在了走读分支，必须重切一次（宁可多算）。
    return dict(root=ROOT, py_sha_start=dict(_py_sha0),
                reads_stat={rel(p): (list(v) if v else None) for p, v in sorted(rstat.items()) if p in reads},
                generation_codes=sorted([rel(f), ln, q] for f, ln, q in gen),
                import_attrib={rel(m): sorted([rel(f), ln, q] for f, ln, q in s) for m, s in sorted(attr.items())},
                import_modules=sorted(rel(m) for m in mods),
                reads={rel(p): ph for p, ph in sorted(reads.items())},
                writes={rel(p): ph for p, ph in sorted(writes.items())},
                probes={rel(p): k for p, k in sorted(probes.items())},
                py_reads=sorted(rel(p) for p in pyr),
                lists={rel(p): ph for p, ph in sorted(lists.items())})

