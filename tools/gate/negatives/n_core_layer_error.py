#!/usr/bin/env python3
"""内核反例（gate_guard，2026-09-25）—— 层崩溃 / 层空跑必须醒目报 BLOCK，不许静默显示"0 条判据"。

事故（校验会话 09-25 18:55）：fasteners.yaml 一条 joins 含半角逗号没加引号，YAML 把它拆成 [str, int] → l5_function 里
" ".join 抛 TypeError，整个模块没交回任何判据；其它层照跑，终端只一行"0 条判据，0 条红 … [层异常]"，记分卡里只多一个
淹在几百个红格里的 L5/_layer，很容易被当成"L5 全过"。

造法（不碰真层、不碰 tools/gate/out 的真记分卡）：gate.discover 换成返回内存里的假层模块（登记进 sys.modules["layers.<名>"]，
gate._run_layer_task 按名 import 拿到它们）；gate.load_data 换成最小数据（义务只剩 8 个 _layer + 1 条特征 + 1 个装配步，假层全部盖住）；
反例包换成桩（不递归跑反例）；core.git_head 换成桩（省 git status）。然后走**真 gate.main**（--out 反例临时目录）：
  C 干净全量（--jobs 1）：8 层都正常 → CLEAR、退出码 0、没有 _layer_error 格、没有 ⚠ 行（守门不误报）；
  A 串行全量：L3 空跑（0 条判据）；L5 两个模块，一个真抛 TypeError（" ".join(["M2 自攻…", 2]) = 事故原样）、另一个正常；
    L7 返回不可解析 → L3/L5/L7 各一格 _layer_error（FAIL，BLOCK；L5 detail 带 TypeError + traceback 末 5 行 + "本层判据全部未评"），
    其余层没有；BLOCKED、退出码 1；终端摘要顶部（结论行之前第一段）有"⚠ 第 5 层 功能达成 崩溃…TypeError…0 条判据不是通过"；
    记分卡 md 标题下第一段就是报错行；
  B 子集（--layers 3,5）：L3 空跑只打印提示行、不加格；L5 崩溃照样加格（崩溃不分跑法）；
  D 并行全量（--jobs 2，真 spawn 进程池）：L5 在层进程里真抛 TypeError → 同 A 的 L5 格；
  E 并行全量：L5 层进程 SIGKILL 自己（= 看门狗 kill -9 / 段错误）→ 进程池断 → L5/_layer_error（BrokenProcessPool）、退出码 1，
    gate.main 正常返回、不整轮炸掉。
全部 assertions（红来自 gate.main 自己的守门，不经过真层）。旧 gate.py（无守门）上跑：没有 _layer_error 格、没有 ⚠ 行，
A 的"返回不可解析"与 E 的 BrokenProcessPool 直接把 gate.main 炸掉 → 本反例红。
"""
from __future__ import annotations

import contextlib
import io
import json
import os
import shutil
import signal
import sys
import types
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))   # tools/gate
sys.path.insert(0, str(Path(__file__).resolve().parent))      # negatives/（层进程按名 import 本文件拿 _child_task / _stub_negatives）

NAME = "层崩溃 / 层空跑：L<n>/_layer_error BLOCK + 摘要与记分卡顶部 ⚠ 报错 + 退出码 1；干净运行不出格"
EXPECT = ("TypeError / 被杀 / 返回不可解析 → _layer_error BLOCK、BLOCKED、rc=1、摘要与 md 顶部有 ⚠ 第 5 层…崩溃；"
          "全量空跑加格、子集空跑只提示；干净 → CLEAR、rc=0、无格无 ⚠")

_COVER = ("FX", "AX")
_MINI = {"features": {"features": [{"id": "FX"}]}, "assembly": {"assembly_order": [{"id": "AX"}]}}
ALL_OK = [f"l{n}_negfake_ok" for n in range(8)] + ["l5_negfake_ok2"]


def _mini_data():
    return json.loads(json.dumps(_MINI))


def _mk_run(n, kind):
    def run(ctx):
        from core import LayerResult, LAYER_NAMES, PASS
        r = LayerResult(n, LAYER_NAMES[n])
        if kind == "boom":
            joins = ["M2 自攻 底孔深 4", 2]          # YAML 把 "…, 2 颗" 这种没加引号的项拆成 [str, int] 的样子
            r.detail = " ".join(joins)              # → TypeError: sequence item 1: expected str instance, int found（事故原样）
        if kind == "kill":
            os.kill(os.getpid(), signal.SIGKILL)    # = 看门狗 kill -9 / 段错误：层进程直接没了
        if kind in ("ok", "ok2"):
            r.add(subject="_negfake", check=f"negfake_{kind}", state=PASS, measured=1,
                  criterion="反例桩：本模块正常发出 1 条判据", evidence_n=1)
            r.covered = set(_COVER)
        return r                                    # empty：0 条判据
    return run


def _module(name):
    """名字 = l<层>_negfake_<种类>，种类 ok / ok2 / empty / boom / kill / junk（junk 由 _junk_wrap 截走，run 不会被调）。"""
    from core import LAYER_NAMES
    n, kind = int(name[1]), name.rsplit("_", 1)[1]
    m = types.ModuleType(f"layers.{name}")
    m.LAYER, m.NAME, m.run = n, LAYER_NAMES[n], _mk_run(n, kind)
    return m


def _install(names):
    mods = {}
    for nm in names:
        m = _module(nm)
        sys.modules[f"layers.{nm}"] = m
        mods.setdefault(m.LAYER, []).append(m)
    return mods


def _child_task(task):
    """并行场景的层进程入口（按名 pickle 成 n_core_layer_error._child_task）：在层进程里登记假层 + 最小数据，再交给真的 gate._run_layer_task。"""
    import gate
    gate.load_data = _mini_data
    _install([task[1]])
    return gate._run_layer_task(task)


def _stub_negatives(use_cache):
    return {"all_passed": True, "n": 1, "rows": [{"file": "n_stub.py", "passed": True}],
            "note": "反例包桩（n_core_layer_error）", "cached": False, "seconds": 0.0}


def _summary_block(stdout):
    """终端摘要 = 最后一行"累计"之后、结论行之前的非空行（去掉 !!! 分隔线）。"""
    lines = stdout.splitlines()
    vi = max((i for i, ln in enumerate(lines) if ln.startswith(("BLOCKED", "CLEAR", "INCOMPLETE"))), default=None)
    li = max((i for i, ln in enumerate(lines) if "累计" in ln), default=-1)
    if vi is None:
        return [], None
    return [ln for ln in lines[li + 1:vi] if ln.strip() and set(ln.strip()) != {"!"}], lines[vi]


def run() -> dict:
    import gate
    import core
    from _harness import _TMP
    import n_core_layer_error as me             # 按名 import 的本模块：它的函数才能 pickle 进 spawn 层进程
    root = _TMP / "layer_error_guard"
    shutil.rmtree(root, ignore_errors=True)
    saved = {"discover": gate.discover, "load_data": gate.load_data, "task": gate._run_layer_task,
             "neg": gate._run_negatives_task, "git": core.git_head, "pool": getattr(gate, "_POOL_BROKEN", None)}
    env = {k: os.environ.get(k) for k in ("DUCK_CHECK_CACHE", "DUCK_CHECK_CACHE_RUN_ID")}
    installed = set()
    res = {}

    def scenario(tag, names, argv, task=None):
        def fake_discover(want=None):
            mods = _install([nm for nm in names if not want or int(nm[1]) in want])
            installed.update(f"layers.{nm}" for nm in names)
            return mods, {}
        gate.discover = fake_discover
        gate._run_layer_task = task or saved["task"]
        out = root / tag
        buf, rc, err = io.StringIO(), None, None
        try:
            with contextlib.redirect_stdout(buf):
                rc = gate.main(argv + ["--out", str(out), "--no-check-cache"])
        except (Exception, SystemExit) as e:          # 旧 gate.py 在"返回不可解析"/断池时会直接炸 —— 记下来当失败
            err = f"{type(e).__name__}: {e}"
        stem = "scorecard.partial" if ("--layers" in argv or "--parts" in argv) else "scorecard"
        try:
            sc = json.loads((out / f"{stem}.json").read_text(encoding="utf-8"))
            md = (out / f"{stem}.md").read_text(encoding="utf-8")
        except OSError:
            sc, md = {"cells": {}, "blocking_cells": [], "verdict": None}, ""
        res[tag] = dict(rc=rc, err=err, out=buf.getvalue(), sc=sc, md=md, pool=getattr(gate, "_POOL_BROKEN", None))
        return res[tag]

    def junk_wrap(task):                            # 串行：L7 的"层进程"交回的不是 dict
        return "不是 dict 的垃圾结果" if task[1].endswith("_junk") else saved["task"](task)

    try:
        gate.load_data = _mini_data
        gate._run_negatives_task = me._stub_negatives
        core.git_head = lambda: "neg-stub"
        C = scenario("C_clean", ALL_OK, ["--jobs", "1"])
        A = scenario("A_serial", ["l0_negfake_ok", "l1_negfake_ok", "l2_negfake_ok", "l3_negfake_empty", "l4_negfake_ok",
                                  "l5_negfake_ok2", "l5_negfake_boom", "l6_negfake_ok", "l7_negfake_junk"],
                     ["--jobs", "1"], task=junk_wrap)
        B = scenario("B_subset", ["l3_negfake_empty", "l5_negfake_ok2", "l5_negfake_boom"], ["--jobs", "1", "--layers", "3,5"])
        D = scenario("D_parallel", [nm for nm in ALL_OK if nm != "l5_negfake_ok"] + ["l5_negfake_boom"],
                     ["--jobs", "2"], task=me._child_task)
        E = scenario("E_killed", ["l0_negfake_ok", "l5_negfake_kill"], ["--jobs", "2"], task=me._child_task)
    finally:
        gate.discover, gate.load_data, gate._run_layer_task = saved["discover"], saved["load_data"], saved["task"]
        gate._run_negatives_task, core.git_head = saved["neg"], saved["git"]
        if saved["pool"] is not None:
            gate._POOL_BROKEN = saved["pool"]
        for k in installed:
            sys.modules.pop(k, None)
        for k, v in env.items():
            if v is None:
                os.environ.pop(k, None)
            else:
                os.environ[k] = v

    def cell(r, n):
        return (r["sc"].get("cells") or {}).get(f"L{n}/_layer_error")

    def chk(c, check):
        return next((k for k in (c or {}).get("checks") or [] if k.get("check") == check), None)

    def blocked(c, check):
        k = chk(c, check)
        return bool(c and c.get("state") == "FAIL" and k and k.get("state") == "FAIL" and k.get("severity") == "BLOCK")

    a = []
    # C：干净 → 守门不误报
    c_cells = [cid for cid in C["sc"].get("cells", {}) if cid.endswith("/_layer_error")]
    a.append({"claim": "C 干净全量：CLEAR、rc=0、没有 _layer_error 格、摘要与 md 没有 ⚠ 行、没有 layer_errors 字段",
              "ok": (C["err"] is None and C["rc"] == 0 and C["sc"].get("verdict") == "CLEAR" and not c_cells
                     and "⚠" not in C["out"] and "⚠" not in C["md"] and "layer_errors" not in C["sc"])})
    # A：串行全量
    l5, l3, l7 = cell(A, 5), cell(A, 3), cell(A, 7)
    k5 = chk(l5, "layer_crashed") or {}
    d5 = str(k5.get("detail") or "")
    a.append({"claim": "A L5（两个模块里一个真抛 TypeError）→ L5/_layer_error FAIL、layer_crashed BLOCK、measured=TypeError、"
                       "detail 带异常行 + traceback 末 5 行 + 本层判据全部未评",
              "ok": (blocked(l5, "layer_crashed") and k5.get("measured") == "TypeError"
                     and "TypeError: sequence item 1: expected str instance, int found" in d5
                     and "traceback 末 5 行" in d5 and "本层判据全部未评" in d5 and "l5_negfake_boom" in d5)})
    a.append({"claim": "A L3 空跑（全量）→ layer_zero_findings BLOCK；L7 返回不可解析 → layer_crashed BLOCK",
              "ok": (blocked(l3, "layer_zero_findings") and (chk(l3, "layer_zero_findings") or {}).get("measured") == 0
                     and blocked(l7, "layer_crashed") and "不可解析" in str((chk(l7, "layer_crashed") or {}).get("detail")))})
    a.append({"claim": "A 其余层（0/1/2/4/6）没有 _layer_error；三格都在 blocking_cells；BLOCKED；rc=1；gate.main 没炸",
              "ok": (A["err"] is None and A["rc"] == 1 and A["sc"].get("verdict") == "BLOCKED"
                     and not any(cell(A, n) for n in (0, 1, 2, 4, 6))
                     and {"L3/_layer_error", "L5/_layer_error", "L7/_layer_error"} <= set(A["sc"].get("blocking_cells") or []))})
    blk, verdict_ln = _summary_block(A["out"])
    l5_line = [ln for ln in blk if ln.startswith("⚠ 第 5 层 功能达成 崩溃") and "TypeError" in ln and "0 条判据不是通过" in ln]
    a.append({"claim": "A 终端摘要顶部（结论行之前第一段）就是 ⚠ 报错，含 \"⚠ 第 5 层 功能达成 崩溃…TypeError…0 条判据不是通过\"",
              "ok": bool(blk and blk[0].startswith("⚠ 第") and l5_line and verdict_ln and verdict_ln.startswith("BLOCKED")
                         and all(ln.startswith("⚠") for ln in blk))})
    md_lines = [ln for ln in A["md"].splitlines() if ln.strip()]
    md_top = md_lines[1:4] if len(md_lines) > 3 else []
    a.append({"claim": "A 记分卡 md 标题下第一段就是 ⚠ 报错（含第 5 层 TypeError 那行）",
              "ok": bool(md_lines and md_lines[0].startswith("# Gate 记分卡") and md_top
                         and all(ln.startswith("> **⚠") for ln in md_top)
                         and any("第 5 层 功能达成 崩溃" in ln and "TypeError" in ln for ln in md_top))})
    # B：子集
    a.append({"claim": "B 子集 --layers 3,5：L3 空跑不加格只打印提示行；L5 崩溃照样加格；rc=1",
              "ok": (B["err"] is None and B["rc"] == 1 and cell(B, 3) is None and blocked(cell(B, 5), "layer_crashed")
                     and "⚠ 提示：第 3 层 静态装配" in B["out"] and "⚠ 提示：第 3 层" in B["md"])})
    # D：并行（真 spawn 层进程）
    kd = chk(cell(D, 5), "layer_crashed") or {}
    blk_d, _ = _summary_block(D["out"])
    a.append({"claim": "D 并行 --jobs 2：层进程里真抛 TypeError → L5/_layer_error BLOCK（TypeError）、其余层无格、BLOCKED、rc=1、摘要顶部有那一行",
              "ok": (D["err"] is None and D["rc"] == 1 and blocked(cell(D, 5), "layer_crashed") and kd.get("measured") == "TypeError"
                     and not any(cell(D, n) for n in (0, 1, 2, 3, 4, 6, 7))
                     and any(ln.startswith("⚠ 第 5 层 功能达成 崩溃") and "TypeError" in ln for ln in blk_d[:3]))})
    # E：层进程被杀
    ke = chk(cell(E, 5), "layer_crashed") or {}
    a.append({"claim": "E 并行：L5 层进程 SIGKILL → 进程池断 → L5/_layer_error BLOCK（BrokenProcessPool）、BLOCKED、rc=1、gate.main 正常返回、标记断池",
              "ok": (E["err"] is None and E["rc"] == 1 and E["sc"].get("verdict") == "BLOCKED"
                     and blocked(cell(E, 5), "layer_crashed") and ke.get("measured") == "BrokenProcessPool"
                     and E["pool"] is True and "进程池断过" in E["out"])})
    ok = all(x["ok"] for x in a)
    if ok:
        shutil.rmtree(root, ignore_errors=True)
    return {"name": NAME, "passed": ok, "expect": EXPECT,
            "got": "；".join(f"{x['claim'][:12]}={'ok' if x['ok'] else 'FAIL'}" for x in a)
                   + "".join(f" ｜{t} 炸了：{r['err']}" for t, r in res.items() if r.get("err")),
            "assertions": a,
            "detail": (f"C rc={C['rc']} {C['sc'].get('verdict')}；A rc={A['rc']} {A['sc'].get('verdict')} 守门格 "
                       f"{sorted(c for c in A['sc'].get('cells', {}) if c.endswith('/_layer_error'))}；"
                       f"B rc={B['rc']}；D rc={D['rc']}；E rc={E['rc']} pool_broken={E['pool']}")}


if __name__ == "__main__":
    r = run()
    print(json.dumps(r, ensure_ascii=False, indent=1))
    sys.exit(0 if r["passed"] else 1)
