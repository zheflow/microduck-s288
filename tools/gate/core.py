#!/usr/bin/env python3
"""Gate 内核：记分卡模型、输入 hash 绑定、过期标记、豁免、证据计数。

设计约束（来自 tools/gate/README.md 第 3 节元规则，改这里前先读那 10 条）：
  · 查最终导出的 STL 读回来的东西，不查内存网格
  · 每个 0 必须带证据数量；evidence_n == 0 判 NOT_RUN，不判 PASS
  · 未知 = 失败：拿不到数、布尔炸了、清单没写 → FAIL(BLOCK)，不许静默跳过
  · 不比历史：与上一版的差异只做 INFO 级变更告警
  · 每个格子记录自己输入的 hash，输入变了自动 STALE
"""
from __future__ import annotations
import hashlib, json, os, subprocess, sys, time
from dataclasses import dataclass, field, asdict
from pathlib import Path
from typing import Any, Callable, Iterable

ROOT = Path(__file__).resolve().parents[2]
DATA = Path(__file__).resolve().parent / "data"
STL_DIR = ROOT / "cad/duck_s288"
PLACED = STL_DIR / "placed"

# ── 状态与严重级 ────────────────────────────────────────────────────────────
PASS, FAIL, STALE, NOT_RUN, WAIVED = "PASS", "FAIL", "STALE", "NOT_RUN", "WAIVED"
# RETIRED（2026-09-13）：判据的**基准失去权威**、只保留为记录 —— 不阻断、不计入 INCOMPLETE、单独计数。
# 以前层里把这种情况硬编码成 STALE，而 STALE 在本内核里的语义是"输入变了没重跑"（继承逻辑可恢复），
# 结果 verdict 永远 INCOMPLETE、tally[STALE] 也说不清哪些格子真过期了。
RETIRED = "RETIRED"
STATES = (PASS, FAIL, STALE, NOT_RUN, WAIVED, RETIRED)
BLOCK, WARN, INFO = "BLOCK", "WARN", "INFO"
_RANK = {FAIL: 0, STALE: 1, NOT_RUN: 2, WAIVED: 3, RETIRED: 4, PASS: 5}


@dataclass
class Finding:
    """一条判据的结果。measured/criterion 必须都填 —— 只说 '通过' 不算证据。"""
    layer: int
    subject: str                  # 件号 / 关系 id / 螺丝组 id / 关节名
    check: str                    # 判据 slug，例如 hole_edges / engagement / horn_clocking
    state: str                    # PASS | FAIL | STALE | NOT_RUN | WAIVED | RETIRED
    severity: str = BLOCK         # BLOCK | WARN | INFO（只有 WARN 可豁免）
    measured: Any = None          # 实测值（数字或简短结构），不四舍五入
    criterion: str = ""           # 要求是什么（范围/阈值/来源）
    evidence_n: int = 0           # 查了几个：几对/几个角度/几条射线
    detail: str = ""
    provenance: str = ""          # 判据出处：data 文件:键 或 README 层号

    @property
    def cell(self) -> str:
        # 单位数层号，避免 "L02/L02"（第2层/L02件）读起来像重复
        return f"L{self.layer}/{self.subject}"


@dataclass
class LayerResult:
    layer: int
    name: str
    findings: list[Finding] = field(default_factory=list)
    inputs: list[str] = field(default_factory=list)   # 仓库相对路径，决定过期
    evidence: dict = field(default_factory=dict)      # 汇总证据量
    error: str | None = None                          # 层自身炸了 → 整层 FAIL
    covered: set = field(default_factory=set)         # 本层真正查到的数据条目 id（特征/装配步…）
                                                      # 义务清单按它算覆盖率；不填 = 覆盖不明 = 红

    def add(self, **kw) -> Finding:
        f = Finding(layer=self.layer, **kw)
        self.findings.append(f)
        return f

    def unknown(self, subject: str, check: str, why: str, provenance: str = "", severity: str = BLOCK):
        """元规则 4：拿不到数就是失败，不是跳过。"""
        return self.add(subject=subject, check=check, state=FAIL, severity=severity,
                        measured=None, criterion="必须有数（未知=失败）",
                        evidence_n=0, detail=why, provenance=provenance)


# ── 输入 hash / 过期 ────────────────────────────────────────────────────────
def placed_instance_map(parts, available):
    """打印件实例必须与独立 BOM 数量一致，且不允许两件占用同一导出实体。"""
    mapping, errors, owners = {}, [], {}
    for p in parts:
        pid, names, qty = p.get("id"), p.get("placed_instances"), p.get("qty")
        if (not isinstance(names, list) or not names or not all(isinstance(n, str) for n in names)
                or not isinstance(qty, int) or isinstance(qty, bool) or qty <= 0
                or len(names) != qty or len(set(names)) != len(names)):
            errors.append(f"{pid}: placed_instances={names!r} 与 qty={qty!r} 不符/缺失/重复")
            continue
        missing = set(names) - set(available)
        if missing:
            errors.append(f"{pid}: 未导出 {sorted(missing)}")
        if pid in mapping:
            errors.append(f"重复件号 {pid}")
        mapping[pid] = list(names)
        for n in names:
            if n in owners:
                errors.append(f"{pid} 与 {owners[n]} 重复使用 {n}")
            owners[n] = pid
    if not parts:
        errors.append("打印件清单为空")
    return ({} if errors else mapping), errors


def sha256_file(p: Path) -> str | None:
    try:
        return hashlib.sha256(p.read_bytes()).hexdigest()
    except OSError:
        return None


def hash_inputs(paths: Iterable[str]) -> dict[str, str | None]:
    return {p: sha256_file(ROOT / p) for p in sorted(set(paths))}


def git_head() -> str:
    try:
        out = subprocess.run(["git", "-C", str(ROOT), "rev-parse", "HEAD"],
                             capture_output=True, text=True, timeout=10)
        dirty = subprocess.run(["git", "-C", str(ROOT), "status", "--porcelain"],
                               capture_output=True, text=True, timeout=20).stdout.strip()
        return out.stdout.strip() + ("+dirty" if dirty else "")
    except Exception:
        return "unknown"


SOURCE_GLOBS = ["duckstructure/*.py", "tools/gate/data/*.yaml", "tools/gate/layers/*.py",
                "tools/gate/core.py", "tools/gate/gate.py",
                "cad/duck_s288/*.stl", "cad/duck_s288/*.json",
                "cad/duck_s288/hooks/*.stl"]      # hr46（2026-09-26）：件的附属实体（钩，build_fast.produce_extra）—— 钩一改 → 记分卡 STALE
# was_until_2026_09_26_hr46: SOURCE_GLOBS 末两项 "cad/duck_s288/*.stl", "cad/duck_s288/*.json"]（子目录 hooks/ 不绑）


def source_manifest(extra: list[str] | None = None) -> dict:
    """SOURCE_GLOBS 之外，各层通过 LayerResult.inputs 声明的文件也纳入 hash 绑定 ——
    否则像 S288 额定扭矩（判据基准，出处在 docs/ 里）这种东西改了不会有人知道。"""
    files: list[str] = []
    for g in SOURCE_GLOBS:
        files += [str(p.relative_to(ROOT)) for p in sorted(ROOT.glob(g))]
    mjcf = ROOT / "upstream/microduck_rl/src/mjlab_microduck/robot/microduck/robot_walk.xml"
    if mjcf.exists():
        files.append(str(mjcf.relative_to(ROOT)))
    files += list(extra or [])
    return hash_inputs(files)


# ── 豁免 ───────────────────────────────────────────────────────────────────
def load_waivers(data: dict) -> dict[str, dict]:
    """只有 WARN 可豁免；豁免绑定输入 hash，输入一变自动失效。"""
    out = {}
    for w in (data.get("waivers", {}).get("waivers") or []):
        if not isinstance(w, dict) or "cell_id" not in w:
            continue
        out[w["cell_id"]] = w
    return out


def apply_waivers(findings: list[Finding], waivers: dict[str, dict], manifest: dict) -> list[str]:
    """豁免 id 必须精确到**判据**：`L<层>/<件号>:<检查名>` 或 `<件号>.<层号>.<检查名>`。
    只写到格子（`L5/X`、`X.5`）的豁免一律拒绝 —— 那会把该格全部 WARN 判据一起放行（审计 F-核-4）。
    绑定：`bound_input_hashes` 是 {仓库相对路径: sha256}，与 source_manifest 的键同一口径。"""
    notes = []
    cell_only = {k for k in waivers if (":" not in k and k.count(".") < 2)}
    for k in sorted(cell_only):
        notes.append(f"豁免 {k!r} 只写到格子没写判据名 —— 拒绝（豁免必须精确到 L<层>/<件>:<check> 或 <件>.<层>.<check>）")
    for f in findings:
        alt = f"{f.subject}.{f.layer}.{f.check}"
        w = waivers.get(f"{f.cell}:{f.check}") or waivers.get(alt)
        if not w or f.state != FAIL:
            continue
        if f.severity != WARN:
            notes.append(f"{f.cell}:{f.check} 声明了豁免但严重级是 {f.severity}，不可豁免 —— 忽略该豁免")
            continue
        # waivers.yaml:29 的 schema 写的是 bound_input_hashes（复数），旧代码只读单数
        # → 按文档填的豁免拿到空 dict，等于无条件生效。两种都认，且**必须非空**。
        bound = w.get("bound_input_hashes") or w.get("bound_input_hash") or {}
        if not bound:
            notes.append(f"{f.cell}:{f.check} 的豁免没有绑定任何输入 hash —— 拒绝豁免"
                         f"（元规则 9：豁免必须绑定输入，输入一变自动失效）")
            continue
        changed = [k for k, v in bound.items() if manifest.get(k) != v]
        if changed:
            notes.append(f"{f.cell}:{f.check} 的豁免已失效（输入变了：{', '.join(changed[:3])}）")
            continue
        f.state = WAIVED
        f.detail = (f.detail + f" | 豁免：{w.get('reason','')}（{w.get('issued_by','?')} {w.get('date','?')}）").strip()
    return notes


# ── 数据加载 ───────────────────────────────────────────────────────────────
def load_data() -> dict:
    import yaml
    out = {}
    for p in sorted(DATA.glob("*.yaml")):
        out[p.stem] = yaml.safe_load(p.read_text(encoding="utf-8")) or {}
    return out


def num(field: Any, default=None):
    """取 {v:…, src:…} 形式的数值；返回 (值, 来源)。值为 None 表示未知。"""
    if isinstance(field, dict) and "v" in field:
        v = field["v"]
        # 数据里存在 mass_g.v = "PLA≈49.8 / TPU≈27.1" 这种字符串，绝不能当数值交出去
        return (v if isinstance(v, (int, float)) and not isinstance(v, bool) else None), field.get("src", "unknown")
    if isinstance(field, (int, float)) and not isinstance(field, bool):
        return field, "unknown"
    return default, "unknown"


# ── 记分卡 ─────────────────────────────────────────────────────────────────
# ── 必检义务清单（元规则 4 的框架级落实）────────────────────────────────────
# 为什么需要它：记分卡原来只为"已发出的 finding"建格子，于是**没生成的检查义务不会失败** ——
# 一个什么都不做的空层在矩阵上根本不存在（是 `·` 不是 ❌），极端情况下把所有层删光会判 CLEAR。
# 义务清单独立于各层是否发 finding，先把"应该查什么"枚举出来，再看谁没被覆盖。
def _ids(seq, key="id"):
    return [x[key] for x in (seq or []) if isinstance(x, dict) and x.get(key)]


def obligations(data: dict) -> dict[str, dict]:
    """{cell_id: {layer, subject, why}} —— 与本轮跑没跑、层实现没实现无关。"""
    P = _ids(data.get("parts", {}).get("parts"))
    # 禁入体的义务按 keepouts.yaml:checked_in_layer 派层（扫掠/运动类禁入体归第 4/6 层，零位姿静态交集对它们没有意义；
    # 以前写死在 l3_static._DEFERRED，第 3 层永远 NOT_RUN → 永久 INCOMPLETE，审计 L3 MINOR）。没写的默认第 3 层。
    KO_ALL = [k for k in (data.get("keepouts", {}).get("keepouts") or []) if isinstance(k, dict) and k.get("id")]
    KO = [k["id"] for k in KO_ALL if k.get("checked_in_layer") in (None, 3)]
    KO_BY_LAYER: dict[int, list] = {}
    for k in KO_ALL:
        lay = k.get("checked_in_layer")
        if isinstance(lay, int) and lay != 3:
            KO_BY_LAYER.setdefault(lay, []).append(k["id"])
    CO = _ids(data.get("components", {}).get("components"))
    FA = _ids(data.get("fasteners", {}).get("fasteners"))
    RE = _ids(data.get("relations", {}).get("relations"))
    JT = _ids((data.get("frozen", {}).get("joint_axes") or {}).get("joints")
              if isinstance(data.get("frozen", {}).get("joint_axes"), dict)
              else data.get("frozen", {}).get("joint_axes"), "name") \
         or _ids(data.get("frozen", {}).get("joint_axes"))
    plan = {0: [("件", P)], 1: [("件", P)], 2: [("件", P)],
            3: [("件", P), ("禁入体", KO), ("元件", CO)],
            4: [("螺丝组", FA)], 5: [("螺丝组", FA), ("功能关系", RE)],
            6: [("件", P), ("关节", JT)], 7: [("件", P)]}
    for lay, ids in KO_BY_LAYER.items():
        plan.setdefault(lay, []).append(("禁入体（checked_in_layer）", ids))
    out: dict[str, dict] = {}
    for lay, groups in plan.items():
        out[f"L{lay}/_layer"] = {"layer": lay, "subject": "_layer",
                                 "why": f"第 {lay} 层（{LAYER_NAMES[lay]}）必须实现并运行"}
        for kind, ids in groups:
            for i in ids:
                out[f"L{lay}/{i}"] = {"layer": lay, "subject": i,
                                      "why": f"{kind} {i} 必须被第 {lay} 层（{LAYER_NAMES[lay]}）检查"}
    return out


# 逐条声明必须被覆盖的数据条目（id 不作为 subject 出现、只能靠层自报 covered 的那些）
COVERAGE_ITEMS = {2: ("features", "features", "特征"),
                  4: ("assembly", "assembly_order", "装配步")}


LAYER_NAMES = {
    0: "网格与坐标", 1: "可打印", 2: "特征存在", 3: "静态装配",
    4: "装得进去", 5: "功能达成", 6: "运动", 7: "质量与力",
}


def build_scorecard(results: list[LayerResult], manifest: dict, waiver_notes: list[str],
                    started: float, prev: dict | None = None, layers_run: list[int] | None = None,
                    data: dict | None = None, only: set | None = None) -> dict:
    """only：--parts 限定的件号集合（None = 全部）。子集跑时，本轮跑过的层里**不在范围内**的格子
    从上一版按 inputs_hash 继承（变了 → STALE），否则义务循环会把它们整片写成 obligation_unmet 红，
    "改一个件只重跑受影响子集"（元规则 7）就不成立（审计 F-核-6 / Codex01 FG02）。"""
    cells: dict[str, dict] = {}
    # 每层的隐式输入：12 个数据文件 + 该层自己的模块 + 内核 + **建模源码**。层只需声明它额外读了哪些
    # 文件（STL、placed、docs 里的判据出处等）；判据数据和代码变了必须让格子变灰。
    # duckstructure/*.py 进隐式输入（审计 F-核-2）：L3 读 lib.conn_cut / s288.S、L4 读 lib.B/sfw、
    # L6 读 lib.B / kin、L7 读 kin —— 以前只有 L0 声明它，改 s288.py 之后其它层的旧绿会按"输入未变"沿用。
    implicit = [str(p.relative_to(ROOT)) for p in sorted(DATA.glob("*.yaml"))] + \
               ["tools/gate/core.py", "tools/gate/gate.py"] + \
               [str(p.relative_to(ROOT)) for p in sorted((ROOT / "duckstructure").glob("*.py"))]
    layer_inputs = {}
    for r in results:
        mod = [str(p.relative_to(ROOT)) for p in sorted((ROOT / "tools/gate/layers").glob(f"l{r.layer}_*.py"))]
        layer_inputs[r.layer] = sorted(set(list(r.inputs) + implicit + mod))
    for r in results:
        for f in r.findings:
            # 元规则 2：没有证据的"通过"不是通过（逐条判，不按格子汇总，否则一条带证据的
            # INFO 备注能把整格没有实际判据的格子托成绿）
            if f.state == PASS and f.evidence_n == 0:
                f.state = NOT_RUN
                f.detail = (f.detail + " | evidence_n=0：没有实际检查任何东西").strip(" |")
            c = cells.setdefault(f.cell, {"layer": f.layer, "subject": f.subject,
                                          "state": PASS, "checks": [], "evidence_n": 0})
            c["checks"].append({k: v for k, v in asdict(f).items() if k not in ("layer", "subject")})
            c["evidence_n"] += f.evidence_n
            if _RANK[f.state] < _RANK[c["state"]]:
                c["state"] = f.state
        # 层自身炸了
        if r.error:
            cells[f"L{r.layer}/_layer"] = {"layer": r.layer, "subject": "_layer", "state": FAIL,
                                               "evidence_n": 0,
                                               "checks": [{"check": "layer_execution", "state": FAIL,
                                                           "severity": BLOCK, "detail": r.error}]}
    ran = {r.layer for r in results}

    # ── 本轮没跑的层：先从上一版继承，输入变了就 STALE —— 不得沿用旧绿
    # 顺序是判据性的：继承必须排在下面的"必检义务"之前。义务循环见到格子已存在就跳过，
    # 反过来的话，本轮没跑的层会先被义务循环整片写成 obligation_unmet 红，继承永远够不着
    # （实测 layers_run=[0,1,7] 那次有 193 格是这么来的），"跑部分层"和"跑全量"的红数没法比。
    inherited: set[str] = set()
    if prev and layers_run is not None:
        recomputed = set(layers_run) | ran      # 本轮跑过／打算跑的层一律重算，不许继承……
        for cid, old in (prev.get("cells") or {}).items():
            if cid in cells:
                continue
            if old.get("layer") in recomputed:
                # ……除非是 --parts 子集跑：范围外的件在本轮根本没被查，按 hash 继承（元规则 7）。
                # 全层跑（only=None）不继承 —— 该层没为它发判据就是覆盖缺口，义务循环会把它写红。
                if only is None or old.get("subject") in only:
                    continue
            carried = dict(old)
            # 直接重算该格记录的那几个文件；不能用本轮 manifest —— 本轮没声明的路径
            # manifest.get() 返回 None，会把没变的格子误判成过期
            changed = [p for p, h in (old.get("inputs_hash") or {}).items() if sha256_file(ROOT / p) != h]
            # 只留最新一条继承说明，并且**新建 list** —— dict(old) 是浅拷贝，直接 append
            # 会改到 prev 自己的 notes，连跑几轮同一个格子会攒出一串重复的"未重跑…"
            keep = [n for n in (old.get("notes") or []) if not str(n).startswith("未重跑")]
            carried["notes"] = keep + [
                f"未重跑，且输入已变（{len(changed)} 个文件）：{', '.join(changed[:3])}" if changed
                else "未重跑，输入未变，沿用上一版结果"]
            if changed:
                carried["state"] = STALE
            cells[cid] = carried
            inherited.add(cid)

    # ── 必检义务：继承之后仍然没被任何 finding／上一版结果覆盖的义务，在这里变成红格，
    # 而不是从记分卡上消失
    if data:
        # 禁入体 checked_in_layer=L 且给了 covered_by_subjects 的：L 层没有以 KO 号为 subject 的格子，
        # 它的判据分散在那些步/工位格子里。这里把它们汇总成 L{L}/KOxx 一格：状态 = 所列格子里最差的（缺一格 = NOT_RUN），
        # evidence_n = 所列格子的证据和。不是新判据，是把义务对到真正查它的格子上（keepouts.yaml:KOxx.covered_by_subjects）。
        for k in (data.get("keepouts", {}).get("keepouts") or []):
            lay, subs = k.get("checked_in_layer"), k.get("covered_by_subjects")
            if not (isinstance(k, dict) and isinstance(lay, int) and lay != 3 and subs):
                continue
            cid = f"L{lay}/{k['id']}"
            if cid in cells or lay not in ran:
                continue
            rows = [(sname, cells.get(f"L{lay}/{sname}")) for sname in subs]
            missing = [sname for sname, c in rows if c is None]
            states = [c["state"] for _, c in rows if c is not None]
            worst = NOT_RUN if missing else min(states, key=lambda st: _RANK[st]) if states else NOT_RUN
            ev = sum(int(c.get("evidence_n") or 0) for _, c in rows if c is not None)
            cells[cid] = {"layer": lay, "subject": k["id"], "state": worst, "evidence_n": ev,
                          "checks": [{"check": "covered_by_cells", "state": worst, "severity": BLOCK,
                                      "measured": None if missing else len(states),
                                      "criterion": f"keepouts.yaml:{k['id']}.covered_by_subjects 所列的 L{lay} 格子全部存在且状态取最差",
                                      "evidence_n": ev,
                                      "detail": ("缺格：" + ", ".join(missing) + "；" if missing else "")
                                                + "；".join(f"{sname}={c['state']}" for sname, c in rows if c is not None)
                                                + (f"（{k.get('covered_by_note')}）" if k.get("covered_by_note") else ""),
                                      "provenance": f"keepouts.yaml:{k['id']}.checked_in_layer/covered_by_subjects + core.py"}]}
        ok_layers = {r.layer for r in results if not r.error and r.findings}
        for cid, ob in obligations(data).items():
            if cid in cells:
                continue
            # 层跑过且发出了判据 → _layer 这条义务算已满足（真炸了上面已经建过 _layer 红格）
            if ob["subject"] == "_layer" and ob["layer"] in ok_layers:
                cells[cid] = {"layer": ob["layer"], "subject": "_layer", "state": PASS,
                              "evidence_n": sum(len(r.findings) for r in results if r.layer == ob["layer"]),
                              "checks": [{"check": "layer_implemented_and_ran", "state": PASS,
                                          "severity": BLOCK, "measured": None, "criterion": ob["why"],
                                          "evidence_n": sum(len(r.findings) for r in results
                                                            if r.layer == ob["layer"]),
                                          "detail": "", "provenance": "core.py:obligations"}]}
                continue
            lay = ob["layer"]
            why = ("该层本轮没跑" if lay not in ran else
                   "该层跑了，但没有为这个对象发出任何判据 —— 覆盖缺口，不是通过")
            cells[cid] = {"layer": lay, "subject": ob["subject"], "state": FAIL, "evidence_n": 0,
                          "checks": [{"check": "obligation_unmet", "state": FAIL, "severity": BLOCK,
                                      "measured": None, "criterion": ob["why"], "evidence_n": 0,
                                      "detail": why,
                                      "provenance": "core.py:obligations（元规则 4 的框架级落实）"}]}
        # 只能靠层自报 covered 的条目类型（特征、装配步）
        cov = {r.layer: r.covered for r in results}
        for lay, (fname, key, kind) in COVERAGE_ITEMS.items():
            rows_all = [x for x in (data.get(fname, {}).get(key) or []) if isinstance(x, dict)]
            want = {x["id"] for x in rows_all if x.get("id")}
            cid = f"L{lay}/_coverage"
            n_noid = sum(1 for x in rows_all if not x.get("id"))
            if not rows_all or n_noid:
                # 元规则 4 的框架级落实不能自己静默跳过（审计 F-核-1）：清单为空或条目缺 id，
                # 覆盖率就算不出来 —— 那是红，不是"没有义务"。
                cells[cid] = {"layer": lay, "subject": "_coverage", "state": FAIL, "evidence_n": 0,
                              "checks": [{"check": f"{kind}覆盖率", "state": FAIL, "severity": BLOCK,
                                          "measured": None,
                                          "criterion": f"{fname}.yaml:{key} 每条{kind}必须有 id，"
                                                       f"否则第 {lay} 层的覆盖率无法计算",
                                          "evidence_n": 0,
                                          "detail": (f"{fname}.yaml:{key} 为空" if not rows_all else
                                                     f"{n_noid}/{len(rows_all)} 条{kind}没有 id —— "
                                                     f"覆盖率义务无法落到条目上（未知=失败，不静默跳过）"),
                                          "provenance": "core.py:COVERAGE_ITEMS"}]}
                continue
            if lay not in ran:
                # 该层本轮没跑：cov 里没有它的 covered，算出来必然是 0/N 假红。
                # 已经继承到的格子沿用（该 STALE 的上面已经判过）；继承不到才写红。
                if cid in cells:
                    continue
                cells[cid] = {"layer": lay, "subject": "_coverage", "state": FAIL, "evidence_n": 0,
                              "checks": [{"check": f"{kind}覆盖率", "state": FAIL, "severity": BLOCK,
                                          "measured": None,
                                          "criterion": f"{fname}.yaml 声明的 {len(want)} 条{kind}"
                                                       f"必须逐条被第 {lay} 层检查",
                                          "evidence_n": 0,
                                          "detail": "该层本轮没跑，上一版也没有这个格子 —— 覆盖率未知",
                                          "provenance": "core.py:COVERAGE_ITEMS"}]}
                continue
            got = cov.get(lay) or set()
            miss = sorted(want - got)
            cells[cid] = {"layer": lay, "subject": "_coverage", "state": PASS if not miss else FAIL,
                          "evidence_n": len(got & want),
                          "checks": [{"check": f"{kind}覆盖率", "state": PASS if not miss else FAIL,
                                      "severity": BLOCK, "measured": f"{len(got & want)}/{len(want)}",
                                      "criterion": f"{fname}.yaml 声明的 {len(want)} 条{kind}必须逐条被第 {lay} 层检查",
                                      "evidence_n": len(got & want),
                                      "detail": ("全覆盖" if not miss else
                                                 f"{len(miss)} 条没被覆盖（层未自报 covered 也算没覆盖）："
                                                 + ", ".join(miss[:6]) + ("…" if len(miss) > 6 else "")),
                                      "provenance": "core.py:COVERAGE_ITEMS"}]}

    # 空格子（一条判据都没有）同样不算通过（义务／覆盖率格子建完再兜一次底）
    for cid, c in cells.items():
        if c["state"] == PASS and not c["checks"]:
            c["state"] = NOT_RUN
    # 每格记下自己输入的 hash 和"本层真跑出来的状态"（元规则 7）。
    # state_computed 必须单独存：否则一次误判 STALE 之后，继承时无法恢复，格子会永远灰着。
    # 继承来的格子跳过：它自带上一版的 inputs_hash / state_computed，重新盖一遍会把
    # "上一版真跑出来的状态"覆写成 STALE，正是上面那条注释要防的事。
    for cid, c in cells.items():
        if cid in inherited:
            continue
        ins = layer_inputs.get(c["layer"], [])
        c["inputs_hash"] = {p: (manifest.get(p) or sha256_file(ROOT / p)) for p in ins}
        c["state_computed"] = c["state"]

    tally = {s: sum(1 for c in cells.values() if c["state"] == s) for s in STATES}
    # 必须是"真的失败了的 BLOCK 项"，不是"格子红了且碰巧有个通过的 BLOCK 项"
    blocks = [cid for cid, c in cells.items()
              if any(k.get("severity") == BLOCK and k.get("state") in (FAIL, STALE, NOT_RUN)
                     for k in c["checks"])]
    return {
        "version": 2,
        "partial": bool(only) or (layers_run is not None and sorted(set(layers_run)) != list(range(8))),
        "only": sorted(only) if only else None,
        # RETIRED 不计入 INCOMPLETE：它是"这条判据的基准没了、留作记录"，不是"没跑完"
        "verdict": "BLOCKED" if blocks else ("INCOMPLETE" if tally[NOT_RUN] or tally[STALE] else "CLEAR"),
        "git": git_head(),
        "elapsed_s": round(time.time() - started, 1),
        "layers_run": sorted({r.layer for r in results}),
        "layers_carried": sorted({c["layer"] for c in cells.values()} - {r.layer for r in results}),
        "tally": tally,
        "blocking_cells": sorted(blocks),
        "waiver_notes": waiver_notes,
        "evidence": {f"L{r.layer:02d}": r.evidence for r in results},
        "cells": dict(sorted(cells.items())),
        "source_manifest": manifest,
    }


def render_markdown(sc: dict, data: dict) -> str:
    parts = [p["id"] for p in (data.get("parts", {}).get("parts") or [])]
    sym = {PASS: "✅", FAIL: "❌", STALE: "🕓", NOT_RUN: "⬜", WAIVED: "🟡", RETIRED: "🪦"}
    L = [f"# Gate 记分卡 —— {sc['verdict']}" + ("（**子集跑，不是放行依据**）" if sc.get("partial") else ""), "",
         f"`{sc['git']}` · {sc['elapsed_s']}s · 层 {sc['layers_run']}"
         + (f" · 件 {sc['only']}" if sc.get("only") else ""), "",
         "| 计 | " + " | ".join(f"{sym[s]} {s}" for s in STATES) + " |",
         "|---|" + "---|" * len(STATES),
         "| | " + " | ".join(str(sc["tally"].get(s, 0)) for s in STATES) + " |", ""]
    if parts:
        L += ["## 17 件 × 8 层", "", "| 件 | " + " | ".join(f"L{i}" for i in range(8)) + " |",
              "|---|" + "---|" * 8]
        for p in parts:
            row = []
            for i in range(8):
                c = sc["cells"].get(f"L{i}/{p}")
                row.append(sym.get(c["state"], "·") if c else "·")
            L.append(f"| {p} | " + " | ".join(row) + " |")
        L.append("")
    other = {k: v for k, v in sc["cells"].items() if v["subject"] not in parts}
    if other:
        L += ["## 非件级格子（按层汇总）", "",
              "| 层 | 格子数 | ✅ | ❌ | ⬜ | 红格 |", "|---|---:|---:|---:|---|"]
        for n in sorted({c["layer"] for c in other.values()}):
            g = {k: v for k, v in other.items() if v["layer"] == n}
            bad = sorted(k for k, v in g.items() if v["state"] == FAIL)
            L.append(f"| L{n} {LAYER_NAMES.get(n,'')} | {len(g)} | "
                     f"{sum(1 for v in g.values() if v['state']==PASS)} | {len(bad)} | "
                     f"{sum(1 for v in g.values() if v['state']==NOT_RUN)} | "
                     + (", ".join(f"`{b.split('/',1)[1]}`" for b in bad[:6])
                        + (f" …+{len(bad)-6}" if len(bad) > 6 else "") if bad else "—") + " |")
        L.append("")
    fails = [(cid, k) for cid, c in sc["cells"].items() for k in c["checks"]
             if k.get("state") == FAIL]
    if fails:
        L += [f"## 红格明细（{len(fails)} 条）", "", "| 格子 | 判据 | 级 | 实测 | 要求 | 说明 |", "|---|---|---|---|---|---|"]
        for cid, k in sorted(fails, key=lambda x: (x[1].get("severity") != BLOCK, x[0])):
            m = k.get("measured"); m = "—" if m is None else (f"{m:.4g}" if isinstance(m, float) else str(m)[:40])
            L.append(f"| `{cid}` | {k.get('check','')} | {k.get('severity','')} | {m} | "
                     f"{str(k.get('criterion',''))[:60]} | {str(k.get('detail',''))[:80]} |")
    for n in sc.get("waiver_notes", []):
        L.append(f"\n> ⚠️ {n}")
    return "\n".join(L) + "\n"
