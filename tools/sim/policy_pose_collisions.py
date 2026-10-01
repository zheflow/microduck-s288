#!/usr/bin/env python3
"""按**原版策略真实走到的姿态**（逐控制步关节元组）在我们的件上查碰撞 —— 不是轴对齐包围盒的角落。

    ./.venv/bin/python tools/sim/policy_envelope.py --seconds 20 --dump-steps tools/gate/out/policy_steps_2026-09-16/policy_steps.npz
    ./.venv/bin/python tools/sim/policy_pose_collisions.py [--grid 2.5] [--limit N]

README A2 / Gate L6 `_capability/box_collisions` 的已知上界问题：包络 json 只有每个关节各自的 min/max，L6 只能判"盒子"，
neck_pitch=+60 与 head_pitch=−50 各自在包络内、策略是否**同时**到过查不到。本脚本：
  · 只取 frozen.yaml:capability_envelope.modes_in_scope 的模式、prefall 段（跌倒后挥舞的角度不算能力）
  · 14 关节元组按 --grid 度分格去重，每格取**第一条真实姿态**（不取格心，不造姿态）
  · 每个姿态用 tools/gate/layers/l6_motion._Scene.evaluate（placed/*.stl 真网格 manifold 布尔，AABB 预筛）
  · 逐姿态追加前验证 run.json 的生成输入；旧目录缺指纹或输入变化时拒绝续跑，不替旧记录盖新指纹
  · --limit 只写 summary.partial.json；完成完整 NPZ 去重义务后才写 complete=true 的 summary.json
结论口径：碰撞需要消除或有独立的接触证据；该采样不证明其余组合姿态和连续过渡安全。
"""
from __future__ import annotations
import argparse, json, sys, time
from pathlib import Path

HERE = Path(__file__).resolve()
ROOT = HERE.parents[2]
for p in (ROOT / "tools/gate", ROOT / "tools/gate/layers", ROOT, ROOT / "tools/cad", ROOT / "tools/sim"):
    if str(p) not in sys.path:
        sys.path.insert(0, str(p))
from core import load_data          # noqa: E402
from gate import Ctx                # noqa: E402
import l6_motion as L6              # noqa: E402
import policy_pose_evidence as evidence  # noqa: E402

STEPS = ROOT / "tools/gate/out/policy_steps_2026-09-16/policy_steps.npz"
OUT = ROOT / "tools/gate/out/policy_steps_2026-09-16"
MANIFEST = ROOT / "tools/sim/cad_geometry_manifest.json"


def _write_json(path, payload):
    temporary = path.with_name(path.name + ".tmp")
    temporary.write_text(json.dumps(payload, ensure_ascii=False, indent=1), encoding="utf-8")
    temporary.replace(path)


# ── 并行工作进程（2026-09-18）：spawn 后各自建一份场景，评估一段姿态；只返回可 pickle 的纯数据。
_W = {}


def _default_jobs():
    import os
    v = os.environ.get("POSE_JOBS")
    if v and v.isdigit():
        return max(1, int(v))
    return max(1, min(6, (os.cpu_count() or 2) // 2))


def _jaw_patch(sc, jaw_deg, jaw_parts):
    """hr39（2026-09-23）：嘴角度覆盖。上游 MJCF 里下颚不是关节，J01（placed 名 jaw）挂在 jaw_soft 上，
    所以在 _Scene.deltas 外面再右乘一次嘴轴旋转 duckstructure.jaw.jaw_R(jaw_deg)（零位世界系、绕 JAW_AXIS_P/D，正角 = 张嘴）。
    嘴件与同刚体（jaw_soft）上其它件之间因此有了相对运动 → 返回要补查的件对。jaw_deg = 0 时什么都不做（与改前逐字节同行为）。"""
    if abs(float(jaw_deg)) < 1e-12:
        return []
    from duckstructure import jaw as JAW
    idx = [i for i, n in enumerate(sc.names) if n in set(jaw_parts)]
    if not idx:
        raise ValueError(f"--jaw-deg: placed 里找不到嘴件 {jaw_parts}")
    Rj = JAW.jaw_R(float(jaw_deg))
    base = sc.deltas

    def deltas(pose_deg, _base=base):
        M = _base(pose_deg)
        for i in idx:
            M[i] = M[i] @ Rj
        return M
    sc.deltas = deltas
    bodies = {sc.bodies[i] for i in idx}
    return sorted({(min(i, k), max(i, k)) for i in idx for k in range(len(sc.names))
                   if k not in idx and sc.bodies[k] in bodies})


def _fingerprint(a, scope, tol):
    fp = evidence.generation_fingerprint(L6.PLACED, a.steps, a.bfp, scope, a.grid, tol)
    if abs(float(a.jaw_deg)) > 1e-12:                    # hr39：嘴角度非 0 的结果与 0° 的分开，不能续跑混在一起
        fp["jaw_deg"] = float(a.jaw_deg); fp["jaw_parts"] = sorted(a.jaw_parts.split(","))
    return fp


def default_jaw_parts() -> str:
    """hr41 落盘 2026-09-25（主设计 Lane C）：--jaw-parts 的默认值 = duckstructure.jaw.JAW_MOVERS（逗号连接），单一数据源，不在这里抄件名。"""
    from duckstructure.jaw import JAW_MOVERS
    return ",".join(JAW_MOVERS)


def _worker_init(placed_dir, bfp_path, tol, names, jaw_deg=0.0, jaw_parts=("jaw",)):   # jaw_parts 由 main 显式传（initargs），这个缺省不生效
    L6.PLACED = Path(placed_dir)
    data = load_data(); ctx = Ctx(data)
    bfp = json.loads(Path(bfp_path).read_text(encoding="utf-8"))["body_for_part"]
    sc = L6._Scene(ctx, bfp); ctx._mesh.clear()
    extra = _jaw_patch(sc, jaw_deg, jaw_parts)
    pairs = sorted(set(sc.moving_pairs(names)) | set(extra))
    _W.update(sc=sc, pairs=pairs, tol=tol)


def _worker_eval(chunk):
    sc, pairs, tol = _W["sc"], _W["pairs"], _W["tol"]
    out = []
    for key, pose in chunk:
        o0 = getattr(sc, "obb_bool", 0)
        hits, _, nb = sc.evaluate(pose, pairs, tol)
        out.append((key, pose, [(x, y, float(v)) for x, y, v in hits], nb, getattr(sc, "obb_bool", 0) - o0))   # hr42：+ 本姿态被凸包分离证明跳过的布尔数
    import check_cache as CC
    CC.worker_flush()                                    # hr42：工作进程每段末尾写原语缓存分片 + 统计（进程池退出不跑 atexit）
    return out


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--grid", type=float, default=2.5, help="去重网格（度），默认 2.5 = L6 扫描步长")
    ap.add_argument("--limit", type=int, default=0, help="只跑前 N 个姿态（计时用）")
    ap.add_argument("--steps", type=Path, default=STEPS)
    ap.add_argument("--placed", type=Path, default=None, help="换一套 placed 目录（如 orig_scene_export.py 导出的原版件），默认 cad/duck_s288/placed")
    ap.add_argument("--bfp", type=Path, default=MANIFEST, help="body_for_part 映射 json（默认 tools/sim/cad_geometry_manifest.json）")
    ap.add_argument("--out", type=Path, default=OUT, help="输出目录（poses.jsonl / summary.json）")
    ap.add_argument("--jobs", type=int, default=None,
                    help="并行进程数（2026-09-18）：每个进程各自建场景、评估一段姿态，主进程按原顺序写 poses.jsonl；默认 POSE_JOBS 或 min(6, cpu/2)；1 = 串行")
    ap.add_argument("--jaw-deg", type=float, default=0.0,
                    help="hr39：嘴张开角（度，0..30，正 = 张嘴）。J01 绕 duckstructure.jaw 的嘴轴转这个角再查；非 0 时指纹里带上它，须用新输出目录")
    # hr41 落盘 2026-09-25（主设计 Lane C，协调员 07:15 #10）：默认改成 duckstructure/jaw.py:JAW_MOVERS（jaw, jaw_adapter, jaw_journal）——
    #   hr41 起随嘴转的是 J01 + J02（拧在法兰上）+ J03（轴颈盘），只转 J01 会漏查 J02/J03 与同刚体件的相对运动。旧默认 "jaw"
    ap.add_argument("--jaw-parts", default=default_jaw_parts(),
                    help="随嘴转的 placed 件名（逗号分隔），默认 = duckstructure.jaw.JAW_MOVERS（J01 + J02 + J03）")
    ap.add_argument("--no-check-cache", action="store_true", help="hr42：不读不写检查原语缓存，全量实跑（交付/复审前的全链用）")
    a = ap.parse_args()
    import os, tempfile
    if a.no_check_cache:
        os.environ["DUCK_CHECK_CACHE"] = "0"
    os.environ["DUCK_CHECK_CACHE_STATS_DIR"] = tempfile.mkdtemp(prefix="hr42_ccstats_")   # 工作进程把原语缓存统计写这里
    import check_cache as CC
    if a.limit < 0:
        ap.error("--limit must be nonnegative")
    if a.placed is not None:
        L6.PLACED = a.placed                                     # _Scene 按模块常量找网格

    data = load_data(); ctx = Ctx(data)
    cap = data["frozen"]["capability_envelope"]
    scope = list(cap["modes_in_scope"])
    expected = evidence.expected_poses(a.steps, scope, a.grid)
    names = expected["joint_names"]
    todo = list(expected["rows"].items())
    print(f"in-scope prefall 样本 {expected['raw_count']}（{len(scope)} 个 mode）")
    print(f"按 {a.grid}° 网格去重 → {len(todo)} 个真实姿态")
    bfp = json.loads(a.bfp.read_text(encoding="utf-8"))["body_for_part"]
    tol = float(data["tolerances"]["feature_check_tolerances"]["static_intersection_mm3"]["max"])
    fingerprint = _fingerprint(a, scope, tol)
    out_dir = a.out
    # Crucially, reject stale rows before constructing expensive solids, creating
    # files, or replacing a summary. Old evidence remains available for audit.
    done = evidence.validate_resume(out_dir, fingerprint, expected)
    from duckstructure.lib import B
    stems = {p.stem for p in L6.PLACED.glob("*.stl")}
    if (not isinstance(bfp, dict) or set(bfp) != stems
            or any(not isinstance(body, str) or body not in B for body in bfp.values())):
        raise ValueError("scene mapping must cover every placed STL exactly once with a valid kinematic body")
    out_dir.mkdir(parents=True, exist_ok=True)
    run_path = out_dir / "run.json"
    if not run_path.exists():
        _write_json(run_path, dict(fingerprint=fingerprint, expected_poses=len(todo),
                                  steps_file=str(a.steps.resolve()), mapping_file=str(a.bfp.resolve()),
                                  placed_dir=str(L6.PLACED.resolve()), modes_in_scope=scope, grid_deg=a.grid))
    jl = out_dir / "poses.jsonl"
    print(f"已有 {len(done)} 条，续跑")
    if a.limit:
        todo = todo[:a.limit]
    t0 = time.time(); sc = L6._Scene(ctx, bfp); ctx._mesh.clear()
    jaw_parts = tuple(x for x in a.jaw_parts.split(",") if x)
    extra = _jaw_patch(sc, a.jaw_deg, jaw_parts)
    pairs = sorted(set(sc.moving_pairs(names)) | set(extra))
    if extra:
        print(f"嘴角度 {a.jaw_deg}°：J01 绕嘴轴转，补查同刚体件对 {len(extra)} 个")
    if not pairs:
        raise ValueError("scene has no moving solid pairs; cannot establish collision clearance")
    print(f"场景 {len(sc.names)} 实体，{len(pairs)} 个可动件对，建场景 {time.time()-t0:.1f} s，tol {tol} mm³")
    t0 = time.time(); n_new = 0
    pending = [(key, pose) for key, pose in todo if key not in done]
    jobs = a.jobs if a.jobs else _default_jobs()
    def _row(key, pose, hits, nb):
        mode, case, i = key
        return json.dumps({"mode": mode, "case": case, "step": i, "pose_deg": {k: round(v, 2) for k, v in pose.items()},
                           "pose_deg_exact": pose, "hits": [[x, y, float(v)] for x, y, v in hits],
                           "booleans": nb}, ensure_ascii=False) + "\n"
    pref_rows = []                                       # hr42：逐姿态 [mode, case, step, 布尔实做数, 凸包分离证明跳过数]（旁注 hr42_prefilter.json）
    with jl.open("a", encoding="utf-8") as f:
        if jobs <= 1 or len(pending) < 2 * jobs:
            for key, pose in pending:
                o0 = getattr(sc, "obb_bool", 0)
                hits, _, nb = sc.evaluate(pose, pairs, tol)
                pref_rows.append([*key, nb, getattr(sc, "obb_bool", 0) - o0])
                f.write(_row(key, pose, hits, nb)); f.flush(); n_new += 1
                if n_new % 50 == 0:
                    el = time.time() - t0
                    print(f"  {n_new}/{len(pending)}  {el:.0f} s，{el/n_new:.2f} s/姿态", flush=True)
        else:
            # 并行：姿态切成 jobs*4 段，每段一个任务；子进程各自建场景（~1 s，~450 MB），结果按段序回写，
            # 所以 poses.jsonl 的行序与串行完全相同；指纹/续跑/汇总逻辑不变。
            import multiprocessing as mp
            from concurrent.futures import ProcessPoolExecutor
            nchunk = max(jobs * 4, 1); size = -(-len(pending) // nchunk)
            chunks = [pending[k:k + size] for k in range(0, len(pending), size)]
            print(f"  并行 {jobs} 进程，{len(chunks)} 段", flush=True)
            init = (str(L6.PLACED), str(a.bfp), tol, names, float(a.jaw_deg), jaw_parts)
            with ProcessPoolExecutor(max_workers=jobs, mp_context=mp.get_context("spawn"),
                                     initializer=_worker_init, initargs=init) as ex:
                for res in ex.map(_worker_eval, chunks):
                    for key, pose, hits, nb, ob in res:
                        pref_rows.append([*key, nb, ob])
                        f.write(_row(key, pose, hits, nb)); n_new += 1
                    f.flush()
                    el = time.time() - t0
                    print(f"  {n_new}/{len(pending)}  {el:.0f} s，{el/max(n_new,1):.3f} s/姿态（并行折算）", flush=True)
    # 汇总
    records = evidence.validate_pose_records(jl, expected, require_complete=not bool(a.limit))
    recs = list(records.values())
    per_pair, per_mode = {}, {}
    for r in recs:
        per_mode.setdefault(r["mode"], {"poses": 0, "hit_poses": 0})
        per_mode[r["mode"]]["poses"] += 1
        if r["hits"]:
            per_mode[r["mode"]]["hit_poses"] += 1
        for x, y, v in r["hits"]:
            key = f"{x}×{y}"
            d = per_pair.setdefault(key, {"hit_poses": 0, "max_mm3": 0.0, "modes": {}, "example": None})
            d["hit_poses"] += 1
            d["modes"][r["mode"]] = d["modes"].get(r["mode"], 0) + 1
            if v > d["max_mm3"]:
                d["max_mm3"] = v; d["example"] = {"mode": r["mode"], "case": r["case"], "step": r["step"], "pose_deg": r["pose_deg"]}
    if fingerprint != _fingerprint(a, scope, tol):
        raise ValueError("generation inputs changed during sampling; result NOT complete, use a NEW output directory")
    complete = not bool(a.limit) and set(records) == set(expected["rows"])
    summary = {"steps_file": str(a.steps.resolve()), "grid_deg": a.grid, "modes_in_scope": scope, "fingerprint": fingerprint,
               "mapping_file": str(a.bfp.resolve()), "placed_dir": str(L6.PLACED.resolve()),
               "complete": complete, "partial": not complete, "limit": a.limit,
               "expected_poses": len(expected["rows"]), "raw_prefall_samples": expected["raw_count"],
               "poses_sha256": evidence.sha(jl),
               "poses_checked": len(recs), "poses_with_hits": sum(1 for r in recs if r["hits"]),
               "per_mode": per_mode, "per_pair": dict(sorted(per_pair.items(), key=lambda kv: -kv[1]["max_mm3"]))}
    if abs(float(a.jaw_deg)) > 1e-12:
        summary["jaw_deg"] = float(a.jaw_deg)             # hr39：只在非 0 时写，0° 的 summary 与改前同形
    summary_path = out_dir / ("summary.json" if complete else "summary.partial.json")
    _write_json(summary_path, summary)
    # hr42：检查原语缓存命中情况另写 check_cache.json（summary.json 格式不变）：命中的布尔来自哪次 run（cached_from）、键版本、存储位置
    ev = CC.evidence(CC.collect_stats(os.environ["DUCK_CHECK_CACHE_STATS_DIR"]))
    _write_json(out_dir / "check_cache.json", ev)
    # hr42（B 线）：凸包分离证明跳过的布尔另写旁注（poses.jsonl 的 booleans = 实做的布尔数；+ 跳过数 = 改前的 booleans，逐姿态恒等）
    _write_json(out_dir / "hr42_prefilter.json", dict(
        note="booleans_run = poses.jsonl 每行 booleans（实做）；booleans_obb_prefiltered = 凸包分离证明交集必空而跳过的布尔（体积按 0.0 计，"
             "与 manifold 求交结果相同）；两者之和 = 改前的 booleans。rows 只含本次新算的姿态（续跑时之前的行不在）。",
        obb=bool(L6._obb_on()), booleans_run=int(sum(r[3] for r in pref_rows)),
        booleans_obb_prefiltered=int(sum(r[4] for r in pref_rows)),
        columns=["mode", "case", "step", "booleans_run", "booleans_obb_prefiltered"], rows=pref_rows))
    print("check_cache", json.dumps(ev["namespaces"], ensure_ascii=False))
    print(f"姿态 {len(recs)}，有碰撞的 {summary['poses_with_hits']}；件对 {len(per_pair)} → {summary_path}")
    for k, d in list(summary["per_pair"].items())[:15]:
        print(f"  {k}: {d['hit_poses']} 姿态，峰值 {d['max_mm3']:.1f} mm³，modes {d['modes']}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
