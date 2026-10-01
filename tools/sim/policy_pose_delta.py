"""增量真实姿态采样（快车道，2026-09-22；09-22 自检：--parts head_bottom_shell 复用 hr32b 得到与 hr32b 逐对逐值相同的 per_pair / per_mode，94 s vs 全量 14 min）：只重算含 --parts 的可动件对，其余件对复用 --reuse 目录里上一轮的逐姿态命中，
复用前逐件核 sha：当前 placed/ 里每个**不在 --parts 的**文件必须与 --snapshot（上一轮采样时 placed/ 的快照）逐字节相同。
输出与 policy_pose_collisions.py 同格式（run.json / poses.jsonl / summary.json，指纹按当前 placed/ 现算），summary 多一段 delta_reuse 记来源。
用法：./.venv/bin/python -B tools/sim/policy_pose_delta.py --parts head_bottom_shell --reuse tools/gate/out/policy_steps_..._hr32b --snapshot docs/.../hr32/placed --out <新目录> --jobs 6
"""
from __future__ import annotations
import argparse, json, sys, time, hashlib
from pathlib import Path
ROOT = Path(__file__).resolve().parents[2]
for _p in (ROOT / "tools/gate", ROOT / "tools/gate/layers", ROOT, ROOT / "tools/cad", ROOT / "tools/sim"):
    if str(_p) not in sys.path: sys.path.insert(0, str(_p))
from core import load_data
from gate import Ctx
import l6_motion as L6
import policy_pose_evidence as evidence
STEPS = ROOT / "tools/gate/out/policy_steps_2026-09-16/policy_steps.npz"
MANIFEST = ROOT / "tools/sim/cad_geometry_manifest.json"

def sha(p): return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def _write_json(path, payload):
    Path(path).write_text(json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")

_W = {}
def _worker_init(placed_dir, bfp_path, tol, names, parts):
    L6.PLACED = Path(placed_dir)
    data = load_data(); ctx = Ctx(data)
    bfp = json.loads(Path(bfp_path).read_text(encoding="utf-8"))["body_for_part"]
    sc = L6._Scene(ctx, bfp); ctx._mesh.clear()
    pairs = [(i, k) for i, k in sc.moving_pairs(names) if sc.names[i] in parts or sc.names[k] in parts]
    _W.update(sc=sc, pairs=pairs, tol=tol)
def _worker_eval(chunk):
    sc, pairs, tol = _W["sc"], _W["pairs"], _W["tol"]
    out = []
    for key, pose in chunk:
        hits, _, nb = sc.evaluate(pose, pairs, tol)
        out.append((key, pose, [(x, y, float(v)) for x, y, v in hits], nb))
    return out

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--parts", required=True, help="逗号分隔 placed 件名（stem），只重算含这些件的件对")
    ap.add_argument("--reuse", type=Path, required=True, help="上一轮完整采样目录（summary.json complete=true）")
    ap.add_argument("--snapshot", type=Path, required=True, help="上一轮采样时 placed/ 的快照目录（hrNN/placed）")
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--jobs", type=int, default=6)
    ap.add_argument("--steps", type=Path, default=STEPS)
    ap.add_argument("--bfp", type=Path, default=MANIFEST)
    ap.add_argument("--grid", type=float, default=2.5)
    a = ap.parse_args()
    parts = {s for s in a.parts.split(",") if s}
    data = load_data(); ctx = Ctx(data)
    cap = data["frozen"]["capability_envelope"]; scope = list(cap["modes_in_scope"])
    tol = float(data["tolerances"]["feature_check_tolerances"]["static_intersection_mm3"]["max"])
    expected = evidence.expected_poses(a.steps, scope, a.grid); names = expected["joint_names"]
    todo = list(expected["rows"].items())
    # ── 复用条件 ──
    old = json.loads((a.reuse / "summary.json").read_text(encoding="utf-8"))
    ofp = old["fingerprint"]
    if old.get("complete") is not True or old.get("partial") is not False: raise SystemExit("复用源不完整")
    if ofp["steps_sha256"] != sha(a.steps) or ofp["mapping_sha256"] != sha(a.bfp): raise SystemExit("复用源的 steps/mapping 与当前不同")
    if ofp["tol_mm3"] != tol or ofp["grid_deg"] != float(a.grid) or sorted(ofp["modes_in_scope"]) != sorted(scope): raise SystemExit("复用源的 tol/grid/scope 不同")
    if ofp["placed_sha256"] != evidence.placed_fingerprint(a.snapshot): raise SystemExit("快照目录与复用源采样时的 placed 指纹不一致，快照不可信")
    # 碰撞数学的代码依赖必须与复用源一致（运动链 MJCF / 场景与 FK l6_motion / core / 采样器）；改了就不能复用，先全量重采一次当新基线
    cur_fp = evidence.generation_fingerprint(L6.PLACED, a.steps, a.bfp, scope, a.grid, tol)
    for k in ("upstream/microduck_rl/src/mjlab_microduck/robot/microduck/robot_walk.xml", "tools/gate/layers/l6_motion.py", "tools/gate/core.py", "tools/sim/policy_pose_evidence.py"):
        if ofp["dependencies_sha256"].get(k) != cur_fp["dependencies_sha256"].get(k): raise SystemExit(f"复用源之后 {k} 变了，件对复用不成立 → 全量重采一次作新基线")
    if ofp.get("script_sha256") != cur_fp["script_sha256"]: raise SystemExit("policy_pose_collisions.py 变了，件对复用不成立 → 全量重采")
    cur = {p.name: sha(p) for p in sorted(L6.PLACED.glob("*.stl"))}
    snap = {p.name: sha(p) for p in sorted(a.snapshot.glob("*.stl"))}
    if set(cur) != set(snap): raise SystemExit(f"placed 文件集合变了：{sorted(set(cur) ^ set(snap))}")
    stems = {Path(n).stem for n in cur}
    if not parts <= stems: raise SystemExit(f"--parts 不在 placed 里：{sorted(parts - stems)}")
    changed_other = sorted(n for n in cur if Path(n).stem not in parts and cur[n] != snap[n])
    if changed_other: raise SystemExit(f"不在 --parts 里但内容变了的件：{changed_other} → 加进 --parts 或全量重采")
    unchanged = sorted(n for n in cur if Path(n).stem not in parts)
    old_records = evidence.validate_pose_records(a.reuse / "poses.jsonl", expected, require_complete=True)
    fingerprint = evidence.generation_fingerprint(L6.PLACED, a.steps, a.bfp, scope, a.grid, tol)
    out = a.out; out.mkdir(parents=True, exist_ok=True)
    if any((out / n).exists() for n in ("poses.jsonl", "summary.json", "run.json")): raise SystemExit("--out 必须是新目录")
    bfp = json.loads(a.bfp.read_text(encoding="utf-8"))["body_for_part"]
    t0 = time.time(); sc = L6._Scene(ctx, bfp); ctx._mesh.clear()
    pairs_all = sc.moving_pairs(names)
    pairs = [(i, k) for i, k in pairs_all if sc.names[i] in parts or sc.names[k] in parts]
    print(f"场景 {len(sc.names)} 实体，可动件对 {len(pairs_all)}，重算 {len(pairs)}（含 {sorted(parts)}），复用 {len(pairs_all)-len(pairs)}；建场景 {time.time()-t0:.1f} s")
    _write_json(out / "run.json", dict(fingerprint=fingerprint, expected_poses=len(todo), steps_file=str(a.steps.resolve()),
                                       mapping_file=str(a.bfp.resolve()), placed_dir=str(L6.PLACED.resolve()), modes_in_scope=scope, grid_deg=a.grid,
                                       delta_reuse=dict(reuse=str(a.reuse), snapshot=str(a.snapshot), parts=sorted(parts))))
    def _row(key, pose, hits, nb):
        mode, case, i = key
        return json.dumps({"mode": mode, "case": case, "step": i, "pose_deg": {k: round(v, 2) for k, v in pose.items()},
                           "pose_deg_exact": pose, "hits": [[x, y, float(v)] for x, y, v in hits], "booleans": nb}, ensure_ascii=False) + "\n"
    def merged(key, new_hits):
        keep = [(x, y, float(v)) for x, y, v in old_records[key]["hits"] if x not in parts and y not in parts]
        return keep + list(new_hits)
    t0 = time.time(); n_new = 0; nb_total = 0
    import multiprocessing as mp
    from concurrent.futures import ProcessPoolExecutor
    nchunk = max(a.jobs * 4, 1); size = -(-len(todo) // nchunk)
    chunks = [todo[k:k + size] for k in range(0, len(todo), size)]
    with (out / "poses.jsonl").open("w", encoding="utf-8") as f:
        with ProcessPoolExecutor(max_workers=a.jobs, mp_context=mp.get_context("spawn"), initializer=_worker_init,
                                 initargs=(str(L6.PLACED), str(a.bfp), tol, names, parts)) as ex:
            for res in ex.map(_worker_eval, chunks):
                for key, pose, hits, nb in res:
                    f.write(_row(key, pose, merged(key, hits), nb)); n_new += 1; nb_total += nb
                f.flush(); el = time.time() - t0
                print(f"  {n_new}/{len(todo)}  {el:.0f} s", flush=True)
    jl = out / "poses.jsonl"
    records = evidence.validate_pose_records(jl, expected, require_complete=True); recs = list(records.values())
    per_pair, per_mode = {}, {}
    for r in recs:
        per_mode.setdefault(r["mode"], {"poses": 0, "hit_poses": 0}); per_mode[r["mode"]]["poses"] += 1
        if r["hits"]: per_mode[r["mode"]]["hit_poses"] += 1
        for x, y, v in r["hits"]:
            d = per_pair.setdefault(f"{x}×{y}", {"hit_poses": 0, "max_mm3": 0.0, "modes": {}, "example": None})
            d["hit_poses"] += 1; d["modes"][r["mode"]] = d["modes"].get(r["mode"], 0) + 1
            if v > d["max_mm3"]: d["max_mm3"] = v; d["example"] = {"mode": r["mode"], "case": r["case"], "step": r["step"], "pose_deg": r["pose_deg"]}
    if fingerprint != evidence.generation_fingerprint(L6.PLACED, a.steps, a.bfp, scope, a.grid, tol): raise SystemExit("采样期间输入变了")
    summary = {"steps_file": str(a.steps.resolve()), "grid_deg": a.grid, "modes_in_scope": scope, "fingerprint": fingerprint,
               "mapping_file": str(a.bfp.resolve()), "placed_dir": str(L6.PLACED.resolve()), "complete": True, "partial": False, "limit": 0,
               "expected_poses": len(expected["rows"]), "raw_prefall_samples": expected["raw_count"], "poses_sha256": evidence.sha(jl),
               "poses_checked": len(recs), "poses_with_hits": sum(1 for r in recs if r["hits"]),
               "per_mode": per_mode, "per_pair": dict(sorted(per_pair.items(), key=lambda kv: -kv[1]["max_mm3"])),
               "delta_reuse": dict(reuse=str(a.reuse), reuse_poses_sha256=old["poses_sha256"], snapshot=str(a.snapshot), parts=sorted(parts),
                                   recomputed_pairs=len(pairs), reused_pairs=len(pairs_all) - len(pairs), booleans=nb_total,
                                   verified_unchanged_files=len(unchanged),
                                   note="复用件对的命中来自上一轮逐姿态记录；只有两件都未变（逐件 sha 与快照相同）的件对才复用")}
    _write_json(out / "summary.json", summary)
    print(f"姿态 {len(recs)}，有碰撞的 {summary['poses_with_hits']}；件对 {len(per_pair)}；布尔 {nb_total} → {out/'summary.json'}")
    for k, d in list(summary["per_pair"].items())[:15]: print(f"  {k}: {d['hit_poses']} 姿态，峰值 {d['max_mm3']:.1f} mm³")
    return 0
if __name__ == "__main__": sys.exit(main())
