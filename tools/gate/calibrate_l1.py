#!/usr/bin/env python3
"""L1 判据标定：拿 printability.yaml:calibration_set 里的原版件（已被实打实打出来、装起来、走起来）跑**真 L1**，
统计每条判据在这些件上的失败比例，写成 data/l1_calibration.yaml。L1 的 `_calibration` 格读它判红绿。

    ./.venv/bin/python tools/gate/calibrate_l1.py            # ≈1~4 min（6 件逐层截面 + 最薄壁二分）

什么时候要重跑：printability.yaml 的 process/criteria/materials/calibration_set 任一改了、l1_printable.py 改了、标定网格换了
—— 记录里有指纹，L1 对不上就红，不会静默用旧记录。
统计口径：unknown（core.LayerResult.unknown 发出的、criterion 为"必须有数（未知=失败）"的 finding，例如没切片记录；
以及 state=PASS 但 evidence_n=0 的 finding —— 内核会把它改判 NOT_RUN，不是评过）不计入分母；
只数真评过的 PASS/FAIL。理由：标定的是判据本身量到的东西，不是数据缺不缺。
范围内 BLOCK 判据 n_eval=0 时 L1 的 `_calibration/proven_parts` 判 unknown（标定不成立）。"""
from __future__ import annotations
import hashlib, sys, time
from pathlib import Path

HERE = Path(__file__).resolve().parent
for p in (str(HERE), str(HERE / "layers"), str(HERE / "negatives")):
    if p not in sys.path:
        sys.path.insert(0, p)

import core  # noqa: E402
import l1_printable as L  # noqa: E402
from _harness import FakeCtx, base_data  # noqa: E402

OUT = core.ROOT / "tools/gate/data/l1_calibration.yaml"
TMP = core.ROOT / "tools/gate/out/_calib"


def main() -> int:
    import trimesh, yaml
    data = core.load_data()
    P = data.get("printability") or {}
    cs = P.get("calibration_set") or {}
    parts = cs.get("parts") or []
    if not parts:
        print("printability.yaml 没有 calibration_set.parts", file=sys.stderr)
        return 2
    TMP.mkdir(parents=True, exist_ok=True)
    fp = L.calibration_fingerprint(P)
    rec_parts, agg = {}, {}
    for cp in parts:
        cid = cp["id"]; src = core.ROOT / cp["mesh"]
        if not src.exists():
            print(f"{cid}: 网格缺失 {src}", file=sys.stderr)
            return 3
        t0 = time.perf_counter()
        m = trimesh.load(str(src), force="mesh")
        if cp.get("scale"):
            m.apply_scale(float(cp["scale"]))
        stl = TMP / f"{cid}.stl"; m.export(str(stl), file_type="stl")
        part = {"id": cid, "inventory_id": cid, "material": cp.get("material", "PLA"),
                "print_orientation": cp.get("note", "标定件"),
                "print_orientation_down_normal": list(cp["down"]),
                "print_orientation_frame": "export_local", "mirrored_copy": None}
        d = base_data([part], [])
        d["printability"] = dict(P, _calibration_inner_run=True)     # 内层不递归判标定格
        d.pop("l1_calibration", None)
        res = L.run(FakeCtx(d, {cid: stl}))
        fs = {}
        for f in res.findings:
            if f.subject != cid:
                continue
            # 两种"没评"：core.unknown 发的（criterion 原文）；以及 PASS 而 evidence_n=0（core.build_scorecard 会把它
            # 改判 NOT_RUN —— 例如 features 为空时 min_hole_dia 的『本件没有声明任何孔类特征』）。
            # 2026-09-13 F-L1-1 之前后者被数成"评过且通过"，8/8 假评。
            unknown = ((f.measured is None and f.criterion == L.CAL_UNKNOWN_CRITERION)
                       or (f.state == core.PASS and int(f.evidence_n or 0) == 0))
            fs[f.check] = {"state": f.state, "severity": f.severity, "unknown": bool(unknown),
                           "measured": (None if f.measured is None else str(f.measured)[:120])}
            a = agg.setdefault(f.check, {"severity": f.severity, "n_eval": 0, "n_fail": 0, "failed_parts": []})
            if not unknown:
                a["n_eval"] += 1
                if f.state == core.FAIL:
                    a["n_fail"] += 1; a["failed_parts"].append(cid)
        rec_parts[cid] = {"mesh_sha256": hashlib.sha256(src.read_bytes()).hexdigest(),
                          "down": list(cp["down"]), "seconds": round(time.perf_counter() - t0, 1), "findings": fs}
        print(f"{cid:24} {time.perf_counter()-t0:5.1f}s  " + "  ".join(
            f"{k}={'?' if v['unknown'] else v['state']}" for k, v in fs.items()), flush=True)
    for a in agg.values():
        a["fail_fraction"] = round(a["n_fail"] / a["n_eval"], 3) if a["n_eval"] else None
    doc = {"l1_calibration": {
        "date": time.strftime("%Y-%m-%d %H:%M"),
        "runner": "tools/gate/calibrate_l1.py",
        "fingerprint": fp,
        "fingerprint_note": "sha256(printability.yaml 的 process/criteria/materials/calibration_set 规范 JSON + layers/l1_printable.py 源码)；L1 对不上判红",
        "max_block_fail_fraction": cs.get("max_block_fail_fraction"),
        "checks_in_scope": list(cs.get("checks_in_scope") or []),
        "checks": agg, "parts": rec_parts}}
    OUT.write_text("# 由 tools/gate/calibrate_l1.py 生成，不要手改；判据/层代码/网格一变就要重跑（指纹校验）。\n"
                   + yaml.safe_dump(doc, allow_unicode=True, sort_keys=False, width=160), encoding="utf-8")
    print("\n判据汇总（BLOCK 超过上限的会让 L1/_calibration 红）：")
    scope = set(cs.get("checks_in_scope") or [])
    for k, a in sorted(agg.items()):
        inscope = k in scope
        flag = " ← 超" if (inscope and a["severity"] == core.BLOCK and a["n_eval"] and a["n_fail"] / a["n_eval"] > float(cs.get("max_block_fail_fraction") or 1)) else ("" if inscope else "  （范围外，只报不判）")
        print(f"  {k:32} [{a['severity']}] {a['n_fail']}/{a['n_eval']}{flag}")
    print("写入", OUT)
    return 0


if __name__ == "__main__":
    sys.exit(main())
