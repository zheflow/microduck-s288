#!/usr/bin/env python3
"""反例（09-13）—— L1 `_calibration` 格（标定集守门）：判据误伤好件必须红、记录过期必须红、记录缺失必须红、网格换了必须红、
范围内 BLOCK 判据一次都没评过必须红（unknown）、WARN 判据超限只报不判；干净记录必须绿。

背景：docs/参考对比_2026-09-12_AI-FanGe复刻.md §5-A —— min_wall_geometric 把 11/16 已实打实打出来的原版件判红，判据被证伪。
自此 printability.yaml:calibration_set 声明一批"已证明能打"的原版件，tools/gate/calibrate_l1.py 跑真 L1 出 data/l1_calibration.yaml，
L1 末尾 `_calibration/proven_parts` 按它判：范围内任何 BLOCK 判据在可评件上的失败比例 > max_block_fail_fraction → 红。

2026-09-13 审计 F-L1-1：真记录里 min_wall_sliced / support_model_brackets_slicer / support_in_no_support_zone 三条 BLOCK 判据
`n_eval: 0`（一次都没评过）时 `proven_parts` 照样绿 —— "没评过"被当成"没误伤"。修法：范围内任何 BLOCK 判据 n_eval == 0 → unknown。
F-L1-2（用户口径）：WARN 判据超限另发 `_calibration/warn_checks_over_limit` FAIL(WARN)，只报不阻断。

本反例不跑标定（那要几分钟），只造 l1_calibration 记录喂给 L1（parts 为空，L1 只剩 `_calibration` 这一格）：
  A 记录新鲜、范围内 BLOCK 判据 min_hole_dia 4/6 失败、其余范围内判据都评过 → FAIL（误伤，measured 点名 min_hole_dia）
  B 记录新鲜、范围内 BLOCK 判据 1/6、WARN 1/6、范围外 BLOCK 6/6 → proven_parts PASS，warn_checks_over_limit PASS（范围外只报不判）
  C 指纹不等（判据/层代码改了没重标定）→ FAIL
  D 没有记录 → FAIL（unknown）
  E 某件网格 sha 与记录不符 → FAIL
  F 声明里没有 checks_in_scope → FAIL（unknown）
  G 范围内 WARN 判据 6/6 超限、BLOCK 全干净 → proven_parts 仍 PASS，warn_checks_over_limit FAIL(WARN)
  H 范围内 **所有** BLOCK 判据 n_eval=0（一次都没评）→ FAIL（unknown；"没评过"不是"没误伤"）
  I 范围内 **一条** BLOCK 判据 n_eval=0、其余干净 → FAIL（unknown）
指纹与网格 sha 都按当前真数据现算，所以本反例不依赖 data/l1_calibration.yaml 的内容。
`red` = A（抓到误伤，measured 非空）+ H（unknown 路径，allow_unknown_red）。
"""
from __future__ import annotations
import copy, hashlib, sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from _harness import FakeCtx, base_data, findings, record, worst_state  # noqa: E402
import core  # noqa: E402

NAME = "L1 标定集守门：误伤红 / 干净绿 / 过期红 / 缺记录红 / 换网格红 / 缺范围红 / 没评过红(unknown) / WARN 超限只报"
EXPECT = ("L1/_calibration:proven_parts（范围内 BLOCK 判据失败比例 > max_block_fail_fraction，或范围内 BLOCK 判据 n_eval=0 → unknown）；"
          "L1/_calibration:warn_checks_over_limit（范围内 WARN 判据超限 → FAIL(WARN)，不阻断）")

# 本层各判据的严重级（按 l1_printable.py 现状；范围内没列到的按 BLOCK 造 —— 最保守）
SEV = {"min_wall_geometric": "WARN", "min_wall_sliced": "BLOCK", "min_hole_dia": "BLOCK",
       "overhang_area": "INFO", "support_model_brackets_slicer": "BLOCK",
       "support_in_no_support_zone": "BLOCK", "support_reachable": "WARN", "first_layer_area": "INFO"}
N = 6


def _record(P, L, shas, scope, *, fails=None, zero=(), out_fail=0, fingerprint=None):
    """范围内每条判据默认评了 N 件、0 失败；fails={name: n_fail} 覆盖；zero=[name] 让该条 n_eval=0。"""
    fails = fails or {}
    checks = {}
    for name in scope:
        sev = SEV.get(name, "BLOCK")
        n_eval = 0 if name in zero else N
        n_fail = 0 if name in zero else int(fails.get(name, 0))
        checks[name] = {"severity": sev, "n_eval": n_eval, "n_fail": n_fail, "failed_parts": []}
    checks["slice_freshness"] = {"severity": "BLOCK", "n_eval": N, "n_fail": out_fail, "failed_parts": []}   # 范围外
    return {"l1_calibration": {"date": "反例", "fingerprint": fingerprint or L.calibration_fingerprint(P),
                               "checks": checks, "parts": {cid: {"mesh_sha256": sha} for cid, sha in shas.items()}}}


def run() -> dict:
    import l1_printable as L
    base = base_data([], [])
    P = base["printability"]
    cs = P.get("calibration_set") or {}
    scope = list(cs.get("checks_in_scope") or [])
    if not cs.get("parts") or not scope:
        return dict(name=NAME, passed=False, expect=EXPECT, got="printability.yaml 没有 calibration_set / checks_in_scope，反例无法构造")
    block_in_scope = [k for k in scope if SEV.get(k, "BLOCK") == "BLOCK"]
    warn_in_scope = [k for k in scope if SEV.get(k) == "WARN"]
    if not block_in_scope or not warn_in_scope:
        return dict(name=NAME, passed=False, expect=EXPECT, got=f"checks_in_scope 里没有 BLOCK 或 WARN 判据（{scope}），反例无法构造")
    shas = {}
    for cp in cs["parts"]:
        mp = core.ROOT / cp["mesh"]
        if not mp.exists():
            return dict(name=NAME, passed=False, expect=EXPECT, got=f"标定集网格缺失：{cp['mesh']}（upstream 不在 git 里，见 upstream/README.md）")
        shas[cp["id"]] = hashlib.sha256(mp.read_bytes()).hexdigest()

    def observe(rec, P_override=None):
        d = copy.deepcopy(base)
        if P_override is not None:
            d["printability"] = P_override
        if rec is None:
            d.pop("l1_calibration", None)
        else:
            d["l1_calibration"] = rec
        res = L.run(FakeCtx(d, {}))
        pp = findings(res, subject="_calibration", check="proven_parts")
        ww = findings(res, subject="_calibration", check="warn_checks_over_limit")
        return dict(state=worst_state(pp), measured=(pp[0].measured if pp else None),
                    detail=(pp[0].detail if pp else ""), pp=pp,
                    w_state=worst_state(ww), w_sev=(ww[0].severity if ww else None),
                    w_measured=(ww[0].measured if ww else None), ww=ww)

    b0, w0 = block_in_scope[0], warn_in_scope[0]
    A = observe(_record(P, L, shas, scope, fails={b0: 4}))
    B = observe(_record(P, L, shas, scope, fails={b0: 1, w0: 1}, out_fail=N))
    C = observe(_record(P, L, shas, scope, fingerprint="0" * 64))
    D = observe(None)
    shas_bad = dict(shas); k0 = next(iter(shas_bad)); shas_bad[k0] = "f" * 64
    E = observe(_record(P, L, shas_bad, scope))
    P2 = copy.deepcopy(P); P2["calibration_set"] = {k: v for k, v in cs.items() if k != "checks_in_scope"}
    F = observe(_record(P2, L, shas, scope), P_override=P2)
    G = observe(_record(P, L, shas, scope, fails={w0: N}))
    H = observe(_record(P, L, shas, scope, zero=block_in_scope))
    I = observe(_record(P, L, shas, scope, zero=[block_in_scope[-1]]))

    checks = [
        ("A 误伤 → proven_parts FAIL 且 measured 点名判据", A["state"] == "FAIL" and A["measured"] is not None and b0 in str(A["measured"])),
        ("B 干净 → proven_parts PASS", B["state"] == "PASS"),
        ("B 干净 → warn_checks_over_limit PASS（范围外 BLOCK 6/6 只报不判）", B["w_state"] == "PASS"),
        ("C 指纹不等 → FAIL", C["state"] == "FAIL" and "指纹" in str(C["measured"])),
        ("D 缺记录 → FAIL(unknown)", D["state"] == "FAIL" and D["measured"] is None),
        ("E 换网格 → FAIL", E["state"] == "FAIL" and "sha" in str(E["measured"])),
        ("F 缺范围 → FAIL(unknown)", F["state"] == "FAIL" and F["measured"] is None),
        ("G WARN 超限 → proven_parts 仍 PASS", G["state"] == "PASS"),
        ("G WARN 超限 → warn_checks_over_limit FAIL 且 severity=WARN", G["w_state"] == "FAIL" and G["w_sev"] == "WARN"
         and G["w_measured"] is not None and w0 in str(G["w_measured"])),
        ("H 范围内 BLOCK 全部 n_eval=0 → FAIL(unknown)", H["state"] == "FAIL" and H["measured"] is None
         and "没评" in str(H["detail"])),
        ("I 范围内一条 BLOCK n_eval=0 → FAIL(unknown)", I["state"] == "FAIL" and I["measured"] is None),
    ]
    failed = [c for c, ok in checks if not ok]
    red = [record(f) for f in A["pp"] if f.state == "FAIL"] + [record(f) for f in H["pp"] if f.state == "FAIL"]
    got = (f"A 误伤 {A['state']}({str(A['measured'])[:40]})；B 干净 {B['state']}/warn {B['w_state']}；C 过期 {C['state']}；"
           f"D 缺记录 {D['state']}；E 换网格 {E['state']}；F 缺范围 {F['state']}；G WARN 超限 proven {G['state']}/warn "
           f"{G['w_state']}({G['w_sev']})；H 全没评 {H['state']}(measured={H['measured']})；I 一条没评 {I['state']}(measured={I['measured']})")
    return dict(name=NAME, passed=not failed, expect=EXPECT, got=got + (f"｜未成立：{failed}" if failed else ""),
                expect_severity="BLOCK", red=red,
                allow_unknown_red="H/I 验的是『范围内 BLOCK 判据一次都没评过 → 标定不成立 = unknown』这条设计路径；A 的 measured 非空",
                detail=f"scope={scope}；BLOCK 在范围内：{block_in_scope}；WARN 在范围内：{warn_in_scope}")


if __name__ == "__main__":
    import json
    r = run()
    print(json.dumps(r, ensure_ascii=False, indent=1))
    sys.exit(0 if r["passed"] else 1)
