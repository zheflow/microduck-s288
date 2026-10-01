#!/usr/bin/env python3
"""反例（gate_guard 第 6 项，2026-09-25）—— fasteners.yaml:<组>.joins 被 YAML 拆碎：只红这一组，不许整层崩。

事故（校验会话 09-25 18:55，hr43d）：joins 流式列表里一项含半角逗号没加引号 → YAML 拆成几项、数字段变成 float/int →
l5_function._blind_depth 的 " ".join 抛 TypeError，整个 l5_function 没交回任何判据（hr43d L5 少 49 格）。
正确判据：只给该组发 joins_format FAIL(BLOCK)（measured 如 "joins[3]=int 23"），该组盲深不从 joins/location 文字取
（写了 pilot.pilot_depth_mm 仍照判），其他组照常评；数据正常时不发这条。

造法（数据注入同 n_l5_pilot_depth_priority：base_data + FakeCtx + l5_function.run）：两组自攻 M2×8、叠厚 4.0，底孔深写在
joins 文字里（"H01 Ø1.6 底孔 4.5"，4.0 + 4.5 − 0.3 = 8.2 ≥ 8 → 绿）。BAD 组的坏 joins 由 yaml.safe_load 读一条**不加引号**的
流式列表 "[N03 板, H01 Ø1.6 底孔 4.5, 孔距 58, 23]" 得到（= 真 YAML 拆碎：[…, '孔距 58', 23]），干净版同一项加引号。
  A 干净：没有任何 joins_format；BAD 组 screw_length_rule 按 joins 文字 4.5 判 PASS
  B 坏样本：层不崩、交回判据；L5/BAD:joins_format FAIL(BLOCK) measured="joins[3]=int 23"，detail 点名 fasteners.yaml:BAD.joins、提示加引号；
    全卡只有这一条 joins_format；OK 组逐条与 A 相同；BAD 组 = A 的判据 + joins_format，只有 screw_length_rule 变 unknown（盲深拿不到数）；
    把结果交给 gate._layer_guard（全量跑口径）→ 不出 L5/_layer_error
  C 坏样本 + BAD 组写了 pilot.pilot_depth_mm=4.5 → screw_length_rule 照判 PASS（盲深来源 pilot.pilot_depth_mm），joins_format 照红
旧 l5_function（没有这道检查）上跑：B/C 在 " ".join 抛 TypeError → 本反例红。
"""
from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))   # tools/gate（gate._layer_guard）
sys.path.insert(0, str(Path(__file__).resolve().parent))
from _harness import FakeCtx, assert_red, base_data, findings, result  # noqa: E402

BAD, OK = "F_NEG_joins_bad", "F_NEG_joins_ok"
NAME = "joins 被 YAML 拆碎（'孔距 58, 23' 没加引号 → int 23）：只红该组 joins_format BLOCK，层不崩，其他组照常"
EXPECT = f"L5/{BAD}:joins_format FAIL(BLOCK) measured='joins[3]=int 23'；OK 组逐条不变；BAD 组只 screw_length_rule 变 unknown；无 L5/_layer_error；干净无 joins_format"
STACK = 4.0
FLOW_BAD = "[N03 板, H01 Ø1.6 底孔 4.5, 孔距 58, 23]"          # 含半角逗号的项没加引号
FLOW_OK = "[N03 板, H01 Ø1.6 底孔 4.5, '孔距 58, 23']"         # 同一项加了引号


def _fa(gid, joins, pilot_depth=None):
    fa = {"id": gid, "spec": "M2×8", "qty": 1, "joins": joins, "joint_type": "pla_self_tap",
          "engagement_mm": {"v": 8.0 - STACK, "src": "measured"}, "stack_mm": {"v": STACK, "src": "measured"},
          "status": "ok", "status_verbatim": "ok", "feature_ids": [], "feature_hole_map": [],
          "provenance": ["negatives/n_l5_joins_format"]}
    if pilot_depth is not None:
        fa["pilot"] = {"pilot_hole_d_mm": {"v": 1.6, "src": "assumed"},
                       "pilot_depth_mm": {"v": pilot_depth, "src": "measured", "src_note": "反例"}}
    return fa


def _run(bad_flow, bad_pilot=None):
    import yaml
    import l5_function
    part = {"id": "N03", "inventory_id": "N03_neg_joins", "material": "PLA", "print_orientation": "任意", "mirrored_copy": None}
    data = base_data([part], [], fasteners=[_fa(BAD, yaml.safe_load(bad_flow), bad_pilot),
                                            _fa(OK, yaml.safe_load(FLOW_OK))], relations=[])
    try:
        return l5_function.run(FakeCtx(data, {})), None
    except Exception as e:                       # 旧 l5_function：" ".join 抛 TypeError —— 记下来当失败
        return None, f"{type(e).__name__}: {e}"


def _key(f):
    return (f.subject, f.check, f.state, f.severity, repr(f.measured), f.evidence_n, str(f.criterion), str(f.detail))


def run() -> dict:
    import gate
    A, errA = _run(FLOW_OK)
    B, errB = _run(FLOW_BAD)
    C, errC = _run(FLOW_BAD, bad_pilot=4.5)
    a = []
    fa_all = list(A.findings) if A else []
    fb_all = list(B.findings) if B else []
    a.append({"claim": "A 干净：层正常交回判据、没有任何 joins_format；BAD 组 screw_length_rule 按 joins 文字 4.5 判 PASS",
              "ok": bool(A and fa_all and not any(f.check == "joins_format" for f in fa_all)
                         and [f.state for f in findings(A, subject=BAD, check="screw_length_rule")] == ["PASS"])})
    jf = [f for f in fb_all if f.check == "joins_format"]
    a.append({"claim": "B 坏样本：层不崩、交回判据；全卡只有一条 joins_format，在 BAD 组，FAIL/BLOCK，measured='joins[3]=int 23'",
              "ok": bool(B and fb_all and len(jf) == 1 and jf[0].subject == BAD and jf[0].state == "FAIL"
                         and jf[0].severity == "BLOCK" and jf[0].measured == "joins[3]=int 23")})
    a.append({"claim": "B joins_format detail 点名 fasteners.yaml:<组>.joins、提示加引号",
              "ok": bool(jf and f"fasteners.yaml:{BAD}.joins" in str(jf[0].detail) and "引号" in str(jf[0].detail))})
    other_a = sorted(_key(f) for f in fa_all if f.subject != BAD)
    other_b = sorted(_key(f) for f in fb_all if f.subject != BAD)
    a.append({"claim": "B 其他格（OK 组 + 汇总格）逐条与干净运行相同", "ok": bool(B and other_a and other_a == other_b)})
    bad_a = sorted(_key(f) for f in fa_all if f.subject == BAD and f.check != "screw_length_rule")
    bad_b = sorted(_key(f) for f in fb_all if f.subject == BAD and f.check not in ("screw_length_rule", "joins_format"))
    slr_b = findings(B, subject=BAD, check="screw_length_rule") if B else []
    a.append({"claim": "B BAD 组 = 干净判据 + joins_format，只有 screw_length_rule 变 unknown（盲深拿不到数，detail 写 joins 格式错）",
              "ok": bool(B and bad_a == bad_b and len(slr_b) == 1 and slr_b[0].state == "FAIL" and slr_b[0].measured is None
                         and "joins 格式错" in str(slr_b[0].detail))})
    guard = None
    if B is not None:
        sc = {"cells": {}, "tally": {"PASS": 0, "FAIL": 0, "STALE": 0, "NOT_RUN": 0, "WAIVED": 0, "RETIRED": 0},
              "blocking_cells": [], "verdict": "CLEAR"}
        outs = [dict(n=5, mod="l5_function", name=B.name, result=B, error=None, seconds=0.0)]
        guard = gate._layer_guard(outs, {}, [B], False, sc)
        guard = (guard, sorted(sc["cells"]))
    a.append({"claim": "B 结果交给 gate._layer_guard（全量跑口径）→ 不出 L5/_layer_error、没有 ⚠ 行",
              "ok": guard == (([], []), [])})
    slr_c = findings(C, subject=BAD, check="screw_length_rule") if C else []
    a.append({"claim": "C 写了 pilot.pilot_depth_mm=4.5 → screw_length_rule 照判 PASS（来源 pilot.pilot_depth_mm），joins_format 照红",
              "ok": bool(C and [f.state for f in slr_c] == ["PASS"] and "pilot.pilot_depth_mm" in str(slr_c[0].detail)
                         and [f.state for f in findings(C, subject=BAD, check="joins_format")] == ["FAIL"])})
    okR, whyR, red = assert_red(jf, severity="BLOCK")
    passed = okR and all(x["ok"] for x in a)
    got = (f"A {'崩了：' + errA if errA else f'{len(fa_all)} 条'}｜B {'崩了：' + errB if errB else f'{len(fb_all)} 条，joins_format={[(f.subject, f.measured) for f in jf]}'}"
           f"｜BAD screw_length_rule={[(f.state, f.measured) for f in slr_b]}｜C {'崩了：' + errC if errC else [(f.state, f.measured) for f in slr_c]}"
           f"｜guard={guard}")
    out = result(NAME, EXPECT, passed, got, red, expect_severity="BLOCK", detail=whyR)
    out["assertions"] = a
    return out


if __name__ == "__main__":
    import json
    r = run()
    print(json.dumps(r, ensure_ascii=False, indent=1, default=str))
    sys.exit(0 if r["passed"] else 1)
