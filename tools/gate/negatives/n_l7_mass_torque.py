#!/usr/bin/env python3
"""反例 —— 质量与力（第 7 层）三个坏样本，都经过 `l7_mass.run(FakeCtx)`。

第 7 层是整机量（每关节静扭矩 / 重心投影 / MJCF 执行器上限），没有"最小解析件"可造：
本反例用**真 placed/ 与真 STL**（只读，不改），只对数据 / 一份临时 STL / 一份临时 MJCF 做最小突变：
  ① components.yaml:servo_s288.mass_g ×SERVO_X（19.5 g/颗，14 颗）→ 某关节 `joint:*:static_torque` FAIL(BLOCK)，
     measured = 峰值/额定 的比值 ≥ 1（额定 0.6 N·m 取自 tolerances.yaml，层里不写死）。
     实测 ×10 只把最吃力的 neck_pitch 推到额定的 89.8%（仍绿），所以取 ×20（≈1.6×额定）。
  ② 电池 placed 实体（zz_battery，按 envelope 72×25×18 认领）整体沿 −x 平移 200 mm → 整机重心
     从 x=−1.9 挪到 −21.4，出了双脚凸包 x∈[−14.7, 28.7] → `_stability:com_in_support_home` FAIL(BLOCK)，
     measured = 负余量 mm（+x 方向 200 mm 只到 17.5，还在包络内，所以选 −x）。placed/ 换成一个替身目录：
     其余实体软链接到真文件，只有 zz_battery.stl 是平移后的临时件。
  ③ sim/duck_s288/robot_walk_s288.xml 的 <actuator forcerange> 改成 ±0.7（临时 XML，猴子补丁 l7_mass._SIM_MJCF）
     → 14 条 `joint:*:forcerange_vs_servo_rating` FAIL(BLOCK)，measured=0.7 > 堵转 0.6
  对照  真数据 + 真 placed + forcerange ±0.5 的临时 XML → static_torque 全 PASS、com_in_support_home PASS、
        forcerange_vs_servo_rating 全 PASS
（真 XML 目前是 ±0.91，本来就红 —— 对照必须用 ±0.5 的临时件，否则"绿"无从证明。）
2026-09-13（F-L7-4）起真数据下 static_torque / com_in_support_home 因质量清单缺口一律 unknown，所以三次 run 的
数据都先过 `_l7_fixture.complete_inventory`（只删不编：无实体无质量的元件 qty=0、无质量的轴承与 orig_* 连实体一起拿掉、
6704/6700 收成单一宿主件），placed/ 替身目录由 `_l7_fixture.placed_stand_in` 建（电池平移是在它之上再做的突变）。
每次 run 都是整机 L7（≈8.5 s）。为了守住 30 s，① 与 ③ 合在同一次 run 里（forcerange 判据只读 MJCF，
静扭矩只读质量，subject/check 互不相干，各自独立断言）；② 单独一次；对照一次 —— 共 3 次。
"""
from __future__ import annotations
import json
import os
import re
import sys
import time
from copy import deepcopy
from pathlib import Path

_HERE = Path(__file__).resolve()
GATE = _HERE.parents[1]
for _p in (str(_HERE.parent), str(GATE), str(GATE / "layers"), str(GATE.parents[1])):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import core                                                                      # noqa: E402
from _harness import FakeCtx, findings, assert_red, assert_green, result, record, summarize  # noqa: E402
import _l7_fixture as FX                                                         # noqa: E402

NAME = "L7：舵机质量×20 → static_torque 红；电池挪 200 mm → com_in_support_home 红；forcerange ±0.7 → 超堵转红；对照全绿"
EXPECT = ("L7/joint:*:static_torque FAIL(BLOCK)；L7/_stability:com_in_support_home FAIL(BLOCK)；"
          "L7/joint:*:forcerange_vs_servo_rating FAIL(BLOCK) measured=0.7；对照三者全 PASS")
_TMP = core.ROOT / "tools/gate/out/_neg_tmp"
SERVO_X = 20.0
BATTERY_SHIFT = (-200.0, 0.0, 0.0)


def _stl_map():
    """与 gate.Ctx.stl 同规则：cad/duck_s288/<件号>_*.stl。"""
    out = {}
    for p in sorted(core.STL_DIR.glob("*.stl")):
        pid = p.stem.split("_", 1)[0]
        out.setdefault(pid, p)
    return out


def _placed_battery_shifted(shift, drop):
    """替身 placed/：真实体软链接（夹具要拿掉的不链），只有 zz_battery.stl 换成平移后的临时件。"""
    import trimesh
    moved = {}

    def mut(stem, src, dst):
        if stem != "zz_battery":
            return False
        m = trimesh.load(str(src), process=False)
        m.apply_translation(list(shift))
        m.export(str(dst), file_type="stl")
        moved["name"] = src.name
        return True

    d = FX.placed_stand_in("battery_shift", drop, mutate=mut)
    return d, moved.get("name")


def _mjcf_with_forcerange(tag, val):
    src = core.ROOT / "sim/duck_s288/robot_walk_s288.xml"
    txt = src.read_text(encoding="utf-8")
    n = len(re.findall(r'forcerange="[^"]*"', txt))
    txt2 = re.sub(r'forcerange="[^"]*"', f'forcerange="-{val} {val}"', txt)
    p = _TMP / f"l7_mjcf_{tag}.xml"
    p.write_text(txt2, encoding="utf-8")
    return p, n


def _run(data, placed_dir=None, mjcf=None):
    import l7_mass as L7
    old_placed, old_mjcf = L7.PLACED, L7._SIM_MJCF
    try:
        if placed_dir is not None:
            L7.PLACED = placed_dir
        if mjcf is not None:
            L7._SIM_MJCF = mjcf
        return L7.run(FakeCtx(data, _stl_map(), {}))
    finally:
        L7.PLACED, L7._SIM_MJCF = old_placed, old_mjcf


def _servo_scaled(base, k):
    d = deepcopy(base)
    c = next(x for x in d["components"]["components"] if x.get("id") == "servo_s288")
    c["mass_g"]["v"] = float(c["mass_g"]["v"]) * k
    return d, c["mass_g"]["v"]


def run() -> dict:
    base, drop = FX.complete_inventory(core.load_data())          # 完整清单夹具（见文件头 / _l7_fixture.py）
    pd_ctrl = FX.placed_stand_in("mass_torque_ctrl", drop)
    t = {}
    mj_good, n_fr = _mjcf_with_forcerange("good_0p5", 0.5)
    mj_bad, _ = _mjcf_with_forcerange("bad_0p7", 0.7)

    # 对照：真数据 + 真 placed + forcerange ±0.5
    t0 = time.perf_counter(); r_good = _run(base, placed_dir=pd_ctrl, mjcf=mj_good); t["good"] = round(time.perf_counter() - t0, 1)
    g_tor = [f for f in r_good.findings if f.check == "static_torque"]
    g_com = findings(r_good, subject="_stability", check="com_in_support_home")
    g_fr = [f for f in r_good.findings if f.check == "forcerange_vs_servo_rating"]
    ok_g_tor, why_g_tor = assert_green(g_tor)
    ok_g_com, why_g_com = assert_green(g_com)
    ok_g_fr, why_g_fr = assert_green(g_fr)

    # ① 舵机质量 ×SERVO_X + ③ forcerange ±0.7（同一次 run，两组判据互不相干）
    d1, m_k = _servo_scaled(base, SERVO_X)
    t0 = time.perf_counter(); r13 = _run(d1, placed_dir=pd_ctrl, mjcf=mj_bad); t["servo_x_and_forcerange"] = round(time.perf_counter() - t0, 1)
    tor1 = [f for f in r13.findings if f.check == "static_torque"]
    ok1, why1, red1 = assert_red(tor1, severity="BLOCK", require_measured=True)
    ratio_ok = ok1 and all(float(x["measured"]) >= 1.0 for x in red1)
    fr3 = [f for f in r13.findings if f.check == "forcerange_vs_servo_rating"]
    ok3, why3, red3 = assert_red(fr3, severity="BLOCK", require_measured=True)
    fr_ok = ok3 and len(red3) == len(fr3) and all(abs(float(x["measured"]) - 0.7) < 1e-9 for x in red3)

    # ② 电池挪 200 mm（−x），质量与 MJCF 都是真的
    pd, moved = _placed_battery_shifted(BATTERY_SHIFT, drop)
    t0 = time.perf_counter(); r2 = _run(base, placed_dir=pd, mjcf=mj_good); t["battery_shift"] = round(time.perf_counter() - t0, 1)
    com2 = findings(r2, subject="_stability", check="com_in_support_home")
    ok2, why2, red2 = assert_red(com2, severity="BLOCK", require_measured=True)
    neg_ok = ok2 and float(red2[0]["measured"]) < 0

    passed = ok_g_tor and ok_g_com and ok_g_fr and ok1 and ratio_ok and ok2 and neg_ok and ok3 and fr_ok
    worst1 = max((float(x["measured"]) for x in red1), default=None)
    return result(NAME, EXPECT, passed,
                  got=(f"① 舵机 ×{SERVO_X:g}={m_k} g/颗 → static_torque {'红' if ok1 else why1}（{len(red1)}/{len(tor1)} 关节红，最大比值 {worst1}）；"
                       f"② 电池 {BATTERY_SHIFT} mm → com_in_support_home {'红' if ok2 else why2}（余量 {red2[0]['measured'] if red2 else None} mm）；"
                       f"③ forcerange ±0.7 → {'红' if ok3 else why3}（{len(red3)}/{len(fr3)} 条，替换了 {n_fr} 处 forcerange）；"
                       f"对照：static_torque {'绿' if ok_g_tor else why_g_tor}，com_in_support_home {'绿' if ok_g_com else why_g_com}"
                       f"（余量 {g_com[0].measured if g_com else None}），forcerange ±0.5 {'绿' if ok_g_fr else why_g_fr}；耗时 {t}"),
                  red=red1 + red2 + red3, expect_severity="BLOCK",
                  detail=(f"①={summarize(sorted(tor1, key=lambda f: -float(f.measured or 0)), n=3)}；"
                          f"②={[record(f) for f in com2]}（挪的实体 {moved}）；③={summarize(fr3, n=2)}"))


if __name__ == "__main__":
    print(json.dumps(run(), ensure_ascii=False, indent=1))
