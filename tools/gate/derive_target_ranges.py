#!/usr/bin/env python3
"""把 frozen.yaml:joint_axes[].target_range_deg 从"能力包络"推导出来（2026-09-16，用户定：能力必须与原版一致，转角可以不同）。

    ./.venv/bin/python tools/gate/derive_target_ranges.py            # 写回 frozen.yaml
    ./.venv/bin/python tools/gate/derive_target_ranges.py --check    # 只核对，不写（Gate 之外的自检）

规则（全部在 frozen.yaml:capability_envelope 里声明，本脚本不带数字）：
  target_range_deg.v = clip( ∪_{mode ∈ modes_in_scope} actual_deg[mode][joint] ± margin_deg ,  上游 MJCF range )
  · 包络来源：docs/workspace_2026-09-09/policy_envelope.json（原版策略在原版 XML 里跑出来的**实际**关节角）
  · 上游 range：upstream/microduck_rl/.../robot_walk.xml 的 <joint range>（扫描域上限，不能超）
  · 某关节在所有 in-scope mode 里都没有数据 → 保持上游值（能力对该关节没有更窄的要求）
  · 写回只动 target_range_deg 的 v / date / reason / src 四行，src_note（上游出处）原样保留

为什么不手填：手填的数字（09-15 那次按我们的件收窄到 62/62/44.5）是"件能转多少判据就定多少"，判据失去约束力。
这里数字只来自包络文件 + 上游 XML，改件不会改判据；换包络文件（重训后重新导出）再跑一次本脚本即可。
"""
from __future__ import annotations

import argparse
import json
import math
import re
import sys
import xml.etree.ElementTree as ET
from datetime import date
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[2]
FROZEN = ROOT / "tools/gate/data/frozen.yaml"
UPSTREAM_XML = ROOT / "upstream/microduck_rl/src/mjlab_microduck/robot/microduck/robot_walk.xml"


def upstream_ranges_deg() -> dict[str, tuple[float, float]]:
    root = ET.parse(str(UPSTREAM_XML)).getroot()
    out = {}
    for j in root.iter("joint"):
        nm, rg = j.get("name"), j.get("range")
        if nm and rg:
            lo, hi = (float(x) for x in rg.split())
            out[nm] = (round(math.degrees(lo), 1), round(math.degrees(hi), 1))
    return out


def envelope_box(env: dict, modes: list[str], key: str) -> tuple[dict, dict, list]:
    """→ (box{joint:(lo,hi)}, per_mode{mode:{joint:(lo,hi)}}, missing_modes)。key 形如 'all.actual_deg'。"""
    k1, k2 = key.split(".", 1)
    pe = env.get("policy_envelope") or {}
    box, per_mode, missing = {}, {}, []
    for m in modes:
        joints = pe.get(m)
        if not joints:
            missing.append(m)
            continue
        for j, w in joints.items():
            t = ((w or {}).get(k1) or {}).get(k2) or {}
            lo, hi = t.get("min"), t.get("max")
            if lo is None or hi is None:
                continue
            lo, hi = float(lo), float(hi)
            per_mode.setdefault(m, {})[j] = (lo, hi)
            b = box.get(j)
            box[j] = (min(lo, b[0]), max(hi, b[1])) if b else (lo, hi)
    return box, per_mode, missing


def _mirror(box: dict, pairs) -> dict:
    """左右按符号镜像求并（就地）：能力不分左右，镜像件只能对称地切。→ {joint: 说明}"""
    mirrored = {}
    for left, right, sign in pairs or []:
        sign = float(sign)
        bl, br = box.get(left), box.get(right)
        if bl is None and br is None:
            continue
        cand = []
        if bl:
            cand.append(bl)
        if br:
            cand.append(tuple(sorted((sign * br[0], sign * br[1]))))
        u = (min(c[0] for c in cand), max(c[1] for c in cand))
        box[left] = u
        box[right] = tuple(sorted((sign * u[0], sign * u[1])))
        mirrored[left] = mirrored[right] = f"镜像并集（{left}↔{right}，sign {sign:+.0f}）"
    return mirrored


def capability_box(cap: dict, env: dict):
    """frozen.yaml:capability_envelope + 包络 json → (box, per_mode, mirrored, missing_modes)。
    box / per_mode 都已按 mirror_pairs 镜像求并（若 mirror_symmetric），**没有**加 margin、没有夹上游 range。
    第 6 层与本脚本共用这一个函数，判据只有一份实现。"""
    modes = list(cap["modes_in_scope"])
    key = str(cap.get("key", "all.actual_deg"))
    box, per_mode, missing = envelope_box(env, modes, key)
    mirrored = {}
    if cap.get("mirror_symmetric"):
        pairs = cap.get("mirror_pairs") or []
        mirrored = _mirror(box, pairs)
        for m in per_mode:
            _mirror(per_mode[m], pairs)
    return box, per_mode, mirrored, missing


def derive(frozen: dict) -> tuple[dict, dict]:
    cap = frozen.get("capability_envelope") or {}
    src = ROOT / str(cap["source"])
    env = json.loads(src.read_text(encoding="utf-8"))
    modes = list(cap["modes_in_scope"])
    margin = float(cap["margin_deg"])
    key = str(cap.get("key", "all.actual_deg"))
    box, per_mode, mirrored, missing = capability_box(cap, env)
    if missing:
        raise SystemExit(f"capability_envelope.modes_in_scope 里这些 mode 在 {src.name} 里不存在：{missing}")
    up = upstream_ranges_deg()
    out = {}
    for j in frozen["joint_axes"]:
        nm = j.get("name")
        if not j.get("exists_as_mjcf_joint") or nm not in up:
            continue
        ulo, uhi = up[nm]
        if nm in box:
            lo = max(ulo, math.floor((box[nm][0] - margin) * 10) / 10)
            hi = min(uhi, math.ceil((box[nm][1] + margin) * 10) / 10)
            why = (f"能力包络：{len([m for m in per_mode if nm in per_mode[m]])} 个 in-scope mode 的实际角并集"
                   f"{'（' + mirrored[nm] + '）' if nm in mirrored else ''} "
                   f"[{box[nm][0]:.1f}, {box[nm][1]:.1f}] 各留 {margin:g}° 边，夹到上游 range [{ulo:.1f}, {uhi:.1f}]")
        else:
            lo, hi = ulo, uhi
            why = f"能力包络里没有本关节的数据 → 保持上游 range [{ulo:.1f}, {uhi:.1f}]"
        out[nm] = dict(v=[lo, hi], upstream=(ulo, uhi), why=why,
                       modes={m: per_mode[m][nm] for m in per_mode if nm in per_mode[m]})
    return out, dict(source=str(cap["source"]), modes=modes, margin=margin, key=key,
                     mirror=bool(cap.get("mirror_symmetric")))


def _block_span(text: str, joint: str) -> tuple[int, int]:
    """target_range_deg: 子块在 text 里的 [start, end)（从 '    target_range_deg:' 行到下一个 4 空格缩进的键）。"""
    m = re.search(rf'^  - name: "{re.escape(joint)}"\n', text, flags=re.M)
    if not m:
        raise SystemExit(f"frozen.yaml 里找不到 joint {joint}")
    s = text.index("    target_range_deg:\n", m.end())
    e = re.compile(r"^    [a-z_]+:", re.M).search(text, s + 1)
    return s, e.start()


def write_back(text: str, derived: dict, meta: dict, today: str) -> tuple[str, list[str]]:
    changed = []
    for nm, d in derived.items():
        s, e = _block_span(text, nm)
        block = text[s:e]
        note = re.search(r"^      src_note: .*\n", block, flags=re.M)
        if not note:
            raise SystemExit(f"{nm}: target_range_deg 子块里没有 src_note（上游出处）行，不敢改")
        # 保留旧的 src_note；如果旧块的 v 是上游值且 src_note 没写"旧值"，把上游出处留在 src_note 里即可
        old_v = re.search(r"^      v: \[(.*?)\]\n", block, flags=re.M)
        lo, hi = d["v"]
        reason = (f"{today} 能力包络推导（tools/gate/derive_target_ranges.py；用户 2026-09-16 定：能力与原版一致，转角可不同）："
                  f"{d['why']}。碰撞落在本区间内 = BLOCK（件的问题）；区间外但仍在上游 range 内 = WARN（工作空间比原版窄，"
                  f"但原版任务用不到）。in-scope modes：{', '.join(meta['modes'])}")
        src = (f"derived {today}：{meta['source']}（key {meta['key']}，各 mode 实际角 "
               + "；".join(f"{m} [{a:.1f},{b:.1f}]" for m, (a, b) in sorted(d["modes"].items()))
               + f"）{'，左右镜像求并' if meta['mirror'] else ''}± {meta['margin']:g}° → 夹到上游 range；上游出处见 src_note")
        new_block = ("    target_range_deg:\n"
                     f"      v: [{lo:.1f}, {hi:.1f}]\n"
                     f'      date: "{today}"\n'
                     f"      reason: {json.dumps(reason, ensure_ascii=False)}\n"
                     f"      src: {json.dumps(src, ensure_ascii=False)}\n"
                     + note.group(0))
        if old_v and old_v.group(1).replace(" ", "") != f"{lo:.1f},{hi:.1f}":
            changed.append(f"{nm} [{old_v.group(1)}] → [{lo:.1f}, {hi:.1f}]")
        elif not old_v:
            changed.append(f"{nm} (无旧值) → [{lo:.1f}, {hi:.1f}]")
        text = text[:s] + new_block + text[e:]
    return text, changed


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--check", action="store_true", help="只核对 frozen.yaml 当前值是否等于推导值，不写")
    args = ap.parse_args()
    text = FROZEN.read_text(encoding="utf-8")
    frozen = yaml.safe_load(text)
    derived, meta = derive(frozen)
    print(f"包络 {meta['source']}  modes {meta['modes']}  margin {meta['margin']}°  key {meta['key']}")
    bad = 0
    for nm, d in derived.items():
        cur = next(x for x in frozen["joint_axes"] if x["name"] == nm).get("target_range_deg", {}).get("v")
        same = cur is not None and abs(float(cur[0]) - d["v"][0]) < 0.05 and abs(float(cur[1]) - d["v"][1]) < 0.05
        bad += 0 if same else 1
        print(f"  {nm:16s} 推导 [{d['v'][0]:6.1f}, {d['v'][1]:6.1f}]  上游 [{d['upstream'][0]:6.1f}, {d['upstream'][1]:6.1f}]"
              f"  当前 {cur}  {'OK' if same else 'DIFF'}   {d['why']}")
    if args.check:
        print(f"{bad} 条与推导值不同")
        return 1 if bad else 0
    new_text, changed = write_back(text, derived, meta, date.today().isoformat())
    yaml.safe_load(new_text)                       # 写回前先证明还能解析
    FROZEN.write_text(new_text, encoding="utf-8")
    print(f"写回 {FROZEN.relative_to(ROOT)}：{len(derived)} 条 target_range_deg，其中值变化 {len(changed)} 条：")
    for c in changed:
        print("   ", c)
    return 0


if __name__ == "__main__":
    sys.exit(main())
