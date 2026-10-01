#!/usr/bin/env python3
"""09-13『有则改之』：M2 过孔 2.2→2.4、M2 头窝/垫管孔 4.2→4.4、N02 共用沉坑 13.8→14.0 之后，把 features.yaml 里按旧值写死的声明改到新值。

    ./.venv/bin/python tools/gate/retarget_hole_d.py [--apply]

不给 --apply 只打印会改哪些行。逐行文本改，注释/顺序/其余字段一字不动（yaml dump 会吃掉注释，所以不用它）。
只改 `nominal_d_mm:` 紧跟的那一行 `v: <旧值>`，而且要满足记录的 kind / 所在子块：
  · kind ∈ {screw_hole, horn_hole}，且 nominal_d_mm 直接挂在 geom / instances[i] 下（不在 counterbore: 子块里）：2.2 → 2.4
  · 任何记录里 counterbore: 子块下的 nominal_d_mm：4.2 → 4.4
  · kind ∈ {counterbore, spot_face} 的 nominal_d_mm：4.2 → 4.4；13.8 → 14.0（N02-F06/F12 共用沉坑 = 2·horn_r + m2_cbore_d + 0.1）
不碰：pilot_hole（H03-F02 的 2.2 是旧自攻底孔，另按 09-12 调查改 1.7）、key 的 depth、idler 侧 Ø2.6、spec_verbatim 文字（历史记录）。
改完必须重跑 `python -m duckstructure.build` 和 `tools/gate/gate.py --layers 2`：L2 会拿 STL 实际孔径和这里的声明对账。
源码常量：s288.py:horn_hole_d / mnt_hole_d = 2.4；lib.py:P.m2_cbore_d / P.tube_hole_d = 4.4；neck.py:NP_CB_D() = 14.0。"""
import argparse, re, sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
FILE = HERE / "data" / "features.yaml"
RULES = {  # (旧值 → 新值) 按上下文
    "through": (2.2, 2.4),
    "cbore": (4.2, 4.4),
    "common_cbore": (13.8, 14.0),
}
SKIP_IDS = {"T01-F19"}   # 扎带孔（XT30 绑带），不是螺丝孔：CAD 没改，2.5 mm 扎带本来就过不了 Ø2.2/2.4，另议
ID_RE = re.compile(r"^- id: (\S+)\s*$")
KIND_RE = re.compile(r"^  kind: (\S+)\s*$")
KEY_RE = re.compile(r"^(?P<ind>\s*)(?:- )?(?P<key>[A-Za-z_][A-Za-z0-9_]*):\s*$")
V_RE = re.compile(r"^(?P<pre>\s*v: )(?P<v>-?\d+(?:\.\d+)?)\s*$")


def indent(s):
    return len(s) - len(s.lstrip(" "))


def process(apply):
    lines = FILE.read_text(encoding="utf-8").split("\n")
    fid = kind = None
    stack = []          # [(indent, key)] 当前映射路径
    changed = []
    i = 0
    while i < len(lines):
        ln = lines[i]
        m = ID_RE.match(ln)
        if m:
            fid, kind, stack = m.group(1), None, []
            i += 1
            continue
        m = KIND_RE.match(ln)
        if m and fid:
            kind = m.group(1)
        m = KEY_RE.match(ln)
        if m and fid:
            ind = indent(ln) + (2 if ln.lstrip().startswith("- ") else 0)
            while stack and stack[-1][0] >= ind:
                stack.pop()
            stack.append((ind, m.group("key")))
            if m.group("key") == "nominal_d_mm" and i + 1 < len(lines):
                mv = V_RE.match(lines[i + 1])
                if mv:
                    v = float(mv.group("v"))
                    keys = [k for _, k in stack]
                    in_cbore = "counterbore" in keys[:-1]
                    new = None
                    if in_cbore and v == RULES["cbore"][0]:
                        new = RULES["cbore"][1]
                    elif kind in ("screw_hole", "horn_hole") and not in_cbore and v == RULES["through"][0]:
                        new = RULES["through"][1]
                    elif kind in ("counterbore", "spot_face") and v == RULES["cbore"][0]:
                        new = RULES["cbore"][1]
                    elif kind in ("counterbore", "spot_face") and v == RULES["common_cbore"][0]:
                        new = RULES["common_cbore"][1]
                    if fid in SKIP_IDS:
                        new = None
                    if new is not None:
                        changed.append((i + 2, fid, kind, "/".join(keys), v, new))
                        lines[i + 1] = f"{mv.group('pre')}{new}"
        i += 1
    for (lineno, f, k, path, old, new) in changed:
        print(f"  L{lineno:5} {f:10} {k:12} {path:40} {old} → {new}")
    print(f"{len(changed)} 处")
    if apply and changed:
        hdr = ("# 2026-09-13 retarget_hole_d.py：M2 过孔 2.2→2.4、头窝/垫管孔 4.2→4.4、N02 共用沉坑 13.8→14.0"
               f"（{len(changed)} 处 nominal_d_mm；spec_verbatim 文字未动，读数以 v 为准）。理由见 docs/参考对比_2026-09-12_AI-FanGe复刻.md §5-C。\n")
        FILE.write_text(hdr + "\n".join(lines), encoding="utf-8")
        print("已写入", FILE)
    return changed


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--apply", action="store_true")
    a = ap.parse_args()
    process(a.apply)
