#!/usr/bin/env python3
"""分度圆定案后，把 Gate 数据里所有按旧 horn_r 写死的 6 孔环坐标改到新值。

    ./.venv/bin/python tools/gate/retarget_horn_r.py --old 4.75 --new 5.25 [--apply]

不给 --apply 只打印会改哪些行。只改**数值**：
  · `holes:` / `hole_positions_mm:` 里 6 个点、且在某个坐标平面上以某中心为圆心、半径 = old 的环 → 每个点相对圆心按 new/old 缩放；
    **两种写法都认**：行内 `[[x,y,z], ...]`，以及 YAML 块式（`- - 4.75` / `  - 0.0` / `  - 15.35` 三行一点）。
    2026-09-14 前只认行内式 —— 而 features.yaml 里 33 处 hole_positions_mm **全是块式**，
    所以 `--old 4.75 --new 5.25` 只找得到 12 处 pitch_r_mm、一个环都找不到；照那样 apply 会让
    分度圆声明 5.25 而坐标还留在 4.75，自相矛盾（本次修复的就是这个）。
    块式改写**只重写数值变了的那一行**，常量轴那行原样保留，缩进/行序不动。
  · `pitch_r_mm:` 下一行 `v: old` → `v: new`。
文字说明（src_note 里的"4.75"、"9.5"）不动 —— 那是历史记录，改了反而说不清来源。
改完必须重跑 `python -m duckstructure.build` 和 `tools/gate/gate.py --layers 2`：L2 会拿 STL 实际孔阵和 pitch_r_mm 对账。

为什么不是让 Gate 从 s288.py 读：Gate 数据是"独立于源码的声明值"，L2 的意义就是拿声明值对 STL。
源码改了、数据不改，L2 会亮红 —— 那是它该亮的。本脚本只是把改 17 处的手工活自动化。"""
import argparse, json, math, re, sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
FILES = [HERE / "data" / "fasteners.yaml", HERE / "data" / "features.yaml"]
NUM = r"-?\d+(?:\.\d+)?"
RING = re.compile(r"^(?P<pre>\s*(?:-\s*)?(?:holes|hole_positions_mm):\s*)(?P<body>\[\[.*\]\])\s*$")
BLOCK_KEY = re.compile(r"^\s*(?:-\s+)?(?:holes|hole_positions_mm):\s*$")
PITCH = re.compile(r"^(?P<pre>\s*v:\s*)(?P<v>" + NUM + r")\s*$")

def retarget_ring(pts, old, new, tol=0.02):
    """pts: 6×3。找一个常量坐标轴 + 圆心，使 6 点到圆心的距离全等于 old；找到就缩放，否则返回 None。"""
    if len(pts) != 6: return None
    for ax in range(3):
        col = [p[ax] for p in pts]
        if max(col) - min(col) > 1e-6: continue
        uv = [i for i in range(3) if i != ax]
        cu = sum(p[uv[0]] for p in pts) / 6; cv = sum(p[uv[1]] for p in pts) / 6
        rs = [math.hypot(p[uv[0]] - cu, p[uv[1]] - cv) for p in pts]
        if all(abs(r - old) <= tol for r in rs):
            k = new / old
            out = []
            for p in pts:
                q = list(p)
                q[uv[0]] = round(cu + (p[uv[0]] - cu) * k, 4)
                q[uv[1]] = round(cv + (p[uv[1]] - cv) * k, 4)
                out.append(q)
            return out
    return None

def _indent(s):
    return len(s) - len(s.lstrip(" "))

def parse_block(lines, i):
    """i = `key:` 所在行（0 基），其后是 YAML 块式点列：
           key:
           - - 4.75
             - 0.0
             - 15.35
    返回 (pts, spans, end)：pts = [[x,y,z]...]，spans = 每个点每个分量所在的行号（0 基），
    end = 块结束后的第一行。不是块式点列则 (None, None, i+1)。"""
    ind = _indent(lines[i])
    pts, spans, cur, cur_ln = [], [], None, None
    j = i + 1
    while j < len(lines):
        ln = lines[j]
        if not ln.strip():
            j += 1; continue
        ii, t = _indent(ln), ln.strip()
        if ii < ind: break
        if ii == ind and not t.startswith("- "): break
        if t.startswith("- - "):
            if cur is not None: pts.append(cur); spans.append(cur_ln)
            try: cur = [float(t[4:])]
            except ValueError: return None, None, j
            cur_ln = [j]
        elif t.startswith("- ") and cur is not None and ii > ind:
            try: cur.append(float(t[2:]))
            except ValueError: return None, None, j
            cur_ln.append(j)
        else:
            return None, None, j
        j += 1
    if cur is not None: pts.append(cur); spans.append(cur_ln)
    if not pts or any(len(p) != 3 for p in pts): return None, None, j
    return pts, spans, j

def rewrite_block(lines, spans, old_pts, new_pts):
    """只重写**数值变了**的那一行，保留原缩进与 `- ` / `- - ` 前缀。"""
    for ln_idx, o, n in zip(spans, old_pts, new_pts):
        for k in range(3):
            if abs(o[k] - n[k]) < 5e-5: continue          # 常量轴 / 未变的分量：原样不动
            src = lines[ln_idx[k]]
            pre = src[:len(src) - len(src.lstrip(" "))]
            body = src.strip()
            mark = "- - " if body.startswith("- - ") else "- "
            lines[ln_idx[k]] = pre + mark + fmt_num(n[k])

def fmt_num(x):
    s = f"{x:.4f}".rstrip("0").rstrip(".")
    if s in ("-0", ""): s = "0.0"
    if "." not in s: s += ".0"
    return s

def fmt_ring(pts):
    f = fmt_num
    return "[" + ", ".join("[" + ", ".join(f(c) for c in p) + "]" for p in pts) + "]"

def process(path, old, new, apply):
    lines = path.read_text(encoding="utf-8").split("\n")
    changed = []
    i = 0
    while i < len(lines):
        ln = lines[i]
        mb = BLOCK_KEY.match(ln)
        if mb:
            pts, spans, end = parse_block(lines, i)
            if pts:
                out = retarget_ring(pts, old, new)
                if out:
                    rewrite_block(lines, spans, pts, out)
                    changed.append((i + 1, f"ring(块式, {len(pts)} 点)"))
                i = end
                continue
            i = end
            continue
        m = RING.match(ln)
        if m:
            try: pts = json.loads(m.group("body"))
            except json.JSONDecodeError: pts = None
            if pts:
                out = retarget_ring(pts, old, new)
                if out:
                    lines[i] = m.group("pre") + fmt_ring(out)
                    changed.append((i + 1, "ring"))
        elif ln.strip() == "pitch_r_mm:" and i + 1 < len(lines):
            m2 = PITCH.match(lines[i + 1])
            if m2 and abs(float(m2.group("v")) - old) <= 0.02:
                lines[i + 1] = m2.group("pre") + f"{new:g}"
                changed.append((i + 2, "pitch_r_mm"))
        i += 1
    for (n, kind) in changed:
        print(f"  {path.name}:{n}  {kind}")
    if apply and changed:
        path.write_text("\n".join(lines), encoding="utf-8")
    return len(changed)

if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--old", type=float, required=True)
    ap.add_argument("--new", type=float, required=True)
    ap.add_argument("--apply", action="store_true")
    a = ap.parse_args()
    total = sum(process(p, a.old, a.new, a.apply) for p in FILES)
    print(f"{'已改' if a.apply else '将改'} {total} 处（--old {a.old} → --new {a.new}）")
    if not a.apply: print("（加 --apply 才真的写文件）")
